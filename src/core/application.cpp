#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/performance_hud.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/weather.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/loading_screen.hpp"
#include <imgui.h>
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "ui/ui_manager.hpp"
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "game/world.hpp"
#include "game/npc_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <chrono>
#include <cstdlib>
#include <set>

namespace wowee {
namespace core {

Application* Application::instance = nullptr;

Application::Application() {
    instance = this;
}

Application::~Application() {
    shutdown();
    instance = nullptr;
}

bool Application::initialize() {
    LOG_INFO("Initializing Wowee Native Client");

    // Create window
    WindowConfig windowConfig;
    windowConfig.title = "Wowee";
    windowConfig.width = 1280;
    windowConfig.height = 720;
    windowConfig.vsync = false;

    window = std::make_unique<Window>(windowConfig);
    if (!window->initialize()) {
        LOG_FATAL("Failed to initialize window");
        return false;
    }

    // Create renderer
    renderer = std::make_unique<rendering::Renderer>();
    if (!renderer->initialize(window.get())) {
        LOG_FATAL("Failed to initialize renderer");
        return false;
    }

    // Create UI manager
    uiManager = std::make_unique<ui::UIManager>();
    if (!uiManager->initialize(window.get())) {
        LOG_FATAL("Failed to initialize UI manager");
        return false;
    }

    // Create subsystems
    authHandler = std::make_unique<auth::AuthHandler>();
    gameHandler = std::make_unique<game::GameHandler>();
    world = std::make_unique<game::World>();

    // Create asset manager
    assetManager = std::make_unique<pipeline::AssetManager>();

    // Try to get WoW data path from environment variable
    const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
    std::string dataPath = dataPathEnv ? dataPathEnv : "./Data";

    LOG_INFO("Attempting to load WoW assets from: ", dataPath);
    if (assetManager->initialize(dataPath)) {
        LOG_INFO("Asset manager initialized successfully");
    } else {
        LOG_WARNING("Failed to initialize asset manager - asset loading will be unavailable");
        LOG_WARNING("Set WOW_DATA_PATH environment variable to your WoW Data directory");
    }

    // Set up UI callbacks
    setupUICallbacks();

    LOG_INFO("Application initialized successfully");
    running = true;
    return true;
}

void Application::run() {
    LOG_INFO("Starting main loop");

    // Show loading screen while loading initial data
    rendering::LoadingScreen loadingScreen;
    if (loadingScreen.initialize()) {
        // Render loading screen
        loadingScreen.setStatus("Initializing...");
        loadingScreen.render();
        window->swapBuffers();

        // Load terrain data
        if (assetManager && assetManager->isInitialized() && renderer) {
            loadingScreen.setStatus("Loading terrain...");
            loadingScreen.render();
            window->swapBuffers();

            renderer->loadTestTerrain(assetManager.get(), "World\\Maps\\Azeroth\\Azeroth_32_49.adt");

            loadingScreen.setStatus("Spawning character...");
            loadingScreen.render();
            window->swapBuffers();

            // Spawn player character with third-person camera
            spawnPlayerCharacter();
        }

        loadingScreen.setStatus("Ready!");
        loadingScreen.render();
        window->swapBuffers();
        SDL_Delay(500);  // Brief pause to show "Ready!"

        loadingScreen.shutdown();
    } else {
        // Fallback: load without loading screen
        if (assetManager && assetManager->isInitialized() && renderer) {
            renderer->loadTestTerrain(assetManager.get(), "World\\Maps\\Azeroth\\Azeroth_32_49.adt");
            spawnPlayerCharacter();
        }
    }

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (running && !window->shouldClose()) {
        // Calculate delta time
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> deltaTimeDuration = currentTime - lastTime;
        float deltaTime = deltaTimeDuration.count();
        lastTime = currentTime;

        // Cap delta time to prevent large jumps
        if (deltaTime > 0.1f) {
            deltaTime = 0.1f;
        }

        // Poll events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            // Pass event to UI manager first
            if (uiManager) {
                uiManager->processEvent(event);
            }

            // Pass mouse events to camera controller (skip when UI has mouse focus)
            if (renderer && renderer->getCameraController() && !ImGui::GetIO().WantCaptureMouse) {
                if (event.type == SDL_MOUSEMOTION) {
                    renderer->getCameraController()->processMouseMotion(event.motion);
                }
                else if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP) {
                    renderer->getCameraController()->processMouseButton(event.button);
                }
                else if (event.type == SDL_MOUSEWHEEL) {
                    renderer->getCameraController()->processMouseWheel(static_cast<float>(event.wheel.y));
                }
            }

            // Handle window events
            if (event.type == SDL_QUIT) {
                window->setShouldClose(true);
            }
            else if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int newWidth = event.window.data1;
                    int newHeight = event.window.data2;
                    glViewport(0, 0, newWidth, newHeight);
                    if (renderer && renderer->getCamera()) {
                        renderer->getCamera()->setAspectRatio(static_cast<float>(newWidth) / newHeight);
                    }
                }
            }
            // Debug controls
            else if (event.type == SDL_KEYDOWN) {
                // Skip non-function-key input when UI (chat) has keyboard focus
                bool uiHasKeyboard = ImGui::GetIO().WantCaptureKeyboard;
                auto sc = event.key.keysym.scancode;
                bool isFKey = (sc >= SDL_SCANCODE_F1 && sc <= SDL_SCANCODE_F12);
                if (uiHasKeyboard && !isFKey) {
                    continue;  // Let ImGui handle the keystroke
                }

                // F1: Toggle performance HUD
                if (event.key.keysym.scancode == SDL_SCANCODE_F1) {
                    if (renderer && renderer->getPerformanceHUD()) {
                        renderer->getPerformanceHUD()->toggle();
                        bool enabled = renderer->getPerformanceHUD()->isEnabled();
                        LOG_INFO("Performance HUD: ", enabled ? "ON" : "OFF");
                    }
                }
                // F2: Toggle wireframe
                else if (event.key.keysym.scancode == SDL_SCANCODE_F2) {
                    static bool wireframe = false;
                    wireframe = !wireframe;
                    if (renderer) {
                        renderer->setWireframeMode(wireframe);
                        LOG_INFO("Wireframe mode: ", wireframe ? "ON" : "OFF");
                    }
                }
                // F3: Load test terrain (if in main menu/auth state)
                else if (event.key.keysym.scancode == SDL_SCANCODE_F3) {
                    if (assetManager && assetManager->isInitialized()) {
                        LOG_INFO("Loading test terrain...");
                        // Load a test ADT tile (Elwynn Forest)
                        if (renderer->loadTestTerrain(assetManager.get(),
                            "World\\Maps\\Azeroth\\Azeroth_32_49.adt")) {
                            LOG_INFO("Test terrain loaded! Use WASD/QE to move, hold right mouse to look");
                        }
                    } else {
                        LOG_WARNING("Asset manager not initialized. Set WOW_DATA_PATH environment variable.");
                    }
                }
                // F4: Toggle frustum culling
                else if (event.key.keysym.scancode == SDL_SCANCODE_F4) {
                    if (renderer && renderer->getTerrainRenderer()) {
                        static bool culling = true;
                        culling = !culling;
                        renderer->getTerrainRenderer()->setFrustumCulling(culling);
                        LOG_INFO("Frustum culling: ", culling ? "ON" : "OFF");
                    }
                }
                // F5: Show rendering statistics
                else if (event.key.keysym.scancode == SDL_SCANCODE_F5) {
                    if (renderer && renderer->getTerrainRenderer()) {
                        auto* terrain = renderer->getTerrainRenderer();
                        LOG_INFO("=== Rendering Statistics ===");
                        LOG_INFO("  Total chunks: ", terrain->getChunkCount());
                        LOG_INFO("  Rendered: ", terrain->getRenderedChunkCount());
                        LOG_INFO("  Culled: ", terrain->getCulledChunkCount());
                        LOG_INFO("  Triangles: ", terrain->getTriangleCount());

                        if (terrain->getChunkCount() > 0) {
                            float visiblePercent = (terrain->getRenderedChunkCount() * 100.0f) / terrain->getChunkCount();
                            LOG_INFO("  Visible: ", static_cast<int>(visiblePercent), "%");
                        }

                        // Show terrain manager stats
                        if (renderer->getTerrainManager()) {
                            auto* manager = renderer->getTerrainManager();
                            LOG_INFO("  Loaded tiles: ", manager->getLoadedTileCount());
                            auto currentTile = manager->getCurrentTile();
                            LOG_INFO("  Current tile: [", currentTile.x, ",", currentTile.y, "]");
                        }
                    }
                }
                // F6: Load multi-tile terrain area
                else if (event.key.keysym.scancode == SDL_SCANCODE_F6) {
                    if (assetManager && assetManager->isInitialized()) {
                        LOG_INFO("Loading 3x3 terrain area (Elwynn Forest)...");
                        // Load 3x3 grid of tiles (Elwynn Forest area)
                        if (renderer->loadTerrainArea("Azeroth", 32, 49, 1)) {
                            LOG_INFO("Terrain area loaded! Streaming enabled.");
                            LOG_INFO("Move around to see dynamic tile loading/unloading");
                        }
                    } else {
                        LOG_WARNING("Asset manager not initialized. Set WOW_DATA_PATH environment variable.");
                    }
                }
                // F7: Toggle terrain streaming
                else if (event.key.keysym.scancode == SDL_SCANCODE_F7) {
                    if (renderer && renderer->getTerrainManager()) {
                        static bool streaming = true;
                        streaming = !streaming;
                        renderer->setTerrainStreaming(streaming);
                    }
                }
                // F8: Toggle water rendering
                else if (event.key.keysym.scancode == SDL_SCANCODE_F8) {
                    if (renderer && renderer->getWaterRenderer()) {
                        static bool water = true;
                        water = !water;
                        renderer->getWaterRenderer()->setEnabled(water);
                        LOG_INFO("Water rendering: ", water ? "ON" : "OFF");
                    }
                }
                // F9: Toggle time progression
                else if (event.key.keysym.scancode == SDL_SCANCODE_F9) {
                    if (renderer && renderer->getSkybox()) {
                        bool progression = !renderer->getSkybox()->isTimeProgressionEnabled();
                        renderer->getSkybox()->setTimeProgression(progression);
                        LOG_INFO("Time progression: ", progression ? "ON" : "OFF");
                    }
                }
                // Plus/Equals: Advance time
                else if (event.key.keysym.scancode == SDL_SCANCODE_EQUALS ||
                         event.key.keysym.scancode == SDL_SCANCODE_KP_PLUS) {
                    if (renderer && renderer->getSkybox()) {
                        float time = renderer->getSkybox()->getTimeOfDay() + 1.0f;
                        renderer->getSkybox()->setTimeOfDay(time);
                        LOG_INFO("Time of day: ", static_cast<int>(time), ":00");
                    }
                }
                // Minus: Rewind time
                else if (event.key.keysym.scancode == SDL_SCANCODE_MINUS ||
                         event.key.keysym.scancode == SDL_SCANCODE_KP_MINUS) {
                    if (renderer && renderer->getSkybox()) {
                        float time = renderer->getSkybox()->getTimeOfDay() - 1.0f;
                        renderer->getSkybox()->setTimeOfDay(time);
                        LOG_INFO("Time of day: ", static_cast<int>(time), ":00");
                    }
                }
                // F10: Toggle celestial rendering (sun/moon)
                else if (event.key.keysym.scancode == SDL_SCANCODE_F10) {
                    if (renderer && renderer->getCelestial()) {
                        bool enabled = !renderer->getCelestial()->isEnabled();
                        renderer->getCelestial()->setEnabled(enabled);
                        LOG_INFO("Celestial rendering: ", enabled ? "ON" : "OFF");
                    }
                }
                // F11: Toggle star field
                else if (event.key.keysym.scancode == SDL_SCANCODE_F11) {
                    if (renderer && renderer->getStarField()) {
                        bool enabled = !renderer->getStarField()->isEnabled();
                        renderer->getStarField()->setEnabled(enabled);
                        LOG_INFO("Star field: ", enabled ? "ON" : "OFF");
                    }
                }
                // F12: Toggle distance fog
                else if (event.key.keysym.scancode == SDL_SCANCODE_F12) {
                    if (renderer && renderer->getTerrainRenderer()) {
                        bool enabled = !renderer->getTerrainRenderer()->isFogEnabled();
                        renderer->getTerrainRenderer()->setFogEnabled(enabled);
                        LOG_INFO("Distance fog: ", enabled ? "ON" : "OFF");
                    }
                }
                // C: Toggle clouds
                else if (event.key.keysym.scancode == SDL_SCANCODE_C) {
                    if (renderer && renderer->getClouds()) {
                        bool enabled = !renderer->getClouds()->isEnabled();
                        renderer->getClouds()->setEnabled(enabled);
                        LOG_INFO("Clouds: ", enabled ? "ON" : "OFF");
                    }
                }
                // [ (Left bracket): Decrease cloud density
                else if (event.key.keysym.scancode == SDL_SCANCODE_LEFTBRACKET) {
                    if (renderer && renderer->getClouds()) {
                        float density = renderer->getClouds()->getDensity() - 0.1f;
                        renderer->getClouds()->setDensity(density);
                        LOG_INFO("Cloud density: ", static_cast<int>(density * 100), "%");
                    }
                }
                // ] (Right bracket): Increase cloud density
                else if (event.key.keysym.scancode == SDL_SCANCODE_RIGHTBRACKET) {
                    if (renderer && renderer->getClouds()) {
                        float density = renderer->getClouds()->getDensity() + 0.1f;
                        renderer->getClouds()->setDensity(density);
                        LOG_INFO("Cloud density: ", static_cast<int>(density * 100), "%");
                    }
                }
                // L: Toggle lens flare
                else if (event.key.keysym.scancode == SDL_SCANCODE_L) {
                    if (renderer && renderer->getLensFlare()) {
                        bool enabled = !renderer->getLensFlare()->isEnabled();
                        renderer->getLensFlare()->setEnabled(enabled);
                        LOG_INFO("Lens flare: ", enabled ? "ON" : "OFF");
                    }
                }
                // , (Comma): Decrease lens flare intensity
                else if (event.key.keysym.scancode == SDL_SCANCODE_COMMA) {
                    if (renderer && renderer->getLensFlare()) {
                        float intensity = renderer->getLensFlare()->getIntensity() - 0.1f;
                        renderer->getLensFlare()->setIntensity(intensity);
                        LOG_INFO("Lens flare intensity: ", static_cast<int>(intensity * 100), "%");
                    }
                }
                // . (Period): Increase lens flare intensity
                else if (event.key.keysym.scancode == SDL_SCANCODE_PERIOD) {
                    if (renderer && renderer->getLensFlare()) {
                        float intensity = renderer->getLensFlare()->getIntensity() + 0.1f;
                        renderer->getLensFlare()->setIntensity(intensity);
                        LOG_INFO("Lens flare intensity: ", static_cast<int>(intensity * 100), "%");
                    }
                }
                // M: Toggle moon phase cycling
                else if (event.key.keysym.scancode == SDL_SCANCODE_M) {
                    if (renderer && renderer->getCelestial()) {
                        bool cycling = !renderer->getCelestial()->isMoonPhaseCycling();
                        renderer->getCelestial()->setMoonPhaseCycling(cycling);
                        LOG_INFO("Moon phase cycling: ", cycling ? "ON" : "OFF");
                    }
                }
                // ; (Semicolon): Previous moon phase
                else if (event.key.keysym.scancode == SDL_SCANCODE_SEMICOLON) {
                    if (renderer && renderer->getCelestial()) {
                        float phase = renderer->getCelestial()->getMoonPhase() - 0.05f;
                        if (phase < 0.0f) phase += 1.0f;
                        renderer->getCelestial()->setMoonPhase(phase);

                        // Log phase name
                        const char* phaseName = "Unknown";
                        if (phase < 0.0625f || phase >= 0.9375f) phaseName = "New Moon";
                        else if (phase < 0.1875f) phaseName = "Waxing Crescent";
                        else if (phase < 0.3125f) phaseName = "First Quarter";
                        else if (phase < 0.4375f) phaseName = "Waxing Gibbous";
                        else if (phase < 0.5625f) phaseName = "Full Moon";
                        else if (phase < 0.6875f) phaseName = "Waning Gibbous";
                        else if (phase < 0.8125f) phaseName = "Last Quarter";
                        else phaseName = "Waning Crescent";

                        LOG_INFO("Moon phase: ", phaseName, " (", static_cast<int>(phase * 100), "%)");
                    }
                }
                // ' (Apostrophe): Next moon phase
                else if (event.key.keysym.scancode == SDL_SCANCODE_APOSTROPHE) {
                    if (renderer && renderer->getCelestial()) {
                        float phase = renderer->getCelestial()->getMoonPhase() + 0.05f;
                        if (phase >= 1.0f) phase -= 1.0f;
                        renderer->getCelestial()->setMoonPhase(phase);

                        // Log phase name
                        const char* phaseName = "Unknown";
                        if (phase < 0.0625f || phase >= 0.9375f) phaseName = "New Moon";
                        else if (phase < 0.1875f) phaseName = "Waxing Crescent";
                        else if (phase < 0.3125f) phaseName = "First Quarter";
                        else if (phase < 0.4375f) phaseName = "Waxing Gibbous";
                        else if (phase < 0.5625f) phaseName = "Full Moon";
                        else if (phase < 0.6875f) phaseName = "Waning Gibbous";
                        else if (phase < 0.8125f) phaseName = "Last Quarter";
                        else phaseName = "Waning Crescent";

                        LOG_INFO("Moon phase: ", phaseName, " (", static_cast<int>(phase * 100), "%)");
                    }
                }
                // X key reserved for sit (handled in camera_controller)
                // < (Shift+,): Decrease weather intensity
                else if (event.key.keysym.scancode == SDL_SCANCODE_COMMA &&
                         (event.key.keysym.mod & KMOD_SHIFT)) {
                    if (renderer && renderer->getWeather()) {
                        float intensity = renderer->getWeather()->getIntensity() - 0.1f;
                        renderer->getWeather()->setIntensity(intensity);
                        LOG_INFO("Weather intensity: ", static_cast<int>(intensity * 100), "%");
                    }
                }
                // > (Shift+.): Increase weather intensity
                else if (event.key.keysym.scancode == SDL_SCANCODE_PERIOD &&
                         (event.key.keysym.mod & KMOD_SHIFT)) {
                    if (renderer && renderer->getWeather()) {
                        float intensity = renderer->getWeather()->getIntensity() + 0.1f;
                        renderer->getWeather()->setIntensity(intensity);
                        LOG_INFO("Weather intensity: ", static_cast<int>(intensity * 100), "%");
                    }
                }
                // K: Spawn player character at camera position
                else if (event.key.keysym.scancode == SDL_SCANCODE_K) {
                    spawnPlayerCharacter();
                }
                // J: Remove all characters
                else if (event.key.keysym.scancode == SDL_SCANCODE_J) {
                    if (renderer && renderer->getCharacterRenderer()) {
                        // Note: CharacterRenderer doesn't have removeAll(), so we'd need to track IDs
                        // For now, just log
                        LOG_INFO("Character removal not yet implemented");
                    }
                }
                // N: Toggle minimap
                else if (event.key.keysym.scancode == SDL_SCANCODE_N) {
                    if (renderer && renderer->getMinimap()) {
                        renderer->getMinimap()->toggle();
                        LOG_INFO("Minimap ", renderer->getMinimap()->isEnabled() ? "enabled" : "disabled");
                    }
                }
                // P: Remove all WMO buildings (O key removed)
                else if (event.key.keysym.scancode == SDL_SCANCODE_P) {
                    if (renderer && renderer->getWMORenderer()) {
                        renderer->getWMORenderer()->clearInstances();
                        LOG_INFO("Cleared all WMO instances");
                    }
                }
            }
        }

        // Update input
        Input::getInstance().update();

        // Update application state
        update(deltaTime);

        // Render
        render();

        // Swap buffers
        window->swapBuffers();
    }

    LOG_INFO("Main loop ended");
}

void Application::shutdown() {
    LOG_INFO("Shutting down application");

    world.reset();
    gameHandler.reset();
    authHandler.reset();
    assetManager.reset();
    uiManager.reset();
    renderer.reset();
    window.reset();

    running = false;
    LOG_INFO("Application shutdown complete");
}

void Application::setState(AppState newState) {
    if (state == newState) {
        return;
    }

    LOG_INFO("State transition: ", static_cast<int>(state), " -> ", static_cast<int>(newState));
    state = newState;

    // Handle state transitions
    switch (newState) {
        case AppState::AUTHENTICATION:
            // Show auth screen
            break;
        case AppState::REALM_SELECTION:
            // Show realm screen
            break;
        case AppState::CHARACTER_SELECTION:
            // Show character screen
            break;
        case AppState::IN_GAME:
            // Wire up movement opcodes from camera controller
            if (renderer && renderer->getCameraController()) {
                auto* cc = renderer->getCameraController();
                cc->setMovementCallback([this](uint32_t opcode) {
                    if (gameHandler && !singlePlayerMode) {
                        gameHandler->sendMovement(static_cast<game::Opcode>(opcode));
                    }
                });
                // Use WoW-correct speeds when connected to a server
                cc->setUseWoWSpeed(!singlePlayerMode);
            }
            break;
        case AppState::DISCONNECTED:
            // Back to auth
            break;
    }
}

void Application::update(float deltaTime) {
    // Update based on current state
    switch (state) {
        case AppState::AUTHENTICATION:
            if (authHandler) {
                authHandler->update(deltaTime);
            }
            break;

        case AppState::REALM_SELECTION:
            // Realm selection update
            break;

        case AppState::CHARACTER_SELECTION:
            // Character selection update
            break;

        case AppState::IN_GAME:
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            if (world) {
                world->update(deltaTime);
            }
            // Spawn local NPCs only in single-player (no server to provide them)
            if (!npcsSpawned && singlePlayerMode) {
                spawnNpcs();
            }
            if (npcManager && renderer && renderer->getCharacterRenderer()) {
                npcManager->update(deltaTime, renderer->getCharacterRenderer());
            }

            // Sync character GL position → movementInfo WoW coords each frame
            if (renderer && gameHandler) {
                glm::vec3 glPos = renderer->getCharacterPosition();
                constexpr float ZEROPOINT = 32.0f * 533.33333f;
                float wowX = ZEROPOINT - glPos.y;
                float wowY = glPos.z;
                float wowZ = ZEROPOINT - glPos.x;
                gameHandler->setPosition(wowX, wowY, wowZ);

                // Sync orientation: camera yaw (degrees) → WoW orientation (radians)
                float yawDeg = renderer->getCharacterYaw();
                float wowOrientation = glm::radians(yawDeg - 90.0f);
                gameHandler->setOrientation(wowOrientation);
            }

            // Send movement heartbeat every 500ms while moving
            if (renderer && renderer->isMoving()) {
                movementHeartbeatTimer += deltaTime;
                if (movementHeartbeatTimer >= 0.5f) {
                    movementHeartbeatTimer = 0.0f;
                    if (gameHandler && !singlePlayerMode) {
                        gameHandler->sendMovement(game::Opcode::CMSG_MOVE_HEARTBEAT);
                    }
                }
            } else {
                movementHeartbeatTimer = 0.0f;
            }
            break;

        case AppState::DISCONNECTED:
            // Handle disconnection
            break;
    }

    // Update renderer (camera, etc.)
    if (renderer) {
        renderer->update(deltaTime);
    }

    // Update UI
    if (uiManager) {
        uiManager->update(deltaTime);
    }
}

void Application::render() {
    if (!renderer) {
        return;
    }

    renderer->beginFrame();

    // Always render atmospheric background (sky, stars, clouds, etc.)
    // This provides a nice animated background for login/UI screens
    // Terrain/characters only render if loaded (in-game)
    switch (state) {
        case AppState::IN_GAME:
            // Render full world with terrain and entities
            if (world) {
                renderer->renderWorld(world.get());
            } else {
                // Fallback: render just atmosphere if world not loaded
                renderer->renderWorld(nullptr);
            }
            break;

        default:
            // Login/UI screens: render atmospheric background only
            // (terrain/water/characters won't render as they're not loaded)
            renderer->renderWorld(nullptr);
            break;
    }

    // Render performance HUD (within ImGui frame, before UI ends the frame)
    if (renderer) {
        renderer->renderHUD();
    }

    // Render UI on top (ends ImGui frame with ImGui::Render())
    if (uiManager) {
        uiManager->render(state, authHandler.get(), gameHandler.get());
    }

    renderer->endFrame();
}

void Application::setupUICallbacks() {
    // Authentication screen callback
    uiManager->getAuthScreen().setOnSuccess([this]() {
        LOG_INFO("Authentication successful, transitioning to realm selection");
        setState(AppState::REALM_SELECTION);
    });

    // Single-player mode callback
    uiManager->getAuthScreen().setOnSinglePlayer([this]() {
        LOG_INFO("Single-player mode selected");
        startSinglePlayer();
    });

    // Realm selection callback
    uiManager->getRealmScreen().setOnRealmSelected([this](const std::string& realmName, const std::string& realmAddress) {
        LOG_INFO("Realm selected: ", realmName, " (", realmAddress, ")");

        // Parse realm address (format: "hostname:port")
        std::string host = realmAddress;
        uint16_t port = 8085;  // Default world server port

        size_t colonPos = realmAddress.find(':');
        if (colonPos != std::string::npos) {
            host = realmAddress.substr(0, colonPos);
            port = static_cast<uint16_t>(std::stoi(realmAddress.substr(colonPos + 1)));
        }

        // Connect to world server
        const auto& sessionKey = authHandler->getSessionKey();
        const std::string accountName = "TESTACCOUNT";  // TODO: Store from auth

        if (gameHandler->connect(host, port, sessionKey, accountName)) {
            LOG_INFO("Connected to world server, transitioning to character selection");
            setState(AppState::CHARACTER_SELECTION);
        } else {
            LOG_ERROR("Failed to connect to world server");
        }
    });

    // Character selection callback
    uiManager->getCharacterScreen().setOnCharacterSelected([this](uint64_t characterGuid) {
        LOG_INFO("Character selected: GUID=0x", std::hex, characterGuid, std::dec);
        setState(AppState::IN_GAME);
    });
}

void Application::spawnPlayerCharacter() {
    if (playerCharacterSpawned) return;
    if (!renderer || !renderer->getCharacterRenderer() || !renderer->getCamera()) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    auto* camera = renderer->getCamera();
    bool loaded = false;

    // Try loading real HumanMale M2 from MPQ
    if (assetManager && assetManager->isInitialized()) {
        std::string m2Path = "Character\\Human\\Male\\HumanMale.m2";
        auto m2Data = assetManager->readFile(m2Path);
        if (!m2Data.empty()) {
            auto model = pipeline::M2Loader::load(m2Data);

            // Load skin file for submesh/batch data
            std::string skinPath = "Character\\Human\\Male\\HumanMale00.skin";
            auto skinData = assetManager->readFile(skinPath);
            if (!skinData.empty()) {
                pipeline::M2Loader::loadSkin(skinData, model);
            }

            if (model.isValid()) {
                // Log texture slots
                for (size_t ti = 0; ti < model.textures.size(); ti++) {
                    auto& tex = model.textures[ti];
                    LOG_INFO("  Texture ", ti, ": type=", tex.type, " name='", tex.filename, "'");
                }

                // Look up underwear textures from CharSections.dbc
                std::string bodySkinPath = "Character\\Human\\Male\\HumanMaleSkin00_00.blp";
                std::string faceLowerTexturePath;
                std::vector<std::string> underwearPaths;

                auto charSectionsDbc = assetManager->loadDBC("CharSections.dbc");
                if (charSectionsDbc) {
                    LOG_INFO("CharSections.dbc loaded: ", charSectionsDbc->getRecordCount(), " records");
                    bool foundSkin = false;
                    bool foundUnderwear = false;
                    bool foundFaceLower = false;
                    for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                        uint32_t raceId = charSectionsDbc->getUInt32(r, 1);
                        uint32_t sexId = charSectionsDbc->getUInt32(r, 2);
                        uint32_t baseSection = charSectionsDbc->getUInt32(r, 3);
                        uint32_t variationIndex = charSectionsDbc->getUInt32(r, 8);
                        uint32_t colorIndex = charSectionsDbc->getUInt32(r, 9);

                        if (raceId != 1 || sexId != 0) continue;

                        if (baseSection == 0 && !foundSkin && variationIndex == 0 && colorIndex == 0) {
                            std::string tex1 = charSectionsDbc->getString(r, 4);
                            if (!tex1.empty()) {
                                bodySkinPath = tex1;
                                foundSkin = true;
                                LOG_INFO("  DBC body skin: ", bodySkinPath);
                            }
                        } else if (baseSection == 3 && colorIndex == 0) {
                            (void)variationIndex;
                        } else if (baseSection == 1 && !foundFaceLower && variationIndex == 0 && colorIndex == 0) {
                            std::string tex1 = charSectionsDbc->getString(r, 4);
                            if (!tex1.empty()) {
                                faceLowerTexturePath = tex1;
                                foundFaceLower = true;
                                LOG_INFO("  DBC face texture: ", faceLowerTexturePath);
                            }
                        } else if (baseSection == 4 && !foundUnderwear && variationIndex == 0 && colorIndex == 0) {
                            for (int f = 4; f <= 6; f++) {
                                std::string tex = charSectionsDbc->getString(r, f);
                                if (!tex.empty()) {
                                    underwearPaths.push_back(tex);
                                    LOG_INFO("  DBC underwear texture: ", tex);
                                }
                            }
                            foundUnderwear = true;
                        }
                    }
                } else {
                    LOG_WARNING("Failed to load CharSections.dbc, using hardcoded textures");
                }

                for (auto& tex : model.textures) {
                    if (tex.type == 1 && tex.filename.empty()) {
                        tex.filename = bodySkinPath;
                    } else if (tex.type == 6 && tex.filename.empty()) {
                        tex.filename = "Character\\Human\\Hair00_00.blp";
                    } else if (tex.type == 8 && tex.filename.empty()) {
                        if (!underwearPaths.empty()) {
                            tex.filename = underwearPaths[0];
                        } else {
                            tex.filename = "Character\\Human\\Male\\HumanMaleNakedPelvisSkin00_00.blp";
                        }
                    }
                }

                // Load external .anim files for sequences with external data.
                // Sequences WITH flag 0x20 have their animation data inline in the M2 file.
                // Sequences WITHOUT flag 0x20 store data in external .anim files.
                for (uint32_t si = 0; si < model.sequences.size(); si++) {
                    if (!(model.sequences[si].flags & 0x20)) {
                        // File naming: <ModelPath><AnimId>-<VariationIndex>.anim
                        // e.g. Character\Human\Male\HumanMale0097-00.anim
                        char animFileName[256];
                        snprintf(animFileName, sizeof(animFileName),
                            "Character\\Human\\Male\\HumanMale%04u-%02u.anim",
                            model.sequences[si].id, model.sequences[si].variationIndex);
                        auto animFileData = assetManager->readFile(animFileName);
                        if (!animFileData.empty()) {
                            pipeline::M2Loader::loadAnimFile(m2Data, animFileData, si, model);
                        }
                    }
                }

                charRenderer->loadModel(model, 1);

                // Save skin composite state for re-compositing on equipment changes
                bodySkinPath_ = bodySkinPath;
                underwearPaths_ = underwearPaths;


                // Composite body skin + underwear overlays
                if (!underwearPaths.empty()) {
                    std::vector<std::string> layers;
                    layers.push_back(bodySkinPath);
                    for (const auto& up : underwearPaths) {
                        layers.push_back(up);
                    }
                    GLuint compositeTex = charRenderer->compositeTextures(layers);
                    if (compositeTex != 0) {
                        for (size_t ti = 0; ti < model.textures.size(); ti++) {
                            if (model.textures[ti].type == 1) {
                                charRenderer->setModelTexture(1, static_cast<uint32_t>(ti), compositeTex);
                                skinTextureSlotIndex_ = static_cast<uint32_t>(ti);
                                LOG_INFO("Replaced type-1 texture slot ", ti, " with composited body+underwear");
                                break;
                            }
                        }
                    }
                }
                // Find cloak (type-2, Object Skin) texture slot index
                for (size_t ti = 0; ti < model.textures.size(); ti++) {
                    if (model.textures[ti].type == 2) {
                        cloakTextureSlotIndex_ = static_cast<uint32_t>(ti);
                        LOG_INFO("Cloak texture slot: ", ti);
                        break;
                    }
                }

                loaded = true;
                LOG_INFO("Loaded HumanMale M2: ", model.vertices.size(), " verts, ",
                         model.bones.size(), " bones, ", model.sequences.size(), " anims, ",
                         model.indices.size(), " indices, ", model.batches.size(), " batches");
                // Log all animation sequence IDs
                for (size_t i = 0; i < model.sequences.size(); i++) {
                    LOG_INFO("  Anim[", i, "]: id=", model.sequences[i].id,
                             " duration=", model.sequences[i].duration, "ms",
                             " speed=", model.sequences[i].movingSpeed);
                }
            }
        }
    }

    // Fallback: create a simple cube if MPQ not available
    if (!loaded) {
        pipeline::M2Model testModel;
        float size = 2.0f;
        glm::vec3 cubePos[] = {
            {-size, -size, -size}, { size, -size, -size},
            { size,  size, -size}, {-size,  size, -size},
            {-size, -size,  size}, { size, -size,  size},
            { size,  size,  size}, {-size,  size,  size}
        };
        for (const auto& pos : cubePos) {
            pipeline::M2Vertex v;
            v.position = pos;
            v.normal = glm::normalize(pos);
            v.texCoords[0] = glm::vec2(0.0f);
            v.boneWeights[0] = 255;
            v.boneWeights[1] = v.boneWeights[2] = v.boneWeights[3] = 0;
            v.boneIndices[0] = 0;
            v.boneIndices[1] = v.boneIndices[2] = v.boneIndices[3] = 0;
            testModel.vertices.push_back(v);
        }
        uint16_t cubeIndices[] = {
            0,1,2, 0,2,3, 4,6,5, 4,7,6,
            0,4,5, 0,5,1, 2,6,7, 2,7,3,
            0,3,7, 0,7,4, 1,5,6, 1,6,2
        };
        for (uint16_t idx : cubeIndices)
            testModel.indices.push_back(idx);

        pipeline::M2Bone bone;
        bone.keyBoneId = -1;
        bone.flags = 0;
        bone.parentBone = -1;
        bone.submeshId = 0;
        bone.pivot = glm::vec3(0.0f);
        testModel.bones.push_back(bone);

        pipeline::M2Sequence seq{};
        seq.id = 0;
        seq.duration = 1000;
        testModel.sequences.push_back(seq);

        testModel.name = "TestCube";
        testModel.globalFlags = 0;
        charRenderer->loadModel(testModel, 1);
        LOG_INFO("Loaded fallback cube model (no MPQ data)");
    }

    // Spawn character at camera's ground position
    glm::vec3 spawnPos = camera->getPosition() - glm::vec3(0.0f, 0.0f, 5.0f);
    uint32_t instanceId = charRenderer->createInstance(1, spawnPos,
        glm::vec3(0.0f), 1.0f);  // Scale 1.0 = normal WoW character size

    if (instanceId > 0) {
        // Set up third-person follow
        renderer->getCharacterPosition() = spawnPos;
        renderer->setCharacterFollow(instanceId);

        // Default geosets for naked human male
        // Use actual submesh IDs from the model (logged at load time)
        std::unordered_set<uint16_t> activeGeosets;
        // Body parts (group 0: IDs 0-18)
        for (uint16_t i = 0; i <= 18; i++) {
            activeGeosets.insert(i);
        }
        // Equipment groups: "01" = bare skin, "02" = first equipped variant
        activeGeosets.insert(101);   // Hair style 1
        activeGeosets.insert(201);   // Facial hair: none
        activeGeosets.insert(301);   // Gloves: bare hands
        activeGeosets.insert(401);   // Boots: bare feet
        activeGeosets.insert(501);   // Chest: bare
        activeGeosets.insert(701);   // Ears: default
        activeGeosets.insert(1301);  // Trousers: bare legs
        activeGeosets.insert(1501);  // Back body (cloak=none)
        // 1703 = DK eye glow mesh — skip for normal characters
        // Normal eyes are part of the face texture on the body mesh
        charRenderer->setActiveGeosets(instanceId, activeGeosets);

        // Play idle animation (Stand = animation ID 0)
        charRenderer->playAnimation(instanceId, 0, true);
        LOG_INFO("Spawned player character at (",
                static_cast<int>(spawnPos.x), ", ",
                static_cast<int>(spawnPos.y), ", ",
                static_cast<int>(spawnPos.z), ")");
        playerCharacterSpawned = true;

        // Set up camera controller for first-person player hiding
        if (renderer->getCameraController()) {
            renderer->getCameraController()->setCharacterRenderer(charRenderer, instanceId);
        }

        // Load equipped weapons (sword + shield)
        loadEquippedWeapons();
    }
}

void Application::loadEquippedWeapons() {
    if (!renderer || !renderer->getCharacterRenderer() || !assetManager || !assetManager->isInitialized())
        return;
    if (!gameHandler) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    uint32_t charInstanceId = renderer->getCharacterInstanceId();
    if (charInstanceId == 0) return;

    auto& inventory = gameHandler->getInventory();

    // Load ItemDisplayInfo.dbc
    auto displayInfoDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) {
        LOG_WARNING("loadEquippedWeapons: failed to load ItemDisplayInfo.dbc");
        return;
    }
    // Mapping: EquipSlot → attachment ID (1=RightHand, 2=LeftHand)
    struct WeaponSlot {
        game::EquipSlot slot;
        uint32_t attachmentId;
    };
    WeaponSlot weaponSlots[] = {
        { game::EquipSlot::MAIN_HAND, 1 },
        { game::EquipSlot::OFF_HAND,  2 },
    };

    for (const auto& ws : weaponSlots) {
        const auto& equipSlot = inventory.getEquipSlot(ws.slot);

        // If slot is empty or has no displayInfoId, detach any existing weapon
        if (equipSlot.empty() || equipSlot.item.displayInfoId == 0) {
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        uint32_t displayInfoId = equipSlot.item.displayInfoId;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) {
            LOG_WARNING("loadEquippedWeapons: displayInfoId ", displayInfoId, " not found in DBC");
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        // DBC field 1 = modelName_1 (e.g. "Sword_1H_Short_A_02.mdx")
        std::string modelName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), 1);
        // DBC field 3 = modelTexture_1 (e.g. "Sword_1H_Short_A_02Rusty")
        std::string textureName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), 3);

        if (modelName.empty()) {
            LOG_WARNING("loadEquippedWeapons: empty model name for displayInfoId ", displayInfoId);
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        // Convert .mdx → .m2
        std::string modelFile = modelName;
        {
            size_t dotPos = modelFile.rfind('.');
            if (dotPos != std::string::npos) {
                modelFile = modelFile.substr(0, dotPos) + ".m2";
            } else {
                modelFile += ".m2";
            }
        }

        // Try Weapon directory first, then Shield
        std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
        auto m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            m2Path = "Item\\ObjectComponents\\Shield\\" + modelFile;
            m2Data = assetManager->readFile(m2Path);
        }
        if (m2Data.empty()) {
            LOG_WARNING("loadEquippedWeapons: failed to read ", modelFile);
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        auto weaponModel = pipeline::M2Loader::load(m2Data);

        // Load skin file
        std::string skinFile = modelFile;
        {
            size_t dotPos = skinFile.rfind('.');
            if (dotPos != std::string::npos) {
                skinFile = skinFile.substr(0, dotPos) + "00.skin";
            }
        }
        // Try same directory as m2
        std::string skinDir = m2Path.substr(0, m2Path.rfind('\\') + 1);
        auto skinData = assetManager->readFile(skinDir + skinFile);
        if (!skinData.empty()) {
            pipeline::M2Loader::loadSkin(skinData, weaponModel);
        }

        if (!weaponModel.isValid()) {
            LOG_WARNING("loadEquippedWeapons: invalid weapon model from ", m2Path);
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        // Build texture path
        std::string texturePath;
        if (!textureName.empty()) {
            texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
            if (!assetManager->fileExists(texturePath)) {
                texturePath = "Item\\ObjectComponents\\Shield\\" + textureName + ".blp";
            }
        }

        uint32_t weaponModelId = nextWeaponModelId_++;
        bool ok = charRenderer->attachWeapon(charInstanceId, ws.attachmentId,
                                              weaponModel, weaponModelId, texturePath);
        if (ok) {
            LOG_INFO("Equipped weapon: ", m2Path, " at attachment ", ws.attachmentId);
        }
    }
}

void Application::spawnNpcs() {
    if (npcsSpawned) return;
    if (!assetManager || !assetManager->isInitialized()) return;
    if (!renderer || !renderer->getCharacterRenderer() || !renderer->getCamera()) return;
    if (!gameHandler) return;

    npcManager = std::make_unique<game::NpcManager>();
    glm::vec3 playerSpawnGL = renderer->getCamera()->getPosition() - glm::vec3(0.0f, 0.0f, 5.0f);
    npcManager->initialize(assetManager.get(),
                           renderer->getCharacterRenderer(),
                           gameHandler->getEntityManager(),
                           playerSpawnGL);

    // If the player WoW position hasn't been set by the server yet (offline mode),
    // derive it from the camera so targeting distance calculations work.
    const auto& movement = gameHandler->getMovementInfo();
    if (movement.x == 0.0f && movement.y == 0.0f && movement.z == 0.0f) {
        constexpr float ZEROPOINT = 32.0f * 533.33333f;
        float wowX = ZEROPOINT - playerSpawnGL.y;
        float wowY = playerSpawnGL.z;
        float wowZ = ZEROPOINT - playerSpawnGL.x;
        gameHandler->setPosition(wowX, wowY, wowZ);
    }

    npcsSpawned = true;
    LOG_INFO("NPCs spawned for in-game session");
}

void Application::startSinglePlayer() {
    LOG_INFO("Starting single-player mode...");

    // Set single-player flag
    singlePlayerMode = true;

    // Create world object for single-player
    if (!world) {
        world = std::make_unique<game::World>();
        LOG_INFO("Single-player world created");
    }

    // Set up camera for single-player mode
    if (renderer && renderer->getCamera()) {
        auto* camera = renderer->getCamera();
        // Position: high above terrain to see landscape (terrain around origin is ~80-100 units high)
        camera->setPosition(glm::vec3(0.0f, 0.0f, 300.0f));  // 300 units up
        // Rotation: looking north (yaw 0) with downward tilt to see terrain
        camera->setRotation(0.0f, -30.0f);  // Look down more to see terrain below
        LOG_INFO("Camera positioned for single-player mode");
    }

    // Populate test inventory for single-player
    if (gameHandler) {
        gameHandler->getInventory().populateTestItems();
    }

    // Load weapon models for equipped items (after inventory is populated)
    loadEquippedWeapons();

    // Try to load test terrain if WOW_DATA_PATH is set
    if (renderer && assetManager && assetManager->isInitialized()) {
        LOG_INFO("Loading test terrain for single-player mode...");

        // Try to load Elwynn Forest (most common starting zone)
        // ADT coordinates: (32, 49) is near Northshire Abbey
        std::string adtPath = "World\\Maps\\Azeroth\\Azeroth_32_49.adt";

        if (renderer->loadTestTerrain(assetManager.get(), adtPath)) {
            LOG_INFO("Test terrain loaded successfully");
        } else {
            LOG_WARNING("Could not load test terrain - continuing with atmospheric rendering only");
            LOG_INFO("Set WOW_DATA_PATH environment variable to load terrain");
        }
    } else {
        LOG_INFO("Asset manager not available - atmospheric rendering only");
        LOG_INFO("Set WOW_DATA_PATH environment variable to enable terrain loading");
    }

    // Spawn test objects for single-player mode
    if (renderer) {
        LOG_INFO("Spawning test objects for single-player mode...");

        // Spawn test characters in a row
        auto* characterRenderer = renderer->getCharacterRenderer();
        if (characterRenderer) {
            // Create test character model (same as K key)
            pipeline::M2Model testModel;
            float size = 2.0f;
            std::vector<glm::vec3> cubePos = {
                {-size, -size, -size}, { size, -size, -size},
                { size,  size, -size}, {-size,  size, -size},
                {-size, -size,  size}, { size, -size,  size},
                { size,  size,  size}, {-size,  size,  size}
            };

            for (const auto& pos : cubePos) {
                pipeline::M2Vertex v;
                v.position = pos;
                v.normal = glm::normalize(pos);
                v.texCoords[0] = glm::vec2(0.0f);
                v.boneWeights[0] = 255;
                v.boneWeights[1] = v.boneWeights[2] = v.boneWeights[3] = 0;
                v.boneIndices[0] = 0;
                v.boneIndices[1] = v.boneIndices[2] = v.boneIndices[3] = 0;
                testModel.vertices.push_back(v);
            }

            // One bone at origin
            pipeline::M2Bone bone;
            bone.keyBoneId = -1;
            bone.flags = 0;
            bone.parentBone = -1;
            bone.submeshId = 0;
            bone.pivot = glm::vec3(0.0f);
            testModel.bones.push_back(bone);

            // Simple animation
            pipeline::M2Sequence seq{};
            seq.id = 0;
            seq.duration = 1000;
            testModel.sequences.push_back(seq);

            // Load model into renderer
            if (characterRenderer->loadModel(testModel, 1)) {
                // Spawn 5 characters in a row
                for (int i = 0; i < 5; i++) {
                    glm::vec3 pos(i * 15.0f - 30.0f, 80.0f, 0.0f);
                    characterRenderer->createInstance(1, pos);
                }
                LOG_INFO("Spawned 5 test characters");
            }
        }

        // Spawn test buildings in a grid
        auto* wmoRenderer = renderer->getWMORenderer();
        if (wmoRenderer) {
            // Create procedural test WMO if not already loaded
            pipeline::WMOModel testWMO;
            testWMO.version = 17;

            pipeline::WMOGroup group;
            group.vertices = {
                {{-5, -5, 0}, {0, 0, 1}, {0, 0}, {0.8f, 0.7f, 0.6f, 1.0f}},
                {{5, -5, 0}, {0, 0, 1}, {1, 0}, {0.8f, 0.7f, 0.6f, 1.0f}},
                {{5, 5, 0}, {0, 0, 1}, {1, 1}, {0.8f, 0.7f, 0.6f, 1.0f}},
                {{-5, 5, 0}, {0, 0, 1}, {0, 1}, {0.8f, 0.7f, 0.6f, 1.0f}},
                {{-5, -5, 10}, {0, 0, 1}, {0, 0}, {0.7f, 0.6f, 0.5f, 1.0f}},
                {{5, -5, 10}, {0, 0, 1}, {1, 0}, {0.7f, 0.6f, 0.5f, 1.0f}},
                {{5, 5, 10}, {0, 0, 1}, {1, 1}, {0.7f, 0.6f, 0.5f, 1.0f}},
                {{-5, 5, 10}, {0, 0, 1}, {0, 1}, {0.7f, 0.6f, 0.5f, 1.0f}}
            };

            pipeline::WMOBatch batch;
            batch.startIndex = 0;
            batch.indexCount = 36;
            batch.materialId = 0;
            group.batches.push_back(batch);

            group.indices = {
                0,1,2, 0,2,3, 4,6,5, 4,7,6,
                0,4,5, 0,5,1, 1,5,6, 1,6,2,
                2,6,7, 2,7,3, 3,7,4, 3,4,0
            };

            testWMO.groups.push_back(group);

            pipeline::WMOMaterial material;
            material.shader = 0;
            material.blendMode = 0;
            testWMO.materials.push_back(material);

            // Load the test model
            if (wmoRenderer->loadModel(testWMO, 1)) {
                // Spawn buildings in a grid pattern
                for (int x = -1; x <= 1; x++) {
                    for (int y = 0; y <= 2; y++) {
                        glm::vec3 pos(x * 30.0f, y * 30.0f + 120.0f, 0.0f);
                        wmoRenderer->createInstance(1, pos);
                    }
                }
                LOG_INFO("Spawned 9 test buildings");
            }
        }

        LOG_INFO("Test objects spawned - you should see characters and buildings");
        LOG_INFO("Use WASD to fly around, mouse to look");
        LOG_INFO("Press K for more characters, O for more buildings");
    }

    // Go directly to game
    setState(AppState::IN_GAME);
    LOG_INFO("Single-player mode started - press F1 for performance HUD");
}

} // namespace core
} // namespace wowee
