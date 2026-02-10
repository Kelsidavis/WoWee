#include "core/application.hpp"
#include "core/coordinates.hpp"
#include <unordered_set>
#include <cmath>
#include "core/spawn_presets.hpp"
#include "core/logger.hpp"
#include "core/memory_monitor.hpp"
#include "rendering/renderer.hpp"
#include "audio/npc_voice_manager.hpp"
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
#include "rendering/m2_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/loading_screen.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
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
#include <algorithm>
#include <cctype>
#include <cctype>
#include <optional>
#include <sstream>
#include <set>

namespace wowee {
namespace core {


const char* Application::mapIdToName(uint32_t mapId) {
    switch (mapId) {
        case 0: return "Azeroth";
        case 1: return "Kalimdor";
        case 530: return "Outland";
        case 571: return "Northrend";
        default: return "Azeroth";
    }
}

std::string Application::getPlayerModelPath() const {
    return game::getPlayerModelPath(playerRace_, playerGender_);
}


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

    // Initialize memory monitoring for dynamic cache sizing
    core::MemoryMonitor::getInstance().initialize();

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
        // Eagerly load creature display DBC lookups so first spawn doesn't stall
        buildCreatureDisplayLookups();
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
                    window->setSize(newWidth, newHeight);
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
                // F4: Toggle shadows
                else if (event.key.keysym.scancode == SDL_SCANCODE_F4) {
                    if (renderer) {
                        bool enabled = !renderer->areShadowsEnabled();
                        renderer->setShadowsEnabled(enabled);
                        LOG_INFO("Shadows: ", enabled ? "ON" : "OFF");
                    }
                }
            }
        }

        // Update input
        Input::getInstance().update();

        // Timing breakdown
        static int frameCount = 0;
        static double totalUpdateMs = 0, totalRenderMs = 0, totalSwapMs = 0;
        auto t1 = std::chrono::steady_clock::now();

        // Update application state
        update(deltaTime);
        auto t2 = std::chrono::steady_clock::now();

        // Render
        render();
        auto t3 = std::chrono::steady_clock::now();

        // Swap buffers
        window->swapBuffers();
        auto t4 = std::chrono::steady_clock::now();

        totalUpdateMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
        totalRenderMs += std::chrono::duration<double, std::milli>(t3 - t2).count();
        totalSwapMs += std::chrono::duration<double, std::milli>(t4 - t3).count();

        if (++frameCount >= 60) {
            printf("[Frame] Update: %.1f ms, Render: %.1f ms, Swap: %.1f ms\n",
                   totalUpdateMs / 60.0, totalRenderMs / 60.0, totalSwapMs / 60.0);
            frameCount = 0;
            totalUpdateMs = totalRenderMs = totalSwapMs = 0;
        }
    }

    LOG_INFO("Main loop ended");
}

void Application::shutdown() {
    LOG_INFO("Shutting down application");

    // Save floor cache before renderer is destroyed
    if (renderer && renderer->getWMORenderer()) {
        size_t cacheSize = renderer->getWMORenderer()->getFloorCacheSize();
        if (cacheSize > 0) {
            LOG_INFO("Saving WMO floor cache (", cacheSize, " entries)...");
            renderer->getWMORenderer()->saveFloorCache();
        }
    }

    // Stop renderer first: terrain streaming workers may still be reading via
    // AssetManager during shutdown, so renderer/terrain teardown must complete
    // before AssetManager is destroyed.
    renderer.reset();

    world.reset();
    gameHandler.reset();
    authHandler.reset();
    assetManager.reset();
    uiManager.reset();
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
        case AppState::CHARACTER_CREATION:
            // Show character create screen
            break;
        case AppState::CHARACTER_SELECTION:
            // Show character screen
            break;
        case AppState::IN_GAME: {
            // Wire up movement opcodes from camera controller
            if (renderer && renderer->getCameraController()) {
                auto* cc = renderer->getCameraController();
                cc->setMovementCallback([this](uint32_t opcode) {
                    if (gameHandler) {
                        gameHandler->sendMovement(static_cast<game::Opcode>(opcode));
                    }
                });
                // Keep player locomotion WoW-like in both single-player and online modes.
                cc->setUseWoWSpeed(true);
            }
            if (gameHandler) {
                gameHandler->setMeleeSwingCallback([this]() {
                    if (renderer) {
                        renderer->triggerMeleeSwing();
                    }
                });
            }
            // Load quest marker models
            loadQuestMarkerModels();
            break;
        }
        case AppState::DISCONNECTED:
            // Back to auth
            break;
    }
}

void Application::logoutToLogin() {
    LOG_INFO("Logout requested");
    if (gameHandler) {
        gameHandler->disconnect();
    }
    npcsSpawned = false;
    playerCharacterSpawned = false;
    world.reset();
    if (renderer) {
        // Remove old player model so it doesn't persist into next session
        if (auto* charRenderer = renderer->getCharacterRenderer()) {
            charRenderer->removeInstance(1);
        }
        if (auto* music = renderer->getMusicManager()) {
            music->stopMusic(0.0f);
        }
    }
    setState(AppState::AUTHENTICATION);
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
            if (authHandler) {
                authHandler->update(deltaTime);
            }
            break;

        case AppState::CHARACTER_CREATION:
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            if (uiManager) {
                uiManager->getCharacterCreateScreen().update(deltaTime);
            }
            break;

        case AppState::CHARACTER_SELECTION:
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            break;

        case AppState::IN_GAME: {
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            if (world) {
                world->update(deltaTime);
            }
            // Spawn NPCs once when entering world
            spawnNpcs();

            // Process deferred online creature spawns (throttled)
            processCreatureSpawnQueue();
            processGameObjectSpawnQueue();
            processPendingMount();
            if (npcManager && renderer && renderer->getCharacterRenderer()) {
                npcManager->update(deltaTime, renderer->getCharacterRenderer());
            }

            // Update 3D quest markers above NPCs
            updateQuestMarkers();

            // Sync server run speed to camera controller
            if (renderer && gameHandler && renderer->getCameraController()) {
                renderer->getCameraController()->setRunSpeedOverride(gameHandler->getServerRunSpeed());
            }

            bool onTaxi = gameHandler && gameHandler->isOnTaxiFlight();
            if (renderer && renderer->getCameraController()) {
                renderer->getCameraController()->setExternalFollow(onTaxi);
                renderer->getCameraController()->setExternalMoving(onTaxi);
                if (lastTaxiFlight_ && !onTaxi) {
                    renderer->getCameraController()->clearMovementInputs();
                }
                bool idleOrbit = renderer->getCameraController()->isIdleOrbit();
                if (idleOrbit && !idleYawned_ && renderer) {
                    renderer->playEmote("yawn");
                    idleYawned_ = true;
                } else if (!idleOrbit) {
                    idleYawned_ = false;
                }
            }
            if (renderer) {
                renderer->setTaxiFlight(onTaxi);
            }
            if (renderer && renderer->getTerrainManager()) {
                renderer->getTerrainManager()->setStreamingEnabled(true);
                // With 8GB tile cache and precaching, minimize streaming during taxi
                if (onTaxi) {
                    renderer->getTerrainManager()->setUpdateInterval(2.0f);  // Very infrequent updates - already precached
                    renderer->getTerrainManager()->setLoadRadius(2);  // 5x5 grid for taxi (each tile ~533 yards)
                } else {
                    // Ramp streaming back in after taxi to avoid end-of-flight hitches.
                    if (lastTaxiFlight_) {
                        taxiStreamCooldown_ = 2.5f;
                        renderer->getTerrainManager()->setLoadRadius(2);  // Back to 5x5
                    }
                    if (taxiStreamCooldown_ > 0.0f) {
                        taxiStreamCooldown_ -= deltaTime;
                        renderer->getTerrainManager()->setUpdateInterval(1.0f);
                    } else {
                        renderer->getTerrainManager()->setUpdateInterval(0.1f);
                    }
                }
            }
            lastTaxiFlight_ = onTaxi;

            // Sync character render position ↔ canonical WoW coords each frame
            if (renderer && gameHandler) {
                if (onTaxi) {
                    auto playerEntity = gameHandler->getEntityManager().getEntity(gameHandler->getPlayerGuid());
                    if (playerEntity) {
                        glm::vec3 canonical(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ());
                        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
                        renderer->getCharacterPosition() = renderPos;
                        // Lock yaw to server/taxi orientation so mouse cannot steer during taxi.
                        float yawDeg = glm::degrees(playerEntity->getOrientation()) + 90.0f;
                        renderer->setCharacterYaw(yawDeg);
                    }
                } else {
                    glm::vec3 renderPos = renderer->getCharacterPosition();
                    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
                    gameHandler->setPosition(canonical.x, canonical.y, canonical.z);

                    // Sync orientation: camera yaw (degrees) → WoW orientation (radians)
                    float yawDeg = renderer->getCharacterYaw();
                    float wowOrientation = glm::radians(yawDeg - 90.0f);
                    gameHandler->setOrientation(wowOrientation);
                }
            }

            // Send movement heartbeat every 500ms (keeps server position in sync)
            // Skip during taxi flights - server controls position
            if (gameHandler && renderer && !onTaxi) {
                movementHeartbeatTimer += deltaTime;
                if (movementHeartbeatTimer >= 0.5f) {
                    movementHeartbeatTimer = 0.0f;
                    gameHandler->sendMovement(game::Opcode::CMSG_MOVE_HEARTBEAT);
                }
            } else {
                movementHeartbeatTimer = 0.0f;
            }
            break;
        }

        case AppState::DISCONNECTED:
            // Handle disconnection
            break;
    }

    // Update renderer (camera, etc.) only when in-game
    if (renderer && state == AppState::IN_GAME) {
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

    // Only render 3D world when in-game (after server connect or single-player)
    if (state == AppState::IN_GAME) {
        if (world) {
            renderer->renderWorld(world.get());
        } else {
            renderer->renderWorld(nullptr);
        }
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
        std::string accountName = authHandler->getUsername();
        if (accountName.empty()) {
            LOG_WARNING("Auth username missing; falling back to TESTACCOUNT");
            accountName = "TESTACCOUNT";
        }

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
        // Always set the active character GUID
        if (gameHandler) {
            gameHandler->setActiveCharacterGuid(characterGuid);
        }
        // Online mode - login will be handled by world entry callback
        setState(AppState::IN_GAME);
    });

    // Character create screen callbacks
    uiManager->getCharacterCreateScreen().setOnCreate([this](const game::CharCreateData& data) {
        pendingCreatedCharacterName_ = data.name;  // Store name for auto-selection
        gameHandler->createCharacter(data);
    });

    uiManager->getCharacterCreateScreen().setOnCancel([this]() {
        setState(AppState::CHARACTER_SELECTION);
    });

    // Character create result callback
    gameHandler->setCharCreateCallback([this](bool success, const std::string& msg) {
        if (success) {
            // Auto-select the newly created character
            if (!pendingCreatedCharacterName_.empty()) {
                uiManager->getCharacterScreen().selectCharacterByName(pendingCreatedCharacterName_);
                pendingCreatedCharacterName_.clear();
            }
            setState(AppState::CHARACTER_SELECTION);
        } else {
            uiManager->getCharacterCreateScreen().setStatus(msg, true);
            pendingCreatedCharacterName_.clear();
        }
    });

    // World entry callback (online mode) - load terrain when entering world
    gameHandler->setWorldEntryCallback([this](uint32_t mapId, float x, float y, float z) {
        LOG_INFO("Online world entry: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")");
        loadOnlineWorldTerrain(mapId, x, y, z);
    });

    // /unstuck — snap upward 10m to escape minor WMO cracks
    gameHandler->setUnstuckCallback([this]() {
        if (!renderer || !renderer->getCameraController()) return;
        auto* cc = renderer->getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;
        glm::vec3 pos = *ft;
        pos.z += 10.0f;
        cc->teleportTo(pos);
    });

    // /unstuckgy — snap upward 50m to clear all WMO geometry, gravity re-settles onto terrain
    gameHandler->setUnstuckGyCallback([this]() {
        if (!renderer || !renderer->getCameraController()) return;
        auto* cc = renderer->getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;

        // Try last safe position first (nearby, terrain already loaded)
        if (cc->hasLastSafePosition()) {
            glm::vec3 safePos = cc->getLastSafePosition();
            safePos.z += 5.0f;
            cc->teleportTo(safePos);
            LOG_INFO("Unstuck: teleported to last safe position");
            return;
        }

        // No safe position — snap 50m upward to clear all WMO geometry
        glm::vec3 pos = *ft;
        pos.z += 50.0f;
        cc->teleportTo(pos);
        LOG_INFO("Unstuck: snapped 50m upward");
    });

    // Auto-unstuck: falling for > 5 seconds = void fall, teleport to map entry
    if (renderer->getCameraController()) {
        renderer->getCameraController()->setAutoUnstuckCallback([this]() {
            if (!renderer || !renderer->getCameraController()) return;
            auto* cc = renderer->getCameraController();

            // Last resort: teleport to map entry point (terrain guaranteed loaded here)
            glm::vec3 spawnPos = cc->getDefaultPosition();
            spawnPos.z += 5.0f;
            cc->teleportTo(spawnPos);
            LOG_INFO("Auto-unstuck: teleported to map entry point");
        });
    }

    // Bind point update (innkeeper) — position stored in gameHandler->getHomeBind()
    gameHandler->setBindPointCallback([this](uint32_t mapId, float x, float y, float z) {
        LOG_INFO("Bindpoint set: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")");
    });

    // Faction hostility map is built in buildFactionHostilityMap() when character enters world

    // Creature spawn callback (online mode) - spawn creature models
    gameHandler->setCreatureSpawnCallback([this](uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
        // Queue spawns to avoid hanging when many creatures appear at once
        pendingCreatureSpawns_.push_back({guid, displayId, x, y, z, orientation});
    });

    // Creature despawn callback (online mode) - remove creature models
    gameHandler->setCreatureDespawnCallback([this](uint64_t guid) {
        despawnOnlineCreature(guid);
    });

    // GameObject spawn callback (online mode) - spawn static models (mailboxes, etc.)
    gameHandler->setGameObjectSpawnCallback([this](uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
        pendingGameObjectSpawns_.push_back({guid, displayId, x, y, z, orientation});
    });

    // GameObject despawn callback (online mode) - remove static models
    gameHandler->setGameObjectDespawnCallback([this](uint64_t guid) {
        despawnOnlineGameObject(guid);
    });

    // Mount callback (online mode) - defer heavy model load to next frame
    gameHandler->setMountCallback([this](uint32_t mountDisplayId) {
        if (mountDisplayId == 0) {
            // Dismount is instant (no loading needed)
            if (renderer && renderer->getCharacterRenderer() && mountInstanceId_ != 0) {
                renderer->getCharacterRenderer()->removeInstance(mountInstanceId_);
                mountInstanceId_ = 0;
            }
            mountModelId_ = 0;
            pendingMountDisplayId_ = 0;
            if (renderer) renderer->clearMount();
            LOG_INFO("Dismounted");
            return;
        }
        // Queue the mount for processing in the next update() frame
        pendingMountDisplayId_ = mountDisplayId;
    });

    // Taxi precache callback - preload terrain tiles along flight path
    gameHandler->setTaxiPrecacheCallback([this](const std::vector<glm::vec3>& path) {
        if (!renderer || !renderer->getTerrainManager()) return;

        std::set<std::pair<int, int>> uniqueTiles;

        // Sample waypoints along path and gather tiles
        for (const auto& waypoint : path) {
            glm::vec3 renderPos = core::coords::canonicalToRender(waypoint);
            int tileX = static_cast<int>(32 - (renderPos.x / 533.33333f));
            int tileY = static_cast<int>(32 - (renderPos.y / 533.33333f));

            // Load tile at waypoint + 1 radius around it (3x3 per waypoint)
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int tx = tileX + dx;
                    int ty = tileY + dy;
                    if (tx >= 0 && tx <= 63 && ty >= 0 && ty <= 63) {
                        uniqueTiles.insert({tx, ty});
                    }
                }
            }
        }

        std::vector<std::pair<int, int>> tilesToLoad(uniqueTiles.begin(), uniqueTiles.end());
        LOG_INFO("Precaching ", tilesToLoad.size(), " tiles for taxi route");
        renderer->getTerrainManager()->precacheTiles(tilesToLoad);
    });

    // Taxi orientation callback - update mount rotation during flight
    gameHandler->setTaxiOrientationCallback([this](float yaw, float pitch, float roll) {
        if (renderer && renderer->getCameraController()) {
            // Convert radians to degrees for camera controller (character facing)
            float yawDegrees = glm::degrees(yaw);
            renderer->getCameraController()->setFacingYaw(yawDegrees);
            // Set mount pitch and roll for realistic flight animation
            renderer->setMountPitchRoll(pitch, roll);
        }
    });

    // Taxi flight start callback - upload all precached tiles to GPU before flight begins
    gameHandler->setTaxiFlightStartCallback([this]() {
        if (renderer && renderer->getTerrainManager() && renderer->getM2Renderer()) {
            LOG_INFO("Uploading all precached tiles (terrain + M2 models) to GPU before taxi flight...");
            auto start = std::chrono::steady_clock::now();
            renderer->getTerrainManager()->processAllReadyTiles();
            auto end = std::chrono::steady_clock::now();
            auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            uint32_t m2Count = renderer->getM2Renderer()->getModelCount();
            uint32_t instCount = renderer->getM2Renderer()->getInstanceCount();
            LOG_INFO("GPU upload completed in ", durationMs, "ms - ", m2Count, " M2 models in VRAM (", instCount, " instances)");
        }
    });

    // Creature move callback (online mode) - update creature positions
    gameHandler->setCreatureMoveCallback([this](uint64_t guid, float x, float y, float z, uint32_t durationMs) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end() && renderer && renderer->getCharacterRenderer()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
            float durationSec = static_cast<float>(durationMs) / 1000.0f;
            renderer->getCharacterRenderer()->moveInstanceTo(it->second, renderPos, durationSec);
        }
    });

    // Transport move callback (online mode) - update transport gameobject positions
    gameHandler->setTransportMoveCallback([this](uint64_t guid, float x, float y, float z, float /*orientation*/) {
        auto it = gameObjectInstances_.find(guid);
        if (it == gameObjectInstances_.end()) return;
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        if (renderer) {
            if (it->second.isWmo) {
                if (auto* wmoRenderer = renderer->getWMORenderer()) {
                    wmoRenderer->setInstancePosition(it->second.instanceId, renderPos);
                }
            } else {
                if (auto* m2Renderer = renderer->getM2Renderer()) {
                    m2Renderer->setInstancePosition(it->second.instanceId, renderPos);
                }
            }

            // Move player with transport if riding it
            if (gameHandler && gameHandler->isOnTransport() && gameHandler->getPlayerTransportGuid() == guid) {
                auto* cc = renderer->getCameraController();
                if (cc) {
                    glm::vec3* ft = cc->getFollowTargetMutable();
                    if (ft) {
                        // Transport offset is in server/canonical coords — convert to render
                        glm::vec3 offset = gameHandler->getPlayerTransportOffset();
                        glm::vec3 canonicalPlayerPos = glm::vec3(x + offset.x, y + offset.y, z + offset.z);
                        glm::vec3 playerRenderPos = core::coords::canonicalToRender(canonicalPlayerPos);
                        *ft = playerRenderPos;
                    }
                }
            }
        }
    });

    // NPC death callback (online mode) - play death animation
    gameHandler->setNpcDeathCallback([this](uint64_t guid) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end() && renderer && renderer->getCharacterRenderer()) {
            renderer->getCharacterRenderer()->playAnimation(it->second, 1, false); // Death
        }
    });

    // NPC respawn callback (online mode) - reset to idle animation
    gameHandler->setNpcRespawnCallback([this](uint64_t guid) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end() && renderer && renderer->getCharacterRenderer()) {
            renderer->getCharacterRenderer()->playAnimation(it->second, 0, true); // Idle
        }
    });

    // NPC swing callback (online mode) - play attack animation
    gameHandler->setNpcSwingCallback([this](uint64_t guid) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end() && renderer && renderer->getCharacterRenderer()) {
            renderer->getCharacterRenderer()->playAnimation(it->second, 16, false); // Attack
        }
    });

    // NPC greeting callback - play voice line
    gameHandler->setNpcGreetingCallback([this](uint64_t guid, const glm::vec3& position) {
        if (renderer && renderer->getNpcVoiceManager()) {
            // Convert canonical to render coords for 3D audio
            glm::vec3 renderPos = core::coords::canonicalToRender(position);

            // Detect voice type from NPC display ID
            audio::VoiceType voiceType = audio::VoiceType::GENERIC;
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                uint32_t displayId = unit->getDisplayId();
                voiceType = detectVoiceTypeFromDisplayId(displayId);
            }

            renderer->getNpcVoiceManager()->playGreeting(guid, voiceType, renderPos);
        }
    });

    // NPC farewell callback - play farewell voice line
    gameHandler->setNpcFarewellCallback([this](uint64_t guid, const glm::vec3& position) {
        if (renderer && renderer->getNpcVoiceManager()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(position);

            audio::VoiceType voiceType = audio::VoiceType::GENERIC;
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                uint32_t displayId = unit->getDisplayId();
                voiceType = detectVoiceTypeFromDisplayId(displayId);
            }

            renderer->getNpcVoiceManager()->playFarewell(guid, voiceType, renderPos);
        }
    });

    // NPC vendor callback - play vendor voice line
    gameHandler->setNpcVendorCallback([this](uint64_t guid, const glm::vec3& position) {
        if (renderer && renderer->getNpcVoiceManager()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(position);

            audio::VoiceType voiceType = audio::VoiceType::GENERIC;
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                uint32_t displayId = unit->getDisplayId();
                voiceType = detectVoiceTypeFromDisplayId(displayId);
            }

            renderer->getNpcVoiceManager()->playVendor(guid, voiceType, renderPos);
        }
    });

    // NPC aggro callback - play combat start voice line
    gameHandler->setNpcAggroCallback([this](uint64_t guid, const glm::vec3& position) {
        if (renderer && renderer->getNpcVoiceManager()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(position);

            audio::VoiceType voiceType = audio::VoiceType::GENERIC;
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                uint32_t displayId = unit->getDisplayId();
                voiceType = detectVoiceTypeFromDisplayId(displayId);
            }

            renderer->getNpcVoiceManager()->playAggro(guid, voiceType, renderPos);
        }
    });

    // "Create Character" button on character screen
    uiManager->getCharacterScreen().setOnCreateCharacter([this]() {
        uiManager->getCharacterCreateScreen().reset();
        uiManager->getCharacterCreateScreen().initializePreview(assetManager.get());
        setState(AppState::CHARACTER_CREATION);
    });

    // "Back" button on character screen
    uiManager->getCharacterScreen().setOnBack([this]() {
        setState(AppState::REALM_SELECTION);
    });

    // "Delete Character" button on character screen
    uiManager->getCharacterScreen().setOnDeleteCharacter([this](uint64_t guid) {
        if (gameHandler) {
            gameHandler->deleteCharacter(guid);
        }
    });

    // Character delete result callback
    gameHandler->setCharDeleteCallback([this](bool success) {
        if (success) {
            uiManager->getCharacterScreen().setStatus("Character deleted.");
            // Refresh character list
            gameHandler->requestCharacterList();
        } else {
            uint8_t code = gameHandler ? gameHandler->getLastCharDeleteResult() : 0xFF;
            uiManager->getCharacterScreen().setStatus(
                "Delete failed (code " + std::to_string(static_cast<int>(code)) + ").");
        }
    });
}

void Application::spawnPlayerCharacter() {
    if (playerCharacterSpawned) return;
    if (!renderer || !renderer->getCharacterRenderer() || !renderer->getCamera()) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    auto* camera = renderer->getCamera();
    bool loaded = false;
    std::string m2Path = getPlayerModelPath();
    std::string modelDir;
    std::string baseName;
    {
        size_t slash = m2Path.rfind('\\');
        if (slash != std::string::npos) {
            modelDir = m2Path.substr(0, slash + 1);
            baseName = m2Path.substr(slash + 1);
        } else {
            baseName = m2Path;
        }
        size_t dot = baseName.rfind('.');
        if (dot != std::string::npos) {
            baseName = baseName.substr(0, dot);
        }
    }

    // Try loading selected character model from MPQ
    if (assetManager && assetManager->isInitialized()) {
        auto m2Data = assetManager->readFile(m2Path);
        if (!m2Data.empty()) {
            auto model = pipeline::M2Loader::load(m2Data);

            // Load skin file for submesh/batch data
            std::string skinPath = modelDir + baseName + "00.skin";
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

                // Look up textures from CharSections.dbc for all races
                bool useCharSections = true;
                uint32_t targetRaceId = static_cast<uint32_t>(playerRace_);
                uint32_t targetSexId = (playerGender_ == game::Gender::FEMALE) ? 1u : 0u;

                // Race name for fallback texture paths
                const char* raceFolderName = "Human";
                switch (playerRace_) {
                    case game::Race::HUMAN:    raceFolderName = "Human"; break;
                    case game::Race::ORC:      raceFolderName = "Orc"; break;
                    case game::Race::DWARF:    raceFolderName = "Dwarf"; break;
                    case game::Race::NIGHT_ELF: raceFolderName = "NightElf"; break;
                    case game::Race::UNDEAD:    raceFolderName = "Scourge"; break;
                    case game::Race::TAUREN:    raceFolderName = "Tauren"; break;
                    case game::Race::GNOME:     raceFolderName = "Gnome"; break;
                    case game::Race::TROLL:     raceFolderName = "Troll"; break;
                    case game::Race::BLOOD_ELF: raceFolderName = "BloodElf"; break;
                    case game::Race::DRAENEI:   raceFolderName = "Draenei"; break;
                    default: break;
                }
                const char* genderFolder = (playerGender_ == game::Gender::FEMALE) ? "Female" : "Male";
                std::string raceGender = std::string(raceFolderName) + genderFolder;
                std::string bodySkinPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "Skin00_00.blp";
                std::string pelvisPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "NakedPelvisSkin00_00.blp";
                std::string faceLowerTexturePath;
                std::vector<std::string> underwearPaths;

                // Extract appearance bytes for texture lookups
                uint8_t charSkinId = 0, charFaceId = 0, charHairStyleId = 0, charHairColorId = 0;
                if (gameHandler) {
                    const game::Character* activeChar = gameHandler->getActiveCharacter();
                    if (activeChar) {
                        charSkinId = activeChar->appearanceBytes & 0xFF;
                        charFaceId = (activeChar->appearanceBytes >> 8) & 0xFF;
                        charHairStyleId = (activeChar->appearanceBytes >> 16) & 0xFF;
                        charHairColorId = (activeChar->appearanceBytes >> 24) & 0xFF;
                        LOG_INFO("Appearance: skin=", (int)charSkinId, " face=", (int)charFaceId,
                                 " hairStyle=", (int)charHairStyleId, " hairColor=", (int)charHairColorId);
                    }
                }

                std::string hairTexturePath;
                if (useCharSections) {
                    auto charSectionsDbc = assetManager->loadDBC("CharSections.dbc");
                    if (charSectionsDbc) {
                        LOG_INFO("CharSections.dbc loaded: ", charSectionsDbc->getRecordCount(), " records");
                        bool foundSkin = false;
                        bool foundUnderwear = false;
                        bool foundFaceLower = false;
                        bool foundHair = false;
                        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                            uint32_t raceId = charSectionsDbc->getUInt32(r, 1);
                            uint32_t sexId = charSectionsDbc->getUInt32(r, 2);
                            uint32_t baseSection = charSectionsDbc->getUInt32(r, 3);
                            uint32_t variationIndex = charSectionsDbc->getUInt32(r, 8);
                            uint32_t colorIndex = charSectionsDbc->getUInt32(r, 9);

                            if (raceId != targetRaceId || sexId != targetSexId) continue;

                            // Section 0 = skin: match by colorIndex = skin byte
                            if (baseSection == 0 && !foundSkin && colorIndex == charSkinId) {
                                std::string tex1 = charSectionsDbc->getString(r, 4);
                                if (!tex1.empty()) {
                                    bodySkinPath = tex1;
                                    foundSkin = true;
                                    LOG_INFO("  DBC body skin: ", bodySkinPath, " (skin=", (int)charSkinId, ")");
                                }
                            }
                            // Section 3 = hair: match variation=hairStyle, color=hairColor
                            else if (baseSection == 3 && !foundHair &&
                                     variationIndex == charHairStyleId && colorIndex == charHairColorId) {
                                hairTexturePath = charSectionsDbc->getString(r, 4);
                                if (!hairTexturePath.empty()) {
                                    foundHair = true;
                                    LOG_INFO("  DBC hair texture: ", hairTexturePath,
                                             " (style=", (int)charHairStyleId, " color=", (int)charHairColorId, ")");
                                }
                            }
                            // Section 1 = face lower: match variation=faceId
                            else if (baseSection == 1 && !foundFaceLower &&
                                     variationIndex == charFaceId && colorIndex == charSkinId) {
                                std::string tex1 = charSectionsDbc->getString(r, 4);
                                if (!tex1.empty()) {
                                    faceLowerTexturePath = tex1;
                                    foundFaceLower = true;
                                    LOG_INFO("  DBC face texture: ", faceLowerTexturePath);
                                }
                            }
                            // Section 4 = underwear
                            else if (baseSection == 4 && !foundUnderwear && colorIndex == charSkinId) {
                                for (int f = 4; f <= 6; f++) {
                                    std::string tex = charSectionsDbc->getString(r, f);
                                    if (!tex.empty()) {
                                        underwearPaths.push_back(tex);
                                        LOG_INFO("  DBC underwear texture: ", tex);
                                    }
                                }
                                foundUnderwear = true;
                            }

                            if (foundSkin && foundHair && foundFaceLower && foundUnderwear) break;
                        }

                        if (!foundHair) {
                            LOG_WARNING("No DBC hair match for style=", (int)charHairStyleId,
                                        " color=", (int)charHairColorId,
                                        " race=", targetRaceId, " sex=", targetSexId);
                        }
                    } else {
                        LOG_WARNING("Failed to load CharSections.dbc, using hardcoded textures");
                    }

                    for (auto& tex : model.textures) {
                        if (tex.type == 1 && tex.filename.empty()) {
                            tex.filename = bodySkinPath;
                        } else if (tex.type == 6) {
                            if (!hairTexturePath.empty()) {
                                tex.filename = hairTexturePath;
                            } else if (tex.filename.empty()) {
                                tex.filename = std::string("Character\\") + raceFolderName + "\\Hair00_00.blp";
                            }
                        } else if (tex.type == 8 && tex.filename.empty()) {
                            if (!underwearPaths.empty()) {
                                tex.filename = underwearPaths[0];
                            } else {
                                tex.filename = pelvisPath;
                            }
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
                            "%s%s%04u-%02u.anim",
                            modelDir.c_str(),
                            baseName.c_str(),
                            model.sequences[si].id,
                            model.sequences[si].variationIndex);
                        auto animFileData = assetManager->readFile(animFileName);
                        if (!animFileData.empty()) {
                            pipeline::M2Loader::loadAnimFile(m2Data, animFileData, si, model);
                        }
                    }
                }

                charRenderer->loadModel(model, 1);

                if (useCharSections) {
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
                    // Override hair texture on GPU (type-6 slot) after model load
                    if (!hairTexturePath.empty()) {
                        GLuint hairTex = charRenderer->loadTexture(hairTexturePath);
                        if (hairTex != 0) {
                            for (size_t ti = 0; ti < model.textures.size(); ti++) {
                                if (model.textures[ti].type == 6) {
                                    charRenderer->setModelTexture(1, static_cast<uint32_t>(ti), hairTex);
                                    LOG_INFO("Applied DBC hair texture to slot ", ti, ": ", hairTexturePath);
                                    break;
                                }
                            }
                        }
                    }
                } else {
                    bodySkinPath_.clear();
                    underwearPaths_.clear();
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
                LOG_INFO("Loaded character model: ", m2Path, " (", model.vertices.size(), " verts, ",
                         model.bones.size(), " bones, ", model.sequences.size(), " anims, ",
                         model.indices.size(), " indices, ", model.batches.size(), " batches");
                // Log all animation sequence IDs
                for (size_t i = 0; i < model.sequences.size(); i++) {
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

    // Spawn character at the camera controller's default position (matches hearthstone).
    // Most presets snap to floor; explicit WMO-floor presets keep their authored Z.
    auto* camCtrl = renderer->getCameraController();
    glm::vec3 spawnPos = camCtrl ? camCtrl->getDefaultPosition()
                                 : (camera->getPosition() - glm::vec3(0.0f, 0.0f, 5.0f));
    if (spawnSnapToGround && renderer->getTerrainManager()) {
        auto terrainH = renderer->getTerrainManager()->getHeightAt(spawnPos.x, spawnPos.y);
        if (terrainH) {
            spawnPos.z = *terrainH + 0.1f;
        }
    }
    uint32_t instanceId = charRenderer->createInstance(1, spawnPos,
        glm::vec3(0.0f), 1.0f);  // Scale 1.0 = normal WoW character size

    if (instanceId > 0) {
        // Set up third-person follow
        renderer->getCharacterPosition() = spawnPos;
        renderer->setCharacterFollow(instanceId);

        // Default geosets for naked human male
        // Use actual submesh IDs from the model (logged at load time)
        std::unordered_set<uint16_t> activeGeosets;
        // Body parts (group 0: IDs 0-99) - humanoid models may have many body submeshes
        for (uint16_t i = 0; i < 100; i++) {
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
    LOG_INFO("spawnNpcs: checking preconditions...");
    if (!assetManager || !assetManager->isInitialized()) {
        LOG_INFO("spawnNpcs: assetManager not ready");
        return;
    }
    if (!renderer || !renderer->getCharacterRenderer() || !renderer->getCamera()) {
        LOG_INFO("spawnNpcs: renderer not ready");
        return;
    }
    if (!gameHandler) {
        LOG_INFO("spawnNpcs: gameHandler not ready");
        return;
    }

    LOG_INFO("spawnNpcs: spawning NPCs...");
    if (npcManager) {
        npcManager->clear(renderer->getCharacterRenderer(), &gameHandler->getEntityManager());
    }
    npcManager = std::make_unique<game::NpcManager>();
    glm::vec3 playerSpawnGL = renderer->getCharacterPosition();
    glm::vec3 playerCanonical = core::coords::renderToCanonical(playerSpawnGL);
    LOG_INFO("spawnNpcs: player position GL=(", playerSpawnGL.x, ",", playerSpawnGL.y, ",", playerSpawnGL.z,
             ") canonical=(", playerCanonical.x, ",", playerCanonical.y, ",", playerCanonical.z, ")");
    std::string mapName = "Azeroth";
    if (auto* minimap = renderer->getMinimap()) {
        mapName = minimap->getMapName();
    }

    npcManager->initialize(assetManager.get(),
                           renderer->getCharacterRenderer(),
                           gameHandler->getEntityManager(),
                           mapName,
                           playerCanonical,
                           renderer->getTerrainManager());

    // If the player WoW position hasn't been set by the server yet (offline mode),
    // derive it from the camera so targeting distance calculations work.
    const auto& movement = gameHandler->getMovementInfo();
    if (movement.x == 0.0f && movement.y == 0.0f && movement.z == 0.0f) {
        glm::vec3 canonical = playerCanonical;
        gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
    }

    // Set NPC animation callbacks (works for both single-player and online creatures)
    if (gameHandler && npcManager) {
        auto* npcMgr = npcManager.get();
        auto* cr = renderer->getCharacterRenderer();
        auto* app = this;
        gameHandler->setNpcDeathCallback([npcMgr, cr, app](uint64_t guid) {
            uint32_t instanceId = npcMgr->findRenderInstanceId(guid);
            if (instanceId == 0) {
                auto it = app->creatureInstances_.find(guid);
                if (it != app->creatureInstances_.end()) instanceId = it->second;
            }
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, 1, false); // animation ID 1 = Death
            }
        });
        gameHandler->setNpcRespawnCallback([npcMgr, cr, app](uint64_t guid) {
            uint32_t instanceId = npcMgr->findRenderInstanceId(guid);
            if (instanceId == 0) {
                auto it = app->creatureInstances_.find(guid);
                if (it != app->creatureInstances_.end()) instanceId = it->second;
            }
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, 0, true); // animation ID 0 = Idle
            }
        });
        gameHandler->setNpcSwingCallback([npcMgr, cr, app](uint64_t guid) {
            uint32_t instanceId = npcMgr->findRenderInstanceId(guid);
            if (instanceId == 0) {
                auto it = app->creatureInstances_.find(guid);
                if (it != app->creatureInstances_.end()) instanceId = it->second;
            }
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, 16, false); // animation ID 16 = Attack1
            }
        });
    }

    npcsSpawned = true;
    LOG_INFO("NPCs spawned for in-game session");
}


void Application::buildFactionHostilityMap(uint8_t playerRace) {
    if (!assetManager || !assetManager->isInitialized() || !gameHandler) return;

    auto ftDbc = assetManager->loadDBC("FactionTemplate.dbc");
    auto fDbc = assetManager->loadDBC("Faction.dbc");
    if (!ftDbc || !ftDbc->isLoaded()) return;

    // Race enum → race mask bit: race 1=0x1, 2=0x2, 3=0x4, 4=0x8, 5=0x10, 6=0x20, 7=0x40, 8=0x80, 10=0x200, 11=0x400
    uint32_t playerRaceMask = 0;
    if (playerRace >= 1 && playerRace <= 8) {
        playerRaceMask = 1u << (playerRace - 1);
    } else if (playerRace == 10) {
        playerRaceMask = 0x200;  // Blood Elf
    } else if (playerRace == 11) {
        playerRaceMask = 0x400;  // Draenei
    }

    // Race → player faction template ID
    // Human=1, Orc=2, Dwarf=3, NightElf=4, Undead=5, Tauren=6, Gnome=115, Troll=116, BloodElf=1610, Draenei=1629
    uint32_t playerFtId = 0;
    switch (playerRace) {
        case 1: playerFtId = 1; break;     // Human
        case 2: playerFtId = 2; break;     // Orc
        case 3: playerFtId = 3; break;     // Dwarf
        case 4: playerFtId = 4; break;     // Night Elf
        case 5: playerFtId = 5; break;     // Undead
        case 6: playerFtId = 6; break;     // Tauren
        case 7: playerFtId = 115; break;   // Gnome
        case 8: playerFtId = 116; break;   // Troll
        case 10: playerFtId = 1610; break; // Blood Elf
        case 11: playerFtId = 1629; break; // Draenei
        default: playerFtId = 1; break;
    }

    // Build set of hostile parent faction IDs from Faction.dbc base reputation
    std::unordered_set<uint32_t> hostileParentFactions;
    if (fDbc && fDbc->isLoaded()) {
        for (uint32_t i = 0; i < fDbc->getRecordCount(); i++) {
            uint32_t factionId = fDbc->getUInt32(i, 0);
            for (int slot = 0; slot < 4; slot++) {
                uint32_t raceMask = fDbc->getUInt32(i, 2 + slot);  // ReputationRaceMask[4] at fields 2-5
                if (raceMask & playerRaceMask) {
                    int32_t baseRep = fDbc->getInt32(i, 10 + slot);  // ReputationBase[4] at fields 10-13
                    if (baseRep < 0) {
                        hostileParentFactions.insert(factionId);
                    }
                    break;
                }
            }
        }
        LOG_INFO("Faction.dbc: ", hostileParentFactions.size(), " factions hostile to race ", (int)playerRace);
    }

    // Get player faction template data
    uint32_t playerFriendGroup = 0;
    uint32_t playerEnemyGroup = 0;
    uint32_t playerFactionId = 0;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        if (ftDbc->getUInt32(i, 0) == playerFtId) {
            playerFriendGroup = ftDbc->getUInt32(i, 4) | ftDbc->getUInt32(i, 3);
            playerEnemyGroup = ftDbc->getUInt32(i, 5);
            playerFactionId = ftDbc->getUInt32(i, 1);
            break;
        }
    }

    // Build hostility map for each faction template
    std::unordered_map<uint32_t, bool> factionMap;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        uint32_t id = ftDbc->getUInt32(i, 0);
        uint32_t parentFaction = ftDbc->getUInt32(i, 1);
        uint32_t factionGroup = ftDbc->getUInt32(i, 3);
        uint32_t friendGroup = ftDbc->getUInt32(i, 4);
        uint32_t enemyGroup = ftDbc->getUInt32(i, 5);

        // 1. Symmetric group check
        bool hostile = (enemyGroup & playerFriendGroup) != 0
                    || (factionGroup & playerEnemyGroup) != 0;

        // 2. Monster factionGroup bit (8)
        if (!hostile && (factionGroup & 8) != 0) {
            hostile = true;
        }

        // 3. Individual enemy faction IDs (fields 6-9)
        if (!hostile && playerFactionId > 0) {
            for (int e = 6; e <= 9; e++) {
                if (ftDbc->getUInt32(i, e) == playerFactionId) {
                    hostile = true;
                    break;
                }
            }
        }

        // 4. Parent faction base reputation check (Faction.dbc)
        if (!hostile && parentFaction > 0) {
            if (hostileParentFactions.count(parentFaction)) {
                hostile = true;
            }
        }

        // 5. If explicitly friendly (friendGroup includes player), override to non-hostile
        if (hostile && (friendGroup & playerFriendGroup) != 0) {
            hostile = false;
        }

        factionMap[id] = hostile;
    }

    uint32_t hostileCount = 0;
    for (const auto& [fid, h] : factionMap) { if (h) hostileCount++; }
    gameHandler->setFactionHostileMap(std::move(factionMap));
    LOG_INFO("Faction hostility for race ", (int)playerRace, " (FT ", playerFtId, "): ",
        hostileCount, "/", ftDbc->getRecordCount(),
        " hostile (friendGroup=0x", std::hex, playerFriendGroup, ", enemyGroup=0x", playerEnemyGroup, std::dec, ")");
}

void Application::loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z) {
    if (!renderer || !assetManager || !assetManager->isInitialized()) {
        LOG_WARNING("Cannot load online terrain: renderer or assets not ready");
        return;
    }

    // --- Loading screen for online mode ---
    rendering::LoadingScreen loadingScreen;
    bool loadingScreenOk = loadingScreen.initialize();

    auto showProgress = [&](const char* msg, float progress) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                window->setShouldClose(true);
                loadingScreen.shutdown();
                return;
            }
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_RESIZED) {
                int w = event.window.data1;
                int h = event.window.data2;
                window->setSize(w, h);
                glViewport(0, 0, w, h);
                if (renderer && renderer->getCamera()) {
                    renderer->getCamera()->setAspectRatio(static_cast<float>(w) / h);
                }
            }
        }
        if (!loadingScreenOk) return;
        loadingScreen.setStatus(msg);
        loadingScreen.setProgress(progress);
        loadingScreen.render();
        window->swapBuffers();
    };

    showProgress("Entering world...", 0.0f);

    std::string mapName = mapIdToName(mapId);
    LOG_INFO("Loading online world terrain for map '", mapName, "' (ID ", mapId, ")");

    // Convert server coordinates to canonical WoW coordinates
    // Server sends: X=West (canonical.Y), Y=North (canonical.X), Z=Up
    glm::vec3 spawnCanonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
    glm::vec3 spawnRender = core::coords::canonicalToRender(spawnCanonical);

    // Set camera position
    if (renderer->getCameraController()) {
        renderer->getCameraController()->setOnlineMode(true);
        renderer->getCameraController()->setDefaultSpawn(spawnRender, 0.0f, 15.0f);
        renderer->getCameraController()->reset();
        renderer->getCameraController()->startIntroPan(2.8f, 140.0f);
    }

    // Set map name for WMO renderer
    if (renderer->getWMORenderer()) {
        renderer->getWMORenderer()->setMapName(mapName);
    }

    // Set map name for terrain manager
    if (renderer->getTerrainManager()) {
        renderer->getTerrainManager()->setMapName(mapName);
    }

    showProgress("Loading character model...", 0.05f);

    // Build faction hostility map for this character's race
    if (gameHandler) {
        const game::Character* activeChar = gameHandler->getActiveCharacter();
        if (activeChar) {
            buildFactionHostilityMap(static_cast<uint8_t>(activeChar->race));
        }
    }

    // Spawn player model for online mode (skip if already spawned, e.g. teleport)
    if (gameHandler) {
        const game::Character* activeChar = gameHandler->getActiveCharacter();
        if (activeChar) {
            if (!playerCharacterSpawned) {
                playerRace_ = activeChar->race;
                playerGender_ = activeChar->gender;
                playerClass_ = activeChar->characterClass;
                spawnSnapToGround = false;
                spawnPlayerCharacter();
            }
            renderer->getCharacterPosition() = spawnRender;
            LOG_INFO("Online player at render pos (", spawnRender.x, ", ", spawnRender.y, ", ", spawnRender.z, ")");
        } else {
            LOG_WARNING("No active character found for player model spawning");
        }
    }

    showProgress("Loading terrain...", 0.20f);

    // Compute ADT tile from canonical coordinates
    auto [tileX, tileY] = core::coords::canonicalToTile(spawnCanonical.x, spawnCanonical.y);
    std::string adtPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                          std::to_string(tileX) + "_" + std::to_string(tileY) + ".adt";
    LOG_INFO("Loading ADT tile [", tileX, ",", tileY, "] from canonical (",
             spawnCanonical.x, ", ", spawnCanonical.y, ", ", spawnCanonical.z, ")");

    // Load the initial terrain tile
    bool terrainOk = renderer->loadTestTerrain(assetManager.get(), adtPath);
    if (!terrainOk) {
        LOG_WARNING("Could not load terrain for online world - atmospheric rendering only");
    } else {
        LOG_INFO("Online world terrain loading initiated");
    }

    showProgress("Streaming terrain tiles...", 0.35f);

    // Wait for surrounding terrain tiles to stream in
    if (terrainOk && renderer->getTerrainManager() && renderer->getCamera()) {
        auto* terrainMgr = renderer->getTerrainManager();
        auto* camera = renderer->getCamera();

        // Trigger tile streaming for surrounding area
        terrainMgr->update(*camera, 1.0f);

        auto startTime = std::chrono::high_resolution_clock::now();
        auto lastProgressTime = startTime;
        const float maxWaitSeconds = 20.0f;
        const float stallSeconds = 5.0f;
        int initialRemaining = terrainMgr->getRemainingTileCount();
        if (initialRemaining < 1) initialRemaining = 1;
        int lastRemaining = initialRemaining;

        // Wait until all pending + ready-queue tiles are finalized
        while (terrainMgr->getRemainingTileCount() > 0) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    window->setShouldClose(true);
                    loadingScreen.shutdown();
                    return;
                }
                if (event.type == SDL_WINDOWEVENT &&
                    event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int w = event.window.data1;
                    int h = event.window.data2;
                    window->setSize(w, h);
                    glViewport(0, 0, w, h);
                    if (renderer->getCamera()) {
                        renderer->getCamera()->setAspectRatio(static_cast<float>(w) / h);
                    }
                }
            }

            // Trigger new streaming and process ALL ready tiles (not just 2)
            terrainMgr->update(*camera, 0.016f);
            terrainMgr->processAllReadyTiles();

            if (loadingScreenOk) {
                int remaining = terrainMgr->getRemainingTileCount();
                int loaded = terrainMgr->getLoadedTileCount();
                float tileProgress = static_cast<float>(initialRemaining - remaining) / initialRemaining;
                if (tileProgress < 0.0f) tileProgress = 0.0f;
                float progress = 0.35f + tileProgress * 0.50f;
                char buf[128];
                snprintf(buf, sizeof(buf), "Loading terrain... %d tiles loaded, %d remaining",
                         loaded, remaining);
                loadingScreen.setStatus(buf);
                loadingScreen.setProgress(progress);
                loadingScreen.render();
                window->swapBuffers();

                if (remaining != lastRemaining) {
                    lastRemaining = remaining;
                    lastProgressTime = std::chrono::high_resolution_clock::now();
                }
            }

            auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
            if (std::chrono::duration<float>(elapsed).count() > maxWaitSeconds) {
                LOG_WARNING("Online terrain streaming timeout after ", maxWaitSeconds, "s");
                break;
            }
            auto stalledFor = std::chrono::high_resolution_clock::now() - lastProgressTime;
            if (std::chrono::duration<float>(stalledFor).count() > stallSeconds) {
                LOG_WARNING("Online terrain streaming stalled for ", stallSeconds,
                            "s (remaining=", lastRemaining, "), continuing without full preload");
                break;
            }

            SDL_Delay(16);
        }

        LOG_INFO("Online terrain streaming complete: ", terrainMgr->getLoadedTileCount(), " tiles loaded");

        // Load/precompute collision cache
        if (renderer->getWMORenderer()) {
            showProgress("Building collision cache...", 0.88f);
            renderer->getWMORenderer()->loadFloorCache();
            if (renderer->getWMORenderer()->getFloorCacheSize() == 0) {
                renderer->getWMORenderer()->precomputeFloorCache();
            }
        }
    }

    // Snap player to loaded terrain so they don't spawn underground
    if (renderer->getCameraController()) {
        renderer->getCameraController()->reset();
    }

    showProgress("Entering world...", 1.0f);

    if (loadingScreenOk) {
        loadingScreen.shutdown();
    }

    // Set game state
    setState(AppState::IN_GAME);
}

void Application::buildCreatureDisplayLookups() {
    if (creatureLookupsBuilt_ || !assetManager || !assetManager->isInitialized()) return;

    LOG_INFO("Building creature display lookups from DBC files");

    // CreatureDisplayInfo.dbc structure (3.3.5a):
    // Col 0: displayId
    // Col 1: modelId
    // Col 3: extendedDisplayInfoID (link to CreatureDisplayInfoExtra.dbc)
    // Col 6: Skin1 (texture name)
    // Col 7: Skin2
    // Col 8: Skin3
    if (auto cdi = assetManager->loadDBC("CreatureDisplayInfo.dbc"); cdi && cdi->isLoaded()) {
        for (uint32_t i = 0; i < cdi->getRecordCount(); i++) {
            CreatureDisplayData data;
            data.modelId = cdi->getUInt32(i, 1);
            data.extraDisplayId = cdi->getUInt32(i, 3);
            data.skin1 = cdi->getString(i, 6);
            data.skin2 = cdi->getString(i, 7);
            data.skin3 = cdi->getString(i, 8);
            displayDataMap_[cdi->getUInt32(i, 0)] = data;
        }
        LOG_INFO("Loaded ", displayDataMap_.size(), " display→model mappings");
    }

    // CreatureDisplayInfoExtra.dbc structure (3.3.5a):
    // Col 0: ID
    // Col 1: DisplayRaceID
    // Col 2: DisplaySexID
    // Col 3: SkinID
    // Col 4: FaceID
    // Col 5: HairStyleID
    // Col 6: HairColorID
    // Col 7: FacialHairID
    // Col 8-18: Item display IDs (equipment slots)
    // Col 19: Flags
    // Col 20: BakeName (pre-baked texture path)
    if (auto cdie = assetManager->loadDBC("CreatureDisplayInfoExtra.dbc"); cdie && cdie->isLoaded()) {
        uint32_t withBakeName = 0;
        for (uint32_t i = 0; i < cdie->getRecordCount(); i++) {
            HumanoidDisplayExtra extra;
            extra.raceId = static_cast<uint8_t>(cdie->getUInt32(i, 1));
            extra.sexId = static_cast<uint8_t>(cdie->getUInt32(i, 2));
            extra.skinId = static_cast<uint8_t>(cdie->getUInt32(i, 3));
            extra.faceId = static_cast<uint8_t>(cdie->getUInt32(i, 4));
            extra.hairStyleId = static_cast<uint8_t>(cdie->getUInt32(i, 5));
            extra.hairColorId = static_cast<uint8_t>(cdie->getUInt32(i, 6));
            extra.facialHairId = static_cast<uint8_t>(cdie->getUInt32(i, 7));
            // Equipment display IDs (columns 8-18)
            for (int eq = 0; eq < 11; eq++) {
                extra.equipDisplayId[eq] = cdie->getUInt32(i, 8 + eq);
            }
            extra.bakeName = cdie->getString(i, 20);
            if (!extra.bakeName.empty()) withBakeName++;
            humanoidExtraMap_[cdie->getUInt32(i, 0)] = extra;
        }
        LOG_INFO("Loaded ", humanoidExtraMap_.size(), " humanoid display extra entries (", withBakeName, " with baked textures)");
    }

    // CreatureModelData.dbc: modelId (col 0) → modelPath (col 2, .mdx → .m2)
    if (auto cmd = assetManager->loadDBC("CreatureModelData.dbc"); cmd && cmd->isLoaded()) {
        for (uint32_t i = 0; i < cmd->getRecordCount(); i++) {
            std::string mdx = cmd->getString(i, 2);
            if (mdx.empty()) continue;
            // Convert .mdx to .m2
            if (mdx.size() >= 4) {
                mdx = mdx.substr(0, mdx.size() - 4) + ".m2";
            }
            modelIdToPath_[cmd->getUInt32(i, 0)] = mdx;
        }
        LOG_INFO("Loaded ", modelIdToPath_.size(), " model→path mappings");
    }

    // Resolve gryphon/wyvern display IDs by exact model path so taxi mounts have textures.
    auto toLower = [](std::string s) {
        for (char& c : s) c = static_cast<char>(::tolower(c));
        return s;
    };
    auto normalizePath = [&](const std::string& p) {
        std::string s = p;
        for (char& c : s) if (c == '/') c = '\\';
        return toLower(s);
    };
    auto resolveDisplayIdForExactPath = [&](const std::string& exactPath) -> uint32_t {
        const std::string target = normalizePath(exactPath);
        uint32_t modelId = 0;
        for (const auto& [mid, path] : modelIdToPath_) {
            if (normalizePath(path) == target) {
                modelId = mid;
                break;
            }
        }
        if (modelId == 0) return 0;
        uint32_t bestDisplayId = 0;
        int bestScore = -1;
        for (const auto& [dispId, data] : displayDataMap_) {
            if (data.modelId != modelId) continue;
            int score = 0;
            if (!data.skin1.empty()) score += 3;
            if (!data.skin2.empty()) score += 2;
            if (!data.skin3.empty()) score += 1;
            if (score > bestScore) {
                bestScore = score;
                bestDisplayId = dispId;
            }
        }
        return bestDisplayId;
    };

    gryphonDisplayId_ = resolveDisplayIdForExactPath("Creature\\Gryphon\\Gryphon.m2");
    wyvernDisplayId_  = resolveDisplayIdForExactPath("Creature\\Wyvern\\Wyvern.m2");
    LOG_INFO("Taxi mount displayIds: gryphon=", gryphonDisplayId_, " wyvern=", wyvernDisplayId_);

    // CharHairGeosets.dbc: maps (race, sex, hairStyleId) → skinSectionId for hair mesh
    // Col 0: ID, Col 1: RaceID, Col 2: SexID, Col 3: VariationID, Col 4: GeosetID, Col 5: Showscalp
    if (auto chg = assetManager->loadDBC("CharHairGeosets.dbc"); chg && chg->isLoaded()) {
        for (uint32_t i = 0; i < chg->getRecordCount(); i++) {
            uint32_t raceId = chg->getUInt32(i, 1);
            uint32_t sexId = chg->getUInt32(i, 2);
            uint32_t variation = chg->getUInt32(i, 3);
            uint32_t geosetId = chg->getUInt32(i, 4);
            uint32_t key = (raceId << 16) | (sexId << 8) | variation;
            hairGeosetMap_[key] = static_cast<uint16_t>(geosetId);
        }
        LOG_INFO("Loaded ", hairGeosetMap_.size(), " hair geoset mappings from CharHairGeosets.dbc");
        // Debug: dump Human Male (race=1, sex=0) hair geoset mappings
        for (uint32_t v = 0; v < 20; v++) {
            uint32_t k = (1u << 16) | (0u << 8) | v;
            auto it = hairGeosetMap_.find(k);
            if (it != hairGeosetMap_.end()) {
            }
        }
    }

    // CharacterFacialHairStyles.dbc: maps (race, sex, facialHairId) → geoset IDs
    // No ID column: Col 0: RaceID, Col 1: SexID, Col 2: VariationID
    // Col 3: Geoset100, Col 4: Geoset300, Col 5: Geoset200
    if (auto cfh = assetManager->loadDBC("CharacterFacialHairStyles.dbc"); cfh && cfh->isLoaded()) {
        for (uint32_t i = 0; i < cfh->getRecordCount(); i++) {
            uint32_t raceId = cfh->getUInt32(i, 0);
            uint32_t sexId = cfh->getUInt32(i, 1);
            uint32_t variation = cfh->getUInt32(i, 2);
            uint32_t key = (raceId << 16) | (sexId << 8) | variation;
            FacialHairGeosets fhg;
            fhg.geoset100 = static_cast<uint16_t>(cfh->getUInt32(i, 3));
            fhg.geoset300 = static_cast<uint16_t>(cfh->getUInt32(i, 4));
            fhg.geoset200 = static_cast<uint16_t>(cfh->getUInt32(i, 5));
            facialHairGeosetMap_[key] = fhg;
        }
        LOG_INFO("Loaded ", facialHairGeosetMap_.size(), " facial hair geoset mappings from CharacterFacialHairStyles.dbc");
    }

    creatureLookupsBuilt_ = true;
}

std::string Application::getModelPathForDisplayId(uint32_t displayId) const {
    if (displayId == 30412) return "Creature\\Gryphon\\Gryphon.m2";
    if (displayId == 30413) return "Creature\\Wyvern\\Wyvern.m2";
    auto itData = displayDataMap_.find(displayId);
    if (itData == displayDataMap_.end()) {
        // Some sources (e.g., taxi nodes) may provide a modelId directly.
        auto itPath = modelIdToPath_.find(displayId);
        if (itPath != modelIdToPath_.end()) {
            return itPath->second;
        }
        if (displayId == 30412) return "Creature\\Gryphon\\Gryphon.m2";
        if (displayId == 30413) return "Creature\\Wyvern\\Wyvern.m2";
        return "";
    }

    auto itPath = modelIdToPath_.find(itData->second.modelId);
    if (itPath == modelIdToPath_.end()) return "";

    return itPath->second;
}

audio::VoiceType Application::detectVoiceTypeFromDisplayId(uint32_t displayId) const {
    // Look up display data
    auto itDisplay = displayDataMap_.find(displayId);
    if (itDisplay == displayDataMap_.end() || itDisplay->second.extraDisplayId == 0) {
        LOG_INFO("Voice detection: displayId ", displayId, " -> GENERIC (no display data)");
        return audio::VoiceType::GENERIC;  // Not a humanoid or no extra data
    }

    // Look up humanoid extra data (race/sex info)
    auto itExtra = humanoidExtraMap_.find(itDisplay->second.extraDisplayId);
    if (itExtra == humanoidExtraMap_.end()) {
        LOG_INFO("Voice detection: displayId ", displayId, " -> GENERIC (no humanoid extra data)");
        return audio::VoiceType::GENERIC;
    }

    uint8_t raceId = itExtra->second.raceId;
    uint8_t sexId = itExtra->second.sexId;

    const char* raceName = "Unknown";
    const char* sexName = (sexId == 0) ? "Male" : "Female";

    // Map (raceId, sexId) to VoiceType
    // Race IDs: 1=Human, 2=Orc, 3=Dwarf, 4=NightElf, 5=Undead, 6=Tauren, 7=Gnome, 8=Troll
    // Sex IDs: 0=Male, 1=Female
    audio::VoiceType result;
    switch (raceId) {
        case 1: raceName = "Human"; result = (sexId == 0) ? audio::VoiceType::HUMAN_MALE : audio::VoiceType::HUMAN_FEMALE; break;
        case 2: raceName = "Orc"; result = (sexId == 0) ? audio::VoiceType::ORC_MALE : audio::VoiceType::ORC_FEMALE; break;
        case 3: raceName = "Dwarf"; result = (sexId == 0) ? audio::VoiceType::DWARF_MALE : audio::VoiceType::GENERIC; break;
        case 4: raceName = "NightElf"; result = (sexId == 0) ? audio::VoiceType::NIGHTELF_MALE : audio::VoiceType::NIGHTELF_FEMALE; break;
        case 5: raceName = "Undead"; result = (sexId == 0) ? audio::VoiceType::UNDEAD_MALE : audio::VoiceType::UNDEAD_FEMALE; break;
        case 6: raceName = "Tauren"; result = (sexId == 0) ? audio::VoiceType::TAUREN_MALE : audio::VoiceType::TAUREN_FEMALE; break;
        case 7: raceName = "Gnome"; result = (sexId == 0) ? audio::VoiceType::GNOME_MALE : audio::VoiceType::GNOME_FEMALE; break;
        case 8: raceName = "Troll"; result = (sexId == 0) ? audio::VoiceType::TROLL_MALE : audio::VoiceType::TROLL_FEMALE; break;
        default: result = audio::VoiceType::GENERIC; break;
    }

    LOG_INFO("Voice detection: displayId ", displayId, " -> ", raceName, " ", sexName, " (race=", (int)raceId, ", sex=", (int)sexId, ")");
    return result;
}

void Application::buildGameObjectDisplayLookups() {
    if (gameObjectLookupsBuilt_ || !assetManager || !assetManager->isInitialized()) return;

    LOG_INFO("Building gameobject display lookups from DBC files");

    // GameObjectDisplayInfo.dbc structure (3.3.5a):
    // Col 0: ID (displayId)
    // Col 1: ModelName
    if (auto godi = assetManager->loadDBC("GameObjectDisplayInfo.dbc"); godi && godi->isLoaded()) {
        for (uint32_t i = 0; i < godi->getRecordCount(); i++) {
            uint32_t displayId = godi->getUInt32(i, 0);
            std::string modelName = godi->getString(i, 1);
            if (modelName.empty()) continue;
            if (modelName.size() >= 4) {
                std::string ext = modelName.substr(modelName.size() - 4);
                for (char& c : ext) c = static_cast<char>(std::tolower(c));
                if (ext == ".mdx") {
                    modelName = modelName.substr(0, modelName.size() - 4) + ".m2";
                }
            }
            gameObjectDisplayIdToPath_[displayId] = modelName;
        }
        LOG_INFO("Loaded ", gameObjectDisplayIdToPath_.size(), " gameobject display mappings");
    }

    gameObjectLookupsBuilt_ = true;
}

std::string Application::getGameObjectModelPathForDisplayId(uint32_t displayId) const {
    auto it = gameObjectDisplayIdToPath_.find(displayId);
    if (it == gameObjectDisplayIdToPath_.end()) return "";
    return it->second;
}

bool Application::getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const {
    if (!renderer || !renderer->getCharacterRenderer()) return false;
    uint32_t instanceId = 0;

    if (gameHandler && guid == gameHandler->getPlayerGuid()) {
        instanceId = renderer->getCharacterInstanceId();
    }
    if (instanceId == 0) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end()) instanceId = it->second;
    }
    if (instanceId == 0 && npcManager) {
        instanceId = npcManager->findRenderInstanceId(guid);
    }
    if (instanceId == 0) return false;

    return renderer->getCharacterRenderer()->getInstanceBounds(instanceId, outCenter, outRadius);
}

void Application::spawnOnlineCreature(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
    if (!renderer || !renderer->getCharacterRenderer() || !assetManager) return;

    // Skip if lookups not yet built (asset manager not ready)
    if (!creatureLookupsBuilt_) return;

    // Skip if already spawned
    if (creatureInstances_.count(guid)) return;

    // Get model path from displayId
    std::string m2Path = getModelPathForDisplayId(displayId);
    if (m2Path.empty()) {
        LOG_WARNING("No model path for displayId ", displayId, " (guid 0x", std::hex, guid, std::dec, ")");
        return;
    }

    auto* charRenderer = renderer->getCharacterRenderer();

    // Check model cache - reuse if same displayId was already loaded
    uint32_t modelId = 0;
    bool modelCached = false;
    auto cacheIt = displayIdModelCache_.find(displayId);
    if (cacheIt != displayIdModelCache_.end()) {
        modelId = cacheIt->second;
        modelCached = true;
    } else {
        // Load model from disk (only once per displayId)
        modelId = nextCreatureModelId_++;

        auto m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            LOG_WARNING("Failed to read creature M2: ", m2Path);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.vertices.empty()) {
            LOG_WARNING("Failed to parse creature M2: ", m2Path);
            return;
        }

        // Load skin file
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty()) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        // Load external .anim files for sequences without flag 0x20
        std::string basePath = m2Path.substr(0, m2Path.size() - 3);
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName), "%s%04u-%02u.anim",
                    basePath.c_str(), model.sequences[si].id, model.sequences[si].variationIndex);
                auto animData = assetManager->readFile(animFileName);
                if (!animData.empty()) {
                    pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
                }
            }
        }

        if (!charRenderer->loadModel(model, modelId)) {
            LOG_WARNING("Failed to load creature model: ", m2Path);
            return;
        }

        displayIdModelCache_[displayId] = modelId;
    }

    // Apply skin textures from CreatureDisplayInfo.dbc (only for newly loaded models)
    auto itDisplayData = displayDataMap_.find(displayId);
    if (!modelCached && itDisplayData != displayDataMap_.end()) {
        const auto& dispData = itDisplayData->second;

        // Get model directory for texture path construction
        std::string modelDir;
        size_t lastSlash = m2Path.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            modelDir = m2Path.substr(0, lastSlash + 1);
        }

        LOG_DEBUG("DisplayId ", displayId, " skins: '", dispData.skin1, "', '", dispData.skin2, "', '", dispData.skin3,
                  "' extraDisplayId=", dispData.extraDisplayId);

        // Get model data from CharacterRenderer for texture iteration
        const auto* modelData = charRenderer->getModelData(modelId);
        if (!modelData) {
            LOG_WARNING("Model data not found for modelId ", modelId);
        }

        // Log texture types in the model
        if (modelData) {
        for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
            LOG_DEBUG("  Model texture ", ti, ": type=", modelData->textures[ti].type, " filename='", modelData->textures[ti].filename, "'");
        }
        }

        // Check if this is a humanoid NPC with extra display info
        bool hasHumanoidTexture = false;
        if (dispData.extraDisplayId != 0) {
            auto itExtra = humanoidExtraMap_.find(dispData.extraDisplayId);
            if (itExtra != humanoidExtraMap_.end()) {
                const auto& extra = itExtra->second;
                LOG_DEBUG("  Found humanoid extra: raceId=", (int)extra.raceId, " sexId=", (int)extra.sexId,
                          " hairStyle=", (int)extra.hairStyleId, " hairColor=", (int)extra.hairColorId,
                          " bakeName='", extra.bakeName, "'");
                LOG_DEBUG("  Equipment: helm=", extra.equipDisplayId[0], " shoulder=", extra.equipDisplayId[1],
                          " shirt=", extra.equipDisplayId[2], " chest=", extra.equipDisplayId[3],
                          " belt=", extra.equipDisplayId[4], " legs=", extra.equipDisplayId[5],
                          " feet=", extra.equipDisplayId[6], " wrist=", extra.equipDisplayId[7],
                          " hands=", extra.equipDisplayId[8], " tabard=", extra.equipDisplayId[9],
                          " cape=", extra.equipDisplayId[10]);

                // Use baked texture for body skin only (types 1, 2)
                // Type 6 (hair) needs its own texture from CharSections.dbc
                if (!extra.bakeName.empty()) {
                    std::string bakePath = "Textures\\BakedNpcTextures\\" + extra.bakeName;
                    GLuint finalTex = charRenderer->loadTexture(bakePath);

                    if (finalTex != 0 && modelData) {
                        for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                            uint32_t texType = modelData->textures[ti].type;
                            if (texType == 1 || texType == 2) {
                                charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), finalTex);
                                LOG_DEBUG("Applied baked NPC texture to slot ", ti, " (type ", texType, "): ", bakePath);
                                hasHumanoidTexture = true;
                            }
                        }
                    } else {
                        LOG_WARNING("Failed to load baked NPC texture: ", bakePath);
                    }
                } else {
                    LOG_DEBUG("  Humanoid extra has empty bakeName, trying CharSections fallback");
                }

                // Load hair texture from CharSections.dbc (section 3)
                auto charSectionsDbc = assetManager->loadDBC("CharSections.dbc");
                if (charSectionsDbc) {
                    uint32_t targetRace = static_cast<uint32_t>(extra.raceId);
                    uint32_t targetSex = static_cast<uint32_t>(extra.sexId);
                    std::string hairTexPath;

                    for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                        uint32_t raceId = charSectionsDbc->getUInt32(r, 1);
                        uint32_t sexId = charSectionsDbc->getUInt32(r, 2);
                        uint32_t section = charSectionsDbc->getUInt32(r, 3);
                        uint32_t variation = charSectionsDbc->getUInt32(r, 8);
                        uint32_t colorIdx = charSectionsDbc->getUInt32(r, 9);

                        if (raceId != targetRace || sexId != targetSex) continue;
                        if (section != 3) continue;  // Section 3 = hair
                        if (variation != static_cast<uint32_t>(extra.hairStyleId)) continue;
                        if (colorIdx != static_cast<uint32_t>(extra.hairColorId)) continue;

                        hairTexPath = charSectionsDbc->getString(r, 4);
                        break;
                    }

                    if (!hairTexPath.empty()) {
                        GLuint hairTex = charRenderer->loadTexture(hairTexPath);
                        if (hairTex != 0 && modelData) {
                            for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                                if (modelData->textures[ti].type == 6) {
                                    charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), hairTex);
                                    LOG_DEBUG("Applied hair texture to slot ", ti, ": ", hairTexPath);
                                }
                            }
                        }
                    }
                }
            } else {
                LOG_WARNING("  extraDisplayId ", dispData.extraDisplayId, " not found in humanoidExtraMap");
            }
        }

        // Apply creature skin textures (for non-humanoid creatures)
        if (!hasHumanoidTexture && modelData) {
            for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                const auto& tex = modelData->textures[ti];
                std::string skinPath;

                // Creature skin types: 11 = skin1, 12 = skin2, 13 = skin3
                if (tex.type == 11 && !dispData.skin1.empty()) {
                    skinPath = modelDir + dispData.skin1 + ".blp";
                } else if (tex.type == 12 && !dispData.skin2.empty()) {
                    skinPath = modelDir + dispData.skin2 + ".blp";
                } else if (tex.type == 13 && !dispData.skin3.empty()) {
                    skinPath = modelDir + dispData.skin3 + ".blp";
                }

                if (!skinPath.empty()) {
                    GLuint skinTex = charRenderer->loadTexture(skinPath);
                    if (skinTex != 0) {
                        charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                        LOG_DEBUG("Applied creature skin texture: ", skinPath, " to slot ", ti);
                    }
                }
            }
        }
    }

    // Convert canonical → render coordinates
    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));

    // Convert canonical WoW orientation (0=north) -> render yaw (0=west)
    float renderYaw = orientation + glm::radians(90.0f);

    // Create instance
    uint32_t instanceId = charRenderer->createInstance(modelId, renderPos,
        glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);

    if (instanceId == 0) {
        LOG_WARNING("Failed to create creature instance for guid 0x", std::hex, guid, std::dec);
        return;
    }

    // Set geosets for humanoid NPCs based on CreatureDisplayInfoExtra
    if (itDisplayData != displayDataMap_.end() && itDisplayData->second.extraDisplayId != 0) {
        auto itExtra = humanoidExtraMap_.find(itDisplayData->second.extraDisplayId);
        if (itExtra != humanoidExtraMap_.end()) {
            const auto& extra = itExtra->second;
            std::unordered_set<uint16_t> activeGeosets;

            // Group 0: body base (id=0 always) + hair scalp mesh from CharHairGeosets.dbc
            activeGeosets.insert(0);  // Body base mesh

            // Hair: CharHairGeosets.dbc maps (race, sex, hairStyleId) → group 0 scalp submeshId
            uint32_t hairKey = (static_cast<uint32_t>(extra.raceId) << 16) |
                               (static_cast<uint32_t>(extra.sexId) << 8) |
                               static_cast<uint32_t>(extra.hairStyleId);
            auto itHairGeo = hairGeosetMap_.find(hairKey);
            uint16_t hairScalpId = (itHairGeo != hairGeosetMap_.end()) ? itHairGeo->second : 0;
            if (hairScalpId > 0) {
                activeGeosets.insert(hairScalpId);                        // Group 0 scalp/hair mesh
                activeGeosets.insert(static_cast<uint16_t>(100 + hairScalpId)); // Group 1 connector (if exists)
            } else {
                // Bald (geosetId=0): body base has a hole at the crown, so include
                // submeshId=1 (bald scalp cap with body skin texture) to cover it.
                activeGeosets.insert(1);    // Group 0 bald scalp mesh
                activeGeosets.insert(101);  // Group 1 connector
            }
            uint16_t hairGeoset = (hairScalpId > 0) ? hairScalpId : 1;

            // Facial hair geosets from CharFacialHairStyles.dbc lookup
            uint32_t facialKey = (static_cast<uint32_t>(extra.raceId) << 16) |
                                 (static_cast<uint32_t>(extra.sexId) << 8) |
                                 static_cast<uint32_t>(extra.facialHairId);
            auto itFacial = facialHairGeosetMap_.find(facialKey);
            if (itFacial != facialHairGeosetMap_.end()) {
                const auto& fhg = itFacial->second;
                // DBC values are variation indices within each group; add group base
                activeGeosets.insert(static_cast<uint16_t>(100 + std::max(fhg.geoset100, (uint16_t)1)));
                activeGeosets.insert(static_cast<uint16_t>(300 + std::max(fhg.geoset300, (uint16_t)1)));
                activeGeosets.insert(static_cast<uint16_t>(200 + std::max(fhg.geoset200, (uint16_t)1)));
            } else {
                activeGeosets.insert(101); // Default group 1: no extra
                activeGeosets.insert(201); // Default group 2: no facial hair
                activeGeosets.insert(301); // Default group 3: no facial hair
            }

            // Default equipment geosets (bare/no armor)
            uint16_t geosetGloves = 301;   // Bare hands
            uint16_t geosetBoots = 401;    // Bare feet
            uint16_t geosetChest = 501;    // Bare chest
            uint16_t geosetPants = 1301;   // Bare legs
            uint16_t geosetCape = 1501;    // No cape
            uint16_t geosetTabard = 1201;  // No tabard

            // Load equipment geosets from ItemDisplayInfo.dbc
            // DBC columns: 7=GeosetGroup[0], 8=GeosetGroup[1], 9=GeosetGroup[2]
            auto itemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
            if (itemDisplayDbc) {
                // Equipment slots: 0=helm, 1=shoulder, 2=shirt, 3=chest, 4=belt, 5=legs, 6=feet, 7=wrist, 8=hands, 9=tabard, 10=cape

                // Helm (slot 0) - noted for helmet model attachment below

                // Chest (slot 3) - geoset group 5xx
                if (extra.equipDisplayId[3] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[3]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 7);
                        if (gg > 0) geosetChest = static_cast<uint16_t>(501 + gg);
                        // Robes: GeosetGroup[2] > 0 shows kilt legs
                        uint32_t gg3 = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 9);
                        if (gg3 > 0) geosetPants = static_cast<uint16_t>(1301 + gg3);
                    }
                }

                // Legs (slot 5) - geoset group 13xx
                if (extra.equipDisplayId[5] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[5]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 7);
                        if (gg > 0) geosetPants = static_cast<uint16_t>(1301 + gg);
                    }
                }

                // Feet (slot 6) - geoset group 4xx
                if (extra.equipDisplayId[6] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[6]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 7);
                        if (gg > 0) geosetBoots = static_cast<uint16_t>(401 + gg);
                    }
                }

                // Hands (slot 8) - geoset group 3xx
                if (extra.equipDisplayId[8] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[8]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 7);
                        if (gg > 0) geosetGloves = static_cast<uint16_t>(301 + gg);
                    }
                }

                // Tabard (slot 9) - geoset group 12xx
                if (extra.equipDisplayId[9] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[9]);
                    if (idx >= 0) {
                        geosetTabard = 1202;  // Show tabard mesh
                    }
                }

                // Cape (slot 10) - geoset group 15xx
                if (extra.equipDisplayId[10] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[10]);
                    if (idx >= 0) {
                        geosetCape = 1502;  // Show cloak mesh
                    }
                }
            }

            // Apply equipment geosets
            activeGeosets.insert(geosetGloves);
            activeGeosets.insert(geosetBoots);
            activeGeosets.insert(geosetChest);
            activeGeosets.insert(geosetPants);
            activeGeosets.insert(geosetCape);
            activeGeosets.insert(geosetTabard);
            activeGeosets.insert(701);  // Ears: default

            // Hide hair under helmets: replace style-specific scalp with bald scalp
            if (extra.equipDisplayId[0] != 0 && hairGeoset > 1) {
                activeGeosets.erase(hairGeoset);                              // Remove style scalp
                activeGeosets.erase(static_cast<uint16_t>(100 + hairGeoset)); // Remove style group 1
                activeGeosets.insert(1);    // Bald scalp cap (group 0)
                activeGeosets.insert(101);  // Default group 1 connector
            }

            // Log model's actual submesh IDs for debugging geoset mismatches
            if (auto* md = charRenderer->getModelData(modelId)) {
                std::string batchIds;
                for (const auto& b : md->batches) {
                    if (!batchIds.empty()) batchIds += ",";
                    batchIds += std::to_string(b.submeshId);
                }
                LOG_INFO("Model batches submeshIds: [", batchIds, "]");
            }

            // Log what geosets we're setting for debugging
            std::string geosetList;
            for (uint16_t g : activeGeosets) {
                if (!geosetList.empty()) geosetList += ",";
                geosetList += std::to_string(g);
            }
            LOG_INFO("NPC geosets for instance ", instanceId, ": [", geosetList, "]");
            charRenderer->setActiveGeosets(instanceId, activeGeosets);
            LOG_DEBUG("Set humanoid geosets: hair=", (int)hairGeoset,
                      " chest=", geosetChest, " pants=", geosetPants,
                      " boots=", geosetBoots, " gloves=", geosetGloves);

            // Load and attach helmet model if equipped
            if (extra.equipDisplayId[0] != 0 && itemDisplayDbc) {
                int32_t helmIdx = itemDisplayDbc->findRecordById(extra.equipDisplayId[0]);
                if (helmIdx >= 0) {
                    // Get helmet model name from ItemDisplayInfo.dbc (col 1 = LeftModel)
                    std::string helmModelName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), 1);
                    if (!helmModelName.empty()) {
                        // Convert .mdx to .m2
                        size_t dotPos = helmModelName.rfind('.');
                        if (dotPos != std::string::npos) {
                            helmModelName = helmModelName.substr(0, dotPos);
                        }

                        // WoW helmet M2 files have per-race/gender variants with a suffix
                        // e.g. Helm_Plate_B_01Stormwind_HuM.M2 for Human Male
                        // ChrRaces.dbc ClientPrefix values (raceId → prefix):
                        static const std::unordered_map<uint8_t, std::string> racePrefix = {
                            {1, "Hu"}, {2, "Or"}, {3, "Dw"}, {4, "Ni"}, {5, "Sc"},
                            {6, "Ta"}, {7, "Gn"}, {8, "Tr"}, {10, "Be"}, {11, "Dr"}
                        };
                        std::string genderSuffix = (extra.sexId == 0) ? "M" : "F";
                        std::string raceSuffix;
                        auto itRace = racePrefix.find(extra.raceId);
                        if (itRace != racePrefix.end()) {
                            raceSuffix = "_" + itRace->second + genderSuffix;
                        }

                        // Try race/gender-specific variant first, then base name
                        std::string helmPath;
                        std::vector<uint8_t> helmData;
                        if (!raceSuffix.empty()) {
                            helmPath = "Item\\ObjectComponents\\Head\\" + helmModelName + raceSuffix + ".m2";
                            helmData = assetManager->readFile(helmPath);
                        }
                        if (helmData.empty()) {
                            helmPath = "Item\\ObjectComponents\\Head\\" + helmModelName + ".m2";
                            helmData = assetManager->readFile(helmPath);
                        }

                        if (!helmData.empty()) {
                            auto helmModel = pipeline::M2Loader::load(helmData);
                            // Load skin
                            std::string skinPath = helmPath.substr(0, helmPath.size() - 3) + "00.skin";
                            auto skinData = assetManager->readFile(skinPath);
                            if (!skinData.empty()) {
                                pipeline::M2Loader::loadSkin(skinData, helmModel);
                            }

                            if (helmModel.isValid()) {
                                // Attachment point 11 = Head
                                uint32_t helmModelId = nextCreatureModelId_++;
                                // Get texture from ItemDisplayInfo (col 3 = LeftModelTexture)
                                std::string helmTexName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), 3);
                                std::string helmTexPath;
                                if (!helmTexName.empty()) {
                                    // Try race/gender suffixed texture first
                                    if (!raceSuffix.empty()) {
                                        std::string suffixedTex = "Item\\ObjectComponents\\Head\\" + helmTexName + raceSuffix + ".blp";
                                        if (assetManager->fileExists(suffixedTex)) {
                                            helmTexPath = suffixedTex;
                                        }
                                    }
                                    if (helmTexPath.empty()) {
                                        helmTexPath = "Item\\ObjectComponents\\Head\\" + helmTexName + ".blp";
                                    }
                                }
                                charRenderer->attachWeapon(instanceId, 11, helmModel, helmModelId, helmTexPath);
                                LOG_DEBUG("Attached helmet model: ", helmPath, " tex: ", helmTexPath);
                            }
                        }
                    }
                }
            }
        }
    }

    // Play idle animation and fade in
    charRenderer->playAnimation(instanceId, 0, true);
    charRenderer->startFadeIn(instanceId, 0.5f);

    // Track instance
    creatureInstances_[guid] = instanceId;
    creatureModelIds_[guid] = modelId;

    LOG_INFO("Spawned creature: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
}

void Application::spawnOnlineGameObject(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
    if (!renderer || !assetManager) return;

    if (!gameObjectLookupsBuilt_) {
        buildGameObjectDisplayLookups();
    }
    if (!gameObjectLookupsBuilt_) return;

    if (gameObjectInstances_.count(guid)) {
        // Already have a render instance — update its position (e.g. transport re-creation)
        auto& info = gameObjectInstances_[guid];
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        LOG_INFO("GameObject position update: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
                 " pos=(", x, ", ", y, ", ", z, ")");
        if (renderer) {
            if (info.isWmo) {
                if (auto* wr = renderer->getWMORenderer())
                    wr->setInstancePosition(info.instanceId, renderPos);
            } else {
                if (auto* mr = renderer->getM2Renderer())
                    mr->setInstancePosition(info.instanceId, renderPos);
            }
        }
        return;
    }

    std::string modelPath = getGameObjectModelPathForDisplayId(displayId);
    if (modelPath.empty()) {
        LOG_WARNING("No model path for gameobject displayId ", displayId, " (guid 0x", std::hex, guid, std::dec, ")");
        return;
    }

    // Log spawns to help debug duplicate objects (e.g., cathedral issue)
    LOG_INFO("GameObject spawn: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
             " model=", modelPath, " pos=(", x, ", ", y, ", ", z, ")");

    std::string lowerPath = modelPath;
    std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool isWmo = lowerPath.size() >= 4 && lowerPath.substr(lowerPath.size() - 4) == ".wmo";

    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
    float renderYaw = orientation;

    bool loadedAsWmo = false;
    if (isWmo) {
        auto* wmoRenderer = renderer->getWMORenderer();
        if (!wmoRenderer) return;

        uint32_t modelId = 0;
        auto itCache = gameObjectDisplayIdWmoCache_.find(displayId);
        if (itCache != gameObjectDisplayIdWmoCache_.end()) {
            modelId = itCache->second;
            loadedAsWmo = true;
        } else {
            auto wmoData = assetManager->readFile(modelPath);
            if (!wmoData.empty()) {
                pipeline::WMOModel wmoModel = pipeline::WMOLoader::load(wmoData);
                LOG_INFO("Gameobject WMO root loaded: ", modelPath, " nGroups=", wmoModel.nGroups);
                int loadedGroups = 0;
                if (wmoModel.nGroups > 0) {
                    std::string basePath = modelPath;
                    std::string extension;
                    if (basePath.size() > 4) {
                        extension = basePath.substr(basePath.size() - 4);
                        std::string extLower = extension;
                        for (char& c : extLower) c = static_cast<char>(std::tolower(c));
                        if (extLower == ".wmo") {
                            basePath = basePath.substr(0, basePath.size() - 4);
                        }
                    }

                    for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                        char groupSuffix[16];
                        snprintf(groupSuffix, sizeof(groupSuffix), "_%03u%s", gi, extension.c_str());
                        std::string groupPath = basePath + groupSuffix;
                        std::vector<uint8_t> groupData = assetManager->readFile(groupPath);
                        if (groupData.empty()) {
                            snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.wmo", gi);
                            groupData = assetManager->readFile(basePath + groupSuffix);
                        }
                        if (groupData.empty()) {
                            snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.WMO", gi);
                            groupData = assetManager->readFile(basePath + groupSuffix);
                        }
                        if (!groupData.empty()) {
                            pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                            loadedGroups++;
                        } else {
                            LOG_WARNING("  Failed to load WMO group ", gi, " for: ", basePath);
                        }
                    }
                }

                if (loadedGroups > 0 || wmoModel.nGroups == 0) {
                    modelId = nextGameObjectWmoModelId_++;
                    if (wmoRenderer->loadModel(wmoModel, modelId)) {
                        gameObjectDisplayIdWmoCache_[displayId] = modelId;
                        loadedAsWmo = true;
                    } else {
                        LOG_WARNING("Failed to load gameobject WMO model: ", modelPath);
                    }
                } else {
                    LOG_WARNING("No WMO groups loaded for gameobject: ", modelPath,
                                " — falling back to M2");
                }
            } else {
                LOG_WARNING("Failed to read gameobject WMO: ", modelPath, " — falling back to M2");
            }
        }

        if (loadedAsWmo) {
            uint32_t instanceId = wmoRenderer->createInstance(modelId, renderPos,
                glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);
            if (instanceId == 0) {
                LOG_WARNING("Failed to create gameobject WMO instance for guid 0x", std::hex, guid, std::dec);
                return;
            }

            gameObjectInstances_[guid] = {modelId, instanceId, true};
            LOG_INFO("Spawned gameobject WMO: guid=0x", std::hex, guid, std::dec,
                     " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
            return;
        }

        // WMO failed — fall through to try as M2
        // Convert .wmo path to .m2 for fallback
        modelPath = modelPath.substr(0, modelPath.size() - 4) + ".m2";
    }

    {
        auto* m2Renderer = renderer->getM2Renderer();
        if (!m2Renderer) return;

        uint32_t modelId = 0;
        auto itCache = gameObjectDisplayIdModelCache_.find(displayId);
        if (itCache != gameObjectDisplayIdModelCache_.end()) {
            modelId = itCache->second;
        } else {
            modelId = nextGameObjectModelId_++;

            auto m2Data = assetManager->readFile(modelPath);
            if (m2Data.empty()) {
                LOG_WARNING("Failed to read gameobject M2: ", modelPath);
                return;
            }

            pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
            if (model.vertices.empty()) {
                LOG_WARNING("Failed to parse gameobject M2: ", modelPath);
                return;
            }

            std::string skinPath = modelPath.substr(0, modelPath.size() - 3) + "00.skin";
            auto skinData = assetManager->readFile(skinPath);
            if (!skinData.empty()) {
                pipeline::M2Loader::loadSkin(skinData, model);
            }

            if (!m2Renderer->loadModel(model, modelId)) {
                LOG_WARNING("Failed to load gameobject model: ", modelPath);
                return;
            }

            gameObjectDisplayIdModelCache_[displayId] = modelId;
        }

        uint32_t instanceId = m2Renderer->createInstance(modelId, renderPos,
            glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);
        if (instanceId == 0) {
            LOG_WARNING("Failed to create gameobject instance for guid 0x", std::hex, guid, std::dec);
            return;
        }

        gameObjectInstances_[guid] = {modelId, instanceId, false};
    }

    LOG_INFO("Spawned gameobject: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
}

void Application::processCreatureSpawnQueue() {
    if (pendingCreatureSpawns_.empty()) return;

    int spawned = 0;
    while (!pendingCreatureSpawns_.empty() && spawned < MAX_SPAWNS_PER_FRAME) {
        auto& s = pendingCreatureSpawns_.front();
        spawnOnlineCreature(s.guid, s.displayId, s.x, s.y, s.z, s.orientation);
        pendingCreatureSpawns_.erase(pendingCreatureSpawns_.begin());
        spawned++;
    }
}

void Application::processGameObjectSpawnQueue() {
    if (pendingGameObjectSpawns_.empty()) return;

    int spawned = 0;
    while (!pendingGameObjectSpawns_.empty() && spawned < MAX_SPAWNS_PER_FRAME) {
        auto& s = pendingGameObjectSpawns_.front();
        spawnOnlineGameObject(s.guid, s.displayId, s.x, s.y, s.z, s.orientation);
        pendingGameObjectSpawns_.erase(pendingGameObjectSpawns_.begin());
        spawned++;
    }
}

void Application::processPendingMount() {
    if (pendingMountDisplayId_ == 0) return;
    uint32_t mountDisplayId = pendingMountDisplayId_;
    pendingMountDisplayId_ = 0;
    LOG_INFO("processPendingMount: loading displayId ", mountDisplayId);

    if (!renderer || !renderer->getCharacterRenderer() || !assetManager) return;
    auto* charRenderer = renderer->getCharacterRenderer();

    std::string m2Path = getModelPathForDisplayId(mountDisplayId);
    if (m2Path.empty()) {
        LOG_WARNING("No model path for mount displayId ", mountDisplayId);
        return;
    }

    // Check model cache
    uint32_t modelId = 0;
    bool modelCached = false;
    auto cacheIt = displayIdModelCache_.find(mountDisplayId);
    if (cacheIt != displayIdModelCache_.end()) {
        modelId = cacheIt->second;
        modelCached = true;
    } else {
        modelId = nextCreatureModelId_++;

        auto m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            LOG_WARNING("Failed to read mount M2: ", m2Path);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.vertices.empty()) {
            LOG_WARNING("Failed to parse mount M2: ", m2Path);
            return;
        }

        // Load skin file
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty()) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        // Load external .anim files (only idle + run needed for mounts)
        std::string basePath = m2Path.substr(0, m2Path.size() - 3);
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                uint32_t animId = model.sequences[si].id;
                // Only load stand(0), walk(4), run(5) anims to avoid hang
                if (animId != 0 && animId != 4 && animId != 5) continue;
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName), "%s%04u-%02u.anim",
                    basePath.c_str(), animId, model.sequences[si].variationIndex);
                auto animData = assetManager->readFile(animFileName);
                if (!animData.empty()) {
                    pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
                }
            }
        }

        if (!charRenderer->loadModel(model, modelId)) {
            LOG_WARNING("Failed to load mount model: ", m2Path);
            return;
        }

        displayIdModelCache_[mountDisplayId] = modelId;
    }

    // Apply creature skin textures from CreatureDisplayInfo.dbc
    if (!modelCached) {
        auto itDisplayData = displayDataMap_.find(mountDisplayId);
        if (itDisplayData != displayDataMap_.end()) {
            CreatureDisplayData dispData = itDisplayData->second;
            // If this displayId has no skins, try to find another displayId for the same model with skins.
            if (dispData.skin1.empty() && dispData.skin2.empty() && dispData.skin3.empty()) {
                uint32_t modelId = dispData.modelId;
                int bestScore = -1;
                for (const auto& [dispId, data] : displayDataMap_) {
                    if (data.modelId != modelId) continue;
                    int score = 0;
                    if (!data.skin1.empty()) score += 3;
                    if (!data.skin2.empty()) score += 2;
                    if (!data.skin3.empty()) score += 1;
                    if (score > bestScore) {
                        bestScore = score;
                        dispData = data;
                    }
                }
                LOG_INFO("Mount skin fallback for displayId=", mountDisplayId,
                         " modelId=", modelId, " skin1='", dispData.skin1,
                         "' skin2='", dispData.skin2, "' skin3='", dispData.skin3, "'");
            }
            const auto* md = charRenderer->getModelData(modelId);
            if (md) {
                std::string modelDir;
                size_t lastSlash = m2Path.find_last_of("\\/");
                if (lastSlash != std::string::npos) {
                    modelDir = m2Path.substr(0, lastSlash + 1);
                }
                int replaced = 0;
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    const auto& tex = md->textures[ti];
                    std::string texPath;
                    if (tex.type == 11 && !dispData.skin1.empty()) {
                        texPath = modelDir + dispData.skin1 + ".blp";
                    } else if (tex.type == 12 && !dispData.skin2.empty()) {
                        texPath = modelDir + dispData.skin2 + ".blp";
                    } else if (tex.type == 13 && !dispData.skin3.empty()) {
                        texPath = modelDir + dispData.skin3 + ".blp";
                    }
                    if (!texPath.empty()) {
                        GLuint skinTex = charRenderer->loadTexture(texPath);
                        if (skinTex != 0) {
                            charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                            replaced++;
                        }
                    }
                }
                // Some mounts (gryphon/wyvern) use empty model textures; force skin1 onto slot 0.
                if (replaced == 0 && !dispData.skin1.empty() && !md->textures.empty()) {
                    std::string texPath = modelDir + dispData.skin1 + ".blp";
                    GLuint skinTex = charRenderer->loadTexture(texPath);
                    if (skinTex != 0) {
                        charRenderer->setModelTexture(modelId, 0, skinTex);
                        LOG_INFO("Forced mount skin1 texture on slot 0: ", texPath);
                    }
                } else if (replaced == 0 && !md->textures.empty() && !md->textures[0].filename.empty()) {
                    // Last-resort: use the model's first texture filename if it exists.
                    GLuint texId = charRenderer->loadTexture(md->textures[0].filename);
                    if (texId != 0) {
                        charRenderer->setModelTexture(modelId, 0, texId);
                        LOG_INFO("Forced mount model texture on slot 0: ", md->textures[0].filename);
                    }
                }
            }
        }
    }

    mountModelId_ = modelId;

    // Create mount instance at player position
    glm::vec3 mountPos = renderer->getCharacterPosition();
    float yawRad = glm::radians(renderer->getCharacterYaw());
    uint32_t instanceId = charRenderer->createInstance(modelId, mountPos,
        glm::vec3(0.0f, 0.0f, yawRad), 1.0f);

    if (instanceId == 0) {
        LOG_WARNING("Failed to create mount instance");
        return;
    }

    mountInstanceId_ = instanceId;

    // Compute height offset — place player above mount's back
    // Use tight bounds from actual vertices (M2 header bounds can be inaccurate)
    const auto* modelData = charRenderer->getModelData(modelId);
    float heightOffset = 1.8f;
    if (modelData && !modelData->vertices.empty()) {
        float minZ =  std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();
        for (const auto& v : modelData->vertices) {
            if (v.position.z < minZ) minZ = v.position.z;
            if (v.position.z > maxZ) maxZ = v.position.z;
        }
        float extentZ = maxZ - minZ;
        LOG_INFO("Mount tight bounds: minZ=", minZ, " maxZ=", maxZ, " extentZ=", extentZ);
        if (extentZ > 0.5f) {
            // Saddle point is roughly 75% up the model, measured from model origin
            heightOffset = maxZ * 0.8f;
            if (heightOffset < 1.0f) heightOffset = extentZ * 0.75f;
            if (heightOffset < 1.0f) heightOffset = 1.8f;
        }
    }

    renderer->setMounted(instanceId, mountDisplayId, heightOffset);
    charRenderer->playAnimation(instanceId, 0, true);

    LOG_INFO("processPendingMount: DONE displayId=", mountDisplayId, " model=", m2Path, " heightOffset=", heightOffset);
}

void Application::despawnOnlineCreature(uint64_t guid) {
    auto it = creatureInstances_.find(guid);
    if (it == creatureInstances_.end()) return;

    if (renderer && renderer->getCharacterRenderer()) {
        renderer->getCharacterRenderer()->removeInstance(it->second);
    }

    creatureInstances_.erase(it);
    creatureModelIds_.erase(guid);

    LOG_INFO("Despawned creature: guid=0x", std::hex, guid, std::dec);
}

void Application::despawnOnlineGameObject(uint64_t guid) {
    auto it = gameObjectInstances_.find(guid);
    if (it == gameObjectInstances_.end()) return;

    if (renderer) {
        if (it->second.isWmo) {
            if (auto* wmoRenderer = renderer->getWMORenderer()) {
                wmoRenderer->removeInstance(it->second.instanceId);
            }
        } else {
            if (auto* m2Renderer = renderer->getM2Renderer()) {
                m2Renderer->removeInstance(it->second.instanceId);
            }
        }
    }

    gameObjectInstances_.erase(it);

    LOG_INFO("Despawned gameobject: guid=0x", std::hex, guid, std::dec);
}

void Application::loadQuestMarkerModels() {
    if (!assetManager || !renderer) return;

    // Quest markers in WoW 3.3.5a are billboard sprites (BLP textures), not M2 models
    // Load the BLP textures for quest markers
    LOG_INFO("Quest markers will be rendered as billboard sprites using BLP textures:");
    LOG_INFO("  - Available: Interface\\GossipFrame\\AvailableQuestIcon.blp");
    LOG_INFO("  - Turn-in: Interface\\GossipFrame\\ActiveQuestIcon.blp");
    LOG_INFO("  - Incomplete: Interface\\GossipFrame\\IncompleteQuestIcon.blp");

    // TODO: Implement billboard sprite rendering for quest markers
    // For now, the 2D ImGui markers will continue to work
}

void Application::updateQuestMarkers() {
    if (!gameHandler || !renderer || questExclamationModelId_ == 0) {
        static bool logged = false;
        if (!logged) {
            LOG_INFO("updateQuestMarkers: skipped - gameHandler=", (gameHandler ? "yes" : "no"),
                     " renderer=", (renderer ? "yes" : "no"),
                     " questExclamationModelId=", questExclamationModelId_);
            logged = true;
        }
        return;
    }

    auto* m2Renderer = renderer->getM2Renderer();
    if (!m2Renderer) return;

    const auto& questStatuses = gameHandler->getNpcQuestStatuses();

    static int logCounter = 0;
    if (++logCounter % 300 == 0) {  // Log every ~10 seconds at 30fps
        LOG_INFO("Quest markers: ", questStatuses.size(), " NPCs with status, ",
                 questMarkerInstances_.size(), " markers active");
    }

    // Remove markers for NPCs that no longer have quest status
    std::vector<uint64_t> toRemove;
    for (const auto& [guid, instanceId] : questMarkerInstances_) {
        if (questStatuses.find(guid) == questStatuses.end()) {
            m2Renderer->removeInstance(instanceId);
            toRemove.push_back(guid);
        }
    }
    for (uint64_t guid : toRemove) {
        questMarkerInstances_.erase(guid);
    }

    // Update or create markers for NPCs with quest status
    for (const auto& [guid, status] : questStatuses) {
        // Determine which marker model to use
        uint32_t markerModelId = 0;
        bool shouldShow = false;

        using game::QuestGiverStatus;
        switch (status) {
            case QuestGiverStatus::AVAILABLE:
            case QuestGiverStatus::AVAILABLE_LOW:
                markerModelId = questExclamationModelId_;
                shouldShow = true;
                break;
            case QuestGiverStatus::REWARD:
                markerModelId = questQuestionMarkModelId_;
                shouldShow = true;
                break;
            case QuestGiverStatus::INCOMPLETE:
                // Gray ? - for now just use regular ? (could load yellow variant later)
                markerModelId = questQuestionMarkModelId_;
                shouldShow = false;  // Don't show incomplete markers
                break;
            default:
                shouldShow = false;
                break;
        }

        // Get NPC entity position
        auto entity = gameHandler->getEntityManager().getEntity(guid);
        if (!entity) continue;

        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
        glm::vec3 renderPos = coords::canonicalToRender(canonical);

        // Offset marker above NPC head
        glm::vec3 boundsCenter;
        float boundsRadius = 0.0f;
        float heightOffset = 3.0f;
        if (getRenderBoundsForGuid(guid, boundsCenter, boundsRadius)) {
            heightOffset = boundsRadius * 2.0f + 1.0f;
        }
        renderPos.z += heightOffset;

        if (shouldShow && markerModelId != 0) {
            // Check if marker already exists
            auto it = questMarkerInstances_.find(guid);
            if (it != questMarkerInstances_.end()) {
                // Update existing marker position
                m2Renderer->setInstancePosition(it->second, renderPos);
            } else {
                // Create new marker instance (billboarded, no rotation needed)
                uint32_t instanceId = m2Renderer->createInstance(
                    markerModelId, renderPos, glm::vec3(0.0f), 1.0f);
                questMarkerInstances_[guid] = instanceId;
            }
        } else {
            // Remove marker if it exists but shouldn't show
            auto it = questMarkerInstances_.find(guid);
            if (it != questMarkerInstances_.end()) {
                m2Renderer->removeInstance(it->second);
                questMarkerInstances_.erase(it);
            }
        }
    }
}

} // namespace core
} // namespace wowee
