#include "core/application.hpp"
#include "core/coordinates.hpp"
#include "core/spawn_presets.hpp"
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
#include <optional>
#include <sstream>
#include <set>

namespace wowee {
namespace core {

namespace {

const SpawnPreset* selectSpawnPreset(const char* envValue) {
    // Return nullptr if no preset specified - use saved character position
    if (!envValue || !*envValue) {
        return nullptr;
    }

    std::string key = envValue;
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    for (int i = 0; i < SPAWN_PRESET_COUNT; i++) {
        if (key == SPAWN_PRESETS[i].key) return &SPAWN_PRESETS[i];
    }

    LOG_WARNING("Unknown WOW_SPAWN='", key, "', falling back to goldshire");
    LOG_INFO("Available WOW_SPAWN presets: goldshire, stormwind, sw_plaza, ironforge, westfall");
    return &SPAWN_PRESETS[0];
}

} // namespace

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
    return game::getPlayerModelPath(spRace_, spGender_);
}

namespace {

std::optional<glm::vec3> parseVec3Csv(const char* raw) {
    if (!raw || !*raw) return std::nullopt;
    std::stringstream ss(raw);
    std::string part;
    float vals[3];
    for (int i = 0; i < 3; i++) {
        if (!std::getline(ss, part, ',')) return std::nullopt;
        try {
            vals[i] = std::stof(part);
        } catch (...) {
            return std::nullopt;
        }
    }
    return glm::vec3(vals[0], vals[1], vals[2]);
}

std::optional<std::pair<float, float>> parseYawPitchCsv(const char* raw) {
    if (!raw || !*raw) return std::nullopt;
    std::stringstream ss(raw);
    std::string part;
    float yaw = 0.0f, pitch = 0.0f;
    if (!std::getline(ss, part, ',')) return std::nullopt;
    try { yaw = std::stof(part); } catch (...) { return std::nullopt; }
    if (!std::getline(ss, part, ',')) return std::nullopt;
    try { pitch = std::stof(part); } catch (...) { return std::nullopt; }
    return std::make_pair(yaw, pitch);
}

} // namespace

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

    // Terrain and character are loaded via startSinglePlayer() when the user
    // picks single-player mode, so nothing is preloaded here.

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
                // T: Toggle teleporter panel
                else if (event.key.keysym.scancode == SDL_SCANCODE_T) {
                    if (state == AppState::IN_GAME && uiManager) {
                        uiManager->getGameScreen().toggleTeleporter();
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
        case AppState::IN_GAME:
            // Wire up movement opcodes from camera controller
            if (renderer && renderer->getCameraController()) {
                auto* cc = renderer->getCameraController();
                cc->setMovementCallback([this](uint32_t opcode) {
                    if (gameHandler && !singlePlayerMode) {
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
            break;
        case AppState::DISCONNECTED:
            // Back to auth
            break;
    }
}

void Application::logoutToLogin() {
    LOG_INFO("Logout requested");
    if (gameHandler) {
        gameHandler->disconnect();
        gameHandler->setSinglePlayerMode(false);
    }
    singlePlayerMode = false;
    npcsSpawned = false;
    world.reset();
    if (renderer) {
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

        case AppState::IN_GAME:
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            if (world) {
                world->update(deltaTime);
            }
            // Spawn/update local single-player NPCs.
            if (!npcsSpawned && singlePlayerMode) {
                spawnNpcs();
            }
            if (npcManager && renderer && renderer->getCharacterRenderer()) {
                npcManager->update(deltaTime, renderer->getCharacterRenderer());
            }

            // Sync character render position → canonical WoW coords each frame
            if (renderer && gameHandler) {
                glm::vec3 renderPos = renderer->getCharacterPosition();
                glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
                gameHandler->setPosition(canonical.x, canonical.y, canonical.z);

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

    // Single-player mode callback — go to character creation first
    uiManager->getAuthScreen().setOnSinglePlayer([this]() {
        LOG_INFO("Single-player mode selected, opening character creation");
        singlePlayerMode = true;
        if (gameHandler) {
            gameHandler->setSinglePlayerMode(true);
            gameHandler->setSinglePlayerCharListReady();
        }
        // If characters exist, go to selection; otherwise go to creation
        if (gameHandler && !gameHandler->getCharacters().empty()) {
            setState(AppState::CHARACTER_SELECTION);
        } else {
            uiManager->getCharacterCreateScreen().reset();
            uiManager->getCharacterCreateScreen().initializePreview(assetManager.get());
            setState(AppState::CHARACTER_CREATION);
        }
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
        if (singlePlayerMode) {
            startSinglePlayer();
        } else {
            // Online mode - login will be handled by world entry callback
            setState(AppState::IN_GAME);
        }
    });

    // Character create screen callbacks
    uiManager->getCharacterCreateScreen().setOnCreate([this](const game::CharCreateData& data) {
        gameHandler->createCharacter(data);
    });

    uiManager->getCharacterCreateScreen().setOnCancel([this]() {
        if (singlePlayerMode) {
            setState(AppState::AUTHENTICATION);
            singlePlayerMode = false;
            gameHandler->setSinglePlayerMode(false);
        } else {
            setState(AppState::CHARACTER_SELECTION);
        }
    });

    // Character create result callback
    gameHandler->setCharCreateCallback([this](bool success, const std::string& msg) {
        if (success) {
            if (singlePlayerMode) {
                // In single-player, go straight to character selection showing the new character
                setState(AppState::CHARACTER_SELECTION);
            } else {
                setState(AppState::CHARACTER_SELECTION);
            }
        } else {
            uiManager->getCharacterCreateScreen().setStatus(msg, true);
        }
    });

    // World entry callback (online mode) - load terrain when entering world
    gameHandler->setWorldEntryCallback([this](uint32_t mapId, float x, float y, float z) {
        LOG_INFO("Online world entry: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")");
        loadOnlineWorldTerrain(mapId, x, y, z);
    });

    // Creature spawn callback (online mode) - spawn creature models
    gameHandler->setCreatureSpawnCallback([this](uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
        spawnOnlineCreature(guid, displayId, x, y, z, orientation);
    });

    // Creature despawn callback (online mode) - remove creature models
    gameHandler->setCreatureDespawnCallback([this](uint64_t guid) {
        despawnOnlineCreature(guid);
    });

    // "Create Character" button on character screen
    uiManager->getCharacterScreen().setOnCreateCharacter([this]() {
        uiManager->getCharacterCreateScreen().reset();
        uiManager->getCharacterCreateScreen().initializePreview(assetManager.get());
        setState(AppState::CHARACTER_CREATION);
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

                // Look up underwear textures from CharSections.dbc (humans only for now)
                bool useCharSections = (spRace_ == game::Race::HUMAN);
                uint32_t targetRaceId = static_cast<uint32_t>(spRace_);
                uint32_t targetSexId = (spGender_ == game::Gender::FEMALE) ? 1u : 0u;
                std::string bodySkinPath = (spGender_ == game::Gender::FEMALE)
                    ? "Character\\Human\\Female\\HumanFemaleSkin00_00.blp"
                    : "Character\\Human\\Male\\HumanMaleSkin00_00.blp";
                std::string pelvisPath = (spGender_ == game::Gender::FEMALE)
                    ? "Character\\Human\\Female\\HumanFemaleNakedPelvisSkin00_00.blp"
                    : "Character\\Human\\Male\\HumanMaleNakedPelvisSkin00_00.blp";
                std::string faceLowerTexturePath;
                std::vector<std::string> underwearPaths;

                if (useCharSections) {
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

                            if (raceId != targetRaceId || sexId != targetSexId) continue;

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

    if (npcManager) {
        npcManager->clear(renderer->getCharacterRenderer(), &gameHandler->getEntityManager());
    }
    npcManager = std::make_unique<game::NpcManager>();
    glm::vec3 playerSpawnGL = renderer->getCharacterPosition();
    glm::vec3 playerCanonical = core::coords::renderToCanonical(playerSpawnGL);
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

    // Set NPC death callback for single-player combat
    if (singlePlayerMode && gameHandler && npcManager) {
        auto* npcMgr = npcManager.get();
        auto* cr = renderer->getCharacterRenderer();
        gameHandler->setNpcDeathCallback([npcMgr, cr](uint64_t guid) {
            uint32_t instanceId = npcMgr->findRenderInstanceId(guid);
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, 1, false); // animation ID 1 = Death
            }
        });
    }

    npcsSpawned = true;
    LOG_INFO("NPCs spawned for in-game session");
}

void Application::startSinglePlayer() {
    LOG_INFO("Starting single-player mode...");

    // Set single-player flag
    singlePlayerMode = true;

    // Enable single-player combat mode on game handler
    if (gameHandler) {
        gameHandler->setSinglePlayerMode(true);
    }

    // Create world object for single-player
    if (!world) {
        world = std::make_unique<game::World>();
        LOG_INFO("Single-player world created");
    }

    const game::Character* activeChar = gameHandler ? gameHandler->getActiveCharacter() : nullptr;
    if (!activeChar && gameHandler) {
        activeChar = gameHandler->getFirstCharacter();
        if (activeChar) {
            gameHandler->setActiveCharacterGuid(activeChar->guid);
        }
    }
    if (!activeChar) {
        LOG_ERROR("Single-player start: no character selected");
        return;
    }

    spRace_ = activeChar->race;
    spGender_ = activeChar->gender;
    spClass_ = activeChar->characterClass;
    spMapId_ = activeChar->mapId;
    spZoneId_ = activeChar->zoneId;
    spSpawnCanonical_ = glm::vec3(activeChar->x, activeChar->y, activeChar->z);
    spYawDeg_ = 0.0f;
    spPitchDeg_ = -5.0f;

    bool loadedState = false;
    if (gameHandler) {
        gameHandler->setPlayerGuid(activeChar->guid);
        loadedState = gameHandler->loadSinglePlayerCharacterState(activeChar->guid);
        if (loadedState) {
            const auto& movement = gameHandler->getMovementInfo();
            spSpawnCanonical_ = glm::vec3(movement.x, movement.y, movement.z);
            spYawDeg_ = glm::degrees(movement.orientation);
            spawnSnapToGround = true;
        } else {
            game::GameHandler::SinglePlayerCreateInfo createInfo;
            bool hasCreate = gameHandler->getSinglePlayerCreateInfo(activeChar->race, activeChar->characterClass, createInfo);
            if (hasCreate) {
                spMapId_ = createInfo.mapId;
                spZoneId_ = createInfo.zoneId;
                spSpawnCanonical_ = glm::vec3(createInfo.x, createInfo.y, createInfo.z);
                spYawDeg_ = glm::degrees(createInfo.orientation);
                spPitchDeg_ = -5.0f;
                spawnSnapToGround = true;
            }
            uint32_t level = std::max<uint32_t>(1, activeChar->level);
            uint32_t maxHealth = 20 + level * 10;
            gameHandler->initLocalPlayerStats(level, maxHealth, maxHealth);
            gameHandler->applySinglePlayerStartData(activeChar->race, activeChar->characterClass);
        }
    }

    // Load weapon models for equipped items (after inventory is populated)
    loadEquippedWeapons();

    if (gameHandler && renderer && window) {
        game::GameHandler::SinglePlayerSettings settings;
        bool hasSettings = gameHandler->getSinglePlayerSettings(settings);
        if (!hasSettings) {
            settings.fullscreen = window->isFullscreen();
            settings.vsync = window->isVsyncEnabled();
            settings.shadows = renderer->areShadowsEnabled();
            settings.resWidth = window->getWidth();
            settings.resHeight = window->getHeight();
            if (auto* music = renderer->getMusicManager()) {
                settings.musicVolume = music->getVolume();
            }
            if (auto* footstep = renderer->getFootstepManager()) {
                settings.sfxVolume = static_cast<int>(footstep->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* cameraController = renderer->getCameraController()) {
                settings.mouseSensitivity = cameraController->getMouseSensitivity();
                settings.invertMouse = cameraController->isInvertMouse();
            }
            gameHandler->setSinglePlayerSettings(settings);
            hasSettings = true;
        }
        if (hasSettings) {
            window->setVsync(settings.vsync);
            window->setFullscreen(settings.fullscreen);
            if (settings.resWidth > 0 && settings.resHeight > 0) {
                window->applyResolution(settings.resWidth, settings.resHeight);
            }
            renderer->setShadowsEnabled(settings.shadows);
            if (auto* music = renderer->getMusicManager()) {
                music->setVolume(settings.musicVolume);
            }
            float sfxScale = static_cast<float>(settings.sfxVolume) / 100.0f;
            if (auto* footstep = renderer->getFootstepManager()) {
                footstep->setVolumeScale(sfxScale);
            }
            if (auto* activity = renderer->getActivitySoundManager()) {
                activity->setVolumeScale(sfxScale);
            }
            if (auto* cameraController = renderer->getCameraController()) {
                cameraController->setMouseSensitivity(settings.mouseSensitivity);
                cameraController->setInvertMouse(settings.invertMouse);
                cameraController->startIntroPan(2.8f, 140.0f);
            }
        }
    }

    // --- Loading screen: load terrain and wait for streaming before spawning ---
    const SpawnPreset* spawnPreset = selectSpawnPreset(std::getenv("WOW_SPAWN"));
    // Canonical WoW coords: +X=North, +Y=West, +Z=Up
    glm::vec3 spawnCanonical = spawnPreset ? spawnPreset->spawnCanonical : spSpawnCanonical_;
    std::string mapName = spawnPreset ? spawnPreset->mapName : mapIdToName(spMapId_);
    float spawnYaw = spawnPreset ? spawnPreset->yawDeg : spYawDeg_;
    float spawnPitch = spawnPreset ? spawnPreset->pitchDeg : spPitchDeg_;
    spawnSnapToGround = spawnPreset ? spawnPreset->snapToGround : spawnSnapToGround;

    if (auto envSpawnPos = parseVec3Csv(std::getenv("WOW_SPAWN_POS"))) {
        spawnCanonical = *envSpawnPos;
        LOG_INFO("Using WOW_SPAWN_POS override (canonical WoW X,Y,Z): (",
                 spawnCanonical.x, ", ", spawnCanonical.y, ", ", spawnCanonical.z, ")");
    }
    if (auto envSpawnRot = parseYawPitchCsv(std::getenv("WOW_SPAWN_ROT"))) {
        spawnYaw = envSpawnRot->first;
        spawnPitch = envSpawnRot->second;
        LOG_INFO("Using WOW_SPAWN_ROT override: yaw=", spawnYaw, " pitch=", spawnPitch);
    }

    // Convert canonical WoW → engine rendering coordinates (swap X/Y)
    glm::vec3 spawnRender = core::coords::canonicalToRender(spawnCanonical);
    if (renderer && renderer->getCameraController()) {
        renderer->getCameraController()->setDefaultSpawn(spawnRender, spawnYaw, spawnPitch);
    }

    if (gameHandler && !loadedState) {
        gameHandler->setPosition(spawnCanonical.x, spawnCanonical.y, spawnCanonical.z);
        gameHandler->setOrientation(glm::radians(spawnYaw - 90.0f));
        gameHandler->flushSinglePlayerSave();
    }
    if (spawnPreset) {
        LOG_INFO("Single-player spawn preset: ", spawnPreset->label,
                 " canonical=(",
                 spawnCanonical.x, ", ", spawnCanonical.y, ", ", spawnCanonical.z,
                 ") (set WOW_SPAWN to change)");
        LOG_INFO("Optional spawn overrides (canonical WoW X,Y,Z): WOW_SPAWN_POS=x,y,z WOW_SPAWN_ROT=yaw,pitch");
    }

    rendering::LoadingScreen loadingScreen;
    bool loadingScreenOk = loadingScreen.initialize();

    auto showStatus = [&](const char* msg) {
        if (!loadingScreenOk) return;
        loadingScreen.setStatus(msg);
        loadingScreen.render();
        window->swapBuffers();
    };

    showStatus("Loading terrain...");

    // Set map name for zone-specific floor cache
    if (renderer->getWMORenderer()) {
        renderer->getWMORenderer()->setMapName(mapName);
    }

    // Try to load test terrain if WOW_DATA_PATH is set
    bool terrainOk = false;
    if (renderer && assetManager && assetManager->isInitialized()) {
        // Compute ADT path from canonical spawn coordinates
        auto [tileX, tileY] = core::coords::canonicalToTile(spawnCanonical.x, spawnCanonical.y);
        std::string adtPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                              std::to_string(tileX) + "_" + std::to_string(tileY) + ".adt";
        LOG_INFO("Initial ADT tile [", tileX, ",", tileY, "] from canonical position");
        terrainOk = renderer->loadTestTerrain(assetManager.get(), adtPath);
        if (!terrainOk) {
            LOG_WARNING("Could not load test terrain - atmospheric rendering only");
        }
    }

    // Wait for surrounding terrain tiles to stream in
    if (terrainOk && renderer->getTerrainManager() && renderer->getCamera()) {
        auto* terrainMgr = renderer->getTerrainManager();
        auto* camera = renderer->getCamera();

        // First update with large dt to trigger streamTiles() immediately
        terrainMgr->update(*camera, 1.0f);

        auto startTime = std::chrono::high_resolution_clock::now();
        const float maxWaitSeconds = 15.0f;

        while (terrainMgr->getPendingTileCount() > 0) {
            // Poll events to keep window responsive
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    window->setShouldClose(true);
                    loadingScreen.shutdown();
                    return;
                }
            }

            // Process ready tiles from worker threads
            terrainMgr->update(*camera, 0.016f);

            // Update loading screen with progress
            if (loadingScreenOk) {
                int loaded = terrainMgr->getLoadedTileCount();
                int pending = terrainMgr->getPendingTileCount();
                char buf[128];
                snprintf(buf, sizeof(buf), "Loading terrain... %d tiles loaded, %d remaining",
                         loaded, pending);
                loadingScreen.setStatus(buf);
                loadingScreen.render();
                window->swapBuffers();
            }

            // Timeout safety
            auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
            if (std::chrono::duration<float>(elapsed).count() > maxWaitSeconds) {
                LOG_WARNING("Terrain streaming timeout after ", maxWaitSeconds, "s");
                break;
            }

            SDL_Delay(16);  // ~60fps cap for loading screen
        }

        LOG_INFO("Terrain streaming complete: ", terrainMgr->getLoadedTileCount(), " tiles loaded");

        // Load zone-specific floor cache, or precompute if none exists
        if (renderer->getWMORenderer()) {
            renderer->getWMORenderer()->loadFloorCache();
            if (renderer->getWMORenderer()->getFloorCacheSize() == 0) {
                showStatus("Pre-computing collision cache...");
                renderer->getWMORenderer()->precomputeFloorCache();
            }
        }

        // Re-snap camera to ground now that all surrounding tiles are loaded
        // (the initial reset inside loadTestTerrain only had 1 tile).
        if (spawnSnapToGround && renderer->getCameraController()) {
            renderer->getCameraController()->reset();
        }
    }

    showStatus("Spawning character...");

    // Spawn player character on loaded terrain
    spawnPlayerCharacter();

    // Final camera reset: now that follow target exists and terrain is loaded,
    // snap the third-person camera into the correct orbit position.
    if (spawnSnapToGround && renderer && renderer->getCameraController()) {
        renderer->getCameraController()->reset();
    }

    if (loadingScreenOk) {
        loadingScreen.shutdown();
    }

    // Wire hearthstone to camera reset (teleport home) in single-player
    if (gameHandler && renderer && renderer->getCameraController()) {
        auto* camCtrl = renderer->getCameraController();
        gameHandler->setHearthstoneCallback([camCtrl]() {
            camCtrl->reset();
        });
    }

    // Go directly to game
    setState(AppState::IN_GAME);
    // Emulate server MOTD in single-player (after entering game)
    if (gameHandler) {
        std::vector<std::string> motdLines;
        if (const char* motdEnv = std::getenv("WOW_SP_MOTD")) {
            std::string raw = motdEnv;
            size_t start = 0;
            while (start <= raw.size()) {
                size_t pos = raw.find('|', start);
                if (pos == std::string::npos) pos = raw.size();
                std::string line = raw.substr(start, pos - start);
                if (!line.empty()) motdLines.push_back(line);
                start = pos + 1;
                if (pos == raw.size()) break;
            }
        }
        if (motdLines.empty()) {
            motdLines.push_back("Wowee Single Player");
        }
        gameHandler->simulateMotd(motdLines);
    }
    LOG_INFO("Single-player mode started - press F1 for performance HUD");
}

void Application::teleportTo(int presetIndex) {
    // Guard: only in single-player + IN_GAME state
    if (!singlePlayerMode || state != AppState::IN_GAME) return;
    if (presetIndex < 0 || presetIndex >= SPAWN_PRESET_COUNT) return;

    const auto& preset = SPAWN_PRESETS[presetIndex];
    LOG_INFO("Teleporting to: ", preset.label);
    spawnSnapToGround = preset.snapToGround;

    // Convert canonical WoW → engine rendering coordinates (swap X/Y)
    glm::vec3 spawnRender = core::coords::canonicalToRender(preset.spawnCanonical);

    // Update camera default spawn
    if (renderer && renderer->getCameraController()) {
        renderer->getCameraController()->setDefaultSpawn(spawnRender, preset.yawDeg, preset.pitchDeg);
    }

    // Save current map's floor cache before unloading
    if (renderer && renderer->getWMORenderer()) {
        auto* wmo = renderer->getWMORenderer();
        if (wmo->getFloorCacheSize() > 0) {
            wmo->saveFloorCache();
        }
    }

    // Unload all current terrain
    if (renderer && renderer->getTerrainManager()) {
        renderer->getTerrainManager()->unloadAll();
    }

    // Compute ADT path from canonical spawn coordinates
    auto [tileX, tileY] = core::coords::canonicalToTile(preset.spawnCanonical.x, preset.spawnCanonical.y);
    std::string mapName = preset.mapName;
    std::string adtPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                          std::to_string(tileX) + "_" + std::to_string(tileY) + ".adt";
    LOG_INFO("Teleport ADT tile [", tileX, ",", tileY, "]");

    // Set map name on terrain manager and WMO renderer
    if (renderer && renderer->getTerrainManager()) {
        renderer->getTerrainManager()->setMapName(mapName);
    }
    if (renderer && renderer->getWMORenderer()) {
        renderer->getWMORenderer()->setMapName(mapName);
    }

    // Load the initial tile
    bool terrainOk = false;
    if (renderer && assetManager && assetManager->isInitialized()) {
        terrainOk = renderer->loadTestTerrain(assetManager.get(), adtPath);
    }

    // Stream surrounding tiles
    if (terrainOk && renderer->getTerrainManager() && renderer->getCamera()) {
        auto* terrainMgr = renderer->getTerrainManager();
        auto* camera = renderer->getCamera();

        terrainMgr->update(*camera, 1.0f);

        auto startTime = std::chrono::high_resolution_clock::now();
        const float maxWaitSeconds = 8.0f;

        while (terrainMgr->getPendingTileCount() > 0) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    window->setShouldClose(true);
                    return;
                }
            }

            terrainMgr->update(*camera, 0.016f);

            auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
            if (std::chrono::duration<float>(elapsed).count() > maxWaitSeconds) {
                LOG_WARNING("Teleport terrain streaming timeout after ", maxWaitSeconds, "s");
                break;
            }

            SDL_Delay(16);
        }

        LOG_INFO("Teleport terrain streaming complete: ", terrainMgr->getLoadedTileCount(), " tiles loaded");

        // Load zone-specific floor cache, or precompute if none exists
        if (renderer->getWMORenderer()) {
            renderer->getWMORenderer()->loadFloorCache();
            if (renderer->getWMORenderer()->getFloorCacheSize() == 0) {
                renderer->getWMORenderer()->precomputeFloorCache();
            }
        }
    }

    // Floor-snapping presets use camera reset. WMO-floor presets keep explicit Z.
    if (spawnSnapToGround && renderer && renderer->getCameraController()) {
        renderer->getCameraController()->reset();
    }

    if (!spawnSnapToGround && renderer) {
        renderer->getCharacterPosition() = spawnRender;
    }

    // Sync final character position to game handler
    if (renderer && gameHandler) {
        glm::vec3 finalRender = renderer->getCharacterPosition();
        glm::vec3 finalCanonical = core::coords::renderToCanonical(finalRender);
        gameHandler->setPosition(finalCanonical.x, finalCanonical.y, finalCanonical.z);
    }

    // Rebuild nearby NPC set for the new location.
    if (singlePlayerMode && gameHandler && renderer && renderer->getCharacterRenderer()) {
        if (npcManager) {
            npcManager->clear(renderer->getCharacterRenderer(), &gameHandler->getEntityManager());
        }
        npcsSpawned = false;
        spawnNpcs();
    }

    LOG_INFO("Teleport to ", preset.label, " complete");
}

void Application::loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z) {
    if (!renderer || !assetManager || !assetManager->isInitialized()) {
        LOG_WARNING("Cannot load online terrain: renderer or assets not ready");
        return;
    }

    std::string mapName = mapIdToName(mapId);
    LOG_INFO("Loading online world terrain for map '", mapName, "' (ID ", mapId, ")");

    // Convert server coordinates to canonical WoW coordinates
    // Server sends: X=West (canonical.Y), Y=North (canonical.X), Z=Up
    glm::vec3 spawnCanonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
    glm::vec3 spawnRender = core::coords::canonicalToRender(spawnCanonical);

    // Set camera position
    if (renderer->getCameraController()) {
        renderer->getCameraController()->setDefaultSpawn(spawnRender, 0.0f, 15.0f);
        renderer->getCameraController()->reset();
    }

    // Set map name for WMO renderer
    if (renderer->getWMORenderer()) {
        renderer->getWMORenderer()->setMapName(mapName);
    }

    // Set map name for terrain manager
    if (renderer->getTerrainManager()) {
        renderer->getTerrainManager()->setMapName(mapName);
    }

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

        // Trigger terrain streaming for surrounding tiles
        if (renderer->getTerrainManager() && renderer->getCamera()) {
            renderer->getTerrainManager()->update(*renderer->getCamera(), 1.0f);
        }
    }

    // Spawn player model for online mode
    if (gameHandler) {
        const game::Character* activeChar = gameHandler->getActiveCharacter();
        if (activeChar) {
            // Set race/gender for player model loading
            spRace_ = activeChar->race;
            spGender_ = activeChar->gender;
            spClass_ = activeChar->characterClass;

            // Don't snap to ground - server provides exact position
            spawnSnapToGround = false;

            // Reset spawn flag and spawn the player character model
            playerCharacterSpawned = false;
            spawnPlayerCharacter();

            // Explicitly set character position to match server coordinates
            renderer->getCharacterPosition() = spawnRender;

            LOG_INFO("Spawned online player model: ", activeChar->name,
                     " (race=", static_cast<int>(spRace_),
                     ", gender=", static_cast<int>(spGender_),
                     ") at render pos (", spawnRender.x, ", ", spawnRender.y, ", ", spawnRender.z, ")");
        } else {
            LOG_WARNING("No active character found for player model spawning");
        }
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

    creatureLookupsBuilt_ = true;
}

std::string Application::getModelPathForDisplayId(uint32_t displayId) const {
    auto itData = displayDataMap_.find(displayId);
    if (itData == displayDataMap_.end()) return "";

    auto itPath = modelIdToPath_.find(itData->second.modelId);
    if (itPath == modelIdToPath_.end()) return "";

    return itPath->second;
}

void Application::spawnOnlineCreature(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation) {
    if (!renderer || !renderer->getCharacterRenderer() || !assetManager) return;

    // Build lookups on first creature spawn
    if (!creatureLookupsBuilt_) {
        buildCreatureDisplayLookups();
    }

    // Skip if already spawned
    if (creatureInstances_.count(guid)) return;

    // Get model path from displayId
    std::string m2Path = getModelPathForDisplayId(displayId);
    if (m2Path.empty()) {
        LOG_WARNING("No model path for displayId ", displayId, " (guid 0x", std::hex, guid, std::dec, ")");
        return;
    }

    auto* charRenderer = renderer->getCharacterRenderer();

    // Load model if not already loaded for this displayId
    uint32_t modelId = nextCreatureModelId_++;

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

    // Apply skin textures from CreatureDisplayInfo.dbc
    auto itDisplayData = displayDataMap_.find(displayId);
    if (itDisplayData != displayDataMap_.end()) {
        const auto& dispData = itDisplayData->second;

        // Get model directory for texture path construction
        std::string modelDir;
        size_t lastSlash = m2Path.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            modelDir = m2Path.substr(0, lastSlash + 1);
        }

        LOG_DEBUG("DisplayId ", displayId, " skins: '", dispData.skin1, "', '", dispData.skin2, "', '", dispData.skin3,
                  "' extraDisplayId=", dispData.extraDisplayId);

        // Log texture types in the model
        for (size_t ti = 0; ti < model.textures.size(); ti++) {
            LOG_DEBUG("  Model texture ", ti, ": type=", model.textures[ti].type, " filename='", model.textures[ti].filename, "'");
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

                // Use baked texture as-is (baked textures already include full NPC appearance)
                // Equipment component textures are only for player characters with CharComponentTextureSections UV layout
                if (!extra.bakeName.empty()) {
                    std::string bakePath = "Textures\\BakedNpcTextures\\" + extra.bakeName;
                    GLuint finalTex = charRenderer->loadTexture(bakePath);

                    if (finalTex != 0) {
                        for (size_t ti = 0; ti < model.textures.size(); ti++) {
                            if (model.textures[ti].type == 1) {
                                charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), finalTex);
                                LOG_DEBUG("Applied baked NPC texture to slot ", ti, ": ", bakePath);
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
                    for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                        uint32_t raceId = charSectionsDbc->getUInt32(r, 1);
                        uint32_t sexId = charSectionsDbc->getUInt32(r, 2);
                        uint32_t baseSection = charSectionsDbc->getUInt32(r, 3);
                        uint32_t variationIndex = charSectionsDbc->getUInt32(r, 8);
                        uint32_t colorIndex = charSectionsDbc->getUInt32(r, 9);

                        // Section 3: Hair (variation = hair style, colorIndex = hair color)
                        if (baseSection == 3 &&
                            raceId == extra.raceId && sexId == extra.sexId &&
                            variationIndex == extra.hairStyleId && colorIndex == extra.hairColorId) {
                            std::string hairPath = charSectionsDbc->getString(r, 4);
                            if (!hairPath.empty()) {
                                GLuint hairTex = charRenderer->loadTexture(hairPath);
                                if (hairTex != 0) {
                                    // Apply to type-6 texture slot (hair)
                                    for (size_t ti = 0; ti < model.textures.size(); ti++) {
                                        if (model.textures[ti].type == 6) {
                                            charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), hairTex);
                                            LOG_DEBUG("Applied hair texture: ", hairPath, " to slot ", ti);
                                        }
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
            } else {
                LOG_WARNING("  extraDisplayId ", dispData.extraDisplayId, " not found in humanoidExtraMap");
            }
        }

        // Apply creature skin textures (for non-humanoid creatures)
        if (!hasHumanoidTexture) {
            for (size_t ti = 0; ti < model.textures.size(); ti++) {
                const auto& tex = model.textures[ti];
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

    // Create instance
    uint32_t instanceId = charRenderer->createInstance(modelId, renderPos,
        glm::vec3(0.0f, 0.0f, orientation), 1.0f);

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

            // Body parts (group 0: IDs 0-18)
            for (uint16_t i = 0; i <= 18; i++) {
                activeGeosets.insert(i);
            }

            // Hair style geoset: 100 + hairStyleId + 1 (101 = style 0, 102 = style 1, etc.)
            activeGeosets.insert(static_cast<uint16_t>(101 + extra.hairStyleId));

            // Facial hair geoset: 200 + facialHairId + 1 (201 = none/style 0, 202 = style 1, etc.)
            activeGeosets.insert(static_cast<uint16_t>(201 + extra.facialHairId));

            // Default equipment geosets (bare/no armor)
            uint16_t geosetGloves = 301;   // Bare hands
            uint16_t geosetBoots = 401;    // Bare feet
            uint16_t geosetChest = 501;    // Bare chest
            uint16_t geosetPants = 1301;   // Bare legs
            uint16_t geosetCape = 1501;    // No cape
            uint16_t geosetTabard = 1201;  // No tabard
            bool hideHair = false;

            // Load equipment geosets from ItemDisplayInfo.dbc
            auto itemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
            if (itemDisplayDbc) {
                // Equipment slots: 0=helm, 1=shoulder, 2=shirt, 3=chest, 4=belt, 5=legs, 6=feet, 7=wrist, 8=hands, 9=tabard, 10=cape
                // ItemDisplayInfo geoset columns: 5=GeosetGroup[0], 6=GeosetGroup[1], 7=GeosetGroup[2]

                // Helm (slot 0) - may hide hair
                if (extra.equipDisplayId[0] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[0]);
                    if (idx >= 0) {
                        // Check helmet vis flags (col 12-13) or just hide hair if helm exists
                        hideHair = true;
                    }
                }

                // Chest (slot 3) - geoset group 5xx/8xx
                if (extra.equipDisplayId[3] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[3]);
                    if (idx >= 0) {
                        uint32_t geoGroup = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 5);
                        if (geoGroup > 0) geosetChest = static_cast<uint16_t>(500 + geoGroup);
                    }
                }

                // Legs (slot 5) - geoset group 13xx
                if (extra.equipDisplayId[5] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[5]);
                    if (idx >= 0) {
                        uint32_t geoGroup = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 5);
                        if (geoGroup > 0) geosetPants = static_cast<uint16_t>(1300 + geoGroup);
                    }
                }

                // Feet (slot 6) - geoset group 4xx
                if (extra.equipDisplayId[6] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[6]);
                    if (idx >= 0) {
                        uint32_t geoGroup = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 5);
                        if (geoGroup > 0) geosetBoots = static_cast<uint16_t>(400 + geoGroup);
                    }
                }

                // Hands (slot 8) - geoset group 3xx
                if (extra.equipDisplayId[8] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[8]);
                    if (idx >= 0) {
                        uint32_t geoGroup = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 5);
                        if (geoGroup > 0) geosetGloves = static_cast<uint16_t>(300 + geoGroup);
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
                        uint32_t geoGroup = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), 5);
                        if (geoGroup > 0) geosetCape = static_cast<uint16_t>(1500 + geoGroup);
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

            // Hide hair if wearing helm
            if (hideHair) {
                activeGeosets.erase(static_cast<uint16_t>(101 + extra.hairStyleId));
            }

            charRenderer->setActiveGeosets(instanceId, activeGeosets);
            LOG_DEBUG("Set humanoid geosets: hair=", hideHair ? 0 : (101 + extra.hairStyleId),
                      " facial=", 201 + extra.facialHairId,
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
                            helmModelName = helmModelName.substr(0, dotPos) + ".m2";
                        } else {
                            helmModelName += ".m2";
                        }

                        // Try to load helmet from various paths
                        std::string helmPath = "Item\\ObjectComponents\\Head\\" + helmModelName;
                        auto helmData = assetManager->readFile(helmPath);
                        if (helmData.empty()) {
                            // Try alternate path
                            helmPath = "Item\\ObjectComponents\\Helmet\\" + helmModelName;
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
                                    helmTexPath = "Item\\ObjectComponents\\Head\\" + helmTexName + ".blp";
                                }
                                charRenderer->attachWeapon(instanceId, 11, helmModel, helmModelId, helmTexPath);
                                LOG_DEBUG("Attached helmet model: ", helmPath);
                            }
                        }
                    }
                }
            }
        }
    }

    // Play idle animation
    charRenderer->playAnimation(instanceId, 0, true);

    // Track instance
    creatureInstances_[guid] = instanceId;
    creatureModelIds_[guid] = modelId;

    LOG_INFO("Spawned creature: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
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

} // namespace core
} // namespace wowee
