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
#include "rendering/quest_marker_renderer.hpp"
#include "rendering/loading_screen.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include <imgui.h>
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "ui/ui_manager.hpp"
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "game/transport_manager.hpp"
#include "game/world.hpp"
#include "game/expansion_profile.hpp"
#include "game/packet_parsers.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"

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
#include <filesystem>

namespace wowee {
namespace core {


const char* Application::mapIdToName(uint32_t mapId) {
    switch (mapId) {
        case 0: return "Azeroth";
        case 1: return "Kalimdor";
        case 369: return "DeeprunTram";
        case 530: return "Outland";
        case 571: return "Northrend";
        default: return "";
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

    // Create and initialize expansion registry
    expansionRegistry_ = std::make_unique<game::ExpansionRegistry>();

    // Create DBC layout
    dbcLayout_ = std::make_unique<pipeline::DBCLayout>();

    // Create asset manager
    assetManager = std::make_unique<pipeline::AssetManager>();

    // Try to get WoW data path from environment variable
    const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
    std::string dataPath = dataPathEnv ? dataPathEnv : "./Data";

    // Scan for available expansion profiles
    expansionRegistry_->initialize(dataPath);

    // Load expansion-specific opcode table
    if (gameHandler && expansionRegistry_) {
        auto* profile = expansionRegistry_->getActive();
        if (profile) {
            std::string opcodesPath = profile->dataPath + "/opcodes.json";
            if (!gameHandler->getOpcodeTable().loadFromJson(opcodesPath)) {
                LOG_ERROR("Failed to load opcodes from ", opcodesPath);
            }
            game::setActiveOpcodeTable(&gameHandler->getOpcodeTable());

            // Load expansion-specific update field table
            std::string updateFieldsPath = profile->dataPath + "/update_fields.json";
            if (!gameHandler->getUpdateFieldTable().loadFromJson(updateFieldsPath)) {
                LOG_ERROR("Failed to load update fields from ", updateFieldsPath);
            }
            game::setActiveUpdateFieldTable(&gameHandler->getUpdateFieldTable());

            // Create expansion-specific packet parsers
            gameHandler->setPacketParsers(game::createPacketParsers(profile->id));

            // Load expansion-specific DBC layouts
            if (dbcLayout_) {
                std::string dbcLayoutsPath = profile->dataPath + "/dbc_layouts.json";
                if (!dbcLayout_->loadFromJson(dbcLayoutsPath)) {
                    LOG_ERROR("Failed to load DBC layouts from ", dbcLayoutsPath);
                }
                pipeline::setActiveDBCLayout(dbcLayout_.get());
            }
        }
    }

    // Try expansion-specific asset path first, fall back to base Data/
    std::string assetPath = dataPath;
    if (expansionRegistry_) {
        auto* profile = expansionRegistry_->getActive();
        if (profile && !profile->dataPath.empty()) {
            // Enable expansion-specific CSV DBC lookup (Data/expansions/<id>/db/*.csv).
            assetManager->setExpansionDataPath(profile->dataPath);

            std::string expansionManifest = profile->dataPath + "/manifest.json";
            if (std::filesystem::exists(expansionManifest)) {
                assetPath = profile->dataPath;
                LOG_INFO("Using expansion-specific asset path: ", assetPath);
            }
        }
    }

    LOG_INFO("Attempting to load WoW assets from: ", assetPath);
    if (assetManager->initialize(assetPath)) {
        LOG_INFO("Asset manager initialized successfully");
        // Eagerly load creature display DBC lookups so first spawn doesn't stall
        buildCreatureDisplayLookups();

        // Ensure the main in-world CharacterRenderer can load textures immediately.
        // Previously this was only wired during terrain initialization, which meant early spawns
        // (before terrain load) would render with white fallback textures (notably hair).
        if (renderer && renderer->getCharacterRenderer()) {
            renderer->getCharacterRenderer()->setAssetManager(assetManager.get());
        }

        // Load transport paths from TransportAnimation.dbc and TaxiPathNode.dbc
        if (gameHandler && gameHandler->getTransportManager()) {
            gameHandler->getTransportManager()->loadTransportAnimationDBC(assetManager.get());
            gameHandler->getTransportManager()->loadTaxiPathNodeDBC(assetManager.get());
        }

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
                // F7: Test level-up effect (ignore key repeat)
                else if (event.key.keysym.scancode == SDL_SCANCODE_F7 && event.key.repeat == 0) {
                    if (renderer) {
                        renderer->triggerLevelUpEffect(renderer->getCharacterPosition());
                        LOG_INFO("Triggered test level-up effect");
                    }
                    if (uiManager) {
                        uiManager->getGameScreen().triggerDing(99);
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
            if (uiManager && assetManager) {
                uiManager->getCharacterScreen().setAssetManager(assetManager.get());
            }
            // Ensure no stale in-world player model leaks into the next login attempt.
            // If we reuse a previously spawned instance without forcing a respawn, appearance (notably hair) can desync.
            npcsSpawned = false;
            playerCharacterSpawned = false;
            weaponsSheathed_ = false;
            wasAutoAttacking_ = false;
            loadedMapId_ = 0xFFFFFFFF;
            spawnedPlayerGuid_ = 0;
            spawnedAppearanceBytes_ = 0;
            spawnedFacialFeatures_ = 0;
            if (renderer && renderer->getCharacterRenderer()) {
                uint32_t oldInst = renderer->getCharacterInstanceId();
                if (oldInst > 0) {
                    renderer->setCharacterFollow(0);
                    renderer->clearMount();
                    renderer->getCharacterRenderer()->removeInstance(oldInst);
                }
            }
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

void Application::reloadExpansionData() {
    if (!expansionRegistry_ || !gameHandler) return;
    auto* profile = expansionRegistry_->getActive();
    if (!profile) return;

    LOG_INFO("Reloading expansion data for: ", profile->name);

    std::string opcodesPath = profile->dataPath + "/opcodes.json";
    if (!gameHandler->getOpcodeTable().loadFromJson(opcodesPath)) {
        LOG_ERROR("Failed to load opcodes from ", opcodesPath);
    }
    game::setActiveOpcodeTable(&gameHandler->getOpcodeTable());

    std::string updateFieldsPath = profile->dataPath + "/update_fields.json";
    if (!gameHandler->getUpdateFieldTable().loadFromJson(updateFieldsPath)) {
        LOG_ERROR("Failed to load update fields from ", updateFieldsPath);
    }
    game::setActiveUpdateFieldTable(&gameHandler->getUpdateFieldTable());

    gameHandler->setPacketParsers(game::createPacketParsers(profile->id));

    if (dbcLayout_) {
        std::string dbcLayoutsPath = profile->dataPath + "/dbc_layouts.json";
        if (!dbcLayout_->loadFromJson(dbcLayoutsPath)) {
            LOG_ERROR("Failed to load DBC layouts from ", dbcLayoutsPath);
        }
        pipeline::setActiveDBCLayout(dbcLayout_.get());
    }

    // Update expansion data path for CSV DBC lookups and clear DBC cache
    if (assetManager && !profile->dataPath.empty()) {
        assetManager->setExpansionDataPath(profile->dataPath);
        assetManager->clearDBCCache();
    }

    // Reset map name cache so it reloads from new expansion's Map.dbc
    mapNameCacheLoaded_ = false;
    mapNameById_.clear();

    // Reset game handler DBC caches so they reload from new expansion data
    if (gameHandler) {
        gameHandler->resetDbcCaches();
    }

    // Rebuild creature display lookups with the new expansion's DBC layout
    creatureLookupsBuilt_ = false;
    displayDataMap_.clear();
    humanoidExtraMap_.clear();
    creatureModelIds_.clear();
    creatureRenderPosCache_.clear();
    buildCreatureDisplayLookups();
}

void Application::logoutToLogin() {
    LOG_INFO("Logout requested");
    if (gameHandler) {
        gameHandler->disconnect();
    }
    npcsSpawned = false;
    playerCharacterSpawned = false;
    weaponsSheathed_ = false;
    wasAutoAttacking_ = false;
    loadedMapId_ = 0xFFFFFFFF;
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
    // Clear stale realm/character selection so switching servers starts fresh
    if (uiManager) {
        uiManager->getRealmScreen().reset();
        uiManager->getCharacterScreen().reset();
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
            // Application update profiling
            static int appProfileCounter = 0;
            static float ghTime = 0.0f, worldTime = 0.0f, spawnTime = 0.0f;
            static float creatureQTime = 0.0f, goQTime = 0.0f, mountTime = 0.0f;
            static float npcMgrTime = 0.0f, questMarkTime = 0.0f, syncTime = 0.0f;

            auto gh1 = std::chrono::high_resolution_clock::now();
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            auto gh2 = std::chrono::high_resolution_clock::now();
            ghTime += std::chrono::duration<float, std::milli>(gh2 - gh1).count();

            // Always unsheath on combat engage.
            if (gameHandler) {
                const bool autoAttacking = gameHandler->isAutoAttacking();
                if (autoAttacking && !wasAutoAttacking_ && weaponsSheathed_) {
                    weaponsSheathed_ = false;
                    loadEquippedWeapons();
                }
                wasAutoAttacking_ = autoAttacking;
            }

            // Toggle weapon sheathe state with Z (ignored while UI captures keyboard).
            {
                const bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard;
                auto& input = Input::getInstance();
                if (!uiWantsKeyboard && input.isKeyJustPressed(SDL_SCANCODE_Z)) {
                    weaponsSheathed_ = !weaponsSheathed_;
                    loadEquippedWeapons();
                }
            }

            auto w1 = std::chrono::high_resolution_clock::now();
            if (world) {
                world->update(deltaTime);
            }
            auto w2 = std::chrono::high_resolution_clock::now();
            worldTime += std::chrono::duration<float, std::milli>(w2 - w1).count();

            auto cq1 = std::chrono::high_resolution_clock::now();
            processPlayerSpawnQueue();
            // Process deferred online creature spawns (throttled)
            processCreatureSpawnQueue();
            // Process deferred equipment compositing (max 1 per frame to avoid stutter)
            processDeferredEquipmentQueue();
            auto cq2 = std::chrono::high_resolution_clock::now();
            creatureQTime += std::chrono::duration<float, std::milli>(cq2 - cq1).count();

            // Self-heal missing creature visuals: if a nearby UNIT exists in
            // entity state but has no render instance, queue a spawn retry.
            if (gameHandler) {
                static float creatureResyncTimer = 0.0f;
                creatureResyncTimer += deltaTime;
                if (creatureResyncTimer >= 1.0f) {
                    creatureResyncTimer = 0.0f;

                    glm::vec3 playerPos(0.0f);
                    bool havePlayerPos = false;
                    uint64_t playerGuid = gameHandler->getPlayerGuid();
                    if (auto playerEntity = gameHandler->getEntityManager().getEntity(playerGuid)) {
                        playerPos = glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ());
                        havePlayerPos = true;
                    }

                    const float kResyncRadiusSq = 260.0f * 260.0f;
                    for (const auto& pair : gameHandler->getEntityManager().getEntities()) {
                        uint64_t guid = pair.first;
                        const auto& entity = pair.second;
                        if (!entity || guid == playerGuid) continue;
                        if (entity->getType() != game::ObjectType::UNIT) continue;
                        auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
                        if (!unit || unit->getDisplayId() == 0) continue;
                        if (creatureInstances_.count(guid) || pendingCreatureSpawnGuids_.count(guid)) continue;

                        if (havePlayerPos) {
                            glm::vec3 pos(unit->getX(), unit->getY(), unit->getZ());
                            glm::vec3 delta = pos - playerPos;
                            float distSq = glm::dot(delta, delta);
                            if (distSq > kResyncRadiusSq) continue;
                        }

                        PendingCreatureSpawn retrySpawn{};
                        retrySpawn.guid = guid;
                        retrySpawn.displayId = unit->getDisplayId();
                        retrySpawn.x = unit->getX();
                        retrySpawn.y = unit->getY();
                        retrySpawn.z = unit->getZ();
                        retrySpawn.orientation = unit->getOrientation();
                        pendingCreatureSpawns_.push_back(retrySpawn);
                        pendingCreatureSpawnGuids_.insert(guid);
                    }
                }
            }

            auto goq1 = std::chrono::high_resolution_clock::now();
            processGameObjectSpawnQueue();
            processPendingTransportDoodads();
            auto goq2 = std::chrono::high_resolution_clock::now();
            goQTime += std::chrono::duration<float, std::milli>(goq2 - goq1).count();

            auto m1 = std::chrono::high_resolution_clock::now();
            processPendingMount();
            auto m2 = std::chrono::high_resolution_clock::now();
            mountTime += std::chrono::duration<float, std::milli>(m2 - m1).count();

            auto nm1 = std::chrono::high_resolution_clock::now();
            auto nm2 = std::chrono::high_resolution_clock::now();
            npcMgrTime += std::chrono::duration<float, std::milli>(nm2 - nm1).count();

            auto qm1 = std::chrono::high_resolution_clock::now();
            // Update 3D quest markers above NPCs
            updateQuestMarkers();
            auto qm2 = std::chrono::high_resolution_clock::now();
            questMarkTime += std::chrono::duration<float, std::milli>(qm2 - qm1).count();

            auto sync1 = std::chrono::high_resolution_clock::now();

            // Sync server run speed to camera controller
            if (renderer && gameHandler && renderer->getCameraController()) {
                renderer->getCameraController()->setRunSpeedOverride(gameHandler->getServerRunSpeed());
            }

            bool onTaxi = gameHandler &&
                          (gameHandler->isOnTaxiFlight() ||
                           gameHandler->isTaxiMountActive() ||
                           gameHandler->isTaxiActivationPending());
            bool onTransportNow = gameHandler && gameHandler->isOnTransport();
            if (worldEntryMovementGraceTimer_ > 0.0f) {
                worldEntryMovementGraceTimer_ -= deltaTime;
            }
            if (renderer && renderer->getCameraController()) {
                const bool externallyDrivenMotion = onTaxi || onTransportNow || chargeActive_;
                // Keep physics frozen (externalFollow) during landing clamp when terrain
                // hasn't loaded yet — prevents gravity from pulling player through void.
                bool landingClampActive = !onTaxi && taxiLandingClampTimer_ > 0.0f &&
                                          worldEntryMovementGraceTimer_ <= 0.0f &&
                                          !gameHandler->isMounted();
                renderer->getCameraController()->setExternalFollow(externallyDrivenMotion || landingClampActive);
                renderer->getCameraController()->setExternalMoving(externallyDrivenMotion);
                if (externallyDrivenMotion) {
                    // Drop any stale local movement toggles while server drives taxi motion.
                    renderer->getCameraController()->clearMovementInputs();
                    taxiLandingClampTimer_ = 0.0f;
                }
                if (lastTaxiFlight_ && !onTaxi) {
                    renderer->getCameraController()->clearMovementInputs();
                    // Keep clamping until terrain loads at landing position.
                    // Timer only counts down once a valid floor is found.
                    taxiLandingClampTimer_ = 2.0f;
                }
                if (landingClampActive) {
                    if (renderer && gameHandler) {
                        glm::vec3 p = renderer->getCharacterPosition();
                        std::optional<float> terrainFloor;
                        std::optional<float> wmoFloor;
                        std::optional<float> m2Floor;
                        if (renderer->getTerrainManager()) {
                            terrainFloor = renderer->getTerrainManager()->getHeightAt(p.x, p.y);
                        }
                        if (renderer->getWMORenderer()) {
                            // Probe from above so we can recover when current Z is already below floor.
                            wmoFloor = renderer->getWMORenderer()->getFloorHeight(p.x, p.y, p.z + 40.0f);
                        }
                        if (renderer->getM2Renderer()) {
                            // Include M2 floors (bridges/platforms) in landing recovery.
                            m2Floor = renderer->getM2Renderer()->getFloorHeight(p.x, p.y, p.z + 40.0f);
                        }

                        std::optional<float> targetFloor;
                        if (terrainFloor) targetFloor = terrainFloor;
                        if (wmoFloor && (!targetFloor || *wmoFloor > *targetFloor)) targetFloor = wmoFloor;
                        if (m2Floor && (!targetFloor || *m2Floor > *targetFloor)) targetFloor = m2Floor;

                        if (targetFloor) {
                            // Floor found — snap player to it and start countdown to release
                            float targetZ = *targetFloor + 0.10f;
                            if (std::abs(p.z - targetZ) > 0.05f) {
                                p.z = targetZ;
                                renderer->getCharacterPosition() = p;
                                glm::vec3 canonical = core::coords::renderToCanonical(p);
                                gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
                                gameHandler->sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
                            }
                            taxiLandingClampTimer_ -= deltaTime;
                        }
                        // No floor found: don't decrement timer, keep player frozen until terrain loads
                    }
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
                // Keep taxi streaming responsive so flight paths don't outrun terrain/model uploads.
                renderer->getTerrainManager()->setUpdateInterval(onTaxi ? 0.1f : 0.1f);
                renderer->getTerrainManager()->setLoadRadius(onTaxi ? 3 : 4);
                renderer->getTerrainManager()->setUnloadRadius(onTaxi ? 6 : 7);
                renderer->getTerrainManager()->setTaxiStreamingMode(onTaxi);
            }
            lastTaxiFlight_ = onTaxi;

            // Sync character render position ↔ canonical WoW coords each frame
            if (renderer && gameHandler) {
                bool onTransport = gameHandler->isOnTransport();

                // Debug: Log transport state changes
                static bool wasOnTransport = false;
                if (onTransport != wasOnTransport) {
                    LOG_DEBUG("Transport state changed: onTransport=", onTransport,
                             " guid=0x", std::hex, gameHandler->getPlayerTransportGuid(), std::dec);
                    wasOnTransport = onTransport;
                }

                if (onTaxi) {
                    auto playerEntity = gameHandler->getEntityManager().getEntity(gameHandler->getPlayerGuid());
                    glm::vec3 canonical(0.0f);
                    bool haveCanonical = false;
                    if (playerEntity) {
                        canonical = glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ());
                        haveCanonical = true;
                    } else {
                        // Fallback for brief entity gaps during taxi start/updates:
                        // movementInfo is still updated by client taxi simulation.
                        const auto& move = gameHandler->getMovementInfo();
                        canonical = glm::vec3(move.x, move.y, move.z);
                        haveCanonical = true;
                    }
                    if (haveCanonical) {
                        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
                        renderer->getCharacterPosition() = renderPos;
                        if (renderer->getCameraController()) {
                            glm::vec3* followTarget = renderer->getCameraController()->getFollowTargetMutable();
                            if (followTarget) {
                                *followTarget = renderPos;
                            }
                        }
                    }
                } else if (onTransport) {
                    // Transport mode: compose world position from transport transform + local offset
                    glm::vec3 canonical = gameHandler->getComposedWorldPosition();
                    glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
                    renderer->getCharacterPosition() = renderPos;
                    // Keep movementInfo in lockstep with composed transport world position.
                    gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
                    // Update camera follow target
                    if (renderer->getCameraController()) {
                        glm::vec3* followTarget = renderer->getCameraController()->getFollowTargetMutable();
                        if (followTarget) {
                            *followTarget = renderPos;
                        }
                    }
                } else if (chargeActive_) {
                    // Warrior Charge: lerp position from start to end using smoothstep
                    chargeTimer_ += deltaTime;
                    float t = std::min(chargeTimer_ / chargeDuration_, 1.0f);
                    // smoothstep for natural acceleration/deceleration
                    float s = t * t * (3.0f - 2.0f * t);
                    glm::vec3 renderPos = chargeStartPos_ + (chargeEndPos_ - chargeStartPos_) * s;
                    renderer->getCharacterPosition() = renderPos;

                    // Keep facing toward target and emit charge effect
                    glm::vec3 dir = chargeEndPos_ - chargeStartPos_;
                    if (glm::length(dir) > 0.01f) {
                        dir = glm::normalize(dir);
                        float yawDeg = glm::degrees(std::atan2(dir.x, dir.y));
                        renderer->setCharacterYaw(yawDeg);
                        renderer->emitChargeEffect(renderPos, dir);
                    }

                    // Sync to game handler
                    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
                    gameHandler->setPosition(canonical.x, canonical.y, canonical.z);

                    // Update camera follow target
                    if (renderer->getCameraController()) {
                        glm::vec3* followTarget = renderer->getCameraController()->getFollowTargetMutable();
                        if (followTarget) {
                            *followTarget = renderPos;
                        }
                    }

                    // Charge complete
                    if (t >= 1.0f) {
                        chargeActive_ = false;
                        renderer->setCharging(false);
                        renderer->stopChargeEffect();
                        renderer->getCameraController()->setExternalFollow(false);
                        renderer->getCameraController()->setExternalMoving(false);

                        // Snap to melee range of target's CURRENT position (it may have moved)
                        if (chargeTargetGuid_ != 0) {
                            auto targetEntity = gameHandler->getEntityManager().getEntity(chargeTargetGuid_);
                            if (targetEntity) {
                                glm::vec3 targetCanonical(targetEntity->getX(), targetEntity->getY(), targetEntity->getZ());
                                glm::vec3 targetRender = core::coords::canonicalToRender(targetCanonical);
                                glm::vec3 toTarget = targetRender - renderPos;
                                float d = glm::length(toTarget);
                                if (d > 1.5f) {
                                    // Place us 1.5 units from target (well within 8-unit melee range)
                                    glm::vec3 snapPos = targetRender - glm::normalize(toTarget) * 1.5f;
                                    renderer->getCharacterPosition() = snapPos;
                                    glm::vec3 snapCanonical = core::coords::renderToCanonical(snapPos);
                                    gameHandler->setPosition(snapCanonical.x, snapCanonical.y, snapCanonical.z);
                                    if (renderer->getCameraController()) {
                                        glm::vec3* ft = renderer->getCameraController()->getFollowTargetMutable();
                                        if (ft) *ft = snapPos;
                                    }
                                }
                            }
                            gameHandler->startAutoAttack(chargeTargetGuid_);
                            renderer->triggerMeleeSwing();
                        }

                        // Send movement heartbeat so server knows our new position
                        gameHandler->sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
                    }
                } else {
                    glm::vec3 renderPos = renderer->getCharacterPosition();
                    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
                    gameHandler->setPosition(canonical.x, canonical.y, canonical.z);

                    // Sync orientation: camera yaw (degrees) → WoW orientation (radians)
                    float yawDeg = renderer->getCharacterYaw();
                    // Keep all game-side orientation in canonical space.
                    // We historically sent serverYaw = radians(yawDeg - 90). With the new
                    // canonical<->server mapping (serverYaw = PI/2 - canonicalYaw), the
                    // equivalent canonical yaw is radians(180 - yawDeg).
                    float canonicalYaw = core::coords::normalizeAngleRad(glm::radians(180.0f - yawDeg));
                    gameHandler->setOrientation(canonicalYaw);

                    // Send MSG_MOVE_SET_FACING when the player changes facing direction
                    // (e.g. via mouse-look). Without this, the server predicts movement in
                    // the old facing and position-corrects on the next heartbeat — the
                    // micro-teleporting the GM observed.
                    // Skip while keyboard-turning: the server tracks that via TURN_LEFT/RIGHT flags.
                    facingSendCooldown_ -= deltaTime;
                    const auto& mi = gameHandler->getMovementInfo();
                    constexpr uint32_t kTurnFlags =
                        static_cast<uint32_t>(game::MovementFlags::TURN_LEFT) |
                        static_cast<uint32_t>(game::MovementFlags::TURN_RIGHT);
                    bool keyboardTurning = (mi.flags & kTurnFlags) != 0;
                    if (!keyboardTurning && facingSendCooldown_ <= 0.0f) {
                        float yawDiff = core::coords::normalizeAngleRad(canonicalYaw - lastSentCanonicalYaw_);
                        if (std::abs(yawDiff) > glm::radians(3.0f)) {
                            gameHandler->sendMovement(game::Opcode::MSG_MOVE_SET_FACING);
                            lastSentCanonicalYaw_ = canonicalYaw;
                            facingSendCooldown_ = 0.1f;  // max 10 Hz
                        }
                    }
                }
            }

            // Keep creature render instances aligned with authoritative entity positions.
            // This prevents desync where target circles move with server entities but
            // creature models remain at stale spawn positions.
            if (renderer && gameHandler && renderer->getCharacterRenderer()) {
                auto* charRenderer = renderer->getCharacterRenderer();
                glm::vec3 playerPos(0.0f);
                glm::vec3 playerRenderPos(0.0f);
                bool havePlayerPos = false;
                float playerCollisionRadius = 0.65f;
                if (auto playerEntity = gameHandler->getEntityManager().getEntity(gameHandler->getPlayerGuid())) {
                    playerPos = glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ());
                    playerRenderPos = core::coords::canonicalToRender(playerPos);
                    havePlayerPos = true;
                    glm::vec3 pc;
                    float pr = 0.0f;
                    if (getRenderBoundsForGuid(gameHandler->getPlayerGuid(), pc, pr)) {
                        playerCollisionRadius = std::clamp(pr * 0.35f, 0.45f, 1.1f);
                    }
                }
                const float syncRadiusSq = 320.0f * 320.0f;
                for (const auto& [guid, instanceId] : creatureInstances_) {
                    auto entity = gameHandler->getEntityManager().getEntity(guid);
                    if (!entity || entity->getType() != game::ObjectType::UNIT) continue;

                    glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
                    if (havePlayerPos) {
                        glm::vec3 d = canonical - playerPos;
                        if (glm::dot(d, d) > syncRadiusSq) continue;
                    }

                    glm::vec3 renderPos = core::coords::canonicalToRender(canonical);

                    // Visual collision guard: keep hostile melee units from rendering inside the
                    // player's model while attacking. This is client-side only (no server position change).
                    auto unit = std::static_pointer_cast<game::Unit>(entity);
                    const uint64_t currentTargetGuid = gameHandler->hasTarget() ? gameHandler->getTargetGuid() : 0;
                    const uint64_t autoAttackGuid = gameHandler->getAutoAttackTargetGuid();
                    const bool isCombatTarget = (guid == currentTargetGuid || guid == autoAttackGuid);
                    bool clipGuardEligible = havePlayerPos &&
                                             unit->getHealth() > 0 &&
                                             (unit->isHostile() ||
                                              gameHandler->isAggressiveTowardPlayer(guid) ||
                                              isCombatTarget);
                    if (clipGuardEligible) {
                        float creatureCollisionRadius = 0.8f;
                        glm::vec3 cc;
                        float cr = 0.0f;
                        if (getRenderBoundsForGuid(guid, cc, cr)) {
                            creatureCollisionRadius = std::clamp(cr * 0.45f, 0.65f, 1.9f);
                        }

                        float minSep = std::max(playerCollisionRadius + creatureCollisionRadius, 1.9f);
                        if (isCombatTarget) {
                            // Stronger spacing for the actively engaged attacker to avoid bite-overlap.
                            minSep = std::max(minSep, 2.2f);
                        }

                        // Species/model-specific spacing for wolf-like creatures (their lunge anims
                        // often put head/torso inside the player capsule).
                        auto mit = creatureModelIds_.find(guid);
                        if (mit != creatureModelIds_.end()) {
                            if (const auto* md = charRenderer->getModelData(mit->second)) {
                                std::string modelName = md->name;
                                std::transform(modelName.begin(), modelName.end(), modelName.begin(),
                                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                                if (modelName.find("wolf") != std::string::npos ||
                                    modelName.find("worg") != std::string::npos) {
                                    minSep = std::max(minSep, 2.45f);
                                }
                            }
                        }

                        glm::vec2 d2(renderPos.x - playerRenderPos.x, renderPos.y - playerRenderPos.y);
                        float distSq2 = glm::dot(d2, d2);
                        if (distSq2 < (minSep * minSep)) {
                            glm::vec2 dir2(1.0f, 0.0f);
                            if (distSq2 > 1e-6f) {
                                dir2 = d2 * (1.0f / std::sqrt(distSq2));
                            }
                            glm::vec2 clamped2 = glm::vec2(playerRenderPos.x, playerRenderPos.y) + dir2 * minSep;
                            renderPos.x = clamped2.x;
                            renderPos.y = clamped2.y;
                        }
                    }

                    auto posIt = creatureRenderPosCache_.find(guid);
                    if (posIt == creatureRenderPosCache_.end()) {
                        charRenderer->setInstancePosition(instanceId, renderPos);
                        creatureRenderPosCache_[guid] = renderPos;
                    } else {
                        const glm::vec3 prevPos = posIt->second;
                        const glm::vec2 delta2(renderPos.x - prevPos.x, renderPos.y - prevPos.y);
                        float planarDist = glm::length(delta2);
                        float dz = std::abs(renderPos.z - prevPos.z);

                        const bool deadOrCorpse = unit->getHealth() == 0;
                        const bool largeCorrection = (planarDist > 6.0f) || (dz > 3.0f);
                        if (deadOrCorpse || largeCorrection) {
                            charRenderer->setInstancePosition(instanceId, renderPos);
                        } else if (planarDist > 0.03f || dz > 0.08f) {
                            // Use movement interpolation so step/run animation can play.
                            float duration = std::clamp(planarDist / 5.5f, 0.05f, 0.22f);
                            charRenderer->moveInstanceTo(instanceId, renderPos, duration);
                        }
                        posIt->second = renderPos;
                    }
                    float renderYaw = entity->getOrientation() + glm::radians(90.0f);
                    charRenderer->setInstanceRotation(instanceId, glm::vec3(0.0f, 0.0f, renderYaw));
                }
            }

            // Movement heartbeat is sent from GameHandler::update() to avoid
            // duplicate packets from multiple update loops.

            auto sync2 = std::chrono::high_resolution_clock::now();
            syncTime += std::chrono::duration<float, std::milli>(sync2 - sync1).count();

            // Log profiling every 60 frames
            if (++appProfileCounter >= 60) {
                LOG_DEBUG("APP UPDATE PROFILE (60 frames): gameHandler=", ghTime / 60.0f,
                         "ms world=", worldTime / 60.0f, "ms spawn=", spawnTime / 60.0f,
                         "ms creatureQ=", creatureQTime / 60.0f, "ms goQ=", goQTime / 60.0f,
                         "ms mount=", mountTime / 60.0f, "ms npcMgr=", npcMgrTime / 60.0f,
                         "ms questMark=", questMarkTime / 60.0f, "ms sync=", syncTime / 60.0f, "ms");
                appProfileCounter = 0;
                ghTime = worldTime = spawnTime = 0.0f;
                creatureQTime = goQTime = mountTime = 0.0f;
                npcMgrTime = questMarkTime = syncTime = 0.0f;
            }
            break;
        }

        case AppState::DISCONNECTED:
            // Handle disconnection
            break;
    }

    // Update renderer (camera, etc.) only when in-game
    static int rendererProfileCounter = 0;
    static float rendererTime = 0.0f, uiTime = 0.0f;

    auto r1 = std::chrono::high_resolution_clock::now();
    if (renderer && state == AppState::IN_GAME) {
        renderer->update(deltaTime);
    }
    auto r2 = std::chrono::high_resolution_clock::now();
    rendererTime += std::chrono::duration<float, std::milli>(r2 - r1).count();

    // Update UI
    auto u1 = std::chrono::high_resolution_clock::now();
    if (uiManager) {
        uiManager->update(deltaTime);
    }
    auto u2 = std::chrono::high_resolution_clock::now();
    uiTime += std::chrono::duration<float, std::milli>(u2 - u1).count();

    if (state == AppState::IN_GAME && ++rendererProfileCounter >= 60) {
        LOG_DEBUG("RENDERER/UI PROFILE (60 frames): renderer=", rendererTime / 60.0f,
                 "ms ui=", uiTime / 60.0f, "ms");
        rendererProfileCounter = 0;
        rendererTime = uiTime = 0.0f;
    }
}

void Application::render() {
    if (!renderer) {
        return;
    }

    renderer->beginFrame();

    // Only render 3D world when in-game
    if (state == AppState::IN_GAME) {
        if (world) {
            renderer->renderWorld(world.get(), gameHandler.get());
        } else {
            renderer->renderWorld(nullptr, gameHandler.get());
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

        uint32_t realmId = 0;
        {
            // WotLK AUTH_SESSION includes a RealmID field; some servers reject if it's wrong/zero.
            const auto& realms = authHandler->getRealms();
            for (const auto& r : realms) {
                if (r.name == realmName && r.address == realmAddress) {
                    realmId = r.id;
                    break;
                }
            }
            LOG_INFO("Selected realmId=", realmId);
        }

        uint32_t clientBuild = 12340; // default WotLK
        if (expansionRegistry_) {
            auto* profile = expansionRegistry_->getActive();
            if (profile) clientBuild = profile->worldBuild;
        }
        if (gameHandler->connect(host, port, sessionKey, accountName, clientBuild, realmId)) {
            LOG_INFO("Connected to world server, transitioning to character selection");
            setState(AppState::CHARACTER_SELECTION);
        } else {
            LOG_ERROR("Failed to connect to world server");
        }
    });

    // Realm screen back button - return to login
    uiManager->getRealmScreen().setOnBack([this]() {
        if (authHandler) {
            authHandler->disconnect();
        }
        uiManager->getRealmScreen().reset();
        setState(AppState::AUTHENTICATION);
    });

    // Character selection callback
    uiManager->getCharacterScreen().setOnCharacterSelected([this](uint64_t characterGuid) {
        LOG_INFO("Character selected: GUID=0x", std::hex, characterGuid, std::dec);
        // Always set the active character GUID
        if (gameHandler) {
            gameHandler->setActiveCharacterGuid(characterGuid);
        }
        // Keep CHARACTER_SELECTION active until world entry is fully loaded.
        // This avoids exposing pre-load hitching before the loading screen/intro.
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

    // Character login failure callback
    gameHandler->setCharLoginFailCallback([this](const std::string& reason) {
        LOG_WARNING("Character login failed: ", reason);
        setState(AppState::CHARACTER_SELECTION);
        uiManager->getCharacterScreen().setStatus("Login failed: " + reason, true);
    });

    // World entry callback (online mode) - load terrain when entering world
    gameHandler->setWorldEntryCallback([this](uint32_t mapId, float x, float y, float z) {
        LOG_INFO("Online world entry: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")");

        // Same-map teleport (taxi landing, GM teleport on same continent):
        // just update position, let terrain streamer handle tile loading incrementally.
        // A full reload is only needed on first entry or map change.
        if (mapId == loadedMapId_ && renderer && renderer->getTerrainManager()) {
            LOG_INFO("Same-map teleport (map ", mapId, "), skipping full world reload");
            glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            renderer->getCharacterPosition() = renderPos;
            if (renderer->getCameraController()) {
                auto* ft = renderer->getCameraController()->getFollowTargetMutable();
                if (ft) *ft = renderPos;
            }
            worldEntryMovementGraceTimer_ = 2.0f;
            taxiLandingClampTimer_ = 0.0f;
            lastTaxiFlight_ = false;
            return;
        }

        worldEntryMovementGraceTimer_ = 2.0f;
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        loadOnlineWorldTerrain(mapId, x, y, z);
        loadedMapId_ = mapId;
    });

    auto sampleBestFloorAt = [this](float x, float y, float probeZ) -> std::optional<float> {
        std::optional<float> terrainFloor;
        std::optional<float> wmoFloor;
        std::optional<float> m2Floor;

        if (renderer && renderer->getTerrainManager()) {
            terrainFloor = renderer->getTerrainManager()->getHeightAt(x, y);
        }
        if (renderer && renderer->getWMORenderer()) {
            wmoFloor = renderer->getWMORenderer()->getFloorHeight(x, y, probeZ);
        }
        if (renderer && renderer->getM2Renderer()) {
            m2Floor = renderer->getM2Renderer()->getFloorHeight(x, y, probeZ);
        }

        std::optional<float> best;
        if (terrainFloor) best = terrainFloor;
        if (wmoFloor && (!best || *wmoFloor > *best)) best = wmoFloor;
        if (m2Floor && (!best || *m2Floor > *best)) best = m2Floor;
        return best;
    };

    auto clearStuckMovement = [this]() {
        if (renderer && renderer->getCameraController()) {
            renderer->getCameraController()->clearMovementInputs();
        }
        if (gameHandler) {
            gameHandler->forceClearTaxiAndMovementState();
            gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP);
            gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP_STRAFE);
            gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP_TURN);
            gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP_SWIM);
            gameHandler->sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
        }
    };

    auto syncTeleportedPositionToServer = [this](const glm::vec3& renderPos) {
        if (!gameHandler) return;
        glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
        gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
        gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP);
        gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP_STRAFE);
        gameHandler->sendMovement(game::Opcode::MSG_MOVE_STOP_TURN);
        gameHandler->sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
    };

    auto forceServerTeleportCommand = [this](const glm::vec3& renderPos) {
        if (!gameHandler) return;
        // Server-authoritative reset first, then teleport.
        gameHandler->sendChatMessage(game::ChatType::SAY, ".revive", "");
        gameHandler->sendChatMessage(game::ChatType::SAY, ".dismount", "");

        glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
        glm::vec3 serverPos = core::coords::canonicalToServer(canonical);
        std::ostringstream cmd;
        cmd.setf(std::ios::fixed);
        cmd.precision(3);
        cmd << ".go xyz "
            << serverPos.x << " "
            << serverPos.y << " "
            << serverPos.z << " "
            << gameHandler->getCurrentMapId() << " "
            << gameHandler->getMovementInfo().orientation;
        gameHandler->sendChatMessage(game::ChatType::SAY, cmd.str(), "");
    };

    // /unstuck — nudge player forward and snap to floor at destination.
    gameHandler->setUnstuckCallback([this, sampleBestFloorAt, clearStuckMovement, syncTeleportedPositionToServer, forceServerTeleportCommand]() {
        if (!renderer || !renderer->getCameraController()) return;
        worldEntryMovementGraceTimer_ = std::max(worldEntryMovementGraceTimer_, 1.5f);
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        clearStuckMovement();
        auto* cc = renderer->getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;

        glm::vec3 pos = *ft;

        // Always nudge forward first to escape stuck geometry (M2 models, collision seams).
        if (gameHandler) {
            float renderYaw = gameHandler->getMovementInfo().orientation + glm::radians(90.0f);
            pos.x += std::cos(renderYaw) * 5.0f;
            pos.y += std::sin(renderYaw) * 5.0f;
        }

        // Sample floor at the DESTINATION position (after nudge).
        if (auto floor = sampleBestFloorAt(pos.x, pos.y, pos.z + 60.0f)) {
            pos.z = *floor + 0.2f;
        } else {
            pos.z += 20.0f;
        }

        cc->teleportTo(pos);
        syncTeleportedPositionToServer(pos);
        forceServerTeleportCommand(pos);
        clearStuckMovement();
        LOG_INFO("Unstuck: nudged forward and snapped to floor");
    });

    // /unstuckgy — stronger recovery: safe/home position, then sampled floor fallback.
    gameHandler->setUnstuckGyCallback([this, sampleBestFloorAt, clearStuckMovement, syncTeleportedPositionToServer, forceServerTeleportCommand]() {
        if (!renderer || !renderer->getCameraController()) return;
        worldEntryMovementGraceTimer_ = std::max(worldEntryMovementGraceTimer_, 1.5f);
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        clearStuckMovement();
        auto* cc = renderer->getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;

        // Try last safe position first (nearby, terrain already loaded)
        if (cc->hasLastSafePosition()) {
            glm::vec3 safePos = cc->getLastSafePosition();
            safePos.z += 5.0f;
            cc->teleportTo(safePos);
            syncTeleportedPositionToServer(safePos);
            forceServerTeleportCommand(safePos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to last safe position");
            return;
        }

        uint32_t bindMap = 0;
        glm::vec3 bindPos(0.0f);
        if (gameHandler && gameHandler->getHomeBind(bindMap, bindPos) &&
            bindMap == gameHandler->getCurrentMapId()) {
            bindPos.z += 2.0f;
            cc->teleportTo(bindPos);
            syncTeleportedPositionToServer(bindPos);
            forceServerTeleportCommand(bindPos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to home bind position");
            return;
        }

        // No safe/bind position — try current XY with a high floor probe.
        glm::vec3 pos = *ft;
        if (auto floor = sampleBestFloorAt(pos.x, pos.y, pos.z + 120.0f)) {
            pos.z = *floor + 0.5f;
            cc->teleportTo(pos);
            syncTeleportedPositionToServer(pos);
            forceServerTeleportCommand(pos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to sampled floor");
            return;
        }

        // Last fallback: high snap to clear deeply bad geometry.
        pos.z += 60.0f;
        cc->teleportTo(pos);
        syncTeleportedPositionToServer(pos);
        forceServerTeleportCommand(pos);
        clearStuckMovement();
        LOG_INFO("Unstuck: high fallback snap");
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
        // Queue spawns to avoid hanging when many creatures appear at once.
        // Deduplicate so repeated updates don't flood pending queue.
        if (creatureInstances_.count(guid)) return;
        if (pendingCreatureSpawnGuids_.count(guid)) return;
        pendingCreatureSpawns_.push_back({guid, displayId, x, y, z, orientation});
        pendingCreatureSpawnGuids_.insert(guid);
    });

    // Player spawn callback (online mode) - spawn player models with correct textures
    gameHandler->setPlayerSpawnCallback([this](uint64_t guid,
                                              uint32_t /*displayId*/,
                                              uint8_t raceId,
                                              uint8_t genderId,
                                              uint32_t appearanceBytes,
                                              uint8_t facialFeatures,
                                              float x, float y, float z, float orientation) {
        // Skip local player — already spawned as the main character
        uint64_t localGuid = gameHandler ? gameHandler->getPlayerGuid() : 0;
        uint64_t activeGuid = gameHandler ? gameHandler->getActiveCharacterGuid() : 0;
        if ((localGuid != 0 && guid == localGuid) ||
            (activeGuid != 0 && guid == activeGuid) ||
            (spawnedPlayerGuid_ != 0 && guid == spawnedPlayerGuid_)) {
            return;
        }
        if (playerInstances_.count(guid)) return;
        if (pendingPlayerSpawnGuids_.count(guid)) return;
        pendingPlayerSpawns_.push_back({guid, raceId, genderId, appearanceBytes, facialFeatures, x, y, z, orientation});
        pendingPlayerSpawnGuids_.insert(guid);
    });

    // Online player equipment callback - apply armor geosets/skin overlays per player instance.
    gameHandler->setPlayerEquipmentCallback([this](uint64_t guid,
                                                  const std::array<uint32_t, 19>& displayInfoIds,
                                                  const std::array<uint8_t, 19>& inventoryTypes) {
        // Queue equipment compositing instead of doing it immediately —
        // compositeWithRegions is expensive (file I/O + CPU blit + GPU upload)
        // and causes frame stutters if multiple players update at once.
        deferredEquipmentQueue_.push_back({guid, {displayInfoIds, inventoryTypes}});
    });

    // Creature despawn callback (online mode) - remove creature models
    gameHandler->setCreatureDespawnCallback([this](uint64_t guid) {
        despawnOnlineCreature(guid);
    });

    gameHandler->setPlayerDespawnCallback([this](uint64_t guid) {
        despawnOnlinePlayer(guid);
    });

    // GameObject spawn callback (online mode) - spawn static models (mailboxes, etc.)
    gameHandler->setGameObjectSpawnCallback([this](uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation) {
        pendingGameObjectSpawns_.push_back({guid, entry, displayId, x, y, z, orientation});
    });

    // GameObject despawn callback (online mode) - remove static models
    gameHandler->setGameObjectDespawnCallback([this](uint64_t guid) {
        despawnOnlineGameObject(guid);
    });

    // Charge callback — warrior rushes toward target
    gameHandler->setChargeCallback([this](uint64_t targetGuid, float tx, float ty, float tz) {
        if (!renderer || !renderer->getCameraController() || !gameHandler) return;

        // Get current player position in render coords
        glm::vec3 startRender = renderer->getCharacterPosition();
        // Convert target from canonical to render
        glm::vec3 targetRender = core::coords::canonicalToRender(glm::vec3(tx, ty, tz));

        // Compute direction and stop 2.0 units short (melee reach)
        glm::vec3 dir = targetRender - startRender;
        float dist = glm::length(dir);
        if (dist < 3.0f) return; // Too close, nothing to do
        glm::vec3 dirNorm = dir / dist;
        glm::vec3 endRender = targetRender - dirNorm * 2.0f;

        // Face toward target BEFORE starting charge
        float yawRad = std::atan2(dirNorm.x, dirNorm.y);
        float yawDeg = glm::degrees(yawRad);
        renderer->setCharacterYaw(yawDeg);
        // Sync canonical orientation to server so it knows we turned
        float canonicalYaw = core::coords::normalizeAngleRad(glm::radians(180.0f - yawDeg));
        gameHandler->setOrientation(canonicalYaw);
        gameHandler->sendMovement(game::Opcode::MSG_MOVE_SET_FACING);

        // Set charge state
        chargeActive_ = true;
        chargeTimer_ = 0.0f;
        chargeDuration_ = std::max(dist / 25.0f, 0.3f); // ~25 units/sec
        chargeStartPos_ = startRender;
        chargeEndPos_ = endRender;
        chargeTargetGuid_ = targetGuid;

        // Disable player input, play charge animation
        renderer->getCameraController()->setExternalFollow(true);
        renderer->getCameraController()->clearMovementInputs();
        renderer->setCharging(true);

        // Start charge visual effect (red haze + dust)
        glm::vec3 chargeDir = glm::normalize(endRender - startRender);
        renderer->startChargeEffect(startRender, chargeDir);

        // Play charge whoosh sound (try multiple paths)
        auto& audio = audio::AudioEngine::instance();
        if (!audio.playSound2D("Sound\\Spells\\Charge.wav", 0.8f)) {
            if (!audio.playSound2D("Sound\\Spells\\charge.wav", 0.8f)) {
                if (!audio.playSound2D("Sound\\Spells\\SpellCharge.wav", 0.8f)) {
                    // Fallback: weapon whoosh
                    audio.playSound2D("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge1.wav", 0.9f);
                }
            }
        }
    });

    // Level-up callback — play sound, cheer emote, and trigger UI ding overlay + 3D effect
    gameHandler->setLevelUpCallback([this](uint32_t newLevel) {
        if (uiManager) {
            uiManager->getGameScreen().triggerDing(newLevel);
        }
        if (renderer) {
            renderer->triggerLevelUpEffect(renderer->getCharacterPosition());
        }
    });

    // Other player level-up callback — trigger 3D effect + chat notification
    gameHandler->setOtherPlayerLevelUpCallback([this](uint64_t guid, uint32_t newLevel) {
        if (!gameHandler || !renderer) return;

        // Trigger 3D effect at the other player's position
        auto entity = gameHandler->getEntityManager().getEntity(guid);
        if (entity) {
            glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            renderer->triggerLevelUpEffect(renderPos);
        }

        // Show chat message if in group
        if (gameHandler->isInGroup()) {
            std::string name = gameHandler->getCachedPlayerName(guid);
            if (name.empty()) name = "A party member";
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = name + " has reached level " + std::to_string(newLevel) + "!";
            gameHandler->addLocalChatMessage(msg);
        }
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

        // Sample waypoints along path and gather tiles.
        // Denser sampling + neighbor coverage reduces in-flight stream spikes.
        const size_t stride = 2;
        for (size_t i = 0; i < path.size(); i += stride) {
            const auto& waypoint = path[i];
            glm::vec3 renderPos = core::coords::canonicalToRender(waypoint);
            int tileX = static_cast<int>(32 - (renderPos.x / 533.33333f));
            int tileY = static_cast<int>(32 - (renderPos.y / 533.33333f));

            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        int nx = tileX + dx;
                        int ny = tileY + dy;
                        if (nx >= 0 && nx <= 63 && ny >= 0 && ny <= 63) {
                            uniqueTiles.insert({nx, ny});
                        }
                    }
                }
            }
        }
        // Ensure final destination tile is included.
        if (!path.empty()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(path.back());
            int tileX = static_cast<int>(32 - (renderPos.x / 533.33333f));
            int tileY = static_cast<int>(32 - (renderPos.y / 533.33333f));
            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        int nx = tileX + dx;
                        int ny = tileY + dy;
                        if (nx >= 0 && nx <= 63 && ny >= 0 && ny <= 63) {
                            uniqueTiles.insert({nx, ny});
                        }
                    }
                }
            }
        }

        std::vector<std::pair<int, int>> tilesToLoad(uniqueTiles.begin(), uniqueTiles.end());
        if (tilesToLoad.size() > 512) {
            tilesToLoad.resize(512);
        }
        LOG_INFO("Precaching ", tilesToLoad.size(), " tiles for taxi route");
        renderer->getTerrainManager()->precacheTiles(tilesToLoad);
    });

    // Taxi orientation callback - update mount rotation during flight
    gameHandler->setTaxiOrientationCallback([this](float yaw, float pitch, float roll) {
        if (renderer && renderer->getCameraController()) {
            // Taxi callback now provides render-space yaw directly.
            float yawDegrees = glm::degrees(yaw);
            renderer->getCameraController()->setFacingYaw(yawDegrees);
            renderer->setCharacterYaw(yawDegrees);
            // Set mount pitch and roll for realistic flight animation
            renderer->setMountPitchRoll(pitch, roll);
        }
    });

    // Taxi flight start callback - keep non-blocking to avoid hitching at takeoff.
    gameHandler->setTaxiFlightStartCallback([this]() {
        if (renderer && renderer->getTerrainManager() && renderer->getM2Renderer()) {
            LOG_INFO("Taxi flight start: incremental terrain/M2 streaming active");
            uint32_t m2Count = renderer->getM2Renderer()->getModelCount();
            uint32_t instCount = renderer->getM2Renderer()->getInstanceCount();
            LOG_INFO("Current M2 VRAM state: ", m2Count, " models (", instCount, " instances)");
        }
    });

    // Creature move callback (online mode) - update creature positions
    gameHandler->setCreatureMoveCallback([this](uint64_t guid, float x, float y, float z, uint32_t durationMs) {
        if (!renderer || !renderer->getCharacterRenderer()) return;
        uint32_t instanceId = 0;
        auto pit = playerInstances_.find(guid);
        if (pit != playerInstances_.end()) instanceId = pit->second;
        else {
            auto it = creatureInstances_.find(guid);
            if (it != creatureInstances_.end()) instanceId = it->second;
        }
        if (instanceId != 0) {
            glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
            float durationSec = static_cast<float>(durationMs) / 1000.0f;
            renderer->getCharacterRenderer()->moveInstanceTo(instanceId, renderPos, durationSec);
        }
    });

    gameHandler->setGameObjectMoveCallback([this](uint64_t guid, float x, float y, float z, float orientation) {
        auto it = gameObjectInstances_.find(guid);
        if (it == gameObjectInstances_.end() || !renderer) {
            return;
        }
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        auto& info = it->second;
        if (info.isWmo) {
            if (auto* wr = renderer->getWMORenderer()) {
                glm::mat4 transform(1.0f);
                transform = glm::translate(transform, renderPos);
                transform = glm::rotate(transform, orientation, glm::vec3(0, 0, 1));
                wr->setInstanceTransform(info.instanceId, transform);
            }
        } else {
            if (auto* mr = renderer->getM2Renderer()) {
                glm::mat4 transform(1.0f);
                transform = glm::translate(transform, renderPos);
                transform = glm::rotate(transform, orientation, glm::vec3(0, 0, 1));
                mr->setInstanceTransform(info.instanceId, transform);
            }
        }
    });

    // Transport spawn callback (online mode) - register transports with TransportManager
    gameHandler->setTransportSpawnCallback([this](uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation) {
        auto* transportManager = gameHandler->getTransportManager();
        if (!transportManager || !renderer) return;

        // Get the WMO instance ID from the GameObject spawn
        auto it = gameObjectInstances_.find(guid);
        if (it == gameObjectInstances_.end()) {
            LOG_WARNING("Transport spawn callback: GameObject instance not found for GUID 0x", std::hex, guid, std::dec);
            return;
        }

        uint32_t wmoInstanceId = it->second.instanceId;
        LOG_DEBUG("Registering server transport: GUID=0x", std::hex, guid, std::dec,
                 " entry=", entry, " displayId=", displayId, " wmoInstance=", wmoInstanceId,
                 " pos=(", x, ", ", y, ", ", z, ")");

        // TransportAnimation.dbc is indexed by GameObject entry
        uint32_t pathId = entry;
        const bool preferServerData = gameHandler && gameHandler->hasServerTransportUpdate(guid);

        bool clientAnim = transportManager->isClientSideAnimation();
        LOG_DEBUG("Transport spawn callback: clientAnimation=", clientAnim,
                 " guid=0x", std::hex, guid, std::dec, " entry=", entry, " pathId=", pathId,
                 " preferServer=", preferServerData);

        // Coordinates are already canonical (converted in game_handler.cpp when entity was created)
        glm::vec3 canonicalSpawnPos(x, y, z);

        // Check if we have a real path from TransportAnimation.dbc (indexed by entry).
        // AzerothCore transport entries are not always 1:1 with DBC path ids.
        const bool shipOrZeppelinDisplay =
            (displayId == 3015 || displayId == 3031 || displayId == 7546 ||
             displayId == 7446 || displayId == 1587 || displayId == 2454 ||
             displayId == 807 || displayId == 808);
        bool hasUsablePath = transportManager->hasPathForEntry(entry);
        if (shipOrZeppelinDisplay) {
            // For true transports, reject tiny XY tracks that effectively look stationary.
            hasUsablePath = transportManager->hasUsableMovingPathForEntry(entry, 25.0f);
        }

        if (preferServerData) {
            // Strict server-authoritative mode: do not infer/remap fallback routes.
            if (!hasUsablePath) {
                std::vector<glm::vec3> path = { canonicalSpawnPos };
                transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                LOG_DEBUG("Server-first strict registration: stationary fallback for GUID 0x",
                         std::hex, guid, std::dec, " entry=", entry);
            } else {
                LOG_DEBUG("Server-first transport registration: using entry DBC path for entry ", entry);
            }
        } else if (!hasUsablePath) {
            // Remap/infer path by spawn position when entry doesn't map 1:1 to DBC ids.
            // For elevators (TB lift platforms), we must allow z-only paths here.
            bool allowZOnly = (displayId == 455 || displayId == 462);
            uint32_t inferredPath = transportManager->inferDbcPathForSpawn(
                canonicalSpawnPos, 1200.0f, allowZOnly);
            if (inferredPath != 0) {
                pathId = inferredPath;
                LOG_DEBUG("Using inferred transport path ", pathId, " for entry ", entry);
            } else {
                uint32_t remappedPath = transportManager->pickFallbackMovingPath(entry, displayId);
                if (remappedPath != 0) {
                    pathId = remappedPath;
                    LOG_DEBUG("Using remapped fallback transport path ", pathId,
                             " for entry ", entry, " displayId=", displayId,
                             " (usableEntryPath=", transportManager->hasPathForEntry(entry), ")");
                } else {
                    LOG_WARNING("No TransportAnimation.dbc path for entry ", entry,
                                " - transport will be stationary");

                    // Fallback: Stationary at spawn point (wait for server to send real position)
                    std::vector<glm::vec3> path = { canonicalSpawnPos };
                    transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                }
            }
        } else {
            LOG_DEBUG("Using real transport path from TransportAnimation.dbc for entry ", entry);
        }

        // Register the transport with spawn position (prevents rendering at origin until server update)
        transportManager->registerTransport(guid, wmoInstanceId, pathId, canonicalSpawnPos, entry);

        // Server-authoritative movement - set initial position from spawn data
        glm::vec3 canonicalPos(x, y, z);
        transportManager->updateServerTransport(guid, canonicalPos, orientation);

        // If a move packet arrived before registration completed, replay latest now.
        auto pendingIt = pendingTransportMoves_.find(guid);
        if (pendingIt != pendingTransportMoves_.end()) {
            const PendingTransportMove pending = pendingIt->second;
            transportManager->updateServerTransport(guid, glm::vec3(pending.x, pending.y, pending.z), pending.orientation);
            LOG_DEBUG("Replayed queued transport move for GUID=0x", std::hex, guid, std::dec,
                     " pos=(", pending.x, ", ", pending.y, ", ", pending.z, ") orientation=", pending.orientation);
            pendingTransportMoves_.erase(pendingIt);
        }

        // For MO_TRANSPORT at (0,0,0): check if GO data is already cached with a taxiPathId
        if (glm::length(canonicalSpawnPos) < 1.0f && gameHandler) {
            auto goData = gameHandler->getCachedGameObjectInfo(entry);
            if (goData && goData->type == 15 && goData->hasData && goData->data[0] != 0) {
                uint32_t taxiPathId = goData->data[0];
                if (transportManager->hasTaxiPath(taxiPathId)) {
                    transportManager->assignTaxiPathToTransport(entry, taxiPathId);
                    LOG_DEBUG("Assigned cached TaxiPathNode path for MO_TRANSPORT entry=", entry,
                             " taxiPathId=", taxiPathId);
                }
            }
        }

        if (auto* tr = transportManager->getTransport(guid); tr) {
            LOG_DEBUG("Transport registered: guid=0x", std::hex, guid, std::dec,
                     " entry=", entry, " displayId=", displayId,
                     " pathId=", tr->pathId,
                     " mode=", (tr->useClientAnimation ? "client" : "server"),
                     " serverUpdates=", tr->serverUpdateCount);
        } else {
            LOG_DEBUG("Transport registered: guid=0x", std::hex, guid, std::dec,
                     " entry=", entry, " displayId=", displayId, " (TransportManager instance missing)");
        }
    });

    // Transport move callback (online mode) - update transport gameobject positions
    gameHandler->setTransportMoveCallback([this](uint64_t guid, float x, float y, float z, float orientation) {
        LOG_DEBUG("Transport move callback: GUID=0x", std::hex, guid, std::dec,
                 " pos=(", x, ", ", y, ", ", z, ") orientation=", orientation);

        auto* transportManager = gameHandler->getTransportManager();
        if (!transportManager) {
            LOG_WARNING("Transport move callback: TransportManager is null!");
            return;
        }

        // Check if transport exists - if not, treat this as a late spawn (reconnection/server restart)
        if (!transportManager->getTransport(guid)) {
            LOG_DEBUG("Received position update for unregistered transport 0x", std::hex, guid, std::dec,
                     " - auto-spawning from position update");

            // Get transport info from entity manager
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::GAMEOBJECT) {
                auto go = std::static_pointer_cast<game::GameObject>(entity);
                uint32_t entry = go->getEntry();
                uint32_t displayId = go->getDisplayId();

                // Find the WMO instance for this transport (should exist from earlier GameObject spawn)
                auto it = gameObjectInstances_.find(guid);
                if (it != gameObjectInstances_.end()) {
                    uint32_t wmoInstanceId = it->second.instanceId;

                    // TransportAnimation.dbc is indexed by GameObject entry
                    uint32_t pathId = entry;
                    const bool preferServerData = gameHandler && gameHandler->hasServerTransportUpdate(guid);

                    // Coordinates are already canonical (converted in game_handler.cpp)
                    glm::vec3 canonicalSpawnPos(x, y, z);

                    // Check if we have a real usable path, otherwise remap/infer/fall back to stationary.
                    const bool shipOrZeppelinDisplay =
                        (displayId == 3015 || displayId == 3031 || displayId == 7546 ||
                         displayId == 7446 || displayId == 1587 || displayId == 2454 ||
                         displayId == 807 || displayId == 808);
                    bool hasUsablePath = transportManager->hasPathForEntry(entry);
                    if (shipOrZeppelinDisplay) {
                        hasUsablePath = transportManager->hasUsableMovingPathForEntry(entry, 25.0f);
                    }

                    if (preferServerData) {
                        // Strict server-authoritative mode: no inferred/remapped fallback routes.
                        if (!hasUsablePath) {
                            std::vector<glm::vec3> path = { canonicalSpawnPos };
                            transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                            LOG_INFO("Auto-spawned transport in strict server-first mode (stationary fallback): entry=", entry,
                                     " displayId=", displayId, " wmoInstance=", wmoInstanceId);
                        } else {
                            LOG_INFO("Auto-spawned transport in server-first mode with entry DBC path: entry=", entry,
                                     " displayId=", displayId, " wmoInstance=", wmoInstanceId);
                        }
                    } else if (!hasUsablePath) {
                        bool allowZOnly = (displayId == 455 || displayId == 462);
                        uint32_t inferredPath = transportManager->inferDbcPathForSpawn(
                            canonicalSpawnPos, 1200.0f, allowZOnly);
                        if (inferredPath != 0) {
                            pathId = inferredPath;
                            LOG_INFO("Auto-spawned transport with inferred path: entry=", entry,
                                     " inferredPath=", pathId, " displayId=", displayId,
                                     " wmoInstance=", wmoInstanceId);
                        } else {
                            uint32_t remappedPath = transportManager->pickFallbackMovingPath(entry, displayId);
                            if (remappedPath != 0) {
                                pathId = remappedPath;
                                LOG_INFO("Auto-spawned transport with remapped fallback path: entry=", entry,
                                         " remappedPath=", pathId, " displayId=", displayId,
                                         " wmoInstance=", wmoInstanceId);
                            } else {
                                std::vector<glm::vec3> path = { canonicalSpawnPos };
                                transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                                LOG_INFO("Auto-spawned transport with stationary path: entry=", entry,
                                         " displayId=", displayId, " wmoInstance=", wmoInstanceId);
                            }
                        }
                    } else {
                        LOG_INFO("Auto-spawned transport with real path: entry=", entry,
                                 " displayId=", displayId, " wmoInstance=", wmoInstanceId);
                    }

                    transportManager->registerTransport(guid, wmoInstanceId, pathId, canonicalSpawnPos, entry);
                } else {
                    pendingTransportMoves_[guid] = PendingTransportMove{x, y, z, orientation};
                    LOG_DEBUG("Cannot auto-spawn transport 0x", std::hex, guid, std::dec,
                              " - WMO instance not found yet (queued move for replay)");
                    return;
                }
            } else {
                pendingTransportMoves_[guid] = PendingTransportMove{x, y, z, orientation};
                LOG_DEBUG("Cannot auto-spawn transport 0x", std::hex, guid, std::dec,
                          " - entity not found in EntityManager (queued move for replay)");
                return;
            }
        }

        // Update TransportManager's internal state (position, rotation, transform matrices)
        // This also updates the WMO renderer automatically
        // Coordinates are already canonical (converted in game_handler.cpp when entity was created)
        glm::vec3 canonicalPos(x, y, z);
        transportManager->updateServerTransport(guid, canonicalPos, orientation);

        // Move player with transport if riding it
        if (gameHandler && gameHandler->isOnTransport() && gameHandler->getPlayerTransportGuid() == guid && renderer) {
            auto* cc = renderer->getCameraController();
            if (cc) {
                glm::vec3* ft = cc->getFollowTargetMutable();
                if (ft) {
                    // Get player world position from TransportManager (handles transform properly)
                    glm::vec3 offset = gameHandler->getPlayerTransportOffset();
                    glm::vec3 worldPos = transportManager->getPlayerWorldPosition(guid, offset);
                    *ft = worldPos;
                }
            }
        }
    });

    // NPC death callback (online mode) - play death animation
    gameHandler->setNpcDeathCallback([this](uint64_t guid) {
        deadCreatureGuids_.insert(guid);
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end() && renderer && renderer->getCharacterRenderer()) {
            renderer->getCharacterRenderer()->playAnimation(it->second, 1, false); // Death
        }
    });

    // NPC respawn callback (online mode) - reset to idle animation
    gameHandler->setNpcRespawnCallback([this](uint64_t guid) {
        deadCreatureGuids_.erase(guid);
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
        // Apply expansion race/class constraints before showing the screen
        if (expansionRegistry_ && expansionRegistry_->getActive()) {
            auto* profile = expansionRegistry_->getActive();
            uiManager->getCharacterCreateScreen().setExpansionConstraints(
                profile->races, profile->classes);
        }
        uiManager->getCharacterCreateScreen().initializePreview(assetManager.get());
        setState(AppState::CHARACTER_CREATION);
    });

    // "Back" button on character screen
    uiManager->getCharacterScreen().setOnBack([this]() {
        // Disconnect from world server and reset UI state for fresh realm selection
        if (gameHandler) {
            gameHandler->disconnect();
        }
        uiManager->getRealmScreen().reset();
        uiManager->getCharacterScreen().reset();
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
                "Delete failed (code " + std::to_string(static_cast<int>(code)) + ").", true);
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
            if (!skinData.empty() && model.version >= 264) {
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
                std::string faceUpperTexturePath;
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
                        const auto* csL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
                        bool foundSkin = false;
                        bool foundUnderwear = false;
                        bool foundFaceLower = false;
                        bool foundHair = false;
                        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                            uint32_t raceId = charSectionsDbc->getUInt32(r, csL ? (*csL)["RaceID"] : 1);
                            uint32_t sexId = charSectionsDbc->getUInt32(r, csL ? (*csL)["SexID"] : 2);
                            uint32_t baseSection = charSectionsDbc->getUInt32(r, csL ? (*csL)["BaseSection"] : 3);
                            uint32_t variationIndex = charSectionsDbc->getUInt32(r, csL ? (*csL)["VariationIndex"] : 4);
                            uint32_t colorIndex = charSectionsDbc->getUInt32(r, csL ? (*csL)["ColorIndex"] : 5);

                            if (raceId != targetRaceId || sexId != targetSexId) continue;

                            // Section 0 = skin: match by colorIndex = skin byte
                            const uint32_t csTex1 = csL ? (*csL)["Texture1"] : 6;
                            if (baseSection == 0 && !foundSkin && colorIndex == charSkinId) {
                                std::string tex1 = charSectionsDbc->getString(r, csTex1);
                                if (!tex1.empty()) {
                                    bodySkinPath = tex1;
                                    foundSkin = true;
                                    LOG_INFO("  DBC body skin: ", bodySkinPath, " (skin=", (int)charSkinId, ")");
                                }
                            }
                            // Section 3 = hair: match variation=hairStyle, color=hairColor
                            else if (baseSection == 3 && !foundHair &&
                                     variationIndex == charHairStyleId && colorIndex == charHairColorId) {
                                hairTexturePath = charSectionsDbc->getString(r, csTex1);
                                if (!hairTexturePath.empty()) {
                                    foundHair = true;
                                    LOG_INFO("  DBC hair texture: ", hairTexturePath,
                                             " (style=", (int)charHairStyleId, " color=", (int)charHairColorId, ")");
                                }
                            }
                            // Section 1 = face: match variation=faceId, colorIndex=skinId
                            // Texture1 = face lower, Texture2 = face upper
                            else if (baseSection == 1 && !foundFaceLower &&
                                     variationIndex == charFaceId && colorIndex == charSkinId) {
                                std::string tex1 = charSectionsDbc->getString(r, csTex1);
                                std::string tex2 = charSectionsDbc->getString(r, csTex1 + 1);
                                if (!tex1.empty()) {
                                    faceLowerTexturePath = tex1;
                                    LOG_INFO("  DBC face lower: ", faceLowerTexturePath);
                                }
                                if (!tex2.empty()) {
                                    faceUpperTexturePath = tex2;
                                    LOG_INFO("  DBC face upper: ", faceUpperTexturePath);
                                }
                                foundFaceLower = true;
                            }
                            // Section 4 = underwear
                            else if (baseSection == 4 && !foundUnderwear && colorIndex == charSkinId) {
                                for (uint32_t f = csTex1; f <= csTex1 + 2; f++) {
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
                        auto animFileData = assetManager->readFileOptional(animFileName);
                        if (!animFileData.empty()) {
                            pipeline::M2Loader::loadAnimFile(m2Data, animFileData, si, model);
                        }
                    }
                }

                charRenderer->loadModel(model, 1);

                if (useCharSections) {
                    // Save skin composite state for re-compositing on equipment changes
                    // Include face textures so compositeWithRegions can rebuild the full base
                    bodySkinPath_ = bodySkinPath;
                    underwearPaths_.clear();
                    if (!faceLowerTexturePath.empty()) underwearPaths_.push_back(faceLowerTexturePath);
                    if (!faceUpperTexturePath.empty()) underwearPaths_.push_back(faceUpperTexturePath);
                    for (const auto& up : underwearPaths) underwearPaths_.push_back(up);

                    // Composite body skin + face + underwear overlays
                    {
                        std::vector<std::string> layers;
                        layers.push_back(bodySkinPath);
                        if (!faceLowerTexturePath.empty()) layers.push_back(faceLowerTexturePath);
                        if (!faceUpperTexturePath.empty()) layers.push_back(faceUpperTexturePath);
                        for (const auto& up : underwearPaths) {
                            layers.push_back(up);
                        }
                        if (layers.size() > 1) {
                            GLuint compositeTex = charRenderer->compositeTextures(layers);
                            if (compositeTex != 0) {
                                for (size_t ti = 0; ti < model.textures.size(); ti++) {
                                    if (model.textures[ti].type == 1) {
                                        charRenderer->setModelTexture(1, static_cast<uint32_t>(ti), compositeTex);
                                        skinTextureSlotIndex_ = static_cast<uint32_t>(ti);
                                        LOG_INFO("Replaced type-1 texture slot ", ti, " with composited body+face+underwear");
                                        break;
                                    }
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

	        // Default geosets for the active character (match CharacterPreview logic).
	        // Previous hardcoded values (notably always inserting 101) caused wrong hair meshes in-world.
	        std::unordered_set<uint16_t> activeGeosets;
	        // Body parts (group 0: IDs 0-99, some models use up to 27)
	        for (uint16_t i = 0; i <= 99; i++) activeGeosets.insert(i);

	        uint8_t hairStyleId = 0;
	        uint8_t facialId = 0;
	        if (gameHandler) {
	            if (const game::Character* ch = gameHandler->getActiveCharacter()) {
	                hairStyleId = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
	                facialId = ch->facialFeatures;
	            }
	        }
	        // Hair style geoset: group 1 = 100 + variation + 1
	        activeGeosets.insert(static_cast<uint16_t>(100 + hairStyleId + 1));
	        // Facial hair geoset: group 2 = 200 + variation + 1
	        activeGeosets.insert(static_cast<uint16_t>(200 + facialId + 1));
	        activeGeosets.insert(401);   // Bare forearms (no gloves) — group 4
	        activeGeosets.insert(502);   // Bare shins (no boots) — group 5
	        activeGeosets.insert(702);   // Ears: default
	        activeGeosets.insert(801);   // Bare wrists (no chest armor sleeves) — group 8
	        activeGeosets.insert(902);   // Kneepads: default — group 9
	        activeGeosets.insert(1301);  // Bare legs (no pants) — group 13
	        activeGeosets.insert(1502);  // No cloak — group 15
	        activeGeosets.insert(2002);  // Bare feet — group 20
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

        // Track which character's appearance this instance represents so we can
        // respawn if the user logs into a different character without restarting.
        spawnedPlayerGuid_ = gameHandler ? gameHandler->getActiveCharacterGuid() : 0;
        spawnedAppearanceBytes_ = 0;
        spawnedFacialFeatures_ = 0;
        if (gameHandler) {
            if (const game::Character* ch = gameHandler->getActiveCharacter()) {
                spawnedAppearanceBytes_ = ch->appearanceBytes;
                spawnedFacialFeatures_ = ch->facialFeatures;
            }
        }

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

    if (weaponsSheathed_) {
        for (const auto& ws : weaponSlots) {
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
        }
        return;
    }

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

        const auto* idiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
        std::string modelName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModel"] : 1);
        std::string textureName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModelTexture"] : 3);

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
        if (!skinData.empty() && weaponModel.version >= 264) {
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
    const auto* facL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Faction") : nullptr;
    const auto* ftL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("FactionTemplate") : nullptr;
    std::unordered_set<uint32_t> hostileParentFactions;
    if (fDbc && fDbc->isLoaded()) {
        const uint32_t facID = facL ? (*facL)["ID"] : 0;
        const uint32_t facRaceMask0 = facL ? (*facL)["ReputationRaceMask0"] : 2;
        const uint32_t facBase0 = facL ? (*facL)["ReputationBase0"] : 10;
        for (uint32_t i = 0; i < fDbc->getRecordCount(); i++) {
            uint32_t factionId = fDbc->getUInt32(i, facID);
            for (int slot = 0; slot < 4; slot++) {
                uint32_t raceMask = fDbc->getUInt32(i, facRaceMask0 + slot);
                if (raceMask & playerRaceMask) {
                    int32_t baseRep = fDbc->getInt32(i, facBase0 + slot);
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
    const uint32_t ftID = ftL ? (*ftL)["ID"] : 0;
    const uint32_t ftFaction = ftL ? (*ftL)["Faction"] : 1;
    const uint32_t ftFG = ftL ? (*ftL)["FactionGroup"] : 3;
    const uint32_t ftFriend = ftL ? (*ftL)["FriendGroup"] : 4;
    const uint32_t ftEnemy = ftL ? (*ftL)["EnemyGroup"] : 5;
    const uint32_t ftEnemy0 = ftL ? (*ftL)["Enemy0"] : 6;
    uint32_t playerFriendGroup = 0;
    uint32_t playerEnemyGroup = 0;
    uint32_t playerFactionId = 0;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        if (ftDbc->getUInt32(i, ftID) == playerFtId) {
            playerFriendGroup = ftDbc->getUInt32(i, ftFriend) | ftDbc->getUInt32(i, ftFG);
            playerEnemyGroup = ftDbc->getUInt32(i, ftEnemy);
            playerFactionId = ftDbc->getUInt32(i, ftFaction);
            break;
        }
    }

    // Build hostility map for each faction template
    std::unordered_map<uint32_t, bool> factionMap;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        uint32_t id = ftDbc->getUInt32(i, ftID);
        uint32_t parentFaction = ftDbc->getUInt32(i, ftFaction);
        uint32_t factionGroup = ftDbc->getUInt32(i, ftFG);
        uint32_t friendGroup = ftDbc->getUInt32(i, ftFriend);
        uint32_t enemyGroup = ftDbc->getUInt32(i, ftEnemy);

        // 1. Symmetric group check
        bool hostile = (enemyGroup & playerFriendGroup) != 0
                    || (factionGroup & playerEnemyGroup) != 0;

        // 2. Monster factionGroup bit (8)
        if (!hostile && (factionGroup & 8) != 0) {
            hostile = true;
        }

        // 3. Individual enemy faction IDs
        if (!hostile && playerFactionId > 0) {
            for (uint32_t e = ftEnemy0; e <= ftEnemy0 + 3; e++) {
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

    // Resolve map folder name from Map.dbc (authoritative for world/instance maps).
    // This is required for instances like DeeprunTram (map 369) that are not Azeroth/Kalimdor.
    if (!mapNameCacheLoaded_ && assetManager) {
        mapNameCacheLoaded_ = true;
        if (auto mapDbc = assetManager->loadDBC("Map.dbc"); mapDbc && mapDbc->isLoaded()) {
            mapNameById_.reserve(mapDbc->getRecordCount());
            const auto* mapL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Map") : nullptr;
            for (uint32_t i = 0; i < mapDbc->getRecordCount(); i++) {
                uint32_t id = mapDbc->getUInt32(i, mapL ? (*mapL)["ID"] : 0);
                std::string internalName = mapDbc->getString(i, mapL ? (*mapL)["InternalName"] : 1);
                if (!internalName.empty() && mapNameById_.find(id) == mapNameById_.end()) {
                    mapNameById_[id] = std::move(internalName);
                }
            }
            LOG_INFO("Loaded Map.dbc map-name cache: ", mapNameById_.size(), " entries");
        } else {
            LOG_WARNING("Map.dbc not available; using fallback map-id mapping");
        }
    }

    std::string mapName;
    if (auto it = mapNameById_.find(mapId); it != mapNameById_.end()) {
        mapName = it->second;
    } else {
        mapName = mapIdToName(mapId);
    }
    if (mapName.empty()) {
        LOG_WARNING("Unknown mapId ", mapId, " (no Map.dbc entry); falling back to Azeroth");
        mapName = "Azeroth";
    }
    LOG_INFO("Loading online world terrain for map '", mapName, "' (ID ", mapId, ")");

    // Convert server coordinates to canonical WoW coordinates
    // Server sends: X=West (canonical.Y), Y=North (canonical.X), Z=Up
    glm::vec3 spawnCanonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
    glm::vec3 spawnRender = core::coords::canonicalToRender(spawnCanonical);

    // Set camera position
    if (renderer->getCameraController()) {
        renderer->getCameraController()->setOnlineMode(true);
        renderer->getCameraController()->setDefaultSpawn(spawnRender, 0.0f, -15.0f);
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

    // Connect TransportManager to WMORenderer (for server transports)
    if (gameHandler && gameHandler->getTransportManager() && renderer->getWMORenderer()) {
        gameHandler->getTransportManager()->setWMORenderer(renderer->getWMORenderer());
        LOG_INFO("TransportManager connected to WMORenderer for online mode");
    }

    // Connect WMORenderer to M2Renderer (for hierarchical transforms: doodads following WMO parents)
    if (renderer->getWMORenderer() && renderer->getM2Renderer()) {
        renderer->getWMORenderer()->setM2Renderer(renderer->getM2Renderer());
        LOG_INFO("WMORenderer connected to M2Renderer for hierarchical doodad transforms");
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
            const uint64_t activeGuid = gameHandler->getActiveCharacterGuid();
            const bool appearanceChanged =
                (activeGuid != spawnedPlayerGuid_) ||
                (activeChar->appearanceBytes != spawnedAppearanceBytes_) ||
                (activeChar->facialFeatures != spawnedFacialFeatures_) ||
                (activeChar->race != playerRace_) ||
                (activeChar->gender != playerGender_) ||
                (activeChar->characterClass != playerClass_);

            if (!playerCharacterSpawned || appearanceChanged) {
                if (appearanceChanged) {
                    LOG_INFO("Respawning player model for new/changed character: guid=0x",
                             std::hex, activeGuid, std::dec);
                }
                // Remove old instance so we don't keep stale visuals.
                if (renderer && renderer->getCharacterRenderer()) {
                    uint32_t oldInst = renderer->getCharacterInstanceId();
                    if (oldInst > 0) {
                        renderer->setCharacterFollow(0);
                        renderer->clearMount();
                        renderer->getCharacterRenderer()->removeInstance(oldInst);
                    }
                }
                playerCharacterSpawned = false;
                spawnedPlayerGuid_ = 0;
                spawnedAppearanceBytes_ = 0;
                spawnedFacialFeatures_ = 0;

                playerRace_ = activeChar->race;
                playerGender_ = activeChar->gender;
                playerClass_ = activeChar->characterClass;
                spawnSnapToGround = false;
                weaponsSheathed_ = false;
                loadEquippedWeapons(); // will no-op until instance exists
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

    // Set up test transport (development feature)
    showProgress("Finalizing world...", 0.94f);
    setupTestTransport();

    // Set up NPC animation callbacks (for online creatures)
    showProgress("Preparing creatures...", 0.97f);
    if (gameHandler && renderer && renderer->getCharacterRenderer()) {
        auto* cr = renderer->getCharacterRenderer();
        auto* app = this;

        gameHandler->setNpcDeathCallback([cr, app](uint64_t guid) {
            app->deadCreatureGuids_.insert(guid);
            auto it = app->creatureInstances_.find(guid);
            if (it != app->creatureInstances_.end() && cr) {
                cr->playAnimation(it->second, 1, false); // animation ID 1 = Death
            }
        });

        gameHandler->setNpcRespawnCallback([cr, app](uint64_t guid) {
            app->deadCreatureGuids_.erase(guid);
            auto it = app->creatureInstances_.find(guid);
            if (it != app->creatureInstances_.end() && cr) {
                cr->playAnimation(it->second, 0, true); // animation ID 0 = Idle
            }
        });

        gameHandler->setNpcSwingCallback([cr, app](uint64_t guid) {
            auto it = app->creatureInstances_.find(guid);
            if (it != app->creatureInstances_.end() && cr) {
                cr->playAnimation(it->second, 16, false); // animation ID 16 = Attack1
            }
        });
    }

    // Hide first-login hitch by draining initial world packets/spawn queues before
    // dropping the loading screen. Keep this bounded so we don't stall indefinitely.
    {
        const float kWarmupMaxSeconds = 2.5f;
        const auto warmupStart = std::chrono::high_resolution_clock::now();
        while (true) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    window->setShouldClose(true);
                    if (loadingScreenOk) loadingScreen.shutdown();
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

            // Drain network and process deferred spawn/composite queues while hidden.
            if (gameHandler) gameHandler->update(1.0f / 60.0f);
            if (world) world->update(1.0f / 60.0f);
            processPlayerSpawnQueue();
            processCreatureSpawnQueue();
            processDeferredEquipmentQueue();
            processGameObjectSpawnQueue();
            processPendingTransportDoodads();
            processPendingMount();
            updateQuestMarkers();

            const auto now = std::chrono::high_resolution_clock::now();
            const float elapsed = std::chrono::duration<float>(now - warmupStart).count();
            const float t = std::clamp(elapsed / kWarmupMaxSeconds, 0.0f, 1.0f);
            showProgress("Finalizing world sync...", 0.97f + t * 0.025f);

            if (elapsed >= kWarmupMaxSeconds) {
                break;
            }
            SDL_Delay(16);
        }
    }

    // Start intro pan right before entering gameplay so it's visible after loading.
    if (renderer->getCameraController()) {
        renderer->getCameraController()->startIntroPan(2.8f, 140.0f);
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
        const auto* cdiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CreatureDisplayInfo") : nullptr;
        for (uint32_t i = 0; i < cdi->getRecordCount(); i++) {
            CreatureDisplayData data;
            data.modelId = cdi->getUInt32(i, cdiL ? (*cdiL)["ModelID"] : 1);
            data.extraDisplayId = cdi->getUInt32(i, cdiL ? (*cdiL)["ExtraDisplayId"] : 3);
            data.skin1 = cdi->getString(i, cdiL ? (*cdiL)["Skin1"] : 6);
            data.skin2 = cdi->getString(i, cdiL ? (*cdiL)["Skin2"] : 7);
            data.skin3 = cdi->getString(i, cdiL ? (*cdiL)["Skin3"] : 8);
            displayDataMap_[cdi->getUInt32(i, cdiL ? (*cdiL)["ID"] : 0)] = data;
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
    // Turtle/Vanilla: 19 fields — 10 equip slots (8-17), BakeName=18 (no Flags field)
    // WotLK/TBC/Classic: 21 fields — 11 equip slots (8-18), Flags=19, BakeName=20
    if (auto cdie = assetManager->loadDBC("CreatureDisplayInfoExtra.dbc"); cdie && cdie->isLoaded()) {
        const auto* cdieL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CreatureDisplayInfoExtra") : nullptr;
        const uint32_t cdieEquip0 = cdieL ? (*cdieL)["EquipDisplay0"] : 8;
        const uint32_t bakeField = cdieL ? (*cdieL)["BakeName"] : 20;
        // Count equipment slots: Vanilla/Turtle has 10, WotLK/TBC has 11
        int numEquipSlots = 10;
        if (cdieL && cdieL->field("EquipDisplay10") != 0xFFFFFFFF) {
            numEquipSlots = 11;
        } else if (!cdieL) {
            numEquipSlots = 11;  // Default (WotLK) has 11
        }
        uint32_t withBakeName = 0;
        for (uint32_t i = 0; i < cdie->getRecordCount(); i++) {
            HumanoidDisplayExtra extra;
            extra.raceId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["RaceID"] : 1));
            extra.sexId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["SexID"] : 2));
            extra.skinId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["SkinID"] : 3));
            extra.faceId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["FaceID"] : 4));
            extra.hairStyleId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["HairStyleID"] : 5));
            extra.hairColorId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["HairColorID"] : 6));
            extra.facialHairId = static_cast<uint8_t>(cdie->getUInt32(i, cdieL ? (*cdieL)["FacialHairID"] : 7));
            for (int eq = 0; eq < numEquipSlots; eq++) {
                extra.equipDisplayId[eq] = cdie->getUInt32(i, cdieEquip0 + eq);
            }
            extra.bakeName = cdie->getString(i, bakeField);
            if (!extra.bakeName.empty()) withBakeName++;
            humanoidExtraMap_[cdie->getUInt32(i, cdieL ? (*cdieL)["ID"] : 0)] = extra;
        }
        LOG_INFO("Loaded ", humanoidExtraMap_.size(), " humanoid display extra entries (",
                 withBakeName, " with baked textures, ", numEquipSlots, " equip slots)");
    }

    // CreatureModelData.dbc: modelId (col 0) → modelPath (col 2, .mdx → .m2)
    if (auto cmd = assetManager->loadDBC("CreatureModelData.dbc"); cmd && cmd->isLoaded()) {
        const auto* cmdL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CreatureModelData") : nullptr;
        for (uint32_t i = 0; i < cmd->getRecordCount(); i++) {
            std::string mdx = cmd->getString(i, cmdL ? (*cmdL)["ModelPath"] : 2);
            if (mdx.empty()) continue;
            if (mdx.size() >= 4) {
                mdx = mdx.substr(0, mdx.size() - 4) + ".m2";
            }
            modelIdToPath_[cmd->getUInt32(i, cmdL ? (*cmdL)["ID"] : 0)] = mdx;
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
        // Collect ALL model IDs that map to this path (multiple model IDs can
        // share the same .m2 file, e.g. modelId 147 and 792 both → Gryphon.m2)
        std::vector<uint32_t> modelIds;
        for (const auto& [mid, path] : modelIdToPath_) {
            if (normalizePath(path) == target) {
                modelIds.push_back(mid);
            }
        }
        if (modelIds.empty()) return 0;
        uint32_t bestDisplayId = 0;
        int bestScore = -1;
        for (const auto& [dispId, data] : displayDataMap_) {
            bool matches = false;
            for (uint32_t mid : modelIds) {
                if (data.modelId == mid) { matches = true; break; }
            }
            if (!matches) continue;
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
        const auto* chgL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharHairGeosets") : nullptr;
        for (uint32_t i = 0; i < chg->getRecordCount(); i++) {
            uint32_t raceId = chg->getUInt32(i, chgL ? (*chgL)["RaceID"] : 1);
            uint32_t sexId = chg->getUInt32(i, chgL ? (*chgL)["SexID"] : 2);
            uint32_t variation = chg->getUInt32(i, chgL ? (*chgL)["Variation"] : 3);
            uint32_t geosetId = chg->getUInt32(i, chgL ? (*chgL)["GeosetID"] : 4);
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
        const auto* cfhL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharacterFacialHairStyles") : nullptr;
        for (uint32_t i = 0; i < cfh->getRecordCount(); i++) {
            uint32_t raceId = cfh->getUInt32(i, cfhL ? (*cfhL)["RaceID"] : 0);
            uint32_t sexId = cfh->getUInt32(i, cfhL ? (*cfhL)["SexID"] : 1);
            uint32_t variation = cfh->getUInt32(i, cfhL ? (*cfhL)["Variation"] : 2);
            uint32_t key = (raceId << 16) | (sexId << 8) | variation;
            FacialHairGeosets fhg;
            fhg.geoset100 = static_cast<uint16_t>(cfh->getUInt32(i, cfhL ? (*cfhL)["Geoset100"] : 3));
            fhg.geoset300 = static_cast<uint16_t>(cfh->getUInt32(i, cfhL ? (*cfhL)["Geoset300"] : 4));
            fhg.geoset200 = static_cast<uint16_t>(cfh->getUInt32(i, cfhL ? (*cfhL)["Geoset200"] : 5));
            facialHairGeosetMap_[key] = fhg;
        }
        LOG_INFO("Loaded ", facialHairGeosetMap_.size(), " facial hair geoset mappings from CharacterFacialHairStyles.dbc");
    }

    creatureLookupsBuilt_ = true;
}

std::string Application::getModelPathForDisplayId(uint32_t displayId) const {
    if (displayId == 30412) return "Creature\\Gryphon\\Gryphon.m2";
    if (displayId == 30413) return "Creature\\Wyvern\\Wyvern.m2";

    // WotLK servers can send display IDs that do not exist in older/local
    // CreatureDisplayInfo datasets. Keep those creatures visible by falling
    // back to a close base model instead of dropping spawn entirely.
    switch (displayId) {
        case 31048: // Diseased Young Wolf variants (AzerothCore WotLK)
        case 31049: // Diseased Wolf variants (AzerothCore WotLK)
            return "Creature\\Wolf\\Wolf.m2";
        default:
            break;
    }

    auto itData = displayDataMap_.find(displayId);
    if (itData == displayDataMap_.end()) {
        // Some sources (e.g., taxi nodes) may provide a modelId directly.
        auto itPath = modelIdToPath_.find(displayId);
        if (itPath != modelIdToPath_.end()) {
            return itPath->second;
        }
        if (displayId == 30412) return "Creature\\Gryphon\\Gryphon.m2";
        if (displayId == 30413) return "Creature\\Wyvern\\Wyvern.m2";
        LOG_WARNING("No display data for displayId ", displayId,
                    " (displayDataMap_ has ", displayDataMap_.size(), " entries)");
        return "";
    }

    auto itPath = modelIdToPath_.find(itData->second.modelId);
    if (itPath == modelIdToPath_.end()) {
        LOG_WARNING("No model path for modelId ", itData->second.modelId,
                    " from displayId ", displayId,
                    " (modelIdToPath_ has ", modelIdToPath_.size(), " entries)");
        return "";
    }

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
        const auto* godiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("GameObjectDisplayInfo") : nullptr;
        for (uint32_t i = 0; i < godi->getRecordCount(); i++) {
            uint32_t displayId = godi->getUInt32(i, godiL ? (*godiL)["ID"] : 0);
            std::string modelName = godi->getString(i, godiL ? (*godiL)["ModelName"] : 1);
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
        auto pit = playerInstances_.find(guid);
        if (pit != playerInstances_.end()) instanceId = pit->second;
    }
    if (instanceId == 0) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end()) instanceId = it->second;
    }
    if (instanceId == 0) return false;

    return renderer->getCharacterRenderer()->getInstanceBounds(instanceId, outCenter, outRadius);
}

bool Application::getRenderFootZForGuid(uint64_t guid, float& outFootZ) const {
    if (!renderer || !renderer->getCharacterRenderer()) return false;
    uint32_t instanceId = 0;

    if (gameHandler && guid == gameHandler->getPlayerGuid()) {
        instanceId = renderer->getCharacterInstanceId();
    }
    if (instanceId == 0) {
        auto pit = playerInstances_.find(guid);
        if (pit != playerInstances_.end()) instanceId = pit->second;
    }
    if (instanceId == 0) {
        auto it = creatureInstances_.find(guid);
        if (it != creatureInstances_.end()) instanceId = it->second;
    }
    if (instanceId == 0) return false;

    return renderer->getCharacterRenderer()->getInstanceFootZ(instanceId, outFootZ);
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
    {
        // Intentionally invisible helper creatures should not consume retry budget.
        std::string lowerPath = m2Path;
        std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lowerPath.find("invisiblestalker") != std::string::npos ||
            lowerPath.find("invisible_stalker") != std::string::npos) {
            creaturePermanentFailureGuids_.insert(guid);
            return;
        }
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

        // Load skin file (only for WotLK M2s - vanilla has embedded skin)
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty() && model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        // Load external .anim files for sequences without flag 0x20
        std::string basePath = m2Path.substr(0, m2Path.size() - 3);
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName), "%s%04u-%02u.anim",
                    basePath.c_str(), model.sequences[si].id, model.sequences[si].variationIndex);
                auto animData = assetManager->readFileOptional(animFileName);
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

                // Build equipment texture region layers from NPC equipment display IDs
                // (texture-only compositing — no geoset changes to avoid invisibility bugs)
                std::vector<std::pair<int, std::string>> npcRegionLayers;
                std::string npcCapeTexturePath;
                auto npcItemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
                    if (npcItemDisplayDbc) {
                        static const char* npcComponentDirs[] = {
                            "ArmUpperTexture", "ArmLowerTexture", "HandTexture",
                            "TorsoUpperTexture", "TorsoLowerTexture",
                            "LegUpperTexture", "LegLowerTexture", "FootTexture",
                        };
                        const auto* idiL = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                        // Texture component region fields (8 regions: ArmUpper..Foot)
                        // Binary DBC (23 fields) has textures at 14+
                        const uint32_t texRegionFields[8] = {
                            idiL ? (*idiL)["TextureArmUpper"]  : 14u,
                            idiL ? (*idiL)["TextureArmLower"]  : 15u,
                            idiL ? (*idiL)["TextureHand"]      : 16u,
                            idiL ? (*idiL)["TextureTorsoUpper"]: 17u,
                            idiL ? (*idiL)["TextureTorsoLower"]: 18u,
                            idiL ? (*idiL)["TextureLegUpper"]  : 19u,
                            idiL ? (*idiL)["TextureLegLower"]  : 20u,
                            idiL ? (*idiL)["TextureFoot"]      : 21u,
                        };
                        const bool npcIsFemale = (extra.sexId == 1);

                        auto regionAllowedForNpcSlot = [](int eqSlot, int region) -> bool {
                            // Regions: 0 ArmUpper, 1 ArmLower, 2 Hand, 3 TorsoUpper, 4 TorsoLower,
                            //          5 LegUpper, 6 LegLower, 7 Foot
                            switch (eqSlot) {
                                case 2: // shirt
                                case 3: // chest
                                    return region <= 4;
                                case 4: // belt
                                    // TODO(#npc-belt-region): belt torso-lower overlay can
                                    // cut out male abdomen on some humanoid NPCs.
                                    // Keep disabled until region compositing is fixed.
                                    return false;
                                case 5: // legs
                                    return region == 5 || region == 6;
                                case 6: // feet
                                    return region == 7;
                                case 7: // wrist
                                    return region == 1;
                                case 8: // hands
                                    return region == 0 || region == 1 || region == 2;
                                case 9: // tabard
                                    return region == 3 || region == 4;
                                default:
                                    return false;
                            }
                        };

                        // Iterate all 11 NPC equipment slots; use slot-aware region filtering
                        for (int eqSlot = 0; eqSlot < 11; eqSlot++) {
                            uint32_t did = extra.equipDisplayId[eqSlot];
                            if (did == 0) continue;
                            int32_t recIdx = npcItemDisplayDbc->findRecordById(did);
                            if (recIdx < 0) continue;

                            for (int region = 0; region < 8; region++) {
                                if (!regionAllowedForNpcSlot(eqSlot, region)) continue;
                                std::string texName = npcItemDisplayDbc->getString(
                                    static_cast<uint32_t>(recIdx), texRegionFields[region]);
                                if (texName.empty()) continue;

                                std::string base = "Item\\TextureComponents\\" +
                                    std::string(npcComponentDirs[region]) + "\\" + texName;
                                std::string genderPath = base + (npcIsFemale ? "_F.blp" : "_M.blp");
                                std::string unisexPath = base + "_U.blp";
                                std::string basePath = base + ".blp";
                                std::string fullPath;
                                if (assetManager->fileExists(genderPath)) fullPath = genderPath;
                                else if (assetManager->fileExists(unisexPath)) fullPath = unisexPath;
                                else if (assetManager->fileExists(basePath)) fullPath = basePath;
                                else continue;

                                npcRegionLayers.emplace_back(region, fullPath);
                            }
                        }

                        // Cloak/cape texture is separate from the body atlas.
                        // Read equipped cape displayId (slot 10) and resolve the best cape texture path.
                        uint32_t capeDisplayId = extra.equipDisplayId[10];
                        if (capeDisplayId != 0) {
                            int32_t capeRecIdx = npcItemDisplayDbc->findRecordById(capeDisplayId);
                            if (capeRecIdx >= 0) {
                                const uint32_t leftTexField = idiL ? (*idiL)["LeftModelTexture"] : 3u;
                                const uint32_t rightTexField = leftTexField + 1u; // modelTexture_2 in 3.3.5a

                                std::vector<std::string> capeNames;
                                auto addName = [&](const std::string& n) {
                                    if (!n.empty() && std::find(capeNames.begin(), capeNames.end(), n) == capeNames.end()) {
                                        capeNames.push_back(n);
                                    }
                                };
                                std::string leftName = npcItemDisplayDbc->getString(
                                    static_cast<uint32_t>(capeRecIdx), leftTexField);
                                std::string rightName = npcItemDisplayDbc->getString(
                                    static_cast<uint32_t>(capeRecIdx), rightTexField);
                                // Female models often prefer modelTexture_2.
                                if (npcIsFemale) {
                                    addName(rightName);
                                    addName(leftName);
                                } else {
                                    addName(leftName);
                                    addName(rightName);
                                }

                                auto hasBlpExt = [](const std::string& p) {
                                    if (p.size() < 4) return false;
                                    std::string ext = p.substr(p.size() - 4);
                                    std::transform(ext.begin(), ext.end(), ext.begin(),
                                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                                    return ext == ".blp";
                                };

                                std::vector<std::string> capeCandidates;
                                auto addCapeCandidate = [&](const std::string& p) {
                                    if (p.empty()) return;
                                    if (std::find(capeCandidates.begin(), capeCandidates.end(), p) == capeCandidates.end()) {
                                        capeCandidates.push_back(p);
                                    }
                                };

                                for (const auto& nameRaw : capeNames) {
                                    std::string name = nameRaw;
                                    std::replace(name.begin(), name.end(), '/', '\\');
                                    bool hasDir = (name.find('\\') != std::string::npos);
                                    bool hasExt = hasBlpExt(name);
                                    if (hasDir) {
                                        addCapeCandidate(name);
                                        if (!hasExt) addCapeCandidate(name + ".blp");
                                    } else {
                                        std::string base = "Item\\ObjectComponents\\Cape\\" + name;
                                        addCapeCandidate(base);
                                        if (!hasExt) addCapeCandidate(base + ".blp");
                                        // Some data sets use gender/unisex suffix variants.
                                        addCapeCandidate(base + (npcIsFemale ? "_F.blp" : "_M.blp"));
                                        addCapeCandidate(base + "_U.blp");
                                    }
                                }

                                for (const auto& candidate : capeCandidates) {
                                    if (assetManager->fileExists(candidate)) {
                                        npcCapeTexturePath = candidate;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                // Use baked texture for body skin (types 1, 2)
                // Type 6 (hair) needs its own texture from CharSections.dbc
                const bool allowNpcRegionComposite = true;
                if (!extra.bakeName.empty()) {
                    std::string bakePath = "Textures\\BakedNpcTextures\\" + extra.bakeName;

                    // Composite equipment textures over baked NPC texture, or just load baked texture
                    GLuint finalTex = 0;
                    if (allowNpcRegionComposite && !npcRegionLayers.empty()) {
                        finalTex = charRenderer->compositeWithRegions(bakePath, {}, npcRegionLayers);
                        LOG_DEBUG("Composited NPC baked texture with ", npcRegionLayers.size(),
                                  " equipment regions: ", bakePath);
                    } else {
                        finalTex = charRenderer->loadTexture(bakePath);
                    }

                    if (finalTex != 0 && modelData) {
                        for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                            uint32_t texType = modelData->textures[ti].type;
                            // Humanoid NPCs typically use creature-skin texture types (11-13).
                            // Keep type 2 (object skin) untouched so cloak/cape slots do not get face/body textures.
                            if (texType == 1 || texType == 11 || texType == 12 || texType == 13) {
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

                    // Build skin texture from CharSections.dbc (same as player character)
                    auto csFallbackDbc = assetManager->loadDBC("CharSections.dbc");
                    if (csFallbackDbc) {
                        const auto* csFL = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
                        uint32_t npcRace = static_cast<uint32_t>(extra.raceId);
                        uint32_t npcSex = static_cast<uint32_t>(extra.sexId);
                        uint32_t npcSkin = static_cast<uint32_t>(extra.skinId);
                        uint32_t npcFace = static_cast<uint32_t>(extra.faceId);
                        std::string npcSkinPath, npcFaceLower, npcFaceUpper;
                        std::vector<std::string> npcUnderwear;

                        for (uint32_t r = 0; r < csFallbackDbc->getRecordCount(); r++) {
                            uint32_t rId = csFallbackDbc->getUInt32(r, csFL ? (*csFL)["RaceID"] : 1);
                            uint32_t sId = csFallbackDbc->getUInt32(r, csFL ? (*csFL)["SexID"] : 2);
                            if (rId != npcRace || sId != npcSex) continue;

                            uint32_t section = csFallbackDbc->getUInt32(r, csFL ? (*csFL)["BaseSection"] : 3);
                            uint32_t variation = csFallbackDbc->getUInt32(r, csFL ? (*csFL)["VariationIndex"] : 8);
                            uint32_t color = csFallbackDbc->getUInt32(r, csFL ? (*csFL)["ColorIndex"] : 9);
                            uint32_t tex1F = csFL ? (*csFL)["Texture1"] : 4;

                            // Section 0 = skin: match colorIndex = skinId
                            if (section == 0 && npcSkinPath.empty() && color == npcSkin) {
                                npcSkinPath = csFallbackDbc->getString(r, tex1F);
                            }
                            // Section 1 = face: match variation=faceId, color=skinId
                            else if (section == 1 && npcFaceLower.empty() &&
                                     variation == npcFace && color == npcSkin) {
                                npcFaceLower = csFallbackDbc->getString(r, tex1F);
                                npcFaceUpper = csFallbackDbc->getString(r, tex1F + 1);
                            }
                            // Section 4 = underwear: match color=skinId
                            else if (section == 4 && npcUnderwear.empty() && color == npcSkin) {
                                for (uint32_t f = tex1F; f <= tex1F + 2; f++) {
                                    std::string tex = csFallbackDbc->getString(r, f);
                                    if (!tex.empty()) npcUnderwear.push_back(tex);
                                }
                            }
                        }

                        if (!npcSkinPath.empty()) {
                            // Composite skin + face + underwear
                            std::vector<std::string> skinLayers;
                            skinLayers.push_back(npcSkinPath);
                            if (!npcFaceLower.empty()) skinLayers.push_back(npcFaceLower);
                            if (!npcFaceUpper.empty()) skinLayers.push_back(npcFaceUpper);
                            for (const auto& uw : npcUnderwear) skinLayers.push_back(uw);

                            GLuint npcSkinTex = 0;
                            if (allowNpcRegionComposite && !npcRegionLayers.empty()) {
                                npcSkinTex = charRenderer->compositeWithRegions(npcSkinPath,
                                    std::vector<std::string>(skinLayers.begin() + 1, skinLayers.end()),
                                    npcRegionLayers);
                            } else if (skinLayers.size() > 1) {
                                npcSkinTex = charRenderer->compositeTextures(skinLayers);
                            } else {
                                npcSkinTex = charRenderer->loadTexture(npcSkinPath);
                            }

                            if (npcSkinTex != 0 && modelData) {
                                for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                                    uint32_t texType = modelData->textures[ti].type;
                                    if (texType == 1 || texType == 11 || texType == 12 || texType == 13) {
                                        charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), npcSkinTex);
                                        hasHumanoidTexture = true;
                                    }
                                }
                                LOG_DEBUG("Applied CharSections skin to NPC: ", npcSkinPath);
                            }
                        }
                    }
                }

                // Load hair texture from CharSections.dbc (section 3)
                auto charSectionsDbc = assetManager->loadDBC("CharSections.dbc");
                if (charSectionsDbc) {
                    const auto* csL2 = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
                    uint32_t targetRace = static_cast<uint32_t>(extra.raceId);
                    uint32_t targetSex = static_cast<uint32_t>(extra.sexId);
                    std::string hairTexPath;

                    for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
                        uint32_t raceId = charSectionsDbc->getUInt32(r, csL2 ? (*csL2)["RaceID"] : 1);
                        uint32_t sexId = charSectionsDbc->getUInt32(r, csL2 ? (*csL2)["SexID"] : 2);
                        uint32_t section = charSectionsDbc->getUInt32(r, csL2 ? (*csL2)["BaseSection"] : 3);
                        uint32_t variation = charSectionsDbc->getUInt32(r, csL2 ? (*csL2)["VariationIndex"] : 4);
                        uint32_t colorIdx = charSectionsDbc->getUInt32(r, csL2 ? (*csL2)["ColorIndex"] : 5);

                        if (raceId != targetRace || sexId != targetSex) continue;
                        if (section != 3) continue;  // Section 3 = hair
                        if (variation != static_cast<uint32_t>(extra.hairStyleId)) continue;
                        if (colorIdx != static_cast<uint32_t>(extra.hairColorId)) continue;

                        hairTexPath = charSectionsDbc->getString(r, csL2 ? (*csL2)["Texture1"] : 6);
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

                // Do not apply cape textures at model scope here. Type-2 texture slots are
                // shared per model and this can leak cape textures/white fallbacks onto
                // unrelated humanoid NPCs that use the same modelId.
            } else {
                LOG_WARNING("  extraDisplayId ", dispData.extraDisplayId, " not found in humanoidExtraMap");
            }
        }

        // Apply creature skin textures (for non-humanoid creatures)
        if (!hasHumanoidTexture && modelData) {
            auto resolveCreatureSkinPath = [&](const std::string& skinField) -> std::string {
                if (skinField.empty()) return "";

                std::string raw = skinField;
                std::replace(raw.begin(), raw.end(), '/', '\\');
                auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
                raw.erase(raw.begin(), std::find_if(raw.begin(), raw.end(), [&](unsigned char c) { return !isSpace(c); }));
                raw.erase(std::find_if(raw.rbegin(), raw.rend(), [&](unsigned char c) { return !isSpace(c); }).base(), raw.end());
                if (raw.empty()) return "";

                auto hasBlpExt = [](const std::string& p) {
                    if (p.size() < 4) return false;
                    std::string ext = p.substr(p.size() - 4);
                    std::transform(ext.begin(), ext.end(), ext.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    return ext == ".blp";
                };
                auto addCandidate = [](std::vector<std::string>& out, const std::string& p) {
                    if (p.empty()) return;
                    if (std::find(out.begin(), out.end(), p) == out.end()) out.push_back(p);
                };

                std::vector<std::string> candidates;
                const bool hasDir = (raw.find('\\') != std::string::npos || raw.find('/') != std::string::npos);
                const bool hasExt = hasBlpExt(raw);

                if (hasDir) {
                    addCandidate(candidates, raw);
                    if (!hasExt) addCandidate(candidates, raw + ".blp");
                } else {
                    addCandidate(candidates, modelDir + raw);
                    if (!hasExt) addCandidate(candidates, modelDir + raw + ".blp");
                    addCandidate(candidates, raw);
                    if (!hasExt) addCandidate(candidates, raw + ".blp");
                }

                for (const auto& c : candidates) {
                    if (assetManager->fileExists(c)) return c;
                }
                return "";
            };

            for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                const auto& tex = modelData->textures[ti];
                std::string skinPath;

                // Creature skin types: 11 = skin1, 12 = skin2, 13 = skin3
                if (tex.type == 11 && !dispData.skin1.empty()) {
                    skinPath = resolveCreatureSkinPath(dispData.skin1);
                } else if (tex.type == 12 && !dispData.skin2.empty()) {
                    skinPath = resolveCreatureSkinPath(dispData.skin2);
                } else if (tex.type == 13 && !dispData.skin3.empty()) {
                    skinPath = resolveCreatureSkinPath(dispData.skin3);
                }

                if (!skinPath.empty()) {
                    GLuint skinTex = charRenderer->loadTexture(skinPath);
                    if (skinTex != 0) {
                        charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                        LOG_DEBUG("Applied creature skin texture: ", skinPath, " to slot ", ti);
                    }
                } else if ((tex.type == 11 && !dispData.skin1.empty()) ||
                           (tex.type == 12 && !dispData.skin2.empty()) ||
                           (tex.type == 13 && !dispData.skin3.empty())) {
                    LOG_WARNING("Creature skin texture not found for displayId ", displayId,
                                " slot ", ti, " type ", tex.type,
                                " (skin fields: '", dispData.skin1, "', '",
                                dispData.skin2, "', '", dispData.skin3, "')");
                }
            }
        }
    }

    // Use the entity's latest server-authoritative position rather than the stale spawn
    // position. Movement packets (SMSG_MONSTER_MOVE) can arrive while a creature is still
    // queued in pendingCreatureSpawns_ and get silently dropped. getLatestX/Y/Z returns
    // the movement destination if the entity is mid-move, which is always up-to-date
    // regardless of distance culling (unlike getX/Y/Z which requires updateMovement).
    if (gameHandler) {
        if (auto entity = gameHandler->getEntityManager().getEntity(guid)) {
            x = entity->getLatestX();
            y = entity->getLatestY();
            z = entity->getLatestZ();
            orientation = entity->getOrientation();
        }
    }

    // Convert canonical → render coordinates
    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));

    // Keep authoritative server Z for online creature spawns.
    // Terrain-based lifting can incorrectly move elevated NPCs (e.g. flight masters on
    // Stormwind ramparts) to bad heights relative to WMO geometry.

    // Convert canonical WoW orientation (0=north) -> render yaw (0=west)
    float renderYaw = orientation + glm::radians(90.0f);

    // Create instance
    uint32_t instanceId = charRenderer->createInstance(modelId, renderPos,
        glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);

    if (instanceId == 0) {
        LOG_WARNING("Failed to create creature instance for guid 0x", std::hex, guid, std::dec);
        return;
    }

    // Optional humanoid NPC geoset mask. Disabled by default because forcing geosets
    // causes long-standing visual artifacts on some models (missing waist, phantom
    // bracers, flickering apron overlays). Prefer model defaults.
    static constexpr bool kEnableNpcSafeGeosetMask = false;
    if (kEnableNpcSafeGeosetMask &&
        itDisplayData != displayDataMap_.end() &&
        itDisplayData->second.extraDisplayId != 0) {
        auto itExtra = humanoidExtraMap_.find(itDisplayData->second.extraDisplayId);
        if (itExtra != humanoidExtraMap_.end()) {
            const auto& extra = itExtra->second;
            std::unordered_set<uint16_t> safeGeosets;
            std::unordered_set<uint16_t> modelGeosets;
            std::unordered_map<uint16_t, uint16_t> firstGeosetByGroup;
            if (const auto* md = charRenderer->getModelData(modelId)) {
                for (const auto& b : md->batches) {
                    const uint16_t sid = b.submeshId;
                    modelGeosets.insert(sid);
                    const uint16_t group = static_cast<uint16_t>(sid / 100);
                    auto it = firstGeosetByGroup.find(group);
                    if (it == firstGeosetByGroup.end() || sid < it->second) {
                        firstGeosetByGroup[group] = sid;
                    }
                }
            }
            auto addSafeGeoset = [&](uint16_t preferredId) {
                if (preferredId < 100 || modelGeosets.empty()) {
                    safeGeosets.insert(preferredId);
                    return;
                }
                if (modelGeosets.count(preferredId) > 0) {
                    safeGeosets.insert(preferredId);
                    return;
                }
                const uint16_t group = static_cast<uint16_t>(preferredId / 100);
                auto it = firstGeosetByGroup.find(group);
                if (it != firstGeosetByGroup.end()) {
                    safeGeosets.insert(it->second);
                }
            };
            uint16_t hairGeoset = 1;
            uint32_t hairKey = (static_cast<uint32_t>(extra.raceId) << 16) |
                               (static_cast<uint32_t>(extra.sexId) << 8) |
                               static_cast<uint32_t>(extra.hairStyleId);
            auto itHairGeo = hairGeosetMap_.find(hairKey);
            if (itHairGeo != hairGeosetMap_.end() && itHairGeo->second > 0) {
                hairGeoset = itHairGeo->second;
            }
            const uint16_t selectedHairScalp = (hairGeoset > 0 ? hairGeoset : 1);
            std::unordered_set<uint16_t> hairScalpGeosetsForRaceSex;
            for (const auto& [k, v] : hairGeosetMap_) {
                uint8_t race = static_cast<uint8_t>((k >> 16) & 0xFF);
                uint8_t sex = static_cast<uint8_t>((k >> 8) & 0xFF);
                if (race == extra.raceId && sex == extra.sexId && v > 0 && v < 100) {
                    hairScalpGeosetsForRaceSex.insert(v);
                }
            }
            // Group 0 contains both base body parts and race/sex hair scalp variants.
            // Keep all non-hair body submeshes, but only the selected hair scalp.
            for (uint16_t sid : modelGeosets) {
                if (sid >= 100) continue;
                if (hairScalpGeosetsForRaceSex.count(sid) > 0 && sid != selectedHairScalp) continue;
                safeGeosets.insert(sid);
            }
            safeGeosets.insert(selectedHairScalp);
            addSafeGeoset(static_cast<uint16_t>(100 + std::max<uint16_t>(hairGeoset, 1)));

            uint32_t facialKey = (static_cast<uint32_t>(extra.raceId) << 16) |
                                 (static_cast<uint32_t>(extra.sexId) << 8) |
                                 static_cast<uint32_t>(extra.facialHairId);
            auto itFacial = facialHairGeosetMap_.find(facialKey);
            if (itFacial != facialHairGeosetMap_.end()) {
                const auto& fhg = itFacial->second;
                addSafeGeoset(static_cast<uint16_t>(200 + std::max<uint16_t>(fhg.geoset200, 1)));
                addSafeGeoset(static_cast<uint16_t>(300 + std::max<uint16_t>(fhg.geoset300, 1)));
            } else {
                addSafeGeoset(201);
                addSafeGeoset(301);
            }

            // Force pants (1301) and avoid robe skirt variants unless we re-enable full slot-accurate geosets.
            addSafeGeoset(301);
            addSafeGeoset(401);
            addSafeGeoset(402);
            addSafeGeoset(501);
            addSafeGeoset(701);
            addSafeGeoset(801);
            addSafeGeoset(901);
            addSafeGeoset(1201);
            addSafeGeoset(1301);
            addSafeGeoset(2002);

            charRenderer->setActiveGeosets(instanceId, safeGeosets);
        }
    }

    // NOTE: Custom humanoid NPC geoset/equipment overrides are currently too
    // aggressive and can make NPCs invisible (targetable but not rendered).
    // Keep default model geosets for online creatures until this path is made
    // data-accurate per display model.
    static constexpr bool kEnableNpcHumanoidOverrides = false;

    // Set geosets for humanoid NPCs based on CreatureDisplayInfoExtra
    if (kEnableNpcHumanoidOverrides &&
        itDisplayData != displayDataMap_.end() &&
        itDisplayData->second.extraDisplayId != 0) {
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
            // CharGeosets: group 4=gloves(forearm), 5=boots(shin), 8=sleeves, 9=kneepads, 13=pants
            std::unordered_set<uint16_t> modelGeosets;
            std::unordered_map<uint16_t, uint16_t> firstByGroup;
            if (const auto* md = charRenderer->getModelData(modelId)) {
                for (const auto& b : md->batches) {
                    const uint16_t sid = b.submeshId;
                    modelGeosets.insert(sid);
                    const uint16_t group = static_cast<uint16_t>(sid / 100);
                    auto it = firstByGroup.find(group);
                    if (it == firstByGroup.end() || sid < it->second) {
                        firstByGroup[group] = sid;
                    }
                }
            }
            auto pickGeoset = [&](uint16_t preferred, uint16_t group) -> uint16_t {
                if (preferred != 0 && modelGeosets.count(preferred) > 0) return preferred;
                auto it = firstByGroup.find(group);
                if (it != firstByGroup.end()) return it->second;
                return preferred;
            };

            uint16_t geosetGloves = pickGeoset(301, 3);   // Bare gloves/forearms (group 3)
            uint16_t geosetBoots = pickGeoset(401, 4);    // Bare boots/shins (group 4)
            uint16_t geosetTorso = pickGeoset(501, 5);    // Base torso/waist (group 5)
            uint16_t geosetSleeves = pickGeoset(801, 8);  // Bare wrists (group 8, controlled by chest)
            uint16_t geosetPants = pickGeoset(1301, 13);  // Bare legs (group 13)
            uint16_t geosetCape = 0;       // Group 15 disabled unless cape is equipped
            uint16_t geosetTabard = 0;     // TODO: NPC tabard geosets currently flicker/apron; keep hidden for now
            GLuint npcCapeTextureId = 0;

            // Load equipment geosets from ItemDisplayInfo.dbc
            // DBC columns: 7=GeosetGroup[0], 8=GeosetGroup[1], 9=GeosetGroup[2]
            auto itemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
            const auto* idiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
            if (itemDisplayDbc) {
                // Equipment slots: 0=helm, 1=shoulder, 2=shirt, 3=chest, 4=belt, 5=legs, 6=feet, 7=wrist, 8=hands, 9=tabard, 10=cape
                const uint32_t fGG1 = idiL ? (*idiL)["GeosetGroup1"] : 7;

                // Chest (slot 3) → group 5 (torso) + group 8 (sleeves/wristbands)
                if (extra.equipDisplayId[3] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[3]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), fGG1);
                        if (gg > 0) geosetTorso = pickGeoset(static_cast<uint16_t>(501 + gg), 5);
                        if (gg > 0) geosetSleeves = pickGeoset(static_cast<uint16_t>(801 + gg), 8);
                        // Do not derive robe/kilt from chest by default here.
                        // Some NPC datasets set chest geosets that cause persistent
                        // apron/robe overlays; prefer explicit legs slot for trousers.
                    }
                }

                // Legs (slot 5) → group 13 (trousers)
                if (extra.equipDisplayId[5] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[5]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), fGG1);
                        if (gg > 0) geosetPants = pickGeoset(static_cast<uint16_t>(1301 + gg), 13);
                    }
                }

                // Feet (slot 6) → group 4 (boots/shins)
                if (extra.equipDisplayId[6] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[6]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), fGG1);
                        if (gg > 0) geosetBoots = pickGeoset(static_cast<uint16_t>(401 + gg), 4);
                    }
                }

                // Hands (slot 8) → group 3 (gloves/forearms)
                if (extra.equipDisplayId[8] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[8]);
                    if (idx >= 0) {
                        uint32_t gg = itemDisplayDbc->getUInt32(static_cast<uint32_t>(idx), fGG1);
                        if (gg > 0) geosetGloves = pickGeoset(static_cast<uint16_t>(301 + gg), 3);
                    }
                }

                // Tabard (slot 9) intentionally disabled for now (see geosetTabard TODO above).

                // Cape (slot 10) → group 15
                if (extra.equipDisplayId[10] != 0) {
                    int32_t idx = itemDisplayDbc->findRecordById(extra.equipDisplayId[10]);
                    if (idx >= 0) {
                        geosetCape = 1502;
                        const bool npcIsFemale = (extra.sexId == 1);
                        const uint32_t leftTexField = idiL ? (*idiL)["LeftModelTexture"] : 3u;
                        std::vector<std::string> capeNames;
                        auto addName = [&](const std::string& n) {
                            if (!n.empty() && std::find(capeNames.begin(), capeNames.end(), n) == capeNames.end()) {
                                capeNames.push_back(n);
                            }
                        };
                        std::string leftName = itemDisplayDbc->getString(static_cast<uint32_t>(idx), leftTexField);
                        addName(leftName);

                        auto hasBlpExt = [](const std::string& p) {
                            if (p.size() < 4) return false;
                            std::string ext = p.substr(p.size() - 4);
                            std::transform(ext.begin(), ext.end(), ext.begin(),
                                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                            return ext == ".blp";
                        };
                        std::vector<std::string> capeCandidates;
                        auto addCapeCandidate = [&](const std::string& p) {
                            if (p.empty()) return;
                            if (std::find(capeCandidates.begin(), capeCandidates.end(), p) == capeCandidates.end()) {
                                capeCandidates.push_back(p);
                            }
                        };
                        for (const auto& nameRaw : capeNames) {
                            std::string name = nameRaw;
                            std::replace(name.begin(), name.end(), '/', '\\');
                            const bool hasDir = (name.find('\\') != std::string::npos);
                            const bool hasExt = hasBlpExt(name);
                            if (hasDir) {
                                addCapeCandidate(name);
                                if (!hasExt) addCapeCandidate(name + ".blp");
                            } else {
                                std::string baseObj = "Item\\ObjectComponents\\Cape\\" + name;
                                std::string baseTex = "Item\\TextureComponents\\Cape\\" + name;
                                addCapeCandidate(baseObj);
                                addCapeCandidate(baseTex);
                                if (!hasExt) {
                                    addCapeCandidate(baseObj + ".blp");
                                    addCapeCandidate(baseTex + ".blp");
                                }
                                addCapeCandidate(baseObj + (npcIsFemale ? "_F.blp" : "_M.blp"));
                                addCapeCandidate(baseObj + "_U.blp");
                                addCapeCandidate(baseTex + (npcIsFemale ? "_F.blp" : "_M.blp"));
                                addCapeCandidate(baseTex + "_U.blp");
                            }
                        }
                        const GLuint whiteTex = charRenderer->loadTexture("");
                        for (const auto& candidate : capeCandidates) {
                            GLuint tex = charRenderer->loadTexture(candidate);
                            if (tex != 0 && tex != whiteTex) {
                                npcCapeTextureId = tex;
                                break;
                            }
                        }
                    }
                }
            }

            // Apply equipment geosets
            activeGeosets.insert(geosetGloves);
            activeGeosets.insert(geosetBoots);
            activeGeosets.insert(geosetTorso);
            activeGeosets.insert(geosetSleeves);
            activeGeosets.insert(geosetPants);
            if (geosetCape != 0) {
                activeGeosets.insert(geosetCape);
            }
            if (geosetTabard != 0) {
                activeGeosets.insert(geosetTabard);
            }
            activeGeosets.insert(pickGeoset(702, 7));   // Ears: default
            activeGeosets.insert(pickGeoset(902, 9));   // Kneepads: default
            activeGeosets.insert(pickGeoset(2002, 20)); // Bare feet mesh
            // Keep all model-present torso variants active to avoid missing male
            // abdomen/waist sections when a single 5xx pick is wrong.
            for (uint16_t sid : modelGeosets) {
                if ((sid / 100) == 5) activeGeosets.insert(sid);
            }
            // Keep all model-present pelvis variants active to avoid missing waist/belt
            // sections on some humanoid males when a single 9xx variant is wrong.
            for (uint16_t sid : modelGeosets) {
                if ((sid / 100) == 9) activeGeosets.insert(sid);
            }

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
            if (geosetCape != 0 && npcCapeTextureId != 0) {
                charRenderer->setGroupTextureOverride(instanceId, 15, npcCapeTextureId);
                if (const auto* md = charRenderer->getModelData(modelId)) {
                    for (size_t ti = 0; ti < md->textures.size(); ti++) {
                        if (md->textures[ti].type == 2) {
                            charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(ti), npcCapeTextureId);
                        }
                    }
                }
            }
            LOG_DEBUG("Set humanoid geosets: hair=", (int)hairGeoset,
                      " sleeves=", geosetSleeves, " pants=", geosetPants,
                      " boots=", geosetBoots, " gloves=", geosetGloves);

            // TODO(#helmet-attach): NPC helmet attachment anchors are currently unreliable
            // on some humanoid models (floating/incorrect bone bind). Keep hidden for now.
            static constexpr bool kEnableNpcHelmetAttachmentsMainPath = false;
            // Load and attach helmet model if equipped
            if (kEnableNpcHelmetAttachmentsMainPath && extra.equipDisplayId[0] != 0 && itemDisplayDbc) {
                int32_t helmIdx = itemDisplayDbc->findRecordById(extra.equipDisplayId[0]);
                if (helmIdx >= 0) {
                    // Get helmet model name from ItemDisplayInfo.dbc (LeftModel)
                    std::string helmModelName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), idiL ? (*idiL)["LeftModel"] : 1);
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
                            // Load skin (only for WotLK M2s)
                            std::string skinPath = helmPath.substr(0, helmPath.size() - 3) + "00.skin";
                            auto skinData = assetManager->readFile(skinPath);
                            if (!skinData.empty() && helmModel.version >= 264) {
                                pipeline::M2Loader::loadSkin(skinData, helmModel);
                            }

                            if (helmModel.isValid()) {
                                // Attachment point 11 = Head
                                uint32_t helmModelId = nextCreatureModelId_++;
                                // Get texture from ItemDisplayInfo (LeftModelTexture)
                                std::string helmTexName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), idiL ? (*idiL)["LeftModelTexture"] : 3);
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
                                bool attached = charRenderer->attachWeapon(instanceId, 0, helmModel, helmModelId, helmTexPath);
                                if (!attached) {
                                    attached = charRenderer->attachWeapon(instanceId, 11, helmModel, helmModelId, helmTexPath);
                                }
                                if (attached) {
                                    LOG_DEBUG("Attached helmet model: ", helmPath, " tex: ", helmTexPath);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // With full humanoid overrides disabled, some character-style NPC models still render
    // conflicting clothing geosets at once (global capes, robe skirts over trousers).
    // Normalize only clothing groups while leaving all other model batches untouched.
    if (const auto* md = charRenderer->getModelData(modelId)) {
        std::unordered_set<uint16_t> allGeosets;
        std::unordered_map<uint16_t, uint16_t> firstByGroup;
        bool hasGroup12 = false; // tabard variants
        bool hasGroup13 = false; // trousers/robe skirt variants
        bool hasGroup15 = false; // cloak variants
        for (const auto& b : md->batches) {
            const uint16_t sid = b.submeshId;
            const uint16_t group = static_cast<uint16_t>(sid / 100);
            allGeosets.insert(sid);
            auto itFirst = firstByGroup.find(group);
            if (itFirst == firstByGroup.end() || sid < itFirst->second) {
                firstByGroup[group] = sid;
            }
            if (group == 12) hasGroup12 = true;
            if (group == 13) hasGroup13 = true;
            if (group == 15) hasGroup15 = true;
        }

        // Only apply to humanoid-like clothing models.
        if (hasGroup12 || hasGroup13 || hasGroup15) {
            bool hasRenderableCape = false;
            bool hasEquippedTabard = false;
            if (itDisplayData != displayDataMap_.end() &&
                itDisplayData->second.extraDisplayId != 0) {
                auto itExtra = humanoidExtraMap_.find(itDisplayData->second.extraDisplayId);
                if (itExtra != humanoidExtraMap_.end()) {
                    hasEquippedTabard = (itExtra->second.equipDisplayId[9] != 0);
                    uint32_t capeDisplayId = itExtra->second.equipDisplayId[10];
                    if (capeDisplayId != 0) {
                        auto itemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
                        const auto* idiL = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                        if (itemDisplayDbc) {
                            int32_t recIdx = itemDisplayDbc->findRecordById(capeDisplayId);
                            if (recIdx >= 0) {
                                const uint32_t leftTexField = idiL ? (*idiL)["LeftModelTexture"] : 3u;
                                const uint32_t rightTexField = idiL ? (*idiL)["RightModelTexture"] : 4u;
                                std::vector<std::string> capeNames;
                                auto addName = [&](const std::string& n) {
                                    if (!n.empty() &&
                                        std::find(capeNames.begin(), capeNames.end(), n) == capeNames.end()) {
                                        capeNames.push_back(n);
                                    }
                                };
                                addName(itemDisplayDbc->getString(static_cast<uint32_t>(recIdx), leftTexField));
                                addName(itemDisplayDbc->getString(static_cast<uint32_t>(recIdx), rightTexField));

                                auto hasBlpExt = [](const std::string& p) {
                                    if (p.size() < 4) return false;
                                    std::string ext = p.substr(p.size() - 4);
                                    std::transform(ext.begin(), ext.end(), ext.begin(),
                                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                                    return ext == ".blp";
                                };

                                const bool npcIsFemale = (itExtra->second.sexId == 1);
                                std::vector<std::string> candidates;
                                auto addCandidate = [&](const std::string& p) {
                                    if (p.empty()) return;
                                    if (std::find(candidates.begin(), candidates.end(), p) == candidates.end()) {
                                        candidates.push_back(p);
                                    }
                                };

                                for (const auto& raw : capeNames) {
                                    std::string name = raw;
                                    std::replace(name.begin(), name.end(), '/', '\\');
                                    const bool hasDir = (name.find('\\') != std::string::npos);
                                    const bool hasExt = hasBlpExt(name);
                                    if (hasDir) {
                                        addCandidate(name);
                                        if (!hasExt) addCandidate(name + ".blp");
                                    } else {
                                        std::string baseObj = "Item\\ObjectComponents\\Cape\\" + name;
                                        std::string baseTex = "Item\\TextureComponents\\Cape\\" + name;
                                        addCandidate(baseObj);
                                        addCandidate(baseTex);
                                        if (!hasExt) {
                                            addCandidate(baseObj + ".blp");
                                            addCandidate(baseTex + ".blp");
                                        }
                                        addCandidate(baseObj + (npcIsFemale ? "_F.blp" : "_M.blp"));
                                        addCandidate(baseObj + "_U.blp");
                                        addCandidate(baseTex + (npcIsFemale ? "_F.blp" : "_M.blp"));
                                        addCandidate(baseTex + "_U.blp");
                                    }
                                }

                                for (const auto& p : candidates) {
                                    if (assetManager->fileExists(p)) {
                                        hasRenderableCape = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            std::unordered_set<uint16_t> normalizedGeosets;
            for (uint16_t sid : allGeosets) {
                const uint16_t group = static_cast<uint16_t>(sid / 100);
                if (group == 12 || group == 13 || group == 15) continue;
                // Some humanoid models carry cloak cloth in group 16. Strip this too
                // when no cape is equipped to avoid "everyone has a cape".
                if (!hasRenderableCape && group == 16) continue;
                normalizedGeosets.insert(sid);
            }

            auto pickFromGroup = [&](uint16_t preferredSid, uint16_t group) -> uint16_t {
                if (allGeosets.count(preferredSid) > 0) return preferredSid;
                auto it = firstByGroup.find(group);
                if (it != firstByGroup.end()) return it->second;
                return 0;
            };

            // Show tabard mesh only when CreatureDisplayInfoExtra equips one.
            if (hasGroup12 && hasEquippedTabard) {
                uint16_t tabardSid = pickFromGroup(1201, 12);
                if (tabardSid != 0) normalizedGeosets.insert(tabardSid);
            }

            // Prefer trousers geoset, not robe/kilt overlays.
            if (hasGroup13) {
                uint16_t pantsSid = pickFromGroup(1301, 13);
                if (pantsSid != 0) normalizedGeosets.insert(pantsSid);
            }

            // Prefer explicit cloak variant only when a cape is equipped.
            if (hasGroup15 && hasRenderableCape) {
                uint16_t capeSid = pickFromGroup(1502, 15);
                if (capeSid != 0) normalizedGeosets.insert(capeSid);
            }

            if (!normalizedGeosets.empty()) {
                charRenderer->setActiveGeosets(instanceId, normalizedGeosets);
            }
        }
    }

    // Optional NPC helmet attachments (kept disabled for stability: this path
    // can increase spawn-time pressure and regress NPC visibility in crowded areas).
    static constexpr bool kEnableNpcHelmetAttachments = false;
    if (kEnableNpcHelmetAttachments &&
        itDisplayData != displayDataMap_.end() &&
        itDisplayData->second.extraDisplayId != 0) {
        auto itExtra = humanoidExtraMap_.find(itDisplayData->second.extraDisplayId);
        if (itExtra != humanoidExtraMap_.end()) {
            const auto& extra = itExtra->second;
            if (extra.equipDisplayId[0] != 0) { // Helm slot
                auto itemDisplayDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
                const auto* idiL2 = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                if (itemDisplayDbc) {
                    int32_t helmIdx = itemDisplayDbc->findRecordById(extra.equipDisplayId[0]);
                    if (helmIdx >= 0) {
                        std::string helmModelName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), idiL2 ? (*idiL2)["LeftModel"] : 1);
                        if (!helmModelName.empty()) {
                            size_t dotPos = helmModelName.rfind('.');
                            if (dotPos != std::string::npos) {
                                helmModelName = helmModelName.substr(0, dotPos);
                            }

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
                                std::string skinPath = helmPath.substr(0, helmPath.size() - 3) + "00.skin";
                                auto skinData = assetManager->readFile(skinPath);
                                if (!skinData.empty() && helmModel.version >= 264) {
                                    pipeline::M2Loader::loadSkin(skinData, helmModel);
                                }

                                if (helmModel.isValid()) {
                                    uint32_t helmModelId = nextCreatureModelId_++;
                                    std::string helmTexName = itemDisplayDbc->getString(static_cast<uint32_t>(helmIdx), idiL2 ? (*idiL2)["LeftModelTexture"] : 3);
                                    std::string helmTexPath;
                                    if (!helmTexName.empty()) {
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
                                    // Attachment point 11 = Head
                                    charRenderer->attachWeapon(instanceId, 11, helmModel, helmModelId, helmTexPath);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Spawn in the correct pose. If the server marked this creature dead before
    // the queued spawn was processed, start directly in death animation.
    if (deadCreatureGuids_.count(guid)) {
        charRenderer->playAnimation(instanceId, 1, false); // Death
    } else {
        charRenderer->playAnimation(instanceId, 0, true); // Idle
    }
    charRenderer->startFadeIn(instanceId, 0.5f);

    // Track instance
    creatureInstances_[guid] = instanceId;
    creatureModelIds_[guid] = modelId;
    creatureRenderPosCache_[guid] = renderPos;
    LOG_DEBUG("Spawned creature: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
}

void Application::spawnOnlinePlayer(uint64_t guid,
                                    uint8_t raceId,
                                    uint8_t genderId,
                                    uint32_t appearanceBytes,
                                    uint8_t facialFeatures,
                                    float x, float y, float z, float orientation) {
    if (!renderer || !renderer->getCharacterRenderer() || !assetManager || !assetManager->isInitialized()) return;
    if (playerInstances_.count(guid)) return;

    // Skip local player — already spawned as the main character
    if (gameHandler) {
        uint64_t localGuid = gameHandler->getPlayerGuid();
        uint64_t activeGuid = gameHandler->getActiveCharacterGuid();
        if ((localGuid != 0 && guid == localGuid) ||
            (activeGuid != 0 && guid == activeGuid) ||
            (spawnedPlayerGuid_ != 0 && guid == spawnedPlayerGuid_)) {
            return;
        }
    }
    auto* charRenderer = renderer->getCharacterRenderer();

    // Base geometry model: cache by (race, gender)
    uint32_t cacheKey = (static_cast<uint32_t>(raceId) << 8) | static_cast<uint32_t>(genderId & 0xFF);
    uint32_t modelId = 0;
    auto itCache = playerModelCache_.find(cacheKey);
    if (itCache != playerModelCache_.end()) {
        modelId = itCache->second;
    } else {
        game::Race race = static_cast<game::Race>(raceId);
        game::Gender gender = (genderId == 1) ? game::Gender::FEMALE : game::Gender::MALE;
        std::string m2Path = game::getPlayerModelPath(race, gender);
        if (m2Path.empty()) {
            LOG_WARNING("spawnOnlinePlayer: unknown race/gender for guid 0x", std::hex, guid, std::dec,
                        " race=", (int)raceId, " gender=", (int)genderId);
            return;
        }

        // Parse modelDir/baseName for skin/anim loading
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
            if (dot != std::string::npos) baseName = baseName.substr(0, dot);
        }

        auto m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            LOG_WARNING("spawnOnlinePlayer: failed to read M2: ", m2Path);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (!model.isValid() || model.vertices.empty()) {
            LOG_WARNING("spawnOnlinePlayer: failed to parse M2: ", m2Path);
            return;
        }

        // Skin file (only for WotLK M2s - vanilla has embedded skin)
        std::string skinPath = modelDir + baseName + "00.skin";
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty() && model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        // Load only core external animations (stand/walk/run) to avoid stalls
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                uint32_t animId = model.sequences[si].id;
                if (animId != 0 && animId != 4 && animId != 5) continue;
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName),
                         "%s%s%04u-%02u.anim",
                         modelDir.c_str(),
                         baseName.c_str(),
                         animId,
                         model.sequences[si].variationIndex);
                auto animData = assetManager->readFileOptional(animFileName);
                if (!animData.empty()) {
                    pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
                }
            }
        }

        modelId = nextPlayerModelId_++;
        if (!charRenderer->loadModel(model, modelId)) {
            LOG_WARNING("spawnOnlinePlayer: failed to load model to GPU: ", m2Path);
            return;
        }

        playerModelCache_[cacheKey] = modelId;
    }

    // Determine texture slots once per model
    if (!playerTextureSlotsByModelId_.count(modelId)) {
        PlayerTextureSlots slots;
        if (const auto* md = charRenderer->getModelData(modelId)) {
            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                uint32_t t = md->textures[ti].type;
                if (t == 1 && slots.skin < 0) slots.skin = (int)ti;
                else if (t == 6 && slots.hair < 0) slots.hair = (int)ti;
                else if (t == 8 && slots.underwear < 0) slots.underwear = (int)ti;
            }
        }
        playerTextureSlotsByModelId_[modelId] = slots;
    }

    // Create instance at server position
    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
    float renderYaw = orientation + glm::radians(90.0f);
    uint32_t instanceId = charRenderer->createInstance(modelId, renderPos, glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);
    if (instanceId == 0) return;

    // Resolve skin/hair texture paths via CharSections, then apply as per-instance overrides
    const char* raceFolderName = "Human";
    switch (static_cast<game::Race>(raceId)) {
        case game::Race::HUMAN: raceFolderName = "Human"; break;
        case game::Race::ORC: raceFolderName = "Orc"; break;
        case game::Race::DWARF: raceFolderName = "Dwarf"; break;
        case game::Race::NIGHT_ELF: raceFolderName = "NightElf"; break;
        case game::Race::UNDEAD: raceFolderName = "Scourge"; break;
        case game::Race::TAUREN: raceFolderName = "Tauren"; break;
        case game::Race::GNOME: raceFolderName = "Gnome"; break;
        case game::Race::TROLL: raceFolderName = "Troll"; break;
        case game::Race::BLOOD_ELF: raceFolderName = "BloodElf"; break;
        case game::Race::DRAENEI: raceFolderName = "Draenei"; break;
        default: break;
    }
    const char* genderFolder = (genderId == 1) ? "Female" : "Male";
    std::string raceGender = std::string(raceFolderName) + genderFolder;
    std::string bodySkinPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "Skin00_00.blp";
    std::string pelvisPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "NakedPelvisSkin00_00.blp";
    std::vector<std::string> underwearPaths;
    std::string hairTexturePath;
    std::string faceLowerPath;
    std::string faceUpperPath;

    uint8_t skinId = appearanceBytes & 0xFF;
    uint8_t faceId = (appearanceBytes >> 8) & 0xFF;
    uint8_t hairStyleId = (appearanceBytes >> 16) & 0xFF;
    uint8_t hairColorId = (appearanceBytes >> 24) & 0xFF;

    if (auto charSectionsDbc = assetManager->loadDBC("CharSections.dbc"); charSectionsDbc && charSectionsDbc->isLoaded()) {
        const auto* csL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
        uint32_t targetRaceId = raceId;
        uint32_t targetSexId = genderId;
        const uint32_t csTex1 = csL ? (*csL)["Texture1"] : 6;

        bool foundSkin = false;
        bool foundUnderwear = false;
        bool foundHair = false;
        bool foundFaceLower = false;

        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
            uint32_t rRace = charSectionsDbc->getUInt32(r, csL ? (*csL)["RaceID"] : 1);
            uint32_t rSex = charSectionsDbc->getUInt32(r, csL ? (*csL)["SexID"] : 2);
            uint32_t baseSection = charSectionsDbc->getUInt32(r, csL ? (*csL)["BaseSection"] : 3);
            uint32_t variationIndex = charSectionsDbc->getUInt32(r, csL ? (*csL)["VariationIndex"] : 4);
            uint32_t colorIndex = charSectionsDbc->getUInt32(r, csL ? (*csL)["ColorIndex"] : 5);

            if (rRace != targetRaceId || rSex != targetSexId) continue;

            if (baseSection == 0 && !foundSkin && colorIndex == skinId) {
                std::string tex1 = charSectionsDbc->getString(r, csTex1);
                if (!tex1.empty()) { bodySkinPath = tex1; foundSkin = true; }
            } else if (baseSection == 3 && !foundHair &&
                       variationIndex == hairStyleId && colorIndex == hairColorId) {
                hairTexturePath = charSectionsDbc->getString(r, csTex1);
                if (!hairTexturePath.empty()) foundHair = true;
            } else if (baseSection == 4 && !foundUnderwear && colorIndex == skinId) {
                for (uint32_t f = csTex1; f <= csTex1 + 2; f++) {
                    std::string tex = charSectionsDbc->getString(r, f);
                    if (!tex.empty()) underwearPaths.push_back(tex);
                }
                foundUnderwear = true;
            } else if (baseSection == 1 && !foundFaceLower &&
                       variationIndex == faceId && colorIndex == skinId) {
                std::string tex1 = charSectionsDbc->getString(r, csTex1);
                std::string tex2 = charSectionsDbc->getString(r, csTex1 + 1);
                if (!tex1.empty()) faceLowerPath = tex1;
                if (!tex2.empty()) faceUpperPath = tex2;
                foundFaceLower = true;
            }

            if (foundSkin && foundUnderwear && foundHair && foundFaceLower) break;
        }
    }

    // Composite base skin + face + underwear overlays
    GLuint compositeTex = 0;
    {
        std::vector<std::string> layers;
        layers.push_back(bodySkinPath);
        if (!faceLowerPath.empty()) layers.push_back(faceLowerPath);
        if (!faceUpperPath.empty()) layers.push_back(faceUpperPath);
        for (const auto& up : underwearPaths) layers.push_back(up);
        if (layers.size() > 1) {
            compositeTex = charRenderer->compositeTextures(layers);
        } else {
            compositeTex = charRenderer->loadTexture(bodySkinPath);
        }
    }

    GLuint hairTex = 0;
    if (!hairTexturePath.empty()) {
        hairTex = charRenderer->loadTexture(hairTexturePath);
    }
    GLuint underwearTex = 0;
    if (!underwearPaths.empty()) underwearTex = charRenderer->loadTexture(underwearPaths[0]);
    else underwearTex = charRenderer->loadTexture(pelvisPath);

    const PlayerTextureSlots& slots = playerTextureSlotsByModelId_[modelId];
    if (slots.skin >= 0 && compositeTex != 0) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.skin), compositeTex);
    }
    if (slots.hair >= 0 && hairTex != 0) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.hair), hairTex);
    }
    if (slots.underwear >= 0 && underwearTex != 0) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.underwear), underwearTex);
    }

    // Geosets: body + hair/facial hair selections
    std::unordered_set<uint16_t> activeGeosets;
    // Body parts (group 0: IDs 0-99, some models use up to 27)
    for (uint16_t i = 0; i <= 99; i++) activeGeosets.insert(i);
    activeGeosets.insert(static_cast<uint16_t>(100 + hairStyleId + 1));
    activeGeosets.insert(static_cast<uint16_t>(200 + facialFeatures + 1));
    activeGeosets.insert(401);   // Bare forearms (no gloves) — group 4
    activeGeosets.insert(502);   // Bare shins (no boots) — group 5
    activeGeosets.insert(702);   // Ears
    activeGeosets.insert(801);   // Bare wrists (no sleeves) — group 8
    activeGeosets.insert(902);   // Kneepads — group 9
    activeGeosets.insert(1301);  // Bare legs — group 13
    activeGeosets.insert(1502);  // No cloak — group 15
    activeGeosets.insert(2002);  // Bare feet — group 20
    charRenderer->setActiveGeosets(instanceId, activeGeosets);

    charRenderer->playAnimation(instanceId, 0, true);
    playerInstances_[guid] = instanceId;

    OnlinePlayerAppearanceState st;
    st.instanceId = instanceId;
    st.modelId = modelId;
    st.raceId = raceId;
    st.genderId = genderId;
    st.appearanceBytes = appearanceBytes;
    st.facialFeatures = facialFeatures;
    st.bodySkinPath = bodySkinPath;
    // Include face textures so compositeWithRegions can rebuild the full base
    if (!faceLowerPath.empty()) st.underwearPaths.push_back(faceLowerPath);
    if (!faceUpperPath.empty()) st.underwearPaths.push_back(faceUpperPath);
    for (const auto& up : underwearPaths) st.underwearPaths.push_back(up);
    onlinePlayerAppearance_[guid] = std::move(st);
}

void Application::setOnlinePlayerEquipment(uint64_t guid,
                                          const std::array<uint32_t, 19>& displayInfoIds,
                                          const std::array<uint8_t, 19>& inventoryTypes) {
    if (!renderer || !renderer->getCharacterRenderer() || !assetManager || !assetManager->isInitialized()) return;

    // Skip local player — equipment handled by GameScreen::updateCharacterGeosets/Textures
    // via consumeOnlineEquipmentDirty(), which fires on the same server update.
    if (gameHandler) {
        uint64_t localGuid = gameHandler->getPlayerGuid();
        if (localGuid != 0 && guid == localGuid) return;
    }

    // If the player isn't spawned yet, store equipment until spawn.
    if (!playerInstances_.count(guid) || !onlinePlayerAppearance_.count(guid)) {
        pendingOnlinePlayerEquipment_[guid] = {displayInfoIds, inventoryTypes};
        return;
    }

    auto it = onlinePlayerAppearance_.find(guid);
    if (it == onlinePlayerAppearance_.end()) return;
    const OnlinePlayerAppearanceState& st = it->second;

    auto* charRenderer = renderer->getCharacterRenderer();
    if (!charRenderer) return;
    if (st.instanceId == 0 || st.modelId == 0) return;

    if (st.bodySkinPath.empty()) return;

    auto displayInfoDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;
    const auto* idiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;

    auto getGeosetGroup = [&](uint32_t displayInfoId, uint32_t fieldIdx) -> uint32_t {
        if (displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), fieldIdx);
    };

    auto findDisplayIdByInvType = [&](std::initializer_list<uint8_t> types) -> uint32_t {
        for (int s = 0; s < 19; s++) {
            uint8_t inv = inventoryTypes[s];
            if (inv == 0 || displayInfoIds[s] == 0) continue;
            for (uint8_t t : types) {
                if (inv == t) return displayInfoIds[s];
            }
        }
        return 0;
    };

    auto hasInvType = [&](std::initializer_list<uint8_t> types) -> bool {
        for (int s = 0; s < 19; s++) {
            uint8_t inv = inventoryTypes[s];
            if (inv == 0) continue;
            for (uint8_t t : types) {
                if (inv == t) return true;
            }
        }
        return false;
    };

    // --- Geosets ---
    std::unordered_set<uint16_t> geosets;
    // Body parts (group 0: IDs 0-99, some models use up to 27)
    for (uint16_t i = 0; i <= 99; i++) geosets.insert(i);

    uint8_t hairStyleId = static_cast<uint8_t>((st.appearanceBytes >> 16) & 0xFF);
    geosets.insert(static_cast<uint16_t>(100 + hairStyleId + 1));
    geosets.insert(static_cast<uint16_t>(200 + st.facialFeatures + 1));
    geosets.insert(401);   // Body joint patches (knees)
    geosets.insert(402);   // Body joint patches (elbows)
    geosets.insert(701);   // Ears
    geosets.insert(902);   // Kneepads
    geosets.insert(2002);  // Bare feet mesh

    const uint32_t geosetGroup1Field = idiL ? (*idiL)["GeosetGroup1"] : 7;
    const uint32_t geosetGroup3Field = idiL ? (*idiL)["GeosetGroup3"] : 9;

    // Chest/Shirt/Robe (invType 4,5,20)
    {
        uint32_t did = findDisplayIdByInvType({4, 5, 20});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        geosets.insert(static_cast<uint16_t>(gg1 > 0 ? 501 + gg1 : 501));

        uint32_t gg3 = getGeosetGroup(did, geosetGroup3Field);
        if (gg3 > 0) geosets.insert(static_cast<uint16_t>(1301 + gg3));
    }

    // Legs (invType 7)
    {
        uint32_t did = findDisplayIdByInvType({7});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (geosets.count(1302) == 0 && geosets.count(1303) == 0) {
            geosets.insert(static_cast<uint16_t>(gg1 > 0 ? 1301 + gg1 : 1301));
        }
    }

    // Feet (invType 8): 401/402 are body patches (always on), 403+ are boot meshes
    {
        uint32_t did = findDisplayIdByInvType({8});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosets.insert(static_cast<uint16_t>(402 + gg1));
    }

    // Hands (invType 10)
    {
        uint32_t did = findDisplayIdByInvType({10});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        geosets.insert(static_cast<uint16_t>(gg1 > 0 ? 301 + gg1 : 301));
    }

    // Back/Cloak (invType 16)
    geosets.insert(hasInvType({16}) ? 1502 : 1501);
    // Tabard (invType 19)
    if (hasInvType({19})) geosets.insert(1201);

    charRenderer->setActiveGeosets(st.instanceId, geosets);

    // --- Textures (skin atlas compositing) ---
    static const char* componentDirs[] = {
        "ArmUpperTexture",
        "ArmLowerTexture",
        "HandTexture",
        "TorsoUpperTexture",
        "TorsoLowerTexture",
        "LegUpperTexture",
        "LegLowerTexture",
        "FootTexture",
    };

    // Texture component region fields from DBC layout
    // Binary DBC (23 fields) has textures at 14+
    const uint32_t texRegionFields[8] = {
        idiL ? (*idiL)["TextureArmUpper"]  : 14u,
        idiL ? (*idiL)["TextureArmLower"]  : 15u,
        idiL ? (*idiL)["TextureHand"]      : 16u,
        idiL ? (*idiL)["TextureTorsoUpper"]: 17u,
        idiL ? (*idiL)["TextureTorsoLower"]: 18u,
        idiL ? (*idiL)["TextureLegUpper"]  : 19u,
        idiL ? (*idiL)["TextureLegLower"]  : 20u,
        idiL ? (*idiL)["TextureFoot"]      : 21u,
    };

    std::vector<std::pair<int, std::string>> regionLayers;
    const bool isFemale = (st.genderId == 1);

    for (int s = 0; s < 19; s++) {
        uint32_t did = displayInfoIds[s];
        if (did == 0) continue;
        int32_t recIdx = displayInfoDbc->findRecordById(did);
        if (recIdx < 0) continue;

        for (int region = 0; region < 8; region++) {
            std::string texName = displayInfoDbc->getString(
                static_cast<uint32_t>(recIdx), texRegionFields[region]);
            if (texName.empty()) continue;

            std::string base = "Item\\TextureComponents\\" + std::string(componentDirs[region]) + "\\" + texName;
            std::string genderPath = base + (isFemale ? "_F.blp" : "_M.blp");
            std::string unisexPath = base + "_U.blp";
            std::string fullPath;
            if (assetManager->fileExists(genderPath)) fullPath = genderPath;
            else if (assetManager->fileExists(unisexPath)) fullPath = unisexPath;
            else fullPath = base + ".blp";

            regionLayers.emplace_back(region, fullPath);
        }
    }

    const auto slotsIt = playerTextureSlotsByModelId_.find(st.modelId);
    if (slotsIt == playerTextureSlotsByModelId_.end()) return;
    const PlayerTextureSlots& slots = slotsIt->second;
    if (slots.skin < 0) return;

    GLuint newTex = charRenderer->compositeWithRegions(st.bodySkinPath, st.underwearPaths, regionLayers);
    if (newTex != 0) {
        charRenderer->setTextureSlotOverride(st.instanceId, static_cast<uint16_t>(slots.skin), newTex);
    }
}

void Application::despawnOnlinePlayer(uint64_t guid) {
    if (!renderer || !renderer->getCharacterRenderer()) return;
    auto it = playerInstances_.find(guid);
    if (it == playerInstances_.end()) return;
    renderer->getCharacterRenderer()->removeInstance(it->second);
    playerInstances_.erase(it);
    onlinePlayerAppearance_.erase(guid);
    pendingOnlinePlayerEquipment_.erase(guid);
}

void Application::spawnOnlineGameObject(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation) {
    if (!renderer || !assetManager) return;

    if (!gameObjectLookupsBuilt_) {
        buildGameObjectDisplayLookups();
    }
    if (!gameObjectLookupsBuilt_) return;

    if (gameObjectInstances_.count(guid)) {
        // Already have a render instance — update its position (e.g. transport re-creation)
        auto& info = gameObjectInstances_[guid];
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        LOG_DEBUG("GameObject position update: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
                 " pos=(", x, ", ", y, ", ", z, ")");
        if (renderer) {
            if (info.isWmo) {
                if (auto* wr = renderer->getWMORenderer()) {
                    glm::mat4 transform(1.0f);
                    transform = glm::translate(transform, renderPos);
                    transform = glm::rotate(transform, orientation, glm::vec3(0, 0, 1));
                    wr->setInstanceTransform(info.instanceId, transform);
                }
            } else {
                if (auto* mr = renderer->getM2Renderer()) {
                    glm::mat4 transform(1.0f);
                    transform = glm::translate(transform, renderPos);
                    transform = glm::rotate(transform, orientation, glm::vec3(0, 0, 1));
                    mr->setInstanceTransform(info.instanceId, transform);
                }
            }
        }
        return;
    }

    std::string modelPath;

        // Override model path for transports with wrong displayIds (preloaded transports)
        // Check if this GUID is a known transport
        bool isTransport = gameHandler && gameHandler->isTransportGuid(guid);
        if (isTransport) {
            // Map common transport displayIds to correct WMO paths
            // NOTE: displayIds 455/462 are elevators in Thunder Bluff and should NOT be forced to ships.
            // Keep ship/zeppelin overrides entry-driven where possible.
            // DisplayIds 807, 808 = Zeppelins
            // DisplayIds 2454, 1587 = Special ships/icebreakers
            if (entry == 20808 || entry == 176231 || entry == 176310) {
                modelPath = "World\\wmo\\transports\\transport_ship\\transportship.wmo";
                LOG_INFO("Overriding transport entry/display ", entry, "/", displayId, " → transportship.wmo");
            } else if (displayId == 807 || displayId == 808 || displayId == 175080 || displayId == 176495 || displayId == 164871) {
                modelPath = "World\\wmo\\transports\\transport_zeppelin\\transport_zeppelin.wmo";
                LOG_INFO("Overriding transport displayId ", displayId, " → transport_zeppelin.wmo");
            } else if (displayId == 1587) {
                modelPath = "World\\wmo\\transports\\transport_horde_zeppelin\\Transport_Horde_Zeppelin.wmo";
                LOG_INFO("Overriding transport displayId ", displayId, " → Transport_Horde_Zeppelin.wmo");
            } else if (displayId == 2454 || displayId == 181688 || displayId == 190536) {
                modelPath = "World\\wmo\\transports\\icebreaker\\Transport_Icebreaker_ship.wmo";
                LOG_INFO("Overriding transport displayId ", displayId, " → Transport_Icebreaker_ship.wmo");
            }
        }

    // Fallback to normal displayId lookup if not a transport or no override matched
    if (modelPath.empty()) {
        modelPath = getGameObjectModelPathForDisplayId(displayId);
    }

    if (modelPath.empty()) {
        LOG_WARNING("No model path for gameobject displayId ", displayId, " (guid 0x", std::hex, guid, std::dec, ")");
        return;
    }

    // Log spawns to help debug duplicate objects (e.g., cathedral issue)
    LOG_DEBUG("GameObject spawn: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
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
                LOG_DEBUG("Gameobject WMO root loaded: ", modelPath, " nGroups=", wmoModel.nGroups);
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
            LOG_DEBUG("Spawned gameobject WMO: guid=0x", std::hex, guid, std::dec,
                     " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");

            // Spawn transport WMO doodads (chairs, furniture, etc.) as child M2 instances
            bool isTransport = false;
            if (gameHandler) {
                std::string lowerModelPath = modelPath;
                std::transform(lowerModelPath.begin(), lowerModelPath.end(), lowerModelPath.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                isTransport = (lowerModelPath.find("transport") != std::string::npos);
            }

            auto* m2Renderer = renderer->getM2Renderer();
            if (m2Renderer && isTransport) {
                const auto* doodadTemplates = wmoRenderer->getDoodadTemplates(modelId);
                if (doodadTemplates && !doodadTemplates->empty()) {
                    constexpr size_t kMaxTransportDoodads = 192;
                    const size_t doodadBudget = std::min(doodadTemplates->size(), kMaxTransportDoodads);
                    LOG_DEBUG("Queueing ", doodadBudget, "/", doodadTemplates->size(),
                             " transport doodads for WMO instance ", instanceId);
                    pendingTransportDoodadBatches_.push_back(PendingTransportDoodadBatch{
                        guid,
                        modelId,
                        instanceId,
                        0,
                        doodadBudget,
                        0,
                        x, y, z,
                        orientation
                    });
                } else {
                LOG_DEBUG("Transport WMO has no doodads or templates not available");
            }
            }

            // Transport GameObjects are not always named "transport" in their WMO path
            // (e.g. elevators/lifts). If the server marks it as a transport, always
            // notify so TransportManager can animate/carry passengers.
            if (gameHandler && gameHandler->isTransportGuid(guid)) {
                gameHandler->notifyTransportSpawned(guid, entry, displayId, x, y, z, orientation);
            }

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
            if (!skinData.empty() && model.version >= 264) {
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

    LOG_DEBUG("Spawned gameobject: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
}

void Application::processCreatureSpawnQueue() {
    if (pendingCreatureSpawns_.empty()) return;
    if (!creatureLookupsBuilt_) {
        buildCreatureDisplayLookups();
        if (!creatureLookupsBuilt_) return;
    }

    int processed = 0;
    int newModelLoads = 0;
    size_t rotationsLeft = pendingCreatureSpawns_.size();
    while (!pendingCreatureSpawns_.empty() &&
           processed < MAX_SPAWNS_PER_FRAME &&
           rotationsLeft > 0) {
        PendingCreatureSpawn s = pendingCreatureSpawns_.front();
        pendingCreatureSpawns_.erase(pendingCreatureSpawns_.begin());

        const bool needsNewModel = (displayIdModelCache_.find(s.displayId) == displayIdModelCache_.end());
        if (needsNewModel && newModelLoads >= MAX_NEW_CREATURE_MODELS_PER_FRAME) {
            // Defer additional first-time model/texture loads to later frames so
            // movement stays responsive in dense areas.
            pendingCreatureSpawns_.push_back(s);
            rotationsLeft--;
            continue;
        }

        spawnOnlineCreature(s.guid, s.displayId, s.x, s.y, s.z, s.orientation);
        pendingCreatureSpawnGuids_.erase(s.guid);

        // If spawn still failed, retry for a limited number of frames.
        if (!creatureInstances_.count(s.guid)) {
            if (creaturePermanentFailureGuids_.erase(s.guid) > 0) {
                creatureSpawnRetryCounts_.erase(s.guid);
                processed++;
                continue;
            }
            uint16_t retries = 0;
            auto it = creatureSpawnRetryCounts_.find(s.guid);
            if (it != creatureSpawnRetryCounts_.end()) {
                retries = it->second;
            }
            if (retries < MAX_CREATURE_SPAWN_RETRIES) {
                creatureSpawnRetryCounts_[s.guid] = static_cast<uint16_t>(retries + 1);
                pendingCreatureSpawns_.push_back(s);
                pendingCreatureSpawnGuids_.insert(s.guid);
            } else {
                creatureSpawnRetryCounts_.erase(s.guid);
                LOG_WARNING("Dropping creature spawn after retries: guid=0x", std::hex, s.guid, std::dec,
                            " displayId=", s.displayId);
            }
        } else {
            creatureSpawnRetryCounts_.erase(s.guid);
        }
        if (needsNewModel) {
            newModelLoads++;
        }
        rotationsLeft = pendingCreatureSpawns_.size();
        processed++;
    }
}

void Application::processPlayerSpawnQueue() {
    if (pendingPlayerSpawns_.empty()) return;
    if (!assetManager || !assetManager->isInitialized()) return;

    int processed = 0;
    while (!pendingPlayerSpawns_.empty() && processed < MAX_SPAWNS_PER_FRAME) {
        PendingPlayerSpawn s = pendingPlayerSpawns_.front();
        pendingPlayerSpawns_.erase(pendingPlayerSpawns_.begin());
        pendingPlayerSpawnGuids_.erase(s.guid);

        // Skip if already spawned (could have been spawned by a previous update this frame)
        if (playerInstances_.count(s.guid)) {
            processed++;
            continue;
        }

        spawnOnlinePlayer(s.guid, s.raceId, s.genderId, s.appearanceBytes, s.facialFeatures, s.x, s.y, s.z, s.orientation);
        // Apply any equipment updates that arrived before the player was spawned.
        auto pit = pendingOnlinePlayerEquipment_.find(s.guid);
        if (pit != pendingOnlinePlayerEquipment_.end()) {
            deferredEquipmentQueue_.push_back({s.guid, pit->second});
            pendingOnlinePlayerEquipment_.erase(pit);
        }
        processed++;
    }
}

void Application::processDeferredEquipmentQueue() {
    if (deferredEquipmentQueue_.empty()) return;
    // Process at most 1 per frame — compositeWithRegions is expensive
    auto [guid, equipData] = deferredEquipmentQueue_.front();
    deferredEquipmentQueue_.erase(deferredEquipmentQueue_.begin());
    setOnlinePlayerEquipment(guid, equipData.first, equipData.second);
}

void Application::processGameObjectSpawnQueue() {
    if (pendingGameObjectSpawns_.empty()) return;

    int spawned = 0;
    while (!pendingGameObjectSpawns_.empty() && spawned < MAX_SPAWNS_PER_FRAME) {
        auto& s = pendingGameObjectSpawns_.front();
        spawnOnlineGameObject(s.guid, s.entry, s.displayId, s.x, s.y, s.z, s.orientation);
        pendingGameObjectSpawns_.erase(pendingGameObjectSpawns_.begin());
        spawned++;
    }
}

void Application::processPendingTransportDoodads() {
    if (pendingTransportDoodadBatches_.empty()) return;
    if (!renderer || !assetManager) return;

    auto* wmoRenderer = renderer->getWMORenderer();
    auto* m2Renderer = renderer->getM2Renderer();
    if (!wmoRenderer || !m2Renderer) return;

    size_t budgetLeft = MAX_TRANSPORT_DOODADS_PER_FRAME;
    for (auto it = pendingTransportDoodadBatches_.begin();
         it != pendingTransportDoodadBatches_.end() && budgetLeft > 0;) {
        auto goIt = gameObjectInstances_.find(it->guid);
        if (goIt == gameObjectInstances_.end() || !goIt->second.isWmo ||
            goIt->second.instanceId != it->instanceId || goIt->second.modelId != it->modelId) {
            it = pendingTransportDoodadBatches_.erase(it);
            continue;
        }

        const auto* doodadTemplates = wmoRenderer->getDoodadTemplates(it->modelId);
        if (!doodadTemplates || doodadTemplates->empty()) {
            it = pendingTransportDoodadBatches_.erase(it);
            continue;
        }

        const size_t maxIndex = std::min(it->doodadBudget, doodadTemplates->size());
        while (it->nextIndex < maxIndex && budgetLeft > 0) {
            const auto& doodadTemplate = (*doodadTemplates)[it->nextIndex];
            it->nextIndex++;
            budgetLeft--;

            uint32_t doodadModelId = static_cast<uint32_t>(std::hash<std::string>{}(doodadTemplate.m2Path));
            auto m2Data = assetManager->readFile(doodadTemplate.m2Path);
            if (m2Data.empty()) continue;

            pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
            std::string skinPath = doodadTemplate.m2Path.substr(0, doodadTemplate.m2Path.size() - 3) + "00.skin";
            std::vector<uint8_t> skinData = assetManager->readFile(skinPath);
            if (!skinData.empty() && m2Model.version >= 264) {
                pipeline::M2Loader::loadSkin(skinData, m2Model);
            }
            if (!m2Model.isValid()) continue;

            m2Renderer->loadModel(m2Model, doodadModelId);
            uint32_t m2InstanceId = m2Renderer->createInstance(doodadModelId, glm::vec3(0.0f), glm::vec3(0.0f), 1.0f);
            if (m2InstanceId == 0) continue;

            wmoRenderer->addDoodadToInstance(it->instanceId, m2InstanceId, doodadTemplate.localTransform);
            it->spawnedDoodads++;
        }

        if (it->nextIndex >= maxIndex) {
            if (it->spawnedDoodads > 0) {
                LOG_DEBUG("Spawned ", it->spawnedDoodads,
                         " transport doodads for WMO instance ", it->instanceId);
                glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(it->x, it->y, it->z));
                glm::mat4 wmoTransform(1.0f);
                wmoTransform = glm::translate(wmoTransform, renderPos);
                wmoTransform = glm::rotate(wmoTransform, it->orientation, glm::vec3(0, 0, 1));
                wmoRenderer->setInstanceTransform(it->instanceId, wmoTransform);
            }
            it = pendingTransportDoodadBatches_.erase(it);
        } else {
            ++it;
        }
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
    auto cacheIt = displayIdModelCache_.find(mountDisplayId);
    if (cacheIt != displayIdModelCache_.end()) {
        modelId = cacheIt->second;
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

        // Load skin file (only for WotLK M2s - vanilla has embedded skin)
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty() && model.version >= 264) {
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
                auto animData = assetManager->readFileOptional(animFileName);
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

    // Apply creature skin textures from CreatureDisplayInfo.dbc.
    // Re-apply even for cached models so transient failures can self-heal.
    std::string modelDir;
    size_t lastSlash = m2Path.find_last_of("\\/");
    if (lastSlash != std::string::npos) {
        modelDir = m2Path.substr(0, lastSlash + 1);
    }

    auto itDisplayData = displayDataMap_.find(mountDisplayId);
    bool haveDisplayData = false;
    CreatureDisplayData dispData{};
    if (itDisplayData != displayDataMap_.end()) {
        dispData = itDisplayData->second;
        haveDisplayData = true;
    } else {
        // Some taxi mount display IDs are sparse; recover skins by matching model path.
        std::string lowerMountPath = m2Path;
        std::transform(lowerMountPath.begin(), lowerMountPath.end(), lowerMountPath.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        int bestScore = -1;
        for (const auto& [dispId, data] : displayDataMap_) {
            auto pit = modelIdToPath_.find(data.modelId);
            if (pit == modelIdToPath_.end()) continue;
            std::string p = pit->second;
            std::transform(p.begin(), p.end(), p.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (p != lowerMountPath) continue;
            int score = 0;
            if (!data.skin1.empty()) {
                std::string p1 = modelDir + data.skin1 + ".blp";
                score += assetManager->fileExists(p1) ? 30 : 3;
            }
            if (!data.skin2.empty()) {
                std::string p2 = modelDir + data.skin2 + ".blp";
                score += assetManager->fileExists(p2) ? 20 : 2;
            }
            if (!data.skin3.empty()) {
                std::string p3 = modelDir + data.skin3 + ".blp";
                score += assetManager->fileExists(p3) ? 10 : 1;
            }
            if (score > bestScore) {
                bestScore = score;
                dispData = data;
                haveDisplayData = true;
            }
        }
        if (haveDisplayData) {
            LOG_INFO("Recovered mount display data by model path for displayId=", mountDisplayId,
                     " skin1='", dispData.skin1, "' skin2='", dispData.skin2,
                     "' skin3='", dispData.skin3, "'");
        }
    }
    if (haveDisplayData) {
        // If this displayId has no skins, try to find another displayId for the same model with skins.
        if (dispData.skin1.empty() && dispData.skin2.empty() && dispData.skin3.empty()) {
            uint32_t sourceModelId = dispData.modelId;
            int bestScore = -1;
            for (const auto& [dispId, data] : displayDataMap_) {
                if (data.modelId != sourceModelId) continue;
                int score = 0;
                if (!data.skin1.empty()) {
                    std::string p = modelDir + data.skin1 + ".blp";
                    score += assetManager->fileExists(p) ? 30 : 3;
                }
                if (!data.skin2.empty()) {
                    std::string p = modelDir + data.skin2 + ".blp";
                    score += assetManager->fileExists(p) ? 20 : 2;
                }
                if (!data.skin3.empty()) {
                    std::string p = modelDir + data.skin3 + ".blp";
                    score += assetManager->fileExists(p) ? 10 : 1;
                }
                if (score > bestScore) {
                    bestScore = score;
                    dispData = data;
                }
            }
            LOG_INFO("Mount skin fallback for displayId=", mountDisplayId,
                     " modelId=", sourceModelId, " skin1='", dispData.skin1,
                     "' skin2='", dispData.skin2, "' skin3='", dispData.skin3, "'");
        }
        const auto* md = charRenderer->getModelData(modelId);
        if (md) {
            LOG_INFO("Mount model textures: ", md->textures.size(), " slots, skin1='", dispData.skin1,
                     "' skin2='", dispData.skin2, "' skin3='", dispData.skin3, "'");
            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                LOG_INFO("  tex[", ti, "] type=", md->textures[ti].type,
                         " filename='", md->textures[ti].filename, "'");
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
                        LOG_INFO("  Applied skin texture slot ", ti, ": ", texPath);
                        replaced++;
                    } else {
                        LOG_WARNING("  Failed to load skin texture slot ", ti, ": ", texPath);
                    }
                }
            }

            // Force skin textures onto type-0 (hardcoded) slots that have no filename
            if (replaced == 0) {
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    const auto& tex = md->textures[ti];
                    if (tex.type == 0 && tex.filename.empty()) {
                        // Empty hardcoded slot — try skin1 then skin2
                        std::string texPath;
                        if (!dispData.skin1.empty() && replaced == 0) {
                            texPath = modelDir + dispData.skin1 + ".blp";
                        } else if (!dispData.skin2.empty()) {
                            texPath = modelDir + dispData.skin2 + ".blp";
                        }
                        if (!texPath.empty()) {
                            GLuint skinTex = charRenderer->loadTexture(texPath);
                            if (skinTex != 0) {
                                charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                                LOG_INFO("  Forced skin on empty hardcoded slot ", ti, ": ", texPath);
                                replaced++;
                            }
                        }
                    }
                }
            }

            // If still no textures, try hardcoded model texture filenames
            if (replaced == 0) {
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    if (!md->textures[ti].filename.empty()) {
                        GLuint texId = charRenderer->loadTexture(md->textures[ti].filename);
                        if (texId != 0) {
                            charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), texId);
                            LOG_INFO("  Used model embedded texture slot ", ti, ": ", md->textures[ti].filename);
                            replaced++;
                        }
                    }
                }
            }

            // Final fallback for gryphon/wyvern: try well-known skin texture names
            if (replaced == 0 && !md->textures.empty()) {
                std::string lowerMountPath = m2Path;
                std::transform(lowerMountPath.begin(), lowerMountPath.end(), lowerMountPath.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (lowerMountPath.find("gryphon") != std::string::npos) {
                    const char* gryphonSkins[] = {
                        "Creature\\Gryphon\\Gryphon_Skin.blp",
                        "Creature\\Gryphon\\Gryphon_Skin01.blp",
                        "Creature\\Gryphon\\GRYPHON_SKIN01.BLP",
                        nullptr
                    };
                    for (const char** p = gryphonSkins; *p; ++p) {
                        GLuint texId = charRenderer->loadTexture(*p);
                        if (texId != 0) {
                            charRenderer->setModelTexture(modelId, 0, texId);
                            LOG_INFO("  Forced gryphon skin fallback: ", *p);
                            replaced++;
                            break;
                        }
                    }
                } else if (lowerMountPath.find("wyvern") != std::string::npos) {
                    const char* wyvernSkins[] = {
                        "Creature\\Wyvern\\Wyvern_Skin.blp",
                        "Creature\\Wyvern\\Wyvern_Skin01.blp",
                        nullptr
                    };
                    for (const char** p = wyvernSkins; *p; ++p) {
                        GLuint texId = charRenderer->loadTexture(*p);
                        if (texId != 0) {
                            charRenderer->setModelTexture(modelId, 0, texId);
                            LOG_INFO("  Forced wyvern skin fallback: ", *p);
                            replaced++;
                            break;
                        }
                    }
                }
            }
            LOG_INFO("Mount texture setup: ", replaced, " textures applied");
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

    renderer->setMounted(instanceId, mountDisplayId, heightOffset, m2Path);

    // For taxi mounts, start with flying animation; for ground mounts, start with stand
    bool isTaxi = gameHandler && gameHandler->isOnTaxiFlight();
    uint32_t startAnim = 0; // ANIM_STAND
    if (isTaxi) {
        // Try WotLK fly anims first, then Vanilla-friendly fallbacks
        uint32_t taxiCandidates[] = {159, 158, 234, 229, 233, 141, 369, 6, 5}; // FlyForward, FlyIdle, FlyRun(234), FlyStand(229), FlyWalk(233), FlyMounted, FlyRun, Fly, Run
        for (uint32_t anim : taxiCandidates) {
            if (charRenderer->hasAnimation(instanceId, anim)) {
                startAnim = anim;
                break;
            }
        }
        // If none found, startAnim stays 0 (Stand/hover) which is fine for flying creatures
    }
    charRenderer->playAnimation(instanceId, startAnim, true);

    LOG_INFO("processPendingMount: DONE displayId=", mountDisplayId, " model=", m2Path, " heightOffset=", heightOffset);
}

void Application::despawnOnlineCreature(uint64_t guid) {
    // If this guid is a PLAYER, it will be tracked in playerInstances_.
    // Route to the correct despawn path so we don't leak instances.
    if (playerInstances_.count(guid)) {
        despawnOnlinePlayer(guid);
        return;
    }

    pendingCreatureSpawnGuids_.erase(guid);
    creatureSpawnRetryCounts_.erase(guid);
    creaturePermanentFailureGuids_.erase(guid);
    deadCreatureGuids_.erase(guid);

    auto it = creatureInstances_.find(guid);
    if (it == creatureInstances_.end()) return;

    if (renderer && renderer->getCharacterRenderer()) {
        renderer->getCharacterRenderer()->removeInstance(it->second);
    }

    creatureInstances_.erase(it);
    creatureModelIds_.erase(guid);
    creatureRenderPosCache_.erase(guid);

    LOG_DEBUG("Despawned creature: guid=0x", std::hex, guid, std::dec);
}

void Application::despawnOnlineGameObject(uint64_t guid) {
    pendingTransportDoodadBatches_.erase(
        std::remove_if(pendingTransportDoodadBatches_.begin(), pendingTransportDoodadBatches_.end(),
                       [guid](const PendingTransportDoodadBatch& b) { return b.guid == guid; }),
        pendingTransportDoodadBatches_.end());

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

    LOG_DEBUG("Despawned gameobject: guid=0x", std::hex, guid, std::dec);
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
    if (!gameHandler || !renderer) {
        return;
    }

    auto* questMarkerRenderer = renderer->getQuestMarkerRenderer();
    if (!questMarkerRenderer) {
        static bool logged = false;
        if (!logged) {
            LOG_WARNING("QuestMarkerRenderer not available!");
            logged = true;
        }
        return;
    }

    const auto& questStatuses = gameHandler->getNpcQuestStatuses();

    static int logCounter = 0;
    if (++logCounter % 300 == 0) {  // Log every ~10 seconds at 30fps
        LOG_DEBUG("Quest markers: ", questStatuses.size(), " NPCs with quest status");
    }

    // Clear all markers (we'll re-add active ones)
    questMarkerRenderer->clear();

    static bool firstRun = true;
    int markersAdded = 0;

    // Add markers for NPCs with quest status
    for (const auto& [guid, status] : questStatuses) {
        // Determine marker type
        int markerType = -1;  // -1 = no marker

        using game::QuestGiverStatus;
        switch (status) {
            case QuestGiverStatus::AVAILABLE:
            case QuestGiverStatus::AVAILABLE_LOW:
                markerType = 0;  // Available (yellow !)
                break;
            case QuestGiverStatus::REWARD:
            case QuestGiverStatus::REWARD_REP:
                markerType = 1;  // Turn-in (yellow ?)
                break;
            case QuestGiverStatus::INCOMPLETE:
                markerType = 2;  // Incomplete (grey ?)
                break;
            default:
                break;
        }

        if (markerType < 0) continue;

        // Get NPC entity position
        auto entity = gameHandler->getEntityManager().getEntity(guid);
        if (!entity) continue;
        if (entity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            std::string name = unit->getName();
            std::transform(name.begin(), name.end(), name.begin(),
                           [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
            if (name.find("spirit healer") != std::string::npos ||
                name.find("spirit guide") != std::string::npos) {
                continue; // Spirit healers/guides use their own white visual cue.
            }
        }

        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
        glm::vec3 renderPos = coords::canonicalToRender(canonical);

        // Get NPC bounding height for proper marker positioning
        glm::vec3 boundsCenter;
        float boundsRadius = 0.0f;
        float boundingHeight = 2.0f;  // Default
        if (getRenderBoundsForGuid(guid, boundsCenter, boundsRadius)) {
            boundingHeight = boundsRadius * 2.0f;
        }

        // Set the marker (renderer will handle positioning, bob, glow, etc.)
        questMarkerRenderer->setMarker(guid, renderPos, markerType, boundingHeight);
        markersAdded++;
    }

    if (firstRun && markersAdded > 0) {
        LOG_DEBUG("Quest markers: Added ", markersAdded, " markers on first run");
        firstRun = false;
    }
}

void Application::setupTestTransport() {
    if (testTransportSetup_) return;
    if (!gameHandler || !renderer || !assetManager) return;

    auto* transportManager = gameHandler->getTransportManager();
    auto* wmoRenderer = renderer->getWMORenderer();
    if (!transportManager || !wmoRenderer) return;

    LOG_INFO("========================================");
    LOG_INFO("   SETTING UP TEST TRANSPORT");
    LOG_INFO("========================================");

    // Connect transport manager to WMO renderer
    transportManager->setWMORenderer(wmoRenderer);

    // Connect WMORenderer to M2Renderer (for hierarchical transforms: doodads following WMO parents)
    if (renderer->getM2Renderer()) {
        wmoRenderer->setM2Renderer(renderer->getM2Renderer());
        LOG_INFO("WMORenderer connected to M2Renderer for test transport doodad transforms");
    }

    // Define a simple circular path around Stormwind harbor (canonical coordinates)
    // These coordinates are approximate - adjust based on actual harbor layout
    std::vector<glm::vec3> harborPath = {
        {-8833.0f, 628.0f, 94.0f},   // Start point (Stormwind harbor)
        {-8900.0f, 650.0f, 94.0f},   // Move west
        {-8950.0f, 700.0f, 94.0f},   // Northwest
        {-8950.0f, 780.0f, 94.0f},   // North
        {-8900.0f, 830.0f, 94.0f},   // Northeast
        {-8833.0f, 850.0f, 94.0f},   // East
        {-8766.0f, 830.0f, 94.0f},   // Southeast
        {-8716.0f, 780.0f, 94.0f},   // South
        {-8716.0f, 700.0f, 94.0f},   // Southwest
        {-8766.0f, 650.0f, 94.0f},   // Back to start direction
    };

    // Register the path with transport manager
    uint32_t pathId = 1;
    float speed = 12.0f;  // 12 units/sec (slower than taxi for a leisurely boat ride)
    transportManager->loadPathFromNodes(pathId, harborPath, true, speed);
    LOG_INFO("Registered transport path ", pathId, " with ", harborPath.size(), " waypoints, speed=", speed);

    // Try transport WMOs in manifest-backed paths first.
    std::vector<std::string> transportCandidates = {
        "World\\wmo\\transports\\transport_ship\\transportship.wmo",
        "World\\wmo\\transports\\transport_zeppelin\\transport_zeppelin.wmo",
        "World\\wmo\\transports\\transport_horde_zeppelin\\Transport_Horde_Zeppelin.wmo",
        "World\\wmo\\transports\\icebreaker\\Transport_Icebreaker_ship.wmo",
        // Legacy fallbacks
        "Transports\\Transportship\\Transportship.wmo",
        "Transports\\Boat\\Boat.wmo",
    };

    std::string transportWmoPath;
    std::vector<uint8_t> wmoData;
    for (const auto& candidate : transportCandidates) {
        wmoData = assetManager->readFile(candidate);
        if (!wmoData.empty()) {
            transportWmoPath = candidate;
            break;
        }
    }

    if (wmoData.empty()) {
        LOG_WARNING("No transport WMO found - test transport disabled");
        LOG_INFO("Expected under World\\wmo\\transports\\...");
        return;
    }

    LOG_INFO("Using transport WMO: ", transportWmoPath);

    // Load WMO model
    pipeline::WMOModel wmoModel = pipeline::WMOLoader::load(wmoData);
    LOG_INFO("Transport WMO root loaded: ", transportWmoPath, " nGroups=", wmoModel.nGroups);

    // Load WMO groups
    int loadedGroups = 0;
    if (wmoModel.nGroups > 0) {
        std::string basePath = transportWmoPath.substr(0, transportWmoPath.size() - 4);

        for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
            char groupSuffix[16];
            snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.wmo", gi);
            std::string groupPath = basePath + groupSuffix;
            std::vector<uint8_t> groupData = assetManager->readFile(groupPath);

            if (!groupData.empty()) {
                pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                loadedGroups++;
            } else {
                LOG_WARNING("  Failed to load WMO group ", gi, " for: ", basePath);
            }
        }
    }

    if (loadedGroups == 0 && wmoModel.nGroups > 0) {
        LOG_WARNING("Failed to load any WMO groups for transport");
        return;
    }

    // Load WMO into renderer
    uint32_t wmoModelId = 99999;  // Use high ID to avoid conflicts
    if (!wmoRenderer->loadModel(wmoModel, wmoModelId)) {
        LOG_WARNING("Failed to load transport WMO model into renderer");
        return;
    }

    // Create WMO instance at first waypoint (convert canonical to render coords)
    glm::vec3 startCanonical = harborPath[0];
    glm::vec3 startRender = core::coords::canonicalToRender(startCanonical);

    uint32_t wmoInstanceId = wmoRenderer->createInstance(wmoModelId, startRender,
                                                          glm::vec3(0.0f, 0.0f, 0.0f), 1.0f);

    if (wmoInstanceId == 0) {
        LOG_WARNING("Failed to create transport WMO instance");
        return;
    }

    // Register transport with transport manager
    uint64_t transportGuid = 0x1000000000000001ULL;  // Fake GUID for test
    transportManager->registerTransport(transportGuid, wmoInstanceId, pathId, startCanonical);

    // Optional: Set deck bounds (rough estimate for a ship deck)
    transportManager->setDeckBounds(transportGuid,
                                    glm::vec3(-15.0f, -30.0f, 0.0f),
                                    glm::vec3(15.0f, 30.0f, 10.0f));

    testTransportSetup_ = true;
    LOG_INFO("========================================");
    LOG_INFO("Test transport registered:");
    LOG_INFO("  GUID: 0x", std::hex, transportGuid, std::dec);
    LOG_INFO("  WMO Instance: ", wmoInstanceId);
    LOG_INFO("  Path: ", pathId, " (", harborPath.size(), " waypoints)");
    LOG_INFO("  Speed: ", speed, " units/sec");
    LOG_INFO("========================================");
    LOG_INFO("");
    LOG_INFO("To board the transport, use console command:");
    LOG_INFO("  /transport board");
    LOG_INFO("To disembark:");
    LOG_INFO("  /transport leave");
    LOG_INFO("========================================");
}

} // namespace core
} // namespace wowee
