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
    if (!envValue || !*envValue) {
        return &SPAWN_PRESETS[0];
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
            // Character creation update
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
        gameHandler->setSinglePlayerMode(true);
        uiManager->getCharacterCreateScreen().reset();
        setState(AppState::CHARACTER_CREATION);
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
        if (singlePlayerMode) {
            // Use created character's data for level/HP
            for (const auto& ch : gameHandler->getCharacters()) {
                if (ch.guid == characterGuid) {
                    uint32_t maxHp = 20 + static_cast<uint32_t>(ch.level) * 10;
                    gameHandler->initLocalPlayerStats(ch.level, maxHp, maxHp);
                    break;
                }
            }
            startSinglePlayer();
        } else {
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

    // "Create Character" button on character screen
    uiManager->getCharacterScreen().setOnCreateCharacter([this]() {
        uiManager->getCharacterCreateScreen().reset();
        setState(AppState::CHARACTER_CREATION);
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
        // Only init stats with defaults if not already set (e.g. via character creation)
        if (gameHandler->getLocalPlayerMaxHealth() == 0) {
            uint32_t level = 10;
            uint32_t maxHealth = 20 + level * 10;
            gameHandler->initLocalPlayerStats(level, maxHealth, maxHealth);
        }
    }

    // Create world object for single-player
    if (!world) {
        world = std::make_unique<game::World>();
        LOG_INFO("Single-player world created");
    }

    // Populate test inventory for single-player
    if (gameHandler) {
        gameHandler->getInventory().populateTestItems();
    }

    // Load weapon models for equipped items (after inventory is populated)
    loadEquippedWeapons();

    // --- Loading screen: load terrain and wait for streaming before spawning ---
    const SpawnPreset* spawnPreset = selectSpawnPreset(std::getenv("WOW_SPAWN"));
    // Canonical WoW coords: +X=North, +Y=West, +Z=Up
    glm::vec3 spawnCanonical = spawnPreset ? spawnPreset->spawnCanonical : glm::vec3(62.0f, -9464.0f, 200.0f);
    std::string mapName = spawnPreset ? spawnPreset->mapName : "Azeroth";
    float spawnYaw = spawnPreset ? spawnPreset->yawDeg : 0.0f;
    float spawnPitch = spawnPreset ? spawnPreset->pitchDeg : -5.0f;
    spawnSnapToGround = spawnPreset ? spawnPreset->snapToGround : true;

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

    // Set map name on terrain manager
    if (renderer && renderer->getTerrainManager()) {
        renderer->getTerrainManager()->setMapName(mapName);
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

} // namespace core
} // namespace wowee
