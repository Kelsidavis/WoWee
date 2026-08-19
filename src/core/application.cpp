#include "core/application.hpp"
#include "core/env_flag.hpp"
#include "core/character_paths.hpp"
#include "ui/settings_schema.hpp"
#include "pipeline/m2_asset_loader.hpp"
#include "core/coordinates.hpp"
#include "ui/minimap_projection.hpp"
#include "core/profiler.hpp"
#include "core/npc_interaction_callback_handler.hpp"
#include "core/audio_callback_handler.hpp"
#include "core/entity_spawn_callback_handler.hpp"
#include "core/animation_callback_handler.hpp"
#include "core/transport_callback_handler.hpp"
#include "core/world_entry_callback_handler.hpp"
#include "core/ui_screen_callback_handler.hpp"
#include "game/spell_classification.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "rendering/animation_controller.hpp"
#include <bit>
#include <unordered_set>
#include <cmath>
#include <chrono>
#include <limits>
#include <utility>
#include "core/logger.hpp"
#include "core/memory_monitor.hpp"
#include "rendering/renderer.hpp"
#include "rendering/vk_context.hpp"
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
#include "rendering/lighting_manager.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/keybinding_manager.hpp"
#include "rendering/quest_marker_renderer.hpp"
#include "rendering/footprint_renderer.hpp"
#include "rendering/loading_screen.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "addons/lua_api_registrations.hpp"
#include "audio/audio_coordinator.hpp"
#include "addons/addon_manager.hpp"
#include "addons/lua_api_helpers.hpp"
#include <imgui.h>
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/wmo_group_path.hpp"
#include "pipeline/wdt_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "ui/ui_manager.hpp"
#include "ui/touch_controls.hpp"
#include "ui/ui_services.hpp"
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "game/chat_handler.hpp"
#include "game/faction_hostility.hpp"
#include "game/transport_manager.hpp"
#include "game/world.hpp"
#include "game/expansion_profile.hpp"
#include "game/packet_parsers.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/spell_icon_paths.hpp"

#include <SDL2/SDL.h>
#include <cstdlib>
#include <climits>
#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>
#include <set>
#include <filesystem>
#include <fstream>

#include <thread>
#ifdef __linux__
#include <sched.h>
#include <pthread.h>
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace wowee {
namespace core {

namespace {

std::optional<float> movingEntityFloor(rendering::Renderer* renderer,
                                        const glm::vec3& renderPos,
                                        const std::optional<glm::vec3>& previousRenderPos) {
    if (!renderer) return std::nullopt;

    // Server movement Z is the reference surface.  In WMO overlap regions the
    // outdoor heightfield may be a roof many units above a tunnel/interior, so
    // choose the closest reachable floor instead of blindly preferring terrain.
    constexpr float kMaxStepUp = 1.5f;
    constexpr float kMaxGroundDrop = 3.0f;
    const float probeZ = renderPos.z + kMaxStepUp;
    std::optional<float> best;

    auto consider = [&](const std::optional<float>& floor) {
        if (!floor || *floor > probeZ || *floor < renderPos.z - kMaxGroundDrop) return;
        if (!best || std::abs(*floor - renderPos.z) < std::abs(*best - renderPos.z)) {
            best = floor;
        }
    };

    if (auto* terrain = renderer->getTerrainManager()) {
        consider(terrain->getHeightAt(renderPos.x, renderPos.y));
    }
    // Outdoor movers almost always match the terrain heightfield. Avoid the
    // expensive WMO/M2 collision walks in that common case. Tunnel, bridge,
    // and interior overlaps still use full arbitration because raw terrain is
    // not close to the server-provided Z there.
    if (best && std::abs(*best - renderPos.z) <= 0.35f) {
        return best;
    }
    if (auto* wmo = renderer->getWMORenderer()) {
        // Closest to the mover's own feet, so a creature or another player
        // under an overhang is not snapped up to the floor of the level above
        // them - the same flip that pulled the local player between levels.
        consider(wmo->getFloorHeight(renderPos.x, renderPos.y, probeZ, nullptr,
                                     renderPos.z));
    }
    if (auto* m2 = renderer->getM2Renderer()) {
        consider(m2->getFloorHeight(renderPos.x, renderPos.y, probeZ));
    }

    // A jump between floors, named once a second while it is happening.
    //
    // Reported as the character pulled between levels in Undercity - in
    // portals, doorways, overhangs - which is this arbitration flipping between
    // two floors that both sit inside the step-up/drop window near a level
    // transition. Which floor won, from which source, and how far it moved the
    // player is the thing a report cannot carry and this can. Warning level and
    // rate-limited, because the default log is warnings only and a per-frame
    // line would bury everything else.
    if (best && previousRenderPos &&
        std::abs(*best - previousRenderPos->z) > 1.0f) {
        static std::chrono::steady_clock::time_point lastFloorJumpLog{};
        const auto now = std::chrono::steady_clock::now();
        if (now - lastFloorJumpLog > std::chrono::seconds(1)) {
            lastFloorJumpLog = now;
            std::optional<float> terrainF, wmoF, m2F;
            if (auto* t = renderer->getTerrainManager())
                terrainF = t->getHeightAt(renderPos.x, renderPos.y);
            if (auto* w = renderer->getWMORenderer())
                wmoF = w->getFloorHeight(renderPos.x, renderPos.y, probeZ);
            if (auto* mm = renderer->getM2Renderer())
                m2F = mm->getFloorHeight(renderPos.x, renderPos.y, probeZ);
            LOG_WARNING("Floor jump: player z=", renderPos.z, " prev z=",
                        previousRenderPos->z, " -> chose ", *best,
                        " (terrain=", terrainF ? *terrainF : -99999.0f,
                        " wmo=", wmoF ? *wmoF : -99999.0f,
                        " m2=", m2F ? *m2F : -99999.0f,
                        ") - moved ", *best - previousRenderPos->z);
        }
    }

    // A broader floor candidate is useful on stairs and uneven terrain, but it is
    // ambiguous near overlapping shells or non-collidable authored props. Require
    // continuity with the last rendered ground position before accepting it. This
    // preserves server-authored waypoint height without creature-entry exceptions.
    if (best && std::abs(*best - renderPos.z) > 0.35f) {
        if (!previousRenderPos) return std::nullopt;
        const glm::vec2 planarDelta = glm::vec2(renderPos) - glm::vec2(*previousRenderPos);
        const float maxContinuousStep = 0.35f + glm::length(planarDelta) * 1.5f;
        if (std::abs(*best - previousRenderPos->z) > maxContinuousStep) {
            return std::nullopt;
        }
    }
    return best;
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

    // Initialize memory monitoring for dynamic cache sizing
    core::MemoryMonitor::getInstance().initialize();

    // Create window
    WindowConfig windowConfig;
    windowConfig.title = "Wowee";
    windowConfig.width = 1280;
    windowConfig.height = 720;
    // Pace rendering to the display by default. The old 240 FPS default kept
    // the main thread near a full core even while the scene was idle.
    windowConfig.vsync = true;

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

    // Create and initialize audio coordinator (owns all audio managers)
    audioCoordinator_ = std::make_unique<audio::AudioCoordinator>();
    if (!audioCoordinator_->initialize())
        LOG_WARNING("Audio coordinator initialization failed - game will run without audio");
    renderer->setAudioCoordinator(audioCoordinator_.get());

    // Create UI manager
    uiManager = std::make_unique<ui::UIManager>();
    if (!uiManager->initialize(window.get())) {
        LOG_FATAL("Failed to initialize UI manager");
        return false;
    }

    // The saved anti-aliasing setting, applied here rather than from the game
    // screen's update.
    //
    // GameScreen's constructor has just read settings.cfg, so the value is
    // known - but update() does not run until the game screen is up, which is
    // after character select. A saved setting therefore rebuilt the swapchain,
    // every render pass and every pipeline around fifteen hundred frames into
    // the session, with a world loaded, textures resident and uploads in
    // flight. Here the renderer has drawn nothing yet and there is nothing for
    // the rebuild to disturb.
    uiManager->getGameScreen().applySavedAntiAliasing(renderer.get());

    // Create subsystems
    authHandler = std::make_unique<auth::AuthHandler>();
    world = std::make_unique<game::World>();

    // Create and initialize expansion registry
    expansionRegistry_ = std::make_unique<game::ExpansionRegistry>();

    // Create DBC layout
    dbcLayout_ = std::make_unique<pipeline::DBCLayout>();

    // Create asset manager
    assetManager = std::make_unique<pipeline::AssetManager>();

    // Populate game services - all subsystems now available
    gameServices_.renderer = renderer.get();
    gameServices_.audioCoordinator = audioCoordinator_.get();
    gameServices_.assetManager = assetManager.get();
    gameServices_.expansionRegistry = expansionRegistry_.get();

    // Create game handler with explicit service dependencies
    gameHandler = std::make_unique<game::GameHandler>(gameServices_);

    // Try to get WoW data path from environment variable
    const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
    std::string dataPath = dataPathEnv ? dataPathEnv : "./Data";

    // Scan for available expansion profiles
    expansionRegistry_->initialize(dataPath);

    // Load the tables this expansion's protocol is described by.
    if (gameHandler && expansionRegistry_) {
        if (auto* profile = expansionRegistry_->getActive()) {
            loadExpansionTables(*profile);
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
                // Register base Data/ as fallback so world terrain files are found
                // even when the expansion path only contains DBC overrides.
                if (assetPath != dataPath) {
                    assetManager->setBaseFallbackPath(dataPath);
                }
            }
        }
    }

    LOG_INFO("Attempting to load WoW assets from: ", assetPath);
    if (assetManager->initialize(assetPath)) {
        LOG_INFO("Asset manager initialized successfully");

        // Fonts, now the archives are open and still before any frame is
        // drawn - the only moment the glyph atlas can take another face
        // without being rebuilt.
        //
        // It used to run above this, which meant it could only ever see loose
        // files. An install that never extracted its data keeps every font
        // inside the MPQs, so the search found nothing and the client fell
        // back to ImGui's built-in face - the same build reading the game's
        // own fonts on one machine and not on another. assetPath, not the raw
        // dataPath: extract_assets.sh always writes with --expansion-subdir,
        // so fonts live under expansions/<id>/ alongside everything else.
        // assetPath first, then dataPath: an extraction made with
        // --expansion-subdir keeps its fonts under expansions/<id>/, and one
        // made without keeps them in the base Data. Both exist in the wild.
        if (uiManager) {
            uiManager->loadInterfaceFont(assetPath, assetManager.get());
            uiManager->loadInterfaceFont(dataPath, assetManager.get());
        }

        // Renderer creation precedes AssetManager creation, so DBC-driven
        // lighting must be initialized here rather than in Renderer::initialize.
        if (renderer && renderer->getLightingManager() &&
            !renderer->getLightingManager()->initialize(assetManager.get())) {
            LOG_WARNING("Lighting manager initialization failed; using fallback lighting");
        }

        // Eagerly load creature display DBC lookups so first spawn doesn't stall
        entitySpawner_ = std::make_unique<EntitySpawner>(
            renderer.get(), assetManager.get(), gameHandler.get(),
            dbcLayout_.get(), &gameServices_);
        entitySpawner_->initialize();

        appearanceComposer_ = std::make_unique<AppearanceComposer>(
            renderer.get(), assetManager.get(), gameHandler.get(),
            entitySpawner_.get());

        // Wire AppearanceComposer to UI components (Phase A singleton breaking)
        if (uiManager) {
            uiManager->setAppearanceComposer(appearanceComposer_.get());
            
            // Wire all services to UI components (Phase B singleton breaking)
            ui::UIServices uiServices;
            uiServices.window = window.get();
            uiServices.renderer = renderer.get();
            uiServices.assetManager = assetManager.get();
            uiServices.gameHandler = gameHandler.get();
            uiServices.expansionRegistry = expansionRegistry_.get();
            uiServices.addonManager = addonManager_.get();  // May be nullptr here, re-wire later
            uiServices.audioCoordinator = audioCoordinator_.get();
            uiServices.entitySpawner = entitySpawner_.get();
            uiServices.appearanceComposer = appearanceComposer_.get();
            uiServices.worldLoader = worldLoader_.get();
            uiManager->setServices(uiServices);
        }

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

        // Initialize addon system
        addonManager_ = std::make_unique<addons::AddonManager>();
        // Bindings ask the interface whether someone is typing before they
        // fire. Set here rather than at the call sites because every panel
        // polls its own key from inside its own draw - there are many askers
        // and one answer.
        ui::setTypedInputProbe(
            [this]() -> bool {
                if (!addonManager_ || !addonsLoaded_) return false;
                auto* engine = addonManager_->getLuaEngine();
                return engine != nullptr && engine->editBoxHasFocus();
            });
        addons::LuaServices luaSvc;
        luaSvc.window            = window.get();
        luaSvc.audioCoordinator  = audioCoordinator_.get();
        luaSvc.expansionRegistry = expansionRegistry_.get();
        luaSvc.requestReloadUI = [this]() { reloadUiPending_ = true; };
        luaSvc.openSettings = [uim = uiManager.get()](const std::string& tab) {
            if (uim) uim->getGameScreen().openSettings(tab.empty() ? nullptr : tab.c_str());
        };
        // FrameXML's options panels, driving this client's own settings.
        //
        // One table rather than a hook per setting. Every row is a CVar the
        // Blizzard panels already have a control for, and a field this client
        // already had - they simply had never been introduced.
        luaSvc.getClientSetting = [uim = uiManager.get()](const std::string& key) {
            return uim ? uim->getGameScreen().getSettingsPanel().settingValue(key) : std::string{};
        };
        luaSvc.setCameraMaxDistanceFactor = [this](float factor) {
            if (!renderer) return;
            if (auto* cam = renderer->getCameraController()) cam->setMaxDistanceFactor(factor);
        };
        luaSvc.quitApplication = [this]() {
            LOG_INFO("Exit Game with no session to leave - closing");
            if (window) window->setShouldClose(true);
        };
        luaSvc.setActionBarPage = [uim = uiManager.get()](int page) {
            if (uim) uim->getGameScreen().actionBarPanel().setMainActionBarPage(page);
        };
        luaSvc.setZoneMusicLooping = [this](bool loop) {
            if (audioCoordinator_) audioCoordinator_->setZoneMusicLooping(loop);
        };
        luaSvc.setGroundDetailDistance = [this](float yards) {
            if (!renderer) return;
            if (auto* m2 = renderer->getM2Renderer()) m2->setGroundDetailDistance(yards);
        };
        luaSvc.setAnisotropyLimit = [this](float limit) {
            if (auto* window = this->window.get()) {
                if (auto* ctx = window->getVkContext()) ctx->setAnisotropyLimit(limit);
            }
        };
        luaSvc.setEnvironmentDetail = [this](float detail) {
            if (!renderer) return;
            if (auto* m2 = renderer->getM2Renderer()) m2->setEnvironmentDetail(detail);
        };
        luaSvc.setParticleDensity = [this](float density) {
            if (!renderer) return;
            if (auto* m2 = renderer->getM2Renderer()) m2->setParticleDensity(density);
        };
        luaSvc.setWeatherDensity = [this](float scale) {
            if (!renderer) return;
            if (auto* weather = renderer->getWeather()) weather->setDensityScale(scale);
        };
        luaSvc.setClientSetting = [uim = uiManager.get()](const std::string& key,
                                                          const std::string& value) {
            if (!uim) return;
            auto& gs = uim->getGameScreen();
            if (gs.getSettingsPanel().setSettingValue(key, value)) gs.saveSettings();
        };

        // Every schema key must be answered, or its control is drawn and inert.
        //
        // The schema names the settings, the panel answers them, and the two are
        // in different files - so a setting added to one and not the other gives
        // a checkbox that does nothing, with nothing to say why. Checked once at
        // startup rather than left to be noticed.
        {
            std::size_t count = 0;
            const auto* schema = ui::clientSettingsSchema(count);
            for (std::size_t i = 0; i < count; ++i) {
                if (luaSvc.getClientSetting(schema[i].key).empty()) {
                    LOG_WARNING("Setting '", schema[i].key, "' (", schema[i].label,
                                ") is in the schema but nothing answers it - its "
                                "control in the interface options will do nothing");
                }
            }
        }

        // FrameXML's Sound options, driving this client's own audio settings.
        // Its sliders are 0..1 and these are percentages, so the conversion
        // happens here rather than in the binding.
        luaSvc.getAudioSetting = [uim = uiManager.get()](const std::string& key) -> float {
            if (!uim) return 0.0f;
            auto& sp = uim->getGameScreen().getSettingsPanel();
            if (key == "master")  return static_cast<float>(sp.pendingMasterVolume) / 100.0f;
            if (key == "music")   return static_cast<float>(sp.pendingMusicVolume) / 100.0f;
            if (key == "ambient") return static_cast<float>(sp.pendingAmbientVolume) / 100.0f;
            if (key == "enableall") return sp.soundMuted_ ? 0.0f : 1.0f;
            return 0.0f;
        };
        luaSvc.setAudioSetting = [uim = uiManager.get(), this](const std::string& key, float v) {
            if (!uim) return;
            auto& gs = uim->getGameScreen();
            auto& sp = gs.getSettingsPanel();
            const int pct = static_cast<int>(std::lround(v * 100.0f));
            if (key == "master") {
                sp.pendingMasterVolume = pct;
                // Raising it means sound is wanted, the same as the slider in
                // the client's own panel does.
                if (pct > 0) sp.soundMuted_ = false;
            } else if (key == "music")   { sp.pendingMusicVolume = pct;
            } else if (key == "ambient") { sp.pendingAmbientVolume = pct;
            } else if (key == "enableall") { sp.soundMuted_ = (v <= 0.0f);
            } else { return; }
            sp.applyAudioVolumes(audioCoordinator_.get());
            gs.saveSettings();
        };
        luaSvc.reapplyAudioVolumes = [uim = uiManager.get(), this]() {
            if (!uim) return;
            uim->getGameScreen().getSettingsPanel().applyAudioVolumes(audioCoordinator_.get());
        };
        luaSvc.runMacroText = [uim = uiManager.get(), gh = gameHandler.get()](const std::string& body) {
            if (uim && gh) uim->getGameScreen().getChatPanel().executeMacroText(*gh, body);
        };
        luaSvc.clientChatCommandNames = [uim = uiManager.get()]() {
            std::vector<std::string> out;
            if (uim) out = uim->getGameScreen().getChatPanel().registryCommandNames();
            return out;
        };
        luaSvc.runClientChatCommand = [uim = uiManager.get(), gh = gameHandler.get()](
                const std::string& alias, const std::string& args) -> bool {
            if (!uim || !gh) return false;
            return uim->getGameScreen().getChatPanel().runRegistryCommand(*gh, alias, args);
        };
        luaSvc.playEmoteAnimation = [rend = renderer.get()](const std::string& name) {
            if (!rend) return;
            if (auto* ac = rend->getAnimationController()) ac->playEmote(name);
        };
        luaSvc.getGamma = [uim = uiManager.get()]() -> float {
            return uim ? uim->getGameScreen().getGamma() : 1.0f;
        };
        luaSvc.setGamma = [uim = uiManager.get()](float g) {
            if (uim) uim->getGameScreen().setGamma(g);
        };
        // The world map's own data, for the interface's map. The client keeps
        // the zones, their overlays and the area POIs and drew them only
        // itself; FrameXML's map has readers for exactly these and had nothing
        // behind them.
        luaSvc.getMapOverlays = [r = renderer.get()]() {
            std::vector<addons::LuaServices::MapOverlay> out;
            auto* wmap = r ? r->getWorldMap() : nullptr;
            if (!wmap) return out;
            for (const auto& o : wmap->currentOverlays()) {
                addons::LuaServices::MapOverlay m;
                m.texture = o.textureName;
                m.width = o.texWidth;
                m.height = o.texHeight;
                m.offsetX = o.offsetX;
                m.offsetY = o.offsetY;
                out.push_back(std::move(m));
            }
            return out;
        };
        luaSvc.getMapLandmarks = [r = renderer.get()]() {
            std::vector<addons::LuaServices::MapLandmark> out;
            auto* wmap = r ? r->getWorldMap() : nullptr;
            if (!wmap) return out;
            for (const auto& l : wmap->currentLandmarks()) {
                addons::LuaServices::MapLandmark m;
                m.name = l.name;
                m.description = l.description;
                m.icon = static_cast<int>(l.iconType);
                m.x = l.x;
                m.y = l.y;
                out.push_back(std::move(m));
            }
            return out;
        };
        luaSvc.getMapZoneNameAt = [r = renderer.get()](float u, float v) -> std::string {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->zoneNameAtMapPoint(u, v) : std::string();
        };
        luaSvc.clickMapPoint = [r = renderer.get()](float u, float v) -> bool {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->clickMapPoint(u, v) : false;
        };
        luaSvc.mapUVForWorldPos = [r = renderer.get()](float x, float y, float z,
                                                      float& u, float& v) -> bool {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->mapUVForCanonical(x, y, z, u, v) : false;
        };
        luaSvc.zoomMapOut = [r = renderer.get()]() {
            if (auto* wmap = r ? r->getWorldMap() : nullptr) wmap->zoomOutOneLevel();
        };
        luaSvc.getMapContinentNames = [r = renderer.get()]() -> std::vector<std::string> {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->continentNames() : std::vector<std::string>();
        };
        luaSvc.getMapZoneNames = [r = renderer.get()](int continent) -> std::vector<std::string> {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->zoneNames(continent) : std::vector<std::string>();
        };
        luaSvc.setMapByIndex = [r = renderer.get()](int continent, int zone) {
            if (auto* wmap = r ? r->getWorldMap() : nullptr) wmap->showMap(continent, zone);
        };
        luaSvc.getMapContinentIndex = [r = renderer.get()]() -> int {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->currentContinentIndex() : 0;
        };
        luaSvc.getMapZoneIndex = [r = renderer.get()]() -> int {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->currentZoneIndex() : 0;
        };
        luaSvc.canZoomMapOut = [r = renderer.get()]() -> bool {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap && wmap->canZoomOut();
        };
        luaSvc.showPlayerMapZone = [r = renderer.get()]() {
            if (auto* wmap = r ? r->getWorldMap() : nullptr) wmap->showPlayerZone();
        };
        luaSvc.getMapWorldAreaId = [r = renderer.get()]() -> uint32_t {
            auto* wmap = r ? r->getWorldMap() : nullptr;
            return wmap ? wmap->currentWorldMapAreaId() : 0u;
        };
        luaSvc.setMapWorldAreaId = [r = renderer.get()](uint32_t id) {
            if (auto* wmap = r ? r->getWorldMap() : nullptr) wmap->showWorldMapArea(id);
        };
        luaSvc.getLiveZoneId = [r = renderer.get()]() -> uint32_t {
            return r ? r->getCurrentZoneId() : 0u;
        };
        luaSvc.isOnOutdoorPvpObjective = [r = renderer.get()]() -> bool {
            return r && r->isOnOutdoorPvpObjective();
        };
        luaSvc.takeScreenshot = [uim = uiManager.get()]() {
            if (uim) uim->getGameScreen().takeScreenshot();
        };
        luaSvc.getNameplatesShown = [uim = uiManager.get()]() -> bool {
            return uim ? uim->getGameScreen().getShowNameplates() : true;
        };
        luaSvc.setNameplatesShown = [uim = uiManager.get()](bool shown) {
            if (uim) uim->getGameScreen().setShowNameplates(shown);
        };
        luaSvc.getBarberStyleInfo = [uim = uiManager.get(), gh = gameHandler.get()](
                int selector, std::string& name, bool& isCurrent) -> bool {
            if (!uim || !gh) return false;
            return uim->getGameScreen().windowManager()
                      .barberStyleInfo(*gh, selector, name, isCurrent);
        };
        luaSvc.setNextBarberStyle = [uim = uiManager.get(), gh = gameHandler.get()](
                int selector, int direction) {
            if (uim && gh) {
                uim->getGameScreen().windowManager().barberCycleStyle(*gh, selector, direction);
            }
        };
        luaSvc.getBarberTotalCost = [uim = uiManager.get(), gh = gameHandler.get()]() -> uint32_t {
            if (!uim || !gh) return 0;
            return uim->getGameScreen().windowManager().barberTotalCostCopper(*gh);
        };
        luaSvc.barberReset = [uim = uiManager.get(), gh = gameHandler.get()]() {
            if (uim && gh) uim->getGameScreen().windowManager().barberResetSelections(*gh);
        };
        luaSvc.barberApply = [uim = uiManager.get(), gh = gameHandler.get()]() {
            if (uim && gh) uim->getGameScreen().windowManager().barberApplySelection(*gh);
        };
        luaSvc.isPlayerIndoors = [r = renderer.get()]() -> bool {
            return r && r->isPlayerIndoors();
        };
        luaSvc.getMinimapRotate = [r = renderer.get()]() -> bool {
            auto* mm = r ? r->getMinimap() : nullptr;
            return mm && mm->isRotateWithCamera();
        };
        luaSvc.setMinimapRotate = [r = renderer.get()](bool rotate) {
            if (auto* mm = r ? r->getMinimap() : nullptr) mm->setRotateWithCamera(rotate);
        };
        luaSvc.getChatBubblesShown = [uim = uiManager.get()]() -> bool {
            return uim ? uim->getGameScreen().getChatPanel().bubblesShown() : true;
        };
        luaSvc.setChatBubblesShown = [uim = uiManager.get()](bool shown) {
            if (uim) uim->getGameScreen().getChatPanel().setBubblesShown(shown);
        };
        luaSvc.getFullscreen = [uim = uiManager.get()]() -> bool {
            return uim ? uim->getGameScreen().getFullscreen() : false;
        };
        luaSvc.setFullscreen = [uim = uiManager.get()](bool on) {
            if (uim) uim->getGameScreen().setFullscreen(on);
        };
        luaSvc.getResolutionIndex = [uim = uiManager.get()]() -> int {
            return uim ? uim->getGameScreen().getResolutionIndex() : 0;
        };
        luaSvc.getAntiAliasingIndex = [uim = uiManager.get()]() -> int {
            return uim ? uim->getGameScreen().getAntiAliasingIndex() : 0;
        };
        luaSvc.setAntiAliasingIndex = [uim = uiManager.get()](int i) {
            if (uim) uim->getGameScreen().setAntiAliasingIndex(i);
        };
        luaSvc.setResolutionIndex = [uim = uiManager.get()](int i) {
            if (uim) uim->getGameScreen().setResolutionIndex(i);
        };
        // Gathered once, on the first ask rather than at startup: it is a walk
        // of the whole manifest, and most sessions never open an icon picker.
        luaSvc.listIconTextures = [am = assetManager.get()]() -> const std::vector<std::string>& {
            static std::vector<std::string> icons;
            static bool built = false;
            if (built) return icons;
            built = true;
            if (!am) return icons;
            // Manifest keys are normalised: lower case, backslashes.
            static const std::string kPrefix = "interface\\icons\\";
            for (const auto& [path, entry] : am->getManifest().getEntries()) {
                (void)entry;
                if (path.compare(0, kPrefix.size(), kPrefix) != 0) continue;
                // SetTexture wants the path without the extension, which is
                // also how every icon is named everywhere else in the
                // interface.
                const size_t dot = path.rfind('.');
                const std::string name = path.substr(
                    kPrefix.size(),
                    dot == std::string::npos ? std::string::npos : dot - kPrefix.size());
                if (!name.empty()) icons.push_back("Interface\\Icons\\" + name);
            }
            // A grid the player scrolls through, so the order has to be stable
            // between one frame and the next - the manifest is a hash map and
            // is not.
            std::sort(icons.begin(), icons.end());
            LOG_INFO("Icon picker: ", icons.size(), " icons from the manifest");
            return icons;
        };
        // The widget renderer needs the asset manager for Interface\ art and the
        // device to upload it; both exist by now.
        widgetRenderer_.initialize(assetManager.get(),
                                   window ? window->getVkContext() : nullptr);
        if (addonManager_->initialize(gameHandler.get(), luaSvc)) {
            // FrameXML and the AddOns folder are loose directories on disk, not
            // assets reached through the manifest, and they belong to the
            // installation rather than to whichever expansion happens to supply
            // asset overrides. assetPath is the expansion's directory whenever
            // one carries its own manifest - and an overlay holding models and
            // a DBC has no interface/ at all, so taking it here meant the whole
            // Blizzard interface stopped loading the moment any overlay
            // existed, with one warning to say so.
            //
            // The expansion still wins if it does ship an interface, which is
            // what an expansion-specific FrameXML would be for.
            std::string interfaceRoot = dataPath;
            if (assetPath != dataPath) {
                std::error_code ec;
                for (const char* cased : {"/interface", "/Interface"}) {
                    if (std::filesystem::is_directory(assetPath + cased, ec)) {
                        interfaceRoot = assetPath;
                        break;
                    }
                }
            }
            std::string addonsDir = interfaceRoot + "/interface/AddOns";
            addonManager_->setFrameXmlDir(interfaceRoot + "/interface/FrameXML");
            addonManager_->scanAddons(addonsDir);
            // Wire Lua errors to UI error display
            addonManager_->getLuaEngine()->setLuaErrorCallback([gh = gameHandler.get()](const std::string& err) {
                // Not addUIError: that reports by firing an event, which runs
                // script, which is how a script error came to report itself.
                if (gh) gh->addScriptError(err);
            });
            // The game-menu button opens this client's settings panel.
            addonManager_->getLuaEngine()->setOpenSettingsCallback([this] {
                if (auto* uim = uiManager.get()) uim->getGameScreen().openSettings();
            });
            // No second announcement of a chat message.
            //
            // This registered a callback that fired CHAT_MSG_* itself, off its
            // own hand-written table of chat types, with two arguments -
            // message and sender. The chat handler already announces every
            // message with the twelve the interface reads, so each line went out
            // twice: once complete, and once so short that
            // ChatFrame_MessageEventHandler raised on strlen(arg4) before it
            // drew anything. That raise is the one that filled the log.
            //
            // The table was the other half of it: fifty chat types have names
            // and this knew thirty-two, so the eighteen it had never heard of -
            // money, experience, honour, reputation, the channel notices -
            // produced no event here at all.
            //
            // What it did better, the channel's index, moved into the handler's
            // own firing.
            // Wire generic game events to addon dispatch
            gameHandler->setAddonEventCallback([this](const std::string& event, const std::vector<std::string>& args) {
                if (addonManager_ && addonsLoaded_) {
                    addonManager_->fireEvent(event, args);
                }
            });
            // How a keybinding reaches the frame that replaced this client's own
            // window. Without it, a key toggled a window nothing draws.
            gameHandler->setInterfaceCommandCallback([this](const std::string& lua) {
                if (addonManager_ && addonsLoaded_) {
                    addonManager_->runInterfaceCommand(lua);
                }
            });
            // The same route, for a question rather than an instruction.
            gameHandler->setInterfaceQueryCallback([this](const std::string& expr) {
                if (!addonManager_ || !addonsLoaded_) return false;
                return addonManager_->interfaceCommandBoolean(expr);
            });
            // Wire spell icon path resolver for Lua API (GetSpellInfo, UnitBuff icon, etc.)
            {
                auto spellIconPaths  = std::make_shared<std::unordered_map<uint32_t, std::string>>();
                auto spellIconIds    = std::make_shared<std::unordered_map<uint32_t, uint32_t>>();
                auto loaded          = std::make_shared<bool>(false);
                auto* am = assetManager.get();
                // Shared, because two resolvers now read these maps and either
                // may be asked first. Left inside the spell resolver, a
                // spellbook tab asking for an icon id before any spell had
                // asked for one would have found the table empty and answered
                // nothing, which is the sort of thing that only shows up when
                // the tabs happen to be built first.
                auto ensureLoaded = std::make_shared<std::function<void()>>(
                    [spellIconPaths, spellIconIds, loaded, am]() {
                    if (!am || !am->isInitialized() || *loaded) return;
                    // Initialised, not merely present: loadDBC answers nullptr
                    // before the assets are up, and latching on that left both
                    // tables empty for the rest of the session.
                    *loaded = true;
                    pipeline::loadSpellIconPaths(am, *spellIconPaths);
                    auto spellDbc = am->loadDBC("Spell.dbc");
                    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
                    if (spellDbc && spellDbc->isLoaded()) {
                        uint32_t fieldCount = spellDbc->getFieldCount();
                        uint32_t iconField = 133; // WotLK default
                        uint32_t idField = 0;
                        if (spellL) {
                            try {
                                uint32_t layoutId = (*spellL)["ID"];
                                uint32_t layoutIcon = (*spellL)["IconID"];
                                if (layoutId < fieldCount && layoutIcon < fieldCount) {
                                    iconField = layoutIcon;
                                    idField = layoutId;
                                }
                            } catch (...) {}
                        }
                        for (uint32_t i = 0; i < spellDbc->getRecordCount(); i++) {
                            uint32_t id = spellDbc->getUInt32(i, idField);
                            uint32_t iconId = spellDbc->getUInt32(i, iconField);
                            if (id > 0 && iconId > 0) (*spellIconIds)[id] = iconId;
                        }
                    }
                });

                gameHandler->setSpellIconPathResolver(
                    [spellIconPaths, spellIconIds, ensureLoaded](uint32_t spellId) -> std::string {
                    (*ensureLoaded)();
                    auto iit = spellIconIds->find(spellId);
                    if (iit == spellIconIds->end()) return {};
                    auto pit = spellIconPaths->find(iit->second);
                    if (pit == spellIconPaths->end()) return {};
                    return pit->second;
                });

                // Straight to the artwork, for callers that already hold an
                // icon id rather than a spell - SkillLine.dbc names one for
                // each spellbook tab.
                gameHandler->setIconPathResolver(
                    [spellIconPaths, ensureLoaded](uint32_t iconId) -> std::string {
                    if (iconId == 0) return {};
                    (*ensureLoaded)();
                    auto pit = spellIconPaths->find(iconId);
                    return pit == spellIconPaths->end() ? std::string{} : pit->second;
                });
            }
            // Wire item icon path resolver: displayInfoId -> "Interface\\Icons\\INV_..."
            {
                auto iconNames = std::make_shared<std::unordered_map<uint32_t, std::string>>();
                auto loaded    = std::make_shared<bool>(false);
                auto* am = assetManager.get();
                gameHandler->setItemIconPathResolver([iconNames, loaded, am](uint32_t displayInfoId) -> std::string {
                    if (!am || displayInfoId == 0) return {};
                    if (!*loaded && am->isInitialized()) {
                        // Initialised, not merely present: loadDBC answers
                        // nullptr before the assets are up, and latching on
                        // that left this table empty for the session.
                        *loaded = true;
                        auto dbc = am->loadDBC("ItemDisplayInfo.dbc");
                        const auto* dispL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                        if (dbc && dbc->isLoaded()) {
                            uint32_t iconField = dispL ? (*dispL)["InventoryIcon"] : 5;
                            for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
                                uint32_t id = dbc->getUInt32(i, 0); // field 0 = ID
                                std::string name = dbc->getString(i, iconField);
                                if (id > 0 && !name.empty()) (*iconNames)[id] = name;
                            }
                            LOG_INFO("Loaded ", iconNames->size(), " item icon names from ItemDisplayInfo.dbc");
                        }
                    }
                    auto it = iconNames->find(displayInfoId);
                    if (it == iconNames->end()) return {};
                    return "Interface\\Icons\\" + it->second;
                });
            }
            // Wire spell data resolver: spellId -> {castTimeMs, minRange, maxRange}
            {
                auto castTimeMap = std::make_shared<std::unordered_map<uint32_t, uint32_t>>();
                auto rangeMap    = std::make_shared<std::unordered_map<uint32_t, std::pair<float,float>>>();
                auto spellCastIdx = std::make_shared<std::unordered_map<uint32_t, uint32_t>>(); // spellId→castTimeIdx
                auto spellRangeIdx = std::make_shared<std::unordered_map<uint32_t, uint32_t>>(); // spellId→rangeIdx
                struct SpellCostEntry { uint32_t manaCost = 0; uint8_t powerType = 0; };
                auto spellCostMap = std::make_shared<std::unordered_map<uint32_t, SpellCostEntry>>();
                auto loaded = std::make_shared<bool>(false);
                auto* am = assetManager.get();
                gameHandler->setSpellDataResolver([castTimeMap, rangeMap, spellCastIdx, spellRangeIdx, spellCostMap, loaded, am](uint32_t spellId) -> game::GameHandler::SpellDataInfo {
                    if (!am) return {};
                    if (!*loaded && am->isInitialized()) {
                        // Initialised, not merely present: loadDBC answers
                        // nullptr before the assets are up, and latching on
                        // that left this table empty for the session.
                        *loaded = true;
                        // Load SpellCastTimes.dbc
                        auto ctDbc = am->loadDBC("SpellCastTimes.dbc");
                        if (ctDbc && ctDbc->isLoaded()) {
                            for (uint32_t i = 0; i < ctDbc->getRecordCount(); ++i) {
                                uint32_t id = ctDbc->getUInt32(i, 0);
                                int32_t base = static_cast<int32_t>(ctDbc->getUInt32(i, 1));
                                if (id > 0 && base > 0) (*castTimeMap)[id] = static_cast<uint32_t>(base);
                            }
                        }
                        // Load SpellRange.dbc
                        const auto* srL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SpellRange") : nullptr;
                        uint32_t minRField = srL ? (*srL)["MinRange"] : 1;
                        uint32_t maxRField = srL ? (*srL)["MaxRange"] : 4;
                        auto rDbc = am->loadDBC("SpellRange.dbc");
                        if (rDbc && rDbc->isLoaded()) {
                            for (uint32_t i = 0; i < rDbc->getRecordCount(); ++i) {
                                uint32_t id = rDbc->getUInt32(i, 0);
                                float minR = rDbc->getFloat(i, minRField);
                                float maxR = rDbc->getFloat(i, maxRField);
                                if (id > 0) (*rangeMap)[id] = {minR, maxR};
                            }
                        }
                        // Load Spell.dbc: extract castTimeIndex and rangeIndex per spell
                        auto sDbc = am->loadDBC("Spell.dbc");
                        const auto* spL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
                        if (sDbc && sDbc->isLoaded()) {
                            uint32_t idF = spL ? (*spL)["ID"] : 0;
                            // From the file's shape - Classic and Turtle named
                            // this 15, which is RequiresSpellFocus and zero for
                            // every spell.
                            uint32_t ctF = pipeline::detectSpellTimingFields(sDbc.get(), spL)
                                               .castingTimeIndex;
                            uint32_t rF  = spL ? (*spL)["RangeIndex"] : 132;
                            uint32_t ptF = UINT32_MAX, mcF = UINT32_MAX;
                            if (spL) {
                                try { ptF = (*spL)["PowerType"]; } catch (...) {}
                                try { mcF = (*spL)["ManaCost"]; } catch (...) {}
                            }
                            uint32_t fc = sDbc->getFieldCount();
                            for (uint32_t i = 0; i < sDbc->getRecordCount(); ++i) {
                                uint32_t id = sDbc->getUInt32(i, idF);
                                if (id == 0) continue;
                                uint32_t ct = sDbc->getUInt32(i, ctF);
                                uint32_t ri = sDbc->getUInt32(i, rF);
                                if (ct > 0) (*spellCastIdx)[id] = ct;
                                if (ri > 0) (*spellRangeIdx)[id] = ri;
                                // Extract power cost
                                uint32_t mc = (mcF < fc) ? sDbc->getUInt32(i, mcF) : 0;
                                uint8_t  pt = (ptF < fc) ? static_cast<uint8_t>(sDbc->getUInt32(i, ptF)) : 0;
                                if (mc > 0) (*spellCostMap)[id] = {.manaCost = mc, .powerType = pt};
                            }
                        }
                        LOG_INFO("SpellDataResolver: loaded ", spellCastIdx->size(), " cast indices, ",
                                 spellRangeIdx->size(), " range indices");
                    }
                    game::GameHandler::SpellDataInfo info;
                    auto ciIt = spellCastIdx->find(spellId);
                    if (ciIt != spellCastIdx->end()) {
                        auto ctIt = castTimeMap->find(ciIt->second);
                        if (ctIt != castTimeMap->end()) info.castTimeMs = ctIt->second;
                    }
                    auto riIt = spellRangeIdx->find(spellId);
                    if (riIt != spellRangeIdx->end()) {
                        auto rIt = rangeMap->find(riIt->second);
                        if (rIt != rangeMap->end()) {
                            info.minRange = rIt->second.first;
                            info.maxRange = rIt->second.second;
                        }
                    }
                    auto mcIt = spellCostMap->find(spellId);
                    if (mcIt != spellCostMap->end()) {
                        info.manaCost = mcIt->second.manaCost;
                        info.powerType = mcIt->second.powerType;
                    }
                    return info;
                });
            }
            // Wire random property/suffix name resolver for item display
            {
                auto propNames   = std::make_shared<std::unordered_map<int32_t, std::string>>();
                auto propLoaded  = std::make_shared<bool>(false);
                auto* amPtr = assetManager.get();
                gameHandler->setRandomPropertyNameResolver([propNames, propLoaded, amPtr](int32_t id) -> std::string {
                    if (!amPtr || id == 0) return {};
                    // Initialised, not merely present: loadDBC answers nullptr
                    // before the assets are up, and latching on that left the
                    // suffix names empty for the rest of the session.
                    if (!*propLoaded && amPtr->isInitialized()) {
                        *propLoaded = true;
                        // Both DBCs carry the display name ("of the Bear" / "of Strength") as
                        // the first string column, field 1, across classic/tbc/wotlk/turtle.
                        constexpr uint32_t kNameField = 1;
                        // ItemRandomProperties.dbc: ID=0, Name=1 (positive IDs)
                        if (auto dbc = amPtr->loadDBC("ItemRandomProperties.dbc"); dbc && dbc->isLoaded()) {
                            for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                                int32_t rid = static_cast<int32_t>(dbc->getUInt32(r, 0));
                                std::string name = dbc->getString(r, kNameField);
                                if (!name.empty() && rid > 0) (*propNames)[rid] = name;
                            }
                        }
                        // ItemRandomSuffix.dbc: ID=0, Name=1 - keyed as negative IDs
                        if (auto dbc = amPtr->loadDBC("ItemRandomSuffix.dbc"); dbc && dbc->isLoaded()) {
                            for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                                int32_t rid = static_cast<int32_t>(dbc->getUInt32(r, 0));
                                std::string name = dbc->getString(r, kNameField);
                                if (!name.empty() && rid > 0) (*propNames)[-rid] = name;
                            }
                        }
                    }
                    auto it = propNames->find(id);
                    return (it != propNames->end()) ? it->second : std::string{};
                });
            }
            // Wire random-suffix/property stat resolver. Reproduces the server's roll from the
            // same DBCs: a suffix (negative id) scales each SpellItemEnchantment STAT amount by
            // AllocationPct*suffixFactor/10000; a property (positive id) uses the enchant's fixed
            // amount. Caches all three tables on first use.
            {
                struct EnchEffect { uint32_t type; uint32_t arg; int32_t minAmount; };
                auto suffixMap = std::make_shared<std::unordered_map<int32_t, std::vector<std::pair<uint32_t,uint32_t>>>>();
                auto propMap   = std::make_shared<std::unordered_map<int32_t, std::vector<uint32_t>>>();
                auto enchMap   = std::make_shared<std::unordered_map<uint32_t, std::vector<EnchEffect>>>();
                auto loaded    = std::make_shared<bool>(false);
                auto* amPtr = assetManager.get();
                gameHandler->setRandomStatResolver(
                    [suffixMap, propMap, enchMap, loaded, amPtr](int32_t id, uint32_t suffixFactor)
                        -> std::vector<game::GameHandler::RandomStatBonus> {
                    std::vector<game::GameHandler::RandomStatBonus> out;
                    if (!amPtr || id == 0) return out;
                    if (!*loaded && amPtr->isInitialized()) {
                        // Initialised, not merely present: loadDBC answers
                        // nullptr before the assets are up, and latching on
                        // that left this table empty for the session.
                        *loaded = true;
                        // SpellItemEnchantment: enchId -> up to 3 (type, statArg, minAmount).
                        // Arg (stat type) is the 3 fields before Name; the effect-type array
                        // precedes the amount block(s) - 2 blocks (Min+Max) on TBC/WotLK, 1 on Vanilla.
                        if (auto dbc = amPtr->loadDBC("SpellItemEnchantment.dbc"); dbc && dbc->isLoaded()) {
                            const auto* sieL = pipeline::getActiveDBCLayout()
                                ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment") : nullptr;
                            const uint32_t fc = dbc->getFieldCount();
                            const uint32_t nameF = pipeline::detectEnchantmentNameField(dbc.get(), sieL);
                            if (nameF >= 12 && nameF < fc) {
                                const uint32_t argBase = nameF - 3;
                                const bool singleAmount = fc < 34;  // Vanilla/Turtle: no separate Max array
                                const uint32_t effBase = argBase - (singleAmount ? 6u : 9u);
                                const uint32_t minBase = effBase + 3u;
                                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                                    uint32_t enchId = dbc->getUInt32(r, 0);
                                    if (enchId == 0) continue;
                                    std::vector<EnchEffect> effs;
                                    for (uint32_t s = 0; s < 3; ++s) {
                                        uint32_t type = dbc->getUInt32(r, effBase + s);
                                        if (type == 0) continue;
                                        effs.push_back({.type = type,
                                                        .arg = dbc->getUInt32(r, argBase + s),
                                                        .minAmount = dbc->getInt32(r, minBase + s)});
                                    }
                                    if (!effs.empty()) (*enchMap)[enchId] = std::move(effs);
                                }
                            }
                        }
                        // ItemRandomSuffix: enchant array at field 19, AllocationPct follows; N=(fc-19)/2.
                        if (auto dbc = amPtr->loadDBC("ItemRandomSuffix.dbc"); dbc && dbc->isLoaded()) {
                            const uint32_t fc = dbc->getFieldCount();
                            if (fc > 21 && (fc - 19u) % 2u == 0u) {
                                const uint32_t n = (fc - 19u) / 2u;
                                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                                    int32_t sid = static_cast<int32_t>(dbc->getUInt32(r, 0));
                                    if (sid <= 0) continue;
                                    std::vector<std::pair<uint32_t,uint32_t>> ens;
                                    for (uint32_t k = 0; k < n; ++k) {
                                        uint32_t ench = dbc->getUInt32(r, 19u + k);
                                        uint32_t pct  = dbc->getUInt32(r, 19u + n + k);
                                        if (ench != 0) ens.emplace_back(ench, pct);
                                    }
                                    if (!ens.empty()) (*suffixMap)[sid] = std::move(ens);
                                }
                            }
                        }
                        // ItemRandomProperties: up to 5 enchant ids at fields 2..6 (fixed amount).
                        if (auto dbc = amPtr->loadDBC("ItemRandomProperties.dbc"); dbc && dbc->isLoaded()) {
                            if (dbc->getFieldCount() > 6) {
                                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                                    int32_t pid = static_cast<int32_t>(dbc->getUInt32(r, 0));
                                    if (pid <= 0) continue;
                                    std::vector<uint32_t> ens;
                                    for (uint32_t k = 2; k <= 6; ++k) {
                                        uint32_t ench = dbc->getUInt32(r, k);
                                        if (ench != 0) ens.push_back(ench);
                                    }
                                    if (!ens.empty()) (*propMap)[pid] = std::move(ens);
                                }
                            }
                        }
                    }
                    auto addEnchant = [&](uint32_t enchId, int32_t computedAmount, bool useComputed) {
                        auto eit = enchMap->find(enchId);
                        if (eit == enchMap->end()) return;
                        for (const auto& e : eit->second) {
                            if (e.type != 5) continue;  // ITEM_ENCHANTMENT_TYPE_STAT only
                            int32_t amount = (useComputed && e.minAmount == 0) ? computedAmount : e.minAmount;
                            if (amount != 0) out.push_back({e.arg, amount});
                        }
                    };
                    if (id < 0) {
                        auto sit = suffixMap->find(-id);
                        if (sit != suffixMap->end())
                            for (const auto& [ench, pct] : sit->second) {
                                int32_t amount = static_cast<int32_t>(
                                    (static_cast<int64_t>(pct) * suffixFactor) / 10000);
                                addEnchant(ench, amount, true);
                            }
                    } else {
                        auto pit = propMap->find(id);
                        if (pit != propMap->end())
                            for (uint32_t ench : pit->second) addEnchant(ench, 0, false);
                    }
                    return out;
                });
            }
            LOG_INFO("Addon system initialized, found ", addonManager_->getAddons().size(), " addon(s)");
        } else {
            LOG_WARNING("Failed to initialize addon system");
            addonManager_.reset();
        }

        // Initialize world loader (handles terrain streaming, world preload, map transitions)
        worldLoader_ = std::make_unique<WorldLoader>(
            *this, renderer.get(), assetManager.get(), gameHandler.get(),
            entitySpawner_.get(), appearanceComposer_.get(), window.get(),
            world.get(), addonManager_.get());

        // Re-wire UIServices now that all services (addonManager_, worldLoader_) are available
        if (uiManager) {
            ui::UIServices uiServices;
            uiServices.window = window.get();
            uiServices.renderer = renderer.get();
            uiServices.assetManager = assetManager.get();
            uiServices.gameHandler = gameHandler.get();
            uiServices.expansionRegistry = expansionRegistry_.get();
            uiServices.addonManager = addonManager_.get();
            uiServices.audioCoordinator = audioCoordinator_.get();
            uiServices.entitySpawner = entitySpawner_.get();
            uiServices.appearanceComposer = appearanceComposer_.get();
            uiServices.worldLoader = worldLoader_.get();
            uiManager->setServices(uiServices);
        }

        // Start background preload for last-played character's world.
        // Warms the file cache so terrain tile loading is faster at Enter World.
        {
            auto lastWorld = worldLoader_->loadLastWorldInfo();
            if (lastWorld.valid) {
                worldLoader_->startWorldPreload(lastWorld.mapId, lastWorld.mapName, lastWorld.x, lastWorld.y);
            }
        }

    } else {
        LOG_WARNING("Failed to initialize asset manager - asset loading will be unavailable");
        LOG_WARNING("Set WOW_DATA_PATH environment variable to your WoW Data directory");
    }

    // If the archives never opened, the fonts were not tried at all. Loose
    // files are still worth a look, and this is still before the first frame.
    if (uiManager) {
        uiManager->loadInterfaceFont(assetPath, nullptr);
        uiManager->loadInterfaceFont(dataPath, nullptr);
    }

    // Set up UI callbacks
    setupUICallbacks();

    LOG_INFO("Application initialized successfully");
    running = true;
    return true;
}

void Application::run() {
    ZoneScopedN("Application::run");
    LOG_INFO("Starting main loop");

    // Do not pin the main thread. The shared render pool is created lazily
    // from this thread, and OS threads inherit their creator's affinity mask
    // on Linux. Pinning here silently confined every later render worker to
    // CPU 0 and defeated all command-recording parallelism.

    const bool frameProfileEnabled = core::envFlagEnabled("WOWEE_FRAME_PROFILE", false);
    if (frameProfileEnabled) {
        LOG_INFO("Frame timing profile enabled (WOWEE_FRAME_PROFILE=1)");
    }

    auto lastTime = std::chrono::high_resolution_clock::now();
    beatWatchdog();
    std::atomic<int64_t>& watchdogHeartbeatMs = watchdogHeartbeatMs_;
    // Signal flag: watchdog sets this when a stall is detected, main loop
    // handles the actual SDL calls. SDL2 video functions must only be called
    // from the main thread (the one that called SDL_Init); calling them from
    // a background thread is UB on macOS (Cocoa) and unsafe on other platforms.
    std::atomic<bool> watchdogRequestRelease{false};
    // std::jthread would express this, but libc++ on the macOS CI image has no
    // <stop_token>, so the flag and the thread stay separate and a guard does
    // what jthread's destructor would: stop and join on every exit from this
    // scope, including an exception. That replaces a try/catch whose only job
    // was to run the same three lines before rethrowing, plus a second copy of
    // them after the loop.
    std::atomic<bool> watchdogRunning{true};
    std::thread watchdogThread([&watchdogRunning, &watchdogHeartbeatMs, &watchdogRequestRelease]() {
        bool signalledForCurrentStall = false;
        while (watchdogRunning.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const int64_t lastBeatMs = watchdogHeartbeatMs.load(std::memory_order_acquire);
            const int64_t stallMs = nowMs - lastBeatMs;

            if (stallMs > 1500) {
                if (!signalledForCurrentStall) {
                    watchdogRequestRelease.store(true, std::memory_order_release);
                    LOG_WARNING("Main-loop stall detected (", stallMs,
                                "ms) - requesting mouse capture release");
                    signalledForCurrentStall = true;
                }
            } else {
                signalledForCurrentStall = false;
            }
        }
    });

    // Stops and joins on the way out of this scope, however that happens.
    struct WatchdogStop {
        std::atomic<bool>& running;
        std::thread& thread;
        ~WatchdogStop() {
            running.store(false, std::memory_order_release);
            if (thread.joinable()) thread.join();
        }
    } watchdogStop{watchdogRunning, watchdogThread};

    while (running && !window->shouldClose()) {
        const auto frameStart = std::chrono::steady_clock::now();
        beatWatchdog();

        // Handle watchdog mouse-release request on the main thread where
        // SDL video calls are safe (required by SDL2 threading model).
        if (watchdogRequestRelease.exchange(false, std::memory_order_acq_rel)) {
            SDL_SetRelativeMouseMode(SDL_FALSE);
            SDL_ShowCursor(SDL_ENABLE);
            if (window && window->getSDLWindow()) {
                SDL_SetWindowGrab(window->getSDLWindow(), SDL_FALSE);
            }
            if (renderer && renderer->getCameraController()) {
                renderer->getCameraController()->releaseMouseCapture();
            }
            LOG_WARNING("Watchdog: force-released mouse capture on main thread");
        }

        // Hold the frame to the cap, if one is set.
        //
        // Before the delta time is taken, so the wait is part of the frame
        // it paces rather than a stall the next one has to absorb. Sleep
        // granularity is a millisecond or so, which is close enough for a
        // cap and far cheaper than spinning.
        if (window) {
            const int capFps = window->frameCap();
            if (capFps > 0) {
                const std::chrono::duration<float> target(1.0f / static_cast<float>(capFps));
                const auto elapsed = std::chrono::high_resolution_clock::now() - lastTime;
                if (elapsed < target) {
                    std::this_thread::sleep_for(target - elapsed);
                }
            }
        }

        // Calculate delta time
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> deltaTimeDuration = currentTime - lastTime;
        float deltaTime = deltaTimeDuration.count();
        lastTime = currentTime;

        // Cap delta time to prevent large jumps
        if (deltaTime > 0.1f) {
            deltaTime = 0.1f;
        }

        if (renderer && renderer->getCameraController() && ImGui::GetIO().WantCaptureMouse) {
            renderer->getCameraController()->releaseMouseCapture();
        }

        // Poll events
        //
        // Cleared here rather than after the draw, because the flag only
        // ever describes the iteration it was set in: the pump sets it and
        // the draw, further down this same iteration, is the only reader.
        ui::clearInterfaceConsumedKeys();
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
#ifdef __ANDROID__
            // The stick and the pinch read the finger events SDL sends
            // alongside the mouse ones. They claim nothing else: every panel,
            // slider and action button keeps working through the mouse SDL
            // already makes of the first finger.
            {
                int tw = 0, th = 0;
                if (window) SDL_GetWindowSize(window->getSDLWindow(), &tw, &th);
                ui::touchControls().handleEvent(event, tw, th);
            }

            // The activity going to the background takes the native window with
            // it, and the Vulkan surface and swapchain built on it die at the
            // same moment. Drawing to them afterwards is what left the client on
            // a black screen it never came back from. SDL sends these on the
            // same thread as the loop, so the teardown happens before the
            // window is gone rather than after.
            if (event.type == SDL_APP_WILLENTERBACKGROUND) {
                if (window && window->getVkContext()) {
                    window->getVkContext()->releaseSurface();
                }
            } else if (event.type == SDL_APP_DIDENTERFOREGROUND) {
                if (window && window->getVkContext()) {
                    int w = 0, h = 0;
                    SDL_GetWindowSize(window->getSDLWindow(), &w, &h);
                    if (!window->getVkContext()->restoreSurface(
                            window->getSDLWindow(), w, h)) {
                        LOG_ERROR("Resuming without a surface; the client cannot draw");
                    }
                }
            }
#endif
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
                    // The interface gets first refusal, and only where a
                    // frame under the cursor asked for the wheel. Zooming
                    // the camera while scrolling a quest log is what
                    // happens without the check, and a frame that did not
                    // ask must not swallow it either.
                    bool takenByUi = false;
                    if (addonManager_ && addonManager_->getLuaEngine()) {
                        // ImGui's cursor, not SDL's, because that is what
                        // the rest of the widget dispatch is fed and the
                        // two need not agree on a scaled display. Flipped
                        // to bottom-origin for the same reason the button
                        // dispatch is: the tree measures upward.
                        const ImGuiIO& mio = ImGui::GetIO();
                        takenByUi = addonManager_->getLuaEngine()->dispatchMouseWheel(
                            mio.MousePos.x, mio.DisplaySize.y - mio.MousePos.y,
                            static_cast<float>(event.wheel.y));
                    }
                    if (!takenByUi) {
                        renderer->getCameraController()->processMouseWheel(
                            static_cast<float>(event.wheel.y));
                    }
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
                    // Mark swapchain dirty so it gets recreated at the correct size
                    if (window->getVkContext()) {
                        window->getVkContext()->markSwapchainDirty();
                    }
                    // Vulkan viewport set in command buffer, not globally
                    if (renderer && renderer->getCamera() && newHeight > 0) {
                        renderer->getCamera()->setAspectRatio(static_cast<float>(newWidth) / newHeight);
                    }
                    // Notify addons so UI layouts can adapt to the new size
                    if (addonManager_)
                        addonManager_->fireEvent("DISPLAY_SIZE_CHANGED");
                }
                // Sound in Background. Off in the real client and off here:
                // losing the window silences the client rather than playing
                // on behind whatever the player switched to. Read at the
                // moment focus changes, so clearing the box takes effect on
                // the next alt-tab and not the next restart.
                else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST ||
                         event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) {
                    const bool focused =
                        (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED);
                    const bool playInBackground =
                        addons::storedCVarValue("Sound_EnableSoundWhenGameIsInBG", "0") != "0";
                    audio::AudioEngine::instance().setSuspended(!focused && !playInBackground);
                }
            }
            // Typed text, when an addon's edit box is listening for it.
            else if (event.type == SDL_TEXTINPUT) {
                if (addonManager_ && addonsLoaded_) {
                    if (auto* engine = addonManager_->getLuaEngine();
                        engine && engine->editBoxHasFocus()) {
                        engine->dispatchText(event.text.text);
                    }
                }
            }
            // Debug controls
            else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
                // Shift, control and alt announce themselves. The interface
                // watches these to swap what a tooltip shows and what a
                // click will do - item comparison appears on shift, and an
                // action button's self-cast indicator on alt. Four frames
                // listen and none had ever been told.
                if (addonManager_ && addonsLoaded_) {
                    const char* modName = nullptr;
                    switch (event.key.keysym.sym) {
                        case SDLK_LSHIFT: modName = "LSHIFT"; break;
                        case SDLK_RSHIFT: modName = "RSHIFT"; break;
                        case SDLK_LCTRL:  modName = "LCTRL";  break;
                        case SDLK_RCTRL:  modName = "RCTRL";  break;
                        case SDLK_LALT:   modName = "LALT";   break;
                        case SDLK_RALT:   modName = "RALT";   break;
                        default: break;
                    }
                    // Repeats are not changes: holding shift sends a stream
                    // of key-downs and the interface would rebuild every
                    // tooltip in the game for each one.
                    if (modName && event.key.repeat == 0) {
                        addonManager_->fireEvent(
                            "MODIFIER_STATE_CHANGED",
                            {modName, event.type == SDL_KEYDOWN ? "1" : "0"});
                    }
                }
            }
            if (event.type == SDL_KEYDOWN) {
                // An addon's edit box takes the keystroke before anything
                // else looks at it. Otherwise typing into one would also
                // walk the character, and backspace would trip a keybind.
                if (addonManager_ && addonsLoaded_) {
                    if (auto* engine = addonManager_->getLuaEngine();
                        engine && engine->editBoxHasFocus()) {
                        const bool ctrl =
                            (event.key.keysym.mod & KMOD_CTRL) != 0;
                        // Said before the dispatch, because dispatching is
                        // what lets go of the focus that the check above
                        // just used - ask afterwards and the box no longer
                        // admits to having taken anything.
                        //
                        // Only the three that let go. Every other key
                        // leaves the box focused, so the poll's own typing
                        // guard still answers for them.
                        switch (event.key.keysym.sym) {
                            case SDLK_ESCAPE:
                                ui::noteInterfaceConsumedKey(ImGuiKey_Escape);
                                // The first of the three ways a press ends
                                // early, and the correct one: an edit box
                                // had the keyboard, so Escape closed the
                                // box and means nothing further. WoW stops
                                // here too.
                                LOG_WARNING("Escape: taken in the pump by a "
                                            "focused edit box; it closes the "
                                            "box and stops");
                                break;
                            case SDLK_RETURN:
                            case SDLK_KP_ENTER:
                                ui::noteInterfaceConsumedKey(ImGuiKey_Enter);
                                break;
                            case SDLK_TAB:
                                ui::noteInterfaceConsumedKey(ImGuiKey_Tab);
                                break;
                            default: break;
                        }
                        engine->dispatchKey(event.key.keysym.sym, ctrl);
                        continue;
                    }
                    // No edit box, but a dialog may still be listening -
                    // the colour picker, the stack splitter, the coin
                    // pickup. The key is only swallowed when one of them
                    // actually takes it, so with nothing up the movement
                    // keys and every binding carry on exactly as before.
                    if (auto* engine = addonManager_->getLuaEngine()) {
                        if (engine->dispatchFrameKey(event.key.keysym.sym, true)) {
                            // Escape says so, because "Escape does nothing"
                            // is a live report and this is one of the three
                            // ways the press can end before the chain that
                            // decides what it means ever runs. At warning:
                            // the default log carries nothing else, so an
                            // info line here is a line nobody will ever
                            // see. See the pair in GameScreen.
                            if (event.key.keysym.sym == SDLK_ESCAPE) {
                                LOG_WARNING("Escape: taken in the pump by a "
                                            "frame listening for keys; the "
                                            "chain below never runs");
                            }
                            continue;
                        }
                    }
                    // Then whatever the interface has bound to the key.
                    //
                    // Last of the three, because a focused edit box and an
                    // open dialog both outrank a binding - which is WoW's
                    // order too. It declines for anything this client
                    // performs itself, so the keys that already work are
                    // untouched; what it adds is every command the client
                    // has no path for, which until now could be bound in
                    // the interface's own key-binding panel and then never
                    // honoured by any press.
                    if (auto* engine = addonManager_->getLuaEngine()) {
                        const SDL_Keymod mods = SDL_GetModState();
                        if (engine->dispatchBindingKey(
                                event.key.keysym.sym,
                                (mods & KMOD_SHIFT) != 0,
                                (mods & KMOD_CTRL) != 0,
                                (mods & KMOD_ALT) != 0, true)) {
                            // The fourth way a press can end in the pump - an
                            // interface key binding claimed it - which had no
                            // line. For the DEFAULT Escape this does not fire:
                            // Escape binds to TOGGLEGAMEMENU, which
                            // clientActsOnBinding lists, so dispatchBindingKey
                            // yields (returns false) and the poll chain below
                            // runs. It fires only if Escape has been rebound to
                            // a FrameXML command a binding script handles - in
                            // which case *that* is why the game menu never
                            // opens, and this line names it. So it is a real
                            // signal for a rebound Escape, not the default one.
                            if (event.key.keysym.sym == SDLK_ESCAPE) {
                                LOG_WARNING("Escape: taken in the pump by an "
                                            "interface key binding (rebound off "
                                            "TOGGLEGAMEMENU); the game-menu chain "
                                            "never runs");
                            }
                            continue;
                        }
                    }
                }
                // The third way, and the ordinary one: nothing in the
                // interface wanted it, so the poll further down decides.
                // Said as well as the other two, because the whole value of
                // these lines is that exactly one of them appears per
                // press - silence would mean the key never arrived at all,
                // and that is a different fault in a different place.
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    LOG_WARNING("Escape: through the pump untaken; the chain "
                                "below decides");
                }
                // Skip non-function-key input when UI (chat) has keyboard focus
                bool uiHasKeyboard = ImGui::GetIO().WantCaptureKeyboard ||
                                     ui::interfaceTakingTypedInput();
                auto sc = event.key.keysym.scancode;
                bool isFKey = (sc >= SDL_SCANCODE_F1 && sc <= SDL_SCANCODE_F12);
                if (uiHasKeyboard && !isFKey) {
                    continue;  // Let ImGui handle the keystroke
                }

                // The development keys are not built into a release.
                //
                // They are not bindings the player chose and not anything
                // the interface knows about, so a stray F-key silently
                // changing how the world is drawn is a bug report about
                // rendering rather than about a keystroke. Independent ifs
                // rather than a chain, so either can be compiled out on its
                // own; the scancodes are mutually exclusive anyway.
#ifndef NDEBUG
                // F1: Toggle performance HUD
                if (event.key.keysym.scancode == SDL_SCANCODE_F1) {
                    if (renderer && renderer->getPerformanceHUD()) {
                        renderer->getPerformanceHUD()->toggle();
                        bool enabled = renderer->getPerformanceHUD()->isEnabled();
                        LOG_INFO("Performance HUD: ", enabled ? "ON" : "OFF");
                    }
                }
                // No F4 shadow toggle.
                //
                // setShadowsEnabled ignores what it is passed and holds
                // shadows on, because turning them off loses the device -
                // which is why the settings panel has no control for it
                // either. So the key did nothing, and said the opposite in
                // the log every time: it read the flag back to decide what
                // to print, the flag never moved, and every press logged
                // "Shadows: OFF" while they stayed on.
#endif
                // F8: Debug WMO floor at current position
                if (event.key.keysym.scancode == SDL_SCANCODE_F8 && event.key.repeat == 0) {
                    if (renderer && renderer->getWMORenderer()) {
                        glm::vec3 pos = renderer->getCharacterPosition();
                        LOG_WARNING("F8: WMO floor debug at render pos (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                        renderer->getWMORenderer()->debugDumpGroupsAtPosition(pos.x, pos.y, pos.z);
                    }
                }
            }
        }

        if (window->shouldClose()) {
            break;
        }

        // Update input
        Input::getInstance().update();

        // Update application state
        try {
            FrameMark;
            update(deltaTime);
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during Application::update (state=", static_cast<int>(state),
                      ", dt=", deltaTime, "): ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during Application::update (state=", static_cast<int>(state),
                      ", dt=", deltaTime, "): ", e.what());
            throw;
        }
        if (window->shouldClose()) {
            break;
        }
        // Render
        try {
            render();
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during Application::render (state=", static_cast<int>(state), "): ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during Application::render (state=", static_cast<int>(state), "): ", e.what());
            throw;
        }
        // Swap buffers
        try {
            window->swapBuffers();
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during swapBuffers: ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during swapBuffers: ", e.what());
            throw;
        }

        processDeferredLogoutToLogin();

        // Exit gracefully on GPU device lost (unrecoverable)
        if (renderer && renderer->getVkContext() && renderer->getVkContext()->isDeviceLost()) {
            LOG_ERROR("GPU device lost - exiting application");
            window->setShouldClose(true);
        }

        // Pace from the start of the frame we just completed. Using deltaTime
        // here measured the previous frame, and relying only on FIFO present
        // still allowed the main thread to saturate a core on high-refresh or
        // compositor-managed displays. VSync defaults to a conservative 60 Hz;
        // disabling it retains the existing 240 Hz ceiling.
        const auto targetFrame = window->isVsyncEnabled()
            ? std::chrono::microseconds(16667)
            : std::chrono::microseconds(4167);
        const auto deadline = frameStart + targetFrame;
        const auto now = std::chrono::steady_clock::now();
        if (now < deadline) {
            std::this_thread::sleep_until(deadline);
        }
    }

    LOG_INFO("Main loop ended");
}

void Application::shutdown() {
    LOG_DEBUG("Shutting down application...");

    // Hide the window immediately so the OS doesn't think the app is frozen
    // during the (potentially slow) resource cleanup below.
    if (window && window->getSDLWindow()) {
        SDL_HideWindow(window->getSDLWindow());
    }

    // Stop background world preloader before destroying AssetManager
    if (worldLoader_) {
        worldLoader_->cancelWorldPreload();
    };

    // End the session while the renderer is still alive. disconnect() fires a
    // despawn callback per entity, and those release creature and player
    // models through EntitySpawner's renderer pointer. Left to ~GameHandler,
    // which runs after renderer.reset(), that pointer is eleven lines of
    // freed memory - the "Exit Game during a logout" path arrived here still
    // holding a world full of entities and crashed on the first one. Calling
    // it now empties the entity list, so the destructor's own disconnect()
    // has nothing left to fire.
    if (gameHandler) {
        gameHandler->disconnect();
    }

    // Save floor cache before renderer is destroyed
    if (renderer && renderer->getWMORenderer()) {
        size_t cacheSize = renderer->getWMORenderer()->getFloorCacheSize();
        if (cacheSize > 0) {
            LOG_DEBUG("Saving WMO floor cache (", cacheSize, " entries)...");
            renderer->getWMORenderer()->saveFloorCache();
            LOG_DEBUG("Floor cache saved.");
        }
    }

    // The portrait's offscreen view holds Vulkan resources and is registered
    // with the renderer, so it has to let go before the renderer does. Left to
    // member destruction order it would free images against a device that has
    // already gone.
    unitPortrait_.shutdown(renderer.get());
    targetPortrait_.shutdown(renderer.get());
    petPortrait_.shutdown(renderer.get());
    focusPortrait_.shutdown(renderer.get());
    npcPortrait_.shutdown(renderer.get());
    inspectModel_.shutdown(renderer.get());
    dressUpModel_.shutdown(renderer.get());
    auctionDressUpModel_.shutdown(renderer.get());
    petModel_.shutdown(renderer.get());
    stableModel_.shutdown(renderer.get());
    companionModel_.shutdown(renderer.get());
    for (auto& p : partyPortraits_) p.shutdown(renderer.get());
    paperdollModel_.shutdown(renderer.get());

    // For the same reason, and it was never being done: ImGui's Vulkan backend
    // holds a pipeline, its layout and descriptor set layout, two shader
    // modules, a sampler, a command pool, the font atlas image with its view
    // and memory, and the per-frame vertex and index buffers. Nineteen objects,
    // which is exactly what vkDestroyDevice reported as leaked on every run.
    //
    // UIManager::shutdown() calls ImGui_ImplVulkan_Shutdown and was reached
    // from nowhere; its destructor is defaulted, so nothing released them. It
    // has to run here rather than at member destruction, because the backend
    // waits on the device and frees against it.
    if (uiManager) {
        uiManager->shutdown();
    }

    // What the block upload actually saved this session, rather than what the
    // asset survey predicted it would. Reported here because it is a whole
    // session's total and the walk decides which textures that covers.
    //
    // Latched because shutdown() runs twice on the way out - once from main and
    // again from ~Application - and the counters are static, so unlike every
    // member below they survive the first pass and would report themselves a
    // second time.
    static bool tallyReported = false;
    if (const auto tally = rendering::VkTexture::blockUploadTally();
        tally.textures > 0 && !std::exchange(tallyReported, true)) {
        const double savedPct =
            100.0 * (1.0 - static_cast<double>(tally.blockBytes) /
                               static_cast<double>(tally.decodedBytes));
        LOG_INFO("Block texture upload: ", tally.textures, " textures, ",
                 tally.blockBytes / (1024 * 1024), " MB uploaded vs ",
                 tally.decodedBytes / (1024 * 1024), " MB decoded (",
                 std::lround(savedPct), "% saved)");
    }

    // Explicitly shut down the renderer before destroying it - this ensures
    // all sub-renderers free their VMA allocations in the correct order,
    // before VkContext::shutdown() calls vmaDestroyAllocator().
    LOG_DEBUG("Shutting down renderer...");
    if (renderer) {
        renderer->shutdown();
    }
    LOG_DEBUG("Renderer shutdown complete, resetting...");
    renderer.reset();

    // Shutdown audio coordinator after renderer (renderer may reference audio during shutdown)
    if (audioCoordinator_) {
        audioCoordinator_->shutdown();
    }
    audioCoordinator_.reset();

    LOG_DEBUG("Resetting world...");
    world.reset();
    LOG_DEBUG("Resetting gameHandler...");
    gameHandler.reset();
    gameServices_ = {};
    LOG_DEBUG("Resetting authHandler...");
    authHandler.reset();
    LOG_DEBUG("Resetting assetManager...");
    assetManager.reset();
    LOG_DEBUG("Resetting uiManager...");
    uiManager.reset();
    LOG_DEBUG("Resetting window...");
    window.reset();

    running = false;
    LOG_DEBUG("Application shutdown complete");
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
            if (addonManager_ && addonsLoaded_) {
                addonManager_->fireEvent("PLAYER_LEAVING_WORLD");
                addonManager_->saveAllSavedVariables();
            }
            npcsSpawned = false;
            playerCharacterSpawned = false;
            addonsLoaded_ = false;
            if (appearanceComposer_) appearanceComposer_->setWeaponsSheathed(false);
            wasAutoAttacking_ = false;
            if (worldLoader_) worldLoader_->resetLoadedMap();
            spawnedPlayerGuid_ = 0;
            spawnedAppearanceBytes_ = 0;
            spawnedFacialFeatures_ = 0;
            if (renderer && renderer->getCharacterRenderer()) {
                uint32_t oldInst = renderer->getCharacterInstanceId();
                if (oldInst > 0) {
                    renderer->setCharacterFollow(0);
                    if (auto* ac = renderer->getAnimationController()) ac->clearMount();
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
                cc->setStandUpCallback([this]() {
                    if (gameHandler) {
                        gameHandler->setStandState(rendering::AnimationController::STAND_STATE_STAND);
                    }
                });
                cc->setSitDownCallback([this]() {
                    if (gameHandler) {
                        gameHandler->setStandState(rendering::AnimationController::STAND_STATE_SIT);
                    }
                    if (renderer) {
                        if (auto* ac = renderer->getAnimationController()) {
                            ac->setStandState(rendering::AnimationController::STAND_STATE_SIT);
                        }
                    }
                });
                cc->setAutoFollowCancelCallback([this]() {
                    if (gameHandler) {
                        gameHandler->cancelFollow();
                    }
                });
                cc->setUseWoWSpeed(true);
            }
            if (gameHandler) {
                gameHandler->setFaceCameraProvider([this]() -> float {
                    // Turn the character to the camera's look direction and report it in
                    // canonical space (same camera-yaw→canonical mapping as the per-frame
                    // orientation sync). Lets a fishing cast drop the bobber in front of
                    // where the player is aiming even while standing still.
                    if (!renderer || !renderer->getCameraController())
                        return gameHandler ? gameHandler->getMovementInfo().orientation : 0.0f;
                    // Use the CAMERA's live look yaw (getYaw), not getFacingYaw: the latter is
                    // the character's facing, which is NOT updated by left-mouse orbit, so while
                    // aiming at the water standing still it is stale. Turn the character to the
                    // camera aim and use the same yaw→canonical mapping the (working) melee
                    // facing uses, so the bobber lands where the player is looking.
                    float camYawDeg = renderer->getCameraController()->getYaw();
                    renderer->setCharacterYaw(camYawDeg);
                    renderer->getCameraController()->setFacingYaw(camYawDeg);
                    // "Face land to fish" ⇒ the bobber landed 180° opposite the aim, so the
                    // canonical is a half-turn off here (getYaw's zero is opposite the look
                    // vector). Add the half-turn so the bobber lands where the camera looks.
                    float canon = core::coords::normalizeAngleRad(glm::radians(360.0f - camYawDeg));
                    LOG_WARNING("[FISH-AIM] getYaw=", camYawDeg,
                                " getFacingYaw=", renderer->getCameraController()->getFacingYaw(),
                                " charYaw=", renderer->getCharacterYaw(),
                                " canonicalDeg=", canon * 57.29578f,
                                " serverYawDeg=", core::coords::canonicalToServerYaw(canon) * 57.29578f);
                    return canon;
                });

                gameHandler->setMeleeSwingCallback([this](uint32_t spellId) {
                    if (renderer) {
                        // Ranged auto-attack spells: Auto Shot (75), Shoot (5019), Throw (2764)
                        if (game::spellclass::isRangedWeaponAutoAttack(spellId)) {
                            if (appearanceComposer_ && !appearanceComposer_->isShowingRanged())
                                appearanceComposer_->showRangedWeapon(true);
                            if (auto* ac = renderer->getAnimationController()) ac->triggerRangedShot();
                        } else if (spellId != 0) {
                            if (appearanceComposer_ && appearanceComposer_->isShowingRanged())
                                appearanceComposer_->showRangedWeapon(false);
                            if (auto* ac = renderer->getAnimationController()) ac->triggerSpecialAttack(spellId);
                        } else {
                            if (appearanceComposer_ && appearanceComposer_->isShowingRanged())
                                appearanceComposer_->showRangedWeapon(false);
                            if (auto* ac = renderer->getAnimationController()) ac->triggerMeleeSwing();
                        }
                    }
                });
                gameHandler->setRangedWeaponSwapCallback([this](bool show) {
                    if (appearanceComposer_) appearanceComposer_->showRangedWeapon(show);
                });
                if (renderer && renderer->getAnimationController()) {
                    renderer->getAnimationController()->setRangedShotCompleteCallback([this]() {
                        if (appearanceComposer_) appearanceComposer_->showRangedWeapon(false);
                    });
                }
                // The logout countdown finishing is not the end of it: the server
                // confirms with SMSG_LOGOUT_COMPLETE, and only then does the client
                // leave. Without this the countdown ran out and nothing happened.
                gameHandler->setLogoutCompleteCallback([this](bool exiting) {
                    if (exiting) {
                        if (auto* ac = getAudioCoordinator()) {
                            if (auto* music = ac->getMusicManager()) music->stopMusic(0.0f);
                        }
                        LOG_INFO("Logout complete - quitting");
                        if (window) window->setShouldClose(true);
                    } else {
                        LOG_INFO("Logout complete - returning to character select");
                        logoutToLogin();
                    }
                });
                gameHandler->setKnockBackCallback([this](float vcos, float vsin, float hspeed, float vspeed) {
                    if (renderer && renderer->getCameraController()) {
                        renderer->getCameraController()->applyKnockBack(vcos, vsin, hspeed, vspeed);
                    }
                });
                gameHandler->setCameraShakeCallback([this](float magnitude, float frequency, float duration) {
                    if (renderer && renderer->getCameraController()) {
                        renderer->getCameraController()->triggerShake(magnitude, frequency, duration);
                    }
                });
                gameHandler->setAutoFollowCallback([this](const glm::vec3* renderPos) {
                    if (renderer && renderer->getCameraController()) {
                        if (renderPos) {
                            renderer->getCameraController()->setAutoFollow(renderPos);
                        } else {
                            renderer->getCameraController()->cancelAutoFollow();
                        }
                    }
                });
                // Barber shop and other live appearance changes rebuild the model.
                gameHandler->setPlayerModelRebuildCallback([this]() {
                    refreshPlayerCharacterModel();
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

bool Application::setAssetExpansionOverride(const std::string& id) {
    if (id.empty() || id == "legacy") {
        assetExpansionOverrideId_ = id;
        return true;
    }
    if (!expansionRegistry_) return false;
    const auto* profile = expansionRegistry_->getProfile(id);
    if (!profile || !std::filesystem::exists(profile->dataPath + "/manifest.json")) {
        LOG_WARNING("Cannot select asset profile '", id,
                    "': no extracted manifest is available");
        return false;
    }
    assetExpansionOverrideId_ = id;
    return true;
}

/// Loads the four things that make a protocol profile: its opcode table, its
/// update field table, the packet parsers built for it, and its DBC layouts.
///
/// Read at startup and again whenever the expansion changes, and each of those
/// had its own copy. A table added to one and not the other is invisible until
/// somebody switches expansion, or until somebody does not: the fault appears
/// on exactly one of the two paths, and which one depends on which copy was
/// edited.
///
/// Each table is set active as it loads, because a failed load leaves the
/// previous expansion's table in place rather than none at all, and a wrong
/// table reads better than a null one.
void Application::loadExpansionTables(const game::ExpansionProfile& profile) {
    if (!gameHandler) return;

    const std::string opcodesPath = profile.dataPath + "/opcodes.json";
    if (!gameHandler->getOpcodeTable().loadFromJson(opcodesPath)) {
        LOG_ERROR("Failed to load opcodes from ", opcodesPath);
    }
    game::setActiveOpcodeTable(&gameHandler->getOpcodeTable());

    const std::string updateFieldsPath = profile.dataPath + "/update_fields.json";
    if (!gameHandler->getUpdateFieldTable().loadFromJson(updateFieldsPath)) {
        LOG_ERROR("Failed to load update fields from ", updateFieldsPath);
    }
    game::setActiveUpdateFieldTable(&gameHandler->getUpdateFieldTable());

    gameHandler->setPacketParsers(game::createPacketParsers(profile.id));

    if (dbcLayout_) {
        const std::string dbcLayoutsPath = profile.dataPath + "/dbc_layouts.json";
        if (!dbcLayout_->loadFromJson(dbcLayoutsPath)) {
            LOG_ERROR("Failed to load DBC layouts from ", dbcLayoutsPath);
        }
        pipeline::setActiveDBCLayout(dbcLayout_.get());
    }
}

void Application::reloadExpansionData() {
    if (!expansionRegistry_ || !gameHandler) return;
    auto* profile = expansionRegistry_->getActive();
    if (!profile) return;

    LOG_INFO("Reloading expansion data for: ", profile->name);

    loadExpansionTables(*profile);

    // Update expansion data path for CSV DBC lookups and clear DBC cache
    if (assetManager && !profile->dataPath.empty()) {
        const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
        const std::string baseDataPath = dataPathEnv ? dataPathEnv : "./Data";
        const game::ExpansionProfile* assetProfile = profile;
        if (!assetExpansionOverrideId_.empty() &&
            assetExpansionOverrideId_ != "legacy") {
            if (const auto* selected = expansionRegistry_->getProfile(assetExpansionOverrideId_)) {
                assetProfile = selected;
            }
        }
        const std::string assetManifest = assetProfile->dataPath + "/manifest.json";
        const bool useLegacyAssets = assetExpansionOverrideId_ == "legacy";
        const std::string desiredAssetPath = !useLegacyAssets &&
                                                  std::filesystem::exists(assetManifest)
            ? assetProfile->dataPath
            : baseDataPath;
        if (desiredAssetPath != assetManager->getDataPath() &&
            assetManager->switchDataPath(desiredAssetPath) &&
            desiredAssetPath != baseDataPath) {
            assetManager->setBaseFallbackPath(baseDataPath);
        }
        LOG_INFO("Protocol profile '", profile->id, "' using asset source '",
                 useLegacyAssets ? std::string("legacy") : assetProfile->id, "'");
        assetManager->setExpansionDataPath(profile->dataPath);
        assetManager->clearDBCCache();
    }

    // Reset map name cache so it reloads from new expansion's Map.dbc
    if (worldLoader_) worldLoader_->resetMapNameCache();

    // Reset game handler DBC caches so they reload from new expansion data
    if (gameHandler) {
        gameHandler->resetDbcCaches();
    }

    // Rebuild creature display lookups with the new expansion's DBC layout
    if (entitySpawner_) entitySpawner_->rebuildLookups();
}

/// The world connection dropped under us.
///
/// Nothing watched for it. WorldState::DISCONNECTED was reached and both of the
/// AppState::DISCONNECTED arms were empty comments, so the client stayed in a
/// world it was no longer connected to - standing in a frozen scene, with the
/// only clue a warning in a log that is not shown.
///
/// The same path a logout takes, so the world is torn down the way it is meant
/// to be, and the reason is put on the login screen where WoW puts it.
void Application::handleWorldDisconnect() {
    if (state != AppState::IN_GAME) return;
    LOG_WARNING("Disconnected from the world server; returning to login");

    disconnectNotice_ = "You have been disconnected from the server.";
    logoutToLogin();
}

void Application::logoutToLogin() {
    if (renderingFrame_) {
        if (!logoutToLoginPending_) {
            LOG_INFO("Logout requested during render; deferring until frame completes");
        }
        logoutToLoginPending_ = true;
        return;
    }

    performLogoutToLogin();
}

void Application::processDeferredLogoutToLogin() {
    if (!logoutToLoginPending_) return;
    logoutToLoginPending_ = false;
    performLogoutToLogin();
}

void Application::performLogoutToLogin() {
    LOG_INFO("Logout requested");

    // Disconnect TransportManager from WMORenderer before tearing down
    if (gameHandler && gameHandler->getTransportManager()) {
        gameHandler->getTransportManager()->setWMORenderer(nullptr);
    }

    if (gameHandler) {
        gameHandler->disconnect();
    }

    // --- Per-session flags ---
    npcsSpawned = false;
    playerCharacterSpawned = false;
    if (appearanceComposer_) appearanceComposer_->setWeaponsSheathed(false);
    wasAutoAttacking_ = false;
    if (worldLoader_) worldLoader_->resetLoadedMap();
    if (worldEntryCallbacks_) worldEntryCallbacks_->resetState();
    facingSendCooldown_ = 0.0f;
    lastSentCanonicalYaw_ = 1000.0f;
    taxiStreamCooldown_ = 0.0f;
    idleYawned_ = false;

    // --- Charge state ---
    if (animationCallbacks_) animationCallbacks_->resetChargeState();

    // --- Player identity ---
    spawnedPlayerGuid_ = 0;
    spawnedAppearanceBytes_ = 0;
    spawnedFacialFeatures_ = 0;

    if (renderer && renderer->getVkContext() && !renderer->getVkContext()->isDeviceLost()) {
        LOG_DEBUG("Waiting for GPU idle before logout scene cleanup...");
        vkDeviceWaitIdle(renderer->getVkContext()->getDevice());
    }

    // --- Reset all EntitySpawner state (mount, creatures, players, GOs, queues, caches) ---
    if (entitySpawner_) entitySpawner_->resetAllState();

    world.reset();

    if (renderer) {
        renderer->resetCombatVisualState();
        // Remove old player model so it doesn't persist into next session
        if (auto* charRenderer = renderer->getCharacterRenderer()) {
            charRenderer->removeInstance(1);
        }
        // Clear all world geometry renderers
        if (auto* wmo = renderer->getWMORenderer()) {
            wmo->clearInstances();
        }
        if (auto* m2 = renderer->getM2Renderer()) {
            m2->clear();
        }
        // Clear terrain tile tracking + water surfaces so next world entry starts fresh.
        // softReset() rather than a full teardown: it clears the tile data
        // without blocking on worker thread joins.
        if (auto* terrain = renderer->getTerrainManager()) {
            terrain->softReset();
        }
        if (auto* questMarkers = renderer->getQuestMarkerRenderer()) {
            questMarkers->clear();
        }
        if (auto* footprints = renderer->getFootprintRenderer()) {
            footprints->clear();
        }
        if (auto* ac = renderer->getAnimationController()) ac->clearMount();
        renderer->setCharacterFollow(0);
        if (auto* music = audioCoordinator_ ? audioCoordinator_->getMusicManager() : nullptr) {
            music->stopMusic(0.0f);
        }
    }

    // Clear stale realm/character selection so switching servers starts fresh
    if (uiManager) {
        uiManager->getRealmScreen().reset();
        uiManager->getCharacterScreen().reset();
    }
    setState(AppState::AUTHENTICATION);

    // Said on the screen the player lands on, because there is nowhere to say
    // it on the way out: the world is being torn down as this runs.
    if (!disconnectNotice_.empty() && uiManager) {
        uiManager->getAuthScreen().setStatus(disconnectNotice_, true, /*prominent=*/true);
        disconnectNotice_.clear();
    }
}

// One frame of being in the world.
//
// The largest arm of update()'s state switch by a wide margin: input, the
// camera, the entity syncs that keep render instances on top of what the server
// says, the quest markers, the sheathe toggle, the spawn retries.
//
// updateCheckpoint travels by reference because the caller's catch reports it:
// an exception here has to say where here was.
// Everything the server says about how the player moves, handed to the camera.
//
// Run, walk, swim, flight and turn speeds, rooting, gravity, feather fall,
// water walking, and which of them the current mount, taxi or transport
// overrides. The camera owns the movement, so this is where the server's word
// about it lands.
void Application::applyServerMovementState(float deltaTime) {
    if (renderer && gameHandler && renderer->getCameraController()) {
        renderer->getCameraController()->setRunSpeedOverride(gameHandler->getServerRunSpeed());
        renderer->getCameraController()->setWalkSpeedOverride(gameHandler->getServerWalkSpeed());
        renderer->getCameraController()->setSwimSpeedOverride(gameHandler->getServerSwimSpeed());
        renderer->getCameraController()->setSwimBackSpeedOverride(gameHandler->getServerSwimBackSpeed());
        renderer->getCameraController()->setFlightSpeedOverride(gameHandler->getServerFlightSpeed());
        renderer->getCameraController()->setFlightBackSpeedOverride(gameHandler->getServerFlightBackSpeed());
        renderer->getCameraController()->setRunBackSpeedOverride(gameHandler->getServerRunBackSpeed());
        renderer->getCameraController()->setTurnRateOverride(gameHandler->getServerTurnRate());
        renderer->getCameraController()->setMovementRooted(gameHandler->isPlayerRooted());
        renderer->getCameraController()->setGravityDisabled(gameHandler->isGravityDisabled());
        renderer->getCameraController()->setFeatherFallActive(gameHandler->isFeatherFalling());
        renderer->getCameraController()->setWaterWalkActive(gameHandler->isWaterWalking());
        // Flight physics engage on the CAN_FLY ability (flying mount
        // or .gm fly), not isPlayerFlying() which also needs the
        // FLYING flag the client only sets once already airborne.
        renderer->getCameraController()->setFlyingActive(gameHandler->canFly());
        renderer->getCameraController()->setHoverActive(gameHandler->isHovering());

        // Sync camera forward pitch to movement packets during flight / swimming.
        // The server writes the pitch field when FLYING or SWIMMING flags are set;
        // without this sync it would always be 0 (horizontal), causing other
        // players to see the character flying flat even when pitching up/down.
        if (gameHandler->isPlayerFlying() || gameHandler->isSwimming()) {
            if (auto* cam = renderer->getCamera()) {
                glm::vec3 fwd = cam->getForward();
                float len = glm::length(fwd);
                if (len > 1e-4f) {
                    float pitchRad = std::asin(std::clamp(fwd.z / len, -1.0f, 1.0f));
                    gameHandler->setMovementPitch(pitchRad);
                    // Tilt the mount/character model to match flight direction
                    // (taxi flight uses setTaxiOrientationCallback for this instead)
                    if (gameHandler->isPlayerFlying() && gameHandler->isMounted()) {
                        if (auto* ac = renderer->getAnimationController()) ac->setMountPitchRoll(pitchRad, 0.0f);
                    }
                }
            }
        } else if (gameHandler->isMounted()) {
            // Reset mount pitch when not flying
            if (auto* ac = renderer->getAnimationController()) ac->setMountPitchRoll(0.0f, 0.0f);
        }
    }

    bool onTaxi = gameHandler &&
                  (gameHandler->isOnTaxiFlight() ||
                   gameHandler->isTaxiMountActive() ||
                   gameHandler->isTaxiActivationPending());
    // Deliberately narrower than onTaxi: only true once the flight is
    // actually happening (mounted/flying), not merely pending a reply.
    // A rejected CMSG_ACTIVATETAXI sets isTaxiActivationPending() true
    // for one or two frames and then clears it - if that alone counted
    // as "was taxiing", the landing clamp below would arm on every
    // rejected activation and snap the player onto whatever floor
    // candidate is closest to their current (never-actually-flown-from)
    // position, exactly as if a real flight had just landed. Live-hit
    // this at Booty Bay (multi-level WMO): a rejected activation while
    // standing on the upper platform snapped the character down to the
    // level below, with a "Cannot take that flight path" chat message
    // and zero actual movement in between.
    bool actuallyFlying = gameHandler &&
                          (gameHandler->isOnTaxiFlight() ||
                           gameHandler->isTaxiMountActive());
    bool onTransportNow = gameHandler && gameHandler->isOnTransport();
    // Clear stale client-side transport state when the tracked transport no longer exists.
    if (onTransportNow && gameHandler->getTransportManager()) {
        auto* currentTracked = gameHandler->getTransportManager()->getTransport(
            gameHandler->getPlayerTransportGuid());
        if (!currentTracked) {
            gameHandler->clearPlayerTransport();
            onTransportNow = false;
        }
    }
    // M2 transports (trams) use position-delta approach: player keeps normal
    // movement and the transport's frame-to-frame delta is applied on top.
    // Only WMO transports (ships) use full external-driven mode.
    bool isM2Transport = false;
    if (onTransportNow && gameHandler->getTransportManager()) {
        auto* tr = gameHandler->getTransportManager()->getTransport(gameHandler->getPlayerTransportGuid());
        isM2Transport = (tr && tr->isM2);
    }
    bool onWMOTransport = onTransportNow && !isM2Transport;
    if (worldEntryCallbacks_ && worldEntryCallbacks_->getWorldEntryMovementGraceTimer() > 0.0f) {
        worldEntryCallbacks_->setWorldEntryMovementGraceTimer(
            worldEntryCallbacks_->getWorldEntryMovementGraceTimer() - deltaTime);
        // Clear stale movement from before teleport each frame
        // until grace period expires (keys may still be held)
        if (renderer && renderer->getCameraController())
            renderer->getCameraController()->clearMovementInputs();
    }
    // Hearth teleport: delegated to WorldEntryCallbackHandler
    if (worldEntryCallbacks_) {
        worldEntryCallbacks_->update(deltaTime);
    }
    if (renderer && renderer->getCameraController()) {
    // A ship carries the player's reference frame, but it must not
    // freeze local controls like a taxi flight. On-deck walking is
    // folded into the transport-local offset in the sync block below.
    const bool externallyDrivenMotion = onTaxi ||
        (animationCallbacks_ && animationCallbacks_->isCharging());
    // Keep physics frozen (externalFollow) during landing clamp when terrain
    // hasn't loaded yet - prevents gravity from pulling player through void.
    bool hearthFreeze = worldEntryCallbacks_ && worldEntryCallbacks_->isHearthTeleportPending();
    const bool transportTransferFreeze = gameHandler &&
        gameHandler->hasPendingPlayerTransportWorldTransfer();
    bool landingClampActive = !onTaxi && worldEntryCallbacks_ && worldEntryCallbacks_->getTaxiLandingClampTimer() > 0.0f &&
                              worldEntryCallbacks_->getWorldEntryMovementGraceTimer() <= 0.0f &&
                              !gameHandler->isMounted();
    renderer->getCameraController()->setExternalFollow(
        externallyDrivenMotion || landingClampActive || hearthFreeze ||
        transportTransferFreeze || deckFloorPending_);
    renderer->getCameraController()->setExternalMoving(externallyDrivenMotion);
    if (externallyDrivenMotion) {
        // Drop any stale local movement toggles while server drives taxi motion.
        renderer->getCameraController()->clearMovementInputs();
        if (worldEntryCallbacks_) worldEntryCallbacks_->setTaxiLandingClampTimer(0.0f);
    }
    if (worldEntryCallbacks_ && worldEntryCallbacks_->getLastTaxiFlight() && !onTaxi) {
        renderer->getCameraController()->clearMovementInputs();
        // Keep clamping until terrain loads at landing position.
        // Timer only counts down once a valid floor is found.
        if (worldEntryCallbacks_) {
            worldEntryCallbacks_->setTaxiLandingClampTimer(2.0f);
            // Capture where the flight itself left the player, before terrain/WMO
            // streaming has a chance to move things around - this is the ground
            // truth the floor-selection below picks the closest candidate to.
            if (renderer) {
                worldEntryCallbacks_->setTaxiLandingReferenceZ(renderer->getCharacterPosition().z);
            }
        }
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
            // Ground truth for this landing: where the flight itself left
            // the player, captured once when the clamp armed. Both the probe
            // height and the candidate selection below key off it.
            const float referenceZ = worldEntryCallbacks_
                ? worldEntryCallbacks_->getTaxiLandingReferenceZ() : p.z;

            // Probe from a height that does not move, because this clamp
            // rewrites p.z every frame and the structure query only looks
            // for candidates within roughly ten yards of where it is asked.
            //
            // Probing p.z + 40 therefore only found a floor while the
            // player was still far below it. Snapping them onto it lifted
            // the probe out of range, the floor stopped being reported, the
            // clamp fell back to terrain and dropped them again - landing
            // at Booty Bay flip-flopped between the WMO deck at 36.5 and
            // the terrain at 4.5 twelve times over, and whichever the last
            // frame chose is where the player was abandoned as the timer
            // ran out, which is how they ended up thrown up and then under
            // the structure.
            //
            // referenceZ is captured once when the clamp arms, from where
            // the flight itself left the player, and is already the value
            // the selection below measures candidates against. Anchoring
            // the probe to it keeps the candidate set the same every frame,
            // so the choice is stable and the streaming re-query it exists
            // for still works.
            constexpr float kLandingProbeAbove = 2.0f;
            const float probeZ = referenceZ + kLandingProbeAbove;
            if (renderer->getWMORenderer()) {
                wmoFloor = renderer->getWMORenderer()->getFloorHeight(p.x, p.y, probeZ);
            }
            if (renderer->getM2Renderer()) {
                // Include M2 floors (bridges/platforms) in landing recovery.
                m2Floor = renderer->getM2Renderer()->getFloorHeight(p.x, p.y, probeZ);
            }

            // Pick whichever floor candidate is closest to where the taxi flight
            // itself left the player, rather than unconditionally preferring
            // WMO/M2 over terrain. Unconditionally preferring WMO/M2 fixed
            // underground landings (terrain has no notion of being underground -
            // e.g. a WMO tunnel beneath a mountain, like Ironforge's flight point,
            // where terrainFloor=769 the mountain surface beat the correct
            // wmoFloor=502 the tunnel floor with "highest wins"), but could just as
            // easily snap an *outdoor* landing down onto an unrelated structure
            // sitting underneath it. referenceZ - captured once when the clamp
            // armed, from wherever the flight simulation actually left the player -
            // is ground truth for which candidate is plausible.
            std::optional<float> targetFloor;
            const char* pickedFrom = "none";
            float bestDist = std::numeric_limits<float>::max();
            const std::pair<std::optional<float>, const char*> floorCandidates[] = {
                {wmoFloor, "wmo"}, {m2Floor, "m2"}, {terrainFloor, "terrain"}};
            for (const auto& [candidate, name] : floorCandidates) {
                if (!candidate) continue;
                float dist = std::abs(*candidate - referenceZ);
                if (dist < bestDist) {
                    bestDist = dist;
                    targetFloor = candidate;
                    pickedFrom = name;
                }
            }

            LOG_INFO("Taxi landing clamp: pos=(", p.x, ", ", p.y, ", ", p.z, ") ",
                     "referenceZ=", referenceZ, " probeZ=", probeZ, " ",
                     "terrainFloor=", terrainFloor ? std::to_string(*terrainFloor) : "none", " ",
                     "wmoFloor=", wmoFloor ? std::to_string(*wmoFloor) : "none", " ",
                     "m2Floor=", m2Floor ? std::to_string(*m2Floor) : "none", " ",
                     "picked=", pickedFrom, " ",
                     "timer=", worldEntryCallbacks_ ? worldEntryCallbacks_->getTaxiLandingClampTimer() : 0.0f);

            if (targetFloor) {
                // Floor found - snap player to it and start countdown to release
                float targetZ = *targetFloor + 0.10f;
                if (std::abs(p.z - targetZ) > 0.05f) {
                    LOG_INFO("Taxi landing clamp: snapping z ", p.z, " -> ", targetZ,
                             " (source=", pickedFrom, ")");
                    p.z = targetZ;
                    renderer->getCharacterPosition() = p;
                    glm::vec3 canonical = core::coords::renderToCanonical(p);
                    gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
                    gameHandler->sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
                }
                float clampTimer = worldEntryCallbacks_ ? worldEntryCallbacks_->getTaxiLandingClampTimer() : 0.0f;
                clampTimer -= deltaTime;
                if (worldEntryCallbacks_) worldEntryCallbacks_->setTaxiLandingClampTimer(clampTimer);
            }
            // No floor found: don't decrement timer, keep player frozen until terrain loads
        }
    }
    bool idleOrbit = renderer->getCameraController()->isIdleOrbit();
    if (idleOrbit && !idleYawned_ && renderer) {
        if (auto* ac = renderer->getAnimationController()) ac->playEmote("yawn");
        idleYawned_ = true;
    } else if (!idleOrbit) {
        idleYawned_ = false;
    }
    }
    if (renderer) {
        if (auto* ac = renderer->getAnimationController()) ac->setTaxiFlight(onTaxi);
    }
    if (renderer && renderer->getTerrainManager()) {
    renderer->getTerrainManager()->setStreamingEnabled(true);
    // Taxi flights move fast (32 u/s) - load further ahead so terrain is ready
    // before the camera arrives.  Keep updates frequent to spot new tiles early.
    renderer->getTerrainManager()->setUpdateInterval(onTaxi ? 0.033f : 0.033f);
    const int configuredLoadRadius = renderer->getTerrainLoadRadius();
    const int configuredUnloadRadius = renderer->getTerrainUnloadRadius();
    renderer->getTerrainManager()->setLoadRadius(
        onTaxi ? std::max(8, configuredLoadRadius) : configuredLoadRadius);
    renderer->getTerrainManager()->setUnloadRadius(
        onTaxi ? std::max(12, configuredUnloadRadius) : configuredUnloadRadius);
    renderer->getTerrainManager()->setTaxiStreamingMode(onTaxi);
    }
    if (worldEntryCallbacks_) worldEntryCallbacks_->setLastTaxiFlight(actuallyFlying);

    // Sync character render position ↔ canonical WoW coords each frame
    if (renderer && gameHandler) {
    // For position sync branching, only WMO transports use the dedicated
    // onTransport branch. M2 transports use the normal movement else branch
    // with a position-delta correction applied on top.
    bool onTransport = onWMOTransport;

    static bool wasOnTransport = false;
    bool onTransportNowDbg = gameHandler->isOnTransport();
    if (onTransportNowDbg != wasOnTransport) {
        LOG_DEBUG("Transport state changed: onTransport=", onTransportNowDbg,
                 " isM2=", isM2Transport,
                 " guid=0x", std::hex, gameHandler->getPlayerTransportGuid(), std::dec);
        wasOnTransport = onTransportNowDbg;
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
        // WMO transport mode (ships): keep the transport-local
        // attachment, while folding real WASD/jump movement since
        // last frame into that offset. Treating ships as fully
        // externally driven cleared movement input and reapplied the
        // boarding-time offset forever, freezing the player in a
        // running pose at the point where they stepped aboard.
        auto* tm = gameHandler->getTransportManager();
        auto* tr = tm ? tm->getTransport(gameHandler->getPlayerTransportGuid()) : nullptr;
        if (tr) {
            const uint64_t transportGuid = gameHandler->getPlayerTransportGuid();
            const uint32_t mapId = gameHandler->getCurrentMapId();
            glm::vec3 localOffset = gameHandler->getPlayerTransportOffset();
            const glm::vec3 tentativeRender = renderer->getCharacterPosition();
            const glm::vec3 expectedRender(
                tr->transform * glm::vec4(localOffset, 1.0f));
            glm::vec3 intendedRender = expectedRender;

            // A cross-map ship transfer can reuse the same synthetic GO
            // GUID on the destination map. Never interpret the continent-
            // sized difference from the previous map's ride lock as deck
            // walking: doing so rewrites a valid server transport offset
            // into a huge negative Z and drops the rider underwater.
            const bool sameRideFrame = hasWMORideLock_ &&
                lastWMORideTransportGuid_ == transportGuid &&
                lastWMORideMapId_ == mapId;
            if (!sameRideFrame && !tr->isM2) {
                deckFloorPending_ = true;
            }
            if (sameRideFrame && renderer->getCameraController()) {
                const glm::vec3 localMotion =
                    tentativeRender - lastWMORideLockedRender_;
                if (renderer->getCameraController()->isMoving()) {
                    intendedRender.x += localMotion.x;
                    intendedRender.y += localMotion.y;
                }
                if (!renderer->getCameraController()->isGrounded()) {
                    intendedRender.z += localMotion.z;
                }
            }

            // The camera controller's ordinary static-world floor query accepts a
            // moving WMO deck only when it is right under the feet, so every WMO
            // ship needs its exact transport-instance floor held under the rider -
            // otherwise gravity folds into the attachment and pulls them through
            // the hull, and multi-deck ships (stairs, ramps) can't be climbed
            // because nothing raises the rider onto the upper geometry. This is
            // the walkable-deck query for ANY ship, keyed on the transport being
            // a WMO (not M2), not on a specific ship entry. getInstanceFloorHeight
            // returns the height of whatever deck/stair is under the player, so
            // walking up onto a higher deck just follows the collision. An upward
            // jump remains fully controlled by vertical physics (skipped below).
            auto* cameraController = renderer->getCameraController();
            if (!tr->isM2 && cameraController &&
                !cameraController->isJumping()) {
                const glm::vec3 intendedCanonical =
                    core::coords::renderToCanonical(intendedRender);
                const auto deckFloor = tm->getTransportDeckFloorHeight(
                    transportGuid, intendedCanonical);
                if (deckFloor && intendedRender.z >= *deckFloor - 3.0f &&
                    intendedRender.z <= *deckFloor + 0.35f) {
                    intendedRender.z = *deckFloor + 0.10f;
                    cameraController->suppressVerticalPhysics();
                    deckFloorPending_ = false;
                } else if (deckFloorPending_ &&
                           !tm->isTransportCollisionReady(transportGuid)) {
                    // A continent transfer registers the transport GO
                    // before its WMO collision necessarily finishes loading.
                    // Preserve the local offset until this exact instance's
                    // deck exists instead of releasing gravity after a timer.
                    intendedRender = expectedRender;
                    cameraController->suppressVerticalPhysics();
                } else if (deckFloorPending_) {
                    // Collision is loaded and still found no deck underfoot,
                    // which is a real answer, not a not-ready one. The hold
                    // above waits for geometry to arrive; it must not
                    // outlive its own premise.
                    //
                    // It did, and there was no way out of it: the flag is set
                    // on boarding and cleared only by a successful deck query,
                    // so boarding somewhere the query never succeeds - a
                    // gangway that belongs to the pier rather than the hull -
                    // reapplied the boarding offset every frame forever. That
                    // is a rider running on the spot, unable to walk far
                    // enough to trigger disembark and so unable to get off.
                    deckFloorPending_ = false;
                }
            }

            localOffset = glm::vec3(
                tr->invTransform * glm::vec4(intendedRender, 1.0f));
            gameHandler->setPlayerTransportOffset(localOffset);
        }

        glm::vec3 canonical = gameHandler->getComposedWorldPosition();
        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
        renderer->getCharacterPosition() = renderPos;
        lastWMORideLockedRender_ = renderPos;
        hasWMORideLock_ = true;
        lastWMORideTransportGuid_ = gameHandler->getPlayerTransportGuid();
        lastWMORideMapId_ = gameHandler->getCurrentMapId();
        gameHandler->setPosition(canonical.x, canonical.y, canonical.z);
        if (renderer->getCameraController()) {
            glm::vec3* followTarget = renderer->getCameraController()->getFollowTargetMutable();
            if (followTarget) {
                *followTarget = renderPos;
            }
        }
        gameHandler->updateM2TransportBoarding(canonical);
    } else if (animationCallbacks_ && animationCallbacks_->isCharging()) {
        // Warrior Charge: interpolation delegated to AnimationCallbackHandler
        animationCallbacks_->updateCharge(deltaTime);
    } else {
        hasWMORideLock_ = false;
        lastWMORideTransportGuid_ = 0;
        lastWMORideMapId_ = 0xFFFFFFFFu;
        deckFloorPending_ = false;
        glm::vec3 renderPos = renderer->getCharacterPosition();

        // M2 transport riding: resolve in canonical space and lock once per frame.
        // This avoids visible jitter from mixed render/canonical delta application.
        if (isM2Transport && gameHandler->getTransportManager()) {
            auto* tr = gameHandler->getTransportManager()->getTransport(
                gameHandler->getPlayerTransportGuid());
            if (tr) {
                // Ride along at a fixed offset from the transport's current position
                // (set once at boarding - see GameHandler::updateM2TransportBoarding),
                // plus whatever the player has walked on the deck since then. WASD
                // input runs earlier in the frame and moves renderer's character
                // position directly; comparing that (tentativeCanonical) against
                // where *we* locked it last frame isolates just that walked delta,
                // so standing still holds a fixed deck position while active input
                // still moves the player, instead of either (a) fully locking
                // movement or (b) recomputing offset from the absolute position,
                // which is a no-op identity once fed back into
                // lockedCanonical = tr->position + offset: the character's render
                // position could never actually change due to the tram moving, so
                // riding appeared to "float" in place no matter how far the tram
                // traveled underneath.
                const bool isDeeprunTram =
                    game::TransportManager::isDeeprunTramTransport(*tr);
                glm::vec3 localOffset = gameHandler->getPlayerTransportOffset();
                glm::vec3 tentativeCanonical = core::coords::renderToCanonical(renderPos);
                if (hasM2RideLock_) {
                    glm::vec3 walkDelta = tentativeCanonical - lastM2RideLockedCanonical_;
                    // Root cause found: the 60-unit clamp added last round was a backstop
                    // that treated the symptom, not the cause - live data showed it
                    // getting maxed out exactly (horizDist=60.0 at the eventual disembark),
                    // meaning the runaway drift reaches whatever ceiling is set as long as
                    // that ceiling is above the 18-unit disembark threshold, so it still
                    // ended the ride ("I still got kicked off... but at least I didn't die
                    // this time" reported live). The actual bug: there's no real floor
                    // under a moving M2 car, so gravity keeps trying to pull the character
                    // down every frame even while standing still; since Z is locked, that
                    // shows up as horizontal render-position drift, and this code
                    // previously couldn't tell that apart from real WASD input - it baked
                    // ANY frame-to-frame position change into localOffset, compounding
                    // forever. Gate on genuine movement input (the same signal driving the
                    // walking animation) so gravity noise while stationary is ignored
                    // instead of accumulated; only apply the delta when the player is
                    // actually pressing a movement key.
                    const bool hasMovementInput = renderer->getCameraController() &&
                        renderer->getCameraController()->isMoving();
                    if (hasMovementInput) {
                        localOffset.x += walkDelta.x;
                        localOffset.y += walkDelta.y;
                    }
                    // Keep a generous distance clamp as a secondary backstop for any
                    // other source of drift (e.g. knockback, server-forced movement)
                    // this input gate doesn't cover.
                    if (isDeeprunTram) {
                        constexpr float kMaxRideOffsetDist = 60.0f;
                        const float offsetLen = std::sqrt(localOffset.x * localOffset.x + localOffset.y * localOffset.y);
                        if (offsetLen > kMaxRideOffsetDist) {
                            const float scale = kMaxRideOffsetDist / offsetLen;
                            localOffset.x *= scale;
                            localOffset.y *= scale;
                        }
                    }
                }
                // Z is fully locked for the Deeprun tram (see below), so
                // CameraController's own gravity integration never sees a
                // grounded frame and silently accumulates fall velocity the
                // entire ride - reported live as clipping through the world
                // "at a weird angle" right after disembarking, and as being
                // unable to jump while riding (coyote time never has a
                // grounded frame to key off). Suppress it every frame the
                // lock is active so nothing is queued up to unleash later.
                if (isDeeprunTram && renderer->getCameraController()) {
                    renderer->getCameraController()->suppressVerticalPhysics();
                }
                // Thunder Bluff lifts have real floor at both ends of their travel,
                // so letting Z track physics while airborne (jumping) is recoverable -
                // the player lands back on the platform. The Deeprun Tram tunnel has
                // no floor at all except at the two station platforms; if this ran for
                // it, isGrounded() ever reporting false mid-tunnel (e.g. because M2
                // collision for a moving instance isn't recognized as ground the same
                // way static terrain is) would let gravity pull the player away from
                // the tram with nothing to land on - reported live as falling through
                // the tram/tunnel and dying. Keep Z fully locked for the tram; only
                // lifts get the airborne exception.
                if (!isDeeprunTram && renderer->getCameraController() &&
                    !renderer->getCameraController()->isGrounded()) {
                    // While airborne (jump/fall), let vertical offset track normal
                    // physics instead of staying pinned to the boarding-time value.
                    // Without this, floor clamping can hold world-Z static unless the
                    // player is jumping, which makes lifts appear to not move vertically.
                    localOffset.z = tentativeCanonical.z - tr->position.z;
                }
                gameHandler->setPlayerTransportOffset(localOffset);

                glm::vec3 lockedCanonical = tr->position + localOffset;
                renderPos = core::coords::canonicalToRender(lockedCanonical);
                renderer->getCharacterPosition() = renderPos;
                lastM2RideLockedCanonical_ = lockedCanonical;
                hasM2RideLock_ = true;
            }
        } else {
            hasM2RideLock_ = false;
        }
        if (auto* ac = renderer->getAnimationController()) {
            ac->setM2TransportRiding(hasM2RideLock_);
        }

        // Model-Z reversal probe: the floor-selection log stays
        // quiet through the reported Undercity bob, so the bob is in
        // the Z that actually reaches the character, not the floor
        // chosen for it - gravity overshooting a stable floor, or a
        // server correction blended in. This watches the committed
        // render Z for a direction flip (up-then-down or the
        // reverse) where both legs clear 0.15, the unmistakable
        // signature of a yo-yo, and names the two steps so the next
        // Undercity walk says how far and how fast it bobs and
        // whether it moves while the feet are otherwise still.
        {
            static float lastZ = renderPos.z;
            static float lastDz = 0.0f;
            static std::chrono::steady_clock::time_point lastRevLog{};
            const float dz = renderPos.z - lastZ;
            if (std::abs(dz) > 0.15f && std::abs(lastDz) > 0.15f &&
                ((dz > 0.0f) != (lastDz > 0.0f))) {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastRevLog > std::chrono::milliseconds(250)) {
                    lastRevLog = now;
                    LOG_WARNING("Player Z reversal: z=", renderPos.z,
                                " step=", dz, " prevStep=", lastDz,
                                " grounded=",
                                renderer->getCameraController() &&
                                renderer->getCameraController()->isGrounded() ? 1 : 0,
                                " moving=",
                                renderer->getCameraController() &&
                                renderer->getCameraController()->isMoving() ? 1 : 0);
                }
            }
            if (std::abs(dz) > 0.02f) lastDz = dz;
            lastZ = renderPos.z;
        }

        glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
        gameHandler->setPosition(canonical.x, canonical.y, canonical.z);

        // Sync orientation: camera yaw (degrees) → WoW orientation (radians)
        float yawDeg = renderer->getCharacterYaw();
        // Keep all game-side orientation in canonical space.
        // We historically sent serverYaw = radians(yawDeg - 90). With the new
        // canonical<->server mapping (serverYaw = PI/2 - canonicalYaw), the
        // equivalent canonical yaw is radians(180 - yawDeg).
        float canonicalYaw = core::coords::characterYawDegToCanonical(yawDeg);
        gameHandler->setOrientation(canonicalYaw);

        // Send MSG_MOVE_SET_FACING when the player changes facing direction
        // (e.g. via mouse-look). Without this, the server predicts movement in
        // the old facing and position-corrects on the next heartbeat - the
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

        // Client-side transport board/disembark check - shared
        // with any other driver that knows the player's canonical position (see
        // GameHandler::updateM2TransportBoarding).
        if (gameHandler->getTransportManager()) {
            glm::vec3 playerCanonical = core::coords::renderToCanonical(renderPos);
            gameHandler->updateM2TransportBoarding(playerCanonical);
        }
    }
    }
}

// Keep the render instances on top of what the server says.
//
// A creature's model is placed once when it spawns and would stay there, while
// the target circle follows the entity - a drift between the two reads as a
// ring sliding off an NPC that never moved. Player instances need the same for
// a different reason: without it they never leave the run animation when they
// stop.
void Application::syncRenderInstancesToEntities(float deltaTime) {
    auto creatureSyncStart = std::chrono::steady_clock::now();
    if (renderer && gameHandler && renderer->getCharacterRenderer()) {
        auto* charRenderer = renderer->getCharacterRenderer();
        static float npcWeaponRetryTimer = 0.0f;
        npcWeaponRetryTimer += deltaTime;
        const bool npcWeaponRetryTick = (npcWeaponRetryTimer >= 1.0f);
        if (npcWeaponRetryTick) npcWeaponRetryTimer = 0.0f;
        int weaponAttachesThisTick = 0;
        glm::vec3 playerPos(0.0f);
        glm::vec3 playerRenderPos(0.0f);
        bool havePlayerPos = false;
        if (gameHandler->getPlayerGuid() != 0) {
            // The server does not continuously echo our own movement into
            // the cached player Entity. MovementInfo is the live canonical
            // position that we render and send to the server; using the
            // Entity here eventually distance-culls nearby enemies against
            // the player's old spawn position and freezes their visuals.
            const auto& movement = gameHandler->getMovementInfo();
            playerPos = glm::vec3(movement.x, movement.y, movement.z);
            playerRenderPos = core::coords::canonicalToRender(playerPos);
            havePlayerPos = true;
        }
        const float syncRadiusSq = 320.0f * 320.0f;
        auto& _creatureInstances = entitySpawner_->getCreatureInstances();
        auto& _creatureRenderPosCache = entitySpawner_->getCreatureRenderPosCache();
        auto& _creatureSwimmingState = entitySpawner_->getCreatureSwimmingState();
        auto& _creatureWalkingState = entitySpawner_->getCreatureWalkingState();
        auto& _creatureFlyingState = entitySpawner_->getCreatureFlyingState();
        auto& _creatureActiveEmotes = entitySpawner_->getCreatureActiveEmotes();
        auto& _creatureWasMoving = entitySpawner_->getCreatureWasMoving();
        auto& _creatureWasSwimming = entitySpawner_->getCreatureWasSwimming();
        auto& _creatureWasFlying = entitySpawner_->getCreatureWasFlying();
        auto& _creatureWasWalking = entitySpawner_->getCreatureWasWalking();
        const uint64_t currentTargetGuid = gameHandler->hasTarget()
            ? gameHandler->getTargetGuid() : 0;
        const uint64_t autoAttackGuid = gameHandler->getAutoAttackTargetGuid();
        for (const auto& [guid, instanceId] : _creatureInstances) {
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (!entity || entity->getType() != game::ObjectType::UNIT) continue;

            if (npcWeaponRetryTick &&
                weaponAttachesThisTick < EntitySpawner::MAX_WEAPON_ATTACHES_PER_TICK) {
                if (entitySpawner_->retryCreatureVirtualWeapons(guid, instanceId, 30)) {
                    weaponAttachesThisTick++;
                }
            }

            // Distance check uses getLatestX/Y/Z (server-authoritative destination) to
            // avoid false-culling entities that moved while getX/Y/Z was stale.
            // Position sync still uses getX/Y/Z to preserve smooth interpolation for
            // nearby entities; distant entities (> 150u) have planarDist≈0 anyway
            // so the renderer remains driven correctly by creatureMoveCallback_.
            glm::vec3 latestCanonical(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
            float canonDistSq = 0.0f;
            if (havePlayerPos) {
                glm::vec3 d = latestCanonical - playerPos;
                canonDistSq = glm::dot(d, d);
                const bool activeCombatTarget =
                    guid == currentTargetGuid || guid == autoAttackGuid;
                if (canonDistSq > syncRadiusSq && !activeCombatTarget) continue;
            }

            // Use the destination position once the entity has reached its
            // target.  During the dead-reckoning overrun window getX/Y/Z
            // drifts past the destination at the last known velocity;
            // using getLatest (== moveEnd while isMoving_) avoids the
            // visible forward-drift followed by a backward snap.
            const bool inOverrun = entity->isEntityMoving() && !entity->isActivelyMoving();
            glm::vec3 canonical(
                inOverrun ? entity->getLatestX() : entity->getX(),
                inOverrun ? entity->getLatestY() : entity->getY(),
                inOverrun ? entity->getLatestZ() : entity->getZ());
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            auto posIt = _creatureRenderPosCache.find(guid);
            const std::optional<glm::vec3> previousRenderPos =
                posIt != _creatureRenderPosCache.end()
                    ? std::optional<glm::vec3>(posIt->second)
                    : std::nullopt;

            // Ground-moving entities need client floor projection between server
            // spline points. Use the floor nearest server Z so outdoor terrain
            // above a tunnel cannot move the model into/onto the WMO shell.
            const bool groundCreature = !_creatureFlyingState.count(guid) &&
                                        !_creatureSwimmingState.count(guid);
            if (entity->isActivelyMoving() && groundCreature) {
                if (auto floorZ = movingEntityFloor(renderer.get(), renderPos,
                                                    previousRenderPos)) {
                    renderPos.z = *floorZ;
                }
            }

            // No visual collision guard here any more.
            //
            // A creature closer than about two yards used to have its *render*
            // position pushed away from the player, so that a hostile one could
            // not appear to stand inside them while attacking. The cost was
            // that walking up to any hostile NPC made it slide backwards, which
            // is not what WoW does: units have no collision against each other
            // there at all, and a mob overlapping the player is ordinary. It
            // also drew every affected creature somewhere other than where it
            // actually was.

            if (posIt == _creatureRenderPosCache.end()) {
                charRenderer->setInstancePosition(instanceId, renderPos);
                _creatureRenderPosCache[guid] = renderPos;
            } else {
                const glm::vec3 prevPos = posIt->second;
                float ddx2 = renderPos.x - prevPos.x;
                float ddy2 = renderPos.y - prevPos.y;
                float planarDistSq = ddx2 * ddx2 + ddy2 * ddy2;
                float dz = std::abs(renderPos.z - prevPos.z);

                auto unitPtr = std::static_pointer_cast<game::Unit>(entity);
                const bool deadOrCorpse = unitPtr->getHealth() == 0;
                const bool largeCorrection = (planarDistSq > 36.0f) || (dz > 3.0f);
                // Use isActivelyMoving() so Run/Walk animation stops when the
                // creature reaches its destination. Don't use position-change
                // (planarDistSq) as a movement indicator when the entity is in
                // the dead-reckoning overrun window - the residual velocity
                // drift would keep the walk/run animation playing long after
                // the creature has actually arrived. Only fall back to position-
                // change detection for entities with no active movement tracking
                // (e.g. teleports or position-only updates from the server).
                const bool entityIsMoving = entity->isActivelyMoving();
                constexpr float kMoveThreshSq = 0.03f * 0.03f;
                const bool posChanging = planarDistSq > kMoveThreshSq || dz > 0.08f;
                const bool transportAttached =
                    gameHandler->transportAttachmentsRef().count(guid) != 0;
                // A stationary deck passenger changes world position every
                // frame because the parent ship moves. That parent motion is
                // not creature locomotion and must not trigger Run. Real
                // transport-local spline movement still reports actively moving.
                const bool positionOnlyLocomotion =
                    posChanging && !entity->isEntityMoving() && !transportAttached;
                const bool isMovingNow =
                    !deadOrCorpse && (entityIsMoving || positionOnlyLocomotion);
                if (deadOrCorpse || largeCorrection) {
                    charRenderer->setInstancePosition(instanceId, renderPos);
                } else if (planarDistSq > kMoveThreshSq || dz > 0.08f) {
                    // Entity::updateMovement already evaluates the server spline for
                    // this frame. Starting another renderer interpolation here resets
                    // that interpolation every frame and leaves the model trailing its
                    // authoritative entity position. Copy the evaluated position
                    // directly; animation transitions remain driven below.
                    charRenderer->setInstancePosition(instanceId, renderPos);
                }
                // When entity is moving but getX/Y/Z is stale (distance-culled),
                // don't call moveInstanceTo - creatureMoveCallback_ already drove
                // the renderer to the correct destination via the spline packet.
                posIt->second = renderPos;

                // Drive movement animation: Walk/Run/Swim (4/5/42) when moving,
                // Stand/SwimIdle (0/41) when idle. Walk(4) selected when WALKING flag is set.
                // WoW M2 animation IDs: 4=Walk, 5=Run, 41=SwimIdle, 42=Swim.
                // Only switch on transitions to avoid resetting animation time.
                // Don't override Death (1) animation.
                const bool isSwimmingNow = _creatureSwimmingState.count(guid) > 0;
                const bool isWalkingNow  = _creatureWalkingState.count(guid) > 0;
                const bool isFlyingNow   = _creatureFlyingState.count(guid) > 0;
                bool prevMoving   = _creatureWasMoving[guid];
                bool prevSwimming = _creatureWasSwimming[guid];
                bool prevFlying   = _creatureWasFlying[guid];
                bool prevWalking  = _creatureWasWalking[guid];
                // Trigger animation update on any locomotion-state transition, not just
                // moving/idle - e.g. creature lands while still moving → FlyForward→Run,
                // or server changes WALKING flag while creature is already running → Walk.
                const bool stateChanged = (isMovingNow  != prevMoving)   ||
                                          (isSwimmingNow != prevSwimming) ||
                                          (isFlyingNow   != prevFlying)   ||
                                          (isWalkingNow  != prevWalking && isMovingNow);
                if (stateChanged) {
                    _creatureWasMoving[guid]   = isMovingNow;
                    _creatureWasSwimming[guid] = isSwimmingNow;
                    _creatureWasFlying[guid]   = isFlyingNow;
                    _creatureWasWalking[guid]  = isWalkingNow;
                    uint32_t curAnimId = 0; float curT = 0.0f, curDur = 0.0f;
                    bool gotState = charRenderer->getAnimationState(instanceId, curAnimId, curT, curDur);
                    if (!gotState || curAnimId != rendering::anim::DEATH) {
                        uint32_t targetAnim;
                        if (isMovingNow) {
                            if (isFlyingNow)        targetAnim = rendering::anim::FLY_FORWARD;
                            else if (isSwimmingNow) targetAnim = rendering::anim::SWIM;
                            else if (isWalkingNow)  targetAnim = rendering::anim::WALK;
                            else                    targetAnim = rendering::anim::RUN;
                        } else {
                            if (isFlyingNow)        targetAnim = rendering::anim::FLY_IDLE;
                            else if (isSwimmingNow) targetAnim = rendering::anim::SWIM_IDLE;
                            else {
                                // Resume a retained state emote (work/chop loop),
                                // but only if this model ships the animation -
                                // display swaps can land on models without it
                                // (the log-carrying peasant has no chop anim).
                                targetAnim = rendering::anim::STAND;
                                auto emoteIt = _creatureActiveEmotes.find(guid);
                                if (emoteIt != _creatureActiveEmotes.end() &&
                                    charRenderer->hasAnimation(instanceId, emoteIt->second)) {
                                    targetAnim = emoteIt->second;
                                }
                            }
                        }
                        charRenderer->playAnimation(instanceId, targetAnim, /*loop=*/true);
                    }
                }
            }
            float renderYaw = entity->getOrientation() + glm::radians(90.0f);
            charRenderer->setInstanceRotation(instanceId, glm::vec3(0.0f, 0.0f, renderYaw));
        }
    }
    {
        float csMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - creatureSyncStart).count();
        if (csMs > 5.0f) {
            LOG_WARNING("SLOW update stage 'creature render sync': ", csMs, "ms (",
                        entitySpawner_->getCreatureInstances().size(), " creatures)");
        }
    }

    // --- Online player render sync (position, orientation, animation) ---
    // Mirrors the creature sync loop above but without collision guard or
    // weapon-attach logic.  Without this, online players never transition
    // back to Stand after movement stops ("run in place" bug).
    auto playerSyncStart = std::chrono::steady_clock::now();
    if (renderer && gameHandler && renderer->getCharacterRenderer()) {
        auto* charRenderer = renderer->getCharacterRenderer();
        glm::vec3 pPos(0.0f);
        bool havePPos = false;
        if (gameHandler->getPlayerGuid() != 0) {
            const auto& movement = gameHandler->getMovementInfo();
            pPos = glm::vec3(movement.x, movement.y, movement.z);
            havePPos = true;
        }
        const float pSyncRadiusSq = 320.0f * 320.0f;

        auto& _playerInstances = entitySpawner_->getPlayerInstances();
        auto& _pCreatureWasMoving = entitySpawner_->getCreatureWasMoving();
        auto& _pCreatureWasSwimming = entitySpawner_->getCreatureWasSwimming();
        auto& _pCreatureWasFlying = entitySpawner_->getCreatureWasFlying();
        auto& _pCreatureWasWalking = entitySpawner_->getCreatureWasWalking();
        auto& _pCreatureSwimmingState = entitySpawner_->getCreatureSwimmingState();
        auto& _pCreatureWalkingState = entitySpawner_->getCreatureWalkingState();
        auto& _pCreatureFlyingState = entitySpawner_->getCreatureFlyingState();
        auto& _pCreatureRenderPosCache = entitySpawner_->getCreatureRenderPosCache();
        const uint64_t playerTargetGuid = gameHandler->hasTarget()
            ? gameHandler->getTargetGuid() : 0;
        const uint64_t playerAutoAttackGuid = gameHandler->getAutoAttackTargetGuid();
        for (const auto& [guid, instanceId] : _playerInstances) {
            auto entity = gameHandler->getEntityManager().getEntity(guid);
            if (!entity || entity->getType() != game::ObjectType::PLAYER) continue;

            // Distance cull
            if (havePPos) {
                glm::vec3 latestCanonical(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
                glm::vec3 d = latestCanonical - pPos;
                const bool activeCombatTarget =
                    guid == playerTargetGuid || guid == playerAutoAttackGuid;
                if (glm::dot(d, d) > pSyncRadiusSq && !activeCombatTarget) continue;
            }

            // Position sync - clamp to destination during dead-reckoning
            // overrun to avoid drift + backward snap (same as creature loop).
            const bool inOverrun = entity->isEntityMoving() && !entity->isActivelyMoving();
            glm::vec3 canonical(
                inOverrun ? entity->getLatestX() : entity->getX(),
                inOverrun ? entity->getLatestY() : entity->getY(),
                inOverrun ? entity->getLatestZ() : entity->getZ());
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            auto posIt = _pCreatureRenderPosCache.find(guid);
            const std::optional<glm::vec3> previousRenderPos =
                posIt != _pCreatureRenderPosCache.end()
                    ? std::optional<glm::vec3>(posIt->second)
                    : std::nullopt;
            const auto* remoteMount = entitySpawner_->getRemotePlayerMount(guid);
            std::optional<glm::vec3> previousMountPos = previousRenderPos;
            if (remoteMount && previousMountPos) {
                previousMountPos->z -= remoteMount->riderHeight;
            }

            // Match creature projection: terrain alone is not a valid floor in
            // WMO overlap regions (tunnels, buildings, bridges).
            const bool groundPlayer = !_pCreatureFlyingState.count(guid) &&
                                      !_pCreatureSwimmingState.count(guid);
            if (entity->isActivelyMoving() && groundPlayer) {
                if (auto floorZ = movingEntityFloor(renderer.get(), renderPos,
                                                    previousMountPos)) {
                    renderPos.z = *floorZ;
                }
            }

            const glm::vec3 mountRenderPos = renderPos;
            if (remoteMount) renderPos.z += remoteMount->riderHeight;

            if (posIt == _pCreatureRenderPosCache.end()) {
                charRenderer->setInstancePosition(instanceId, renderPos);
                if (remoteMount) {
                    charRenderer->setInstancePosition(remoteMount->instanceId, mountRenderPos);
                }
                _pCreatureRenderPosCache[guid] = renderPos;
            } else {
                const glm::vec3 prevPos = posIt->second;
                float ddx2 = renderPos.x - prevPos.x;
                float ddy2 = renderPos.y - prevPos.y;
                float planarDistSq = ddx2 * ddx2 + ddy2 * ddy2;
                float dz = std::abs(renderPos.z - prevPos.z);

                auto unitPtr = std::static_pointer_cast<game::Unit>(entity);
                const bool deadOrCorpse = unitPtr->getHealth() == 0;
                const bool largeCorrection = (planarDistSq > 36.0f) || (dz > 3.0f);
                const bool entityIsMoving = entity->isActivelyMoving();
                constexpr float kMoveThreshSq2 = 0.03f * 0.03f;
                const bool posChanging2 = planarDistSq > kMoveThreshSq2 || dz > 0.08f;
                const bool transportAttached =
                    gameHandler->transportAttachmentsRef().count(guid) != 0;
                const bool positionOnlyLocomotion =
                    posChanging2 && !entity->isEntityMoving() && !transportAttached;
                const bool isMovingNow =
                    !deadOrCorpse && (entityIsMoving || positionOnlyLocomotion);

                if (deadOrCorpse || largeCorrection) {
                    charRenderer->setInstancePosition(instanceId, renderPos);
                    if (remoteMount) {
                        charRenderer->setInstancePosition(remoteMount->instanceId, mountRenderPos);
                    }
                } else if (planarDistSq > kMoveThreshSq2 || dz > 0.08f) {
                    float planarDist = std::sqrt(planarDistSq);
                    float duration = std::clamp(planarDist / 5.5f, 0.05f, 0.22f);
                    charRenderer->moveInstanceTo(instanceId, renderPos, duration);
                    if (remoteMount) {
                        charRenderer->moveInstanceTo(remoteMount->instanceId, mountRenderPos, duration);
                    }
                }
                posIt->second = renderPos;

                // Drive movement animation (same logic as creatures)
                const bool isSwimmingNow = _pCreatureSwimmingState.count(guid) > 0;
                const bool isWalkingNow  = _pCreatureWalkingState.count(guid) > 0;
                const bool isFlyingNow   = _pCreatureFlyingState.count(guid) > 0;
                uint32_t mountedRiderAnim = rendering::anim::MOUNT;
                if (remoteMount && isFlyingNow) {
                    const uint32_t flightPose = isMovingNow
                        ? rendering::anim::MOUNT_FLIGHT_FORWARD
                        : rendering::anim::MOUNT_FLIGHT_IDLE;
                    if (charRenderer->hasAnimation(instanceId, flightPose)) {
                        mountedRiderAnim = flightPose;
                    }
                }
                bool prevMoving   = _pCreatureWasMoving[guid];
                bool prevSwimming = _pCreatureWasSwimming[guid];
                bool prevFlying   = _pCreatureWasFlying[guid];
                bool prevWalking  = _pCreatureWasWalking[guid];
                const bool stateChanged = (isMovingNow  != prevMoving)   ||
                                          (isSwimmingNow != prevSwimming) ||
                                          (isFlyingNow   != prevFlying)   ||
                                          (isWalkingNow  != prevWalking && isMovingNow);
                if (stateChanged) {
                    _pCreatureWasMoving[guid]   = isMovingNow;
                    _pCreatureWasSwimming[guid] = isSwimmingNow;
                    _pCreatureWasFlying[guid]   = isFlyingNow;
                    _pCreatureWasWalking[guid]  = isWalkingNow;
                    uint32_t curAnimId = 0; float curT = 0.0f, curDur = 0.0f;
                    bool gotState = charRenderer->getAnimationState(instanceId, curAnimId, curT, curDur);
                    if (!gotState || curAnimId != rendering::anim::DEATH) {
                        uint32_t targetAnim;
                        if (remoteMount) {
                            // The rider keeps the mounted seat pose; locomotion
                            // belongs to the separately rendered mount model.
                            targetAnim = mountedRiderAnim;
                            uint32_t mountAnim = rendering::anim::STAND;
                            if (isMovingNow) {
                                if (isFlyingNow) mountAnim = rendering::anim::FLY_FORWARD;
                                else if (isWalkingNow) mountAnim = rendering::anim::WALK;
                                else mountAnim = rendering::anim::RUN;
                            } else if (isFlyingNow) {
                                mountAnim = rendering::anim::FLY_IDLE;
                            }
                            if (!charRenderer->hasAnimation(remoteMount->instanceId, mountAnim)) {
                                mountAnim = isMovingNow ? rendering::anim::RUN : rendering::anim::STAND;
                            }
                            charRenderer->playAnimation(remoteMount->instanceId, mountAnim, true);
                        } else if (isMovingNow) {
                            if (isFlyingNow)        targetAnim = rendering::anim::FLY_FORWARD;
                            else if (isSwimmingNow) targetAnim = rendering::anim::SWIM;
                            else if (isWalkingNow)  targetAnim = rendering::anim::WALK;
                            else                    targetAnim = rendering::anim::RUN;
                        } else {
                            if (isFlyingNow)        targetAnim = rendering::anim::FLY_IDLE;
                            else if (isSwimmingNow) targetAnim = rendering::anim::SWIM_IDLE;
                            else                    targetAnim = rendering::anim::STAND;
                        }
                        charRenderer->playAnimation(instanceId, targetAnim, /*loop=*/true);
                    }
                }

                // Server emotes and state updates can arrive after the mount
                // field and replace the one-shot mounted pose with Stand. A
                // rider's mount field is authoritative, so repair that pose
                // even when their movement state did not transition this frame.
                if (remoteMount) {
                    uint32_t riderAnim = 0;
                    float riderTime = 0.0f, riderDuration = 0.0f;
                    const bool haveRiderState = charRenderer->getAnimationState(
                        instanceId, riderAnim, riderTime, riderDuration);
                    if ((!haveRiderState || riderAnim != mountedRiderAnim) &&
                        riderAnim != rendering::anim::DEATH) {
                        charRenderer->playAnimation(instanceId, mountedRiderAnim,
                                                    /*loop=*/true);
                    }
                }
            }

            // Orientation sync
            float renderYaw = entity->getOrientation() + glm::radians(90.0f);
            charRenderer->setInstanceRotation(instanceId, glm::vec3(0.0f, 0.0f, renderYaw));
            if (remoteMount) {
                charRenderer->setInstanceRotation(remoteMount->instanceId,
                                                  glm::vec3(0.0f, 0.0f, renderYaw));
            }
        }
    }
    {
        float psMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - playerSyncStart).count();
        if (psMs > 5.0f) {
            LOG_WARNING("SLOW update stage 'player render sync': ", psMs, "ms (",
                        entitySpawner_->getPlayerInstances().size(), " players)");
        }
    }

}

void Application::updateInGame(float deltaTime, const char*& updateCheckpoint) {
    updateCheckpoint = "in_game: enter";
    const char* inGameStep = "begin";
    try {
    auto runInGameStage = [&](const char* stageName, auto&& fn) {
        auto stageStart = std::chrono::steady_clock::now();
        try {
            fn();
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during IN_GAME update stage '", stageName, "': ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during IN_GAME update stage '", stageName, "': ", e.what());
            throw;
        }
        auto stageEnd = std::chrono::steady_clock::now();
        float stageMs = std::chrono::duration<float, std::milli>(stageEnd - stageStart).count();
        noteStageTime(stageName, stageMs);
        if (stageMs > 50.0f) {
            LOG_WARNING("SLOW update stage '", stageName, "': ", stageMs, "ms");
        }
    };
    inGameStep = "gameHandler update";
    updateCheckpoint = "in_game: gameHandler update";
#ifdef __ANDROID__
    ui::touchControls().update();
    if (renderer && renderer->getCameraController()) {
        auto* cam = renderer->getCameraController();
        ui::touchControls().setCameraController(cam);
        // The touch controls turn the view themselves, from whichever finger is
        // doing it. SDL's own translation of the first finger must not turn it
        // as well, or a drag counts twice and the stick swings the camera.
        cam->setRotationSuppressed(true);
        cam->setSteering(ui::touchControls().isSteering());
    }
#endif
    runInGameStage("gameHandler->update", [&] {
        if (gameHandler) {
            gameHandler->update(deltaTime);
        }
    });
    if (addonManager_ && addonsLoaded_) {
        addonManager_->update(deltaTime);
    }
    // Always unsheath on combat engage.
    inGameStep = "auto-unsheathe";
    updateCheckpoint = "in_game: auto-unsheathe";
    if (gameHandler) {
        const bool autoAttacking = gameHandler->isAutoAttacking();
        // Keep the attachment state consistent with the ongoing attack, not
        // just the initial false -> true transition. Z can be pressed after
        // combat has already started, and pre-WotLK servers briefly send
        // ATTACKSTOP while the client retains attack intent for a retry.
        const bool attackWeaponNeeded = autoAttacking || gameHandler->hasAutoAttackIntent();
        const auto& inventory = gameHandler->getInventory();
        const auto& mainHand = inventory.getEquipSlot(game::EquipSlot::MAIN_HAND);
        const auto& offHand = inventory.getEquipSlot(game::EquipSlot::OFF_HAND);
        const auto& ranged = inventory.getEquipSlot(game::EquipSlot::RANGED);
        const bool hasOffHandWeapon = !offHand.empty() &&
            offHand.item.inventoryType == game::InvType::ONE_HAND;
        const bool hasRangedWeapon = !ranged.empty() &&
            (ranged.item.inventoryType == game::InvType::RANGED_BOW ||
             ranged.item.inventoryType == game::InvType::RANGED_GUN ||
             ranged.item.inventoryType == game::InvType::THROWN);
        const bool hasDrawableWeapon = !mainHand.empty() || hasOffHandWeapon || hasRangedWeapon;
        if (attackWeaponNeeded && hasDrawableWeapon && appearanceComposer_ &&
            appearanceComposer_->isWeaponsSheathed()) {
            if (renderer && renderer->getAnimationController()) {
                renderer->getAnimationController()->playWeaponSheathAnimation(false);
            }
            appearanceComposer_->setWeaponsSheathed(false);
            appearanceComposer_->loadEquippedWeapons();
        }
        // Swap back to melee weapon when auto-attack stops
        if (!autoAttacking && wasAutoAttacking_ && appearanceComposer_ && appearanceComposer_->isShowingRanged()) {
            appearanceComposer_->showRangedWeapon(false);
        }
        wasAutoAttacking_ = autoAttacking;
    }

    // Weapons go away on entering the water. You cannot swim with a sword out,
    // and retail puts them away for you rather than leaving them drawn through
    // the swim cycle. No reach animation: the character is already swimming, and
    // the sheathe reach would be played over a stroke it does not fit.
    {
        auto* cc = renderer ? renderer->getCameraController() : nullptr;
        const bool swimmingNow = cc && cc->isSwimming();
        if (swimmingNow && !wasSwimmingForSheath_ && appearanceComposer_
            && !appearanceComposer_->isWeaponsSheathed()) {
            appearanceComposer_->setWeaponsSheathed(true);
            appearanceComposer_->loadEquippedWeapons();
        }
        wasSwimmingForSheath_ = swimmingNow;
    }

    // Toggle weapon sheathe state with Z (ignored while UI captures keyboard).
    inGameStep = "weapon-toggle input";
    updateCheckpoint = "in_game: weapon-toggle input";
    {
        const bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard ||
                                     ui::interfaceTakingTypedInput();
        auto& input = Input::getInstance();
        if (!uiWantsKeyboard && input.isKeyJustPressed(SDL_SCANCODE_Z) && appearanceComposer_) {
            const bool sheathing = !appearanceComposer_->isWeaponsSheathed();
            if (renderer && renderer->getAnimationController()) {
                renderer->getAnimationController()->playWeaponSheathAnimation(sheathing);
            }
            appearanceComposer_->toggleWeaponsSheathed();
            appearanceComposer_->loadEquippedWeapons();
        }
    }

    inGameStep = "world update";
    updateCheckpoint = "in_game: world update";
    runInGameStage("world->update", [&] {
        if (world) {
            world->update(deltaTime);
        }
    });
    inGameStep = "spawn/equipment queues";
    updateCheckpoint = "in_game: spawn/equipment queues";
    runInGameStage("spawn/equipment queues", [&] {
        if (entitySpawner_) entitySpawner_->update();
        if (auto* cr = renderer ? renderer->getCharacterRenderer() : nullptr) {
            cr->processPendingNormalMaps(4);
        }
    });
    // Self-heal missing creature visuals: if a nearby UNIT exists in
    // entity state but has no render instance, queue a spawn retry.
    inGameStep = "creature resync scan";
    updateCheckpoint = "in_game: creature resync scan";
    if (gameHandler) {
        static float creatureResyncTimer = 0.0f;
        creatureResyncTimer += deltaTime;
        if (creatureResyncTimer >= 3.0f) {
            creatureResyncTimer = 0.0f;

            glm::vec3 playerPos(0.0f);
            bool havePlayerPos = false;
            uint64_t playerGuid = gameHandler->getPlayerGuid();
            if (auto playerEntity = gameHandler->getEntityManager().getEntity(playerGuid)) {
                playerPos = glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ());
                havePlayerPos = true;
            }

            // Counters for the summary below. "No NPCs at all" has three
            // possible causes that look identical in-world - the server
            // sent no units, the units exist but carry no display id, or
            // they exist and the spawner is refusing them - and the
            // difference decides where to look.
            int unitsSeen = 0, unitsNoDisplay = 0, unitsSpawned = 0, unitsQueued = 0;

            const float kResyncRadiusSq = 260.0f * 260.0f;
            for (const auto& pair : gameHandler->getEntityManager().getEntities()) {
                uint64_t guid = pair.first;
                const auto& entity = pair.second;
                if (!entity || guid == playerGuid) continue;
                if (entity->getType() != game::ObjectType::UNIT) continue;
                unitsSeen++;
                auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
                if (!unit || unit->getDisplayId() == 0) { unitsNoDisplay++; continue; }
                if (entitySpawner_->isCreatureSpawned(guid)) { unitsSpawned++; continue; }
                if (entitySpawner_->isCreaturePending(guid)) { unitsQueued++; continue; }

                if (havePlayerPos) {
                    glm::vec3 pos(unit->getX(), unit->getY(), unit->getZ());
                    glm::vec3 delta = pos - playerPos;
                    float distSq = glm::dot(delta, delta);
                    if (distSq > kResyncRadiusSq) continue;
                }

                float retryScale = 1.0f;
                {
                    using game::fieldIndex; using game::UF;
                    uint16_t si = fieldIndex(UF::OBJECT_FIELD_SCALE_X);
                    if (si != 0xFFFF) {
                        uint32_t raw = unit->getField(si);
                        if (raw != 0) {
                            float s2 = 1.0f;
                            s2 = std::bit_cast<float>(raw);
                            if (s2 > 0.01f && s2 < 100.0f) retryScale = s2;
                        }
                    }
                }
                entitySpawner_->queueCreatureSpawn(guid, unit->getDisplayId(),
                    unit->getX(), unit->getY(), unit->getZ(),
                    unit->getOrientation(), retryScale);
            }

            // Silent during normal play: this only speaks up when the
            // world holds units and none of them have a model.
            if (unitsSeen > 0 && unitsSpawned == 0) {
                LOG_WARNING("No creature models for ", unitsSeen,
                            " units in range (", unitsNoDisplay,
                            " without a display id, ", unitsQueued,
                            " queued, lookups built=",
                            entitySpawner_->areCreatureLookupsBuilt() ? "yes" : "no", ")");
            } else if (unitsSeen == 0) {
                static float noUnitsFor = 0.0f;
                noUnitsFor += 3.0f;
                if (noUnitsFor >= 9.0f) {
                    noUnitsFor = 0.0f;
                    LOG_WARNING("No UNIT entities in range at all - the server has "
                                "sent no creatures, or their update blocks were dropped");
                }
            }
        }
    }

    inGameStep = "gameobject/transport queues";
    updateCheckpoint = "in_game: gameobject/transport queues";
    runInGameStage("gameobject/transport queues", [&] {
        // GO/transport queues handled by entitySpawner_->update() above
    });
    inGameStep = "pending mount";
    updateCheckpoint = "in_game: pending mount";
    runInGameStage("processPendingMount", [&] {
        // Mount processing handled by entitySpawner_->update() above
    });
    // Update 3D quest markers above NPCs
    inGameStep = "quest markers";
    updateCheckpoint = "in_game: quest markers";
    runInGameStage("updateQuestMarkers", [&] {
        updateQuestMarkers();
    });
    // Sync server run speed to camera controller
    inGameStep = "post-update sync";
    updateCheckpoint = "in_game: post-update sync";
    runInGameStage("post-update sync", [&] { applyServerMovementState(deltaTime); });

    // Keep creature render instances aligned with authoritative entity positions.
    // This prevents desync where target circles move with server entities but
    // creature models remain at stale spawn positions.
    inGameStep = "creature render sync";
    updateCheckpoint = "in_game: creature render sync";
    syncRenderInstancesToEntities(deltaTime);
    // Movement heartbeat is sent from GameHandler::update() to avoid
    // duplicate packets from multiple update loops.

    } catch (const std::bad_alloc& e) {
        LOG_ERROR("OOM inside AppState::IN_GAME at step '", inGameStep, "': ", e.what());
        throw;
    } catch (const std::exception& e) {
        LOG_ERROR("Exception inside AppState::IN_GAME at step '", inGameStep, "': ", e.what());
        throw;
    }
}

void Application::update(float deltaTime) {
    ZoneScopedN("Application::update");
    const char* updateCheckpoint = "enter";
    try {
    // A reload asked for by the interface. Done here, between frames, because
    // the request comes from inside the Lua state that the reload destroys -
    // performing it at the call site would free the machinery mid-call.
    if (reloadUiPending_) {
        reloadUiPending_ = false;
        if (addonManager_) {
            addonManager_->reload();
            // The same three the /reload command sends, so an interface
            // reloaded from a popup comes up in the same state as one reloaded
            // from chat rather than waiting for events that already fired.
            // Silent through the login burst, for the same reason the load
            // itself is: the unit frames initialize their dropdowns on these
            // events, every initializer ends in UnitPopup_ShowMenu, and that
            // ends in PlaySound("igMainMenuOpen"). Eight of them arrive inside
            // thirty milliseconds and stack into one hit a few seconds after
            // entering the world. The real client is behind a loading screen
            // here. See LuaEngine::setUiSoundsSuppressed.
            addons::LuaEngine::setUiSoundsSuppressed(true);
            addonManager_->fireEvent("VARIABLES_LOADED");
            // After VARIABLES_LOADED, which is when the chat window
            // settings are available to be read. Every chat frame
            // answers this by applying its saved font, colour, size
            // and docked state; without it they keep the defaults
            // they were built with however much was saved.
            addonManager_->fireEvent("UPDATE_CHAT_WINDOWS");
            addonManager_->fireEvent("PLAYER_LOGIN");
            addonManager_->fireEvent("PLAYER_ENTERING_WORLD");
            addons::LuaEngine::setUiSoundsSuppressed(false);
        }
    }
    // Update based on current state
    updateCheckpoint = "state switch";
    switch (state) {
        case AppState::AUTHENTICATION:
            updateCheckpoint = "auth: enter";
            if (authHandler) {
                authHandler->update(deltaTime);
            }
            break;

        case AppState::REALM_SELECTION:
            updateCheckpoint = "realm_selection: enter";
            if (authHandler) {
                authHandler->update(deltaTime);
            }
            break;

        case AppState::CHARACTER_CREATION:
            updateCheckpoint = "char_creation: enter";
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            if (uiManager) {
                uiManager->getCharacterCreateScreen().update(deltaTime);
            }
            break;

        case AppState::CHARACTER_SELECTION:
            updateCheckpoint = "char_selection: enter";
            if (gameHandler) {
                gameHandler->update(deltaTime);
            }
            break;

        case AppState::IN_GAME:
            // Checked before the frame rather than after it: everything below
            // reads a world that is no longer being updated.
            if (gameHandler &&
                gameHandler->getState() == game::WorldState::DISCONNECTED) {
                handleWorldDisconnect();
                break;
            }
            updateInGame(deltaTime, updateCheckpoint);
            break;

        case AppState::DISCONNECTED:
            // Handle disconnection
            break;
    }

    // Process any pending world entry request via WorldLoader
    if (worldLoader_ && state != AppState::DISCONNECTED) {
        worldLoader_->processPendingEntry();
    }

    // Update renderer (camera, etc.) only when in-game
    updateCheckpoint = "renderer update";
    if (renderer && state == AppState::IN_GAME) {
        auto rendererUpdateStart = std::chrono::steady_clock::now();
        try {
            renderer->update(deltaTime);
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during Application::update stage 'renderer->update': ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during Application::update stage 'renderer->update': ", e.what());
            throw;
        }
        float ruMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - rendererUpdateStart).count();
        if (ruMs > 50.0f) {
            LOG_WARNING("SLOW update stage 'renderer->update': ", ruMs, "ms");
        }
    }
    // Update UI
    updateCheckpoint = "ui update";
    if (uiManager) {
        try {
            uiManager->update(deltaTime);
        } catch (const std::bad_alloc& e) {
            LOG_ERROR("OOM during Application::update stage 'uiManager->update': ", e.what());
            throw;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during Application::update stage 'uiManager->update': ", e.what());
            throw;
        }
    }
    } catch (const std::bad_alloc& e) {
        LOG_ERROR("OOM in Application::update checkpoint '", updateCheckpoint, "': ", e.what());
        throw;
    } catch (const std::exception& e) {
        LOG_ERROR("Exception in Application::update checkpoint '", updateCheckpoint, "': ", e.what());
        throw;
    }
}

void Application::beatWatchdog() {
    watchdogHeartbeatMs_.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(),
        std::memory_order_release);
}

void Application::render() {
#ifdef __ANDROID__
    // Nothing to draw to between the activity leaving the foreground and coming
    // back. Acquiring an image from a destroyed swapchain is undefined, and the
    // frame would be thrown away regardless.
    //
    // update() has already opened an ImGui frame by this point, and ImGui wants
    // every frame closed before the next is opened - returning without doing so
    // aborted on the following NewFrame rather than on anything to do with the
    // surface. EndFrame closes it without drawing it.
    if (window && window->getVkContext() && window->getVkContext()->isSurfaceLost()) {
        if (ImGui::GetCurrentContext() && ImGui::GetFrameCount() > 0) {
            ImGui::EndFrame();
        }
        // The loop still runs, to keep reading the socket. It does not need to
        // run at frame rate to do that, and a phone in someone's pocket should
        // not be spending a core on it.
        SDL_Delay(50);
        return;
    }
#endif

    if (!renderer) {
        return;
    }

    // Mirrors the IN_GAME update stages: a frame that blocks long enough to trip the
    // watchdog needs to say which phase did it.
    auto runRenderStage = [this](const char* stageName, auto&& fn) {
        auto stageStart = std::chrono::steady_clock::now();
        fn();
        float stageMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - stageStart).count();
        noteStageTime(stageName, stageMs);
        if (stageMs > 50.0f) {
            LOG_WARNING("SLOW render stage '", stageName, "': ", stageMs, "ms");
        }
    };

#ifdef __ANDROID__
    // Drawn into the background list, under everything the interface puts on
    // screen, and only while a thumb is holding it.
    ui::touchControls().setInWorld(state == AppState::IN_GAME);
    ui::touchControls().draw();
#endif

    renderingFrame_ = true;
    runRenderStage("beginFrame", [&] { renderer->beginFrame(); });

    // Only render 3D world when in-game
    if (state == AppState::IN_GAME) {
        runRenderStage("renderWorld", [&] {
            renderer->renderWorld(world ? world.get() : nullptr, gameHandler.get());
        });
    }

    // Render performance HUD (within ImGui frame, before UI ends the frame)
    if (renderer) {
        runRenderStage("renderHUD", [&] { renderer->renderHUD(); });
    }

    // Addon-built frames, drawn under the client's own interface while the two
    // coexist. This is the whole point of the widget tree: an addon that calls
    // CreateTexture now puts something on the screen instead of talking to a
    // table of no-ops.
    //
    // FrameXML is a different matter. Loading it builds a hundred of Blizzard's
    // own frames at once, most of them half-supported, and putting all of them
    // on screen buries the interface this client actually draws. So when it has
    // been loaded, its frames are shown only for the elements someone named in
    // WOWEE_FRAMEXML_UI; a run that loads it to exercise the parser gets the
    // client's own interface, in the game's typefaces, and nothing else.
    static const bool interfaceEnabled = [] {
        const auto set = [](const char* n) {
            const char* v = std::getenv(n);
            return v && *v && std::string(v) != "0";
        };
        return !set("WOWEE_LOAD_FRAMEXML") || set("WOWEE_FRAMEXML_UI");
    }();
    // And only while there is a world to draw it over. This was the env flag
    // alone, so every frame FrameXML had built stayed on screen through a
    // logout and sat on top of character select - /logout ran, the character
    // left the world, and the interface it had been using never went away.
    const bool drawWidgets = interfaceEnabled && state == AppState::IN_GAME;
    if (drawWidgets && addonManager_ && addonManager_->getLuaEngine() && renderer) {
        runRenderStage("addonWidgets", [&] {
            const ImGuiIO& io = ImGui::GetIO();
            auto* engine = addonManager_->getLuaEngine();

            // The portrait is the character itself rendered small, so it is
            // produced here rather than read from a file, and handed to the
            // widget by name each frame. Told every frame rather than once,
            // because the render target is rebuilt when the window resizes and
            // a handle kept across that would be stale.
            if (gameHandler && assetManager) {
                auto& widgets = engine->widgets();
                ui::Widget* portrait = portraitWidgetId_
                    ? widgets.get(portraitWidgetId_) : nullptr;
                if (!portrait || portrait->name != "PlayerPortrait") {
                    portrait = widgets.findByName("PlayerPortrait");
                    portraitWidgetId_ = portrait ? portrait->id : 0;
                }
                // Only while something is going to show it. This is a whole
                // offscreen character pass, and running it for a frame that
                // FrameXML has not been given costs a model render and a
                // composite every frame to produce a picture nothing draws.
                // The minimap is a Vulkan pass of its own drawn inside the
                // main render pass, not an image this renderer could place, so
                // the map is told where FrameXML's frame ended up rather than
                // the frame being handed a picture. One frame behind, because
                // that pass has already run by the time this lays out - which
                // for a frame that does not move is not visible.
                // The world map is an ImGui window that centres itself, so
                // like the minimap it is told where to be rather than handed
                // a picture. The detail frame, not the outer one: that is the
                // map area inside the panel FrameXML drew.
                if (auto* wmap = renderer->getWorldMap()) {
                    ui::Widget* wm = worldMapWidgetId_
                        ? widgets.get(worldMapWidgetId_) : nullptr;
                    if (!wm || wm->name != "WorldMapDetailFrame") {
                        wm = widgets.findByName("WorldMapDetailFrame");
                        worldMapWidgetId_ = wm ? wm->id : 0;
                    }
                    if (wm && wm->visible && wm->rectW > 0.0f && wm->rectH > 0.0f) {
                        const float sc = widgets.uiScale();
                        wmap->setFrameRect(wm->left * sc,
                                           io.DisplaySize.y - (wm->bottom + wm->rectH) * sc,
                                           wm->rectW * sc, wm->rectH * sc);
                    } else {
                        wmap->clearFrameRect();
                    }
                }

                if (auto* map = renderer->getMinimap()) {
                    ui::Widget* mm = minimapWidgetId_
                        ? widgets.get(minimapWidgetId_) : nullptr;
                    if (!mm || mm->name != "Minimap") {
                        mm = widgets.findByName("Minimap");
                        minimapWidgetId_ = mm ? mm->id : 0;
                    }
                    if (mm && mm->visible && mm->rectW > 0.0f && mm->rectH > 0.0f) {
                        const float sc = widgets.uiScale();
                        map->setScreenRect(mm->left * sc,
                                           io.DisplaySize.y - (mm->bottom + mm->rectH) * sc,
                                           mm->rectW * sc, mm->rectH * sc);

                        // The zoom buttons move a number on the frame, and the
                        // map has to be told: Minimap:SetZoom is the interface
                        // saying what it wants, not the map changing. Five
                        // levels as WoW has them, zero furthest out.
                        static int appliedZoom = -1;
                        if (mm->zoomLevel != appliedZoom) {
                            appliedZoom = mm->zoomLevel;
                            static const float kRadius[5] = {
                                800.0f, 620.0f, 460.0f, 320.0f, 200.0f
                            };
                            const int lvl = (mm->zoomLevel < 0) ? 0
                                          : (mm->zoomLevel > 4 ? 4 : mm->zoomLevel);
                            map->setViewRadius(kRadius[lvl]);
                        }

                        // Minimap:PingLocation, which is all Minimap_OnClick
                        // does. The offsets are minimap-local interface units
                        // from the centre with +y up; the projection works in
                        // pixels with +y down, and the ratio of the two radii
                        // is what carries the scale, so both being interface
                        // units is enough.
                        if (mm->pingRequested) {
                            mm->pingRequested = false;
                            ui::MinimapView view;
                            view.viewRadius = map->getViewRadius();
                            view.mapRadius = mm->rectW * 0.5f;
                            if (map->isRotateWithCamera()) {
                                if (auto* cam = renderer->getCamera()) {
                                    const glm::vec3 fwd = cam->getForward();
                                    const float bearing = std::atan2(fwd.y, -fwd.x);
                                    view.cosBearing = std::cos(bearing);
                                    view.sinBearing = std::sin(bearing);
                                }
                            }
                            glm::vec3 playerRender = renderer->getCharacterInstanceId() != 0
                                ? renderer->getCharacterPosition()
                                : (renderer->getCamera() ? renderer->getCamera()->getPosition()
                                                         : glm::vec3(0.0f));
                            const glm::vec2 d = ui::minimapOffsetToRenderDelta(
                                mm->pingX, -mm->pingY, view);
                            const glm::vec3 canon = core::coords::renderToCanonical(
                                playerRender + glm::vec3(d.x, d.y, 0.0f));
                            if (gameHandler) gameHandler->sendMinimapPing(canon.x, canon.y);
                        }
                    } else {
                        map->clearScreenRect();
                    }
                }

                // How big to build a model view, from the frame it will be
                // drawn into.
                //
                // The aspect is the point. Every one of these was built at
                // 640x800 and stretched to fill a frame with a different shape:
                // the paperdoll's is 233x215, so the character was drawn a
                // third too wide, and the companion preview's is 288x160, so a
                // mount was more than twice too wide. Only the inspect frame
                // happened to match.
                //
                // In pixels, so a larger interface scale gets a larger image
                // rather than a blurrier one, and bounded either way: nothing
                // smaller than is worth compositing, nothing larger than the
                // paperdoll ever needed.
                auto sizeFor = [&widgets](ui::UnitPortrait& view, const ui::Widget* frame) {
                    if (!frame || frame->rectW <= 0.0f || frame->rectH <= 0.0f) return;
                    const float scale = widgets.uiScale();
                    const auto clampDim = [](float v) {
                        return static_cast<int>(std::clamp(v, 64.0f, 800.0f));
                    };
                    view.setTargetSize(clampDim(frame->rectW * scale),
                                       clampDim(frame->rectH * scale));
                };

                // The paperdoll's figure, on the same terms as the portrait:
                // rendered only while a frame is there to show it, and told to
                // the widget every frame because the render target is rebuilt
                // whenever the model is.
                ui::Widget* doll = paperdollWidgetId_
                    ? widgets.get(paperdollWidgetId_) : nullptr;
                if (!doll || doll->name != "CharacterModelFrame") {
                    doll = widgets.findByName("CharacterModelFrame");
                    paperdollWidgetId_ = doll ? doll->id : 0;
                }
                if (doll && doll->visible) {
                    sizeFor(paperdollModel_, doll);
                    paperdollModel_.setFraming(ui::UnitPortrait::Framing::FullBody);
                    // The rotate buttons keep their own running total and set
                    // it as an absolute facing, so the turn to apply is the
                    // change since last frame.
                    if (doll->modelFacing != paperdollFacing_) {
                        paperdollModel_.rotate(doll->modelFacing - paperdollFacing_);
                        paperdollFacing_ = doll->modelFacing;
                    }
                    paperdollModel_.update(*gameHandler, assetManager.get(),
                                           renderer.get(), io.DeltaTime);
                    doll->externalTexture = paperdollModel_.textureId();
                } else if (doll) {
                    doll->externalTexture = 0;
                }

                // The dressing room. The player as they are, plus whatever
                // has been tried on - which is the player's own paperdoll with
                // an overlay, so it is built from the same character record.
                //
                // Two of them: the auction house has a dressing room of its
                // own, with its own try-on list, and a view each rather than a
                // shared one because sharing would let whichever drew last
                // decide what the other showed.
                struct DressUp {
                    const char* name;
                    ui::UnitPortrait* model;
                    uint32_t* widgetId;
                };
                // NOLINTBEGIN(modernize-use-designated-initializers) - a table
                // whose columns are its field names, with the struct in view
                // directly above.
                const DressUp kDressUps[] = {
                    {"DressUpModel",        &dressUpModel_,        &dressUpWidgetId_},
                    {"AuctionDressUpModel", &auctionDressUpModel_, &auctionDressUpWidgetId_},
                };
                // NOLINTEND(modernize-use-designated-initializers)
                for (const DressUp& room : kDressUps) {
                    ui::Widget* dressUp = *room.widgetId
                        ? widgets.get(*room.widgetId) : nullptr;
                    if (!dressUp || dressUp->name != room.name) {
                        dressUp = widgets.findByName(room.name);
                        *room.widgetId = dressUp ? dressUp->id : 0;
                    }
                    bool shown = false;
                    if (dressUp && dressUp->visible) {
                        const game::Character* self = nullptr;
                        for (const auto& c : gameHandler->getCharacters()) {
                            if (c.guid == gameHandler->getPlayerGuid()) { self = &c; break; }
                        }
                        if (self) {
                            // What they are wearing, with a tried-on piece
                            // standing in for the slot it belongs to. Replaced
                            // rather than appended: wearing two chests is
                            // wearing the second.
                            std::vector<game::EquipmentItem> worn = self->equipment;
                            for (const auto& tried : dressUp->tryOnItems) {
                                bool replaced = false;
                                for (auto& item : worn) {
                                    if (item.inventoryType != tried.inventoryType) continue;
                                    item.displayModel = tried.displayInfoId;
                                    replaced = true;
                                    break;
                                }
                                if (!replaced) {
                                    worn.push_back({.displayModel = tried.displayInfoId,
                                                    .inventoryType = tried.inventoryType,
                                                    .enchantment = 0u});
                                }
                            }
                            const uint8_t gender =
                                (self->gender == game::Gender::FEMALE ||
                                 (self->gender == game::Gender::NONBINARY &&
                                  self->useFemaleModel)) ? 1 : 0;
                            sizeFor(*room.model, dressUp);
                            room.model->setFraming(ui::UnitPortrait::Framing::FullBody);
                            shown = room.model->updatePlayer(
                                static_cast<uint8_t>(self->race), gender,
                                self->appearanceBytes, self->facialFeatures, worn,
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                    }
                    if (dressUp) {
                        dressUp->externalTexture = shown ? room.model->textureId() : 0;
                    }
                }

                // Frames that have been *told* which creature to show, rather
                // than frames that can work it out. The stable's paperdoll is
                // told a slot and the companion preview a mount or critter, and
                // in both cases only the binding knows which - so the id is
                // written onto the frame and this builds whatever it finds.
                struct CreatureModel {
                    const char* name;
                    ui::UnitPortrait* model;
                    uint32_t* widgetId;
                };
                // NOLINTBEGIN(modernize-use-designated-initializers) - a table
                // whose columns are its field names, with the struct in view
                // directly above.
                const CreatureModel kCreatureModels[] = {
                    {"PetStableModel",      &stableModel_,    &stableModelWidgetId_},
                    {"CompanionModelFrame", &companionModel_, &companionModelWidgetId_},
                };
                // NOLINTEND(modernize-use-designated-initializers)
                for (const CreatureModel& cm : kCreatureModels) {
                    ui::Widget* frame = *cm.widgetId ? widgets.get(*cm.widgetId) : nullptr;
                    if (!frame || frame->name != cm.name) {
                        frame = widgets.findByName(cm.name);
                        *cm.widgetId = frame ? frame->id : 0;
                    }
                    bool shown = false;
                    if (frame && frame->visible && frame->modelDisplayId != 0 &&
                        entitySpawner_) {
                        const std::string modelPath =
                            entitySpawner_->getModelPathForDisplayId(frame->modelDisplayId);
                        if (!modelPath.empty()) {
                            sizeFor(*cm.model, frame);
                            cm.model->setFraming(ui::UnitPortrait::Framing::FullBody);
                            shown = cm.model->updateCreature(
                                modelPath,
                                entitySpawner_->getCreatureSkinPaths(
                                    frame->modelDisplayId, modelPath),
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                    }
                    if (frame) {
                        frame->externalTexture = shown ? cm.model->textureId() : 0;
                    }
                }

                // The pet tab's figure: the pet at full length, from the same
                // display id its portrait is drawn from.
                {
                    ui::Widget* petModel = petModelWidgetId_
                        ? widgets.get(petModelWidgetId_) : nullptr;
                    if (!petModel || petModel->name != "PetModelFrame") {
                        petModel = widgets.findByName("PetModelFrame");
                        petModelWidgetId_ = petModel ? petModel->id : 0;
                    }
                    bool shown = false;
                    if (petModel && petModel->visible) {
                        std::string modelPath;
                        uint32_t petDisplayId = 0;
                        if (const uint64_t petGuid = gameHandler->getPetGuid()) {
                            if (game::Unit* u = gameHandler->getUnitByGuid(petGuid)) {
                                petDisplayId = u->getDisplayId();
                                if (petDisplayId != 0 && entitySpawner_) {
                                    modelPath =
                                        entitySpawner_->getModelPathForDisplayId(petDisplayId);
                                }
                            }
                        }
                        if (!modelPath.empty()) {
                            sizeFor(petModel_, petModel);
                            petModel_.setFraming(ui::UnitPortrait::Framing::FullBody);
                            shown = petModel_.updateCreature(
                                modelPath,
                                entitySpawner_->getCreatureSkinPaths(petDisplayId, modelPath),
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                    }
                    if (petModel) {
                        petModel->externalTexture = shown ? petModel_.textureId() : 0;
                    }
                }

                // The inspect window's figure, on the same terms as the
                // paperdoll's - a whole model rather than a face, and only
                // while the frame is up. Whoever was inspected rather than
                // whoever is targeted: the interface keeps showing the player
                // it was opened on while the target moves on, and the inspect
                // result is where that guid already lives.
                {
                    ui::Widget* inspectModel = inspectModelWidgetId_
                        ? widgets.get(inspectModelWidgetId_) : nullptr;
                    if (!inspectModel || inspectModel->name != "InspectModelFrame") {
                        inspectModel = widgets.findByName("InspectModelFrame");
                        inspectModelWidgetId_ = inspectModel ? inspectModel->id : 0;
                    }
                    bool shown = false;
                    if (inspectModel && inspectModel->visible) {
                        const auto* result = gameHandler->getInspectResult();
                        uint8_t race = 0, gender = 0, facial = 0;
                        uint32_t appearance = 0;
                        if (result && result->guid != 0 &&
                            gameHandler->getPlayerAppearance(result->guid, race, gender,
                                                             appearance, facial)) {
                            std::vector<game::EquipmentItem> worn;
                            std::array<uint32_t, 19> displayIds{};
                            std::array<uint8_t, 19> invTypes{};
                            if (gameHandler->getOtherPlayerEquipment(result->guid, displayIds,
                                                                     invTypes)) {
                                for (size_t slot = 0; slot < displayIds.size(); ++slot) {
                                    if (displayIds[slot] == 0) continue;
                                    worn.push_back({.displayModel = displayIds[slot],
                                                    .inventoryType = invTypes[slot],
                                                    .enchantment = 0u});
                                }
                            }
                            sizeFor(inspectModel_, inspectModel);
                            inspectModel_.setFraming(ui::UnitPortrait::Framing::FullBody);
                            shown = inspectModel_.updatePlayer(
                                race, gender, appearance, facial, worn,
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                    }
                    if (inspectModel) {
                        inspectModel->externalTexture = shown ? inspectModel_.textureId() : 0;
                    }
                }

                if (portrait) {
                    sizeFor(unitPortrait_, portrait);
                    unitPortrait_.update(*gameHandler, assetManager.get(),
                                         renderer.get(), io.DeltaTime);
                    // Assigned every frame including when it is zero. Keeping
                    // the last good handle instead would leave the widget
                    // pointing at a descriptor set that has been destroyed -
                    // the render target is torn down whenever the model is
                    // rebuilt - and drawing from that is a use-after-free
                    // rather than a stale picture.
                    portrait->externalTexture = unitPortrait_.textureId();
                }

                // Every texture SetPortraitTexture was told to fill with the
                // player. Assigned here rather than when it was asked for,
                // because the handle is rebuilt whenever the portrait's render
                // target is and one kept across that is stale. The list is
                // short - five frames name the player in all of FrameXML - so
                // this costs a handful of lookups rather than a scan.
                const uint64_t playerFace = unitPortrait_.textureId();
                for (uint32_t id : widgets.playerPortraits()) {
                    if (ui::Widget* w = widgets.get(id)) {
                        w->externalTexture = playerFace;
                    }
                }

                // The target's face and the pet's, on the same terms as the
                // player's, and only while a frame is asking for one - each is
                // a whole offscreen model pass, and running it for a portrait
                // nothing draws costs a render and a composite every frame to
                // produce a picture nobody sees.
                //
                // A player from their appearance and anything else from its
                // display id. Both are read from what the world already has:
                // a creature is a model path, and a player's skin, face and
                // hair come off the update fields their spawn was built from.
                struct UnitFace {
                    const char* unit;
                    ui::UnitPortrait* portrait;
                    uint64_t guid;
                    /// A second name for the same unit, drawn from the same
                    /// view. "npc" and "questnpc" are one face under two names,
                    /// and giving each its own row would render it twice a
                    /// frame to produce the same picture.
                    const char* alias = nullptr;
                };
                // Party guids in the interface's order, which excludes the
                // player - party1 is the first other member.
                std::array<uint64_t, 4> partyGuids{};
                {
                    int found = 0;
                    for (const auto& m : gameHandler->getPartyData().members) {
                        if (m.guid == gameHandler->getPlayerGuid()) continue;
                        if (found >= 4) break;
                        partyGuids[static_cast<size_t>(found++)] = m.guid;
                    }
                }
                const uint64_t npcGuid = gameHandler->getInteractNpcGuid();
                // NOLINTBEGIN(modernize-use-designated-initializers) - a table
                // whose columns are its field names, with the struct in view
                // directly above.
                const UnitFace kFaces[] = {
                    // "npc" and "questnpc" are the same unit under two names -
                    // whoever the open window belongs to. The interface uses
                    // the second only in questframe and means nothing different
                    // by it.
                    {"npc", &npcPortrait_, npcGuid, "questnpc"},
                    {"target", &targetPortrait_,   gameHandler->getTargetGuid()},
                    {"pet",    &petPortrait_,      gameHandler->getPetGuid()},
                    {"focus",  &focusPortrait_,    gameHandler->getFocusGuid()},
                    {"party1", &partyPortraits_[0], partyGuids[0]},
                    {"party2", &partyPortraits_[1], partyGuids[1]},
                    {"party3", &partyPortraits_[2], partyGuids[2]},
                    {"party4", &partyPortraits_[3], partyGuids[3]},
                };
                // NOLINTEND(modernize-use-designated-initializers)
                for (const UnitFace& face : kFaces) {
                    static const std::vector<uint32_t> kNoAlias;
                    const auto& claimed = widgets.portraitsFor(face.unit);
                    const auto& alsoClaimed = face.alias
                        ? widgets.portraitsFor(face.alias) : kNoAlias;
                    if (claimed.empty() && alsoClaimed.empty()) continue;

                    bool built = false;
                    face.portrait->setFraming(ui::UnitPortrait::Framing::Face);
                    // Sized for the circle it is drawn into rather than for the
                    // paperdoll - the shape as much as the scale, since a
                    // portrait frame is square and the old target was not.
                    sizeFor(*face.portrait,
                            widgets.get(claimed.empty() ? alsoClaimed.front()
                                                        : claimed.front()));

                    // A player first, because a player has a display id too and
                    // it is the wrong thing to draw them from: it names the
                    // naked race model, with none of their skin, face or hair.
                    // The world already reads all three off their update
                    // fields, and now so does this.
                    uint8_t race = 0, gender = 0, facial = 0;
                    uint32_t appearance = 0;
                    if (face.guid != 0 &&
                        gameHandler->getPlayerAppearance(face.guid, race, gender,
                                                         appearance, facial)) {
                        // What they are visibly wearing, where it is known.
                        // Empty means the item queries have not come back, and
                        // the portrait leaves the model dressed as it was
                        // rather than stripping it.
                        // A real player composites their own skin; no bake.
                        face.portrait->setBakedSkin("");
                        std::vector<game::EquipmentItem> worn;
                        std::array<uint32_t, 19> displayIds{};
                        std::array<uint8_t, 19> invTypes{};
                        if (gameHandler->getOtherPlayerEquipment(face.guid, displayIds,
                                                                 invTypes)) {
                            for (size_t slot = 0; slot < displayIds.size(); ++slot) {
                                if (displayIds[slot] == 0) continue;
                                worn.push_back({.displayModel = displayIds[slot],
                                                .inventoryType = invTypes[slot],
                                                .enchantment = 0u});
                            }
                        }
                        // Whether a model actually loaded, not whether one was
                        // asked for. A failed load leaves the view holding the
                        // last unit's face, and a frame that has just changed
                        // unit would go on showing it.
                        built = face.portrait->updatePlayer(
                            race, gender, appearance, facial, worn,
                            assetManager.get(), renderer.get(), io.DeltaTime);
                    } else {
                        uint32_t displayId = 0;
                        if (face.guid != 0) {
                            // A unit, not any entity: the display id lives on
                            // Unit and on GameObject, and a targeted game
                            // object has no portrait to put one in.
                            if (game::Unit* u = gameHandler->getUnitByGuid(face.guid)) {
                                displayId = u->getDisplayId();
                            }
                        }
                        // A humanoid NPC - a guard, a questgiver, an innkeeper -
                        // is drawn the way a character is, from a race and the
                        // same skin, face and hair choices a player has.
                        // CreatureDisplayInfo leaves the skin fields empty for
                        // these, so loading one as a creature gives a shape with
                        // nothing on it. Tried first for that reason.
                        uint8_t nRace = 0, nSex = 0, nFacial = 0;
                        uint32_t nBytes = 0;
                        if (displayId != 0 && entitySpawner_ &&
                            entitySpawner_->getHumanoidAppearance(displayId, nRace, nSex,
                                                                  nBytes, nFacial)) {
                            // Dressed, from the same row. A questgiver's helm
                            // and shoulders are most of what a portrait framed
                            // on the head can show of them.
                            std::vector<game::EquipmentItem> npcWorn;
                            for (const auto& [did, invType] :
                                     entitySpawner_->getHumanoidEquipment(displayId)) {
                                npcWorn.push_back({.displayModel = did, .inventoryType = invType,
                                                   .enchantment = 0u});
                            }
                            // The bake, where there is one - nearly always.
                            // It is the whole appearance already composited,
                            // armour included, which is the half this client
                            // cannot build for an NPC. The equipment above
                            // still matters: it decides the geosets, and the
                            // bake only paints what they draw.
                            face.portrait->setBakedSkin(
                                entitySpawner_->getHumanoidBakePath(displayId));
                            built = face.portrait->updatePlayer(
                                nRace, nSex, nBytes, nFacial, npcWorn,
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                        std::string modelPath;
                        if (!built && displayId != 0 && entitySpawner_) {
                            modelPath = entitySpawner_->getModelPathForDisplayId(displayId);
                        }
                        if (!modelPath.empty()) {
                            built = face.portrait->updateCreature(
                                modelPath,
                                entitySpawner_->getCreatureSkinPaths(displayId, modelPath),
                                assetManager.get(), renderer.get(), io.DeltaTime);
                        }
                    }
                    const uint64_t drawn = built ? face.portrait->textureId() : 0;
                    for (uint32_t id : claimed) {
                        if (ui::Widget* w = widgets.get(id)) w->externalTexture = drawn;
                    }
                    for (uint32_t id : alsoClaimed) {
                        if (ui::Widget* w = widgets.get(id)) w->externalTexture = drawn;
                    }
                }
            }

            // Lay out first: hit testing reads the rects this produces, so
            // clicking a frame that moved this frame would otherwise use where
            // it used to be.
            //
            // Only the layout. The drawing waits until after the UI stage,
            // which is what puts the nameplates and the minimap's blips into
            // the same ImGui background list - and in a draw list the last
            // thing added is the thing on top. Drawing here put the panels
            // down first, so every player's name and health bar in the world
            // showed through the bags and the auction house.
            widgetRenderer_.layout(engine->widgets(), io.DisplaySize.x, io.DisplaySize.y);

            // The client's own interface has first claim, but only over the
            // point the cursor is actually on.
            //
            // WantCaptureMouse is the wrong test: it is also true whenever any
            // ImGui item is active anywhere, and this client keeps a chat input
            // on screen. A focused input would hold it true for as long as it
            // held focus, and no addon frame would ever see the mouse no matter
            // where the cursor was. Asking whether a window is under the cursor
            // is the question that was meant.
            //
            // First claim, though, is claimed on the press. Once a frame has
            // taken one the interface keeps the mouse until it comes up again,
            // wherever the cursor goes in the meantime. A drag crosses the
            // screen by definition - spellbook to action bar passes over
            // whatever else is on the way - and dispatchMouse is the only thing
            // that advances the press state, so cutting it off part way does
            // not pause the drag, it strands it: OnDragStop and OnReceiveDrag
            // live on the release path and the release is never seen. That is
            // the whole of "cannot drag a spell to the action bar"; every link
            // in the chain below this one was already correct.
            const bool overClientUi = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) &&
                                      !engine->holdsMousePress();

            // The keys that open what FrameXML now owns.
            //
            // Each of this client's panels polls its own keybinding from
            // inside its own draw, so a panel that is no longer drawn never
            // sees the key - handing one over made it unopenable. The key has
            // to reach the replacement instead, which is FrameXML's own toggle
            // for that panel.
            {
                using ui::UiElement;
                using K = ui::KeybindingManager;
                struct Route { UiElement element; K::Action action; const char* call; };
                // NOLINTBEGIN(modernize-use-designated-initializers) - a table
                // whose columns are its field names, with the struct in view
                // directly above.
                static const Route kRoutes[] = {
                    // Names checked against this FrameXML rather than assumed:
                    // ToggleAllBags, ToggleQuestLog and ToggleWorldMap are all
                    // later additions and do not exist in 3.3.5, so calling
                    // them did nothing at all - the fallback answers an
                    // unknown name and calling it yields nothing.
                    {UiElement::Bags,           K::Action::TOGGLE_BAGS,             "ToggleBackpack()"},
                    {UiElement::Spellbook,      K::Action::TOGGLE_SPELLBOOK,        "ToggleSpellBook(BOOKTYPE_SPELL)"},
                    {UiElement::QuestLog,       K::Action::TOGGLE_QUESTS,           "ToggleFrame(QuestLogFrame)"},
                    {UiElement::CharacterFrame, K::Action::TOGGLE_CHARACTER_SCREEN, "ToggleCharacter(\"PaperDollFrame\")"},
                    {UiElement::WorldMap,       K::Action::TOGGLE_WORLD_MAP,        "ToggleFrame(WorldMapFrame)"},
                    // Three more panels that poll their own key from inside
                    // their own draw, found by asking which of this client's
                    // windows do that and which of those are gated. Every one
                    // that is gated and not listed here is a key that stops
                    // working the moment its element is handed over - the
                    // talent frame on N, the guild roster and the dungeon
                    // finder on theirs.
                    //
                    // Names checked against this FrameXML, as the note above
                    // says to: ToggleTalentFrame is in uiparent.lua,
                    // ToggleFriendsFrame takes a tab number and three is the
                    // guild one, and the dungeon finder is ToggleLFDParentFrame
                    // - the binding is still called TOGGLELFGPARENT, which is
                    // the old name kept so nobody's key was reset.
                    {UiElement::Talents,        K::Action::TOGGLE_TALENTS,          "ToggleTalentFrame()"},
                    // ToggleFriendsFrame with no tab, which is what this key
                    // is: bindings.xml gives TOGGLESOCIAL - O by default, and
                    // the action here is O - the body ToggleFriendsFrame(),
                    // while ToggleFriendsFrame(3) belongs to TOGGLEGUILDTAB on
                    // a key of its own.
                    //
                    // The difference is not cosmetic. The tab form has two
                    // paths that do not close: it returns without doing
                    // anything at all when the player is in no guild, and when
                    // the selected tab is not the one asked for it re-shows on
                    // that tab rather than hiding. The plain form is what its
                    // name says - shown becomes hidden, hidden becomes shown -
                    // so the key stopped closing the window it had opened.
                    {UiElement::Social,         K::Action::TOGGLE_GUILD_ROSTER,     "ToggleFriendsFrame()"},
                    {UiElement::DungeonFinder,  K::Action::TOGGLE_DUNGEON_FINDER,   "ToggleLFDParentFrame()"},
                };
                // NOLINTEND(modernize-use-designated-initializers)
                for (const Route& r : kRoutes) {
                    if (!ui::frameXmlOwns(r.element)) continue;
                    if (!K::getInstance().isActionPressed(r.action)) continue;
                    engine->executeString(r.call);
                }
            }

            // After layout, because the range follows from the rects it just
            // resolved, and before the mouse, so a scroll bar enabled by this
            // frame's range can be clicked in it.
            // Both after layout: what is on screen and how far a frame can
            // scroll are answers layout produces, not things the interface
            // announced. Visibility first, because a panel's OnShow is what
            // fills it in and the size of what it filled is what the range is
            // then measured from.
            engine->reportEventListenersOnce();
            // Keep this after the widget render, which is where suppressed
            // windows are forced hidden. A suppressed frame that some handler
            // showed earlier this iteration is already hidden again by the time
            // this looks, so it never counts as having appeared and its
            // OnShow/OnHide never run. Several of those handlers do real work:
            // LootFrame_OnHide calls CloseLoot, which releases the loot on the
            // server - through the client's own loot window, the one actually
            // on screen. Moving this earlier would take the player's loot.
            engine->updateVisibility();
            // After visibility, because a frame shown this frame is measured
            // this frame and its size is new rather than changed.
            engine->updateSizeChanges();
            engine->updateScrollRanges();

            // A click that never reaches the interface and a click whose
            // handler does nothing look the same from the chair. Said once a
            // second while a button is actually held, so it costs nothing and
            // appears exactly when someone is wondering why nothing happened.
            if (overClientUi && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                static double lastSaid = 0.0;
                const double now = ImGui::GetTime();
                if (now - lastSaid > 1.0) {
                    lastSaid = now;
                    LOG_WARNING("WidgetInput: click held over this client's own "
                                "window, so it was not passed to the interface");
                }
            }

            if (overClientUi) {
                // Nothing below is going to run, so say so rather than leaving
                // the tree believing the cursor is still where it last saw it.
                engine->releaseMouseHover();
            } else {
                addons::LuaEngine::MouseButtons buttons;
                buttons.left   = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                buttons.right  = ImGui::IsMouseDown(ImGuiMouseButton_Right);
                buttons.middle = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
                engine->dispatchMouse(io.MousePos.x, io.MousePos.y,
                                      io.DisplaySize.y, buttons);

                // Letting a carried item go over the world is how an item is
                // destroyed. The C client raises DELETE_ITEM_CONFIRM for it and
                // uiparent.lua answers with the delete prompt, choosing the
                // sterner wording by quality; nothing fired it, so dragging an
                // item out of a bag and dropping it did nothing at all.
                //
                // Only when the release landed on no widget: over a frame the
                // drop belongs to that frame, which has already had its say
                // above.
                if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
                    !engine->mouseOverFrameXml()) {
                    const uint32_t carried = addons::cursorItemId();
                    if (carried != 0 && gameHandler) {
                        const auto* info = gameHandler->getItemInfo(carried);
                        const std::string name = info ? info->name : std::string();
                        const uint32_t quality = info ? info->quality : 0u;
                        gameHandler->fireAddonEvent(
                            "DELETE_ITEM_CONFIRM", {name, std::to_string(quality)});
                    }
                }
            }
        });
    }

    // Render UI on top (ends ImGui frame with ImGui::Render())
    if (uiManager) {
        runRenderStage("uiManager->render", [&] {
            // Not wrapped in an upload batch, though the temptation is
            // strong: nine screens here upload icons one texture at a time,
            // each submitting and waiting on the shared immediate fence, and
            // batching them took this stage from 682ms to under 200.
            //
            // It also broke correctness, which is why it is not done. This
            // stage does not only build draw lists - the unit portrait and the
            // character preview render models to offscreen targets inside it,
            // and those are real draws. With the batch still open they sampled
            // images whose uploads had not been submitted, and validation
            // began reporting VK_IMAGE_LAYOUT_UNDEFINED one second into every
            // run rather than only at the moment the device was lost.
            //
            // The stall is real and worth fixing, but it has to be fixed where
            // the uploads are - around each screen's icon loop, inside which
            // nothing draws - not around a stage that draws.
            uiManager->render(state, authHandler.get(), gameHandler.get());
        });

        // The interface, over the world and everything the world drew on top
        // of itself. Laid out much earlier, because the frame's clicks are
        // resolved against those rects; drawn here, because the stage above is
        // what adds the nameplates and the minimap blips to the background
        // draw list and the last thing added to one is the thing on top.
        //
        // The same guard the layout runs under, so the two never disagree
        // about whether there is an interface to draw.
        if (drawWidgets && addonManager_ && addonManager_->getLuaEngine() && renderer) {
            runRenderStage("frameXml->draw", [&] {
                const ImGuiIO& uiIo = ImGui::GetIO();
                widgetRenderer_.draw(addonManager_->getLuaEngine()->widgets(),
                                     uiIo.DisplaySize.x, uiIo.DisplaySize.y);
            });
        }

        // Only now is the draw data closed.
        uiManager->finishImGuiFrame();
    }

    runRenderStage("endFrame", [&] { renderer->endFrame(); });

    stageStatFrames_ += 1;
    reportStageTimes();
    renderingFrame_ = false;
    processDeferredLogoutToLogin();
}

void Application::noteStageTime(const char* stage, float milliseconds) {
    auto& stat = stageStats_[stage];
    stat.totalMs += milliseconds;
    stat.frames += 1;
    if (milliseconds > stat.worstMs) stat.worstMs = milliseconds;
}

void Application::reportStageTimes() {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    if (stageStatsSince_.time_since_epoch().count() == 0) {
        stageStatsSince_ = now;
        return;
    }
    const float windowMs = duration<float, std::milli>(now - stageStatsSince_).count();
    if (windowMs < 10000.0f) return;

    if (stageStatFrames_ > 0) {
        const float fps = stageStatFrames_ * 1000.0f / windowMs;
        LOG_INFO("frame budget over ", static_cast<int>(windowMs / 1000.0f), "s: ",
                 stageStatFrames_, " frames, ", fps, " fps, ",
                 windowMs / static_cast<float>(stageStatFrames_), "ms each");
        // Worst first: the stage to look at is the one taking the time, and a
        // long tail matters more than an average on a device that stutters.
        std::vector<std::pair<double, std::string>> byCost;
        byCost.reserve(stageStats_.size());
        for (const auto& [name, stat] : stageStats_) {
            byCost.emplace_back(stat.totalMs, name);
        }
        std::sort(byCost.rbegin(), byCost.rend());
        for (const auto& [total, name] : byCost) {
            const auto& stat = stageStats_[name];
            if (stat.frames == 0) continue;
            LOG_INFO("  ", name, ": ", static_cast<float>(total / stageStatFrames_),
                     "ms/frame avg, ", stat.worstMs, "ms worst, ",
                     static_cast<int>(100.0 * total / windowMs), "% of the window");
        }
    }

    stageStats_.clear();
    stageStatFrames_ = 0;
    stageStatsSince_ = now;
}

void Application::setupUICallbacks() {
    // Everything below binds references by dereferencing these, and four of
    // them are only created inside `if (assetManager->initialize(...))`. When
    // that fails they stay null, `*entitySpawner_` binds a reference to
    // address zero, and the first callback to touch a member writes through
    // it - the mount callback, which fires from handleLoginVerifyWorld, so the
    // client came up, listed the characters, and died the instant the world
    // was entered. Reported as issue #117 with the fault address 0x9f8, which
    // is a member offset from nothing.
    //
    // A reference bound to null is undefined the moment it is bound, so the
    // check has to be here rather than inside the callbacks.
    if (!entitySpawner_ || !renderer || !gameHandler || !assetManager ||
        !uiManager || !authHandler || !appearanceComposer_) {
        LOG_ERROR("Cannot wire the interface callbacks: ",
                  !assetManager ? "asset manager" :
                  !entitySpawner_ ? "entity spawner" :
                  !appearanceComposer_ ? "appearance composer" :
                  !renderer ? "renderer" :
                  !gameHandler ? "game handler" :
                  !uiManager ? "UI manager" : "auth handler",
                  " was never created - the game data path is the usual reason. "
                  "The client will run without them rather than crash on the "
                  "first packet that needs one.");
        return;
    }

    // ── UI screen callbacks (auth, realm, character selection/creation) ──
    uiScreenCallbacks_ = std::make_unique<UIScreenCallbackHandler>(
        *uiManager, *gameHandler, *authHandler, expansionRegistry_.get(),
        assetManager.get(),
        [this](AppState s) { setState(s); });
    uiScreenCallbacks_->setupCallbacks();

    // ── World entry, unstuck, hearthstone, bind point ──
    worldEntryCallbacks_ = std::make_unique<WorldEntryCallbackHandler>(
        *renderer, *gameHandler, worldLoader_.get(), entitySpawner_.get(),
        assetManager.get());
    worldEntryCallbacks_->setupCallbacks();

    // ── Entity spawn/despawn (creatures, players, game objects) ──
    entitySpawnCallbacks_ = std::make_unique<EntitySpawnCallbackHandler>(
        *entitySpawner_, *renderer, *gameHandler,
        [this](uint64_t guid) {
            uint64_t localGuid = gameHandler ? gameHandler->getPlayerGuid() : 0;
            uint64_t activeGuid = gameHandler ? gameHandler->getActiveCharacterGuid() : 0;
            return (localGuid != 0 && guid == localGuid) ||
                   (activeGuid != 0 && guid == activeGuid) ||
                   (spawnedPlayerGuid_ != 0 && guid == spawnedPlayerGuid_);
        });
    entitySpawnCallbacks_->setupCallbacks();

    // ── Animation: death, respawn, swing, hit, spell, emote, charge, etc. ──
    animationCallbacks_ = std::make_unique<AnimationCallbackHandler>(
        *entitySpawner_, *renderer, *gameHandler, *appearanceComposer_);
    animationCallbacks_->setupCallbacks();

    // ── NPC interaction: greeting, farewell, vendor, aggro voice ──
    npcInteractionCallbacks_ = std::make_unique<NPCInteractionCallbackHandler>(
        *entitySpawner_, renderer.get(), *gameHandler, audioCoordinator_.get());
    npcInteractionCallbacks_->setupCallbacks();

    // ── Audio: music, sound effects, level-up, achievement, LFG ──
    audioCallbacks_ = std::make_unique<AudioCallbackHandler>(
        *assetManager, audioCoordinator_.get(), renderer.get(),
        uiManager.get(), *gameHandler);
    audioCallbacks_->setupCallbacks();

    // ── Transport: mount, taxi, transport spawn/move ──
    transportCallbacks_ = std::make_unique<TransportCallbackHandler>(
        *entitySpawner_, *renderer, *gameHandler, appearanceComposer_.get());
    transportCallbacks_->setupCallbacks();
}

void Application::spawnPlayerCharacter() {
    if (playerCharacterSpawned) return;
    if (!renderer || !renderer->getCharacterRenderer() || !renderer->getCamera()) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    auto* camera = renderer->getCamera();
    bool loaded = false;
    // A nonbinary character chose which body to wear, and that choice lives on
    // the character rather than in the gender. Without it this took the
    // two-argument default - the male model - so the paper doll and the
    // character screen showed the body the player picked and the world showed
    // the other one. The voice below already resolves it this way.
    bool useFemaleModel = false;
    if (playerGender_ == game::Gender::NONBINARY && gameHandler) {
        if (const game::Character* ch = gameHandler->getActiveCharacter()) {
            useFemaleModel = ch->useFemaleModel;
        }
    }
    std::string m2Path =
        game::getPlayerModelPath(playerRace_, playerGender_, useFemaleModel);

    // Try loading selected character model from MPQ
    if (assetManager && assetManager->isInitialized()) {
        auto m2Data = assetManager->readFile(m2Path);
        if (!m2Data.empty()) {
            auto model = pipeline::M2Loader::load(m2Data);
            if (model.name.empty()) model.name = m2Path;

            // Load skin file for submesh/batch data
            std::string skinPath = pipeline::skinPathForM2(m2Path);
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

                // Resolve textures from CharSections.dbc via AppearanceComposer
                PlayerTextureInfo texInfo;
                bool useCharSections = true;
                if (appearanceComposer_) {
                    uint32_t appearanceBytes = 0;
                    if (gameHandler) {
                        const game::Character* activeChar = gameHandler->getActiveCharacter();
                        if (activeChar) {
                            appearanceBytes = activeChar->appearanceBytes;
                        }
                    }
                    texInfo = appearanceComposer_->resolvePlayerTextures(
                        model, playerRace_, playerGender_, appearanceBytes, useFemaleModel);
                }

                // Load external .anim files for sequences with external data.
                // Sequences WITH flag 0x20 have their animation data inline in the M2 file.
                // Sequences WITHOUT flag 0x20 store data in external .anim files.
                pipeline::loadExternalAnimations(*assetManager, m2Path, m2Data, model);

                charRenderer->loadModel(model, 1);

                // Apply composited textures via AppearanceComposer (saves skin state for re-compositing)
                if (useCharSections && appearanceComposer_) {
                    appearanceComposer_->compositePlayerSkin(1, texInfo);
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

	        // Build default geosets for the active character via AppearanceComposer
	        uint8_t hairStyleId = 0;
	        uint8_t facialId = 0;
	        uint8_t raceId = 0;
	        uint8_t sexId = 0;
	        if (gameHandler) {
	            if (const game::Character* ch = gameHandler->getActiveCharacter()) {
	                hairStyleId = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
	                facialId = ch->facialFeatures;
	                raceId = static_cast<uint8_t>(ch->race);
	                sexId = static_cast<uint8_t>(ch->gender);
	            }
	        }
	        auto activeGeosets = appearanceComposer_
	            ? appearanceComposer_->buildDefaultPlayerGeosets(raceId, sexId, hairStyleId, facialId)
	            : std::unordered_set<uint16_t>{};
	        charRenderer->setActiveGeosets(instanceId, activeGeosets);
	        // The player's type 8 slot is filled from CharSections by
	        // resolvePlayerTextures above, so the head-detail batch has art to
	        // draw and is worth drawing. Nothing else is set up for it.
	        charRenderer->setDrawSkinExtra(instanceId, true);

        // Play idle animation
        charRenderer->playAnimation(instanceId, rendering::anim::STAND, true);
        LOG_INFO("Spawned player character at (",
                static_cast<int>(spawnPos.x), ", ",
                static_cast<int>(spawnPos.y), ", ",
                static_cast<int>(spawnPos.z), ")");
        playerCharacterSpawned = true;

        // Set voice profile to match character race/gender
        if (auto* asm_ = audioCoordinator_ ? audioCoordinator_->getActivitySoundManager() : nullptr) {
            // Voice art is filed under the same race folder as the models -
            // Scourge for the Undead - so it comes from the one table in
            // core/character_paths.hpp. The two names here were always equal.
            const char* raceFolder = core::raceModelFolder(static_cast<uint32_t>(playerRace_));
            const char* raceBase = raceFolder;
            bool useFemaleVoice = (playerGender_ == game::Gender::FEMALE);
            if (playerGender_ == game::Gender::NONBINARY && gameHandler) {
                if (const game::Character* ch = gameHandler->getActiveCharacter()) {
                    useFemaleVoice = ch->useFemaleModel;
                }
            }
            asm_->setCharacterVoiceProfile(std::string(raceFolder), std::string(raceBase), !useFemaleVoice);
        }

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
        if (appearanceComposer_) appearanceComposer_->loadEquippedWeapons();
    }
}

void Application::refreshPlayerCharacterModel() {
    if (!playerCharacterSpawned || !gameHandler) return;
    const game::Character* ch = gameHandler->getActiveCharacter();
    if (!ch) return;
    // Only rebuild when the visible appearance actually changed. PLAYER_BYTES_2
    // also carries rest state, so the appearance hook fires on entering/leaving
    // inns and cities - a full respawn on those would be needless and jarring.
    if (ch->appearanceBytes == spawnedAppearanceBytes_ &&
        ch->facialFeatures == spawnedFacialFeatures_) {
        return;
    }
    LOG_INFO("Rebuilding player model in place for appearance change (barber shop)");

    // Keep the character exactly where it is - this is the same respawn path
    // teleport uses (so equipment/geometry are fully re-applied), just live.
    const glm::vec3 savedPos = renderer ? renderer->getCharacterPosition() : glm::vec3(0.0f);

    if (renderer && renderer->getCharacterRenderer()) {
        uint32_t oldInst = renderer->getCharacterInstanceId();
        if (oldInst > 0) {
            renderer->setCharacterFollow(0);
            if (auto* ac = renderer->getAnimationController()) ac->clearMount();
            renderer->getCharacterRenderer()->removeInstance(oldInst);
        }
    }
    playerCharacterSpawned = false;
    spawnedPlayerGuid_ = 0;
    spawnedAppearanceBytes_ = 0;
    spawnedFacialFeatures_ = 0;

    spawnSnapToGround = false; // don't snap Z - stay at the current position
    if (appearanceComposer_) appearanceComposer_->setWeaponsSheathed(false);
    spawnPlayerCharacter();

    if (renderer) renderer->getCharacterPosition() = savedPos;

    // Force equipment geosets/textures to be re-composited onto the fresh model
    // next frame - the respawn only builds the base body, and equipment isn't
    // "dirty" (nothing was equipped), so without this the model loses its armor.
    if (gameHandler) gameHandler->resetEquipmentDirtyTracking();
}

void Application::buildFactionHostilityMap(uint8_t playerRace) {
    if (!assetManager || !gameHandler) return;
    game::buildFactionHostilityMap(*assetManager, *gameHandler, playerRace);
}

// Render bounds/position queries - delegates to EntitySpawner
bool Application::getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const {
    if (entitySpawner_) return entitySpawner_->getRenderBoundsForGuid(guid, outCenter, outRadius);
    return false;
}

bool Application::getRenderFootZForGuid(uint64_t guid, float& outFootZ) const {
    if (entitySpawner_) return entitySpawner_->getRenderFootZForGuid(guid, outFootZ);
    return false;
}

bool Application::getRenderPositionForGuid(uint64_t guid, glm::vec3& outPos) const {
    if (entitySpawner_) return entitySpawner_->getRenderPositionForGuid(guid, outPos);
    return false;
}

void Application::loadQuestMarkerModels() {
    if (!assetManager || !renderer) return;

    // Quest markers are billboard sprites; the renderer's QuestMarkerRenderer handles
    // texture loading and pipeline setup during world initialization.
    // Calling initialize() here is a no-op if already done; harmless if called early.
    if (auto* qmr = renderer->getQuestMarkerRenderer()) {
        if (auto* vkCtx = renderer->getVkContext()) {
            VkDescriptorSetLayout pfl = renderer->getPerFrameSetLayout();
            if (pfl != VK_NULL_HANDLE) {
                if (!qmr->initialize(vkCtx, pfl, assetManager.get()))
                    LOG_WARNING("Quest marker renderer re-init failed (non-fatal)");
            }
        }
    }
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

    // Clear all markers (we'll re-add active ones)
    questMarkerRenderer->clear();

    static bool firstRun = true;
    int markersAdded = 0;

    // Add markers for NPCs with quest status
    for (const auto& [guid, status] : questStatuses) {
        // Determine marker type
        int markerType = -1;  // -1 = no marker

        // One mapping for all five places that draw this - see
        // quest_giver_status.hpp. Five of the eleven statuses the server can
        // send had no name here, so an NPC whose quests you have out-levelled
        // got no marker at all.
        const auto marker = game::questGiverMarker(status);
        float markerGrayscale = marker.dim ? 1.0f : 0.0f;
        if (marker.symbol) {
            const bool bang = marker.symbol[0] == '!';
            // 0 is the exclamation texture, 1 the gold question mark and 2 the
            // grey one; the shader desaturates from markerGrayscale.
            markerType = bang ? 0 : (marker.dim ? 2 : 1);
        }

        if (markerType < 0) continue;

        // Get NPC entity position
        auto entity = gameHandler->getEntityManager().getEntity(guid);
        if (!entity) continue;
        if (entity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            const std::string& name = unit->getName();
            // Case-insensitive substring scan without copying or lowercasing the
            // whole name into a fresh std::string every frame. Spirit healers
            // and spirit guides use their own white visual cue, so skip them.
            auto containsCI = [&](const char* needle, size_t nlen) {
                if (name.size() < nlen) return false;
                const size_t last = name.size() - nlen;
                for (size_t i = 0; i <= last; ++i) {
                    bool match = true;
                    for (size_t j = 0; j < nlen; ++j) {
                        unsigned char a = static_cast<unsigned char>(name[i + j]);
                        unsigned char b = static_cast<unsigned char>(needle[j]);
                        if (std::tolower(a) != b) { match = false; break; }
                    }
                    if (match) return true;
                }
                return false;
            };
            if (containsCI("spirit healer", 13) || containsCI("spirit guide", 12)) {
                continue;
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
        questMarkerRenderer->setMarker(guid, renderPos, markerType, boundingHeight, markerGrayscale);
        markersAdded++;
    }

    if (firstRun && markersAdded > 0) {
        LOG_DEBUG("Quest markers: Added ", markersAdded, " markers on first run");
        firstRun = false;
    }
}

void Application::setupTestTransport() {
    if (!entitySpawner_) return;
    if (entitySpawner_->isTestTransportSetup()) return;
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
        for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
            bool loaded = false;
            for (const std::string& groupPath :
                 pipeline::wmoGroupCandidates(transportWmoPath, gi)) {
                std::vector<uint8_t> groupData = assetManager->readFile(groupPath);
                if (groupData.empty()) continue;
                pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                loadedGroups++;
                loaded = true;
                break;
            }
            if (!loaded) {
                LOG_WARNING("  Failed to load WMO group ", gi, " for: ", transportWmoPath);
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

    entitySpawner_->setTestTransportSetup(true);
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
