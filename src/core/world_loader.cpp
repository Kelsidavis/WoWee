// WorldLoader - terrain streaming, map transitions, world preloading
// Extracted from Application as part of god-class decomposition (Section 3.3)

#include "rendering/animation/melee_anim_chains.hpp"
#include "core/world_loader.hpp"
#include "core/application.hpp"
#include "core/world_entry_callback_handler.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "core/entity_spawner.hpp"
#include "core/appearance_composer.hpp"
#include "core/window.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/quest_marker_renderer.hpp"
#include "rendering/footprint_renderer.hpp"
#include "rendering/loading_screen.hpp"
#include "addons/addon_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/wmo_group_path.hpp"
#include "pipeline/wdt_loader.hpp"
#include "game/game_handler.hpp"
#include "game/transport_manager.hpp"
#include "game/world.hpp"

#include <SDL2/SDL.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <functional>
#include <glm/gtc/quaternion.hpp>

namespace wowee {
namespace core {

namespace {

struct InstancePortalVisual {
    uint32_t mapId;
    glm::vec3 canonicalPos;
    float canonicalYaw;
    float scale;
};

void spawnInstancePortalVisuals(uint32_t mapId,
                                rendering::Renderer* renderer,
                                pipeline::AssetManager* assetManager) {
    if (!renderer || !assetManager || !assetManager->isInitialized()) return;
    auto* m2Renderer = renderer->getM2Renderer();
    if (!m2Renderer) return;

    static constexpr const char* kPortalPath =
        "World\\GENERIC\\ACTIVEDOODADS\\INSTANCEPORTAL\\InstancePortal.m2";
    static constexpr const char* kPortalModelName =
        "World\\GENERIC\\ACTIVEDOODADS\\INSTANCEPORTAL\\InstancePortal.m2#world-entry-glow";
    static const uint32_t kPortalModelId =
        static_cast<uint32_t>(std::hash<std::string>{}(kPortalModelName));

    if (!m2Renderer->hasModel(kPortalModelId)) {
        auto m2Data = assetManager->readFile(kPortalPath);
        if (m2Data.empty()) {
            LOG_WARNING("Instance portal visual unavailable: ", kPortalPath);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        model.name = kPortalModelName;

        const std::string skinPath = pipeline::skinPathForM2(std::string(kPortalPath));
        auto skinData = assetManager->readFile(skinPath);
        if (!skinData.empty() && model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        if (!m2Renderer->loadModel(model, kPortalModelId)) {
            LOG_WARNING("Failed to load instance portal visual model: ", kPortalPath);
            return;
        }
        m2Renderer->markModelAsSpellEffect(kPortalModelId);
    }

    static const InstancePortalVisual kPortals[] = {
        // Ironforge station -> Deeprun Tram
        {.mapId = 0,   .canonicalPos = glm::vec3(-1330.46f, -4840.26f, 503.85f),
         .canonicalYaw = 3.10f, .scale = 4.25f},
        // Deeprun Tram -> Ironforge station
        {.mapId = 369, .canonicalPos = glm::vec3(10.50f, 76.03f, -2.30f),
         .canonicalYaw = 3.12f, .scale = 4.25f},
        // Stormwind station -> Deeprun Tram. Coordinates from a live playthrough's
        // actual AreaTrigger 2173 fire: entrance trigger position on map 0, and the
        // canonical position the player actually landed at on map 369 after transfer
        // (Z lifted by the same ~+2.0 offset the Ironforge map-369 entry uses relative
        // to its own arrival point, since both tunnel ends have similar floor height).
        // Yaw reuses the Ironforge value as a starting point - Stormwind's entrance
        // faces a different direction, so this likely needs live tuning.
        {.mapId = 0,   .canonicalPos = glm::vec3(514.03f, -8346.46f, 97.60f),
         .canonicalYaw = 3.12f, .scale = 4.25f},
        // Deeprun Tram -> Stormwind station
        {.mapId = 369, .canonicalPos = glm::vec3(2490.91f, 68.30f, -2.30f),
         .canonicalYaw = 3.12f, .scale = 4.25f},
        // The Stockade exit. The WMO contains this InstancePortal doodad, but
        // WMO-child matrix instances bypass the standalone portal presentation.
        // Position and scale come from StormwindJail.wmo's authored doodad after
        // applying the instance's 180-degree placement rotation.
        {.mapId = 34,  .canonicalPos = glm::vec3(0.6882f, 45.7492f, -16.1133f),
         .canonicalYaw = 0.0f, .scale = 1.36f},
    };

    for (const auto& portal : kPortals) {
        if (portal.mapId != mapId) continue;

        glm::vec3 renderPos = core::coords::canonicalToRender(portal.canonicalPos);
        float renderYaw = portal.canonicalYaw + glm::radians(90.0f);
        // Each entry keeps the scale appropriate to its authored doorway; the
        // Deeprun entries are intentionally larger because their original effect
        // was too subtle at station scale.
        uint32_t instanceId = m2Renderer->createInstance(
            kPortalModelId, renderPos, glm::vec3(0.0f, 0.0f, renderYaw), portal.scale);
        if (instanceId) {
            m2Renderer->setSkipCollision(instanceId, true);
            LOG_INFO("Spawned instance portal visual map=", mapId,
                     " canonical=(", portal.canonicalPos.x, ", ",
                     portal.canonicalPos.y, ", ", portal.canonicalPos.z, ")");
        }
    }
}

} // namespace

WorldLoader::WorldLoader(Application& app,
                         rendering::Renderer* renderer,
                         pipeline::AssetManager* assetManager,
                         game::GameHandler* gameHandler,
                         EntitySpawner* entitySpawner,
                         AppearanceComposer* appearanceComposer,
                         Window* window,
                         game::World* world,
                         addons::AddonManager* addonManager)
    : app_(app)
    , renderer_(renderer)
    , assetManager_(assetManager)
    , gameHandler_(gameHandler)
    , entitySpawner_(entitySpawner)
    , appearanceComposer_(appearanceComposer)
    , window_(window)
    , world_(world)
    , addonManager_(addonManager)
{}

WorldLoader::~WorldLoader() {
    cancelWorldPreload();
}

const char* WorldLoader::mapDisplayName(uint32_t mapId) {
    // Friendly display names for the loading screen
    switch (mapId) {
        case 0: return "Eastern Kingdoms";
        case 1: return "Kalimdor";
        case 13: return "Test";
        case 34: return "The Stockade";
        case 169: return "Emerald Dream";
        case 530: return "Outland";
        case 571: return "Northrend";
        default: return nullptr;
    }
}

const char* WorldLoader::mapIdToName(uint32_t mapId) {
    // Fallback when Map.dbc is unavailable. Names must match WDT directory names
    // (case-insensitive - AssetManager lowercases all paths).
    switch (mapId) {
        // Continents
        case 0: return "Azeroth";
        case 1: return "Kalimdor";
        case 530: return "Expansion01";
        case 571: return "Northrend";
        // Classic dungeons/raids
        case 30: return "PVPZone01";
        case 33: return "Shadowfang";
        case 34: return "StormwindJail";
        case 36: return "DeadminesInstance";
        case 43: return "WailingCaverns";
        case 47: return "RazserfenKraulInstance";
        case 48: return "Blackfathom";
        case 70: return "Uldaman";
        case 90: return "GnomeragonInstance";
        case 109: return "SunkenTemple";
        case 129: return "RazorfenDowns";
        case 189: return "MonasteryInstances";
        case 209: return "TanarisInstance";
        case 229: return "BlackRockSpire";
        case 230: return "BlackrockDepths";
        case 249: return "OnyxiaLairInstance";
        case 289: return "ScholomanceInstance";
        case 309: return "Zul'Gurub";
        case 329: return "Stratholme";
        case 349: return "Mauradon";
        case 369: return "DeeprunTram";
        case 389: return "OrgrimmarInstance";
        case 409: return "MoltenCore";
        case 429: return "DireMaul";
        case 469: return "BlackwingLair";
        case 489: return "PVPZone03";
        case 509: return "AhnQiraj";
        case 529: return "PVPZone04";
        case 531: return "AhnQirajTemple";
        case 533: return "Stratholme Raid";
        // TBC
        case 532: return "Karazahn";
        case 534: return "HyjalPast";
        case 540: return "HellfireMilitary";
        case 542: return "HellfireDemon";
        case 543: return "HellfireRampart";
        case 544: return "HellfireRaid";
        case 545: return "CoilfangPumping";
        case 546: return "CoilfangMarsh";
        case 547: return "CoilfangDraenei";
        case 548: return "CoilfangRaid";
        case 550: return "TempestKeepRaid";
        case 552: return "TempestKeepArcane";
        case 553: return "TempestKeepAtrium";
        case 554: return "TempestKeepFactory";
        case 555: return "AuchindounShadow";
        case 556: return "AuchindounDraenei";
        case 557: return "AuchindounEthereal";
        case 558: return "AuchindounDemon";
        case 560: return "HillsbradPast";
        case 564: return "BlackTemple";
        case 565: return "GruulsLair";
        case 566: return "PVPZone05";
        case 568: return "ZulAman";
        case 580: return "SunwellPlateau";
        case 585: return "Sunwell5ManFix";
        // WotLK
        case 574: return "Valgarde70";
        case 575: return "UtgardePinnacle";
        case 576: return "Nexus70";
        case 578: return "Nexus80";
        case 595: return "StratholmeCOT";
        case 599: return "Ulduar70";
        case 600: return "Ulduar80";
        case 601: return "DrakTheronKeep";
        case 602: return "GunDrak";
        case 603: return "UlduarRaid";
        case 608: return "DalaranPrison";
        case 615: return "ChamberOfAspectsBlack";
        case 617: return "DeathKnightStart";
        case 619: return "Azjol_Uppercity";
        case 624: return "WintergraspRaid";
        case 631: return "IcecrownCitadel";
        case 632: return "IcecrownCitadel5Man";
        case 649: return "ArgentTournamentRaid";
        case 650: return "ArgentTournamentDungeon";
        case 658: return "QuarryOfTears";
        case 668: return "HallsOfReflection";
        case 724: return "ChamberOfAspectsRed";
        default: return "";
    }
}
void WorldLoader::processPendingEntry() {
    if (!pendingWorldEntry_ || loadingWorld_) return;
    auto entry = *pendingWorldEntry_;
    pendingWorldEntry_.reset();
    LOG_DEBUG("Processing deferred world entry: map ", entry.mapId);
    if (app_.worldEntryCallbacks_) {
        app_.worldEntryCallbacks_->setWorldEntryMovementGraceTimer(2.0f);
        app_.worldEntryCallbacks_->setTaxiLandingClampTimer(0.0f);
        app_.worldEntryCallbacks_->setLastTaxiFlight(false);
    }
    // Clear camera movement inputs before loading terrain
    if (renderer_ && renderer_->getCameraController()) {
        renderer_->getCameraController()->clearMovementInputs();
        renderer_->getCameraController()->suppressMovementFor(1.0f);
        renderer_->getCameraController()->suspendGravityFor(10.0f);
    }
    loadOnlineWorldTerrain(entry.mapId, entry.x, entry.y, entry.z);
}

// Read the map's WDT, then load either its root WMO or its terrain tiles, and
// precompute what the player will stand on.
//
// The longest step of a world load by far, which is why it carries the loading
// screen with it: several of the stages inside take long enough that the window
// would stop answering between them.
void WorldLoader::LoadingUi::pump() const {
    if (!ok || !screen || !window) return;
    screen->render();
    window->swapBuffers();
}

void WorldLoader::loadMapGeometry(uint32_t mapId, const std::string& mapName,
                                  const glm::vec3& spawnCanonical,
                                  const glm::vec3& spawnRender,
                                  const LoadingUi& ui) {
    const auto& showProgress = ui.showProgress;
    // The tile loader below drives the screen itself rather than through pump(),
    // because it also writes the per-tile status line.
    auto* loadingScreen = ui.screen;
    const bool loadingScreenOk = ui.ok;
    // Check WDT to detect WMO-only maps (dungeons, raids, BGs)
    bool isWMOOnlyMap = false;
    pipeline::WDTInfo wdtInfo;
    {
        std::string wdtPath = "World\\Maps\\" + mapName + "\\" + mapName + ".wdt";
        LOG_DEBUG("Reading WDT: ", wdtPath);
        std::vector<uint8_t> wdtData = assetManager_->readFile(wdtPath);
        if (!wdtData.empty()) {
            wdtInfo = pipeline::parseWDT(wdtData);
            isWMOOnlyMap = wdtInfo.isWMOOnly() && !wdtInfo.rootWMOPath.empty();
            LOG_DEBUG("WDT result: isWMOOnly=", isWMOOnlyMap, " rootWMO='", wdtInfo.rootWMOPath, "'");
        } else {
            LOG_WARNING("No WDT file found at ", wdtPath);
        }
    }

    bool terrainOk = false;

    if (isWMOOnlyMap) {
        // ---- WMO-only map (dungeon/raid/BG): load root WMO directly ----
        LOG_DEBUG("WMO-only map detected - loading root WMO: ", wdtInfo.rootWMOPath);
        showProgress("Loading instance geometry...", 0.25f);

        // Initialize renderers if they don't exist yet (first login to a WMO-only map).
        // On map change, renderers already exist from the previous map.
        if (!renderer_->getWMORenderer() || !renderer_->getTerrainManager()) {
            renderer_->initializeRenderers(assetManager_, mapName);
        }

        // Set map name on WMO renderer and disable terrain streaming (no ADT tiles for instances)
        if (renderer_->getWMORenderer()) {
            renderer_->getWMORenderer()->setMapName(mapName);
            renderer_->getWMORenderer()->setWMOOnlyMap(true);
        }
        if (renderer_->getTerrainManager()) {
            renderer_->getTerrainManager()->setStreamingEnabled(false);
        }

        // Spawn player character now that renderers are initialized
        if (!app_.playerCharacterSpawned) {
            app_.spawnPlayerCharacter();
            if (appearanceComposer_) appearanceComposer_->loadEquippedWeapons();
        }

        // Load the root WMO
        auto* wmoRenderer = renderer_->getWMORenderer();
        LOG_DEBUG("WMO-only: wmoRenderer=", (wmoRenderer ? "valid" : "NULL"));
        if (wmoRenderer) {
            LOG_DEBUG("WMO-only: reading root WMO file: ", wdtInfo.rootWMOPath);
            std::vector<uint8_t> wmoData = assetManager_->readFile(wdtInfo.rootWMOPath);
            LOG_DEBUG("WMO-only: root WMO data size=", wmoData.size());
            if (!wmoData.empty()) {
                pipeline::WMOModel wmoModel = pipeline::WMOLoader::load(wmoData);
                LOG_DEBUG("WMO-only: parsed WMO model, nGroups=", wmoModel.nGroups);

                if (wmoModel.nGroups > 0) {
                    showProgress("Loading instance groups...", 0.35f);
                    uint32_t loadedGroups = 0;
                    for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                        for (const std::string& groupPath :
                             pipeline::wmoGroupCandidates(wdtInfo.rootWMOPath, gi)) {
                            std::vector<uint8_t> groupData =
                                assetManager_->readFile(groupPath);
                            if (groupData.empty()) continue;
                            pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                            loadedGroups++;
                            break;
                        }

                        // Update loading progress
                        if (wmoModel.nGroups > 1) {
                            float groupProgress = 0.35f + 0.30f * static_cast<float>(gi + 1) / wmoModel.nGroups;
                            char buf[128];
                            snprintf(buf, sizeof(buf), "Loading instance groups... %u / %u", gi + 1, wmoModel.nGroups);
                            showProgress(buf, groupProgress);
                        }
                    }

                    LOG_INFO("Loaded ", loadedGroups, " / ", wmoModel.nGroups, " WMO groups for instance");
                }

                // WMO-only maps: MODF uses same format as ADT MODF.
                // Apply the same rotation conversion that outdoor WMOs get
                // (including the implicit +180° Z yaw), but skip the ZEROPOINT
                // position offset for zero-position instances (server sends
                // coordinates relative to the WMO, not relative to map corner).
                glm::vec3 wmoPos(0.0f);
                glm::vec3 wmoRot(
                    -wdtInfo.rotation[2] * core::coords::PI / 180.0f,
                    -wdtInfo.rotation[0] * core::coords::PI / 180.0f,
                    (wdtInfo.rotation[1] + 180.0f) * core::coords::PI / 180.0f
                );
                if (wdtInfo.position[0] != 0.0f || wdtInfo.position[1] != 0.0f || wdtInfo.position[2] != 0.0f) {
                    wmoPos = core::coords::adtToWorld(
                        wdtInfo.position[0], wdtInfo.position[1], wdtInfo.position[2]);
                }

                showProgress("Uploading instance geometry...", 0.70f);
                uint32_t wmoModelId = 900000 + mapId;  // Unique ID range for instance WMOs
                if (wmoRenderer->loadModel(wmoModel, wmoModelId)) {
                    uint32_t instanceId = wmoRenderer->createInstance(wmoModelId, wmoPos, wmoRot, 1.0f);
                    if (instanceId > 0) {
                        LOG_DEBUG("Instance WMO loaded: modelId=", wmoModelId,
                                " instanceId=", instanceId);
                        LOG_DEBUG("  MOHD bbox local: (",
                                   wmoModel.boundingBoxMin.x, ", ", wmoModel.boundingBoxMin.y, ", ", wmoModel.boundingBoxMin.z,
                                   ") to (", wmoModel.boundingBoxMax.x, ", ", wmoModel.boundingBoxMax.y, ", ", wmoModel.boundingBoxMax.z, ")");
                        LOG_DEBUG("  WMO pos: (", wmoPos.x, ", ", wmoPos.y, ", ", wmoPos.z,
                                   ") rot: (", wmoRot.x, ", ", wmoRot.y, ", ", wmoRot.z, ")");
                        LOG_DEBUG("  Player render pos: (", spawnRender.x, ", ", spawnRender.y, ", ", spawnRender.z, ")");
                        LOG_DEBUG("  Player canonical: (", spawnCanonical.x, ", ", spawnCanonical.y, ", ", spawnCanonical.z, ")");
                        // Show player position in WMO local space
                        {
                            glm::mat4 instMat(1.0f);
                            instMat = glm::translate(instMat, wmoPos);
                            instMat = glm::rotate(instMat, wmoRot.z, glm::vec3(0,0,1));
                            instMat = glm::rotate(instMat, wmoRot.y, glm::vec3(0,1,0));
                            instMat = glm::rotate(instMat, wmoRot.x, glm::vec3(1,0,0));
                            glm::mat4 invMat = glm::inverse(instMat);
                            glm::vec3 localPlayer = glm::vec3(invMat * glm::vec4(spawnRender, 1.0f));
                            LOG_DEBUG("  Player in WMO local: (", localPlayer.x, ", ", localPlayer.y, ", ", localPlayer.z, ")");
                            bool inside = localPlayer.x >= wmoModel.boundingBoxMin.x && localPlayer.x <= wmoModel.boundingBoxMax.x &&
                                          localPlayer.y >= wmoModel.boundingBoxMin.y && localPlayer.y <= wmoModel.boundingBoxMax.y &&
                                          localPlayer.z >= wmoModel.boundingBoxMin.z && localPlayer.z <= wmoModel.boundingBoxMax.z;
                            LOG_DEBUG("  Player inside MOHD bbox: ", inside ? "YES" : "NO");
                        }

                        // Load doodads from the specified doodad set
                        auto* m2Renderer = renderer_->getM2Renderer();
                        if (m2Renderer && !wmoModel.doodadSets.empty() && !wmoModel.doodads.empty()) {
                            uint32_t setIdx = std::min(static_cast<uint32_t>(wdtInfo.doodadSet),
                                                       static_cast<uint32_t>(wmoModel.doodadSets.size() - 1));
                            const auto& doodadSet = wmoModel.doodadSets[setIdx];

                            showProgress("Loading instance doodads...", 0.75f);
                            glm::mat4 wmoMatrix(1.0f);
                            wmoMatrix = glm::translate(wmoMatrix, wmoPos);
                            wmoMatrix = glm::rotate(wmoMatrix, wmoRot.z, glm::vec3(0, 0, 1));
                            wmoMatrix = glm::rotate(wmoMatrix, wmoRot.y, glm::vec3(0, 1, 0));
                            wmoMatrix = glm::rotate(wmoMatrix, wmoRot.x, glm::vec3(1, 0, 0));

                            uint32_t loadedDoodads = 0;
                            for (uint32_t di = 0; di < doodadSet.count; di++) {
                                uint32_t doodadIdx = doodadSet.startIndex + di;
                                if (doodadIdx >= wmoModel.doodads.size()) break;

                                const auto& doodad = wmoModel.doodads[doodadIdx];
                                auto nameIt = wmoModel.doodadNames.find(doodad.nameIndex);
                                if (nameIt == wmoModel.doodadNames.end()) continue;

                                std::string m2Path = nameIt->second;
                                if (m2Path.empty()) continue;

                                m2Path = pipeline::modelPathToM2(m2Path);

                                std::vector<uint8_t> m2Data = assetManager_->readFile(m2Path);
                                if (m2Data.empty()) continue;

                                pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
                                m2Model.name = m2Path;

                                std::string skinPath = pipeline::skinPathForM2(m2Path);
                                std::vector<uint8_t> skinData = assetManager_->readFile(skinPath);
                                if (!skinData.empty() && m2Model.version >= 264) {
                                    pipeline::M2Loader::loadSkin(skinData, m2Model);
                                }
                                if (!m2Model.isValid()) continue;

                                glm::quat fixedRotation(doodad.rotation.w, doodad.rotation.x,
                                                        doodad.rotation.y, doodad.rotation.z);
                                glm::mat4 doodadLocal(1.0f);
                                doodadLocal = glm::translate(doodadLocal, doodad.position);
                                doodadLocal *= glm::mat4_cast(fixedRotation);
                                doodadLocal = glm::scale(doodadLocal, glm::vec3(doodad.scale));

                                glm::mat4 worldMatrix = wmoMatrix * doodadLocal;
                                glm::vec3 worldPos = glm::vec3(worldMatrix[3]);

                                uint32_t doodadModelId = static_cast<uint32_t>(std::hash<std::string>{}(m2Path));
                                if (!m2Renderer->loadModel(m2Model, doodadModelId)) continue;
                                uint32_t doodadInstId = m2Renderer->createInstanceWithMatrix(doodadModelId, worldMatrix, worldPos);
                                if (doodadInstId) m2Renderer->setSkipWallCollision(doodadInstId, true);
                                loadedDoodads++;
                            }
                            LOG_INFO("Loaded ", loadedDoodads, " instance WMO doodads");
                        }
                    } else {
                        LOG_WARNING("Failed to create instance WMO instance");
                    }
                } else {
                    LOG_WARNING("Failed to load instance WMO model");
                }
            } else {
                LOG_WARNING("Failed to read root WMO file: ", wdtInfo.rootWMOPath);
            }

            // Build collision cache for the instance WMO
            showProgress("Building collision cache...", 0.88f);
            ui.pump();
            wmoRenderer->loadFloorCache();
            if (wmoRenderer->getFloorCacheSize() == 0) {
                showProgress("Computing walkable surfaces...", 0.90f);
                ui.pump();
                wmoRenderer->precomputeFloorCache();
            }
        }

        // Snap player to WMO floor so they don't fall through on first frame
        if (wmoRenderer && renderer_) {
            glm::vec3 playerPos = renderer_->getCharacterPosition();
            // getFloorHeight deliberately rejects probes far above a WMO's bounds.
            // Keep this within its step-up window; a +50 probe made low-ceiling
            // instances such as The Stockade fail their otherwise valid floor hit.
            auto floor = wmoRenderer->getFloorHeight(playerPos.x, playerPos.y, playerPos.z + 5.0f);
            if (floor) {
                playerPos.z = *floor + 0.1f;  // Small offset above floor
                renderer_->getCharacterPosition() = playerPos;
                if (gameHandler_) {
                    glm::vec3 canonical = core::coords::renderToCanonical(playerPos);
                    gameHandler_->setPosition(canonical.x, canonical.y, canonical.z);
                }
                LOG_INFO("Snapped player to instance WMO floor: z=", *floor);
            } else {
                // Fires routinely for this map's floor-query path (correlates with the
                // warmup ground-check below also failing to find a floor here) without
                // visible player impact so far - the server-provided spawn position
                // already lands correctly. Demoted from WARNING; if this map ever does
                // let a player fall through, this and the warmup hard-cap message are
                // the first places to look.
                LOG_DEBUG("Could not find WMO floor at player spawn (",
                           playerPos.x, ", ", playerPos.y, ", ", playerPos.z, ")");
            }
        }

        // Diagnostic: verify WMO renderer state after instance loading
        LOG_DEBUG("=== INSTANCE WMO LOAD COMPLETE ===");
        LOG_DEBUG("  wmoRenderer models loaded: ", wmoRenderer->getLoadedModelCount());
        LOG_DEBUG("  wmoRenderer instances: ", wmoRenderer->getInstanceCount());
        LOG_DEBUG("  wmoRenderer floor cache: ", wmoRenderer->getFloorCacheSize());

        terrainOk = true;  // Mark as OK so post-load setup runs
    } else {
        // ---- Normal ADT-based map ----
        // Compute ADT tile from canonical coordinates
        auto [tileX, tileY] = core::coords::canonicalToTile(spawnCanonical.x, spawnCanonical.y);
        std::string adtPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                              std::to_string(tileX) + "_" + std::to_string(tileY) + ".adt";
        LOG_INFO("Loading ADT tile [", tileX, ",", tileY, "] from canonical (",
                 spawnCanonical.x, ", ", spawnCanonical.y, ", ", spawnCanonical.z, ")");

        // Load the initial terrain tile
        terrainOk = renderer_->loadTestTerrain(assetManager_, adtPath);
        if (!terrainOk) {
            LOG_WARNING("Could not load terrain for online world - atmospheric rendering only");
        } else {
            LOG_INFO("Online world terrain loading initiated");
        }

        // Set map name on WMO renderer (initializeRenderers handles terrain/minimap/worldMap)
        if (renderer_->getWMORenderer()) {
            renderer_->getWMORenderer()->setMapName(mapName);
        }

        // Character renderer is created inside loadTestTerrain(), so spawn the
        // player model now that the renderer actually exists.
        if (!app_.playerCharacterSpawned) {
            app_.spawnPlayerCharacter();
            if (appearanceComposer_) appearanceComposer_->loadEquippedWeapons();
        }

        showProgress("Streaming terrain tiles...", 0.35f);

        // Wait for surrounding terrain tiles to stream in
        if (terrainOk && renderer_->getTerrainManager() && renderer_->getCamera()) {
            auto* terrainMgr = renderer_->getTerrainManager();
            auto* camera = renderer_->getCamera();

            // Use a small radius for the initial load (just immediate tiles),
            // then restore the full radius after entering the game.
            // This matches WoW's behavior: load quickly, stream the rest in-game.
            const int savedLoadRadius = 6;
            terrainMgr->setLoadRadius(4);   // 9x9=81 tiles - prevents hitches on spawn
            terrainMgr->setUnloadRadius(9);

            // Trigger tile streaming for surrounding area
            terrainMgr->update(*camera, 1.0f);

            auto startTime = std::chrono::high_resolution_clock::now();
            auto lastProgressTime = startTime;
            const float maxWaitSeconds = 60.0f;
            const float stallSeconds = 10.0f;
            int initialRemaining = terrainMgr->getRemainingTileCount();
            if (initialRemaining < 1) initialRemaining = 1;
            int lastRemaining = initialRemaining;

            // Wait until all pending + ready-queue tiles are finalized
            while (terrainMgr->getRemainingTileCount() > 0) {
                // This loop presents its own frames but never reaches showProgress,
                // so it must beat the watchdog itself or the whole tile stream reads
                // as a hung main loop.
                app_.beatWatchdog();
                SDL_Event event;
                while (SDL_PollEvent(&event)) {
                    if (event.type == SDL_QUIT) {
                        window_->setShouldClose(true);
                        loadingScreen->shutdown();
                        return;
                    }
                    if (event.type == SDL_WINDOWEVENT &&
                        event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        int w = event.window.data1;
                        int h = event.window.data2;
                        window_->setSize(w, h);
                        // Vulkan viewport set in command buffer
                        if (renderer_->getCamera()) {
                            renderer_->getCamera()->setAspectRatio(static_cast<float>(w) / h);
                        }
                    }
                }

                // Trigger new streaming - enqueue tiles for background workers
                terrainMgr->update(*camera, 0.016f);

                // Process ONE tile per iteration so the progress bar updates
                // smoothly between tiles instead of stalling on large batches.
                terrainMgr->processOneReadyTile();

                int remaining = terrainMgr->getRemainingTileCount();
                int loaded = terrainMgr->getLoadedTileCount();
                int total = loaded + remaining;
                if (total < 1) total = 1;
                float tileProgress = static_cast<float>(loaded) / static_cast<float>(total);
                float progress = 0.35f + tileProgress * 0.50f;

                auto now = std::chrono::high_resolution_clock::now();
                float elapsedSec = std::chrono::duration<float>(now - startTime).count();

                char buf[192];
                if (loaded > 0 && remaining > 0) {
                    float tilesPerSec = static_cast<float>(loaded) / std::max(elapsedSec, 0.1f);
                    float etaSec = static_cast<float>(remaining) / std::max(tilesPerSec, 0.1f);
                    snprintf(buf, sizeof(buf), "Loading terrain... %d / %d tiles (%.0f tiles/s, ~%.0fs remaining)",
                             loaded, total, tilesPerSec, etaSec);
                } else {
                    snprintf(buf, sizeof(buf), "Loading terrain... %d / %d tiles",
                             loaded, total);
                }

                if (loadingScreenOk) {
                    loadingScreen->setStatus(buf);
                    loadingScreen->setProgress(progress);
                    loadingScreen->render();
                    window_->swapBuffers();
                }

                if (remaining != lastRemaining) {
                    lastRemaining = remaining;
                    lastProgressTime = now;
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

                // Don't sleep if there are more tiles to finalize - keep processing
                if (remaining > 0 && terrainMgr->getReadyQueueCount() == 0) {
                    SDL_Delay(16);
                }
            }

            LOG_INFO("Online terrain streaming complete: ", terrainMgr->getLoadedTileCount(), " tiles loaded");

            // Restore full load radius - remaining tiles stream in-game
            terrainMgr->setLoadRadius(savedLoadRadius);

            // Load/precompute collision cache
            if (renderer_->getWMORenderer()) {
                showProgress("Building collision cache...", 0.88f);
                ui.pump();
                renderer_->getWMORenderer()->loadFloorCache();
                if (renderer_->getWMORenderer()->getFloorCacheSize() == 0) {
                    showProgress("Computing walkable surfaces...", 0.90f);
                    ui.pump();
                    renderer_->getWMORenderer()->precomputeFloorCache();
                }
            }
        }
    }
}

void WorldLoader::loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z) {
    if (!renderer_ || !assetManager_ || !assetManager_->isInitialized()) {
        LOG_WARNING("Cannot load online terrain: renderer or assets not ready");
        return;
    }

    // Guard against re-entrant calls. The worldEntryCallback defers new
    // entries while this flag is set; we process them at the end.
    loadingWorld_ = true;
    pendingWorldEntry_.reset();

    // --- Loading screen for online mode ---
    rendering::LoadingScreen loadingScreen;
    loadingScreen.setVkContext(window_->getVkContext());
    loadingScreen.setSDLWindow(window_->getSDLWindow());
    bool loadingScreenOk = loadingScreen.initialize();

    auto showProgress = [&](const char* msg, float progress) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                window_->setShouldClose(true);
                loadingScreen.shutdown();
                return;
            }
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_RESIZED) {
                int w = event.window.data1;
                int h = event.window.data2;
                window_->setSize(w, h);
                // Vulkan viewport set in command buffer
                if (renderer_ && renderer_->getCamera()) {
                    renderer_->getCamera()->setAspectRatio(static_cast<float>(w) / h);
                }
            }
        }
        // The world load blocks the main loop for over a second while presenting its
        // own frames. Beat the watchdog so it does not read a healthy load as a hang
        // and force-release the player's mouse capture.
        app_.beatWatchdog();
        if (!loadingScreenOk) return;
        loadingScreen.setStatus(msg);
        loadingScreen.setProgress(progress);
        loadingScreen.render();
        window_->swapBuffers();
    };

    // Everything a slow step needs to keep the screen alive.
    LoadingUi loadingUi;
    loadingUi.screen = &loadingScreen;
    loadingUi.window = window_;
    loadingUi.ok = loadingScreenOk;
    loadingUi.showProgress = showProgress;

    // Set zone name on loading screen - prefer friendly display name, then DBC
    {
        const char* friendly = mapDisplayName(mapId);
        if (friendly) {
            loadingScreen.setZoneName(friendly);
        } else if (gameHandler_) {
            std::string dbcName = gameHandler_->getMapName(mapId);
            if (!dbcName.empty())
                loadingScreen.setZoneName(dbcName);
            else
                loadingScreen.setZoneName("Loading...");
        }
    }

    showProgress("Entering world...", 0.0f);

    // --- Clean up previous map's state on map change ---
    // (Same cleanup as logout, but preserves player identity and renderer objects.)
    LOG_DEBUG("loadOnlineWorldTerrain: mapId=", mapId, " loadedMapId_=", loadedMapId_);
    bool hasRendererData = renderer_ && (renderer_->getWMORenderer() || renderer_->getM2Renderer());
    if (loadedMapId_ != 0xFFFFFFFF || hasRendererData) {
        LOG_WARNING("Map change: cleaning up old map ", loadedMapId_, " before loading map ", mapId);

        // Clear pending queues first (these don't touch GPU resources)
        entitySpawner_->clearAllQueues();

        // Transport GUIDs are reused across maps (for example, Undercity elevator
        // GUIDs become Deeprun tram GUIDs). The renderer instances are map-local,
        // so the logical transport/path state must be map-local too; otherwise a
        // newly spawned SubwayCar is rebound to the old elevator path.
        if (gameHandler_) {
            gameHandler_->clearPlayerTransport();
            if (auto* tm = gameHandler_->getTransportManager()) {
                tm->clearTransports();
            }
        }

        if (renderer_) {
            // Clear all world geometry from old map (including textures/models).
            // WMO clearAll and M2 clear both call vkDeviceWaitIdle internally,
            // ensuring no GPU command buffers reference old resources.
            if (auto* wmo = renderer_->getWMORenderer()) {
                wmo->clearAll();
            }
            if (auto* m2 = renderer_->getM2Renderer()) {
                m2->clear();
            }

            // Full clear of character renderer: removes all instances, models,
            // textures, and resets descriptor pools. This prevents stale GPU
            // resources from accumulating across map changes (old creature
            // models, bone buffers, texture descriptor sets) which can cause
            // VK_ERROR_DEVICE_LOST on some drivers.
            if (auto* cr = renderer_->getCharacterRenderer()) {
                cr->clear();
                renderer_->setCharacterFollow(0);
            }
            // Reset equipment dirty tracking so composited textures are rebuilt
            // after spawnPlayerCharacter() recreates the character instance.
            if (gameHandler_) {
                gameHandler_->resetEquipmentDirtyTracking();
            }

            if (auto* terrain = renderer_->getTerrainManager()) {
                terrain->softReset();
                terrain->setStreamingEnabled(true);  // Re-enable in case previous map disabled it
            }
            if (auto* questMarkers = renderer_->getQuestMarkerRenderer()) {
                questMarkers->clear();
            }
            if (auto* footprints = renderer_->getFootprintRenderer()) {
                footprints->clear();
            }
            if (auto* ac = renderer_->getAnimationController()) ac->clearMount();
        }

        // Clear application-level instance tracking (after renderer cleanup)
        entitySpawner_->resetAllState();

        // Force player character re-spawn on new map
        app_.playerCharacterSpawned = false;
    }

    // Resolve map folder name from Map.dbc (authoritative for world/instance maps).
    // This is required for instances like DeeprunTram (map 369) that are not Azeroth/Kalimdor.
    if (!mapNameCacheLoaded_ && assetManager_ && assetManager_->isInitialized()) {
        mapNameCacheLoaded_ = true;
        if (auto mapDbc = assetManager_->loadDBC("Map.dbc"); mapDbc && mapDbc->isLoaded()) {
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

    // Cancel any stale preload (if it was for a different map, the file cache
    // still retains whatever was loaded - it doesn't hurt).
    if (worldPreload_) {
        if (worldPreload_->mapId == mapId) {
            LOG_INFO("World preload: cache-warm hit for map '", mapName, "'");
        } else {
            LOG_INFO("World preload: map mismatch (preloaded ", worldPreload_->mapName,
                     ", entering ", mapName, ")");
        }
    }
    cancelWorldPreload();

    // Save this world info for next session's early preload
    saveLastWorldInfo(mapId, mapName, x, y);

    // Convert server coordinates to canonical WoW coordinates
    // Server sends: X=West (canonical.Y), Y=North (canonical.X), Z=Up
    glm::vec3 spawnCanonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
    glm::vec3 spawnRender = core::coords::canonicalToRender(spawnCanonical);

    // Set camera position and facing from server orientation
    float spawnYawDeg = 0.0f;
    if (gameHandler_) {
        float canonicalYaw = gameHandler_->getMovementInfo().orientation;
        spawnYawDeg = core::coords::canonicalToCharacterYawDeg(canonicalYaw);
    }
    if (renderer_->getCameraController()) {
        renderer_->getCameraController()->setOnlineMode(true);
        renderer_->getCameraController()->setDefaultSpawn(spawnRender, spawnYawDeg, -15.0f);
        renderer_->getCameraController()->reset();
    }
    renderer_->setCharacterYaw(spawnYawDeg);

    // Set map name for WMO renderer and reset instance mode
    if (renderer_->getWMORenderer()) {
        renderer_->getWMORenderer()->setMapName(mapName);
        renderer_->getWMORenderer()->setWMOOnlyMap(false);
    }

    // Set map name for terrain manager
    if (renderer_->getTerrainManager()) {
        renderer_->getTerrainManager()->setMapName(mapName);
    }

    // NOTE: TransportManager renderer connection moved to after initializeRenderers (later in this function)

    // Connect WMORenderer to M2Renderer (for hierarchical transforms: doodads following WMO parents)
    if (renderer_->getWMORenderer() && renderer_->getM2Renderer()) {
        renderer_->getWMORenderer()->setM2Renderer(renderer_->getM2Renderer());
        LOG_INFO("WMORenderer connected to M2Renderer for hierarchical doodad transforms");
    }

    showProgress("Loading character model...", 0.05f);

    // Build faction hostility map for this character's race
    if (gameHandler_) {
        const game::Character* activeChar = gameHandler_->getActiveCharacter();
        if (activeChar) {
            app_.buildFactionHostilityMap(static_cast<uint8_t>(activeChar->race));
        }
    }

    // Spawn player model for online mode (skip if already spawned, e.g. teleport)
    if (gameHandler_) {
        const game::Character* activeChar = gameHandler_->getActiveCharacter();
        if (activeChar) {
            const uint64_t activeGuid = gameHandler_->getActiveCharacterGuid();
            const bool appearanceChanged =
                (activeGuid != app_.spawnedPlayerGuid_) ||
                (activeChar->appearanceBytes != app_.spawnedAppearanceBytes_) ||
                (activeChar->facialFeatures != app_.spawnedFacialFeatures_) ||
                (activeChar->race != app_.playerRace_) ||
                (activeChar->gender != app_.playerGender_) ||
                (activeChar->characterClass != app_.playerClass_);

            if (!app_.playerCharacterSpawned || appearanceChanged) {
                if (appearanceChanged) {
                    LOG_INFO("Respawning player model for new/changed character: guid=0x",
                             std::hex, activeGuid, std::dec);
                }
                // Remove old instance so we don't keep stale visuals.
                if (renderer_ && renderer_->getCharacterRenderer()) {
                    uint32_t oldInst = renderer_->getCharacterInstanceId();
                    if (oldInst > 0) {
                        renderer_->setCharacterFollow(0);
                        if (auto* ac = renderer_->getAnimationController()) ac->clearMount();
                        renderer_->getCharacterRenderer()->removeInstance(oldInst);
                    }
                }
                app_.playerCharacterSpawned = false;
                app_.spawnedPlayerGuid_ = 0;
                app_.spawnedAppearanceBytes_ = 0;
                app_.spawnedFacialFeatures_ = 0;

                app_.playerRace_ = activeChar->race;
                app_.playerGender_ = activeChar->gender;
                app_.playerClass_ = activeChar->characterClass;
                app_.spawnSnapToGround = false;
                if (appearanceComposer_) appearanceComposer_->setWeaponsSheathed(false);
                if (appearanceComposer_) appearanceComposer_->loadEquippedWeapons(); // will no-op until instance exists
                app_.spawnPlayerCharacter();
            }
            renderer_->getCharacterPosition() = spawnRender;
            LOG_INFO("Online player at render pos (", spawnRender.x, ", ", spawnRender.y, ", ", spawnRender.z, ")");
        } else {
            LOG_WARNING("No active character found for player model spawning");
        }
    }

    showProgress("Loading terrain...", 0.20f);

    // The map's geometry, and the loading screen it takes with it.
    loadMapGeometry(mapId, mapName, spawnCanonical, spawnRender, loadingUi);

    // Snap player to loaded terrain so they don't spawn underground
    if (renderer_->getCameraController()) {
        renderer_->getCameraController()->reset();
    }
    renderer_->setCharacterYaw(spawnYawDeg);
    spawnInstancePortalVisuals(mapId, renderer_, assetManager_);

    // Test transport disabled - real transports come from server via UPDATEFLAG_TRANSPORT
    showProgress("Finalizing world...", 0.94f);
    // setupTestTransport();

    // Connect TransportManager to renderers (must happen AFTER initializeRenderers)
    if (gameHandler_ && gameHandler_->getTransportManager()) {
        auto* tm = gameHandler_->getTransportManager();
        if (renderer_->getWMORenderer()) tm->setWMORenderer(renderer_->getWMORenderer());
        if (renderer_->getM2Renderer()) tm->setM2Renderer(renderer_->getM2Renderer());
        LOG_DEBUG("TransportManager connected: wmoR=", (renderer_->getWMORenderer() ? "yes" : "NULL"),
                   " m2R=", (renderer_->getM2Renderer() ? "yes" : "NULL"));
    }

    // Set up NPC animation callbacks (for online creatures)
    showProgress("Preparing creatures...", 0.97f);
    if (gameHandler_ && renderer_ && renderer_->getCharacterRenderer()) {
        auto* cr = renderer_->getCharacterRenderer();
        auto* spawner = entitySpawner_;

        gameHandler_->setNpcDeathCallback([cr, spawner](uint64_t guid) {
            spawner->markCreatureDead(guid);
            uint32_t instanceId = spawner->getCreatureInstanceId(guid);
            if (instanceId == 0) instanceId = spawner->getPlayerInstanceId(guid);
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, rendering::anim::DEATH, false);
            }
        });

        gameHandler_->setNpcRespawnCallback([cr, spawner](uint64_t guid) {
            spawner->unmarkCreatureDead(guid);
            uint32_t instanceId = spawner->getCreatureInstanceId(guid);
            if (instanceId == 0) instanceId = spawner->getPlayerInstanceId(guid);
            if (instanceId != 0 && cr) {
                cr->playAnimation(instanceId, rendering::anim::STAND, true);
            }
        });

        // Probe the creature model for the best available attack animation
        gameHandler_->setNpcSwingCallback([cr, spawner](uint64_t guid) {
            uint32_t instanceId = spawner->getCreatureInstanceId(guid);
            if (instanceId == 0) instanceId = spawner->getPlayerInstanceId(guid);
            if (instanceId != 0 && cr) {
                const auto attackAnims =
                rendering::anim::meleeAnimChain(rendering::anim::MeleeChain::Generic);
                bool played = false;
                for (uint32_t anim : attackAnims) {
                    if (cr->hasAnimation(instanceId, anim)) {
                        cr->playAnimation(instanceId, anim, false);
                        played = true;
                        break;
                    }
                }
                if (!played) cr->playAnimation(instanceId, rendering::anim::ATTACK_UNARMED, false);
            }
        });
    }

    // Keep the loading screen visible until all spawn/equipment/gameobject queues
    // are fully drained. This ensures the player sees a fully populated world
    // (character clothed, NPCs placed, game objects loaded) when the screen drops.
    {
        const float kMinWarmupSeconds = 2.0f;   // minimum time to drain network packets
        const float kMaxWarmupSeconds = 25.0f;  // hard cap to avoid infinite stall
        const auto warmupStart = std::chrono::high_resolution_clock::now();
        // Track consecutive idle iterations (all queues empty) to detect convergence
        int idleIterations = 0;
        const int kIdleThreshold = 5;  // require 5 consecutive empty loops (~80ms)
        // Throttle for the "ground not ready" debug log below - must be a fresh local
        // (not static), since a static here would retain its value across separate
        // warmup calls (e.g. different map loads) and could suppress logging on a
        // later call for a long time after an earlier call ran close to the hard cap.
        float lastGroundNotReadyLogTime = -1000.0f;

        while (true) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    window_->setShouldClose(true);
                    if (loadingScreenOk) loadingScreen.shutdown();
                    return;
                }
                if (event.type == SDL_WINDOWEVENT &&
                    event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int w = event.window.data1;
                    int h = event.window.data2;
                    window_->setSize(w, h);
                    if (renderer_ && renderer_->getCamera()) {
                        renderer_->getCamera()->setAspectRatio(static_cast<float>(w) / h);
                    }
                }
            }

            // Drain network and process deferred spawn/composite queues while hidden.
            if (gameHandler_) gameHandler_->update(1.0f / 60.0f);

            // If a new world entry was deferred during packet processing,
            // stop warming up this map - we'll load the new one after cleanup.
            if (pendingWorldEntry_) {
                LOG_DEBUG("loadOnlineWorldTerrain(map ", mapId,
                            ") - deferred world entry pending, stopping warmup");
                break;
            }

            if (world_) world_->update(1.0f / 60.0f);

            // Process all spawn/equipment/transport queues during warmup
            entitySpawner_->update();
            if (auto* cr = renderer_ ? renderer_->getCharacterRenderer() : nullptr) {
                cr->processPendingNormalMaps(4);
            }
            app_.updateQuestMarkers();

            // Update renderer (terrain streaming, animations)
            if (renderer_) {
                renderer_->update(1.0f / 60.0f);
            }

            const auto now = std::chrono::high_resolution_clock::now();
            const float elapsed = std::chrono::duration<float>(now - warmupStart).count();

            // Check if all queues are drained
            bool queuesEmpty = !entitySpawner_->hasWorkPending();

            if (queuesEmpty) {
                idleIterations++;
            } else {
                idleIterations = 0;
            }

            // Don't exit warmup until the ground under the player exists.
            // In cities like Stormwind, players stand on WMO floors, not terrain.
            // Check BOTH terrain AND WMO floor - require at least one to be valid.
            bool groundReady = false;
            if (renderer_) {
                // spawnRender already went server -> canonical -> render above.
                // Applying canonicalToRender directly to the server tuple here
                // swapped X/Y a second time, so WMO-only instances checked for a
                // floor at the wrong location and eventually released at hard cap.
                float rx = spawnRender.x, ry = spawnRender.y, rz = spawnRender.z;

                // Check WMO floor FIRST (cities like Stormwind stand on WMO floors).
                // Terrain exists below WMOs but at the wrong height.
                if (auto* wmo = renderer_->getWMORenderer()) {
                    auto wmoH = wmo->getFloorHeight(rx, ry, rz + 5.0f);
                    if (wmoH.has_value() && std::abs(*wmoH - rz) < 15.0f) {
                        groundReady = true;
                    }
                }
                // Check terrain - but only if it's close to spawn Z (within 15 units).
                // Terrain far below a WMO city doesn't count as ground.
                if (!groundReady) {
                    if (auto* tm = renderer_->getTerrainManager()) {
                        auto tH = tm->getHeightAt(rx, ry);
                        if (tH.has_value() && std::abs(*tH - rz) < 15.0f) {
                            groundReady = true;
                        }
                    }
                }
                // After 5s with enough tiles loaded, accept terrain as ready even if
                // the height sample doesn't match spawn Z exactly. This handles cases
                // where getHeightAt returns a slightly different value than the server's
                // spawn Z (e.g. terrain LOD, MCNK chunk boundaries, or spawn inside a
                // building where floor height differs from terrain below).
                if (!groundReady && elapsed >= 5.0f) {
                    if (auto* tm = renderer_->getTerrainManager()) {
                        if (tm->getLoadedTileCount() >= 4) {
                            groundReady = true;
                            LOG_DEBUG("Warmup: using tile-count fallback (", tm->getLoadedTileCount(), " tiles) after ", elapsed, "s");
                        }
                    }
                }

                // Routine during a normal load window, not exceptional by itself - only
                // the hard-cap-with-ground-still-not-ready case below is worth WARNING.
                // The old throttle condition (elapsed*2 % 3 == 0) looked like "once every
                // ~1.5s" but int-truncation actually held it true for a continuous 0.5s
                // window every 1.5s, so it fired on nearly every frame in that window -
                // explains the burst-then-gap pattern seen live. Use a real elapsed-time
                // gate instead.
                if (!groundReady && elapsed > 5.0f && elapsed - lastGroundNotReadyLogTime >= 1.5f) {
                    lastGroundNotReadyLogTime = elapsed;
                    LOG_DEBUG("Warmup: ground not ready at spawn (", rx, ",", ry, ",", rz,
                                ") after ", elapsed, "s");
                }
            }

            // Exit when: (min time passed AND queues drained AND ground ready) OR hard cap
            bool readyToExit = (elapsed >= kMinWarmupSeconds && idleIterations >= kIdleThreshold && groundReady);
            if (readyToExit || elapsed >= kMaxWarmupSeconds) {
                if (elapsed >= kMaxWarmupSeconds && !groundReady) {
                    LOG_WARNING("Warmup hit hard cap (", kMaxWarmupSeconds, "s), ground NOT ready - may fall through world");
                } else if (elapsed >= kMaxWarmupSeconds) {
                    LOG_WARNING("Warmup hit hard cap (", kMaxWarmupSeconds, "s), entering world with pending work");
                }
                break;
            }

            const float t = std::clamp(elapsed / kMaxWarmupSeconds, 0.0f, 1.0f);
            showProgress("Finalizing world sync...", 0.97f + t * 0.025f);
            SDL_Delay(16);
        }
    }

    // Start intro pan right before entering gameplay so it's visible after loading.
    if (renderer_->getCameraController()) {
        renderer_->getCameraController()->startIntroPan(2.8f, 140.0f);
    }

    showProgress("Entering world...", 1.0f);

    // Ensure all GPU resources (textures, buffers, pipelines) created during
    // world load are fully flushed before the first render frame. Without this,
    // vkCmdBeginRenderPass can crash on NVIDIA 590.x when resources from async
    // uploads haven't completed their queue operations.
    if (renderer_ && renderer_->getVkContext()) {
        vkDeviceWaitIdle(renderer_->getVkContext()->getDevice());
    }

    if (loadingScreenOk) {
        loadingScreen.shutdown();
    }

    // Track which map we actually loaded (used by same-map teleport check).
    loadedMapId_ = mapId;

    // Clear loading flag and process any deferred world entry.
    // A deferred entry occurs when SMSG_NEW_WORLD arrived during our warmup
    // (e.g., an area trigger in a dungeon immediately teleporting the player out).
    loadingWorld_ = false;
    if (pendingWorldEntry_) {
        auto entry = *pendingWorldEntry_;
        pendingWorldEntry_.reset();
        LOG_DEBUG("Processing deferred world entry: map ", entry.mapId);
        if (app_.worldEntryCallbacks_) {
            app_.worldEntryCallbacks_->setWorldEntryMovementGraceTimer(2.0f);
            app_.worldEntryCallbacks_->setTaxiLandingClampTimer(0.0f);
            app_.worldEntryCallbacks_->setLastTaxiFlight(false);
        }
        // Recursive call - sets loadedMapId_ and IN_GAME state for the final map.
        loadOnlineWorldTerrain(entry.mapId, entry.x, entry.y, entry.z);
        return;  // The recursive call handles setState(IN_GAME).
    }

    // Only enter IN_GAME when this is the final map (no deferred entry pending).
    app_.setState(AppState::IN_GAME);

    // Load addons once per session on first world entry
    if (addonManager_ && !app_.addonsLoaded_) {
        // Set character name for per-character SavedVariables
        if (gameHandler_) {
            const std::string& charName = gameHandler_->lookupName(gameHandler_->getPlayerGuid());
            if (!charName.empty()) {
                addonManager_->setCharacterName(charName);
            } else {
                // Fallback: find name from character list
                for (const auto& c : gameHandler_->getCharacters()) {
                    if (c.guid == gameHandler_->getPlayerGuid()) {
                        addonManager_->setCharacterName(c.name);
                        break;
                    }
                }
            }
        }
        addonManager_->loadAllAddons();
        app_.addonsLoaded_ = true;
        addonManager_->fireEvent("VARIABLES_LOADED");
        // After VARIABLES_LOADED, which is when the chat window
        // settings are available to be read. Every chat frame
        // answers this by applying its saved font, colour, size
        // and docked state; without it they keep the defaults
        // they were built with however much was saved.
        addonManager_->fireEvent("UPDATE_CHAT_WINDOWS");
        addonManager_->fireEvent("PLAYER_LOGIN");
        addonManager_->fireEvent("PLAYER_ENTERING_WORLD");
    } else if (addonManager_ && app_.addonsLoaded_) {
        // Subsequent world entries (e.g. teleport, instance entry)
        addonManager_->fireEvent("PLAYER_ENTERING_WORLD");
    }
}

void WorldLoader::startWorldPreload(uint32_t mapId, const std::string& mapName,
                                     float serverX, float serverY) {
    cancelWorldPreload();
    if (!assetManager_ || !assetManager_->isInitialized() || mapName.empty()) return;

    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, 0.0f));
    auto [tileX, tileY] = core::coords::canonicalToTile(canonical.x, canonical.y);

    worldPreload_ = std::make_unique<WorldPreload>();
    worldPreload_->mapId = mapId;
    worldPreload_->mapName = mapName;
    worldPreload_->centerTileX = tileX;
    worldPreload_->centerTileY = tileY;

    LOG_INFO("World preload: starting for map '", mapName, "' tile [", tileX, ",", tileY, "]");

    // Build list of tiles to preload (radius 1 = 3x3 = 9 tiles, matching load screen)
    struct TileJob { int x, y; };
    auto jobs = std::make_shared<std::vector<TileJob>>();
    // Center tile first (most important)
    jobs->push_back({tileX, tileY});
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int tx = tileX + dx, ty = tileY + dy;
            if (tx < 0 || tx > 63 || ty < 0 || ty > 63) continue;
            jobs->push_back({tx, ty});
        }
    }

    // Spawn worker threads (one per tile for maximum parallelism)
    auto cancelFlag = &worldPreload_->cancel;
    auto* am = assetManager_;
    std::string mn = mapName;

    int numWorkers = std::min(static_cast<int>(jobs->size()), 4);
    auto nextJob = std::make_shared<std::atomic<int>>(0);

    for (int w = 0; w < numWorkers; w++) {
        worldPreload_->workers.emplace_back([am, mn, jobs, nextJob, cancelFlag]() {
            while (!cancelFlag->load(std::memory_order_relaxed)) {
                int idx = nextJob->fetch_add(1, std::memory_order_relaxed);
                if (idx >= static_cast<int>(jobs->size())) break;

                int tx = (*jobs)[idx].x;
                int ty = (*jobs)[idx].y;

                // Read ADT file (warms file cache)
                std::string adtPath = "World\\Maps\\" + mn + "\\" + mn + "_" +
                                      std::to_string(tx) + "_" + std::to_string(ty) + ".adt";
                // Discarded on purpose: the read is for the cache it warms.
                (void)am->readFile(adtPath);
                if (cancelFlag->load(std::memory_order_relaxed)) break;

                // Read obj0 variant
                std::string objPath = "World\\Maps\\" + mn + "\\" + mn + "_" +
                                      std::to_string(tx) + "_" + std::to_string(ty) + "_obj0.adt";
                (void)am->readFile(objPath);
            }
            LOG_DEBUG("World preload worker finished");
        });
    }
}

void WorldLoader::cancelWorldPreload() {
    if (!worldPreload_) return;
    worldPreload_->cancel.store(true, std::memory_order_relaxed);
    for (auto& t : worldPreload_->workers) {
        if (t.joinable()) t.join();
    }
    LOG_INFO("World preload: cancelled (map=", worldPreload_->mapName,
             " tile=[", worldPreload_->centerTileX, ",", worldPreload_->centerTileY, "])");
    worldPreload_.reset();
}

void WorldLoader::saveLastWorldInfo(uint32_t mapId, const std::string& mapName,
                                     float serverX, float serverY) {
#ifdef _WIN32
    const char* base = std::getenv("APPDATA");
    std::string dir = base ? std::string(base) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    std::string dir = home ? std::string(home) + "/.wowee" : ".";
#endif
    std::filesystem::create_directories(dir);
    std::ofstream f(dir + "/last_world.cfg");
    if (f) {
        f << mapId << "\n" << mapName << "\n" << serverX << "\n" << serverY << "\n";
    }
}

WorldLoader::LastWorldInfo WorldLoader::loadLastWorldInfo() const {
#ifdef _WIN32
    const char* base = std::getenv("APPDATA");
    std::string dir = base ? std::string(base) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    std::string dir = home ? std::string(home) + "/.wowee" : ".";
#endif
    LastWorldInfo info;
    std::ifstream f(dir + "/last_world.cfg");
    if (!f) return info;
    std::string line;
    try {
        if (std::getline(f, line)) info.mapId = static_cast<uint32_t>(std::stoul(line));
        if (std::getline(f, line)) info.mapName = line;
        if (std::getline(f, line)) info.x = std::stof(line);
        if (std::getline(f, line)) info.y = std::stof(line);
    } catch (...) {
        LOG_WARNING("Malformed last_world.cfg, ignoring saved position");
        return info;
    }
    info.valid = !info.mapName.empty();
    return info;
}

} // namespace core
} // namespace wowee
