// world_map_facade.cpp - Public API for the world map system.
// Composes all extracted components and orchestrates the world map (Phase 10).
#include "rendering/world_map/world_map_facade.hpp"
#include "rendering/world_map/data_repository.hpp"
#include "rendering/world_map/view_state_machine.hpp"
#include "rendering/world_map/composite_renderer.hpp"
#include "rendering/world_map/exploration_state.hpp"
#include "rendering/world_map/zone_metadata.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "core/coordinates.hpp"
#include "rendering/world_map/map_resolver.hpp"
#include "rendering/world_map/overlay_renderer.hpp"
#include "rendering/world_map/input_handler.hpp"
#include "rendering/world_map/layers/player_marker_layer.hpp"
#include "rendering/world_map/layers/party_dot_layer.hpp"
#include "rendering/world_map/layers/taxi_node_layer.hpp"
#include "rendering/world_map/layers/poi_marker_layer.hpp"
#include "rendering/world_map/layers/quest_poi_layer.hpp"
#include "rendering/world_map/layers/corpse_marker_layer.hpp"
#include "rendering/world_map/layers/rare_tracker_layer.hpp"
#include "rendering/world_map/layers/chest_tracker_layer.hpp"
#include "rendering/world_map/layers/zone_highlight_layer.hpp"
#include "rendering/world_map/layers/coordinate_display.hpp"
#include "rendering/world_map/layers/subzone_tooltip_layer.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "ui/ui_colors.hpp"
#include "game/game_utils.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <cmath>
#include <algorithm>

namespace wowee {
namespace rendering {
namespace world_map {

// Find the zone index for the WORLD view background.
// Find the best continent root zone for displaying a map in CONTINENT view.
// Skips synthetic zones (Cosmic, World) and prefers a zone matching mapName.
static int findContinentRootIdx(const std::vector<Zone>& zones,
                                 int cosmicIdx,
                                 int worldIdx,
                                 const std::string& mapName) {
    LOG_INFO("findContinentRootIdx: searching ", zones.size(), " zones, mapName='", mapName, "'");
    // 1) Exact areaName match for the map name (e.g. "Azeroth", "Kalimdor")
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx || i == worldIdx) continue;
        if (zones[i].areaID == 0 && zones[i].areaName == mapName) {
            LOG_INFO("findContinentRootIdx: matched mapName '", mapName, "' at zone[", i, "]");
            return i;
        }
    }
    // 2) Root continent (parent of leaf continents)
    int firstContinent = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx || i == worldIdx) continue;
        if (zones[i].areaID == 0) {
            if (firstContinent < 0) firstContinent = i;
            if (isRootContinent(zones, i)) return i;
        }
    }
    // 3) First continent entry
    return firstContinent;
}

// Find the best zone for the WORLD view (prefers synthetic "World" zone).
// Used only as a fallback when data.worldIdx() is not available.
static int findWorldViewContinentIdx(const std::vector<Zone>& zones,
                                      int cosmicIdx,
                                      const std::string& mapName) {
    LOG_INFO("findWorldViewContinentIdx: searching ", zones.size(), " zones, cosmicIdx=", cosmicIdx, " mapName='", mapName, "'");
    // 1) Exact areaName match for "World" folder
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx) continue;
        if (zones[i].areaID == 0 && zones[i].areaName == "World") {
            LOG_INFO("findWorldViewContinentIdx: matched 'World' at zone[", i, "]");
            return i;
        }
    }
    // 2) Exact areaName match for the map name (e.g. "Azeroth")
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx) continue;
        if (zones[i].areaID == 0 && zones[i].areaName == mapName) {
            LOG_INFO("findWorldViewContinentIdx: matched mapName '", mapName, "' at zone[", i, "]");
            return i;
        }
    }
    // 3) Root continent (parent of leaf continents)
    int firstContinent = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx) continue;
        if (zones[i].areaID == 0) {
            if (firstContinent < 0) firstContinent = i;
            if (isRootContinent(zones, i)) return i;
        }
    }
    // 4) First continent entry
    return firstContinent;
}

// ── PIMPL Implementation ─────────────────────────────────────

struct WorldMapFacade::Impl {
    VkContext* vkCtx = nullptr;
    pipeline::AssetManager* assetManager = nullptr;
    bool initialized = false;
    bool open = false;
    // The zone the server says the player is in (SMSG_INIT_WORLD_STATES). 0 until
    // it has said. Preferred over guessing from WorldMapArea boxes, which overlap
    // their neighbours enough to open the map on a zone the player is merely near.
    uint32_t playerZoneId = 0;
    std::string mapName = "Azeroth";
    std::string physicalMapName = "Azeroth";
    bool virtualMapOverride = false;
    std::string pendingMapName;   // stored by external setMapName while in world/cosmic view
    bool userMapOverride = false; // true when user manually navigated to world/cosmic view
    // Set the map back to where the player is standing, on the next render.
    // The interface asks for this every time the map is shown, and the flow
    // that works it out is the one the first open already runs - so this
    // re-enters that rather than keeping a second copy of it here.
    bool recenterOnPlayer = false;
    bool viewChanged = false;

    DataRepository data;
    ViewStateMachine viewState;
    CompositeRenderer compositor;
    ExplorationState exploration;
    ZoneMetadata zoneMetadata;
    InputHandler input;
    OverlayRenderer overlay;

    // Typed layer pointers for setters (non-owning references into overlay)
    PartyDotLayer* partyDotLayer = nullptr;
    std::vector<RareMark> rares;
    std::vector<ChestMark> chests;
    TaxiNodeLayer* taxiNodeLayer = nullptr;
    POIMarkerLayer* poiMarkerLayer = nullptr;
    QuestPOILayer* questPOILayer = nullptr;
    CorpseMarkerLayer* corpseMarkerLayer = nullptr;
    RareTrackerLayer* rareTrackerLayer = nullptr;
    ChestTrackerLayer* chestTrackerLayer = nullptr;
    ZoneHighlightLayer* zoneHighlightLayer = nullptr;
    PlayerMarkerLayer* playerMarkerLayer = nullptr;

    // Data set each frame from the UI layer
    std::vector<PartyDot> partyDots;
    std::vector<TaxiNode> taxiNodes;
    std::vector<QuestPOI> questPois;

    // Flight-map (taxi selection) mode - locks the view to the continent and
    // renders only the map, player marker, and interactive flight nodes.
    bool taxiMode = false;

    // Where a host frame wants the map drawn, in pixels from the top-left.
    // Lives here rather than on the facade because the drawing does.
    bool  hasFrameRect = false;
    float frameRectX = 0.0f, frameRectY = 0.0f;
    float frameRectW = 0.0f, frameRectH = 0.0f;
    std::function<void()> taxiCloseHandler;

    float lastFrameTime = 0.0f;

    void initOverlayLayers();
    void closeMap();
    void switchToMap(const std::string& newMapName);
    void switchToWorldView();
    void renderImGuiOverlay(const glm::vec3& playerRenderPos,
                            int screenWidth, int screenHeight,
                            float playerYawDeg,
                            bool rightClickConsumed);
};

void WorldMapFacade::Impl::closeMap() {
    open = false;
    userMapOverride = false;
    // Apply any map name that was deferred while in world/cosmic view.
    if (!pendingMapName.empty()) {
        mapName = pendingMapName;
        pendingMapName.clear();
    }
    // A user-dismissed flight map also closes the flight master window.
    if (taxiMode) {
        taxiMode = false;
        if (taxiNodeLayer) taxiNodeLayer->setTaxiMode(false);
        if (taxiCloseHandler) taxiCloseHandler();
        taxiCloseHandler = nullptr;
    }
}

void WorldMapFacade::Impl::switchToMap(const std::string& newMapName) {
    if (mapName == newMapName && !data.zones().empty()) return;
    userMapOverride = true;
    pendingMapName.clear();
    if (zoneHighlightLayer) zoneHighlightLayer->clearTextures();
    compositor.detachZoneTextures();
    data.clear();
    compositor.invalidateComposite();

    mapName = newMapName;
    data.loadZones(mapName, *assetManager);
    zoneMetadata.initialize();
    viewState.setCosmicEnabled(data.cosmicEnabled());

    // Find the continent root zone and display it (skip synthetic World/Cosmic)
    int rootIdx = findContinentRootIdx(data.zones(), data.cosmicIdx(), data.worldIdx(), mapName);
    if (rootIdx < 0) rootIdx = 0;
    viewState.setContinentIdx(rootIdx);
    compositor.loadZoneTextures(rootIdx, data.zones(), mapName);
    compositor.requestComposite(rootIdx);
    viewState.setCurrentZoneIdx(rootIdx);
    viewState.setLevel(ViewLevel::CONTINENT);
}

void WorldMapFacade::Impl::switchToWorldView() {
    LOG_INFO("switchToWorldView: mapName='", mapName, "'");

    // Determine whether the current map is an Azeroth continent (EK, Kalimdor,
    // Northrend) or a separate world (Outland).  Azeroth continents go to the
    // world view; other worlds go to the cosmic view.
    bool isAzerothContinent = (mapName == "Azeroth");
    if (!isAzerothContinent) {
        int curMapId = folderToMapId(mapName);
        for (const auto& region : data.azerothRegions()) {
            if (static_cast<int>(region.mapId) == curMapId) {
                isAzerothContinent = true;
                break;
            }
        }
    }

    // If on a different map, switch back to Azeroth first.
    if (mapName != "Azeroth") {
        if (zoneHighlightLayer) zoneHighlightLayer->clearTextures();
        compositor.detachZoneTextures();
        data.clear();
        compositor.invalidateComposite();
        mapName = "Azeroth";
        data.loadZones(mapName, *assetManager);
        zoneMetadata.initialize();
        viewState.setCosmicEnabled(data.cosmicEnabled());
    }
    userMapOverride = true;

    // Non-Azeroth worlds (e.g. Outland) go to cosmic view.
    if (!isAzerothContinent && viewState.cosmicEnabled() && data.cosmicIdx() >= 0) {
        viewState.enterCosmicView();
        compositor.loadZoneTextures(data.cosmicIdx(), data.zones(), mapName);
        compositor.requestComposite(data.cosmicIdx());
        viewState.setCurrentZoneIdx(data.cosmicIdx());
        return;
    }

    viewState.enterWorldView();

    // Use the dedicated synthetic "World" zone - its tiles (world1-12.blp)
    // are cached independently from zone[0] (Azeroth), avoiding stale-tile
    // conflicts when transitioning between WORLD and CONTINENT views.
    int worldIdx = data.worldIdx();
    LOG_INFO("switchToWorldView: worldIdx=", worldIdx);
    if (worldIdx >= 0) {
        compositor.loadZoneTextures(worldIdx, data.zones(), mapName);
        if (compositor.hasAnyTile(worldIdx)) {
            compositor.invalidateComposite();
            compositor.requestComposite(worldIdx);
            viewState.setCurrentZoneIdx(worldIdx);
            return;
        }
    }

    // Fallback: try the root continent zone
    int rootIdx = findWorldViewContinentIdx(data.zones(), data.cosmicIdx(), mapName);
    LOG_INFO("switchToWorldView: fallback rootIdx=", rootIdx);
    if (rootIdx >= 0) {
        compositor.loadZoneTextures(rootIdx, data.zones(), mapName);
        if (compositor.hasAnyTile(rootIdx)) {
            compositor.invalidateComposite();
            compositor.requestComposite(rootIdx);
            viewState.setCurrentZoneIdx(rootIdx);
        }
    }
}

void WorldMapFacade::Impl::initOverlayLayers() {
    // Order matters: later layers draw on top of earlier ones

    // Zone highlights (continent view)
    auto zhLayer = std::make_unique<ZoneHighlightLayer>();
    zhLayer->setMetadata(&zoneMetadata);
    zoneHighlightLayer = zhLayer.get();
    overlay.addLayer(std::move(zhLayer));

    // Player marker
    auto pmLayer = std::make_unique<PlayerMarkerLayer>();
    playerMarkerLayer = pmLayer.get();
    overlay.addLayer(std::move(pmLayer));

    // Party dots
    auto pdLayer = std::make_unique<PartyDotLayer>();
    partyDotLayer = pdLayer.get();
    overlay.addLayer(std::move(pdLayer));

    // Taxi nodes
    auto tnLayer = std::make_unique<TaxiNodeLayer>();
    taxiNodeLayer = tnLayer.get();
    overlay.addLayer(std::move(tnLayer));

    // // POI markers
    // auto poiLayer = std::make_unique<POIMarkerLayer>();
    // poiMarkerLayer = poiLayer.get();
    // overlay.addLayer(std::move(poiLayer));

    // Quest POI markers
    auto qpLayer = std::make_unique<QuestPOILayer>();
    questPOILayer = qpLayer.get();
    overlay.addLayer(std::move(qpLayer));

    // Corpse marker
    auto cmLayer = std::make_unique<CorpseMarkerLayer>();
    corpseMarkerLayer = cmLayer.get();
    overlay.addLayer(std::move(cmLayer));

    // Nearby rare/rare-elite tracker
    auto rtLayer = std::make_unique<RareTrackerLayer>();
    rareTrackerLayer = rtLayer.get();
    overlay.addLayer(std::move(rtLayer));

    // Nearby spawned chest tracker
    auto ctLayer = std::make_unique<ChestTrackerLayer>();
    chestTrackerLayer = ctLayer.get();
    overlay.addLayer(std::move(ctLayer));

    // Coordinate display
    overlay.addLayer(std::make_unique<CoordinateDisplay>());

    // Subzone tooltip
    overlay.addLayer(std::make_unique<SubzoneTooltipLayer>());
}

// ── WorldMapFacade Public Methods ────────────────────────────

WorldMapFacade::WorldMapFacade() : impl_(std::make_unique<Impl>()) {
    impl_->zoneMetadata.initialize();
    impl_->initOverlayLayers();
}

WorldMapFacade::~WorldMapFacade() {
    shutdown();
}

bool WorldMapFacade::initialize(VkContext* ctx, pipeline::AssetManager* am) {
    impl_->vkCtx = ctx;
    impl_->assetManager = am;
    if (!impl_->compositor.initialize(ctx, am)) return false;
    if (impl_->zoneHighlightLayer)
        impl_->zoneHighlightLayer->initialize(ctx, am);
    if (impl_->playerMarkerLayer)
        impl_->playerMarkerLayer->initialize(ctx, am);
    if (impl_->corpseMarkerLayer)
        impl_->corpseMarkerLayer->initialize(ctx, am);
    impl_->initialized = true;
    return true;
}

void WorldMapFacade::shutdown() {
    if (!impl_) return;
    if (impl_->zoneHighlightLayer)
        impl_->zoneHighlightLayer->clearTextures();
    if (impl_->corpseMarkerLayer)
        impl_->corpseMarkerLayer->clearTexture();
    impl_->compositor.shutdown();
    impl_->data.clear();
    impl_->initialized = false;
}

void WorldMapFacade::compositePass(VkCommandBuffer cmd) {
    impl_->compositor.flushStaleTextures();
    impl_->compositor.compositePass(cmd,
                                     impl_->data.zones(),
                                     impl_->exploration.exploredOverlays(),
                                     impl_->exploration.hasServerMask());
}

void WorldMapFacade::render(const glm::vec3& playerRenderPos,
                             int screenWidth, int screenHeight,
                             float playerYawDeg) {
    auto& d = *impl_;
    if (!d.initialized || !d.assetManager) return;

    // Update transition animation
    float now = static_cast<float>(ImGui::GetTime());
    float dt = now - d.lastFrameTime;
    d.lastFrameTime = now;
    d.viewState.updateTransition(dt);

    const int physicalMapId = folderToMapId(d.physicalMapName);
    auto displayedPlayerPosition = [&]() {
        return physicalMapId >= 0
            ? d.data.transformRenderPosition(
                  static_cast<uint32_t>(physicalMapId), playerRenderPos)
            : playerRenderPos;
    };
    glm::vec3 displayPlayerRenderPos = displayedPlayerPosition();

    // Update exploration state
    if (!d.data.zones().empty()) {
        d.exploration.update(d.data.zones(), displayPlayerRenderPos,
                             d.viewState.currentZoneIdx(),
                             d.data.exploreFlagByAreaId(), d.playerZoneId);
        if (d.exploration.overlaysChanged() && d.viewState.currentZoneIdx() >= 0) {
            d.compositor.invalidateComposite();
            d.compositor.requestComposite(d.viewState.currentZoneIdx());
        }
    }

    // First-time open, zones lost after a map change, or the interface asking
    // to go back to the player's own zone
    if (!d.open || d.data.zones().empty() || d.recenterOnPlayer) {
        d.open = true;
        d.recenterOnPlayer = false;
        if (d.data.zones().empty()) {
            d.data.loadZones(d.mapName, *d.assetManager);
            d.zoneMetadata.initialize();
            d.viewState.setCosmicEnabled(d.data.cosmicEnabled());
            displayPlayerRenderPos = displayedPlayerPosition();
        }

        int playerZone = findZoneForPlayer(d.data.zones(), displayPlayerRenderPos, d.playerZoneId);

        // Some zones are stored on a different physical map from the continent
        // shown by the world-map UI. The draenei islands are the important case:
        // terrain/minimap data comes from Expansion01 (map 530), while their
        // WorldMapArea DisplayMapID is Kalimdor (map 1). Follow that metadata
        // instead of opening Outland merely because the terrain map is 530.
        if (!d.virtualMapOverride && playerZone >= 0) {
            const Zone& zone = d.data.zones()[playerZone];
            if (zone.displayMapID != 0 &&
                zone.displayMapID != static_cast<uint32_t>(d.data.currentMapId())) {
                const char* virtualFolder = mapIdToFolder(zone.displayMapID);
                if (virtualFolder && *virtualFolder) {
                    d.physicalMapName = d.mapName;
                    d.virtualMapOverride = true;
                    d.mapName = virtualFolder;
                    if (d.zoneHighlightLayer) d.zoneHighlightLayer->clearTextures();
                    d.compositor.detachZoneTextures();
                    d.data.clear();
                    d.compositor.invalidateComposite();
                    d.data.loadZones(d.mapName, *d.assetManager);
                    d.zoneMetadata.initialize();
                    d.viewState.setCosmicEnabled(d.data.cosmicEnabled());
                    d.viewState.setContinentIdx(-1);
                    d.viewState.setCurrentZoneIdx(-1);
                    displayPlayerRenderPos = displayedPlayerPosition();
                    playerZone = findZoneForPlayer(d.data.zones(), displayPlayerRenderPos, d.playerZoneId);
                    LOG_INFO("World map virtual continent: physical='",
                             d.physicalMapName, "' display='", d.mapName, "'");
                }
            }
        } else if (d.virtualMapOverride) {
            const bool stillInVirtualArea = playerZone >= 0 &&
                d.data.zones()[playerZone].mapID !=
                    static_cast<uint32_t>(d.data.currentMapId()) &&
                d.data.zones()[playerZone].displayMapID ==
                    static_cast<uint32_t>(d.data.currentMapId());
            if (!stillInVirtualArea) {
                d.virtualMapOverride = false;
                d.mapName = d.physicalMapName;
                if (d.zoneHighlightLayer) d.zoneHighlightLayer->clearTextures();
                d.compositor.detachZoneTextures();
                d.data.clear();
                d.compositor.invalidateComposite();
                d.data.loadZones(d.mapName, *d.assetManager);
                d.zoneMetadata.initialize();
                d.viewState.setCosmicEnabled(d.data.cosmicEnabled());
                d.viewState.setContinentIdx(-1);
                d.viewState.setCurrentZoneIdx(-1);
                displayPlayerRenderPos = displayedPlayerPosition();
                playerZone = findZoneForPlayer(d.data.zones(), displayPlayerRenderPos, d.playerZoneId);
            }
        }

        int bestContinent = findBestContinentForPlayer(d.data.zones(), displayPlayerRenderPos);
        if (bestContinent >= 0 && bestContinent != d.viewState.continentIdx()) {
            d.viewState.setContinentIdx(bestContinent);
            d.compositor.invalidateComposite();
        }

        // Flight map is always the continent overview - never open at zone level.
        if (!d.taxiMode &&
            playerZone >= 0 && d.viewState.continentIdx() >= 0 &&
            zoneBelongsToContinent(d.data.zones(), playerZone, d.viewState.continentIdx())) {
            d.compositor.loadZoneTextures(playerZone, d.data.zones(), d.mapName);
            d.compositor.loadOverlayTextures(playerZone, d.data.zones());
            d.viewState.setCurrentZoneIdx(playerZone);
            d.viewState.setLevel(ViewLevel::ZONE);
            d.exploration.update(d.data.zones(), displayPlayerRenderPos, playerZone,
                                 d.data.exploreFlagByAreaId(), d.playerZoneId);
            d.compositor.requestComposite(playerZone);
        } else if (d.viewState.continentIdx() >= 0) {
            d.compositor.loadZoneTextures(d.viewState.continentIdx(), d.data.zones(), d.mapName);
            d.compositor.requestComposite(d.viewState.continentIdx());
            d.viewState.setCurrentZoneIdx(d.viewState.continentIdx());
            d.viewState.setLevel(ViewLevel::CONTINENT);
        }
        // The view has only now become the player's zone, and the interface
        // asked before it did.
        //
        // SetMapToCurrentZone runs on every open and fires WORLD_MAP_UPDATE on
        // the spot, which is when both dropdowns rebuild themselves. All this
        // call does here is raise the recenter flag - the move happens in this
        // block, one frame later - so GetCurrentMapContinent answered 0 while
        // the dropdowns were being built, and 0 is what makes watchframe's
        // ClearAll branch run and GetMapZones(0) return an empty list. Both
        // came up blank and stayed blank, because nothing asked again.
        d.viewChanged = true;
    }

    // Process input
    int hoveredZone = d.zoneHighlightLayer ? d.zoneHighlightLayer->hoveredZone() : -1;
    InputResult inputResult = d.input.process(d.viewState.currentLevel(),
                                               hoveredZone,
                                               d.viewState.cosmicEnabled());

    // Flight map is locked to the continent view: node clicks are handled by
    // the taxi layer, so only closing the map is a valid input action here.
    if (d.taxiMode && inputResult.action != InputAction::CLOSE) {
        inputResult.action = InputAction::NONE;
    }

    switch (inputResult.action) {
        case InputAction::CLOSE:
            d.closeMap();
            return;

        case InputAction::ZOOM_IN: {
            int playerZone = findZoneForPlayer(d.data.zones(), displayPlayerRenderPos, d.playerZoneId);
            // For continent→zone, verify the zone belongs to the current continent
            int candidateZone = hoveredZone >= 0 ? hoveredZone : playerZone;
            if (d.viewState.currentLevel() == ViewLevel::CONTINENT &&
                candidateZone >= 0 &&
                !zoneBelongsToContinent(d.data.zones(), candidateZone, d.viewState.continentIdx())) {
                candidateZone = -1;
            }
            // Bug fix: also validate playerZone against the continent so the
            // fallback inside zoomIn() doesn't navigate to the wrong continent.
            int validPlayerZone = playerZone;
            if (d.viewState.currentLevel() == ViewLevel::CONTINENT &&
                validPlayerZone >= 0 &&
                !zoneBelongsToContinent(d.data.zones(), validPlayerZone, d.viewState.continentIdx())) {
                validPlayerZone = -1;
            }
            auto zr = d.viewState.zoomIn(candidateZone, validPlayerZone);
            if (zr.changed && zr.targetIdx >= 0) {
                d.compositor.loadZoneTextures(zr.targetIdx, d.data.zones(), d.mapName);
                if (zr.newLevel == ViewLevel::ZONE) {
                    d.compositor.loadOverlayTextures(zr.targetIdx, d.data.zones());
                }
                d.compositor.requestComposite(zr.targetIdx);
            } else if (zr.changed && zr.newLevel == ViewLevel::WORLD) {
                d.switchToWorldView();
            }
            break;
        }

        case InputAction::ZOOM_OUT: {
            auto zr = d.viewState.zoomOut();
            if (zr.changed && zr.targetIdx >= 0) {
                d.compositor.loadZoneTextures(zr.targetIdx, d.data.zones(), d.mapName);
                d.compositor.requestComposite(zr.targetIdx);
            } else if (zr.changed && zr.newLevel == ViewLevel::WORLD) {
                d.switchToWorldView();
            } else if (zr.changed && zr.newLevel == ViewLevel::COSMIC) {
                if (d.data.cosmicIdx() >= 0) {
                    d.compositor.loadZoneTextures(d.data.cosmicIdx(), d.data.zones(), d.mapName);
                    d.compositor.requestComposite(d.data.cosmicIdx());
                    d.viewState.setCurrentZoneIdx(d.data.cosmicIdx());
                }
            }
            break;
        }

        case InputAction::CLICK_ZONE: {
            int hz = inputResult.targetIdx;
            if (hz >= 0) {
                // Use centralized resolver to handle cross-map zone navigation
                auto zoneResult = resolveZoneClick(hz, d.data.zones(), d.data.currentMapId());
                switch (zoneResult.action) {
                    case MapResolveAction::LOAD_MAP:
                        d.switchToMap(zoneResult.targetMapName);
                        break;
                    case MapResolveAction::ENTER_ZONE:
                        d.compositor.loadZoneTextures(hz, d.data.zones(), d.mapName);
                        d.compositor.loadOverlayTextures(hz, d.data.zones());
                        d.compositor.requestComposite(hz);
                        d.viewState.enterZone(hz);
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case InputAction::RIGHT_CLICK_BACK: {
            // Only process right-click if we're at zone or continent level
            if (d.viewState.currentLevel() == ViewLevel::ZONE &&
                d.viewState.continentIdx() >= 0) {
                d.compositor.loadZoneTextures(d.viewState.continentIdx(), d.data.zones(), d.mapName);
                d.compositor.requestComposite(d.viewState.continentIdx());
                d.viewState.setCurrentZoneIdx(d.viewState.continentIdx());
                d.viewState.setLevel(ViewLevel::CONTINENT);
            } else if (d.viewState.currentLevel() == ViewLevel::CONTINENT) {
                d.switchToWorldView();
            } else if (d.viewState.currentLevel() == ViewLevel::WORLD &&
                       d.viewState.cosmicEnabled()) {
                d.viewState.enterCosmicView();
                if (d.data.cosmicIdx() >= 0) {
                    d.compositor.loadZoneTextures(d.data.cosmicIdx(), d.data.zones(), d.mapName);
                    d.compositor.requestComposite(d.data.cosmicIdx());
                    d.viewState.setCurrentZoneIdx(d.data.cosmicIdx());
                }
            }
            break;
        }

        default:
            break;
    }

    if (!d.open) return;
    bool rightClickConsumed = (inputResult.action == InputAction::RIGHT_CLICK_BACK);
    d.renderImGuiOverlay(displayPlayerRenderPos, screenWidth, screenHeight,
                         playerYawDeg, rightClickConsumed);
}

void WorldMapFacade::setMapName(const std::string& name) {
    auto& d = *impl_;
    // While the user has manually navigated to the world/cosmic overview,
    // remember the game's desired map but don't reset the view.
    if (d.userMapOverride) {
        d.pendingMapName = name;
        return;
    }
    // The terrain/minimap continues reporting the physical map (Expansion01)
    // while Azuremyst is intentionally displayed under Kalimdor. Do not undo
    // that virtual-continent selection every frame.
    if (d.virtualMapOverride && name == d.physicalMapName) return;
    d.physicalMapName = name;
    d.virtualMapOverride = false;
    if (d.mapName == name && !d.data.zones().empty()) return;
    d.mapName = name;

    if (d.zoneHighlightLayer)
        d.zoneHighlightLayer->clearTextures();
    d.compositor.detachZoneTextures();
    d.data.clear();
    d.viewState.setContinentIdx(-1);
    d.viewState.setCurrentZoneIdx(-1);
    d.compositor.invalidateComposite();
    d.viewState.setLevel(ViewLevel::WORLD);
    d.open = false;
}

void WorldMapFacade::setServerExplorationMask(const std::vector<uint32_t>& masks, bool hasData) {
    impl_->exploration.setServerMask(masks, hasData);
}

int WorldMapFacade::zoneAtMapPoint(float u, float v) const {
    if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) return -1;
    auto& d = *impl_;
    if (!d.data.hasZmpData()) return -1;

    const int continentIdx = d.viewState.continentIdx();
    if (continentIdx < 0) return -1;

    // The point is on the continent map, so it turns into world coordinates
    // through that continent's own projection bounds.
    float left = 0, right = 0, top = 0, bottom = 0;
    if (!getContinentProjectionBounds(d.data.zones(), continentIdx,
                                      left, right, top, bottom)) {
        return -1;
    }
    const float denomU = left - right;
    const float denomV = top - bottom;
    if (std::abs(denomU) < 0.001f || std::abs(denomV) < 0.001f) return -1;

    const float wowX = left - u * denomU;
    const float wowY = top  - v * denomV;

    // World coordinates into the ZMP's own grid, which covers 64 by 64 ADTs
    // with the world's centre at its middle.
    constexpr float kWorldSize = 64.0f * core::coords::TILE_SIZE;
    const float zmpX = 0.5f - wowX / kWorldSize;
    const float zmpY = 0.5f - wowY / kWorldSize;
    if (zmpX < 0.0f || zmpX > 1.0f || zmpY < 0.0f || zmpY > 1.0f) return -1;

    const int col = std::clamp(static_cast<int>(zmpX * 128.0f), 0, 127);
    const int row = std::clamp(static_cast<int>(zmpY * 128.0f), 0, 127);
    const uint32_t areaId = d.data.zmpGrid()[static_cast<size_t>(row) * 128 + col];
    if (areaId == 0) return -1;

    const int zoneIdx = d.data.zoneIndexForAreaId(areaId);
    if (zoneIdx < 0) return -1;
    // A point can land on a zone of a different continent at the edges.
    if (!zoneBelongsToContinent(d.data.zones(), zoneIdx, continentIdx)) return -1;
    return zoneIdx;
}

std::string WorldMapFacade::zoneNameAtMapPoint(float u, float v) const {
    const int idx = zoneAtMapPoint(u, v);
    const auto& zones = impl_->data.zones();
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return {};
    const uint32_t areaId = zones[static_cast<size_t>(idx)].areaID;
    const auto& names = impl_->data.areaNameByAreaId();
    auto it = names.find(areaId);
    // The DBC's own area name, not the texture folder the zone is keyed by -
    // those differ, and the folder is not what anyone wants to read.
    if (it != names.end()) return it->second;
    return zones[static_cast<size_t>(idx)].areaName;
}

bool WorldMapFacade::clickMapPoint(float u, float v) {
    const int idx = zoneAtMapPoint(u, v);
    if (idx < 0) return false;
    impl_->viewState.zoomIn(idx, /*playerZoneIdx=*/-1);
    return true;
}

void WorldMapFacade::zoomOutOneLevel() {
    impl_->viewState.zoomOut();
}

std::vector<OverlayEntry> WorldMapFacade::currentOverlays() const {
    const int idx = impl_->viewState.currentZoneIdx();
    const auto& zones = impl_->data.zones();
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return {};
    // A continent has no overlay art of its own; its detail is the child
    // zones' maps, which the interface reaches by drilling in.
    if (zones[static_cast<size_t>(idx)].areaID == 0) return {};
    return zones[static_cast<size_t>(idx)].overlays;
}
bool WorldMapFacade::mapUVForCanonical(float wowX, float wowY, float wowZ,
                                       float& u, float& v) const {
    const int idx = impl_->viewState.currentZoneIdx();
    const auto& zones = impl_->data.zones();
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return false;
    const Zone& zone = zones[static_cast<size_t>(idx)];
    const bool isContinent = (zone.areaID == 0);

    // The same bounds every marker layer projects against.
    bool boundsAreContinent = false;
    const ZoneBounds bounds = projectionBoundsFor(
        zones, impl_->viewState.continentIdx(), boundsAreContinent);
    const glm::vec2 uv = renderPosToMapUV(
        core::coords::canonicalToRender(glm::vec3(wowX, wowY, wowZ)),
        bounds, isContinent);
    if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) return false;
    u = uv.x;
    v = uv.y;
    return true;
}

std::vector<WorldMapFacade::Landmark> WorldMapFacade::currentLandmarks() const {
    const int idx = impl_->viewState.currentZoneIdx();
    const auto& zones = impl_->data.zones();
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return {};
    const Zone& zone = zones[static_cast<size_t>(idx)];
    const bool isContinent = (zone.areaID == 0);

    std::vector<Landmark> out;
    for (const POI& poi : impl_->data.poiMarkers()) {
        if (poi.mapId != zone.mapID) continue;
        // The POI carries canonical WoW coordinates and the projection wants
        // render space, which is the same swap every other caller makes.
        const glm::vec2 uv = renderPosToMapUV(
            core::coords::canonicalToRender(glm::vec3(poi.wowX, poi.wowY, poi.wowZ)),
            zone.bounds, isContinent);
        // Outside the map it is on some other part of the continent, and a pin
        // clamped to the edge is worse than no pin.
        if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) continue;
        Landmark mark;
        mark.name = poi.name;
        mark.description = poi.description;
        mark.iconType = poi.iconType;
        mark.x = uv.x;
        mark.y = uv.y;
        out.push_back(std::move(mark));
    }
    return out;
}

void WorldMapFacade::setPlayerZoneId(uint32_t zoneId) {
    impl_->playerZoneId = zoneId;
}

void WorldMapFacade::setPartyDots(std::vector<PartyDot> dots) {
    impl_->partyDots = std::move(dots);
}

void WorldMapFacade::setRares(std::vector<RareMark> rares) {
    impl_->rares = std::move(rares);
}

void WorldMapFacade::setChests(std::vector<ChestMark> chests) {
    impl_->chests = std::move(chests);
}

void WorldMapFacade::setTaxiNodes(std::vector<TaxiNode> nodes) {
    impl_->taxiNodes = std::move(nodes);
}

void WorldMapFacade::setQuestPois(std::vector<QuestPOI> pois) {
    impl_->questPois = std::move(pois);
}

void WorldMapFacade::setFrameRect(float x, float y, float w, float h) {
    impl_->frameRectX = x; impl_->frameRectY = y;
    impl_->frameRectW = w; impl_->frameRectH = h;
    impl_->hasFrameRect = true;
}

void WorldMapFacade::clearFrameRect() { impl_->hasFrameRect = false; }
bool WorldMapFacade::hasFrameRect() const { return impl_->hasFrameRect; }

void WorldMapFacade::setCorpsePos(bool hasCorpse, glm::vec3 renderPos) {
    if (impl_->corpseMarkerLayer)
        impl_->corpseMarkerLayer->setCorpse(hasCorpse, renderPos);
}

void WorldMapFacade::setGraveyardPos(bool hasGraveyard, glm::vec3 renderPos) {
    if (impl_->corpseMarkerLayer)
        impl_->corpseMarkerLayer->setGraveyard(hasGraveyard, renderPos);
}

bool WorldMapFacade::isOpen() const { return impl_->open; }
void WorldMapFacade::close() {
    impl_->closeMap();
}

void WorldMapFacade::openTaxiMap(std::function<std::vector<uint32_t>(uint32_t)> routeProvider,
                                 std::function<void(uint32_t)> onSelect,
                                 std::function<void()> onClose) {
    auto& d = *impl_;
    if (d.taxiNodeLayer) {
        d.taxiNodeLayer->setTaxiMode(true);
        d.taxiNodeLayer->setTaxiHandlers(std::move(routeProvider), std::move(onSelect));
    }
    d.taxiCloseHandler = std::move(onClose);
    d.taxiMode = true;
    d.userMapOverride = false;
    // Force the first-open flow on the next render so the view snaps to the
    // player's continent regardless of where the map was last left.
    d.open = false;
}

void WorldMapFacade::closeTaxiMap() {
    auto& d = *impl_;
    if (!d.taxiMode) return;
    d.taxiMode = false;
    d.taxiCloseHandler = nullptr;
    if (d.taxiNodeLayer) d.taxiNodeLayer->setTaxiMode(false);
    d.open = false;
}

bool WorldMapFacade::isTaxiMapOpen() const {
    return impl_->taxiMode;
}

// ── ImGui Overlay ────────────────────────────────────────────

void WorldMapFacade::Impl::renderImGuiOverlay(const glm::vec3& playerRenderPos,
                                                int screenWidth, int screenHeight,
                                                float playerYawDeg,
                                                bool rightClickConsumed) {
    float sw = static_cast<float>(screenWidth);
    float sh = static_cast<float>(screenHeight);

    // Use the visible WoW map area (1002×668) for aspect ratio.
    float mapAspect = static_cast<float>(CompositeRenderer::MAP_W) /
                      static_cast<float>(CompositeRenderer::MAP_H);
    float availW = sw * 0.70f;
    float availH = sh * 0.70f;
    float displayW, displayH;
    if (availW / availH > mapAspect) {
        displayH = availH;
        displayW = availH * mapAspect;
    } else {
        displayW = availW;
        displayH = availW / mapAspect;
    }

    // Floor to pixel boundary
    displayW = std::floor(displayW);
    displayH = std::floor(displayH);

    // Account for the ImGui title bar so the content area matches the map
    float titleBarH = ImGui::GetFrameHeight();
    float windowW = displayW;
    float windowH = displayH + titleBarH;
    float mapX = std::floor((sw - windowW) / 2.0f);
    float mapY = std::floor((sh - windowH) / 2.0f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoScrollWithMouse |
                             ImGuiWindowFlags_NoFocusOnAppearing;

    // Hosted inside a frame someone else drew: fill exactly that rect, keep no
    // title bar of its own, and follow the frame rather than staying where it
    // was first put - the frame is what moves now.
    if (hasFrameRect) {
        mapX = frameRectX;
        mapY = frameRectY;
        windowW = frameRectW;
        windowH = frameRectH;
        flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoBackground;
        ImGui::SetNextWindowPos(ImVec2(mapX, mapY), ImGuiCond_Always);
    } else {
        // Map window - styled like the character selection window
        ImGui::SetNextWindowPos(ImVec2(mapX, mapY), ImGuiCond_Once);
    }
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    // Keep ImGui's close state separate so clicking the title-bar X can use
    // the same cleanup path as Escape instead of mutating open directly.
    // The ### suffix keeps one window identity across both titles.
    bool windowOpen = open;
    const char* windowTitle = taxiMode ? "Flight Map###WorldMapWindow"
                                       : "World Map###WorldMapWindow";
    if (ImGui::Begin(windowTitle, &windowOpen, flags)) {
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        // imgMin/imgMax = the content area (after title bar)
        const float contentLocalY = ImGui::GetCursorPosY();
        ImVec2 contentPos = ImGui::GetCursorScreenPos();
        ImVec2 contentSize = ImGui::GetContentRegionAvail();
        ImVec2 imgMin = contentPos;
        ImVec2 imgMax(contentPos.x + contentSize.x, contentPos.y + contentSize.y);
        displayW = contentSize.x;
        displayH = contentSize.y;
        // Show only the visible 1002×668 content region of the 1024×768 FBO.
        //
        // Not before the first composite has run. The image is created and
        // never written until then, so opening the map sampled whatever its
        // memory happened to hold for the one frame between the window
        // appearing and the pass that fills it - which is the magenta flash.
        // A zone change is not this case: invalidateComposite() only says the
        // picture is stale, and a stale map for one frame is the right thing
        // to show.
        if (compositor.everComposited()) {
            ImGui::Image(
                reinterpret_cast<ImTextureID>(compositor.displayDescriptorSet()),
                ImVec2(displayW, displayH),
                ImVec2(0, 0), ImVec2(CompositeRenderer::MAP_U_MAX,
                                     CompositeRenderer::MAP_V_MAX));
        } else {
            // The colour the composite pass clears to, so the first frame is
            // the map's own background rather than a hole.
            ImGui::Dummy(ImVec2(displayW, displayH));
            drawList->AddRectFilled(imgMin, imgMax, IM_COL32(13, 20, 31, 255));
        }

        // Transition fade overlay
        const auto& trans = viewState.transition();
        if (trans.active) {
            float alpha = std::max(0.0f, 1.0f - trans.progress);
            if (alpha > 0.01f) {
                uint8_t fadeAlpha = static_cast<uint8_t>(alpha * 180.0f);
                drawList->AddRectFilled(imgMin, imgMax,
                                        IM_COL32(0, 0, 0, fadeAlpha));
            }
        }



        // Build continent index list (expansion-aware filtering, excludes cosmic)
        std::vector<int> continentIndices;
        int cosmicZoneIdx = data.cosmicIdx();
        bool hasLeafContinents = false;
        for (int i = 0; i < static_cast<int>(data.zones().size()); i++) {
            if (i == cosmicZoneIdx) continue;
            if (isLeafContinent(data.zones(), i)) { hasLeafContinents = true; break; }
        }
        for (int i = 0; i < static_cast<int>(data.zones().size()); i++) {
            if (i == cosmicZoneIdx) continue;
            if (data.zones()[i].areaID != 0) continue;
            if (hasLeafContinents) {
                if (isLeafContinent(data.zones(), i)) continentIndices.push_back(i);
            } else if (!isRootContinent(data.zones(), i)) {
                continentIndices.push_back(i);
            }
        }
        if (continentIndices.size() > 1) {
            std::vector<int> filtered;
            filtered.reserve(continentIndices.size());
            for (int idx : continentIndices) {
                if (data.zones()[idx].areaName == mapName) continue;
                filtered.push_back(idx);
            }
            if (!filtered.empty()) continentIndices = std::move(filtered);
        }
        if (continentIndices.empty()) {
            for (int i = 0; i < static_cast<int>(data.zones().size()); i++) {
                if (i == cosmicZoneIdx) continue;
                if (data.zones()[i].areaID == 0) continentIndices.push_back(i);
            }
        }

        // Expansion filtering
        {
            std::vector<int> expFiltered;
            expFiltered.reserve(continentIndices.size());
            for (int ci : continentIndices) {
                uint32_t mapId = data.zones()[ci].displayMapID;
                if (mapId == 530 && game::isPreWotlk() && !game::isActiveExpansion("tbc")) continue;
                if (mapId == 571 && game::isPreWotlk()) continue;
                expFiltered.push_back(ci);
            }
            if (!expFiltered.empty()) continentIndices = std::move(expFiltered);
        }

        // Update layer data pointers
        if (partyDotLayer) partyDotLayer->setDots(partyDots);
        if (rareTrackerLayer) rareTrackerLayer->setRares(rares);
        if (chestTrackerLayer) chestTrackerLayer->setChests(chests);
        if (taxiNodeLayer) taxiNodeLayer->setNodes(taxiNodes);
        if (poiMarkerLayer) poiMarkerLayer->setMarkers(data.poiMarkers());
        if (questPOILayer) questPOILayer->setPois(questPois);

        // Build layer context
        LayerContext layerCtx;
        layerCtx.drawList = drawList;
        layerCtx.imgMin = imgMin;
        layerCtx.displayW = displayW;
        layerCtx.displayH = displayH;
        layerCtx.playerRenderPos = playerRenderPos;
        layerCtx.playerZoneId = playerZoneId;
        layerCtx.playerYawDeg = playerYawDeg;
        layerCtx.currentZoneIdx = viewState.currentZoneIdx();
        layerCtx.continentIdx = viewState.continentIdx();
        layerCtx.currentMapId = data.currentMapId();
        layerCtx.viewLevel = viewState.currentLevel();
        layerCtx.zones = &data.zones();
        layerCtx.exploredZones = &exploration.exploredZones();
        layerCtx.exploredOverlays = &exploration.exploredOverlays();
        layerCtx.areaNameByAreaId = &data.areaNameByAreaId();
        layerCtx.fboW = CompositeRenderer::FBO_W;
        layerCtx.fboH = CompositeRenderer::FBO_H;

        // ZMP pixel map for continent-view hover
        if (data.hasZmpData()) {
            layerCtx.zmpGrid = &data.zmpGrid();
            layerCtx.hasZmpData = true;
            layerCtx.zmpResolveZoneIdx = [](const void* repo, uint32_t areaId) -> int {
                return static_cast<const DataRepository*>(repo)->zoneIndexForAreaId(areaId);
            };
            layerCtx.zmpRepoPtr = &data;
            layerCtx.zmpZoneBounds = &data.zmpZoneBounds();
        }

        // Flight-map mode: just the map, the player marker, and the
        // interactive flight nodes - no zone navigation chrome.
        if (taxiMode) {
            if (playerMarkerLayer) playerMarkerLayer->render(layerCtx);
            if (taxiNodeLayer) taxiNodeLayer->render(layerCtx);

            // Continent name title
            int curIdx = viewState.currentZoneIdx();
            if (curIdx >= 0 && curIdx < static_cast<int>(data.zones().size())) {
                std::string title = data.zones()[curIdx].areaName;
                if (title == "Azeroth") title = mapDisplayName(0);
                if (!title.empty()) {
                    ImVec2 titleSz = ImGui::CalcTextSize(title.c_str());
                    float tx = imgMin.x + (displayW - titleSz.x) * 0.5f;
                    float ty = imgMin.y + 8.0f;
                    drawList->AddText(ImVec2(tx + 1.0f, ty + 1.0f),
                                      IM_COL32(0, 0, 0, 220), title.c_str());
                    drawList->AddText(ImVec2(tx, ty),
                                      IM_COL32(255, 215, 0, 255), title.c_str());
                }
            }

            const char* taxiHelp = "Click a destination to fly there | Escape to close";
            ImVec2 helpSz = ImGui::CalcTextSize(taxiHelp);
            float hx = imgMin.x + (displayW - helpSz.x) / 2.0f;
            float hy = imgMax.y - helpSz.y - 4.0f;
            ImGui::SetCursorScreenPos(ImVec2(hx, hy));
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.8f), "%s", taxiHelp);

            ImGui::End();
            if (!windowOpen) closeMap();
            ImGui::PopStyleVar(3);
            return;
        }

        // World-level: Azeroth map with clickable continent regions
        ViewLevel vl = viewState.currentLevel();
        if (vl == ViewLevel::WORLD) {
            bool goCosmic = false;
            if (viewState.cosmicEnabled() && !rightClickConsumed) {
                goCosmic = ImGui::GetIO().MouseClicked[1];
            }

            // "< Cosmic" back button (only if cosmic view is available for this expansion)
            if (viewState.cosmicEnabled()) {
                ImGui::SetCursorPos(ImVec2(8.0f, 8.0f));
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
                ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGold);
                if (ImGui::Button("< Cosmic")) goCosmic = true;
                ImGui::PopStyleColor(3);
            }

            if (goCosmic) {
                viewState.enterCosmicView();
                if (data.cosmicIdx() >= 0) {
                    compositor.loadZoneTextures(data.cosmicIdx(), data.zones(), mapName);
                    compositor.requestComposite(data.cosmicIdx());
                    viewState.setCurrentZoneIdx(data.cosmicIdx());
                }
            }

            // Title
            ImVec2 titleSz = ImGui::CalcTextSize("World");
            float titleX = imgMin.x + (displayW - titleSz.x) * 0.5f;
            float titleY = imgMin.y - titleSz.y - 8.0f;
            if (titleY > 0.0f) {
                drawList->AddText(ImVec2(titleX + 1.0f, titleY + 1.0f),
                                  IM_COL32(0, 0, 0, 220), "World");
                drawList->AddText(ImVec2(titleX, titleY),
                                  IM_COL32(255, 215, 0, 255), "World");
            }

            // Clickable continent regions on the Azeroth map
            ImVec2 mp2 = ImGui::GetMousePos();
            auto& io = ImGui::GetIO();
            for (const auto& region : data.azerothRegions()) {
                float rx0 = imgMin.x + region.uvLeft * displayW;
                float ry0 = imgMin.y + region.uvTop * displayH;
                float rx1 = imgMin.x + region.uvRight * displayW;
                float ry1 = imgMin.y + region.uvBottom * displayH;

                bool hovered = (mp2.x >= rx0 && mp2.x <= rx1 &&
                                mp2.y >= ry0 && mp2.y <= ry1);

                if (hovered) {
                    // Map region mapId to the highlight texture folder name
                    std::string regionFolder = mapIdToFolder(region.mapId);

                    // Draw highlight texture covering the full map area
                    if (zoneHighlightLayer && !regionFolder.empty()) {
                        ImTextureID hlTex = zoneHighlightLayer->getHighlightTexture(regionFolder);
                        if (hlTex) {
                            drawList->AddImage(hlTex,
                                ImVec2(imgMin.x, imgMin.y),
                                ImVec2(imgMin.x + displayW, imgMin.y + displayH),
                                ImVec2(0, 0), ImVec2(1, 1),
                                IM_COL32(255, 255, 255, 180));
                        } else {
                            drawList->AddRectFilled(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                                    IM_COL32(255, 215, 0, 25));
                        }
                    } else {
                        drawList->AddRectFilled(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                                IM_COL32(255, 215, 0, 25));
                    }
                    drawList->AddRect(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                      IM_COL32(255, 215, 0, 100), 0, 0, 1.5f);

                    ImFont* font = ImGui::GetFont();
                    ImVec2 labelSz = font->CalcTextSizeA(ImGui::GetFontSize(), FLT_MAX, 0.0f,
                                                          region.label.c_str());
                    float lx = (rx0 + rx1 - labelSz.x) * 0.5f;
                    float ly = ry0 - labelSz.y - 4.0f;
                    if (ly < imgMin.y) ly = ry0 + 4.0f;
                    drawList->AddText(ImVec2(lx + 1.0f, ly + 1.0f),
                                      IM_COL32(0, 0, 0, 200), region.label.c_str());
                    drawList->AddText(ImVec2(lx, ly),
                                      IM_COL32(255, 230, 100, 255), region.label.c_str());

                    if (io.MouseClicked[0]) {
                        // Use centralized map resolver to determine navigation action
                        auto resolveResult = resolveWorldRegionClick(
                            region.mapId, data.zones(), data.currentMapId(), data.cosmicIdx());
                        switch (resolveResult.action) {
                            case MapResolveAction::NAVIGATE_CONTINENT:
                                // Same map - just switch to the continent view
                                viewState.setContinentIdx(resolveResult.targetZoneIdx);
                                compositor.loadZoneTextures(resolveResult.targetZoneIdx, data.zones(), mapName);
                                compositor.requestComposite(resolveResult.targetZoneIdx);
                                viewState.setCurrentZoneIdx(resolveResult.targetZoneIdx);
                                viewState.setLevel(ViewLevel::CONTINENT);
                                break;
                            case MapResolveAction::LOAD_MAP:
                                switchToMap(resolveResult.targetMapName);
                                break;
                            default:
                                break;
                        }
                        break;
                    }
                }
            }
        } else if (vl == ViewLevel::CONTINENT && continentIndices.size() > 1) {
            ImGui::SetCursorPos(ImVec2(8.0f, contentLocalY + 8.0f));
            for (size_t i = 0; i < continentIndices.size(); i++) {
                int ci = continentIndices[i];
                if (i > 0) ImGui::SameLine();
                const bool selected = (ci == viewState.continentIdx());
                if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.25f, 0.05f, 0.9f));
                std::string rawName = data.zones()[ci].areaName.empty() ? "Continent" : data.zones()[ci].areaName;
                if (rawName == "Azeroth") rawName = mapDisplayName(0);
                std::string label = rawName + "##" + std::to_string(ci);
                if (ImGui::Button(label.c_str())) {
                    viewState.setContinentIdx(ci);
                    compositor.loadZoneTextures(ci, data.zones(), mapName);
                    compositor.requestComposite(ci);
                    viewState.setCurrentZoneIdx(ci);
                }
                if (selected) ImGui::PopStyleColor();
            }
        }

        // Render all overlay layers
        overlay.render(layerCtx);

        // Zone view: back to continent + zone name
        if (vl == ViewLevel::ZONE && viewState.continentIdx() >= 0) {
            ImGui::SetCursorPos(ImVec2(8.0f, contentLocalY + 8.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGold);
            if (ImGui::Button("< Back")) {
                compositor.loadZoneTextures(viewState.continentIdx(), data.zones(), mapName);
                compositor.requestComposite(viewState.continentIdx());
                viewState.setCurrentZoneIdx(viewState.continentIdx());
                viewState.setLevel(ViewLevel::CONTINENT);
            }
            ImGui::PopStyleColor(3);

            int curIdx = viewState.currentZoneIdx();
            if (curIdx >= 0 && curIdx < static_cast<int>(data.zones().size())) {
                const char* zoneName = data.zones()[curIdx].areaName.c_str();
                ImVec2 nameSize = ImGui::CalcTextSize(zoneName);
                float nameX = imgMin.x + (displayW - nameSize.x) * 0.5f;
                float nameY = imgMin.y + 8.0f;
                drawList->AddText(ImVec2(nameX + 1.0f, nameY + 1.0f),
                                  IM_COL32(0, 0, 0, 220), zoneName);
                drawList->AddText(ImVec2(nameX, nameY),
                                  IM_COL32(255, 215, 0, 230), zoneName);
            }
        }

        // Continent view: back to world + hovered zone name
        if (vl == ViewLevel::CONTINENT) {
            float localBtnY = contentLocalY +
                              (continentIndices.size() > 1 ? 40.0f : 8.0f);
            ImGui::SetCursorPos(ImVec2(8.0f, localBtnY));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGold);
            if (ImGui::Button("< Azeroth")) {
                switchToWorldView();
            }
            ImGui::PopStyleColor(3);

            // Show hovered zone name above the map
            int hovZone = zoneHighlightLayer ? zoneHighlightLayer->hoveredZone() : -1;
            if (hovZone >= 0 && hovZone < static_cast<int>(data.zones().size())) {
                const std::string& rawName = data.zones()[hovZone].areaName;
                if (!rawName.empty()) {
                    const ZoneMeta* meta = zoneMetadata.find(rawName);
                    std::string hoverLabel = ZoneMetadata::formatHoverLabel(rawName, meta);

                    ImVec2 hoverSz = ImGui::CalcTextSize(hoverLabel.c_str());
                    float hx = imgMin.x + (displayW - hoverSz.x) * 0.5f;
                    float hy = imgMin.y + 8.0f;
                    drawList->AddText(ImVec2(hx + 1.0f, hy + 1.0f),
                                      IM_COL32(0, 0, 0, 220), hoverLabel.c_str());
                    ImU32 hoverColor = IM_COL32(255, 215, 0, 255);
                    if (meta) {
                        switch (meta->faction) {
                            case ZoneFaction::Alliance: hoverColor = IM_COL32(100, 160, 255, 255); break;
                            case ZoneFaction::Horde:    hoverColor = IM_COL32(255, 80, 80, 255); break;
                            default: break;
                        }
                    }
                    drawList->AddText(ImVec2(hx, hy), hoverColor, hoverLabel.c_str());
                }
            }
        }

        // Cosmic view: title + clickable landmass regions
        if (vl == ViewLevel::COSMIC) {
            ImGui::SetCursorPos(ImVec2(8.0f, contentLocalY + 8.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGold);
            if (ImGui::Button("< Azeroth")) {
                switchToWorldView();
            }
            ImGui::PopStyleColor(3);

            ImVec2 titleSz = ImGui::CalcTextSize("Cosmic");
            float titleX = imgMin.x + (displayW - titleSz.x) * 0.5f;
            float titleY = imgMin.y + 8.0f;
            drawList->AddText(ImVec2(titleX + 1.0f, titleY + 1.0f),
                              IM_COL32(0, 0, 0, 220), "Cosmic");
            drawList->AddText(ImVec2(titleX, titleY),
                              IM_COL32(255, 215, 0, 255), "Cosmic");

            ImVec2 mp2 = ImGui::GetMousePos();
            auto& io = ImGui::GetIO();

            for (const auto& entry : data.cosmicMaps()) {
                float rx0 = imgMin.x + entry.uvLeft * displayW;
                float ry0 = imgMin.y + entry.uvTop * displayH;
                float rx1 = imgMin.x + entry.uvRight * displayW;
                float ry1 = imgMin.y + entry.uvBottom * displayH;

                bool hovered = (mp2.x >= rx0 && mp2.x <= rx1 &&
                                mp2.y >= ry0 && mp2.y <= ry1);

                if (hovered) {
                    // Cosmic highlight files: cosmic-{label}-highlight.blp
                    std::string cosmicLabel = entry.label;
                    std::transform(cosmicLabel.begin(), cosmicLabel.end(), cosmicLabel.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    std::string cosmicKey = "cosmic-" + cosmicLabel;
                    std::string cosmicPath = "Interface\\WorldMap\\Cosmic\\cosmic-" + cosmicLabel + "-highlight.blp";

                    // ─── Cosmic Highlight Rendering Logic ───────────────────
                    //
                    // SOURCE TEXTURES:
                    //   cosmic-azeroth-highlight.blp  → 512×512 px (DXT3, has alpha)
                    //   cosmic-outland-highlight.blp  → 512×512 px (DXT3, has alpha)
                    //   The glow is baked into the alpha channel:
                    //     - Azeroth highlight: glow sits in the RIGHT-CENTER of the texture
                    //     - Outland highlight: glow sits in the LEFT-CENTER of the texture
                    //
                    // DISPLAY AREA:
                    //   The map on screen is displayW × displayH pixels.
                    //   displayW/displayH ≈ 1002/668 ≈ 1.5:1 (wider than tall).
                    //   imgMin = top-left corner,  imgMax = bottom-right corner.
                    //
                    // THE PROBLEM:
                    //   512×512 is square, but the display area is 1.5× wider than tall.
                    //   If we stretch the texture to fill the full display area
                    //   (imgMin → imgMax), the circular glow becomes an ellipse
                    //   (horizontally stretched ~50%).
                    //   If we render it as a square (side = displayH), it has the
                    //   correct aspect but only covers 2/3 of the map width.
                    //
                    // CURRENT APPROACH:
                    //   Render as a square (side = displayH), anchored:
                    //     Azeroth → flush to the RIGHT edge of the map (glow lands bottom-right)
                    //     Outland → flush to the LEFT edge of the map  (glow lands top-left)
                    //   This preserves the 1:1 aspect ratio of the glow shape.
                    //
                    // TO ADJUST:
                    //   • Make glow wider:  increase hlW (e.g. displayH * 1.2f)
                    //   • Make glow taller: increase hlH (e.g. displayH * 1.1f)
                    //   • Full stretch (like WoW original): hlW = displayW, hlH = displayH
                    //   • Shift glow position: adjust hlX offset
                    //
                    // Render highlight as a square (side = displayH) to preserve
                    // the 1:1 aspect of the 512×512 glow textures at any resolution.
                    float hlW = displayH;
                    float hlH = displayH;
                    float hlX, hlY;
                    if (cosmicLabel == "azeroth") {
                        hlX = imgMax.x - hlW;   // flush right (glow sits in right-center of texture)
                        hlY = imgMin.y;          // flush top
                    } else {
                        hlX = imgMin.x;          // flush left (glow sits in left-center of texture)
                        hlY = imgMin.y;          // flush top
                    }

                    if (zoneHighlightLayer) {
                        ImTextureID hlTex = zoneHighlightLayer->getHighlightTexture(cosmicKey, cosmicPath);
                        if (hlTex) {
                            drawList->AddImage(hlTex,
                                ImVec2(hlX, hlY),
                                ImVec2(hlX + hlW, hlY + hlH),
                                ImVec2(0, 0), ImVec2(1, 1),
                                IM_COL32(255, 255, 255, 180));
                        } else {
                            drawList->AddRectFilled(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                                    IM_COL32(255, 215, 0, 25));
                        }
                    } else {
                        drawList->AddRectFilled(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                                IM_COL32(255, 215, 0, 25));
                    }
                    drawList->AddRect(ImVec2(rx0, ry0), ImVec2(rx1, ry1),
                                      IM_COL32(255, 215, 0, 100), 0, 0, 1.5f);

                    ImFont* font = ImGui::GetFont();
                    ImVec2 labelSz = font->CalcTextSizeA(ImGui::GetFontSize(), FLT_MAX, 0.0f,
                                                          entry.label.c_str());
                    float lx = (rx0 + rx1 - labelSz.x) * 0.5f;
                    float ly = ry0 - labelSz.y - 4.0f;
                    if (ly < imgMin.y) ly = ry0 + 4.0f;
                    drawList->AddText(ImVec2(lx + 1.0f, ly + 1.0f),
                                      IM_COL32(0, 0, 0, 200), entry.label.c_str());
                    drawList->AddText(ImVec2(lx, ly),
                                      IM_COL32(255, 230, 100, 255), entry.label.c_str());

                    if (io.MouseClicked[0]) {
                        if (entry.label == "Outland") {
                            switchToMap("Expansion01");
                        } else {
                            viewState.enterWorldView();
                            int wIdx = data.worldIdx();
                            if (wIdx >= 0) {
                                compositor.loadZoneTextures(wIdx, data.zones(), mapName);
                                compositor.invalidateComposite();
                                compositor.requestComposite(wIdx);
                                viewState.setCurrentZoneIdx(wIdx);
                            }
                        }
                        break;
                    }
                }
            }
        }

        // Help text
        const char* helpText;
        if (vl == ViewLevel::ZONE)
            helpText = "Right-click to zoom out | M or Escape to close";
        else if (vl == ViewLevel::COSMIC)
            helpText = "Scroll in or click to zoom in | M or Escape to close";
        else if (vl == ViewLevel::WORLD && viewState.cosmicEnabled())
            helpText = "Click a continent | Right-click for Cosmic view | M or Escape to close";
        else if (vl == ViewLevel::WORLD)
            helpText = "Click a continent | M or Escape to close";
        else
            helpText = "Click zone to open | Right-click to zoom out | M or Escape to close";

        ImVec2 textSize = ImGui::CalcTextSize(helpText);
        float textX = imgMin.x + (displayW - textSize.x) / 2.0f;
        float textY = imgMax.y - textSize.y - 4.0f;
        ImGui::SetCursorScreenPos(ImVec2(textX, textY));
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.8f), "%s", helpText);
    }
    ImGui::End();

    if (!windowOpen) {
        closeMap();
    }

    ImGui::PopStyleVar(3);  // WindowPadding + ItemSpacing + WindowBorderSize
}

// ── Navigating by name ─────────────────────────────────────────────────────

namespace {

/// The four continents 3.3.5 lists, and the map each one is. Fixed, and in
/// that order, because the interface passes the position in this list straight
/// back as the continent to show - a list discovered from the data would
/// reorder itself with the data and silently point every saved index at a
/// different continent.
struct ContinentEntry { uint32_t mapId; const char* name; };
const ContinentEntry kContinents[] = {
    {   .mapId = 1, .name = "Kalimdor"         },
    {   .mapId = 0, .name = "Eastern Kingdoms" },
    { .mapId = 530, .name = "Outland"          },
    { .mapId = 571, .name = "Northrend"        },
};
constexpr int kContinentCount = static_cast<int>(std::size(kContinents));

/// The zone-list index of a continent, or -1. A continent is the zone with no
/// area of its own on that map.
int continentZoneIdx(const std::vector<Zone>& zones, uint32_t mapId) {
    for (size_t i = 0; i < zones.size(); ++i) {
        if (zones[i].mapID == mapId && zones[i].areaID == 0 &&
            isLeafContinent(zones, static_cast<int>(i))) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

} // namespace

std::vector<std::string> WorldMapFacade::continentNames() const {
    std::vector<std::string> out;
    out.reserve(kContinentCount);
    for (const ContinentEntry& c : kContinents) out.emplace_back(c.name);
    return out;
}

/// The zones on a continent as (display name, zone index), alphabetically.
/// Both the name list and the navigation need this and would drift apart if
/// each worked it out for itself - the index the interface hands back has to
/// mean the same row it was shown.
static std::vector<std::pair<std::string, int>> zonesOnContinent(
        const std::vector<Zone>& zones,
        const std::unordered_map<uint32_t, std::string>& names,
        int contIdx) {
    std::vector<std::pair<std::string, int>> out;
    if (contIdx < 0) return out;
    for (size_t i = 0; i < zones.size(); ++i) {
        if (zones[i].areaID == 0) continue;
        if (!zoneBelongsToContinent(zones, static_cast<int>(i), contIdx)) continue;
        auto it = names.find(zones[i].areaID);
        // The DBC's own area name rather than the texture folder, which is
        // what the folder-keyed areaName holds and is not readable.
        std::string name = (it != names.end()) ? it->second : zones[i].areaName;
        if (name.empty()) continue;
        out.emplace_back(std::move(name), static_cast<int>(i));
    }
    std::sort(out.begin(), out.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    return out;
}

std::vector<std::string> WorldMapFacade::zoneNames(int continentIndex) const {
    std::vector<std::string> out;
    if (continentIndex < 1 || continentIndex > kContinentCount) return out;
    const auto& zones = impl_->data.zones();
    const int contIdx = continentZoneIdx(
        zones, kContinents[continentIndex - 1].mapId);
    for (auto& row : zonesOnContinent(zones, impl_->data.areaNameByAreaId(), contIdx)) {
        out.push_back(std::move(row.first));
    }
    return out;
}

bool WorldMapFacade::showMap(int continentIndex, int zoneIndex) {
    // Continent zero is the world, which is what the zoom-out button asks for
    // from a continent.
    if (continentIndex <= 0) {
        impl_->viewState.enterWorldView();
        return true;
    }
    if (continentIndex > kContinentCount) return false;
    const auto& zones = impl_->data.zones();
    const int contIdx = continentZoneIdx(
        zones, kContinents[continentIndex - 1].mapId);
    if (contIdx < 0) return false;

    if (zoneIndex <= 0) {
        impl_->viewState.setContinentIdx(contIdx);
        impl_->viewState.setCurrentZoneIdx(contIdx);
        impl_->viewState.setLevel(ViewLevel::CONTINENT);
        return true;
    }
    const auto rows = zonesOnContinent(zones, impl_->data.areaNameByAreaId(), contIdx);
    if (zoneIndex > static_cast<int>(rows.size())) return false;
    impl_->viewState.setContinentIdx(contIdx);
    impl_->viewState.enterZone(rows[static_cast<size_t>(zoneIndex) - 1].second);
    return true;
}

int WorldMapFacade::currentContinentIndex() const {
    const auto& zones = impl_->data.zones();
    const int contIdx = impl_->viewState.continentIdx();
    if (contIdx < 0 || contIdx >= static_cast<int>(zones.size())) return 0;
    if (impl_->viewState.currentLevel() == ViewLevel::WORLD ||
        impl_->viewState.currentLevel() == ViewLevel::COSMIC) {
        return 0;
    }
    const uint32_t mapId = zones[static_cast<size_t>(contIdx)].mapID;
    for (int i = 0; i < kContinentCount; ++i) {
        if (kContinents[i].mapId == mapId) return i + 1;
    }
    return 0;
}

int WorldMapFacade::currentZoneIndex() const {
    if (impl_->viewState.currentLevel() != ViewLevel::ZONE) return 0;
    const int contIdx = impl_->viewState.continentIdx();
    const int zoneIdx = impl_->viewState.currentZoneIdx();
    const auto rows = zonesOnContinent(
        impl_->data.zones(), impl_->data.areaNameByAreaId(), contIdx);
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].second == zoneIdx) return static_cast<int>(i) + 1;
    }
    return 0;
}

bool WorldMapFacade::takeViewChanged() {
    const bool was = impl_->viewChanged;
    impl_->viewChanged = false;
    return was;
}

void WorldMapFacade::showPlayerZone() {
    impl_->userMapOverride = false;
    impl_->recenterOnPlayer = true;
}

uint32_t WorldMapFacade::currentWorldMapAreaId() const {
    // Zero unless a zone is being shown. The interface branches on exactly
    // that: a continent has no area of its own to name, and the caller falls
    // back to asking which continent instead.
    if (impl_->viewState.currentLevel() != ViewLevel::ZONE) return 0;
    const int idx = impl_->viewState.currentZoneIdx();
    const auto& zones = impl_->data.zones();
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return 0;
    if (zones[static_cast<size_t>(idx)].areaID == 0) return 0;
    return zones[static_cast<size_t>(idx)].wmaID;
}

bool WorldMapFacade::showWorldMapArea(uint32_t worldMapAreaId) {
    if (worldMapAreaId == 0) return false;
    const auto& zones = impl_->data.zones();
    for (size_t i = 0; i < zones.size(); ++i) {
        if (zones[i].wmaID != worldMapAreaId) continue;
        // A continent is a zone with no area of its own, and entering one as
        // if it were a zone would show its overview at zone level.
        if (zones[i].areaID == 0) {
            impl_->viewState.setContinentIdx(static_cast<int>(i));
            impl_->viewState.setCurrentZoneIdx(static_cast<int>(i));
            impl_->viewState.setLevel(ViewLevel::CONTINENT);
        } else {
            impl_->viewState.enterZone(static_cast<int>(i));
        }
        impl_->userMapOverride = true;
        return true;
    }
    return false;
}

bool WorldMapFacade::canZoomOut() const {
    // Everything but the outermost view has somewhere to go. The button that
    // asks this was disabled at all times, so the only way out of a zone map
    // was the right-click the same handler also answers.
    const ViewLevel level = impl_->viewState.currentLevel();
    return level == ViewLevel::ZONE || level == ViewLevel::CONTINENT ||
           (level == ViewLevel::WORLD && impl_->viewState.cosmicEnabled());
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
