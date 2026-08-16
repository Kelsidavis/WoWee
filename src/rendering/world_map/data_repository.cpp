// data_repository.cpp - DBC data loading, ZMP pixel map, and zone/POI/overlay storage.
// Extracted from WorldMap::loadZonesFromDBC, loadPOIData, buildCosmicView
// (Phase 5 of refactoring plan).
#include "rendering/world_map/data_repository.hpp"
#include "rendering/world_map/map_resolver.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include "core/application.hpp"
#include "game/expansion_profile.hpp"
#include "game/game_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace wowee {
namespace rendering {
namespace world_map {

void DataRepository::clear() {
    zones_.clear();
    mapTransforms_.clear();
    poiMarkers_.clear();
    cosmicMaps_.clear();
    azerothRegions_.clear();
    exploreFlagByAreaId_.clear();
    areaNameByAreaId_.clear();
    areaIdToZoneIdx_.clear();
    zmpZoneBounds_.clear();
    zmpGrid_.fill(0);
    zmpLoaded_ = false;
    cosmicIdx_ = -1;
    worldIdx_ = -1;
    currentMapId_ = -1;
    cosmicEnabled_ = true;
    poisLoaded_ = false;
}

// --------------------------------------------------------
// DBC zone loading (moved from WorldMap::loadZonesFromDBC)
// --------------------------------------------------------

void DataRepository::loadZones(const std::string& mapName,
                                pipeline::AssetManager& assetManager) {
    if (!zones_.empty()) return;

    const auto* activeLayout = pipeline::getActiveDBCLayout();
    const auto* mapL = activeLayout ? activeLayout->getLayout("Map") : nullptr;

    int mapID = -1;
    auto mapDbc = assetManager.loadDBC("Map.dbc");
    if (mapDbc && mapDbc->isLoaded()) {
        for (uint32_t i = 0; i < mapDbc->getRecordCount(); i++) {
            std::string dir = mapDbc->getString(i, mapL ? (*mapL)["InternalName"] : 1);
            if (dir == mapName) {
                mapID = static_cast<int>(mapDbc->getUInt32(i, mapL ? (*mapL)["ID"] : 0));
                LOG_INFO("DataRepository: Map.dbc '", mapName, "' -> mapID=", mapID);
                break;
            }
        }
    }

    if (mapID < 0) {
        mapID = folderToMapId(mapName);
        if (mapID < 0) {
            LOG_WARNING("DataRepository: unknown map '", mapName, "'");
            return;
        }
    }

    // Physical-map regions that the UI presents on another continent. In
    // particular, Azuremyst/Bloodmyst use map 530 terrain but are projected
    // onto Kalimdor (map 1) with a +10133.333/+17600 coordinate offset.
    auto transformsDbc = assetManager.loadDBC("WorldMapTransforms.dbc");
    if (transformsDbc && transformsDbc->isLoaded() &&
        transformsDbc->getFieldCount() >= 9) {
        for (uint32_t i = 0; i < transformsDbc->getRecordCount(); ++i) {
            MapTransform tr;
            tr.sourceMapId = transformsDbc->getUInt32(i, 1);
            tr.minX = transformsDbc->getFloat(i, 2);
            tr.minY = transformsDbc->getFloat(i, 3);
            tr.maxX = transformsDbc->getFloat(i, 4);
            tr.maxY = transformsDbc->getFloat(i, 5);
            tr.targetMapId = transformsDbc->getUInt32(i, 6);
            tr.offsetX = transformsDbc->getFloat(i, 7);
            tr.offsetY = transformsDbc->getFloat(i, 8);
            mapTransforms_.push_back(tr);
        }
    }

    // Use expansion-aware DBC layout when available; fall back to WotLK stock field
    // indices (ID=0, ParentAreaNum=2, ExploreFlag=3) when layout metadata is missing.
    const auto* atL = activeLayout ? activeLayout->getLayout("AreaTable") : nullptr;
    std::unordered_map<uint32_t, uint32_t> exploreFlagByAreaId;
    std::unordered_map<uint32_t, std::vector<uint32_t>> childBitsByParent;
    auto areaDbc = assetManager.loadDBC("AreaTable.dbc");
    // Bug fix: old code used > 3 which covers core fields (ID=0, ParentAreaNum=2,
    // ExploreFlag=3).  The > 11 threshold broke exploration for DBC variants with
    // 4-11 fields.  Load core exploration data with > 3; area name only when > 11.
    if (areaDbc && areaDbc->isLoaded() && areaDbc->getFieldCount() > 3) {
        const uint32_t fieldCount = areaDbc->getFieldCount();
        const uint32_t parentField = atL ? (*atL)["ParentAreaNum"] : 2;
        for (uint32_t i = 0; i < areaDbc->getRecordCount(); i++) {
            const uint32_t areaId = areaDbc->getUInt32(i, atL ? (*atL)["ID"] : 0);
            const uint32_t exploreFlag = areaDbc->getUInt32(i, atL ? (*atL)["ExploreFlag"] : 3);
            const uint32_t parentArea = areaDbc->getUInt32(i, parentField);
            if (areaId != 0) exploreFlagByAreaId[areaId] = exploreFlag;
            if (parentArea != 0) childBitsByParent[parentArea].push_back(exploreFlag);
            // Cache area display name (field 11 = AreaName_lang enUS)
            if (areaId != 0 && fieldCount > 11) {
                std::string areaDispName = areaDbc->getString(i, 11);
                if (!areaDispName.empty())
                    areaNameByAreaId_[areaId] = std::move(areaDispName);
            }
        }
    }

    auto wmaDbc = assetManager.loadDBC("WorldMapArea.dbc");
    if (!wmaDbc || !wmaDbc->isLoaded()) {
        LOG_WARNING("DataRepository: WorldMapArea.dbc not found");
        return;
    }

    const auto* wmaL = activeLayout ? activeLayout->getLayout("WorldMapArea") : nullptr;

    int continentIdx = -1;
    for (uint32_t i = 0; i < wmaDbc->getRecordCount(); i++) {
        uint32_t recMapID = wmaDbc->getUInt32(i, wmaL ? (*wmaL)["MapID"] : 1);
        uint32_t displayMapID = wmaDbc->getUInt32(i, wmaL ? (*wmaL)["DisplayMapID"] : 8);
        // WorldMapArea can place a zone on one physical map while displaying it
        // under another continent. Azuremyst/Bloodmyst physically live on map
        // 530, but DisplayMapID=1 makes them children of Kalimdor. Include those
        // virtual children when building the display continent.
        if (static_cast<int>(recMapID) != mapID &&
            static_cast<int>(displayMapID) != mapID) continue;

        Zone zone;
        zone.wmaID   = wmaDbc->getUInt32(i, wmaL ? (*wmaL)["ID"] : 0);
        zone.mapID   = recMapID;
        zone.areaID  = wmaDbc->getUInt32(i, wmaL ? (*wmaL)["AreaID"] : 2);
        zone.areaName = wmaDbc->getString(i, wmaL ? (*wmaL)["AreaName"] : 3);
        zone.bounds.locLeft   = wmaDbc->getFloat(i, wmaL ? (*wmaL)["LocLeft"] : 4);
        zone.bounds.locRight  = wmaDbc->getFloat(i, wmaL ? (*wmaL)["LocRight"] : 5);
        zone.bounds.locTop    = wmaDbc->getFloat(i, wmaL ? (*wmaL)["LocTop"] : 6);
        zone.bounds.locBottom = wmaDbc->getFloat(i, wmaL ? (*wmaL)["LocBottom"] : 7);
        zone.displayMapID = displayMapID;
        zone.parentWorldMapID = wmaDbc->getUInt32(i, wmaL ? (*wmaL)["ParentWorldMapID"] : 10);
        if (recMapID != static_cast<uint32_t>(mapID) && displayMapID == static_cast<uint32_t>(mapID)) {
            const float centerX = (zone.bounds.locLeft + zone.bounds.locRight) * 0.5f;
            const float centerY = (zone.bounds.locTop + zone.bounds.locBottom) * 0.5f;
            for (const MapTransform& tr : mapTransforms_) {
                const float minX = std::min(tr.minX, tr.maxX);
                const float maxX = std::max(tr.minX, tr.maxX);
                const float minY = std::min(tr.minY, tr.maxY);
                const float maxY = std::max(tr.minY, tr.maxY);
                if (tr.sourceMapId != recMapID || tr.targetMapId != displayMapID ||
                    centerX < minX || centerX > maxX || centerY < minY || centerY > maxY) {
                    continue;
                }
                zone.virtualOffsetWowX = tr.offsetX;
                zone.virtualOffsetWowY = tr.offsetY;
                zone.bounds.locLeft += tr.offsetX;
                zone.bounds.locRight += tr.offsetX;
                zone.bounds.locTop += tr.offsetY;
                zone.bounds.locBottom += tr.offsetY;
                break;
            }
        }
        // Collect the zone's own AreaBit plus all subzone AreaBits
        auto exploreIt = exploreFlagByAreaId.find(zone.areaID);
        if (exploreIt != exploreFlagByAreaId.end())
            zone.exploreBits.push_back(exploreIt->second);
        auto childIt = childBitsByParent.find(zone.areaID);
        if (childIt != childBitsByParent.end()) {
            for (uint32_t bit : childIt->second)
                zone.exploreBits.push_back(bit);
        }

        int idx = static_cast<int>(zones_.size());

        LOG_INFO("DataRepository: zone[", idx, "] areaID=", zone.areaID,
                 " '", zone.areaName, "' L=", zone.bounds.locLeft,
                 " R=", zone.bounds.locRight, " T=", zone.bounds.locTop,
                 " B=", zone.bounds.locBottom);

        if (zone.areaID == 0 && continentIdx < 0)
            continentIdx = idx;

        zones_.push_back(std::move(zone));
    }

    // Derive continent bounds from child zones if missing
    for (auto& cont : zones_) {
        if (cont.areaID != 0) continue;
        if (std::abs(cont.bounds.locLeft) > 0.001f || std::abs(cont.bounds.locRight) > 0.001f ||
            std::abs(cont.bounds.locTop) > 0.001f || std::abs(cont.bounds.locBottom) > 0.001f)
            continue;

        bool first = true;
        for (const auto& z : zones_) {
            if (z.areaID == 0) continue;
            if (std::abs(z.bounds.locLeft - z.bounds.locRight) < 0.001f ||
                std::abs(z.bounds.locTop - z.bounds.locBottom) < 0.001f)
                continue;
            if (z.parentWorldMapID != 0 && cont.wmaID != 0 && z.parentWorldMapID != cont.wmaID)
                continue;

            if (first) {
                cont.bounds.locLeft = z.bounds.locLeft; cont.bounds.locRight = z.bounds.locRight;
                cont.bounds.locTop = z.bounds.locTop; cont.bounds.locBottom = z.bounds.locBottom;
                first = false;
            } else {
                // WorldMapArea loc coords descend left→right and top→bottom
                // (locLeft/locTop hold the larger world coordinate), so the
                // union keeps the max on the left/top edges.
                cont.bounds.locLeft = std::max(cont.bounds.locLeft, z.bounds.locLeft);
                cont.bounds.locRight = std::min(cont.bounds.locRight, z.bounds.locRight);
                cont.bounds.locTop = std::max(cont.bounds.locTop, z.bounds.locTop);
                cont.bounds.locBottom = std::min(cont.bounds.locBottom, z.bounds.locBottom);
            }
        }
    }

    currentMapId_ = mapID;
    exploreFlagByAreaId_ = exploreFlagByAreaId;  // cache for overlay exploration checks
    LOG_INFO("DataRepository: loaded ", zones_.size(), " zones for mapID=", mapID,
             ", continentIdx=", continentIdx);

    // Build wmaID → zone index lookup
    std::unordered_map<uint32_t, int> wmaIdToZoneIdx;
    for (int i = 0; i < static_cast<int>(zones_.size()); i++)
        wmaIdToZoneIdx[zones_[i].wmaID] = i;

    // Parse WorldMapOverlay.dbc → attach overlay entries to their zones
    auto wmoDbc = assetManager.loadDBC("WorldMapOverlay.dbc");
    if (wmoDbc && wmoDbc->isLoaded()) {
        // WotLK field layout:
        // 0:ID, 1:WorldMapAreaID, 2-5:AreaTableID[4],
        // 6:MapPointX, 7:MapPointY, 8:TextureName(str),
        // 9:TextureWidth, 10:TextureHeight,
        // 11:OffsetX, 12:OffsetY, 13-16:HitRect
        int totalOverlays = 0;
        for (uint32_t i = 0; i < wmoDbc->getRecordCount(); i++) {
            uint32_t wmaID = wmoDbc->getUInt32(i, 1);
            auto it = wmaIdToZoneIdx.find(wmaID);
            if (it == wmaIdToZoneIdx.end()) continue;

            OverlayEntry ov;
            ov.areaIDs[0] = wmoDbc->getUInt32(i, 2);
            ov.areaIDs[1] = wmoDbc->getUInt32(i, 3);
            ov.areaIDs[2] = wmoDbc->getUInt32(i, 4);
            ov.areaIDs[3] = wmoDbc->getUInt32(i, 5);
            ov.textureName = wmoDbc->getString(i, 8);
            ov.texWidth  = static_cast<uint16_t>(wmoDbc->getUInt32(i, 9));
            ov.texHeight = static_cast<uint16_t>(wmoDbc->getUInt32(i, 10));
            ov.offsetX   = static_cast<uint16_t>(wmoDbc->getUInt32(i, 11));
            ov.offsetY   = static_cast<uint16_t>(wmoDbc->getUInt32(i, 12));
            // HitRect (fields 13-16): fast AABB pre-filter for subzone hover
            ov.hitRectLeft   = static_cast<uint16_t>(wmoDbc->getUInt32(i, 13));
            ov.hitRectRight  = static_cast<uint16_t>(wmoDbc->getUInt32(i, 14));
            ov.hitRectTop    = static_cast<uint16_t>(wmoDbc->getUInt32(i, 15));
            ov.hitRectBottom = static_cast<uint16_t>(wmoDbc->getUInt32(i, 16));

            if (ov.textureName.empty() || ov.texWidth == 0 || ov.texHeight == 0) continue;

            ov.tileCols = (ov.texWidth + 255) / 256;
            ov.tileRows = (ov.texHeight + 255) / 256;

            zones_[it->second].overlays.push_back(std::move(ov));
            totalOverlays++;
        }
        LOG_INFO("DataRepository: loaded ", totalOverlays, " overlay entries from WorldMapOverlay.dbc");
    }

    // Create a synthetic "Cosmic" zone for the cross-world map (Azeroth + Outland)
    {
        Zone cosmic;
        cosmic.areaName = "Cosmic";
        cosmicIdx_ = static_cast<int>(zones_.size());
        zones_.push_back(std::move(cosmic));
        LOG_INFO("DataRepository: added synthetic Cosmic zone at index ", cosmicIdx_);
    }

    // Create a synthetic "World" zone for the combined world overview map
    {
        Zone world;
        world.areaName = "World";
        worldIdx_ = static_cast<int>(zones_.size());
        zones_.push_back(std::move(world));
        LOG_INFO("DataRepository: added synthetic World zone at index ", worldIdx_);
    }

    // Load area POI data (towns, dungeons, etc.)
    loadPOIs(assetManager);

    // Build areaID → zone index lookup for ZMP resolution
    for (int i = 0; i < static_cast<int>(zones_.size()); i++) {
        if (zones_[i].areaID != 0)
            areaIdToZoneIdx_[zones_[i].areaID] = i;
    }

    // Load ZMP pixel map for continent-level hover detection
    loadZmpPixelMap(mapName, assetManager);

    // Build views based on active expansion
    buildCosmicView();
    buildAzerothView();
}

glm::vec3 DataRepository::transformRenderPosition(
    uint32_t sourceMapId, const glm::vec3& renderPos) const {
    if (currentMapId_ < 0 || sourceMapId == static_cast<uint32_t>(currentMapId_)) {
        return renderPos;
    }
    const float wowX = renderPos.y;
    const float wowY = renderPos.x;
    for (const MapTransform& tr : mapTransforms_) {
        const float minX = std::min(tr.minX, tr.maxX);
        const float maxX = std::max(tr.minX, tr.maxX);
        const float minY = std::min(tr.minY, tr.maxY);
        const float maxY = std::max(tr.minY, tr.maxY);
        if (tr.sourceMapId == sourceMapId &&
            tr.targetMapId == static_cast<uint32_t>(currentMapId_) &&
            wowX >= minX && wowX <= maxX && wowY >= minY && wowY <= maxY) {
            return glm::vec3(wowY + tr.offsetY, wowX + tr.offsetX, renderPos.z);
        }
    }
    return renderPos;
}

// --------------------------------------------------------
// ZMP pixel map loading
// --------------------------------------------------------

void DataRepository::loadZmpPixelMap(const std::string& continentName,
                                      pipeline::AssetManager& assetManager) {
    zmpGrid_.fill(0);
    zmpLoaded_ = false;

    // ZMP path: Interface\WorldMap\{name_lower}.zmp
    std::string lower = continentName;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::string zmpPath = "Interface\\WorldMap\\" + lower + ".zmp";

    auto data = assetManager.readFileOptional(zmpPath);
    if (data.empty()) {
        LOG_INFO("DataRepository: ZMP not found at '", zmpPath, "' (ok for non-continent maps)");
        return;
    }

    // ZMP is a 128x128 grid of uint32 = 65536 bytes
    constexpr size_t kExpectedSize = ZMP_SIZE * ZMP_SIZE * sizeof(uint32_t);
    if (data.size() != kExpectedSize) {
        LOG_WARNING("DataRepository: ZMP '", zmpPath, "' unexpected size ",
                    data.size(), " (expected ", kExpectedSize, ")");
        return;
    }

    std::memcpy(zmpGrid_.data(), data.data(), kExpectedSize);
    zmpLoaded_ = true;

    // Count non-zero cells and find grid extent for diagnostic
    int nonZero = 0;
    int maxRow = -1, maxCol = -1;
    for (int r = 0; r < ZMP_SIZE; r++) {
        for (int c = 0; c < ZMP_SIZE; c++) {
            if (zmpGrid_[r * ZMP_SIZE + c] != 0) {
                nonZero++;
                if (r > maxRow) maxRow = r;
                if (c > maxCol) maxCol = c;
            }
        }
    }
    LOG_INFO("DataRepository: loaded ZMP '", zmpPath, "' - ",
             nonZero, "/", ZMP_SIZE * ZMP_SIZE, " non-zero cells, "
             "maxCol=", maxCol, " maxRow=", maxRow,
             " (if ~125/~111 → maps to 1024x768 FBO, if ~127/~127 → maps to 1002x668 visible)");

    // Derive zone bounding boxes from ZMP grid
    buildZmpZoneBounds();
}

void DataRepository::buildZmpZoneBounds() {
    zmpZoneBounds_.clear();
    if (!zmpLoaded_) return;

    // Scan the 128x128 ZMP grid and find the bounding box of each area ID's pixels.
    // The ZMP grid maps directly to the visible 1002×668 content area, so
    // (col/128, row/128) gives display UV without FBO conversion.
    struct RawRect { int minCol = ZMP_SIZE, maxCol = -1, minRow = ZMP_SIZE, maxRow = -1; };
    std::unordered_map<uint32_t, RawRect> areaRects;

    for (int row = 0; row < ZMP_SIZE; row++) {
        for (int col = 0; col < ZMP_SIZE; col++) {
            uint32_t areaId = zmpGrid_[row * ZMP_SIZE + col];
            if (areaId == 0) continue;
            auto& r = areaRects[areaId];
            r.minCol = std::min(r.minCol, col);
            r.maxCol = std::max(r.maxCol, col);
            r.minRow = std::min(r.minRow, row);
            r.maxRow = std::max(r.maxRow, row);
        }
    }

    // Map area ID bounding boxes → zone index bounding boxes.
    // Multiple area IDs may resolve to the same zone, so union their rects.
    constexpr float kInvSize = 1.0f / static_cast<float>(ZMP_SIZE);
    int mapped = 0;
    for (const auto& [areaId, rect] : areaRects) {
        int zi = zoneIndexForAreaId(areaId);
        if (zi < 0) continue;

        float uMin = static_cast<float>(rect.minCol) * kInvSize;
        float uMax = static_cast<float>(rect.maxCol + 1) * kInvSize;
        float vMin = static_cast<float>(rect.minRow) * kInvSize;
        float vMax = static_cast<float>(rect.maxRow + 1) * kInvSize;

        auto it = zmpZoneBounds_.find(zi);
        if (it != zmpZoneBounds_.end()) {
            // Union with existing rect for this zone
            it->second.uMin = std::min(it->second.uMin, uMin);
            it->second.uMax = std::max(it->second.uMax, uMax);
            it->second.vMin = std::min(it->second.vMin, vMin);
            it->second.vMax = std::max(it->second.vMax, vMax);
        } else {
            ZmpRect zr;
            zr.uMin = uMin; zr.uMax = uMax;
            zr.vMin = vMin; zr.vMax = vMax;
            zr.valid = true;
            zmpZoneBounds_[zi] = zr;
            mapped++;
        }
    }

    LOG_INFO("DataRepository: built ZMP zone bounds for ", mapped, " zones from ",
             areaRects.size(), " area IDs");
}

int DataRepository::zoneIndexForAreaId(uint32_t areaId) const {
    if (areaId == 0) return -1;
    auto it = areaIdToZoneIdx_.find(areaId);
    if (it != areaIdToZoneIdx_.end()) return it->second;

    // Fallback: check if areaId is a sub-zone whose parent is in our zone list.
    // Some ZMP cells reference sub-area IDs not directly in WorldMapArea.dbc.
    // Walk the AreaTable parent chain via exploreFlagByAreaId_ (which was built
    // from AreaTable.dbc and includes parentArea relationships).
    // For now, iterate zones looking for one whose overlays reference this areaId.
    for (int i = 0; i < static_cast<int>(zones_.size()); i++) {
        for (const auto& ov : zones_[i].overlays) {
            for (uint32_t ovAreaId : ov.areaIDs) {
                if (ovAreaId == areaId) return i;
            }
        }
    }
    return -1;
}

void DataRepository::loadPOIs(pipeline::AssetManager& assetManager) {
    if (poisLoaded_) return;
    poisLoaded_ = true;

    auto poiDbc = assetManager.loadDBC("AreaPOI.dbc");
    if (!poiDbc || !poiDbc->isLoaded()) {
        LOG_INFO("DataRepository: AreaPOI.dbc not found, skipping POI markers");
        return;
    }

    const uint32_t fieldCount = poiDbc->getFieldCount();
    if (fieldCount < 17) {
        LOG_WARNING("DataRepository: AreaPOI.dbc has too few fields (", fieldCount, ")");
        return;
    }

    // AreaPOI.dbc field layout (WotLK 3.3.5a):
    // 0:ID, 1:Importance, 2-10:Icon[9], 11:FactionID,
    // 12:X, 13:Y, 14:Z, 15:MapID,
    // 16:Name_lang (enUS), ...
    int loaded = 0;
    for (uint32_t i = 0; i < poiDbc->getRecordCount(); i++) {
        POI poi;
        poi.id = poiDbc->getUInt32(i, 0);
        poi.importance = poiDbc->getUInt32(i, 1);
        poi.iconType = poiDbc->getUInt32(i, 2);
        poi.factionId = poiDbc->getUInt32(i, 11);
        poi.wowX = poiDbc->getFloat(i, 12);
        poi.wowY = poiDbc->getFloat(i, 13);
        poi.wowZ = poiDbc->getFloat(i, 14);
        poi.mapId = poiDbc->getUInt32(i, 15);
        poi.name = poiDbc->getString(i, 16);

        if (poi.name.empty()) continue;

        poiMarkers_.push_back(std::move(poi));
        loaded++;
    }

    // Sort by importance ascending so high-importance POIs are drawn last (on top)
    std::sort(poiMarkers_.begin(), poiMarkers_.end(),
              [](const POI& a, const POI& b) { return a.importance < b.importance; });

    LOG_INFO("DataRepository: loaded ", loaded, " POI markers from AreaPOI.dbc");
}

void DataRepository::buildCosmicView(int /*expLevel*/) {
    cosmicMaps_.clear();

    if (game::isClassicLikeExpansion()) {
        // Vanilla/Classic: No cosmic view - skip from WORLD straight to CONTINENT.
        cosmicEnabled_ = false;
        LOG_INFO("DataRepository: Classic mode - cosmic view disabled");
        return;
    }

    cosmicEnabled_ = true;

    // Azeroth (EK + Kalimdor) - always present; bottom-right region of cosmic map
    cosmicMaps_.push_back({.mapId = 0, .label = "Azeroth", .uvLeft = 0.58f, .uvTop = 0.05f, .uvRight = 0.95f, .uvBottom = 0.95f});

    if (game::isActiveExpansion("tbc") || game::isActiveExpansion("wotlk")) {
        // TBC+: Add Outland - top-left region of cosmic map
        cosmicMaps_.push_back({.mapId = 530, .label = "Outland", .uvLeft = 0.05f, .uvTop = 0.10f, .uvRight = 0.55f, .uvBottom = 0.90f});
    }

    LOG_INFO("DataRepository: cosmic view built with ", cosmicMaps_.size(), " landmasses");
}

void DataRepository::buildAzerothView(int /*expLevel*/) {
    azerothRegions_.clear();

    // Clickable continent regions on the Azeroth world map (azeroth1-12.blp).
    // UV coordinates are approximate positions of each landmass on the combined map.

    // Eastern Kingdoms - right side of the Azeroth map
    azerothRegions_.push_back({.mapId = 0, .label = mapDisplayName(0), .uvLeft = 0.55f, .uvTop = 0.05f, .uvRight = 0.95f, .uvBottom = 0.95f});

    // Kalimdor - left side of the Azeroth map
    azerothRegions_.push_back({.mapId = 1, .label = mapDisplayName(1), .uvLeft = 0.05f, .uvTop = 0.10f, .uvRight = 0.45f, .uvBottom = 0.95f});

    if (game::isActiveExpansion("wotlk")) {
        // WotLK: Northrend - top-center of the Azeroth map
        azerothRegions_.push_back({.mapId = 571, .label = mapDisplayName(571), .uvLeft = 0.30f, .uvTop = 0.0f, .uvRight = 0.72f, .uvBottom = 0.28f});
    }

    LOG_INFO("DataRepository: Azeroth view built with ", azerothRegions_.size(), " continent regions");
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
