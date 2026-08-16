// data_repository.hpp - DBC data loading, ZMP pixel map, and zone/POI/overlay storage.
// Extracted from WorldMap::loadZonesFromDBC, loadPOIData, buildCosmicView
// (Phase 5 of refactoring plan). SRP - all DBC parsing lives here.
#pragma once

#include "rendering/world_map/world_map_types.hpp"
#include <unordered_map>
#include <string>
#include <vector>
#include <array>
#include <cstdint>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {
namespace world_map {

class DataRepository {
public:
    /// Load all zone data from DBC files for the given map name.
    void loadZones(const std::string& mapName, pipeline::AssetManager& assetManager);

    /// Load area POI markers from AreaPOI.dbc.
    void loadPOIs(pipeline::AssetManager& assetManager);

    /// Build cosmic view entries for the active expansion (uses isActiveExpansion).
    void buildCosmicView(int expansionLevel = 0);

    /// Build Azeroth world-view continent regions for the active expansion.
    void buildAzerothView(int expansionLevel = 0);

    /// Load ZMP pixel map for the given continent name (e.g. "Azeroth").
    /// The ZMP is a 128x128 grid of uint32 AreaTable IDs.
    void loadZmpPixelMap(const std::string& continentName,
                         pipeline::AssetManager& assetManager);

    /// Determine expansion level from the active expansion profile.

    // --- Accessors ---
    std::vector<Zone>& zones() { return zones_; }
    [[nodiscard]] const std::vector<Zone>& zones() const { return zones_; }
    [[nodiscard]] int cosmicIdx() const { return cosmicIdx_; }
    [[nodiscard]] int worldIdx() const { return worldIdx_; }
    [[nodiscard]] int currentMapId() const { return currentMapId_; }
    [[nodiscard]] const std::vector<CosmicMapEntry>& cosmicMaps() const { return cosmicMaps_; }
    [[nodiscard]] const std::vector<CosmicMapEntry>& azerothRegions() const { return azerothRegions_; }
    [[nodiscard]] bool cosmicEnabled() const { return cosmicEnabled_; }
    [[nodiscard]] const std::vector<POI>& poiMarkers() const { return poiMarkers_; }

    [[nodiscard]] const std::unordered_map<uint32_t, uint32_t>& exploreFlagByAreaId() const { return exploreFlagByAreaId_; }
    [[nodiscard]] const std::unordered_map<uint32_t, std::string>& areaNameByAreaId() const { return areaNameByAreaId_; }

    /// ZMP pixel map accessors.
    static constexpr int ZMP_SIZE = 128;
    [[nodiscard]] const std::array<uint32_t, 128 * 128>& zmpGrid() const { return zmpGrid_; }
    [[nodiscard]] bool hasZmpData() const { return zmpLoaded_; }

    /// Look up zone index from an AreaTable ID (from ZMP). Returns -1 if not found.
    [[nodiscard]] int zoneIndexForAreaId(uint32_t areaId) const;

    /// Convert a render-space position from a physical map into the currently
    /// loaded display map using WorldMapTransforms.dbc.
    [[nodiscard]] glm::vec3 transformRenderPosition(uint32_t sourceMapId,
                                      const glm::vec3& renderPos) const;

    /// ZMP-derived bounding rectangles per zone index (UV [0,1] on display).
    [[nodiscard]] const std::unordered_map<int, ZmpRect>& zmpZoneBounds() const { return zmpZoneBounds_; }

    /// Reset all data (called on map change).
    void clear();

private:
    struct MapTransform {
        uint32_t sourceMapId = 0;
        uint32_t targetMapId = 0;
        float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
        float offsetX = 0.0f, offsetY = 0.0f;
    };

    std::vector<Zone> zones_;
    std::vector<MapTransform> mapTransforms_;
    std::vector<POI> poiMarkers_;
    std::vector<CosmicMapEntry> cosmicMaps_;
    std::vector<CosmicMapEntry> azerothRegions_;
    std::unordered_map<uint32_t, uint32_t> exploreFlagByAreaId_;
    std::unordered_map<uint32_t, std::string> areaNameByAreaId_;
    int cosmicIdx_ = -1;
    int worldIdx_ = -1;
    int currentMapId_ = -1;
    bool cosmicEnabled_ = true;
    bool poisLoaded_ = false;

    // ZMP pixel map: 128x128 grid of AreaTable IDs for continent-level hover
    std::array<uint32_t, 128 * 128> zmpGrid_{};
    bool zmpLoaded_ = false;
    // AreaID → zone index (zones_ vector) for quick resolution
    std::unordered_map<uint32_t, int> areaIdToZoneIdx_;
    // ZMP-derived bounding boxes per zone index (UV coords on display)
    std::unordered_map<int, ZmpRect> zmpZoneBounds_;

    /// Scan ZMP grid and build bounding boxes for each zone.
    void buildZmpZoneBounds();
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
