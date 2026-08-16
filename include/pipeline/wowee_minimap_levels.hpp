#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Minimap Multi-Level catalog (.wmnl) -
// novel replacement for the WorldMapTransforms.dbc +
// WorldMapOverlay.dbc pair that vanilla WoW used to
// describe zones with multiple vertical layers visible
// on the minimap (Stormwind has Old Town / Cathedral /
// Keep visible at separate altitudes; Dalaran has
// Sewers / Street / Above Street / Floating; Undercity
// has Throne / Inner Ring / Outer Ring / Canal / Sewer).
// Each entry binds one (mapId, areaId, levelIndex)
// triplet to its Z-range, minimap texture, and display
// label.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the WMS map catalog;
//         areaId references the WMS sub-area entry.
//   WMPX: WMNL acts as a per-level overlay on top of
//         the per-zone WMPX world-map mapping; if a
//         player's Z falls within a WMNL entry's
//         range, the WMPX overlay layer texture
//         switches to the WMNL.texturePath.
//
// Binary layout (little-endian):
//   magic[4]            = "WMNL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     levelId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32) / areaId (uint32)
//     levelIndex (uint8)         - 0=ground, 1=upper,
//                                   2=second-upper, etc.
//     pad0 (uint8) / pad1 (uint8) / pad2 (uint8)
//     minZ (float) / maxZ (float)  - world units
//     pathLen + texturePath        - minimap layer
//                                     texture (BLP/WOT)
//     labelLen + displayName       - UI label, e.g.
//                                     "Ground Floor"
//     iconColorRGBA (uint32)
struct WoweeMinimapLevels {
    struct Entry {
        uint32_t levelId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t areaId = 0;
        uint8_t levelIndex = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        float minZ = 0.0f;
        float maxZ = 0.0f;
        std::string texturePath;
        std::string displayName;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t levelId) const;

    // Returns the level entry whose Z-range contains the
    // player's current Z position, for the given (map,
    // area). Used by the minimap renderer at every camera
    // tick to swap the overlay layer when the player
    // crosses a floor boundary.
    [[nodiscard]] const Entry* findContainingZ(uint32_t mapId,
                                   uint32_t areaId,
                                   float z) const;

    // Returns all levels for one (map, area) sorted by
    // levelIndex. Used by the world-map UI to populate
    // the per-zone level-picker dropdown.
    [[nodiscard]] std::vector<const Entry*> findByArea(uint32_t mapId,
                                           uint32_t areaId) const;
};

class WoweeMinimapLevelsLoader {
public:
    static bool save(const WoweeMinimapLevels& cat,
                     const std::string& basePath);
    static WoweeMinimapLevels load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mnl* variants.
    //
    //   makeStormwind  - 3 levels (Old Town / Cathedral
    //                     District / Stormwind Keep
    //                     Throne Room) covering the
    //                     city's vertical extent at
    //                     mapId 0 / areaId 1519.
    //   makeDalaran    - 4 levels (Sewers / Street /
    //                     Above Street / Floating
    //                     Cathedral) - most vertical
    //                     city in Northrend.
    //   makeUndercity  - 5 levels (Throne / Inner Ring /
    //                     Outer Ring / Canal / Sewer) -
    //                     deepest vertical-layer city.
    static WoweeMinimapLevels makeStormwind(const std::string& catalogName);
    static WoweeMinimapLevels makeDalaran(const std::string& catalogName);
    static WoweeMinimapLevels makeUndercity(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
