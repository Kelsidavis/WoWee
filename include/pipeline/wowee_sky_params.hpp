#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Sky Parameters catalog (.wskp) - novel
// replacement for the LightParams.dbc + Light.dbc pair
// that vanilla WoW used to drive the per-zone diurnal
// sky cycle: sky-dome zenith and horizon colors, sun
// angle and color, fog falloff distances, cloud layer
// opacity and drift speed. Each entry binds one
// (mapId, areaId, timeOfDayHour) triplet to its sky
// rendering parameters; the renderer interpolates
// between adjacent entries when the in-game clock
// crosses an hour boundary.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the WMS map catalog;
//         areaId references the WMS sub-area entry.
//   WOLA: WSKP supersedes WOLA's outdoor-light catalog
//         where the latter exists; WOLA remains for
//         backward-compat with engine code that hasn't
//         migrated yet.
//
// Binary layout (little-endian):
//   magic[4]            = "WSKP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     skyId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32) / areaId (uint32)
//     timeOfDayHour (uint8)      - 0..23 keyframe hour
//     pad0 (uint8) / pad1 (uint8) / pad2 (uint8)
//     zenithColor (uint32)       - RGBA top-of-sky
//     horizonColor (uint32)      - RGBA at horizon
//     sunColor (uint32)          - RGBA sun disc tint
//     sunAngleDeg (float)        - 0..360 azimuth
//     fogStartYards (float)      - distance fog begins
//     fogEndYards (float)        - distance fog opaque
//     cloudOpacity (uint8)       - 0..255 cloud layer
//                                   alpha
//     cloudSpeedX10 (uint8)      - wind speed in
//                                   tenths-mph (0..255
//                                   = 0..25.5 mph drift
//                                   rate for the cloud
//                                   layer)
//     pad3 (uint8) / pad4 (uint8)
//     iconColorRGBA (uint32)
struct WoweeSkyParams {
    struct Entry {
        uint32_t skyId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t areaId = 0;
        uint8_t timeOfDayHour = 12;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t zenithColor = 0xFF000000u;
        uint32_t horizonColor = 0xFF000000u;
        uint32_t sunColor = 0xFFFFFFFFu;
        float sunAngleDeg = 0.0f;
        float fogStartYards = 100.0f;
        float fogEndYards = 500.0f;
        uint8_t cloudOpacity = 128;
        uint8_t cloudSpeedX10 = 30;     // 3.0 mph
        uint8_t pad3 = 0;
        uint8_t pad4 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t skyId) const;

    // Returns all entries for one (map, area) sorted by
    // hour. Used by the sky renderer to build the
    // diurnal interpolation curve at zone load time.
    [[nodiscard]] std::vector<const Entry*> findByArea(uint32_t mapId,
                                           uint32_t areaId) const;
};

class WoweeSkyParamsLoader {
public:
    static bool save(const WoweeSkyParams& cat,
                     const std::string& basePath);
    static WoweeSkyParams load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-skp* variants.
    //
    //   makeStormwindDay  - 4 keyframes for Stormwind's
    //                        diurnal cycle (Dawn 6AM /
    //                        Noon 12 / Dusk 18 /
    //                        Midnight 0).
    //   makeNorthrendArctic - 4 cold steel-blue keyframes
    //                          for Northrend zones (Dawn /
    //                          Noon / Dusk / Midnight).
    //   makeOutlandHellfire - 3 keyframes for Outland's
    //                          iconic red/orange skies
    //                          (Dawn / Noon / Sunset; no
    //                          midnight keyframe - Outland
    //                          is permanently bright).
    static WoweeSkyParams makeStormwindDay(const std::string& catalogName);
    static WoweeSkyParams makeNorthrendArctic(const std::string& catalogName);
    static WoweeSkyParams makeOutlandHellfire(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
