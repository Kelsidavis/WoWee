#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Light format (.wol) - novel replacement for WoW's
// Light.dbc / LightParams.dbc / LightIntBand.dbc / LightFloatBand.dbc
// stack. A WOL file holds a list of time-of-day keyframes for one
// zone, each capturing the ambient + directional + fog state at that
// moment. The renderer interpolates between adjacent keyframes by
// time-of-day.
//
// Binary layout (little-endian):
//   magic[4]            = "WOLA"
//   version (uint32)    = current 1
//   nameLen (uint32)
//   name bytes (nameLen)
//   keyframeCount (uint32)
//   keyframes (each):
//     timeOfDayMin (uint32)         -- 0..1439, minutes since midnight
//     ambientColor.rgb (3 × float)
//     directionalColor.rgb (3 × float)
//     directionalDir.xyz (3 × float)  -- unit vector pointing FROM
//                                        the sun TO the surface
//     fogColor.rgb (3 × float)
//     fogStart (float)              -- meters
//     fogEnd (float)                -- meters
struct WoweeLight {
    struct Keyframe {
        uint32_t timeOfDayMin = 0;
        glm::vec3 ambientColor{0.20f, 0.20f, 0.25f};
        glm::vec3 directionalColor{0.95f, 0.92f, 0.85f};
        glm::vec3 directionalDir{0.0f, -1.0f, 0.0f};
        glm::vec3 fogColor{0.65f, 0.70f, 0.78f};
        float fogStart = 80.0f;
        float fogEnd = 600.0f;
    };

    std::string name;                  // zone or scene name
    std::vector<Keyframe> keyframes;   // sorted by timeOfDayMin

    [[nodiscard]] bool isValid() const { return !keyframes.empty(); }
};

class WoweeLightLoader {
public:
    static bool save(const WoweeLight& light, const std::string& basePath);
    static WoweeLight load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Convenience: emit a 4-keyframe day/night cycle (dawn 6:00,
    // noon 12:00, dusk 18:00, midnight 0:00) with reasonable
    // outdoor defaults. Used by --gen-light to create a starter
    // file users can edit.
    static WoweeLight makeDefaultDayNight(const std::string& zoneName);

    // Preset variants for non-outdoor zones - these emit a
    // single-keyframe WOL since the lighting doesn't vary by
    // time-of-day for an enclosed scene.
    //
    //   makeCave        dim blue ambient + heavy fog
    //   makeDungeon     moody indoor torchlit + medium fog
    //   makeNight       dark blue ambient + far fog (night-only zone)
    static WoweeLight makeCave(const std::string& zoneName);
    static WoweeLight makeDungeon(const std::string& zoneName);
    static WoweeLight makeNight(const std::string& zoneName);

    // Lookup the interpolated lighting state at any time-of-day
    // (clamped to 0..1439 minutes). Linearly blends between the
    // two adjacent keyframes; wraps around midnight if the query
    // time falls between the last and first keyframe.
    static WoweeLight::Keyframe sampleAtTime(const WoweeLight& light,
                                              uint32_t timeMin);
};

} // namespace pipeline
} // namespace wowee
