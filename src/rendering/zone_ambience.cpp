#include "rendering/lighting_manager.hpp"

#include <algorithm>

namespace wowee::rendering {

namespace {

/// A zone that is darker than the light tables alone would make it.
///
/// The day and night cycle follows the server's clock everywhere, and these
/// zones are no exception: visiting Tirisfal during the day gives you daytime.
/// What sets them apart is that the sky is overcast through all of it, so the
/// light stays muted, foggy and deeply shadowed next to somewhere like Elwynn.
///
/// Duskwood was the only zone handled here, written as a pair of
/// `if (zone == 10)` branches, so adding a second meant copying both.
///
/// `visualTimeHours` is the hour the light bands are sampled at, or negative
/// to follow the world clock. Pinning the hour and darkening the sky are two
/// different things: Duskwood is canonically stuck at night, while Tirisfal
/// and Silverpine run a normal day and are overcast through all of it. Only
/// Duskwood pins its hour.
///
/// The ceilings are applied after the normal DBC and weather blend, so a clear
/// noon cannot wash the zone out, and a value the client already supplies that
/// is darker than the ceiling is kept.
struct DarkZone {
    uint32_t zoneId;
    float visualTimeHours;

    glm::vec3 ambientCeiling;
    glm::vec3 diffuseCeiling;

    glm::vec3 fogColor;
    float fogStartMax;
    float fogEndMax;
    float fogDensityMin;

    glm::vec3 skyTop;
    glm::vec3 skyMiddle;
    glm::vec3 skyBand1;
    glm::vec3 skyBand2;
    float cloudDensityMin;
    float horizonGlowMax;
};

constexpr DarkZone kDarkZones[] = {
    // Duskwood: trapped beneath a dark, fog-heavy sky. Blue-black, and the
    // fog is close enough to hide the far side of the road.
    {.zoneId = 10, .visualTimeHours = 22.0f,
     .ambientCeiling = {0.20f, 0.22f, 0.26f}, .diffuseCeiling = {0.26f, 0.28f, 0.32f},
     .fogColor = {0.075f, 0.095f, 0.11f}, .fogStartMax = 35.0f, .fogEndMax = 525.0f, .fogDensityMin = 0.006f,
     .skyTop = {0.025f, 0.035f, 0.055f}, .skyMiddle = {0.055f, 0.070f, 0.085f},
     .skyBand1 = {0.075f, 0.090f, 0.105f}, .skyBand2 = {0.095f, 0.105f, 0.115f},
     .cloudDensityMin = 0.88f, .horizonGlowMax = 0.08f},

    // Tirisfal Glades: the day runs, and the sky is overcast through all of
    // it. Dark and shadowy rather than dark as night, with the sickly green
    // cast of the plague in the haze. The hour is not pinned.
    {.zoneId = 85, .visualTimeHours = -1.0f,
     .ambientCeiling = {0.26f, 0.30f, 0.25f}, .diffuseCeiling = {0.34f, 0.38f, 0.31f},
     .fogColor = {0.13f, 0.17f, 0.13f}, .fogStartMax = 60.0f, .fogEndMax = 460.0f, .fogDensityMin = 0.005f,
     .skyTop = {0.16f, 0.20f, 0.17f}, .skyMiddle = {0.20f, 0.24f, 0.20f},
     .skyBand1 = {0.23f, 0.27f, 0.23f}, .skyBand2 = {0.26f, 0.29f, 0.25f},
     .cloudDensityMin = 0.80f, .horizonGlowMax = 0.10f},

    // Silverpine Forest: the same overcast, greyer and a little less green
    // the further south it runs.
    {.zoneId = 130, .visualTimeHours = -1.0f,
     .ambientCeiling = {0.25f, 0.28f, 0.26f}, .diffuseCeiling = {0.32f, 0.35f, 0.33f},
     .fogColor = {0.13f, 0.15f, 0.14f}, .fogStartMax = 60.0f, .fogEndMax = 480.0f, .fogDensityMin = 0.005f,
     .skyTop = {0.17f, 0.19f, 0.19f}, .skyMiddle = {0.21f, 0.23f, 0.22f},
     .skyBand1 = {0.24f, 0.26f, 0.25f}, .skyBand2 = {0.27f, 0.28f, 0.28f},
     .cloudDensityMin = 0.80f, .horizonGlowMax = 0.10f},
};

const DarkZone* findDarkZone(uint32_t zoneId) {
    for (const DarkZone& zone : kDarkZones) {
        if (zone.zoneId == zoneId) return &zone;
    }
    return nullptr;
}

}  // namespace

float resolveZoneVisualTimeHours(uint32_t zoneId, bool isIndoors, float worldTimeHours) {
    if (isIndoors) return worldTimeHours;
    const DarkZone* zone = findDarkZone(zoneId);
    if (!zone || zone->visualTimeHours < 0.0f) return worldTimeHours;
    return zone->visualTimeHours;
}

void applyZoneAmbienceOverride(uint32_t zoneId, LightingParams& params) {
    const DarkZone* zone = findDarkZone(zoneId);
    if (!zone) return;

    params.ambientColor = glm::min(params.ambientColor, zone->ambientCeiling);
    params.diffuseColor = glm::min(params.diffuseColor, zone->diffuseCeiling);

    params.fogColor = zone->fogColor;
    params.fogStart = std::min(params.fogStart, zone->fogStartMax);
    params.fogEnd = std::min(params.fogEnd, zone->fogEndMax);
    params.fogDensity = std::max(params.fogDensity, zone->fogDensityMin);

    params.skyTopColor = zone->skyTop;
    params.skyMiddleColor = zone->skyMiddle;
    params.skyBand1Color = zone->skyBand1;
    params.skyBand2Color = zone->skyBand2;
    params.cloudDensity = std::max(params.cloudDensity, zone->cloudDensityMin);
    params.horizonGlow = std::min(params.horizonGlow, zone->horizonGlowMax);
}

} // namespace wowee::rendering
