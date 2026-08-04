#include "rendering/lighting_manager.hpp"
#include <glm/gtc/constants.hpp>
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cmath>
#include <ctime>

namespace wowee {
namespace rendering {

// Light coordinate scaling (test with 1.0f first, then try 36.0f if distances seem off)
constexpr float LIGHT_COORD_SCALE = 1.0f;

// WoW's Light.dbc stores time-of-day as half-minutes (0..2879).
// 24 hours × 60 minutes × 2 = 2880 half-minute ticks per day cycle.
constexpr uint16_t kHalfMinutesPerDay = 2880;

// Maximum volumes to blend (top 2-4)
constexpr size_t MAX_BLEND_VOLUMES = 2;

LightingManager::LightingManager() {
    // Set fallback lighting (Elwynn Forest-ish outdoor daytime)
    fallbackParams_.ambientColor = glm::vec3(0.5f, 0.5f, 0.6f);
    fallbackParams_.diffuseColor = glm::vec3(1.0f, 0.95f, 0.85f);
    fallbackParams_.directionalDir = glm::normalize(glm::vec3(0.3f, -0.7f, 0.6f));
    fallbackParams_.fogColor = glm::vec3(0.6f, 0.7f, 0.85f);
    fallbackParams_.fogStart = 300.0f;
    fallbackParams_.fogEnd = 1500.0f;
    fallbackParams_.skyTopColor = glm::vec3(0.4f, 0.6f, 0.9f);
    fallbackParams_.skyMiddleColor = glm::vec3(0.6f, 0.75f, 0.95f);

    currentParams_ = fallbackParams_;
}

LightingManager::~LightingManager() {
}

bool LightingManager::initialize(pipeline::AssetManager* assetManager) {
    if (!assetManager) {
        LOG_ERROR("LightingManager::initialize: null AssetManager");
        return false;
    }

    assetManager_ = assetManager;

    // Load DBCs (non-fatal if missing, will use fallback lighting)
    loadLightDbc(assetManager);
    loadLightParamsDbc(assetManager);
    loadLightSkyboxDbc(assetManager);
    loadLightBandDbcs(assetManager);

    initialized_ = true;
    LOG_INFO("LightingManager initialized: ", lightVolumesByMap_.size(), " maps with lighting");
    return true;
}

bool LightingManager::loadLightDbc(pipeline::AssetManager* assetManager) {
    auto dbcData = assetManager->readFile("DBFilesClient\\Light.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("Light.dbc not found, using fallback lighting");
        return false;
    }

    auto dbc = std::make_unique<pipeline::DBCFile>();
    if (!dbc->load(dbcData)) {
        LOG_ERROR("Failed to load Light.dbc");
        return false;
    }

    uint32_t recordCount = dbc->getRecordCount();
    LOG_INFO("Loading Light.dbc: ", recordCount, " light volumes");

    // Parse light volumes
    // Light.dbc structure (WotLK 3.3.5a):
    // 0: uint32 ID
    // 1: uint32 MapID
    // 2-4: float X, Z, Y (note: z and y swapped!)
    // 5: float FalloffStart (inner radius)
    // 6: float FalloffEnd (outer radius)
    // 7: uint32 LightParamsID (clear weather)
    // 8: uint32 LightParamsID (overcast/rain)
    // 9: uint32 LightParamsID (underwater)
    // ... more params for death, phases, etc.

    const auto* activeLayout = pipeline::getActiveDBCLayout();
    const auto* lL = activeLayout ? activeLayout->getLayout("Light") : nullptr;

    for (uint32_t i = 0; i < recordCount; ++i) {
        LightVolume volume;
        volume.lightId = dbc->getUInt32(i, lL ? (*lL)["ID"] : 0);
        volume.mapId = dbc->getUInt32(i, lL ? (*lL)["MapID"] : 1);

        // Position (note: DBC stores as x,z,y - need to swap!)
        float x = dbc->getFloat(i, lL ? (*lL)["X"] : 2);
        float z = dbc->getFloat(i, lL ? (*lL)["Z"] : 3);
        float y = dbc->getFloat(i, lL ? (*lL)["Y"] : 4);
        volume.position = glm::vec3(x, y, z);  // Convert to x,y,z

        volume.innerRadius = dbc->getFloat(i, lL ? (*lL)["InnerRadius"] : 5);
        volume.outerRadius = dbc->getFloat(i, lL ? (*lL)["OuterRadius"] : 6);

        // LightParams IDs for different conditions
        volume.lightParamsId = dbc->getUInt32(i, lL ? (*lL)["LightParamsID"] : 7);
        if (dbc->getFieldCount() > 8) {
            volume.lightParamsIdRain = dbc->getUInt32(i, lL ? (*lL)["LightParamsIDRain"] : 8);
        }
        if (dbc->getFieldCount() > 9) {
            volume.lightParamsIdUnderwater = dbc->getUInt32(i, lL ? (*lL)["LightParamsIDUnderwater"] : 9);
        }

        // Add to map-specific list
        lightVolumesByMap_[volume.mapId].push_back(volume);
    }

    LOG_INFO("Loaded ", lightVolumesByMap_.size(), " maps with lighting volumes");
    return true;
}

bool LightingManager::loadLightParamsDbc(pipeline::AssetManager* assetManager) {
    auto dbcData = assetManager->readFile("DBFilesClient\\LightParams.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("LightParams.dbc not found");
        return false;
    }

    auto dbc = std::make_unique<pipeline::DBCFile>();
    if (!dbc->load(dbcData)) {
        LOG_ERROR("Failed to load LightParams.dbc");
        return false;
    }

    uint32_t recordCount = dbc->getRecordCount();
    LOG_INFO("Loaded LightParams.dbc: ", recordCount, " profiles");

    // Create profile entries (will be populated by band loading)
    const auto* lpL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("LightParams") : nullptr;
    for (uint32_t i = 0; i < recordCount; ++i) {
        uint32_t paramId = dbc->getUInt32(i, lpL ? (*lpL)["LightParamsID"] : 0);
        LightParamsProfile profile;
        profile.lightParamsId = paramId;
        if (dbc->getFieldCount() > 2) {
            profile.lightSkyboxId = dbc->getUInt32(i, 2);
        }
        lightParamsProfiles_[paramId] = profile;
    }

    return true;
}

bool LightingManager::loadLightSkyboxDbc(pipeline::AssetManager* assetManager) {
    auto dbcData = assetManager->readFile("DBFilesClient\\LightSkybox.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("LightSkybox.dbc not found");
        return false;
    }

    pipeline::DBCFile dbc;
    if (!dbc.load(dbcData) || dbc.getFieldCount() < 2) {
        LOG_ERROR("Failed to load LightSkybox.dbc");
        return false;
    }

    lightSkyboxPaths_.clear();
    for (uint32_t i = 0; i < dbc.getRecordCount(); ++i) {
        const uint32_t id = dbc.getUInt32(i, 0);
        std::string path = dbc.getString(i, 1);
        if (id != 0 && !path.empty()) lightSkyboxPaths_[id] = std::move(path);
    }
    LOG_INFO("Loaded LightSkybox.dbc: ", lightSkyboxPaths_.size(), " model paths");
    return !lightSkyboxPaths_.empty();
}

bool LightingManager::loadLightBandDbcs(pipeline::AssetManager* assetManager) {
    // Load LightIntBand.dbc for RGB color curves (18 channels per LightParams)
    auto intBandData = assetManager->readFile("DBFilesClient\\LightIntBand.dbc");
    if (!intBandData.empty()) {
        auto dbc = std::make_unique<pipeline::DBCFile>();
        if (dbc->load(intBandData)) {
            LOG_INFO("Loaded LightIntBand.dbc: ", dbc->getRecordCount(), " color bands");

            // Parse int bands
            // Structure: ID, Entry (block index), NumValues, Time[16], Color[16]
            // Block index = LightParamsID * 18 + channel
            const auto* libL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("LightIntBand") : nullptr;
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                uint32_t blockIndex = dbc->getUInt32(i, libL ? (*libL)["BlockIndex"] : 1);
                uint32_t lightParamsId = blockIndex / 18;
                uint32_t channelIndex = blockIndex % 18;

                auto it = lightParamsProfiles_.find(lightParamsId);
                if (it == lightParamsProfiles_.end()) continue;

                if (channelIndex >= LightParamsProfile::COLOR_CHANNEL_COUNT) continue;

                ColorBand& band = it->second.colorBands[channelIndex];
                band.numKeyframes = dbc->getUInt32(i, libL ? (*libL)["NumKeyframes"] : 2);
                if (band.numKeyframes > 16) band.numKeyframes = 16;

                // Read time keys (field 3-18) - stored as uint16 half-minutes
                uint32_t timeKeyBase = libL ? (*libL)["TimeKey0"] : 3;
                for (uint8_t k = 0; k < band.numKeyframes && k < 16; ++k) {
                    uint32_t timeValue = dbc->getUInt32(i, timeKeyBase + k);
                    band.times[k] = static_cast<uint16_t>(timeValue % kHalfMinutesPerDay);  // Clamp to valid range
                }

                // Read color values (field 19-34) - stored as BGRA packed uint32
                uint32_t valueBase = libL ? (*libL)["Value0"] : 19;
                for (uint8_t k = 0; k < band.numKeyframes && k < 16; ++k) {
                    uint32_t colorBGRA = dbc->getUInt32(i, valueBase + k);
                    band.colors[k] = dbcColorToVec3(colorBGRA);
                }
            }
        }
    }

    // Load LightFloatBand.dbc for fog/intensity curves (6 channels per LightParams)
    auto floatBandData = assetManager->readFile("DBFilesClient\\LightFloatBand.dbc");
    if (!floatBandData.empty()) {
        auto dbc = std::make_unique<pipeline::DBCFile>();
        if (dbc->load(floatBandData)) {
            LOG_INFO("Loaded LightFloatBand.dbc: ", dbc->getRecordCount(), " float bands");

            // Parse float bands
            // Structure: ID, Entry (block index), NumValues, Time[16], Value[16]
            // Block index = LightParamsID * 6 + channel
            const auto* lfbL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("LightFloatBand") : nullptr;
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                uint32_t blockIndex = dbc->getUInt32(i, lfbL ? (*lfbL)["BlockIndex"] : 1);
                uint32_t lightParamsId = blockIndex / 6;
                uint32_t channelIndex = blockIndex % 6;

                auto it = lightParamsProfiles_.find(lightParamsId);
                if (it == lightParamsProfiles_.end()) continue;

                if (channelIndex >= LightParamsProfile::FLOAT_CHANNEL_COUNT) continue;

                FloatBand& band = it->second.floatBands[channelIndex];
                band.numKeyframes = dbc->getUInt32(i, lfbL ? (*lfbL)["NumKeyframes"] : 2);
                if (band.numKeyframes > 16) band.numKeyframes = 16;

                // Read time keys (field 3-18)
                uint32_t timeKeyBase = lfbL ? (*lfbL)["TimeKey0"] : 3;
                for (uint8_t k = 0; k < band.numKeyframes && k < 16; ++k) {
                    uint32_t timeValue = dbc->getUInt32(i, timeKeyBase + k);
                    band.times[k] = static_cast<uint16_t>(timeValue % kHalfMinutesPerDay);  // Clamp to valid range
                }

                // Read float values (field 19-34)
                uint32_t valueBase = lfbL ? (*lfbL)["Value0"] : 19;
                for (uint8_t k = 0; k < band.numKeyframes && k < 16; ++k) {
                    band.values[k] = dbc->getFloat(i, valueBase + k);
                }
            }
        }
    }

    LOG_INFO("Loaded bands for ", lightParamsProfiles_.size(), " LightParams profiles");
    return true;
}

void LightingManager::update(const glm::vec3& playerPos, uint32_t mapId, uint32_t zoneId,
                              float gameTime,
                              bool isRaining, bool isUnderwater) {
    if (!initialized_) return;

    // Update time
    if (!manualTime_) {
        if (gameTime >= 0.0f) {
            // Server-sent game time, in hours since midnight. It was being
            // divided by 86400 as though it were seconds, against a field that
            // held a raw packed bitfield — so the sky's clock was a number with
            // no relation to the hour, and it looked plausible because any
            // value lands somewhere in the day.
            timeOfDay_ = std::fmod(gameTime / 24.0f, 1.0f);  // 0.0-1.0
        } else {
            // Fallback: use real time for day/night cycle
            std::time_t now = std::time(nullptr);
            std::tm* localTime = std::localtime(&now);
            float secondsSinceMidnight = localTime->tm_hour * 3600.0f +
                                          localTime->tm_min * 60.0f +
                                          localTime->tm_sec;
            timeOfDay_ = secondsSinceMidnight / 86400.0f;  // 0.0-1.0
        }
    }
    // else: manualTime_ is set, use timeOfDay_ as-is

    // Duskwood's visible sky is permanently late-night even while the global
    // world clock continues normally for gameplay and every other zone.
    visualTimeOfDayHours_ = resolveZoneVisualTimeHours(
        zoneId, isIndoors_, timeOfDay_ * 24.0f);

    // Convert visual time to half-minutes (WoW DBC format: 0-2879).
    const float visualDayFraction = visualTimeOfDayHours_ / 24.0f;
    uint16_t timeHalfMinutes = static_cast<uint16_t>(visualDayFraction * static_cast<float>(kHalfMinutesPerDay)) % kHalfMinutesPerDay;

    // Update player position and map
    currentPlayerPos_ = playerPos;
    currentMapId_ = mapId;

    // Find light volumes for blending
    activeVolumes_ = findLightVolumes(playerPos, mapId);
    activeSkyboxPath_.clear();
    if (!isIndoors_) {
        for (const auto& wv : activeVolumes_) {
            const uint32_t paramsId = selectLightParamsId(wv.volume, isRaining, isUnderwater);
            auto profileIt = lightParamsProfiles_.find(paramsId);
            if (profileIt == lightParamsProfiles_.end() || profileIt->second.lightSkyboxId == 0) continue;
            auto skyIt = lightSkyboxPaths_.find(profileIt->second.lightSkyboxId);
            if (skyIt != lightSkyboxPaths_.end()) {
                activeSkyboxPath_ = skyIt->second;
                break;
            }
        }
    }

    // Sample and blend lighting
    LightingParams newParams;

    if (isIndoors_) {
        // Indoor lighting: static ambient-heavy
        newParams.ambientColor = glm::vec3(0.6f, 0.6f, 0.65f);
        newParams.diffuseColor = glm::vec3(0.3f, 0.3f, 0.35f);
        newParams.fogColor = glm::vec3(0.3f, 0.3f, 0.4f);
        newParams.fogStart = 50.0f;
        newParams.fogEnd = 300.0f;
    } else if (!activeVolumes_.empty()) {
        // Outdoor with DBC lighting - blend multiple volumes
        newParams = {}; // Zero-initialize
        glm::vec3 blendedDir(0.0f);  // Accumulate weighted directions

        for (const auto& wv : activeVolumes_) {
            uint32_t lightParamsId = selectLightParamsId(wv.volume, isRaining, isUnderwater);
            auto it = lightParamsProfiles_.find(lightParamsId);

            if (it != lightParamsProfiles_.end()) {
                LightingParams params = sampleLightParams(&it->second, timeHalfMinutes);

                // Blend this volume's contribution
                newParams.ambientColor += params.ambientColor * wv.weight;
                newParams.diffuseColor += params.diffuseColor * wv.weight;
                newParams.fogColor += params.fogColor * wv.weight;
                newParams.skyTopColor += params.skyTopColor * wv.weight;
                newParams.skyMiddleColor += params.skyMiddleColor * wv.weight;
                newParams.skyBand1Color += params.skyBand1Color * wv.weight;
                newParams.skyBand2Color += params.skyBand2Color * wv.weight;

                newParams.fogStart += params.fogStart * wv.weight;
                newParams.fogEnd += params.fogEnd * wv.weight;
                newParams.fogDensity += params.fogDensity * wv.weight;
                newParams.cloudDensity += params.cloudDensity * wv.weight;
                newParams.horizonGlow += params.horizonGlow * wv.weight;

                // Blend direction weighted (normalize at end)
                blendedDir += params.directionalDir * wv.weight;
            }
        }

        // Normalize blended direction
        float blendedDirLenSq = glm::dot(blendedDir, blendedDir);
        if (blendedDirLenSq > 1e-6f) {
            newParams.directionalDir = blendedDir * glm::inversesqrt(blendedDirLenSq);
        } else {
            // Fallback if all directions cancelled out
            newParams.directionalDir = glm::vec3(0.3f, -0.7f, 0.6f);
        }
    } else {
        // No light volume, use fallback with time-based animation
        newParams = fallbackParams_;

        // Animate sun direction
        float angle = timeOfDay_ * glm::two_pi<float>();
        newParams.directionalDir = glm::normalize(glm::vec3(
            std::sin(angle) * 0.6f,
            -0.6f + std::cos(angle) * 0.4f,
            std::cos(angle) * 0.6f
        ));

        // Time-of-day color adjustments
        if (timeOfDay_ < 0.25f || timeOfDay_ > 0.75f) {
            // Night: darker, bluer
            float nightness = (timeOfDay_ < 0.25f) ? (0.25f - timeOfDay_) * 4.0f
                                                    : (timeOfDay_ - 0.75f) * 4.0f;
            newParams.ambientColor *= (0.3f + 0.7f * (1.0f - nightness));
            newParams.diffuseColor *= (0.2f + 0.8f * (1.0f - nightness));
            newParams.ambientColor.b += nightness * 0.1f;
        }
    }

    if (!isIndoors_) {
        applyZoneAmbienceOverride(zoneId, newParams);
    }

    // Smooth temporal blending to avoid snapping (5.0 = blend rate)
    float deltaTime = 0.016f;  // Assume ~60 FPS for now
    float blendFactor = 1.0f - std::exp(-deltaTime * 5.0f);

    currentParams_.ambientColor = glm::mix(currentParams_.ambientColor, newParams.ambientColor, blendFactor);
    currentParams_.diffuseColor = glm::mix(currentParams_.diffuseColor, newParams.diffuseColor, blendFactor);
    currentParams_.fogColor = glm::mix(currentParams_.fogColor, newParams.fogColor, blendFactor);
    currentParams_.skyTopColor = glm::mix(currentParams_.skyTopColor, newParams.skyTopColor, blendFactor);
    currentParams_.skyMiddleColor = glm::mix(currentParams_.skyMiddleColor, newParams.skyMiddleColor, blendFactor);
    currentParams_.skyBand1Color = glm::mix(currentParams_.skyBand1Color, newParams.skyBand1Color, blendFactor);
    currentParams_.skyBand2Color = glm::mix(currentParams_.skyBand2Color, newParams.skyBand2Color, blendFactor);
    currentParams_.fogStart = glm::mix(currentParams_.fogStart, newParams.fogStart, blendFactor);
    currentParams_.fogEnd = glm::mix(currentParams_.fogEnd, newParams.fogEnd, blendFactor);
    currentParams_.fogDensity = glm::mix(currentParams_.fogDensity, newParams.fogDensity, blendFactor);
    currentParams_.cloudDensity = glm::mix(currentParams_.cloudDensity, newParams.cloudDensity, blendFactor);
    currentParams_.horizonGlow = glm::mix(currentParams_.horizonGlow, newParams.horizonGlow, blendFactor);
    currentParams_.directionalDir = glm::normalize(glm::mix(currentParams_.directionalDir, newParams.directionalDir, blendFactor));
}

std::vector<LightingManager::WeightedVolume> LightingManager::findLightVolumes(const glm::vec3& playerPos, uint32_t mapId) const {
    auto it = lightVolumesByMap_.find(mapId);
    if (it == lightVolumesByMap_.end()) {
        return {};
    }

    const std::vector<LightVolume>& volumes = it->second;
    if (volumes.empty()) {
        return {};
    }

    // Collect all volumes with weight > 0
    std::vector<WeightedVolume> weighted;
    weighted.reserve(volumes.size());

    for (const auto& volume : volumes) {
        // Apply coordinate scaling (test with 1.0f, try 36.0f if distances are off)
        glm::vec3 scaledPos = volume.position * LIGHT_COORD_SCALE;
        glm::vec3 toPlayer = playerPos - scaledPos;
        float distSq = glm::dot(toPlayer, toPlayer);

        float weight = 0.0f;
        if (distSq <= volume.innerRadius * volume.innerRadius) {
            // Inside inner radius: full weight
            weight = 1.0f;
        } else if (distSq < volume.outerRadius * volume.outerRadius) {
            // Between inner and outer: fade out with smoothstep (sqrt needed for interpolation)
            float dist = std::sqrt(distSq);
            float t = (dist - volume.innerRadius) / (volume.outerRadius - volume.innerRadius);
            t = glm::clamp(t, 0.0f, 1.0f);
            weight = 1.0f - (t * t * (3.0f - 2.0f * t));  // Smoothstep
        }

        if (weight > 0.0f) {
            weighted.push_back({&volume, weight});

            // One-time diagnostic on the first map/frame this function fires —
            // was logging the first three volumes EVERY FRAME, generating a
            // continuous LOG_INFO stream during normal gameplay.
            static int diagFramesRemaining = 1;
            if (diagFramesRemaining > 0 && weighted.size() <= 3) {
                LOG_INFO("Light volume ", volume.lightId, ": distSq=", distSq,
                         " inner=", volume.innerRadius, " outer=", volume.outerRadius,
                         " weight=", weight);
                if (weighted.size() == 3) --diagFramesRemaining;
            }
        }
    }

    if (weighted.empty()) {
        return {};
    }

    // Keep top N volumes by weight (partial sort is O(n) vs O(n log n) for full sort)
    if (weighted.size() > MAX_BLEND_VOLUMES) {
        std::partial_sort(weighted.begin(),
                          weighted.begin() + MAX_BLEND_VOLUMES,
                          weighted.end(),
                          [](const WeightedVolume& a, const WeightedVolume& b) {
                              return a.weight > b.weight;
                          });
        weighted.resize(MAX_BLEND_VOLUMES);
    } else {
        std::sort(weighted.begin(), weighted.end(),
                  [](const WeightedVolume& a, const WeightedVolume& b) {
                      return a.weight > b.weight;
                  });
    }

    // Normalize weights to sum to 1.0
    float totalWeight = 0.0f;
    for (const auto& wv : weighted) {
        totalWeight += wv.weight;
    }

    if (totalWeight > 0.0f) {
        for (auto& wv : weighted) {
            wv.weight /= totalWeight;
        }
    }

    return weighted;
}

uint32_t LightingManager::selectLightParamsId(const LightVolume* volume, bool isRaining, bool isUnderwater) const {
    if (!volume) return 0;

    // Select appropriate LightParams based on conditions
    if (isUnderwater && volume->lightParamsIdUnderwater != 0) {
        return volume->lightParamsIdUnderwater;
    } else if (isRaining && volume->lightParamsIdRain != 0) {
        return volume->lightParamsIdRain;
    } else {
        return volume->lightParamsId;
    }
}

LightingParams LightingManager::sampleLightParams(const LightParamsProfile* profile, uint16_t timeHalfMinutes) const {
    if (!profile) return fallbackParams_;

    LightingParams params;

    // Sample color bands
    params.ambientColor = sampleColorBand(profile->colorBands[LightParamsProfile::AMBIENT_COLOR], timeHalfMinutes);
    params.diffuseColor = sampleColorBand(profile->colorBands[LightParamsProfile::DIFFUSE_COLOR], timeHalfMinutes);
    params.fogColor = sampleColorBand(profile->colorBands[LightParamsProfile::FOG_COLOR], timeHalfMinutes);
    params.skyTopColor = sampleColorBand(profile->colorBands[LightParamsProfile::SKY_TOP_COLOR], timeHalfMinutes);
    params.skyMiddleColor = sampleColorBand(profile->colorBands[LightParamsProfile::SKY_MIDDLE_COLOR], timeHalfMinutes);
    params.skyBand1Color = sampleColorBand(profile->colorBands[LightParamsProfile::SKY_BAND1_COLOR], timeHalfMinutes);
    params.skyBand2Color = sampleColorBand(profile->colorBands[LightParamsProfile::SKY_BAND2_COLOR], timeHalfMinutes);

    // Sample float bands
    params.fogEnd = sampleFloatBand(profile->floatBands[LightParamsProfile::FOG_END], timeHalfMinutes);
    float fogStartScalar = sampleFloatBand(profile->floatBands[LightParamsProfile::FOG_START_SCALAR], timeHalfMinutes);
    params.fogStart = params.fogEnd * fogStartScalar;  // Start is a scalar of end distance
    params.fogDensity = sampleFloatBand(profile->floatBands[LightParamsProfile::FOG_DENSITY], timeHalfMinutes);
    params.cloudDensity = sampleFloatBand(profile->floatBands[LightParamsProfile::CLOUD_DENSITY], timeHalfMinutes);

    // Debug logging for fog params (first few samples only)
    static int debugCount = 0;
    if (debugCount < 3) {
        LOG_INFO("Fog params: start=", params.fogStart, " end=", params.fogEnd,
                 " color=(", params.fogColor.r, ",", params.fogColor.g, ",", params.fogColor.b, ")");
        debugCount++;
    }

    // Compute sun direction from time
    float angle = (timeHalfMinutes / static_cast<float>(kHalfMinutesPerDay)) * glm::two_pi<float>();
    params.directionalDir = glm::normalize(glm::vec3(
        std::sin(angle) * 0.6f,
        -0.6f + std::cos(angle) * 0.4f,
        std::cos(angle) * 0.6f
    ));

    return params;
}

glm::vec3 LightingManager::sampleColorBand(const ColorBand& band, uint16_t timeHalfMinutes) const {
    if (band.numKeyframes == 0) {
        return glm::vec3(0.5f);  // Fallback gray
    }

    if (band.numKeyframes == 1) {
        return band.colors[0];  // Single keyframe
    }

    // Safer initialization: default to wrapping last→first
    uint8_t idx1 = band.numKeyframes - 1;
    uint8_t idx2 = 0;

    // Find surrounding keyframes
    for (uint8_t i = 0; i < band.numKeyframes; ++i) {
        if (timeHalfMinutes < band.times[i]) {
            idx2 = i;
            idx1 = (i > 0) ? (i - 1) : (band.numKeyframes - 1);  // Wrap to last
            break;
        }
    }

    // Calculate interpolation factor
    uint16_t t1 = band.times[idx1];
    uint16_t t2 = band.times[idx2];

    // Handle midnight wrap
    uint16_t timeSpan = (t2 > t1) ? (t2 - t1) : (kHalfMinutesPerDay - t1 + t2);
    uint16_t elapsed = (timeHalfMinutes >= t1) ? (timeHalfMinutes - t1) : (kHalfMinutesPerDay - t1 + timeHalfMinutes);

    float t = (timeSpan > 0) ? (static_cast<float>(elapsed) / static_cast<float>(timeSpan)) : 0.0f;
    t = glm::clamp(t, 0.0f, 1.0f);

    // Linear interpolation
    return glm::mix(band.colors[idx1], band.colors[idx2], t);
}

float LightingManager::sampleFloatBand(const FloatBand& band, uint16_t timeHalfMinutes) const {
    if (band.numKeyframes == 0) {
        return 1.0f;  // Fallback
    }

    if (band.numKeyframes == 1) {
        return band.values[0];
    }

    // Safer initialization: default to wrapping last→first
    uint8_t idx1 = band.numKeyframes - 1;
    uint8_t idx2 = 0;

    // Find surrounding keyframes
    for (uint8_t i = 0; i < band.numKeyframes; ++i) {
        if (timeHalfMinutes < band.times[i]) {
            idx2 = i;
            idx1 = (i > 0) ? (i - 1) : (band.numKeyframes - 1);
            break;
        }
    }

    uint16_t t1 = band.times[idx1];
    uint16_t t2 = band.times[idx2];

    uint16_t timeSpan = (t2 > t1) ? (t2 - t1) : (kHalfMinutesPerDay - t1 + t2);
    uint16_t elapsed = (timeHalfMinutes >= t1) ? (timeHalfMinutes - t1) : (kHalfMinutesPerDay - t1 + timeHalfMinutes);

    float t = (timeSpan > 0) ? (static_cast<float>(elapsed) / static_cast<float>(timeSpan)) : 0.0f;
    t = glm::clamp(t, 0.0f, 1.0f);

    return glm::mix(band.values[idx1], band.values[idx2], t);
}

glm::vec3 LightingManager::dbcColorToVec3(uint32_t dbcColor) const {
    // DBC colors are stored as BGR (0x00BBGGRR on little-endian)
    uint8_t b = (dbcColor >> 16) & 0xFF;
    uint8_t g = (dbcColor >> 8) & 0xFF;
    uint8_t r = dbcColor & 0xFF;

    return glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
}

} // namespace rendering
} // namespace wowee
