#pragma once

#include <chrono>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <glm/glm.hpp>

namespace wowee {
namespace pipeline { class DBCFile; class AssetManager; }

namespace rendering {

/**
 * Time-of-day lighting parameters sampled from DBC curves
 */
struct LightingParams {
    glm::vec3 ambientColor{0.4f, 0.4f, 0.5f};      // Fill lighting
    glm::vec3 diffuseColor{1.0f, 0.95f, 0.8f};     // Directional sun color
    glm::vec3 directionalDir{0.0f, -1.0f, 0.5f};   // Sun direction (normalized)

    glm::vec3 fogColor{0.5f, 0.6f, 0.7f};          // Fog color
    float fogStart = 100.0f;                        // Fog start distance
    float fogEnd = 1000.0f;                         // Fog end distance
    float fogDensity = 0.001f;                      // Fog density

    glm::vec3 skyTopColor{0.5f, 0.7f, 1.0f};       // Sky zenith color
    glm::vec3 skyMiddleColor{0.7f, 0.85f, 1.0f};   // Sky horizon color
    glm::vec3 skyBand1Color{0.9f, 0.95f, 1.0f};    // Sky band 1
    glm::vec3 skyBand2Color{1.0f, 0.98f, 0.9f};    // Sky band 2

    float cloudDensity = 0.3f;                      // Cloud density/opacity
    float horizonGlow = 0.3f;                       // Horizon glow intensity
};

/** Apply any authored ambience that must remain stable regardless of world time. */
void applyZoneAmbienceOverride(uint32_t zoneId, LightingParams& params);

/** Resolve the sky clock shown for a zone without changing the world clock. */
float resolveZoneVisualTimeHours(uint32_t zoneId, bool isIndoors, float worldTimeHours);

/**
 * Light set keyframe for time-of-day interpolation
 */
struct LightKeyframe {
    uint32_t time;  // Time in minutes since midnight (0-1439)

    // Colors stored as RGB int tuples (0-255) in DBC
    glm::vec3 ambientColor;
    glm::vec3 diffuseColor;
    glm::vec3 fogColor;
    glm::vec3 skyTopColor;
    glm::vec3 skyMiddleColor;
    glm::vec3 skyBand1Color;
    glm::vec3 skyBand2Color;

    float fogStart;
    float fogEnd;
    float fogDensity;
    float cloudDensity;
    float horizonGlow;
};

/**
 * Light volume from Light.dbc (spatial lighting)
 */
struct LightVolume {
    uint32_t lightId = 0;
    uint32_t mapId = 0;
    glm::vec3 position{0.0f};  // World position (note: DBC stores as x,z,y!)
    float innerRadius = 0.0f;   // Full weight radius
    float outerRadius = 0.0f;   // Fade-out radius

    // LightParams IDs for different conditions
    uint32_t lightParamsId = 0;        // Normal/clear weather
    uint32_t lightParamsIdRain = 0;    // Rainy weather
    uint32_t lightParamsIdUnderwater = 0;
    // More variants exist for phases, death, etc.
};

/**
 * Color band with time-of-day keyframes
 */
struct ColorBand {
    uint8_t numKeyframes = 0;
    uint16_t times[16];        // Time keyframes (half-minutes since midnight)
    glm::vec3 colors[16];      // Color values (RGB 0-1)
};

/**
 * Float band with time-of-day keyframes
 */
struct FloatBand {
    uint8_t numKeyframes = 0;
    uint16_t times[16];        // Time keyframes (half-minutes since midnight)
    float values[16];          // Float values
};

/**
 * LightParams profile with 18 color bands + 6 float bands
 */
struct LightParamsProfile {
    uint32_t lightParamsId = 0;
    uint32_t lightSkyboxId = 0;

    // 18 color channels (IntBand)
    //
    // Read off the file rather than from the order they are usually listed in.
    // The sky gradient anchors the rest: for Stormwind at noon, channels 2 to 5
    // are (0,31,73), (58,162,207), (153,220,245), (175,218,224) - a deep blue
    // overhead paling to the horizon, which can only be the sky and fixes every
    // index around it.
    //
    // Channel 0 is not the ambient. Across zones it is the warm, bright one -
    // (199,168,134) over Dun Morogh's snow, (255,224,169) in Teldrassil,
    // (255,136,0) in Stormwind - while channel 1 is the dark cool one that
    // carries the zone's cast: (31,82,125) in Dun Morogh, violet (125,72,130)
    // in Teldrassil and Winterspring. A scene ambient multiplies every surface,
    // so reading channel 0 into it put Stormwind's orange over the whole city
    // and left it looking like Durotar, whose channel 0 is (255,204,148).
    //
    // Channel 7 is the fog. Channel 6 is the sky's smog layer and is a neutral
    // grey - (180,180,180) at Stormwind noon - which is the pale grey the fog
    // blend further down was written to work around.
    enum ColorChannel {
        DIFFUSE_COLOR = 0,
        AMBIENT_COLOR = 1,
        SKY_TOP_COLOR = 2,
        SKY_MIDDLE_COLOR = 3,
        SKY_BAND1_COLOR = 4,
        SKY_BAND2_COLOR = 5,
        SKY_SMOG_COLOR = 6,
        FOG_COLOR = 7,
        SUN_COLOR = 9,
        // ... more channels exist (clouds, ocean, river)
        COLOR_CHANNEL_COUNT = 18
    };

    ColorBand colorBands[COLOR_CHANNEL_COUNT];

    // 6 float channels (FloatBand)
    enum FloatChannel {
        FOG_END = 0,
        FOG_START_SCALAR = 1,  // Multiplier for fog start
        // Channel 2 is not a density. Measured over every band in the file,
        // 98% of its 2509 samples are exactly 1.0 - it is the switch for how
        // much the sun and moon show through cloud, and reading it as cloud
        // density left almost every zone under solid overcast at all hours,
        // which is what hid the skybox.
        CELESTIAL_GLOW_THROUGH = 2,
        // Channel 3 is the curve that actually varies: 0 to 5, mean 0.50.
        CLOUD_DENSITY = 3,
        FOG_DENSITY = 3,
        // ... more channels
        FLOAT_CHANNEL_COUNT = 6
    };

    FloatBand floatBands[FLOAT_CHANNEL_COUNT];
};

/**
 * WoW DBC-driven lighting manager
 *
 * Implements WotLK's time-of-day lighting system:
 * - Loads Light.dbc, LightParams.dbc, LightIntBand.dbc, LightFloatBand.dbc
 * - Samples lighting curves based on time-of-day
 * - Interpolates between keyframes
 * - Provides lighting parameters for rendering
 */
class LightingManager {
public:
    LightingManager();
    ~LightingManager();

    /**
     * Initialize lighting system and load DBCs
     */
    bool initialize(pipeline::AssetManager* assetManager);

    /**
     * Update lighting for current time and player position
     * @param playerPos Player world position
     * @param mapId Current map ID
     * @param gameTime Optional game time in seconds (use -1 for real time)
     * @param isRaining Whether it's raining
     * @param isUnderwater Whether player is underwater
     *
     * Note: WoW uses server-sent game time, not local PC time.
     * Pass gameTime from SMSG_LOGIN_SETTIMESPEED or similar.
     */
    void update(const glm::vec3& playerPos, uint32_t mapId, uint32_t zoneId,
                float gameTime = -1.0f,
                bool isRaining = false, bool isUnderwater = false);

    /**
     * Get current lighting parameters
     */
    [[nodiscard]] const LightingParams& getLightingParams() const { return currentParams_; }

    /**
     * Set whether player is indoors (disables outdoor lighting)
     */
    void setIndoors(bool indoors) { isIndoors_ = indoors; }
    /// How far the distance fog is pulled toward the sky's horizon colour.
    /// 0 keeps LightParams' own fog colour, 1 is the sky itself. See
    /// LightingManager::update.
    void setFogSkyBlend(float blend) { fogSkyBlend_ = blend; }
    [[nodiscard]] float getFogSkyBlend() const { return fogSkyBlend_; }
    /// How much distance fog, as a multiplier on the zone's own fog distances.
    /// 1 is the DBC unchanged, above 1 is thicker, 0 is none; the default
    /// 0.4 thins the authored fog, which reads too heavy here. See
    /// LightingManager::update.
    void setFogStrength(float strength) { fogStrength_ = strength; }
    [[nodiscard]] float getFogStrength() const { return fogStrength_; }

    /**
     * Get current time of day (0.0-1.0)
     */
    [[nodiscard]] float getTimeOfDay() const { return timeOfDay_; }

    /** Time used by the visible sky, including persistent zone ambience. */
    [[nodiscard]] float getVisualTimeOfDayHours() const { return visualTimeOfDayHours_; }

    /// One of the original client's sky models, and how much of it is up.
    struct SkyboxLayer {
        std::string path;
        float weight = 0.0f;   ///< 0..1: the share of the lights here that name it
    };
    /// Every sky model the lights around the player name, heaviest first, at
    /// the weights those lights have - the same falloff the sky colours blend
    /// by, smoothed the same way. A zone's sky fades out across its edge as
    /// the next one's fades in, rather than one swapping for the other.
    [[nodiscard]] const std::vector<SkyboxLayer>& getSkyboxLayers() const { return skyboxLayers_; }

    /**
     * Manually set time of day for testing
     */
    void setTimeOfDay(float tod) { timeOfDay_ = tod; manualTime_ = true; }

    /**
     * Use real time for day/night cycle
     */
    void useRealTime(bool use) { manualTime_ = !use; }

private:
    /**
     * Load Light.dbc
     */
    bool loadLightDbc(pipeline::AssetManager* assetManager);

    /**
     * Load LightParams.dbc for zone→light mapping
     */
    bool loadLightParamsDbc(pipeline::AssetManager* assetManager);

    bool loadLightSkyboxDbc(pipeline::AssetManager* assetManager);

    /**
     * Load LightIntBand.dbc and LightFloatBand.dbc for time curves
     */
    bool loadLightBandDbcs(pipeline::AssetManager* assetManager);

    /**
     * Weighted light volume for blending
     */
    struct WeightedVolume {
        const LightVolume* volume = nullptr;
        float weight = 0.0f;
    };

    /**
     * Find light volumes for blending (up to 4 with weight > 0)
     */
    [[nodiscard]] std::vector<WeightedVolume> findLightVolumes(const glm::vec3& playerPos, uint32_t mapId) const;

    /**
     * Get LightParams ID based on conditions
     */
    uint32_t selectLightParamsId(const LightVolume* volume, bool isRaining, bool isUnderwater) const;

    /**
     * Sample lighting from LightParams profile
     */
    LightingParams sampleLightParams(const LightParamsProfile* profile, uint16_t timeHalfMinutes) const;

    /**
     * Sample color from band
     */
    [[nodiscard]] glm::vec3 sampleColorBand(const ColorBand& band, uint16_t timeHalfMinutes) const;

    /**
     * Sample float from band
     */
    [[nodiscard]] float sampleFloatBand(const FloatBand& band, uint16_t timeHalfMinutes) const;

    /**
     * Convert DBC BGR color to RGB vec3
     */
    [[nodiscard]] glm::vec3 dbcColorToVec3(uint32_t dbcColor) const;


    // Light volumes by map
    std::map<uint32_t, std::vector<LightVolume>> lightVolumesByMap_;

    // LightParams profiles by ID
    std::map<uint32_t, LightParamsProfile> lightParamsProfiles_;
    std::map<uint32_t, std::string> lightSkyboxPaths_;

    // Current state
    LightingParams currentParams_;
    std::vector<WeightedVolume> activeVolumes_;
    glm::vec3 currentPlayerPos_{0.0f};
    float timeOfDay_ = 0.5f;  // Start at noon
    float visualTimeOfDayHours_ = 12.0f;
    std::vector<SkyboxLayer> skyboxLayers_;
    /// When update last ran, for smoothing by real time rather than by frame.
    std::chrono::steady_clock::time_point lastUpdate_{};
    bool isIndoors_ = false;
    float fogSkyBlend_ = 0.7f;
    float fogStrength_ = 0.4f;

    // Last values the sky diagnostic reported, so it prints on a change
    // rather than every frame. See LightingManager::update.
    /// The map the volume list was last named for, so it is named once per map
    /// rather than once per frame. findLightVolumes is const.
    mutable uint32_t diagLoggedMapId_ = 0xFFFFFFFFu;
    uint32_t diagZoneId_ = 0xFFFFFFFFu;
    uint32_t diagCallsSinceLog_ = 0;
    uint32_t diagFirstVolume_ = 0xFFFFFFFFu;
    uint32_t diagSecondVolume_ = 0xFFFFFFFFu;
    float diagVisualHours_ = -1.0f;
    float diagSkyLuma_ = -1.0f;
    std::string diagSkyboxPath_ = "\x01";
    bool manualTime_ = false;
    bool initialized_ = false;

    // Fallback lighting
    LightingParams fallbackParams_;
};

} // namespace rendering
} // namespace wowee
