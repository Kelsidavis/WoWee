#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include <glm/vec3.hpp>

namespace wowee {
namespace pipeline {
class AssetManager;
}

namespace audio {

class AmbientSoundManager {
public:
    AmbientSoundManager() = default;
    ~AmbientSoundManager() = default;

    // Initialization
    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Main update loop - called from renderer
    void update(float deltaTime, const glm::vec3& cameraPos, bool isIndoor, bool isSwimming = false, bool isBlacksmith = false);

    // Weather control
    enum class WeatherType { NONE, RAIN_LIGHT, RAIN_MEDIUM, RAIN_HEAVY, SNOW_LIGHT, SNOW_MEDIUM, SNOW_HEAVY };
    void setWeather(WeatherType type);
    WeatherType getCurrentWeather() const { return currentWeather_; }

    // Zone ambience control
    enum class ZoneType {
        NONE,
        FOREST_NORMAL,
        FOREST_SNOW,
        BEACH,
        GRASSLANDS,
        JUNGLE,
        MARSH,
        DESERT_CANYON,
        DESERT_PLAINS
    };
    void setZoneType(ZoneType type);
    ZoneType getCurrentZone() const { return currentZone_; }

    // City ambience control
    enum class CityType {
        NONE,
        STORMWIND,
        IRONFORGE,
        DARNASSUS,
        ORGRIMMAR,
        UNDERCITY,
        THUNDERBLUFF
    };
    void setCityType(CityType type);
    CityType getCurrentCity() const { return currentCity_; }

    // Emitter management
    enum class AmbientType {
        FIREPLACE_SMALL,
        FIREPLACE_LARGE,
        TORCH,
        FOUNTAIN,
        WATER_SURFACE,
        RIVER,
        WATERFALL,
        WIND,
        BIRD_DAY,
        CRICKET_NIGHT,
        OWL_NIGHT
    };

    uint64_t addEmitter(const glm::vec3& position, AmbientType type);
    void removeEmitter(uint64_t id);
    void clearEmitters();

    // Time of day control (0-24 hours)
    void setGameTime(float hours);

    // Volume control
    void setVolumeScale(float scale);

private:
    struct AmbientEmitter {
        uint64_t id;
        AmbientType type;
        glm::vec3 position;
        bool active;
        float lastPlayTime;
        float loopInterval;  // For periodic/looping sounds
    };

    struct AmbientSample {
        std::string path;
        std::vector<uint8_t> data;
        bool loaded;
    };

    // Sound libraries
    std::vector<AmbientSample> fireSoundsSmall_;
    std::vector<AmbientSample> fireSoundsLarge_;
    std::vector<AmbientSample> torchSounds_;
    std::vector<AmbientSample> waterSounds_;
    std::vector<AmbientSample> riverSounds_;
    std::vector<AmbientSample> waterfallSounds_;
    std::vector<AmbientSample> windSounds_;
    std::vector<AmbientSample> tavernSounds_;
    std::vector<AmbientSample> blacksmithSounds_;

    // Weather sound libraries
    std::vector<AmbientSample> rainLightSounds_;
    std::vector<AmbientSample> rainMediumSounds_;
    std::vector<AmbientSample> rainHeavySounds_;
    std::vector<AmbientSample> snowLightSounds_;
    std::vector<AmbientSample> snowMediumSounds_;
    std::vector<AmbientSample> snowHeavySounds_;

    // Water ambience libraries
    std::vector<AmbientSample> oceanSounds_;
    std::vector<AmbientSample> underwaterSounds_;

    // Zone ambience libraries (day and night versions)
    std::vector<AmbientSample> forestNormalDaySounds_;
    std::vector<AmbientSample> forestNormalNightSounds_;
    std::vector<AmbientSample> forestSnowDaySounds_;
    std::vector<AmbientSample> forestSnowNightSounds_;
    std::vector<AmbientSample> beachDaySounds_;
    std::vector<AmbientSample> beachNightSounds_;
    std::vector<AmbientSample> grasslandsDaySounds_;
    std::vector<AmbientSample> grasslandsNightSounds_;
    std::vector<AmbientSample> jungleDaySounds_;
    std::vector<AmbientSample> jungleNightSounds_;
    std::vector<AmbientSample> marshDaySounds_;
    std::vector<AmbientSample> marshNightSounds_;
    std::vector<AmbientSample> desertCanyonDaySounds_;
    std::vector<AmbientSample> desertCanyonNightSounds_;
    std::vector<AmbientSample> desertPlainsDaySounds_;
    std::vector<AmbientSample> desertPlainsNightSounds_;

    // City ambience libraries (day and night versions)
    std::vector<AmbientSample> stormwindDaySounds_;
    std::vector<AmbientSample> stormwindNightSounds_;
    std::vector<AmbientSample> ironforgeSounds_;  // No separate day/night
    std::vector<AmbientSample> darnassusDaySounds_;
    std::vector<AmbientSample> darnassusNightSounds_;
    std::vector<AmbientSample> orgrimmarDaySounds_;
    std::vector<AmbientSample> orgrimmarNightSounds_;
    std::vector<AmbientSample> undercitySounds_;  // No separate day/night (underground)
    std::vector<AmbientSample> thunderbluffDaySounds_;
    std::vector<AmbientSample> thunderbluffNightSounds_;

    // City bell sounds
    std::vector<AmbientSample> bellAllianceSounds_;
    std::vector<AmbientSample> bellHordeSounds_;
    std::vector<AmbientSample> bellNightElfSounds_;
    std::vector<AmbientSample> bellTribalSounds_;

    // Active emitters
    std::vector<AmbientEmitter> emitters_;
    uint64_t nextEmitterId_ = 1;

    // State tracking
    float gameTimeHours_ = 12.0f;  // Default noon
    float volumeScale_ = 1.0f;
    float birdTimer_ = 0.0f;
    float cricketTimer_ = 0.0f;
    float windLoopTime_ = 0.0f;
    float blacksmithLoopTime_ = 0.0f;
    float weatherLoopTime_ = 0.0f;
    float oceanLoopTime_ = 0.0f;
    float zoneLoopTime_ = 0.0f;
    float cityLoopTime_ = 0.0f;
    float bellTollTime_ = 0.0f;
    bool wasIndoor_ = false;
    bool wasBlacksmith_ = false;
    bool wasSwimming_ = false;
    bool initialized_ = false;
    WeatherType currentWeather_ = WeatherType::NONE;
    ZoneType currentZone_ = ZoneType::NONE;
    CityType currentCity_ = CityType::NONE;

    // Active audio tracking
    struct ActiveSound {
        uint64_t emitterId;
        float startTime;
    };
    std::vector<ActiveSound> activeSounds_;

    // Helper methods
    void updatePositionalEmitters(float deltaTime, const glm::vec3& cameraPos);
    void updatePeriodicSounds(float deltaTime, bool isIndoor, bool isSwimming);
    void updateWindAmbience(float deltaTime, bool isIndoor);
    void updateBlacksmithAmbience(float deltaTime);
    void updateWeatherAmbience(float deltaTime, bool isIndoor);
    void updateWaterAmbience(float deltaTime, bool isSwimming);
    void updateZoneAmbience(float deltaTime, bool isIndoor);
    void updateCityAmbience(float deltaTime);
    void updateBellTolls(float deltaTime);
    bool loadSound(const std::string& path, AmbientSample& sample, pipeline::AssetManager* assets);

    // Time of day helpers
    bool isDaytime() const { return gameTimeHours_ >= 6.0f && gameTimeHours_ < 20.0f; }
    bool isNighttime() const { return !isDaytime(); }
};

} // namespace audio
} // namespace wowee
