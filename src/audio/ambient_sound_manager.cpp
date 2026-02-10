#include "audio/ambient_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <random>
#include <algorithm>
#include <cmath>

namespace wowee {
namespace audio {

namespace {
    // Distance thresholds (in game units)
    constexpr float MAX_FIRE_DISTANCE = 20.0f;
    constexpr float MAX_WATER_DISTANCE = 35.0f;
    constexpr float MAX_AMBIENT_DISTANCE = 50.0f;

    // Volume settings
    constexpr float FIRE_VOLUME = 0.7f;
    constexpr float WATER_VOLUME = 0.5f;
    constexpr float WIND_VOLUME = 0.35f;
    constexpr float BIRD_VOLUME = 0.6f;
    constexpr float CRICKET_VOLUME = 0.5f;

    // Timing settings (seconds)
    constexpr float BIRD_MIN_INTERVAL = 8.0f;
    constexpr float BIRD_MAX_INTERVAL = 20.0f;
    constexpr float CRICKET_MIN_INTERVAL = 6.0f;
    constexpr float CRICKET_MAX_INTERVAL = 15.0f;
    constexpr float FIRE_LOOP_INTERVAL = 3.0f;  // Fire crackling loop length

    std::random_device rd;
    std::mt19937 gen(rd());

    float randomFloat(float min, float max) {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(gen);
    }
}

bool AmbientSoundManager::initialize(pipeline::AssetManager* assets) {
    if (!assets) {
        LOG_ERROR("AmbientSoundManager: AssetManager is null");
        return false;
    }

    LOG_INFO("AmbientSoundManager: Initializing...");

    // Load fire sounds
    fireSoundsSmall_.resize(1);
    loadSound("Sound\\Doodad\\CampFireSmallLoop.wav", fireSoundsSmall_[0], assets);

    fireSoundsLarge_.resize(1);
    loadSound("Sound\\Doodad\\CampFireLargeLoop.wav", fireSoundsLarge_[0], assets);

    torchSounds_.resize(1);
    loadSound("Sound\\Doodad\\TorchFireLoop.wav", torchSounds_[0], assets);

    // Load water sounds
    waterSounds_.resize(1);
    loadSound("Sound\\Ambience\\Water\\River_LakeStillA.wav", waterSounds_[0], assets);

    riverSounds_.resize(1);
    loadSound("Sound\\Ambience\\Water\\RiverSlowA.wav", riverSounds_[0], assets);

    waterfallSounds_.resize(1);
    loadSound("Sound\\Doodad\\WaterFallSmall.wav", waterfallSounds_[0], assets);

    // Load wind/ambience sounds
    windSounds_.resize(1);
    bool windLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\ForestNormalDay.wav", windSounds_[0], assets);

    tavernSounds_.resize(1);
    bool tavernLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\Tavern.wav", tavernSounds_[0], assets);

    // Load blacksmith ambience loop
    blacksmithSounds_.resize(1);
    bool blacksmithLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\BlackSmith.wav", blacksmithSounds_[0], assets);

    // Load weather sounds
    rainLightSounds_.resize(1);
    bool rainLightLoaded = loadSound("Sound\\Ambience\\Weather\\RainLight.wav", rainLightSounds_[0], assets);

    rainMediumSounds_.resize(1);
    bool rainMediumLoaded = loadSound("Sound\\Ambience\\Weather\\RainMedium.wav", rainMediumSounds_[0], assets);

    rainHeavySounds_.resize(1);
    bool rainHeavyLoaded = loadSound("Sound\\Ambience\\Weather\\RainHeavy.wav", rainHeavySounds_[0], assets);

    snowLightSounds_.resize(1);
    bool snowLightLoaded = loadSound("Sound\\Ambience\\Weather\\SnowLight.wav", snowLightSounds_[0], assets);

    snowMediumSounds_.resize(1);
    bool snowMediumLoaded = loadSound("Sound\\Ambience\\Weather\\SnowMedium.wav", snowMediumSounds_[0], assets);

    snowHeavySounds_.resize(1);
    bool snowHeavyLoaded = loadSound("Sound\\Ambience\\Weather\\SnowHeavy.wav", snowHeavySounds_[0], assets);

    // Load water ambience sounds
    oceanSounds_.resize(1);
    bool oceanLoaded = loadSound("Sound\\Ambience\\Water\\OceanDeepDay.wav", oceanSounds_[0], assets);

    underwaterSounds_.resize(1);
    bool underwaterLoaded = loadSound("Sound\\Ambience\\Water\\UnderwaterSwim.wav", underwaterSounds_[0], assets);

    // Load zone ambience sounds (day and night)
    forestNormalDaySounds_.resize(1);
    bool forestDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\ForestNormalDay.wav", forestNormalDaySounds_[0], assets);

    forestNormalNightSounds_.resize(1);
    bool forestNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\ForestNormalNight.wav", forestNormalNightSounds_[0], assets);

    forestSnowDaySounds_.resize(1);
    bool forestSnowDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\ForestSnowDay.wav", forestSnowDaySounds_[0], assets);

    forestSnowNightSounds_.resize(1);
    bool forestSnowNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\ForestSnowNight.wav", forestSnowNightSounds_[0], assets);

    beachDaySounds_.resize(1);
    bool beachDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\BeachDay.wav", beachDaySounds_[0], assets);

    beachNightSounds_.resize(1);
    bool beachNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\BeachNight.wav", beachNightSounds_[0], assets);

    grasslandsDaySounds_.resize(1);
    bool grasslandsDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\GrasslandsDay.wav", grasslandsDaySounds_[0], assets);

    grasslandsNightSounds_.resize(1);
    bool grasslandsNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\GrassLandsNight.wav", grasslandsNightSounds_[0], assets);

    jungleDaySounds_.resize(1);
    bool jungleDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\JungleDay.wav", jungleDaySounds_[0], assets);

    jungleNightSounds_.resize(1);
    bool jungleNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\JungleNight.wav", jungleNightSounds_[0], assets);

    marshDaySounds_.resize(1);
    bool marshDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\MarshDay.wav", marshDaySounds_[0], assets);

    marshNightSounds_.resize(1);
    bool marshNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\MarshNight.wav", marshNightSounds_[0], assets);

    desertCanyonDaySounds_.resize(1);
    bool desertCanyonDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\CanyonDesertDay.wav", desertCanyonDaySounds_[0], assets);

    desertCanyonNightSounds_.resize(1);
    bool desertCanyonNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\CanyonDesertNight.wav", desertCanyonNightSounds_[0], assets);

    desertPlainsDaySounds_.resize(1);
    bool desertPlainsDayLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\PlainsDesertDay.wav", desertPlainsDaySounds_[0], assets);

    desertPlainsNightSounds_.resize(1);
    bool desertPlainsNightLoaded = loadSound("Sound\\Ambience\\ZoneAmbience\\PlainsDesertNight.wav", desertPlainsNightSounds_[0], assets);

    // Load city ambience sounds (day and night where available)
    stormwindDaySounds_.resize(1);
    bool stormwindDayLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\StormwindDay.wav", stormwindDaySounds_[0], assets);

    stormwindNightSounds_.resize(1);
    bool stormwindNightLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\StormwindNight.wav", stormwindNightSounds_[0], assets);

    ironforgeSounds_.resize(1);
    bool ironforgeLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\Ironforge.wav", ironforgeSounds_[0], assets);

    darnassusDaySounds_.resize(1);
    bool darnassusDayLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\DarnassusDay.wav", darnassusDaySounds_[0], assets);

    darnassusNightSounds_.resize(1);
    bool darnassusNightLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\DarnassusNight.wav", darnassusNightSounds_[0], assets);

    orgrimmarDaySounds_.resize(1);
    bool orgrimmarDayLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\OrgrimmarDay.wav", orgrimmarDaySounds_[0], assets);

    orgrimmarNightSounds_.resize(1);
    bool orgrimmarNightLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\OrgrimmarNight.wav", orgrimmarNightSounds_[0], assets);

    undercitySounds_.resize(1);
    bool undercityLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\Undercity.wav", undercitySounds_[0], assets);

    thunderbluffDaySounds_.resize(1);
    bool thunderbluffDayLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\ThunderBluffDay.wav", thunderbluffDaySounds_[0], assets);

    thunderbluffNightSounds_.resize(1);
    bool thunderbluffNightLoaded = loadSound("Sound\\Ambience\\WMOAmbience\\ThunderBluffNight.wav", thunderbluffNightSounds_[0], assets);

    // Load bell toll sounds
    bellAllianceSounds_.resize(1);
    bool bellAllianceLoaded = loadSound("Sound\\Doodad\\BellTollAlliance.wav", bellAllianceSounds_[0], assets);

    bellHordeSounds_.resize(1);
    bool bellHordeLoaded = loadSound("Sound\\Doodad\\BellTollHorde.wav", bellHordeSounds_[0], assets);

    bellNightElfSounds_.resize(1);
    bool bellNightElfLoaded = loadSound("Sound\\Doodad\\BellTollNightElf.wav", bellNightElfSounds_[0], assets);

    bellTribalSounds_.resize(1);
    bool bellTribalLoaded = loadSound("Sound\\Doodad\\BellTollTribal.wav", bellTribalSounds_[0], assets);

    LOG_INFO("AmbientSoundManager: Wind loaded: ", windLoaded ? "YES" : "NO",
             ", Tavern loaded: ", tavernLoaded ? "YES" : "NO",
             ", Blacksmith loaded: ", blacksmithLoaded ? "YES" : "NO");
    LOG_INFO("AmbientSoundManager: Weather sounds - Rain: ", (rainLightLoaded && rainMediumLoaded && rainHeavyLoaded) ? "YES" : "NO",
             ", Snow: ", (snowLightLoaded && snowMediumLoaded && snowHeavyLoaded) ? "YES" : "NO");
    LOG_INFO("AmbientSoundManager: Water sounds - Ocean: ", oceanLoaded ? "YES" : "NO",
             ", Underwater: ", underwaterLoaded ? "YES" : "NO");
    LOG_INFO("AmbientSoundManager: Zone sounds - Forest: ", (forestDayLoaded && forestNightLoaded) ? "YES" : "NO",
             ", Beach: ", (beachDayLoaded && beachNightLoaded) ? "YES" : "NO",
             ", Desert: ", (desertCanyonDayLoaded && desertPlainsDayLoaded) ? "YES" : "NO");
    LOG_INFO("AmbientSoundManager: City sounds - Stormwind: ", (stormwindDayLoaded && stormwindNightLoaded) ? "YES" : "NO",
             ", Ironforge: ", ironforgeLoaded ? "YES" : "NO",
             ", Orgrimmar: ", (orgrimmarDayLoaded && orgrimmarNightLoaded) ? "YES" : "NO");
    LOG_INFO("AmbientSoundManager: Bell tolls - Alliance: ", bellAllianceLoaded ? "YES" : "NO",
             ", Horde: ", bellHordeLoaded ? "YES" : "NO",
             ", NightElf: ", bellNightElfLoaded ? "YES" : "NO",
             ", Tribal: ", bellTribalLoaded ? "YES" : "NO");

    // Initialize timers with random offsets
    birdTimer_ = randomFloat(0.0f, 5.0f);
    cricketTimer_ = randomFloat(0.0f, 5.0f);

    initialized_ = true;
    LOG_INFO("AmbientSoundManager: Initialization complete");
    return true;
}

void AmbientSoundManager::shutdown() {
    emitters_.clear();
    activeSounds_.clear();
    initialized_ = false;
}

bool AmbientSoundManager::loadSound(const std::string& path, AmbientSample& sample, pipeline::AssetManager* assets) {
    sample.path = path;
    sample.loaded = false;

    try {
        sample.data = assets->readFile(path);
        if (!sample.data.empty()) {
            sample.loaded = true;
            return true;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("AmbientSoundManager: Failed to load ", path, ": ", e.what());
    }

    return false;
}

void AmbientSoundManager::update(float deltaTime, const glm::vec3& cameraPos, bool isIndoor, bool isSwimming, bool isBlacksmith) {
    if (!initialized_) return;

    // Update all emitter systems
    updatePositionalEmitters(deltaTime, cameraPos);

    // Don't play outdoor periodic sounds (birds) when indoors OR in blacksmith
    if (!isIndoor && !isBlacksmith) {
        updatePeriodicSounds(deltaTime, isIndoor, isSwimming);
    }

    // Handle state changes
    if (wasBlacksmith_ && !isBlacksmith) {
        LOG_INFO("Ambient: EXITED BLACKSMITH");
        blacksmithLoopTime_ = 0.0f;  // Reset timer when leaving
    }

    // Blacksmith takes priority over tavern
    if (isBlacksmith) {
        updateBlacksmithAmbience(deltaTime);
    } else {
        updateWindAmbience(deltaTime, isIndoor);
    }

    // Update weather, water, zone, and city ambience
    updateWeatherAmbience(deltaTime, isIndoor);
    updateWaterAmbience(deltaTime, isSwimming);
    updateZoneAmbience(deltaTime, isIndoor);
    updateCityAmbience(deltaTime);
    updateBellTolls(deltaTime);

    // Track indoor state changes
    wasIndoor_ = isIndoor;
    wasBlacksmith_ = isBlacksmith;
}

void AmbientSoundManager::updatePositionalEmitters(float deltaTime, const glm::vec3& cameraPos) {
    // First pass: mark emitters as active/inactive based on distance
    int activeFireCount = 0;
    int activeWaterCount = 0;
    const int MAX_ACTIVE_FIRE = 5;      // Max 5 fire sounds at once
    const int MAX_ACTIVE_WATER = 3;     // Max 3 water sounds at once

    for (auto& emitter : emitters_) {
        float distance = glm::distance(emitter.position, cameraPos);

        // Determine max distance based on type
        float maxDist = MAX_AMBIENT_DISTANCE;
        bool isFire = false;
        bool isWater = false;

        if (emitter.type == AmbientType::FIREPLACE_SMALL ||
            emitter.type == AmbientType::FIREPLACE_LARGE ||
            emitter.type == AmbientType::TORCH) {
            maxDist = MAX_FIRE_DISTANCE;
            isFire = true;
        } else if (emitter.type == AmbientType::WATER_SURFACE ||
                   emitter.type == AmbientType::RIVER ||
                   emitter.type == AmbientType::WATERFALL) {
            maxDist = MAX_WATER_DISTANCE;
            isWater = true;
        }

        // Update active state based on distance AND limits
        bool withinRange = (distance < maxDist);

        if (isFire && withinRange && activeFireCount < MAX_ACTIVE_FIRE) {
            emitter.active = true;
            activeFireCount++;
        } else if (isWater && withinRange && activeWaterCount < MAX_ACTIVE_WATER) {
            emitter.active = true;
            activeWaterCount++;
        } else if (!isFire && !isWater && withinRange) {
            emitter.active = true;  // Other types (fountain, etc)
        } else {
            emitter.active = false;
        }

        if (!emitter.active) continue;

        // Update play timer
        emitter.lastPlayTime += deltaTime;

        // Handle different emitter types
        switch (emitter.type) {
            case AmbientType::FIREPLACE_SMALL:
                if (emitter.lastPlayTime >= FIRE_LOOP_INTERVAL && !fireSoundsSmall_.empty() && fireSoundsSmall_[0].loaded) {
                    float volume = FIRE_VOLUME * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(fireSoundsSmall_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            case AmbientType::FIREPLACE_LARGE:
                if (emitter.lastPlayTime >= FIRE_LOOP_INTERVAL && !fireSoundsLarge_.empty() && fireSoundsLarge_[0].loaded) {
                    float volume = FIRE_VOLUME * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(fireSoundsLarge_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            case AmbientType::TORCH:
                if (emitter.lastPlayTime >= FIRE_LOOP_INTERVAL && !torchSounds_.empty() && torchSounds_[0].loaded) {
                    float volume = FIRE_VOLUME * 0.7f * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(torchSounds_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            case AmbientType::WATER_SURFACE:
                if (emitter.lastPlayTime >= 5.0f && !waterSounds_.empty() && waterSounds_[0].loaded) {
                    float volume = WATER_VOLUME * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(waterSounds_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            case AmbientType::RIVER:
                if (emitter.lastPlayTime >= 5.0f && !riverSounds_.empty() && riverSounds_[0].loaded) {
                    float volume = WATER_VOLUME * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(riverSounds_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            case AmbientType::WATERFALL:
                if (emitter.lastPlayTime >= 4.0f && !waterfallSounds_.empty() && waterfallSounds_[0].loaded) {
                    float volume = WATER_VOLUME * 1.2f * volumeScale_ * (1.0f - (distance / maxDist));
                    AudioEngine::instance().playSound3D(waterfallSounds_[0].data, emitter.position, volume);
                    emitter.lastPlayTime = 0.0f;
                }
                break;

            default:
                break;
        }
    }
}

void AmbientSoundManager::updatePeriodicSounds(float deltaTime, bool isIndoor, bool isSwimming) {
    // Only play outdoor periodic sounds when outdoors and not swimming/underwater
    if (isIndoor || isSwimming) return;

    // Bird sounds during daytime
    if (isDaytime()) {
        birdTimer_ += deltaTime;
        if (birdTimer_ >= randomFloat(BIRD_MIN_INTERVAL, BIRD_MAX_INTERVAL)) {
            // Play a random bird chirp (we'll use wind sound as placeholder for now)
            // TODO: Add actual bird sound files when available
            birdTimer_ = 0.0f;
        }
    }

    // Cricket sounds during nighttime
    if (isNighttime()) {
        cricketTimer_ += deltaTime;
        if (cricketTimer_ >= randomFloat(CRICKET_MIN_INTERVAL, CRICKET_MAX_INTERVAL)) {
            // Play cricket sounds
            // TODO: Add actual cricket sound files when available
            cricketTimer_ = 0.0f;
        }
    }
}

void AmbientSoundManager::updateBlacksmithAmbience(float deltaTime) {
    bool stateChanged = !wasBlacksmith_;

    if (stateChanged) {
        LOG_INFO("Ambient: ENTERED BLACKSMITH");
        blacksmithLoopTime_ = 1.5f;  // Play first hammer soon
    }

    // Only play if we have loaded sounds
    bool hasSound = false;
    for (const auto& sound : blacksmithSounds_) {
        if (sound.loaded) {
            hasSound = true;
            break;
        }
    }

    if (hasSound && blacksmithSounds_[0].loaded) {
        blacksmithLoopTime_ += deltaTime;
        // Play blacksmith ambience loop every 15 seconds
        if (blacksmithLoopTime_ >= 15.0f) {
            float volume = 0.6f * volumeScale_;  // Ambient loop volume
            AudioEngine::instance().playSound2D(blacksmithSounds_[0].data, volume, 1.0f);
            LOG_INFO("Playing blacksmith ambience loop");
            blacksmithLoopTime_ = 0.0f;
        }
    }
}

void AmbientSoundManager::updateWindAmbience(float deltaTime, bool isIndoor) {
    // Always track indoor state for next frame
    bool stateChanged = (wasIndoor_ != isIndoor);

    if (stateChanged) {
        LOG_INFO("Ambient: ", isIndoor ? "ENTERED BUILDING" : "EXITED TO OUTDOORS");
        // Start timer at 10 seconds so ambience plays after ~5 seconds
        if (isIndoor) {
            windLoopTime_ = 10.0f;  // Play tavern ambience soon
        } else {
            windLoopTime_ = 25.0f;  // Play outdoor ambience soon
        }
    }

    wasIndoor_ = isIndoor;

    // Indoor ambience (tavern sounds) - glass clinking, chatter
    if (isIndoor) {
        if (!tavernSounds_.empty() && tavernSounds_[0].loaded) {
            windLoopTime_ += deltaTime;
            // Play every 15 seconds for ambient atmosphere
            if (windLoopTime_ >= 15.0f) {
                float volume = 0.5f * volumeScale_;
                AudioEngine::instance().playSound2D(tavernSounds_[0].data, volume, 1.0f);
                LOG_INFO("Playing tavern ambience (glasses clinking)");
                windLoopTime_ = 0.0f;
            }
        }
    }
    // Outdoor wind ambience
    else {
        if (!windSounds_.empty() && windSounds_[0].loaded) {
            windLoopTime_ += deltaTime;
            if (windLoopTime_ >= 30.0f) {
                float volume = 0.3f * volumeScale_;
                AudioEngine::instance().playSound2D(windSounds_[0].data, volume, 1.0f);
                LOG_INFO("Playing outdoor ambience");
                windLoopTime_ = 0.0f;
            }
        }
    }
}

uint64_t AmbientSoundManager::addEmitter(const glm::vec3& position, AmbientType type) {
    AmbientEmitter emitter;
    emitter.id = nextEmitterId_++;
    emitter.type = type;
    emitter.position = position;
    emitter.active = false;
    emitter.lastPlayTime = randomFloat(0.0f, 2.0f);  // Random initial offset
    emitter.loopInterval = FIRE_LOOP_INTERVAL;

    emitters_.push_back(emitter);
    return emitter.id;
}

void AmbientSoundManager::removeEmitter(uint64_t id) {
    emitters_.erase(
        std::remove_if(emitters_.begin(), emitters_.end(),
            [id](const AmbientEmitter& e) { return e.id == id; }),
        emitters_.end()
    );
}

void AmbientSoundManager::clearEmitters() {
    emitters_.clear();
}

void AmbientSoundManager::setGameTime(float hours) {
    gameTimeHours_ = std::fmod(hours, 24.0f);
    if (gameTimeHours_ < 0.0f) gameTimeHours_ += 24.0f;
}

void AmbientSoundManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

void AmbientSoundManager::setWeather(WeatherType type) {
    if (currentWeather_ != type) {
        LOG_INFO("AmbientSoundManager: Weather changed from ", static_cast<int>(currentWeather_),
                 " to ", static_cast<int>(type));
        currentWeather_ = type;
        weatherLoopTime_ = 0.0f;  // Reset timer on weather change
    }
}

void AmbientSoundManager::setZoneType(ZoneType type) {
    if (currentZone_ != type) {
        LOG_INFO("AmbientSoundManager: Zone changed from ", static_cast<int>(currentZone_),
                 " to ", static_cast<int>(type));
        currentZone_ = type;
        zoneLoopTime_ = 15.0f;  // Play zone ambience soon after entering
    }
}

void AmbientSoundManager::setCityType(CityType type) {
    if (currentCity_ != type) {
        LOG_INFO("AmbientSoundManager: City changed from ", static_cast<int>(currentCity_),
                 " to ", static_cast<int>(type));
        currentCity_ = type;
        cityLoopTime_ = 12.0f;  // Play city ambience soon after entering
        bellTollTime_ = randomFloat(60.0f, 90.0f);  // First bell toll after 1-1.5 minutes
    }
}

void AmbientSoundManager::updateWeatherAmbience(float deltaTime, bool isIndoor) {
    // Don't play weather sounds when indoors
    if (isIndoor || currentWeather_ == WeatherType::NONE) return;

    weatherLoopTime_ += deltaTime;

    // Select appropriate sound library based on weather type
    const std::vector<AmbientSample>* weatherLibrary = nullptr;
    float loopInterval = 20.0f;  // Default 20 second loop for weather

    switch (currentWeather_) {
        case WeatherType::RAIN_LIGHT:
            weatherLibrary = &rainLightSounds_;
            loopInterval = 25.0f;
            break;
        case WeatherType::RAIN_MEDIUM:
            weatherLibrary = &rainMediumSounds_;
            loopInterval = 20.0f;
            break;
        case WeatherType::RAIN_HEAVY:
            weatherLibrary = &rainHeavySounds_;
            loopInterval = 18.0f;
            break;
        case WeatherType::SNOW_LIGHT:
            weatherLibrary = &snowLightSounds_;
            loopInterval = 30.0f;
            break;
        case WeatherType::SNOW_MEDIUM:
            weatherLibrary = &snowMediumSounds_;
            loopInterval = 25.0f;
            break;
        case WeatherType::SNOW_HEAVY:
            weatherLibrary = &snowHeavySounds_;
            loopInterval = 22.0f;
            break;
        default:
            return;
    }

    // Play weather sound if library is loaded and timer expired
    if (weatherLibrary && !weatherLibrary->empty() && (*weatherLibrary)[0].loaded) {
        if (weatherLoopTime_ >= loopInterval) {
            float volume = 0.4f * volumeScale_;  // Weather ambience at moderate volume
            AudioEngine::instance().playSound2D((*weatherLibrary)[0].data, volume, 1.0f);
            LOG_INFO("Playing weather ambience: type ", static_cast<int>(currentWeather_));
            weatherLoopTime_ = 0.0f;
        }
    }
}

void AmbientSoundManager::updateWaterAmbience(float deltaTime, bool isSwimming) {
    bool stateChanged = (wasSwimming_ != isSwimming);

    if (stateChanged) {
        LOG_INFO("Ambient: ", isSwimming ? "ENTERED WATER" : "EXITED WATER");
        oceanLoopTime_ = 0.0f;  // Reset timer on state change
    }

    wasSwimming_ = isSwimming;

    // Play underwater sounds when swimming
    if (isSwimming) {
        if (!underwaterSounds_.empty() && underwaterSounds_[0].loaded) {
            oceanLoopTime_ += deltaTime;
            // Play every 18 seconds for underwater ambience
            if (oceanLoopTime_ >= 18.0f) {
                float volume = 0.5f * volumeScale_;
                AudioEngine::instance().playSound2D(underwaterSounds_[0].data, volume, 1.0f);
                LOG_INFO("Playing underwater ambience");
                oceanLoopTime_ = 0.0f;
            }
        }
    }
    // Play ocean sounds when near water but not swimming
    // (This could be enhanced later with proximity detection to water surfaces)
}

void AmbientSoundManager::updateZoneAmbience(float deltaTime, bool isIndoor) {
    // Don't play zone ambience when indoors or in cities
    if (isIndoor || currentZone_ == ZoneType::NONE || currentCity_ != CityType::NONE) return;

    zoneLoopTime_ += deltaTime;

    // Select appropriate sound library based on zone type and time of day
    const std::vector<AmbientSample>* zoneLibrary = nullptr;
    bool isDay = isDaytime();

    switch (currentZone_) {
        case ZoneType::FOREST_NORMAL:
            zoneLibrary = isDay ? &forestNormalDaySounds_ : &forestNormalNightSounds_;
            break;
        case ZoneType::FOREST_SNOW:
            zoneLibrary = isDay ? &forestSnowDaySounds_ : &forestSnowNightSounds_;
            break;
        case ZoneType::BEACH:
            zoneLibrary = isDay ? &beachDaySounds_ : &beachNightSounds_;
            break;
        case ZoneType::GRASSLANDS:
            zoneLibrary = isDay ? &grasslandsDaySounds_ : &grasslandsNightSounds_;
            break;
        case ZoneType::JUNGLE:
            zoneLibrary = isDay ? &jungleDaySounds_ : &jungleNightSounds_;
            break;
        case ZoneType::MARSH:
            zoneLibrary = isDay ? &marshDaySounds_ : &marshNightSounds_;
            break;
        case ZoneType::DESERT_CANYON:
            zoneLibrary = isDay ? &desertCanyonDaySounds_ : &desertCanyonNightSounds_;
            break;
        case ZoneType::DESERT_PLAINS:
            zoneLibrary = isDay ? &desertPlainsDaySounds_ : &desertPlainsNightSounds_;
            break;
        default:
            return;
    }

    // Play zone ambience sound if library is loaded and timer expired
    if (zoneLibrary && !zoneLibrary->empty() && (*zoneLibrary)[0].loaded) {
        // Play every 30 seconds for zone ambience (longer intervals for background atmosphere)
        if (zoneLoopTime_ >= 30.0f) {
            float volume = 0.35f * volumeScale_;  // Zone ambience at moderate-low volume
            AudioEngine::instance().playSound2D((*zoneLibrary)[0].data, volume, 1.0f);
            LOG_INFO("Playing zone ambience: type ", static_cast<int>(currentZone_),
                     " (", isDay ? "day" : "night", ")");
            zoneLoopTime_ = 0.0f;
        }
    }
}

void AmbientSoundManager::updateCityAmbience(float deltaTime) {
    // Only play city ambience when actually in a city
    if (currentCity_ == CityType::NONE) return;

    cityLoopTime_ += deltaTime;

    // Select appropriate sound library based on city type and time of day
    const std::vector<AmbientSample>* cityLibrary = nullptr;
    bool isDay = isDaytime();

    switch (currentCity_) {
        case CityType::STORMWIND:
            cityLibrary = isDay ? &stormwindDaySounds_ : &stormwindNightSounds_;
            break;
        case CityType::IRONFORGE:
            cityLibrary = &ironforgeSounds_;  // No day/night (underground)
            break;
        case CityType::DARNASSUS:
            cityLibrary = isDay ? &darnassusDaySounds_ : &darnassusNightSounds_;
            break;
        case CityType::ORGRIMMAR:
            cityLibrary = isDay ? &orgrimmarDaySounds_ : &orgrimmarNightSounds_;
            break;
        case CityType::UNDERCITY:
            cityLibrary = &undercitySounds_;  // No day/night (underground)
            break;
        case CityType::THUNDERBLUFF:
            cityLibrary = isDay ? &thunderbluffDaySounds_ : &thunderbluffNightSounds_;
            break;
        default:
            return;
    }

    // Play city ambience sound if library is loaded and timer expired
    if (cityLibrary && !cityLibrary->empty() && (*cityLibrary)[0].loaded) {
        // Play every 20 seconds for city ambience (moderate intervals for urban atmosphere)
        if (cityLoopTime_ >= 20.0f) {
            float volume = 0.4f * volumeScale_;  // City ambience at moderate volume
            AudioEngine::instance().playSound2D((*cityLibrary)[0].data, volume, 1.0f);
            LOG_INFO("Playing city ambience: type ", static_cast<int>(currentCity_),
                     " (", isDay ? "day" : "night", ")");
            cityLoopTime_ = 0.0f;
        }
    }
}

void AmbientSoundManager::updateBellTolls(float deltaTime) {
    // Only play bells when in a city
    if (currentCity_ == CityType::NONE) return;

    bellTollTime_ += deltaTime;

    // Select appropriate bell sound based on city faction
    const std::vector<AmbientSample>* bellLibrary = nullptr;

    switch (currentCity_) {
        case CityType::STORMWIND:
        case CityType::IRONFORGE:
            bellLibrary = &bellAllianceSounds_;
            break;
        case CityType::DARNASSUS:
            bellLibrary = &bellNightElfSounds_;
            break;
        case CityType::ORGRIMMAR:
        case CityType::UNDERCITY:
            bellLibrary = &bellHordeSounds_;
            break;
        case CityType::THUNDERBLUFF:
            bellLibrary = &bellTribalSounds_;
            break;
        default:
            return;
    }

    // Play bell toll every 120-180 seconds (2-3 minutes) with random variation
    float bellInterval = randomFloat(120.0f, 180.0f);

    if (bellLibrary && !bellLibrary->empty() && (*bellLibrary)[0].loaded) {
        if (bellTollTime_ >= bellInterval) {
            float volume = 0.5f * volumeScale_;  // Bell tolls at moderate-high volume
            AudioEngine::instance().playSound2D((*bellLibrary)[0].data, volume, 1.0f);
            LOG_INFO("Bell toll ringing in city: type ", static_cast<int>(currentCity_));
            bellTollTime_ = 0.0f;
        }
    }
}

} // namespace audio
} // namespace wowee
