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

    LOG_INFO("AmbientSoundManager: Wind loaded: ", windLoaded ? "YES" : "NO",
             ", Tavern loaded: ", tavernLoaded ? "YES" : "NO",
             ", Blacksmith loaded: ", blacksmithLoaded ? "YES" : "NO");

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

} // namespace audio
} // namespace wowee
