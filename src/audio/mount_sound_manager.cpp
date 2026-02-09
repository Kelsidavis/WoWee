#include "audio/mount_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <random>

namespace wowee {
namespace audio {

MountSoundManager::MountSoundManager() {
    lastSoundUpdate_ = std::chrono::steady_clock::now();
}

MountSoundManager::~MountSoundManager() {
    shutdown();
}

bool MountSoundManager::initialize(pipeline::AssetManager* assets) {
    assetManager_ = assets;
    if (!assetManager_) {
        LOG_WARNING("Mount sound manager: no asset manager");
        return false;
    }

    loadMountSounds();

    int totalSamples = wingFlapSounds_.size() + wingIdleSounds_.size() +
                       horseBreathSounds_.size() + horseMoveSounds_.size();
    LOG_INFO("Mount sound manager initialized (", totalSamples, " clips)");
    return true;
}

void MountSoundManager::shutdown() {
    stopAllMountSounds();
    mounted_ = false;
    wingFlapSounds_.clear();
    wingIdleSounds_.clear();
    horseBreathSounds_.clear();
    horseMoveSounds_.clear();
    assetManager_ = nullptr;
}

void MountSoundManager::loadMountSounds() {
    if (!assetManager_) return;

    // Flying mount wing flaps (movement)
    std::vector<std::string> wingFlapPaths = {
        "Sound\\Creature\\Gryphon\\GryphonWingFlap1.wav",
        "Sound\\Creature\\Gryphon\\GryphonWingFlap2.wav",
        "Sound\\Creature\\Gryphon\\GryphonWingFlap3.wav",
        "Sound\\Creature\\WindRider\\WindRiderWingFlap1.wav",
        "Sound\\Creature\\WindRider\\WindRiderWingFlap2.wav",
    };

    for (const auto& path : wingFlapPaths) {
        MountSample sample;
        if (loadSound(path, sample)) {
            wingFlapSounds_.push_back(std::move(sample));
        }
    }

    // Flying mount idle/hovering
    std::vector<std::string> wingIdlePaths = {
        "Sound\\Creature\\Gryphon\\GryphonIdle1.wav",
        "Sound\\Creature\\Gryphon\\GryphonIdle2.wav",
        "Sound\\Creature\\WindRider\\WindRiderIdle1.wav",
    };

    for (const auto& path : wingIdlePaths) {
        MountSample sample;
        if (loadSound(path, sample)) {
            wingIdleSounds_.push_back(std::move(sample));
        }
    }

    // Ground mount breathing/idle
    std::vector<std::string> horseBreathPaths = {
        "Sound\\Creature\\Horse\\HorseBreath1.wav",
        "Sound\\Creature\\Horse\\HorseBreath2.wav",
        "Sound\\Creature\\Horse\\HorseSnort1.wav",
    };

    for (const auto& path : horseBreathPaths) {
        MountSample sample;
        if (loadSound(path, sample)) {
            horseBreathSounds_.push_back(std::move(sample));
        }
    }

    // Ground mount movement ambient
    std::vector<std::string> horseMovePaths = {
        "Sound\\Creature\\Horse\\HorseWhinny1.wav",
        "Sound\\Creature\\Horse\\HorseWhinny2.wav",
    };

    for (const auto& path : horseMovePaths) {
        MountSample sample;
        if (loadSound(path, sample)) {
            horseMoveSounds_.push_back(std::move(sample));
        }
    }

    if (!wingFlapSounds_.empty()) {
        LOG_INFO("Loaded ", wingFlapSounds_.size(), " wing flap sounds");
    }
    if (!wingIdleSounds_.empty()) {
        LOG_INFO("Loaded ", wingIdleSounds_.size(), " wing idle sounds");
    }
    if (!horseBreathSounds_.empty()) {
        LOG_INFO("Loaded ", horseBreathSounds_.size(), " horse breath sounds");
    }
    if (!horseMoveSounds_.empty()) {
        LOG_INFO("Loaded ", horseMoveSounds_.size(), " horse move sounds");
    }
}

bool MountSoundManager::loadSound(const std::string& path, MountSample& sample) {
    if (!assetManager_ || !assetManager_->fileExists(path)) {
        return false;
    }

    auto data = assetManager_->readFile(path);
    if (data.empty()) {
        return false;
    }

    sample.path = path;
    sample.data = std::move(data);
    return true;
}

void MountSoundManager::update(float deltaTime) {
    if (!mounted_) {
        soundLoopTimer_ = 0.0f;
        return;
    }

    soundLoopTimer_ += deltaTime;
    updateMountSounds();
}

void MountSoundManager::onMount(uint32_t creatureDisplayId, bool isFlying) {
    mounted_ = true;
    currentDisplayId_ = creatureDisplayId;
    currentMountType_ = detectMountType(creatureDisplayId);
    flying_ = isFlying;
    moving_ = false;

    LOG_INFO("Mount sound: mounted on display ID ", creatureDisplayId,
             " type=", static_cast<int>(currentMountType_),
             " flying=", flying_);

    updateMountSounds();
}

void MountSoundManager::onDismount() {
    stopAllMountSounds();
    mounted_ = false;
    currentMountType_ = MountType::NONE;
    currentDisplayId_ = 0;
    flying_ = false;
    moving_ = false;
}

void MountSoundManager::setMoving(bool moving) {
    if (moving_ != moving) {
        moving_ = moving;
        if (mounted_) {
            updateMountSounds();
        }
    }
}

void MountSoundManager::setFlying(bool flying) {
    if (flying_ != flying) {
        flying_ = flying;
        if (mounted_) {
            updateMountSounds();
        }
    }
}

void MountSoundManager::setGrounded(bool grounded) {
    setFlying(!grounded);
}

MountType MountSoundManager::detectMountType(uint32_t creatureDisplayId) const {
    // TODO: Load from CreatureDisplayInfo.dbc or CreatureModelData.dbc
    // For now, use simple heuristics based on common display IDs
    // This is a placeholder - we'd need proper DBC parsing for accuracy

    // Common flying mount display IDs (approximate ranges)
    // Gryphons: ~2300-2310
    // Wyverns: ~1566-1570
    // Drakes: ~25830-25870
    // Phoenixes: ~17890-17900

    if (creatureDisplayId >= 2300 && creatureDisplayId <= 2320) return MountType::FLYING; // Gryphons
    if (creatureDisplayId >= 1560 && creatureDisplayId <= 1580) return MountType::FLYING; // Wyverns
    if (creatureDisplayId >= 25800 && creatureDisplayId <= 25900) return MountType::FLYING; // Drakes
    if (creatureDisplayId >= 17880 && creatureDisplayId <= 17910) return MountType::FLYING; // Phoenixes

    // Most other mounts are ground
    return MountType::GROUND;
}

void MountSoundManager::updateMountSounds() {
    if (!AudioEngine::instance().isInitialized() || !mounted_) {
        return;
    }

    static std::mt19937 rng(std::random_device{}());

    // Flying mounts
    if (currentMountType_ == MountType::FLYING && flying_) {
        if (moving_ && !wingFlapSounds_.empty()) {
            // Wing flaps when moving (play periodically for continuous flapping sound)
            if (soundLoopTimer_ >= 1.2f) {
                std::uniform_int_distribution<size_t> dist(0, wingFlapSounds_.size() - 1);
                const auto& sample = wingFlapSounds_[dist(rng)];
                std::uniform_real_distribution<float> volumeDist(0.4f, 0.5f);
                std::uniform_real_distribution<float> pitchDist(0.95f, 1.05f);
                AudioEngine::instance().playSound2D(
                    sample.data,
                    volumeDist(rng) * volumeScale_,
                    pitchDist(rng)
                );
                soundLoopTimer_ = 0.0f;
                playingMovementSound_ = true;
            }
        } else if (!moving_ && !wingIdleSounds_.empty()) {
            // Idle/hovering sounds (less frequent)
            if (soundLoopTimer_ >= 3.5f) {
                std::uniform_int_distribution<size_t> dist(0, wingIdleSounds_.size() - 1);
                const auto& sample = wingIdleSounds_[dist(rng)];
                std::uniform_real_distribution<float> volumeDist(0.3f, 0.4f);
                std::uniform_real_distribution<float> pitchDist(0.98f, 1.02f);
                AudioEngine::instance().playSound2D(
                    sample.data,
                    volumeDist(rng) * volumeScale_,
                    pitchDist(rng)
                );
                soundLoopTimer_ = 0.0f;
                playingIdleSound_ = true;
            }
        }
    }
    // Ground mounts
    else if (currentMountType_ == MountType::GROUND && !flying_) {
        if (moving_ && !horseMoveSounds_.empty()) {
            // Occasional whinny/ambient sounds while moving
            if (soundLoopTimer_ >= 8.0f) {
                std::uniform_int_distribution<size_t> dist(0, horseMoveSounds_.size() - 1);
                const auto& sample = horseMoveSounds_[dist(rng)];
                std::uniform_real_distribution<float> volumeDist(0.35f, 0.45f);
                std::uniform_real_distribution<float> pitchDist(0.97f, 1.03f);
                AudioEngine::instance().playSound2D(
                    sample.data,
                    volumeDist(rng) * volumeScale_,
                    pitchDist(rng)
                );
                soundLoopTimer_ = 0.0f;
                playingMovementSound_ = true;
            }
        } else if (!moving_ && !horseBreathSounds_.empty()) {
            // Breathing/snorting when idle
            if (soundLoopTimer_ >= 4.5f) {
                std::uniform_int_distribution<size_t> dist(0, horseBreathSounds_.size() - 1);
                const auto& sample = horseBreathSounds_[dist(rng)];
                std::uniform_real_distribution<float> volumeDist(0.25f, 0.35f);
                std::uniform_real_distribution<float> pitchDist(0.98f, 1.02f);
                AudioEngine::instance().playSound2D(
                    sample.data,
                    volumeDist(rng) * volumeScale_,
                    pitchDist(rng)
                );
                soundLoopTimer_ = 0.0f;
                playingIdleSound_ = true;
            }
        }
    }
}

void MountSoundManager::stopAllMountSounds() {
    // Reset state flags
    playingMovementSound_ = false;
    playingIdleSound_ = false;
    soundLoopTimer_ = 0.0f;
}

} // namespace audio
} // namespace wowee
