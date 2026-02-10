#include "audio/combat_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <random>

namespace wowee {
namespace audio {

namespace {
    std::random_device rd;
    std::mt19937 gen(rd());
}

bool CombatSoundManager::initialize(pipeline::AssetManager* assets) {
    if (!assets) {
        LOG_ERROR("CombatSoundManager: AssetManager is null");
        return false;
    }

    LOG_INFO("CombatSoundManager: Initializing...");

    // Load weapon swing sounds
    swingSmallSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshSmall1.wav", swingSmallSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshSmall2.wav", swingSmallSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshSmall3.wav", swingSmallSounds_[2], assets);

    swingMediumSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium1.wav", swingMediumSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium2.wav", swingMediumSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium3.wav", swingMediumSounds_[2], assets);

    swingLargeSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge1.wav", swingLargeSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge2.wav", swingLargeSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge3.wav", swingLargeSounds_[2], assets);

    swingSmallCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshSmallCrit.wav", swingSmallCritSounds_[0], assets);

    swingMediumCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshMediumCrit.wav", swingMediumCritSounds_[0], assets);

    swingLargeCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLargeCrit.wav", swingLargeCritSounds_[0], assets);

    missWhoosh1HSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\MissSwings\\MissWhoosh1Handed.wav", missWhoosh1HSounds_[0], assets);

    missWhoosh2HSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\MissSwings\\MissWhoosh2Handed.wav", missWhoosh2HSounds_[0], assets);

    // Load impact sounds (using 1H axe as base)
    hitFleshSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitFlesh1a.wav", hitFleshSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitFlesh1b.wav", hitFleshSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitFlesh1c.wav", hitFleshSounds_[2], assets);

    hitChainSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitChain1a.wav", hitChainSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitChain1b.wav", hitChainSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitChain1c.wav", hitChainSounds_[2], assets);

    hitPlateSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitPlate1a.wav", hitPlateSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitPlate1b.wav", hitPlateSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitPlate1c.wav", hitPlateSounds_[2], assets);

    hitShieldSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitMetalShield1a.wav", hitShieldSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitMetalShield1b.wav", hitShieldSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitMetalShield1c.wav", hitShieldSounds_[2], assets);

    hitMetalWeaponSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitMetalWeapon1a.wav", hitMetalWeaponSounds_[0], assets);

    hitWoodSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitWood1A.wav", hitWoodSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitWood1B.wav", hitWoodSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitWood1C.wav", hitWoodSounds_[2], assets);

    hitStoneSounds_.resize(3);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitStone1A.wav", hitStoneSounds_[0], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitStone1B.wav", hitStoneSounds_[1], assets);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitStone1C.wav", hitStoneSounds_[2], assets);

    // Load crit impact sounds
    hitFleshCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitFleshCrit.wav", hitFleshCritSounds_[0], assets);

    hitChainCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitChainCrit.wav", hitChainCritSounds_[0], assets);

    hitPlateCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitPlateCrit.wav", hitPlateCritSounds_[0], assets);

    hitShieldCritSounds_.resize(1);
    loadSound("Sound\\Item\\Weapons\\Axe1H\\m1hAxeHitMetalShieldCrit.wav", hitShieldCritSounds_[0], assets);

    // Load emote sounds
    clapSounds_.resize(7);
    for (int i = 0; i < 7; ++i) {
        loadSound("Sound\\Character\\EmoteClap" + std::to_string(i + 1) + ".wav", clapSounds_[i], assets);
    }

    LOG_INFO("CombatSoundManager: Weapon swings - Small: ", swingSmallSounds_[0].loaded ? "YES" : "NO",
             ", Medium: ", swingMediumSounds_[0].loaded ? "YES" : "NO",
             ", Large: ", swingLargeSounds_[0].loaded ? "YES" : "NO");
    LOG_INFO("CombatSoundManager: Impact sounds - Flesh: ", hitFleshSounds_[0].loaded ? "YES" : "NO",
             ", Chain: ", hitChainSounds_[0].loaded ? "YES" : "NO",
             ", Plate: ", hitPlateSounds_[0].loaded ? "YES" : "NO");
    LOG_INFO("CombatSoundManager: Emote sounds - Clap: ", clapSounds_[0].loaded ? "YES" : "NO");

    initialized_ = true;
    LOG_INFO("CombatSoundManager: Initialization complete");
    return true;
}

void CombatSoundManager::shutdown() {
    initialized_ = false;
}

bool CombatSoundManager::loadSound(const std::string& path, CombatSample& sample, pipeline::AssetManager* assets) {
    sample.path = path;
    sample.loaded = false;

    try {
        sample.data = assets->readFile(path);
        if (!sample.data.empty()) {
            sample.loaded = true;
            return true;
        }
    } catch (const std::exception& e) {
        // Silently fail - not all sounds may exist
    }

    return false;
}

void CombatSoundManager::playSound(const std::vector<CombatSample>& library, float volumeMultiplier) {
    if (!initialized_ || library.empty() || !library[0].loaded) return;

    float volume = 0.8f * volumeScale_ * volumeMultiplier;
    AudioEngine::instance().playSound2D(library[0].data, volume, 1.0f);
}

void CombatSoundManager::playRandomSound(const std::vector<CombatSample>& library, float volumeMultiplier) {
    if (!initialized_ || library.empty()) return;

    // Count loaded sounds
    std::vector<const CombatSample*> loadedSounds;
    for (const auto& sample : library) {
        if (sample.loaded) {
            loadedSounds.push_back(&sample);
        }
    }

    if (loadedSounds.empty()) return;

    // Pick random sound
    std::uniform_int_distribution<size_t> dist(0, loadedSounds.size() - 1);
    size_t index = dist(gen);

    float volume = 0.8f * volumeScale_ * volumeMultiplier;
    AudioEngine::instance().playSound2D(loadedSounds[index]->data, volume, 1.0f);
}

void CombatSoundManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

void CombatSoundManager::playWeaponSwing(WeaponSize size, bool isCrit) {
    if (isCrit) {
        switch (size) {
            case WeaponSize::SMALL:
                playSound(swingSmallCritSounds_);
                break;
            case WeaponSize::MEDIUM:
                playSound(swingMediumCritSounds_);
                break;
            case WeaponSize::LARGE:
                playSound(swingLargeCritSounds_);
                break;
        }
    } else {
        switch (size) {
            case WeaponSize::SMALL:
                playRandomSound(swingSmallSounds_);
                break;
            case WeaponSize::MEDIUM:
                playRandomSound(swingMediumSounds_);
                break;
            case WeaponSize::LARGE:
                playRandomSound(swingLargeSounds_);
                break;
        }
    }
}

void CombatSoundManager::playWeaponMiss(bool twoHanded) {
    if (twoHanded) {
        playSound(missWhoosh2HSounds_);
    } else {
        playSound(missWhoosh1HSounds_);
    }
}

void CombatSoundManager::playImpact(WeaponSize weaponSize, ImpactType impactType, bool isCrit) {
    // Select appropriate impact sound library
    const std::vector<CombatSample>* normalLibrary = nullptr;
    const std::vector<CombatSample>* critLibrary = nullptr;

    switch (impactType) {
        case ImpactType::FLESH:
            normalLibrary = &hitFleshSounds_;
            critLibrary = &hitFleshCritSounds_;
            break;
        case ImpactType::CHAIN:
            normalLibrary = &hitChainSounds_;
            critLibrary = &hitChainCritSounds_;
            break;
        case ImpactType::PLATE:
            normalLibrary = &hitPlateSounds_;
            critLibrary = &hitPlateCritSounds_;
            break;
        case ImpactType::SHIELD:
            normalLibrary = &hitShieldSounds_;
            critLibrary = &hitShieldCritSounds_;
            break;
        case ImpactType::METAL_WEAPON:
            normalLibrary = &hitMetalWeaponSounds_;
            break;
        case ImpactType::WOOD:
            normalLibrary = &hitWoodSounds_;
            break;
        case ImpactType::STONE:
            normalLibrary = &hitStoneSounds_;
            break;
    }

    if (!normalLibrary) return;

    // Play crit version if available, otherwise use normal
    if (isCrit && critLibrary && !critLibrary->empty()) {
        playSound(*critLibrary, 1.2f);  // Crits slightly louder
    } else {
        playRandomSound(*normalLibrary);
    }
}

void CombatSoundManager::playClap() {
    playRandomSound(clapSounds_, 0.9f);
}

} // namespace audio
} // namespace wowee
