#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cstdint>

namespace wowee {
namespace pipeline {
class AssetManager;
}

namespace audio {

class CombatSoundManager {
public:
    CombatSoundManager() = default;
    ~CombatSoundManager() = default;

    // Initialization
    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Volume control
    void setVolumeScale(float scale);

    // Weapon swing sounds (whoosh sounds before impact)
    enum class WeaponSize {
        SMALL,   // 1H weapons (daggers, swords, maces)
        MEDIUM,  // 2H weapons (2H swords, axes)
        LARGE    // 2H heavy weapons (polearms, staves)
    };

    void playWeaponSwing(WeaponSize size, bool isCrit = false);
    void playWeaponMiss(bool twoHanded = false);

    // Impact sounds (when weapon hits target)
    enum class ImpactType {
        FLESH,        // Hit unarmored/cloth
        CHAIN,        // Hit chain armor
        PLATE,        // Hit plate armor
        SHIELD,       // Hit shield
        METAL_WEAPON, // Parry/weapon clash
        WOOD,         // Hit wooden objects
        STONE         // Hit stone/rock
    };

    void playImpact(WeaponSize weaponSize, ImpactType impactType, bool isCrit = false);

    // Emote sounds
    void playClap();

    // Player character vocals
    enum class PlayerRace {
        BLOOD_ELF_MALE,
        BLOOD_ELF_FEMALE,
        DRAENEI_MALE,
        DRAENEI_FEMALE
    };

    void playPlayerAttackGrunt(PlayerRace race);
    void playPlayerWound(PlayerRace race, bool isCrit = false);
    void playPlayerDeath(PlayerRace race);

private:
    struct CombatSample {
        std::string path;
        std::vector<uint8_t> data;
        bool loaded;
    };

    // Weapon swing sound libraries
    std::vector<CombatSample> swingSmallSounds_;
    std::vector<CombatSample> swingMediumSounds_;
    std::vector<CombatSample> swingLargeSounds_;
    std::vector<CombatSample> swingSmallCritSounds_;
    std::vector<CombatSample> swingMediumCritSounds_;
    std::vector<CombatSample> swingLargeCritSounds_;
    std::vector<CombatSample> missWhoosh1HSounds_;
    std::vector<CombatSample> missWhoosh2HSounds_;

    // Impact sound libraries (using 1H axe as base - similar across weapon types)
    std::vector<CombatSample> hitFleshSounds_;
    std::vector<CombatSample> hitChainSounds_;
    std::vector<CombatSample> hitPlateSounds_;
    std::vector<CombatSample> hitShieldSounds_;
    std::vector<CombatSample> hitMetalWeaponSounds_;
    std::vector<CombatSample> hitWoodSounds_;
    std::vector<CombatSample> hitStoneSounds_;
    std::vector<CombatSample> hitFleshCritSounds_;
    std::vector<CombatSample> hitChainCritSounds_;
    std::vector<CombatSample> hitPlateCritSounds_;
    std::vector<CombatSample> hitShieldCritSounds_;

    // Emote sounds
    std::vector<CombatSample> clapSounds_;

    // Player character vocal libraries
    std::vector<CombatSample> bloodElfMaleAttackSounds_;
    std::vector<CombatSample> bloodElfMaleWoundSounds_;
    std::vector<CombatSample> bloodElfMaleWoundCritSounds_;
    std::vector<CombatSample> bloodElfMaleDeathSounds_;

    std::vector<CombatSample> bloodElfFemaleAttackSounds_;
    std::vector<CombatSample> bloodElfFemaleWoundSounds_;
    std::vector<CombatSample> bloodElfFemaleDeathSounds_;

    std::vector<CombatSample> draeneiMaleAttackSounds_;
    std::vector<CombatSample> draeneiMaleWoundSounds_;
    std::vector<CombatSample> draeneiMaleWoundCritSounds_;
    std::vector<CombatSample> draeneiMaleDeathSounds_;

    std::vector<CombatSample> draeneiFemaleAttackSounds_;
    std::vector<CombatSample> draeneiFemaleWoundSounds_;
    std::vector<CombatSample> draeneiFemaleWoundCritSounds_;
    std::vector<CombatSample> draeneiFemaleDeathSounds_;

    // State tracking
    float volumeScale_ = 1.0f;
    bool initialized_ = false;

    // Helper methods
    bool loadSound(const std::string& path, CombatSample& sample, pipeline::AssetManager* assets);
    void playSound(const std::vector<CombatSample>& library, float volumeMultiplier = 1.0f);
    void playRandomSound(const std::vector<CombatSample>& library, float volumeMultiplier = 1.0f);
};

} // namespace audio
} // namespace wowee
