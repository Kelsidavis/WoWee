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

class SpellSoundManager {
public:
    SpellSoundManager() = default;
    ~SpellSoundManager() = default;

    // Initialization
    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Volume control
    void setVolumeScale(float scale);

    // Magic school types
    enum class MagicSchool {
        FIRE,
        FROST,
        HOLY,
        NATURE,
        SHADOW,
        ARCANE,
        PHYSICAL  // Non-magical abilities
    };

    // Spell power level
    enum class SpellPower {
        LOW,      // Weak spells, low level
        MEDIUM,   // Standard spells
        HIGH      // Powerful spells
    };

    // Spell casting sounds
    void playPrecast(MagicSchool school, SpellPower power);  // Channeling/preparation
    void playCast(MagicSchool school);                        // When spell fires
    void playImpact(MagicSchool school, SpellPower power);    // When spell hits target

    // Specific spell sounds
    void playFireball();
    void playFrostbolt();
    void playLightningBolt();
    void playHeal();
    void playShadowBolt();

private:
    struct SpellSample {
        std::string path;
        std::vector<uint8_t> data;
        bool loaded;
    };

    // Precast sound libraries (channeling)
    std::vector<SpellSample> precastFireLowSounds_;
    std::vector<SpellSample> precastFireMediumSounds_;
    std::vector<SpellSample> precastFireHighSounds_;
    std::vector<SpellSample> precastFrostLowSounds_;
    std::vector<SpellSample> precastFrostMediumSounds_;
    std::vector<SpellSample> precastFrostHighSounds_;
    std::vector<SpellSample> precastHolyLowSounds_;
    std::vector<SpellSample> precastHolyMediumSounds_;
    std::vector<SpellSample> precastHolyHighSounds_;
    std::vector<SpellSample> precastNatureLowSounds_;
    std::vector<SpellSample> precastNatureMediumSounds_;
    std::vector<SpellSample> precastNatureHighSounds_;
    std::vector<SpellSample> precastShadowLowSounds_;
    std::vector<SpellSample> precastShadowMediumSounds_;
    std::vector<SpellSample> precastShadowHighSounds_;
    std::vector<SpellSample> precastArcaneSounds_;

    // Cast sound libraries (spell release)
    std::vector<SpellSample> castFireSounds_;
    std::vector<SpellSample> castFrostSounds_;
    std::vector<SpellSample> castHolySounds_;
    std::vector<SpellSample> castNatureSounds_;
    std::vector<SpellSample> castShadowSounds_;

    // Impact sound libraries (spell hits)
    std::vector<SpellSample> impactFireballSounds_;
    std::vector<SpellSample> impactBlizzardSounds_;
    std::vector<SpellSample> impactHolySounds_;
    std::vector<SpellSample> impactArcaneMissileSounds_;

    // State tracking
    float volumeScale_ = 1.0f;
    bool initialized_ = false;

    // Helper methods
    bool loadSound(const std::string& path, SpellSample& sample, pipeline::AssetManager* assets);
    void playSound(const std::vector<SpellSample>& library, float volumeMultiplier = 1.0f);
    void playRandomSound(const std::vector<SpellSample>& library, float volumeMultiplier = 1.0f);
};

} // namespace audio
} // namespace wowee
