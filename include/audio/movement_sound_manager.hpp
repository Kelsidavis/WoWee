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

class MovementSoundManager {
public:
    MovementSoundManager() = default;
    ~MovementSoundManager() = default;

    // Initialization
    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Volume control
    void setVolumeScale(float scale);

    // Character size (for water splash intensity)
    enum class CharacterSize {
        SMALL,   // Gnome, Dwarf
        MEDIUM,  // Human, Night Elf, Undead, Troll, Blood Elf, Draenei
        LARGE,   // Orc, Tauren
        GIANT    // Large NPCs, bosses
    };

    // Player race (for jump/land vocalizations)
    enum class PlayerRace {
        HUMAN_MALE,
        HUMAN_FEMALE,
        DWARF_MALE,
        DWARF_FEMALE,
        NIGHT_ELF_MALE,
        NIGHT_ELF_FEMALE,
        ORC_MALE,
        ORC_FEMALE,
        TAUREN_MALE,
        TAUREN_FEMALE,
        TROLL_MALE,
        TROLL_FEMALE,
        UNDEAD_MALE,
        UNDEAD_FEMALE,
        GNOME_MALE,
        GNOME_FEMALE
    };

    // Water interaction sounds
    void playEnterWater(CharacterSize size);        // Jumping into water
    void playWaterFootstep(CharacterSize size);     // Walking/running in water

    // Jump/Land sounds (player vocalizations)
    void playJump(PlayerRace race);
    void playLand(PlayerRace race);

private:
    struct MovementSample {
        std::string path;
        std::vector<uint8_t> data;
        bool loaded;
    };

    // Water splash sound libraries
    std::vector<MovementSample> enterWaterSmallSounds_;
    std::vector<MovementSample> enterWaterMediumSounds_;
    std::vector<MovementSample> enterWaterGiantSounds_;

    std::vector<MovementSample> waterFootstepSmallSounds_;
    std::vector<MovementSample> waterFootstepMediumSounds_;
    std::vector<MovementSample> waterFootstepHugeSounds_;

    // Jump/Land vocal libraries (all 16 race/gender combos)
    std::vector<MovementSample> jumpHumanMaleSounds_;
    std::vector<MovementSample> landHumanMaleSounds_;
    std::vector<MovementSample> jumpHumanFemaleSounds_;
    std::vector<MovementSample> landHumanFemaleSounds_;
    std::vector<MovementSample> jumpDwarfMaleSounds_;
    std::vector<MovementSample> landDwarfMaleSounds_;
    std::vector<MovementSample> jumpDwarfFemaleSounds_;
    std::vector<MovementSample> landDwarfFemaleSounds_;
    std::vector<MovementSample> jumpNightElfMaleSounds_;
    std::vector<MovementSample> landNightElfMaleSounds_;
    std::vector<MovementSample> jumpNightElfFemaleSounds_;
    std::vector<MovementSample> landNightElfFemaleSounds_;
    std::vector<MovementSample> jumpOrcMaleSounds_;
    std::vector<MovementSample> landOrcMaleSounds_;
    std::vector<MovementSample> jumpOrcFemaleSounds_;
    std::vector<MovementSample> landOrcFemaleSounds_;
    std::vector<MovementSample> jumpTaurenMaleSounds_;
    std::vector<MovementSample> landTaurenMaleSounds_;
    std::vector<MovementSample> jumpTaurenFemaleSounds_;
    std::vector<MovementSample> landTaurenFemaleSounds_;
    std::vector<MovementSample> jumpTrollMaleSounds_;
    std::vector<MovementSample> landTrollMaleSounds_;
    std::vector<MovementSample> jumpTrollFemaleSounds_;
    std::vector<MovementSample> landTrollFemaleSounds_;
    std::vector<MovementSample> jumpUndeadMaleSounds_;
    std::vector<MovementSample> landUndeadMaleSounds_;
    std::vector<MovementSample> jumpUndeadFemaleSounds_;
    std::vector<MovementSample> landUndeadFemaleSounds_;
    std::vector<MovementSample> jumpGnomeMaleSounds_;
    std::vector<MovementSample> landGnomeMaleSounds_;
    std::vector<MovementSample> jumpGnomeFemaleSounds_;
    std::vector<MovementSample> landGnomeFemaleSounds_;

    // State tracking
    float volumeScale_ = 1.0f;
    bool initialized_ = false;

    // Helper methods
    bool loadSound(const std::string& path, MovementSample& sample, pipeline::AssetManager* assets);
    void playSound(const std::vector<MovementSample>& library, float volumeMultiplier = 1.0f);
    void playRandomSound(const std::vector<MovementSample>& library, float volumeMultiplier = 1.0f);
};

} // namespace audio
} // namespace wowee
