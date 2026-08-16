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
    [[nodiscard]] float getVolumeScale() const { return volumeScale_; }

    // Character size (for water splash intensity)
    enum class CharacterSize {
        SMALL,   // Gnome, Dwarf
        MEDIUM,  // Human, Night Elf, Undead, Troll, Blood Elf, Draenei
        LARGE,   // Orc, Tauren
        GIANT    // Large NPCs, bosses
    };

    // Water interaction sounds
    void playEnterWater(CharacterSize size);        // Jumping into water
    void playWaterFootstep(CharacterSize size);     // Walking/running in water

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
