#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <chrono>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace audio {

enum class MountType {
    NONE,
    GROUND,      // Horse, wolf, raptor, etc.
    FLYING,      // Griffin, wyvern, drake, etc.
    SWIMMING     // Sea turtle, etc.
};

struct MountSample {
    std::string path;
    std::vector<uint8_t> data;
};

class MountSoundManager {
public:
    MountSoundManager();
    ~MountSoundManager();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();
    void update(float deltaTime);

    // Called when mounting/dismounting
    void onMount(uint32_t creatureDisplayId, bool isFlying);
    void onDismount();

    // Update movement state
    void setMoving(bool moving);
    void setFlying(bool flying);
    void setGrounded(bool grounded);

    bool isMounted() const { return mounted_; }
    void setVolumeScale(float scale) { volumeScale_ = scale; }

private:
    MountType detectMountType(uint32_t creatureDisplayId) const;
    void updateMountSounds();
    void stopAllMountSounds();
    void loadMountSounds();
    bool loadSound(const std::string& path, MountSample& sample);

    pipeline::AssetManager* assetManager_ = nullptr;
    bool mounted_ = false;
    bool moving_ = false;
    bool flying_ = false;
    MountType currentMountType_ = MountType::NONE;
    uint32_t currentDisplayId_ = 0;
    float volumeScale_ = 1.0f;

    // Mount sound samples (loaded from MPQ)
    std::vector<MountSample> wingFlapSounds_;
    std::vector<MountSample> wingIdleSounds_;
    std::vector<MountSample> horseBreathSounds_;
    std::vector<MountSample> horseMoveSounds_;

    // Sound state tracking
    bool playingMovementSound_ = false;
    bool playingIdleSound_ = false;
    std::chrono::steady_clock::time_point lastSoundUpdate_;
    float soundLoopTimer_ = 0.0f;
};

} // namespace audio
} // namespace wowee
