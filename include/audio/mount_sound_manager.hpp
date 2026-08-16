#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <unordered_map>
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

enum class MountFamily {
    UNKNOWN,
    HORSE,
    RAM,
    WOLF,
    TIGER,
    RAPTOR,
    DRAGON,
    KODO,
    MECHANOSTRIDER,
    TALLSTRIDER,
    UNDEAD_HORSE
};

struct MountFamilyHash {
    std::size_t operator()(MountFamily f) const { return static_cast<std::size_t>(f); }
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
    void onMount(uint32_t creatureDisplayId, bool isFlying, const std::string& modelPath = "");
    void onDismount();

    // Update movement state
    void setMoving(bool moving);
    void setFlying(bool flying);
    void setGrounded(bool grounded);

    // Play semantic mount action sounds (triggered on animation state changes)
    void playRearUpSound();   // Rear-up flourish (whinny/roar)
    void playJumpSound();     // Jump start (grunt/snort)
    void playLandSound();     // Landing (thud/hoof)
    void playIdleSound();     // Ambient idle (snort/stomp/breath)

    [[nodiscard]] bool isMounted() const { return mounted_; }
    [[nodiscard]] MountFamily getCurrentMountFamily() const { return currentMountFamily_; }
    void setVolumeScale(float scale) { volumeScale_ = scale; }
    [[nodiscard]] float getVolumeScale() const { return volumeScale_; }

private:
    [[nodiscard]] MountType detectMountType(uint32_t creatureDisplayId) const;
    [[nodiscard]] MountFamily detectMountFamily(uint32_t creatureDisplayId) const;
    [[nodiscard]] MountFamily detectMountFamilyFromPath(const std::string& modelPath) const;
    void updateMountSounds();
    void stopAllMountSounds();
    void loadMountSounds();
    bool loadSound(const std::string& path, MountSample& sample);

    pipeline::AssetManager* assetManager_ = nullptr;
    bool mounted_ = false;
    bool moving_ = false;
    bool flying_ = false;
    MountType currentMountType_ = MountType::NONE;
    MountFamily currentMountFamily_ = MountFamily::UNKNOWN;
    uint32_t currentDisplayId_ = 0;
    float volumeScale_ = 0.7f;

    // Mount sound samples (loaded from MPQ)
    std::vector<MountSample> wingFlapSounds_;
    std::vector<MountSample> wingIdleSounds_;

    // Per-family ground mount sounds
    struct FamilySounds {
        std::vector<MountSample> move;    // Movement ambient (alerts/whinnies/growls)
        std::vector<MountSample> jump;    // Jump effort sounds
        std::vector<MountSample> land;    // Landing wound/thud sounds
        std::vector<MountSample> idle;    // Idle ambient (snorts/breathing/fidgets)
    };
    std::unordered_map<MountFamily, FamilySounds, MountFamilyHash> familySounds_;

    // Helper to get sounds for current family (unknown/unloaded -> silent)
    [[nodiscard]] const FamilySounds& getCurrentFamilySounds() const;

    // Sound state tracking
    bool playingMovementSound_ = false;
    bool playingIdleSound_ = false;
    std::chrono::steady_clock::time_point lastSoundUpdate_;
    std::chrono::steady_clock::time_point lastActionSoundTime_;  // Cooldown for action sounds
    float soundLoopTimer_ = 0.0f;
};

} // namespace audio
} // namespace wowee
