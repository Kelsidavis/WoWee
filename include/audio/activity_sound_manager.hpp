#pragma once

#include "audio/footstep_manager.hpp"
#include "platform/process.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace audio {

class ActivitySoundManager {
public:
    ActivitySoundManager();
    ~ActivitySoundManager();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();
    void update(float deltaTime);
    bool isInitialized() const { return initialized; }

    void playJump();
    void playLanding(FootstepSurface surface, bool hardLanding);
    void setSwimmingState(bool swimming, bool moving);
    void setCharacterVoiceProfile(const std::string& modelName);
    void playWaterEnter();
    void playWaterExit();
    void playMeleeSwing();
    void playAttackGrunt();
    void playWound(bool isCrit = false);
    void playDeath();
    void setVolumeScale(float scale) { volumeScale = scale; }
    float getVolumeScale() const { return volumeScale; }

private:
    struct Sample {
        std::string path;
        std::vector<uint8_t> data;
    };

    struct SurfaceLandingSet {
        std::vector<Sample> clips;
    };

    bool initialized = false;
    pipeline::AssetManager* assetManager = nullptr;

    std::vector<Sample> jumpClips;
    std::vector<Sample> splashEnterClips;
    std::vector<Sample> splashExitClips;
    std::vector<Sample> swimLoopClips;
    std::vector<Sample> hardLandClips;
    std::vector<Sample> meleeSwingClips;
    std::vector<Sample> attackGruntClips;
    std::vector<Sample> woundClips;
    std::vector<Sample> woundCritClips;
    std::vector<Sample> deathClips;
    std::array<SurfaceLandingSet, 7> landingSets;

    bool swimmingActive = false;
    bool swimMoving = false;
    ProcessHandle swimLoopPid = INVALID_PROCESS;
    ProcessHandle oneShotPid = INVALID_PROCESS;
    std::string loopTempPath = platform::getTempFilePath("wowee_swim_loop.wav");
    std::string oneShotTempPath = platform::getTempFilePath("wowee_activity.wav");
    std::mt19937 rng;

    std::chrono::steady_clock::time_point lastJumpAt{};
    std::chrono::steady_clock::time_point lastLandAt{};
    std::chrono::steady_clock::time_point lastSplashAt{};
    std::chrono::steady_clock::time_point lastMeleeSwingAt{};
    std::chrono::steady_clock::time_point lastAttackGruntAt{};
    std::chrono::steady_clock::time_point lastWoundAt{};
    std::chrono::steady_clock::time_point lastSwimStrokeAt{};
    bool meleeSwingWarned = false;
    std::string voiceProfileKey;
    float volumeScale = 1.0f;

    void preloadCandidates(std::vector<Sample>& out, const std::vector<std::string>& candidates);
    void preloadLandingSet(FootstepSurface surface, const std::string& material);
    void rebuildJumpClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male);
    void rebuildSwimLoopClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male);
    void rebuildHardLandClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male);
    void rebuildCombatVocalClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male);
    bool playOneShot(const std::vector<Sample>& clips, float volume, float pitchLo, float pitchHi);
    void startSwimLoop();
    void stopSwimLoop();
    void stopOneShot();
    void reapProcesses();
};

} // namespace audio
} // namespace wowee
