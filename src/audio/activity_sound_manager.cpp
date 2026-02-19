#include "audio/activity_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>

namespace wowee {
namespace audio {

namespace {

std::vector<std::string> buildClassicSet(const std::string& material) {
    std::vector<std::string> out;
    for (char c = 'A'; c <= 'L'; ++c) {
        out.push_back("Sound\\Character\\Footsteps\\mFootMediumLarge" + material + std::string(1, c) + ".wav");
    }
    return out;
}

} // namespace

ActivitySoundManager::ActivitySoundManager() : rng(std::random_device{}()) {}
ActivitySoundManager::~ActivitySoundManager() { shutdown(); }

bool ActivitySoundManager::initialize(pipeline::AssetManager* assets) {
    shutdown();
    assetManager = assets;
    if (!assetManager) return false;

    rebuildJumpClipsForProfile("Human", "Human", true);
    rebuildSwimLoopClipsForProfile("Human", "Human", true);
    rebuildHardLandClipsForProfile("Human", "Human", true);

    preloadCandidates(splashEnterClips, {
        "Sound\\Character\\Footsteps\\EnterWaterSplash\\EnterWaterSmallA.wav",
        "Sound\\Character\\Footsteps\\EnterWaterSplash\\EnterWaterMediumA.wav",
        "Sound\\Character\\Footsteps\\EnterWaterSplash\\EnterWaterGiantA.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterA.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterB.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterC.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterD.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterE.wav"
    });
    splashExitClips = splashEnterClips;

    preloadLandingSet(FootstepSurface::STONE, "Stone");
    preloadLandingSet(FootstepSurface::DIRT, "Dirt");
    preloadLandingSet(FootstepSurface::GRASS, "Grass");
    preloadLandingSet(FootstepSurface::WOOD, "Wood");
    preloadLandingSet(FootstepSurface::METAL, "Metal");
    preloadLandingSet(FootstepSurface::WATER, "Water");
    preloadLandingSet(FootstepSurface::SNOW, "Snow");

    preloadCandidates(meleeSwingClips, {
        "Sound\\Item\\Weapons\\Sword\\SwordSwing1.wav",
        "Sound\\Item\\Weapons\\Sword\\SwordSwing2.wav",
        "Sound\\Item\\Weapons\\Sword\\SwordSwing3.wav",
        "Sound\\Item\\Weapons\\Sword\\SwordHit1.wav",
        "Sound\\Item\\Weapons\\Sword\\SwordHit2.wav",
        "Sound\\Item\\Weapons\\Sword\\SwordHit3.wav",
        "Sound\\Item\\Weapons\\OneHanded\\Sword\\SwordSwing1.wav",
        "Sound\\Item\\Weapons\\OneHanded\\Sword\\SwordSwing2.wav",
        "Sound\\Item\\Weapons\\OneHanded\\Sword\\SwordSwing3.wav",
        "Sound\\Item\\Weapons\\Melee\\MeleeSwing1.wav",
        "Sound\\Item\\Weapons\\Melee\\MeleeSwing2.wav",
        "Sound\\Item\\Weapons\\Melee\\MeleeSwing3.wav"
    });

    initialized = true;
    core::Logger::getInstance().info("Activity SFX loaded: jump=", jumpClips.size(),
                                     " splash=", splashEnterClips.size(),
                                     " swimLoop=", swimLoopClips.size());
    return true;
}

void ActivitySoundManager::shutdown() {
    stopSwimLoop();
    stopOneShot();
    std::remove(loopTempPath.c_str());
    std::remove(oneShotTempPath.c_str());
    for (auto& set : landingSets) set.clips.clear();
    jumpClips.clear();
    splashEnterClips.clear();
    splashExitClips.clear();
    swimLoopClips.clear();
    hardLandClips.clear();
    meleeSwingClips.clear();
    swimmingActive = false;
    swimMoving = false;
    initialized = false;
    assetManager = nullptr;
}

void ActivitySoundManager::update(float deltaTime) {
    reapProcesses();

    // Play swimming stroke sounds periodically when swimming and moving
    if (swimmingActive && swimMoving && !swimLoopClips.empty()) {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - lastSwimStrokeAt).count();

        // Play swimming stroke sound every 0.8 seconds (swim stroke rhythm)
        if (lastSwimStrokeAt.time_since_epoch().count() == 0 || elapsed >= 0.8f) {
            std::uniform_int_distribution<size_t> clipDist(0, swimLoopClips.size() - 1);
            const Sample& sample = swimLoopClips[clipDist(rng)];

            // Play as one-shot 2D sound
            float volume = 0.6f * volumeScale;
            AudioEngine::instance().playSound2D(sample.data, volume, false);

            lastSwimStrokeAt = now;
        }
    } else if (!swimmingActive) {
        // Reset timer when not swimming
        lastSwimStrokeAt = std::chrono::steady_clock::time_point{};
    }
}

void ActivitySoundManager::preloadCandidates(std::vector<Sample>& out, const std::vector<std::string>& candidates) {
    if (!assetManager) return;
    for (const auto& path : candidates) {
        if (!assetManager->fileExists(path)) continue;
        auto data = assetManager->readFile(path);
        if (data.empty()) continue;
        out.push_back({path, std::move(data)});
    }
}

void ActivitySoundManager::preloadLandingSet(FootstepSurface surface, const std::string& material) {
    auto& clips = landingSets[static_cast<size_t>(surface)].clips;
    preloadCandidates(clips, buildClassicSet(material));
}

void ActivitySoundManager::rebuildJumpClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    jumpClips.clear();
    const std::string gender = male ? "Male" : "Female";
    const std::string prefix = "Sound\\Character\\" + raceFolder + "\\";
    const std::string stem = raceBase + gender;
    const std::string genderDir = male ? "Male" : "Female";
    preloadCandidates(jumpClips, {
        // Common WotLK-style variants.
        prefix + stem + "\\" + stem + "Jump01.wav",
        prefix + stem + "\\" + stem + "Jump02.wav",
        prefix + stem + "\\" + stem + "Jump03.wav",
        prefix + stem + "\\" + stem + "Exertion01.wav",
        prefix + stem + "\\" + stem + "Exertion02.wav",
        prefix + stem + "JumpA.wav",
        prefix + stem + "JumpB.wav",
        prefix + stem + "JumpC.wav",
        prefix + stem + "Jump.wav",
        prefix + stem + "JumpStart.wav",
        prefix + stem + "Land.wav",
        prefix + genderDir + "\\" + stem + "JumpA.wav",
        prefix + genderDir + "\\" + stem + "JumpB.wav",
        prefix + genderDir + "\\" + stem + "JumpC.wav",
        prefix + genderDir + "\\" + stem + "Jump.wav",
        prefix + genderDir + "\\" + stem + "JumpStart.wav",
        prefix + raceBase + "JumpA.wav",
        prefix + raceBase + "JumpB.wav",
        prefix + raceBase + "JumpC.wav",
        prefix + raceBase + "Jump.wav",
        prefix + raceBase + "\\" + stem + "JumpA.wav",
        prefix + raceBase + "\\" + stem + "JumpB.wav",
        prefix + raceBase + "\\" + stem + "JumpC.wav",
        // Alternate folder naming in some packs.
        "Sound\\Character\\" + stem + "\\" + stem + "JumpA.wav",
        "Sound\\Character\\" + stem + "\\" + stem + "JumpB.wav",
        "Sound\\Character\\" + stem + "\\" + stem + "Jump.wav",
        // Fallback safety
        "Sound\\Character\\Human\\HumanMaleJumpA.wav",
        "Sound\\Character\\Human\\HumanMaleJumpB.wav",
        "Sound\\Character\\Human\\HumanFemaleJumpA.wav",
        "Sound\\Character\\Human\\HumanFemaleJumpB.wav",
        "Sound\\Character\\Human\\Male\\HumanMaleJumpA.wav",
        "Sound\\Character\\Human\\Male\\HumanMaleJumpB.wav",
        "Sound\\Character\\Human\\Female\\HumanFemaleJumpA.wav",
        "Sound\\Character\\Human\\Female\\HumanFemaleJumpB.wav",
        "Sound\\Character\\Human\\HumanMale\\HumanMaleJump01.wav",
        "Sound\\Character\\Human\\HumanMale\\HumanMaleJump02.wav",
        "Sound\\Character\\Human\\HumanMale\\HumanMaleJump03.wav",
        "Sound\\Character\\Human\\HumanFemale\\HumanFemaleJump01.wav",
        "Sound\\Character\\Human\\HumanFemale\\HumanFemaleJump02.wav",
        "Sound\\Character\\Human\\HumanFemale\\HumanFemaleJump03.wav",
        "Sound\\Character\\HumanMale\\HumanMaleJumpA.wav",
        "Sound\\Character\\HumanMale\\HumanMaleJumpB.wav",
        "Sound\\Character\\HumanFemale\\HumanFemaleJumpA.wav",
        "Sound\\Character\\HumanFemale\\HumanFemaleJumpB.wav"
    });
}

void ActivitySoundManager::rebuildSwimLoopClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    swimLoopClips.clear();

    // WoW 3.3.5a doesn't have dedicated swim loop sounds
    // Use water splash/footstep sounds as swimming stroke sounds
    preloadCandidates(swimLoopClips, {
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterA.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterB.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterC.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterD.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsMediumWaterE.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsSmallWaterA.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsSmallWaterB.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsSmallWaterC.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsSmallWaterD.wav",
        "Sound\\Character\\Footsteps\\WaterSplash\\FootStepsSmallWaterE.wav"
    });
}

void ActivitySoundManager::rebuildHardLandClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    hardLandClips.clear();
    const std::string gender = male ? "Male" : "Female";
    const std::string prefix = "Sound\\Character\\" + raceFolder + "\\";
    const std::string stem = raceBase + gender;
    preloadCandidates(hardLandClips, {
        prefix + stem + "\\" + stem + "LandHard01.wav",
        prefix + stem + "\\" + stem + "LandHard02.wav",
        prefix + stem + "LandHard01.wav",
        prefix + stem + "LandHard02.wav"
    });
}

bool ActivitySoundManager::playOneShot(const std::vector<Sample>& clips, float volume, float pitchLo, float pitchHi) {
    if (clips.empty()) return false;
    if (volumeScale <= 0.0001f || volume <= 0.0001f) return true; // Intentionally muted
    reapProcesses();
    if (oneShotPid != INVALID_PROCESS) return false;

    std::uniform_int_distribution<size_t> clipDist(0, clips.size() - 1);
    const Sample& sample = clips[clipDist(rng)];
    std::ofstream out(oneShotTempPath, std::ios::binary);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(sample.data.data()), static_cast<std::streamsize>(sample.data.size()));
    out.close();

    std::uniform_real_distribution<float> pitchDist(pitchLo, pitchHi);
    float pitch = pitchDist(rng);
    volume *= volumeScale;
    if (volume <= 0.0001f) return true; // Intentionally muted
    if (volume > 1.2f) volume = 1.2f;
    std::string filter = "asetrate=44100*" + std::to_string(pitch) +
                         ",aresample=44100,volume=" + std::to_string(volume);

    oneShotPid = platform::spawnProcess({
        "-nodisp", "-autoexit", "-loglevel", "quiet",
        "-af", filter, oneShotTempPath
    });

    return oneShotPid != INVALID_PROCESS;
}

void ActivitySoundManager::startSwimLoop() {
    // Swimming sounds now handled by periodic playback in update() method
    // This method kept for API compatibility but does nothing
    return;
}

void ActivitySoundManager::stopSwimLoop() {
    platform::killProcess(swimLoopPid);
}

void ActivitySoundManager::stopOneShot() {
    platform::killProcess(oneShotPid);
}

void ActivitySoundManager::reapProcesses() {
    if (oneShotPid != INVALID_PROCESS) {
        platform::isProcessRunning(oneShotPid);
    }
    if (swimLoopPid != INVALID_PROCESS) {
        platform::isProcessRunning(swimLoopPid);
    }
}

void ActivitySoundManager::playJump() {
    if (!AudioEngine::instance().isInitialized() || jumpClips.empty()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (lastJumpAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastJumpAt).count() < 0.35f) return;
    }

    // Pick random clip
    std::uniform_int_distribution<size_t> dist(0, jumpClips.size() - 1);
    const Sample& sample = jumpClips[dist(rng)];

    // Play with slight volume/pitch variation
    std::uniform_real_distribution<float> volumeDist(0.65f, 0.75f);
    std::uniform_real_distribution<float> pitchDist(0.98f, 1.04f);
    float volume = volumeDist(rng) * volumeScale;
    float pitch = pitchDist(rng);

    if (AudioEngine::instance().playSound2D(sample.data, volume, pitch)) {
        lastJumpAt = now;
    }
}

void ActivitySoundManager::playLanding(FootstepSurface surface, bool hardLanding) {
    if (!AudioEngine::instance().isInitialized()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (lastLandAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastLandAt).count() < 0.10f) return;
    }

    const auto& clips = landingSets[static_cast<size_t>(surface)].clips;
    if (!clips.empty()) {
        std::uniform_int_distribution<size_t> dist(0, clips.size() - 1);
        const Sample& sample = clips[dist(rng)];

        float baseVolume = hardLanding ? 1.00f : 0.82f;
        std::uniform_real_distribution<float> volumeDist(baseVolume * 0.95f, baseVolume * 1.05f);
        std::uniform_real_distribution<float> pitchDist(0.95f, 1.03f);

        AudioEngine::instance().playSound2D(sample.data, volumeDist(rng) * volumeScale, pitchDist(rng));
        lastLandAt = now;
    }

    if (hardLanding && !hardLandClips.empty()) {
        std::uniform_int_distribution<size_t> dist(0, hardLandClips.size() - 1);
        const Sample& sample = hardLandClips[dist(rng)];
        std::uniform_real_distribution<float> volumeDist(0.80f, 0.88f);
        std::uniform_real_distribution<float> pitchDist(0.97f, 1.03f);
        AudioEngine::instance().playSound2D(sample.data, volumeDist(rng) * volumeScale, pitchDist(rng));
    }
}

void ActivitySoundManager::playMeleeSwing() {
    if (!AudioEngine::instance().isInitialized() || meleeSwingClips.empty()) {
        if (meleeSwingClips.empty() && !meleeSwingWarned) {
            core::Logger::getInstance().warning("No melee swing SFX found in assets");
            meleeSwingWarned = true;
        }
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (lastMeleeSwingAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastMeleeSwingAt).count() < 0.12f) return;
    }

    std::uniform_int_distribution<size_t> dist(0, meleeSwingClips.size() - 1);
    const Sample& sample = meleeSwingClips[dist(rng)];

    std::uniform_real_distribution<float> volumeDist(0.76f, 0.84f);
    std::uniform_real_distribution<float> pitchDist(0.96f, 1.04f);

    if (AudioEngine::instance().playSound2D(sample.data, volumeDist(rng) * volumeScale, pitchDist(rng))) {
        lastMeleeSwingAt = now;
    }
}

void ActivitySoundManager::setSwimmingState(bool swimming, bool moving) {
    swimMoving = moving;
    if (swimming == swimmingActive) return;
    swimmingActive = swimming;
    if (swimmingActive) {
        LOG_INFO("Swimming started - playing swim loop");
        startSwimLoop();
    } else {
        LOG_INFO("Swimming stopped - stopping swim loop");
        stopSwimLoop();
    }
}

void ActivitySoundManager::setCharacterVoiceProfile(const std::string& modelName) {
    if (!assetManager || modelName.empty()) return;

    std::string lower = modelName;
    for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    bool male = (lower.find("female") == std::string::npos);
    std::string folder = "Human";
    std::string base = "Human";

    struct RaceMap { const char* token; const char* folder; const char* base; };
    static const RaceMap races[] = {
        {"human", "Human", "Human"},
        {"orc", "Orc", "Orc"},
        {"dwarf", "Dwarf", "Dwarf"},
        {"nightelf", "NightElf", "NightElf"},
        {"scourge", "Scourge", "Scourge"},
        {"undead", "Scourge", "Scourge"},
        {"tauren", "Tauren", "Tauren"},
        {"gnome", "Gnome", "Gnome"},
        {"troll", "Troll", "Troll"},
        {"bloodelf", "BloodElf", "BloodElf"},
        {"draenei", "Draenei", "Draenei"},
        {"goblin", "Goblin", "Goblin"},
        {"worgen", "Worgen", "Worgen"},
    };
    for (const auto& r : races) {
        if (lower.find(r.token) != std::string::npos) {
            folder = r.folder;
            base = r.base;
            break;
        }
    }

    std::string key = folder + "|" + base + "|" + (male ? "M" : "F");
    if (key == voiceProfileKey) return;
    voiceProfileKey = key;
    rebuildJumpClipsForProfile(folder, base, male);
    rebuildSwimLoopClipsForProfile(folder, base, male);
    rebuildHardLandClipsForProfile(folder, base, male);
    core::Logger::getInstance().info("Activity SFX voice profile: ", voiceProfileKey,
                                     " jump clips=", jumpClips.size(),
                                     " swim clips=", swimLoopClips.size(),
                                     " hardLand clips=", hardLandClips.size());
}

void ActivitySoundManager::playWaterEnter() {
    LOG_INFO("Water entry detected - attempting to play splash sound");
    auto now = std::chrono::steady_clock::now();
    if (lastSplashAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastSplashAt).count() < 0.20f) {
            LOG_DEBUG("Water splash throttled (too soon)");
            return;
        }
    }
    if (playOneShot(splashEnterClips, 0.95f, 0.95f, 1.05f)) {
        LOG_INFO("Water splash enter sound played");
        lastSplashAt = now;
    } else {
        LOG_ERROR("Failed to play water splash enter sound");
    }
}

void ActivitySoundManager::playWaterExit() {
    LOG_INFO("Water exit detected - attempting to play splash sound");
    auto now = std::chrono::steady_clock::now();
    if (lastSplashAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastSplashAt).count() < 0.20f) {
            LOG_DEBUG("Water splash throttled (too soon)");
            return;
        }
    }
    if (playOneShot(splashExitClips, 0.95f, 0.95f, 1.05f)) {
        LOG_INFO("Water splash exit sound played");
        lastSplashAt = now;
    } else {
        LOG_ERROR("Failed to play water splash exit sound");
    }
}

} // namespace audio
} // namespace wowee
