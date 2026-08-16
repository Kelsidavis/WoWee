#include "audio/activity_sound_manager.hpp"
#include "audio/footstep_paths.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>

namespace wowee {
namespace audio {

ActivitySoundManager::ActivitySoundManager() : rng(std::random_device{}()) {}
ActivitySoundManager::~ActivitySoundManager() { shutdown(); }

bool ActivitySoundManager::initialize(pipeline::AssetManager* assets) {
    shutdown();
    assetManager = assets;
    if (!assetManager) return false;

    // Voice profile clips (jump, swim, hardLand, combat vocals) are set at
    // character spawn via setCharacterVoiceProfile() with the correct race/gender.

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
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium1.wav",
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium2.wav",
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshMedium3.wav",
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge1.wav",
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge2.wav",
        "Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge3.wav",
        "Sound\\Item\\Weapons\\MissSwings\\MissWhoosh1Handed.wav",
        "Sound\\Item\\Weapons\\MissSwings\\MissWhoosh2Handed.wav"
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

void ActivitySoundManager::update([[maybe_unused]] float deltaTime) {
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

// PlayerExertions naming is inconsistent across races: most use
// {Race}{Gender}Final\{Race}{Gender}Main{Type}{Letter}.wav, but OrcMale's
// folder drops "Final", TaurenFemale and UndeadFemale name files with a
// "Final" stem instead of "Main", and Blizzard shipped typo'd stems for
// HumanFemale ("HumanFeamle") and TrollFemale ("TrollFemal"). Generate every
// observed folder\stem prefix; preloadCandidates skips paths that don't exist.
static std::vector<std::string> exertionPrefixes(const std::string& raceBase, bool male) {
    const std::string gender = male ? "Male" : "Female";
    // Undead uses "Scourge" in raceBase but "Undead" in PlayerExertions
    const std::string race = (raceBase == "Scourge") ? "Undead" : raceBase;
    const std::string stem = race + gender;
    const std::string base = "Sound\\Character\\PlayerExertions\\";

    std::vector<std::string> prefixes;
    prefixes.push_back(base + stem + "Final\\" + stem + "Main");   // most races
    prefixes.push_back(base + stem + "\\" + stem + "Main");        // OrcMale folder
    prefixes.push_back(base + stem + "Final\\" + stem + "Final");  // TaurenFemale/UndeadFemale stem
    if (race == "Human" && !male) {
        prefixes.push_back(base + stem + "Final\\HumanFeamleMain");
    }
    if (race == "Troll" && !male) {
        prefixes.push_back(base + stem + "Final\\TrollFemalMain");
    }
    return prefixes;
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
    preloadCandidates(clips, classicFootstepPaths(material));
}

void ActivitySoundManager::rebuildJumpClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    jumpClips.clear();
    const std::string gender = male ? "Male" : "Female";
    const std::string prefix = "Sound\\Character\\" + raceFolder + "\\";
    const std::string stem = raceBase + gender;

    // PlayerExertions prefixes (same naming quirks as combat vocals)
    std::vector<std::string> candidates;
    for (const auto& exert : exertionPrefixes(raceBase, male)) {
        candidates.push_back(exert + "Jump.wav");
    }
    // movement_sound_manager convention (also verified working)
    candidates.push_back(prefix + stem + "Jump1.wav");
    candidates.push_back(prefix + stem + "Land1.wav");
    // Other common variants
    candidates.push_back(prefix + stem + "JumpA.wav");
    candidates.push_back(prefix + stem + "JumpB.wav");
    candidates.push_back(prefix + stem + "Jump.wav");
    candidates.push_back(prefix + gender + "\\" + stem + "JumpA.wav");
    candidates.push_back(prefix + gender + "\\" + stem + "JumpB.wav");
    candidates.push_back(prefix + stem + "\\" + stem + "Jump01.wav");
    candidates.push_back(prefix + stem + "\\" + stem + "Jump02.wav");
    preloadCandidates(jumpClips, candidates);
    if (jumpClips.empty()) {
        LOG_WARNING("No jump clips found for ", stem);
    } else {
        LOG_INFO("Loaded ", jumpClips.size(), " jump clips for ", stem);
    }
}

void ActivitySoundManager::rebuildSwimLoopClipsForProfile([[maybe_unused]] const std::string& raceFolder, [[maybe_unused]] const std::string& raceBase, [[maybe_unused]] bool male) {
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

void ActivitySoundManager::startSwimLoop() {
    // Swimming sounds now handled by periodic playback in update() method
    // This method kept for API compatibility but does nothing
}

void ActivitySoundManager::stopSwimLoop() {
    platform::killProcess(swimLoopPid);
}

void ActivitySoundManager::stopOneShot() {
}

void ActivitySoundManager::reapProcesses() {
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
    static constexpr RaceMap races[] = {
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
    rebuildCombatVocalClipsForProfile(folder, base, male);
    core::Logger::getInstance().info("Activity SFX voice profile: ", voiceProfileKey,
                                     " jump clips=", jumpClips.size(),
                                     " swim clips=", swimLoopClips.size(),
                                     " hardLand clips=", hardLandClips.size(),
                                     " attackGrunt clips=", attackGruntClips.size(),
                                     " wound clips=", woundClips.size(),
                                     " death clips=", deathClips.size());
}

void ActivitySoundManager::setCharacterVoiceProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    if (!assetManager) return;
    std::string key = raceFolder + "|" + raceBase + "|" + (male ? "M" : "F");
    if (key == voiceProfileKey) return;
    voiceProfileKey = key;
    rebuildJumpClipsForProfile(raceFolder, raceBase, male);
    rebuildSwimLoopClipsForProfile(raceFolder, raceBase, male);
    rebuildHardLandClipsForProfile(raceFolder, raceBase, male);
    rebuildCombatVocalClipsForProfile(raceFolder, raceBase, male);
    core::Logger::getInstance().info("Activity SFX voice profile (explicit): ", voiceProfileKey,
                                     " jump clips=", jumpClips.size(),
                                     " swim clips=", swimLoopClips.size(),
                                     " hardLand clips=", hardLandClips.size(),
                                     " attackGrunt clips=", attackGruntClips.size(),
                                     " wound clips=", woundClips.size(),
                                     " death clips=", deathClips.size());
}

// Splashes go through the mixer like every other activity sound. They used to
// spawn ffplay on a temp file, which allowed exactly one at a time - so wading
// in and out of the shallows dropped the second sound and logged it as an error,
// and with the landing and jump sounds suppressed on water exit that left the
// transition silent.
bool ActivitySoundManager::playSplash(const std::vector<Sample>& clips) {
    if (!AudioEngine::instance().isInitialized() || clips.empty()) return false;
    std::uniform_int_distribution<size_t> clipDist(0, clips.size() - 1);
    const Sample& sample = clips[clipDist(rng)];
    std::uniform_real_distribution<float> pitchDist(0.95f, 1.05f);
    return AudioEngine::instance().playSound2D(sample.data, 0.95f * volumeScale, pitchDist(rng));
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
    if (playSplash(splashEnterClips)) {
        lastSplashAt = now;
    } else {
        LOG_WARNING("Water splash enter sound unavailable (", splashEnterClips.size(), " clips loaded)");
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
    if (playSplash(splashExitClips)) {
        lastSplashAt = now;
    } else {
        LOG_WARNING("Water splash exit sound unavailable (", splashExitClips.size(), " clips loaded)");
    }
}

void ActivitySoundManager::rebuildCombatVocalClipsForProfile(const std::string& raceFolder, const std::string& raceBase, bool male) {
    attackGruntClips.clear();
    woundClips.clear();
    woundCritClips.clear();
    deathClips.clear();

    const std::string gender = male ? "Male" : "Female";
    const std::string stem = raceBase + gender;  // e.g. HumanFemale

    // WoW 3.3.5a has two sound sources for player combat vocalizations:
    //
    // 1) Vox files (some races only):
    //    Sound\Character\{Race}\{Gender}\m{Race}{Gender}{Type}Vox{Letter}.wav
    //    e.g. Sound\Character\Human\Female\mHumanFemaleAttackVoxA.wav
    //
    // 2) PlayerExertions (all races):
    //    Sound\Character\PlayerExertions\{Race}{Gender}Final\{Race}{Gender}Main{Type}{Letter}.wav
    //    e.g. Sound\Character\PlayerExertions\HumanMaleFinal\HumanMaleMainAttackA.wav
    //    EXCEPTIONS:
    //    - OrcMale uses folder "OrcMale" (no "Final" suffix)
    //    - HumanFemale files have Blizzard typo: "HumanFeamle" instead of "HumanFemale"

    const std::vector<std::string> exertPrefixes_ = exertionPrefixes(raceBase, male);
    const std::string voxPrefix = "Sound\\Character\\" + raceFolder + "\\" + gender + "\\m" + stem;

    // Attack grunts
    std::vector<std::string> attackPaths;
    for (char c = 'A'; c <= 'F'; ++c) {
        std::string s(1, c);
        for (const auto& exert : exertPrefixes_) {
            attackPaths.push_back(exert + "Attack" + s + ".wav");
        }
        attackPaths.push_back(voxPrefix + "AttackVox" + s + ".wav");
    }
    preloadCandidates(attackGruntClips, attackPaths);

    // Wound sounds (UndeadFemale ships one variant with no letter suffix)
    std::vector<std::string> woundPaths;
    for (char c = 'A'; c <= 'F'; ++c) {
        std::string s(1, c);
        for (const auto& exert : exertPrefixes_) {
            woundPaths.push_back(exert + "Wound" + s + ".wav");
        }
        woundPaths.push_back(voxPrefix + "WoundVox" + s + ".wav");
    }
    for (const auto& exert : exertPrefixes_) {
        woundPaths.push_back(exert + "Wound.wav");
    }
    preloadCandidates(woundClips, woundPaths);

    // Wound crit sounds. Some races have WoundCrit without a letter suffix,
    // and UndeadFemale's is typo'd "WoundCriatA".
    std::vector<std::string> woundCritPaths;
    for (char c = 'A'; c <= 'C'; ++c) {
        std::string s(1, c);
        for (const auto& exert : exertPrefixes_) {
            woundCritPaths.push_back(exert + "WoundCrit" + s + ".wav");
            woundCritPaths.push_back(exert + "WoundCriat" + s + ".wav");
        }
        woundCritPaths.push_back(voxPrefix + "WoundCriticalVox" + s + ".wav");
    }
    for (const auto& exert : exertPrefixes_) {
        woundCritPaths.push_back(exert + "WoundCrit.wav");
    }
    preloadCandidates(woundCritClips, woundCritPaths);

    // Death sounds (NightElf/Troll/Undead ship "Death" with no letter suffix)
    std::vector<std::string> deathPaths;
    for (char c = 'A'; c <= 'C'; ++c) {
        std::string s(1, c);
        for (const auto& exert : exertPrefixes_) {
            deathPaths.push_back(exert + "Death" + s + ".wav");
        }
        deathPaths.push_back(voxPrefix + "DeathVox" + s + ".wav");
    }
    for (const auto& exert : exertPrefixes_) {
        deathPaths.push_back(exert + "Death.wav");
    }
    preloadCandidates(deathClips, deathPaths);

    LOG_INFO("Combat vocals for ", stem, ": attack=", attackGruntClips.size(),
             " wound=", woundClips.size(), " woundCrit=", woundCritClips.size(),
             " death=", deathClips.size());
    if (!attackGruntClips.empty()) LOG_INFO("  First attack: ", attackGruntClips[0].path);
    if (!woundClips.empty()) LOG_INFO("  First wound: ", woundClips[0].path);
    if (attackGruntClips.empty() && woundClips.empty()) {
        LOG_WARNING("No combat vocal sounds found for ", stem);
        LOG_WARNING("  Tried ", exertPrefixes_.size(), " exert prefixes, first: ",
                    exertPrefixes_.empty() ? "(none)" : exertPrefixes_[0]);
        LOG_WARNING("  Tried vox prefix: ", voxPrefix);
    }
}

void ActivitySoundManager::playAttackGrunt() {
    if (!AudioEngine::instance().isInitialized() || attackGruntClips.empty()) return;
    auto now = std::chrono::steady_clock::now();
    if (lastAttackGruntAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastAttackGruntAt).count() < 1.5f) return;
    }
    // ~30% chance per swing to grunt (not every hit)
    std::uniform_int_distribution<int> chance(0, 9);
    if (chance(rng) > 2) return;

    std::uniform_int_distribution<size_t> dist(0, attackGruntClips.size() - 1);
    const Sample& sample = attackGruntClips[dist(rng)];
    std::uniform_real_distribution<float> volDist(0.55f, 0.70f);
    std::uniform_real_distribution<float> pitchDist(0.96f, 1.04f);
    if (AudioEngine::instance().playSound2D(sample.data, volDist(rng) * volumeScale, pitchDist(rng))) {
        lastAttackGruntAt = now;
    }
}

void ActivitySoundManager::playWound(bool isCrit) {
    if (!AudioEngine::instance().isInitialized()) return;
    auto& clips = (isCrit && !woundCritClips.empty()) ? woundCritClips : woundClips;
    if (clips.empty()) return;
    auto now = std::chrono::steady_clock::now();
    if (lastWoundAt.time_since_epoch().count() != 0) {
        if (std::chrono::duration<float>(now - lastWoundAt).count() < 0.8f) return;
    }
    std::uniform_int_distribution<size_t> dist(0, clips.size() - 1);
    const Sample& sample = clips[dist(rng)];
    float vol = isCrit ? 0.80f : 0.65f;
    std::uniform_real_distribution<float> pitchDist(0.96f, 1.04f);
    if (AudioEngine::instance().playSound2D(sample.data, vol * volumeScale, pitchDist(rng))) {
        lastWoundAt = now;
    }
}
} // namespace audio
} // namespace wowee
