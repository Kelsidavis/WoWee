#include "audio/npc_voice_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>

namespace wowee {
namespace audio {

NpcVoiceManager::NpcVoiceManager() : rng_(std::random_device{}()) {}

NpcVoiceManager::~NpcVoiceManager() {
    shutdown();
}

bool NpcVoiceManager::initialize(pipeline::AssetManager* assets) {
    assetManager_ = assets;
    if (!assetManager_) {
        LOG_WARNING("NPC voice manager: no asset manager");
        return false;
    }

    // Probe for actual sound file paths (NPC sounds are in Sound\Creature\)
    LOG_INFO("=== Probing for NPC voice files ===");
    std::vector<std::string> testPaths = {
        "Sound\\Creature\\HumanMaleStandardNPC\\HumanMaleStandardNPCGreetings01.ogg",
        "Sound\\Creature\\HumanFemaleStandardNPC\\HumanFemaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\DwarfMaleStandardNPC\\DwarfMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\NightElfMaleStandardNPC\\NightElfMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\OrcMaleStandardNPC\\OrcMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\TaurenFemaleStandardNPC\\TaurenFemaleStandardNPCGreeting01.ogg",
    };
    for (const auto& path : testPaths) {
        bool exists = assetManager_->fileExists(path);
        LOG_INFO("  ", path, ": ", (exists ? "EXISTS" : "NOT FOUND"));
    }
    LOG_INFO("=== Probing for tavern music files ===");
    std::vector<std::string> musicPaths = {
        "Sound\\Music\\GlueScreenMusic\\tavern_01.mp3",
        "Sound\\Music\\GlueScreenMusic\\BC_Alehouse.mp3",
        "Sound\\Music\\ZoneMusic\\Tavern\\tavernAlliance01.mp3",
    };
    for (const auto& path : musicPaths) {
        bool exists = assetManager_->fileExists(path);
        LOG_INFO("  ", path, ": ", (exists ? "EXISTS" : "NOT FOUND"));
    }
    LOG_INFO("===================================");

    loadVoiceSounds();

    int totalSamples = 0;
    for (const auto& [type, samples] : voiceLibrary_) {
        totalSamples += samples.size();
    }
    LOG_INFO("NPC voice manager initialized (", totalSamples, " voice clips)");
    return true;
}

void NpcVoiceManager::shutdown() {
    voiceLibrary_.clear();
    lastPlayTime_.clear();
    assetManager_ = nullptr;
}

void NpcVoiceManager::loadVoiceSounds() {
    if (!assetManager_) return;

    // Load all standard NPC greetings from Wowhead database
    // Note: Human male uses "Greetings" (plural), others use "Greeting" (singular)

    // Generic - mix of all races for variety
    auto& genericVoices = voiceLibrary_[VoiceType::GENERIC];
    for (const auto& path : {
        "Sound\\Creature\\HumanMaleStandardNPC\\HumanMaleStandardNPCGreetings01.ogg",
        "Sound\\Creature\\HumanFemaleStandardNPC\\HumanFemaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\DwarfMaleStandardNPC\\DwarfMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\GnomeMaleStandardNPC\\GnomeMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\NightElfMaleStandardNPC\\NightElfMaleStandardNPCGreeting01.ogg",
        "Sound\\Creature\\OrcMaleStandardNPC\\OrcMaleStandardNPCGreeting01.ogg",
    }) {
        VoiceSample sample;
        if (loadSound(path, sample)) genericVoices.push_back(std::move(sample));
    }

    // Human Male
    auto& humanMale = voiceLibrary_[VoiceType::HUMAN_MALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\HumanMaleStandardNPC\\HumanMaleStandardNPCGreetings0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) humanMale.push_back(std::move(sample));
    }

    // Human Female
    auto& humanFemale = voiceLibrary_[VoiceType::HUMAN_FEMALE];
    for (int i = 1; i <= 5; ++i) {
        std::string path = "Sound\\Creature\\HumanFemaleStandardNPC\\HumanFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) humanFemale.push_back(std::move(sample));
    }

    // Dwarf Male
    auto& dwarfMale = voiceLibrary_[VoiceType::DWARF_MALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\DwarfMaleStandardNPC\\DwarfMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) dwarfMale.push_back(std::move(sample));
    }

    // Gnome Male
    auto& gnomeMale = voiceLibrary_[VoiceType::GNOME_MALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\GnomeMaleStandardNPC\\GnomeMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) gnomeMale.push_back(std::move(sample));
    }

    // Gnome Female
    auto& gnomeFemale = voiceLibrary_[VoiceType::GNOME_FEMALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\GnomeFemaleStandardNPC\\GnomeFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) gnomeFemale.push_back(std::move(sample));
    }

    // Night Elf Male
    auto& nelfMale = voiceLibrary_[VoiceType::NIGHTELF_MALE];
    for (int i = 1; i <= 8; ++i) {
        std::string path = "Sound\\Creature\\NightElfMaleStandardNPC\\NightElfMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) nelfMale.push_back(std::move(sample));
    }

    // Night Elf Female
    auto& nelfFemale = voiceLibrary_[VoiceType::NIGHTELF_FEMALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\NightElfFemaleStandardNPC\\NightElfFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) nelfFemale.push_back(std::move(sample));
    }

    // Orc Male
    auto& orcMale = voiceLibrary_[VoiceType::ORC_MALE];
    for (int i = 1; i <= 5; ++i) {
        std::string path = "Sound\\Creature\\OrcMaleStandardNPC\\OrcMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) orcMale.push_back(std::move(sample));
    }

    // Orc Female
    auto& orcFemale = voiceLibrary_[VoiceType::ORC_FEMALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\OrcFemaleStandardNPC\\OrcFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) orcFemale.push_back(std::move(sample));
    }

    // Tauren Male
    auto& taurenMale = voiceLibrary_[VoiceType::TAUREN_MALE];
    for (int i = 1; i <= 5; ++i) {
        std::string path = "Sound\\Creature\\TaurenMaleStandardNPC\\TaurenMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) taurenMale.push_back(std::move(sample));
    }

    // Tauren Female
    auto& taurenFemale = voiceLibrary_[VoiceType::TAUREN_FEMALE];
    for (int i = 1; i <= 5; ++i) {
        std::string path = "Sound\\Creature\\TaurenFemaleStandardNPC\\TaurenFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) taurenFemale.push_back(std::move(sample));
    }

    // Troll Male
    auto& trollMale = voiceLibrary_[VoiceType::TROLL_MALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\TrollMaleStandardNPC\\TrollMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) trollMale.push_back(std::move(sample));
    }

    // Troll Female
    auto& trollFemale = voiceLibrary_[VoiceType::TROLL_FEMALE];
    for (int i = 1; i <= 5; ++i) {
        std::string path = "Sound\\Creature\\TrollFemaleStandardNPC\\TrollFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) trollFemale.push_back(std::move(sample));
    }

    // Undead Male
    auto& undeadMale = voiceLibrary_[VoiceType::UNDEAD_MALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\UndeadMaleStandardNPC\\UndeadMaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) undeadMale.push_back(std::move(sample));
    }

    // Undead Female
    auto& undeadFemale = voiceLibrary_[VoiceType::UNDEAD_FEMALE];
    for (int i = 1; i <= 6; ++i) {
        std::string path = "Sound\\Creature\\UndeadFemaleStandardNPC\\UndeadFemaleStandardNPCGreeting0" + std::to_string(i) + ".ogg";
        VoiceSample sample;
        if (loadSound(path, sample)) undeadFemale.push_back(std::move(sample));
    }

    // Log loaded voice types
    int totalLoaded = 0;
    for (const auto& [type, samples] : voiceLibrary_) {
        if (!samples.empty()) {
            LOG_INFO("Loaded ", samples.size(), " voice samples for type ", static_cast<int>(type));
            totalLoaded += samples.size();
        }
    }

    if (totalLoaded == 0) {
        LOG_WARNING("NPC voice manager: no voice samples loaded (files may not exist in MPQ)");
    }
}

bool NpcVoiceManager::loadSound(const std::string& path, VoiceSample& sample) {
    if (!assetManager_ || !assetManager_->fileExists(path)) {
        return false;
    }

    auto data = assetManager_->readFile(path);
    if (data.empty()) {
        return false;
    }

    sample.path = path;
    sample.data = std::move(data);
    return true;
}

void NpcVoiceManager::playGreeting(uint64_t npcGuid, VoiceType voiceType, const glm::vec3& position) {
    LOG_INFO("NPC voice: playGreeting called for GUID ", npcGuid);

    if (!AudioEngine::instance().isInitialized()) {
        LOG_WARNING("NPC voice: AudioEngine not initialized");
        return;
    }

    // Check cooldown
    auto now = std::chrono::steady_clock::now();
    auto it = lastPlayTime_.find(npcGuid);
    if (it != lastPlayTime_.end()) {
        float elapsed = std::chrono::duration<float>(now - it->second).count();
        if (elapsed < GREETING_COOLDOWN) {
            LOG_INFO("NPC voice: on cooldown (", elapsed, "s elapsed)");
            return;  // Still on cooldown
        }
    }

    // Find voice library for this type
    auto libIt = voiceLibrary_.find(voiceType);
    if (libIt == voiceLibrary_.end() || libIt->second.empty()) {
        LOG_INFO("NPC voice: No samples for type ", static_cast<int>(voiceType), ", falling back to GENERIC");
        // Fall back to generic
        libIt = voiceLibrary_.find(VoiceType::GENERIC);
        if (libIt == voiceLibrary_.end() || libIt->second.empty()) {
            LOG_WARNING("NPC voice: No voice samples available (library empty)");
            return;  // No voice samples available
        }
    }

    const auto& samples = libIt->second;

    // Pick random voice line
    std::uniform_int_distribution<size_t> dist(0, samples.size() - 1);
    const auto& sample = samples[dist(rng_)];

    LOG_INFO("NPC voice: Playing sound from: ", sample.path);

    // Play with 3D positioning
    std::uniform_real_distribution<float> volumeDist(0.85f, 1.0f);
    std::uniform_real_distribution<float> pitchDist(0.98f, 1.02f);

    bool success = AudioEngine::instance().playSound3D(
        sample.data,
        position,
        volumeDist(rng_) * volumeScale_,
        pitchDist(rng_),
        40.0f  // Max distance for voice
    );

    if (success) {
        LOG_INFO("NPC voice: Sound played successfully");
        lastPlayTime_[npcGuid] = now;
    } else {
        LOG_WARNING("NPC voice: Failed to play sound");
    }
}

VoiceType NpcVoiceManager::detectVoiceType(uint32_t creatureEntry) const {
    // TODO: Use CreatureTemplate.dbc or other data to map creature entry to voice type
    // For now, return generic
    (void)creatureEntry;
    return VoiceType::GENERIC;
}

} // namespace audio
} // namespace wowee
