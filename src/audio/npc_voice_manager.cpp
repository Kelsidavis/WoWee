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

    // Generic NPC greetings using Hello emote (more reliable than Greeting)
    std::vector<std::string> genericPaths = {
        "Sound\\Character\\Human\\HumanMaleHello01.wav",
        "Sound\\Character\\Human\\HumanMaleHello02.wav",
        "Sound\\Character\\Human\\HumanMaleHello03.wav",
        "Sound\\Character\\Human\\HumanFemaleHello01.wav",
        "Sound\\Character\\Human\\HumanFemaleHello02.wav",
        "Sound\\Character\\Dwarf\\DwarfMaleHello01.wav",
        "Sound\\Character\\Dwarf\\DwarfMaleHello02.wav",
        "Sound\\Character\\NightElf\\NightElfMaleHello01.wav",
        "Sound\\Character\\NightElf\\NightElfFemaleHello01.wav",
    };

    auto& genericVoices = voiceLibrary_[VoiceType::GENERIC];
    for (const auto& path : genericPaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            genericVoices.push_back(std::move(sample));
        }
    }

    // Human male
    std::vector<std::string> humanMalePaths = {
        "Sound\\Character\\Human\\HumanMaleHello01.wav",
        "Sound\\Character\\Human\\HumanMaleHello02.wav",
        "Sound\\Character\\Human\\HumanMaleHello03.wav",
        "Sound\\Character\\Human\\HumanMaleYes01.wav",
        "Sound\\Character\\Human\\HumanMaleYes02.wav",
    };
    auto& humanMale = voiceLibrary_[VoiceType::HUMAN_MALE];
    for (const auto& path : humanMalePaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            humanMale.push_back(std::move(sample));
        }
    }

    // Human female
    std::vector<std::string> humanFemalePaths = {
        "Sound\\Character\\Human\\HumanFemaleHello01.wav",
        "Sound\\Character\\Human\\HumanFemaleHello02.wav",
        "Sound\\Character\\Human\\HumanFemaleYes01.wav",
    };
    auto& humanFemale = voiceLibrary_[VoiceType::HUMAN_FEMALE];
    for (const auto& path : humanFemalePaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            humanFemale.push_back(std::move(sample));
        }
    }

    // Dwarf male
    std::vector<std::string> dwarfMalePaths = {
        "Sound\\Character\\Dwarf\\DwarfMaleHello01.wav",
        "Sound\\Character\\Dwarf\\DwarfMaleHello02.wav",
        "Sound\\Character\\Dwarf\\DwarfMaleYes01.wav",
    };
    auto& dwarfMale = voiceLibrary_[VoiceType::DWARF_MALE];
    for (const auto& path : dwarfMalePaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            dwarfMale.push_back(std::move(sample));
        }
    }

    // Night elf male
    std::vector<std::string> nelfMalePaths = {
        "Sound\\Character\\NightElf\\NightElfMaleHello01.wav",
        "Sound\\Character\\NightElf\\NightElfMaleYes01.wav",
    };
    auto& nelfMale = voiceLibrary_[VoiceType::NIGHTELF_MALE];
    for (const auto& path : nelfMalePaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            nelfMale.push_back(std::move(sample));
        }
    }

    // Night elf female
    std::vector<std::string> nelfFemalePaths = {
        "Sound\\Character\\NightElf\\NightElfFemaleHello01.wav",
        "Sound\\Character\\NightElf\\NightElfFemaleYes01.wav",
    };
    auto& nelfFemale = voiceLibrary_[VoiceType::NIGHTELF_FEMALE];
    for (const auto& path : nelfFemalePaths) {
        VoiceSample sample;
        if (loadSound(path, sample)) {
            nelfFemale.push_back(std::move(sample));
        }
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
    if (!AudioEngine::instance().isInitialized()) {
        return;
    }

    // Check cooldown
    auto now = std::chrono::steady_clock::now();
    auto it = lastPlayTime_.find(npcGuid);
    if (it != lastPlayTime_.end()) {
        float elapsed = std::chrono::duration<float>(now - it->second).count();
        if (elapsed < GREETING_COOLDOWN) {
            return;  // Still on cooldown
        }
    }

    // Find voice library for this type
    auto libIt = voiceLibrary_.find(voiceType);
    if (libIt == voiceLibrary_.end() || libIt->second.empty()) {
        // Fall back to generic
        libIt = voiceLibrary_.find(VoiceType::GENERIC);
        if (libIt == voiceLibrary_.end() || libIt->second.empty()) {
            return;  // No voice samples available
        }
    }

    const auto& samples = libIt->second;

    // Pick random voice line
    std::uniform_int_distribution<size_t> dist(0, samples.size() - 1);
    const auto& sample = samples[dist(rng_)];

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
        lastPlayTime_[npcGuid] = now;
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
