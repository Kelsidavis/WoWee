#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <random>
#include <chrono>
#include <glm/glm.hpp>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace audio {

struct VoiceSample {
    std::string path;
    std::vector<uint8_t> data;
};

// NPC voice types (based on creature model/gender)
enum class VoiceType {
    HUMAN_MALE,
    HUMAN_FEMALE,
    DWARF_MALE,
    DWARF_FEMALE,
    NIGHTELF_MALE,
    NIGHTELF_FEMALE,
    ORC_MALE,
    ORC_FEMALE,
    TAUREN_MALE,
    TAUREN_FEMALE,
    TROLL_MALE,
    TROLL_FEMALE,
    UNDEAD_MALE,
    UNDEAD_FEMALE,
    GNOME_MALE,
    GNOME_FEMALE,
    GOBLIN_MALE,
    GOBLIN_FEMALE,
    BLOODELF_MALE,
    BLOODELF_FEMALE,
    DRAENEI_MALE,
    DRAENEI_FEMALE,
    GENERIC,  // Fallback
};

class NpcVoiceManager {
public:
    NpcVoiceManager();
    ~NpcVoiceManager();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Play NPC interaction sounds
    void playGreeting(uint64_t npcGuid, VoiceType voiceType, const glm::vec3& position);
    void playFarewell(uint64_t npcGuid, VoiceType voiceType, const glm::vec3& position);
    void playVendor(uint64_t npcGuid, VoiceType voiceType, const glm::vec3& position);
    void playPissed(uint64_t npcGuid, VoiceType voiceType, const glm::vec3& position);

    // Play NPC combat sounds
    void playAggro(uint64_t npcGuid, uint32_t displayId, VoiceType voiceType,
                   const glm::vec3& position);
    void playCombatAttack(uint64_t npcGuid, uint32_t displayId,
                          const glm::vec3& position);

    void setVolumeScale(float scale) { volumeScale_ = scale; }
    [[nodiscard]] float getVolumeScale() const { return volumeScale_; }

private:
    enum class SoundCategory {
        GREETING,
        FAREWELL,
        VENDOR,
        PISSED,
        AGGRO,
        FLEE
    };

    void loadVoiceSounds();
    void loadCreatureAggroSounds();
    bool loadSound(const std::string& path, VoiceSample& sample);
    bool playSoundEntry(uint32_t soundId, const glm::vec3& position);
    void playSound(uint64_t npcGuid, VoiceType voiceType, SoundCategory category, const glm::vec3& position);

    pipeline::AssetManager* assetManager_ = nullptr;
    float volumeScale_ = 1.0f;

    // Voice samples grouped by type and category
    std::unordered_map<VoiceType, std::vector<VoiceSample>> greetingLibrary_;
    std::unordered_map<VoiceType, std::vector<VoiceSample>> farewellLibrary_;
    std::unordered_map<VoiceType, std::vector<VoiceSample>> vendorLibrary_;
    std::unordered_map<VoiceType, std::vector<VoiceSample>> pissedLibrary_;
    std::unordered_map<VoiceType, std::vector<VoiceSample>> aggroLibrary_;
    std::unordered_map<VoiceType, std::vector<VoiceSample>> fleeLibrary_;

    // CreatureDisplayInfo/CreatureModelData sound set ->
    // CreatureSoundData.SoundAggroID. These are the model-specific combat
    // barks used by hostile creatures; the race voice library above is only
    // a fallback for incomplete DBC rows.
    std::unordered_map<uint32_t, uint32_t> creatureAggroSoundByDisplay_;
    std::unordered_map<uint32_t, uint32_t> creatureAttackSoundByDisplay_;

    // Cooldown tracking (prevent spam clicking same NPC)
    std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> lastPlayTime_;
    std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> lastAggroTime_;
    std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> lastCombatVocalTime_;
    std::unordered_map<uint64_t, int> clickCount_;  // Track clicks for pissed sounds
    static constexpr float GREETING_COOLDOWN = 2.0f;  // seconds
    static constexpr float AGGRO_COOLDOWN = 5.0f;     // duplicate AI_REACTION/ATTACKSTART guard
    static constexpr float COMBAT_VOCAL_COOLDOWN = 1.25f;
    static constexpr int PISSED_CLICK_THRESHOLD = 5;  // clicks before pissed

    std::mt19937 rng_;
};

} // namespace audio
} // namespace wowee
