#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <random>
#include <chrono>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace audio {

// Player character error speech lines ("I can't do that", "Not enough mana", ...)
// Each value maps to a Sound\Character\...\{Race}{Gender}_err_{token}NN.wav voice set.
enum class PlayerErrorSpeech : uint8_t {
    NO_MANA,
    NO_RAGE,
    NO_ENERGY,
    OUT_OF_RANGE,
    GENERIC_NO_TARGET,
    INVALID_ATTACK_TARGET,
    SPELL_COOLDOWN,
    ABILITY_COOLDOWN,
    ITEM_COOLDOWN,
    POTION_COOLDOWN,
    NO_AMMO,
    AMMO_ONLY,
    CHEST_IN_USE,
    INVENTORY_FULL,
    BAG_FULL,
    ITEM_MAX_COUNT,
    NOT_A_BAG,
    CANT_USE_ITEM,
    ITEM_LOCKED,
    MUST_EQUIP_ITEM,
    NOT_EQUIPPABLE,
    CANT_EQUIP_LEVEL,
    CANT_EQUIP_SKILL,
    CANT_EQUIP_EVER,
    CANT_DROP_SOULBOUND,
    CANT_TRADE_SOULBOUND,
    NOT_ENOUGH_MONEY,
    CANT_LEARN_SPELL,
    LOOT_TOO_FAR,
    LOOT_DIDNT_KILL,
    CANT_LOOT,
    ALREADY_IN_GROUP,
    PARTY_FULL,
    GUILD_PERMISSIONS,
    CANT_CREATE_HERE,
};

/**
 * Plays the player character's spoken error responses ("I can't do that",
 * "Not enough mana", ...). Voice lines are loaded lazily for the active
 * character's race/gender only. Gated by the "Character Speech" setting.
 */
class PlayerVoiceManager {
public:
    PlayerVoiceManager() = default;
    ~PlayerVoiceManager() = default;

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    void setEnabled(bool enabled) { enabled_ = enabled; }
    [[nodiscard]] bool isEnabled() const { return enabled_; }

    void setVolumeScale(float scale);
    [[nodiscard]] float getVolumeScale() const { return volumeScale_; }

    /// Play a spoken error response for the player's race/gender.
    /// raceId: game Race enum value (1=Human .. 11=Draenei); gender: 0=male, 1=female
    void playError(PlayerErrorSpeech type, uint8_t raceId, uint8_t gender);

private:
    struct SpeechSample {
        std::string path;
        std::vector<uint8_t> data;
    };

    // (Re)load the voice library when the active race/gender changes
    void ensureLibrary(uint8_t raceId, uint8_t gender);

    pipeline::AssetManager* assetManager_ = nullptr;
    bool enabled_ = true;
    float volumeScale_ = 1.0f;

    // Library for the currently loaded race/gender, keyed by PlayerErrorSpeech
    int loadedRace_ = -1;
    int loadedGender_ = -1;
    std::unordered_map<uint8_t, std::vector<SpeechSample>> library_;

    // Global throttle so key-mashing doesn't stack voice lines
    std::chrono::steady_clock::time_point lastPlayTime_{};
    static constexpr float PLAY_COOLDOWN = 1.0f;  // seconds

    std::mt19937 rng_{std::random_device{}()};
};

} // namespace audio
} // namespace wowee
