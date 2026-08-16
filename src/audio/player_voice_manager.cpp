#include "audio/player_voice_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <cstdio>

namespace wowee {
namespace audio {

namespace {

// Where a race's error voice files live and how they are named.
// Nested races: Sound\Character\{folder}\{dirPrefix}{Gender}ErrorMessages\{filePrefix}{Gender}_err_{token}NN.wav
// Flat races (TBC): Sound\Character\{folder}\{filePrefix}{Gender}_err_{token}NN.wav
struct RaceVoiceInfo {
    const char* folder;      // directory under Sound\Character
    const char* dirPrefix;   // prefix of the ErrorMessages subdirectory ("" = none, nullptr = flat layout)
    const char* filePrefix;  // race portion of the file name
};

const RaceVoiceInfo* raceVoiceInfo(uint8_t raceId) {
    // Race IDs match game::Race. Goblin (9) has no player error voices.
    // NOLINTBEGIN(modernize-use-designated-initializers) - a table whose
    // columns are its field names, with the struct in view directly above.
    switch (raceId) {
        case 1:  { static const RaceVoiceInfo v{"Human",    "",         "Human"};    return &v; }
        case 2:  { static const RaceVoiceInfo v{"Orc",      "Orc",      "Orc"};      return &v; }
        case 3:  { static const RaceVoiceInfo v{"Dwarf",    "Dwarf",    "Dwarf"};    return &v; }
        case 4:  { static const RaceVoiceInfo v{"NightElf", "NightElf", "NightElf"}; return &v; }
        case 5:  { static const RaceVoiceInfo v{"Scourge",  "Scourge",  "Undead"};   return &v; }
        case 6:  { static const RaceVoiceInfo v{"Tauren",   "Tauren",   "Tauren"};   return &v; }
        case 7:  { static const RaceVoiceInfo v{"Gnome",    "Gnome",    "Gnome"};    return &v; }
        case 8:  { static const RaceVoiceInfo v{"Troll",    "Troll",    "Troll"};    return &v; }
        case 10: { static const RaceVoiceInfo v{"BloodElf", nullptr,    "BloodElf"}; return &v; }
        case 11: { static const RaceVoiceInfo v{"Draenei",  nullptr,    "Draenei"};  return &v; }
        default: return nullptr;
    }
    // NOLINTEND(modernize-use-designated-initializers)
}

// File name token for each speech type ({race}{gender}_err_{token}NN.wav)
const char* speechToken(PlayerErrorSpeech type) {
    switch (type) {
        case PlayerErrorSpeech::NO_MANA:               return "nomana";
        case PlayerErrorSpeech::NO_RAGE:               return "norage";
        case PlayerErrorSpeech::NO_ENERGY:             return "noenergy";
        case PlayerErrorSpeech::OUT_OF_RANGE:          return "outofrange";
        case PlayerErrorSpeech::GENERIC_NO_TARGET:     return "genericnotarget";
        case PlayerErrorSpeech::INVALID_ATTACK_TARGET: return "invalidattacktarget";
        case PlayerErrorSpeech::SPELL_COOLDOWN:        return "spellcooldown";
        case PlayerErrorSpeech::ABILITY_COOLDOWN:      return "abilitycooldown";
        case PlayerErrorSpeech::ITEM_COOLDOWN:         return "itemcooldown";
        case PlayerErrorSpeech::POTION_COOLDOWN:       return "potioncooldown";
        case PlayerErrorSpeech::NO_AMMO:               return "noammo";
        case PlayerErrorSpeech::AMMO_ONLY:             return "ammoonly";
        case PlayerErrorSpeech::CHEST_IN_USE:          return "chestinuse";
        case PlayerErrorSpeech::INVENTORY_FULL:        return "inventoryfull";
        case PlayerErrorSpeech::BAG_FULL:              return "bagfull";
        case PlayerErrorSpeech::ITEM_MAX_COUNT:        return "itemmaxcount";
        case PlayerErrorSpeech::NOT_A_BAG:             return "notabag";
        case PlayerErrorSpeech::CANT_USE_ITEM:         return "cantuseitem";
        case PlayerErrorSpeech::ITEM_LOCKED:           return "itemlocked";
        case PlayerErrorSpeech::MUST_EQUIP_ITEM:       return "mustequipitem";
        case PlayerErrorSpeech::NOT_EQUIPPABLE:        return "notequippable";
        case PlayerErrorSpeech::CANT_EQUIP_LEVEL:      return "cantequiplevel";
        case PlayerErrorSpeech::CANT_EQUIP_SKILL:      return "cantequipskill";
        case PlayerErrorSpeech::CANT_EQUIP_EVER:       return "cantequipever";
        case PlayerErrorSpeech::CANT_DROP_SOULBOUND:   return "cantdropsoulbounditem";
        case PlayerErrorSpeech::CANT_TRADE_SOULBOUND:  return "canttradesoulbounditem";
        case PlayerErrorSpeech::NOT_ENOUGH_MONEY:      return "notenoughmoney";
        case PlayerErrorSpeech::CANT_LEARN_SPELL:      return "cantlearnspell";
        case PlayerErrorSpeech::LOOT_TOO_FAR:          return "loottoofar";
        case PlayerErrorSpeech::LOOT_DIDNT_KILL:       return "lootdidntkill";
        case PlayerErrorSpeech::CANT_LOOT:             return "cantloot";
        case PlayerErrorSpeech::ALREADY_IN_GROUP:      return "alreadyingroup";
        case PlayerErrorSpeech::PARTY_FULL:            return "partyfull";
        case PlayerErrorSpeech::GUILD_PERMISSIONS:     return "guildpermissions";
        case PlayerErrorSpeech::CANT_CREATE_HERE:      return "cantcreatehere";
    }
    return nullptr;
}

constexpr PlayerErrorSpeech kAllSpeechTypes[] = {
    PlayerErrorSpeech::NO_MANA,
    PlayerErrorSpeech::NO_RAGE,
    PlayerErrorSpeech::NO_ENERGY,
    PlayerErrorSpeech::OUT_OF_RANGE,
    PlayerErrorSpeech::GENERIC_NO_TARGET,
    PlayerErrorSpeech::INVALID_ATTACK_TARGET,
    PlayerErrorSpeech::SPELL_COOLDOWN,
    PlayerErrorSpeech::ABILITY_COOLDOWN,
    PlayerErrorSpeech::ITEM_COOLDOWN,
    PlayerErrorSpeech::POTION_COOLDOWN,
    PlayerErrorSpeech::NO_AMMO,
    PlayerErrorSpeech::AMMO_ONLY,
    PlayerErrorSpeech::CHEST_IN_USE,
    PlayerErrorSpeech::INVENTORY_FULL,
    PlayerErrorSpeech::BAG_FULL,
    PlayerErrorSpeech::ITEM_MAX_COUNT,
    PlayerErrorSpeech::NOT_A_BAG,
    PlayerErrorSpeech::CANT_USE_ITEM,
    PlayerErrorSpeech::ITEM_LOCKED,
    PlayerErrorSpeech::MUST_EQUIP_ITEM,
    PlayerErrorSpeech::NOT_EQUIPPABLE,
    PlayerErrorSpeech::CANT_EQUIP_LEVEL,
    PlayerErrorSpeech::CANT_EQUIP_SKILL,
    PlayerErrorSpeech::CANT_EQUIP_EVER,
    PlayerErrorSpeech::CANT_DROP_SOULBOUND,
    PlayerErrorSpeech::CANT_TRADE_SOULBOUND,
    PlayerErrorSpeech::NOT_ENOUGH_MONEY,
    PlayerErrorSpeech::CANT_LEARN_SPELL,
    PlayerErrorSpeech::LOOT_TOO_FAR,
    PlayerErrorSpeech::LOOT_DIDNT_KILL,
    PlayerErrorSpeech::CANT_LOOT,
    PlayerErrorSpeech::ALREADY_IN_GROUP,
    PlayerErrorSpeech::PARTY_FULL,
    PlayerErrorSpeech::GUILD_PERMISSIONS,
    PlayerErrorSpeech::CANT_CREATE_HERE,
};

} // namespace

bool PlayerVoiceManager::initialize(pipeline::AssetManager* assets) {
    assetManager_ = assets;
    return assetManager_ != nullptr;
}

void PlayerVoiceManager::shutdown() {
    library_.clear();
    loadedRace_ = -1;
    loadedGender_ = -1;
    assetManager_ = nullptr;
}

void PlayerVoiceManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

void PlayerVoiceManager::ensureLibrary(uint8_t raceId, uint8_t gender) {
    if (loadedRace_ == raceId && loadedGender_ == gender) return;

    library_.clear();
    loadedRace_ = raceId;
    loadedGender_ = gender;

    const RaceVoiceInfo* info = raceVoiceInfo(raceId);
    if (!info || !assetManager_) return;

    const char* genderStr = (gender == 1) ? "Female" : "Male";

    std::string dir = std::string("Sound\\Character\\") + info->folder + "\\";
    if (info->dirPrefix) {
        dir += std::string(info->dirPrefix) + genderStr + "ErrorMessages\\";
    }

    size_t totalSamples = 0;
    for (PlayerErrorSpeech type : kAllSpeechTypes) {
        const std::string base = dir + info->filePrefix + genderStr + "_err_" + speechToken(type);
        auto& samples = library_[static_cast<uint8_t>(type)];
        // Variant numbering is sparse (e.g. 02, 04, 06) - probe each slot individually
        for (int n = 1; n <= 8; ++n) {
            char num[8];
            std::snprintf(num, sizeof(num), "%02d", n);
            std::string path = base + num + ".wav";
            if (!assetManager_->fileExists(path)) continue;
            SpeechSample sample;
            sample.path = path;
            sample.data = assetManager_->readFile(path);
            if (!sample.data.empty()) {
                samples.push_back(std::move(sample));
                ++totalSamples;
            }
        }
    }

    LOG_INFO("Player voice: loaded ", totalSamples, " error speech samples for race=",
             static_cast<int>(raceId), " gender=", static_cast<int>(gender));
}

void PlayerVoiceManager::playError(PlayerErrorSpeech type, uint8_t raceId, uint8_t gender) {
    if (!enabled_ || !assetManager_) return;
    if (!AudioEngine::instance().isInitialized()) return;

    auto now = std::chrono::steady_clock::now();
    if (lastPlayTime_.time_since_epoch().count() != 0) {
        float elapsed = std::chrono::duration<float>(now - lastPlayTime_).count();
        if (elapsed < PLAY_COOLDOWN) return;
    }

    ensureLibrary(raceId, gender);

    auto it = library_.find(static_cast<uint8_t>(type));
    if (it == library_.end() || it->second.empty()) return;

    const auto& samples = it->second;
    std::uniform_int_distribution<size_t> dist(0, samples.size() - 1);
    const auto& sample = samples[dist(rng_)];

    if (AudioEngine::instance().playSound2D(sample.data, volumeScale_, 1.0f)) {
        lastPlayTime_ = now;
    }
}

} // namespace audio
} // namespace wowee
