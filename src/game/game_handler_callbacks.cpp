#include "game/game_handler.hpp"
#include "game/packed_time.hpp"
#include "game/game_utils.hpp"
#include "game/chat_handler.hpp"
#include "game/movement_handler.hpp"
#include "game/combat_handler.hpp"
#include "game/spell_handler.hpp"
#include "game/inventory_handler.hpp"
#include "game/social_handler.hpp"
#include "game/quest_handler.hpp"
#include "game/warden_handler.hpp"
#include "game/packet_parsers.hpp"
#include "game/transport_manager.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
#include "game/opcodes.hpp"
#include "game/update_field_table.hpp"
#include "game/expansion_profile.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/spell_visual_system.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "auth/crypto.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include "rendering/animation/animation_ids.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <vector>
#include <random>
#include <zlib.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <array>
#include <cstdlib>
#include <cstdio>
#include <climits>
#include <cstring>
#include <limits>
#include <openssl/sha.h>
#include <openssl/hmac.h>


namespace wowee {
namespace game {

namespace {

const char* worldStateName(WorldState state) {
    switch (state) {
        case WorldState::DISCONNECTED: return "DISCONNECTED";
        case WorldState::CONNECTING: return "CONNECTING";
        case WorldState::CONNECTED: return "CONNECTED";
        case WorldState::CHALLENGE_RECEIVED: return "CHALLENGE_RECEIVED";
        case WorldState::AUTH_SENT: return "AUTH_SENT";
        case WorldState::AUTHENTICATED: return "AUTHENTICATED";
        case WorldState::READY: return "READY";
        case WorldState::CHAR_LIST_REQUESTED: return "CHAR_LIST_REQUESTED";
        case WorldState::CHAR_LIST_RECEIVED: return "CHAR_LIST_RECEIVED";
        case WorldState::ENTERING_WORLD: return "ENTERING_WORLD";
        case WorldState::IN_WORLD: return "IN_WORLD";
        case WorldState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

std::string lowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool containsAnyTerm(const std::string& haystack, const char* const* terms, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (haystack.find(terms[i]) != std::string::npos) return true;
    }
    return false;
}

// GAMEOBJECT_TYPE_FISHINGHOLE. A fishing school is not a container: retail
// gives it no interact cursor at all, and the only way to take from it is to
// fish in it. Clicking one here sent CMSG_GAMEOBJ_USE and, while its metadata
// was still pending, a CMSG_LOOT as well — which the server answers with the
// hole's loot, harvesting the school in one right-click.
constexpr uint32_t kGoTypeFishingHole = 25;

bool isLootContainerName(const std::string& name) {
    const std::string lower = lowerCopy(name);
    static constexpr const char* kContainerTerms[] = {
        "chest", "lockbox", "strongbox", "coffer", "cache", "bundle",
        "sack", "bag", "crate", "barrel", "basket", "oats"
    };
    return containsAnyTerm(lower, kContainerTerms,
                           sizeof(kContainerTerms) / sizeof(kContainerTerms[0]));
}

// How the local player should open a game object, decided from its Lock.dbc row.
enum class LockOpenMethod {
    UseDirect,  // CMSG_GAMEOBJ_USE. Opens unlocked/quest containers and, crucially,
                // key-item chests: the server credits the key (from a bag) and opens
                // it. keyItemId names the key so the caller can ensure it's in a bag.
    CastSpell,  // cast a KNOWN player OPEN_LOCK spell at the GO (rogue Pick Lock).
                // Generic Opening spells the player does not know are rejected by
                // the server (BAD_TARGETS), so we never fabricate a cast.
};

struct LockOpenPlan {
    LockOpenMethod method = LockOpenMethod::UseDirect;
    uint32_t spellId = 0;
    uint32_t keyItemId = 0;  // required key item (LOCK_KEY_ITEM), 0 if none
};

// Decide how to open a game object from its Lock.dbc row. Only a known player
// OPEN_LOCK spell yields CastSpell; every other case (including key-item locks)
// is UseDirect, with keyItemId set so the caller can move the key into a bag.
LockOpenPlan planGameObjectOpen(pipeline::AssetManager* assets,
                                const GameObjectQueryResponseData* info,
                                const std::unordered_set<uint32_t>& knownSpells) {
    LockOpenPlan plan;
    if (!assets || !info || !info->hasData || info->data[0] == 0) return plan;

    auto lockDbc = assets->loadDBC("Lock.dbc");
    auto spellDbc = assets->loadDBC("Spell.dbc");
    if (!lockDbc || !spellDbc || !lockDbc->isLoaded() || !spellDbc->isLoaded() ||
        lockDbc->getFieldCount() < 33 || spellDbc->getFieldCount() < 234) {
        return plan; // Can't inspect the lock — let the server adjudicate a USE.
    }

    const uint32_t lockId = info->data[0];
    for (uint32_t row = 0; row < lockDbc->getRecordCount(); ++row) {
        if (lockDbc->getUInt32(row, 0) != lockId) continue;

        for (uint32_t slot = 0; slot < 8; ++slot) {
            // Lock.dbc: Type[8] at cols 1-8, Index[8] at cols 9-16.
            constexpr uint32_t kLockKeyItem = 1;   // requires a key item
            constexpr uint32_t kLockKeySkill = 2;  // requires a skill (LockType id)
            const uint32_t keyType = lockDbc->getUInt32(row, 1 + slot);
            const uint32_t keyIndex = lockDbc->getUInt32(row, 9 + slot);
            if (keyIndex == 0) continue;

            if (keyType == kLockKeyItem) {
                plan.keyItemId = keyIndex; // remember the key so we can locate it
                continue;
            }
            if (keyType != kLockKeySkill) continue;

            // A known player spell whose OPEN_LOCK effect matches this LockType
            // lets us open it directly (Lockpicking, etc.).
            for (uint32_t spellRow = 0; spellRow < spellDbc->getRecordCount(); ++spellRow) {
                const uint32_t spellId = spellDbc->getUInt32(spellRow, 0);
                if (knownSpells.count(spellId) == 0) continue;
                for (uint32_t effect = 0; effect < 3; ++effect) {
                    constexpr uint32_t kSpellEffectOpenLock = 33;
                    if (spellDbc->getUInt32(spellRow, 71 + effect) == kSpellEffectOpenLock &&
                        spellDbc->getUInt32(spellRow, 110 + effect) == keyIndex) {
                        plan.method = LockOpenMethod::CastSpell;
                        plan.spellId = spellId;
                        return plan;
                    }
                }
            }
        }
        break;
    }
    return plan;
}


uint32_t gatherSpellForGameObject(const GameObjectQueryResponseData* info, const std::string& name) {
    if (info && info->type != 3) return 0; // GAMEOBJECT_TYPE_CHEST

    const std::string lower = lowerCopy(name);
    static constexpr const char* kMiningTerms[] = {
        "vein", "deposit", "mineral"
    };
    static constexpr const char* kHerbTerms[] = {
        "peacebloom", "silverleaf", "earthroot", "mageroyal", "briarthorn",
        "stranglekelp", "bruiseweed", "steelbloom", "grave moss", "kingsblood",
        "liferoot", "fadeleaf", "goldthorn", "khadgar", "wintersbite",
        "firebloom", "purple lotus", "arthas", "sungrass", "blindweed",
        "ghost mushroom", "gromsblood", "dreamfoil", "silversage",
        "plaguebloom", "icecap", "black lotus", "felweed", "dreaming glory",
        "terocone", "ragveil", "ancient lichen", "netherbloom",
        "nightmare vine", "mana thistle"
    };

    if (containsAnyTerm(lower, kMiningTerms, sizeof(kMiningTerms) / sizeof(kMiningTerms[0]))) return 2575; // Mining
    if (containsAnyTerm(lower, kHerbTerms, sizeof(kHerbTerms) / sizeof(kHerbTerms[0]))) return 2366; // Herb Gathering
    return 0;
}

uint32_t knownGatherRank(const SpellHandler* spellHandler, uint32_t baseSpellId) {
    if (!spellHandler) return 0;

    static constexpr uint32_t kMiningRanks[] = {
        2575, 2576, 3564, 10248, 29354
    };
    static constexpr uint32_t kHerbRanks[] = {
        2366, 2368, 3570, 11993, 28695
    };

    const uint32_t* ranks = nullptr;
    size_t count = 0;
    if (baseSpellId == kMiningRanks[0]) {
        ranks = kMiningRanks;
        count = sizeof(kMiningRanks) / sizeof(kMiningRanks[0]);
    } else if (baseSpellId == kHerbRanks[0]) {
        ranks = kHerbRanks;
        count = sizeof(kHerbRanks) / sizeof(kHerbRanks[0]);
    } else {
        return 0;
    }

    for (size_t i = count; i > 0; --i) {
        const uint32_t spellId = ranks[i - 1];
        if (spellHandler->hasKnownSpell(spellId)) return spellId;
    }
    return 0;
}

} // end anonymous namespace

void GameHandler::handleAuthChallenge(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_CHALLENGE");

    AuthChallengeData challenge;
    if (!AuthChallengeParser::parse(packet, challenge)) {
        fail("Failed to parse SMSG_AUTH_CHALLENGE");
        return;
    }

    if (!challenge.isValid()) {
        fail("Invalid auth challenge data");
        return;
    }

    // Store server seed
    serverSeed = challenge.serverSeed;
    LOG_DEBUG("Server seed: 0x", std::hex, serverSeed, std::dec);

    setState(WorldState::CHALLENGE_RECEIVED);

    // Send authentication session
    sendAuthSession();
}

void GameHandler::sendAuthSession() {
    LOG_INFO("Sending CMSG_AUTH_SESSION");

    // Build authentication packet
    auto packet = AuthSessionPacket::build(
        build,
        accountName,
        clientSeed,
        sessionKey,
        serverSeed,
        realmId_
    );

    LOG_DEBUG("CMSG_AUTH_SESSION packet size: ", packet.getSize(), " bytes");

    // Send packet (unencrypted - this is the last unencrypted packet)
    socket->send(packet);

    // Enable encryption IMMEDIATELY after sending AUTH_SESSION
    // AzerothCore enables encryption before sending AUTH_RESPONSE,
    // so we need to be ready to decrypt the response
    LOG_INFO("Enabling encryption immediately after AUTH_SESSION");
    socket->initEncryption(sessionKey, build);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption enabled, waiting for AUTH_RESPONSE...");
}

void GameHandler::handleAuthResponse(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_AUTH_RESPONSE, size=", packet.getSize());

    AuthResponseData response;
    if (!AuthResponseParser::parse(packet, response)) {
        fail("Failed to parse SMSG_AUTH_RESPONSE");
        return;
    }

    if (!response.isSuccess()) {
        std::string reason = std::string("Authentication failed: ") +
                           getAuthResultString(response.result);
        fail(reason);
        return;
    }

    // Encryption was already enabled after sending AUTH_SESSION
    LOG_INFO("AUTH_RESPONSE OK - world authentication successful");

    setState(WorldState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   WORLD AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("Connected to world server");
    LOG_INFO("Ready for character operations");

    setState(WorldState::READY);

    // Request character list automatically
    requestCharacterList();

    // Call success callback
    if (onSuccess) {
        onSuccess();
    }
}

void GameHandler::requestCharacterList() {
    if (requiresWarden_) {
        // Gate already surfaced via failure callback/chat; avoid per-frame warning spam.
        wardenCharEnumBlockedLogged_ = true;
        return;
    }

    if (state == WorldState::FAILED || !socket || !socket->isConnected()) {
        return;
    }

    if (state != WorldState::READY && state != WorldState::AUTHENTICATED &&
        state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot request character list in state: ", worldStateName(state));
        return;
    }

    LOG_INFO("Requesting character list from server...");

    // Prevent the UI from showing/selecting stale characters while we wait for the new SMSG_CHAR_ENUM.
    // This matters after character create/delete where the old list can linger for a few frames.
    characters.clear();

    // Build CMSG_CHAR_ENUM packet (no body, just opcode)
    auto packet = CharEnumPacket::build();

    // Send packet
    socket->send(packet);

    setState(WorldState::CHAR_LIST_REQUESTED);
    LOG_INFO("CMSG_CHAR_ENUM sent, waiting for character list...");
}

void GameHandler::handleCharEnum(network::Packet& packet) {
    LOG_INFO("Handling SMSG_CHAR_ENUM");

    CharEnumResponse response;
    // IMPORTANT: Do not infer packet formats from numeric build alone.
    // Turtle WoW uses a "high" build but classic-era world packet formats.
    bool parsed = packetParsers_ ? packetParsers_->parseCharEnum(packet, response)
                                 : CharEnumParser::parse(packet, response);
    if (!parsed) {
        fail("Failed to parse SMSG_CHAR_ENUM");
        return;
    }

    // Store characters
    characters = response.characters;

    setState(WorldState::CHAR_LIST_RECEIVED);

    std::vector<uint32_t> queriedGuildIds;
    for (const auto& character : characters) {
        if (!character.hasGuild()) continue;
        if (std::find(queriedGuildIds.begin(), queriedGuildIds.end(), character.guildId) != queriedGuildIds.end())
            continue;
        queriedGuildIds.push_back(character.guildId);
        queryGuildInfo(character.guildId);
    }
    if (!queriedGuildIds.empty()) {
        LOG_INFO("Queued guild name queries for ", queriedGuildIds.size(), " guild(s) from character list");
    }

    LOG_INFO("========================================");
    LOG_INFO("   CHARACTER LIST RECEIVED");
    LOG_INFO("========================================");
    LOG_INFO("Found ", characters.size(), " character(s)");

    if (characters.empty()) {
        LOG_INFO("No characters on this account");
    } else {
        LOG_INFO("Characters:");
        for (size_t i = 0; i < characters.size(); ++i) {
            const auto& character = characters[i];
            LOG_INFO("  [", i + 1, "] ", character.name);
            LOG_INFO("      GUID: 0x", std::hex, character.guid, std::dec);
            LOG_INFO("      ", getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            LOG_INFO("      Level ", static_cast<int>(character.level));
        }
    }

    LOG_INFO("Ready to select character");
}

void GameHandler::createCharacter(const CharCreateData& data) {

    // Online mode: send packet to server
    if (!socket) {
        LOG_WARNING("Cannot create character: not connected");
        if (charCreateCallback_) {
            charCreateCallback_(false, "Not connected to server");
        }
        return;
    }

    if (requiresWarden_) {
        std::string msg = "Server requires anti-cheat/Warden; character creation blocked.";
        LOG_WARNING("Blocking CMSG_CHAR_CREATE while Warden gate is active");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
        return;
    }

    if (state != WorldState::CHAR_LIST_RECEIVED) {
        std::string msg = "Character list not ready yet. Wait for SMSG_CHAR_ENUM.";
        LOG_WARNING("Blocking CMSG_CHAR_CREATE in state=", worldStateName(state),
                    " (awaiting CHAR_LIST_RECEIVED)");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
        return;
    }

    auto packet = CharCreatePacket::build(data);
    socket->send(packet);
    LOG_INFO("CMSG_CHAR_CREATE sent for: ", data.name);
}

void GameHandler::handleCharCreateResponse(network::Packet& packet) {
    CharCreateResponseData data;
    if (!CharCreateResponseParser::parse(packet, data)) {
        LOG_ERROR("Failed to parse SMSG_CHAR_CREATE");
        return;
    }

    if (data.result == CharCreateResult::SUCCESS || data.result == CharCreateResult::IN_PROGRESS) {
        LOG_INFO("Character created successfully (code=", static_cast<int>(data.result), ")");
        requestCharacterList();
        if (charCreateCallback_) {
            charCreateCallback_(true, "Character created!");
        }
    } else {
        std::string msg;
        switch (data.result) {
            case CharCreateResult::CHAR_ERROR: msg = "Server error"; break;
            case CharCreateResult::FAILED: msg = "Creation failed"; break;
            case CharCreateResult::NAME_IN_USE: msg = "Name already in use"; break;
            case CharCreateResult::DISABLED: msg = "Character creation disabled"; break;
            case CharCreateResult::PVP_TEAMS_VIOLATION: msg = "PvP faction violation"; break;
            case CharCreateResult::SERVER_LIMIT: msg = "Server character limit reached"; break;
            case CharCreateResult::ACCOUNT_LIMIT: msg = "Account character limit reached"; break;
            case CharCreateResult::SERVER_QUEUE: msg = "Server is queued"; break;
            case CharCreateResult::ONLY_EXISTING: msg = "Only existing characters allowed"; break;
            case CharCreateResult::EXPANSION: msg = "Expansion required"; break;
            case CharCreateResult::EXPANSION_CLASS: msg = "Expansion required for this class"; break;
            case CharCreateResult::LEVEL_REQUIREMENT: msg = "Level requirement not met"; break;
            case CharCreateResult::UNIQUE_CLASS_LIMIT: msg = "Unique class limit reached"; break;
            case CharCreateResult::RESTRICTED_RACECLASS: msg = "Race/class combination not allowed"; break;
            case CharCreateResult::IN_PROGRESS: msg = "Character creation in progress..."; break;
            case CharCreateResult::CHARACTER_CHOOSE_RACE: msg = "Please choose a different race"; break;
            case CharCreateResult::CHARACTER_ARENA_LEADER: msg = "Arena team leader restriction"; break;
            case CharCreateResult::CHARACTER_DELETE_MAIL: msg = "Character has mail"; break;
            case CharCreateResult::CHARACTER_SWAP_FACTION: msg = "Faction swap restriction"; break;
            case CharCreateResult::CHARACTER_RACE_ONLY: msg = "Race-only restriction"; break;
            case CharCreateResult::CHARACTER_GOLD_LIMIT: msg = "Gold limit reached"; break;
            case CharCreateResult::FORCE_LOGIN: msg = "Force login required"; break;
            case CharCreateResult::CHARACTER_IN_GUILD: msg = "Character is in a guild"; break;
            // Name validation errors
            case CharCreateResult::NAME_FAILURE: msg = "Invalid name"; break;
            case CharCreateResult::NAME_NO_NAME: msg = "Please enter a name"; break;
            case CharCreateResult::NAME_TOO_SHORT: msg = "Name is too short"; break;
            case CharCreateResult::NAME_TOO_LONG: msg = "Name is too long"; break;
            case CharCreateResult::NAME_INVALID_CHARACTER: msg = "Name contains invalid characters"; break;
            case CharCreateResult::NAME_MIXED_LANGUAGES: msg = "Name mixes languages"; break;
            case CharCreateResult::NAME_PROFANE: msg = "Name contains profanity"; break;
            case CharCreateResult::NAME_RESERVED: msg = "Name is reserved"; break;
            case CharCreateResult::NAME_INVALID_APOSTROPHE: msg = "Invalid apostrophe in name"; break;
            case CharCreateResult::NAME_MULTIPLE_APOSTROPHES: msg = "Name has multiple apostrophes"; break;
            case CharCreateResult::NAME_THREE_CONSECUTIVE: msg = "Name has 3+ consecutive same letters"; break;
            case CharCreateResult::NAME_INVALID_SPACE: msg = "Invalid space in name"; break;
            case CharCreateResult::NAME_CONSECUTIVE_SPACES: msg = "Name has consecutive spaces"; break;
            default: msg = "Unknown error (code " + std::to_string(static_cast<int>(data.result)) + ")"; break;
        }
        LOG_WARNING("Character creation failed: ", msg, " (code=", static_cast<int>(data.result), ")");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
    }
}

void GameHandler::deleteCharacter(uint64_t characterGuid) {
    if (!socket) {
        if (charDeleteCallback_) charDeleteCallback_(false, "Delete failed: not connected to server.");
        return;
    }

    network::Packet packet(wireOpcode(Opcode::CMSG_CHAR_DELETE));
    packet.writeUInt64(characterGuid);
    socket->send(packet);
    pendingCharDeleteResponse_ = true;
    pendingDeleteGuid_ = characterGuid;
    pendingDeleteTimer_ = 0.0f;
    LOG_INFO("CMSG_CHAR_DELETE sent for GUID: 0x", std::hex, characterGuid, std::dec);
}

const Character* GameHandler::getActiveCharacter() const {
    if (activeCharacterGuid_ == 0) return nullptr;
    for (const auto& ch : characters) {
        if (ch.guid == activeCharacterGuid_) return &ch;
    }
    return nullptr;
}

const Character* GameHandler::getFirstCharacter() const {
    if (characters.empty()) return nullptr;
    return &characters.front();
}

void GameHandler::playErrorSpeech(audio::PlayerErrorSpeech type) {
    auto* ac = services_.audioCoordinator;
    if (!ac) return;
    auto* voice = ac->getPlayerVoiceManager();
    if (!voice) return;
    const Character* ch = getActiveCharacter();
    if (!ch) return;
    voice->playError(type, static_cast<uint8_t>(ch->race),
                     static_cast<uint8_t>(toServerGender(ch->gender)));
}

void GameHandler::handleCharLoginFailed(network::Packet& packet) {
    uint8_t reason = packet.readUInt8();

    static const char* reasonNames[] = {
        "Login failed",          // 0
        "World server is down",  // 1
        "Duplicate character",   // 2 (session still active)
        "No instance servers",   // 3
        "Login disabled",        // 4
        "Character not found",   // 5
        "Locked for transfer",   // 6
        "Locked by billing",     // 7
        "Using remote",          // 8
    };
    const char* msg = (reason < 9) ? reasonNames[reason] : "Unknown reason";

    LOG_ERROR("SMSG_CHARACTER_LOGIN_FAILED: reason=", static_cast<int>(reason), " (", msg, ")");

    // Allow the player to re-select a character
    setState(WorldState::CHAR_LIST_RECEIVED);

    if (charLoginFailCallback_) {
        charLoginFailCallback_(msg);
    }
}

void GameHandler::selectCharacter(uint64_t characterGuid) {
    if (state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot select character in state: ", static_cast<int>(state));
        return;
    }

    // Make the selected character authoritative in GameHandler.
    // This avoids relying on UI/Application ordering for appearance-dependent logic.
    activeCharacterGuid_ = characterGuid;

    LOG_INFO("========================================");
    LOG_INFO("   ENTERING WORLD");
    LOG_INFO("========================================");
    LOG_INFO("Character GUID: 0x", std::hex, characterGuid, std::dec);

    // Find character name for logging
    for (const auto& character : characters) {
        if (character.guid == characterGuid) {
            LOG_INFO("Character: ", character.name);
            LOG_INFO("Level ", static_cast<int>(character.level), " ",
                     getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            playerRace_ = character.race;
            break;
        }
    }

    // Store player GUID
    playerGuid = characterGuid;

    // Reset per-character state so previous character data doesn't bleed through
    inventory = Inventory();
    onlineItems_.clear();
    itemInfoCache_.clear();
    pendingItemQueries_.clear();
    equipSlotGuids_ = {};
    backpackSlotGuids_ = {};
    keyringSlotGuids_ = {};
    invSlotBase_ = -1;
    packSlotBase_ = -1;
    lastPlayerFields_.clear();
    onlineEquipDirty_ = false;
    playerMoneyCopper_ = 0;
    playerArmorRating_ = 0;
    std::fill(std::begin(playerResistances_), std::end(playerResistances_), 0);
    std::fill(std::begin(playerStats_), std::end(playerStats_), -1);
    playerMeleeAP_ = -1;
    playerRangedAP_ = -1;
    std::fill(std::begin(playerSpellDmgBonus_), std::end(playerSpellDmgBonus_), -1);
    playerHealBonus_ = -1;
    playerDodgePct_ = -1.0f;
    playerParryPct_ = -1.0f;
    playerBlockPct_ = -1.0f;
    playerCritPct_  = -1.0f;
    playerRangedCritPct_ = -1.0f;
    std::fill(std::begin(playerSpellCritPct_), std::end(playerSpellCritPct_), -1.0f);
    std::fill(std::begin(playerCombatRatings_), std::end(playerCombatRatings_), -1);
    if (spellHandler_) spellHandler_->resetAllState();
    if (auto* renderer = services_.renderer) {
        if (auto* camera = renderer->getCameraController()) camera->setIntoxication(0.0f);
        if (auto* post = renderer->getPostProcessPipeline()) post->setIntoxication(0.0f);
    }
    spellFlatMods_.clear();
    spellPctMods_.clear();
    actionBar = {};
    petGuid_ = 0;
    stableWindowOpen_  = false;
    stableMasterGuid_  = 0;
    stableNumSlots_    = 0;
    stabledPets_.clear();
    playerXp_ = 0;
    playerNextLevelXp_ = 0;
    serverPlayerLevel_ = 1;
    std::fill(playerExploredZones_.begin(), playerExploredZones_.end(), 0u);
    hasPlayerExploredZones_ = false;
    playerSkills_.clear();
    // The handler's, not this class's same-named members — every reader
    // forwards there, so clearing the copies here cleared nothing anyone
    // looks at and the previous character's quests stayed in the log.
    if (questHandler_) questHandler_->clearQuestStateForCharacterSwitch();
    pendingLoginQuestResync_ = false;
    pendingLoginQuestResyncTimeout_ = 0.0f;
    if (questHandler_) {
        questHandler_->pendingQuestAcceptTimeoutsRef().clear();
        questHandler_->pendingQuestAcceptNpcGuidsRef().clear();
    }
    if (combatHandler_) combatHandler_->resetAllCombatState();
    // resetCastState() already called inside resetAllState() above
    pendingGameObjectInteractGuid_ = 0;
    lastInteractedGoGuid_ = 0;
    playerDead_ = false;
    releasedSpirit_ = false;
    corpseGuid_ = 0;
    corpsePositionValid_ = false;
    corpseReclaimAvailableMs_ = 0;
    targetGuid = 0;
    focusGuid = 0;
    lastTargetGuid = 0;
    tabCycleStale = true;
    entityController_->clearAll();

    // Build CMSG_PLAYER_LOGIN packet
    auto packet = PlayerLoginPacket::build(characterGuid);

    // Send packet
    socket->send(packet);

    setState(WorldState::ENTERING_WORLD);
    LOG_INFO("CMSG_PLAYER_LOGIN sent, entering world...");
}

void GameHandler::handleLoginSetTimeSpeed(network::Packet& packet) {
    // SMSG_LOGIN_SETTIMESPEED (0x042)
    // Structure: PackedTime gameTime, float timeScale
    //
    // gameTime is a packed bitfield, not a count of seconds — Player.cpp writes
    // it with AppendPackedTime. It was being stored raw and then divided by
    // 86400 to get a time of day, which made the sky's clock a number with no
    // relation to the hour the server sent.
    //
    // timeScale: time speed multiplier (typically 0.0166 for 1 day = 1 hour)

    if (packet.getSize() < 8) {
        LOG_WARNING("SMSG_LOGIN_SETTIMESPEED: packet too small (", packet.getSize(), " bytes)");
        return;
    }

    const WowDate now = unpackWowPackedTime(packet.readUInt32());
    float timeScale = packet.readFloat();

    // Hours since midnight, which is what GetGameTime already assumed and what
    // the sky wants. One meaning for this field, written down here.
    gameTime_ = static_cast<float>(now.hour) + static_cast<float>(now.minute) / 60.0f;
    timeSpeed_ = timeScale;

    LOG_INFO("Server time: ", now.hour, ":", now.minute, " (gameTime=", gameTime_,
             "h), timeSpeed=", timeSpeed_);
    LOG_INFO("  (1 game day = ", (1.0f / timeSpeed_) / 60.0f, " real minutes)");
}

void GameHandler::handleLoginVerifyWorld(network::Packet& packet) {
    LOG_INFO("Handling SMSG_LOGIN_VERIFY_WORLD");
    const bool initialWorldEntry = (state == WorldState::ENTERING_WORLD);

    LoginVerifyWorldData data;
    if (!LoginVerifyWorldParser::parse(packet, data)) {
        fail("Failed to parse SMSG_LOGIN_VERIFY_WORLD");
        return;
    }

    if (!data.isValid()) {
        fail("Invalid world entry data");
        return;
    }

    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(data.x, data.y, data.z));
    const bool alreadyInWorld = (state == WorldState::IN_WORLD);
    const bool sameMap = alreadyInWorld && (currentMapId_ == data.mapId);
    const float dxCurrent = movementInfo.x - canonical.x;
    const float dyCurrent = movementInfo.y - canonical.y;
    const float dzCurrent = movementInfo.z - canonical.z;
    const float distSqCurrent = dxCurrent * dxCurrent + dyCurrent * dyCurrent + dzCurrent * dzCurrent;

    // Some realms emit a late duplicate LOGIN_VERIFY_WORLD after the client is already
    // in-world. Re-running full world-entry handling here can trigger an expensive
    // same-map reload/reset path and starve networking for tens of seconds.
    // The distance test alone is not enough while flying. A taxi carries the
    // player a long way from where the packet's position points, so a late
    // duplicate stops looking like a duplicate and is taken for a teleport —
    // and world entry puts the player back at the position it carries, which
    // is where they logged in. Any of these arriving mid-flight is a duplicate
    // whatever the distance says: the server moves a passenger along the
    // spline, not by sending them into the world again.
    const bool flying = isOnTaxiFlight();
    if (!initialWorldEntry && sameMap &&
        (flying || distSqCurrent <= (5.0f * 5.0f))) {
        LOG_INFO("Ignoring duplicate SMSG_LOGIN_VERIFY_WORLD while already in world: mapId=",
                 data.mapId, " dist=", std::sqrt(distSqCurrent),
                 flying ? " (on a taxi flight)" : "");
        return;
    }

    // Said at warning level when it happens in-world, because from here the
    // player is moved to the position this packet carries. A duplicate that
    // slips past the test above is indistinguishable, after the fact, from a
    // teleport nobody asked for — and one arriving in-world is what puts a
    // player back where they logged in.
    if (!initialWorldEntry) {
        LOG_WARNING("SMSG_LOGIN_VERIFY_WORLD in-world is being treated as a "
                    "teleport: mapId=", data.mapId, " sameMap=", sameMap,
                    " dist=", std::sqrt(distSqCurrent),
                    " onTaxi=", flying,
                    " — the player is about to be placed at (", data.x, ", ",
                    data.y, ", ", data.z, ")");
    }

    // Successfully entered the world (or teleported)
    currentMapId_ = data.mapId;
    setState(WorldState::IN_WORLD);
    if (socket) {
        socket->tracePacketsFor(std::chrono::seconds(12), "login_verify_world");
    }

    LOG_INFO("========================================");
    LOG_INFO("   SUCCESSFULLY ENTERED WORLD!");
    LOG_INFO("========================================");
    LOG_INFO("Map ID: ", data.mapId);
    LOG_INFO("Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("Orientation: ", data.orientation, " radians");
    LOG_INFO("Player is now in the game world");

    // Initialize movement info with world entry position (server → canonical)
    LOG_DEBUG("LOGIN_VERIFY_WORLD: server=(", data.x, ", ", data.y, ", ", data.z,
             ") canonical=(", canonical.x, ", ", canonical.y, ", ", canonical.z, ") mapId=", data.mapId);
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(data.orientation);
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    if (movementHandler_) {
        movementHandler_->resetMovementClock();
    }
    movementInfo.time = nextMovementTimestampMs();
    if (movementHandler_) {
        movementHandler_->setFalling(false);
        movementHandler_->setFallStartMs(0);
    }
    movementInfo.fallTime = 0;
    movementInfo.jumpVelocity = 0.0f;
    movementInfo.jumpSinAngle = 0.0f;
    movementInfo.jumpCosAngle = 0.0f;
    movementInfo.jumpXYSpeed = 0.0f;
    resurrectPending_ = false;
    resurrectRequestPending_ = false;
    selfResAvailable_ = false;
    onTaxiFlight_ = false;
    taxiMountActive_ = false;
    taxiActivatePending_ = false;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    // taxiRecoverPending_ is NOT cleared here — it must survive the general
    // state reset so the recovery check below can detect a mid-flight reconnect.
    taxiStartGrace_ = 0.0f;
    currentMountDisplayId_ = 0;
    taxiMountDisplayId_ = 0;
    vehicleId_ = 0;
    if (mountCallback_) {
        mountCallback_(0);
    }

    // Clear boss encounter unit slots and raid marks on world transfer
    if (socialHandler_) socialHandler_->resetTransferState();

    // Suppress area triggers on initial login — prevents exit portals from
    // immediately firing when spawning inside a dungeon/instance. Deeprun Tram
    // (map 369) needs a shorter window because exits are close to the spawn.
    const bool deeprunTram = data.mapId == 369;
    activeAreaTriggers_.clear();
    areaTriggerCheckTimer_ = deeprunTram ? -1.0f : -5.0f;
    areaTriggerSuppressFirst_ = true;
    areaTriggerCooldown_ = deeprunTram ? 1.5f : 10.0f;

    // Notify application to load terrain for this map/position (online mode)
    if (worldEntryCallback_) {
        worldEntryCallback_(data.mapId, data.x, data.y, data.z, initialWorldEntry);
    }

    // Send CMSG_SET_ACTIVE_MOVER on initial world entry and world transfers.
    if (playerGuid != 0 && socket) {
        auto activeMoverPacket = SetActiveMoverPacket::build(playerGuid);
        socket->send(activeMoverPacket);
        LOG_INFO("Sent CMSG_SET_ACTIVE_MOVER for player 0x", std::hex, playerGuid, std::dec);
    }

    // Immediately sync our position so the server doesn't keep stale coordinates
    // from a previous session or character. Without this, the server can think
    // the player is elsewhere and issue a corrective teleport.
    if (socket) {
        sendMovement(game::Opcode::MSG_MOVE_STOP);
        sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
    }

    // Kick the first keepalive immediately on world entry. Classic-like realms
    // can close the session before our default 30s ping cadence fires.
    timeSinceLastPing = 0.0f;
    if (socket) {
        LOG_DEBUG("World entry keepalive: sending immediate ping after LOGIN_VERIFY_WORLD");
        sendPing();
    }

    // If we disconnected mid-taxi, attempt to recover to destination after login.
    if (taxiRecoverPending_ && taxiRecoverMapId_ == data.mapId) {
        float dx = movementInfo.x - taxiRecoverPos_.x;
        float dy = movementInfo.y - taxiRecoverPos_.y;
        float dz = movementInfo.z - taxiRecoverPos_.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist > 5.0f) {
            // Keep pending until player entity exists; update() will apply.
            LOG_INFO("Taxi recovery pending: dist=", dist);
        } else {
            taxiRecoverPending_ = false;
        }
    }

    if (initialWorldEntry) {
        // Clear inspect caches on world entry to avoid showing stale data.
        inspectedPlayerAchievements_.clear();

        // Buyback mirror persists across vendor windows within a session but
        // must not leak into another character's session.
        if (inventoryHandler_) inventoryHandler_->clearBuybackState();

        // Reset talent initialization so the first SMSG_TALENTS_INFO after login
        // correctly sets the active spec (static locals don't reset across logins).
        if (spellHandler_) spellHandler_->resetTalentState();

        // Auto-join default chat channels only on first world entry.
        autoJoinDefaultChannels();

        // Auto-query guild info on login.
        const Character* activeChar = getActiveCharacter();
        if (activeChar && activeChar->hasGuild() && socket) {
            auto gqPacket = GuildQueryPacket::build(activeChar->guildId);
            socket->send(gqPacket);
            auto grPacket = GuildRosterPacket::build();
            socket->send(grPacket);
            LOG_INFO("Auto-queried guild info (guildId=", activeChar->guildId, ")");
        }

        if (questHandler_) {
            questHandler_->pendingQuestAcceptTimeoutsRef().clear();
            questHandler_->pendingQuestAcceptNpcGuidsRef().clear();
        }
        pendingQuestQueryIds_.clear();
        pendingLoginQuestResync_ = true;
        pendingLoginQuestResyncTimeout_ = 10.0f;
        completedQuests_.clear();
        LOG_INFO("Queued quest log resync for login (from server quest slots)");

        // Request completed quest IDs when the expansion supports it. Classic-like
        // opcode tables do not define this packet, and sending 0xFFFF during world
        // entry can desync the early session handshake.
        if (socket) {
            const uint16_t queryCompletedWire = wireOpcode(Opcode::CMSG_QUERY_QUESTS_COMPLETED);
            if (queryCompletedWire != 0xFFFF) {
                network::Packet cqcPkt(queryCompletedWire);
                socket->send(cqcPkt);
                LOG_INFO("Sent CMSG_QUERY_QUESTS_COMPLETED");
            } else {
                LOG_INFO("Skipping CMSG_QUERY_QUESTS_COMPLETED: opcode not mapped for current expansion");
            }
        }

        // Auto-request played time on login so the character Stats tab is
        // populated immediately without requiring /played.
        if (socket) {
            auto ptPkt = RequestPlayedTimePacket::build(false);  // false = don't show in chat
            socket->send(ptPkt);
            LOG_INFO("Auto-requested played time on login");
        }
    }

    // Pre-load DBC name caches during world entry so the first packet that
    // needs spell/title/achievement data doesn't stall mid-gameplay (the
    // Spell.dbc cache alone is ~170ms on a cold load).
    if (initialWorldEntry) {
        preloadDBCCaches();
    }

    // Fire PLAYER_ENTERING_WORLD — THE most important event for addon initialization.
    // Fires on initial login, teleports, instance transitions, and zone changes.
    if (addonEventCallback_) {
        fireAddonEvent("PLAYER_ENTERING_WORLD", {initialWorldEntry ? "1" : "0"});
        // Also fire ZONE_CHANGED_NEW_AREA and UPDATE_WORLD_STATES so map/BG addons refresh
        fireAddonEvent("ZONE_CHANGED_NEW_AREA", {});
        fireAddonEvent("UPDATE_WORLD_STATES", {});
        // PLAYER_LOGIN fires only on initial login (not teleports)
        if (initialWorldEntry) {
            fireAddonEvent("PLAYER_LOGIN", {});
        }
    }
}

void GameHandler::handleClientCacheVersion(network::Packet& packet) {
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_CLIENTCACHE_VERSION too short: ", packet.getSize(), " bytes");
        return;
    }

    uint32_t version = packet.readUInt32();
    LOG_INFO("SMSG_CLIENTCACHE_VERSION: ", version);
}

void GameHandler::handleTutorialFlags(network::Packet& packet) {
    if (packet.getSize() < 32) {
        LOG_WARNING("SMSG_TUTORIAL_FLAGS too short: ", packet.getSize(), " bytes");
        return;
    }

    std::array<uint32_t, 8> flags{};
    for (uint32_t& v : flags) {
        v = packet.readUInt32();
    }

    LOG_INFO("SMSG_TUTORIAL_FLAGS: [",
             flags[0], ", ", flags[1], ", ", flags[2], ", ", flags[3], ", ",
             flags[4], ", ", flags[5], ", ", flags[6], ", ", flags[7], "]");
}

void GameHandler::handleAccountDataTimes(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_ACCOUNT_DATA_TIMES");

    AccountDataTimesData data;
    if (!AccountDataTimesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACCOUNT_DATA_TIMES");
        return;
    }

    LOG_DEBUG("Account data times received (server time: ", data.serverTime, ")");
}

void GameHandler::handleMotd(network::Packet& packet) {
    if (chatHandler_) chatHandler_->handleMotd(packet);
}

void GameHandler::handleNotification(network::Packet& packet) {
    // SMSG_NOTIFICATION: single null-terminated string
    std::string message = packet.readString();
    if (!message.empty()) {
        LOG_INFO("Server notification: ", message);
        addSystemChatMessage(message);
    }
}

void GameHandler::sendPing() {
    if (state != WorldState::IN_WORLD) {
        return;
    }

    // Increment sequence number
    pingSequence++;

    LOG_DEBUG("Sending CMSG_PING: sequence=", pingSequence,
              " latencyHintMs=", lastLatency);

    // Record send time for RTT measurement
    pingTimestamp_ = std::chrono::steady_clock::now();

    // Build and send ping packet
    auto packet = PingPacket::build(pingSequence, lastLatency);
    socket->send(packet);
}

void GameHandler::sendRequestVehicleExit() {
    if (state != WorldState::IN_WORLD || vehicleId_ == 0) return;
    // CMSG_REQUEST_VEHICLE_EXIT has no payload — opcode only
    network::Packet pkt(wireOpcode(Opcode::CMSG_REQUEST_VEHICLE_EXIT));
    socket->send(pkt);
    vehicleId_ = 0;  // Optimistically clear; server will confirm via SMSG_PLAYER_VEHICLE_DATA(0)
}

const std::vector<GameHandler::EquipmentSetInfo>& GameHandler::getEquipmentSets() const {
    if (inventoryHandler_) return inventoryHandler_->getEquipmentSets();
    static const std::vector<EquipmentSetInfo> empty;
    return empty;
}

// Trade state delegation to InventoryHandler (which owns the canonical trade state)
GameHandler::TradeStatus GameHandler::getTradeStatus() const {
    if (inventoryHandler_) return static_cast<TradeStatus>(inventoryHandler_->getTradeStatus());
    return tradeStatus_;
}
bool GameHandler::hasPendingTradeRequest() const {
    return inventoryHandler_ ? inventoryHandler_->hasPendingTradeRequest() : false;
}
bool GameHandler::isTradeOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isTradeOpen() : false;
}
const std::string& GameHandler::getTradePeerName() const {
    if (inventoryHandler_) return inventoryHandler_->getTradePeerName();
    return tradePeerName_;
}
const std::array<GameHandler::TradeSlot, GameHandler::TRADE_SLOT_COUNT>& GameHandler::getMyTradeSlots() const {
    if (inventoryHandler_) {
        // Convert InventoryHandler::TradeSlot → GameHandler::TradeSlot (different struct layouts)
        static std::array<TradeSlot, TRADE_SLOT_COUNT> converted{};
        const auto& src = inventoryHandler_->getMyTradeSlots();
        for (size_t i = 0; i < TRADE_SLOT_COUNT; i++) {
            converted[i].itemId = src[i].itemId;
            converted[i].displayId = src[i].displayId;
            converted[i].stackCount = src[i].stackCount;
            converted[i].itemGuid = src[i].itemGuid;
            converted[i].occupied = (src[i].itemId != 0);
        }
        return converted;
    }
    return myTradeSlots_;
}
const std::array<GameHandler::TradeSlot, GameHandler::TRADE_SLOT_COUNT>& GameHandler::getPeerTradeSlots() const {
    if (inventoryHandler_) {
        static std::array<TradeSlot, TRADE_SLOT_COUNT> converted{};
        const auto& src = inventoryHandler_->getPeerTradeSlots();
        for (size_t i = 0; i < TRADE_SLOT_COUNT; i++) {
            converted[i].itemId = src[i].itemId;
            converted[i].displayId = src[i].displayId;
            converted[i].stackCount = src[i].stackCount;
            converted[i].itemGuid = src[i].itemGuid;
            converted[i].occupied = (src[i].itemId != 0);
        }
        return converted;
    }
    return peerTradeSlots_;
}
uint64_t GameHandler::getMyTradeGold() const {
    return inventoryHandler_ ? inventoryHandler_->getMyTradeGold() : myTradeGold_;
}
uint64_t GameHandler::getPeerTradeGold() const {
    return inventoryHandler_ ? inventoryHandler_->getPeerTradeGold() : peerTradeGold_;
}

bool GameHandler::supportsEquipmentSets() const {
    return inventoryHandler_ && inventoryHandler_->supportsEquipmentSets();
}

void GameHandler::useEquipmentSet(uint32_t setId) {
    if (inventoryHandler_) inventoryHandler_->useEquipmentSet(setId);
}

void GameHandler::saveEquipmentSet(const std::string& name, const std::string& iconName,
                                    uint64_t existingGuid, uint32_t setIndex) {
    if (inventoryHandler_) inventoryHandler_->saveEquipmentSet(name, iconName, existingGuid, setIndex);
}

void GameHandler::deleteEquipmentSet(uint64_t setGuid) {
    if (inventoryHandler_) inventoryHandler_->deleteEquipmentSet(setGuid);
}

// --- Inventory state delegation (canonical state lives in InventoryHandler) ---

// Item text
bool GameHandler::isItemTextOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isItemTextOpen() : itemTextOpen_;
}
const std::string& GameHandler::getItemText() const {
    if (inventoryHandler_) return inventoryHandler_->getItemText();
    return itemText_;
}
void GameHandler::closeItemText() {
    if (inventoryHandler_) inventoryHandler_->closeItemText();
    else itemTextOpen_ = false;
}

// Loot
bool GameHandler::isLootWindowOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isLootWindowOpen() : lootWindowOpen;
}
const LootResponseData& GameHandler::getCurrentLoot() const {
    if (inventoryHandler_) return inventoryHandler_->getCurrentLoot();
    return currentLoot;
}
void GameHandler::setAutoLoot(bool enabled) {
    if (inventoryHandler_) inventoryHandler_->setAutoLoot(enabled);
    else autoLoot_ = enabled;
}
bool GameHandler::isAutoLoot() const {
    return inventoryHandler_ ? inventoryHandler_->isAutoLoot() : autoLoot_;
}
void GameHandler::setAutoSellGrey(bool enabled) {
    if (inventoryHandler_) inventoryHandler_->setAutoSellGrey(enabled);
    else autoSellGrey_ = enabled;
}
bool GameHandler::isAutoSellGrey() const {
    return inventoryHandler_ ? inventoryHandler_->isAutoSellGrey() : autoSellGrey_;
}
void GameHandler::setAutoRepair(bool enabled) {
    if (inventoryHandler_) inventoryHandler_->setAutoRepair(enabled);
    else autoRepair_ = enabled;
}
bool GameHandler::isAutoRepair() const {
    return inventoryHandler_ ? inventoryHandler_->isAutoRepair() : autoRepair_;
}
const std::vector<uint64_t>& GameHandler::getMasterLootCandidates() const {
    if (inventoryHandler_) return inventoryHandler_->getMasterLootCandidates();
    return masterLootCandidates_;
}
bool GameHandler::hasMasterLootCandidates() const {
    return inventoryHandler_ ? inventoryHandler_->hasMasterLootCandidates() : !masterLootCandidates_.empty();
}
bool GameHandler::hasPendingLootRoll() const {
    return inventoryHandler_ ? inventoryHandler_->hasPendingLootRoll() : pendingLootRollActive_;
}
const LootRollEntry& GameHandler::getPendingLootRoll() const {
    if (inventoryHandler_) return inventoryHandler_->getPendingLootRoll();
    return pendingLootRoll_;
}

// Vendor
bool GameHandler::isVendorWindowOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isVendorWindowOpen() : vendorWindowOpen;
}
const ListInventoryData& GameHandler::getVendorItems() const {
    if (inventoryHandler_) return inventoryHandler_->getVendorItems();
    return currentVendorItems;
}
void GameHandler::setVendorCanRepair(bool v) {
    if (inventoryHandler_) inventoryHandler_->setVendorCanRepair(v);
    else currentVendorItems.canRepair = v;
}
const std::deque<GameHandler::BuybackItem>& GameHandler::getBuybackItems() const {
    if (inventoryHandler_) {
        // Layout-identical structs (InventoryHandler::BuybackItem == GameHandler::BuybackItem)
        return reinterpret_cast<const std::deque<BuybackItem>&>(inventoryHandler_->getBuybackItems());
    }
    return buybackItems_;
}
uint64_t GameHandler::getVendorGuid() const {
    if (inventoryHandler_) return inventoryHandler_->getVendorGuid();
    return currentVendorItems.vendorGuid;
}

// Mail
bool GameHandler::isMailboxOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isMailboxOpen() : mailboxOpen_;
}
const std::vector<MailMessage>& GameHandler::getMailInbox() const {
    if (inventoryHandler_) return inventoryHandler_->getMailInbox();
    return mailInbox_;
}
std::string GameHandler::getMailDisplaySubject(const MailMessage& mail) {
    if (mail.messageType != 2) return mail.subject;

    AuctionMailSubject auction;
    if (!parseAuctionMailSubject(mail.subject, auction)) return mail.subject;

    ensureItemInfo(auction.itemEntry);
    const auto* info = getItemInfo(auction.itemEntry);
    const std::string itemName = info && !info->name.empty()
        ? info->name
        : "Item #" + std::to_string(auction.itemEntry);
    return formatAuctionMailSubject(auction, itemName);
}
int GameHandler::getSelectedMailIndex() const {
    return inventoryHandler_ ? inventoryHandler_->getSelectedMailIndex() : selectedMailIndex_;
}
void GameHandler::setSelectedMailIndex(int idx) {
    if (inventoryHandler_) inventoryHandler_->setSelectedMailIndex(idx);
    else selectedMailIndex_ = idx;
}
bool GameHandler::isMailComposeOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isMailComposeOpen() : showMailCompose_;
}
void GameHandler::openMailCompose() {
    if (inventoryHandler_) inventoryHandler_->openMailCompose();
    else { showMailCompose_ = true; clearMailAttachments(); }
}
void GameHandler::closeMailCompose() {
    if (inventoryHandler_) inventoryHandler_->closeMailCompose();
    else { showMailCompose_ = false; clearMailAttachments(); }
}
bool GameHandler::hasNewMail() const {
    return inventoryHandler_ ? inventoryHandler_->hasNewMail() : hasNewMail_;
}
const std::array<GameHandler::MailAttachSlot, 12>& GameHandler::getMailAttachments() const {
    if (inventoryHandler_) {
        // Layout-identical structs (InventoryHandler::MailAttachSlot == GameHandler::MailAttachSlot)
        return reinterpret_cast<const std::array<MailAttachSlot, 12>&>(inventoryHandler_->getMailAttachments());
    }
    return mailAttachments_;
}

int GameHandler::getMaxMailAttachments() const {
    return InventoryHandler::maxSendableMailAttachments();
}

// Bank
bool GameHandler::isBankOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isBankOpen() : bankOpen_;
}
uint64_t GameHandler::getBankerGuid() const {
    return inventoryHandler_ ? inventoryHandler_->getBankerGuid() : bankerGuid_;
}
int GameHandler::getEffectiveBankSlots() const {
    return inventoryHandler_ ? inventoryHandler_->getEffectiveBankSlots() : effectiveBankSlots_;
}
int GameHandler::getEffectiveBankBagSlots() const {
    return inventoryHandler_ ? inventoryHandler_->getEffectiveBankBagSlots() : effectiveBankBagSlots_;
}

// Guild Bank
bool GameHandler::isGuildBankOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isGuildBankOpen() : guildBankOpen_;
}
const GuildBankData& GameHandler::getGuildBankData() const {
    if (inventoryHandler_) return inventoryHandler_->getGuildBankData();
    return guildBankData_;
}
uint8_t GameHandler::getGuildBankActiveTab() const {
    return inventoryHandler_ ? inventoryHandler_->getGuildBankActiveTab() : guildBankActiveTab_;
}
void GameHandler::setGuildBankActiveTab(uint8_t tab) {
    if (inventoryHandler_) inventoryHandler_->setGuildBankActiveTab(tab);
    else guildBankActiveTab_ = tab;
}

// Auction House
bool GameHandler::isAuctionHouseOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isAuctionHouseOpen() : auctionOpen_;
}
uint64_t GameHandler::getAuctioneerGuid() const {
    return inventoryHandler_ ? inventoryHandler_->getAuctioneerGuid() : auctioneerGuid_;
}
const AuctionListResult& GameHandler::getAuctionBrowseResults() const {
    if (inventoryHandler_) return inventoryHandler_->getAuctionBrowseResults();
    return auctionBrowseResults_;
}
const AuctionListResult& GameHandler::getAuctionOwnerResults() const {
    if (inventoryHandler_) return inventoryHandler_->getAuctionOwnerResults();
    return auctionOwnerResults_;
}
const AuctionListResult& GameHandler::getAuctionBidderResults() const {
    if (inventoryHandler_) return inventoryHandler_->getAuctionBidderResults();
    return auctionBidderResults_;
}
int GameHandler::getAuctionActiveTab() const {
    return inventoryHandler_ ? inventoryHandler_->getAuctionActiveTab() : auctionActiveTab_;
}
void GameHandler::setAuctionActiveTab(int tab) {
    if (inventoryHandler_) inventoryHandler_->setAuctionActiveTab(tab);
    else auctionActiveTab_ = tab;
}
float GameHandler::getAuctionSearchDelay() const {
    return inventoryHandler_ ? inventoryHandler_->getAuctionSearchDelay() : 0.0f;
}

// Trainer
bool GameHandler::isTrainerWindowOpen() const {
    return inventoryHandler_ ? inventoryHandler_->isTrainerWindowOpen() : trainerWindowOpen_;
}
const TrainerListData& GameHandler::getTrainerSpells() const {
    if (inventoryHandler_) return inventoryHandler_->getTrainerSpells();
    return currentTrainerList_;
}
const std::vector<GameHandler::TrainerTab>& GameHandler::getTrainerTabs() const {
    if (inventoryHandler_) {
        // Layout-identical structs (InventoryHandler::TrainerTab == GameHandler::TrainerTab)
        return reinterpret_cast<const std::vector<TrainerTab>&>(inventoryHandler_->getTrainerTabs());
    }
    return trainerTabs_;
}

void GameHandler::sendMinimapPing(float wowX, float wowY) {
    if (socialHandler_) socialHandler_->sendMinimapPing(wowX, wowY);
}

void GameHandler::handlePong(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_PONG");

    PongData data;
    if (!PongParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_PONG");
        return;
    }

    // Verify sequence matches
    if (data.sequence != pingSequence) {
        LOG_WARNING("SMSG_PONG sequence mismatch: expected ", pingSequence,
                    ", got ", data.sequence);
        return;
    }

    // Measure round-trip time
    auto rtt = std::chrono::steady_clock::now() - pingTimestamp_;
    lastLatency = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(rtt).count());

    LOG_DEBUG("SMSG_PONG acknowledged: sequence=", data.sequence,
              " latencyMs=", lastLatency);
}

bool GameHandler::isServerMovementAllowed() const {
    return movementHandler_ ? movementHandler_->isServerMovementAllowed() : true;
}

uint32_t GameHandler::nextMovementTimestampMs() {
    if (movementHandler_) return movementHandler_->nextMovementTimestampMs();
    return 0;
}

void GameHandler::sendMovement(Opcode opcode) {
    if (movementHandler_) movementHandler_->sendMovement(opcode);
}

void GameHandler::sanitizeMovementForTaxi() {
    if (movementHandler_) movementHandler_->sanitizeMovementForTaxi();
}

void GameHandler::forceClearTaxiAndMovementState() {
    if (movementHandler_) movementHandler_->forceClearTaxiAndMovementState();
}

void GameHandler::setPosition(float x, float y, float z) {
    if (movementHandler_) movementHandler_->setPosition(x, y, z);
}

void GameHandler::setOrientation(float orientation) {
    if (movementHandler_) movementHandler_->setOrientation(orientation);
}

// Entity lifecycle methods (handleUpdateObject, processOutOfRangeObjects,
// applyUpdateObjectBlock, finalizeUpdateObjectBatch, handleCompressedUpdateObject,
// handleDestroyObject) moved to EntityController — see entity_controller.cpp

void GameHandler::sendChatMessage(ChatType type, const std::string& message, const std::string& target) {
    if (chatHandler_) chatHandler_->sendChatMessage(type, message, target);
}

void GameHandler::sendAddonMessage(ChatType type, const std::string& message, const std::string& target) {
    if (chatHandler_) chatHandler_->sendAddonMessage(type, message, target);
}

void GameHandler::sendTextEmote(uint32_t textEmoteId, uint64_t targetGuid) {
    if (chatHandler_) chatHandler_->sendTextEmote(textEmoteId, targetGuid);
}

void GameHandler::joinChannel(const std::string& channelName, const std::string& password) {
    if (chatHandler_) chatHandler_->joinChannel(channelName, password);
}

void GameHandler::leaveChannel(const std::string& channelName) {
    if (chatHandler_) chatHandler_->leaveChannel(channelName);
}

std::string GameHandler::getChannelByIndex(int index) const {
    return chatHandler_ ? chatHandler_->getChannelByIndex(index) : "";
}

int GameHandler::getChannelIndex(const std::string& channelName) const {
    return chatHandler_ ? chatHandler_->getChannelIndex(channelName) : 0;
}

void GameHandler::autoJoinDefaultChannels() {
    if (chatHandler_) {
        chatHandler_->chatAutoJoin.general = chatAutoJoin.general;
        chatHandler_->chatAutoJoin.trade = chatAutoJoin.trade;
        chatHandler_->chatAutoJoin.localDefense = chatAutoJoin.localDefense;
        chatHandler_->chatAutoJoin.lfg = chatAutoJoin.lfg;
        chatHandler_->chatAutoJoin.local = chatAutoJoin.local;
        chatHandler_->autoJoinDefaultChannels();
    }
}

void GameHandler::setTarget(uint64_t guid) {
    if (combatHandler_) combatHandler_->setTarget(guid);
}

void GameHandler::clearTarget() {
    if (combatHandler_) combatHandler_->clearTarget();
}

std::shared_ptr<Entity> GameHandler::getTarget() const {
    return combatHandler_ ? combatHandler_->getTarget() : nullptr;
}

void GameHandler::setFocus(uint64_t guid) {
    if (combatHandler_) combatHandler_->setFocus(guid);
}

void GameHandler::clearFocus() {
    if (combatHandler_) combatHandler_->clearFocus();
}

void GameHandler::setMouseoverGuid(uint64_t guid) {
    if (combatHandler_) combatHandler_->setMouseoverGuid(guid);
}

std::shared_ptr<Entity> GameHandler::getFocus() const {
    return combatHandler_ ? combatHandler_->getFocus() : nullptr;
}

void GameHandler::targetLastTarget() {
    if (combatHandler_) combatHandler_->targetLastTarget();
}

void GameHandler::targetEnemy(bool reverse) {
    if (combatHandler_) combatHandler_->targetEnemy(reverse);
}

void GameHandler::targetFriend(bool reverse) {
    if (combatHandler_) combatHandler_->targetFriend(reverse);
}

void GameHandler::inspectTarget() {
    if (socialHandler_) socialHandler_->inspectTarget();
}

const GameHandler::InspectResult* GameHandler::getInspectResult() const {
    return socialHandler_ ? socialHandler_->getInspectResult() : nullptr;
}

void GameHandler::queryServerTime() {
    if (socialHandler_) socialHandler_->queryServerTime();
}

void GameHandler::requestPlayedTime() {
    if (socialHandler_) socialHandler_->requestPlayedTime();
}

void GameHandler::queryWho(const std::string& playerName) {
    if (socialHandler_) socialHandler_->queryWho(playerName);
}

void GameHandler::addFriend(const std::string& playerName, const std::string& note) {
    if (socialHandler_) socialHandler_->addFriend(playerName, note);
}

void GameHandler::removeFriend(const std::string& playerName) {
    if (socialHandler_) socialHandler_->removeFriend(playerName);
}

void GameHandler::setFriendNote(const std::string& playerName, const std::string& note) {
    if (socialHandler_) socialHandler_->setFriendNote(playerName, note);
}

void GameHandler::randomRoll(uint32_t minRoll, uint32_t maxRoll) {
    if (socialHandler_) socialHandler_->randomRoll(minRoll, maxRoll);
}

void GameHandler::addIgnore(const std::string& playerName) {
    if (socialHandler_) socialHandler_->addIgnore(playerName);
}

void GameHandler::removeIgnore(const std::string& playerName) {
    if (socialHandler_) socialHandler_->removeIgnore(playerName);
}

void GameHandler::faceCanonicalYaw(float canonicalYaw) {
    movementInfo.orientation = canonicalYaw;

    // One-shot check that the renderer's facing round-trips to the same
    // canonical yaw we just computed from positions. If it does not, the
    // per-frame resync replaces this with a different heading and the server
    // re-checks the arc against that instead.
    if (auto* r = services_.renderer) {
        const float fromRender = core::coords::characterYawDegToCanonical(r->getCharacterYaw());
        const float delta = core::coords::normalizeAngleRad(fromRender - canonicalYaw);
        if (std::abs(delta) > 0.2f) {
            LOG_WARNING("Facing mismatch: computed canonical=", canonicalYaw,
                        " but the character's ", r->getCharacterYaw(),
                        " deg maps to ", fromRender, " (off by ",
                        delta * 57.2957795f, " deg) — the resync will undo this");
        }
    }

    // The renderer owns facing; the game side is downstream of it every frame.
    if (auto* renderer = services_.renderer) {
        const float facingDeg = core::coords::canonicalToCharacterYawDeg(canonicalYaw);
        renderer->setCharacterYaw(facingDeg);
        if (auto* cc = renderer->getCameraController()) {
            cc->setFacingYaw(facingDeg);
        }
    }

    sendMovement(Opcode::MSG_MOVE_SET_FACING);
}

void GameHandler::requestLogout(bool exitAfterLogout) {
    if (socialHandler_) socialHandler_->requestLogout(exitAfterLogout);
}

void GameHandler::cancelLogout() {
    if (socialHandler_) socialHandler_->cancelLogout();
}

void GameHandler::sendSetDifficulty(uint32_t difficulty) {
    if (socialHandler_) socialHandler_->sendSetDifficulty(difficulty);
}

void GameHandler::setStandState(uint8_t standState) {
    if (socialHandler_) socialHandler_->setStandState(standState);
}

void GameHandler::toggleHelm() {
    if (socialHandler_) socialHandler_->toggleHelm();
}

void GameHandler::toggleCloak() {
    if (socialHandler_) socialHandler_->toggleCloak();
}

void GameHandler::followTarget() {
    if (movementHandler_) movementHandler_->followTarget();
}

void GameHandler::cancelFollow() {
    if (movementHandler_) movementHandler_->cancelFollow();
}

void GameHandler::assistTarget() {
    if (combatHandler_) combatHandler_->assistTarget();
}

void GameHandler::togglePvp() {
    if (combatHandler_) combatHandler_->togglePvp();
}

void GameHandler::requestGuildInfo() {
    if (socialHandler_) socialHandler_->requestGuildInfo();
}

void GameHandler::requestGuildRoster() {
    if (socialHandler_) socialHandler_->requestGuildRoster();
}

void GameHandler::setGuildMotd(const std::string& motd) {
    if (socialHandler_) socialHandler_->setGuildMotd(motd);
}

void GameHandler::promoteGuildMember(const std::string& playerName) {
    if (socialHandler_) socialHandler_->promoteGuildMember(playerName);
}

void GameHandler::demoteGuildMember(const std::string& playerName) {
    if (socialHandler_) socialHandler_->demoteGuildMember(playerName);
}

void GameHandler::leaveGuild() {
    if (socialHandler_) socialHandler_->leaveGuild();
}

void GameHandler::inviteToGuild(const std::string& playerName) {
    if (socialHandler_) socialHandler_->inviteToGuild(playerName);
}

void GameHandler::initiateReadyCheck() {
    if (socialHandler_) socialHandler_->initiateReadyCheck();
}

void GameHandler::respondToReadyCheck(bool ready) {
    if (socialHandler_) socialHandler_->respondToReadyCheck(ready);
}

void GameHandler::acceptDuel() {
    if (socialHandler_) socialHandler_->acceptDuel();
}

void GameHandler::forfeitDuel() {
    if (socialHandler_) socialHandler_->forfeitDuel();
}

void GameHandler::toggleAfk(const std::string& message) {
    if (chatHandler_) chatHandler_->toggleAfk(message);
}

void GameHandler::toggleDnd(const std::string& message) {
    if (chatHandler_) chatHandler_->toggleDnd(message);
}

void GameHandler::replyToLastWhisper(const std::string& message) {
    if (chatHandler_) chatHandler_->replyToLastWhisper(message);
}

void GameHandler::uninvitePlayer(const std::string& playerName) {
    if (socialHandler_) socialHandler_->uninvitePlayer(playerName);
}

void GameHandler::leaveParty() {
    if (socialHandler_) socialHandler_->leaveParty();
}

void GameHandler::setMainTank(uint64_t targetGuid) {
    if (socialHandler_) socialHandler_->setMainTank(targetGuid);
}

void GameHandler::setMainAssist(uint64_t targetGuid) {
    if (socialHandler_) socialHandler_->setMainAssist(targetGuid);
}

void GameHandler::clearMainTank() {
    if (socialHandler_) socialHandler_->clearMainTank();
}

void GameHandler::clearMainAssist() {
    if (socialHandler_) socialHandler_->clearMainAssist();
}

void GameHandler::setRaidMark(uint64_t guid, uint8_t icon) {
    if (socialHandler_) socialHandler_->setRaidMark(guid, icon);
}

// The marks live in SocialHandler — MSG_RAID_TARGET_UPDATE writes them there.
// GameHandler kept a second, identical array that nothing ever wrote, so every
// caller of these (target frame, nameplates, minimap, social panel) read an
// array of zeros and no marker icon was ever drawn.
uint64_t GameHandler::getRaidMarkGuid(uint32_t icon) const {
    return socialHandler_ ? socialHandler_->getRaidMarkGuid(icon) : 0;
}

uint8_t GameHandler::getEntityRaidMark(uint64_t guid) const {
    return socialHandler_ ? socialHandler_->getEntityRaidMark(guid) : 0xFF;
}

void GameHandler::requestRaidInfo() {
    if (socialHandler_) socialHandler_->requestRaidInfo();
}

void GameHandler::proposeDuel(uint64_t targetGuid) {
    if (socialHandler_) socialHandler_->proposeDuel(targetGuid);
}

void GameHandler::initiateTrade(uint64_t targetGuid) {
    if (inventoryHandler_) inventoryHandler_->initiateTrade(targetGuid);
}

void GameHandler::reportPlayer(uint64_t targetGuid, const std::string& reason) {
    if (socialHandler_) socialHandler_->reportPlayer(targetGuid, reason);
}

void GameHandler::stopCasting() {
    if (spellHandler_) spellHandler_->stopCasting();
}

void GameHandler::resetCastState() {
    if (spellHandler_) spellHandler_->resetCastState();
}

void GameHandler::clearUnitCaches() {
    if (spellHandler_) spellHandler_->clearUnitCaches();
}

void GameHandler::releaseSpirit() {
    if (combatHandler_) combatHandler_->releaseSpirit();
}

bool GameHandler::canReclaimCorpse() const {
    return combatHandler_ ? combatHandler_->canReclaimCorpse() : false;
}

float GameHandler::getCorpseReclaimDelaySec() const {
    return combatHandler_ ? combatHandler_->getCorpseReclaimDelaySec() : 0.0f;
}

void GameHandler::reclaimCorpse() {
    if (combatHandler_) combatHandler_->reclaimCorpse();
}

void GameHandler::useSelfRes() {
    if (combatHandler_) combatHandler_->useSelfRes();
}

void GameHandler::activateSpiritHealer(uint64_t npcGuid) {
    if (combatHandler_) combatHandler_->activateSpiritHealer(npcGuid);
}

void GameHandler::acceptResurrect() {
    if (combatHandler_) combatHandler_->acceptResurrect();
}

void GameHandler::declineResurrect() {
    if (combatHandler_) combatHandler_->declineResurrect();
}

void GameHandler::tabTarget(float playerX, float playerY, float playerZ) {
    if (combatHandler_) combatHandler_->tabTarget(playerX, playerY, playerZ);
}

void GameHandler::addLocalChatMessage(const MessageChatData& msg) {
    if (chatHandler_) chatHandler_->addLocalChatMessage(msg);
}

const std::deque<MessageChatData>& GameHandler::getChatHistory() const {
    if (chatHandler_) return chatHandler_->getChatHistory();
    static const std::deque<MessageChatData> kEmpty;
    return kEmpty;
}

void GameHandler::clearChatHistory() {
    if (chatHandler_) chatHandler_->getChatHistory().clear();
}

const std::vector<std::string>& GameHandler::getJoinedChannels() const {
    if (chatHandler_) return chatHandler_->getJoinedChannels();
    static const std::vector<std::string> kEmpty;
    return kEmpty;
}

// ============================================================
// Name Queries (delegated to EntityController)
// ============================================================

// Who a piece of mail is from. The wire gives a player GUID for player mail and
// an entry for everything else, and nothing resolved either — the inbox showed
// an empty From line for every message.
//
// Resolved on demand rather than stored, so a name that arrives after the inbox
// was parsed shows up on the next frame instead of staying blank.
std::string GameHandler::getMailSenderName(const MailMessage& mail) const {
    switch (mail.messageType) {
        case 0: {  // from another player
            // The name-query response backfills senderName, and Reply uses that
            // field, so prefer it and fall back to the cache for the window
            // between the inbox arriving and the query returning.
            if (!mail.senderName.empty()) return mail.senderName;
            if (mail.senderGuid == 0) break;
            const std::string& name = lookupName(mail.senderGuid);
            if (!name.empty()) return name;
            return "Unknown";
        }
        case 2:    // the auction house, which has no creature behind it
            return "Auction House";
        case 3: {  // a creature
            std::string name = getCachedCreatureName(mail.senderEntry);
            if (!name.empty()) return name;
            break;
        }
        case 4: {  // a game object
            if (const auto* info = getCachedGameObjectInfo(mail.senderEntry);
                info && !info->name.empty()) {
                return info->name;
            }
            break;
        }
        default:
            break;
    }
    return "Unknown";
}

void GameHandler::queryPlayerName(uint64_t guid) {
    if (entityController_) entityController_->queryPlayerName(guid);
}

void GameHandler::queryCreatureInfo(uint32_t entry, uint64_t guid) {
    if (entityController_) entityController_->queryCreatureInfo(entry, guid);
}

void GameHandler::queryGameObjectInfo(uint32_t entry, uint64_t guid) {
    if (entityController_) entityController_->queryGameObjectInfo(entry, guid);
}

std::string GameHandler::getCachedPlayerName(uint64_t guid) const {
    return entityController_ ? entityController_->getCachedPlayerName(guid) : "";
}

std::string GameHandler::getCachedCreatureName(uint32_t entry) const {
    return entityController_ ? entityController_->getCachedCreatureName(entry) : "";
}

// ============================================================
// Item Query (forwarded to InventoryHandler)
// ============================================================

void GameHandler::queryItemInfo(uint32_t entry, uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->queryItemInfo(entry, guid);
}

void GameHandler::handleItemQueryResponse(network::Packet& packet) {
    if (inventoryHandler_) inventoryHandler_->handleItemQueryResponse(packet);
}

uint64_t GameHandler::resolveOnlineItemGuid(uint32_t itemId) const {
    return inventoryHandler_ ? inventoryHandler_->resolveOnlineItemGuid(itemId) : 0;
}

void GameHandler::detectInventorySlotBases(const FlatFieldMap& fields) {
    if (inventoryHandler_) inventoryHandler_->detectInventorySlotBases(fields);
}

bool GameHandler::applyInventoryFields(const FlatFieldMap& fields) {
    return inventoryHandler_ ? inventoryHandler_->applyInventoryFields(fields) : false;
}

void GameHandler::extractContainerFields(uint64_t containerGuid, const FlatFieldMap& fields) {
    if (inventoryHandler_) inventoryHandler_->extractContainerFields(containerGuid, fields);
}

void GameHandler::rebuildOnlineInventory() {
    if (inventoryHandler_) inventoryHandler_->rebuildOnlineInventory();
}

void GameHandler::maybeDetectVisibleItemLayout() {
    if (inventoryHandler_) inventoryHandler_->maybeDetectVisibleItemLayout();
}

void GameHandler::updateOtherPlayerVisibleItems(uint64_t guid, const FlatFieldMap& fields) {
    if (inventoryHandler_) inventoryHandler_->updateOtherPlayerVisibleItems(guid, fields);
}

void GameHandler::cacheInspectedPlayerEquipment(uint64_t guid, const std::array<uint32_t, 19>& itemEntries) {
    if (inventoryHandler_) inventoryHandler_->cacheInspectedPlayerEquipment(guid, itemEntries);
}

void GameHandler::emitOtherPlayerEquipment(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->emitOtherPlayerEquipment(guid);
}

void GameHandler::emitAllOtherPlayerEquipment() {
    if (inventoryHandler_) inventoryHandler_->emitAllOtherPlayerEquipment();
}

// ============================================================
// Combat (delegated to CombatHandler)
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    if (combatHandler_) combatHandler_->startAutoAttack(targetGuid);
}

void GameHandler::stopAutoAttack() {
    if (combatHandler_) combatHandler_->stopAutoAttack();
}

void GameHandler::addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId, bool isPlayerSource, uint8_t powerType,
                                uint64_t srcGuid, uint64_t dstGuid) {
    if (combatHandler_) combatHandler_->addCombatText(type, amount, spellId, isPlayerSource, powerType, srcGuid, dstGuid);
}

bool GameHandler::shouldLogSpellstealAura(uint64_t casterGuid, uint64_t victimGuid, uint32_t spellId) {
    return combatHandler_ ? combatHandler_->shouldLogSpellstealAura(casterGuid, victimGuid, spellId) : false;
}

void GameHandler::updateCombatText(float deltaTime) {
    if (combatHandler_) combatHandler_->updateCombatText(deltaTime);
}

bool GameHandler::isAutoAttacking() const {
    return combatHandler_ ? combatHandler_->isAutoAttacking() : false;
}

bool GameHandler::hasAutoAttackIntent() const {
    return combatHandler_ ? combatHandler_->hasAutoAttackIntent() : false;
}

bool GameHandler::isInCombat() const {
    return combatHandler_ ? combatHandler_->isInCombat() : false;
}

bool GameHandler::isInCombatWith(uint64_t guid) const {
    return combatHandler_ ? combatHandler_->isInCombatWith(guid) : false;
}

uint64_t GameHandler::getAutoAttackTargetGuid() const {
    return combatHandler_ ? combatHandler_->getAutoAttackTargetGuid() : 0;
}

bool GameHandler::isAggressiveTowardPlayer(uint64_t guid) const {
    return combatHandler_ ? combatHandler_->isAggressiveTowardPlayer(guid) : false;
}

uint64_t GameHandler::getLastMeleeSwingMs() const {
    return combatHandler_ ? combatHandler_->getLastMeleeSwingMs() : 0;
}

const std::vector<CombatTextEntry>& GameHandler::getCombatText() const {
    static const std::vector<CombatTextEntry> empty;
    return combatHandler_ ? combatHandler_->getCombatText() : empty;
}

const std::deque<CombatLogEntry>& GameHandler::getCombatLog() const {
    static const std::deque<CombatLogEntry> empty;
    return combatHandler_ ? combatHandler_->getCombatLog() : empty;
}

void GameHandler::clearCombatLog() {
    if (combatHandler_) combatHandler_->clearCombatLog();
}

void GameHandler::clearCombatText() {
    if (combatHandler_) combatHandler_->clearCombatText();
}

void GameHandler::clearHostileAttackers() {
    if (combatHandler_) combatHandler_->clearHostileAttackers();
}

const std::vector<GameHandler::ThreatEntry>* GameHandler::getThreatList(uint64_t unitGuid) const {
    return combatHandler_ ? combatHandler_->getThreatList(unitGuid) : nullptr;
}

const std::vector<GameHandler::ThreatEntry>* GameHandler::getTargetThreatList() const {
    return targetGuid ? getThreatList(targetGuid) : nullptr;
}

bool GameHandler::isHostileAttacker(uint64_t guid) const {
    return combatHandler_ ? combatHandler_->isHostileAttacker(guid) : false;
}

void GameHandler::dismount() {
    if (movementHandler_) movementHandler_->dismount();
}

// ============================================================
// Arena / Battleground Handlers
// ============================================================

void GameHandler::declineBattlefield(uint32_t queueSlot) {
    if (socialHandler_) socialHandler_->declineBattlefield(queueSlot);
}

bool GameHandler::hasPendingBgInvite() const {
    return socialHandler_ && socialHandler_->hasPendingBgInvite();
}

void GameHandler::acceptBattlefield(uint32_t queueSlot) {
    if (socialHandler_) socialHandler_->acceptBattlefield(queueSlot);
}

// ---------------------------------------------------------------------------
// LFG / Dungeon Finder handlers (WotLK 3.3.5a)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LFG outgoing packets
// ---------------------------------------------------------------------------

void GameHandler::lfgJoin(uint32_t dungeonId, uint8_t roles) {
    if (socialHandler_) socialHandler_->lfgJoin(dungeonId, roles);
}

void GameHandler::lfgLeave() {
    if (socialHandler_) socialHandler_->lfgLeave();
}

void GameHandler::lfgSetRoles(uint8_t roles) {
    if (socialHandler_) socialHandler_->lfgSetRoles(roles);
}

void GameHandler::lfgAcceptProposal(uint32_t proposalId, bool accept) {
    if (socialHandler_) socialHandler_->lfgAcceptProposal(proposalId, accept);
}

void GameHandler::lfgTeleport(bool toLfgDungeon) {
    if (socialHandler_) socialHandler_->lfgTeleport(toLfgDungeon);
}

void GameHandler::lfgSetBootVote(bool vote) {
    if (socialHandler_) socialHandler_->lfgSetBootVote(vote);
}

void GameHandler::loadAreaTriggerDbc() {
    if (movementHandler_) movementHandler_->loadAreaTriggerDbc();
}

void GameHandler::checkAreaTriggers() {
    if (movementHandler_) movementHandler_->checkAreaTriggers();
}

void GameHandler::requestArenaTeamRoster(uint32_t teamId) {
    if (socialHandler_) socialHandler_->requestArenaTeamRoster(teamId);
}

void GameHandler::requestPvpLog() {
    if (socialHandler_) socialHandler_->requestPvpLog();
}

// ============================================================
// Spells
// ============================================================

void GameHandler::castSpell(uint32_t spellId, uint64_t targetGuid) {
    if (spellHandler_) spellHandler_->castSpell(spellId, targetGuid);
}

void GameHandler::cancelCast() {
    if (spellHandler_) spellHandler_->cancelCast();
}

void GameHandler::startCraftQueue(uint32_t spellId, int count) {
    if (spellHandler_) spellHandler_->startCraftQueue(spellId, count);
}

void GameHandler::cancelCraftQueue() {
    if (spellHandler_) spellHandler_->cancelCraftQueue();
}

void GameHandler::cancelAura(uint32_t spellId) {
    if (spellHandler_) spellHandler_->cancelAura(spellId);
}

uint32_t GameHandler::getTempEnchantRemainingMs(uint32_t slot) const {
    return inventoryHandler_ ? inventoryHandler_->getTempEnchantRemainingMs(slot) : 0u;
}

void GameHandler::handlePetSpells(network::Packet& packet) {
    if (spellHandler_) spellHandler_->handlePetSpells(packet);
}

void GameHandler::sendPetAction(uint32_t action, uint64_t targetGuid) {
    if (spellHandler_) spellHandler_->sendPetAction(action, targetGuid);
}

void GameHandler::dismissCritter() {
    if (activeCritterGuid_ == 0 || !getSocket() || getState() != WorldState::IN_WORLD) return;
    const uint16_t wire = wireOpcode(Opcode::CMSG_DISMISS_CRITTER);
    if (wire == 0xFFFF) return;   // expansion has no such opcode
    auto pkt = DismissCritterPacket::build(activeCritterGuid_);
    getSocket()->send(pkt);
    LOG_INFO("Dismissing companion: guid=0x", std::hex, activeCritterGuid_, std::dec,
             " summonedBy=", activeCritterSpellId_);
    // The server confirms by clearing the critter field; leave the local state
    // alone so a refused dismiss does not leave the client thinking it worked.
}

void GameHandler::dismissPet() {
    if (spellHandler_) spellHandler_->dismissPet();
}

void GameHandler::togglePetSpellAutocast(uint32_t spellId) {
    if (spellHandler_) spellHandler_->togglePetSpellAutocast(spellId);
}

void GameHandler::renamePet(const std::string& newName) {
    if (spellHandler_) spellHandler_->renamePet(newName);
}

void GameHandler::requestStabledPetList() {
    if (spellHandler_) spellHandler_->requestStabledPetList();
}

void GameHandler::stablePet(uint8_t slot) {
    if (spellHandler_) spellHandler_->stablePet(slot);
}

void GameHandler::unstablePet(uint32_t petNumber) {
    if (spellHandler_) spellHandler_->unstablePet(petNumber);
}

void GameHandler::handleListStabledPets(network::Packet& packet) {
    if (spellHandler_) spellHandler_->handleListStabledPets(packet);
}

void GameHandler::setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id) {
    if (slot < 0 || slot >= ACTION_BAR_SLOTS) return;
    actionBar[slot].type = type;
    actionBar[slot].id = id;
    // Pre-query item information so action bar displays item name instead of "Item" placeholder
    if (type == ActionBarSlot::ITEM && id != 0) {
        queryItemInfo(id, 0);
    }
    saveCharacterConfig();
    // Notify Lua addons that the action bar changed
        fireAddonEvent("ACTIONBAR_SLOT_CHANGED", {std::to_string(slot + 1)});
        fireAddonEvent("ACTIONBAR_UPDATE_STATE", {});
    // Notify the server so the action bar persists across relogs.
    if (isInWorld()) {
        const bool classic = isClassicLikeExpansion();
        auto pkt = SetActionButtonPacket::build(
            static_cast<uint8_t>(slot),
            static_cast<uint8_t>(type),
            id,
            classic);
        socket->send(pkt);
    }
}

float GameHandler::getSpellCooldown(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellCooldown(spellId);
    return 0;
}

// ============================================================
// Talents
// ============================================================

void GameHandler::learnTalent(uint32_t talentId, uint32_t requestedRank) {
    if (spellHandler_) spellHandler_->learnTalent(talentId, requestedRank);
}

void GameHandler::switchTalentSpec(uint8_t newSpec) {
    if (spellHandler_) spellHandler_->switchTalentSpec(newSpec);
}

void GameHandler::confirmPetUnlearn() {
    if (spellHandler_) spellHandler_->confirmPetUnlearn();
}

void GameHandler::confirmTalentWipe() {
    if (spellHandler_) spellHandler_->confirmTalentWipe();
}

void GameHandler::sendAlterAppearance(uint32_t hairStyleEntry, uint32_t hairColor,
                                      uint32_t facialHairEntry, uint32_t skinColorEntry) {
    if (socialHandler_) {
        socialHandler_->sendAlterAppearance(hairStyleEntry, hairColor,
                                            facialHairEntry, skinColorEntry);
    }
}

// ============================================================
// Group/Party
// ============================================================

void GameHandler::inviteToGroup(const std::string& playerName) {
    if (socialHandler_) socialHandler_->inviteToGroup(playerName);
}

void GameHandler::acceptGroupInvite() {
    if (socialHandler_) socialHandler_->acceptGroupInvite();
}

void GameHandler::declineGroupInvite() {
    if (socialHandler_) socialHandler_->declineGroupInvite();
}

void GameHandler::leaveGroup() {
    if (socialHandler_) socialHandler_->leaveGroup();
}

void GameHandler::convertToRaid() {
    if (socialHandler_) socialHandler_->convertToRaid();
}

void GameHandler::sendSetLootMethod(uint32_t method, uint32_t threshold, uint64_t masterLooterGuid) {
    if (socialHandler_) socialHandler_->sendSetLootMethod(method, threshold, masterLooterGuid);
}

// ============================================================
// Guild Handlers
// ============================================================

void GameHandler::kickGuildMember(const std::string& playerName) {
    if (socialHandler_) socialHandler_->kickGuildMember(playerName);
}

void GameHandler::disbandGuild() {
    if (socialHandler_) socialHandler_->disbandGuild();
}

void GameHandler::setGuildLeader(const std::string& name) {
    if (socialHandler_) socialHandler_->setGuildLeader(name);
}

void GameHandler::setGuildPublicNote(const std::string& name, const std::string& note) {
    if (socialHandler_) socialHandler_->setGuildPublicNote(name, note);
}

void GameHandler::setGuildOfficerNote(const std::string& name, const std::string& note) {
    if (socialHandler_) socialHandler_->setGuildOfficerNote(name, note);
}

void GameHandler::acceptGuildInvite() {
    if (socialHandler_) socialHandler_->acceptGuildInvite();
}

void GameHandler::declineGuildInvite() {
    if (socialHandler_) socialHandler_->declineGuildInvite();
}

void GameHandler::submitGmTicket(const std::string& text) {
    if (chatHandler_) chatHandler_->submitGmTicket(text);
}

void GameHandler::deleteGmTicket() {
    if (socialHandler_) socialHandler_->deleteGmTicket();
}

void GameHandler::requestGmTicket() {
    if (socialHandler_) socialHandler_->requestGmTicket();
}

void GameHandler::queryGuildInfo(uint32_t guildId) {
    if (socialHandler_) socialHandler_->queryGuildInfo(guildId);
}

const std::string& GameHandler::lookupGuildName(uint32_t guildId) {
    static const std::string kEmpty;
    if (socialHandler_) return socialHandler_->lookupGuildName(guildId);
    return kEmpty;
}

uint32_t GameHandler::getEntityGuildId(uint64_t guid) const {
    if (socialHandler_) return socialHandler_->getEntityGuildId(guid);
    return 0;
}

void GameHandler::createGuild(const std::string& guildName) {
    if (socialHandler_) socialHandler_->createGuild(guildName);
}

void GameHandler::addGuildRank(const std::string& rankName) {
    if (socialHandler_) socialHandler_->addGuildRank(rankName);
}

void GameHandler::deleteGuildRank() {
    if (socialHandler_) socialHandler_->deleteGuildRank();
}

void GameHandler::requestPetitionShowlist(uint64_t npcGuid) {
    if (socialHandler_) socialHandler_->requestPetitionShowlist(npcGuid);
}

void GameHandler::buyPetition(uint64_t npcGuid, const std::string& guildName) {
    if (socialHandler_) socialHandler_->buyPetition(npcGuid, guildName);
}

void GameHandler::signPetition(uint64_t petitionGuid) {
    if (socialHandler_) socialHandler_->signPetition(petitionGuid);
}

void GameHandler::turnInPetition(uint64_t petitionGuid) {
    if (socialHandler_) socialHandler_->turnInPetition(petitionGuid);
}

// ============================================================
// Loot, Gossip, Vendor
// ============================================================

void GameHandler::lootTarget(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->lootTarget(guid);
}

void GameHandler::lootItem(uint8_t slotIndex) {
    if (inventoryHandler_) inventoryHandler_->lootItem(slotIndex);
}

void GameHandler::closeLoot() {
    if (inventoryHandler_) inventoryHandler_->closeLoot();
}

void GameHandler::scheduleGameObjectLootOpen(uint64_t guid, float delaySeconds, uint8_t attempts) {
    if (guid == 0) return;
    clearPendingGameObjectLootOpen(guid);
    PendingLootOpen pending;
    pending.guid = guid;
    pending.timer = std::max(0.0f, delaySeconds);
    pending.remainingAttempts = std::max<uint8_t>(attempts, 1);
    pendingGameObjectLootOpens_.push_back(pending);
}

bool GameHandler::isFishingHoleGameObject(uint64_t guid) const {
    if (guid == 0 || !entityController_) return false;
    auto entity = entityController_->getEntityManager().getEntity(guid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return false;
    const auto* info = getCachedGameObjectInfo(
        std::static_pointer_cast<GameObject>(entity)->getEntry());
    return info && info->type == kGoTypeFishingHole;
}

void GameHandler::clearPendingGameObjectLootOpen(uint64_t guid) {
    pendingGameObjectLootOpens_.erase(
        std::remove_if(pendingGameObjectLootOpens_.begin(), pendingGameObjectLootOpens_.end(),
                       [guid](const PendingLootOpen& pending) {
                           return guid == 0 || pending.guid == guid;
                       }),
        pendingGameObjectLootOpens_.end());
}

bool GameHandler::hasPendingGameObjectLootOpen(uint64_t guid) const {
    if (guid == 0) return false;
    return std::any_of(pendingGameObjectLootOpens_.begin(), pendingGameObjectLootOpens_.end(),
                       [guid](const PendingLootOpen& pending) {
                           return pending.guid == guid;
                       });
}

bool GameHandler::isGatherGameObject(uint64_t guid) const {
    if (guid == 0 || !entityController_) return false;
    auto entity = entityController_->getEntityManager().getEntity(guid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return false;

    auto go = std::static_pointer_cast<GameObject>(entity);
    const GameObjectQueryResponseData* goInfo = getCachedGameObjectInfo(go->getEntry());
    return gatherSpellForGameObject(goInfo, go->getName()) != 0;
}

void GameHandler::despawnCreatureLocally(uint64_t guid) {
    if (guid == 0 || !entityController_) return;

    auto& entityManager = entityController_->getEntityManager();
    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::UNIT) return;

    if (creatureDespawnCallback_) creatureDespawnCallback_(guid);
    entityManager.removeEntity(guid);

    if (combatHandler_ && guid == combatHandler_->getAutoAttackTargetGuid()) stopAutoAttack();
    if (getTargetGuid() == guid) setTargetGuidRaw(0);
    tabCycleStale = true;

    LOG_INFO("Locally despawned looted corpse: 0x", std::hex, guid, std::dec);
}

void GameHandler::despawnGameObjectLocally(uint64_t guid) {
    if (guid == 0 || !entityController_) return;

    auto& entityManager = entityController_->getEntityManager();
    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return;

    if (gameObjectDespawnCallback_) gameObjectDespawnCallback_(guid);
    entityManager.removeEntity(guid);

    clearPendingGameObjectLootOpen(guid);
    if (lastInteractedGoGuid_ == guid) lastInteractedGoGuid_ = 0;
    if (pendingGameObjectInteractGuid_ == guid) pendingGameObjectInteractGuid_ = 0;
    if (getTargetGuid() == guid) setTargetGuidRaw(0);
    tabCycleStale = true;

    LOG_INFO("Locally despawned game object: 0x", std::hex, guid, std::dec);
}

void GameHandler::lootMasterGive(uint8_t lootSlot, uint64_t targetGuid) {
    if (inventoryHandler_) inventoryHandler_->lootMasterGive(lootSlot, targetGuid);
}

void GameHandler::interactWithNpc(uint64_t guid) {
    if (!isInWorld()) return;
    auto packet = GossipHelloPacket::build(guid);
    socket->send(packet);
}

void GameHandler::interactWithGameObject(uint64_t guid) {
    LOG_DEBUG("[GO-DIAG] interactWithGameObject called: guid=0x", std::hex, guid, std::dec);
    if (guid == 0) { LOG_DEBUG("[GO-DIAG] BLOCKED: guid==0"); return; }
    if (!isInWorld()) { LOG_DEBUG("[GO-DIAG] BLOCKED: not in world"); return; }
    // Do not overlap an actual spell cast. Reeling an owned fishing bobber is
    // the deliberate exception: the fishing spell remains channelled while the
    // bobber is in the water, and using the hooked bobber is what ends it.
    const bool reelingHookedFishingBobber = guid == hookedFishingBobberGuid_;
    if (!reelingHookedFishingBobber && spellHandler_ && spellHandler_->isCasting() &&
        spellHandler_->getCurrentCastSpellId() != 0) {
        LOG_DEBUG("[GO-DIAG] BLOCKED: already casting spellId=", spellHandler_->getCurrentCastSpellId());
        return;
    }
    // Always clear melee intent before GO interactions.
    stopAutoAttack();
    // And get off the mount. Opening a chest, gathering a node or using very
    // nearly anything else puts a player on foot in WoW, and staying mounted
    // through it both looks wrong and leaves the server refusing the actions
    // that check for it.
    //
    // Not on a taxi: the flight's mount is not the player's to dismiss, and
    // there is nothing to interact with mid-flight anyway.
    if (isMounted() && !isTaxiMountActive()) {
        LOG_DEBUG("[GO-DIAG] dismounting before interacting");
        dismount();
    }
    // Set the pending GO guid so that:
    // 1. cancelCast() won't send CMSG_CANCEL_CAST for GO-triggered casts
    //    (e.g., "Opening" on a quest chest) — without this, any movement
    //    during the cast cancels it server-side and quest credit is lost.
    // 2. The cast-completion fallback in update() can call
    //    performGameObjectInteractionNow after the cast timer expires.
    // 3. isGameObjectInteractionCasting() returns true during GO casts.
    pendingGameObjectInteractGuid_ = guid;
    performGameObjectInteractionNow(guid);
}

void GameHandler::performGameObjectInteractionNow(uint64_t guid) {
    if (guid == 0) return;
    if (!isInWorld()) return;
    // Rate-limit to prevent spamming the server
    static uint64_t lastInteractGuid = 0;
    static std::chrono::steady_clock::time_point lastInteractTime{};
    auto now = std::chrono::steady_clock::now();
    // Keep duplicate suppression, but allow quick retry clicks.
    constexpr int64_t minRepeatMs = 150;
    if (guid == lastInteractGuid &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastInteractTime).count() < minRepeatMs) {
        return;
    }
    lastInteractGuid = guid;
    lastInteractTime = now;

    // Ensure GO interaction isn't blocked by stale or active melee state.
    stopAutoAttack();
    auto entity = entityController_->getEntityManager().getEntity(guid);
    uint32_t goEntry = 0;
    uint32_t goType = 0;
    std::string goName;
    const GameObjectQueryResponseData* goInfo = nullptr;
    float interactionDistance = -1.0f;

    if (entity) {
        if (entity->getType() == ObjectType::GAMEOBJECT) {
            auto go = std::static_pointer_cast<GameObject>(entity);
            goEntry = go->getEntry();
            goName = go->getName();
            goInfo = getCachedGameObjectInfo(goEntry);
            if (goInfo) goType = goInfo->type;
            if (goType == kGoTypeFishingHole) {
                // Nothing sent, and nothing scheduled: the school is fished, not
                // opened. Silent, because retail does not respond to the click
                // either.
                LOG_INFO("GO fishing hole click ignored: guid=0x", std::hex, guid,
                         std::dec, " entry=", goEntry, " name='", goName, "'");
                return;
            }
            if (goType == 5 && !goName.empty()) {
                std::string lower = lowerCopy(goName);
                if (lower.rfind("doodad_", 0) != 0) {
                    addSystemChatMessage(goName);
                }
            }
        }
        // Face object and send heartbeat before use so strict servers don't require
        // a nudge movement to accept interaction.
        float dx = entity->getX() - movementInfo.x;
        float dy = entity->getY() - movementInfo.y;
        float dz = entity->getZ() - movementInfo.z;
        float dist3d = std::sqrt(dx * dx + dy * dy + dz * dz);
        interactionDistance = dist3d;
        // Fishing bobbers are intentionally cast beyond normal GO-use range.
        // Only the owned bobber whose bite we observed gets the extended range.
        const float maxInteractDistance = (guid == hookedFishingBobberGuid_) ? 30.0f : 10.0f;
        if (dist3d > maxInteractDistance) {
            addSystemChatMessage("Too far away.");
            return;
        }
        // Stop movement before interacting — servers may reject GO use or
        // immediately cancel the resulting spell cast if the player is moving.
        const uint32_t moveFlags = movementInfo.flags;
        const bool isMoving = (moveFlags & 0x00000001u) || // FORWARD
                              (moveFlags & 0x00000002u) || // BACKWARD
                              (moveFlags & 0x00000004u) || // STRAFE_LEFT
                              (moveFlags & 0x00000008u);   // STRAFE_RIGHT
        if (isMoving) {
            movementInfo.flags &= ~0x0000000Fu; // clear directional movement flags
            sendMovement(Opcode::MSG_MOVE_STOP);
        }
        if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
            faceCanonicalYaw(std::atan2(-dy, dx));
        }
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    // Determine GO type for interaction strategy
    bool isMailbox = false;
    bool isGuildBank = false;
    bool chestLike = false;
    bool metadataPending = false;
    if (entity && entity->getType() == ObjectType::GAMEOBJECT) {
        auto go = std::static_pointer_cast<GameObject>(entity);
        if (!goInfo) goInfo = getCachedGameObjectInfo(go->getEntry());
        metadataPending = (goInfo == nullptr);
        if (goInfo && goInfo->type == 19) {
            isMailbox = true;
        } else if (goInfo && goInfo->type == 34) { // GAMEOBJECT_TYPE_GUILD_BANK
            isGuildBank = true;
        } else if (goInfo && goInfo->type == 3) {
            chestLike = true;
        }
    }
    if (!chestLike && !goName.empty()) {
        // Query metadata can arrive after the player clicks. Recognize common
        // quest-loot containers by name so objects such as Sack of Oats still
        // receive the delayed CMSG_LOOT sequence used by type-3 chests.
        chestLike = isLootContainerName(goName);
    }

    LOG_INFO("GO interaction: guid=0x", std::hex, guid, std::dec,
             " entry=", goEntry, " type=", goType,
             " name='", goName, "' chestLike=", chestLike,
             " metadataPending=", metadataPending, " isMailbox=", isMailbox,
             " lockId=", (goInfo && goInfo->hasData ? goInfo->data[0] : 0));

    const uint32_t gatherBaseSpellId = gatherSpellForGameObject(goInfo, goName);
    if (gatherBaseSpellId != 0) {
        const uint32_t gatherSpellId = knownGatherRank(spellHandler_.get(), gatherBaseSpellId);
        if (gatherSpellId == 0) {
            addSystemChatMessage(gatherBaseSpellId == 2575 ? "Requires Mining." : "Requires Herbalism.");
            LOG_INFO("GO gather skipped: no known rank for base spell=", gatherBaseSpellId,
                     " guid=0x", std::hex, guid, std::dec, " name='", goName, "'");
            return;
        }
        auto castPacket = getPacketParsers()
            ? getPacketParsers()->buildCastGameObjectSpell(gatherSpellId, guid, 0)
            : CastSpellPacket::buildGameObjectTarget(gatherSpellId, guid, 0);
        socket->send(castPacket);
        lastInteractedGoGuid_ = guid;
        scheduleGameObjectLootOpen(guid, 0.50f, 8);
        LOG_INFO("GO gather cast: spell=", gatherSpellId, " guid=0x",
                 std::hex, guid, std::dec, " name='", goName, "'");
        return;
    }

    if (chestLike && isActiveExpansion("wotlk")) {
        // A locked type-3 chest is opened by casting an OPEN_LOCK spell at it (a
        // known Pick Lock, or the generic Opening); the server then validates the
        // lock. A plain CMSG_GAMEOBJ_USE only opens unlocked / server-gated quest
        // containers (e.g. the Legend of Stalvan Sealed Crate).
        const LockOpenPlan plan = planGameObjectOpen(
            services_.assetManager, goInfo, spellHandler_->getKnownSpells());
        if (plan.method == LockOpenMethod::CastSpell) {
            auto castPacket = getPacketParsers()->buildCastGameObjectSpell(
                plan.spellId, guid, 0);
            socket->send(castPacket);
            lastInteractedGoGuid_ = guid;
            scheduleGameObjectLootOpen(guid, 0.60f, 8);
            LOG_INFO("GO chest open-lock cast: spell=", plan.spellId,
                     " guid=0x", std::hex, guid, std::dec,
                     " entry=", goEntry, " name='", goName, "'");
            return;
        }
        // Key-item lock: a warrior knows no open-lock spell. Open it by USING the
        // key item on the chest (CMSG_USE_ITEM targeting the GO) — the server casts
        // the key's on-use OPEN_LOCK spell because the player holds the key. A plain
        // USE does not open a locked type-3 chest, and casting the item's spell via
        // CMSG_CAST_SPELL is rejected (the player doesn't "know" it).
        if (plan.keyItemId != 0) {
            const ItemQueryResponseData* keyInfo = getItemInfo(plan.keyItemId);
            uint32_t keyUseSpell = 0;
            if (keyInfo) {
                for (const auto& sp : keyInfo->spells)
                    if (sp.spellTrigger == 0 && sp.spellId != 0 && keyUseSpell == 0)
                        keyUseSpell = sp.spellId;
            } else {
                ensureItemInfo(plan.keyItemId); // not cached yet — request for next click
            }

            // Locate the key: wire (bag, slot) + item GUID. Keyring wire slots
            // start at 86; backpack at 23 in bag 0xFF; equipped bags are 19..22.
            const Inventory& inv = getInventory();
            uint8_t keyBag = 0xFF, keySlot = 0; uint64_t keyGuid = 0; bool keyFound = false;
            for (int i = 0; i < inv.getBackpackSize() && !keyFound; ++i)
                if (inv.getBackpackSlot(i).item.itemId == plan.keyItemId) {
                    keyBag = 0xFF; keySlot = static_cast<uint8_t>(23 + i);
                    keyGuid = inv.getBackpackSlot(i).item.guid; keyFound = true;
                }
            for (int b = 0; b < Inventory::NUM_BAG_SLOTS && !keyFound; ++b)
                for (int s = 0; s < inv.getBagSize(b) && !keyFound; ++s)
                    if (inv.getBagSlot(b, s).item.itemId == plan.keyItemId) {
                        keyBag = static_cast<uint8_t>(19 + b); keySlot = static_cast<uint8_t>(s);
                        keyGuid = inv.getBagSlot(b, s).item.guid; keyFound = true;
                    }
            for (int i = 0; i < inv.getKeyringSize() && !keyFound; ++i)
                if (inv.getKeyringSlot(i).item.itemId == plan.keyItemId) {
                    keyBag = 0xFF; keySlot = static_cast<uint8_t>(86 + i);
                    keyGuid = inv.getKeyringSlot(i).item.guid; keyFound = true;
                }

            if (keyFound && keyUseSpell != 0) {
                auto usePacket = UseItemPacket::build(keyBag, keySlot, keyGuid,
                                                      keyUseSpell, 0, 0, guid);
                socket->send(usePacket);
                lastInteractedGoGuid_ = guid;
                scheduleGameObjectLootOpen(guid, 0.60f, 8);
                LOG_INFO("GO chest key-item use: keyItem=", plan.keyItemId,
                         " spell=", keyUseSpell, " keySlot(bag=", (int)keyBag,
                         " slot=", (int)keySlot, ") guid=0x", std::hex, guid, std::dec,
                         " entry=", goEntry, " name='", goName, "'");
                return;
            }
        }
        // LockOpenMethod::UseDirect — fall through to CMSG_GAMEOBJ_USE.
        LOG_INFO("GO chest opens via direct USE: lockId=",
                 goInfo && goInfo->hasData ? goInfo->data[0] : 0,
                 " keyItem=", plan.keyItemId,
                 " guid=0x", std::hex, guid, std::dec,
                 " entry=", goEntry, " name='", goName, "'");
    }

    // Every expansion activates game objects through CMSG_GAMEOBJ_USE.
    auto usePacket = GameObjectUsePacket::build(guid);
    socket->send(usePacket);
    lastInteractedGoGuid_ = guid;
    if (guid == hookedFishingBobberGuid_) {
        LOG_WARNING("Fishing bobber reel sent: guid=0x", std::hex, guid, std::dec,
                    " distance=", interactionDistance);
        hookedFishingBobberGuid_ = 0;
    }

    if (chestLike || metadataPending) {
        // Don't send CMSG_LOOT immediately — the server may start a timed cast
        // (e.g., "Opening") and the GO isn't lootable until the cast finishes.
        // Sending LOOT prematurely gets an empty response or is silently dropped,
        // which can interfere with the server's loot state machine.
        // Queue a delayed open: if a server-side gather cast starts, update()
        // defers this until the cast is over; if no cast packet arrives, retry
        // a few times so resource nodes do not fail after one early CMSG_LOOT.
        // Unknown metadata is common immediately after a GO spawn. A delayed
        // loot probe is harmless for non-loot objects and prevents the first
        // click on quest containers from being lost while their query is pending.
        scheduleGameObjectLootOpen(guid, 0.35f, 8);
    } else if (isMailbox) {
        openMailbox(guid);
    } else if (isGuildBank) {
        // Guild vault: CMSG_GAMEOBJ_USE above is a no-op on the server; the bank
        // opens via CMSG_GUILD_BANKER_ACTIVATE, which openGuildBank() sends.
        openGuildBank(guid);
    }

    // CMSG_GAMEOBJ_REPORT_USE triggers GO AI scripts (SmartAI, ScriptAI) which
    // is where many quest objectives grant credit. Previously this was only sent
    // for non-chest GOs, so chest-type quest objectives (Bundle of Wood, etc.)
    // never triggered the server-side quest credit script.
    if (!isMailbox && !isGuildBank) {
        const auto* table = getActiveOpcodeTable();
        if (table && table->hasOpcode(Opcode::CMSG_GAMEOBJ_REPORT_USE)) {
            network::Packet reportUse(wireOpcode(Opcode::CMSG_GAMEOBJ_REPORT_USE));
            reportUse.writeUInt64(guid);
            socket->send(reportUse);
        }
    }
}

void GameHandler::selectGossipOption(uint32_t optionId) {
    if (questHandler_) questHandler_->selectGossipOption(optionId);
}

void GameHandler::selectGossipQuest(uint32_t questId) {
    if (questHandler_) questHandler_->selectGossipQuest(questId);
}

bool GameHandler::requestQuestQuery(uint32_t questId, bool force) {
    return questHandler_ && questHandler_->requestQuestQuery(questId, force);
}

bool GameHandler::hasQuestInLog(uint32_t questId) const {
    return questHandler_ && questHandler_->hasQuestInLog(questId);
}

Unit* GameHandler::getUnitByGuid(uint64_t guid) {
    auto entity = entityController_->getEntityManager().getEntity(guid);
    // Use the type tag to skip RTTI — both UNIT and PLAYER object types derive from Unit.
    if (!entity || !entity->isUnit()) return nullptr;
    return static_cast<Unit*>(entity.get());
}

std::string GameHandler::guidToUnitId(uint64_t guid) const {
    if (guid == playerGuid)      return "player";
    if (guid == targetGuid)      return "target";
    if (guid == focusGuid)       return "focus";
    if (guid == petGuid_)        return "pet";
    return {};
}

std::string GameHandler::getQuestTitle(uint32_t questId) const {
    for (const auto& q : questLog_)
        if (q.questId == questId && !q.title.empty()) return q.title;
    return {};
}

const GameHandler::QuestLogEntry* GameHandler::findQuestLogEntry(uint32_t questId) const {
    for (const auto& q : questLog_)
        if (q.questId == questId) return &q;
    return nullptr;
}

int GameHandler::findQuestLogSlotIndexFromServer(uint32_t questId) const {
    if (questHandler_) return questHandler_->findQuestLogSlotIndexFromServer(questId);
    return 0;
}

void GameHandler::addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives) {
    if (questHandler_) questHandler_->addQuestToLocalLogIfMissing(questId, title, objectives);
}

bool GameHandler::resyncQuestLogFromServerSlots(bool forceQueryMetadata) {
    return questHandler_ && questHandler_->resyncQuestLogFromServerSlots(forceQueryMetadata);
}

// Apply quest completion state from player update fields to already-tracked local quests.
// Called from VALUES update handler so quests that complete mid-session (or that were
// complete on login) get quest.complete=true without waiting for SMSG_QUESTUPDATE_COMPLETE.
void GameHandler::applyQuestStateFromFields(const FlatFieldMap& fields) {
    if (questHandler_) questHandler_->applyQuestStateFromFields(fields);
}

// Extract expansion-specific kill/objective counters from quest-log update fields
// and populate quest.killCounts using the structured objectives obtained from a prior
// SMSG_QUEST_QUERY_RESPONSE. Silently does nothing if objectives are absent.
void GameHandler::applyPackedKillCountsFromFields(QuestLogEntry& quest) {
    if (questHandler_) questHandler_->applyPackedKillCountsFromFields(quest);
}

void GameHandler::clearPendingQuestAccept(uint32_t questId) {
    if (questHandler_) questHandler_->clearPendingQuestAccept(questId);
}

void GameHandler::triggerQuestAcceptResync(uint32_t questId, uint64_t npcGuid, const char* reason) {
    if (questHandler_) questHandler_->triggerQuestAcceptResync(questId, npcGuid, reason);
}

void GameHandler::acceptQuest() {
    if (questHandler_) questHandler_->acceptQuest();
}

void GameHandler::declineQuest() {
    if (questHandler_) questHandler_->declineQuest();
}

void GameHandler::abandonQuest(uint32_t questId) {
    if (questHandler_) questHandler_->abandonQuest(questId);
    setQuestTracked(questId, false);
    setQuestShownOnMap(questId, false);
}

void GameHandler::shareQuestWithParty(uint32_t questId) {
    if (questHandler_) questHandler_->shareQuestWithParty(questId);
}

void GameHandler::completeQuest() {
    if (questHandler_) questHandler_->completeQuest();
}

void GameHandler::closeQuestRequestItems() {
    if (questHandler_) questHandler_->closeQuestRequestItems();
}

void GameHandler::chooseQuestReward(uint32_t rewardIndex) {
    if (questHandler_) questHandler_->chooseQuestReward(rewardIndex);
}

void GameHandler::closeQuestOfferReward() {
    if (questHandler_) questHandler_->closeQuestOfferReward();
}

void GameHandler::closeGossip() {
    if (questHandler_) questHandler_->closeGossip();
}

void GameHandler::offerQuestFromItem(uint64_t itemGuid, uint32_t questId) {
    if (questHandler_) questHandler_->offerQuestFromItem(itemGuid, questId);
}

uint64_t GameHandler::getBagItemGuid(int bagIndex, int slotIndex) const {
    if (bagIndex < 0 || bagIndex >= inventory.NUM_BAG_SLOTS) return 0;
    if (slotIndex < 0) return 0;
    uint64_t bagGuid = equipSlotGuids_[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (bagGuid == 0) return 0;
    auto it = containerContents_.find(bagGuid);
    if (it == containerContents_.end()) return 0;
    if (slotIndex >= static_cast<int>(it->second.numSlots)) return 0;
    return it->second.slotGuids[slotIndex];
}

void GameHandler::openVendor(uint64_t npcGuid) {
    if (inventoryHandler_) inventoryHandler_->openVendor(npcGuid);
}

void GameHandler::closeVendor() {
    if (inventoryHandler_) inventoryHandler_->closeVendor();
}

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
    if (inventoryHandler_) inventoryHandler_->buyItem(vendorGuid, itemId, slot, count);
}

void GameHandler::buyBackItem(uint32_t buybackSlot) {
    if (inventoryHandler_) inventoryHandler_->buyBackItem(buybackSlot);
}

void GameHandler::repairItem(uint64_t vendorGuid, uint64_t itemGuid) {
    if (inventoryHandler_) inventoryHandler_->repairItem(vendorGuid, itemGuid);
}

void GameHandler::repairAll(uint64_t vendorGuid, bool useGuildBank) {
    if (inventoryHandler_) inventoryHandler_->repairAll(vendorGuid, useGuildBank);
}

uint32_t GameHandler::estimateRepairAllCost() const {
    if (inventoryHandler_) return inventoryHandler_->estimateRepairAllCost();
    return 0;
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    if (inventoryHandler_) inventoryHandler_->sellItem(vendorGuid, itemGuid, count);
}

void GameHandler::sellItemBySlot(int backpackIndex) {
    if (inventoryHandler_) inventoryHandler_->sellItemBySlot(backpackIndex);
}

void GameHandler::autoEquipItemBySlot(int backpackIndex) {
    if (inventoryHandler_) inventoryHandler_->autoEquipItemBySlot(backpackIndex);
}

void GameHandler::autoEquipItemInBag(int bagIndex, int slotIndex) {
    if (inventoryHandler_) inventoryHandler_->autoEquipItemInBag(bagIndex, slotIndex);
}

void GameHandler::sellItemInBag(int bagIndex, int slotIndex) {
    if (inventoryHandler_) inventoryHandler_->sellItemInBag(bagIndex, slotIndex);
}

void GameHandler::unequipToBackpack(EquipSlot equipSlot) {
    if (inventoryHandler_) inventoryHandler_->unequipToBackpack(equipSlot);
}

void GameHandler::swapContainerItems(uint8_t srcBag, uint8_t srcSlot, uint8_t dstBag, uint8_t dstSlot) {
    if (inventoryHandler_) inventoryHandler_->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
}

void GameHandler::swapBagSlots(int srcBagIndex, int dstBagIndex) {
    if (inventoryHandler_) inventoryHandler_->swapBagSlots(srcBagIndex, dstBagIndex);
}

void GameHandler::destroyItem(uint8_t bag, uint8_t slot, uint8_t count) {
    if (inventoryHandler_) inventoryHandler_->destroyItem(bag, slot, count);
}

void GameHandler::splitItem(uint8_t srcBag, uint8_t srcSlot, uint8_t count) {
    if (inventoryHandler_) inventoryHandler_->splitItem(srcBag, srcSlot, count);
}

void GameHandler::useItemBySlot(int backpackIndex) {
    if (inventoryHandler_) inventoryHandler_->useItemBySlot(backpackIndex);
}

void GameHandler::useItemInBag(int bagIndex, int slotIndex) {
    if (inventoryHandler_) inventoryHandler_->useItemInBag(bagIndex, slotIndex);
}

bool GameHandler::isAwaitingItemTarget() const {
    return inventoryHandler_ && inventoryHandler_->isAwaitingItemTarget();
}

void GameHandler::beginSpellItemTargeting(uint32_t spellId, const std::string& spellName) {
    if (inventoryHandler_) inventoryHandler_->beginSpellItemTargeting(spellId, spellName);
}

uint32_t GameHandler::getPendingItemTargetSourceItemId() const {
    return inventoryHandler_ ? inventoryHandler_->getPendingItemTargetSourceItemId() : 0;
}

void GameHandler::cancelItemTargeting() {
    if (inventoryHandler_) inventoryHandler_->cancelItemTargeting();
}

void GameHandler::completeItemUseOnItem(uint64_t targetItemGuid) {
    if (inventoryHandler_) inventoryHandler_->completeItemUseOnItem(targetItemGuid);
}

void GameHandler::openItemBySlot(int backpackIndex) {
    if (inventoryHandler_) inventoryHandler_->openItemBySlot(backpackIndex);
}

void GameHandler::openItemInBag(int bagIndex, int slotIndex) {
    if (inventoryHandler_) inventoryHandler_->openItemInBag(bagIndex, slotIndex);
}

void GameHandler::readItemBySlot(int backpackIndex) {
    if (inventoryHandler_) inventoryHandler_->readItemBySlot(backpackIndex);
}

void GameHandler::readItemInBag(int bagIndex, int slotIndex) {
    if (inventoryHandler_) inventoryHandler_->readItemInBag(bagIndex, slotIndex);
}

void GameHandler::useItemById(uint32_t itemId) {
    if (inventoryHandler_) inventoryHandler_->useItemById(itemId);
}

uint32_t GameHandler::getItemIdForSpell(uint32_t spellId) const {
    if (spellId == 0) return 0;
    // Search backpack and bags for an item whose on-use spell matches
    for (int i = 0; i < inventory.getBackpackSize(); i++) {
        const auto& slot = inventory.getBackpackSlot(i);
        if (slot.empty()) continue;
        auto* info = getItemInfo(slot.item.itemId);
        if (!info || !info->valid) continue;
        for (const auto& sp : info->spells) {
            if (sp.spellId == spellId && (sp.spellTrigger == 0 || sp.spellTrigger == 5))
                return slot.item.itemId;
        }
    }
    for (int bag = 0; bag < inventory.NUM_BAG_SLOTS; bag++) {
        for (int s = 0; s < inventory.getBagSize(bag); s++) {
            const auto& slot = inventory.getBagSlot(bag, s);
            if (slot.empty()) continue;
            auto* info = getItemInfo(slot.item.itemId);
            if (!info || !info->valid) continue;
            for (const auto& sp : info->spells) {
                if (sp.spellId == spellId && (sp.spellTrigger == 0 || sp.spellTrigger == 5))
                    return slot.item.itemId;
            }
        }
    }
    return 0;
}

void GameHandler::unstuck() {
    if (unstuckCallback_) {
        unstuckCallback_();
        addSystemChatMessage("Unstuck: snapped upward. Use /unstuckgy for full teleport.");
    }
}

void GameHandler::unstuckGy() {
    if (unstuckGyCallback_) {
        unstuckGyCallback_();
        addSystemChatMessage("Unstuck: teleported to safe location.");
    }
}

void GameHandler::unstuckHearth() {
    if (unstuckHearthCallback_) {
        unstuckHearthCallback_();
        addSystemChatMessage("Unstuck: teleported to hearthstone location.");
    } else {
        addSystemChatMessage("No hearthstone bind point set.");
    }
}

// ============================================================
// Trainer
// ============================================================

void GameHandler::trainSpell(uint32_t spellId) {
    if (inventoryHandler_) inventoryHandler_->trainSpell(spellId);
}

void GameHandler::closeTrainer() {
    if (inventoryHandler_) inventoryHandler_->closeTrainer();
}

void GameHandler::preloadDBCCaches() const {
    LOG_INFO("Pre-loading DBC caches during world entry...");
    auto t0 = std::chrono::steady_clock::now();

    loadSpellNameCache();   // Spell.dbc — largest, ~170ms cold
    loadTitleNameCache();   // CharTitles.dbc
    loadFactionNameCache(); // Faction.dbc
    loadAreaNameCache();    // WorldMapArea.dbc
    loadMapNameCache();     // Map.dbc
    loadLfgDungeonDbc();    // LFGDungeons.dbc

    // Validate animation constants against AnimationData.dbc
    if (auto* am = services_.assetManager) {
        auto animDbc = am->loadDBC("AnimationData.dbc");
        rendering::anim::validateAgainstDBC(animDbc);
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    LOG_INFO("DBC cache pre-load complete in ", elapsed, " ms");
}

void GameHandler::loadSpellNameCache() const {
    if (spellHandler_) spellHandler_->loadSpellNameCache();
}

void GameHandler::loadSkillLineAbilityDbc() {
    if (spellHandler_) spellHandler_->loadSkillLineAbilityDbc();
}

const std::vector<GameHandler::SpellBookTab>& GameHandler::getSpellBookTabs() {
    static const std::vector<SpellBookTab> kEmpty;
    if (spellHandler_) return spellHandler_->getSpellBookTabs();
    return kEmpty;
}

void GameHandler::categorizeTrainerSpells() {
    if (spellHandler_) spellHandler_->categorizeTrainerSpells();
}

void GameHandler::loadTalentDbc() {
    if (spellHandler_) spellHandler_->loadTalentDbc();
}

static const std::string EMPTY_STRING;

const int32_t* GameHandler::getSpellEffectBasePoints(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellEffectBasePoints(spellId);
    return nullptr;
}

float GameHandler::getSpellDuration(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellDuration(spellId);
    return 0.0f;
}

const std::string& GameHandler::getSpellName(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellName(spellId);
    return EMPTY_STRING;
}

uint32_t GameHandler::getSpellTargetFlags(uint32_t spellId) const {
    return spellHandler_ ? spellHandler_->getSpellTargetFlags(spellId) : 0;
}

uint32_t GameHandler::getSpellTargetAuraState(uint32_t spellId) const {
    return spellHandler_ ? spellHandler_->getSpellTargetAuraState(spellId) : 0;
}

const std::string& GameHandler::getSpellRank(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellRank(spellId);
    return EMPTY_STRING;
}

const std::string& GameHandler::getSpellDescription(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellDescription(spellId);
    return EMPTY_STRING;
}

namespace {
// Format a duration in seconds the way WoW tooltips do: "12 sec", "5 min", "1 hour".
std::string formatSpellDurationSec(float sec) {
    if (sec <= 0.0f) return {};
    auto trim = [](float v) {
        char buf[32];
        if (v == static_cast<float>(static_cast<long>(v)))
            snprintf(buf, sizeof(buf), "%ld", static_cast<long>(v));
        else
            snprintf(buf, sizeof(buf), "%.1f", v);
        return std::string(buf);
    };
    if (sec >= 3600.0f && std::fmod(sec, 3600.0f) == 0.0f) {
        float h = sec / 3600.0f;
        return trim(h) + (h == 1.0f ? " hour" : " hours");
    }
    if (sec >= 60.0f) return trim(sec / 60.0f) + " min";
    return trim(sec) + " sec";
}
} // namespace

std::string GameHandler::formatSpellDescription(uint32_t selfSpellId,
                                                const std::string& raw) const {
    if (raw.empty()) return raw;

    std::string out;
    out.reserve(raw.size());
    const size_t n = raw.size();
    long lastValue = -1;  // drives $l plural selection

    // Resolve a base-point magnitude ($s/$o/$m/$M) for a spell + effect index.
    auto basePoints = [&](uint32_t sid, int idx) -> long {
        const int32_t* bp = getSpellEffectBasePoints(sid);
        if (!bp || idx < 0 || idx > 2) return LONG_MIN;
        return std::labs(static_cast<long>(bp[idx]) + 1);  // stored as value-1
    };

    for (size_t i = 0; i < n; ) {
        char c = raw[i];
        if (c != '$') { out += c; ++i; continue; }
        if (i + 1 >= n) { out += c; ++i; continue; }

        char next = raw[i + 1];

        // Plural "$lsingular:plural;" / gender "$gmale:female;" — emit one branch.
        if (next == 'l' || next == 'L' || next == 'g' || next == 'G') {
            size_t semi = raw.find(';', i + 2);
            if (semi != std::string::npos) {
                std::string body = raw.substr(i + 2, semi - (i + 2));
                size_t colon = body.find(':');
                std::string a = (colon == std::string::npos) ? body : body.substr(0, colon);
                std::string b = (colon == std::string::npos) ? body : body.substr(colon + 1);
                bool plural = (next == 'l' || next == 'L') ? (lastValue != 1) : false;
                out += plural ? b : a;
                i = semi + 1;
                continue;
            }
        }

        // Bracketed math expression "${...}" — can't evaluate; strip it (and a trailing %).
        if (next == '{') {
            size_t close = raw.find('}', i + 2);
            if (close != std::string::npos) {
                i = close + 1;
                if (i < n && raw[i] == '%') ++i;
                continue;
            }
        }

        // Division token "$/N;<valueToken>" — e.g. "$/5;s1" is "s1 divided by 5"
        // (food eat spells: "Restores $/5;s1 health per second").
        if (next == '/') {
            size_t k = i + 2;
            long divisor = 0;
            while (k < n && raw[k] >= '0' && raw[k] <= '9') { divisor = divisor * 10 + (raw[k] - '0'); ++k; }
            if (k < n && raw[k] == ';' && divisor != 0) {
                ++k;  // past ';'
                uint32_t rid = 0; bool hr = false;
                while (k < n && raw[k] >= '0' && raw[k] <= '9') { rid = rid * 10 + (raw[k] - '0'); hr = true; ++k; }
                if (k < n) {
                    char cd = raw[k++];
                    int ix = 0;
                    if (k < n && raw[k] >= '1' && raw[k] <= '3') { ix = raw[k] - '1'; ++k; }
                    if (cd == 's' || cd == 'S' || cd == 'o' || cd == 'O' || cd == 'm' || cd == 'M') {
                        long v = basePoints(hr ? rid : selfSpellId, ix);
                        if (v != LONG_MIN) {
                            long dv = v / divisor;
                            out += std::to_string(dv);
                            lastValue = dv;
                            i = k;
                            continue;
                        }
                    }
                }
            }
            // Unparseable division token — strip through the terminating ';' if present.
            size_t semi = raw.find(';', i + 2);
            i = (semi != std::string::npos) ? semi + 1 : i + 2;
            if (i < n && raw[i] == '%') ++i;
            continue;
        }

        // General token: $ [spellId] letter [effectIndex]
        size_t j = i + 1;
        uint32_t refId = 0;
        bool hasRef = false;
        while (j < n && raw[j] >= '0' && raw[j] <= '9') {
            refId = refId * 10 + static_cast<uint32_t>(raw[j] - '0');
            hasRef = true;
            ++j;
        }
        if (j >= n) { out += c; ++i; continue; }
        char code = raw[j++];
        int idx = 0;
        if (j < n && raw[j] >= '1' && raw[j] <= '3') { idx = raw[j] - '1'; ++j; }
        uint32_t target = hasRef ? refId : selfSpellId;

        bool resolved = false;
        switch (code) {
            case 's': case 'S': case 'o': case 'O':
            case 'm': case 'M': {
                long v = basePoints(target, idx);
                if (v != LONG_MIN) { out += std::to_string(v); lastValue = v; resolved = true; }
                break;
            }
            case 'd': {
                std::string dur = formatSpellDurationSec(getSpellDuration(target));
                if (!dur.empty()) { out += dur; resolved = true; }
                break;
            }
            default: break;  // $h proc chance, $t period, $a radius, ... — not resolvable here
        }

        if (resolved) {
            i = j;
        } else {
            // Drop the whole token; a value slot immediately followed by '%' drops that too,
            // so we don't leave a dangling "%".
            i = j;
            if (i < n && raw[i] == '%') ++i;
        }
    }

    // Collapse whitespace left behind by stripped tokens ("a  chance" -> "a chance").
    std::string cleaned;
    cleaned.reserve(out.size());
    bool prevSpace = false;
    for (char c : out) {
        bool isSpace = (c == ' ');
        if (isSpace && prevSpace) continue;
        cleaned += c;
        prevSpace = isSpace;
    }
    return cleaned;
}

const std::string& GameHandler::getSpellFocusName(uint32_t focusId) const {
    if (spellHandler_) return spellHandler_->getSpellFocusName(focusId);
    return EMPTY_STRING;
}

const std::string& GameHandler::getTotemCategoryName(uint32_t categoryId) const {
    if (spellHandler_) return spellHandler_->getTotemCategoryName(categoryId);
    return EMPTY_STRING;
}

std::string GameHandler::getEnchantName(uint32_t enchantId) const {
    if (spellHandler_) return spellHandler_->getEnchantName(enchantId);
    return {};
}

uint8_t GameHandler::getSpellDispelType(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellDispelType(spellId);
    return 0;
}

bool GameHandler::isSpellInterruptible(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->isSpellInterruptible(spellId);
    return true;
}

uint32_t GameHandler::getSpellSchoolMask(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSpellSchoolMask(spellId);
    return 0;
}

const std::string& GameHandler::getSkillLineName(uint32_t spellId) const {
    if (spellHandler_) return spellHandler_->getSkillLineName(spellId);
    return EMPTY_STRING;
}


} // namespace game
} // namespace wowee
