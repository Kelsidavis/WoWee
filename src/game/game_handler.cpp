#include "game/game_handler.hpp"
#include "game/opcodes.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <random>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <zlib.h>

namespace wowee {
namespace game {


GameHandler::GameHandler() {
    LOG_DEBUG("GameHandler created");

    // Default spells always available
    knownSpells.push_back(6603);  // Attack
    knownSpells.push_back(8690);  // Hearthstone

    // Default action bar layout
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;   // Attack in slot 1
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone in slot 12
}

GameHandler::~GameHandler() {
    disconnect();
}

bool GameHandler::connect(const std::string& host,
                          uint16_t port,
                          const std::vector<uint8_t>& sessionKey,
                          const std::string& accountName,
                          uint32_t build) {

    if (sessionKey.size() != 40) {
        LOG_ERROR("Invalid session key size: ", sessionKey.size(), " (expected 40)");
        fail("Invalid session key");
        return false;
    }

    LOG_INFO("========================================");
    LOG_INFO("   CONNECTING TO WORLD SERVER");
    LOG_INFO("========================================");
    LOG_INFO("Host: ", host);
    LOG_INFO("Port: ", port);
    LOG_INFO("Account: ", accountName);
    LOG_INFO("Build: ", build);

    // Store authentication data
    this->sessionKey = sessionKey;
    this->accountName = accountName;
    this->build = build;

    // Generate random client seed
    this->clientSeed = generateClientSeed();
    LOG_DEBUG("Generated client seed: 0x", std::hex, clientSeed, std::dec);

    // Create world socket
    socket = std::make_unique<network::WorldSocket>();

    // Set up packet callback
    socket->setPacketCallback([this](const network::Packet& packet) {
        network::Packet mutablePacket = packet;
        handlePacket(mutablePacket);
    });

    // Connect to world server
    setState(WorldState::CONNECTING);

    if (!socket->connect(host, port)) {
        LOG_ERROR("Failed to connect to world server");
        fail("Connection failed");
        return false;
    }

    setState(WorldState::CONNECTED);
    LOG_INFO("Connected to world server, waiting for SMSG_AUTH_CHALLENGE...");

    return true;
}

void GameHandler::disconnect() {
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
    activeCharacterGuid_ = 0;
    playerNameCache.clear();
    pendingNameQueries.clear();
    setState(WorldState::DISCONNECTED);
    LOG_INFO("Disconnected from world server");
}

bool GameHandler::isConnected() const {
    return socket && socket->isConnected();
}

void GameHandler::update(float deltaTime) {
    // Fire deferred char-create callback (outside ImGui render)
    if (pendingCharCreateResult_) {
        pendingCharCreateResult_ = false;
        if (charCreateCallback_) {
            charCreateCallback_(pendingCharCreateSuccess_, pendingCharCreateMsg_);
        }
    }

    if (!socket) {
        return;
    }

    // Update socket (processes incoming data and triggers callbacks)
    if (socket) {
        socket->update();
    }

    // Validate target still exists
    if (targetGuid != 0 && !entityManager.hasEntity(targetGuid)) {
        clearTarget();
    }

    // Send periodic heartbeat if in world
    if (state == WorldState::IN_WORLD) {
        timeSinceLastPing += deltaTime;

        if (timeSinceLastPing >= pingInterval) {
            if (socket) {
                sendPing();
            }
            timeSinceLastPing = 0.0f;
        }

        // Update cast timer (Phase 3)
        if (casting && castTimeRemaining > 0.0f) {
            castTimeRemaining -= deltaTime;
            if (castTimeRemaining <= 0.0f) {
                casting = false;
                currentCastSpellId = 0;
                castTimeRemaining = 0.0f;
            }
        }

        // Update spell cooldowns (Phase 3)
        for (auto it = spellCooldowns.begin(); it != spellCooldowns.end(); ) {
            it->second -= deltaTime;
            if (it->second <= 0.0f) {
                it = spellCooldowns.erase(it);
            } else {
                ++it;
            }
        }

        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.cooldownRemaining > 0.0f) {
                slot.cooldownRemaining -= deltaTime;
                if (slot.cooldownRemaining < 0.0f) slot.cooldownRemaining = 0.0f;
            }
        }

        // Update combat text (Phase 2)
        updateCombatText(deltaTime);

        // Update entity movement interpolation (keeps targeting in sync with visuals)
        for (auto& [guid, entity] : entityManager.getEntities()) {
            entity->updateMovement(deltaTime);
        }
    }
}

void GameHandler::handlePacket(network::Packet& packet) {
    if (packet.getSize() < 1) {
        LOG_WARNING("Received empty packet");
        return;
    }

    uint16_t opcode = packet.getOpcode();

    LOG_DEBUG("Received world packet: opcode=0x", std::hex, opcode, std::dec,
              " size=", packet.getSize(), " bytes");

    // Route packet based on opcode
    Opcode opcodeEnum = static_cast<Opcode>(opcode);

    switch (opcodeEnum) {
        case Opcode::SMSG_AUTH_CHALLENGE:
            if (state == WorldState::CONNECTED) {
                handleAuthChallenge(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_CHALLENGE in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_AUTH_RESPONSE:
            if (state == WorldState::AUTH_SENT) {
                handleAuthResponse(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_RESPONSE in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_CHAR_CREATE:
            handleCharCreateResponse(packet);
            break;

        case Opcode::SMSG_CHAR_DELETE: {
            uint8_t result = packet.readUInt8();
            lastCharDeleteResult_ = result;
            bool success = (result == 0x00 || result == 0x47); // Common success codes
            LOG_INFO("SMSG_CHAR_DELETE result: ", (int)result, success ? " (success)" : " (failed)");
            requestCharacterList();
            if (charDeleteCallback_) charDeleteCallback_(success);
            break;
        }

        case Opcode::SMSG_CHAR_ENUM:
            if (state == WorldState::CHAR_LIST_REQUESTED) {
                handleCharEnum(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_CHAR_ENUM in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_LOGIN_VERIFY_WORLD:
            if (state == WorldState::ENTERING_WORLD) {
                handleLoginVerifyWorld(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_LOGIN_VERIFY_WORLD in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_ACCOUNT_DATA_TIMES:
            // Can be received at any time after authentication
            handleAccountDataTimes(packet);
            break;

        case Opcode::SMSG_MOTD:
            // Can be received at any time after entering world
            handleMotd(packet);
            break;

        case Opcode::SMSG_PONG:
            // Can be received at any time after entering world
            handlePong(packet);
            break;

        case Opcode::SMSG_UPDATE_OBJECT:
            LOG_INFO("Received SMSG_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleUpdateObject(packet);
            }
            break;

        case Opcode::SMSG_COMPRESSED_UPDATE_OBJECT:
            LOG_INFO("Received SMSG_COMPRESSED_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Compressed version of UPDATE_OBJECT
            if (state == WorldState::IN_WORLD) {
                handleCompressedUpdateObject(packet);
            }
            break;

        case Opcode::SMSG_DESTROY_OBJECT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleDestroyObject(packet);
            }
            break;

        case Opcode::SMSG_MESSAGECHAT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleMessageChat(packet);
            }
            break;

        case Opcode::SMSG_QUERY_TIME_RESPONSE:
            if (state == WorldState::IN_WORLD) {
                handleQueryTimeResponse(packet);
            }
            break;

        case Opcode::SMSG_PLAYED_TIME:
            if (state == WorldState::IN_WORLD) {
                handlePlayedTime(packet);
            }
            break;

        case Opcode::SMSG_WHO:
            if (state == WorldState::IN_WORLD) {
                handleWho(packet);
            }
            break;

        // ---- Phase 1: Foundation ----
        case Opcode::SMSG_NAME_QUERY_RESPONSE:
            handleNameQueryResponse(packet);
            break;

        case Opcode::SMSG_CREATURE_QUERY_RESPONSE:
            handleCreatureQueryResponse(packet);
            break;

        case Opcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE:
            handleItemQueryResponse(packet);
            break;

        // ---- XP ----
        case Opcode::SMSG_LOG_XPGAIN:
            handleXpGain(packet);
            break;

        // ---- Creature Movement ----
        case Opcode::SMSG_MONSTER_MOVE:
            handleMonsterMove(packet);
            break;

        // ---- Phase 2: Combat ----
        case Opcode::SMSG_ATTACKSTART:
            handleAttackStart(packet);
            break;
        case Opcode::SMSG_ATTACKSTOP:
            handleAttackStop(packet);
            break;
        case Opcode::SMSG_ATTACKERSTATEUPDATE:
            handleAttackerStateUpdate(packet);
            break;
        case Opcode::SMSG_SPELLNONMELEEDAMAGELOG:
            handleSpellDamageLog(packet);
            break;
        case Opcode::SMSG_SPELLHEALLOG:
            handleSpellHealLog(packet);
            break;

        // ---- Phase 3: Spells ----
        case Opcode::SMSG_INITIAL_SPELLS:
            handleInitialSpells(packet);
            break;
        case Opcode::SMSG_CAST_FAILED:
            handleCastFailed(packet);
            break;
        case Opcode::SMSG_SPELL_START:
            handleSpellStart(packet);
            break;
        case Opcode::SMSG_SPELL_GO:
            handleSpellGo(packet);
            break;
        case Opcode::SMSG_SPELL_FAILURE:
            // Spell failed mid-cast
            casting = false;
            currentCastSpellId = 0;
            break;
        case Opcode::SMSG_SPELL_COOLDOWN:
            handleSpellCooldown(packet);
            break;
        case Opcode::SMSG_COOLDOWN_EVENT:
            handleCooldownEvent(packet);
            break;
        case Opcode::SMSG_AURA_UPDATE:
            handleAuraUpdate(packet, false);
            break;
        case Opcode::SMSG_AURA_UPDATE_ALL:
            handleAuraUpdate(packet, true);
            break;
        case Opcode::SMSG_LEARNED_SPELL:
            handleLearnedSpell(packet);
            break;
        case Opcode::SMSG_REMOVED_SPELL:
            handleRemovedSpell(packet);
            break;

        // ---- Phase 4: Group ----
        case Opcode::SMSG_GROUP_INVITE:
            handleGroupInvite(packet);
            break;
        case Opcode::SMSG_GROUP_DECLINE:
            handleGroupDecline(packet);
            break;
        case Opcode::SMSG_GROUP_LIST:
            handleGroupList(packet);
            break;
        case Opcode::SMSG_GROUP_UNINVITE:
            handleGroupUninvite(packet);
            break;
        case Opcode::SMSG_PARTY_COMMAND_RESULT:
            handlePartyCommandResult(packet);
            break;

        // ---- Phase 5: Loot/Gossip/Vendor ----
        case Opcode::SMSG_LOOT_RESPONSE:
            handleLootResponse(packet);
            break;
        case Opcode::SMSG_LOOT_RELEASE_RESPONSE:
            handleLootReleaseResponse(packet);
            break;
        case Opcode::SMSG_LOOT_REMOVED:
            handleLootRemoved(packet);
            break;
        case Opcode::SMSG_GOSSIP_MESSAGE:
            handleGossipMessage(packet);
            break;
        case Opcode::SMSG_GOSSIP_COMPLETE:
            handleGossipComplete(packet);
            break;
        case Opcode::SMSG_LIST_INVENTORY:
            handleListInventory(packet);
            break;

        // Silently ignore common packets we don't handle yet
        case Opcode::SMSG_FEATURE_SYSTEM_STATUS:
        case Opcode::SMSG_SET_FLAT_SPELL_MODIFIER:
        case Opcode::SMSG_SET_PCT_SPELL_MODIFIER:
        case Opcode::SMSG_SPELL_DELAYED:
        case Opcode::SMSG_UPDATE_AURA_DURATION:
        case Opcode::SMSG_PERIODICAURALOG:
        case Opcode::SMSG_SPELLENERGIZELOG:
        case Opcode::SMSG_ENVIRONMENTALDAMAGELOG:
        case Opcode::SMSG_LOOT_MONEY_NOTIFY: {
            // uint32 money + uint8 soleLooter
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t amount = packet.readUInt32();
                playerMoneyCopper_ += amount;
                LOG_INFO("Looted ", amount, " copper (total: ", playerMoneyCopper_, ")");
            }
            break;
        }
        case Opcode::SMSG_LOOT_CLEAR_MONEY:
        case Opcode::SMSG_NPC_TEXT_UPDATE:
            break;
        case Opcode::SMSG_SELL_ITEM: {
            // uint64 vendorGuid, uint64 itemGuid, uint8 result
            if ((packet.getSize() - packet.getReadPos()) >= 17) {
                packet.readUInt64(); // vendorGuid
                packet.readUInt64(); // itemGuid
                uint8_t result = packet.readUInt8();
                if (result != 0) {
                    static const char* sellErrors[] = {
                        "OK", "Can't find item", "Can't sell item",
                        "Can't find vendor", "You don't own that item",
                        "Unknown error", "Only empty bag"
                    };
                    const char* msg = (result < 7) ? sellErrors[result] : "Unknown sell error";
                    addSystemChatMessage(std::string("Sell failed: ") + msg);
                    LOG_WARNING("SMSG_SELL_ITEM error: ", (int)result, " (", msg, ")");
                }
            }
            break;
        }
        case Opcode::SMSG_INVENTORY_CHANGE_FAILURE: {
            if ((packet.getSize() - packet.getReadPos()) >= 1) {
                uint8_t error = packet.readUInt8();
                if (error != 0) {
                    LOG_WARNING("SMSG_INVENTORY_CHANGE_FAILURE: error=", (int)error);
                    // Common error codes
                    std::string msg;
                    switch (error) {
                        case 1: msg = "Item slots occupied."; break;
                        case 4: msg = "Item doesn't go there."; break;
                        case 5: msg = "Bag is full."; break;
                        case 14: msg = "Can't equip that."; break;
                        case 23: msg = "Can't equip with two-handed weapon."; break;
                        case 26: msg = "Inventory full."; break;
                        case 29: msg = "Item is locked."; break;
                        default: msg = "Inventory error (" + std::to_string(error) + ")."; break;
                    }
                    addSystemChatMessage(msg);
                }
            }
            break;
        }
        case Opcode::SMSG_BUY_FAILED:
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
        case Opcode::MSG_RAID_TARGET_UPDATE:
            break;
        case Opcode::SMSG_QUESTGIVER_STATUS: {
            // uint64 npcGuid + uint8 status
            if (packet.getSize() - packet.getReadPos() >= 9) {
                uint64_t npcGuid = packet.readUInt64();
                uint8_t status = packet.readUInt8();
                npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS: guid=0x", std::hex, npcGuid, std::dec, " status=", (int)status);
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_STATUS_MULTIPLE: {
            // uint32 count, then count * (uint64 guid + uint8 status)
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t count = packet.readUInt32();
                for (uint32_t i = 0; i < count; ++i) {
                    if (packet.getSize() - packet.getReadPos() < 9) break;
                    uint64_t npcGuid = packet.readUInt64();
                    uint8_t status = packet.readUInt8();
                    npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                }
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS_MULTIPLE: ", count, " entries");
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_QUEST_DETAILS:
            handleQuestDetails(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_QUEST_COMPLETE: {
            // Mark quest as complete in local log
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t questId = packet.readUInt32();
                for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
                    if (it->questId == questId) {
                        questLog_.erase(it);
                        break;
                    }
                }
            }
            // Re-query all nearby quest giver NPCs so markers refresh
            if (socket) {
                for (const auto& [guid, entity] : entityManager.getEntities()) {
                    if (entity->getType() != ObjectType::UNIT) continue;
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    if (unit->getNpcFlags() & 0x02) {
                        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                        qsPkt.writeUInt64(guid);
                        socket->send(qsPkt);
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_REQUEST_ITEMS:
            handleQuestRequestItems(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_OFFER_REWARD:
            handleQuestOfferReward(packet);
            break;
        case Opcode::SMSG_GROUP_SET_LEADER:
            LOG_DEBUG("Ignoring known opcode: 0x", std::hex, opcode, std::dec);
            break;

        default:
            LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec);
            break;
    }
}

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
        serverSeed
    );

    LOG_DEBUG("CMSG_AUTH_SESSION packet size: ", packet.getSize(), " bytes");

    // Send packet (unencrypted - this is the last unencrypted packet)
    socket->send(packet);

    // Enable encryption IMMEDIATELY after sending AUTH_SESSION
    // AzerothCore enables encryption before sending AUTH_RESPONSE,
    // so we need to be ready to decrypt the response
    LOG_INFO("Enabling encryption immediately after AUTH_SESSION");
    socket->initEncryption(sessionKey);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption enabled, waiting for AUTH_RESPONSE...");
}

void GameHandler::handleAuthResponse(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_RESPONSE");

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
    if (state != WorldState::READY && state != WorldState::AUTHENTICATED &&
        state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot request character list in state: ", (int)state);
        return;
    }

    LOG_INFO("Requesting character list from server...");

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
    if (!CharEnumParser::parse(packet, response)) {
        fail("Failed to parse SMSG_CHAR_ENUM");
        return;
    }

    // Store characters
    characters = response.characters;

    setState(WorldState::CHAR_LIST_RECEIVED);

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
            LOG_INFO("      Level ", (int)character.level);
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

    if (data.result == CharCreateResult::SUCCESS) {
        LOG_INFO("Character created successfully");
        requestCharacterList();
        if (charCreateCallback_) {
            charCreateCallback_(true, "Character created!");
        }
    } else {
        std::string msg;
        switch (data.result) {
            case CharCreateResult::ERROR: msg = "Server error"; break;
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
        if (charDeleteCallback_) charDeleteCallback_(false);
        return;
    }

    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CHAR_DELETE));
    packet.writeUInt64(characterGuid);
    socket->send(packet);
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














void GameHandler::selectCharacter(uint64_t characterGuid) {
    if (state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot select character in state: ", (int)state);
        return;
    }

    LOG_INFO("========================================");
    LOG_INFO("   ENTERING WORLD");
    LOG_INFO("========================================");
    LOG_INFO("Character GUID: 0x", std::hex, characterGuid, std::dec);

    // Find character name for logging
    for (const auto& character : characters) {
        if (character.guid == characterGuid) {
            LOG_INFO("Character: ", character.name);
            LOG_INFO("Level ", (int)character.level, " ",
                     getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            break;
        }
    }

    // Store player GUID
    playerGuid = characterGuid;

    // Build CMSG_PLAYER_LOGIN packet
    auto packet = PlayerLoginPacket::build(characterGuid);

    // Send packet
    socket->send(packet);

    setState(WorldState::ENTERING_WORLD);
    LOG_INFO("CMSG_PLAYER_LOGIN sent, entering world...");
}

void GameHandler::handleLoginVerifyWorld(network::Packet& packet) {
    LOG_INFO("Handling SMSG_LOGIN_VERIFY_WORLD");

    LoginVerifyWorldData data;
    if (!LoginVerifyWorldParser::parse(packet, data)) {
        fail("Failed to parse SMSG_LOGIN_VERIFY_WORLD");
        return;
    }

    if (!data.isValid()) {
        fail("Invalid world entry data");
        return;
    }

    // Successfully entered the world!
    setState(WorldState::IN_WORLD);

    LOG_INFO("========================================");
    LOG_INFO("   SUCCESSFULLY ENTERED WORLD!");
    LOG_INFO("========================================");
    LOG_INFO("Map ID: ", data.mapId);
    LOG_INFO("Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("Orientation: ", data.orientation, " radians");
    LOG_INFO("Player is now in the game world");
    addSystemChatMessage("You have entered the world.");

    // Initialize movement info with world entry position (server → canonical)
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(data.x, data.y, data.z));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = data.orientation;
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    movementInfo.time = 0;

    // Send CMSG_SET_ACTIVE_MOVER (required by some servers)
    if (playerGuid != 0 && socket) {
        auto activeMoverPacket = SetActiveMoverPacket::build(playerGuid);
        socket->send(activeMoverPacket);
        LOG_INFO("Sent CMSG_SET_ACTIVE_MOVER for player 0x", std::hex, playerGuid, std::dec);
    }

    // Notify application to load terrain for this map/position (online mode)
    if (worldEntryCallback_) {
        worldEntryCallback_(data.mapId, data.x, data.y, data.z);
    }
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
    LOG_INFO("Handling SMSG_MOTD");

    MotdData data;
    if (!MotdParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MOTD");
        return;
    }

    if (!data.isEmpty()) {
        LOG_INFO("========================================");
        LOG_INFO("   MESSAGE OF THE DAY");
        LOG_INFO("========================================");
        for (const auto& line : data.lines) {
            LOG_INFO(line);
            addSystemChatMessage(std::string("MOTD: ") + line);
        }
        LOG_INFO("========================================");
    }
}

void GameHandler::sendPing() {
    if (state != WorldState::IN_WORLD) {
        return;
    }

    // Increment sequence number
    pingSequence++;

    LOG_DEBUG("Sending CMSG_PING (heartbeat)");
    LOG_DEBUG("  Sequence: ", pingSequence);

    // Build and send ping packet
    auto packet = PingPacket::build(pingSequence, lastLatency);
    socket->send(packet);
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

    LOG_DEBUG("Heartbeat acknowledged (sequence: ", data.sequence, ")");
}

void GameHandler::sendMovement(Opcode opcode) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send movement in state: ", (int)state);
        return;
    }

    // Use real millisecond timestamp (server validates for anti-cheat)
    static auto startTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    movementInfo.time = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count());

    // Update movement flags based on opcode
    switch (opcode) {
        case Opcode::CMSG_MOVE_START_FORWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FORWARD);
            break;
        case Opcode::CMSG_MOVE_START_BACKWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::BACKWARD);
            break;
        case Opcode::CMSG_MOVE_STOP:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                    static_cast<uint32_t>(MovementFlags::BACKWARD));
            break;
        case Opcode::CMSG_MOVE_START_STRAFE_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_LEFT);
            break;
        case Opcode::CMSG_MOVE_START_STRAFE_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
            break;
        case Opcode::CMSG_MOVE_STOP_STRAFE:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT));
            break;
        case Opcode::CMSG_MOVE_JUMP:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::CMSG_MOVE_START_TURN_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_LEFT);
            break;
        case Opcode::CMSG_MOVE_START_TURN_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_RIGHT);
            break;
        case Opcode::CMSG_MOVE_STOP_TURN:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
            break;
        case Opcode::CMSG_MOVE_FALL_LAND:
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::CMSG_MOVE_HEARTBEAT:
            // No flag changes — just sends current position
            break;
        default:
            break;
    }

    LOG_DEBUG("Sending movement packet: opcode=0x", std::hex,
              static_cast<uint16_t>(opcode), std::dec);

    // Convert canonical → server coordinates for the wire
    MovementInfo wireInfo = movementInfo;
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wireInfo.x, wireInfo.y, wireInfo.z));
    wireInfo.x = serverPos.x;
    wireInfo.y = serverPos.y;
    wireInfo.z = serverPos.z;

    // Build and send movement packet
    auto packet = MovementPacket::build(opcode, wireInfo, playerGuid);
    socket->send(packet);
}

void GameHandler::setPosition(float x, float y, float z) {
    movementInfo.x = x;
    movementInfo.y = y;
    movementInfo.z = z;
}

void GameHandler::setOrientation(float orientation) {
    movementInfo.orientation = orientation;
}

void GameHandler::handleUpdateObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_UPDATE_OBJECT");

    UpdateObjectData data;
    if (!UpdateObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_UPDATE_OBJECT");
        return;
    }

    // Process out-of-range objects first
    for (uint64_t guid : data.outOfRangeGuids) {
        if (entityManager.hasEntity(guid)) {
            LOG_INFO("Entity went out of range: 0x", std::hex, guid, std::dec);
            // Trigger creature despawn callback before removing entity
            if (creatureDespawnCallback_) {
                creatureDespawnCallback_(guid);
            }
            entityManager.removeEntity(guid);
        }
    }

    // Process update blocks
    for (const auto& block : data.blocks) {
        switch (block.updateType) {
            case UpdateType::CREATE_OBJECT:
            case UpdateType::CREATE_OBJECT2: {
                // Create new entity
                std::shared_ptr<Entity> entity;

                switch (block.objectType) {
                    case ObjectType::PLAYER:
                        entity = std::make_shared<Player>(block.guid);
                        LOG_INFO("Created player entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    case ObjectType::UNIT:
                        entity = std::make_shared<Unit>(block.guid);
                        LOG_INFO("Created unit entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    case ObjectType::GAMEOBJECT:
                        entity = std::make_shared<GameObject>(block.guid);
                        LOG_INFO("Created gameobject entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    default:
                        entity = std::make_shared<Entity>(block.guid);
                        entity->setType(block.objectType);
                        LOG_INFO("Created generic entity: 0x", std::hex, block.guid, std::dec,
                                 ", type=", static_cast<int>(block.objectType));
                        break;
                }

                // Set position from movement block (server → canonical)
                if (block.hasMovement) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("  Position: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                }

                // Set fields
                for (const auto& field : block.fields) {
                    entity->setField(field.first, field.second);
                }

                // Add to manager
                entityManager.addEntity(block.guid, entity);

                // Auto-query names (Phase 1)
                if (block.objectType == ObjectType::PLAYER) {
                    queryPlayerName(block.guid);
                } else if (block.objectType == ObjectType::UNIT) {
                    // Extract creature entry from fields (UNIT_FIELD_ENTRY = index 54 in 3.3.5a,
                    // but the OBJECT_FIELD_ENTRY is at index 3)
                    auto it = block.fields.find(3); // OBJECT_FIELD_ENTRY
                    if (it != block.fields.end() && it->second != 0) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        unit->setEntry(it->second);
                        // Set name from cache immediately if available
                        std::string cached = getCachedCreatureName(it->second);
                        if (!cached.empty()) {
                            unit->setName(cached);
                        }
                        queryCreatureInfo(it->second, block.guid);
                    }
                }

                // Extract health/mana/power from fields (Phase 2) — single pass
                if (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) {
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    for (const auto& [key, val] : block.fields) {
                        switch (key) {
                            case 24:
                                unit->setHealth(val);
                                // Detect dead player on login
                                if (block.guid == playerGuid && val == 0) {
                                    playerDead_ = true;
                                    LOG_INFO("Player logged in dead");
                                }
                                break;
                            case 25: unit->setPower(val); break;
                            case 32: unit->setMaxHealth(val); break;
                            case 33: unit->setMaxPower(val); break;
                            case 55: unit->setFactionTemplate(val); break; // UNIT_FIELD_FACTIONTEMPLATE
                            case 59: unit->setUnitFlags(val); break;   // UNIT_FIELD_FLAGS
                            case 54: unit->setLevel(val); break;
                            case 67: unit->setDisplayId(val); break;  // UNIT_FIELD_DISPLAYID
                            case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                            default: break;
                        }
                    }
                    // Determine hostility from faction template for online creatures
                    if (unit->getFactionTemplate() != 0) {
                        unit->setHostile(isHostileFaction(unit->getFactionTemplate()));
                    }
                    // Trigger creature spawn callback for units with displayId
                    if (block.objectType == ObjectType::UNIT && unit->getDisplayId() != 0) {
                        if (creatureSpawnCallback_) {
                            creatureSpawnCallback_(block.guid, unit->getDisplayId(),
                                unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                        }
                        // Query quest giver status for NPCs with questgiver flag (0x02)
                        if ((unit->getNpcFlags() & 0x02) && socket) {
                            network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                            qsPkt.writeUInt64(block.guid);
                            socket->send(qsPkt);
                        }
                    }
                }
                // Track online item objects
                if (block.objectType == ObjectType::ITEM) {
                    auto entryIt = block.fields.find(3);  // OBJECT_FIELD_ENTRY
                    auto stackIt = block.fields.find(14); // ITEM_FIELD_STACK_COUNT
                    if (entryIt != block.fields.end() && entryIt->second != 0) {
                        OnlineItemInfo info;
                        info.entry = entryIt->second;
                        info.stackCount = (stackIt != block.fields.end()) ? stackIt->second : 1;
                        onlineItems_[block.guid] = info;
                        queryItemInfo(info.entry, block.guid);
                    }
                }

                // Extract XP / inventory slot fields for player entity
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    lastPlayerFields_ = block.fields;
                    detectInventorySlotBases(block.fields);
                    bool slotsChanged = false;
                    for (const auto& [key, val] : block.fields) {
                        if (key == 634) { playerXp_ = val; }                // PLAYER_XP
                        else if (key == 635) { playerNextLevelXp_ = val; }  // PLAYER_NEXT_LEVEL_XP
                        else if (key == 54) {
                            serverPlayerLevel_ = val;                        // UNIT_FIELD_LEVEL
                            for (auto& ch : characters) {
                                if (ch.guid == playerGuid) { ch.level = val; break; }
                            }
                        }
                        else if (key == 1170) { playerMoneyCopper_ = val; }  // PLAYER_FIELD_COINAGE
                    }
                    if (applyInventoryFields(block.fields)) slotsChanged = true;
                    if (slotsChanged) rebuildOnlineInventory();
                }
                break;
            }

            case UpdateType::VALUES: {
                // Update existing entity fields
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    for (const auto& field : block.fields) {
                        entity->setField(field.first, field.second);
                    }

                    // Update cached health/mana/power values (Phase 2) — single pass
                    if (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        for (const auto& [key, val] : block.fields) {
                            switch (key) {
                                case 24: {
                                    uint32_t oldHealth = unit->getHealth();
                                    unit->setHealth(val);
                                    if (val == 0) {
                                        if (block.guid == autoAttackTarget) {
                                            stopAutoAttack();
                                        }
                                        hostileAttackers_.erase(block.guid);
                                        // Player death
                                        if (block.guid == playerGuid) {
                                            playerDead_ = true;
                                            stopAutoAttack();
                                            LOG_INFO("Player died!");
                                        }
                                        // Trigger death animation for NPC units
                                        if (entity->getType() == ObjectType::UNIT && npcDeathCallback_) {
                                            npcDeathCallback_(block.guid);
                                        }
                                    } else if (oldHealth == 0 && val > 0) {
                                        // Player resurrection
                                        if (block.guid == playerGuid) {
                                            playerDead_ = false;
                                            LOG_INFO("Player resurrected!");
                                        }
                                        // Respawn: health went from 0 to >0, reset animation
                                        if (entity->getType() == ObjectType::UNIT && npcRespawnCallback_) {
                                            npcRespawnCallback_(block.guid);
                                        }
                                    }
                                    break;
                                }
                                case 25: unit->setPower(val); break;
                                case 32: unit->setMaxHealth(val); break;
                                case 33: unit->setMaxPower(val); break;
                                case 59: unit->setUnitFlags(val); break;   // UNIT_FIELD_FLAGS
                                case 54: unit->setLevel(val); break;
                                case 55:  // UNIT_FIELD_FACTIONTEMPLATE
                                    unit->setFactionTemplate(val);
                                    unit->setHostile(isHostileFaction(val));
                                    break;
                                case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                                default: break;
                            }
                        }
                    }
                    // Update XP / inventory slot fields for player entity
                    if (block.guid == playerGuid) {
                        for (const auto& [key, val] : block.fields) {
                            lastPlayerFields_[key] = val;
                        }
                        detectInventorySlotBases(block.fields);
                        bool slotsChanged = false;
                        for (const auto& [key, val] : block.fields) {
                            if (key == 634) {
                                playerXp_ = val;
                                LOG_INFO("XP updated: ", val);
                            }
                            else if (key == 635) {
                                playerNextLevelXp_ = val;
                                LOG_INFO("Next level XP updated: ", val);
                            }
                            else if (key == 54) {
                                serverPlayerLevel_ = val;
                                LOG_INFO("Level updated: ", val);
                                // Update Character struct for character selection screen
                                for (auto& ch : characters) {
                                    if (ch.guid == playerGuid) {
                                        ch.level = val;
                                        break;
                                    }
                                }
                            }
                            else if (key == 1170) {
                                playerMoneyCopper_ = val;
                                LOG_INFO("Money updated via VALUES: ", val, " copper");
                            }
                        }
                        if (applyInventoryFields(block.fields)) slotsChanged = true;
                        if (slotsChanged) rebuildOnlineInventory();
                    }

                    // Update item stack count for online items
                    if (entity->getType() == ObjectType::ITEM) {
                        for (const auto& [key, val] : block.fields) {
                            if (key == 14) { // ITEM_FIELD_STACK_COUNT
                                auto it = onlineItems_.find(block.guid);
                                if (it != onlineItems_.end()) it->second.stackCount = val;
                            }
                        }
                        rebuildOnlineInventory();
                    }

                    LOG_DEBUG("Updated entity fields: 0x", std::hex, block.guid, std::dec);
                } else {
                    LOG_WARNING("VALUES update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            case UpdateType::MOVEMENT: {
                // Update entity position (server → canonical)
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("Updated entity position: 0x", std::hex, block.guid, std::dec);
                } else {
                    LOG_WARNING("MOVEMENT update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            default:
                break;
        }
    }

    tabCycleStale = true;
    LOG_INFO("Entity count: ", entityManager.getEntityCount());

    // Late inventory base detection once items are known
    if (playerGuid != 0 && invSlotBase_ < 0 && !lastPlayerFields_.empty() && !onlineItems_.empty()) {
        detectInventorySlotBases(lastPlayerFields_);
        if (invSlotBase_ >= 0) {
            if (applyInventoryFields(lastPlayerFields_)) {
                rebuildOnlineInventory();
            }
        }
    }
}

void GameHandler::handleCompressedUpdateObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_COMPRESSED_UPDATE_OBJECT, packet size: ", packet.getSize());

    // First 4 bytes = decompressed size
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_COMPRESSED_UPDATE_OBJECT too small");
        return;
    }

    uint32_t decompressedSize = packet.readUInt32();
    LOG_INFO("  Decompressed size: ", decompressedSize);

    if (decompressedSize == 0 || decompressedSize > 1024 * 1024) {
        LOG_WARNING("Invalid decompressed size: ", decompressedSize);
        return;
    }

    // Remaining data is zlib compressed
    size_t compressedSize = packet.getSize() - packet.getReadPos();
    const uint8_t* compressedData = packet.getData().data() + packet.getReadPos();

    // Decompress
    std::vector<uint8_t> decompressed(decompressedSize);
    uLongf destLen = decompressedSize;
    int ret = uncompress(decompressed.data(), &destLen, compressedData, compressedSize);

    if (ret != Z_OK) {
        LOG_WARNING("Failed to decompress UPDATE_OBJECT: zlib error ", ret);
        return;
    }

    LOG_DEBUG("  Decompressed ", compressedSize, " -> ", destLen, " bytes");

    // Create packet from decompressed data and parse it
    network::Packet decompressedPacket(static_cast<uint16_t>(Opcode::SMSG_UPDATE_OBJECT), decompressed);
    handleUpdateObject(decompressedPacket);
}

void GameHandler::handleDestroyObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_DESTROY_OBJECT");

    DestroyObjectData data;
    if (!DestroyObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_DESTROY_OBJECT");
        return;
    }

    // Remove entity
    if (entityManager.hasEntity(data.guid)) {
        entityManager.removeEntity(data.guid);
        LOG_INFO("Destroyed entity: 0x", std::hex, data.guid, std::dec,
                 " (", (data.isDeath ? "death" : "despawn"), ")");
    } else {
        LOG_WARNING("Destroy object for unknown entity: 0x", std::hex, data.guid, std::dec);
    }

    // Clean up auto-attack and target if destroyed entity was our target
    if (data.guid == autoAttackTarget) {
        stopAutoAttack();
    }
    if (data.guid == targetGuid) {
        targetGuid = 0;
    }
    hostileAttackers_.erase(data.guid);

    // Remove online item tracking
    if (onlineItems_.erase(data.guid)) {
        rebuildOnlineInventory();
    }

    // Clean up quest giver status
    npcQuestStatus_.erase(data.guid);

    tabCycleStale = true;
    LOG_INFO("Entity count: ", entityManager.getEntityCount());
}

void GameHandler::sendChatMessage(ChatType type, const std::string& message, const std::string& target) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send chat in state: ", (int)state);
        return;
    }

    if (message.empty()) {
        LOG_WARNING("Cannot send empty chat message");
        return;
    }

    LOG_INFO("Sending chat message: [", getChatTypeString(type), "] ", message);

    // Determine language based on character (for now, use COMMON)
    ChatLanguage language = ChatLanguage::COMMON;

    // Build and send packet
    auto packet = MessageChatPacket::build(type, language, message, target);
    socket->send(packet);
}

void GameHandler::handleMessageChat(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_MESSAGECHAT");

    MessageChatData data;
    if (!MessageChatParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MESSAGECHAT");
        return;
    }

    // Add to chat history
    chatHistory.push_back(data);

    // Limit chat history size
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.erase(chatHistory.begin());
    }

    // Log the message
    std::string senderInfo;
    if (!data.senderName.empty()) {
        senderInfo = data.senderName;
    } else if (data.senderGuid != 0) {
        // Try to find entity name
        auto entity = entityManager.getEntity(data.senderGuid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::dynamic_pointer_cast<Player>(entity);
            if (player && !player->getName().empty()) {
                senderInfo = player->getName();
            } else {
                senderInfo = "Player-" + std::to_string(data.senderGuid);
            }
        } else {
            senderInfo = "Unknown-" + std::to_string(data.senderGuid);
        }
    } else {
        senderInfo = "System";
    }

    std::string channelInfo;
    if (!data.channelName.empty()) {
        channelInfo = "[" + data.channelName + "] ";
    }

    LOG_INFO("========================================");
    LOG_INFO(" CHAT [", getChatTypeString(data.type), "]");
    LOG_INFO("========================================");
    LOG_INFO(channelInfo, senderInfo, ": ", data.message);
    LOG_INFO("========================================");
}

void GameHandler::setTarget(uint64_t guid) {
    if (guid == targetGuid) return;
    targetGuid = guid;

    // Inform server of target selection (Phase 1)
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = SetSelectionPacket::build(guid);
        socket->send(packet);
    }

    if (guid != 0) {
        LOG_INFO("Target set: 0x", std::hex, guid, std::dec);
    }
}

void GameHandler::clearTarget() {
    if (targetGuid != 0) {
        LOG_INFO("Target cleared");
    }
    targetGuid = 0;
    tabCycleIndex = -1;
    tabCycleStale = true;
}

std::shared_ptr<Entity> GameHandler::getTarget() const {
    if (targetGuid == 0) return nullptr;
    return entityManager.getEntity(targetGuid);
}

void GameHandler::inspectTarget() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot inspect: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to inspect.");
        return;
    }

    auto target = getTarget();
    if (!target || target->getType() != ObjectType::PLAYER) {
        addSystemChatMessage("You can only inspect players.");
        return;
    }

    auto packet = InspectPacket::build(targetGuid);
    socket->send(packet);

    auto player = std::static_pointer_cast<Player>(target);
    std::string name = player->getName().empty() ? "Target" : player->getName();
    addSystemChatMessage("Inspecting " + name + "...");
    LOG_INFO("Sent inspect request for player: ", name, " (GUID: 0x", std::hex, targetGuid, std::dec, ")");
}

void GameHandler::queryServerTime() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot query time: not in world or not connected");
        return;
    }

    auto packet = QueryTimePacket::build();
    socket->send(packet);
    LOG_INFO("Requested server time");
}

void GameHandler::requestPlayedTime() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request played time: not in world or not connected");
        return;
    }

    auto packet = RequestPlayedTimePacket::build(true);
    socket->send(packet);
    LOG_INFO("Requested played time");
}

void GameHandler::queryWho(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot query who: not in world or not connected");
        return;
    }

    auto packet = WhoPacket::build(0, 0, playerName);
    socket->send(packet);
    LOG_INFO("Sent WHO query", playerName.empty() ? "" : " for: " + playerName);
}

void GameHandler::releaseSpirit() {
    if (!playerDead_) return;
    if (socket && state == WorldState::IN_WORLD) {
        auto packet = RepopRequestPacket::build();
        socket->send(packet);
        LOG_INFO("Sent CMSG_REPOP_REQUEST (Release Spirit)");
    }
}

void GameHandler::activateSpiritHealer(uint64_t npcGuid) {
    if (!playerDead_) return;
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SpiritHealerActivatePacket::build(npcGuid);
    socket->send(packet);
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE to 0x", std::hex, npcGuid, std::dec);
}

void GameHandler::tabTarget(float playerX, float playerY, float playerZ) {
    // Rebuild cycle list if stale
    if (tabCycleStale) {
        tabCycleList.clear();
        tabCycleIndex = -1;

        struct EntityDist {
            uint64_t guid;
            float distance;
        };
        std::vector<EntityDist> sortable;

        for (const auto& [guid, entity] : entityManager.getEntities()) {
            auto t = entity->getType();
            if (t != ObjectType::UNIT && t != ObjectType::PLAYER) continue;
            if (guid == playerGuid) continue;  // Don't tab-target self
            float dx = entity->getX() - playerX;
            float dy = entity->getY() - playerY;
            float dz = entity->getZ() - playerZ;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            sortable.push_back({guid, dist});
        }

        std::sort(sortable.begin(), sortable.end(),
                  [](const EntityDist& a, const EntityDist& b) { return a.distance < b.distance; });

        for (const auto& ed : sortable) {
            tabCycleList.push_back(ed.guid);
        }
        tabCycleStale = false;
    }

    if (tabCycleList.empty()) {
        clearTarget();
        return;
    }

    tabCycleIndex = (tabCycleIndex + 1) % static_cast<int>(tabCycleList.size());
    setTarget(tabCycleList[tabCycleIndex]);
}

void GameHandler::addLocalChatMessage(const MessageChatData& msg) {
    chatHistory.push_back(msg);
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.pop_front();
    }
}

// ============================================================
// Phase 1: Name Queries
// ============================================================

void GameHandler::queryPlayerName(uint64_t guid) {
    if (playerNameCache.count(guid) || pendingNameQueries.count(guid)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingNameQueries.insert(guid);
    auto packet = NameQueryPacket::build(guid);
    socket->send(packet);
}

void GameHandler::queryCreatureInfo(uint32_t entry, uint64_t guid) {
    if (creatureInfoCache.count(entry) || pendingCreatureQueries.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingCreatureQueries.insert(entry);
    auto packet = CreatureQueryPacket::build(entry, guid);
    socket->send(packet);
}

std::string GameHandler::getCachedPlayerName(uint64_t guid) const {
    auto it = playerNameCache.find(guid);
    return (it != playerNameCache.end()) ? it->second : "";
}

std::string GameHandler::getCachedCreatureName(uint32_t entry) const {
    auto it = creatureInfoCache.find(entry);
    return (it != creatureInfoCache.end()) ? it->second.name : "";
}

void GameHandler::handleNameQueryResponse(network::Packet& packet) {
    NameQueryResponseData data;
    if (!NameQueryResponseParser::parse(packet, data)) return;

    pendingNameQueries.erase(data.guid);

    if (data.isValid()) {
        playerNameCache[data.guid] = data.name;
        // Update entity name
        auto entity = entityManager.getEntity(data.guid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<Player>(entity);
            player->setName(data.name);
        }
    }
}

void GameHandler::handleCreatureQueryResponse(network::Packet& packet) {
    CreatureQueryResponseData data;
    if (!CreatureQueryResponseParser::parse(packet, data)) return;

    pendingCreatureQueries.erase(data.entry);

    if (data.isValid()) {
        creatureInfoCache[data.entry] = data;
        // Update all unit entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(entity);
                if (unit->getEntry() == data.entry) {
                    unit->setName(data.name);
                }
            }
        }
    }
}

// ============================================================
// Item Query
// ============================================================

void GameHandler::queryItemInfo(uint32_t entry, uint64_t guid) {
    if (itemInfoCache_.count(entry) || pendingItemQueries_.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingItemQueries_.insert(entry);
    auto packet = ItemQueryPacket::build(entry, guid);
    socket->send(packet);
}

void GameHandler::handleItemQueryResponse(network::Packet& packet) {
    ItemQueryResponseData data;
    if (!ItemQueryResponseParser::parse(packet, data)) return;

    pendingItemQueries_.erase(data.entry);

    if (data.valid) {
        itemInfoCache_[data.entry] = data;
        rebuildOnlineInventory();
    }
}

uint64_t GameHandler::resolveOnlineItemGuid(uint32_t itemId) const {
    if (itemId == 0) return 0;
    uint64_t found = 0;
    for (const auto& [guid, info] : onlineItems_) {
        if (info.entry != itemId) continue;
        if (found != 0) {
            return 0; // Ambiguous
        }
        found = guid;
    }
    return found;
}

void GameHandler::detectInventorySlotBases(const std::map<uint16_t, uint32_t>& fields) {
    if (invSlotBase_ >= 0 && packSlotBase_ >= 0) return;
    if (onlineItems_.empty() || fields.empty()) return;

    std::vector<uint16_t> matchingPairs;
    matchingPairs.reserve(32);

    for (const auto& [idx, low] : fields) {
        if ((idx % 2) != 0) continue;
        auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
        if (itHigh == fields.end()) continue;
        uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
        if (guid == 0) continue;
        if (onlineItems_.count(guid)) {
            matchingPairs.push_back(idx);
        }
    }

    if (matchingPairs.empty()) return;
    std::sort(matchingPairs.begin(), matchingPairs.end());

    if (invSlotBase_ < 0) {
        // The lowest matching field is the first EQUIPPED slot (not necessarily HEAD).
        // With 2+ matches we can derive the true base: all matches must be at
        // even offsets from the base, spaced 2 fields per slot.
        // Use the known 3.3.5a default (324) and verify matches align to it.
        constexpr int knownBase = 324;
        constexpr int slotStride = 2;
        bool allAlign = true;
        for (uint16_t p : matchingPairs) {
            if (p < knownBase || (p - knownBase) % slotStride != 0) {
                allAlign = false;
                break;
            }
        }
        if (allAlign) {
            invSlotBase_ = knownBase;
        } else {
            // Fallback: if we have 2+ matches, derive base from their spacing
            if (matchingPairs.size() >= 2) {
                uint16_t lo = matchingPairs[0];
                // lo must be base + 2*slotN, and slotN is 0..22
                // Try each possible slot for 'lo' and see if all others also land on valid slots
                for (int s = 0; s <= 22; s++) {
                    int candidate = lo - s * slotStride;
                    if (candidate < 0) break;
                    bool ok = true;
                    for (uint16_t p : matchingPairs) {
                        int off = p - candidate;
                        if (off < 0 || off % slotStride != 0 || off / slotStride > 22) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        invSlotBase_ = candidate;
                        break;
                    }
                }
                if (invSlotBase_ < 0) invSlotBase_ = knownBase;
            } else {
                invSlotBase_ = knownBase;
            }
        }
        packSlotBase_ = invSlotBase_ + (game::Inventory::NUM_EQUIP_SLOTS * 2);
        LOG_INFO("Detected inventory field base: equip=", invSlotBase_,
                 " pack=", packSlotBase_);
    }
}

bool GameHandler::applyInventoryFields(const std::map<uint16_t, uint32_t>& fields) {
    bool slotsChanged = false;
    // WoW 3.3.5a: PLAYER_FIELD_INV_SLOT_HEAD = UNIT_END + 0x00B0 = 324
    //             PLAYER_FIELD_PACK_SLOT_1  = UNIT_END + 0x00DE = 370
    int equipBase = (invSlotBase_ >= 0) ? invSlotBase_ : 324;
    int packBase = (packSlotBase_ >= 0) ? packSlotBase_ : 370;

    for (const auto& [key, val] : fields) {
        if (key >= equipBase && key <= equipBase + (game::Inventory::NUM_EQUIP_SLOTS * 2 - 1)) {
            int slotIndex = (key - equipBase) / 2;
            bool isLow = ((key - equipBase) % 2 == 0);
            if (slotIndex < static_cast<int>(equipSlotGuids_.size())) {
                uint64_t& guid = equipSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        } else if (key >= packBase && key <= packBase + (game::Inventory::BACKPACK_SLOTS * 2 - 1)) {
            int slotIndex = (key - packBase) / 2;
            bool isLow = ((key - packBase) % 2 == 0);
            if (slotIndex < static_cast<int>(backpackSlotGuids_.size())) {
                uint64_t& guid = backpackSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }
    }

    return slotsChanged;
}

void GameHandler::rebuildOnlineInventory() {

    inventory = Inventory();

    // Equipment slots
    for (int i = 0; i < 23; i++) {
        uint64_t guid = equipSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        inventory.setEquipSlot(static_cast<EquipSlot>(i), def);
    }

    // Backpack slots
    for (int i = 0; i < 16; i++) {
        uint64_t guid = backpackSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        inventory.setBackpackSlot(i, def);
    }

    onlineEquipDirty_ = true;

    LOG_DEBUG("Rebuilt online inventory: equip=", [&](){
        int c = 0; for (auto g : equipSlotGuids_) if (g) c++; return c;
    }(), " backpack=", [&](){
        int c = 0; for (auto g : backpackSlotGuids_) if (g) c++; return c;
    }());
}

// ============================================================
// Phase 2: Combat
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    autoAttacking = true;
    autoAttackTarget = targetGuid;
    autoAttackOutOfRange_ = false;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackSwingPacket::build(targetGuid);
        socket->send(packet);
    }
    LOG_INFO("Starting auto-attack on 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopAutoAttack() {
    if (!autoAttacking) return;
    autoAttacking = false;
    autoAttackTarget = 0;
    autoAttackOutOfRange_ = false;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackStopPacket::build();
        socket->send(packet);
    }
    LOG_INFO("Stopping auto-attack");
}

void GameHandler::addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId, bool isPlayerSource) {
    CombatTextEntry entry;
    entry.type = type;
    entry.amount = amount;
    entry.spellId = spellId;
    entry.age = 0.0f;
    entry.isPlayerSource = isPlayerSource;
    combatText.push_back(entry);
}

void GameHandler::updateCombatText(float deltaTime) {
    for (auto& entry : combatText) {
        entry.age += deltaTime;
    }
    combatText.erase(
        std::remove_if(combatText.begin(), combatText.end(),
                       [](const CombatTextEntry& e) { return e.isExpired(); }),
        combatText.end());
}

void GameHandler::handleAttackStart(network::Packet& packet) {
    AttackStartData data;
    if (!AttackStartParser::parse(packet, data)) return;

    if (data.attackerGuid == playerGuid) {
        autoAttacking = true;
        autoAttackTarget = data.victimGuid;
    }
}

void GameHandler::handleAttackStop(network::Packet& packet) {
    AttackStopData data;
    if (!AttackStopParser::parse(packet, data)) return;

    // Don't clear autoAttacking on SMSG_ATTACKSTOP - the server sends this
    // when the attack loop pauses (out of range, etc). The player's intent
    // to attack persists until target dies or player explicitly cancels.
    // We'll re-send CMSG_ATTACKSWING periodically in the update loop.
    if (data.attackerGuid == playerGuid) {
        LOG_DEBUG("SMSG_ATTACKSTOP received (keeping auto-attack intent)");
    } else if (data.victimGuid == playerGuid) {
        hostileAttackers_.erase(data.attackerGuid);
    }
}

void GameHandler::handleMonsterMove(network::Packet& packet) {
    MonsterMoveData data;
    if (!MonsterMoveParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MONSTER_MOVE");
        return;
    }

    // Update entity position in entity manager
    auto entity = entityManager.getEntity(data.guid);
    if (entity) {
        if (data.hasDest) {
            // Convert destination from server to canonical coords
            glm::vec3 destCanonical = core::coords::serverToCanonical(
                glm::vec3(data.destX, data.destY, data.destZ));

            // Calculate facing angle
            float orientation = entity->getOrientation();
            if (data.moveType == 4) {
                // FacingAngle - server specifies exact angle
                orientation = data.facingAngle;
            } else if (data.moveType == 3) {
                // FacingTarget - face toward the target entity
                auto target = entityManager.getEntity(data.facingTarget);
                if (target) {
                    float dx = target->getX() - entity->getX();
                    float dy = target->getY() - entity->getY();
                    if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                        orientation = std::atan2(dy, dx);
                    }
                }
            } else {
                // Normal move - face toward destination
                float dx = destCanonical.x - entity->getX();
                float dy = destCanonical.y - entity->getY();
                if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                    orientation = std::atan2(dy, dx);
                }
            }

            // Interpolate entity position alongside renderer (so targeting matches visual)
            entity->startMoveTo(destCanonical.x, destCanonical.y, destCanonical.z,
                                orientation, data.duration / 1000.0f);

            // Notify renderer to smoothly move the creature
            if (creatureMoveCallback_) {
                creatureMoveCallback_(data.guid,
                    destCanonical.x, destCanonical.y, destCanonical.z,
                    data.duration);
            }
        } else if (data.moveType == 1) {
            // Stop at current position
            glm::vec3 posCanonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            entity->setPosition(posCanonical.x, posCanonical.y, posCanonical.z,
                                entity->getOrientation());

            if (creatureMoveCallback_) {
                creatureMoveCallback_(data.guid,
                    posCanonical.x, posCanonical.y, posCanonical.z, 0);
            }
        }
    }
}

void GameHandler::handleAttackerStateUpdate(network::Packet& packet) {
    AttackerStateUpdateData data;
    if (!AttackerStateUpdateParser::parse(packet, data)) return;

    bool isPlayerAttacker = (data.attackerGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);
    if (isPlayerAttacker && meleeSwingCallback_) {
        meleeSwingCallback_();
    }
    if (!isPlayerAttacker && npcSwingCallback_) {
        npcSwingCallback_(data.attackerGuid);
    }

    if (isPlayerTarget && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
    }

    if (data.isMiss()) {
        addCombatText(CombatTextEntry::MISS, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 1) {
        addCombatText(CombatTextEntry::DODGE, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 2) {
        addCombatText(CombatTextEntry::PARRY, 0, 0, isPlayerAttacker);
    } else {
        auto type = data.isCrit() ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::MELEE_DAMAGE;
        addCombatText(type, data.totalDamage, 0, isPlayerAttacker);
    }

    (void)isPlayerTarget; // Used for future incoming damage display
}

void GameHandler::handleSpellDamageLog(network::Packet& packet) {
    SpellDamageLogData data;
    if (!SpellDamageLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.attackerGuid == playerGuid);
    auto type = data.isCrit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::SPELL_DAMAGE;
    addCombatText(type, static_cast<int32_t>(data.damage), data.spellId, isPlayerSource);
}

void GameHandler::handleSpellHealLog(network::Packet& packet) {
    SpellHealLogData data;
    if (!SpellHealLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.casterGuid == playerGuid);
    auto type = data.isCrit ? CombatTextEntry::CRIT_HEAL : CombatTextEntry::HEAL;
    addCombatText(type, static_cast<int32_t>(data.heal), data.spellId, isPlayerSource);
}

// ============================================================
// Phase 3: Spells
// ============================================================

void GameHandler::castSpell(uint32_t spellId, uint64_t targetGuid) {
    // Attack (6603) routes to auto-attack instead of cast
    if (spellId == 6603) {
        uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
        if (target != 0) {
            if (autoAttacking) {
                stopAutoAttack();
            } else {
                startAutoAttack(target);
            }
        }
        return;
    }

    if (state != WorldState::IN_WORLD || !socket) return;

    if (casting) return; // Already casting

    uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
    auto packet = CastSpellPacket::build(spellId, target, ++castCount);
    socket->send(packet);
    LOG_INFO("Casting spell: ", spellId, " on 0x", std::hex, target, std::dec);
}

void GameHandler::cancelCast() {
    if (!casting) return;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = CancelCastPacket::build(currentCastSpellId);
        socket->send(packet);
    }
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;
}

void GameHandler::cancelAura(uint32_t spellId) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = CancelAuraPacket::build(spellId);
    socket->send(packet);
}

void GameHandler::setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id) {
    if (slot < 0 || slot >= ACTION_BAR_SLOTS) return;
    actionBar[slot].type = type;
    actionBar[slot].id = id;
}

float GameHandler::getSpellCooldown(uint32_t spellId) const {
    auto it = spellCooldowns.find(spellId);
    return (it != spellCooldowns.end()) ? it->second : 0.0f;
}

void GameHandler::handleInitialSpells(network::Packet& packet) {
    InitialSpellsData data;
    if (!InitialSpellsParser::parse(packet, data)) return;

    knownSpells = data.spellIds;

    // Ensure Attack (6603) and Hearthstone (8690) are always present
    if (std::find(knownSpells.begin(), knownSpells.end(), 6603u) == knownSpells.end()) {
        knownSpells.insert(knownSpells.begin(), 6603u);
    }
    if (std::find(knownSpells.begin(), knownSpells.end(), 8690u) == knownSpells.end()) {
        knownSpells.push_back(8690u);
    }

    // Set initial cooldowns
    for (const auto& cd : data.cooldowns) {
        if (cd.cooldownMs > 0) {
            spellCooldowns[cd.spellId] = cd.cooldownMs / 1000.0f;
        }
    }

    // Auto-populate action bar: Attack in slot 1, Hearthstone in slot 12
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone

    LOG_INFO("Learned ", knownSpells.size(), " spells");
}

void GameHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    if (!CastFailedParser::parse(packet, data)) return;

    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Add system message about failed cast with readable reason
    const char* reason = getSpellCastResultString(data.result);
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    if (reason) {
        msg.message = reason;
    } else {
        msg.message = "Spell cast failed (error " + std::to_string(data.result) + ")";
    }
    addLocalChatMessage(msg);
}

void GameHandler::handleSpellStart(network::Packet& packet) {
    SpellStartData data;
    if (!SpellStartParser::parse(packet, data)) return;

    // If this is the player's own cast, start cast bar
    if (data.casterUnit == playerGuid && data.castTime > 0) {
        casting = true;
        currentCastSpellId = data.spellId;
        castTimeTotal = data.castTime / 1000.0f;
        castTimeRemaining = castTimeTotal;
    }
}

void GameHandler::handleSpellGo(network::Packet& packet) {
    SpellGoData data;
    if (!SpellGoParser::parse(packet, data)) return;

    // Cast completed
    if (data.casterUnit == playerGuid) {
        casting = false;
        currentCastSpellId = 0;
        castTimeRemaining = 0.0f;
    }
}

void GameHandler::handleSpellCooldown(network::Packet& packet) {
    SpellCooldownData data;
    if (!SpellCooldownParser::parse(packet, data)) return;

    for (const auto& [spellId, cooldownMs] : data.cooldowns) {
        float seconds = cooldownMs / 1000.0f;
        spellCooldowns[spellId] = seconds;
        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
                slot.cooldownTotal = seconds;
                slot.cooldownRemaining = seconds;
            }
        }
    }
}

void GameHandler::handleCooldownEvent(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    // Cooldown finished
    spellCooldowns.erase(spellId);
    for (auto& slot : actionBar) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
            slot.cooldownRemaining = 0.0f;
        }
    }
}

void GameHandler::handleAuraUpdate(network::Packet& packet, bool isAll) {
    AuraUpdateData data;
    if (!AuraUpdateParser::parse(packet, data, isAll)) return;

    // Determine which aura list to update
    std::vector<AuraSlot>* auraList = nullptr;
    if (data.guid == playerGuid) {
        auraList = &playerAuras;
    } else if (data.guid == targetGuid) {
        auraList = &targetAuras;
    }

    if (auraList) {
        for (const auto& [slot, aura] : data.updates) {
            // Ensure vector is large enough
            while (auraList->size() <= slot) {
                auraList->push_back(AuraSlot{});
            }
            (*auraList)[slot] = aura;
        }
    }
}

void GameHandler::handleLearnedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.push_back(spellId);
    LOG_INFO("Learned spell: ", spellId);
}

void GameHandler::handleRemovedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.erase(
        std::remove(knownSpells.begin(), knownSpells.end(), spellId),
        knownSpells.end());
    LOG_INFO("Removed spell: ", spellId);
}

// ============================================================
// Phase 4: Group/Party
// ============================================================

void GameHandler::inviteToGroup(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupInvitePacket::build(playerName);
    socket->send(packet);
    LOG_INFO("Inviting ", playerName, " to group");
}

void GameHandler::acceptGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupAcceptPacket::build();
    socket->send(packet);
    LOG_INFO("Accepted group invite");
}

void GameHandler::declineGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupDeclinePacket::build();
    socket->send(packet);
    LOG_INFO("Declined group invite");
}

void GameHandler::leaveGroup() {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupDisbandPacket::build();
    socket->send(packet);
    partyData = GroupListData{};
    LOG_INFO("Left group");
}

void GameHandler::handleGroupInvite(network::Packet& packet) {
    GroupInviteResponseData data;
    if (!GroupInviteResponseParser::parse(packet, data)) return;

    pendingGroupInvite = true;
    pendingInviterName = data.inviterName;
    LOG_INFO("Group invite from: ", data.inviterName);
    if (!data.inviterName.empty()) {
        addSystemChatMessage(data.inviterName + " has invited you to a group.");
    }
}

void GameHandler::handleGroupDecline(network::Packet& packet) {
    GroupDeclineData data;
    if (!GroupDeclineResponseParser::parse(packet, data)) return;

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = data.playerName + " has declined your group invitation.";
    addLocalChatMessage(msg);
}

void GameHandler::handleGroupList(network::Packet& packet) {
    if (!GroupListParser::parse(packet, partyData)) return;

    if (partyData.isEmpty()) {
        LOG_INFO("No longer in a group");
        addSystemChatMessage("You are no longer in a group.");
    } else {
        LOG_INFO("In group with ", partyData.memberCount, " members");
        addSystemChatMessage("You are now in a group with " + std::to_string(partyData.memberCount) + " members.");
    }
}

void GameHandler::handleGroupUninvite(network::Packet& packet) {
    (void)packet;
    partyData = GroupListData{};
    LOG_INFO("Removed from group");

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "You have been removed from the group.";
    addLocalChatMessage(msg);
}

void GameHandler::handlePartyCommandResult(network::Packet& packet) {
    PartyCommandResultData data;
    if (!PartyCommandResultParser::parse(packet, data)) return;

    if (data.result != PartyResult::OK) {
        MessageChatData msg;
        msg.type = ChatType::SYSTEM;
        msg.language = ChatLanguage::UNIVERSAL;
        msg.message = "Party command failed (error " + std::to_string(static_cast<uint32_t>(data.result)) + ")";
        if (!data.name.empty()) msg.message += " for " + data.name;
        addLocalChatMessage(msg);
    }
}

// ============================================================
// Phase 5: Loot, Gossip, Vendor
// ============================================================

void GameHandler::lootTarget(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = LootPacket::build(guid);
    socket->send(packet);
}

void GameHandler::lootItem(uint8_t slotIndex) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = AutostoreLootItemPacket::build(slotIndex);
    socket->send(packet);
}

void GameHandler::closeLoot() {
    if (!lootWindowOpen) return;
    lootWindowOpen = false;
    if (currentLoot.lootGuid != 0 && targetGuid == currentLoot.lootGuid) {
        clearTarget();
    }
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = LootReleasePacket::build(currentLoot.lootGuid);
        socket->send(packet);
    }
    currentLoot = LootResponseData{};
}

void GameHandler::interactWithNpc(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GossipHelloPacket::build(guid);
    socket->send(packet);
}

void GameHandler::selectGossipOption(uint32_t optionId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, currentGossip.menuId, optionId);
    socket->send(packet);
}

void GameHandler::selectGossipQuest(uint32_t questId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    auto packet = QuestgiverQueryQuestPacket::build(currentGossip.npcGuid, questId);
    socket->send(packet);
    gossipWindowOpen = false;
}

void GameHandler::handleQuestDetails(network::Packet& packet) {
    QuestDetailsData data;
    if (!QuestDetailsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_QUEST_DETAILS");
        return;
    }
    currentQuestDetails = data;
    questDetailsOpen = true;
    gossipWindowOpen = false;
}

void GameHandler::acceptQuest() {
    if (!questDetailsOpen || state != WorldState::IN_WORLD || !socket) return;
    uint64_t npcGuid = currentQuestDetails.npcGuid;
    auto packet = QuestgiverAcceptQuestPacket::build(
        npcGuid, currentQuestDetails.questId);
    socket->send(packet);

    // Add to quest log
    bool alreadyInLog = false;
    for (const auto& q : questLog_) {
        if (q.questId == currentQuestDetails.questId) { alreadyInLog = true; break; }
    }
    if (!alreadyInLog) {
        QuestLogEntry entry;
        entry.questId = currentQuestDetails.questId;
        entry.title = currentQuestDetails.title;
        entry.objectives = currentQuestDetails.objectives;
        questLog_.push_back(entry);
    }

    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};

    // Re-query quest giver status so marker updates (! → ?)
    if (npcGuid) {
        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::declineQuest() {
    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};
}

void GameHandler::abandonQuest(uint32_t questId) {
    // Find the quest's index in our local log
    for (size_t i = 0; i < questLog_.size(); i++) {
        if (questLog_[i].questId == questId) {
            // Tell server to remove it (slot index in server quest log)
            // We send the local index; server maps it via PLAYER_QUEST_LOG fields
            if (state == WorldState::IN_WORLD && socket) {
                network::Packet pkt(static_cast<uint16_t>(Opcode::CMSG_QUESTLOG_REMOVE_QUEST));
                pkt.writeUInt8(static_cast<uint8_t>(i));
                socket->send(pkt);
            }
            questLog_.erase(questLog_.begin() + static_cast<ptrdiff_t>(i));
            return;
        }
    }
}

void GameHandler::handleQuestRequestItems(network::Packet& packet) {
    QuestRequestItemsData data;
    if (!QuestRequestItemsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_REQUEST_ITEMS");
        return;
    }
    currentQuestRequestItems_ = data;
    questRequestItemsOpen_ = true;
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for required items
    for (const auto& item : data.requiredItems) {
        queryItemInfo(item.itemId, 0);
    }
}

void GameHandler::handleQuestOfferReward(network::Packet& packet) {
    QuestOfferRewardData data;
    if (!QuestOfferRewardParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_OFFER_REWARD");
        return;
    }
    currentQuestOfferReward_ = data;
    questOfferRewardOpen_ = true;
    questRequestItemsOpen_ = false;
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for reward items
    for (const auto& item : data.choiceRewards)
        queryItemInfo(item.itemId, 0);
    for (const auto& item : data.fixedRewards)
        queryItemInfo(item.itemId, 0);
}

void GameHandler::completeQuest() {
    if (!questRequestItemsOpen_ || state != WorldState::IN_WORLD || !socket) return;
    auto packet = QuestgiverCompleteQuestPacket::build(
        currentQuestRequestItems_.npcGuid, currentQuestRequestItems_.questId);
    socket->send(packet);
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::closeQuestRequestItems() {
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::chooseQuestReward(uint32_t rewardIndex) {
    if (!questOfferRewardOpen_ || state != WorldState::IN_WORLD || !socket) return;
    uint64_t npcGuid = currentQuestOfferReward_.npcGuid;
    auto packet = QuestgiverChooseRewardPacket::build(
        npcGuid, currentQuestOfferReward_.questId, rewardIndex);
    socket->send(packet);
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};

    // Re-query quest giver status so markers update
    if (npcGuid) {
        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::closeQuestOfferReward() {
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};
}

void GameHandler::closeGossip() {
    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::openVendor(uint64_t npcGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = ListInventoryPacket::build(npcGuid);
    socket->send(packet);
}

void GameHandler::closeVendor() {
    vendorWindowOpen = false;
    currentVendorItems = ListInventoryData{};
}

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = BuyItemPacket::build(vendorGuid, itemId, slot, count);
    socket->send(packet);
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    socket->send(packet);
}

void GameHandler::sellItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }
    LOG_DEBUG("sellItemBySlot: slot=", backpackIndex,
              " item=", slot.item.name,
              " itemGuid=0x", std::hex, itemGuid, std::dec,
              " vendorGuid=0x", std::hex, currentVendorItems.vendorGuid, std::dec);
    if (itemGuid != 0 && currentVendorItems.vendorGuid != 0) {
        sellItem(currentVendorItems.vendorGuid, itemGuid, 1);
    } else if (itemGuid == 0) {
        addSystemChatMessage("Cannot sell: item not found in inventory.");
        LOG_WARNING("Sell failed: missing item GUID for slot ", backpackIndex);
    } else {
        addSystemChatMessage("Cannot sell: no vendor.");
    }
}

void GameHandler::autoEquipItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = AutoEquipItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex));
        socket->send(packet);
    }
}

void GameHandler::useItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }
    if (itemGuid != 0 && state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = UseItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex), itemGuid);
        socket->send(packet);
    } else if (itemGuid == 0) {
        LOG_WARNING("Use item failed: missing item GUID for slot ", backpackIndex);
    }
}

void GameHandler::useItemById(uint32_t itemId) {
    if (itemId == 0) return;
    for (int i = 0; i < inventory.getBackpackSize(); i++) {
        const auto& slot = inventory.getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            useItemBySlot(i);
            return;
        }
    }
}

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;
    if (currentLoot.gold > 0) {
        if (state == WorldState::IN_WORLD && socket) {
            // Auto-loot gold by sending CMSG_LOOT_MONEY (server handles the rest)
            auto pkt = LootMoneyPacket::build();
            socket->send(pkt);
            currentLoot.gold = 0;
        }
    }
}

void GameHandler::handleLootReleaseResponse(network::Packet& packet) {
    (void)packet;
    lootWindowOpen = false;
    currentLoot = LootResponseData{};
}

void GameHandler::handleLootRemoved(network::Packet& packet) {
    uint8_t slotIndex = packet.readUInt8();
    for (auto it = currentLoot.items.begin(); it != currentLoot.items.end(); ++it) {
        if (it->slotIndex == slotIndex) {
            currentLoot.items.erase(it);
            break;
        }
    }
}

void GameHandler::handleGossipMessage(network::Packet& packet) {
    if (!GossipMessageParser::parse(packet, currentGossip)) return;
    if (questDetailsOpen) return; // Don't reopen gossip while viewing quest
    gossipWindowOpen = true;
    vendorWindowOpen = false; // Close vendor if gossip opens
}

void GameHandler::handleGossipComplete(network::Packet& packet) {
    (void)packet;
    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::handleListInventory(network::Packet& packet) {
    if (!ListInventoryParser::parse(packet, currentVendorItems)) return;
    vendorWindowOpen = true;
    gossipWindowOpen = false; // Close gossip if vendor opens

    // Query item info for all vendor items so we can show names
    for (const auto& item : currentVendorItems.items) {
        queryItemInfo(item.itemId, 0);
    }
}

// ============================================================
// Single-player local combat
// ============================================================








// ============================================================
// XP tracking
// ============================================================

// WotLK 3.3.5a XP-to-next-level table (from player_xp_for_level)
static const uint32_t XP_TABLE[] = {
    0,       // level 0 (unused)
    400,     900,     1400,    2100,    2800,    3600,    4500,    5400,    6500,    7600,     // 1-10
    8700,    9800,    11000,   12300,   13600,   15000,   16400,   17800,   19300,   20800,    // 11-20
    22400,   24000,   25500,   27200,   28900,   30500,   32200,   33900,   36300,   38800,    // 21-30
    41600,   44600,   48000,   51400,   55000,   58700,   62400,   66200,   70200,   74300,    // 31-40
    78500,   82800,   87100,   91600,   96300,   101000,  105800,  110700,  115700,  120900,   // 41-50
    126100,  131500,  137000,  142500,  148200,  154000,  159900,  165800,  172000,  290000,   // 51-60
    317000,  349000,  386000,  428000,  475000,  527000,  585000,  648000,  717000,  1523800,  // 61-70
    1539600, 1555700, 1571800, 1587900, 1604200, 1620700, 1637400, 1653900, 1670800           // 71-79
};
static constexpr uint32_t XP_TABLE_SIZE = sizeof(XP_TABLE) / sizeof(XP_TABLE[0]);

uint32_t GameHandler::xpForLevel(uint32_t level) {
    if (level == 0 || level >= XP_TABLE_SIZE) return 0;
    return XP_TABLE[level];
}

uint32_t GameHandler::killXp(uint32_t playerLevel, uint32_t victimLevel) {
    if (playerLevel == 0 || victimLevel == 0) return 0;

    // Gray level check (too low = 0 XP)
    int32_t grayLevel;
    if (playerLevel <= 5)        grayLevel = 0;
    else if (playerLevel <= 39)  grayLevel = static_cast<int32_t>(playerLevel) - 5 - static_cast<int32_t>(playerLevel) / 10;
    else if (playerLevel <= 59)  grayLevel = static_cast<int32_t>(playerLevel) - 1 - static_cast<int32_t>(playerLevel) / 5;
    else                         grayLevel = static_cast<int32_t>(playerLevel) - 9;

    if (static_cast<int32_t>(victimLevel) <= grayLevel) return 0;

    // Base XP = 45 + 5 * victimLevel (WoW-like ZeroDifference formula)
    uint32_t baseXp = 45 + 5 * victimLevel;

    // Level difference multiplier
    int32_t diff = static_cast<int32_t>(victimLevel) - static_cast<int32_t>(playerLevel);
    float multiplier = 1.0f + diff * 0.05f;
    if (multiplier < 0.1f) multiplier = 0.1f;
    if (multiplier > 2.0f) multiplier = 2.0f;

    return static_cast<uint32_t>(baseXp * multiplier);
}



void GameHandler::handleXpGain(network::Packet& packet) {
    XpGainData data;
    if (!XpGainParser::parse(packet, data)) return;

    // Server already updates PLAYER_XP via update fields,
    // but we can show combat text for XP gains
    addCombatText(CombatTextEntry::HEAL, static_cast<int32_t>(data.totalXp), 0, true);

    std::string msg = "You gain " + std::to_string(data.totalXp) + " experience.";
    if (data.groupBonus > 0) {
        msg += " (+" + std::to_string(data.groupBonus) + " group bonus)";
    }
    addSystemChatMessage(msg);
}


void GameHandler::addMoneyCopper(uint32_t amount) {
    if (amount == 0) return;
    playerMoneyCopper_ += amount;
    uint32_t gold = amount / 10000;
    uint32_t silver = (amount / 100) % 100;
    uint32_t copper = amount % 100;
    std::string msg = "You receive ";
    msg += std::to_string(gold) + "g ";
    msg += std::to_string(silver) + "s ";
    msg += std::to_string(copper) + "c.";
    addSystemChatMessage(msg);
}

void GameHandler::addSystemChatMessage(const std::string& message) {
    if (message.empty()) return;
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = message;
    addLocalChatMessage(msg);
}

// ============================================================
// Server Info Command Handlers
// ============================================================

void GameHandler::handleQueryTimeResponse(network::Packet& packet) {
    QueryTimeResponseData data;
    if (!QueryTimeResponseParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUERY_TIME_RESPONSE");
        return;
    }

    // Convert Unix timestamp to readable format
    time_t serverTime = static_cast<time_t>(data.serverTime);
    struct tm* timeInfo = localtime(&serverTime);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", timeInfo);

    std::string msg = "Server time: " + std::string(timeStr);
    addSystemChatMessage(msg);
    LOG_INFO("Server time: ", data.serverTime, " (", timeStr, ")");
}

void GameHandler::handlePlayedTime(network::Packet& packet) {
    PlayedTimeData data;
    if (!PlayedTimeParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_PLAYED_TIME");
        return;
    }

    if (data.triggerMessage) {
        // Format total time played
        uint32_t totalDays = data.totalTimePlayed / 86400;
        uint32_t totalHours = (data.totalTimePlayed % 86400) / 3600;
        uint32_t totalMinutes = (data.totalTimePlayed % 3600) / 60;

        // Format level time played
        uint32_t levelDays = data.levelTimePlayed / 86400;
        uint32_t levelHours = (data.levelTimePlayed % 86400) / 3600;
        uint32_t levelMinutes = (data.levelTimePlayed % 3600) / 60;

        std::string totalMsg = "Total time played: ";
        if (totalDays > 0) totalMsg += std::to_string(totalDays) + " days, ";
        if (totalHours > 0 || totalDays > 0) totalMsg += std::to_string(totalHours) + " hours, ";
        totalMsg += std::to_string(totalMinutes) + " minutes";

        std::string levelMsg = "Time played this level: ";
        if (levelDays > 0) levelMsg += std::to_string(levelDays) + " days, ";
        if (levelHours > 0 || levelDays > 0) levelMsg += std::to_string(levelHours) + " hours, ";
        levelMsg += std::to_string(levelMinutes) + " minutes";

        addSystemChatMessage(totalMsg);
        addSystemChatMessage(levelMsg);
    }

    LOG_INFO("Played time: total=", data.totalTimePlayed, "s, level=", data.levelTimePlayed, "s");
}

void GameHandler::handleWho(network::Packet& packet) {
    // Parse WHO response
    uint32_t displayCount = packet.readUInt32();
    uint32_t onlineCount = packet.readUInt32();

    LOG_INFO("WHO response: ", displayCount, " players displayed, ", onlineCount, " total online");

    if (displayCount == 0) {
        addSystemChatMessage("No players found.");
        return;
    }

    addSystemChatMessage(std::to_string(onlineCount) + " player(s) online:");

    for (uint32_t i = 0; i < displayCount; ++i) {
        std::string playerName = packet.readString();
        std::string guildName = packet.readString();
        uint32_t level = packet.readUInt32();
        uint32_t classId = packet.readUInt32();
        uint32_t raceId = packet.readUInt32();
        packet.readUInt8();   // gender (unused)
        packet.readUInt32();  // zoneId (unused)

        std::string msg = "  " + playerName;
        if (!guildName.empty()) {
            msg += " <" + guildName + ">";
        }
        msg += " - Level " + std::to_string(level);

        addSystemChatMessage(msg);
        LOG_INFO("  ", playerName, " (", guildName, ") Lv", level, " Class:", classId, " Race:", raceId);
    }
}

uint32_t GameHandler::generateClientSeed() {
    // Generate cryptographically random seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(1, 0xFFFFFFFF);
    return dis(gen);
}

void GameHandler::setState(WorldState newState) {
    if (state != newState) {
        LOG_DEBUG("World state: ", (int)state, " -> ", (int)newState);
        state = newState;
    }
}

void GameHandler::fail(const std::string& reason) {
    LOG_ERROR("World connection failed: ", reason);
    setState(WorldState::FAILED);

    if (onFailure) {
        onFailure(reason);
    }
}


} // namespace game
} // namespace wowee
