#include "game/game_handler.hpp"
#include "game/opcodes.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <random>
#include <chrono>

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
    setState(WorldState::DISCONNECTED);
    LOG_INFO("Disconnected from world server");
}

bool GameHandler::isConnected() const {
    return socket && socket->isConnected();
}

void GameHandler::update(float deltaTime) {
    if (!socket) {
        return;
    }

    // Update socket (processes incoming data and triggers callbacks)
    socket->update();

    // Validate target still exists
    if (targetGuid != 0 && !entityManager.hasEntity(targetGuid)) {
        clearTarget();
    }

    // Send periodic heartbeat if in world
    if (state == WorldState::IN_WORLD) {
        timeSinceLastPing += deltaTime;

        if (timeSinceLastPing >= pingInterval) {
            sendPing();
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
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleUpdateObject(packet);
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

        // ---- Phase 1: Foundation ----
        case Opcode::SMSG_NAME_QUERY_RESPONSE:
            handleNameQueryResponse(packet);
            break;

        case Opcode::SMSG_CREATURE_QUERY_RESPONSE:
            handleCreatureQueryResponse(packet);
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
        case Opcode::SMSG_LOOT_MONEY_NOTIFY:
        case Opcode::SMSG_LOOT_CLEAR_MONEY:
        case Opcode::SMSG_NPC_TEXT_UPDATE:
        case Opcode::SMSG_SELL_ITEM:
        case Opcode::SMSG_BUY_FAILED:
        case Opcode::SMSG_INVENTORY_CHANGE_FAILURE:
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
        case Opcode::MSG_RAID_TARGET_UPDATE:
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

    // Send packet (NOT encrypted yet)
    socket->send(packet);

    // CRITICAL: Initialize encryption AFTER sending AUTH_SESSION
    // but BEFORE receiving AUTH_RESPONSE
    LOG_INFO("Initializing RC4 header encryption...");
    socket->initEncryption(sessionKey);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption initialized, waiting for response...");
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

    // Authentication successful!
    setState(WorldState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   WORLD AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("Connected to world server");
    LOG_INFO("Ready for character operations");

    setState(WorldState::READY);

    // Call success callback
    if (onSuccess) {
        onSuccess();
    }
}

void GameHandler::requestCharacterList() {
    if (state != WorldState::READY && state != WorldState::AUTHENTICATED) {
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

    // Initialize movement info with world entry position
    movementInfo.x = data.x;
    movementInfo.y = data.y;
    movementInfo.z = data.z;
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

    // Update movement time
    movementInfo.time = ++movementTime;

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

    // Build and send movement packet
    auto packet = MovementPacket::build(opcode, movementInfo);
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

                // Set position from movement block
                if (block.hasMovement) {
                    entity->setPosition(block.x, block.y, block.z, block.orientation);
                    LOG_DEBUG("  Position: (", block.x, ", ", block.y, ", ", block.z, ")");
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
                        queryCreatureInfo(it->second, block.guid);
                    }
                }

                // Extract health/mana/power from fields (Phase 2) — single pass
                if (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) {
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    for (const auto& [key, val] : block.fields) {
                        switch (key) {
                            case 24: unit->setHealth(val); break;
                            case 25: unit->setPower(val); break;
                            case 32: unit->setMaxHealth(val); break;
                            case 33: unit->setMaxPower(val); break;
                            case 54: unit->setLevel(val); break;
                            default: break;
                        }
                    }
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
                                case 24: unit->setHealth(val); break;
                                case 25: unit->setPower(val); break;
                                case 32: unit->setMaxHealth(val); break;
                                case 33: unit->setMaxPower(val); break;
                                case 54: unit->setLevel(val); break;
                                default: break;
                            }
                        }
                    }

                    LOG_DEBUG("Updated entity fields: 0x", std::hex, block.guid, std::dec);
                } else {
                    LOG_WARNING("VALUES update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            case UpdateType::MOVEMENT: {
                // Update entity position
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    entity->setPosition(block.x, block.y, block.z, block.orientation);
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
// Phase 2: Combat
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    autoAttacking = true;
    autoAttackTarget = targetGuid;
    auto packet = AttackSwingPacket::build(targetGuid);
    socket->send(packet);
    LOG_INFO("Starting auto-attack on 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopAutoAttack() {
    if (!autoAttacking) return;
    autoAttacking = false;
    autoAttackTarget = 0;
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

    if (data.attackerGuid == playerGuid) {
        autoAttacking = false;
        autoAttackTarget = 0;
    }
}

void GameHandler::handleAttackerStateUpdate(network::Packet& packet) {
    AttackerStateUpdateData data;
    if (!AttackerStateUpdateParser::parse(packet, data)) return;

    bool isPlayerAttacker = (data.attackerGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);

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
    if (state != WorldState::IN_WORLD || !socket) return;

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

    if (casting) return; // Already casting

    uint64_t target = targetGuid != 0 ? targetGuid : targetGuid;
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

    // Auto-populate action bar: Attack in slot 1, Hearthstone in slot 12, rest filled with known spells
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone
    int slot = 1;
    for (int i = 0; i < static_cast<int>(knownSpells.size()) && slot < 11; ++i) {
        if (knownSpells[i] == 6603 || knownSpells[i] == 8690) continue;
        actionBar[slot].type = ActionBarSlot::SPELL;
        actionBar[slot].id = knownSpells[i];
        slot++;
    }

    LOG_INFO("Learned ", knownSpells.size(), " spells");
}

void GameHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    if (!CastFailedParser::parse(packet, data)) return;

    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Add system message about failed cast
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "Spell cast failed (error " + std::to_string(data.result) + ")";
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
    } else {
        LOG_INFO("In group with ", partyData.memberCount, " members");
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
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, optionId);
    socket->send(packet);
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

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = BuyItemPacket::build(vendorGuid, itemId, slot, count);
    socket->send(packet);
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    socket->send(packet);
}

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;
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
