#include "game/game_handler.hpp"
#include "game/transport_manager.hpp"
#include "game/opcodes.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
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

    // Initialize transport manager
    transportManager_ = std::make_unique<TransportManager>();

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
    if (onTaxiFlight_) {
        taxiRecoverPending_ = true;
    } else {
        taxiRecoverPending_ = false;
    }
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
    // Timing profiling (log every 60 frames to reduce spam)
    static int profileCounter = 0;
    static float socketTime = 0.0f;
    static float taxiTime = 0.0f;
    static float distanceCheckTime = 0.0f;
    static float entityUpdateTime = 0.0f;
    static float totalTime = 0.0f;

    auto updateStart = std::chrono::high_resolution_clock::now();

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
    auto socketStart = std::chrono::high_resolution_clock::now();
    if (socket) {
        socket->update();
    }
    auto socketEnd = std::chrono::high_resolution_clock::now();
    socketTime += std::chrono::duration<float, std::milli>(socketEnd - socketStart).count();

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

        // Update taxi landing cooldown
        if (taxiLandingCooldown_ > 0.0f) {
            taxiLandingCooldown_ -= deltaTime;
        }

        // Taxi logic timing
        auto taxiStart = std::chrono::high_resolution_clock::now();

        // Detect taxi flight landing: UNIT_FLAG_TAXI_FLIGHT (0x00000100) cleared
        if (onTaxiFlight_) {
            updateClientTaxi(deltaTime);
            if (!taxiMountActive_ && !taxiClientActive_ && taxiClientPath_.empty()) {
                onTaxiFlight_ = false;
                LOG_INFO("Cleared stale taxi state in update");
            }
            auto playerEntity = entityManager.getEntity(playerGuid);
            if (playerEntity && playerEntity->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(playerEntity);
                if ((unit->getUnitFlags() & 0x00000100) == 0) {
                    onTaxiFlight_ = false;
                    taxiLandingCooldown_ = 2.0f;  // 2 second cooldown to prevent re-entering
                    if (taxiMountActive_ && mountCallback_) {
                        mountCallback_(0);
                    }
                    taxiMountActive_ = false;
                    taxiMountDisplayId_ = 0;
                    currentMountDisplayId_ = 0;
                    taxiClientActive_ = false;
                    taxiClientPath_.clear();
                    taxiRecoverPending_ = false;
                    movementInfo.flags = 0;
                    movementInfo.flags2 = 0;
                    if (socket) {
                        sendMovement(Opcode::CMSG_MOVE_STOP);
                        sendMovement(Opcode::CMSG_MOVE_HEARTBEAT);
                    }
                    LOG_INFO("Taxi flight landed");
                }
            }
        }

        // Safety: if taxi flight ended but mount is still active, force dismount.
        if (!onTaxiFlight_ && taxiMountActive_) {
            if (mountCallback_) mountCallback_(0);
            taxiMountActive_ = false;
            taxiMountDisplayId_ = 0;
            currentMountDisplayId_ = 0;
            movementInfo.flags = 0;
            movementInfo.flags2 = 0;
            if (socket) {
                sendMovement(Opcode::CMSG_MOVE_STOP);
                sendMovement(Opcode::CMSG_MOVE_HEARTBEAT);
            }
            LOG_INFO("Taxi dismount cleanup");
        }

        if (taxiRecoverPending_ && state == WorldState::IN_WORLD) {
            auto playerEntity = entityManager.getEntity(playerGuid);
            if (playerEntity) {
                playerEntity->setPosition(taxiRecoverPos_.x, taxiRecoverPos_.y,
                                          taxiRecoverPos_.z, movementInfo.orientation);
                movementInfo.x = taxiRecoverPos_.x;
                movementInfo.y = taxiRecoverPos_.y;
                movementInfo.z = taxiRecoverPos_.z;
                if (socket) {
                    sendMovement(Opcode::CMSG_MOVE_HEARTBEAT);
                }
                taxiRecoverPending_ = false;
                LOG_INFO("Taxi recovery applied");
            }
        }

        if (taxiActivatePending_) {
            taxiActivateTimer_ += deltaTime;
            if (!onTaxiFlight_ && taxiActivateTimer_ > 5.0f) {
                taxiActivatePending_ = false;
                taxiActivateTimer_ = 0.0f;
                if (taxiMountActive_ && mountCallback_) {
                    mountCallback_(0);
                }
                taxiMountActive_ = false;
                taxiMountDisplayId_ = 0;
                taxiClientActive_ = false;
                taxiClientPath_.clear();
                onTaxiFlight_ = false;
                LOG_WARNING("Taxi activation timed out");
            }
        }

        // Mounting delay for taxi (terrain + M2 model precache time)
        if (taxiMountingDelay_) {
            taxiMountingTimer_ += deltaTime;
            // 5 second delay for terrain and M2 models to load and upload to VRAM
            if (taxiMountingTimer_ >= 5.0f) {
                taxiMountingDelay_ = false;
                taxiMountingTimer_ = 0.0f;
                // Upload all precached tiles to GPU before flight starts
                if (taxiFlightStartCallback_) {
                    taxiFlightStartCallback_();
                }
                if (!taxiPendingPath_.empty()) {
                    startClientTaxiPath(taxiPendingPath_);
                    taxiPendingPath_.clear();
                }
            }
        }

        auto taxiEnd = std::chrono::high_resolution_clock::now();
        taxiTime += std::chrono::duration<float, std::milli>(taxiEnd - taxiStart).count();

        // Update transport manager
        if (transportManager_) {
            transportManager_->update(deltaTime);
        }

        // Distance check timing
        auto distanceStart = std::chrono::high_resolution_clock::now();

        // Leave combat if auto-attack target is too far away (leash range)
        if (autoAttacking && autoAttackTarget != 0) {
            auto targetEntity = entityManager.getEntity(autoAttackTarget);
            if (targetEntity) {
                float dx = movementInfo.x - targetEntity->getX();
                float dy = movementInfo.y - targetEntity->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 40.0f) {
                    stopAutoAttack();
                    LOG_INFO("Left combat: target too far (", dist, " yards)");
                }
            }
        }

        // Close vendor/gossip/taxi window if player walks too far from NPC
        if (vendorWindowOpen && currentVendorItems.vendorGuid != 0) {
            auto npc = entityManager.getEntity(currentVendorItems.vendorGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeVendor();
                    LOG_INFO("Vendor closed: walked too far from NPC");
                }
            }
        }
        if (gossipWindowOpen && currentGossip.npcGuid != 0) {
            auto npc = entityManager.getEntity(currentGossip.npcGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeGossip();
                    LOG_INFO("Gossip closed: walked too far from NPC");
                }
            }
        }
        if (taxiWindowOpen_ && taxiNpcGuid_ != 0) {
            auto npc = entityManager.getEntity(taxiNpcGuid_);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeTaxi();
                    LOG_INFO("Taxi window closed: walked too far from NPC");
                }
            }
        }
        if (trainerWindowOpen_ && currentTrainerList_.trainerGuid != 0) {
            auto npc = entityManager.getEntity(currentTrainerList_.trainerGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeTrainer();
                    LOG_INFO("Trainer closed: walked too far from NPC");
                }
            }
        }

        auto distanceEnd = std::chrono::high_resolution_clock::now();
        distanceCheckTime += std::chrono::duration<float, std::milli>(distanceEnd - distanceStart).count();

        // Entity update timing
        auto entityStart = std::chrono::high_resolution_clock::now();

        // Update entity movement interpolation (keeps targeting in sync with visuals)
        // Only update entities within reasonable distance for performance
        const float updateRadiusSq = 150.0f * 150.0f;  // 150 unit radius
        auto playerEntity = entityManager.getEntity(playerGuid);
        glm::vec3 playerPos = playerEntity ? glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ()) : glm::vec3(0.0f);

        for (auto& [guid, entity] : entityManager.getEntities()) {
            // Always update player
            if (guid == playerGuid) {
                entity->updateMovement(deltaTime);
                continue;
            }

            // Distance cull other entities
            glm::vec3 entityPos(entity->getX(), entity->getY(), entity->getZ());
            float distSq = glm::dot(entityPos - playerPos, entityPos - playerPos);
            if (distSq < updateRadiusSq) {
                entity->updateMovement(deltaTime);
            }
        }

        auto entityEnd = std::chrono::high_resolution_clock::now();
        entityUpdateTime += std::chrono::duration<float, std::milli>(entityEnd - entityStart).count();
    }

    auto updateEnd = std::chrono::high_resolution_clock::now();
    totalTime += std::chrono::duration<float, std::milli>(updateEnd - updateStart).count();

    // Log profiling every 60 frames
    if (++profileCounter >= 60) {
        LOG_INFO("UPDATE PROFILE (60 frames): socket=", socketTime / 60.0f, "ms taxi=", taxiTime / 60.0f,
                 "ms distance=", distanceCheckTime / 60.0f, "ms entity=", entityUpdateTime / 60.0f,
                 "ms TOTAL=", totalTime / 60.0f, "ms");
        profileCounter = 0;
        socketTime = 0.0f;
        taxiTime = 0.0f;
        distanceCheckTime = 0.0f;
        entityUpdateTime = 0.0f;
        totalTime = 0.0f;
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
            if (state == WorldState::ENTERING_WORLD || state == WorldState::IN_WORLD) {
                handleLoginVerifyWorld(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_LOGIN_VERIFY_WORLD in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_LOGIN_SETTIMESPEED:
            // Can be received during login or at any time after
            handleLoginSetTimeSpeed(packet);
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

        case Opcode::SMSG_FRIEND_STATUS:
            if (state == WorldState::IN_WORLD) {
                handleFriendStatus(packet);
            }
            break;

        case Opcode::MSG_RANDOM_ROLL:
            if (state == WorldState::IN_WORLD) {
                handleRandomRoll(packet);
            }
            break;

        case Opcode::SMSG_LOGOUT_RESPONSE:
            handleLogoutResponse(packet);
            break;

        case Opcode::SMSG_LOGOUT_COMPLETE:
            handleLogoutComplete(packet);
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

        case Opcode::SMSG_MONSTER_MOVE_TRANSPORT:
            handleMonsterMoveTransport(packet);
            break;

        // ---- Speed Changes ----
        case Opcode::SMSG_FORCE_RUN_SPEED_CHANGE:
            handleForceRunSpeedChange(packet);
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
        case Opcode::SMSG_SUPERCEDED_SPELL:
            handleSupercededSpell(packet);
            break;
        case Opcode::SMSG_REMOVED_SPELL:
            handleRemovedSpell(packet);
            break;
        case Opcode::SMSG_SEND_UNLEARN_SPELLS:
            handleUnlearnSpells(packet);
            break;

        // ---- Talents ----
        case Opcode::SMSG_TALENTS_INFO:
            handleTalentsInfo(packet);
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
        case Opcode::SMSG_BINDPOINTUPDATE: {
            BindPointUpdateData data;
            if (BindPointUpdateParser::parse(packet, data)) {
                LOG_INFO("Bindpoint updated: mapId=", data.mapId,
                         " pos=(", data.x, ", ", data.y, ", ", data.z, ")");
                glm::vec3 canonical = core::coords::serverToCanonical(
                    glm::vec3(data.x, data.y, data.z));
                hasHomeBind_ = true;
                homeBindMapId_ = data.mapId;
                homeBindPos_ = canonical;
                if (bindPointCallback_) {
                    bindPointCallback_(data.mapId, canonical.x, canonical.y, canonical.z);
                }
                addSystemChatMessage("Your home has been set.");
            } else {
                LOG_WARNING("Failed to parse SMSG_BINDPOINTUPDATE");
            }
            break;
        }
        case Opcode::SMSG_GOSSIP_COMPLETE:
            handleGossipComplete(packet);
            break;
        case Opcode::SMSG_SPIRIT_HEALER_CONFIRM: {
            if (packet.getSize() - packet.getReadPos() < 8) {
                LOG_WARNING("SMSG_SPIRIT_HEALER_CONFIRM too short");
                break;
            }
            uint64_t npcGuid = packet.readUInt64();
            LOG_INFO("Spirit healer confirm from 0x", std::hex, npcGuid, std::dec);
            if (npcGuid) {
                resurrectCasterGuid_ = npcGuid;
                resurrectRequestPending_ = true;
            }
            break;
        }
        case Opcode::SMSG_RESURRECT_REQUEST: {
            if (packet.getSize() - packet.getReadPos() < 8) {
                LOG_WARNING("SMSG_RESURRECT_REQUEST too short");
                break;
            }
            uint64_t casterGuid = packet.readUInt64();
            LOG_INFO("Resurrect request from 0x", std::hex, casterGuid, std::dec);
            if (casterGuid) {
                resurrectCasterGuid_ = casterGuid;
                resurrectRequestPending_ = true;
            }
            break;
        }
        case Opcode::SMSG_RESURRECT_CANCEL: {
            if (packet.getSize() - packet.getReadPos() < 4) {
                LOG_WARNING("SMSG_RESURRECT_CANCEL too short");
                break;
            }
            uint32_t reason = packet.readUInt32();
            LOG_INFO("Resurrect cancel reason: ", reason);
            resurrectPending_ = false;
            resurrectRequestPending_ = false;
            break;
        }
        case Opcode::SMSG_LIST_INVENTORY:
            handleListInventory(packet);
            break;
        case Opcode::SMSG_TRAINER_LIST:
            handleTrainerList(packet);
            break;
        case Opcode::SMSG_TRAINER_BUY_SUCCEEDED: {
            uint64_t guid = packet.readUInt64();
            uint32_t spellId = packet.readUInt32();
            (void)guid;

            // Add to known spells immediately for prerequisite re-evaluation
            // (SMSG_LEARNED_SPELL may come separately, but we need immediate update)
            bool alreadyKnown = std::find(knownSpells.begin(), knownSpells.end(), spellId) != knownSpells.end();
            if (!alreadyKnown) {
                knownSpells.push_back(spellId);
                LOG_INFO("Added spell ", spellId, " to known spells (trainer purchase)");
            }

            const std::string& name = getSpellName(spellId);
            if (!name.empty())
                addSystemChatMessage("You have learned " + name + ".");
            else
                addSystemChatMessage("Spell learned.");
            break;
        }
        case Opcode::SMSG_TRAINER_BUY_FAILED: {
            // Server rejected the spell purchase
            // Packet format: uint64 trainerGuid, uint32 spellId, uint32 errorCode
            uint64_t trainerGuid = packet.readUInt64();
            uint32_t spellId = packet.readUInt32();
            uint32_t errorCode = 0;
            if (packet.getSize() - packet.getReadPos() >= 4) {
                errorCode = packet.readUInt32();
            }
            LOG_WARNING("Trainer buy spell failed: guid=", trainerGuid,
                       " spellId=", spellId, " error=", errorCode);

            const std::string& spellName = getSpellName(spellId);
            std::string msg = "Cannot learn ";
            if (!spellName.empty()) msg += spellName;
            else msg += "spell #" + std::to_string(spellId);

            // Common error reasons
            if (errorCode == 0) msg += " (not enough money)";
            else if (errorCode == 1) msg += " (not enough skill)";
            else if (errorCode == 2) msg += " (already known)";
            else if (errorCode != 0) msg += " (error " + std::to_string(errorCode) + ")";

            addSystemChatMessage(msg);
            break;
        }

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
                    // InventoryResult enum (AzerothCore 3.3.5a)
                    const char* errMsg = nullptr;
                    switch (error) {
                        case 1:  errMsg = "You must reach level %d to use that item."; break;
                        case 2:  errMsg = "You don't have the required skill."; break;
                        case 3:  errMsg = "That item doesn't go in that slot."; break;
                        case 4:  errMsg = "That bag is full."; break;
                        case 5:  errMsg = "Can't put bags in bags."; break;
                        case 6:  errMsg = "Can't trade equipped bags."; break;
                        case 7:  errMsg = "That slot only holds ammo."; break;
                        case 8:  errMsg = "You can't use that item."; break;
                        case 9:  errMsg = "No equipment slot available."; break;
                        case 10: errMsg = "You can never use that item."; break;
                        case 11: errMsg = "You can never use that item."; break;
                        case 12: errMsg = "No equipment slot available."; break;
                        case 13: errMsg = "Can't equip with a two-handed weapon."; break;
                        case 14: errMsg = "Can't dual-wield."; break;
                        case 15: errMsg = "That item doesn't go in that bag."; break;
                        case 16: errMsg = "That item doesn't go in that bag."; break;
                        case 17: errMsg = "You can't carry any more of those."; break;
                        case 18: errMsg = "No equipment slot available."; break;
                        case 19: errMsg = "Can't stack those items."; break;
                        case 20: errMsg = "That item can't be equipped."; break;
                        case 21: errMsg = "Can't swap items."; break;
                        case 22: errMsg = "That slot is empty."; break;
                        case 23: errMsg = "Item not found."; break;
                        case 24: errMsg = "Can't drop soulbound items."; break;
                        case 25: errMsg = "Out of range."; break;
                        case 26: errMsg = "Need to split more than 1."; break;
                        case 27: errMsg = "Split failed."; break;
                        case 28: errMsg = "Not enough reagents."; break;
                        case 29: errMsg = "Not enough money."; break;
                        case 30: errMsg = "Not a bag."; break;
                        case 31: errMsg = "Can't destroy non-empty bag."; break;
                        case 32: errMsg = "You don't own that item."; break;
                        case 33: errMsg = "You can only have one quiver."; break;
                        case 34: errMsg = "No free bank slots."; break;
                        case 35: errMsg = "No bank here."; break;
                        case 36: errMsg = "Item is locked."; break;
                        case 37: errMsg = "You are stunned."; break;
                        case 38: errMsg = "You are dead."; break;
                        case 39: errMsg = "Can't do that right now."; break;
                        case 40: errMsg = "Internal bag error."; break;
                        case 49: errMsg = "Loot is gone."; break;
                        case 50: errMsg = "Inventory is full."; break;
                        case 51: errMsg = "Bank is full."; break;
                        case 52: errMsg = "That item is sold out."; break;
                        case 58: errMsg = "That object is busy."; break;
                        case 60: errMsg = "Can't do that in combat."; break;
                        case 61: errMsg = "Can't do that while disarmed."; break;
                        case 63: errMsg = "Requires a higher rank."; break;
                        case 64: errMsg = "Requires higher reputation."; break;
                        case 67: errMsg = "That item is unique-equipped."; break;
                        case 69: errMsg = "Not enough honor points."; break;
                        case 70: errMsg = "Not enough arena points."; break;
                        case 77: errMsg = "Too much gold."; break;
                        case 78: errMsg = "Can't do that during arena match."; break;
                        case 80: errMsg = "Requires a personal arena rating."; break;
                        case 87: errMsg = "Requires a higher level."; break;
                        case 88: errMsg = "Requires the right talent."; break;
                        default: break;
                    }
                    std::string msg = errMsg ? errMsg : "Inventory error (" + std::to_string(error) + ").";
                    addSystemChatMessage(msg);
                }
            }
            break;
        }
        case Opcode::SMSG_BUY_FAILED:
        case Opcode::MSG_RAID_TARGET_UPDATE:
            break;
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
            handleGameObjectQueryResponse(packet);
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
        case Opcode::SMSG_QUESTGIVER_QUEST_INVALID: {
            // Quest query failed - parse failure reason
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t failReason = packet.readUInt32();
                const char* reasonStr = "Unknown";
                switch (failReason) {
                    case 0: reasonStr = "Don't have quest"; break;
                    case 1: reasonStr = "Quest level too low"; break;
                    case 4: reasonStr = "Insufficient money"; break;
                    case 5: reasonStr = "Inventory full"; break;
                    case 13: reasonStr = "Already on that quest"; break;
                    case 18: reasonStr = "Already completed quest"; break;
                    case 19: reasonStr = "Can't take any more quests"; break;
                }
                LOG_WARNING("Quest invalid: reason=", failReason, " (", reasonStr, ")");
                // Only show error to user for real errors (not informational messages)
                if (failReason != 13 && failReason != 18) {  // Don't spam "already on/completed"
                    addSystemChatMessage(std::string("Quest unavailable: ") + reasonStr);
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_QUEST_COMPLETE: {
            // Mark quest as complete in local log
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t questId = packet.readUInt32();
                LOG_INFO("Quest completed: questId=", questId);
                for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
                    if (it->questId == questId) {
                        questLog_.erase(it);
                        LOG_INFO("  Removed quest ", questId, " from quest log");
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
        case Opcode::SMSG_QUESTUPDATE_ADD_KILL: {
            // Quest kill count update
            if (packet.getSize() - packet.getReadPos() >= 16) {
                uint32_t questId = packet.readUInt32();
                uint32_t entry = packet.readUInt32();  // Creature entry
                uint32_t count = packet.readUInt32();  // Current kills
                uint32_t reqCount = packet.readUInt32(); // Required kills

                LOG_INFO("Quest kill update: questId=", questId, " entry=", entry,
                         " count=", count, "/", reqCount);

                // Update quest log with kill count
                for (auto& quest : questLog_) {
                    if (quest.questId == questId) {
                        // Store kill progress (using entry as objective index)
                        quest.killCounts[entry] = {count, reqCount};

                        // Show progress message
                        std::string progressMsg = quest.title + ": " +
                                                std::to_string(count) + "/" +
                                                std::to_string(reqCount);
                        addSystemChatMessage(progressMsg);

                        LOG_INFO("Updated kill count for quest ", questId, ": ",
                                 count, "/", reqCount);
                        break;
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTUPDATE_COMPLETE: {
            // Quest objectives completed - mark as ready to turn in
            uint32_t questId = packet.readUInt32();
            LOG_INFO("Quest objectives completed: questId=", questId);

            for (auto& quest : questLog_) {
                if (quest.questId == questId) {
                    quest.complete = true;
                    addSystemChatMessage("Quest Complete: " + quest.title);
                    LOG_INFO("Marked quest ", questId, " as complete");
                    break;
                }
            }
            break;
        }
        case Opcode::SMSG_QUEST_QUERY_RESPONSE: {
            // Quest data from server (big packet with title, objectives, rewards, etc.)
            LOG_INFO("SMSG_QUEST_QUERY_RESPONSE: packet size=", packet.getSize());

            if (packet.getSize() < 8) {
                LOG_WARNING("SMSG_QUEST_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
                break;
            }

            uint32_t questId = packet.readUInt32();
            uint32_t questMethod = packet.readUInt32();  // Quest method/type

            LOG_INFO("  questId=", questId, " questMethod=", questMethod);

            // Parse quest title (after method comes level, flags, type, etc., then title string)
            // Skip intermediate fields to get to title
            if (packet.getReadPos() + 16 < packet.getSize()) {
                packet.readUInt32(); // quest level
                packet.readUInt32(); // min level
                packet.readUInt32(); // sort ID (zone)
                packet.readUInt32(); // quest type/info
                packet.readUInt32(); // suggested players
                packet.readUInt32(); // reputation objective faction
                packet.readUInt32(); // reputation objective value
                packet.readUInt32(); // required opposite faction
                packet.readUInt32(); // next quest in chain
                packet.readUInt32(); // XP ID
                packet.readUInt32(); // reward or required money
                packet.readUInt32(); // reward money max level
                packet.readUInt32(); // reward spell
                packet.readUInt32(); // reward spell cast
                packet.readUInt32(); // reward honor
                packet.readUInt32(); // reward honor multiplier
                packet.readUInt32(); // source item ID
                packet.readUInt32(); // quest flags
                // ... there are many more fields before title, let's try to read title string
                if (packet.getReadPos() + 1 < packet.getSize()) {
                    std::string title = packet.readString();
                    LOG_INFO("  Quest title: ", title);

                    // Update quest log entry with title
                    for (auto& q : questLog_) {
                        if (q.questId == questId) {
                            q.title = title;
                            LOG_INFO("Updated quest log entry ", questId, " with title: ", title);
                            break;
                        }
                    }
                }
            }

            break;
        }
        case Opcode::SMSG_QUESTLOG_FULL: {
            LOG_INFO("***** RECEIVED SMSG_QUESTLOG_FULL *****");
            LOG_INFO("  Packet size: ", packet.getSize());
            LOG_INFO("  Server uses SMSG_QUESTLOG_FULL for quest log sync!");
            // TODO: Parse quest log entries from this packet
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

        // ---- Teleport / Transfer ----
        case Opcode::MSG_MOVE_TELEPORT_ACK:
            handleTeleportAck(packet);
            break;
        case Opcode::SMSG_TRANSFER_PENDING: {
            // SMSG_TRANSFER_PENDING: uint32 mapId, then optional transport data
            uint32_t pendingMapId = packet.readUInt32();
            LOG_INFO("SMSG_TRANSFER_PENDING: mapId=", pendingMapId);
            // Optional: if remaining data, there's a transport entry + mapId
            if (packet.getReadPos() + 8 <= packet.getSize()) {
                uint32_t transportEntry = packet.readUInt32();
                uint32_t transportMapId = packet.readUInt32();
                LOG_INFO("  Transport entry=", transportEntry, " transportMapId=", transportMapId);
            }
            break;
        }
        case Opcode::SMSG_NEW_WORLD:
            handleNewWorld(packet);
            break;
        case Opcode::SMSG_TRANSFER_ABORTED: {
            uint32_t mapId = packet.readUInt32();
            uint8_t reason = (packet.getReadPos() < packet.getSize()) ? packet.readUInt8() : 0;
            LOG_WARNING("SMSG_TRANSFER_ABORTED: mapId=", mapId, " reason=", (int)reason);
            addSystemChatMessage("Transfer aborted.");
            break;
        }

        // ---- Taxi / Flight Paths ----
        case Opcode::SMSG_SHOWTAXINODES:
            handleShowTaxiNodes(packet);
            break;
        case Opcode::SMSG_ACTIVATETAXIREPLY:
        case Opcode::SMSG_ACTIVATETAXIREPLY_ALT:
            handleActivateTaxiReply(packet);
            break;
        case Opcode::SMSG_NEW_TAXI_PATH:
            // Empty packet - server signals a new flight path was learned
            // The actual node details come in the next SMSG_SHOWTAXINODES
            addSystemChatMessage("New flight path discovered!");
            break;

        // ---- Arena / Battleground ----
        case Opcode::SMSG_BATTLEFIELD_STATUS:
            handleBattlefieldStatus(packet);
            break;
        case Opcode::SMSG_BATTLEFIELD_LIST:
            LOG_INFO("Received SMSG_BATTLEFIELD_LIST");
            break;
        case Opcode::SMSG_BATTLEFIELD_PORT_DENIED:
            addSystemChatMessage("Battlefield port denied.");
            break;
        case Opcode::SMSG_REMOVED_FROM_PVP_QUEUE:
            addSystemChatMessage("You have been removed from the PvP queue.");
            break;
        case Opcode::SMSG_GROUP_JOINED_BATTLEGROUND:
            addSystemChatMessage("Your group has joined the battleground.");
            break;
        case Opcode::SMSG_JOINED_BATTLEGROUND_QUEUE:
            addSystemChatMessage("You have joined the battleground queue.");
            break;
        case Opcode::SMSG_BATTLEGROUND_PLAYER_JOINED:
            LOG_INFO("Battleground player joined");
            break;
        case Opcode::SMSG_BATTLEGROUND_PLAYER_LEFT:
            LOG_INFO("Battleground player left");
            break;
        case Opcode::SMSG_ARENA_TEAM_COMMAND_RESULT:
            handleArenaTeamCommandResult(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_QUERY_RESPONSE:
            handleArenaTeamQueryResponse(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_ROSTER:
            LOG_INFO("Received SMSG_ARENA_TEAM_ROSTER");
            break;
        case Opcode::SMSG_ARENA_TEAM_INVITE:
            handleArenaTeamInvite(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_EVENT:
            handleArenaTeamEvent(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_STATS:
            LOG_INFO("Received SMSG_ARENA_TEAM_STATS");
            break;
        case Opcode::SMSG_ARENA_ERROR:
            handleArenaError(packet);
            break;
        case Opcode::MSG_PVP_LOG_DATA:
            LOG_INFO("Received MSG_PVP_LOG_DATA");
            break;
        case Opcode::MSG_INSPECT_ARENA_TEAMS:
            LOG_INFO("Received MSG_INSPECT_ARENA_TEAMS");
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
            playerRace_ = character.race;
            break;
        }
    }

    // Store player GUID
    playerGuid = characterGuid;

    // Reset per-character state so previous character data doesn't bleed through
    inventory = Inventory();
    onlineItems_.clear();
    pendingItemQueries_.clear();
    equipSlotGuids_ = {};
    backpackSlotGuids_ = {};
    invSlotBase_ = -1;
    packSlotBase_ = -1;
    lastPlayerFields_.clear();
    onlineEquipDirty_ = false;
    playerMoneyCopper_ = 0;
    knownSpells.clear();
    spellCooldowns.clear();
    actionBar = {};
    playerAuras.clear();
    targetAuras.clear();
    playerXp_ = 0;
    playerNextLevelXp_ = 0;
    serverPlayerLevel_ = 1;
    playerSkills_.clear();
    questLog_.clear();
    npcQuestStatus_.clear();
    hostileAttackers_.clear();
    combatText.clear();
    autoAttacking = false;
    autoAttackTarget = 0;
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;
    castTimeTotal = 0.0f;
    playerDead_ = false;
    releasedSpirit_ = false;
    targetGuid = 0;
    focusGuid = 0;
    lastTargetGuid = 0;
    tabCycleStale = true;
    entityManager = EntityManager();

    // Build CMSG_PLAYER_LOGIN packet
    auto packet = PlayerLoginPacket::build(characterGuid);

    // Send packet
    socket->send(packet);

    setState(WorldState::ENTERING_WORLD);
    LOG_INFO("CMSG_PLAYER_LOGIN sent, entering world...");
}

void GameHandler::handleLoginSetTimeSpeed(network::Packet& packet) {
    // SMSG_LOGIN_SETTIMESPEED (0x042)
    // Structure: uint32 gameTime, float timeScale
    // gameTime: Game time in seconds since epoch
    // timeScale: Time speed multiplier (typically 0.0166 for 1 day = 1 hour)

    if (packet.getSize() < 8) {
        LOG_WARNING("SMSG_LOGIN_SETTIMESPEED: packet too small (", packet.getSize(), " bytes)");
        return;
    }

    uint32_t gameTimePacked = packet.readUInt32();
    float timeScale = packet.readFloat();

    // Store for celestial/sky system use
    gameTime_ = static_cast<float>(gameTimePacked);
    timeSpeed_ = timeScale;

    LOG_INFO("Server time: gameTime=", gameTime_, "s, timeSpeed=", timeSpeed_);
    LOG_INFO("  (1 game day = ", (1.0f / timeSpeed_) / 60.0f, " real minutes)");
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

    // Successfully entered the world (or teleported)
    currentMapId_ = data.mapId;
    setState(WorldState::IN_WORLD);

    LOG_INFO("========================================");
    LOG_INFO("   SUCCESSFULLY ENTERED WORLD!");
    LOG_INFO("========================================");
    LOG_INFO("Map ID: ", data.mapId);
    LOG_INFO("Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("Orientation: ", data.orientation, " radians");
    LOG_INFO("Player is now in the game world");

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

    // Block movement during taxi flight
    if (onTaxiFlight_) {
        // If taxi visuals are already gone, clear taxi state to avoid stuck movement.
        if (!taxiMountActive_ && !taxiClientActive_ && taxiClientPath_.empty()) {
            onTaxiFlight_ = false;
            LOG_INFO("Cleared stale taxi state in sendMovement");
        } else {
            return;
        }
    }
    if (resurrectPending_) return;

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

    UpdateObjectData data;
    if (!UpdateObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_UPDATE_OBJECT");
        return;
    }

    // Process out-of-range objects first
    for (uint64_t guid : data.outOfRangeGuids) {
        if (entityManager.hasEntity(guid)) {
            LOG_INFO("Entity went out of range: 0x", std::hex, guid, std::dec);
            // Trigger despawn callbacks before removing entity
            auto entity = entityManager.getEntity(guid);
            if (entity) {
                if (entity->getType() == ObjectType::UNIT && creatureDespawnCallback_) {
                    creatureDespawnCallback_(guid);
                } else if (entity->getType() == ObjectType::GAMEOBJECT && gameObjectDespawnCallback_) {
                    gameObjectDespawnCallback_(guid);
                }
            }
            transportGuids_.erase(guid);
            if (playerTransportGuid_ == guid) {
                playerTransportGuid_ = 0;
                playerTransportOffset_ = glm::vec3(0.0f);
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
                        break;

                    case ObjectType::UNIT:
                        entity = std::make_shared<Unit>(block.guid);
                        break;

                    case ObjectType::GAMEOBJECT:
                        entity = std::make_shared<GameObject>(block.guid);
                        break;

                    default:
                        entity = std::make_shared<Entity>(block.guid);
                        entity->setType(block.objectType);
                        break;
                }

                // Set position from movement block (server → canonical)
                if (block.hasMovement) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("  Position: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                    if (block.guid == playerGuid && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
                        serverRunSpeed_ = block.runSpeed;
                    }
                    // Track player-on-transport state
                    if (block.guid == playerGuid) {
                        if (block.onTransport) {
                            playerTransportGuid_ = block.transportGuid;
                            playerTransportOffset_ = glm::vec3(block.transportX, block.transportY, block.transportZ);
                            LOG_INFO("Player on transport: 0x", std::hex, playerTransportGuid_, std::dec,
                                     " offset=(", playerTransportOffset_.x, ", ", playerTransportOffset_.y, ", ", playerTransportOffset_.z, ")");
                        } else {
                            if (playerTransportGuid_ != 0) {
                                LOG_INFO("Player left transport");
                            }
                            playerTransportGuid_ = 0;
                            playerTransportOffset_ = glm::vec3(0.0f);
                        }
                    }
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
                    constexpr uint32_t UNIT_DYNFLAG_DEAD = 0x0008;
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
                            case 147: unit->setDynamicFlags(val); break; // UNIT_DYNAMIC_FLAGS
                            case 54: unit->setLevel(val); break;
                            case 67: unit->setDisplayId(val); break;  // UNIT_FIELD_DISPLAYID
                            case 69: // UNIT_FIELD_MOUNTDISPLAYID
                                if (block.guid == playerGuid) {
                                    uint32_t old = currentMountDisplayId_;
                                    currentMountDisplayId_ = val;
                                    if (val != old && mountCallback_) mountCallback_(val);
                                    if (old != 0 && val == 0) {
                                        for (auto& a : playerAuras)
                                            if (!a.isEmpty() && a.maxDurationMs < 0) a = AuraSlot{};
                                    }
                                }
                                unit->setMountDisplayId(val);
                                break;
                            case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                            default: break;
                        }
                    }
                    if (block.guid == playerGuid) {
                        constexpr uint32_t UNIT_FLAG_TAXI_FLIGHT = 0x00000100;
                        if ((unit->getUnitFlags() & UNIT_FLAG_TAXI_FLIGHT) != 0 && !onTaxiFlight_ && taxiLandingCooldown_ <= 0.0f) {
                            onTaxiFlight_ = true;
                            applyTaxiMountForCurrentNode();
                        }
                    }
                    if (block.guid == playerGuid &&
                        (unit->getDynamicFlags() & UNIT_DYNFLAG_DEAD) != 0) {
                        playerDead_ = true;
                        LOG_INFO("Player logged in dead (dynamic flags)");
                    }
                    // Detect ghost state on login via PLAYER_FLAGS (field 150)
                    if (block.guid == playerGuid) {
                        constexpr uint32_t PLAYER_FLAGS_IDX = 150;  // UNIT_END(148) + 2
                        constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
                        auto pfIt = block.fields.find(PLAYER_FLAGS_IDX);
                        if (pfIt != block.fields.end() && (pfIt->second & PLAYER_FLAGS_GHOST) != 0) {
                            releasedSpirit_ = true;
                            playerDead_ = true;
                            LOG_INFO("Player logged in as ghost (PLAYER_FLAGS)");
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
                // Extract displayId and entry for gameobjects (3.3.5a: GAMEOBJECT_DISPLAYID = field 8)
                if (block.objectType == ObjectType::GAMEOBJECT) {
                    auto go = std::static_pointer_cast<GameObject>(entity);
                    auto itDisp = block.fields.find(8);
                    if (itDisp != block.fields.end()) {
                        go->setDisplayId(itDisp->second);
                    }
                    // Extract entry and query name (OBJECT_FIELD_ENTRY = index 3)
                    auto itEntry = block.fields.find(3);
                    if (itEntry != block.fields.end() && itEntry->second != 0) {
                        go->setEntry(itEntry->second);
                        auto cacheIt = gameObjectInfoCache_.find(itEntry->second);
                        if (cacheIt != gameObjectInfoCache_.end()) {
                            go->setName(cacheIt->second.name);
                        }
                        queryGameObjectInfo(itEntry->second, block.guid);
                    }
                    // Detect transport GameObjects via UPDATEFLAG_TRANSPORT (0x0002)
                    if (block.updateFlags & 0x0002) {
                        transportGuids_.insert(block.guid);
                        LOG_INFO("Detected transport GameObject: 0x", std::hex, block.guid, std::dec,
                                 " displayId=", go->getDisplayId(),
                                 " pos=(", go->getX(), ", ", go->getY(), ", ", go->getZ(), ")");
                        // Note: TransportSpawnCallback will be invoked from Application after WMO instance is created
                    }
                    if (go->getDisplayId() != 0 && gameObjectSpawnCallback_) {
                        gameObjectSpawnCallback_(block.guid, go->getEntry(), go->getDisplayId(),
                            go->getX(), go->getY(), go->getZ(), go->getOrientation());
                    }
                    // Fire transport move callback for transports (position update on re-creation)
                    if (transportGuids_.count(block.guid) && transportMoveCallback_) {
                        transportMoveCallback_(block.guid,
                            go->getX(), go->getY(), go->getZ(), go->getOrientation());
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

                // Extract XP / inventory slot / skill fields for player entity
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    // Store baseline snapshot on first update
                    static bool baselineStored = false;
                    static std::map<uint16_t, uint32_t> baselineFields;

                    if (!baselineStored) {
                        baselineFields = block.fields;
                        baselineStored = true;
                        LOG_INFO("===== BASELINE PLAYER FIELDS STORED =====");
                        LOG_INFO("  Total fields: ", block.fields.size());
                    }

                    // Diff against baseline to find changes
                    std::vector<uint16_t> changedIndices;
                    std::vector<uint16_t> newIndices;
                    std::vector<uint16_t> removedIndices;

                    for (const auto& [idx, val] : block.fields) {
                        auto it = baselineFields.find(idx);
                        if (it == baselineFields.end()) {
                            newIndices.push_back(idx);
                        } else if (it->second != val) {
                            changedIndices.push_back(idx);
                        }
                    }

                    for (const auto& [idx, val] : baselineFields) {
                        if (block.fields.find(idx) == block.fields.end()) {
                            removedIndices.push_back(idx);
                        }
                    }

                    lastPlayerFields_ = block.fields;
                    detectInventorySlotBases(block.fields);

                    // Debug: Show field changes
                    LOG_INFO("Player update with ", block.fields.size(), " fields");

                    if (!changedIndices.empty() || !newIndices.empty() || !removedIndices.empty()) {
                        LOG_INFO("  ===== FIELD CHANGES DETECTED =====");
                        if (!changedIndices.empty()) {
                            LOG_INFO("  Changed fields (", changedIndices.size(), "):");
                            std::sort(changedIndices.begin(), changedIndices.end());
                            for (size_t i = 0; i < std::min(size_t(30), changedIndices.size()); ++i) {
                                uint16_t idx = changedIndices[i];
                                uint32_t oldVal = baselineFields[idx];
                                uint32_t newVal = block.fields.at(idx);
                                LOG_INFO("    [", idx, "]: ", oldVal, " -> ", newVal,
                                         " (0x", std::hex, oldVal, " -> 0x", newVal, std::dec, ")");
                            }
                            if (changedIndices.size() > 30) {
                                LOG_INFO("    ... (", changedIndices.size() - 30, " more)");
                            }
                        }
                        if (!newIndices.empty()) {
                            LOG_INFO("  New fields (", newIndices.size(), "):");
                            std::sort(newIndices.begin(), newIndices.end());
                            for (size_t i = 0; i < std::min(size_t(20), newIndices.size()); ++i) {
                                uint16_t idx = newIndices[i];
                                uint32_t val = block.fields.at(idx);
                                LOG_INFO("    [", idx, "]: ", val, " (0x", std::hex, val, std::dec, ")");
                            }
                            if (newIndices.size() > 20) {
                                LOG_INFO("    ... (", newIndices.size() - 20, " more)");
                            }
                        }
                        if (!removedIndices.empty()) {
                            LOG_INFO("  Removed fields (", removedIndices.size(), "):");
                            std::sort(removedIndices.begin(), removedIndices.end());
                            for (size_t i = 0; i < std::min(size_t(20), removedIndices.size()); ++i) {
                                uint16_t idx = removedIndices[i];
                                uint32_t val = baselineFields.at(idx);
                                LOG_INFO("    [", idx, "]: was ", val, " (0x", std::hex, val, std::dec, ")");
                            }
                        }
                    }

                    uint16_t maxField = 0;
                    for (const auto& [key, val] : block.fields) {
                        if (key > maxField) maxField = key;
                    }

                    LOG_INFO("  Highest field index: ", maxField);

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
                        else if (key == 1170) {
                            playerMoneyCopper_ = val;
                            LOG_INFO("Money set from update fields: ", val, " copper");
                        }  // PLAYER_FIELD_COINAGE
                        // Parse quest log fields (PLAYER_QUEST_LOG_1_1 = UNIT_END + 10 = 158, stride 5)
                        // Quest slots: 158, 163, 168, 173, ... (25 slots max = up to index 278)
                        else if (key >= 158 && key < 283 && (key - 158) % 5 == 0) {
                            uint32_t questId = val;
                            if (questId != 0) {
                                // Check if quest is already in log
                                bool found = false;
                                for (auto& q : questLog_) {
                                    if (q.questId == questId) {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found) {
                                    // Add quest to log and request quest details
                                    QuestLogEntry entry;
                                    entry.questId = questId;
                                    entry.complete = false;  // Will be updated by gossip or quest status packets
                                    entry.title = "Quest #" + std::to_string(questId);
                                    questLog_.push_back(entry);
                                    LOG_INFO("Found quest in update fields: ", questId);

                                    // Request quest details from server
                                    if (socket) {
                                        network::Packet qPkt(static_cast<uint16_t>(Opcode::CMSG_QUEST_QUERY));
                                        qPkt.writeUInt32(questId);
                                        socket->send(qPkt);
                                    }
                                }
                            }
                        }
                    }
                    if (applyInventoryFields(block.fields)) slotsChanged = true;
                    if (slotsChanged) rebuildOnlineInventory();
                    extractSkillFields(lastPlayerFields_);
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
                        constexpr uint32_t UNIT_DYNFLAG_DEAD = 0x0008;
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
                                            releasedSpirit_ = false;
                                            stopAutoAttack();
                                            LOG_INFO("Player died!");
                                        }
                                        // Trigger death animation for NPC units
                                        if (entity->getType() == ObjectType::UNIT && npcDeathCallback_) {
                                            npcDeathCallback_(block.guid);
                                        }
                                    } else if (oldHealth == 0 && val > 0) {
                                        // Player resurrection or ghost form
                                        if (block.guid == playerGuid) {
                                            playerDead_ = false;
                                            if (!releasedSpirit_) {
                                                LOG_INFO("Player resurrected!");
                                            } else {
                                                LOG_INFO("Player entered ghost form");
                                            }
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
                                case 147: {
                                    uint32_t oldDyn = unit->getDynamicFlags();
                                    unit->setDynamicFlags(val);
                                    if (block.guid == playerGuid) {
                                        bool wasDead = (oldDyn & UNIT_DYNFLAG_DEAD) != 0;
                                        bool nowDead = (val & UNIT_DYNFLAG_DEAD) != 0;
                                        if (!wasDead && nowDead) {
                                            playerDead_ = true;
                                            releasedSpirit_ = false;
                                            LOG_INFO("Player died (dynamic flags)");
                                        } else if (wasDead && !nowDead) {
                                            playerDead_ = false;
                                            releasedSpirit_ = false;
                                            LOG_INFO("Player resurrected (dynamic flags)");
                                        }
                                    }
                                    break;
                                }
                                case 54: unit->setLevel(val); break;
                                case 55:  // UNIT_FIELD_FACTIONTEMPLATE
                                    unit->setFactionTemplate(val);
                                    unit->setHostile(isHostileFaction(val));
                                    break;
                                case 67: unit->setDisplayId(val); break;  // UNIT_FIELD_DISPLAYID
                                case 69: // UNIT_FIELD_MOUNTDISPLAYID
                                    if (block.guid == playerGuid) {
                                        uint32_t old = currentMountDisplayId_;
                                        currentMountDisplayId_ = val;
                                        if (val != old && mountCallback_) mountCallback_(val);
                                        if (old != 0 && val == 0) {
                                            for (auto& a : playerAuras)
                                                if (!a.isEmpty() && a.maxDurationMs < 0) a = AuraSlot{};
                                        }
                                    }
                                    unit->setMountDisplayId(val);
                                    break;
                                case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                                default: break;
                            }
                        }
                    }
                    // Update XP / inventory slot / skill fields for player entity
                    if (block.guid == playerGuid) {
                        if (block.hasMovement && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
                            serverRunSpeed_ = block.runSpeed;
                        }
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
                            else if (key == 150) {  // PLAYER_FLAGS (UNIT_END+2)
                                constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
                                bool wasGhost = releasedSpirit_;
                                bool nowGhost = (val & PLAYER_FLAGS_GHOST) != 0;
                                if (!wasGhost && nowGhost) {
                                    releasedSpirit_ = true;
                                    LOG_INFO("Player entered ghost form (PLAYER_FLAGS)");
                                } else if (wasGhost && !nowGhost) {
                                    releasedSpirit_ = false;
                                    playerDead_ = false;
                                    repopPending_ = false;
                                    resurrectPending_ = false;
                                    LOG_INFO("Player resurrected (PLAYER_FLAGS ghost cleared)");
                                }
                            }
                        }
                        if (applyInventoryFields(block.fields)) slotsChanged = true;
                        if (slotsChanged) rebuildOnlineInventory();
                        extractSkillFields(lastPlayerFields_);
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
                }
                break;
            }

            case UpdateType::MOVEMENT: {
                // Diagnostic: Log if we receive MOVEMENT blocks for transports
                if (transportGuids_.count(block.guid)) {
                    LOG_INFO("MOVEMENT update for transport 0x", std::hex, block.guid, std::dec,
                             " pos=(", block.x, ", ", block.y, ", ", block.z, ")");
                }

                // Update entity position (server → canonical)
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("Updated entity position: 0x", std::hex, block.guid, std::dec);
                    if (block.guid == playerGuid) {
                        movementInfo.x = pos.x;
                        movementInfo.y = pos.y;
                        movementInfo.z = pos.z;
                        movementInfo.orientation = block.orientation;
                    }

                    // Fire transport move callback if this is a known transport
                    if (transportGuids_.count(block.guid) && transportMoveCallback_) {
                        transportMoveCallback_(block.guid, pos.x, pos.y, pos.z, block.orientation);
                    }
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
    // Entity count logging disabled

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
    // Entity count logging disabled
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

    // Add local echo so the player sees their own message immediately
    MessageChatData echo;
    echo.senderGuid = playerGuid;
    echo.language = language;
    echo.message = message;

    // Look up player name
    auto nameIt = playerNameCache.find(playerGuid);
    if (nameIt != playerNameCache.end()) {
        echo.senderName = nameIt->second;
    }

    if (type == ChatType::WHISPER) {
        echo.type = ChatType::WHISPER_INFORM;
        echo.senderName = target;  // "To [target]: message"
    } else {
        echo.type = type;
    }

    addLocalChatMessage(echo);
}

void GameHandler::handleMessageChat(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_MESSAGECHAT");

    MessageChatData data;
    if (!MessageChatParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MESSAGECHAT");
        return;
    }

    // Skip server echo of our own messages (we already added a local echo)
    if (data.senderGuid == playerGuid && data.senderGuid != 0) {
        // Still track whisper sender for /r even if it's our own whisper-inform
        if (data.type == ChatType::WHISPER && !data.senderName.empty()) {
            lastWhisperSender_ = data.senderName;
        }
        return;
    }

    // Add to chat history
    chatHistory.push_back(data);

    // Limit chat history size
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.erase(chatHistory.begin());
    }

    // Track whisper sender for /r command
    if (data.type == ChatType::WHISPER && !data.senderName.empty()) {
        lastWhisperSender_ = data.senderName;
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

    // Save previous target
    if (targetGuid != 0) {
        lastTargetGuid = targetGuid;
    }

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

void GameHandler::setFocus(uint64_t guid) {
    focusGuid = guid;
    if (guid != 0) {
        auto entity = entityManager.getEntity(guid);
        if (entity) {
            std::string name = "Unknown";
            if (entity->getType() == ObjectType::PLAYER) {
                auto player = std::dynamic_pointer_cast<Player>(entity);
                if (player && !player->getName().empty()) {
                    name = player->getName();
                }
            }
            addSystemChatMessage("Focus set: " + name);
            LOG_INFO("Focus set: 0x", std::hex, guid, std::dec);
        }
    }
}

void GameHandler::clearFocus() {
    if (focusGuid != 0) {
        addSystemChatMessage("Focus cleared.");
        LOG_INFO("Focus cleared");
    }
    focusGuid = 0;
}

std::shared_ptr<Entity> GameHandler::getFocus() const {
    if (focusGuid == 0) return nullptr;
    return entityManager.getEntity(focusGuid);
}

void GameHandler::targetLastTarget() {
    if (lastTargetGuid == 0) {
        addSystemChatMessage("No previous target.");
        return;
    }

    // Swap current and last target
    uint64_t temp = targetGuid;
    setTarget(lastTargetGuid);
    lastTargetGuid = temp;
}

void GameHandler::targetEnemy(bool reverse) {
    // Get list of hostile entities
    std::vector<uint64_t> hostiles;
    auto& entities = entityManager.getEntities();

    for (const auto& [guid, entity] : entities) {
        if (entity->getType() == ObjectType::UNIT) {
            // Check if hostile (this is simplified - would need faction checking)
            auto unit = std::dynamic_pointer_cast<Unit>(entity);
            if (unit && guid != playerGuid) {
                hostiles.push_back(guid);
            }
        }
    }

    if (hostiles.empty()) {
        addSystemChatMessage("No enemies in range.");
        return;
    }

    // Find current target in list
    auto it = std::find(hostiles.begin(), hostiles.end(), targetGuid);

    if (it == hostiles.end()) {
        // Not currently targeting a hostile, target first one
        setTarget(reverse ? hostiles.back() : hostiles.front());
    } else {
        // Cycle to next/previous
        if (reverse) {
            if (it == hostiles.begin()) {
                setTarget(hostiles.back());
            } else {
                setTarget(*(--it));
            }
        } else {
            ++it;
            if (it == hostiles.end()) {
                setTarget(hostiles.front());
            } else {
                setTarget(*it);
            }
        }
    }
}

void GameHandler::targetFriend(bool reverse) {
    // Get list of friendly entities (players)
    std::vector<uint64_t> friendlies;
    auto& entities = entityManager.getEntities();

    for (const auto& [guid, entity] : entities) {
        if (entity->getType() == ObjectType::PLAYER && guid != playerGuid) {
            friendlies.push_back(guid);
        }
    }

    if (friendlies.empty()) {
        addSystemChatMessage("No friendly targets in range.");
        return;
    }

    // Find current target in list
    auto it = std::find(friendlies.begin(), friendlies.end(), targetGuid);

    if (it == friendlies.end()) {
        // Not currently targeting a friend, target first one
        setTarget(reverse ? friendlies.back() : friendlies.front());
    } else {
        // Cycle to next/previous
        if (reverse) {
            if (it == friendlies.begin()) {
                setTarget(friendlies.back());
            } else {
                setTarget(*(--it));
            }
        } else {
            ++it;
            if (it == friendlies.end()) {
                setTarget(friendlies.front());
            } else {
                setTarget(*it);
            }
        }
    }
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

void GameHandler::addFriend(const std::string& playerName, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot add friend: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = AddFriendPacket::build(playerName, note);
    socket->send(packet);
    addSystemChatMessage("Sending friend request to " + playerName + "...");
    LOG_INFO("Sent friend request to: ", playerName);
}

void GameHandler::removeFriend(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot remove friend: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = friendsCache.find(playerName);
    if (it == friendsCache.end()) {
        addSystemChatMessage(playerName + " is not in your friends list.");
        LOG_WARNING("Friend not found in cache: ", playerName);
        return;
    }

    auto packet = DelFriendPacket::build(it->second);
    socket->send(packet);
    addSystemChatMessage("Removing " + playerName + " from friends list...");
    LOG_INFO("Sent remove friend request for: ", playerName, " (GUID: 0x", std::hex, it->second, std::dec, ")");
}

void GameHandler::setFriendNote(const std::string& playerName, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set friend note: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = friendsCache.find(playerName);
    if (it == friendsCache.end()) {
        addSystemChatMessage(playerName + " is not in your friends list.");
        return;
    }

    auto packet = SetContactNotesPacket::build(it->second, note);
    socket->send(packet);
    addSystemChatMessage("Updated note for " + playerName);
    LOG_INFO("Set friend note for: ", playerName);
}

void GameHandler::randomRoll(uint32_t minRoll, uint32_t maxRoll) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot roll: not in world or not connected");
        return;
    }

    if (minRoll > maxRoll) {
        std::swap(minRoll, maxRoll);
    }

    if (maxRoll > 10000) {
        maxRoll = 10000;  // Cap at reasonable value
    }

    auto packet = RandomRollPacket::build(minRoll, maxRoll);
    socket->send(packet);
    LOG_INFO("Rolled ", minRoll, "-", maxRoll);
}

void GameHandler::addIgnore(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot add ignore: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = AddIgnorePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Adding " + playerName + " to ignore list...");
    LOG_INFO("Sent ignore request for: ", playerName);
}

void GameHandler::removeIgnore(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot remove ignore: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = ignoreCache.find(playerName);
    if (it == ignoreCache.end()) {
        addSystemChatMessage(playerName + " is not in your ignore list.");
        LOG_WARNING("Ignored player not found in cache: ", playerName);
        return;
    }

    auto packet = DelIgnorePacket::build(it->second);
    socket->send(packet);
    addSystemChatMessage("Removing " + playerName + " from ignore list...");
    ignoreCache.erase(it);
    LOG_INFO("Sent remove ignore request for: ", playerName, " (GUID: 0x", std::hex, it->second, std::dec, ")");
}

void GameHandler::requestLogout() {
    if (!socket) {
        LOG_WARNING("Cannot logout: not connected");
        return;
    }

    if (loggingOut_) {
        addSystemChatMessage("Already logging out.");
        return;
    }

    auto packet = LogoutRequestPacket::build();
    socket->send(packet);
    loggingOut_ = true;
    LOG_INFO("Sent logout request");
}

void GameHandler::cancelLogout() {
    if (!socket) {
        LOG_WARNING("Cannot cancel logout: not connected");
        return;
    }

    if (!loggingOut_) {
        addSystemChatMessage("Not currently logging out.");
        return;
    }

    auto packet = LogoutCancelPacket::build();
    socket->send(packet);
    loggingOut_ = false;
    addSystemChatMessage("Logout cancelled.");
    LOG_INFO("Cancelled logout");
}

void GameHandler::setStandState(uint8_t standState) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot change stand state: not in world or not connected");
        return;
    }

    auto packet = StandStateChangePacket::build(standState);
    socket->send(packet);
    LOG_INFO("Changed stand state to: ", (int)standState);
}

void GameHandler::toggleHelm() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle helm: not in world or not connected");
        return;
    }

    helmVisible_ = !helmVisible_;
    auto packet = ShowingHelmPacket::build(helmVisible_);
    socket->send(packet);
    addSystemChatMessage(helmVisible_ ? "Helm is now visible." : "Helm is now hidden.");
    LOG_INFO("Helm visibility toggled: ", helmVisible_);
}

void GameHandler::toggleCloak() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle cloak: not in world or not connected");
        return;
    }

    cloakVisible_ = !cloakVisible_;
    auto packet = ShowingCloakPacket::build(cloakVisible_);
    socket->send(packet);
    addSystemChatMessage(cloakVisible_ ? "Cloak is now visible." : "Cloak is now hidden.");
    LOG_INFO("Cloak visibility toggled: ", cloakVisible_);
}

void GameHandler::followTarget() {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot follow: not in world");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target someone to follow.");
        return;
    }

    auto target = getTarget();
    if (!target) {
        addSystemChatMessage("Invalid target.");
        return;
    }

    // Set follow target
    followTargetGuid_ = targetGuid;

    // Get target name
    std::string targetName = "Target";
    if (target->getType() == ObjectType::PLAYER) {
        auto player = std::static_pointer_cast<Player>(target);
        if (!player->getName().empty()) {
            targetName = player->getName();
        }
    } else if (target->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(target);
        targetName = unit->getName();
    }

    addSystemChatMessage("Now following " + targetName + ".");
    LOG_INFO("Following target: ", targetName, " (GUID: 0x", std::hex, targetGuid, std::dec, ")");
}

void GameHandler::assistTarget() {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot assist: not in world");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target someone to assist.");
        return;
    }

    auto target = getTarget();
    if (!target) {
        addSystemChatMessage("Invalid target.");
        return;
    }

    // Get target name
    std::string targetName = "Target";
    if (target->getType() == ObjectType::PLAYER) {
        auto player = std::static_pointer_cast<Player>(target);
        if (!player->getName().empty()) {
            targetName = player->getName();
        }
    } else if (target->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(target);
        targetName = unit->getName();
    }

    // Try to read target GUID from update fields (UNIT_FIELD_TARGET)
    // Field offset 6 is typically UNIT_FIELD_TARGET in 3.3.5a
    uint64_t assistTargetGuid = 0;
    const auto& fields = target->getFields();
    auto it = fields.find(6);
    if (it != fields.end()) {
        // Low 32 bits
        assistTargetGuid = it->second;
        // Try to get high 32 bits from next field
        auto it2 = fields.find(7);
        if (it2 != fields.end()) {
            assistTargetGuid |= (static_cast<uint64_t>(it2->second) << 32);
        }
    }

    if (assistTargetGuid == 0) {
        addSystemChatMessage(targetName + " has no target.");
        LOG_INFO("Assist: ", targetName, " has no target");
        return;
    }

    // Set our target to their target
    setTarget(assistTargetGuid);
    LOG_INFO("Assisting ", targetName, ", now targeting GUID: 0x", std::hex, assistTargetGuid, std::dec);
}

void GameHandler::togglePvp() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle PvP: not in world or not connected");
        return;
    }

    auto packet = TogglePvpPacket::build();
    socket->send(packet);
    // Check current PVP state from player's UNIT_FIELD_FLAGS (index 59)
    // UNIT_FLAG_PVP = 0x00001000
    auto entity = entityManager.getEntity(playerGuid);
    bool currentlyPvp = false;
    if (entity) {
        currentlyPvp = (entity->getField(59) & 0x00001000) != 0;
    }
    // We're toggling, so report the NEW state
    if (currentlyPvp) {
        addSystemChatMessage("PvP flag disabled.");
    } else {
        addSystemChatMessage("PvP flag enabled.");
    }
    LOG_INFO("Toggled PvP flag");
}

void GameHandler::requestGuildInfo() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request guild info: not in world or not connected");
        return;
    }

    auto packet = GuildInfoPacket::build();
    socket->send(packet);
    LOG_INFO("Requested guild info");
}

void GameHandler::requestGuildRoster() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request guild roster: not in world or not connected");
        return;
    }

    auto packet = GuildRosterPacket::build();
    socket->send(packet);
    addSystemChatMessage("Requesting guild roster...");
    LOG_INFO("Requested guild roster");
}

void GameHandler::setGuildMotd(const std::string& motd) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set guild MOTD: not in world or not connected");
        return;
    }

    auto packet = GuildMotdPacket::build(motd);
    socket->send(packet);
    addSystemChatMessage("Guild MOTD updated.");
    LOG_INFO("Set guild MOTD: ", motd);
}

void GameHandler::promoteGuildMember(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot promote guild member: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildPromotePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Promoting " + playerName + "...");
    LOG_INFO("Promoting guild member: ", playerName);
}

void GameHandler::demoteGuildMember(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot demote guild member: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildDemotePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Demoting " + playerName + "...");
    LOG_INFO("Demoting guild member: ", playerName);
}

void GameHandler::leaveGuild() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot leave guild: not in world or not connected");
        return;
    }

    auto packet = GuildLeavePacket::build();
    socket->send(packet);
    addSystemChatMessage("Leaving guild...");
    LOG_INFO("Leaving guild");
}

void GameHandler::inviteToGuild(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot invite to guild: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildInvitePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Inviting " + playerName + " to guild...");
    LOG_INFO("Inviting to guild: ", playerName);
}

void GameHandler::initiateReadyCheck() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot initiate ready check: not in world or not connected");
        return;
    }

    if (!isInGroup()) {
        addSystemChatMessage("You must be in a group to initiate a ready check.");
        return;
    }

    auto packet = ReadyCheckPacket::build();
    socket->send(packet);
    addSystemChatMessage("Ready check initiated.");
    LOG_INFO("Initiated ready check");
}

void GameHandler::respondToReadyCheck(bool ready) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot respond to ready check: not in world or not connected");
        return;
    }

    auto packet = ReadyCheckConfirmPacket::build(ready);
    socket->send(packet);
    addSystemChatMessage(ready ? "You are ready." : "You are not ready.");
    LOG_INFO("Responded to ready check: ", ready ? "ready" : "not ready");
}

void GameHandler::forfeitDuel() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot forfeit duel: not in world or not connected");
        return;
    }

    auto packet = DuelCancelPacket::build();
    socket->send(packet);
    addSystemChatMessage("You have forfeited the duel.");
    LOG_INFO("Forfeited duel");
}

void GameHandler::toggleAfk(const std::string& message) {
    afkStatus_ = !afkStatus_;
    afkMessage_ = message;

    if (afkStatus_) {
        if (message.empty()) {
            addSystemChatMessage("You are now AFK.");
        } else {
            addSystemChatMessage("You are now AFK: " + message);
        }
        // If DND was active, turn it off
        if (dndStatus_) {
            dndStatus_ = false;
            dndMessage_.clear();
        }
    } else {
        addSystemChatMessage("You are no longer AFK.");
        afkMessage_.clear();
    }

    LOG_INFO("AFK status: ", afkStatus_, ", message: ", message);
}

void GameHandler::toggleDnd(const std::string& message) {
    dndStatus_ = !dndStatus_;
    dndMessage_ = message;

    if (dndStatus_) {
        if (message.empty()) {
            addSystemChatMessage("You are now DND (Do Not Disturb).");
        } else {
            addSystemChatMessage("You are now DND: " + message);
        }
        // If AFK was active, turn it off
        if (afkStatus_) {
            afkStatus_ = false;
            afkMessage_.clear();
        }
    } else {
        addSystemChatMessage("You are no longer DND.");
        dndMessage_.clear();
    }

    LOG_INFO("DND status: ", dndStatus_, ", message: ", message);
}

void GameHandler::replyToLastWhisper(const std::string& message) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot send whisper: not in world or not connected");
        return;
    }

    if (lastWhisperSender_.empty()) {
        addSystemChatMessage("No one has whispered you yet.");
        return;
    }

    if (message.empty()) {
        addSystemChatMessage("You must specify a message to send.");
        return;
    }

    // Send whisper using the standard message chat function
    sendChatMessage(ChatType::WHISPER, message, lastWhisperSender_);
    LOG_INFO("Replied to ", lastWhisperSender_, ": ", message);
}

void GameHandler::uninvitePlayer(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot uninvite player: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name to uninvite.");
        return;
    }

    auto packet = GroupUninvitePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Removed " + playerName + " from the group.");
    LOG_INFO("Uninvited player: ", playerName);
}

void GameHandler::leaveParty() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot leave party: not in world or not connected");
        return;
    }

    auto packet = GroupDisbandPacket::build();
    socket->send(packet);
    addSystemChatMessage("You have left the group.");
    LOG_INFO("Left party/raid");
}

void GameHandler::setMainTank(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set main tank: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must have a target selected.");
        return;
    }

    // Main tank uses index 0
    auto packet = RaidTargetUpdatePacket::build(0, targetGuid);
    socket->send(packet);
    addSystemChatMessage("Main tank set.");
    LOG_INFO("Set main tank: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::setMainAssist(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set main assist: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must have a target selected.");
        return;
    }

    // Main assist uses index 1
    auto packet = RaidTargetUpdatePacket::build(1, targetGuid);
    socket->send(packet);
    addSystemChatMessage("Main assist set.");
    LOG_INFO("Set main assist: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::clearMainTank() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot clear main tank: not in world or not connected");
        return;
    }

    // Clear main tank by setting GUID to 0
    auto packet = RaidTargetUpdatePacket::build(0, 0);
    socket->send(packet);
    addSystemChatMessage("Main tank cleared.");
    LOG_INFO("Cleared main tank");
}

void GameHandler::clearMainAssist() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot clear main assist: not in world or not connected");
        return;
    }

    // Clear main assist by setting GUID to 0
    auto packet = RaidTargetUpdatePacket::build(1, 0);
    socket->send(packet);
    addSystemChatMessage("Main assist cleared.");
    LOG_INFO("Cleared main assist");
}

void GameHandler::requestRaidInfo() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request raid info: not in world or not connected");
        return;
    }

    auto packet = RequestRaidInfoPacket::build();
    socket->send(packet);
    addSystemChatMessage("Requesting raid lockout information...");
    LOG_INFO("Requested raid info");
}

void GameHandler::proposeDuel(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot propose duel: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to challenge to a duel.");
        return;
    }

    auto packet = DuelProposedPacket::build(targetGuid);
    socket->send(packet);
    addSystemChatMessage("You have challenged your target to a duel.");
    LOG_INFO("Proposed duel to target: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::initiateTrade(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot initiate trade: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to trade with.");
        return;
    }

    auto packet = InitiateTradePacket::build(targetGuid);
    socket->send(packet);
    addSystemChatMessage("Requesting trade with target.");
    LOG_INFO("Initiated trade with target: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopCasting() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot stop casting: not in world or not connected");
        return;
    }

    if (!casting) {
        return; // Not casting anything
    }

    // Send cancel cast packet with current spell ID
    auto packet = CancelCastPacket::build(currentCastSpellId);
    socket->send(packet);

    // Reset casting state
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;
    castTimeTotal = 0.0f;

    LOG_INFO("Cancelled spell cast");
}

void GameHandler::releaseSpirit() {
    if (socket && state == WorldState::IN_WORLD) {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (repopPending_ && now - static_cast<int64_t>(lastRepopRequestMs_) < 1000) {
            return;
        }
        auto packet = RepopRequestPacket::build();
        socket->send(packet);
        releasedSpirit_ = true;
        repopPending_ = true;
        lastRepopRequestMs_ = static_cast<uint64_t>(now);
        LOG_INFO("Sent CMSG_REPOP_REQUEST (Release Spirit)");
    }
}

void GameHandler::activateSpiritHealer(uint64_t npcGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingSpiritHealerGuid_ = npcGuid;
    auto packet = SpiritHealerActivatePacket::build(npcGuid);
    socket->send(packet);
    resurrectPending_ = true;
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE for 0x", std::hex, npcGuid, std::dec);
}

void GameHandler::acceptResurrect() {
    if (state != WorldState::IN_WORLD || !socket || !resurrectRequestPending_) return;
    // Send spirit healer activate (correct response to SMSG_SPIRIT_HEALER_CONFIRM)
    auto activate = SpiritHealerActivatePacket::build(resurrectCasterGuid_);
    socket->send(activate);
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE (0x21C) for 0x",
             std::hex, resurrectCasterGuid_, std::dec);
    resurrectRequestPending_ = false;
    resurrectPending_ = true;
}

void GameHandler::declineResurrect() {
    if (state != WorldState::IN_WORLD || !socket || !resurrectRequestPending_) return;
    auto resp = ResurrectResponsePacket::build(resurrectCasterGuid_, false);
    socket->send(resp);
    LOG_INFO("Sent CMSG_RESURRECT_RESPONSE (decline) for 0x",
             std::hex, resurrectCasterGuid_, std::dec);
    resurrectRequestPending_ = false;
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

void GameHandler::queryGameObjectInfo(uint32_t entry, uint64_t guid) {
    if (gameObjectInfoCache_.count(entry) || pendingGameObjectQueries_.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingGameObjectQueries_.insert(entry);
    auto packet = GameObjectQueryPacket::build(entry, guid);
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
// GameObject Query
// ============================================================

void GameHandler::handleGameObjectQueryResponse(network::Packet& packet) {
    GameObjectQueryResponseData data;
    if (!GameObjectQueryResponseParser::parse(packet, data)) return;

    pendingGameObjectQueries_.erase(data.entry);

    if (data.isValid()) {
        gameObjectInfoCache_[data.entry] = data;
        // Update all gameobject entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::GAMEOBJECT) {
                auto go = std::static_pointer_cast<GameObject>(entity);
                if (go->getEntry() == data.entry) {
                    go->setName(data.name);
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
    // Can't attack yourself
    if (targetGuid == playerGuid) return;

    // Dismount when entering combat
    if (isMounted()) {
        dismount();
    }
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

void GameHandler::autoTargetAttacker(uint64_t attackerGuid) {
    if (attackerGuid == 0 || attackerGuid == playerGuid) return;
    if (targetGuid != 0) return;
    if (!entityManager.hasEntity(attackerGuid)) return;
    setTarget(attackerGuid);
}

void GameHandler::handleAttackStart(network::Packet& packet) {
    AttackStartData data;
    if (!AttackStartParser::parse(packet, data)) return;

    if (data.attackerGuid == playerGuid) {
        autoAttacking = true;
        autoAttackTarget = data.victimGuid;
    } else if (data.victimGuid == playerGuid && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
        autoTargetAttacker(data.attackerGuid);

        // Play aggro sound when NPC attacks player
        if (npcAggroCallback_) {
            auto entity = entityManager.getEntity(data.attackerGuid);
            if (entity && entity->getType() == ObjectType::UNIT) {
                glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
                npcAggroCallback_(data.attackerGuid, pos);
            }
        }
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

void GameHandler::dismount() {
    if (!isMounted() || !socket) return;
    network::Packet pkt(static_cast<uint16_t>(Opcode::CMSG_CANCEL_MOUNT_AURA));
    socket->send(pkt);
    LOG_INFO("Sent CMSG_CANCEL_MOUNT_AURA");
}

void GameHandler::handleForceRunSpeedChange(network::Packet& packet) {
    // Packed GUID
    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    // uint32 counter
    uint32_t counter = packet.readUInt32();

    // Determine format from remaining bytes:
    //   5 bytes remaining = uint8(1) + float(4)  — standard 3.3.5a
    //   8 bytes remaining = uint32(4) + float(4) — some forks
    //   4 bytes remaining = float(4)             — no unknown field
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining >= 8) {
        packet.readUInt32();  // unknown (extended format)
    } else if (remaining >= 5) {
        packet.readUInt8();   // unknown (standard 3.3.5a)
    }
    // float newSpeed
    float newSpeed = packet.readFloat();

    LOG_INFO("SMSG_FORCE_RUN_SPEED_CHANGE: guid=0x", std::hex, guid, std::dec,
             " counter=", counter, " speed=", newSpeed);

    if (guid != playerGuid) return;

    // Always ACK the speed change to prevent server stall
    if (socket) {
        network::Packet ack(static_cast<uint16_t>(Opcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK));
        ack.writeUInt64(playerGuid);
        ack.writeUInt32(counter);
        // MovementInfo (minimal — no flags set means no optional fields)
        ack.writeUInt32(0);  // moveFlags
        ack.writeUInt16(0);  // moveFlags2
        ack.writeUInt32(movementTime);
        ack.writeFloat(movementInfo.x);
        ack.writeFloat(movementInfo.y);
        ack.writeFloat(movementInfo.z);
        ack.writeFloat(movementInfo.orientation);
        ack.writeUInt32(0);  // fallTime
        ack.writeFloat(newSpeed);
        socket->send(ack);
    }

    // Validate speed - reject garbage/NaN values but still ACK
    if (std::isnan(newSpeed) || newSpeed < 0.1f || newSpeed > 100.0f) {
        LOG_WARNING("Ignoring invalid run speed: ", newSpeed);
        return;
    }

    serverRunSpeed_ = newSpeed;
}

// ============================================================
// Arena / Battleground Handlers
// ============================================================

void GameHandler::handleBattlefieldStatus(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t queueSlot = packet.readUInt32();

    // Minimal packet = just queueSlot + arenaType(1) when status is NONE
    if (packet.getSize() - packet.getReadPos() < 1) {
        LOG_INFO("Battlefield status: queue slot ", queueSlot, " cleared");
        return;
    }

    uint8_t arenaType = packet.readUInt8();
    if (packet.getSize() - packet.getReadPos() < 1) return;

    // Unknown byte
    packet.readUInt8();
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t bgTypeId = packet.readUInt32();

    if (packet.getSize() - packet.getReadPos() < 2) return;
    uint16_t unk2 = packet.readUInt16();
    (void)unk2;

    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t clientInstanceId = packet.readUInt32();
    (void)clientInstanceId;

    if (packet.getSize() - packet.getReadPos() < 1) return;
    uint8_t isRatedArena = packet.readUInt8();
    (void)isRatedArena;

    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t statusId = packet.readUInt32();

    std::string bgName = "Battleground #" + std::to_string(bgTypeId);
    if (arenaType > 0) {
        bgName = std::to_string(arenaType) + "v" + std::to_string(arenaType) + " Arena";
    }

    switch (statusId) {
        case 0: // STATUS_NONE
            LOG_INFO("Battlefield status: NONE for ", bgName);
            break;
        case 1: // STATUS_WAIT_QUEUE
            addSystemChatMessage("Queued for " + bgName + ".");
            LOG_INFO("Battlefield status: WAIT_QUEUE for ", bgName);
            break;
        case 2: // STATUS_WAIT_JOIN
            addSystemChatMessage(bgName + " is ready! Type /join to enter.");
            LOG_INFO("Battlefield status: WAIT_JOIN for ", bgName);
            break;
        case 3: // STATUS_IN_PROGRESS
            addSystemChatMessage("Entered " + bgName + ".");
            LOG_INFO("Battlefield status: IN_PROGRESS for ", bgName);
            break;
        case 4: // STATUS_WAIT_LEAVE
            LOG_INFO("Battlefield status: WAIT_LEAVE for ", bgName);
            break;
        default:
            LOG_INFO("Battlefield status: unknown (", statusId, ") for ", bgName);
            break;
    }
}

void GameHandler::handleArenaTeamCommandResult(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) return;
    uint32_t command = packet.readUInt32();
    std::string name = packet.readString();
    uint32_t error = packet.readUInt32();

    static const char* commands[] = { "create", "invite", "leave", "remove", "disband", "leader" };
    std::string cmdName = (command < 6) ? commands[command] : "unknown";

    if (error == 0) {
        addSystemChatMessage("Arena team " + cmdName + " successful" +
                           (name.empty() ? "." : ": " + name));
    } else {
        addSystemChatMessage("Arena team " + cmdName + " failed" +
                           (name.empty() ? "." : " for " + name + "."));
    }
    LOG_INFO("Arena team command: ", cmdName, " name=", name, " error=", error);
}

void GameHandler::handleArenaTeamQueryResponse(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t teamId = packet.readUInt32();
    std::string teamName = packet.readString();
    LOG_INFO("Arena team query response: id=", teamId, " name=", teamName);
}

void GameHandler::handleArenaTeamInvite(network::Packet& packet) {
    std::string playerName = packet.readString();
    std::string teamName = packet.readString();
    addSystemChatMessage(playerName + " has invited you to join " + teamName + ".");
    LOG_INFO("Arena team invite from ", playerName, " to ", teamName);
}

void GameHandler::handleArenaTeamEvent(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 1) return;
    uint8_t event = packet.readUInt8();

    static const char* events[] = {
        "joined", "left", "removed", "leader changed",
        "disbanded", "created"
    };
    std::string eventName = (event < 6) ? events[event] : "unknown event";

    // Read string params (up to 3)
    uint8_t strCount = 0;
    if (packet.getSize() - packet.getReadPos() >= 1) {
        strCount = packet.readUInt8();
    }

    std::string param1, param2;
    if (strCount >= 1 && packet.getSize() > packet.getReadPos()) param1 = packet.readString();
    if (strCount >= 2 && packet.getSize() > packet.getReadPos()) param2 = packet.readString();

    std::string msg = "Arena team " + eventName;
    if (!param1.empty()) msg += ": " + param1;
    if (!param2.empty()) msg += " (" + param2 + ")";
    addSystemChatMessage(msg);
    LOG_INFO("Arena team event: ", eventName, " ", param1, " ", param2);
}

void GameHandler::handleArenaError(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t error = packet.readUInt32();

    std::string msg;
    switch (error) {
        case 1: msg = "The other team is not big enough."; break;
        case 2: msg = "That team is full."; break;
        case 3: msg = "Not enough members to start."; break;
        case 4: msg = "Too many members."; break;
        default: msg = "Arena error (code " + std::to_string(error) + ")"; break;
    }
    addSystemChatMessage(msg);
    LOG_INFO("Arena error: ", error, " - ", msg);
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

void GameHandler::handleMonsterMoveTransport(network::Packet& packet) {
    // Parse transport-relative creature movement (NPCs on boats/zeppelins)
    // Packet structure: mover GUID + transport GUID + spline data (local coords)

    uint64_t moverGuid = packet.readUInt64();
    uint8_t unk = packet.readUInt8();  // Unknown byte (usually 0)
    uint64_t transportGuid = packet.readUInt64();

    // Transport-local coordinates
    float localX = packet.readFloat();
    float localY = packet.readFloat();
    float localZ = packet.readFloat();

    LOG_INFO("SMSG_MONSTER_MOVE_TRANSPORT: mover=0x", std::hex, moverGuid,
             " transport=0x", transportGuid, std::dec,
             " localPos=(", localX, ", ", localY, ", ", localZ, ")");

    // Compose world position: worldPos = transportTransform * localPos
    auto entity = entityManager.getEntity(moverGuid);
    if (!entity) {
        LOG_WARNING("  NPC 0x", std::hex, moverGuid, std::dec, " not found in entity manager");
        return;
    }

    if (transportManager_) {
        // Use TransportManager to compose world position from local offset
        glm::vec3 localPos(localX, localY, localZ);
        glm::vec3 worldPos = transportManager_->getPlayerWorldPosition(transportGuid, localPos);

        entity->setPosition(worldPos.x, worldPos.y, worldPos.z, entity->getOrientation());

        LOG_INFO("  Composed NPC world position: (", worldPos.x, ", ", worldPos.y, ", ", worldPos.z, ")");
    } else {
        LOG_WARNING("  TransportManager not available for NPC position composition");
    }

    // TODO: Parse full spline data for smooth NPC movement on transport
    // Then update entity position and call creatureMoveCallback_

    // Suppress unused variable warning for now
    (void)unk;
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
        autoTargetAttacker(data.attackerGuid);
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

    if (data.targetGuid == playerGuid && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
        autoTargetAttacker(data.attackerGuid);
    }

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

    // Casting any spell while mounted → dismount instead
    if (isMounted()) {
        dismount();
        return;
    }

    if (casting) return; // Already casting

    // Hearthstone is item-bound; use the item rather than direct spell cast.
    if (spellId == 8690) {
        useItemById(6948);
        return;
    }

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
    saveCharacterConfig();
}

float GameHandler::getSpellCooldown(uint32_t spellId) const {
    auto it = spellCooldowns.find(spellId);
    return (it != spellCooldowns.end()) ? it->second : 0.0f;
}

void GameHandler::handleInitialSpells(network::Packet& packet) {
    InitialSpellsData data;
    if (!InitialSpellsParser::parse(packet, data)) return;

    knownSpells = data.spellIds;

    // Debug: check if specific spells are in initial spells
    bool has527 = std::find(knownSpells.begin(), knownSpells.end(), 527u) != knownSpells.end();
    bool has988 = std::find(knownSpells.begin(), knownSpells.end(), 988u) != knownSpells.end();
    bool has1180 = std::find(knownSpells.begin(), knownSpells.end(), 1180u) != knownSpells.end();
    LOG_INFO("Initial spells include: 527=", has527, " 988=", has988, " 1180=", has1180);

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

    // Load saved action bar or use defaults (Attack slot 1, Hearthstone slot 12)
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone
    loadCharacterConfig();

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
        if (isAll) {
            auraList->clear();
        }
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

    // Check if this spell corresponds to a talent rank
    for (const auto& [talentId, talent] : talentCache_) {
        for (int rank = 0; rank < 5; ++rank) {
            if (talent.rankSpells[rank] == spellId) {
                // Found the talent! Update the rank for the active spec
                uint8_t newRank = rank + 1; // rank is 0-indexed in array, but stored as 1-indexed
                learnedTalents_[activeTalentSpec_][talentId] = newRank;
                LOG_INFO("Talent learned: id=", talentId, " rank=", (int)newRank,
                         " (spell ", spellId, ") in spec ", (int)activeTalentSpec_);
                return;
            }
        }
    }
}

void GameHandler::handleRemovedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.erase(
        std::remove(knownSpells.begin(), knownSpells.end(), spellId),
        knownSpells.end());
    LOG_INFO("Removed spell: ", spellId);
}

void GameHandler::handleSupercededSpell(network::Packet& packet) {
    // Old spell replaced by new rank (e.g., Fireball Rank 1 -> Fireball Rank 2)
    uint32_t oldSpellId = packet.readUInt32();
    uint32_t newSpellId = packet.readUInt32();

    // Remove old spell
    knownSpells.erase(
        std::remove(knownSpells.begin(), knownSpells.end(), oldSpellId),
        knownSpells.end());

    // Add new spell
    knownSpells.push_back(newSpellId);

    LOG_INFO("Spell superceded: ", oldSpellId, " -> ", newSpellId);

    const std::string& newName = getSpellName(newSpellId);
    if (!newName.empty()) {
        addSystemChatMessage("Upgraded to " + newName);
    }
}

void GameHandler::handleUnlearnSpells(network::Packet& packet) {
    // Sent when unlearning multiple spells (e.g., spec change, respec)
    uint32_t spellCount = packet.readUInt32();
    LOG_INFO("Unlearning ", spellCount, " spells");

    for (uint32_t i = 0; i < spellCount && packet.getSize() - packet.getReadPos() >= 4; ++i) {
        uint32_t spellId = packet.readUInt32();
        knownSpells.erase(
            std::remove(knownSpells.begin(), knownSpells.end(), spellId),
            knownSpells.end());
        LOG_INFO("  Unlearned spell: ", spellId);
    }

    if (spellCount > 0) {
        addSystemChatMessage("Unlearned " + std::to_string(spellCount) + " spells");
    }
}

// ============================================================
// Talents
// ============================================================

void GameHandler::handleTalentsInfo(network::Packet& packet) {
    TalentsInfoData data;
    if (!TalentsInfoParser::parse(packet, data)) return;

    // Ensure talent DBCs are loaded
    loadTalentDbc();

    // Validate spec number
    if (data.talentSpec > 1) {
        LOG_WARNING("Invalid talent spec: ", (int)data.talentSpec);
        return;
    }

    // Store talents for this spec
    unspentTalentPoints_[data.talentSpec] = data.unspentPoints;

    // Clear and rebuild learned talents map for this spec
    // Note: If a talent appears in the packet, it's learned (ranks are 0-indexed)
    learnedTalents_[data.talentSpec].clear();
    for (const auto& talent : data.talents) {
        learnedTalents_[data.talentSpec][talent.talentId] = talent.currentRank;
    }

    LOG_INFO("Talents loaded: spec=", (int)data.talentSpec,
             " unspent=", (int)unspentTalentPoints_[data.talentSpec],
             " learned=", learnedTalents_[data.talentSpec].size());

    // If this is the first spec received, set it as active
    static bool firstSpecReceived = false;
    if (!firstSpecReceived) {
        firstSpecReceived = true;
        activeTalentSpec_ = data.talentSpec;

        // Show message to player about active spec
        if (unspentTalentPoints_[data.talentSpec] > 0) {
            std::string msg = "You have " + std::to_string(unspentTalentPoints_[data.talentSpec]) +
                             " unspent talent point";
            if (unspentTalentPoints_[data.talentSpec] > 1) msg += "s";
            msg += " in spec " + std::to_string(data.talentSpec + 1);
            addSystemChatMessage(msg);
        }
    }
}

void GameHandler::learnTalent(uint32_t talentId, uint32_t requestedRank) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("learnTalent: Not in world or no socket connection");
        return;
    }

    LOG_INFO("Requesting to learn talent: id=", talentId, " rank=", requestedRank);

    auto packet = LearnTalentPacket::build(talentId, requestedRank);
    socket->send(packet);
}

void GameHandler::switchTalentSpec(uint8_t newSpec) {
    if (newSpec > 1) {
        LOG_WARNING("Invalid talent spec: ", (int)newSpec);
        return;
    }

    if (newSpec == activeTalentSpec_) {
        LOG_INFO("Already on spec ", (int)newSpec);
        return;
    }

    // For now, just switch locally. In a real implementation, we'd send
    // MSG_TALENT_WIPE_CONFIRM to the server to trigger a spec switch.
    // The server would respond with new SMSG_TALENTS_INFO for the new spec.
    activeTalentSpec_ = newSpec;

    LOG_INFO("Switched to talent spec ", (int)newSpec,
             " (unspent=", (int)unspentTalentPoints_[newSpec],
             ", learned=", learnedTalents_[newSpec].size(), ")");

    std::string msg = "Switched to spec " + std::to_string(newSpec + 1);
    if (unspentTalentPoints_[newSpec] > 0) {
        msg += " (" + std::to_string(unspentTalentPoints_[newSpec]) + " unspent point";
        if (unspentTalentPoints_[newSpec] > 1) msg += "s";
        msg += ")";
    }
    addSystemChatMessage(msg);
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

void GameHandler::interactWithGameObject(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GameObjectUsePacket::build(guid);
    socket->send(packet);
}

void GameHandler::selectGossipOption(uint32_t optionId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, currentGossip.menuId, optionId);
    socket->send(packet);

    // If this is an innkeeper "make this inn your home" option, send binder activate.
    for (const auto& opt : currentGossip.options) {
        if (opt.id != optionId) continue;
        std::string text = opt.text;
        std::transform(text.begin(), text.end(), text.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        if (text.find("make this inn your home") != std::string::npos ||
            text.find("set your home") != std::string::npos) {
            auto bindPkt = BinderActivatePacket::build(currentGossip.npcGuid);
            socket->send(bindPkt);
            LOG_INFO("Sent CMSG_BINDER_ACTIVATE for npc=0x", std::hex, currentGossip.npcGuid, std::dec);
        }
        break;
    }
}

void GameHandler::selectGossipQuest(uint32_t questId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;

    // Check if quest is in our quest log and completable
    bool isInLog = false;
    bool isCompletable = false;
    for (const auto& quest : questLog_) {
        if (quest.questId == questId) {
            isInLog = true;
            isCompletable = quest.complete;
            break;
        }
    }

    LOG_INFO("selectGossipQuest: questId=", questId, " isInLog=", isInLog, " isCompletable=", isCompletable);
    LOG_INFO("  Current quest log size: ", questLog_.size());
    for (const auto& q : questLog_) {
        LOG_INFO("    Quest ", q.questId, ": complete=", q.complete);
    }

    if (isInLog && isCompletable) {
        // Quest is ready to turn in - request reward
        LOG_INFO("Turning in quest: questId=", questId, " npcGuid=", currentGossip.npcGuid);
        network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_REQUEST_REWARD));
        packet.writeUInt64(currentGossip.npcGuid);
        packet.writeUInt32(questId);
        socket->send(packet);
    } else {
        // New quest or not completable - query details
        LOG_INFO("Querying quest details: questId=", questId, " npcGuid=", currentGossip.npcGuid);
        auto packet = QuestgiverQueryQuestPacket::build(currentGossip.npcGuid, questId);
        socket->send(packet);
    }

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
    LOG_INFO("Quest offer reward: questId=", data.questId, " title=\"", data.title, "\"");
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
    LOG_INFO("Completing quest: questId=", currentQuestOfferReward_.questId,
             " npcGuid=", npcGuid, " rewardIndex=", rewardIndex);
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

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
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

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;

    // Query item info so loot window can show names instead of IDs
    for (const auto& item : currentLoot.items) {
        queryItemInfo(item.itemId, 0);
    }

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

    // Query quest data and update quest log based on gossip quests
    for (const auto& questItem : currentGossip.quests) {
        // Update quest log based on questIcon:
        // questIcon & 0x04 = blue ? (turn-in/reward)
        // questIcon & 0x02 = yellow ! (available)
        // questIcon & 0x01 = gray ? (incomplete)
        bool isCompletable = (questItem.questIcon & 0x04) != 0;  // Can turn in
        bool isIncomplete = (questItem.questIcon & 0x01) != 0;   // Have but incomplete
        // Note: questIcon & 0x02 = available (new quest), not added to log yet

        // Add or update quest in log
        bool found = false;
        for (auto& quest : questLog_) {
            if (quest.questId == questItem.questId) {
                quest.complete = isCompletable;
                quest.title = questItem.title;
                found = true;
                LOG_INFO("Updated quest ", questItem.questId, " in log: complete=", isCompletable);
                break;
            }
        }

        if (!found && (isCompletable || isIncomplete)) {
            // Quest is active (either completable or incomplete) - add to log
            QuestLogEntry entry;
            entry.questId = questItem.questId;
            entry.complete = isCompletable;
            entry.title = questItem.title;
            questLog_.push_back(entry);
            LOG_INFO("Added quest ", questItem.questId, " to log: complete=", isCompletable);
        }
    }

    // Play NPC greeting voice
    if (npcGreetingCallback_ && currentGossip.npcGuid != 0) {
        auto entity = entityManager.getEntity(currentGossip.npcGuid);
        if (entity) {
            glm::vec3 npcPos(entity->getX(), entity->getY(), entity->getZ());
            npcGreetingCallback_(currentGossip.npcGuid, npcPos);
        }
    }
}

void GameHandler::handleGossipComplete(network::Packet& packet) {
    (void)packet;

    // Play farewell sound before closing
    if (npcFarewellCallback_ && currentGossip.npcGuid != 0) {
        auto entity = entityManager.getEntity(currentGossip.npcGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            npcFarewellCallback_(currentGossip.npcGuid, pos);
        }
    }

    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::handleListInventory(network::Packet& packet) {
    if (!ListInventoryParser::parse(packet, currentVendorItems)) return;
    vendorWindowOpen = true;
    gossipWindowOpen = false; // Close gossip if vendor opens

    // Play vendor sound
    if (npcVendorCallback_ && currentVendorItems.vendorGuid != 0) {
        auto entity = entityManager.getEntity(currentVendorItems.vendorGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            npcVendorCallback_(currentVendorItems.vendorGuid, pos);
        }
    }

    // Query item info for all vendor items so we can show names
    for (const auto& item : currentVendorItems.items) {
        queryItemInfo(item.itemId, 0);
    }
}

// ============================================================
// Trainer
// ============================================================

void GameHandler::handleTrainerList(network::Packet& packet) {
    if (!TrainerListParser::parse(packet, currentTrainerList_)) return;
    trainerWindowOpen_ = true;
    gossipWindowOpen = false;

    // Debug: log known spells
    LOG_INFO("Known spells count: ", knownSpells.size());
    if (knownSpells.size() <= 50) {
        std::string spellList;
        for (uint32_t id : knownSpells) {
            if (!spellList.empty()) spellList += ", ";
            spellList += std::to_string(id);
        }
        LOG_INFO("Known spells: ", spellList);
    }

    // Check if specific prerequisite spells are known
    bool has527 = std::find(knownSpells.begin(), knownSpells.end(), 527u) != knownSpells.end();
    bool has25312 = std::find(knownSpells.begin(), knownSpells.end(), 25312u) != knownSpells.end();
    LOG_INFO("Prerequisite check: 527=", has527, " 25312=", has25312);

    // Debug: log first few trainer spells to see their state
    LOG_INFO("Trainer spells received: ", currentTrainerList_.spells.size(), " spells");
    for (size_t i = 0; i < std::min(size_t(5), currentTrainerList_.spells.size()); ++i) {
        const auto& s = currentTrainerList_.spells[i];
        LOG_INFO("  Spell[", i, "]: id=", s.spellId, " state=", (int)s.state,
                 " cost=", s.spellCost, " reqLvl=", (int)s.reqLevel,
                 " chain=(", s.chainNode1, ",", s.chainNode2, ",", s.chainNode3, ")");
    }


    // Ensure caches are populated
    loadSpellNameCache();
    loadSkillLineDbc();
    loadSkillLineAbilityDbc();
    categorizeTrainerSpells();
}

void GameHandler::trainSpell(uint32_t spellId) {
    LOG_INFO("trainSpell called: spellId=", spellId, " state=", (int)state, " socket=", (socket ? "yes" : "no"));
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("trainSpell: Not in world or no socket connection");
        return;
    }

    // Find spell cost in trainer list
    uint32_t spellCost = 0;
    for (const auto& spell : currentTrainerList_.spells) {
        if (spell.spellId == spellId) {
            spellCost = spell.spellCost;
            break;
        }
    }
    LOG_INFO("Player money: ", playerMoneyCopper_, " copper, spell cost: ", spellCost, " copper");

    LOG_INFO("Sending CMSG_TRAINER_BUY_SPELL: guid=", currentTrainerList_.trainerGuid,
             " spellId=", spellId);
    auto packet = TrainerBuySpellPacket::build(
        currentTrainerList_.trainerGuid,
        spellId);
    socket->send(packet);
    LOG_INFO("CMSG_TRAINER_BUY_SPELL sent");
}

void GameHandler::closeTrainer() {
    trainerWindowOpen_ = false;
    currentTrainerList_ = TrainerListData{};
    trainerTabs_.clear();
}

void GameHandler::loadSpellNameCache() {
    if (spellNameCacheLoaded_) return;
    spellNameCacheLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Trainer: Could not load Spell.dbc for spell names");
        return;
    }

    if (dbc->getFieldCount() < 154) {
        LOG_WARNING("Trainer: Spell.dbc has too few fields");
        return;
    }

    // Fields: 0=SpellID, 136=SpellName_enUS, 153=RankText_enUS
    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t id = dbc->getUInt32(i, 0);
        if (id == 0) continue;
        std::string name = dbc->getString(i, 136);
        std::string rank = dbc->getString(i, 153);
        if (!name.empty()) {
            spellNameCache_[id] = {std::move(name), std::move(rank)};
        }
    }
    LOG_INFO("Trainer: Loaded ", spellNameCache_.size(), " spell names from Spell.dbc");
}

void GameHandler::loadSkillLineAbilityDbc() {
    if (skillLineAbilityLoaded_) return;
    skillLineAbilityLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    // SkillLineAbility.dbc: field 1=skillLineID, field 2=spellID
    auto slaDbc = am->loadDBC("SkillLineAbility.dbc");
    if (slaDbc && slaDbc->isLoaded()) {
        for (uint32_t i = 0; i < slaDbc->getRecordCount(); i++) {
            uint32_t skillLineId = slaDbc->getUInt32(i, 1);
            uint32_t spellId = slaDbc->getUInt32(i, 2);
            if (spellId > 0 && skillLineId > 0) {
                spellToSkillLine_[spellId] = skillLineId;
            }
        }
        LOG_INFO("Trainer: Loaded ", spellToSkillLine_.size(), " skill line abilities");
    }
}

void GameHandler::categorizeTrainerSpells() {
    trainerTabs_.clear();

    static constexpr uint32_t SKILLLINE_CATEGORY_CLASS = 7;

    // Group spells by skill line (category 7 = class spec tabs)
    std::map<uint32_t, std::vector<const TrainerSpell*>> specialtySpells;
    std::vector<const TrainerSpell*> generalSpells;

    for (const auto& spell : currentTrainerList_.spells) {
        auto slIt = spellToSkillLine_.find(spell.spellId);
        if (slIt != spellToSkillLine_.end()) {
            uint32_t skillLineId = slIt->second;
            auto catIt = skillLineCategories_.find(skillLineId);
            if (catIt != skillLineCategories_.end() && catIt->second == SKILLLINE_CATEGORY_CLASS) {
                specialtySpells[skillLineId].push_back(&spell);
                continue;
            }
        }
        generalSpells.push_back(&spell);
    }

    // Sort by spell name within each group
    auto byName = [this](const TrainerSpell* a, const TrainerSpell* b) {
        return getSpellName(a->spellId) < getSpellName(b->spellId);
    };

    // Build named tabs sorted alphabetically
    std::vector<std::pair<std::string, std::vector<const TrainerSpell*>>> named;
    for (auto& [skillLineId, spells] : specialtySpells) {
        auto nameIt = skillLineNames_.find(skillLineId);
        std::string tabName = (nameIt != skillLineNames_.end()) ? nameIt->second : "Specialty";
        std::sort(spells.begin(), spells.end(), byName);
        named.push_back({std::move(tabName), std::move(spells)});
    }
    std::sort(named.begin(), named.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (auto& [name, spells] : named) {
        trainerTabs_.push_back({std::move(name), std::move(spells)});
    }

    // General tab last
    if (!generalSpells.empty()) {
        std::sort(generalSpells.begin(), generalSpells.end(), byName);
        trainerTabs_.push_back({"General", std::move(generalSpells)});
    }

    LOG_INFO("Trainer: Categorized into ", trainerTabs_.size(), " tabs");
}

void GameHandler::loadTalentDbc() {
    if (talentDbcLoaded_) return;
    talentDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    // Load Talent.dbc
    auto talentDbc = am->loadDBC("Talent.dbc");
    if (talentDbc && talentDbc->isLoaded()) {
        // Talent.dbc structure (WoW 3.3.5a):
        // 0: TalentID
        // 1: TalentTabID
        // 2: Row (tier)
        // 3: Column
        // 4-8: RankID[0-4] (spell IDs for ranks 1-5)
        // 9-11: PrereqTalent[0-2]
        // 12-14: PrereqRank[0-2]
        // (other fields less relevant for basic functionality)

        uint32_t count = talentDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentEntry entry;
            entry.talentId = talentDbc->getUInt32(i, 0);
            if (entry.talentId == 0) continue;

            entry.tabId = talentDbc->getUInt32(i, 1);
            entry.row = static_cast<uint8_t>(talentDbc->getUInt32(i, 2));
            entry.column = static_cast<uint8_t>(talentDbc->getUInt32(i, 3));

            // Rank spells (1-5 ranks)
            for (int r = 0; r < 5; ++r) {
                entry.rankSpells[r] = talentDbc->getUInt32(i, 4 + r);
            }

            // Prerequisites
            for (int p = 0; p < 3; ++p) {
                entry.prereqTalent[p] = talentDbc->getUInt32(i, 9 + p);
                entry.prereqRank[p] = static_cast<uint8_t>(talentDbc->getUInt32(i, 12 + p));
            }

            // Calculate max rank
            entry.maxRank = 0;
            for (int r = 0; r < 5; ++r) {
                if (entry.rankSpells[r] != 0) {
                    entry.maxRank = r + 1;
                }
            }

            talentCache_[entry.talentId] = entry;
        }
        LOG_INFO("Loaded ", talentCache_.size(), " talents from Talent.dbc");
    } else {
        LOG_WARNING("Could not load Talent.dbc");
    }

    // Load TalentTab.dbc
    auto tabDbc = am->loadDBC("TalentTab.dbc");
    if (tabDbc && tabDbc->isLoaded()) {
        // TalentTab.dbc structure (WoW 3.3.5a):
        // 0: TalentTabID
        // 1-17: Name (16 localized strings + flags = 17 fields)
        // 18: SpellIconID
        // 19: RaceMask
        // 20: ClassMask
        // 21: PetTalentMask
        // 22: OrderIndex
        // 23-39: BackgroundFile (16 localized strings + flags = 17 fields)

        uint32_t count = tabDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentTabEntry entry;
            entry.tabId = tabDbc->getUInt32(i, 0);
            if (entry.tabId == 0) continue;

            entry.name = tabDbc->getString(i, 1);
            entry.classMask = tabDbc->getUInt32(i, 20);
            entry.orderIndex = static_cast<uint8_t>(tabDbc->getUInt32(i, 22));
            entry.backgroundFile = tabDbc->getString(i, 23);

            talentTabCache_[entry.tabId] = entry;

            // Log first few tabs to debug class mask issue
            if (talentTabCache_.size() <= 10) {
                LOG_INFO("  Tab ", entry.tabId, ": ", entry.name, " (classMask=0x", std::hex, entry.classMask, std::dec, ")");
            }
        }
        LOG_INFO("Loaded ", talentTabCache_.size(), " talent tabs from TalentTab.dbc");
    } else {
        LOG_WARNING("Could not load TalentTab.dbc");
    }
}

static const std::string EMPTY_STRING;

const std::string& GameHandler::getSpellName(uint32_t spellId) const {
    auto it = spellNameCache_.find(spellId);
    return (it != spellNameCache_.end()) ? it->second.name : EMPTY_STRING;
}

const std::string& GameHandler::getSpellRank(uint32_t spellId) const {
    auto it = spellNameCache_.find(spellId);
    return (it != spellNameCache_.end()) ? it->second.rank : EMPTY_STRING;
}

const std::string& GameHandler::getSkillLineName(uint32_t spellId) const {
    auto slIt = spellToSkillLine_.find(spellId);
    if (slIt == spellToSkillLine_.end()) return EMPTY_STRING;
    auto nameIt = skillLineNames_.find(slIt->second);
    return (nameIt != skillLineNames_.end()) ? nameIt->second : EMPTY_STRING;
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
// Teleport Handler
// ============================================================

void GameHandler::handleTeleportAck(network::Packet& packet) {
    // MSG_MOVE_TELEPORT_ACK (server→client): packedGuid + u32 counter + u32 time
    // followed by movement info with the new position
    if (packet.getSize() - packet.getReadPos() < 4) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK too short");
        return;
    }

    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t counter = packet.readUInt32();

    // Read the movement info embedded in the teleport
    // Format: u32 flags, u16 flags2, u32 time, float x, float y, float z, float o
    if (packet.getSize() - packet.getReadPos() < 4 + 2 + 4 + 4 * 4) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK: not enough data for movement info");
        return;
    }

    packet.readUInt32();  // moveFlags
    packet.readUInt16();  // moveFlags2
    uint32_t moveTime = packet.readUInt32();
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();

    LOG_INFO("MSG_MOVE_TELEPORT_ACK: guid=0x", std::hex, guid, std::dec,
             " counter=", counter,
             " pos=(", serverX, ", ", serverY, ", ", serverZ, ")");

    // Update our position
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = orientation;
    movementInfo.flags = 0;

    // Send the ack back to the server
    // Client→server MSG_MOVE_TELEPORT_ACK: u64 guid + u32 counter + u32 time
    if (socket) {
        network::Packet ack(static_cast<uint16_t>(Opcode::MSG_MOVE_TELEPORT_ACK));
        // Write packed guid
        uint8_t mask = 0;
        uint8_t bytes[8];
        int byteCount = 0;
        uint64_t g = playerGuid;
        for (int i = 0; i < 8; i++) {
            uint8_t b = static_cast<uint8_t>(g & 0xFF);
            g >>= 8;
            if (b != 0) {
                mask |= (1 << i);
                bytes[byteCount++] = b;
            }
        }
        ack.writeUInt8(mask);
        for (int i = 0; i < byteCount; i++) {
            ack.writeUInt8(bytes[i]);
        }
        ack.writeUInt32(counter);
        ack.writeUInt32(moveTime);
        socket->send(ack);
        LOG_INFO("Sent MSG_MOVE_TELEPORT_ACK response");
    }

    // Notify application to reload terrain at new position
    if (worldEntryCallback_) {
        worldEntryCallback_(currentMapId_, serverX, serverY, serverZ);
    }
}

void GameHandler::handleNewWorld(network::Packet& packet) {
    // SMSG_NEW_WORLD: uint32 mapId, float x, y, z, orientation
    if (packet.getSize() - packet.getReadPos() < 20) {
        LOG_WARNING("SMSG_NEW_WORLD too short");
        return;
    }

    uint32_t mapId = packet.readUInt32();
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();

    LOG_INFO("SMSG_NEW_WORLD: mapId=", mapId,
             " pos=(", serverX, ", ", serverY, ", ", serverZ, ")",
             " orient=", orientation);

    currentMapId_ = mapId;

    // Update player position
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = orientation;
    movementInfo.flags = 0;

    // Clear world state for the new map
    entityManager.clear();
    hostileAttackers_.clear();
    stopAutoAttack();
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Send MSG_MOVE_WORLDPORT_ACK to tell the server we're ready
    if (socket) {
        network::Packet ack(static_cast<uint16_t>(Opcode::MSG_MOVE_WORLDPORT_ACK));
        socket->send(ack);
        LOG_INFO("Sent MSG_MOVE_WORLDPORT_ACK");
    }

    // Reload terrain at new position
    if (worldEntryCallback_) {
        worldEntryCallback_(mapId, serverX, serverY, serverZ);
    }
}

// ============================================================
// Taxi / Flight Path Handlers
// ============================================================

void GameHandler::loadTaxiDbc() {
    if (taxiDbcLoaded_) return;
    taxiDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    // Load TaxiNodes.dbc: 0=ID, 1=mapId, 2=x, 3=y, 4=z, 5=name(enUS locale)
    auto nodesDbc = am->loadDBC("TaxiNodes.dbc");
    if (nodesDbc && nodesDbc->isLoaded()) {
        uint32_t fieldCount = nodesDbc->getFieldCount();
        for (uint32_t i = 0; i < nodesDbc->getRecordCount(); i++) {
            TaxiNode node;
            node.id = nodesDbc->getUInt32(i, 0);
            node.mapId = nodesDbc->getUInt32(i, 1);
            node.x = nodesDbc->getFloat(i, 2);
            node.y = nodesDbc->getFloat(i, 3);
            node.z = nodesDbc->getFloat(i, 4);
            node.name = nodesDbc->getString(i, 5);
            // TaxiNodes.dbc (3.3.5a): last two fields are mount display IDs (Alliance, Horde)
            if (fieldCount >= 24) {
                node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, 22);
                node.mountDisplayIdHorde = nodesDbc->getUInt32(i, 23);
                if (node.mountDisplayIdAlliance == 0 && node.mountDisplayIdHorde == 0 && fieldCount >= 22) {
                    node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, 20);
                    node.mountDisplayIdHorde = nodesDbc->getUInt32(i, 21);
                }
            }
            if (node.id > 0) {
                taxiNodes_[node.id] = std::move(node);
            }
            if (node.id == 195) {
                std::string fields;
                for (uint32_t f = 0; f < fieldCount; f++) {
                    fields += std::to_string(f) + ":" + std::to_string(nodesDbc->getUInt32(i, f)) + " ";
                }
                LOG_INFO("TaxiNodes[195] fields: ", fields);
            }
        }
        LOG_INFO("Loaded ", taxiNodes_.size(), " taxi nodes from TaxiNodes.dbc");
    } else {
        LOG_WARNING("Could not load TaxiNodes.dbc");
    }

    // Load TaxiPath.dbc: 0=pathId, 1=fromNode, 2=toNode, 3=cost
    auto pathDbc = am->loadDBC("TaxiPath.dbc");
    if (pathDbc && pathDbc->isLoaded()) {
        for (uint32_t i = 0; i < pathDbc->getRecordCount(); i++) {
            TaxiPathEdge edge;
            edge.pathId = pathDbc->getUInt32(i, 0);
            edge.fromNode = pathDbc->getUInt32(i, 1);
            edge.toNode = pathDbc->getUInt32(i, 2);
            edge.cost = pathDbc->getUInt32(i, 3);
            taxiPathEdges_.push_back(edge);
        }
        LOG_INFO("Loaded ", taxiPathEdges_.size(), " taxi path edges from TaxiPath.dbc");
    } else {
        LOG_WARNING("Could not load TaxiPath.dbc");
    }

    // Load TaxiPathNode.dbc: actual spline waypoints for each path
    // 0=ID, 1=PathID, 2=NodeIndex, 3=MapID, 4=X, 5=Y, 6=Z
    auto pathNodeDbc = am->loadDBC("TaxiPathNode.dbc");
    if (pathNodeDbc && pathNodeDbc->isLoaded()) {
        for (uint32_t i = 0; i < pathNodeDbc->getRecordCount(); i++) {
            TaxiPathNode node;
            node.id = pathNodeDbc->getUInt32(i, 0);
            node.pathId = pathNodeDbc->getUInt32(i, 1);
            node.nodeIndex = pathNodeDbc->getUInt32(i, 2);
            node.mapId = pathNodeDbc->getUInt32(i, 3);
            node.x = pathNodeDbc->getFloat(i, 4);
            node.y = pathNodeDbc->getFloat(i, 5);
            node.z = pathNodeDbc->getFloat(i, 6);
            taxiPathNodes_[node.pathId].push_back(node);
        }
        // Sort waypoints by nodeIndex for each path
        for (auto& [pathId, nodes] : taxiPathNodes_) {
            std::sort(nodes.begin(), nodes.end(),
                [](const TaxiPathNode& a, const TaxiPathNode& b) {
                    return a.nodeIndex < b.nodeIndex;
                });
        }
        LOG_INFO("Loaded ", pathNodeDbc->getRecordCount(), " taxi path waypoints from TaxiPathNode.dbc");
    } else {
        LOG_WARNING("Could not load TaxiPathNode.dbc");
    }
}

void GameHandler::handleShowTaxiNodes(network::Packet& packet) {
    ShowTaxiNodesData data;
    if (!ShowTaxiNodesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_SHOWTAXINODES");
        return;
    }

    loadTaxiDbc();

    // Detect newly discovered flight paths by comparing with stored mask
    if (taxiMaskInitialized_) {
        for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
            uint32_t newBits = data.nodeMask[i] & ~knownTaxiMask_[i];
            if (newBits == 0) continue;
            for (uint32_t bit = 0; bit < 32; ++bit) {
                if (newBits & (1u << bit)) {
                    uint32_t nodeId = i * 32 + bit + 1;
                    auto it = taxiNodes_.find(nodeId);
                    if (it != taxiNodes_.end()) {
                        addSystemChatMessage("Discovered flight path: " + it->second.name);
                    }
                }
            }
        }
    }

    // Update stored mask
    for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
        knownTaxiMask_[i] = data.nodeMask[i];
    }
    taxiMaskInitialized_ = true;

    currentTaxiData_ = data;
    taxiNpcGuid_ = data.npcGuid;
    taxiWindowOpen_ = true;
    gossipWindowOpen = false;
    buildTaxiCostMap();
    auto it = taxiNodes_.find(data.nearestNode);
    if (it != taxiNodes_.end()) {
        LOG_INFO("Taxi node ", data.nearestNode, " mounts: A=", it->second.mountDisplayIdAlliance,
                 " H=", it->second.mountDisplayIdHorde);
    }
    LOG_INFO("Taxi window opened, nearest node=", data.nearestNode);
}

void GameHandler::applyTaxiMountForCurrentNode() {
    if (taxiMountActive_ || !mountCallback_) return;
    auto it = taxiNodes_.find(currentTaxiData_.nearestNode);
    if (it == taxiNodes_.end()) return;

    bool isAlliance = true;
    switch (playerRace_) {
        case Race::ORC:
        case Race::UNDEAD:
        case Race::TAUREN:
        case Race::TROLL:
        case Race::GOBLIN:
        case Race::BLOOD_ELF:
            isAlliance = false;
            break;
        default:
            isAlliance = true;
            break;
    }
    uint32_t mountId = isAlliance ? it->second.mountDisplayIdAlliance
                                  : it->second.mountDisplayIdHorde;
    if (mountId == 0) {
        mountId = isAlliance ? it->second.mountDisplayIdHorde
                             : it->second.mountDisplayIdAlliance;
    }
    if (mountId == 0) {
        auto& app = core::Application::getInstance();
        uint32_t gryphonId = app.getGryphonDisplayId();
        uint32_t wyvernId = app.getWyvernDisplayId();
        if (isAlliance && gryphonId != 0) mountId = gryphonId;
        if (!isAlliance && wyvernId != 0) mountId = wyvernId;
        if (mountId == 0) {
            mountId = (isAlliance ? wyvernId : gryphonId);
        }
    }
    if (mountId == 0) {
        // Fallback: any non-zero mount display from the node.
        if (it->second.mountDisplayIdAlliance != 0) mountId = it->second.mountDisplayIdAlliance;
        else if (it->second.mountDisplayIdHorde != 0) mountId = it->second.mountDisplayIdHorde;
    }
    if (mountId == 0 || mountId == 541) {
        mountId = isAlliance ? 30412u : 30413u;
    }
    if (mountId != 0) {
        taxiMountDisplayId_ = mountId;
        taxiMountActive_ = true;
        LOG_INFO("Taxi mount apply: displayId=", mountId);
        mountCallback_(mountId);
    }
}

void GameHandler::startClientTaxiPath(const std::vector<uint32_t>& pathNodes) {
    taxiClientPath_.clear();
    taxiClientIndex_ = 0;
    taxiClientActive_ = false;
    taxiClientSegmentProgress_ = 0.0f;

    // Build full spline path using TaxiPathNode waypoints (not just node positions)
    for (size_t i = 0; i + 1 < pathNodes.size(); i++) {
        uint32_t fromNode = pathNodes[i];
        uint32_t toNode = pathNodes[i + 1];
        // Find the pathId connecting these nodes
        uint32_t pathId = 0;
        for (const auto& edge : taxiPathEdges_) {
            if (edge.fromNode == fromNode && edge.toNode == toNode) {
                pathId = edge.pathId;
                break;
            }
        }
        if (pathId == 0) {
            LOG_WARNING("No taxi path found from node ", fromNode, " to ", toNode);
            continue;
        }
        // Get spline waypoints for this path segment
        auto pathIt = taxiPathNodes_.find(pathId);
        if (pathIt != taxiPathNodes_.end()) {
            for (const auto& wpNode : pathIt->second) {
                glm::vec3 serverPos(wpNode.x, wpNode.y, wpNode.z);
                glm::vec3 canonical = core::coords::serverToCanonical(serverPos);
                taxiClientPath_.push_back(canonical);
            }
        } else {
            LOG_WARNING("No spline waypoints found for taxi pathId ", pathId);
        }
    }

    if (taxiClientPath_.size() < 2) {
        LOG_WARNING("Taxi path too short: ", taxiClientPath_.size(), " waypoints");
        return;
    }

    // Set initial orientation to face the first flight segment
    if (!entityManager.hasEntity(playerGuid)) return;
    auto playerEntity = entityManager.getEntity(playerGuid);
    if (playerEntity) {
        glm::vec3 start = taxiClientPath_[0];
        glm::vec3 end = taxiClientPath_[1];
        glm::vec3 dir = end - start;
        float initialOrientation = std::atan2(dir.y, dir.x) - 1.57079632679f;

        // Calculate initial pitch from altitude change
        glm::vec3 dirNorm = glm::normalize(dir);
        float initialPitch = std::asin(std::clamp(dirNorm.z, -1.0f, 1.0f));
        float initialRoll = 0.0f;  // No initial banking

        playerEntity->setPosition(start.x, start.y, start.z, initialOrientation);
        movementInfo.orientation = initialOrientation;

        // Update mount rotation immediately with pitch and roll
        if (taxiOrientationCallback_) {
            taxiOrientationCallback_(initialOrientation, initialPitch, initialRoll);
        }
    }

    LOG_INFO("Taxi flight started with ", taxiClientPath_.size(), " spline waypoints");
    taxiClientActive_ = true;
}

void GameHandler::updateClientTaxi(float deltaTime) {
    if (!taxiClientActive_ || taxiClientPath_.size() < 2) return;
    if (!entityManager.hasEntity(playerGuid)) return;
    auto playerEntity = entityManager.getEntity(playerGuid);
    if (!playerEntity) return;

    if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
        taxiClientActive_ = false;
        return;
    }

    glm::vec3 start = taxiClientPath_[taxiClientIndex_];
    glm::vec3 end = taxiClientPath_[taxiClientIndex_ + 1];
    glm::vec3 dir = end - start;
    float segmentLen = glm::length(dir);
    if (segmentLen < 0.01f) {
        taxiClientIndex_++;
        taxiClientSegmentProgress_ = 0.0f;
        return;
    }

    taxiClientSegmentProgress_ += taxiClientSpeed_ * deltaTime;
    float t = taxiClientSegmentProgress_ / segmentLen;
    if (t >= 1.0f) {
        taxiClientIndex_++;
        taxiClientSegmentProgress_ = 0.0f;
        if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
            taxiClientActive_ = false;
            onTaxiFlight_ = false;
            taxiLandingCooldown_ = 2.0f;  // 2 second cooldown to prevent re-entering
            if (taxiMountActive_ && mountCallback_) {
                mountCallback_(0);
            }
            taxiMountActive_ = false;
            taxiMountDisplayId_ = 0;
            currentMountDisplayId_ = 0;
            taxiClientPath_.clear();
            taxiRecoverPending_ = false;
            movementInfo.flags = 0;
            movementInfo.flags2 = 0;
            if (socket) {
                sendMovement(Opcode::CMSG_MOVE_STOP);
                sendMovement(Opcode::CMSG_MOVE_HEARTBEAT);
            }
            LOG_INFO("Taxi flight landed (client path)");
        }
        return;
    }

    // Use Catmull-Rom spline for smooth interpolation between waypoints
    // Get surrounding points for spline curve
    glm::vec3 p0 = (taxiClientIndex_ > 0) ? taxiClientPath_[taxiClientIndex_ - 1] : start;
    glm::vec3 p1 = start;
    glm::vec3 p2 = end;
    glm::vec3 p3 = (taxiClientIndex_ + 2 < taxiClientPath_.size()) ?
                   taxiClientPath_[taxiClientIndex_ + 2] : end;

    // Catmull-Rom spline formula for smooth curves
    float t2 = t * t;
    float t3 = t2 * t;
    glm::vec3 nextPos = 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
    );

    // Calculate smooth direction for orientation (tangent to spline)
    glm::vec3 tangent = 0.5f * (
        (-p0 + p2) +
        2.0f * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t +
        3.0f * (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t2
    );

    // Calculate yaw from horizontal direction
    float targetOrientation = std::atan2(tangent.y, tangent.x) - 1.57079632679f;

    // Calculate pitch from vertical component (altitude change)
    glm::vec3 tangentNorm = glm::normalize(tangent);
    float pitch = std::asin(std::clamp(tangentNorm.z, -1.0f, 1.0f));

    // Calculate roll (banking) from rate of yaw change
    float currentOrientation = movementInfo.orientation;
    float orientDiff = targetOrientation - currentOrientation;
    // Normalize angle difference to [-PI, PI]
    while (orientDiff > 3.14159265f) orientDiff -= 6.28318530f;
    while (orientDiff < -3.14159265f) orientDiff += 6.28318530f;
    // Bank proportional to turn rate (scaled for visual effect)
    float roll = -orientDiff * 2.5f;
    roll = std::clamp(roll, -0.7f, 0.7f);  // Limit to ~40 degrees

    // Smooth rotation transition (lerp towards target)
    float smoothOrientation = currentOrientation + orientDiff * std::min(1.0f, deltaTime * 3.0f);

    playerEntity->setPosition(nextPos.x, nextPos.y, nextPos.z, smoothOrientation);
    movementInfo.x = nextPos.x;
    movementInfo.y = nextPos.y;
    movementInfo.z = nextPos.z;
    movementInfo.orientation = smoothOrientation;

    // Update mount rotation with yaw, pitch, and roll for realistic flight
    if (taxiOrientationCallback_) {
        taxiOrientationCallback_(smoothOrientation, pitch, roll);
    }
}

void GameHandler::handleActivateTaxiReply(network::Packet& packet) {
    ActivateTaxiReplyData data;
    if (!ActivateTaxiReplyParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACTIVATETAXIREPLY");
        return;
    }

    if (data.result == 0) {
        onTaxiFlight_ = true;
        taxiWindowOpen_ = false;
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        applyTaxiMountForCurrentNode();
        LOG_INFO("Taxi flight started!");
    } else {
        LOG_WARNING("Taxi activation failed, result=", data.result);
        addSystemChatMessage("Cannot take that flight path.");
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        if (taxiMountActive_ && mountCallback_) {
            mountCallback_(0);
        }
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        onTaxiFlight_ = false;
    }
}

void GameHandler::closeTaxi() {
    taxiWindowOpen_ = false;

    // If we optimistically mounted during node selection, dismount now
    if (taxiMountActive_ && mountCallback_) {
        mountCallback_(0);  // Dismount
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;

    // Clear any pending activation
    taxiActivatePending_ = false;
    onTaxiFlight_ = false;

    // Set cooldown to prevent auto-mount trigger from re-applying taxi mount
    // (The UNIT_FLAG_TAXI_FLIGHT check in handleUpdateObject won't re-trigger during cooldown)
    taxiLandingCooldown_ = 2.0f;
}

void GameHandler::buildTaxiCostMap() {
    taxiCostMap_.clear();
    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0) return;

    // Build adjacency list with costs from all edges (path may traverse unknown nodes)
    struct AdjEntry { uint32_t node; uint32_t cost; };
    std::unordered_map<uint32_t, std::vector<AdjEntry>> adj;
    for (const auto& edge : taxiPathEdges_) {
        adj[edge.fromNode].push_back({edge.toNode, edge.cost});
    }

    // BFS from startNode, accumulating costs along the path
    std::deque<uint32_t> queue;
    queue.push_back(startNode);
    taxiCostMap_[startNode] = 0;

    while (!queue.empty()) {
        uint32_t cur = queue.front();
        queue.pop_front();
        for (const auto& next : adj[cur]) {
            if (taxiCostMap_.find(next.node) == taxiCostMap_.end()) {
                taxiCostMap_[next.node] = taxiCostMap_[cur] + next.cost;
                queue.push_back(next.node);
            }
        }
    }
}

uint32_t GameHandler::getTaxiCostTo(uint32_t destNodeId) const {
    auto it = taxiCostMap_.find(destNodeId);
    return (it != taxiCostMap_.end()) ? it->second : 0;
}

void GameHandler::activateTaxi(uint32_t destNodeId) {
    if (!socket || state != WorldState::IN_WORLD) return;

    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0 || destNodeId == 0 || startNode == destNodeId) return;

    // If already mounted, dismount before starting a taxi flight.
    if (isMounted()) {
        LOG_INFO("Taxi activate: dismounting current mount");
        if (mountCallback_) mountCallback_(0);
        currentMountDisplayId_ = 0;
        dismount();
    }

    addSystemChatMessage("Taxi: requesting flight...");

    // BFS to find path from startNode to destNodeId
    std::unordered_map<uint32_t, std::vector<uint32_t>> adj;
    for (const auto& edge : taxiPathEdges_) {
        adj[edge.fromNode].push_back(edge.toNode);
    }

    std::unordered_map<uint32_t, uint32_t> parent;
    std::deque<uint32_t> queue;
    queue.push_back(startNode);
    parent[startNode] = startNode;

    bool found = false;
    while (!queue.empty()) {
        uint32_t cur = queue.front();
        queue.pop_front();
        if (cur == destNodeId) { found = true; break; }
        for (uint32_t next : adj[cur]) {
            if (parent.find(next) == parent.end()) {
                parent[next] = cur;
                queue.push_back(next);
            }
        }
    }

    if (!found) {
        LOG_WARNING("No taxi path found from node ", startNode, " to ", destNodeId);
        addSystemChatMessage("No flight path available to that destination.");
        return;
    }

    std::vector<uint32_t> path;
    for (uint32_t n = destNodeId; n != startNode; n = parent[n]) {
        path.push_back(n);
    }
    path.push_back(startNode);
    std::reverse(path.begin(), path.end());

    LOG_INFO("Taxi path: ", path.size(), " nodes, from ", startNode, " to ", destNodeId);

    LOG_INFO("Taxi activate: npc=0x", std::hex, taxiNpcGuid_, std::dec,
             " start=", startNode, " dest=", destNodeId, " pathLen=", path.size());
    if (!path.empty()) {
        std::string pathStr;
        for (size_t i = 0; i < path.size(); i++) {
            pathStr += std::to_string(path[i]);
            if (i + 1 < path.size()) pathStr += "->";
        }
        LOG_INFO("Taxi path nodes: ", pathStr);
    }

    uint32_t totalCost = getTaxiCostTo(destNodeId);
    LOG_INFO("Taxi activate: start=", startNode, " dest=", destNodeId, " cost=", totalCost);

    // Some servers only accept basic CMSG_ACTIVATETAXI.
    auto basicPkt = ActivateTaxiPacket::build(taxiNpcGuid_, startNode, destNodeId);
    socket->send(basicPkt);

    // Others accept express with a full node path + cost.
    auto pkt = ActivateTaxiExpressPacket::build(taxiNpcGuid_, totalCost, path);
    socket->send(pkt);

    // Optimistically start taxi visuals; server will correct if it denies.
    taxiActivatePending_ = true;
    taxiActivateTimer_ = 0.0f;
    if (!onTaxiFlight_) {
        onTaxiFlight_ = true;
        applyTaxiMountForCurrentNode();
    }

    // Start mounting delay (gives terrain precache time to load)
    taxiMountingDelay_ = true;
    taxiMountingTimer_ = 0.0f;
    taxiPendingPath_ = path;

    // Trigger terrain precache immediately (uses mounting delay time to load)
    if (taxiPrecacheCallback_) {
        std::vector<glm::vec3> previewPath;
        // Build full spline path using TaxiPathNode waypoints
        for (size_t i = 0; i + 1 < path.size(); i++) {
            uint32_t fromNode = path[i];
            uint32_t toNode = path[i + 1];
            // Find the pathId connecting these nodes
            uint32_t pathId = 0;
            for (const auto& edge : taxiPathEdges_) {
                if (edge.fromNode == fromNode && edge.toNode == toNode) {
                    pathId = edge.pathId;
                    break;
                }
            }
            if (pathId == 0) continue;
            // Get spline waypoints for this path segment
            auto pathIt = taxiPathNodes_.find(pathId);
            if (pathIt != taxiPathNodes_.end()) {
                for (const auto& wpNode : pathIt->second) {
                    glm::vec3 serverPos(wpNode.x, wpNode.y, wpNode.z);
                    glm::vec3 canonical = core::coords::serverToCanonical(serverPos);
                    previewPath.push_back(canonical);
                }
            }
        }
        if (previewPath.size() >= 2) {
            taxiPrecacheCallback_(previewPath);
        }
    }

    addSystemChatMessage("Mounting for flight...");

    // Save recovery target in case of disconnect during taxi.
    auto destIt = taxiNodes_.find(destNodeId);
    if (destIt != taxiNodes_.end()) {
        taxiRecoverMapId_ = destIt->second.mapId;
        taxiRecoverPos_ = core::coords::serverToCanonical(
            glm::vec3(destIt->second.x, destIt->second.y, destIt->second.z));
        taxiRecoverPending_ = false;
    }
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

void GameHandler::handleFriendStatus(network::Packet& packet) {
    FriendStatusData data;
    if (!FriendStatusParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_FRIEND_STATUS");
        return;
    }

    // Look up player name from GUID
    std::string playerName;
    auto it = playerNameCache.find(data.guid);
    if (it != playerNameCache.end()) {
        playerName = it->second;
    } else {
        playerName = "Unknown";
    }

    // Update friends cache
    if (data.status == 1 || data.status == 2) {  // Added or online
        friendsCache[playerName] = data.guid;
    } else if (data.status == 0) {  // Removed
        friendsCache.erase(playerName);
    }

    // Status messages
    switch (data.status) {
        case 0:
            addSystemChatMessage(playerName + " has been removed from your friends list.");
            break;
        case 1:
            addSystemChatMessage(playerName + " has been added to your friends list.");
            break;
        case 2:
            addSystemChatMessage(playerName + " is now online.");
            break;
        case 3:
            addSystemChatMessage(playerName + " is now offline.");
            break;
        case 4:
            addSystemChatMessage("Player not found.");
            break;
        case 5:
            addSystemChatMessage(playerName + " is already in your friends list.");
            break;
        case 6:
            addSystemChatMessage("Your friends list is full.");
            break;
        case 7:
            addSystemChatMessage(playerName + " is ignoring you.");
            break;
        default:
            LOG_INFO("Friend status: ", (int)data.status, " for ", playerName);
            break;
    }

    LOG_INFO("Friend status update: ", playerName, " status=", (int)data.status);
}

void GameHandler::handleRandomRoll(network::Packet& packet) {
    RandomRollData data;
    if (!RandomRollParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_RANDOM_ROLL");
        return;
    }

    // Get roller name
    std::string rollerName;
    if (data.rollerGuid == playerGuid) {
        rollerName = "You";
    } else {
        auto it = playerNameCache.find(data.rollerGuid);
        if (it != playerNameCache.end()) {
            rollerName = it->second;
        } else {
            rollerName = "Someone";
        }
    }

    // Build message
    std::string msg = rollerName;
    if (data.rollerGuid == playerGuid) {
        msg += " roll ";
    } else {
        msg += " rolls ";
    }
    msg += std::to_string(data.result);
    msg += " (" + std::to_string(data.minRoll) + "-" + std::to_string(data.maxRoll) + ")";

    addSystemChatMessage(msg);
    LOG_INFO("Random roll: ", rollerName, " rolled ", data.result, " (", data.minRoll, "-", data.maxRoll, ")");
}

void GameHandler::handleLogoutResponse(network::Packet& packet) {
    LogoutResponseData data;
    if (!LogoutResponseParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_LOGOUT_RESPONSE");
        return;
    }

    if (data.result == 0) {
        // Success - logout initiated
        if (data.instant) {
            addSystemChatMessage("Logging out...");
        } else {
            addSystemChatMessage("Logging out in 20 seconds...");
        }
        LOG_INFO("Logout response: success, instant=", (int)data.instant);
    } else {
        // Failure
        addSystemChatMessage("Cannot logout right now.");
        loggingOut_ = false;
        LOG_WARNING("Logout failed, result=", data.result);
    }
}

void GameHandler::handleLogoutComplete(network::Packet& /*packet*/) {
    addSystemChatMessage("Logout complete.");
    loggingOut_ = false;
    LOG_INFO("Logout complete");
    // Server will disconnect us
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


// ============================================================
// Player Skills
// ============================================================

static const std::string kEmptySkillName;

const std::string& GameHandler::getSkillName(uint32_t skillId) const {
    auto it = skillLineNames_.find(skillId);
    return (it != skillLineNames_.end()) ? it->second : kEmptySkillName;
}

uint32_t GameHandler::getSkillCategory(uint32_t skillId) const {
    auto it = skillLineCategories_.find(skillId);
    return (it != skillLineCategories_.end()) ? it->second : 0;
}

void GameHandler::loadSkillLineDbc() {
    if (skillLineDbcLoaded_) return;
    skillLineDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("SkillLine.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("GameHandler: Could not load SkillLine.dbc");
        return;
    }

    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id = dbc->getUInt32(i, 0);
        uint32_t category = dbc->getUInt32(i, 1);
        std::string name = dbc->getString(i, 3);
        if (id > 0 && !name.empty()) {
            skillLineNames_[id] = name;
            skillLineCategories_[id] = category;
        }
    }
    LOG_INFO("GameHandler: Loaded ", skillLineNames_.size(), " skill line names");
}

void GameHandler::extractSkillFields(const std::map<uint16_t, uint32_t>& fields) {
    loadSkillLineDbc();

    // PLAYER_SKILL_INFO_1_1 = field 636, 128 slots x 3 fields each (636..1019)
    static constexpr uint16_t PLAYER_SKILL_INFO_START = 636;
    static constexpr int MAX_SKILL_SLOTS = 128;

    std::map<uint32_t, PlayerSkill> newSkills;

    for (int slot = 0; slot < MAX_SKILL_SLOTS; slot++) {
        uint16_t baseField = PLAYER_SKILL_INFO_START + slot * 3;

        auto idIt = fields.find(baseField);
        if (idIt == fields.end()) continue;

        uint32_t raw0 = idIt->second;
        uint16_t skillId = raw0 & 0xFFFF;
        if (skillId == 0) continue;

        auto valIt = fields.find(baseField + 1);
        if (valIt == fields.end()) continue;

        uint32_t raw1 = valIt->second;
        uint16_t value = raw1 & 0xFFFF;
        uint16_t maxValue = (raw1 >> 16) & 0xFFFF;

        PlayerSkill skill;
        skill.skillId = skillId;
        skill.value = value;
        skill.maxValue = maxValue;
        newSkills[skillId] = skill;
    }

    // Detect increases and emit chat messages
    for (const auto& [skillId, skill] : newSkills) {
        if (skill.value == 0) continue;
        auto oldIt = playerSkills_.find(skillId);
        if (oldIt != playerSkills_.end() && skill.value > oldIt->second.value) {
            // Filter out racial, generic, and hidden skills from announcements
            // Category 5 = Attributes (Defense, etc.)
            // Category 10 = Languages (Orcish, Common, etc.)
            // Category 12 = Not Displayed (generic/hidden)
            auto catIt = skillLineCategories_.find(skillId);
            if (catIt != skillLineCategories_.end()) {
                uint32_t category = catIt->second;
                if (category == 5 || category == 10 || category == 12) {
                    continue; // Skip announcement for racial/generic skills
                }
            }

            const std::string& name = getSkillName(skillId);
            std::string skillName = name.empty() ? ("Skill #" + std::to_string(skillId)) : name;
            addSystemChatMessage("Your skill in " + skillName + " has increased to " + std::to_string(skill.value) + ".");
        }
    }

    playerSkills_ = std::move(newSkills);
}

std::string GameHandler::getCharacterConfigDir() {
    std::string dir;
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    dir = appdata ? std::string(appdata) + "\\wowee\\characters" : "characters";
#else
    const char* home = std::getenv("HOME");
    dir = home ? std::string(home) + "/.wowee/characters" : "characters";
#endif
    return dir;
}

void GameHandler::saveCharacterConfig() {
    const Character* ch = getActiveCharacter();
    if (!ch || ch->name.empty()) return;

    std::string dir = getCharacterConfigDir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::string path = dir + "/" + ch->name + ".cfg";
    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save character config to ", path);
        return;
    }

    out << "character_guid=" << playerGuid << "\n";
    out << "gender=" << static_cast<int>(ch->gender) << "\n";
    out << "use_female_model=" << (ch->useFemaleModel ? 1 : 0) << "\n";
    for (int i = 0; i < ACTION_BAR_SLOTS; i++) {
        out << "action_bar_" << i << "_type=" << static_cast<int>(actionBar[i].type) << "\n";
        out << "action_bar_" << i << "_id=" << actionBar[i].id << "\n";
    }

    // Save quest log
    out << "quest_log_count=" << questLog_.size() << "\n";
    for (size_t i = 0; i < questLog_.size(); i++) {
        const auto& quest = questLog_[i];
        out << "quest_" << i << "_id=" << quest.questId << "\n";
        out << "quest_" << i << "_title=" << quest.title << "\n";
        out << "quest_" << i << "_complete=" << (quest.complete ? 1 : 0) << "\n";
    }

    LOG_INFO("Character config saved to ", path);
}

void GameHandler::loadCharacterConfig() {
    const Character* ch = getActiveCharacter();
    if (!ch || ch->name.empty()) return;

    std::string path = getCharacterConfigDir() + "/" + ch->name + ".cfg";
    std::ifstream in(path);
    if (!in.is_open()) return;

    uint64_t savedGuid = 0;
    std::array<int, ACTION_BAR_SLOTS> types{};
    std::array<uint32_t, ACTION_BAR_SLOTS> ids{};
    bool hasSlots = false;
    int savedGender = -1;
    int savedUseFemaleModel = -1;

    std::string line;
    while (std::getline(in, line)) {
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        if (key == "character_guid") {
            try { savedGuid = std::stoull(val); } catch (...) {}
        } else if (key == "gender") {
            try { savedGender = std::stoi(val); } catch (...) {}
        } else if (key == "use_female_model") {
            try { savedUseFemaleModel = std::stoi(val); } catch (...) {}
        } else if (key.rfind("action_bar_", 0) == 0) {
            // Parse action_bar_N_type or action_bar_N_id
            size_t firstUnderscore = 11; // length of "action_bar_"
            size_t secondUnderscore = key.find('_', firstUnderscore);
            if (secondUnderscore == std::string::npos) continue;
            int slot = -1;
            try { slot = std::stoi(key.substr(firstUnderscore, secondUnderscore - firstUnderscore)); } catch (...) { continue; }
            if (slot < 0 || slot >= ACTION_BAR_SLOTS) continue;
            std::string suffix = key.substr(secondUnderscore + 1);
            try {
                if (suffix == "type") {
                    types[slot] = std::stoi(val);
                    hasSlots = true;
                } else if (suffix == "id") {
                    ids[slot] = static_cast<uint32_t>(std::stoul(val));
                    hasSlots = true;
                }
            } catch (...) {}
        }
    }

    // Validate guid matches current character
    if (savedGuid != 0 && savedGuid != playerGuid) {
        LOG_WARNING("Character config guid mismatch for ", ch->name, ", using defaults");
        return;
    }

    // Apply saved gender and body type (allows nonbinary to persist even though server only stores male/female)
    if (savedGender >= 0 && savedGender <= 2) {
        for (auto& character : characters) {
            if (character.guid == playerGuid) {
                character.gender = static_cast<Gender>(savedGender);
                if (savedUseFemaleModel >= 0) {
                    character.useFemaleModel = (savedUseFemaleModel != 0);
                }
                LOG_INFO("Applied saved gender: ", getGenderName(character.gender),
                         ", body type: ", (character.useFemaleModel ? "feminine" : "masculine"));
                break;
            }
        }
    }

    if (hasSlots) {
        for (int i = 0; i < ACTION_BAR_SLOTS; i++) {
            actionBar[i].type = static_cast<ActionBarSlot::Type>(types[i]);
            actionBar[i].id = ids[i];
        }
        LOG_INFO("Character config loaded from ", path);
    }
}

glm::vec3 GameHandler::getComposedWorldPosition() {
    if (playerTransportGuid_ != 0 && transportManager_) {
        return transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
    }
    // Not on transport, return normal movement position
    return glm::vec3(movementInfo.x, movementInfo.y, movementInfo.z);
}

} // namespace game
} // namespace wowee
