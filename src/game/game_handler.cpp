#include "game/game_handler.hpp"
#include "game/item_text.hpp"
#include "game/achievement_criteria.hpp"
#include "game/game_utils.hpp"
#include "game/chat_handler.hpp"
#include "game/movement_handler.hpp"
#include "game/combat_handler.hpp"
#include "game/spell_handler.hpp"
#include "game/inventory_handler.hpp"
#include "game/social_handler.hpp"
#include "game/quest_handler.hpp"
#include "game/warden_handler.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
#include "game/packet_parsers.hpp"
#include "game/transport_manager.hpp"
#include "game/opcodes.hpp"
#include "game/update_field_table.hpp"
#include "game/expansion_profile.hpp"
#include "rendering/renderer.hpp"
#include "rendering/spell_visual_system.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "auth/crypto.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "core/config_paths.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include "game/protocol_constants.hpp"
#include "rendering/animation/animation_ids.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <random>
#include <zlib.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <array>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <openssl/sha.h>
#include <openssl/hmac.h>

namespace wowee {
namespace game {


// Registration helpers for common dispatch table patterns
void GameHandler::registerSkipHandler(LogicalOpcode op) {
    dispatchTable_[op] = [](network::Packet& packet) { packet.skipAll(); };
}
void GameHandler::registerHandler(LogicalOpcode op, void (GameHandler::*handler)(network::Packet&)) {
    dispatchTable_[op] = [this, handler](network::Packet& packet) { (this->*handler)(packet); };
}
GameHandler::GameHandler(GameServices& services)
    : services_(services) {
    LOG_DEBUG("GameHandler created");

    setActiveOpcodeTable(&opcodeTable_);
    setActiveUpdateFieldTable(&updateFieldTable_);

    // Initialize packet parsers (WotLK default, may be replaced for other expansions)
    packetParsers_ = std::make_unique<WotlkPacketParsers>();

    // Initialize transport manager
    transportManager_ = std::make_unique<TransportManager>();

    // Initialize Warden module manager

    // Initialize domain handlers
    entityController_ = std::make_unique<EntityController>(*this);
    chatHandler_      = std::make_unique<ChatHandler>(*this);
    movementHandler_  = std::make_unique<MovementHandler>(*this);
    combatHandler_    = std::make_unique<CombatHandler>(*this);
    spellHandler_     = std::make_unique<SpellHandler>(*this);
    inventoryHandler_ = std::make_unique<InventoryHandler>(*this);
    socialHandler_    = std::make_unique<SocialHandler>(*this);
    questHandler_     = std::make_unique<QuestHandler>(*this);
    wardenHandler_    = std::make_unique<WardenHandler>(*this);

    // Default action bar layout
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = game::SPELL_ID_ATTACK;   // Attack in slot 1
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = game::SPELL_ID_HEARTHSTONE;  // Hearthstone in slot 12

    // Build the opcode dispatch table (replaces switch(*logicalOp) in handlePacket)
    registerOpcodeHandlers();
}

GameHandler::~GameHandler() {
    disconnect();
}

void GameHandler::setPacketParsers(std::unique_ptr<PacketParsers> parsers) {
    packetParsers_ = std::move(parsers);
}

bool GameHandler::connect(const std::string& host,
                          uint16_t port,
                          const std::vector<uint8_t>& sessionKey,
                          const std::string& accountName,
                          uint32_t build,
                          uint32_t realmId) {

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
    this->realmId_ = realmId;

    // The session key is the shared secret the world handshake proves knowledge
    // of and the header cipher is keyed from: anyone holding these 40 bytes can
    // authenticate as this account and decrypt its traffic. It must never reach
    // a log, which is a file users are routinely asked to attach to a bug
    // report. Its length is the only part of it that diagnoses anything.
    LOG_INFO("GameHandler session key received (", sessionKey.size(), " bytes)");
    // Generate random client seed
    this->clientSeed = generateClientSeed();
    LOG_DEBUG("Generated client seed: 0x", std::hex, clientSeed, std::dec);

    // Create world socket
    socket = std::make_unique<network::WorldSocket>();

    // Set up packet callback
    socket->setPacketCallback([this](const network::Packet& packet) {
        enqueueIncomingPacket(packet);
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
    taxiRecoverPending_ = onTaxiFlight_;
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
    activeCharacterGuid_ = 0;
    guildNameCache_.clear();
    pendingGuildNameQueries_.clear();
    friendGuids_.clear();
    contacts_.clear();
    transportAttachments_.clear();
    // Warden state is WardenHandler's; this used to clear a second copy of it
    // that nothing ever filled.
    if (wardenHandler_) wardenHandler_->reset();
    pendingIncomingPackets_.clear();
    // Fire despawn callbacks so the renderer releases M2/character model resources.
    for (const auto& [guid, entity] : entityController_->getEntityManager().getEntities()) {
        if (guid == playerGuid) continue;
        if (entity->getType() == ObjectType::UNIT && creatureDespawnCallback_)
            creatureDespawnCallback_(guid);
        else if (entity->getType() == ObjectType::PLAYER && playerDespawnCallback_)
            playerDespawnCallback_(guid);
        else if (entity->getType() == ObjectType::GAMEOBJECT && gameObjectDespawnCallback_)
            gameObjectDespawnCallback_(guid);
    }
    otherPlayerVisibleItemEntries_.clear();
    otherPlayerVisibleDirty_.clear();
    otherPlayerMoveTimeMs_.clear();
    if (spellHandler_) spellHandler_->clearUnitCastStates();
    if (spellHandler_) spellHandler_->clearUnitAurasCache();
    if (combatHandler_) combatHandler_->clearCombatText();
    entityController_->clearAll();
    setState(WorldState::DISCONNECTED);
    LOG_INFO("Disconnected from world server");
}

void GameHandler::resetDbcCaches() {
    spellNameCacheLoaded_ = false;
    spellNameCache_.clear();
    skillLineDbcLoaded_ = false;
    skillLineNames_.clear();
    skillLineCategories_.clear();
    skillLineAbilityLoaded_ = false;
    spellToSkillLine_.clear();
    taxiNodes_.clear();
    taxiPathEdges_.clear();
    taxiPathNodes_.clear();
    areaTriggerDbcLoaded_ = false;
    areaTriggers_.clear();
    activeAreaTriggers_.clear();
    talentCache_.clear();
    talentTabCache_.clear();
    // The copies that are actually read live in the sub-handlers - the getters
    // beside these forward there. Clearing only the local ones left the
    // previous expansion's talents and flight points live after a switch.
    if (spellHandler_) spellHandler_->resetTalentDbcCache();
    if (movementHandler_) movementHandler_->resetTaxiDbcCache();
    // Clear the AssetManager DBC file cache so that expansion-specific DBCs
    // (CharSections, ItemDisplayInfo, etc.) are reloaded from the new expansion's
    // MPQ files instead of returning stale data from a previous session/expansion.
    auto* am = services_.assetManager;
    if (am) {
        am->clearDBCCache();
    }
    LOG_INFO("GameHandler: DBC caches cleared for expansion switch");
}

bool GameHandler::isConnected() const {
    return socket && socket->isConnected();
}

void GameHandler::sortBags() {
    if (!sortSwapQueue_.empty()) return;   // one sort at a time
    auto& inv = getInventory();
    // Pour partial stacks together first, so the sort places one stack of
    // twenty rather than two of ten. Merging both plans and applies, so the
    // swaps below are computed from the merged layout.
    auto merges = inv.mergePartialStacks();
    auto swaps = inv.computeSortSwaps();
    // The local layout changes now so the bags read as sorted immediately; the
    // server is told the same thing over the following ticks.
    inv.sortBags();
    // And say so. The layout above is the one the player asked for and it is
    // already in the model, but every bag on screen reads through
    // GetContainerItemInfo and redraws on BAG_UPDATE - so without this the
    // sorted bags sat behind the unsorted picture until something unrelated
    // forced a rebuild, which is most of what "sorting does nothing" looked
    // like. The moves queued below no longer fire it, having stopped touching
    // the model at all.
    notifyBagsChanged();
    for (auto& m : merges) sortSwapQueue_.push_back(m);
    for (auto& s : swaps)  sortSwapQueue_.push_back(s);
}

void GameHandler::sortBank(int mainSlotCount) {
    if (!sortSwapQueue_.empty()) return;   // one sort at a time
    auto& inv = getInventory();
    auto merges = inv.mergeBankPartialStacks(mainSlotCount);
    auto swaps = inv.computeBankSortSwaps(mainSlotCount);
    inv.sortBank(mainSlotCount);
    notifyBagsChanged();
    for (auto& m : merges) sortSwapQueue_.push_back(m);
    for (auto& s : swaps)  sortSwapQueue_.push_back(s);
}

void GameHandler::sortBankBag(int bagIndex) {
    if (!sortSwapQueue_.empty()) return;   // one sort at a time
    auto& inv = getInventory();
    auto swaps = inv.computeBankBagSortSwaps(bagIndex);
    inv.sortBankBag(bagIndex);
    notifyBagsChanged();
    for (auto& s : swaps) sortSwapQueue_.push_back(s);
}

void GameHandler::queuePacedChat(std::vector<std::string> lines) {
    for (std::string& line : lines) pacedChatQueue_.push_back(std::move(line));
}

void GameHandler::updateNetworking() {
    // One queued sort move per tick. A sort is dozens of swaps and the server
    // drops a burst of them, so they go out at the rate anything else does.
    if (!sortSwapQueue_.empty()) {
        const auto op = sortSwapQueue_.front();
        sortSwapQueue_.pop_front();
        // Sent, not applied. The sort put the finished layout into the model
        // before queuing any of this; moving each pair again on the way out
        // permuted that layout a second time - and a merge, which the server
        // answers by combining two stacks, would have been applied here as a
        // swap of the two slots. See swapContainerItems.
        swapContainerItems(op.srcBag, op.srcSlot, op.dstBag, op.dstSlot, false);
    }

    // Reset per-tick monster-move budget tracking (Classic/Turtle flood protection).
    if (movementHandler_) {
        movementHandler_->monsterMovePacketsThisTickRef() = 0;
        movementHandler_->monsterMovePacketsDroppedThisTickRef() = 0;
    }

    // One queued chat line per tick, for the same reason and at the same rate
    // as the sort moves above.
    if (!pacedChatQueue_.empty()) {
        const std::string line = std::move(pacedChatQueue_.front());
        pacedChatQueue_.pop_front();
        sendChatMessage(ChatType::SAY, line, "");
    }

    // Anything still waiting on a name query that never came back. After the
    // socket below would be a tick later than it needs to be; before it means
    // a line waits one extra tick at most.
    if (chatHandler_) chatHandler_->expireChatAwaitingName();

    // Update socket (processes incoming data and triggers callbacks)
    if (socket) {
        auto socketStart = std::chrono::steady_clock::now();
        socket->update();
        float socketMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - socketStart).count();
        if (socketMs > 3.0f) {
            LOG_WARNING("SLOW socket->update: ", socketMs, "ms");
        }
    }

    {
        auto packetStart = std::chrono::steady_clock::now();
        processQueuedIncomingPackets();
        float packetMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - packetStart).count();
        if (packetMs > 3.0f) {
            LOG_WARNING("SLOW queued packet handling: ", packetMs, "ms");
        }
    }

    // Detect RX silence (server stopped sending packets but TCP still open)
    if (isInWorld() && socket->isConnected() &&
        lastRxTime_.time_since_epoch().count() > 0) {
        auto silenceMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - lastRxTime_).count();
        if (silenceMs > game::RX_SILENCE_WARNING_MS && !rxSilenceLogged_) {
            rxSilenceLogged_ = true;
            LOG_WARNING("RX SILENCE: No packets from server for ", silenceMs, "ms - possible soft disconnect");
        }
        if (silenceMs > game::RX_SILENCE_CRITICAL_MS && !rxSilence15sLogged_) {
            rxSilence15sLogged_ = true;
            LOG_WARNING("RX SILENCE: 15s - server appears to have stopped sending");
        }
    }

    // Detect server-side disconnect (socket closed during update)
    if (socket && !socket->isConnected() && state != WorldState::DISCONNECTED) {
        if (pendingIncomingPackets_.empty() && !entityController_->hasPendingUpdateObjectWork()) {
            LOG_WARNING("Server closed connection in state: ", worldStateName(state));
            disconnect();
            return;
        }
        LOG_DEBUG("World socket closed with ", pendingIncomingPackets_.size(),
                  " queued packet(s) and update-object batch(es) pending dispatch");
    }

}

void GameHandler::updateTaxiAndMountState(float deltaTime) {
// MovementHandler owns the active taxi path state. Always let it advance its
// client path; updateClientTaxi() is a cheap no-op when no taxi is active.
// Gating this on GameHandler's legacy duplicate onTaxiFlight_ flag stranded
// every route at waypoint zero after taxi handling was extracted.
updateClientTaxi(deltaTime);

// Update taxi landing cooldown
if (taxiLandingCooldown_ > 0.0f) {
    taxiLandingCooldown_ -= deltaTime;
}
if (taxiStartGrace_ > 0.0f) {
    taxiStartGrace_ -= deltaTime;
}
if (playerTransportStickyTimer_ > 0.0f) {
    playerTransportStickyTimer_ -= deltaTime;
    if (playerTransportStickyTimer_ <= 0.0f) {
        playerTransportStickyTimer_ = 0.0f;
        playerTransportStickyGuid_ = 0;
    }
}

// Landing detection/cleanup already happens inside MovementHandler::updateClientTaxi()'s
// finishTaxiFlight() (called unconditionally above, using MovementHandler's own real taxi
// state) - mount callback, MSG_MOVE_STOP/HEARTBEAT, state reset, all handled there. Upstream
// independently discovered the same "updateClientTaxi() never called" bug and fixed it by
// making the call above unconditional, but kept this landing-detection duplicate gated on
// GameHandler's onTaxiFlight_ - the same legacy copy noted above, never set true reachably
// (activateTaxi() only sets MovementHandler's copy), so it's dead code here. Dropped rather
// than merged in to avoid double-processing landing every frame a flight completes.

// Safety: if taxi flight ended but mount is still active, force dismount.
// Guard against transient taxi-state flicker.
if (!onTaxiFlight_ && taxiMountActive_) {
    bool serverStillTaxi = false;
    auto playerEntity = entityController_->getEntityManager().getEntity(playerGuid);
    auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
    if (playerUnit) {
        serverStillTaxi = (playerUnit->getUnitFlags() & game::UNIT_FLAG_TAXI_FLIGHT) != 0;
    }

    if (taxiStartGrace_ > 0.0f || serverStillTaxi || taxiClientActive_ || taxiActivatePending_) {
        onTaxiFlight_ = true;
    } else {
        if (mountCallback_) mountCallback_(0);
        taxiMountActive_ = false;
        currentMountDisplayId_ = 0;
        movementInfo.flags = 0;
        movementInfo.flags2 = 0;
        if (socket) {
            sendMovement(Opcode::MSG_MOVE_STOP);
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
        }
        LOG_INFO("Taxi dismount cleanup");
    }
}

// Keep non-taxi mount state server-authoritative.
// Some server paths don't emit explicit mount field updates in lockstep
// with local visual state changes, so reconcile continuously.
if (!(movementHandler_ && movementHandler_->isOnTaxiFlight()) && !taxiMountActive_) {
    auto playerEntity = entityController_->getEntityManager().getEntity(playerGuid);
    auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
    if (playerUnit) {
        uint32_t serverMountDisplayId = playerUnit->getMountDisplayId();
        if (serverMountDisplayId != currentMountDisplayId_) {
            LOG_INFO("Mount reconcile: server=", serverMountDisplayId,
                     " local=", currentMountDisplayId_);
            currentMountDisplayId_ = serverMountDisplayId;
            if (mountCallback_) {
                mountCallback_(serverMountDisplayId);
            }
        }
    }
}

if (taxiRecoverPending_ && state == WorldState::IN_WORLD) {
    auto playerEntity = entityController_->getEntityManager().getEntity(playerGuid);
    if (playerEntity) {
        playerEntity->setPosition(taxiRecoverPos_.x, taxiRecoverPos_.y,
                                  taxiRecoverPos_.z, movementInfo.orientation);
        movementInfo.x = taxiRecoverPos_.x;
        movementInfo.y = taxiRecoverPos_.y;
        movementInfo.z = taxiRecoverPos_.z;
        if (socket) {
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
        }
        taxiRecoverPending_ = false;
        LOG_INFO("Taxi recovery applied");
    }
}

if (taxiActivatePending_) {
    taxiActivateTimer_ += deltaTime;
    if (taxiActivateTimer_ > 5.0f) {
        // If client taxi simulation is already active, server reply may be missing/late.
        // Do not cancel the flight in that case; clear pending state and continue.
        if (onTaxiFlight_ || taxiClientActive_ || taxiMountActive_) {
            taxiActivatePending_ = false;
            taxiActivateTimer_ = 0.0f;
        } else {
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        if (taxiMountActive_ && mountCallback_) {
            mountCallback_(0);
        }
        taxiMountActive_ = false;
        taxiClientActive_ = false;
        taxiClientPath_.clear();
        onTaxiFlight_ = false;
        LOG_WARNING("Taxi activation timed out");
        }
    }
}
}

void GameHandler::updateAutoAttack(float deltaTime) {
    if (combatHandler_) combatHandler_->updateAutoAttack(deltaTime);

// Close NPC windows if player walks too far (15 units)
}

void GameHandler::updateEntityInterpolation(float deltaTime) {
// Advance every server-tracked unit once per frame. This is intentionally not
// selected-target or distance gated: the spatial query is indexed by each
// mover's latest destination, so a creature visibly beside the player could be
// excluded while its destination was outside the query radius. Selecting that
// creature then put it on the unconditional path and made it snap forward.
// Entity interpolation is inexpensive; rendering retains its own distance
// culling and remains the correct place to skip costly visual work.
for (const auto& [guid, entity] : entityController_->getEntityManager().getEntities()) {
    (void)guid;
    if (entity && entity->isUnit()) {
        entity->updateMovement(deltaTime);
    }
}
}

void GameHandler::updateTimers(float deltaTime) {
    if (spellHandler_) spellHandler_->updateTimers(deltaTime);

    // Periodically clear stale pending item queries so they can be retried.
    // Without this, a lost/malformed response leaves the entry stuck forever.
    pendingItemQueryTimer_ += deltaTime;
    if (pendingItemQueryTimer_ >= 5.0f) {
        pendingItemQueryTimer_ = 0.0f;
        if (!pendingItemQueries_.empty()) {
            LOG_DEBUG("Clearing ", pendingItemQueries_.size(), " stale pending item queries");
            pendingItemQueries_.clear();
        }
    }

    // Tick InventoryHandler's auction search cooldown (the authoritative
    // timer - GameHandler previously ticked its own never-set duplicate).
    if (inventoryHandler_) inventoryHandler_->tickAuctionSearchDelay(deltaTime);

    // Tick QuestHandler's pending-accept timeouts (the authoritative maps -
    // GameHandler previously ticked its own never-populated copies, so lost
    // or rejected quest accepts were never resynced or unblocked).
    if (questHandler_) {
        questHandler_->tickQuestGiverStatusRequery(deltaTime);
        auto& acceptTimeouts = questHandler_->pendingQuestAcceptTimeoutsRef();
        auto& acceptNpcGuids = questHandler_->pendingQuestAcceptNpcGuidsRef();
        for (auto it = acceptTimeouts.begin(); it != acceptTimeouts.end();) {
            it->second -= deltaTime;
            if (it->second <= 0.0f) {
                const uint32_t questId = it->first;
                auto guidIt = acceptNpcGuids.find(questId);
                const uint64_t npcGuid = guidIt != acceptNpcGuids.end() ? guidIt->second : 0;
                triggerQuestAcceptResync(questId, npcGuid, "timeout");
                it = acceptTimeouts.erase(it);
                acceptNpcGuids.erase(questId);
            } else {
                ++it;
            }
        }
    }

    // Announce the corpse-proximity crossing. FrameXML answers CORPSE_IN_RANGE
    // with StaticPopup "RECOVER_CORPSE", whose button calls RetrieveCorpse;
    // without the edge that prompt could never appear and this client's own
    // button was the only way back from a corpse run.
    {
        const bool inRange = releasedSpirit_ && canReclaimCorpse();
        if (inRange != corpseInRangeAnnounced_) {
            corpseInRangeAnnounced_ = inRange;
            fireAddonEvent(inRange ? "CORPSE_IN_RANGE" : "CORPSE_OUT_OF_RANGE", {});
        }
    }

    if (pendingMoneyDeltaTimer_ > 0.0f) {
        pendingMoneyDeltaTimer_ -= deltaTime;
        if (pendingMoneyDeltaTimer_ <= 0.0f) {
            pendingMoneyDeltaTimer_ = 0.0f;
            pendingMoneyDelta_ = 0;
        }
    }
    // autoAttackRangeWarnCooldown_ decrement moved into CombatHandler::updateAutoAttack()

    if (pendingLoginQuestResync_) {
        pendingLoginQuestResyncTimeout_ -= deltaTime;
        if (resyncQuestLogFromServerSlots(true)) {
            pendingLoginQuestResync_ = false;
            pendingLoginQuestResyncTimeout_ = 0.0f;
        } else if (pendingLoginQuestResyncTimeout_ <= 0.0f) {
            pendingLoginQuestResync_ = false;
            pendingLoginQuestResyncTimeout_ = 0.0f;
            LOG_WARNING("Quest login resync timed out waiting for player quest slot fields");
        }
    }

    for (auto it = pendingGameObjectLootRetries_.begin(); it != pendingGameObjectLootRetries_.end();) {
        it->timer -= deltaTime;
        if (it->timer <= 0.0f) {
            if (it->remainingRetries > 0 && isInWorld()) {
                // Keep server-side position/facing fresh before retrying GO use.
                sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                auto usePacket = GameObjectUsePacket::build(it->guid);
                socket->send(usePacket);
                if (it->sendLoot) {
                    auto lootPacket = LootPacket::build(it->guid);
                    socket->send(lootPacket);
                }
                --it->remainingRetries;
                it->timer = 0.20f;
            }
        }
        if (it->remainingRetries == 0) {
            it = pendingGameObjectLootRetries_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = pendingGameObjectLootOpens_.begin(); it != pendingGameObjectLootOpens_.end();) {
        it->timer -= deltaTime;
        if (it->timer <= 0.0f) {
            if (isInWorld()) {
                // Avoid sending CMSG_LOOT while a timed cast is active (e.g. gathering).
                // handleSpellGo will trigger loot after the cast completes.
                if (spellHandler_ && spellHandler_->isCasting() && spellHandler_->getCurrentCastSpellId() != 0) {
                    it->timer = 0.20f;
                    ++it;
                    continue;
                }
                // The click may have landed before the object's metadata did.
                // A fishing school is never opened by looting it.
                if (isFishingHoleGameObject(it->guid)) {
                    it = pendingGameObjectLootOpens_.erase(it);
                    continue;
                }
                lootTarget(it->guid);
            }
            if (it->remainingAttempts > 1) {
                --it->remainingAttempts;
                it->timer = 0.75f;
                ++it;
            } else {
                it = pendingGameObjectLootOpens_.erase(it);
            }
        } else {
            ++it;
        }
    }

    // Periodically re-query names for players whose initial CMSG_NAME_QUERY was
    // lost (server didn't respond) or whose entity was recreated while the query
    // was still pending. Runs every 5 seconds to keep overhead minimal.
    if (isInWorld()) {
        static float nameResyncTimer = 0.0f;
        nameResyncTimer += deltaTime;
        if (nameResyncTimer >= 5.0f) {
            nameResyncTimer = 0.0f;
            for (const auto& [guid, entity] : entityController_->getEntityManager().getEntities()) {
                if (!entity || entity->getType() != ObjectType::PLAYER) continue;
                if (guid == playerGuid) continue;
                auto player = std::static_pointer_cast<Player>(entity);
                if (!player->getName().empty()) continue;
                // Player entity exists with empty name and no pending query - resend.
                entityController_->queryPlayerName(guid);
            }
        }
    }

    // The fallback for a server that sends no SMSG_LOOT_MONEY_NOTIFY lives
    // with the state it reads - this ticked a copy of its own that nothing
    // ever set, so the fallback never fired.
    if (inventoryHandler_) inventoryHandler_->tickLootMoneyFallback(deltaTime);

    for (auto it = recentLootMoneyAnnounceCooldowns_.begin(); it != recentLootMoneyAnnounceCooldowns_.end();) {
        it->second -= deltaTime;
        if (it->second <= 0.0f) {
            it = recentLootMoneyAnnounceCooldowns_.erase(it);
        } else {
            ++it;
        }
    }

    // Auto-inspect throttling (fallback for player equipment visuals).
    if (inspectRateLimit_ > 0.0f) {
        inspectRateLimit_ = std::max(0.0f, inspectRateLimit_ - deltaTime);
    }
    if (isInWorld() && inspectRateLimit_ <= 0.0f && !pendingAutoInspect_.empty()) {
        uint64_t guid = *pendingAutoInspect_.begin();
        pendingAutoInspect_.erase(pendingAutoInspect_.begin());
        if (guid != 0 && guid != playerGuid && entityController_->getEntityManager().hasEntity(guid)) {
            auto pkt = InspectPacket::build(guid);
            socket->send(pkt);
            inspectRateLimit_ = 2.0f; // throttle to avoid compositing stutter
            LOG_DEBUG("Sent CMSG_INSPECT for player 0x", std::hex, guid, std::dec);
        }
    }
}

void GameHandler::update(float deltaTime) {
    // The char-create callback used to be deferred to here, out of the packet
    // handler and so out of any render pass. It is not any more: the
    // SMSG_CHAR_CREATE handler calls charCreateCallback_ itself, and nothing
    // had set the flag this waited on for long enough that the three members
    // behind it were never written at all. A block that cannot run, guarding a
    // deferral that no longer happens, reads as though both still do.
    if (!socket) {
        return;
    }

    updateNetworking();
    if (!socket) return;  // disconnect() may have been called

    // Fallback for CMSG_CHAR_DELETE with no server response: if the server
    // doesn't send SMSG_CHAR_DELETE within 3 seconds, re-request the character
    // list.  Some server cores silently process the delete without responding.
    if (pendingCharDeleteResponse_) {
        pendingDeleteTimer_ += deltaTime;
        if (pendingDeleteTimer_ >= 3.0f) {
            LOG_WARNING("No SMSG_CHAR_DELETE response after 3s - requesting character list to verify");
            pendingCharDeleteResponse_ = false;
            pendingDeleteFallbackEnum_ = true;
            requestCharacterList();
        }
    }

    // After the fallback SMSG_CHAR_ENUM has been processed, check if the
    // character was actually removed and fire the delete callback.
    if (pendingDeleteFallbackEnum_ && state == WorldState::CHAR_LIST_RECEIVED) {
        pendingDeleteFallbackEnum_ = false;
        uint64_t deletedGuid = pendingDeleteGuid_;
        pendingDeleteGuid_ = 0;
        bool found = false;
        for (const auto& ch : characters) {
            if (ch.guid == deletedGuid) { found = true; break; }
        }
        bool deleted = !found;
        LOG_INFO("Char delete fallback: GUID 0x", std::hex, deletedGuid, std::dec,
                 deleted ? " was deleted" : " still exists");
        std::string msg;
        if (deleted) {
            msg = "Character deleted.";
        } else {
            msg = "Delete failed: the server did not respond. "
                  "This usually happens if you recently logged out - "
                  "wait 20-30 seconds and try again.";
        }
        if (charDeleteCallback_) charDeleteCallback_(deleted, msg);
    }

    // Validate target still exists
    if (targetGuid != 0 && !entityController_->getEntityManager().hasEntity(targetGuid)) {
        clearTarget();
    }

    // Update auto-follow: refresh render position or cancel if entity disappeared
    if (followTargetGuid_ != 0) {
        auto followEnt = entityController_->getEntityManager().getEntity(followTargetGuid_);
        if (followEnt) {
            followRenderPos_ = core::coords::canonicalToRender(
                glm::vec3(followEnt->getX(), followEnt->getY(), followEnt->getZ()));
            if (movementHandler_) movementHandler_->updateFollowMovement(deltaTime);
        } else {
            cancelFollow();
        }
    }

    // Entering and leaving combat is fired from the update block that carries
    // UNIT_FLAG_IN_COMBAT, in EntityController - not from here.
    //
    // Both places used to fire it, so every fight announced itself twice: the
    // combat text showed "Entering Combat" and then "Entering Combat" again,
    // and the same on the way out. The two do not even agree on the question.
    // This one asked isInCombat(), which is `autoAttacking_ ||
    // !hostileAttackers_.empty()` - the client's own inference from what it has
    // seen swing - while the other reads the flag the *server* sets, which is
    // what PLAYER_REGEN_DISABLED means in WoW and what decides whether a panel
    // may be changed. Two sources, two edges, at slightly different moments.

    updateTimers(deltaTime);

    // Send periodic heartbeat if in world
    if (state == WorldState::IN_WORLD) {
        timeSinceLastPing += deltaTime;
        if (movementHandler_) movementHandler_->timeSinceLastMoveHeartbeatRef() += deltaTime;

        const float currentPingInterval =
            (isPreWotlk()) ? game::CLASSIC_PING_INTERVAL_SEC : pingInterval;
        if (timeSinceLastPing >= currentPingInterval) {
            if (socket) {
                sendPing();
            }
            timeSinceLastPing = 0.0f;
        }

        const bool classicLikeCombatSync =
            (combatHandler_ && combatHandler_->hasAutoAttackIntent()) && (isPreWotlk());
        const bool onRealTaxiFlight = movementHandler_ && movementHandler_->isOnTaxiFlight();
        const bool classicLikeStationaryCombatSync =
            classicLikeCombatSync &&
            !onRealTaxiFlight &&
            !taxiActivatePending_ &&
            !taxiClientActive_ &&
            (movementInfo.flags & kLocomotionFlags) == 0;
        float heartbeatInterval = (onRealTaxiFlight || taxiActivatePending_ || taxiClientActive_)
                                      ? game::HEARTBEAT_INTERVAL_TAXI
                                      : (classicLikeStationaryCombatSync ? game::HEARTBEAT_INTERVAL_STATIONARY_COMBAT
                                                                         : (classicLikeCombatSync ? game::HEARTBEAT_INTERVAL_MOVING_COMBAT
                                                                                                  : moveHeartbeatInterval_));
        if (movementHandler_ && movementHandler_->timeSinceLastMoveHeartbeatRef() >= heartbeatInterval) {
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
            movementHandler_->timeSinceLastMoveHeartbeatRef() = 0.0f;
        }

        // Check area triggers (instance portals, tavern rests, etc.)
        if (areaTriggerCooldown_ > 0.0f) areaTriggerCooldown_ -= deltaTime;
        areaTriggerCheckTimer_ += deltaTime;
        if (areaTriggerCheckTimer_ >= game::AREA_TRIGGER_CHECK_INTERVAL) {
            areaTriggerCheckTimer_ = 0.0f;
            checkAreaTriggers();
        }

        // Cancel GO interaction cast if player enters combat (auto-attack).
        if (pendingGameObjectInteractGuid_ != 0 &&
            combatHandler_ && (combatHandler_->isAutoAttacking() || combatHandler_->hasAutoAttackIntent())) {
            pendingGameObjectInteractGuid_ = 0;
            if (spellHandler_) spellHandler_->resetCastState();
            addUIError("Interrupted.");
            addSystemChatMessage("Interrupted.");
        }
        // Check if client-side cast timer expired (tick-down is in SpellHandler::updateTimers).
        // Three paths depending on whether this is a GO interaction or craft-queue cast:
        if (spellHandler_ && spellHandler_->isCasting() && spellHandler_->getCastTimeRemaining() <= 0.0f) {
            if (pendingGameObjectInteractGuid_ != 0) {
                // GO interaction cast: do NOT call resetCastState() here. The server
                // sends SMSG_SPELL_GO when the cast completes server-side (~50-200ms
                // after the client timer expires due to float precision/frame timing).
                // handleSpellGo checks `wasInTimedCast = casting_ && spellId == currentCastSpellId_`
                // - if we clear those fields now, wasInTimedCast is false and the loot
                // path (CMSG_LOOT via lastInteractedGoGuid_) never fires.
                // Let the cast bar sit at 100% until SMSG_SPELL_GO arrives to clean up.
                pendingGameObjectInteractGuid_ = 0;
            } else if (spellHandler_->getCraftQueueRemaining() > 0 && craftCastGoGraceSec_ < 2.0f) {
                // Craft queue cast: SMSG_SPELL_GO is what decrements the queue and
                // re-casts the next item, and it races this client-side timer.
                // resetCastState() here would wipe the queue mid-run ("Create All"
                // stopping after one item), so let the cast bar sit at 100% briefly.
                // The 2s grace bails out if SPELL_GO never arrives (cast failed
                // without a result packet, e.g. reagents gone).
                craftCastGoGraceSec_ += deltaTime;
            } else {
                // Regular cast with no GO pending: clean up immediately.
                spellHandler_->resetCastState();
            }
        } else {
            craftCastGoGraceSec_ = 0.0f;
        }

        // Unit cast states and spell cooldowns are ticked by SpellHandler::updateTimers()
        // (called from GameHandler::updateTimers above). No duplicate tick-down here.

        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.cooldownRemaining > 0.0f) {
                slot.cooldownRemaining -= deltaTime;
                if (slot.cooldownRemaining < 0.0f) slot.cooldownRemaining = 0.0f;
            }
        }

        // Update combat text (Phase 2)
        updateCombatText(deltaTime);
        tickMinimapPings(deltaTime);
        tickMirrorTimers(deltaTime);

        // Tick logout countdown
        if (socialHandler_) socialHandler_->updateLogoutCountdown(deltaTime);

        updateTaxiAndMountState(deltaTime);

        // Update transport manager
        if (transportManager_) {
            transportManager_->update(deltaTime);
            updateAttachedTransportChildren(deltaTime);
        }

        updateAutoAttack(deltaTime);
        auto closeIfTooFar = [&](bool windowOpen, uint64_t npcGuid, auto closeFn, const char* label) {
            if (!windowOpen || npcGuid == 0) return;
            auto npc = entityController_->getEntityManager().getEntity(npcGuid);
            if (!npc) return;
            float dx = movementInfo.x - npc->getX();
            float dy = movementInfo.y - npc->getY();
            if (std::sqrt(dx * dx + dy * dy) > game::NPC_INTERACT_MAX_DISTANCE) {
                closeFn();
                LOG_INFO(label, " closed: walked too far from NPC");
            }
        };
        closeIfTooFar(isVendorWindowOpen(), getVendorItems().vendorGuid, [this]{ closeVendor(); }, "Vendor");
        closeIfTooFar(isGossipWindowOpen(), getCurrentGossip().npcGuid, [this]{ closeGossip(); }, "Gossip");
        // The movement handler's guid, not a copy of it: the one this class
        // kept was never assigned, so this asked about guid 0, found no NPC and
        // returned - the only one of these four windows that stayed open when
        // the player walked away from it.
        closeIfTooFar(isTaxiWindowOpen(),
                      movementHandler_ ? movementHandler_->getTaxiNpcGuid() : 0,
                      [this]{ closeTaxi(); }, "Taxi window");
        closeIfTooFar(isTrainerWindowOpen(), getTrainerSpells().trainerGuid, [this]{ closeTrainer(); }, "Trainer");

        updateEntityInterpolation(deltaTime);

    }
}

// ============================================================
// Single-player local combat
// ============================================================

// ============================================================
// XP tracking
// ============================================================

uint32_t GameHandler::killXp(uint32_t playerLevel, uint32_t victimLevel) {
    return CombatHandler::killXp(playerLevel, victimLevel);
}

void GameHandler::handleXpGain(network::Packet& packet) {
    if (combatHandler_) combatHandler_->handleXpGain(packet);
}

void GameHandler::addMoneyCopper(uint32_t amount) {
    if (inventoryHandler_) inventoryHandler_->addMoneyCopper(amount);
}

void GameHandler::addSystemChatMessage(const std::string& message) {
    if (chatHandler_) chatHandler_->addSystemChatMessage(message);
}
void GameHandler::addLocalChatLine(game::ChatType type, const std::string& message) {
    if (chatHandler_) chatHandler_->addLocalChatLine(type, message);
}

void GameHandler::raiseUiError(const std::string& message) {
    // On screen and nowhere else, which is where the real client puts these.
    //
    // This wrote the same line to the chat log as well, on the grounds that
    // these messages had been chat-only before the on-screen line existed and
    // taking them out would lose them. In a fight it loses nothing and costs a
    // great deal: "Not ready", "Not enough rage" and "You can't do that right
    // now" arrive once per rejected keypress, so a few seconds of combat
    // scrolls the log past everything that mattered - the loot, the whispers,
    // the guild chat. UIErrorsFrame is the red line above the middle of the
    // screen and it is transient on purpose.
    //
    // Through addUIError rather than firing the event again here: it already
    // raises UI_ERROR_MESSAGE, which is what addons watch and what
    // UIErrorsFrame is registered for.
    addUIError(message);
}

// ============================================================
// Taxi / Flight Path Handlers
// ============================================================

void GameHandler::updateClientTaxi(float deltaTime) {
    if (movementHandler_) movementHandler_->updateClientTaxi(deltaTime);
}

void GameHandler::closeTaxi() {
    if (movementHandler_) movementHandler_->closeTaxi();
}

uint32_t GameHandler::getTaxiCostTo(uint32_t destNodeId) const {
    if (movementHandler_) return movementHandler_->getTaxiCostTo(destNodeId);
    return 0;
}

bool GameHandler::hasTaxiRouteTo(uint32_t destNodeId) const {
    return movementHandler_ && movementHandler_->hasTaxiRouteTo(destNodeId);
}

std::vector<uint32_t> GameHandler::getTaxiRouteTo(uint32_t destNodeId) const {
    if (movementHandler_) return movementHandler_->getTaxiRouteTo(destNodeId);
    return {};
}

void GameHandler::activateTaxi(uint32_t destNodeId) {
    if (movementHandler_) movementHandler_->activateTaxi(destNodeId);
}

// ============================================================
// Server Info Command Handlers
// ============================================================

uint32_t GameHandler::generateClientSeed() {
    // Generate cryptographically random seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(1, 0xFFFFFFFF);
    return dis(gen);
}

void GameHandler::setState(WorldState newState) {
    if (state != newState) {
        LOG_DEBUG("World state: ", static_cast<int>(state), " -> ", static_cast<int>(newState));
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
    // Asked for a name is a reason to read the file. The load used to be
    // driven only from the *write* paths - a player update block carrying
    // skill fields, the spellbook building its tabs - so if every one of those
    // ran before the assets were up, nothing ever went back for it and every
    // skill stayed nameless. getSpellName has always done this; this did not.
    loadSkillLineDbc();
    auto it = skillLineNames_.find(skillId);
    return (it != skillLineNames_.end()) ? it->second : kEmptySkillName;
}

uint32_t GameHandler::getSkillCategory(uint32_t skillId) const {
    // Loaded on the ask, like getSkillName above and for the same reason: the
    // write paths can all have run before the assets were up, and then nothing
    // goes back for the file.
    loadSkillLineDbc();
    auto it = skillLineCategories_.find(skillId);
    return (it != skillLineCategories_.end()) ? it->second : 0;
}

const std::string& GameHandler::getSkillCategoryName(uint32_t categoryId) const {
    loadSkillLineDbc();
    auto it = skillCategoryNames_.find(categoryId);
    return (it != skillCategoryNames_.end()) ? it->second : kEmptySkillName;
}

uint32_t GameHandler::getSkillCategorySortIndex(uint32_t categoryId) const {
    loadSkillLineDbc();
    auto it = skillCategorySort_.find(categoryId);
    // Unknown headings sort last rather than first, so a category this build's
    // file does not name cannot displace the ones it does.
    return (it != skillCategorySort_.end()) ? it->second : 0xFFFFFFFFu;
}

void GameHandler::setSkillCategoryCollapsed(uint32_t categoryId, bool collapsed) {
    if (categoryId == 0) return;
    if (collapsed) collapsedSkillCategories_.insert(categoryId);
    else           collapsedSkillCategories_.erase(categoryId);
}

bool GameHandler::isProfessionSpell(uint32_t spellId) const {
    auto slIt = spellToSkillLine_.find(spellId);
    if (slIt == spellToSkillLine_.end()) return false;
    auto catIt = skillLineCategories_.find(slIt->second);
    if (catIt == skillLineCategories_.end()) return false;
    // Category 11 = profession (Blacksmithing, etc.), 9 = secondary (Cooking, First Aid, Fishing)
    return catIt->second == 11 || catIt->second == 9;
}

void GameHandler::loadSkillLineDbc() const {
    if (spellHandler_) spellHandler_->loadSkillLineDbc();
}

void GameHandler::extractSkillFields(const FlatFieldMap& fields) {
    if (spellHandler_) spellHandler_->extractSkillFields(fields);
}

void GameHandler::extractExploredZoneFields(const FlatFieldMap& fields) {
    if (spellHandler_) spellHandler_->extractExploredZoneFields(fields);
}

std::string GameHandler::getCharacterConfigDir() {
    return core::getConfigRoot() + "/characters";
}

static const std::string EMPTY_MACRO_TEXT;

const std::string& GameHandler::getMacroText(uint32_t macroId) const {
    auto it = macros_.find(macroId);
    return (it != macros_.end()) ? it->second : EMPTY_MACRO_TEXT;
}

int GameHandler::getRecipeDifficulty(uint32_t spellId) const {
    auto cit = spellNameCache_.find(spellId);
    if (cit == spellNameCache_.end()) return 0;
    const auto& se = cit->second;
    // No thresholds means the recipe never greys out.
    if (se.trivialSkillHigh == 0 && se.trivialSkillLow == 0) return 0;
    auto slIt = spellToSkillLine_.find(spellId);
    if (slIt == spellToSkillLine_.end()) return 0;
    auto skIt = getPlayerSkills().find(slIt->second);
    if (skIt == getPlayerSkills().end()) return 0;
    const uint32_t skill = skIt->second.effectiveValue();
    if (skill >= se.trivialSkillHigh) return 3;
    if (skill >= se.trivialSkillLow)  return 2;
    const uint32_t yellow = se.minSkillRank +
                            (se.trivialSkillLow - se.minSkillRank) / 2;
    if (skill >= yellow) return 1;
    return 0;
}

uint32_t GameHandler::countItemInBags(uint32_t itemId) const {
    return getInventory().countItem(itemId);
}

std::vector<GameHandler::CraftRecipe> GameHandler::getCraftingRecipes() const {
    std::vector<CraftRecipe> out;
    const uint32_t skillLine = getCraftingSkillLine();
    if (skillLine == 0) return out;
    for (uint32_t spellId : getKnownSpells()) {
        auto slIt = spellToSkillLine_.find(spellId);
        if (slIt == spellToSkillLine_.end() || slIt->second != skillLine) continue;
        auto cit = spellNameCache_.find(spellId);
        if (cit == spellNameCache_.end()) continue;
        const auto& se = cit->second;

        bool hasReagents = false;
        for (const auto& r : se.reagents) {
            if (r.itemId != 0) { hasReagents = true; break; }
        }
        // The window-opener spell and passive skill ranks are not recipes.
        if (!hasReagents && se.createdItemId == 0) continue;

        CraftRecipe row;
        row.spellId = spellId;
        row.name = se.name;
        row.difficulty = getRecipeDifficulty(spellId);
        row.canMake = hasReagents ? 999 : 0;
        for (const auto& r : se.reagents) {
            if (r.itemId == 0 || r.count == 0) continue;
            row.canMake = std::min(row.canMake,
                static_cast<int>(countItemInBags(r.itemId) / r.count));
        }
        out.push_back(std::move(row));
    }
    std::sort(out.begin(), out.end(),
              [](const CraftRecipe& a, const CraftRecipe& b) {
                  return a.name < b.name;
              });
    return out;
}

const std::string& GameHandler::getMacroName(uint32_t macroId) const {
    auto it = macroNames_.find(macroId);
    return (it != macroNames_.end()) ? it->second : EMPTY_MACRO_TEXT;
}

const std::string& GameHandler::getMacroIcon(uint32_t macroId) const {
    auto it = macroIcons_.find(macroId);
    return (it != macroIcons_.end()) ? it->second : EMPTY_MACRO_TEXT;
}

void GameHandler::setMacroMeta(uint32_t macroId, const std::string& name,
                               const std::string& icon) {
    if (name.empty()) macroNames_.erase(macroId); else macroNames_[macroId] = name;
    if (icon.empty()) macroIcons_.erase(macroId); else macroIcons_[macroId] = icon;
    saveCharacterConfig();
}

std::vector<uint32_t> GameHandler::getMacroIds() const {
    std::vector<uint32_t> ids;
    ids.reserve(macros_.size());
    for (const auto& [id, text] : macros_) { (void)text; ids.push_back(id); }
    // Ascending, because the interface indexes macros by position in this list
    // and an unordered_map hands them back in whatever order it likes - the
    // same macro would answer to a different index each session.
    std::sort(ids.begin(), ids.end());
    return ids;
}

void GameHandler::setMacroText(uint32_t macroId, const std::string& text) {
    if (text.empty())
        macros_.erase(macroId);
    else
        macros_[macroId] = text;
    saveCharacterConfig();
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
    // For male/female, derive from gender; only nonbinary has a meaningful separate choice
    bool saveUseFemaleModel = (ch->gender == Gender::NONBINARY) ? ch->useFemaleModel
                                                                 : (ch->gender == Gender::FEMALE);
    out << "use_female_model=" << (saveUseFemaleModel ? 1 : 0) << "\n";
    for (int i = 0; i < ACTION_BAR_SLOTS; i++) {
        out << "action_bar_" << i << "_type=" << static_cast<int>(actionBar[i].type) << "\n";
        out << "action_bar_" << i << "_id=" << actionBar[i].id << "\n";
    }

    // Save client-side macro text (escape newlines as \n literal)
    for (const auto& [id, text] : macros_) {
        if (!text.empty()) {
            std::string escaped;
            escaped.reserve(text.size());
            for (char c : text) {
                if (c == '\n') { escaped += "\\n"; }
                else if (c == '\r') { /* skip CR */ }
                else if (c == '\\') { escaped += "\\\\"; }
                else { escaped += c; }
            }
            out << "macro_" << id << "_text=" << escaped << "\n";
        }
    }
    for (const auto& [id, name] : macroNames_) {
        if (!name.empty()) out << "macro_" << id << "_name=" << name << "\n";
    }
    for (const auto& [id, icon] : macroIcons_) {
        if (!icon.empty()) out << "macro_" << id << "_icon=" << icon << "\n";
    }

    // The quest log itself is not saved: the server sends it on every login,
    // so a stored copy could only ever be staler than what arrives. What is
    // saved below is the tracker selection, which is a client-side choice the
    // server knows nothing about.

    // Save tracked quest IDs so the quest tracker restores on login
    if (!trackedQuestIds_.empty()) {
        std::string ids;
        for (uint32_t qid : trackedQuestIds_) {
            if (!ids.empty()) ids += ',';
            ids += std::to_string(qid);
        }
        out << "tracked_quests=" << ids << "\n";
    }

    // Which reputation headers are closed. The server has no opinion about it,
    // so this file is the only place it can survive a logout.
    if (!collapsedFactionIds_.empty()) {
        std::string ids;
        for (uint32_t fid : collapsedFactionIds_) {
            if (!ids.empty()) ids += ',';
            ids += std::to_string(fid);
        }
        out << "collapsed_factions=" << ids << "\n";
    }
    if (!collapsedSkillCategories_.empty()) {
        std::string ids;
        for (uint32_t cid : collapsedSkillCategories_) {
            if (!ids.empty()) ids += ',';
            ids += std::to_string(cid);
        }
        out << "collapsed_skill_categories=" << ids << "\n";
    }

    LOG_INFO("Character config saved to ", path);
}

void GameHandler::loadCharacterConfig() {
    const Character* ch = getActiveCharacter();
    if (!ch || ch->name.empty()) return;

    // These selections are per-character. Clear the previous character's
    // values even when the new character has no saved config yet.
    trackedQuestIds_.clear();

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
        } else if (key.rfind("macro_", 0) == 0) {
            // Parse macro_N_text
            size_t firstUnder = 6; // length of "macro_"
            size_t secondUnder = key.find('_', firstUnder);
            if (secondUnder == std::string::npos) continue;
            uint32_t macroId = 0;
            try { macroId = static_cast<uint32_t>(std::stoul(key.substr(firstUnder, secondUnder - firstUnder))); } catch (...) { continue; }
            if (key.substr(secondUnder + 1) == "text" && !val.empty()) {
                // Unescape \n and \\ sequences
                std::string unescaped;
                unescaped.reserve(val.size());
                for (size_t i = 0; i < val.size(); ++i) {
                    if (val[i] == '\\' && i + 1 < val.size()) {
                        if (val[i+1] == 'n')  { unescaped += '\n'; ++i; }
                        else if (val[i+1] == '\\') { unescaped += '\\'; ++i; }
                        else { unescaped += val[i]; }
                    } else {
                        unescaped += val[i];
                    }
                }
                macros_[macroId] = std::move(unescaped);
            } else if (key.substr(secondUnder + 1) == "name" && !val.empty()) {
                macroNames_[macroId] = val;
            } else if (key.substr(secondUnder + 1) == "icon" && !val.empty()) {
                macroIcons_[macroId] = val;
            }
        } else if ((key == "tracked_quests" || key == "collapsed_factions" ||
                    key == "collapsed_skill_categories") && !val.empty()) {
            // Parse a comma-separated id list into the matching selection.
            // map_visible_quests was a fourth of these and is no longer read:
            // the quests whose objectives the maps draw are the ones in the
            // log and the ones being tracked. An old file still carrying the
            // key falls through to the unknown-key branch.
            auto& destination =
                  key == "tracked_quests"    ? trackedQuestIds_
                : key == "collapsed_factions" ? collapsedFactionIds_
                                              : collapsedSkillCategories_;
            if (key == "collapsed_factions") reputationRowsDirty_ = true;
            destination.clear();
            size_t tqPos = 0;
            while (tqPos <= val.size()) {
                size_t comma = val.find(',', tqPos);
                std::string idStr = (comma != std::string::npos)
                    ? val.substr(tqPos, comma - tqPos) : val.substr(tqPos);
                try {
                    uint32_t qid = static_cast<uint32_t>(std::stoul(idStr));
                    if (qid != 0) destination.insert(qid);
                } catch (...) {}
                if (comma == std::string::npos) break;
                tqPos = comma + 1;
            }
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
                if (character.gender == Gender::NONBINARY) {
                    // Only nonbinary characters have a meaningful body type choice
                    if (savedUseFemaleModel >= 0) {
                        character.useFemaleModel = (savedUseFemaleModel != 0);
                    }
                } else {
                    // Male/female always use the model matching their gender
                    character.useFemaleModel = (character.gender == Gender::FEMALE);
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

void GameHandler::setTransportAttachment(uint64_t childGuid, ObjectType type, uint64_t transportGuid,
                                         const glm::vec3& localOffset, bool hasLocalOrientation,
                                         float localOrientation,
                                         bool offsetNeedsTransportResolution) {
    if (movementHandler_) movementHandler_->setTransportAttachment(
        childGuid, type, transportGuid, localOffset, hasLocalOrientation,
        localOrientation, offsetNeedsTransportResolution);
}

void GameHandler::clearTransportAttachment(uint64_t childGuid) {
    if (movementHandler_) movementHandler_->clearTransportAttachment(childGuid);
}

void GameHandler::updateAttachedTransportChildren(float deltaTime) {
    if (movementHandler_) movementHandler_->updateAttachedTransportChildren(deltaTime);
}

// ============================================================
// Mail System
// ============================================================

void GameHandler::openMailbox(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->openMailbox(guid);
}

void GameHandler::closeMailbox() {
    if (inventoryHandler_) inventoryHandler_->closeMailbox();
}

void GameHandler::refreshMailList() {
    if (inventoryHandler_) inventoryHandler_->refreshMailList();
}

void GameHandler::sendMail(const std::string& recipient, const std::string& subject,
                           const std::string& body, uint64_t money, uint64_t cod) {
    if (inventoryHandler_) inventoryHandler_->sendMail(recipient, subject, body, money, cod);
}

bool GameHandler::attachItemFromBackpack(int backpackIndex) {
    return inventoryHandler_ && inventoryHandler_->attachItemFromBackpack(backpackIndex);
}

bool GameHandler::attachItemFromBag(int bagIndex, int slotIndex) {
    return inventoryHandler_ && inventoryHandler_->attachItemFromBag(bagIndex, slotIndex);
}

bool GameHandler::detachMailAttachment(int attachIndex) {
    return inventoryHandler_ && inventoryHandler_->detachMailAttachment(attachIndex);
}

void GameHandler::clearMailAttachments() {
    if (inventoryHandler_) inventoryHandler_->clearMailAttachments();
}

int GameHandler::getMailAttachmentCount() const {
    if (inventoryHandler_) return inventoryHandler_->getMailAttachmentCount();
    return 0;
}

void GameHandler::mailTakeMoney(uint32_t mailId) {
    if (inventoryHandler_) inventoryHandler_->mailTakeMoney(mailId);
}

void GameHandler::mailTakeItem(uint32_t mailId, uint32_t itemGuidLow) {
    if (inventoryHandler_) inventoryHandler_->mailTakeItem(mailId, itemGuidLow);
}

void GameHandler::mailDelete(uint32_t mailId) {
    if (inventoryHandler_) inventoryHandler_->mailDelete(mailId);
}

void GameHandler::mailReturnToSender(uint32_t mailId) {
    if (inventoryHandler_) inventoryHandler_->mailReturnToSender(mailId);
}

void GameHandler::mailMarkAsRead(uint32_t mailId) {
    if (inventoryHandler_) inventoryHandler_->mailMarkAsRead(mailId);
}

glm::vec3 GameHandler::getComposedWorldPosition() {
    if (playerTransportGuid_ != 0 && transportManager_) {
        auto* tr = transportManager_->getTransport(playerTransportGuid_);
        if (tr) {
            return transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
        }
        // Transport not tracked - fall through to normal position
    }
    // Not on transport, return normal movement position
    return glm::vec3(movementInfo.x, movementInfo.y, movementInfo.z);
}

void GameHandler::beginPlayerTransportWorldTransfer(
    uint32_t destinationMapId, const glm::vec3& localOffset) {
    if (playerTransportGuid_ == 0) return;

    pendingPlayerTransportTransfer_ = true;
    pendingPlayerTransportGuid_ = playerTransportGuid_;
    pendingPlayerTransportMapId_ = destinationMapId;
    pendingPlayerTransportOffset_ = localOffset;
    pendingPlayerTransportEntry_ = 0;
    if (transportManager_) {
        if (const auto* transport = transportManager_->getTransport(playerTransportGuid_)) {
            pendingPlayerTransportEntry_ = transport->entry;
        }
    }

    LOG_WARNING("Transport world transfer armed: guid=0x", std::hex,
                pendingPlayerTransportGuid_, std::dec,
                " entry=", pendingPlayerTransportEntry_,
                " destinationMap=", destinationMapId,
                " local=(", localOffset.x, ",", localOffset.y, ",", localOffset.z, ")");
}

bool GameHandler::completePlayerTransportWorldTransfer(
    uint64_t transportGuid, glm::vec3& worldPosition) {
    if (!pendingPlayerTransportTransfer_ || !transportManager_ ||
        currentMapId_ != pendingPlayerTransportMapId_) {
        return false;
    }

    auto* transport = transportManager_->getTransport(transportGuid);
    if (!transport) return false;
    const bool matchingTransport = transportGuid == pendingPlayerTransportGuid_ ||
        (pendingPlayerTransportEntry_ != 0 &&
         transport->entry == pendingPlayerTransportEntry_);
    if (!matchingTransport) return false;

    setPlayerOnTransport(transportGuid, pendingPlayerTransportOffset_);
    worldPosition = transportManager_->getPlayerWorldPosition(
        transportGuid, pendingPlayerTransportOffset_);
    setPosition(worldPosition.x, worldPosition.y, worldPosition.z);

    LOG_WARNING("Transport world transfer restored: guid=0x", std::hex,
                transportGuid, std::dec,
                " entry=", transport->entry,
                " map=", currentMapId_,
                " local=(", pendingPlayerTransportOffset_.x, ",",
                pendingPlayerTransportOffset_.y, ",",
                pendingPlayerTransportOffset_.z, ") world=(",
                worldPosition.x, ",", worldPosition.y, ",", worldPosition.z, ")");

    pendingPlayerTransportTransfer_ = false;
    pendingPlayerTransportGuid_ = 0;
    pendingPlayerTransportEntry_ = 0;
    pendingPlayerTransportMapId_ = 0xFFFFFFFFu;
    return true;
}

// Client-side transport boarding/disembark detection for locally animated transports.
// This includes M2 trams/lifts and TaxiPathNode WMO ships: once the client owns a
// ship's route, the server's static spawn cannot provide timely attachment data.
// Shared between the GUI Application's
// per-frame update loop and any other driver (e.g. a headless harness) that knows the
// player's current canonical world position but has no renderer/camera of its own.
void GameHandler::updateM2TransportBoarding(const glm::vec3& playerCanonical) {
    auto* tm = getTransportManager();
    if (!tm) return;

    if (!isOnTransport()) {
        // Thunder Bluff elevators use model origins that can be far from the deck
        // the player stands on, so they need wider attachment bounds.
        constexpr float kM2BoardHorizDistSq = 12.0f * 12.0f;
        constexpr float kM2BoardVertDist = 15.0f;
        constexpr float kTbLiftBoardHorizDistSq = 22.0f * 22.0f;
        constexpr float kTbLiftBoardVertDist = 14.0f;
        // Deeprun's capture radius has swung twice on the horizontal axis already:
        // 18/18 (too wide - boarding triggered far enough away that the rider ended up
        // floating off to the side) down to 7/6 (too tight - stopped attaching at all).
        // 9.5 horizontal turned out fine - live diagnostic data (logged every second
        // while no candidate is in range) showed horizDist getting down to ~1-2 units
        // repeatedly, well inside the box. But boarding *still* never fired, because the
        // vertical threshold was the real problem all along: vertDist sat consistently
        // around 10.4-11.9 units at every one of those close-horizontal-approach samples,
        // across all three different cars tested, far outside the old 8.0 vertical
        // window. That's not noise - it's SubwayCar.m2's model origin sitting well below
        // the deck the player actually stands on (same root cause already called out for
        // Thunder Bluff lifts above: kTbLiftBoardVertDist=14 exists for exactly this
        // reason). Widen vertical to comfortably clear the observed ~11.9 max.
        constexpr float kDeeprunTramBoardVertDist = 13.0f;
        // An isotropic horizontal radius from the car's center can't tell "on this car"
        // from "on the neighboring car" (adjacent cars are only ~20 units apart along the
        // track - a 9.5 radius from each center overlaps its neighbor) or from "still
        // standing on the platform, several units to the side" (the platform-to-deck
        // vertical gap is nearly identical on the platform as on the actual car, so vert
        // distance alone can't disambiguate either). Live logs caught both: walking along
        // the platform re-boarded the adjacent car in the same train, and walking across
        // one car's width boarded on one side then rode with a lateral offset of ~9.5 -
        // effectively standing on empty platform on the *other* side of the car body
        // (SubwayCar.m2's real half-width from its own mesh is only 5.68). Test against
        // the car's actual oriented footprint instead: transform the player into the
        // car's local model space (same invTransform already computed every frame for
        // collision) and bound against the real mesh extents (X ~9.18 half-length along
        // the direction of travel, Y ~5.68 half-width across it), not a radius from a
        // point. Small margin added on top for the "about to step across from the
        // platform" approach.
        constexpr float kDeeprunTramBoardHalfLength = 9.18f + 1.5f;  // along direction of travel
        constexpr float kDeeprunTramBoardHalfWidth = 5.68f + 1.5f;   // across the car's width
        constexpr float kShipBoardHalfLength = 65.0f;
        constexpr float kShipBoardHalfWidth = 30.0f;
        constexpr float kShipBoardMinZ = 1.0f;
        constexpr float kShipBoardMaxZ = 35.0f;

        uint64_t bestGuid = 0;
        float bestScore = 1e30f;
        // Tracks the closest Deeprun tram car even when it's outside the capture
        // radius, purely so a rejection can be logged with real distance numbers -
        // the capture radius has been re-tuned blind twice already (18/18, then 7/6)
        // from user reports alone with no actual miss-distance data to tune against.
        uint64_t nearestDeeprunGuid = 0;
        float nearestDeeprunLocalX = 0.0f;
        float nearestDeeprunLocalY = 0.0f;
        float nearestDeeprunLocalDistSq = 1e30f;
        float nearestDeeprunVertDist = 0.0f;
        const glm::vec3 playerRenderPos = core::coords::canonicalToRender(playerCanonical);
        for (auto& [guid, transport] : tm->getTransports()) {
            const bool isClientShip = !transport.isM2 && transport.worldCoords &&
                                      transport.useClientAnimation;
            if (!transport.isM2 && !isClientShip) continue;
            const bool isThunderBluffLift =
                (transport.entry >= 20649u && transport.entry <= 20657u);
            const bool isDeeprunTram =
                TransportManager::isDeeprunTramTransport(transport);
            glm::vec3 diff = playerCanonical - transport.position;
            float vertDist = std::abs(diff.z);

            if (isClientShip) {
                const glm::vec3 local(transport.invTransform * glm::vec4(playerRenderPos, 1.0f));
                if (std::abs(local.x) < kShipBoardHalfLength &&
                    std::abs(local.y) < kShipBoardHalfWidth &&
                    local.z > kShipBoardMinZ && local.z < kShipBoardMaxZ &&
                    tm->isPointOnTransportDeck(guid, playerCanonical)) {
                    const float score = local.x * local.x + local.y * local.y;
                    if (score < bestScore) {
                        bestScore = score;
                        bestGuid = guid;
                    }
                }
                continue;
            }

            if (isDeeprunTram) {
                const glm::vec4 local4 = transport.invTransform * glm::vec4(playerRenderPos, 1.0f);
                const glm::vec3 local(local4);
                const float localDistSq = local.x * local.x + local.y * local.y;
                if (localDistSq < nearestDeeprunLocalDistSq) {
                    nearestDeeprunLocalDistSq = localDistSq;
                    nearestDeeprunLocalX = local.x;
                    nearestDeeprunLocalY = local.y;
                    nearestDeeprunVertDist = vertDist;
                    nearestDeeprunGuid = guid;
                }
                if (std::abs(local.x) < kDeeprunTramBoardHalfLength &&
                    std::abs(local.y) < kDeeprunTramBoardHalfWidth &&
                    vertDist < kDeeprunTramBoardVertDist) {
                    const float score = localDistSq + vertDist * vertDist;
                    if (score < bestScore) {
                        bestScore = score;
                        bestGuid = guid;
                    }
                }
                continue;
            }

            const float maxHorizDistSq = isThunderBluffLift
                ? kTbLiftBoardHorizDistSq
                : kM2BoardHorizDistSq;
            const float maxVertDist = isThunderBluffLift
                ? kTbLiftBoardVertDist
                : kM2BoardVertDist;
            float horizDistSq = diff.x * diff.x + diff.y * diff.y;
            if (horizDistSq < maxHorizDistSq && vertDist < maxVertDist) {
                float score = horizDistSq + vertDist * vertDist;
                if (score < bestScore) {
                    bestScore = score;
                    bestGuid = guid;
                }
            }
        }
        if (bestGuid == 0 && nearestDeeprunGuid != 0) {
            // Boarding/disembark are now on a well-tested oriented-footprint check
            // (verified live) rather than the blind-tuned isotropic radius this was
            // originally added to debug. Routine while just walking around Deeprun not
            // boarded - demoted to DEBUG. Still throttled to ~1/sec since this runs
            // every frame while not boarded.
            static double lastLogTime = -1000.0;
            const double now = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count()) / 1000.0;
            if (now - lastLogTime >= 1.0) {
                lastLogTime = now;
                LOG_DEBUG("Deeprun tram boarding: no candidate in range, nearest car guid=0x",
                            std::hex, nearestDeeprunGuid, std::dec,
                            " localX=", nearestDeeprunLocalX,
                            " localY=", nearestDeeprunLocalY,
                            " vertDist=", nearestDeeprunVertDist);
            }
        }
        if (bestGuid != 0) {
            auto* tr = tm->getTransport(bestGuid);
            if (tr) {
                const bool isDeeprunTram =
                    TransportManager::isDeeprunTramTransport(*tr);
                const glm::vec3 offset = tr->isM2
                    ? playerCanonical - tr->position
                    : glm::vec3(tr->invTransform * glm::vec4(playerRenderPos, 1.0f));
                setPlayerOnTransport(bestGuid, offset);
                if (isDeeprunTram) {
                    const bool attached = getPlayerTransportGuid() == bestGuid;
                    LOG_INFO("Deeprun tram boarding candidate ", (attached ? "accepted" : "rejected"),
                                ": guid=0x", std::hex, bestGuid, std::dec,
                                " entry=", tr->entry,
                                " pathId=", tr->pathId,
                                " player=(", playerCanonical.x, ",", playerCanonical.y, ",", playerCanonical.z, ")",
                                " tram=(", tr->position.x, ",", tr->position.y, ",", tr->position.z, ")",
                                " offset=(", offset.x, ",", offset.y, ",", offset.z, ")");
                } else if (!tr->isM2) {
                    LOG_WARNING("Client ship boarding accepted: guid=0x", std::hex, bestGuid, std::dec,
                                " entry=", tr->entry,
                                " local=(", offset.x, ",", offset.y, ",", offset.z, ")");
                } else {
                    LOG_DEBUG("M2 transport boarding: guid=0x", std::hex, bestGuid, std::dec);
                }
            }
        }
        return;
    }

    // M2 transport disembark: player walked far enough from transport center.
    auto* tr = tm->getTransport(getPlayerTransportGuid());
    if (!tr) return;
    glm::vec3 diff = playerCanonical - tr->position;
    float horizDistSq = diff.x * diff.x + diff.y * diff.y;

    // Sanity guard against a transient bad position sample rather than a real disembark.
    // Live data caught one: player=(-44.9,2309.03,..) vs tram=(2309.83,-45.4,..) - the
    // player's X and Y are each within ~1 unit of the tram's Y and X respectively (an
    // axis swap, not a real position), giving horizDist~3330 in a single frame despite
    // riding smoothly for the prior ~70 seconds. Nothing can legitimately move that far
    // between frames; whatever produced that sample (suspected: a raw server-coordinate
    // movement update landing without the usual serverToCanonical swap, since that
    // conversion has the exact same X/Y-swap shape as the corruption seen - possibly
    // tied to a packet glitch, a "MSG_MOVE_TELEPORT_ACK: not enough data for movement
    // info" was logged moments later in the same session) shouldn't be trusted enough to
    // eject the player over open track - "kicked me off when I almost got to the other
    // station" reported live. Skip disembark entirely this frame on an implausible jump;
    // it'll re-evaluate next frame against (presumably) good data instead.
    constexpr float kImplausibleDisembarkDistSq = 200.0f * 200.0f;
    if (horizDistSq > kImplausibleDisembarkDistSq) {
        LOG_WARNING("Deeprun tram disembark check skipped - implausible jump: guid=0x",
                    std::hex, getPlayerTransportGuid(), std::dec,
                    " horizDist=", std::sqrt(horizDistSq),
                    " player=(", playerCanonical.x, ",", playerCanonical.y, ",", playerCanonical.z, ")",
                    " tram=(", tr->position.x, ",", tr->position.y, ",", tr->position.z, ")");
        return;
    }

    const bool isThunderBluffLift =
        (tr->entry >= 20649u && tr->entry <= 20657u);
    const bool isDeeprunTram = TransportManager::isDeeprunTramTransport(*tr);

    if (!tr->isM2 && tr->worldCoords && tr->useClientAnimation) {
        constexpr float kShipDisembarkHalfLength = 70.0f;
        constexpr float kShipDisembarkHalfWidth = 35.0f;
        constexpr float kShipDisembarkMinZ = -3.0f;
        constexpr float kShipDisembarkMaxZ = 40.0f;
        const glm::vec3 playerRenderPos = core::coords::canonicalToRender(playerCanonical);
        const glm::vec3 local(tr->invTransform * glm::vec4(playerRenderPos, 1.0f));
        const bool outsideBounds =
            std::abs(local.x) > kShipDisembarkHalfLength ||
            std::abs(local.y) > kShipDisembarkHalfWidth ||
            local.z < kShipDisembarkMinZ || local.z > kShipDisembarkMaxZ;
        const bool hasDeckSupport =
            tm->isPointOnTransportDeck(getPlayerTransportGuid(), playerCanonical, 3.0f);

        // The footprint above is deliberately generous - larger than any hull -
        // so it only catches someone clearly away from the ship. Stepping off
        // onto the pier alongside leaves the rider well inside it, which is why
        // they stayed attached while standing on the dock. Losing the deck is
        // what actually tells the two apart, and it was already being measured
        // here and used for nothing but the log line.
        //
        // Counted over several frames rather than acted on at once, because a
        // jump also leaves the deck for a moment and must not put the rider
        // ashore in mid-air.
        constexpr int kFramesAshore = 20;
        if (hasDeckSupport) {
            shipNoDeckSupportFrames_ = 0;
        } else if (shipNoDeckSupportFrames_ < kFramesAshore) {
            ++shipNoDeckSupportFrames_;
        }
        const bool ashore = shipNoDeckSupportFrames_ >= kFramesAshore;

        if (outsideBounds || ashore) {
            LOG_WARNING("Client ship disembark: guid=0x", std::hex,
                        getPlayerTransportGuid(), std::dec,
                        " outsideBounds=", outsideBounds,
                        " ashore=", ashore,
                        " local=(", local.x, ",", local.y, ",", local.z, ")");
            shipNoDeckSupportFrames_ = 0;
            clearPlayerTransport();
        }
        return;
    }

    if (isDeeprunTram) {
        // Same oriented-footprint reasoning as the boarding scan above: an isotropic
        // radius from the car's center can't tell "still on the car" from "walked off
        // the far side onto open platform." Live data caught exactly that - a rider's
        // lateral offset swung from +9.5 to -9.5 (fully across and off SubwayCar.m2's
        // real 5.68-unit half-width) while still reading as "on board" the whole way,
        // because the old 18-unit isotropic radius didn't clear until well past the
        // real car body in every direction. Test the player's position in the car's own
        // local space instead, with a modest margin over the real mesh extents (X 9.18,
        // Y 5.68) so ordinary standing/shifting on the deck doesn't hair-trigger an
        // eject, without allowing a full walk-through before releasing.
        constexpr float kDeeprunTramDisembarkHalfLength = 9.18f + 3.0f;
        constexpr float kDeeprunTramDisembarkHalfWidth = 5.68f + 3.0f;
        constexpr float kDeeprunTramDisembarkVertDist = 22.0f;
        const glm::vec3 playerRenderPos = core::coords::canonicalToRender(playerCanonical);
        const glm::vec4 local4 = tr->invTransform * glm::vec4(playerRenderPos, 1.0f);
        const glm::vec3 local(local4);
        if (std::abs(local.x) > kDeeprunTramDisembarkHalfLength ||
            std::abs(local.y) > kDeeprunTramDisembarkHalfWidth ||
            std::abs(diff.z) > kDeeprunTramDisembarkVertDist) {
            LOG_INFO("Deeprun tram disembark: guid=0x", std::hex, getPlayerTransportGuid(), std::dec,
                        " localX=", local.x, " localY=", local.y, " vertDist=", std::abs(diff.z),
                        " player=(", playerCanonical.x, ",", playerCanonical.y, ",", playerCanonical.z, ")",
                        " tram=(", tr->position.x, ",", tr->position.y, ",", tr->position.z, ")");
            clearPlayerTransport();
            LOG_DEBUG("M2 transport disembark");
        }
        return;
    }

    constexpr float kM2DisembarkHorizDistSq = 15.0f * 15.0f;
    constexpr float kTbLiftDisembarkHorizDistSq = 28.0f * 28.0f;
    constexpr float kM2DisembarkVertDist = 18.0f;
    constexpr float kTbLiftDisembarkVertDist = 16.0f;
    const float disembarkHorizDistSq = isThunderBluffLift
        ? kTbLiftDisembarkHorizDistSq
        : kM2DisembarkHorizDistSq;
    const float disembarkVertDist = isThunderBluffLift
        ? kTbLiftDisembarkVertDist
        : kM2DisembarkVertDist;
    if (horizDistSq > disembarkHorizDistSq || std::abs(diff.z) > disembarkVertDist) {
        clearPlayerTransport();
        LOG_DEBUG("M2 transport disembark");
    }
}

// ============================================================
// Bank System
// ============================================================

void GameHandler::openBank(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->openBank(guid);
}

void GameHandler::closeBank() {
    if (inventoryHandler_) inventoryHandler_->closeBank();
}

void GameHandler::buyBankSlot() {
    if (inventoryHandler_) inventoryHandler_->buyBankSlot();
}

uint32_t GameHandler::getBankBagSlotPrice(int slotIndex) const {
    return InventoryHandler::getBankBagSlotPrice(slotIndex);
}

void GameHandler::depositItem(uint8_t srcBag, uint8_t srcSlot) {
    if (inventoryHandler_) inventoryHandler_->depositItem(srcBag, srcSlot);
}

void GameHandler::withdrawItem(uint8_t srcBag, uint8_t srcSlot) {
    if (inventoryHandler_) inventoryHandler_->withdrawItem(srcBag, srcSlot);
}

// ============================================================
// Guild Bank System
// ============================================================

void GameHandler::openGuildBank(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->openGuildBank(guid);
}

void GameHandler::closeGuildBank() {
    if (inventoryHandler_) inventoryHandler_->closeGuildBank();
}

void GameHandler::queryGuildBankTab(uint8_t tabId) {
    if (inventoryHandler_) inventoryHandler_->queryGuildBankTab(tabId);
}

void GameHandler::buyGuildBankTab() {
    if (inventoryHandler_) inventoryHandler_->buyGuildBankTab();
}

void GameHandler::depositGuildBankMoney(uint32_t amount) {
    if (inventoryHandler_) inventoryHandler_->depositGuildBankMoney(amount);
}

void GameHandler::withdrawGuildBankMoney(uint32_t amount) {
    if (inventoryHandler_) inventoryHandler_->withdrawGuildBankMoney(amount);
}

void GameHandler::guildBankWithdrawItem(uint8_t tabId, uint8_t bankSlot, uint8_t destBag,
                                        uint8_t destSlot, uint32_t splitCount) {
    if (inventoryHandler_) inventoryHandler_->guildBankWithdrawItem(tabId, bankSlot, destBag,
                                                                   destSlot, splitCount);
}

void GameHandler::guildBankDepositItem(uint8_t tabId, uint8_t bankSlot, uint8_t srcBag, uint8_t srcSlot) {
    if (inventoryHandler_) inventoryHandler_->guildBankDepositItem(tabId, bankSlot, srcBag, srcSlot);
}

void GameHandler::queryGuildBankText(uint8_t tabId) {
    if (inventoryHandler_) inventoryHandler_->queryGuildBankText(tabId);
}

const std::string& GameHandler::getGuildBankTabText(uint8_t tabId) const {
    static const std::string kNone;
    return inventoryHandler_ ? inventoryHandler_->getGuildBankTabText(tabId) : kNone;
}

void GameHandler::setGuildBankTabInfo(uint8_t tabId, const std::string& name,
                                      const std::string& icon) {
    if (inventoryHandler_) inventoryHandler_->setGuildBankTabInfo(tabId, name, icon);
}

void GameHandler::guildBankDepositFromInventory(uint8_t srcBag, uint8_t srcSlot) {
    if (inventoryHandler_) inventoryHandler_->guildBankDepositFromInventory(srcBag, srcSlot);
}

// ============================================================
// Auction House System
// ============================================================

void GameHandler::openAuctionHouse(uint64_t guid) {
    if (inventoryHandler_) inventoryHandler_->openAuctionHouse(guid);
}

void GameHandler::closeAuctionHouse() {
    if (inventoryHandler_) inventoryHandler_->closeAuctionHouse();
}

void GameHandler::auctionSearch(const std::string& name, uint8_t levelMin, uint8_t levelMax,
                                 uint32_t quality, uint32_t itemClass, uint32_t itemSubClass,
                                 uint32_t invTypeMask, uint8_t usableOnly, uint32_t offset,
                                 const std::vector<AuctionSortKey>& sort) {
    if (inventoryHandler_) inventoryHandler_->auctionSearch(name, levelMin, levelMax, quality, itemClass, itemSubClass, invTypeMask, usableOnly, offset, sort);
}

void GameHandler::auctionSellItem(int backpackIndex, uint32_t bid,
                                    uint32_t buyout, uint32_t duration) {
    if (inventoryHandler_) inventoryHandler_->auctionSellItem(backpackIndex, bid, buyout, duration);
}

void GameHandler::auctionSellItemByGuid(uint64_t itemGuid, uint32_t stackCount, uint32_t bid,
                                        uint32_t buyout, uint32_t duration) {
    if (inventoryHandler_) inventoryHandler_->auctionSellItemByGuid(itemGuid, stackCount, bid, buyout, duration);
}

void GameHandler::auctionPlaceBid(uint32_t auctionId, uint32_t amount) {
    if (inventoryHandler_) inventoryHandler_->auctionPlaceBid(auctionId, amount);
}

void GameHandler::auctionBuyout(uint32_t auctionId, uint32_t buyoutPrice) {
    if (inventoryHandler_) inventoryHandler_->auctionBuyout(auctionId, buyoutPrice);
}

void GameHandler::auctionCancelItem(uint32_t auctionId) {
    if (inventoryHandler_) inventoryHandler_->auctionCancelItem(auctionId);
}

void GameHandler::auctionListOwnerItems(uint32_t offset) {
    if (inventoryHandler_) inventoryHandler_->auctionListOwnerItems(offset);
}

void GameHandler::auctionListBidderItems(uint32_t offset) {
    if (inventoryHandler_) inventoryHandler_->auctionListBidderItems(offset);
}

// ---------------------------------------------------------------------------
// Item text (SMSG_ITEM_TEXT_QUERY_RESPONSE)
//   uint64 itemGuid + uint8 isEmpty + string text (when !isEmpty)
// ---------------------------------------------------------------------------

void GameHandler::queryItemText(uint64_t itemGuid) {
    if (inventoryHandler_) inventoryHandler_->queryItemText(itemGuid);
}

// ---------------------------------------------------------------------------
// SMSG_QUEST_CONFIRM_ACCEPT (shared quest from group member)
//   uint32 questId + string questTitle + uint64 sharerGuid
// ---------------------------------------------------------------------------

void GameHandler::acceptSharedQuest() {
    if (questHandler_) questHandler_->acceptSharedQuest();
}

void GameHandler::declineSharedQuest() {
    if (questHandler_) questHandler_->declineSharedQuest();
}

// ---------------------------------------------------------------------------
// SMSG_SUMMON_REQUEST
//   uint64 summonerGuid + uint32 zoneId + uint32 timeoutMs
// ---------------------------------------------------------------------------

void GameHandler::handleSummonRequest(network::Packet& packet) {
    if (socialHandler_) socialHandler_->handleSummonRequest(packet);
}

void GameHandler::acceptSummon() {
    if (socialHandler_) socialHandler_->acceptSummon();
}

void GameHandler::declineSummon() {
    if (socialHandler_) socialHandler_->declineSummon();
}

// ---------------------------------------------------------------------------
// Trade (SMSG_TRADE_STATUS / SMSG_TRADE_STATUS_EXTENDED)
// WotLK 3.3.5a status values:
//   0=busy, 1=begin_trade(+guid), 2=open_window, 3=cancelled, 4=accepted,
//   5=busy2, 6=no_target, 7=back_to_trade, 8=complete, 9=rejected,
//   10=too_far, 11=wrong_faction, 12=close_window, 13=ignore,
//   14-19=stun/dead/logout, 20=trial, 21=conjured_only
// ---------------------------------------------------------------------------

void GameHandler::acceptTradeRequest() {
    if (inventoryHandler_) inventoryHandler_->acceptTradeRequest();
}

void GameHandler::declineTradeRequest() {
    if (inventoryHandler_) inventoryHandler_->declineTradeRequest();
}

void GameHandler::acceptTrade() {
    if (inventoryHandler_) inventoryHandler_->acceptTrade();
}

void GameHandler::unacceptTrade() {
    if (inventoryHandler_) inventoryHandler_->unacceptTrade();
}

void GameHandler::cancelTrade() {
    if (inventoryHandler_) inventoryHandler_->cancelTrade();
}

void GameHandler::setTradeItem(uint8_t tradeSlot, uint8_t bag, uint8_t bagSlot) {
    if (inventoryHandler_) inventoryHandler_->setTradeItem(tradeSlot, bag, bagSlot);
}

void GameHandler::clearTradeItem(uint8_t tradeSlot) {
    if (inventoryHandler_) inventoryHandler_->clearTradeItem(tradeSlot);
}

void GameHandler::setTradeGold(uint64_t copper) {
    if (inventoryHandler_) inventoryHandler_->setTradeGold(copper);
}

void GameHandler::resetTradeState() {
    if (inventoryHandler_) inventoryHandler_->resetTradeState();
}

// ---------------------------------------------------------------------------
// Group loot roll (SMSG_LOOT_ROLL / SMSG_LOOT_ROLL_WON / CMSG_LOOT_ROLL)
// ---------------------------------------------------------------------------

void GameHandler::sendLootRoll(uint64_t objectGuid, uint32_t slot, uint8_t rollType) {
    if (inventoryHandler_) inventoryHandler_->sendLootRoll(objectGuid, slot, rollType);
}

// ---------------------------------------------------------------------------
// SMSG_ACHIEVEMENT_EARNED (WotLK 3.3.5a wire 0x468)
//   uint64 guid          - player who earned it (may be another player)
//   uint32 achievementId - Achievement.dbc ID
//   PackedTime date      - uint32 bitfield (seconds since epoch)
//   uint32 realmFirst    - how many on realm also got it (0 = realm first)
// ---------------------------------------------------------------------------
void GameHandler::loadTitleNameCache() const {
    if (titleNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    titleNameCacheLoaded_ = true;

    auto dbc = am->loadDBC("CharTitles.dbc");
    if (!dbc || !dbc->isLoaded() || dbc->getFieldCount() < 5) return;

    const auto* layout = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("CharTitles") : nullptr;

    uint32_t titleField = layout ? layout->field("Title")    : 2;
    uint32_t bitField   = layout ? layout->field("TitleBit") : 36;
    if (titleField == 0xFFFFFFFF) titleField = 2;
    if (bitField   == 0xFFFFFFFF) bitField   = static_cast<uint32_t>(dbc->getFieldCount() - 1);

    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        const uint32_t id = dbc->getUInt32(i, 0);   // the row id a quest reward references
        std::string name = dbc->getString(i, titleField);
        if (name.empty()) continue;
        // By id for a quest reward (which carries the id), and by mask bit for
        // the worn title (which is a bit off the player's own fields).
        if (id != 0) titleFormatById_[id] = name;
        uint32_t bit = dbc->getUInt32(i, bitField);
        if (bit != 0) titleNameCache_[bit] = std::move(name);
    }
    LOG_INFO("CharTitles: loaded ", titleNameCache_.size(), " title names from DBC");
}

std::string GameHandler::getFormattedTitle(uint32_t bit) const {
    loadTitleNameCache();
    auto it = titleNameCache_.find(bit);
    if (it == titleNameCache_.end() || it->second.empty()) return {};
    return formatTitleString(it->second);
}

std::string GameHandler::getFormattedTitleById(uint32_t id) const {
    loadTitleNameCache();
    auto it = titleFormatById_.find(id);
    if (it == titleFormatById_.end() || it->second.empty()) return {};
    return formatTitleString(it->second);
}

float GameHandler::critPercentFromGameTable(std::vector<float>& baseCache,
                                            std::vector<float>& ratioCache, bool& loaded,
                                            const char* baseDbc, const char* ratioDbc,
                                            int statIdx) const {
    const uint8_t pclass = getPlayerClass();
    uint32_t level = getPlayerLevel();
    if (pclass == 0 || pclass > 11 || level == 0) return 0.0f;
    constexpr uint32_t kGtMaxLevel = 100;
    if (level > kGtMaxLevel) level = kGtMaxLevel;
    if (!loaded) {
        loaded = true;
        auto* am = services_.assetManager;
        if (am && am->isInitialized()) {
            // Single-column float game tables; getFloat reads that one column.
            if (auto base = am->loadDBC(baseDbc); base && base->isLoaded())
                for (uint32_t i = 0; i < base->getRecordCount(); ++i)
                    baseCache.push_back(base->getFloat(i, 0));
            if (auto ratio = am->loadDBC(ratioDbc); ratio && ratio->isLoaded())
                for (uint32_t i = 0; i < ratio->getRecordCount(); ++i)
                    ratioCache.push_back(ratio->getFloat(i, 0));
        }
    }
    const size_t baseIdx  = static_cast<size_t>(pclass - 1);
    const size_t ratioIdx = static_cast<size_t>(pclass - 1) * kGtMaxLevel + (level - 1);
    if (baseIdx >= baseCache.size() || ratioIdx >= ratioCache.size()) return 0.0f;
    const float stat = static_cast<float>(std::max(0, getPlayerStat(statIdx)));
    return (baseCache[baseIdx] + stat * ratioCache[ratioIdx]) * 100.0f;  // the sheet reads a percent
}

float GameHandler::getMeleeCritFromAgility() const {
    return critPercentFromGameTable(gtMeleeCritBase_, gtMeleeCrit_, gtMeleeCritLoaded_,
                                    "gtChanceToMeleeCritBase.dbc", "gtChanceToMeleeCrit.dbc",
                                    1 /* STAT_AGILITY */);
}

float GameHandler::getSpellCritFromIntellect() const {
    return critPercentFromGameTable(gtSpellCritBase_, gtSpellCrit_, gtSpellCritLoaded_,
                                    "gtChanceToSpellCritBase.dbc", "gtChanceToSpellCrit.dbc",
                                    3 /* STAT_INTELLECT */);
}

namespace {
// A single-column float game table into a flat vector.
void loadFloatColumn(pipeline::AssetManager* am, const char* dbc, std::vector<float>& out) {
    if (auto t = am->loadDBC(dbc); t && t->isLoaded())
        for (uint32_t i = 0; i < t->getRecordCount(); ++i) out.push_back(t->getFloat(i, 0));
}
}  // namespace

float GameHandler::getHealthRegenFromSpirit() const {
    const uint8_t pclass = getPlayerClass();
    uint32_t level = getPlayerLevel();
    if (pclass == 0 || pclass > 11 || level == 0) return 0.0f;
    constexpr uint32_t kGtMaxLevel = 100;
    if (level > kGtMaxLevel) level = kGtMaxLevel;
    if (!gtRegenLoaded_) {
        gtRegenLoaded_ = true;
        if (auto* am = services_.assetManager; am && am->isInitialized()) {
            loadFloatColumn(am, "gtOCTRegenHP.dbc", gtOctRegenHp_);
            loadFloatColumn(am, "gtRegenHPPerSpt.dbc", gtRegenHpPerSpt_);
            loadFloatColumn(am, "gtRegenMPPerSpt.dbc", gtRegenMpPerSpt_);
        }
    }
    const size_t idx = static_cast<size_t>(pclass - 1) * kGtMaxLevel + (level - 1);
    if (idx >= gtOctRegenHp_.size() || idx >= gtRegenHpPerSpt_.size()) return 0.0f;
    const float spirit = static_cast<float>(std::max(0, getPlayerStat(4 /* SPIRIT */)));
    const float baseSpirit = std::min(spirit, 50.0f);
    const float moreSpirit = spirit - baseSpirit;
    return (baseSpirit * gtOctRegenHp_[idx] + moreSpirit * gtRegenHpPerSpt_[idx]) * 2.0f;
}

float GameHandler::getManaRegenFromSpirit() const {
    const uint8_t pclass = getPlayerClass();
    uint32_t level = getPlayerLevel();
    if (pclass == 0 || pclass > 11 || level == 0) return 0.0f;
    constexpr uint32_t kGtMaxLevel = 100;
    if (level > kGtMaxLevel) level = kGtMaxLevel;
    if (!gtRegenLoaded_) { getHealthRegenFromSpirit(); }  // shares the same lazy load
    const size_t idx = static_cast<size_t>(pclass - 1) * kGtMaxLevel + (level - 1);
    if (idx >= gtRegenMpPerSpt_.size()) return 0.0f;
    const float spirit = static_cast<float>(std::max(0, getPlayerStat(4 /* SPIRIT */)));
    return spirit * gtRegenMpPerSpt_[idx];
}

float GameHandler::getCombatRatingBonus(int cr) const {
    const uint8_t pclass = getPlayerClass();
    uint32_t level = getPlayerLevel();
    constexpr uint32_t kGtMaxLevel = 100, kGtMaxRating = 32;
    if (pclass == 0 || pclass > 11 || level == 0 || cr < 0 ||
        cr >= static_cast<int>(kGtMaxRating)) return 0.0f;
    if (level > kGtMaxLevel) level = kGtMaxLevel;
    if (!gtCombatRatingsLoaded_) {
        gtCombatRatingsLoaded_ = true;
        auto* am = services_.assetManager;
        if (am && am->isInitialized()) {
            if (auto t = am->loadDBC("gtCombatRatings.dbc"); t && t->isLoaded())
                for (uint32_t i = 0; i < t->getRecordCount(); ++i)
                    gtCombatRatings_.push_back(t->getFloat(i, 0));
            // The scalar table is id + ratio; the ratio is the second column.
            if (auto s = am->loadDBC("gtOCTClassCombatRatingScalar.dbc"); s && s->isLoaded())
                for (uint32_t i = 0; i < s->getRecordCount(); ++i)
                    gtClassRatingScalar_.push_back(s->getFloat(i, 1));
        }
    }
    const size_t coefIdx   = static_cast<size_t>(cr) * kGtMaxLevel + (level - 1);
    const size_t scalarIdx = static_cast<size_t>(pclass - 1) * kGtMaxRating + cr;
    if (coefIdx >= gtCombatRatings_.size() || scalarIdx >= gtClassRatingScalar_.size()) return 0.0f;
    const float coef = gtCombatRatings_[coefIdx];
    if (coef == 0.0f) return 0.0f;
    const float ratingValue = static_cast<float>(std::max(0, getCombatRating(cr)));
    return ratingValue * (gtClassRatingScalar_[scalarIdx] / coef);
}

std::string GameHandler::formatTitleString(const std::string& fmt) const {
    const auto& ln2 = lookupName(playerGuid);
    static const std::string kUnknown = "unknown";
    return formatTitleStringFor(fmt, ln2.empty() ? kUnknown : ln2);
}

std::string GameHandler::formatTitleStringFor(const std::string& fmt,
                                              const std::string& name) const {
    // A title is a sentence with a hole in it - "%s the Explorer", "Sergeant
    // %s" - so the name goes where the row says and not before it.
    size_t pos = fmt.find("%s");
    if (pos != std::string::npos) {
        return fmt.substr(0, pos) + name + fmt.substr(pos + 2);
    }
    return fmt;
}

std::string GameHandler::getFormattedTitleFor(uint32_t bit,
                                              const std::string& name) const {
    loadTitleNameCache();
    auto it = titleNameCache_.find(bit);
    if (it == titleNameCache_.end() || it->second.empty()) return {};
    return formatTitleStringFor(it->second, name);
}

void GameHandler::sendSetTitle(int32_t bit) {
    if (!isInWorld()) return;
    auto packet = SetTitlePacket::build(bit);
    socket->send(packet);
    chosenTitleBit_ = bit;
    LOG_INFO("sendSetTitle: bit=", bit);
}

void GameHandler::loadAchievementNameCache() {
    if (achievementNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    achievementNameCacheLoaded_ = true;

    auto dbc = am->loadDBC("Achievement.dbc");
    if (!dbc || !dbc->isLoaded() || dbc->getFieldCount() < 22) return;

    const auto* achL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("Achievement") : nullptr;
    uint32_t titleField = achL ? achL->field("Title") : 4;
    if (titleField == 0xFFFFFFFF) titleField = 4;
    uint32_t descField = achL ? achL->field("Description") : 0xFFFFFFFF;
    uint32_t ptsField  = achL ? achL->field("Points")      : 0xFFFFFFFF;
    // IconID references SpellIcon.dbc for the achievement's artwork. Field 42 in the
    // stock 3.3.5a Achievement.dbc when the layout doesn't name it.
    uint32_t iconField = achL ? achL->field("IconID") : 0xFFFFFFFF;
    if (iconField == 0xFFFFFFFF && dbc->getFieldCount() > 42) iconField = 42;

    uint32_t fieldCount = dbc->getFieldCount();
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        uint32_t id = dbc->getUInt32(i, 0);
        if (id == 0) continue;
        std::string title = dbc->getString(i, titleField);
        if (!title.empty()) achievementNameCache_[id] = std::move(title);
        if (descField != 0xFFFFFFFF && descField < fieldCount) {
            std::string desc = dbc->getString(i, descField);
            if (!desc.empty()) achievementDescCache_[id] = std::move(desc);
        }
        if (ptsField != 0xFFFFFFFF && ptsField < fieldCount) {
            uint32_t pts = dbc->getUInt32(i, ptsField);
            if (pts > 0) achievementPointsCache_[id] = pts;
        }
        if (iconField != 0xFFFFFFFF && iconField < fieldCount) {
            uint32_t iconId = dbc->getUInt32(i, iconField);
            if (iconId > 0) achievementIconCache_[id] = iconId;
        }
        // Field 41 is Flags. Read here rather than in the category loader,
        // which used to read it separately for the statistic bit alone - one
        // pass over the file, one idea of what the column is. Confirmed twice
        // over: bit 0x1 agrees with "does this row's category descend from
        // Statistics" on all 1817 rows, and FrameXML's own constants.lua names
        // that same bit ACHIEVEMENT_FLAGS_STATISTIC.
        if (fieldCount > 41) {
            uint32_t flags = dbc->getUInt32(i, 41);
            if (flags != 0) achievementFlagsCache_[id] = flags;
        }
    }
    LOG_INFO("Achievement: loaded ", achievementNameCache_.size(), " names from Achievement.dbc");
}

const GameHandler::BattlemasterEntry* GameHandler::getBattlemasterInfo(uint32_t bgTypeId) {
    if (!battlemasterListLoaded_) {
        auto* am = services_.assetManager;
        // Checked before the latch, as with the other DBC caches here.
        if (!am || !am->isInitialized()) return nullptr;
        battlemasterListLoaded_ = true;
        auto dbc = am->loadDBC("BattlemasterList.dbc");
        if (dbc && dbc->isLoaded() && dbc->getFieldCount() > 31) {
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                const uint32_t id = dbc->getUInt32(i, 0);
                if (id == 0) continue;
                BattlemasterEntry e;
                e.id           = id;
                e.name         = dbc->getString(i, 11);
                e.maxGroupSize = dbc->getUInt32(i, 28);
                e.minLevel     = dbc->getUInt32(i, 30);
                e.maxLevel     = dbc->getUInt32(i, 31);
                // Fields 1-8 are map ids, 0xFFFFFFFF where unused, and field 9
                // is the instance type - verified against the file rather than
                // taken from a layout table, which does not carry this row.
                e.instanceType = dbc->getUInt32(i, 9);
                for (uint32_t f = 1; f <= 8; ++f) {
                    const uint32_t mapId = dbc->getUInt32(i, f);
                    if (mapId == 0xFFFFFFFFu) continue;
                    e.mapCount++;
                    e.mapIds.push_back(mapId);
                }
                if (e.instanceType == 3 && !e.name.empty()) battlegroundTypes_.push_back(e);
                battlemasterList_[id] = std::move(e);
            }
            // Ordered by id, so an index handed to the interface still means
            // the same battleground the next time it asks.
            std::sort(battlegroundTypes_.begin(), battlegroundTypes_.end(),
                      [](const BattlemasterEntry& a, const BattlemasterEntry& b) {
                          return a.id < b.id;
                      });
            LOG_INFO("Battleground: ", battlemasterList_.size(), " from BattlemasterList.dbc");
        }
    }
    auto it = battlemasterList_.find(bgTypeId);
    return it != battlemasterList_.end() ? &it->second : nullptr;
}

bool GameHandler::isBattlegroundMap(uint32_t mapId) {
    // Only the battleground rows, so an arena does not read as one. The pooled
    // "Random Battleground" row names the same maps the individual rows do, so
    // it costs nothing to walk and cannot answer wrongly.
    for (const auto& bg : getBattlegroundTypes()) {
        for (uint32_t id : bg.mapIds) {
            if (id == mapId) return true;
        }
    }
    return false;
}

bool GameHandler::isArenaMap(uint32_t mapId) {
    getBattlemasterInfo(0);   // latch the load, as isBattlegroundMap does
    for (const auto& [id, e] : battlemasterList_) {
        (void)id;
        if (e.instanceType != 4) continue;   // 4 = arena
        for (uint32_t m : e.mapIds) {
            if (m == mapId) return true;
        }
    }
    return false;
}

const std::vector<GameHandler::BattlemasterEntry>& GameHandler::getBattlegroundTypes() {
    // Through the same accessor, because that is where the load is latched.
    // Asking for the list before anything asked for an entry would otherwise
    // answer empty and latch nothing.
    getBattlemasterInfo(0);
    return battlegroundTypes_;
}

// CurrencyTypes.dbc: id, then the item that carries the amount. Everything the
// currency tab shows about a currency - name, icon, how many - comes from that
// item, so the row itself needs nothing else.
const std::vector<GameHandler::CurrencyType>& GameHandler::getCurrencyTypes() {
    if (currencyTypesLoaded_) return currencyTypes_;
    auto* am = services_.assetManager;
    // Checked before the latch: an early ask would otherwise leave the tab
    // permanently empty for the session.
    if (!am || !am->isInitialized()) return currencyTypes_;
    currencyTypesLoaded_ = true;

    auto dbc = am->loadDBC("CurrencyTypes.dbc");
    if (!dbc || !dbc->isLoaded() || dbc->getFieldCount() < 2) return currencyTypes_;
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        CurrencyType c;
        c.id     = dbc->getUInt32(i, 0);
        c.itemId = dbc->getUInt32(i, 1);
        if (c.id == 0 || c.itemId == 0) continue;
        currencyTypes_.push_back(c);
    }
    LOG_INFO("Currency: ", currencyTypes_.size(), " types from CurrencyTypes.dbc");
    return currencyTypes_;
}

// Achievement.dbc field 38 is the category. It is not in the layout file, but
// the fields either side of it are - Points at 39, Description at 21, IconID at
// 42 - which is the stock 3.3.5a order, so 38 is where the category sits.
void GameHandler::ensureGlyphPropertiesLoaded() {
    if (glyphPropertiesLoaded_) return;
    auto* am = services_.assetManager;
    // Checked before the latch, like the achievement loader below: one call
    // made before the asset manager is up would disable this for the session.
    if (!am || !am->isInitialized()) return;
    glyphPropertiesLoaded_ = true;

    auto dbc = am->loadDBC("GlyphProperties.dbc");
    if (!dbc || !dbc->isLoaded() || dbc->getFieldCount() < 2) return;
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        const uint32_t id = dbc->getUInt32(i, 0);
        const uint32_t spellId = dbc->getUInt32(i, 1);
        if (id != 0 && spellId != 0) glyphSpellCache_[id] = spellId;
    }
    LOG_INFO("Glyphs: ", glyphSpellCache_.size(), " properties mapped to spells");
}

void GameHandler::ensureAchievementCategoriesLoaded() {
    if (achievementCategoriesLoaded_) return;
    auto* am = services_.assetManager;
    // Checked before the latch, not after: one call made before the asset
    // manager is up would otherwise disable this for the rest of the session.
    if (!am || !am->isInitialized()) return;
    loadAchievementNameCache();
    achievementCategoriesLoaded_ = true;

    auto dbc = am->loadDBC("Achievement.dbc");
    if (!dbc || !dbc->isLoaded()) return;
    const uint32_t fieldCount = dbc->getFieldCount();
    if (fieldCount <= 38) return;
    // Field 41 is Flags, and bit 0x1 marks a statistic rather than an
    // achievement - verified against all 1817 rows of the 3.3.5a file by the
    // other thing that says the same: whether the row's category descends from
    // the top-level Statistics category. The two agree on every record, and no
    // category holds a mixture, so either signal alone would do; the flag is
    // used because it does not depend on a category being named in English.
    std::unordered_set<uint32_t> statisticCategories;
    const bool haveFlags = fieldCount > 41;
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        const uint32_t id = dbc->getUInt32(i, 0);
        if (id == 0 || achievementNameCache_.find(id) == achievementNameCache_.end()) continue;
        const uint32_t category = dbc->getUInt32(i, 38);
        achievementCategoryCache_[id] = category;
        categoryAchievements_[category].push_back(id);
        if (haveFlags && (getAchievementFlags(id) & 0x1u) != 0)
            statisticCategories.insert(category);
        // Field 3 is Supercedes: the achievement this one follows on from.
        // Checked by reading it - "Level 20" points at "Level 10" and "Expert
        // Cook" at "Journeyman Cook", and all 176 non-zero values are ids that
        // exist. Kept both ways round because the panel walks it backwards to
        // list a finished chain and forwards to find the step still open.
        const uint32_t supercedes = dbc->getUInt32(i, 3);
        if (supercedes != 0) {
            achievementSupercedes_[id] = supercedes;
            achievementSupercededBy_[supercedes] = id;
        }
    }

    // Achievement_Category.dbc: id, parent, then the localised names. A parent
    // of -1 (0xFFFFFFFF) means the category sits at the top of the tree.
    auto catDbc = am->loadDBC("Achievement_Category.dbc");
    if (catDbc && catDbc->isLoaded() && catDbc->getFieldCount() >= 3) {
        for (uint32_t i = 0; i < catDbc->getRecordCount(); ++i) {
            const uint32_t id = catDbc->getUInt32(i, 0);
            const uint32_t parent = catDbc->getUInt32(i, 1);
            AchievementCategoryInfo info;
            info.name = catDbc->getString(i, 2);
            info.parentId = (parent == 0xFFFFFFFFu) ? -1 : static_cast<int32_t>(parent);
            // Field 19 is where a category sits among its siblings. Sorted by
            // it, the top level comes out General, Quests, Exploration, Player
            // vs. Player, Dungeons & Raids, Professions, Reputation, World
            // Events, Feats of Strength, Statistics - which is the order the
            // real client shows. Record order is not that, so the tree would
            // otherwise read scrambled.
            if (catDbc->getFieldCount() > 19) info.uiOrder = catDbc->getUInt32(i, 19);
            achievementCategoryInfo_[id] = std::move(info);
            achievementCategoryOrder_.push_back(id);
        }

        // A category holding statistics is one, and so is every category above
        // it: the Statistics root holds no rows of its own, so marking only the
        // leaves would leave the tab with children and no trees to hang them
        // on. Walking up from each marked leaf reaches exactly 34 of the 86
        // categories, all of them descending from the one top-level Statistics
        // category and none of them shared with an achievement category.
        for (uint32_t leaf : statisticCategories) {
            for (int32_t c = static_cast<int32_t>(leaf); c >= 0; ) {
                auto it = achievementCategoryInfo_.find(static_cast<uint32_t>(c));
                if (it == achievementCategoryInfo_.end() || it->second.isStatistic) break;
                it->second.isStatistic = true;
                c = it->second.parentId;
            }
        }
        // Id breaks ties, so the order is total and the same on every load -
        // siblings share a ui_order across different parents.
        std::sort(achievementCategoryOrder_.begin(), achievementCategoryOrder_.end(),
                  [this](uint32_t a, uint32_t b) {
                      const uint32_t oa = achievementCategoryInfo_[a].uiOrder;
                      const uint32_t ob = achievementCategoryInfo_[b].uiOrder;
                      if (oa != ob) return oa < ob;
                      return a < b;
                  });
        // Split into the two lists the two tabs ask for, keeping that order.
        auto split = std::stable_partition(
            achievementCategoryOrder_.begin(), achievementCategoryOrder_.end(),
            [this](uint32_t c) { return !achievementCategoryInfo_[c].isStatistic; });
        statisticCategoryOrder_.assign(split, achievementCategoryOrder_.end());
        achievementCategoryOrder_.erase(split, achievementCategoryOrder_.end());
    }
    LOG_INFO("Achievement: ", achievementCategoryOrder_.size(), " categories, ",
             statisticCategoryOrder_.size(), " statistic categories, ",
             achievementCategoryCache_.size(), " achievements placed");
}

// Achievement_Criteria.dbc: id, achievement, type, asset, quantity, then the
// localised descriptions. Type and asset are what the panel needs to tell a
// "kill N" criterion from a "reach level N" one.
void GameHandler::ensureAchievementCriteriaLoaded() {
    if (achievementCriteriaLoaded_) return;
    auto* am = services_.assetManager;
    if (!am || !am->isInitialized()) return;
    achievementCriteriaLoaded_ = true;

    auto dbc = am->loadDBC("Achievement_Criteria.dbc");
    if (!dbc || !dbc->isLoaded() || dbc->getFieldCount() < 10) return;

    const auto* critL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("AchievementCriteria") : nullptr;
    auto fieldOr = [&](const char* name, uint32_t fallback) {
        const uint32_t f = critL ? critL->field(name) : 0xFFFFFFFFu;
        return (f == 0xFFFFFFFFu) ? fallback : f;
    };
    const uint32_t achField  = fieldOr("AchievementID", 1);
    const uint32_t qtyField  = fieldOr("Quantity", 4);
    const uint32_t descField = fieldOr("Description", 9);
    // Field 29 holds the timer, where there is one. Fifty-nine criteria have a
    // non-zero value and every one is a round number of seconds that the
    // criteria's own description states in words - 420 against "Win Warsong
    // Gulch in under 7 minutes", 1200 against "Kill Maexxna within 20 minutes",
    // 60 against "Kill 100 Risen Zombies in 1 minute". No other field in the
    // record agrees with the text that way.
    const uint32_t limitField = fieldOr("TimeLimit", 29);
    // Field 26, identified the same way field 29 was: the hundred and sixty
    // rows with bit one set are the "Complete N quests" family that WoW draws
    // as bars, and the large-quantity rows without it - weapon skills at four
    // hundred, reputations at forty-two thousand - are the ones it draws as
    // text. Fields 9 to 25 are the sixteen locale strings and their mask, so
    // this is the first word past them.
    const uint32_t flagsField = fieldOr("Flags", 26);
    const uint32_t fieldCount = dbc->getFieldCount();

    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        AchievementCriterion c;
        c.id = dbc->getUInt32(i, 0);
        const uint32_t achId = dbc->getUInt32(i, achField);
        if (achId == 0) continue;
        if (fieldCount > 2) c.type    = dbc->getUInt32(i, 2);
        if (fieldCount > 3) c.assetId = dbc->getUInt32(i, 3);
        if (qtyField  < fieldCount) c.quantity    = dbc->getUInt32(i, qtyField);
        if (descField < fieldCount) c.description = dbc->getString(i, descField);
        if (limitField < fieldCount) c.timeLimit = dbc->getUInt32(i, limitField);
        if (flagsField < fieldCount) c.flags = dbc->getUInt32(i, flagsField);
        achievementCriterionById_[c.id] = {.achievementId = achId, .timeLimit = c.timeLimit};
        achievementCriteria_[achId].push_back(std::move(c));
    }
    LOG_INFO("Achievement: criteria for ", achievementCriteria_.size(), " achievements");
}

// ---------------------------------------------------------------------------
// SMSG_ALL_ACHIEVEMENT_DATA (WotLK 3.3.5a)
//   Achievement records: repeated { uint32 id, uint32 packedDate } until 0xFFFFFFFF sentinel
//   Criteria records:    repeated { uint32 id, uint64 counter, uint32 packedDate, ... } until 0xFFFFFFFF
// ---------------------------------------------------------------------------
void GameHandler::handleAllAchievementData(network::Packet& packet) {
    loadAchievementNameCache();
    earnedAchievements_.clear();
    achievementDates_.clear();

    // Parse achievement entries (id + packedDate pairs, sentinel 0xFFFFFFFF)
    while (packet.hasRemaining(4)) {
        uint32_t id = packet.readUInt32();
        if (id == 0xFFFFFFFF) break;
        if (!packet.hasRemaining(4)) break;
        uint32_t date = packet.readUInt32();
        earnedAchievements_.insert(id);
        achievementDates_[id] = date;
    }

    // Parse the criteria block: records until an id of -1. The record's shape
    // is in readCriteriaProgressTail, which SMSG_CRITERIA_UPDATE reads with too
    // - the server builds both from the same lines.
    criteriaProgress_.clear();
    while (packet.hasRemaining(4)) {
        uint32_t id = packet.readUInt32();
        if (id == 0xFFFFFFFF) break;
        CriteriaProgressRecord rec;
        if (!readCriteriaProgressTail(packet, rec)) break;
        criteriaProgress_[id] = rec.counter;
    }

    LOG_INFO("SMSG_ALL_ACHIEVEMENT_DATA: loaded ", earnedAchievements_.size(),
             " achievements, ", criteriaProgress_.size(), " criteria");
    // The panel builds its whole tree on this, and it arrives once at login -
    // well before anything opens the panel - so without the event a panel
    // opened later showed the empty state it was built with.
    if (addonEventCallback_) addonEventCallback_("RECEIVED_ACHIEVEMENT_LIST", {});
}

// ---------------------------------------------------------------------------
// SMSG_RESPOND_INSPECT_ACHIEVEMENTS (WotLK 3.3.5a)
//   Wire format: packed_guid (inspected player) + same achievement/criteria
//   blocks as SMSG_ALL_ACHIEVEMENT_DATA:
//     Achievement records: repeated { uint32 id, uint32 packedDate } until 0xFFFFFFFF sentinel
//     Criteria records:    repeated { uint32 id, uint64 counter, uint32 date, uint32 unk }
//                          until 0xFFFFFFFF sentinel
//   We store only the earned achievement IDs (not criteria) per inspected player.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Faction name cache (lazily loaded from Faction.dbc)
// ---------------------------------------------------------------------------

void GameHandler::loadFactionNameCache() const {
    if (factionNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    factionNameCacheLoaded_ = true;

    auto dbc = am->loadDBC("Faction.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    // Faction.dbc WotLK 3.3.5a field layout:
    //   0: ID
    //   1: ReputationListID  (-1 / 0xFFFFFFFF = no reputation tracking)
    //   2-5:  ReputationRaceMask[4]
    //   6-9:  ReputationClassMask[4]
    //  10-13: ReputationBase[4]
    //  14-17: ReputationFlags[4]
    //  18:    ParentFactionID
    //  19-20: SpilloverRateIn, SpilloverRateOut (floats)
    //  21-22: SpilloverMaxRankIn, SpilloverMaxRankOut
    //  23:    Name (English locale, string ref)
    constexpr uint32_t ID_FIELD      = 0;
    constexpr uint32_t REPLIST_FIELD = 1;
    constexpr uint32_t PARENT_FIELD  = 18;
    constexpr uint32_t NAME_FIELD    = 23;  // enUS name string

    // Classic/TBC have fewer fields; fall back gracefully
    const bool hasRepListField = dbc->getFieldCount() > REPLIST_FIELD;
    // The parent chain is what groups the reputation panel into the headers
    // the original interface draws. Read here rather than in a second pass so
    // there is one opinion about which field holds it.
    const bool hasParentField = dbc->getFieldCount() > PARENT_FIELD;
    if (dbc->getFieldCount() <= NAME_FIELD) {
        LOG_WARNING("Faction.dbc: unexpected field count ", dbc->getFieldCount());
        // Don't abort - still try to load names from a shorter layout
    }
    const uint32_t nameField = (dbc->getFieldCount() > NAME_FIELD) ? NAME_FIELD : 22u;

    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t factionId = dbc->getUInt32(i, ID_FIELD);
        if (factionId == 0) continue;
        if (dbc->getFieldCount() > nameField) {
            std::string name = dbc->getString(i, nameField);
            if (!name.empty()) {
                factionNameCache_[factionId] = std::move(name);
            }
        }
        if (hasParentField) {
            const uint32_t parentId = dbc->getUInt32(i, PARENT_FIELD);
            if (parentId != 0 && parentId != factionId) {
                factionParent_[factionId] = parentId;
            }
        }
        // Build repListId ↔ factionId mapping (WotLK field 1)
        if (hasRepListField) {
            uint32_t repListId = dbc->getUInt32(i, REPLIST_FIELD);
            if (repListId != 0xFFFFFFFFu) {
                factionRepListToId_[repListId] = factionId;
                factionIdToRepList_[factionId] = repListId;
            }
        }
    }
    LOG_INFO("Faction.dbc: loaded ", factionNameCache_.size(), " faction names, ",
             factionRepListToId_.size(), " with reputation tracking");
}

uint32_t GameHandler::getFactionIdByRepListId(uint32_t repListId) const {
    loadFactionNameCache();
    auto it = factionRepListToId_.find(repListId);
    return (it != factionRepListToId_.end()) ? it->second : 0u;
}

uint32_t GameHandler::getRepListIdByFactionId(uint32_t factionId) const {
    loadFactionNameCache();
    auto it = factionIdToRepList_.find(factionId);
    return (it != factionIdToRepList_.end()) ? it->second : 0xFFFFFFFFu;
}

void GameHandler::setWatchedFactionId(uint32_t factionId) {
    watchedFactionId_ = factionId;
    if (!isInWorld()) return;
    // CMSG_SET_WATCHED_FACTION: int32 repListId (-1 = unwatch)
    int32_t repListId = -1;
    if (factionId != 0) {
        uint32_t rl = getRepListIdByFactionId(factionId);
        if (rl != 0xFFFFFFFFu) repListId = static_cast<int32_t>(rl);
    }
    network::Packet pkt(wireOpcode(Opcode::CMSG_SET_WATCHED_FACTION));
    pkt.writeUInt32(static_cast<uint32_t>(repListId));
    socket->send(pkt);
    LOG_DEBUG("CMSG_SET_WATCHED_FACTION: repListId=", repListId, " (factionId=", factionId, ")");
}

void GameHandler::setFactionAtWar(uint32_t repListId, bool atWar) {
    if (repListId >= initialFactions_.size()) return;
    // The server forbids declaring war on some factions; don't fight the flag.
    if (atWar && isFactionPeaceForced(repListId)) return;
    // Optimistic local update; the server echoes SMSG_SET_FACTION_ATWAR to confirm.
    if (atWar) initialFactions_[repListId].flags |=  FACTION_FLAG_AT_WAR;
    else       initialFactions_[repListId].flags &= ~FACTION_FLAG_AT_WAR;
    if (!isInWorld() || !socket) return;
    // CMSG_SET_FACTION_ATWAR: uint32 repListId + uint8 flag
    network::Packet pkt(wireOpcode(Opcode::CMSG_SET_FACTION_ATWAR));
    pkt.writeUInt32(repListId);
    pkt.writeUInt8(atWar ? 1u : 0u);
    socket->send(pkt);
    LOG_DEBUG("CMSG_SET_FACTION_ATWAR: repListId=", repListId, " atWar=", atWar);
}

void GameHandler::setFactionInactive(uint32_t repListId, bool inactive) {
    if (repListId >= initialFactions_.size()) return;
    // No SMSG confirmation is sent for inactive, so update the local flag directly.
    if (inactive) initialFactions_[repListId].flags |=  FACTION_FLAG_INACTIVE;
    else          initialFactions_[repListId].flags &= ~FACTION_FLAG_INACTIVE;
    if (!isInWorld() || !socket) return;
    // CMSG_SET_FACTION_INACTIVE: uint32 repListId + uint8 flag
    network::Packet pkt(wireOpcode(Opcode::CMSG_SET_FACTION_INACTIVE));
    pkt.writeUInt32(repListId);
    pkt.writeUInt8(inactive ? 1u : 0u);
    socket->send(pkt);
    LOG_DEBUG("CMSG_SET_FACTION_INACTIVE: repListId=", repListId, " inactive=", inactive);
}

std::string GameHandler::getFactionName(uint32_t factionId) const {
    auto it = factionNameCache_.find(factionId);
    if (it != factionNameCache_.end()) return it->second;
    return "faction #" + std::to_string(factionId);
}

const std::string& GameHandler::getFactionNamePublic(uint32_t factionId) const {
    loadFactionNameCache();
    auto it = factionNameCache_.find(factionId);
    if (it != factionNameCache_.end()) return it->second;
    static const std::string empty;
    return empty;
}

// ---------------------------------------------------------------------------
// Area name cache (lazy-loaded from WorldMapArea.dbc)
// ---------------------------------------------------------------------------

void GameHandler::loadAreaNameCache() const {
    if (areaNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    areaNameCacheLoaded_ = true;

    // AreaTable.dbc has the canonical zone/area names keyed by AreaID.
    // Field 0 = ID, field 11 = AreaName (enUS locale).
    auto areaDbc = am->loadDBC("AreaTable.dbc");
    if (areaDbc && areaDbc->isLoaded() && areaDbc->getFieldCount() > 11) {
        for (uint32_t i = 0; i < areaDbc->getRecordCount(); ++i) {
            uint32_t areaId = areaDbc->getUInt32(i, 0);
            if (areaId == 0) continue;
            std::string name = areaDbc->getString(i, 11);
            if (!name.empty()) {
                areaNameCache_[areaId] = std::move(name);
            }
            // Field 4 is the flag word and field 28 the controlling team, which
            // together are the whole of what GetZonePVPInfo answers. Read while
            // the file is open rather than opening it twice.
            if (areaDbc->getFieldCount() > 28) {
                areaPvpCache_[areaId] = AreaPvpInfo{.flags = areaDbc->getUInt32(i, 4),
                                                    .team = areaDbc->getUInt32(i, 28)};
            }
        }
    }

    // WorldMapArea.dbc supplements with map-UI area names (different ID space).
    auto dbc = am->loadDBC("WorldMapArea.dbc");
    if (dbc && dbc->isLoaded()) {
        const auto* layout = pipeline::getActiveDBCLayout()
            ? pipeline::getActiveDBCLayout()->getLayout("WorldMapArea") : nullptr;
        const uint32_t areaIdField   = layout ? (*layout)["AreaID"]   : 2;
        const uint32_t areaNameField = layout ? (*layout)["AreaName"] : 3;

        if (dbc->getFieldCount() > areaNameField) {
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                uint32_t areaId = dbc->getUInt32(i, areaIdField);
                if (areaId == 0) continue;
                std::string name = dbc->getString(i, areaNameField);
                // Don't overwrite AreaTable names - those are authoritative
                if (!name.empty() && !areaNameCache_.count(areaId)) {
                    areaNameCache_[areaId] = std::move(name);
                }
            }
        }
    }

    LOG_INFO("Area name cache: loaded ", areaNameCache_.size(), " entries");
}

std::string GameHandler::getAreaName(uint32_t areaId) const {
    if (areaId == 0) return {};
    loadAreaNameCache();
    auto it = areaNameCache_.find(areaId);
    return (it != areaNameCache_.end()) ? it->second : std::string{};
}

void GameHandler::loadMapNameCache() const {
    if (mapNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    mapNameCacheLoaded_ = true;

    auto dbc = am->loadDBC("Map.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    // Map.dbc layout: 0=ID, 1=InternalName, 2=InstanceType, 3=Flags,
    // 4=MapName_enUS (display name), fields 5+ = other locales
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        uint32_t id = dbc->getUInt32(i, 0);
        mapInstanceTypeCache_[id] = dbc->getUInt32(i, 2);
        std::string name = dbc->getString(i, 4);
        if (name.empty()) name = dbc->getString(i, 1); // internal name fallback
        if (!name.empty() && !mapNameCache_.count(id)) {
            mapNameCache_[id] = std::move(name);
        }
    }
    LOG_INFO("Map.dbc: loaded ", mapNameCache_.size(), " map names");
}

std::string GameHandler::getMapName(uint32_t mapId) const {
    loadMapNameCache();
    auto it = mapNameCache_.find(mapId);
    return (it != mapNameCache_.end()) ? it->second : std::string{};
}

void GameHandler::sendOptOutOfLoot(bool optOut) {
    if (!isInWorld() || !getSocket()) return;
    // One uint32, one or zero. The server keeps the state and never sends it
    // back, which is why the interface remembers its own copy.
    network::Packet packet(wireOpcode(Opcode::CMSG_OPT_OUT_OF_LOOT));
    packet.writeUInt32(optOut ? 1u : 0u);
    getSocket()->send(packet);
    LOG_INFO("CMSG_OPT_OUT_OF_LOOT: ", optOut ? "opting out" : "opting in");
}

void GameHandler::runInterfaceCommand(const std::string& lua) const {
    if (!interfaceCommand_) {
        // A key reaching here before the interface is up does nothing, and does
        // it silently. Said out loud, because "the window did not open" is
        // otherwise indistinguishable from a key that never arrived.
        LOG_WARNING("interface command dropped, no interface yet: ", lua);
        return;
    }
    // At warning level, because the file log filters info out by default and
    // this is the line that separates "the key never arrived" from "the key
    // arrived and the interface did nothing with it".
    LOG_WARNING("interface command: ", lua);
    interfaceCommand_(lua);
}

void GameHandler::requestGuildBankLog(uint8_t tab) {
    if (socialHandler_) socialHandler_->requestGuildBankLog(tab);
}

const std::vector<GuildBankLogEntry>& GameHandler::getGuildBankLog(uint8_t tab) const {
    static const std::vector<GuildBankLogEntry> empty;
    return socialHandler_ ? socialHandler_->getGuildBankLog(tab) : empty;
}

void GameHandler::requestChannelList(const std::string& channel) {
    if (chatHandler_) chatHandler_->requestChannelList(channel);
}

const std::vector<ChannelMember>&
GameHandler::getChannelRoster(const std::string& channel) const {
    static const std::vector<ChannelMember> empty;
    return chatHandler_ ? chatHandler_->getChannelRoster(channel) : empty;
}

uint64_t GameHandler::getEncounterUnitGuid(uint32_t slot) const {
    return socialHandler_ ? socialHandler_->getEncounterUnitGuid(slot) : 0;
}

bool GameHandler::getInstanceLockPrompt(float& secondsLeft, bool& previouslySaved,
                                        uint32_t& completedMask) const {
    if (!socialHandler_) return false;
    const auto& prompt = socialHandler_->getInstanceLockPrompt();
    if (!prompt.active) return false;
    secondsLeft = prompt.secondsLeft;
    previouslySaved = prompt.previouslySaved;
    completedMask = prompt.completedEncounterMask;
    return true;
}

void GameHandler::respondInstanceLock(bool accept) {
    if (socialHandler_) socialHandler_->respondInstanceLock(accept);
}

uint32_t GameHandler::getDungeonEncounterCount(uint32_t mapId, uint32_t difficulty) const {
    if (!dungeonEncounterCacheLoaded_) {
        auto* am = services_.assetManager;
        // Not an attempt: the assets are not there to read yet, and a caller
        // can reach this before they are.
        if (!am || !am->isInitialized()) return 0;
        dungeonEncounterCacheLoaded_ = true;

        auto dbc = am->loadDBC("DungeonEncounter.dbc");
        if (dbc && dbc->isLoaded()) {
            // ID, MapID, Difficulty, OrderIndex, Bit, Name[17], SpellIconID.
            // Verified against record 0: id 464, map 33 (Shadowfang Keep),
            // difficulty 0, order 0, bit 0.
            constexpr uint32_t kMapField = 1, kDiffField = 2;
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                const uint64_t map = dbc->getUInt32(i, kMapField);
                const uint64_t diff = dbc->getUInt32(i, kDiffField);
                dungeonEncounterCounts_[(map << 32) | diff]++;
                // Most instances list their bosses once, under difficulty 0,
                // and apply to every difficulty. Keeping a per-map total means
                // a heroic run still has a number to show.
                dungeonEncounterCounts_[(map << 32) | 0xFFFFFFFFull]++;
            }
        }
        LOG_INFO("DungeonEncounter.dbc: ", dungeonEncounterCounts_.size(), " map/difficulty pairs");
    }

    auto it = dungeonEncounterCounts_.find((static_cast<uint64_t>(mapId) << 32) | difficulty);
    if (it != dungeonEncounterCounts_.end()) return it->second;
    it = dungeonEncounterCounts_.find((static_cast<uint64_t>(mapId) << 32) | 0xFFFFFFFFull);
    return (it != dungeonEncounterCounts_.end()) ? it->second : 0;
}

// ---------------------------------------------------------------------------
// LFG dungeon name cache (WotLK: LFGDungeons.dbc)
// ---------------------------------------------------------------------------

void GameHandler::loadLfgDungeonDbc() const {
    if (lfgDungeonNameCacheLoaded_) return;

    auto* am = services_.assetManager;
    // Not an attempt: the assets are not there to read yet, and a
    // caller can reach this before they are. Marking it loaded here
    // meant one early call disabled this file for the whole session.
    if (!am || !am->isInitialized()) return;
    lfgDungeonNameCacheLoaded_ = true;

    auto dbc = am->loadDBC("LFGDungeons.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    const auto* layout = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("LFGDungeons") : nullptr;
    const uint32_t idField   = layout ? (*layout)["ID"]   : 0;
    const uint32_t nameField = layout ? (*layout)["Name"] : 1;

    // Everything past the name, which the layout table does not describe. The
    // WotLK file is 49 fields wide: sixteen name locales and their flags fill
    // 1-17, and the rest follow in this order. Read off the file and checked
    // against rows that can be recognised rather than taken on trust - Wailing
    // Caverns on map 43, Ragefire Chasm the one entry with faction 0, Karazhan
    // grouped with the Burning Crusade raids.
    constexpr uint32_t kMinLevel = 18, kMaxLevel = 19, kTargetLevel = 20;
    constexpr uint32_t kTargetMin = 21, kTargetMax = 22, kMapId = 23;
    constexpr uint32_t kDifficulty = 24, kTypeId = 26, kFaction = 27;
    constexpr uint32_t kTexture = 28, kExpansion = 29, kOrderIndex = 30, kGroupId = 31;
    // 32 through 48 are the description in each locale. Only the holiday rows
    // and a handful of others carry one.
    constexpr uint32_t kDescription = 32;
    // Group 11 is the four seasonal bosses - the Headless Horseman, Ahune,
    // Coren Direbrew and the Crown Chemical Co. - and nothing else. The file
    // has no holiday flag of its own; the group is the flag.
    constexpr uint32_t kHolidayGroup = 11;
    const bool wideEnough = dbc->getFieldCount() > kGroupId;
    if (!wideEnough) {
        LOG_WARNING("LFGDungeons.dbc has only ", dbc->getFieldCount(),
                    " fields - the dungeon finder will list names only");
    }

    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        uint32_t id = dbc->getUInt32(i, idField);
        if (id == 0) continue;
        std::string name = dbc->getString(i, nameField);
        if (name.empty()) continue;
        lfgDungeonNameCache_[id] = name;
        if (!wideEnough) continue;

        LfgDungeon d;
        d.id          = id;
        d.name        = std::move(name);
        d.minLevel    = dbc->getUInt32(i, kMinLevel);
        d.maxLevel    = dbc->getUInt32(i, kMaxLevel);
        d.recLevel    = dbc->getUInt32(i, kTargetLevel);
        d.minRecLevel = dbc->getUInt32(i, kTargetMin);
        d.maxRecLevel = dbc->getUInt32(i, kTargetMax);
        d.mapId       = dbc->getUInt32(i, kMapId);
        d.difficulty  = dbc->getUInt32(i, kDifficulty);
        d.typeId      = dbc->getUInt32(i, kTypeId);
        d.faction     = static_cast<int32_t>(dbc->getUInt32(i, kFaction));
        d.texture     = dbc->getString(i, kTexture);
        d.expansion   = dbc->getUInt32(i, kExpansion);
        d.orderIndex  = dbc->getUInt32(i, kOrderIndex);
        d.groupId     = dbc->getUInt32(i, kGroupId);
        d.isHoliday   = (d.groupId == kHolidayGroup);
        if (dbc->getFieldCount() > kDescription) {
            d.description = dbc->getString(i, kDescription);
        }
        lfgDungeons_.push_back(std::move(d));
    }
    // The order the picker lists them in: by group, then by the level the
    // dungeon is meant for, so a category reads from lowest to highest.
    std::sort(lfgDungeons_.begin(), lfgDungeons_.end(),
              [](const LfgDungeon& a, const LfgDungeon& b) {
                  if (a.groupId != b.groupId) return a.groupId < b.groupId;
                  if (a.orderIndex != b.orderIndex) return a.orderIndex < b.orderIndex;
                  if (a.recLevel != b.recLevel) return a.recLevel < b.recLevel;
                  return a.id < b.id;
              });
    LOG_INFO("LFGDungeons.dbc: loaded ", lfgDungeonNameCache_.size(), " dungeon names, ",
             lfgDungeons_.size(), " listable entries");
}

// ---------------------------------------------------------------------------
// Mounts and critters
//
// The pet tab lists both, and both are spells the player knows. What separates
// them is what the spell does, which is in Spell.dbc:
//
//   a mount   applies aura 78 (SPELL_AURA_MOUNTED)
//   a critter uses effect 28 (SPELL_EFFECT_SUMMON) with EffectMiscValueB 41,
//             which is the critter SummonProperties row
//
// Both checked against the file rather than assumed: aura 78 picks out Brown
// Horse, Gray Wolf and White Stallion, and effect 28 with miscB 41 picks out
// Mechanical Squirrel, Cockroach and Worg Pup. EffectMiscValue beside it is the
// creature, which is what GetCompanionInfo reports first.
// ---------------------------------------------------------------------------
void GameHandler::rebuildCompanions() const {
    const auto& known = getKnownSpells();
    // Only the spellbook changing can change the answer, and it changes rarely.
    if (companionsBuiltFromSpellCount_ == known.size()) return;
    companionsBuiltFromSpellCount_ = known.size();
    mountSpells_.clear();
    critterSpells_.clear();

    auto* am = services_.assetManager;
    if (!am || !am->isInitialized()) return;
    auto dbc = am->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    const auto* layout = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
    if (!layout) return;
    const uint32_t idField = (*layout)["ID"];

    // Built once per rebuild rather than searched per spell: the file is fifty
    // thousand rows and the spellbook is a few hundred.
    std::unordered_map<uint32_t, uint32_t> rowOfSpell;
    rowOfSpell.reserve(dbc->getRecordCount());
    for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
        rowOfSpell[dbc->getUInt32(row, idField)] = row;
    }

    constexpr uint32_t kAuraMounted = 78;
    constexpr uint32_t kEffectSummon = 28;
    constexpr uint32_t kCritterSummonProperties = 41;

    for (uint32_t spellId : known) {
        auto it = rowOfSpell.find(spellId);
        if (it == rowOfSpell.end()) continue;
        const uint32_t row = it->second;

        Companion c;
        c.spellId = spellId;
        c.name = getSpellName(spellId);
        if (c.name.empty()) continue;

        bool mount = false, critter = false;
        for (int i = 0; i < 3 && !mount && !critter; ++i) {
            const std::string idx = std::to_string(i);
            if (dbc->getUInt32(row, (*layout)["EffectApplyAuraName" + idx]) == kAuraMounted) {
                mount = true;
                // SPELL_AURA_MOUNTED's misc value is a creature *entry*, the
                // same number space a summon's is. AuraEffect::HandleAuraMounted
                // opens with `uint32 creatureEntry = GetMiscValue();` and then
                // picks the model with ChooseDisplayId on that entry's
                // template - it is not handed to Player::Mount as a display id,
                // which is what the comment here used to say. Stored as the
                // entry and resolved by the reader, exactly like a critter's.
                c.creatureId = dbc->getUInt32(row, (*layout)["EffectMiscValue" + idx]);
            } else if (dbc->getUInt32(row, (*layout)["Effect" + idx]) == kEffectSummon &&
                       dbc->getUInt32(row, (*layout)["EffectMiscValueB" + idx]) ==
                           kCritterSummonProperties) {
                critter = true;
                // A summon's misc value is a creature *entry*, not a display
                // id, and the model frame takes a display id. Left as the entry
                // and resolved in GetCompanionInfo, which is the only reader.
                // Both kinds are entries, so it no longer matters there which
                // kind was asked for.
                //
                // It used to resolve here when the cache happened to be warm
                // and leave the entry when it did not, on the reading that a
                // later rebuild would catch it - but nothing rebuilds on a
                // query answer and nothing sent the query, so the field meant a
                // display id or an entry depending on what had been near the
                // player. One number space, decided in one place.
                c.creatureId = dbc->getUInt32(row, (*layout)["EffectMiscValue" + idx]);
            }
        }
        if (!mount && !critter) continue;
        c.isMount = mount;
        (mount ? mountSpells_ : critterSpells_).push_back(std::move(c));
    }

    auto byName = [](const Companion& a, const Companion& b) { return a.name < b.name; };
    std::sort(mountSpells_.begin(), mountSpells_.end(), byName);
    std::sort(critterSpells_.begin(), critterSpells_.end(), byName);
    LOG_INFO("Companions: ", mountSpells_.size(), " mounts, ",
             critterSpells_.size(), " critters from ", known.size(), " known spells");
}

uint32_t GameHandler::getBonusActionBarOffset() const {
    const uint8_t form = shapeshiftFormId_;
    if (form == 0) return 0;

    // Cached per form: the file does not change, and this is asked on every
    // action button update.
    static std::unordered_map<uint8_t, uint32_t> barOfForm;
    if (auto it = barOfForm.find(form); it != barOfForm.end()) return it->second;

    uint32_t bar = 0;
    auto* am = services_.assetManager;
    if (am && am->isInitialized()) {
        if (auto dbc = am->loadDBC("SpellShapeshiftForm.dbc"); dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                ? pipeline::getActiveDBCLayout()->getLayout("SpellShapeshiftForm") : nullptr;
            const uint32_t idField  = layout ? (*layout)["ID"] : 0;
            const uint32_t barField = layout ? (*layout)["BonusActionBar"] : 1;
            for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
                if (dbc->getUInt32(row, idField) != form) continue;
                bar = dbc->getUInt32(row, barField);
                break;
            }
        }
    }
    barOfForm[form] = bar;
    return bar;
}

void GameHandler::announceCompanionChange() {
    if (!addonEventCallback_) return;
    for (bool mounts : {true, false}) {
        uint32_t active = 0;
        for (const auto& c : getCompanions(mounts)) {
            for (const auto& a : getPlayerAuras()) {
                if (a.spellId == c.spellId) { active = c.spellId; break; }
            }
            if (active) break;
        }
        uint32_t& remembered = mounts ? activeMountSpell_ : activeCritterSpell_;
        if (remembered == active) continue;
        remembered = active;
        addonEventCallback_("COMPANION_UPDATE", {mounts ? "MOUNT" : "CRITTER"});
    }
}

const std::vector<Companion>& GameHandler::getCompanions(bool mounts) const {
    rebuildCompanions();
    return mounts ? mountSpells_ : critterSpells_;
}
std::string GameHandler::companionKindForCreature(uint32_t entry) const {
    if (entry == 0) return {};
    // Both lists, not critters alone. A mount's creature id was taken for a
    // display id already, and it is not one: AzerothCore reads the mounted
    // aura's misc value with GetCreatureTemplate and picks the model with
    // ChooseDisplayId, so it is a template entry exactly as a critter's is.
    // Leaving mounts out meant a mount's model waited on a query that nothing
    // reported the answer to, and appeared on the next login instead.
    for (const Companion& c : getCompanions(/*mounts=*/false))
        if (c.creatureId == entry) return "CRITTER";
    for (const Companion& c : getCompanions(/*mounts=*/true))
        if (c.creatureId == entry) return "MOUNT";
    return {};
}

std::string GameHandler::getLfgDungeonName(uint32_t dungeonId) const {
    if (dungeonId == 0) return {};
    loadLfgDungeonDbc();
    auto it = lfgDungeonNameCache_.find(dungeonId);
    return (it != lfgDungeonNameCache_.end()) ? it->second : std::string{};
}

// ---------------------------------------------------------------------------
// Aura duration update
// ---------------------------------------------------------------------------

void GameHandler::handleUpdateAuraDuration(uint8_t slot, uint32_t durationMs) {
    if (spellHandler_) spellHandler_->handleUpdateAuraDuration(slot, durationMs);
}

// ---------------------------------------------------------------------------
// Equipment set list
// ---------------------------------------------------------------------------

// ---- Battlefield Manager (WotLK Wintergrasp / outdoor battlefields) ----

void GameHandler::acceptBfMgrInvite() {
    if (socialHandler_) socialHandler_->acceptBfMgrInvite();
}

void GameHandler::declineBfMgrInvite() {
    if (socialHandler_) socialHandler_->declineBfMgrInvite();
}

void GameHandler::respondBfMgrQueueInvite(uint32_t battleId, bool accept) {
    if (socialHandler_) socialHandler_->respondBfMgrQueueInvite(battleId, accept);
}

void GameHandler::requestBfMgrExit(uint32_t battleId) {
    if (socialHandler_) socialHandler_->requestBfMgrExit(battleId);
}

// ---- WotLK Calendar ----

void GameHandler::requestCalendar() {
    if (socialHandler_) socialHandler_->requestCalendar();
}

void GameHandler::inviteToCalendarEvent(uint64_t eventId, uint64_t inviteId,
                                        const std::string& name,
                                        bool isPreInvite, bool isGuildEvent) {
    if (socialHandler_) {
        socialHandler_->inviteToCalendarEvent(eventId, inviteId, name,
                                              isPreInvite, isGuildEvent);
    }
}

void GameHandler::massInviteGuildToCalendarEvent(uint32_t minLevel,
                                                 uint32_t maxLevel,
                                                 uint32_t minRank) {
    if (socialHandler_) {
        socialHandler_->massInviteGuildToCalendarEvent(minLevel, maxLevel, minRank);
    }
}

void GameHandler::updateCalendarEvent(uint64_t eventId, uint64_t inviteId,
                                      const CalendarEventDraft& draft) {
    if (socialHandler_) socialHandler_->updateCalendarEvent(eventId, inviteId, draft);
}

void GameHandler::removeCalendarEvent(uint64_t eventId, uint64_t inviteId) {
    if (socialHandler_) socialHandler_->removeCalendarEvent(eventId, inviteId);
}

void GameHandler::setCalendarInviteStatus(uint64_t inviteeGuid, uint64_t eventId,
                                          uint64_t inviteId, uint8_t status) {
    if (socialHandler_) {
        socialHandler_->setCalendarInviteStatus(inviteeGuid, eventId, inviteId, status);
    }
}

void GameHandler::setCalendarInviteModerator(uint64_t inviteeGuid, uint64_t eventId,
                                             uint64_t inviteId, uint8_t rank) {
    if (socialHandler_) {
        socialHandler_->setCalendarInviteModerator(inviteeGuid, eventId, inviteId, rank);
    }
}

void GameHandler::requestCalendarEvent(uint64_t eventId) {
    if (socialHandler_) socialHandler_->requestCalendarEvent(eventId);
}

void GameHandler::createCalendarEvent(const CalendarEventDraft& draft) {
    if (socialHandler_) socialHandler_->createCalendarEvent(draft);
}

void GameHandler::respondToCalendarInvite(uint64_t eventId, uint64_t inviteId,
                                          uint32_t status) {
    if (socialHandler_) {
        socialHandler_->respondToCalendarInvite(eventId, inviteId, status);
    }
}

// ============================================================
// Delegating getters - SocialHandler owns the canonical state
// ============================================================

uint32_t GameHandler::getTotalTimePlayed() const {
    return socialHandler_ ? socialHandler_->getTotalTimePlayed() : 0;
}

uint32_t GameHandler::getLevelTimePlayed() const {
    return socialHandler_ ? socialHandler_->getLevelTimePlayed() : 0;
}

const std::vector<GameHandler::WhoEntry>& GameHandler::getWhoResults() const {
    if (socialHandler_) return socialHandler_->getWhoResults();
    static const std::vector<WhoEntry> empty;
    return empty;
}

bool GameHandler::isInGroup() const {
    return socialHandler_ ? socialHandler_->isInGroup() : !partyData.isEmpty();
}

const GroupListData& GameHandler::getPartyData() const {
    if (socialHandler_) return socialHandler_->getPartyData();
    return partyData;
}

uint32_t GameHandler::getWhoOnlineCount() const {
    return socialHandler_ ? socialHandler_->getWhoOnlineCount() : 0;
}

void GameHandler::setWhoToUI(bool toUI) {
    if (socialHandler_) socialHandler_->setWhoToUI(toUI);
}

const std::array<GameHandler::BgQueueSlot, 3>& GameHandler::getBgQueues() const {
    if (socialHandler_) return socialHandler_->getBgQueues();
    static const std::array<BgQueueSlot, 3> empty{};
    return empty;
}

const std::vector<GameHandler::AvailableBgInfo>& GameHandler::getAvailableBgs() const {
    if (socialHandler_) return socialHandler_->getAvailableBgs();
    static const std::vector<AvailableBgInfo> empty;
    return empty;
}

const GameHandler::BgScoreboardData* GameHandler::getBgScoreboard() const {
    return socialHandler_ ? socialHandler_->getBgScoreboard() : nullptr;
}

const std::vector<GameHandler::BgPlayerPosition>& GameHandler::getBgPlayerPositions() const {
    if (socialHandler_) return socialHandler_->getBgPlayerPositions();
    static const std::vector<BgPlayerPosition> empty;
    return empty;
}

bool GameHandler::isLoggingOut() const {
    return socialHandler_ ? socialHandler_->isLoggingOut() : false;
}

float GameHandler::getLogoutCountdown() const {
    return socialHandler_ ? socialHandler_->getLogoutCountdown() : 0.0f;
}

bool GameHandler::isInGuild() const {
    if (socialHandler_) return socialHandler_->isInGuild();
    const Character* ch = getActiveCharacter();
    return ch && ch->hasGuild();
}

bool GameHandler::hasPendingGroupInvite() const {
    return socialHandler_ ? socialHandler_->hasPendingGroupInvite() : pendingGroupInvite;
}
const std::string& GameHandler::getPendingInviterName() const {
    if (socialHandler_) return socialHandler_->getPendingInviterName();
    return pendingInviterName;
}

const std::string& GameHandler::getGuildName() const {
    if (socialHandler_) return socialHandler_->getGuildName();
    static const std::string empty;
    return empty;
}

const GuildRosterData& GameHandler::getGuildRoster() const {
    if (socialHandler_) return socialHandler_->getGuildRoster();
    static const GuildRosterData empty;
    return empty;
}

bool GameHandler::hasGuildRoster() const {
    return socialHandler_ ? socialHandler_->hasGuildRoster() : false;
}

const std::vector<std::string>& GameHandler::getGuildRankNames() const {
    if (socialHandler_) return socialHandler_->getGuildRankNames();
    static const std::vector<std::string> empty;
    return empty;
}

bool GameHandler::hasPendingGuildInvite() const {
    return socialHandler_ ? socialHandler_->hasPendingGuildInvite() : false;
}

const std::string& GameHandler::getPendingGuildInviterName() const {
    if (socialHandler_) return socialHandler_->getPendingGuildInviterName();
    static const std::string empty;
    return empty;
}

const std::string& GameHandler::getPendingGuildInviteGuildName() const {
    if (socialHandler_) return socialHandler_->getPendingGuildInviteGuildName();
    static const std::string empty;
    return empty;
}

const GuildInfoData& GameHandler::getGuildInfoData() const {
    if (socialHandler_) return socialHandler_->getGuildInfoData();
    static const GuildInfoData empty;
    return empty;
}

const GuildQueryResponseData& GameHandler::getGuildQueryData() const {
    if (socialHandler_) return socialHandler_->getGuildQueryData();
    static const GuildQueryResponseData empty;
    return empty;
}

bool GameHandler::hasGuildInfoData() const {
    return socialHandler_ ? socialHandler_->hasGuildInfoData() : false;
}

bool GameHandler::hasPetitionShowlist() const {
    return socialHandler_ ? socialHandler_->hasPetitionShowlist() : false;
}

void GameHandler::clearPetitionDialog() {
    if (socialHandler_) socialHandler_->clearPetitionDialog();
}

void GameHandler::sortArenaTeamRosters(const std::string& key) {
    if (key.empty()) return;
    // The same column twice reverses. A different one starts again in the
    // direction that column reads best: a name from A, a number from the top.
    if (key == arenaSortKey_) {
        arenaSortAscending_ = !arenaSortAscending_;
    } else {
        arenaSortKey_ = key;
        arenaSortAscending_ = (key == "name" || key == "class");
    }
    const bool asc = arenaSortAscending_;

    auto value = [&key](const ArenaTeamMember& m) -> uint64_t {
        if (key == "class")        return m.classId;
        if (key == "played")       return m.weekGames;
        if (key == "won")          return m.weekWins;
        if (key == "seasonplayed") return m.seasonGames;
        if (key == "seasonwon")    return m.seasonWins;
        if (key == "rating")       return m.personalRating;
        return 0;
    };

    for (auto& roster : arenaTeamRosters_) {
        std::stable_sort(roster.members.begin(), roster.members.end(),
            [&](const ArenaTeamMember& a, const ArenaTeamMember& b) {
                if (key == "name") {
                    return asc ? (a.name < b.name) : (b.name < a.name);
                }
                const uint64_t va = value(a), vb = value(b);
                if (va == vb) return a.name < b.name;   // steady under ties
                return asc ? (va < vb) : (vb < va);
            });
    }
}

void GameHandler::requestItemRefundInfo(uint64_t itemGuid) {
    if (itemGuid == 0) return;
    if (getState() != WorldState::IN_WORLD || !getSocket()) return;
    if (!itemRefundAsked_.insert(itemGuid).second) return;   // asked already
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_REFUND_INFO));
    packet.writeUInt64(itemGuid);
    getSocket()->send(packet);
}

void GameHandler::refundItem(uint64_t itemGuid) {
    if (itemGuid == 0) return;
    if (getState() != WorldState::IN_WORLD || !getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_REFUND));
    packet.writeUInt64(itemGuid);
    getSocket()->send(packet);
}

void GameHandler::resolveGMResponse() {
    if (getState() != WorldState::IN_WORLD || !getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GMRESPONSE_RESOLVE));
    getSocket()->send(packet);
}

void GameHandler::renamePetition(uint64_t petitionGuid, const std::string& newName) {
    if (petitionGuid == 0 || newName.empty()) return;
    if (getState() != WorldState::IN_WORLD || !getSocket()) return;
    // MSG_, not CMSG_: the server answers on the same opcode with the name it
    // accepted, which is why the interface does not redraw from its own text.
    network::Packet packet(wireOpcode(Opcode::MSG_PETITION_RENAME));
    packet.writeUInt64(petitionGuid);
    packet.writeString(newName);
    getSocket()->send(packet);
}

void GameHandler::removeGlyphFromSocket(uint32_t slot) {
    if (slot >= MAX_GLYPH_SLOTS) return;
    if (getState() != WorldState::IN_WORLD || !getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_REMOVE_GLYPH));
    packet.writeUInt32(slot);
    getSocket()->send(packet);
}

void GameHandler::acceptArenaTeamInvite() {
    if (socialHandler_) socialHandler_->acceptArenaTeamInvite();
}

void GameHandler::declineArenaTeamInvite() {
    if (socialHandler_) socialHandler_->declineArenaTeamInvite();
}

void GameHandler::arenaTeamInvite(uint32_t teamId, const std::string& name) {
    if (socialHandler_) socialHandler_->arenaTeamInvite(teamId, name);
}
void GameHandler::arenaTeamLeave(uint32_t teamId) {
    if (socialHandler_) socialHandler_->arenaTeamLeave(teamId);
}
void GameHandler::arenaTeamRemove(uint32_t teamId, const std::string& name) {
    if (socialHandler_) socialHandler_->arenaTeamRemove(teamId, name);
}
void GameHandler::arenaTeamSetLeader(uint32_t teamId, const std::string& name) {
    if (socialHandler_) socialHandler_->arenaTeamSetLeader(teamId, name);
}
void GameHandler::disbandArenaTeam(uint32_t teamId) {
    if (socialHandler_) socialHandler_->disbandArenaTeam(teamId);
}

void GameHandler::reportMailSpam(uint64_t senderGuid, uint32_t mailId) {
    if (socialHandler_) socialHandler_->reportMailSpam(senderGuid, mailId);
}

bool GameHandler::getPetitionCharter(int index, uint32_t& itemId,
                                     uint32_t& displayId, uint32_t& cost) const {
    if (!socialHandler_ || index < 0) return false;
    const auto& charters = socialHandler_->getPetitionCharters();
    if (index >= static_cast<int>(charters.size())) return false;
    itemId    = charters[static_cast<size_t>(index)].itemId;
    displayId = charters[static_cast<size_t>(index)].displayId;
    cost      = charters[static_cast<size_t>(index)].cost;
    return true;
}

void GameHandler::closePetitionVendor() {
    if (socialHandler_) socialHandler_->closePetitionVendor();
}

uint32_t GameHandler::getPetitionCost() const {
    return socialHandler_ ? socialHandler_->getPetitionCost() : 0;
}

uint64_t GameHandler::getPetitionNpcGuid() const {
    return socialHandler_ ? socialHandler_->getPetitionNpcGuid() : 0;
}

const GameHandler::PetitionInfo& GameHandler::getPetitionInfo() const {
    if (socialHandler_) return socialHandler_->getPetitionInfo();
    static const PetitionInfo empty;
    return empty;
}

bool GameHandler::hasPetitionSignaturesUI() const {
    return socialHandler_ ? socialHandler_->hasPetitionSignaturesUI() : false;
}

bool GameHandler::hasPendingReadyCheck() const {
    return socialHandler_ ? socialHandler_->hasPendingReadyCheck() : false;
}

void GameHandler::dismissReadyCheck() {
    if (socialHandler_) socialHandler_->dismissReadyCheck();
}

const std::string& GameHandler::getReadyCheckInitiator() const {
    if (socialHandler_) return socialHandler_->getReadyCheckInitiator();
    static const std::string empty;
    return empty;
}

const std::vector<GameHandler::ReadyCheckResult>& GameHandler::getReadyCheckResults() const {
    if (socialHandler_) return socialHandler_->getReadyCheckResults();
    static const std::vector<ReadyCheckResult> empty;
    return empty;
}

uint32_t GameHandler::getInstanceDifficulty() const {
    return socialHandler_ ? socialHandler_->getInstanceDifficulty() : 0;
}

bool GameHandler::isInstanceHeroic() const {
    return socialHandler_ ? socialHandler_->isInstanceHeroic() : false;
}

bool GameHandler::isInInstance() const {
    // Difficulty packets advertise the player's preferred dungeon setting and
    // are also sent in the open world, so they cannot establish instance
    // presence. Map.dbc InstanceType is authoritative: 1=party, 2=raid.
    loadMapNameCache();
    auto it = mapInstanceTypeCache_.find(currentMapId_);
    return it != mapInstanceTypeCache_.end() &&
           (it->second == 1 || it->second == 2);
}

bool GameHandler::hasPendingDuelRequest() const {
    return socialHandler_ ? socialHandler_->hasPendingDuelRequest() : false;
}

const std::string& GameHandler::getDuelChallengerName() const {
    if (socialHandler_) return socialHandler_->getDuelChallengerName();
    static const std::string empty;
    return empty;
}

float GameHandler::getDuelCountdownRemaining() const {
    return socialHandler_ ? socialHandler_->getDuelCountdownRemaining() : 0.0f;
}

const std::vector<GameHandler::InstanceLockout>& GameHandler::getInstanceLockouts() const {
    if (socialHandler_) return socialHandler_->getInstanceLockouts();
    static const std::vector<InstanceLockout> empty;
    return empty;
}

GameHandler::LfgState GameHandler::getLfgState() const {
    return socialHandler_ ? socialHandler_->getLfgState() : LfgState::None;
}

const LfgCompletionReward& GameHandler::getLfgCompletionReward() const {
    static const LfgCompletionReward kNone;
    return socialHandler_ ? socialHandler_->getLfgCompletionReward() : kNone;
}

bool GameHandler::isLfgQueued() const {
    return socialHandler_ ? socialHandler_->isLfgQueued() : false;
}

bool GameHandler::isLfgInDungeon() const {
    return socialHandler_ ? socialHandler_->isLfgInDungeon() : false;
}

uint32_t GameHandler::getLfgDungeonId() const {
    return socialHandler_ ? socialHandler_->getLfgDungeonId() : 0;
}

std::string GameHandler::getCurrentLfgDungeonName() const {
    return socialHandler_ ? socialHandler_->getCurrentLfgDungeonName() : std::string{};
}

uint32_t GameHandler::getLfgProposalId() const {
    return socialHandler_ ? socialHandler_->getLfgProposalId() : 0;
}

uint8_t GameHandler::getLfgOfferedRoles() const {
    return socialHandler_ ? socialHandler_->getLfgOfferedRoles() : 0;
}

int32_t GameHandler::getLfgWaitTank() const   { return socialHandler_ ? socialHandler_->getLfgWaitTank()   : -1; }
int32_t GameHandler::getLfgWaitHealer() const { return socialHandler_ ? socialHandler_->getLfgWaitHealer() : -1; }
int32_t GameHandler::getLfgWaitDps() const    { return socialHandler_ ? socialHandler_->getLfgWaitDps()    : -1; }
uint8_t GameHandler::getLfgNeedTank() const   { return socialHandler_ ? socialHandler_->getLfgNeedTank()   : 0; }
uint8_t GameHandler::getLfgNeedHealer() const { return socialHandler_ ? socialHandler_->getLfgNeedHealer() : 0; }
uint8_t GameHandler::getLfgNeedDps() const    { return socialHandler_ ? socialHandler_->getLfgNeedDps()    : 0; }

const std::vector<LfgProposalMember>& GameHandler::getLfgProposalMembers() const {
    static const std::vector<LfgProposalMember> empty;
    return socialHandler_ ? socialHandler_->getLfgProposalMembers() : empty;
}

const std::unordered_map<uint32_t, uint32_t>& GameHandler::getLfgLocks() const {
    static const std::unordered_map<uint32_t, uint32_t> empty;
    return socialHandler_ ? socialHandler_->getLfgLocks() : empty;
}

const std::vector<LfgReward>& GameHandler::getLfgRewards() const {
    static const std::vector<LfgReward> empty;
    return socialHandler_ ? socialHandler_->getLfgRewards() : empty;
}

int32_t GameHandler::getLfgAvgWaitSec() const {
    return socialHandler_ ? socialHandler_->getLfgAvgWaitSec() : -1;
}

int32_t GameHandler::getLfgMyWaitSec() const {
    return socialHandler_ ? socialHandler_->getLfgMyWaitSec() : -1;
}

uint32_t GameHandler::getLfgQueuedSeconds() const {
    return socialHandler_ ? socialHandler_->getLfgQueuedSeconds() : 0;
}

uint32_t GameHandler::getLfgBootVotes() const {
    return socialHandler_ ? socialHandler_->getLfgBootVotes() : 0;
}

const std::vector<GameHandler::LfgRoleCheckDungeon>&
GameHandler::getLfgRoleCheckDungeons() const {
    static const std::vector<LfgRoleCheckDungeon> empty;
    return socialHandler_ ? socialHandler_->getLfgRoleCheckDungeons() : empty;
}

uint8_t GameHandler::getLfgRoleCheckMembers() const {
    return socialHandler_ ? socialHandler_->getLfgRoleCheckMembers() : 0u;
}

bool GameHandler::isLfgBootInProgress() const {
    return socialHandler_ && socialHandler_->isLfgBootInProgress();
}

bool GameHandler::hasLfgBootVoted() const {
    return socialHandler_ && socialHandler_->hasLfgBootVoted();
}

bool GameHandler::getLfgBootMyVote() const {
    return socialHandler_ && socialHandler_->getLfgBootMyVote();
}

uint32_t GameHandler::getLfgBootTotal() const {
    return socialHandler_ ? socialHandler_->getLfgBootTotal() : 0;
}

uint32_t GameHandler::getLfgBootTimeLeft() const {
    return socialHandler_ ? socialHandler_->getLfgBootTimeLeft() : 0;
}

uint32_t GameHandler::getLfgBootNeeded() const {
    return socialHandler_ ? socialHandler_->getLfgBootNeeded() : 0;
}

const std::string& GameHandler::getLfgBootTargetName() const {
    if (socialHandler_) return socialHandler_->getLfgBootTargetName();
    static const std::string empty;
    return empty;
}

const std::string& GameHandler::getLfgBootReason() const {
    if (socialHandler_) return socialHandler_->getLfgBootReason();
    static const std::string empty;
    return empty;
}

const std::vector<GameHandler::ArenaTeamStats>& GameHandler::getArenaTeamStats() const {
    if (socialHandler_) return socialHandler_->getArenaTeamStats();
    static const std::vector<ArenaTeamStats> empty;
    return empty;
}

// ---- SpellHandler delegating getters ----

int GameHandler::getCraftQueueRemaining() const {
    return spellHandler_ ? spellHandler_->getCraftQueueRemaining() : 0;
}
uint32_t GameHandler::getCraftQueueSpellId() const {
    return spellHandler_ ? spellHandler_->getCraftQueueSpellId() : 0;
}
uint32_t GameHandler::getQueuedSpellId() const {
    return spellHandler_ ? spellHandler_->getQueuedSpellId() : 0;
}
const std::unordered_map<uint32_t, TalentEntry>& GameHandler::getAllTalents() const {
    if (spellHandler_) return spellHandler_->getAllTalents();
    static const std::unordered_map<uint32_t, TalentEntry> empty;
    return empty;
}
const std::unordered_map<uint32_t, TalentTabEntry>& GameHandler::getAllTalentTabs() const {
    if (spellHandler_) return spellHandler_->getAllTalentTabs();
    static const std::unordered_map<uint32_t, TalentTabEntry> empty;
    return empty;
}
float GameHandler::getGCDTotal() const {
    return spellHandler_ ? spellHandler_->getGCDTotal() : 0.0f;
}
bool GameHandler::showTalentWipeConfirmDialog() const {
    return spellHandler_ ? spellHandler_->showTalentWipeConfirmDialog() : false;
}
uint32_t GameHandler::getTalentWipeCost() const {
    return spellHandler_ ? spellHandler_->getTalentWipeCost() : 0;
}
void GameHandler::cancelTalentWipe() {
    if (spellHandler_) spellHandler_->cancelTalentWipe();
}
bool GameHandler::showPetUnlearnDialog() const {
    return spellHandler_ ? spellHandler_->showPetUnlearnDialog() : false;
}
uint32_t GameHandler::getPetUnlearnCost() const {
    return spellHandler_ ? spellHandler_->getPetUnlearnCost() : 0;
}
void GameHandler::cancelPetUnlearn() {
    if (spellHandler_) spellHandler_->cancelPetUnlearn();
}

// ---- QuestHandler delegating getters ----

bool GameHandler::isGossipWindowOpen() const {
    return questHandler_ ? questHandler_->isGossipWindowOpen() : gossipWindowOpen;
}
const GossipMessageData& GameHandler::getCurrentGossip() const {
    if (questHandler_) return questHandler_->getCurrentGossip();
    return currentGossip;
}
int32_t GameHandler::getFactionStanding(uint32_t factionId) const {
    auto it = factionStandings_.find(factionId);
    if (it != factionStandings_.end()) return it->second;
    // Nothing in the by-faction map means nothing has *changed* since login:
    // SMSG_SET_FACTION_STANDING is what writes it, and it only arrives when
    // reputation moves. The standings the character logged in with come in
    // SMSG_INITIALIZE_FACTIONS, indexed by ReputationListID rather than by
    // faction, and the two were never joined - so every faction read zero and
    // the reputation tab drew every bar empty at Neutral, whatever the
    // character had earned, until the next point of reputation with it.
    const uint32_t repListId = getRepListIdByFactionId(factionId);
    if (repListId < initialFactions_.size()) {
        return initialFactions_[repListId].standing;
    }
    return 0;
}

const std::vector<GameHandler::ReputationEntry>& GameHandler::getReputationList() const {
    // Nothing to resolve until the server has sent the standings, and building
    // an empty list once would keep it empty for the session.
    if (reputationListBuilt_ || initialFactions_.empty()) return reputationList_;

    // The reputation index to faction id mapping already exists: the faction
    // name cache reads it out of Faction.dbc and keeps both directions. Reading
    // the file a second time here would be a second opinion about which field
    // holds the index, and the two could disagree.
    // getFactionIdByRepListId loads the cache itself on the first ask, so there
    // is nothing to prime here.
    reputationListBuilt_ = true;

    for (size_t repIndex = 0; repIndex < initialFactions_.size(); ++repIndex) {
        const auto& standing = initialFactions_[repIndex];
        // Hidden factions are never shown, and one the player has not met yet
        // is not visible either.
        if ((standing.flags & FACTION_FLAG_HIDDEN) != 0) continue;
        if ((standing.flags & FACTION_FLAG_VISIBLE) == 0) continue;

        const uint32_t factionId =
            getFactionIdByRepListId(static_cast<uint32_t>(repIndex));
        if (factionId == 0) continue;

        ReputationEntry e;
        e.factionId = factionId;
        e.reputationIndex = static_cast<uint32_t>(repIndex);
        e.name = getFactionNamePublic(factionId);
        e.flags = standing.flags;
        reputationList_.push_back(std::move(e));
    }
    // Walked in the server's own order already, so no sort is needed.
    LOG_INFO("Reputation: ", reputationList_.size(), " visible factions");
    return reputationList_;
}

const std::string& GameHandler::getLanguageName(uint32_t languageId) const {
    static const std::string kUniversal = "Universal";
    static const std::string kNone;
    // Zero is not in the file: it is the "everyone understands this" language,
    // and it is also the name the interface tests for before deciding whether
    // to print a language header at all.
    if (languageId == 0) return kUniversal;
    if (!languageNamesLoaded_) {
        auto* am = services().assetManager;
        if (am && am->isInitialized()) {
            languageNamesLoaded_ = true;
            if (auto dbc = am->loadDBC("Languages.dbc"); dbc && dbc->isLoaded()) {
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    const uint32_t id = dbc->getUInt32(i, 0);
                    std::string name = dbc->getString(i, 1);
                    if (id != 0 && !name.empty()) languageNames_[id] = std::move(name);
                }
                LOG_INFO("Languages.dbc: loaded ", languageNames_.size(), " languages");
            }
        }
    }
    auto it = languageNames_.find(languageId);
    return it != languageNames_.end() ? it->second : kNone;
}

const std::string& GameHandler::getPageTextMaterialName(uint32_t materialId) const {
    static const std::string kNone;
    if (!pageTextMaterialsLoaded_) {
        auto* am = services().assetManager;
        // Not an attempt while the assets are down: marking it loaded here
        // would disable the file for the session, the way the faction and
        // skill caches beside it were once caught doing.
        if (am && am->isInitialized()) {
            pageTextMaterialsLoaded_ = true;
            if (auto dbc = am->loadDBC("PageTextMaterial.dbc"); dbc && dbc->isLoaded()) {
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    const uint32_t id = dbc->getUInt32(i, 0);
                    std::string name = dbc->getString(i, 1);
                    if (id != 0 && !name.empty()) {
                        pageTextMaterialNames_[id] = std::move(name);
                    }
                }
                LOG_INFO("PageTextMaterial.dbc: loaded ",
                         pageTextMaterialNames_.size(), " page materials");
            }
        }
    }
    auto it = pageTextMaterialNames_.find(materialId);
    return it != pageTextMaterialNames_.end() ? it->second : kNone;
}

uint32_t GameHandler::getFactionParentId(uint32_t factionId) const {
    loadFactionNameCache();
    auto it = factionParent_.find(factionId);
    return (it != factionParent_.end()) ? it->second : 0u;
}

void GameHandler::setFactionCollapsed(uint32_t factionId, bool collapsed) {
    if (factionId == 0) return;
    const bool changed = collapsed ? collapsedFactionIds_.insert(factionId).second
                                   : collapsedFactionIds_.erase(factionId) > 0;
    // Rebuilding on a no-op would be harmless but the redraw is not: the
    // reputation panel reads its scroll position back from the row count.
    if (changed) reputationRowsDirty_ = true;
}

const std::vector<GameHandler::ReputationRow>& GameHandler::getReputationRows() const {
    const auto& flat = getReputationList();
    if (!reputationRowsDirty_) return reputationRows_;
    reputationRows_.clear();
    // An empty list means the standings have not arrived, not that there are
    // none - latching that would leave the panel empty for the session.
    if (flat.empty()) return reputationRows_;
    reputationRowsDirty_ = false;

    // Everything drawn: the visible factions, plus every faction one of them
    // descends from. A group whose members the player has not met is not drawn
    // at all, so its heading is not either.
    std::unordered_map<uint32_t, const ReputationEntry*> byId;
    std::unordered_set<uint32_t> displayed;
    for (const auto& e : flat) { byId[e.factionId] = &e; displayed.insert(e.factionId); }
    for (const auto& e : flat) {
        // Faction.dbc is data, and a cycle in it would spin here. The real
        // chain is three deep, so a small ceiling costs nothing.
        uint32_t p = getFactionParentId(e.factionId);
        for (int depth = 0; p != 0 && depth < 8; ++depth) {
            displayed.insert(p);
            p = getFactionParentId(p);
        }
    }

    // Where each group sits: at its first member's place in the server's list.
    // Ordering by the heading's own repListId instead would scatter the groups,
    // since Classic is 96 while the factions inside it start at 10.
    constexpr size_t kNoRank = static_cast<size_t>(-1);
    std::unordered_map<uint32_t, size_t> rank;
    for (uint32_t id : displayed) rank[id] = kNoRank;
    for (size_t i = 0; i < flat.size(); ++i) {
        uint32_t id = flat[i].factionId;
        for (int depth = 0; id != 0 && depth < 9; ++depth) {
            auto it = rank.find(id);
            if (it != rank.end() && i < it->second) it->second = i;
            id = getFactionParentId(id);
        }
    }

    // The tree, with a parent outside the drawn set counting as no parent.
    std::unordered_map<uint32_t, std::vector<uint32_t>> children;
    for (uint32_t id : displayed) {
        const uint32_t parent = getFactionParentId(id);
        children[displayed.count(parent) ? parent : 0u].push_back(id);
    }
    const auto byRank = [&](uint32_t a, uint32_t b) {
        if (rank[a] != rank[b]) return rank[a] < rank[b];
        return getFactionNamePublic(a) < getFactionNamePublic(b);
    };
    for (auto& [parent, kids] : children) {
        (void)parent;
        std::sort(kids.begin(), kids.end(), byRank);
    }

    // Depth first, so a heading is immediately followed by what is under it.
    // FrameXML draws a flat list and takes the nesting from isHeader/isChild,
    // which means the order *is* the grouping: emitting in the server's order
    // and relying on the flags would file each faction under whichever heading
    // happened to be printed last.
    std::vector<std::pair<uint32_t, int>> stack;
    for (uint32_t child : std::views::reverse(children[0])) {
        stack.emplace_back(child, 0);
    }
    while (!stack.empty()) {
        const auto [factionId, depth] = stack.back();
        stack.pop_back();

        auto kidsIt = children.find(factionId);
        const bool isHeader = kidsIt != children.end() && !kidsIt->second.empty();

        ReputationRow row;
        row.factionId = factionId;
        row.isHeader = isHeader;
        // Only two levels are drawn: FrameXML knows a heading and an indented
        // row and nothing deeper, so everything below the top shares one indent.
        row.isChild = depth > 0;
        auto entry = byId.find(factionId);
        if (entry != byId.end()) {
            row.reputationIndex = entry->second->reputationIndex;
            row.name = entry->second->name;
            row.flags = entry->second->flags;
            row.hasRep = true;
        } else {
            // A heading the player has no standing with - Classic and the two
            // expansion groups are the usual ones. Drawn as a bare heading.
            row.name = getFactionNamePublic(factionId);
            row.hasRep = false;
        }
        reputationRows_.push_back(std::move(row));

        // A closed group keeps its heading and loses everything under it: the
        // panel counts rows and indexes into them, so a hidden row must not be
        // in the list at all.
        if (!isHeader || isFactionCollapsed(factionId)) continue;
        for (uint32_t child : std::views::reverse(kidsIt->second)) {
            stack.emplace_back(child, depth + 1);
        }
    }
    return reputationRows_;
}

const GameHandler::ContinentBounds&
GameHandler::getContinentBounds(uint32_t mapId) const {
    auto cached = continentBoundsCache_.find(mapId);
    if (cached != continentBoundsCache_.end()) return cached->second;

    ContinentBounds bounds;
    auto* am = services_.assetManager;
    if (am && am->isInitialized()) {
        if (auto dbc = am->loadDBC("WorldMapArea.dbc"); dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                ? pipeline::getActiveDBCLayout()->getLayout("WorldMapArea") : nullptr;
            const auto field = [&](const char* name, uint32_t fallback) {
                return layout ? (*layout)[name] : fallback;
            };
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                if (dbc->getUInt32(i, field("MapID", 1)) != mapId) continue;
                // The continent's own row, not one of its zones.
                if (dbc->getUInt32(i, field("AreaID", 2)) != 0) continue;
                bounds.left   = dbc->getFloat(i, field("LocLeft", 4));
                bounds.right  = dbc->getFloat(i, field("LocRight", 5));
                bounds.top    = dbc->getFloat(i, field("LocTop", 6));
                bounds.bottom = dbc->getFloat(i, field("LocBottom", 7));
                // A degenerate rectangle projects everything to one point, so
                // it is no more usable than a missing row.
                bounds.valid = std::abs(bounds.left - bounds.right) > 0.001f &&
                               std::abs(bounds.top - bounds.bottom) > 0.001f;
                break;
            }
        }
    }
    if (!bounds.valid) {
        LOG_WARNING("No continent-wide WorldMapArea row for map ", mapId,
                    " - anything projecting onto its map will have nothing to "
                    "project against");
    }
    return continentBoundsCache_.emplace(mapId, bounds).first->second;
}

const GameHandler::ExtendedCostEntry*
GameHandler::getExtendedCost(uint32_t extendedCostId) const {
    if (!extendedCostCacheLoaded_) {
        extendedCostCacheLoaded_ = true;
        auto* am = services_.assetManager;
        if (am && am->isInitialized()) {
            if (auto dbc = am->loadDBC("ItemExtendedCost.dbc"); dbc && dbc->isLoaded()) {
                // WotLK layout: 0=ID, 1=honorPoints, 2=arenaPoints,
                // 3=arenaSlotRestrictions, 4-8=itemId[5], 9-13=itemCount[5],
                // 14=reqRating, 15=purchaseGroup
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    const uint32_t id = dbc->getUInt32(i, 0);
                    if (id == 0) continue;
                    ExtendedCostEntry e;
                    e.honorPoints = dbc->getUInt32(i, 1);
                    e.arenaPoints = dbc->getUInt32(i, 2);
                    for (int j = 0; j < 5; ++j) {
                        e.itemId[j]    = dbc->getUInt32(i, 4 + j);
                        e.itemCount[j] = dbc->getUInt32(i, 9 + j);
                    }
                    extendedCostCache_[id] = e;
                }
                LOG_INFO("ItemExtendedCost.dbc: loaded ", extendedCostCache_.size(),
                         " entries");
            }
        }
    }
    auto it = extendedCostCache_.find(extendedCostId);
    return it == extendedCostCache_.end() ? nullptr : &it->second;
}

const std::string& GameHandler::getQuestGreeting() const {
    static const std::string empty;
    if (questHandler_) return questHandler_->getQuestGreeting();
    return empty;
}
const std::string& GameHandler::getNpcText(uint32_t textId) const {
    static const std::string empty;
    if (questHandler_) return questHandler_->getNpcText(textId);
    return empty;
}
bool GameHandler::isQuestDetailsOpen() {
    if (questHandler_) return questHandler_->isQuestDetailsOpen();
    return questDetailsOpen;
}
bool GameHandler::questDetailsItemInfoReady() const {
    if (questHandler_) return questHandler_->questDetailsItemInfoReady();
    return true;
}
const QuestDetailsData& GameHandler::getQuestDetails() const {
    if (questHandler_) return questHandler_->getQuestDetails();
    return currentQuestDetails;
}

const std::vector<GossipPoi>& GameHandler::getGossipPois() const {
    if (questHandler_) return questHandler_->getGossipPois();
    static const std::vector<GossipPoi> empty;
    return empty;
}

void GameHandler::clearGossipPois() {
    if (questHandler_) questHandler_->clearGossipPois();
}
const std::unordered_map<uint64_t, QuestGiverStatus>& GameHandler::getNpcQuestStatuses() const {
    if (questHandler_) return questHandler_->getNpcQuestStatuses();
    static const std::unordered_map<uint64_t, QuestGiverStatus> empty;
    return empty;
}
QuestGiverStatus GameHandler::getQuestGiverStatus(uint64_t guid) const {
    if (questHandler_) return questHandler_->getQuestGiverStatus(guid);
    return QuestGiverStatus::NONE;
}
std::vector<std::pair<uint32_t, uint32_t>> GameHandler::getQuestTimers() const {
    if (questHandler_) return questHandler_->getQuestTimers();
    return {};
}

const std::vector<GameHandler::QuestLogEntry>& GameHandler::getQuestLog() const {
    if (questHandler_) return questHandler_->getQuestLog();
    static const std::vector<QuestLogEntry> empty;
    return empty;
}

void GameHandler::reconcileQuestItemObjectives(
    const std::unordered_map<uint32_t, uint32_t>& carriedCounts) {
    if (questHandler_) questHandler_->reconcileItemObjectivesFromInventory(carriedCounts);
}
const std::string& GameHandler::getQuestSortName(uint32_t sortId) const {
    static const std::string empty;
    return questHandler_ ? questHandler_->getQuestSortName(sortId) : empty;
}
bool GameHandler::isQuestOfferRewardOpen() const {
    return questHandler_ ? questHandler_->isQuestOfferRewardOpen() : false;
}
const QuestOfferRewardData& GameHandler::getQuestOfferReward() const {
    if (questHandler_) return questHandler_->getQuestOfferReward();
    static const QuestOfferRewardData empty;
    return empty;
}
bool GameHandler::isQuestRequestItemsOpen() const {
    return questHandler_ ? questHandler_->isQuestRequestItemsOpen() : false;
}
const QuestRequestItemsData& GameHandler::getQuestRequestItems() const {
    if (questHandler_) return questHandler_->getQuestRequestItems();
    static const QuestRequestItemsData empty;
    return empty;
}
int GameHandler::getSelectedQuestLogIndex() const {
    return questHandler_ ? questHandler_->getSelectedQuestLogIndex() : 0;
}
void GameHandler::setSelectedQuestLogIndex(int idx) {
    if (questHandler_) questHandler_->setSelectedQuestLogIndex(idx);
}
uint32_t GameHandler::getSharedQuestId() const {
    return questHandler_ ? questHandler_->getSharedQuestId() : 0;
}
const std::string& GameHandler::getSharedQuestSharerName() const {
    if (questHandler_) return questHandler_->getSharedQuestSharerName();
    static const std::string empty;
    return empty;
}
const std::string& GameHandler::getSharedQuestTitle() const {
    if (questHandler_) return questHandler_->getSharedQuestTitle();
    static const std::string empty;
    return empty;
}
const std::unordered_set<uint32_t>& GameHandler::getTrackedQuestIds() const {
    return trackedQuestIds_;
}
bool GameHandler::hasPendingSharedQuest() const {
    return questHandler_ ? questHandler_->hasPendingSharedQuest() : false;
}

// ---- MovementHandler delegating getters ----

float GameHandler::getServerRunSpeed() const {
    return movementHandler_ ? movementHandler_->getServerRunSpeed() : 7.0f;
}
float GameHandler::getServerWalkSpeed() const {
    return movementHandler_ ? movementHandler_->getServerWalkSpeed() : 2.5f;
}
float GameHandler::getServerSwimSpeed() const {
    return movementHandler_ ? movementHandler_->getServerSwimSpeed() : 4.722f;
}
float GameHandler::getServerSwimBackSpeed() const {
    return movementHandler_ ? movementHandler_->getServerSwimBackSpeed() : 2.5f;
}
float GameHandler::getServerFlightSpeed() const {
    return movementHandler_ ? movementHandler_->getServerFlightSpeed() : 7.0f;
}
float GameHandler::getServerFlightBackSpeed() const {
    return movementHandler_ ? movementHandler_->getServerFlightBackSpeed() : 4.5f;
}
float GameHandler::getServerRunBackSpeed() const {
    return movementHandler_ ? movementHandler_->getServerRunBackSpeed() : 4.5f;
}
float GameHandler::getServerTurnRate() const {
    return movementHandler_ ? movementHandler_->getServerTurnRate() : core::coords::PI;
}
bool GameHandler::isTaxiWindowOpen() const {
    return movementHandler_ ? movementHandler_->isTaxiWindowOpen() : false;
}
bool GameHandler::isOnTaxiFlight() const {
    return movementHandler_ ? movementHandler_->isOnTaxiFlight() : false;
}
bool GameHandler::isTaxiMountActive() const {
    return movementHandler_ ? movementHandler_->isTaxiMountActive() : false;
}
bool GameHandler::isTaxiActivationPending() const {
    return movementHandler_ ? movementHandler_->isTaxiActivationPending() : false;
}
const std::string& GameHandler::getTaxiDestName() const {
    if (movementHandler_) return movementHandler_->getTaxiDestName();
    static const std::string empty;
    return empty;
}
const ShowTaxiNodesData& GameHandler::getTaxiData() const {
    if (movementHandler_) return movementHandler_->getTaxiData();
    static const ShowTaxiNodesData empty;
    return empty;
}
uint32_t GameHandler::getTaxiCurrentNode() const {
    if (movementHandler_) return movementHandler_->getTaxiData().nearestNode;
    return 0;
}
const std::unordered_map<uint32_t, GameHandler::TaxiNode>& GameHandler::getTaxiNodes() const {
    if (movementHandler_) return movementHandler_->getTaxiNodes();
    static const std::unordered_map<uint32_t, TaxiNode> empty;
    return empty;
}
bool GameHandler::isKnownTaxiNode(uint32_t nodeId) const {
    // Was reading a GameHandler-local knownTaxiMask_ that nothing ever wrote
    // to (handleShowTaxiNodes only updates MovementHandler's own copy), so
    // this always returned false - broke the world map's discovered-node
    // display and the Lua IsTaxiNodeKnown-equivalent. Delegate like the
    // other taxi accessors above.
    return movementHandler_ && movementHandler_->isKnownTaxiNode(nodeId);
}

} // namespace game
} // namespace wowee
