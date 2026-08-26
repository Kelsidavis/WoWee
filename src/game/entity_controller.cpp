#include "game/entity_controller.hpp"
#include "core/env_flag.hpp"
#include "game/game_handler.hpp"
#include "game/protocol_constants.hpp"
#include "game/game_utils.hpp"
#include "game/packet_parsers.hpp"
#include "game/entity.hpp"
#include "game/update_field_table.hpp"
#include "game/opcode_table.hpp"
#include "game/chat_handler.hpp"
#include "game/transport_manager.hpp"
#include "game/movement_handler.hpp"
#include "core/logger.hpp"
#include "core/coordinates.hpp"
#include "network/world_socket.hpp"
#include "rendering/animation_controller.hpp"
#include <algorithm>
#include <bit>
#include <cstring>
#include <zlib.h>

namespace wowee {
namespace game {

namespace {


int updateObjectBlocksBudgetPerUpdate(WorldState state) {
    static const int inWorldBudget =
        core::envIntClamped("WOWEE_NET_MAX_UPDATE_OBJECT_BLOCKS", 24, 1, 2048);
    static const int loginBudget =
        core::envIntClamped("WOWEE_NET_MAX_UPDATE_OBJECT_BLOCKS_LOGIN", 128, 1, 4096);
    return state == WorldState::IN_WORLD ? inWorldBudget : loginBudget;
}

float slowUpdateObjectBlockLogThresholdMs() {
    static const int thresholdMs =
        core::envIntClamped("WOWEE_NET_SLOW_UPDATE_BLOCK_LOG_MS", 10, 1, 60000);
    return static_cast<float>(thresholdMs);
}

} // anonymous namespace

EntityController::EntityController(GameHandler& owner)
    : owner_(owner) { initTypeHandlers(); }

void EntityController::registerOpcodes(DispatchTable& table) {
    // World object updates - accept during ENTERING_WORLD too so that entity
    // packets arriving before SMSG_LOGIN_VERIFY_WORLD are parsed and queued
    // rather than silently dropped (the budget system processes them later once
    // the state transitions to IN_WORLD).
    auto inWorldOrEntering = [this]() {
        auto s = owner_.getState();
        return s == WorldState::IN_WORLD || s == WorldState::ENTERING_WORLD;
    };
    table[Opcode::SMSG_UPDATE_OBJECT] = [this, inWorldOrEntering](network::Packet& packet) {
        LOG_DEBUG("Received SMSG_UPDATE_OBJECT, state=", static_cast<int>(owner_.getState()), " size=", packet.getSize());
        if (inWorldOrEntering()) handleUpdateObject(packet);
    };
    table[Opcode::SMSG_COMPRESSED_UPDATE_OBJECT] = [this, inWorldOrEntering](network::Packet& packet) {
        LOG_DEBUG("Received SMSG_COMPRESSED_UPDATE_OBJECT, state=", static_cast<int>(owner_.getState()), " size=", packet.getSize());
        if (inWorldOrEntering()) handleCompressedUpdateObject(packet);
    };
    table[Opcode::SMSG_DESTROY_OBJECT] = [this, inWorldOrEntering](network::Packet& packet) {
        if (inWorldOrEntering()) handleDestroyObject(packet);
    };

    // Entity queries
    table[Opcode::SMSG_NAME_QUERY_RESPONSE] = [this](network::Packet& packet) {
        handleNameQueryResponse(packet);
    };
    table[Opcode::SMSG_CREATURE_QUERY_RESPONSE] = [this](network::Packet& packet) {
        handleCreatureQueryResponse(packet);
    };
    table[Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE] = [this](network::Packet& packet) {
        handleGameObjectQueryResponse(packet);
    };
    table[Opcode::SMSG_GAMEOBJECT_PAGETEXT] = [this](network::Packet& packet) {
        handleGameObjectPageText(packet);
    };
    table[Opcode::SMSG_PAGE_TEXT_QUERY_RESPONSE] = [this](network::Packet& packet) {
        handlePageTextQueryResponse(packet);
    };
}

void EntityController::clearAll() {
    pendingUpdateObjectWork_.clear();
    playerNameCache.clear();
    playerClassRaceCache_.clear();
    pendingNameQueries.clear();
    creatureInfoCache.clear();
    pendingCreatureQueries.clear();
    gameObjectInfoCache_.clear();
    pendingGameObjectQueries_.clear();
    transportGuids_.clear();
    serverUpdatedTransportGuids_.clear();
    entityManager.clear();
}

// ============================================================
// Update Object Pipeline
// ============================================================

void EntityController::enqueueUpdateObjectWork(UpdateObjectData&& data) {
    pendingUpdateObjectWork_.push_back(PendingUpdateObjectWork{.data = std::move(data)});
}
void EntityController::processPendingUpdateObjectWork(const std::chrono::steady_clock::time_point& start,
                                                 float budgetMs) {
    if (pendingUpdateObjectWork_.empty()) {
        return;
    }

    const int maxBlocksThisUpdate = updateObjectBlocksBudgetPerUpdate(owner_.getState());
    int processedBlocks = 0;

    while (!pendingUpdateObjectWork_.empty() && processedBlocks < maxBlocksThisUpdate) {
        float elapsedMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsedMs >= budgetMs) {
            break;
        }

        auto& work = pendingUpdateObjectWork_.front();
        if (!work.outOfRangeProcessed) {
            auto outOfRangeStart = std::chrono::steady_clock::now();
            processOutOfRangeObjects(work.data.outOfRangeGuids);
            float outOfRangeMs = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - outOfRangeStart).count();
            if (outOfRangeMs > slowUpdateObjectBlockLogThresholdMs()) {
                LOG_WARNING("SLOW update-object out-of-range handling: ", outOfRangeMs,
                            "ms guidCount=", work.data.outOfRangeGuids.size());
            }
            work.outOfRangeProcessed = true;
        }

        while (work.nextBlockIndex < work.data.blocks.size() && processedBlocks < maxBlocksThisUpdate) {
            elapsedMs = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsedMs >= budgetMs) {
                break;
            }

            const UpdateBlock& block = work.data.blocks[work.nextBlockIndex];
            auto blockStart = std::chrono::steady_clock::now();
            applyUpdateObjectBlock(block, work.newItemCreated);
            float blockMs = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - blockStart).count();
            if (blockMs > slowUpdateObjectBlockLogThresholdMs()) {
                LOG_WARNING("SLOW update-object block apply: ", blockMs,
                            "ms index=", work.nextBlockIndex,
                            " type=", static_cast<int>(block.updateType),
                            " guid=0x", std::hex, block.guid, std::dec,
                            " objectType=", static_cast<int>(block.objectType),
                            " fieldCount=", block.fields.size(),
                            " hasMovement=", block.hasMovement ? 1 : 0);
            }
            ++work.nextBlockIndex;
            ++processedBlocks;
        }

        if (work.nextBlockIndex >= work.data.blocks.size()) {
            finalizeUpdateObjectBatch(work.newItemCreated);
            pendingUpdateObjectWork_.pop_front();
            continue;
        }
        break;
    }

    if (!pendingUpdateObjectWork_.empty()) {
        const auto& work = pendingUpdateObjectWork_.front();
        LOG_DEBUG("GameHandler update-object budget reached (remainingBatches=",
                  pendingUpdateObjectWork_.size(), ", nextBlockIndex=", work.nextBlockIndex,
                  "/", work.data.blocks.size(), ", owner_.getState()=", worldStateName(owner_.getState()), ")");
    }
}
void EntityController::handleUpdateObject(network::Packet& packet) {
    UpdateObjectData data;
    if (!owner_.getPacketParsers()->parseUpdateObject(packet, data)) {
        static int updateObjErrors = 0;
        if (++updateObjErrors <= 5)
            LOG_WARNING("Failed to parse SMSG_UPDATE_OBJECT");
        if (data.blocks.empty()) return;
        // Fall through: process any blocks that were successfully parsed before the failure.
    }

    enqueueUpdateObjectWork(std::move(data));
}

void EntityController::processOutOfRangeObjects(const std::vector<uint64_t>& guids) {
    // Process out-of-range objects first
    bool inventoryChanged = false;
    for (uint64_t guid : guids) {
        // An item leaving is how the server says it is gone. Selling one, or
        // handing it to a quest, takes it out of range rather than destroying
        // it, and handleDestroyObject is the only path that was clearing the
        // online item tracking - so a sold item stayed in the picture the bags
        // are drawn from and went on being drawn in the slot it left.
        //
        // Before the entity check, because an item need not be in the entity
        // manager at all: the partial updates that carry them are tracked in
        // onlineItems_ alone, which is why the values path has a branch for
        // exactly that case.
        owner_.containerContentsRef().erase(guid);
        if (owner_.onlineItemsRef().erase(guid)) inventoryChanged = true;

        auto entity = entityManager.getEntity(guid);
        if (!entity) continue;

        const bool isKnownTransport = transportGuids_.count(guid) > 0;
        if (isKnownTransport) {
            // Keep transports alive across out-of-range flapping.
            // Boats/zeppelins are global movers and removing them here can make
            // them disappear until a later movement snapshot happens to recreate them.
            const bool playerAboardNow = (owner_.playerTransportGuidRef() == guid);
            const bool stickyAboard = (owner_.playerTransportStickyGuidRef() == guid && owner_.playerTransportStickyTimerRef() > 0.0f);
            const bool movementSaysAboard = (owner_.movementInfoRef().transportGuid == guid);
            LOG_INFO("Preserving transport on out-of-range: 0x",
                     std::hex, guid, std::dec,
                     " now=", playerAboardNow,
                     " sticky=", stickyAboard,
                     " movement=", movementSaysAboard);
            continue;
        }

        LOG_DEBUG("Entity went out of range: 0x", std::hex, guid, std::dec);
        // Trigger despawn callbacks before removing entity
        if (entity->getType() == ObjectType::UNIT && owner_.creatureDespawnCallbackRef()) {
            owner_.creatureDespawnCallbackRef()(guid);
        } else if (entity->getType() == ObjectType::PLAYER && owner_.playerDespawnCallbackRef()) {
            owner_.playerDespawnCallbackRef()(guid);
            owner_.otherPlayerVisibleItemEntriesRef().erase(guid);
            owner_.otherPlayerVisibleDirtyRef().erase(guid);
            owner_.otherPlayerMoveTimeMsRef().erase(guid);
            owner_.inspectedPlayerItemEntriesRef().erase(guid);
            owner_.pendingAutoInspectRef().erase(guid);
            // Clear pending name query so the query is re-sent when this player
            // comes back into range (entity is recreated as a new object).
            pendingNameQueries.erase(guid);
        } else if (entity->getType() == ObjectType::GAMEOBJECT && owner_.gameObjectDespawnCallbackRef()) {
            owner_.gameObjectDespawnCallbackRef()(guid);
        } else if (entity->getType() == ObjectType::CORPSE && owner_.playerDespawnCallbackRef()) {
            owner_.playerDespawnCallbackRef()(guid);
        }
        transportGuids_.erase(guid);
        serverUpdatedTransportGuids_.erase(guid);
        owner_.clearTransportAttachment(guid);
        if (owner_.playerTransportGuidRef() == guid) {
            owner_.clearPlayerTransport();
        }
        entityManager.removeEntity(guid);
    }

    // Once, after the whole batch: a vendor sale can take several items out of
    // range together, and rebuilding per item would redraw the bags as many
    // times for one answer.
    if (inventoryChanged) owner_.rebuildOnlineInventory();
}

// ============================================================
// Extracted helper methods
// ============================================================

bool EntityController::getPlayerAppearance(uint64_t guid, uint8_t& outRace,
                                           uint8_t& outGender,
                                           uint32_t& outAppearanceBytes,
                                           uint8_t& outFacial) const {
    auto entity = const_cast<EntityController*>(this)->getEntityManager().getEntity(guid);
    if (!entity) return false;
    // Players only, and this is not a formality. extractPlayerAppearance falls
    // back to scanning the fields for something that *looks* like packed
    // appearance when the named ones are absent, and the test is loose - any
    // field whose four bytes are small enough. On a creature it matches almost
    // immediately, and the answer is race zero with somebody's health for hair.
    //
    // That fallback is safe where it was written, because the spawn path only
    // reaches it after deciding the object is a player. This is a second
    // caller, and it has to decide the same thing for itself.
    if (entity->getType() != ObjectType::PLAYER) return false;
    return extractPlayerAppearance(entity->getFields(), outRace, outGender,
                                   outAppearanceBytes, outFacial);
}

bool EntityController::extractPlayerAppearance(const FlatFieldMap& fields,
                                               uint8_t& outRace,
                                               uint8_t& outGender,
                                               uint32_t& outAppearanceBytes,
                                               uint8_t& outFacial) const {
    outRace = 0;
    outGender = 0;
    outAppearanceBytes = 0;
    outFacial = 0;

    auto readField = [&](uint16_t idx, uint32_t& out) -> bool {
        if (idx == 0xFFFF) return false;
        auto it = fields.find(idx);
        if (it == fields.end()) return false;
        out = it->second;
        return true;
    };

    uint32_t bytes0 = 0;
    uint32_t pbytes = 0;
    uint32_t pbytes2 = 0;

    const uint16_t ufBytes0 = fieldIndex(UF::UNIT_FIELD_BYTES_0);
    const uint16_t ufPbytes = fieldIndex(UF::PLAYER_BYTES);
    const uint16_t ufPbytes2 = fieldIndex(UF::PLAYER_BYTES_2);

    bool haveBytes0 = readField(ufBytes0, bytes0);
    bool havePbytes = readField(ufPbytes, pbytes);
    bool havePbytes2 = readField(ufPbytes2, pbytes2);

    // Heuristic fallback: Turtle can run with unusual build numbers; if the JSON table is missing,
    // try to locate plausible packed fields by scanning.
    if (!haveBytes0) {
        for (const auto& [idx, v] : fields) {
            uint8_t race = static_cast<uint8_t>(v & 0xFF);
            uint8_t cls = static_cast<uint8_t>((v >> 8) & 0xFF);
            uint8_t gender = static_cast<uint8_t>((v >> 16) & 0xFF);
            uint8_t power = static_cast<uint8_t>((v >> 24) & 0xFF);
            if (race >= 1 && race <= 20 &&
                cls >= 1 && cls <= 20 &&
                gender <= 1 &&
                power <= 10) {
                bytes0 = v;
                haveBytes0 = true;
                break;
            }
        }
    }
    if (!havePbytes) {
        for (const auto& [idx, v] : fields) {
            uint8_t skin = static_cast<uint8_t>(v & 0xFF);
            uint8_t face = static_cast<uint8_t>((v >> 8) & 0xFF);
            uint8_t hair = static_cast<uint8_t>((v >> 16) & 0xFF);
            uint8_t color = static_cast<uint8_t>((v >> 24) & 0xFF);
            if (skin <= 50 && face <= 50 && hair <= 100 && color <= 50) {
                pbytes = v;
                havePbytes = true;
                break;
            }
        }
    }
    if (!havePbytes2) {
        for (const auto& [idx, v] : fields) {
            uint8_t facial = static_cast<uint8_t>(v & 0xFF);
            if (facial <= 100) {
                pbytes2 = v;
                havePbytes2 = true;
                break;
            }
        }
    }

    if (!haveBytes0 || !havePbytes) return false;

    outRace = static_cast<uint8_t>(bytes0 & 0xFF);
    outGender = static_cast<uint8_t>((bytes0 >> 16) & 0xFF);
    outAppearanceBytes = pbytes;
    outFacial = havePbytes2 ? static_cast<uint8_t>(pbytes2 & 0xFF) : 0;
    return true;
}

void EntityController::maybeDetectCoinageIndex(const FlatFieldMap& oldFields,
                                               const FlatFieldMap& newFields) {
    if (owner_.pendingMoneyDeltaRef() == 0 || owner_.pendingMoneyDeltaTimerRef() <= 0.0f) return;
    if (oldFields.empty() || newFields.empty()) return;

    constexpr uint32_t kMaxPlausibleCoinage = 2147483647u;
    std::vector<uint16_t> candidates;
    candidates.reserve(8);

    for (const auto& [idx, newVal] : newFields) {
        auto itOld = oldFields.find(idx);
        if (itOld == oldFields.end()) continue;
        uint32_t oldVal = itOld->second;
        if (newVal < oldVal) continue;
        uint32_t delta = newVal - oldVal;
        if (delta != owner_.pendingMoneyDeltaRef()) continue;
        if (newVal > kMaxPlausibleCoinage) continue;
        candidates.push_back(idx);
    }

    if (candidates.empty()) return;

    uint16_t current = fieldIndex(UF::PLAYER_FIELD_COINAGE);
    uint16_t chosen = candidates[0];
    if (std::find(candidates.begin(), candidates.end(), current) != candidates.end()) {
        chosen = current;
    } else {
        std::sort(candidates.begin(), candidates.end());
        chosen = candidates[0];
    }

    if (chosen != current && current != 0xFFFF) {
        owner_.updateFieldTableRef().setIndex(UF::PLAYER_FIELD_COINAGE, chosen);
        LOG_WARNING("Auto-detected PLAYER_FIELD_COINAGE index: ", chosen, " (was ", current, ")");
    }

    owner_.pendingMoneyDeltaRef() = 0;
    owner_.pendingMoneyDeltaTimerRef() = 0.0f;
}

// ============================================================
// Update type dispatch
// ============================================================

void EntityController::applyUpdateObjectBlock(const UpdateBlock& block, bool& newItemCreated) {
    switch (block.updateType) {
        case UpdateType::CREATE_OBJECT:
        case UpdateType::CREATE_OBJECT2:
            handleCreateObject(block, newItemCreated);
            break;
        case UpdateType::VALUES:
            handleValuesUpdate(block);
            break;
        case UpdateType::MOVEMENT:
            handleMovementUpdate(block);
            break;
        default:
            break;
    }
}

// ============================================================
// Concern-specific helpers
// ============================================================

// Non-player transport child attachment - identical in CREATE/VALUES/MOVEMENT
void EntityController::updateNonPlayerTransportAttachment(const UpdateBlock& block,
                                                           const std::shared_ptr<Entity>& entity,
                                                           ObjectType entityType) {
    if (block.guid == owner_.getPlayerGuid()) return;
    if (entityType != ObjectType::UNIT && entityType != ObjectType::GAMEOBJECT) return;

    if (block.onTransport && block.transportGuid != 0) {
        const glm::vec3 serverOffset(block.transportX, block.transportY, block.transportZ);
        const bool transportResolved = owner_.getTransportManager() &&
            owner_.getTransportManager()->getTransport(block.transportGuid);
        // Preserve the raw wire offset if the child arrives before its parent
        // transport during a map load. Its coordinate convention depends on
        // whether that parent is an M2 or WMO and can only be decided once the
        // transport has registered.
        glm::vec3 localOffset = transportResolved
            ? owner_.getTransportManager()->serverToTransportLocal(block.transportGuid, serverOffset)
            : serverOffset;
        const bool hasLocalOrientation = (block.updateFlags & 0x0020) != 0; // UPDATEFLAG_LIVING
        float localOriCanonical = core::coords::normalizeAngleRad(-block.transportO);
        owner_.setTransportAttachment(block.guid, entityType, block.transportGuid,
                               localOffset, hasLocalOrientation, localOriCanonical,
                               !transportResolved);
        if (owner_.getTransportManager() && owner_.getTransportManager()->getTransport(block.transportGuid)) {
            glm::vec3 composed = owner_.getTransportManager()->getPlayerWorldPosition(block.transportGuid, localOffset);
            entity->setPosition(composed.x, composed.y, composed.z, entity->getOrientation());
        }
    } else {
        owner_.clearTransportAttachment(block.guid);
    }
}

//     Rebuild playerAuras from UNIT_FIELD_AURAS (Classic/TBC-era clients).
//     blockFields is used to check if any aura field was updated in this packet.
//     entity->getFields() is used for reading the full accumulated state.
//     Normalises pre-WotLK harmful bit (0x02) to WotLK debuff bit (0x80) so
//     downstream code checking for 0x80 works consistently across expansions.
void EntityController::syncPreWotlkAurasFromFields(const std::shared_ptr<Entity>& entity) {
    if (!isPreWotlk() || !owner_.getSpellHandler()) return;

    const uint16_t ufAuras     = fieldIndex(UF::UNIT_FIELD_AURAS);
    const uint16_t ufAuraFlags = fieldIndex(UF::UNIT_FIELD_AURAFLAGS);
    if (ufAuras == 0xFFFF) return;

    const auto& allFields = entity->getFields();
    bool hasAuraField = false;
    for (const auto& [fk, fv] : allFields) {
        if (fk >= ufAuras && fk < ufAuras + 48) { hasAuraField = true; break; }
    }
    if (!hasAuraField) return;

    owner_.getSpellHandler()->resetPlayerAuras(48);
    uint64_t nowMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    for (int slot = 0; slot < 48; ++slot) {
        auto it = allFields.find(static_cast<uint16_t>(ufAuras + slot));
        if (it != allFields.end() && it->second != 0) {
            AuraSlot& a = owner_.getSpellHandler()->getPlayerAuraSlotRef(slot);
            a.spellId = it->second;
            // Read aura flag byte: packed 4-per-uint32 at ufAuraFlags
            uint8_t aFlag = 0;
            if (ufAuraFlags != 0xFFFF) {
                auto fit = allFields.find(static_cast<uint16_t>(ufAuraFlags + slot / 4));
                if (fit != allFields.end())
                    aFlag = static_cast<uint8_t>((fit->second >> ((slot % 4) * 8)) & 0xFF);
            }
            // Normalize pre-WotLK harmful bit (0x02) to WotLK debuff bit (0x80)
            // so downstream code checking for 0x80 works consistently.
            if (aFlag & 0x02)
                aFlag = (aFlag & ~0x02) | 0x80;
            a.flags = aFlag;
            a.durationMs = -1;
            a.maxDurationMs = -1;
            a.casterGuid = owner_.getPlayerGuid();
            a.receivedAtMs = nowMs;
        }
    }
    LOG_DEBUG("[pre-WotLK] Rebuilt playerAuras from UNIT_FIELD_AURAS");
    owner_.getSpellHandler()->refreshRestorationState();
    // How many the player actually has, said once. "BuffButton1 - NOT BUILT"
    // means the interface asked UnitAura for the first buff and got nothing,
    // and a character with no buffs is the correct reason for that - this
    // separates it from the interface never having asked.
    static bool saidAuraCount = false;
    if (!saidAuraCount) {
        saidAuraCount = true;
        LOG_WARNING("player auras on first rebuild: ", owner_.getPlayerAuras().size());
    }
    pendingEvents_.emit("UNIT_AURA", {"player"});
    // The 1.12 name for the same news, carrying no unit - which is how a
    // vanilla interface hears that its buffs changed at all.
    pendingEvents_.emit("PLAYER_AURAS_CHANGED", {});
    owner_.announceCompanionChange();
    // Tracking is one of these auras, and the minimap's tracking icon is drawn
    // from whichever tracking spell is active - GetTrackingTexture walks the
    // player's tracking spells and asks exactly this aura list. The icon
    // updates on MINIMAP_UPDATE_TRACKING and nothing else, so it never changed.
    //
    // Here rather than at SetTracking, and that is what makes it complete
    // rather than half: both routes end in the aura arriving, whether the
    // spell was cast from the interface's dropdown or from a button.
    pendingEvents_.emit("MINIMAP_UPDATE_TRACKING", {});
}

// Detect player mount/dismount from UNIT_FIELD_MOUNTDISPLAYID changes
void EntityController::detectPlayerMountChange(uint32_t newMountDisplayId,
                                                const FlatFieldMap& blockFields) {
    // Live-confirmed: CMaNGOS can push a player values-update mid-taxi-flight
    // that zeroes UNIT_FIELD_MOUNTDISPLAYID before the client's own flight
    // simulation actually finishes (same early-completion behavior already
    // seen and guarded for SMSG_DISMOUNT) - obeying it here cut the mount
    // animation while MovementHandler::updateClientTaxi() kept flying the
    // real path for several more seconds. But some cores finalize taxi
    // flights server-side, so a bare "ignore while flying" guard could let
    // the client fly past a genuine server landing if local spline timing
    // ever drifts. Distinguish using the server's own UNIT_FLAG_TAXI_FLIGHT,
    // same as the SMSG_DISMOUNT guard: still set means premature (ignore,
    // let the client spline finish naturally); already cleared means the
    // server considers the flight over (honor it now).
    const bool onRealTaxiFlight = owner_.getMovementHandler() && owner_.getMovementHandler()->isOnTaxiFlight();
    if (onRealTaxiFlight && newMountDisplayId == 0) {
        bool serverStillTaxiing = false;
        auto playerEntity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid());
        auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
        if (playerUnit) {
            serverStillTaxiing = (playerUnit->getUnitFlags() & UNIT_FLAG_TAXI_FLIGHT) != 0;
        }
        const bool nearDestination = owner_.getMovementHandler()->isNearTaxiDestination();
        if (serverStillTaxiing || !nearDestination) {
            owner_.getMovementHandler()->deferServerTaxiCompletion();
            if (!nearDestination) {
                LOG_WARNING("Deferring premature taxi mount clear until landing zone");
            }
            return;
        }
        // Authoritative server completion ahead of our own spline - stop the
        // client flight now rather than snapping to a final waypoint that may
        // not match where the server actually stopped us.
        owner_.getMovementHandler()->finishClientTaxiFlight(/*snapToFinalWaypoint=*/false);
        return;
    }
    uint32_t old = owner_.currentMountDisplayIdRef();

    // A dismount the player just asked for has not reached this field yet: it
    // keeps its old value for a few frames. Taking that at face value put them
    // straight back on the mount, and the restored value then made the server's
    // own SMSG_DISMOUNT read as transient and get discarded - so the mount
    // blinked off, back on, and off again, with the character left holding the
    // seated rider pose in between.
    auto* mh = owner_.getMovementHandler();
    if (newMountDisplayId != 0 && mh && mh->isDismountPending()) {
        return;
    }
    // The server agrees: nothing left to wait for.
    if (newMountDisplayId == 0 && mh) mh->clearDismountPending();

    if (old != 0 && newMountDisplayId == 0) {
        LOG_WARNING("Authoritative mount field cleared: oldDisplay=", old,
                    " casting=", owner_.isCasting(),
                    " channeling=", owner_.isChanneling(),
                    " spell=", owner_.getCurrentCastSpellId());
    }
    owner_.currentMountDisplayIdRef() = newMountDisplayId;
    if (newMountDisplayId != old && owner_.mountCallbackRef()) owner_.mountCallbackRef()(newMountDisplayId);
    if (newMountDisplayId != old)
        pendingEvents_.emit("UNIT_MODEL_CHANGED", {"player"});
    if (old == 0 && newMountDisplayId != 0) {
        // Just mounted - find the mount aura (indefinite duration, self-cast).
        // Prefer the spell the player just cast: the blind scan below keeps the
        // last matching aura, which is as likely to be a racial or a tracking
        // buff as the mount, and the id has to be right or pressing the mount
        // again cannot be recognised as a dismount.
        owner_.mountAuraSpellIdRef() = 0;
        uint32_t justCast = 0;
        if (owner_.getSpellHandler()) {
            justCast = owner_.getSpellHandler()->getLastGroundCastSpellId();
            for (const auto& a : owner_.getSpellHandler()->getPlayerAuras()) {
                if (!a.isEmpty() && a.maxDurationMs < 0 && a.casterGuid == owner_.getPlayerGuid()) {
                    if (justCast != 0 && a.spellId == justCast) {
                        owner_.mountAuraSpellIdRef() = a.spellId;
                        break;
                    }
                    owner_.mountAuraSpellIdRef() = a.spellId;
                }
            }
        }
        // The spell that was cast, before either blind scan gets a turn.
        //
        // Both scans keep whichever indefinite aura they happen to see last,
        // which is as likely to be a racial or a tracking buff as the mount -
        // and on the pre-WotLK path the aura list this checks is often empty, so
        // the cast id was being discarded in favour of a guess. Getting it wrong
        // is not cosmetic: pressing the mount again is recognised as a dismount
        // by comparing against exactly this, so a wrong id dismounted the player
        // and put them straight back on, and the button appeared to do nothing.
        if (justCast != 0) owner_.mountAuraSpellIdRef() = justCast;

        // Pre-WotLK fallback: scan UNIT_FIELD_AURAS from same update block
        if (owner_.mountAuraSpellIdRef() == 0) {
            const uint16_t ufAuras = fieldIndex(UF::UNIT_FIELD_AURAS);
            if (ufAuras != 0xFFFF) {
                for (const auto& [fk, fv] : blockFields) {
                    if (fk >= ufAuras && fk < ufAuras + 48 && fv != 0) {
                        owner_.mountAuraSpellIdRef() = fv;
                        break;
                    }
                }
            }
        }
        LOG_INFO("Mount detected: displayId=", newMountDisplayId, " auraSpellId=", owner_.mountAuraSpellIdRef());
    }
    if (old != 0 && newMountDisplayId == 0) {
        // Only clear the specific mount aura, not all indefinite auras.
        // Previously this cleared every aura with maxDurationMs < 0, which
        // would strip racial passives, tracking, and zone buffs on dismount.
        uint32_t mountSpell = owner_.mountAuraSpellIdRef();
        owner_.mountAuraSpellIdRef() = 0;
        if (mountSpell != 0 && owner_.getSpellHandler()) {
            for (auto& a : owner_.getSpellHandler()->getPlayerAurasMut()) {
                if (!a.isEmpty() && a.spellId == mountSpell) {
                    a = AuraSlot{};
                    break;
                }
            }
        }
    }
}

namespace {
/// A float field arrives as the raw uint32 its bits spell. Named once rather
/// than written out at each of the scale fields -- which two of them still
/// did, in memcpy form, until this was pointed at them.
float bitsToFloat(uint32_t raw) {
    return std::bit_cast<float>(raw);
}
}  // namespace

// Resolve cached field indices once per handler call.
EntityController::UnitFieldIndices EntityController::UnitFieldIndices::resolve() {
    return UnitFieldIndices{
        .health = fieldIndex(UF::UNIT_FIELD_HEALTH),
        .maxHealth = fieldIndex(UF::UNIT_FIELD_MAXHEALTH),
        .powerBase = fieldIndex(UF::UNIT_FIELD_POWER1),
        .maxPowerBase = fieldIndex(UF::UNIT_FIELD_MAXPOWER1),
        .level = fieldIndex(UF::UNIT_FIELD_LEVEL),
        .faction = fieldIndex(UF::UNIT_FIELD_FACTIONTEMPLATE),
        .flags = fieldIndex(UF::UNIT_FIELD_FLAGS),
        .dynFlags = fieldIndex(UF::UNIT_DYNAMIC_FLAGS),
        .auraState = fieldIndex(UF::UNIT_FIELD_AURASTATE),
        .displayId = fieldIndex(UF::UNIT_FIELD_DISPLAYID),
        .mountDisplayId = fieldIndex(UF::UNIT_FIELD_MOUNTDISPLAYID),
        .npcFlags = fieldIndex(UF::UNIT_NPC_FLAGS),
        .npcEmoteState = fieldIndex(UF::UNIT_NPC_EMOTESTATE),
        // In the struct's own order: this is an aggregate initializer, so a
        // pair added in the wrong place here silently assigns two other fields.
        .boundingRadius = fieldIndex(UF::UNIT_FIELD_BOUNDINGRADIUS),
        .combatReach = fieldIndex(UF::UNIT_FIELD_COMBATREACH),
        .bytes0 = fieldIndex(UF::UNIT_FIELD_BYTES_0),
        .bytes1 = fieldIndex(UF::UNIT_FIELD_BYTES_1),
        .petXp = fieldIndex(UF::UNIT_FIELD_PETEXPERIENCE),
        .petNextLevelXp = fieldIndex(UF::UNIT_FIELD_PETNEXTLEVELEXP),
        .stat0 = fieldIndex(UF::UNIT_FIELD_STAT0),
        .resistances = fieldIndex(UF::UNIT_FIELD_RESISTANCES),
        .attackPower = fieldIndex(UF::UNIT_FIELD_ATTACK_POWER),
        .minDamage = fieldIndex(UF::UNIT_FIELD_MINDAMAGE),
        .maxDamage = fieldIndex(UF::UNIT_FIELD_MAXDAMAGE)
    };
}

EntityController::PlayerFieldIndices EntityController::PlayerFieldIndices::resolve() {
    return PlayerFieldIndices{
        .xp = fieldIndex(UF::PLAYER_XP),
        .nextXp = fieldIndex(UF::PLAYER_NEXT_LEVEL_XP),
        .restedXp = fieldIndex(UF::PLAYER_REST_STATE_EXPERIENCE),
        .level = fieldIndex(UF::UNIT_FIELD_LEVEL),
        .coinage = fieldIndex(UF::PLAYER_FIELD_COINAGE),
        .honor = fieldIndex(UF::PLAYER_FIELD_HONOR_CURRENCY),
        .arena = fieldIndex(UF::PLAYER_FIELD_ARENA_CURRENCY),
        .playerFlags = fieldIndex(UF::PLAYER_FLAGS),
        .armor = fieldIndex(UF::UNIT_FIELD_RESISTANCES),
        .pBytes = fieldIndex(UF::PLAYER_BYTES),
        .pBytes2 = fieldIndex(UF::PLAYER_BYTES_2),
        .chosenTitle = fieldIndex(UF::PLAYER_CHOSEN_TITLE),
        .stats = {fieldIndex(UF::UNIT_FIELD_STAT0), fieldIndex(UF::UNIT_FIELD_STAT1),
         fieldIndex(UF::UNIT_FIELD_STAT2), fieldIndex(UF::UNIT_FIELD_STAT3),
         fieldIndex(UF::UNIT_FIELD_STAT4)},
        .meleeAP = fieldIndex(UF::UNIT_FIELD_ATTACK_POWER),
        .rangedAP = fieldIndex(UF::UNIT_FIELD_RANGED_ATTACK_POWER),
        .spDmg1 = fieldIndex(UF::PLAYER_FIELD_MOD_DAMAGE_DONE_POS),
        .healBonus = fieldIndex(UF::PLAYER_FIELD_MOD_HEALING_DONE_POS),
        .blockPct = fieldIndex(UF::PLAYER_BLOCK_PERCENTAGE),
        .dodgePct = fieldIndex(UF::PLAYER_DODGE_PERCENTAGE),
        .parryPct = fieldIndex(UF::PLAYER_PARRY_PERCENTAGE),
        .critPct = fieldIndex(UF::PLAYER_CRIT_PERCENTAGE),
        .rangedCritPct = fieldIndex(UF::PLAYER_RANGED_CRIT_PERCENTAGE),
        .sCrit1 = fieldIndex(UF::PLAYER_SPELL_CRIT_PERCENTAGE1),
        .rating1 = fieldIndex(UF::PLAYER_FIELD_COMBAT_RATING_1),
        .expertise = fieldIndex(UF::PLAYER_EXPERTISE),
        .offhandExpertise = fieldIndex(UF::PLAYER_OFFHAND_EXPERTISE),
        // Mana is power index 0, so the flat modifier and its interrupted
        // (while-casting) twin sit at the base of each seven-wide array.
        .manaRegen = fieldIndex(UF::UNIT_FIELD_POWER_REGEN_FLAT_MODIFIER),
        .manaRegenCasting = fieldIndex(UF::UNIT_FIELD_POWER_REGEN_INTERRUPTED_FLAT_MODIFIER)
    };
}

// Create the appropriate Entity subclass from the block's object type.
std::shared_ptr<Entity> EntityController::createEntityFromBlock(const UpdateBlock& block) {
    switch (block.objectType) {
        case ObjectType::PLAYER:
            return std::make_shared<Player>(block.guid);
        case ObjectType::UNIT:
            return std::make_shared<Unit>(block.guid);
        case ObjectType::GAMEOBJECT:
            return std::make_shared<GameObject>(block.guid);
        default: {
            auto entity = std::make_shared<Entity>(block.guid);
            entity->setType(block.objectType);
            return entity;
        }
    }
}

//     Track player-on-transport state from movement blocks.
//     Consolidates near-identical logic from CREATE and MOVEMENT handlers.
//     When updateMovementInfoPos is true (MOVEMENT), movementInfo.x/y/z are set
//     to the raw canonical position when not on a resolved transport.
//     When false (CREATE), movementInfo is only set for resolved transport positions.
void EntityController::applyPlayerTransportState(const UpdateBlock& block,
                                                   const std::shared_ptr<Entity>& entity,
                                                   const glm::vec3& canonicalPos, float oCanonical,
                                                   bool updateMovementInfoPos) {
    // Reject server-pushed position corrections to near-origin on map 0.
    // The server stores a corrupted position from a faulty area-trigger
    // destination; accepting it lets heartbeats reinforce the bad save.
    auto positionIsBad = [&](float x, float y) {
        return owner_.getCurrentMapId() == 0 &&
               std::abs(x) < 1000.0f && std::abs(y) < 1000.0f;
    };

    if (block.onTransport) {
        // Convert transport offset from server → canonical coordinates
        glm::vec3 serverOffset(block.transportX, block.transportY, block.transportZ);
        glm::vec3 canonicalOffset = owner_.getTransportManager()
            ? owner_.getTransportManager()->serverToTransportLocal(block.transportGuid, serverOffset)
            : core::coords::serverToCanonical(serverOffset);
        owner_.setPlayerOnTransport(block.transportGuid, canonicalOffset);
        if (owner_.getTransportManager() && owner_.getTransportManager()->getTransport(owner_.playerTransportGuidRef())) {
            glm::vec3 composed = owner_.getTransportManager()->getPlayerWorldPosition(owner_.playerTransportGuidRef(), owner_.playerTransportOffsetRef());
            entity->setPosition(composed.x, composed.y, composed.z, oCanonical);
            owner_.movementInfoRef().x = composed.x;
            owner_.movementInfoRef().y = composed.y;
            owner_.movementInfoRef().z = composed.z;
        } else if (updateMovementInfoPos) {
            if (positionIsBad(canonicalPos.x, canonicalPos.y)) {
                LOG_WARNING("REJECTED player UPDATE_OBJECT to near-origin canonical=(",
                            canonicalPos.x, ", ", canonicalPos.y, ", ", canonicalPos.z, ")");
            } else {
                owner_.movementInfoRef().x = canonicalPos.x;
                owner_.movementInfoRef().y = canonicalPos.y;
                owner_.movementInfoRef().z = canonicalPos.z;
            }
        }
        LOG_INFO("Player on transport: 0x", std::hex, owner_.playerTransportGuidRef(), std::dec,
                " offset=(", owner_.playerTransportOffsetRef().x, ", ", owner_.playerTransportOffsetRef().y,
                ", ", owner_.playerTransportOffsetRef().z, ")");
    } else {
        if (updateMovementInfoPos) {
            if (positionIsBad(canonicalPos.x, canonicalPos.y)) {
                LOG_WARNING("REJECTED player UPDATE_OBJECT to near-origin canonical=(",
                            canonicalPos.x, ", ", canonicalPos.y, ", ", canonicalPos.z, ")");
            } else {
                owner_.movementInfoRef().x = canonicalPos.x;
                owner_.movementInfoRef().y = canonicalPos.y;
                owner_.movementInfoRef().z = canonicalPos.z;
            }
        }
        // Don't clear client-detected transport boarding. The server does not know
        // about locally animated M2 trams or TaxiPathNode WMO ships until our next
        // movement heartbeat publishes the attachment.
        bool isClientAnimatedTransport = false;
        if (owner_.playerTransportGuidRef() != 0 && owner_.getTransportManager()) {
            auto* tr = owner_.getTransportManager()->getTransport(owner_.playerTransportGuidRef());
            isClientAnimatedTransport = tr &&
                (tr->isM2 || (tr->worldCoords && tr->useClientAnimation));
        }
        if (owner_.playerTransportGuidRef() != 0 && !isClientAnimatedTransport) {
            LOG_INFO("Player left transport");
            owner_.clearPlayerTransport();
        }
    }
}

//     Apply unit fields during CREATE - sets health/power/level/flags/displayId/etc.
//     Returns true if the entity is initially dead (health=0 or DYNFLAG_DEAD).
bool EntityController::applyUnitFieldsOnCreate(const UpdateBlock& block,
                                                 std::shared_ptr<Unit>& unit,
                                                 const UnitFieldIndices& ufi) {
    bool unitInitiallyDead = false;
    for (const auto& [key, val] : block.fields) {
        // Check all specific fields BEFORE power/maxpower range checks.
        // In Classic, power indices (23-27) are adjacent to maxHealth (28),
        // and maxPower indices (29-33) are adjacent to level (34) and faction (35).
        // A range check like "key >= powerBase && key < powerBase+7" would
        // incorrectly capture maxHealth/level/faction in Classic's tight layout.
        // The events FrameXML's unit frames update on. Each carries the unit id
        // as its first argument, which is how a frame knows whether the change
        // was to the unit it is showing.
        auto emitForUnit = [&](const char* event) {
            if (!owner_.addonEventCallbackRef()) return;
            const auto uid = owner_.guidToUnitId(block.guid);
            if (!uid.empty()) pendingEvents_.emit(event, {uid});
        };

        if (key == ufi.health) {
            unit->setHealth(val);
            emitForUnit("UNIT_HEALTH");
            if ((block.objectType == ObjectType::UNIT ||
                 block.objectType == ObjectType::PLAYER) && val == 0) {
                unitInitiallyDead = true;
            }
            if (block.guid == owner_.getPlayerGuid() && val == 0) {
                owner_.playerDeadRef() = true;
                LOG_INFO("Player logged in dead");
            }
        } else if (key == ufi.maxHealth) {
            unit->setMaxHealth(val);
            emitForUnit("UNIT_MAXHEALTH");
        }
        else if (key == ufi.level) {
            unit->setLevel(val);
            emitForUnit("UNIT_LEVEL");
        } else if (key == ufi.faction) {
            unit->setFactionTemplate(val);
            if (owner_.addonEventCallbackRef()) {
                auto uid = owner_.guidToUnitId(block.guid);
                if (!uid.empty())
                    pendingEvents_.emit("UNIT_FACTION", {uid});
            }
        }
        else if (key == ufi.flags) {
            unit->setUnitFlags(val);
            if (owner_.addonEventCallbackRef()) {
                auto uid = owner_.guidToUnitId(block.guid);
                if (!uid.empty())
                    pendingEvents_.emit("UNIT_FLAGS", {uid});
            }
        }
        else if (ufi.auraState != 0xFFFF && key == ufi.auraState) {
            unit->setAuraState(val);
        }
        else if (key == ufi.bytes0) {
            unit->setPowerType(static_cast<uint8_t>((val >> 24) & 0xFF));
            // Which bar to show at all - a druid shifting form changes it.
            emitForUnit("UNIT_DISPLAYPOWER");
        } else if (key == ufi.boundingRadius) {
            unit->setBoundingRadius(bitsToFloat(val));
        } else if (key == ufi.combatReach) {
            unit->setCombatReach(bitsToFloat(val));
        } else if (key == ufi.displayId) {
            unit->setDisplayId(val);
            if (owner_.addonEventCallbackRef()) {
                auto uid = owner_.guidToUnitId(block.guid);
                if (!uid.empty()) {
                    pendingEvents_.emit("UNIT_MODEL_CHANGED", {uid});
                    // The portrait is drawn from the display id, so a unit that
                    // changes model - a shapeshift, a polymorph, a mount -
                    // needs its portrait redrawn too. Six frames listen for
                    // this and none of them had ever heard it.
                    pendingEvents_.emit("UNIT_PORTRAIT_UPDATE", {uid});
                }
            }
        }
        else if (key == ufi.npcFlags) { unit->setNpcFlags(val); }
        else if (key == ufi.npcEmoteState) { unit->setNpcEmoteState(val); }
        else if (key == ufi.dynFlags) {
            unit->setDynamicFlags(val);
            if ((block.objectType == ObjectType::UNIT ||
                 block.objectType == ObjectType::PLAYER) &&
                ((val & UNIT_DYNFLAG_DEAD) != 0 || (val & UNIT_DYNFLAG_LOOTABLE) != 0)) {
                unitInitiallyDead = true;
            }
        }
        // Power/maxpower range checks AFTER all specific fields
        else if (key >= ufi.powerBase && key < ufi.powerBase + 7) {
            const auto powerType = static_cast<uint8_t>(key - ufi.powerBase);
            unit->setPowerByType(powerType, val);
            // Named per power rather than one event: FrameXML registers only
            // the one its bar shows, so a rogue's frame is not woken by every
            // mana tick in the party.
            static const char* kPowerEvents[7] = {
                "UNIT_MANA", "UNIT_RAGE", "UNIT_FOCUS", "UNIT_ENERGY",
                "UNIT_HAPPINESS", "UNIT_RUNIC_POWER", "UNIT_RUNIC_POWER"
            };
            if (powerType < 7) emitForUnit(kPowerEvents[powerType]);
        } else if (key >= ufi.maxPowerBase && key < ufi.maxPowerBase + 7) {
            unit->setMaxPowerByType(static_cast<uint8_t>(key - ufi.maxPowerBase), val);
            // The maximum a bar is scaled against, which FrameXML redraws on.
            static const char* kMaxPowerEvents[7] = {
                "UNIT_MAXMANA", "UNIT_MAXRAGE", "UNIT_MAXFOCUS", "UNIT_MAXENERGY",
                "UNIT_MAXHAPPINESS", "UNIT_MAXRUNIC_POWER", "UNIT_MAXRUNIC_POWER"
            };
            if (const auto t = static_cast<uint8_t>(key - ufi.maxPowerBase); t < 7)
                emitForUnit(kMaxPowerEvents[t]);
        }
        else if (key == ufi.mountDisplayId) {
            if (block.guid == owner_.getPlayerGuid()) {
                detectPlayerMountChange(val, block.fields);
            } else if (block.objectType == ObjectType::PLAYER &&
                       owner_.otherPlayerMountCallbackRef()) {
                owner_.otherPlayerMountCallbackRef()(block.guid, val);
            }
            unit->setMountDisplayId(val);
        }
    }
    // Initial update masks commonly omit fields whose value is zero. A dead unit
    // can therefore arrive without UNIT_FIELD_HEALTH even though its default
    // client-side health is zero; the dynamic corpse bits remain authoritative.
    if ((block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) &&
        isUnitCorpseState(unit->getHealth(), unit->getMaxHealth(), unit->getDynamicFlags())) {
        unitInitiallyDead = true;
    }
    return unitInitiallyDead;
}

// Consolidates player-death state into one place so both the health==0 and
// dynFlags UNIT_DYNFLAG_DEAD paths share the same corpse-caching logic.
// Classic WoW does not send SMSG_DEATH_RELEASE_LOC, so this cached position
// is the primary source for canReclaimCorpse().
void EntityController::markPlayerDead(const char* source) {
    owner_.playerDeadRef() = true;
    owner_.releasedSpiritRef() = false;
    // owner_.movementInfoRef() is canonical (x=north, y=west); corpseX_/Y_ are
    // raw server coords (x=west, y=north) - swap axes.
    owner_.corpseXRef()     = owner_.movementInfoRef().y;
    owner_.corpseYRef()     = owner_.movementInfoRef().x;
    owner_.corpseZRef()     = owner_.movementInfoRef().z;
    owner_.corpseMapIdRef() = owner_.currentMapIdRef();
    owner_.corpsePositionValidRef() = true;
    LOG_INFO("Player died (", source, "). Corpse cached at server=(",
             owner_.corpseXRef(), ",", owner_.corpseYRef(), ",", owner_.corpseZRef(),
             ") map=", owner_.corpseMapIdRef());
}

// 3c: Apply unit fields during VALUES update - tracks health/power/display changes
//     and fires events for transitions (death, resurrect, level up, etc.).
EntityController::UnitFieldUpdateResult EntityController::applyUnitFieldsOnUpdate(
        const UpdateBlock& block, const std::shared_ptr<Entity>& entity,
        std::shared_ptr<Unit>& unit, const UnitFieldIndices& ufi) {
    UnitFieldUpdateResult result;
    result.oldDisplayId = unit->getDisplayId();
    uint32_t oldHealth = unit->getHealth();
    bool petExperienceChanged = false;
    bool petStatsChanged = false;
    for (const auto& [key, val] : block.fields) {
        if (key == ufi.health) {
            unit->setHealth(val);
            result.healthChanged = true;
            if (val == 0) {
                if (owner_.getCombatHandler() && block.guid == owner_.getCombatHandler()->getAutoAttackTargetGuid()) {
                    owner_.stopAutoAttack();
                }
                if (owner_.getCombatHandler()) owner_.getCombatHandler()->removeHostileAttacker(block.guid);
                if (block.guid == owner_.getPlayerGuid()) {
                    markPlayerDead("health=0");
                    owner_.stopAutoAttack();
                    pendingEvents_.emit("PLAYER_DEAD", {});
                }
                if ((entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) && owner_.npcDeathCallbackRef()) {
                    owner_.npcDeathCallbackRef()(block.guid);
                    result.npcDeathNotified = true;
                }
            } else if (oldHealth == 0 && val > 0) {
                if (block.guid == owner_.getPlayerGuid()) {
                    bool wasGhost = owner_.releasedSpiritRef();
                    owner_.playerDeadRef() = false;
                    if (!wasGhost) {
                        LOG_INFO("Player resurrected!");
                        pendingEvents_.emit("PLAYER_ALIVE", {});
                    } else {
                        LOG_INFO("Player entered ghost form");
                        owner_.releasedSpiritRef() = false;
                        pendingEvents_.emit("PLAYER_UNGHOST", {});
                    }
                }
                if ((entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) && owner_.npcRespawnCallbackRef()) {
                    owner_.npcRespawnCallbackRef()(block.guid);
                    result.npcRespawnNotified = true;
                }
            }
        // Specific fields checked BEFORE power/maxpower range checks
        // (Classic packs maxHealth/level/faction adjacent to power indices)
        } else if (key == ufi.maxHealth) {
            unit->setMaxHealth(val);
            result.healthChanged = true;
            result.maxHealthChanged = true;
        }
        else if (key == ufi.bytes0) {
            uint8_t oldPT = unit->getPowerType();
            unit->setPowerType(static_cast<uint8_t>((val >> 24) & 0xFF));
            if (unit->getPowerType() != oldPT) {
                auto uid = owner_.guidToUnitId(block.guid);
                if (!uid.empty())
                    pendingEvents_.emit("UNIT_DISPLAYPOWER", {uid});
            }
        } else if (key == ufi.flags) {
            uint32_t oldFlags = unit->getUnitFlags();
            unit->setUnitFlags(val);
            // UNIT_FIELD_FLAGS is the server's authoritative combat state. Spell-only
            // attackers do not necessarily produce SMSG_ATTACKSTOP, so retaining them
            // after this bit clears leaves the client permanently "in combat".
            if (block.guid == owner_.getPlayerGuid() &&
                (oldFlags & UNIT_FLAG_IN_COMBAT) != 0 &&
                (val & UNIT_FLAG_IN_COMBAT) == 0 && owner_.getCombatHandler()) {
                owner_.getCombatHandler()->clearHostileAttackers();
            }
            // Entering and leaving combat, which the interface names after the
            // health regeneration that stops and starts with it. Registered for
            // seven times across FrameXML and the addons and fired by nothing:
            // it is how a panel knows to lock itself, and how anything that
            // must not change mid-fight finds out the fight has started.
            if (block.guid == owner_.getPlayerGuid()) {
                const bool was = (oldFlags & UNIT_FLAG_IN_COMBAT) != 0;
                const bool now = (val & UNIT_FLAG_IN_COMBAT) != 0;
                if (was != now) {
                    pendingEvents_.emit(now ? "PLAYER_REGEN_DISABLED"
                                            : "PLAYER_REGEN_ENABLED", {});
                }
            }
            // Detect stun state change on local player
            constexpr uint32_t UNIT_FLAG_STUNNED = 0x00040000;
            if (block.guid == owner_.getPlayerGuid() && owner_.stunStateCallbackRef()) {
                bool wasStunned = (oldFlags & UNIT_FLAG_STUNNED) != 0;
                bool nowStunned = (val & UNIT_FLAG_STUNNED) != 0;
                // The server stuns the player for the logout countdown, to root them
                // in place - it sits them down in the same breath. That is a movement
                // restriction, not a stun: playing the stun animation over it leaves
                // the character slumped rather than seated. Clearing the flag is still
                // honoured, so a cancelled logout recovers.
                if (nowStunned && owner_.isLoggingOut()) {
                    nowStunned = false;
                }
                if (wasStunned != nowStunned) {
                    owner_.stunStateCallbackRef()(nowStunned);
                }
            }
        }
        else if (ufi.auraState != 0xFFFF && key == ufi.auraState) {
            unit->setAuraState(val);
        }
        else if (ufi.bytes1 != 0xFFFF && key == ufi.bytes1) {
            const uint8_t oldVisibilityFlags = unit->getVisibilityFlags();
            const uint8_t newVisibilityFlags = static_cast<uint8_t>((val >> 16) & 0xFF);
            unit->setVisibilityFlags(newVisibilityFlags);

            if (block.guid == owner_.getPlayerGuid()) {
                const bool wasStealthed = (oldVisibilityFlags & UNIT_VIS_FLAG_CREEP) != 0;
                const bool nowStealthed = (newVisibilityFlags & UNIT_VIS_FLAG_CREEP) != 0;
                if (wasStealthed != nowStealthed && owner_.stealthStateCallbackRef()) {
                    owner_.stealthStateCallbackRef()(nowStealthed);
                }

                uint8_t newForm = static_cast<uint8_t>((val >> 24) & 0xFF);
                if (newForm != owner_.shapeshiftFormIdRef()) {
                    owner_.shapeshiftFormIdRef() = newForm;
                    LOG_INFO("Shapeshift form changed: ", static_cast<int>(newForm));
                    pendingEvents_.emit("UPDATE_SHAPESHIFT_FORM", {});
                    pendingEvents_.emit("UPDATE_SHAPESHIFT_FORMS", {});
                    // The stance bar and the action bar are two different
                    // things. These two move the stance bar; the extra action
                    // bar a form brings with it is shown and hidden by
                    // UPDATE_BONUS_ACTIONBAR, and actionbutton.lua repages on
                    // the same event. Neither was fired, so the bar never
                    // appeared and the buttons kept reading page one.
                    pendingEvents_.emit("UPDATE_BONUS_ACTIONBAR", {});
                    pendingEvents_.emit("ACTIONBAR_PAGE_CHANGED", {});
                }
            }
        }
        else if (key == ufi.dynFlags) {
            uint32_t oldDyn = unit->getDynamicFlags();
            unit->setDynamicFlags(val);
            if (block.guid == owner_.getPlayerGuid()) {
                bool wasDead = (oldDyn & UNIT_DYNFLAG_DEAD) != 0;
                bool nowDead = (val & UNIT_DYNFLAG_DEAD) != 0;
                if (!wasDead && nowDead) {
                    markPlayerDead("dynFlags");
                    // And tell the interface, which the health=0 path did and
                    // this one did not. The server marks a death either way -
                    // sometimes by dropping health to zero, sometimes by
                    // setting UNIT_DYNFLAG_DEAD, and not always both - and only
                    // the health path fired PLAYER_DEAD. When death came by the
                    // flag alone, the player was dead in every internal sense
                    // (could not attack, health read zero) but the interface
                    // never heard it: no release-spirit popup, until the server
                    // gave up waiting and pulled the corpse to the graveyard.
                    // FrameXML guards the popup on its own visibility, so a
                    // second PLAYER_DEAD from the health path is harmless.
                    owner_.stopAutoAttack();
                    pendingEvents_.emit("PLAYER_DEAD", {});
                } else if (wasDead && !nowDead) {
                    owner_.playerDeadRef() = false;
                    owner_.releasedSpiritRef() = false;
                    owner_.selfResAvailableRef() = false;
                    LOG_INFO("Player resurrected (dynamic flags)");
                    // The other side of the same signal. PLAYER_ALIVE is what
                    // hides the death popup and restores the interface, and
                    // firing PLAYER_DEAD without ever firing this left the
                    // popup up through a resurrection that came by the flag.
                    pendingEvents_.emit("PLAYER_ALIVE", {});
                }
            } else if (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) {
                bool wasDead = (oldDyn & UNIT_DYNFLAG_DEAD) != 0;
                bool nowDead = (val & UNIT_DYNFLAG_DEAD) != 0;
                if (!wasDead && nowDead) {
                    if (!result.npcDeathNotified && owner_.npcDeathCallbackRef()) {
                        owner_.npcDeathCallbackRef()(block.guid);
                        result.npcDeathNotified = true;
                    }
                } else if (wasDead && !nowDead) {
                    if (!result.npcRespawnNotified && owner_.npcRespawnCallbackRef()) {
                        owner_.npcRespawnCallbackRef()(block.guid);
                        result.npcRespawnNotified = true;
                    }
                }
                if (entity->getType() == ObjectType::UNIT &&
                    (oldDyn & UNIT_DYNFLAG_LOOTABLE) != 0 &&
                    (val & UNIT_DYNFLAG_LOOTABLE) == 0) {
                    result.lootableCleared = true;
                }
            }
        } else if (key == ufi.level) {
            uint32_t oldLvl = unit->getLevel();
            unit->setLevel(val);
            if (val != oldLvl) {
                auto uid = owner_.guidToUnitId(block.guid);
                if (!uid.empty())
                    pendingEvents_.emit("UNIT_LEVEL", {uid});
            }
            if (block.guid != owner_.getPlayerGuid() &&
                entity->getType() == ObjectType::PLAYER &&
                val > oldLvl && oldLvl > 0 &&
                owner_.otherPlayerLevelUpCallbackRef()) {
                owner_.otherPlayerLevelUpCallbackRef()(block.guid, val);
            }
        }
        else if (key == ufi.faction) {
            unit->setFactionTemplate(val);
            unit->setHostile(owner_.isHostileFaction(val));
        } else if (key == ufi.boundingRadius) {
            unit->setBoundingRadius(bitsToFloat(val));
        } else if (key == ufi.combatReach) {
            unit->setCombatReach(bitsToFloat(val));
        } else if (key == ufi.displayId) {
            if (val != unit->getDisplayId()) {
                unit->setDisplayId(val);
                result.displayIdChanged = true;
            }
        } else if (key == ufi.mountDisplayId) {
            if (block.guid == owner_.getPlayerGuid()) {
                detectPlayerMountChange(val, block.fields);
            } else if (entity->getType() == ObjectType::PLAYER &&
                       val != unit->getMountDisplayId() &&
                       owner_.otherPlayerMountCallbackRef()) {
                owner_.otherPlayerMountCallbackRef()(block.guid, val);
            }
            unit->setMountDisplayId(val);
        } else if (ufi.attackPower != 0xFFFF && key == ufi.attackPower &&
                   block.guid == owner_.petGuidRef()) {
            owner_.petAttackPowerRef() = static_cast<int32_t>(val);
            petStatsChanged = true;
        } else if (ufi.minDamage != 0xFFFF && key == ufi.minDamage &&
                   block.guid == owner_.petGuidRef()) {
            // A float, sent as its bits like every other float field.
            std::memcpy(&owner_.petMinDamageRef(), &val, 4);
            petStatsChanged = true;
        } else if (ufi.maxDamage != 0xFFFF && key == ufi.maxDamage &&
                   block.guid == owner_.petGuidRef()) {
            std::memcpy(&owner_.petMaxDamageRef(), &val, 4);
            petStatsChanged = true;
        } else if (ufi.stat0 != 0xFFFF && key >= ufi.stat0 && key < ufi.stat0 + 5 &&
                   block.guid == owner_.petGuidRef()) {
            // The pet's own five, not the player's. The paperdoll's pet tab
            // reads them through UnitStat("pet"), which used to answer from the
            // player - so a hunter's pet listed its owner's Strength.
            owner_.petStatsRef()[key - ufi.stat0] = static_cast<int32_t>(val);
            petStatsChanged = true;
        } else if (ufi.resistances != 0xFFFF && key >= ufi.resistances &&
                   key < ufi.resistances + 7 && block.guid == owner_.petGuidRef()) {
            // Armor is index zero and the six schools follow it, the same shape
            // as the player's.
            owner_.petResistancesRef()[key - ufi.resistances] = static_cast<int32_t>(val);
            petStatsChanged = true;
        } else if (ufi.petXp != 0xFFFF && key == ufi.petXp &&
                   block.guid == owner_.petGuidRef()) {
            // Only the player's own pet. Every unit carries these fields and
            // only one of them has an experience bar to draw.
            if (owner_.petExperienceRef() != val) {
                owner_.petExperienceRef() = val;
                petExperienceChanged = true;
            }
        } else if (ufi.petNextLevelXp != 0xFFFF && key == ufi.petNextLevelXp &&
                   block.guid == owner_.petGuidRef()) {
            if (owner_.petNextLevelExpRef() != val) {
                owner_.petNextLevelExpRef() = val;
                petExperienceChanged = true;
            }
        } else if (key == ufi.npcFlags) { unit->setNpcFlags(val); }
        else if (key == ufi.npcEmoteState) {
            uint32_t oldEmote = unit->getNpcEmoteState();
            unit->setNpcEmoteState(val);
            // Fire emote animation callback so entity_spawner can update the NPC's idle anim
            if (val != oldEmote && owner_.emoteAnimCallbackRef()) {
                uint32_t animId = val != 0 ? rendering::AnimationController::getEmoteAnimByEmotesId(val) : 0;
                if (val == 0 || animId != 0) {
                    // UNIT_NPC_EMOTESTATE is persistent by definition - a zero
                    // here genuinely clears the state loop.
                    owner_.emoteAnimCallbackRef()(block.guid, animId, /*isState=*/true);
                } else {
                    LOG_DEBUG("UNIT_NPC_EMOTESTATE emoteId=", val, " had no Emotes.dbc animation mapping");
                }
            }
        }
        // Power/maxpower range checks AFTER all specific fields
        else if (key >= ufi.powerBase && key < ufi.powerBase + 7) {
            unit->setPowerByType(static_cast<uint8_t>(key - ufi.powerBase), val);
            result.powerChanged = true;
            result.powerTypeChanged = static_cast<int>(key - ufi.powerBase);
        } else if (key >= ufi.maxPowerBase && key < ufi.maxPowerBase + 7) {
            unit->setMaxPowerByType(static_cast<uint8_t>(key - ufi.maxPowerBase), val);
            result.powerChanged = true;
            result.maxPowerTypeChanged = static_cast<int>(key - ufi.maxPowerBase);
        }
    }

    // Fire UNIT_HEALTH / UNIT_POWER events for Lua addons
    if ((result.healthChanged || result.powerChanged)) {
        auto unitId = owner_.guidToUnitId(block.guid);
        if (!unitId.empty()) {
            if (result.healthChanged) pendingEvents_.emit("UNIT_HEALTH", {unitId});
            if (result.maxHealthChanged) pendingEvents_.emit("UNIT_MAXHEALTH", {unitId});
            if (result.powerChanged) {
                // The event a WotLK interface is listening for is named after
                // the power itself. UNIT_POWER is the later, generic one - it
                // arrived in Cataclysm, and every unit frame this client
                // targets registers UNIT_MANA, UNIT_RAGE, UNIT_ENERGY or
                // UNIT_FOCUS instead. Sending only the generic name meant the
                // mana bar was told nothing it understood and never moved,
                // even though the number behind it was current.
                //
                // The create path already names them; this is the same table.
                static const char* kPowerEvents[7] = {
                    "UNIT_MANA", "UNIT_RAGE", "UNIT_FOCUS", "UNIT_ENERGY",
                    "UNIT_HAPPINESS", "UNIT_RUNIC_POWER", "UNIT_RUNIC_POWER"
                };
                static const char* kMaxPowerEvents[7] = {
                    "UNIT_MAXMANA", "UNIT_MAXRAGE", "UNIT_MAXFOCUS", "UNIT_MAXENERGY",
                    "UNIT_MAXHAPPINESS", "UNIT_MAXRUNIC_POWER", "UNIT_MAXRUNIC_POWER"
                };
                if (result.powerTypeChanged >= 0 && result.powerTypeChanged < 7)
                    pendingEvents_.emit(kPowerEvents[result.powerTypeChanged], {unitId});
                if (result.maxPowerTypeChanged >= 0 && result.maxPowerTypeChanged < 7)
                    pendingEvents_.emit(kMaxPowerEvents[result.maxPowerTypeChanged], {unitId});
                pendingEvents_.emit("UNIT_POWER", {unitId});
                // When player power changes, action bar usability may change
                if (block.guid == owner_.getPlayerGuid()) {
                    pendingEvents_.emit("ACTIONBAR_UPDATE_USABLE", {});
                    pendingEvents_.emit("SPELL_UPDATE_USABLE", {});
                }
            }
        }
    }

    // Fire player health callback for wounded-idle animation
    if (result.healthChanged && block.guid == owner_.getPlayerGuid() && owner_.playerHealthCallbackRef()) {
        owner_.playerHealthCallbackRef()(unit->getHealth(), unit->getMaxHealth());
    }

    // The pet frame's experience bar reads GetPetExperience and redraws on
    // this; without it the bar was filled once when the pet was summoned and
    // never moved again.
    if (petExperienceChanged && owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("UNIT_PET_EXPERIENCE", {"pet"});
    }
    // The pet tab redraws its stat block on these, named for the pet.
    if (petStatsChanged && owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("UNIT_STATS", {"pet"});
        owner_.addonEventCallbackRef()("UNIT_RESISTANCES", {"pet"});
    }

    return result;
}

//     Apply player stat fields (XP, coinage, combat stats, etc.).
//     Shared between CREATE and VALUES - isCreate controls event firing differences.
bool EntityController::applyPlayerStatFields(const FlatFieldMap& fields,
                                               const PlayerFieldIndices& pfi,
                                               bool isCreate) {
    bool slotsChanged = false;
    // Whether anything the character sheet prints actually moved. The sheet
    // refreshes on events, not on a timer, so a stat that changes without one
    // being fired leaves the panel showing the old number until something else
    // happens to redraw it.
    bool statsChanged = false;
    bool ratingsChanged = false;
    bool powerChanged = false;
    bool rangedPowerChanged = false;
    bool resistancesChanged = false;
    bool spellBonusChanged = false;
    for (const auto& [key, val] : fields) {
        if (key == pfi.xp) {
            owner_.playerXpRef() = val;
            if (!isCreate) {
                LOG_DEBUG("XP updated: ", val);
                pendingEvents_.emit("PLAYER_XP_UPDATE", {std::to_string(val)});
            }
        }
        else if (key == pfi.nextXp) {
            owner_.playerNextLevelXpRef() = val;
            if (!isCreate) LOG_DEBUG("Next level XP updated: ", val);
        }
        else if (pfi.restedXp != 0xFFFF && key == pfi.restedXp) {
            owner_.playerRestedXpRef() = val;
            if (!isCreate) pendingEvents_.emit("UPDATE_EXHAUSTION", {});
        }
        else if (key == pfi.level) {
            owner_.serverPlayerLevelRef() = val;
            if (!isCreate) LOG_DEBUG("Level updated: ", val);
            for (auto& ch : owner_.charactersRef()) {
                if (ch.guid == owner_.getPlayerGuid()) { ch.level = val; break; }
            }
        }
        else if (key == pfi.coinage) {
            uint64_t oldMoney = owner_.playerMoneyCopperRef();
            owner_.playerMoneyCopperRef() = val;
            LOG_DEBUG("Money ", isCreate ? "set from update fields: " : "updated via VALUES: ", val, " copper");
            if (val != oldMoney)
                pendingEvents_.emit("PLAYER_MONEY", {});
        }
        else if (pfi.honor != 0xFFFF && key == pfi.honor) {
            owner_.playerHonorPointsRef() = val;
            LOG_DEBUG("Honor points ", isCreate ? "from update fields: " : "updated: ", val);
        }
        else if (pfi.arena != 0xFFFF && key == pfi.arena) {
            owner_.playerArenaPointsRef() = val;
            LOG_DEBUG("Arena points ", isCreate ? "from update fields: " : "updated: ", val);
        }
        else if (pfi.armor != 0xFFFF && key == pfi.armor) {
            const int32_t armorVal = static_cast<int32_t>(val);
            if (owner_.playerArmorRatingRef() != armorVal) resistancesChanged = true;
            owner_.playerArmorRatingRef() = armorVal;
            if (isCreate) LOG_DEBUG("Armor rating from update fields: ", owner_.playerArmorRatingRef());
        }
        else if (pfi.armor != 0xFFFF && key > pfi.armor && key <= pfi.armor + 6) {
            const int32_t resVal = static_cast<int32_t>(val);
            // Armor is the first of the seven - pfi.armor is
            // UNIT_FIELD_RESISTANCES and the six schools follow it - which is
            // why one event covers the block.
            if (owner_.playerResistancesArr()[key - pfi.armor - 1] != resVal) resistancesChanged = true;
            owner_.playerResistancesArr()[key - pfi.armor - 1] = resVal;
        }
        else if (pfi.pBytes2 != 0xFFFF && key == pfi.pBytes2) {
            uint8_t bankBagSlots = static_cast<uint8_t>((val >> 16) & 0xFF);
            owner_.inventoryRef().setPurchasedBankBagSlots(bankBagSlots);
            // Byte 3 (bits 24-31): REST_STATE
            // 0 = not resting, 1 = REST_TYPE_IN_TAVERN, 2 = REST_TYPE_IN_CITY
            uint8_t restStateByte = static_cast<uint8_t>((val >> 24) & 0xFF);
            if (isCreate) {
                LOG_DEBUG("PLAYER_BYTES_2 (CREATE): raw=0x", std::hex, val, std::dec,
                           " bankBagSlots=", static_cast<int>(bankBagSlots));
                bool wasResting = owner_.isRestingRef();
                owner_.isRestingRef() = (restStateByte != 0);
                if (owner_.isRestingRef() != wasResting) {
                    pendingEvents_.emit("UPDATE_EXHAUSTION", {});
                    pendingEvents_.emit("PLAYER_UPDATE_RESTING", {});
                }
            } else {
                // Byte 0 (bits 0-7): facial hair / piercings
                uint8_t facialHair = static_cast<uint8_t>(val & 0xFF);
                for (auto& ch : owner_.charactersRef()) {
                    if (ch.guid == owner_.getPlayerGuid()) { ch.facialFeatures = facialHair; break; }
                }
                LOG_DEBUG("PLAYER_BYTES_2 (VALUES): raw=0x", std::hex, val, std::dec,
                           " bankBagSlots=", static_cast<int>(bankBagSlots),
                           " facial=", static_cast<int>(facialHair));
                owner_.isRestingRef() = (restStateByte != 0);
                if (owner_.appearanceChangedCallbackRef())
                    owner_.appearanceChangedCallbackRef()();
                if (owner_.playerModelRebuildCallbackRef())
                    owner_.playerModelRebuildCallbackRef()();
            }
        }
        else if (pfi.chosenTitle != 0xFFFF && key == pfi.chosenTitle) {
            owner_.chosenTitleBitRef() = static_cast<int32_t>(val);
            LOG_DEBUG("PLAYER_CHOSEN_TITLE ", isCreate ? "from update fields: " : "updated: ",
                      owner_.chosenTitleBitRef());
        }
        // VALUES-only fields: PLAYER_BYTES (appearance) and PLAYER_FLAGS (ghost state)
        else if (!isCreate && pfi.pBytes != 0xFFFF && key == pfi.pBytes) {
            // PLAYER_BYTES changed (barber shop, polymorph, etc.)
            for (auto& ch : owner_.charactersRef()) {
                if (ch.guid == owner_.getPlayerGuid()) { ch.appearanceBytes = val; break; }
            }
            if (owner_.appearanceChangedCallbackRef())
                owner_.appearanceChangedCallbackRef()();
            if (owner_.playerModelRebuildCallbackRef())
                owner_.playerModelRebuildCallbackRef()();
        }
        else if (key == pfi.playerFlags) {
          // Resting is read on create as well as on change, because a player
          // who logs in inside an inn is already resting and there is no later
          // transition to notice. The ghost handling below stays on updates
          // only: it drives release and resurrection, which are transitions.
          constexpr uint32_t PLAYER_FLAGS_RESTING = 0x00000020;
          const bool nowResting = (val & PLAYER_FLAGS_RESTING) != 0;
          if (nowResting != owner_.isRestingRef()) {
              owner_.isRestingRef() = nowResting;
              pendingEvents_.emit("PLAYER_UPDATE_RESTING", {});
          }
          // The helm and cloak switches, which the realm keeps and this client
          // never read back. Both are set by CMSG_SHOWING_HELM/CLOAK and come
          // home in PLAYER_FLAGS, so the realm had them right the whole time -
          // helmVisible_ simply started true every session and was written by
          // nothing but the toggle. Hide the helm, log out, and it was back.
          //
          // On create as well as on change, for the reason resting is: the
          // flags arrive with the player and there is no later transition.
          //
          // The flag says *hide*, so it is inverted here.
          {
            constexpr uint32_t PLAYER_FLAGS_HIDE_HELM  = 0x00000400;
            constexpr uint32_t PLAYER_FLAGS_HIDE_CLOAK = 0x00000800;
            const bool helm  = (val & PLAYER_FLAGS_HIDE_HELM) == 0;
            const bool cloak = (val & PLAYER_FLAGS_HIDE_CLOAK) == 0;
            if (helm != owner_.helmVisibleRef() || cloak != owner_.cloakVisibleRef()) {
                owner_.helmVisibleRef() = helm;
                owner_.cloakVisibleRef() = cloak;
                // The same rebuild the toggle asks for: the flag on its own
                // changes nothing on screen, and the hair under a helm has to
                // come back with it.
                owner_.markOnlineEquipmentDirty();
                LOG_INFO("PLAYER_FLAGS: helm ", helm ? "shown" : "hidden",
                         ", cloak ", cloak ? "shown" : "hidden");
            }
          }
          {
            // Not gated on !isCreate, and that is the whole of a bug: logging
            // in already dead delivers the player as a CREATE block, so the
            // ghost flag it carries was never read. releasedSpirit_ stayed
            // false, canReclaimCorpse refused, and a player who logged in
            // standing on their own corpse could not take it back - the one
            // case where the flag arrives without a transition to notice.
            constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
            bool wasGhost = owner_.releasedSpiritRef();
            bool nowGhost = (val & PLAYER_FLAGS_GHOST) != 0;
            if (!wasGhost && nowGhost) {
                owner_.releasedSpiritRef() = true;
                LOG_INFO("Player entered ghost form (PLAYER_FLAGS)");
                if (owner_.ghostStateCallbackRef()) owner_.ghostStateCallbackRef()(true);
            } else if (wasGhost && !nowGhost) {
                owner_.releasedSpiritRef() = false;
                owner_.playerDeadRef() = false;
                owner_.repopPendingRef() = false;
                owner_.resurrectPendingRef() = false;
                owner_.selfResAvailableRef() = false;
                owner_.corpseMapIdRef() = 0;  // corpse reclaimed
                owner_.corpsePositionValidRef() = false;
                owner_.corpseGuidRef() = 0;
                owner_.corpseReclaimAvailableMsRef() = 0;
                LOG_INFO("Player resurrected (PLAYER_FLAGS ghost cleared)");
                pendingEvents_.emit("PLAYER_ALIVE", {});
                if (owner_.ghostStateCallbackRef()) owner_.ghostStateCallbackRef()(false);
            }
            // Which unit's flags, because that is the only thing the
            // handlers do with it: TargetFrame compares arg1 against its
            // own unit before redrawing the away and busy markers, so an
            // absent one matched no frame and none of them ever redrew.
            // The event only where something changed. A create block is the
            // interface being told what is, not what moved, and every frame
            // reads the current values when it is built.
            if (!isCreate) pendingEvents_.emit("PLAYER_FLAGS_CHANGED", {"player"});
          }
        }
        else if (pfi.meleeAP  != 0xFFFF && key == pfi.meleeAP)  {
            const int32_t ap = static_cast<int32_t>(val);
            if (owner_.playerMeleeAPRef() != ap) powerChanged = true;
            owner_.playerMeleeAPRef() = ap;
        }
        else if (pfi.rangedAP != 0xFFFF && key == pfi.rangedAP) {
            const int32_t ap = static_cast<int32_t>(val);
            // Its own flag, because its own event is the one that lands.
            if (owner_.playerRangedAPRef() != ap) rangedPowerChanged = true;
            owner_.playerRangedAPRef() = ap;
        }
        else if (pfi.spDmg1   != 0xFFFF && key >= pfi.spDmg1 && key < pfi.spDmg1 + 7) {
            const int32_t dmgVal = static_cast<int32_t>(val);
            if (owner_.playerSpellDmgBonusArr()[key - pfi.spDmg1] != dmgVal)
                spellBonusChanged = true;
            owner_.playerSpellDmgBonusArr()[key - pfi.spDmg1] = dmgVal;
        }
        else if (pfi.healBonus != 0xFFFF && key == pfi.healBonus) {
            const int32_t healVal = static_cast<int32_t>(val);
            if (owner_.playerHealBonusRef() != healVal) spellBonusChanged = true;
            owner_.playerHealBonusRef() = healVal;
        }
        // Percentage stats are stored as IEEE 754 floats packed into uint32 update fields.
        // memcpy reinterprets the bits; clamp to [0..100] to guard against NaN/Inf from
        // corrupted packets reaching the UI (display-only, no gameplay logic depends on these).
        // Points, not a percentage, so read straight rather than through the
        // float reinterpretation the lines below need.
        // Mana regen per second, sent already computed by the server (spirit,
        // intellect, mp5 from gear and any while-casting talent are all folded
        // in) as float bits, one figure while not casting and one during the
        // five-second rule. A change refreshes the stat panel like any stat.
        else if (pfi.manaRegen != 0xFFFF && key == pfi.manaRegen) {
            float mr; std::memcpy(&mr, &val, 4);
            if (owner_.playerManaRegenRef() != mr) statsChanged = true;
            owner_.playerManaRegenRef() = mr;
        }
        else if (pfi.manaRegenCasting != 0xFFFF && key == pfi.manaRegenCasting) {
            float mr; std::memcpy(&mr, &val, 4);
            if (owner_.playerManaRegenCastingRef() != mr) statsChanged = true;
            owner_.playerManaRegenCastingRef() = mr;
        }
        else if (pfi.expertise != 0xFFFF && key == pfi.expertise) { owner_.playerExpertiseRef() = static_cast<int32_t>(val); }
        else if (pfi.offhandExpertise != 0xFFFF && key == pfi.offhandExpertise) { owner_.playerOffhandExpertiseRef() = static_cast<int32_t>(val); }
        else if (pfi.blockPct != 0xFFFF && key == pfi.blockPct) { std::memcpy(&owner_.playerBlockPctRef(), &val, 4); owner_.playerBlockPctRef() = std::clamp(owner_.playerBlockPctRef(), 0.0f, 100.0f); }
        else if (pfi.dodgePct != 0xFFFF && key == pfi.dodgePct) { std::memcpy(&owner_.playerDodgePctRef(), &val, 4); owner_.playerDodgePctRef() = std::clamp(owner_.playerDodgePctRef(), 0.0f, 100.0f); }
        else if (pfi.parryPct != 0xFFFF && key == pfi.parryPct) { std::memcpy(&owner_.playerParryPctRef(), &val, 4); owner_.playerParryPctRef() = std::clamp(owner_.playerParryPctRef(), 0.0f, 100.0f); }
        else if (pfi.critPct  != 0xFFFF && key == pfi.critPct)  { std::memcpy(&owner_.playerCritPctRef(),  &val, 4); owner_.playerCritPctRef()  = std::clamp(owner_.playerCritPctRef(),  0.0f, 100.0f); }
        else if (pfi.rangedCritPct != 0xFFFF && key == pfi.rangedCritPct) { std::memcpy(&owner_.playerRangedCritPctRef(), &val, 4); owner_.playerRangedCritPctRef() = std::clamp(owner_.playerRangedCritPctRef(), 0.0f, 100.0f); }
        else if (pfi.sCrit1   != 0xFFFF && key >= pfi.sCrit1 && key < pfi.sCrit1 + 7) {
            std::memcpy(&owner_.playerSpellCritPctArr()[key - pfi.sCrit1], &val, 4);
        }
        else if (pfi.rating1  != 0xFFFF && key >= pfi.rating1 && key < pfi.rating1 + 25) {
            const int32_t r = static_cast<int32_t>(val);
            if (owner_.playerCombatRatingsRef()[key - pfi.rating1] != r) ratingsChanged = true;
            owner_.playerCombatRatingsRef()[key - pfi.rating1] = r;
        }
        else {
            for (int si = 0; si < 5; ++si) {
                if (pfi.stats[si] != 0xFFFF && key == pfi.stats[si]) {
                    const int32_t sv = static_cast<int32_t>(val);
                    if (owner_.playerStatsArr()[si] != sv) statsChanged = true;
                    owner_.playerStatsArr()[si] = sv;
                    break;
                }
            }
        }
        // Do not synthesize quest-log entries from raw update-field slots.
        // Slot layouts differ on some classic-family realms and can produce
        // phantom "already accepted" quests that block quest acceptance.
    }
    if (owner_.applyInventoryFields(fields)) slotsChanged = true;

    // Announced only on an update, never on the create block: at create the
    // sheet has not been built yet, and every field looks like a change.
    //
    // PaperDollFrame listens for these and calls PaperDollFrame_UpdateStats
    // from each. Without them the panels were filled once at login and then
    // never again - a new weapon or a stat buff left the old numbers on screen
    // with nothing to say they were stale.
    if (!isCreate) {
        if (statsChanged)   pendingEvents_.emit("UNIT_STATS", {"player"});
        if (ratingsChanged) pendingEvents_.emit("COMBAT_RATING_UPDATE", {});
        if (powerChanged)   pendingEvents_.emit("UNIT_ATTACK_POWER", {"player"});
        // The ranged half separately, because the two are not
        // interchangeable to the sheet. PaperDollFrame registers
        // UNIT_ATTACK_POWER and then handles it nowhere - it is absent from
        // the branch that calls PaperDollFrame_UpdateStats and from every
        // other branch in the file, in this FrameXML and in the pet one. So
        // sending it for a ranged change refreshed nothing at all: a hunter
        // could gain or lose ranged attack power and the character sheet went
        // on showing the old number.
        //
        // UNIT_RANGED_ATTACK_POWER is in that branch, and is what the value
        // actually changed, so it is both the effective event and the true
        // one.
        if (rangedPowerChanged) {
            pendingEvents_.emit("UNIT_RANGED_ATTACK_POWER", {"player"});
        }
        // Armor with the six resistances behind it, and the spell power and
        // healing bonus. The paperdoll refreshes its resistance panel from
        // UNIT_RESISTANCES alone - UNIT_DEFENSE is the *pet* sheet's event, not
        // this one - and its spell-power panel shares a branch with UNIT_STATS,
        // which only fires when one of the five base stats moves. So a spell
        // power buff that touched no stat, or a resistance aura, left the old
        // number on the sheet until the next login.
        if (resistancesChanged) pendingEvents_.emit("UNIT_RESISTANCES", {"player"});
        if (spellBonusChanged)  pendingEvents_.emit("PLAYER_DAMAGE_DONE_MODS", {"player"});
    }
    return slotsChanged;
}

//     Dispatch entity spawn callbacks for units/players.
//     Consolidates player/creature spawn callback invocation from CREATE and VALUES handlers.
//     isDead = unitInitiallyDead (CREATE) or computed isDeadNow && !npcDeathNotified (VALUES).
void EntityController::dispatchEntitySpawn(uint64_t guid, ObjectType objectType,
                                            const std::shared_ptr<Entity>& entity,
                                            const std::shared_ptr<Unit>& unit,
                                            bool isDead) {
    if (objectType == ObjectType::PLAYER && guid == owner_.getPlayerGuid()) {
        return;  // Skip local player - spawned separately via spawnPlayerCharacter()
    }
    if (objectType == ObjectType::PLAYER) {
        if (owner_.playerSpawnCallbackRef()) {
            uint8_t race = 0, gender = 0, facial = 0;
            uint32_t appearanceBytes = 0;
            if (extractPlayerAppearance(entity->getFields(), race, gender, appearanceBytes, facial)) {
                owner_.playerSpawnCallbackRef()(guid, unit->getDisplayId(), race, gender,
                                    appearanceBytes, facial,
                                    unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
            } else {
                LOG_WARNING("[Spawn] PLAYER guid=0x", std::hex, guid, std::dec,
                          " displayId=", unit->getDisplayId(), " appearance extraction failed - model will not render");
            }
        }
    } else if (owner_.creatureSpawnCallbackRef()) {
        LOG_DEBUG("[Spawn] UNIT guid=0x", std::hex, guid, std::dec,
                  " displayId=", unit->getDisplayId(), " at (",
                  unit->getX(), ",", unit->getY(), ",", unit->getZ(), ")");
        float unitScale = 1.0f;
        uint16_t scaleIdx = fieldIndex(UF::OBJECT_FIELD_SCALE_X);
        if (scaleIdx != 0xFFFF) {
            // raw == 0 means the field was never populated (IEEE 754 0.0f is all-zero bits).
            // Keep the default 1.0f rather than setting scale to 0 and making the entity invisible.
            uint32_t raw = entity->getField(scaleIdx);
            if (raw != 0) {
                unitScale = bitsToFloat(raw);
                if (unitScale <= 0.01f || unitScale > 100.0f) unitScale = 1.0f;
            }
        }
        owner_.creatureSpawnCallbackRef()(guid, unit->getDisplayId(),
            unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation(), unitScale);
    }
    if (isDead && owner_.npcDeathCallbackRef()) {
        owner_.npcDeathCallbackRef()(guid);
    }
    // Query quest giver status for NPCs with questgiver flag (0x02)
    if (objectType == ObjectType::UNIT && (unit->getNpcFlags() & 0x02) && owner_.getSocket()) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(guid);
        owner_.getSocket()->send(qsPkt);
    }
}

// Track online item/container objects during CREATE.
void EntityController::trackItemOnCreate(const UpdateBlock& block, bool& newItemCreated) {
    auto entryIt = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
    auto stackIt = block.fields.find(fieldIndex(UF::ITEM_FIELD_STACK_COUNT));
    auto durIt   = block.fields.find(fieldIndex(UF::ITEM_FIELD_DURABILITY));
    auto maxDurIt= block.fields.find(fieldIndex(UF::ITEM_FIELD_MAXDURABILITY));
    // ITEM_FIELD_FLAGS is 7 fields after STACK_COUNT (DURATION + 5×SPELL_CHARGES = 6, then
    // FLAGS) across every expansion; the enchant block that follows starts at +8.
    const uint16_t flagsField = (fieldIndex(UF::ITEM_FIELD_STACK_COUNT) != 0xFFFF)
        ? static_cast<uint16_t>(fieldIndex(UF::ITEM_FIELD_STACK_COUNT) + 7u) : 0xFFFFu;
    // ITEM_FIELD_RANDOM_PROPERTIES_ID sits just before ITEM_FIELD_DURABILITY: pre-WotLK an
    // ITEM_TEXT_ID field separates them (DUR-2), WotLK dropped that field so it is DUR-1.
    const uint16_t randPropField = (fieldIndex(UF::ITEM_FIELD_DURABILITY) != 0xFFFF)
        ? static_cast<uint16_t>(fieldIndex(UF::ITEM_FIELD_DURABILITY) - (isPreWotlk() ? 2u : 1u))
        : 0xFFFFu;
    // ITEM_FIELD_PROPERTY_SEED (the random-suffix stat scale) sits immediately before
    // RANDOM_PROPERTIES_ID; the client multiplies each suffix allocation % by it.
    const uint16_t seedField = (randPropField != 0xFFFF && randPropField > 0)
        ? static_cast<uint16_t>(randPropField - 1u) : 0xFFFFu;
    auto flagsIt    = (flagsField != 0xFFFF)    ? block.fields.find(flagsField)    : block.fields.end();
    auto randPropIt = (randPropField != 0xFFFF) ? block.fields.find(randPropField) : block.fields.end();
    auto seedIt     = (seedField != 0xFFFF)     ? block.fields.find(seedField)     : block.fields.end();
    const uint16_t enchBase = (fieldIndex(UF::ITEM_FIELD_STACK_COUNT) != 0xFFFF)
        ? static_cast<uint16_t>(fieldIndex(UF::ITEM_FIELD_STACK_COUNT) + 8u) : 0xFFFFu;
    auto permEnchIt  = (enchBase != 0xFFFF) ? block.fields.find(enchBase)       : block.fields.end();
    auto tempEnchIt  = (enchBase != 0xFFFF) ? block.fields.find(enchBase + 3u)  : block.fields.end();
    auto sock1EnchIt = (enchBase != 0xFFFF) ? block.fields.find(enchBase + 6u)  : block.fields.end();
    auto sock2EnchIt = (enchBase != 0xFFFF) ? block.fields.find(enchBase + 9u)  : block.fields.end();
    auto sock3EnchIt = (enchBase != 0xFFFF) ? block.fields.find(enchBase + 12u) : block.fields.end();
    if (entryIt != block.fields.end() && entryIt->second != 0) {
        // Preserve existing info when doing partial updates
        GameHandler::OnlineItemInfo prev = owner_.onlineItemsRef().count(block.guid)
            ? owner_.onlineItemsRef()[block.guid] : GameHandler::OnlineItemInfo{};
        GameHandler::OnlineItemInfo info = prev;
        info.entry = entryIt->second;
        if (stackIt    != block.fields.end()) info.stackCount            = stackIt->second;
        if (durIt      != block.fields.end()) info.curDurability         = durIt->second;
        if (maxDurIt   != block.fields.end()) info.maxDurability         = maxDurIt->second;
        if (permEnchIt != block.fields.end()) info.permanentEnchantId    = permEnchIt->second;
        if (tempEnchIt != block.fields.end()) info.temporaryEnchantId    = tempEnchIt->second;
        if (sock1EnchIt != block.fields.end()) info.socketEnchantIds[0]  = sock1EnchIt->second;
        if (sock2EnchIt != block.fields.end()) info.socketEnchantIds[1]  = sock2EnchIt->second;
        if (sock3EnchIt != block.fields.end()) info.socketEnchantIds[2]  = sock3EnchIt->second;
        if (flagsIt    != block.fields.end()) info.flags                 = flagsIt->second;
        if (randPropIt != block.fields.end()) info.randomPropertyId      = static_cast<int32_t>(randPropIt->second);
        if (seedIt     != block.fields.end()) info.suffixFactor          = seedIt->second;
        auto [itemIt, isNew] = owner_.onlineItemsRef().insert_or_assign(block.guid, info);
        // A CREATE_OBJECT that re-sends an already-tracked item with a changed stack
        // count (AzerothCore does this when crafting consumes a reagent) must still
        // refresh the built inventory - otherwise bag and crafting-window counts stay
        // stale until some later rebuild. Flag the batch rebuild on any tracked change,
        // not just brand-new items.
        const bool itemChanged = isNew ||
            info.stackCount        != prev.stackCount ||
            info.curDurability     != prev.curDurability ||
            info.maxDurability     != prev.maxDurability ||
            info.entry             != prev.entry ||
            info.flags             != prev.flags ||
            info.randomPropertyId  != prev.randomPropertyId ||
            info.suffixFactor      != prev.suffixFactor ||
            info.permanentEnchantId != prev.permanentEnchantId ||
            info.temporaryEnchantId != prev.temporaryEnchantId ||
            info.socketEnchantIds  != prev.socketEnchantIds;
        if (itemChanged) newItemCreated = true;
        owner_.queryItemInfo(info.entry, block.guid);
    }
    // Extract container slot GUIDs for bags
    if (block.objectType == ObjectType::CONTAINER) {
        owner_.extractContainerFields(block.guid, block.fields);
    }
}

// Update item stack count / durability / enchants for existing items during VALUES.
void EntityController::updateItemOnValuesUpdate(const UpdateBlock& block,
                                                  const std::shared_ptr<Entity>& entity) {
    bool inventoryChanged = false;
    // Tracked apart from inventoryChanged so the durability events fire on a
    // durability change and not on every stack count that moves.
    bool durabilityChanged = false;
    const uint16_t itemStackField   = fieldIndex(UF::ITEM_FIELD_STACK_COUNT);
    const uint16_t itemDurField     = fieldIndex(UF::ITEM_FIELD_DURABILITY);
    const uint16_t itemMaxDurField  = fieldIndex(UF::ITEM_FIELD_MAXDURABILITY);
    const uint16_t containerNumSlotsField = fieldIndex(UF::CONTAINER_FIELD_NUM_SLOTS);
    const uint16_t containerSlot1Field = fieldIndex(UF::CONTAINER_FIELD_SLOT_1);
    // ITEM_FIELD_ENCHANTMENT starts 8 fields after ITEM_FIELD_STACK_COUNT (fixed offset
    // across all expansions: +DURATION, +5×SPELL_CHARGES, +FLAGS = +8).
    // Slot 0 = permanent (field +0), slot 1 = temp (+3), slots 2-4 = sockets (+6,+9,+12).
    const uint16_t itemEnchBase      = (itemStackField != 0xFFFF) ? (itemStackField + 8u)  : 0xFFFF;
    const uint16_t itemPermEnchField = itemEnchBase;
    const uint16_t itemTempEnchField = (itemEnchBase != 0xFFFF) ? (itemEnchBase + 3u)  : 0xFFFF;
    const uint16_t itemSock1EnchField= (itemEnchBase != 0xFFFF) ? (itemEnchBase + 6u)  : 0xFFFF;
    const uint16_t itemSock2EnchField= (itemEnchBase != 0xFFFF) ? (itemEnchBase + 9u)  : 0xFFFF;
    const uint16_t itemSock3EnchField= (itemEnchBase != 0xFFFF) ? (itemEnchBase + 12u) : 0xFFFF;
    // ITEM_FIELD_FLAGS (STACK+7) carries the soulbound bit that a BoE item gains on equip;
    // RANDOM_PROPERTIES_ID sits before DURABILITY (DUR-2 pre-WotLK, DUR-1 on WotLK).
    const uint16_t itemFlagsField    = (itemStackField != 0xFFFF) ? (itemStackField + 7u) : 0xFFFF;
    const uint16_t itemRandPropField = (itemDurField != 0xFFFF)
        ? static_cast<uint16_t>(itemDurField - (isPreWotlk() ? 2u : 1u)) : 0xFFFF;
    const uint16_t itemSeedField = (itemRandPropField != 0xFFFF && itemRandPropField > 0)
        ? static_cast<uint16_t>(itemRandPropField - 1u) : 0xFFFF;

    auto it = owner_.onlineItemsRef().find(block.guid);
    bool isItemInInventory = (it != owner_.onlineItemsRef().end());

    for (const auto& [key, val] : block.fields) {
        if (key == itemStackField && isItemInInventory) {
            if (it->second.stackCount != val) {
                it->second.stackCount = val;
                inventoryChanged = true;
            }
        } else if (key == itemDurField && isItemInInventory) {
            if (it->second.curDurability != val) {
                const uint32_t prevDur = it->second.curDurability;
                it->second.curDurability = val;
                inventoryChanged = true;
                durabilityChanged = true;
                LOG_DEBUG("Item durability update: guid=0x", std::hex, block.guid,
                          std::dec, " dur ", prevDur, "->", val, "/", it->second.maxDurability);
                // Warn once when durability drops below 20% for an equipped item.
                const uint32_t maxDur = it->second.maxDurability;
                if (maxDur > 0 && val < maxDur / 5u && prevDur >= maxDur / 5u) {
                    // Check if this item is in an equip slot (not bag inventory).
                    bool isEquipped = false;
                    for (uint64_t slotGuid : owner_.equipSlotGuidsRef()) {
                        if (slotGuid == block.guid) { isEquipped = true; break; }
                    }
                    if (isEquipped) {
                        std::string itemName;
                        const auto* info = owner_.getItemInfo(it->second.entry);
                        if (info) itemName = info->name;
                        char buf[128];
                        if (!itemName.empty())
                            std::snprintf(buf, sizeof(buf), "%s is about to break!", itemName.c_str());
                        else
                            std::snprintf(buf, sizeof(buf), "An equipped item is about to break!");
                        owner_.addUIError(buf);
                        owner_.addSystemChatMessage(buf);
                    }
                }
            }
        } else if (key == itemMaxDurField && isItemInInventory) {
            if (it->second.maxDurability != val) {
                it->second.maxDurability = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemPermEnchField != 0xFFFF && key == itemPermEnchField) {
            if (it->second.permanentEnchantId != val) {
                it->second.permanentEnchantId = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemTempEnchField != 0xFFFF && key == itemTempEnchField) {
            if (it->second.temporaryEnchantId != val) {
                it->second.temporaryEnchantId = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemSock1EnchField != 0xFFFF && key == itemSock1EnchField) {
            if (it->second.socketEnchantIds[0] != val) {
                it->second.socketEnchantIds[0] = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemSock2EnchField != 0xFFFF && key == itemSock2EnchField) {
            if (it->second.socketEnchantIds[1] != val) {
                it->second.socketEnchantIds[1] = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemSock3EnchField != 0xFFFF && key == itemSock3EnchField) {
            if (it->second.socketEnchantIds[2] != val) {
                it->second.socketEnchantIds[2] = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemFlagsField != 0xFFFF && key == itemFlagsField) {
            if (it->second.flags != val) {
                it->second.flags = val;
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemRandPropField != 0xFFFF && key == itemRandPropField) {
            if (it->second.randomPropertyId != static_cast<int32_t>(val)) {
                it->second.randomPropertyId = static_cast<int32_t>(val);
                inventoryChanged = true;
            }
        } else if (isItemInInventory && itemSeedField != 0xFFFF && key == itemSeedField) {
            if (it->second.suffixFactor != val) {
                it->second.suffixFactor = val;
                inventoryChanged = true;
            }
        }
    }
    // Update container slot GUIDs on bag content changes
    if (entity && entity->getType() == ObjectType::CONTAINER) {
        for (const auto& [key, _] : block.fields) {
            if ((containerNumSlotsField != 0xFFFF && key == containerNumSlotsField) ||
                (containerSlot1Field != 0xFFFF && key >= containerSlot1Field && key < containerSlot1Field + 72)) {
                inventoryChanged = true;
                break;
            }
        }
        owner_.extractContainerFields(block.guid, block.fields);
    }
    if (inventoryChanged) {
        owner_.rebuildOnlineInventory();
        // One per bag: the interface redraws only the bag whose id matches
        // the event's argument.
        for (int bag = 0; bag <= 4; ++bag) {
            pendingEvents_.emit("BAG_UPDATE", {std::to_string(bag)});
        }
        LOG_WARNING("BAG_UPDATE fired for bags 0-4 (inventory fields changed)");
        pendingEvents_.emit("UNIT_INVENTORY_CHANGED", {"player"});
        // An item on the action bar shows how many are left, and nothing in
        // FrameXML recomputes that from a bag change: ActionButton_UpdateCount
        // runs only from ActionButton_Update, which BAG_UPDATE does not reach.
        // So a stack of bandages went down and the bar kept the number it had
        // when the button was last drawn - usually the count at login.
        //
        // Slot zero means every button, which is what the interface reads it
        // as. Only sent when something on the bar is an item, so a bag change
        // does not rebuild a hundred and twenty buttons for nothing.
        for (const auto& action : owner_.getActionBar()) {
            if (action.type == ActionBarSlot::ITEM && action.id != 0) {
                pendingEvents_.emit("ACTIONBAR_SLOT_CHANGED", {"0"});
                break;
            }
        }
    }
    if (durabilityChanged) {
        // The armour indicator is DurabilityFrame, which listens for
        // UPDATE_INVENTORY_ALERTS; the merchant's repair buttons listen for
        // UPDATE_INVENTORY_DURABILITY. Neither was fired anywhere but at world
        // entry, so the indicator showed the durability the player logged in
        // with -- a repair changed nothing on screen, and neither did damage.
        pendingEvents_.emit("UPDATE_INVENTORY_ALERTS", {});
        pendingEvents_.emit("UPDATE_INVENTORY_DURABILITY", {});
    }
}

// ============================================================
// Object-type handler struct definitions
// ============================================================

struct EntityController::UnitTypeHandler : EntityController::IObjectTypeHandler {
    EntityController& ctl_;
    explicit UnitTypeHandler(EntityController& c) : ctl_(c) {}
    void onCreate(const UpdateBlock& block, std::shared_ptr<Entity>& entity, bool&) override { ctl_.onCreateUnit(block, entity); }
    void onValuesUpdate(const UpdateBlock& block, std::shared_ptr<Entity>& entity) override { ctl_.onValuesUpdateUnit(block, entity); }
};

struct EntityController::PlayerTypeHandler : EntityController::IObjectTypeHandler {
    EntityController& ctl_;
    explicit PlayerTypeHandler(EntityController& c) : ctl_(c) {}
    void onCreate(const UpdateBlock& block, std::shared_ptr<Entity>& entity, bool&) override { ctl_.onCreatePlayer(block, entity); }
    void onValuesUpdate(const UpdateBlock& block, std::shared_ptr<Entity>& entity) override { ctl_.onValuesUpdatePlayer(block, entity); }
};

struct EntityController::GameObjectTypeHandler : EntityController::IObjectTypeHandler {
    EntityController& ctl_;
    explicit GameObjectTypeHandler(EntityController& c) : ctl_(c) {}
    void onCreate(const UpdateBlock& block, std::shared_ptr<Entity>& entity, bool&) override { ctl_.onCreateGameObject(block, entity); }
    void onValuesUpdate(const UpdateBlock& block, std::shared_ptr<Entity>& entity) override { ctl_.onValuesUpdateGameObject(block, entity); }
};

struct EntityController::ItemTypeHandler : EntityController::IObjectTypeHandler {
    EntityController& ctl_;
    explicit ItemTypeHandler(EntityController& c) : ctl_(c) {}
    void onCreate(const UpdateBlock& block, std::shared_ptr<Entity>& /*entity*/, bool& newItemCreated) override { ctl_.onCreateItem(block, newItemCreated); }
    void onValuesUpdate(const UpdateBlock& block, std::shared_ptr<Entity>& entity) override { ctl_.onValuesUpdateItem(block, entity); }
};

struct EntityController::CorpseTypeHandler : EntityController::IObjectTypeHandler {
    EntityController& ctl_;
    explicit CorpseTypeHandler(EntityController& c) : ctl_(c) {}
    void onCreate(const UpdateBlock& block, std::shared_ptr<Entity>& /*entity*/, bool&) override { ctl_.onCreateCorpse(block); }
};

// ============================================================
// Handler registry infrastructure
// ============================================================

void EntityController::initTypeHandlers() {
    typeHandlers_[static_cast<uint8_t>(ObjectType::UNIT)] = std::make_unique<UnitTypeHandler>(*this);
    typeHandlers_[static_cast<uint8_t>(ObjectType::PLAYER)] = std::make_unique<PlayerTypeHandler>(*this);
    typeHandlers_[static_cast<uint8_t>(ObjectType::GAMEOBJECT)] = std::make_unique<GameObjectTypeHandler>(*this);
    typeHandlers_[static_cast<uint8_t>(ObjectType::ITEM)] = std::make_unique<ItemTypeHandler>(*this);
    typeHandlers_[static_cast<uint8_t>(ObjectType::CONTAINER)] = std::make_unique<ItemTypeHandler>(*this);
    typeHandlers_[static_cast<uint8_t>(ObjectType::CORPSE)] = std::make_unique<CorpseTypeHandler>(*this);
}

EntityController::IObjectTypeHandler* EntityController::getTypeHandler(ObjectType type) const {
    auto it = typeHandlers_.find(static_cast<uint8_t>(type));
    return it != typeHandlers_.end() ? it->second.get() : nullptr;
}

// ============================================================
// Deferred event bus flush
// ============================================================

void EntityController::flushPendingEvents() {
    for (const auto& [name, args] : pendingEvents_.events) {
        owner_.fireAddonEvent(name, args);
    }
    pendingEvents_.clear();
}

// ============================================================
// Type-specific CREATE handlers
// ============================================================

void EntityController::onCreateUnit(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    // Name query for creatures
    auto it = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
    if (it != block.fields.end() && it->second != 0) {
        auto unit = std::static_pointer_cast<Unit>(entity);
        unit->setEntry(it->second);
        std::string cached = getCachedCreatureName(it->second);
        if (!cached.empty()) {
            unit->setName(cached);
        }
        queryCreatureInfo(it->second, block.guid);
    }

    // Unit fields
    auto unit = std::static_pointer_cast<Unit>(entity);
    UnitFieldIndices ufi = UnitFieldIndices::resolve();
    bool unitInitiallyDead = applyUnitFieldsOnCreate(block, unit, ufi);

    // Hostility
    unit->setHostile(owner_.isHostileFaction(unit->getFactionTemplate()));

    // Spawn dispatch
    if (unit->getDisplayId() == 0) {
        LOG_WARNING("[Spawn] UNIT guid=0x", std::hex, block.guid, std::dec,
                  " has displayId=0 - no spawn (entry=", unit->getEntry(),
                  " at ", unit->getX(), ",", unit->getY(), ",", unit->getZ(), ")");
    }
    if (unit->getDisplayId() != 0) {
        dispatchEntitySpawn(block.guid, block.objectType, entity, unit, unitInitiallyDead);
        if (block.hasMovement && block.moveFlags != 0 && owner_.unitMoveFlagsCallbackRef() &&
            block.guid != owner_.getPlayerGuid()) {
            owner_.unitMoveFlagsCallbackRef()(block.guid, block.moveFlags, ~0u);
        }
    }
}

void EntityController::onCreatePlayer(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    static const bool kVerboseUpdateObject = core::envFlagEnabled("WOWEE_LOG_UPDATE_OBJECT_VERBOSE", false);

    // For the local player, capture the full initial field state
    if (block.guid == owner_.getPlayerGuid()) {
        owner_.lastPlayerFieldsRef() = entity->getFields();
        owner_.maybeDetectVisibleItemLayout();
    }

    // Name query + visible items
    queryPlayerName(block.guid);
    if (block.guid != owner_.getPlayerGuid()) {
        owner_.updateOtherPlayerVisibleItems(block.guid, entity->getFields());
    }

    // Unit fields (PLAYER is a unit)
    auto unit = std::static_pointer_cast<Unit>(entity);
    UnitFieldIndices ufi = UnitFieldIndices::resolve();
    bool unitInitiallyDead = applyUnitFieldsOnCreate(block, unit, ufi);

    // Self-player post-unit-field handling
    if (block.guid == owner_.getPlayerGuid()) {
        if ((unit->getUnitFlags() & UNIT_FLAG_TAXI_FLIGHT) != 0 && !owner_.onTaxiFlightRef() && owner_.taxiLandingCooldownRef() <= 0.0f) {
            owner_.onTaxiFlightRef() = true;
            owner_.taxiStartGraceRef() = std::max(owner_.taxiStartGraceRef(), 2.0f);
            owner_.sanitizeMovementForTaxi();
            if (owner_.getMovementHandler()) owner_.getMovementHandler()->applyTaxiMountForCurrentNode();
        }
    }
    if (block.guid == owner_.getPlayerGuid() &&
        (unit->getDynamicFlags() & UNIT_DYNFLAG_DEAD) != 0) {
        owner_.playerDeadRef() = true;
        LOG_INFO("Player logged in dead (dynamic flags)");
    }
    // Detect ghost state on login via PLAYER_FLAGS
    if (block.guid == owner_.getPlayerGuid()) {
        constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
        auto pfIt = block.fields.find(fieldIndex(UF::PLAYER_FLAGS));
        if (pfIt != block.fields.end() && (pfIt->second & PLAYER_FLAGS_GHOST) != 0) {
            owner_.releasedSpiritRef() = true;
            owner_.playerDeadRef() = true;
            LOG_INFO("Player logged in as ghost (PLAYER_FLAGS)");
            if (owner_.ghostStateCallbackRef()) owner_.ghostStateCallbackRef()(true);
            // Query corpse position so minimap marker is accurate on reconnect
            if (owner_.getSocket()) {
                network::Packet cq(wireOpcode(Opcode::MSG_CORPSE_QUERY));
                owner_.getSocket()->send(cq);
            }
        }
    }
    // Pre-WotLK aura sync on initial object create
    if (block.guid == owner_.getPlayerGuid()) {
        syncPreWotlkAurasFromFields(entity);
    }

    // Hostility
    unit->setHostile(owner_.isHostileFaction(unit->getFactionTemplate()));

    // Spawn dispatch
    if (unit->getDisplayId() != 0) {
        dispatchEntitySpawn(block.guid, block.objectType, entity, unit, unitInitiallyDead);
        if (block.hasMovement && block.moveFlags != 0 && owner_.unitMoveFlagsCallbackRef() &&
            block.guid != owner_.getPlayerGuid()) {
            owner_.unitMoveFlagsCallbackRef()(block.guid, block.moveFlags, ~0u);
        }
    }

    // Player stat fields (self only)
    if (block.guid == owner_.getPlayerGuid()) {
        // Auto-detect coinage index using the previous snapshot vs this full snapshot.
        maybeDetectCoinageIndex(owner_.lastPlayerFieldsRef(), block.fields);

        owner_.lastPlayerFieldsRef() = block.fields;
        owner_.detectInventorySlotBases(block.fields);

        if (kVerboseUpdateObject) {
            uint16_t maxField = 0;
            for (const auto& [key, _val] : block.fields) {
                if (key > maxField) maxField = key;
            }
            LOG_INFO("Player update with ", block.fields.size(),
                     " fields (max index=", maxField, ")");
        }

        PlayerFieldIndices pfi = PlayerFieldIndices::resolve();
        bool slotsChanged = applyPlayerStatFields(block.fields, pfi, true);
        if (slotsChanged) owner_.rebuildOnlineInventory();
        owner_.maybeDetectVisibleItemLayout();
        owner_.extractSkillFields(owner_.lastPlayerFieldsRef());
        owner_.extractExploredZoneFields(owner_.lastPlayerFieldsRef());
        owner_.applyQuestStateFromFields(owner_.lastPlayerFieldsRef());
    }
}

void EntityController::onCreateGameObject(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    auto go = std::static_pointer_cast<GameObject>(entity);
    auto itDisp = block.fields.find(fieldIndex(UF::GAMEOBJECT_DISPLAYID));
    if (itDisp != block.fields.end()) {
        go->setDisplayId(itDisp->second);
    }
    auto itEntry = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
    if (itEntry != block.fields.end() && itEntry->second != 0) {
        go->setEntry(itEntry->second);
        auto cacheIt = gameObjectInfoCache_.find(itEntry->second);
        if (cacheIt != gameObjectInfoCache_.end()) {
            go->setName(cacheIt->second.name);
        }
        queryGameObjectInfo(itEntry->second, block.guid);
    }
    // Detect transport GameObjects via UPDATEFLAG_TRANSPORT (0x0002)
    LOG_DEBUG("GameObject CREATE: guid=0x", std::hex, block.guid, std::dec,
             " entry=", go->getEntry(), " displayId=", go->getDisplayId(),
             " updateFlags=0x", std::hex, block.updateFlags, std::dec,
             " pos=(", go->getX(), ", ", go->getY(), ", ", go->getZ(), ")");
    if (block.updateFlags & 0x0002) {
        transportGuids_.insert(block.guid);
        LOG_INFO("Detected transport GameObject: 0x", std::hex, block.guid, std::dec,
                 " entry=", go->getEntry(),
                 " displayId=", go->getDisplayId(),
                 " pos=(", go->getX(), ", ", go->getY(), ", ", go->getZ(), ")");
        // Note: TransportSpawnCallback will be invoked from Application after WMO instance is created
    }
    if (go->getDisplayId() != 0 && owner_.gameObjectSpawnCallbackRef()) {
        float goScale = 1.0f;
        {
            uint16_t scaleIdx = fieldIndex(UF::OBJECT_FIELD_SCALE_X);
            if (scaleIdx != 0xFFFF) {
                uint32_t raw = entity->getField(scaleIdx);
                if (raw != 0) {
                    goScale = bitsToFloat(raw);
                    if (goScale <= 0.01f || goScale > 100.0f) goScale = 1.0f;
                }
            }
        }
        owner_.gameObjectSpawnCallbackRef()(block.guid, go->getEntry(), go->getDisplayId(),
            go->getX(), go->getY(), go->getZ(), go->getOrientation(), goScale);
    }
    // Fire transport move callback for transports (position update on re-creation).
    // NOTE: do NOT mark the guid as server-updated here. A CREATE only carries the
    // spawn position, not evidence that the server is authoritatively streaming this
    // transport's motion. Marking it here forces preferServerData=true at spawn
    // resolution, which puts client-animated ships/zeppelins into strict mode and
    // skips the entry->DBC path remap - WotLK ship GO entries don't match
    // TransportAnimation.dbc 1:1, so they were left stationary. Only genuine
    // movement updates (onValuesUpdateGameObject / MOVEMENT blocks) set that flag.
    if (transportGuids_.count(block.guid) && owner_.transportMoveCallbackRef()) {
        owner_.transportMoveCallbackRef()(block.guid,
            go->getX(), go->getY(), go->getZ(), go->getOrientation());
    }
}

void EntityController::onCreateItem(const UpdateBlock& block, bool& newItemCreated) {
    trackItemOnCreate(block, newItemCreated);
}

void EntityController::onCreateCorpse(const UpdateBlock& block) {
    // Detect player's own corpse object so we have the position even when
    // SMSG_DEATH_RELEASE_LOC hasn't been received (e.g. login as ghost).
    //
    // Not gated on hasMovement, and that gate was a bug of its own: a corpse
    // is a stationary object and its create block need not carry a movement
    // block at all. What reclaiming needs from here is the *guid* -
    // CMSG_RECLAIM_CORPSE names it, and canReclaimCorpse refuses while it is
    // zero - and the position can come from MSG_CORPSE_QUERY, which the
    // login-as-ghost path already sends. Gating the guid on the position
    // meant a ghost who walked back to a corpse that arrived without one
    // could never take it, however close they stood.
    {
        // CORPSE_FIELD_OWNER is at index 6 (uint64, low word at 6, high at 7)
        uint16_t ownerLowIdx = 6;
        auto ownerLowIt = block.fields.find(ownerLowIdx);
        uint32_t ownerLow = (ownerLowIt != block.fields.end()) ? ownerLowIt->second : 0;
        auto ownerHighIt = block.fields.find(ownerLowIdx + 1);
        uint32_t ownerHigh = (ownerHighIt != block.fields.end()) ? ownerHighIt->second : 0;
        uint64_t ownerGuid = (static_cast<uint64_t>(ownerHigh) << 32) | ownerLow;
        if (ownerGuid == owner_.getPlayerGuid() || ownerLow == static_cast<uint32_t>(owner_.getPlayerGuid())) {
            owner_.corpseGuidRef()  = block.guid;
            // The position only where the block actually carried one; a
            // stationary create may not, and overwriting a good answer from
            // MSG_CORPSE_QUERY with three zeroes would put the corpse at the
            // map origin and refuse the reclaim on distance instead.
            if (block.hasMovement) {
                owner_.corpseXRef()     = block.x;
                owner_.corpseYRef()     = block.y;
                owner_.corpseZRef()     = block.z;
                owner_.corpseMapIdRef() = owner_.currentMapIdRef();
                owner_.corpsePositionValidRef() = true;
            }

            // Corpse objects carry ownership and position but not a standalone
            // render model. Reuse the owning character's appearance and equipment
            // under the corpse GUID, then force the queued instance into DEATH.
            auto characterIt = std::find_if(owner_.charactersRef().begin(), owner_.charactersRef().end(),
                [&](const Character& character) { return character.guid == owner_.getPlayerGuid(); });
            if (characterIt != owner_.charactersRef().end() && owner_.playerSpawnCallbackRef()) {
                glm::vec3 canonical = core::coords::serverToCanonical(
                    glm::vec3(block.x, block.y, block.z));
                float orientation = core::coords::serverToCanonicalYaw(block.orientation);
                owner_.playerSpawnCallbackRef()(
                    block.guid, 0,
                    static_cast<uint8_t>(characterIt->race),
                    static_cast<uint8_t>(characterIt->gender),
                    characterIt->appearanceBytes,
                    characterIt->facialFeatures,
                    canonical.x, canonical.y, canonical.z, orientation);

                if (owner_.playerEquipmentCallbackRef()) {
                    std::array<uint32_t, 19> displayInfoIds{};
                    std::array<uint8_t, 19> inventoryTypes{};
                    const size_t count = std::min<size_t>(19, characterIt->equipment.size());
                    for (size_t i = 0; i < count; ++i) {
                        displayInfoIds[i] = characterIt->equipment[i].displayModel;
                        inventoryTypes[i] = characterIt->equipment[i].inventoryType;
                    }
                    owner_.playerEquipmentCallbackRef()(block.guid, displayInfoIds, inventoryTypes);
                }
            }
            if (owner_.npcDeathCallbackRef()) {
                owner_.npcDeathCallbackRef()(block.guid);
            }
            LOG_INFO("Corpse object detected: guid=0x", std::hex, owner_.corpseGuidRef(), std::dec,
                     " server=(", block.x, ", ", block.y, ", ", block.z,
                     ") map=", owner_.corpseMapIdRef());
        }
    }
}

// ============================================================
// Type-specific VALUES UPDATE handlers
// ============================================================

void EntityController::handleDisplayIdChange(const UpdateBlock& block,
                                              const std::shared_ptr<Entity>& entity,
                                              const std::shared_ptr<Unit>& unit,
                                              const UnitFieldUpdateResult& result) {
    if (!result.displayIdChanged || unit->getDisplayId() == 0 ||
        unit->getDisplayId() == result.oldDisplayId)
        return;

    bool isDeadNow = isUnitCorpseState(
        unit->getHealth(), unit->getMaxHealth(), unit->getDynamicFlags());
    dispatchEntitySpawn(block.guid, entity->getType(), entity, unit,
                        isDeadNow && !result.npcDeathNotified);
    if (owner_.addonEventCallbackRef()) {
        std::string uid;
        if (block.guid == owner_.getTargetGuid()) uid = "target";
        else if (block.guid == owner_.focusGuidRef()) uid = "focus";
        else if (block.guid == owner_.petGuidRef()) uid = "pet";
        if (!uid.empty())
            pendingEvents_.emit("UNIT_MODEL_CHANGED", {uid});
    }
}

void EntityController::onValuesUpdateUnit(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    auto unit = std::static_pointer_cast<Unit>(entity);
    UnitFieldIndices ufi = UnitFieldIndices::resolve();
    UnitFieldUpdateResult result = applyUnitFieldsOnUpdate(block, entity, unit, ufi);
    handleDisplayIdChange(block, entity, unit, result);

    // A corpse that has been looted empty has nothing left to offer, so drop it.
    // Skinnable corpses stay until they have been skinned. Done here rather than
    // inside the field loop so the entity is not removed while it is being read.
    constexpr uint32_t UNIT_FLAG_SKINNABLE = 0x04000000;
    if (result.lootableCleared && unit->getHealth() == 0 &&
        (unit->getUnitFlags() & UNIT_FLAG_SKINNABLE) == 0) {
        owner_.despawnCreatureLocally(block.guid);
    }
}

void EntityController::onValuesUpdatePlayer(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    // Other player visible items
    if (block.guid != owner_.getPlayerGuid()) {
        owner_.updateOtherPlayerVisibleItems(block.guid, entity->getFields());
    }

    // Unit field update (player IS a unit)
    auto unit = std::static_pointer_cast<Unit>(entity);
    UnitFieldIndices ufi = UnitFieldIndices::resolve();
    UnitFieldUpdateResult result = applyUnitFieldsOnUpdate(block, entity, unit, ufi);

    // Pre-WotLK aura sync from UNIT_FIELD_AURAS when those fields are updated
    if (block.guid == owner_.getPlayerGuid()) {
        syncPreWotlkAurasFromFields(entity);
    }

    // Display ID changed - re-spawn/model-change
    handleDisplayIdChange(block, entity, unit, result);

    // Self-player stat/inventory/quest field updates
    if (block.guid == owner_.getPlayerGuid()) {
        const bool needCoinageDetectSnapshot =
            (owner_.pendingMoneyDeltaRef() != 0 && owner_.pendingMoneyDeltaTimerRef() > 0.0f);
        FlatFieldMap oldFieldsSnapshot;
        if (needCoinageDetectSnapshot) {
            oldFieldsSnapshot = owner_.lastPlayerFieldsRef();
        }
        if (block.hasMovement && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
            // Speed is independent of mount ownership: slows and ordinary movement
            // snapshots may report base-speed values while UNIT_FIELD_MOUNTDISPLAYID
            // still authoritatively says the player is mounted.
            if (auto* movement = owner_.getMovementHandler()) {
                movement->applyServerMovementSpeeds(
                    block.walkSpeed, block.runSpeed, block.runBackSpeed,
                    block.swimSpeed, block.swimBackSpeed, block.flightSpeed,
                    block.flightBackSpeed, block.turnRate, block.pitchRate);
            }
        }
        // Merge block fields into the persistent snapshot. Both are sorted, so
        // a simple insert_or_assign per key keeps the invariant intact.
        for (const auto& [key, val] : block.fields) {
            owner_.lastPlayerFieldsRef().insert_or_assign(key, val);
        }
        if (needCoinageDetectSnapshot) {
            maybeDetectCoinageIndex(oldFieldsSnapshot, owner_.lastPlayerFieldsRef());
        }
        owner_.maybeDetectVisibleItemLayout();
        owner_.detectInventorySlotBases(block.fields);

        PlayerFieldIndices pfi = PlayerFieldIndices::resolve();
        bool slotsChanged = applyPlayerStatFields(block.fields, pfi, false);
        if (slotsChanged) {
            owner_.rebuildOnlineInventory();
            pendingEvents_.emit("PLAYER_EQUIPMENT_CHANGED", {});
        }
        owner_.extractSkillFields(owner_.lastPlayerFieldsRef());
        owner_.extractExploredZoneFields(owner_.lastPlayerFieldsRef());
        owner_.applyQuestStateFromFields(owner_.lastPlayerFieldsRef());
    }
}

void EntityController::onValuesUpdateItem(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    updateItemOnValuesUpdate(block, entity);
}

void EntityController::onValuesUpdateGameObject(const UpdateBlock& block, std::shared_ptr<Entity>& entity) {
    if (block.hasMovement) {
        if (transportGuids_.count(block.guid) && owner_.transportMoveCallbackRef()) {
            serverUpdatedTransportGuids_.insert(block.guid);
            owner_.transportMoveCallbackRef()(block.guid, entity->getX(), entity->getY(),
                                   entity->getZ(), entity->getOrientation());
        } else if (owner_.gameObjectMoveCallbackRef()) {
            owner_.gameObjectMoveCallbackRef()(block.guid, entity->getX(), entity->getY(),
                                    entity->getZ(), entity->getOrientation());
        }
    }

    // Detect GO state changes from GAMEOBJECT_BYTES_1 (packed: byte0=state, byte1=type, byte2=artKit, byte3=animProgress)
    const uint16_t ufGoBytes1 = fieldIndex(UF::GAMEOBJECT_BYTES_1);
    if (ufGoBytes1 != 0xFFFF) {
        auto itB = block.fields.find(ufGoBytes1);
        if (itB != block.fields.end()) {
            uint8_t goState = static_cast<uint8_t>(itB->second & 0xFF);
            if (owner_.gameObjectStateCallbackRef())
                owner_.gameObjectStateCallbackRef()(block.guid, goState);
        }
    }

    applyTransportRouteClock(block);
}

void EntityController::trackActiveCritter(const UpdateBlock& block) {
    // A non-combat companion is announced only by this field on the player: it
    // gets no aura, so there is nothing in the buff bar to cancel and nothing
    // else to notice it by. Remembering which spell called it is what lets
    // casting that spell again put it away rather than summon a second one.
    if (block.guid != owner_.getPlayerGuid()) return;
    const uint16_t critterIdx = fieldIndex(UF::UNIT_FIELD_CRITTER);
    if (critterIdx == 0xFFFF) return;   // pre-WotLK: no such field, no dismiss opcode

    auto loIt = block.fields.find(critterIdx);
    auto hiIt = block.fields.find(static_cast<uint16_t>(critterIdx + 1));
    if (loIt == block.fields.end() && hiIt == block.fields.end()) return;

    const uint32_t lo = (loIt != block.fields.end()) ? loIt->second
                                                     : static_cast<uint32_t>(owner_.getActiveCritterGuid());
    const uint32_t hi = (hiIt != block.fields.end())
                            ? hiIt->second
                            : static_cast<uint32_t>(owner_.getActiveCritterGuid() >> 32);
    const uint64_t critterGuid = (static_cast<uint64_t>(hi) << 32) | lo;
    if (critterGuid == owner_.getActiveCritterGuid()) return;

    if (critterGuid == 0) {
        LOG_INFO("Companion dismissed: was guid=0x", std::hex,
                 owner_.getActiveCritterGuid(), std::dec);
        owner_.setActiveCritter(0, 0);
        return;
    }
    // Attribute it to the spell just cast from the ground, the same way the
    // mount aura is identified - a blind guess at "some spell" would make the
    // toggle fire on the wrong button.
    uint32_t summonedBy = 0;
    if (owner_.getSpellHandler()) summonedBy = owner_.getSpellHandler()->getLastGroundCastSpellId();
    owner_.setActiveCritter(critterGuid, summonedBy);
    LOG_INFO("Companion summoned: guid=0x", std::hex, critterGuid, std::dec,
             " by spell=", summonedBy);
}

void EntityController::applyTransportRouteClock(const UpdateBlock& block) {
    // A moving transport publishes where it is on its route: LEVEL is the route's
    // period in milliseconds, and the high int16 of DYNAMIC is how far through
    // that period it currently is, as a fraction of 65535.
    //
    // Without this the client animated on a clock it invented from distance over
    // speed, which is why a ferry could lap its shore several times while the
    // server's schedule caught up, and why a rider's world position - which the
    // client composes from its own idea of where the hull is - could disagree
    // with the server's.
    //
    // Both fields are WotLK-only. Nothing earlier published a transport's phase,
    // so on those expansions fieldIndex returns 0xFFFF and this does nothing.
    auto* tm = owner_.getTransportManager();
    if (!tm || !tm->getTransport(block.guid)) return;

    const uint16_t ufLevel = fieldIndex(UF::GAMEOBJECT_LEVEL);
    const uint16_t ufDynamic = fieldIndex(UF::GAMEOBJECT_DYNAMIC);
    if (ufLevel == 0xFFFF || ufDynamic == 0xFFFF) return;

    auto itPeriod = block.fields.find(ufLevel);
    auto itDynamic = block.fields.find(ufDynamic);
    if (itPeriod == block.fields.end() || itDynamic == block.fields.end()) return;

    const uint32_t periodMs = itPeriod->second;
    // Signed on the wire: -1 means "this object has no path progress to report",
    // which every non-transport GameObject sends.
    const int16_t rawProgress = static_cast<int16_t>((itDynamic->second >> 16) & 0xFFFF);
    if (periodMs == 0 || rawProgress < 0) return;

    const float phase = static_cast<float>(rawProgress) / 65535.0f;
    tm->applyServerRouteClock(block.guid, phase, periodMs);
}

// ============================================================
// Update type handlers
// ============================================================

void EntityController::resyncPlayerIfFarFromServer(const glm::vec3& serverCanonicalPos) {
    // The client owns its own position and the server echoes it back, so these
    // agree to within a step and this does nothing almost always. What it
    // catches is the case where they stop agreeing at all.
    //
    // Only the orientation was ever taken from these blocks. The position went
    // onto the player's entity and never into movementInfo, which is what the
    // client moves by and sends - so once the two diverged there was no way
    // back: the client kept walking from where it thought it was, the server
    // kept answering about where it thought the player was, and a relog put
    // the player wherever the server had them. Creatures arrive around the
    // server's position, which is why none of them are where the player is
    // standing.
    //
    // Far enough that ordinary lag cannot reach it: fifty yards is seven
    // seconds of running. Snapping on a smaller gap would fight the client's
    // own movement and rubber-band.
    constexpr float kResyncDistance = 50.0f;
    if (owner_.isOnTransport()) return;
    if (auto* movement = owner_.getMovementHandler()) {
        if (movement->isOnTaxiFlight()) return;
    }
    auto& mi = owner_.movementInfoRef();
    const glm::vec3 clientPos(mi.x, mi.y, mi.z);
    const float gap = glm::length(serverCanonicalPos - clientPos);
    if (gap <= kResyncDistance) return;

    LOG_WARNING("Player position desync: the server places the player ", gap,
                " yards away, at canonical ", serverCanonicalPos.x, ", ",
                serverCanonicalPos.y, ", ", serverCanonicalPos.z,
                " rather than ", clientPos.x, ", ", clientPos.y, ", ",
                clientPos.z, " - taking the server's");
    mi.x = serverCanonicalPos.x;
    mi.y = serverCanonicalPos.y;
    mi.z = serverCanonicalPos.z;
    if (owner_.playerPositionCorrectionCallbackRef()) {
        owner_.playerPositionCorrectionCallbackRef()(
            serverCanonicalPos.x, serverCanonicalPos.y, serverCanonicalPos.z);
    }
}

void EntityController::handleCreateObject(const UpdateBlock& block, bool& newItemCreated) {
    trackActiveCritter(block);
    pendingEvents_.clear();

    // 3a: Create entity from block type
    std::shared_ptr<Entity> entity = createEntityFromBlock(block);

    // Set position from movement block (server → canonical)
    if (block.hasMovement) {
        glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
        float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
        entity->setPosition(pos.x, pos.y, pos.z, oCanonical);
        LOG_DEBUG("  Position: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
        if (block.guid == owner_.getPlayerGuid() && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
            if (auto* movement = owner_.getMovementHandler()) {
                movement->applyServerMovementSpeeds(
                    block.walkSpeed, block.runSpeed, block.runBackSpeed,
                    block.swimSpeed, block.swimBackSpeed, block.flightSpeed,
                    block.flightBackSpeed, block.turnRate, block.pitchRate);
            }
        }
        // 3b: Track player-on-transport state
        if (block.guid == owner_.getPlayerGuid()) {
            applyPlayerTransportState(block, entity, pos, oCanonical, false);
        }
        // 3i: Track transport-relative children so they follow parent transport motion.
        updateNonPlayerTransportAttachment(block, entity, block.objectType);
    }

    // Set fields
    for (const auto& field : block.fields) {
        entity->setField(field.first, field.second);
    }

    // Add to manager
    entityManager.addEntity(block.guid, entity);

    // Dispatch to type-specific handler
    auto* handler = getTypeHandler(block.objectType);
    if (handler) handler->onCreate(block, entity, newItemCreated);

    flushPendingEvents();
}

void EntityController::handleValuesUpdate(const UpdateBlock& block) {
    trackActiveCritter(block);
    auto entity = entityManager.getEntity(block.guid);
    if (!entity) {
        // Item/container entities may be absent from entityManager (e.g. server
        // only sent a partial update) but we still track them in onlineItems_.
        // Process field updates so durability/stack changes from repair aren't lost.
        if (owner_.onlineItemsRef().count(block.guid)) {
            pendingEvents_.clear();
            updateItemOnValuesUpdate(block, entity);
            flushPendingEvents();
            LOG_DEBUG("Updated orphan item fields: 0x", std::hex, block.guid, std::dec);
        }
        return;
    }
    pendingEvents_.clear();

    // Position update (common)
    if (block.hasMovement) {
        glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
        float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
        entity->setPosition(pos.x, pos.y, pos.z, oCanonical);
        updateNonPlayerTransportAttachment(block, entity, entity->getType());
    }

    // Set fields (common)
    for (const auto& field : block.fields) {
        entity->setField(field.first, field.second);
    }

    // Dispatch to type-specific handler
    auto* handler = getTypeHandler(entity->getType());
    if (handler) handler->onValuesUpdate(block, entity);

    flushPendingEvents();
    LOG_DEBUG("Updated entity fields: 0x", std::hex, block.guid, std::dec);
}

void EntityController::handleMovementUpdate(const UpdateBlock& block) {
            // Diagnostic: Log if we receive MOVEMENT blocks for transports
            if (transportGuids_.count(block.guid)) {
                LOG_INFO("MOVEMENT update for transport 0x", std::hex, block.guid, std::dec,
                         " pos=(", block.x, ", ", block.y, ", ", block.z, ")");
            }

            // Update entity position (server → canonical)
            auto entity = entityManager.getEntity(block.guid);
            if (entity) {
                glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
                entity->setPosition(pos.x, pos.y, pos.z, oCanonical);
                LOG_DEBUG("Updated entity position: 0x", std::hex, block.guid, std::dec);

                updateNonPlayerTransportAttachment(block, entity, entity->getType());

                // 3b: Track player-on-transport state from MOVEMENT updates
                if (block.guid == owner_.getPlayerGuid()) {
                    owner_.movementInfoRef().orientation = oCanonical;
                    resyncPlayerIfFarFromServer(pos);
                    applyPlayerTransportState(block, entity, pos, oCanonical, true);
                }

                // Fire transport move callback if this is a known transport
                if (transportGuids_.count(block.guid) && owner_.transportMoveCallbackRef()) {
                    serverUpdatedTransportGuids_.insert(block.guid);
                    owner_.transportMoveCallbackRef()(block.guid, pos.x, pos.y, pos.z, oCanonical);
                }
                // Fire move callback for non-transport gameobjects.
                if (entity->getType() == ObjectType::GAMEOBJECT &&
                    transportGuids_.count(block.guid) == 0 &&
                    owner_.gameObjectMoveCallbackRef()) {
                    owner_.gameObjectMoveCallbackRef()(block.guid, entity->getX(), entity->getY(),
                                            entity->getZ(), entity->getOrientation());
                }
                // Fire move callback for non-player units (creatures).
                // SMSG_MONSTER_MOVE handles smooth interpolated movement, but many
                // servers (especially vanilla/Turtle WoW) communicate NPC positions
                // via MOVEMENT blocks instead. Use duration=0 for an instant snap.
                if (block.guid != owner_.getPlayerGuid() &&
                    entity->getType() == ObjectType::UNIT &&
                    transportGuids_.count(block.guid) == 0 &&
                    owner_.creatureMoveCallbackRef()) {
                    owner_.creatureMoveCallbackRef()(block.guid, pos.x, pos.y, pos.z, 0);
                }
            } else {
                LOG_WARNING("MOVEMENT update for unknown entity: 0x", std::hex, block.guid, std::dec);
            }
}

void EntityController::finalizeUpdateObjectBatch(bool newItemCreated) {
    owner_.tabCycleStaleRef() = true;
    // Entity count logging disabled

    // Deferred rebuild: if new item objects were created in this packet, rebuild
    // owner_.inventoryRef() so that slot GUIDs updated earlier in the same packet can resolve.
    if (newItemCreated) {
        owner_.rebuildOnlineInventory();
    }

    // Late owner_.inventoryRef() base detection once items are known
    if (owner_.getPlayerGuid() != 0 && owner_.invSlotBaseRef() < 0 && !owner_.lastPlayerFieldsRef().empty() && !owner_.onlineItemsRef().empty()) {
        owner_.detectInventorySlotBases(owner_.lastPlayerFieldsRef());
        if (owner_.invSlotBaseRef() >= 0) {
            if (owner_.applyInventoryFields(owner_.lastPlayerFieldsRef())) {
                owner_.rebuildOnlineInventory();
            }
        }
    }
}

void EntityController::handleCompressedUpdateObject(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_COMPRESSED_UPDATE_OBJECT, packet size: ", packet.getSize());

    // First 4 bytes = decompressed size
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_COMPRESSED_UPDATE_OBJECT too small");
        return;
    }

    uint32_t decompressedSize = packet.readUInt32();
    LOG_DEBUG("  Decompressed size: ", decompressedSize);

    // Capital cities and large raids can produce very large update packets.
    // The real WoW client handles up to ~10MB; 5MB covers all practical cases.
    if (decompressedSize == 0 || decompressedSize > 5 * 1024 * 1024) {
        LOG_WARNING("Invalid decompressed size: ", decompressedSize);
        return;
    }

    // Remaining data is zlib compressed
    size_t compressedSize = packet.getRemainingSize();
    const uint8_t* compressedData = packet.getData().data() + packet.getReadPos();

    // Decompress
    std::vector<uint8_t> decompressed(decompressedSize);
    uLongf destLen = decompressedSize;
    int ret = uncompress(decompressed.data(), &destLen, compressedData, compressedSize);

    if (ret != Z_OK) {
        LOG_WARNING("Failed to decompress UPDATE_OBJECT: zlib error ", ret);
        return;
    }

    // Create packet from decompressed data and parse it
    network::Packet decompressedPacket(wireOpcode(Opcode::SMSG_UPDATE_OBJECT), decompressed);
    handleUpdateObject(decompressedPacket);
}
void EntityController::handleDestroyObject(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_DESTROY_OBJECT");

    DestroyObjectData data;
    if (!DestroyObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_DESTROY_OBJECT");
        return;
    }

    // Remove entity
    if (entityManager.hasEntity(data.guid)) {
        if (transportGuids_.count(data.guid) > 0) {
            const bool playerAboardNow = (owner_.playerTransportGuidRef() == data.guid);
            const bool stickyAboard = (owner_.playerTransportStickyGuidRef() == data.guid && owner_.playerTransportStickyTimerRef() > 0.0f);
            const bool movementSaysAboard = (owner_.movementInfoRef().transportGuid == data.guid);
            // CMaNGOS judges visibility using the transport's static DB spawn
            // coordinate, which can be far from where a client-animated transport
            // currently is - keep all transports alive unconditionally on pre-WotLK.
            // AzerothCore (WotLK) sends legitimate destroy/recreate cycles, so only
            // preserve if the player is actually aboard.
            if (isPreWotlk() || playerAboardNow || stickyAboard || movementSaysAboard) {
                serverUpdatedTransportGuids_.erase(data.guid);
                LOG_INFO("Preserving transport on destroy: 0x", std::hex, data.guid, std::dec,
                         " now=", playerAboardNow,
                         " sticky=", stickyAboard,
                         " movement=", movementSaysAboard);
                return;
            }
        }
        // Mirror out-of-range handling: invoke render-layer despawn callbacks before entity removal.
        auto entity = entityManager.getEntity(data.guid);
        if (entity) {
            if (entity->getType() == ObjectType::UNIT && owner_.creatureDespawnCallbackRef()) {
                owner_.creatureDespawnCallbackRef()(data.guid);
            } else if (entity->getType() == ObjectType::PLAYER && owner_.playerDespawnCallbackRef()) {
                // Player entities also need renderer cleanup on DESTROY_OBJECT, not just out-of-range.
                owner_.playerDespawnCallbackRef()(data.guid);
                owner_.otherPlayerVisibleItemEntriesRef().erase(data.guid);
                owner_.otherPlayerVisibleDirtyRef().erase(data.guid);
                owner_.otherPlayerMoveTimeMsRef().erase(data.guid);
                owner_.inspectedPlayerItemEntriesRef().erase(data.guid);
                owner_.pendingAutoInspectRef().erase(data.guid);
                pendingNameQueries.erase(data.guid);
            } else if (entity->getType() == ObjectType::GAMEOBJECT && owner_.gameObjectDespawnCallbackRef()) {
                owner_.gameObjectDespawnCallbackRef()(data.guid);
            } else if (entity->getType() == ObjectType::CORPSE && owner_.playerDespawnCallbackRef()) {
                owner_.playerDespawnCallbackRef()(data.guid);
            }
        }
        if (transportGuids_.count(data.guid) > 0) {
            transportGuids_.erase(data.guid);
            serverUpdatedTransportGuids_.erase(data.guid);
            if (owner_.playerTransportGuidRef() == data.guid) {
                owner_.clearPlayerTransport();
            }
        }
        owner_.clearTransportAttachment(data.guid);
        entityManager.removeEntity(data.guid);
        LOG_INFO("Destroyed entity: 0x", std::hex, data.guid, std::dec,
                 " (", (data.isDeath ? "death" : "despawn"), ")");
    } else {
        LOG_DEBUG("Destroy object for unknown entity: 0x", std::hex, data.guid, std::dec);
    }

    // Clean up auto-attack and target if destroyed entity was our target
    if (owner_.getCombatHandler() && data.guid == owner_.getCombatHandler()->getAutoAttackTargetGuid()) {
        owner_.stopAutoAttack();
    }
    if (data.guid == owner_.getTargetGuid()) {
        owner_.setTargetGuidRaw(0);
    }
    if (owner_.getCombatHandler()) owner_.getCombatHandler()->removeHostileAttacker(data.guid);

    // Remove online item/container tracking
    owner_.containerContentsRef().erase(data.guid);
    if (owner_.onlineItemsRef().erase(data.guid)) {
        owner_.rebuildOnlineInventory();
    }

    // Clean up quest giver status
    // QuestHandler's map, which is the one every reader forwards to.
    // GameHandler's same-named member is dead, so erasing there left a
    // despawned quest giver's mark behind on whatever took its place.
    if (auto* qh = owner_.getQuestHandler()) qh->npcQuestStatusRef().erase(data.guid);

    // Remove combat text entries referencing the destroyed entity so floating
    // damage numbers don't linger after the source/target despawns.
    if (owner_.getCombatHandler()) owner_.getCombatHandler()->removeCombatTextForGuid(data.guid);

    // Clean up unit cast owner_.getState() (cast bar) for the destroyed unit
    if (owner_.getSpellHandler()) owner_.getSpellHandler()->removeUnitCastState(data.guid);
    // Clean up cached auras
    if (owner_.getSpellHandler()) owner_.getSpellHandler()->removeUnitAuraCache(data.guid);

    owner_.tabCycleStaleRef() = true;
}

// Name Queries
// ============================================================

void EntityController::queryPlayerName(uint64_t guid) {
    // If already cached, apply the name to the entity (handles entity recreation after
    // moving out/in range - the entity object is new but the cached name is valid).
    auto cacheIt = playerNameCache.find(guid);
    if (cacheIt != playerNameCache.end()) {
        auto entity = entityManager.getEntity(guid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<Player>(entity);
            if (player->getName().empty()) {
                player->setName(cacheIt->second);
            }
        }
        return;
    }
    if (pendingNameQueries.count(guid)) return;
    if (!owner_.isInWorld()) {
        LOG_INFO("queryPlayerName: skipped guid=0x", std::hex, guid, std::dec,
                 " owner_.getState()=", worldStateName(owner_.getState()), " owner_.getSocket()=", (owner_.getSocket() ? "yes" : "no"));
        return;
    }

    LOG_INFO("queryPlayerName: sending CMSG_NAME_QUERY for guid=0x", std::hex, guid, std::dec);
    pendingNameQueries.insert(guid);
    auto packet = NameQueryPacket::build(guid);
    owner_.getSocket()->send(packet);
}

void EntityController::queryCreatureInfo(uint32_t entry, uint64_t guid) {
    if (creatureInfoCache.count(entry) || pendingCreatureQueries.count(entry)) return;
    if (!owner_.isInWorld()) return;

    pendingCreatureQueries.insert(entry);
    auto packet = CreatureQueryPacket::build(entry, guid);
    owner_.getSocket()->send(packet);
}

void EntityController::queryGameObjectInfo(uint32_t entry, uint64_t guid) {
    if (gameObjectInfoCache_.count(entry) || pendingGameObjectQueries_.count(entry)) return;
    if (!owner_.isInWorld()) return;

    pendingGameObjectQueries_.insert(entry);
    auto packet = GameObjectQueryPacket::build(entry, guid);
    owner_.getSocket()->send(packet);
}

std::string EntityController::getCachedPlayerName(uint64_t guid) const {
    return std::string(lookupName(guid));
}

std::string EntityController::getCachedCreatureName(uint32_t entry) const {
    auto it = creatureInfoCache.find(entry);
    return (it != creatureInfoCache.end()) ? it->second.name : "";
}
void EntityController::handleNameQueryResponse(network::Packet& packet) {
    NameQueryResponseData data;
    if (!owner_.getPacketParsers() || !owner_.getPacketParsers()->parseNameQueryResponse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_NAME_QUERY_RESPONSE (size=", packet.getSize(), ")");
        return;
    }

    pendingNameQueries.erase(data.guid);

    LOG_INFO("Name query response: guid=0x", std::hex, data.guid, std::dec,
             " found=", static_cast<int>(data.found), " name='", data.name, "'",
             " race=", static_cast<int>(data.race), " class=", static_cast<int>(data.classId));

    if (data.isValid()) {
        playerNameCache[data.guid] = data.name;
        // Chat lines held back for this name can go out now, with it on them.
        if (owner_.getChatHandler()) owner_.getChatHandler()->flushChatAwaitingName(data.guid);
        // Cache class/race from name query for UnitClass/UnitRace fallback
        if (data.classId != 0 || data.race != 0) {
            playerClassRaceCache_[data.guid] = {.classId = data.classId, .raceId = data.race};
        }
        // Update entity name
        auto entity = entityManager.getEntity(data.guid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<Player>(entity);
            player->setName(data.name);
        }

        // Backfill chat history entries that arrived before we knew the name.
        if (owner_.getChatHandler()) {
            for (auto& msg : owner_.getChatHandler()->getChatHistory()) {
                if (msg.senderGuid == data.guid && msg.senderName.empty()) {
                    msg.senderName = data.name;
                }
            }
        }

        // Backfill whisper reply target if the name arrived after the whisper.
        if (owner_.lastWhisperSenderGuidRef() == data.guid && owner_.lastWhisperSenderRef().empty()) {
            owner_.lastWhisperSenderRef() = data.name;
        }

        // Backfill mail inbox sender names
        for (auto& mail : owner_.mailInboxRef()) {
            if (mail.messageType == 0 && mail.senderGuid == data.guid) {
                mail.senderName = data.name;
            }
        }

        // Backfill friend list: if this GUID came from a friend list packet,
        // register the name in owner_.friendsCacheRef() now that we know it.
        if (owner_.friendGuidsRef().count(data.guid)) {
            owner_.friendsCacheRef()[data.name] = data.guid;
        }

        // Backfill ignore list: SMSG_IGNORE_LIST only contains GUIDs, so
        // ignoreCache (name→guid for UI) is populated here once names resolve.
        if (owner_.ignoreListGuidsRef().count(data.guid)) {
            owner_.ignoreCacheRef()[data.name] = data.guid;
        }

        // Fire UNIT_NAME_UPDATE so nameplate/unit frame addons know the name is available
        if (owner_.addonEventCallbackRef()) {
            std::string unitId;
            if (data.guid == owner_.getTargetGuid()) unitId = "target";
            else if (data.guid == owner_.focusGuidRef()) unitId = "focus";
            else if (data.guid == owner_.getPlayerGuid()) unitId = "player";
            if (!unitId.empty())
                owner_.fireAddonEvent("UNIT_NAME_UPDATE", {unitId});
        }
    }
}

void EntityController::handleCreatureQueryResponse(network::Packet& packet) {
    CreatureQueryResponseData data;
    if (!owner_.getPacketParsers()->parseCreatureQueryResponse(packet, data)) return;

    pendingCreatureQueries.erase(data.entry);

    if (data.isValid()) {
        creatureInfoCache[data.entry] = data;
        // Update all unit entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(entity);
                if (unit->getEntry() == data.entry) {
                    unit->setName(data.name);
                    // The rank came with this response, and UnitClassification
                    // reads it from the cache just filled. A unit targeted
                    // before its query came back answered "normal" until now -
                    // so an elite or a rare drew a plain border and kept it,
                    // because the frame only rechecks when told to.
                    //
                    // Only for a unit the interface has a token for: the frame
                    // compares the argument against its own unit and ignores
                    // anything else, and most of a zone is neither targeted nor
                    // focused.
                    const std::string unitId = owner_.guidToUnitId(guid);
                    if (!unitId.empty() && owner_.addonEventCallbackRef()) {
                        owner_.addonEventCallbackRef()("UNIT_CLASSIFICATION_CHANGED", {unitId});
                    }
                }
            }
        }
        // A companion may have been waiting on exactly this. A companion's
        // creature id is a template entry and the model frame needs a display
        // id, so GetCompanionInfo answers zero until the query comes back -
        // and the tab only re-reads when told. Without this the model appeared
        // on the next login and not before.
        //
        // With the kind, because the event carries it and the tab re-reads the
        // list it names: a mount's answer announced as a critter's left the
        // mount tab holding the zero it started with.
        const std::string kind = owner_.companionKindForCreature(data.entry);
        if (!kind.empty() && owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("COMPANION_UPDATE", {kind});
        }
    }
}

// ============================================================
// GameObject Query
// ============================================================

void EntityController::handleGameObjectQueryResponse(network::Packet& packet) {
    GameObjectQueryResponseData data;
    bool ok = owner_.getPacketParsers() ? owner_.getPacketParsers()->parseGameObjectQueryResponse(packet, data)
                             : GameObjectQueryResponseParser::parse(packet, data);
    if (!ok) return;

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

        // The model was spawned from the display id before this response arrived,
        // so anything keyed off the game object's type has to be revisited now.
        if (owner_.gameObjectInfoCallbackRef()) owner_.gameObjectInfoCallbackRef()(data.entry);

        // MO_TRANSPORT (type 15): assign TaxiPathNode path if available.
        const uint32_t mapId = owner_.getCurrentMapId();
        if (data.type == 15 && data.hasData && data.data[0] != 0 && owner_.getTransportManager()) {
            uint32_t taxiPathId = data.data[0];
            if (owner_.getTransportManager()->hasTaxiPathForMap(taxiPathId, mapId)) {
                if (owner_.getTransportManager()->assignTaxiPathToTransport(data.entry, taxiPathId, mapId)) {
                    LOG_INFO("MO_TRANSPORT entry=", data.entry, " assigned TaxiPathNode path ", taxiPathId,
                             " on map ", mapId);
                }
            } else {
                LOG_WARNING("MO_TRANSPORT entry=", data.entry, " taxiPathId=", taxiPathId,
                         " has no TaxiPathNode segment on map ", mapId);
            }
        }
    }
}

void EntityController::handleGameObjectPageText(network::Packet& packet) {
    if (!packet.hasRemaining(8)) return;
    uint64_t guid = packet.readUInt64();
    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return;

    auto go = std::static_pointer_cast<GameObject>(entity);
    uint32_t entry = go->getEntry();
    if (entry == 0) return;

    auto cacheIt = gameObjectInfoCache_.find(entry);
    if (cacheIt == gameObjectInfoCache_.end()) {
        queryGameObjectInfo(entry, guid);
        return;
    }

    const GameObjectQueryResponseData& info = cacheIt->second;
    uint32_t pageId = 0;
    // AzerothCore layout:
    // type 9 (TEXT): data[0]=pageID
    // type 10 (GOOBER): data[7]=pageId
    // The backing sits two words behind the page in both layouts, which is the
    // whole of what tells a stone tablet from a parchment letter on screen.
    uint32_t pageMaterial = 0;
    if (info.type == 9)       { pageId = info.data[0]; pageMaterial = info.data[2]; }
    else if (info.type == 10) { pageId = info.data[7]; pageMaterial = info.data[9]; }

    if (pageId != 0 && owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
        owner_.bookPagesRef().clear();  // start a fresh book for this interaction
        owner_.setBookTitle(info.name);
        owner_.setBookMaterial(pageMaterial);
        auto req = PageTextQueryPacket::build(pageId, guid);
        owner_.getSocket()->send(req);
        return;
    }

    if (!info.name.empty()) {
        owner_.addSystemChatMessage(info.name);
    }
}

void EntityController::handlePageTextQueryResponse(network::Packet& packet) {
    PageTextQueryResponseData data;
    if (!PageTextQueryResponseParser::parse(packet, data)) return;

    if (!data.isValid()) return;

    std::string playerName;
    if (auto entity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid())) {
        if (auto unit = std::dynamic_pointer_cast<Unit>(entity))
            playerName = unit->getName();
    }
    data.text = normalizeWowTextTokens(data.text, playerName);

    // Append page if not already collected
    bool alreadyHave = false;
    for (const auto& bp : owner_.bookPagesRef()) {
        if (bp.pageId == data.pageId) { alreadyHave = true; break; }
    }
    if (!alreadyHave) {
        owner_.bookPagesRef().push_back({.pageId = data.pageId, .text = data.text});
    }

    // Follow the chain: if there's a next page we haven't fetched yet, request it
    if (data.nextPageId != 0) {
        bool nextHave = false;
        for (const auto& bp : owner_.bookPagesRef()) {
            if (bp.pageId == data.nextPageId) { nextHave = true; break; }
        }
        if (!nextHave && owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
            auto req = PageTextQueryPacket::build(data.nextPageId, owner_.getPlayerGuid());
            owner_.getSocket()->send(req);
        }
    }
    // Tell the interface, which is otherwise never told about a book at all.
    // The mail path fires these two from InventoryHandler and FrameXML's
    // ItemTextFrame is wired to them; a book arrives on a different opcode and
    // fired nothing, so with the Book element handed over there was nothing to
    // read. BEGIN on the first page - the frame clears itself and picks its
    // material on that - and READY on every page, since each one that lands is
    // another the reader can turn to.
    if (owner_.addonEventCallbackRef()) {
        if (owner_.bookPagesRef().size() == 1) {
            owner_.addonEventCallbackRef()("ITEM_TEXT_BEGIN", {});
        }
        owner_.addonEventCallbackRef()("ITEM_TEXT_READY", {});
    }
    LOG_DEBUG("handlePageTextQueryResponse: pageId=", data.pageId,
              " nextPage=", data.nextPageId,
              " totalPages=", owner_.bookPagesRef().size());
}


} // namespace game
} // namespace wowee
