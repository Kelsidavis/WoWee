#include "game/movement_handler.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/packet_parsers.hpp"
#include "game/spline_packet.hpp"
#include "game/transport_manager.hpp"
#include "game/entity.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <zlib.h>
#include <array>
#include <unordered_set>

namespace wowee {
namespace game {

namespace {

struct KnownAreaTriggerDestination {
    uint32_t triggerId;
    uint32_t mapId;
    float serverX;
    float serverY;
    float serverZ;
    float serverO;
};

constexpr KnownAreaTriggerDestination kKnownAreaTriggerDestinations[] = {
    {2166, 0,   -4838.95f, -1318.46f, 501.868f, 1.42372f}, // Deeprun Tram -> Ironforge
    {2171, 0,   -8364.57f,   535.981f, 91.7969f, 2.24619f}, // Deeprun Tram -> Stormwind
    {2173, 369,    68.3006f, 2490.91f, -4.29647f, 3.12192f}, // Stormwind -> Deeprun Tram
    {2175, 369,    69.2542f,   10.257f, -4.29664f, 3.09832f}, // Ironforge -> Deeprun Tram
};

const KnownAreaTriggerDestination* findKnownAreaTriggerDestination(uint32_t triggerId, uint32_t mapId) {
    for (const auto& dest : kKnownAreaTriggerDestinations) {
        if (dest.triggerId == triggerId && dest.mapId == mapId) return &dest;
    }
    return nullptr;
}

const KnownAreaTriggerDestination* findKnownAreaTriggerDestinationByTrigger(uint32_t triggerId) {
    for (const auto& dest : kKnownAreaTriggerDestinations) {
        if (dest.triggerId == triggerId) return &dest;
    }
    return nullptr;
}

uint32_t findKnownReturnAreaTrigger(uint32_t triggerId) {
    switch (triggerId) {
        case 2166: return 2175; // Deeprun Tram -> Ironforge, return via Ironforge portal
        case 2171: return 2173; // Deeprun Tram -> Stormwind, return via Stormwind portal
        case 2173: return 2171; // Stormwind -> Deeprun Tram, exit on Stormwind side
        case 2175: return 2166; // Ironforge -> Deeprun Tram, exit on Ironforge side
        default: return 0;
    }
}

} // namespace

MovementHandler::MovementHandler(GameHandler& owner)
    : owner_(owner), movementInfo(owner.movementInfoRef()) {}

void MovementHandler::applyServerMovementSpeeds(float walk, float run, float runBack,
                                                 float swim, float swimBack, float flight,
                                                 float flightBack, float turn, float pitch) {
    auto apply = [](float value, float& destination) {
        if (std::isfinite(value) && value > 0.01f && value < 200.0f) {
            destination = value;
        }
    };
    apply(walk, serverWalkSpeed_);
    apply(run, serverRunSpeed_);
    apply(runBack, serverRunBackSpeed_);
    apply(swim, serverSwimSpeed_);
    apply(swimBack, serverSwimBackSpeed_);
    apply(flight, serverFlightSpeed_);
    apply(flightBack, serverFlightBackSpeed_);
    apply(turn, serverTurnRate_);
    apply(pitch, serverPitchRate_);
    LOG_INFO("Movement speeds restored from object update: walk=", serverWalkSpeed_,
             " run=", serverRunSpeed_, " runBack=", serverRunBackSpeed_,
             " swim=", serverSwimSpeed_, " flight=", serverFlightSpeed_);
}

void MovementHandler::registerOpcodes(DispatchTable& table) {
    // Creature movement
    table[Opcode::SMSG_MONSTER_MOVE] = [this](network::Packet& packet) { handleMonsterMove(packet); };
    table[Opcode::SMSG_COMPRESSED_MOVES] = [this](network::Packet& packet) { handleCompressedMoves(packet); };
    table[Opcode::SMSG_MONSTER_MOVE_TRANSPORT] = [this](network::Packet& packet) { handleMonsterMoveTransport(packet); };

    // Spline move: consume-only (no state change)
    for (auto op : { Opcode::SMSG_SPLINE_MOVE_FEATHER_FALL,
                     Opcode::SMSG_SPLINE_MOVE_GRAVITY_DISABLE,
                     Opcode::SMSG_SPLINE_MOVE_GRAVITY_ENABLE,
                     Opcode::SMSG_SPLINE_MOVE_LAND_WALK,
                     Opcode::SMSG_SPLINE_MOVE_NORMAL_FALL,
                     Opcode::SMSG_SPLINE_MOVE_ROOT,
                     Opcode::SMSG_SPLINE_MOVE_SET_HOVER }) {
        table[op] = [](network::Packet& packet) {
            if (packet.hasRemaining(1))
                (void)packet.readPackedGuid();
        };
    }

    // Spline move: synth flags (each opcode produces different flags)
    {
        auto makeSynthHandler = [this](uint32_t synthFlags) {
            return [this, synthFlags](network::Packet& packet) {
                if (!packet.hasRemaining(1)) return;
                uint64_t guid = packet.readPackedGuid();
                if (guid == 0 || guid == owner_.getPlayerGuid() || !owner_.unitMoveFlagsCallbackRef()) return;
                owner_.unitMoveFlagsCallbackRef()(guid, synthFlags);
            };
        };
        table[Opcode::SMSG_SPLINE_MOVE_SET_WALK_MODE] = makeSynthHandler(0x00000100u);
        table[Opcode::SMSG_SPLINE_MOVE_SET_RUN_MODE]  = makeSynthHandler(0u);
        table[Opcode::SMSG_SPLINE_MOVE_SET_FLYING]    = makeSynthHandler(0x01000000u | 0x00800000u);
        table[Opcode::SMSG_SPLINE_MOVE_START_SWIM]    = makeSynthHandler(0x00200000u);
        table[Opcode::SMSG_SPLINE_MOVE_STOP_SWIM]     = makeSynthHandler(0u);
    }

    // Spline speed: all opcodes share the same PackedGuid+float format, differing
    // only in which member receives the value. Factory avoids 8 copy-pasted lambdas.
    auto makeSplineSpeedHandler = [this](float MovementHandler::* member) {
        return [this, member](network::Packet& packet) {
            if (!packet.hasRemaining(5)) return;
            uint64_t guid = packet.readPackedGuid();
            if (!packet.hasRemaining(4)) return;
            float speed = packet.readFloat();
            if (guid == owner_.getPlayerGuid() && std::isfinite(speed) && speed > 0.01f && speed < 200.0f)
                this->*member = speed;
        };
    };
    table[Opcode::SMSG_SPLINE_SET_RUN_SPEED]      = makeSplineSpeedHandler(&MovementHandler::serverRunSpeed_);
    table[Opcode::SMSG_SPLINE_SET_RUN_BACK_SPEED]  = makeSplineSpeedHandler(&MovementHandler::serverRunBackSpeed_);
    table[Opcode::SMSG_SPLINE_SET_SWIM_SPEED]      = makeSplineSpeedHandler(&MovementHandler::serverSwimSpeed_);

    // Force speed changes
    table[Opcode::SMSG_FORCE_RUN_SPEED_CHANGE] = [this](network::Packet& packet) { handleForceRunSpeedChange(packet); };
    table[Opcode::SMSG_FORCE_MOVE_ROOT] = [this](network::Packet& packet) { handleForceMoveRootState(packet, true); };
    table[Opcode::SMSG_FORCE_MOVE_UNROOT] = [this](network::Packet& packet) { handleForceMoveRootState(packet, false); };
    table[Opcode::SMSG_FORCE_WALK_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "WALK_SPEED", Opcode::CMSG_FORCE_WALK_SPEED_CHANGE_ACK, &serverWalkSpeed_);
    };
    table[Opcode::SMSG_FORCE_RUN_BACK_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "RUN_BACK_SPEED", Opcode::CMSG_FORCE_RUN_BACK_SPEED_CHANGE_ACK, &serverRunBackSpeed_);
    };
    table[Opcode::SMSG_FORCE_SWIM_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "SWIM_SPEED", Opcode::CMSG_FORCE_SWIM_SPEED_CHANGE_ACK, &serverSwimSpeed_);
    };
    table[Opcode::SMSG_FORCE_SWIM_BACK_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "SWIM_BACK_SPEED", Opcode::CMSG_FORCE_SWIM_BACK_SPEED_CHANGE_ACK, &serverSwimBackSpeed_);
    };
    table[Opcode::SMSG_FORCE_FLIGHT_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "FLIGHT_SPEED", Opcode::CMSG_FORCE_FLIGHT_SPEED_CHANGE_ACK, &serverFlightSpeed_);
    };
    table[Opcode::SMSG_FORCE_FLIGHT_BACK_SPEED_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "FLIGHT_BACK_SPEED", Opcode::CMSG_FORCE_FLIGHT_BACK_SPEED_CHANGE_ACK, &serverFlightBackSpeed_);
    };
    table[Opcode::SMSG_FORCE_TURN_RATE_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "TURN_RATE", Opcode::CMSG_FORCE_TURN_RATE_CHANGE_ACK, &serverTurnRate_);
    };
    table[Opcode::SMSG_FORCE_PITCH_RATE_CHANGE] = [this](network::Packet& packet) {
        handleForceSpeedChange(packet, "PITCH_RATE", Opcode::CMSG_FORCE_PITCH_RATE_CHANGE_ACK, &serverPitchRate_);
    };

    // Movement flag toggles
    table[Opcode::SMSG_MOVE_SET_CAN_FLY] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "SET_CAN_FLY", Opcode::CMSG_MOVE_SET_CAN_FLY_ACK,
            static_cast<uint32_t>(MovementFlags::CAN_FLY), true);
    };
    table[Opcode::SMSG_MOVE_UNSET_CAN_FLY] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "UNSET_CAN_FLY", Opcode::CMSG_MOVE_SET_CAN_FLY_ACK,
            static_cast<uint32_t>(MovementFlags::CAN_FLY), false);
    };
    table[Opcode::SMSG_MOVE_FEATHER_FALL] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "FEATHER_FALL", Opcode::CMSG_MOVE_FEATHER_FALL_ACK,
            static_cast<uint32_t>(MovementFlags::FEATHER_FALL), true);
    };
    table[Opcode::SMSG_MOVE_WATER_WALK] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "WATER_WALK", Opcode::CMSG_MOVE_WATER_WALK_ACK,
            static_cast<uint32_t>(MovementFlags::WATER_WALK), true);
    };
    table[Opcode::SMSG_MOVE_SET_HOVER] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "SET_HOVER", Opcode::CMSG_MOVE_HOVER_ACK,
            static_cast<uint32_t>(MovementFlags::HOVER), true);
    };
    table[Opcode::SMSG_MOVE_UNSET_HOVER] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "UNSET_HOVER", Opcode::CMSG_MOVE_HOVER_ACK,
            static_cast<uint32_t>(MovementFlags::HOVER), false);
    };
    table[Opcode::SMSG_MOVE_KNOCK_BACK] = [this](network::Packet& packet) { handleMoveKnockBack(packet); };

    // Teleport
    for (auto op : { Opcode::MSG_MOVE_TELEPORT, Opcode::MSG_MOVE_TELEPORT_ACK }) {
        table[op] = [this](network::Packet& packet) { handleTeleportAck(packet); };
    }
    table[Opcode::SMSG_NEW_WORLD] = [this](network::Packet& packet) { handleNewWorld(packet); };

    // Taxi
    table[Opcode::SMSG_SHOWTAXINODES] = [this](network::Packet& packet) { handleShowTaxiNodes(packet); };
    table[Opcode::SMSG_ACTIVATETAXIREPLY] = [this](network::Packet& packet) { handleActivateTaxiReply(packet); };

    // MSG_MOVE_* relay (other player movement)
    for (auto op : { Opcode::MSG_MOVE_START_FORWARD, Opcode::MSG_MOVE_START_BACKWARD,
                     Opcode::MSG_MOVE_STOP, Opcode::MSG_MOVE_START_STRAFE_LEFT,
                     Opcode::MSG_MOVE_START_STRAFE_RIGHT, Opcode::MSG_MOVE_STOP_STRAFE,
                     Opcode::MSG_MOVE_JUMP, Opcode::MSG_MOVE_START_TURN_LEFT,
                     Opcode::MSG_MOVE_START_TURN_RIGHT, Opcode::MSG_MOVE_STOP_TURN,
                     Opcode::MSG_MOVE_SET_FACING, Opcode::MSG_MOVE_FALL_LAND,
                     Opcode::MSG_MOVE_HEARTBEAT, Opcode::MSG_MOVE_START_SWIM,
                     Opcode::MSG_MOVE_STOP_SWIM, Opcode::MSG_MOVE_SET_WALK_MODE,
                     Opcode::MSG_MOVE_SET_RUN_MODE, Opcode::MSG_MOVE_START_PITCH_UP,
                     Opcode::MSG_MOVE_START_PITCH_DOWN, Opcode::MSG_MOVE_STOP_PITCH,
                     Opcode::MSG_MOVE_START_ASCEND, Opcode::MSG_MOVE_STOP_ASCEND,
                     Opcode::MSG_MOVE_START_DESCEND, Opcode::MSG_MOVE_SET_PITCH,
                     Opcode::MSG_MOVE_GRAVITY_CHNG, Opcode::MSG_MOVE_UPDATE_CAN_FLY,
                     Opcode::MSG_MOVE_UPDATE_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY,
                     Opcode::MSG_MOVE_ROOT, Opcode::MSG_MOVE_UNROOT }) {
        table[op] = [this](network::Packet& packet) {
            if (owner_.getState() == WorldState::IN_WORLD) handleOtherPlayerMovement(packet);
        };
    }

    // MSG_MOVE_SET_*_SPEED relay
    for (auto op : { Opcode::MSG_MOVE_SET_RUN_SPEED, Opcode::MSG_MOVE_SET_RUN_BACK_SPEED,
                     Opcode::MSG_MOVE_SET_WALK_SPEED, Opcode::MSG_MOVE_SET_SWIM_SPEED,
                     Opcode::MSG_MOVE_SET_SWIM_BACK_SPEED, Opcode::MSG_MOVE_SET_FLIGHT_SPEED,
                     Opcode::MSG_MOVE_SET_FLIGHT_BACK_SPEED }) {
        table[op] = [this](network::Packet& packet) {
            if (owner_.getState() == WorldState::IN_WORLD) handleMoveSetSpeed(packet);
        };
    }

    // ---- Client control & spline speed/flag changes ----

    // Client control update
    table[Opcode::SMSG_CLIENT_CONTROL_UPDATE] = [this](network::Packet& packet) {
        handleClientControlUpdate(packet);
    };

    // Spline move flag changes for other units (unroot/unset_hover/water_walk)
    for (auto op : {Opcode::SMSG_SPLINE_MOVE_UNROOT,
                    Opcode::SMSG_SPLINE_MOVE_UNSET_HOVER,
                    Opcode::SMSG_SPLINE_MOVE_WATER_WALK}) {
        table[op] = [](network::Packet& packet) {
            // Minimal parse: PackedGuid only — no animation-relevant state change.
            if (packet.hasRemaining(1)) {
                (void)packet.readPackedGuid();
            }
        };
    }

    table[Opcode::SMSG_SPLINE_MOVE_UNSET_FLYING] = [this](network::Packet& packet) {
        // PackedGuid + synthesised move-flags=0 → clears flying animation.
        if (!packet.hasRemaining(1)) return;
        uint64_t guid = packet.readPackedGuid();
        if (guid == 0 || guid == owner_.getPlayerGuid() || !owner_.unitMoveFlagsCallbackRef()) return;
        owner_.unitMoveFlagsCallbackRef()(guid, 0u); // clear flying/CAN_FLY
    };

    // Remaining spline speed opcodes — same factory as above.
    table[Opcode::SMSG_SPLINE_SET_FLIGHT_SPEED]      = makeSplineSpeedHandler(&MovementHandler::serverFlightSpeed_);
    table[Opcode::SMSG_SPLINE_SET_FLIGHT_BACK_SPEED]  = makeSplineSpeedHandler(&MovementHandler::serverFlightBackSpeed_);
    table[Opcode::SMSG_SPLINE_SET_SWIM_BACK_SPEED]    = makeSplineSpeedHandler(&MovementHandler::serverSwimBackSpeed_);
    table[Opcode::SMSG_SPLINE_SET_WALK_SPEED]          = makeSplineSpeedHandler(&MovementHandler::serverWalkSpeed_);
    table[Opcode::SMSG_SPLINE_SET_TURN_RATE]           = makeSplineSpeedHandler(&MovementHandler::serverTurnRate_);
    // Pitch rate not stored locally — consume packet to keep stream aligned.
    table[Opcode::SMSG_SPLINE_SET_PITCH_RATE] = [](network::Packet& packet) { packet.skipAll(); };

    // ---- Player movement flag changes (server-pushed) ----
    table[Opcode::SMSG_MOVE_GRAVITY_DISABLE] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "GRAVITY_DISABLE", Opcode::CMSG_MOVE_GRAVITY_DISABLE_ACK,
            static_cast<uint32_t>(MovementFlags::LEVITATING), true);
    };
    table[Opcode::SMSG_MOVE_GRAVITY_ENABLE] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "GRAVITY_ENABLE", Opcode::CMSG_MOVE_GRAVITY_ENABLE_ACK,
            static_cast<uint32_t>(MovementFlags::LEVITATING), false);
    };
    table[Opcode::SMSG_MOVE_LAND_WALK] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "LAND_WALK", Opcode::CMSG_MOVE_WATER_WALK_ACK,
            static_cast<uint32_t>(MovementFlags::WATER_WALK), false);
    };
    table[Opcode::SMSG_MOVE_NORMAL_FALL] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "NORMAL_FALL", Opcode::CMSG_MOVE_FEATHER_FALL_ACK,
            static_cast<uint32_t>(MovementFlags::FEATHER_FALL), false);
    };
    table[Opcode::SMSG_MOVE_SET_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "SET_CAN_TRANSITION_SWIM_FLY",
            Opcode::CMSG_MOVE_SET_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY_ACK, 0, true);
    };
    table[Opcode::SMSG_MOVE_UNSET_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "UNSET_CAN_TRANSITION_SWIM_FLY",
            Opcode::CMSG_MOVE_SET_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY_ACK, 0, false);
    };
    table[Opcode::SMSG_MOVE_SET_COLLISION_HGT] = [this](network::Packet& packet) {
        handleMoveSetCollisionHeight(packet);
    };
    table[Opcode::SMSG_MOVE_SET_FLIGHT] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "SET_FLIGHT", Opcode::CMSG_MOVE_FLIGHT_ACK,
            static_cast<uint32_t>(MovementFlags::FLYING), true);
    };
    table[Opcode::SMSG_MOVE_UNSET_FLIGHT] = [this](network::Packet& packet) {
        handleForceMoveFlagChange(packet, "UNSET_FLIGHT", Opcode::CMSG_MOVE_FLIGHT_ACK,
            static_cast<uint32_t>(MovementFlags::FLYING), false);
    };
}

// ============================================================
// handleClientControlUpdate
// ============================================================

void MovementHandler::handleClientControlUpdate(network::Packet& packet) {
    // Minimal parse: PackedGuid + uint8 allowMovement.
    if (!packet.hasRemaining(2)) {
        LOG_WARNING("SMSG_CLIENT_CONTROL_UPDATE too short: ", packet.getSize(), " bytes");
        return;
    }
    uint8_t guidMask = packet.readUInt8();
    size_t guidBytes = 0;
    uint64_t controlGuid = 0;
    for (int i = 0; i < 8; ++i) {
        if (guidMask & (1u << i)) ++guidBytes;
    }
    if (!packet.hasRemaining(guidBytes + 1)) {
        LOG_WARNING("SMSG_CLIENT_CONTROL_UPDATE malformed (truncated packed guid)");
        packet.skipAll();
        return;
    }
    for (int i = 0; i < 8; ++i) {
        if (guidMask & (1u << i)) {
            uint8_t b = packet.readUInt8();
            controlGuid |= (static_cast<uint64_t>(b) << (i * 8));
        }
    }
    bool allowMovement = (packet.readUInt8() != 0);
    if (controlGuid == 0 || controlGuid == owner_.getPlayerGuid()) {
        bool changed = (serverMovementAllowed_ != allowMovement);
        serverMovementAllowed_ = allowMovement;
        if (changed && !allowMovement) {
            // Force-stop local movement immediately when server revokes control.
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                    static_cast<uint32_t>(MovementFlags::BACKWARD) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
            owner_.sendMovement(Opcode::MSG_MOVE_STOP);
            owner_.sendMovement(Opcode::MSG_MOVE_STOP_STRAFE);
            owner_.sendMovement(Opcode::MSG_MOVE_STOP_TURN);
            owner_.sendMovement(Opcode::MSG_MOVE_STOP_SWIM);
            owner_.addSystemChatMessage("Movement disabled by server.");
            owner_.fireAddonEvent("PLAYER_CONTROL_LOST", {});
        } else if (changed && allowMovement) {
            owner_.addSystemChatMessage("Movement re-enabled.");
            owner_.fireAddonEvent("PLAYER_CONTROL_GAINED", {});
        }
    }
}

// ============================================================
// Movement Timestamp
// ============================================================

uint32_t MovementHandler::nextMovementTimestampMs() {
    auto now = std::chrono::steady_clock::now();
    uint64_t elapsed = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - movementClockStart_).count()) + 1ULL;
    if (elapsed > std::numeric_limits<uint32_t>::max()) {
        movementClockStart_ = now;
        elapsed = 1ULL;
    }

    uint32_t candidate = static_cast<uint32_t>(elapsed);
    if (candidate <= lastMovementTimestampMs_) {
        candidate = lastMovementTimestampMs_ + 1U;
        if (candidate == 0) {
            movementClockStart_ = now;
            candidate = 1U;
        }
    }

    lastMovementTimestampMs_ = candidate;
    return candidate;
}

bool MovementHandler::restoreWorldTransferFallbackIfNearOrigin(const char* context) {
    if (!worldTransferFallbackValid_ || owner_.getCurrentMapId() != worldTransferFallbackMapId_) {
        return false;
    }
    if (owner_.getCurrentMapId() != 0 ||
        std::abs(movementInfo.x) >= 1000.0f || std::abs(movementInfo.y) >= 1000.0f) {
        return false;
    }

    LOG_WARNING(context,
                ": correcting near-origin map 0 position using area trigger ",
                worldTransferFallbackTriggerId_,
                " fallback canonical=(",
                worldTransferFallbackCanonicalPos_.x, ", ",
                worldTransferFallbackCanonicalPos_.y, ", ",
                worldTransferFallbackCanonicalPos_.z, ")");

    movementInfo.x = worldTransferFallbackCanonicalPos_.x;
    movementInfo.y = worldTransferFallbackCanonicalPos_.y;
    movementInfo.z = worldTransferFallbackCanonicalPos_.z;
    movementInfo.orientation = worldTransferFallbackCanonicalO_;
    movementInfo.transportGuid = 0;
    movementInfo.transportSeat = -1;
    movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
    owner_.clearPlayerTransport();

    if (auto player = owner_.getEntityManager().getEntity(owner_.getPlayerGuid())) {
        player->setPosition(movementInfo.x, movementInfo.y, movementInfo.z, movementInfo.orientation);
    }

    return true;
}

// ============================================================
// sendMovement
// ============================================================

void MovementHandler::sendMovement(Opcode opcode) {
    if (owner_.getState() != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send movement in state: ", (int)owner_.getState());
        return;
    }

    // Block manual movement while taxi is active/mounted, but always allow
    // stop/heartbeat opcodes so stuck states can be recovered.
    bool taxiAllowed =
        (opcode == Opcode::MSG_MOVE_HEARTBEAT) ||
        (opcode == Opcode::MSG_MOVE_STOP) ||
        (opcode == Opcode::MSG_MOVE_STOP_STRAFE) ||
        (opcode == Opcode::MSG_MOVE_STOP_TURN) ||
        (opcode == Opcode::MSG_MOVE_STOP_SWIM);
    if (!serverMovementAllowed_ && !taxiAllowed) return;
    if ((onTaxiFlight_ || taxiMountActive_) && !taxiAllowed) return;
    if (owner_.resurrectPendingRef() && !taxiAllowed) return;

    // Always send a strictly increasing non-zero client movement clock value.
    const uint32_t movementTime = nextMovementTimestampMs();
    movementInfo.time = movementTime;

    if (opcode == Opcode::MSG_MOVE_SET_FACING &&
        isPreWotlk()) {
        const float facingDelta = core::coords::normalizeAngleRad(
            movementInfo.orientation - lastFacingSentOrientation_);
        const uint32_t sinceLastFacingMs =
            lastFacingSendTimeMs_ != 0 && movementTime >= lastFacingSendTimeMs_
                ? (movementTime - lastFacingSendTimeMs_)
                : std::numeric_limits<uint32_t>::max();
        if (std::abs(facingDelta) < 0.02f && sinceLastFacingMs < 200U) {
            return;
        }
    }

    // Track movement state transition for PLAYER_STARTED/STOPPED_MOVING events
    const uint32_t kMoveMask = static_cast<uint32_t>(MovementFlags::FORWARD) |
                               static_cast<uint32_t>(MovementFlags::BACKWARD) |
                               static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                               static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
    const bool wasMoving = (movementInfo.flags & kMoveMask) != 0;

    // Cancel any timed non-channel cast, plus food/water restoration, the moment
    // the player starts moving. Other channels retain their normal behavior.
    if (owner_.isCasting() && (!owner_.isChanneling() || owner_.isRestoring())) {
        const bool isPositionalMove =
            opcode == Opcode::MSG_MOVE_START_FORWARD  ||
            opcode == Opcode::MSG_MOVE_START_BACKWARD ||
            opcode == Opcode::MSG_MOVE_START_STRAFE_LEFT  ||
            opcode == Opcode::MSG_MOVE_START_STRAFE_RIGHT ||
            opcode == Opcode::MSG_MOVE_JUMP;
        if (isPositionalMove) {
            owner_.cancelCast();
        }
    }

    // Update movement flags based on opcode
    switch (opcode) {
        case Opcode::MSG_MOVE_START_FORWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FORWARD);
            break;
        case Opcode::MSG_MOVE_START_BACKWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::BACKWARD);
            break;
        case Opcode::MSG_MOVE_STOP:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                    static_cast<uint32_t>(MovementFlags::BACKWARD));
            break;
        case Opcode::MSG_MOVE_START_STRAFE_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_LEFT);
            break;
        case Opcode::MSG_MOVE_START_STRAFE_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
            break;
        case Opcode::MSG_MOVE_STOP_STRAFE:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT));
            break;
        case Opcode::MSG_MOVE_JUMP:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FALLING);
            isFalling_ = true;
            fallStartMs_ = movementInfo.time;
            movementInfo.fallTime = 0;
            movementInfo.jumpVelocity = 7.96f;
            {
                const float facingRad = movementInfo.orientation;
                movementInfo.jumpCosAngle = std::cos(facingRad);
                movementInfo.jumpSinAngle = std::sin(facingRad);
                const uint32_t horizFlags =
                    static_cast<uint32_t>(MovementFlags::FORWARD) |
                    static_cast<uint32_t>(MovementFlags::BACKWARD) |
                    static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
                const bool movingHoriz = (movementInfo.flags & horizFlags) != 0;
                if (movingHoriz) {
                    const bool isWalking = (movementInfo.flags & static_cast<uint32_t>(MovementFlags::WALKING)) != 0;
                    movementInfo.jumpXYSpeed = isWalking ? 2.5f : (serverRunSpeed_ > 0.0f ? serverRunSpeed_ : 7.0f);
                } else {
                    movementInfo.jumpXYSpeed = 0.0f;
                }
            }
            break;
        case Opcode::MSG_MOVE_START_TURN_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_LEFT);
            break;
        case Opcode::MSG_MOVE_START_TURN_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_RIGHT);
            break;
        case Opcode::MSG_MOVE_STOP_TURN:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
            break;
        case Opcode::MSG_MOVE_FALL_LAND:
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::FALLING);
            isFalling_ = false;
            fallStartMs_ = 0;
            movementInfo.fallTime = 0;
            movementInfo.jumpVelocity = 0.0f;
            movementInfo.jumpSinAngle = 0.0f;
            movementInfo.jumpCosAngle = 0.0f;
            movementInfo.jumpXYSpeed = 0.0f;
            break;
        case Opcode::MSG_MOVE_HEARTBEAT:
            timeSinceLastMoveHeartbeat_ = 0.0f;
            break;
        case Opcode::MSG_MOVE_START_ASCEND:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::ASCENDING);
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::DESCENDING);
            break;
        case Opcode::MSG_MOVE_STOP_ASCEND:
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ASCENDING);
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::DESCENDING);
            break;
        case Opcode::MSG_MOVE_START_DESCEND:
            // Must set DESCENDING so outgoing movement packets carry the correct
            // flag during flight descent. Only clearing ASCENDING left the flag
            // field ambiguous (neither ascending nor descending).
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ASCENDING);
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::DESCENDING);
            break;
        default:
            break;
    }

    // Fire PLAYER_STARTED/STOPPED_MOVING on movement state transitions
    {
        const bool isMoving = (movementInfo.flags & kMoveMask) != 0;
        if (isMoving && !wasMoving && owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("PLAYER_STARTED_MOVING", {});
        else if (!isMoving && wasMoving && owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("PLAYER_STOPPED_MOVING", {});
    }

    if (opcode == Opcode::MSG_MOVE_SET_FACING) {
        lastFacingSendTimeMs_ = movementInfo.time;
        lastFacingSentOrientation_ = movementInfo.orientation;
    }

    // Keep fallTime current
    if (isFalling_ && movementInfo.hasFlag(MovementFlags::FALLING)) {
        uint32_t elapsed = (movementInfo.time >= fallStartMs_)
                               ? (movementInfo.time - fallStartMs_)
                               : 0u;
        movementInfo.fallTime = elapsed;
    } else if (!movementInfo.hasFlag(MovementFlags::FALLING)) {
        if (isFalling_) {
            isFalling_ = false;
            fallStartMs_ = 0;
        }
        movementInfo.fallTime = 0;
    }

    if (onTaxiFlight_ || taxiMountActive_ || taxiActivatePending_ || taxiClientActive_) {
        sanitizeMovementForTaxi();
    }

    bool includeTransportInWire = owner_.isOnTransport();
    if (includeTransportInWire && owner_.getTransportManager()) {
        if (auto* tr = owner_.getTransportManager()->getTransport(owner_.playerTransportGuidRef()); tr && tr->isM2) {
            includeTransportInWire = false;
        }
    }

    // Add transport data if player is on a server-recognized transport
    if (includeTransportInWire) {
        bool transportResolved = false;
        if (owner_.getTransportManager()) {
            auto* tr = owner_.getTransportManager()->getTransport(owner_.playerTransportGuidRef());
            if (tr) {
                transportResolved = true;
                glm::vec3 composed = owner_.getTransportManager()->getPlayerWorldPosition(owner_.playerTransportGuidRef(), owner_.playerTransportOffsetRef());
                movementInfo.x = composed.x;
                movementInfo.y = composed.y;
                movementInfo.z = composed.z;
            }
        }
        if (!transportResolved) {
            // Transport not tracked — don't send ONTRANSPORT to the server.
            // Sending stale transport GUID + local offset causes the server to
            // compute a bad world position and teleport us to map origin.
            LOG_WARNING("sendMovement: transport 0x", std::hex, owner_.playerTransportGuidRef(),
                        std::dec, " not found — clearing transport state");
            includeTransportInWire = false;
            owner_.clearPlayerTransport();
        }
    }
    if (includeTransportInWire) {
        movementInfo.flags |= static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
        movementInfo.transportGuid = owner_.playerTransportGuidRef();
        movementInfo.transportX = owner_.playerTransportOffsetRef().x;
        movementInfo.transportY = owner_.playerTransportOffsetRef().y;
        movementInfo.transportZ = owner_.playerTransportOffsetRef().z;
        movementInfo.transportTime = movementInfo.time;
        movementInfo.transportSeat = -1;
        movementInfo.transportTime2 = movementInfo.time;

        float transportYawCanonical = 0.0f;
        if (owner_.getTransportManager()) {
            if (auto* tr = owner_.getTransportManager()->getTransport(owner_.playerTransportGuidRef()); tr) {
                if (tr->hasServerYaw) {
                    transportYawCanonical = tr->serverYaw;
                } else {
                    transportYawCanonical = glm::eulerAngles(tr->rotation).z;
                }
            }
        }

        movementInfo.transportO =
            core::coords::normalizeAngleRad(movementInfo.orientation - transportYawCanonical);
    } else {
        movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
        movementInfo.transportGuid = 0;
        movementInfo.transportSeat = -1;
    }

    if (opcode == Opcode::MSG_MOVE_HEARTBEAT && isClassicLikeExpansion()) {
        const uint32_t locomotionFlags =
            static_cast<uint32_t>(MovementFlags::FORWARD) |
            static_cast<uint32_t>(MovementFlags::BACKWARD) |
            static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
            static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT) |
            static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
            static_cast<uint32_t>(MovementFlags::TURN_RIGHT) |
            static_cast<uint32_t>(MovementFlags::ASCENDING) |
            static_cast<uint32_t>(MovementFlags::FALLING) |
            static_cast<uint32_t>(MovementFlags::FALLINGFAR) |
            static_cast<uint32_t>(MovementFlags::SWIMMING);
        const bool stationaryIdle =
            !onTaxiFlight_ &&
            !taxiMountActive_ &&
            !taxiActivatePending_ &&
            !taxiClientActive_ &&
            !includeTransportInWire &&
            (movementInfo.flags & locomotionFlags) == 0;
        const uint32_t sinceLastHeartbeatMs =
            lastHeartbeatSendTimeMs_ != 0 && movementTime >= lastHeartbeatSendTimeMs_
                ? (movementTime - lastHeartbeatSendTimeMs_)
                : std::numeric_limits<uint32_t>::max();
        const bool unchangedState =
            std::abs(movementInfo.x - lastHeartbeatX_) < 0.01f &&
            std::abs(movementInfo.y - lastHeartbeatY_) < 0.01f &&
            std::abs(movementInfo.z - lastHeartbeatZ_) < 0.01f &&
            movementInfo.flags == lastHeartbeatFlags_ &&
            movementInfo.transportGuid == lastHeartbeatTransportGuid_;
        if (stationaryIdle && unchangedState && sinceLastHeartbeatMs < 1500U) {
            timeSinceLastMoveHeartbeat_ = 0.0f;
            return;
        }
        const uint32_t sinceLastNonHeartbeatMoveMs =
            lastNonHeartbeatMoveSendTimeMs_ != 0 && movementTime >= lastNonHeartbeatMoveSendTimeMs_
                ? (movementTime - lastNonHeartbeatMoveSendTimeMs_)
                : std::numeric_limits<uint32_t>::max();
        if (sinceLastNonHeartbeatMoveMs < 350U) {
            timeSinceLastMoveHeartbeat_ = 0.0f;
            return;
        }
    }

    LOG_DEBUG("Sending movement packet: opcode=0x", std::hex,
              wireOpcode(opcode), std::dec,
              (includeTransportInWire ? " ONTRANSPORT" : ""));

    // Block heartbeats from a near-origin position on Eastern Kingdoms (map 0).
    // These positions are almost certainly bugs (area trigger misfire, corrupted
    // save). Sending them tells the server the player is there, which persists
    // the bad position across sessions and creates a teleport loop.
    if (owner_.getCurrentMapId() == 0 &&
        std::abs(movementInfo.x) < 1000.0f && std::abs(movementInfo.y) < 1000.0f) {
        if (!restoreWorldTransferFallbackIfNearOrigin("sendMovement")) {
            LOG_WARNING("sendMovement: BLOCKED near-origin heartbeat canonical=(",
                        movementInfo.x, ", ", movementInfo.y, ", ", movementInfo.z,
                        ") onTransport=", owner_.isOnTransport(),
                        " transportGuid=0x", std::hex, owner_.playerTransportGuidRef(), std::dec,
                        " flags=0x", std::hex, movementInfo.flags, std::dec);
            return;
        }
    }

    // Convert canonical → server coordinates for the wire
    MovementInfo wireInfo = movementInfo;
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wireInfo.x, wireInfo.y, wireInfo.z));
    wireInfo.x = serverPos.x;
    wireInfo.y = serverPos.y;
    wireInfo.z = serverPos.z;

    // Periodic position audit — log every ~60 heartbeats (~30s) to trace position drift.
    if (opcode == Opcode::MSG_MOVE_HEARTBEAT && ++heartbeatLogCount_ % 60 == 0) {
        LOG_DEBUG("HEARTBEAT #", heartbeatLogCount_, " canonical=(",
                    movementInfo.x, ",", movementInfo.y, ",", movementInfo.z,
                    ") server=(", wireInfo.x, ",", wireInfo.y, ",", wireInfo.z,
                    ") flags=0x", std::hex, movementInfo.flags, std::dec);
    }

    wireInfo.orientation = core::coords::canonicalToServerYaw(wireInfo.orientation);

    if (includeTransportInWire) {
        glm::vec3 serverTransportPos = core::coords::canonicalToServer(
            glm::vec3(wireInfo.transportX, wireInfo.transportY, wireInfo.transportZ));
        wireInfo.transportX = serverTransportPos.x;
        wireInfo.transportY = serverTransportPos.y;
        wireInfo.transportZ = serverTransportPos.z;
        wireInfo.transportO = core::coords::normalizeAngleRad(-wireInfo.transportO);
    }

    // Build and send movement packet (expansion-specific format)
    auto packet = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildMovementPacket(opcode, wireInfo, owner_.getPlayerGuid())
        : MovementPacket::build(opcode, wireInfo, owner_.getPlayerGuid());
    owner_.getSocket()->send(packet);

    if (opcode == Opcode::MSG_MOVE_HEARTBEAT) {
        lastHeartbeatSendTimeMs_ = movementInfo.time;
        lastHeartbeatX_ = movementInfo.x;
        lastHeartbeatY_ = movementInfo.y;
        lastHeartbeatZ_ = movementInfo.z;
        lastHeartbeatFlags_ = movementInfo.flags;
        lastHeartbeatTransportGuid_ = movementInfo.transportGuid;
    } else {
        lastNonHeartbeatMoveSendTimeMs_ = movementInfo.time;
    }
}

// ============================================================
// sanitizeMovementForTaxi
// ============================================================

void MovementHandler::sanitizeMovementForTaxi() {
    constexpr uint32_t kClearTaxiFlags =
        static_cast<uint32_t>(MovementFlags::FORWARD) |
        static_cast<uint32_t>(MovementFlags::BACKWARD) |
        static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
        static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT) |
        static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
        static_cast<uint32_t>(MovementFlags::TURN_RIGHT) |
        static_cast<uint32_t>(MovementFlags::PITCH_UP) |
        static_cast<uint32_t>(MovementFlags::PITCH_DOWN) |
        static_cast<uint32_t>(MovementFlags::FALLING) |
        static_cast<uint32_t>(MovementFlags::FALLINGFAR) |
        static_cast<uint32_t>(MovementFlags::SWIMMING);

    movementInfo.flags &= ~kClearTaxiFlags;
    movementInfo.fallTime = 0;
    movementInfo.jumpVelocity = 0.0f;
    movementInfo.jumpSinAngle = 0.0f;
    movementInfo.jumpCosAngle = 0.0f;
    movementInfo.jumpXYSpeed = 0.0f;
    movementInfo.pitch = 0.0f;
}

// ============================================================
// forceClearTaxiAndMovementState
// ============================================================

void MovementHandler::forceClearTaxiAndMovementState() {
    taxiActivatePending_ = false;
    taxiActivateTimer_ = 0.0f;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    taxiServerCompletionPending_ = false;
    taxiRecoverPending_ = false;
    taxiStartGrace_ = 0.0f;
    onTaxiFlight_ = false;

    const bool clearingTaxiMount = taxiMountActive_;
    if (clearingTaxiMount && owner_.mountCallbackRef()) {
        owner_.mountCallbackRef()(0);
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;
    // A normal mount is owned by its aura/update-field path. Do not silently
    // zero it here without firing the mount callback: that leaves the rendered
    // mount behind and reconciliation spawns a duplicate. Unstuck explicitly
    // dismounts normal mounts before calling this movement reset.
    if (clearingTaxiMount) {
        owner_.currentMountDisplayIdRef() = 0;
    }
    owner_.vehicleIdRef() = 0;
    // Death/resurrect state is intentionally NOT cleared here.
    // Previously this method reset 10 death-related fields despite being named
    // "forceClearTaxiAndMovementState", which could cancel pending resurrections
    // or clear death state on taxi dismount. Death state is managed by
    // entity_controller (markPlayerDead) and the resurrect packet handlers.

    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    movementInfo.transportGuid = 0;
    owner_.clearPlayerTransport();

    if (owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
        sendMovement(Opcode::MSG_MOVE_STOP);
        sendMovement(Opcode::MSG_MOVE_STOP_STRAFE);
        sendMovement(Opcode::MSG_MOVE_STOP_TURN);
        sendMovement(Opcode::MSG_MOVE_STOP_SWIM);
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    LOG_INFO("Force-cleared taxi/movement state");
}

// ============================================================
// setPosition / setOrientation
// ============================================================

void MovementHandler::setPosition(float x, float y, float z) {
    movementInfo.x = x;
    movementInfo.y = y;
    movementInfo.z = z;
}

void MovementHandler::setOrientation(float orientation) {
    movementInfo.orientation = orientation;
}

// ============================================================
// dismount
// ============================================================

void MovementHandler::dismount() {
    if (!owner_.getSocket()) return;
    // The player's mount field goes on reading its old value for a few frames
    // after the request goes out. Believing it re-mounts the player behind their
    // back, and that stale value then makes the server's own SMSG_DISMOUNT look
    // transient and get thrown away — the mount blinked off, back on, and off
    // again over about two hundred milliseconds, with the character caught
    // holding the seated rider pose in between.
    constexpr float kDismountGraceSeconds = 1.5f;
    dismountGraceRemaining_ = kDismountGraceSeconds;
    uint32_t savedMountAura = owner_.mountAuraSpellIdRef();
    if (owner_.currentMountDisplayIdRef() != 0 || taxiMountActive_) {
        if (owner_.mountCallbackRef()) {
            owner_.mountCallbackRef()(0);
        }
        owner_.currentMountDisplayIdRef() = 0;
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        owner_.mountAuraSpellIdRef() = 0;
        LOG_INFO("Dismount: cleared local mount state, onTaxiFlight=", onTaxiFlight_);
    }
    uint16_t cancelMountWire = wireOpcode(Opcode::CMSG_CANCEL_MOUNT_AURA);
    if (cancelMountWire != 0xFFFF) {
        network::Packet pkt(cancelMountWire);
        owner_.getSocket()->send(pkt);
        LOG_INFO("Sent CMSG_CANCEL_MOUNT_AURA");
    } else if (savedMountAura != 0) {
        auto pkt = CancelAuraPacket::build(savedMountAura);
        owner_.getSocket()->send(pkt);
        LOG_INFO("Sent CMSG_CANCEL_AURA (mount spell ", savedMountAura, ") — Classic fallback");
    } else {
        for (const auto& a : owner_.getPlayerAuras()) {
            if (!a.isEmpty() && a.maxDurationMs < 0 && a.casterGuid == owner_.getPlayerGuid()) {
                auto pkt = CancelAuraPacket::build(a.spellId);
                owner_.getSocket()->send(pkt);
                LOG_INFO("Sent CMSG_CANCEL_AURA (spell ", a.spellId, ") — brute force dismount");
            }
        }
    }
}

// ============================================================
// Force Speed / Root / Flag Change Handlers
// ============================================================

// Shared force-ACK packet builder. All server-forced movement changes (speed,
// root, flag, collision-height, knockback) require the same ACK structure:
// GUID + counter + movement payload with server-space coordinates. Centralised
// here so transport coordinate conversion can't diverge between handlers.
network::Packet MovementHandler::buildForceAck(Opcode ackOpcode, uint32_t counter) {
    network::Packet ack(wireOpcode(ackOpcode));
    const bool legacyGuid =
        isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle");
    if (legacyGuid) {
        ack.writeUInt64(owner_.getPlayerGuid());
    } else {
        ack.writePackedGuid(owner_.getPlayerGuid());
    }
    ack.writeUInt32(counter);

    MovementInfo wire = movementInfo;
    wire.time = nextMovementTimestampMs();
    if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
        wire.transportTime = wire.time;
        wire.transportTime2 = wire.time;
    }
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wire.x, wire.y, wire.z));
    wire.x = serverPos.x;
    wire.y = serverPos.y;
    wire.z = serverPos.z;
    if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
        glm::vec3 serverTransport =
            core::coords::canonicalToServer(glm::vec3(wire.transportX, wire.transportY, wire.transportZ));
        wire.transportX = serverTransport.x;
        wire.transportY = serverTransport.y;
        wire.transportZ = serverTransport.z;
    }
    if (owner_.getPacketParsers()) {
        owner_.getPacketParsers()->writeMovementPayload(ack, wire);
    } else {
        MovementPacket::writeMovementPayload(ack, wire);
    }
    return ack;
}

void MovementHandler::handleForceSpeedChange(network::Packet& packet, const char* name,
                                              Opcode ackOpcode, float* speedStorage) {
    const bool fscTbcLike = isPreWotlk();
    uint64_t guid = fscTbcLike
        ? packet.readUInt64() : packet.readPackedGuid();
    uint32_t counter = packet.readUInt32();

    size_t remaining = packet.getRemainingSize();
    if (remaining >= 8) {
        packet.readUInt32();
    } else if (remaining >= 5) {
        packet.readUInt8();
    }
    float newSpeed = packet.readFloat();

    LOG_INFO("SMSG_FORCE_", name, "_CHANGE: guid=0x", std::hex, guid, std::dec,
             " counter=", counter, " speed=", newSpeed);

    if (guid != owner_.getPlayerGuid()) return;

    // Validate BEFORE sending ACK — if we echo a bad speed back to the server
    // but don't apply it locally, the client and server desync on movement speed.
    if (std::isnan(newSpeed) || newSpeed < 0.1f || newSpeed > 100.0f) {
        LOG_WARNING("Ignoring invalid ", name, " speed: ", newSpeed);
        return;
    }

    if (owner_.getSocket()) {
        auto ack = buildForceAck(ackOpcode, counter);
        ack.writeFloat(newSpeed);
        owner_.getSocket()->send(ack);
    }

    if (speedStorage) *speedStorage = newSpeed;
}

void MovementHandler::handleForceRunSpeedChange(network::Packet& packet) {
    // A speed change is not a dismount signal. Mount ownership is updated by
    // SMSG_DISMOUNT / UNIT_FIELD_MOUNTDISPLAYID instead.
    handleForceSpeedChange(packet, "RUN_SPEED", Opcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK, &serverRunSpeed_);
}

void MovementHandler::handleForceMoveRootState(network::Packet& packet, bool rooted) {
    const bool rootTbc = isPreWotlk();
    if (packet.getRemainingSize() < (rootTbc ? 8u : 2u)) return;
    uint64_t guid = rootTbc
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(4)) return;
    uint32_t counter = packet.readUInt32();

    LOG_INFO(rooted ? "SMSG_FORCE_MOVE_ROOT" : "SMSG_FORCE_MOVE_UNROOT",
             ": guid=0x", std::hex, guid, std::dec, " counter=", counter);

    if (guid != owner_.getPlayerGuid()) return;

    if (rooted) {
        movementInfo.flags |= static_cast<uint32_t>(MovementFlags::ROOT);
    } else {
        movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ROOT);
    }

    if (!owner_.getSocket()) return;
    Opcode ackOp = rooted ? Opcode::CMSG_FORCE_MOVE_ROOT_ACK : Opcode::CMSG_FORCE_MOVE_UNROOT_ACK;
    if (wireOpcode(ackOp) == 0xFFFF) return;
    owner_.getSocket()->send(buildForceAck(ackOp, counter));
}

void MovementHandler::handleForceMoveFlagChange(network::Packet& packet, const char* name,
                                                  Opcode ackOpcode, uint32_t flag, bool set) {
    const bool fmfTbcLike = isPreWotlk();
    if (packet.getRemainingSize() < (fmfTbcLike ? 8u : 2u)) return;
    uint64_t guid = fmfTbcLike
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(4)) return;
    uint32_t counter = packet.readUInt32();

    LOG_INFO("SMSG_FORCE_", name, ": guid=0x", std::hex, guid, std::dec, " counter=", counter);

    if (guid != owner_.getPlayerGuid()) return;

    if (flag != 0) {
        if (set) {
            movementInfo.flags |= flag;
        } else {
            movementInfo.flags &= ~flag;
        }
    }

    if (!owner_.getSocket()) return;
    if (wireOpcode(ackOpcode) == 0xFFFF) return;
    owner_.getSocket()->send(buildForceAck(ackOpcode, counter));
}

void MovementHandler::handleMoveSetCollisionHeight(network::Packet& packet) {
    const bool legacyGuid = isPreWotlk();
    if (packet.getRemainingSize() < (legacyGuid ? 8u : 2u)) return;
    uint64_t guid = legacyGuid ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(8)) return;
    uint32_t counter = packet.readUInt32();
    float height = packet.readFloat();

    LOG_INFO("SMSG_MOVE_SET_COLLISION_HGT: guid=0x", std::hex, guid, std::dec,
             " counter=", counter, " height=", height);

    if (guid != owner_.getPlayerGuid()) return;
    if (!owner_.getSocket()) return;

    if (wireOpcode(Opcode::CMSG_MOVE_SET_COLLISION_HGT_ACK) == 0xFFFF) return;
    // buildForceAck now handles transport coordinate conversion, fixing the
    // previous omission that caused desync when riding boats/zeppelins.
    auto ack = buildForceAck(Opcode::CMSG_MOVE_SET_COLLISION_HGT_ACK, counter);
    ack.writeFloat(height);
    owner_.getSocket()->send(ack);
}

void MovementHandler::handleMoveKnockBack(network::Packet& packet) {
    const bool mkbTbc = isPreWotlk();
    if (packet.getRemainingSize() < (mkbTbc ? 8u : 2u)) return;
    uint64_t guid = mkbTbc
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(20)) return;
    uint32_t counter = packet.readUInt32();
    float vcos    = packet.readFloat();
    float vsin    = packet.readFloat();
    float hspeed  = packet.readFloat();
    float vspeed  = packet.readFloat();

    LOG_INFO("SMSG_MOVE_KNOCK_BACK: guid=0x", std::hex, guid, std::dec,
             " counter=", counter, " vcos=", vcos, " vsin=", vsin,
             " hspeed=", hspeed, " vspeed=", vspeed);

    if (guid != owner_.getPlayerGuid()) return;

    if (owner_.knockBackCallbackRef()) {
        owner_.knockBackCallbackRef()(vcos, vsin, hspeed, vspeed);
    }

    if (!owner_.getSocket()) return;
    if (wireOpcode(Opcode::CMSG_MOVE_KNOCK_BACK_ACK) == 0xFFFF) return;
    owner_.getSocket()->send(buildForceAck(Opcode::CMSG_MOVE_KNOCK_BACK_ACK, counter));
}

// ============================================================
// Other Player / Creature Movement Handlers
// ============================================================

void MovementHandler::handleMoveSetSpeed(network::Packet& packet) {
    const bool useFull = isPreWotlk();
    uint64_t moverGuid = useFull
        ? packet.readUInt64() : packet.readPackedGuid();

    const size_t remaining = packet.getRemainingSize();
    if (remaining < 4) return;
    if (remaining > 4) {
        packet.setReadPos(packet.getSize() - 4);
    }

    float speed = packet.readFloat();
    if (!std::isfinite(speed) || speed <= 0.01f || speed > 200.0f) return;

    if (moverGuid != owner_.getPlayerGuid()) return;
    const uint16_t wireOp = packet.getOpcode();
    if      (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_RUN_SPEED))        serverRunSpeed_      = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_RUN_BACK_SPEED))   serverRunBackSpeed_  = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_WALK_SPEED))       serverWalkSpeed_     = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_SWIM_SPEED))       serverSwimSpeed_     = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_SWIM_BACK_SPEED))  serverSwimBackSpeed_ = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_FLIGHT_SPEED))     serverFlightSpeed_   = speed;
    else if (wireOp == wireOpcode(Opcode::MSG_MOVE_SET_FLIGHT_BACK_SPEED))serverFlightBackSpeed_= speed;
}

void MovementHandler::handleOtherPlayerMovement(network::Packet& packet) {
    // Same packed-guid mismatch found and fixed for MSG_MOVE_TELEPORT_ACK:
    // CMaNGOS sends a packed guid here on TBC too, not a raw UInt64. Reading
    // it as raw corrupted moverGuid, so getEntity(moverGuid) below always
    // missed - the entity still existed (created fine), but its position
    // never updated from these packets. Live-observed as another player's
    // tracked position staying frozen at wherever they were first spotted,
    // even while they kept walking.
    uint64_t moverGuid = packet.readPackedGuid();
    if (moverGuid == owner_.getPlayerGuid() || moverGuid == 0) {
        return;
    }

    MovementInfo info = {};
    info.flags = packet.readUInt32();
    uint8_t flags2Size = owner_.getPacketParsers() ? owner_.getPacketParsers()->movementFlags2Size() : 2;
    if (flags2Size == 2) info.flags2 = packet.readUInt16();
    else if (flags2Size == 1) info.flags2 = packet.readUInt8();
    info.time = packet.readUInt32();
    info.x = packet.readFloat();
    info.y = packet.readFloat();
    info.z = packet.readFloat();
    info.orientation = packet.readFloat();

    const uint32_t wireTransportFlag = owner_.getPacketParsers() ? owner_.getPacketParsers()->wireOnTransportFlag() : 0x00000200;
    const bool onTransport = (info.flags & wireTransportFlag) != 0;
    uint64_t transportGuid = 0;
    float tLocalX = 0, tLocalY = 0, tLocalZ = 0, tLocalO = 0;
    if (onTransport) {
        transportGuid = packet.readPackedGuid();
        tLocalX = packet.readFloat();
        tLocalY = packet.readFloat();
        tLocalZ = packet.readFloat();
        tLocalO = packet.readFloat();
        if (flags2Size >= 1) {
            /*uint32_t transportTime =*/ packet.readUInt32();
        }
        if (flags2Size >= 2) {
            /*int8_t transportSeat =*/ packet.readUInt8();
            if (info.flags2 & 0x0200) {
                /*uint32_t transportTime2 =*/ packet.readUInt32();
            }
        }
    }

    auto entity = owner_.getEntityManager().getEntity(moverGuid);
    if (!entity) {
        return;
    }

    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(info.x, info.y, info.z));
    float canYaw = core::coords::serverToCanonicalYaw(info.orientation);

    if (onTransport && transportGuid != 0 && owner_.getTransportManager()) {
        glm::vec3 localCanonical = owner_.getTransportManager()->serverToTransportLocal(
            transportGuid, glm::vec3(tLocalX, tLocalY, tLocalZ));
        owner_.setTransportAttachment(moverGuid, entity->getType(), transportGuid, localCanonical, true,
                               core::coords::serverToCanonicalYaw(tLocalO));
        glm::vec3 worldPos = owner_.getTransportManager()->getPlayerWorldPosition(transportGuid, localCanonical);
        canonical = worldPos;
    } else if (!onTransport) {
        owner_.clearTransportAttachment(moverGuid);
    }

    uint32_t durationMs = 120;
    auto itPrev = otherPlayerMoveTimeMs_.find(moverGuid);
    if (itPrev != otherPlayerMoveTimeMs_.end()) {
        uint32_t rawDt = info.time - itPrev->second;
        if (rawDt >= 20 && rawDt <= 2000) {
            float fDt = static_cast<float>(rawDt);
            auto& smoothed = otherPlayerSmoothedIntervalMs_[moverGuid];
            if (smoothed < 1.0f) smoothed = fDt;
            smoothed = 0.7f * smoothed + 0.3f * fDt;
            float clamped = std::max(60.0f, std::min(500.0f, smoothed));
            durationMs = static_cast<uint32_t>(clamped);
        }
    }
    otherPlayerMoveTimeMs_[moverGuid] = info.time;

    const uint16_t wireOp = packet.getOpcode();
    const bool isStopOpcode =
        (wireOp == wireOpcode(Opcode::MSG_MOVE_STOP)) ||
        (wireOp == wireOpcode(Opcode::MSG_MOVE_STOP_STRAFE)) ||
        (wireOp == wireOpcode(Opcode::MSG_MOVE_STOP_TURN)) ||
        (wireOp == wireOpcode(Opcode::MSG_MOVE_STOP_SWIM)) ||
        (wireOp == wireOpcode(Opcode::MSG_MOVE_FALL_LAND));
    const bool isJumpOpcode  = (wireOp == wireOpcode(Opcode::MSG_MOVE_JUMP));

    const float entityDuration = isStopOpcode ? 0.0f : (durationMs / 1000.0f);
    entity->startMoveTo(canonical.x, canonical.y, canonical.z, canYaw, entityDuration);

    if (owner_.creatureMoveCallbackRef()) {
        const uint32_t notifyDuration = isStopOpcode ? 0u : durationMs;
        owner_.creatureMoveCallbackRef()(moverGuid, canonical.x, canonical.y, canonical.z, notifyDuration);
    }

    if (owner_.unitAnimHintCallbackRef() && isJumpOpcode) {
        owner_.unitAnimHintCallbackRef()(moverGuid, 38u);
    }

    if (owner_.unitMoveFlagsCallbackRef()) {
        owner_.unitMoveFlagsCallbackRef()(moverGuid, info.flags);
    }
}

void MovementHandler::handleCompressedMoves(network::Packet& packet) {
    std::vector<uint8_t> decompressedStorage;
    const std::vector<uint8_t>* dataPtr = &packet.getData();

    const auto& rawData = packet.getData();
    const bool hasCompressedWrapper =
        rawData.size() >= 6 &&
        rawData[4] == 0x78 &&
        (rawData[5] == 0x01 || rawData[5] == 0x9C ||
         rawData[5] == 0xDA || rawData[5] == 0x5E);
    if (hasCompressedWrapper) {
        uint32_t decompressedSize = static_cast<uint32_t>(rawData[0]) |
                                    (static_cast<uint32_t>(rawData[1]) << 8) |
                                    (static_cast<uint32_t>(rawData[2]) << 16) |
                                    (static_cast<uint32_t>(rawData[3]) << 24);
        if (decompressedSize == 0 || decompressedSize > 65536) {
            LOG_WARNING("SMSG_COMPRESSED_MOVES: bad decompressedSize=", decompressedSize);
            return;
        }

        decompressedStorage.resize(decompressedSize);
        uLongf destLen = decompressedSize;
        int ret = uncompress(decompressedStorage.data(), &destLen,
                             rawData.data() + 4, rawData.size() - 4);
        if (ret != Z_OK) {
            LOG_WARNING("SMSG_COMPRESSED_MOVES: zlib error ", ret);
            return;
        }

        decompressedStorage.resize(destLen);
        dataPtr = &decompressedStorage;
    }

    const auto& data = *dataPtr;
    const size_t dataLen = data.size();

    uint16_t monsterMoveWire          = wireOpcode(Opcode::SMSG_MONSTER_MOVE);
    uint16_t monsterMoveTransportWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE_TRANSPORT);

    const std::array<uint16_t, 29> kMoveOpcodes = {
        wireOpcode(Opcode::MSG_MOVE_START_FORWARD),
        wireOpcode(Opcode::MSG_MOVE_START_BACKWARD),
        wireOpcode(Opcode::MSG_MOVE_STOP),
        wireOpcode(Opcode::MSG_MOVE_START_STRAFE_LEFT),
        wireOpcode(Opcode::MSG_MOVE_START_STRAFE_RIGHT),
        wireOpcode(Opcode::MSG_MOVE_STOP_STRAFE),
        wireOpcode(Opcode::MSG_MOVE_JUMP),
        wireOpcode(Opcode::MSG_MOVE_START_TURN_LEFT),
        wireOpcode(Opcode::MSG_MOVE_START_TURN_RIGHT),
        wireOpcode(Opcode::MSG_MOVE_STOP_TURN),
        wireOpcode(Opcode::MSG_MOVE_SET_FACING),
        wireOpcode(Opcode::MSG_MOVE_FALL_LAND),
        wireOpcode(Opcode::MSG_MOVE_HEARTBEAT),
        wireOpcode(Opcode::MSG_MOVE_START_SWIM),
        wireOpcode(Opcode::MSG_MOVE_STOP_SWIM),
        wireOpcode(Opcode::MSG_MOVE_SET_WALK_MODE),
        wireOpcode(Opcode::MSG_MOVE_SET_RUN_MODE),
        wireOpcode(Opcode::MSG_MOVE_START_PITCH_UP),
        wireOpcode(Opcode::MSG_MOVE_START_PITCH_DOWN),
        wireOpcode(Opcode::MSG_MOVE_STOP_PITCH),
        wireOpcode(Opcode::MSG_MOVE_START_ASCEND),
        wireOpcode(Opcode::MSG_MOVE_STOP_ASCEND),
        wireOpcode(Opcode::MSG_MOVE_START_DESCEND),
        wireOpcode(Opcode::MSG_MOVE_SET_PITCH),
        wireOpcode(Opcode::MSG_MOVE_GRAVITY_CHNG),
        wireOpcode(Opcode::MSG_MOVE_UPDATE_CAN_FLY),
        wireOpcode(Opcode::MSG_MOVE_UPDATE_CAN_TRANSITION_BETWEEN_SWIM_AND_FLY),
        wireOpcode(Opcode::MSG_MOVE_ROOT),
        wireOpcode(Opcode::MSG_MOVE_UNROOT),
    };

    struct CompressedMoveSubPacket {
        uint16_t opcode = 0;
        std::vector<uint8_t> payload;
    };
    struct DecodeResult {
        bool ok = false;
        bool overrun = false;
        bool usedPayloadOnlySize = false;
        size_t endPos = 0;
        size_t recognizedCount = 0;
        size_t subPacketCount = 0;
        std::vector<CompressedMoveSubPacket> packets;
    };

    auto isRecognizedSubOpcode = [&](uint16_t subOpcode) {
        return subOpcode == monsterMoveWire ||
               subOpcode == monsterMoveTransportWire ||
               std::find(kMoveOpcodes.begin(), kMoveOpcodes.end(), subOpcode) != kMoveOpcodes.end();
    };

    auto decodeSubPackets = [&](bool payloadOnlySize) -> DecodeResult {
        DecodeResult result;
        result.usedPayloadOnlySize = payloadOnlySize;
        size_t pos = 0;
        while (pos < dataLen) {
            if (pos + 1 > dataLen) break;
            uint8_t subSize = data[pos];
            if (subSize == 0) {
                result.ok = true;
                result.endPos = pos + 1;
                return result;
            }

            const size_t payloadLen = payloadOnlySize
                ? static_cast<size_t>(subSize)
                : (subSize >= 2 ? static_cast<size_t>(subSize) - 2 : 0);
            if (!payloadOnlySize && subSize < 2) {
                result.endPos = pos;
                return result;
            }

            const size_t packetLen = 1 + 2 + payloadLen;
            if (pos + packetLen > dataLen) {
                result.overrun = true;
                result.endPos = pos;
                return result;
            }

            uint16_t subOpcode = static_cast<uint16_t>(data[pos + 1]) |
                                 (static_cast<uint16_t>(data[pos + 2]) << 8);
            size_t payloadStart = pos + 3;

            CompressedMoveSubPacket subPacket;
            subPacket.opcode = subOpcode;
            subPacket.payload.assign(data.begin() + payloadStart,
                                     data.begin() + payloadStart + payloadLen);
            result.packets.push_back(std::move(subPacket));
            ++result.subPacketCount;
            if (isRecognizedSubOpcode(subOpcode)) {
                ++result.recognizedCount;
            }

            pos += packetLen;
        }
        result.ok = (result.endPos == 0 || result.endPos == dataLen);
        result.endPos = dataLen;
        return result;
    };

    DecodeResult decoded = decodeSubPackets(false);
    if (!decoded.ok || decoded.overrun) {
        DecodeResult payloadOnlyDecoded = decodeSubPackets(true);
        const bool preferPayloadOnly =
            payloadOnlyDecoded.ok &&
            (!decoded.ok || decoded.overrun || payloadOnlyDecoded.recognizedCount > decoded.recognizedCount);
        if (preferPayloadOnly) {
            decoded = std::move(payloadOnlyDecoded);
            static uint32_t payloadOnlyFallbackCount = 0;
            ++payloadOnlyFallbackCount;
            if (payloadOnlyFallbackCount <= 10 || (payloadOnlyFallbackCount % 100) == 0) {
                LOG_WARNING("SMSG_COMPRESSED_MOVES decoded via payload-only size fallback",
                            " (occurrence=", payloadOnlyFallbackCount, ")");
            }
        }
    }

    if (!decoded.ok || decoded.overrun) {
        LOG_WARNING("SMSG_COMPRESSED_MOVES: sub-packet overruns buffer at pos=", decoded.endPos);
        return;
    }

    std::unordered_set<uint16_t> unhandledSeen;

    for (const auto& entry : decoded.packets) {
        network::Packet subPacket(entry.opcode, entry.payload);

        if (entry.opcode == monsterMoveWire) {
            handleMonsterMove(subPacket);
        } else if (entry.opcode == monsterMoveTransportWire) {
            handleMonsterMoveTransport(subPacket);
        } else if (owner_.getState() == WorldState::IN_WORLD &&
                   std::find(kMoveOpcodes.begin(), kMoveOpcodes.end(), entry.opcode) != kMoveOpcodes.end()) {
            handleOtherPlayerMovement(subPacket);
        } else {
            if (unhandledSeen.insert(entry.opcode).second) {
                LOG_INFO("SMSG_COMPRESSED_MOVES: unhandled sub-opcode 0x",
                         std::hex, entry.opcode, std::dec, " payloadLen=", entry.payload.size());
            }
        }
    }
}

// ============================================================
// Monster Move Handlers
// ============================================================

void MovementHandler::handleMonsterMove(network::Packet& packet) {
    if (isActiveExpansion("classic") || isActiveExpansion("turtle")) {
        constexpr uint32_t kMaxMonsterMovesPerTick = 256;
        ++monsterMovePacketsThisTick_;
        if (monsterMovePacketsThisTick_ > kMaxMonsterMovesPerTick) {
            ++monsterMovePacketsDroppedThisTick_;
            if (monsterMovePacketsDroppedThisTick_ <= 3 ||
                (monsterMovePacketsDroppedThisTick_ % 100) == 0) {
                LOG_WARNING("SMSG_MONSTER_MOVE: per-tick cap exceeded, dropping packet",
                            " (processed=", monsterMovePacketsThisTick_,
                            " dropped=", monsterMovePacketsDroppedThisTick_, ")");
            }
            return;
        }
    }

    MonsterMoveData data;
    auto logMonsterMoveParseFailure = [&](const std::string& msg) {
        static uint32_t failCount = 0;
        ++failCount;
        if (failCount <= 10 || (failCount % 100) == 0) {
            LOG_WARNING(msg, " (occurrence=", failCount, ")");
        }
    };
    auto logWrappedUncompressedFallbackUsed = [&]() {
        static uint32_t wrappedUncompressedFallbackCount = 0;
        ++wrappedUncompressedFallbackCount;
        if (wrappedUncompressedFallbackCount <= 10 || (wrappedUncompressedFallbackCount % 100) == 0) {
            LOG_WARNING("SMSG_MONSTER_MOVE parsed via uncompressed wrapped-subpacket fallback",
                        " (occurrence=", wrappedUncompressedFallbackCount, ")");
        }
    };
    auto stripWrappedSubpacket = [&](const std::vector<uint8_t>& bytes, std::vector<uint8_t>& stripped) -> bool {
        if (bytes.size() < 3) return false;
        uint8_t subSize = bytes[0];
        if (subSize < 2) return false;
        size_t wrappedLen = static_cast<size_t>(subSize) + 1;
        if (wrappedLen != bytes.size()) return false;
        size_t payloadLen = static_cast<size_t>(subSize) - 2;
        if (3 + payloadLen > bytes.size()) return false;
        stripped.assign(bytes.begin() + 3, bytes.begin() + 3 + payloadLen);
        return true;
    };

    const auto& rawData = packet.getData();
    const bool allowTurtleMoveCompression = isActiveExpansion("turtle");
    bool isCompressed = allowTurtleMoveCompression &&
                        rawData.size() >= 6 &&
                        rawData[4] == 0x78 &&
                        (rawData[5] == 0x01 || rawData[5] == 0x9C ||
                         rawData[5] == 0xDA || rawData[5] == 0x5E);
    if (isCompressed) {
        uint32_t decompSize = static_cast<uint32_t>(rawData[0]) |
                              (static_cast<uint32_t>(rawData[1]) << 8) |
                              (static_cast<uint32_t>(rawData[2]) << 16) |
                              (static_cast<uint32_t>(rawData[3]) << 24);
        if (decompSize == 0 || decompSize > 65536) {
            LOG_WARNING("SMSG_MONSTER_MOVE: bad decompSize=", decompSize);
            return;
        }
        std::vector<uint8_t> decompressed(decompSize);
        uLongf destLen = decompSize;
        int ret = uncompress(decompressed.data(), &destLen,
                             rawData.data() + 4, rawData.size() - 4);
        if (ret != Z_OK) {
            LOG_WARNING("SMSG_MONSTER_MOVE: zlib error ", ret);
            return;
        }
        decompressed.resize(destLen);
        std::vector<uint8_t> stripped;
        bool hasWrappedForm = stripWrappedSubpacket(decompressed, stripped);

        bool parsed = false;
        if (hasWrappedForm) {
            network::Packet wrappedPacket(packet.getOpcode(), stripped);
            if (owner_.getPacketParsers()->parseMonsterMove(wrappedPacket, data)) {
                parsed = true;
            }
        }
        if (!parsed) {
            network::Packet decompPacket(packet.getOpcode(), decompressed);
            if (owner_.getPacketParsers()->parseMonsterMove(decompPacket, data)) {
                parsed = true;
            }
        }

        if (!parsed) {
            if (hasWrappedForm) {
                logMonsterMoveParseFailure("Failed to parse SMSG_MONSTER_MOVE (decompressed " +
                                           std::to_string(destLen) + " bytes, wrapped payload " +
                                           std::to_string(stripped.size()) + " bytes)");
            } else {
                logMonsterMoveParseFailure("Failed to parse SMSG_MONSTER_MOVE (decompressed " +
                                           std::to_string(destLen) + " bytes)");
            }
            return;
        }
    } else if (!owner_.getPacketParsers()->parseMonsterMove(packet, data)) {
        std::vector<uint8_t> stripped;
        if (stripWrappedSubpacket(rawData, stripped)) {
            network::Packet wrappedPacket(packet.getOpcode(), stripped);
            if (owner_.getPacketParsers()->parseMonsterMove(wrappedPacket, data)) {
                logWrappedUncompressedFallbackUsed();
            } else {
                logMonsterMoveParseFailure("Failed to parse SMSG_MONSTER_MOVE");
                return;
            }
        } else {
            logMonsterMoveParseFailure("Failed to parse SMSG_MONSTER_MOVE");
            return;
        }
    }

    auto entity = owner_.getEntityManager().getEntity(data.guid);
    if (!entity) {
        return;
    }

    // Classic/TBC creature patrols encode ground locomotion in the spline:
    // RUNMODE set means Run, while a clear bit means Walk. Publish it before
    // the render callback. WotLK uses the separate spline mode opcodes already
    // registered above, and 0x100 means DONE there, so do not reinterpret it.
    const bool usesPreWotlkSplineFlags = isActiveExpansion("classic") ||
                                          isActiveExpansion("turtle") ||
                                          isActiveExpansion("tbc");
    if (usesPreWotlkSplineFlags && entity->getType() == ObjectType::UNIT &&
        owner_.unitMoveFlagsCallbackRef()) {
        const uint32_t locomotionFlags =
            isPreWotlkSplineWalking(data.splineFlags)
                ? static_cast<uint32_t>(MovementFlags::WALKING)
                : 0u;
        owner_.unitMoveFlagsCallbackRef()(data.guid, locomotionFlags);
    }

    if (data.hasDest) {
        glm::vec3 destCanonical = core::coords::serverToCanonical(
            glm::vec3(data.destX, data.destY, data.destZ));

        float orientation = entity->getOrientation();
        if (data.moveType == 4) {
            orientation = core::coords::serverToCanonicalYaw(data.facingAngle);
        } else if (data.moveType == 3) {
            auto target = owner_.getEntityManager().getEntity(data.facingTarget);
            if (target) {
                float dx = target->getX() - entity->getX();
                float dy = target->getY() - entity->getY();
                if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                    orientation = std::atan2(-dy, dx);
                }
            }
        } else {
            float dx = destCanonical.x - entity->getX();
            float dy = destCanonical.y - entity->getY();
            if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                orientation = std::atan2(-dy, dx);
            }
        }

        if (data.moveType != 3) {
            glm::vec3 startCanonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            float travelDx = destCanonical.x - startCanonical.x;
            float travelDy = destCanonical.y - startCanonical.y;
            float travelLen = std::sqrt(travelDx * travelDx + travelDy * travelDy);
            if (travelLen > 0.5f) {
                float travelAngle = std::atan2(-travelDy, travelDx);
                float diff = orientation - travelAngle;
                while (diff >  static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
                while (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);
                if (std::abs(diff) > static_cast<float>(M_PI) * 0.5f) {
                    orientation = travelAngle;
                }
            }
        }

        // Build full path: start → waypoints → destination (all in canonical coords)
        if (!data.waypoints.empty()) {
            glm::vec3 startCanonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            std::vector<std::array<float, 3>> path;
            // Path = start + waypoints + dest. Reserve once to avoid the
            // 2–3 reallocations push_back would trigger as it doubles.
            path.reserve(data.waypoints.size() + 2);
            path.push_back({startCanonical.x, startCanonical.y, startCanonical.z});
            for (const auto& wp : data.waypoints) {
                glm::vec3 wpCanonical = core::coords::serverToCanonical(
                    glm::vec3(wp.x, wp.y, wp.z));
                path.push_back({wpCanonical.x, wpCanonical.y, wpCanonical.z});
            }
            path.push_back({destCanonical.x, destCanonical.y, destCanonical.z});
            entity->startMoveAlongPath(path, orientation, data.duration / 1000.0f);
        } else {
            entity->startMoveTo(destCanonical.x, destCanonical.y, destCanonical.z,
                                orientation, data.duration / 1000.0f);
        }

        if (owner_.creatureMoveCallbackRef()) {
            owner_.creatureMoveCallbackRef()(data.guid,
                destCanonical.x, destCanonical.y, destCanonical.z,
                data.duration);
        }
    } else if (data.moveType == 1) {
        glm::vec3 posCanonical = core::coords::serverToCanonical(
            glm::vec3(data.x, data.y, data.z));
        entity->setPosition(posCanonical.x, posCanonical.y, posCanonical.z,
                            entity->getOrientation());

        if (owner_.creatureMoveCallbackRef()) {
            owner_.creatureMoveCallbackRef()(data.guid,
                posCanonical.x, posCanonical.y, posCanonical.z, 0);
        }
    } else if (data.moveType == 4) {
        float orientation = core::coords::serverToCanonicalYaw(data.facingAngle);
        glm::vec3 posCanonical = core::coords::serverToCanonical(
            glm::vec3(data.x, data.y, data.z));
        entity->setPosition(posCanonical.x, posCanonical.y, posCanonical.z, orientation);
        if (owner_.creatureMoveCallbackRef()) {
            owner_.creatureMoveCallbackRef()(data.guid,
                posCanonical.x, posCanonical.y, posCanonical.z, 0);
        }
    } else if (data.moveType == 3 && data.facingTarget != 0) {
        auto target = owner_.getEntityManager().getEntity(data.facingTarget);
        if (target) {
            float dx = target->getX() - entity->getX();
            float dy = target->getY() - entity->getY();
            if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                float orientation = std::atan2(-dy, dx);
                entity->setOrientation(orientation);
            }
        }
    }
}

void MovementHandler::handleMonsterMoveTransport(network::Packet& packet) {
    if (packet.getRemainingSize() < 8 + 1 + 8 + 12) return;
    uint64_t moverGuid = packet.readUInt64();
    /*uint8_t unk =*/ packet.readUInt8();
    uint64_t transportGuid = packet.readUInt64();

    float localX = packet.readFloat();
    float localY = packet.readFloat();
    float localZ = packet.readFloat();

    auto entity = owner_.getEntityManager().getEntity(moverGuid);
    if (!entity) return;

    if (packet.getReadPos() + 5 > packet.getSize()) {
        if (owner_.getTransportManager()) {
            glm::vec3 localCanonical = owner_.getTransportManager()->serverToTransportLocal(
                transportGuid, glm::vec3(localX, localY, localZ));
            owner_.setTransportAttachment(moverGuid, entity->getType(), transportGuid, localCanonical, false, 0.0f);
            glm::vec3 worldPos = owner_.getTransportManager()->getPlayerWorldPosition(transportGuid, localCanonical);
            entity->setPosition(worldPos.x, worldPos.y, worldPos.z, entity->getOrientation());
            if (entity->getType() == ObjectType::UNIT && owner_.creatureMoveCallbackRef())
                owner_.creatureMoveCallbackRef()(moverGuid, worldPos.x, worldPos.y, worldPos.z, 0);
        }
        return;
    }

    /*uint32_t splineId =*/ packet.readUInt32();
    uint8_t moveType = packet.readUInt8();

    if (moveType == 1) {
        if (owner_.getTransportManager()) {
            glm::vec3 localCanonical = owner_.getTransportManager()->serverToTransportLocal(
                transportGuid, glm::vec3(localX, localY, localZ));
            owner_.setTransportAttachment(moverGuid, entity->getType(), transportGuid, localCanonical, false, 0.0f);
            glm::vec3 worldPos = owner_.getTransportManager()->getPlayerWorldPosition(transportGuid, localCanonical);
            entity->setPosition(worldPos.x, worldPos.y, worldPos.z, entity->getOrientation());
            if (entity->getType() == ObjectType::UNIT && owner_.creatureMoveCallbackRef())
                owner_.creatureMoveCallbackRef()(moverGuid, worldPos.x, worldPos.y, worldPos.z, 0);
        }
        return;
    }

    float facingAngle = entity->getOrientation();
    if (moveType == 2) {
        if (packet.getReadPos() + 12 > packet.getSize()) return;
        float sx = packet.readFloat(), sy = packet.readFloat(), sz = packet.readFloat();
        facingAngle = std::atan2(-(sy - localY), sx - localX);
        (void)sz;
    } else if (moveType == 3) {
        if (packet.getReadPos() + 8 > packet.getSize()) return;
        uint64_t tgtGuid = packet.readUInt64();
        if (auto tgt = owner_.getEntityManager().getEntity(tgtGuid)) {
            float dx = tgt->getX() - entity->getX();
            float dy = tgt->getY() - entity->getY();
            if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f)
                facingAngle = std::atan2(-dy, dx);
        }
    } else if (moveType == 4) {
        if (packet.getReadPos() + 4 > packet.getSize()) return;
        facingAngle = core::coords::serverToCanonicalYaw(packet.readFloat());
    }

    if (packet.getReadPos() + 4 > packet.getSize()) return;
    uint32_t splineFlags = packet.readUInt32();

    // Consolidated spline body parser
    SplineBlockData spline;
    if (!parseMonsterMoveSplineBody(packet, spline, splineFlags,
                                    glm::vec3(localX, localY, localZ),
                                    /*useTbcUncompressedMask=*/false,
                                    isPreWotlk() ? SplineFlagSet::PreWotlk
                                                 : SplineFlagSet::Wotlk)) {
        return;
    }
    uint32_t duration = spline.duration;
    float destLocalX = spline.hasDest ? spline.destination.x : localX;
    float destLocalY = spline.hasDest ? spline.destination.y : localY;
    float destLocalZ = spline.hasDest ? spline.destination.z : localZ;
    bool hasDest = spline.hasDest;

    const bool usesPreWotlkSplineFlags = isActiveExpansion("classic") ||
                                          isActiveExpansion("turtle") ||
                                          isActiveExpansion("tbc");
    if (usesPreWotlkSplineFlags && entity->getType() == ObjectType::UNIT &&
        owner_.unitMoveFlagsCallbackRef()) {
        const uint32_t locomotionFlags =
            isPreWotlkSplineWalking(splineFlags)
                ? static_cast<uint32_t>(MovementFlags::WALKING)
                : 0u;
        owner_.unitMoveFlagsCallbackRef()(moverGuid, locomotionFlags);
    }

    if (!owner_.getTransportManager()) {
        LOG_WARNING("SMSG_MONSTER_MOVE_TRANSPORT: TransportManager not available for mover 0x",
                    std::hex, moverGuid, std::dec);
        return;
    }

    glm::vec3 startLocalCanonical = owner_.getTransportManager()->serverToTransportLocal(
        transportGuid, glm::vec3(localX, localY, localZ));

    if (hasDest && duration > 0) {
        glm::vec3 destLocalCanonical = owner_.getTransportManager()->serverToTransportLocal(
            transportGuid, glm::vec3(destLocalX, destLocalY, destLocalZ));
        glm::vec3 destWorld  = owner_.getTransportManager()->getPlayerWorldPosition(transportGuid, destLocalCanonical);

        if (moveType == 0) {
            float dx = destLocalCanonical.x - startLocalCanonical.x;
            float dy = destLocalCanonical.y - startLocalCanonical.y;
            if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f)
                facingAngle = std::atan2(-dy, dx);
        }

        owner_.setTransportAttachment(moverGuid, entity->getType(), transportGuid, destLocalCanonical, false, 0.0f);
        entity->startMoveTo(destWorld.x, destWorld.y, destWorld.z, facingAngle, duration / 1000.0f);

        if (entity->getType() == ObjectType::UNIT && owner_.creatureMoveCallbackRef())
            owner_.creatureMoveCallbackRef()(moverGuid, destWorld.x, destWorld.y, destWorld.z, duration);

        LOG_DEBUG("SMSG_MONSTER_MOVE_TRANSPORT: mover=0x", std::hex, moverGuid,
                  " transport=0x", transportGuid, std::dec,
                  " dur=", duration, "ms dest=(", destWorld.x, ",", destWorld.y, ",", destWorld.z, ")");
    } else {
        glm::vec3 startWorld = owner_.getTransportManager()->getPlayerWorldPosition(transportGuid, startLocalCanonical);
        owner_.setTransportAttachment(moverGuid, entity->getType(), transportGuid, startLocalCanonical, false, 0.0f);
        entity->setPosition(startWorld.x, startWorld.y, startWorld.z, facingAngle);
        if (entity->getType() == ObjectType::UNIT && owner_.creatureMoveCallbackRef())
            owner_.creatureMoveCallbackRef()(moverGuid, startWorld.x, startWorld.y, startWorld.z, 0);
    }
}

// ============================================================
// Teleport Handlers
// ============================================================

void MovementHandler::handleTeleportAck(network::Packet& packet) {
    if (packet.getRemainingSize() < 1u) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK too short");
        return;
    }

    // CMaNGOS sends a packed guid here on every expansion, including TBC -
    // confirmed by live packet capture: reading a raw UInt64 instead
    // misaligned every field after it, so the guid never matched the local
    // player and the position update was silently dropped as "remote entity".
    uint64_t guid = packet.readPackedGuid();
    if (!packet.hasRemaining(4)) return;
    uint32_t counter = packet.readUInt32();

    const bool taNoFlags2 = isPreWotlk();
    // Pre-WotLK sends one extra byte here that isn't part of the documented
    // MovementInfo layout - confirmed by live capture (byte value 0x04,
    // constant across samples) and verified against a known ground-truth
    // teleport target (.go xyz to ironforge-city decoded to the exact
    // expected x/y/z only once this byte was accounted for).
    const size_t minMoveSz = taNoFlags2 ? (4 + 4 + 1 + 4 * 4) : (4 + 2 + 4 + 4 * 4);
    if (packet.getRemainingSize() < minMoveSz) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK: not enough data for movement info");
        return;
    }

    packet.readUInt32();  // moveFlags
    if (!taNoFlags2)
        packet.readUInt16();  // moveFlags2 (WotLK only)
    uint32_t moveTime = packet.readUInt32();
    if (taNoFlags2)
        packet.readUInt8();  // unknown byte, pre-WotLK only (see comment above)
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();

    LOG_WARNING("MSG_MOVE_TELEPORT_ACK: guid=0x", std::hex, guid, std::dec,
                " counter=", counter,
                " pos=(", serverX, ", ", serverY, ", ", serverZ, ")",
                " currentPos=(", movementInfo.x, ", ", movementInfo.y, ", ", movementInfo.z, ")");

    if (guid != owner_.getPlayerGuid()) {
        glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
        auto entity = owner_.getEntityManager().getEntity(guid);
        if (entity) {
            entity->setPosition(canonical.x, canonical.y, canonical.z,
                                core::coords::serverToCanonicalYaw(orientation));
        }
        LOG_INFO("MSG_MOVE_TELEPORT_ACK for remote entity 0x", std::hex, guid, std::dec, " — ignored for local player");
        return;
    }

    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));

    // Reject teleports to a near-origin position on Eastern Kingdoms (map 0).
    // No legitimate gameplay sends the player there; in this codebase it has
    // only ever been an area-trigger-destination misfire on the server side.
    // Refusing it (no ACK, no position update, no world reload) keeps the
    // client at its current position. Heartbeats from the real position will
    // eventually convince the server's anti-cheat to update its record.
    if (owner_.getCurrentMapId() == 0 &&
        std::abs(canonical.x) < 1000.0f && std::abs(canonical.y) < 1000.0f) {
        LOG_WARNING("REJECTED MSG_MOVE_TELEPORT to near-origin canonical=(",
                    canonical.x, ", ", canonical.y, ", ", canonical.z, ")"
                    " — keeping current position");
        return;
    }

    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
    movementInfo.flags = 0;

    // Clear cast bar on teleport — SpellHandler owns the casting_ flag
    if (owner_.getSpellHandler()) owner_.getSpellHandler()->resetCastState();

    // Suppress area triggers briefly after teleport. A one-shot flag is not
    // enough — the player can leave and re-enter a trigger within seconds and
    // get teleported again before the world has finished loading. Deeprun Tram
    // (map 369) is a narrow hallway with portal triggers close to the spawn,
    // so keep its grace window short enough that exits remain usable.
    const bool deeprunTram = owner_.getCurrentMapId() == 369;
    owner_.activeAreaTriggersRef().clear();
    owner_.areaTriggerCheckTimerRef() = deeprunTram ? -1.0f : -5.0f;
    owner_.areaTriggerSuppressFirstRef() = true;
    owner_.areaTriggerCooldownRef() = deeprunTram ? 1.5f : 10.0f;

    if (owner_.getSocket()) {
        network::Packet ack(wireOpcode(Opcode::MSG_MOVE_TELEPORT_ACK));
        const bool legacyGuidAck =
            isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle");
        if (legacyGuidAck) {
            ack.writeUInt64(owner_.getPlayerGuid());
        } else {
            ack.writePackedGuid(owner_.getPlayerGuid());
        }
        ack.writeUInt32(counter);
        ack.writeUInt32(moveTime);
        owner_.getSocket()->send(ack);
        LOG_INFO("Sent MSG_MOVE_TELEPORT_ACK response");

        // Immediately tell the server our position so it doesn't keep using
        // stale pre-teleport coordinates for area-trigger / anti-cheat checks.
        sendMovement(Opcode::MSG_MOVE_STOP);
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    if (owner_.worldEntryCallbackRef()) {
        owner_.worldEntryCallbackRef()(owner_.currentMapIdRef(), serverX, serverY, serverZ, false);
    }
}

void MovementHandler::handleNewWorld(network::Packet& packet) {
    if (!packet.hasRemaining(20)) {
        LOG_WARNING("SMSG_NEW_WORLD too short");
        return;
    }

    uint32_t mapId = packet.readUInt32();
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();
    const bool crossMapTransportTransfer = owner_.isOnTransport() &&
        mapId != owner_.getCurrentMapId();
    if (crossMapTransportTransfer) {
        // While riding a cross-continent transport, SMSG_NEW_WORLD carries the
        // player's transport-local coordinates. They are not a world position
        // near map origin. Hold them until the same ship is registered on the
        // destination map, then compose them through its new transform.
        owner_.beginPlayerTransportWorldTransfer(
            mapId, core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ)));
    }
    const uint32_t transferAreaTriggerId = lastAreaTriggerId_;
    lastAreaTriggerId_ = 0;
    postTransferReturnAreaTriggerId_ = findKnownReturnAreaTrigger(transferAreaTriggerId);
    postTransferReturnAreaTriggerSawNear_ = false;
    const bool hasPendingAreaTriggerDestination = pendingAreaTriggerDestinationValid_;
    const uint32_t pendingAreaTriggerDestinationMapId = pendingAreaTriggerDestinationMapId_;
    const glm::vec3 pendingAreaTriggerDestinationServerPos = pendingAreaTriggerDestinationServerPos_;
    const float pendingAreaTriggerDestinationServerO = pendingAreaTriggerDestinationServerO_;
    pendingAreaTriggerDestinationValid_ = false;
    const auto* knownTransferDestination = findKnownAreaTriggerDestination(transferAreaTriggerId, mapId);

    bool transferFallbackValid = false;
    glm::vec3 transferFallbackServerPos(0.0f);
    float transferFallbackServerO = orientation;
    if (hasPendingAreaTriggerDestination && pendingAreaTriggerDestinationMapId == mapId) {
        transferFallbackValid = true;
        transferFallbackServerPos = pendingAreaTriggerDestinationServerPos;
        transferFallbackServerO = pendingAreaTriggerDestinationServerO;
    } else if (knownTransferDestination) {
        transferFallbackValid = true;
        transferFallbackServerPos =
            glm::vec3(knownTransferDestination->serverX,
                      knownTransferDestination->serverY,
                      knownTransferDestination->serverZ);
        transferFallbackServerO = knownTransferDestination->serverO;
    }
    if (transferFallbackValid) {
        worldTransferFallbackValid_ = true;
        worldTransferFallbackMapId_ = mapId;
        worldTransferFallbackTriggerId_ = transferAreaTriggerId;
        worldTransferFallbackCanonicalPos_ = core::coords::serverToCanonical(transferFallbackServerPos);
        worldTransferFallbackCanonicalO_ = core::coords::serverToCanonicalYaw(transferFallbackServerO);
        orientation = transferFallbackServerO;
        LOG_WARNING("World transfer fallback armed: trigger=", transferAreaTriggerId,
                    " map=", mapId,
                    " canonical=(",
                    worldTransferFallbackCanonicalPos_.x, ", ",
                    worldTransferFallbackCanonicalPos_.y, ", ",
                    worldTransferFallbackCanonicalPos_.z, ")");
    } else {
        worldTransferFallbackValid_ = false;
    }

    LOG_INFO("SMSG_NEW_WORLD: mapId=", mapId,
             " pos=(", serverX, ", ", serverY, ", ", serverZ, ")",
             " orient=", orientation);

    glm::vec3 receivedCanonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
    const bool badEasternKingdomsNearOrigin =
        mapId == 0 && std::abs(receivedCanonical.x) < 1000.0f && std::abs(receivedCanonical.y) < 1000.0f;
    if (badEasternKingdomsNearOrigin) {
        if (hasPendingAreaTriggerDestination && pendingAreaTriggerDestinationMapId == mapId) {
            LOG_WARNING("Correcting bad SMSG_NEW_WORLD near-origin destination for area trigger ",
                        transferAreaTriggerId,
                        ": received server=(", serverX, ", ", serverY, ", ", serverZ,
                        ") using pending server=(", pendingAreaTriggerDestinationServerPos.x, ", ",
                        pendingAreaTriggerDestinationServerPos.y, ", ",
                        pendingAreaTriggerDestinationServerPos.z, ")");
            serverX = pendingAreaTriggerDestinationServerPos.x;
            serverY = pendingAreaTriggerDestinationServerPos.y;
            serverZ = pendingAreaTriggerDestinationServerPos.z;
            orientation = pendingAreaTriggerDestinationServerO;
            receivedCanonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
        } else if (knownTransferDestination) {
            LOG_WARNING("Correcting bad SMSG_NEW_WORLD near-origin destination for area trigger ",
                        transferAreaTriggerId,
                        ": received server=(", serverX, ", ", serverY, ", ", serverZ,
                        ") using server=(", knownTransferDestination->serverX, ", ",
                        knownTransferDestination->serverY, ", ", knownTransferDestination->serverZ, ")");
            serverX = knownTransferDestination->serverX;
            serverY = knownTransferDestination->serverY;
            serverZ = knownTransferDestination->serverZ;
            orientation = knownTransferDestination->serverO;
            receivedCanonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
        } else {
            LOG_WARNING("SMSG_NEW_WORLD near-origin destination on map 0 without known area trigger fallback: trigger=",
                        transferAreaTriggerId,
                        " server=(", serverX, ", ", serverY, ", ", serverZ, ")");
        }
    }

    const bool isSameMap       = (mapId == owner_.currentMapIdRef());
    const bool isResurrection  = owner_.resurrectPendingRef();
    if (isSameMap && isResurrection) {
        LOG_INFO("SMSG_NEW_WORLD same-map resurrection — skipping world reload");

        glm::vec3 canonical = receivedCanonical;
        movementInfo.x = canonical.x;
        movementInfo.y = canonical.y;
        movementInfo.z = canonical.z;
        movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
        movementInfo.flags  = 0;
        movementInfo.flags2 = 0;

        owner_.resurrectPendingRef()       = false;
        owner_.resurrectRequestPendingRef() = false;
        owner_.releasedSpiritRef()         = false;
        owner_.playerDeadRef()             = false;
        owner_.repopPendingRef()           = false;
        owner_.pendingSpiritHealerGuidRef() = 0;
        owner_.resurrectCasterGuidRef()    = 0;
        owner_.corpseMapIdRef()            = 0;
        owner_.corpsePositionValidRef()    = false;
        owner_.corpseGuidRef()             = 0;
        owner_.clearHostileAttackers();
        owner_.stopAutoAttack();
        owner_.tabCycleStaleRef() = true;
        owner_.resetCastState();

        const bool deeprunTram = owner_.getCurrentMapId() == 369;
        owner_.activeAreaTriggersRef().clear();
        owner_.areaTriggerCheckTimerRef() = deeprunTram ? -1.0f : -5.0f;
        owner_.areaTriggerSuppressFirstRef() = true;
        owner_.areaTriggerCooldownRef() = deeprunTram ? 1.5f : 10.0f;

        if (owner_.getSocket()) {
            network::Packet ack(wireOpcode(Opcode::MSG_MOVE_WORLDPORT_ACK));
            owner_.getSocket()->send(ack);
            LOG_INFO("Sent MSG_MOVE_WORLDPORT_ACK (resurrection)");

            sendMovement(Opcode::MSG_MOVE_STOP);
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
        }
        return;
    }

    owner_.currentMapIdRef() = mapId;
    owner_.inInstanceRef() = false;
    if (owner_.getSocket()) {
        owner_.getSocket()->tracePacketsFor(std::chrono::seconds(12), "new_world");
    }

    glm::vec3 canonical = receivedCanonical;
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    serverMovementAllowed_ = true;
    owner_.resurrectPendingRef() = false;
    owner_.resurrectRequestPendingRef() = false;
    onTaxiFlight_ = false;
    taxiMountActive_ = false;
    taxiActivatePending_ = false;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    taxiServerCompletionPending_ = false;
    taxiRecoverPending_ = false;
    taxiStartGrace_ = 0.0f;
    owner_.currentMountDisplayIdRef() = 0;
    taxiMountDisplayId_ = 0;
    if (owner_.mountCallbackRef()) {
        owner_.mountCallbackRef()(0);
    }

    for (const auto& [guid, entity] : owner_.getEntityManager().getEntities()) {
        if (guid == owner_.getPlayerGuid()) continue;
        if (entity->getType() == ObjectType::UNIT && owner_.creatureDespawnCallbackRef()) {
            owner_.creatureDespawnCallbackRef()(guid);
        } else if (entity->getType() == ObjectType::PLAYER && owner_.playerDespawnCallbackRef()) {
            owner_.playerDespawnCallbackRef()(guid);
        } else if (entity->getType() == ObjectType::GAMEOBJECT && owner_.gameObjectDespawnCallbackRef()) {
            owner_.gameObjectDespawnCallbackRef()(guid);
        }
    }
    owner_.otherPlayerVisibleItemEntriesRef().clear();
    owner_.otherPlayerVisibleDirtyRef().clear();
    otherPlayerMoveTimeMs_.clear();
    if (owner_.getSpellHandler()) owner_.getSpellHandler()->clearUnitCastStates();
    owner_.unitAurasCacheRef().clear();
    owner_.clearCombatText();
    owner_.getEntityManager().clear();
    owner_.clearHostileAttackers();
    owner_.worldStatesRef().clear();
    owner_.gossipPoisRef().clear();
    owner_.worldStateMapIdRef() = mapId;
    owner_.worldStateZoneIdRef() = 0;
    owner_.activeAreaTriggersRef().clear();
    const bool deeprunTram = mapId == 369;
    owner_.areaTriggerCheckTimerRef() = deeprunTram ? -1.0f : -5.0f;
    owner_.areaTriggerSuppressFirstRef() = true;
    owner_.areaTriggerCooldownRef() = deeprunTram ? 1.5f : 10.0f;
    owner_.stopAutoAttack();
    owner_.resetCastState();

    if (owner_.getSocket()) {
        network::Packet ack(wireOpcode(Opcode::MSG_MOVE_WORLDPORT_ACK));
        owner_.getSocket()->send(ack);
        LOG_INFO("Sent MSG_MOVE_WORLDPORT_ACK");

        // Sync position immediately so the server doesn't keep stale coordinates.
        sendMovement(Opcode::MSG_MOVE_STOP);
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    owner_.timeSinceLastPingRef() = 0.0f;
    if (owner_.getSocket()) {
        LOG_WARNING("World transfer keepalive: sending immediate ping after MSG_MOVE_WORLDPORT_ACK");
        owner_.sendPing();
    }

    if (owner_.worldEntryCallbackRef()) {
        owner_.worldEntryCallbackRef()(mapId, serverX, serverY, serverZ, isSameMap);
    }

    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("PLAYER_ENTERING_WORLD", {"0"});
        owner_.addonEventCallbackRef()("ZONE_CHANGED_NEW_AREA", {});
    }
}

// ============================================================
// Taxi / Flight Path Handlers
// ============================================================

void MovementHandler::loadTaxiDbc() {
    if (taxiDbcLoaded_) return;
    taxiDbcLoaded_ = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    auto nodesDbc = am->loadDBC("TaxiNodes.dbc");
    if (nodesDbc && nodesDbc->isLoaded()) {
        const auto* tnL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiNodes") : nullptr;
        // Cache field indices before the loop
        const uint32_t tnIdField   = tnL ? (*tnL)["ID"]    : 0;
        const uint32_t tnMapField  = tnL ? (*tnL)["MapID"] : 1;
        const uint32_t tnXField    = tnL ? (*tnL)["X"]     : 2;
        const uint32_t tnYField    = tnL ? (*tnL)["Y"]     : 3;
        const uint32_t tnZField    = tnL ? (*tnL)["Z"]     : 4;
        const uint32_t tnNameField = tnL ? (*tnL)["Name"]  : 5;
        const uint32_t mountAllianceField = tnL ? (*tnL)["MountDisplayIdAlliance"]         : 22;
        const uint32_t mountHordeField    = tnL ? (*tnL)["MountDisplayIdHorde"]            : 23;
        const uint32_t mountAllianceFB    = tnL ? (*tnL)["MountDisplayIdAllianceFallback"] : 20;
        const uint32_t mountHordeFB       = tnL ? (*tnL)["MountDisplayIdHordeFallback"]    : 21;
        uint32_t fieldCount = nodesDbc->getFieldCount();
        for (uint32_t i = 0; i < nodesDbc->getRecordCount(); i++) {
            TaxiNode node;
            node.id = nodesDbc->getUInt32(i, tnIdField);
            node.mapId = nodesDbc->getUInt32(i, tnMapField);
            node.x = nodesDbc->getFloat(i, tnXField);
            node.y = nodesDbc->getFloat(i, tnYField);
            node.z = nodesDbc->getFloat(i, tnZField);
            node.name = nodesDbc->getString(i, tnNameField);
            if (fieldCount > mountHordeField) {
                node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, mountAllianceField);
                node.mountDisplayIdHorde = nodesDbc->getUInt32(i, mountHordeField);
                if (node.mountDisplayIdAlliance == 0 && node.mountDisplayIdHorde == 0 && fieldCount > mountHordeFB) {
                    node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, mountAllianceFB);
                    node.mountDisplayIdHorde = nodesDbc->getUInt32(i, mountHordeFB);
                }
            }
            uint32_t nodeId = node.id;
            if (nodeId > 0) {
                taxiNodes_[nodeId] = std::move(node);
            }
            if (nodeId == 195) {
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

    auto pathDbc = am->loadDBC("TaxiPath.dbc");
    if (pathDbc && pathDbc->isLoaded()) {
        const auto* tpL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiPath") : nullptr;
        const uint32_t tpIdField   = tpL ? (*tpL)["ID"]       : 0;
        const uint32_t tpFromField = tpL ? (*tpL)["FromNode"] : 1;
        const uint32_t tpToField   = tpL ? (*tpL)["ToNode"]   : 2;
        const uint32_t tpCostField = tpL ? (*tpL)["Cost"]     : 3;
        for (uint32_t i = 0; i < pathDbc->getRecordCount(); i++) {
            TaxiPathEdge edge;
            edge.pathId = pathDbc->getUInt32(i, tpIdField);
            edge.fromNode = pathDbc->getUInt32(i, tpFromField);
            edge.toNode = pathDbc->getUInt32(i, tpToField);
            edge.cost = pathDbc->getUInt32(i, tpCostField);
            taxiPathEdges_.push_back(edge);
        }
        LOG_INFO("Loaded ", taxiPathEdges_.size(), " taxi path edges from TaxiPath.dbc");
    } else {
        LOG_WARNING("Could not load TaxiPath.dbc");
    }

    auto pathNodeDbc = am->loadDBC("TaxiPathNode.dbc");
    if (pathNodeDbc && pathNodeDbc->isLoaded()) {
        const auto* tpnL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiPathNode") : nullptr;
        const uint32_t tpnIdField    = tpnL ? (*tpnL)["ID"]        : 0;
        const uint32_t tpnPathField  = tpnL ? (*tpnL)["PathID"]    : 1;
        const uint32_t tpnIndexField = tpnL ? (*tpnL)["NodeIndex"] : 2;
        const uint32_t tpnMapField   = tpnL ? (*tpnL)["MapID"]     : 3;
        const uint32_t tpnXField     = tpnL ? (*tpnL)["X"]         : 4;
        const uint32_t tpnYField     = tpnL ? (*tpnL)["Y"]         : 5;
        const uint32_t tpnZField     = tpnL ? (*tpnL)["Z"]         : 6;
        for (uint32_t i = 0; i < pathNodeDbc->getRecordCount(); i++) {
            TaxiPathNode node;
            node.id = pathNodeDbc->getUInt32(i, tpnIdField);
            node.pathId = pathNodeDbc->getUInt32(i, tpnPathField);
            node.nodeIndex = pathNodeDbc->getUInt32(i, tpnIndexField);
            node.mapId = pathNodeDbc->getUInt32(i, tpnMapField);
            node.x = pathNodeDbc->getFloat(i, tpnXField);
            node.y = pathNodeDbc->getFloat(i, tpnYField);
            node.z = pathNodeDbc->getFloat(i, tpnZField);
            taxiPathNodes_[node.pathId].push_back(node);
        }
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

void MovementHandler::handleShowTaxiNodes(network::Packet& packet) {
    ShowTaxiNodesData data;
    if (!ShowTaxiNodesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_SHOWTAXINODES");
        return;
    }

    loadTaxiDbc();

    if (taxiMaskInitialized_) {
        for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
            uint32_t newBits = data.nodeMask[i] & ~knownTaxiMask_[i];
            if (newBits == 0) continue;
            for (uint32_t bit = 0; bit < 32; ++bit) {
                if (newBits & (1u << bit)) {
                    uint32_t nodeId = i * 32 + bit + 1;
                    auto it = taxiNodes_.find(nodeId);
                    if (it != taxiNodes_.end()) {
                        owner_.addSystemChatMessage("Discovered flight path: " + it->second.name);
                    }
                }
            }
        }
    }

    for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
        knownTaxiMask_[i] = data.nodeMask[i];
    }
    taxiMaskInitialized_ = true;

    currentTaxiData_ = data;
    taxiNpcGuid_ = data.npcGuid;
    taxiWindowOpen_ = true;
    owner_.closeGossip();
    buildTaxiCostMap();
    auto it = taxiNodes_.find(data.nearestNode);
    if (it != taxiNodes_.end()) {
        LOG_INFO("Taxi node ", data.nearestNode, " mounts: A=", it->second.mountDisplayIdAlliance,
                 " H=", it->second.mountDisplayIdHorde);
    }
    LOG_INFO("Taxi window opened, nearest node=", data.nearestNode);
}

void MovementHandler::applyTaxiMountForCurrentNode() {
    if (taxiMountActive_ || !owner_.mountCallbackRef()) return;
    auto it = taxiNodes_.find(currentTaxiData_.nearestNode);
    if (it == taxiNodes_.end()) {
        bool isAlliance = true;
        switch (owner_.playerRaceRef()) {
            case Race::ORC: case Race::UNDEAD: case Race::TAUREN: case Race::TROLL:
            case Race::GOBLIN: case Race::BLOOD_ELF:
                isAlliance = false; break;
            default: break;
        }
        uint32_t mountId = isAlliance ? 1210u : 1310u;
        taxiMountDisplayId_ = mountId;
        taxiMountActive_ = true;
        LOG_INFO("Taxi mount fallback (node ", currentTaxiData_.nearestNode, " not in DBC): displayId=", mountId);
        owner_.mountCallbackRef()(mountId);
        return;
    }

    bool isAlliance = true;
    switch (owner_.playerRaceRef()) {
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
    if (mountId == 541) mountId = 0;
    if (mountId == 0) {
        mountId = isAlliance ? it->second.mountDisplayIdHorde
                             : it->second.mountDisplayIdAlliance;
        if (mountId == 541) mountId = 0;
    }
    if (mountId == 0) {
        uint32_t gryphonId = owner_.services().gryphonDisplayId;
        uint32_t wyvernId = owner_.services().wyvernDisplayId;
        if (isAlliance && gryphonId != 0) mountId = gryphonId;
        if (!isAlliance && wyvernId != 0) mountId = wyvernId;
        if (mountId == 0) {
            mountId = (isAlliance ? wyvernId : gryphonId);
        }
    }
    if (mountId == 0) {
        if (it->second.mountDisplayIdAlliance != 0) mountId = it->second.mountDisplayIdAlliance;
        else if (it->second.mountDisplayIdHorde != 0) mountId = it->second.mountDisplayIdHorde;
    }
    if (mountId == 0) {
        static constexpr uint32_t kAllianceTaxiDisplays[] = {1210u, 1211u, 1212u, 1213u};
        static constexpr uint32_t kHordeTaxiDisplays[] = {1310u, 1311u, 1312u};
        mountId = isAlliance ? kAllianceTaxiDisplays[0] : kHordeTaxiDisplays[0];
    }
    if (mountId == 0) {
        mountId = isAlliance ? 30412u : 30413u;
    }
    if (mountId != 0) {
        taxiMountDisplayId_ = mountId;
        taxiMountActive_ = true;
        LOG_INFO("Taxi mount apply: displayId=", mountId);
        owner_.mountCallbackRef()(mountId);
    }
}

glm::vec3 MovementHandler::evalTaxiCatmullRom(const glm::vec3& p0, const glm::vec3& p1,
                                               const glm::vec3& p2, const glm::vec3& p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
    );
}

void MovementHandler::startClientTaxiPath(const std::vector<uint32_t>& pathNodes) {
    taxiClientPath_.clear();
    taxiClientIndex_ = 0;
    taxiClientActive_ = false;
    taxiClientSegmentProgress_ = 0.0f;

    for (size_t i = 0; i + 1 < pathNodes.size(); i++) {
        uint32_t fromNode = pathNodes[i];
        uint32_t toNode = pathNodes[i + 1];
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
        taxiClientPath_.clear();
        for (uint32_t nodeId : pathNodes) {
            auto nodeIt = taxiNodes_.find(nodeId);
            if (nodeIt == taxiNodes_.end()) continue;
            glm::vec3 serverPos(nodeIt->second.x, nodeIt->second.y, nodeIt->second.z);
            taxiClientPath_.push_back(core::coords::serverToCanonical(serverPos));
        }
    }

    if (taxiClientPath_.size() < 2) {
        LOG_WARNING("Taxi path too short: ", taxiClientPath_.size(), " waypoints");
        return;
    }

    // Precompute each segment's real curve arc length (see taxiClientSegmentArcLengths_'s
    // comment for why - pacing must use this, not the straight-line chord length, to keep
    // total flight duration in line with the server's own identical-speed spline). Sampled
    // numerically since the Catmull-Rom curve has no closed-form arc length; 16 sub-steps
    // is more than enough resolution for a taxi path segment's typical curvature.
    taxiClientSegmentArcLengths_.clear();
    taxiClientSegmentArcLengths_.reserve(taxiClientPath_.size() - 1);
    constexpr int kArcLengthSamples = 16;
    for (size_t i = 0; i + 1 < taxiClientPath_.size(); i++) {
        glm::vec3 p0 = (i > 0) ? taxiClientPath_[i - 1] : taxiClientPath_[i];
        glm::vec3 p1 = taxiClientPath_[i];
        glm::vec3 p2 = taxiClientPath_[i + 1];
        glm::vec3 p3 = (i + 2 < taxiClientPath_.size()) ? taxiClientPath_[i + 2] : taxiClientPath_[i + 1];

        float arcLength = 0.0f;
        glm::vec3 prev = p1;
        for (int s = 1; s <= kArcLengthSamples; s++) {
            float t = static_cast<float>(s) / static_cast<float>(kArcLengthSamples);
            glm::vec3 cur = evalTaxiCatmullRom(p0, p1, p2, p3, t);
            arcLength += glm::length(cur - prev);
            prev = cur;
        }
        taxiClientSegmentArcLengths_.push_back(arcLength);
    }

    // NOTE: this only builds taxiClientPath_ as pending data - it deliberately does
    // NOT snap the player to the path start or set taxiClientActive_. CMSG_ACTIVATETAXI
    // can still be rejected by the server at this point; see activateTaxi()'s comment
    // and beginTaxiFlightMotion(), which does the actual activation once
    // SMSG_ACTIVATETAXIREPLY confirms success.
}

void MovementHandler::beginTaxiFlightMotion() {
    if (taxiClientPath_.size() < 2) return;

    glm::vec3 start = taxiClientPath_[0];
    glm::vec3 dir(0.0f);
    float dirLenSq = 0.0f;
    for (size_t i = 1; i < taxiClientPath_.size(); i++) {
        dir = taxiClientPath_[i] - start;
        dirLenSq = glm::dot(dir, dir);
        if (dirLenSq >= 1e-6f) {
            break;
        }
    }

    float initialOrientation = movementInfo.orientation;
    float initialRenderYaw = movementInfo.orientation;
    float initialPitch = 0.0f;
    float initialRoll = 0.0f;
    if (dirLenSq >= 1e-6f) {
        initialOrientation = std::atan2(dir.y, dir.x);
        glm::vec3 renderDir = core::coords::canonicalToRender(dir);
        initialRenderYaw = std::atan2(renderDir.y, renderDir.x);
        glm::vec3 dirNorm = dir * glm::inversesqrt(dirLenSq);
        initialPitch = std::asin(std::clamp(dirNorm.z, -1.0f, 1.0f));
    }

    movementInfo.x = start.x;
    movementInfo.y = start.y;
    movementInfo.z = start.z;
    movementInfo.orientation = initialOrientation;
    sanitizeMovementForTaxi();

    auto playerEntity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid());
    if (playerEntity) {
        playerEntity->setPosition(start.x, start.y, start.z, initialOrientation);
    }

    if (owner_.taxiOrientationCallbackRef()) {
        owner_.taxiOrientationCallbackRef()(initialRenderYaw, initialPitch, initialRoll);
    }

    LOG_INFO("Taxi flight started with ", taxiClientPath_.size(), " spline waypoints");
    taxiServerCompletionPending_ = false;
    taxiClientActive_ = true;
}

// Ends the active client-simulated taxi flight. snapToFinalWaypoint controls
// whether the player is teleported to the known landing position first:
// - true: natural completion (the client spline reached its own end) - our
//   own path data is authoritative for where "arrived" means.
// - false: an authoritative server signal (SMSG_DISMOUNT with
//   UNIT_FLAG_TAXI_FLIGHT already cleared, or the equivalent
//   UNIT_FIELD_MOUNTDISPLAYID update) arrived ahead of our own spline
//   finishing.
//
//   The server's stop point may not match our path's precomputed final
//   waypoint - e.g. a genuinely interrupted/mid-route stop - so this does
//   NOT unconditionally snap there. But per AzerothCore's own
//   FlightPathMovementGenerator::DoFinalize() (confirmed by reading its
//   source): "update z position to ground ... this prevent cheating with
//   landing point at lags when client side flight end early in comparison
//   server side" - the server itself anticipates exactly this scenario and
//   corrects the player's own authoritative position to the true
//   destination on the terminal leg. So: if we're within a generous sanity
//   distance of our own known final waypoint (kAuthoritativeLandingSnapDist),
//   treat this as that same normal "client finished slightly early" case
//   and snap there - safety net for whatever spline/server pacing drift
//   remains despite arc-length-parameterizing taxiClientPath_'s pacing (see
//   taxiClientSegmentArcLengths_). Beyond the sanity distance, stay
//   conservative and leave the player where the spline currently has them,
//   per the original caution.
//
// The "known final waypoint" itself is taxiNodes_[taxiDestNodeId_]'s own
// registered position (TaxiNodes.dbc), NOT taxiClientPath_.back() (the last
// entry of the separate TaxiPathNode.dbc waypoint trace) - live-confirmed
// the two can disagree by 100+ yards, since a taxi path's waypoint trace
// doesn't necessarily terminate exactly on the destination node's actual
// coordinate. TaxiNodes.dbc's own position is what GM teleports/.gps
// validated as accurate throughout this whole effort, so it's the correct
// "true destination" to trust here. Falls back to taxiClientPath_.back()
// only if the destination node can't be resolved for some reason.
void MovementHandler::finishClientTaxiFlight(bool snapToFinalWaypoint) {
    glm::vec3 landingPos(0.0f);
    bool haveLandingPos = false;
    auto destNodeIt = taxiNodes_.find(taxiDestNodeId_);
    if (destNodeIt != taxiNodes_.end()) {
        landingPos = core::coords::serverToCanonical(
            glm::vec3(destNodeIt->second.x, destNodeIt->second.y, destNodeIt->second.z));
        haveLandingPos = true;
    } else if (!taxiClientPath_.empty()) {
        landingPos = taxiClientPath_.back();
        haveLandingPos = true;
    }

    if (!snapToFinalWaypoint && haveLandingPos) {
        constexpr float kAuthoritativeLandingSnapDist = 300.0f;
        glm::vec3 gap = landingPos - glm::vec3(movementInfo.x, movementInfo.y, movementInfo.z);
        if (glm::length(gap) <= kAuthoritativeLandingSnapDist) {
            LOG_INFO("Taxi landing (authoritative early completion): within sanity "
                     "distance of known final waypoint, snapping there instead of "
                     "current spline position (gap=", glm::length(gap), ")");
            snapToFinalWaypoint = true;
        }
    }
    if (snapToFinalWaypoint && haveLandingPos) {
        auto playerEntity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid());
        if (playerEntity) {
            playerEntity->setPosition(landingPos.x, landingPos.y, landingPos.z,
                                      movementInfo.orientation);
        }
        movementInfo.x = landingPos.x;
        movementInfo.y = landingPos.y;
        movementInfo.z = landingPos.z;
        // Application's own per-frame render-position sync only runs while onTaxi
        // is true (see its "Sync character render position" block) - by the time
        // this function returns, onTaxiFlight_/taxiMountActive_ are already false,
        // so that sync won't pick up this correction. Push it to the renderer
        // directly instead - see taxiLandingPositionCallbackRef()'s comment.
        if (owner_.taxiLandingPositionCallbackRef()) {
            owner_.taxiLandingPositionCallbackRef()(landingPos.x, landingPos.y, landingPos.z);
        }
        LOG_INFO("Taxi landing: snapped to final waypoint (",
                 landingPos.x, ", ", landingPos.y, ", ", landingPos.z, ")");
    }
    taxiClientActive_ = false;
    onTaxiFlight_ = false;
    taxiLandingCooldown_ = 2.0f;
    if (taxiMountActive_ && owner_.mountCallbackRef()) {
        owner_.mountCallbackRef()(0);
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;
    owner_.currentMountDisplayIdRef() = 0;
    // Some WotLK servers expose the taxi mount through vehicle data.
    // Clear that cached ID at landing so the dismount control does not
    // remain visible after the flight has completed.
    owner_.vehicleIdRef() = 0;
    taxiClientPath_.clear();
    taxiDestNodeId_ = 0;
    taxiServerCompletionPending_ = false;
    taxiRecoverPending_ = false;
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    if (owner_.getSocket()) {
        sendMovement(Opcode::MSG_MOVE_STOP);
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }
    LOG_INFO("Taxi flight landed (client path)");
}

void MovementHandler::deferServerTaxiCompletion() {
    if (!taxiClientActive_) return;
    taxiServerCompletionPending_ = true;
}

bool MovementHandler::isNearTaxiDestination(float maxDistance) const {
    if (!taxiClientActive_ || taxiClientPath_.empty()) return false;

    const glm::vec3& destination = taxiClientPath_.back();
    const float dx = movementInfo.x - destination.x;
    const float dy = movementInfo.y - destination.y;
    const float dz = movementInfo.z - destination.z;
    return dx * dx + dy * dy + dz * dz <= maxDistance * maxDistance;
}

void MovementHandler::updateClientTaxi(float deltaTime) {
    if (dismountGraceRemaining_ > 0.0f) {
        dismountGraceRemaining_ -= deltaTime;
    }

    // Live-confirmed: a CMSG_ACTIVATETAXI the server can't make sense of (e.g.
    // a stale start node vs. the player's actual position) can go completely
    // unanswered - no SMSG_ACTIVATETAXIREPLY at all, success or failure. Before
    // activation was deferred to the reply (see activateTaxi()'s comment), the
    // speculative flight start accidentally "recovered" from this since it
    // never waited on a reply in the first place. Now that it correctly does,
    // a dropped reply left taxiActivatePending_ true forever - every future
    // activateTaxi() call silently no-ops on its pending-guard, permanently
    // soft-locking the player's taxi system until relog. Time out and clear
    // pending state the same way an explicit failure reply would.
    if (taxiActivatePending_) {
        taxiActivateTimer_ += deltaTime;
        if (taxiActivateTimer_ > kTaxiActivateReplyTimeoutSeconds) {
            LOG_WARNING("Taxi activation reply timed out after ",
                        kTaxiActivateReplyTimeoutSeconds, "s - clearing pending state");
            taxiActivatePending_ = false;
            taxiActivateTimer_ = 0.0f;
            taxiClientPath_.clear();
            taxiClientIndex_ = 0;
            taxiClientSegmentProgress_ = 0.0f;
            if (taxiMountActive_ && owner_.mountCallbackRef()) {
                owner_.mountCallbackRef()(0);
            }
            taxiMountActive_ = false;
            taxiMountDisplayId_ = 0;
            onTaxiFlight_ = false;
        }
    }

    if (!taxiClientActive_ || taxiClientPath_.size() < 2) return;
    auto playerEntity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid());

    if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
        finishClientTaxiFlight(/*snapToFinalWaypoint=*/true);
        return;
    }

    float remainingDistance = taxiClientSegmentProgress_ + (taxiClientSpeed_ * deltaTime);
    glm::vec3 start(0.0f);
    glm::vec3 end(0.0f);
    glm::vec3 dir(0.0f);
    float segmentLen = 0.0f;
    float t = 0.0f;

    while (true) {
        if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
            finishClientTaxiFlight(/*snapToFinalWaypoint=*/true);
            return;
        }

        start = taxiClientPath_[taxiClientIndex_];
        end = taxiClientPath_[taxiClientIndex_ + 1];
        dir = end - start;
        float segLenSq = glm::dot(dir, dir);

        if (segLenSq < 1e-4f) {
            taxiClientIndex_++;
            continue;
        }
        // Pace by the curve's real arc length, not the straight-line chord -
        // see taxiClientSegmentArcLengths_'s comment. Fall back to the chord
        // length only if the precomputed array is somehow missing/stale.
        segmentLen = (taxiClientIndex_ < taxiClientSegmentArcLengths_.size())
            ? taxiClientSegmentArcLengths_[taxiClientIndex_]
            : std::sqrt(segLenSq);

        if (remainingDistance >= segmentLen) {
            remainingDistance -= segmentLen;
            taxiClientIndex_++;
            taxiClientSegmentProgress_ = 0.0f;
            continue;
        }

        taxiClientSegmentProgress_ = remainingDistance;
        t = taxiClientSegmentProgress_ / segmentLen;
        break;
    }

    glm::vec3 p0 = (taxiClientIndex_ > 0) ? taxiClientPath_[taxiClientIndex_ - 1] : start;
    glm::vec3 p1 = start;
    glm::vec3 p2 = end;
    glm::vec3 p3 = (taxiClientIndex_ + 2 < taxiClientPath_.size()) ?
                   taxiClientPath_[taxiClientIndex_ + 2] : end;

    glm::vec3 nextPos = evalTaxiCatmullRom(p0, p1, p2, p3, t);

    float t2 = t * t;
    glm::vec3 tangent = 0.5f * (
        (-p0 + p2) +
        2.0f * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t +
        3.0f * (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t2
    );
    float tangentLenSq = glm::dot(tangent, tangent);
    if (tangentLenSq < 1e-8f) {
        tangent = dir;
        tangentLenSq = glm::dot(tangent, tangent);
        if (tangentLenSq < 1e-8f) {
            tangent = glm::vec3(std::cos(movementInfo.orientation), std::sin(movementInfo.orientation), 0.0f);
            tangentLenSq = 1.0f; // unit vector
        }
    }

    float targetOrientation = std::atan2(tangent.y, tangent.x);

    glm::vec3 tangentNorm = tangent * glm::inversesqrt(std::max(tangentLenSq, 1e-8f));
    float pitch = std::asin(std::clamp(tangentNorm.z, -1.0f, 1.0f));

    float currentOrientation = movementInfo.orientation;
    float orientDiff = targetOrientation - currentOrientation;
    while (orientDiff > 3.14159265f) orientDiff -= 6.28318530f;
    while (orientDiff < -3.14159265f) orientDiff += 6.28318530f;
    float roll = -orientDiff * 2.5f;
    roll = std::clamp(roll, -0.7f, 0.7f);

    float smoothOrientation = currentOrientation + orientDiff * std::min(1.0f, deltaTime * 3.0f);

    if (playerEntity) {
        playerEntity->setPosition(nextPos.x, nextPos.y, nextPos.z, smoothOrientation);
    }
    movementInfo.x = nextPos.x;
    movementInfo.y = nextPos.y;
    movementInfo.z = nextPos.z;
    movementInfo.orientation = smoothOrientation;

    // A server completion packet can arrive much earlier than the local DBC
    // spline endpoint (observed on CMaNGOS), and it is commonly sent only once.
    // Do not land at the packet's early position, but also do not discard it:
    // consume it inside a small landing radius so the mount comes off promptly
    // rather than lingering through the tail of a mismatched local path.
    if (taxiServerCompletionPending_ && isNearTaxiDestination()) {
        finishClientTaxiFlight(/*snapToFinalWaypoint=*/true);
        return;
    }

    if (owner_.taxiOrientationCallbackRef()) {
        glm::vec3 renderTangent = core::coords::canonicalToRender(tangent);
        float renderYaw = std::atan2(renderTangent.y, renderTangent.x);
        owner_.taxiOrientationCallbackRef()(renderYaw, pitch, roll);
    }
}

void MovementHandler::handleActivateTaxiReply(network::Packet& packet) {
    ActivateTaxiReplyData data;
    if (!ActivateTaxiReplyParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACTIVATETAXIREPLY");
        return;
    }

    // Whether a reply arrived at all is the first thing worth knowing when a
    // flight does not happen, and it was only ever said at info level.
    LOG_WARNING("SMSG_ACTIVATETAXIREPLY: result=", data.result,
                " pending=", taxiActivatePending_);

    if (!taxiActivatePending_) {
        LOG_DEBUG("Ignoring stray taxi reply: result=", data.result);
        return;
    }

    if (data.result == 0) {
        if (onTaxiFlight_ && !taxiActivatePending_) {
            return;
        }
        onTaxiFlight_ = true;
        taxiStartGrace_ = std::max(taxiStartGrace_, 2.0f);
        sanitizeMovementForTaxi();
        taxiWindowOpen_ = false;
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        applyTaxiMountForCurrentNode();
        // Now that the server has confirmed, actually move the player to the path
        // start and start advancing the spline (see activateTaxi()'s comment).
        beginTaxiFlightMotion();
        if (owner_.getSocket()) {
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
        }
        LOG_INFO("Taxi flight started!");
    } else {
        if (onTaxiFlight_ || taxiClientActive_) {
            LOG_WARNING("Ignoring stale taxi failure reply while flight is active: result=", data.result);
            taxiActivatePending_ = false;
            taxiActivateTimer_ = 0.0f;
            return;
        }
        LOG_WARNING("Taxi activation failed, result=", data.result);
        owner_.addSystemChatMessage("Cannot take that flight path.");
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        // Clear the pending path data built speculatively by activateTaxi() -
        // it was never committed (taxiClientActive_ never went true), but drop it
        // explicitly so a retry can't somehow pick up stale waypoints.
        taxiClientPath_.clear();
        taxiClientIndex_ = 0;
        taxiClientSegmentProgress_ = 0.0f;
        taxiDestNodeId_ = 0;
        if (taxiMountActive_ && owner_.mountCallbackRef()) {
            owner_.mountCallbackRef()(0);
        }
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        onTaxiFlight_ = false;
    }
}

void MovementHandler::closeTaxi() {
    taxiWindowOpen_ = false;

    if (taxiActivatePending_ || onTaxiFlight_ || taxiClientActive_) {
        return;
    }

    if (taxiMountActive_ && owner_.mountCallbackRef()) {
        LOG_INFO("closeTaxi: clearing taxi mount");
        owner_.mountCallbackRef()(0);
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;

    taxiActivatePending_ = false;
    onTaxiFlight_ = false;

    taxiLandingCooldown_ = 2.0f;
}

void MovementHandler::buildTaxiCostMap() {
    taxiCostMap_.clear();
    taxiPrevMap_.clear();
    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0) return;

    struct AdjEntry { uint32_t node; uint32_t cost; };
    std::unordered_map<uint32_t, std::vector<AdjEntry>> adj;
    for (const auto& edge : taxiPathEdges_) {
        adj[edge.fromNode].push_back({edge.toNode, edge.cost});
    }

    std::deque<uint32_t> queue;
    queue.push_back(startNode);
    taxiCostMap_[startNode] = 0;

    while (!queue.empty()) {
        uint32_t cur = queue.front();
        queue.pop_front();
        for (const auto& next : adj[cur]) {
            // Flights only route through nodes the player has discovered —
            // the server rejects paths crossing unknown nodes, so a cost map
            // built over them would show routes/prices the server won't honor.
            if (!currentTaxiData_.isNodeKnown(next.node)) continue;
            if (taxiCostMap_.find(next.node) == taxiCostMap_.end()) {
                taxiCostMap_[next.node] = taxiCostMap_[cur] + next.cost;
                taxiPrevMap_[next.node] = cur;
                queue.push_back(next.node);
            }
        }
    }
}

uint32_t MovementHandler::getTaxiCostTo(uint32_t destNodeId) const {
    auto it = taxiCostMap_.find(destNodeId);
    return (it != taxiCostMap_.end()) ? it->second : 0;
}

bool MovementHandler::hasTaxiRouteTo(uint32_t destNodeId) const {
    return destNodeId != currentTaxiData_.nearestNode &&
           taxiCostMap_.find(destNodeId) != taxiCostMap_.end();
}

std::vector<uint32_t> MovementHandler::getTaxiRouteTo(uint32_t destNodeId) const {
    std::vector<uint32_t> route;
    if (!hasTaxiRouteTo(destNodeId)) return route;

    uint32_t startNode = currentTaxiData_.nearestNode;
    uint32_t cur = destNodeId;
    while (cur != startNode) {
        route.push_back(cur);
        auto it = taxiPrevMap_.find(cur);
        if (it == taxiPrevMap_.end() || route.size() > taxiPrevMap_.size()) {
            return {};  // broken chain — should not happen with a consistent BFS
        }
        cur = it->second;
    }
    route.push_back(startNode);
    std::reverse(route.begin(), route.end());
    return route;
}

void MovementHandler::activateTaxi(uint32_t destNodeId) {
    if (!owner_.getSocket() || owner_.getState() != WorldState::IN_WORLD) return;

    if (taxiActivatePending_ || onTaxiFlight_) {
        return;
    }

    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0 || destNodeId == 0 || startNode == destNodeId) return;

    taxiDestNodeId_ = destNodeId;

    if (owner_.isMounted()) {
        LOG_INFO("Taxi activate: dismounting current mount");
        if (owner_.mountCallbackRef()) owner_.mountCallbackRef()(0);
        owner_.currentMountDisplayIdRef() = 0;
        dismount();
    }

    {
        auto destIt = taxiNodes_.find(destNodeId);
        if (destIt != taxiNodes_.end() && !destIt->second.name.empty()) {
            taxiDestName_ = destIt->second.name;
            owner_.addSystemChatMessage("Requesting flight to " + destIt->second.name + "...");
        } else {
            taxiDestName_.clear();
            owner_.addSystemChatMessage("Taxi: requesting flight...");
        }
    }

    // The route the cost map already worked out, rather than a second search of
    // the same graph.
    //
    // The one here searched every edge, including those reaching nodes the
    // player has never been to. The server refuses a flight the moment one
    // appears in the list — ERR_TAXINOTVISITED, checked node by node — so a
    // route through an undiscovered stop is turned down outright rather than
    // flown badly. buildTaxiCostMap already skips those, which is why the price
    // quoted came from a route this then declined to use.
    std::vector<uint32_t> path = getTaxiRouteTo(destNodeId);

    if (path.size() < 2) {
        LOG_WARNING("No taxi route through discovered nodes from ", startNode,
                    " to ", destNodeId);
        owner_.addSystemChatMessage("No flight path available to that destination.");
        return;
    }

    LOG_INFO("Taxi path: ", path.size(), " nodes, from ", startNode, " to ", destNodeId);

    // At warning level: the log carries nothing below it, and a flight that
    // goes wrong is otherwise reported with the request that caused it
    // invisible. A guid of zero here means no flight master was recorded, and
    // the server has nothing to answer.
    LOG_WARNING("Taxi activate: npc=0x", std::hex, taxiNpcGuid_, std::dec,
                " start=", startNode, " dest=", destNodeId,
                " pathLen=", path.size());
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

    // A route of more than two nodes has to go as ACTIVATETAXIEXPRESS with the
    // whole path. CMSG_ACTIVATETAXI carries only a source and a destination,
    // and the server answers it by looking for a single TaxiPath joining the
    // two — so anything needing an intermediate stop has no such path, gets no
    // reply, and times out. Stormwind to Westfall is one hop and worked;
    // everything further did not.
    if (path.size() > 2) {
        auto expressPkt =
            ActivateTaxiExpressPacket::build(taxiNpcGuid_, totalCost, path);
        owner_.getSocket()->send(expressPkt);
        LOG_WARNING("Taxi activate: sent EXPRESS for ", path.size(),
                    " nodes (a multi-hop route has no single path to ask for)");
    } else {
        auto basicPkt = ActivateTaxiPacket::build(taxiNpcGuid_, startNode, destNodeId);
        owner_.getSocket()->send(basicPkt);
    }

    taxiWindowOpen_ = false;
    taxiActivatePending_ = true;
    taxiActivateTimer_ = 0.0f;
    // Do NOT mount, set onTaxiFlight_, or activate the client spline here.
    // CMSG_ACTIVATETAXI can be rejected by the server (insufficient gold, invalid
    // route, distance/state restrictions). Starting the flight speculatively meant
    // a rejection reply became indistinguishable from a stale/duplicate one once
    // updateClientTaxi() started running unconditionally every frame - the client
    // would just keep flying an unauthorized route. Everything that actually starts
    // the flight (mount, onTaxiFlight_, taxiClientActive_) now happens only in the
    // success branch of handleActivateTaxiReply(), once the server confirms.
    // taxiClientPath_ is still built below as pending data - see
    // startClientTaxiPath()'s comment.
    if (owner_.getSocket()) {
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    if (owner_.taxiPrecacheCallbackRef()) {
        std::vector<glm::vec3> previewPath;
        for (size_t i = 0; i + 1 < path.size(); i++) {
            uint32_t fromNode = path[i];
            uint32_t toNode = path[i + 1];
            uint32_t pathId = 0;
            for (const auto& edge : taxiPathEdges_) {
                if (edge.fromNode == fromNode && edge.toNode == toNode) {
                    pathId = edge.pathId;
                    break;
                }
            }
            if (pathId == 0) continue;
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
            owner_.taxiPrecacheCallbackRef()(previewPath);
        }
    }

    if (owner_.taxiFlightStartCallbackRef()) {
        owner_.taxiFlightStartCallbackRef()();
    }
    startClientTaxiPath(path);
}

// ============================================================
// Area Trigger Detection (moved from GameHandler)
// ============================================================

void MovementHandler::loadAreaTriggerDbc() {
    if (owner_.areaTriggerDbcLoadedRef()) return;

    auto* am = owner_.services().assetManager;
    // Don't latch "loaded" until the asset manager is actually ready - the
    // first checkAreaTriggers() tick can land well before asset manager
    // init finishes (observed in headless mode), and latching here first
    // would permanently strand areaTriggersRef() empty with no retry and
    // no warning, silently disabling every area trigger for the process's
    // lifetime.
    if (!am || !am->isInitialized()) return;
    owner_.areaTriggerDbcLoadedRef() = true;

    auto dbc = am->loadDBC("AreaTrigger.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Failed to load AreaTrigger.dbc");
        return;
    }

    owner_.areaTriggersRef().reserve(dbc->getRecordCount());
    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        GameHandler::AreaTriggerEntry at;
        at.id     = dbc->getUInt32(i, 0);
        at.mapId  = dbc->getUInt32(i, 1);
        // DBC stores positions in server/wire format (X=west, Y=north) — swap to canonical
        at.x = dbc->getFloat(i, 3);  // canonical X (north) = DBC field 3 (Y_wire)
        at.y = dbc->getFloat(i, 2);  // canonical Y (west)  = DBC field 2 (X_wire)
        at.z = dbc->getFloat(i, 4);
        at.radius    = dbc->getFloat(i, 5);
        at.boxLength = dbc->getFloat(i, 6);
        at.boxWidth  = dbc->getFloat(i, 7);
        at.boxHeight = dbc->getFloat(i, 8);
        at.boxYaw    = dbc->getFloat(i, 9);
        owner_.areaTriggersRef().push_back(at);
    }

    LOG_DEBUG("Loaded ", owner_.areaTriggersRef().size(), " area triggers from AreaTrigger.dbc");
}

void MovementHandler::checkAreaTriggers() {
    if (!owner_.isInWorld()) return;
    if (onTaxiFlight_ || taxiClientActive_) return;

    loadAreaTriggerDbc();
    if (owner_.areaTriggersRef().empty()) return;

    const float px = movementInfo.x;
    const float py = movementInfo.y;
    const float pz = movementInfo.z;

    // Sanity: if position is near map origin on Eastern Kingdoms (map 0),
    // something has corrupted movementInfo — skip area trigger check to
    // avoid firing Alterac/Hillsbrad triggers and causing a rogue teleport.
    if (owner_.getCurrentMapId() == 0 &&
        std::abs(px) < 1000.0f && std::abs(py) < 1000.0f) {
        if (!restoreWorldTransferFallbackIfNearOrigin("checkAreaTriggers")) {
            LOG_WARNING("checkAreaTriggers: position near map origin (", px, ", ", py, ", ", pz,
                        ") on map 0 — skipping to avoid rogue teleport. onTransport=",
                        owner_.isOnTransport(), " transportGuid=0x", std::hex,
                        owner_.playerTransportGuidRef(), std::dec);
            return;
        }
    }

    const float checkedPx = movementInfo.x;
    const float checkedPy = movementInfo.y;
    const float checkedPz = movementInfo.z;

    // Swept-path samples: also test points between the previous check position
    // and the current one, so fast (mounted) movement cannot step across a
    // small portal box between 0.25s polls — the Deeprun entrance trigger only
    // fired intermittently when ridden into at mount speed.
    glm::vec3 sweepSamples[10];
    int sweepCount = 0;
    {
        const glm::vec3 cur(checkedPx, checkedPy, checkedPz);
        const bool sameMap = lastAreaTriggerCheckValid_ &&
                             lastAreaTriggerCheckMapId_ == owner_.currentMapIdRef();
        if (sameMap) {
            const glm::vec3 d = cur - lastAreaTriggerCheckPos_;
            const float movedSq = d.x * d.x + d.y * d.y + d.z * d.z;
            // Skip sweep on teleport-sized jumps; only bridge normal movement.
            if (movedSq > 1.0f && movedSq < 50.0f * 50.0f) {
                const float moved = std::sqrt(movedSq);
                const int steps = std::min(8, static_cast<int>(moved));
                for (int s = 1; s <= steps; ++s) {
                    const float t = static_cast<float>(s) / static_cast<float>(steps + 1);
                    sweepSamples[sweepCount++] = lastAreaTriggerCheckPos_ + d * t;
                }
            }
        }
        sweepSamples[sweepCount++] = cur;
        lastAreaTriggerCheckPos_ = cur;
        lastAreaTriggerCheckMapId_ = owner_.currentMapIdRef();
        lastAreaTriggerCheckValid_ = true;
    }

    // Time-based cooldown after teleport/world entry — suppress ALL trigger
    // firing (not just the first check) to prevent re-entry from immediately
    // sending us to a wrong destination.
    const bool cooldownActive = owner_.areaTriggerCooldownRef() > 0.0f;

    // On first check after map transfer, just mark which triggers we're inside
    // without firing them — prevents exit portal from immediately sending us back
    bool suppressFirst = owner_.areaTriggerSuppressFirstRef();
    if (suppressFirst) {
        owner_.areaTriggerSuppressFirstRef() = false;
    }

    for (const auto& at : owner_.areaTriggersRef()) {
        if (at.mapId != owner_.currentMapIdRef()) continue;

        auto insideTrigger = [&at](float qx, float qy, float qz) -> bool {
            if (at.radius > 0.0f) {
                // Sphere trigger — use actual DBC radius
                float dx = qx - at.x;
                float dy = qy - at.y;
                float dz = qz - at.z;
                float distSq = dx * dx + dy * dy + dz * dz;
                return distSq <= at.radius * at.radius;
            }
            if (at.boxLength > 0.0f || at.boxWidth > 0.0f || at.boxHeight > 0.0f) {
                // Box trigger. AreaTrigger.dbc stores box axes in server-space
                // X/Y. The trigger center is cached in canonical space, so swap
                // deltas back to server-space before applying length/width/yaw.
                float serverDx = qy - at.y;
                float serverDy = qx - at.x;
                float dz = qz - at.z;

                // Rotate into box-local space
                float cosYaw = std::cos(-at.boxYaw);
                float sinYaw = std::sin(-at.boxYaw);
                float localX = serverDx * cosYaw - serverDy * sinYaw;
                float localY = serverDx * sinYaw + serverDy * cosYaw;

                return std::abs(localX) <= at.boxLength * 0.5f &&
                       std::abs(localY) <= at.boxWidth * 0.5f &&
                       std::abs(dz) <= at.boxHeight * 0.5f;
            }
            return false;
        };

        bool inside = false;
        for (int s = 0; s < sweepCount && !inside; ++s) {
            inside = insideTrigger(sweepSamples[s].x, sweepSamples[s].y, sweepSamples[s].z);
        }

        if (inside) {
            if (owner_.activeAreaTriggersRef().count(at.id) == 0) {
                const bool suppressCurrentTrigger =
                    suppressFirst || cooldownActive || postTransferReturnAreaTriggerId_ == at.id;
                if (suppressFirst) {
                    // If we spawn already inside a portal trigger, arm it as
                    // active so cooldown expiry cannot immediately bounce us
                    // back through the destination portal. The player must
                    // leave and re-enter before it can fire.
                    if (postTransferReturnAreaTriggerId_ == at.id) {
                        postTransferReturnAreaTriggerSawNear_ = true;
                    }
                    owner_.activeAreaTriggersRef().insert(at.id);
                    LOG_INFO("AreaTrigger armed on post-transfer spawn: AT", at.id,
                                " cooldown=", owner_.areaTriggerCooldownRef());
                } else if (postTransferReturnAreaTriggerId_ == at.id) {
                    // After a map transfer, the destination can place or
                    // nudge the player into the paired return trigger after
                    // the normal grace window has elapsed. Arm that trigger
                    // until the player leaves it once; a later re-entry can
                    // then intentionally fire it.
                    postTransferReturnAreaTriggerSawNear_ = true;
                    owner_.activeAreaTriggersRef().insert(at.id);
                    LOG_WARNING("AreaTrigger armed for post-transfer return portal: AT", at.id,
                                " cooldown=", owner_.areaTriggerCooldownRef());
                } else if (cooldownActive) {
                    // Do not mark ordinary cooldown-suppressed triggers active.
                    // If the player walks into a trigger during the cooldown,
                    // it should fire once the grace window ends.
                    LOG_DEBUG("AreaTrigger cooldown suppressed: AT", at.id,
                              " cooldown=", owner_.areaTriggerCooldownRef());
                } else {
                    owner_.activeAreaTriggersRef().insert(at.id);
                }

                if (suppressCurrentTrigger) {
                    // Suppressed above; if suppressFirst inserted the trigger,
                    // it will not fire until the player leaves/re-enters.
                } else {
                    network::Packet pkt(wireOpcode(Opcode::CMSG_AREATRIGGER));
                    pkt.writeUInt32(at.id);
                    lastAreaTriggerId_ = at.id;
                    if (const auto* dest = findKnownAreaTriggerDestinationByTrigger(at.id)) {
                        pendingAreaTriggerDestinationValid_ = true;
                        pendingAreaTriggerDestinationMapId_ = dest->mapId;
                        pendingAreaTriggerDestinationServerPos_ =
                            glm::vec3(dest->serverX, dest->serverY, dest->serverZ);
                        pendingAreaTriggerDestinationServerO_ = dest->serverO;
                    }
                    owner_.getSocket()->send(pkt);
                    const float dx = checkedPx - at.x;
                    const float dy = checkedPy - at.y;
                    LOG_WARNING("Fired CMSG_AREATRIGGER: id=", at.id,
                                " pos=(", checkedPx, ", ", checkedPy, ", ", checkedPz, ")",
                                " trigger=(", at.x, ", ", at.y, ", ", at.z, ")",
                                " dist2d=", std::sqrt(dx * dx + dy * dy));
                }
            }
        } else {
            if (postTransferReturnAreaTriggerId_ == at.id) {
                const float dx = checkedPx - at.x;
                const float dy = checkedPy - at.y;
                const float dz = checkedPz - at.z;
                const float distSq = dx * dx + dy * dy + dz * dz;
                const float triggerExtent = std::max({
                    at.radius,
                    at.boxLength * 0.5f,
                    at.boxWidth * 0.5f,
                    at.boxHeight * 0.5f,
                    12.0f
                });
                const float clearDistance = triggerExtent + 8.0f;
                if (distSq <= clearDistance * clearDistance) {
                    postTransferReturnAreaTriggerSawNear_ = true;
                    // A single outside sample near a destination portal is not
                    // enough to prove the player intentionally left it. Deeprun
                    // can report one outside tick and then drift back inside,
                    // which causes an immediate bounce. Keep the return portal
                    // armed until the player is comfortably clear.
                    owner_.activeAreaTriggersRef().insert(at.id);
                    LOG_DEBUG("AreaTrigger post-transfer return portal retained near trigger: AT", at.id,
                              " dist=", std::sqrt(distSq),
                              " clear=", clearDistance);
                    continue;
                }

                // Only while the post-transfer window is still open. The
                // distance test alone never expires: a player who has not been
                // near the portal and is far from it stays that way, so the
                // guard held for the rest of the session and that portal could
                // never be used again.
                //
                // Dying inside is exactly that shape. The release teleports the
                // player to a graveyard nowhere near the portal they came
                // through, without ever standing at it, so walking back as a
                // ghost was suppressed every time. The cooldown is ten seconds
                // and a destination position arrives in one, so anything still
                // far away after it has genuinely left.
                if (!postTransferReturnAreaTriggerSawNear_ && distSq > 1000.0f * 1000.0f &&
                    owner_.areaTriggerCooldownRef() > 0.0f) {
                    // During map transfer there can be a tick where the current
                    // map has switched but movementInfo still contains the old
                    // map's coordinates. That looks thousands of yards away
                    // from the destination trigger and must not clear the guard
                    // before the destination position arrives.
                    owner_.activeAreaTriggersRef().insert(at.id);
                    // Fires every frame for the brief window between a map switch and
                    // the destination position arriving - routine, not exceptional.
                    LOG_DEBUG("AreaTrigger post-transfer return portal waiting for destination position: AT",
                                at.id, " dist=", std::sqrt(distSq));
                    continue;
                }

                postTransferReturnAreaTriggerId_ = 0;
                postTransferReturnAreaTriggerSawNear_ = false;
                LOG_INFO("AreaTrigger post-transfer return portal cleared after leaving: AT", at.id,
                            " dist=", std::sqrt(distSq));
            }

            // Player left the trigger — allow re-fire on re-entry
            owner_.activeAreaTriggersRef().erase(at.id);
        }
    }
}

// ============================================================
// Transport Attachments (moved from GameHandler)
// ============================================================

void MovementHandler::setTransportAttachment(uint64_t childGuid, ObjectType type, uint64_t transportGuid,
                                             const glm::vec3& localOffset, bool hasLocalOrientation,
                                             float localOrientation,
                                             bool offsetNeedsTransportResolution) {
    if (childGuid == 0 || transportGuid == 0) {
        return;
    }

    GameHandler::TransportAttachment& attachment = owner_.transportAttachmentsRef()[childGuid];
    attachment.type = type;
    attachment.transportGuid = transportGuid;
    attachment.localOffset = localOffset;
    attachment.hasLocalOrientation = hasLocalOrientation;
    attachment.localOrientation = localOrientation;
    attachment.offsetNeedsTransportResolution = offsetNeedsTransportResolution;
}

void MovementHandler::clearTransportAttachment(uint64_t childGuid) {
    if (childGuid == 0) {
        return;
    }
    owner_.transportAttachmentsRef().erase(childGuid);
}

void MovementHandler::updateAttachedTransportChildren(float /*deltaTime*/) {
    if (!owner_.getTransportManager() || owner_.transportAttachmentsRef().empty()) {
        return;
    }

    constexpr float kPosEpsilonSq = 0.0001f;
    constexpr float kOriEpsilon = 0.001f;
    std::vector<uint64_t> stale;
    stale.reserve(8);

    for (auto& [childGuid, attachment] : owner_.transportAttachmentsRef()) {
        auto entity = owner_.getEntityManager().getEntity(childGuid);
        if (!entity) {
            stale.push_back(childGuid);
            continue;
        }

        ActiveTransport* transport = owner_.getTransportManager()->getTransport(attachment.transportGuid);
        if (!transport) {
            continue;
        }

        if (attachment.offsetNeedsTransportResolution) {
            attachment.localOffset = owner_.getTransportManager()->serverToTransportLocal(
                attachment.transportGuid, attachment.localOffset);
            attachment.offsetNeedsTransportResolution = false;
        }

        glm::vec3 composed = owner_.getTransportManager()->getPlayerWorldPosition(
            attachment.transportGuid, attachment.localOffset);

        float composedOrientation = entity->getOrientation();
        if (attachment.hasLocalOrientation) {
            // Client-owned ship routes do not retain hasServerYaw, but their
            // live quaternion still carries the tangent/dock rotation. Using
            // zero made attached NPCs face in a fixed world direction while
            // their deck turned beneath them.
            float baseYaw = 2.0f * std::atan2(transport->rotation.z, transport->rotation.w);
            composedOrientation = baseYaw + attachment.localOrientation;
        }

        glm::vec3 oldPos(entity->getX(), entity->getY(), entity->getZ());
        float oldOrientation = entity->getOrientation();
        glm::vec3 delta = composed - oldPos;
        const bool positionChanged = glm::dot(delta, delta) > kPosEpsilonSq;
        const bool orientationChanged = std::abs(composedOrientation - oldOrientation) > kOriEpsilon;
        if (!positionChanged && !orientationChanged) {
            continue;
        }

        entity->setPosition(composed.x, composed.y, composed.z, composedOrientation);

        if (attachment.type == ObjectType::UNIT) {
            if (owner_.creatureMoveCallbackRef()) {
                owner_.creatureMoveCallbackRef()(childGuid, composed.x, composed.y, composed.z, 0);
            }
        } else if (attachment.type == ObjectType::GAMEOBJECT) {
            if (owner_.gameObjectMoveCallbackRef()) {
                owner_.gameObjectMoveCallbackRef()(childGuid, composed.x, composed.y, composed.z, composedOrientation);
            }
        }
    }

    for (uint64_t guid : stale) {
        owner_.transportAttachmentsRef().erase(guid);
    }
}

// ============================================================
// Follow target (moved from GameHandler)
// ============================================================

void MovementHandler::followTarget() {
    if (owner_.getState() != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot follow: not in world");
        return;
    }

    if (owner_.getTargetGuid() == 0) {
        owner_.addSystemChatMessage("You must target someone to follow.");
        return;
    }

    auto target = owner_.getTarget();
    if (!target) {
        owner_.addSystemChatMessage("Invalid target.");
        return;
    }

    // Set follow target
    owner_.followTargetGuidRef() = owner_.getTargetGuid();

    // Initialize render-space position from entity's canonical coords
    owner_.followRenderPosRef() = core::coords::canonicalToRender(glm::vec3(target->getX(), target->getY(), target->getZ()));

    // Tell camera controller to start auto-following
    if (owner_.autoFollowCallbackRef()) {
        owner_.autoFollowCallbackRef()(&owner_.followRenderPosRef());
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

    owner_.addSystemChatMessage("Now following " + targetName + ".");
    LOG_INFO("Following target: ", targetName, " (GUID: 0x", std::hex, owner_.getTargetGuid(), std::dec, ")");
    owner_.fireAddonEvent("AUTOFOLLOW_BEGIN", {});
}

void MovementHandler::cancelFollow() {
    if (owner_.followTargetGuidRef() == 0) {
        return;
    }
    owner_.followTargetGuidRef() = 0;
    if (followMoveMoving_) {
        sendMovement(Opcode::MSG_MOVE_STOP);
        followMoveMoving_ = false;
    }
    if (owner_.autoFollowCallbackRef()) {
        owner_.autoFollowCallbackRef()(nullptr);
    }
    owner_.addSystemChatMessage("You stop following.");
    owner_.fireAddonEvent("AUTOFOLLOW_END", {});
}

// Same straight-line step used by the headless client's single-shot
// /movement/goto (updateMovementTask() in tools/headless_client/main.cpp),
// but the target position is read fresh from the live entity every call
// instead of a stored waypoint, and stops at kFollowStopDistance rather
// than snapping onto the target.
void MovementHandler::updateFollowMovement(float deltaTime) {
    const uint64_t guid = owner_.followTargetGuidRef();
    if (guid == 0) {
        return;
    }
    if (owner_.getState() != WorldState::IN_WORLD) {
        return;
    }
    if (isPlayerRooted() || !isServerMovementAllowed() || isOnTaxiFlight()) {
        if (followMoveMoving_) {
            sendMovement(Opcode::MSG_MOVE_STOP);
            followMoveMoving_ = false;
        }
        return;
    }

    auto target = owner_.getEntityManager().getEntity(guid);
    if (!target) {
        // cancelFollow() (called from GameHandler::update()'s
        // followRenderPos_ refresh) handles the disappeared-target case;
        // just stay quiet here rather than duplicate that check.
        return;
    }

    constexpr float kFollowStopDistance = 1.0f;
    const float dx = target->getX() - movementInfo.x;
    const float dy = target->getY() - movementInfo.y;
    const float dz = target->getZ() - movementInfo.z;
    const float horizontalDist = std::sqrt(dx * dx + dy * dy);
    const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance <= kFollowStopDistance) {
        if (followMoveMoving_) {
            sendMovement(Opcode::MSG_MOVE_STOP);
            followMoveMoving_ = false;
        }
        return;
    }

    if (horizontalDist > 0.001f) {
        setOrientation(std::atan2(-dy, dx));
        sendMovement(Opcode::MSG_MOVE_SET_FACING);
    }
    if (!followMoveMoving_) {
        sendMovement(Opcode::MSG_MOVE_START_FORWARD);
        followMoveMoving_ = true;
    }

    const float speed = std::max(0.1f, getServerRunSpeed());
    const float step = std::min(distance - kFollowStopDistance, speed * std::max(0.0f, deltaTime));
    const float t = distance > 0.001f ? (step / distance) : 0.0f;
    setPosition(movementInfo.x + dx * t, movementInfo.y + dy * t, movementInfo.z + dz * t);
    sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
}

} // namespace game
} // namespace wowee
