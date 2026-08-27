#pragma once

// Cataclysm's movement wire, which is a different shape from every expansion
// before it.
//
// 4.3.4 marshals movement as a bit stream interleaved with the byte stream, and
// chooses the order of both per opcode with no pattern between them:
// MSG_MOVE_START_FORWARD writes position Y, Z, X and then guid mask bits 5, 2,
// 0, while MSG_MOVE_HEARTBEAT writes Z, X, Y and then pitch, timestamp and fall
// bits. There are 109 such orders. None can be derived from another, so they
// are read off a 4.3.4 core into Data/expansions/cata/movement_sequences.json
// by tools/derive_cata_movement.py and executed here, rather than written out
// by hand 109 times.
//
// This mirrors how the core itself does it, and for the same reason.

#include "game/world_packets.hpp"
#include "network/packet.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace game {

/// One field of a movement payload, named as the core names it.
///
/// The order is the enum's order in the core, but nothing depends on the
/// numeric values: the sequences arrive as strings and are looked up by name.
enum class MovementElement : uint8_t {
    Unknown = 0,

    // Presence bits. Five of these are inverted on the wire, which is the
    // detail that cannot be guessed: a set bit means the field is *absent*
    // for movement flags, flags2, timestamp, orientation, pitch and spline
    // elevation, and means present for transport, fall and vehicle.
    HasGuidByte0, HasGuidByte1, HasGuidByte2, HasGuidByte3,
    HasGuidByte4, HasGuidByte5, HasGuidByte6, HasGuidByte7,
    HasMovementFlags, HasMovementFlags2, HasTimestamp, HasOrientation,
    HasTransportData,
    HasTransportGuidByte0, HasTransportGuidByte1, HasTransportGuidByte2,
    HasTransportGuidByte3, HasTransportGuidByte4, HasTransportGuidByte5,
    HasTransportGuidByte6, HasTransportGuidByte7,
    HasTransportTime2, HasVehicleId, HasPitch, HasFallData, HasFallDirection,
    HasSplineElevation, HasSpline, HasHeightChangeFailed,

    // Values.
    GuidByte0, GuidByte1, GuidByte2, GuidByte3,
    GuidByte4, GuidByte5, GuidByte6, GuidByte7,
    MovementFlags, MovementFlags2, Timestamp,
    PositionX, PositionY, PositionZ, Orientation,
    TransportGuidByte0, TransportGuidByte1, TransportGuidByte2, TransportGuidByte3,
    TransportGuidByte4, TransportGuidByte5, TransportGuidByte6, TransportGuidByte7,
    TransportPositionX, TransportPositionY, TransportPositionZ, TransportOrientation,
    TransportSeat, TransportTime, TransportTime2, TransportVehicleId,
    Pitch, FallTime, FallVerticalSpeed, FallCosAngle, FallSinAngle,
    FallHorizontalSpeed, SplineElevation, Counter,

    // Structural.
    ZeroBit,       ///< a bit whose value is not read
    OneBit,        ///< likewise; the two differ only on the write side
    FlushBits,     ///< the writer padded here, so the reader closes its bit byte
    ExtraElement,  ///< opcode-specific payload this reader does not decode
};

/// The name the core gives an element, e.g. "MSEPositionX". Unknown for a name
/// this build does not implement, which is not an error on its own: a sequence
/// carrying one is refused rather than half read.
[[nodiscard]] MovementElement movementElementFromName(const std::string& name);

/// What a movement payload carried, beyond what MovementInfo has room for.
struct CataMovementExtra {
    uint64_t guid = 0;          ///< whose movement this is; 4.x carries it inline
    uint32_t counter = 0;
    float splineElevation = 0.0f;
    bool hasSpline = false;
    bool sawExtraElement = false;  ///< the payload had a field this reader skipped
};

/// Read one movement payload laid out by `sequence`.
///
/// False means the packet ran out part way, in which case `out` holds whatever
/// had been read and must not be used: a short movement packet that half fills
/// a position is worse than one that fails, because the position is plausible.
[[nodiscard]] bool readCataMovement(network::Packet& packet,
                                    const std::vector<MovementElement>& sequence,
                                    MovementInfo& out,
                                    CataMovementExtra& extra);

/// The per-opcode layouts, keyed by the core's opcode name.
class CataMovementSequences {
public:
    /// Load from movement_sequences.json as the derive script writes it.
    bool loadFromJson(const std::string& path);

    /// Null when the opcode carries no movement payload.
    [[nodiscard]] const std::vector<MovementElement>* find(const std::string& opcodeName) const;

    [[nodiscard]] size_t size() const { return sequences_.size(); }

    /// Names the file used that this build has no element for. Empty is the
    /// expected state; anything here means the reader is behind the core.
    [[nodiscard]] const std::vector<std::string>& unknownElements() const {
        return unknownElements_;
    }

private:
    std::unordered_map<std::string, std::vector<MovementElement>> sequences_;
    std::vector<std::string> unknownElements_;
};

}  // namespace game
}  // namespace wowee
