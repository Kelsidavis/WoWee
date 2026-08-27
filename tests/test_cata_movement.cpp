// Cataclysm's movement payloads, over the 109 layouts derived from a 4.3.4 core.
//
// The layouts are data, in Data/expansions/cata/movement_sequences.json, written
// by tools/derive_cata_movement.py from MovementStructures.cpp. What is tested
// here is the reader that executes them: that it walks a layout in order, gates
// each value on the right presence bit, and gets the inverted ones the right way
// round.
//
// What this cannot test is whether the layouts match the wire. Only traffic from
// a running realm settles that, and that is the next step rather than this one.
// A writer built beside the reader agrees with it by construction; it catches an
// ordering or gating fault, not a wrong table.
#include <catch_amalgamated.hpp>

#include "game/cata_movement.hpp"

#include <filesystem>
#include <string>

using wowee::game::CataMovementExtra;
using wowee::game::CataMovementSequences;
using wowee::game::MovementElement;
using wowee::game::MovementInfo;
using wowee::game::movementElementFromName;
using wowee::game::readCataMovement;
using wowee::network::Packet;

namespace {

const char* kSequencesPath = "Data/expansions/cata/movement_sequences.json";

/// The values a payload is built from and checked against.
struct Sample {
    uint64_t guid = 0xF130000012345678ull;
    uint32_t flags = 0x0000000Fu;
    uint16_t flags2 = 0x0ABCu;
    uint32_t time = 0x11223344u;
    float x = 1.5f, y = -2.25f, z = 3.75f, o = 0.5f;
    float pitch = -0.25f;
    uint32_t fallTime = 0x55667788u;
    float jumpVelocity = -9.5f;
    bool present = true;   ///< the optional scalars are there
    bool transport = false;
    bool fall = false;
};

/// Lay a payload out by the same sequence the reader walks.
///
/// Mirrors the core's write side: the presence bits for flags, flags2,
/// timestamp, orientation, pitch and spline elevation are written inverted,
/// and the transport, fall and vehicle bits are not.
Packet build(const std::vector<MovementElement>& sequence, const Sample& s) {
    Packet p(0);
    static constexpr uint8_t kOne[] = {0};
    (void)kOne;
    for (MovementElement e : sequence) {
        switch (e) {
            case MovementElement::HasGuidByte0: case MovementElement::HasGuidByte1:
            case MovementElement::HasGuidByte2: case MovementElement::HasGuidByte3:
            case MovementElement::HasGuidByte4: case MovementElement::HasGuidByte5:
            case MovementElement::HasGuidByte6: case MovementElement::HasGuidByte7: {
                const int i = static_cast<int>(e) - static_cast<int>(MovementElement::HasGuidByte0);
                p.writeBit(((s.guid >> (i * 8)) & 0xFFu) != 0);
                break;
            }
            case MovementElement::GuidByte0: case MovementElement::GuidByte1:
            case MovementElement::GuidByte2: case MovementElement::GuidByte3:
            case MovementElement::GuidByte4: case MovementElement::GuidByte5:
            case MovementElement::GuidByte6: case MovementElement::GuidByte7: {
                const int i = static_cast<int>(e) - static_cast<int>(MovementElement::GuidByte0);
                const uint8_t b = static_cast<uint8_t>((s.guid >> (i * 8)) & 0xFFu);
                if (b != 0) p.writeUInt8(static_cast<uint8_t>(b ^ 1u));
                break;
            }
            // Inverted on the wire.
            case MovementElement::HasMovementFlags:   p.writeBit(!s.present); break;
            case MovementElement::HasMovementFlags2:  p.writeBit(!s.present); break;
            case MovementElement::HasTimestamp:       p.writeBit(!s.present); break;
            case MovementElement::HasOrientation:     p.writeBit(!s.present); break;
            case MovementElement::HasPitch:           p.writeBit(!s.present); break;
            case MovementElement::HasSplineElevation: p.writeBit(true);  break;  // absent
            // Not inverted.
            case MovementElement::HasTransportData:   p.writeBit(s.transport); break;
            case MovementElement::HasFallData:        p.writeBit(s.fall); break;
            case MovementElement::HasTransportTime2:  if (s.transport) p.writeBit(false); break;
            case MovementElement::HasVehicleId:       if (s.transport) p.writeBit(false); break;
            case MovementElement::HasFallDirection:   if (s.fall) p.writeBit(false); break;
            case MovementElement::HasSpline:
            case MovementElement::HasHeightChangeFailed:
            case MovementElement::ZeroBit:
            case MovementElement::OneBit:             p.writeBit(false); break;

            case MovementElement::HasTransportGuidByte0: case MovementElement::HasTransportGuidByte1:
            case MovementElement::HasTransportGuidByte2: case MovementElement::HasTransportGuidByte3:
            case MovementElement::HasTransportGuidByte4: case MovementElement::HasTransportGuidByte5:
            case MovementElement::HasTransportGuidByte6: case MovementElement::HasTransportGuidByte7:
                if (s.transport) p.writeBit(false);
                break;
            case MovementElement::TransportGuidByte0: case MovementElement::TransportGuidByte1:
            case MovementElement::TransportGuidByte2: case MovementElement::TransportGuidByte3:
            case MovementElement::TransportGuidByte4: case MovementElement::TransportGuidByte5:
            case MovementElement::TransportGuidByte6: case MovementElement::TransportGuidByte7:
                break;  // all mask bits above were zero

            case MovementElement::MovementFlags:  if (s.present) p.writeBits(s.flags, 30); break;
            case MovementElement::MovementFlags2: if (s.present) p.writeBits(s.flags2, 12); break;
            case MovementElement::Timestamp:      if (s.present) p.writeUInt32(s.time); break;
            case MovementElement::PositionX:      p.writeFloat(s.x); break;
            case MovementElement::PositionY:      p.writeFloat(s.y); break;
            case MovementElement::PositionZ:      p.writeFloat(s.z); break;
            case MovementElement::Orientation:    if (s.present) p.writeFloat(s.o); break;
            case MovementElement::Pitch:          if (s.present) p.writeFloat(s.pitch); break;
            case MovementElement::FallTime:       if (s.fall) p.writeUInt32(s.fallTime); break;
            case MovementElement::FallVerticalSpeed: if (s.fall) p.writeFloat(s.jumpVelocity); break;
            case MovementElement::Counter:        p.writeUInt32(0x99AABBCCu); break;
            case MovementElement::FlushBits:      p.flushBits(); break;
            default: break;  // gated off by a bit written false above
        }
    }
    p.flushBits();
    return p;
}

}  // namespace

TEST_CASE("element names come off the core's spelling", "[catamove]") {
    CHECK(movementElementFromName("MSEPositionX") == MovementElement::PositionX);
    CHECK(movementElementFromName("MSEHasGuidByte5") == MovementElement::HasGuidByte5);
    CHECK(movementElementFromName("MSEFlushBits") == MovementElement::FlushBits);
    CHECK(movementElementFromName("MSENotAThing") == MovementElement::Unknown);
}

TEST_CASE("every derived layout loads and is understood", "[catamove]") {
    if (!std::filesystem::exists(kSequencesPath)) {
        SUCCEED("no derived layouts in this checkout");
        return;
    }
    CataMovementSequences sequences;
    REQUIRE(sequences.loadFromJson(kSequencesPath));
    CHECK(sequences.size() == 109);
    // An element the core has and this build does not would be read as a hole
    // in the middle of a layout, so it is named rather than skipped.
    CHECK(sequences.unknownElements().empty());
}

TEST_CASE("a payload round-trips through every layout", "[catamove]") {
    if (!std::filesystem::exists(kSequencesPath)) {
        SUCCEED("no derived layouts in this checkout");
        return;
    }
    CataMovementSequences sequences;
    REQUIRE(sequences.loadFromJson(kSequencesPath));

    const char* names[] = {"MSG_MOVE_START_FORWARD", "MSG_MOVE_HEARTBEAT",
                           "MSG_MOVE_JUMP", "MSG_MOVE_SET_FACING",
                           "MSG_MOVE_STOP"};
    Sample sample;
    for (const char* name : names) {
        const auto* sequence = sequences.find(name);
        REQUIRE(sequence != nullptr);

        Packet packet = build(*sequence, sample);
        MovementInfo info;
        CataMovementExtra extra;
        INFO("layout " << name);
        REQUIRE(readCataMovement(packet, *sequence, info, extra));

        CHECK(extra.guid == sample.guid);
        CHECK(info.x == sample.x);
        CHECK(info.y == sample.y);
        CHECK(info.z == sample.z);
        CHECK(info.flags == sample.flags);
        CHECK(info.flags2 == sample.flags2);
        CHECK(info.time == sample.time);
    }
}

TEST_CASE("an absent field leaves its value alone", "[catamove]") {
    if (!std::filesystem::exists(kSequencesPath)) {
        SUCCEED("no derived layouts in this checkout");
        return;
    }
    CataMovementSequences sequences;
    REQUIRE(sequences.loadFromJson(kSequencesPath));
    const auto* sequence = sequences.find("MSG_MOVE_HEARTBEAT");
    REQUIRE(sequence != nullptr);

    // present=false sets every inverted bit, which on this wire means "absent".
    // Reading them as present would take four bytes that are not there and
    // drag every field after them out of step.
    Sample sample;
    sample.present = false;
    Packet packet = build(*sequence, sample);
    MovementInfo info;
    CataMovementExtra extra;
    REQUIRE(readCataMovement(packet, *sequence, info, extra));
    CHECK(info.flags == 0);
    CHECK(info.time == 0);
    CHECK(info.x == sample.x);   // position is unconditional
    CHECK(extra.guid == sample.guid);
}

TEST_CASE("a truncated payload fails rather than half filling a position",
          "[catamove]") {
    if (!std::filesystem::exists(kSequencesPath)) {
        SUCCEED("no derived layouts in this checkout");
        return;
    }
    CataMovementSequences sequences;
    REQUIRE(sequences.loadFromJson(kSequencesPath));
    const auto* sequence = sequences.find("MSG_MOVE_START_FORWARD");
    REQUIRE(sequence != nullptr);

    Packet full = build(*sequence, Sample{});
    std::vector<uint8_t> bytes = full.getData();
    REQUIRE(bytes.size() > 8);
    bytes.resize(bytes.size() / 2);

    Packet truncated(0, std::move(bytes));
    MovementInfo info;
    CataMovementExtra extra;
    CHECK_FALSE(readCataMovement(truncated, *sequence, info, extra));
}
