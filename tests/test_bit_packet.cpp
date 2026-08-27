// Bit-level packet access, which is the primitive Cataclysm's wire needs.
//
// 4.x marshals a packet as a bit stream interleaved with the byte stream in one
// buffer, and a GUID as a mask bit per byte followed by the non-zero bytes
// XOR'd with 1, each pass in an order that belongs to the opcode. None of the
// three expansions this client speaks today reads a single bit, so every case
// here is about the primitive rather than about any packet: the orders are
// parameters, and the real per-opcode tables have to come off a running 4.3.4
// core rather than out of this file. See docs/plan-cataclysm.md.
#include <catch_amalgamated.hpp>

#include "network/packet.hpp"

#include <cstdint>

using wowee::network::Packet;

TEST_CASE("bits go in most significant first", "[bitpacket]") {
    Packet p(0);
    p.writeBit(true);
    p.writeBit(false);
    p.writeBit(true);
    p.flushBits();

    // 101 in the top three bits, the rest padding.
    REQUIRE(p.getSize() == 1);
    CHECK(p.getData()[0] == 0xA0);
}

TEST_CASE("a bit round-trips through the same packet", "[bitpacket]") {
    const bool pattern[] = {true, true, false, true, false, false, false, true,
                            false, true};
    Packet p(0);
    for (bool bit : pattern) p.writeBit(bit);
    p.flushBits();

    for (bool bit : pattern) CHECK(p.readBit() == bit);
}

TEST_CASE("writeBits and readBits agree over every width", "[bitpacket]") {
    for (int count = 1; count <= 64; ++count) {
        const uint64_t mask = (count == 64) ? ~uint64_t{0}
                                            : ((uint64_t{1} << count) - 1);
        const uint64_t value = uint64_t{0x0123456789ABCDEF} & mask;
        Packet p(0);
        p.writeBits(value, count);
        p.flushBits();
        CHECK(p.readBits(count) == value);
    }
}

TEST_CASE("a partial bit byte is padded when the stream goes back to bytes",
          "[bitpacket]") {
    Packet p(0);
    p.writeBit(true);
    p.writeUInt32(0xDEADBEEF);

    // The single bit occupies a byte of its own; the uint32 follows it whole.
    REQUIRE(p.getSize() == 5);
    CHECK(p.getData()[0] == 0x80);

    CHECK(p.readBit() == true);
    CHECK(p.readUInt32() == 0xDEADBEEF);
}

TEST_CASE("bits, bytes and bits again keep their places", "[bitpacket]") {
    Packet p(0);
    p.writeBit(true);
    p.writeBit(false);
    p.writeUInt16(0x1234);
    p.writeBit(true);
    p.writeBit(true);
    p.writeBit(false);
    p.writeUInt8(0x77);

    CHECK(p.readBit() == true);
    CHECK(p.readBit() == false);
    CHECK(p.readUInt16() == 0x1234);
    CHECK(p.readBit() == true);
    CHECK(p.readBit() == true);
    CHECK(p.readBit() == false);
    CHECK(p.readUInt8() == 0x77);
}

TEST_CASE("flushBits does nothing with no bits pending", "[bitpacket]") {
    Packet p(0);
    p.writeUInt8(0x11);
    p.flushBits();
    p.flushBits();
    CHECK(p.getSize() == 1);
}

TEST_CASE("a guid round-trips through mask and byte passes", "[bitpacket]") {
    // Two different orders on purpose: the wire uses two, and a helper that
    // quietly assumed one would pass a same-order test and nothing else.
    static constexpr uint8_t kMaskOrder[]  = {4, 0, 7, 1, 2, 3, 6, 5};
    static constexpr uint8_t kBytesOrder[] = {2, 6, 5, 0, 7, 3, 1, 4};

    const uint64_t guid = 0xF130000000ABCDEFull;

    Packet p(0);
    p.writeGuidMask(guid, kMaskOrder);
    p.writeGuidBytes(guid, kBytesOrder);

    Packet::GuidBytes decoded{};
    p.readGuidMask(decoded, kMaskOrder);
    p.readGuidBytes(decoded, kBytesOrder);
    CHECK(Packet::guidFromBytes(decoded) == guid);
}

TEST_CASE("a guid byte travels xor'd with one", "[bitpacket]") {
    static constexpr uint8_t kOrder[] = {0};

    Packet p(0);
    p.writeGuidMask(uint64_t{0x41}, kOrder);
    p.writeGuidBytes(uint64_t{0x41}, kOrder);

    // The mask byte, then 0x41 ^ 1. A reader that skipped the XOR would land
    // one off and never notice, because the value stays a plausible byte.
    REQUIRE(p.getSize() == 2);
    CHECK(p.getData()[0] == 0x80);
    CHECK(p.getData()[1] == 0x40);
}

TEST_CASE("a zero byte is absent from the stream, not sent as zero",
          "[bitpacket]") {
    static constexpr uint8_t kOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};

    Packet p(0);
    p.writeGuidMask(uint64_t{0x0000000000FF00FFull}, kOrder);
    p.writeGuidBytes(uint64_t{0x0000000000FF00FFull}, kOrder);

    // One mask byte plus the two non-zero bytes.
    CHECK(p.getSize() == 3);

    Packet::GuidBytes decoded{};
    p.readGuidMask(decoded, kOrder);
    p.readGuidBytes(decoded, kOrder);
    CHECK(Packet::guidFromBytes(decoded) == 0x0000000000FF00FFull);
}

TEST_CASE("a guid of zero occupies its mask and nothing else", "[bitpacket]") {
    static constexpr uint8_t kOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};

    Packet p(0);
    p.writeGuidMask(uint64_t{0}, kOrder);
    p.writeGuidBytes(uint64_t{0}, kOrder);
    CHECK(p.getSize() == 1);

    Packet::GuidBytes decoded{};
    p.readGuidMask(decoded, kOrder);
    p.readGuidBytes(decoded, kOrder);
    CHECK(Packet::guidFromBytes(decoded) == 0);
}

TEST_CASE("two guids interleave their passes", "[bitpacket]") {
    // The shape that broke a hand-written reader every time it was tried: both
    // masks go out before either set of bytes, so a reader that finishes one
    // guid before starting the next reads the second one's bytes as the first's.
    static constexpr uint8_t kOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};
    const uint64_t first  = 0x0000000012345678ull;
    const uint64_t second = 0xF13000000000ABCDull;

    Packet p(0);
    p.writeGuidMask(first, kOrder);
    p.writeGuidMask(second, kOrder);
    p.writeGuidBytes(first, kOrder);
    p.writeGuidBytes(second, kOrder);

    Packet::GuidBytes a{};
    Packet::GuidBytes b{};
    p.readGuidMask(a, kOrder);
    p.readGuidMask(b, kOrder);
    p.readGuidBytes(a, kOrder);
    p.readGuidBytes(b, kOrder);
    CHECK(Packet::guidFromBytes(a) == first);
    CHECK(Packet::guidFromBytes(b) == second);
}

TEST_CASE("reading bits off the end answers false rather than running on",
          "[bitpacket]") {
    Packet p(0);
    p.writeBit(true);
    p.flushBits();

    CHECK(p.readBit() == true);
    for (int i = 0; i < 7; ++i) CHECK(p.readBit() == false);  // padding
    for (int i = 0; i < 8; ++i) CHECK(p.readBit() == false);  // past the end
    CHECK(p.getRemainingSize() == 0);
}

TEST_CASE("the byte paths are unchanged when no bit is ever written",
          "[bitpacket]") {
    // The guard on the pre-Cata expansions: every byte writer now flushes and
    // every byte reader resets, and neither may alter a packet that holds no
    // bits at all.
    Packet p(0);
    p.writeUInt8(0x01);
    p.writeUInt16(0x0302);
    p.writeUInt32(0x07060504);
    p.writeUInt64(0x0F0E0D0C0B0A0908ull);
    p.writeFloat(1.5f);
    p.writeString("ab");
    p.writePackedGuid(0x0000000000FF00FFull);

    const uint8_t expectedHead[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
                                    0x0F};
    REQUIRE(p.getSize() >= sizeof(expectedHead));
    for (size_t i = 0; i < sizeof(expectedHead); ++i)
        CHECK(p.getData()[i] == expectedHead[i]);

    CHECK(p.readUInt8() == 0x01);
    CHECK(p.readUInt16() == 0x0302);
    CHECK(p.readUInt32() == 0x07060504);
    CHECK(p.readUInt64() == 0x0F0E0D0C0B0A0908ull);
    CHECK(p.readFloat() == 1.5f);
    CHECK(p.readString() == "ab");
    CHECK(p.readPackedGuid() == 0x0000000000FF00FFull);
}
