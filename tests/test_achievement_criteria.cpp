// The criteria progress record, whose counter is a packed guid.
//
// It was read as a fixed uint64. A counter of five is two bytes on the wire and
// not eight, so the read swallowed the player guid behind it and every field
// after was out of place. In SMSG_ALL_ACHIEVEMENT_DATA, where the records
// repeat until an id of -1, the first misread took the whole list: the sentinel
// was never where the reader looked, so it kept reading noise as ids until the
// packet ran out.
//
// Nothing about that fails loudly. The panel drew a number for every criteria
// and the numbers were wrong, which is only visible to someone who knows what
// the right one is. So the layout is pinned here against packets built the way
// AzerothCore builds them — appendPackGUID and all.

#include <catch_amalgamated.hpp>

#include <utility>
#include <vector>

#include "game/achievement_criteria.hpp"
#include "network/packet.hpp"

using wowee::game::CriteriaProgressRecord;
using wowee::game::readCriteriaProgressTail;
using wowee::network::Packet;

namespace {

/// One record as AchievementMgr writes it: id, packed counter, packed player
/// guid, flags, packed date, elapsed, elapsed again.
void appendRecord(Packet& p, uint32_t id, uint64_t counter, uint64_t playerGuid,
                  uint32_t flags, uint32_t date, uint32_t elapsed) {
    p.writeUInt32(id);
    p.writePackedGuid(counter);
    p.writePackedGuid(playerGuid);
    p.writeUInt32(flags);
    p.writeUInt32(date);
    p.writeUInt32(elapsed);
    p.writeUInt32(elapsed);
}

Packet forReading(Packet& built) {
    return Packet(built.getOpcode(), built.getData());
}

}  // namespace

TEST_CASE("a criteria record reads back the counter it was written with",
          "[achievements]") {
    Packet built(0);
    appendRecord(built, 12345, 7, 0xF130000000000042ULL, 0, 0x1A2B3C4D, 90);
    Packet p = forReading(built);

    REQUIRE(p.readUInt32() == 12345u);
    CriteriaProgressRecord rec;
    REQUIRE(readCriteriaProgressTail(p, rec));
    CHECK(rec.counter == 7u);
    CHECK(rec.playerGuid == 0xF130000000000042ULL);
    CHECK(rec.flags == 0u);
    CHECK(rec.date == 0x1A2B3C4Du);
    CHECK(rec.timeElapsed == 90u);
    // The whole record and nothing more.
    CHECK(p.getRemainingSize() == 0u);
}

TEST_CASE("a small counter is two bytes, not eight", "[achievements]") {
    // This is the case a fixed uint64 read gets wrong. Five packs to a mask
    // byte and one value byte; reading eight would take six bytes of the guid
    // with it and answer a counter in the billions.
    Packet built(0);
    appendRecord(built, 1, 5, 0xF130000000000001ULL, 0, 111, 222);
    Packet p = forReading(built);
    p.readUInt32();
    CriteriaProgressRecord rec;
    REQUIRE(readCriteriaProgressTail(p, rec));
    CHECK(rec.counter == 5u);
}

TEST_CASE("a zero counter is a single mask byte", "[achievements]") {
    Packet built(0);
    appendRecord(built, 1, 0, 0xF130000000000001ULL, 0, 111, 222);
    Packet p = forReading(built);
    p.readUInt32();
    CriteriaProgressRecord rec;
    REQUIRE(readCriteriaProgressTail(p, rec));
    CHECK(rec.counter == 0u);
    CHECK(rec.date == 111u);
}

TEST_CASE("a list of records walks to its sentinel", "[achievements]") {
    // The failure the fixed read produced was never one wrong number: it was
    // the loop losing its place and reading the rest of the packet as records.
    // Three records of different counter widths, then -1.
    Packet built(0);
    appendRecord(built, 100, 3, 0xF130000000000001ULL, 0, 10, 1);
    appendRecord(built, 200, 0x1234, 0xF130000000000001ULL, 0, 20, 2);
    appendRecord(built, 300, 0xFFFFFFFFFFULL, 0xF130000000000001ULL, 0, 30, 3);
    built.writeUInt32(0xFFFFFFFF);
    Packet p = forReading(built);

    std::vector<std::pair<uint32_t, uint64_t>> got;
    while (p.hasRemaining(4)) {
        uint32_t id = p.readUInt32();
        if (id == 0xFFFFFFFF) break;
        CriteriaProgressRecord rec;
        if (!readCriteriaProgressTail(p, rec)) break;
        got.emplace_back(id, rec.counter);
    }

    REQUIRE(got.size() == 3u);
    CHECK(got[0] == std::make_pair(100u, uint64_t{3}));
    CHECK(got[1] == std::make_pair(200u, uint64_t{0x1234}));
    CHECK(got[2] == std::make_pair(300u, uint64_t{0xFFFFFFFFFF}));
    CHECK(p.getRemainingSize() == 0u);
}

TEST_CASE("a truncated record is refused rather than half-read",
          "[achievements]") {
    // A criteria id read out of the tail of a short packet is indistinguishable
    // from a real one, so a record that does not fit must not become an entry.
    Packet built(0);
    appendRecord(built, 100, 3, 0xF130000000000001ULL, 0, 10, 1);
    auto bytes = built.getData();
    bytes.resize(bytes.size() - 5);
    Packet p(0, bytes);

    REQUIRE(p.readUInt32() == 100u);
    CriteriaProgressRecord rec;
    CHECK_FALSE(readCriteriaProgressTail(p, rec));
}
