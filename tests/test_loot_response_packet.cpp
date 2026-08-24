// SMSG_LOOT_RESPONSE, and the section that only exists on some realms.
//
// The quest-item list at the end is WotLK's. Whether the parser looks for it is
// decided by the expansion this client is configured for, not by the realm - and
// those two can disagree. A 3.3.5 profile pointed at a 1.12 realm reads the byte
// after the regular items as a quest count when it is nothing of the kind, and a
// bogus count is a list that cannot parse.
//
// That used to lose the whole response. The regular items had already been read
// correctly and were thrown away with it, so the loot window never opened and
// the only trace was one warning about a section the realm never sent - which
// reads as "the realm sent nothing" rather than "we dropped what it sent".
#include <catch_amalgamated.hpp>
#include "game/world_packets.hpp"
#include "core/application.hpp"

namespace wowee {
namespace core {
Application* Application::instance = nullptr;
}
}

using namespace wowee::game;

namespace {

struct Bytes {
    std::vector<uint8_t> b;
    void u8(uint8_t v) { b.push_back(v); }
    void u32(uint32_t v) {
        b.push_back(uint8_t(v)); b.push_back(uint8_t(v >> 8));
        b.push_back(uint8_t(v >> 16)); b.push_back(uint8_t(v >> 24));
    }
    void u64(uint64_t v) { u32(uint32_t(v)); u32(uint32_t(v >> 32)); }
    /// One loot item, 22 bytes, the same on every expansion.
    void item(uint8_t slot, uint32_t itemId, uint32_t count) {
        u8(slot); u32(itemId); u32(count);
        u32(0);      // displayInfo
        u32(0);      // randomSuffix
        u32(0);      // randomPropertyId
        u8(0);       // lootSlotType
    }
};

wowee::network::Packet packetOf(const std::vector<uint8_t>& body) {
    return wowee::network::Packet(0x160, body);  // SMSG_LOOT_RESPONSE
}

/// The body a realm sends for one chest holding one stack, with no quest
/// section - which is every realm older than 3.3.5.
std::vector<uint8_t> chestWithoutQuestSection(uint64_t guid) {
    Bytes p;
    p.u64(guid);
    p.u8(3);            // lootType
    p.u32(120);         // gold
    p.u8(1);            // one regular item
    p.item(0, 1673, 2);
    return p.b;
}

}  // namespace

TEST_CASE("a realm with no quest section still yields its loot", "[loot]") {
    // Parsed as WotLK, which is the mismatch: there is no quest count byte to
    // read and whatever follows is not one.
    auto body = chestWithoutQuestSection(0xF110000689000699ull);
    body.push_back(0x7F);  // a stray byte read as a count of 127

    auto packet = packetOf(body);
    LootResponseData data;
    REQUIRE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/true));
    CHECK(data.lootGuid == 0xF110000689000699ull);
    CHECK(data.gold == 120);
    REQUIRE(data.items.size() == 1);
    CHECK(data.items[0].itemId == 1673);
    CHECK(data.items[0].count == 2);
    CHECK_FALSE(data.items[0].isQuestItem);
}

TEST_CASE("the same body parsed as its own expansion", "[loot]") {
    const auto body = chestWithoutQuestSection(0x42ull);
    auto packet = packetOf(body);
    LootResponseData data;
    REQUIRE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/false));
    CHECK(data.gold == 120);
    REQUIRE(data.items.size() == 1);
    CHECK(data.items[0].itemId == 1673);
}

TEST_CASE("a real quest section is read", "[loot]") {
    Bytes p;
    p.u64(0x99ull);
    p.u8(3);
    p.u32(5);
    p.u8(1);
    p.item(0, 111, 1);
    p.u8(1);              // one quest item
    p.item(1, 222, 3);

    auto packet = packetOf(p.b);
    LootResponseData data;
    REQUIRE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/true));
    REQUIRE(data.items.size() == 2);
    CHECK(data.items[0].itemId == 111);
    CHECK_FALSE(data.items[0].isQuestItem);
    CHECK(data.items[1].itemId == 222);
    CHECK(data.items[1].isQuestItem);
}

TEST_CASE("a quest count larger than the bytes behind it keeps the rest", "[loot]") {
    // A truncated genuine section, rather than a mismatched expansion. The
    // regular items still stand: losing the quest items is the smaller loss.
    Bytes p;
    p.u64(0x7ull);
    p.u8(3);
    p.u32(0);
    p.u8(1);
    p.item(0, 555, 1);
    p.u8(4);              // claims four, sends one
    p.item(1, 666, 1);

    auto packet = packetOf(p.b);
    LootResponseData data;
    REQUIRE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/true));
    REQUIRE(data.items.size() == 1);
    CHECK(data.items[0].itemId == 555);
}

TEST_CASE("a refusal is not loot", "[loot]") {
    // guid + lootType and nothing else: the realm answered the use and offered
    // nothing. The caller must not open a window on it.
    Bytes p;
    p.u64(0x1234ull);
    p.u8(0);

    auto packet = packetOf(p.b);
    LootResponseData data;
    CHECK_FALSE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/true));
}

TEST_CASE("a truncated regular list is refused outright", "[loot]") {
    // Nothing trustworthy was read, so there is nothing to keep.
    Bytes p;
    p.u64(0x1ull);
    p.u8(3);
    p.u32(0);
    p.u8(2);              // claims two
    p.item(0, 777, 1);    // sends one

    auto packet = packetOf(p.b);
    LootResponseData data;
    CHECK_FALSE(LootResponseParser::parse(packet, data, /*isWotlkFormat=*/false));
}
