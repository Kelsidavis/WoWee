// SMSG_GUILD_BANK_LIST, whose tab-name block is optional.
//
// Guild::SendBankList writes the tab names under one flag - "with tab info" -
// and the items after them. The reader gated the names on that flag *and* on
// the list describing tab zero, which is Cataclysm's rule
// (GuildBankQueryResults::Write tests `!Tab && FullUpdate`) and not 3.3.5's. On
// a server that answers a tab query with the names attached, the block is on
// the wire and was skipped: the item count came out of the tab count and the
// items out of the names, so the panel drew nothing.
//
// The three shapes are pinned here because nothing about a mis-parse raises -
// the bytes are all valid, they are just read a block early.
#include <catch_amalgamated.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "core/application.hpp"
#include "game/world_packets.hpp"

namespace wowee {
namespace core {
Application* Application::instance = nullptr;
}
}

using namespace wowee::game;

namespace {

struct Writer {
    std::vector<uint8_t> bytes;
    void u8(uint8_t v) { bytes.push_back(v); }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<uint8_t>(v >> (i * 8)));
    }
    void u64(uint64_t v) {
        for (int i = 0; i < 8; ++i) bytes.push_back(static_cast<uint8_t>(v >> (i * 8)));
    }
    void cstr(const std::string& s) {
        bytes.insert(bytes.end(), s.begin(), s.end());
        bytes.push_back(0);
    }
    /// One occupied slot, in the order the reader expects it.
    void item(uint8_t slotId, uint32_t entry, uint32_t stack) {
        u8(slotId);
        u32(entry);
        u32(0);        // flags
        u32(0);        // randomPropertyId - zero, so no seed follows
        u32(stack);
        u32(0);        // permanent enchantment
        u8(0);         // charges
        u8(0);         // sockets
    }
};

/// money, tabId, remaining withdrawals, then the optional tab block.
Writer header(uint8_t tabId, bool withTabInfo,
              const std::vector<std::string>& tabNames) {
    Writer w;
    w.u64(123456);
    w.u8(tabId);
    w.u32(7);                    // withdrawals remaining
    w.u8(withTabInfo ? 1 : 0);
    if (withTabInfo) {
        w.u8(static_cast<uint8_t>(tabNames.size()));
        for (const std::string& name : tabNames) {
            w.cstr(name);
            w.cstr("Interface\\Icons\\INV_Misc_Bag_08");
        }
    }
    return w;
}

GuildBankData parse(Writer& w) {
    wowee::network::Packet packet(0, std::move(w.bytes));
    GuildBankData data;
    REQUIRE(GuildBankListParser::parse(packet, data));
    return data;
}

}  // namespace

TEST_CASE("the opening list carries the tabs and the first tab's items") {
    Writer w = header(0, true, {"Tab One", "Tab Two"});
    w.u8(2);
    w.item(0, 6948, 1);
    w.item(3, 4306, 20);

    const GuildBankData data = parse(w);
    CHECK(data.money == 123456u);
    CHECK(data.tabId == 0);
    CHECK(data.withdrawAmount == 7);
    CHECK(data.tabsIncluded);
    REQUIRE(data.tabs.size() == 2);
    CHECK(data.tabs[0].tabName == "Tab One");
    CHECK(data.tabs[1].tabName == "Tab Two");
    REQUIRE(data.tabItems.size() == 2);
    CHECK(data.tabItems[0].slotId == 0);
    CHECK(data.tabItems[0].itemEntry == 6948u);
    CHECK(data.tabItems[1].slotId == 3);
    CHECK(data.tabItems[1].itemEntry == 4306u);
    CHECK(data.tabItems[1].stackCount == 20u);
}

TEST_CASE("a later tab may carry the names too") {
    // The regression. Reading this as though the names were absent takes the
    // tab count for the item count and the first name for a slot.
    Writer w = header(2, true, {"Tab One", "Tab Two", "Tab Three"});
    w.u8(1);
    w.item(5, 2589, 12);

    const GuildBankData data = parse(w);
    CHECK(data.tabId == 2);
    CHECK(data.tabsIncluded);
    REQUIRE(data.tabs.size() == 3);
    CHECK(data.tabs[2].tabName == "Tab Three");
    REQUIRE(data.tabItems.size() == 1);
    CHECK(data.tabItems[0].slotId == 5);
    CHECK(data.tabItems[0].itemEntry == 2589u);
    CHECK(data.tabItems[0].stackCount == 12u);
}

TEST_CASE("a slots-only update leaves the tabs it found") {
    Writer first = header(0, true, {"Tab One", "Tab Two"});
    first.u8(0);
    wowee::network::Packet firstPacket(0, std::move(first.bytes));
    GuildBankData data;
    REQUIRE(GuildBankListParser::parse(firstPacket, data));
    REQUIRE(data.tabs.size() == 2);

    Writer second = header(1, false, {});
    second.u8(1);
    second.item(9, 1234, 3);
    wowee::network::Packet secondPacket(0, std::move(second.bytes));
    REQUIRE(GuildBankListParser::parse(secondPacket, data));

    CHECK_FALSE(data.tabsIncluded);
    CHECK(data.tabs.size() == 2);          // kept, not cleared
    CHECK(data.tabId == 1);
    REQUIRE(data.tabItems.size() == 1);
    CHECK(data.tabItems[0].itemEntry == 1234u);
}

TEST_CASE("an empty slot carries nothing after its entry") {
    Writer w = header(0, true, {"Only Tab"});
    w.u8(2);
    w.u8(4);
    w.u32(0);          // empty: no flags, no stack, no enchant
    w.item(6, 858, 5);

    const GuildBankData data = parse(w);
    REQUIRE(data.tabItems.size() == 2);
    CHECK(data.tabItems[0].itemEntry == 0u);
    CHECK(data.tabItems[1].slotId == 6);
    CHECK(data.tabItems[1].itemEntry == 858u);
    CHECK(data.tabItems[1].stackCount == 5u);
}
