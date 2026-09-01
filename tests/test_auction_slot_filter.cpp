// Which equipment slot the auction house searches for.
//
// The filter tree's third tier - Armor, Cloth, Head - is built by
// GetAuctionInvTypes, and FrameXML numbers the rows it gets back from one.
// So the number that comes back through QueryAuctionItems is a position in
// the subset offered for that class, not a position in the whole slot table.
//
// Reading the whole table at the raw position is the failure this is here to
// stop: picking One-Hand under Weapons would send the id for Head, the server
// would answer with helmets, and nothing anywhere would say the filter had
// been misread. A wrong search returns results, which is what makes it quiet.
#include <catch_amalgamated.hpp>

#include "game/auction_filters.hpp"

#include <set>

using namespace wowee::game;

TEST_CASE("a weapon slot position resolves to a weapon slot", "[auction-slot]") {
    // Weapons offer One-Hand first, which is inventory type 13.
    CHECK(auctionSlotIdAt(2, 0, 1) == 13);
    // And not the first row of the whole table, which is Head.
    CHECK(auctionSlotIdAt(2, 0, 1) != kAuctionSlots[1].invType);
}

TEST_CASE("an armour slot position resolves to a worn slot", "[auction-slot]") {
    CHECK(auctionSlotIdAt(4, 0, 1) == 1);   // Head
    CHECK(auctionSlotIdAt(4, 0, 2) == 2);   // Neck
    CHECK(auctionSlotIdAt(4, 0, 3) == 3);   // Shoulder
}

namespace {

/// Every inventory type offered under a class and subclass.
std::set<uint32_t> offeredSlots(uint32_t classId, int subIndex) {
    std::set<uint32_t> slots;
    int count = 0;
    if (!auctionSlotsFor(classId, subIndex, count)) return slots;
    for (int position = 1; position <= count; ++position) {
        slots.insert(auctionSlotIdAt(classId, subIndex, position));
    }
    return slots;
}

// Positions in kAuctionArmorSubs.
constexpr int kArmorAll           = 0;
constexpr int kArmorCloth         = 1;
constexpr int kArmorMiscellaneous = 6;

// Inventory types, as kAuctionSlots carries them.
constexpr uint32_t kNeck    = 2;
constexpr uint32_t kFinger  = 11;
constexpr uint32_t kTrinket = 12;
constexpr uint32_t kCloak   = 16;
constexpr uint32_t kChest   = 5;

}  // namespace

TEST_CASE("the category rings are in offers the finger slot", "[auction-slot]") {
    // Armor > Miscellaneous is the subclass every ring, neck and trinket in the
    // game belongs to. Its list was written as positions with the slot names in
    // a comment beside them, and the two had drifted a row apart - Finger was
    // not in it, so no search in the auction house could ask for a ring.
    //
    // The checks above cannot see this: each position it did offer was a real
    // slot, and no slot was offered twice. Only the names catch it.
    const std::set<uint32_t> misc = offeredSlots(4, kArmorMiscellaneous);
    CHECK(misc.count(kFinger) == 1);
    CHECK(misc.count(kNeck) == 1);
    CHECK(misc.count(kTrinket) == 1);
    // And under everything, which is where a search starts.
    CHECK(offeredSlots(4, kArmorAll).count(kFinger) == 1);
}

TEST_CASE("a suit of armour is not searched by the fingers", "[auction-slot]") {
    // The other half of the same drift: cloth and plate offered Neck and Finger,
    // which no item is - jewellery is Miscellaneous, whatever it is made of -
    // and did not offer Back, which every cloak in the game is.
    const std::set<uint32_t> cloth = offeredSlots(4, kArmorCloth);
    CHECK(cloth.count(kFinger) == 0);
    CHECK(cloth.count(kNeck) == 0);
    CHECK(cloth.count(kCloak) == 1);
    CHECK(cloth.count(kChest) == 1);
}

TEST_CASE("every offered position resolves to a real slot", "[auction-slot]") {
    for (const uint32_t classId : {uint32_t{2}, uint32_t{4}}) {
        int count = 0;
        const uint8_t* slots = auctionSlotsFor(classId, 0, count);
        REQUIRE(slots != nullptr);
        REQUIRE(count > 0);
        for (int position = 1; position <= count; ++position) {
            INFO("class " << classId << " position " << position);
            const uint32_t id = auctionSlotIdAt(classId, 0, position);
            CHECK(id != kAuctionAny);
            CHECK(id != 0);
        }
    }
}

TEST_CASE("no slot is offered twice under one class", "[auction-slot]") {
    // A repeat would give two rows that search the same thing and shift every
    // position after it.
    for (const uint32_t classId : {uint32_t{2}, uint32_t{4}}) {
        int count = 0;
        const uint8_t* slots = auctionSlotsFor(classId, 0, count);
        REQUIRE(slots != nullptr);
        for (int i = 0; i < count; ++i) {
            for (int j = i + 1; j < count; ++j) {
                INFO("class " << classId << " positions " << i + 1 << " and " << j + 1);
                CHECK(slots[i] != slots[j]);
            }
        }
    }
}

TEST_CASE("every offered index is inside the slot table", "[auction-slot]") {
    for (const uint32_t classId : {uint32_t{2}, uint32_t{4}}) {
        int count = 0;
        const uint8_t* slots = auctionSlotsFor(classId, 0, count);
        REQUIRE(slots != nullptr);
        for (int i = 0; i < count; ++i) {
            INFO("class " << classId << " entry " << i);
            CHECK(slots[i] >= 1);
            CHECK(slots[i] < kNumAuctionSlots);
        }
    }
}

TEST_CASE("nothing selected searches every slot", "[auction-slot]") {
    // Zero is what an unselected filter arrives as, and it has to read as
    // "any" rather than as the first row.
    CHECK(auctionSlotIdAt(2, 0, 0) == kAuctionAny);
    CHECK(auctionSlotIdAt(4, 0, 0) == kAuctionAny);
    CHECK(auctionSlotIdAt(4, 0, -1) == kAuctionAny);
}

TEST_CASE("a class with no slots offers none", "[auction-slot]") {
    // Consumables and the rest search whole, as they do in the real client.
    int count = 99;
    CHECK(auctionSlotsFor(0, 0, count) == nullptr);
    CHECK(count == 0);
    CHECK(auctionSlotIdAt(0, 0, 1) == kAuctionAny);
}

TEST_CASE("a position past the end is refused", "[auction-slot]") {
    int count = 0;
    auctionSlotsFor(2, 0, count);
    CHECK(auctionSlotIdAt(2, 0, count + 1) == kAuctionAny);
    CHECK(auctionSlotIdAt(2, 0, 999) == kAuctionAny);
}

TEST_CASE("every slot has a global string token", "[auction-slot]") {
    // FrameXML resolves the name it is given; an empty one draws nothing and
    // the row is a blank line in the tree.
    for (int i = 1; i < kNumAuctionSlots; ++i) {
        INFO(kAuctionSlots[i].label);
        REQUIRE(kAuctionSlots[i].token != nullptr);
        CHECK(std::string(kAuctionSlots[i].token).rfind("INVTYPE_", 0) == 0);
    }
}

TEST_CASE("a subclass narrows the list to where that item is worn",
          "[auction-slot]") {
    // A shield has one slot and a bow has one, so offering twenty under each
    // is a tree to read past rather than a filter. The subclass positions are
    // the ones GetAuctionItemSubClasses hands out.
    int count = 0;

    auctionSlotsFor(4, 5, count);            // Armor, Shield
    CHECK(count == 1);
    CHECK(auctionSlotIdAt(4, 5, 1) == 14);   // INVTYPE_SHIELD

    auctionSlotsFor(2, 3, count);            // Weapon, Bow
    CHECK(count == 1);
    CHECK(auctionSlotIdAt(2, 3, 1) == 26);   // INVTYPE_RANGED

    // Nine: the eight places a suit is worn, and the back. Ten before, which
    // was those nine with Neck and Finger in place of the back - jewellery
    // this subclass cannot contain, and no room for the cloaks it does.
    auctionSlotsFor(4, 1, count);            // Armor, Cloth
    CHECK(count == 9);
    CHECK(auctionSlotIdAt(4, 1, 1) == 1);    // Head

    auctionSlotsFor(2, 9, count);            // Weapon, Sword (2H)
    CHECK(count == 1);
    CHECK(auctionSlotIdAt(2, 9, 1) == 17);   // INVTYPE_2HWEAPON

    // Miscellaneous armour is the jewellery, not the worn plate.
    auctionSlotsFor(4, 6, count);
    CHECK(count == 4);
    CHECK(auctionSlotIdAt(4, 6, 1) == 2);    // Neck
}

TEST_CASE("an unpicked subclass still offers the whole class", "[auction-slot]") {
    // Selecting Armor without a subclass has to keep every armour slot, or
    // the filter disappears until a subclass is chosen.
    int count = 0;
    // Fifteen with the off hand, which is armour and was in none of the lists.
    auctionSlotsFor(4, 0, count);
    CHECK(count == 15);
    auctionSlotsFor(2, 0, count);
    CHECK(count == 7);
}
