// auction_filters.hpp - the auction house's category tree, in one place.
//
// Both interfaces ask the same question of the same server field, so they ask
// it from the same table. This client's own auction window had these three
// arrays inside the function that drew them; FrameXML needs the identical
// lists, because GetAuctionItemClasses is what fills its filter column and the
// index it hands back to QueryAuctionItems has to mean the same thing on the
// way in as it did on the way out. Two copies of a list whose *positions* are
// the protocol is the shape that goes wrong quietly: the second category would
// search for the first one's items and nothing would say so.
//
// Row zero of every list is "All", and that is load-bearing rather than
// decorative. FrameXML's filter indices are 1-based and it passes nil when
// nothing is picked, which arrives as zero - so an unselected filter lands on
// row zero and reads as "any" without a special case. Before this, nil arrived
// as zero and went to the wire as zero, which is a real item class: searching
// the auction house by name alone returned consumables and nothing else.
//
// The labels are English literals, as they were in the window. ItemClass.dbc
// field 3 and ItemSubClass.dbc field 10 carry the localised names for the same
// ids (verified: class 0 is Consumable, class 1 subclass 2 is Herb Bag), and
// are where a localisation pass should read them from.
#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string_view>

namespace wowee {
namespace game {

/// 0xFFFFFFFF is what CMSG_AUCTION_LIST_ITEMS means by "do not filter on this".
/// AzerothCore's searcher compares `proto->Class != itemClass` for anything
/// else, so any other value is a filter - including zero.
inline constexpr uint32_t kAuctionAny = 0xFFFFFFFFu;

struct AuctionClassFilter { const char* label; uint32_t classId; };
struct AuctionSubFilter   { const char* label; uint32_t subId; };
/// A slot filter. `token` is the name of the global string FrameXML shows for
/// it - GetAuctionInvTypes hands back the name and the interface resolves it,
/// so the two sides agree on the wording without either spelling it out twice.
struct AuctionSlotFilter  { const char* label; uint32_t invType; const char* token; };

// WoW 3.3.5a item class ids: 0=Consumable, 1=Container, 2=Weapon, 3=Gem,
// 4=Armor, 6=Projectile, 7=Trade Goods, 9=Recipe, 11=Quiver, 12=Quest,
// 15=Miscellaneous, 16=Glyph.
//
// Twelve categories in the real client's order, and both facts matter to
// something outside this file. GetAuctionItemClasses hands this list to Lua
// positionally, and addons read positions out of it rather than names -
// Bagnon's item component opens with
//
//     select(12, GetAuctionItemClasses())
//
// to get "Quest" for its quest-item search, and a list of nine ending in
// Miscellaneous answered nothing there. format() was then handed a nil, which
// took down the file that builds every item slot it draws, so the addon loaded
// and displayed nothing.
//
// Glyph, Projectile and Quest were the three missing. Reordering the rest to
// match is safe because the position *is* the protocol only between this table
// and itself: FrameXML and this client's own window both read the categories
// from here and send back an index into the same array.
inline constexpr AuctionClassFilter kAuctionClasses[] = {
    {.label = "All",           .classId = kAuctionAny},
    {.label = "Weapon",        .classId = 2},
    {.label = "Armor",         .classId = 4},
    {.label = "Container",     .classId = 1},
    {.label = "Consumable",    .classId = 0},
    {.label = "Glyph",         .classId = 16},
    {.label = "Trade Goods",   .classId = 7},
    {.label = "Projectile",    .classId = 6},
    {.label = "Quiver",        .classId = 11},
    {.label = "Recipe",        .classId = 9},
    {.label = "Gem",           .classId = 3},
    {.label = "Miscellaneous", .classId = 15},
    {.label = "Quest",         .classId = 12},
};
inline constexpr int kNumAuctionClasses = 13;

inline constexpr AuctionSubFilter kAuctionWeaponSubs[] = {
    {.label = "All", .subId = kAuctionAny}, {.label = "Axe (1H)", .subId = 0}, {.label = "Axe (2H)", .subId = 1}, {.label = "Bow", .subId = 2},
    {.label = "Gun", .subId = 3}, {.label = "Mace (1H)", .subId = 4}, {.label = "Mace (2H)", .subId = 5}, {.label = "Polearm", .subId = 6},
    {.label = "Sword (1H)", .subId = 7}, {.label = "Sword (2H)", .subId = 8}, {.label = "Staff", .subId = 10},
    {.label = "Fist Weapon", .subId = 13}, {.label = "Dagger", .subId = 15}, {.label = "Thrown", .subId = 16},
    {.label = "Crossbow", .subId = 18}, {.label = "Wand", .subId = 19},
};
inline constexpr int kNumAuctionWeaponSubs = 16;

inline constexpr AuctionSubFilter kAuctionArmorSubs[] = {
    {.label = "All", .subId = kAuctionAny}, {.label = "Cloth", .subId = 1}, {.label = "Leather", .subId = 2}, {.label = "Mail", .subId = 3},
    {.label = "Plate", .subId = 4}, {.label = "Shield", .subId = 6}, {.label = "Miscellaneous", .subId = 0},
};
inline constexpr int kNumAuctionArmorSubs = 7;

/// Equipment-slot ids, carried in the auctionSlotID field of the same request.
inline constexpr AuctionSlotFilter kAuctionSlots[] = {
    {.label = "All Slots",     .invType = kAuctionAny, .token = ""},
    {.label = "Head",          .invType = 1,  .token = "INVTYPE_HEAD"},
    {.label = "Neck",          .invType = 2,  .token = "INVTYPE_NECK"},
    {.label = "Shoulder",      .invType = 3,  .token = "INVTYPE_SHOULDER"},
    {.label = "Chest",         .invType = 5,  .token = "INVTYPE_CHEST"},
    {.label = "Waist",         .invType = 6,  .token = "INVTYPE_WAIST"},
    {.label = "Legs",          .invType = 7,  .token = "INVTYPE_LEGS"},
    {.label = "Feet",          .invType = 8,  .token = "INVTYPE_FEET"},
    {.label = "Wrist",         .invType = 9,  .token = "INVTYPE_WRIST"},
    {.label = "Hands",         .invType = 10, .token = "INVTYPE_HAND"},
    {.label = "Finger",        .invType = 11, .token = "INVTYPE_FINGER"},
    {.label = "Trinket",       .invType = 12, .token = "INVTYPE_TRINKET"},
    {.label = "Back",          .invType = 16, .token = "INVTYPE_CLOAK"},
    {.label = "One-Hand",      .invType = 13, .token = "INVTYPE_WEAPON"},
    {.label = "Two-Hand",      .invType = 17, .token = "INVTYPE_2HWEAPON"},
    {.label = "Main Hand",     .invType = 21, .token = "INVTYPE_WEAPONMAINHAND"},
    {.label = "Off Hand",      .invType = 22, .token = "INVTYPE_WEAPONOFFHAND"},
    {.label = "Ranged",        .invType = 26, .token = "INVTYPE_RANGED"},
    {.label = "Shield",        .invType = 14, .token = "INVTYPE_SHIELD"},
    {.label = "Held Off-hand", .invType = 23, .token = "INVTYPE_HOLDABLE"},
    {.label = "Relic",         .invType = 28, .token = "INVTYPE_RELIC"},
};
inline constexpr int kNumAuctionSlots = 21;

/// Where a slot sits in kAuctionSlots, found by its inventory-type token.
///
/// The lists below used to be written as bare positions with the slot names in
/// a comment beside them, and the two drifted apart: the Miscellaneous list
/// said "Neck, Finger, Trinket, Back" and held the positions of Neck, Trinket,
/// Back and Relic. So Armor > Miscellaneous - the category every ring in the
/// game is in - offered every slot except the one for rings, and searching for
/// one found nothing. Nothing could have noticed: every number in it was a
/// valid row, just not the row the comment named.
///
/// Looked up by name, the list is the comment.
inline constexpr uint8_t auctionSlotPos(std::string_view token) {
    for (int i = 0; i < kNumAuctionSlots; ++i) {
        if (std::string_view(kAuctionSlots[i].token) == token) {
            return static_cast<uint8_t>(i);
        }
    }
    // Out of range on purpose: the static_asserts below fail on it, and
    // auctionSlotIdAt treats it as no slot rather than as some other one.
    return static_cast<uint8_t>(kNumAuctionSlots);
}

inline constexpr uint8_t kSlotHead        = auctionSlotPos("INVTYPE_HEAD");
inline constexpr uint8_t kSlotNeck        = auctionSlotPos("INVTYPE_NECK");
inline constexpr uint8_t kSlotShoulder    = auctionSlotPos("INVTYPE_SHOULDER");
inline constexpr uint8_t kSlotChest       = auctionSlotPos("INVTYPE_CHEST");
inline constexpr uint8_t kSlotWaist       = auctionSlotPos("INVTYPE_WAIST");
inline constexpr uint8_t kSlotLegs        = auctionSlotPos("INVTYPE_LEGS");
inline constexpr uint8_t kSlotFeet        = auctionSlotPos("INVTYPE_FEET");
inline constexpr uint8_t kSlotWrist       = auctionSlotPos("INVTYPE_WRIST");
inline constexpr uint8_t kSlotHands       = auctionSlotPos("INVTYPE_HAND");
inline constexpr uint8_t kSlotFinger      = auctionSlotPos("INVTYPE_FINGER");
inline constexpr uint8_t kSlotTrinket     = auctionSlotPos("INVTYPE_TRINKET");
inline constexpr uint8_t kSlotBack        = auctionSlotPos("INVTYPE_CLOAK");
inline constexpr uint8_t kSlotOneHand     = auctionSlotPos("INVTYPE_WEAPON");
inline constexpr uint8_t kSlotTwoHand     = auctionSlotPos("INVTYPE_2HWEAPON");
inline constexpr uint8_t kSlotMainHand    = auctionSlotPos("INVTYPE_WEAPONMAINHAND");
inline constexpr uint8_t kSlotOffHand     = auctionSlotPos("INVTYPE_WEAPONOFFHAND");
inline constexpr uint8_t kSlotRanged      = auctionSlotPos("INVTYPE_RANGED");
inline constexpr uint8_t kSlotShield      = auctionSlotPos("INVTYPE_SHIELD");
inline constexpr uint8_t kSlotHeldOffHand = auctionSlotPos("INVTYPE_HOLDABLE");
inline constexpr uint8_t kSlotRelic       = auctionSlotPos("INVTYPE_RELIC");

/// Every position in a list names a row. False is a token above that names
/// nothing, which is the one way the lookup can go wrong.
inline constexpr bool auctionSlotsKnown(const uint8_t* list, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        if (list[i] >= kNumAuctionSlots) return false;
    }
    return true;
}

/// The subclass list for a class id, or null when that class has none offered.
/// Only weapons and armour are divided here, which is what the window has
/// always shown; every other class searches whole.
inline const AuctionSubFilter* auctionSubsFor(uint32_t classId, int& count) {
    if (classId == 2) { count = kNumAuctionWeaponSubs; return kAuctionWeaponSubs; }
    if (classId == 4) { count = kNumAuctionArmorSubs;  return kAuctionArmorSubs;  }
    count = 0;
    return nullptr;
}

/// Which slots the interface offers under a class, as positions in
/// kAuctionSlots.
///
/// The position is the protocol: FrameXML sends the index it was given back
/// through QueryAuctionItems, which reads kAuctionSlots at that index. So this
/// answers indices rather than a rebuilt list - a subset renumbered from one
/// would search a different slot than the one that was clicked, and nothing
/// would say so.
///
/// Only weapons and armour have a slot to speak of; every other class searches
/// whole, which is what the real client does too.
inline const uint8_t* auctionSlotsFor(uint32_t classId, int subIndex, int& count) {
    // Positions in kAuctionSlots. A subclass narrows the list the way the real
    // client does: a shield is only ever worn in one place, and a bow is only
    // ever ranged, so offering every slot under each of them is a tree the
    // player has to read past rather than one that helps.
    static constexpr uint8_t kOneHand[] = {
        kSlotOneHand, kSlotMainHand, kSlotOffHand};
    static constexpr uint8_t kTwoHand[] = {kSlotTwoHand};
    static constexpr uint8_t kRanged[]  = {kSlotRanged};
    static constexpr uint8_t kAnyWeapon[] = {
        kSlotOneHand, kSlotTwoHand, kSlotMainHand, kSlotOffHand, kSlotRanged,
        kSlotHeldOffHand, kSlotRelic};
    // What a suit of armour is worn in. Not the neck or the fingers: a ring is
    // Miscellaneous armour, never cloth or plate, so those two rows searched a
    // combination no item has. The back is here instead, because a cloak is
    // cloth - it was in neither list, and could not be searched for at all.
    static constexpr uint8_t kWorn[] = {
        kSlotHead, kSlotShoulder, kSlotChest, kSlotWaist, kSlotLegs,
        kSlotFeet, kSlotWrist, kSlotHands, kSlotBack};
    static constexpr uint8_t kShield[] = {kSlotShield};
    // Miscellaneous armour: the jewellery, and the things held in the off hand
    // that are not weapons.
    static constexpr uint8_t kTrinketry[] = {
        kSlotNeck, kSlotFinger, kSlotTrinket, kSlotHeldOffHand};
    static constexpr uint8_t kAnyArmor[] = {
        kSlotHead, kSlotNeck, kSlotShoulder, kSlotChest, kSlotWaist, kSlotLegs,
        kSlotFeet, kSlotWrist, kSlotHands, kSlotFinger, kSlotTrinket, kSlotBack,
        kSlotShield, kSlotHeldOffHand, kSlotRelic};

    static_assert(auctionSlotsKnown(kOneHand, std::size(kOneHand)));
    static_assert(auctionSlotsKnown(kTwoHand, std::size(kTwoHand)));
    static_assert(auctionSlotsKnown(kRanged, std::size(kRanged)));
    static_assert(auctionSlotsKnown(kAnyWeapon, std::size(kAnyWeapon)));
    static_assert(auctionSlotsKnown(kWorn, std::size(kWorn)));
    static_assert(auctionSlotsKnown(kShield, std::size(kShield)));
    static_assert(auctionSlotsKnown(kTrinketry, std::size(kTrinketry)));
    static_assert(auctionSlotsKnown(kAnyArmor, std::size(kAnyArmor)));

    const auto give = [&count](const uint8_t* list, size_t n) {
        count = static_cast<int>(n);
        return list;
    };

    if (classId == 2) {
        // Positions in kAuctionWeaponSubs, which is what FrameXML hands back.
        switch (subIndex) {
            case 1: case 5: case 8: case 11: case 12:   // Axe/Mace/Sword 1H, Fist, Dagger
                return give(kOneHand, sizeof(kOneHand));
            case 2: case 6: case 7: case 9: case 10:    // Axe/Mace/Sword 2H, Polearm, Staff
                return give(kTwoHand, sizeof(kTwoHand));
            case 3: case 4: case 13: case 14: case 15:  // Bow, Gun, Thrown, Crossbow, Wand
                return give(kRanged, sizeof(kRanged));
            default:
                return give(kAnyWeapon, sizeof(kAnyWeapon));
        }
    }
    if (classId == 4) {
        // Positions in kAuctionArmorSubs.
        switch (subIndex) {
            case 1: case 2: case 3: case 4:   // Cloth, Leather, Mail, Plate
                return give(kWorn, sizeof(kWorn));
            case 5:                           // Shield
                return give(kShield, sizeof(kShield));
            case 6:                           // Miscellaneous
                return give(kTrinketry, sizeof(kTrinketry));
            default:
                return give(kAnyArmor, sizeof(kAnyArmor));
        }
    }
    count = 0;
    return nullptr;
}

/// The slot id for the position FrameXML hands back, under a given class.
///
/// FrameXML numbers the rows it was given from one - GetAuctionInvTypes
/// returns pairs and the row's index is its pair position - so the number that
/// comes back is a position in the offered subset, not in kAuctionSlots. The
/// two are only the same when the subset is the whole table, which it never
/// is. Reading kAuctionSlots at the raw position searched Head for a
/// one-handed weapon and said nothing.
inline uint32_t auctionSlotIdAt(uint32_t classId, int subIndex, int position) {
    if (position <= 0) return kAuctionAny;
    int count = 0;
    const uint8_t* slots = auctionSlotsFor(classId, subIndex, count);
    if (!slots || position > count) return kAuctionAny;
    const uint8_t at = slots[position - 1];
    if (at >= kNumAuctionSlots) return kAuctionAny;
    return kAuctionSlots[at].invType;
}

/// The three lengths an auction may run for, in minutes, which is what the
/// wire field carries: AzerothCore multiplies it by sixty and then accepts
/// only 1, 2 or 4 times MIN_AUCTION_TIME (12 hours), rejecting the request
/// outright otherwise. FrameXML's duration dropdown does not deal in minutes -
/// its values are 1, 2 and 3 for the same three lengths - so a posting made
/// through it asked for one, two or three minutes and the server dropped it
/// without a word.
inline constexpr uint32_t kAuctionDurationMinutes[3] = {720, 1440, 2880};

/// Minutes for a duration written either way: the dropdown's 1..3, or minutes
/// already. Anything else falls to twelve hours, which is the shortest the
/// server will take and so the least costly guess.
inline constexpr uint32_t auctionDurationMinutes(uint32_t d) {
    if (d >= 1 && d <= 3) return kAuctionDurationMinutes[d - 1];
    for (uint32_t m : kAuctionDurationMinutes) if (d == m) return d;
    return kAuctionDurationMinutes[0];
}

} // namespace game
} // namespace wowee
