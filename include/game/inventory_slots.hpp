#pragma once

#include <cstdint>

// Where everything the player carries sits, as the wire numbers it.
//
// These figures were written out by hand in more than twenty places across
// three files - the client's own bag and bank windows, and the interface
// bindings, each arriving at the same numbers by its own route. They agreed,
// and nothing made them agree.
//
// That matters more here than duplication usually does. A slot number is what
// a swap or a split request *names*, so a mismatch does not draw something odd
// or answer nil: it **moves an item somewhere real that nobody asked for**, and
// the wrong place looks like somewhere the player put it.
//
// Two conventions cross here, and between them they account for every mistake
// this arithmetic has produced:
//
//   * the wire counts from zero and the interface counts from one
//   * a bag is both a *container* things sit in and an *item* sitting in a
//     slot, and those are the same number
//
// The second is why bankBagWireSlot is used in two apparently different ways a
// few lines apart in the same function, and it is correct both times.

namespace wowee::game::slots {

/// The container number for things not inside a bag: worn equipment, the
/// backpack, and the bank's own slots.
inline constexpr uint8_t kNoContainer = 0xFF;

/// The backpack's own slots, inside kNoContainer.
inline constexpr int kBackpackFirst = 23;
inline constexpr int kBackpackCount = 16;

/// The four worn bags. A worn bag is a container of its own.
inline constexpr int kWornBagFirst = 19;
inline constexpr int kWornBagCount = 4;

/// The general bank slots, inside kNoContainer, after the worn equipment.
///
/// Where they start is the same in every expansion; how many there are is not,
/// and everything after them moves with it. Vanilla has 24 general slots and 6
/// bank bags, so its bank bags begin at 63 and its keyring at 81; 2.0 added
/// four general slots and a seventh bag, which is the layout WotLK still has.
///
/// The count and everything past it are read rather than fixed - see
/// bankLayout(). Only the first is a constant, because only the first agrees.
inline constexpr int kBankGeneralFirst = 39;

/// Where the bank's slots are, for the expansion now being played.
///
/// Verified against the cores rather than recalled: turtle's Player.h has
/// BANK_SLOT_ITEM_START 39 / _END 63, BANK_SLOT_BAG_START 63 / _END 69 and
/// KEYRING_SLOT_START 81; 3.3.5 has 39/67, 67/74 and 86.
///
/// TBC is on the later layout here on the strength of 2.0 having added the
/// slots, which is the one part of this not read off a core. If a 2.4.3 tree
/// ever says otherwise, this is the one place to change.
struct BankLayout {
    int generalCount;
    int bagFirst;
    int bagCount;
    int keyringFirst;
};

/// The two tables, without asking anything about the world - so both can be
/// checked in a test, where there is no expansion to be playing.
inline constexpr BankLayout bankLayoutFor(bool classicLike) {
    return classicLike ? BankLayout{24, 63, 6, 81}
                       : BankLayout{28, 67, 7, 86};
}

/// The one the expansion now being played uses.
///
/// State rather than a lookup, and set once when the expansion is chosen -
/// ExpansionRegistry::setActive does it. The alternative was asking the
/// registry from inside the slot arithmetic, which every handler includes and
/// none of which should have to know what an Application is.
///
/// The later layout until something says otherwise, which is what a client with
/// no expansion chosen has always assumed.
inline BankLayout& bankLayoutRef() {
    static BankLayout layout = bankLayoutFor(false);
    return layout;
}
inline BankLayout bankLayout() { return bankLayoutRef(); }
inline void setBankLayout(const BankLayout& layout) { bankLayoutRef() = layout; }

/// The same four, for a caller that wants one of them and not the struct.
inline int bankGeneralCount() { return bankLayout().generalCount; }
inline int bankBagFirst()     { return bankLayout().bagFirst; }
inline int bankBagCount()     { return bankLayout().bagCount; }
inline int keyringFirst()     { return bankLayout().keyringFirst; }

inline constexpr int backpackWireSlot(int index)    { return kBackpackFirst + index; }
inline constexpr int bankGeneralWireSlot(int index) { return kBankGeneralFirst + index; }
inline int keyringWireSlot(int index)              { return keyringFirst() + index; }

/// The container number of the nth worn bag.
inline constexpr int wornBagContainer(int index)    { return kWornBagFirst + index; }

/// The nth bank bag - both the slot the bag sits in and the container its
/// contents sit in, which are the same number. Named twice so the two uses read
/// as deliberate rather than as one of them being a mistake.
inline int bankBagWireSlot(int index)               { return bankBagFirst() + index; }
inline int bankBagContainer(int index)              { return bankBagWireSlot(index); }

/// The interface numbers inventory slots from one; the wire numbers them from
/// zero. Every place that takes a slot from Lua and sends it crosses this, and
/// every place that got it wrong was off by exactly this.
inline constexpr int toInventorySlot(int wireSlot)  { return wireSlot + 1; }
inline constexpr int toWireSlot(int inventorySlot)  { return inventorySlot - 1; }

/// The inventory slot the first bank bag occupies, as Lua counts.
inline int firstBankBagInventorySlot() { return toInventorySlot(bankBagWireSlot(0)); }

/// Where an item on the cursor was picked up from, in the numbering the
/// interface bindings keep beside it: 0 the backpack, 1 to 4 a worn bag, and
/// -1 worn equipment, where the slot is an equipment slot rather than an index
/// inside a container.
///
/// And one that is none of those. An item can be on the cursor without being
/// anywhere the wire can name. An action bar slot holds a *reference* to an
/// item, not the item: picking a food up off the bar puts the food on the
/// cursor while the food itself stays in whatever bag it was always in. There
/// is no source slot to swap from, and the number the pickup does have - which
/// button it came off - belongs to a fourth numbering that means nothing here.
///
/// With no value for that, an item taken off the bar read as the paperdoll and
/// its action slot read as an equipment slot, so dropping it in a bag sent a
/// swap for whatever was worn one below its button. Food on the ninth button
/// moved the player's bracers into the bag.
inline constexpr int kCursorNoSource = -2;

/// Whether an item on the cursor came from somewhere a swap can name as its
/// source. Only ask it of a cursor that is holding an item.
inline constexpr bool cursorSourceIsInventory(int cursorBag) {
    return cursorBag >= -1;
}

} // namespace wowee::game::slots
