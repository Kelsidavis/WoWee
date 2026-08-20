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

/// The 28 general bank slots, inside kNoContainer, after the worn equipment.
inline constexpr int kBankGeneralFirst = 39;
inline constexpr int kBankGeneralCount = 28;

/// The 7 bank bag slots, following the general ones.
inline constexpr int kBankBagFirst = kBankGeneralFirst + kBankGeneralCount;  // 67
inline constexpr int kBankBagCount = 7;

/// The keyring, past the bags.
inline constexpr int kKeyringFirst = 86;

inline constexpr int backpackWireSlot(int index)    { return kBackpackFirst + index; }
inline constexpr int bankGeneralWireSlot(int index) { return kBankGeneralFirst + index; }
inline constexpr int keyringWireSlot(int index)     { return kKeyringFirst + index; }

/// The container number of the nth worn bag.
inline constexpr int wornBagContainer(int index)    { return kWornBagFirst + index; }

/// The nth bank bag - both the slot the bag sits in and the container its
/// contents sit in, which are the same number. Named twice so the two uses read
/// as deliberate rather than as one of them being a mistake.
inline constexpr int bankBagWireSlot(int index)     { return kBankBagFirst + index; }
inline constexpr int bankBagContainer(int index)    { return bankBagWireSlot(index); }

/// The interface numbers inventory slots from one; the wire numbers them from
/// zero. Every place that takes a slot from Lua and sends it crosses this, and
/// every place that got it wrong was off by exactly this.
inline constexpr int toInventorySlot(int wireSlot)  { return wireSlot + 1; }
inline constexpr int toWireSlot(int inventorySlot)  { return inventorySlot - 1; }

/// The inventory slot the first bank bag occupies, as Lua counts.
inline constexpr int kFirstBankBagInventorySlot = toInventorySlot(bankBagWireSlot(0));

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
