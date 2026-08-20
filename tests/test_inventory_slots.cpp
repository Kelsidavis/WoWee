#include <catch_amalgamated.hpp>

#include "game/inventory_slots.hpp"

using namespace wowee::game::slots;

// A slot number is what a swap request names. Getting one wrong does not draw
// something odd or answer nil - it moves an item to a real place nobody asked
// for, which looks like the player did it. These pin the figures the client's
// own bank window has been sending, so the interface bindings and that window
// cannot drift apart.

TEST_CASE("The bank's three regions do not overlap", "[bank][slots]") {
    // The general slots run out exactly where the bags begin.
    REQUIRE(bankGeneralWireSlot(kBankGeneralCount - 1) + 1 == bankBagWireSlot(0));
    // And the bags end before the keyring starts.
    REQUIRE(bankBagWireSlot(kBankBagCount - 1) < keyringWireSlot(0));
}

TEST_CASE("The wire slots are the ones this client already sends",
          "[bank][slots]") {
    // Read out of inventory_screen.cpp, which has been sending these all along.
    REQUIRE(bankGeneralWireSlot(0) == 39);
    REQUIRE(bankBagWireSlot(0) == 67);
    REQUIRE(keyringWireSlot(0) == 86);
}

TEST_CASE("The interface's first bank bag is the wire's, one higher",
          "[bank][slots]") {
    // BankButtonIDToInvSlotID(1, isBag) answers this, and PickupBagFromSlot is
    // handed it back. The two have to meet exactly or a bag is picked up from
    // one slot and put down in another.
    REQUIRE(kFirstBankBagInventorySlot == 68);
    REQUIRE(toWireSlot(kFirstBankBagInventorySlot) == bankBagWireSlot(0));
}

TEST_CASE("Crossing between the two numberings round-trips", "[bank][slots]") {
    for (int wire = 0; wire < 100; ++wire) {
        REQUIRE(toWireSlot(toInventorySlot(wire)) == wire);
    }
}

TEST_CASE("Every bank bag maps to a distinct slot in both numberings",
          "[bank][slots]") {
    for (int i = 0; i < kBankBagCount; ++i) {
        const int inv = toInventorySlot(bankBagWireSlot(i));
        REQUIRE(inv == kFirstBankBagInventorySlot + i);
        REQUIRE(toWireSlot(inv) == bankBagWireSlot(i));
    }
}

TEST_CASE("The backpack and the worn bags are where the wire puts them",
          "[bank][slots]") {
    // Read out of cursorWireSlot and the split path, which have been sending
    // these all along.
    REQUIRE(backpackWireSlot(0) == 23);
    REQUIRE(wornBagContainer(0) == 19);
    // The backpack's slots sit inside no container, so they must not collide
    // with the bank's, which sit there too.
    REQUIRE(backpackWireSlot(kBackpackCount - 1) < bankGeneralWireSlot(0));
}

TEST_CASE("A bank bag's container is its slot", "[bank][slots]") {
    // Not a coincidence and not a bug: a bag is an item in a slot and a
    // container at once, and the wire gives both the same number. Two names
    // for it so neither use reads as a mistake.
    for (int i = 0; i < kBankBagCount; ++i) {
        REQUIRE(bankBagContainer(i) == bankBagWireSlot(i));
    }
}

TEST_CASE("No two regions inside the no-container overlap", "[bank][slots]") {
    // Equipment, backpack, bank and keyring all address through kNoContainer,
    // so an overlap would make one region's slot mean another's.
    REQUIRE(kNoContainer == 0xFF);
    REQUIRE(backpackWireSlot(kBackpackCount - 1) < kBankGeneralFirst);
    REQUIRE(bankBagWireSlot(kBankBagCount - 1) < kKeyringFirst);
}

// ---------------------------------------------------------------------------
// Where the cursor's item came from, which is a slot number like any other and
// went wrong the same way.

TEST_CASE("An item picked up off the action bar names no source slot",
          "[cursor][slots]") {
    // Reported: dragging a food off the action bar into a bag moved the
    // player's equipped bracers into it instead.
    //
    // Every negative source used to mean the paperdoll, and an item action
    // carries a real itemId, so a food lifted off a button was indistinguisha-
    // ble from one lifted off worn equipment - and the only number the pickup
    // had was which button it came from. Button nine less one is eight, which
    // is EquipSlot::WRISTS, so the swap sent named the bracers as its source.
    // Not the ninth button in particular: every button below twenty-four
    // pointed at something worn.
    REQUIRE_FALSE(cursorSourceIsInventory(kCursorNoSource));

    // The three kinds that do name a place, and still do.
    REQUIRE(cursorSourceIsInventory(-1));   // the paperdoll
    REQUIRE(cursorSourceIsInventory(0));    // the backpack
    for (int bag = 1; bag <= kWornBagCount; ++bag) {
        INFO("worn bag " << bag);
        REQUIRE(cursorSourceIsInventory(bag));
    }
}

TEST_CASE("The no-source marker is not a container number", "[cursor][slots]") {
    // It has to sit outside every value a real source can take, or it would
    // read as one of them - which is exactly how the paperdoll came to stand
    // in for the action bar.
    for (int bag = -1; bag <= kWornBagCount; ++bag) {
        INFO("source " << bag);
        REQUIRE(kCursorNoSource != bag);
    }
}
