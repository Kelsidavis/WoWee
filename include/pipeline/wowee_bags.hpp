#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Bag / Bank Slot catalog (.wbnk) - novel
// replacement for Blizzard's ItemBag.dbc plus the bank-
// storage and special-purpose container tables. Defines
// every slot the player has access to: equipped bags,
// bank bags, the keyring, soul shard bag, quiver, reagent
// bag, hunter pet stable, etc.
//
// Each entry describes ONE container slot - its kind, its
// fixed capacity (or whether it accepts a player-equipped
// bag for variable capacity), its display order in the
// inventory UI, and the unlock state (some bank bags
// require a gold purchase to open).
//
// Cross-references with previously-added formats:
//   WBNK.entry.fixedBagItemId → WIT.itemId
//                                (the bag item that this
//                                 slot ALWAYS contains -
//                                 0 = player-equipped slot)
//
// Binary layout (little-endian):
//   magic[4]            = "WBNK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     bagSlotId (uint32)
//     nameLen + name
//     descLen + description
//     bagKind (uint8) / containerSize (uint8) /
//       displayOrder (uint8) / isUnlocked (uint8)
//     fixedBagItemId (uint32)
//     unlockCostCopper (uint32)
//     acceptsBagSubclassMask (uint32)
struct WoweeBagSlot {
    enum BagKind : uint8_t {
        Inventory   = 0,    // base inventory bag slots
        Bank        = 1,    // bank window bags
        Keyring     = 2,    // keyring (fixed, classic-only)
        Quiver      = 3,    // arrow quiver - hunter only
        SoulShard   = 4,    // soul shard bag - warlock only
        Stable      = 5,    // hunter pet stable slot
        Reagent     = 6,    // reagent bag (post-Cata)
        Wallet      = 7,    // currency wallet (token-style)
    };

    // acceptsBagSubclassMask bits - only bags whose item
    // subclass matches at least one bit may be equipped here.
    // Bit 0 = generic container, bit 1 = soul shard bag,
    // bit 2 = herb bag, bit 3 = enchanting bag, bit 4 = engineer
    // bag, bit 5 = gem bag, bit 6 = mining bag, bit 7 = leather
    // bag, bit 8 = inscription bag, bit 9 = quiver, bit 10 =
    // ammo pouch.
    static constexpr uint32_t kAcceptsAnyContainer = 1u << 0;
    static constexpr uint32_t kAcceptsSoulShard    = 1u << 1;
    static constexpr uint32_t kAcceptsHerb         = 1u << 2;
    static constexpr uint32_t kAcceptsEnchanting   = 1u << 3;
    static constexpr uint32_t kAcceptsEngineer     = 1u << 4;
    static constexpr uint32_t kAcceptsGem          = 1u << 5;
    static constexpr uint32_t kAcceptsMining       = 1u << 6;
    static constexpr uint32_t kAcceptsLeather      = 1u << 7;
    static constexpr uint32_t kAcceptsInscription  = 1u << 8;
    static constexpr uint32_t kAcceptsQuiver       = 1u << 9;
    static constexpr uint32_t kAcceptsAmmoPouch    = 1u << 10;
    static constexpr uint32_t kAcceptsAll          = 0xFFFFFFFFu;

    struct Entry {
        uint32_t bagSlotId = 0;
        std::string name;
        std::string description;
        uint8_t bagKind = Inventory;
        uint8_t containerSize = 16;        // slots within the bag
        uint8_t displayOrder = 0;
        uint8_t isUnlocked = 1;            // 0 = needs purchase
        uint32_t fixedBagItemId = 0;       // WIT cross-ref or 0
        uint32_t unlockCostCopper = 0;     // gold to unlock (0 = free)
        uint32_t acceptsBagSubclassMask = kAcceptsAnyContainer;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bagSlotId) const;

    static const char* bagKindName(uint8_t k);
};

class WoweeBagSlotLoader {
public:
    static bool save(const WoweeBagSlot& cat,
                     const std::string& basePath);
    static WoweeBagSlot load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bnk* variants.
    //
    //   makeStarter - 5 inventory slots: 16-slot main backpack
    //                  (fixed) + 4 player-equipped bag slots
    //                  (variable, accept generic containers).
    //   makeBank    - 8 bank bag slots - slots 0..1 free, slots
    //                  2..7 require ascending gold purchases
    //                  (10g, 1g, 10g, 25g, 50g, 100g - matches
    //                  the WoW bank bag costs).
    //   makeSpecial - 4 special-purpose slots (Keyring fixed
    //                  with no equippable bag, SoulShard
    //                  warlock-only, Quiver hunter-only,
    //                  HuntersStable for pet storage).
    static WoweeBagSlot makeStarter(const std::string& catalogName);
    static WoweeBagSlot makeBank(const std::string& catalogName);
    static WoweeBagSlot makeSpecial(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
