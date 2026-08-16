#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Equipment catalog (.wceq) - novel
// replacement for the AzerothCore-style creature_equip_template
// SQL tables plus the visible-weapon / shield / ranged-slot
// data that's traditionally embedded in creature templates.
// New open format that closes a long-standing gap in the
// creature subsystem.
//
// Until this format existed, WCRT defined a creature's stats,
// WSPN placed it in the world, and WLOT defined what it drops
// - but nothing defined what items it visibly equips. WCEQ
// binds a creatureId to up to three equipped items (main
// hand, off hand, ranged) plus the visual kit that fires
// when the main-hand weapon is brandished.
//
// Cross-references with previously-added formats:
//   WCEQ.entry.creatureId       → WCRT.creatureId
//   WCEQ.entry.mainHandItemId   → WIT.itemId
//   WCEQ.entry.offHandItemId    → WIT.itemId
//   WCEQ.entry.rangedItemId     → WIT.itemId
//   WCEQ.entry.mainHandVisualId → WSVK.visualKitId
//                                  (cast effect when equipped)
//
// Binary layout (little-endian):
//   magic[4]            = "WCEQ"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     equipmentId (uint32)
//     creatureId (uint32)
//     nameLen + name
//     descLen + description
//     mainHandItemId (uint32)
//     offHandItemId (uint32)
//     rangedItemId (uint32)
//     mainHandSlot (uint8) / offHandSlot (uint8) /
//       rangedSlot (uint8) / equipFlags (uint8)
//     mainHandVisualId (uint32)
struct WoweeCreatureEquipment {
    // Equipment-slot indices match the WoW visible-equipment
    // attachment table - Mainhand=16, Offhand=17, Ranged=18.
    static constexpr uint8_t kSlotMainHand = 16;
    static constexpr uint8_t kSlotOffHand  = 17;
    static constexpr uint8_t kSlotRanged   = 18;

    // equipFlags bits - modifiers for how the items are
    // worn / brandished.
    static constexpr uint8_t kFlagHidden        = 0x01;  // weapons sheathed
    static constexpr uint8_t kFlagDualWield     = 0x02;  // off-hand is weapon
    static constexpr uint8_t kFlagShieldOffhand = 0x04;  // off-hand is shield
    static constexpr uint8_t kFlagThrownRanged  = 0x08;  // ranged is thrown
    static constexpr uint8_t kFlagPolearmTwoHand = 0x10; // main is 2H polearm

    struct Entry {
        uint32_t equipmentId = 0;
        uint32_t creatureId = 0;        // WCRT cross-ref
        std::string name;
        std::string description;
        uint32_t mainHandItemId = 0;    // WIT cross-ref (0 = unarmed)
        uint32_t offHandItemId = 0;     // WIT cross-ref (0 = none)
        uint32_t rangedItemId = 0;      // WIT cross-ref (0 = none)
        uint8_t mainHandSlot = kSlotMainHand;
        uint8_t offHandSlot = kSlotOffHand;
        uint8_t rangedSlot = kSlotRanged;
        uint8_t equipFlags = 0;
        uint32_t mainHandVisualId = 0;  // WSVK cross-ref (optional)
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t equipmentId) const;
};

class WoweeCreatureEquipmentLoader {
public:
    static bool save(const WoweeCreatureEquipment& cat,
                     const std::string& basePath);
    static WoweeCreatureEquipment load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-ceq* variants.
    //
    //   makeStarter     - 3 generic loadouts (warrior 1H+
    //                      shield, hunter bow + offhand,
    //                      rogue dual-dagger).
    //   makeBosses      - 4 boss loadouts (Onyxian 2H sword,
    //                      Lich King's Frostmourne, Sylvanas's
    //                      bow, Illidan dual warglaives) with
    //                      WSVK main-hand visual cross-refs.
    //   makeRanged      - 3 ranged-only loadouts (gun, bow,
    //                      crossbow) covering the kFlagThrown
    //                      and Ranged-slot variants.
    static WoweeCreatureEquipment makeStarter(const std::string& catalogName);
    static WoweeCreatureEquipment makeBosses(const std::string& catalogName);
    static WoweeCreatureEquipment makeRanged(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
