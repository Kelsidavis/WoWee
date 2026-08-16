#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Trainer / Vendor catalog (.wtrn) - novel
// replacement for AzerothCore-style npc_trainer + npc_vendor
// SQL tables PLUS the Blizzard TrainerSpells.dbc family.
// The 22nd open format added to the editor.
//
// Unifies trainer spell lists and vendor inventories into one
// per-NPC entry. A creature flagged Trainer or Vendor in WCRT
// references a WTRN entry that lists what they teach / sell.
// The same NPC can be both - kindMask is a bitmask covering
// the Trainer (0x01) and Vendor (0x02) kinds.
//
// Cross-references with previously-added formats:
//   WTRN.entry.npcId            → WCRT.entry.creatureId
//   WTRN.spell.spellId          → WSPL.entry.spellId
//   WTRN.spell.requiredSkillId  → WSKL.entry.skillId
//   WTRN.item.itemId            → WIT.entry.itemId
//
// Binary layout (little-endian):
//   magic[4]            = "WTRN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     npcId (uint32)
//     kindMask (uint8) + pad[3]
//     greetingLen + greeting
//     spellCount (uint16) + itemCount (uint16)
//     spells (spellCount × {
//       spellId (uint32)
//       moneyCostCopper (uint32)
//       requiredSkillId (uint32)
//       requiredSkillRank (uint16)
//       requiredLevel (uint16)
//     })
//     items (itemCount × {
//       itemId (uint32)
//       stockCount (uint32)        -- 0xFFFFFFFF = unlimited
//       restockSec (uint32)
//       extendedCost (uint32)
//       moneyCostCopper (uint32)   -- 0 = use WIT.buyPrice
//     })
struct WoweeTrainer {
    enum KindMask : uint8_t {
        Trainer = 0x01,
        Vendor  = 0x02,
    };

    static constexpr uint32_t kUnlimitedStock = 0xFFFFFFFFu;

    struct SpellOffer {
        uint32_t spellId = 0;
        uint32_t moneyCostCopper = 0;
        uint32_t requiredSkillId = 0;        // 0 = no skill prerequisite
        uint16_t requiredSkillRank = 0;
        uint16_t requiredLevel = 1;
    };

    struct ItemOffer {
        uint32_t itemId = 0;
        uint32_t stockCount = kUnlimitedStock;
        uint32_t restockSec = 0;
        uint32_t extendedCost = 0;           // 0 = copper-only
        uint32_t moneyCostCopper = 0;        // 0 = inherit from WIT
    };

    struct Entry {
        uint32_t npcId = 0;
        uint8_t kindMask = 0;
        std::string greeting;
        std::vector<SpellOffer> spells;
        std::vector<ItemOffer> items;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by npcId - nullptr if not present.
    [[nodiscard]] const Entry* findByNpc(uint32_t npcId) const;

    // Decode the kindMask into a short string (e.g.
    // "trainer+vendor" or just "vendor").
    static std::string kindMaskName(uint8_t k);
};

class WoweeTrainerLoader {
public:
    static bool save(const WoweeTrainer& cat,
                     const std::string& basePath);
    static WoweeTrainer load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-trainers* variants.
    //
    //   makeStarter - 1 NPC (Bartleby innkeeper, npcId=4001)
    //                  acting as both vendor + trainer: sells
    //                  3 starter items + teaches First Aid.
    //   makeMageTrainer - npcId=4003 (alchemist) becomes a
    //                      mage trainer offering Frostbolt /
    //                      Fireball / Arcane Intellect / Blink
    //                      at appropriate ranks.
    //   makeWeaponVendor - npcId=4002 (smith) sells 5 weapons
    //                       across the WIT weapon catalog.
    static WoweeTrainer makeStarter(const std::string& catalogName);
    static WoweeTrainer makeMageTrainer(const std::string& catalogName);
    static WoweeTrainer makeWeaponVendor(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
