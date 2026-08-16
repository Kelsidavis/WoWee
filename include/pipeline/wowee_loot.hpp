#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Loot Table (.wlot) - novel replacement for the
// creature_loot_template / gameobject_loot_template SQL
// tables AzerothCore-style servers use to drive what drops
// when a creature is killed (or a chest is opened). The
// 13th open format added to the editor.
//
// Pairs naturally with the WIT item catalog from the
// previous commit: each loot drop's itemId references an
// entry in a WIT file, so a content pack ships both the
// item definitions and the loot tables that reference them.
//
// One file holds many creature loot tables in one catalog.
// Each table has a moneyMin/moneyMax range plus a list of
// possible item drops. dropCount controls how many distinct
// drops to roll per kill (one die roll per slot, independent).
//
// Binary layout (little-endian):
//   magic[4]            = "WLOT"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes              -- catalog label
//   entryCount (uint32)
//   entries (each):
//     creatureId (uint32)
//     flags (uint32)
//     dropCount (uint8) + pad[3]
//     moneyMinCopper (uint32)
//     moneyMaxCopper (uint32)
//     itemDropCount (uint32)
//     itemDrops (itemDropCount × {
//         itemId (uint32)
//         chancePercent (float)            -- 0..100
//         minQty (uint8)
//         maxQty (uint8)
//         drop_flags (uint8) + pad[1]
//     })
struct WoweeLoot {
    enum TableFlags : uint32_t {
        QuestOnly  = 0x01,    // table only used while killer has matching quest
        GroupOnly  = 0x02,    // table only used in group / raid (not solo)
        Pickpocket = 0x04,    // alternate table used by rogues, not normal kill
    };

    enum DropFlags : uint8_t {
        QuestRequired   = 0x01,    // drop only if killer has matching quest
        GroupRollOnly   = 0x02,    // skip on solo kills (rare/epic-tier loot)
        AlwaysDrop      = 0x04,    // bypass dropCount slot limit
    };

    struct ItemDrop {
        uint32_t itemId = 0;
        float chancePercent = 100.0f;
        uint8_t minQty = 1;
        uint8_t maxQty = 1;
        uint8_t flags = 0;
    };

    struct Entry {
        uint32_t creatureId = 0;
        uint32_t flags = 0;
        uint8_t dropCount = 1;          // distinct drops rolled per kill
        uint32_t moneyMinCopper = 0;
        uint32_t moneyMaxCopper = 0;
        std::vector<ItemDrop> itemDrops;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by creatureId - nullptr if not present.
    [[nodiscard]] const Entry* findByCreatureId(uint32_t creatureId) const;
};

class WoweeLootLoader {
public:
    static bool save(const WoweeLoot& cat,
                     const std::string& basePath);
    static WoweeLoot load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-loot* variants.
    //
    //   makeStarter - minimal: 1 creature with 1 drop slot,
    //                  1 item @ 50% chance + 0..50 copper.
    //   makeBandit  - bandit table: dropCount=2, 4 candidate
    //                  items (linen, cloth, knife, ale), each
    //                  at distinct chances; 5..50 copper.
    //   makeBoss    - elite boss: dropCount=4, 6 candidates
    //                  including a guaranteed quest item, plus
    //                  GroupRollOnly epic at 5%; 50..200 silver.
    static WoweeLoot makeStarter(const std::string& catalogName);
    static WoweeLoot makeBandit(const std::string& catalogName);
    static WoweeLoot makeBoss(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
