#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Gossip Menu catalog (.wgsp) - novel replacement
// for AzerothCore-style gossip_menu + gossip_menu_option +
// npc_text SQL tables PLUS the Blizzard NpcText.dbc family.
// The 23rd open format added to the editor.
//
// An NPC's dialogue tree: a menu of options the player can
// pick from when right-clicking the NPC. Each option may
// bridge to another menu, trigger a vendor / trainer
// interaction, offer a quest, etc.
//
// This format closes the WCRT.gossipId cross-reference gap
// from batch 116 - until now WCRT had a gossipId field that
// pointed to a format that didn't exist yet.
//
// Cross-references with previously-added formats:
//   WCRT.entry.gossipId        → WGSP.entry.menuId
//   WGSP.option.actionTarget (kind=Submenu) → WGSP.entry.menuId
//                                              (intra-format chain)
//   WGSP.option.actionTarget (kind=Vendor / Trainer)
//                                          → WTRN.entry.npcId
//   WGSP.option.actionTarget (kind=Quest)  → WQT.entry.questId
//
// Binary layout (little-endian):
//   magic[4]            = "WGSP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     menuId (uint32)
//     titleLen + titleText
//     optionCount (uint8) + pad[3]
//     options (optionCount × {
//       optionId (uint32)
//       textLen + text
//       kind (uint8) + pad[3]
//       actionTarget (uint32)
//       requiredFlags (uint32)
//       moneyCostCopper (uint32)
//     })
struct WoweeGossip {
    enum OptionKind : uint8_t {
        Close         = 0,    // closes the menu, no action
        Submenu       = 1,    // jumps to another menuId
        Vendor        = 2,    // opens vendor inventory window
        Trainer       = 3,    // opens trainer spell window
        Quest         = 4,    // opens quest dialog
        Tabard        = 5,    // opens tabard customization
        Banker        = 6,    // opens bank
        Innkeeper     = 7,    // sets hearth + opens vendor
        FlightMaster  = 8,    // opens taxi node
        TextOnly      = 9,    // dialogue only, no action
        Script        = 10,   // triggers a server-side script
        Battlemaster  = 11,
        Auctioneer    = 12,
    };

    enum OptionFlags : uint32_t {
        AllianceOnly  = 0x01,
        HordeOnly     = 0x02,
        Coinpouch     = 0x04,    // shows the coin icon when paid
        QuestGated    = 0x08,    // visible only with matching quest
        Closes        = 0x10,    // closes the menu after the action
    };

    struct Option {
        uint32_t optionId = 0;
        std::string text;
        uint8_t kind = TextOnly;
        uint32_t actionTarget = 0;     // submenu / NPC / quest id
        uint32_t requiredFlags = 0;
        uint32_t moneyCostCopper = 0;
    };

    struct Entry {
        uint32_t menuId = 0;
        std::string titleText;
        std::vector<Option> options;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by menuId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t menuId) const;

    static const char* optionKindName(uint8_t k);
};

class WoweeGossipLoader {
public:
    static bool save(const WoweeGossip& cat,
                     const std::string& basePath);
    static WoweeGossip load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-gossip* variants.
    //
    //   makeStarter - single menu with greeting + 3 options
    //                  (vendor / trainer / close).
    //   makeInnkeeper - menu 4001 (matches WCRT.gossipId on
    //                    Bartleby): set hearth + browse goods
    //                    + bind to flight + close.
    //   makeQuestGiver - branching menu: greeting + 2 quests +
    //                     submenu "tell me about the area"
    //                     leading to lore text + close.
    static WoweeGossip makeStarter(const std::string& catalogName);
    static WoweeGossip makeInnkeeper(const std::string& catalogName);
    static WoweeGossip makeQuestGiver(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
