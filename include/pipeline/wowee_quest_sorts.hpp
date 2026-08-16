#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Quest Sort catalog (.wqso) - novel
// replacement for Blizzard's QuestSort.dbc plus the quest-
// log categorization fields in QuestInfo.dbc. Defines the
// categories that quests fall into for the quest-log UI:
// Class quests / Profession quests / Daily quests / PvP /
// Holiday / Reputation / Dungeon / Raid / Heroic /
// Repeatable / Tournament etc.
//
// Each WQT (quest) entry references a sortId here to be
// grouped under the right header in the quest log. Sorts
// can be class-restricted (Warrior class quests only show
// for warriors), profession-restricted, or faction-
// reputation-gated.
//
// Cross-references with previously-added formats:
//   WQSO.entry.targetClassMask uses WCHC.classId bit
//                              positions (matches WGLY/WSET
//                              convention).
//   WQSO.entry.targetProfessionId → WTSK.profession enum
//                                   (Blacksmithing=0, etc).
//   WQSO.entry.targetFactionId    → WFAC.factionId
//
// Binary layout (little-endian):
//   magic[4]            = "WQSO"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     sortId (uint32)
//     nameLen + name
//     displayLen + displayName
//     descLen + description
//     iconLen + iconPath
//     sortKind (uint8) / displayPriority (uint8) /
//       targetProfessionId (uint8) / pad[1]
//     targetClassMask (uint32)
//     targetFactionId (uint32)
struct WoweeQuestSort {
    enum SortKind : uint8_t {
        General      = 0,    // catch-all / area-quest
        ClassQuest   = 1,    // class-specific (Warrior trial)
        Profession   = 2,    // profession recipe / quest
        Daily        = 3,    // daily reset quest
        Holiday      = 4,    // seasonal event quest
        Reputation   = 5,    // faction grind quest
        Dungeon      = 6,    // 5-man dungeon quest
        Raid         = 7,    // 10/25-man raid quest
        Heroic       = 8,    // dungeon heroic-mode quest
        Repeatable   = 9,    // non-daily repeatable
        PvP          = 10,   // battleground / arena quest
        Tournament   = 11,   // Argent Tournament style
    };

    struct Entry {
        uint32_t sortId = 0;
        std::string name;
        std::string displayName;     // shown in quest log UI
        std::string description;
        std::string iconPath;
        uint8_t sortKind = General;
        uint8_t displayPriority = 0;
        uint8_t targetProfessionId = 0;  // WTSK profession enum
        uint32_t targetClassMask = 0;    // WCHC bit mask (0 = any)
        uint32_t targetFactionId = 0;    // WFAC cross-ref (0 = any)
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t sortId) const;

    static const char* sortKindName(uint8_t k);
};

class WoweeQuestSortLoader {
public:
    static bool save(const WoweeQuestSort& cat,
                     const std::string& basePath);
    static WoweeQuestSort load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-qso* variants.
    //
    //   makeStarter    - 3 generic sorts (General catch-all,
    //                     Daily reset, Repeatable non-daily).
    //   makeClass      - 10 class-specific sorts (Warrior /
    //                     Paladin / Hunter / Rogue / Priest /
    //                     DK / Shaman / Mage / Warlock / Druid)
    //                     each with the matching WCHC class
    //                     bit set.
    //   makeProfession - 8 profession sorts (Blacksmithing,
    //                     Tailoring, Engineering, Alchemy,
    //                     Enchanting, Leatherworking,
    //                     Jewelcrafting, Inscription).
    static WoweeQuestSort makeStarter(const std::string& catalogName);
    static WoweeQuestSort makeClass(const std::string& catalogName);
    static WoweeQuestSort makeProfession(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
