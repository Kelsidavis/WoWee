#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Achievement Catalog (.wach) - novel replacement
// for Blizzard's Achievement.dbc + AchievementCriteria.dbc +
// AchievementCategory.dbc + the AzerothCore-style
// character_achievement / character_achievement_progress
// SQL tables. The 21st open format added to the editor.
//
// Each achievement carries display metadata (name, description,
// icon, points, faction restriction) plus a list of criteria
// the player must satisfy. Criteria mirror the WQT objective
// model (kind + targetId + quantity), which means the runtime
// can reuse the same progress-tracking machinery for both.
//
// Cross-references with previously-added formats:
//   WACH.criteria.targetId (kind=KillCreature)    → WCRT.creatureId
//   WACH.criteria.targetId (kind=CompleteQuest)   → WQT.questId
//   WACH.criteria.targetId (kind=LootItem)        → WIT.itemId
//   WACH.criteria.targetId (kind=CastSpell)       → WSPL.spellId
//   WACH.criteria.targetId (kind=ReachSkillLevel) → WSKL.skillId
//   WACH.criteria.targetId (kind=EarnReputation)  → WFAC.factionId
//   WACH.entry.categoryId  → WACH.entry.achievementId (for header
//                             rows; achievements can be parented
//                             to other achievements as headers)
//
// Binary layout (little-endian):
//   magic[4]            = "WACH"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     achievementId (uint32)
//     categoryId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     titleLen + titleReward
//     points (uint32)
//     minLevel (uint16) / faction (uint8) / criteriaCount (uint8)
//     flags (uint32)
//     criteria (criteriaCount × {
//         criteriaId (uint32)
//         kind (uint8) + pad[3]
//         targetId (uint32)
//         quantity (uint32)
//         descLen + description
//     })
struct WoweeAchievement {
    enum CriteriaKind : uint8_t {
        KillCreature    = 0,
        CompleteQuest   = 1,
        LootItem        = 2,
        ReachLevel      = 3,
        EarnReputation  = 4,
        CastSpell       = 5,
        ReachSkillLevel = 6,
        VisitArea       = 7,
        CompleteAchievement = 8,    // meta-achievements
    };

    enum Faction : uint8_t {
        FactionBoth     = 0,
        FactionAlliance = 1,
        FactionHorde    = 2,
    };

    enum Flags : uint32_t {
        HiddenUntilEarned = 0x01,    // not shown in panel until completed
        ServerFirst       = 0x02,    // first-on-server rewards a global tag
        RealmFirst        = 0x04,
        Tracking          = 0x08,    // shows progress UI in the panel
        Counter           = 0x10,    // counts up forever (Pet Battles wins)
        Account           = 0x20,    // account-wide, not per-character
    };

    struct Criterion {
        uint32_t criteriaId = 0;
        uint8_t kind = KillCreature;
        uint32_t targetId = 0;
        uint32_t quantity = 1;
        std::string description;
    };

    struct Entry {
        uint32_t achievementId = 0;
        uint32_t categoryId = 0;          // 0 = top-level
        std::string name;
        std::string description;
        std::string iconPath;
        std::string titleReward;          // empty = no title
        uint32_t points = 10;
        uint16_t minLevel = 1;
        uint8_t faction = FactionBoth;
        uint32_t flags = 0;
        std::vector<Criterion> criteria;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by achievementId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t achievementId) const;

    static const char* criteriaKindName(uint8_t k);
    static const char* factionName(uint8_t f);
};

class WoweeAchievementLoader {
public:
    static bool save(const WoweeAchievement& cat,
                     const std::string& basePath);
    static WoweeAchievement load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-achievements* variants.
    //
    //   makeStarter - 3 demo achievements covering kill / quest
    //                  completion / level reached criteria.
    //   makeBandit  - bandit-themed: "Slay 50 Defias Bandits"
    //                  + "Loot the Bandit Strongbox" + "Complete
    //                  Bandit Trouble" - all referencing the
    //                  WCRT/WGOT/WQT/WIT cross-referenced demo IDs.
    //   makeMeta    - 3 base achievements + 1 meta-achievement
    //                  that requires completing the others.
    static WoweeAchievement makeStarter(const std::string& catalogName);
    static WoweeAchievement makeBandit(const std::string& catalogName);
    static WoweeAchievement makeMeta(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
