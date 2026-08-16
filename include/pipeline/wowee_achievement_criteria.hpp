#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Achievement Criteria catalog (.wacr) -
// novel replacement for Blizzard's Achievement_Criteria.dbc.
// Defines the individual progression criteria that a
// character must complete to earn an achievement.
//
// Each WACH achievement has a tree of WACR criteria -
// "Kill 100 boars" is one criteria entry with
// criteriaType=KillCreature, targetId=boarCreatureId,
// requiredCount=100. Multi-criteria achievements
// (e.g. "Visit all 3 capital cities") have one entry
// per sub-objective, all referencing the same
// achievementId, with progressOrder determining their
// display sequence in the achievement UI.
//
// Cross-references with previously-added formats:
//   WACH: achievementId references the WACH parent.
//   WCRT: targetId references WCRT.creatureId for
//         KillCreature criteria.
//   WQT:  targetId references WQT.questId for
//         CompleteQuest criteria.
//   WIT:  targetId references WIT.itemId for LootItem /
//         UseItem criteria.
//   WMS:  targetId references WMS.zoneId for ExploreZone
//         criteria.
//
// Binary layout (little-endian):
//   magic[4]            = "WACR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     criteriaId (uint32)
//     nameLen + name
//     descLen + description
//     achievementId (uint32)
//     targetId (uint32)
//     requiredCount (uint32)
//     timeLimitMs (uint32)
//     criteriaType (uint8) / progressOrder (uint8) / pad[2]
//     iconColorRGBA (uint32)
struct WoweeAchievementCriteria {
    enum CriteriaType : uint8_t {
        KillCreature       = 0,    // count creature kills (targetId=creature)
        ReachLevel         = 1,    // hit a character level (requiredCount=lvl)
        CompleteQuest      = 2,    // finish a specific quest (targetId=quest)
        EarnGold           = 3,    // accumulate copper (requiredCount=copper)
        GainHonor          = 4,    // earn honor (requiredCount=honor)
        EarnReputation     = 5,    // hit rep level with faction (targetId=faction)
        ExploreZone        = 6,    // discover all subzones (targetId=zone)
        LootItem           = 7,    // loot a specific item (targetId=item)
        UseItem            = 8,    // use a specific item (targetId=item)
        CastSpell          = 9,    // cast a specific spell (targetId=spell)
        PvPKill            = 10,   // kill players (requiredCount=count)
        DungeonRun         = 11,   // complete a dungeon (targetId=instance)
        Misc               = 12,   // engine-defined custom progression
    };

    struct Entry {
        uint32_t criteriaId = 0;
        std::string name;
        std::string description;
        uint32_t achievementId = 0;
        uint32_t targetId = 0;
        uint32_t requiredCount = 1;
        uint32_t timeLimitMs = 0;
        uint8_t criteriaType = KillCreature;
        uint8_t progressOrder = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t criteriaId) const;

    // Return all criteria for the given achievement, sorted
    // by progressOrder. The achievement UI uses this to
    // show the progress checklist.
    [[nodiscard]] std::vector<const Entry*> findByAchievement(
        uint32_t achievementId) const;

    static const char* criteriaTypeName(uint8_t k);
};

class WoweeAchievementCriteriaLoader {
public:
    static bool save(const WoweeAchievementCriteria& cat,
                     const std::string& basePath);
    static WoweeAchievementCriteria load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-acr* variants.
    //
    //   makeKill   - 5 kill-counting criteria (Defias 50,
    //                 Murloc 25, Naga 100, Dragon 1,
    //                 RareElite 1) for a "Kill 'Em All"
    //                 style achievement.
    //   makeQuest  - 4 quest-completion criteria covering
    //                 tutorial/zone/daily/escort progression
    //                 in one composite achievement.
    //   makeMixed  - 5 cross-type criteria (ReachLevel 80,
    //                 EarnGold 10000, GainHonor 5000, PvPKill
    //                 100, ExploreZone Stormwind) showing
    //                 the variety of CriteriaType values.
    static WoweeAchievementCriteria makeKill(const std::string& catalogName);
    static WoweeAchievementCriteria makeQuest(const std::string& catalogName);
    static WoweeAchievementCriteria makeMixed(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
