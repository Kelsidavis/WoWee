#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Player Condition catalog (.wpcn) - novel
// replacement for Blizzard's PlayerCondition.dbc plus the
// AzerothCore-style condition resolver. The 49th open
// format added to the editor.
//
// Defines reusable conditional checks that other catalogs
// reference by conditionId: quest availability ("player
// is level 60+ AND has reputation honored with Stormwind"),
// gossip option visibility, vendor item gating, achievement
// criteria, spell trainer offerings. Conditions can chain
// via chainNextId + chainOp to express AND / OR / NOT
// composites, so even the simplest scalar entries scale up
// to arbitrary boolean trees.
//
// Cross-references with previously-added formats:
//   WPCN.entry.targetIdA      → polymorphic by conditionKind:
//                                 Race      → WCHC.race.raceId
//                                 Class     → WCHC.class.classId
//                                 Zone      → WMS.area.areaId
//                                 Map       → WMS.map.mapId
//                                 Reputation→ WFAC.factionId
//                                 Achievement→WACH.achievementId
//                                 Quest     → WQT.questId
//                                 SpellKnown→ WSPL.spellId
//                                 ItemEquipped→WIT.itemId
//                                 Faction   → WFAC.factionId
//   WPCN.entry.chainNextId    → WPCN.entry.conditionId
//                                 (composite chain terminator)
//
// Binary layout (little-endian):
//   magic[4]            = "WPCN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     conditionId (uint32)
//     nameLen + name
//     descLen + description
//     conditionKind (uint8) / comparisonOp (uint8) /
//       chainOp (uint8) / pad[1]
//     targetIdA (uint32)
//     targetIdB (uint32)
//     intValueA (int32)
//     intValueB (int32)
//     chainNextId (uint32)
//     failMsgLen + failMessage
struct WoweePlayerCondition {
    enum ConditionKind : uint8_t {
        Always         = 0,    // unconditional pass (used as base)
        Race           = 1,    // targetIdA = raceId
        Class          = 2,    // targetIdA = classId
        Level          = 3,    // intValueA = level threshold
        Zone           = 4,    // targetIdA = areaId
        Map            = 5,    // targetIdA = mapId
        Reputation     = 6,    // targetIdA = factionId, intA = standing
        AchievementWon = 7,    // targetIdA = achievementId
        QuestComplete  = 8,    // targetIdA = questId
        QuestActive    = 9,    // targetIdA = questId
        SpellKnown     = 10,   // targetIdA = spellId
        ItemEquipped   = 11,   // targetIdA = itemId
        Faction        = 12,   // targetIdA = factionId (membership)
        InCombat       = 13,   // boolean
        Mounted        = 14,   // boolean
        Resting        = 15,   // boolean
    };

    enum ComparisonOp : uint8_t {
        Equal          = 0,
        NotEqual       = 1,
        GreaterThan    = 2,
        GreaterOrEqual = 3,
        LessThan       = 4,
        LessOrEqual    = 5,
        InSet          = 6,    // targetIdA, targetIdB are 2 valid values
        NotInSet       = 7,
    };

    enum ChainOp : uint8_t {
        ChainNone = 0,         // no further check - terminator
        ChainAnd  = 1,         // also requires chainNextId to pass
        ChainOr   = 2,         // either this or chainNextId passes
        ChainNot  = 3,         // negate the chainNextId result
    };

    struct Entry {
        uint32_t conditionId = 0;
        std::string name;
        std::string description;
        uint8_t conditionKind = Always;
        uint8_t comparisonOp = Equal;
        uint8_t chainOp = ChainNone;
        uint32_t targetIdA = 0;
        uint32_t targetIdB = 0;
        int32_t intValueA = 0;
        int32_t intValueB = 0;
        uint32_t chainNextId = 0;       // WPCN cross-ref
        std::string failMessage;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t conditionId) const;

    static const char* conditionKindName(uint8_t k);
    static const char* comparisonOpName(uint8_t o);
    static const char* chainOpName(uint8_t c);
};

class WoweePlayerConditionLoader {
public:
    static bool save(const WoweePlayerCondition& cat,
                     const std::string& basePath);
    static WoweePlayerCondition load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-pcn* variants.
    //
    //   makeStarter    - 3 single-check conditions (level
    //                     60+, race Human, class Warrior).
    //   makeQuestGates - 4 quest-style gates (quest X
    //                     complete, reputation honored with
    //                     faction Y, achievement Z earned,
    //                     player in zone W).
    //   makeComposite  - 3 chained conditions exercising
    //                     AND / OR / NOT chainOps (level 80
    //                     AND warrior; ally rep OR horde rep;
    //                     NOT in-combat).
    static WoweePlayerCondition makeStarter(const std::string& catalogName);
    static WoweePlayerCondition makeQuestGates(const std::string& catalogName);
    static WoweePlayerCondition makeComposite(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
