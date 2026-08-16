#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Player Condition catalog (.wpcd) - novel
// replacement for Blizzard's PlayerCondition.dbc + the
// AzerothCore-style condition_template SQL tables. The
// 37th open format added to the editor.
//
// Defines reusable boolean conditions that other formats
// reference for gating: "player has quest X completed",
// "player level >= N", "player class is mage", "player
// has item Y in inventory", "event is currently active".
//
// Conditions can be grouped and combined with AND/OR
// aggregators on a per-group basis: a quest-giver gossip
// option that says "show only to level 60 alliance mages
// who completed quest 1234" composes 4 conditions sharing
// the same groupId with AND aggregation.
//
// Cross-references with previously-added formats:
//   WPCD.targetId (kind=QuestCompleted/Active) → WQT.questId
//   WPCD.targetId (kind=HasItem)               → WIT.itemId
//   WPCD.targetId (kind=HasSpell)              → WSPL.spellId
//   WPCD.targetId (kind=HasAchievement)        → WACH.achievementId
//   WPCD.targetId (kind=AreaId)                → WMS.areaId
//   WPCD.targetId (kind=EventActive)           → WSEA.eventId
//   WPCD.targetId (kind=HasTitle)              → WTIT.titleId
//   WPCD.targetId (kind=FactionRep)            → WFAC.factionId
//
// Other formats can reference WPCD.conditionId in future
// extensions to gate triggers, gossip options, quest
// availability, vendor visibility, mount summons, etc.
//
// Binary layout (little-endian):
//   magic[4]            = "WPCD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     conditionId (uint32)
//     groupId (uint32)
//     nameLen + name
//     descLen + description
//     kind (uint8) / aggregator (uint8) / negated (uint8) / pad[1]
//     targetId (uint32)
//     minValue (int32) / maxValue (int32)
struct WoweeCondition {
    enum Kind : uint8_t {
        AlwaysTrue       = 0,
        AlwaysFalse      = 1,
        QuestCompleted   = 2,
        QuestActive      = 3,
        HasItem          = 4,    // qty >= minValue
        HasSpell         = 5,    // spell known
        MinLevel         = 6,    // player.level >= minValue
        MaxLevel         = 7,    // player.level <= minValue
        ClassMatch       = 8,    // player.classId in raceMask sense
        RaceMatch        = 9,
        FactionRep       = 10,   // player.rep[targetId] >= minValue
        HasAchievement   = 11,
        TeamSize         = 12,   // group size in [minValue, maxValue]
        GuildLevel       = 13,   // player's guild level >= minValue
        EventActive      = 14,   // WSEA event currently running
        AreaId           = 15,   // player in WMS.areaId
        HasTitle         = 16,
    };

    enum Aggregator : uint8_t {
        And = 0,    // all siblings in groupId must be true
        Or  = 1,    // any sibling in groupId being true is enough
    };

    struct Entry {
        uint32_t conditionId = 0;
        uint32_t groupId = 0;       // 0 = standalone (no group)
        std::string name;
        std::string description;
        uint8_t kind = AlwaysTrue;
        uint8_t aggregator = And;
        uint8_t negated = 0;        // 1 = invert the result
        uint32_t targetId = 0;
        int32_t minValue = 0;
        int32_t maxValue = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t conditionId) const;

    static const char* kindName(uint8_t k);
    static const char* aggregatorName(uint8_t a);
};

class WoweeConditionLoader {
public:
    static bool save(const WoweeCondition& cat,
                     const std::string& basePath);
    static WoweeCondition load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-conditions* variants.
    //
    //   makeStarter - 4 standalone conditions covering common
    //                  kinds (quest completed / has item /
    //                  min level / class match).
    //   makeGated   - 5 conditions in 2 groups demonstrating
    //                  AND aggregation (group 100: alliance
    //                  AND mage AND lvl 60+) and OR
    //                  aggregation (group 200: completed
    //                  quest 1 OR completed quest 2).
    //   makeEvent   - 3 event-gated conditions (Brewfest /
    //                  Hallow's End / Winter's Veil)
    //                  cross-referencing WSEA event IDs.
    static WoweeCondition makeStarter(const std::string& catalogName);
    static WoweeCondition makeGated(const std::string& catalogName);
    static WoweeCondition makeEvent(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
