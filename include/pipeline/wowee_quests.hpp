#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Quest Template (.wqt) - novel replacement for
// AzerothCore-style quest_template SQL tables PLUS the
// Blizzard Quest.dbc / QuestObjective.dbc trio. The 15th
// open format added to the editor.
//
// Cross-references with previously-added formats:
//   WQT.giverCreatureId    → WCRT.entry.creatureId
//   WQT.turninCreatureId   → WCRT.entry.creatureId
//   WQT.objective.targetId → WCRT (kill) / WIT (collect) / WOB (interact)
//   WQT.rewardItem.itemId  → WIT.entry.itemId
//   WQT.prevQuestId        → WQT.entry.questId    (intra-format chain)
//   WQT.nextQuestId        → WQT.entry.questId
//
// Together with WIT / WCRT / WLOT / WSPN this completes the
// gameplay graph: a content pack can ship items + creatures
// + spawns + loot tables + quests with no SQL or .dbc files.
//
// Binary layout (little-endian):
//   magic[4]            = "WQTM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     questId (uint32)
//     titleLen + title
//     objectiveLen + objective         -- 1-line "Kill 10 wolves"
//     descriptionLen + description     -- flavor text
//     minLevel (uint16) / questLevel (uint16) / maxLevel (uint16) / pad
//     requiredClassMask (uint32)       -- bitmask of class IDs; 0 = any
//     requiredRaceMask (uint32)
//     prevQuestId (uint32)             -- 0 = chain start
//     nextQuestId (uint32)
//     giverCreatureId (uint32)
//     turninCreatureId (uint32)
//     objectiveCount (uint8) + pad[3]
//     objectives (objectiveCount × {
//         kind (uint8) + pad[3]
//         targetId (uint32)
//         quantity (uint16) + pad[2]
//     })
//     xpReward (uint32)
//     moneyCopperReward (uint32)
//     rewardItemCount (uint8) + pad[3]
//     rewardItems (rewardItemCount × {
//         itemId (uint32)
//         qty (uint8) + pickFlags (uint8) + pad[2]
//     })
//     flags (uint32)
struct WoweeQuest {
    enum ObjectiveKind : uint8_t {
        KillCreature   = 0,
        CollectItem    = 1,
        InteractObject = 2,
        VisitArea      = 3,
        EscortNpc      = 4,
        SpellCast      = 5,
    };

    enum Flags : uint32_t {
        Daily         = 0x01,
        Weekly        = 0x02,
        Raid          = 0x04,
        Group         = 0x08,    // requires party / not solo-completable
        AutoComplete  = 0x10,    // turns in immediately on objective met
        AutoAccept    = 0x20,    // accepts on giver-NPC interaction
        Repeatable    = 0x40,
        ClassQuest    = 0x80,
        Pvp           = 0x100,
    };

    enum RewardPickFlags : uint8_t {
        AutoGiven   = 0x01,    // always handed out on turn-in
        PlayerChoice = 0x02,   // player picks from a list at turn-in
    };

    struct Objective {
        uint8_t kind = KillCreature;
        uint32_t targetId = 0;
        uint16_t quantity = 1;
    };

    struct RewardItem {
        uint32_t itemId = 0;
        uint8_t qty = 1;
        uint8_t pickFlags = AutoGiven;
    };

    struct Entry {
        uint32_t questId = 0;
        std::string title;
        std::string objective;
        std::string description;
        uint16_t minLevel = 1;
        uint16_t questLevel = 1;
        uint16_t maxLevel = 0;          // 0 = no upper cap
        uint32_t requiredClassMask = 0;
        uint32_t requiredRaceMask = 0;
        uint32_t prevQuestId = 0;
        uint32_t nextQuestId = 0;
        uint32_t giverCreatureId = 0;
        uint32_t turninCreatureId = 0;
        std::vector<Objective> objectives;
        uint32_t xpReward = 0;
        uint32_t moneyCopperReward = 0;
        std::vector<RewardItem> rewardItems;
        uint32_t flags = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by questId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t questId) const;

    static const char* objectiveKindName(uint8_t k);
};

class WoweeQuestLoader {
public:
    static bool save(const WoweeQuest& cat,
                     const std::string& basePath);
    static WoweeQuest load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-quests* variants.
    //
    //   makeStarter - 1 short kill quest: "Kill 10 bandits" with
    //                  XP + small money reward. Cross-references
    //                  WCRT bandit (creatureId=1000) + WCRT
    //                  village innkeeper (giver/turnin=4001).
    //   makeChain   - 3-quest chain: "Investigate" -> "Recover"
    //                  -> "Report Back". Each quest's nextQuestId
    //                  points to the next; the third closes the loop.
    //   makeDaily   - 1 daily repeatable quest with the Daily +
    //                  Repeatable + AutoAccept flag combo.
    static WoweeQuest makeStarter(const std::string& catalogName);
    static WoweeQuest makeChain(const std::string& catalogName);
    static WoweeQuest makeDaily(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
