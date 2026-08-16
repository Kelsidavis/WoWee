#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Quest Graph catalog (.wqgr) - novel
// representation of quest-chain dependencies that
// vanilla WoW carried implicitly in
// QuestRelations.dbc (the prequest column) +
// per-quest server scripts. Each WQGR entry binds
// one quest to its display name, level/class/race
// gating, prerequisite quest list (must be
// completed first), follow-up quest hints (next-
// quest suggestions for the journal UI), and quest
// type flags (Normal / Daily / Repeatable / Group /
// Raid).
//
// The variable-length prereq array gives the
// validator something interesting to check: a DFS
// cycle detector flags player-unreachable quests
// (Q1 prereq=Q2, Q2 prereq=Q3, Q3 prereq=Q1 is a
// progression deadlock - no player could ever
// satisfy the cycle).
//
// Cross-references with previously-added formats:
//   WQTM: questId references the WQTM quest catalog
//         (the actual quest objectives + rewards
//         live in WQTM; WQGR only describes the
//         dependency graph between them).
//   WMS:  zoneId references the WMS map catalog.
//
// Binary layout (little-endian):
//   magic[4]            = "WQGR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     questId (uint32)
//     nameLen + name
//     minLevel (uint8)
//     maxLevel (uint8)         - 0 = no upper gate
//     questType (uint8)        - 0=Normal /
//                                 1=Daily /
//                                 2=Repeatable /
//                                 3=Group /
//                                 4=Raid
//     factionAccess (uint8)    - 0=Both /
//                                 1=Alliance /
//                                 2=Horde /
//                                 3=Neutral
//     classRestriction (uint16)  - bitmask of allowed
//                                   classIds (0 =
//                                   no restriction)
//     raceRestriction (uint16)   - bitmask of allowed
//                                   raceIds (0 =
//                                   no restriction)
//     zoneId (uint32)
//     chainHeadHint (uint8)    - 0/1 bool - first
//                                 quest in a chain
//                                 (UI sort hint)
//     pad0 (uint8)
//     pad1 (uint16)
//     prevCount (uint32)       - prereq array
//     prevQuestIds (uint32 × count)
//     followupCount (uint32)   - hint array
//     followupQuestIds (uint32 × count)
struct WoweeQuestGraph {
    enum QuestType : uint8_t {
        Normal     = 0,
        Daily      = 1,
        Repeatable = 2,
        Group      = 3,
        Raid       = 4,
    };

    enum FactionAccess : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
        Neutral  = 3,
    };

    struct Entry {
        uint32_t questId = 0;
        std::string name;
        uint8_t minLevel = 0;
        uint8_t maxLevel = 0;
        uint8_t questType = Normal;
        uint8_t factionAccess = Both;
        uint16_t classRestriction = 0;
        uint16_t raceRestriction = 0;
        uint32_t zoneId = 0;
        uint8_t chainHeadHint = 0;
        uint8_t pad0 = 0;
        uint16_t pad1 = 0;
        std::vector<uint32_t> prevQuestIds;
        std::vector<uint32_t> followupQuestIds;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t questId) const;

    // Returns all quests that have the given questId
    // as a prereq (the "what unlocks once I finish
    // this" lookup - used by the journal UI's
    // "completing this opens" panel).
    [[nodiscard]] std::vector<const Entry*> findUnlocksFrom(uint32_t questId) const;

    // Returns all quests in a zone - used by the
    // zone-detail UI to populate the per-zone quest
    // list.
    [[nodiscard]] std::vector<const Entry*> findByZone(uint32_t zoneId) const;
};

class WoweeQuestGraphLoader {
public:
    static bool save(const WoweeQuestGraph& cat,
                     const std::string& basePath);
    static WoweeQuestGraph load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-qgr* variants.
    //
    //   makeStarterChain  - 5-quest linear chain
    //                        (Northshire human starter
    //                        Q1->Q2->Q3->Q4->Q5).
    //                        Q1 has chainHeadHint=1.
    //                        Levels 1..5.
    //   makeBranchedChain - 4-quest converging chain
    //                        (Q1 -> Q2a, Q1 -> Q2b,
    //                        both -> Q3). Demonstrates
    //                        DAG semantics not just
    //                        a linear list.
    //   makeDailies       - 3 standalone daily quests
    //                        (Daily type, no prereqs,
    //                        no follow-ups). Baseline
    //                        empty-deps path.
    static WoweeQuestGraph makeStarterChain(const std::string& catalogName);
    static WoweeQuestGraph makeBranchedChain(const std::string& catalogName);
    static WoweeQuestGraph makeDailies(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
