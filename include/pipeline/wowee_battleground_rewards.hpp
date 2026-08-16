#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Battleground Reward Stages catalog
// (.wbrd) - novel replacement for the per-BG per-
// bracket reward configuration vanilla WoW carried
// in BattlemasterList.dbc + the hard-coded honor
// table in the server's BattlegroundMgr (the
// "Mark of Honor" item granted on win/loss was
// hard-coded per-BG type with no formal data-
// driven scaling). Each WBRD entry binds one
// (battlegroundId, levelBracket) pair to its win/
// loss honor amounts, win/loss marks, the mark
// itemId, an optional weekly-bonus item, and a
// minimum-players-to-start gate.
//
// Cross-references with previously-added formats:
//   WBGD: battlegroundId references the existing
//         WBGD battleground catalog (1=AV, 2=WSG,
//         3=AB).
//   WIT:  markItemId and bonusItemId both reference
//         the WIT item catalog (Mark of Honor IDs
//         are 17502 AV / 20558 WSG / 20559 AB).
//
// Binary layout (little-endian):
//   magic[4]            = "WBRD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     rewardId (uint32)
//     battlegroundId (uint16)      - WBGD ref
//                                     (1=AV/2=WSG/
//                                     3=AB)
//     bracketIndex (uint8)         - 1=10-19 /
//                                     2=20-29 / ...
//                                     / 6=60-69
//     minPlayersToStart (uint8)    - AV=20 / WSG=10
//                                     / AB=15 etc.
//     winHonor (uint32)
//     lossHonor (uint32)
//     markItemId (uint32)          - WIT ref
//     winMarks (uint16)
//     lossMarks (uint16)
//     bonusItemId (uint32)         - weekly bonus
//                                     quest token
//                                     (0 = no bonus)
//     bonusItemCount (uint16)
//     pad0 (uint16)
struct WoweeBattlegroundRewards {
    static constexpr uint8_t kMaxBracketIndex = 6;

    struct Entry {
        uint32_t rewardId = 0;
        uint16_t battlegroundId = 0;
        uint8_t bracketIndex = 0;
        uint8_t minPlayersToStart = 0;
        uint32_t winHonor = 0;
        uint32_t lossHonor = 0;
        uint32_t markItemId = 0;
        uint16_t winMarks = 0;
        uint16_t lossMarks = 0;
        uint32_t bonusItemId = 0;
        uint16_t bonusItemCount = 0;
        uint16_t pad0 = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t rewardId) const;

    // Resolve the reward stage for a (BG, level
    // bracket) pair - the canonical lookup the
    // post-match handler uses to credit honor +
    // marks to each participant.
    [[nodiscard]] const Entry* find(uint16_t bgId,
                       uint8_t bracketIndex) const;

    // Returns all reward entries for a single
    // battleground - used by the BG queue UI to
    // show per-bracket reward previews.
    [[nodiscard]] std::vector<const Entry*> findByBg(uint16_t bgId) const;
};

class WoweeBattlegroundRewardsLoader {
public:
    static bool save(const WoweeBattlegroundRewards& cat,
                     const std::string& basePath);
    static WoweeBattlegroundRewards load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-brd* variants.
    //
    //   makeAlteracValley - AV (bgId=1) reward
    //                        ladder for brackets 5
    //                        and 6 (51-60 / 61-69).
    //                        AV requires 20 players
    //                        per side. High honor +
    //                        Mark of AV (itemId
    //                        17502).
    //   makeWarsong       - WSG (bgId=2) reward
    //                        ladder for brackets
    //                        1..6 (10-19 through
    //                        61-69). WSG min 10
    //                        players. Mark of WSG
    //                        (20558).
    //   makeArathiBasin   - AB (bgId=3) reward
    //                        ladder for brackets 2..6
    //                        (20-29 through 61-69).
    //                        Min 15 players. Mark of
    //                        AB (20559) + weekly
    //                        bonus quest token.
    static WoweeBattlegroundRewards makeAlteracValley(const std::string& catalogName);
    static WoweeBattlegroundRewards makeWarsong(const std::string& catalogName);
    static WoweeBattlegroundRewards makeArathiBasin(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
