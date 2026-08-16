#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open PvP Honor / Rank catalog (.wpvp) - novel
// replacement for the AzerothCore-style PvP rank /
// arena-tier tables plus the vanilla honor-rank reward
// chains. The 60th open format added to the editor.
//
// Defines PvP progression rungs: vanilla honor ranks
// (Private through Grand Marshal / High Warlord), arena
// rating brackets (Combatant / Challenger / Rival /
// Duelist / Gladiator), and battleground rated tiers.
// Each entry has alliance / horde alternate names, an
// honor or rating threshold, an optional title award
// (cross-ref WTTL), and gear cross-refs (chest, gloves,
// shoulders) into WIT for the matching PvP set.
//
// Cross-references with previously-added formats:
//   WPVP.entry.titleId        → WTTL.titleId
//   WPVP.entry.chestItemId    → WIT.itemId
//   WPVP.entry.glovesItemId   → WIT.itemId
//   WPVP.entry.shouldersItemId→ WIT.itemId
//   WPVP.entry.bracketBgId    → WBGD.bgId (battleground-
//                                bracket gating)
//
// Binary layout (little-endian):
//   magic[4]            = "WPVP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     rankId (uint32)
//     nameLen + name
//     allyLen + factionAllianceName
//     hordeLen + factionHordeName
//     descLen + description
//     rankKind (uint8) / minBracketLevel (uint8) /
//       maxBracketLevel (uint8) / pad[1]
//     minHonorOrRating (uint32)
//     rewardEmblems (uint16) / pad[2]
//     titleId (uint32)
//     chestItemId (uint32)
//     glovesItemId (uint32)
//     shouldersItemId (uint32)
//     bracketBgId (uint32)
struct WoweePVPRank {
    enum RankKind : uint8_t {
        VanillaHonor   = 0,    // Private / Knight / etc - uses
                                // minHonor (kill points)
        ArenaRating    = 1,    // 1500-2400+ rating-based
        BattlegroundRated = 2, // 10v10 rated BG bracket
        WorldPvP       = 3,    // Wintergrasp / Tol Barad rank
        ConquestPoint  = 4,    // currency-based threshold
    };

    struct Entry {
        uint32_t rankId = 0;
        std::string name;
        std::string factionAllianceName;   // alt name for Alliance
        std::string factionHordeName;      // alt name for Horde
        std::string description;
        uint8_t rankKind = VanillaHonor;
        uint8_t minBracketLevel = 1;       // for level bracket gating
        uint8_t maxBracketLevel = 80;
        uint32_t minHonorOrRating = 0;     // kills or rating points
        uint16_t rewardEmblems = 0;        // bonus emblem currency
        uint32_t titleId = 0;              // WTTL cross-ref
        uint32_t chestItemId = 0;          // WIT cross-ref
        uint32_t glovesItemId = 0;         // WIT cross-ref
        uint32_t shouldersItemId = 0;      // WIT cross-ref
        uint32_t bracketBgId = 0;          // WBGD cross-ref (optional)
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t rankId) const;

    static const char* rankKindName(uint8_t k);
};

class WoweePVPRankLoader {
public:
    static bool save(const WoweePVPRank& cat,
                     const std::string& basePath);
    static WoweePVPRank load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-pvp* variants.
    //
    //   makeStarter     - 3 vanilla honor entry tiers
    //                      (Private/Knight, Sergeant/Stone
    //                      Guard, Knight-Lieutenant/Blood
    //                      Guard) showing the alliance-vs-
    //                      horde alternate-name pattern.
    //   makeAllianceFull - 7 alliance vanilla ranks (R6-R14:
    //                       Knight-Captain through Grand
    //                       Marshal) with chest/gloves/
    //                       shoulders cross-refs into WIT.
    //   makeArenaTiers  - 5 arena rating brackets
    //                      (Combatant 1500 / Challenger 1750
    //                      / Rival 2000 / Duelist 2200 /
    //                      Gladiator 2400) with title +
    //                      emblem rewards.
    static WoweePVPRank makeStarter(const std::string& catalogName);
    static WoweePVPRank makeAllianceFull(const std::string& catalogName);
    static WoweePVPRank makeArenaTiers(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
