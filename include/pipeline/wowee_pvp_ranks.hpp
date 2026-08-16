#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open PvP Ranking Grades catalog (.wprg) -
// novel replacement for the hardcoded 14-rank PvP
// ladder vanilla WoW shipped (Private through Grand
// Marshal for Alliance, Scout through High Warlord for
// Horde). Each entry binds one (factionFilter, tier)
// combination to its display name, weekly RP threshold
// to maintain rank, lifetime honor for first-time
// achievement, title prefix, and tier-set gear reward.
//
// Cross-references with previously-added formats:
//   WCHC: factionFilter uses the WCHC faction-mask
//         convention (1=Alliance, 2=Horde).
//   WIT:  gearItemId references the WIT item catalog
//         (the rank-tier set piece - typically the
//         legendary battlegear shoulders unlocked at
//         high ranks).
//   WPVP: WPRG supersedes the older WPVP simpler PvP
//         currency catalog where rank-progression
//         semantics matter.
//
// Binary layout (little-endian):
//   magic[4]            = "WPRG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     rankId (uint32)
//     nameLen + name
//     descLen + description
//     factionFilter (uint8)      - 1=Alliance, 2=Horde
//     tier (uint8)               - 1..14 vanilla rank
//                                   ladder
//     pad0 (uint8) / pad1 (uint8)
//     honorRequiredWeekly (uint32) - RP threshold per
//                                     week to maintain
//     honorRequiredAchieve (uint32) - total RP for
//                                     first-time
//                                     achievement
//     prefixLen + titlePrefix      - e.g. "Sergeant"
//     gearItemId (uint32)          - 0 if no gear
//                                     reward at this
//                                     tier
//     iconColorRGBA (uint32)
struct WoweePvPRanks {
    enum FactionFilter : uint8_t {
        AllianceOnly = 1,
        HordeOnly    = 2,
    };

    struct Entry {
        uint32_t rankId = 0;
        std::string name;
        std::string description;
        uint8_t factionFilter = AllianceOnly;
        uint8_t tier = 1;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t honorRequiredWeekly = 0;
        uint32_t honorRequiredAchieve = 0;
        std::string titlePrefix;
        uint32_t gearItemId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t rankId) const;

    // Returns all entries for one faction sorted by
    // tier. Used by the rank-progression UI to render
    // the ladder.
    [[nodiscard]] std::vector<const Entry*> findByFaction(uint8_t faction) const;

    // Returns the entry for a specific (faction, tier)
    // combination. Used by the weekly-honor processor
    // to look up "what's the threshold for tier 7?"
    [[nodiscard]] const Entry* findByTier(uint8_t faction,
                              uint8_t tier) const;
};

class WoweePvPRanksLoader {
public:
    static bool save(const WoweePvPRanks& cat,
                     const std::string& basePath);
    static WoweePvPRanks load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-prg* variants.
    //
    //   makeAllianceRanks - 7 lower-tier Alliance ranks
    //                        (Private through Knight-
    //                        Lieutenant, tiers 1-7).
    //   makeHordeRanks    - 7 lower-tier Horde ranks
    //                        (Scout through Blood Guard,
    //                        tiers 1-7) with mirrored
    //                        honor thresholds.
    //   makeHighRanks     - 4 high-tier Alliance ranks
    //                        (Knight-Captain through
    //                        Commander, tiers 8-11).
    //                        Plus 4 mirrored Horde
    //                        (Legionnaire through Lt.
    //                        Commander).
    static WoweePvPRanks makeAllianceRanks(const std::string& catalogName);
    static WoweePvPRanks makeHordeRanks(const std::string& catalogName);
    static WoweePvPRanks makeHighRanks(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
