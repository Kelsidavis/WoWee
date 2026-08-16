#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Loot Mode Policy catalog (.wlma) - novel
// replacement for the implicit loot-distribution rules
// vanilla WoW encoded across the GroupLoot system
// (CMSG_LOOT_METHOD), the per-quality thresholds for
// Need-roll triggering, and the master-looter
// permission gates. Each entry binds one group-loot
// policy mode to its kind (FFA / RoundRobin / Master
// Loot / Need-Before-Greed / Personal / Disenchant)
// plus quality threshold and master-looter requirement.
//
// Cross-references with previously-added formats:
//   WIQR: thresholdQuality references the WIQR item-
//         quality tier catalog (0=Poor, 1=Common,
//         2=Uncommon, 3=Rare, 4=Epic, 5=Legendary,
//         6=Artifact, 7=Heirloom).
//
// Binary layout (little-endian):
//   magic[4]            = "WLMA"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     modeId (uint32)
//     nameLen + name
//     descLen + description
//     modeKind (uint8)           - FFA / RoundRobin /
//                                   MasterLoot /
//                                   NeedBeforeGreed /
//                                   Personal /
//                                   Disenchant
//     thresholdQuality (uint8)   - 0..7 quality tier;
//                                   loot at or above
//                                   triggers special-
//                                   roll behavior
//     masterLooterRequired (uint8) - 0/1 bool
//     idleSkipSec (uint8)        - 0..255 sec; 0 = no
//                                   idle skip; round-
//                                   robin advances if
//                                   current pick is
//                                   AFK > N seconds
//     timeoutFallbackKind (uint8) - fallback mode if
//                                   master looter
//                                   disconnects
//                                   mid-distribution
//     pad0 / pad1 / pad2 (uint8)
//     iconColorRGBA (uint32)
struct WoweeLootModes {
    enum ModeKind : uint8_t {
        FreeForAll       = 0,    // first-click wins
        RoundRobin       = 1,    // rotating per-player
                                  // assignment
        MasterLoot       = 2,    // group leader
                                  // distributes
        NeedBeforeGreed  = 3,    // roll Need (gear) /
                                  // Greed (vendor) /
                                  // Pass per drop
        Personal         = 4,    // each player gets
                                  // their own roll
        Disenchant       = 5,    // bundles with
                                  // Need/Greed adding
                                  // Disenchant button
                                  // for enchanters
    };

    struct Entry {
        uint32_t modeId = 0;
        std::string name;
        std::string description;
        uint8_t modeKind = FreeForAll;
        uint8_t thresholdQuality = 2;     // Uncommon
                                           // default
        uint8_t masterLooterRequired = 0;
        uint8_t idleSkipSec = 0;
        uint8_t timeoutFallbackKind = FreeForAll;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t modeId) const;

    // Returns all modes of one kind. Used by the loot-
    // policy UI to populate per-kind preset selectors
    // (the Master Loot section, the Need-Before-Greed
    // tier picker, etc.).
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t modeKind) const;
};

class WoweeLootModesLoader {
public:
    static bool save(const WoweeLootModes& cat,
                     const std::string& basePath);
    static WoweeLootModes load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-lma* variants.
    //
    //   makeStandard      - 4 standard 5-man / casual
    //                        modes (FreeForAll farming /
    //                        RoundRobin trash / NeedBefore-
    //                        Greed Uncommon / MasterLoot
    //                        Rare).
    //   makeRaidPolicies  - 3 raid loot policies
    //                        (MasterLoot Epic threshold /
    //                        Personal Loot / NeedBefore-
    //                        Greed Rare default).
    //   makeAFKPrevention - 3 AFK-mitigating modes
    //                        (RoundRobin idle-skip 30s /
    //                        MasterLoot 60s timeout fall-
    //                        back to NeedBeforeGreed /
    //                        Personal idle-skip 45s).
    static WoweeLootModes makeStandard(const std::string& catalogName);
    static WoweeLootModes makeRaidPolicies(const std::string& catalogName);
    static WoweeLootModes makeAFKPrevention(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
