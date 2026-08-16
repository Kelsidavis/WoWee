#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Reputation Reward Tier catalog (.wrpr) -
// novel replacement for the implicit reputation-tier
// rules vanilla WoW encoded across multiple SQL tables
// (npc_vendor with reqstanding columns, item_template
// AllowableRace/Class plus PaperDoll faction gates,
// quest_template ReqMinRepFaction). Each entry binds
// one (factionId, minStanding) tier to its rewards: a
// vendor discount percentage, a list of unlocked item
// IDs, a list of unlocked recipe IDs, and tabard +
// mount unlock flags.
//
// First catalog with TWO variable-length payload arrays
// per entry (unlockedItemIds + unlockedRecipeIds) -
// previous variable-length formats used a single array
// (WCMR waypoints, WCMG members, WPTT spellIdsByRank).
//
// Cross-references with previously-added formats:
//   WFAC: factionId references the WFAC faction catalog.
//   WIT:  unlockedItemIds entries reference the WIT
//         item catalog (gear, consumables, mounts).
//   WTSK: unlockedRecipeIds entries reference the WTSK
//         trade-skill recipe catalog.
//   WTBD: when grantsTabard=1, the faction tabard
//         becomes purchasable (per-faction tabardId
//         lookup deferred to the WTBD catalog at runtime).
//
// Binary layout (little-endian):
//   magic[4]            = "WRPR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tierId (uint32)
//     nameLen + name
//     descLen + description
//     factionId (uint32)
//     minStanding (int32)            - Hated -42000 to
//                                       Exalted +42000
//     discountPct (uint8)            - 0..20 vendor
//                                       discount tier
//     grantsTabard (uint8)           - 0/1 bool
//     grantsMount (uint8)            - 0/1 bool
//     pad0 (uint8)
//     iconColorRGBA (uint32)
//     unlockedItemCount (uint32)
//     unlockedItemIds (count × uint32)
//     unlockedRecipeCount (uint32)
//     unlockedRecipeIds (count × uint32)
struct WoweeReputationRewards {
    struct Entry {
        uint32_t tierId = 0;
        std::string name;
        std::string description;
        uint32_t factionId = 0;
        int32_t minStanding = 0;
        uint8_t discountPct = 0;
        uint8_t grantsTabard = 0;
        uint8_t grantsMount = 0;
        uint8_t pad0 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
        std::vector<uint32_t> unlockedItemIds;
        std::vector<uint32_t> unlockedRecipeIds;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tierId) const;

    // Returns the tier for a given (faction, current
    // standing). Picks the highest tier whose
    // minStanding the player meets. Used by the vendor
    // UI to compute "at what discount can I buy from
    // this NPC?" without scanning the catalog.
    [[nodiscard]] const Entry* findActiveTierFor(uint32_t factionId,
                                     int32_t currentStanding) const;

    // Returns all tiers for a faction in ascending
    // standing order. Used by the achievement / unlock
    // preview UI ("what do I get at Revered?").
    [[nodiscard]] std::vector<const Entry*> findByFaction(uint32_t factionId) const;
};

class WoweeReputationRewardsLoader {
public:
    static bool save(const WoweeReputationRewards& cat,
                     const std::string& basePath);
    static WoweeReputationRewards load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-rpr* variants.
    //
    //   makeArgentCrusade - 4 tiers (Friendly 3000 /
    //                        Honored 9000 / Revered 21000
    //                        / Exalted 42000) with
    //                        progressive item + recipe
    //                        unlocks plus tabard at
    //                        Friendly and mount at
    //                        Exalted.
    //   makeKaluak        - 4 fishing-themed tiers for
    //                        Kalu'ak faction, progressive
    //                        cooking recipe unlocks.
    //   makeAccordTabard  - 3 tiers showcasing the
    //                        grantsTabard + grantsMount
    //                        flags (Wyrmrest Accord
    //                        Honored item / Revered
    //                        tabard / Exalted Red Drake
    //                        mount).
    static WoweeReputationRewards makeArgentCrusade(const std::string& catalogName);
    static WoweeReputationRewards makeKaluak(const std::string& catalogName);
    static WoweeReputationRewards makeAccordTabard(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
