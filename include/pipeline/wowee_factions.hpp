#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Faction Catalog (.wfac) - novel replacement for
// Blizzard's Faction.dbc + FactionTemplate.dbc + the
// AzerothCore-style reputation_reward / reputation_spillover
// SQL tables. The 17th open format added to the editor.
//
// Combines the "displayable Faction" (player-facing name,
// reputation thresholds for friendly/honored/revered/exalted)
// with the "FactionTemplate matrix" (which factions are
// hostile to which) into one entry. The runtime walks the
// catalog to answer two questions:
//   • "Will faction A attack faction B on sight?"  -> enemy list
//   • "What reputation tier is this player with X?" -> thresholds
//
// Cross-references:
//   WCRT.entry.factionId       -> WFAC.entry.factionId
//   WFAC.entry.parentFactionId -> WFAC.entry.factionId
//                                  (intra-format hierarchy)
//   WFAC.entry.enemies[]       -> WFAC.entry.factionId
//   WFAC.entry.friends[]       -> WFAC.entry.factionId
//
// Binary layout (little-endian):
//   magic[4]            = "WFAC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     factionId (uint32)
//     parentFactionId (uint32)
//     nameLen + name
//     descLen + description
//     reputationFlags (uint32)
//     baseReputation (int32)
//     thresholdHostile (int32)
//     thresholdUnfriendly (int32)
//     thresholdNeutral (int32)
//     thresholdFriendly (int32)
//     thresholdHonored (int32)
//     thresholdRevered (int32)
//     thresholdExalted (int32)
//     enemyCount (uint8) + pad[3] + enemies[] (factionId × N)
//     friendCount (uint8) + pad[3] + friends[] (factionId × N)
struct WoweeFaction {
    enum ReputationFlags : uint32_t {
        VisibleOnTab    = 0x01,    // shows up in the player's reputation panel
        AtWarDefault    = 0x02,    // new players are at-war on first contact
        Hidden          = 0x04,    // never shown (internal reputation tracking)
        NoReputation    = 0x08,    // gives no reputation gains/losses
        IsHeader        = 0x10,    // grouping header (parent), not a faction itself
    };

    // Canonical reputation tiers - clients use the threshold
    // values to decide which tier badge to display.
    enum Tier : int32_t {
        Hated      = -42000,
        Hostile    = -6000,
        Unfriendly = -3000,
        Neutral    = 0,
        Friendly   = 3000,
        Honored    = 9000,
        Revered    = 21000,
        Exalted    = 42000,
    };

    struct Entry {
        uint32_t factionId = 0;
        uint32_t parentFactionId = 0;
        std::string name;
        std::string description;
        uint32_t reputationFlags = VisibleOnTab;
        int32_t baseReputation = 0;
        int32_t thresholdHostile = Hostile;
        int32_t thresholdUnfriendly = Unfriendly;
        int32_t thresholdNeutral = Neutral;
        int32_t thresholdFriendly = Friendly;
        int32_t thresholdHonored = Honored;
        int32_t thresholdRevered = Revered;
        int32_t thresholdExalted = Exalted;
        std::vector<uint32_t> enemies;
        std::vector<uint32_t> friends;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by factionId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t factionId) const;

    // True if A's enemies list contains B (hostile-on-sight).
    // Does NOT walk parent factions; use isAtWarTransitive
    // for that.
    [[nodiscard]] bool isHostile(uint32_t aFactionId, uint32_t bFactionId) const;
};

class WoweeFactionLoader {
public:
    static bool save(const WoweeFaction& cat,
                     const std::string& basePath);
    static WoweeFaction load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-factions* variants.
    //
    //   makeStarter - 3 factions: Friendly (35), Hostile (14),
    //                  PlayerHorde (1). Friendly is enemies of
    //                  Hostile, vice versa. Useful as a starter
    //                  template.
    //   makeAlliance - Stormwind / Ironforge / Darnassus (with
    //                   reciprocal friend lists) + the canonical
    //                   Defias enemy.
    //   makeWildlife - neutral wildlife factions: wolves, bears,
    //                   spiders, kobolds. Each hostile to players
    //                   but not to each other.
    static WoweeFaction makeStarter(const std::string& catalogName);
    static WoweeFaction makeAlliance(const std::string& catalogName);
    static WoweeFaction makeWildlife(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
