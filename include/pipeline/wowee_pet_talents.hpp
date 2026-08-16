#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Pet Talent Tree catalog (.wptt) - novel
// replacement for the PetTalent.dbc + PetTalentTab.dbc
// pair that defined the Hunter pet talent system added
// in WotLK. Each entry is one talent in one of the three
// pet trees (Cunning / Ferocity / Tenacity), placed at a
// (tier, column) grid position with a per-rank spell ID
// array, an optional prerequisite-talent edge, and a
// legacy loyalty-level requirement carried over from
// Vanilla pet happiness mechanics.
//
// Combines three patterns previously seen separately:
//   - variable-length payload (spellIdsByRank[], like
//     WCMR's members[])
//   - graph edge (prerequisiteTalentId, like WBAB's
//     previousRankId)
//   - grid placement (tier+column, novel - first format
//     with explicit 2D layout coordinates)
//
// Cross-references with previously-added formats:
//   WSPL: spellIdsByRank entries reference the WSPL
//         spell catalog.
//   WPTT: prerequisiteTalentId references ANOTHER entry
//         in the same WPTT catalog - internal graph edge.
//   WPET: pet families that can train this tree are
//         filtered via the WPET catalog's treeKind field
//         (lookup external).
//
// Binary layout (little-endian):
//   magic[4]            = "WPTT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     talentId (uint32)
//     nameLen + name
//     descLen + description
//     treeKind (uint8)            - Cunning / Ferocity /
//                                    Tenacity
//     tier (uint8)                - 0..6 (7 tiers in
//                                    standard pet tree)
//     column (uint8)              - 0..2 (3 columns per
//                                    tier)
//     maxRank (uint8)             - typically 1, 2, 3, or
//                                    5 talent points
//     prerequisiteTalentId (uint32) - 0 if no prereq
//     requiredLoyalty (uint8)     - Vanilla loyalty
//                                    (0 = always);
//                                    cosmetic in WotLK+
//     pad0 (uint8) / pad1 (uint8) / pad2 (uint8)
//     iconColorRGBA (uint32)
//     spellCountByRank (uint32)   - = maxRank, captured
//                                    explicitly so reader
//                                    can validate the
//                                    array size matches
//     spellIdsByRank (count × uint32)
struct WoweePetTalents {
    enum TreeKind : uint8_t {
        Cunning  = 0,    // utility - Roar of Recovery,
                          // Dash, Heart of the Phoenix
        Ferocity = 1,    // damage  - Cobra Reflexes,
                          // Spiked Collar, Rabid
        Tenacity = 2,    // tank    - Charge, Thunder-
                          // stomp, Last Stand
    };

    struct Entry {
        uint32_t talentId = 0;
        std::string name;
        std::string description;
        uint8_t treeKind = Cunning;
        uint8_t tier = 0;
        uint8_t column = 0;
        uint8_t maxRank = 1;
        uint32_t prerequisiteTalentId = 0;
        uint8_t requiredLoyalty = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
        std::vector<uint32_t> spellIdsByRank;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t talentId) const;

    // Returns all talents in one tree (used by the pet
    // talent UI to populate the tree-switching tabs).
    [[nodiscard]] std::vector<const Entry*> findByTree(uint8_t treeKind) const;

    // Returns the talent (if any) at the given (tier,
    // column) of the given tree. Used by the talent grid
    // renderer to look up "what occupies this cell?"
    [[nodiscard]] const Entry* findAtCell(uint8_t treeKind, uint8_t tier,
                             uint8_t column) const;
};

class WoweePetTalentsLoader {
public:
    static bool save(const WoweePetTalents& cat,
                     const std::string& basePath);
    static WoweePetTalents load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-ptt* variants.
    //
    //   makeFerocity - 6 Ferocity (DPS) tree talents
    //                   spanning tiers 0-3 with prereq
    //                   chains (Cobra Reflexes /
    //                   Spiked Collar / Boar's Speed /
    //                   Spider's Bite / Rabid /
    //                   Wolverine Bite).
    //   makeCunning  - 5 Cunning (utility) tree talents
    //                   (Dash / Roar of Recovery /
    //                   Heart of the Phoenix / Owl's
    //                   Focus / Cornered).
    //   makeTenacity - 5 Tenacity (tank) tree talents
    //                   (Charge / Thunderstomp / Last
    //                   Stand / Taunt / Roar of
    //                   Sacrifice).
    static WoweePetTalents makeFerocity(const std::string& catalogName);
    static WoweePetTalents makeCunning(const std::string& catalogName);
    static WoweePetTalents makeTenacity(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
