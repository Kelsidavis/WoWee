#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Combat Rating Conversion catalog (.wcrr) -
// novel replacement for Blizzard's gtCombatRatings.dbc plus
// the per-level rating-to-percentage tables in
// gtRegenHPPerSpt.dbc and related stat-curve DBCs. Defines
// per-rating-type conversion factors at canonical level
// breakpoints (1 / 60 / 70 / 80) - the runtime linearly
// interpolates between breakpoints for intermediate levels.
//
// pointsAtLevelN is "how many rating points equal 1% of
// the benefit at that level." Higher level = more rating
// needed for the same %. So at level 60 you might need 14
// crit rating per 1%, but at level 80 you need 45 - the
// curve gradually requires more rating to reach the same
// percentage benefit.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the combat
//   engine's stat resolver. WSPL spell scaling and WIT item
//   stat conversion read this catalog at runtime.
//
// Binary layout (little-endian):
//   magic[4]            = "WCRR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     ratingType (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     ratingKind (uint8) / pad[3]
//     pointsAtL1 (float)
//     pointsAtL60 (float)
//     pointsAtL70 (float)
//     pointsAtL80 (float)
//     maxBenefitPercent (float)
struct WoweeCombatRating {
    enum RatingKind : uint8_t {
        Combat     = 0,    // Hit / Crit / Haste / ExpertiseRating
        Defense    = 1,    // Defense / Dodge / Parry / Block / ArmorPen
        Spell      = 2,    // SpellPower / SpellPenetration / MP5
        Resilience = 3,    // PvP damage reduction
        Other      = 4,    // misc (mining, fishing skill caps)
    };

    struct Entry {
        uint32_t ratingType = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t ratingKind = Combat;
        float pointsAtL1 = 1.0f;
        float pointsAtL60 = 14.0f;     // canonical L60 base
        float pointsAtL70 = 22.0f;     // canonical L70 base
        float pointsAtL80 = 45.0f;     // canonical L80 base
        float maxBenefitPercent = 100.0f;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t ratingType) const;

    static const char* ratingKindName(uint8_t k);
};

class WoweeCombatRatingLoader {
public:
    static bool save(const WoweeCombatRating& cat,
                     const std::string& basePath);
    static WoweeCombatRating load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-crr* variants.
    //
    //   makeStarter   - 3 essential combat ratings (Hit /
    //                    Crit / Haste) at canonical WoW
    //                    L80 conversion values.
    //   makeDefensive - 4 defensive ratings (Defense / Dodge
    //                    / Parry / Block) for tank stat
    //                    scaling.
    //   makeSpell     - 3 spell-related ratings (SpellPower
    //                    direct conversion, SpellPenetration,
    //                    MP5 mana regeneration) for caster
    //                    stat scaling.
    static WoweeCombatRating makeStarter(const std::string& catalogName);
    static WoweeCombatRating makeDefensive(const std::string& catalogName);
    static WoweeCombatRating makeSpell(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
