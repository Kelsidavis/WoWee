#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Stat Modifier Curve catalog (.wstm) -
// novel replacement for the gtChanceTo*.dbc / gtRegen*.dbc
// / gtCombatRatings.dbc family of "1D level-keyed curve"
// tables. Each entry defines a single linear curve mapping
// character level to a stat value: melee crit chance per
// level, mana regen per spirit per level, base armor per
// level, etc.
//
// Curves are simple linear: value(level) = baseValue +
// perLevelDelta * (level - 1), with the result optionally
// scaled by a global multiplier and clamped to a level
// range. Most stock WoW curves fit this shape - the few
// that don't (Combat Ratings) live in the dedicated WCRR
// catalog with cubic spline support.
//
// Distinct from WCRR (Combat Rating conversion) which
// converts integer rating points to percentages, and
// distinct from WSPC (Spell Power Cost buckets) which
// scales per-spell costs. WSTM is for the generic
// engine-side stat curves that aren't per-spell or
// per-rating.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the
//   character stat resolver. Engines look up curves by
//   curveId or by name.
//
// Binary layout (little-endian):
//   magic[4]            = "WSTM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     curveId (uint32)
//     nameLen + name
//     descLen + description
//     curveKind (uint8) / minLevel (uint8) / maxLevel (uint8) / pad (uint8)
//     baseValue (float)
//     perLevelDelta (float)
//     multiplier (float)
//     iconColorRGBA (uint32)
struct WoweeStatCurve {
    enum CurveKind : uint8_t {
        Crit       = 0,    // crit chance scaling per level
        Hit        = 1,    // hit chance scaling
        Power      = 2,    // attack power / spell power growth
        Regen      = 3,    // mana / health regen rates
        Resist     = 4,    // resistance scaling
        Mitigation = 5,    // armor / damage reduction
        Misc       = 6,    // catch-all
    };

    struct Entry {
        uint32_t curveId = 0;
        std::string name;
        std::string description;
        uint8_t curveKind = Misc;
        uint8_t minLevel = 1;
        uint8_t maxLevel = 80;
        uint8_t pad0 = 0;
        float baseValue = 0.0f;
        float perLevelDelta = 0.0f;
        float multiplier = 1.0f;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t curveId) const;

    // Resolve the curve at the given character level,
    // applying the linear formula then scaling by the
    // multiplier and clamping to [minLevel..maxLevel].
    // Returns 0.0 if the level is below minLevel.
    [[nodiscard]] float resolveAtLevel(uint32_t curveId, uint8_t level) const;

    static const char* curveKindName(uint8_t k);
};

class WoweeStatCurveLoader {
public:
    static bool save(const WoweeStatCurve& cat,
                     const std::string& basePath);
    static WoweeStatCurve load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-stm* variants.
    //
    //   makeCrit   - 5 crit-related curves (MeleeCritChance,
    //                 RangedCritChance, SpellCritChance,
    //                 ParryChance, DodgeChance) covering
    //                 the standard crit-tier scaling.
    //   makeRegen  - 4 regen curves (ManaPerSpirit,
    //                 HpPerSpirit, EnergyPerSec, RageDecay)
    //                 with the canonical Vanilla/TBC/WotLK
    //                 stock formulas.
    //   makeArmor  - 3 armor / mitigation curves
    //                 (BaseArmorPerLevel, ArmorMitigation,
    //                 ResistancePerLevel).
    static WoweeStatCurve makeCrit(const std::string& catalogName);
    static WoweeStatCurve makeRegen(const std::string& catalogName);
    static WoweeStatCurve makeArmor(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
