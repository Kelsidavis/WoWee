#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Range Index catalog (.wsrg) - novel
// replacement for Blizzard's SpellRange.dbc plus the per-
// spell range-bucket fields in Spell.dbc. Defines the
// categorical range buckets that spells use ("Combat Range"
// 0-5y for melee, "Long Range" 0-40y for ranged casts,
// "Vision Range" 0-100y for area effects).
//
// Each spell references a rangeId here rather than carrying
// its own min/max yards. This lets the engine share range
// metadata across thousands of spells (every Frostbolt
// references the same 30y bucket) and lets the UI draw
// consistent range indicators (color-coded per-bucket).
//
// Friendly vs hostile range can differ - Heal might reach
// 40y on allies but Smite only 30y on enemies - so each
// entry carries separate min/max pairs for each affiliation.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the spell
//   engine and HUD. WSPL spell entries reference rangeId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSRG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     rangeId (uint32)
//     nameLen + name
//     descLen + description
//     rangeKind (uint8) / pad[3]
//     minRange (float)
//     maxRange (float)
//     minRangeFriendly (float)
//     maxRangeFriendly (float)
//     iconColorRGBA (uint32)
struct WoweeSpellRange {
    enum RangeKind : uint8_t {
        Self        = 0,    // 0-0 yards (caster only)
        Melee       = 1,    // 0-5 yards (white attack range)
        ShortRanged = 2,    // 0-20 yards (close-quarters spell)
        Ranged      = 3,    // 0-30 yards (standard cast)
        LongRanged  = 4,    // 0-40 yards (rifle / long-cast spell)
        VeryLong    = 5,    // 0-100 yards (vision / aura range)
        Unlimited   = 6,    // any range (server-tracked global)
    };

    struct Entry {
        uint32_t rangeId = 0;
        std::string name;
        std::string description;
        uint8_t rangeKind = Ranged;
        float minRange = 0.0f;
        float maxRange = 30.0f;
        float minRangeFriendly = 0.0f;
        float maxRangeFriendly = 30.0f;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t rangeId) const;

    static const char* rangeKindName(uint8_t k);
};

class WoweeSpellRangeLoader {
public:
    static bool save(const WoweeSpellRange& cat,
                     const std::string& basePath);
    static WoweeSpellRange load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-srg* variants.
    //
    //   makeStarter    - 3 baseline buckets (Self 0-0,
    //                     Melee 0-5, Spell 0-30) covering
    //                     the most common range categories.
    //   makeRanged     - 5 ranged spell buckets (Short 0-20,
    //                     Medium 0-30, Long 0-40, VeryLong
    //                     0-100, Unlimited) for varied
    //                     ranged-class spell ranges.
    //   makeFriendly   - 3 buckets where friendly-target
    //                     range exceeds hostile-target range
    //                     (Heal 40y friendly / 0 hostile,
    //                     Cleanse 30y friendly / 0 hostile,
    //                     Buff 30y friendly / 0 hostile).
    static WoweeSpellRange makeStarter(const std::string& catalogName);
    static WoweeSpellRange makeRanged(const std::string& catalogName);
    static WoweeSpellRange makeFriendly(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
