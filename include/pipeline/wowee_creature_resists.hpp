#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Resistance & Immunity catalog
// (.wcre) - novel replacement for the per-creature
// resistance columns that vanilla WoW buried inside
// creature_template (resistance1..6 fields) plus the
// SpellSchoolMask immunity / mechanic_immune_mask
// columns. Each entry is one creature's full resistance
// + immunity profile: 6 magic-school resist values, a
// physical-resistance percentage, plus three immunity
// bitmasks (crowd-control kinds, spell mechanics, magic
// schools).
//
// Cross-references with previously-added formats:
//   WCRT: creatureEntry references the WCRT creature
//         catalog. One WCRE row per WCRT entry that
//         needs non-default resistances; absence means
//         the creature uses default zero resistances.
//   WSCH: schoolImmunityMask uses the WSCH school-bit
//         convention.
//
// Binary layout (little-endian):
//   magic[4]            = "WCRE"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     resistId (uint32)
//     nameLen + name
//     descLen + description
//     creatureEntry (uint32)
//     holyResist (int16) / fireResist (int16)
//     natureResist (int16) / frostResist (int16)
//     shadowResist (int16) / arcaneResist (int16)
//     physicalResistPct (uint8)   - 0..75 (cap is 75%)
//     pad0 (uint8)
//     ccImmunityMask (uint16)     - Stun/Root/Fear/etc.
//     mechanicImmunityMask (uint32)  - full mechanic
//                                     bitmask
//     schoolImmunityMask (uint8)  - WSCH-bit immunity
//     pad1 (uint8) / pad2 (uint8) / pad3 (uint8)
//     iconColorRGBA (uint32)
struct WoweeCreatureResists {
    enum CCImmunityBit : uint16_t {
        ImmuneRoot       = 0x0001,
        ImmuneSnare      = 0x0002,
        ImmuneStun       = 0x0004,
        ImmuneFear       = 0x0008,
        ImmuneSleep      = 0x0010,
        ImmuneSilence    = 0x0020,
        ImmuneCharm      = 0x0040,
        ImmuneDisarm     = 0x0080,
        ImmunePolymorph  = 0x0100,
        ImmuneBanish     = 0x0200,
        ImmuneKnockback  = 0x0400,
        ImmuneInterrupt  = 0x0800,
        ImmuneTaunt      = 0x1000,
        ImmuneBleed      = 0x2000,
    };

    struct Entry {
        uint32_t resistId = 0;
        std::string name;
        std::string description;
        uint32_t creatureEntry = 0;
        int16_t holyResist = 0;
        int16_t fireResist = 0;
        int16_t natureResist = 0;
        int16_t frostResist = 0;
        int16_t shadowResist = 0;
        int16_t arcaneResist = 0;
        uint8_t physicalResistPct = 0;
        uint8_t pad0 = 0;
        uint16_t ccImmunityMask = 0;
        uint32_t mechanicImmunityMask = 0;
        uint8_t schoolImmunityMask = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint8_t pad3 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t resistId) const;

    // Returns the resist profile for a given creature
    // entry, or nullptr if the creature uses defaults.
    // Used by the damage-calculation path to look up
    // "what's this mob's frost resist?" without scanning.
    [[nodiscard]] const Entry* findByCreature(uint32_t creatureEntry) const;
};

class WoweeCreatureResistsLoader {
public:
    static bool save(const WoweeCreatureResists& cat,
                     const std::string& basePath);
    static WoweeCreatureResists load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cre* variants.
    //
    //   makeRaidBosses - 5 canonical raid-boss profiles
    //                     with high single-school
    //                     resistances or full immunities
    //                     (Ragnaros 100% fire / Vaelastrasz
    //                     50% all / Hakkar arcane immune /
    //                     Kel'Thuzad shadow immune /
    //                     Onyxia fire+frost partial).
    //   makeElites     - 5 mid-tier elite profiles with
    //                     moderate resists (water elementals
    //                     fire-resistant / stone golems
    //                     nature-resistant / etc.).
    //   makeImmunities - 4 CC-immunity test cases (root /
    //                     stun / silence / fear immune
    //                     creatures for boss-mechanic
    //                     verification).
    static WoweeCreatureResists makeRaidBosses(const std::string& catalogName);
    static WoweeCreatureResists makeElites(const std::string& catalogName);
    static WoweeCreatureResists makeImmunities(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
