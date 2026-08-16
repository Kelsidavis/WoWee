#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Power Cost catalog (.wspc) - novel
// replacement for the per-spell power-cost fields in
// Spell.dbc plus SpellPowerCost-related side tables.
// Defines categorical power-cost buckets that spells
// reference (LowMana 5% / MediumMana 15% / HighMana 30% /
// fixed Rage-30 / Energy-40 / Runic-30 / etc), so spells
// share cost metadata across ranks instead of embedding
// per-rank cost numbers.
//
// Completes the small lookup-bucket set:
//   WSRG - range bucket
//   WSCT - cast time bucket
//   WSDR - duration bucket
//   WSCD - cooldown bucket
//   WSPC - power cost bucket   (this catalog)
//
// Five small integer ids per spell (range / cast / dur /
// cd / cost) replace the dozens of duplicate per-rank
// fields that Blizzard's Spell.dbc carries. The engine
// retunes thousands of spells at once by editing one
// bucket here.
//
// Cost can be flat (baseCost), per-level scaled
// (perLevelCost), or percentage-of-max-power
// (percentOfBase). The engine uses whichever fields are
// non-zero, summing them. percentOfBase=0.05 with
// baseCost=10 means "5% of max mana + 10 flat".
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the spell
//   engine. WSPL spell entries reference powerCostId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSPC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     powerCostId (uint32)
//     nameLen + name
//     descLen + description
//     powerType (uint8) / pad[3]
//     baseCost (int32)
//     perLevelCost (int32)
//     percentOfBase (float)
//     costFlags (uint32)
//     iconColorRGBA (uint32)
struct WoweeSpellPowerCost {
    enum PowerType : uint8_t {
        Mana         = 0,
        Rage         = 1,
        Focus        = 2,    // hunter pet only in Classic; hunter in Cata+
        Energy       = 3,    // rogue / cat druid
        Happiness    = 4,    // hunter pet (Classic-Wrath)
        RunicPower   = 5,    // death knight
        Runes        = 6,    // death knight rune count (separate bucket)
        SoulShards   = 7,    // warlock
        HolyPower    = 8,    // paladin (Cata+)
        Eclipse      = 9,    // balance druid (Cata+)
        Health       = 10,   // sacrificial / blood-tap costs
        NoCost       = 11,   // free spell
    };

    enum CostFlag : uint32_t {
        RequiresCombatStance = 1u << 0,   // berserker stance / battle stance
        RefundOnMiss         = 1u << 1,   // refunds 80% if spell misses
        DoublesInForm        = 1u << 2,   // doubles in cat/bear form
        ScalesWithMastery    = 1u << 3,   // mastery reduces this cost
    };

    struct Entry {
        uint32_t powerCostId = 0;
        std::string name;
        std::string description;
        uint8_t powerType = Mana;
        int32_t baseCost = 0;
        int32_t perLevelCost = 0;
        float percentOfBase = 0.0f;
        uint32_t costFlags = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t powerCostId) const;

    // Resolve the actual cost amount at the given caster
    // level + max power pool. Sums baseCost + perLevelCost
    // *level + percentOfBase*maxPower (whichever fields are
    // non-zero). Returns 0 for NoCost type.
    [[nodiscard]] int32_t resolveCost(uint32_t powerCostId,
                         uint32_t casterLevel,
                         int32_t maxPower) const;

    static const char* powerTypeName(uint8_t k);
};

class WoweeSpellPowerCostLoader {
public:
    static bool save(const WoweeSpellPowerCost& cat,
                     const std::string& basePath);
    static WoweeSpellPowerCost load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spc* variants.
    //
    //   makeStarter - 4 baseline mana cost tiers (NoCost,
    //                  LowMana 5%, MediumMana 15%, HighMana
    //                  30%) - % of max mana, scales with
    //                  caster level naturally.
    //   makeRage    - 4 fixed rage costs for warrior abilities
    //                  (HeroicStrike 15, Slam 20, Whirlwind 25,
    //                  MortalStrike 30).
    //   makeMixed   - 5 cross-class cost buckets covering
    //                  every non-mana power type (Hunter
    //                  Focus 30, Rogue Energy 40, DK Runic 30,
    //                  Paladin Holy 1, Warlock SoulShard 1).
    static WoweeSpellPowerCost makeStarter(const std::string& catalogName);
    static WoweeSpellPowerCost makeRage(const std::string& catalogName);
    static WoweeSpellPowerCost makeMixed(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
