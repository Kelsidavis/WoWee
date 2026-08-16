#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Aura Type catalog (.waur) - novel
// replacement for the SpellEffect.EffectAuraType field
// meanings used when SpellEffect.Effect=APPLY_AURA.
// Defines what each aura-type integer value actually does
// once an aura is attached to a unit - PERIODIC_DAMAGE
// ticks damage every N seconds, MOD_STAT adds a stat
// bonus, MOD_INCREASE_SPEED scales movement speed,
// MOD_DAMAGE_PERCENT_DONE scales spell power, etc.
//
// Companion to WSEF (Spell Effect Type) - together they
// cover the full spell-effect classification space:
//   WSEF: outer effect ID (what does the effect DO?)
//   WAUR: inner aura type (when WSEF=APPLY_AURA, what
//          KIND of aura is applied?)
//
// WotLK's Spell.dbc has 300+ aura types, each with its
// own resolver in the aura update loop. This catalog
// lets the engine look up "given auraType=3, how often
// do I tick and what behavior do I run?" via a single
// table lookup.
//
// Cross-references with previously-added formats:
//   WSPL: spells with effect=APPLY_AURA reference an
//         auraTypeId here.
//   WSEF: this catalog is the secondary classification
//         that WSEF entry id 6 (APPLY_AURA) dispatches
//         into.
//
// Binary layout (little-endian):
//   magic[4]            = "WAUR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     auraTypeId (uint32)
//     nameLen + name
//     descLen + description
//     auraKind (uint8) / targetingHint (uint8)
//     isStackable (uint8) / maxStackCount (uint8)
//     updateFrequencyMs (uint32)
//     iconColorRGBA (uint32)
struct WoweeSpellAuraType {
    enum AuraKind : uint8_t {
        Periodic        = 0,    // ticks damage/heal/energize over time
        StatMod         = 1,    // modifies a stat (Strength, Stamina)
        DamageMod       = 2,    // modifies damage done/taken
        Movement        = 3,    // changes mobility (Stun, Root, Snare)
        Visual          = 4,    // pure cosmetic (glow, particle effect)
        Trigger         = 5,    // periodic trigger of another spell
        Resource        = 6,    // affects power regen / consumption
        Control         = 7,    // mind control / charm / fear
        Misc            = 8,    // catch-all
    };

    enum TargetingHint : uint8_t {
        AnyUnit         = 0,    // can apply to any unit
        SelfOnly        = 1,    // caster-only
        HostileOnly     = 2,    // hostile-target only
        BeneficialOnly  = 3,    // friendly-target only
    };

    struct Entry {
        uint32_t auraTypeId = 0;
        std::string name;
        std::string description;
        uint8_t auraKind = Periodic;
        uint8_t targetingHint = AnyUnit;
        uint8_t isStackable = 0;        // 0/1 bool
        uint8_t maxStackCount = 0;      // 0 = no cap (when stackable)
        uint32_t updateFrequencyMs = 0; // periodic tick interval; 0 for non-periodic
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t auraTypeId) const;

    static const char* auraKindName(uint8_t k);
    static const char* targetingHintName(uint8_t t);
};

class WoweeSpellAuraTypeLoader {
public:
    static bool save(const WoweeSpellAuraType& cat,
                     const std::string& basePath);
    static WoweeSpellAuraType load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-aur* variants.
    //
    //   makePeriodic - 5 periodic auras (PeriodicDamage,
    //                   PeriodicHeal, PeriodicEnergize,
    //                   PeriodicLeech, PeriodicTriggerSpell)
    //                   all with 3000ms canonical tick.
    //   makeStatMod  - 5 stat modifiers (ModStat,
    //                   ModResistance, ModDamageDone,
    //                   ModHaste, ModCritPercent) - non-
    //                   periodic, instantly applied.
    //   makeMovement - 4 movement-impairing auras (Stun,
    //                   Root, ModDecreaseSpeed, ModConfuse)
    //                   typically applied by CC spells.
    static WoweeSpellAuraType makePeriodic(const std::string& catalogName);
    static WoweeSpellAuraType makeStatMod(const std::string& catalogName);
    static WoweeSpellAuraType makeMovement(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
