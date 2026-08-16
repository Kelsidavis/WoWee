#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Effect Type catalog (.wsef) - novel
// replacement for the SpellEffect.Effect field meanings
// in Spell.dbc plus the engine's hard-coded effect
// dispatch table. Defines what each spell-effect integer
// value actually does - SCHOOL_DAMAGE=2 deals magical
// damage, DUMMY=3 is a script hook, HEAL=10 restores
// health, ENERGIZE=30 restores power, APPLY_AURA=6
// attaches a buff/debuff, etc.
//
// WotLK's Spell.dbc has 192+ effect type integers, each
// with its own resolver in the spell engine. This catalog
// lets the engine look up "given effect=10, what
// resolution behavior do I run?" via a single table lookup
// instead of a hard-coded switch statement, and lets
// server-custom spells reference new effect IDs without
// touching engine code.
//
// Distinct from WAUR (Spell Aura Type) which is the
// secondary classification used when effectType is
// APPLY_AURA - that's a separate enum entirely with its
// own ~300 values.
//
// Cross-references with previously-added formats:
//   WSPL: spell.effect[1..3] each reference an effectId
//         here.
//   WSPC: spells with cost-modifying effects can reference
//         WSPC powerCostId via the targetType field
//         interpretation.
//
// Binary layout (little-endian):
//   magic[4]            = "WSEF"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     effectId (uint32)
//     nameLen + name
//     descLen + description
//     effectKind (uint8) / behaviorFlags (uint8) / pad[2]
//     baseAmount (int32)
//     iconColorRGBA (uint32)
struct WoweeSpellEffectType {
    enum EffectKind : uint8_t {
        Damage      = 0,    // deals damage to target
        Heal        = 1,    // restores health
        Aura        = 2,    // applies buff/debuff
        Energize    = 3,    // restores power resource
        Trigger     = 4,    // fires another spell
        Movement    = 5,    // teleport / charge / pull
        Summon      = 6,    // summon pet/totem/object
        Dispel      = 7,    // remove auras
        Dummy       = 8,    // script hook, no built-in behavior
        Misc        = 9,    // catch-all
    };

    enum BehaviorFlag : uint8_t {
        RequiresTarget       = 1u << 0,    // must have a target
        RequiresLineOfSight  = 1u << 1,    // LoS check on target
        IsHostileEffect      = 1u << 2,    // hostile only (PvP gating)
        IsBeneficialEffect   = 1u << 3,    // friendly only
        IgnoresImmunities    = 1u << 4,    // bypasses Bubble / IBF / etc
        TriggersGCD          = 1u << 5,    // counts toward GCD
    };

    struct Entry {
        uint32_t effectId = 0;
        std::string name;
        std::string description;
        uint8_t effectKind = Damage;
        uint8_t behaviorFlags = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        int32_t baseAmount = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t effectId) const;

    static const char* effectKindName(uint8_t k);
};

class WoweeSpellEffectTypeLoader {
public:
    static bool save(const WoweeSpellEffectType& cat,
                     const std::string& basePath);
    static WoweeSpellEffectType load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sef* variants.
    //
    //   makeDamage  - 5 damage effect entries (SchoolDamage,
    //                  EnvironmentalDamage, WeaponDamageNoSchool,
    //                  NormalizedWeaponDmg, PowerBurn) covering
    //                  the standard damage-effect IDs from
    //                  Spell.dbc.
    //   makeHealing - 4 healing effects (Heal, HealMaxHealth,
    //                  HealPct, ScriptedHeal) - all flagged
    //                  IsBeneficialEffect.
    //   makeAura    - 5 aura-application effects
    //                  (ApplyAura, ApplyAuraOnPet,
    //                  AreaAuraParty, AreaAuraOwner,
    //                  PersistentAreaAura).
    static WoweeSpellEffectType makeDamage(const std::string& catalogName);
    static WoweeSpellEffectType makeHealing(const std::string& catalogName);
    static WoweeSpellEffectType makeAura(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
