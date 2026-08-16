#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Proc Rules catalog (.wprc) -
// novel replacement for the implicit
// proc-on-event spell triggers vanilla WoW carried
// in SpellProcEvents (later rows of Spell.dbc) +
// the per-spell procFlags + procChance fields
// scattered across multiple DBC tables. Each WPRC
// entry binds one proc rule to its source spell
// (the aura/buff that has the proc), trigger event
// (OnHit/OnCrit/OnTakeDamage/etc), proc chance in
// basis points, internal cooldown, the spell to
// trigger on proc, max-stacks-on-target cap, and
// optional condition flags (require melee weapon,
// require spell school, etc).
//
// Cross-references with previously-added formats:
//   WSPL: sourceSpellId AND procEffectSpellId both
//         reference the WSPL spell catalog.
//   WBHV: creature behaviors that cast sourceSpellId
//         get the proc behavior automatically via
//         this catalog at runtime.
//
// Binary layout (little-endian):
//   magic[4]            = "WPRC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     procRuleId (uint32)
//     nameLen + name
//     sourceSpellId (uint32)        - the aura/buff
//                                     that owns the
//                                     proc
//     procEffectSpellId (uint32)    - the spell to
//                                     trigger on
//                                     proc
//     triggerEvent (uint8)          - 0=OnHit /
//                                     1=OnCrit /
//                                     2=OnCast /
//                                     3=OnTakeDamage
//                                     /4=OnHeal /
//                                     5=OnDodge /
//                                     6=OnParry /
//                                     7=OnBlock /
//                                     8=OnKill
//     maxStacksOnTarget (uint8)     - 0 = no stack
//                                     limit
//     procChancePct (uint16)        - basis points
//                                     0..10000 (100=
//                                     1%)
//     internalCooldownMs (uint32)   - 0 = no ICD
//     procFlagsMask (uint16)        - bitmask of
//                                     additional
//                                     conditions
//     pad0 (uint16)
struct WoweeSpellProcRules {
    enum TriggerEvent : uint8_t {
        OnHit         = 0,
        OnCrit        = 1,
        OnCast        = 2,
        OnTakeDamage  = 3,
        OnHeal        = 4,
        OnDodge       = 5,
        OnParry       = 6,
        OnBlock       = 7,
        OnKill        = 8,
    };

    enum ProcFlag : uint16_t {
        RequireMeleeWeapon   = 0x0001,
        RequireRangedWeapon  = 0x0002,
        RequireSpellSchool   = 0x0004,
        ExcludeAutoAttack    = 0x0008,
        OnlyFromBehind       = 0x0010,
        OnlyVsPvPTarget      = 0x0020,
    };

    struct Entry {
        uint32_t procRuleId = 0;
        std::string name;
        uint32_t sourceSpellId = 0;
        uint32_t procEffectSpellId = 0;
        uint8_t triggerEvent = OnHit;
        uint8_t maxStacksOnTarget = 0;
        uint16_t procChancePct = 0;
        uint32_t internalCooldownMs = 0;
        uint16_t procFlagsMask = 0;
        uint16_t pad0 = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t procRuleId) const;

    // Returns all proc rules where the given spellId
    // is the source aura. A single buff may have
    // multiple procs on different events.
    [[nodiscard]] std::vector<const Entry*> findBySourceSpell(uint32_t spellId) const;

    // Returns all proc rules that fire on a given
    // event - used by the combat-event dispatcher
    // hot path.
    [[nodiscard]] std::vector<const Entry*> findByEvent(uint8_t triggerEvent) const;
};

class WoweeSpellProcRulesLoader {
public:
    static bool save(const WoweeSpellProcRules& cat,
                     const std::string& basePath);
    static WoweeSpellProcRules load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-prc* variants.
    //
    //   makeWeaponProcs   - 3 vanilla weapon-enchant
    //                        procs (Crusader buff
    //                        OnHit / Lifesteal heal-
    //                        on-hit / Fiery Weapon
    //                        damage proc).
    //   makeRetPaladin    - 4 procs from a
    //                        Retribution Paladin
    //                        rotation (Vengeance buff
    //                        OnCrit / Seal of Justice
    //                        stun proc / Reckoning
    //                        block-counter / Sanctity
    //                        Aura damage amp).
    //   makeRageGen       - 3 Rage-generation procs
    //                        (Bloodrage instant /
    //                        Berserker Rage immunity
    //                        / Anger Mgmt OnDodge).
    static WoweeSpellProcRules makeWeaponProcs(const std::string& catalogName);
    static WoweeSpellProcRules makeRetPaladin(const std::string& catalogName);
    static WoweeSpellProcRules makeRageGen(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
