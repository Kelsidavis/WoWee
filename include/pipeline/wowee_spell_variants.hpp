#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Variant catalog (.wspv) - novel
// replacement for the implicit context-conditional
// spell substitution rules vanilla WoW encoded across
// SpellSpecificType, SpellEffect.EffectMechanic
// override fields, and the proc-modified spell tables
// in SpellProcEvent. Each entry binds one base spell
// to a variant spell that activates when a runtime
// condition is met (player in a specific stance,
// talent talented, racial buff active, etc.).
//
// The spell-cast pipeline consults this catalog at the
// moment of cast: if any variant whose condition is
// satisfied has higher priority than the base, the
// variant spell ID is substituted for the cast.
//
// Cross-references with previously-added formats:
//   WSPL: baseSpellId and variantSpellId both reference
//         the WSPL spell catalog.
//   WCMG: when conditionKind=Stance/Form, conditionValue
//         is a WCMG spellId (the stance/form active
//         on the caster).
//   WTAL: when conditionKind=Talent, conditionValue is
//         a WTAL talent ID (the talented passive that
//         enables the variant).
//   WCHC: when conditionKind=Race, conditionValue is a
//         WCHC race bit.
//
// Binary layout (little-endian):
//   magic[4]            = "WSPV"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     variantId (uint32)
//     nameLen + name
//     descLen + description
//     baseSpellId (uint32)
//     variantSpellId (uint32)
//     conditionKind (uint8)      - Stance / Form /
//                                   Talent / Race /
//                                   EquippedWeapon /
//                                   AuraActive
//     priority (uint8)           - higher overrides
//                                   lower; 0 = base
//                                   spell baseline
//     pad0 (uint8) / pad1 (uint8)
//     conditionValue (uint32)    - polymorphic - see
//                                   conditionKind
//     iconColorRGBA (uint32)
struct WoweeSpellVariants {
    enum ConditionKind : uint8_t {
        Stance         = 0,    // value = stance spellId
        Form           = 1,    // value = druid form
                                // spellId
        Talent         = 2,    // value = talentId
        Race           = 3,    // value = WCHC race bit
        EquippedWeapon = 4,    // value = weapon-type
                                // bitmask (sword=2,
                                // mace=4, etc.)
        AuraActive     = 5,    // value = aura spellId
                                // currently on caster
    };

    struct Entry {
        uint32_t variantId = 0;
        std::string name;
        std::string description;
        uint32_t baseSpellId = 0;
        uint32_t variantSpellId = 0;
        uint8_t conditionKind = Stance;
        uint8_t priority = 1;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t conditionValue = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t variantId) const;

    // Returns all variants of one base spell, sorted by
    // descending priority. The spell-cast pipeline
    // iterates this list at cast time and picks the
    // highest-priority variant whose condition is
    // satisfied (or falls through to the base spell).
    [[nodiscard]] std::vector<const Entry*> findByBaseSpell(uint32_t baseSpellId) const;
};

class WoweeSpellVariantsLoader {
public:
    static bool save(const WoweeSpellVariants& cat,
                     const std::string& basePath);
    static WoweeSpellVariants load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spv* variants.
    //
    //   makeWarriorStance - 4 stance-conditional Warrior
    //                        spell variants (Heroic Strike
    //                        Battle vs Berserker damage
    //                        bonus, Mocking Blow
    //                        Defensive AoE, Pummel
    //                        Berserker silence).
    //   makeTalentMod    - 4 talent-modified spell
    //                        variants (Frostbolt + Brain
    //                        Freeze proc, Lava Burst +
    //                        Flame Shock auto-crit,
    //                        Earth Shield + Improved,
    //                        Ferocious Bite + Berserk).
    //   makeRacial       - 4 racial spell variants
    //                        (Stoneform / War Stomp /
    //                        Berserking / WoTF).
    static WoweeSpellVariants makeWarriorStance(const std::string& catalogName);
    static WoweeSpellVariants makeTalentMod(const std::string& catalogName);
    static WoweeSpellVariants makeRacial(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
