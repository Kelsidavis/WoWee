#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Catalog (.wspl) - novel replacement for
// Blizzard's Spell.dbc + SpellEffect.dbc + the AzerothCore-
// style spell_dbc / spell_proc tables. The 20th open format
// added to the editor.
//
// Each entry holds the metadata side of a spell: name,
// description, school, range, mana cost, cast / cooldown
// times, plus a single primary effect. The simplified
// effect model (one effectKind + min/max value + misc field)
// covers the common cases (damage / heal / buff / debuff /
// teleport / summon / dispel) without needing to reproduce
// the full multi-effect graph that classic Spell.dbc carries.
//
// Cross-references with previously-added formats:
//   WLCK.channel.targetId (kind=Spell) → WSPL.entry.spellId
//   WQT.objective.targetId (kind=SpellCast) → WSPL.entry.spellId
//   WCRT.entry.equippedMain (item with on-use) → WIT → WSPL
//
// Binary layout (little-endian):
//   magic[4]            = "WSPL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     spellId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     school (uint8) / targetType (uint8) / effectKind (uint8) / pad
//     castTimeMs (uint32)
//     cooldownMs (uint32)
//     gcdMs (uint32)
//     manaCost (uint32)
//     rangeMin (float)
//     rangeMax (float)
//     minLevel (uint16) / maxStacks (uint16)
//     durationMs (int32)         -- -1 = permanent, 0 = instant
//     effectValueMin (int32)
//     effectValueMax (int32)
//     effectMisc (int32)
//     flags (uint32)
struct WoweeSpell {
    enum School : uint8_t {
        SchoolPhysical = 0,
        SchoolHoly     = 1,
        SchoolFire     = 2,
        SchoolNature   = 3,
        SchoolFrost    = 4,
        SchoolShadow   = 5,
        SchoolArcane   = 6,
    };

    enum TargetType : uint8_t {
        TargetSelf       = 0,
        TargetSingle     = 1,
        TargetCone       = 2,
        TargetAoeFromSelf = 3,
        TargetLine       = 4,
        TargetGround     = 5,
    };

    enum EffectKind : uint8_t {
        EffectDamage   = 0,
        EffectHeal     = 1,
        EffectBuff     = 2,
        EffectDebuff   = 3,
        EffectTeleport = 4,
        EffectSummon   = 5,
        EffectDispel   = 6,
    };

    enum Flags : uint32_t {
        Passive       = 0x01,
        Hidden        = 0x02,    // not shown in spellbook
        Channeled     = 0x04,
        Ranged        = 0x08,
        AreaOfEffect  = 0x10,
        Triggered     = 0x20,    // no mana cost, no GCD
        UnitTargetOnly = 0x40,
        FriendlyOnly  = 0x80,
        HostileOnly   = 0x100,
    };

    struct Entry {
        uint32_t spellId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t school = SchoolPhysical;
        uint8_t targetType = TargetSelf;
        uint8_t effectKind = EffectDamage;
        uint32_t castTimeMs = 0;
        uint32_t cooldownMs = 0;
        uint32_t gcdMs = 1500;        // canonical 1.5s GCD default
        uint32_t manaCost = 0;
        float rangeMin = 0.0f;
        float rangeMax = 5.0f;        // melee range default
        uint16_t minLevel = 1;
        uint16_t maxStacks = 1;
        int32_t durationMs = 0;       // 0 = instant, -1 = permanent
        int32_t effectValueMin = 0;
        int32_t effectValueMax = 0;
        int32_t effectMisc = 0;
        uint32_t flags = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by spellId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t spellId) const;

    static const char* schoolName(uint8_t s);
    static const char* targetTypeName(uint8_t t);
    static const char* effectKindName(uint8_t e);
};

class WoweeSpellLoader {
public:
    static bool save(const WoweeSpell& cat,
                     const std::string& basePath);
    static WoweeSpell load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spells* variants.
    //
    //   makeStarter - 4 demo spells covering damage / heal /
    //                  buff / teleport effect kinds.
    //   makeMage    - frost bolt + fireball + arcane intellect
    //                  + blink - classic mage starter rotation.
    //   makeWarrior - mortal strike + shield bash + battle shout
    //                  + heroic strike - classic warrior toolkit.
    static WoweeSpell makeStarter(const std::string& catalogName);
    static WoweeSpell makeMage(const std::string& catalogName);
    static WoweeSpell makeWarrior(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
