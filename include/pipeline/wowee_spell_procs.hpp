#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Proc Trigger catalog (.wsps) - novel
// replacement for AzerothCore's spell_proc_event SQL
// table plus the per-spell proc fields embedded in
// Spell.dbc. Defines when a "trigger" spell fires in
// response to other spell/combat events: Windfury Weapon
// procs on melee attack, Clearcasting on damaging cast,
// Judgement of Wisdom on melee hit, etc.
//
// Each entry says "when an event matching procFlags fires
// from a spell matching procFromSpellId (0 = any), at
// procChance probability with at most one trigger per
// internalCooldownMs window, fire triggerSpellId". The
// procPpm field provides an alternative procs-per-minute
// formula (when non-zero, supersedes procChance and
// scales with weapon speed for melee procs).
//
// Cross-references with previously-added formats:
//   WSPL: triggerSpellId references the spell that fires
//         on proc; procFromSpellId references the source
//         spell whose events qualify (0 = any spell).
//
// Binary layout (little-endian):
//   magic[4]            = "WSPS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     procId (uint32)
//     nameLen + name
//     descLen + description
//     triggerSpellId (uint32)
//     procFromSpellId (uint32)
//     procChance (float)
//     procPpm (float)
//     procFlags (uint32)
//     internalCooldownMs (uint32)
//     charges (uint8) / pad[3]
//     iconColorRGBA (uint32)
struct WoweeSpellProc {
    enum ProcFlag : uint32_t {
        DealtMeleeAutoAttack    = 1u << 0,    // white melee swing landed
        DealtMeleeSpell         = 1u << 1,    // melee ability hit
        TakenMeleeAutoAttack    = 1u << 2,    // received white melee
        TakenMeleeSpell         = 1u << 3,    // received melee ability
        DealtRangedAutoAttack   = 1u << 4,    // ranged auto-shot landed
        DealtRangedSpell        = 1u << 5,    // ranged spell landed
        DealtSpell              = 1u << 6,    // any harmful spell landed
        DealtSpellHeal          = 1u << 7,    // healing spell landed
        TakenSpell              = 1u << 8,    // received any harmful spell
        OnKill                  = 1u << 9,    // fired a killing blow
        OnDeath                 = 1u << 10,   // wearer died
        OnCastFinished          = 1u << 11,   // any spell cast completed
        Critical                = 1u << 12,   // restricted to crit events
    };

    struct Entry {
        uint32_t procId = 0;
        std::string name;
        std::string description;
        uint32_t triggerSpellId = 0;
        uint32_t procFromSpellId = 0;     // 0 = any source
        float procChance = 0.0f;          // 0..1
        float procPpm = 0.0f;             // 0 = use procChance
        uint32_t procFlags = 0;
        uint32_t internalCooldownMs = 0;
        uint8_t charges = 0;              // 0 = unlimited
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t procId) const;

    static const char* procFlagName(uint32_t bit);
};

class WoweeSpellProcLoader {
public:
    static bool save(const WoweeSpellProc& cat,
                     const std::string& basePath);
    static WoweeSpellProc load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sps* variants.
    //
    //   makeWeapon  - 4 weapon-imbue procs (Windfury /
    //                  Frostbrand / Flametongue / Mana Oil)
    //                  triggered on DealtMeleeAutoAttack
    //                  with PPM-style chance.
    //   makeAura    - 4 aura-tied procs (Blessing of
    //                  Wisdom mana return, Molten Armor
    //                  crit-reflect, Earth Shield heal,
    //                  Judgement of Wisdom).
    //   makeTalent  - 4 talent procs (Clearcasting,
    //                  Omen of Clarity, Seal of
    //                  Righteousness, Nightfall) with
    //                  internal cooldowns to match canonical
    //                  WoW behavior.
    static WoweeSpellProc makeWeapon(const std::string& catalogName);
    static WoweeSpellProc makeAura(const std::string& catalogName);
    static WoweeSpellProc makeTalent(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
