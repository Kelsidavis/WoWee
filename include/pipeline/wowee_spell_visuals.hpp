#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Visual Kit catalog (.wsvk) - novel
// replacement for Blizzard's SpellVisualKit.dbc +
// SpellVisualEffectName.dbc + the AzerothCore-style spell
// visual SQL data. The 47th open format added to the editor.
//
// Defines per-spell visual presentations: cast-bar effect
// model, projectile model + travel speed + arc gravity,
// impact effect model, hand effect on the caster, and the
// animations + sounds that fire at cast / channel / impact
// time. Spells reference a visualKitId here from Spell.dbc
// (or WSPL.spellId.visualKitId), so this catalog binds the
// "what happens visually" to the "what mechanically happens"
// in the spell catalog.
//
// Cross-references with previously-added formats:
//   WSVK.entry.castAnimId    → WANI.entry.animationId
//   WSVK.entry.impactAnimId  → WANI.entry.animationId
//   WSVK.entry.precastAnimId → WANI.entry.animationId
//   WSVK.entry.castSoundId   → WSND.entry.soundId
//   WSVK.entry.impactSoundId → WSND.entry.soundId
//
// Binary layout (little-endian):
//   magic[4]            = "WSVK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     visualKitId (uint32)
//     nameLen + name
//     descLen + description
//     castModelLen + castEffectModelPath
//     projModelLen + projectileModelPath
//     impactModelLen + impactEffectModelPath
//     handModelLen + handEffectModelPath
//     precastAnimId (uint32)
//     castAnimId (uint32)
//     impactAnimId (uint32)
//     castSoundId (uint32)
//     impactSoundId (uint32)
//     projectileSpeed (float)         - units/sec, 0=instant
//     projectileGravity (float)       - 0=straight line
//     castDurationMs (uint32)
//     impactRadius (float)            - splash AoE in units
struct WoweeSpellVisualKit {
    struct Entry {
        uint32_t visualKitId = 0;
        std::string name;
        std::string description;
        std::string castEffectModelPath;     // M2 played during cast
        std::string projectileModelPath;     // M2 that flies to target
        std::string impactEffectModelPath;   // M2 played on impact
        std::string handEffectModelPath;     // M2 attached to caster hand
        uint32_t precastAnimId = 0;          // WANI cross-ref
        uint32_t castAnimId = 0;             // WANI cross-ref
        uint32_t impactAnimId = 0;           // WANI cross-ref
        uint32_t castSoundId = 0;            // WSND cross-ref
        uint32_t impactSoundId = 0;          // WSND cross-ref
        float projectileSpeed = 0.0f;        // 0 = instant hit
        float projectileGravity = 0.0f;      // 0 = straight line
        uint32_t castDurationMs = 0;
        float impactRadius = 0.0f;           // 0 = single-target
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t visualKitId) const;
};

class WoweeSpellVisualKitLoader {
public:
    static bool save(const WoweeSpellVisualKit& cat,
                     const std::string& basePath);
    static WoweeSpellVisualKit load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-svk* variants.
    //
    //   makeStarter - 3 visual kits (Frostbolt / Fireball /
    //                  Healing) covering the canonical
    //                  projectile + heal triad.
    //   makeCombat  - 5 combat visuals (sword swing impact,
    //                  arrow shot, ground pound, parry,
    //                  deflect) with WANI animation refs.
    //   makeUtility - 4 utility visuals (portal/teleport,
    //                  hearthstone return, mount summon,
    //                  resurrection) with no projectile.
    static WoweeSpellVisualKit makeStarter(const std::string& catalogName);
    static WoweeSpellVisualKit makeCombat(const std::string& catalogName);
    static WoweeSpellVisualKit makeUtility(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
