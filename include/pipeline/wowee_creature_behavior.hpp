#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Behavior Tree catalog (.wbhv)
// - novel replacement for the implicit creature-AI
// rules vanilla WoW carried in
// creature_template.AIName + per-creature C++
// scripts in the server's ScriptMgr (most rare-
// elites and bosses had hand-coded class-derived
// AI). Each WBHV entry binds one combat behavior
// archetype to its creature kind (Melee / Caster /
// Tank / Healer / Pet / Beast), aggro / leash
// radii, evade-on-leash policy, corpse persistence
// duration, default rotation spell, and a variable-
// length list of special abilities (spellId +
// cooldown + use-chance triplets).
//
// Cross-references with previously-added formats:
//   WCRT: behaviorId is referenced by WCRT creature
//         entries (each creature picks one WBHV
//         policy).
//   WSPL: mainAttackSpellId and every special
//         ability spellId reference the WSPL spell
//         catalog.
//
// Binary layout (little-endian):
//   magic[4]            = "WBHV"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     behaviorId (uint32)
//     nameLen + name
//     creatureKind (uint8)         - 0=Melee /
//                                     1=Caster /
//                                     2=Tank /
//                                     3=Healer /
//                                     4=Pet /
//                                     5=Beast
//     evadeBehavior (uint8)        - 0=ResetToSpawn
//                                     /1=HealAtPath
//                                     /2=FleeToSpawn
//                                     /3=NoEvade
//     pad0 (uint16)
//     aggroRadius (float)
//     leashRadius (float)
//     corpseDurationSec (uint32)
//     mainAttackSpellId (uint32)
//     specialAbilityCount (uint32)
//     specialAbilities (each: spellId(4) +
//                              cooldownMs(4) +
//                              useChancePct(2) +
//                              pad1(2)) = 12 bytes
struct WoweeCreatureBehavior {
    enum CreatureKind : uint8_t {
        Melee  = 0,
        Caster = 1,
        Tank   = 2,
        Healer = 3,
        Pet    = 4,
        Beast  = 5,
    };

    enum EvadeBehavior : uint8_t {
        ResetToSpawn = 0,    // teleport home + full
                              //  HP/mana
        HealAtPath   = 1,    // run home, regen along
                              //  path
        FleeToSpawn  = 2,    // run home but stay
                              //  attackable
        NoEvade      = 3,    // permanent leash -
                              //  bosses only
    };

    struct SpecialAbility {
        uint32_t spellId = 0;
        uint32_t cooldownMs = 0;
        uint16_t useChancePct = 0;    // basis
                                        //  points
                                        //  0..10000
        uint16_t pad1 = 0;
    };

    struct Entry {
        uint32_t behaviorId = 0;
        std::string name;
        uint8_t creatureKind = Melee;
        uint8_t evadeBehavior = ResetToSpawn;
        uint16_t pad0 = 0;
        float aggroRadius = 0.f;
        float leashRadius = 0.f;
        uint32_t corpseDurationSec = 0;
        uint32_t mainAttackSpellId = 0;
        std::vector<SpecialAbility> specialAbilities;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t behaviorId) const;

    // Returns all behaviors of one kind - used by the
    // creature-template editor to suggest archetype
    // policies when authoring a new creature.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t creatureKind) const;
};

class WoweeCreatureBehaviorLoader {
public:
    static bool save(const WoweeCreatureBehavior& cat,
                     const std::string& basePath);
    static WoweeCreatureBehavior load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bhv* variants.
    //
    //   makeMeleeBehaviors  - 3 entry-tier melee
    //                          creatures (Kobold /
    //                          Wolf / Raptor) with
    //                          1 special each.
    //   makeCasterBehaviors - 3 caster creatures
    //                          (Defias Wizard /
    //                          Murloc Coastrunner /
    //                          Voidwalker) with 2-3
    //                          spells in rotation.
    //   makeBossBehaviors   - 1 boss-style behavior
    //                          (Onyxia-pattern) with
    //                          4 special abilities,
    //                          NoEvade, and 600s
    //                          corpse duration.
    static WoweeCreatureBehavior makeMeleeBehaviors(const std::string& catalogName);
    static WoweeCreatureBehavior makeCasterBehaviors(const std::string& catalogName);
    static WoweeCreatureBehavior makeBossBehaviors(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
