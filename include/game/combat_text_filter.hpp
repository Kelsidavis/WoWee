#pragma once

#include <cstdint>

#include "game/spell_defines.hpp"

namespace wowee {
namespace game {

/// Which Combat Text panel checkbox covers one floating line.
///
/// Six controls in that panel name a CVar that nothing read, so clearing any
/// of them changed the panel and not the screen. They divide one stream by what
/// produced it, so the mapping is decided once, here, rather than at each of
/// the call sites that raise a line.
///
/// The rule is returned rather than applied: this header knows which question
/// to ask and what the answer is when nobody has chosen, and the caller does
/// the CVar lookup. That keeps the decision testable without a settings store,
/// and keeps the store out of a header that only describes combat text.
struct CombatTextFilterRule {
    /// The CVar that governs this line, or nullptr when none does.
    const char* cvar = nullptr;
    /// What that CVar means when it has never been set.
    const char* fallback = "1";
};

/// The rule covering one line, given who caused it and where it landed.
///
/// Only lines the player caused are filtered. Damage arriving at the player is
/// a different question with its own rows in the panel, and a checkbox reading
/// "Target Damage" does not mean "damage taken".
inline CombatTextFilterRule combatTextFilterFor(CombatTextEntry::Type type,
                                                bool isPlayerSource,
                                                uint64_t srcGuid,
                                                uint64_t dstGuid,
                                                uint64_t petGuid,
                                                uint64_t targetGuid) {
    using T = CombatTextEntry;
    switch (type) {
        // Effects rather than numbers, read against the unit they landed on
        // instead of who caused them: the panel offers your target and
        // everything else as two separate rows.
        case T::IMMUNE: case T::RESIST: case T::DEFLECT: case T::REFLECT:
        case T::INTERRUPT: case T::DISPEL: case T::STEAL: case T::ABSORB: {
            const bool onTarget = (dstGuid != 0 && dstGuid == targetGuid);
            return onTarget ? CombatTextFilterRule{.cvar = "fctSpellMechanics", .fallback = "1"}
                            : CombatTextFilterRule{.cvar = "fctSpellMechanicsOther", .fallback = "0"};
        }
        // A pet's swing is its own row, and it is answered before the damage
        // row below so that clearing pet damage does not also require the
        // player to clear their own.
        case T::MELEE_DAMAGE:
            if (petGuid != 0 && srcGuid == petGuid) {
                return {.cvar = "PetMeleeDamage", .fallback = "1"};
            }
            [[fallthrough]];
        case T::SPELL_DAMAGE: case T::CRIT_DAMAGE:
        case T::GLANCING: case T::CRUSHING:
            if (!isPlayerSource) return {};
            return {.cvar = "CombatDamage", .fallback = "1"};
        case T::PERIODIC_DAMAGE: case T::PERIODIC_HEAL:
            if (!isPlayerSource) return {};
            return {.cvar = "CombatLogPeriodicSpells", .fallback = "1"};
        case T::HEAL: case T::CRIT_HEAL:
            if (!isPlayerSource) return {};
            return {.cvar = "CombatHealing", .fallback = "1"};
        default:
            return {};
    }
}

} // namespace game
} // namespace wowee
