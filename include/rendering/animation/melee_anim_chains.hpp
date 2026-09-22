#pragma once

/// Which attack animation to try, and what to fall back to when a model has
/// not got it.
///
/// Not every model carries every attack. A creature with no ATTACK_1H still
/// has to swing something, so each weapon kind names an ordered chain and the
/// first sequence the model actually has is the one played.
///
/// All ten were written out twice: once in AnimationController, which picks
/// the sequence to play, and once in the capability probe, which answers what
/// a model can do. The probe's copy carried a comment saying its chains
/// "match resolveMeleeAnimId" - a statement about another file that nothing
/// enforced. They did match, character for character, which is the state a
/// pair of tables is in right before one of them is edited.
///
/// A divergence here reports nothing. The probe says a model can attack
/// one-handed, the controller reaches a different point in a different chain
/// and plays something else, and a rogue swings a dagger like a mace.

#include <cstdint>
#include <span>

#include "rendering/animation/animation_ids.hpp"

namespace wowee::rendering::anim {

/// The weapon situations that pick a different chain.
enum class MeleeChain {
    OneHand,
    TwoHand,
    TwoHandLoose,   ///< polearms and staves, which thrust
    Dagger,
    Fist,
    Unarmed,
    OffHand,
    OffHandPierce,
    OffHandFist,
    OffHandUnarmed,
    /// No weapon known - a creature swinging whatever it has. Deliberately
    /// without the parry fallbacks the weapon chains end in: these callers
    /// want an attack or nothing, and a parry in place of one reads as the
    /// model flinching.
    Generic,
};

/// The ordered fallbacks for one situation, best first.
///
/// Each chain starts with the animation that situation is really asking for,
/// and every chain ends somewhere every model has. The parry entries at the
/// end of the melee chains are deliberate: a model with no attack at all still
/// moves its weapon arm for a parry, which reads better than standing still.
inline std::span<const uint32_t> meleeAnimChain(MeleeChain kind) {
    static constexpr uint32_t kOneHand[] = {
        ATTACK_1H, ATTACK_2H, ATTACK_UNARMED,
        ATTACK_2H_LOOSE, PARRY_UNARMED, PARRY_1H};
    static constexpr uint32_t kTwoHand[] = {
        ATTACK_2H, ATTACK_1H, ATTACK_UNARMED,
        ATTACK_2H_LOOSE, PARRY_UNARMED, PARRY_1H};
    static constexpr uint32_t kTwoHandLoose[] = {
        ATTACK_2H_LOOSE_PIERCE, ATTACK_2H_LOOSE,
        ATTACK_2H, ATTACK_1H, ATTACK_UNARMED};
    static constexpr uint32_t kDagger[] = {
        ATTACK_1H_PIERCE, ATTACK_1H, ATTACK_UNARMED};
    // Punches: 3.3.5 has no fist-weapon animations of its own.
    static constexpr uint32_t kFist[] = {
        ATTACK_UNARMED, ATTACK_1H, PARRY_UNARMED, PARRY_1H};
    static constexpr uint32_t kUnarmed[] = {
        ATTACK_UNARMED, ATTACK_1H, ATTACK_2H,
        ATTACK_2H_LOOSE, PARRY_UNARMED, PARRY_1H};
    static constexpr uint32_t kOffHand[] = {
        ATTACK_OFF, ATTACK_1H, ATTACK_UNARMED};
    static constexpr uint32_t kOffHandPierce[] = {
        ATTACK_OFF_PIERCE, ATTACK_OFF, ATTACK_1H_PIERCE, ATTACK_1H};
    static constexpr uint32_t kOffHandFist[] = {
        ATTACK_UNARMED_OFF, ATTACK_OFF, ATTACK_UNARMED, ATTACK_1H};
    static constexpr uint32_t kOffHandUnarmed[] = {
        ATTACK_UNARMED_OFF, ATTACK_UNARMED, ATTACK_OFF, ATTACK_1H};
    static constexpr uint32_t kGeneric[] = {
        ATTACK_1H, ATTACK_2H, ATTACK_2H_LOOSE, ATTACK_UNARMED};

    switch (kind) {
        case MeleeChain::OneHand:        return kOneHand;
        case MeleeChain::TwoHand:        return kTwoHand;
        case MeleeChain::TwoHandLoose:   return kTwoHandLoose;
        case MeleeChain::Dagger:         return kDagger;
        case MeleeChain::Fist:           return kFist;
        case MeleeChain::Unarmed:        return kUnarmed;
        case MeleeChain::OffHand:        return kOffHand;
        case MeleeChain::OffHandPierce:  return kOffHandPierce;
        case MeleeChain::OffHandFist:    return kOffHandFist;
        case MeleeChain::OffHandUnarmed: return kOffHandUnarmed;
        case MeleeChain::Generic:        return kGeneric;
    }
    return kUnarmed;
}

}  // namespace wowee::rendering::anim
