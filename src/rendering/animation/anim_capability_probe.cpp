// ============================================================================
// AnimCapabilityProbe
//
// Scans a model's animation capabilities once and returns an
// AnimCapabilitySet with resolved IDs (after fallback chains).
// Extracted from the scattered hasAnimation/pickFirstAvailable calls
// in AnimationController.
// ============================================================================

#include "rendering/animation/melee_anim_chains.hpp"
#include "rendering/animation/anim_capability_probe.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "rendering/renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/m2_renderer.hpp"

namespace wowee {
namespace rendering {

uint32_t AnimCapabilityProbe::pickFirst(Renderer* renderer, uint32_t instanceId,
                                        const uint32_t* candidates, size_t count) {
    auto* charRenderer = renderer->getCharacterRenderer();
    if (!charRenderer) return 0;
    for (size_t i = 0; i < count; ++i) {
        if (charRenderer->hasAnimation(instanceId, candidates[i])) {
            return candidates[i];
        }
    }
    return 0;
}

AnimCapabilitySet AnimCapabilityProbe::probe(Renderer* renderer, uint32_t instanceId) {
    AnimCapabilitySet caps;
    if (!renderer || instanceId == 0) return caps;

    auto* cr = renderer->getCharacterRenderer();
    if (!cr) return caps;

    auto has = [&](uint32_t id) -> bool {
        return cr->hasAnimation(instanceId, id);
    };

    // Helper: pick first available from static array
    auto pick = [&](const uint32_t* candidates, size_t count) -> uint32_t {
        return pickFirst(renderer, instanceId, candidates, count);
    };
    // The same, for the melee chains, which are shared with the controller
    // that plays them rather than written out again here.
    auto pickFrom = [&](anim::MeleeChain kind) -> uint32_t {
        const auto chain = anim::meleeAnimChain(kind);
        return pickFirst(renderer, instanceId, chain.data(), chain.size());
    };

    // ── Locomotion ──────────────────────────────────────────────────────
    // STAND is animation ID 0. Every M2 model has sequence 0 as its base idle.
    // Cannot use ternary `has(0) ? 0 : 0` - both branches are 0.
    caps.resolvedStand = anim::STAND;
    caps.hasStand = has(anim::STAND);

    {
        static const uint32_t walkCands[] = {anim::WALK, anim::RUN};
        caps.resolvedWalk = pick(walkCands, 2);
        caps.hasWalk = (caps.resolvedWalk != 0);
    }
    {
        static const uint32_t runCands[] = {anim::RUN, anim::WALK};
        caps.resolvedRun = pick(runCands, 2);
        caps.hasRun = (caps.resolvedRun != 0);
    }
    {
        static const uint32_t sprintCands[] = {anim::SPRINT, anim::RUN, anim::WALK};
        caps.resolvedSprint = pick(sprintCands, 3);
        caps.hasSprint = (caps.resolvedSprint != 0);
    }
    {
        static const uint32_t walkBackCands[] = {anim::WALK_BACKWARDS, anim::WALK};
        caps.resolvedWalkBackwards = pick(walkBackCands, 2);
        caps.hasWalkBackwards = (caps.resolvedWalkBackwards != 0);
    }
    // Note: no resolvedStrafeLeft/Right or resolvedRunLeft/Right here --
    // strafing (pure or diagonal) reuses plain Walk/Run and the renderer's
    // torso-bone rotation (CharacterRenderer::setInstanceTorsoYaw) for the
    // visual angle. See LocomotionFSM::resolve().

    // ── Jump ────────────────────────────────────────────────────────────
    caps.resolvedJumpStart = has(anim::JUMP_START) ? anim::JUMP_START : 0;
    caps.resolvedJump = has(anim::JUMP) ? anim::JUMP : 0;
    caps.resolvedJumpEnd = has(anim::JUMP_END) ? anim::JUMP_END : 0;
    caps.hasJump = (caps.resolvedJumpStart != 0);

    // ── Swim ────────────────────────────────────────────────────────────
    caps.resolvedSwimIdle = has(anim::SWIM_IDLE) ? anim::SWIM_IDLE : 0;
    caps.resolvedSwim = has(anim::SWIM) ? anim::SWIM : 0;
    caps.hasSwim = (caps.resolvedSwimIdle != 0 || caps.resolvedSwim != 0);
    {
        static const uint32_t swimBackCands[] = {anim::SWIM_BACKWARDS, anim::SWIM};
        caps.resolvedSwimBackwards = pick(swimBackCands, 2);
    }
    {
        static const uint32_t swimLeftCands[] = {anim::SWIM_LEFT, anim::SWIM};
        caps.resolvedSwimLeft = pick(swimLeftCands, 2);
    }
    {
        static const uint32_t swimRightCands[] = {anim::SWIM_RIGHT, anim::SWIM};
        caps.resolvedSwimRight = pick(swimRightCands, 2);
    }

    // ── Melee combat ────────────────────────────────────────────────────
    // The chains are in melee_anim_chains.hpp, which the controller that
    // plays them reads too. They were written out in both files, matching,
    // under a comment here saying so.
    caps.resolvedMelee1H = pickFrom(anim::MeleeChain::OneHand);
    caps.resolvedMelee2H = pickFrom(anim::MeleeChain::TwoHand);
    caps.resolvedMelee2HLoose = pickFrom(anim::MeleeChain::TwoHandLoose);
    caps.resolvedMeleeUnarmed = pickFrom(anim::MeleeChain::Unarmed);
    caps.resolvedMeleeFist = pickFrom(anim::MeleeChain::Fist);
    caps.resolvedMeleePierce = pickFrom(anim::MeleeChain::Dagger);
    caps.resolvedMeleeOffHand = pickFrom(anim::MeleeChain::OffHand);
    caps.resolvedMeleeOffHandFist = pickFrom(anim::MeleeChain::OffHandFist);
    caps.resolvedMeleeOffHandPierce = pickFrom(anim::MeleeChain::OffHandPierce);
    caps.resolvedMeleeOffHandUnarmed = pickFrom(anim::MeleeChain::OffHandUnarmed);
    caps.hasMelee = (caps.resolvedMelee1H != 0 || caps.resolvedMeleeUnarmed != 0);

    // ── Ready stances ───────────────────────────────────────────────────
    {
        static const uint32_t ready1HCands[] = {
            anim::READY_1H, anim::READY_2H, anim::READY_UNARMED};
        caps.resolvedReady1H = pick(ready1HCands, 3);
    }
    {
        static const uint32_t ready2HCands[] = {
            anim::READY_2H, anim::READY_2H_LOOSE, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReady2H = pick(ready2HCands, 4);
    }
    {
        static const uint32_t ready2HLooseCands[] = {
            anim::READY_2H_LOOSE, anim::READY_2H, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReady2HLoose = pick(ready2HLooseCands, 4);
    }
    {
        static const uint32_t readyUnarmedCands[] = {
            anim::READY_UNARMED, anim::READY_1H, anim::READY_FIST};
        caps.resolvedReadyUnarmed = pick(readyUnarmedCands, 3);
    }
    {
        static const uint32_t readyFistCands[] = {
            anim::READY_FIST_1H, anim::READY_FIST, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyFist = pick(readyFistCands, 4);
    }
    {
        static const uint32_t readyBowCands[] = {
            anim::READY_BOW, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyBow = pick(readyBowCands, 3);
    }
    {
        static const uint32_t readyRifleCands[] = {
            anim::READY_RIFLE, anim::READY_BOW, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyRifle = pick(readyRifleCands, 4);
    }
    {
        static const uint32_t readyCrossbowCands[] = {
            anim::READY_CROSSBOW, anim::READY_BOW, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyCrossbow = pick(readyCrossbowCands, 4);
    }
    {
        static const uint32_t readyThrownCands[] = {
            anim::READY_THROWN, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyThrown = pick(readyThrownCands, 3);
    }
    {
        // A wand takes the directed spell stance, not the rifle's.
        static const uint32_t readyWandCands[] = {
            anim::READY_SPELL_DIRECTED, anim::READY_1H, anim::READY_UNARMED};
        caps.resolvedReadyWand = pick(readyWandCands, 3);
    }

    // ── Ranged attacks ──────────────────────────────────────────────────
    {
        static const uint32_t fireBowCands[] = {anim::FIRE_BOW, anim::ATTACK_BOW};
        caps.resolvedFireBow = pick(fireBowCands, 2);
    }
    caps.resolvedAttackRifle = has(anim::ATTACK_RIFLE) ? anim::ATTACK_RIFLE : 0;
    {
        static const uint32_t attackCrossbowCands[] = {anim::ATTACK_CROSSBOW, anim::ATTACK_BOW};
        caps.resolvedAttackCrossbow = pick(attackCrossbowCands, 2);
    }
    caps.resolvedAttackThrown = has(anim::ATTACK_THROWN) ? anim::ATTACK_THROWN : 0;
    {
        // The shot, matching the stance above.
        static const uint32_t attackWandCands[] = {
            anim::SPELL_CAST_DIRECTED, anim::ATTACK_RIFLE};
        caps.resolvedAttackWand = pick(attackWandCands, 2);
    }
    caps.resolvedLoadBow = has(anim::LOAD_BOW) ? anim::LOAD_BOW : 0;
    caps.resolvedLoadRifle = has(anim::LOAD_RIFLE) ? anim::LOAD_RIFLE : 0;

    // ── Special attacks ─────────────────────────────────────────────────
    caps.resolvedSpecial1H = has(anim::SPECIAL_1H) ? anim::SPECIAL_1H : 0;
    caps.resolvedSpecial2H = has(anim::SPECIAL_2H) ? anim::SPECIAL_2H : 0;
    caps.resolvedSpecialUnarmed = has(anim::SPECIAL_UNARMED) ? anim::SPECIAL_UNARMED : 0;
    caps.resolvedShieldBash = has(anim::SHIELD_BASH) ? anim::SHIELD_BASH : 0;

    // ── Combat idle ─────────────────────────────────────────────────────
    // Base combat idle - weapon-specific stances are resolved per-frame
    // using ready stance fields above
    caps.resolvedCombatIdle = has(anim::READY_1H) ? anim::READY_1H
                            : (has(anim::READY_UNARMED) ? anim::READY_UNARMED : 0);

    // ── Activity animations ─────────────────────────────────────────────
    caps.resolvedStandWound = has(anim::STAND_WOUND) ? anim::STAND_WOUND : 0;
    caps.resolvedDeath = has(anim::DEATH) ? anim::DEATH : 0;
    caps.hasDeath = (caps.resolvedDeath != 0);
    caps.resolvedLoot = has(anim::LOOT) ? anim::LOOT : 0;
    caps.resolvedSitDown = has(anim::SIT_GROUND_DOWN) ? anim::SIT_GROUND_DOWN : 0;
    caps.resolvedSitLoop = has(anim::SITTING) ? anim::SITTING : 0;
    caps.resolvedSitUp = has(anim::SIT_GROUND_UP) ? anim::SIT_GROUND_UP : 0;
    caps.resolvedKneel = has(anim::KNEEL_LOOP) ? anim::KNEEL_LOOP : 0;

    // ── Stealth ─────────────────────────────────────────────────────────
    caps.resolvedStealthIdle = has(anim::STEALTH_STAND) ? anim::STEALTH_STAND : 0;
    caps.resolvedStealthWalk = has(anim::STEALTH_WALK) ? anim::STEALTH_WALK : 0;
    caps.resolvedStealthRun = has(anim::STEALTH_RUN) ? anim::STEALTH_RUN : 0;
    caps.hasStealth = (caps.resolvedStealthIdle != 0);

    // ── Misc ────────────────────────────────────────────────────────────
    caps.resolvedMount = has(anim::MOUNT) ? anim::MOUNT : 0;
    caps.hasMount = (caps.resolvedMount != 0);
    caps.resolvedUnsheathe = has(anim::UNSHEATHE) ? anim::UNSHEATHE : 0;
    {
        static const uint32_t sheatheCands[] = {anim::SHEATHE, anim::HIP_SHEATHE};
        caps.resolvedSheathe = pick(sheatheCands, 2);
    }
    caps.resolvedStun = has(anim::STUN) ? anim::STUN : 0;
    caps.resolvedCombatWound = has(anim::COMBAT_WOUND) ? anim::COMBAT_WOUND : 0;

    return caps;
}
} // namespace rendering
} // namespace wowee
