#pragma once

#include <cstdint>

namespace wowee {
namespace rendering {

// ============================================================================
// AnimFallbackPolicy
//
// Controls what happens when a requested animation is unavailable.
// ============================================================================
enum class AnimFallbackPolicy : uint8_t {
    STAY_IN_STATE,     // Keep current animation (default for player)
    FIRST_AVAILABLE,   // Try candidates list, stay if all fail
    NONE,              // Do nothing (default for expired queue)
};

// ============================================================================
// AnimOutput
//
// Unified animation selection result. When valid=false, callers should
// keep the currently playing animation (STAY_IN_STATE policy).
// ============================================================================
struct AnimOutput {
    uint32_t animId = 0;
    bool loop = false;
    bool valid = false;

    /// Construct a valid output.
    static AnimOutput ok(uint32_t id, bool looping) {
        return {id, looping, true};
    }
    /// Construct an invalid output (STAY policy).
    static AnimOutput stay() {
        return {0, false, false};
    }
};

// ============================================================================
// AnimCapabilitySet
//
// Probed once per model load. Caches which animations a model supports
// and the resolved IDs (after fallback chains). This eliminates per-frame
// hasAnimation() calls.
// ============================================================================
struct AnimCapabilitySet {
    // ── Locomotion resolved IDs ─────────────────────────────────────────
    uint32_t resolvedStand = 0;
    uint32_t resolvedWalk = 0;
    uint32_t resolvedRun = 0;
    uint32_t resolvedSprint = 0;
    uint32_t resolvedWalkBackwards = 0;
    uint32_t resolvedJumpStart = 0;
    uint32_t resolvedJump = 0;       // Mid-air loop
    uint32_t resolvedJumpEnd = 0;
    uint32_t resolvedSwimIdle = 0;
    uint32_t resolvedSwim = 0;
    uint32_t resolvedSwimBackwards = 0;
    uint32_t resolvedSwimLeft = 0;
    uint32_t resolvedSwimRight = 0;

    // ── Combat resolved IDs ─────────────────────────────────────────────
    uint32_t resolvedCombatIdle = 0;
    uint32_t resolvedMelee1H = 0;
    uint32_t resolvedMelee2H = 0;
    uint32_t resolvedMelee2HLoose = 0;
    uint32_t resolvedMeleeUnarmed = 0;
    uint32_t resolvedMeleeFist = 0;
    uint32_t resolvedMeleePierce = 0;   // Dagger
    uint32_t resolvedMeleeOffHand = 0;
    uint32_t resolvedMeleeOffHandFist = 0;
    uint32_t resolvedMeleeOffHandPierce = 0;
    uint32_t resolvedMeleeOffHandUnarmed = 0;

    // ── Ready stances ───────────────────────────────────────────────────
    uint32_t resolvedReady1H = 0;
    uint32_t resolvedReady2H = 0;
    uint32_t resolvedReady2HLoose = 0;
    uint32_t resolvedReadyUnarmed = 0;
    uint32_t resolvedReadyFist = 0;
    uint32_t resolvedReadyBow = 0;
    uint32_t resolvedReadyRifle = 0;
    uint32_t resolvedReadyCrossbow = 0;
    uint32_t resolvedReadyThrown = 0;
    /// A wand's stance: the directed spell pose, arm out in front.
    uint32_t resolvedReadyWand = 0;

    // ── Ranged attack resolved IDs ──────────────────────────────────────
    uint32_t resolvedFireBow = 0;
    uint32_t resolvedAttackRifle = 0;
    uint32_t resolvedAttackCrossbow = 0;
    uint32_t resolvedAttackThrown = 0;
    /// A wand's shot, which is a directed cast rather than a shouldered shot.
    uint32_t resolvedAttackWand = 0;
    uint32_t resolvedLoadBow = 0;
    uint32_t resolvedLoadRifle = 0;

    // ── Special attacks ─────────────────────────────────────────────────
    uint32_t resolvedSpecial1H = 0;
    uint32_t resolvedSpecial2H = 0;
    uint32_t resolvedSpecialUnarmed = 0;
    uint32_t resolvedShieldBash = 0;

    // ── Activity resolved IDs ───────────────────────────────────────────
    uint32_t resolvedStandWound = 0;
    uint32_t resolvedSitDown = 0;
    uint32_t resolvedSitLoop = 0;
    uint32_t resolvedSitUp = 0;
    uint32_t resolvedKneel = 0;
    uint32_t resolvedDeath = 0;

    // ── Stealth ─────────────────────────────────────────────────────────
    uint32_t resolvedStealthIdle = 0;
    uint32_t resolvedStealthWalk = 0;
    uint32_t resolvedStealthRun = 0;

    // ── Misc ────────────────────────────────────────────────────────────
    uint32_t resolvedMount = 0;
    uint32_t resolvedUnsheathe = 0;
    uint32_t resolvedSheathe = 0;
    uint32_t resolvedStun = 0;
    uint32_t resolvedCombatWound = 0;
    uint32_t resolvedLoot = 0;

    // ── Capability flags (bitfield) ─────────────────────────────────────
    bool hasStand : 1;
    bool hasWalk : 1;
    bool hasRun : 1;
    bool hasSprint : 1;
    bool hasWalkBackwards : 1;
    bool hasJump : 1;
    bool hasSwim : 1;
    bool hasMelee : 1;
    bool hasStealth : 1;
    bool hasDeath : 1;
    bool hasMount : 1;

    // Default-initialize all flags to false
    AnimCapabilitySet()
        : hasStand(false), hasWalk(false), hasRun(false), hasSprint(false),
          hasWalkBackwards(false), hasJump(false), hasSwim(false),
          hasMelee(false), hasStealth(false), hasDeath(false), hasMount(false) {}
};

} // namespace rendering
} // namespace wowee
