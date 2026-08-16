#pragma once

#include "rendering/animation/i_character_animator.hpp"
#include "rendering/animation/anim_capability_set.hpp"
#include "rendering/animation/locomotion_fsm.hpp"
#include "rendering/animation/combat_fsm.hpp"
#include "rendering/animation/activity_fsm.hpp"
#include "rendering/animation/mount_fsm.hpp"
#include <cstdint>

namespace wowee {
namespace rendering {

// ============================================================================
// CharacterAnimator
//
// Generic animator for any character (player, NPC, companion).
// Composes LocomotionFSM, CombatFSM, ActivityFSM, and MountFSM with
// a priority resolver. Implements ICharacterAnimator.
//
// Priority order: Stun > HitReaction > Spell > Charge > Combat > Activity > Locomotion
//
// No idle fallback: if all FSMs return {valid=false}, last animation continues.
//
// Overlay layer (stealth, sprint) substitutes the resolved anim without
// changing sub-FSM state.
// ============================================================================
class CharacterAnimator final : public ICharacterAnimator {
public:
    CharacterAnimator();

    // ── IAnimator ───────────────────────────────────────────────────────
    void onEvent(AnimEvent event) override;
    void update(float dt) override;

    // ── ICharacterAnimator ──────────────────────────────────────────────
    void startSpellCast(uint32_t precast, uint32_t cast, bool loop, uint32_t finalize) override;
    void stopSpellCast() override;
    void triggerMeleeSwing() override;
    void triggerRangedShot() override;
    void triggerHitReaction(uint32_t animId) override;
    void triggerSpecialAttack(uint32_t spellId) override;
    void setEquippedWeaponType(const WeaponLoadout& loadout) override;
    void setEquippedRangedType(RangedWeaponType type) override;
    void setRangedWeaponActive(bool active) override;
    void playEmote(uint32_t animId, bool loop) override;
    void cancelEmote() override;
    void startLooting() override;
    void stopLooting() override;
    void setStunned(bool stunned) override;
    void setCharging(bool charging) override;
    void setStandState(uint8_t state) override;
    void setStealthed(bool stealth) override;
    void setInCombat(bool combat) override;
    void setLowHealth(bool low) override;
    void setSprintAuraActive(bool active) override;

    // ── Configuration ───────────────────────────────────────────────────
    void setCapabilities(const AnimCapabilitySet& caps) { caps_ = caps; }
    [[nodiscard]] const AnimCapabilitySet& getCapabilities() const { return caps_; }
    void setWeaponLoadout(const WeaponLoadout& loadout) { loadout_ = loadout; }
    [[nodiscard]] const WeaponLoadout& getWeaponLoadout() const { return loadout_; }

    // ── Mount ───────────────────────────────────────────────────────────
    void configureMountFSM(const MountFSM::MountAnimSet& anims, bool taxiFlight);
    void clearMountFSM();
    [[nodiscard]] bool isMountActive() const { return mount_.isActive(); }
    MountFSM& getMountFSM() { return mount_; }

    // ── Sub-FSM access (for transition queries) ─────────────────────────
    LocomotionFSM& getLocomotion() { return locomotion_; }
    [[nodiscard]] const LocomotionFSM& getLocomotion() const { return locomotion_; }
    CombatFSM& getCombat() { return combat_; }
    [[nodiscard]] const CombatFSM& getCombat() const { return combat_; }
    ActivityFSM& getActivity() { return activity_; }
    [[nodiscard]] const ActivityFSM& getActivity() const { return activity_; }

    // ── Last resolved output ────────────────────────────────────────────
    [[nodiscard]] AnimOutput getLastOutput() const { return lastOutput_; }

    // ── Input injection (set per-frame from AnimationController) ────────
    struct FrameInput {
        // From camera controller
        bool moving = false;
        bool sprinting = false;
        bool movingForward = false;
        bool movingBackward = false;
        bool strafeLeft = false;
        bool strafeRight = false;
        bool grounded = true;
        bool jumping = false;
        bool swimming = false;
        bool sitting = false;
        // No flyingActive. The mount path asks the camera controller directly,
        // and an unmounted flying character has no animation path at all -
        // ActivityFSM branches on swimming, jumping, moving and sitting, and
        // there is no flight among them. That is a gap rather than something
        // this field was covering: it was filled in every frame and read
        // nowhere, which is a harder thing to notice than a missing branch.
        bool ascending = false;
        bool descending = false;
        bool jumpKeyPressed = false;
        float characterYaw = 0.0f;
        // Melee/ranged timers
        float meleeSwingTimer = 0.0f;
        float rangedShootTimer = 0.0f;
        uint32_t specialAttackAnimId = 0;
        uint32_t rangedAnimId = 0;
        // Animation state query
        uint32_t currentAnimId = 0;
        float currentAnimTime = 0.0f;
        float currentAnimDuration = 0.0f;
        bool haveAnimState = false;
        // Mount state query
        uint32_t curMountAnim = 0;
        float curMountTime = 0.0f;
        float curMountDuration = 0.0f;
        bool haveMountState = false;
    };

    void setFrameInput(const FrameInput& input) { frameInput_ = input; }

private:
    AnimCapabilitySet caps_;
    WeaponLoadout loadout_;

    LocomotionFSM locomotion_;
    CombatFSM     combat_;
    ActivityFSM   activity_;
    MountFSM      mount_;

    // Overlay flags
    bool stealthed_ = false;
    bool sprintAura_ = false;
    bool lowHealth_ = false;
    bool inCombat_ = false;
    bool rangedWeaponActive_ = false;

    float lastDt_ = 0.0f;
    FrameInput frameInput_;
    AnimOutput lastOutput_;

    /// Priority resolver: highest-priority active FSM wins.
    AnimOutput resolveAnimation();

    /// Apply stealth/sprint overlays to the resolved animation.
    [[nodiscard]] AnimOutput applyOverlays(AnimOutput base) const;
};

} // namespace rendering
} // namespace wowee
