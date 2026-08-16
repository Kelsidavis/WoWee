#pragma once

#include "rendering/animation/anim_capability_set.hpp"
#include "rendering/animation/anim_event.hpp"
#include <cstdint>
#include <vector>
#include <random>
#include <glm/glm.hpp>

namespace wowee {
namespace rendering {

// ============================================================================
// MountFSM
//
// Self-contained mount animation state machine. Replaces the ~400-line
// mounted branch of updateCharacterAnimation() and eliminates the `goto`.
//
// Owns: fidget timer, RNG, idle sound timer, mount action state.
// All static RNG replaced with per-instance members.
// ============================================================================
class MountFSM {
public:
    // Animation set discovered at mount time (property-based, not hardcoded)
    struct MountAnimSet {
        uint32_t jumpStart = 0;
        uint32_t jumpLoop = 0;
        uint32_t jumpEnd = 0;
        uint32_t rearUp = 0;
        uint32_t run = 0;
        uint32_t runLeft = 0;
        uint32_t runRight = 0;
        uint32_t stand = 0;
        // Flight animations
        uint32_t flyIdle = 0;
        uint32_t flyForward = 0;
        uint32_t flyBackwards = 0;
        uint32_t flyLeft = 0;
        uint32_t flyRight = 0;
        uint32_t flyUp = 0;
        uint32_t flyDown = 0;
        std::vector<uint32_t> fidgets;
    };

    enum class MountState : uint8_t {
        IDLE, RUN, JUMP_START, JUMP_LOOP, JUMP_LAND, REAR_UP, FLY,
    };

    enum class MountAction : uint8_t { None, Jump, RearUp };

    struct Input {
        bool moving = false;
        bool movingBackward = false;
        bool strafeLeft = false;
        bool strafeRight = false;
        bool grounded = true;
        bool jumpKeyPressed = false;
        bool flying = false;
        bool swimming = false;
        bool ascending = false;
        bool descending = false;
        bool taxiFlight = false;
        float deltaTime = 0.0f;
        float characterYaw = 0.0f;
        // Mount anim state query
        uint32_t curMountAnim = 0;
        float curMountTime = 0.0f;
        float curMountDuration = 0.0f;
        bool haveMountState = false;
    };

    /// Output from evaluate(): what to play on rider + mount, and positioning data.
    struct Output {
        // Mount animation
        uint32_t mountAnimId = 0;
        bool mountAnimLoop = true;
        bool mountAnimChanged = false;  // true = should call playAnimation

        // No rider animation here. The caller resolves what the rider does from
        // its own capability set - a flight variant of the seated pose where the
        // model has one - and never read the three fields this used to carry,
        // one of which was never even written.

        // Mount procedural motion
        float mountBob = 0.0f;         // Vertical bob offset
        float mountPitch = 0.0f;       // Pitch (forward lean)
        float mountRoll = 0.0f;        // Roll (banking)

        // Signals
        bool playJumpSound = false;
        bool playLandSound = false;
        bool playRearUpSound = false;
        bool playIdleSound = false;
        bool triggerMountJump = false;  // Tell camera controller to jump
    };

    void configure(const MountAnimSet& anims, bool taxiFlight);
    void clear();
    void onEvent(AnimEvent event);

    /// Main evaluation: produces Output describing what to play.
    Output evaluate(const Input& in);

    [[nodiscard]] bool isActive() const { return active_; }
    [[nodiscard]] MountState getState() const { return state_; }
    [[nodiscard]] MountAction getAction() const { return action_; }
    [[nodiscard]] const MountAnimSet& getAnims() const { return anims_; }

private:
    bool active_ = false;
    MountState state_ = MountState::IDLE;
    MountAction action_ = MountAction::None;
    uint32_t actionPhase_ = 0;
    // Animation the current action asked the renderer to play. One-shot anims
    // are auto-switched to STAND by the renderer when they finish, so seeing a
    // different anim than requested also means the action anim completed.
    uint32_t actionAnimId_ = 0;

    MountAnimSet anims_;
    bool taxiFlight_ = false;

    // Fidget system - per-instance, not static
    float fidgetTimer_ = 0.0f;
    float nextFidgetTime_ = 30.0f;
    uint32_t activeFidget_ = 0;
    std::mt19937 rng_;

    // Idle ambient sound timer
    float idleSoundTimer_ = 0.0f;
    float nextIdleSoundTime_ = 60.0f;

    // Procedural lean
    float prevYaw_ = 0.0f;
    float roll_ = 0.0f;

    // Last mount animation for change detection
    uint32_t lastMountAnim_ = 0;

    /// Resolve the mount animation for the given input (non-taxi).
    [[nodiscard]] uint32_t resolveGroundOrFlyAnim(const Input& in) const;

    /// Check if an action animation has completed.
    [[nodiscard]] bool actionAnimComplete(const Input& in) const;
};

} // namespace rendering
} // namespace wowee
