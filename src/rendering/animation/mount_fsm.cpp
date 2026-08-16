#include "rendering/animation/mount_fsm.hpp"
#include "core/coordinates.hpp"
#include "rendering/animation/animation_ids.hpp"
#include <algorithm>
#include <cmath>

namespace wowee {
namespace rendering {

// ── Configure / Clear ────────────────────────────────────────────────────────

void MountFSM::configure(const MountAnimSet& anims, bool taxiFlight) {
    anims_ = anims;
    taxiFlight_ = taxiFlight;
    active_ = true;
    state_ = MountState::IDLE;
    action_ = MountAction::None;
    actionPhase_ = 0;
    actionAnimId_ = 0;
    fidgetTimer_ = 0.0f;
    activeFidget_ = 0;
    idleSoundTimer_ = 0.0f;
    prevYaw_ = 0.0f;
    roll_ = 0.0f;
    lastMountAnim_ = 0;

    // Seed per-instance RNG
    std::random_device rd;
    rng_.seed(rd());
    // Mount fidgets are noticeable full-body animations. Keep them occasional;
    // the old 6-12 second cadence made horses look continuously restless.
    nextFidgetTime_ = std::uniform_real_distribution<float>(20.0f, 45.0f)(rng_);
    nextIdleSoundTime_ = std::uniform_real_distribution<float>(45.0f, 90.0f)(rng_);
}

void MountFSM::clear() {
    active_ = false;
    state_ = MountState::IDLE;
    action_ = MountAction::None;
    actionPhase_ = 0;
    actionAnimId_ = 0;
    taxiFlight_ = false;
    anims_ = {};
    fidgetTimer_ = 0.0f;
    activeFidget_ = 0;
    idleSoundTimer_ = 0.0f;
    lastMountAnim_ = 0;
}

// ── Event handling ───────────────────────────────────────────────────────────

void MountFSM::onEvent(AnimEvent event) {
    if (!active_) return;
    switch (event) {
        case AnimEvent::JUMP:
            // Jump only triggered via evaluate() input check
            break;
        case AnimEvent::DISMOUNT:
            clear();
            break;
        default:
            break;
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

bool MountFSM::actionAnimComplete(const Input& in) const {
    if (!in.haveMountState) return false;
    // The renderer auto-switches finished one-shot animations to STAND. When
    // that happens before this poll runs, the timeline we read belongs to
    // STAND and never reports completion, stranding the action forever (horse
    // frozen in stand pose after a jump). Playing something other than the
    // requested action anim therefore also means the action anim finished.
    if (actionAnimId_ != 0 && in.curMountAnim != actionAnimId_) return true;
    return in.curMountDuration > 0.1f &&
           (in.curMountTime >= in.curMountDuration - 0.05f);
}

uint32_t MountFSM::resolveGroundOrFlyAnim(const Input& in) const {
    const bool pureStrafe = !in.movingBackward;
    const bool anyStrafeLeft = in.strafeLeft && !in.strafeRight && pureStrafe;
    const bool anyStrafeRight = in.strafeRight && !in.strafeLeft && pureStrafe;

    if (in.moving) {
        if (in.flying) {
            if (in.ascending) {
                return anims_.flyUp ? anims_.flyUp : (anims_.flyForward ? anims_.flyForward : anim::RUN);
            } else if (in.descending) {
                return anims_.flyDown ? anims_.flyDown : (anims_.flyForward ? anims_.flyForward : anim::RUN);
            } else if (anyStrafeLeft) {
                return anims_.flyLeft ? anims_.flyLeft : (anims_.flyForward ? anims_.flyForward : anim::RUN);
            } else if (anyStrafeRight) {
                return anims_.flyRight ? anims_.flyRight : (anims_.flyForward ? anims_.flyForward : anim::RUN);
            } else if (in.movingBackward) {
                return anims_.flyBackwards ? anims_.flyBackwards : (anims_.flyForward ? anims_.flyForward : anim::RUN);
            } else {
                return anims_.flyForward ? anims_.flyForward : (anims_.flyIdle ? anims_.flyIdle : anim::RUN);
            }
        } else if (in.swimming) {
            // Mounted swimming - simplified, no per-direction mount swim anims needed here
            // (the original code used pickMountAnim with mount-specific swim IDs)
            return anims_.run ? anims_.run : anim::RUN;
        } else if (anyStrafeLeft) {
            return anims_.runLeft ? anims_.runLeft : (anims_.run ? anims_.run : anim::RUN);
        } else if (anyStrafeRight) {
            return anims_.runRight ? anims_.runRight : (anims_.run ? anims_.run : anim::RUN);
        } else if (in.movingBackward) {
            return anims_.run ? anims_.run : anim::RUN;
        } else {
            return anim::RUN;
        }
    } else {
        // Idle
        if (in.swimming) {
            return anims_.stand ? anims_.stand : anim::STAND;
        } else if (in.flying) {
            if (in.ascending) {
                return anims_.flyUp ? anims_.flyUp : (anims_.flyIdle ? anims_.flyIdle : anim::STAND);
            } else if (in.descending) {
                return anims_.flyDown ? anims_.flyDown : (anims_.flyIdle ? anims_.flyIdle : anim::STAND);
            } else {
                return anims_.flyIdle ? anims_.flyIdle : (anims_.flyForward ? anims_.flyForward : anim::STAND);
            }
        } else {
            return anims_.stand ? anims_.stand : anim::STAND;
        }
    }
}

// ── Main evaluation ──────────────────────────────────────────────────────────

MountFSM::Output MountFSM::evaluate(const Input& in) {
    Output out;
    if (!active_) return out;

    const float dt = in.deltaTime;

    // On a taxi, as of this frame.
    //
    // The configured flag is only what was true when the mount was set up, and
    // a taxi sets the mount up before it says the flight has begun - so on
    // every flight it was false, the branch below was skipped, and the gryphon
    // resolved through the ordinary ground path and ran through the air. The
    // per-frame answer was already being passed in and read by nothing; the
    // comment where the mount is configured has described this the whole time.
    //
    // Either one counts, so a mount configured mid-flight still flies.
    const bool onTaxi = in.taxiFlight || taxiFlight_;

    // ── Procedural lean ─────────────────────────────────────────────────
    if (!onTaxi && in.moving && dt > 0.0f) {
        float turnRate = (in.characterYaw - prevYaw_) / dt;
        while (turnRate > 180.0f) turnRate -= 360.0f;
        while (turnRate < -180.0f) turnRate += 360.0f;
        float targetLean = std::clamp(turnRate * 0.15f, -0.25f, 0.25f);
        roll_ = roll_ + (targetLean - roll_) * (1.0f - std::exp(-6.0f * dt));
    } else {
        roll_ = roll_ + (0.0f - roll_) * (1.0f - std::exp(-8.0f * dt));
    }
    prevYaw_ = in.characterYaw;
    out.mountRoll = roll_;

    // ── Rider animation ─────────────────────────────────────────────────
    // The rider is the caller's to resolve; see MountFSM::Output.

    // ── Taxi flight branch ──────────────────────────────────────────────
    if (onTaxi) {
        // Try flight animations in preference order using discovered anims
        uint32_t taxiAnim = anim::STAND;
        if (anims_.flyForward) taxiAnim = anims_.flyForward;
        else if (anims_.flyIdle) taxiAnim = anims_.flyIdle;
        else if (anims_.run) taxiAnim = anims_.run;

        out.mountAnimId = taxiAnim;
        out.mountAnimLoop = true;
        out.mountAnimChanged = (!in.haveMountState || in.curMountAnim != taxiAnim);

        // Bob calculation for taxi
        if (in.moving && in.haveMountState && in.curMountDuration > 1.0f) {
            float wrappedTime = in.curMountTime;
            while (wrappedTime >= in.curMountDuration) wrappedTime -= in.curMountDuration;
            float norm = wrappedTime / in.curMountDuration;
            out.mountBob = std::sin(norm * core::coords::TWO_PI * 2.0f) * 0.12f;
        }

        lastMountAnim_ = out.mountAnimId;
        return out;
    }

    // ── Jump/rear-up trigger ────────────────────────────────────────────
    if (in.jumpKeyPressed && in.grounded && action_ == MountAction::None) {
        if (in.moving && anims_.jumpLoop > 0) {
            action_ = MountAction::Jump;
            actionPhase_ = 1; // Start with loop directly (matching original)
            actionAnimId_ = anims_.jumpLoop;
            out.mountAnimId = anims_.jumpLoop;
            out.mountAnimLoop = true;
            out.mountAnimChanged = true;
            out.playJumpSound = true;
            out.triggerMountJump = true;
            lastMountAnim_ = out.mountAnimId;

            // Bob calc
            if (in.haveMountState && in.curMountDuration > 1.0f) {
                float wrappedTime = in.curMountTime;
                while (wrappedTime >= in.curMountDuration) wrappedTime -= in.curMountDuration;
                float norm = wrappedTime / in.curMountDuration;
                out.mountBob = std::sin(norm * core::coords::TWO_PI) * 0.12f;
            }
            return out;
        } if (!in.moving && anims_.rearUp > 0) {
            action_ = MountAction::RearUp;
            actionPhase_ = 0;
            actionAnimId_ = anims_.rearUp;
            out.mountAnimId = anims_.rearUp;
            out.mountAnimLoop = false;
            out.mountAnimChanged = true;
            out.playRearUpSound = true;
            lastMountAnim_ = out.mountAnimId;
            return out;
        }
    }

    // ── Handle active mount actions (jump chaining or rear-up) ──────────
    if (action_ != MountAction::None) {
        bool animFinished = actionAnimComplete(in);

        if (action_ == MountAction::Jump) {
            if (actionPhase_ == 0 && animFinished && anims_.jumpLoop > 0) {
                actionPhase_ = 1;
                actionAnimId_ = anims_.jumpLoop;
                out.mountAnimId = anims_.jumpLoop;
                out.mountAnimLoop = true;
                out.mountAnimChanged = true;
            } else if (actionPhase_ == 0 && animFinished) {
                actionPhase_ = 1;
                actionAnimId_ = 0;
                out.mountAnimId = in.curMountAnim;
            } else if (actionPhase_ == 1 && in.grounded && anims_.jumpEnd > 0) {
                actionPhase_ = 2;
                actionAnimId_ = anims_.jumpEnd;
                out.mountAnimId = anims_.jumpEnd;
                out.mountAnimLoop = false;
                out.mountAnimChanged = true;
                out.playLandSound = true;
            } else if (actionPhase_ == 1 && in.grounded) {
                action_ = MountAction::None;
                actionAnimId_ = 0;
                out.mountAnimId = in.moving ? anims_.run : anims_.stand;
                out.mountAnimLoop = true;
                out.mountAnimChanged = true;
            } else if (actionPhase_ == 2 && animFinished) {
                action_ = MountAction::None;
                actionAnimId_ = 0;
                out.mountAnimId = in.moving ? anims_.run : anims_.stand;
                out.mountAnimLoop = true;
                out.mountAnimChanged = true;
            } else {
                out.mountAnimId = in.curMountAnim;
            }
        } else if (action_ == MountAction::RearUp) {
            if (animFinished) {
                action_ = MountAction::None;
                actionAnimId_ = 0;
                out.mountAnimId = in.moving ? anims_.run : anims_.stand;
                out.mountAnimLoop = true;
                out.mountAnimChanged = true;
            } else {
                out.mountAnimId = in.curMountAnim;
            }
        }

        // Bob calc
        if (in.moving && in.haveMountState && in.curMountDuration > 1.0f) {
            float wrappedTime = in.curMountTime;
            while (wrappedTime >= in.curMountDuration) wrappedTime -= in.curMountDuration;
            float norm = wrappedTime / in.curMountDuration;
            out.mountBob = std::sin(norm * core::coords::TWO_PI) * 0.12f;
        }
        lastMountAnim_ = out.mountAnimId;
        return out;
    }

    // ── Normal movement animation resolution ────────────────────────────
    uint32_t mountAnimId = resolveGroundOrFlyAnim(in);

    // ── Cancel active fidget on movement ────────────────────────────────
    if (in.moving && activeFidget_ != 0) {
        activeFidget_ = 0;
        out.mountAnimId = mountAnimId;
        out.mountAnimLoop = true;
        out.mountAnimChanged = true;
        lastMountAnim_ = out.mountAnimId;

        // Bob calc
        if (in.haveMountState && in.curMountDuration > 1.0f) {
            float wrappedTime = in.curMountTime;
            while (wrappedTime >= in.curMountDuration) wrappedTime -= in.curMountDuration;
            float norm = wrappedTime / in.curMountDuration;
            out.mountBob = std::sin(norm * core::coords::TWO_PI) * 0.12f;
        }
        return out;
    }

    // ── Check if active fidget completed ────────────────────────────────
    if (!in.moving && activeFidget_ != 0) {
        if (in.haveMountState) {
            if (in.curMountAnim != activeFidget_ ||
                in.curMountTime >= in.curMountDuration * 0.95f) {
                activeFidget_ = 0;
            }
        }
    }

    // ── Idle fidgets ────────────────────────────────────────────────────
    if (!in.moving && action_ == MountAction::None && activeFidget_ == 0 && !anims_.fidgets.empty()) {
        fidgetTimer_ += dt;
        if (fidgetTimer_ >= nextFidgetTime_) {
            std::uniform_int_distribution<size_t> dist(0, anims_.fidgets.size() - 1);
            uint32_t fidgetAnim = anims_.fidgets[dist(rng_)];
            activeFidget_ = fidgetAnim;
            fidgetTimer_ = 0.0f;
            nextFidgetTime_ = std::uniform_real_distribution<float>(20.0f, 45.0f)(rng_);

            out.mountAnimId = fidgetAnim;
            out.mountAnimLoop = false;
            out.mountAnimChanged = true;
            lastMountAnim_ = out.mountAnimId;
            return out;
        }
    }
    if (in.moving) fidgetTimer_ = 0.0f;

    // ── Idle ambient sounds ─────────────────────────────────────────────
    if (!in.moving) {
        idleSoundTimer_ += dt;
        if (idleSoundTimer_ >= nextIdleSoundTime_) {
            out.playIdleSound = true;
            idleSoundTimer_ = 0.0f;
            nextIdleSoundTime_ = std::uniform_real_distribution<float>(45.0f, 90.0f)(rng_);
        }
    } else {
        idleSoundTimer_ = 0.0f;
    }

    // ── Set output ──────────────────────────────────────────────────────
    out.mountAnimId = activeFidget_ != 0 ? activeFidget_ : mountAnimId;
    out.mountAnimLoop = (activeFidget_ == 0);
    // Only trigger playAnimation if animation actually changed and no action/fidget active
    if (action_ == MountAction::None && activeFidget_ == 0 &&
        (!in.haveMountState || in.curMountAnim != mountAnimId)) {
        out.mountAnimChanged = true;
        out.mountAnimId = mountAnimId;
    }

    // Bob calculation
    if (in.moving && in.haveMountState && in.curMountDuration > 1.0f) {
        float wrappedTime = in.curMountTime;
        while (wrappedTime >= in.curMountDuration) wrappedTime -= in.curMountDuration;
        float norm = wrappedTime / in.curMountDuration;
        float bobSpeed = taxiFlight_ ? 2.0f : 1.0f;
        out.mountBob = std::sin(norm * core::coords::TWO_PI * bobSpeed) * 0.12f;
    }

    lastMountAnim_ = out.mountAnimId;
    return out;
}

} // namespace rendering
} // namespace wowee
