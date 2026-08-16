#pragma once

#include "rendering/animation/anim_capability_set.hpp"
#include "rendering/animation/anim_event.hpp"
#include <cstdint>
#include <string>

namespace wowee {
namespace rendering {

// ============================================================================
// ActivityFSM
//
// Pure logic state machine for non-combat activities: emote, loot, sit/sleep/kneel.
// Sit chain (down → loop → up) auto-advances on one-shot completion.
// All activities cancel on movement.
// ============================================================================
class ActivityFSM {
public:
    enum class State : uint8_t {
        NONE,
        EMOTE,
        LOOTING,        // One-shot LOOT anim
        LOOT_KNEELING,  // KNEEL_LOOP until loot window closes
        LOOT_END,       // One-shot KNEEL_END exit anim
        SIT_DOWN,
        SITTING,
        SIT_UP,
    };

    struct Input {
        bool moving = false;
        bool sprinting = false;
        bool jumping = false;
        bool grounded = true;
        bool swimming = false;
        bool sitting = false;    // Camera controller sitting state
        // No stunned here: CombatFSM holds that state and resolves before this
        // one, returning the stun pose itself, so this was filled in from
        // CombatFSM::isStunned and then never looked at.
        // Animation state query for one-shot completion detection
        uint32_t currentAnimId = 0;
        float currentAnimTime = 0.0f;
        float currentAnimDuration = 0.0f;
        bool haveAnimState = false;
    };

    void onEvent(AnimEvent event);

    /// Evaluate current state against input and capabilities.
    AnimOutput resolve(const Input& in, const AnimCapabilitySet& caps);

    [[nodiscard]] State getState() const { return state_; }
    void setState(State s) { state_ = s; }
    [[nodiscard]] bool isActive() const { return state_ != State::NONE; }
    void reset();

    // ── Emote management ────────────────────────────────────────────────
    void startEmote(uint32_t animId, bool loop);
    void cancelEmote();
    [[nodiscard]] bool isEmoteActive() const { return emoteActive_; }
    [[nodiscard]] uint32_t getEmoteAnimId() const { return emoteAnimId_; }

    // ── Sit/sleep/kneel management ──────────────────────────────────────
    // WoW UnitStandStateType constants
    static constexpr uint8_t STAND_STATE_STAND    = 0;
    static constexpr uint8_t STAND_STATE_SIT      = 1;
    static constexpr uint8_t STAND_STATE_SIT_CHAIR = 2;
    static constexpr uint8_t STAND_STATE_SLEEP    = 3;
    static constexpr uint8_t STAND_STATE_SIT_LOW  = 4;
    static constexpr uint8_t STAND_STATE_SIT_MED  = 5;
    static constexpr uint8_t STAND_STATE_SIT_HIGH = 6;
    static constexpr uint8_t STAND_STATE_DEAD     = 7;
    static constexpr uint8_t STAND_STATE_KNEEL    = 8;

    void setStandState(uint8_t standState);
    // Overrides the seated loop without skipping SIT_GROUND_DOWN. Passing zero
    // restores the normal seated idle.
    void setSeatedLoopAnimation(uint32_t animationId);
    [[nodiscard]] uint8_t getStandState() const { return standState_; }

    // ── Loot management ─────────────────────────────────────────────────
    void startLooting();
    void stopLooting();

    static constexpr uint8_t PRIORITY = 30;

private:
    State state_ = State::NONE;

    // Emote state
    bool emoteActive_ = false;
    uint32_t emoteAnimId_ = 0;
    bool emoteLoop_ = false;

    // Sit/sleep/kneel transition animations
    uint8_t standState_ = 0;
    uint32_t sitDownAnim_ = 0;
    uint32_t sitLoopAnim_ = 0;
    uint32_t seatedLoopOverride_ = 0;
    uint32_t sitUpAnim_ = 0;
    bool sitDownAnimSeen_ = false;  // Track whether one-shot has started playing
    bool sitUpAnimSeen_ = false;
    uint8_t sitDownFrames_ = 0;    // Frames spent in SIT_DOWN (for safety timeout)
    uint8_t sitUpFrames_ = 0;      // Frames spent in SIT_UP
    bool lootAnimSeen_ = false;
    uint8_t lootFrames_ = 0;
    bool lootEndAnimSeen_ = false;
    uint8_t lootEndFrames_ = 0;

    void updateTransitions(const Input& in);
    [[nodiscard]] bool oneShotComplete(const Input& in, uint32_t expectedAnimId) const;
};

} // namespace rendering
} // namespace wowee
