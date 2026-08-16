#pragma once

#include "rendering/animation/anim_capability_set.hpp"
#include "rendering/animation/anim_event.hpp"
#include "rendering/animation/weapon_type.hpp"
#include <cstdint>

namespace wowee {
namespace rendering {

// ============================================================================
// CombatFSM
//
// Pure logic state machine for combat animation. No renderer dependency.
// States: INACTIVE · COMBAT_IDLE · MELEE_SWING · RANGED_SHOOT · RANGED_LOAD ·
//         SPELL_PRECAST · SPELL_CASTING · SPELL_FINALIZE · HIT_REACTION ·
//         STUNNED · CHARGE · UNSHEATHE · SHEATHE
//
// Stun overrides all combat states. Spell state cleared on interrupts.
// offHandTurn_ alternation managed internally.
// ============================================================================
class CombatFSM {
public:
    enum class State : uint8_t {
        INACTIVE,
        COMBAT_IDLE,
        MELEE_SWING,
        RANGED_SHOOT,
        RANGED_LOAD,
        SPELL_PRECAST,
        SPELL_CASTING,
        SPELL_FINALIZE,
        HIT_REACTION,
        STUNNED,
        CHARGE,
        UNSHEATHE,
        SHEATHE,
    };

    struct Input {
        bool inCombat = false;
        bool grounded = true;
        bool jumping = false;
        bool swimming = false;
        bool moving = false;
        bool sprinting = false;
        bool lowHealth = false;
        bool rangedWeaponActive = false;
        float meleeSwingTimer = 0.0f;   // >0 = melee active
        float rangedShootTimer = 0.0f;  // >0 = ranged active
        uint32_t specialAttackAnimId = 0;
        uint32_t rangedAnimId = 0;
        // Animation state query for one-shot completion detection
        uint32_t currentAnimId = 0;
        float currentAnimTime = 0.0f;
        float currentAnimDuration = 0.0f;
        bool haveAnimState = false;
        // Whether model has specific one-shot animations
        bool hasUnsheathe = false;
        bool hasSheathe = false;
    };

    void onEvent(AnimEvent event);

    /// Evaluate current state against input and capabilities.
    AnimOutput resolve(const Input& in, const AnimCapabilitySet& caps,
                       const WeaponLoadout& loadout);

    [[nodiscard]] State getState() const { return state_; }
    void setState(State s) { state_ = s; }

    [[nodiscard]] bool isStunned() const { return state_ == State::STUNNED; }
    [[nodiscard]] bool isActive() const { return state_ != State::INACTIVE; }
    void reset();

    // ── Spell cast management ───────────────────────────────────────────
    void startSpellCast(uint32_t precast, uint32_t cast, bool castLoop, uint32_t finalize);
    void stopSpellCast();
    void clearSpellState();

    // ── Hit/stun management ─────────────────────────────────────────────
    void triggerHitReaction(uint32_t animId);
    void setStunned(bool stunned);
    void setCharging(bool charging);

    static constexpr uint8_t PRIORITY = 50;

private:
    State state_ = State::INACTIVE;

    // Spell cast sequence
    uint32_t spellPrecastAnimId_ = 0;
    uint32_t spellCastAnimId_ = 0;
    uint32_t spellFinalizeAnimId_ = 0;
    bool spellCastLoop_ = false;
    bool spellPrecastAnimSeen_ = false;
    uint8_t spellPrecastFrames_ = 0;
    bool spellFinalizeAnimSeen_ = false;
    uint8_t spellFinalizeFrames_ = 0;

    // Hit reaction
    uint32_t hitReactionAnimId_ = 0;

    // Stun
    bool stunned_ = false;

    // Charge
    bool charging_ = false;

    // Off-hand alternation for dual wielding
    bool offHandTurn_ = false;

    /// Internal: update state transitions based on input.
    void updateTransitions(const Input& in);

    /// Detect if a one-shot animation has completed.
    [[nodiscard]] bool oneShotComplete(const Input& in, uint32_t expectedAnimId) const;
};

} // namespace rendering
} // namespace wowee
