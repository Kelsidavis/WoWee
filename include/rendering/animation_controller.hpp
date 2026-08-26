#pragma once

#include <cstdint>
#include <functional>
#include <utility>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "rendering/animation/footstep_driver.hpp"
#include "rendering/animation/sfx_state_driver.hpp"
#include "rendering/animation/weapon_type.hpp"
#include "rendering/animation/anim_capability_set.hpp"
#include "rendering/animation/character_animator.hpp"

namespace wowee {
namespace rendering {

class Renderer;

// ============================================================================
// AnimationController - thin adapter wrapping CharacterAnimator
//
// Bridges the Renderer world (camera state, CharacterRenderer, audio)
// and the pure-logic CharacterAnimator + sub-FSMs. Public API unchanged.
//
// Responsibilities:
//   · Collect inputs from renderer/camera → CharacterAnimator::FrameInput
//   · Forward state-changing calls → CharacterAnimator
//   · Read AnimOutput from CharacterAnimator → apply via CharacterRenderer
//   · Mount discovery (needs renderer for sequence queries)
//   · Mount positioning (needs renderer for attachment queries)
//   · Melee/ranged resolution (needs renderer for sequence queries)
//   · Footstep and SFX drivers (already extracted)
// ============================================================================
class AnimationController {
public:
    AnimationController();
    ~AnimationController();

    void initialize(Renderer* renderer);

    /// Probe (or re-probe) animation capabilities for the current character model.
    /// Called once during initialize() / onCharacterFollow() and after model changes.
    void probeCapabilities();
    [[nodiscard]] const AnimCapabilitySet& getCapabilities() const { return characterAnimator_.getCapabilities(); }

    // ── Per-frame update hooks (called from Renderer::update) ──────────────
    // Runs the character animation state machine (mounted + unmounted).
    void updateCharacterAnimation();
    // Processes animation-driven footstep events (player + mount).
    void updateFootsteps(float deltaTime);
    // Tracks state transitions for activity SFX (jump, landing, swim) and
    // mount ambient sounds.
    void updateSfxState(float deltaTime);
    // Decrements melee swing timer / cooldown.
    void updateMeleeTimers(float deltaTime);
    // Store per-frame delta time (used inside animation state machine).
    void setDeltaTime(float dt) { lastDeltaTime_ = dt; }

    // ── Character follow ───────────────────────────────────────────────────
    void onCharacterFollow(uint32_t instanceId);

    // ── Emote support ──────────────────────────────────────────────────────
    void playEmote(const std::string& emoteName);
    /// Play the one-shot reach animation used by the manual Z sheath toggle.
    void playWeaponSheathAnimation(bool sheathing);
    void cancelEmote();
    [[nodiscard]] bool isEmoteActive() const { return characterAnimator_.getActivity().isEmoteActive(); }
    static std::string getEmoteText(const std::string& emoteName,
                                    const std::string* targetName = nullptr);
    static uint32_t getEmoteDbcId(const std::string& emoteName);
    static std::string getEmoteTextByDbcId(uint32_t dbcId,
                                           const std::string& senderName,
                                           const std::string* targetName = nullptr);
    static uint32_t getEmoteAnimByEmotesId(uint32_t emoteId);
    /// True if the Emotes.dbc entry is a persistent STATE_ emote (loops until
    /// cleared) rather than a one-shot.
    static bool isStateEmoteById(uint32_t emoteId);

    // ── Targeting / combat ─────────────────────────────────────────────────
    void setTargetPosition(const glm::vec3* pos);
    void setInCombat(bool combat);
    [[nodiscard]] bool isInCombat() const { return inCombat_; }
    [[nodiscard]] const glm::vec3* getTargetPosition() const { return targetPosition_; }
    void resetCombatVisualState();
    [[nodiscard]] bool isMoving() const;

    // ── Melee combat ───────────────────────────────────────────────────────
    void triggerMeleeSwing();
    /// inventoryType: WoW inventory type (0=unarmed, 13=1H, 17=2H, 21=main-hand, …)
    /// is2HLoose: true for polearms/staves (use ATTACK_2H_LOOSE instead of ATTACK_2H)
    /// isFist/isDagger describe the main hand; offHandIsFist/offHandIsDagger
    /// describe the off-hand weapon, which picks the off-hand swing.
    void setEquippedWeaponType(uint32_t inventoryType, bool is2HLoose = false,
                               bool isFist = false, bool isDagger = false,
                               bool hasOffHand = false, bool hasShield = false,
                               bool offHandIsFist = false,
                               bool offHandIsDagger = false);
    /// Play a special attack animation for a melee ability (spellId → SPECIAL_1H/2H/SHIELD_BASH/WHIRLWIND)
    void triggerSpecialAttack(uint32_t spellId);

    // ── Sprint aura animation ────────────────────────────────────────────
    void setSprintAuraActive(bool active);

    // ── Ranged combat ──────────────────────────────────────────────────────
    void setEquippedRangedType(RangedWeaponType type);
    void setRangedWeaponActive(bool active);
    void setRangedShotCompleteCallback(std::function<void()> callback) {
        rangedShotCompleteCallback_ = std::move(callback);
    }
    /// Trigger a ranged shot animation (Auto Shot, Shoot, Throw)
    void triggerRangedShot();
    [[nodiscard]] RangedWeaponType getEquippedRangedType() const { return weaponLoadout_.rangedType; }
    void setCharging(bool charging);
    [[nodiscard]] bool isCharging() const { return charging_; }

    // ── Spell casting ──────────────────────────────────────────────────────
    /// Enter spell cast animation sequence:
    ///   precastAnimId (one-shot wind-up) → castAnimId (looping hold) → finalizeAnimId (one-shot release)
    /// Any phase can be 0 to skip it.
    void startSpellCast(uint32_t precastAnimId, uint32_t castAnimId, bool castLoop,
                        uint32_t finalizeAnimId = 0);
    /// Leave spell cast animation state → plays finalization anim then idle.
    void stopSpellCast();
    void setSeatedLoopAnimation(uint32_t animationId);

    // ── Loot animation ─────────────────────────────────────────────────────
    void startLooting();
    void stopLooting();

    // ── Hit reactions ──────────────────────────────────────────────────────
    /// Play a one-shot hit reaction animation (wound, dodge, block, etc.)
    /// on the player character.  The state machine returns to the previous
    /// state once the reaction animation finishes.
    void triggerHitReaction(uint32_t animId);

    // ── Crowd control ──────────────────────────────────────────────────────
    /// Enter/exit stunned state (loops STUN animation until cleared).
    void setStunned(bool stunned);
    [[nodiscard]] bool isStunned() const { return stunned_; }

    // ── Health-based idle ──────────────────────────────────────────────────
    /// When true, idle/combat-idle will prefer STAND_WOUND if the model has it.
    void setLowHealth(bool low);

    // ── Stand state (sit/sleep/kneel transitions) ──────────────────────────
    // WoW UnitStandStateType constants
    static constexpr uint8_t STAND_STATE_STAND      = 0;
    static constexpr uint8_t STAND_STATE_SIT         = 1;
    static constexpr uint8_t STAND_STATE_SIT_CHAIR   = 2;
    static constexpr uint8_t STAND_STATE_SLEEP       = 3;
    static constexpr uint8_t STAND_STATE_SIT_LOW     = 4;
    static constexpr uint8_t STAND_STATE_SIT_MED     = 5;
    static constexpr uint8_t STAND_STATE_SIT_HIGH    = 6;
    static constexpr uint8_t STAND_STATE_DEAD        = 7;
    static constexpr uint8_t STAND_STATE_KNEEL       = 8;
    static constexpr uint8_t STAND_STATE_SUBMERGED   = 9;
    void setStandState(uint8_t state);

    // ── Stealth ────────────────────────────────────────────────────────────
    /// When true, idle/walk/run use stealth animation variants.
    void setStealthed(bool stealth);

    // ── Effect triggers ────────────────────────────────────────────────────
    void triggerLevelUpEffect(const glm::vec3& position);
    void startChargeEffect(const glm::vec3& position, const glm::vec3& direction);
    void emitChargeEffect(const glm::vec3& position, const glm::vec3& direction);
    void stopChargeEffect();

    // ── Mount ──────────────────────────────────────────────────────────────
    void setMounted(uint32_t mountInstId, uint32_t mountDisplayId,
                    float heightOffset, const std::string& modelPath = "");
    void setTaxiFlight(bool onTaxi) { taxiFlight_ = onTaxi; }
    // M2 transport riding (Deeprun Tram, Thunder Bluff lifts): the ride-lock code in
    // Application overwrites the character's render position directly every frame, so
    // the camera controller's own gravity/collision never finds real ground beneath a
    // moving platform over open track and reports not-grounded - correctly for physics
    // purposes (there genuinely isn't a static floor there), but that fed straight into
    // the animation FSM as "falling", so riders saw a falling animation play the entire
    // ride even though position tracking was correct - "kept like playing the falling
    // animation the whole time... it sure looked like it was going to [fall off]"
    // reported live. Override just the animation-facing grounded signal while actively
    // riding; leave the camera controller's real physics/collision untouched (needed
    // for walk-while-riding to keep working).
    void setM2TransportRiding(bool riding) { m2TransportRiding_ = riding; }
    void setMountPitchRoll(float pitch, float roll) { mountPitch_ = pitch; mountRoll_ = roll; }
    void clearMount();
    [[nodiscard]] bool isMounted() const { return mountInstanceId_ != 0; }
    [[nodiscard]] uint32_t getMountInstanceId() const { return mountInstanceId_; }

    // ── Query helpers (used by Renderer) ───────────────────────────────────
    [[nodiscard]] bool isFootstepAnimationState() const;
    [[nodiscard]] float getMeleeSwingTimer() const { return meleeSwingTimer_; }
    [[nodiscard]] float getMountHeightOffset() const { return mountHeightOffset_; }
    [[nodiscard]] bool isTaxiFlight() const { return taxiFlight_; }

private:
    Renderer* renderer_ = nullptr;

    // ── CharacterAnimator: owns the complete character animation FSM ─────
    CharacterAnimator characterAnimator_;
    bool capabilitiesProbed_ = false;

    // ── Animation change detection ───────────────────────────────────────
    uint32_t lastPlayerAnimRequest_ = UINT32_MAX;
    bool lastPlayerAnimLoopRequest_ = true;
    float lastDeltaTime_ = 0.0f;

    // ── Externally-queried state (mirrored to CharacterAnimator) ────────────
    const glm::vec3* targetPosition_ = nullptr;
    bool inCombat_ = false;
    bool stunned_ = false;
    bool charging_ = false;

    // ── Footstep event tracking (delegated to FootstepDriver) ────────────
    FootstepDriver footstepDriver_;

    // ── SFX transition state (delegated to SfxStateDriver) ───────────────
    SfxStateDriver sfxStateDriver_;

    // ── Melee combat (needs renderer for sequence queries) ───────────────
    float meleeSwingTimer_ = 0.0f;
    float meleeSwingCooldown_ = 0.0f;
    float meleeAnimDurationMs_ = 0.0f;
    uint32_t meleeAnimId_ = 0;
    uint32_t specialAttackAnimId_ = 0;
    WeaponLoadout weaponLoadout_;
    bool meleeOffHandTurn_ = false;

    // ── Ranged weapon state ──────────────────────────────────────────────
    float rangedShootTimer_ = 0.0f;
    uint32_t rangedAnimId_ = 0;
    bool restoreWeaponAfterRangedShot_ = false;
    std::function<void()> rangedShotCompleteCallback_;

    // ── Mount state (discovery + positioning need renderer) ──────────────
    uint32_t mountInstanceId_ = 0;
    float mountHeightOffset_ = 0.0f;
    float mountPitch_ = 0.0f;
    float mountRoll_ = 0.0f;       // External roll for taxi flights (lean roll computed by MountFSM)
    int mountSeatAttachmentId_ = -1;
    glm::vec3 smoothedMountSeatPos_ = glm::vec3(0.0f);
    bool mountSeatSmoothingInit_ = false;
    // Last frame's seat target, so the smoothing can tell a rider standing still
    // from one being carried. Pressing no movement keys is not the same as not
    // moving through the world, and on a boat the difference is the whole bug.
    glm::vec3 lastMountSeatTarget_ = glm::vec3(0.0f);
    bool taxiFlight_ = false;
    bool m2TransportRiding_ = false;

    // ── Private helpers ──────────────────────────────────────────────────
    uint32_t resolveMeleeAnimId();
    void updateMountedAnimation(float deltaTime);
    void applyMountPositioning(float mountBob, float mountRoll, float characterYaw);
};

} // namespace rendering
} // namespace wowee
