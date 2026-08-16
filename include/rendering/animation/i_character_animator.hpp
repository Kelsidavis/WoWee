#pragma once

#include "rendering/animation/i_animator.hpp"
#include "rendering/animation/weapon_type.hpp"
#include <cstdint>
#include <string>

namespace wowee {
namespace rendering {

// ============================================================================
// ICharacterAnimator
//
// Player-specific animation interface. Extends IAnimator with combat,
// spell, emote, and mount operations.
// ============================================================================
class ICharacterAnimator : public IAnimator {
public:
    virtual void startSpellCast(uint32_t precast, uint32_t cast, bool loop, uint32_t finalize) = 0;
    virtual void stopSpellCast() = 0;
    virtual void triggerMeleeSwing() = 0;
    virtual void triggerRangedShot() = 0;
    virtual void triggerHitReaction(uint32_t animId) = 0;
    virtual void triggerSpecialAttack(uint32_t spellId) = 0;
    virtual void setEquippedWeaponType(const WeaponLoadout& loadout) = 0;
    virtual void setEquippedRangedType(RangedWeaponType type) = 0;
    virtual void setRangedWeaponActive(bool active) = 0;
    virtual void playEmote(uint32_t animId, bool loop) = 0;
    virtual void cancelEmote() = 0;
    virtual void startLooting() = 0;
    virtual void stopLooting() = 0;
    virtual void setStunned(bool stunned) = 0;
    virtual void setCharging(bool charging) = 0;
    virtual void setStandState(uint8_t state) = 0;
    virtual void setStealthed(bool stealth) = 0;
    virtual void setInCombat(bool combat) = 0;
    virtual void setLowHealth(bool low) = 0;
    virtual void setSprintAuraActive(bool active) = 0;
    ~ICharacterAnimator() override = default;
};

} // namespace rendering
} // namespace wowee
