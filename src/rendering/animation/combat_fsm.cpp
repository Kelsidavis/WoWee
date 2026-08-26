#include "rendering/animation/combat_fsm.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "game/inventory.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {

// ── One-shot completion helper ───────────────────────────────────────────────

bool CombatFSM::oneShotComplete(const Input& in, uint32_t expectedAnimId) const {
    if (!in.haveAnimState) return false;
    // Renderer auto-returns one-shots to STAND; detect that OR normal completion
    return in.currentAnimId != expectedAnimId ||
           (in.currentAnimDuration > 0.1f && in.currentAnimTime >= in.currentAnimDuration - 0.05f);
}

// ── Event handling ───────────────────────────────────────────────────────────

void CombatFSM::onEvent(AnimEvent event) {
    switch (event) {
        case AnimEvent::COMBAT_ENTER:
            if (state_ == State::INACTIVE)
                state_ = State::UNSHEATHE;
            break;
        case AnimEvent::COMBAT_EXIT:
            if (state_ == State::COMBAT_IDLE)
                state_ = State::SHEATHE;
            break;
        case AnimEvent::STUN_ENTER:
            clearSpellState();
            hitReactionAnimId_ = 0;
            stunned_ = true;
            state_ = State::STUNNED;
            break;
        case AnimEvent::STUN_EXIT:
            stunned_ = false;
            if (state_ == State::STUNNED)
                state_ = State::INACTIVE;
            break;
        case AnimEvent::HIT_REACT:
            // Handled by triggerHitReaction() with animId
            break;
        case AnimEvent::CHARGE_START:
            charging_ = true;
            clearSpellState();
            state_ = State::CHARGE;
            break;
        case AnimEvent::CHARGE_END:
            charging_ = false;
            if (state_ == State::CHARGE)
                state_ = State::INACTIVE;
            break;
        case AnimEvent::SWIM_ENTER:
            clearSpellState();
            hitReactionAnimId_ = 0;
            stunned_ = false;
            state_ = State::INACTIVE;
            break;
        default:
            break;
    }
}

// ── Spell cast management ────────────────────────────────────────────────────

void CombatFSM::startSpellCast(uint32_t precast, uint32_t cast, bool castLoop, uint32_t finalize) {
    spellPrecastAnimId_ = precast;
    spellCastAnimId_ = cast;
    spellCastLoop_ = castLoop;
    spellFinalizeAnimId_ = finalize;
    spellPrecastAnimSeen_ = false;
    spellPrecastFrames_ = 0;
    spellFinalizeAnimSeen_ = false;
    spellFinalizeFrames_ = 0;
    state_ = (precast != 0) ? State::SPELL_PRECAST : State::SPELL_CASTING;
}

void CombatFSM::stopSpellCast() {
    if (state_ != State::SPELL_PRECAST && state_ != State::SPELL_CASTING) return;
    spellFinalizeAnimSeen_ = false;
    spellFinalizeFrames_ = 0;
    state_ = State::SPELL_FINALIZE;
}

void CombatFSM::clearSpellState() {
    spellPrecastAnimId_ = 0;
    spellCastAnimId_ = 0;
    spellCastLoop_ = false;
    spellFinalizeAnimId_ = 0;
    spellPrecastAnimSeen_ = false;
    spellPrecastFrames_ = 0;
    spellFinalizeAnimSeen_ = false;
    spellFinalizeFrames_ = 0;
}

// ── Hit/stun ─────────────────────────────────────────────────────────────────

void CombatFSM::triggerHitReaction(uint32_t animId) {
    // Don't interrupt swim/jump/stun states
    if (state_ == State::STUNNED) return;
    // Interrupt spell casting
    if (state_ == State::SPELL_PRECAST || state_ == State::SPELL_CASTING || state_ == State::SPELL_FINALIZE) {
        clearSpellState();
    }
    hitReactionAnimId_ = animId;
    state_ = State::HIT_REACTION;
}

void CombatFSM::setStunned(bool stunned) {
    stunned_ = stunned;
    if (stunned) {
        if (state_ == State::SPELL_PRECAST || state_ == State::SPELL_CASTING || state_ == State::SPELL_FINALIZE) {
            clearSpellState();
        }
        hitReactionAnimId_ = 0;
        state_ = State::STUNNED;
    } else {
        if (state_ == State::STUNNED)
            state_ = State::INACTIVE;
    }
}

void CombatFSM::setCharging(bool charging) {
    charging_ = charging;
    if (charging) {
        clearSpellState();
        hitReactionAnimId_ = 0;
        state_ = State::CHARGE;
    } else if (state_ == State::CHARGE) {
        state_ = State::INACTIVE;
    }
}

// ── State transitions ────────────────────────────────────────────────────────

void CombatFSM::updateTransitions(const Input& in) {
    // Stun override: can't act while stunned
    if (stunned_ && state_ != State::STUNNED) {
        state_ = State::STUNNED;
        return;
    }

    // Force melee/ranged overrides
    if (in.meleeSwingTimer > 0.0f && !stunned_ && in.grounded && !in.swimming) {
        if (state_ != State::MELEE_SWING) {
            clearSpellState();
            hitReactionAnimId_ = 0;
        }
        state_ = State::MELEE_SWING;
        return;
    }
    if (in.rangedShootTimer > 0.0f && !stunned_ && in.meleeSwingTimer <= 0.0f && in.grounded && !in.swimming) {
        if (state_ != State::RANGED_SHOOT) {
            clearSpellState();
            hitReactionAnimId_ = 0;
        }
        state_ = State::RANGED_SHOOT;
        return;
    }
    if (charging_ && !stunned_) {
        if (state_ != State::CHARGE) {
            clearSpellState();
            hitReactionAnimId_ = 0;
        }
        state_ = State::CHARGE;
        return;
    }

    switch (state_) {
        case State::INACTIVE:
            if (in.inCombat && in.grounded && !in.swimming && !in.moving) {
                state_ = in.hasUnsheathe ? State::UNSHEATHE : State::COMBAT_IDLE;
            }
            break;

        case State::COMBAT_IDLE:
            if (in.swimming || in.jumping || !in.grounded || in.moving) {
                state_ = State::INACTIVE;
            } else if (!in.inCombat) {
                state_ = in.hasSheathe ? State::SHEATHE : State::INACTIVE;
            }
            break;

        case State::MELEE_SWING:
            if (in.meleeSwingTimer <= 0.0f) {
                if (in.swimming) {
                    state_ = State::INACTIVE;
                } else if (in.inCombat && in.grounded) {
                    state_ = State::COMBAT_IDLE;
                } else {
                    state_ = State::INACTIVE;
                }
            }
            break;

        case State::RANGED_SHOOT:
            if (in.rangedShootTimer <= 0.0f) {
                if (in.swimming) {
                    state_ = State::INACTIVE;
                } else if (in.inCombat && in.grounded) {
                    state_ = State::RANGED_LOAD;
                } else {
                    state_ = State::INACTIVE;
                }
            }
            break;

        case State::RANGED_LOAD:
            if (in.swimming || in.jumping || !in.grounded || in.moving) {
                state_ = State::INACTIVE;
            } else if (in.inCombat) {
                state_ = State::COMBAT_IDLE;
            } else {
                state_ = State::INACTIVE;
            }
            break;

        case State::SPELL_PRECAST:
            if (in.swimming || (in.jumping && !in.grounded) || (!in.grounded && !in.jumping)) {
                clearSpellState();
                state_ = State::INACTIVE;
            } else if (in.haveAnimState) {
                uint32_t expectedAnim = spellPrecastAnimId_ ? spellPrecastAnimId_ : anim::SPELL_PRECAST;
                if (in.currentAnimId == expectedAnim) spellPrecastAnimSeen_ = true;
                if (spellPrecastAnimSeen_ && oneShotComplete(in, expectedAnim)) {
                    state_ = State::SPELL_CASTING;
                }
                if (!spellPrecastAnimSeen_ && ++spellPrecastFrames_ > 10) {
                    state_ = State::SPELL_CASTING;
                }
            }
            break;

        case State::SPELL_CASTING:
            if (in.swimming || (in.jumping && !in.grounded) || (!in.grounded && !in.jumping)) {
                clearSpellState();
                state_ = State::INACTIVE;
            } else if (in.moving) {
                clearSpellState();
                state_ = State::INACTIVE;
            }
            // Stays in SPELL_CASTING until stopSpellCast() is called externally
            break;

        case State::SPELL_FINALIZE: {
            if (in.swimming || (in.jumping && !in.grounded)) {
                clearSpellState();
                state_ = State::INACTIVE;
            } else if (in.haveAnimState) {
                uint32_t expectedAnim = spellFinalizeAnimId_ ? spellFinalizeAnimId_
                    : (spellCastAnimId_ ? spellCastAnimId_ : anim::SPELL);
                if (in.currentAnimId == expectedAnim) spellFinalizeAnimSeen_ = true;
                if (spellFinalizeAnimSeen_ && oneShotComplete(in, expectedAnim)) {
                    clearSpellState();
                    state_ = in.inCombat ? State::COMBAT_IDLE : State::INACTIVE;
                }
                // Safety: if finalize anim never seen (model lacks it), finish after timeout
                if (!spellFinalizeAnimSeen_ && ++spellFinalizeFrames_ > 10) {
                    clearSpellState();
                    state_ = in.inCombat ? State::COMBAT_IDLE : State::INACTIVE;
                }
            }
            break;
        }

        case State::HIT_REACTION:
            if (in.swimming || in.moving) {
                hitReactionAnimId_ = 0;
                state_ = State::INACTIVE;
            } else if (in.haveAnimState) {
                uint32_t expectedAnim = hitReactionAnimId_ ? hitReactionAnimId_ : anim::COMBAT_WOUND;
                if (oneShotComplete(in, expectedAnim)) {
                    hitReactionAnimId_ = 0;
                    state_ = in.inCombat ? State::COMBAT_IDLE : State::INACTIVE;
                }
            }
            break;

        case State::STUNNED:
            if (!stunned_) {
                state_ = in.inCombat ? State::COMBAT_IDLE : State::INACTIVE;
            } else if (in.swimming) {
                stunned_ = false;
                state_ = State::INACTIVE;
            }
            break;

        case State::CHARGE:
            if (!charging_) {
                state_ = State::INACTIVE;
            }
            break;

        case State::UNSHEATHE:
            if (in.swimming || in.moving) {
                state_ = State::INACTIVE;
            } else if (in.haveAnimState && oneShotComplete(in, anim::UNSHEATHE)) {
                state_ = State::COMBAT_IDLE;
            }
            break;

        case State::SHEATHE:
            if (in.swimming || in.moving) {
                state_ = State::INACTIVE;
            } else if (in.inCombat) {
                state_ = State::COMBAT_IDLE;
            } else if (in.haveAnimState && oneShotComplete(in, anim::SHEATHE)) {
                state_ = State::INACTIVE;
            }
            break;
    }
}

// ── Animation resolution ─────────────────────────────────────────────────────

AnimOutput CombatFSM::resolve(const Input& in, const AnimCapabilitySet& caps,
                               const WeaponLoadout& loadout) {
    updateTransitions(in);

    if (state_ == State::INACTIVE) return AnimOutput::stay();

    uint32_t animId = 0;
    bool loop = true;

    switch (state_) {
        case State::INACTIVE:
            return AnimOutput::stay();

        case State::COMBAT_IDLE:
            if (in.lowHealth && caps.resolvedStandWound) {
                animId = caps.resolvedStandWound;
            } else if (in.rangedWeaponActive && loadout.rangedType == RangedWeaponType::BOW) {
                animId = caps.resolvedReadyBow;
            } else if (in.rangedWeaponActive && loadout.rangedType == RangedWeaponType::GUN) {
                animId = caps.resolvedReadyRifle;
            } else if (in.rangedWeaponActive && loadout.rangedType == RangedWeaponType::CROSSBOW) {
                animId = caps.resolvedReadyCrossbow;
            } else if (in.rangedWeaponActive && loadout.rangedType == RangedWeaponType::THROWN) {
                animId = caps.resolvedReadyThrown;
            } else if (in.rangedWeaponActive && loadout.rangedType == RangedWeaponType::WAND) {
                animId = caps.resolvedReadyWand;
            } else if (loadout.is2HLoose) {
                animId = caps.resolvedReady2HLoose;
            } else if (loadout.inventoryType == game::InvType::TWO_HAND) {
                animId = caps.resolvedReady2H;
            } else if (loadout.isFist) {
                animId = caps.resolvedReadyFist;
            } else if (loadout.inventoryType == game::InvType::NON_EQUIP) {
                animId = caps.resolvedReadyUnarmed;
            } else {
                animId = caps.resolvedReady1H;
            }
            loop = true;
            break;

        case State::MELEE_SWING:
            if (in.specialAttackAnimId != 0) {
                animId = in.specialAttackAnimId;
            } else {
                // Resolve melee animation using probed capabilities + weapon loadout
                bool useOffHand = loadout.hasOffHand && offHandTurn_;
                offHandTurn_ = loadout.hasOffHand ? !offHandTurn_ : false;

                if (useOffHand) {
                    // The off-hand weapon's kind, not the main hand's.
                    if (loadout.offHandIsFist) animId = caps.resolvedMeleeOffHandFist;
                    else if (loadout.offHandIsDagger) animId = caps.resolvedMeleeOffHandPierce;
                    else if (loadout.inventoryType == game::InvType::NON_EQUIP) animId = caps.resolvedMeleeOffHandUnarmed;
                    else animId = caps.resolvedMeleeOffHand;
                } else if (loadout.isFist) {
                    animId = caps.resolvedMeleeFist;
                } else if (loadout.isDagger) {
                    animId = caps.resolvedMeleePierce;
                } else if (loadout.is2HLoose) {
                    animId = caps.resolvedMelee2HLoose;
                } else if (loadout.inventoryType == game::InvType::TWO_HAND) {
                    animId = caps.resolvedMelee2H;
                } else if (loadout.inventoryType == game::InvType::NON_EQUIP) {
                    animId = caps.resolvedMeleeUnarmed;
                } else {
                    animId = caps.resolvedMelee1H;
                }
            }
            if (animId == 0) {
                LOG_DEBUG("CombatFSM: MELEE_SWING resolved animId=0, falling back to STAND");
                animId = anim::STAND;
            }
            loop = false;
            break;

        case State::RANGED_SHOOT:
            animId = in.rangedAnimId ? in.rangedAnimId : anim::ATTACK_BOW;
            loop = false;
            break;

        case State::RANGED_LOAD:
            switch (loadout.rangedType) {
                case RangedWeaponType::BOW:      animId = caps.resolvedLoadBow; break;
                case RangedWeaponType::GUN:      animId = caps.resolvedLoadRifle; break;
                case RangedWeaponType::CROSSBOW: animId = caps.resolvedLoadBow; break;
                default: break;
            }
            loop = false;
            break;

        case State::SPELL_PRECAST:
            animId = spellPrecastAnimId_ ? spellPrecastAnimId_ : anim::SPELL_PRECAST;
            loop = false;
            break;

        case State::SPELL_CASTING:
            animId = spellCastAnimId_ ? spellCastAnimId_ : anim::SPELL;
            loop = spellCastLoop_;
            break;

        case State::SPELL_FINALIZE:
            animId = spellFinalizeAnimId_ ? spellFinalizeAnimId_
                   : (spellCastAnimId_ ? spellCastAnimId_ : anim::SPELL);
            loop = false;
            break;

        case State::HIT_REACTION:
            animId = hitReactionAnimId_ ? hitReactionAnimId_
                   : (caps.resolvedCombatWound ? caps.resolvedCombatWound : anim::COMBAT_WOUND);
            loop = false;
            break;

        case State::STUNNED:
            animId = caps.resolvedStun ? caps.resolvedStun : anim::STUN;
            loop = true;
            break;

        case State::CHARGE:
            animId = caps.resolvedRun ? caps.resolvedRun : anim::RUN;
            loop = true;
            break;

        case State::UNSHEATHE:
            animId = caps.resolvedUnsheathe ? caps.resolvedUnsheathe : anim::UNSHEATHE;
            loop = false;
            break;

        case State::SHEATHE:
            animId = caps.resolvedSheathe ? caps.resolvedSheathe : anim::SHEATHE;
            loop = false;
            break;
    }

    if (animId == 0) return AnimOutput::stay();
    return AnimOutput::ok(animId, loop);
}

// ── Reset ────────────────────────────────────────────────────────────────────

void CombatFSM::reset() {
    state_ = State::INACTIVE;
    clearSpellState();
    hitReactionAnimId_ = 0;
    stunned_ = false;
    charging_ = false;
    offHandTurn_ = false;
}

} // namespace rendering
} // namespace wowee
