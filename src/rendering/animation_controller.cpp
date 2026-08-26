#include <span>

#include "rendering/animation/melee_anim_chains.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/animation/emote_registry.hpp"
#include "rendering/animation/anim_capability_probe.hpp"
#include "rendering/animation/mount_fsm.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/levelup_effect.hpp"
#include "rendering/charge_effect.hpp"
#include "rendering/spell_visual_system.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "game/inventory.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/music_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "rendering/swim_effects.hpp"
#include <algorithm>
#include <cstdlib>
#include <unordered_map>
#include <set>
#include <random>
#include <cctype>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

namespace wowee {
namespace rendering {

// ── AnimationController implementation ───────────────────────────────────────

AnimationController::AnimationController() = default;
AnimationController::~AnimationController() = default;

void AnimationController::initialize(Renderer* renderer) {
    renderer_ = renderer;
}

void AnimationController::probeCapabilities() {
    if (!renderer_) return;
    uint32_t instanceId = renderer_->getCharacterInstanceId();
    if (instanceId == 0) return;
    auto caps = AnimCapabilityProbe::probe(renderer_, instanceId);
    characterAnimator_.setCapabilities(caps);
    capabilitiesProbed_ = true;
}

void AnimationController::onCharacterFollow(uint32_t /*instanceId*/) {
    // Reset animation state when follow target changes
    capabilitiesProbed_ = false;
}

// ── Emote support ────────────────────────────────────────────────────────────

void AnimationController::playEmote(const std::string& emoteName) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    auto result = registry.findEmote(emoteName);
    if (!result) return;

    uint32_t animId = result->animId;
    bool loop = result->loop;

    // For looping emotes, prefer the EMOTE_STATE_* variant if the model has it
    if (loop) {
        uint32_t stateVariant = registry.getStateVariant(animId);
        if (stateVariant != 0) {
            auto* characterRenderer = renderer_->getCharacterRenderer();
            uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
            if (characterRenderer && characterInstanceId > 0 &&
                characterRenderer->hasAnimation(characterInstanceId, stateVariant)) {
                animId = stateVariant;
            }
        }
    }

    // What an emote resolved to, and whether the model can play it. An emote
    // that does nothing is either a name that resolved to no animation or an
    // animation the model does not carry, and from outside those look the same.
    {
        auto* cr = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
        uint32_t ci = renderer_ ? renderer_->getCharacterInstanceId() : 0;
        const bool has = (cr && ci > 0) ? cr->hasAnimation(ci, animId) : false;
        LOG_WARNING("Emote '", emoteName, "' -> animation ", animId,
                    (loop ? " (looping)" : " (one shot)"),
                    " model has it: ", (has ? "yes" : "NO"));
    }

    // Forward to CharacterAnimator (ActivityFSM handles emote state)
    characterAnimator_.playEmote(animId, loop);

    // Immediately play the emote animation on the renderer
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (characterRenderer && characterInstanceId > 0) {
        characterRenderer->playAnimation(characterInstanceId, animId, loop);
        lastPlayerAnimRequest_ = animId;
        lastPlayerAnimLoopRequest_ = loop;
    }
}

void AnimationController::playWeaponSheathAnimation(bool sheathing) {
    if (!renderer_) return;
    auto* characterRenderer = renderer_->getCharacterRenderer();
    const uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) return;

    uint32_t animId = sheathing ? anim::SHEATHE : anim::UNSHEATHE;
    if (!characterRenderer->hasAnimation(characterInstanceId, animId)) {
        if (sheathing && characterRenderer->hasAnimation(characterInstanceId, anim::HIP_SHEATHE)) {
            animId = anim::HIP_SHEATHE;
        } else {
            return;
        }
    }

    // ActivityFSM owns the one-shot until completion so locomotion/combat does
    // not replace the reach animation on the next frame.
    characterAnimator_.playEmote(animId, false);
    characterRenderer->playAnimation(characterInstanceId, animId, false);
    lastPlayerAnimRequest_ = animId;
    lastPlayerAnimLoopRequest_ = false;
}

void AnimationController::cancelEmote() {
    characterAnimator_.cancelEmote();
}

std::string AnimationController::getEmoteText(const std::string& emoteName, const std::string* targetName) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    return registry.textFor(emoteName, targetName);
}

uint32_t AnimationController::getEmoteDbcId(const std::string& emoteName) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    return registry.dbcIdFor(emoteName);
}

std::string AnimationController::getEmoteTextByDbcId(uint32_t dbcId, const std::string& senderName,
                                                      const std::string* targetName) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    return registry.textByDbcId(dbcId, senderName, targetName);
}
uint32_t AnimationController::getEmoteAnimByEmotesId(uint32_t emoteId) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    return registry.animByEmotesId(emoteId);
}

bool AnimationController::isStateEmoteById(uint32_t emoteId) {
    auto& registry = EmoteRegistry::instance();
    registry.loadFromDbc();
    return registry.isStateEmote(emoteId);
}

// ── Spell casting ────────────────────────────────────────────────────────────

void AnimationController::startSpellCast(uint32_t precastAnimId, uint32_t castAnimId, bool castLoop,
                                         uint32_t finalizeAnimId) {
    characterAnimator_.startSpellCast(precastAnimId, castAnimId, castLoop, finalizeAnimId);
}

void AnimationController::stopSpellCast() {
    characterAnimator_.stopSpellCast();
}

void AnimationController::setSeatedLoopAnimation(uint32_t animationId) {
    characterAnimator_.getActivity().setSeatedLoopAnimation(animationId);
}

// ── Loot animation ───────────────────────────────────────────────────────────

void AnimationController::startLooting() {
    characterAnimator_.startLooting();
}

void AnimationController::stopLooting() {
    characterAnimator_.stopLooting();
}

// ── Hit reactions ────────────────────────────────────────────────────────────

void AnimationController::triggerHitReaction(uint32_t animId) {
    characterAnimator_.triggerHitReaction(animId);
}

// ── Crowd control ────────────────────────────────────────────────────────────

void AnimationController::setStunned(bool stunned) {
    stunned_ = stunned;
    characterAnimator_.setStunned(stunned);
}

// ── Health-based idle ────────────────────────────────────────────────────────

void AnimationController::setLowHealth(bool low) {
    characterAnimator_.setLowHealth(low);
}

// ── Stand state ──────────────────────────────────────────────────────────────

void AnimationController::setStandState(uint8_t state) {
    characterAnimator_.setStandState(state);
}

// ── Stealth ──────────────────────────────────────────────────────────────────

void AnimationController::setStealthed(bool stealth) {
    characterAnimator_.setStealthed(stealth);
}

// ── Sprint aura ──────────────────────────────────────────────────────────────

void AnimationController::setSprintAuraActive(bool active) {
    // Straight through. The animator keeps the copy the locomotion state
    // machine reads; this class kept one of its own and read it nowhere.
    characterAnimator_.setSprintAuraActive(active);
}

// ── Targeting / combat ───────────────────────────────────────────────────────

void AnimationController::setTargetPosition(const glm::vec3* pos) {
    targetPosition_ = pos;
}

void AnimationController::setInCombat(bool combat) {
    inCombat_ = combat;
    characterAnimator_.setInCombat(combat);
}

void AnimationController::resetCombatVisualState() {
    inCombat_ = false;
    targetPosition_ = nullptr;
    meleeSwingTimer_ = 0.0f;
    meleeSwingCooldown_ = 0.0f;
    specialAttackAnimId_ = 0;
    rangedShootTimer_ = 0.0f;
    rangedAnimId_ = 0;
    restoreWeaponAfterRangedShot_ = false;
    characterAnimator_.setRangedWeaponActive(false);
    stunned_ = false;
    charging_ = false;

    // Reset all CharacterAnimator combat state
    characterAnimator_.setInCombat(false);
    characterAnimator_.setStunned(false);
    characterAnimator_.setCharging(false);
    characterAnimator_.setLowHealth(false);
    characterAnimator_.stopSpellCast();
    characterAnimator_.triggerHitReaction(0);  // Clear hit reaction

    if (auto* svs = renderer_->getSpellVisualSystem()) svs->reset();
}

bool AnimationController::isMoving() const {
    auto* cameraController = renderer_->getCameraController();
    return cameraController && cameraController->isMoving();
}

// ── Melee combat ─────────────────────────────────────────────────────────────

void AnimationController::triggerMeleeSwing() {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) return;
    if (meleeSwingCooldown_ > 0.0f) return;
    if (characterAnimator_.getActivity().isEmoteActive()) {
        characterAnimator_.cancelEmote();
    }
    specialAttackAnimId_ = 0;  // Clear any special attack override
    resolveMeleeAnimId();
    meleeSwingCooldown_ = 0.1f;
    float durationSec = meleeAnimDurationMs_ > 0.0f ? meleeAnimDurationMs_ / 1000.0f : 0.6f;
    if (durationSec < 0.25f) durationSec = 0.25f;
    if (durationSec > 1.0f) durationSec = 1.0f;
    meleeSwingTimer_ = durationSec;

    if (renderer_->getAudioCoordinator()->getActivitySoundManager()) {
        renderer_->getAudioCoordinator()->getActivitySoundManager()->playMeleeSwing();
    }
}

void AnimationController::setEquippedWeaponType(uint32_t inventoryType, bool is2HLoose,
                                                 bool isFist, bool isDagger,
                                                 bool hasOffHand, bool hasShield,
                                                 bool offHandIsFist,
                                                 bool offHandIsDagger) {
    weaponLoadout_.inventoryType = inventoryType;
    weaponLoadout_.is2HLoose = is2HLoose;
    weaponLoadout_.isFist = isFist;
    weaponLoadout_.isDagger = isDagger;
    weaponLoadout_.hasOffHand = hasOffHand;
    weaponLoadout_.offHandIsFist = offHandIsFist;
    weaponLoadout_.offHandIsDagger = offHandIsDagger;
    weaponLoadout_.hasShield = hasShield;
    meleeAnimId_ = 0;  // Force re-resolve on next swing
    characterAnimator_.setEquippedWeaponType(weaponLoadout_);
}

void AnimationController::triggerSpecialAttack(uint32_t /*spellId*/) {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) return;
    if (meleeSwingCooldown_ > 0.0f) return;
    if (characterAnimator_.getActivity().isEmoteActive()) {
        characterAnimator_.cancelEmote();
    }

    auto has = [&](uint32_t id) { return characterRenderer->hasAnimation(characterInstanceId, id); };

    // Choose special attack animation based on equipped weapon type
    uint32_t specAnim = 0;
    if (weaponLoadout_.hasShield && has(anim::SHIELD_BASH)) {
        specAnim = anim::SHIELD_BASH;
    } else if ((weaponLoadout_.inventoryType == game::InvType::TWO_HAND || weaponLoadout_.is2HLoose) && has(anim::SPECIAL_2H)) {
        specAnim = anim::SPECIAL_2H;
    } else if (weaponLoadout_.inventoryType != game::InvType::NON_EQUIP && has(anim::SPECIAL_1H)) {
        specAnim = anim::SPECIAL_1H;
    } else if (has(anim::SPECIAL_UNARMED)) {
        specAnim = anim::SPECIAL_UNARMED;
    } else if (has(anim::SPECIAL_1H)) {
        specAnim = anim::SPECIAL_1H;
    }

    if (specAnim == 0) {
        // No special animation available - fall back to regular melee swing
        triggerMeleeSwing();
        return;
    }

    specialAttackAnimId_ = specAnim;
    meleeSwingCooldown_ = 0.1f;
    // Query the special attack animation duration
    std::vector<pipeline::M2Sequence> sequences;
    float dur = 0.6f;
    if (characterRenderer->getAnimationSequences(characterInstanceId, sequences)) {
        for (const auto& seq : sequences) {
            if (seq.id == specAnim && seq.duration > 0) {
                dur = static_cast<float>(seq.duration) / 1000.0f;
                break;
            }
        }
    }
    if (dur < 0.25f) dur = 0.25f;
    if (dur > 1.0f) dur = 1.0f;
    meleeSwingTimer_ = dur;
    if (renderer_->getAudioCoordinator()->getActivitySoundManager()) {
        renderer_->getAudioCoordinator()->getActivitySoundManager()->playMeleeSwing();
    }
}

// ── Ranged combat ────────────────────────────────────────────────────────────

void AnimationController::setEquippedRangedType(RangedWeaponType type) {
    weaponLoadout_.rangedType = type;
    rangedAnimId_ = 0;
    characterAnimator_.setEquippedRangedType(type);
}

void AnimationController::setRangedWeaponActive(bool active) {
    characterAnimator_.setRangedWeaponActive(active);
}

void AnimationController::setCharging(bool c) {
    charging_ = c;
    characterAnimator_.setCharging(c);
}

void AnimationController::triggerRangedShot() {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) return;
    if (rangedShootTimer_ > 0.0f) return;
    if (characterAnimator_.getActivity().isEmoteActive()) characterAnimator_.cancelEmote();

    auto has = [&](uint32_t id) { return characterRenderer->hasAnimation(characterInstanceId, id); };

    // Resolve ranged attack animation based on weapon type
    uint32_t shootAnim = 0;
    switch (weaponLoadout_.rangedType) {
        case RangedWeaponType::BOW:
            if (has(anim::FIRE_BOW))        shootAnim = anim::FIRE_BOW;
            else if (has(anim::ATTACK_BOW)) shootAnim = anim::ATTACK_BOW;
            break;
        case RangedWeaponType::GUN:
            if (has(anim::ATTACK_RIFLE))    shootAnim = anim::ATTACK_RIFLE;
            break;
        case RangedWeaponType::CROSSBOW:
            if (has(anim::ATTACK_CROSSBOW)) shootAnim = anim::ATTACK_CROSSBOW;
            else if (has(anim::ATTACK_BOW)) shootAnim = anim::ATTACK_BOW;
            break;
        case RangedWeaponType::THROWN:
            if (has(anim::ATTACK_THROWN))    shootAnim = anim::ATTACK_THROWN;
            break;
        case RangedWeaponType::WAND: {
            // WOWEE_WAND_ANIM=<id> overrides the shot animation, for comparing
            // candidates on screen: 32 cast, 49 rifle, 53 directed, 54 omni.
            static const uint32_t kForced = []() -> uint32_t {
                const char* v = std::getenv("WOWEE_WAND_ANIM");
                return v ? static_cast<uint32_t>(std::atoi(v)) : 0;
            }();
            if (kForced != 0 && has(kForced))        shootAnim = kForced;
            else if (has(anim::SPELL_CAST_DIRECTED)) shootAnim = anim::SPELL_CAST_DIRECTED;
            else if (has(anim::ATTACK_RIFLE))        shootAnim = anim::ATTACK_RIFLE;
            break;
        }
        default: break;
    }
    if (shootAnim == 0) return;  // Model has no ranged animation

    rangedAnimId_ = shootAnim;

    // Query animation duration
    std::vector<pipeline::M2Sequence> sequences;
    float dur = 0.6f;
    if (characterRenderer->getAnimationSequences(characterInstanceId, sequences)) {
        for (const auto& seq : sequences) {
            if (seq.id == shootAnim && seq.duration > 0) {
                dur = static_cast<float>(seq.duration) / 1000.0f;
                break;
            }
        }
    }
    if (dur < 0.25f) dur = 0.25f;
    if (dur > 1.5f) dur = 1.5f;
    rangedShootTimer_ = dur;
    restoreWeaponAfterRangedShot_ =
        (weaponLoadout_.rangedType == RangedWeaponType::THROWN);
}

uint32_t AnimationController::resolveMeleeAnimId() {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) {
        meleeAnimId_ = 0;
        meleeAnimDurationMs_ = 0.0f;
        return 0;
    }

    // When dual-wielding, bypass cache to alternate main/off-hand animations
    if (!weaponLoadout_.hasOffHand && meleeAnimId_ != 0 && characterRenderer->hasAnimation(characterInstanceId, meleeAnimId_)) {
        return meleeAnimId_;
    }

    std::vector<pipeline::M2Sequence> sequences;
    if (!characterRenderer->getAnimationSequences(characterInstanceId, sequences)) {
        meleeAnimId_ = 0;
        meleeAnimDurationMs_ = 0.0f;
        return 0;
    }

    auto findDuration = [&](uint32_t id) -> float {
        for (const auto& seq : sequences) {
            if (seq.id == id && seq.duration > 0) {
                return static_cast<float>(seq.duration);
            }
        }
        return 0.0f;
    };

    std::span<const uint32_t> chain;
    // The chains live in melee_anim_chains.hpp, which the capability probe
    // reads too - it used to keep its own copies under a comment saying they
    // matched these.
    using anim::MeleeChain;

    // Dual-wield: alternate main-hand and off-hand swings
    bool useOffHand = weaponLoadout_.hasOffHand && meleeOffHandTurn_;
    meleeOffHandTurn_ = weaponLoadout_.hasOffHand ? !meleeOffHandTurn_ : false;

    if (useOffHand) {
        // The off-hand weapon's kind, not the main hand's.
        if (weaponLoadout_.offHandIsFist) {
            chain = anim::meleeAnimChain(MeleeChain::OffHandFist);
        } else if (weaponLoadout_.offHandIsDagger) {
            chain = anim::meleeAnimChain(MeleeChain::OffHandPierce);
        } else if (weaponLoadout_.inventoryType == game::InvType::NON_EQUIP) {
            chain = anim::meleeAnimChain(MeleeChain::OffHandUnarmed);
        } else {
            chain = anim::meleeAnimChain(MeleeChain::OffHand);
        }
    } else if (weaponLoadout_.isFist) {
        chain = anim::meleeAnimChain(MeleeChain::Fist);
    } else if (weaponLoadout_.isDagger) {
        chain = anim::meleeAnimChain(MeleeChain::Dagger);
    } else if (weaponLoadout_.is2HLoose) {
        // Polearm thrust uses pierce variant
        chain = anim::meleeAnimChain(MeleeChain::TwoHandLoose);
    } else if (weaponLoadout_.inventoryType == game::InvType::TWO_HAND) {
        chain = anim::meleeAnimChain(MeleeChain::TwoHand);
    } else if (weaponLoadout_.inventoryType == game::InvType::NON_EQUIP) {
        chain = anim::meleeAnimChain(MeleeChain::Unarmed);
    } else {
        chain = anim::meleeAnimChain(MeleeChain::OneHand);
    }
    for (uint32_t id : chain) {
        if (characterRenderer->hasAnimation(characterInstanceId, id)) {
            meleeAnimId_ = id;
            meleeAnimDurationMs_ = findDuration(id);
            return meleeAnimId_;
        }
    }

    const uint32_t avoidIds[] = {anim::STAND, anim::DEATH, anim::WALK, anim::RUN, anim::SHUFFLE_LEFT, anim::SHUFFLE_RIGHT, anim::WALK_BACKWARDS, anim::JUMP_START, anim::JUMP, anim::JUMP_END, anim::SWIM_IDLE, anim::SWIM, anim::SITTING};
    auto isAvoid = [&](uint32_t id) -> bool {
        for (uint32_t avoid : avoidIds) {
            if (id == avoid) return true;
        }
        return false;
    };

    uint32_t bestId = 0;
    uint32_t bestDuration = 0;
    for (const auto& seq : sequences) {
        if (seq.duration == 0) continue;
        if (isAvoid(seq.id)) continue;
        if (seq.movingSpeed > 0.1f) continue;
        if (seq.duration < 150 || seq.duration > 2000) continue;
        if (bestId == 0 || seq.duration < bestDuration) {
            bestId = seq.id;
            bestDuration = seq.duration;
        }
    }

    if (bestId == 0) {
        for (const auto& seq : sequences) {
            if (seq.duration == 0) continue;
            if (isAvoid(seq.id)) continue;
            if (bestId == 0 || seq.duration < bestDuration) {
                bestId = seq.id;
                bestDuration = seq.duration;
            }
        }
    }

    meleeAnimId_ = bestId;
    meleeAnimDurationMs_ = static_cast<float>(bestDuration);
    return meleeAnimId_;
}

// ── Effect triggers ──────────────────────────────────────────────────────────

void AnimationController::triggerLevelUpEffect(const glm::vec3& position) {
    auto* levelUpEffect = renderer_->getLevelUpEffect();
    if (!levelUpEffect) return;

    if (!levelUpEffect->isModelLoaded()) {
        auto* m2Renderer = renderer_->getM2Renderer();
        if (m2Renderer) {
            auto* assetManager = core::Application::getInstance().getAssetManager();
            if (!assetManager) {
                LOG_WARNING("LevelUpEffect: no asset manager available");
            } else {
                auto m2Data = assetManager->readFile("Spells\\LevelUp\\LevelUp.m2");
                auto skinData = assetManager->readFile("Spells\\LevelUp\\LevelUp00.skin");
                LOG_INFO("LevelUpEffect: m2Data=", m2Data.size(), " skinData=", skinData.size());
                if (!m2Data.empty()) {
                    levelUpEffect->loadModel(m2Renderer, m2Data, skinData);
                } else {
                    LOG_WARNING("LevelUpEffect: failed to read Spell\\LevelUp\\LevelUp.m2");
                }
            }
        }
    }

    levelUpEffect->trigger(position);
}

void AnimationController::startChargeEffect(const glm::vec3& position, const glm::vec3& direction) {
    auto* chargeEffect = renderer_->getChargeEffect();
    if (!chargeEffect) return;

    if (!chargeEffect->isActive()) {
        auto* m2Renderer = renderer_->getM2Renderer();
        if (m2Renderer) {
            auto* assetManager = core::Application::getInstance().getAssetManager();
            if (assetManager) {
                chargeEffect->tryLoadM2Models(m2Renderer, assetManager);
            }
        }
    }

    chargeEffect->start(position, direction);
}

void AnimationController::emitChargeEffect(const glm::vec3& position, const glm::vec3& direction) {
    if (auto* chargeEffect = renderer_->getChargeEffect()) {
        chargeEffect->emit(position, direction);
    }
}

void AnimationController::stopChargeEffect() {
    if (auto* chargeEffect = renderer_->getChargeEffect()) {
        chargeEffect->stop();
    }
}

// ── Mount ────────────────────────────────────────────────────────────────────

void AnimationController::setMounted(uint32_t mountInstId, uint32_t mountDisplayId, float heightOffset, const std::string& modelPath) {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    auto* cameraController = renderer_->getCameraController();

    mountInstanceId_ = mountInstId;
    mountHeightOffset_ = heightOffset;
    mountSeatAttachmentId_ = -1;
    smoothedMountSeatPos_ = renderer_->getCharacterPosition();
    mountSeatSmoothingInit_ = false;
    mountPitch_ = 0.0f;

    if (cameraController) {
        cameraController->setMounted(true);
        cameraController->setMountHeightOffset(heightOffset);
    }

    if (characterRenderer && mountInstId > 0) {
        characterRenderer->dumpAnimations(mountInstId);
    }

    // Discover mount animation capabilities (property-based, not hardcoded IDs)
    LOG_DEBUG("=== Mount Animation Dump (Display ID ", mountDisplayId, ") ===");
    if (characterRenderer) characterRenderer->dumpAnimations(mountInstId);

    std::vector<pipeline::M2Sequence> sequences;
    if (!characterRenderer || !characterRenderer->getAnimationSequences(mountInstId, sequences)) {
        LOG_WARNING("Failed to get animation sequences for mount, using fallback IDs");
        sequences.clear();
    }

    auto findFirst = [&](std::initializer_list<uint32_t> candidates) -> uint32_t {
        for (uint32_t id : candidates) {
            if (characterRenderer && characterRenderer->hasAnimation(mountInstId, id)) {
                return id;
            }
        }
        return 0;
    };

    // Property-based jump animation discovery with chain-based scoring
    auto discoverJumpSet = [&]() {
        LOG_DEBUG("=== Full sequence table for mount ===");
        for (const auto& seq : sequences) {
            LOG_DEBUG("SEQ id=", seq.id,
                     " dur=", seq.duration,
                     " flags=0x", std::hex, seq.flags, std::dec,
                     " moveSpd=", seq.movingSpeed,
                     " blend=", seq.blendTime,
                     " next=", seq.nextAnimation,
                     " alias=", seq.aliasNext);
        }
        LOG_DEBUG("=== End sequence table ===");

        std::set<uint32_t> forbiddenIds = {53, 54, 16};

        auto scoreNear = [](int a, int b) -> int {
            int d = std::abs(a - b);
            return (d <= 8) ? (20 - d) : 0;
        };

        auto isForbidden = [&](uint32_t id) {
            return forbiddenIds.count(id) != 0;
        };

        auto findSeqById = [&](uint32_t id) -> const pipeline::M2Sequence* {
            for (const auto& s : sequences) {
                if (s.id == id) return &s;
            }
            return nullptr;
        };

        uint32_t runId = findFirst({anim::RUN, anim::WALK});
        uint32_t standId = findFirst({anim::STAND});

        std::vector<uint32_t> loops;
        for (const auto& seq : sequences) {
            if (isForbidden(seq.id)) continue;
            bool isLoop = (seq.flags & 0x01) == 0;
            if (isLoop && seq.duration >= 350 && seq.duration <= 1000 &&
                seq.id != runId && seq.id != standId) {
                loops.push_back(seq.id);
            }
        }

        uint32_t loop = 0;
        if (!loops.empty()) {
            uint32_t best = loops[0];
            int bestScore = -999;
            for (uint32_t id : loops) {
                int sc = 0;
                sc += scoreNear(static_cast<int>(id), 38);
                const auto* s = findSeqById(id);
                if (s) sc += (s->duration >= 500 && s->duration <= 800) ? 5 : 0;
                if (sc > bestScore) {
                    bestScore = sc;
                    best = id;
                }
            }
            loop = best;
        }

        uint32_t start = 0, end = 0;
        int bestStart = -999, bestEnd = -999;

        for (const auto& seq : sequences) {
            if (isForbidden(seq.id)) continue;
            bool isLoop = (seq.flags & 0x01) == 0;
            if (isLoop) continue;

            if (seq.duration >= 450 && seq.duration <= 1100) {
                int sc = 0;
                if (loop) sc += scoreNear(static_cast<int>(seq.id), static_cast<int>(loop));
                if (loop && (seq.nextAnimation == static_cast<int16_t>(loop) || seq.aliasNext == loop)) sc += 30;
                if (loop && scoreNear(seq.nextAnimation, static_cast<int>(loop)) > 0) sc += 10;
                if (seq.blendTime > 400) sc -= 5;

                if (sc > bestStart) {
                    bestStart = sc;
                    start = seq.id;
                }
            }

            if (seq.duration >= 650 && seq.duration <= 1600) {
                int sc = 0;
                if (loop) sc += scoreNear(static_cast<int>(seq.id), static_cast<int>(loop));
                if (seq.nextAnimation == static_cast<int16_t>(runId) || seq.nextAnimation == static_cast<int16_t>(standId)) sc += 10;
                if (seq.nextAnimation < 0) sc += 5;
                if (sc > bestEnd) {
                    bestEnd = sc;
                    end = seq.id;
                }
            }
        }

        LOG_DEBUG("Property-based jump discovery: start=", start, " loop=", loop, " end=", end,
                 " scores: start=", bestStart, " end=", bestEnd);
        return std::make_tuple(start, loop, end);
    };

    auto [discoveredStart, discoveredLoop, discoveredEnd] = discoverJumpSet();

    // Build MountAnimSet for MountFSM
    MountFSM::MountAnimSet mountAnims;
    mountAnims.jumpStart = discoveredStart > 0 ? discoveredStart : findFirst({anim::FALL, anim::JUMP_START});
    mountAnims.jumpLoop  = discoveredLoop > 0 ? discoveredLoop : findFirst({anim::JUMP});
    mountAnims.jumpEnd   = discoveredEnd > 0 ? discoveredEnd : findFirst({anim::JUMP_END});
    mountAnims.rearUp    = findFirst({anim::MOUNT_SPECIAL, anim::RUN_RIGHT, anim::FALL});
    mountAnims.run       = findFirst({anim::RUN, anim::WALK});
    mountAnims.runLeft   = findFirst({anim::MOUNT_RUN_LEFT, anim::RUN_LEFT});
    mountAnims.runRight  = findFirst({anim::MOUNT_RUN_RIGHT, anim::RUN_RIGHT});
    mountAnims.stand     = findFirst({anim::STAND});
    // Discover flight animations (flying mounts only - may all be 0 for ground mounts)
    mountAnims.flyIdle      = findFirst({anim::FLY_IDLE});
    mountAnims.flyForward   = findFirst({anim::FLY_FORWARD, anim::FLY_RUN_2});
    mountAnims.flyBackwards = findFirst({anim::FLY_BACKWARDS, anim::FLY_WALK_BACKWARDS});
    mountAnims.flyLeft      = findFirst({anim::FLY_LEFT, anim::FLY_SHUFFLE_LEFT});
    mountAnims.flyRight     = findFirst({anim::FLY_RIGHT, anim::FLY_SHUFFLE_RIGHT});
    mountAnims.flyUp        = findFirst({anim::FLY_UP, anim::FLY_RISE});
    mountAnims.flyDown      = findFirst({anim::FLY_DOWN});

    // Discover idle fidget animations using proper WoW M2 metadata
    core::Logger::getInstance().debug("Scanning for fidget animations in ", sequences.size(), " sequences");

    core::Logger::getInstance().debug("=== ALL potential fidgets (no metadata filter) ===");
    for (const auto& seq : sequences) {
        bool isLoop = (seq.flags & 0x01) == 0;
        bool isStationary = std::abs(seq.movingSpeed) < 0.05f;
        bool reasonableDuration = seq.duration >= 400 && seq.duration <= 2500;

        if (!isLoop && reasonableDuration && isStationary) {
            core::Logger::getInstance().debug("  ALL: id=", seq.id,
                " dur=", seq.duration, "ms",
                " freq=", seq.frequency,
                " replay=", seq.replayMin, "-", seq.replayMax,
                " flags=0x", std::hex, seq.flags, std::dec,
                " next=", seq.nextAnimation);
        }
    }

    for (const auto& seq : sequences) {
        bool isLoop = (seq.flags & 0x01) == 0;
        bool hasFrequency = seq.frequency > 0;
        bool hasReplay = seq.replayMax > 0;
        bool isStationary = std::abs(seq.movingSpeed) < 0.05f;
        bool reasonableDuration = seq.duration >= 400 && seq.duration <= 2500;

        if (!isLoop && reasonableDuration && isStationary && (hasFrequency || hasReplay)) {
            core::Logger::getInstance().debug("  Candidate: id=", seq.id,
                " dur=", seq.duration, "ms",
                " freq=", seq.frequency,
                " replay=", seq.replayMin, "-", seq.replayMax,
                " next=", seq.nextAnimation,
                " speed=", seq.movingSpeed);
        }

        bool isDeathOrWound = (seq.id >= 5 && seq.id <= 9);
        bool isAttackOrCombat = (seq.id >= 11 && seq.id <= 21);
        bool isSpecial = (seq.id == 2 || seq.id == 3);

        if (!isLoop && (hasFrequency || hasReplay) && isStationary && reasonableDuration &&
            !isDeathOrWound && !isAttackOrCombat && !isSpecial) {
            bool chainsToStand = (seq.nextAnimation == static_cast<int16_t>(mountAnims.stand)) ||
                                 (seq.aliasNext == mountAnims.stand) ||
                                 (seq.nextAnimation == -1);

            mountAnims.fidgets.push_back(seq.id);
            core::Logger::getInstance().debug("  >> Selected fidget: id=", seq.id,
                (chainsToStand ? " (chains to stand)" : ""));
        }
    }

    if (mountAnims.run == 0) mountAnims.run = mountAnims.stand;

    core::Logger::getInstance().debug("Mount animation set: jumpStart=", mountAnims.jumpStart,
        " jumpLoop=", mountAnims.jumpLoop,
        " jumpEnd=", mountAnims.jumpEnd,
        " rearUp=", mountAnims.rearUp,
        " run=", mountAnims.run,
        " stand=", mountAnims.stand,
        " fidgets=", mountAnims.fidgets.size());

    // Configure MountFSM via CharacterAnimator.
    //
    // What is passed here is only what was true at this moment. The FSM takes
    // the per-frame answer as well now, and either counts - a taxi sets the
    // mount up before it says the flight has begun, so this one is false on
    // every flight.
    LOG_INFO("Mount configured: displayId=", mountDisplayId, " taxiFlight=", taxiFlight_);
    characterAnimator_.configureMountFSM(mountAnims, taxiFlight_);

    if (renderer_->getAudioCoordinator()->getMountSoundManager()) {
        bool isFlying = taxiFlight_;
        renderer_->getAudioCoordinator()->getMountSoundManager()->onMount(mountDisplayId, isFlying, modelPath);
    }
}

void AnimationController::clearMount() {
    LOG_INFO("Mount cleared: was taxiFlight=", taxiFlight_, " mountInstanceId=", mountInstanceId_);
    mountInstanceId_ = 0;
    mountHeightOffset_ = 0.0f;
    mountPitch_ = 0.0f;
    mountRoll_ = 0.0f;
    mountSeatAttachmentId_ = -1;
    smoothedMountSeatPos_ = glm::vec3(0.0f);
    mountSeatSmoothingInit_ = false;

    // Clear MountFSM via CharacterAnimator
    characterAnimator_.clearMountFSM();

    if (auto* cameraController = renderer_->getCameraController()) {
        cameraController->setMounted(false);
        cameraController->setMountHeightOffset(0.0f);
    }

    if (renderer_->getAudioCoordinator()->getMountSoundManager()) {
        renderer_->getAudioCoordinator()->getMountSoundManager()->onDismount();
    }
}

// ── Query helpers ────────────────────────────────────────────────────────────

bool AnimationController::isFootstepAnimationState() const {
    auto state = characterAnimator_.getLocomotion().getState();
    return state == LocomotionFSM::State::WALK || state == LocomotionFSM::State::RUN;
}

// ── Melee timers ─────────────────────────────────────────────────────────────

void AnimationController::updateMeleeTimers(float deltaTime) {
    if (meleeSwingCooldown_ > 0.0f) {
        meleeSwingCooldown_ = std::max(0.0f, meleeSwingCooldown_ - deltaTime);
    }
    if (meleeSwingTimer_ > 0.0f) {
        meleeSwingTimer_ = std::max(0.0f, meleeSwingTimer_ - deltaTime);
        if (meleeSwingTimer_ <= 0.0f) specialAttackAnimId_ = 0;
    }
    // Ranged shot timer (same pattern as melee)
    if (rangedShootTimer_ > 0.0f) {
        rangedShootTimer_ = std::max(0.0f, rangedShootTimer_ - deltaTime);
        if (rangedShootTimer_ <= 0.0f && restoreWeaponAfterRangedShot_) {
            restoreWeaponAfterRangedShot_ = false;
            if (rangedShotCompleteCallback_) rangedShotCompleteCallback_();
        }
    }
}

// ── Mount positioning helper ─────────────────────────────────────────────────

// A seat that shifts more than this between frames is being carried, not
// jittering: standing still on solid ground moves it by a fraction of this,
// while a ferry at speed moves it by several times as much in a single frame.
static constexpr float kSeatCarriedStepSq = 0.05f * 0.05f;

void AnimationController::applyMountPositioning(float mountBob, float mountRoll, float characterYaw) {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    if (!characterRenderer || characterInstanceId == 0) return;

    float mountYawRad = glm::radians(characterYaw);

    // Position mount model
    if (mountInstanceId_ > 0) {
        const glm::vec3& characterPosition = renderer_->getCharacterPosition();
        characterRenderer->setInstancePosition(mountInstanceId_, characterPosition);
        characterRenderer->setInstanceRotation(mountInstanceId_, glm::vec3(mountPitch_, mountRoll, mountYawRad));
    }

    // Use mount's attachment point for proper bone-driven rider positioning.
    if (taxiFlight_) {
        glm::mat4 mountSeatTransform(1.0f);
        bool haveSeat = false;
        static constexpr uint32_t kTaxiSeatAttachmentId = 0;
        if (mountSeatAttachmentId_ == -1) {
            mountSeatAttachmentId_ = static_cast<int>(kTaxiSeatAttachmentId);
        }
        if (mountSeatAttachmentId_ >= 0) {
            haveSeat = characterRenderer->getAttachmentTransform(
                mountInstanceId_, static_cast<uint32_t>(mountSeatAttachmentId_), mountSeatTransform);
        }
        // A failed lookup is "not yet", not "never". The seat is read off the
        // mount's model, and a mount summoned before its M2 has finished
        // loading answers no on the first frame or two - latching that closed
        // left the rider on the guessed height for the whole ride.
        if (haveSeat) {
            glm::vec3 targetRiderPos = glm::vec3(mountSeatTransform[3]) + glm::vec3(0.0f, 0.0f, 0.02f);
            mountSeatSmoothingInit_ = false;
            smoothedMountSeatPos_ = targetRiderPos;
            characterRenderer->setInstancePosition(characterInstanceId, targetRiderPos);
        } else {
            mountSeatSmoothingInit_ = false;
            const glm::vec3& characterPosition = renderer_->getCharacterPosition();
            glm::vec3 playerPos = characterPosition + glm::vec3(0.0f, 0.0f, mountHeightOffset_ + 0.10f);
            characterRenderer->setInstancePosition(characterInstanceId, playerPos);
        }

        float riderPitch = mountPitch_ * 0.35f;
        float riderRoll = mountRoll * 0.35f;
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(riderPitch, riderRoll, mountYawRad));
        return;
    }

    // Ground mounts: try a seat attachment first.
    const glm::vec3& characterPosition = renderer_->getCharacterPosition();
    bool moving = renderer_->getCameraController() && renderer_->getCameraController()->isMoving();

    glm::mat4 mountSeatTransform;
    bool haveSeat = false;
    if (mountSeatAttachmentId_ >= 0) {
        haveSeat = characterRenderer->getAttachmentTransform(
            mountInstanceId_, static_cast<uint32_t>(mountSeatAttachmentId_), mountSeatTransform);
    } else if (mountSeatAttachmentId_ == -1) {
        static constexpr uint32_t kSeatAttachments[] = {0, 5, 6, 7, 8};
        for (uint32_t attId : kSeatAttachments) {
            if (characterRenderer->getAttachmentTransform(mountInstanceId_, attId, mountSeatTransform)) {
                mountSeatAttachmentId_ = static_cast<int>(attId);
                haveSeat = true;
                break;
            }
        }
        // Left at -1 on failure, so the next frame probes again - see the note
        // in the taxi branch above.
    }

    if (haveSeat) {
        glm::vec3 mountSeatPos = glm::vec3(mountSeatTransform[3]);
        glm::vec3 seatOffset = glm::vec3(0.0f, 0.0f, taxiFlight_ ? 0.04f : 0.08f);
        glm::vec3 targetRiderPos = mountSeatPos + seatOffset;

        // The smoothing below is for a rider sitting still, where it damps the
        // jitter of a bone-driven seat. `moving` only reports movement *input*,
        // though, and a player standing on a boat presses nothing while the world
        // carries them at the ship's speed. The filter then trailed the rider
        // behind the seat by its own time constant - about two yards at ferry
        // speed, swinging out to one side as the hull turned. Reported as the
        // character riding behind the direction of travel, alongside its mount.
        //
        // So ask whether the seat is actually moving in the world rather than
        // whether the player asked for it to. This covers anything that carries a
        // rider - ships, elevators, a moving platform - without naming any of them.
        const glm::vec3 seatStep = targetRiderPos - lastMountSeatTarget_;
        const bool seatCarried = mountSeatSmoothingInit_ &&
                                 glm::dot(seatStep, seatStep) > kSeatCarriedStepSq;
        lastMountSeatTarget_ = targetRiderPos;

        if (moving || seatCarried) {
            mountSeatSmoothingInit_ = false;
            smoothedMountSeatPos_ = targetRiderPos;
        } else if (!mountSeatSmoothingInit_) {
            smoothedMountSeatPos_ = targetRiderPos;
            mountSeatSmoothingInit_ = true;
        } else {
            float smoothHz = taxiFlight_ ? 10.0f : 14.0f;
            float alpha = 1.0f - std::exp(-smoothHz * std::max(lastDeltaTime_, 0.001f));
            smoothedMountSeatPos_ = glm::mix(smoothedMountSeatPos_, targetRiderPos, alpha);
        }

        characterRenderer->setInstancePosition(characterInstanceId, smoothedMountSeatPos_);

        float yawRad = glm::radians(characterYaw);
        float riderPitch = mountPitch_ * 0.35f;
        float riderRoll = mountRoll * 0.35f;
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(riderPitch, riderRoll, yawRad));
    } else {
        mountSeatSmoothingInit_ = false;
        float yawRad = glm::radians(characterYaw);
        glm::mat4 mountRotation = glm::mat4(1.0f);
        mountRotation = glm::rotate(mountRotation, yawRad, glm::vec3(0.0f, 0.0f, 1.0f));
        mountRotation = glm::rotate(mountRotation, mountRoll, glm::vec3(1.0f, 0.0f, 0.0f));
        mountRotation = glm::rotate(mountRotation, mountPitch_, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::vec3 localOffset(0.0f, 0.0f, mountHeightOffset_ + mountBob);
        glm::vec3 worldOffset = glm::vec3(mountRotation * glm::vec4(localOffset, 0.0f));
        glm::vec3 playerPos = characterPosition + worldOffset;
        characterRenderer->setInstancePosition(characterInstanceId, playerPos);
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(mountPitch_, mountRoll, yawRad));
    }
}

// ── Mounted animation update (uses MountFSM) ────────────────────────────────

void AnimationController::updateMountedAnimation(float deltaTime) {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    auto* cameraController = renderer_->getCameraController();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();
    float characterYaw = renderer_->getCharacterYaw();
    auto& mountFSM = characterAnimator_.getMountFSM();

    // Build MountFSM input
    MountFSM::Input mountIn;
    mountIn.moving = cameraController->isMoving();
    mountIn.movingBackward = cameraController->isMovingBackward();
    mountIn.strafeLeft = cameraController->isStrafingLeft();
    mountIn.strafeRight = cameraController->isStrafingRight();
    mountIn.grounded = cameraController->isGrounded();
    mountIn.jumpKeyPressed = cameraController->isJumpKeyPressed();
    mountIn.flying = cameraController->isFlyingActive();
    mountIn.swimming = cameraController->isSwimming();
    mountIn.ascending = cameraController->isAscending();
    mountIn.descending = cameraController->isDescending();
    mountIn.taxiFlight = taxiFlight_;
    mountIn.deltaTime = deltaTime;
    mountIn.characterYaw = characterYaw;
    // Mount animation state query
    if (mountInstanceId_ > 0 && characterRenderer) {
        mountIn.haveMountState = characterRenderer->getAnimationState(
            mountInstanceId_, mountIn.curMountAnim, mountIn.curMountTime, mountIn.curMountDuration);
    }

    // Evaluate MountFSM
    auto mountOut = mountFSM.evaluate(mountIn);

    // Apply mount animation if changed
    if (mountOut.mountAnimChanged && mountInstanceId_ > 0 && characterRenderer) {
        characterRenderer->playAnimation(mountInstanceId_, mountOut.mountAnimId, mountOut.mountAnimLoop);
    }

    // Rider animation - defaults to MOUNT, but uses MOUNT_FLIGHT_* variants when flying
    uint32_t riderAnim = anim::MOUNT;
    if (cameraController->isFlyingActive()) {
        auto hasRider = [&](uint32_t id) { return characterRenderer->hasAnimation(characterInstanceId, id); };
        if (mountIn.moving) {
            if (cameraController->isAscending() && hasRider(anim::MOUNT_FLIGHT_UP))
                riderAnim = anim::MOUNT_FLIGHT_UP;
            else if (cameraController->isDescending() && hasRider(anim::MOUNT_FLIGHT_DOWN))
                riderAnim = anim::MOUNT_FLIGHT_DOWN;
            else if (hasRider(anim::MOUNT_FLIGHT_FORWARD))
                riderAnim = anim::MOUNT_FLIGHT_FORWARD;
        } else {
            if (hasRider(anim::MOUNT_FLIGHT_IDLE))
                riderAnim = anim::MOUNT_FLIGHT_IDLE;
        }
    }

    // Apply rider animation
    uint32_t currentAnimId = 0;
    float currentAnimTimeMs = 0.0f, currentAnimDurationMs = 0.0f;
    bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
    if (!haveState || currentAnimId != riderAnim) {
        characterRenderer->playAnimation(characterInstanceId, riderAnim, true);
        lastPlayerAnimRequest_ = riderAnim;
        lastPlayerAnimLoopRequest_ = true;
    }

    // Handle mount sounds
    auto* mountSoundMgr = renderer_->getAudioCoordinator()->getMountSoundManager();
    if (mountOut.playJumpSound && mountSoundMgr) {
        mountSoundMgr->playJumpSound();
    }
    if (mountOut.playLandSound && mountSoundMgr) {
        mountSoundMgr->playLandSound();
    }
    if (mountOut.playRearUpSound && mountSoundMgr) {
        mountSoundMgr->playRearUpSound();
    }
    if (mountOut.playIdleSound && mountSoundMgr) {
        mountSoundMgr->playIdleSound();
    }
    if (mountOut.triggerMountJump && cameraController) {
        cameraController->triggerMountJump();
    }

    // Apply positioning (uses mountBob and mountRoll from MountFSM)
    // For taxi flights, use external mountRoll_ set by setMountPitchRoll
    // For ground mounts, use MountFSM's computed lean roll
    float finalRoll = taxiFlight_ ? mountRoll_ : mountOut.mountRoll;
    applyMountPositioning(mountOut.mountBob, finalRoll, characterYaw);
}

// ── Character animation state machine (delegates to CharacterAnimator) ──────────

void AnimationController::updateCharacterAnimation() {
    auto* characterRenderer = renderer_->getCharacterRenderer();
    auto* cameraController = renderer_->getCameraController();
    uint32_t characterInstanceId = renderer_->getCharacterInstanceId();

    // Lazy probe: populate capability set once per model.
    // Re-probe if melee capabilities are missing (model may not have been fully
    // loaded on the first probe attempt).
    if (characterRenderer && characterInstanceId != 0) {
        if (!capabilitiesProbed_) {
            probeCapabilities();
        } else if (meleeSwingTimer_ > 0.0f && !characterAnimator_.getCapabilities().hasMelee) {
            capabilitiesProbed_ = false;
            probeCapabilities();
        }
    }

    // When mounted, delegate to MountFSM and handle positioning
    if (isMounted()) {
        updateMountedAnimation(lastDeltaTime_);
        return;
    }

    // Build FrameInput for CharacterAnimator from camera/renderer state
    CharacterAnimator::FrameInput fi;
    fi.moving = cameraController->isMoving();
    fi.sprinting = cameraController->isSprinting();
    fi.movingForward = cameraController->isMovingForward();
    fi.movingBackward = cameraController->isMovingBackward();
    fi.strafeLeft = cameraController->isStrafingLeft();
    fi.strafeRight = cameraController->isStrafingRight();
    // See setM2TransportRiding() comment: real physics correctly reports "not grounded"
    // over open track under a moving platform, but the animation FSM needs "grounded"
    // to avoid playing a falling animation for the whole ride.
    fi.grounded = m2TransportRiding_ ? true : cameraController->isGrounded();
    fi.jumping = cameraController->isJumping();
    fi.swimming = cameraController->isSwimming();
    fi.sitting = cameraController->isSitting();
    fi.ascending = cameraController->isAscending();
    fi.descending = cameraController->isDescending();
    fi.jumpKeyPressed = cameraController->isJumpKeyPressed();
    fi.characterYaw = renderer_->getCharacterYaw();
    // Melee/ranged timers
    fi.meleeSwingTimer = meleeSwingTimer_;
    fi.rangedShootTimer = rangedShootTimer_;
    fi.specialAttackAnimId = specialAttackAnimId_;
    fi.rangedAnimId = rangedAnimId_;
    // Animation state query for one-shot completion detection
    if (characterRenderer && characterInstanceId > 0) {
        fi.haveAnimState = characterRenderer->getAnimationState(
            characterInstanceId, fi.currentAnimId, fi.currentAnimTime, fi.currentAnimDuration);
    }

    // Inject FrameInput and resolve animation via CharacterAnimator
    characterAnimator_.setFrameInput(fi);
    characterAnimator_.update(lastDeltaTime_);

    // Read the resolved animation output
    AnimOutput output = characterAnimator_.getLastOutput();

    // STAY policy: if CharacterAnimator returns invalid, keep current animation
    if (!output.valid) return;

    uint32_t animId = output.animId;
    bool loop = output.loop;

    // Apply animation to the character renderer
    uint32_t currentAnimId = 0;
    float currentAnimTimeMs = 0.0f;
    float currentAnimDurationMs = 0.0f;
    bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
    const bool requestChanged = (lastPlayerAnimRequest_ != animId) || (lastPlayerAnimLoopRequest_ != loop);
    // Only re-assert looping animations if the renderer drifted (e.g., external
    // playAnimation call).  One-shot animations must NOT be re-asserted after the
    // renderer auto-resets them to STAND on completion - the FSM detects the ID
    // change via oneShotComplete and transitions to the next state in the same frame.
    const bool drifted = haveState && currentAnimId != animId && loop;
    const bool shouldPlay = requestChanged || drifted;

    // Debug: log animation decisions (only when animation changes or replays)
    static uint32_t dbgLastAnim = UINT32_MAX;
    if (shouldPlay || animId != dbgLastAnim) {
        LOG_DEBUG("[AnimDbg] FSM→", animId, " loop=", loop,
                  " cur=", currentAnimId, " t=", currentAnimTimeMs, "/", currentAnimDurationMs,
                  " haveState=", haveState,
                  " reqChanged=", requestChanged, " drifted=", drifted, " shouldPlay=", shouldPlay,
                  " lastReq=", lastPlayerAnimRequest_,
                  " locoState=", static_cast<int>(characterAnimator_.getLocomotion().getState()),
                  " actState=", static_cast<int>(characterAnimator_.getActivity().getState()),
                  " fwd=", fi.movingForward, " back=", fi.movingBackward,
                  " strafeL=", fi.strafeLeft, " strafeR=", fi.strafeRight,
                  " moving=", fi.moving, " sprinting=", fi.sprinting);
        dbgLastAnim = animId;
    }

    if (shouldPlay) {
        characterRenderer->playAnimation(characterInstanceId, animId, loop);
        lastPlayerAnimRequest_ = animId;
        lastPlayerAnimLoopRequest_ = loop;
    }
}

// ── Footstep update (delegated to FootstepDriver) ───────────────────────────

void AnimationController::updateFootsteps(float deltaTime) {
    footstepDriver_.update(deltaTime, renderer_, isMounted(), mountInstanceId_,
                            taxiFlight_, isFootstepAnimationState());
}

// ── Activity SFX state tracking ──────────────────────────────────────────────

void AnimationController::updateSfxState(float deltaTime) {
    sfxStateDriver_.update(deltaTime, renderer_, isMounted(), taxiFlight_,
                           footstepDriver_);
}

} // namespace rendering
} // namespace wowee
