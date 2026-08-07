#include "core/animation_callback_handler.hpp"
#include "core/appearance_composer.hpp"
#include "core/entity_spawner.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "rendering/animation/emote_registry.hpp"
#include "game/game_handler.hpp"
#include "game/spell_classification.hpp"
#include "game/world_packets.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/ui_sound_manager.hpp"

#include <cmath>
#include <algorithm>

namespace wowee { namespace core {

AnimationCallbackHandler::AnimationCallbackHandler(
    EntitySpawner& entitySpawner,
    rendering::Renderer& renderer,
    game::GameHandler& gameHandler,
    AppearanceComposer& appearanceComposer)
    : entitySpawner_(entitySpawner)
    , renderer_(renderer)
    , gameHandler_(gameHandler)
    , appearanceComposer_(appearanceComposer)
{
}

void AnimationCallbackHandler::resetChargeState() {
    chargeActive_ = false;
    chargeTimer_ = 0.0f;
    chargeDuration_ = 0.0f;
    chargeTargetGuid_ = 0;
}

bool AnimationCallbackHandler::updateCharge(float deltaTime) {
    if (!chargeActive_) return false;

    // Warrior Charge: lerp position from start to end using smoothstep
    chargeTimer_ += deltaTime;
    float t = std::min(chargeTimer_ / chargeDuration_, 1.0f);
    // smoothstep for natural acceleration/deceleration
    float s = t * t * (3.0f - 2.0f * t);
    glm::vec3 renderPos = chargeStartPos_ + (chargeEndPos_ - chargeStartPos_) * s;
    renderer_.getCharacterPosition() = renderPos;

    // Keep facing toward target and emit charge effect
    glm::vec3 dir = chargeEndPos_ - chargeStartPos_;
    float dirLenSq = glm::dot(dir, dir);
    if (dirLenSq > 1e-4f) {
        dir *= glm::inversesqrt(dirLenSq);
        float yawDeg = glm::degrees(std::atan2(dir.x, dir.y));
        renderer_.setCharacterYaw(yawDeg);
        if (auto* ac = renderer_.getAnimationController()) ac->emitChargeEffect(renderPos, dir);
    }

    // Sync to game handler
    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
    gameHandler_.setPosition(canonical.x, canonical.y, canonical.z);

    // Update camera follow target
    if (renderer_.getCameraController()) {
        glm::vec3* followTarget = renderer_.getCameraController()->getFollowTargetMutable();
        if (followTarget) {
            *followTarget = renderPos;
        }
    }

    // Charge complete
    if (t >= 1.0f) {
        chargeActive_ = false;
        if (auto* ac = renderer_.getAnimationController()) ac->setCharging(false);
        if (auto* ac = renderer_.getAnimationController()) ac->stopChargeEffect();
        renderer_.getCameraController()->setExternalFollow(false);
        renderer_.getCameraController()->setExternalMoving(false);

        // Snap to melee range of target's CURRENT position (it may have moved)
        if (chargeTargetGuid_ != 0) {
            auto targetEntity = gameHandler_.getEntityManager().getEntity(chargeTargetGuid_);
            if (targetEntity) {
                glm::vec3 targetCanonical(targetEntity->getX(), targetEntity->getY(), targetEntity->getZ());
                glm::vec3 targetRender = core::coords::canonicalToRender(targetCanonical);
                glm::vec3 toTarget = targetRender - renderPos;
                float dSq = glm::dot(toTarget, toTarget);
                if (dSq > 2.25f) {
                    // Place us 1.5 units from target (well within 8-unit melee range)
                    glm::vec3 snapPos = targetRender - toTarget * (1.5f * glm::inversesqrt(dSq));
                    renderer_.getCharacterPosition() = snapPos;
                    glm::vec3 snapCanonical = core::coords::renderToCanonical(snapPos);
                    gameHandler_.setPosition(snapCanonical.x, snapCanonical.y, snapCanonical.z);
                    if (renderer_.getCameraController()) {
                        glm::vec3* ft = renderer_.getCameraController()->getFollowTargetMutable();
                        if (ft) *ft = snapPos;
                    }
                }
            }
            gameHandler_.startAutoAttack(chargeTargetGuid_);
            if (auto* ac = renderer_.getAnimationController()) ac->triggerMeleeSwing();
        }

        // Send movement heartbeat so server knows our new position
        gameHandler_.sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
    }

    return true; // charge is active
}

void AnimationCallbackHandler::setupCallbacks() {
    // Sprint aura callback — use SPRINT(143) animation when sprint-type buff is active
    gameHandler_.setSprintAuraCallback([this](bool active) {
        auto* ac = renderer_.getAnimationController();
        if (ac) ac->setSprintAuraActive(active);
    });

    // Vehicle state callback — hide player character when inside a vehicle
    gameHandler_.setVehicleStateCallback([this](bool entered, uint32_t /*vehicleId*/) {
        auto* cr = renderer_.getCharacterRenderer();
        uint32_t instId = renderer_.getCharacterInstanceId();
        if (!cr || instId == 0) return;
        cr->setInstanceVisible(instId, !entered);
    });

    // Charge callback — warrior rushes toward target
    gameHandler_.setChargeCallback([this](uint64_t targetGuid, float tx, float ty, float tz) {
        if (!renderer_.getCameraController()) return;

        // Get current player position in render coords
        glm::vec3 startRender = renderer_.getCharacterPosition();
        // Convert target from canonical to render
        glm::vec3 targetRender = core::coords::canonicalToRender(glm::vec3(tx, ty, tz));

        // Compute direction and stop 2.0 units short (melee reach)
        glm::vec3 dir = targetRender - startRender;
        float distSq = glm::dot(dir, dir);
        if (distSq < 9.0f) return; // Too close, nothing to do
        float invDist = glm::inversesqrt(distSq);
        glm::vec3 dirNorm = dir * invDist;
        glm::vec3 endRender = targetRender - dirNorm * 2.0f;

        // Face toward target BEFORE starting charge
        float yawRad = std::atan2(dirNorm.x, dirNorm.y);
        float yawDeg = glm::degrees(yawRad);
        renderer_.setCharacterYaw(yawDeg);
        // Sync canonical orientation to server so it knows we turned
        float canonicalYaw = core::coords::characterYawDegToCanonical(yawDeg);
        gameHandler_.setOrientation(canonicalYaw);
        gameHandler_.sendMovement(game::Opcode::MSG_MOVE_SET_FACING);

        // Set charge state
        chargeActive_ = true;
        chargeTimer_ = 0.0f;
        chargeDuration_ = std::max(std::sqrt(distSq) / 25.0f, 0.3f); // ~25 units/sec
        chargeStartPos_ = startRender;
        chargeEndPos_ = endRender;
        chargeTargetGuid_ = targetGuid;

        // Disable player input, play charge animation
        renderer_.getCameraController()->setExternalFollow(true);
        renderer_.getCameraController()->clearMovementInputs();
        if (auto* ac = renderer_.getAnimationController()) ac->setCharging(true);

        // Start charge visual effect (red haze + dust)
        glm::vec3 chargeDir = glm::normalize(endRender - startRender);
        if (auto* ac = renderer_.getAnimationController()) ac->startChargeEffect(startRender, chargeDir);

        // Charge's cast kit points at SoundEntries 3330 ("HeroicLeap"), whose file name
        // carries Blizzard's own typo. There is no Sound\Spells\Charge.wav.
        auto& audio = audio::AudioEngine::instance();
        if (!audio.playSound2D("Sound\\Spells\\HeroricLeap.wav", 0.8f)) {
            // Fallback: weapon whoosh
            audio.playSound2D("Sound\\Item\\Weapons\\WeaponSwings\\mWooshLarge1.wav", 0.9f);
        }
    });

    // GUID → CharacterRenderer instance for bone-attached spell visuals:
    // local player uses the dedicated character instance, other units the
    // spawner's creature/player instances (0 = not rendered, no attachment).
    gameHandler_.setUnitRenderInstanceResolver([this](uint64_t guid) -> uint32_t {
        if (guid == gameHandler_.getPlayerGuid()) return renderer_.getCharacterInstanceId();
        uint32_t instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(guid);
        return instanceId;
    });

    // NPC/player death callback (online mode) - play death animation
    gameHandler_.setNpcDeathCallback([this](uint64_t guid) {
        entitySpawner_.markCreatureDead(guid);
        if (!renderer_.getCharacterRenderer()) return;
        uint32_t instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (instanceId != 0) {
            renderer_.getCharacterRenderer()->playAnimation(instanceId, rendering::anim::DEATH, false);
        }
    });

    // NPC/player respawn callback (online mode) - play rise animation then idle
    gameHandler_.setNpcRespawnCallback([this](uint64_t guid) {
        entitySpawner_.unmarkCreatureDead(guid);
        if (!renderer_.getCharacterRenderer()) return;
        uint32_t instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (instanceId != 0) {
            auto* cr = renderer_.getCharacterRenderer();
            // Play RISE one-shot (auto-returns to STAND when finished), fall back to STAND
            if (cr->hasAnimation(instanceId, rendering::anim::RISE))
                cr->playAnimation(instanceId, rendering::anim::RISE, false);
            else
                cr->playAnimation(instanceId, rendering::anim::STAND, true);
        }
    });

    // NPC/player swing callback (online mode) - play attack animation
    // Probes the model for the best available attack animation:
    //   ATTACK_1H(17) → ATTACK_2H(18) → ATTACK_2H_LOOSE(19) → ATTACK_UNARMED(16)
    gameHandler_.setNpcSwingCallback([this](uint64_t guid) {
        if (!renderer_.getCharacterRenderer()) return;
        uint32_t instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (instanceId != 0) {
            auto* cr = renderer_.getCharacterRenderer();
            static const uint32_t attackAnims[] = {
                rendering::anim::ATTACK_1H,
                rendering::anim::ATTACK_2H,
                rendering::anim::ATTACK_2H_LOOSE,
                rendering::anim::ATTACK_UNARMED
            };
            bool played = false;
            for (uint32_t anim : attackAnims) {
                if (cr->hasAnimation(instanceId, anim)) {
                    cr->playAnimation(instanceId, anim, false);
                    played = true;
                    break;
                }
            }
            if (!played) cr->playAnimation(instanceId, rendering::anim::ATTACK_UNARMED, false);
        }
    });

    // Hit reaction callback — plays one-shot dodge/block/wound animation on the victim
    gameHandler_.setHitReactionCallback([this](uint64_t victimGuid, game::GameHandler::HitReaction reaction) {
        auto* cr = renderer_.getCharacterRenderer();
        if (!cr) return;

        // Determine animation based on reaction type
        uint32_t animId = rendering::anim::COMBAT_WOUND;
        switch (reaction) {
            case game::GameHandler::HitReaction::DODGE:      animId = rendering::anim::DODGE; break;
            case game::GameHandler::HitReaction::PARRY:      break; // Parry already handled by existing system
            case game::GameHandler::HitReaction::BLOCK:      animId = rendering::anim::BLOCK; break;
            case game::GameHandler::HitReaction::SHIELD_BLOCK: animId = rendering::anim::SHIELD_BLOCK; break;
            case game::GameHandler::HitReaction::CRIT_WOUND: animId = rendering::anim::COMBAT_CRITICAL; break;
            case game::GameHandler::HitReaction::WOUND:      animId = rendering::anim::COMBAT_WOUND; break;
        }

        // For local player: use AnimationController state
        bool isLocalPlayer = (victimGuid == gameHandler_.getPlayerGuid());
        if (isLocalPlayer) {
            auto* ac = renderer_.getAnimationController();
            if (ac) {
                uint32_t charInstId = renderer_.getCharacterInstanceId();
                if (charInstId && cr->hasAnimation(charInstId, animId))
                    ac->triggerHitReaction(animId);
            }
            return;
        }

        // For NPCs/other players: direct playAnimation
        uint32_t instanceId = entitySpawner_.getCreatureInstanceId(victimGuid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(victimGuid);
        if (instanceId != 0 && cr->hasAnimation(instanceId, animId))
            cr->playAnimation(instanceId, animId, false);
    });

    // Stun state callback — enters/exits STUNNED animation on local player
    gameHandler_.setStunStateCallback([this](bool stunned) {
        auto* ac = renderer_.getAnimationController();
        if (ac) ac->setStunned(stunned);
    });

    // Stealth state callback — switches to stealth animation variants
    gameHandler_.setStealthStateCallback([this](bool stealthed) {
        auto* ac = renderer_.getAnimationController();
        if (ac) ac->setStealthed(stealthed);
    });

    // Player health callback — switches to wounded idle when HP < 20%
    gameHandler_.setPlayerHealthCallback([this](uint32_t health, uint32_t maxHealth) {
        auto* ac = renderer_.getAnimationController();
        if (!ac) return;
        bool lowHp = (maxHealth > 0) && (health > 0) && (health * 5 <= maxHealth);
        ac->setLowHealth(lowHp);
    });

    // Unit animation hint callback — plays jump (38=JumpMid) animation on other players/NPCs.
    // Swim/walking state is now authoritative from the move-flags callback below.
    // animId=38 (JumpMid): airborne jump animation; land detection is via per-frame sync.
    gameHandler_.setUnitAnimHintCallback([this](uint64_t guid, uint32_t animId) {
        auto* cr = renderer_.getCharacterRenderer();
        if (!cr) return;
        uint32_t instanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) return;
        // Don't override Death animation
        uint32_t curAnim = 0; float curT = 0.0f, curDur = 0.0f;
        if (cr->getAnimationState(instanceId, curAnim, curT, curDur) && curAnim == rendering::anim::DEATH) return;
        cr->playAnimation(instanceId, animId, /*loop=*/true);
    });

    // Unit move-flags callback — updates swimming and walking state from every MSG_MOVE_* packet.
    // This is more reliable than opcode-based hints for cold joins and heartbeats:
    // a player already swimming when we join will have SWIMMING set on the first heartbeat.
    // Walking(4) vs Running(5) is also driven here from the WALKING flag.
    gameHandler_.setUnitMoveFlagsCallback([this](uint64_t guid, uint32_t moveFlags) {
        const bool isSwimming = (moveFlags & static_cast<uint32_t>(game::MovementFlags::SWIMMING)) != 0;
        const bool isWalking  = (moveFlags & static_cast<uint32_t>(game::MovementFlags::WALKING))  != 0;
        const bool isFlying   = (moveFlags & static_cast<uint32_t>(game::MovementFlags::FLYING))   != 0;
        auto& swimState = entitySpawner_.getCreatureSwimmingState();
        auto& walkState = entitySpawner_.getCreatureWalkingState();
        auto& flyState  = entitySpawner_.getCreatureFlyingState();
        if (isSwimming) swimState[guid] = true;
        else            swimState.erase(guid);
        if (isWalking)  walkState[guid] = true;
        else            walkState.erase(guid);
        if (isFlying)   flyState[guid] = true;
        else            flyState.erase(guid);
    });

    // Emote animation callback — play server-driven emote animations on NPCs and other players.
    // isState marks persistent STATE_ emotes (UNIT_NPC_EMOTESTATE or state-type
    // SMSG_EMOTE): they loop until cleared and are retained across model
    // reloads. One-shots play once and resume the retained state loop.
    gameHandler_.setEmoteAnimCallback([this](uint64_t guid, uint32_t emoteAnim, bool isState) {
        auto* cr = renderer_.getCharacterRenderer();
        if (!cr) return;
        auto entity = gameHandler_.getEntityManager().getEntity(guid);
        const bool isCreature = entity && entity->getType() == game::ObjectType::UNIT;
        auto& activeEmotes = entitySpawner_.getCreatureActiveEmotes();
        if (isCreature && isState) {
            // State emotes can arrive while the creature's model is still loading.
            // Retain the raw animation so spawnOnlineCreature can apply it once the
            // correct display is instantiated.
            if (emoteAnim == 0) activeEmotes.erase(guid);
            else                activeEmotes[guid] = emoteAnim;
        }
        // Look up creature instance first, then online players
        uint32_t emoteInstanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (emoteInstanceId != 0) {
            if (emoteAnim == 0) {
                // A state clear returns to idle. A one-shot cancel (SMSG_EMOTE 0)
                // must not stomp a persistent UNIT_NPC_EMOTESTATE work loop —
                // resume it instead (Westfall woodworkers hammer through these).
                if (isState) activeEmotes.erase(guid);
                auto retained = activeEmotes.find(guid);
                uint32_t resumeAnim = rendering::anim::STAND;
                if (retained != activeEmotes.end() &&
                    cr->hasAnimation(emoteInstanceId, retained->second)) {
                    resumeAnim = retained->second;
                }
                cr->playAnimation(emoteInstanceId, resumeAnim, true);
            } else if (isState) {
                const uint32_t stateAnim =
                    rendering::EmoteRegistry::instance().getStateVariant(emoteAnim);
                const uint32_t persistentAnim =
                    stateAnim != 0 && cr->hasAnimation(emoteInstanceId, stateAnim)
                        ? stateAnim : emoteAnim;
                activeEmotes[guid] = persistentAnim;
                cr->playAnimation(emoteInstanceId, persistentAnim, true);
            } else {
                // One-shot: play once, then resume the retained state loop (or Stand).
                auto retained = activeEmotes.find(guid);
                uint32_t resumeAnim = 0;
                if (retained != activeEmotes.end() &&
                    cr->hasAnimation(emoteInstanceId, retained->second)) {
                    resumeAnim = retained->second;
                }
                cr->playAnimation(emoteInstanceId, emoteAnim, false, resumeAnim);
            }
            return;
        }
        emoteInstanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (emoteInstanceId != 0) {
            if (emoteAnim == 0) {
                cr->playAnimation(emoteInstanceId, rendering::anim::STAND, true);
            } else if (isState) {
                // Emotes.dbc state entries commonly point at the introductory
                // one-shot (dance, laugh, eat, etc.). Other players need the
                // same looping STATE_* resolution as NPCs or they visibly
                // repeat the intro instead of holding the real emote pose.
                const uint32_t stateAnim =
                    rendering::EmoteRegistry::instance().getStateVariant(emoteAnim);
                const uint32_t persistentAnim =
                    stateAnim != 0 && cr->hasAnimation(emoteInstanceId, stateAnim)
                        ? stateAnim : emoteAnim;
                cr->playAnimation(emoteInstanceId, persistentAnim, true);
            } else {
                cr->playAnimation(emoteInstanceId, emoteAnim, false);
            }
        }
    });

    // Spell cast animation callback — play cast animation on caster (player or NPC/other player)
    // WoW-accurate 3-phase spell animation sequence:
    //   SPELL_PRECAST (31)              — one-shot wind-up
    //   READY_SPELL_DIRECTED/OMNI (51/52) — looping hold while cast bar fills
    //   SPELL_CAST_DIRECTED/OMNI/AREA (53/54/33) — one-shot release at completion
    // Channels use CHANNEL_CAST_DIRECTED/OMNI (124/125) or SPELL_CHANNEL_DIRECTED_OMNI (201).
    // castType comes from the spell packet's targetGuid:
    //   DIRECTED — spell targets a specific unit  (Frostbolt, Heal)
    //   OMNI     — self-cast / no explicit target (Arcane Explosion, buffs)
    //   AREA     — ground-targeted AoE           (Blizzard, Rain of Fire)
    gameHandler_.setSpellCastAnimCallback([this](uint64_t guid, bool start, bool isChannel,
                                                  game::SpellCastType castType) {
        auto* cr = renderer_.getCharacterRenderer();
        if (!cr) return;

        // Determine if this is the local player
        bool isLocalPlayer = false;
        uint32_t instanceId = 0;
        {
            uint32_t charInstId = renderer_.getCharacterInstanceId();
            if (charInstId != 0 && guid == gameHandler_.getPlayerGuid()) {
                instanceId = charInstId;
                isLocalPlayer = true;
            }
        }
        if (instanceId == 0) instanceId = entitySpawner_.getCreatureInstanceId(guid);
        if (instanceId == 0) instanceId = entitySpawner_.getPlayerInstanceId(guid);
        if (instanceId == 0) return;

        const bool isDirected = (castType == game::SpellCastType::DIRECTED);
        const bool isArea     = (castType == game::SpellCastType::AREA);

        if (start) {
            uint32_t currentSpell = isLocalPlayer ? gameHandler_.getCurrentCastSpellId() : 0;
            bool isFishing = isChannel && game::spellclass::isFishingCast(currentSpell);

            // Helper: pick first animation the model supports from a list
            auto pickFirst = [&](std::initializer_list<uint32_t> ids) -> uint32_t {
                for (uint32_t id : ids)
                    if (cr->hasAnimation(instanceId, id)) return id;
                return 0;
            };

            bool isBandage = false;
            bool isMining = false;
            bool isGathering = false;
            game::spellclass::RestChannelKind restChannel =
                game::spellclass::RestChannelKind::NONE;
            if (isLocalPlayer && currentSpell != 0) {
                gameHandler_.loadSpellNameCache();
                auto it = gameHandler_.spellNameCacheRef().find(currentSpell);
                isBandage = (it != gameHandler_.spellNameCacheRef().end() &&
                             it->second.name.find("Bandage") != std::string::npos);
                if (it != gameHandler_.spellNameCacheRef().end()) {
                    restChannel = game::spellclass::classifyRestChannel(it->second.name);
                }
                if (!isBandage && !isFishing && it != gameHandler_.spellNameCacheRef().end()) {
                    const auto& nm = it->second.name;
                    isMining = (nm == "Mining" || nm == "Smelting");
                    if (!isMining) {
                        isGathering = gameHandler_.isProfessionSpell(currentSpell);
                        if (!isGathering)
                            isGathering = (nm == "Opening" || nm == "Open Lock");
                    }
                }
            }

            if (restChannel != game::spellclass::RestChannelKind::NONE) {
                if (auto* ac = renderer_.getAnimationController()) {
                    if (restChannel == game::spellclass::RestChannelKind::CANNIBALIZE) {
                        // Standing, and looping for the length of the channel.
                        //
                        // Not the food path: that seats the player and swaps
                        // the seated idle, and cannibalize does not sit — the
                        // character stays on their feet over the corpse. Not
                        // the potion path either, which is one swig and done,
                        // where this runs until the channel ends.
                        uint32_t eatAnim = pickFirst({rendering::anim::EMOTE_EAT,
                                                      rendering::anim::EATING_LOOP});
                        if (eatAnim != 0) {
                            ac->startSpellCast(0, eatAnim, /*castLoop=*/true, 0);
                        }
                    } else if (restChannel == game::spellclass::RestChannelKind::POTION ||
                        restChannel == game::spellclass::RestChannelKind::ALCOHOL) {
                        // Potions/alcohol are drunk standing: one EmoteEat swig.
                        uint32_t swigAnim = pickFirst({rendering::anim::EMOTE_EAT});
                        if (swigAnim != 0) {
                            ac->startSpellCast(0, swigAnim, false, 0);
                        }
                    } else {
                        // Weapons go straight to the back before the sit — the real
                        // client auto-sheathes on consume with no sheath animation
                        // (playing one here would fight the FSM's sit-down one-shot).
                        if (!appearanceComposer_.isWeaponsSheathed()) {
                            appearanceComposer_.setWeaponsSheathed(true);
                            appearanceComposer_.loadEquippedWeapons();
                        }
                        // Food and water keep the player seated (UNIT_FIELD_BYTES_1)
                        // with the plain seated idle, exactly like the real client:
                        // player models ship NO seated eating animation — EmoteEat is
                        // authored standing (root bone stays at stand height) and
                        // looping it while seated pops the model upright. Only
                        // override the seated idle for models that truly have the
                        // dedicated EatingLoop sequence.
                        ac->setSeatedLoopAnimation(
                            pickFirst({rendering::anim::EATING_LOOP}));
                    }
                }
                // Food/water consume sounds repeat from SpellHandler::updateTimers
                // for the whole aura; potions get their single swig sound here.
                if (restChannel == game::spellclass::RestChannelKind::POTION ||
                    restChannel == game::spellclass::RestChannelKind::ALCOHOL) {
                    if (auto* audio = gameHandler_.services().audioCoordinator) {
                        if (auto* ui = audio->getUiSoundManager()) {
                            ui->playDrinking();
                        }
                    }
                }
            } else if (isMining) {
                appearanceComposer_.showMiningPick(true);
                uint32_t mineAnim = pickFirst({
                    rendering::anim::ATTACK_1H,
                    rendering::anim::EMOTE_WORK,
                    rendering::anim::EMOTE_USE_STANDING
                });
                if (auto* ac = renderer_.getAnimationController()) {
                    ac->startSpellCast(0, mineAnim ? mineAnim : rendering::anim::STAND, true, 0);
                }
            } else if (isBandage || isGathering) {
                uint32_t useStart = isChannel ? 0 : pickFirst({
                    rendering::anim::USE_STANDING_START,
                    rendering::anim::EMOTE_USE_STANDING_NO_SHEATHE,
                    rendering::anim::EMOTE_USE_STANDING
                });
                uint32_t useLoop = pickFirst({
                    rendering::anim::USE_STANDING_LOOP,
                    rendering::anim::EMOTE_USE_STANDING_NO_SHEATHE,
                    rendering::anim::EMOTE_USE_STANDING,
                    rendering::anim::CHANNEL_CAST_OMNI,
                    rendering::anim::READY_SPELL_OMNI,
                    rendering::anim::STAND
                });
                uint32_t useEnd = isChannel ? 0 : pickFirst({
                    rendering::anim::USE_STANDING_END,
                    rendering::anim::STAND
                });
                if (auto* ac = renderer_.getAnimationController()) {
                    ac->startSpellCast(useStart, useLoop ? useLoop : rendering::anim::STAND, true, useEnd);
                }
            } else if (isFishing && cr->hasAnimation(instanceId, rendering::anim::FISHING_LOOP)) {
                // Fishing is a one-shot pole cast followed by the channel idle. If the
                // player had weapons sheathed, move the equipped pole into their hand
                // before starting so the cast animation actually swings it.
                if (isLocalPlayer) {
                    if (appearanceComposer_.isWeaponsSheathed()) {
                        appearanceComposer_.setWeaponsSheathed(false);
                        appearanceComposer_.loadEquippedWeapons();
                    }
                    appearanceComposer_.showFishingPole(true);
                    auto* ac = renderer_.getAnimationController();
                    if (ac) {
                        uint32_t castAnim = cr->hasAnimation(instanceId, rendering::anim::FISHING_CAST)
                            ? rendering::anim::FISHING_CAST : 0;
                        ac->startSpellCast(castAnim, rendering::anim::FISHING_LOOP, true, 0);
                    }
                } else {
                    cr->playAnimation(instanceId, rendering::anim::FISHING_LOOP, true);
                }
            } else {
            // Precast wind-up (one-shot, non-channels only)
            uint32_t precastAnim = 0;
            if (!isChannel) {
                precastAnim = pickFirst({rendering::anim::SPELL_PRECAST});
            }

            // Cast hold (looping while cast bar fills / channel active)
            uint32_t castAnim = 0;
            if (isChannel) {
                // Channel hold: prefer DIRECTED/OMNI based on spell target classification
                if (isDirected) {
                    castAnim = pickFirst({
                        rendering::anim::CHANNEL_CAST_DIRECTED,
                        rendering::anim::CHANNEL_CAST_OMNI,
                        rendering::anim::SPELL_CHANNEL_DIRECTED_OMNI,
                        rendering::anim::READY_SPELL_DIRECTED,
                        rendering::anim::SPELL
                    });
                } else {
                    // OMNI or AREA channels (Blizzard channel, Tranquility, etc.)
                    castAnim = pickFirst({
                        rendering::anim::CHANNEL_CAST_OMNI,
                        rendering::anim::CHANNEL_CAST_DIRECTED,
                        rendering::anim::SPELL_CHANNEL_DIRECTED_OMNI,
                        rendering::anim::READY_SPELL_OMNI,
                        rendering::anim::SPELL
                    });
                }
            } else {
                // Regular cast hold: READY_SPELL_DIRECTED/OMNI while cast bar fills
                if (isDirected) {
                    castAnim = pickFirst({
                        rendering::anim::READY_SPELL_DIRECTED,
                        rendering::anim::READY_SPELL_OMNI,
                        rendering::anim::SPELL_CAST_DIRECTED,
                        rendering::anim::SPELL_CAST,
                        rendering::anim::SPELL
                    });
                } else {
                    // OMNI (self-buff) or AREA (AoE targeting)
                    castAnim = pickFirst({
                        rendering::anim::READY_SPELL_OMNI,
                        rendering::anim::READY_SPELL_DIRECTED,
                        rendering::anim::SPELL_CAST_OMNI,
                        rendering::anim::SPELL_CAST,
                        rendering::anim::SPELL
                    });
                }
            }
            // TBC models may omit the legacy SPELL (2) sequence; idle is a
            // quieter fallback than asking the renderer to play a missing anim.
            if (castAnim == 0) castAnim = rendering::anim::STAND;

            // Finalization release (one-shot after cast completes)
            // Animation chosen by spell target type: AREA → SPELL_CAST_AREA,
            // DIRECTED → SPELL_CAST_DIRECTED, OMNI → SPELL_CAST_OMNI
            uint32_t finalizeAnim = 0;
            if (isLocalPlayer && !isChannel) {
                if (isArea) {
                    // Ground-targeted AoE: SPELL_CAST_AREA → SPELL_CAST_OMNI
                    finalizeAnim = pickFirst({
                        rendering::anim::SPELL_CAST_AREA,
                        rendering::anim::SPELL_CAST_OMNI,
                        rendering::anim::SPELL_CAST,
                        rendering::anim::SPELL
                    });
                } else if (isDirected) {
                    // Single-target: SPELL_CAST_DIRECTED → SPELL_CAST_OMNI
                    finalizeAnim = pickFirst({
                        rendering::anim::SPELL_CAST_DIRECTED,
                        rendering::anim::SPELL_CAST_OMNI,
                        rendering::anim::SPELL_CAST,
                        rendering::anim::SPELL
                    });
                } else {
                    // OMNI (self-buff, Arcane Explosion): SPELL_CAST_OMNI → SPELL_CAST_AREA
                    finalizeAnim = pickFirst({
                        rendering::anim::SPELL_CAST_OMNI,
                        rendering::anim::SPELL_CAST_AREA,
                        rendering::anim::SPELL_CAST,
                        rendering::anim::SPELL
                    });
                }
            }

            if (isLocalPlayer) {
                auto* ac = renderer_.getAnimationController();
                if (ac) ac->startSpellCast(precastAnim, castAnim, true, finalizeAnim);
            } else {
                cr->playAnimation(instanceId, castAnim, true);
            }
            } // end !isFishing
        } else {
            // Cast/channel ended — plays finalization anim completely then returns to idle
            if (isLocalPlayer) {
                appearanceComposer_.showMiningPick(false);
                appearanceComposer_.showFishingPole(false);
                auto* ac = renderer_.getAnimationController();
                if (ac) {
                    ac->setSeatedLoopAnimation(0);
                    ac->stopSpellCast();
                }
            } else if (isChannel) {
                cr->playAnimation(instanceId, rendering::anim::STAND, true);
            }
        }
    });

    // Ghost state callback — make player semi-transparent when in spirit form
    gameHandler_.setGhostStateCallback([this](bool isGhost) {
        auto* cr = renderer_.getCharacterRenderer();
        if (!cr) return;
        uint32_t charInstId = renderer_.getCharacterInstanceId();
        if (charInstId == 0) return;
        cr->setInstanceOpacity(charInstId, isGhost ? 0.5f : 1.0f);
    });

    // Stand state animation callback — route through AnimationController state machine
    // for proper sit/sleep/kneel transition animations (down → loop → up)
    gameHandler_.setStandStateCallback([this](uint8_t standState) {
        using AC = rendering::AnimationController;

        // Sync camera controller sitting flag: block movement while sitting/kneeling
        if (auto* cc = renderer_.getCameraController()) {
            cc->setSitting(standState >= AC::STAND_STATE_SIT &&
                           standState <= AC::STAND_STATE_KNEEL &&
                           standState != AC::STAND_STATE_DEAD);
        }

        auto* ac = renderer_.getAnimationController();
        if (!ac) return;

        // Death is special — play directly, not through sit state machine
        if (standState == AC::STAND_STATE_DEAD) {
            auto* cr = renderer_.getCharacterRenderer();
            if (!cr) return;
            uint32_t charInstId = renderer_.getCharacterInstanceId();
            if (charInstId == 0) return;
            cr->playAnimation(charInstId, rendering::anim::DEATH, false);
            return;
        }

        ac->setStandState(standState);
        // Restoration food/water is an aura rather than MSG_CHANNEL_START on
        // supported realms. The aura update starts its loop; standing up ends it.
        // Only stop the cast presentation while restoring — the server also
        // stands a seated player when a real cast begins, and stopping there
        // would kill that cast's animation as it starts.
        if (standState == AC::STAND_STATE_STAND) {
            ac->setSeatedLoopAnimation(0);
            if (gameHandler_.isRestoring()) ac->stopSpellCast();
        }
    });

    // Loot window callback — play kneel/loot animation while looting
    gameHandler_.setLootWindowCallback([this](bool open) {
        auto* ac = renderer_.getAnimationController();
        if (!ac) return;
        if (open) ac->startLooting();
        else      ac->stopLooting();
    });
}

}} // namespace wowee::core
