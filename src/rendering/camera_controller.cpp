#include "rendering/camera_controller.hpp"
#include "rendering/movement_limits.hpp"
#include <algorithm>
#include <future>
#include "core/thread_pool.hpp"
#include <imgui.h>
#include "rendering/terrain_manager.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "game/opcodes.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {

namespace {

constexpr float kMaxPhysicsDelta = 1.0f / 30.0f;

std::optional<float> selectReachableFloor(const std::optional<float>& terrainH,
                                          const std::optional<float>& wmoH,
                                          float refZ,
                                          float maxStepUp) {
    // Filter to reachable floors (not too far above)
    std::optional<float> reachTerrain;
    std::optional<float> reachWmo;
    if (terrainH && *terrainH <= refZ + maxStepUp) reachTerrain = terrainH;
    if (wmoH && *wmoH <= refZ + maxStepUp) reachWmo = wmoH;

    if (reachTerrain && reachWmo) {
        // Prefer the highest surface — prevents clipping through
        // WMO floors that sit above terrain.
        return (*reachWmo >= *reachTerrain) ? reachWmo : reachTerrain;
    }
    if (reachWmo) return reachWmo;
    if (reachTerrain) return reachTerrain;
    return std::nullopt;
}

std::optional<float> selectHighestFloor(const std::optional<float>& a,
                                        const std::optional<float>& b,
                                        const std::optional<float>& c) {
    std::optional<float> best;
    auto consider = [&](const std::optional<float>& h) {
        if (!h) return;
        if (!best || *h > *best) best = *h;
    };
    consider(a);
    consider(b);
    consider(c);
    return best;
}

std::optional<float> selectClosestFloor(const std::optional<float>& a,
                                        const std::optional<float>& b,
                                        float refZ) {
    if (a && b) {
        float da = std::abs(*a - refZ);
        float db = std::abs(*b - refZ);
        return (da <= db) ? a : b;
    }
    if (a) return a;
    if (b) return b;
    return std::nullopt;
}

std::optional<float> selectReachableFloor3(const std::optional<float>& a,
                                           const std::optional<float>& b,
                                           const std::optional<float>& c,
                                           float refZ,
                                           float maxStepUp) {
    std::optional<float> best;
    auto consider = [&](const std::optional<float>& h) {
        if (!h) return;
        if (*h > refZ + maxStepUp) return;
        if (!best || *h > *best) best = *h;
    };
    consider(a);
    consider(b);
    consider(c);
    return best;
}

} // namespace

CameraController::CameraController(Camera* cam) : camera(cam) {
    yaw = defaultYaw;
    facingYaw = defaultYaw;
    pitch = defaultPitch;
    reset();
}

void CameraController::startIntroPan(float durationSec, float orbitDegrees) {
    if (!camera) return;
    introActive = true;
    introTimer = 0.0f;
    idleTimer_ = 0.0f;
    introDuration = std::max(0.5f, durationSec);
    introStartYaw = yaw;
    introEndYaw = yaw - orbitDegrees;
    introOrbitDegrees = orbitDegrees;
    introStartPitch = pitch;
    introEndPitch = pitch;
    introStartDistance = currentDistance;
    introEndDistance = currentDistance;
    thirdPerson = true;
}

std::optional<float> CameraController::getCachedFloorHeight(float x, float y, float z) {
    // Check cache validity (position within threshold and frame count)
    glm::vec2 queryPos(x, y);
    glm::vec2 cachedPos(lastFloorQueryPos.x, lastFloorQueryPos.y);
    glm::vec2 dq = queryPos - cachedPos;
    float distSq = glm::dot(dq, dq);
    constexpr float kFloorThresholdSq = FLOOR_QUERY_DISTANCE_THRESHOLD * FLOOR_QUERY_DISTANCE_THRESHOLD;

    if (distSq < kFloorThresholdSq && floorQueryFrameCounter < FLOOR_QUERY_FRAME_INTERVAL) {
        floorQueryFrameCounter++;
        return cachedFloorHeight;
    }

    // Cache miss - query and update
    floorQueryFrameCounter = 0;
    lastFloorQueryPos = glm::vec3(x, y, z);

    std::optional<float> result;
    if (terrainManager) {
        result = terrainManager->getHeightAt(x, y);
    }
    if (wmoRenderer) {
        auto wh = wmoRenderer->getFloorHeight(x, y, z + 2.0f);
        if (wh && (!result || *wh > *result)) result = wh;
    }
    if (m2Renderer && !externalFollow_) {
        auto mh = m2Renderer->getFloorHeight(x, y, z);
        if (mh && (!result || *mh > *result)) result = mh;
    }

    cachedFloorHeight = result;
    return result;
}

float CameraController::raymarchTerrainCameraLimit(const glm::vec3& pivot, const glm::vec3& camDir,
                                                   float maxDist) const {
    if (!terrainManager || maxDist <= MIN_DISTANCE) return maxDist;

    // Clearance the camera keeps above the terrain surface along the whole ray.
    constexpr float kClearance = CAM_SPHERE_RADIUS + 0.25f;

    // If the pivot itself sits below the heightfield (caves, WMO basements,
    // terrain holes under cities), the heightfield is not the relevant occluder
    // here — skip limiting rather than pinning the camera to first-person.
    auto pivotH = terrainManager->getHeightAt(pivot.x, pivot.y);
    if (pivotH && pivot.z < *pivotH - 0.2f) return maxDist;

    // Coarse march in ~1.25-unit steps (capped so worst case stays cheap),
    // then bisect between the last clear sample and the first blocked one.
    const int steps = std::clamp(static_cast<int>(maxDist / 1.25f) + 1, 4, 24);
    const float stepLen = maxDist / static_cast<float>(steps);
    float prevT = 0.0f;
    for (int i = 1; i <= steps; ++i) {
        float t = stepLen * static_cast<float>(i);
        glm::vec3 p = pivot + camDir * t;
        auto h = terrainManager->getHeightAt(p.x, p.y);
        if (h && p.z < *h + kClearance) {
            float lo = prevT;
            float hi = t;
            for (int j = 0; j < 4; ++j) {
                float mid = 0.5f * (lo + hi);
                glm::vec3 mp = pivot + camDir * mid;
                auto mh = terrainManager->getHeightAt(mp.x, mp.y);
                if (mh && mp.z < *mh + kClearance) hi = mid; else lo = mid;
            }
            return std::max(MIN_DISTANCE, lo);
        }
        prevT = t;
    }
    return maxDist;
}

void CameraController::triggerShake(float magnitude, float frequency, float duration) {
    // Allow stronger shake to override weaker; don't allow zero magnitude.
    if (magnitude <= 0.0f || duration <= 0.0f) return;
    if (magnitude > shakeMagnitude_ || shakeElapsed_ >= shakeDuration_) {
        shakeMagnitude_ = magnitude;
        shakeFrequency_ = frequency;
        shakeDuration_  = duration;
        shakeElapsed_   = 0.0f;
    }
}

void CameraController::update(float deltaTime) {
    if (!enabled || !camera) {
        return;
    }
    // Keep physics integration stable during render hitches to avoid floor tunneling.
    const float physicsDeltaTime = std::min(deltaTime, kMaxPhysicsDelta);
    intoxicationTime_ += deltaTime;

    // During taxi flights, skip movement logic but keep camera orbit/zoom controls.
    if (externalFollow_) {
        // Cancel any active intro/idle orbit so mouse panning works during taxi.
        // The intro handling code (below) is unreachable during externalFollow_.
        introActive = false;
        idleOrbit_ = false;
        idleTimer_ = 0.0f;

        camera->setRotation(yaw, pitch);
        float zoomLerp = 1.0f - std::exp(-ZOOM_SMOOTH_SPEED * deltaTime);
        currentDistance += (userTargetDistance - currentDistance) * zoomLerp;
        collisionDistance = currentDistance;

        // Position camera behind character during taxi
        if (thirdPerson && followTarget) {
            glm::vec3 targetPos = *followTarget;
            glm::vec3 forward3D = camera->getForward();

            // Pivot point at upper chest/neck
            float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
            glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, pivotHeight_ + mountedOffset);

            // Camera direction from yaw/pitch
            glm::vec3 camDir = -forward3D;

            // Use current distance
            float actualDist = std::min(currentDistance, collisionDistance);

            // Compute camera position
            glm::vec3 actualCam;
            // Small offset prevents the camera from clipping into the character
            // model when collision pushes it to near-minimum distance.
            constexpr float kCameraClipEpsilon = 0.1f;
            if (actualDist < MIN_DISTANCE + kCameraClipEpsilon) {
                actualCam = pivot + forward3D * kCameraClipEpsilon;
            } else {
                actualCam = pivot + camDir * actualDist;
            }

            // Smooth camera position (1:1 while actively dragging, see main update()).
            if (glm::dot(smoothedCamPos, smoothedCamPos) < 1e-4f) {
                smoothedCamPos = actualCam;
            }
            float camLerp = (mouseButtonDown && !smoothCameraFollow_)
                ? 1.0f : (1.0f - std::exp(-camSmoothSpeed_ * deltaTime));
            smoothedCamPos += (actualCam - smoothedCamPos) * camLerp;

            camera->setPosition(smoothedCamPos);
        }

        return;
    }

    auto& input = core::Input::getInstance();

    // Don't process keyboard input when UI text input (e.g. chat box) has focus
    bool uiWantsKeyboard = ImGui::GetIO().WantTextInput;

    // Suppress movement input after teleport/portal (keys may still be held)
    if (movementSuppressTimer_ > 0.0f) {
        movementSuppressTimer_ -= deltaTime;
    }
    bool movementSuppressed = movementSuppressTimer_ > 0.0f;

    // Determine current key states
    bool keyW = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_W);
    bool keyS = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_S);
    bool keyA = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_A);
    bool keyD = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_D);
    bool keyQ = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_Q);
    bool keyE = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_E);
    bool shiftDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LSHIFT) || input.isKeyPressed(SDL_SCANCODE_RSHIFT));
    bool ctrlDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LCTRL) || input.isKeyPressed(SDL_SCANCODE_RCTRL));
    bool nowJump = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyJustPressed(SDL_SCANCODE_SPACE);
    // Swimming needs the held state, not the press edge: on land space is a
    // one-shot jump, but in water it is continuous ascent, and an edge gave a
    // single impulse that then bled away.
    bool swimUpHeld = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_SPACE);
    bool spaceDown = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_SPACE);

    // Idle camera: any input resets the timer; timeout triggers a slow orbit pan
    bool anyInput = leftMouseDown || rightMouseDown || keyW || keyS || keyA || keyD || keyQ || keyE || nowJump;
    if (anyInput) {
        idleTimer_ = 0.0f;
    } else if (!introActive && idleOrbitEnabled_) {
        idleTimer_ += deltaTime;
        if (idleTimer_ >= IDLE_TIMEOUT) {
            idleTimer_ = 0.0f;
            startIntroPan(30.0f, 360.0f); // Slow casual orbit over 30 seconds
            idleOrbit_ = true;
        }
    } else if (!idleOrbitEnabled_) {
        idleTimer_ = 0.0f;
    }

    if (introActive) {
        if (anyInput) {
            introActive = false;
            idleOrbit_ = false;
            idleTimer_ = 0.0f;
        } else {
            introTimer += deltaTime;
            if (idleOrbit_) {
                // Continuous smooth rotation — no lerp endpoint, just constant angular velocity
                float degreesPerSec = introOrbitDegrees / introDuration;
                yaw -= degreesPerSec * deltaTime;
                camera->setRotation(yaw, pitch);
                facingYaw = yaw;
            } else {
                float t = (introDuration > 0.0f) ? std::min(introTimer / introDuration, 1.0f) : 1.0f;
                yaw = introStartYaw + (introEndYaw - introStartYaw) * t;
                pitch = introStartPitch + (introEndPitch - introStartPitch) * t;
                currentDistance = introStartDistance + (introEndDistance - introStartDistance) * t;
                userTargetDistance = introEndDistance;
                camera->setRotation(yaw, pitch);
                facingYaw = yaw;
                if (t >= 1.0f) {
                    introActive = false;
                }
            }
        }
        // Suppress player movement/input during intro.
        keyW = keyS = keyA = keyD = keyQ = keyE = nowJump = swimUpHeld = false;
    }

    // Tilde or NumLock toggles auto-run; any forward/backward key cancels it
    bool tildeDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_GRAVE) ||
                                           input.isKeyPressed(SDL_SCANCODE_NUMLOCKCLEAR));
    if (tildeDown && !tildeWasDown) {
        autoRunning = !autoRunning;
    }
    tildeWasDown = tildeDown;
    // Helper: cancel auto-follow and notify game handler
    auto doCancelAutoFollow = [&]() {
        if (autoFollowTarget_) {
            autoFollowTarget_ = nullptr;
            if (autoFollowCancelCallback_) autoFollowCancelCallback_();
        }
    };

    if (keyW || keyS) {
        autoRunning = false;
        doCancelAutoFollow();
    }

    bool mouseAutorun = !uiWantsKeyboard && !sitting && leftMouseDown && rightMouseDown;
    if (mouseAutorun) {
        autoRunning = false;
        doCancelAutoFollow();
    }

    // Auto-follow: face target and run toward them when within range
    bool autoFollowMove = false;
    if (autoFollowTarget_ && followTarget && !movementRooted_) {
        glm::vec3 myPos = *followTarget;
        glm::vec3 tgtPos = *autoFollowTarget_;
        float dx = tgtPos.x - myPos.x;
        float dy = tgtPos.y - myPos.y;
        float distSq2D = dx * dx + dy * dy;

        if (distSq2D > FOLLOW_MAX_DIST * FOLLOW_MAX_DIST) {
            doCancelAutoFollow();
        } else if (distSq2D > FOLLOW_STOP_DIST * FOLLOW_STOP_DIST) {
            // Face target (render-space yaw: atan2(-dx, -dy) -> degrees)
            float targetYawRad = std::atan2(-dx, -dy);
            float targetYawDeg = targetYawRad * 180.0f / 3.14159265f;
            facingYaw = targetYawDeg;
            yaw = targetYawDeg;
            autoFollowMove = true;
        }
        // else: within stop distance, stay put

        // Cancel on strafe/turn keys
        if (keyA || keyD || keyQ || keyE) {
            doCancelAutoFollow();
            autoFollowMove = false;
        }
    }

    // When the server has rooted the player, suppress all horizontal movement input.
    const bool movBlocked = movementRooted_;
    // Auto-follow uses run speed (same as auto-run), not walk speed
    if (autoFollowMove) autoRunning = true;
    bool nowForward = !movBlocked && (keyW || mouseAutorun || autoRunning);
    bool nowBackward = !movBlocked && keyS;
    bool nowStrafeLeft = false;
    bool nowStrafeRight = false;
    bool nowTurnLeft = false;
    bool nowTurnRight = false;

    // WoW-like third-person keyboard behavior:
    // - RMB held: A/D strafe
    // - RMB released: A/D turn character+camera, Q/E strafe
    // Turning is allowed even while rooted; only positional movement is blocked.
    if (thirdPerson && !rightMouseDown) {
        nowTurnLeft = keyA;
        nowTurnRight = keyD;
        nowStrafeLeft = !movBlocked && keyQ;
        nowStrafeRight = !movBlocked && keyE;
    } else {
        nowStrafeLeft = !movBlocked && (keyA || keyQ);
        nowStrafeRight = !movBlocked && (keyD || keyE);
    }

    // Keyboard turning updates camera yaw (character follows yaw in renderer).
    // Use server turn rate (rad/s) when set; otherwise fall back to WOW_TURN_SPEED (deg/s).
    const float activeTurnSpeedDeg = (turnRateOverride_ > 0.0f && turnRateOverride_ < 20.0f
                                       && !std::isnan(turnRateOverride_))
                                         ? glm::degrees(turnRateOverride_)
                                         : WOW_TURN_SPEED;
    if (nowTurnLeft && !nowTurnRight) {
        yaw += activeTurnSpeedDeg * deltaTime;
    } else if (nowTurnRight && !nowTurnLeft) {
        yaw -= activeTurnSpeedDeg * deltaTime;
    }
    if (nowTurnLeft || nowTurnRight) {
        camera->setRotation(yaw, pitch);
        facingYaw = yaw;
    }

    // Tick down gravity suspension timer (used after world entry to prevent
    // falling through WMO floors before collision is loaded)
    if (gravitySuspendTimer_ > 0.0f) {
        gravitySuspendTimer_ -= deltaTime;
    }

    // Select physics constants based on mode
    float gravity = useWoWSpeed ? WOW_GRAVITY : GRAVITY;
    float jumpVel = useWoWSpeed ? WOW_JUMP_VELOCITY : JUMP_VELOCITY;

    // Suspend gravity after world entry — hold Z position until timer expires
    // OR a floor is detected. This prevents falling through unloaded WMO floors.
    if (gravitySuspendTimer_ > 0.0f) {
        gravity = 0.0f;
        verticalVelocity = 0.0f;
    }

    // Calculate movement speed based on direction and modifiers
    float speed;
    if (useWoWSpeed) {
        // Movement speeds (WoW-like: Ctrl walk, default run, backpedal slower)
        if (nowBackward && !nowForward) {
            speed = (runBackSpeedOverride_ > 0.0f && runBackSpeedOverride_ < 100.0f
                     && !std::isnan(runBackSpeedOverride_))
                        ? runBackSpeedOverride_ : WOW_BACK_SPEED;
        } else if (ctrlDown) {
            speed = (walkSpeedOverride_ > 0.0f && walkSpeedOverride_ < 100.0f && !std::isnan(walkSpeedOverride_))
                        ? walkSpeedOverride_ : WOW_WALK_SPEED;
        } else if (runSpeedOverride_ > 0.0f && runSpeedOverride_ < 100.0f && !std::isnan(runSpeedOverride_)) {
            speed = runSpeedOverride_;
        } else {
            speed = WOW_RUN_SPEED;
        }
    } else {
        // Exploration mode (original behavior)
        speed = movementSpeed;
        if (shiftDown) {
            speed *= sprintMultiplier;
        }
        if (ctrlDown) {
            speed *= slowMultiplier;
        }
    }

    bool hasMoveInput = nowForward || nowBackward || nowStrafeLeft || nowStrafeRight;
    if (useWoWSpeed) {
        // "Sprinting" flag drives run animation/stronger footstep set.
        // In WoW mode this means running pace (not walk/backpedal), not Shift.
        runPace = hasMoveInput && !ctrlDown && !nowBackward;
    } else {
        runPace = hasMoveInput && shiftDown;
    }

    // Get camera axes — project forward onto XY plane for walking
    glm::vec3 forward3D = camera->getForward();
    bool cameraDrivesFacing = rightMouseDown || mouseAutorun;
    // During taxi flights, orientation is controlled by the flight path, not player input
    if (cameraDrivesFacing && !externalFollow_) {
        facingYaw = yaw;
    }
    float moveYaw = cameraDrivesFacing ? yaw : facingYaw;
    if (intoxication_ > 0.34f) {
        // Drunk and smashed characters weave while trying to walk. Keep facing
        // authoritative; only the travel direction stumbles side to side.
        moveYaw += std::sin(intoxicationTime_ * 2.1f) * 11.0f * intoxication_;
    }
    float moveYawRad = glm::radians(moveYaw);
    glm::vec3 forward(std::cos(moveYawRad), std::sin(moveYawRad), 0.0f);
    glm::vec3 right(-std::sin(moveYawRad), std::cos(moveYawRad), 0.0f);

    // Toggle sit/crouch with X key (edge-triggered) — only when UI doesn't want keyboard
    // Blocked while mounted
    bool prevSitting = sitting;
    bool xDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_X);
    // While swimming, X dives down instead of toggling sit
    if (xDown && !xKeyWasDown && !mounted_ && !swimming) {
        sitting = !sitting;
    }
    if (mounted_) sitting = false;
    xKeyWasDown = xDown;

    // Reset camera angles with R key (edge-triggered) — only when UI doesn't want keyboard
    // Does NOT move the player; full reset() is reserved for world-entry/respawn.
    bool rDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_R);
    if (rDown && !rKeyWasDown) {
        resetAngles();
    }
    rKeyWasDown = rDown;

    // Stand up on any movement key or jump while sitting (WoW behaviour)
    if (!uiWantsKeyboard && sitting && !movementSuppressed) {
        bool anyMoveKey =
            input.isKeyPressed(SDL_SCANCODE_W) || input.isKeyPressed(SDL_SCANCODE_S) ||
            input.isKeyPressed(SDL_SCANCODE_A) || input.isKeyPressed(SDL_SCANCODE_D) ||
            input.isKeyPressed(SDL_SCANCODE_Q) || input.isKeyPressed(SDL_SCANCODE_E) ||
            input.isKeyPressed(SDL_SCANCODE_SPACE);
        if (anyMoveKey) sitting = false;
    }

    // Notify server when the player stands up via local input
    if (prevSitting && !sitting && standUpCallback_) {
        standUpCallback_();
    }

    // Notify server when the player sits down via local input
    if (!prevSitting && sitting && sitDownCallback_) {
        sitDownCallback_();
    }

    // Update eye height based on crouch state (smooth transition)
    float targetEyeHeight = sitting ? CROUCH_EYE_HEIGHT : STAND_EYE_HEIGHT;
    float heightLerpSpeed = 10.0f * deltaTime;
    eyeHeight = eyeHeight + (targetEyeHeight - eyeHeight) * std::min(1.0f, heightLerpSpeed);

    // Calculate horizontal movement vector
    glm::vec3 movement(0.0f);

    if (nowForward) movement += forward;
    if (nowBackward) movement -= forward;
    if (nowStrafeLeft) movement += right;
    if (nowStrafeRight) movement -= right;

    if (glm::dot(movement, movement) > 0.0001f) {
        travelYaw_ = glm::degrees(std::atan2(movement.y, movement.x));
    } else {
        travelYaw_ = facingYaw;
    }

    // Third-person orbit camera mode
    if (thirdPerson && followTarget) {
        // Move the follow target (character position) instead of the camera
        glm::vec3 targetPos = *followTarget;
        const glm::vec3 prevTargetPos = *followTarget;
        if (!externalFollow_) {
            if (wmoRenderer) {
                wmoRenderer->setCollisionFocus(targetPos, COLLISION_FOCUS_RADIUS_THIRD_PERSON);
            }
            if (m2Renderer) {
                m2Renderer->setCollisionFocus(targetPos, COLLISION_FOCUS_RADIUS_THIRD_PERSON);
            }
        }

        if (!externalFollow_) {
            // Enter swim only when water is deep enough (waist-deep+),
            // not for shallow wading.
            std::optional<float> waterH;
            if (waterRenderer) {
                waterH = waterRenderer->getWaterHeightAt(targetPos.x, targetPos.y);
            }
            constexpr float MAX_SWIM_DEPTH_FROM_SURFACE = 12.0f;
            // Hysteresis: starting to swim needs deeper water than continuing to
            // swim does. With one threshold, a character wading at the boundary
            // flipped between swim and walk every frame — each flip restarts the
            // locomotion animation and sends a START_SWIM/STOP_SWIM pair, which
            // is what made walking out of water stutter. The two bounds sit
            // either side of the single 1.0 this replaces.
            constexpr float SWIM_ENTER_WATER_DEPTH = 1.15f;
            constexpr float SWIM_EXIT_WATER_DEPTH  = 0.85f;
            const float MIN_SWIM_WATER_DEPTH =
                swimming ? SWIM_EXIT_WATER_DEPTH : SWIM_ENTER_WATER_DEPTH;
            bool inWater = false;
            // Water Walk: treat water surface as ground — player walks on top, not through.
            if (waterWalkActive_ && waterH && targetPos.z >= *waterH - 0.5f) {
                // Clamp to water surface so the player stands on it
                targetPos.z = *waterH;
                verticalVelocity = 0.0f;
                grounded = true;
                inWater = false;
            } else if (waterH && targetPos.z < *waterH) {
                std::optional<uint16_t> waterType;
                if (waterRenderer) {
                    waterType = waterRenderer->getWaterTypeAt(targetPos.x, targetPos.y);
                }
                bool isOcean = false;
                if (waterType && *waterType != 0) {
                    isOcean = (((*waterType - 1) % 4) == 1);
                }
                // Depth gate only applies when ENTERING swim (e.g. don't start
                // swimming in a dry cave beneath a lake's water plane). Once
                // swimming, any depth is fine — you can only get deep by
                // deliberately diving through the water column.
                bool depthAllowed = swimming || isOcean ||
                                    ((*waterH - targetPos.z) <= MAX_SWIM_DEPTH_FROM_SURFACE);
                if (depthAllowed) {
                    std::optional<float> terrainH;
                    std::optional<float> wmoH;
                    std::optional<float> m2H;
                    if (terrainManager) terrainH = terrainManager->getHeightAt(targetPos.x, targetPos.y);
                    if (wmoRenderer) wmoH = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 2.0f);
                    if (m2Renderer) m2H = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 1.0f);
                    auto floorH = selectHighestFloor(terrainH, wmoH, m2H);

                    // Prefer measured depth from floor; if floor sample is missing,
                    // fall back to feet-to-surface depth.
                    float depthFromFeet = (*waterH - targetPos.z);
                    inWater = (floorH && ((*waterH - *floorH) >= MIN_SWIM_WATER_DEPTH)) ||
                              (!floorH && (depthFromFeet >= MIN_SWIM_WATER_DEPTH));

                    // Ramp exit assist: when swimming forward near the surface toward a
                    // reachable floor (dock/shore ramp), switch to walking sooner.
                    if (swimming && inWater && floorH && nowForward) {
                        float floorDelta = *floorH - targetPos.z;
                        float waterOverFloor = *waterH - *floorH;
                        bool nearSurface = depthFromFeet <= 1.45f;
                        bool reachableRamp = (floorDelta >= -0.30f && floorDelta <= 1.10f);
                        bool shallowRampWater = waterOverFloor <= 1.55f;
                        bool notDiving = forward3D.z > -0.20f && !xDown;
                        if (nearSurface && reachableRamp && shallowRampWater && notDiving) {
                            inWater = false;
                        }
                    }

                    // Forward plank/ramp assist: sample structure floors ahead so water exit
                    // can happen when the ramp is in front of us (not only under our feet).
                    if (swimming && inWater && nowForward && forward3D.z > -0.20f && !xDown) {
                        auto queryFloorAt = [&](float x, float y, float probeZ) -> std::optional<float> {
                            std::optional<float> best;
                            if (terrainManager) {
                                best = terrainManager->getHeightAt(x, y);
                            }
                            if (wmoRenderer) {
                                float nz = 1.0f;
                                auto wh = wmoRenderer->getFloorHeight(x, y, probeZ, &nz);
                                if (wh && nz >= 0.40f && (!best || *wh > *best)) best = wh;
                            }
                            if (m2Renderer && !externalFollow_) {
                                float nz = 1.0f;
                                auto mh = m2Renderer->getFloorHeight(x, y, probeZ, &nz);
                                if (mh && nz >= 0.35f && (!best || *mh > *best)) best = mh;
                            }
                            return best;
                        };

                        glm::vec2 fwd2(forward.x, forward.y);
                        float fwdLenSq = glm::dot(fwd2, fwd2);
                        if (fwdLenSq > 1e-8f) {
                            fwd2 *= glm::inversesqrt(fwdLenSq);
                            std::optional<float> aheadFloor;
                            const float probeZ = targetPos.z + 2.0f;
                            const float dists[] = {0.45f, 0.90f, 1.25f};
                            for (float d : dists) {
                                float sx = targetPos.x + fwd2.x * d;
                                float sy = targetPos.y + fwd2.y * d;
                                auto h = queryFloorAt(sx, sy, probeZ);
                                if (h && (!aheadFloor || *h > *aheadFloor)) aheadFloor = h;
                            }

                            if (aheadFloor) {
                                float floorDelta = *aheadFloor - targetPos.z;
                                float waterOverFloor = *waterH - *aheadFloor;
                                bool nearSurface = depthFromFeet <= 1.65f;
                                bool reachableRamp = (floorDelta >= -0.35f && floorDelta <= 1.25f);
                                bool shallowRampWater = waterOverFloor <= 1.75f;
                                if (nearSurface && reachableRamp && shallowRampWater) {
                                    inWater = false;
                                }
                            }
                        }
                    }
                }
            }
            // Keep swimming through water-data gaps at chunk boundaries.
            if (!inWater && swimming && !waterH) {
                inWater = true;
            }


            if (inWater) {
            swimming = true;
            // Swim movement follows look pitch (forward/back), while strafe stays
            // lateral for stable control.
            float swimSpeed = (swimSpeedOverride_ > 0.0f && swimSpeedOverride_ < 100.0f && !std::isnan(swimSpeedOverride_))
                                  ? swimSpeedOverride_ : speed * SWIM_SPEED_FACTOR;
            float waterSurfaceZ = waterH ? (*waterH - WATER_SURFACE_OFFSET) : targetPos.z;

            // For auto-run/auto-swim: use character facing (immune to camera pan)
            // For manual W key: use camera direction (swim where you look)
            glm::vec3 swimForward;
            if (autoRunning || (leftMouseDown && rightMouseDown)) {
                // Auto-running: use character's horizontal facing direction
                swimForward = forward;
            } else {
                // Manual control: use camera's 3D direction (swim where you look)
                swimForward = glm::normalize(forward3D);
                if (glm::dot(swimForward, swimForward) < 1e-8f) {
                    swimForward = forward;
                }
            }
            // Use character's facing direction for strafe, not camera's right vector
            glm::vec3 swimRight = right;  // Character's right (horizontal facing), not camera's

            float swimBackSpeed = (swimBackSpeedOverride_ > 0.0f && swimBackSpeedOverride_ < 100.0f
                                    && !std::isnan(swimBackSpeedOverride_))
                                       ? swimBackSpeedOverride_ : swimSpeed * 0.5f;

            glm::vec3 swimMove(0.0f);
            if (nowForward) swimMove += swimForward;
            if (nowBackward) swimMove -= swimForward;
            if (nowStrafeLeft) swimMove += swimRight;
            if (nowStrafeRight) swimMove -= swimRight;

            float swimMoveLenSq = glm::dot(swimMove, swimMove);
            if (swimMoveLenSq > 1e-6f) {
                swimMove *= glm::inversesqrt(swimMoveLenSq);
                // Use backward swim speed when moving backwards only (not when combining with strafe)
                float applySpeed = (nowBackward && !nowForward) ? swimBackSpeed : swimSpeed;
                targetPos += swimMove * applySpeed * physicsDeltaTime;
            }

            // Spacebar = swim up, X = swim down (both continuous, not a jump)
            bool diveKey = xDown;
            bool diveIntent = diveKey || (nowForward && (forward3D.z < -0.28f));
            if (swimUpHeld) {
                verticalVelocity = SWIM_BUOYANCY;
            } else if (diveKey) {
                verticalVelocity = -SWIM_BUOYANCY;
            } else {
                // No vertical key held. Retail holds depth here rather than
                // pulling the swimmer back to the surface: a character can idle
                // underwater and drown doing it, which a surface spring would
                // make impossible. The old behaviour yanked you up unless you
                // were actively diving, so a dive only held while a key was
                // down. Bleed off whatever vertical momentum was carried in and
                // then hover.
                verticalVelocity *= std::max(0.0f, 1.0f - SWIM_VERTICAL_DAMPING * physicsDeltaTime);
                if (std::abs(verticalVelocity) < 0.05f) verticalVelocity = 0.0f;

                // Near the surface a body does float up to it, which is what
                // keeps a swimmer's head out of the water instead of drifting
                // just below. Only within a short band, and not while the look
                // direction says the player is diving.
                const float surfaceErr = waterSurfaceZ - targetPos.z;
                if (!diveIntent && surfaceErr > 0.0f && surfaceErr < SWIM_FLOAT_BAND) {
                    verticalVelocity += surfaceErr * 6.0f * physicsDeltaTime;
                    if (surfaceErr < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                        verticalVelocity = 0.0f;
                    }
                }
            }

            targetPos.z += verticalVelocity * physicsDeltaTime;

            // Don't rise above water surface
            if (waterH && targetPos.z > *waterH - WATER_SURFACE_OFFSET) {
                targetPos.z = *waterH - WATER_SURFACE_OFFSET;
                if (verticalVelocity > 0.0f) verticalVelocity = 0.0f;
            }

            // Prevent sinking/clipping through world floor while swimming.
            // Cache floor queries (update every 3 frames or 1 unit movement)
            std::optional<float> floorH;
            float dx2D = targetPos.x - lastFloorQueryPos.x;
            float dy2D = targetPos.y - lastFloorQueryPos.y;
            float dist2DSq = dx2D * dx2D + dy2D * dy2D;
            constexpr float kFloorDistSq = FLOOR_QUERY_DISTANCE_THRESHOLD * FLOOR_QUERY_DISTANCE_THRESHOLD;
            bool updateFloorCache = (floorQueryFrameCounter++ >= FLOOR_QUERY_FRAME_INTERVAL) ||
                                     (dist2DSq > kFloorDistSq);

            if (updateFloorCache) {
                floorQueryFrameCounter = 0;
                lastFloorQueryPos = targetPos;
                constexpr float MAX_SWIM_FLOOR_ABOVE_FEET = 0.25f;
                constexpr float MIN_SWIM_CEILING_ABOVE_FEET = 0.30f;
                constexpr float MAX_SWIM_CEILING_ABOVE_FEET = 1.80f;
                std::optional<float> ceilingH;
                auto considerFloor = [&](const std::optional<float>& h) {
                    if (!h) return;
                    // Swim-floor guard: only accept surfaces at or very slightly above feet.
                    if (*h <= targetPos.z + MAX_SWIM_FLOOR_ABOVE_FEET) {
                        if (!floorH || *h > *floorH) floorH = h;
                    }
                    // Swim-ceiling guard: detect structures just above feet so upward swim
                    // can't clip through docks/platform undersides.
                    float dz = *h - targetPos.z;
                    if (dz >= MIN_SWIM_CEILING_ABOVE_FEET && dz <= MAX_SWIM_CEILING_ABOVE_FEET) {
                        if (!ceilingH || *h < *ceilingH) ceilingH = h;
                    }
                };

                if (terrainManager) {
                    considerFloor(terrainManager->getHeightAt(targetPos.x, targetPos.y));
                }
                if (wmoRenderer) {
                    auto wh = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 2.0f);
                    considerFloor(wh);
                }
                if (m2Renderer && !externalFollow_) {
                    auto mh = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 2.0f);
                    considerFloor(mh);
                }

                if (ceilingH && verticalVelocity > 0.0f) {
                    float ceilingLimit = *ceilingH - 0.35f;
                    if (targetPos.z > ceilingLimit) {
                        targetPos.z = ceilingLimit;
                        verticalVelocity = 0.0f;
                    }
                }

                cachedFloorHeight = floorH;
            } else {
                floorH = cachedFloorHeight;
            }
            if (floorH) {
                float swimFloor = *floorH + 0.5f;
                if (targetPos.z < swimFloor) {
                    targetPos.z = swimFloor;
                    if (verticalVelocity < 0.0f) verticalVelocity = 0.0f;
                }
            }

            // Enforce collision while swimming too (horizontal only), skip when stationary.
            {
                glm::vec3 swimFrom = *followTarget;
                glm::vec3 swimTo = targetPos;
                glm::vec3 swimDelta = swimTo - swimFrom;
                float swimMoveDistSq = glm::dot(swimDelta, swimDelta);
                glm::vec3 stepPos = swimFrom;

                if (swimMoveDistSq > 1e-4f) {
                    float swimMoveDist = std::sqrt(swimMoveDistSq);
                    float swimStepSize = cachedInsideWMO ? 0.20f : 0.35f;
                    int swimSteps = std::max(1, std::min(8, static_cast<int>(std::ceil(swimMoveDist / swimStepSize))));
                    glm::vec3 stepDelta = (swimTo - swimFrom) / static_cast<float>(swimSteps);

                    for (int i = 0; i < swimSteps; i++) {
                        glm::vec3 candidate = stepPos + stepDelta;

                        if (wmoRenderer) {
                            glm::vec3 adjusted;
                            if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted, cachedInsideWMO)) {
                                candidate.x = adjusted.x;
                                candidate.y = adjusted.y;
                                candidate.z = std::max(candidate.z, adjusted.z);
                            }
                        }

                        if (m2Renderer && !externalFollow_) {
                            glm::vec3 adjusted;
                            if (m2Renderer->checkCollision(stepPos, candidate, adjusted)) {
                                candidate.x = adjusted.x;
                                candidate.y = adjusted.y;
                            }
                        }

                        stepPos = candidate;
                    }
                }

                targetPos.x = stepPos.x;
                targetPos.y = stepPos.y;
            }

            grounded = false;
            } else {
            // Exiting water — boost upward to help climb onto shore/stairs.
            if (wasSwimming) {
                // Anchor lastGroundZ to current position so WMO floor probes
                // start from a sensible height instead of stale pre-swim values.
                lastGroundZ = targetPos.z;
                grounded = true;  // Treat as grounded so step-up budget is full
                // Small upward boost to clear stair lip geometry
                if (verticalVelocity < 1.5f) {
                    verticalVelocity = 1.5f;
                }
            }
            swimming = false;

            // Player-controlled flight (flying mount / druid Flight Form):
            // Use 3D pitch-following movement with no gravity or grounding.
            if (flyingActive_) {
                grounded = true;  // suppress fall-damage checks
                verticalVelocity = 0.0f;
                jumpBufferTimer = 0.0f;
                coyoteTimer = 0.0f;

                // Forward/back follows camera 3D direction (same as swim)
                glm::vec3 flyFwd = glm::normalize(forward3D);
                if (glm::dot(flyFwd, flyFwd) < 1e-8f) flyFwd = forward;
                glm::vec3 flyMove(0.0f);
                if (nowForward)     flyMove += flyFwd;
                if (nowBackward)    flyMove -= flyFwd;
                if (nowStrafeLeft)  flyMove += right;
                if (nowStrafeRight) flyMove -= right;
                // Space = ascend, X = descend while airborne (works on foot too,
                // e.g. .gm fly, not just on a flying mount).
                bool flyDescend = !uiWantsKeyboard && xDown;
                if (nowJump)       flyMove.z += 1.0f;
                if (flyDescend)    flyMove.z -= 1.0f;
                float flyMoveLenSq = glm::dot(flyMove, flyMove);
                if (flyMoveLenSq > 1e-6f) {
                    flyMove *= glm::inversesqrt(flyMoveLenSq);
                    float flyFwdSpeed = (flightSpeedOverride_ > 0.0f && flightSpeedOverride_ < 200.0f
                                         && !std::isnan(flightSpeedOverride_))
                                            ? flightSpeedOverride_ : speed;
                    float flyBackSpeed = (flightBackSpeedOverride_ > 0.0f && flightBackSpeedOverride_ < 200.0f
                                          && !std::isnan(flightBackSpeedOverride_))
                                             ? flightBackSpeedOverride_ : flyFwdSpeed * 0.5f;
                    float flySpeed = (nowBackward && !nowForward) ? flyBackSpeed : flyFwdSpeed;
                    targetPos += flyMove * flySpeed * physicsDeltaTime;
                }
                targetPos.z += verticalVelocity * physicsDeltaTime;
                // Skip all ground physics — go straight to collision/WMO sections
            } else {

            float moveLenSq = glm::dot(movement, movement);
            if (moveLenSq > 1e-6f) {
                movement *= glm::inversesqrt(moveLenSq);
                targetPos += movement * speed * physicsDeltaTime;
            }

            // Apply server-driven knockback horizontal velocity (decays over time).
            if (knockbackActive_) {
                targetPos.x += knockbackHorizVel_.x * physicsDeltaTime;
                targetPos.y += knockbackHorizVel_.y * physicsDeltaTime;
                // Exponential drag: reduce each frame so the player decelerates naturally.
                float drag = std::exp(-KNOCKBACK_HORIZ_DRAG * physicsDeltaTime);
                knockbackHorizVel_ *= drag;
                // Once negligible, clear the flag so collision/grounding work normally.
                if (glm::dot(knockbackHorizVel_, knockbackHorizVel_) < 0.0025f) {
                    knockbackActive_ = false;
                    knockbackHorizVel_ = glm::vec2(0.0f);
                }
            }

            // Jump with input buffering and coyote time
            if (nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
            if (grounded) coyoteTimer = COYOTE_TIME;

            bool canJump = (coyoteTimer > 0.0f) && (jumpBufferTimer > 0.0f) && !mounted_;
            if (canJump) {
                verticalVelocity = jumpVel;
                grounded = false;
                jumpBufferTimer = 0.0f;
                coyoteTimer = 0.0f;
            }

            jumpBufferTimer -= physicsDeltaTime;
            coyoteTimer -= physicsDeltaTime;

            // Apply gravity (skip when server has disabled gravity, e.g. Levitate spell)
            if (gravityDisabled_) {
                // Float in place: bleed off any downward velocity, allow upward to decay slowly
                if (verticalVelocity < 0.0f) verticalVelocity = 0.0f;
                else verticalVelocity *= std::max(0.0f, 1.0f - 3.0f * physicsDeltaTime);
            } else {
                verticalVelocity += gravity * physicsDeltaTime;
                // Feather Fall / Slow Fall: cap downward terminal velocity to ~2 m/s
                if (featherFallActive_ && verticalVelocity < -2.0f)
                    verticalVelocity = -2.0f;
            }
            targetPos.z += verticalVelocity * physicsDeltaTime;
            } // end !flyingActive_ ground physics
            } // end !inWater
        } else {
            // External follow (e.g., taxi): trust server position without grounding.
            swimming = false;
            grounded = true;
            verticalVelocity = 0.0f;
        }

        // Refresh inside-WMO state before collision/grounding so we don't use stale
        // terrain-first caches while entering enclosed tunnel/building spaces.
        if (wmoRenderer && !externalFollow_) {
            glm::vec3 insideDelta = targetPos - lastInsideStateCheckPos_;
            float insideDistSq = glm::dot(insideDelta, insideDelta);
            if (++insideStateCheckCounter_ >= 2 || insideDistSq > 0.1225f) {
                insideStateCheckCounter_ = 0;
                lastInsideStateCheckPos_ = targetPos;

                bool prevInside = cachedInsideWMO;
                bool prevInsideInterior = cachedInsideInteriorWMO;
                cachedInsideWMO = wmoRenderer->isInsideWMO(targetPos.x, targetPos.y, targetPos.z + 1.0f, nullptr);
                cachedInsideInteriorWMO = cachedInsideWMO &&
                    wmoRenderer->isInsideInteriorWMO(targetPos.x, targetPos.y, targetPos.z + 1.0f);
                if (cachedInsideWMO != prevInside || cachedInsideInteriorWMO != prevInsideInterior) {
                    hasCachedFloor_ = false;
                    hasCachedCamFloor = false;
                    cachedPivotLift_ = 0.0f;
                }
            }
        }

        // Sweep collisions in small steps to reduce tunneling through thin walls/floors.
        // Skip entirely when stationary to avoid wasting collision calls.
        // Use tighter steps when inside WMO for more precise collision.
        {
            glm::vec3 startPos = *followTarget;
            glm::vec3 desiredPos = targetPos;
            glm::vec3 moveDelta = desiredPos - startPos;
            float moveDistSq = glm::dot(moveDelta, moveDelta);

            if (moveDistSq > 1e-4f) {
                float moveDist = std::sqrt(moveDistSq);
                // Smaller step size when inside buildings for tighter collision
                float stepSize = cachedInsideWMO ? 0.20f : 0.35f;
                int sweepSteps = std::max(1, std::min(8, static_cast<int>(std::ceil(moveDist / stepSize))));
                glm::vec3 stepPos = startPos;
                glm::vec3 stepDelta = (desiredPos - startPos) / static_cast<float>(sweepSteps);

                for (int i = 0; i < sweepSteps; i++) {
                    glm::vec3 candidate = stepPos + stepDelta;

                    if (wmoRenderer) {
                        glm::vec3 adjusted;
                        if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted, cachedInsideWMO)) {
                            candidate.x = adjusted.x;
                            candidate.y = adjusted.y;
                            // Accept upward Z correction (ramps), reject downward
                            candidate.z = std::max(candidate.z, adjusted.z);
                        }
                    }

                    if (m2Renderer && !externalFollow_) {
                        glm::vec3 adjusted;
                        if (m2Renderer->checkCollision(stepPos, candidate, adjusted)) {
                            candidate.x = adjusted.x;
                            candidate.y = adjusted.y;
                        }
                    }

                    stepPos = candidate;
                }

                targetPos = stepPos;
            }
        }

        // Ground the character to terrain or WMO floor
        // Skip entirely while swimming — the swim floor clamp handles vertical bounds.
        if (!swimming) {
            float stepUpBudget = grounded ? movement::kMaxStepUp : 1.2f;
            // 1. Center-only sample for terrain/WMO floor selection.
            //    Using only the center prevents tunnel entrances from snapping
            //    to terrain when offset samples miss the WMO floor geometry.
            // Slope limit: reject surfaces too steep to walk (prevent clipping).
            // WMO tunnel/bridge ramps are often steeper than outdoor terrain ramps.
            constexpr float MIN_WALKABLE_NORMAL_TERRAIN = movement::kMinWalkableNormalZ;
            constexpr float MIN_WALKABLE_NORMAL_WMO = movement::kMinWalkableNormalZ;
            constexpr float MIN_WALKABLE_NORMAL_M2 = movement::kMinWalkableNormalZ;

            std::optional<float> groundH;
            std::optional<float> centerTerrainH;
            std::optional<float> centerWmoH;
            std::optional<float> centerM2H;
            {
                // Collision cache: skip expensive checks if barely moved (15cm threshold)
                float dmx = targetPos.x - lastCollisionCheckPos_.x;
                float dmy = targetPos.y - lastCollisionCheckPos_.y;
                float distMovedSq = dmx * dmx + dmy * dmy;
                constexpr float kCollisionCacheDistSq = COLLISION_CACHE_DISTANCE * COLLISION_CACHE_DISTANCE;
                bool useCached = grounded && hasCachedFloor_ && distMovedSq < kCollisionCacheDistSq;
                if (useCached) {
                    // Never trust cached ground while actively descending or when
                    // vertical drift from cached floor is meaningful.
                    float dzCached = std::abs(targetPos.z - cachedFloorHeight_);
                    if (verticalVelocity < -0.4f || dzCached > 0.35f) {
                        useCached = false;
                    }
                }

                if (useCached) {
                    groundH = cachedFloorHeight_;
                } else {
                    // Full collision check — run terrain/WMO/M2 queries in parallel
                    std::optional<float> terrainH;
                    std::optional<float> wmoH;
                    std::optional<float> m2H;
                    // When airborne, anchor probe to last ground level so the
                    // ceiling doesn't rise with the jump and catch roof geometry.
                    float wmoBaseZ = grounded ? std::max(targetPos.z, lastGroundZ) : lastGroundZ;
                    float wmoProbeZ = wmoBaseZ + stepUpBudget + 0.5f;
                    float wmoNormalZ = 1.0f;

                    // Launch WMO + M2 floor queries asynchronously while terrain runs on this thread.
                    // Collision scratch buffers are thread_local so concurrent calls are safe.
                    using FloorResult = std::pair<std::optional<float>, float>;
                    std::future<FloorResult> wmoFuture;
                    std::future<FloorResult> m2Future;
                    bool wmoAsync = false, m2Async = false;
                    float px = targetPos.x, py = targetPos.y;
                    if (wmoRenderer) {
                        wmoAsync = true;
                        wmoFuture = core::ThreadPool::frameWorkers().submit(
                            [this, px, py, wmoProbeZ]() -> FloorResult {
                                float nz = 1.0f;
                                auto h = wmoRenderer->getFloorHeight(px, py, wmoProbeZ, &nz);
                                return {h, nz};
                            });
                    }
                    if (m2Renderer && !externalFollow_) {
                        m2Async = true;
                        m2Future = core::ThreadPool::frameWorkers().submit(
                            [this, px, py, wmoProbeZ]() -> FloorResult {
                                float nz = 1.0f;
                                auto h = m2Renderer->getFloorHeight(px, py, wmoProbeZ, &nz);
                                return {h, nz};
                            });
                    }
                    if (terrainManager) {
                        terrainH = terrainManager->getHeightAt(targetPos.x, targetPos.y);
                    }
                    if (wmoAsync) {
                        try { auto [h, nz] = wmoFuture.get(); wmoH = h; wmoNormalZ = nz; }
                        catch (const std::exception& e) { LOG_ERROR("WMO floor query: ", e.what()); }
                    }
                    if (m2Async) {
                        try {
                            auto [h, nz] = m2Future.get();
                            m2H = h;
                            if (m2H && nz < MIN_WALKABLE_NORMAL_M2) {
                                m2H = std::nullopt;
                            }
                        } catch (const std::exception& e) { LOG_ERROR("M2 floor query: ", e.what()); }
                    }

                    // A tunnel mouth can overlap the outdoor heightfield before the
                    // player's eye point is inside the WMO bounds.  Treat a nearby WMO
                    // floor below that terrain as transition space immediately; waiting
                    // for cachedInsideWMO makes the outdoor terrain win one frame at a
                    // time (climbing through the roof), while rejecting a steep entrance
                    // ramp here makes the player fall through it.
                    bool atTunnelSeam = false;
                    if (terrainH && wmoH) {
                        const float terrainAboveWmo = *terrainH - *wmoH;
                        const float wmoDropFromPlayer = targetPos.z - *wmoH;
                        atTunnelSeam = terrainAboveWmo > 1.2f && terrainAboveWmo < 12.0f &&
                                       wmoDropFromPlayer >= -0.4f && wmoDropFromPlayer < 1.8f;
                    }
                    // A real tunnel mouth burrows into rising ground: the heightfield
                    // just ahead climbs above head height (or stops in a hole cut for
                    // the passage). WMO ramps that merely run beneath flat walkable
                    // streets must not steal the player from the terrain above them —
                    // that pulled players through the ground at the Stormwind gate
                    // ramparts and down ramps into the void. Inside an interior WMO
                    // group (tram entrance buildings) the heightfield under the city
                    // is meaningless, so it gets no veto — otherwise entry becomes
                    // dependent on approach angle.
                    if (atTunnelSeam && !cachedInsideInteriorWMO && terrainManager) {
                        glm::vec3 moveDir = targetPos - lastCollisionCheckPos_;
                        moveDir.z = 0.0f;
                        const float moveLen = glm::length(moveDir);
                        if (moveLen < 1e-3f) {
                            atTunnelSeam = false;  // stationary — nothing to enter
                        } else {
                            const glm::vec3 aheadPos = targetPos + moveDir * (2.5f / moveLen);
                            auto terrainAhead = terrainManager->getHeightAt(aheadPos.x, aheadPos.y);
                            atTunnelSeam = !terrainAhead ||
                                           *terrainAhead > targetPos.z + 2.2f;
                        }
                    }

                    // Reject steep WMO slopes. Tunnel ramps use the more permissive WMO
                    // limit even at the boundary, where isInsideWMO is not reliable yet.
                    float minWalkableWmo = (cachedInsideWMO || atTunnelSeam)
                        ? MIN_WALKABLE_NORMAL_WMO : MIN_WALKABLE_NORMAL_TERRAIN;
                    if (wmoH && wmoNormalZ < minWalkableWmo) {
                        wmoH = std::nullopt;  // Treat as unwalkable
                    }

                    // Reject WMO floors far above last known ground when airborne
                    // (prevents snapping to roof/ceiling surfaces during jumps)
                    if (wmoH && !grounded && *wmoH > lastGroundZ + stepUpBudget + 0.5f) {
                        wmoH = std::nullopt;
                        centerWmoH = std::nullopt;
                    }
                    centerTerrainH = terrainH;
                    centerWmoH = wmoH;
                    centerM2H = m2H;

                    // Guard against extremely bad WMO void ramps, but keep normal tunnel
                    // transitions valid. Only reject when the WMO sample is implausibly far
                    // below terrain and player is not already descending.
                    if (terrainH && wmoH && !cachedInsideWMO) {
                        float terrainMinusWmo = *terrainH - *wmoH;
                        if (terrainMinusWmo > 12.0f && verticalVelocity > -8.0f) {
                            wmoH = std::nullopt;
                            centerWmoH = std::nullopt;
                        }
                    }

                    if ((cachedInsideWMO || atTunnelSeam) && wmoH) {
                        // Transition seam (e.g. tunnel mouths): if terrain is much higher than
                        // nearby WMO walkable floor, prefer the WMO floor so we can enter.
                        // Do not require downward velocity or an already-inside state:
                        // both arrive after a level tunnel entrance has begun choosing
                        // between the two surfaces.
                        bool preferWmoAtSeam = atTunnelSeam;
                        if (preferWmoAtSeam) {
                            groundH = wmoH;
                        } else if (terrainH) {
                            // At tunnel seams where both exist, pick the one closest to current feet Z
                            // to avoid oscillating between top terrain and deep WMO floors.
                            groundH = selectClosestFloor(terrainH, wmoH, targetPos.z);
                        } else {
                            groundH = selectReachableFloor3(terrainH, wmoH, m2H, targetPos.z, stepUpBudget);
                        }
                    } else {
                        groundH = selectReachableFloor3(terrainH, wmoH, m2H, targetPos.z, stepUpBudget);
                    }

                    // Update cache
                    lastCollisionCheckPos_ = targetPos;
                    if (groundH) {
                        cachedFloorHeight_ = *groundH;
                        hasCachedFloor_ = true;
                        // Ground found — cancel gravity suspension (WMO floor loaded)
                        if (gravitySuspendTimer_ > 0.0f) gravitySuspendTimer_ = 0.0f;
                    } else {
                        hasCachedFloor_ = false;
                    }
                }
            }

            // Transition safety: if no reachable floor was selected, choose the higher
            // of terrain/WMO center surfaces when it is still near the player.
            // This avoids dropping into void gaps at terrain<->WMO seams.
            const bool nearWmoSpace = cachedInsideWMO || centerWmoH.has_value();
            bool nearStructureSpace = nearWmoSpace || centerM2H.has_value();
            if (!nearStructureSpace && hasRealGround_) {
                // Plank-gap hint: center probes can miss sparse bridge segments.
                // Probe once around last known ground before allowing a full drop.
                if (wmoRenderer) {
                    auto whHint = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, lastGroundZ + 1.5f);
                    if (whHint && std::abs(*whHint - lastGroundZ) <= 2.0f) nearStructureSpace = true;
                }
                if (!nearStructureSpace && m2Renderer && !externalFollow_) {
                    float nz = 1.0f;
                    auto mhHint = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, lastGroundZ + 1.5f, &nz);
                    if (mhHint && nz >= MIN_WALKABLE_NORMAL_M2 &&
                        std::abs(*mhHint - lastGroundZ) <= 2.0f) nearStructureSpace = true;
                }
            }
            if (!groundH) {
                auto highestCenter = selectHighestFloor(centerTerrainH, centerWmoH, centerM2H);
                if (highestCenter) {
                    float dz = targetPos.z - *highestCenter;
                    // Keep this fallback narrow: only for WMO seam cases, or very short
                    // transient misses while still almost touching the last floor.
                    bool allowFallback = nearStructureSpace || (noGroundTimer_ < 0.10f && dz < 0.6f);
                    if (allowFallback && dz >= -0.5f && dz < 2.0f) {
                        groundH = highestCenter;
                    }
                }
            }

            // Continuity guard only for WMO seam overlap: avoid instantly switching to a
            // much lower floor sample at tunnel mouths (bad WMO ramp chains into void).
            if (groundH && hasRealGround_ && nearWmoSpace && !cachedInsideInteriorWMO) {
                float dropFromLast = lastGroundZ - *groundH;
                if (dropFromLast > 1.5f) {
                    if (centerTerrainH && *centerTerrainH > *groundH + 1.5f) {
                        groundH = centerTerrainH;
                    }
                }
            }

            // Seam stability: while overlapping WMO shells, cap how fast floor height can
            // step downward in a single frame to avoid following bad ramp samples into void.
            if (groundH && nearWmoSpace && !cachedInsideInteriorWMO && lastGroundZ > 1.0f) {
                float maxDropPerFrame = (verticalVelocity < -8.0f) ? 2.0f : 0.60f;
                float minAllowed = lastGroundZ - maxDropPerFrame;
                // Extra seam guard: outside interior groups, avoid accepting floors that
                // are far below nearby terrain. Keeps shark-mouth transitions from
                // following erroneous WMO ramps into void.
                if (centerTerrainH) {
                    // Never let terrain-based seam guard push floor above current feet;
                    // it should only prevent excessive downward drops.
                    float terrainGuard = std::min(*centerTerrainH - 1.0f, targetPos.z - 0.15f);
                    minAllowed = std::max(minAllowed, terrainGuard);
                }
                if (*groundH < minAllowed) {
                    *groundH = minAllowed;
                }
            }

            // Structure continuity guard: if a floor query suddenly jumps far below
            // recent support while near dock/bridge geometry, keep a conservative
            // support height to avoid dropping through sparse collision seams.
            if (groundH && hasRealGround_ && nearStructureSpace && !nowJump) {
                float dropFromLast = lastGroundZ - *groundH;
                if (dropFromLast > 1.0f && verticalVelocity > -6.0f) {
                    *groundH = std::max(*groundH, lastGroundZ - 0.20f);
                }
            }

            // Void recovery: far beneath the terrain heightfield with no structure
            // floor anywhere below usually means a seam heuristic already failed.
            // Never use the heightfield as a rescue target while WMO containment says
            // the player is inside, though: Ironforge's valid interior floor is over
            // 200 units below the mountain terrain, and a transient floor-query miss
            // must not teleport the player onto the mountain above the city.
            if (!groundH && centerTerrainH && !cachedInsideWMO &&
                targetPos.z < *centerTerrainH - 60.0f) {
                LOG_WARNING("Void recovery: player at z=", targetPos.z,
                            " with terrain at ", *centerTerrainH, " and no floor below");
                targetPos.z = *centerTerrainH + 0.5f;
                verticalVelocity = 0.0f;
                groundH = centerTerrainH;
            }

            // WMO-only maps (Deeprun Tram, instances), and deep WMO interiors on
            // terrain maps (Ironforge), must recover to the last structural floor
            // rather than the unrelated outdoor heightfield above them.
            if (!groundH && (!centerTerrainH || cachedInsideWMO) && hasLastGroundedPos_ &&
                noGroundTimer_ > 2.5f && targetPos.z < lastGroundZ - 40.0f) {
                LOG_WARNING("Void recovery (WMO/ no heightfield): player at z=", targetPos.z,
                            " returning to last grounded pos (", lastGroundedPos_.x, ", ",
                            lastGroundedPos_.y, ", ", lastGroundedPos_.z, ")");
                targetPos = lastGroundedPos_ + glm::vec3(0.0f, 0.0f, 0.5f);
                verticalVelocity = 0.0f;
                groundH = lastGroundedPos_.z;
            }

            // 1b. Multi-sample WMO floors when in/near WMO space to avoid
            // falling through narrow board/plank gaps where center ray misses.
            if (wmoRenderer && nearWmoSpace) {
                constexpr float WMO_FOOTPRINT = 0.35f;
                const glm::vec2 wmoOffsets[] = {
                    {0.0f, 0.0f},
                    { WMO_FOOTPRINT, 0.0f}, {-WMO_FOOTPRINT, 0.0f},
                    {0.0f,  WMO_FOOTPRINT}, {0.0f, -WMO_FOOTPRINT}
                };

                float wmoMultiBaseZ = grounded ? std::max(targetPos.z, lastGroundZ) : lastGroundZ;
                float wmoProbeZ = wmoMultiBaseZ + stepUpBudget + 0.6f;
                float minWalkableWmo = cachedInsideWMO ? MIN_WALKABLE_NORMAL_WMO : MIN_WALKABLE_NORMAL_TERRAIN;

                for (const auto& o : wmoOffsets) {
                    float nz = 1.0f;
                    auto wh = wmoRenderer->getFloorHeight(targetPos.x + o.x, targetPos.y + o.y, wmoProbeZ, &nz);
                    if (!wh) continue;
                    if (nz < minWalkableWmo) continue;

                    // Reject roof/ceiling surfaces when airborne
                    if (!grounded && *wh > lastGroundZ + stepUpBudget + 0.5f) continue;

                    // Keep to nearby, walkable steps only.
                    if (*wh > targetPos.z + stepUpBudget) continue;
                    if (*wh < lastGroundZ - 3.5f) continue;

                    if (!groundH || *wh > *groundH) {
                        groundH = wh;
                    }
                }
            }

            // WMO recovery probe: when no floor is found while descending, do a wider
            // footprint sample around the player to catch narrow plank/stair misses.
            if (!groundH && wmoRenderer && hasRealGround_ && verticalVelocity <= 0.0f) {
                constexpr float RESCUE_FOOTPRINT = 0.65f;
                const glm::vec2 rescueOffsets[] = {
                    {0.0f, 0.0f},
                    { RESCUE_FOOTPRINT, 0.0f}, {-RESCUE_FOOTPRINT, 0.0f},
                    {0.0f,  RESCUE_FOOTPRINT}, {0.0f, -RESCUE_FOOTPRINT},
                    { RESCUE_FOOTPRINT,  RESCUE_FOOTPRINT},
                    { RESCUE_FOOTPRINT, -RESCUE_FOOTPRINT},
                    {-RESCUE_FOOTPRINT,  RESCUE_FOOTPRINT},
                    {-RESCUE_FOOTPRINT, -RESCUE_FOOTPRINT}
                };
                float rescueProbeZ = std::max(lastGroundZ, targetPos.z) + stepUpBudget + 1.2f;
                std::optional<float> rescueFloor;
                for (const auto& o : rescueOffsets) {
                    float nz = 1.0f;
                    auto wh = wmoRenderer->getFloorHeight(targetPos.x + o.x, targetPos.y + o.y, rescueProbeZ, &nz);
                    if (!wh) continue;
                    if (nz < MIN_WALKABLE_NORMAL_WMO) continue;
                    if (*wh > lastGroundZ + stepUpBudget + 0.75f) continue;
                    if (*wh < lastGroundZ - 6.0f) continue;
                    if (!rescueFloor || *wh > *rescueFloor) {
                        rescueFloor = wh;
                    }
                }
                if (rescueFloor) {
                    groundH = rescueFloor;
                }
            }

            // M2 recovery probe: Booty Bay-style wooden platforms can be represented
            // as M2 collision where center probes intermittently miss.
            if (!groundH && m2Renderer && !externalFollow_ && hasRealGround_ && verticalVelocity <= 0.0f) {
                constexpr float RESCUE_FOOTPRINT = 0.75f;
                const glm::vec2 rescueOffsets[] = {
                    {0.0f, 0.0f},
                    { RESCUE_FOOTPRINT, 0.0f}, {-RESCUE_FOOTPRINT, 0.0f},
                    {0.0f,  RESCUE_FOOTPRINT}, {0.0f, -RESCUE_FOOTPRINT},
                    { RESCUE_FOOTPRINT,  RESCUE_FOOTPRINT},
                    { RESCUE_FOOTPRINT, -RESCUE_FOOTPRINT},
                    {-RESCUE_FOOTPRINT,  RESCUE_FOOTPRINT},
                    {-RESCUE_FOOTPRINT, -RESCUE_FOOTPRINT}
                };
                float rescueProbeZ = std::max(lastGroundZ, targetPos.z) + stepUpBudget + 1.4f;
                std::optional<float> rescueFloor;
                for (const auto& o : rescueOffsets) {
                    float nz = 1.0f;
                    auto mh = m2Renderer->getFloorHeight(targetPos.x + o.x, targetPos.y + o.y, rescueProbeZ, &nz);
                    if (!mh) continue;
                    if (nz < MIN_WALKABLE_NORMAL_M2) continue;
                    if (*mh > lastGroundZ + stepUpBudget + 0.90f) continue;
                    if (*mh < lastGroundZ - 6.0f) continue;
                    if (!rescueFloor || *mh > *rescueFloor) {
                        rescueFloor = mh;
                    }
                }
                if (rescueFloor) {
                    groundH = rescueFloor;
                }
            }

            // Path recovery probe: sample structure floors along the movement segment
            // (prev -> current) to catch narrow plank gaps missed at endpoints.
            if (!groundH && hasRealGround_ && (wmoRenderer || (m2Renderer && !externalFollow_))) {
                std::optional<float> segmentFloor;
                const float probeZ = std::max(lastGroundZ, targetPos.z) + stepUpBudget + 1.2f;
                const float ts[] = {0.25f, 0.5f, 0.75f};
                for (float t : ts) {
                    float sx = prevTargetPos.x + (targetPos.x - prevTargetPos.x) * t;
                    float sy = prevTargetPos.y + (targetPos.y - prevTargetPos.y) * t;

                    if (wmoRenderer) {
                        float nz = 1.0f;
                        auto wh = wmoRenderer->getFloorHeight(sx, sy, probeZ, &nz);
                        if (wh && nz >= MIN_WALKABLE_NORMAL_WMO &&
                            *wh <= lastGroundZ + stepUpBudget + 0.9f &&
                            *wh >= lastGroundZ - 3.0f) {
                            if (!segmentFloor || *wh > *segmentFloor) segmentFloor = wh;
                        }
                    }
                    if (m2Renderer && !externalFollow_) {
                        float nz = 1.0f;
                        auto mh = m2Renderer->getFloorHeight(sx, sy, probeZ, &nz);
                        if (mh && nz >= MIN_WALKABLE_NORMAL_M2 &&
                            *mh <= lastGroundZ + stepUpBudget + 0.9f &&
                            *mh >= lastGroundZ - 3.0f) {
                            if (!segmentFloor || *mh > *segmentFloor) segmentFloor = mh;
                        }
                    }
                }
                if (segmentFloor) {
                    groundH = segmentFloor;
                }
            }

            // 2. Multi-sample for M2 objects (rugs, planks, bridges, ships) —
            //    these are narrow and need offset probes to detect reliably.
            if (m2Renderer && !externalFollow_) {
                constexpr float FOOTPRINT = 0.6f;
                const glm::vec2 offsets[] = {
                    {0.0f, 0.0f},
                    {FOOTPRINT, 0.0f}, {-FOOTPRINT, 0.0f},
                    {0.0f, FOOTPRINT}, {0.0f, -FOOTPRINT},
                    {FOOTPRINT, FOOTPRINT}, {FOOTPRINT, -FOOTPRINT},
                    {-FOOTPRINT, FOOTPRINT}, {-FOOTPRINT, -FOOTPRINT}
                };
                float m2ProbeZ = std::max(targetPos.z, lastGroundZ) + 6.0f;
                for (const auto& o : offsets) {
                    float m2NormalZ = 1.0f;
                    auto m2H = m2Renderer->getFloorHeight(
                        targetPos.x + o.x, targetPos.y + o.y, m2ProbeZ, &m2NormalZ);

                    // Reject steep M2 slopes
                    if (m2H && m2NormalZ < MIN_WALKABLE_NORMAL_TERRAIN) {
                        continue;  // Skip unwalkable M2 surface
                    }

                    // Prefer M2 floors (ships, platforms) even if slightly lower than terrain
                    // to prevent falling through ship decks to water below
                    if (m2H && *m2H <= targetPos.z + stepUpBudget) {
                        if (!groundH || *m2H > *groundH ||
                            (*m2H >= targetPos.z - 0.5f && *groundH < targetPos.z - 1.0f)) {
                            groundH = m2H;
                        }
                    }
                }
            }

            // Outdoors the heightfield has exactly one surface per column, so feet
            // below it means the player is inside the hill, which is never a valid
            // position. Climbing a slope that rises faster than the step-up budget
            // — a steep hill, or an ordinary one crossed in a long frame — got the
            // terrain rejected as unreachable by selectReachableFloor3. Once inside,
            // every later sample was rejected the same way and the gap only widened
            // as the player fell, so nothing recovered until the void check fired 60
            // yards down. Push back out to the surface instead.
            //
            // Only where there is nothing else the player could be standing in or
            // under: a cave, a tunnel or Ironforge is legitimately beneath the
            // heightfield, and must never be yanked up onto the mountain above it.
            // Standing on something settles it before any of the probing below.
            // Grounding has already resolved a floor for this exact position; if
            // that floor sits under the heightfield and is right at the feet, the
            // player is on a structure beneath the terrain, not buried inside it.
            // That is better evidence than the probe further down, which asks
            // from a fixed height above the feet and can miss what grounding
            // already found.
            //
            // Stairs descending below ground are the case that needs it: a
            // stairwell cut into a keep passes under the heightfield within a step
            // or two, and being hauled back to the surface each time reads as not
            // being able to walk down them at all.
            const bool standingOnStructure =
                groundH && centerTerrainH &&
                *groundH < *centerTerrainH - 0.5f &&
                *groundH >= targetPos.z - 2.0f &&
                *groundH <= targetPos.z + 0.6f;

            if (!swimming && !flyingActive_ && !hoverActive_ && !externalFollow_ &&
                !cachedInsideWMO && !nearStructureSpace && !standingOnStructure &&
                centerTerrainH && verticalVelocity <= 0.0f) {
                const float penetration = *centerTerrainH - targetPos.z;
                // Below the shallow bound is ordinary contact and sampling jitter;
                // above the deep bound is somewhere this heuristic cannot vouch for,
                // which the void recovery above already handles.
                constexpr float kMinPenetration = 0.10f;
                constexpr float kMaxPenetration = 12.0f;
                if (penetration > kMinPenetration && penetration < kMaxPenetration) {
                    // Everywhere the player may legitimately stand below the
                    // heightfield — the Darkshire crypts, the tunnel under the hill
                    // into Booty Bay, any cave — is a structure sitting under the
                    // terrain at this column. Probe for one directly instead of
                    // trusting cachedInsideWMO and centerWmoH: the first lags by
                    // design, and the second is cleared outright for floors more
                    // than 12 yards below terrain while containment still reads
                    // false, which is exactly what descending a tunnel mouth looks
                    // like. Finding anything at all here means hands off — the
                    // player belongs under the terrain, and lifting them out would
                    // put them on the hillside above the entrance they just walked
                    // into.
                    // Probed from the player, not from the terrain surface: the
                    // floor query culls any group whose top sits more than 4 yards
                    // below the probe height, so probing from up at the heightfield
                    // would skip the crypt or the tunnel entirely and report clear
                    // ground — the exact opposite of the truth. From here the
                    // structure the player is standing in is right above them.
                    const float probeZ = targetPos.z + 1.5f;
                    bool structureBelowTerrain = false;
                    if (wmoRenderer &&
                        wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, probeZ)) {
                        structureBelowTerrain = true;
                    }
                    if (!structureBelowTerrain && m2Renderer &&
                        m2Renderer->getFloorHeight(targetPos.x, targetPos.y, probeZ)) {
                        structureBelowTerrain = true;
                    }
                    // A hole-cut chunk is how a cave mouth or a below-ground
                    // entrance is opened in the first place. getHeightAt
                    // interpolates straight across the hole, so the surface this
                    // rescue would push up to is not there at all — and the
                    // structure underneath may still be too far below to have been
                    // found by the probes above.
                    if (!structureBelowTerrain && terrainManager &&
                        terrainManager->chunkHasHoles(targetPos.x, targetPos.y)) {
                        structureBelowTerrain = true;
                    }

                    if (!structureBelowTerrain) {
                        if (!terrainRescueActive_) {
                            terrainRescueActive_ = true;
                            LOG_INFO("Terrain penetration rescue: feet ", penetration,
                                     " below the heightfield at (", targetPos.x, ", ",
                                     targetPos.y, ") with no structure under it");
                        }
                        targetPos.z = *centerTerrainH;
                        verticalVelocity = 0.0f;
                        groundH = centerTerrainH;
                        lastGroundZ = *centerTerrainH;
                    } else {
                        terrainRescueActive_ = false;
                    }
                } else {
                    terrainRescueActive_ = false;
                }
            } else {
                terrainRescueActive_ = false;
            }

            if (groundH) {
                hasRealGround_ = true;
                noGroundTimer_ = 0.0f;
                lastGroundedPos_ = glm::vec3(targetPos.x, targetPos.y, *groundH);
                hasLastGroundedPos_ = true;
                float feetZ = targetPos.z;
                float stepUp = stepUpBudget;
                stepUp += 0.05f;
                float fallCatch = 3.0f;
                float dz = *groundH - feetZ;

                // Only snap when:
                // 1. Near ground (within step-up range above) - handles walking
                // 2. Actually falling from height (was airborne + falling fast)
                //    Scale snap range with fall speed so slow falls don't teleport
                //    while extreme speeds still catch geometry penetration.
                // 3. Was grounded + ground is close (grace for slopes)
                bool nearGround = (dz >= 0.0f && dz <= stepUp);
                float airSnapRange = std::min(fallCatch,
                    std::max(0.5f, std::abs(verticalVelocity) * physicsDeltaTime * 2.0f));
                bool airFalling = (!grounded && verticalVelocity < -5.0f
                                   && dz >= -airSnapRange);
                bool slopeGrace = (grounded && verticalVelocity > -1.0f &&
                                   dz >= -0.25f && dz <= stepUp * 1.5f);

                if (dz >= -fallCatch && (nearGround || airFalling || slopeGrace)) {
                    // HOVER: float at fixed height above ground instead of standing on it
                    static constexpr float HOVER_HEIGHT = 4.0f;  // ~4 yards above ground
                    const float snapH = hoverActive_ ? (*groundH + HOVER_HEIGHT) : *groundH;
                    targetPos.z = snapH;
                    verticalVelocity = 0.0f;
                    grounded = true;
                    lastGroundZ = *groundH;
                } else {
                    grounded = false;
                    lastGroundZ = *groundH;
                }
                } else {
                    hasRealGround_ = false;
                    noGroundTimer_ += physicsDeltaTime;

                    float dropFromLastGround = lastGroundZ - targetPos.z;
                    bool seamSizedGap = dropFromLastGround <= (nearStructureSpace ? 2.5f : 0.35f);
                    if (noGroundTimer_ < NO_GROUND_GRACE && seamSizedGap) {
                        // Near WMO floors, prefer continuity over falling on transient
                        // floor-query misses (stairs/planks/portal seams).
                        float maxSlip = nearStructureSpace ? 1.0f : 0.10f;
                        targetPos.z = std::max(targetPos.z, lastGroundZ - maxSlip);
                        if (nearStructureSpace && verticalVelocity < -2.0f) {
                            verticalVelocity = -2.0f;
                        }
                        grounded = false;
                    } else if (nearStructureSpace && noGroundTimer_ < 1.0f && dropFromLastGround <= 3.0f) {
                        // Extended WMO rescue window: hold close to last valid floor so we
                        // do not tunnel through walkable geometry during short hitches.
                        targetPos.z = std::max(targetPos.z, lastGroundZ - 0.35f);
                        if (verticalVelocity < -1.5f) {
                            verticalVelocity = -1.5f;
                        }
                        grounded = false;
                    } else if (nearStructureSpace && noGroundTimer_ < 1.20f && dropFromLastGround <= 4.0f && !nowJump) {
                        // Extended adhesion for sparse dock/bridge collision: keep us on the
                        // last valid support long enough for adjacent structure probes to hit.
                        targetPos.z = std::max(targetPos.z, lastGroundZ - 0.10f);
                        if (verticalVelocity < -0.5f) verticalVelocity = -0.5f;
                        grounded = true;
                    } else {
                        grounded = false;
                    }
                }
            }

        // Update follow target position
        *followTarget = targetPos;

        // --- Safe position caching + void fall detection ---
        if (grounded && hasRealGround_ && !swimming && verticalVelocity >= 0.0f) {
            // Player is safely on real geometry — save periodically
            continuousFallTime_ = 0.0f;
            autoUnstuckFired_ = false;
            safePosSaveTimer_ += physicsDeltaTime;
            if (safePosSaveTimer_ >= SAFE_POS_SAVE_INTERVAL) {
                safePosSaveTimer_ = 0.0f;
                lastSafePos_ = targetPos;
                hasLastSafe_ = true;
            }
        } else if (!grounded && !swimming && !externalFollow_) {
            // Falling (or standing on nothing past grace period) — accumulate fall time
            continuousFallTime_ += physicsDeltaTime;
            if (continuousFallTime_ >= AUTO_UNSTUCK_FALL_TIME && !autoUnstuckFired_) {
                autoUnstuckFired_ = true;
                if (autoUnstuckCallback_) {
                    autoUnstuckCallback_();
                }
            }
        }

        // ===== WoW-style orbit camera =====
        // Pivot point at upper chest/neck.
        float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
        float pivotLift = 0.0f;
        if (terrainManager && !externalFollow_ && !cachedInsideInteriorWMO) {
            float plx = targetPos.x - lastPivotLiftQueryPos_.x;
            float ply = targetPos.y - lastPivotLiftQueryPos_.y;
            float movedSq = plx * plx + ply * ply;
            constexpr float kPivotLiftPosSq = PIVOT_LIFT_POS_THRESHOLD * PIVOT_LIFT_POS_THRESHOLD;
            float distDelta = std::abs(currentDistance - lastPivotLiftDistance_);
            bool queryLift = (++pivotLiftQueryCounter_ >= PIVOT_LIFT_QUERY_INTERVAL) ||
                             (movedSq >= kPivotLiftPosSq) ||
                             (distDelta >= PIVOT_LIFT_DIST_THRESHOLD);
            if (queryLift) {
                pivotLiftQueryCounter_ = 0;
                lastPivotLiftQueryPos_ = targetPos;
                lastPivotLiftDistance_ = currentDistance;

                // Estimate where camera sits horizontally and ensure enough terrain clearance.
                glm::vec3 probeCam = targetPos + (-forward3D) * currentDistance;
                auto terrainAtCam = terrainManager->getHeightAt(probeCam.x, probeCam.y);
                auto terrainAtPivot = terrainManager->getHeightAt(targetPos.x, targetPos.y);

                float desiredLift = 0.0f;
                if (terrainAtCam) {
                    // Keep pivot high enough so near-hill camera rays don't cut through terrain.
                    constexpr float kMinRayClearance = 2.0f;
                    float basePivotZ = targetPos.z + pivotHeight_ + mountedOffset;
                    float rayClearance = basePivotZ - *terrainAtCam;
                    if (rayClearance < kMinRayClearance) {
                        desiredLift = std::clamp(kMinRayClearance - rayClearance, 0.0f, 1.4f);
                    }
                }
                // If character is already below local terrain sample, avoid lifting aggressively.
                if (terrainAtPivot && targetPos.z < *terrainAtPivot - 0.2f) {
                    desiredLift = 0.0f;
                }
                cachedPivotLift_ = desiredLift;
            }
            pivotLift = cachedPivotLift_;
        } else if (cachedInsideInteriorWMO) {
            // Inside WMO volumes (including tunnel/cave shells): terrain-above samples
            // are not relevant for camera pivoting.
            cachedPivotLift_ = 0.0f;
        }
        glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, pivotHeight_ + mountedOffset + pivotLift);

        // Camera direction from yaw/pitch (already computed as forward3D)
        glm::vec3 camDir = -forward3D;  // Camera looks at pivot, so it's behind

        // Smooth zoom toward user target
        float zoomLerp = 1.0f - std::exp(-ZOOM_SMOOTH_SPEED * deltaTime);
        currentDistance += (userTargetDistance - currentDistance) * zoomLerp;

        // Limit max zoom when inside a WMO with a ceiling (building interior)
        // Throttle: only recheck every 10 frames or when position changes >2 units.
        if (wmoRenderer) {
            glm::vec3 wmoCheckDelta = targetPos - lastInsideWMOCheckPos;
            float distFromLastCheckSq = glm::dot(wmoCheckDelta, wmoCheckDelta);
            if (++insideWMOCheckCounter >= 10 || distFromLastCheckSq > 4.0f) {
                wmoRenderer->updateActiveGroup(targetPos.x, targetPos.y, targetPos.z + 1.0f);
                insideWMOCheckCounter = 0;
                lastInsideWMOCheckPos = targetPos;
            }

            // Smoothly pull camera in when entering WMO interiors
            if (cachedInsideWMO && userTargetDistance > MAX_DISTANCE_INTERIOR) {
                userTargetDistance = MAX_DISTANCE_INTERIOR;
            }
        }

        // ===== Camera collision (WMO raycast + terrain heightfield march) =====
        // Cast a ray from the pivot toward the camera direction to find the
        // nearest WMO wall, and march the terrain heightfield along the same
        // ray so hills between the pivot and camera pull the camera in too.
        // Uses asymmetric smoothing: pull-in is fast (so the camera never
        // visibly clips through geometry) but recovery is slow (so passing a
        // doorway or hill crest doesn't cause a zoom-out snap).
        collisionDistance = currentDistance;

        if ((wmoRenderer || terrainManager) && currentDistance > MIN_DISTANCE) {
            float rawLimit = currentDistance;
            if (wmoRenderer) {
                float rawHitDist = wmoRenderer->raycastBoundingBoxes(pivot, camDir, currentDistance);
                // rawHitDist == currentDistance means no hit (function returns maxDistance on miss)
                if (rawHitDist < currentDistance) {
                    rawLimit = std::max(MIN_DISTANCE, rawHitDist - CAM_SPHERE_RADIUS - CAM_EPSILON);
                }
            }
            // Terrain samples above enclosed tunnels/caves are not real occluders.
            if (!cachedInsideInteriorWMO) {
                rawLimit = std::min(rawLimit,
                    raymarchTerrainCameraLimit(pivot, camDir, currentDistance));
            }

            // Initialise smoothed state on first use.
            if (smoothedCollisionDist_ < 0.0f) {
                smoothedCollisionDist_ = rawLimit;
            }

            // Asymmetric smoothing:
            //   • Pull-in: τ ≈ 60 ms  — react quickly to prevent clipping
            //     (instant while actively rotating, matching the 1:1 drag snap)
            //   • Recover: τ ≈ 400 ms — zoom out slowly after leaving geometry
            bool rotatingNow = mouseButtonDown || nowTurnLeft || nowTurnRight;
            if (rawLimit < smoothedCollisionDist_ && rotatingNow) {
                smoothedCollisionDist_ = rawLimit;
            } else {
                const float tau = (rawLimit < smoothedCollisionDist_) ? 0.06f : 0.40f;
                float alpha = 1.0f - std::exp(-deltaTime / tau);
                smoothedCollisionDist_ += (rawLimit - smoothedCollisionDist_) * alpha;
            }

            collisionDistance = std::min(collisionDistance, smoothedCollisionDist_);
        } else {
            smoothedCollisionDist_ = -1.0f;   // Reset when no collision sources available
        }

        // Camera collision: terrain-only floor clamping
        auto getTerrainFloorAt = [&](float x, float y) -> std::optional<float> {
            if (terrainManager) {
                return terrainManager->getHeightAt(x, y);
            }
            return std::nullopt;
        };

        // Use collision distance (don't exceed user target)
        float actualDist = std::min(currentDistance, collisionDistance);

        // Compute actual camera position
        glm::vec3 actualCam;
        if (actualDist < MIN_DISTANCE + 0.1f) {
            // First-person: position camera at pivot (player's eyes)
            actualCam = pivot + forward3D * 0.1f;  // Slightly forward to not clip head
        } else {
            actualCam = pivot + camDir * actualDist;
        }

        // Smooth camera position to avoid jitter
        if (glm::dot(smoothedCamPos, smoothedCamPos) < 1e-4f) {
            smoothedCamPos = actualCam;  // Initialize
        }
        bool activelyRotating = mouseButtonDown || nowTurnLeft || nowTurnRight;
        float camLerp = (activelyRotating && !smoothCameraFollow_)
            ? 1.0f : (1.0f - std::exp(-camSmoothSpeed_ * deltaTime));
        smoothedCamPos += (actualCam - smoothedCamPos) * camLerp;

        // ===== Final floor clearance check =====
        // Use WMO-aware floor so the camera doesn't pop above tunnels/caves.
        constexpr float MIN_FLOOR_CLEARANCE = 0.35f;
        if (!cachedInsideWMO) {
            std::optional<float> camTerrainH;
            if (!cachedInsideInteriorWMO) {
                camTerrainH = getTerrainFloorAt(smoothedCamPos.x, smoothedCamPos.y);
            }
            std::optional<float> camWmoH;
            if (wmoRenderer) {
                // Skip expensive WMO floor query if camera barely moved
                float cdx = smoothedCamPos.x - lastCamFloorQueryPos.x;
                float cdy = smoothedCamPos.y - lastCamFloorQueryPos.y;
                float camDeltaSq = cdx * cdx + cdy * cdy;
                if (camDeltaSq < 0.09f && hasCachedCamFloor) {
                    camWmoH = cachedCamWmoFloor;
                } else {
                    float camFloorProbeZ = smoothedCamPos.z;
                    if (cachedInsideInteriorWMO) {
                        // Inside tunnels/buildings, probe near player height so roof
                        // triangles above the camera don't get treated as floor.
                        camFloorProbeZ = std::min(smoothedCamPos.z, targetPos.z + 1.0f);
                    }
                    camWmoH = wmoRenderer->getFloorHeight(
                        smoothedCamPos.x, smoothedCamPos.y, camFloorProbeZ);

                    if (cachedInsideInteriorWMO && camWmoH) {
                        // Never let camera floor clamp latch to tunnel ceilings / upper decks.
                        float maxValidIndoorFloor = targetPos.z + 0.9f;
                        if (*camWmoH > maxValidIndoorFloor) {
                            camWmoH = std::nullopt;
                        }
                    }
                    cachedCamWmoFloor = camWmoH;
                    hasCachedCamFloor = true;
                    lastCamFloorQueryPos = smoothedCamPos;
                }
            }
            // When camera/character are inside a WMO, force WMO floor usage for camera
            // clearance to avoid snapping toward terrain above enclosed tunnels/caves.
            std::optional<float> camFloorH;
            if (cachedInsideWMO && camWmoH && camTerrainH) {
                // Transition seam: avoid terrain-above clamp near tunnel entrances.
                float camDropFromPlayer = targetPos.z - *camWmoH;
                if ((*camTerrainH - *camWmoH) > 1.2f &&
                    (*camTerrainH - *camWmoH) < 8.0f &&
                    camDropFromPlayer >= -0.4f &&
                    camDropFromPlayer < 1.8f) {
                    camFloorH = camWmoH;
                } else {
                    camFloorH = selectClosestFloor(camTerrainH, camWmoH, smoothedCamPos.z);
                }
            } else {
                camFloorH = selectReachableFloor(
                    camTerrainH, camWmoH, smoothedCamPos.z, 0.5f);
            }
            if (camFloorH && smoothedCamPos.z < *camFloorH + MIN_FLOOR_CLEARANCE) {
                smoothedCamPos.z = *camFloorH + MIN_FLOOR_CLEARANCE;
            }
        }
        // Never let camera sink below the character's feet plane.
        smoothedCamPos.z = std::max(smoothedCamPos.z, targetPos.z + 0.15f);

        camera->setPosition(smoothedCamPos);

        // Hide player model when in first-person (camera too close)
        // WoW fades between ~1.0m and ~0.5m, hides fully below 0.5m
        // For now, just hide below first-person threshold
        if (characterRenderer && playerInstanceId > 0) {
            // Hide only on first-person *intent* (the user's zoom), not on a
            // collision-squeezed distance. The `actualDist < MIN_DISTANCE + 0.1`
            // term fired whenever geometry pushed the camera close in third
            // person — under Undercity's overhangs, or backing into a wall —
            // but the renderer's visibility hardening forces the player visible
            // again in third person on the very same frame. So the two wrote
            // opposite values every frame, toggling the instance and its weapon
            // attachments on and off (visible in the log as a rapid
            // setInstanceVisible flip) while what actually drew never changed.
            // isFirstPersonView already covers the anti-clip-pushback case the
            // extra term was meant for, since it reads the target distance, not
            // the squeezed one.
            bool shouldHidePlayer = isFirstPersonView();
            characterRenderer->setInstanceVisible(playerInstanceId, !shouldHidePlayer);

            // Note: the Renderer's CharAnimState machine drives player character animations
            // (Run, Walk, Jump, Swim, etc.) — no additional animation driving needed here.
        }
    } else {
        // Free-fly camera mode (original behavior)
        glm::vec3 newPos = camera->getPosition();
        if (wmoRenderer) {
            wmoRenderer->setCollisionFocus(newPos, COLLISION_FOCUS_RADIUS_FREE_FLY);
        }
        if (m2Renderer) {
            m2Renderer->setCollisionFocus(newPos, COLLISION_FOCUS_RADIUS_FREE_FLY);
        }
        float feetZ = newPos.z - eyeHeight;

        // Check for water at feet position
        std::optional<float> waterH;
        if (waterRenderer) {
            waterH = waterRenderer->getWaterHeightAt(newPos.x, newPos.y);
        }
        constexpr float MAX_SWIM_DEPTH_FROM_SURFACE = 12.0f;
        bool inWater = false;
        if (waterH && feetZ < *waterH) {
            std::optional<uint16_t> waterType;
            if (waterRenderer) {
                waterType = waterRenderer->getWaterTypeAt(newPos.x, newPos.y);
            }
            bool isOcean = false;
            if (waterType && *waterType != 0) {
                isOcean = (((*waterType - 1) % 4) == 1);
            }
            bool depthAllowed = isOcean || ((*waterH - feetZ) <= MAX_SWIM_DEPTH_FROM_SURFACE);
            if (!depthAllowed) {
                inWater = false;
            } else {
            std::optional<float> terrainH;
            std::optional<float> wmoH;
            std::optional<float> m2H;
            if (terrainManager) terrainH = terrainManager->getHeightAt(newPos.x, newPos.y);
            if (wmoRenderer) wmoH = wmoRenderer->getFloorHeight(newPos.x, newPos.y, feetZ + 2.0f);
            if (m2Renderer && !externalFollow_) m2H = m2Renderer->getFloorHeight(newPos.x, newPos.y, feetZ + 1.0f);
            auto floorH = selectHighestFloor(terrainH, wmoH, m2H);
            // Hysteresis: starting to swim needs deeper water than continuing to
            // swim does. With one threshold, a character wading at the boundary
            // flipped between swim and walk every frame — each flip restarts the
            // locomotion animation and sends a START_SWIM/STOP_SWIM pair, which
            // is what made walking out of water stutter. The two bounds sit
            // either side of the single 1.0 this replaces.
            constexpr float SWIM_ENTER_WATER_DEPTH = 1.15f;
            constexpr float SWIM_EXIT_WATER_DEPTH  = 0.85f;
            const float MIN_SWIM_WATER_DEPTH =
                swimming ? SWIM_EXIT_WATER_DEPTH : SWIM_ENTER_WATER_DEPTH;
            inWater = (floorH && ((*waterH - *floorH) >= MIN_SWIM_WATER_DEPTH)) || (isOcean && !floorH);
            }
        }


        if (inWater) {
            swimming = true;
            float swimSpeed = (swimSpeedOverride_ > 0.0f && swimSpeedOverride_ < 100.0f && !std::isnan(swimSpeedOverride_))
                                  ? swimSpeedOverride_ : speed * SWIM_SPEED_FACTOR;
            float waterSurfaceCamZ = waterH ? (*waterH - WATER_SURFACE_OFFSET + eyeHeight) : newPos.z;
            bool diveIntent = nowForward && (forward3D.z < -0.28f);

            float movLenSq = glm::dot(movement, movement);
            if (movLenSq > 1e-6f) {
                movement *= glm::inversesqrt(movLenSq);
                newPos += movement * swimSpeed * physicsDeltaTime;
            }

            if (nowJump) {
                verticalVelocity = SWIM_BUOYANCY;
            } else {
                verticalVelocity += SWIM_GRAVITY * physicsDeltaTime;
                if (verticalVelocity < SWIM_SINK_SPEED) {
                    verticalVelocity = SWIM_SINK_SPEED;
                }
                if (!diveIntent) {
                    float surfaceErr = (waterSurfaceCamZ - newPos.z);
                    verticalVelocity += surfaceErr * 7.0f * physicsDeltaTime;
                    verticalVelocity *= std::max(0.0f, 1.0f - 3.2f * physicsDeltaTime);
                    if (std::abs(surfaceErr) < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                        verticalVelocity = 0.0f;
                    }
                }
            }

            newPos.z += verticalVelocity * physicsDeltaTime;

            // Don't rise above water surface (feet at water level)
            if (waterH && (newPos.z - eyeHeight) > *waterH - WATER_SURFACE_OFFSET) {
                newPos.z = *waterH - WATER_SURFACE_OFFSET + eyeHeight;
                if (verticalVelocity > 0.0f) verticalVelocity = 0.0f;
            }

            grounded = false;
        } else {
            swimming = false;

            float movLenSq2 = glm::dot(movement, movement);
            if (movLenSq2 > 1e-6f) {
                movement *= glm::inversesqrt(movLenSq2);
                newPos += movement * speed * physicsDeltaTime;
            }

            // Jump with input buffering and coyote time
            if (nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
            if (grounded) coyoteTimer = COYOTE_TIME;

            if (coyoteTimer > 0.0f && jumpBufferTimer > 0.0f && !mounted_) {
                verticalVelocity = jumpVel;
                grounded = false;
                jumpBufferTimer = 0.0f;
                coyoteTimer = 0.0f;
            }

            jumpBufferTimer -= physicsDeltaTime;
            coyoteTimer -= physicsDeltaTime;

            // Apply gravity
            verticalVelocity += gravity * physicsDeltaTime;
            newPos.z += verticalVelocity * physicsDeltaTime;
        }

        // Wall sweep collision before grounding (skip when stationary).
        if (wmoRenderer) {
            glm::vec3 startFeet = camera->getPosition() - glm::vec3(0, 0, eyeHeight);
            glm::vec3 desiredFeet = newPos - glm::vec3(0, 0, eyeHeight);
            glm::vec3 feetDelta = desiredFeet - startFeet;
            float moveDistSq2 = glm::dot(feetDelta, feetDelta);

            if (moveDistSq2 > 1e-4f) {
                float moveDist = std::sqrt(moveDistSq2);
                float stepSize = cachedInsideWMO ? 0.20f : 0.35f;
                int sweepSteps = std::max(1, std::min(8, static_cast<int>(std::ceil(moveDist / stepSize))));
                glm::vec3 stepPos = startFeet;
                glm::vec3 stepDelta = (desiredFeet - startFeet) / static_cast<float>(sweepSteps);

                for (int i = 0; i < sweepSteps; i++) {
                    glm::vec3 candidate = stepPos + stepDelta;
                    glm::vec3 adjusted;
                    if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted, cachedInsideWMO)) {
                        candidate.x = adjusted.x;
                        candidate.y = adjusted.y;
                        candidate.z = std::max(candidate.z, adjusted.z);
                    }
                    stepPos = candidate;
                }

                newPos = stepPos + glm::vec3(0, 0, eyeHeight);
            }
        }

        // Ground to terrain or WMO floor
        {
            auto sampleGround = [&](float x, float y) -> std::optional<float> {
                std::optional<float> terrainH;
                std::optional<float> wmoH;
                std::optional<float> m2H;
                if (terrainManager) {
                    terrainH = terrainManager->getHeightAt(x, y);
                }
                float feetZ = newPos.z - eyeHeight;
                float wmoProbeZ = std::max(feetZ, lastGroundZ) + 1.5f;
                float m2ProbeZ = std::max(feetZ, lastGroundZ) + 6.0f;
                if (wmoRenderer) {
                    wmoH = wmoRenderer->getFloorHeight(x, y, wmoProbeZ);
                }
                if (m2Renderer && !externalFollow_) {
                    m2H = m2Renderer->getFloorHeight(x, y, m2ProbeZ);
                }
                auto base = selectReachableFloor(terrainH, wmoH, feetZ, 1.0f);
                if (m2H && *m2H <= feetZ + 1.0f && (!base || *m2H > *base)) {
                    base = m2H;
                }
                return base;
            };

            // Single center probe.
            std::optional<float> groundH = sampleGround(newPos.x, newPos.y);

            if (groundH) {
                float feetZ = newPos.z - eyeHeight;
                float stepUp = 1.0f;
                float fallCatch = 3.0f;
                float dz = *groundH - feetZ;

                // Only snap when:
                // 1. Near ground (within step-up range above) - handles walking
                // 2. Actually falling from height (was airborne + falling fast)
                // 3. Was grounded + ground is close (grace for slopes)
                bool nearGround = (dz >= 0.0f && dz <= stepUp);
                bool airFalling = (!grounded && verticalVelocity < -5.0f);
                bool slopeGrace = (grounded && dz >= -1.0f && dz <= stepUp * 2.0f);

                if (dz >= -fallCatch && (nearGround || airFalling || slopeGrace)) {
                    newPos.z = *groundH + eyeHeight;
                    verticalVelocity = 0.0f;
                    grounded = true;
                    lastGroundZ = *groundH;
                    swimming = false;
                } else if (!swimming) {
                    grounded = false;
                    lastGroundZ = *groundH;
                }
            } else if (!swimming) {
                newPos.z = lastGroundZ + eyeHeight;
                verticalVelocity = 0.0f;
                grounded = true;
            }
        }

        camera->setPosition(newPos);
    }

    // --- Edge-detection: send movement opcodes on state transitions ---
    if (movementCallback) {
        // Forward/backward
        if (nowForward && !wasMovingForward) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_FORWARD));
        }
        if (nowBackward && !wasMovingBackward) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_BACKWARD));
        }
        if ((!nowForward && wasMovingForward) || (!nowBackward && wasMovingBackward)) {
            if (!nowForward && !nowBackward) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP));
            }
        }

        // Strafing
        if (nowStrafeLeft && !wasStrafingLeft) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_STRAFE_LEFT));
        }
        if (nowStrafeRight && !wasStrafingRight) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_STRAFE_RIGHT));
        }
        if ((!nowStrafeLeft && wasStrafingLeft) || (!nowStrafeRight && wasStrafingRight)) {
            if (!nowStrafeLeft && !nowStrafeRight) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_STRAFE));
            }
        }

        // Turning
        if (nowTurnLeft && !wasTurningLeft) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_TURN_LEFT));
        }
        if (nowTurnRight && !wasTurningRight) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_TURN_RIGHT));
        }
        if ((!nowTurnLeft && wasTurningLeft) || (!nowTurnRight && wasTurningRight)) {
            if (!nowTurnLeft && !nowTurnRight) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_TURN));
            }
        }

        // Jump
        if (nowJump && !wasJumping && grounded) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_JUMP));
        }

        // Fall landing
        if (wasFalling && grounded) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_FALL_LAND));
        }
    }

    // Swimming state transitions
    if (movementCallback) {
        if (swimming && !wasSwimming) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_SWIM));
        } else if (!swimming && wasSwimming) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_SWIM));
        }
    }

    // Flight ascend/descend transitions (Space = ascend, X = descend while mounted+flying)
    if (movementCallback && !externalFollow_) {
        const bool nowAscending = flyingActive_ && spaceDown;
        const bool nowDescending = flyingActive_ && xDown && mounted_;

        if (flyingActive_) {
            if (nowAscending && !wasAscending_) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_ASCEND));
            } else if (!nowAscending && wasAscending_) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_ASCEND));
            }
            if (nowDescending && !wasDescending_) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_START_DESCEND));
            } else if (!nowDescending && wasDescending_) {
                // No separate STOP_DESCEND opcode; STOP_ASCEND ends all vertical movement
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_ASCEND));
            }
        } else {
            // Left flight mode: clear any lingering vertical movement states
            if (wasAscending_) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_ASCEND));
            } else if (wasDescending_) {
                movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_STOP_ASCEND));
            }
        }
        wasAscending_ = nowAscending;
        wasDescending_ = nowDescending;
    }

    // Update previous-frame state
    wasSwimming = swimming;
    wasMovingForward = nowForward;
    wasMovingBackward = nowBackward;
    wasStrafingLeft = nowStrafeLeft;
    wasStrafingRight = nowStrafeRight;
    moveForwardActive = nowForward;
    moveBackwardActive = nowBackward;
    strafeLeftActive = nowStrafeLeft;
    strafeRightActive = nowStrafeRight;
    turningLeftActive = nowTurnLeft;
    turningRightActive = nowTurnRight;
    wasTurningLeft = nowTurnLeft;
    wasTurningRight = nowTurnRight;
    wasJumping = nowJump;
    wasFalling = !grounded && verticalVelocity <= 0.0f;

    // R key is now handled above with chat safeguard (WantTextInput check)

    // Camera shake (SMSG_CAMERA_SHAKE): apply sinusoidal offset to final camera position.
    if (shakeElapsed_ < shakeDuration_) {
        shakeElapsed_ += deltaTime;
        float t = shakeElapsed_ / shakeDuration_;
        // Envelope: fade out over the last 30% of shake duration
        float envelope = (t < 0.7f) ? 1.0f : (1.0f - (t - 0.7f) / 0.3f);
        float theta = shakeElapsed_ * shakeFrequency_ * 2.0f * 3.14159265f;
        glm::vec3 offset(
            shakeMagnitude_ * envelope * std::sin(theta),
            shakeMagnitude_ * envelope * std::cos(theta * 1.3f),
            shakeMagnitude_ * envelope * std::sin(theta * 0.7f) * 0.5f
        );
        if (camera) {
            camera->setPosition(camera->getPosition() + offset);
        }
    }

    if (intoxication_ > 0.0f && camera) {
        const float swayYaw = std::sin(intoxicationTime_ * 1.35f) * 2.5f * intoxication_;
        const float swayPitch = std::sin(intoxicationTime_ * 1.8f + 0.7f) * 1.6f * intoxication_;
        camera->setRotation(yaw + swayYaw,
                            glm::clamp(pitch + swayPitch, MIN_PITCH, MAX_PITCH));
    }
}

void CameraController::processMouseMotion(const SDL_MouseMotionEvent& event) {
    if (!enabled || !camera) {
        return;
    }
    if (introActive) {
        return;
    }

    if (!mouseButtonDown) {
        return;
    }

    // Hold rotation until the drag clears a small dead-zone, so a select-click with
    // slight jitter doesn't rotate the view (which makes NPCs seem to move away).
    if (!rotateArmed_) {
        dragPixelsSincePress_ += std::abs(static_cast<float>(event.xrel)) +
                                 std::abs(static_cast<float>(event.yrel));
        if (dragPixelsSincePress_ < kRotateDeadzonePixels) {
            return;
        }
        rotateArmed_ = true;  // past the dead-zone: this is a deliberate drag
    }

    // Directly update stored yaw/pitch (no lossy forward-vector derivation)
    yaw -= event.xrel * mouseSensitivity;
    // SDL yrel > 0 = mouse moved DOWN. In WoW, mouse-down = look down = pitch decreases.
    // invertMouse flips to flight-sim style (mouse-down = look up).
    float invert = invertMouse ? 1.0f : -1.0f;
    pitch += event.yrel * mouseSensitivity * invert;

    // WoW-style pitch limits: can look almost straight down, limited upward
    pitch = glm::clamp(pitch, MIN_PITCH, MAX_PITCH);

    camera->setRotation(yaw, pitch);
}

void CameraController::processMouseButton(const SDL_MouseButtonEvent& event) {
    if (!enabled) {
        return;
    }

    // Don't capture mouse when ImGui wants it (hovering UI windows)
    bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;

    if (event.button == SDL_BUTTON_LEFT) {
        leftMouseDown = (event.state == SDL_PRESSED) && !uiWantsMouse;
        if (event.state == SDL_PRESSED && event.clicks >= 2) {
            autoRunning = false;
        }
    }
    if (event.button == SDL_BUTTON_RIGHT) {
        rightMouseDown = (event.state == SDL_PRESSED) && !uiWantsMouse;
    }

    bool anyDown = leftMouseDown || rightMouseDown;
    if (anyDown && !mouseButtonDown) {
        SDL_SetRelativeMouseMode(SDL_TRUE);
        // Arm the rotation dead-zone fresh for this press.
        rotateArmed_ = false;
        dragPixelsSincePress_ = 0.0f;
    } else if (!anyDown && mouseButtonDown) {
        SDL_SetRelativeMouseMode(SDL_FALSE);
        rotateArmed_ = false;
    }
    mouseButtonDown = anyDown;
}

void CameraController::releaseMouseCapture() {
    leftMouseDown = false;
    rightMouseDown = false;
    mouseButtonDown = false;
    rotateArmed_ = false;
    dragPixelsSincePress_ = 0.0f;
    SDL_SetRelativeMouseMode(SDL_FALSE);
    SDL_ShowCursor(SDL_ENABLE);
}

void CameraController::resetAngles() {
    if (!camera) return;
    yaw = defaultYaw;
    facingYaw = defaultYaw;
    pitch = defaultPitch;
    camera->setRotation(yaw, pitch);
}

void CameraController::reset() {
    if (!camera) {
        return;
    }

    yaw = defaultYaw;
    facingYaw = defaultYaw;
    pitch = defaultPitch;
    verticalVelocity = 0.0f;
    grounded = true;
    swimming = false;
    sitting = false;
    autoRunning = false;
    noGroundTimer_ = 0.0f;
    autoUnstuckFired_ = false;

    // Clear edge-state so movement packets can re-start cleanly after respawn.
    wasMovingForward = false;
    wasMovingBackward = false;
    wasStrafingLeft = false;
    wasStrafingRight = false;
    wasTurningLeft = false;
    wasTurningRight = false;
    wasJumping = false;
    wasFalling = false;
    wasSwimming = false;
    moveForwardActive = false;
    moveBackwardActive = false;
    strafeLeftActive = false;
    strafeRightActive = false;
    turningLeftActive = false;
    turningRightActive = false;

    glm::vec3 spawnPos = defaultPosition;

    auto evalFloorAt = [&](float x, float y, float refZ) -> std::optional<float> {
        std::optional<float> terrainH;
        std::optional<float> wmoH;
        std::optional<float> m2H;
        if (terrainManager) {
            terrainH = terrainManager->getHeightAt(x, y);
        }
        // Probe from the highest of terrain, refZ (server position), and defaultPosition.z
        // so we don't miss WMO floors above terrain (e.g. Stormwind city surface).
        float floorProbeZ = std::max(terrainH.value_or(refZ), refZ);
        if (wmoRenderer) {
            wmoH = wmoRenderer->getFloorHeight(x, y, floorProbeZ + 4.0f);
        }
        if (m2Renderer && !externalFollow_) {
            m2H = m2Renderer->getFloorHeight(x, y, floorProbeZ + 4.0f);
        }
        auto h = selectReachableFloor(terrainH, wmoH, refZ, 16.0f);
        if (!h) {
            h = selectHighestFloor(terrainH, wmoH, m2H);
        }
        return h;
    };

    // In online mode, try to snap to a nearby floor but fall back to the server
    // position when no WMO floor is found (e.g. WMO not loaded yet in cities).
    // This prevents spawning under WMO cities like Stormwind.
    if (onlineMode) {
        auto h = evalFloorAt(spawnPos.x, spawnPos.y, spawnPos.z);
        if (h && std::abs(*h - spawnPos.z) < 16.0f) {
            spawnPos.z = *h + 0.05f;
        }
        // else: keep server Z as-is
        lastGroundZ = spawnPos.z - 0.05f;

        camera->setRotation(yaw, pitch);
        glm::vec3 forward3D = camera->getForward();

        if (thirdPerson && followTarget) {
            *followTarget = spawnPos;
            currentDistance = userTargetDistance;
            collisionDistance = currentDistance;
            float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
            glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, pivotHeight_ + mountedOffset);
            glm::vec3 camDir = -forward3D;
            glm::vec3 camPos = pivot + camDir * currentDistance;
            smoothedCamPos = camPos;
            camera->setPosition(camPos);
        } else {
            spawnPos.z += eyeHeight;
            smoothedCamPos = spawnPos;
            camera->setPosition(spawnPos);
        }

        LOG_INFO("Camera reset to server position (online mode)");
        return;
    }

    // Search nearby for a stable, non-steep spawn floor to avoid waterfall/ledge spawns.
    float bestScore = std::numeric_limits<float>::max();
    glm::vec3 bestPos = spawnPos;
    bool foundBest = false;
    constexpr float radiiOffline[] = {0.0f, 6.0f, 12.0f, 18.0f, 24.0f, 32.0f};
    const float* radii = radiiOffline;
    const int radiiCount = 6;
    constexpr int ANGLES = 16;
    constexpr float PI = 3.14159265f;
    for (int ri = 0; ri < radiiCount; ri++) {
        float r = radii[ri];
        int steps = (r <= 0.01f) ? 1 : ANGLES;
        for (int i = 0; i < steps; i++) {
            float a = (2.0f * PI * static_cast<float>(i)) / static_cast<float>(steps);
            float x = defaultPosition.x + r * std::cos(a);
            float y = defaultPosition.y + r * std::sin(a);
            auto h = evalFloorAt(x, y, defaultPosition.z);
            if (!h) continue;

            // Allow large downward snaps, but avoid snapping onto high roofs/odd geometry.
            constexpr float MAX_SPAWN_SNAP_UP = 16.0f;
            if (*h > defaultPosition.z + MAX_SPAWN_SNAP_UP) continue;

            float score = r * 0.02f;
            if (terrainManager) {
                // Penalize steep/unstable spots.
                int slopeSamples = 0;
                float slopeAccum = 0.0f;
                constexpr float off = 2.5f;
                const float dx[4] = {off, -off, 0.0f, 0.0f};
                const float dy[4] = {0.0f, 0.0f, off, -off};
                for (int s = 0; s < 4; s++) {
                    auto hn = terrainManager->getHeightAt(x + dx[s], y + dy[s]);
                    if (!hn) continue;
                    slopeAccum += std::abs(*hn - *h);
                    slopeSamples++;
                }
                if (slopeSamples > 0) {
                    score += (slopeAccum / static_cast<float>(slopeSamples)) * 2.0f;
                }
            }
            if (waterRenderer) {
                auto wh = waterRenderer->getWaterHeightAt(x, y);
                if (wh && *h < *wh - 0.2f) {
                    score += 8.0f;
                }
            }
            if (wmoRenderer) {
                const glm::vec3 from(x, y, *h + 0.20f);
                const bool insideWMO = wmoRenderer->isInsideWMO(x, y, *h + 1.5f, nullptr);

                // Prefer outdoors for default hearth-like spawn points (offline only).
                // In online mode, trust the server position even if inside a WMO.
                if (insideWMO && !onlineMode) {
                    score += 120.0f;
                }

                // Reject points embedded in nearby walls by probing tiny cardinal moves.
                int wallHits = 0;
                constexpr float probeStep = 0.85f;
                const glm::vec3 probes[4] = {
                    glm::vec3(x + probeStep, y, *h + 0.20f),
                    glm::vec3(x - probeStep, y, *h + 0.20f),
                    glm::vec3(x, y + probeStep, *h + 0.20f),
                    glm::vec3(x, y - probeStep, *h + 0.20f),
                };
                for (const auto& to : probes) {
                    glm::vec3 adjusted;
                    if (wmoRenderer->checkWallCollision(from, to, adjusted)) {
                        wallHits++;
                    }
                }
                if (wallHits >= 2) {
                    continue; // Likely wedged in geometry.
                }
                if (wallHits == 1) {
                    score += 30.0f;
                }

                // If the point is inside a WMO, ensure there is an easy escape path.
                // If almost all directions are blocked, treat it as invalid spawn.
                if (insideWMO) {
                    int blocked = 0;
                    constexpr int radialChecks = 12;
                    constexpr float radialDist = 2.2f;
                    for (int ri = 0; ri < radialChecks; ri++) {
                        float ang = (2.0f * PI * static_cast<float>(ri)) / static_cast<float>(radialChecks);
                        glm::vec3 to(
                            x + std::cos(ang) * radialDist,
                            y + std::sin(ang) * radialDist,
                            *h + 0.20f
                        );
                        glm::vec3 adjusted;
                        if (wmoRenderer->checkWallCollision(from, to, adjusted)) {
                            blocked++;
                        }
                    }
                    if (blocked >= 9) {
                        continue; // Enclosed by interior/wall geometry.
                    }
                    score += static_cast<float>(blocked) * 3.0f;
                }
            }

            if (score < bestScore) {
                bestScore = score;
                bestPos = glm::vec3(x, y, *h + 0.05f);
                foundBest = true;
            }
        }
    }
    if (foundBest) {
        spawnPos = bestPos;
        lastGroundZ = spawnPos.z - 0.05f;
    }

    camera->setRotation(yaw, pitch);
    glm::vec3 forward3D = camera->getForward();

    if (thirdPerson && followTarget) {
        // In follow mode, respawn the character (feet position), then place camera behind it.
        *followTarget = spawnPos;

        currentDistance = userTargetDistance;
        collisionDistance = currentDistance;

        float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
        glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, pivotHeight_ + mountedOffset);
        glm::vec3 camDir = -forward3D;
        glm::vec3 camPos = pivot + camDir * currentDistance;
        smoothedCamPos = camPos;
        camera->setPosition(camPos);
    } else {
        // Free-fly mode keeps camera eye-height above ground.
        if (foundBest) {
            spawnPos.z += eyeHeight;
        }
        smoothedCamPos = spawnPos;
        camera->setPosition(spawnPos);
    }

    LOG_INFO("Camera reset to default position");
}

void CameraController::teleportTo(const glm::vec3& pos) {
    if (!camera) return;

    verticalVelocity = 0.0f;
    grounded = true;
    swimming = false;
    sitting = false;
    lastGroundZ = pos.z;
    noGroundTimer_ = 0.0f;  // Reset grace period so terrain has time to stream
    autoUnstuckFired_ = false;
    continuousFallTime_ = 0.0f;

    // Invalidate active WMO group so it's re-detected at new position
    if (wmoRenderer) {
        wmoRenderer->updateActiveGroup(pos.x, pos.y, pos.z + 1.0f);
    }

    if (thirdPerson && followTarget) {
        *followTarget = pos;
        camera->setRotation(yaw, pitch);
        glm::vec3 forward3D = camera->getForward();
        float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
        glm::vec3 pivot = pos + glm::vec3(0.0f, 0.0f, pivotHeight_ + mountedOffset);
        glm::vec3 camDir = -forward3D;
        glm::vec3 camPos = pivot + camDir * currentDistance;
        smoothedCamPos = camPos;
        camera->setPosition(camPos);
    } else {
        glm::vec3 camPos = pos + glm::vec3(0.0f, 0.0f, eyeHeight);
        smoothedCamPos = camPos;
        camera->setPosition(camPos);
    }

    LOG_INFO("Teleported to (", pos.x, ", ", pos.y, ", ", pos.z, ")");
}

void CameraController::processMouseWheel(float delta) {
    // Scale zoom speed proportionally to current distance for fine control up close
    float zoomSpeed = glm::max(userTargetDistance * 0.15f, 0.3f);
    userTargetDistance -= delta * zoomSpeed;
    float maxDist = extendedZoom_ ? MAX_DISTANCE_EXTENDED : MAX_DISTANCE_NORMAL;
    if (cachedInsideWMO) maxDist = std::min(maxDist, MAX_DISTANCE_INTERIOR);
    userTargetDistance = glm::clamp(userTargetDistance, MIN_DISTANCE, maxDist);
}

void CameraController::setFollowTarget(glm::vec3* target) {
    followTarget = target;
    if (target) {
        thirdPerson = true;
        LOG_INFO("Third-person camera enabled");
    } else {
        thirdPerson = false;
        LOG_INFO("Free-fly camera enabled");
    }
}

bool CameraController::isMoving() const {
    if (!enabled || !camera) {
        return false;
    }
    if (externalMoving_) return true;
    return moveForwardActive || moveBackwardActive || strafeLeftActive || strafeRightActive || autoRunning;
}

void CameraController::clearMovementInputs() {
    moveForwardActive = false;
    moveBackwardActive = false;
    strafeLeftActive = false;
    strafeRightActive = false;
    turningLeftActive = false;
    turningRightActive = false;
    autoRunning = false;
}

bool CameraController::isSprinting() const {
    return enabled && camera && runPace;
}

void CameraController::triggerMountJump() {
    // Apply physics-driven mount jump: vz = sqrt(2 * g * h)
    // Desired height and gravity are configurable constants
    if (grounded || coyoteTimer > 0.0f) {
        verticalVelocity = getMountJumpVelocity();
        grounded = false;
        coyoteTimer = 0.0f;
    }
}

void CameraController::applyKnockBack(float vcos, float vsin, float hspeed, float vspeed) {
    // The server sends (vcos, vsin) as the 2D direction vector in server/wire
    // coordinate space.  After the server→canonical→render swaps, the direction
    // in render space is simply (vcos, vsin) — the two swaps cancel each other.
    knockbackHorizVel_ = glm::vec2(vcos, vsin) * hspeed;
    knockbackActive_ = true;

    // vspeed in the wire packet is negative when the server wants to launch the
    // player upward (matches TrinityCore: data << float(-speedZ)).  Negate it
    // here to obtain the correct upward initial velocity.
    verticalVelocity = -vspeed;
    grounded = false;
    coyoteTimer = 0.0f;
    jumpBufferTimer = 0.0f;

    // Notify the server that the player left the ground so the FALLING flag is
    // set in subsequent movement heartbeats.  The normal jump detection
    // (nowJump && grounded) does not fire during a server-driven knockback.
    if (movementCallback) {
        movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_JUMP));
    }
}

} // namespace rendering
} // namespace wowee
