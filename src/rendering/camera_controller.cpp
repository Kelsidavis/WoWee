#include "rendering/camera_controller.hpp"
#include <algorithm>
#include <imgui.h>
#include "rendering/terrain_manager.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "game/opcodes.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>
#include <imgui.h>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {

namespace {

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
    float dist = glm::length(queryPos - cachedPos);

    if (dist < FLOOR_QUERY_DISTANCE_THRESHOLD && floorQueryFrameCounter < FLOOR_QUERY_FRAME_INTERVAL) {
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

void CameraController::update(float deltaTime) {
    if (!enabled || !camera) {
        return;
    }
    // Keep physics integration stable during render hitches to avoid floor tunneling.
    const float physicsDeltaTime = std::min(deltaTime, 1.0f / 30.0f);

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
            glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT + mountedOffset);

            // Camera direction from yaw/pitch
            glm::vec3 camDir = -forward3D;

            // Use current distance
            float actualDist = std::min(currentDistance, collisionDistance);

            // Compute camera position
            glm::vec3 actualCam;
            if (actualDist < MIN_DISTANCE + 0.1f) {
                actualCam = pivot + forward3D * 0.1f;
            } else {
                actualCam = pivot + camDir * actualDist;
            }

            // Smooth camera position
            if (glm::length(smoothedCamPos) < 0.01f) {
                smoothedCamPos = actualCam;
            }
            float camLerp = 1.0f - std::exp(-CAM_SMOOTH_SPEED * deltaTime);
            smoothedCamPos += (actualCam - smoothedCamPos) * camLerp;

            camera->setPosition(smoothedCamPos);
        }

        return;
    }

    auto& input = core::Input::getInstance();

    // Don't process keyboard input when UI text input (e.g. chat box) has focus
    bool uiWantsKeyboard = ImGui::GetIO().WantTextInput;

    // Determine current key states
    bool keyW = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_W);
    bool keyS = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_S);
    bool keyA = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_A);
    bool keyD = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_D);
    bool keyQ = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_Q);
    bool keyE = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_E);
    bool shiftDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LSHIFT) || input.isKeyPressed(SDL_SCANCODE_RSHIFT));
    bool ctrlDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LCTRL) || input.isKeyPressed(SDL_SCANCODE_RCTRL));
    bool nowJump = !uiWantsKeyboard && !sitting && input.isKeyJustPressed(SDL_SCANCODE_SPACE);

    // Idle camera: any input resets the timer; timeout triggers a slow orbit pan
    bool anyInput = leftMouseDown || rightMouseDown || keyW || keyS || keyA || keyD || keyQ || keyE || nowJump;
    if (anyInput) {
        idleTimer_ = 0.0f;
    } else if (!introActive) {
        idleTimer_ += deltaTime;
        if (idleTimer_ >= IDLE_TIMEOUT) {
            idleTimer_ = 0.0f;
            startIntroPan(30.0f, 360.0f); // Slow casual orbit over 30 seconds
            idleOrbit_ = true;
        }
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
        keyW = keyS = keyA = keyD = keyQ = keyE = nowJump = false;
    }

    // Tilde toggles auto-run; any forward/backward key cancels it
    bool tildeDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_GRAVE);
    if (tildeDown && !tildeWasDown) {
        autoRunning = !autoRunning;
    }
    tildeWasDown = tildeDown;
    if (keyW || keyS) {
        autoRunning = false;
    }

    bool mouseAutorun = !uiWantsKeyboard && !sitting && leftMouseDown && rightMouseDown;
    if (mouseAutorun) {
        autoRunning = false;
    }
    bool nowForward = keyW || mouseAutorun || autoRunning;
    bool nowBackward = keyS;
    bool nowStrafeLeft = false;
    bool nowStrafeRight = false;
    bool nowTurnLeft = false;
    bool nowTurnRight = false;

    // WoW-like third-person keyboard behavior:
    // - RMB held: A/D strafe
    // - RMB released: A/D turn character+camera, Q/E strafe
    if (thirdPerson && !rightMouseDown) {
        nowTurnLeft = keyA;
        nowTurnRight = keyD;
        nowStrafeLeft = keyQ;
        nowStrafeRight = keyE;
    } else {
        nowStrafeLeft = keyA || keyQ;
        nowStrafeRight = keyD || keyE;
    }

    // Keyboard turning updates camera yaw (character follows yaw in renderer)
    if (nowTurnLeft && !nowTurnRight) {
        yaw += WOW_TURN_SPEED * deltaTime;
    } else if (nowTurnRight && !nowTurnLeft) {
        yaw -= WOW_TURN_SPEED * deltaTime;
    }
    if (nowTurnLeft || nowTurnRight) {
        camera->setRotation(yaw, pitch);
        facingYaw = yaw;
    }

    // Select physics constants based on mode
    float gravity = useWoWSpeed ? WOW_GRAVITY : GRAVITY;
    float jumpVel = useWoWSpeed ? WOW_JUMP_VELOCITY : JUMP_VELOCITY;

    // Calculate movement speed based on direction and modifiers
    float speed;
    if (useWoWSpeed) {
        // Movement speeds (WoW-like: Ctrl walk, default run, backpedal slower)
        if (nowBackward && !nowForward) {
            speed = WOW_BACK_SPEED;
        } else if (ctrlDown) {
            speed = WOW_WALK_SPEED;
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
    float moveYawRad = glm::radians(moveYaw);
    glm::vec3 forward(std::cos(moveYawRad), std::sin(moveYawRad), 0.0f);
    glm::vec3 right(-std::sin(moveYawRad), std::cos(moveYawRad), 0.0f);

    // Toggle sit/crouch with X key (edge-triggered) — only when UI doesn't want keyboard
    // Blocked while mounted
    bool xDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_X);
    if (xDown && !xKeyWasDown && !mounted_) {
        sitting = !sitting;
    }
    if (mounted_) sitting = false;
    xKeyWasDown = xDown;

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

    // Stand up if jumping while crouched
    if (!uiWantsKeyboard && sitting && input.isKeyPressed(SDL_SCANCODE_SPACE)) {
        sitting = false;
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
            constexpr float MIN_SWIM_WATER_DEPTH = 1.0f;
            bool inWater = false;
            if (waterH && targetPos.z < *waterH) {
                std::optional<uint16_t> waterType;
                if (waterRenderer) {
                    waterType = waterRenderer->getWaterTypeAt(targetPos.x, targetPos.y);
                }
                bool isOcean = false;
                if (waterType && *waterType != 0) {
                    isOcean = (((*waterType - 1) % 4) == 1);
                }
                bool depthAllowed = isOcean || ((*waterH - targetPos.z) <= MAX_SWIM_DEPTH_FROM_SURFACE);
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
                        bool notDiving = forward3D.z > -0.20f;
                        if (nearSurface && reachableRamp && shallowRampWater && notDiving) {
                            inWater = false;
                        }
                    }

                    // Forward plank/ramp assist: sample structure floors ahead so water exit
                    // can happen when the ramp is in front of us (not only under our feet).
                    if (swimming && inWater && nowForward && forward3D.z > -0.20f) {
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
                        float fwdLen = glm::length(fwd2);
                        if (fwdLen > 1e-4f) {
                            fwd2 /= fwdLen;
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
            float swimSpeed = speed * SWIM_SPEED_FACTOR;
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
                if (glm::length(swimForward) < 1e-4f) {
                    swimForward = forward;
                }
            }
            // Use character's facing direction for strafe, not camera's right vector
            glm::vec3 swimRight = right;  // Character's right (horizontal facing), not camera's

            glm::vec3 swimMove(0.0f);
            if (nowForward) swimMove += swimForward;
            if (nowBackward) swimMove -= swimForward;
            if (nowStrafeLeft) swimMove -= swimRight;
            if (nowStrafeRight) swimMove += swimRight;

            if (glm::length(swimMove) > 0.001f) {
                swimMove = glm::normalize(swimMove);
                targetPos += swimMove * swimSpeed * physicsDeltaTime;
            }

            // Spacebar = swim up (continuous, not a jump)
            bool diveIntent = nowForward && (forward3D.z < -0.28f);
            if (nowJump) {
                verticalVelocity = SWIM_BUOYANCY;
            } else {
                // Gentle sink when not pressing space
                verticalVelocity += SWIM_GRAVITY * physicsDeltaTime;
                if (verticalVelocity < SWIM_SINK_SPEED) {
                    verticalVelocity = SWIM_SINK_SPEED;
                }
                // Strong surface lock while idle/normal swim so buoyancy keeps
                // you afloat unless you're intentionally diving.
                if (!diveIntent) {
                    float surfaceErr = (waterSurfaceZ - targetPos.z);
                    verticalVelocity += surfaceErr * 7.0f * physicsDeltaTime;
                    verticalVelocity *= std::max(0.0f, 1.0f - 3.2f * physicsDeltaTime);
                    if (std::abs(surfaceErr) < 0.06f && std::abs(verticalVelocity) < 0.35f) {
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
            float dist2D = glm::length(glm::vec2(targetPos.x - lastFloorQueryPos.x,
                                                   targetPos.y - lastFloorQueryPos.y));
            bool updateFloorCache = (floorQueryFrameCounter++ >= FLOOR_QUERY_FRAME_INTERVAL) ||
                                     (dist2D > FLOOR_QUERY_DISTANCE_THRESHOLD);

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
                float swimMoveDist = glm::length(swimTo - swimFrom);
                glm::vec3 stepPos = swimFrom;

                if (swimMoveDist > 0.01f) {
                    int swimSteps = std::max(1, std::min(3, static_cast<int>(std::ceil(swimMoveDist / 0.65f))));
                    glm::vec3 stepDelta = (swimTo - swimFrom) / static_cast<float>(swimSteps);

                    for (int i = 0; i < swimSteps; i++) {
                        glm::vec3 candidate = stepPos + stepDelta;

                        if (wmoRenderer) {
                            glm::vec3 adjusted;
                            if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted)) {
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
            // Exiting water — give a small upward boost to help climb onto shore.
            swimming = false;

            if (glm::length(movement) > 0.001f) {
                movement = glm::normalize(movement);
                targetPos += movement * speed * physicsDeltaTime;
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

            // Apply gravity
            verticalVelocity += gravity * physicsDeltaTime;
            targetPos.z += verticalVelocity * physicsDeltaTime;
            }
        } else {
            // External follow (e.g., taxi): trust server position without grounding.
            swimming = false;
            grounded = true;
            verticalVelocity = 0.0f;
        }

        // Refresh inside-WMO state before collision/grounding so we don't use stale
        // terrain-first caches while entering enclosed tunnel/building spaces.
        if (wmoRenderer && !externalFollow_) {
            const float insideDist = glm::length(targetPos - lastInsideStateCheckPos_);
            if (++insideStateCheckCounter_ >= 2 || insideDist > 0.35f) {
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
            float moveDist = glm::length(desiredPos - startPos);

            if (moveDist > 0.01f) {
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
            float stepUpBudget = grounded ? 1.6f : 1.2f;
            // 1. Center-only sample for terrain/WMO floor selection.
            //    Using only the center prevents tunnel entrances from snapping
            //    to terrain when offset samples miss the WMO floor geometry.
            // Slope limit: reject surfaces too steep to walk (prevent clipping).
            // WMO tunnel/bridge ramps are often steeper than outdoor terrain ramps.
            constexpr float MIN_WALKABLE_NORMAL_TERRAIN = 0.7f; // ~45°
            constexpr float MIN_WALKABLE_NORMAL_WMO = 0.45f;    // allow tunnel ramps
            constexpr float MIN_WALKABLE_NORMAL_M2 = 0.45f;     // allow bridge/deck ramps

            std::optional<float> groundH;
            std::optional<float> centerTerrainH;
            std::optional<float> centerWmoH;
            std::optional<float> centerM2H;
            {
                // Collision cache: skip expensive checks if barely moved (15cm threshold)
                float distMoved = glm::length(glm::vec2(targetPos.x, targetPos.y) -
                                             glm::vec2(lastCollisionCheckPos_.x, lastCollisionCheckPos_.y));
                bool useCached = grounded && hasCachedFloor_ && distMoved < COLLISION_CACHE_DISTANCE;
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
                    // Full collision check
                    std::optional<float> terrainH;
                    std::optional<float> wmoH;
                    std::optional<float> m2H;
                    if (terrainManager) {
                        terrainH = terrainManager->getHeightAt(targetPos.x, targetPos.y);
                    }
                    // When airborne, anchor probe to last ground level so the
                    // ceiling doesn't rise with the jump and catch roof geometry.
                    float wmoBaseZ = grounded ? std::max(targetPos.z, lastGroundZ) : lastGroundZ;
                    float wmoProbeZ = wmoBaseZ + stepUpBudget + 0.5f;
                    float wmoNormalZ = 1.0f;
                    if (wmoRenderer) {
                        wmoH = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, wmoProbeZ, &wmoNormalZ);
                    }
                    if (m2Renderer && !externalFollow_) {
                        float m2NormalZ = 1.0f;
                        m2H = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, wmoProbeZ, &m2NormalZ);
                        if (m2H && m2NormalZ < MIN_WALKABLE_NORMAL_M2) {
                            m2H = std::nullopt;
                        }
                    }

                    // Reject steep WMO slopes
                    float minWalkableWmo = cachedInsideWMO ? MIN_WALKABLE_NORMAL_WMO : MIN_WALKABLE_NORMAL_TERRAIN;
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
                    if (terrainH && wmoH) {
                        float terrainMinusWmo = *terrainH - *wmoH;
                        if (terrainMinusWmo > 12.0f && verticalVelocity > -8.0f) {
                            wmoH = std::nullopt;
                            centerWmoH = std::nullopt;
                        }
                    }

                    if (cachedInsideWMO && wmoH) {
                        // Transition seam (e.g. tunnel mouths): if terrain is much higher than
                        // nearby WMO walkable floor, prefer the WMO floor so we can enter.
                        bool preferWmoAtSeam = false;
                        if (terrainH) {
                            float terrainAboveWmo = *terrainH - *wmoH;
                            float wmoDropFromPlayer = targetPos.z - *wmoH;
                            float playerVsTerrain = targetPos.z - *terrainH;
                            bool descendingIntoTunnel = (verticalVelocity < -1.0f) || (playerVsTerrain < -0.35f);
                            if (terrainAboveWmo > 1.2f && terrainAboveWmo < 8.0f &&
                                wmoDropFromPlayer >= -0.4f && wmoDropFromPlayer < 1.8f &&
                                *wmoH <= targetPos.z + stepUpBudget &&
                                descendingIntoTunnel) {
                                preferWmoAtSeam = true;
                            }
                        }
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

            if (groundH) {
                hasRealGround_ = true;
                noGroundTimer_ = 0.0f;
                float feetZ = targetPos.z;
                float stepUp = stepUpBudget;
                stepUp += 0.05f;
                float fallCatch = 3.0f;
                float dz = *groundH - feetZ;

                // Only snap when:
                // 1. Near ground (within step-up range above) - handles walking
                // 2. Actually falling from height (was airborne + falling fast)
                // 3. Was grounded + ground is close (grace for slopes)
                bool nearGround = (dz >= 0.0f && dz <= stepUp);
                bool airFalling = (!grounded && verticalVelocity < -5.0f);
                bool slopeGrace = (grounded && verticalVelocity > -1.0f &&
                                   dz >= -0.25f && dz <= stepUp * 1.5f);

                if (dz >= -fallCatch && (nearGround || airFalling || slopeGrace)) {
                    targetPos.z = *groundH;
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
            float moved = glm::length(glm::vec2(targetPos.x - lastPivotLiftQueryPos_.x,
                                                targetPos.y - lastPivotLiftQueryPos_.y));
            float distDelta = std::abs(currentDistance - lastPivotLiftDistance_);
            bool queryLift = (++pivotLiftQueryCounter_ >= PIVOT_LIFT_QUERY_INTERVAL) ||
                             (moved >= PIVOT_LIFT_POS_THRESHOLD) ||
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
                    float basePivotZ = targetPos.z + PIVOT_HEIGHT + mountedOffset;
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
        glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT + mountedOffset + pivotLift);

        // Camera direction from yaw/pitch (already computed as forward3D)
        glm::vec3 camDir = -forward3D;  // Camera looks at pivot, so it's behind

        // Smooth zoom toward user target
        float zoomLerp = 1.0f - std::exp(-ZOOM_SMOOTH_SPEED * deltaTime);
        currentDistance += (userTargetDistance - currentDistance) * zoomLerp;

        // Limit max zoom when inside a WMO with a ceiling (building interior)
        // Throttle: only recheck every 10 frames or when position changes >2 units.
        if (wmoRenderer) {
            float distFromLastCheck = glm::length(targetPos - lastInsideWMOCheckPos);
            if (++insideWMOCheckCounter >= 10 || distFromLastCheck > 2.0f) {
                wmoRenderer->updateActiveGroup(targetPos.x, targetPos.y, targetPos.z + 1.0f);
                insideWMOCheckCounter = 0;
                lastInsideWMOCheckPos = targetPos;
            }

            // Do not clamp zoom target by ceiling checks. First-person should always
            // be reachable; occlusion handling below will resolve camera placement safely.
        }

        // ===== Camera collision (sphere sweep approximation) =====
        // Find max safe distance using raycast + sphere radius
        collisionDistance = currentDistance;

        // WMO raycast collision: zoom in when camera would clip through walls/floors
        if (wmoRenderer && currentDistance > MIN_DISTANCE) {
            glm::vec3 camRayOrigin = pivot;
            glm::vec3 camRayDir = camDir;
            float wmoHitDist = wmoRenderer->raycastBoundingBoxes(camRayOrigin, camRayDir, currentDistance);
            if (wmoHitDist < currentDistance) {
                // Hit WMO geometry — pull camera in to avoid clipping
                constexpr float CAM_RADIUS = 0.3f;
                collisionDistance = std::max(MIN_DISTANCE, wmoHitDist - CAM_RADIUS);
            }
        }

        // M2 raycast collision: zoom in when camera would clip through doodads
        if (m2Renderer && !externalFollow_ && currentDistance > MIN_DISTANCE) {
            glm::vec3 camRayOrigin = pivot;
            glm::vec3 camRayDir = camDir;
            float m2HitDist = m2Renderer->raycastBoundingBoxes(camRayOrigin, camRayDir, currentDistance);
            if (m2HitDist < collisionDistance) {
                constexpr float CAM_RADIUS = 0.3f;
                collisionDistance = std::max(MIN_DISTANCE, m2HitDist - CAM_RADIUS);
            }
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
        if (glm::length(smoothedCamPos) < 0.01f) {
            smoothedCamPos = actualCam;  // Initialize
        }
        float camLerp = 1.0f - std::exp(-CAM_SMOOTH_SPEED * deltaTime);
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
                float camDelta = glm::length(glm::vec2(smoothedCamPos.x - lastCamFloorQueryPos.x,
                                                         smoothedCamPos.y - lastCamFloorQueryPos.y));
                if (camDelta < 0.3f && hasCachedCamFloor) {
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
            // Honor first-person intent even if anti-clipping pushes camera back slightly.
            bool shouldHidePlayer = isFirstPersonView() || (actualDist < MIN_DISTANCE + 0.1f);
            characterRenderer->setInstanceVisible(playerInstanceId, !shouldHidePlayer);
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
            constexpr float MIN_SWIM_WATER_DEPTH = 1.0f;
            inWater = (floorH && ((*waterH - *floorH) >= MIN_SWIM_WATER_DEPTH)) || (isOcean && !floorH);
            }
        }


        if (inWater) {
            swimming = true;
            float swimSpeed = speed * SWIM_SPEED_FACTOR;
            float waterSurfaceCamZ = waterH ? (*waterH - WATER_SURFACE_OFFSET + eyeHeight) : newPos.z;
            bool diveIntent = nowForward && (forward3D.z < -0.28f);

            if (glm::length(movement) > 0.001f) {
                movement = glm::normalize(movement);
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

            if (glm::length(movement) > 0.001f) {
                movement = glm::normalize(movement);
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
            float moveDist = glm::length(desiredFeet - startFeet);

            if (moveDist > 0.01f) {
                int sweepSteps = std::max(1, std::min(3, static_cast<int>(std::ceil(moveDist / 0.65f))));
                glm::vec3 stepPos = startFeet;
                glm::vec3 stepDelta = (desiredFeet - startFeet) / static_cast<float>(sweepSteps);

                for (int i = 0; i < sweepSteps; i++) {
                    glm::vec3 candidate = stepPos + stepDelta;
                    glm::vec3 adjusted;
                    if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted)) {
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
    wasTurningLeft = nowTurnLeft;
    wasTurningRight = nowTurnRight;
    wasJumping = nowJump;
    wasFalling = !grounded && verticalVelocity <= 0.0f;

    // R key disabled — was camera reset, conflicts with chat reply
    rKeyWasDown = false;
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

    // Directly update stored yaw/pitch (no lossy forward-vector derivation)
    yaw -= event.xrel * mouseSensitivity;
    float invert = invertMouse ? -1.0f : 1.0f;
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
    } else if (!anyDown && mouseButtonDown) {
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }
    mouseButtonDown = anyDown;
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

    // Search nearby for a stable, non-steep spawn floor to avoid waterfall/ledge spawns.
    // In online mode, use a tight search radius since the server dictates position.
    float bestScore = std::numeric_limits<float>::max();
    glm::vec3 bestPos = spawnPos;
    bool foundBest = false;
    constexpr float radiiOffline[] = {0.0f, 6.0f, 12.0f, 18.0f, 24.0f, 32.0f};
    constexpr float radiiOnline[] = {0.0f, 2.0f};
    const float* radii = onlineMode ? radiiOnline : radiiOffline;
    const int radiiCount = onlineMode ? 2 : 6;
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
        glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT + mountedOffset);
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
        glm::vec3 pivot = pos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT + mountedOffset);
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

} // namespace rendering
} // namespace wowee
