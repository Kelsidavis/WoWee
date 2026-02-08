#include "rendering/camera_controller.hpp"
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
        // Both available: prefer the one closest to the player's feet.
        // This prevents tunnels/caves from snapping the player up to the
        // terrain surface above, while still working on top of buildings.
        float distTerrain = std::abs(*reachTerrain - refZ);
        float distWmo = std::abs(*reachWmo - refZ);
        return (distWmo <= distTerrain) ? reachWmo : reachTerrain;
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

void CameraController::update(float deltaTime) {
    if (!enabled || !camera) {
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
    if (cameraDrivesFacing) {
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
        if (wmoRenderer) {
            wmoRenderer->setCollisionFocus(targetPos, COLLISION_FOCUS_RADIUS_THIRD_PERSON);
        }
        if (m2Renderer) {
            m2Renderer->setCollisionFocus(targetPos, COLLISION_FOCUS_RADIUS_THIRD_PERSON);
        }

        // Check for water at current position — simple submersion test.
        // If the player's feet are meaningfully below the water surface, swim.
        std::optional<float> waterH;
        if (waterRenderer) {
            waterH = waterRenderer->getWaterHeightAt(targetPos.x, targetPos.y);
        }
        bool inWater = waterH && (targetPos.z < (*waterH - 0.3f));
        // Keep swimming through water-data gaps (chunk boundaries).
        if (!inWater && swimming && !waterH) {
            inWater = true;
        }


        if (inWater) {
            swimming = true;
            // Swim movement follows look pitch (forward/back), while strafe stays
            // lateral for stable control.
            float swimSpeed = speed * SWIM_SPEED_FACTOR;
            float waterSurfaceZ = waterH ? (*waterH - WATER_SURFACE_OFFSET) : targetPos.z;

            glm::vec3 swimForward = glm::normalize(forward3D);
            if (glm::length(swimForward) < 1e-4f) {
                swimForward = forward;
            }
            glm::vec3 swimRight = camera->getRight();
            swimRight.z = 0.0f;
            if (glm::length(swimRight) > 1e-4f) {
                swimRight = glm::normalize(swimRight);
            } else {
                swimRight = right;
            }

            glm::vec3 swimMove(0.0f);
            if (nowForward) swimMove += swimForward;
            if (nowBackward) swimMove -= swimForward;
            if (nowStrafeLeft) swimMove -= swimRight;
            if (nowStrafeRight) swimMove += swimRight;

            if (glm::length(swimMove) > 0.001f) {
                swimMove = glm::normalize(swimMove);
                targetPos += swimMove * swimSpeed * deltaTime;
            }

            // Spacebar = swim up (continuous, not a jump)
            bool diveIntent = nowForward && (forward3D.z < -0.28f);
            if (nowJump) {
                verticalVelocity = SWIM_BUOYANCY;
            } else {
                // Gentle sink when not pressing space
                verticalVelocity += SWIM_GRAVITY * deltaTime;
                if (verticalVelocity < SWIM_SINK_SPEED) {
                    verticalVelocity = SWIM_SINK_SPEED;
                }
                // Strong surface lock while idle/normal swim so buoyancy keeps
                // you afloat unless you're intentionally diving.
                if (!diveIntent) {
                    float surfaceErr = (waterSurfaceZ - targetPos.z);
                    verticalVelocity += surfaceErr * 7.0f * deltaTime;
                    verticalVelocity *= std::max(0.0f, 1.0f - 3.2f * deltaTime);
                    if (std::abs(surfaceErr) < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                        verticalVelocity = 0.0f;
                    }
                }
            }

            targetPos.z += verticalVelocity * deltaTime;

            // Don't rise above water surface
            if (waterH && targetPos.z > *waterH - WATER_SURFACE_OFFSET) {
                targetPos.z = *waterH - WATER_SURFACE_OFFSET;
                if (verticalVelocity > 0.0f) verticalVelocity = 0.0f;
            }

            // Prevent sinking/clipping through world floor while swimming.
            std::optional<float> floorH;
            if (terrainManager) {
                floorH = terrainManager->getHeightAt(targetPos.x, targetPos.y);
            }
            if (wmoRenderer) {
                auto wh = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 2.0f);
                if (wh && (!floorH || *wh > *floorH)) floorH = wh;
            }
            if (m2Renderer) {
                auto mh = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z);
                if (mh && (!floorH || *mh > *floorH)) floorH = mh;
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
                            }
                        }

                        if (m2Renderer) {
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
                targetPos += movement * speed * deltaTime;
            }

            // Jump with input buffering and coyote time
            if (nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
            if (grounded) coyoteTimer = COYOTE_TIME;

            bool canJump = (coyoteTimer > 0.0f) && (jumpBufferTimer > 0.0f);
            if (canJump) {
                verticalVelocity = jumpVel;
                grounded = false;
                jumpBufferTimer = 0.0f;
                coyoteTimer = 0.0f;
            }

            jumpBufferTimer -= deltaTime;
            coyoteTimer -= deltaTime;

            // Apply gravity
            verticalVelocity += gravity * deltaTime;
            targetPos.z += verticalVelocity * deltaTime;
        }

        // Sweep collisions in small steps to reduce tunneling through thin walls/floors.
        // Skip entirely when stationary to avoid wasting collision calls.
        {
            glm::vec3 startPos = *followTarget;
            glm::vec3 desiredPos = targetPos;
            float moveDist = glm::length(desiredPos - startPos);

            if (moveDist > 0.01f) {
                int sweepSteps = std::max(1, std::min(5, static_cast<int>(std::ceil(moveDist / 0.35f))));
                glm::vec3 stepPos = startPos;
                glm::vec3 stepDelta = (desiredPos - startPos) / static_cast<float>(sweepSteps);

                for (int i = 0; i < sweepSteps; i++) {
                    glm::vec3 candidate = stepPos + stepDelta;

                    if (wmoRenderer) {
                        glm::vec3 adjusted;
                        if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted)) {
                            // Before blocking, check if there's a floor at the
                            // destination above current feet (stair step-up).
                            float feetZ = stepPos.z;
                            float probeZ = feetZ + 2.5f;
                            auto floorH = wmoRenderer->getFloorHeight(
                                candidate.x, candidate.y, probeZ);
                            bool isStair = floorH &&
                                           *floorH > feetZ + 0.1f &&
                                           *floorH <= feetZ + 1.6f;
                            if (!isStair) {
                                candidate.x = adjusted.x;
                                candidate.y = adjusted.y;
                            }
                        }
                    }

                    if (m2Renderer) {
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

        // WoW-style slope limiting (50 degrees, with sliding)
        // dot(normal, up) >= 0.64 is walkable, otherwise slide
        constexpr bool ENABLE_SLOPE_SLIDE = false;
        constexpr float MAX_WALK_SLOPE_DOT = 0.6428f;  // cos(50°)
        constexpr float SAMPLE_DIST = 0.3f;  // Distance to sample for normal calculation
        if (ENABLE_SLOPE_SLIDE) {
            glm::vec3 oldPos = *followTarget;
            float moveXY = glm::length(glm::vec2(targetPos.x - oldPos.x, targetPos.y - oldPos.y));
            if (moveXY >= 0.03f) {
                struct GroundSample {
                    std::optional<float> height;
                    bool fromM2 = false;
                };
                // Helper to get ground height at a position and whether M2 provided the top floor.
                auto getGroundAt = [&](float x, float y) -> GroundSample {
                    std::optional<float> terrainH;
                    std::optional<float> wmoH;
                    std::optional<float> m2H;
                    if (terrainManager) {
                        terrainH = terrainManager->getHeightAt(x, y);
                    }
                    float stepUpBudget = grounded ? 1.6f : 1.2f;
                    if (wmoRenderer) {
                        wmoH = wmoRenderer->getFloorHeight(x, y, targetPos.z + stepUpBudget + 0.5f);
                    }
                    if (m2Renderer) {
                        m2H = m2Renderer->getFloorHeight(x, y, targetPos.z);
                    }
                    auto base = selectReachableFloor(terrainH, wmoH, targetPos.z, stepUpBudget);
                    bool fromM2 = false;
                    if (m2H && *m2H <= targetPos.z + stepUpBudget && (!base || *m2H > *base)) {
                        base = m2H;
                        fromM2 = true;
                    }
                    return GroundSample{base, fromM2};
                };

                // Get ground height at target position
                auto center = getGroundAt(targetPos.x, targetPos.y);
                bool skipSlopeCheck = center.height && center.fromM2;
                if (center.height && !skipSlopeCheck) {

                // Calculate ground normal using height samples
                auto hPosX = getGroundAt(targetPos.x + SAMPLE_DIST, targetPos.y);
                auto hNegX = getGroundAt(targetPos.x - SAMPLE_DIST, targetPos.y);
                auto hPosY = getGroundAt(targetPos.x, targetPos.y + SAMPLE_DIST);
                auto hNegY = getGroundAt(targetPos.x, targetPos.y - SAMPLE_DIST);

                // Estimate partial derivatives
                float dzdx = 0.0f, dzdy = 0.0f;
                if (hPosX.height && hNegX.height) {
                    dzdx = (*hPosX.height - *hNegX.height) / (2.0f * SAMPLE_DIST);
                } else if (hPosX.height) {
                    dzdx = (*hPosX.height - *center.height) / SAMPLE_DIST;
                } else if (hNegX.height) {
                    dzdx = (*center.height - *hNegX.height) / SAMPLE_DIST;
                }

                if (hPosY.height && hNegY.height) {
                    dzdy = (*hPosY.height - *hNegY.height) / (2.0f * SAMPLE_DIST);
                } else if (hPosY.height) {
                    dzdy = (*hPosY.height - *center.height) / SAMPLE_DIST;
                } else if (hNegY.height) {
                    dzdy = (*center.height - *hNegY.height) / SAMPLE_DIST;
                }

                // Ground normal = normalize(cross(tangentX, tangentY))
                // tangentX = (1, 0, dzdx), tangentY = (0, 1, dzdy)
                // cross = (-dzdx, -dzdy, 1)
                glm::vec3 groundNormal = glm::normalize(glm::vec3(-dzdx, -dzdy, 1.0f));
                float slopeDot = groundNormal.z;  // dot(normal, up) where up = (0,0,1)

                // Check if slope is too steep
                if (slopeDot < MAX_WALK_SLOPE_DOT) {
                    // Slope too steep - slide instead of walk
                    // Calculate slide direction (downhill, horizontal only)
                    glm::vec2 slideDir = glm::normalize(glm::vec2(-groundNormal.x, -groundNormal.y));

                    // Only block uphill movement, allow downhill/across
                    glm::vec2 moveDir = glm::vec2(targetPos.x - oldPos.x, targetPos.y - oldPos.y);
                    float moveDist = glm::length(moveDir);

                    if (moveDist > 0.001f) {
                        glm::vec2 moveDirNorm = moveDir / moveDist;

                        // How much are we trying to go uphill?
                        float uphillAmount = -glm::dot(moveDirNorm, slideDir);

                        if (uphillAmount > 0.0f) {
                            // Trying to go uphill on steep slope - slide back
                            float slideStrength = (1.0f - slopeDot / MAX_WALK_SLOPE_DOT);
                            targetPos.x = oldPos.x + slideDir.x * moveDist * slideStrength * 0.5f;
                            targetPos.y = oldPos.y + slideDir.y * moveDist * slideStrength * 0.5f;
                        }
                    }
                }
                }
            }
        }

        // Ground the character to terrain or WMO floor
        // Skip entirely while swimming — the swim floor clamp handles vertical bounds.
        if (!swimming) {
            float stepUpBudget = grounded ? 1.6f : 1.2f;

            // 1. Center-only sample for terrain/WMO floor selection.
            //    Using only the center prevents tunnel entrances from snapping
            //    to terrain when offset samples miss the WMO floor geometry.
            std::optional<float> groundH;
            {
                std::optional<float> terrainH;
                std::optional<float> wmoH;
                if (terrainManager) {
                    terrainH = terrainManager->getHeightAt(targetPos.x, targetPos.y);
                }
                float wmoProbeZ = std::max(targetPos.z, lastGroundZ) + stepUpBudget + 0.5f;
                if (wmoRenderer) {
                    wmoH = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, wmoProbeZ);
                }
                groundH = selectReachableFloor(terrainH, wmoH, targetPos.z, stepUpBudget);
            }

            // 2. Multi-sample for M2 objects (rugs, planks, bridges) —
            //    these are narrow and need offset probes to detect reliably.
            if (m2Renderer) {
                constexpr float FOOTPRINT = 0.4f;
                const glm::vec2 offsets[] = {
                    {0.0f, 0.0f},
                    {FOOTPRINT, 0.0f}, {-FOOTPRINT, 0.0f},
                    {0.0f, FOOTPRINT}, {0.0f, -FOOTPRINT}
                };
                float m2ProbeZ = std::max(targetPos.z, lastGroundZ) + 6.0f;
                for (const auto& o : offsets) {
                    auto m2H = m2Renderer->getFloorHeight(
                        targetPos.x + o.x, targetPos.y + o.y, m2ProbeZ);
                    if (m2H && *m2H <= targetPos.z + stepUpBudget &&
                        (!groundH || *m2H > *groundH)) {
                        groundH = m2H;
                    }
                }
            }

            if (groundH) {
                float groundDiff = *groundH - lastGroundZ;
                if (groundDiff > 2.0f) {
                    // Landing on a higher ledge - snap up
                    lastGroundZ = *groundH;
                } else {
                    // Smooth toward detected ground. Use a slower rate for large
                    // drops so multi-story buildings don't snap to the wrong floor,
                    // but always converge so walking off a fountain works.
                    float rate = (groundDiff > -2.0f) ? 15.0f : 6.0f;
                    lastGroundZ += groundDiff * std::min(1.0f, deltaTime * rate);
                }

                if (targetPos.z <= lastGroundZ + 0.1f && verticalVelocity <= 0.0f) {
                    targetPos.z = lastGroundZ;
                    verticalVelocity = 0.0f;
                    grounded = true;
                } else {
                    grounded = false;
                }
            } else {
                // No terrain found — hold at last known ground
                targetPos.z = lastGroundZ;
                verticalVelocity = 0.0f;
                grounded = true;
            }
        }

        // Update follow target position
        *followTarget = targetPos;

        // ===== WoW-style orbit camera =====
        // Pivot point at upper chest/neck
        float mountedOffset = mounted_ ? mountHeightOffset_ : 0.0f;
        glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT + mountedOffset);

        // Camera direction from yaw/pitch (already computed as forward3D)
        glm::vec3 camDir = -forward3D;  // Camera looks at pivot, so it's behind

        // Smooth zoom toward user target
        float zoomLerp = 1.0f - std::exp(-ZOOM_SMOOTH_SPEED * deltaTime);
        currentDistance += (userTargetDistance - currentDistance) * zoomLerp;

        // Limit max zoom when inside a WMO (building interior)
        // Throttle: only recheck every 10 frames or when position changes >2 units.
        static constexpr float WMO_MAX_DISTANCE = 3.5f;
        if (wmoRenderer) {
            float distFromLastCheck = glm::length(targetPos - lastInsideWMOCheckPos);
            if (++insideWMOCheckCounter >= 10 || distFromLastCheck > 2.0f) {
                cachedInsideWMO = wmoRenderer->isInsideWMO(targetPos.x, targetPos.y, targetPos.z + 1.0f, nullptr);
                insideWMOCheckCounter = 0;
                lastInsideWMOCheckPos = targetPos;
            }
            if (cachedInsideWMO && currentDistance > WMO_MAX_DISTANCE) {
                currentDistance = WMO_MAX_DISTANCE;
            }
        }

        // ===== Camera collision (sphere sweep approximation) =====
        // Find max safe distance using raycast + sphere radius
        collisionDistance = currentDistance;

        // Camera collision: terrain-only floor clamping (skip expensive WMO raycasts).
        // The camera may clip through WMO walls but won't go underground.
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
        {
            auto camTerrainH = getTerrainFloorAt(smoothedCamPos.x, smoothedCamPos.y);
            std::optional<float> camWmoH;
            if (wmoRenderer) {
                camWmoH = wmoRenderer->getFloorHeight(
                    smoothedCamPos.x, smoothedCamPos.y, smoothedCamPos.z + 3.0f);
            }
            auto camFloorH = selectReachableFloor(
                camTerrainH, camWmoH, smoothedCamPos.z, 3.0f);
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
            bool shouldHidePlayer = (actualDist < MIN_DISTANCE + 0.1f);  // Hide in first-person
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
            if (m2Renderer) m2H = m2Renderer->getFloorHeight(newPos.x, newPos.y, feetZ + 1.0f);
            auto floorH = selectHighestFloor(terrainH, wmoH, m2H);
            constexpr float MIN_SWIM_WATER_DEPTH = 1.8f;
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
                newPos += movement * swimSpeed * deltaTime;
            }

            if (nowJump) {
                verticalVelocity = SWIM_BUOYANCY;
            } else {
                verticalVelocity += SWIM_GRAVITY * deltaTime;
                if (verticalVelocity < SWIM_SINK_SPEED) {
                    verticalVelocity = SWIM_SINK_SPEED;
                }
                if (!diveIntent) {
                    float surfaceErr = (waterSurfaceCamZ - newPos.z);
                    verticalVelocity += surfaceErr * 7.0f * deltaTime;
                    verticalVelocity *= std::max(0.0f, 1.0f - 3.2f * deltaTime);
                    if (std::abs(surfaceErr) < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                        verticalVelocity = 0.0f;
                    }
                }
            }

            newPos.z += verticalVelocity * deltaTime;

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
                newPos += movement * speed * deltaTime;
            }

            // Jump with input buffering and coyote time
            if (nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
            if (grounded) coyoteTimer = COYOTE_TIME;

            if (coyoteTimer > 0.0f && jumpBufferTimer > 0.0f) {
                verticalVelocity = jumpVel;
                grounded = false;
                jumpBufferTimer = 0.0f;
                coyoteTimer = 0.0f;
            }

            jumpBufferTimer -= deltaTime;
            coyoteTimer -= deltaTime;

            // Apply gravity
            verticalVelocity += gravity * deltaTime;
            newPos.z += verticalVelocity * deltaTime;
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
                        candidate = adjusted;
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
                if (m2Renderer) {
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
                float groundDiff = *groundH - lastGroundZ;
                if (groundDiff > 2.0f) {
                    lastGroundZ = *groundH;
                } else {
                    float rate = (groundDiff > -2.0f) ? 15.0f : 6.0f;
                    lastGroundZ += groundDiff * std::min(1.0f, deltaTime * rate);
                }

                float groundZ = lastGroundZ + eyeHeight;
                if (newPos.z <= groundZ) {
                    newPos.z = groundZ;
                    verticalVelocity = 0.0f;
                    grounded = true;
                    swimming = false;  // Touching ground = wading
                } else if (!swimming) {
                    grounded = false;
                }
            } else if (!swimming) {
                float groundZ = lastGroundZ + eyeHeight;
                newPos.z = groundZ;
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
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_FORWARD));
        }
        if (nowBackward && !wasMovingBackward) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_BACKWARD));
        }
        if ((!nowForward && wasMovingForward) || (!nowBackward && wasMovingBackward)) {
            if (!nowForward && !nowBackward) {
                movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_STOP));
            }
        }

        // Strafing
        if (nowStrafeLeft && !wasStrafingLeft) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_STRAFE_LEFT));
        }
        if (nowStrafeRight && !wasStrafingRight) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_STRAFE_RIGHT));
        }
        if ((!nowStrafeLeft && wasStrafingLeft) || (!nowStrafeRight && wasStrafingRight)) {
            if (!nowStrafeLeft && !nowStrafeRight) {
                movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_STOP_STRAFE));
            }
        }

        // Turning
        if (nowTurnLeft && !wasTurningLeft) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_TURN_LEFT));
        }
        if (nowTurnRight && !wasTurningRight) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_TURN_RIGHT));
        }
        if ((!nowTurnLeft && wasTurningLeft) || (!nowTurnRight && wasTurningRight)) {
            if (!nowTurnLeft && !nowTurnRight) {
                movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_STOP_TURN));
            }
        }

        // Jump
        if (nowJump && !wasJumping && grounded) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_JUMP));
        }

        // Fall landing
        if (wasFalling && grounded) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_FALL_LAND));
        }
    }

    // Swimming state transitions
    if (movementCallback) {
        if (swimming && !wasSwimming) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_START_SWIM));
        } else if (!swimming && wasSwimming) {
            movementCallback(static_cast<uint32_t>(game::Opcode::CMSG_MOVE_STOP_SWIM));
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

    if (event.button == SDL_BUTTON_LEFT) {
        leftMouseDown = (event.state == SDL_PRESSED);
        if (event.state == SDL_PRESSED && event.clicks >= 2) {
            autoRunning = false;
        }
    }
    if (event.button == SDL_BUTTON_RIGHT) {
        rightMouseDown = (event.state == SDL_PRESSED);
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
        if (m2Renderer) {
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

void CameraController::processMouseWheel(float delta) {
    // Adjust user's target distance (collision may limit actual distance)
    userTargetDistance -= delta * 2.0f;  // 2.0 units per scroll notch
    userTargetDistance = glm::clamp(userTargetDistance, MIN_DISTANCE, MAX_DISTANCE);
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
    return moveForwardActive || moveBackwardActive || strafeLeftActive || strafeRightActive || autoRunning;
}

bool CameraController::isSprinting() const {
    return enabled && camera && runPace;
}

} // namespace rendering
} // namespace wowee
