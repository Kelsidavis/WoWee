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

namespace wowee {
namespace rendering {

namespace {

std::optional<float> selectReachableFloor(const std::optional<float>& terrainH,
                                          const std::optional<float>& wmoH,
                                          float refZ,
                                          float maxStepUp) {
    std::optional<float> best;
    auto consider = [&](const std::optional<float>& h) {
        if (!h) return;
        if (*h > refZ + maxStepUp) return;  // Ignore roofs/floors too far above us.
        if (!best || *h > *best) {
            best = *h;  // Choose highest reachable floor.
        }
    };
    consider(terrainH);
    consider(wmoH);
    return best;
}

} // namespace

CameraController::CameraController(Camera* cam) : camera(cam) {
    yaw = defaultYaw;
    facingYaw = defaultYaw;
    pitch = defaultPitch;
    reset();
}

void CameraController::update(float deltaTime) {
    if (!enabled || !camera) {
        return;
    }

    auto& input = core::Input::getInstance();

    // Don't process keyboard input when UI (e.g. chat box) has focus
    bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard;

    // Determine current key states
    bool keyW = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_W);
    bool keyS = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_S);
    bool keyA = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_A);
    bool keyD = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_D);
    bool keyQ = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_Q);
    bool keyE = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_E);
    bool shiftDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LSHIFT) || input.isKeyPressed(SDL_SCANCODE_RSHIFT));
    bool ctrlDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LCTRL) || input.isKeyPressed(SDL_SCANCODE_RCTRL));
    bool nowJump = !uiWantsKeyboard && !sitting && input.isKeyPressed(SDL_SCANCODE_SPACE);

    bool mouseAutorun = !uiWantsKeyboard && !sitting && leftMouseDown && rightMouseDown;
    bool nowForward = keyW || mouseAutorun;
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

    // Toggle sit/crouch with X or C key (edge-triggered) — only when UI doesn't want keyboard
    bool xDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_X) || input.isKeyPressed(SDL_SCANCODE_C));
    if (xDown && !xKeyWasDown) {
        sitting = !sitting;
    }
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

        // Check for water at current position
        std::optional<float> waterH;
        if (waterRenderer) {
            waterH = waterRenderer->getWaterHeightAt(targetPos.x, targetPos.y);
        }
        constexpr float MAX_SWIM_DEPTH_FROM_SURFACE = 12.0f;
        bool inWater = waterH && targetPos.z < *waterH &&
                       ((*waterH - targetPos.z) <= MAX_SWIM_DEPTH_FROM_SURFACE);


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
                auto wh = wmoRenderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z + 5.0f);
                if (wh && (!floorH || *wh > *floorH)) floorH = wh;
            }
            if (m2Renderer) {
                auto mh = m2Renderer->getFloorHeight(targetPos.x, targetPos.y, targetPos.z);
                if (mh && (!floorH || *mh > *floorH)) floorH = mh;
            }
            if (floorH) {
                float swimFloor = *floorH + 0.30f;
                if (targetPos.z < swimFloor) {
                    targetPos.z = swimFloor;
                    if (verticalVelocity < 0.0f) verticalVelocity = 0.0f;
                }
            }

            // Enforce collision while swimming too (horizontal only), so we don't
            // pass through walls/props when underwater or at waterline.
            {
                glm::vec3 swimFrom = *followTarget;
                glm::vec3 swimTo = targetPos;
                float swimMoveDist = glm::length(swimTo - swimFrom);
                int swimSteps = std::max(1, std::min(12, static_cast<int>(std::ceil(swimMoveDist / 0.22f))));
                glm::vec3 stepPos = swimFrom;
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

                targetPos.x = stepPos.x;
                targetPos.y = stepPos.y;
            }

            grounded = false;
        } else {
            swimming = false;

            if (glm::length(movement) > 0.001f) {
                movement = glm::normalize(movement);
                targetPos += movement * speed * deltaTime;
            }

            // Jump
            if (nowJump && grounded) {
                verticalVelocity = jumpVel;
                grounded = false;
            }

            // Apply gravity
            verticalVelocity += gravity * deltaTime;
            targetPos.z += verticalVelocity * deltaTime;
        }

        // Sweep collisions in small steps to reduce tunneling through thin walls/floors.
        {
            glm::vec3 startPos = *followTarget;
            glm::vec3 desiredPos = targetPos;
            float moveDist = glm::length(desiredPos - startPos);
            // Adaptive CCD: keep per-step movement short, especially on low FPS spikes.
            int sweepSteps = std::max(1, std::min(14, static_cast<int>(std::ceil(moveDist / 0.24f))));
            if (deltaTime > 0.04f) {
                sweepSteps = std::min(16, std::max(sweepSteps, static_cast<int>(std::ceil(deltaTime / 0.016f)) * 2));
            }
            glm::vec3 stepPos = startPos;
            glm::vec3 stepDelta = (desiredPos - startPos) / static_cast<float>(sweepSteps);

            for (int i = 0; i < sweepSteps; i++) {
                glm::vec3 candidate = stepPos + stepDelta;

                if (wmoRenderer) {
                    glm::vec3 adjusted;
                    if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted)) {
                        // Keep vertical motion from physics/grounding; only block horizontal wall penetration.
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

            targetPos = stepPos;
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
                    if (wmoRenderer) {
                        wmoH = wmoRenderer->getFloorHeight(x, y, targetPos.z + 5.0f);
                    }
                    if (m2Renderer) {
                        m2H = m2Renderer->getFloorHeight(x, y, targetPos.z);
                    }
                    float stepUpBudget = grounded ? 1.6f : 1.2f;
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
        {
            auto sampleGround = [&](float x, float y) -> std::optional<float> {
                std::optional<float> terrainH;
                std::optional<float> wmoH;
                std::optional<float> m2H;
                if (terrainManager) {
                    terrainH = terrainManager->getHeightAt(x, y);
                }
                if (wmoRenderer) {
                    wmoH = wmoRenderer->getFloorHeight(x, y, targetPos.z + eyeHeight);
                }
                if (m2Renderer) {
                    m2H = m2Renderer->getFloorHeight(x, y, targetPos.z);
                }
                float stepUpBudget = grounded ? 1.6f : 1.2f;
                auto base = selectReachableFloor(terrainH, wmoH, targetPos.z, stepUpBudget);
                if (m2H && *m2H <= targetPos.z + stepUpBudget && (!base || *m2H > *base)) {
                    base = m2H;
                }
                return base;
            };

            // Sample center + small footprint to avoid slipping through narrow floor pieces.
            std::optional<float> groundH;
            constexpr float FOOTPRINT = 0.28f;
            const glm::vec2 offsets[] = {
                {0.0f, 0.0f},
                {FOOTPRINT, 0.0f}, {-FOOTPRINT, 0.0f},
                {0.0f, FOOTPRINT}, {0.0f, -FOOTPRINT}
            };
            for (const auto& o : offsets) {
                auto h = sampleGround(targetPos.x + o.x, targetPos.y + o.y);
                if (h && (!groundH || *h > *groundH)) {
                    groundH = h;
                }
            }

            if (groundH) {
                float groundDiff = *groundH - lastGroundZ;
                if (std::abs(groundDiff) < 2.0f) {
                    // Small height difference - smooth it
                    lastGroundZ += groundDiff * std::min(1.0f, deltaTime * 15.0f);
                } else {
                    // Large height difference - snap (for falling onto ledges)
                    lastGroundZ = *groundH;
                }

                if (targetPos.z <= lastGroundZ + 0.1f) {
                    targetPos.z = lastGroundZ;
                    verticalVelocity = 0.0f;
                    grounded = true;
                    swimming = false;  // Touching ground = wading, not swimming
                } else if (!swimming) {
                    grounded = false;
                }
            } else if (!swimming) {
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
        glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT);

        // Camera direction from yaw/pitch (already computed as forward3D)
        glm::vec3 camDir = -forward3D;  // Camera looks at pivot, so it's behind

        // Smooth zoom toward user target
        float zoomLerp = 1.0f - std::exp(-ZOOM_SMOOTH_SPEED * deltaTime);
        currentDistance += (userTargetDistance - currentDistance) * zoomLerp;

        // ===== Camera collision (sphere sweep approximation) =====
        // Find max safe distance using raycast + sphere radius
        collisionDistance = currentDistance;

        // Helper to get floor height
        auto getFloorAt = [&](float x, float y, float z) -> std::optional<float> {
            std::optional<float> terrainH;
            std::optional<float> wmoH;
            if (terrainManager) {
                terrainH = terrainManager->getHeightAt(x, y);
            }
            if (wmoRenderer) {
                wmoH = wmoRenderer->getFloorHeight(x, y, z + 5.0f);
            }
            // Camera floor clamp must allow larger step-up on ramps/stairs.
            // Too-small limits let the camera slip below rising ground and see through floors.
            return selectReachableFloor(terrainH, wmoH, z, 2.0f);
        };

        // Raycast against WMO bounding boxes
        if (wmoRenderer && collisionDistance > MIN_DISTANCE) {
            float wmoHit = wmoRenderer->raycastBoundingBoxes(pivot, camDir, collisionDistance);
            if (wmoHit < collisionDistance) {
                collisionDistance = std::max(MIN_DISTANCE, wmoHit - CAM_SPHERE_RADIUS - CAM_EPSILON);
            }
        }

        // Intentionally ignore M2 doodads for camera collision to match WoW feel.

        // Check floor collision along the camera path
        // Sample a few points to find where camera would go underground
        for (int i = 1; i <= 2; i++) {
            float testDist = collisionDistance * (float(i) / 2.0f);
            glm::vec3 testPos = pivot + camDir * testDist;
            auto floorH = getFloorAt(testPos.x, testPos.y, testPos.z);

            if (floorH && testPos.z < *floorH + CAM_SPHERE_RADIUS + CAM_EPSILON) {
                // Camera would be underground at this distance
                collisionDistance = std::max(MIN_DISTANCE, testDist - CAM_SPHERE_RADIUS);
                break;
            }
        }

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
        // Sample a small footprint around the camera to avoid peeking through ramps/stairs
        // when zoomed out and pitched down.
        constexpr float MIN_FLOOR_CLEARANCE = 0.35f;
        constexpr float FLOOR_SAMPLE_R = 0.35f;
        std::optional<float> finalFloorH;
        const glm::vec2 floorOffsets[] = {
            {0.0f, 0.0f},
            {FLOOR_SAMPLE_R * 0.7f, FLOOR_SAMPLE_R * 0.7f},
            {-FLOOR_SAMPLE_R * 0.7f, -FLOOR_SAMPLE_R * 0.7f}
        };
        for (const auto& o : floorOffsets) {
            auto h = getFloorAt(smoothedCamPos.x + o.x, smoothedCamPos.y + o.y, smoothedCamPos.z);
            if (h && (!finalFloorH || *h > *finalFloorH)) {
                finalFloorH = h;
            }
        }
        if (finalFloorH && smoothedCamPos.z < *finalFloorH + MIN_FLOOR_CLEARANCE) {
            smoothedCamPos.z = *finalFloorH + MIN_FLOOR_CLEARANCE;
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
        bool inWater = waterH && feetZ < *waterH &&
                       ((*waterH - feetZ) <= MAX_SWIM_DEPTH_FROM_SURFACE);


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

            // Jump
            if (nowJump && grounded) {
                verticalVelocity = jumpVel;
                grounded = false;
            }

            // Apply gravity
            verticalVelocity += gravity * deltaTime;
            newPos.z += verticalVelocity * deltaTime;
        }

        // Wall sweep collision before grounding (reduces tunneling at low FPS/high speed).
        if (wmoRenderer) {
            glm::vec3 startFeet = camera->getPosition() - glm::vec3(0, 0, eyeHeight);
            glm::vec3 desiredFeet = newPos - glm::vec3(0, 0, eyeHeight);
            float moveDist = glm::length(desiredFeet - startFeet);
            int sweepSteps = std::max(1, std::min(14, static_cast<int>(std::ceil(moveDist / 0.24f))));
            if (deltaTime > 0.04f) {
                sweepSteps = std::min(16, std::max(sweepSteps, static_cast<int>(std::ceil(deltaTime / 0.016f)) * 2));
            }
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

        // Ground to terrain or WMO floor
        {
            auto sampleGround = [&](float x, float y) -> std::optional<float> {
                std::optional<float> terrainH;
                std::optional<float> wmoH;
                std::optional<float> m2H;
                if (terrainManager) {
                    terrainH = terrainManager->getHeightAt(x, y);
                }
                if (wmoRenderer) {
                    wmoH = wmoRenderer->getFloorHeight(x, y, newPos.z);
                }
                if (m2Renderer) {
                    m2H = m2Renderer->getFloorHeight(x, y, newPos.z - eyeHeight);
                }
                auto base = selectReachableFloor(terrainH, wmoH, newPos.z - eyeHeight, 1.0f);
                if (m2H && *m2H <= (newPos.z - eyeHeight) + 1.0f && (!base || *m2H > *base)) {
                    base = m2H;
                }
                return base;
            };

            std::optional<float> groundH;
            constexpr float FOOTPRINT = 0.28f;
            const glm::vec2 offsets[] = {
                {0.0f, 0.0f}, {FOOTPRINT, 0.0f}, {-FOOTPRINT, 0.0f}, {0.0f, FOOTPRINT}, {0.0f, -FOOTPRINT}
            };
            for (const auto& o : offsets) {
                auto h = sampleGround(newPos.x + o.x, newPos.y + o.y);
                if (h && (!groundH || *h > *groundH)) {
                    groundH = h;
                }
            }

            if (groundH) {
                lastGroundZ = *groundH;
                float groundZ = *groundH + eyeHeight;
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

    // Reset camera/character (R key, edge-triggered)
    bool rDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_R);
    if (rDown && !rKeyWasDown) {
        reset();
    }
    rKeyWasDown = rDown;
}

void CameraController::processMouseMotion(const SDL_MouseMotionEvent& event) {
    if (!enabled || !camera) {
        return;
    }

    if (!mouseButtonDown) {
        return;
    }

    // Directly update stored yaw/pitch (no lossy forward-vector derivation)
    yaw -= event.xrel * mouseSensitivity;
    pitch += event.yrel * mouseSensitivity;

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

    // Snap spawn to terrain or WMO surface
    std::optional<float> h;
    if (terrainManager) {
        h = terrainManager->getHeightAt(spawnPos.x, spawnPos.y);
    }
    if (wmoRenderer) {
        auto wh = wmoRenderer->getFloorHeight(spawnPos.x, spawnPos.y, spawnPos.z);
        if (wh && (!h || *wh > *h)) {
            h = wh;
        }
    }
    if (h) {
        lastGroundZ = *h;
        spawnPos.z = *h;
    }

    camera->setRotation(yaw, pitch);
    glm::vec3 forward3D = camera->getForward();

    if (thirdPerson && followTarget) {
        // In follow mode, respawn the character (feet position), then place camera behind it.
        *followTarget = spawnPos;

        currentDistance = userTargetDistance;
        collisionDistance = currentDistance;

        glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, PIVOT_HEIGHT);
        glm::vec3 camDir = -forward3D;
        glm::vec3 camPos = pivot + camDir * currentDistance;
        smoothedCamPos = camPos;
        camera->setPosition(camPos);
    } else {
        // Free-fly mode keeps camera eye-height above ground.
        if (h) {
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

    if (ImGui::GetIO().WantCaptureKeyboard) {
        return false;
    }

    auto& input = core::Input::getInstance();
    bool keyW = input.isKeyPressed(SDL_SCANCODE_W);
    bool keyS = input.isKeyPressed(SDL_SCANCODE_S);
    bool keyA = input.isKeyPressed(SDL_SCANCODE_A);
    bool keyD = input.isKeyPressed(SDL_SCANCODE_D);
    bool keyQ = input.isKeyPressed(SDL_SCANCODE_Q);
    bool keyE = input.isKeyPressed(SDL_SCANCODE_E);

    // In third-person without RMB, A/D are turn keys (not movement).
    if (thirdPerson && !rightMouseDown) {
        return keyW || keyS || keyQ || keyE;
    }

    bool mouseAutorun = leftMouseDown && rightMouseDown;
    return keyW || keyS || keyA || keyD || keyQ || keyE || mouseAutorun;
}

bool CameraController::isSprinting() const {
    return enabled && camera && runPace;
}

} // namespace rendering
} // namespace wowee
