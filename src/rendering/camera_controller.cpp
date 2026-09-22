#include "rendering/camera_controller.hpp"

#include "core/env_flag.hpp"
#include "rendering/swim_wall.hpp"
#include "core/click_drag.hpp"
#include "core/coordinates.hpp"
#include "ui/keybinding_manager.hpp"
#include "ui/framexml_takeover.hpp"
#include "rendering/movement_limits.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <future>
#include <sstream>
#include <string>
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

/// WoW's own key bone id for the head, as the M2 skeleton names it.
constexpr int32_t kKeyBoneHead = 6;

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
        // Prefer the highest surface - prevents clipping through
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
        if (!best || *h > *best) best = h;
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
        if (!best || *h > *best) best = h;
    };
    consider(a);
    consider(b);
    consider(c);
    return best;
}


/// How flat a surface has to be to stand on rather than slide off.
///
/// Three names for one number. They are kept apart because the surfaces are:
/// a WMO tunnel or bridge ramp is authored steeper than outdoor terrain, and if
/// that ever needs different limits this is where they part. Today they agree.
constexpr float MIN_WALKABLE_NORMAL_TERRAIN = movement::kMinWalkableNormalZ;
constexpr float MIN_WALKABLE_NORMAL_WMO = movement::kMinWalkableNormalZ;
constexpr float MIN_WALKABLE_NORMAL_M2 = movement::kMinWalkableNormalZ;

/// The points around the feet a floor probe samples.
///
/// One ray straight down at the character's centre falls between two planks, or
/// misses a stair tread, and the character drops through a floor that is plainly
/// there. Sampling a ring around the feet instead is what stops it. Five points
/// for a cheap check, nine when something has already gone wrong and a wider
/// footprint is worth the queries.
///
/// The ring was written out five times, each with its own footprint constant
/// spelled into nine brace pairs.
std::array<glm::vec2, 5> feetCross(float footprint) {
    return {{ {0.0f, 0.0f},
              { footprint, 0.0f}, {-footprint, 0.0f},
              {0.0f,  footprint}, {0.0f, -footprint} }};
}

std::array<glm::vec2, 9> feetRing(float footprint) {
    return {{ {0.0f, 0.0f},
              { footprint, 0.0f}, {-footprint, 0.0f},
              {0.0f,  footprint}, {0.0f, -footprint},
              { footprint,  footprint}, { footprint, -footprint},
              {-footprint,  footprint}, {-footprint, -footprint} }};
}

/// The band a floor has to be in to be believed: high enough to step onto,
/// low enough not to be the ceiling or another storey.
struct FloorBand {
    float minZ;
    float maxZ;
};

/// The highest walkable floor among the sampled points, if any.
///
/// `query` is a renderer's getFloorHeight - it answers a height and writes the
/// surface normal's Z, and a normal flatter than `minNormalZ` is a wall rather
/// than something to stand on.
template <typename Offsets, typename Query>
std::optional<float> highestWalkableFloor(Query&& query, float x, float y,
                                          const Offsets& offsets, float probeZ,
                                          float minNormalZ, FloorBand band) {
    std::optional<float> best;
    for (const auto& o : offsets) {
        float normalZ = 1.0f;
        auto h = query(x + o.x, y + o.y, probeZ, &normalZ);
        if (!h) continue;
        if (normalZ < minNormalZ) continue;
        if (*h > band.maxZ || *h < band.minZ) continue;
        if (!best || *h > *best) best = h;
    }
    return best;
}

} // namespace

CameraController::CameraController(Camera* cam) : camera(cam) {
    yaw = defaultYaw;
    facingYaw = defaultYaw;
    pitch = defaultPitch;
    reset();
}

/// The floor dumps, which are a hunt rather than a fault.
///
/// Each is ten lines of every WMO group and doodad near a position, written
/// whenever a floor probe comes up empty - which happens in ordinary play and
/// is recovered from. In one six-minute session they were ninety-one lines of
/// eleven hundred, at the level these logs carry, describing something that
/// then worked. The one-line statement that a probe found nothing stays; the
/// inventory behind it is asked for.
///
/// F8 still dumps on demand, which is somebody asking for exactly this.
bool floorDiagEnabled() {
    static const bool on = core::envFlagEnabled("WOWEE_FLOOR_DIAG", false);
    return on;
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
float CameraController::raymarchTerrainCameraLimit(const glm::vec3& pivot, const glm::vec3& camDir,
                                                   float maxDist) const {
    if (!terrainManager || maxDist <= MIN_DISTANCE) return maxDist;

    // Clearance the camera keeps above the terrain surface along the whole ray.
    // Enough to keep the near plane out of the ground and no more: the quarter
    // yard on top of the sphere that used to be here was felt as the ground
    // pushing the camera forward on every rise.
    constexpr float kClearance = CAM_SPHERE_RADIUS + 0.08f;

    // If the pivot itself sits below the heightfield (caves, WMO basements,
    // terrain holes under cities), the heightfield is not the relevant occluder
    // here - skip limiting rather than pinning the camera to first-person.
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
    // Scaled here rather than at each caller, so the setting covers the sources
    // that exist - a spell's shake and a thunderstorm's - and any added later.
    magnitude *= shakeScale_;
    // Allow stronger shake to override weaker; don't allow zero magnitude.
    if (magnitude <= 0.0f || duration <= 0.0f) return;
    if (magnitude > shakeMagnitude_ || shakeElapsed_ >= shakeDuration_) {
        shakeMagnitude_ = magnitude;
        shakeFrequency_ = frequency;
        shakeDuration_  = duration;
        shakeElapsed_   = 0.0f;
    }
}

glm::vec3 CameraController::sweepAgainstWalls(const glm::vec3& from, const glm::vec3& to,
                                              bool includeDoodads) {
    const glm::vec3 delta = to - from;
    const float distSq = glm::dot(delta, delta);
    if (distSq <= 1e-4f) return to;   // stationary: not worth a collision call

    const float dist = std::sqrt(distSq);
    // Tighter steps indoors, where the geometry is closer together.
    const float stepSize = cachedInsideWMO ? 0.20f : 0.35f;
    const int steps = std::max(1, std::min(8, static_cast<int>(std::ceil(dist / stepSize))));
    const glm::vec3 stepDelta = delta / static_cast<float>(steps);

    glm::vec3 stepPos = from;
    for (int i = 0; i < steps; i++) {
        glm::vec3 candidate = stepPos + stepDelta;

        if (wmoRenderer) {
            glm::vec3 adjusted;
            if (wmoRenderer->checkWallCollision(stepPos, candidate, adjusted, cachedInsideWMO)) {
                candidate.x = adjusted.x;
                candidate.y = adjusted.y;
                // Up is a ramp; down would be a wall pulling the feet under the floor.
                candidate.z = std::max(candidate.z, adjusted.z);
            }
        }

        if (includeDoodads && m2Renderer && !externalFollow_) {
            glm::vec3 adjusted;
            if (m2Renderer->checkCollision(stepPos, candidate, adjusted)) {
                candidate.x = adjusted.x;
                candidate.y = adjusted.y;
            }
        }

        stepPos = candidate;
    }
    return stepPos;
}

// The camera that orbits a character and moves it.
//
// Collision, grounding, swimming, flight, the zoom, and the pullback that keeps
// the camera out of walls. This was the larger half of a two-thousand-line
// update(), and everything it needs from the half before it arrives in `f`.
// Where the followed character wants to be this frame.
//
// The movement intent - walking, swimming, flying, jumping, falling - and the
// swept wall collision that keeps it out of geometry. Answers the position it
// arrived at; the floor has not been consulted yet.
void CameraController::setFlyingActive(bool active) {
    // Said when it changes, as take-off and landing are: whether space climbs
    // or jumps comes down to this one flag from the server, and a report that
    // the mount "will not fly" is otherwise a guess at whether it was sent.
    if (active != flyingActive_) {
        LOG_WARNING("Flight: ", active ? "the server allows flying here"
                                       : "flying is no longer allowed");
    }
    flyingActive_ = active;
    if (!active) flightAirborne_ = false;
}

glm::vec3 CameraController::moveFollowedCharacter(float /*deltaTime*/, FrameInput& f,
                                                  glm::vec3& prevTargetPos) {
    // Move the follow target (character position) instead of the camera
    prevTargetPos = *followTarget;
    glm::vec3 targetPos = *followTarget;
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
        // flipped between swim and walk every frame - each flip restarts the
        // locomotion animation and sends a START_SWIM/STOP_SWIM pair, which
        // is what made walking out of water stutter. The two bounds sit
        // either side of the single 1.0 this replaces.
        constexpr float SWIM_ENTER_WATER_DEPTH = 1.15f;
        constexpr float SWIM_EXIT_WATER_DEPTH  = 0.85f;
        const float MIN_SWIM_WATER_DEPTH =
            swimming ? SWIM_EXIT_WATER_DEPTH : SWIM_ENTER_WATER_DEPTH;
        bool inWater = false;
        // Water Walk: treat water surface as ground - player walks on top, not through.
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
            // swimming, any depth is fine - you can only get deep by
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
                if (swimming && inWater && floorH && f.nowForward) {
                    float floorDelta = *floorH - targetPos.z;
                    float waterOverFloor = *waterH - *floorH;
                    // Measured from where a swimmer actually floats, not from
                    // a bare number.
                    //
                    // depthFromFeet is WATER_SURFACE_OFFSET exactly while
                    // swimming - the feet are pinned that far under the
                    // surface - so this constant was really "the float depth
                    // plus a little". It was 1.45 against a float depth of 0.9.
                    // Raising the float depth to 1.45 so the character treads at
                    // chest height put the test exactly on its own threshold,
                    // and the assist that walks a swimmer up a shore stopped
                    // firing: the character stayed swimming in ankle-deep water,
                    // shuddering on the boundary, and never fully left it.
                    bool nearSurface = depthFromFeet <= WATER_SURFACE_OFFSET + 0.55f;
                    bool reachableRamp = (floorDelta >= -0.30f && floorDelta <= 1.10f);
                    bool shallowRampWater = waterOverFloor <= 1.55f;
                    bool notDiving = f.forward3D.z > -0.20f && !f.xDown;
                    if (nearSurface && reachableRamp && shallowRampWater && notDiving) {
                        inWater = false;
                    }
                }

                // Forward plank/ramp assist: sample structure floors ahead so water exit
                // can happen when the ramp is in front of us (not only under our feet).
                if (swimming && inWater && f.nowForward && f.forward3D.z > -0.20f && !f.xDown) {
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

                    glm::vec2 fwd2(f.forward.x, f.forward.y);
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
        // Swim movement follows look pitch (f.forward/back), while strafe stays
        // lateral for stable control.
        float swimSpeed = (swimSpeedOverride_ > 0.0f && swimSpeedOverride_ < 100.0f && !std::isnan(swimSpeedOverride_))
                              ? swimSpeedOverride_ : f.speed * SWIM_SPEED_FACTOR;
        float waterSurfaceZ = waterH ? (*waterH - WATER_SURFACE_OFFSET) : targetPos.z;

        // For auto-run/auto-swim: use character facing (immune to camera pan)
        // For manual W key: use camera direction (swim where you look)
        glm::vec3 swimForward;
        if (autoRunning || (leftMouseDown && rightMouseDown)) {
            // Auto-running: use character's horizontal facing direction
            swimForward = f.forward;
        } else {
            // Manual control: use camera's 3D direction (swim where you look)
            swimForward = glm::normalize(f.forward3D);
            if (glm::dot(swimForward, swimForward) < 1e-8f) {
                swimForward = f.forward;
            }
        }
        // Use character's facing direction for strafe, not camera's right vector
        glm::vec3 swimRight = f.right;  // Character's right (horizontal facing), not camera's

        float swimBackSpeed = (swimBackSpeedOverride_ > 0.0f && swimBackSpeedOverride_ < 100.0f
                                && !std::isnan(swimBackSpeedOverride_))
                                   ? swimBackSpeedOverride_ : swimSpeed * 0.5f;

        glm::vec3 swimMove(0.0f);
        if (f.nowForward) swimMove += swimForward;
        if (f.nowBackward) swimMove -= swimForward;
        if (f.nowStrafeLeft) swimMove += swimRight;
        if (f.nowStrafeRight) swimMove -= swimRight;

        float swimMoveLenSq = glm::dot(swimMove, swimMove);
        if (swimMoveLenSq > 1e-6f) {
            swimMove *= glm::inversesqrt(swimMoveLenSq);
            // Use backward swim speed when moving backwards only (not when combining with strafe)
            float applySpeed = (f.nowBackward && !f.nowForward) ? swimBackSpeed : swimSpeed;
            targetPos += swimMove * applySpeed * f.physicsDeltaTime;
        }

        // Spacebar = swim up, X = swim down (both continuous, not a jump)
        bool diveKey = f.xDown;
        bool diveIntent = diveKey || (f.nowForward && (f.forward3D.z < -0.28f));
        if (f.spaceHeld) {
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
            verticalVelocity *= std::max(0.0f, 1.0f - SWIM_VERTICAL_DAMPING * f.physicsDeltaTime);
            if (std::abs(verticalVelocity) < 0.05f) verticalVelocity = 0.0f;

            // Near the surface a body does float up to it, which is what
            // keeps a swimmer's head out of the water instead of drifting
            // just below. Only within a short band, and not while the look
            // direction says the player is diving.
            const float surfaceErr = waterSurfaceZ - targetPos.z;
            if (!diveIntent && surfaceErr > 0.0f && surfaceErr < SWIM_FLOAT_BAND) {
                verticalVelocity += surfaceErr * 6.0f * f.physicsDeltaTime;
                if (surfaceErr < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                    verticalVelocity = 0.0f;
                }
            }
        }

        targetPos.z += verticalVelocity * f.physicsDeltaTime;

        // Don't rise above water surface
        if (waterH && targetPos.z > *waterH - WATER_SURFACE_OFFSET) {
            targetPos.z = *waterH - WATER_SURFACE_OFFSET;
            if (verticalVelocity > 0.0f) verticalVelocity = 0.0f;
        }

        // Prevent sinking/clipping through world floor while swimming.
        // Cache floor queries (update every 3 frames or 1 unit f.movement)
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

            // Terrain is not a surface a swimmer can be underneath.
            //
            // considerFloor only takes a floor at or just above the feet, so a
            // dive can pass under a dock rather than being held on top of it.
            // Terrain has no underside to pass under, and once the feet are
            // below the heightmap its sample is far above them: the guard drops
            // it, no floor is left, and nothing pushes back. One fast frame
            // through the surface and the swimmer is under the map for good.
            //
            // A structure floor found beneath the feet still owns the vertical,
            // so a cave or an interior that genuinely sits below the heightmap
            // keeps swimming as it did.
            if (terrainManager && !floorH) {
                auto th = terrainManager->getHeightAt(targetPos.x, targetPos.y);
                if (th && *th > targetPos.z) floorH = th;
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
            // Horizontal only: the swim clamp owns the vertical.
            glm::vec3 stepPos = sweepAgainstWalls(*followTarget, targetPos, true);

            // Terrain is a wall to a swimmer, not only a floor.
            //
            // sweepAgainstWalls knows WMO walls and doodad collision, and a
            // shoreline is neither: it is the heightmap, which the swim path
            // only ever consulted downward, for something to float above. So
            // nothing stopped a swimmer crossing into a hillside - and the
            // floor probe above will not push them out of it either, because
            // it takes a floor only at or just above the feet and a cliff face
            // ahead is far above them. Leaving the water inside the hill then
            // drops them through the world.
            //
            // Only a rise the swimmer could not float over counts. The clamp
            // holds them half a yard above the floor, so a beach shelving up
            // under them stays passable and they swim ashore as before.
            constexpr float SWIM_TERRAIN_WALL_CLEARANCE = 0.9f;
            if (terrainManager) {
                const SwimStep allowed = swimStepAgainstTerrain(
                    {followTarget->x, followTarget->y}, {stepPos.x, stepPos.y},
                    targetPos.z + SWIM_TERRAIN_WALL_CLEARANCE,
                    [&](float x, float y) { return terrainManager->getHeightAt(x, y); });
                stepPos.x = allowed.x;
                stepPos.y = allowed.y;
            }

            targetPos.x = stepPos.x;
            targetPos.y = stepPos.y;
        }

        grounded = false;
        } else {
        // Exiting water - boost upward to help climb onto shore/stairs.
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

        // Taking off: space, on something allowed to fly. It climbs from the
        // first frame rather than jumping first, which is what the key means
        // once a flying mount is under the player.
        if (flyingActive_ && !flightAirborne_ && f.spaceHeld) {
            flightAirborne_ = true;
            LOG_WARNING("Flight: took off");
        }

        // Player-controlled flight (flying mount / druid Flight Form):
        // Use 3D pitch-following movement with no gravity or grounding.
        if (flyingActive_ && flightAirborne_) {
            grounded = true;  // suppress fall-damage checks
            verticalVelocity = 0.0f;
            jumpBufferTimer = 0.0f;
            coyoteTimer = 0.0f;

            // Forward/back along the mount's facing at the flight pitch - see
            // flightForward. The camera's own look direction is not it.
            glm::vec3 flyFwd = f.flightForward;
            if (glm::dot(flyFwd, flyFwd) < 1e-8f) flyFwd = f.forward;
            const glm::vec3 flightFrom = targetPos;
            glm::vec3 flyMove(0.0f);
            if (f.nowForward)     flyMove += flyFwd;
            if (f.nowBackward)    flyMove -= flyFwd;
            if (f.nowStrafeLeft)  flyMove += f.right;
            if (f.nowStrafeRight) flyMove -= f.right;
            // Space = ascend, X = descend while airborne (works on foot too,
            // e.g. .gm fly, not just on a flying mount).
            bool flyDescend = !f.uiWantsKeyboard && f.xDown;
            if (f.spaceHeld)     flyMove.z += 1.0f;
            if (flyDescend)      flyMove.z -= 1.0f;
            float flyMoveLenSq = glm::dot(flyMove, flyMove);
            if (flyMoveLenSq > 1e-6f) {
                flyMove *= glm::inversesqrt(flyMoveLenSq);
                float flyFwdSpeed = (flightSpeedOverride_ > 0.0f && flightSpeedOverride_ < 200.0f
                                     && !std::isnan(flightSpeedOverride_))
                                        ? flightSpeedOverride_ : f.speed;
                float flyBackSpeed = (flightBackSpeedOverride_ > 0.0f && flightBackSpeedOverride_ < 200.0f
                                      && !std::isnan(flightBackSpeedOverride_))
                                         ? flightBackSpeedOverride_ : flyFwdSpeed * 0.5f;
                float flySpeed = (f.nowBackward && !f.nowForward) ? flyBackSpeed : flyFwdSpeed;
                targetPos += flyMove * flySpeed * f.physicsDeltaTime;
            }
            targetPos.z += verticalVelocity * f.physicsDeltaTime;

            // The ground holds a flying mount up - see flightStepAgainstGround.
            // Not where a WMO floor lies at the feet under the heightfield: that
            // is a tunnel burrowing under unholed ground, flown into, not over.
            if (terrainManager) {
                // Half a heightfield cell, as the slope reading in
                // sampleFloorUnderFeet uses, for the same reason.
                constexpr float kGroundNormalSpacing = 2.0f;
                const glm::vec3 slid = movement::flightStepAgainstGround(
                    [this](float x, float y) -> std::optional<float> {
                        if (terrainManager->isHoleAt(x, y)) return std::nullopt;
                        return terrainManager->getHeightAt(x, y);
                    },
                    flightFrom, targetPos, kGroundNormalSpacing);
                if (slid != targetPos &&
                    !(wmoRenderer && wmoRenderer->getFloorHeight(
                          targetPos.x, targetPos.y, targetPos.z + 1.5f))) {
                    targetPos = slid;
                }
            }
            // Skip all ground physics - go straight to collision/WMO sections
        } else {

        float moveLenSq = glm::dot(f.movement, f.movement);
        if (moveLenSq > 1e-6f) {
            f.movement *= glm::inversesqrt(moveLenSq);
            targetPos += f.movement * f.speed * f.physicsDeltaTime;
        }

        // Apply server-driven knockback horizontal velocity (decays over time).
        if (knockbackActive_) {
            targetPos.x += knockbackHorizVel_.x * f.physicsDeltaTime;
            targetPos.y += knockbackHorizVel_.y * f.physicsDeltaTime;
            // Exponential drag: reduce each frame so the player decelerates naturally.
            float drag = std::exp(-KNOCKBACK_HORIZ_DRAG * f.physicsDeltaTime);
            knockbackHorizVel_ *= drag;
            // Once negligible, clear the flag so collision/grounding work normally.
            if (glm::dot(knockbackHorizVel_, knockbackHorizVel_) < 0.0025f) {
                knockbackActive_ = false;
                knockbackHorizVel_ = glm::vec2(0.0f);
            }
        }

        // Jump with input buffering and coyote time
        if (f.nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
        if (grounded) coyoteTimer = COYOTE_TIME;

        bool canJump = (coyoteTimer > 0.0f) && (jumpBufferTimer > 0.0f) && !mounted_;
        if (canJump) {
            verticalVelocity = f.jumpVel;
            grounded = false;
            jumpBufferTimer = 0.0f;
            coyoteTimer = 0.0f;
        }

        jumpBufferTimer -= f.physicsDeltaTime;
        coyoteTimer -= f.physicsDeltaTime;

        // Apply gravity (skip when server has disabled f.gravity, e.g. Levitate spell)
        //
        // ...and skip it while the ground has not arrived. An unloaded tile and
        // a hole both answer nothing when asked for a height, and only one of
        // them should be fallen through. Entering the world is faster than
        // streaming the tile under it on a slow device, so without this the
        // character falls from the spawn point until the server kills it.
        if (seatedInChair_) {
            // In a chair, and the chair's height is the server's to decide.
            //
            // Gravity ran anyway: the character fell off the seat toward the
            // floor, the server put them back, and they fell again. In the
            // Undercity barber shop that is a bob of about a yard - the seat
            // is at -42.07, the floor under it at -43.02, and the server holds
            // the character at -42.73 between them. Reported as the view
            // bobbing up and down in the chair; it is the character bobbing,
            // and the camera only following.
            //
            // Grounding already stands down for a seat. Falling had to as
            // well, or standing the one down just leaves the other to do it.
            verticalVelocity = 0.0f;
        } else if (groundNotStreamedYet(targetPos.x, targetPos.y)) {
            verticalVelocity = 0.0f;
        } else if (gravityDisabled_) {
            // Float in place: bleed off any downward velocity, allow upward to decay slowly
            if (verticalVelocity < 0.0f) verticalVelocity = 0.0f;
            else verticalVelocity *= std::max(0.0f, 1.0f - 3.0f * f.physicsDeltaTime);
        } else {
            verticalVelocity += f.gravity * f.physicsDeltaTime;
            // Feather Fall / Slow Fall: cap downward terminal velocity to ~2 m/s
            if (featherFallActive_ && verticalVelocity < -2.0f)
                verticalVelocity = -2.0f;
        }
        targetPos.z += verticalVelocity * f.physicsDeltaTime;
        } // end ground physics (not airborne in flight)
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
    targetPos = sweepAgainstWalls(*followTarget, targetPos, true);

    return targetPos;
}

// Put the character on the floor, and publish where it ended up.
//
// The expensive half: terrain, WMO and doodad floor queries, the step-up
// budget, the slope limit, the cache that skips all of it when barely moving.
// Then the follow target is written, and a fall through the world is caught.
// What the floor queries answer directly under the character.
//
// The expensive part of grounding: three renderers asked where their surface
// is, two of them on other threads, and a cache that skips all of it when the
// character has barely moved. The three centre samples come back with the
// chosen floor because the recovery probes above need to know which surfaces
// existed at all, not just which one won.
CameraController::FloorSample CameraController::sampleFloorUnderFeet(const glm::vec3& targetPos,
                                                                     float stepUpBudget) {
    std::optional<float> groundH;
    std::optional<float> centerTerrainH;
    std::optional<float> centerWmoH;
    std::optional<float> centerM2H;
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
        // Full collision check - run terrain/WMO/M2 queries in parallel
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
            // Closest to the feet, not the highest below the probe.
            //
            // Under an overhang - the doorways and portals of
            // Undercity - the floor of the level above sits about a
            // metre over the one you stand on, both inside the
            // step-up window. The default highest-floor pick snapped
            // the player up to the level above even standing still,
            // the server pulled them back to the real floor, and the
            // two fought once a second. Anchoring the pick to the
            // player's own ground keeps them where they are; a real
            // step onto a ledge still wins once the feet are level
            // with it.
            //
            // The reference is the feet themselves, NOT wmoBaseZ:
            // wmoBaseZ is max(z, lastGroundZ) so the probe can reach
            // up for a step, but feeding that same raised value back
            // as the arbitration anchor ratchets the pick upward -
            // one stray upper-floor frame lifts lastGroundZ, and the
            // reference then stays on the level above even after the
            // server drags the player back down, which is the very
            // yo-yo this pick was meant to stop. Grounded, the true
            // feet are targetPos.z; airborne, aim at the last ground.
            //
            // Except that a player cannot be standing on a floor while
            // their feet are a long way under it, and the flag says
            // grounded anyway. At the Undercity elevator ramp the feet
            // were 2.81 below the landing they had been standing on -
            // still a candidate, at -41.4273 against a last ground of
            // -41.4245 - and 2.39 above the Trade Quarter floor five
            // yards further down. Measured from the feet the lower one
            // is nearer by 0.42, so the pick took it and the player
            // dropped through the ramp.
            //
            // Below the last ground by more than a step is falling,
            // whatever the flag says, so anchor where the airborne case
            // already anchors: the ground they were last on. That floor
            // then wins by 5.2 instead of losing by 0.42. It cannot pull
            // anyone up onto a floor they truly left, because a floor
            // they have walked off is not a candidate at their feet.
            const float feetRef = movement::floorArbitrationAnchor(
                grounded, targetPos.z, lastGroundZ);
            wmoFuture = core::ThreadPool::frameWorkers().submit(
                [this, px, py, wmoProbeZ, feetRef]() -> FloorResult {
                    float nz = 1.0f;
                    auto h = wmoRenderer->getFloorHeight(px, py, wmoProbeZ, &nz, feetRef);
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
            // ...and how steep it is there, which the height alone cannot say.
            // Sampled at a third of a yard: wide enough not to read the
            // interpolation inside one heightfield cell as a cliff, narrow
            // enough to still see the face of one.
            if (terrainH) {
                // Half a heightfield cell, so the difference measures the
                // slope of the ground rather than the seam between two of the
                // triangles it is built from.
                //
                // This was a third of a yard, chosen when the floor query
                // interpolated the four corners of a cell and was smooth
                // across it. It samples the real surface now - four triangles
                // fanned from the cell's centre vertex - and at that spacing
                // the two samples routinely land in different wedges, so the
                // difference reads the crease between them and calls a walkable
                // hillside a cliff. A cell is 4.17 yards; a real cliff spans
                // several of them and is still seen.
                constexpr float kSlopeSampleSpacing = 2.0f;
                const float nz = movement::heightfieldNormalZ(
                    [this](float sx, float sy) {
                        return terrainManager->getHeightAt(sx, sy);
                    },
                    targetPos.x, targetPos.y, kSlopeSampleSpacing);
                // The slope limit does NOT remove the ground.
                //
                // It used to: too steep, and terrainH was cleared. But a floor
                // that is not there is not a wall - the player does not fail to
                // climb the hill, they fall through it and keep going, which is
                // how this was reported. Whatever the limit is worth, it is not
                // worth dropping someone out of the world.
                //
                // A slope limit belongs on the movement: refuse the step onto
                // ground too steep to stand on, and leave the ground where it
                // is. Until that is written, a steep hillside is climbable
                // again, which is what it was before the limit was added and is
                // the lesser of the two faults.
                (void)nz;
            }
        }
        if (wmoAsync) {
            try { auto [h, nz] = wmoFuture.get(); wmoH = h; wmoNormalZ = nz; }
            catch (const std::exception& e) { LOG_ERROR("WMO floor query: ", e.what()); }
        }
        if (m2Async) {
            try {
                auto [h, nz] = m2Future.get();
                m2H = h;
                if (m2H && !ignoreSlopeLimit_ && nz < MIN_WALKABLE_NORMAL_M2) {
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
        // streets must not steal the player from the terrain above them -
        // that pulled players through the ground at the Stormwind gate
        // ramparts and down ramps into the void. Inside an interior WMO
        // group (tram entrance buildings) the heightfield under the city
        // is meaningless, so it gets no veto - otherwise entry becomes
        // dependent on approach angle.
        if (atTunnelSeam && !cachedInsideInteriorWMO && terrainManager) {
            glm::vec3 moveDir = targetPos - lastCollisionCheckPos_;
            moveDir.z = 0.0f;
            const float moveLen = glm::length(moveDir);
            if (moveLen < 1e-3f) {
                atTunnelSeam = false;  // stationary - nothing to enter
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
        if (wmoH && !ignoreSlopeLimit_ && wmoNormalZ < minWalkableWmo) {
            wmoH = std::nullopt;  // Treat as unwalkable
        }

        // Reject WMO floors far above last known ground when airborne
        // (prevents snapping to roof/ceiling surfaces during jumps)
        if (wmoH && !grounded && *wmoH > lastGroundZ + stepUpBudget + 0.5f) {
            wmoH = std::nullopt;
            centerWmoH = std::nullopt;
        }
        // An M2 doodad's collision sitting well below a valid WMO
        // floor is BENEATH that floor - a decoration or structural
        // base under the walkway, not a surface to stand on. Letting
        // it win drops the player through the WMO floor: in Undercity
        // the pick took an m2 floor ~6m below the wmo one (feet -48.8,
        // wmo -48.15, m2 -54.23) and fell. When a WMO floor is present
        // here, reject an m2 floor more than 1.5m below it. (If the
        // player were standing ON the m2 platform, the higher WMO
        // floor would be above the probe and not returned, so this
        // cannot strand them off a legitimate lower deck.)
        if (m2H && wmoH && *m2H < *wmoH - 1.5f) {
            m2H = std::nullopt;
        }
        // A terrain hole is the artist saying there is no ground
        // here - it is how cave mouths and sunken entrances get
        // opened up, and the mesh builder already skips those
        // quads, so nothing is drawn over them. getHeightAt does
        // not ask: it interpolates straight across the gap and
        // reports a surface right at the player's feet, which then
        // wins the floor selection against the real floor below.
        // At Gadgetzan's auction house that put the player out over
        // the stairwell on ground that is not there, unable to walk
        // down the steps and looking into a hole they were standing
        // on. Every seam guard below is downstream of this and none
        // of them could see it, because to them the terrain sample
        // was real.
        //
        // Only ever hand the floor to the building underneath - a
        // hole with nothing below it keeps the heightfield it has
        // always had, so this cannot open a pit anywhere new.
        if (terrainH && wmoH && terrainManager &&
            terrainManager->isHoleAt(targetPos.x, targetPos.y)) {
            terrainH = std::nullopt;
        }

        // Inside an interior WMO group - Undercity's halls, a
        // building's rooms - the outdoor heightfield is the roof far
        // overhead, never the floor. Veto it there so a brief gap in
        // the WMO floor query does not kick the player up to the
        // surface. (The seam case is handled above and keeps
        // cachedInsideInteriorWMO false at entrances.)
        //
        // Only when it really is overhead, though. isInsideInteriorWMO
        // is a bounding-box containment test, and an underground WMO's
        // interior box reaches up through the ground above it - so
        // standing on the grass over a cave counted as being inside
        // it. Vetoing terrain there left the WMO as the only
        // candidate, and the nearest WMO surface below is the cave's
        // ceiling: the player dropped through the hillside they were
        // walking on and stood on the roof of the room underneath.
        //
        // Higher than the player could step onto is the test, because
        // that is the whole of what the veto meant: ground you cannot
        // reach is not the ground you are standing on. Undercity's
        // surface sits ~113m over the halls and is still vetoed; the
        // hillside at your feet is not. The rule itself lives in
        // movement_limits.hpp, where it is pinned by a test.
        if (terrainH && movement::terrainIsOverheadRoof(
                cachedInsideInteriorWMO, *terrainH,
                targetPos.z, stepUpBudget)) {
            terrainH = std::nullopt;
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
            // The terrain wins at a seam unless it is not really there.
            //
            // Two ways it can fail to be. The quad is cut by a hole, which is
            // how the map opens a cave mouth or a below-ground entrance - that
            // is vetoed further up, before any of this, so there is simply no
            // terrain candidate left here. Or it is standing overhead out of
            // reach, which is what the heightfield is once the player is
            // inside a passage burrowing under unholed ground: taking it would
            // lift them back out. See wmoFloorIsWayIn.
            //
            // Anything else is real ground under real feet, and a WMO floor
            // that happens to run beneath it is not a way in. Orgrimmar's
            // valley entrance is the case: its ramp passes about 1.3 yards
            // under the ground beside it, and there is no hole anywhere in
            // that tile.
            const bool preferWmoAtSeam =
                atTunnelSeam &&
                (!terrainH ||
                 movement::wmoFloorIsWayIn(*terrainH, targetPos.z, stepUpBudget));
            if (preferWmoAtSeam) {
                groundH = wmoH;
            } else if (terrainH) {
                // Both real and both reachable: whichever is actually under the
                // feet, rather than oscillating between the ground above and a
                // deep WMO floor below.
                groundH = selectClosestFloor(terrainH, wmoH, targetPos.z);
            } else {
                groundH = selectReachableFloor3(terrainH, wmoH, m2H, targetPos.z, stepUpBudget);
            }
        } else {
            groundH = selectReachableFloor3(terrainH, wmoH, m2H, targetPos.z, stepUpBudget);
        }

        // Nothing to stand on at all, with a building overhead.
        //
        // The jump dump below only fires when a floor is chosen and it is a
        // long way from the last one. Stepping onto an Undercity lift produces
        // no floor whatsoever - the deck is an M2 transport and the shaft has
        // no WMO floor under it - so the player simply falls, and the one dump
        // that would name the deck never ran. UndeadElevator ships 68
        // collision triangles and is not marked skipCollision, so what is
        // wanted here is which test the instance fails, or whether it is a
        // candidate at all.
        if (!groundH && m2Renderer) {
            static std::chrono::steady_clock::time_point lastNoFloorDump{};
            const auto nowNoFloor = std::chrono::steady_clock::now();
            if (nowNoFloor - lastNoFloorDump > std::chrono::seconds(5)) {
                lastNoFloorDump = nowNoFloor;
                // The centre pick's answer, not the final one: the recovery
                // passes in groundFollowedCharacter run after this and often
                // find a floor the one ray missed. Said plainly because the
                // first wording read as "the player is over a void" and is
                // what sent this hunt after the wrong thing.
                LOG_WARNING("Centre floor probe found nothing (feet ", targetPos.z,
                            ") - the recovery passes have not run yet");
                if (floorDiagEnabled()) {
                    if (wmoRenderer) {
                        wmoRenderer->debugDumpGroupsAtPosition(
                            targetPos.x, targetPos.y, targetPos.z);
                    }
                    m2Renderer->debugDumpFloorCandidatesAt(
                        targetPos.x, targetPos.y, targetPos.z);
                }
            }
        }

        // Just arrived, and the buildings have not.
        //
        // On entering a world the heightfield is there before the WMOs are, and
        // over anything that runs underground it lies well above where the
        // player actually is. Logging out in the Undercity entrance tunnel and
        // back in put them on its roof, and once the tunnel loaded its roof was
        // a perfectly good floor to keep them there. Nothing far above the
        // position the server gave is their floor while that is still true.
        //
        // Generous, because the server's Z and the floor under it disagree by a
        // little at the best of times; two yards admits that and refuses forty.
        if (entryFloorAnchorTimer_ > 0.0f && groundH &&
            *groundH > entryFloorAnchorZ_ + 2.0f) {
            groundH = std::nullopt;
        }

        // A storey down, with the floor still under the next step.
        //
        // One ray at the character's centre misses the lip of a surface, and
        // the answer it comes back with is not "no floor" but whatever lies at
        // the bottom of the room - at the top of the Undercity lift that is the
        // shaft floor, ninety-six yards down. groundFollowedCharacter samples a
        // foot cross afterwards and recovers, which is why this reads as almost
        // falling rather than falling, but the frame in between is a dip the
        // player sees.
        //
        // So before handing back a drop of more than a few yards, ask the four
        // points around the feet. Only on that frame, so the hot path still
        // casts one ray. A drop nothing around the feet can account for is a
        // real one and is passed through untouched - walking off a ledge still
        // works.
        if (groundH && grounded && wmoRenderer &&
            *groundH < lastGroundZ - 4.0f && targetPos.z > lastGroundZ - 4.0f) {
            const float lipProbeZ = std::max(targetPos.z, lastGroundZ) + stepUpBudget + 0.6f;
            auto lip = highestWalkableFloor(
                [this](float x, float y, float z, float* nz) {
                    return wmoRenderer->getFloorHeight(x, y, z, nz);
                },
                targetPos.x, targetPos.y, feetCross(0.35f), lipProbeZ,
                MIN_WALKABLE_NORMAL_WMO,
                {.minZ = lastGroundZ - 3.5f, .maxZ = targetPos.z + stepUpBudget});
            if (lip) groundH = lip;
        }

        // The local player's own floor pick, named when it jumps.
        //
        // The movingEntityFloor log covers creatures and other
        // players; this is the reporter's own character in
        // Undercity. It says which floor was chosen for the player
        // and the three candidates it chose among, so a jump that
        // survived the closest-to-feet change shows what is still
        // competing - and whether both floors are even offered here
        // or only one is, which decides whether the fix belongs in
        // the selection or deeper in the WMO query.
        // Threshold sits below a floor gap (~1m) on purpose: the
        // ratchet oscillation the feetRef fix targets stays under a
        // metre frame to frame, so a 1.0 gate saw nothing while the
        // player visibly bobbed. 0.4 catches the sub-metre bob.
        if (groundH && std::abs(*groundH - lastGroundZ) > 0.4f) {
            static std::chrono::steady_clock::time_point lastPlayerFloorLog{};
            const auto now = std::chrono::steady_clock::now();
            if (now - lastPlayerFloorLog > std::chrono::seconds(1)) {
                lastPlayerFloorLog = now;
                core::Logger::getInstance().warning(
                    "Player floor jump: feet=", targetPos.z,
                    " lastGround=", lastGroundZ, " -> chose ", *groundH,
                    " grounded=", grounded ? 1 : 0,
                    " (terrain=", terrainH ? *terrainH : -99999.0f,
                    " wmo=", wmoH ? *wmoH : -99999.0f,
                    " m2=", m2H ? *m2H : -99999.0f,
                    " seam=", atTunnelSeam ? 1 : 0, ")");
            }
            // A jump - the floor query landed a level away, not a
            // step - is the Undercity pull, and the one thing needed
            // to solve it is which WMO groups and floors exist under
            // the player when it happens. That is exactly what F8's
            // dump prints, so dump it here automatically on a jump
            // (rate-limited hard, it is verbose): the log then carries
            // the answer from ordinary play, with no key to remember.
            // Threshold matches the line above it, not a metre.
            //
            // 0.9 was chosen for a pull that crossed an overhang, and it
            // never fired for the one actually reported in Undercity: the
            // sink there is 0.41, 0.60, 0.56, 0.40 - the thickness of the
            // floor slab, the player dropping onto the ceiling of the room
            // beneath - and every one of those passed under the gate with
            // the jump line printed and no group data behind it. Whatever
            // is worth naming is worth dumping; the five second limiter is
            // what keeps it cheap, not the size of the drop.
            if (wmoRenderer && std::abs(*groundH - lastGroundZ) > 0.35f) {
                static std::chrono::steady_clock::time_point lastFloorDump{};
                if (floorDiagEnabled() && now - lastFloorDump > std::chrono::seconds(5)) {
                    lastFloorDump = now;
                    core::Logger::getInstance().warning(
                        "Player floor jump: dumping WMO groups at the "
                        "pull (feet ", targetPos.z, " -> ", *groundH, ")");
                    wmoRenderer->debugDumpGroupsAtPosition(
                        targetPos.x, targetPos.y, targetPos.z);
                    // And the doodads, which is where the Undercity pull
                    // actually lives: every one of these jumps happens while
                    // the player is attached to a lift, and m2 comes back
                    // empty each time even though UndeadElevator ships 68
                    // collision triangles. The WMO dump alone could never say
                    // why, because the deck it is looking for is not a WMO.
                    if (m2Renderer) {
                        m2Renderer->debugDumpFloorCandidatesAt(
                            targetPos.x, targetPos.y, targetPos.z);
                    }
                }
            }
        }

        // Update cache
        lastCollisionCheckPos_ = targetPos;
        if (groundH) {
            cachedFloorHeight_ = *groundH;
            hasCachedFloor_ = true;
            // Ground found - cancel gravity suspension (WMO floor loaded)
            if (gravitySuspendTimer_ > 0.0f) gravitySuspendTimer_ = 0.0f;
        } else {
            hasCachedFloor_ = false;
        }
    }
    return {.floor = groundH, .terrain = centerTerrainH, .wmo = centerWmoH, .m2 = centerM2H};
}

void CameraController::groundFollowedCharacter(float deltaTime, FrameInput& f,
                                               glm::vec3& targetPos,
                                               const glm::vec3& prevTargetPos) {
    // Seated: the server placed the character, and this pass would place them
    // again from whatever is underneath.
    //
    // A chair is an M2 with collision whose seat is about a metre off the
    // floor and nothing here can tell a seat from a step, so the feet went on
    // top of the chair and the character sat in the air above it. Dropping the
    // chair from the floor candidates was not enough: it only applied to the
    // stand states that name a chair, and a server that answers a chair with a
    // plain SIT - which is what the inn's chairs did - left the whole rule
    // switched off while the pose said otherwise.
    //
    // Nothing drifts while this stands down: movement is blocked for as long
    // as the character is seated, and the moment they stand up the pass runs
    // again from wherever the server left them.
    //
    // One thing is still enforced: the floor. Standing the pass down entirely
    // left nothing to catch a seat that ends up under the ground - a tile that
    // streamed in after the sit, a server seating the character at a Z the
    // floor here does not agree with - and a character below the floor is a
    // character who falls through it when they stand. So the floor under the
    // feet is still sampled and used one way only, upward, and never from the
    // chair's own M2, which is what standing on the furniture was.
    if (sitting) {
        const FloorSample seated = sampleFloorUnderFeet(targetPos, movement::kMaxStepUp);
        const std::optional<float> under = selectReachableFloor3(
            seated.terrain, seated.wmo, std::nullopt, targetPos.z, movement::kMaxStepUp);
        if (under && targetPos.z < *under) {
            targetPos.z = *under;
            lastGroundZ = *under;
            hasRealGround_ = true;
            grounded = true;
            verticalVelocity = 0.0f;
        }
        return;
    }

    // Ground the character to terrain or WMO floor
    // Skip entirely while swimming - the swim floor clamp handles vertical bounds.
    if (!swimming) {
        float stepUpBudget = grounded ? movement::kMaxStepUp : 1.2f;
        // 1. Center-only sample for terrain/WMO floor selection.
        //    Using only the center prevents tunnel entrances from snapping
        //    to terrain when offset samples miss the WMO floor geometry.
        // Slope limit: reject surfaces too steep to walk (prevent clipping).
        // WMO tunnel/bridge ramps are often steeper than outdoor terrain ramps.
        // The floor directly under the feet, and the surfaces that answered.
        FloorSample sample = sampleFloorUnderFeet(targetPos, stepUpBudget);
        std::optional<float> groundH = sample.floor;
        const std::optional<float> centerTerrainH = sample.terrain;
        const std::optional<float> centerWmoH = sample.wmo;
        const std::optional<float> centerM2H = sample.m2;

        // Transition safety: if no reachable floor was selected, choose the higher
        // of terrain/WMO center surfaces when it is still near the player.
        // This avoids dropping into void gaps at terrain<->WMO seams.
        // Whether there is structure here at all, which gates every recovery
        // below it - the multi-sample pass, the wider rescue rings, all of it.
        //
        // It rested on a ray at the character's centre, and a centre ray is the
        // one thing that misses the lip of a surface: at the top of the
        // Undercity lift the landing lies a third of a yard away in +y at the
        // height of the feet, the centre finds nothing, so this said there is
        // no structure nearby and shut off the passes that would have found it.
        // The player steps off the deck and dips into the gap.
        //
        // The hint asks the foot cross now, the same four points the pass it
        // guards would use. It is the last thing standing between the player
        // and a fall, so it should not fail for the reason those passes exist.
        const bool nearWmoSpace = cachedInsideWMO || centerWmoH.has_value();
        bool nearStructureSpace = nearWmoSpace || centerM2H.has_value();
        if (!nearStructureSpace && hasRealGround_) {
            const auto nearLastGround = [&](std::optional<float> h) {
                return h && std::abs(*h - lastGroundZ) <= 2.0f;
            };
            if (wmoRenderer) {
                if (nearLastGround(highestWalkableFloor(
                        [this](float x, float y, float z, float* nz) {
                            return wmoRenderer->getFloorHeight(x, y, z, nz);
                        },
                        targetPos.x, targetPos.y, feetCross(0.35f), lastGroundZ + 1.5f,
                        MIN_WALKABLE_NORMAL_WMO,
                        {.minZ = lastGroundZ - 2.0f, .maxZ = lastGroundZ + 2.0f}))) {
                    nearStructureSpace = true;
                }
            }
            if (!nearStructureSpace && m2Renderer && !externalFollow_) {
                if (nearLastGround(highestWalkableFloor(
                        [this](float x, float y, float z, float* nz) {
                            return m2Renderer->getFloorHeight(x, y, z, nz);
                        },
                        targetPos.x, targetPos.y, feetCross(0.35f), lastGroundZ + 1.5f,
                        MIN_WALKABLE_NORMAL_M2,
                        {.minZ = lastGroundZ - 2.0f, .maxZ = lastGroundZ + 2.0f}))) {
                    nearStructureSpace = true;
                }
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
            // Not while standing inside a building. Terrain is the ground
            // *outside*, and a staircase descending below it - Gadgetzan's
            // auction house, any cellar - is a legitimate floor far under
            // nearby terrain, which is exactly what this refuses. The
            // player hovered over the steps and sank at the clamp's rate
            // instead of walking down them.
            //
            // The interior test above is not enough on its own: it wants
            // the point a metre over the player's head inside a group
            // flagged 0x2000, and at the top of a stairwell that group's
            // box has not started yet. Being inside the WMO at all is the
            // question this guard should have asked.
            if (centerTerrainH && !cachedInsideWMO) {
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
        if (groundH && hasRealGround_ && nearStructureSpace && !f.nowJump) {
            float dropFromLast = lastGroundZ - *groundH;
            // Only reject the lower sample while the feet are still up near
            // lastGroundZ - that is the transient bad-ramp case this guard is
            // for. Once the feet have actually descended below the clamp
            // target the player is standing on the lower floor, and pinning
            // groundH back up to lastGroundZ leaves the ground reference
            // hanging above the feet so gravity and the snap fight over the
            // gap forever. That is the Undercity overhang yo-yo: walk under
            // the level above, lastGroundZ stays stuck on it (-43) while the
            // real floor (-47) is refused, and the feet bob in between. If
            // the feet are already past the target, let the real floor
            // through so lastGroundZ can follow the player down onto it.
            if (dropFromLast > 1.0f && verticalVelocity > -6.0f &&
                targetPos.z > lastGroundZ - 0.20f) {
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
            const float wmoMultiBaseZ = grounded ? std::max(targetPos.z, lastGroundZ) : lastGroundZ;
            const float wmoProbeZ = wmoMultiBaseZ + stepUpBudget + 0.6f;
            const float minWalkableWmo =
                cachedInsideWMO ? MIN_WALKABLE_NORMAL_WMO : MIN_WALKABLE_NORMAL_TERRAIN;
            // Airborne, anything above the last known ground is a roof rather
            // than a step, so the ceiling of the band comes down.
            float maxZ = targetPos.z + stepUpBudget;
            if (!grounded) maxZ = std::min(maxZ, lastGroundZ + stepUpBudget + 0.5f);

            auto wh = highestWalkableFloor(
                [this](float x, float y, float z, float* nz) {
                    return wmoRenderer->getFloorHeight(x, y, z, nz);
                },
                targetPos.x, targetPos.y, feetCross(0.35f), wmoProbeZ, minWalkableWmo,
                {.minZ = lastGroundZ - 3.5f, .maxZ = maxZ});
            if (wh && (!groundH || *wh > *groundH)) groundH = wh;
        }

        // WMO recovery probe: when no floor is found while descending, do a wider
        // footprint sample around the player to catch narrow plank/stair misses.
        if (!groundH && wmoRenderer && hasRealGround_ && verticalVelocity <= 0.0f) {
            const float rescueProbeZ = std::max(lastGroundZ, targetPos.z) + stepUpBudget + 1.2f;
            auto rescueFloor = highestWalkableFloor(
                [this](float x, float y, float z, float* nz) {
                    return wmoRenderer->getFloorHeight(x, y, z, nz);
                },
                targetPos.x, targetPos.y, feetRing(0.65f), rescueProbeZ,
                MIN_WALKABLE_NORMAL_WMO,
                {.minZ = lastGroundZ - 6.0f, .maxZ = lastGroundZ + stepUpBudget + 0.75f});
            if (rescueFloor) groundH = rescueFloor;
        }

        // M2 recovery probe: Booty Bay-style wooden platforms can be represented
        // as M2 collision where center probes intermittently miss.
        if (!groundH && m2Renderer && !externalFollow_ && hasRealGround_ && verticalVelocity <= 0.0f) {
            const float rescueProbeZ = std::max(lastGroundZ, targetPos.z) + stepUpBudget + 1.4f;
            auto rescueFloor = highestWalkableFloor(
                [this](float x, float y, float z, float* nz) {
                    return m2Renderer->getFloorHeight(x, y, z, nz);
                },
                targetPos.x, targetPos.y, feetRing(0.75f), rescueProbeZ,
                MIN_WALKABLE_NORMAL_M2,
                {.minZ = lastGroundZ - 6.0f, .maxZ = lastGroundZ + stepUpBudget + 0.90f});
            if (rescueFloor) groundH = rescueFloor;
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

        // 2. Multi-sample for M2 objects (rugs, planks, bridges, ships) -
        //    these are narrow and need offset probes to detect reliably.
        if (m2Renderer && !externalFollow_) {
            // Not highestWalkableFloor: this one prefers an M2 floor even when
            // it is slightly lower than the terrain, so a ship's deck wins over
            // the water under it.
            const auto offsets = feetRing(0.6f);
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
        // - a steep hill, or an ordinary one crossed in a long frame - got the
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

        // In flight as well: flightStepAgainstGround keeps a mount from flying
        // into a hill, and this is what brings one back out that is already in
        // it.
        if (!swimming && !hoverActive_ && !externalFollow_ &&
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
                // heightfield - the Darkshire crypts, the tunnel under the hill
                // into Booty Bay, any cave - is a structure sitting under the
                // terrain at this column. Probe for one directly instead of
                // trusting cachedInsideWMO and centerWmoH: the first lags by
                // design, and the second is cleared outright for floors more
                // than 12 yards below terrain while containment still reads
                // false, which is exactly what descending a tunnel mouth looks
                // like. Finding anything at all here means hands off - the
                // player belongs under the terrain, and lifting them out would
                // put them on the hillside above the entrance they just walked
                // into.
                // Probed from the player, not from the terrain surface: the
                // floor query culls any group whose top sits more than 4 yards
                // below the probe height, so probing from up at the heightfield
                // would skip the crypt or the tunnel entirely and report clear
                // ground - the exact opposite of the truth. From here the
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
                // rescue would push up to is not there at all - and the
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

        // On the ground under a building's floor.
        //
        // A raised floor sits a yard or so over the heightfield it is built on,
        // which is more than a step. Once the feet are down on that ground -
        // one frame at the doorway where the floor query misses the threshold
        // is enough - the floor is out of reach of the step-up, so it is never
        // chosen again, and the player walks about inside the building sunk to
        // the chest in its floor. Reported entering a house at an angle.
        //
        // Nowhere legitimate to stand has a walkable floor across the whole
        // footprint at waist height: the body would be inside it. Put the feet
        // back on it. The centre is asked first, so the four around it cost
        // nothing on the frames that matter least - every one where the answer
        // is no.
        if (!isFlightAirborne() && !hoverActive_ && !externalFollow_ && wmoRenderer &&
            nearWmoSpace && groundH && centerTerrainH && *groundH == *centerTerrainH) {
            constexpr float kWaistHeight = 1.8f;
            const float feetZ = targetPos.z;
            const auto slabAt = [&](const glm::vec2& o) -> std::optional<float> {
                float nz = 1.0f;
                const auto h = wmoRenderer->getFloorHeight(
                    targetPos.x + o.x, targetPos.y + o.y, feetZ + kWaistHeight, &nz);
                if (!h || nz < MIN_WALKABLE_NORMAL_WMO) return std::nullopt;
                if (*h <= feetZ + stepUpBudget || *h >= feetZ + kWaistHeight) return std::nullopt;
                return h;
            };
            const auto cross = feetCross(0.35f);
            std::optional<float> slab = slabAt(cross[0]);
            for (std::size_t i = 1; slab && i < cross.size(); ++i) {
                if (!slabAt(cross[i])) slab = std::nullopt;
            }
            if (slab) {
                LOG_WARNING("Under a floor: feet ", feetZ, " on the ground at (", targetPos.x,
                            ", ", targetPos.y, ") with a WMO floor at ", *slab,
                            " across the footprint - back onto it");
                targetPos.z = *slab;
                groundH = slab;
                lastGroundZ = *slab;
                verticalVelocity = 0.0f;
            }
        }

        // WOWEE_FLOOR_DEBUG=1 - what every floor query answered and which
        // one won, four times a second.
        //
        // Two fixes for one report of not being able to walk down a
        // staircase were both aimed at the wrong thing, because from the
        // outside "standing on nothing" looks the same whether the floor
        // below was never found, was found and rejected, or was found and
        // lost a selection. Those are different bugs in different places
        // and this is the line that tells them apart.
        {
            static constexpr const char* kFloorDebugPath = "/tmp/wowee_floor.log";
            static const bool kFloorDebug = [] {
                const char* v = std::getenv("WOWEE_FLOOR_DEBUG");
                return v && v[0] && v[0] != '0';
            }();
            if (kFloorDebug) {
                floorDebugTimer_ += deltaTime;
                if (floorDebugTimer_ >= 0.25f) {
                    floorDebugTimer_ = 0.0f;
                    auto say = [](const std::optional<float>& v) {
                        return v ? std::to_string(*v) : std::string("-");
                    };
                    std::ostringstream line;
                    line << "FLOOR at (" << targetPos.x << ", " << targetPos.y
                         << ", " << targetPos.z << ")"
                         << " terrain=" << say(centerTerrainH)
                         << " wmo="     << say(centerWmoH)
                         << " m2="      << say(centerM2H)
                         << " chose="   << say(groundH)
                         << " hole="    << (terrainManager &&
                                terrainManager->isHoleAt(targetPos.x, targetPos.y))
                         << " insideWMO="      << cachedInsideWMO
                         << " insideInterior=" << cachedInsideInteriorWMO
                         << " grounded="       << grounded
                         << " vz="             << verticalVelocity
                         << " lastGroundZ="    << lastGroundZ;
                    LOG_WARNING(line.str());
                    // Also to a file. The console is where this used to go
                    // and it scrolled past - asking someone to reproduce a
                    // bug and then hand back terminal scrollback is asking
                    // twice. Truncated on the first line of a run so the
                    // file is always this session's.
                    static std::ofstream trace(kFloorDebugPath,
                                               std::ios::out | std::ios::trunc);
                    if (trace) trace << line.str() << '\n' << std::flush;
                }
            }
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
                std::max(0.5f, std::abs(verticalVelocity) * f.physicsDeltaTime * 2.0f));
            bool airFalling = (!grounded && verticalVelocity < -5.0f
                               && dz >= -airSnapRange);
            // Not while climbing in flight: a frame's climb at a high frame
            // rate is less than the quarter yard this reaches down, so taking
            // off was pulled back to the ground every frame and never left it.
            bool slopeGrace = (grounded && verticalVelocity > -1.0f &&
                               dz >= -0.25f && dz <= stepUp * 1.5f) &&
                              !(isFlightAirborne() && f.spaceHeld);

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
                noGroundTimer_ += f.physicsDeltaTime;

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
                } else if (nearStructureSpace && noGroundTimer_ < 1.20f && dropFromLastGround <= 4.0f && !f.nowJump) {
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
        // Player is safely on real geometry - save periodically
        continuousFallTime_ = 0.0f;
        autoUnstuckFired_ = false;
        safePosSaveTimer_ += f.physicsDeltaTime;
        if (safePosSaveTimer_ >= SAFE_POS_SAVE_INTERVAL) {
            safePosSaveTimer_ = 0.0f;
            lastSafePos_ = targetPos;
            hasLastSafe_ = true;
        }
    } else if (!grounded && !swimming && !externalFollow_) {
        // Falling (or standing on nothing past grace period) - accumulate fall time
        continuousFallTime_ += f.physicsDeltaTime;
        if (continuousFallTime_ >= AUTO_UNSTUCK_FALL_TIME && !autoUnstuckFired_) {
            autoUnstuckFired_ = true;
            if (autoUnstuckCallback_) {
                autoUnstuckCallback_();
            }
        }
    }
}

// The camera itself: where it sits relative to the character it follows.
//
// Pivot at the neck, zoom smoothing, the raycast that pulls the camera in when
// a wall is behind it, the floor clearance that stops it sinking through a
// tunnel roof, and the first-person threshold where the character is hidden.
void CameraController::updateOrbitCamera(float deltaTime, FrameInput& f,
                                         const glm::vec3& targetPos) {
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
            glm::vec3 probeCam = targetPos + (-f.forward3D) * currentDistance;
            auto terrainAtCam = terrainManager->getHeightAt(probeCam.x, probeCam.y);
            auto terrainAtPivot = terrainManager->getHeightAt(targetPos.x, targetPos.y);

            float desiredLift = 0.0f;
            if (terrainAtCam) {
                // Keep the camera high enough that its ray does not cut
                // through a hill behind the character.
                //
                // Measured at the camera, which is the thing that has to
                // clear the ground. It was measured at the pivot, against a
                // fixed two metres above the terrain - and an absolute height
                // undoes any attempt to place the pivot on the character, by
                // exactly the amount that attempt moved it. On flat ground a
                // gnome's pivot at 0.65 was lifted 1.35 and a human's at 1.60
                // was lifted 0.40, both landing on 2.00 above the feet. Every
                // race got the same pivot, every time, and which character was
                // being followed made no difference to it.
                //
                // That is why zooming in on a gnome put them at the bottom of
                // the frame: the camera was aiming at a point nearly twice
                // their height, on level ground, with no hill in sight.
                const float basePivotZ = targetPos.z + followedPivotHeight() + mountedOffset;
                // The camera sits behind the character along the look
                // direction, which carries the pitch with it.
                desiredLift = terrainPivotLift(basePivotZ, (-f.forward3D).z,
                                               currentDistance, *terrainAtCam);
            }
            // If character is already below local terrain sample, avoid lifting aggressively.
            if (terrainAtPivot && targetPos.z < *terrainAtPivot - 0.2f) {
                desiredLift = 0.0f;
            }
            // Said when it starts and when it stops, because a lift is the
            // camera being moved by the ground rather than by the player and
            // there was no way to tell the two apart from outside. This one
            // was applying on flat ground in every session ever played.
            const bool lifting = desiredLift > 0.01f;
            if (lifting != pivotLiftActive_) {
                pivotLiftActive_ = lifting;
                LOG_WARNING("Camera pivot: terrain lift ",
                            lifting ? "on, raising the pivot by " : "off, was ",
                            desiredLift, " (pivot ", followedPivotHeight(),
                            " above the feet, ground at the camera ",
                            terrainAtCam ? *terrainAtCam - targetPos.z : 0.0f,
                            " relative to them)");
            }
            cachedPivotLift_ = desiredLift;
        }
        pivotLift = cachedPivotLift_;
    } else if (cachedInsideInteriorWMO) {
        // Inside WMO volumes (including tunnel/cave shells): terrain-above samples
        // are not relevant for camera pivoting.
        cachedPivotLift_ = 0.0f;
    }
    glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, followedPivotHeight() + mountedOffset + pivotLift);

    // Camera direction from yaw/pitch (already computed as forward3D)
    glm::vec3 camDir = -f.forward3D;  // Camera looks at pivot, so it's behind

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

        // Pulled in indoors, and let back out again.
        //
        // This is the same containment test IsIndoors answers with - the one a
        // macro's [indoors] reads and the one that says a mount is not allowed
        // here - asked at the player rather than at the camera. Indoors the
        // camera is forever being pushed off a wall and let back out, and that
        // is what turns a corridor into a bout of motion sickness.
        if (cachedInsideWMO) {
            if (!indoorZoomHeld_) {
                indoorZoomHeld_ = true;
                outdoorTargetDistance_ = userTargetDistance;
            }
            if (userTargetDistance > MAX_DISTANCE_INTERIOR) {
                userTargetDistance = MAX_DISTANCE_INTERIOR;
            }
        } else if (indoorZoomHeld_) {
            indoorZoomHeld_ = false;
            // Given back only if they have not chosen something closer while
            // they were inside: their own wheel outranks the restore.
            // Plus what the collision took: a zoom the sweep pulled in is not
            // a zoom the player chose, and without this a walk down one
            // corridor meant never getting the outdoor distance back.
            if (userTargetDistance + collisionZoomDebt_ >= MAX_DISTANCE_INTERIOR - 0.01f &&
                outdoorTargetDistance_ > userTargetDistance) {
                userTargetDistance = outdoorTargetDistance_;
            }
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
        // Built geometry and the ground are treated separately from here on.
        //
        // A wall or a pillar is a thing with a side to step around, and it wants
        // reacting to sharply. A hillside is neither: it has no edge to clear,
        // it is under the camera for as long as the slope lasts, and the height
        // it reports changes a little with every step the player takes. Feeding
        // the ground through the same sharp path made the camera jitter along a
        // slope and shove forward at every rise.
        auto wmoLimitAlong = [&](const glm::vec3& dir) -> float {
            if (!wmoRenderer) return currentDistance;
            float hit = wmoRenderer->raycastBoundingBoxes(pivot, dir, currentDistance);
            // hit == currentDistance means no hit (returns maxDistance on miss)
            if (hit >= currentDistance) return currentDistance;
            return std::max(MIN_DISTANCE, hit - CAM_SPHERE_RADIUS - CAM_EPSILON);
        };

        float rawLimit = wmoLimitAlong(camDir);

        // Step around what is in the way rather than only backing away from it.
        //
        // Pulling straight in is what puts the camera on the player's neck the
        // moment they set their back to a wall, and from inside their own head
        // there is nothing to steer by. A few degrees of yaw usually clears the
        // obstruction entirely - a pillar, a doorframe, the corner of a
        // building - and keeps the shot.
        //
        // Two extra rays at most, and only on a frame that is actually blocked,
        // which is the minority of them. The winner has to buy a real amount of
        // clearance, so the camera does not wander for nothing.
        constexpr float DEFLECT_MAX_DEG = 14.0f;
        constexpr float DEFLECT_MIN_GAIN = 1.0f;   // yards of clearance worth turning for
        float deflectTarget = 0.0f;
        if (rawLimit < currentDistance - 0.5f && rawLimit < userTargetDistance) {
            float bestLimit = rawLimit;
            for (float trialDeg : {DEFLECT_MAX_DEG, -DEFLECT_MAX_DEG}) {
                const float a = glm::radians(trialDeg);
                const float ca = std::cos(a), sa = std::sin(a);
                // Yaw about world Z: the camera should step sideways, not rise.
                glm::vec3 trialDir(camDir.x * ca - camDir.y * sa,
                                   camDir.x * sa + camDir.y * ca,
                                   camDir.z);
                float trialLimit = wmoLimitAlong(glm::normalize(trialDir));
                if (trialLimit > bestLimit + DEFLECT_MIN_GAIN) {
                    bestLimit = trialLimit;
                    deflectTarget = trialDeg;
                }
            }
            rawLimit = bestLimit;
        }

        // Ease into and out of the turn; snapping to it reads as the camera
        // being knocked sideways.
        {
            const float tau = (deflectTarget != 0.0f) ? 0.12f : 0.30f;
            const float alpha = 1.0f - std::exp(-deltaTime / tau);
            cameraDeflectDeg_ += (deflectTarget - cameraDeflectDeg_) * alpha;
            if (std::abs(cameraDeflectDeg_) > 0.05f) {
                const float a = glm::radians(cameraDeflectDeg_);
                const float ca = std::cos(a), sa = std::sin(a);
                camDir = glm::normalize(glm::vec3(camDir.x * ca - camDir.y * sa,
                                                  camDir.x * sa + camDir.y * ca,
                                                  camDir.z));
            }
        }

        // The ground, folded in after the deflection and on its own timing.
        //
        // It is never a reason to turn: there is no side to a hillside to step
        // past, so trying to yaw around one only wanders. And its own limit is
        // smoothed before it is used, because the marched height changes a
        // little every step the player takes and that arrives as a shudder if
        // it is passed straight through.
        if (!cachedInsideInteriorWMO) {
            const float terrainLimit =
                raymarchTerrainCameraLimit(pivot, camDir, currentDistance);
            if (smoothedTerrainDist_ < 0.0f) smoothedTerrainDist_ = terrainLimit;
            // Slower coming in than the built geometry above, which is what
            // makes a slope feel like a slope rather than a series of nudges.
            const float terrainTau = (terrainLimit < smoothedTerrainDist_) ? 0.18f : 0.45f;
            const float terrainAlpha = 1.0f - std::exp(-deltaTime / terrainTau);
            smoothedTerrainDist_ += (terrainLimit - smoothedTerrainDist_) * terrainAlpha;
            rawLimit = std::min(rawLimit, smoothedTerrainDist_);
        } else {
            smoothedTerrainDist_ = -1.0f;
        }

        // Initialise smoothed state on first use.
        if (smoothedCollisionDist_ < 0.0f) {
            smoothedCollisionDist_ = rawLimit;
        }

        // Asymmetric smoothing:
        //   • Pull-in: τ ≈ 60 ms  - react quickly to prevent clipping
        //     (instant while actively rotating, matching the 1:1 drag snap)
        //   • Recover: τ ≈ 400 ms - zoom out slowly after leaving geometry
        bool rotatingNow = mouseButtonDown || f.nowTurnLeft || f.nowTurnRight;
        if (rawLimit < smoothedCollisionDist_ && rotatingNow) {
            smoothedCollisionDist_ = rawLimit;
        } else {
            const float tau = (rawLimit < smoothedCollisionDist_) ? 0.06f : 0.40f;
            float alpha = 1.0f - std::exp(-deltaTime / tau);
            smoothedCollisionDist_ += (rawLimit - smoothedCollisionDist_) * alpha;
        }

        collisionDistance = std::min(collisionDistance, smoothedCollisionDist_);

        // A collision takes the zoom in with it.
        //
        // The sweep moved the camera and left the zoom where it was, so the
        // moment the obstruction passed the camera sprang back out to a
        // distance the room had already refused - and the next step pushed it
        // in again. That in-and-out is the whole of what makes an interior
        // unpleasant, and no amount of smoothing fixes it, because the target
        // it is smoothing toward is the wrong one. Bringing the target in
        // behind the camera is what lets the view settle.
        // Not while something else owns the distance.
        //
        // The barber's chair sets it to 2.4 and holds the camera on the
        // character's face, where the sweep meets the chair and the character
        // constantly. Pulling in there and handing it back a second later is a
        // slow breathe in and out at close range, which from the chair reads as
        // the view bobbing - reported as exactly that. A scripted view is not a
        // player deciding how far back to stand.
        const float bite = barberView_ ? 0.0f
                                       : userTargetDistance - smoothedCollisionDist_;
        if (bite > 0.1f) {
            collisionClearSeconds_ = 0.0f;
            // Toward what the sweep is asking for and no further, over about a
            // second: passing a doorway should not collapse the zoom.
            constexpr float kPullTau = 1.0f;
            const float step = bite * (1.0f - std::exp(-deltaTime / kPullTau));
            const float pulled = std::max(userTargetDistance - step, MIN_DISTANCE);
            collisionZoomDebt_ += userTargetDistance - pulled;
            userTargetDistance = pulled;
        } else {
            collisionClearSeconds_ += deltaTime;
            // Given back only once nothing has been in the way for a moment,
            // and more slowly than it was taken, so clearing a doorway does
            // not throw the camera straight out into the next wall. Slower out
            // than in is also what stops the two rules chasing each other.
            constexpr float kSettleSeconds = 1.0f;
            constexpr float kGiveTau = 2.5f;
            if (collisionClearSeconds_ > kSettleSeconds && collisionZoomDebt_ > 0.001f) {
                const float give = collisionZoomDebt_ * (1.0f - std::exp(-deltaTime / kGiveTau));
                userTargetDistance += give;
                collisionZoomDebt_ -= give;
            }
        }
    } else {
        smoothedCollisionDist_ = -1.0f;   // Reset when no collision sources available
        smoothedTerrainDist_ = -1.0f;
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
        actualCam = pivot + f.forward3D * 0.1f;  // Slightly forward to not clip head
    } else {
        actualCam = pivot + camDir * actualDist;
    }

    // Prefer a view clearly above the water or clearly below it.
    //
    // Sitting exactly on the surface is the one place the water has nothing
    // sensible to draw: the near plane cuts it, and what is left is a seam
    // across the middle of the view. Lean out of a band either side - whichever
    // side the camera is already nearer - and let it settle there.
    //
    // Applied to the target, not to the smoothed position. Nudging the smoothed
    // position after the fact fought the follow, which re-aims at this target
    // every frame: the two pulled opposite ways and the camera hovered on the
    // waterline crossing it several times a second, which is exactly the state
    // this is meant to avoid. Moving the target instead lets the existing
    // smoothing carry it, and it stays where it is put.
    //
    // The band is the near plane's half-height, because that is the depth range
    // over which the near plane can be half in the water. Lifting eases the
    // camera back a little and dropping draws it in, so the character stays
    // framed rather than sliding down the screen.
    if (waterRenderer && actualDist >= MIN_DISTANCE + 0.1f) {
        const float band = std::max(0.05f, camera->getNearPlane()
                                           * std::tan(glm::radians(camera->getFovDegrees()) * 0.5f));
        auto surfaceZ = waterRenderer->getNearestWaterHeightAt(
            actualCam.x, actualCam.y, actualCam.z, 30.0f);
        // Clear of the band, not resting on its edge.
        //
        // Parking exactly at the edge puts the camera on the same threshold the
        // underwater overlay switches at, so the smallest movement flipped it on
        // and off and the view changed brightness with it. Overshooting settles
        // the camera on one side of that decision.
        float wantOffset = 0.0f;
        if (surfaceZ) {
            const float above = actualCam.z - *surfaceZ;
            if (std::abs(above) < band) {
                const float clear = band * 1.35f;
                wantOffset = ((above >= 0.0f) ? (*surfaceZ + clear) : (*surfaceZ - clear))
                           - actualCam.z;
            }
        }
        // Eased, and eased back to nothing when there is no water under the
        // camera at all. Applying it the instant the query finds a surface, and
        // dropping it the instant it does not, made the camera jump at the
        // water's edge - which is where a player crossing in and out of a lake
        // spends their time.
        const float ease = 1.0f - std::exp(-deltaTime / 0.18f);
        waterNudgeZ_ += (wantOffset - waterNudgeZ_) * ease;
        if (std::abs(waterNudgeZ_) > 1e-3f) {
            actualCam.z += waterNudgeZ_;
            glm::vec2 outward(actualCam.x - pivot.x, actualCam.y - pivot.y);
            const float outLen = glm::length(outward);
            if (outLen > 1e-3f) {
                outward /= outLen;
                actualCam.x += outward.x * waterNudgeZ_ * 0.6f;
                actualCam.y += outward.y * waterNudgeZ_ * 0.6f;
            }
        }
    } else {
        waterNudgeZ_ = 0.0f;
    }

    // Smooth camera position to avoid jitter
    if (glm::dot(smoothedCamPos, smoothedCamPos) < 1e-4f) {
        smoothedCamPos = actualCam;  // Initialize
    }
    bool activelyRotating = mouseButtonDown || f.nowTurnLeft || f.nowTurnRight;
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
        // How tall whoever is being followed actually is, so the pivot lands
        // on them rather than at a fixed distance above their feet. Measured
        // here because it changes with the model: a race change, a mount, a
        // shapeshift. Kept if it ever fails to measure, so a frame that could
        // not answer does not snap the camera.
        float height = 0.0f;
        if (characterRenderer->getInstanceHeight(playerInstanceId, height) && height > 0.01f) {
            // The head bone, which is what the pivot is placed against. A
            // skeleton that does not name one leaves this zero and falls back
            // to the height.
            //
            // A frame that cannot answer keeps the last answer. The height
            // above is already guarded that way by the enclosing `if`; the
            // head bone was not, and zeroing it dropped the pivot to a lower
            // tier of pivotHeightFor - a gnome went 0.65 to 0.88, or to the
            // flat 1.6 when the height was missing too. Entering a building is
            // where this showed: the WMO transition re-registers the instance
            // and the bone query returns false for a frame, so the camera rose
            // on the doorstep and stayed up.
            float headZ = 0.0f;
            const bool measuredHead =
                characterRenderer->getInstanceKeyBonePivotZ(playerInstanceId, kKeyBoneHead, headZ)
                && headZ > 0.01f;
            if (!measuredHead) headZ = followedHeadZ_;
            if (std::abs(height - followedHeight_) > 0.01f ||
                std::abs(headZ - followedHeadZ_) > 0.01f) {
                // Said when it changes, which is when a character is first
                // drawn and whenever its model does. These numbers settle
                // whether the camera is pivoting where it should, and none of
                // them was knowable from outside: a report of "still too high"
                // can mean nothing was measured, or that it was and the
                // proportion is wrong, and they tell the two apart.
                LOG_WARNING("Camera pivot: character is ", height,
                            " tall with its head bone at ", headZ,
                            ", pivoting at ", pivotHeightFor(pivotHeight_, height, headZ),
                            " (setting ", pivotHeight_, " against a ",
                            kPivotReferenceHeadZ, " head reference)");
            }
            followedHeight_ = height;
            if (measuredHead) followedHeadZ_ = headZ;
        }

        // Hide only on first-person *intent* (the user's zoom), not on a
        // collision-squeezed distance. The `actualDist < MIN_DISTANCE + 0.1`
        // term fired whenever geometry pushed the camera close in third
        // person - under Undercity's overhangs, or backing into a wall -
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
        // (Run, Walk, Jump, Swim, etc.) - no additional animation driving needed here.
    }
}

void CameraController::updateThirdPersonCamera(float deltaTime, FrameInput& f) {
    glm::vec3 prevTargetPos(0.0f);
    glm::vec3 targetPos = moveFollowedCharacter(deltaTime, f, prevTargetPos);
    groundFollowedCharacter(deltaTime, f, targetPos, prevTargetPos);
    // Landing: flying down onto real ground, or skimming it, without climbing.
    // Grounding has just put the feet on the floor if they were within reach
    // of it; on a floor that was actually found, that is touching down.
    if (isFlightAirborne() && grounded && hasRealGround_ && !f.spaceHeld &&
        std::abs(targetPos.z - lastGroundZ) < 0.3f) {
        flightAirborne_ = false;
        LOG_WARNING("Flight: landed");
    }
    updateOrbitCamera(deltaTime, f, targetPos);
}

// The camera that flies itself, used when nothing is being followed.
void CameraController::updateFreeFlyCamera(float /*deltaTime*/, FrameInput& f) {
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
        // flipped between swim and walk every frame - each flip restarts the
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
                              ? swimSpeedOverride_ : f.speed * SWIM_SPEED_FACTOR;
        float waterSurfaceCamZ = waterH ? (*waterH - WATER_SURFACE_OFFSET + eyeHeight) : newPos.z;
        bool diveIntent = f.nowForward && (f.forward3D.z < -0.28f);

        float movLenSq = glm::dot(f.movement, f.movement);
        if (movLenSq > 1e-6f) {
            f.movement *= glm::inversesqrt(movLenSq);
            newPos += f.movement * swimSpeed * f.physicsDeltaTime;
        }

        if (f.nowJump) {
            verticalVelocity = SWIM_BUOYANCY;
        } else {
            verticalVelocity += SWIM_GRAVITY * f.physicsDeltaTime;
            if (verticalVelocity < SWIM_SINK_SPEED) {
                verticalVelocity = SWIM_SINK_SPEED;
            }
            if (!diveIntent) {
                float surfaceErr = (waterSurfaceCamZ - newPos.z);
                verticalVelocity += surfaceErr * 7.0f * f.physicsDeltaTime;
                verticalVelocity *= std::max(0.0f, 1.0f - 3.2f * f.physicsDeltaTime);
                if (std::abs(surfaceErr) < 0.06f && std::abs(verticalVelocity) < 0.35f) {
                    verticalVelocity = 0.0f;
                }
            }
        }

        newPos.z += verticalVelocity * f.physicsDeltaTime;

        // Don't rise above water surface (feet at water level)
        if (waterH && (newPos.z - eyeHeight) > *waterH - WATER_SURFACE_OFFSET) {
            newPos.z = *waterH - WATER_SURFACE_OFFSET + eyeHeight;
            if (verticalVelocity > 0.0f) verticalVelocity = 0.0f;
        }

        grounded = false;
    } else {
        swimming = false;

        float movLenSq2 = glm::dot(f.movement, f.movement);
        if (movLenSq2 > 1e-6f) {
            f.movement *= glm::inversesqrt(movLenSq2);
            newPos += f.movement * f.speed * f.physicsDeltaTime;
        }

        // Jump with input buffering and coyote time
        if (f.nowJump) jumpBufferTimer = JUMP_BUFFER_TIME;
        if (grounded) coyoteTimer = COYOTE_TIME;

        if (coyoteTimer > 0.0f && jumpBufferTimer > 0.0f && !mounted_) {
            verticalVelocity = f.jumpVel;
            grounded = false;
            jumpBufferTimer = 0.0f;
            coyoteTimer = 0.0f;
        }

        jumpBufferTimer -= f.physicsDeltaTime;
        coyoteTimer -= f.physicsDeltaTime;

        // Apply f.gravity, unless the ground under the character has not
        // streamed in yet. See the note at the other gravity site.
        if (groundNotStreamedYet(newPos.x, newPos.y)) {
            verticalVelocity = 0.0f;
        } else {
            verticalVelocity += f.gravity * f.physicsDeltaTime;
            newPos.z += verticalVelocity * f.physicsDeltaTime;
        }
    }

    // Wall sweep collision before grounding (skip when stationary).
    {
        // Swept at the feet, not the eyes. Doodads are left out here, which
        // is the one thing this camera does differently.
        const glm::vec3 eyeOffset(0, 0, eyeHeight);
        newPos = sweepAgainstWalls(camera->getPosition() - eyeOffset,
                                   newPos - eyeOffset, false) + eyeOffset;
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
            glm::vec3 pivot = targetPos + glm::vec3(0.0f, 0.0f, followedPivotHeight() + mountedOffset);

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

    // Don't process keyboard input when UI text input (e.g. chat box) has focus.
    // Both interfaces are asked: the flag below is ImGui's own and says nothing
    // about a chat box FrameXML draws, so on its own it let the character walk
    // away while someone was typing.
    bool uiWantsKeyboard = ImGui::GetIO().WantTextInput ||
                           ui::interfaceTakingTypedInput();

    // Suppress movement input after teleport/portal (keys may still be held)
    if (movementSuppressTimer_ > 0.0f) {
        movementSuppressTimer_ -= deltaTime;
    }
    bool movementSuppressed = movementSuppressTimer_ > 0.0f;

    // Determine current key states
    // Arrow keys mirror WASD the way the original client maps them:
    // Up/Down move, Left/Right turn (strafe stays on Q/E).
    bool keyW = !uiWantsKeyboard && !sitting && !movementSuppressed && (input.isKeyPressed(SDL_SCANCODE_W) || input.isKeyPressed(SDL_SCANCODE_UP));
    bool keyS = !uiWantsKeyboard && !sitting && !movementSuppressed && (input.isKeyPressed(SDL_SCANCODE_S) || input.isKeyPressed(SDL_SCANCODE_DOWN));
    bool keyA = !uiWantsKeyboard && !sitting && !movementSuppressed && (input.isKeyPressed(SDL_SCANCODE_A) || input.isKeyPressed(SDL_SCANCODE_LEFT));
    bool keyD = !uiWantsKeyboard && !sitting && !movementSuppressed && (input.isKeyPressed(SDL_SCANCODE_D) || input.isKeyPressed(SDL_SCANCODE_RIGHT));
    bool keyQ = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_Q);
    bool keyE = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_E);
    bool shiftDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LSHIFT) || input.isKeyPressed(SDL_SCANCODE_RSHIFT));
    bool ctrlDown = !uiWantsKeyboard && (input.isKeyPressed(SDL_SCANCODE_LCTRL) || input.isKeyPressed(SDL_SCANCODE_RCTRL));
    bool nowJump = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyJustPressed(SDL_SCANCODE_SPACE);
    // Swimming and flying need the held state, not the press edge: on land
    // space is a one-shot jump, but in water and in the air it is continuous
    // ascent, and an edge gave a single impulse that then bled away - or, in
    // flight, a single frame of climb per press.
    bool spaceHeld = !uiWantsKeyboard && !sitting && !movementSuppressed && input.isKeyPressed(SDL_SCANCODE_SPACE);

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
                // Continuous smooth rotation - no lerp endpoint, just constant angular velocity
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
        keyW = keyS = keyA = keyD = keyQ = keyE = nowJump = spaceHeld = false;
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
            float targetYawDeg = targetYawRad * 180.0f / core::coords::PI;
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
    if (entryFloorAnchorTimer_ > 0.0f) entryFloorAnchorTimer_ -= deltaTime;
    if (gravitySuspendTimer_ > 0.0f) {
        gravitySuspendTimer_ -= deltaTime;
    }

    // Select physics constants based on mode
    float gravity = useWoWSpeed ? WOW_GRAVITY : GRAVITY;
    float jumpVel = useWoWSpeed ? WOW_JUMP_VELOCITY : JUMP_VELOCITY;

    // Suspend gravity after world entry - hold Z position until timer expires
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

    // Get camera axes - project forward onto XY plane for walking
    glm::vec3 forward3D = camera->getForward();
    // In water the camera always steers, moving or turning, the way holding the
    // right button steers on land.
    //
    // Swimming took its forward from the camera's 3D direction but its strafe
    // and its facing from the character's own, so the two disagreed the moment
    // the camera was panned: the swimmer went one way, sidled another, and kept
    // facing a third, with the stroke animation pointing wherever the body
    // happened to be left. Retail turns the swimmer to the camera and swims
    // them where they look.
    //
    // Only while there is movement input. Treading on the spot and orbiting the
    // camera to look around should not spin the character on the water.
    // steering_ is the touch controls saying a finger is dragging the view.
    // On a phone that is what the right mouse button is on a desktop: the
    // character turns to face where the player is looking, so that walking
    // forward goes where the screen points.
    bool cameraDrivesFacing = rightMouseDown || mouseAutorun || steering_ ||
                              (swimming && hasMoveInput);
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

    // Flight goes where the mount faces, not where the camera looks. Orbiting
    // the camera with the left button is looking around, and it took the
    // flight direction with it: W flew toward the camera's view, whatever the
    // mount was pointing at. The pitch follows the camera only while the
    // camera steers, as the facing does, and holds otherwise.
    if (cameraDrivesFacing && !externalFollow_) {
        const float len = glm::length(forward3D);
        if (len > 1e-4f) {
            flightPitchDeg_ = glm::degrees(std::asin(std::clamp(forward3D.z / len, -1.0f, 1.0f)));
        }
    }
    if (!isFlightAirborne()) flightPitchDeg_ = 0.0f;
    const float flightPitchRad = glm::radians(flightPitchDeg_);
    const glm::vec3 flightForward(std::cos(moveYawRad) * std::cos(flightPitchRad),
                                  std::sin(moveYawRad) * std::cos(flightPitchRad),
                                  std::sin(flightPitchRad));

    // Toggle sit/crouch with X key (edge-triggered) - only when UI doesn't want keyboard
    // Blocked while mounted
    bool prevSitting = sitting;
    bool xDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_X);
    // While swimming, X dives down instead of toggling sit
    if (xDown && !xKeyWasDown && !mounted_ && !swimming) {
        sitting = !sitting;
    }
    if (mounted_) sitting = false;
    xKeyWasDown = xDown;

    // Reset camera angles with R key (edge-triggered) - only when UI doesn't want keyboard
    // Does NOT move the player; full reset() is reserved for world-entry/respawn.
    bool rDown = !uiWantsKeyboard && input.isKeyPressed(SDL_SCANCODE_R);
    if (rDown && !rKeyWasDown) {
        resetAngles();
    }
    rKeyWasDown = rDown;

    // Stand up on any movement input or jump while sitting (WoW behaviour)
    //
    // Both mouse buttons is a way of walking forward, and it was missing from
    // this list while every key that does the same was on it. Sitting gates
    // the autorun it produces, so the two together meant that after eating
    // there was no way to move with the mouse at all: the input that would
    // have stood the character up was the one input that could not.
    if (sitting && !movementSuppressed) {
        const bool anyMoveKey = !uiWantsKeyboard && (
            input.isKeyPressed(SDL_SCANCODE_W) || input.isKeyPressed(SDL_SCANCODE_S) ||
            input.isKeyPressed(SDL_SCANCODE_A) || input.isKeyPressed(SDL_SCANCODE_D) ||
            input.isKeyPressed(SDL_SCANCODE_Q) || input.isKeyPressed(SDL_SCANCODE_E) ||
            input.isKeyPressed(SDL_SCANCODE_UP) || input.isKeyPressed(SDL_SCANCODE_DOWN) ||
            input.isKeyPressed(SDL_SCANCODE_LEFT) || input.isKeyPressed(SDL_SCANCODE_RIGHT) ||
            input.isKeyPressed(SDL_SCANCODE_SPACE));
        // Not gated on uiWantsKeyboard, which is a different question, and not
        // on the interface wanting the mouse either: both flags are already
        // cleared at the press when it did.
        const bool mouseWalk = leftMouseDown && rightMouseDown;
        if (anyMoveKey || mouseWalk) sitting = false;
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

    // Everything the two camera modes need from the work above.
    FrameInput f;
    f.physicsDeltaTime = physicsDeltaTime;
    f.gravity = gravity;
    f.jumpVel = jumpVel;
    f.speed = speed;
    f.forward = forward;
    f.right = right;
    f.forward3D = forward3D;
    f.flightForward = flightForward;
    f.movement = movement;
    f.nowForward = nowForward;
    f.nowBackward = nowBackward;
    f.nowStrafeLeft = nowStrafeLeft;
    f.nowStrafeRight = nowStrafeRight;
    f.nowTurnLeft = nowTurnLeft;
    f.nowTurnRight = nowTurnRight;
    f.nowJump = nowJump;
    f.spaceHeld = spaceHeld;
    f.xDown = xDown;
    f.uiWantsKeyboard = uiWantsKeyboard;

    // Two cameras: one orbits a character and moves it, one flies itself.
    if (thirdPerson && followTarget) {
        updateThirdPersonCamera(deltaTime, f);
    } else {
        updateFreeFlyCamera(deltaTime, f);
    }

    // The modes adjust the move - the swim clamp and the flight path both do -
    // and the movement opcodes below are decided from what actually happened.
    movement = f.movement;

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

        // Jump. Not in flight, where space climbs - and where grounded is held
        // true only to keep the fall-damage checks quiet, so every press sent
        // the server a jump from mid-air.
        if (nowJump && !wasJumping && grounded && !isFlightAirborne()) {
            movementCallback(static_cast<uint32_t>(game::Opcode::MSG_MOVE_JUMP));
        }

        // Fall landing, for a fall that actually happened.
        //
        // A bump at riding speed breaks ground contact for a frame, and the
        // frame after it this read as a landing: the packet went out, and
        // every client in range played the landing animation on the mount.
        // Crossing open ground did that several times a second, which is what
        // other players see as a mount behaving erratically.
        //
        // Long enough to be a fall rather than a step. Well under the shortest
        // real drop - a jump is airborne for about a second - so nothing a
        // player would call a fall is lost.
        constexpr float kFallReportSeconds = 0.15f;
        if (wasFalling && grounded && airborneSeconds_ >= kFallReportSeconds) {
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
        const bool airborne = isFlightAirborne();
        const bool nowAscending = airborne && spaceHeld;
        const bool nowDescending = airborne && xDown && mounted_;

        if (airborne) {
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
    // Counted while airborne and zeroed the moment the ground is back, so the
    // landing test above reads the length of the fall that just ended.
    airborneSeconds_ = grounded ? 0.0f : (airborneSeconds_ + deltaTime);

    // R key is now handled above with chat safeguard (WantTextInput check)

    // Camera shake (SMSG_CAMERA_SHAKE): apply sinusoidal offset to final camera position.
    if (shakeElapsed_ < shakeDuration_) {
        shakeElapsed_ += deltaTime;
        float t = shakeElapsed_ / shakeDuration_;
        // Envelope: fade out over the last 30% of shake duration
        float envelope = (t < 0.7f) ? 1.0f : (1.0f - (t - 0.7f) / 0.3f);
        float theta = shakeElapsed_ * shakeFrequency_ * core::coords::TWO_PI;
        glm::vec3 offset(
            shakeMagnitude_ * envelope * std::sin(theta),
            shakeMagnitude_ * envelope * std::cos(theta * 1.3f),
            shakeMagnitude_ * envelope * std::sin(theta * 0.7f) * 0.5f
        );
        if (camera) {
            camera->setPosition(camera->getPosition() + offset);
        }
    }

    // Scaled by the same setting as the shake above, because it is the same
    // thing from the player's side: the view moving without being asked to.
    //
    // The stumble in the travel direction is deliberately not scaled with it.
    // That one is what being drunk does to the character rather than to the
    // picture, and turning it off would be an advantage rather than a comfort.
    if (intoxication_ > 0.0f && camera) {
        const float swayYaw = std::sin(intoxicationTime_ * 1.35f) * 2.5f * intoxication_ * shakeScale_;
        const float swayPitch = std::sin(intoxicationTime_ * 1.8f + 0.7f) * 1.6f * intoxication_ * shakeScale_;
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
    if (rotationSuppressed_) {
        return;
    }

    // Hold rotation until the cursor has actually gone somewhere, so a click
    // that means "target this" does not swing the view on the way.
    //
    // The same distance the click test uses, measured the same way: the net
    // offset from the press, not the length of the path taken to get there.
    if (!rotateArmed_) {
        dragOffsetX_ += static_cast<float>(event.xrel);
        dragOffsetY_ += static_cast<float>(event.yrel);
        const float threshold = core::clickDragThreshold();
        if (dragOffsetX_ * dragOffsetX_ + dragOffsetY_ * dragOffsetY_ <
            threshold * threshold) {
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

    // Don't capture mouse when ImGui wants it (hovering UI windows), or when
    // FrameXML does. FrameXML draws into ImGui's background draw list, so
    // WantCaptureMouse is false over every frame it owns - pressing a bag item
    // turned the camera as well as pressing the item, and dragging one swung
    // the view around.
    bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse || ui::frameXmlOwnsMouse();

    if (event.button == SDL_BUTTON_LEFT) {
        leftMouseDown = (event.down) && !uiWantsMouse;
        if (event.down && event.clicks >= 2) {
            autoRunning = false;
        }
    }
    if (event.button == SDL_BUTTON_RIGHT) {
        rightMouseDown = (event.down) && !uiWantsMouse;
    }

    bool anyDown = leftMouseDown || rightMouseDown;
    if (anyDown && !mouseButtonDown) {
        // SDL3 captures per window rather than globally. The focused window is the one the player is pointing at, and relative mode means nothing for any other - so that is the one to ask.
        SDL_SetWindowRelativeMouseMode(SDL_GetKeyboardFocus(), true);
        // Throw away the delta the switch itself makes. Entering relative
        // mode hands over the movement since the last relative read, which on
        // a press is everything the cursor did on its way to the thing being
        // clicked - a whole screen's worth, delivered as one motion event
        // that cleared any dead-zone on its own.
        SDL_GetRelativeMouseState(nullptr, nullptr);
        // Arm the rotation dead-zone fresh for this press.
        rotateArmed_ = false;
        dragOffsetX_ = 0.0f;
        dragOffsetY_ = 0.0f;
    } else if (!anyDown && mouseButtonDown) {
        SDL_SetWindowRelativeMouseMode(SDL_GetKeyboardFocus(), false);
        rotateArmed_ = false;
    }
    mouseButtonDown = anyDown;
}

void CameraController::releaseMouseCapture() {
    leftMouseDown = false;
    rightMouseDown = false;
    mouseButtonDown = false;
    rotateArmed_ = false;
    dragOffsetX_ = 0.0f;
    dragOffsetY_ = 0.0f;
    SDL_SetWindowRelativeMouseMode(SDL_GetKeyboardFocus(), false);
    SDL_ShowCursor();
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
            glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, followedPivotHeight() + mountedOffset);
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
    constexpr float PI = core::coords::PI;
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
        glm::vec3 pivot = spawnPos + glm::vec3(0.0f, 0.0f, followedPivotHeight() + mountedOffset);
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
        glm::vec3 pivot = pos + glm::vec3(0.0f, 0.0f, followedPivotHeight() + mountedOffset);
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

bool CameraController::groundNotStreamedYet(float x, float y) const {
    return terrainManager != nullptr && !terrainManager->isTileLoadedAt(x, y);
}

void CameraController::applyLookDelta(float dxPixels, float dyPixels) {
    if (!enabled || !camera || introActive) return;

    yaw -= dxPixels * mouseSensitivity;
    const float invert = invertMouse ? 1.0f : -1.0f;
    pitch += dyPixels * mouseSensitivity * invert;
    pitch = glm::clamp(pitch, MIN_PITCH, MAX_PITCH);
    camera->setRotation(yaw, pitch);
}

void CameraController::processMouseWheel(float delta) {
    // The player's own choice clears whatever the sweep had taken: a distance
    // they picked outranks one this arrived at, and it must not be given back
    // on top of theirs a moment later.
    collisionZoomDebt_ = 0.0f;
    collisionClearSeconds_ = 0.0f;
    // Scale zoom speed proportionally to current distance for fine control up close
    float zoomSpeed = glm::max(userTargetDistance * 0.15f, 0.3f);
    userTargetDistance -= delta * zoomSpeed;
    // A multiple of the original client's limit rather than a choice between
    // two numbers, so the game's own Max Camera Distance slider means something
    // at every position instead of only at its ends.
    float maxDist = MAX_DISTANCE_NORMAL * maxDistanceFactor_;
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
    // in render space is simply (vcos, vsin) - the two swaps cancel each other.
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
