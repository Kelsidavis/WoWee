#pragma once

#include "rendering/camera.hpp"
#include "core/input.hpp"
#include <SDL2/SDL.h>
#include <functional>
#include <optional>

namespace wowee {
namespace rendering {

class TerrainManager;
class WMORenderer;
class M2Renderer;
class WaterRenderer;

class CameraController {
public:
    CameraController(Camera* camera);

    void update(float deltaTime);
    void processMouseMotion(const SDL_MouseMotionEvent& event);
    void processMouseButton(const SDL_MouseButtonEvent& event);

    void setMovementSpeed(float speed) { movementSpeed = speed; }
    void setMouseSensitivity(float sensitivity) { mouseSensitivity = sensitivity; }
    float getMouseSensitivity() const { return mouseSensitivity; }
    void setInvertMouse(bool invert) { invertMouse = invert; }
    bool isInvertMouse() const { return invertMouse; }
    void setEnabled(bool enabled) { this->enabled = enabled; }
    void setTerrainManager(TerrainManager* tm) { terrainManager = tm; }
    void setWMORenderer(WMORenderer* wmo) { wmoRenderer = wmo; }
    void setM2Renderer(M2Renderer* m2) { m2Renderer = m2; }
    void setWaterRenderer(WaterRenderer* wr) { waterRenderer = wr; }

    void processMouseWheel(float delta);
    void setFollowTarget(glm::vec3* target);
    void setDefaultSpawn(const glm::vec3& position, float yawDeg, float pitchDeg) {
        defaultPosition = position;
        defaultYaw = yawDeg;
        defaultPitch = pitchDeg;
    }

    void reset();
    void teleportTo(const glm::vec3& pos);
    void setOnlineMode(bool online) { onlineMode = online; }

    // Last known safe position (grounded, not falling)
    bool hasLastSafePosition() const { return hasLastSafe_; }
    const glm::vec3& getLastSafePosition() const { return lastSafePos_; }
    float getContinuousFallTime() const { return continuousFallTime_; }

    // Auto-unstuck callback (triggered when falling too long)
    using AutoUnstuckCallback = std::function<void()>;
    void setAutoUnstuckCallback(AutoUnstuckCallback cb) { autoUnstuckCallback_ = std::move(cb); }
    void startIntroPan(float durationSec = 2.8f, float orbitDegrees = 140.0f);
    bool isIntroActive() const { return introActive; }
    bool isIdleOrbit() const { return idleOrbit_; }

    float getMovementSpeed() const { return movementSpeed; }
    const glm::vec3& getDefaultPosition() const { return defaultPosition; }
    bool isMoving() const;
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getFacingYaw() const { return facingYaw; }
    bool isThirdPerson() const { return thirdPerson; }
    bool isGrounded() const { return grounded; }
    bool isJumping() const { return !grounded && verticalVelocity > 0.0f; }
    bool isFalling() const { return !grounded && verticalVelocity <= 0.0f; }
    bool isSprinting() const;
    bool isMovingForward() const { return moveForwardActive; }
    bool isMovingBackward() const { return moveBackwardActive; }
    bool isStrafingLeft() const { return strafeLeftActive; }
    bool isStrafingRight() const { return strafeRightActive; }
    bool isRightMouseHeld() const { return rightMouseDown; }
    bool isSitting() const { return sitting; }
    bool isSwimming() const { return swimming; }
    bool isInsideWMO() const { return cachedInsideWMO; }
    const glm::vec3* getFollowTarget() const { return followTarget; }
    glm::vec3* getFollowTargetMutable() { return followTarget; }

    // Movement callback for sending opcodes to server
    using MovementCallback = std::function<void(uint32_t opcode)>;
    void setMovementCallback(MovementCallback cb) { movementCallback = std::move(cb); }
    void setUseWoWSpeed(bool use) { useWoWSpeed = use; }
    void setRunSpeedOverride(float speed) { runSpeedOverride_ = speed; }
    void setMounted(bool m) { mounted_ = m; }
    void setMountHeightOffset(float offset) { mountHeightOffset_ = offset; }
    void setExternalFollow(bool enabled) { externalFollow_ = enabled; }
    void setExternalMoving(bool moving) { externalMoving_ = moving; }
    void clearMovementInputs();

    // For first-person player hiding
    void setCharacterRenderer(class CharacterRenderer* cr, uint32_t playerId) {
        characterRenderer = cr;
        playerInstanceId = playerId;
    }

private:
    Camera* camera;
    TerrainManager* terrainManager = nullptr;
    WMORenderer* wmoRenderer = nullptr;
    M2Renderer* m2Renderer = nullptr;
    WaterRenderer* waterRenderer = nullptr;
    CharacterRenderer* characterRenderer = nullptr;
    uint32_t playerInstanceId = 0;

    // Stored rotation (avoids lossy forward-vector round-trip)
    float yaw = 180.0f;
    float pitch = -30.0f;
    float facingYaw = 180.0f;  // Character-facing yaw (can differ from camera yaw)

    // Movement settings
    float movementSpeed = 50.0f;
    float sprintMultiplier = 3.0f;
    float slowMultiplier = 0.3f;

    // Mouse settings
    float mouseSensitivity = 0.2f;
    bool invertMouse = false;
    bool mouseButtonDown = false;
    bool leftMouseDown = false;
    bool rightMouseDown = false;

    // Third-person orbit camera (WoW-style)
    bool thirdPerson = false;
    float userTargetDistance = 10.0f;   // What the player wants (scroll wheel)
    float currentDistance = 10.0f;      // Smoothed actual distance
    float collisionDistance = 10.0f;    // Max allowed by collision
    bool externalFollow_ = false;
    static constexpr float MIN_DISTANCE = 0.5f;     // Minimum zoom (first-person threshold)
    static constexpr float MAX_DISTANCE = 50.0f;    // Maximum zoom out
    static constexpr float ZOOM_SMOOTH_SPEED = 15.0f;  // How fast zoom eases
    static constexpr float CAM_SMOOTH_SPEED = 20.0f;   // How fast camera position smooths
    static constexpr float PIVOT_HEIGHT = 1.8f;     // Pivot at head height
    static constexpr float CAM_SPHERE_RADIUS = 0.32f;  // Keep camera farther from geometry to avoid clipping-through surfaces
    static constexpr float CAM_EPSILON = 0.22f;        // Extra wall offset to avoid near-plane clipping artifacts
    static constexpr float COLLISION_FOCUS_RADIUS_THIRD_PERSON = 20.0f;  // Reduced for performance
    static constexpr float COLLISION_FOCUS_RADIUS_FREE_FLY = 20.0f;
    static constexpr float MIN_PITCH = -88.0f;      // Look almost straight down
    static constexpr float MAX_PITCH = 35.0f;       // Limited upward look
    glm::vec3* followTarget = nullptr;
    glm::vec3 smoothedCamPos = glm::vec3(0.0f);     // For smooth camera movement

    // Gravity / grounding
    float verticalVelocity = 0.0f;
    bool grounded = false;
    static constexpr float STAND_EYE_HEIGHT = 1.2f;  // Standing eye height
    static constexpr float CROUCH_EYE_HEIGHT = 0.6f; // Crouching eye height
    float eyeHeight = STAND_EYE_HEIGHT;
    float lastGroundZ = 0.0f;  // Last known ground height (fallback when no terrain)
    static constexpr float GRAVITY = -30.0f;
    static constexpr float JUMP_VELOCITY = 15.0f;
    float jumpBufferTimer = 0.0f;   // Time since space was pressed
    float coyoteTimer = 0.0f;       // Time since last grounded
    static constexpr float JUMP_BUFFER_TIME = 0.15f;  // 150ms input buffer
    static constexpr float COYOTE_TIME = 0.10f;        // 100ms grace after leaving ground

    // Cached isInsideWMO result (throttled to avoid per-frame cost)
    bool cachedInsideWMO = false;
    int insideWMOCheckCounter = 0;
    glm::vec3 lastInsideWMOCheckPos = glm::vec3(0.0f);

    // Cached camera WMO floor query (skip if camera moved < 0.3 units)
    glm::vec3 lastCamFloorQueryPos = glm::vec3(0.0f);
    std::optional<float> cachedCamWmoFloor;
    bool hasCachedCamFloor = false;

    // Swimming
    bool swimming = false;
    bool wasSwimming = false;
    static constexpr float SWIM_SPEED_FACTOR = 0.67f;
    static constexpr float SWIM_GRAVITY = -5.0f;
    static constexpr float SWIM_BUOYANCY = 8.0f;
    static constexpr float SWIM_SINK_SPEED = -3.0f;
    static constexpr float WATER_SURFACE_OFFSET = 0.9f;

    // State
    bool enabled = true;
    bool sitting = false;
    bool xKeyWasDown = false;
    bool rKeyWasDown = false;
    bool runPace = false;
    bool autoRunning = false;
    bool tildeWasDown = false;

    // Movement state tracking (for sending opcodes on state change)
    bool wasMovingForward = false;
    bool wasMovingBackward = false;
    bool wasStrafingLeft = false;
    bool wasStrafingRight = false;
    bool wasTurningLeft = false;
    bool wasTurningRight = false;
    bool wasJumping = false;
    bool wasFalling = false;
    bool moveForwardActive = false;
    bool moveBackwardActive = false;
    bool strafeLeftActive = false;
    bool strafeRightActive = false;

    // Movement callback
    MovementCallback movementCallback;

    // Movement speeds
    bool useWoWSpeed = false;
    static constexpr float WOW_RUN_SPEED = 7.0f;     // Normal run (WotLK)
    static constexpr float WOW_SPRINT_SPEED = 10.5f; // Optional fast mode (not default WoW behavior)
    static constexpr float WOW_WALK_SPEED = 2.5f;    // Walk
    static constexpr float WOW_BACK_SPEED = 4.5f;    // Backpedal
    static constexpr float WOW_TURN_SPEED = 180.0f;  // Keyboard turn deg/sec
    static constexpr float WOW_GRAVITY = -19.29f;
    static constexpr float WOW_JUMP_VELOCITY = 7.96f;

    // Server-driven run speed override (0 = use default WOW_RUN_SPEED)
    float runSpeedOverride_ = 0.0f;
    bool mounted_ = false;
    float mountHeightOffset_ = 0.0f;
    bool externalMoving_ = false;

    // Online mode: trust server position, don't prefer outdoors over WMO floors
    bool onlineMode = false;

    // Default spawn position (Goldshire Inn)
    glm::vec3 defaultPosition = glm::vec3(-9464.0f, 62.0f, 200.0f);
    float defaultYaw = 0.0f;
    float defaultPitch = -5.0f;

    // Spawn intro camera pan
    bool introActive = false;
    float introTimer = 0.0f;
    float introDuration = 0.0f;
    float introStartYaw = 0.0f;
    float introEndYaw = 0.0f;
    float introOrbitDegrees = 0.0f;
    float introStartPitch = -15.0f;
    float introEndPitch = -5.0f;
    float introStartDistance = 12.0f;
    float introEndDistance = 10.0f;

    // Idle camera: triggers intro pan after IDLE_TIMEOUT seconds of no input
    float idleTimer_ = 0.0f;
    bool idleOrbit_ = false;  // true when current intro pan is an idle orbit (loops)
    static constexpr float IDLE_TIMEOUT = 120.0f; // 2 minutes

    // Last known safe position (saved periodically when grounded on real geometry)
    bool hasLastSafe_ = false;
    glm::vec3 lastSafePos_ = glm::vec3(0.0f);
    float safePosSaveTimer_ = 0.0f;
    bool hasRealGround_ = false; // True only when terrain/WMO/M2 floor is detected
    static constexpr float SAFE_POS_SAVE_INTERVAL = 2.0f; // Save every 2 seconds

    // No-ground timer: after grace period, let the player fall instead of hovering
    float noGroundTimer_ = 0.0f;
    static constexpr float NO_GROUND_GRACE = 0.5f; // 500ms grace for terrain streaming

    // Continuous fall time (for auto-unstuck detection)
    float continuousFallTime_ = 0.0f;
    bool autoUnstuckFired_ = false;
    AutoUnstuckCallback autoUnstuckCallback_;
    static constexpr float AUTO_UNSTUCK_FALL_TIME = 5.0f; // 5 seconds of falling
};

} // namespace rendering
} // namespace wowee
