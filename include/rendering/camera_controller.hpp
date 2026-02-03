#pragma once

#include "rendering/camera.hpp"
#include "core/input.hpp"
#include <SDL2/SDL.h>
#include <functional>

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
    void setEnabled(bool enabled) { this->enabled = enabled; }
    void setTerrainManager(TerrainManager* tm) { terrainManager = tm; }
    void setWMORenderer(WMORenderer* wmo) { wmoRenderer = wmo; }
    void setM2Renderer(M2Renderer* m2) { m2Renderer = m2; }
    void setWaterRenderer(WaterRenderer* wr) { waterRenderer = wr; }

    void processMouseWheel(float delta);
    void setFollowTarget(glm::vec3* target);

    void reset();

    float getMovementSpeed() const { return movementSpeed; }
    bool isMoving() const;
    float getYaw() const { return yaw; }
    bool isThirdPerson() const { return thirdPerson; }
    bool isGrounded() const { return grounded; }
    bool isJumping() const { return !grounded && verticalVelocity > 0.0f; }
    bool isFalling() const { return !grounded && verticalVelocity <= 0.0f; }
    bool isSprinting() const;
    bool isRightMouseHeld() const { return rightMouseDown; }
    bool isSitting() const { return sitting; }
    bool isSwimming() const { return swimming; }
    const glm::vec3* getFollowTarget() const { return followTarget; }

    // Movement callback for sending opcodes to server
    using MovementCallback = std::function<void(uint32_t opcode)>;
    void setMovementCallback(MovementCallback cb) { movementCallback = std::move(cb); }
    void setUseWoWSpeed(bool use) { useWoWSpeed = use; }

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

    // Movement settings
    float movementSpeed = 50.0f;
    float sprintMultiplier = 3.0f;
    float slowMultiplier = 0.3f;

    // Mouse settings
    float mouseSensitivity = 0.2f;
    bool mouseButtonDown = false;
    bool leftMouseDown = false;
    bool rightMouseDown = false;

    // Third-person orbit camera (WoW-style)
    bool thirdPerson = false;
    float userTargetDistance = 10.0f;   // What the player wants (scroll wheel)
    float currentDistance = 10.0f;      // Smoothed actual distance
    float collisionDistance = 10.0f;    // Max allowed by collision
    static constexpr float MIN_DISTANCE = 0.5f;     // Minimum zoom (first-person threshold)
    static constexpr float MAX_DISTANCE = 50.0f;    // Maximum zoom out
    static constexpr float ZOOM_SMOOTH_SPEED = 15.0f;  // How fast zoom eases
    static constexpr float CAM_SMOOTH_SPEED = 20.0f;   // How fast camera position smooths
    static constexpr float PIVOT_HEIGHT = 1.8f;     // Pivot at head height
    static constexpr float CAM_SPHERE_RADIUS = 0.2f;   // Collision sphere radius
    static constexpr float CAM_EPSILON = 0.05f;     // Offset from walls
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

    // Swimming
    bool swimming = false;
    bool wasSwimming = false;
    static constexpr float SWIM_SPEED_FACTOR = 0.67f;
    static constexpr float SWIM_GRAVITY = -5.0f;
    static constexpr float SWIM_BUOYANCY = 8.0f;
    static constexpr float SWIM_SINK_SPEED = -3.0f;
    static constexpr float WATER_SURFACE_OFFSET = 1.5f;

    // State
    bool enabled = true;
    bool sitting = false;
    bool xKeyWasDown = false;

    // Movement state tracking (for sending opcodes on state change)
    bool wasMovingForward = false;
    bool wasMovingBackward = false;
    bool wasStrafingLeft = false;
    bool wasStrafingRight = false;
    bool wasJumping = false;
    bool wasFalling = false;

    // Movement callback
    MovementCallback movementCallback;

    // Movement speeds
    bool useWoWSpeed = false;
    static constexpr float WOW_RUN_SPEED = 14.0f;    // Normal run (WASD)
    static constexpr float WOW_SPRINT_SPEED = 28.0f; // Sprint (hold Shift)
    static constexpr float WOW_WALK_SPEED = 5.0f;    // Walk (hold Ctrl)
    static constexpr float WOW_BACK_SPEED = 9.0f;    // Backpedal
    static constexpr float WOW_GRAVITY = -19.29f;
    static constexpr float WOW_JUMP_VELOCITY = 7.96f;

    // Default spawn position (Stormwind Trade District)
    glm::vec3 defaultPosition = glm::vec3(-8830.0f, 640.0f, 200.0f);
    float defaultYaw = 0.0f;   // Look north toward canals
    float defaultPitch = -5.0f;
};

} // namespace rendering
} // namespace wowee
