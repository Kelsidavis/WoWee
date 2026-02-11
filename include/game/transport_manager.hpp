#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace wowee::rendering {
    class WMORenderer;
}

namespace wowee::pipeline {
    class AssetManager;
}

namespace wowee::game {

struct TimedPoint {
    uint32_t tMs;          // Time in milliseconds from DBC
    glm::vec3 pos;         // Position at this time
};

struct TransportPath {
    uint32_t pathId;
    std::vector<TimedPoint> points;  // Time-indexed waypoints (includes duplicate first point at end for wrap)
    bool looping;  // Set to false after adding explicit wrap point
    uint32_t durationMs;   // Total loop duration in ms (includes wrap segment if added)
};

struct ActiveTransport {
    uint64_t guid;              // Entity GUID
    uint32_t wmoInstanceId;     // WMO renderer instance ID
    uint32_t pathId;            // Current path
    glm::vec3 basePosition;     // Spawn position (base offset for path)
    glm::vec3 position;         // Current world position
    glm::quat rotation;         // Current world rotation
    glm::mat4 transform;        // Cached world transform
    glm::mat4 invTransform;     // Cached inverse for collision

    // Player attachment (single-player for now)
    bool playerOnBoard;
    glm::vec3 playerLocalOffset;

    // Optional deck boundaries
    glm::vec3 deckMin;
    glm::vec3 deckMax;
    bool hasDeckBounds;

    // Time-based animation (deterministic, no drift)
    uint32_t localClockMs;         // Local path time in milliseconds
    bool hasServerClock;            // Whether we've synced with server time
    int32_t serverClockOffsetMs;   // Offset: serverClock - localNow
    bool useClientAnimation;        // Use client-side path animation
    float serverYaw;                // Server-authoritative yaw (radians)
    bool hasServerYaw;              // Whether we've received server yaw

    float lastServerUpdate;         // Time of last server movement update
    int serverUpdateCount;          // Number of server updates received
};

class TransportManager {
public:
    TransportManager();
    ~TransportManager();

    void setWMORenderer(rendering::WMORenderer* renderer) { wmoRenderer_ = renderer; }

    void update(float deltaTime);
    void registerTransport(uint64_t guid, uint32_t wmoInstanceId, uint32_t pathId, const glm::vec3& spawnWorldPos);
    void unregisterTransport(uint64_t guid);

    ActiveTransport* getTransport(uint64_t guid);
    glm::vec3 getPlayerWorldPosition(uint64_t transportGuid, const glm::vec3& localOffset);
    glm::mat4 getTransportInvTransform(uint64_t transportGuid);

    void loadPathFromNodes(uint32_t pathId, const std::vector<glm::vec3>& waypoints, bool looping = true, float speed = 18.0f);
    void setDeckBounds(uint64_t guid, const glm::vec3& min, const glm::vec3& max);

    // Load transport paths from TransportAnimation.dbc
    bool loadTransportAnimationDBC(pipeline::AssetManager* assetMgr);

    // Check if a path exists for a given GameObject entry
    bool hasPathForEntry(uint32_t entry) const;

    // Update server-controlled transport position/rotation directly (bypasses path movement)
    void updateServerTransport(uint64_t guid, const glm::vec3& position, float orientation);

    // Enable/disable client-side animation for transports without server updates
    void setClientSideAnimation(bool enabled) { clientSideAnimation_ = enabled; }
    bool isClientSideAnimation() const { return clientSideAnimation_; }

private:
    void updateTransportMovement(ActiveTransport& transport, float deltaTime);
    glm::vec3 evalTimedCatmullRom(const TransportPath& path, uint32_t pathTimeMs);
    glm::quat orientationFromTangent(const TransportPath& path, uint32_t pathTimeMs);
    void updateTransformMatrices(ActiveTransport& transport);

    std::unordered_map<uint64_t, ActiveTransport> transports_;
    std::unordered_map<uint32_t, TransportPath> paths_;  // Indexed by transportEntry (pathId from TransportAnimation.dbc)
    rendering::WMORenderer* wmoRenderer_ = nullptr;
    bool clientSideAnimation_ = true;  // Enable client animation - smooth movement, synced with server updates
    float elapsedTime_ = 0.0f;  // Total elapsed time (seconds)
};

} // namespace wowee::game
