#include "game/transport_manager.hpp"
#include "rendering/wmo_renderer.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <iostream>
#include <map>
#include <algorithm>

namespace wowee::game {

TransportManager::TransportManager() = default;
TransportManager::~TransportManager() = default;

void TransportManager::update(float deltaTime) {
    elapsedTime_ += deltaTime;

    for (auto& [guid, transport] : transports_) {
        // Once we have server clock offset, we can predict server time indefinitely
        // No need for watchdog - keep using the offset even if server updates stop
        updateTransportMovement(transport, deltaTime);
    }
}

void TransportManager::registerTransport(uint64_t guid, uint32_t wmoInstanceId, uint32_t pathId, const glm::vec3& spawnWorldPos) {
    auto pathIt = paths_.find(pathId);
    if (pathIt == paths_.end()) {
        std::cerr << "TransportManager: Path " << pathId << " not found for transport " << guid << std::endl;
        return;
    }

    const auto& path = pathIt->second;
    if (path.points.empty()) {
        std::cerr << "TransportManager: Path " << pathId << " has no waypoints" << std::endl;
        return;
    }

    ActiveTransport transport;
    transport.guid = guid;
    transport.wmoInstanceId = wmoInstanceId;
    transport.pathId = pathId;

    // CRITICAL: Set basePosition from spawn position and t=0 offset
    // For stationary paths (1 waypoint), just use spawn position directly
    if (path.durationMs == 0 || path.points.size() <= 1) {
        // Stationary transport - no path animation
        transport.basePosition = spawnWorldPos;
        transport.position = spawnWorldPos;
    } else {
        // Moving transport - infer base from first path offset
        glm::vec3 offset0 = evalTimedCatmullRom(path, 0);
        transport.basePosition = spawnWorldPos - offset0;  // Infer base from spawn
        transport.position = spawnWorldPos;  // Start at spawn position (base + offset0)
    }

    transport.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion
    transport.playerOnBoard = false;
    transport.playerLocalOffset = glm::vec3(0.0f);
    transport.hasDeckBounds = false;
    transport.localClockMs = 0;
    transport.hasServerClock = false;
    transport.serverClockOffsetMs = 0;
    transport.useClientAnimation = clientSideAnimation_;  // Enable client animation by default
    transport.serverYaw = 0.0f;
    transport.hasServerYaw = false;
    transport.lastServerUpdate = 0.0f;
    transport.serverUpdateCount = 0;

    updateTransformMatrices(transport);

    // CRITICAL: Update WMO renderer with initial transform
    if (wmoRenderer_) {
        wmoRenderer_->setInstanceTransform(transport.wmoInstanceId, transport.transform);
    }

    transports_[guid] = transport;

    glm::vec3 renderPos = core::coords::canonicalToRender(transport.position);
    LOG_INFO("TransportManager: Registered transport 0x", std::hex, guid, std::dec,
             " at path ", pathId, " with ", path.points.size(), " waypoints",
             " wmoInstanceId=", wmoInstanceId,
             " spawnPos=(", spawnWorldPos.x, ", ", spawnWorldPos.y, ", ", spawnWorldPos.z, ")",
             " basePos=(", transport.basePosition.x, ", ", transport.basePosition.y, ", ", transport.basePosition.z, ")",
             " initialRenderPos=(", renderPos.x, ", ", renderPos.y, ", ", renderPos.z, ")");
}

void TransportManager::unregisterTransport(uint64_t guid) {
    transports_.erase(guid);
    std::cout << "TransportManager: Unregistered transport " << guid << std::endl;
}

ActiveTransport* TransportManager::getTransport(uint64_t guid) {
    auto it = transports_.find(guid);
    if (it != transports_.end()) {
        return &it->second;
    }
    return nullptr;
}

glm::vec3 TransportManager::getPlayerWorldPosition(uint64_t transportGuid, const glm::vec3& localOffset) {
    auto* transport = getTransport(transportGuid);
    if (!transport) {
        return localOffset;  // Fallback
    }

    glm::vec4 localPos(localOffset, 1.0f);
    glm::vec4 worldPos = transport->transform * localPos;
    return glm::vec3(worldPos);
}

glm::mat4 TransportManager::getTransportInvTransform(uint64_t transportGuid) {
    auto* transport = getTransport(transportGuid);
    if (!transport) {
        return glm::mat4(1.0f);  // Identity fallback
    }
    return transport->invTransform;
}

void TransportManager::loadPathFromNodes(uint32_t pathId, const std::vector<glm::vec3>& waypoints, bool looping, float speed) {
    if (waypoints.empty()) {
        std::cerr << "TransportManager: Cannot load empty path " << pathId << std::endl;
        return;
    }

    TransportPath path;
    path.pathId = pathId;
    path.looping = looping;

    // Convert waypoints to TimedPoint (for fallback paths, use arbitrary timing)
    // Single point = stationary (durationMs = 0)
    // Multiple points = assume constant speed
    path.points.reserve(waypoints.size());
    if (waypoints.size() == 1) {
        path.points.push_back({0, waypoints[0]});
        path.durationMs = 0;  // Stationary
    } else {
        // Calculate cumulative time based on distance and speed
        uint32_t cumulativeMs = 0;
        path.points.push_back({0, waypoints[0]});
        for (size_t i = 1; i < waypoints.size(); i++) {
            float dist = glm::distance(waypoints[i-1], waypoints[i]);
            uint32_t segmentMs = speed > 0.0f ? (uint32_t)((dist / speed) * 1000.0f) : 1000;
            cumulativeMs += segmentMs;
            path.points.push_back({cumulativeMs, waypoints[i]});
        }
        path.durationMs = cumulativeMs;
    }

    paths_[pathId] = path;

    LOG_INFO("TransportManager: Loaded path ", pathId,
             " with ", waypoints.size(), " waypoints, "
             "looping=", looping, ", speed=", speed);
}

void TransportManager::setDeckBounds(uint64_t guid, const glm::vec3& min, const glm::vec3& max) {
    auto* transport = getTransport(guid);
    if (!transport) {
        std::cerr << "TransportManager: Cannot set deck bounds for unknown transport " << guid << std::endl;
        return;
    }

    transport->deckMin = min;
    transport->deckMax = max;
    transport->hasDeckBounds = true;
}

void TransportManager::updateTransportMovement(ActiveTransport& transport, float deltaTime) {
    auto pathIt = paths_.find(transport.pathId);
    if (pathIt == paths_.end()) {
        return;
    }

    const auto& path = pathIt->second;
    if (path.points.empty()) {
        return;
    }

    // Stationary transport (durationMs = 0)
    if (path.durationMs == 0) {
        // Just update transform (position already set)
        updateTransformMatrices(transport);
        if (wmoRenderer_) {
            wmoRenderer_->setInstanceTransform(transport.wmoInstanceId, transport.transform);
        }
        return;
    }

    // Evaluate path time
    uint32_t nowMs = (uint32_t)(elapsedTime_ * 1000.0f);
    uint32_t pathTimeMs = 0;

    if (transport.hasServerClock) {
        // Predict server time using clock offset (works for both client and server-driven modes)
        int64_t serverTimeMs = (int64_t)nowMs + transport.serverClockOffsetMs;
        int64_t mod = (int64_t)path.durationMs;
        int64_t wrapped = serverTimeMs % mod;
        if (wrapped < 0) wrapped += mod;
        pathTimeMs = (uint32_t)wrapped;
    } else if (transport.useClientAnimation) {
        // Pure local clock (no server sync yet, client-driven)
        transport.localClockMs += (uint32_t)(deltaTime * 1000.0f);
        pathTimeMs = transport.localClockMs % path.durationMs;
    } else {
        // Server-driven but no clock yet - don't move
        updateTransformMatrices(transport);
        if (wmoRenderer_) {
            wmoRenderer_->setInstanceTransform(transport.wmoInstanceId, transport.transform);
        }
        return;
    }

    // Evaluate position from time (path is local offsets, add base position)
    glm::vec3 pathOffset = evalTimedCatmullRom(path, pathTimeMs);
    transport.position = transport.basePosition + pathOffset;

    // Use server yaw if available (authoritative), otherwise compute from tangent
    if (transport.hasServerYaw) {
        transport.rotation = glm::angleAxis(transport.serverYaw, glm::vec3(0.0f, 0.0f, 1.0f));
    } else {
        transport.rotation = orientationFromTangent(path, pathTimeMs);
    }

    // Update transform matrices
    updateTransformMatrices(transport);

    // Update WMO instance position
    if (wmoRenderer_) {
        wmoRenderer_->setInstanceTransform(transport.wmoInstanceId, transport.transform);
    }

    // Debug logging every 120 frames (~2 seconds at 60fps)
    static int debugFrameCount = 0;
    if (debugFrameCount++ % 120 == 0) {
        // Log canonical position AND render position to check coordinate conversion
        glm::vec3 renderPos = core::coords::canonicalToRender(transport.position);
        LOG_INFO("Transport 0x", std::hex, transport.guid, std::dec,
                 " pathTime=", pathTimeMs, "ms / ", path.durationMs, "ms",
                 " canonicalPos=(", transport.position.x, ", ", transport.position.y, ", ", transport.position.z, ")",
                 " renderPos=(", renderPos.x, ", ", renderPos.y, ", ", renderPos.z, ")",
                 " basePos=(", transport.basePosition.x, ", ", transport.basePosition.y, ", ", transport.basePosition.z, ")",
                 " pathOffset=(", pathOffset.x, ", ", pathOffset.y, ", ", pathOffset.z, ")",
                 " mode=", (transport.useClientAnimation ? "client" : "server"),
                 " hasServerClock=", transport.hasServerClock,
                 " offset=", transport.serverClockOffsetMs, "ms");
    }
}

glm::vec3 TransportManager::evalTimedCatmullRom(const TransportPath& path, uint32_t pathTimeMs) {
    if (path.points.empty()) {
        return glm::vec3(0.0f);
    }
    if (path.points.size() == 1) {
        return path.points[0].pos;
    }

    // Find the segment containing pathTimeMs
    size_t segmentIdx = 0;
    bool found = false;

    for (size_t i = 0; i + 1 < path.points.size(); i++) {
        if (pathTimeMs >= path.points[i].tMs && pathTimeMs < path.points[i + 1].tMs) {
            segmentIdx = i;
            found = true;
            break;
        }
    }

    // Handle not found (wraparound or timing gaps)
    if (!found) {
        segmentIdx = path.looping ? (path.points.size() - 1) :
                     (path.points.size() >= 2 ? path.points.size() - 2 : 0);
    }

    size_t numPoints = path.points.size();

    // Get 4 control points for Catmull-Rom
    size_t p0Idx = (segmentIdx == 0) ? (path.looping ? numPoints - 1 : 0) : segmentIdx - 1;
    size_t p1Idx = segmentIdx;
    size_t p2Idx = (segmentIdx + 1) % numPoints;
    size_t p3Idx = (segmentIdx + 2) % numPoints;

    if (!path.looping) {
        if (segmentIdx == 0) p0Idx = 0;
        if (segmentIdx >= numPoints - 2) p3Idx = numPoints - 1;
        if (segmentIdx >= numPoints - 1) p2Idx = numPoints - 1;
    }

    glm::vec3 p0 = path.points[p0Idx].pos;
    glm::vec3 p1 = path.points[p1Idx].pos;
    glm::vec3 p2 = path.points[p2Idx].pos;
    glm::vec3 p3 = path.points[p3Idx].pos;

    // Calculate t (0.0 to 1.0 within segment)
    // No special case needed - wrap point is explicit in the array now
    uint32_t t1Ms = path.points[p1Idx].tMs;
    uint32_t t2Ms = path.points[p2Idx].tMs;
    uint32_t segmentDurationMs = (t2Ms > t1Ms) ? (t2Ms - t1Ms) : 1;
    float t = (float)(pathTimeMs - t1Ms) / (float)segmentDurationMs;
    t = glm::clamp(t, 0.0f, 1.0f);

    // Catmull-Rom spline formula
    float t2 = t * t;
    float t3 = t2 * t;

    glm::vec3 result = 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
    );

    return result;
}

glm::quat TransportManager::orientationFromTangent(const TransportPath& path, uint32_t pathTimeMs) {
    if (path.points.empty()) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    if (path.points.size() == 1) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    // Find the segment containing pathTimeMs
    size_t segmentIdx = 0;
    bool found = false;

    for (size_t i = 0; i + 1 < path.points.size(); i++) {
        if (pathTimeMs >= path.points[i].tMs && pathTimeMs < path.points[i + 1].tMs) {
            segmentIdx = i;
            found = true;
            break;
        }
    }

    // Handle not found (wraparound or timing gaps)
    if (!found) {
        segmentIdx = path.looping ? (path.points.size() - 1) :
                     (path.points.size() >= 2 ? path.points.size() - 2 : 0);
    }

    size_t numPoints = path.points.size();

    // Get 4 control points
    size_t p0Idx = (segmentIdx == 0) ? (path.looping ? numPoints - 1 : 0) : segmentIdx - 1;
    size_t p1Idx = segmentIdx;
    size_t p2Idx = (segmentIdx + 1) % numPoints;
    size_t p3Idx = (segmentIdx + 2) % numPoints;

    if (!path.looping) {
        if (segmentIdx == 0) p0Idx = 0;
        if (segmentIdx >= numPoints - 2) p3Idx = numPoints - 1;
        if (segmentIdx >= numPoints - 1) p2Idx = numPoints - 1;
    }

    glm::vec3 p0 = path.points[p0Idx].pos;
    glm::vec3 p1 = path.points[p1Idx].pos;
    glm::vec3 p2 = path.points[p2Idx].pos;
    glm::vec3 p3 = path.points[p3Idx].pos;

    // Calculate t (0.0 to 1.0 within segment)
    // No special case needed - wrap point is explicit in the array now
    uint32_t t1Ms = path.points[p1Idx].tMs;
    uint32_t t2Ms = path.points[p2Idx].tMs;
    uint32_t segmentDurationMs = (t2Ms > t1Ms) ? (t2Ms - t1Ms) : 1;
    float t = (float)(pathTimeMs - t1Ms) / (float)segmentDurationMs;
    t = glm::clamp(t, 0.0f, 1.0f);

    // Tangent of Catmull-Rom spline (derivative)
    float t2 = t * t;
    glm::vec3 tangent = 0.5f * (
        (-p0 + p2) +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * 2.0f * t +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * 3.0f * t2
    );

    // Normalize tangent
    float tangentLength = glm::length(tangent);
    if (tangentLength < 0.001f) {
        // Fallback to simple direction
        tangent = p2 - p1;
        tangentLength = glm::length(tangent);
    }

    if (tangentLength < 0.001f) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    }

    tangent /= tangentLength;

    // Calculate rotation from forward direction
    glm::vec3 forward = tangent;
    glm::vec3 up(0.0f, 0.0f, 1.0f);  // WoW Z is up

    // If forward is nearly vertical, use different up vector
    if (std::abs(forward.z) > 0.99f) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 right = glm::normalize(glm::cross(up, forward));
    up = glm::cross(forward, right);

    // Build rotation matrix and convert to quaternion
    glm::mat3 rotMat;
    rotMat[0] = right;
    rotMat[1] = forward;
    rotMat[2] = up;

    return glm::quat_cast(rotMat);
}

void TransportManager::updateTransformMatrices(ActiveTransport& transport) {
    // Convert position from canonical to render coordinates for WMO rendering
    // Canonical: +X=North, +Y=West, +Z=Up
    // Render: renderX=wowY (west), renderY=wowX (north), renderZ=wowZ (up)
    glm::vec3 renderPos = core::coords::canonicalToRender(transport.position);

    // Convert rotation from canonical to render space using proper basis change
    // Canonical → Render is a 90° CCW rotation around Z (swaps X and Y)
    // Proper formula: q_render = q_basis * q_canonical * q_basis^-1
    glm::quat basisRotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::quat basisInverse = glm::conjugate(basisRotation);
    glm::quat renderRot = basisRotation * transport.rotation * basisInverse;

    // Build transform matrix: translate * rotate * scale
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), renderPos);
    glm::mat4 rotation = glm::mat4_cast(renderRot);
    glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f));  // No scaling for transports

    transport.transform = translation * rotation * scale;
    transport.invTransform = glm::inverse(transport.transform);
}

void TransportManager::updateServerTransport(uint64_t guid, const glm::vec3& position, float orientation) {
    auto* transport = getTransport(guid);
    if (!transport) {
        LOG_WARNING("TransportManager::updateServerTransport: Transport not found: 0x", std::hex, guid, std::dec);
        return;
    }

    // Track server updates
    transport->serverUpdateCount++;
    transport->lastServerUpdate = elapsedTime_;

    auto pathIt = paths_.find(transport->pathId);
    if (pathIt == paths_.end() || pathIt->second.durationMs == 0) {
        // No path or stationary - just set position directly
        transport->basePosition = position;
        transport->position = position;
        transport->rotation = glm::angleAxis(orientation, glm::vec3(0.0f, 0.0f, 1.0f));
        updateTransformMatrices(*transport);
        if (wmoRenderer_) {
            wmoRenderer_->setInstanceTransform(transport->wmoInstanceId, transport->transform);
        }
        return;
    }

    const auto& path = pathIt->second;

    // Seed basePosition from t=0 assumption before first search
    // (t=0 corresponds to spawn point / first path point)
    if (!transport->hasServerClock) {
        glm::vec3 offset0 = evalTimedCatmullRom(path, 0);
        transport->basePosition = position - offset0;
    }

    // Estimate server's path time by projecting position onto path
    // Path positions are local offsets, server position is world position
    // basePosition = serverWorldPos - pathLocalOffset

    uint32_t bestTimeMs = 0;
    float bestD2 = FLT_MAX;
    glm::vec3 bestPathOffset(0.0f);

    // After initial sync, search only in small window around predicted time
    bool hasInitialSync = transport->hasServerClock;
    uint32_t nowMs = (uint32_t)(elapsedTime_ * 1000.0f);
    uint32_t predictedTimeMs = 0;
    if (hasInitialSync) {
        // Predict where server should be based on last clock offset
        int64_t serverTimeMs = (int64_t)nowMs + transport->serverClockOffsetMs;
        int64_t mod = (int64_t)path.durationMs;
        int64_t wrapped = serverTimeMs % mod;
        if (wrapped < 0) wrapped += mod;
        predictedTimeMs = (uint32_t)wrapped;
    }

    uint32_t searchStart = 0;
    uint32_t searchEnd = path.durationMs;
    uint32_t sampleCount = 1000;  // Dense sampling for accuracy

    if (hasInitialSync) {
        // Search in ±5 second window around predicted time
        uint32_t windowMs = 5000;
        searchStart = (predictedTimeMs > windowMs) ? (predictedTimeMs - windowMs) : 0;
        searchEnd = glm::min(predictedTimeMs + windowMs, path.durationMs);
        sampleCount = 200;  // Fewer samples needed in small window
    }

    for (uint32_t i = 0; i < sampleCount; i++) {
        // Map i to [searchStart, searchEnd)
        uint32_t testTimeMs = searchStart + (uint32_t)((uint64_t)i * (searchEnd - searchStart) / sampleCount);
        glm::vec3 testPathOffset = evalTimedCatmullRom(path, testTimeMs);
        glm::vec3 testWorldPos = transport->basePosition + testPathOffset;  // Convert local → world
        glm::vec3 diff = testWorldPos - position;
        float d2 = glm::dot(diff, diff);  // distance² (cheaper, no sqrt)
        if (d2 < bestD2) {
            bestD2 = d2;
            bestTimeMs = testTimeMs;
            bestPathOffset = testPathOffset;
        }
    }

    // Refine with finer sampling around best match
    uint32_t refineSampleCount = 50;
    uint32_t refineWindow = glm::max(1u, (searchEnd - searchStart) / sampleCount);  // Clamp to prevent zero
    uint32_t refineStart = (bestTimeMs > refineWindow) ? (bestTimeMs - refineWindow) : 0;
    uint32_t refineEnd = glm::min(bestTimeMs + refineWindow, path.durationMs);
    uint32_t refineInterval = (refineEnd > refineStart) ? ((refineEnd - refineStart) / refineSampleCount) : 1;
    if (refineInterval > 0) {
        for (uint32_t i = 0; i < refineSampleCount; i++) {
            uint32_t testTimeMs = refineStart + i * refineInterval;
            glm::vec3 testPathOffset = evalTimedCatmullRom(path, testTimeMs);  // local offset
            glm::vec3 testWorldPos = transport->basePosition + testPathOffset;  // Convert local → world
            glm::vec3 diff = testWorldPos - position;  // Compare world to world
            float d2 = glm::dot(diff, diff);
            if (d2 < bestD2) {
                bestD2 = d2;
                bestTimeMs = testTimeMs;
                bestPathOffset = testPathOffset;  // Update best offset when improving match
            }
        }
    }

    float bestDistance = std::sqrt(bestD2);

    // Infer base position: serverWorldPos = basePos + pathOffset
    // So: basePos = serverWorldPos - pathOffset
    glm::vec3 inferredBasePos = position - bestPathOffset;

    // Compute server clock offset with wrap-aware smoothing
    int32_t newOffset = (int32_t)bestTimeMs - (int32_t)nowMs;

    if (!transport->hasServerClock) {
        // First sync: accept immediately and set base position
        transport->basePosition = inferredBasePos;
        transport->serverClockOffsetMs = newOffset;
        transport->hasServerClock = true;
        LOG_INFO("TransportManager: Initial server clock sync for transport 0x", std::hex, guid, std::dec,
                 " serverTime=", bestTimeMs, "ms / ", path.durationMs, "ms",
                 " drift=", bestDistance, " units",
                 " basePos=(", inferredBasePos.x, ", ", inferredBasePos.y, ", ", inferredBasePos.z, ")",
                 " offset=", newOffset, "ms");
    } else {
        // Subsequent syncs: wrap-aware smoothing to avoid phase jumps
        int32_t oldOffset = transport->serverClockOffsetMs;
        int32_t delta = newOffset - oldOffset;
        int32_t mod = (int32_t)path.durationMs;

        // Wrap delta to shortest path: [-mod/2, mod/2]
        if (delta > mod / 2) delta -= mod;
        if (delta < -mod / 2) delta += mod;

        // Smooth delta, not absolute offset
        transport->serverClockOffsetMs = oldOffset + (int32_t)(0.1f * delta);

        // Only update basePosition if projection is accurate (< 5 units drift)
        // This prevents "swim" from projection noise near ambiguous geometry
        if (bestDistance < 5.0f) {
            transport->basePosition = glm::mix(transport->basePosition, inferredBasePos, 0.1f);
            LOG_INFO("TransportManager: Server clock correction for transport 0x", std::hex, guid, std::dec,
                     " drift=", bestDistance, " units (updated base)",
                     " oldOffset=", oldOffset, "ms → newOffset=", transport->serverClockOffsetMs, "ms",
                     " (delta=", delta, "ms, smoothed by 0.1)");
        } else {
            LOG_INFO("TransportManager: Server clock correction for transport 0x", std::hex, guid, std::dec,
                     " drift=", bestDistance, " units (base unchanged, clock only)",
                     " oldOffset=", oldOffset, "ms → newOffset=", transport->serverClockOffsetMs, "ms",
                     " (delta=", delta, "ms, smoothed by 0.1)");
        }
    }

    // Update position immediately from synced clock
    glm::vec3 pathOffset = evalTimedCatmullRom(path, bestTimeMs);
    transport->position = transport->basePosition + pathOffset;

    // Store server's authoritative yaw (orientation is in radians around Z axis)
    transport->serverYaw = orientation;
    transport->hasServerYaw = true;
    transport->rotation = glm::angleAxis(transport->serverYaw, glm::vec3(0.0f, 0.0f, 1.0f));

    updateTransformMatrices(*transport);
    if (wmoRenderer_) {
        wmoRenderer_->setInstanceTransform(transport->wmoInstanceId, transport->transform);
    }
}

bool TransportManager::loadTransportAnimationDBC(pipeline::AssetManager* assetMgr) {
    LOG_INFO("Loading TransportAnimation.dbc...");

    if (!assetMgr) {
        LOG_ERROR("AssetManager is null");
        return false;
    }

    // Load DBC file
    auto dbcData = assetMgr->readFile("DBFilesClient\\TransportAnimation.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("TransportAnimation.dbc not found - transports will use fallback paths");
        return false;
    }

    pipeline::DBCFile dbc;
    if (!dbc.load(dbcData)) {
        LOG_ERROR("Failed to parse TransportAnimation.dbc");
        return false;
    }

    LOG_INFO("TransportAnimation.dbc: ", dbc.getRecordCount(), " records, ",
             dbc.getFieldCount(), " fields per record");

    // Debug: dump first 3 records to see all field values
    for (uint32_t i = 0; i < std::min(3u, dbc.getRecordCount()); i++) {
        LOG_INFO("  DEBUG Record ", i, ": ",
                 " [0]=", dbc.getUInt32(i, 0),
                 " [1]=", dbc.getUInt32(i, 1),
                 " [2]=", dbc.getUInt32(i, 2),
                 " [3]=", dbc.getFloat(i, 3),
                 " [4]=", dbc.getFloat(i, 4),
                 " [5]=", dbc.getFloat(i, 5),
                 " [6]=", dbc.getUInt32(i, 6));
    }

    // Group waypoints by transportEntry
    std::map<uint32_t, std::vector<std::pair<uint32_t, glm::vec3>>> waypointsByTransport;

    for (uint32_t i = 0; i < dbc.getRecordCount(); i++) {
        // uint32_t id = dbc.getUInt32(i, 0);  // Not needed
        uint32_t transportEntry = dbc.getUInt32(i, 1);
        uint32_t timeIndex = dbc.getUInt32(i, 2);
        float posX = dbc.getFloat(i, 3);
        float posY = dbc.getFloat(i, 4);
        float posZ = dbc.getFloat(i, 5);
        // uint32_t sequenceId = dbc.getUInt32(i, 6);  // Not needed for basic paths

        // RAW FLOAT SANITY CHECK: Log first 10 records to see if DBC has real data
        if (i < 10) {
            uint32_t ux = dbc.getUInt32(i, 3);
            uint32_t uy = dbc.getUInt32(i, 4);
            uint32_t uz = dbc.getUInt32(i, 5);
            LOG_INFO("TA raw rec ", i,
                     " entry=", transportEntry,
                     " t=", timeIndex,
                     " raw=(", posX, ",", posY, ",", posZ, ")",
                     " u32=(", ux, ",", uy, ",", uz, ")");
        }

        waypointsByTransport[transportEntry].push_back({timeIndex, glm::vec3(posX, posY, posZ)});
    }

    // Create time-indexed paths from waypoints
    int pathsLoaded = 0;
    for (const auto& [transportEntry, waypoints] : waypointsByTransport) {
        if (waypoints.empty()) continue;

        // Sort by timeIndex
        auto sortedWaypoints = waypoints;
        std::sort(sortedWaypoints.begin(), sortedWaypoints.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // CRITICAL: Normalize timeIndex to start at 0 (DBC records don't start at 0!)
        // This makes evalTimedCatmullRom(path, 0) valid and stabilizes basePosition seeding
        uint32_t t0 = sortedWaypoints.front().first;

        // Build TimedPoint array with normalized time indices
        std::vector<TimedPoint> timedPoints;
        timedPoints.reserve(sortedWaypoints.size() + 1);  // +1 for wrap point

        // Log first few waypoints for transport 2074 to see conversion
        for (size_t idx = 0; idx < sortedWaypoints.size(); idx++) {
            const auto& [tMs, pos] = sortedWaypoints[idx];

            // TransportAnimation.dbc uses server coordinates - convert to canonical
            glm::vec3 canonical = core::coords::serverToCanonical(pos);

            // Debug waypoint conversion for first transport (entry 2074)
            if (transportEntry == 2074 && idx < 5) {
                LOG_INFO("COORD CONVERT: entry=", transportEntry, " t=", tMs,
                         " serverPos=(", pos.x, ", ", pos.y, ", ", pos.z, ")",
                         " → canonical=(", canonical.x, ", ", canonical.y, ", ", canonical.z, ")");
            }

            timedPoints.push_back({tMs - t0, canonical});  // Normalize: subtract first timeIndex
        }

        // Get base duration from last normalized timeIndex
        uint32_t lastTimeMs = sortedWaypoints.back().first - t0;

        // Calculate wrap duration (last → first segment)
        // Use average segment duration as wrap duration
        uint32_t totalDelta = 0;
        int segmentCount = 0;
        for (size_t i = 1; i < sortedWaypoints.size(); i++) {
            uint32_t delta = sortedWaypoints[i].first - sortedWaypoints[i-1].first;
            if (delta > 0) {
                totalDelta += delta;
                segmentCount++;
            }
        }
        uint32_t wrapMs = (segmentCount > 0) ? (totalDelta / segmentCount) : 1000;

        // Add duplicate first point at end with wrap duration
        // This makes the wrap segment (last → first) have proper duration
        glm::vec3 firstCanonical = core::coords::serverToCanonical(sortedWaypoints.front().second);
        timedPoints.push_back({lastTimeMs + wrapMs, firstCanonical});

        uint32_t durationMs = lastTimeMs + wrapMs;

        // Store path
        TransportPath path;
        path.pathId = transportEntry;
        path.points = timedPoints;
        // Keep looping=true even with duplicate wrap point for smooth control point selection at seam
        // This prevents kinks on the last segment approaching the wrap
        path.looping = true;
        path.durationMs = durationMs;
        paths_[transportEntry] = path;
        pathsLoaded++;

        // Log first, middle, and last points to verify path data
        glm::vec3 firstOffset = timedPoints[0].pos;
        size_t midIdx = timedPoints.size() / 2;
        glm::vec3 midOffset = timedPoints[midIdx].pos;
        glm::vec3 lastOffset = timedPoints[timedPoints.size() - 2].pos;  // -2 to skip wrap duplicate
        LOG_INFO("  Transport ", transportEntry, ": ", timedPoints.size() - 1, " waypoints + wrap, ",
                 durationMs, "ms duration (wrap=", wrapMs, "ms, t0_normalized=", timedPoints[0].tMs, "ms)",
                 " firstOffset=(", firstOffset.x, ", ", firstOffset.y, ", ", firstOffset.z, ")",
                 " midOffset=(", midOffset.x, ", ", midOffset.y, ", ", midOffset.z, ")",
                 " lastOffset=(", lastOffset.x, ", ", lastOffset.y, ", ", lastOffset.z, ")");
    }

    LOG_INFO("Loaded ", pathsLoaded, " transport paths from TransportAnimation.dbc");
    return pathsLoaded > 0;
}

bool TransportManager::hasPathForEntry(uint32_t entry) const {
    return paths_.find(entry) != paths_.end();
}

} // namespace wowee::game
