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

        // Sanity check: firstWaypoint should match spawnWorldPos
        glm::vec3 firstWaypoint = path.points[0].pos;
        glm::vec3 waypointDiff = spawnWorldPos - firstWaypoint;
        if (glm::length(waypointDiff) > 1.0f) {
            LOG_WARNING("Transport 0x", std::hex, guid, std::dec, " path ", pathId,
                       ": firstWaypoint mismatch! spawnPos=(", spawnWorldPos.x, ",", spawnWorldPos.y, ",", spawnWorldPos.z, ")",
                       " firstWaypoint=(", firstWaypoint.x, ",", firstWaypoint.y, ",", firstWaypoint.z, ")",
                       " diff=(", waypointDiff.x, ",", waypointDiff.y, ",", waypointDiff.z, ")");
        }
    }

    transport.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion
    transport.playerOnBoard = false;
    transport.playerLocalOffset = glm::vec3(0.0f);
    transport.hasDeckBounds = false;
    transport.localClockMs = 0;
    transport.hasServerClock = false;
    transport.serverClockOffsetMs = 0;
    // Server-authoritative movement only - no client-side animation
    transport.useClientAnimation = false;
    transport.clientAnimationReverse = false;
    transport.serverYaw = 0.0f;
    transport.hasServerYaw = false;
    transport.lastServerUpdate = 0.0f;
    transport.serverUpdateCount = 0;
    transport.serverLinearVelocity = glm::vec3(0.0f);
    transport.serverAngularVelocity = 0.0f;
    transport.hasServerVelocity = false;

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
    path.zOnly = false;  // Manually loaded paths are assumed to have XY movement
    path.fromDBC = false;

    // Helper: compute segment duration from distance and speed
    auto segMsFromDist = [&](float dist) -> uint32_t {
        if (speed <= 0.0f) return 1000;
        return (uint32_t)((dist / speed) * 1000.0f);
    };

    // Single point = stationary (durationMs = 0)
    if (waypoints.size() == 1) {
        path.points.push_back({0, waypoints[0]});
        path.durationMs = 0;
        path.looping = false;
        paths_[pathId] = path;
        LOG_INFO("TransportManager: Loaded stationary path ", pathId);
        return;
    }

    // Multiple points: calculate cumulative time based on distance and speed
    path.points.reserve(waypoints.size() + (looping ? 1 : 0));
    uint32_t cumulativeMs = 0;
    path.points.push_back({0, waypoints[0]});

    for (size_t i = 1; i < waypoints.size(); i++) {
        float dist = glm::distance(waypoints[i-1], waypoints[i]);
        cumulativeMs += glm::max(1u, segMsFromDist(dist));
        path.points.push_back({cumulativeMs, waypoints[i]});
    }

    // Add explicit wrap segment (last → first) for looping paths
    if (looping) {
        float wrapDist = glm::distance(waypoints.back(), waypoints.front());
        uint32_t wrapMs = glm::max(1u, segMsFromDist(wrapDist));
        cumulativeMs += wrapMs;
        path.points.push_back({cumulativeMs, waypoints.front()});  // Duplicate first point
        path.looping = false;  // Time-closed path, no need for index wrapping
    } else {
        path.looping = false;
    }

    path.durationMs = cumulativeMs;
    paths_[pathId] = path;

    LOG_INFO("TransportManager: Loaded path ", pathId,
             " with ", waypoints.size(), " waypoints",
             (looping ? " + wrap segment" : ""),
             ", duration=", path.durationMs, "ms, speed=", speed);
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
        uint32_t dtMs = static_cast<uint32_t>(deltaTime * 1000.0f);
        if (!transport.clientAnimationReverse) {
            transport.localClockMs += dtMs;
        } else {
            if (dtMs > path.durationMs) {
                dtMs %= path.durationMs;
            }
            if (transport.localClockMs >= dtMs) {
                transport.localClockMs -= dtMs;
            } else {
                transport.localClockMs = path.durationMs - (dtMs - transport.localClockMs);
            }
        }
        pathTimeMs = transport.localClockMs % path.durationMs;
    } else {
        // Server-driven transport without clock sync:
        // stay server-authoritative and never switch to DBC/client animation fallback.
        // For short server update gaps, apply lightweight dead-reckoning only when we
        // have measured velocity from at least one authoritative delta.
        constexpr float kMaxExtrapolationSec = 8.0f;
        const float ageSec = elapsedTime_ - transport.lastServerUpdate;
        if (transport.hasServerVelocity && ageSec > 0.0f && ageSec <= kMaxExtrapolationSec) {
            const float blend = glm::clamp(1.0f - (ageSec / kMaxExtrapolationSec), 0.0f, 1.0f);
            transport.position += transport.serverLinearVelocity * (deltaTime * blend);
        }

        updateTransformMatrices(transport);
        if (wmoRenderer_) {
            wmoRenderer_->setInstanceTransform(transport.wmoInstanceId, transport.transform);
        }
        return;
    }

    // Evaluate position from time (path is local offsets, add base position)
    glm::vec3 pathOffset = evalTimedCatmullRom(path, pathTimeMs);
    // Guard against bad fallback Z curves on some remapped transport paths (notably icebreakers),
    // where path offsets can sink far below sea level when we only have spawn-time data.
    if (transport.useClientAnimation && transport.serverUpdateCount <= 1) {
        constexpr float kMinFallbackZOffset = -2.0f;
        pathOffset.z = glm::max(pathOffset.z, kMinFallbackZOffset);
    }
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
        LOG_DEBUG("Transport 0x", std::hex, transport.guid, std::dec,
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

    // Handle not found (timing gaps or past last segment)
    if (!found) {
        // For time-closed paths (explicit wrap point), last valid segment is points.size() - 2
        segmentIdx = (path.points.size() >= 2) ? (path.points.size() - 2) : 0;
    }

    size_t numPoints = path.points.size();

    // Get 4 control points for Catmull-Rom
    // Helper to clamp index (no wrapping for non-looping paths)
    auto idxClamp = [&](size_t i) -> size_t {
        return (i >= numPoints) ? (numPoints - 1) : i;
    };

    size_t p0Idx, p1Idx, p2Idx, p3Idx;
    p1Idx = segmentIdx;

    if (path.looping) {
        // Index-wrapped path (old DBC style with looping=true)
        p0Idx = (segmentIdx == 0) ? (numPoints - 1) : (segmentIdx - 1);
        p2Idx = (segmentIdx + 1) % numPoints;
        p3Idx = (segmentIdx + 2) % numPoints;
    } else {
        // Time-closed path (explicit wrap point at end, looping=false)
        // No index wrapping - points are sequential with possible duplicate at end
        p0Idx = (segmentIdx == 0) ? 0 : (segmentIdx - 1);
        p2Idx = idxClamp(segmentIdx + 1);
        p3Idx = idxClamp(segmentIdx + 2);
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

    // Handle not found (timing gaps or past last segment)
    if (!found) {
        // For time-closed paths (explicit wrap point), last valid segment is points.size() - 2
        segmentIdx = (path.points.size() >= 2) ? (path.points.size() - 2) : 0;
    }

    size_t numPoints = path.points.size();

    // Get 4 control points for tangent calculation
    // Helper to clamp index (no wrapping for non-looping paths)
    auto idxClamp = [&](size_t i) -> size_t {
        return (i >= numPoints) ? (numPoints - 1) : i;
    };

    size_t p0Idx, p1Idx, p2Idx, p3Idx;
    p1Idx = segmentIdx;

    if (path.looping) {
        // Index-wrapped path (old DBC style with looping=true)
        p0Idx = (segmentIdx == 0) ? (numPoints - 1) : (segmentIdx - 1);
        p2Idx = (segmentIdx + 1) % numPoints;
        p3Idx = (segmentIdx + 2) % numPoints;
    } else {
        // Time-closed path (explicit wrap point at end, looping=false)
        // No index wrapping - points are sequential with possible duplicate at end
        p0Idx = (segmentIdx == 0) ? 0 : (segmentIdx - 1);
        p2Idx = idxClamp(segmentIdx + 1);
        p3Idx = idxClamp(segmentIdx + 2);
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

    const bool hadPrevUpdate = (transport->serverUpdateCount > 0);
    const float prevUpdateTime = transport->lastServerUpdate;
    const glm::vec3 prevPos = transport->position;

    // Track server updates
    transport->serverUpdateCount++;
    transport->lastServerUpdate = elapsedTime_;
    transport->useClientAnimation = false;  // Server updates take precedence
    transport->clientAnimationReverse = false;

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

    // Server-authoritative transport mode:
    // Trust explicit server world position/orientation directly for all moving transports.
    // This avoids wrong-route and direction errors when local DBC path mapping differs from server route IDs.
    transport->hasServerClock = false;
    transport->useClientAnimation = false;
    if (transport->serverUpdateCount == 1) {
        // Seed once from first authoritative update; keep stable base for fallback phase estimation.
        transport->basePosition = position;
    }
    transport->position = position;
    transport->serverYaw = orientation;
    transport->hasServerYaw = true;
    transport->rotation = glm::angleAxis(transport->serverYaw, glm::vec3(0.0f, 0.0f, 1.0f));

    if (hadPrevUpdate) {
        const float dt = elapsedTime_ - prevUpdateTime;
        if (dt > 0.001f) {
            glm::vec3 v = (position - prevPos) / dt;
            const float speed = glm::length(v);
            constexpr float kMaxSpeed = 60.0f;
            if (speed > kMaxSpeed) {
                v *= (kMaxSpeed / speed);
            }

            transport->serverLinearVelocity = v;
            transport->serverAngularVelocity = 0.0f;
            transport->hasServerVelocity = true;
        }
    }

    updateTransformMatrices(*transport);
    if (wmoRenderer_) {
        wmoRenderer_->setInstanceTransform(transport->wmoInstanceId, transport->transform);
    }
    return;
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

        // DIAGNOSTIC: Log ALL records for problematic ferries (20655, 20657, 149046)
        // AND first few records for known-good transports to verify DBC reading
        if (i < 5 || transportEntry == 2074 ||
            transportEntry == 20655 || transportEntry == 20657 || transportEntry == 149046) {
            LOG_INFO("RAW DBC [", i, "] entry=", transportEntry, " t=", timeIndex,
                     " raw=(", posX, ",", posY, ",", posZ, ")");
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

            // CRITICAL: Detect if serverToCanonical is zeroing nonzero inputs
            if ((pos.x != 0.0f || pos.y != 0.0f || pos.z != 0.0f) &&
                (canonical.x == 0.0f && canonical.y == 0.0f && canonical.z == 0.0f)) {
                LOG_ERROR("serverToCanonical ZEROED! entry=", transportEntry,
                          " server=(", pos.x, ",", pos.y, ",", pos.z, ")",
                          " → canon=(", canonical.x, ",", canonical.y, ",", canonical.z, ")");
            }

            // Debug waypoint conversion for first transport (entry 2074)
            if (transportEntry == 2074 && idx < 5) {
                LOG_INFO("COORD CONVERT: entry=", transportEntry, " t=", tMs,
                         " serverPos=(", pos.x, ", ", pos.y, ", ", pos.z, ")",
                         " → canonical=(", canonical.x, ", ", canonical.y, ", ", canonical.z, ")");
            }

            // DIAGNOSTIC: Log ALL conversions for problematic ferries
            if (transportEntry == 20655 || transportEntry == 20657 || transportEntry == 149046) {
                LOG_INFO("CONVERT ", transportEntry, " t=", tMs,
                         " server=(", pos.x, ",", pos.y, ",", pos.z, ")",
                         " → canon=(", canonical.x, ",", canonical.y, ",", canonical.z, ")");
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

        // Detect Z-only paths (elevator/bobbing animation, not real XY travel)
        float minX = timedPoints[0].pos.x;
        float maxX = timedPoints[0].pos.x;
        float minY = timedPoints[0].pos.y;
        float maxY = timedPoints[0].pos.y;
        for (const auto& pt : timedPoints) {
            minX = std::min(minX, pt.pos.x);
            maxX = std::max(maxX, pt.pos.x);
            minY = std::min(minY, pt.pos.y);
            maxY = std::max(maxY, pt.pos.y);
        }
        float rangeX = maxX - minX;
        float rangeY = maxY - minY;
        bool isZOnly = (rangeX < 0.01f && rangeY < 0.01f);

        // Store path
        TransportPath path;
        path.pathId = transportEntry;
        path.points = timedPoints;
        // CRITICAL: We added an explicit wrap point (last → first), so this is TIME-CLOSED, not index-wrapped
        // Setting looping=false ensures evalTimedCatmullRom uses clamp logic (not modulo) for control points
        path.looping = false;
        path.durationMs = durationMs;
        path.zOnly = isZOnly;
        path.fromDBC = true;
        paths_[transportEntry] = path;
        pathsLoaded++;

        // Log first, middle, and last points to verify path data
        glm::vec3 firstOffset = timedPoints[0].pos;
        size_t midIdx = timedPoints.size() / 2;
        glm::vec3 midOffset = timedPoints[midIdx].pos;
        glm::vec3 lastOffset = timedPoints[timedPoints.size() - 2].pos;  // -2 to skip wrap duplicate
        LOG_INFO("  Transport ", transportEntry, ": ", timedPoints.size() - 1, " waypoints + wrap, ",
                 durationMs, "ms duration (wrap=", wrapMs, "ms, t0_normalized=", timedPoints[0].tMs, "ms)",
                 " rangeXY=(", rangeX, ",", rangeY, ") ", (isZOnly ? "[Z-ONLY]" : "[XY-PATH]"),
                 " firstOffset=(", firstOffset.x, ", ", firstOffset.y, ", ", firstOffset.z, ")",
                 " midOffset=(", midOffset.x, ", ", midOffset.y, ", ", midOffset.z, ")",
                 " lastOffset=(", lastOffset.x, ", ", lastOffset.y, ", ", lastOffset.z, ")");
    }

    LOG_INFO("Loaded ", pathsLoaded, " transport paths from TransportAnimation.dbc");
    return pathsLoaded > 0;
}

bool TransportManager::hasPathForEntry(uint32_t entry) const {
    auto it = paths_.find(entry);
    return it != paths_.end() && it->second.fromDBC;
}

bool TransportManager::hasUsableMovingPathForEntry(uint32_t entry, float minXYRange) const {
    auto it = paths_.find(entry);
    if (it == paths_.end()) return false;

    const auto& path = it->second;
    if (!path.fromDBC || path.points.size() < 2 || path.durationMs == 0 || path.zOnly) {
        return false;
    }

    float minX = path.points.front().pos.x;
    float maxX = minX;
    float minY = path.points.front().pos.y;
    float maxY = minY;
    for (const auto& p : path.points) {
        minX = std::min(minX, p.pos.x);
        maxX = std::max(maxX, p.pos.x);
        minY = std::min(minY, p.pos.y);
        maxY = std::max(maxY, p.pos.y);
    }

    float rangeXY = std::max(maxX - minX, maxY - minY);
    return rangeXY >= minXYRange;
}

uint32_t TransportManager::inferMovingPathForSpawn(const glm::vec3& spawnWorldPos, float maxDistance) const {
    float bestD2 = maxDistance * maxDistance;
    uint32_t bestPathId = 0;

    for (const auto& [pathId, path] : paths_) {
        if (!path.fromDBC || path.durationMs == 0 || path.zOnly || path.points.empty()) {
            continue;
        }

        // Find nearest waypoint on this path to spawn.
        for (const auto& p : path.points) {
            glm::vec3 diff = p.pos - spawnWorldPos;
            float d2 = glm::dot(diff, diff);
            if (d2 < bestD2) {
                bestD2 = d2;
                bestPathId = pathId;
            }
        }
    }

    if (bestPathId != 0) {
        LOG_INFO("TransportManager: Inferred moving DBC path ", bestPathId,
                 " for spawn at (", spawnWorldPos.x, ", ", spawnWorldPos.y, ", ", spawnWorldPos.z,
                 "), dist=", std::sqrt(bestD2));
    }

    return bestPathId;
}

uint32_t TransportManager::pickFallbackMovingPath(uint32_t entry, uint32_t displayId) const {
    auto isUsableMovingPath = [this](uint32_t pathId) -> bool {
        auto it = paths_.find(pathId);
        if (it == paths_.end()) return false;
        const auto& path = it->second;
        return path.fromDBC && !path.zOnly && path.durationMs > 0 && path.points.size() > 1;
    };

    // Known AzerothCore transport entry remaps (WotLK): server entry -> moving DBC path id.
    // These entries commonly do not match TransportAnimation.dbc ids 1:1.
    static const std::unordered_map<uint32_t, uint32_t> kEntryRemap = {
        {176231u, 176080u}, // The Maiden's Fancy
        {176310u, 176081u}, // The Bravery
        {20808u,  176082u}, // The Black Princess
        {164871u, 193182u}, // The Thundercaller
        {176495u, 193183u}, // The Purple Princess
        {175080u, 193182u}, // The Iron Eagle
        {181689u, 193183u}, // Cloudkisser
        {186238u, 193182u}, // The Mighty Wind
        {181688u, 176083u}, // Northspear (icebreaker)
        {190536u, 176084u}, // Stormwind's Pride (icebreaker)
    };

    auto itMapped = kEntryRemap.find(entry);
    if (itMapped != kEntryRemap.end() && isUsableMovingPath(itMapped->second)) {
        return itMapped->second;
    }

    // Fallback by display model family.
    const bool looksLikeShip =
        (displayId == 3015u || displayId == 2454u || displayId == 7446u || displayId == 455u || displayId == 462u);
    const bool looksLikeZeppelin =
        (displayId == 3031u || displayId == 7546u || displayId == 1587u || displayId == 807u || displayId == 808u);

    if (looksLikeShip) {
        static const uint32_t kShipCandidates[] = {176080u, 176081u, 176082u, 176083u, 176084u, 176085u, 194675u};
        for (uint32_t id : kShipCandidates) {
            if (isUsableMovingPath(id)) return id;
        }
    }

    if (looksLikeZeppelin) {
        static const uint32_t kZeppelinCandidates[] = {193182u, 193183u, 188360u, 190587u};
        for (uint32_t id : kZeppelinCandidates) {
            if (isUsableMovingPath(id)) return id;
        }
    }

    // Last-resort: pick any moving DBC path so transport does not remain stationary.
    for (const auto& [pathId, path] : paths_) {
        if (path.fromDBC && !path.zOnly && path.durationMs > 0 && path.points.size() > 1) {
            return pathId;
        }
    }

    return 0;
}

} // namespace wowee::game
