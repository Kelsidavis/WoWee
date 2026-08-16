// src/math/spline.cpp
// Standalone Catmull-Rom spline implementation.
// Ported from TransportManager::evalTimedCatmullRom() + orientationFromTangent()
// with improvements: binary search segment lookup, combined position+tangent eval.
#include "math/spline.hpp"
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

namespace wowee::math {

CatmullRomSpline::CatmullRomSpline(std::vector<SplineKey> keys, bool timeClosed)
    : keys_(std::move(keys))
    , timeClosed_(timeClosed)
{
    if (keys_.size() >= 2) {
        durationMs_ = keys_.back().timeMs - keys_.front().timeMs;
        if (durationMs_ == 0) {
            durationMs_ = 1; // Avoid division by zero
        }
    }
}

glm::vec3 CatmullRomSpline::evaluatePosition(uint32_t pathTimeMs) const {
    if (keys_.empty()) {
        return glm::vec3(0.0f);
    }
    if (keys_.size() == 1) {
        return keys_[0].position;
    }

    // Position-only path: skip the tangent computation that evaluate() always
    // does. Callers that don't need orientation (entity movement interp, etc.)
    // were paying for the derivative every call.
    size_t segIdx = findSegment(pathTimeMs);
    uint32_t t1Ms = keys_[segIdx].timeMs;
    uint32_t t2Ms = keys_[segIdx + 1].timeMs;
    uint32_t segDuration = (t2Ms > t1Ms) ? (t2Ms - t1Ms) : 1;
    // Repeated positions encode an intentional stop/dwell. A normal Catmull-Rom
    // segment with p1 == p2 still bows away from the point because p0/p3 influence
    // it, making a supposedly docked transport drift and turn in place.
    if (glm::dot(keys_[segIdx + 1].position - keys_[segIdx].position,
                 keys_[segIdx + 1].position - keys_[segIdx].position) < 1e-8f) {
        return keys_[segIdx].position;
    }
    float t = glm::clamp(static_cast<float>(pathTimeMs - t1Ms)
                       / static_cast<float>(segDuration), 0.0f, 1.0f);
    float t2 = t * t;
    float t3 = t2 * t;
    ControlPoints cp = getControlPoints(segIdx);
    return 0.5f * (
        (2.0f * cp.p1) +
        (-cp.p0 + cp.p2) * t +
        (2.0f * cp.p0 - 5.0f * cp.p1 + 4.0f * cp.p2 - cp.p3) * t2 +
        (-cp.p0 + 3.0f * cp.p1 - 3.0f * cp.p2 + cp.p3) * t3
    );
}

SplineEvalResult CatmullRomSpline::evaluate(uint32_t pathTimeMs) const {
    if (keys_.empty()) {
        return {glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f)};
    }
    if (keys_.size() == 1) {
        return {keys_[0].position, glm::vec3(0.0f, 1.0f, 0.0f)};
    }

    // Find the segment containing pathTimeMs (binary search, O(log n))
    size_t segIdx = findSegment(pathTimeMs);

    // Calculate t (0.0 to 1.0 within segment)
    uint32_t t1Ms = keys_[segIdx].timeMs;
    uint32_t t2Ms = keys_[segIdx + 1].timeMs;
    uint32_t segDuration = (t2Ms > t1Ms) ? (t2Ms - t1Ms) : 1;

    if (glm::dot(keys_[segIdx + 1].position - keys_[segIdx].position,
                 keys_[segIdx + 1].position - keys_[segIdx].position) < 1e-8f) {
        return {keys_[segIdx].position, glm::vec3(0.0f)};
    }

    float t = static_cast<float>(pathTimeMs - t1Ms)
            / static_cast<float>(segDuration);
    t = glm::clamp(t, 0.0f, 1.0f);

    // Get 4 control points and evaluate
    ControlPoints cp = getControlPoints(segIdx);
    return evalSegment(cp, t);
}

glm::quat CatmullRomSpline::orientationFromTangent(const glm::vec3& tangent) {
    // Normalize tangent
    float tangentLenSq = glm::dot(tangent, tangent);
    if (tangentLenSq < 1e-6f) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // Identity
    }

    glm::vec3 forward = tangent * glm::inversesqrt(tangentLenSq);
    glm::vec3 up(0.0f, 0.0f, 1.0f); // WoW Z is up

    // If forward is nearly vertical, use different up vector
    if (std::abs(forward.z) > 0.99f) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 right = glm::normalize(glm::cross(forward, up));
    up = glm::normalize(glm::cross(right, forward));

    // Build rotation matrix and convert to quaternion
    glm::mat3 rotMat;
    rotMat[0] = right;
    rotMat[1] = forward;
    rotMat[2] = up;

    return glm::quat_cast(rotMat);
}

bool CatmullRomSpline::hasXYMovement(float minRange) const {
    if (keys_.size() < 2) return false;

    float minX = keys_[0].position.x, maxX = minX;
    float minY = keys_[0].position.y, maxY = minY;

    for (size_t i = 1; i < keys_.size(); ++i) {
        minX = std::min(minX, keys_[i].position.x);
        maxX = std::max(maxX, keys_[i].position.x);
        minY = std::min(minY, keys_[i].position.y);
        maxY = std::max(maxY, keys_[i].position.y);
    }

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    return (rangeX >= minRange || rangeY >= minRange);
}

size_t CatmullRomSpline::findNearestKey(const glm::vec3& position) const {
    if (keys_.empty()) return 0;

    size_t nearest = 0;
    float bestDistSq = glm::dot(position - keys_[0].position,
                                 position - keys_[0].position);

    for (size_t i = 1; i < keys_.size(); ++i) {
        glm::vec3 diff = position - keys_[i].position;
        float distSq = glm::dot(diff, diff);
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            nearest = i;
        }
    }

    return nearest;
}

size_t CatmullRomSpline::findSegment(uint32_t pathTimeMs) const {
    // Binary search for the segment containing pathTimeMs.
    // Segment i spans [keys_[i].timeMs, keys_[i+1].timeMs).
    // We need the largest i such that keys_[i].timeMs <= pathTimeMs.

    if (keys_.size() < 2) return 0;

    // Clamp to valid range
    if (pathTimeMs <= keys_.front().timeMs) return 0;
    if (pathTimeMs >= keys_.back().timeMs) return keys_.size() - 2;

    // Binary search: find rightmost key where timeMs <= pathTimeMs
    size_t lo = 0;
    size_t hi = keys_.size() - 1;

    while (lo + 1 < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (keys_[mid].timeMs <= pathTimeMs) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return lo;
}

CatmullRomSpline::ControlPoints CatmullRomSpline::getControlPoints(size_t segIdx) const {
    // Ported from TransportManager::evalTimedCatmullRom control point logic
    size_t numPoints = keys_.size();

    auto idxClamp = [numPoints](size_t i) -> size_t {
        return (i >= numPoints) ? (numPoints - 1) : i;
    };

    size_t p0Idx, p1Idx, p2Idx, p3Idx;
    p1Idx = segIdx;

    if (timeClosed_) {
        // Time-closed path: index wraps around (looping transport)
        p0Idx = (segIdx == 0) ? (numPoints - 1) : (segIdx - 1);
        p2Idx = (segIdx + 1) % numPoints;
        p3Idx = (segIdx + 2) % numPoints;
    } else {
        // Clamped endpoints (non-looping path)
        p0Idx = (segIdx == 0) ? 0 : (segIdx - 1);
        p2Idx = idxClamp(segIdx + 1);
        p3Idx = idxClamp(segIdx + 2);
    }

    return {
        keys_[p0Idx].position,
        keys_[p1Idx].position,
        keys_[p2Idx].position,
        keys_[p3Idx].position
    };
}

SplineEvalResult CatmullRomSpline::evalSegment(
    const ControlPoints& cp, float t) const
{
    // Standard Catmull-Rom spline formula (from TransportManager::evalTimedCatmullRom)
    float t2 = t * t;
    float t3 = t2 * t;

    // Position: 0.5 * ((2*p1) + (-p0+p2)*t + (2p0-5p1+4p2-p3)*t² + (-p0+3p1-3p2+p3)*t³)
    glm::vec3 position = 0.5f * (
        (2.0f * cp.p1) +
        (-cp.p0 + cp.p2) * t +
        (2.0f * cp.p0 - 5.0f * cp.p1 + 4.0f * cp.p2 - cp.p3) * t2 +
        (-cp.p0 + 3.0f * cp.p1 - 3.0f * cp.p2 + cp.p3) * t3
    );

    // Tangent (derivative): 0.5 * ((-p0+p2) + (2p0-5p1+4p2-p3)*2t + (-p0+3p1-3p2+p3)*3t²)
    // Ported from TransportManager::orientationFromTangent
    glm::vec3 tangent = 0.5f * (
        (-cp.p0 + cp.p2) +
        (2.0f * cp.p0 - 5.0f * cp.p1 + 4.0f * cp.p2 - cp.p3) * 2.0f * t +
        (-cp.p0 + 3.0f * cp.p1 - 3.0f * cp.p2 + cp.p3) * 3.0f * t2
    );

    return {position, tangent};
}

} // namespace wowee::math
