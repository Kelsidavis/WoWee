// include/math/spline.hpp
// Standalone Catmull-Rom spline module with zero external dependencies beyond GLM.
// Immutable after construction - thread-safe for concurrent reads.
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cstdint>

namespace wowee::math {

/// A single time-indexed control point on a spline.
struct SplineKey {
    uint32_t timeMs;
    glm::vec3 position;
};

/// Result of evaluating a spline at a given time - position + tangent.
struct SplineEvalResult {
    glm::vec3 position;
    glm::vec3 tangent;       // Unnormalized derivative
};

/// Immutable spline path. Constructed once, evaluated many times.
/// Thread-safe for concurrent reads after construction.
class CatmullRomSpline {
public:
    /// Construct from time-sorted keys.
    /// If `timeClosed` is true, the path wraps: first and last keys share endpoints.
    /// If false, uses clamped endpoints (no wrapping).
    explicit CatmullRomSpline(std::vector<SplineKey> keys, bool timeClosed = false);

    /// Evaluate position at given path time (clamped to [0, duration]).
    [[nodiscard]] glm::vec3 evaluatePosition(uint32_t pathTimeMs) const;

    /// Evaluate position and tangent at given path time.
    [[nodiscard]] SplineEvalResult evaluate(uint32_t pathTimeMs) const;

    /// Derive orientation quaternion from tangent (Z-up convention).
    [[nodiscard]] static glm::quat orientationFromTangent(const glm::vec3& tangent);

    /// Total duration of the spline in milliseconds.
    [[nodiscard]] uint32_t durationMs() const { return durationMs_; }

    /// Number of control points.
    [[nodiscard]] size_t keyCount() const { return keys_.size(); }

    /// Direct access to keys (for path inference, etc.).
    [[nodiscard]] const std::vector<SplineKey>& keys() const { return keys_; }

    /// Whether this spline has meaningful XY movement (not just Z-only like elevators).
    [[nodiscard]] bool hasXYMovement(float minRange = 1.0f) const;

    /// Find nearest key index to a world position (for phase estimation).
    [[nodiscard]] size_t findNearestKey(const glm::vec3& position) const;

    /// Whether the spline is time-closed (wrapping).
    [[nodiscard]] bool isTimeClosed() const { return timeClosed_; }

private:
    /// Binary search for segment containing pathTimeMs. O(log n).
    [[nodiscard]] size_t findSegment(uint32_t pathTimeMs) const;

    /// Get 4 control points {p0,p1,p2,p3} for the segment starting at `segIdx`.
    struct ControlPoints { glm::vec3 p0, p1, p2, p3; };
    [[nodiscard]] ControlPoints getControlPoints(size_t segIdx) const;

    /// Evaluate position and tangent for a segment at parameter t in [0,1].
    [[nodiscard]] SplineEvalResult evalSegment(
        const ControlPoints& cp, float t) const;

    std::vector<SplineKey> keys_;
    bool timeClosed_;
    uint32_t durationMs_ = 0;
};

} // namespace wowee::math
