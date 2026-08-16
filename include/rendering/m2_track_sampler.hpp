#pragma once

#include "pipeline/m2_loader.hpp"

#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace wowee::rendering::m2_track {

struct SampleTime {
    int sequenceIndex = -1;
    float timeMs = 0.0f;
};

inline SampleTime resolveTime(const pipeline::M2AnimationTrack& track,
                              int animationSequenceIndex, float animationTimeMs,
                              float globalTimeMs,
                              const std::vector<uint32_t>& globalSequenceDurations) {
    if (track.globalSequence >= 0 &&
        static_cast<size_t>(track.globalSequence) < globalSequenceDurations.size()) {
        const float duration =
            static_cast<float>(globalSequenceDurations[track.globalSequence]);
        float time = duration > 0.0f ? std::fmod(globalTimeMs, duration) : 0.0f;
        if (time < 0.0f) time += duration;
        return {.sequenceIndex = 0, .timeMs = time};
    }
    return {.sequenceIndex = animationSequenceIndex, .timeMs = animationTimeMs};
}

inline size_t lowerKeyIndex(const std::vector<uint32_t>& timestamps,
                            size_t keyCount, float timeMs) {
    keyCount = std::min(keyCount, timestamps.size());
    if (keyCount <= 1 || timeMs <= static_cast<float>(timestamps[0])) return 0;
    const auto end = timestamps.begin() + static_cast<std::ptrdiff_t>(keyCount);
    const auto upper = std::upper_bound(
        timestamps.begin(), end, timeMs,
        [](float time, uint32_t timestamp) {
            return time < static_cast<float>(timestamp);
        });
    if (upper == end) return keyCount - 1;
    return static_cast<size_t>(upper - timestamps.begin() - 1);
}

inline float interpolationFraction(const pipeline::M2AnimationTrack& track,
                                   const std::vector<uint32_t>& timestamps,
                                   size_t lower, size_t keyCount, float timeMs) {
    // Type 0 is discrete. The loader currently stores values but not the tangent
    // pairs required by Hermite/Bezier tracks, so types 1-3 use the stable linear
    // fallback that the renderers historically used.
    if (track.interpolationType == 0 || lower + 1 >= keyCount) return 0.0f;
    const float t0 = static_cast<float>(timestamps[lower]);
    const float t1 = static_cast<float>(timestamps[lower + 1]);
    return t1 > t0 ? glm::clamp((timeMs - t0) / (t1 - t0), 0.0f, 1.0f) : 0.0f;
}

inline float sampleFloat(const pipeline::M2AnimationTrack& track,
                         int animationSequenceIndex, float animationTimeMs,
                         float globalTimeMs,
                         const std::vector<uint32_t>& globalSequenceDurations,
                         float defaultValue) {
    const auto sampleTime = resolveTime(track, animationSequenceIndex,
                                        animationTimeMs, globalTimeMs,
                                        globalSequenceDurations);
    if (sampleTime.sequenceIndex < 0 ||
        static_cast<size_t>(sampleTime.sequenceIndex) >= track.sequences.size()) {
        return defaultValue;
    }
    const auto& keys = track.sequences[static_cast<size_t>(sampleTime.sequenceIndex)];
    const size_t count = std::min(keys.timestamps.size(), keys.floatValues.size());
    if (count == 0) return defaultValue;
    const size_t lower = lowerKeyIndex(keys.timestamps, count, sampleTime.timeMs);
    const float fraction = interpolationFraction(track, keys.timestamps, lower,
                                                 count, sampleTime.timeMs);
    return lower + 1 < count
        ? glm::mix(keys.floatValues[lower], keys.floatValues[lower + 1], fraction)
        : keys.floatValues[lower];
}

inline glm::vec3 sampleVec3(const pipeline::M2AnimationTrack& track,
                            int animationSequenceIndex, float animationTimeMs,
                            float globalTimeMs,
                            const std::vector<uint32_t>& globalSequenceDurations,
                            const glm::vec3& defaultValue) {
    const auto sampleTime = resolveTime(track, animationSequenceIndex,
                                        animationTimeMs, globalTimeMs,
                                        globalSequenceDurations);
    if (sampleTime.sequenceIndex < 0 ||
        static_cast<size_t>(sampleTime.sequenceIndex) >= track.sequences.size()) {
        return defaultValue;
    }
    const auto& keys = track.sequences[static_cast<size_t>(sampleTime.sequenceIndex)];
    const size_t count = std::min(keys.timestamps.size(), keys.vec3Values.size());
    if (count == 0) return defaultValue;
    const auto safe = [&](const glm::vec3& value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z)
            ? value : defaultValue;
    };
    const size_t lower = lowerKeyIndex(keys.timestamps, count, sampleTime.timeMs);
    const float fraction = interpolationFraction(track, keys.timestamps, lower,
                                                 count, sampleTime.timeMs);
    return lower + 1 < count
        ? safe(glm::mix(safe(keys.vec3Values[lower]), safe(keys.vec3Values[lower + 1]), fraction))
        : safe(keys.vec3Values[lower]);
}

inline glm::quat sampleQuat(const pipeline::M2AnimationTrack& track,
                            int animationSequenceIndex, float animationTimeMs,
                            float globalTimeMs,
                            const std::vector<uint32_t>& globalSequenceDurations) {
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    const auto sampleTime = resolveTime(track, animationSequenceIndex,
                                        animationTimeMs, globalTimeMs,
                                        globalSequenceDurations);
    if (sampleTime.sequenceIndex < 0 ||
        static_cast<size_t>(sampleTime.sequenceIndex) >= track.sequences.size()) {
        return identity;
    }
    const auto& keys = track.sequences[static_cast<size_t>(sampleTime.sequenceIndex)];
    const size_t count = std::min(keys.timestamps.size(), keys.quatValues.size());
    if (count == 0) return identity;
    const auto safe = [&](const glm::quat& value) {
        const float lengthSquared = glm::dot(value, value);
        return std::isfinite(lengthSquared) && lengthSquared >= 0.000001f
            ? glm::normalize(value) : identity;
    };
    const size_t lower = lowerKeyIndex(keys.timestamps, count, sampleTime.timeMs);
    const float fraction = interpolationFraction(track, keys.timestamps, lower,
                                                 count, sampleTime.timeMs);
    return lower + 1 < count
        ? glm::normalize(glm::slerp(safe(keys.quatValues[lower]),
                                   safe(keys.quatValues[lower + 1]), fraction))
        : safe(keys.quatValues[lower]);
}

} // namespace wowee::rendering::m2_track
