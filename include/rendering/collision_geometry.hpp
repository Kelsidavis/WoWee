#pragma once

/// The two primitives every collision query in this client is built from, and
/// the focus that decides which instances a query looks at.
///
/// The doodad renderer and the building renderer each had their own copy of
/// all three. They agreed, which is the point: a difference here does not
/// crash or log, it moves where the world is solid. A ray test that starts
/// rejecting backfaces makes floors one-sided; a distance test that is off by
/// its own radius drops the instance a character is standing on out of the
/// query, and the character falls through it.
///
/// The focus is the one with the most copies: two setters and four tests, the
/// tests written inline three times in the doodad renderer and once as a
/// method in the building one.

#include <algorithm>

#include <glm/glm.hpp>

namespace wowee::rendering {

/// Squared distance from `p` to the nearest point of the box.
///
/// Zero inside the box, which is what the callers rely on: an instance the
/// query point is inside is never further away than one it is outside.
inline float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin,
                                 const glm::vec3& bmax) {
    const glm::vec3 nearest = glm::clamp(p, bmin, bmax);
    const glm::vec3 d = p - nearest;
    return glm::dot(d, d);
}

/// Moller-Trumbore. Distance along `dir` to the triangle, or negative for a
/// miss.
///
/// Two-sided on purpose. A floor is a triangle whose winding says which way
/// is up, and a character standing on the underside of a bridge or inside a
/// building still needs it to be solid, so a version that culled backfaces
/// would make half the world's surfaces one-way.
inline float rayTriangleIntersect(const glm::vec3& origin, const glm::vec3& dir,
                                  const glm::vec3& v0, const glm::vec3& v1,
                                  const glm::vec3& v2) {
    constexpr float EPSILON = 1e-6f;
    const glm::vec3 e1 = v1 - v0;
    const glm::vec3 e2 = v2 - v0;
    const glm::vec3 h = glm::cross(dir, e2);
    const float a = glm::dot(e1, h);
    if (a > -EPSILON && a < EPSILON) return -1.0f;  // ray parallel to the plane

    const float f = 1.0f / a;
    const glm::vec3 s = origin - v0;
    const float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return -1.0f;

    const glm::vec3 q = glm::cross(s, e1);
    const float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return -1.0f;

    const float t = f * glm::dot(e2, q);
    return t > EPSILON ? t : -1.0f;  // behind the origin counts as a miss
}

/// The sphere a collision query is restricted to, so that a query near the
/// player does not walk every instance in the world.
///
/// Radius zero means no restriction rather than an empty one: that is what
/// both renderers meant by an unset focus, and reading it the other way would
/// make the world non-solid rather than merely slow.
struct CollisionFocus {
    bool enabled = false;
    glm::vec3 position{0.0f};
    float radius = 0.0f;
    float radiusSq = 0.0f;

    void set(const glm::vec3& worldPos, float newRadius) {
        enabled = (newRadius > 0.0f);
        position = worldPos;
        radius = std::max(0.0f, newRadius);
        radiusSq = radius * radius;
    }

    /// Whether an instance with these world bounds is outside the focus and
    /// can be skipped. Always false while unset.
    ///
    /// The box is tested rather than its centre, so a long building whose
    /// origin is far away is still collided while the player stands on one
    /// end of it.
    [[nodiscard]] bool excludes(const glm::vec3& boundsMin, const glm::vec3& boundsMax) const {
        return enabled &&
               pointAABBDistanceSq(position, boundsMin, boundsMax) > radiusSq;
    }
};

}  // namespace wowee::rendering
