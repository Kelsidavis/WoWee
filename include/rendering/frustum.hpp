#pragma once

#include <glm/glm.hpp>
#include <array>

namespace wowee {
namespace rendering {

/**
 * Frustum plane
 */
struct Plane {
    glm::vec3 normal;
    float distance;

    Plane() : normal(0.0f), distance(0.0f) {}
    Plane(const glm::vec3& n, float d) : normal(n), distance(d) {}

    /**
     * Calculate signed distance from point to plane
     * Positive = in front, Negative = behind
     */
    [[nodiscard]] float distanceToPoint(const glm::vec3& point) const {
        return glm::dot(normal, point) + distance;
    }
};

/**
 * View frustum for culling
 *
 * Six planes: left, right, bottom, top, near, far
 */
class Frustum {
public:
    enum Side {
        PLANE_LEFT = 0,
        PLANE_RIGHT,
        PLANE_BOTTOM,
        PLANE_TOP,
        PLANE_NEAR,
        PLANE_FAR
    };

    Frustum() = default;

    /**
     * Extract frustum planes from view-projection matrix
     * @param viewProjection Combined view * projection matrix
     */
    void extractFromMatrix(const glm::mat4& viewProjection);

    /**
     * Test if point is inside frustum
     */
    [[nodiscard]] bool containsPoint(const glm::vec3& point) const;

    /**
     * Test if sphere is inside or intersecting frustum
     * @param center Sphere center
     * @param radius Sphere radius
     * @return true if sphere is visible (fully or partially inside)
     */
    [[nodiscard]] bool intersectsSphere(const glm::vec3& center, float radius) const;

    /**
     * Test if axis-aligned bounding box intersects frustum
     * @param min Box minimum corner
     * @param max Box maximum corner
     * @return true if box is visible (fully or partially inside)
     */
    [[nodiscard]] bool intersectsAABB(const glm::vec3& min, const glm::vec3& max) const;

    /**
     * Get frustum plane
     */
    [[nodiscard]] const Plane& getPlane(Side side) const { return planes[side]; }

private:
    std::array<Plane, 6> planes;

    /**
     * Normalize plane (ensure unit length normal)
     */
    void normalizePlane(Plane& plane);
};

} // namespace rendering
} // namespace wowee
