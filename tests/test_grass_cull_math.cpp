// The frustum test the grass cull actually performs, exercised on the CPU for
// every compass direction.
//
// Grass has been reported vanishing when the camera faces one way and not
// another, which is a claim about this arithmetic and nothing else: the
// population is uniform around the player and the distance test is a circle.
// So the arithmetic gets run here rather than reasoned about, for a camera
// turned all the way round, with the same plane extraction the shader is fed
// and the same rejection the shader applies.

#include <catch_amalgamated.hpp>

#include <cmath>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "rendering/frustum.hpp"

using wowee::rendering::Frustum;

namespace {

constexpr float kPi = 3.14159265358979323846f;

/// The rejection grass_cull.comp.glsl makes, verbatim: outside if any plane
/// has the sphere entirely behind it.
bool culledByShader(const Frustum& frustum, const glm::vec3& center, float radius) {
    for (int i = 0; i < 6; ++i) {
        const auto& plane = frustum.getPlane(static_cast<Frustum::Side>(i));
        if (glm::dot(plane.normal, center) + plane.distance < -radius) return true;
    }
    return false;
}

/// A camera orbiting the player at `bearing`, looking back at them - which is
/// how this game's camera works, and why turning the view also moves it.
Frustum frustumForBearing(const glm::vec3& player, float bearing) {
    const glm::vec3 offset(std::cos(bearing) * 8.0f, std::sin(bearing) * 8.0f, 4.0f);
    const glm::vec3 eye = player + offset;
    const glm::mat4 view = glm::lookAt(eye, player, glm::vec3(0.0f, 0.0f, 1.0f));
    // Z-up world, Vulkan depth, the aspect and field of view the client uses.
    const glm::mat4 proj = glm::perspective(glm::radians(70.0f), 16.0f / 9.0f, 0.1f, 1000.0f);

    Frustum frustum;
    frustum.extractFromMatrix(proj * view);
    return frustum;
}

} // namespace

TEST_CASE("the cull keeps grass ahead of the camera from every bearing",
          "[grass][cull]") {
    const glm::vec3 player(-9246.0f, 278.0f, 72.0f);

    for (int step = 0; step < 16; ++step) {
        const float bearing = static_cast<float>(step) * 2.0f * kPi / 16.0f;
        const Frustum frustum = frustumForBearing(player, bearing);

        // The camera sits at `bearing` and looks back through the player, so
        // the view runs on past them in the opposite direction.
        const glm::vec2 forward(-std::cos(bearing), -std::sin(bearing));

        for (float distance : {2.0f, 10.0f, 25.0f, 40.0f}) {
            const glm::vec3 root(player.x + forward.x * distance,
                                 player.y + forward.y * distance, player.z);
            const glm::vec3 center = root + glm::vec3(0.0f, 0.0f, 0.2f);
            INFO("bearing step " << step << " at " << distance << " yards");
            REQUIRE_FALSE(culledByShader(frustum, center, 0.3f));
        }
    }
}

TEST_CASE("the cull rejects grass behind the camera from every bearing",
          "[grass][cull]") {
    // The other half of the contract: a test that keeps everything is not a
    // working frustum test, it is a disabled one.
    const glm::vec3 player(-9246.0f, 278.0f, 72.0f);

    for (int step = 0; step < 16; ++step) {
        const float bearing = static_cast<float>(step) * 2.0f * kPi / 16.0f;
        const Frustum frustum = frustumForBearing(player, bearing);

        // Well behind the camera, which sits 8 yards out along the bearing.
        const glm::vec3 behind(player.x + std::cos(bearing) * 40.0f,
                               player.y + std::sin(bearing) * 40.0f, player.z);
        INFO("bearing step " << step);
        REQUIRE(culledByShader(frustum, behind + glm::vec3(0.0f, 0.0f, 0.2f), 0.3f));
    }
}

TEST_CASE("no bearing keeps a different amount of a uniform field",
          "[grass][cull]") {
    // The reported fault, stated as a measurement: an even field of grass
    // around the player, counted through the cull at each bearing. A frustum
    // that is wrong in one world direction shows up here as one bearing
    // keeping a different count from the rest, whatever the reason.
    const glm::vec3 player(-9246.0f, 278.0f, 72.0f);

    std::vector<glm::vec3> field;
    for (int gx = -45; gx <= 45; ++gx) {
        for (int gy = -45; gy <= 45; ++gy) {
            const glm::vec3 p(player.x + static_cast<float>(gx),
                              player.y + static_cast<float>(gy), player.z);
            if (glm::distance(glm::vec2(p), glm::vec2(player)) <= 45.0f) field.push_back(p);
        }
    }
    REQUIRE(field.size() > 5000);

    size_t lowest = field.size();
    size_t highest = 0;
    for (int step = 0; step < 16; ++step) {
        const float bearing = static_cast<float>(step) * 2.0f * kPi / 16.0f;
        const Frustum frustum = frustumForBearing(player, bearing);
        size_t kept = 0;
        for (const auto& p : field) {
            if (!culledByShader(frustum, p + glm::vec3(0.0f, 0.0f, 0.2f), 0.3f)) ++kept;
        }
        INFO("bearing step " << step << " kept " << kept);
        REQUIRE(kept > 0);
        lowest = std::min(lowest, kept);
        highest = std::max(highest, kept);
    }

    // Bearings differ a little because the frustum is a rectangle over a round
    // field, but nothing like the two-and-a-half times reported in the client.
    REQUIRE(static_cast<float>(highest) < static_cast<float>(lowest) * 1.35f);
}
