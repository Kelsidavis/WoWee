// Where the sun is, and in particular where it is not.
//
// SkySystem::getSunPosition mirrored a sun below the horizon back up into the
// sky - `sunDir = dir` instead of `-dir` - and the lens flare is the only
// thing that ever asks it. So the mirror did nothing but invent a sun for the
// flare to draw around, at a position no sun was at, and the flare's own
// height attenuation read the mirrored height as a sun climbing the sky and
// let it through at full strength.
//
// Reported as a lens flare with no sun in sight.
#include <catch_amalgamated.hpp>

#include "rendering/sun_direction.hpp"

#include <glm/gtc/matrix_transform.hpp>

using wowee::rendering::sunDirectionFromLightDir;
using wowee::rendering::sunScreenPosition;

TEST_CASE("the sun is opposite the direction its light travels", "[sky][sun]") {
    // Light going down means a sun overhead.
    const glm::vec3 up = sunDirectionFromLightDir(glm::vec3(0.0f, 0.0f, -1.0f));
    CHECK(up.z == Catch::Approx(1.0f));

    // And the answer is a unit vector whatever length it is handed.
    const glm::vec3 scaled = sunDirectionFromLightDir(glm::vec3(0.0f, 0.0f, -57.0f));
    CHECK(glm::length(scaled) == Catch::Approx(1.0f));
    CHECK(scaled.z == Catch::Approx(1.0f));
}

TEST_CASE("a sun below the horizon stays below it", "[sky][sun]") {
    // Light travelling upward is light from under the ground. The sun is down
    // there with it, and this is the assertion the mirror would fail.
    const glm::vec3 night = sunDirectionFromLightDir(glm::vec3(0.0f, 0.0f, 1.0f));
    CHECK(night.z == Catch::Approx(-1.0f));
    CHECK(night.z < 0.0f);

    // Just under the horizon, which is where the mirror did the most damage:
    // a flare at full strength opposite a sun that had already set.
    const glm::vec3 dusk = sunDirectionFromLightDir(glm::vec3(0.6f, 0.0f, 0.08f));
    CHECK(dusk.z < 0.0f);
}

TEST_CASE("no sun given reads as straight up", "[sky][sun]") {
    // Everything downstream gates on height, and straight up is the value that
    // leaves those gates to decide rather than producing a sideways sun.
    const glm::vec3 none = sunDirectionFromLightDir(glm::vec3(0.0f));
    CHECK(none.z == Catch::Approx(1.0f));
    CHECK(glm::length(none) == Catch::Approx(1.0f));
}

// The sun shafts stream from where this puts the sun, and every full-screen pass
// samples with (0,0) at the top left. A sun that came out mirrored top to bottom
// would pour its rays up out of the ground.
TEST_CASE("the sun lands on screen where it is in the sky", "[sky][sun]") {
    const glm::vec3 eye(-8913.0f, 554.0f, 94.0f);
    const glm::vec3 forward(1.0f, 0.0f, 0.0f);
    const glm::mat4 view = glm::lookAt(eye, eye + forward, glm::vec3(0.0f, 0.0f, 1.0f));
    // As the camera builds it: Y flipped for Vulkan.
    glm::mat4 proj = glm::perspective(glm::radians(60.0f), 16.0f / 9.0f, 0.5f, 30000.0f);
    proj[1][1] *= -1.0f;

    // Dead ahead is the middle of the screen, wherever the camera stands.
    const auto ahead = sunScreenPosition(view, proj, forward);
    REQUIRE(ahead.inFront);
    CHECK(ahead.uv.x == Catch::Approx(0.5f).margin(1e-5f));
    CHECK(ahead.uv.y == Catch::Approx(0.5f).margin(1e-5f));

    // Ahead and a little up is above the middle - smaller v, toward the top.
    const auto raised = sunScreenPosition(view, proj, glm::normalize(glm::vec3(1.0f, 0.0f, 0.2f)));
    REQUIRE(raised.inFront);
    CHECK(raised.uv.y < 0.5f);
    CHECK(raised.uv.x == Catch::Approx(0.5f).margin(1e-5f));

    // Behind the camera it has no place on screen at all.
    CHECK_FALSE(sunScreenPosition(view, proj, -forward).inFront);
}
