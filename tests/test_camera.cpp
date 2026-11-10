// Tests for rendering::Camera setter guards and degenerate-pose math.
#include <catch_amalgamated.hpp>
#include "rendering/camera.hpp"
#include "rendering/m2_foliage_culling.hpp"
#include "rendering/placement_transform.hpp"
#include <array>
#include <cmath>
#include <limits>

using namespace wowee::rendering;

TEST_CASE("Camera::setPosition rejects NaN/inf", "[camera]") {
    Camera cam;
    glm::vec3 originalPos = cam.getPosition();

    cam.setPosition(glm::vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0));
    REQUIRE(cam.getPosition() == originalPos);

    cam.setPosition(glm::vec3(std::numeric_limits<float>::infinity(), 0, 0));
    REQUIRE(cam.getPosition() == originalPos);

    cam.setPosition(glm::vec3(10, 20, 30));
    REQUIRE(cam.getPosition() == glm::vec3(10, 20, 30));
}

TEST_CASE("Camera::setRotation rejects NaN", "[camera]") {
    Camera cam;
    cam.setRotation(45.0f, 30.0f);
    cam.setRotation(std::numeric_limits<float>::quiet_NaN(), 60.0f);
    // Yaw stays at 45, pitch stays at 30 (set was rejected wholesale)
    glm::vec3 fwdAfterBad = cam.getForward();
    cam.setRotation(45.0f, 30.0f);
    REQUIRE(cam.getForward() == fwdAfterBad);
}

TEST_CASE("Camera::setFov rejects NaN/zero/180+", "[camera]") {
    Camera cam;
    float originalFov = cam.getFovDegrees();

    cam.setFov(std::numeric_limits<float>::quiet_NaN());
    REQUIRE(cam.getFovDegrees() == originalFov);

    cam.setFov(0.0f);
    REQUIRE(cam.getFovDegrees() == originalFov);

    cam.setFov(-30.0f);
    REQUIRE(cam.getFovDegrees() == originalFov);

    cam.setFov(180.0f);
    REQUIRE(cam.getFovDegrees() == originalFov);

    cam.setFov(90.0f);
    REQUIRE(cam.getFovDegrees() == 90.0f);
}

TEST_CASE("Camera::setAspectRatio rejects non-positive", "[camera]") {
    Camera cam;
    float originalAspect = cam.getAspectRatio();
    cam.setAspectRatio(0.0f);
    REQUIRE(cam.getAspectRatio() == originalAspect);
    cam.setAspectRatio(-1.0f);
    REQUIRE(cam.getAspectRatio() == originalAspect);
    cam.setAspectRatio(std::numeric_limits<float>::quiet_NaN());
    REQUIRE(cam.getAspectRatio() == originalAspect);
    cam.setAspectRatio(2.0f);
    REQUIRE(cam.getAspectRatio() == 2.0f);
}

TEST_CASE("Camera::getRight/getUp return finite at +/-89 pitch", "[camera]") {
    Camera cam;
    cam.setRotation(0.0f, 89.0f);
    glm::vec3 r = cam.getRight();
    glm::vec3 u = cam.getUp();
    REQUIRE(std::isfinite(r.x));
    REQUIRE(std::isfinite(r.y));
    REQUIRE(std::isfinite(r.z));
    REQUIRE(std::isfinite(u.x));
    REQUIRE(std::isfinite(u.y));
    REQUIRE(std::isfinite(u.z));
}

TEST_CASE("Camera::getRight/getUp degrade safely at +/-90 pitch", "[camera]") {
    Camera cam;
    // Force the degenerate pose: forward exactly = world up
    cam.setRotation(0.0f, 90.0f);
    glm::vec3 r = cam.getRight();
    glm::vec3 u = cam.getUp();
    REQUIRE(std::isfinite(r.x));
    REQUIRE(std::isfinite(r.y));
    REQUIRE(std::isfinite(r.z));
    REQUIRE(std::isfinite(u.x));
    REQUIRE(std::isfinite(u.y));
    REQUIRE(std::isfinite(u.z));
    // Right should be the +X fallback (since cross would be zero).
    REQUIRE(r == glm::vec3(1.0f, 0.0f, 0.0f));
}

TEST_CASE("one-sided M2 foliage keeps its authored front through the camera", "[camera][m2]") {
    // A foliage triangle from StormPeaks_TreeB00.skin, index offsets 3..5.
    // Its vertex normals point to the same side as cross(v1-v0, v2-v0).
    // The old diagnostic used CLOCKWISE, which culled this side after the
    // camera's projection Y flip and left only the back of the snow tree.
    const std::array<glm::vec3, 3> local = {{
        {0.92906f, -2.31908f, 3.71287f},
        {1.52553f, -2.41602f, 1.92326f},
        {1.85329f, -0.93441f, 2.71328f},
    }};
    // Triangle 433 is almost coincident but faces the other way and uses
    // different UVs. Drawing both sides can make them compete for depth.
    const std::array<glm::vec3, 3> otherSide = {{
        {1.52439f, -2.41536f, 1.91937f},
        {0.92792f, -2.31842f, 3.70898f},
        {1.85215f, -0.93375f, 2.70939f},
    }};
    const glm::vec3 authoredNormal(0.47799f, -0.61636f, 0.62580f);
    const glm::vec3 localFace = glm::cross(local[1] - local[0], local[2] - local[0]);
    REQUIRE(glm::dot(localFace, authoredNormal) > 0.0f);
    REQUIRE(glm::length(local[0] - otherSide[1]) < 0.005f);
    REQUIRE(glm::length(local[1] - otherSide[0]) < 0.005f);
    REQUIRE(glm::length(local[2] - otherSide[2]) < 0.005f);
    REQUIRE(glm::dot(localFace, glm::cross(otherSide[1] - otherSide[0],
                                           otherSide[2] - otherSide[0])) < 0.0f);

    // ADT placements translate, rotate and positively scale the original mesh.
    // Look toward its front from each placement, using the actual Camera path.
    for (float angle : {0.0f, 0.8f, 3.1f}) {
        const glm::mat4 model = placementModelMatrix({100.0f, 200.0f, 10.0f},
                                                     {0.0f, 0.0f, angle}, 1.3f);
        std::array<glm::vec3, 3> world, worldOtherSide;
        for (size_t i = 0; i < world.size(); ++i) {
            world[i] = glm::vec3(model * glm::vec4(local[i], 1.0f));
            worldOtherSide[i] = glm::vec3(model * glm::vec4(otherSide[i], 1.0f));
        }
        const glm::vec3 center = (world[0] + world[1] + world[2]) / 3.0f;
        const glm::vec3 face = glm::normalize(glm::cross(world[1] - world[0],
                                                         world[2] - world[0]));
        for (float side : {1.0f, -1.0f}) {
            const glm::vec3 forward = -face * side;
            Camera camera;
            camera.setPosition(center + face * (12.0f * side));
            camera.setRotation(glm::degrees(std::atan2(forward.y, forward.x)),
                               glm::degrees(std::asin(forward.z)));

            auto projectedArea = [&camera](const std::array<glm::vec3, 3>& triangle) {
                std::array<glm::vec2, 3> screen;
                for (size_t i = 0; i < triangle.size(); ++i) {
                    const glm::vec4 clip = camera.getViewProjectionMatrix() * glm::vec4(triangle[i], 1.0f);
                    REQUIRE(clip.w > 0.0f);
                    screen[i] = glm::vec2(clip) / clip.w;
                }
                const glm::vec2 a = screen[1] - screen[0];
                const glm::vec2 b = screen[2] - screen[0];
                // Vulkan framebuffer area is -1/2 times the usual signed
                // shoelace area. A positive-height viewport preserves the sign.
                return -0.5f * (a.x * b.y - a.y * b.x);
            };
            REQUIRE(projectedArea(world) * side > 0.0f);
            REQUIRE(projectedArea(worldOtherSide) * side < 0.0f);
            REQUIRE(kM2FoliageFrontFace == VK_FRONT_FACE_COUNTER_CLOCKWISE);
        }
    }
}

TEST_CASE("foliage culling declines reflected and degenerate transforms", "[camera][m2]") {
    const glm::mat4 identity(1.0f);
    CHECK(m2FoliageTransformPreservesWinding(identity));
    CHECK(m2FoliageTransformPreservesWinding(glm::scale(identity, glm::vec3(1.0f, 2.0f, 3.0f))));
    CHECK(m2FoliageTransformPreservesWinding(glm::scale(identity, glm::vec3(-1.0f, -2.0f, 3.0f))));
    CHECK_FALSE(m2FoliageTransformPreservesWinding(glm::scale(identity, glm::vec3(-1.0f, 2.0f, 3.0f))));
    CHECK_FALSE(m2FoliageTransformPreservesWinding(glm::scale(identity, glm::vec3(0.0f, 2.0f, 3.0f))));
    glm::mat4 invalid = identity;
    invalid[0][0] = std::numeric_limits<float>::quiet_NaN();
    CHECK_FALSE(m2FoliageTransformPreservesWinding(invalid));
}
