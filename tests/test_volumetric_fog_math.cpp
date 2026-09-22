// The fog volume's layout against the projection the shaders read it with.
//
// The inject pass places each cell at camera + ray(uv) * depth, the ray a
// bilinear mix of four corner rays built on the CPU; every world shader finds
// its cell the other way round, by projecting its position and taking
// clip.xy / clip.w for uv and clip.w for depth. If those two disagree - a sign
// on the flipped Y, the upscaler's jitter left out, a corner in the wrong slot
// - the fog is lit for one place and read at another, which shows as shafts
// that slide against the trees casting them. Nothing else would say so.
#include <catch_amalgamated.hpp>
#include "rendering/volumetric_fog_math.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using wowee::rendering::froxelCornerRays;

namespace {

glm::vec3 rayAt(const std::array<glm::vec3, 4>& rays, glm::vec2 uv) {
    const glm::vec3 top = glm::mix(rays[0], rays[1], uv.x);
    const glm::vec3 bottom = glm::mix(rays[2], rays[3], uv.x);
    return glm::mix(top, bottom, uv.y);
}

/// Project, then rebuild the point from where the projection put it, as the
/// shaders and the inject pass respectively do.
void checkRoundTrip(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& camPos,
                    const glm::vec3& point) {
    const glm::vec4 clip = proj * view * glm::vec4(point, 1.0f);
    REQUIRE(clip.w > 0.0f);
    const glm::vec2 uv = glm::vec2(clip) / clip.w * 0.5f + 0.5f;
    const glm::vec3 rebuilt = camPos + rayAt(froxelCornerRays(view, proj), uv) * clip.w;
    INFO("point (" << point.x << ", " << point.y << ", " << point.z << ") came back as ("
                   << rebuilt.x << ", " << rebuilt.y << ", " << rebuilt.z << ")");
    CHECK(glm::length(rebuilt - point) < 1e-4f * clip.w + 1e-3f);
}

}  // namespace

TEST_CASE("a fog cell is lit where the shaders will read it", "[volumetric]") {
    // The camera the renderer builds: Z up, glm::perspective, Y flipped for Vulkan.
    const glm::vec3 camPos(-8913.0f, 554.0f, 94.0f);
    const glm::mat4 view = glm::lookAt(camPos, camPos + glm::vec3(0.6f, 0.7f, -0.2f),
                                       glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 proj = glm::perspective(glm::radians(70.0f), 16.0f / 9.0f, 0.5f, 30000.0f);
    proj[1][1] *= -1.0f;

    const glm::vec3 points[] = {
        camPos + glm::vec3(30.0f, 40.0f, -8.0f),
        camPos + glm::vec3(300.0f, 120.0f, 25.0f),
        camPos + glm::vec3(2.0f, 3.0f, -1.5f),
        camPos + glm::vec3(-40.0f, 380.0f, -60.0f),
    };
    for (const auto& p : points) checkRoundTrip(view, proj, camPos, p);

    SECTION("with the upscaler's jitter in the projection") {
        glm::mat4 jittered = proj;
        jittered[2][0] += 0.0013f;
        jittered[2][1] -= 0.0007f;
        for (const auto& p : points) checkRoundTrip(view, jittered, camPos, p);
    }
}

TEST_CASE("the corner rays are one yard deep and span the screen the right way up", "[volumetric]") {
    const glm::vec3 camPos(0.0f);
    const glm::vec3 forward(1.0f, 0.0f, 0.0f);
    const glm::mat4 view = glm::lookAt(camPos, forward, glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 proj = glm::perspective(glm::radians(60.0f), 1.5f, 0.5f, 30000.0f);
    proj[1][1] *= -1.0f;
    const auto rays = froxelCornerRays(view, proj);

    for (const auto& r : rays) CHECK(glm::dot(r, forward) == Catch::Approx(1.0f));
    // uv (0,0) is the top of the framebuffer once Y is flipped: the upper
    // corners point up, the lower ones down. The volume's rows run the same
    // way as the screen's, which is what the sky's TexCoord lookup assumes.
    CHECK(rays[0].z > 0.0f);
    CHECK(rays[1].z > 0.0f);
    CHECK(rays[2].z < 0.0f);
    CHECK(rays[3].z < 0.0f);
}
