#pragma once

/// The geometry the fog volume is laid out in, apart from anything Vulkan so a
/// test can hold it against the projection the shaders use. See VolumetricFog.

#include <array>
#include <glm/glm.hpp>

namespace wowee::rendering {

/// The rays from the camera through the four corners of the view, each one
/// yard deep along it, for uv (0,0), (1,0), (0,1) and (1,1) - uv being
/// clip.xy / clip.w * 0.5 + 0.5, which is how every shader finds its place in
/// the volume. A perspective projection makes screen position linear in these,
/// so the ray through any uv is their bilinear mix, and the point d yards deep
/// along the view there is the camera plus d times that ray.
///
/// Inverted from the projection rather than rebuilt from a field of view, so
/// the upscaler's jitter and the flipped Y come along with it. The view must be
/// a rotation and a translation, as a look-at is.
inline std::array<glm::vec3, 4> froxelCornerRays(const glm::mat4& view, const glm::mat4& projection) {
    const glm::mat4 invProj = glm::inverse(projection);
    const glm::mat3 invViewRot = glm::transpose(glm::mat3(view));
    const glm::vec2 corners[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}};
    std::array<glm::vec3, 4> rays{};
    for (int i = 0; i < 4; i++) {
        const glm::vec2 ndc = corners[i] * 2.0f - 1.0f;
        const glm::vec4 v = invProj * glm::vec4(ndc, 0.5f, 1.0f);
        glm::vec3 viewRay = glm::vec3(v) / v.w;
        viewRay /= -viewRay.z;  // one yard in front, along -Z
        rays[i] = invViewRot * viewRay;
    }
    return rays;
}

}  // namespace wowee::rendering
