#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <vulkan/vulkan_core.h>

namespace wowee::rendering {

// The main camera flips projection Y and uses a positive-height viewport. An
// M2 triangle wound toward its authored normal therefore has positive Vulkan
// framebuffer area when viewed from that side. Used only for one-sided
// foliage; the other M2 pipelines keep their existing state.
inline constexpr VkFrontFace kM2FoliageFrontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

// A reflected instance reverses the winding. Since an instanced draw shares
// one pipeline, it cannot cull a mixture of reflected and ordinary instances
// correctly with a single front-face setting. Keep such groups two-sided.
inline bool m2FoliageTransformPreservesWinding(const glm::mat4& model) {
    const float determinant = glm::determinant(glm::mat3(model));
    return std::isfinite(determinant) && determinant > 0.0f;
}

}  // namespace wowee::rendering
