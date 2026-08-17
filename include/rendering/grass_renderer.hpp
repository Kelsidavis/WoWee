#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "rendering/grass_blade.hpp"
#include "rendering/vk_context.hpp"

namespace wowee {

namespace rendering {

class Camera;

/// GPU-driven grass.
///
/// Phase 1 of `docs/plan-grass.md`: the whole path - source buffer, compute
/// cull with atomic compaction, indirect draw - running on a fixed test
/// population at a hardcoded origin. No terrain input, no wind, no curvature.
/// Those arrive in later phases; this one exists so that when they do, the
/// Vulkan plumbing underneath them is already known to work.
///
/// The counter is never read on the CPU. `instanceCount` in the indirect
/// command is both the compaction cursor the compute shader atomically
/// advances and the field the draw consumes, so no readback and no host
/// synchronisation is involved.
class GrassRenderer {
public:
    GrassRenderer() = default;
    ~GrassRenderer();

    GrassRenderer(const GrassRenderer&) = delete;
    GrassRenderer& operator=(const GrassRenderer&) = delete;

    bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);
    void shutdown();

    /// Record the cull dispatch. Must run outside a render pass, before
    /// `render()` for the same frame.
    ///
    /// `playerPos` plants the test field the first time it is non-zero, and is
    /// ignored afterwards. Latched rather than followed: a field that tracked
    /// the player would slide under them, and blade positions have to depend on
    /// the world rather than on the frame (spec §36).
    void dispatchCull(VkCommandBuffer cmd, uint32_t frameIndex, const Camera& camera,
                      const glm::vec3& playerPos);

    /// Record the indirect draw. Must run inside the render pass.
    void render(VkCommandBuffer cmd, uint32_t frameIndex, VkDescriptorSet perFrameSet);

    [[nodiscard]] bool isReady() const { return pipeline_ != VK_NULL_HANDLE; }
    [[nodiscard]] uint32_t bladeCount() const { return bladeCount_; }

    /// Blades the test population is built with. Phase 3 replaces this with a
    /// per-tile generator driven by terrain suitability.
    static constexpr uint32_t kTestBladeCount = 100000;

private:
    bool createSourceBuffer();
    bool createPerFrameBuffers();
    bool createCullPipeline();
    bool createDrawPipeline(VkDescriptorSetLayout perFrameLayout);

    VkContext* vkCtx_ = nullptr;
    uint32_t bladeCount_ = 0;

    // Where the test field was planted, and whether it has been. Phase 3
    // generates real world positions per tile and both of these go.
    glm::vec3 fieldOrigin_{0.0f};
    bool fieldPlanted_ = false;

    // Shared, written once at load.
    VkBuffer sourceBuffer_ = VK_NULL_HANDLE;
    VmaAllocation sourceAlloc_ = VK_NULL_HANDLE;
    VkBuffer indexBuffer_ = VK_NULL_HANDLE;
    VmaAllocation indexAlloc_ = VK_NULL_HANDLE;

    // Per frame in flight: the compute shader writes these while the previous
    // frame's copies are still being drawn from.
    static constexpr uint32_t kFrames = MAX_FRAMES_IN_FLIGHT;
    VkBuffer cullUniform_[kFrames]{};
    VmaAllocation cullUniformAlloc_[kFrames]{};
    void* cullUniformMapped_[kFrames]{};
    VkBuffer visibleBuffer_[kFrames]{};
    VmaAllocation visibleAlloc_[kFrames]{};
    VkBuffer indirectBuffer_[kFrames]{};
    VmaAllocation indirectAlloc_[kFrames]{};

    VkDescriptorPool descPool_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout cullSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSet cullSet_[kFrames]{};
    VkPipelineLayout cullPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline cullPipeline_ = VK_NULL_HANDLE;

    VkDescriptorSetLayout drawSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSet drawSet_[kFrames]{};
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
};

} // namespace rendering
} // namespace wowee
