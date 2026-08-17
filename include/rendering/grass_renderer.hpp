#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "pipeline/grass_population.hpp"
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

    /// Rebuild the graphics pipeline against the current render pass. Called
    /// when MSAA or the swapchain changes, like every other renderer.
    void recreatePipelines();

    /// Record the cull dispatch. Must run outside a render pass, before
    /// `render()` for the same frame.
    ///
    void dispatchCull(VkCommandBuffer cmd, uint32_t frameIndex, const Camera& camera);

    /// Record the indirect draw. Must run inside the render pass.
    void render(VkCommandBuffer cmd, uint32_t frameIndex, VkDescriptorSet perFrameSet);

    /// Replace the live population. Blades carry world positions, so this is
    /// the whole of what the renderer knows about where grass is.
    ///
    /// Stages into a buffer allocated once at its full capacity: the set of
    /// blades changes every time the player leaves the generated window, and
    /// reallocating a device-local buffer on that cadence would mean either
    /// stalling the queue or deferring destruction on every rebuild.
    /// Blades beyond kMaxBlades are dropped, and the caller is told.
    bool setPopulation(const pipeline::GrassBladeSample* blades, size_t count);

    /// Log how many blades the cull kept, once. Diagnostic only.
    void reportCullResult();

    [[nodiscard]] bool isReady() const { return pipeline_ != VK_NULL_HANDLE; }
    [[nodiscard]] uint32_t bladeCount() const { return bladeCount_; }

    /// Capacity of the source buffer, in blades. 12 MB at 32 bytes each.
    static constexpr uint32_t kMaxBlades = 400000;

private:
    bool createSourceBuffer();
    bool createPerFrameBuffers();
    bool createCullPipeline();
    bool createDrawPipeline(VkDescriptorSetLayout perFrameLayout);
    bool buildDrawPipeline();

    VkContext* vkCtx_ = nullptr;
    VkDescriptorSetLayout perFrameLayout_ = VK_NULL_HANDLE;
    uint32_t bladeCount_ = 0;
    bool cullReported_ = false;
    uint32_t framesSincePopulated_ = 0;
    bool drawReported_ = false;


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
