#pragma once

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace wowee {
namespace rendering {

class VkContext;

/**
 * Hierarchical-Z (HiZ) depth pyramid for GPU occlusion culling (Phase 6.3 Option B).
 *
 * Builds a min-depth mip chain from the previous frame's depth buffer each frame.
 * The M2 cull compute shader samples this pyramid to reject objects hidden behind
 * geometry, complementing the existing frustum culling.
 *
 * Lifecycle:
 *   initialize()   - create pyramid image, sampler, compute pipeline, descriptors
 *   buildPyramid() - dispatch compute to reduce depth → mip chain (once per frame)
 *   shutdown()      - destroy all Vulkan resources
 *
 * The pyramid is double-buffered (per frame-in-flight) so builds and reads
 * never race across concurrent GPU submissions.
 */
class HiZSystem {
public:
    HiZSystem() = default;
    ~HiZSystem();

    HiZSystem(const HiZSystem&) = delete;
    HiZSystem& operator=(const HiZSystem&) = delete;

    /**
     * Create all Vulkan resources.
     * @param ctx       Vulkan context (device, allocator, etc.)
     * @param width     Full-resolution depth buffer width
     * @param height    Full-resolution depth buffer height
     * @return true on success
     */
    [[nodiscard]] bool initialize(VkContext* ctx, uint32_t width, uint32_t height);

    /**
     * Release all Vulkan resources.
     */
    void shutdown();

    /**
     * Rebuild the pyramid after a swapchain resize.
     * Safe to call repeatedly - destroys old resources first.
     */
    [[nodiscard]] bool resize(uint32_t width, uint32_t height);

    /**
     * Dispatch compute shader to build the HiZ pyramid from the current depth buffer.
     * Must be called AFTER the main scene pass has finished writing to the depth buffer.
     *
     * @param cmd        Active command buffer (in recording state)
     * @param frameIndex Current frame-in-flight index (0 or 1)
     * @param depthImage Source depth image (VK_FORMAT_D32_SFLOAT)
     */
    void buildPyramid(VkCommandBuffer cmd, uint32_t frameIndex, VkImage depthImage);

    /**
     * @return Descriptor set layout for the HiZ pyramid sampler (set 1 for m2_cull_hiz).
     */
    [[nodiscard]] VkDescriptorSetLayout getDescriptorSetLayout() const { return hizSetLayout_; }

    /**
     * @return Descriptor set for the given frame (sampler2D of the HiZ pyramid).
     *         Bind as set 1 in the M2 HiZ cull pipeline.
     */
    [[nodiscard]] VkDescriptorSet getDescriptorSet(uint32_t frameIndex) const { return hizDescSet_[frameIndex]; }

    /**
     * @return true if HiZ system is initialized and ready.
     */
    [[nodiscard]] bool isReady() const { return ready_; }

    /**
     * @return Number of mip levels in the pyramid.
     */
    [[nodiscard]] uint32_t getMipLevels() const { return mipLevels_; }

    /**
     * @return Pyramid base resolution (mip 0).
     */
    [[nodiscard]] uint32_t getPyramidWidth() const { return pyramidWidth_; }
    [[nodiscard]] uint32_t getPyramidHeight() const { return pyramidHeight_; }

private:
    bool createPyramidImage();
    void destroyPyramidImage();
    bool createComputePipeline();
    void destroyComputePipeline();
    bool createDescriptors();
    void destroyDescriptors();

    VkContext* ctx_ = nullptr;
    bool ready_ = false;

    // Pyramid dimensions (mip 0 = half of full-res depth)
    uint32_t fullWidth_ = 0;
    uint32_t fullHeight_ = 0;
    uint32_t pyramidWidth_ = 0;
    uint32_t pyramidHeight_ = 0;
    uint32_t mipLevels_ = 0;

    static constexpr uint32_t MAX_FRAMES = 2;

    // Per-frame HiZ pyramid images (R32_SFLOAT, full mip chain)
    VkImage pyramidImage_[MAX_FRAMES] = {};
    VmaAllocation pyramidAlloc_[MAX_FRAMES] = {};
    VkImageView pyramidViewAll_[MAX_FRAMES] = {};          // View of all mip levels (for sampling)
    std::vector<VkImageView> pyramidMipViews_[MAX_FRAMES]; // Per-mip views (for storage image writes)

    // Depth input - image view for sampling the depth buffer as a texture
    VkImageView depthSamplerView_[MAX_FRAMES] = {};

    // Sampler for depth reads (nearest, clamp-to-edge)
    VkSampler depthSampler_ = VK_NULL_HANDLE;

    // Compute pipeline for building the pyramid
    VkPipeline buildPipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout buildPipelineLayout_ = VK_NULL_HANDLE;

    // Descriptor set layout for build pipeline (set 0: src sampler + dst storage image)
    VkDescriptorSetLayout buildSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool buildDescPool_ = VK_NULL_HANDLE;
    // Per-frame, per-mip descriptor sets for pyramid build
    std::vector<VkDescriptorSet> buildDescSets_[MAX_FRAMES];

    // HiZ sampling descriptor: exposed to M2 cull shader (set 1: combined image sampler)
    VkDescriptorSetLayout hizSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool hizDescPool_ = VK_NULL_HANDLE;
    VkDescriptorSet hizDescSet_[MAX_FRAMES] = {};

    // Push constant for build shader
    struct HiZBuildPushConstants {
        int32_t dstWidth;
        int32_t dstHeight;
        int32_t mipLevel;
    };
};

} // namespace rendering
} // namespace wowee
