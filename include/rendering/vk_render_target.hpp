#pragma once

#include "rendering/vk_utils.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <cstdint>

namespace wowee {
namespace rendering {

class VkContext;

/**
 * Off-screen render target encapsulating VkRenderPass + VkFramebuffer + color VkImage.
 * Used for minimap compositing, world map compositing, and other off-screen passes.
 * Supports optional MSAA with automatic resolve.
 */
class VkRenderTarget {
public:
    VkRenderTarget() = default;
    ~VkRenderTarget();

    VkRenderTarget(const VkRenderTarget&) = delete;
    VkRenderTarget& operator=(const VkRenderTarget&) = delete;

    /**
     * Create the render target with given dimensions and format.
     * Creates: color image, image view, sampler, render pass, framebuffer.
     * When withDepth is true, also creates a D32_SFLOAT depth attachment.
     * When msaaSamples > 1, creates multisampled images and a resolve attachment.
     */
    bool create(VkContext& ctx, uint32_t width, uint32_t height,
                VkFormat format = VK_FORMAT_R8G8B8A8_UNORM, bool withDepth = false,
                VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT);

    /**
     * Destroy all Vulkan resources.
     */
    void destroy(VkDevice device, VmaAllocator allocator);

    /**
     * Begin the off-screen render pass (clears to given color).
     * Must be called outside any other active render pass.
     */
    void beginPass(VkCommandBuffer cmd,
                   const VkClearColorValue& clear = {{0.0f, 0.0f, 0.0f, 1.0f}});

    /**
     * End the off-screen render pass.
     * After this, the color image is in SHADER_READ_ONLY_OPTIMAL layout.
     */
    void endPass(VkCommandBuffer cmd);

    // Accessors - always return the resolved (single-sample) image for reading
    [[nodiscard]] VkImage getColorImage() const { return resolveImage_.image ? resolveImage_.image : colorImage_.image; }
    [[nodiscard]] VkImageView getColorImageView() const { return resolveImage_.imageView ? resolveImage_.imageView : colorImage_.imageView; }
    [[nodiscard]] VkSampler getSampler() const { return sampler_; }
    [[nodiscard]] VkRenderPass getRenderPass() const { return renderPass_; }
    [[nodiscard]] VkExtent2D getExtent() const { return { colorImage_.extent.width, colorImage_.extent.height }; }
    [[nodiscard]] VkFormat getFormat() const { return colorImage_.format; }
    [[nodiscard]] bool isValid() const { return framebuffer_ != VK_NULL_HANDLE; }
    [[nodiscard]] VkSampleCountFlagBits getSampleCount() const { return msaaSamples_; }

    /**
     * Descriptor info for binding the color attachment as a texture in a shader.
     */
    [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const;

private:
    AllocatedImage colorImage_{};     // MSAA color (or single-sample if no MSAA)
    AllocatedImage resolveImage_{};   // Single-sample resolve target (only when MSAA)
    AllocatedImage depthImage_{};
    bool hasDepth_ = false;
    VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    VkSampler sampler_ = VK_NULL_HANDLE;
    bool ownsSampler_ = true;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    VkFramebuffer framebuffer_ = VK_NULL_HANDLE;
};

} // namespace rendering
} // namespace wowee
