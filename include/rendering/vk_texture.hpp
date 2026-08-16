#pragma once

#include <source_location>

#include "rendering/vk_utils.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <string>
#include <cstdint>

namespace wowee {
namespace rendering {

class VkContext;

class VkTexture {
public:
    VkTexture() = default;
    ~VkTexture();

    VkTexture(const VkTexture&) = delete;
    VkTexture& operator=(const VkTexture&) = delete;
    VkTexture(VkTexture&& other) noexcept;
    VkTexture& operator=(VkTexture&& other) noexcept;

    // Upload RGBA8 pixel data to GPU
    bool upload(VkContext& ctx, const uint8_t* pixels, uint32_t width, uint32_t height,
        VkFormat format = VK_FORMAT_R8G8B8A8_UNORM, bool generateMips = true,
        /// Defaulted, so the allocation records the caller rather than this
        /// file. Names the allocation for the shutdown leak dump only.
        std::source_location where = std::source_location::current());

    // Upload with pre-existing mip data (array of mip levels)
    bool uploadMips(VkContext& ctx, const uint8_t* const* mipData, const uint32_t* mipSizes,
        uint32_t mipCount, uint32_t width, uint32_t height,
        VkFormat format = VK_FORMAT_R8G8B8A8_UNORM);

    // Create a depth/stencil texture (no upload)
    bool createDepth(VkContext& ctx, uint32_t width, uint32_t height,
        VkFormat format = VK_FORMAT_D32_SFLOAT);

    // Create sampler with specified filtering
    bool createSampler(VkDevice device,
        VkFilter minFilter = VK_FILTER_LINEAR,
        VkFilter magFilter = VK_FILTER_LINEAR,
        VkSamplerAddressMode addressMode = VK_SAMPLER_ADDRESS_MODE_REPEAT,
        float maxAnisotropy = 16.0f);

    // Overload with separate S/T address modes
    /// `mipLodBias` pushes sampling toward smaller mips. Zero everywhere but
    /// the sky, whose model stacks eighteen additive layers over one dome -
    /// each layer's sampling noise is added to the last, so aliasing that is
    /// invisible on any one model is eighteen times as loud there.
    bool createSampler(VkDevice device,
        VkFilter filter,
        VkSamplerAddressMode addressModeU,
        VkSamplerAddressMode addressModeV,
        float maxAnisotropy = 16.0f,
        float mipLodBias = 0.0f);


    /// Release now. Safe to call more than once, and unnecessary since the
    /// destructor does it -- kept because call sites that free eagerly rather
    /// than at scope exit are still meaningful.
    void destroy(VkDevice device, VmaAllocator allocator);

    [[nodiscard]] VkImage getImage() const { return image_.image; }
    [[nodiscard]] VkImageView getImageView() const { return image_.imageView; }
    [[nodiscard]] VkSampler getSampler() const { return sampler_; }
    [[nodiscard]] uint32_t getWidth() const { return image_.extent.width; }
    [[nodiscard]] uint32_t getHeight() const { return image_.extent.height; }
    [[nodiscard]] VkFormat getFormat() const { return image_.format; }
    [[nodiscard]] uint32_t getMipLevels() const { return mipLevels_; }
    [[nodiscard]] bool isValid() const { return image_.image != VK_NULL_HANDLE && sampler_ != VK_NULL_HANDLE; }

    // Write descriptor info for binding
    [[nodiscard]] VkDescriptorImageInfo descriptorInfo(VkImageLayout layout =
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) const;

private:
    void generateMipmaps(VkContext& ctx, VkFormat format, uint32_t width, uint32_t height);
    // Shared sampler finalization: prefer the global cache, fall back to direct creation
    bool finalizeSampler(VkDevice device, const VkSamplerCreateInfo& samplerInfo);

    AllocatedImage image_{};
    VkSampler sampler_ = VK_NULL_HANDLE;
    uint32_t mipLevels_ = 1;
    bool ownsSampler_ = true; // false when sampler comes from VkContext cache

    /// The device and allocator whatever is held above came from, recorded so
    /// the destructor can release it without being told.
    ///
    /// Before these existed ~VkTexture was empty and every owner had to call
    /// destroy() by hand, which meant a unique_ptr<VkTexture> released
    /// nothing, an unordered_map::emplace that lost a key race silently
    /// dropped a texture with no handle left to free it by, and a second
    /// upload() over the same object overwrote the first. Four separate leaks
    /// came from that, one of them 540 MB.
    VkDevice device_ = VK_NULL_HANDLE;
    VmaAllocator allocator_ = VK_NULL_HANDLE;
};

} // namespace rendering
} // namespace wowee
