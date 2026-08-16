#include "rendering/vk_texture.hpp"
#include "rendering/vk_context.hpp"
#include "core/logger.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace wowee {
namespace rendering {

VkTexture::~VkTexture() {
    destroy(device_, allocator_);
}

VkTexture::VkTexture(VkTexture&& other) noexcept
    : image_(other.image_), sampler_(other.sampler_), mipLevels_(other.mipLevels_),
      ownsSampler_(other.ownsSampler_), device_(other.device_),
      allocator_(other.allocator_) {
    other.image_ = {};
    other.sampler_ = VK_NULL_HANDLE;
    // Source no longer owns the sampler - ownership transferred to this instance
    other.ownsSampler_ = false;
    // ...nor the device, which is what stops its destructor freeing ours.
    other.device_ = VK_NULL_HANDLE;
    other.allocator_ = VK_NULL_HANDLE;
}

VkTexture& VkTexture::operator=(VkTexture&& other) noexcept {
    if (this != &other) {
        // Whatever this already held is otherwise overwritten and lost.
        destroy(device_, allocator_);
        image_ = other.image_;
        sampler_ = other.sampler_;
        mipLevels_ = other.mipLevels_;
        ownsSampler_ = other.ownsSampler_;
        device_ = other.device_;
        allocator_ = other.allocator_;
        other.image_ = {};
        other.sampler_ = VK_NULL_HANDLE;
        other.ownsSampler_ = false;
        other.device_ = VK_NULL_HANDLE;
        other.allocator_ = VK_NULL_HANDLE;
    }
    return *this;
}

bool VkTexture::upload(VkContext& ctx, const uint8_t* pixels, uint32_t width, uint32_t height,
    VkFormat format, bool generateMips, std::source_location where) {
    if (!pixels || width == 0 || height == 0) return false;

    mipLevels_ = generateMips
        ? static_cast<uint32_t>(std::floor(std::log2(std::max(width, height)))) + 1
        : 1;

    // Determine bytes per pixel from format
    uint32_t bpp = 4; // default RGBA8
    if (format == VK_FORMAT_R8_UNORM) bpp = 1;
    else if (format == VK_FORMAT_R8G8_UNORM) bpp = 2;
    else if (format == VK_FORMAT_R8G8B8_UNORM) bpp = 3;

    VkDeviceSize imageSize = static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * bpp;

    // Create staging buffer
    AllocatedBuffer staging = createBuffer(ctx.getAllocator(), imageSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);

    void* mapped;
    vmaMapMemory(ctx.getAllocator(), staging.allocation, &mapped);
    std::memcpy(mapped, pixels, imageSize);
    vmaUnmapMemory(ctx.getAllocator(), staging.allocation);

    // Create image with transfer dst + src (src for mipmap generation) + sampled
    VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    if (generateMips) {
        usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    }
    // Release anything this object already held: a second upload over the
    // same texture used to abandon the first, which is how three quest
    // marker textures survived every re-initialisation.
    destroy(device_, allocator_);
    device_ = ctx.getDevice();
    allocator_ = ctx.getAllocator();
    image_ = createImage(ctx.getDevice(), ctx.getAllocator(), width, height,
        format, usage, VK_SAMPLE_COUNT_1_BIT, mipLevels_, where);

    if (!image_.image) {
        destroyBuffer(ctx.getAllocator(), staging);
        return false;
    }

    ctx.immediateSubmit([&](VkCommandBuffer cmd) {
        // Transition to transfer dst
        transitionImageLayout(cmd, image_.image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

        // Copy staging buffer to image (mip 0)
        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.layerCount = 1;
        region.imageExtent = {width, height, 1};

        vkCmdCopyBufferToImage(cmd, staging.buffer, image_.image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        if (!generateMips) {
            // Transition to shader read
            transitionImageLayout(cmd, image_.image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        }
    });

    if (generateMips) {
        generateMipmaps(ctx, format, width, height);
    }

    if (ctx.isInUploadBatch()) {
        ctx.deferStagingCleanup(staging);
    } else {
        destroyBuffer(ctx.getAllocator(), staging);
    }
    return true;
}

bool VkTexture::uploadMips(VkContext& ctx, const uint8_t* const* mipData,
    const uint32_t* mipSizes, uint32_t mipCount, uint32_t width, uint32_t height, VkFormat format)
{
    if (!mipData || mipCount == 0) return false;

    mipLevels_ = mipCount;

    // Calculate total staging size
    VkDeviceSize totalSize = 0;
    for (uint32_t i = 0; i < mipCount; i++) {
        totalSize += mipSizes[i];
    }

    AllocatedBuffer staging = createBuffer(ctx.getAllocator(), totalSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);

    void* mapped;
    vmaMapMemory(ctx.getAllocator(), staging.allocation, &mapped);
    VkDeviceSize offset = 0;
    for (uint32_t i = 0; i < mipCount; i++) {
        std::memcpy(static_cast<uint8_t*>(mapped) + offset, mipData[i], mipSizes[i]);
        offset += mipSizes[i];
    }
    vmaUnmapMemory(ctx.getAllocator(), staging.allocation);

    // Release anything this object already held: a second upload over the
    // same texture used to abandon the first, which is how three quest
    // marker textures survived every re-initialisation.
    destroy(device_, allocator_);
    device_ = ctx.getDevice();
    allocator_ = ctx.getAllocator();
    image_ = createImage(ctx.getDevice(), ctx.getAllocator(), width, height,
        format, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
        VK_SAMPLE_COUNT_1_BIT, mipLevels_);

    if (!image_.image) {
        destroyBuffer(ctx.getAllocator(), staging);
        return false;
    }

    ctx.immediateSubmit([&](VkCommandBuffer cmd) {
        transitionImageLayout(cmd, image_.image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

        VkDeviceSize bufOffset = 0;
        uint32_t mipW = width, mipH = height;
        for (uint32_t i = 0; i < mipCount; i++) {
            VkBufferImageCopy region{};
            region.bufferOffset = bufOffset;
            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = i;
            region.imageSubresource.layerCount = 1;
            region.imageExtent = {mipW, mipH, 1};

            vkCmdCopyBufferToImage(cmd, staging.buffer, image_.image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

            bufOffset += mipSizes[i];
            mipW = std::max(1u, mipW / 2);
            mipH = std::max(1u, mipH / 2);
        }

        transitionImageLayout(cmd, image_.image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
    });

    if (ctx.isInUploadBatch()) {
        ctx.deferStagingCleanup(staging);
    } else {
        destroyBuffer(ctx.getAllocator(), staging);
    }
    return true;
}

bool VkTexture::createDepth(VkContext& ctx, uint32_t width, uint32_t height, VkFormat format) {
    mipLevels_ = 1;

    // Release anything this object already held: a second upload over the
    // same texture used to abandon the first, which is how three quest
    // marker textures survived every re-initialisation.
    destroy(device_, allocator_);
    device_ = ctx.getDevice();
    allocator_ = ctx.getAllocator();
    image_ = createImage(ctx.getDevice(), ctx.getAllocator(), width, height,
        format, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

    if (!image_.image) return false;

    ctx.immediateSubmit([&](VkCommandBuffer cmd) {
        transitionImageLayout(cmd, image_.image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT);
    });

    return true;
}

// Shared sampler finalization: try the global cache first (avoids duplicate Vulkan
// sampler objects), fall back to direct creation if no VkContext is available.
bool VkTexture::finalizeSampler(VkDevice device, const VkSamplerCreateInfo& samplerInfo) {
    // A texture may be given a sampler before an image; record the device so
    // the destructor can still free it.
    if (device_ == VK_NULL_HANDLE) device_ = device;
    auto* ctx = VkContext::globalInstance();
    if (ctx) {
        sampler_ = ctx->getOrCreateSampler(samplerInfo);
        ownsSampler_ = false;
        return sampler_ != VK_NULL_HANDLE;
    }
    if (vkCreateSampler(device, &samplerInfo, nullptr, &sampler_) != VK_SUCCESS) {
        LOG_ERROR("Failed to create texture sampler");
        return false;
    }
    ownsSampler_ = true;
    return true;
}

bool VkTexture::createSampler(VkDevice device,
    VkFilter minFilter, VkFilter magFilter,
    VkSamplerAddressMode addressMode, float maxAnisotropy)
{
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = minFilter;
    samplerInfo.magFilter = magFilter;
    samplerInfo.addressModeU = addressMode;
    samplerInfo.addressModeV = addressMode;
    samplerInfo.addressModeW = addressMode;
    samplerInfo.anisotropyEnable = maxAnisotropy > 1.0f ? VK_TRUE : VK_FALSE;
    samplerInfo.maxAnisotropy = maxAnisotropy;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.mipmapMode = (minFilter == VK_FILTER_LINEAR)
        ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = static_cast<float>(mipLevels_ > 0 ? mipLevels_ - 1 : 0);
    return finalizeSampler(device, samplerInfo);
}

bool VkTexture::createSampler(VkDevice device,
    VkFilter filter,
    VkSamplerAddressMode addressModeU,
    VkSamplerAddressMode addressModeV,
    float maxAnisotropy,
    float mipLodBias)
{
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = filter;
    samplerInfo.magFilter = filter;
    samplerInfo.addressModeU = addressModeU;
    samplerInfo.addressModeV = addressModeV;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.anisotropyEnable = maxAnisotropy > 1.0f ? VK_TRUE : VK_FALSE;
    samplerInfo.maxAnisotropy = maxAnisotropy;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.mipmapMode = (filter == VK_FILTER_LINEAR)
        ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.mipLodBias = mipLodBias;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = static_cast<float>(mipLevels_ > 0 ? mipLevels_ - 1 : 0);
    return finalizeSampler(device, samplerInfo);
}
void VkTexture::destroy(VkDevice device, VmaAllocator allocator) {
    // Nothing was ever created, or this has already run. Both are ordinary:
    // the destructor calls this after an explicit destroy() has, and a
    // default-constructed texture is destroyed without having been used.
    if (device == VK_NULL_HANDLE) {
        return;
    }
    if (sampler_ != VK_NULL_HANDLE && ownsSampler_) {
        vkDestroySampler(device, sampler_, nullptr);
    }
    sampler_ = VK_NULL_HANDLE;
    ownsSampler_ = false;
    destroyImage(device, allocator, image_);
    device_ = VK_NULL_HANDLE;
    allocator_ = VK_NULL_HANDLE;
}

VkDescriptorImageInfo VkTexture::descriptorInfo(VkImageLayout layout) const {
    VkDescriptorImageInfo info{};
    info.sampler = sampler_;
    info.imageView = image_.imageView;
    info.imageLayout = layout;
    return info;
}

void VkTexture::generateMipmaps(VkContext& ctx, VkFormat format,
    uint32_t width, uint32_t height)
{
    // Check if format supports linear blitting
    VkFormatProperties formatProperties;
    vkGetPhysicalDeviceFormatProperties(ctx.getPhysicalDevice(), format, &formatProperties);

    bool canBlit = (formatProperties.optimalTilingFeatures &
        VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT) != 0;

    if (!canBlit) {
        LOG_WARNING("Format does not support linear blitting for mipmap generation");
        // Fall back to simple transition
        ctx.immediateSubmit([&](VkCommandBuffer cmd) {
            transitionImageLayout(cmd, image_.image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        });
        return;
    }

    ctx.immediateSubmit([&](VkCommandBuffer cmd) {
        int32_t mipW = static_cast<int32_t>(width);
        int32_t mipH = static_cast<int32_t>(height);

        for (uint32_t i = 1; i < mipLevels_; i++) {
            // Transition previous mip to transfer src
            VkImageMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.image = image_.image;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.baseMipLevel = i - 1;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

            vkCmdPipelineBarrier(cmd,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                0, 0, nullptr, 0, nullptr, 1, &barrier);

            // Blit from previous mip to current
            VkImageBlit blit{};
            blit.srcOffsets[0] = {0, 0, 0};
            blit.srcOffsets[1] = {mipW, mipH, 1};
            blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.srcSubresource.mipLevel = i - 1;
            blit.srcSubresource.layerCount = 1;
            blit.dstOffsets[0] = {0, 0, 0};
            blit.dstOffsets[1] = {
                mipW > 1 ? mipW / 2 : 1,
                mipH > 1 ? mipH / 2 : 1,
                1
            };
            blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.dstSubresource.mipLevel = i;
            blit.dstSubresource.layerCount = 1;

            vkCmdBlitImage(cmd,
                image_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                image_.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1, &blit, VK_FILTER_LINEAR);

            // Transition previous mip to shader read
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            vkCmdPipelineBarrier(cmd,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                0, 0, nullptr, 0, nullptr, 1, &barrier);

            mipW = mipW > 1 ? mipW / 2 : 1;
            mipH = mipH > 1 ? mipH / 2 : 1;
        }

        // Transition last mip to shader read
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.image = image_.image;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = mipLevels_ - 1;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0, 0, nullptr, 0, nullptr, 1, &barrier);
    });
}

} // namespace rendering
} // namespace wowee
