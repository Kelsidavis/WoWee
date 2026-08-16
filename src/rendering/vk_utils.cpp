#include "rendering/vk_utils.hpp"

#include <cstdio>
#include "rendering/vk_context.hpp"
#include "core/logger.hpp"
#include <cstring>

namespace wowee {
namespace rendering {

AllocatedBuffer createBuffer(VmaAllocator allocator, VkDeviceSize size,
    VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage)
{
    AllocatedBuffer result{};

    VkBufferCreateInfo bufInfo{};
    bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufInfo.size = size;
    bufInfo.usage = usage;
    bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = memoryUsage;
    if (memoryUsage == VMA_MEMORY_USAGE_CPU_TO_GPU || memoryUsage == VMA_MEMORY_USAGE_CPU_ONLY) {
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    }

    if (vmaCreateBuffer(allocator, &bufInfo, &allocInfo,
            &result.buffer, &result.allocation, &result.info) != VK_SUCCESS) {
        LOG_ERROR("Failed to create VMA buffer (size=", size, ")");
    }

    return result;
}

void destroyBuffer(VmaAllocator allocator, AllocatedBuffer& buffer) {
    destroy(allocator, buffer.buffer, buffer.allocation);
}

AllocatedImage createImage(VkDevice device, VmaAllocator allocator,
    uint32_t width, uint32_t height, VkFormat format,
    VkImageUsageFlags usage, VkSampleCountFlagBits samples, uint32_t mipLevels)
{
    AllocatedImage result{};
    result.extent = {width, height};
    result.format = format;

    VkImageCreateInfo imgInfo{};
    imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgInfo.imageType = VK_IMAGE_TYPE_2D;
    imgInfo.format = format;
    imgInfo.extent = {width, height, 1};
    imgInfo.mipLevels = mipLevels;
    imgInfo.arrayLayers = 1;
    imgInfo.samples = samples;
    imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgInfo.usage = usage;
    imgInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    if (vmaCreateImage(allocator, &imgInfo, &allocInfo,
            &result.image, &result.allocation, nullptr) != VK_SUCCESS) {
        LOG_ERROR("Failed to create VMA image (", width, "x", height, ")");
        return result;
    }

    // Name it by shape, so an allocation that survives shutdown can be traced
    // back from the leak dump to whatever asked for an image this size.
    {
        char name[64];
        std::snprintf(name, sizeof(name), "img %ux%u mips=%u fmt=%d",
                      width, height, mipLevels, static_cast<int>(format));
        vmaSetAllocationName(allocator, result.allocation, name);
    }

    // Create image view
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = result.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = format;

    // Determine aspect mask from format
    if (format == VK_FORMAT_D32_SFLOAT || format == VK_FORMAT_D16_UNORM ||
        format == VK_FORMAT_D24_UNORM_S8_UINT || format == VK_FORMAT_D32_SFLOAT_S8_UINT) {
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    } else {
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    }
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = mipLevels;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device, &viewInfo, nullptr, &result.imageView) != VK_SUCCESS) {
        LOG_ERROR("Failed to create image view");
    }

    return result;
}

void destroyImage(VkDevice device, VmaAllocator allocator, AllocatedImage& image) {
    if (image.imageView) {
        vkDestroyImageView(device, image.imageView, nullptr);
        image.imageView = VK_NULL_HANDLE;
    }
    if (image.image) {
        vmaDestroyImage(allocator, image.image, image.allocation);
        image.image = VK_NULL_HANDLE;
        image.allocation = VK_NULL_HANDLE;
    }
}

void transitionImageLayout(VkCommandBuffer cmd, VkImage image,
    VkImageLayout oldLayout, VkImageLayout newLayout,
    VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage)
{
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;

    if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL ||
        newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL) {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    } else {
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    }

    // Set access masks based on layouts
    switch (oldLayout) {
        case VK_IMAGE_LAYOUT_UNDEFINED:
            barrier.srcAccessMask = 0;
            break;
        case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            break;
        case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
            barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            break;
        case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
            barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
            break;
        default:
            barrier.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
            break;
    }

    switch (newLayout) {
        case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            break;
        case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            break;
        case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
            barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            break;
        case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
            barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            break;
        case VK_IMAGE_LAYOUT_PRESENT_SRC_KHR:
            barrier.dstAccessMask = 0;
            break;
        default:
            barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
            break;
    }

    vkCmdPipelineBarrier(cmd, srcStage, dstStage, 0,
        0, nullptr, 0, nullptr, 1, &barrier);
}

AllocatedBuffer uploadBuffer(VkContext& ctx, const void* data, VkDeviceSize size,
    VkBufferUsageFlags usage)
{
    // Create staging buffer
    AllocatedBuffer staging = createBuffer(ctx.getAllocator(), size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);

    // Copy data to staging
    void* mapped;
    vmaMapMemory(ctx.getAllocator(), staging.allocation, &mapped);
    std::memcpy(mapped, data, size);
    vmaUnmapMemory(ctx.getAllocator(), staging.allocation);

    // Create GPU buffer
    AllocatedBuffer gpuBuffer = createBuffer(ctx.getAllocator(), size,
        usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Copy staging -> GPU
    ctx.immediateSubmit([&](VkCommandBuffer cmd) {
        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(cmd, staging.buffer, gpuBuffer.buffer, 1, &copyRegion);
    });

    // Destroy staging buffer (deferred if in batch mode)
    if (ctx.isInUploadBatch()) {
        ctx.deferStagingCleanup(staging);
    } else {
        destroyBuffer(ctx.getAllocator(), staging);
    }

    return gpuBuffer;
}

} // namespace rendering
} // namespace wowee
