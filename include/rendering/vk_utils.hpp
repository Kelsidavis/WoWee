#pragma once

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <cstdint>

namespace wowee {
namespace rendering {

class VkContext;

struct AllocatedBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VmaAllocationInfo info{};
};

struct AllocatedImage {
    VkImage image = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VkImageView imageView = VK_NULL_HANDLE;
    VkExtent2D extent{};
    VkFormat format = VK_FORMAT_UNDEFINED;
};

// Buffer creation
AllocatedBuffer createBuffer(VmaAllocator allocator, VkDeviceSize size,
    VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage);

void destroyBuffer(VmaAllocator allocator, AllocatedBuffer& buffer);

// Image creation
AllocatedImage createImage(VkDevice device, VmaAllocator allocator,
    uint32_t width, uint32_t height, VkFormat format,
    VkImageUsageFlags usage, VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT,
    uint32_t mipLevels = 1);

void destroyImage(VkDevice device, VmaAllocator allocator, AllocatedImage& image);

// Image layout transitions
void transitionImageLayout(VkCommandBuffer cmd, VkImage image,
    VkImageLayout oldLayout, VkImageLayout newLayout,
    VkPipelineStageFlags srcStage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
    VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);

// Staging upload helper — copies CPU data to a GPU-local buffer
AllocatedBuffer uploadBuffer(VkContext& ctx, const void* data, VkDeviceSize size,
    VkBufferUsageFlags usage);

// Check VkResult and log on failure
inline bool vkCheck(VkResult result, [[maybe_unused]] const char* msg) {
    if (result != VK_SUCCESS) {
        // Caller should log the message
        return false;
    }
    return true;
}

} // namespace rendering
} // namespace wowee
