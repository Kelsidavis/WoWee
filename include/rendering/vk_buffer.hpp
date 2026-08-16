#pragma once

#include "rendering/vk_utils.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <cstdint>
#include <cstring>

namespace wowee {
namespace rendering {

class VkContext;

// RAII wrapper for a Vulkan buffer with VMA allocation.
// Supports vertex, index, uniform, and storage buffer usage.
class VkBuffer {
public:
    VkBuffer() = default;
    ~VkBuffer();

    VkBuffer(const VkBuffer&) = delete;
    VkBuffer& operator=(const VkBuffer&) = delete;
    VkBuffer(VkBuffer&& other) noexcept;
    VkBuffer& operator=(VkBuffer&& other) noexcept;

    // Create a GPU-local buffer and upload data via staging
    [[nodiscard]] bool uploadToGPU(VkContext& ctx, const void* data, VkDeviceSize size,
        VkBufferUsageFlags usage);

    // Create a host-visible buffer (for uniform/dynamic data updated each frame)
    [[nodiscard]] bool createMapped(VmaAllocator allocator, VkDeviceSize size,
        VkBufferUsageFlags usage);


    void destroy();

    [[nodiscard]] ::VkBuffer getBuffer() const { return buf_.buffer; }
    [[nodiscard]] VkDeviceSize getSize() const { return size_; }
    [[nodiscard]] void* getMappedData() const { return buf_.info.pMappedData; }
    [[nodiscard]] bool isValid() const { return buf_.buffer != VK_NULL_HANDLE; }

    // Descriptor info for uniform/storage buffer binding
    [[nodiscard]] VkDescriptorBufferInfo descriptorInfo(VkDeviceSize offset = 0,
        VkDeviceSize range = VK_WHOLE_SIZE) const;

private:
    AllocatedBuffer buf_{};
    VmaAllocator allocator_ = VK_NULL_HANDLE;
    VkDeviceSize size_ = 0;
};

} // namespace rendering
} // namespace wowee
