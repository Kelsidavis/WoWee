#pragma once

#include <source_location>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <cstdint>
#include <limits>
#include <cstdlib>

#include "core/env_flag.hpp"

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

// Destroying a handle and forgetting it, which the renderers do about two
// hundred and twenty times between them.
//
// Every one of those sites was the same four lines: test the handle, destroy
// it, set it back to VK_NULL_HANDLE. Written out, the last step is easy to
// leave off, and a handle that still holds a destroyed object is a double
// destroy the next time a shutdown runs. Taking the handle by reference makes
// forgetting it part of destroying it.

inline void destroy(VkDevice device, VkPipeline& pipeline) {
    if (pipeline == VK_NULL_HANDLE) return;
    vkDestroyPipeline(device, pipeline, nullptr);
    pipeline = VK_NULL_HANDLE;
}

inline void destroy(VkDevice device, VkPipelineLayout& layout) {
    if (layout == VK_NULL_HANDLE) return;
    vkDestroyPipelineLayout(device, layout, nullptr);
    layout = VK_NULL_HANDLE;
}

inline void destroy(VkDevice device, VkDescriptorSetLayout& layout) {
    if (layout == VK_NULL_HANDLE) return;
    vkDestroyDescriptorSetLayout(device, layout, nullptr);
    layout = VK_NULL_HANDLE;
}

inline void destroy(VkDevice device, VkDescriptorPool& pool) {
    if (pool == VK_NULL_HANDLE) return;
    vkDestroyDescriptorPool(device, pool, nullptr);
    pool = VK_NULL_HANDLE;
}

inline void destroy(VkDevice device, VkSampler& sampler) {
    if (sampler == VK_NULL_HANDLE) return;
    vkDestroySampler(device, sampler, nullptr);
    sampler = VK_NULL_HANDLE;
}

/// A buffer and its allocation go together, and forgetting one of the two is
/// the same fault as forgetting the handle.
inline void destroy(VmaAllocator allocator, VkBuffer& buffer, VmaAllocation& allocation) {
    if (buffer == VK_NULL_HANDLE) return;
    vmaDestroyBuffer(allocator, buffer, allocation);
    buffer = VK_NULL_HANDLE;
    allocation = VK_NULL_HANDLE;
}

/// A pipeline and the layout it was built with.
///
/// They are created together and every renderer releases them together, in
/// the same two calls behind the same null check - seven of them wrote it out.
/// Forgetting the layout leaks it for the run and nothing reports it.
inline void destroyPipeline(VkDevice device, VkPipeline& pipeline,
                            VkPipelineLayout& layout) {
    destroy(device, pipeline);
    destroy(device, layout);
}

/// Everything a particle system holds on the device.
///
/// MountDust and Weather are the same system with different sprites, and each
/// wrote this out: a pipeline, its layout, and one dynamic vertex buffer with
/// its allocation. A teardown that forgets one of the four leaks for the run,
/// and nothing reports it, so the two are better off unable to disagree.
inline void destroyParticleResources(VkDevice device, VmaAllocator allocator,
                                     VkPipeline& pipeline,
                                     VkPipelineLayout& layout,
                                     VkBuffer& vertexBuffer,
                                     VmaAllocation& vertexAllocation) {
    destroyPipeline(device, pipeline, layout);
    destroy(allocator, vertexBuffer, vertexAllocation);
}

// Buffer creation
AllocatedBuffer createBuffer(VmaAllocator allocator, VkDeviceSize size,
    VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage);

void destroyBuffer(VmaAllocator allocator, AllocatedBuffer& buffer);

// Image creation
AllocatedImage createImage(VkDevice device, VmaAllocator allocator,
    uint32_t width, uint32_t height, VkFormat format,
    VkImageUsageFlags usage, VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT,
    uint32_t mipLevels = 1,
    /// Defaulted, so the allocation records who asked for it without any call
    /// site having to say. Used only to name the allocation for the shutdown
    /// leak dump.
    std::source_location where = std::source_location::current());

void destroyImage(VkDevice device, VmaAllocator allocator, AllocatedImage& image);

/// Give a Vulkan object a name the validation layer will print.
///
/// The leak report at vkDestroyDevice lists what was never destroyed as bare
/// handles - "VkImage 0x2f7ce000002f7ce" - which says nothing about where it
/// came from. Named, the same line carries the file and line that created it,
/// which is how the VMA allocation names turned the last leak hunt from
/// guesswork into a lookup.
///
/// Does nothing when VK_EXT_debug_utils is absent, so it costs nothing in a
/// build without validation.
void setObjectName(VkDevice device, VkObjectType type, uint64_t handle, const char* name);

/// Set by VkContext once the device exists; nullptr disables naming.
void setObjectNameFn(PFN_vkSetDebugUtilsObjectNameEXT fn);

/// Whether naming resolved, which is true exactly when validation is on.
/// Used to gate logging that only matters while chasing a leak report.
bool isObjectNamingActive();

/// Record a dependency described the synchronization2 way.
///
/// With VK_KHR_synchronization2 present this is vkCmdPipelineBarrier2KHR.
/// Without it the same dependency is lowered to vkCmdPipelineBarrier: the
/// per-barrier stage masks are ORed into the single pair the legacy call
/// takes, which is the only thing a legacy barrier could express anyway.
///
/// Barriers are written once, in the newer form, on both paths.
void cmdPipelineBarrier2(VkCommandBuffer cmd, const VkDependencyInfo& dep);

/// Set by VkContext once the device exists; nullptr selects the legacy path.
/// One device per process, which VkContext already assumes.
void setPipelineBarrier2Fn(PFN_vkCmdPipelineBarrier2KHR fn);

// Image layout transitions
void transitionImageLayout(VkCommandBuffer cmd, VkImage image,
    VkImageLayout oldLayout, VkImageLayout newLayout,
    VkPipelineStageFlags srcStage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
    VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);

// Staging upload helper - copies CPU data to a GPU-local buffer
AllocatedBuffer uploadBuffer(VkContext& ctx, const void* data, VkDeviceSize size,
    VkBufferUsageFlags usage);

// Environment variable utility functions
inline size_t envSizeMBOrDefault(const char* name, size_t defMb) {
    const char* v = std::getenv(name);
    if (!v || !*v) return defMb;
    char* end = nullptr;
    unsigned long long mb = std::strtoull(v, &end, 10);
    if (end == v || mb == 0) return defMb;
    if (mb > (std::numeric_limits<size_t>::max() / (1024ull * 1024ull))) return defMb;
    return static_cast<size_t>(mb);
}

inline size_t envSizeOrDefault(const char* name, size_t defValue) {
    const char* v = std::getenv(name);
    if (!v || !*v) return defValue;
    char* end = nullptr;
    unsigned long long n = std::strtoull(v, &end, 10);
    if (end == v || n == 0) return defValue;
    return static_cast<size_t>(n);
}

// Opt-in rendering diagnostics, read once per process. These exist to bisect a
// visual artifact to the subsystem that draws it: turn one off and see whether
// the artifact survives.
//
// Through the one reader in core/env_flag.hpp. This was a third rule: only the
// exact string "0" disabled, so WOWEE_M2_NO_PARTICLES=off switched the
// suppression on - the opposite of what anyone typing it means, on a flag
// whose whole job is to be turned on and off while chasing an artifact.
inline bool envFlagEnabled(const char* name) {
    return core::envFlagEnabled(name, false);
}

} // namespace rendering
} // namespace wowee
