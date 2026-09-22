#include "rendering/screen_capture.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

namespace wowee {
namespace rendering {

ScreenCapture::~ScreenCapture() {
    shutdown();
}

bool ScreenCapture::initialize(VkContext* ctx, uint32_t width, uint32_t height) {
    if (!ctx || width == 0 || height == 0) return false;
    ctx_ = ctx;
    width_ = width;
    height_ = height;

    // Copied as it is when the swapchain already holds BGRA bytes, so what the
    // encoder reads is what was on screen. Anything else is blitted into BGRA
    // on the way, which converts it.
    const VkFormat swapFormat = ctx_->getSwapchainFormat();
    const bool swapIsBgra = swapFormat == VK_FORMAT_B8G8R8A8_UNORM || swapFormat == VK_FORMAT_B8G8R8A8_SRGB;
    copyFormat_ = swapIsBgra ? swapFormat : VK_FORMAT_B8G8R8A8_UNORM;

    VkFormatProperties swapProps{};
    VkFormatProperties copyProps{};
    vkGetPhysicalDeviceFormatProperties(ctx_->getPhysicalDevice(), swapFormat, &swapProps);
    vkGetPhysicalDeviceFormatProperties(ctx_->getPhysicalDevice(), copyFormat_, &copyProps);
    if (!(swapProps.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) ||
        !(copyProps.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT)) {
        LOG_WARNING("ScreenCapture: the swapchain cannot be blitted here");
        return false;
    }

    VmaAllocator allocator = ctx_->getAllocator();
    const VkDeviceSize bytes = static_cast<VkDeviceSize>(width_) * height_ * 4;
    for (auto& buffer : buffers_) {
        VkBufferCreateInfo bufCI{.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufCI.size = bytes;
        bufCI.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        VmaAllocationCreateInfo allocCI{};
        // Read by the CPU, so host-cached where the device offers it.
        allocCI.usage = VMA_MEMORY_USAGE_GPU_TO_CPU;
        allocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info{};
        if (vmaCreateBuffer(allocator, &bufCI, &allocCI, &buffer.buffer, &buffer.alloc, &info) != VK_SUCCESS ||
            !info.pMappedData) {
            LOG_WARNING("ScreenCapture: could not allocate ", bytes / (1024 * 1024), " MB readback buffers");
            return false;
        }
        buffer.mapped = static_cast<const uint8_t*>(info.pMappedData);
    }
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        free_.clear();
        for (uint32_t i = 0; i < kBuffers; ++i) free_.push_back(i);
    }
    for (auto& p : pending_) p = Pending{};

    VkImageCreateInfo imgCI{};
    imgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgCI.imageType = VK_IMAGE_TYPE_2D;
    imgCI.format = copyFormat_;
    imgCI.extent = {.width = width_, .height = height_, .depth = 1};
    imgCI.mipLevels = 1;
    imgCI.arrayLayers = 1;
    imgCI.samples = VK_SAMPLE_COUNT_1_BIT;
    imgCI.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgCI.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imgCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VmaAllocationCreateInfo imgAlloc{};
    imgAlloc.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    if (vmaCreateImage(allocator, &imgCI, &imgAlloc, &scaled_, &scaledAlloc_, nullptr) != VK_SUCCESS) {
        LOG_WARNING("ScreenCapture: could not allocate the scaling image");
        return false;
    }

    return createIndicatorPipeline();
}

bool ScreenCapture::createIndicatorPipeline() {
    VkDevice device = ctx_->getDevice();
    const VkPushConstantRange push{.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT, .offset = 0,
                                   .size = sizeof(glm::vec4)};
    indicatorLayout_ = createPipelineLayout(device, {}, {push});
    if (indicatorLayout_ == VK_NULL_HANDLE) return false;

    ShaderPair shaders = loadShaderPair(device, "assets/shaders/postprocess.vert.spv",
                                        "assets/shaders/recording_dot.frag.spv", "recording dot");
    if (!shaders) return false;
    indicatorPipeline_ = PipelineBuilder()
        .setShaders(shaders.vertStage, shaders.fragStage)
        .setVertexInput({}, {})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(VK_SAMPLE_COUNT_1_BIT)
        .setLayout(indicatorLayout_)
        .setRenderPass(ctx_->getOverlayRenderPass())
        .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
        .build(device, ctx_->getPipelineCache());
    if (indicatorPipeline_ == VK_NULL_HANDLE) {
        LOG_WARNING("ScreenCapture: could not build the recording dot's pipeline");
        return false;
    }
    return true;
}

bool ScreenCapture::record(VkCommandBuffer cmd, uint32_t frameSlot, VkImage swapchainImage,
                           VkExtent2D extent, int64_t pts) {
    if (!ctx_ || frameSlot >= MAX_FRAMES || cmd == VK_NULL_HANDLE || swapchainImage == VK_NULL_HANDLE ||
        extent.width == 0 || extent.height == 0) {
        return false;
    }
    // A slot's previous copy is collected when its fence is waited on, before
    // this frame began. One still here means that was skipped; its buffer is
    // taken back rather than lost.
    if (pending_[frameSlot].buffer >= 0) {
        release(static_cast<uint32_t>(pending_[frameSlot].buffer));
        pending_[frameSlot] = Pending{};
    }
    uint32_t index = 0;
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        if (free_.empty()) return false;
        index = free_.back();
        free_.pop_back();
    }

    const auto imageBarrier = [&](VkImage image, VkImageLayout from, VkImageLayout to,
                                  VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                                  VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
        VkImageMemoryBarrier2 b{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        b.srcStageMask = srcStage;
        b.srcAccessMask = srcAccess;
        b.dstStageMask = dstStage;
        b.dstAccessMask = dstAccess;
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        VkDependencyInfo dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dep.imageMemoryBarrierCount = 1;
        dep.pImageMemoryBarriers = &b;
        cmdPipelineBarrier2(cmd, dep);
    };

    imageBarrier(swapchainImage, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                 VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                 VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);

    VkBufferImageCopy region{};
    region.imageSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
    region.imageExtent = {.width = width_, .height = height_, .depth = 1};

    const bool sameSize = extent.width == width_ && extent.height == height_;
    if (sameSize && copyFormat_ == ctx_->getSwapchainFormat()) {
        vkCmdCopyImageToBuffer(cmd, swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               buffers_[index].buffer, 1, &region);
    } else {
        // Its last copy out, a frame or two ago, has to be done before this
        // blit overwrites it; its contents do not.
        imageBarrier(scaled_, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, 0,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        VkImageBlit blit{};
        blit.srcSubresource = region.imageSubresource;
        blit.dstSubresource = region.imageSubresource;
        blit.srcOffsets[1] = {.x = static_cast<int32_t>(extent.width), .y = static_cast<int32_t>(extent.height), .z = 1};
        blit.dstOffsets[1] = {.x = static_cast<int32_t>(width_), .y = static_cast<int32_t>(height_), .z = 1};
        vkCmdBlitImage(cmd, swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       scaled_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);
        imageBarrier(scaled_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
        vkCmdCopyImageToBuffer(cmd, scaled_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               buffers_[index].buffer, 1, &region);
    }

    // Back where presenting, and the dot's pass, expect it.
    imageBarrier(swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                 VK_PIPELINE_STAGE_2_TRANSFER_BIT, 0,
                 VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                 VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

    // And the bytes visible to the CPU once the fence says the copy is done.
    VkBufferMemoryBarrier2 toHost{.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2};
    toHost.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    toHost.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    toHost.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
    toHost.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
    toHost.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    toHost.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    toHost.buffer = buffers_[index].buffer;
    toHost.offset = 0;
    toHost.size = VK_WHOLE_SIZE;
    VkDependencyInfo dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dep.bufferMemoryBarrierCount = 1;
    dep.pBufferMemoryBarriers = &toHost;
    cmdPipelineBarrier2(cmd, dep);

    pending_[frameSlot] = Pending{.buffer = static_cast<int>(index), .pts = pts};
    return true;
}

ScreenCapture::Ready ScreenCapture::collect(uint32_t frameSlot) {
    Ready ready;
    if (!ctx_ || frameSlot >= MAX_FRAMES || pending_[frameSlot].buffer < 0) return ready;
    const auto index = static_cast<uint32_t>(pending_[frameSlot].buffer);
    vmaInvalidateAllocation(ctx_->getAllocator(), buffers_[index].alloc, 0, VK_WHOLE_SIZE);
    ready.bgra = buffers_[index].mapped;
    ready.stride = width_ * 4;
    ready.pts = pending_[frameSlot].pts;
    ready.buffer = index;
    ready.valid = true;
    pending_[frameSlot] = Pending{};
    return ready;
}

void ScreenCapture::release(uint32_t buffer) {
    if (buffer >= kBuffers) return;
    std::lock_guard<std::mutex> lock(freeMutex_);
    if (std::find(free_.begin(), free_.end(), buffer) == free_.end()) free_.push_back(buffer);
}

void ScreenCapture::drawIndicator(VkCommandBuffer cmd, VkFramebuffer overlayFramebuffer,
                                  VkExtent2D extent, float seconds) {
    if (!ctx_ || indicatorPipeline_ == VK_NULL_HANDLE || overlayFramebuffer == VK_NULL_HANDLE ||
        extent.width < 64 || extent.height < 64) {
        return;
    }
    // Sized to the screen's height, so it is the same dot at 900 and at 2160.
    const float scale = static_cast<float>(extent.height) / 1080.0f;
    const float radius = std::max(5.0f, 9.0f * scale);
    const glm::vec2 centre(static_cast<float>(extent.width) - 30.0f * scale, 30.0f * scale);
    // A slow pulse, a second from bright to dim and back.
    const float opacity = 0.6f + 0.4f * (0.5f + 0.5f * std::cos(seconds * 6.2831853f));
    const glm::vec4 push(centre, radius, opacity);

    VkRenderPassBeginInfo rp{.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rp.renderPass = ctx_->getOverlayRenderPass();
    rp.framebuffer = overlayFramebuffer;
    rp.renderArea.extent = extent;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{};
    vp.width = static_cast<float>(extent.width);
    vp.height = static_cast<float>(extent.height);
    vp.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &vp);
    // Only the dot's own square is shaded.
    const float reach = radius + 4.0f;
    VkRect2D box{};
    box.offset = {.x = std::max(0, static_cast<int32_t>(centre.x - reach)),
                  .y = std::max(0, static_cast<int32_t>(centre.y - reach))};
    box.extent = {.width = static_cast<uint32_t>(reach * 2.0f) + 1,
                  .height = static_cast<uint32_t>(reach * 2.0f) + 1};
    box.extent.width = std::min(box.extent.width, extent.width - static_cast<uint32_t>(box.offset.x));
    box.extent.height = std::min(box.extent.height, extent.height - static_cast<uint32_t>(box.offset.y));
    vkCmdSetScissor(cmd, 0, 1, &box);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, indicatorPipeline_);
    vkCmdPushConstants(cmd, indicatorLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
}

void ScreenCapture::shutdown() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();
    vkDeviceWaitIdle(device);
    VmaAllocator allocator = ctx_->getAllocator();
    for (auto& b : buffers_) {
        destroy(allocator, b.buffer, b.alloc);
        b.mapped = nullptr;
    }
    if (scaled_) {
        vmaDestroyImage(allocator, scaled_, scaledAlloc_);
        scaled_ = VK_NULL_HANDLE;
        scaledAlloc_ = VK_NULL_HANDLE;
    }
    destroy(device, indicatorPipeline_);
    destroy(device, indicatorLayout_);
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        free_.clear();
    }
    for (auto& p : pending_) p = Pending{};
    ctx_ = nullptr;
}

} // namespace rendering
} // namespace wowee
