#pragma once

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <cstdint>
#include <mutex>
#include <vector>

namespace wowee {
namespace rendering {

class VkContext;

/**
 * Reads finished frames back from the GPU for the screen recorder, and draws
 * the dot that says a recording is running.
 *
 * A frame is copied out at the very end of the frame's own command buffer -
 * interface and all, since a recording should show what the player saw - into
 * one of a few host-visible buffers, scaled on the way when the recording is
 * smaller than the window. Nothing waits for it: the buffer is handed over
 * when that frame slot's fence has signalled, two frames later, and comes back
 * when the encoder has read it. With every buffer out the frame is dropped
 * rather than the game stalled.
 *
 * The dot is drawn after the copy, in a second instance of the overlay pass,
 * so it is on the screen and not in the video.
 */
class ScreenCapture {
public:
    /// A frame whose copy is complete.
    struct Ready {
        const uint8_t* bgra = nullptr;
        uint32_t stride = 0;
        int64_t pts = 0;
        uint32_t buffer = 0;   ///< give back through release()
        bool valid = false;
    };

    ScreenCapture() = default;
    ~ScreenCapture();
    ScreenCapture(const ScreenCapture&) = delete;
    ScreenCapture& operator=(const ScreenCapture&) = delete;

    /// Frames come out width x height, BGRA.
    [[nodiscard]] bool initialize(VkContext* ctx, uint32_t width, uint32_t height);
    /// Waits for the device: buffers may still be the target of a copy.
    void shutdown();

    /// Copy the finished swapchain image, which is in PRESENT_SRC and is left
    /// there. False when every buffer is still with the encoder.
    bool record(VkCommandBuffer cmd, uint32_t frameSlot, VkImage swapchainImage,
                VkExtent2D extent, int64_t pts);

    /// After frameSlot's fence: the frame it copied, if it copied one.
    Ready collect(uint32_t frameSlot);

    /// Hand a buffer back. Any thread.
    void release(uint32_t buffer);

    /// The recording dot, in the top right corner. Opens and closes its own
    /// instance of the overlay pass; the swapchain image is in PRESENT_SRC
    /// before and after.
    void drawIndicator(VkCommandBuffer cmd, VkFramebuffer overlayFramebuffer,
                       VkExtent2D extent, float seconds);

private:
    static constexpr uint32_t MAX_FRAMES = 2;
    /// Two frames in flight on the GPU and two with the encoder.
    static constexpr uint32_t kBuffers = 4;

    struct Buffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation alloc = VK_NULL_HANDLE;
        const uint8_t* mapped = nullptr;
    };

    bool createIndicatorPipeline();

    VkContext* ctx_ = nullptr;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    VkFormat copyFormat_ = VK_FORMAT_B8G8R8A8_UNORM;

    Buffer buffers_[kBuffers];
    std::mutex freeMutex_;
    std::vector<uint32_t> free_;

    struct Pending {
        int buffer = -1;
        int64_t pts = 0;
    };
    Pending pending_[MAX_FRAMES];

    // The recording's size on the GPU, for a window that is not that size.
    VkImage scaled_ = VK_NULL_HANDLE;
    VmaAllocation scaledAlloc_ = VK_NULL_HANDLE;

    VkPipelineLayout indicatorLayout_ = VK_NULL_HANDLE;
    VkPipeline indicatorPipeline_ = VK_NULL_HANDLE;
};

} // namespace rendering
} // namespace wowee
