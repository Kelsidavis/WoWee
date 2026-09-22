#pragma once

// A second window's swapchain, drawn into the main frame.
//
// The client was written for one window: one surface, one swapchain, one
// present, and a frame ring whose deferred destruction and per-frame
// descriptor sets assume a single submit per frame. A second window that kept
// a frame ring of its own would have to be taught all of that. This one does
// not - it records into the main frame's command buffer, and VkContext's
// endFrame waits on its image, signals its semaphore and presents it beside the
// main one (VkContext::addExtraPresent). The main frame's fence covers both.
//
// It never makes the main window wait. An image is taken only if one is ready
// right now; if the window's display is not ready for another frame, this
// frame simply does not draw it.

#include <vulkan/vulkan.h>
#include <SDL3/SDL.h>
#include <cstdint>
#include <functional>
#include <vector>

namespace wowee {
namespace rendering {

class VkContext;

class AuxSwapchain {
public:
    AuxSwapchain() = default;
    AuxSwapchain(const AuxSwapchain&) = delete;
    AuxSwapchain& operator=(const AuxSwapchain&) = delete;
    ~AuxSwapchain();

    /// A surface for the window and a swapchain on it. False, with the reason
    /// logged, when the device cannot present to that window.
    bool create(VkContext* ctx, SDL_Window* window);
    void destroy();

    /// The window was resized or moved to a display of another density; the
    /// swapchain is rebuilt before the next image is taken.
    void markDirty() { dirty_ = true; }

    /// Take an image if one is ready now. False when none is, when the window
    /// has no area to draw (minimised), or when the swapchain could not be
    /// rebuilt - in every case this frame simply does not draw the window.
    bool acquire();

    /// Draw into the image acquire() took and queue it to be presented with
    /// the frame. The render pass is begun cleared to `clear`, with the
    /// viewport and scissor covering the whole image.
    void record(VkCommandBuffer cmd, const float clear[4],
                const std::function<void(VkCommandBuffer)>& draw);

    /// The pass `draw` records inside. Kept across rebuilds - only the format
    /// decides it, and a pipeline built against it stays valid.
    [[nodiscard]] VkRenderPass renderPass() const { return renderPass_; }
    [[nodiscard]] uint32_t imageCount() const { return static_cast<uint32_t>(images_.size()); }
    [[nodiscard]] VkExtent2D extent() const { return extent_; }

private:
    bool rebuild();
    void destroySwapchainResources();
    void createSemaphores();
    void destroySemaphores();

    VkContext* ctx_ = nullptr;
    SDL_Window* window_ = nullptr;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
    VkExtent2D extent_{0, 0};
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    std::vector<VkImage> images_;
    std::vector<VkImageView> views_;
    std::vector<VkFramebuffer> framebuffers_;

    // The same arrangement as the main swapchain's: acquire signals the spare
    // semaphore, which is then swapped into the acquired image's slot, and the
    // image's previous one - released by the presentation engine when the
    // image came back - becomes the spare.
    std::vector<VkSemaphore> imageAcquired_;
    VkSemaphore spareAcquire_ = VK_NULL_HANDLE;
    std::vector<VkSemaphore> rendered_;

    uint32_t imageIndex_ = 0;
    bool acquired_ = false;
    bool dirty_ = false;
    bool loggedAcquireFailure_ = false;
    uint64_t syncGeneration_ = 0;
};

}  // namespace rendering
}  // namespace wowee
