#include "rendering/aux_swapchain.hpp"

#include "rendering/vk_context.hpp"
#include "core/logger.hpp"

#include <SDL3/SDL_vulkan.h>
#include <VkBootstrap.h>

namespace wowee {
namespace rendering {

AuxSwapchain::~AuxSwapchain() { destroy(); }

bool AuxSwapchain::create(VkContext* ctx, SDL_Window* window) {
    destroy();
    ctx_ = ctx;
    window_ = window;
    if (!ctx_ || !window_) return false;

    if (!SDL_Vulkan_CreateSurface(window_, ctx_->getInstance(), nullptr, &surface_)) {
        LOG_ERROR("Second window: no Vulkan surface - ", SDL_GetError());
        surface_ = VK_NULL_HANDLE;
        return false;
    }

    // Every present goes to the one queue the main window presents on, so that
    // queue has to be able to reach this surface too. On one GPU driving both
    // displays it always can; said plainly when it cannot.
    VkBool32 supported = VK_FALSE;
    vkGetPhysicalDeviceSurfaceSupportKHR(ctx_->getPhysicalDevice(),
                                         ctx_->getPresentQueueFamily(), surface_,
                                         &supported);
    if (!supported) {
        LOG_ERROR("Second window: the GPU cannot present to it from the queue the "
                  "game presents on - is it on a display driven by another GPU?");
        destroy();
        return false;
    }

    syncGeneration_ = ctx_->syncResetGeneration();
    if (!rebuild()) {
        destroy();
        return false;
    }
    return true;
}

void AuxSwapchain::destroy() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();
    if (device != VK_NULL_HANDLE) vkDeviceWaitIdle(device);
    destroySwapchainResources();
    destroySemaphores();
    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device, swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }
    if (renderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(ctx_->getInstance(), surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }
    acquired_ = false;
    ctx_ = nullptr;
    window_ = nullptr;
}

void AuxSwapchain::destroySwapchainResources() {
    VkDevice device = ctx_ ? ctx_->getDevice() : VK_NULL_HANDLE;
    if (device == VK_NULL_HANDLE) return;
    for (VkFramebuffer fb : framebuffers_) vkDestroyFramebuffer(device, fb, nullptr);
    for (VkImageView view : views_) vkDestroyImageView(device, view, nullptr);
    framebuffers_.clear();
    views_.clear();
    images_.clear();
}

void AuxSwapchain::createSemaphores() {
    VkDevice device = ctx_->getDevice();
    VkSemaphoreCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    imageAcquired_.assign(images_.size(), VK_NULL_HANDLE);
    rendered_.assign(images_.size(), VK_NULL_HANDLE);
    for (size_t i = 0; i < images_.size(); ++i) {
        vkCreateSemaphore(device, &info, nullptr, &imageAcquired_[i]);
        vkCreateSemaphore(device, &info, nullptr, &rendered_[i]);
    }
    vkCreateSemaphore(device, &info, nullptr, &spareAcquire_);
}

void AuxSwapchain::destroySemaphores() {
    VkDevice device = ctx_ ? ctx_->getDevice() : VK_NULL_HANDLE;
    if (device == VK_NULL_HANDLE) return;
    for (VkSemaphore s : imageAcquired_) if (s) vkDestroySemaphore(device, s, nullptr);
    for (VkSemaphore s : rendered_) if (s) vkDestroySemaphore(device, s, nullptr);
    if (spareAcquire_) vkDestroySemaphore(device, spareAcquire_, nullptr);
    imageAcquired_.clear();
    rendered_.clear();
    spareAcquire_ = VK_NULL_HANDLE;
}

bool AuxSwapchain::rebuild() {
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window_, &w, &h);
    // Minimised, or mid-resize to nothing. Tried again once it has an area.
    if (w <= 0 || h <= 0) {
        dirty_ = true;
        return false;
    }

    VkDevice device = ctx_->getDevice();
    // The old images may still be in a frame the GPU has not finished, and
    // their views and framebuffers go with them.
    vkDeviceWaitIdle(device);

    vkb::SwapchainBuilder builder{ctx_->getPhysicalDevice(), device, surface_};
    builder.set_desired_format({.format = VK_FORMAT_B8G8R8A8_UNORM,
                                .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_extent(static_cast<uint32_t>(w), static_cast<uint32_t>(h))
        .set_image_usage_flags(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
        .set_desired_min_image_count(2)
        // FIFO everywhere: it is the one mode every driver has, and a map has
        // no use for frames the display will not show. acquire() never waits
        // for it, so it cannot hold the game to this display's refresh.
        .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
        .set_old_swapchain(swapchain_);
    auto built = builder.build();
    if (!built) {
        LOG_ERROR("Second window: swapchain not built - ", built.error().message());
        return false;
    }

    destroySwapchainResources();
    destroySemaphores();
    if (swapchain_ != VK_NULL_HANDLE) vkDestroySwapchainKHR(device, swapchain_, nullptr);

    vkb::Swapchain swap = built.value();
    swapchain_ = swap.swapchain;
    extent_ = swap.extent;
    images_ = swap.get_images().value();
    views_ = swap.get_image_views().value();

    // Built once, on the first format: every rebuild asks for the same one, and
    // the interface's pipelines for this window are built against this pass.
    if (renderPass_ == VK_NULL_HANDLE || swap.image_format != format_) {
        if (renderPass_ != VK_NULL_HANDLE) vkDestroyRenderPass(device, renderPass_, nullptr);
        format_ = swap.image_format;

        VkAttachmentDescription color{};
        color.format = format_;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorRef{};
        colorRef.attachment = 0;
        colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;

        // The image is not ours until its acquire semaphore signals, and the
        // submit waits on that at colour output - so the transition out of
        // UNDEFINED waits there too.
        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.srcAccessMask = 0;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpInfo.attachmentCount = 1;
        rpInfo.pAttachments = &color;
        rpInfo.subpassCount = 1;
        rpInfo.pSubpasses = &subpass;
        rpInfo.dependencyCount = 1;
        rpInfo.pDependencies = &dependency;
        if (vkCreateRenderPass(device, &rpInfo, nullptr, &renderPass_) != VK_SUCCESS) {
            LOG_ERROR("Second window: render pass not created");
            renderPass_ = VK_NULL_HANDLE;
            return false;
        }
    }

    framebuffers_.assign(views_.size(), VK_NULL_HANDLE);
    for (size_t i = 0; i < views_.size(); ++i) {
        VkFramebufferCreateInfo fb{};
        fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fb.renderPass = renderPass_;
        fb.attachmentCount = 1;
        fb.pAttachments = &views_[i];
        fb.width = extent_.width;
        fb.height = extent_.height;
        fb.layers = 1;
        if (vkCreateFramebuffer(device, &fb, nullptr, &framebuffers_[i]) != VK_SUCCESS) {
            LOG_ERROR("Second window: framebuffer ", i, " not created");
            return false;
        }
    }

    // Remade with the images: the device is idle, so none is in use, and a
    // rebuild can change how many images there are.
    createSemaphores();
    dirty_ = false;
    acquired_ = false;
    return true;
}

bool AuxSwapchain::acquire() {
    acquired_ = false;
    if (!ctx_ || surface_ == VK_NULL_HANDLE || ctx_->isDeviceLost()) return false;
    if (dirty_ || swapchain_ == VK_NULL_HANDLE) {
        if (!rebuild()) return false;
    }
    // A failed submit remade every semaphore the main window owns, because it
    // left some signalled; the same is true of these.
    if (ctx_->syncResetGeneration() != syncGeneration_) {
        syncGeneration_ = ctx_->syncResetGeneration();
        destroySemaphores();
        createSemaphores();
    }

    uint32_t index = 0;
    const VkResult result = vkAcquireNextImageKHR(ctx_->getDevice(), swapchain_, 0,
                                                  spareAcquire_, VK_NULL_HANDLE, &index);
    if (result == VK_TIMEOUT || result == VK_NOT_READY) return false;
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        dirty_ = true;
        return false;
    }
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        if (!loggedAcquireFailure_) {
            loggedAcquireFailure_ = true;
            LOG_WARNING("Second window: image not acquired (", static_cast<int>(result),
                        ") - it is not drawn until that changes");
        }
        return false;
    }
    // Suboptimal still hands over an image and signals the semaphore, so it is
    // drawn and presented; the rebuild waits for the next frame.
    if (result == VK_SUBOPTIMAL_KHR) dirty_ = true;

    const VkSemaphore justSignalled = spareAcquire_;
    spareAcquire_ = imageAcquired_[index];
    imageAcquired_[index] = justSignalled;
    imageIndex_ = index;
    acquired_ = true;
    return true;
}

void AuxSwapchain::record(VkCommandBuffer cmd, const float clear[4],
                          const std::function<void(VkCommandBuffer)>& draw) {
    if (!acquired_ || cmd == VK_NULL_HANDLE) return;
    acquired_ = false;

    VkClearValue clearValue{};
    clearValue.color = {{clear[0], clear[1], clear[2], clear[3]}};
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = renderPass_;
    rp.framebuffer = framebuffers_[imageIndex_];
    rp.renderArea.extent = extent_;
    rp.clearValueCount = 1;
    rp.pClearValues = &clearValue;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{};
    vp.width = static_cast<float>(extent_.width);
    vp.height = static_cast<float>(extent_.height);
    vp.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &vp);
    VkRect2D scissor{};
    scissor.extent = extent_;
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    if (draw) draw(cmd);
    vkCmdEndRenderPass(cmd);

    VkContext::ExtraPresent present;
    present.swapchain = swapchain_;
    present.imageIndex = imageIndex_;
    present.acquired = imageAcquired_[imageIndex_];
    present.rendered = rendered_[imageIndex_];
    present.onResult = [this](VkResult r) {
        if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR) dirty_ = true;
    };
    ctx_->addExtraPresent(std::move(present));
}

}  // namespace rendering
}  // namespace wowee
