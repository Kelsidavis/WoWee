#include "rendering/vk_render_target.hpp"
#include "rendering/vk_context.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {

VkRenderTarget::~VkRenderTarget() {
    // Must call destroy() explicitly with device/allocator before destruction
}

bool VkRenderTarget::create(VkContext& ctx, uint32_t width, uint32_t height,
                            VkFormat format, bool withDepth, VkSampleCountFlagBits msaaSamples) {
    VkDevice device = ctx.getDevice();
    VmaAllocator allocator = ctx.getAllocator();
    hasDepth_ = withDepth;
    msaaSamples_ = msaaSamples;
    bool useMSAA = msaaSamples != VK_SAMPLE_COUNT_1_BIT;

    // Create color image (multisampled if MSAA)
    colorImage_ = createImage(device, allocator, width, height, format,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | (useMSAA ? static_cast<VkImageUsageFlags>(0) : static_cast<VkImageUsageFlags>(VK_IMAGE_USAGE_SAMPLED_BIT)),
        msaaSamples);

    if (!colorImage_.image) {
        LOG_ERROR("VkRenderTarget: failed to create color image (", width, "x", height, ")");
        return false;
    }

    // Create resolve image for MSAA (single-sample, sampled for reading)
    if (useMSAA) {
        resolveImage_ = createImage(device, allocator, width, height, format,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
        if (!resolveImage_.image) {
            LOG_ERROR("VkRenderTarget: failed to create resolve image (", width, "x", height, ")");
            destroy(device, allocator);
            return false;
        }
    }

    // Create depth image if requested (multisampled to match color)
    if (withDepth) {
        depthImage_ = createImage(device, allocator, width, height,
            VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, msaaSamples);
        if (!depthImage_.image) {
            LOG_ERROR("VkRenderTarget: failed to create depth image (", width, "x", height, ")");
            destroy(device, allocator);
            return false;
        }
    }

    // Create sampler (linear filtering, clamp to edge)
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = 0.0f;

    if (vkCreateSampler(device, &samplerInfo, nullptr, &sampler_) != VK_SUCCESS) {
        LOG_ERROR("VkRenderTarget: failed to create sampler");
        destroy(device, allocator);
        return false;
    }

    // Create render pass
    if (useMSAA) {
        // MSAA render pass: color(MSAA) + resolve(1x) + optional depth(MSAA)
        // Attachment 0: MSAA color (rendered into, not stored after resolve)
        // Attachment 1: resolve color (stores final resolved result)
        // Attachment 2: MSAA depth (optional)
        VkAttachmentDescription attachments[3]{};

        // MSAA color
        attachments[0].format = format;
        attachments[0].samples = msaaSamples;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE; // resolved, don't need MSAA data
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        // Resolve color
        attachments[1].format = format;
        attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        // MSAA depth
        attachments[2].format = VK_FORMAT_D32_SFLOAT;
        attachments[2].samples = msaaSamples;
        attachments[2].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[2].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[2].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference resolveRef{1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depthRef{2, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;
        subpass.pResolveAttachments = &resolveRef;
        if (withDepth) subpass.pDepthStencilAttachment = &depthRef;

        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        if (withDepth) {
            dep.dstStageMask |= VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dep.dstAccessMask |= VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        }

        VkRenderPassCreateInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpInfo.attachmentCount = withDepth ? 3u : 2u;
        rpInfo.pAttachments = attachments;
        rpInfo.subpassCount = 1;
        rpInfo.pSubpasses = &subpass;
        rpInfo.dependencyCount = 1;
        rpInfo.pDependencies = &dep;

        if (vkCreateRenderPass(device, &rpInfo, nullptr, &renderPass_) != VK_SUCCESS) {
            LOG_ERROR("VkRenderTarget: failed to create MSAA render pass");
            destroy(device, allocator);
            return false;
        }

        // Create framebuffer: MSAA color + resolve + optional MSAA depth
        VkImageView fbAttachments[3] = { colorImage_.imageView, resolveImage_.imageView, depthImage_.imageView };
        VkFramebufferCreateInfo fbInfo{};
        fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbInfo.renderPass = renderPass_;
        fbInfo.attachmentCount = withDepth ? 3u : 2u;
        fbInfo.pAttachments = fbAttachments;
        fbInfo.width = width;
        fbInfo.height = height;
        fbInfo.layers = 1;

        if (vkCreateFramebuffer(device, &fbInfo, nullptr, &framebuffer_) != VK_SUCCESS) {
            LOG_ERROR("VkRenderTarget: failed to create MSAA framebuffer");
            destroy(device, allocator);
            return false;
        }
    } else {
        // Non-MSAA render pass (original path)
        VkAttachmentDescription attachments[2]{};
        attachments[0].format = format;
        attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        attachments[1].format = VK_FORMAT_D32_SFLOAT;
        attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;
        if (withDepth) subpass.pDepthStencilAttachment = &depthRef;

        VkSubpassDependency dependencies[2]{};
        uint32_t depCount = 1;
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        if (withDepth) {
            dependencies[0].dstStageMask |= VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependencies[0].dstAccessMask |= VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            dependencies[1].srcSubpass = 0;
            dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
            dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                                            VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
            dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                                             VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            depCount = 2;
        }

        VkRenderPassCreateInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpInfo.attachmentCount = withDepth ? 2u : 1u;
        rpInfo.pAttachments = attachments;
        rpInfo.subpassCount = 1;
        rpInfo.pSubpasses = &subpass;
        rpInfo.dependencyCount = depCount;
        rpInfo.pDependencies = dependencies;

        if (vkCreateRenderPass(device, &rpInfo, nullptr, &renderPass_) != VK_SUCCESS) {
            LOG_ERROR("VkRenderTarget: failed to create render pass");
            destroy(device, allocator);
            return false;
        }

        VkImageView fbAttachments[2] = { colorImage_.imageView, depthImage_.imageView };
        VkFramebufferCreateInfo fbInfo{};
        fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbInfo.renderPass = renderPass_;
        fbInfo.attachmentCount = withDepth ? 2u : 1u;
        fbInfo.pAttachments = fbAttachments;
        fbInfo.width = width;
        fbInfo.height = height;
        fbInfo.layers = 1;

        if (vkCreateFramebuffer(device, &fbInfo, nullptr, &framebuffer_) != VK_SUCCESS) {
            LOG_ERROR("VkRenderTarget: failed to create framebuffer");
            destroy(device, allocator);
            return false;
        }
    }

    LOG_INFO("VkRenderTarget created (", width, "x", height,
             withDepth ? ", depth" : "",
             useMSAA ? ", MSAAx" : "", useMSAA ? std::to_string(msaaSamples) : "", ")");
    return true;
}

void VkRenderTarget::destroy(VkDevice device, VmaAllocator allocator) {
    if (framebuffer_) {
        vkDestroyFramebuffer(device, framebuffer_, nullptr);
        framebuffer_ = VK_NULL_HANDLE;
    }
    if (renderPass_) {
        vkDestroyRenderPass(device, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (sampler_) {
        vkDestroySampler(device, sampler_, nullptr);
        sampler_ = VK_NULL_HANDLE;
    }
    destroyImage(device, allocator, resolveImage_);
    destroyImage(device, allocator, depthImage_);
    destroyImage(device, allocator, colorImage_);
    hasDepth_ = false;
    msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
}

void VkRenderTarget::beginPass(VkCommandBuffer cmd, const VkClearColorValue& clear) {
    VkRenderPassBeginInfo rpBegin{};
    rpBegin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBegin.renderPass = renderPass_;
    rpBegin.framebuffer = framebuffer_;
    rpBegin.renderArea.offset = {0, 0};
    rpBegin.renderArea.extent = getExtent();

    VkClearValue clearValues[3]{};
    clearValues[0].color = clear;           // MSAA color (or single-sample color)
    clearValues[1].color = clear;           // resolve (only used for MSAA)
    clearValues[2].depthStencil = {1.0f, 0}; // depth

    bool useMSAA = msaaSamples_ != VK_SAMPLE_COUNT_1_BIT;
    if (useMSAA) {
        rpBegin.clearValueCount = hasDepth_ ? 3u : 2u;
    } else {
        clearValues[1].depthStencil = {1.0f, 0}; // depth is attachment 1 in non-MSAA
        rpBegin.clearValueCount = hasDepth_ ? 2u : 1u;
    }
    rpBegin.pClearValues = clearValues;

    vkCmdBeginRenderPass(cmd, &rpBegin, VK_SUBPASS_CONTENTS_INLINE);

    // Set viewport and scissor to match render target
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(colorImage_.extent.width);
    viewport.height = static_cast<float>(colorImage_.extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = getExtent();
    vkCmdSetScissor(cmd, 0, 1, &scissor);
}

void VkRenderTarget::endPass(VkCommandBuffer cmd) {
    vkCmdEndRenderPass(cmd);
    // Image is now in SHADER_READ_ONLY_OPTIMAL (from render pass finalLayout)
}

VkDescriptorImageInfo VkRenderTarget::descriptorInfo() const {
    VkDescriptorImageInfo info{};
    info.sampler = sampler_;
    // Always return the resolved (single-sample) image for shader reads
    info.imageView = resolveImage_.imageView ? resolveImage_.imageView : colorImage_.imageView;
    info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    return info;
}

} // namespace rendering
} // namespace wowee
