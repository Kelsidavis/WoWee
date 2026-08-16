#include "rendering/hiz_system.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "core/logger.hpp"
#include "core/profiler.hpp"
#include <cmath>
#include <algorithm>

namespace wowee {
namespace rendering {

HiZSystem::~HiZSystem() {
    shutdown();
}

bool HiZSystem::initialize(VkContext* ctx, uint32_t width, uint32_t height) {
    if (!ctx || width == 0 || height == 0) return false;
    ctx_ = ctx;
    fullWidth_ = width;
    fullHeight_ = height;

    // Pyramid mip 0 is half the full resolution (the first downscale)
    pyramidWidth_ = std::max(1u, width / 2);
    pyramidHeight_ = std::max(1u, height / 2);
    mipLevels_ = static_cast<uint32_t>(std::floor(std::log2(std::max(pyramidWidth_, pyramidHeight_)))) + 1;

    if (!createComputePipeline()) return false;
    if (!createPyramidImage()) { destroyComputePipeline(); return false; }
    if (!createDescriptors()) { destroyPyramidImage(); destroyComputePipeline(); return false; }

    ready_ = true;
    LOG_INFO("HiZSystem: initialized ", pyramidWidth_, "x", pyramidHeight_,
             " pyramid (", mipLevels_, " mips) from ", width, "x", height, " depth");
    return true;
}

void HiZSystem::shutdown() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();
    vkDeviceWaitIdle(device);

    destroyDescriptors();
    destroyPyramidImage();
    destroyComputePipeline();

    ctx_ = nullptr;
    ready_ = false;
}

bool HiZSystem::resize(uint32_t width, uint32_t height) {
    if (!ctx_) return false;
    VkDevice device = ctx_->getDevice();
    vkDeviceWaitIdle(device);

    destroyDescriptors();
    destroyPyramidImage();

    fullWidth_ = width;
    fullHeight_ = height;
    pyramidWidth_ = std::max(1u, width / 2);
    pyramidHeight_ = std::max(1u, height / 2);
    mipLevels_ = static_cast<uint32_t>(std::floor(std::log2(std::max(pyramidWidth_, pyramidHeight_)))) + 1;

    if (!createPyramidImage()) return false;
    if (!createDescriptors()) { destroyPyramidImage(); return false; }

    ready_ = true;
    LOG_INFO("HiZSystem: resized to ", pyramidWidth_, "x", pyramidHeight_,
             " (", mipLevels_, " mips)");
    return true;
}

// --- Pyramid image creation ---

bool HiZSystem::createPyramidImage() {
    VkDevice device = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();

    for (uint32_t f = 0; f < MAX_FRAMES; f++) {
        // Create R32F image with full mip chain
        VkImageCreateInfo imgCi{.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        imgCi.imageType = VK_IMAGE_TYPE_2D;
        imgCi.format = VK_FORMAT_R32_SFLOAT;
        imgCi.extent = {.width = pyramidWidth_, .height = pyramidHeight_, .depth = 1};
        imgCi.mipLevels = mipLevels_;
        imgCi.arrayLayers = 1;
        imgCi.samples = VK_SAMPLE_COUNT_1_BIT;
        imgCi.tiling = VK_IMAGE_TILING_OPTIMAL;
        imgCi.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
        imgCi.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo allocCi{};
        allocCi.usage = VMA_MEMORY_USAGE_GPU_ONLY;

        if (vmaCreateImage(alloc, &imgCi, &allocCi, &pyramidImage_[f], &pyramidAlloc_[f], nullptr) != VK_SUCCESS) {
            LOG_ERROR("HiZSystem: failed to create pyramid image for frame ", f);
            return false;
        }

        // View of ALL mip levels (for sampling in the cull shader)
        VkImageViewCreateInfo viewCi{.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewCi.image = pyramidImage_[f];
        viewCi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewCi.format = VK_FORMAT_R32_SFLOAT;
        viewCi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewCi.subresourceRange.baseMipLevel = 0;
        viewCi.subresourceRange.levelCount = mipLevels_;
        viewCi.subresourceRange.layerCount = 1;

        if (vkCreateImageView(device, &viewCi, nullptr, &pyramidViewAll_[f]) != VK_SUCCESS) {
            LOG_ERROR("HiZSystem: failed to create pyramid view-all for frame ", f);
            return false;
        }

        // Per-mip views (for storage image writes in the build shader)
        pyramidMipViews_[f].resize(mipLevels_, VK_NULL_HANDLE);
        for (uint32_t mip = 0; mip < mipLevels_; mip++) {
            VkImageViewCreateInfo mipViewCi{.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            mipViewCi.image = pyramidImage_[f];
            mipViewCi.viewType = VK_IMAGE_VIEW_TYPE_2D;
            mipViewCi.format = VK_FORMAT_R32_SFLOAT;
            mipViewCi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            mipViewCi.subresourceRange.baseMipLevel = mip;
            mipViewCi.subresourceRange.levelCount = 1;
            mipViewCi.subresourceRange.layerCount = 1;

            if (vkCreateImageView(device, &mipViewCi, nullptr, &pyramidMipViews_[f][mip]) != VK_SUCCESS) {
                LOG_ERROR("HiZSystem: failed to create mip ", mip, " view for frame ", f);
                return false;
            }
        }
    }

    // Sampler for depth reads and HiZ pyramid reads (nearest, clamp)
    VkSamplerCreateInfo samplerCi{.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerCi.magFilter = VK_FILTER_NEAREST;
    samplerCi.minFilter = VK_FILTER_NEAREST;
    samplerCi.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerCi.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCi.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCi.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCi.maxLod = static_cast<float>(mipLevels_);

    if (vkCreateSampler(device, &samplerCi, nullptr, &depthSampler_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create sampler");
        return false;
    }

    return true;
}

void HiZSystem::destroyPyramidImage() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();

    destroy(device, depthSampler_);

    for (uint32_t f = 0; f < MAX_FRAMES; f++) {
        for (auto& view : pyramidMipViews_[f]) {
            if (view) { vkDestroyImageView(device, view, nullptr); view = VK_NULL_HANDLE; }
        }
        pyramidMipViews_[f].clear();

        if (pyramidViewAll_[f]) { vkDestroyImageView(device, pyramidViewAll_[f], nullptr); pyramidViewAll_[f] = VK_NULL_HANDLE; }
        if (depthSamplerView_[f]) { vkDestroyImageView(device, depthSamplerView_[f], nullptr); depthSamplerView_[f] = VK_NULL_HANDLE; }
        if (pyramidImage_[f]) { vmaDestroyImage(alloc, pyramidImage_[f], pyramidAlloc_[f]); pyramidImage_[f] = VK_NULL_HANDLE; }
    }
}

// --- Compute pipeline ---

bool HiZSystem::createComputePipeline() {
    VkDevice device = ctx_->getDevice();

    // Build descriptor set layout for pyramid build (set 0):
    //   binding 0: combined image sampler (source depth / previous mip)
    //   binding 1: storage image (destination mip)
    VkDescriptorSetLayoutBinding bindings[2] = {};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo layoutCi{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutCi.bindingCount = 2;
    layoutCi.pBindings = bindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCi, nullptr, &buildSetLayout_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create build set layout");
        return false;
    }

    // HiZ sampling layout (for M2 cull shader, set 1):
    //   binding 0: combined image sampler (HiZ pyramid, all mips)
    VkDescriptorSetLayoutBinding hizBinding{};
    hizBinding.binding = 0;
    hizBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    hizBinding.descriptorCount = 1;
    hizBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo hizLayoutCi{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    hizLayoutCi.bindingCount = 1;
    hizLayoutCi.pBindings = &hizBinding;
    if (vkCreateDescriptorSetLayout(device, &hizLayoutCi, nullptr, &hizSetLayout_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create HiZ set layout");
        return false;
    }

    // Push constant range for build shader
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(HiZBuildPushConstants);

    VkPipelineLayoutCreateInfo plCi{.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plCi.setLayoutCount = 1;
    plCi.pSetLayouts = &buildSetLayout_;
    plCi.pushConstantRangeCount = 1;
    plCi.pPushConstantRanges = &pushRange;
    if (vkCreatePipelineLayout(device, &plCi, nullptr, &buildPipelineLayout_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create build pipeline layout");
        return false;
    }

    // Load and create compute pipeline
    VkShaderModule buildShader;
    if (!buildShader.loadFromFile(device, "assets/shaders/hiz_build.comp.spv")) {
        LOG_ERROR("HiZSystem: failed to load hiz_build.comp.spv");
        return false;
    }

    VkComputePipelineCreateInfo cpCi{.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    cpCi.stage = buildShader.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
    cpCi.layout = buildPipelineLayout_;
    if (vkCreateComputePipelines(device, ctx_->getPipelineCache(), 1, &cpCi, nullptr, &buildPipeline_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create build compute pipeline");
        buildShader.destroy();
        return false;
    }
    buildShader.destroy();

    return true;
}

void HiZSystem::destroyComputePipeline() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();

    destroy(device, buildPipeline_);
    destroy(device, buildPipelineLayout_);
    destroy(device, buildSetLayout_);
    destroy(device, hizSetLayout_);
}

// --- Descriptors ---

bool HiZSystem::createDescriptors() {
    VkDevice device = ctx_->getDevice();

    // Pool: per-frame × per-mip build sets + 2 HiZ sampling sets
    // Each build set needs 1 sampler + 1 storage image
    // Each HiZ sampling set needs 1 sampler
    const uint32_t totalBuildSets = MAX_FRAMES * mipLevels_;
    const uint32_t totalHizSets = MAX_FRAMES;
    const uint32_t totalSets = totalBuildSets + totalHizSets;

    VkDescriptorPoolSize poolSizes[2] = {};
    poolSizes[0] = {.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = totalBuildSets + totalHizSets};
    poolSizes[1] = {.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, .descriptorCount = totalBuildSets};

    VkDescriptorPoolCreateInfo poolCi{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolCi.maxSets = totalSets;
    poolCi.poolSizeCount = 2;
    poolCi.pPoolSizes = poolSizes;
    if (vkCreateDescriptorPool(device, &poolCi, nullptr, &buildDescPool_) != VK_SUCCESS) {
        LOG_ERROR("HiZSystem: failed to create descriptor pool");
        return false;
    }

    // We use the same pool for both build and HiZ sets - simpler cleanup
    hizDescPool_ = VK_NULL_HANDLE; // sharing buildDescPool_

    for (uint32_t f = 0; f < MAX_FRAMES; f++) {
        // Create a temporary depth image view for sampling the depth buffer.
        // This is SEPARATE from the VkContext's depth image view because we need
        // DEPTH aspect sampling which requires specific format view.
        {
            VkImage depthSrc = ctx_->getDepthCopySourceImage();
            VkImageViewCreateInfo viewCi{.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            viewCi.image = depthSrc;
            viewCi.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewCi.format = ctx_->getDepthFormat();
            viewCi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
            viewCi.subresourceRange.levelCount = 1;
            viewCi.subresourceRange.layerCount = 1;
            if (vkCreateImageView(device, &viewCi, nullptr, &depthSamplerView_[f]) != VK_SUCCESS) {
                LOG_ERROR("HiZSystem: failed to create depth sampler view for frame ", f);
                return false;
            }
        }

        // Allocate per-mip build descriptor sets
        buildDescSets_[f].resize(mipLevels_);
        for (uint32_t mip = 0; mip < mipLevels_; mip++) {
            VkDescriptorSetAllocateInfo allocInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            allocInfo.descriptorPool = buildDescPool_;
            allocInfo.descriptorSetCount = 1;
            allocInfo.pSetLayouts = &buildSetLayout_;
            if (vkAllocateDescriptorSets(device, &allocInfo, &buildDescSets_[f][mip]) != VK_SUCCESS) {
                LOG_ERROR("HiZSystem: failed to allocate build desc set frame=", f, " mip=", mip);
                return false;
            }

            // Write descriptors:
            // Binding 0 (sampler): mip 0 reads depth buffer, mip N reads pyramid mip N-1
            VkDescriptorImageInfo srcInfo{};
            srcInfo.sampler = depthSampler_;
            if (mip == 0) {
                srcInfo.imageView = depthSamplerView_[f];
                srcInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            } else {
                srcInfo.imageView = pyramidViewAll_[f]; // shader uses texelFetch with explicit mip
                srcInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
            }

            // Binding 1 (storage image): write to current mip
            VkDescriptorImageInfo dstInfo{};
            dstInfo.imageView = pyramidMipViews_[f][mip];
            dstInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

            VkWriteDescriptorSet writes[2] = {};
            writes[0] = {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[0].dstSet = buildDescSets_[f][mip];
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[0].pImageInfo = &srcInfo;

            writes[1] = {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[1].dstSet = buildDescSets_[f][mip];
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
            writes[1].pImageInfo = &dstInfo;

            vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
        }

        // Allocate HiZ sampling descriptor set (for M2 cull shader)
        {
            VkDescriptorSetAllocateInfo allocInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            allocInfo.descriptorPool = buildDescPool_;
            allocInfo.descriptorSetCount = 1;
            allocInfo.pSetLayouts = &hizSetLayout_;
            if (vkAllocateDescriptorSets(device, &allocInfo, &hizDescSet_[f]) != VK_SUCCESS) {
                LOG_ERROR("HiZSystem: failed to allocate HiZ sampling desc set for frame ", f);
                return false;
            }

            VkDescriptorImageInfo hizInfo{};
            hizInfo.sampler = depthSampler_;
            hizInfo.imageView = pyramidViewAll_[f];
            hizInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

            VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet = hizDescSet_[f];
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            write.pImageInfo = &hizInfo;
            vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }
    }

    return true;
}

void HiZSystem::destroyDescriptors() {
    if (!ctx_) return;
    VkDevice device = ctx_->getDevice();

    // All descriptor sets are freed when pool is destroyed
    destroy(device, buildDescPool_);
    // hizDescPool_ shares buildDescPool_, so nothing extra to destroy

    for (uint32_t f = 0; f < MAX_FRAMES; f++) {
        buildDescSets_[f].clear();
        hizDescSet_[f] = VK_NULL_HANDLE;
        if (depthSamplerView_[f]) { vkDestroyImageView(device, depthSamplerView_[f], nullptr); depthSamplerView_[f] = VK_NULL_HANDLE; }
    }
}

// --- Pyramid build dispatch ---

void HiZSystem::buildPyramid(VkCommandBuffer cmd, uint32_t frameIndex, VkImage depthImage) {
    ZoneScopedN("HiZSystem::buildPyramid");
    if (!ready_ || !buildPipeline_) return;

    // Transition depth image from DEPTH_STENCIL_ATTACHMENT to SHADER_READ_ONLY for sampling
    {
        VkImageMemoryBarrier barrier{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        barrier.oldLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = depthImage;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.layerCount = 1;

        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    // Transition entire pyramid to GENERAL layout for storage writes
    {
        VkImageMemoryBarrier barrier{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = pyramidImage_[frameIndex];
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = mipLevels_;
        barrier.subresourceRange.layerCount = 1;

        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, buildPipeline_);

    // Build each mip level sequentially
    uint32_t mipW = pyramidWidth_;
    uint32_t mipH = pyramidHeight_;

    for (uint32_t mip = 0; mip < mipLevels_; mip++) {
        // Bind descriptor set for this mip level
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
            buildPipelineLayout_, 0, 1, &buildDescSets_[frameIndex][mip], 0, nullptr);

        // Push constants: destination size + mip level
        HiZBuildPushConstants pc{};
        pc.dstWidth = static_cast<int32_t>(mipW);
        pc.dstHeight = static_cast<int32_t>(mipH);
        pc.mipLevel = static_cast<int32_t>(mip);
        vkCmdPushConstants(cmd, buildPipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(pc), &pc);

        // Dispatch compute
        uint32_t groupsX = (mipW + 7) / 8;
        uint32_t groupsY = (mipH + 7) / 8;
        vkCmdDispatch(cmd, groupsX, groupsY, 1);

        // Barrier between mip levels: ensure writes to mip N are visible before reads for mip N+1
        if (mip + 1 < mipLevels_) {
            VkImageMemoryBarrier mipBarrier{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
            mipBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            mipBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            mipBarrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
            mipBarrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
            mipBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            mipBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            mipBarrier.image = pyramidImage_[frameIndex];
            mipBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            mipBarrier.subresourceRange.baseMipLevel = mip;
            mipBarrier.subresourceRange.levelCount = 1;
            mipBarrier.subresourceRange.layerCount = 1;

            vkCmdPipelineBarrier(cmd,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0, 0, nullptr, 0, nullptr, 1, &mipBarrier);
        }

        // Next mip level dimensions
        mipW = std::max(1u, mipW / 2);
        mipH = std::max(1u, mipH / 2);
    }

    // Transition depth back to DEPTH_STENCIL_ATTACHMENT for next frame
    {
        VkImageMemoryBarrier barrier{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        barrier.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = depthImage;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.layerCount = 1;

        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
            0, 0, nullptr, 0, nullptr, 1, &barrier);
    }
}

} // namespace rendering
} // namespace wowee
