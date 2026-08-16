#include "core/coordinates.hpp"
#include "rendering/water_surface_grid.hpp"
#include "rendering/water_mask.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/water_mask.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <array>
#include <unordered_map>

namespace wowee {
namespace rendering {

// Matches set 1 binding 0 in water.frag.glsl
struct WaterMaterialUBO {
    glm::vec4 waterColor;
    float waterAlpha;
    float shimmerStrength;
    float alphaScale;
    float _pad;
};

// Push constants matching water.vert.glsl
struct WaterPushConstants {
    glm::mat4 model;
    float waveAmp;
    float waveFreq;
    float waveSpeed;
    float liquidBasicType; // 0=water, 1=ocean, 2=magma, 3=slime
    glm::vec2 screenSize;  // target size, for screen-space UVs
    glm::vec2 depthRange;  // camera near, far - the shader linearises SceneDepth with these
};



WaterRenderer::WaterRenderer() = default;

WaterRenderer::~WaterRenderer() {
    shutdown();
}

bool WaterRenderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    vkCtx = ctx;
    if (!vkCtx) return false;

    LOG_INFO("Initializing water renderer (Vulkan)");
    VkDevice device = vkCtx->getDevice();

    // --- Material descriptor set layout (set 1) ---
    // binding 0: WaterMaterial UBO
    VkDescriptorSetLayoutBinding matBinding{};
    matBinding.binding = 0;
    matBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    matBinding.descriptorCount = 1;
    matBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    materialSetLayout = createDescriptorSetLayout(device, { matBinding });
    if (!materialSetLayout) {
        LOG_ERROR("WaterRenderer: failed to create material set layout");
        return false;
    }

    // --- Descriptor pool ---
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = MAX_WATER_SETS;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = MAX_WATER_SETS;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &materialDescPool) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create descriptor pool");
        return false;
    }

    // --- Scene history + reflection descriptor set layout (set 2) ---
    VkDescriptorSetLayoutBinding sceneColorBinding{};
    sceneColorBinding.binding = 0;
    sceneColorBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sceneColorBinding.descriptorCount = 1;
    sceneColorBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding sceneDepthBinding{};
    sceneDepthBinding.binding = 1;
    sceneDepthBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sceneDepthBinding.descriptorCount = 1;
    sceneDepthBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding reflColorBinding{};
    reflColorBinding.binding = 2;
    reflColorBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    reflColorBinding.descriptorCount = 1;
    reflColorBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding reflUBOBinding{};
    reflUBOBinding.binding = 3;
    reflUBOBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    reflUBOBinding.descriptorCount = 1;
    reflUBOBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    sceneSetLayout = createDescriptorSetLayout(device,
        {sceneColorBinding, sceneDepthBinding, reflColorBinding, reflUBOBinding});
    if (!sceneSetLayout) {
        LOG_ERROR("WaterRenderer: failed to create scene set layout");
        return false;
    }

    // Pool needs 3 combined image samplers + 1 uniform buffer per frame
    std::array<VkDescriptorPoolSize, 2> scenePoolSizes{};
    scenePoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    scenePoolSizes[0].descriptorCount = 3 * SCENE_HISTORY_FRAMES;
    scenePoolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    scenePoolSizes[1].descriptorCount = SCENE_HISTORY_FRAMES;
    VkDescriptorPoolCreateInfo scenePoolInfo{};
    scenePoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    scenePoolInfo.maxSets = SCENE_HISTORY_FRAMES;
    scenePoolInfo.poolSizeCount = static_cast<uint32_t>(scenePoolSizes.size());
    scenePoolInfo.pPoolSizes = scenePoolSizes.data();
    if (vkCreateDescriptorPool(device, &scenePoolInfo, nullptr, &sceneDescPool) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene descriptor pool");
        return false;
    }

    // --- Pipeline layout ---
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(WaterPushConstants);

    std::vector<VkDescriptorSetLayout> setLayouts = { perFrameLayout, materialSetLayout, sceneSetLayout };
    pipelineLayout = createPipelineLayout(device, setLayouts, { pushRange });
    if (!pipelineLayout) {
        LOG_ERROR("WaterRenderer: failed to create pipeline layout");
        return false;
    }

    // Create reflection resources FIRST so reflectionUBO exists when
    // createSceneHistoryResources writes descriptor binding 3
    createReflectionResources();

    createSceneHistoryResources(refractionCaptureExtent(),
                                vkCtx->getSwapchainFormat(),
                                vkCtx->getDepthFormat());

    // --- Shaders ---
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/water.vert.spv")) {
        LOG_ERROR("WaterRenderer: failed to load vertex shader");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/water.frag.spv")) {
        LOG_ERROR("WaterRenderer: failed to load fragment shader");
        return false;
    }

    // --- Vertex input (interleaved: pos3 + normal3 + uv2 = 8 floats = 32 bytes) ---
    VkVertexInputBindingDescription vertBinding{};
    vertBinding.binding = 0;
    vertBinding.stride = 8 * sizeof(float);
    vertBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    // Water vertex shader only takes aPos(vec3) at loc 0 and aTexCoord(vec2) at loc 1
    // (normal is computed in shader from wave derivatives)
    std::vector<VkVertexInputAttributeDescription> vertAttribs = {
        { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },                     // aPos
        { 1, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float) },        // aTexCoord (skip normal)
    };

    VkRenderPass mainPass = vkCtx->getImGuiRenderPass();

    waterPipeline = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertBinding }, vertAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)  // depth test yes, write no
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(mainPass)
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx->getPipelineCache());

    vertShader.destroy();
    fragShader.destroy();

    if (!waterPipeline) {
        LOG_ERROR("WaterRenderer: failed to create pipeline");
        return false;
    }

    LOG_INFO("Water renderer initialized (Vulkan)");
    return true;
}

void WaterRenderer::recreatePipelines() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    destroyReflectionResources();
    createReflectionResources();
    createSceneHistoryResources(refractionCaptureExtent(),
                                vkCtx->getSwapchainFormat(),
                                vkCtx->getDepthFormat());

    // Destroy old pipeline (keep layout)
    destroy(device, waterPipeline);

    // Load shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/water.vert.spv")) {
        LOG_ERROR("WaterRenderer::recreatePipelines: failed to load vertex shader");
        return;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/water.frag.spv")) {
        LOG_ERROR("WaterRenderer::recreatePipelines: failed to load fragment shader");
        vertShader.destroy();
        return;
    }

    // Vertex input (same as initialize)
    VkVertexInputBindingDescription vertBinding{};
    vertBinding.binding = 0;
    vertBinding.stride = 8 * sizeof(float);
    vertBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> vertAttribs = {
        { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },
        { 1, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float) },
    };

    VkRenderPass mainPass = vkCtx->getImGuiRenderPass();

    waterPipeline = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertBinding }, vertAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(mainPass)
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx->getPipelineCache());

    vertShader.destroy();
    fragShader.destroy();

    if (!waterPipeline) {
        LOG_ERROR("WaterRenderer::recreatePipelines: failed to create pipeline");
    }
}

void WaterRenderer::setRefractionEnabled(bool enabled) {
    if (refractionEnabled == enabled) return;
    refractionEnabled = enabled;

    // When turning off, clear scene history images to black so the shader
    // detects "no data" and uses the non-refraction path.
    if (!enabled && vkCtx) {
        vkCtx->immediateSubmit([&](VkCommandBuffer cmd) {
            for (uint32_t f = 0; f < SCENE_HISTORY_FRAMES; f++) {
                auto& sh = sceneHistory[f];
                if (!sh.colorImage) continue;

                VkImageMemoryBarrier toTransfer{};
                toTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                toTransfer.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                toTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                toTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                toTransfer.image = sh.colorImage;
                toTransfer.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
                toTransfer.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
                toTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                     0, 0, nullptr, 0, nullptr, 1, &toTransfer);

                VkClearColorValue clearColor = {{0.0f, 0.0f, 0.0f, 0.0f}};
                VkImageSubresourceRange range = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
                vkCmdClearColorImage(cmd, sh.colorImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearColor, 1, &range);

                VkImageMemoryBarrier toRead = toTransfer;
                toRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                toRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                toRead.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                toRead.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                     0, 0, nullptr, 0, nullptr, 1, &toRead);
            }
        });
    }
}

void WaterRenderer::shutdown() {
    clear();

    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    vkDeviceWaitIdle(device);

    // clear() defers surface destruction, and those lambdas free descriptor sets
    // from the pools destroyed just below. Drain them here, while the pools are
    // still valid - otherwise they linger until some later subsystem's flush
    // runs them against a dead pool.
    vkCtx->flushDeferredCleanup();

    destroyWater1xResources();
    destroyReflectionResources();
    destroySceneHistoryResources();
    destroy(device, waterPipeline);
    destroy(device, pipelineLayout);
    destroy(device, sceneDescPool);
    destroy(device, sceneSetLayout);
    destroy(device, materialDescPool);
    destroy(device, materialSetLayout);

    vkCtx = nullptr;
}

VkDescriptorSet WaterRenderer::allocateMaterialSet() {
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = materialDescPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &materialSetLayout;

    VkDescriptorSet set = VK_NULL_HANDLE;
    if (vkAllocateDescriptorSets(vkCtx->getDevice(), &allocInfo, &set) != VK_SUCCESS) {
        return VK_NULL_HANDLE;
    }
    return set;
}

void WaterRenderer::destroySceneHistoryResources() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    for (auto& sh : sceneHistory) {
        if (sh.colorView) { vkDestroyImageView(device, sh.colorView, nullptr); sh.colorView = VK_NULL_HANDLE; }
        if (sh.depthView) { vkDestroyImageView(device, sh.depthView, nullptr); sh.depthView = VK_NULL_HANDLE; }
        if (sh.colorImage) { vmaDestroyImage(vkCtx->getAllocator(), sh.colorImage, sh.colorAlloc); sh.colorImage = VK_NULL_HANDLE; sh.colorAlloc = VK_NULL_HANDLE; }
        if (sh.depthImage) { vmaDestroyImage(vkCtx->getAllocator(), sh.depthImage, sh.depthAlloc); sh.depthImage = VK_NULL_HANDLE; sh.depthAlloc = VK_NULL_HANDLE; }
        sh.sceneSet = VK_NULL_HANDLE;
    }
    sceneColorSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
    sceneDepthSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
    sceneHistoryExtent = {.width = 0, .height = 0};
    sceneHistoryReady = false;
}

// Full resolution, deliberately.
//
// This was half, on the grounds that refraction is low-frequency - sampled
// through a rippling normal that displaces it by a few pixels anyway - so the
// difference would not show through moving water. That holds for a lake bed.
// It does not hold for the player's own character, which is what is under the
// surface most of the time this is being looked at, and which arrived through
// the refraction visibly aliased however much anti-aliasing was asked for: the
// copy it was sampled from had already thrown half the resolution away, and
// nothing downstream can put that back. It is the thing in the middle of the
// screen, so it gets the pixels.
VkExtent2D WaterRenderer::refractionCaptureExtent() const {
    VkExtent2D full = vkCtx ? vkCtx->getSwapchainExtent() : VkExtent2D{.width = 0, .height = 0};
    return { std::max(1u, full.width), std::max(1u, full.height) };
}

void WaterRenderer::createSceneHistoryResources(VkExtent2D extent, VkFormat colorFormat, VkFormat depthFormat) {
    if (!vkCtx || extent.width == 0 || extent.height == 0 || !sceneSetLayout || !sceneDescPool) return;
    VkDevice device = vkCtx->getDevice();

    destroySceneHistoryResources();
    vkResetDescriptorPool(device, sceneDescPool, 0);
    sceneHistoryExtent = extent;

    // Create shared samplers
    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sceneColorSampler = vkCtx->getOrCreateSampler(sampCI);
    if (sceneColorSampler == VK_NULL_HANDLE) {
        LOG_ERROR("WaterRenderer: failed to create scene color sampler");
        return;
    }
    sampCI.magFilter = VK_FILTER_NEAREST;
    sampCI.minFilter = VK_FILTER_NEAREST;
    sceneDepthSampler = vkCtx->getOrCreateSampler(sampCI);
    if (sceneDepthSampler == VK_NULL_HANDLE) {
        LOG_ERROR("WaterRenderer: failed to create scene depth sampler");
        return;
    }

    VkImageCreateInfo colorImgInfo{};
    colorImgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    colorImgInfo.imageType = VK_IMAGE_TYPE_2D;
    colorImgInfo.format = colorFormat;
    colorImgInfo.extent = {.width = extent.width, .height = extent.height, .depth = 1};
    colorImgInfo.mipLevels = 1;
    colorImgInfo.arrayLayers = 1;
    colorImgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    colorImgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    colorImgInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    colorImgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VkImageCreateInfo depthImgInfo = colorImgInfo;
    depthImgInfo.format = depthFormat;

    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    // Create per-frame images, views, and descriptor sets
    for (uint32_t f = 0; f < SCENE_HISTORY_FRAMES; f++) {
        auto& sh = sceneHistory[f];

        if (vmaCreateImage(vkCtx->getAllocator(), &colorImgInfo, &allocCI, &sh.colorImage, &sh.colorAlloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to create scene color history image [", f, "]");
            return;
        }
        if (vmaCreateImage(vkCtx->getAllocator(), &depthImgInfo, &allocCI, &sh.depthImage, &sh.depthAlloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to create scene depth history image [", f, "]");
            return;
        }

        VkImageViewCreateInfo colorViewInfo{};
        colorViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        colorViewInfo.image = sh.colorImage;
        colorViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        colorViewInfo.format = colorFormat;
        colorViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        colorViewInfo.subresourceRange.levelCount = 1;
        colorViewInfo.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device, &colorViewInfo, nullptr, &sh.colorView) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to create scene color history view [", f, "]");
            return;
        }

        VkImageViewCreateInfo depthViewInfo = colorViewInfo;
        depthViewInfo.image = sh.depthImage;
        depthViewInfo.format = depthFormat;
        depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        if (vkCreateImageView(device, &depthViewInfo, nullptr, &sh.depthView) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to create scene depth history view [", f, "]");
            return;
        }

        // Allocate descriptor set for this frame
        VkDescriptorSetAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        ai.descriptorPool = sceneDescPool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &sceneSetLayout;
        if (vkAllocateDescriptorSets(device, &ai, &sh.sceneSet) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to allocate scene descriptor set [", f, "]");
            sh.sceneSet = VK_NULL_HANDLE;
            return;
        }

        VkDescriptorImageInfo colorInfo{};
        colorInfo.sampler = sceneColorSampler;
        colorInfo.imageView = sh.colorView;
        colorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo depthInfo{};
        depthInfo.sampler = sceneDepthSampler;
        depthInfo.imageView = sh.depthView;
        depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo reflColorInfo{};
        reflColorInfo.sampler = sceneColorSampler;
        reflColorInfo.imageView = reflectionColorView ? reflectionColorView : sh.colorView;
        reflColorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorBufferInfo reflUBOInfo{};
        reflUBOInfo.buffer = reflectionUBO;
        reflUBOInfo.offset = 0;
        reflUBOInfo.range = sizeof(WaterFrameUBOData);

        std::vector<VkWriteDescriptorSet> writes;

        VkWriteDescriptorSet w0{};
        w0.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w0.dstSet = sh.sceneSet;
        w0.dstBinding = 0;
        w0.descriptorCount = 1;
        w0.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w0.pImageInfo = &colorInfo;
        writes.push_back(w0);

        VkWriteDescriptorSet w1{};
        w1.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w1.dstSet = sh.sceneSet;
        w1.dstBinding = 1;
        w1.descriptorCount = 1;
        w1.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w1.pImageInfo = &depthInfo;
        writes.push_back(w1);

        VkWriteDescriptorSet w2{};
        w2.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w2.dstSet = sh.sceneSet;
        w2.dstBinding = 2;
        w2.descriptorCount = 1;
        w2.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w2.pImageInfo = &reflColorInfo;
        writes.push_back(w2);

        if (reflectionUBO) {
            VkWriteDescriptorSet w3{};
            w3.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w3.dstSet = sh.sceneSet;
            w3.dstBinding = 3;
            w3.descriptorCount = 1;
            w3.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w3.pBufferInfo = &reflUBOInfo;
            writes.push_back(w3);
        }

        vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }

    // Initialize all per-frame history images to shader-read layout
    vkCtx->immediateSubmit([&](VkCommandBuffer cmd) {
        std::vector<VkImageMemoryBarrier> barriers;
        for (uint32_t f = 0; f < SCENE_HISTORY_FRAMES; f++) {
            VkImageMemoryBarrier b{};
            b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.image = sceneHistory[f].colorImage;
            b.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
            b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            barriers.push_back(b);

            b.image = sceneHistory[f].depthImage;
            b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
            barriers.push_back(b);
        }
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                             0, 0, nullptr, 0, nullptr, static_cast<uint32_t>(barriers.size()), barriers.data());
    });
}

void WaterRenderer::updateMaterialUBO(WaterSurface& surface) {
    glm::vec4 color = getLiquidColor(surface.liquidType);
    float alpha = getLiquidAlpha(surface.liquidType);

    // WMO liquid material override
    if (surface.wmoId != 0) {
        const uint8_t basicType = (surface.liquidType == 0) ? 0 : ((surface.liquidType - 1) % 4);
        if (basicType == 2) {
            // Magma - bright orange-red, opaque
            color = glm::vec4(1.0f, 0.35f, 0.05f, 1.0f);
            alpha = 0.95f;
        } else if (basicType == 3) {
            // Slime - green, semi-opaque
            color = glm::vec4(0.2f, 0.6f, 0.1f, 1.0f);
            alpha = 0.85f;
        }
    }

    bool canalProfile = (surface.wmoId != 0) || (surface.liquidType == 5);
    float shimmerStrength = canalProfile ? 0.95f : 0.50f;
    float alphaScale = canalProfile ? 0.90f : 1.00f;

    WaterMaterialUBO mat{};
    mat.waterColor = color;
    mat.waterAlpha = alpha;
    mat.shimmerStrength = shimmerStrength;
    mat.alphaScale = alphaScale;

    // Create UBO
    VkBufferCreateInfo bufCI{};
    bufCI.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCI.size = sizeof(WaterMaterialUBO);
    bufCI.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VmaAllocationInfo mapInfo{};
    vmaCreateBuffer(vkCtx->getAllocator(), &bufCI, &allocCI,
                    &surface.materialUBO, &surface.materialAlloc, &mapInfo);
    if (mapInfo.pMappedData) {
        std::memcpy(mapInfo.pMappedData, &mat, sizeof(mat));
    }

    // Allocate and write descriptor set
    surface.materialSet = allocateMaterialSet();
    if (surface.materialSet) {
        VkDescriptorBufferInfo bufInfo{};
        bufInfo.buffer = surface.materialUBO;
        bufInfo.offset = 0;
        bufInfo.range = sizeof(WaterMaterialUBO);

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = surface.materialSet;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.pBufferInfo = &bufInfo;

        vkUpdateDescriptorSets(vkCtx->getDevice(), 1, &write, 0, nullptr);
    } else {
        LOG_WARNING("Water: failed to allocate material descriptor set (pool exhaustion?)");
    }
}

// ==============================================================
// Data loading (preserved from GL version - no GL calls)
// ==============================================================

void WaterRenderer::loadFromTerrain(const pipeline::ADTTerrain& terrain, bool append,
                                     int tileX, int tileY) {
    constexpr float TILE_SIZE = 33.33333f / 8.0f;

    if (!append) {
        clear();
    }

    // ── Pass 1: collect layers into merge groups keyed by {liquidType, roundedHeight} ──
    struct ChunkLayerInfo {
        int chunkX, chunkY;
        const pipeline::ADTTerrain::WaterLayer* layer;
    };

    struct MergeKey {
        uint16_t liquidType;
        int32_t roundedHeight;  // minHeight * 2, rounded to int
        bool operator==(const MergeKey& o) const {
            return liquidType == o.liquidType && roundedHeight == o.roundedHeight;
        }
    };

    struct MergeKeyHash {
        size_t operator()(const MergeKey& k) const {
            return std::hash<uint64_t>()((uint64_t(k.liquidType) << 32) | uint32_t(k.roundedHeight));
        }
    };

    std::unordered_map<MergeKey, std::vector<ChunkLayerInfo>, MergeKeyHash> mergeGroups;

    for (int chunkIdx = 0; chunkIdx < 256; chunkIdx++) {
        const auto& chunkWater = terrain.waterData[chunkIdx];
        if (!chunkWater.hasWater()) continue;

        int chunkX = chunkIdx % 16;
        int chunkY = chunkIdx / 16;

        for (const auto& layer : chunkWater.layers) {
            MergeKey key;
            key.liquidType = layer.liquidType;
            key.roundedHeight = static_cast<int32_t>(std::round(layer.minHeight * 2.0f));
            mergeGroups[key].push_back({chunkX, chunkY, &layer});
        }
    }

    // Tile origin = NW corner = chunk(0,0) position
    const auto& chunk00 = terrain.getChunk(0, 0);

    // Stormwind water lowering check
    bool isStormwindArea = (tileX >= 28 && tileX <= 50 && tileY >= 28 && tileY <= 52);
    float tileWorldX = 0, tileWorldY = 0;
    glm::vec2 moonwellPos2D(0.0f);
    if (isStormwindArea) {
        tileWorldX = (32.0f - tileX) * core::coords::TILE_SIZE;
        tileWorldY = (32.0f - tileY) * core::coords::TILE_SIZE;
        moonwellPos2D = glm::vec2(-8755.9f, 1108.9f);
    }

    int totalSurfaces = 0;

    // Merge threshold: groups with more than this many chunks get merged into
    // one tile-wide surface.  Small groups (shore, lakes) stay per-chunk so
    // their original mask / height data is preserved exactly.
    constexpr size_t MERGE_THRESHOLD = 4;

    // ── Pass 2: create surfaces ──
    for (auto& [key, chunkLayers] : mergeGroups) {

        // ── Small group → per-chunk surfaces (original code path) ──
        if (chunkLayers.size() <= MERGE_THRESHOLD) {
            for (const auto& info : chunkLayers) {
                const auto& layer = *info.layer;
                const auto& terrainChunk = terrain.getChunk(info.chunkX, info.chunkY);

                WaterSurface surface;
                surface.position = glm::vec3(
                    terrainChunk.position[0],
                    terrainChunk.position[1],
                    layer.minHeight
                );
                surface.origin = glm::vec3(
                    // Same axis convention as terrain_mesh.cpp vertices:
                    // MH2O rows (y) advance along render X, columns (x)
                    // along render Y.
                    surface.position.x - (static_cast<float>(layer.y) * TILE_SIZE),
                    surface.position.y - (static_cast<float>(layer.x) * TILE_SIZE),
                    layer.minHeight
                );
                // stepX walks the mesh grid's x (MH2O column → render Y),
                // stepY walks the grid's y (MH2O row → render X).
                surface.stepX = glm::vec3(0.0f, -TILE_SIZE, 0.0f);
                surface.stepY = glm::vec3(-TILE_SIZE, 0.0f, 0.0f);

                surface.minHeight = layer.minHeight;
                surface.maxHeight = layer.maxHeight;
                surface.liquidType = layer.liquidType;
                // Keep mask coordinates in the MH2O chunk's native x/y order;
                // only the world-space origin needs the terrain axis conversion.
                surface.xOffset = layer.x;
                surface.yOffset = layer.y;
                surface.width = layer.width;
                surface.height = layer.height;

                size_t numVertices = (layer.width + 1) * (layer.height + 1);
                bool useFlat = true;
                if (layer.heights.size() == numVertices) {
                    bool sane = true;
                    for (float h : layer.heights) {
                        if (!std::isfinite(h) || std::abs(h) > 50000.0f) { sane = false; break; }
                        if (h < layer.minHeight - 8.0f || h > layer.maxHeight + 8.0f) { sane = false; break; }
                    }
                    if (sane) { useFlat = false; surface.heights = layer.heights; }
                }
                if (useFlat) surface.heights.resize(numVertices, layer.minHeight);

                if (isStormwindArea && layer.minHeight > 94.0f) {
                    float distToMoonwell = glm::distance(glm::vec2(tileWorldX, tileWorldY), moonwellPos2D);
                    if (distToMoonwell > 300.0f) {
                        for (float& h : surface.heights) h -= 1.0f;
                        surface.minHeight -= 1.0f;
                        surface.maxHeight -= 1.0f;
                    }
                }

                surface.mask = layer.mask;
                surface.tileX = tileX;
                surface.tileY = tileY;

                createWaterMesh(surface);
                if (surface.indexCount > 0 && vkCtx) {
                    updateMaterialUBO(surface);
                }
                surfaces.push_back(std::move(surface));
                totalSurfaces++;
            }
            continue;
        }

        // ── Large group → merged tile-wide surface ──
        WaterSurface surface;

        float groupHeight = key.roundedHeight / 2.0f;

        surface.width = 128;
        surface.height = 128;
        surface.xOffset = 0;
        surface.yOffset = 0;
        surface.liquidType = key.liquidType;
        surface.tileX = tileX;
        surface.tileY = tileY;

        // Origin = chunk(0,0) position (NW corner of tile)
        surface.origin = glm::vec3(chunk00.position[0], chunk00.position[1], groupHeight);
        surface.position = surface.origin;
        surface.stepX = glm::vec3(-TILE_SIZE, 0.0f, 0.0f);
        surface.stepY = glm::vec3(0.0f, -TILE_SIZE, 0.0f);

        surface.minHeight = groupHeight;
        surface.maxHeight = groupHeight;

        // Initialize height grid (129×129) with group height
        constexpr int MERGED_W = 128;
        const int gridW = MERGED_W + 1;  // 129
        const int gridH = MERGED_W + 1;
        surface.heights.resize(gridW * gridH, groupHeight);

        // Initialize mask (128×128 sub-tiles)
        // Mask uses LSB bit order: tileIndex = row * 128 + col
        const int maskBytes = (MERGED_W * MERGED_W + 7) / 8;
        // Ocean water (basicType 1) at sea level fills each chunk that declares
        // it, whatever its own bitmap says - an ocean layer's sub-rect is often
        // sparse and the gaps read as holes in open sea.
        //
        // Each chunk that declares it, not the whole tile. This used to seed
        // the entire 128×128 mask, on the reading that depth testing against
        // the terrain would hide the water wherever there was land. That holds
        // only for land *above* sea level. Somewhere the ground drops below it
        // the water is drawn over the top with nothing to occlude it, and a
        // chunk that declares no liquid at all got a surface anyway: the
        // Caverns of Time, a hole in a coastal tile in Tanaris, was roofed with
        // ocean. Any cave below sea level on an ocean tile had the same.
        uint8_t basicType = (key.liquidType == 0) ? 0 : ((key.liquidType - 1) % 4);
        bool isOcean = (basicType == 1) && (std::abs(groupHeight) < 1.0f);
        surface.mask.resize(maskBytes, 0x00);

        // ── Fill from each contributing chunk ──
        for (const auto& info : chunkLayers) {
            const auto& layer = *info.layer;

            // Merged grid offset for this chunk.
            // Grid x is the row axis (render X): chunk row (idx/16) and MH2O y.
            // Grid y is the column axis (render Y): chunk col (idx%16) and MH2O x.
            int baseGx = info.chunkY * 8;
            int baseGy = info.chunkX * 8;

            // Copy heights
            int layerGridW = layer.width + 1;
            size_t numVertices = static_cast<size_t>(layerGridW) * (layer.height + 1);
            bool useFlat = true;
            if (layer.heights.size() == numVertices) {
                bool sane = true;
                for (float h : layer.heights) {
                    if (!std::isfinite(h) || std::abs(h) > 50000.0f) { sane = false; break; }
                    if (h < layer.minHeight - 8.0f || h > layer.maxHeight + 8.0f) { sane = false; break; }
                }
                if (sane) useFlat = false;
            }

            for (int ly = 0; ly <= layer.height; ly++) {
                for (int lx = 0; lx <= layer.width; lx++) {
                    int mgx = baseGx + layer.y + ly;
                    int mgy = baseGy + layer.x + lx;
                    if (mgx >= gridW || mgy >= gridH) continue;

                    float h;
                    if (!useFlat) {
                        int layerIdx = ly * layerGridW + lx;
                        h = layer.heights[layerIdx];
                    } else {
                        h = layer.minHeight;
                    }

                    surface.heights[mgy * gridW + mgx] = h;
                    if (h < surface.minHeight) surface.minHeight = h;
                    if (h > surface.maxHeight) surface.maxHeight = h;
                }
            }

            // Which of this chunk's sixty-four sub-tiles are drawn. The rule
            // lives in water_mask.hpp so it can be checked without a device;
            // this places the answer, in the chunk-row-to-render-X order the
            // merged grid uses.
            const uint64_t chunkMask = chunkWaterMask(
                layer.mask, layer.x, layer.y, layer.width, layer.height, isOcean);
            for (int row = 0; row < 8; row++) {
                for (int col = 0; col < 8; col++) {
                    if (!(chunkMask & (uint64_t{1} << (row * 8 + col)))) continue;
                    const int mx = baseGx + row;
                    const int my = baseGy + col;
                    if (mx >= MERGED_W || my >= MERGED_W) continue;
                    const int mergedTileIdx = my * MERGED_W + mx;
                    surface.mask[mergedTileIdx / 8] |=
                        static_cast<uint8_t>(1 << (mergedTileIdx % 8));
                }
            }
        }

        // Stormwind water lowering
        if (isStormwindArea && surface.minHeight > 94.0f) {
            float distToMoonwell = glm::distance(glm::vec2(tileWorldX, tileWorldY), moonwellPos2D);
            if (distToMoonwell > 300.0f) {
                for (float& h : surface.heights) h -= 1.0f;
                surface.minHeight -= 1.0f;
                surface.maxHeight -= 1.0f;
            }
        }

        createWaterMesh(surface);
        if (surface.indexCount > 0 && vkCtx) {
            updateMaterialUBO(surface);
        }
        surfaces.push_back(std::move(surface));
        totalSurfaces++;
    }

    if (totalSurfaces > 0) {
        LOG_INFO("Water: Loaded ", totalSurfaces, " surfaces from tile [", tileX, ",", tileY,
                 "] (", mergeGroups.size(), " groups), total surfaces: ", surfaces.size());
    }
}

void WaterRenderer::removeTile(int tileX, int tileY) {
    int removed = 0;
    auto it = surfaces.begin();
    while (it != surfaces.end()) {
        if (it->tileX == tileX && it->tileY == tileY) {
            destroyWaterMesh(*it);
            it = surfaces.erase(it);
            removed++;
        } else {
            ++it;
        }
    }
    if (removed > 0) {
        LOG_DEBUG("Water: Removed ", removed, " surfaces for tile [", tileX, ",", tileY, "], remaining: ", surfaces.size());
    }
}

void WaterRenderer::loadFromWMO([[maybe_unused]] const pipeline::WMOLiquid& liquid,
                                 [[maybe_unused]] const glm::mat4& modelMatrix,
                                 [[maybe_unused]] uint32_t wmoId) {
    if (!liquid.hasLiquid() || liquid.xTiles == 0 || liquid.yTiles == 0) return;
    if (liquid.xVerts < 2 || liquid.yVerts < 2) return;
    if (liquid.xTiles != liquid.xVerts - 1 || liquid.yTiles != liquid.yVerts - 1) return;
    if (liquid.xTiles > 64 || liquid.yTiles > 64) return;

    WaterSurface surface;
    surface.tileX = -1;
    surface.tileY = -1;
    surface.wmoId = wmoId;
    surface.liquidType = liquid.materialId;
    surface.xOffset = 0;
    surface.yOffset = 0;
    surface.width = static_cast<uint8_t>(std::min<uint32_t>(255, liquid.xTiles));
    surface.height = static_cast<uint8_t>(std::min<uint32_t>(255, liquid.yTiles));

    constexpr float WMO_LIQUID_TILE_SIZE = 4.1666625f;
    const glm::vec3 localBase(liquid.basePosition.x, liquid.basePosition.y, liquid.basePosition.z);
    const glm::vec3 localStepX(WMO_LIQUID_TILE_SIZE, 0.0f, 0.0f);
    const glm::vec3 localStepY(0.0f, WMO_LIQUID_TILE_SIZE, 0.0f);

    surface.origin = glm::vec3(modelMatrix * glm::vec4(localBase, 1.0f));
    surface.stepX = glm::vec3(modelMatrix * glm::vec4(localStepX, 0.0f));
    surface.stepY = glm::vec3(modelMatrix * glm::vec4(localStepY, 0.0f));
    surface.position = surface.origin;

    float stepXLen = glm::length(surface.stepX);
    float stepYLen = glm::length(surface.stepY);
    glm::vec3 planeN = glm::cross(surface.stepX, surface.stepY);
    float planeNLenSq = glm::dot(planeN, planeN);
    float nz = (planeNLenSq > 1e-8f) ? std::abs(planeN.z * glm::inversesqrt(planeNLenSq)) : 0.0f;
    float spanX = stepXLen * static_cast<float>(surface.width);
    float spanY = stepYLen * static_cast<float>(surface.height);
    if (stepXLen < 0.2f || stepXLen > 12.0f ||
        stepYLen < 0.2f || stepYLen > 12.0f ||
        nz < 0.60f || spanX > 450.0f || spanY > 450.0f) return;

    const int gridWidth = static_cast<int>(surface.width) + 1;
    const int gridHeight = static_cast<int>(surface.height) + 1;
    const int vertexCount = gridWidth * gridHeight;

    // WMO liquid base heights sit ~2 units above the visual waterline.
    constexpr float WMO_WATER_Z_OFFSET = -1.0f;
    float adjustedZ = surface.origin.z + WMO_WATER_Z_OFFSET;
    surface.heights.assign(vertexCount, adjustedZ);
    surface.minHeight = adjustedZ;
    surface.maxHeight = adjustedZ;
    surface.origin.z = adjustedZ;
    surface.position.z = adjustedZ;

    if (surface.origin.z > 2000.0f || surface.origin.z < -500.0f) return;

    // Build tile mask from MLIQ flags
    size_t tileCount = static_cast<size_t>(surface.width) * static_cast<size_t>(surface.height);
    size_t maskBytes = (tileCount + 7) / 8;
    surface.mask.assign(maskBytes, 0x00);
    for (size_t t = 0; t < tileCount; t++) {
        bool hasLiquid = true;
        int tx = static_cast<int>(t) % surface.width;
        int ty = static_cast<int>(t) / surface.width;

        // Standard WoW check: low nibble 0x0F = "don't render"
        if (t < liquid.flags.size()) {
            if ((liquid.flags[t] & 0x0F) == 0x0F) {
                hasLiquid = false;
            }
        }
        // Suppress water tiles that extend into enclosed WMO areas
        // (e.g. Stormwind barracks stairway where canal water pokes through)
        // Render coords: x=wowY(west), y=wowX(north)
        if (hasLiquid) {
            glm::vec3 tileWorld = surface.origin +
                surface.stepX * (static_cast<float>(tx) + 0.5f) +
                surface.stepY * (static_cast<float>(ty) + 0.5f);
            // Stormwind Barracks / Stockade stairway:
            // Stockade entrance at approximately render (-8768, 848)
            if (tileWorld.x > -8790.0f && tileWorld.x < -8735.0f &&
                tileWorld.y > 828.0f && tileWorld.y < 878.0f) {
                hasLiquid = false;
            }
        }
        if (hasLiquid) {
            size_t byteIdx = t / 8;
            size_t bitIdx = t % 8;
            surface.mask[byteIdx] |= (1 << bitIdx);
        }
    }

    createWaterMesh(surface);

    // Count how many tiles passed the flag check and compute bounds
    size_t activeTiles = 0;
    float minWX = 1e9f, maxWX = -1e9f, minWY = 1e9f, maxWY = -1e9f;
    for (size_t t = 0; t < tileCount; t++) {
        size_t byteIdx = t / 8;
        size_t bitIdx = t % 8;
        if (surface.mask[byteIdx] & (1 << bitIdx)) {
            activeTiles++;
            int atx = static_cast<int>(t) % surface.width;
            int aty = static_cast<int>(t) / surface.width;
            glm::vec3 tw = surface.origin +
                surface.stepX * (static_cast<float>(atx) + 0.5f) +
                surface.stepY * (static_cast<float>(aty) + 0.5f);
            if (tw.x < minWX) minWX = tw.x;
            if (tw.x > maxWX) maxWX = tw.x;
            if (tw.y < minWY) minWY = tw.y;
            if (tw.y > maxWY) maxWY = tw.y;
        }
    }
    LOG_DEBUG("WMO water: origin=(", surface.origin.x, ",", surface.origin.y, ",", surface.origin.z,
             ") tiles=", static_cast<int>(surface.width), "x", static_cast<int>(surface.height),
             " active=", activeTiles, "/", tileCount,
             " wmoId=", wmoId, " indexCount=", surface.indexCount,
             " bounds x=[", minWX, "..", maxWX, "] y=[", minWY, "..", maxWY, "]");

    if (surface.indexCount > 0) {
        if (vkCtx) updateMaterialUBO(surface);
        surfaces.push_back(std::move(surface));
    }
}

void WaterRenderer::removeWMO(uint32_t wmoId) {
    if (wmoId == 0) return;
    auto it = surfaces.begin();
    while (it != surfaces.end()) {
        if (it->wmoId == wmoId) {
            destroyWaterMesh(*it);
            it = surfaces.erase(it);
        } else {
            ++it;
        }
    }
}

void WaterRenderer::clear() {
    for (auto& surface : surfaces) {
        destroyWaterMesh(surface);
    }
    surfaces.clear();
}

// ==============================================================
// Rendering
// ==============================================================

void WaterRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet,
                            const Camera& camera, float /*time*/, bool use1x, uint32_t frameIndex) {
    VkPipeline pipeline = (use1x && water1xPipeline) ? water1xPipeline : waterPipeline;
    if (!renderingEnabled || surfaces.empty() || !pipeline) {
        if (renderDiagCounter_++ % 300 == 0 && !surfaces.empty()) {
            LOG_WARNING("Water: render skipped - enabled=", renderingEnabled,
                        " surfaces=", surfaces.size(),
                        " pipeline=", (pipeline ? "ok" : "null"),
                        " use1x=", use1x);
        }
        return;
    }
    uint32_t fi = frameIndex % SCENE_HISTORY_FRAMES;
    VkDescriptorSet activeSceneSet = sceneHistory[fi].sceneSet;
    if (!activeSceneSet) {
        if (renderDiagCounter_++ % 300 == 0) {
            LOG_WARNING("Water: render skipped - sceneSet is null, surfaces=", surfaces.size());
        }
        return;
    }

    // Frustum culling setup
    Frustum frustum;
    frustum.extractFromMatrix(camera.getViewProjectionMatrix());

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                             0, 1, &perFrameSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                             2, 1, &activeSceneSet, 0, nullptr);

    for (const auto& surface : surfaces) {
        if (surface.vertexBuffer == VK_NULL_HANDLE || surface.indexCount == 0) continue;
        if (!surface.materialSet) continue;

        // Frustum cull: compute AABB from surface origin + step vectors
        {
            const glm::vec3 extentX = surface.stepX * static_cast<float>(surface.width);
            const glm::vec3 extentY = surface.stepY * static_cast<float>(surface.height);
            const glm::vec3 c0 = surface.origin;
            const glm::vec3 c1 = surface.origin + extentX;
            const glm::vec3 c2 = surface.origin + extentY;
            const glm::vec3 c3 = surface.origin + extentX + extentY;
            const glm::vec3 aabbMin(
                std::min({c0.x, c1.x, c2.x, c3.x}),
                std::min({c0.y, c1.y, c2.y, c3.y}),
                surface.minHeight
            );
            const glm::vec3 aabbMax(
                std::max({c0.x, c1.x, c2.x, c3.x}),
                std::max({c0.y, c1.y, c2.y, c3.y}),
                surface.maxHeight
            );
            if (!frustum.intersectsAABB(aabbMin, aabbMax)) continue;
        }

        bool isWmoWater = (surface.wmoId != 0);
        bool canalProfile = isWmoWater || (surface.liquidType == 5);
        uint8_t basicType = (surface.liquidType == 0) ? 0 : ((surface.liquidType - 1) % 4);
        // WMO water gets no wave displacement - prevents visible slosh at
        // geometry edges (bridges, docks) where water is far below the surface.
        float waveAmp = isWmoWater ? 0.0f : (basicType == 1 ? 0.35f : 0.08f);
        float waveFreq = canalProfile ? 0.35f : (basicType == 1 ? 0.20f : 0.30f);
        float waveSpeed = canalProfile ? 1.00f : (basicType == 1 ? 1.20f : 1.40f);

        WaterPushConstants push{};
        push.model = glm::mat4(1.0f);
        push.waveAmp = waveAmp;
        push.waveFreq = waveFreq;
        push.waveSpeed = waveSpeed;
        push.liquidBasicType = static_cast<float>(basicType);
        push.screenSize = glm::vec2(static_cast<float>(renderExtent_.width),
                                    static_cast<float>(renderExtent_.height));
        // Asked of the camera rather than written out in the shader, which is
        // how the two came to disagree: the shader linearised the depth buffer
        // against a near plane of 0.05 while the camera's is 0.5.
        push.depthRange = glm::vec2(camera.getNearPlane(), camera.getFarPlane());

        vkCmdPushConstants(cmd, pipelineLayout,
                            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                            0, sizeof(WaterPushConstants), &push);

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                                 1, 1, &surface.materialSet, 0, nullptr);

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &surface.vertexBuffer, &offset);
        vkCmdBindIndexBuffer(cmd, surface.indexBuffer, 0, VK_INDEX_TYPE_UINT32);

        vkCmdDrawIndexed(cmd, static_cast<uint32_t>(surface.indexCount), 1, 0, 0, 0);
    }
}

void WaterRenderer::captureSceneHistory(VkCommandBuffer cmd,
                                        VkImage srcColorImage,
                                        VkImage srcDepthImage,
                                        VkExtent2D srcExtent,
                                        bool srcDepthIsMsaa,
                                        uint32_t frameIndex) {
    uint32_t fi = frameIndex % SCENE_HISTORY_FRAMES;
    auto& sh = sceneHistory[fi];
    if (!vkCtx || !cmd || !sh.colorImage || !sh.depthImage || srcExtent.width == 0 || srcExtent.height == 0) {
        return;
    }

    // The scene is not always rendered at the history's resolution - FSR renders
    // smaller and upscales later. Copying the smaller image into the corner of a
    // larger history would leave the refraction sampling a fraction of the frame
    // stretched over all of it, so scale with a blit whenever they differ and
    // keep the cheap straight copy when they match.
    const bool needsScaling = (srcExtent.width != sceneHistoryExtent.width ||
                               srcExtent.height != sceneHistoryExtent.height);
    VkExtent2D copyExtent{
        std::min(srcExtent.width, sceneHistoryExtent.width),
        std::min(srcExtent.height, sceneHistoryExtent.height)
    };
    if (copyExtent.width == 0 || copyExtent.height == 0) return;

    auto barrier2 = [&](VkImage image,
                        VkImageAspectFlags aspect,
                        VkImageLayout oldLayout,
                        VkImageLayout newLayout,
                        VkAccessFlags srcAccess,
                        VkAccessFlags dstAccess,
                        VkPipelineStageFlags srcStage,
                        VkPipelineStageFlags dstStage) {
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = oldLayout;
        b.newLayout = newLayout;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange.aspectMask = aspect;
        b.subresourceRange.baseMipLevel = 0;
        b.subresourceRange.levelCount = 1;
        b.subresourceRange.baseArrayLayer = 0;
        b.subresourceRange.layerCount = 1;
        b.srcAccessMask = srcAccess;
        b.dstAccessMask = dstAccess;
        vkCmdPipelineBarrier(cmd, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &b);
    };

    // Color source: final render pass layout is PRESENT_SRC.
    // srcAccessMask must be COLOR_ATTACHMENT_WRITE (not 0) so that GPU cache flushes
    // happen before the transfer read.  Using srcAccessMask=0 with BOTTOM_OF_PIPE
    // causes VK_ERROR_DEVICE_LOST on strict drivers (AMD/Mali) because color writes
    // are not made visible to the transfer unit before the copy begins.
    barrier2(srcColorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
             VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
             VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    barrier2(sh.colorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
             VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

    if (needsScaling) {
        VkImageBlit colorBlit{};
        colorBlit.srcSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        colorBlit.dstSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        colorBlit.srcOffsets[1] = {.x = static_cast<int32_t>(srcExtent.width),
                                   .y = static_cast<int32_t>(srcExtent.height), .z = 1};
        colorBlit.dstOffsets[1] = {.x = static_cast<int32_t>(sceneHistoryExtent.width),
                                   .y = static_cast<int32_t>(sceneHistoryExtent.height), .z = 1};
        vkCmdBlitImage(cmd, srcColorImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       sh.colorImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       1, &colorBlit, VK_FILTER_LINEAR);
    } else {
        VkImageCopy colorCopy{};
        colorCopy.srcSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        colorCopy.dstSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        colorCopy.extent = {.width = copyExtent.width, .height = copyExtent.height, .depth = 1};
        vkCmdCopyImage(cmd, srcColorImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       sh.colorImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &colorCopy);
    }

    barrier2(sh.colorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
             VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
             VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
    barrier2(srcColorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
             VK_ACCESS_TRANSFER_READ_BIT, 0,
             VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

    // Depth source: only copy when source is single-sampled.
    if (!srcDepthIsMsaa && srcDepthImage != VK_NULL_HANDLE) {
        barrier2(srcDepthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
                 VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                 VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                 VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
        barrier2(sh.depthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                 VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

        if (needsScaling) {
            // Depth must not be filtered - an interpolated depth is a surface
            // that exists nowhere.
            VkImageBlit depthBlit{};
            depthBlit.srcSubresource = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
            depthBlit.dstSubresource = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
            depthBlit.srcOffsets[1] = {.x = static_cast<int32_t>(srcExtent.width),
                                       .y = static_cast<int32_t>(srcExtent.height), .z = 1};
            depthBlit.dstOffsets[1] = {.x = static_cast<int32_t>(sceneHistoryExtent.width),
                                       .y = static_cast<int32_t>(sceneHistoryExtent.height), .z = 1};
            vkCmdBlitImage(cmd, srcDepthImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           sh.depthImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           1, &depthBlit, VK_FILTER_NEAREST);
        } else {
            VkImageCopy depthCopy{};
            depthCopy.srcSubresource = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
            depthCopy.dstSubresource = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
            depthCopy.extent = {.width = copyExtent.width, .height = copyExtent.height, .depth = 1};
            vkCmdCopyImage(cmd, srcDepthImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           sh.depthImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &depthCopy);
        }

        barrier2(sh.depthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                 VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                 VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        barrier2(srcDepthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
                 VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                 VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                 VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT);
    }

    sceneHistoryReady = true;
}

// ==============================================================
// Mesh creation (Vulkan upload instead of GL)
// ==============================================================

void WaterRenderer::createWaterMesh(WaterSurface& surface) {
    const int gridWidth = surface.width + 1;
    const int gridHeight = surface.height + 1;
    constexpr float VISUAL_WATER_Z_BIAS = 0.02f;

    std::vector<float> vertices;
    std::vector<uint32_t> indices;

    for (int y = 0; y < gridHeight; y++) {
        for (int x = 0; x < gridWidth; x++) {
            int index = y * gridWidth + x;
            float height = (index < static_cast<int>(surface.heights.size()))
                ? surface.heights[index] : surface.minHeight;

            glm::vec3 pos = surface.origin +
                            surface.stepX * static_cast<float>(x) +
                            surface.stepY * static_cast<float>(y);
            pos.z = height + VISUAL_WATER_Z_BIAS;

            // pos (3 floats)
            vertices.push_back(pos.x);
            vertices.push_back(pos.y);
            vertices.push_back(pos.z);
            // normal (3 floats) - up
            vertices.push_back(0.0f);
            vertices.push_back(0.0f);
            vertices.push_back(1.0f);
            // texcoord (2 floats)
            vertices.push_back(static_cast<float>(x) / std::max(1, gridWidth - 1));
            vertices.push_back(static_cast<float>(y) / std::max(1, gridHeight - 1));
        }
    }

    // Generate indices respecting render mask (same logic as GL version)
    for (int y = 0; y < gridHeight - 1; y++) {
        for (int x = 0; x < gridWidth - 1; x++) {
            const bool renderTile =
                waterCellRendered(surface.mask, surface.wmoId, surface.width,
                                  surface.xOffset, surface.yOffset, x, y);

            if (!renderTile) continue;

            int topLeft = y * gridWidth + x;
            int topRight = topLeft + 1;
            int bottomLeft = (y + 1) * gridWidth + x;
            int bottomRight = bottomLeft + 1;

            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    if (indices.empty()) return;
    surface.indexCount = static_cast<int>(indices.size());

    if (!vkCtx) return;

    // Upload vertex buffer
    VkDeviceSize vbSize = vertices.size() * sizeof(float);
    AllocatedBuffer vb = uploadBuffer(*vkCtx, vertices.data(), vbSize,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    surface.vertexBuffer = vb.buffer;
    surface.vertexAlloc = vb.allocation;

    // Upload index buffer
    VkDeviceSize ibSize = indices.size() * sizeof(uint32_t);
    AllocatedBuffer ib = uploadBuffer(*vkCtx, indices.data(), ibSize,
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    surface.indexBuffer = ib.buffer;
    surface.indexAlloc = ib.allocation;
}

void WaterRenderer::destroyWaterMesh(WaterSurface& surface) {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    VmaAllocator allocator = vkCtx->getAllocator();

    ::VkBuffer vertexBuffer = surface.vertexBuffer;
    VmaAllocation vertexAlloc = surface.vertexAlloc;
    ::VkBuffer indexBuffer = surface.indexBuffer;
    VmaAllocation indexAlloc = surface.indexAlloc;
    ::VkBuffer materialUBO = surface.materialUBO;
    VmaAllocation materialAlloc = surface.materialAlloc;
    VkDescriptorPool pool = materialDescPool;
    VkDescriptorSet materialSet = surface.materialSet;

    surface.vertexBuffer = VK_NULL_HANDLE;
    surface.vertexAlloc = VK_NULL_HANDLE;
    surface.indexBuffer = VK_NULL_HANDLE;
    surface.indexAlloc = VK_NULL_HANDLE;
    surface.materialUBO = VK_NULL_HANDLE;
    surface.materialAlloc = VK_NULL_HANDLE;
    surface.materialSet = VK_NULL_HANDLE;

    vkCtx->deferAfterAllFrameFences([device, allocator, vertexBuffer, vertexAlloc, indexBuffer, indexAlloc,
                                     materialUBO, materialAlloc, pool, materialSet]() {
        if (vertexBuffer) {
            AllocatedBuffer ab{}; ab.buffer = vertexBuffer; ab.allocation = vertexAlloc;
            destroyBuffer(allocator, ab);
        }
        if (indexBuffer) {
            AllocatedBuffer ab{}; ab.buffer = indexBuffer; ab.allocation = indexAlloc;
            destroyBuffer(allocator, ab);
        }
        if (materialUBO) {
            AllocatedBuffer ab{}; ab.buffer = materialUBO; ab.allocation = materialAlloc;
            destroyBuffer(allocator, ab);
        }
        if (materialSet && pool) {
            VkDescriptorSet set = materialSet;
            vkFreeDescriptorSets(device, pool, 1, &set);
        }
    });
}

// ==============================================================
// Query functions (data-only, no GL)
// ==============================================================

/// Where a point sits on a surface, if that surface has water there at all.
///
/// Every water query starts with the same pair of questions: does the point
/// fall on this surface, and does its render mask say the cell it lands in
/// holds water. Three of them asked it separately - the height, the nearest
/// height and the liquid type - and the cell arithmetic between the two halves
/// was written out each time.
///
/// The cell is clamped at the far edge because a grid of `width` cells has
/// `width + 1` corners: a point exactly on the last corner belongs to the last
/// cell, not to one past it.
std::optional<glm::vec2> WaterRenderer::wateredGridPosition(const WaterSurface& surface,
                                                            float glX, float glY) const {
    const auto grid = surfaceGridPosition(surface.origin, surface.stepX, surface.stepY,
                                          surface.width, surface.height, glX, glY);
    if (!grid) return std::nullopt;

    const int ix = std::min(static_cast<int>(grid->x), static_cast<int>(surface.width) - 1);
    const int iy = std::min(static_cast<int>(grid->y), static_cast<int>(surface.height) - 1);
    if (!waterCellRendered(surface.mask, surface.wmoId, surface.width,
                           surface.xOffset, surface.yOffset, ix, iy)) {
        return std::nullopt;
    }
    return grid;
}

std::optional<float> WaterRenderer::getWaterHeightAt(float glX, float glY) const {
    std::optional<float> best;

    for (const auto& surface : surfaces) {
        const auto grid = wateredGridPosition(surface, glX, glY);
        if (!grid) continue;

        const auto sampled = sampleGridHeight(surface.heights, surface.width,
                                              surface.height, grid->x, grid->y);
        if (!sampled) continue;
        const float h = *sampled;

        if (!best || h > *best) best = h;
    }

    return best;
}

std::optional<float> WaterRenderer::getNearestWaterHeightAt(float glX, float glY, float queryZ, float maxAbove) const {
    std::optional<float> best;
    float bestDist = 1e9f;

    for (const auto& surface : surfaces) {
        const auto grid = wateredGridPosition(surface, glX, glY);
        if (!grid) continue;

        const auto sampled = sampleGridHeight(surface.heights, surface.width,
                                              surface.height, grid->x, grid->y);
        if (!sampled) continue;
        const float h = *sampled;

        // Only consider water that's above queryZ but not too far above
        if (h < queryZ - 2.0f) continue;  // water below camera, skip
        if (h > queryZ + maxAbove) continue;  // water way above camera, skip

        float dist = std::abs(h - queryZ);
        if (!best || dist < bestDist) {
            best = h;
            bestDist = dist;
        }
    }

    return best;
}

std::optional<uint16_t> WaterRenderer::getWaterTypeAt(float glX, float glY) const {
    std::optional<float> bestHeight;
    std::optional<uint16_t> bestType;

    for (const auto& surface : surfaces) {
        if (!wateredGridPosition(surface, glX, glY)) continue;

        float h = surface.minHeight;
        if (!bestHeight || h > *bestHeight) {
            bestHeight = h;
            bestType = surface.liquidType;
        }
    }

    return bestType;
}

bool WaterRenderer::isWmoWaterAt(float glX, float glY) const {
    for (const auto& surface : surfaces) {
        if (surface.wmoId == 0) continue;
        if (surfaceGridPosition(surface.origin, surface.stepX, surface.stepY,
                                surface.width, surface.height, glX, glY)) {
            return true;
        }
    }
    return false;
}

glm::vec4 WaterRenderer::getLiquidColor(uint16_t liquidType) const {
    uint8_t basicType = (liquidType == 0) ? 0 : ((liquidType - 1) % 4);
    switch (basicType) {
        case 0:  return glm::vec4(0.10f, 0.28f, 0.55f, 1.0f); // inland: richer blue
        case 1:  return glm::vec4(0.04f, 0.16f, 0.38f, 1.0f); // ocean: deep blue
        case 2:  return glm::vec4(0.9f, 0.3f, 0.05f, 1.0f);   // magma
        case 3:  return glm::vec4(0.2f, 0.6f, 0.1f, 1.0f);    // slime
        default: return glm::vec4(0.10f, 0.28f, 0.55f, 1.0f);
    }
}

float WaterRenderer::getLiquidAlpha(uint16_t liquidType) const {
    uint8_t basicType = (liquidType == 0) ? 0 : ((liquidType - 1) % 4);
    switch (basicType) {
        case 1:  return 0.72f;  // ocean
        case 2:  return 0.75f;  // magma
        case 3:  return 0.65f;  // slime
        default: return 0.48f;  // inland water
    }
}

// ==============================================================
// Planar reflection resources
// ==============================================================

void WaterRenderer::createReflectionResources() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    VmaAllocator allocator = vkCtx->getAllocator();

    // --- Reflection color image ---
    VkImageCreateInfo colorImgCI{};
    colorImgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    colorImgCI.imageType = VK_IMAGE_TYPE_2D;
    colorImgCI.format = vkCtx->getSwapchainFormat();
    colorImgCI.extent = {.width = REFLECTION_WIDTH, .height = REFLECTION_HEIGHT, .depth = 1};
    colorImgCI.mipLevels = 1;
    colorImgCI.arrayLayers = 1;
    colorImgCI.samples = VK_SAMPLE_COUNT_1_BIT;
    colorImgCI.tiling = VK_IMAGE_TILING_OPTIMAL;
    colorImgCI.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    colorImgCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    if (vmaCreateImage(allocator, &colorImgCI, &allocCI,
                       &reflectionColorImage, &reflectionColorAlloc, nullptr) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection color image");
        return;
    }

    VkImageViewCreateInfo colorViewCI{};
    colorViewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    colorViewCI.image = reflectionColorImage;
    colorViewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
    colorViewCI.format = vkCtx->getSwapchainFormat();
    colorViewCI.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    if (vkCreateImageView(device, &colorViewCI, nullptr, &reflectionColorView) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection color view");
        return;
    }

    // --- Reflection depth image ---
    VkImageCreateInfo depthImgCI{};
    depthImgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    depthImgCI.imageType = VK_IMAGE_TYPE_2D;
    depthImgCI.format = vkCtx->getDepthFormat();
    depthImgCI.extent = {.width = REFLECTION_WIDTH, .height = REFLECTION_HEIGHT, .depth = 1};
    depthImgCI.mipLevels = 1;
    depthImgCI.arrayLayers = 1;
    depthImgCI.samples = VK_SAMPLE_COUNT_1_BIT;
    depthImgCI.tiling = VK_IMAGE_TILING_OPTIMAL;
    depthImgCI.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    depthImgCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    if (vmaCreateImage(allocator, &depthImgCI, &allocCI,
                       &reflectionDepthImage, &reflectionDepthAlloc, nullptr) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection depth image");
        return;
    }

    VkImageViewCreateInfo depthViewCI{};
    depthViewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    depthViewCI.image = reflectionDepthImage;
    depthViewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
    depthViewCI.format = vkCtx->getDepthFormat();
    depthViewCI.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    if (vkCreateImageView(device, &depthViewCI, nullptr, &reflectionDepthView) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection depth view");
        return;
    }

    // --- Reflection sampler ---
    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    reflectionSampler = vkCtx->getOrCreateSampler(sampCI);
    if (reflectionSampler == VK_NULL_HANDLE) {
        LOG_ERROR("WaterRenderer: failed to create reflection sampler");
        return;
    }

    // --- Reflection render pass ---
    VkAttachmentDescription colorAttach{};
    colorAttach.format = vkCtx->getSwapchainFormat();
    colorAttach.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttach.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttach.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription depthAttach{};
    depthAttach.format = vkCtx->getDepthFormat();
    depthAttach.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttach.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttach.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttach.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{.attachment = 0, .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{.attachment = 1, .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::array<VkAttachmentDescription, 2> attachments = {colorAttach, depthAttach};
    VkRenderPassCreateInfo rpCI{};
    rpCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpCI.attachmentCount = static_cast<uint32_t>(attachments.size());
    rpCI.pAttachments = attachments.data();
    rpCI.subpassCount = 1;
    rpCI.pSubpasses = &subpass;
    rpCI.dependencyCount = 1;
    rpCI.pDependencies = &dep;

    if (vkCreateRenderPass(device, &rpCI, nullptr, &reflectionRenderPass) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection render pass");
        return;
    }

    // --- Reflection framebuffer ---
    std::array<VkImageView, 2> fbAttach = {reflectionColorView, reflectionDepthView};
    VkFramebufferCreateInfo fbCI{};
    fbCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbCI.renderPass = reflectionRenderPass;
    fbCI.attachmentCount = static_cast<uint32_t>(fbAttach.size());
    fbCI.pAttachments = fbAttach.data();
    fbCI.width = REFLECTION_WIDTH;
    fbCI.height = REFLECTION_HEIGHT;
    fbCI.layers = 1;

    if (vkCreateFramebuffer(device, &fbCI, nullptr, &reflectionFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection framebuffer");
        return;
    }

    // --- Reflection UBO ---
    VkBufferCreateInfo bufCI{};
    bufCI.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCI.size = sizeof(WaterFrameUBOData);
    bufCI.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

    VmaAllocationCreateInfo uboAllocCI{};
    uboAllocCI.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    uboAllocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

    VmaAllocationInfo mapInfo{};
    if (vmaCreateBuffer(allocator, &bufCI, &uboAllocCI,
                        &reflectionUBO, &reflectionUBOAlloc, &mapInfo) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection UBO");
        return;
    }
    reflectionUBOMapped = mapInfo.pMappedData;

    // Initialize with identity
    WaterFrameUBOData initData{};
    initData.reflViewProj = glm::mat4(1.0f);
    if (reflectionUBOMapped) {
        std::memcpy(reflectionUBOMapped, &initData, sizeof(initData));
    }

    // Transition reflection color image to shader-read so first frame doesn't read undefined
    vkCtx->immediateSubmit([&](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = reflectionColorImage;
        barrier.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                             0, 0, nullptr, 0, nullptr, 1, &barrier);
    });
    reflectionColorLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    LOG_INFO("Water reflection resources created (", REFLECTION_WIDTH, "x", REFLECTION_HEIGHT, ")");
}

void WaterRenderer::destroyReflectionResources() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    VmaAllocator allocator = vkCtx->getAllocator();

    if (reflectionFramebuffer) { vkDestroyFramebuffer(device, reflectionFramebuffer, nullptr); reflectionFramebuffer = VK_NULL_HANDLE; }
    if (reflectionRenderPass) { vkDestroyRenderPass(device, reflectionRenderPass, nullptr); reflectionRenderPass = VK_NULL_HANDLE; }
    if (reflectionColorView) { vkDestroyImageView(device, reflectionColorView, nullptr); reflectionColorView = VK_NULL_HANDLE; }
    if (reflectionDepthView) { vkDestroyImageView(device, reflectionDepthView, nullptr); reflectionDepthView = VK_NULL_HANDLE; }
    if (reflectionColorImage) { vmaDestroyImage(allocator, reflectionColorImage, reflectionColorAlloc); reflectionColorImage = VK_NULL_HANDLE; }
    if (reflectionDepthImage) { vmaDestroyImage(allocator, reflectionDepthImage, reflectionDepthAlloc); reflectionDepthImage = VK_NULL_HANDLE; }
    reflectionSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
    if (reflectionUBO) {
        AllocatedBuffer ab{}; ab.buffer = reflectionUBO; ab.allocation = reflectionUBOAlloc;
        destroyBuffer(allocator, ab);
        reflectionUBO = VK_NULL_HANDLE;
        reflectionUBOMapped = nullptr;
    }
    reflectionColorLayout = VK_IMAGE_LAYOUT_UNDEFINED;
}

// ==============================================================
// Reflection pass begin/end
// ==============================================================

bool WaterRenderer::beginReflectionPass(VkCommandBuffer cmd) {
    if (!reflectionRenderPass || !reflectionFramebuffer || !cmd) return false;

    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = reflectionRenderPass;
    rpInfo.framebuffer = reflectionFramebuffer;
    rpInfo.renderArea = {.offset = {.x = 0, .y = 0}, .extent = {.width = REFLECTION_WIDTH, .height = REFLECTION_HEIGHT}};

    VkClearValue clears[2]{};
    clears[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clears[1].depthStencil = {.depth = 1.0f, .stencil = 0};
    rpInfo.clearValueCount = 2;
    rpInfo.pClearValues = clears;

    vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{.x = 0, .y = 0, .width = static_cast<float>(REFLECTION_WIDTH), .height = static_cast<float>(REFLECTION_HEIGHT), .minDepth = 0.0f, .maxDepth = 1.0f};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    VkRect2D sc{.offset = {.x = 0, .y = 0}, .extent = {.width = REFLECTION_WIDTH, .height = REFLECTION_HEIGHT}};
    vkCmdSetScissor(cmd, 0, 1, &sc);

    return true;
}

void WaterRenderer::endReflectionPass(VkCommandBuffer cmd) {
    if (!cmd) return;
    vkCmdEndRenderPass(cmd);
    reflectionColorLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // Update only the current frame's scene descriptor set with the reflection texture.
    // Updating all frames would race with in-flight command buffers that have the
    // other frame's descriptor set bound.
    if (reflectionColorView && reflectionSampler && vkCtx) {
        uint32_t fi = vkCtx->getCurrentFrame() % SCENE_HISTORY_FRAMES;
        if (sceneHistory[fi].sceneSet) {
            VkDescriptorImageInfo reflInfo{};
            reflInfo.sampler = reflectionSampler;
            reflInfo.imageView = reflectionColorView;
            reflInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            VkWriteDescriptorSet write{};
            write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            write.dstSet = sceneHistory[fi].sceneSet;
            write.dstBinding = 2;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            write.pImageInfo = &reflInfo;
            vkUpdateDescriptorSets(vkCtx->getDevice(), 1, &write, 0, nullptr);
        }
    }
}

void WaterRenderer::updateReflectionUBO(const glm::mat4& reflViewProj) {
    frameUBO_.reflViewProj = reflViewProj;
    uploadFrameUBO();
}

void WaterRenderer::uploadFrameUBO() {
    if (!reflectionUBOMapped) return;
    std::memcpy(reflectionUBOMapped, &frameUBO_, sizeof(frameUBO_));
}

// Disturbance trail. Points are dropped along the path rather than parented to
// the character, so the froth stays where the water was churned and the
// character runs out of it - froth pinned under the feet reads as a decal.
void WaterRenderer::updateWake(float deltaTime, const glm::vec2& pos,
                               const glm::vec2& travelDir, float intensity, bool wading) {
    // Age the existing trail. Drift is what spreads a swimmer's two emission
    // lines apart with distance behind, which is the V.
    for (auto& p : wakePoints_) {
        p.age += deltaTime;
        p.pos += p.drift * deltaTime;
        // Barely any drag. A wake's arms are straight because the separation
        // keeps growing with distance behind; damping the drift collapsed the
        // two arms back onto each other and the V read as a single line.
        p.drift *= std::max(0.0f, 1.0f - 0.25f * deltaTime);
    }
    wakePoints_.erase(std::remove_if(wakePoints_.begin(), wakePoints_.end(),
                                     [](const WakePoint& p) { return p.age >= p.life; }),
                      wakePoints_.end());

    if (intensity > 0.0f) {
        // Emit by distance travelled, not by time: standing still in the
        // shallows should settle rather than pile froth up in one spot.
        // Close enough together that the patches overlap into a continuous
        // trail. Spaced out, each one reads as its own blob.
        const float spacing = wading ? 0.40f : 0.60f;
        bool emit = !hasWakeEmitPos_;
        if (!emit) {
            const glm::vec2 delta = pos - lastWakeEmitPos_;
            emit = glm::dot(delta, delta) >= spacing * spacing;
        }

        if (emit) {
            hasWakeEmitPos_ = true;
            lastWakeEmitPos_ = pos;

            glm::vec2 fwd = travelDir;
            const float fwdLenSq = glm::dot(fwd, fwd);
            fwd = (fwdLenSq > 1e-6f) ? fwd * glm::inversesqrt(fwdLenSq) : glm::vec2(0.0f, 1.0f);
            const glm::vec2 side(-fwd.y, fwd.x);

            auto push = [&](const glm::vec2& at, const glm::vec2& drift,
                            float life, float strength) {
                if (static_cast<int>(wakePoints_.size()) >= kMaxWakePoints) {
                    wakePoints_.erase(wakePoints_.begin());  // oldest goes first
                }
                WakePoint wp;
                wp.pos = at;
                wp.drift = drift;
                wp.life = life;
                wp.strength = strength;
                wakePoints_.push_back(wp);
            };

            if (wading) {
                // Legs punching through the surface: one churned patch per
                // stride, thrown slightly forward of where the foot lands.
                push(pos + fwd * 0.25f, side * 0.30f, 1.5f, intensity);
            } else {
                // Swimming: a pair off the shoulders, each drifting outward and
                // back. The arms open with age because the emissions keep
                // separating after the swimmer has moved on, so the trail is a V
                // even though every point is laid down on the same line.
                const float spread = 0.50f;
                const float lateral = 1.15f;
                const float life = 1.9f;
                push(pos + side * spread - fwd * 0.2f,
                     side * lateral - fwd * 0.30f, life, intensity * 0.8f);
                push(pos - side * spread - fwd * 0.2f,
                     -side * lateral - fwd * 0.30f, life, intensity * 0.8f);
            }
        }
    } else {
        hasWakeEmitPos_ = false;
    }

    // Pack for the shader, and build the bounding circle the fragment shader
    // uses to reject the rest of the sheet in one test.
    const int count = std::min(static_cast<int>(wakePoints_.size()), kMaxWakePoints);
    glm::vec2 centre(0.0f);
    for (int i = 0; i < count; ++i) centre += wakePoints_[i].pos;
    if (count > 0) centre /= static_cast<float>(count);

    float cullRadius = 0.0f;
    for (int i = 0; i < count; ++i) {
        const WakePoint& p = wakePoints_[i];
        const float age01 = std::min(1.0f, p.age / p.life);
        // Must cover the point's own spread, or froth is clipped mid-patch.
        constexpr float kMaxPatchRadius = 1.45f;
        cullRadius = std::max(cullRadius, glm::length(p.pos - centre) + kMaxPatchRadius);
        frameUBO_.wakePoints[i] = glm::vec4(p.pos.x, p.pos.y, age01, p.strength);
    }
    frameUBO_.wakeBounds = glm::vec4(centre.x, centre.y, cullRadius, static_cast<float>(count));
    uploadFrameUBO();
}

// ==============================================================
// Mirror camera computations
// ==============================================================

std::optional<float> WaterRenderer::getDominantWaterHeight(const glm::vec3& cameraPos) const {
    if (surfaces.empty()) return std::nullopt;

    // Find the water surface closest to the camera (XY distance)
    float bestDist = std::numeric_limits<float>::max();
    float bestHeight = 0.0f;
    bool found = false;

    for (const auto& surface : surfaces) {
        // Skip magma/slime - only reflect water/ocean
        uint8_t basicType = (surface.liquidType == 0) ? 0 : ((surface.liquidType - 1) % 4);
        if (basicType >= 2) continue;

        // Compute center of surface in world space
        glm::vec3 center = surface.origin +
            surface.stepX * (static_cast<float>(surface.width) * 0.5f) +
            surface.stepY * (static_cast<float>(surface.height) * 0.5f);

        float dx = cameraPos.x - center.x;
        float dy = cameraPos.y - center.y;
        float dist = dx * dx + dy * dy;
        if (dist < bestDist) {
            bestDist = dist;
            bestHeight = surface.minHeight;
            found = true;
        }
    }

    if (!found) return std::nullopt;
    return bestHeight;
}

glm::mat4 WaterRenderer::computeReflectedView(const Camera& camera, float waterHeight) {
    // In this engine, Z is up. Water height is stored in the Z component.
    // Mirror camera position across Z = waterHeight plane.

    glm::vec3 camPos = camera.getPosition();
    glm::vec3 reflPos = camPos;
    reflPos.z = 2.0f * waterHeight - camPos.z;

    // Get camera forward and reflect the Z component
    glm::vec3 forward = camera.getForward();
    forward.z = -forward.z;
    glm::vec3 reflTarget = reflPos + forward;

    glm::vec3 up(0.0f, 0.0f, 1.0f);
    return glm::lookAt(reflPos, reflTarget, up);
}

glm::mat4 WaterRenderer::computeObliqueProjection(const glm::mat4& proj, const glm::mat4& view,
                                                    float waterHeight) {
    // Clip plane: everything below waterHeight in world space
    // Z is up, so the clip plane normal is (0, 0, 1)
    glm::vec4 clipPlaneWorld(0.0f, 0.0f, 1.0f, -waterHeight);
    glm::vec4 clipPlaneView = glm::transpose(glm::inverse(view)) * clipPlaneWorld;

    // Lengyel's oblique near-plane projection matrix modification
    glm::mat4 result = proj;
    glm::vec4 q;
    q.x = (glm::sign(clipPlaneView.x) + result[2][0]) / result[0][0];
    q.y = (glm::sign(clipPlaneView.y) + result[2][1]) / result[1][1];
    q.z = -1.0f;
    q.w = (1.0f + result[2][2]) / result[3][2];

    glm::vec4 c = clipPlaneView * (2.0f / glm::dot(clipPlaneView, q));
    result[0][2] = c.x;
    result[1][2] = c.y;
    result[2][2] = c.z + 1.0f;
    result[3][2] = c.w;

    return result;
}

// ==============================================================
// Separate 1x water pass (used when MSAA is active)
// ==============================================================

bool WaterRenderer::createWater1xPass(VkFormat colorFormat, VkFormat depthFormat) {
    if (!vkCtx) return false;
    VkDevice device = vkCtx->getDevice();

    VkAttachmentDescription attachments[2]{};
    // Color: load existing resolved content, store after water draw
    attachments[0].format = colorFormat;
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // Depth: load resolved depth for depth testing
    attachments[1].format = depthFormat;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{.attachment = 0, .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{.attachment = 1, .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    VkRenderPassCreateInfo rpCI{};
    rpCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpCI.attachmentCount = 2;
    rpCI.pAttachments = attachments;
    rpCI.subpassCount = 1;
    rpCI.pSubpasses = &subpass;
    rpCI.dependencyCount = 1;
    rpCI.pDependencies = &dep;

    if (vkCreateRenderPass(device, &rpCI, nullptr, &water1xRenderPass) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create 1x water render pass");
        return false;
    }

    // Build 1x water pipeline against this render pass
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/water.vert.spv") ||
        !fragShader.loadFromFile(device, "assets/shaders/water.frag.spv")) {
        LOG_ERROR("WaterRenderer: failed to load shaders for 1x pipeline");
        return false;
    }

    VkVertexInputBindingDescription vertBinding{};
    vertBinding.binding = 0;
    vertBinding.stride = 8 * sizeof(float);
    vertBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> vertAttribs = {
        { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },
        { 1, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float) },
    };

    water1xPipeline = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertBinding }, vertAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(VK_SAMPLE_COUNT_1_BIT)
        .setLayout(pipelineLayout)
        .setRenderPass(water1xRenderPass)
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx->getPipelineCache());

    vertShader.destroy();
    fragShader.destroy();

    if (!water1xPipeline) {
        LOG_ERROR("WaterRenderer: failed to create 1x water pipeline");
        return false;
    }

    LOG_INFO("WaterRenderer: created 1x water pass and pipeline");
    return true;
}

void WaterRenderer::createWater1xFramebuffers(const std::vector<VkImageView>& swapViews,
                                               VkImageView depthView, VkExtent2D extent) {
    if (!vkCtx || !water1xRenderPass || !depthView) return;
    VkDevice device = vkCtx->getDevice();

    // Destroy old framebuffers
    for (auto fb : water1xFramebuffers) {
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    }
    water1xFramebuffers.clear();

    water1xFramebuffers.resize(swapViews.size());
    for (size_t i = 0; i < swapViews.size(); i++) {
        VkImageView views[2] = { swapViews[i], depthView };
        VkFramebufferCreateInfo fbCI{};
        fbCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbCI.renderPass = water1xRenderPass;
        fbCI.attachmentCount = 2;
        fbCI.pAttachments = views;
        fbCI.width = extent.width;
        fbCI.height = extent.height;
        fbCI.layers = 1;
        if (vkCreateFramebuffer(device, &fbCI, nullptr, &water1xFramebuffers[i]) != VK_SUCCESS) {
            LOG_ERROR("WaterRenderer: failed to create 1x framebuffer ", i);
        }
    }
}

void WaterRenderer::destroyWater1xResources() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    for (auto fb : water1xFramebuffers) {
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    }
    water1xFramebuffers.clear();
    destroy(device, water1xPipeline);
    if (water1xRenderPass) { vkDestroyRenderPass(device, water1xRenderPass, nullptr); water1xRenderPass = VK_NULL_HANDLE; }
}
} // namespace rendering
} // namespace wowee
