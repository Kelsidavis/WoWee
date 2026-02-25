#include "rendering/water_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
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
};

// Matches set 2 binding 3 in water.frag.glsl
struct ReflectionUBOData {
    glm::mat4 reflViewProj;
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

    // Pool needs 3 combined image samplers + 1 uniform buffer
    std::array<VkDescriptorPoolSize, 2> scenePoolSizes{};
    scenePoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    scenePoolSizes[0].descriptorCount = 3;
    scenePoolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    scenePoolSizes[1].descriptorCount = 1;
    VkDescriptorPoolCreateInfo scenePoolInfo{};
    scenePoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    scenePoolInfo.maxSets = 1;
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

    createSceneHistoryResources(vkCtx->getSwapchainExtent(),
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
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

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
    createSceneHistoryResources(vkCtx->getSwapchainExtent(),
                                vkCtx->getSwapchainFormat(),
                                vkCtx->getDepthFormat());

    // Destroy old pipeline (keep layout)
    if (waterPipeline) { vkDestroyPipeline(device, waterPipeline, nullptr); waterPipeline = VK_NULL_HANDLE; }

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
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    vertShader.destroy();
    fragShader.destroy();

    if (!waterPipeline) {
        LOG_ERROR("WaterRenderer::recreatePipelines: failed to create pipeline");
    }
}

void WaterRenderer::shutdown() {
    clear();

    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    vkDeviceWaitIdle(device);

    destroyWater1xResources();
    destroyReflectionResources();
    destroySceneHistoryResources();
    if (waterPipeline) { vkDestroyPipeline(device, waterPipeline, nullptr); waterPipeline = VK_NULL_HANDLE; }
    if (pipelineLayout) { vkDestroyPipelineLayout(device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
    if (sceneDescPool) { vkDestroyDescriptorPool(device, sceneDescPool, nullptr); sceneDescPool = VK_NULL_HANDLE; }
    if (sceneSetLayout) { vkDestroyDescriptorSetLayout(device, sceneSetLayout, nullptr); sceneSetLayout = VK_NULL_HANDLE; }
    if (materialDescPool) { vkDestroyDescriptorPool(device, materialDescPool, nullptr); materialDescPool = VK_NULL_HANDLE; }
    if (materialSetLayout) { vkDestroyDescriptorSetLayout(device, materialSetLayout, nullptr); materialSetLayout = VK_NULL_HANDLE; }

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
    if (sceneColorView) { vkDestroyImageView(device, sceneColorView, nullptr); sceneColorView = VK_NULL_HANDLE; }
    if (sceneDepthView) { vkDestroyImageView(device, sceneDepthView, nullptr); sceneDepthView = VK_NULL_HANDLE; }
    if (sceneColorImage) { vmaDestroyImage(vkCtx->getAllocator(), sceneColorImage, sceneColorAlloc); sceneColorImage = VK_NULL_HANDLE; sceneColorAlloc = VK_NULL_HANDLE; }
    if (sceneDepthImage) { vmaDestroyImage(vkCtx->getAllocator(), sceneDepthImage, sceneDepthAlloc); sceneDepthImage = VK_NULL_HANDLE; sceneDepthAlloc = VK_NULL_HANDLE; }
    if (sceneColorSampler) { vkDestroySampler(device, sceneColorSampler, nullptr); sceneColorSampler = VK_NULL_HANDLE; }
    if (sceneDepthSampler) { vkDestroySampler(device, sceneDepthSampler, nullptr); sceneDepthSampler = VK_NULL_HANDLE; }
    sceneSet = VK_NULL_HANDLE;
    sceneHistoryExtent = {0, 0};
    sceneHistoryReady = false;
}

void WaterRenderer::createSceneHistoryResources(VkExtent2D extent, VkFormat colorFormat, VkFormat depthFormat) {
    if (!vkCtx || extent.width == 0 || extent.height == 0 || !sceneSetLayout || !sceneDescPool) return;
    VkDevice device = vkCtx->getDevice();

    destroySceneHistoryResources();
    vkResetDescriptorPool(device, sceneDescPool, 0);
    sceneHistoryExtent = extent;

    VkImageCreateInfo colorImgInfo{};
    colorImgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    colorImgInfo.imageType = VK_IMAGE_TYPE_2D;
    colorImgInfo.format = colorFormat;
    colorImgInfo.extent = {extent.width, extent.height, 1};
    colorImgInfo.mipLevels = 1;
    colorImgInfo.arrayLayers = 1;
    colorImgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    colorImgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    colorImgInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    colorImgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    if (vmaCreateImage(vkCtx->getAllocator(), &colorImgInfo, &allocCI, &sceneColorImage, &sceneColorAlloc, nullptr) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene color history image");
        return;
    }

    VkImageCreateInfo depthImgInfo = colorImgInfo;
    depthImgInfo.format = depthFormat;
    if (vmaCreateImage(vkCtx->getAllocator(), &depthImgInfo, &allocCI, &sceneDepthImage, &sceneDepthAlloc, nullptr) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene depth history image");
        return;
    }

    VkImageViewCreateInfo colorViewInfo{};
    colorViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    colorViewInfo.image = sceneColorImage;
    colorViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    colorViewInfo.format = colorFormat;
    colorViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colorViewInfo.subresourceRange.levelCount = 1;
    colorViewInfo.subresourceRange.layerCount = 1;
    if (vkCreateImageView(device, &colorViewInfo, nullptr, &sceneColorView) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene color history view");
        return;
    }

    VkImageViewCreateInfo depthViewInfo = colorViewInfo;
    depthViewInfo.image = sceneDepthImage;
    depthViewInfo.format = depthFormat;
    depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    if (vkCreateImageView(device, &depthViewInfo, nullptr, &sceneDepthView) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene depth history view");
        return;
    }

    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (vkCreateSampler(device, &sampCI, nullptr, &sceneColorSampler) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene color sampler");
        return;
    }
    sampCI.magFilter = VK_FILTER_NEAREST;
    sampCI.minFilter = VK_FILTER_NEAREST;
    if (vkCreateSampler(device, &sampCI, nullptr, &sceneDepthSampler) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create scene depth sampler");
        return;
    }

    VkDescriptorSetAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    ai.descriptorPool = sceneDescPool;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &sceneSetLayout;
    if (vkAllocateDescriptorSets(device, &ai, &sceneSet) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to allocate scene descriptor set");
        sceneSet = VK_NULL_HANDLE;
        return;
    }

    VkDescriptorImageInfo colorInfo{};
    colorInfo.sampler = sceneColorSampler;
    colorInfo.imageView = sceneColorView;
    colorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkDescriptorImageInfo depthInfo{};
    depthInfo.sampler = sceneDepthSampler;
    depthInfo.imageView = sceneDepthView;
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // Reflection color texture (binding 2) — use scene color as placeholder until reflection is created
    VkDescriptorImageInfo reflColorInfo{};
    reflColorInfo.sampler = sceneColorSampler;
    reflColorInfo.imageView = reflectionColorView ? reflectionColorView : sceneColorView;
    reflColorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // Reflection UBO (binding 3)
    VkDescriptorBufferInfo reflUBOInfo{};
    reflUBOInfo.buffer = reflectionUBO;
    reflUBOInfo.offset = 0;
    reflUBOInfo.range = sizeof(ReflectionUBOData);

    // Write bindings 0,1 always; write 2,3 only if reflection resources exist
    std::vector<VkWriteDescriptorSet> writes;

    VkWriteDescriptorSet w0{};
    w0.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w0.dstSet = sceneSet;
    w0.dstBinding = 0;
    w0.descriptorCount = 1;
    w0.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w0.pImageInfo = &colorInfo;
    writes.push_back(w0);

    VkWriteDescriptorSet w1{};
    w1.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w1.dstSet = sceneSet;
    w1.dstBinding = 1;
    w1.descriptorCount = 1;
    w1.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w1.pImageInfo = &depthInfo;
    writes.push_back(w1);

    VkWriteDescriptorSet w2{};
    w2.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w2.dstSet = sceneSet;
    w2.dstBinding = 2;
    w2.descriptorCount = 1;
    w2.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w2.pImageInfo = &reflColorInfo;
    writes.push_back(w2);

    if (reflectionUBO) {
        VkWriteDescriptorSet w3{};
        w3.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w3.dstSet = sceneSet;
        w3.dstBinding = 3;
        w3.descriptorCount = 1;
        w3.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w3.pBufferInfo = &reflUBOInfo;
        writes.push_back(w3);
    }

    vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

    // Initialize history images to shader-read layout so first frame samples are defined.
    vkCtx->immediateSubmit([&](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barriers[2]{};
        barriers[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barriers[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barriers[0].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[0].image = sceneColorImage;
        barriers[0].subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        barriers[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        barriers[1] = barriers[0];
        barriers[1].image = sceneDepthImage;
        barriers[1].subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                             0, 0, nullptr, 0, nullptr, 2, barriers);
    });
}

void WaterRenderer::updateMaterialUBO(WaterSurface& surface) {
    glm::vec4 color = getLiquidColor(surface.liquidType);
    float alpha = getLiquidAlpha(surface.liquidType);

    // WMO liquid material override
    if (surface.wmoId != 0) {
        const uint8_t basicType = (surface.liquidType == 0) ? 0 : ((surface.liquidType - 1) % 4);
        if (basicType == 2 || basicType == 3) {
            color = glm::vec4(0.2f, 0.4f, 0.6f, 1.0f);
            alpha = 0.45f;
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
    }
}

// ==============================================================
// Data loading (preserved from GL version — no GL calls)
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
        tileWorldX = (32.0f - tileX) * 533.33333f;
        tileWorldY = (32.0f - tileY) * 533.33333f;
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
                    surface.position.x - (static_cast<float>(layer.y) * TILE_SIZE),
                    surface.position.y - (static_cast<float>(layer.x) * TILE_SIZE),
                    layer.minHeight
                );
                surface.stepX = glm::vec3(0.0f, -TILE_SIZE, 0.0f);
                surface.stepY = glm::vec3(-TILE_SIZE, 0.0f, 0.0f);

                surface.minHeight = layer.minHeight;
                surface.maxHeight = layer.maxHeight;
                surface.liquidType = layer.liquidType;
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
        surface.stepX = glm::vec3(0.0f, -TILE_SIZE, 0.0f);
        surface.stepY = glm::vec3(-TILE_SIZE, 0.0f, 0.0f);

        surface.minHeight = groupHeight;
        surface.maxHeight = groupHeight;

        // Initialize height grid (129×129) with group height
        constexpr int MERGED_W = 128;
        const int gridW = MERGED_W + 1;  // 129
        const int gridH = MERGED_W + 1;
        surface.heights.resize(gridW * gridH, groupHeight);

        // Initialize mask (128×128 sub-tiles, all masked OUT)
        // Mask uses LSB bit order: tileIndex = row * 128 + col
        const int maskBytes = (MERGED_W * MERGED_W + 7) / 8;
        surface.mask.resize(maskBytes, 0);

        // ── Fill from each contributing chunk ──
        for (const auto& info : chunkLayers) {
            const auto& layer = *info.layer;

            // Merged grid offset for this chunk
            // gx = chunkY*8 + layer.x + localX, gy = chunkX*8 + layer.y + localY
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
                    int mgx = baseGx + layer.x + lx;
                    int mgy = baseGy + layer.y + ly;
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

            // Copy mask — mark contributing sub-tiles as renderable
            for (int ly = 0; ly < layer.height; ly++) {
                for (int lx = 0; lx < layer.width; lx++) {
                    bool render = true;
                    if (!layer.mask.empty()) {
                        int cx = layer.x + lx;
                        int cy = layer.y + ly;
                        int origTileIdx = cy * 8 + cx;
                        int origByte = origTileIdx / 8;
                        int origBit = origTileIdx % 8;
                        if (origByte < static_cast<int>(layer.mask.size())) {
                            uint8_t mb = layer.mask[origByte];
                            render = (mb & (1 << origBit)) || (mb & (1 << (7 - origBit)));
                        }
                    }

                    if (render) {
                        int mx = baseGx + layer.x + lx;
                        int my = baseGy + layer.y + ly;
                        if (mx >= MERGED_W || my >= MERGED_W) continue;

                        int mergedTileIdx = my * MERGED_W + mx;
                        int byteIdx = mergedTileIdx / 8;
                        int bitIdx = mergedTileIdx % 8;
                        surface.mask[byteIdx] |= static_cast<uint8_t>(1 << bitIdx);
                    }
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

    LOG_DEBUG("Water: Loaded ", totalSurfaces, " surfaces from tile [", tileX, ",", tileY,
              "] (", mergeGroups.size(), " groups), total surfaces: ", surfaces.size());
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
    float nz = (glm::length(planeN) > 1e-4f) ? std::abs(glm::normalize(planeN).z) : 0.0f;
    float spanX = stepXLen * static_cast<float>(surface.width);
    float spanY = stepYLen * static_cast<float>(surface.height);
    if (stepXLen < 0.2f || stepXLen > 12.0f ||
        stepYLen < 0.2f || stepYLen > 12.0f ||
        nz < 0.60f || spanX > 450.0f || spanY > 450.0f) return;

    const int gridWidth = static_cast<int>(surface.width) + 1;
    const int gridHeight = static_cast<int>(surface.height) + 1;
    const int vertexCount = gridWidth * gridHeight;

    // WMO liquid base heights sit ~2 units above the visual waterline.
    // Lower them to match surrounding terrain water and prevent clipping
    // at bridge edges and walkways.
    constexpr float WMO_WATER_Z_OFFSET = -1.0f;
    float adjustedZ = surface.origin.z + WMO_WATER_Z_OFFSET;
    surface.heights.assign(vertexCount, adjustedZ);
    surface.minHeight = adjustedZ;
    surface.maxHeight = adjustedZ;
    surface.origin.z = adjustedZ;
    surface.position.z = adjustedZ;


    if (surface.origin.z > 300.0f || surface.origin.z < -100.0f) return;

    // Build tile mask from MLIQ flags — tiles with (flag & 0x0F) == 0x0F have no liquid
    size_t tileCount = static_cast<size_t>(surface.width) * static_cast<size_t>(surface.height);
    size_t maskBytes = (tileCount + 7) / 8;
    surface.mask.assign(maskBytes, 0x00);
    for (size_t t = 0; t < tileCount; t++) {
        bool hasLiquid = true;
        if (t < liquid.flags.size()) {
            // In WoW MLIQ format, (flags & 0x0F) == 0x0F means "no liquid" for this tile
            if ((liquid.flags[t] & 0x0F) == 0x0F) {
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

    if (vkCtx && materialDescPool) {
        vkResetDescriptorPool(vkCtx->getDevice(), materialDescPool, 0);
    }
}

// ==============================================================
// Rendering
// ==============================================================

void WaterRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet,
                            const Camera& /*camera*/, float /*time*/, bool use1x) {
    VkPipeline pipeline = (use1x && water1xPipeline) ? water1xPipeline : waterPipeline;
    if (!renderingEnabled || surfaces.empty() || !pipeline) return;
    if (!sceneSet) return;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                             0, 1, &perFrameSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                             2, 1, &sceneSet, 0, nullptr);

    for (const auto& surface : surfaces) {
        if (surface.vertexBuffer == VK_NULL_HANDLE || surface.indexCount == 0) continue;
        if (!surface.materialSet) continue;

        bool isWmoWater = (surface.wmoId != 0);
        bool canalProfile = isWmoWater || (surface.liquidType == 5);
        uint8_t basicType = (surface.liquidType == 0) ? 0 : ((surface.liquidType - 1) % 4);
        // WMO water gets no wave displacement — prevents visible slosh at
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
                                        bool srcDepthIsMsaa) {
    if (!vkCtx || !cmd || !sceneColorImage || !sceneDepthImage || srcExtent.width == 0 || srcExtent.height == 0) {
        return;
    }

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
    barrier2(srcColorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
             0, VK_ACCESS_TRANSFER_READ_BIT,
             VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    barrier2(sceneColorImage, VK_IMAGE_ASPECT_COLOR_BIT,
             VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
             VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

    VkImageCopy colorCopy{};
    colorCopy.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    colorCopy.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    colorCopy.extent = {copyExtent.width, copyExtent.height, 1};
    vkCmdCopyImage(cmd, srcColorImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   sceneColorImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &colorCopy);

    barrier2(sceneColorImage, VK_IMAGE_ASPECT_COLOR_BIT,
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
        barrier2(sceneDepthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                 VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

        VkImageCopy depthCopy{};
        depthCopy.srcSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 0, 1};
        depthCopy.dstSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 0, 1};
        depthCopy.extent = {copyExtent.width, copyExtent.height, 1};
        vkCmdCopyImage(cmd, srcDepthImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       sceneDepthImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &depthCopy);

        barrier2(sceneDepthImage, VK_IMAGE_ASPECT_DEPTH_BIT,
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
            bool renderTile = true;
            if (!surface.mask.empty()) {
                int tileIndex;
                bool isMergedTerrain = (surface.wmoId == 0 && surface.width > 8);
                if (surface.wmoId == 0 && surface.width <= 8 && surface.mask.size() >= 8) {
                    int cx = static_cast<int>(surface.xOffset) + x;
                    int cy = static_cast<int>(surface.yOffset) + y;
                    tileIndex = cy * 8 + cx;
                } else {
                    tileIndex = y * surface.width + x;
                }
                int byteIndex = tileIndex / 8;
                int bitIndex = tileIndex % 8;
                if (byteIndex < static_cast<int>(surface.mask.size())) {
                    uint8_t maskByte = surface.mask[byteIndex];
                    if (isMergedTerrain) {
                        // Merged surfaces use LSB-only bit order
                        renderTile = (maskByte & (1 << bitIndex)) != 0;
                    } else {
                        bool lsbOrder = (maskByte & (1 << bitIndex)) != 0;
                        bool msbOrder = (maskByte & (1 << (7 - bitIndex))) != 0;
                        renderTile = lsbOrder || msbOrder;
                    }

                    if (!renderTile) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                if (dx == 0 && dy == 0) continue;
                                int nx = x + dx, ny = y + dy;
                                if (nx < 0 || ny < 0 || nx >= gridWidth-1 || ny >= gridHeight-1) continue;
                                int neighborIdx;
                                if (surface.wmoId == 0 && surface.width <= 8 && surface.mask.size() >= 8) {
                                    neighborIdx = (static_cast<int>(surface.yOffset) + ny) * 8 +
                                                  (static_cast<int>(surface.xOffset) + nx);
                                } else {
                                    neighborIdx = ny * surface.width + nx;
                                }
                                int nByteIdx = neighborIdx / 8;
                                int nBitIdx = neighborIdx % 8;
                                if (nByteIdx < static_cast<int>(surface.mask.size())) {
                                    uint8_t nMask = surface.mask[nByteIdx];
                                    if (isMergedTerrain) {
                                        if (nMask & (1 << nBitIdx)) {
                                            renderTile = true;
                                            goto found_neighbor;
                                        }
                                    } else {
                                        if ((nMask & (1 << nBitIdx)) || (nMask & (1 << (7 - nBitIdx)))) {
                                            renderTile = true;
                                            goto found_neighbor;
                                        }
                                    }
                                }
                            }
                        }
                        found_neighbor:;
                    }
                }
            }

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

    // Fallback: if terrain MH2O mask produced no tiles, render full rect
    if (indices.empty() && surface.wmoId == 0) {
        for (int y = 0; y < gridHeight - 1; y++) {
            for (int x = 0; x < gridWidth - 1; x++) {
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
    VmaAllocator allocator = vkCtx->getAllocator();

    if (surface.vertexBuffer) {
        AllocatedBuffer ab{}; ab.buffer = surface.vertexBuffer; ab.allocation = surface.vertexAlloc;
        destroyBuffer(allocator, ab);
        surface.vertexBuffer = VK_NULL_HANDLE;
    }
    if (surface.indexBuffer) {
        AllocatedBuffer ab{}; ab.buffer = surface.indexBuffer; ab.allocation = surface.indexAlloc;
        destroyBuffer(allocator, ab);
        surface.indexBuffer = VK_NULL_HANDLE;
    }
    if (surface.materialUBO) {
        AllocatedBuffer ab{}; ab.buffer = surface.materialUBO; ab.allocation = surface.materialAlloc;
        destroyBuffer(allocator, ab);
        surface.materialUBO = VK_NULL_HANDLE;
    }
    surface.materialSet = VK_NULL_HANDLE;
}

// ==============================================================
// Query functions (data-only, no GL)
// ==============================================================

std::optional<float> WaterRenderer::getWaterHeightAt(float glX, float glY) const {
    std::optional<float> best;

    for (const auto& surface : surfaces) {
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 sX(surface.stepX.x, surface.stepX.y);
        glm::vec2 sY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(sX, sX);
        float lenSqY = glm::dot(sY, sY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) continue;
        float gx = glm::dot(rel, sX) / lenSqX;
        float gy = glm::dot(rel, sY) / lenSqY;

        if (gx < 0.0f || gx > static_cast<float>(surface.width) ||
            gy < 0.0f || gy > static_cast<float>(surface.height)) continue;

        int gridWidth = surface.width + 1;
        int ix = static_cast<int>(gx);
        int iy = static_cast<int>(gy);
        float fx = gx - ix;
        float fy = gy - iy;

        if (ix >= surface.width) { ix = surface.width - 1; fx = 1.0f; }
        if (iy >= surface.height) { iy = surface.height - 1; fy = 1.0f; }
        if (ix < 0 || iy < 0) continue;

        if (!surface.mask.empty()) {
            int tileIndex;
            if (surface.wmoId == 0 && surface.width <= 8 && surface.mask.size() >= 8) {
                tileIndex = (static_cast<int>(surface.yOffset) + iy) * 8 +
                            (static_cast<int>(surface.xOffset) + ix);
            } else {
                tileIndex = iy * surface.width + ix;
            }
            int byteIndex = tileIndex / 8;
            int bitIndex = tileIndex % 8;
            if (byteIndex < static_cast<int>(surface.mask.size())) {
                uint8_t maskByte = surface.mask[byteIndex];
                bool renderTile;
                if (surface.wmoId == 0 && surface.width > 8) {
                    renderTile = (maskByte & (1 << bitIndex)) != 0;
                } else {
                    renderTile = (maskByte & (1 << bitIndex)) || (maskByte & (1 << (7 - bitIndex)));
                }
                if (!renderTile) continue;
            }
        }

        int idx00 = iy * gridWidth + ix;
        int idx10 = idx00 + 1;
        int idx01 = idx00 + gridWidth;
        int idx11 = idx01 + 1;

        int total = static_cast<int>(surface.heights.size());
        if (idx11 >= total) continue;

        float h00 = surface.heights[idx00], h10 = surface.heights[idx10];
        float h01 = surface.heights[idx01], h11 = surface.heights[idx11];
        float h = h00*(1-fx)*(1-fy) + h10*fx*(1-fy) + h01*(1-fx)*fy + h11*fx*fy;

        if (!best || h > *best) best = h;
    }

    return best;
}

std::optional<float> WaterRenderer::getNearestWaterHeightAt(float glX, float glY, float queryZ, float maxAbove) const {
    std::optional<float> best;
    float bestDist = 1e9f;

    for (const auto& surface : surfaces) {
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 sX(surface.stepX.x, surface.stepX.y);
        glm::vec2 sY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(sX, sX);
        float lenSqY = glm::dot(sY, sY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) continue;
        float gx = glm::dot(rel, sX) / lenSqX;
        float gy = glm::dot(rel, sY) / lenSqY;

        if (gx < 0.0f || gx > static_cast<float>(surface.width) ||
            gy < 0.0f || gy > static_cast<float>(surface.height)) continue;

        int gridWidth = surface.width + 1;
        int ix = static_cast<int>(gx);
        int iy = static_cast<int>(gy);
        float fx = gx - ix;
        float fy = gy - iy;

        if (ix >= surface.width) { ix = surface.width - 1; fx = 1.0f; }
        if (iy >= surface.height) { iy = surface.height - 1; fy = 1.0f; }
        if (ix < 0 || iy < 0) continue;

        if (!surface.mask.empty()) {
            int tileIndex;
            if (surface.wmoId == 0 && surface.width <= 8 && surface.mask.size() >= 8) {
                tileIndex = (static_cast<int>(surface.yOffset) + iy) * 8 +
                            (static_cast<int>(surface.xOffset) + ix);
            } else {
                tileIndex = iy * surface.width + ix;
            }
            int byteIndex = tileIndex / 8;
            int bitIndex = tileIndex % 8;
            if (byteIndex < static_cast<int>(surface.mask.size())) {
                uint8_t maskByte = surface.mask[byteIndex];
                bool renderTile;
                if (surface.wmoId == 0 && surface.width > 8) {
                    renderTile = (maskByte & (1 << bitIndex)) != 0;
                } else {
                    renderTile = (maskByte & (1 << bitIndex)) || (maskByte & (1 << (7 - bitIndex)));
                }
                if (!renderTile) continue;
            }
        }

        int idx00 = iy * gridWidth + ix;
        int idx10 = idx00 + 1;
        int idx01 = idx00 + gridWidth;
        int idx11 = idx01 + 1;

        int total = static_cast<int>(surface.heights.size());
        if (idx11 >= total) continue;

        float h00 = surface.heights[idx00], h10 = surface.heights[idx10];
        float h01 = surface.heights[idx01], h11 = surface.heights[idx11];
        float h = h00*(1-fx)*(1-fy) + h10*fx*(1-fy) + h01*(1-fx)*fy + h11*fx*fy;

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
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 sX(surface.stepX.x, surface.stepX.y);
        glm::vec2 sY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(sX, sX);
        float lenSqY = glm::dot(sY, sY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) continue;

        float gx = glm::dot(rel, sX) / lenSqX;
        float gy = glm::dot(rel, sY) / lenSqY;
        if (gx < 0.0f || gx > static_cast<float>(surface.width) ||
            gy < 0.0f || gy > static_cast<float>(surface.height)) continue;

        int ix = static_cast<int>(gx);
        int iy = static_cast<int>(gy);
        if (ix >= surface.width) ix = surface.width - 1;
        if (iy >= surface.height) iy = surface.height - 1;
        if (ix < 0 || iy < 0) continue;

        if (!surface.mask.empty()) {
            int tileIndex;
            if (surface.wmoId == 0 && surface.width <= 8 && surface.mask.size() >= 8) {
                tileIndex = (static_cast<int>(surface.yOffset) + iy) * 8 +
                            (static_cast<int>(surface.xOffset) + ix);
            } else {
                tileIndex = iy * surface.width + ix;
            }
            int byteIndex = tileIndex / 8;
            int bitIndex = tileIndex % 8;
            if (byteIndex < static_cast<int>(surface.mask.size())) {
                uint8_t maskByte = surface.mask[byteIndex];
                bool renderTile;
                if (surface.wmoId == 0 && surface.width > 8) {
                    renderTile = (maskByte & (1 << bitIndex)) != 0;
                } else {
                    renderTile = (maskByte & (1 << bitIndex)) || (maskByte & (1 << (7 - bitIndex)));
                }
                if (!renderTile) continue;
            }
        }

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
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 sX(surface.stepX.x, surface.stepX.y);
        glm::vec2 sY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(sX, sX);
        float lenSqY = glm::dot(sY, sY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) continue;
        float gx = glm::dot(rel, sX) / lenSqX;
        float gy = glm::dot(rel, sY) / lenSqY;
        if (gx >= 0.0f && gx <= static_cast<float>(surface.width) &&
            gy >= 0.0f && gy <= static_cast<float>(surface.height))
            return true;
    }
    return false;
}

glm::vec4 WaterRenderer::getLiquidColor(uint16_t liquidType) const {
    uint8_t basicType = (liquidType == 0) ? 0 : ((liquidType - 1) % 4);
    switch (basicType) {
        case 0:  return glm::vec4(0.12f, 0.32f, 0.48f, 1.0f); // inland: blue-green
        case 1:  return glm::vec4(0.04f, 0.14f, 0.30f, 1.0f); // ocean: deep blue
        case 2:  return glm::vec4(0.9f, 0.3f, 0.05f, 1.0f);   // magma
        case 3:  return glm::vec4(0.2f, 0.6f, 0.1f, 1.0f);    // slime
        default: return glm::vec4(0.12f, 0.32f, 0.48f, 1.0f);
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
    colorImgCI.extent = {REFLECTION_WIDTH, REFLECTION_HEIGHT, 1};
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
    colorViewCI.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (vkCreateImageView(device, &colorViewCI, nullptr, &reflectionColorView) != VK_SUCCESS) {
        LOG_ERROR("WaterRenderer: failed to create reflection color view");
        return;
    }

    // --- Reflection depth image ---
    VkImageCreateInfo depthImgCI{};
    depthImgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    depthImgCI.imageType = VK_IMAGE_TYPE_2D;
    depthImgCI.format = vkCtx->getDepthFormat();
    depthImgCI.extent = {REFLECTION_WIDTH, REFLECTION_HEIGHT, 1};
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
    depthViewCI.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
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
    if (vkCreateSampler(device, &sampCI, nullptr, &reflectionSampler) != VK_SUCCESS) {
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

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
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
    bufCI.size = sizeof(ReflectionUBOData);
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
    ReflectionUBOData initData{};
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
        barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
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
    if (reflectionSampler) { vkDestroySampler(device, reflectionSampler, nullptr); reflectionSampler = VK_NULL_HANDLE; }
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
    rpInfo.renderArea = {{0, 0}, {REFLECTION_WIDTH, REFLECTION_HEIGHT}};

    VkClearValue clears[2]{};
    clears[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clears[1].depthStencil = {1.0f, 0};
    rpInfo.clearValueCount = 2;
    rpInfo.pClearValues = clears;

    vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{0, 0, static_cast<float>(REFLECTION_WIDTH), static_cast<float>(REFLECTION_HEIGHT), 0.0f, 1.0f};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    VkRect2D sc{{0, 0}, {REFLECTION_WIDTH, REFLECTION_HEIGHT}};
    vkCmdSetScissor(cmd, 0, 1, &sc);

    return true;
}

void WaterRenderer::endReflectionPass(VkCommandBuffer cmd) {
    if (!cmd) return;
    vkCmdEndRenderPass(cmd);
    reflectionColorLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // Update scene descriptor set with the freshly rendered reflection texture
    if (sceneSet && reflectionColorView && reflectionSampler) {
        VkDescriptorImageInfo reflInfo{};
        reflInfo.sampler = reflectionSampler;
        reflInfo.imageView = reflectionColorView;
        reflInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = sceneSet;
        write.dstBinding = 2;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &reflInfo;
        vkUpdateDescriptorSets(vkCtx->getDevice(), 1, &write, 0, nullptr);
    }
}

void WaterRenderer::updateReflectionUBO(const glm::mat4& reflViewProj) {
    if (!reflectionUBOMapped) return;
    ReflectionUBOData data{};
    data.reflViewProj = reflViewProj;
    std::memcpy(reflectionUBOMapped, &data, sizeof(data));
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
        // Skip magma/slime — only reflect water/ocean
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

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

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
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

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
    if (water1xPipeline) { vkDestroyPipeline(device, water1xPipeline, nullptr); water1xPipeline = VK_NULL_HANDLE; }
    if (water1xRenderPass) { vkDestroyRenderPass(device, water1xRenderPass, nullptr); water1xRenderPass = VK_NULL_HANDLE; }
}

bool WaterRenderer::beginWater1xPass(VkCommandBuffer cmd, uint32_t imageIndex, VkExtent2D extent) {
    if (!water1xRenderPass || imageIndex >= water1xFramebuffers.size() || !water1xFramebuffers[imageIndex])
        return false;

    VkRenderPassBeginInfo rpBI{};
    rpBI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBI.renderPass = water1xRenderPass;
    rpBI.framebuffer = water1xFramebuffers[imageIndex];
    rpBI.renderArea = {{0, 0}, extent};
    rpBI.clearValueCount = 0;
    rpBI.pClearValues = nullptr;
    vkCmdBeginRenderPass(cmd, &rpBI, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{0, 0, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetScissor(cmd, 0, 1, &sc);

    return true;
}

void WaterRenderer::endWater1xPass(VkCommandBuffer cmd) {
    vkCmdEndRenderPass(cmd);
}

} // namespace rendering
} // namespace wowee
