/**
 * CharacterRenderer — GPU rendering of M2 character models with skeletal animation (Vulkan)
 *
 * Handles:
 *  - Uploading M2 vertex/index data to Vulkan buffers via VMA
 *  - Per-frame bone matrix computation (hierarchical, with keyframe interpolation)
 *  - GPU vertex skinning via a bone-matrix SSBO in the vertex shader
 *  - Per-batch texture binding through the M2 texture-lookup indirection
 *  - Geoset filtering (activeGeosets) to show/hide body part groups
 *  - CPU texture compositing for character skins (base skin + underwear overlays)
 *
 * The character texture compositing uses the WoW CharComponentTextureSections
 * layout, placing region overlays (pelvis, torso, etc.) at their correct pixel
 * positions on the 512x512 body skin atlas. Region coordinates sourced from
 * the original WoW Model Viewer (charcontrol.h, REGION_FAC=2).
 */
#include "rendering/character_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <future>
#include <thread>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <cstring>

namespace wowee {
namespace rendering {

namespace {
size_t envSizeMBOrDefault(const char* name, size_t defMb) {
    const char* v = std::getenv(name);
    if (!v || !*v) return defMb;
    char* end = nullptr;
    unsigned long long mb = std::strtoull(v, &end, 10);
    if (end == v || mb == 0) return defMb;
    if (mb > (std::numeric_limits<size_t>::max() / (1024ull * 1024ull))) return defMb;
    return static_cast<size_t>(mb);
}

size_t envSizeOrDefault(const char* name, size_t defValue) {
    const char* v = std::getenv(name);
    if (!v || !*v) return defValue;
    char* end = nullptr;
    unsigned long long n = std::strtoull(v, &end, 10);
    if (end == v || n == 0) return defValue;
    return static_cast<size_t>(n);
}

size_t approxTextureBytesWithMips(int w, int h) {
    if (w <= 0 || h <= 0) return 0;
    size_t base = static_cast<size_t>(w) * static_cast<size_t>(h) * 4ull;
    return base + (base / 3);  // ~4/3 for mip chain
}
} // namespace

// Descriptor pool sizing
static constexpr uint32_t MAX_MATERIAL_SETS = 4096;
static constexpr uint32_t MAX_BONE_SETS = 8192;

// CharMaterial UBO layout (matches character.frag.glsl set=1 binding=1)
struct CharMaterialUBO {
    float opacity;
    int32_t alphaTest;
    int32_t colorKeyBlack;
    int32_t unlit;
    float emissiveBoost;
    float emissiveTintR, emissiveTintG, emissiveTintB;
    float specularIntensity;
    int32_t enableNormalMap;
    int32_t enablePOM;
    float pomScale;
    int32_t pomMaxSamples;
    float heightMapVariance;
    float normalMapStrength;
    float _pad[2]; // pad to 64 bytes
};

// GPU vertex struct with tangent (expanded from M2Vertex for normal mapping)
struct CharVertexGPU {
    glm::vec3 position;      // 12 bytes, offset 0
    uint8_t boneWeights[4];  // 4 bytes,  offset 12
    uint8_t boneIndices[4];  // 4 bytes,  offset 16
    glm::vec3 normal;        // 12 bytes, offset 20
    glm::vec2 texCoords;     // 8 bytes,  offset 32
    glm::vec4 tangent;       // 16 bytes, offset 40 (xyz=dir, w=handedness)
};  // 56 bytes total

CharacterRenderer::CharacterRenderer() {
}

CharacterRenderer::~CharacterRenderer() {
    shutdown();
}

bool CharacterRenderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                                    pipeline::AssetManager* am,
                                    VkRenderPass renderPassOverride,
                                    VkSampleCountFlagBits msaaSamples) {
    core::Logger::getInstance().info("Initializing character renderer (Vulkan)...");

    vkCtx_ = ctx;
    assetManager = am;
    perFrameLayout_ = perFrameLayout;
    renderPassOverride_ = renderPassOverride;
    msaaSamplesOverride_ = msaaSamples;
    const unsigned hc = std::thread::hardware_concurrency();
    const size_t availableCores = (hc > 1u) ? static_cast<size_t>(hc - 1u) : 1ull;
    // Character updates run alongside M2/WMO work; default to a smaller share.
    const size_t defaultAnimThreads = std::max<size_t>(1, availableCores / 4);
    numAnimThreads_ = static_cast<uint32_t>(std::max<size_t>(
        1, envSizeOrDefault("WOWEE_CHAR_ANIM_THREADS", defaultAnimThreads)));
    core::Logger::getInstance().info("Character anim threads: ", numAnimThreads_);

    VkDevice device = vkCtx_->getDevice();

    // --- Descriptor set layouts ---

    // Material set layout (set 1): binding 0 = sampler2D, binding 1 = CharMaterial UBO, binding 2 = normal/height map
    {
        VkDescriptorSetLayoutBinding bindings[3] = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[2].binding = 2;
        bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[2].descriptorCount = 1;
        bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 3;
        ci.pBindings = bindings;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &materialSetLayout_);
    }

    // Bone set layout (set 2): binding 0 = STORAGE_BUFFER (bone matrices)
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 1;
        ci.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &boneSetLayout_);
    }

    // --- Descriptor pools ---
    // Material descriptors are transient and allocated every draw; keep per-frame
    // pools so we can reset safely each frame slot without exhausting descriptors.
    for (int i = 0; i < 2; i++) {
        VkDescriptorPoolSize sizes[] = {
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_MATERIAL_SETS * 2},  // diffuse + normal/height
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_MATERIAL_SETS},
        };
        VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets = MAX_MATERIAL_SETS;
        ci.poolSizeCount = 2;
        ci.pPoolSizes = sizes;
        ci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        vkCreateDescriptorPool(device, &ci, nullptr, &materialDescPools_[i]);
    }
    {
        VkDescriptorPoolSize sizes[] = {
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, MAX_BONE_SETS},
        };
        VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets = MAX_BONE_SETS;
        ci.poolSizeCount = 1;
        ci.pPoolSizes = sizes;
        ci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        vkCreateDescriptorPool(device, &ci, nullptr, &boneDescPool_);
    }

    // --- Pipeline layout ---
    // set 0 = perFrame, set 1 = material, set 2 = bones
    // Push constant: mat4 model = 64 bytes
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout, materialSetLayout_, boneSetLayout_};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = 64; // mat4

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 3;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &pipelineLayout_);
    }

    // --- Load shaders ---
    rendering::VkShaderModule charVert, charFrag;
    charVert.loadFromFile(device, "assets/shaders/character.vert.spv");
    charFrag.loadFromFile(device, "assets/shaders/character.frag.spv");

    if (!charVert.isValid() || !charFrag.isValid()) {
        LOG_ERROR("Character: Missing required shaders, cannot initialize");
        return false;
    }

    VkRenderPass mainPass = renderPassOverride_ ? renderPassOverride_ : vkCtx_->getImGuiRenderPass();
    VkSampleCountFlagBits samples = renderPassOverride_ ? msaaSamplesOverride_ : vkCtx_->getMsaaSamples();

    // --- Vertex input ---
    // CharVertexGPU: vec3 pos(12) + uint8[4] boneWeights(4) + uint8[4] boneIndices(4) +
    //               vec3 normal(12) + vec2 texCoords(8) + vec4 tangent(16) = 56 bytes
    VkVertexInputBindingDescription charBinding{};
    charBinding.binding = 0;
    charBinding.stride = sizeof(CharVertexGPU);
    charBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> charAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(CharVertexGPU, position))},
        {1, 0, VK_FORMAT_R8G8B8A8_UNORM,   static_cast<uint32_t>(offsetof(CharVertexGPU, boneWeights))},
        {2, 0, VK_FORMAT_R8G8B8A8_UINT,     static_cast<uint32_t>(offsetof(CharVertexGPU, boneIndices))},
        {3, 0, VK_FORMAT_R32G32B32_SFLOAT,  static_cast<uint32_t>(offsetof(CharVertexGPU, normal))},
        {4, 0, VK_FORMAT_R32G32_SFLOAT,     static_cast<uint32_t>(offsetof(CharVertexGPU, texCoords))},
        {5, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(CharVertexGPU, tangent))},
    };

    // --- Build pipelines ---
    auto buildCharPipeline = [&](VkPipelineColorBlendAttachmentState blendState, bool depthWrite) -> VkPipeline {
        return PipelineBuilder()
            .setShaders(charVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        charFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({charBinding}, charAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, depthWrite, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(blendState)
            .setMultisample(samples)
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    };

    opaquePipeline_ = buildCharPipeline(PipelineBuilder::blendDisabled(), true);
    alphaTestPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), true);
    alphaPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), false);
    additivePipeline_ = buildCharPipeline(PipelineBuilder::blendAdditive(), false);

    // Clean up shader modules
    charVert.destroy();
    charFrag.destroy();

    // --- Create white fallback texture ---
    {
        uint8_t white[] = {255, 255, 255, 255};
        whiteTexture_ = std::make_unique<VkTexture>();
        whiteTexture_->upload(*vkCtx_, white, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        whiteTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }

    // --- Create transparent fallback texture ---
    {
        uint8_t transparent[] = {0, 0, 0, 0};
        transparentTexture_ = std::make_unique<VkTexture>();
        transparentTexture_->upload(*vkCtx_, transparent, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        transparentTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }

    // --- Create flat normal placeholder texture (128,128,255,128) = neutral normal, 0.5 height ---
    {
        uint8_t flatNormal[] = {128, 128, 255, 128};
        flatNormalTexture_ = std::make_unique<VkTexture>();
        flatNormalTexture_->upload(*vkCtx_, flatNormal, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        flatNormalTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }

    // Diagnostics-only: cache lifetime is currently tied to renderer lifetime.
    textureCacheBudgetBytes_ = envSizeMBOrDefault("WOWEE_CHARACTER_TEX_CACHE_MB", 4096) * 1024ull * 1024ull;
    LOG_INFO("Character texture cache budget: ", textureCacheBudgetBytes_ / (1024 * 1024), " MB");

    core::Logger::getInstance().info("Character renderer initialized (Vulkan)");
    return true;
}

void CharacterRenderer::shutdown() {
    if (!vkCtx_) return;

    LOG_INFO("CharacterRenderer::shutdown instances=", instances.size(),
             " models=", models.size(), " override=", (void*)renderPassOverride_);

    vkDeviceWaitIdle(vkCtx_->getDevice());
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();

    // Clean up GPU resources for models
    for (auto& pair : models) {
        destroyModelGPU(pair.second);
    }

    // Clean up instance bone buffers
    for (auto& pair : instances) {
        destroyInstanceBones(pair.second);
    }

    // Clean up texture cache (VkTexture unique_ptrs auto-destroy)
    textureCache.clear();
    textureHasAlphaByPtr_.clear();
    textureColorKeyBlackByPtr_.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;

    // Clean up composite cache
    compositeCache_.clear();
    failedTextureCache_.clear();

    whiteTexture_.reset();
    transparentTexture_.reset();
    flatNormalTexture_.reset();

    models.clear();
    instances.clear();

    // Destroy pipelines
    auto destroyPipeline = [&](VkPipeline& p) {
        if (p) { vkDestroyPipeline(device, p, nullptr); p = VK_NULL_HANDLE; }
    };
    destroyPipeline(opaquePipeline_);
    destroyPipeline(alphaTestPipeline_);
    destroyPipeline(alphaPipeline_);
    destroyPipeline(additivePipeline_);

    if (pipelineLayout_) { vkDestroyPipelineLayout(device, pipelineLayout_, nullptr); pipelineLayout_ = VK_NULL_HANDLE; }

    // Release any deferred transient material UBOs.
    for (int i = 0; i < 2; i++) {
        for (const auto& b : transientMaterialUbos_[i]) {
            if (b.first) {
                vmaDestroyBuffer(alloc, b.first, b.second);
            }
        }
        transientMaterialUbos_[i].clear();
    }

    // Destroy descriptor pools and layouts
    for (int i = 0; i < 2; i++) {
        if (materialDescPools_[i]) {
            vkDestroyDescriptorPool(device, materialDescPools_[i], nullptr);
            materialDescPools_[i] = VK_NULL_HANDLE;
        }
    }
    if (boneDescPool_) { vkDestroyDescriptorPool(device, boneDescPool_, nullptr); boneDescPool_ = VK_NULL_HANDLE; }
    if (materialSetLayout_) { vkDestroyDescriptorSetLayout(device, materialSetLayout_, nullptr); materialSetLayout_ = VK_NULL_HANDLE; }
    if (boneSetLayout_) { vkDestroyDescriptorSetLayout(device, boneSetLayout_, nullptr); boneSetLayout_ = VK_NULL_HANDLE; }

    // Shadow resources
    if (shadowPipeline_) { vkDestroyPipeline(device, shadowPipeline_, nullptr); shadowPipeline_ = VK_NULL_HANDLE; }
    if (shadowPipelineLayout_) { vkDestroyPipelineLayout(device, shadowPipelineLayout_, nullptr); shadowPipelineLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsPool_) { vkDestroyDescriptorPool(device, shadowParamsPool_, nullptr); shadowParamsPool_ = VK_NULL_HANDLE; }
    if (shadowParamsLayout_) { vkDestroyDescriptorSetLayout(device, shadowParamsLayout_, nullptr); shadowParamsLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsUBO_) { vmaDestroyBuffer(alloc, shadowParamsUBO_, shadowParamsAlloc_); shadowParamsUBO_ = VK_NULL_HANDLE; shadowParamsAlloc_ = VK_NULL_HANDLE; }

    vkCtx_ = nullptr;
}

void CharacterRenderer::destroyModelGPU(M2ModelGPU& gpuModel) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();
    if (gpuModel.vertexBuffer) { vmaDestroyBuffer(alloc, gpuModel.vertexBuffer, gpuModel.vertexAlloc); gpuModel.vertexBuffer = VK_NULL_HANDLE; }
    if (gpuModel.indexBuffer) { vmaDestroyBuffer(alloc, gpuModel.indexBuffer, gpuModel.indexAlloc); gpuModel.indexBuffer = VK_NULL_HANDLE; }
}

void CharacterRenderer::destroyInstanceBones(CharacterInstance& inst) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();
    VkDevice device = vkCtx_->getDevice();
    for (int i = 0; i < 2; i++) {
        if (inst.boneSet[i] != VK_NULL_HANDLE && boneDescPool_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device, boneDescPool_, 1, &inst.boneSet[i]);
            inst.boneSet[i] = VK_NULL_HANDLE;
        }
        if (inst.boneBuffer[i]) {
            vmaDestroyBuffer(alloc, inst.boneBuffer[i], inst.boneAlloc[i]);
            inst.boneBuffer[i] = VK_NULL_HANDLE;
            inst.boneAlloc[i] = VK_NULL_HANDLE;
            inst.boneMapped[i] = nullptr;
        }
    }
}

std::unique_ptr<VkTexture> CharacterRenderer::generateNormalHeightMap(
        const uint8_t* pixels, uint32_t width, uint32_t height, float& outVariance) {
    if (!vkCtx_ || width == 0 || height == 0) return nullptr;

    const uint32_t totalPixels = width * height;

    // Step 1: Compute height from luminance
    std::vector<float> heightMap(totalPixels);
    double sumH = 0.0, sumH2 = 0.0;
    for (uint32_t i = 0; i < totalPixels; i++) {
        float r = pixels[i * 4 + 0] / 255.0f;
        float g = pixels[i * 4 + 1] / 255.0f;
        float b = pixels[i * 4 + 2] / 255.0f;
        float h = 0.299f * r + 0.587f * g + 0.114f * b;
        heightMap[i] = h;
        sumH += h;
        sumH2 += h * h;
    }
    double mean = sumH / totalPixels;
    outVariance = static_cast<float>(sumH2 / totalPixels - mean * mean);

    // Step 1.5: Box blur the height map to reduce noise from diffuse textures
    auto wrapSample = [&](const std::vector<float>& map, int x, int y) -> float {
        x = ((x % (int)width) + (int)width) % (int)width;
        y = ((y % (int)height) + (int)height) % (int)height;
        return map[y * width + x];
    };

    std::vector<float> blurredHeight(totalPixels);
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            int ix = static_cast<int>(x), iy = static_cast<int>(y);
            float sum = 0.0f;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    sum += wrapSample(heightMap, ix + dx, iy + dy);
            blurredHeight[y * width + x] = sum / 9.0f;
        }
    }

    // Step 2: Sobel 3x3 → normal map (crisp detail from original, blurred for POM alpha)
    // Higher strength than WMO (2.0) because character/weapon textures are hand-painted
    // with baked-in lighting that produces low-contrast gradients in the Sobel filter.
    const float strength = 5.0f;
    std::vector<uint8_t> output(totalPixels * 4);

    auto sampleH = [&](int x, int y) -> float {
        x = ((x % (int)width) + (int)width) % (int)width;
        y = ((y % (int)height) + (int)height) % (int)height;
        return heightMap[y * width + x];
    };

    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            int ix = static_cast<int>(x);
            int iy = static_cast<int>(y);
            float gx = -sampleH(ix-1, iy-1) - 2.0f*sampleH(ix-1, iy) - sampleH(ix-1, iy+1)
                       + sampleH(ix+1, iy-1) + 2.0f*sampleH(ix+1, iy) + sampleH(ix+1, iy+1);
            float gy = -sampleH(ix-1, iy-1) - 2.0f*sampleH(ix, iy-1) - sampleH(ix+1, iy-1)
                       + sampleH(ix-1, iy+1) + 2.0f*sampleH(ix, iy+1) + sampleH(ix+1, iy+1);

            float nx = -gx * strength;
            float ny = -gy * strength;
            float nz = 1.0f;
            float len = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (len > 0.0f) { nx /= len; ny /= len; nz /= len; }

            uint32_t idx = (y * width + x) * 4;
            output[idx + 0] = static_cast<uint8_t>(std::clamp((nx * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f));
            output[idx + 1] = static_cast<uint8_t>(std::clamp((ny * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f));
            output[idx + 2] = static_cast<uint8_t>(std::clamp((nz * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f));
            output[idx + 3] = static_cast<uint8_t>(std::clamp(blurredHeight[y * width + x] * 255.0f, 0.0f, 255.0f));
        }
    }

    auto tex = std::make_unique<VkTexture>();
    if (!tex->upload(*vkCtx_, output.data(), width, height, VK_FORMAT_R8G8B8A8_UNORM, true)) {
        return nullptr;
    }
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                        VK_SAMPLER_ADDRESS_MODE_REPEAT);
    return tex;
}

VkTexture* CharacterRenderer::loadTexture(const std::string& path) {
    // Skip empty or whitespace-only paths (type-0 textures have no filename)
    if (path.empty()) return whiteTexture_.get();
    bool allWhitespace = true;
    for (char c : path) {
        if (c != ' ' && c != '\t' && c != '\0' && c != '\n') { allWhitespace = false; break; }
    }
    if (allWhitespace) return whiteTexture_.get();

    auto normalizeKey = [](std::string key) {
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    };
    std::string key = normalizeKey(path);
    auto containsToken = [](const std::string& haystack, const char* token) {
        return haystack.find(token) != std::string::npos;
    };
    const bool colorKeyBlackHint =
        containsToken(key, "candle") ||
        containsToken(key, "flame") ||
        containsToken(key, "fire") ||
        containsToken(key, "torch");

    // Check cache
    auto it = textureCache.find(key);
    if (it != textureCache.end()) {
        it->second.lastUse = ++textureCacheCounter_;
        return it->second.texture.get();
    }

    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture_.get();
    }

    auto blpImage = assetManager->loadTexture(key);
    if (!blpImage.isValid()) {
        // Return white fallback but don't cache the failure — allow retry
        // on next character load in case the asset becomes available.
        if (loggedTextureLoadFails_.insert(key).second) {
            core::Logger::getInstance().warning("Failed to load texture: ", path);
        }
        return whiteTexture_.get();
    }

    size_t approxBytes = approxTextureBytesWithMips(blpImage.width, blpImage.height);
    if (textureCacheBytes_ + approxBytes > textureCacheBudgetBytes_) {
        static constexpr size_t kMaxFailedTextureCache = 200000;
        if (failedTextureCache_.size() < kMaxFailedTextureCache) {
            // Budget is saturated; avoid repeatedly decoding/uploading this texture.
            failedTextureCache_.insert(key);
        }
        if (textureBudgetRejectWarnings_ < 3) {
            core::Logger::getInstance().warning(
                "Character texture cache full (",
                textureCacheBytes_ / (1024 * 1024), " MB / ",
                textureCacheBudgetBytes_ / (1024 * 1024), " MB), rejecting texture: ",
                path);
        }
        ++textureBudgetRejectWarnings_;
        return whiteTexture_.get();
    }

    bool hasAlpha = false;
    for (size_t i = 3; i < blpImage.data.size(); i += 4) {
        if (blpImage.data[i] != 255) {
            hasAlpha = true;
            break;
        }
    }

    auto tex = std::make_unique<VkTexture>();
    tex->upload(*vkCtx_, blpImage.data.data(), blpImage.width, blpImage.height,
                VK_FORMAT_R8G8B8A8_UNORM, true);
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                       VK_SAMPLER_ADDRESS_MODE_REPEAT);

    VkTexture* texPtr = tex.get();

    TextureCacheEntry e;
    e.texture = std::move(tex);
    e.approxBytes = approxBytes;
    e.lastUse = ++textureCacheCounter_;
    e.hasAlpha = hasAlpha;
    e.colorKeyBlack = colorKeyBlackHint;

    // Generate normal/height map from diffuse texture
    float nhVariance = 0.0f;
    auto nhMap = generateNormalHeightMap(blpImage.data.data(), blpImage.width, blpImage.height, nhVariance);
    if (nhMap) {
        e.heightMapVariance = nhVariance;
        e.approxBytes += approxTextureBytesWithMips(blpImage.width, blpImage.height);
        e.normalHeightMap = std::move(nhMap);
    }

    textureCacheBytes_ += e.approxBytes;
    textureHasAlphaByPtr_[texPtr] = hasAlpha;
    textureColorKeyBlackByPtr_[texPtr] = colorKeyBlackHint;
    textureCache[key] = std::move(e);

    core::Logger::getInstance().debug("Loaded character texture: ", path, " (", blpImage.width, "x", blpImage.height, ")");
    return texPtr;
}

// Alpha-blend overlay onto composite at (dstX, dstY)
static void blitOverlay(std::vector<uint8_t>& composite, int compW, int compH,
                         const pipeline::BLPImage& overlay, int dstX, int dstY) {
    for (int sy = 0; sy < overlay.height; sy++) {
        int dy = dstY + sy;
        if (dy < 0 || dy >= compH) continue;
        for (int sx = 0; sx < overlay.width; sx++) {
            int dx = dstX + sx;
            if (dx < 0 || dx >= compW) continue;

            size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            size_t dstIdx = (static_cast<size_t>(dy) * compW + dx) * 4;

            uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            if (srcA == 255) {
                composite[dstIdx + 0] = overlay.data[srcIdx + 0];
                composite[dstIdx + 1] = overlay.data[srcIdx + 1];
                composite[dstIdx + 2] = overlay.data[srcIdx + 2];
                composite[dstIdx + 3] = 255;
            } else {
                float alpha = srcA / 255.0f;
                float invAlpha = 1.0f - alpha;
                composite[dstIdx + 0] = static_cast<uint8_t>(overlay.data[srcIdx + 0] * alpha + composite[dstIdx + 0] * invAlpha);
                composite[dstIdx + 1] = static_cast<uint8_t>(overlay.data[srcIdx + 1] * alpha + composite[dstIdx + 1] * invAlpha);
                composite[dstIdx + 2] = static_cast<uint8_t>(overlay.data[srcIdx + 2] * alpha + composite[dstIdx + 2] * invAlpha);
                composite[dstIdx + 3] = std::max(composite[dstIdx + 3], srcA);
            }
        }
    }
}

// Nearest-neighbor NxN scale blit of overlay onto composite at (dstX, dstY)
static void blitOverlayScaledN(std::vector<uint8_t>& composite, int compW, int compH,
                                const pipeline::BLPImage& overlay, int dstX, int dstY, int scale) {
    if (scale < 1) scale = 1;
    for (int sy = 0; sy < overlay.height; sy++) {
        for (int sx = 0; sx < overlay.width; sx++) {
            size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            // Write to scale x scale block of destination pixels
            for (int dy2 = 0; dy2 < scale; dy2++) {
                int dy = dstY + sy * scale + dy2;
                if (dy < 0 || dy >= compH) continue;
                for (int dx2 = 0; dx2 < scale; dx2++) {
                    int dx = dstX + sx * scale + dx2;
                    if (dx < 0 || dx >= compW) continue;

                    size_t dstIdx = (static_cast<size_t>(dy) * compW + dx) * 4;
                    if (srcA == 255) {
                        composite[dstIdx + 0] = overlay.data[srcIdx + 0];
                        composite[dstIdx + 1] = overlay.data[srcIdx + 1];
                        composite[dstIdx + 2] = overlay.data[srcIdx + 2];
                        composite[dstIdx + 3] = 255;
                    } else {
                        float alpha = srcA / 255.0f;
                        float invAlpha = 1.0f - alpha;
                        composite[dstIdx + 0] = static_cast<uint8_t>(overlay.data[srcIdx + 0] * alpha + composite[dstIdx + 0] * invAlpha);
                        composite[dstIdx + 1] = static_cast<uint8_t>(overlay.data[srcIdx + 1] * alpha + composite[dstIdx + 1] * invAlpha);
                        composite[dstIdx + 2] = static_cast<uint8_t>(overlay.data[srcIdx + 2] * alpha + composite[dstIdx + 2] * invAlpha);
                        composite[dstIdx + 3] = std::max(composite[dstIdx + 3], srcA);
                    }
                }
            }
        }
    }
}

// Legacy 2x wrapper
static void blitOverlayScaled2x(std::vector<uint8_t>& composite, int compW, int compH,
                                 const pipeline::BLPImage& overlay, int dstX, int dstY) {
    blitOverlayScaledN(composite, compW, compH, overlay, dstX, dstY, 2);
}

VkTexture* CharacterRenderer::compositeTextures(const std::vector<std::string>& layerPaths) {
    if (layerPaths.empty() || !assetManager || !assetManager->isInitialized()) {
        return whiteTexture_.get();
    }

    // Composite key is deterministic from layer set; if we've already built it,
    // reuse the existing GPU texture to keep live instance pointers valid.
    std::string cacheKey = "__composite__";
    for (const auto& lp : layerPaths) { cacheKey += '|'; cacheKey += lp; }
    auto cachedComposite = textureCache.find(cacheKey);
    if (cachedComposite != textureCache.end()) {
        cachedComposite->second.lastUse = ++textureCacheCounter_;
        return cachedComposite->second.texture.get();
    }

    // Load base layer
    auto base = assetManager->loadTexture(layerPaths[0]);
    if (!base.isValid()) {
        core::Logger::getInstance().warning("Composite: failed to load base layer: ", layerPaths[0]);
        return whiteTexture_.get();
    }

    // Copy base pixel data as our working buffer
    std::vector<uint8_t> composite = base.data;
    int width = base.width;
    int height = base.height;

    core::Logger::getInstance().info("Composite: base layer ", width, "x", height, " from ", layerPaths[0]);

    // WoW character texture atlas regions (from WoW Model Viewer / CharComponentTextureSections)
    // Coordinates at 256x256 base resolution:
    // Region          X    Y    W    H
    // Base            0    0    256  256
    // Arm Upper       0    0    128  64
    // Arm Lower       0    64   128  64
    // Hand            0    128  128  32
    // Face Upper      0    160  128  32
    // Face Lower      0    192  128  64
    // Torso Upper     128  0    128  64
    // Torso Lower     128  64   128  32
    // Pelvis Upper    128  96   128  64
    // Pelvis Lower    128  160  128  64
    // Foot            128  224  128  32

    // Scale factor: base texture may be larger than the 256x256 reference atlas
    int coordScale = width / 256;
    if (coordScale < 1) coordScale = 1;

    // Atlas region sizes at 256x256 base (w, h) for known regions
    struct AtlasRegion { int x, y, w, h; };
    static const AtlasRegion faceLowerRegion256 = {0, 192, 128, 64};
    static const AtlasRegion faceUpperRegion256 = {0, 160, 128, 32};

    // Alpha-blend each overlay onto the composite
    for (size_t layer = 1; layer < layerPaths.size(); layer++) {
        if (layerPaths[layer].empty()) continue;

        auto overlay = assetManager->loadTexture(layerPaths[layer]);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("Composite: FAILED to load overlay: ", layerPaths[layer]);
            continue;
        }

        core::Logger::getInstance().info("Composite: overlay ", layerPaths[layer],
            " (", overlay.width, "x", overlay.height, ")");

        if (overlay.width == width && overlay.height == height) {
            // Same size: full alpha-blend
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // Determine region by filename keywords
            // Coordinates scale with base texture size (256x256 is reference)
            int dstX = 0, dstY = 0;
            int expectedW256 = 0, expectedH256 = 0; // Expected size at 256-base
            std::string pathLower = layerPaths[layer];
            for (auto& c : pathLower) c = std::tolower(c);

            if (pathLower.find("faceupper") != std::string::npos) {
                dstX = faceUpperRegion256.x; dstY = faceUpperRegion256.y;
                expectedW256 = faceUpperRegion256.w; expectedH256 = faceUpperRegion256.h;
            } else if (pathLower.find("facelower") != std::string::npos) {
                dstX = faceLowerRegion256.x; dstY = faceLowerRegion256.y;
                expectedW256 = faceLowerRegion256.w; expectedH256 = faceLowerRegion256.h;
            } else if (pathLower.find("pelvis") != std::string::npos) {
                dstX = 128; dstY = 96;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("torso") != std::string::npos) {
                dstX = 128; dstY = 0;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 64;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 128;
                expectedW256 = 128; expectedH256 = 32;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 128; dstY = 224;
                expectedW256 = 128; expectedH256 = 32;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 128; dstY = 160;
                expectedW256 = 128; expectedH256 = 64;
            } else {
                // Unknown -- center placement as fallback
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
                core::Logger::getInstance().info("Composite: UNKNOWN region for '",
                    layerPaths[layer], "', centering at (", dstX, ",", dstY, ")");
                blitOverlay(composite, width, height, overlay, dstX, dstY);
                continue;
            }

            // Scale coordinates from 256-base to actual canvas
            dstX *= coordScale;
            dstY *= coordScale;

            // If overlay is 256-base sized but canvas is larger, scale the overlay up
            int expectedW = expectedW256 * coordScale;
            int expectedH = expectedH256 * coordScale;
            bool needsScale = (coordScale > 1 &&
                               overlay.width == expectedW256 && overlay.height == expectedH256);

            core::Logger::getInstance().info("Composite: placing '", layerPaths[layer],
                "' (", overlay.width, "x", overlay.height,
                ") at (", dstX, ",", dstY, ") on ", width, "x", height,
                " expected=", expectedW, "x", expectedH,
                needsScale ? " [SCALING]" : "");

            if (needsScale) {
                blitOverlayScaledN(composite, width, height, overlay, dstX, dstY, coordScale);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        }
    }

    // Debug: dump composite to temp dir for visual inspection
    {
        std::string dumpPath = (std::filesystem::temp_directory_path() / ("wowee_composite_debug_" +
            std::to_string(width) + "x" + std::to_string(height) + ".raw")).string();
        std::ofstream dump(dumpPath, std::ios::binary);
        if (dump) {
            dump.write(reinterpret_cast<const char*>(composite.data()),
                       static_cast<std::streamsize>(composite.size()));
            core::Logger::getInstance().info("Composite debug dump: ", dumpPath,
                " (", width, "x", height, ", ", composite.size(), " bytes)");
        }
    }

    // Upload composite to GPU via VkTexture
    auto tex = std::make_unique<VkTexture>();
    tex->upload(*vkCtx_, composite.data(), width, height, VK_FORMAT_R8G8B8A8_UNORM, true);
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                       VK_SAMPLER_ADDRESS_MODE_REPEAT);

    VkTexture* texPtr = tex.get();

    // Store in texture cache with deterministic key.
    // Keep the first allocation for a key to avoid invalidating raw pointers
    // held by active render instances.
    TextureCacheEntry e;
    e.texture = std::move(tex);
    e.approxBytes = approxTextureBytesWithMips(width, height);
    e.lastUse = ++textureCacheCounter_;
    e.hasAlpha = false;
    e.colorKeyBlack = false;
    textureCache.emplace(cacheKey, std::move(e));

    core::Logger::getInstance().info("Composite texture created: ", width, "x", height, " from ", layerPaths.size(), " layers");
    return texPtr;
}

void CharacterRenderer::clearCompositeCache() {
    // Just clear the lookup map so next compositeWithRegions() creates fresh textures.
    // Don't delete GPU textures -- they may still be referenced by models or instances.
    // Orphaned textures will be cleaned up when their model/instance is destroyed.
    compositeCache_.clear();
}

VkTexture* CharacterRenderer::compositeWithRegions(const std::string& basePath,
                                                const std::vector<std::string>& baseLayers,
                                                const std::vector<std::pair<int, std::string>>& regionLayers) {
    // Build cache key from all inputs to avoid redundant compositing
    std::string cacheKey = basePath;
    for (const auto& bl : baseLayers) { cacheKey += '|'; cacheKey += bl; }
    cacheKey += '#';
    for (const auto& rl : regionLayers) {
        cacheKey += std::to_string(rl.first);
        cacheKey += ':';
        cacheKey += rl.second;
        cacheKey += ',';
    }
    auto cacheIt = compositeCache_.find(cacheKey);
    if (cacheIt != compositeCache_.end() && cacheIt->second != nullptr) {
        return cacheIt->second;
    }

    // If the lookup map was cleared, recover from the texture cache without
    // regenerating/replacing the underlying GPU texture.
    std::string storageKey = "__compositeRegions__" + cacheKey;
    auto cachedComposite = textureCache.find(storageKey);
    if (cachedComposite != textureCache.end()) {
        cachedComposite->second.lastUse = ++textureCacheCounter_;
        VkTexture* texPtr = cachedComposite->second.texture.get();
        compositeCache_[cacheKey] = texPtr;
        return texPtr;
    }

    // Region index -> pixel coordinates on the 256x256 base atlas
    // These are scaled up by (width/256, height/256) for larger textures (512x512, 1024x1024)
    static const int regionCoords256[][2] = {
        {   0,   0 },  // 0 = ArmUpper
        {   0,  64 },  // 1 = ArmLower
        {   0, 128 },  // 2 = Hand
        { 128,   0 },  // 3 = TorsoUpper
        { 128,  64 },  // 4 = TorsoLower
        { 128,  96 },  // 5 = LegUpper
        { 128, 160 },  // 6 = LegLower
        { 128, 224 },  // 7 = Foot
    };

    // First, build base skin + underwear using existing compositeTextures
    std::vector<std::string> layers;
    layers.push_back(basePath);
    for (const auto& ul : baseLayers) {
        layers.push_back(ul);
    }
    // Load base composite into CPU buffer
    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture_.get();
    }

    auto base = assetManager->loadTexture(basePath);
    if (!base.isValid()) {
        return whiteTexture_.get();
    }

    std::vector<uint8_t> composite;
    int width = base.width;
    int height = base.height;

    // If base texture is 256x256 (e.g., baked NPC texture), upscale to 512x512
    // so equipment regions can be composited at correct coordinates
    if (width == 256 && height == 256 && !regionLayers.empty()) {
        width = 512;
        height = 512;
        composite.resize(width * height * 4);
        // Simple 2x nearest-neighbor upscale
        for (int y = 0; y < 512; y++) {
            for (int x = 0; x < 512; x++) {
                int srcX = x / 2;
                int srcY = y / 2;
                int srcIdx = (srcY * 256 + srcX) * 4;
                int dstIdx = (y * 512 + x) * 4;
                composite[dstIdx + 0] = base.data[srcIdx + 0];
                composite[dstIdx + 1] = base.data[srcIdx + 1];
                composite[dstIdx + 2] = base.data[srcIdx + 2];
                composite[dstIdx + 3] = base.data[srcIdx + 3];
            }
        }
        core::Logger::getInstance().debug("compositeWithRegions: upscaled 256x256 to 512x512");
    } else {
        composite = base.data;
    }

    // Blend face + underwear overlays
    // If we upscaled from 256->512, scale coords and texels with blitOverlayScaled2x.
    // For native 512/1024 textures, face overlays are full atlas size (hit width==width branch).
    bool upscaled = (base.width == 256 && base.height == 256 && width == 512);
    for (const auto& ul : baseLayers) {
        if (ul.empty()) continue;
        auto overlay = assetManager->loadTexture(ul);
        if (!overlay.isValid()) continue;

        if (overlay.width == width && overlay.height == height) {
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // WoW 256-scale atlas coordinates (from CharComponentTextureSections)
            int dstX = 0, dstY = 0;
            std::string pathLower = ul;
            for (auto& c : pathLower) c = std::tolower(c);

            // Scale factor from 256-base coordinates to actual canvas size
            int coordScale = width / 256;
            if (coordScale < 1) coordScale = 1;
            bool useScale = true;

            if (pathLower.find("faceupper") != std::string::npos) {
                dstX = 0; dstY = 160;
            } else if (pathLower.find("facelower") != std::string::npos) {
                dstX = 0; dstY = 192;
            } else if (pathLower.find("pelvis") != std::string::npos) {
                dstX = 128; dstY = 96;
            } else if (pathLower.find("torso") != std::string::npos) {
                dstX = 128; dstY = 0;
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 64;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 128;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 128; dstY = 224;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 128; dstY = 160;
            } else {
                // Fallback: center overlay on canvas (already in canvas coords)
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
                useScale = false;
            }

            if (useScale) {
                dstX *= coordScale;
                dstY *= coordScale;
            }

            if (upscaled) {
                // Overlay is 256-base sized, needs 2x texel scaling for 512 canvas
                blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        }
    }

    // Expected region sizes on the 256x256 base atlas (scaled like coords)
    static const int regionSizes256[][2] = {
        { 128,  64 },  // 0 = ArmUpper
        { 128,  64 },  // 1 = ArmLower
        { 128,  32 },  // 2 = Hand
        { 128,  64 },  // 3 = TorsoUpper
        { 128,  32 },  // 4 = TorsoLower
        { 128,  64 },  // 5 = LegUpper
        { 128,  64 },  // 6 = LegLower
        { 128,  32 },  // 7 = Foot
    };

    // Scale factor from 256-base to actual texture size
    int scaleX = width / 256;
    int scaleY = height / 256;
    if (scaleX < 1) scaleX = 1;
    if (scaleY < 1) scaleY = 1;

    // Now blit equipment region textures at explicit coordinates
    for (const auto& rl : regionLayers) {
        int regionIdx = rl.first;
        if (regionIdx < 0 || regionIdx >= 8) continue;

        auto overlay = assetManager->loadTexture(rl.second);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("compositeWithRegions: failed to load ", rl.second);
            continue;
        }

        int dstX = regionCoords256[regionIdx][0] * scaleX;
        int dstY = regionCoords256[regionIdx][1] * scaleY;

        // Expected full-resolution size for this region at current atlas scale
        int expectedW = regionSizes256[regionIdx][0] * scaleX;
        int expectedH = regionSizes256[regionIdx][1] * scaleY;
        if (overlay.width * 2 == expectedW && overlay.height * 2 == expectedH) {
            blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
        } else {
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }

        core::Logger::getInstance().debug("compositeWithRegions: region ", regionIdx,
            " at (", dstX, ",", dstY, ") ", overlay.width, "x", overlay.height, " from ", rl.second);
    }

    // Upload to GPU via VkTexture
    auto tex = std::make_unique<VkTexture>();
    tex->upload(*vkCtx_, composite.data(), width, height, VK_FORMAT_R8G8B8A8_UNORM, true);
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                       VK_SAMPLER_ADDRESS_MODE_REPEAT);

    VkTexture* texPtr = tex.get();

    // Store in texture cache.
    // Use emplace to avoid replacing an existing texture for this key; replacing
    // would invalidate pointers currently bound to active instances.
    TextureCacheEntry entry;
    entry.texture = std::move(tex);
    entry.approxBytes = approxTextureBytesWithMips(width, height);
    entry.lastUse = ++textureCacheCounter_;
    entry.hasAlpha = false;
    entry.colorKeyBlack = false;
    auto ins = textureCache.emplace(storageKey, std::move(entry));
    if (!ins.second) {
        // Existing texture already owns this key; keep pointer stable.
        ins.first->second.lastUse = ++textureCacheCounter_;
        compositeCache_[cacheKey] = ins.first->second.texture.get();
        return ins.first->second.texture.get();
    }

    core::Logger::getInstance().debug("compositeWithRegions: created ", width, "x", height,
        " texture with ", regionLayers.size(), " equipment regions");
    compositeCache_[cacheKey] = texPtr;
    return texPtr;
}

void CharacterRenderer::setModelTexture(uint32_t modelId, uint32_t textureSlot, VkTexture* texture) {
    auto it = models.find(modelId);
    if (it == models.end()) {
        core::Logger::getInstance().warning("setModelTexture: model ", modelId, " not found");
        return;
    }

    auto& gpuModel = it->second;
    if (textureSlot >= gpuModel.textureIds.size()) {
        core::Logger::getInstance().warning("setModelTexture: slot ", textureSlot, " out of range (", gpuModel.textureIds.size(), " textures)");
        return;
    }

    gpuModel.textureIds[textureSlot] = texture;
    core::Logger::getInstance().debug("Replaced model ", modelId, " texture slot ", textureSlot, " with composited texture");
}

void CharacterRenderer::resetModelTexture(uint32_t modelId, uint32_t textureSlot) {
    setModelTexture(modelId, textureSlot, whiteTexture_.get());
}

bool CharacterRenderer::loadModel(const pipeline::M2Model& model, uint32_t id) {
    if (!model.isValid()) {
        core::Logger::getInstance().error("Cannot load invalid M2 model");
        return false;
    }

    if (models.find(id) != models.end()) {
        core::Logger::getInstance().warning("Model ID ", id, " already loaded, replacing");
        destroyModelGPU(models[id]);
    }

    M2ModelGPU gpuModel;
    gpuModel.data = model;

    // Setup GPU buffers
    setupModelBuffers(gpuModel);

    // Calculate bind pose
    calculateBindPose(gpuModel);

    // Load textures from model
    for (const auto& tex : model.textures) {
        VkTexture* texPtr = loadTexture(tex.filename);
        gpuModel.textureIds.push_back(texPtr);
    }

    models[id] = std::move(gpuModel);

    core::Logger::getInstance().debug("Loaded M2 model ", id, " (", model.vertices.size(),
                       " verts, ", model.bones.size(), " bones, ", model.sequences.size(),
                       " anims, ", model.textures.size(), " textures)");

    return true;
}

void CharacterRenderer::setupModelBuffers(M2ModelGPU& gpuModel) {
    auto& model = gpuModel.data;

    if (model.vertices.empty() || model.indices.empty()) return;

    const size_t vertCount = model.vertices.size();
    const size_t idxCount = model.indices.size();

    // Build expanded GPU vertex buffer with tangents (Lengyel's method)
    std::vector<CharVertexGPU> gpuVerts(vertCount);
    std::vector<glm::vec3> tanAccum(vertCount, glm::vec3(0.0f));
    std::vector<glm::vec3> bitanAccum(vertCount, glm::vec3(0.0f));

    // Copy base vertex data
    for (size_t i = 0; i < vertCount; i++) {
        const auto& src = model.vertices[i];
        auto& dst = gpuVerts[i];
        dst.position = src.position;
        std::memcpy(dst.boneWeights, src.boneWeights, 4);
        std::memcpy(dst.boneIndices, src.boneIndices, 4);
        dst.normal = src.normal;
        dst.texCoords = src.texCoords[0]; // Use first UV set
        dst.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f); // default
    }

    // Accumulate tangent/bitangent per triangle
    for (size_t i = 0; i + 2 < idxCount; i += 3) {
        uint16_t i0 = model.indices[i], i1 = model.indices[i+1], i2 = model.indices[i+2];
        if (i0 >= vertCount || i1 >= vertCount || i2 >= vertCount) continue;

        const glm::vec3& p0 = gpuVerts[i0].position;
        const glm::vec3& p1 = gpuVerts[i1].position;
        const glm::vec3& p2 = gpuVerts[i2].position;
        const glm::vec2& uv0 = gpuVerts[i0].texCoords;
        const glm::vec2& uv1 = gpuVerts[i1].texCoords;
        const glm::vec2& uv2 = gpuVerts[i2].texCoords;

        glm::vec3 edge1 = p1 - p0;
        glm::vec3 edge2 = p2 - p0;
        glm::vec2 duv1 = uv1 - uv0;
        glm::vec2 duv2 = uv2 - uv0;

        float det = duv1.x * duv2.y - duv2.x * duv1.y;
        if (std::abs(det) < 1e-8f) continue;
        float invDet = 1.0f / det;

        glm::vec3 t = (edge1 * duv2.y - edge2 * duv1.y) * invDet;
        glm::vec3 b = (edge2 * duv1.x - edge1 * duv2.x) * invDet;

        tanAccum[i0] += t; tanAccum[i1] += t; tanAccum[i2] += t;
        bitanAccum[i0] += b; bitanAccum[i1] += b; bitanAccum[i2] += b;
    }

    // Orthogonalize and compute handedness
    for (size_t i = 0; i < vertCount; i++) {
        const glm::vec3& n = gpuVerts[i].normal;
        const glm::vec3& t = tanAccum[i];
        if (glm::dot(t, t) < 1e-8f) {
            gpuVerts[i].tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
            continue;
        }
        // Gram-Schmidt orthogonalize
        glm::vec3 tOrtho = glm::normalize(t - n * glm::dot(n, t));
        float w = (glm::dot(glm::cross(n, t), bitanAccum[i]) < 0.0f) ? -1.0f : 1.0f;
        gpuVerts[i].tangent = glm::vec4(tOrtho, w);
    }

    // Upload vertex buffer (CharVertexGPU, 56 bytes per vertex)
    auto vb = uploadBuffer(*vkCtx_,
        gpuVerts.data(),
        gpuVerts.size() * sizeof(CharVertexGPU),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    gpuModel.vertexBuffer = vb.buffer;
    gpuModel.vertexAlloc = vb.allocation;
    gpuModel.vertexCount = static_cast<uint32_t>(vertCount);

    // Upload index buffer
    auto ib = uploadBuffer(*vkCtx_,
        model.indices.data(),
        idxCount * sizeof(uint16_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    gpuModel.indexBuffer = ib.buffer;
    gpuModel.indexAlloc = ib.allocation;
    gpuModel.indexCount = static_cast<uint32_t>(idxCount);
}

void CharacterRenderer::calculateBindPose(M2ModelGPU& gpuModel) {
    auto& bones = gpuModel.data.bones;
    size_t numBones = bones.size();
    gpuModel.bindPose.resize(numBones);

    // Compute full hierarchical rest pose, then invert.
    // Each bone's rest position is T(pivot), composed with its parent chain.
    std::vector<glm::mat4> restPose(numBones);
    for (size_t i = 0; i < numBones; i++) {
        glm::mat4 local = glm::translate(glm::mat4(1.0f), bones[i].pivot);
        if (bones[i].parentBone >= 0 && static_cast<size_t>(bones[i].parentBone) < numBones) {
            restPose[i] = restPose[bones[i].parentBone] * local;
        } else {
            restPose[i] = local;
        }
        gpuModel.bindPose[i] = glm::inverse(restPose[i]);
    }
}

uint32_t CharacterRenderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                           const glm::vec3& rotation, float scale) {
    if (models.find(modelId) == models.end()) {
        core::Logger::getInstance().error("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    CharacterInstance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;

    // Initialize bone matrices to identity
    auto& model = models[modelId].data;
    instance.boneMatrices.resize(std::max(static_cast<size_t>(1), model.bones.size()), glm::mat4(1.0f));

    uint32_t id = instance.id;
    instances[id] = std::move(instance);
    return id;
}

void CharacterRenderer::playAnimation(uint32_t instanceId, uint32_t animationId, bool loop) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        core::Logger::getInstance().warning("Cannot play animation: instance ", instanceId, " not found");
        return;
    }

    auto& instance = it->second;
    auto& model = models[instance.modelId].data;

    // Track death state for preventing movement while dead
    if (animationId == 1) {
        instance.isDead = true;
    } else if (instance.isDead && animationId == 0) {
        instance.isDead = false;  // Respawned
    }

    // Find animation sequence index by ID
    instance.currentAnimationId = animationId;
    instance.currentSequenceIndex = -1;
    instance.animationTime = 0.0f;
    instance.animationLoop = loop;

    for (size_t i = 0; i < model.sequences.size(); i++) {
        if (model.sequences[i].id == animationId) {
            instance.currentSequenceIndex = static_cast<int>(i);
            break;
        }
    }

    if (instance.currentSequenceIndex < 0) {
        // Fall back to first sequence
        if (!model.sequences.empty()) {
            instance.currentSequenceIndex = 0;
            instance.currentAnimationId = model.sequences[0].id;
        }

        // Only log missing animation once per model (reduce spam)
        static std::unordered_map<uint32_t, std::unordered_set<uint32_t>> loggedMissingAnims;
        uint32_t mId = instance.modelId;  // Use modelId as identifier
        if (loggedMissingAnims[mId].insert(animationId).second) {
            // First time seeing this missing animation for this model
            LOG_WARNING("Animation ", animationId, " not found in model ", mId, ", using default");
        }
    }
}

void CharacterRenderer::update(float deltaTime, const glm::vec3& cameraPos) {
    // Distance culling for animation updates in dense areas.
    const float animUpdateRadius = static_cast<float>(envSizeOrDefault("WOWEE_CHAR_ANIM_RADIUS", 120));
    const float animUpdateRadiusSq = animUpdateRadius * animUpdateRadius;

    // Update fade-in opacity
    for (auto& [id, inst] : instances) {
        if (inst.fadeInDuration > 0.0f && inst.opacity < 1.0f) {
            inst.fadeInTime += deltaTime;
            inst.opacity = std::min(1.0f, inst.fadeInTime / inst.fadeInDuration);
            if (inst.opacity >= 1.0f) {
                inst.fadeInDuration = 0.0f;
            }
        }
    }

    // Interpolate creature movement
    for (auto& [id, inst] : instances) {
        if (inst.isMoving) {
            inst.moveElapsed += deltaTime;
            float t = inst.moveElapsed / inst.moveDuration;
            if (t >= 1.0f) {
                inst.position = inst.moveEnd;
                inst.isMoving = false;
                // Return to idle when movement completes
                if (inst.currentAnimationId == 4 || inst.currentAnimationId == 5) {
                    playAnimation(id, 0, true);
                }
            } else {
                inst.position = glm::mix(inst.moveStart, inst.moveEnd, t);
            }
        }
    }

    // Only update animations for nearby characters (performance optimization)
    // Collect instances that need updates
    std::vector<std::reference_wrapper<CharacterInstance>> toUpdate;
    toUpdate.reserve(instances.size());

    for (auto& pair : instances) {
        float distSq = glm::distance2(pair.second.position, cameraPos);
        if (distSq < animUpdateRadiusSq) {
            toUpdate.push_back(std::ref(pair.second));
        }
    }

    const size_t updatedCount = toUpdate.size();

    // Thread animation updates in chunks to avoid spawning one task per instance.
    if (updatedCount >= 8 && numAnimThreads_ > 1) {
        static const size_t minAnimWorkPerThread = std::max<size_t>(
            16, envSizeOrDefault("WOWEE_CHAR_ANIM_WORK_PER_THREAD", 64));
        const size_t maxUsefulThreads = std::max<size_t>(
            1, (updatedCount + minAnimWorkPerThread - 1) / minAnimWorkPerThread);
        const size_t numThreads = std::min(static_cast<size_t>(numAnimThreads_), maxUsefulThreads);

        if (numThreads <= 1) {
            for (auto& instRef : toUpdate) {
                updateAnimation(instRef.get(), deltaTime);
            }
        } else {
            const size_t chunkSize = updatedCount / numThreads;
            const size_t remainder = updatedCount % numThreads;

            animFutures_.clear();
            if (animFutures_.capacity() < numThreads) {
                animFutures_.reserve(numThreads);
            }

            size_t start = 0;
            for (size_t t = 0; t < numThreads; t++) {
                size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                animFutures_.push_back(std::async(std::launch::async,
                    [this, &toUpdate, start, end, deltaTime]() {
                        for (size_t i = start; i < end; i++) {
                            updateAnimation(toUpdate[i].get(), deltaTime);
                        }
                    }));
                start = end;
            }

            for (auto& f : animFutures_) {
                f.get();
            }
        }
    } else {
        // Sequential for small counts (avoid thread overhead)
        for (auto& instRef : toUpdate) {
            updateAnimation(instRef.get(), deltaTime);
        }
    }

    // Update weapon attachment transforms (after all bone matrices are computed)
    for (auto& pair : instances) {
        auto& instance = pair.second;
        if (instance.weaponAttachments.empty()) continue;
        if (glm::distance2(instance.position, cameraPos) > animUpdateRadiusSq) continue;

        glm::mat4 charModelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);

        for (const auto& wa : instance.weaponAttachments) {
            auto weapIt = instances.find(wa.weaponInstanceId);
            if (weapIt == instances.end()) continue;

            // Get the bone matrix for the attachment bone
            glm::mat4 boneMat(1.0f);
            if (wa.boneIndex < instance.boneMatrices.size()) {
                boneMat = instance.boneMatrices[wa.boneIndex];
            }

            // Weapon model matrix = character model * bone transform * offset translation
            weapIt->second.overrideModelMatrix =
                charModelMat * boneMat * glm::translate(glm::mat4(1.0f), wa.offset);
            weapIt->second.hasOverrideModelMatrix = true;
        }
    }
}

void CharacterRenderer::updateAnimation(CharacterInstance& instance, float deltaTime) {
    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        return;
    }
    const auto& model = modelIt->second.data;

    if (model.sequences.empty()) {
        return;
    }

    // Resolve sequence index if not set
    if (instance.currentSequenceIndex < 0) {
        instance.currentSequenceIndex = 0;
        instance.currentAnimationId = model.sequences[0].id;
    }

    const auto& sequence = model.sequences[instance.currentSequenceIndex];

    // Update animation time (convert to milliseconds)
    instance.animationTime += deltaTime * 1000.0f;

    if (sequence.duration > 0 && instance.animationTime >= static_cast<float>(sequence.duration)) {
        if (instance.animationLoop) {
            instance.animationTime = std::fmod(instance.animationTime, static_cast<float>(sequence.duration));
        } else {
            instance.animationTime = static_cast<float>(sequence.duration);
        }
    }

    // Update bone matrices
    calculateBoneMatrices(instance);
}

// --- Keyframe interpolation helpers ---

int CharacterRenderer::findKeyframeIndex(const std::vector<uint32_t>& timestamps, float time) {
    if (timestamps.empty()) return -1;
    if (timestamps.size() == 1) return 0;

    // Binary search for the keyframe bracket
    for (size_t i = 0; i < timestamps.size() - 1; i++) {
        if (time < static_cast<float>(timestamps[i + 1])) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(timestamps.size() - 2);
}

glm::vec3 CharacterRenderer::interpolateVec3(const pipeline::M2AnimationTrack& track,
                                              int seqIdx, float time, const glm::vec3& defaultVal) {
    if (!track.hasData()) return defaultVal;
    if (seqIdx < 0 || seqIdx >= static_cast<int>(track.sequences.size())) return defaultVal;

    const auto& keys = track.sequences[seqIdx];
    if (keys.timestamps.empty() || keys.vec3Values.empty()) return defaultVal;

    auto safeVec3 = [&](const glm::vec3& v) -> glm::vec3 {
        if (std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z)) return defaultVal;
        return v;
    };

    if (keys.vec3Values.size() == 1) return safeVec3(keys.vec3Values[0]);

    int idx = findKeyframeIndex(keys.timestamps, time);
    if (idx < 0) return defaultVal;

    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.vec3Values.size() - 1);

    if (i0 == i1) return safeVec3(keys.vec3Values[i0]);

    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float duration = t1 - t0;
    float t = (duration > 0.0f) ? glm::clamp((time - t0) / duration, 0.0f, 1.0f) : 0.0f;

    return safeVec3(glm::mix(keys.vec3Values[i0], keys.vec3Values[i1], t));
}

glm::quat CharacterRenderer::interpolateQuat(const pipeline::M2AnimationTrack& track,
                                              int seqIdx, float time) {
    glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    if (!track.hasData()) return identity;
    if (seqIdx < 0 || seqIdx >= static_cast<int>(track.sequences.size())) return identity;

    const auto& keys = track.sequences[seqIdx];
    if (keys.timestamps.empty() || keys.quatValues.empty()) return identity;

    auto safeQuat = [&](const glm::quat& q) -> glm::quat {
        float len = glm::length(q);
        if (len < 0.001f || std::isnan(len)) return identity;
        return q;
    };

    if (keys.quatValues.size() == 1) return safeQuat(keys.quatValues[0]);

    int idx = findKeyframeIndex(keys.timestamps, time);
    if (idx < 0) return identity;

    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.quatValues.size() - 1);

    if (i0 == i1) return safeQuat(keys.quatValues[i0]);

    glm::quat q0 = safeQuat(keys.quatValues[i0]);
    glm::quat q1 = safeQuat(keys.quatValues[i1]);

    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float duration = t1 - t0;
    float t = (duration > 0.0f) ? glm::clamp((time - t0) / duration, 0.0f, 1.0f) : 0.0f;

    return glm::slerp(q0, q1, t);
}

// --- Bone transform calculation ---

void CharacterRenderer::calculateBoneMatrices(CharacterInstance& instance) {
    auto& model = models[instance.modelId].data;

    if (model.bones.empty()) {
        return;
    }

    size_t numBones = model.bones.size();
    instance.boneMatrices.resize(numBones);

    static bool dumpedOnce = false;

    for (size_t i = 0; i < numBones; i++) {
        const auto& bone = model.bones[i];

        // Local transform includes pivot bracket: T(pivot)*T*R*S*T(-pivot)
        // At rest this is identity, so no separate bind pose is needed
        glm::mat4 localTransform = getBoneTransform(bone, instance.animationTime, instance.currentSequenceIndex);

        // Debug: dump first frame bone data
        if (!dumpedOnce && i < 5) {
            glm::vec3 t = interpolateVec3(bone.translation, instance.currentSequenceIndex, instance.animationTime, glm::vec3(0.0f));
            glm::quat r = interpolateQuat(bone.rotation, instance.currentSequenceIndex, instance.animationTime);
            glm::vec3 s = interpolateVec3(bone.scale, instance.currentSequenceIndex, instance.animationTime, glm::vec3(1.0f));
            core::Logger::getInstance().info("Bone ", i, " parent=", bone.parentBone,
                " pivot=(", bone.pivot.x, ",", bone.pivot.y, ",", bone.pivot.z, ")",
                " t=(", t.x, ",", t.y, ",", t.z, ")",
                " r=(", r.w, ",", r.x, ",", r.y, ",", r.z, ")",
                " s=(", s.x, ",", s.y, ",", s.z, ")",
                " seqIdx=", instance.currentSequenceIndex);
        }

        // Compose with parent
        if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) < numBones) {
            instance.boneMatrices[i] = instance.boneMatrices[bone.parentBone] * localTransform;
        } else {
            instance.boneMatrices[i] = localTransform;
        }
    }
    if (!dumpedOnce) {
        dumpedOnce = true;
        // Dump final matrix for bone 0
        auto& m = instance.boneMatrices[0];
        core::Logger::getInstance().info("Bone 0 final matrix row0=(", m[0][0], ",", m[1][0], ",", m[2][0], ",", m[3][0], ")");
    }
}

glm::mat4 CharacterRenderer::getBoneTransform(const pipeline::M2Bone& bone, float time, int sequenceIndex) {
    glm::vec3 translation = interpolateVec3(bone.translation, sequenceIndex, time, glm::vec3(0.0f));
    glm::quat rotation = interpolateQuat(bone.rotation, sequenceIndex, time);
    glm::vec3 scale = interpolateVec3(bone.scale, sequenceIndex, time, glm::vec3(1.0f));

    // M2 bone transform: T(pivot) * T(trans) * R(rot) * S(scale) * T(-pivot)
    // At rest (no animation): T(pivot) * I * I * I * T(-pivot) = identity
    glm::mat4 transform = glm::translate(glm::mat4(1.0f), bone.pivot);
    transform = glm::translate(transform, translation);
    transform *= glm::toMat4(rotation);
    transform = glm::scale(transform, scale);
    transform = glm::translate(transform, -bone.pivot);

    return transform;
}

// --- Rendering ---

void CharacterRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, [[maybe_unused]] const Camera& camera) {
    if (instances.empty() || !opaquePipeline_) {
        return;
    }
    const float renderRadius = static_cast<float>(envSizeOrDefault("WOWEE_CHAR_RENDER_RADIUS", 130));
    const float renderRadiusSq = renderRadius * renderRadius;
    const float nearNoConeCullSq = 16.0f * 16.0f;
    const float backfaceDotCull = -0.30f;
    const glm::vec3 camPos = camera.getPosition();
    const glm::vec3 camForward = camera.getForward();

    uint32_t frameIndex = vkCtx_->getCurrentFrame();
    uint32_t frameSlot = frameIndex % 2u;

    // Reset transient material allocations once per frame slot.
    // beginFrame() waits on this slot's fence before recording.
    if (lastMaterialPoolResetFrame_ != frameIndex) {
        VmaAllocator alloc = vkCtx_->getAllocator();
        for (const auto& b : transientMaterialUbos_[frameSlot]) {
            if (b.first) {
                vmaDestroyBuffer(alloc, b.first, b.second);
            }
        }
        transientMaterialUbos_[frameSlot].clear();
        if (materialDescPools_[frameSlot]) {
            vkResetDescriptorPool(vkCtx_->getDevice(), materialDescPools_[frameSlot], 0);
        }
        lastMaterialPoolResetFrame_ = frameIndex;
    }

    // Bind per-frame descriptor set (set 0) -- shared across all draws
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    // Start with opaque pipeline
    VkPipeline currentPipeline = opaquePipeline_;
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, currentPipeline);

    for (const auto& pair : instances) {
        const auto& instance = pair.second;

        // Skip invisible instances (e.g., player in first-person mode)
        if (!instance.visible) continue;
        // Character instance culling: avoid drawing far-away / strongly behind-camera
        // actors in dense city scenes.
        if (!instance.hasOverrideModelMatrix) {
            glm::vec3 toInst = instance.position - camPos;
            float distSq = glm::dot(toInst, toInst);
            if (distSq > renderRadiusSq) continue;
            if (distSq > nearNoConeCullSq) {
                float invDist = 1.0f / std::sqrt(distSq);
                float facingDot = glm::dot(toInst, camForward) * invDist;
                if (facingDot < backfaceDotCull) continue;
            }
        }

        auto modelIt = models.find(instance.modelId);
        if (modelIt == models.end()) continue;
        const auto& gpuModel = modelIt->second;

        // Skip models without GPU buffers
        if (!gpuModel.vertexBuffer) continue;

        // Skip fully transparent instances
        if (instance.opacity <= 0.0f) continue;

        // Set model matrix (use override for weapon instances)
        glm::mat4 modelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);

        // Push model matrix
        vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &modelMat);

        // Upload bone matrices to SSBO
        int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), MAX_BONES);
        if (numBones > 0) {
            // Lazy-allocate bone SSBO on first use
            auto& instMut = const_cast<CharacterInstance&>(instance);
            if (!instMut.boneBuffer[frameIndex]) {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = MAX_BONES * sizeof(glm::mat4);
                bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo allocInfo{};
                vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci,
                                &instMut.boneBuffer[frameIndex], &instMut.boneAlloc[frameIndex], &allocInfo);
                instMut.boneMapped[frameIndex] = allocInfo.pMappedData;

                // Allocate descriptor set for bone SSBO
                VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                ai.descriptorPool = boneDescPool_;
                ai.descriptorSetCount = 1;
                ai.pSetLayouts = &boneSetLayout_;
                VkResult dsRes = vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &instMut.boneSet[frameIndex]);
                if (dsRes != VK_SUCCESS) {
                    LOG_ERROR("CharacterRenderer: bone descriptor allocation failed (instance=",
                              instMut.id, ", frame=", frameIndex, ", vk=", static_cast<int>(dsRes), ")");
                    if (instMut.boneBuffer[frameIndex]) {
                        vmaDestroyBuffer(vkCtx_->getAllocator(),
                                         instMut.boneBuffer[frameIndex], instMut.boneAlloc[frameIndex]);
                        instMut.boneBuffer[frameIndex] = VK_NULL_HANDLE;
                        instMut.boneAlloc[frameIndex] = VK_NULL_HANDLE;
                        instMut.boneMapped[frameIndex] = nullptr;
                    }
                }

                if (instMut.boneSet[frameIndex]) {
                    VkDescriptorBufferInfo bufInfo{};
                    bufInfo.buffer = instMut.boneBuffer[frameIndex];
                    bufInfo.offset = 0;
                    bufInfo.range = bci.size;
                    VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                    write.dstSet = instMut.boneSet[frameIndex];
                    write.dstBinding = 0;
                    write.descriptorCount = 1;
                    write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    write.pBufferInfo = &bufInfo;
                    vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &write, 0, nullptr);
                }
            }

            // Upload bone matrices
            if (instMut.boneMapped[frameIndex]) {
                memcpy(instMut.boneMapped[frameIndex], instance.boneMatrices.data(),
                       numBones * sizeof(glm::mat4));
            }

            // Bind bone descriptor set (set 2)
            if (instMut.boneSet[frameIndex]) {
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 2, 1, &instMut.boneSet[frameIndex], 0, nullptr);
            }
        }

        // Bind vertex and index buffers
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &gpuModel.vertexBuffer, &offset);
        vkCmdBindIndexBuffer(cmd, gpuModel.indexBuffer, 0, VK_INDEX_TYPE_UINT16);

        if (!gpuModel.data.batches.empty()) {
            bool applyGeosetFilter = !instance.activeGeosets.empty();
            if (applyGeosetFilter) {
                bool hasRenderableGeoset = false;
                for (const auto& batch : gpuModel.data.batches) {
                    if (instance.activeGeosets.find(batch.submeshId) != instance.activeGeosets.end()) {
                        hasRenderableGeoset = true;
                        break;
                    }
                }
                if (!hasRenderableGeoset) {
                    static std::unordered_set<uint32_t> loggedGeosetFallback;
                    if (loggedGeosetFallback.insert(instance.id).second) {
                        LOG_WARNING("Geoset filter matched no batches for instance ",
                                    instance.id, " (model ", instance.modelId,
                                    "); rendering all batches as fallback");
                    }
                    applyGeosetFilter = false;
                }
            }

            auto resolveBatchTexture = [&](const CharacterInstance& inst, const M2ModelGPU& gm, const pipeline::M2Batch& b) -> VkTexture* {
                // A skin batch can reference multiple textures (b.textureCount) starting at b.textureIndex.
                // We currently bind only a single texture, so pick the most appropriate one.
                if (b.textureIndex == 0xFFFF) return whiteTexture_.get();
                if (gm.data.textureLookup.empty() || gm.textureIds.empty()) return whiteTexture_.get();

                uint32_t comboCount = b.textureCount ? static_cast<uint32_t>(b.textureCount) : 1u;
                comboCount = std::min<uint32_t>(comboCount, 8u);

                struct Candidate { VkTexture* tex; uint32_t type; };
                Candidate first{whiteTexture_.get(), 0};
                bool hasFirst = false;
                Candidate firstNonWhite{whiteTexture_.get(), 0};
                bool hasFirstNonWhite = false;

                for (uint32_t i = 0; i < comboCount; i++) {
                    uint32_t lookupPos = static_cast<uint32_t>(b.textureIndex) + i;
                    if (lookupPos >= gm.data.textureLookup.size()) break;
                    uint16_t texSlot = gm.data.textureLookup[lookupPos];
                    if (texSlot >= gm.textureIds.size()) continue;

                    VkTexture* texPtr = gm.textureIds[texSlot];
                    uint32_t texType = (texSlot < gm.data.textures.size()) ? gm.data.textures[texSlot].type : 0;
                    // Apply texture slot overrides.
                    // For type-1 (skin) overrides, only apply to skin-group batches
                    // to prevent the skin composite from bleeding onto cloak/hair.
                    {
                        auto itO = inst.textureSlotOverrides.find(texSlot);
                        if (itO != inst.textureSlotOverrides.end() && itO->second != nullptr) {
                            if (texType == 1) {
                                // Only apply skin override to skin groups
                                uint16_t grp = b.submeshId / 100;
                                bool isSkinGroup = (grp == 0 || grp == 3 || grp == 4 || grp == 5 ||
                                                    grp == 8 || grp == 9 || grp == 13 || grp == 20);
                                if (isSkinGroup) texPtr = itO->second;
                            } else {
                                texPtr = itO->second;
                            }
                        }
                    }

                    if (!hasFirst) {
                        first = {texPtr, texType};
                        hasFirst = true;
                    }

                    if (texPtr == nullptr || texPtr == whiteTexture_.get()) continue;

                    // Prefer the hair texture slot (type 6) whenever present in the combo.
                    if (texType == 6) {
                        return texPtr;
                    }

                    if (!hasFirstNonWhite) {
                        firstNonWhite = {texPtr, texType};
                        hasFirstNonWhite = true;
                    }
                }

                if (hasFirstNonWhite) return firstNonWhite.tex;
                if (hasFirst && first.tex != nullptr) return first.tex;
                return whiteTexture_.get();
            };

            // One-time debug dump of rendered batches per model
            // Draw batches (submeshes) with per-batch textures
            for (const auto& batch : gpuModel.data.batches) {
                if (applyGeosetFilter) {
                    if (instance.activeGeosets.find(batch.submeshId) == instance.activeGeosets.end()) {
                        continue;
                    }
                }

                // Resolve texture for this batch (prefer hair textures for hair geosets).
                VkTexture* texPtr = resolveBatchTexture(instance, gpuModel, batch);
                const uint16_t batchGroup = static_cast<uint16_t>(batch.submeshId / 100);
                auto groupTexIt = instance.groupTextureOverrides.find(batchGroup);
                if (groupTexIt != instance.groupTextureOverrides.end() && groupTexIt->second != nullptr) {
                    texPtr = groupTexIt->second;
                }

                // Respect M2 material blend mode for creature/character submeshes.
                uint16_t blendMode = 0;
                uint16_t materialFlags = 0;
                if (batch.materialIndex < gpuModel.data.materials.size()) {
                    blendMode = gpuModel.data.materials[batch.materialIndex].blendMode;
                    materialFlags = gpuModel.data.materials[batch.materialIndex].flags;
                }

                // Attached weapon models can include additive FX/card batches that
                // appear as detached flat quads for some swords. Keep core geometry
                // and drop FX-style passes for weapon attachments.
                if (instance.hasOverrideModelMatrix && blendMode >= 3) {
                    continue;
                }

                // Select pipeline based on blend mode
                VkPipeline desiredPipeline;
                switch (blendMode) {
                    case 0: desiredPipeline = opaquePipeline_; break;
                    case 1: desiredPipeline = alphaTestPipeline_; break;
                    case 2: desiredPipeline = alphaPipeline_; break;
                    case 3:
                    case 6: desiredPipeline = additivePipeline_; break;
                    default: desiredPipeline = alphaPipeline_; break;
                }
                if (desiredPipeline != currentPipeline) {
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
                    currentPipeline = desiredPipeline;
                }

                // For body/equipment parts with white/fallback texture, use skin (type 1) texture.
                if (texPtr == whiteTexture_.get()) {
                    uint16_t group = batchGroup;
                    bool isSkinGroup = (group == 0 || group == 3 || group == 4 || group == 5 ||
                                        group == 8 || group == 9 || group == 13);
                    if (isSkinGroup) {
                        uint32_t texType = 0;
                        if (batch.textureIndex < gpuModel.data.textureLookup.size()) {
                            uint16_t lk = gpuModel.data.textureLookup[batch.textureIndex];
                            if (lk < gpuModel.data.textures.size()) {
                                texType = gpuModel.data.textures[lk].type;
                            }
                        }
                        // Do NOT apply skin composite to hair (type 6) batches
                        if (texType != 6) {
                            for (size_t ti = 0; ti < gpuModel.textureIds.size(); ti++) {
                                VkTexture* candidate = gpuModel.textureIds[ti];
                                auto itO = instance.textureSlotOverrides.find(static_cast<uint16_t>(ti));
                                if (itO != instance.textureSlotOverrides.end() && itO->second != nullptr) {
                                    candidate = itO->second;
                                }
                                if (candidate != whiteTexture_.get() && candidate != nullptr) {
                                    if (ti < gpuModel.data.textures.size() &&
                                        (gpuModel.data.textures[ti].type == 1 || gpuModel.data.textures[ti].type == 11)) {
                                        texPtr = candidate;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // Determine material properties
                bool alphaCutout = false;
                bool colorKeyBlack = false;
                if (texPtr != nullptr && texPtr != whiteTexture_.get()) {
                    auto ait = textureHasAlphaByPtr_.find(texPtr);
                    alphaCutout = (ait != textureHasAlphaByPtr_.end()) ? ait->second : false;
                    auto cit = textureColorKeyBlackByPtr_.find(texPtr);
                    colorKeyBlack = (cit != textureColorKeyBlackByPtr_.end()) ? cit->second : false;
                }
                const bool blendNeedsCutout = (blendMode == 1) || (blendMode >= 2 && !alphaCutout);
                const bool unlit = ((materialFlags & 0x01) != 0) || (blendMode >= 3);

                float emissiveBoost = 1.0f;
                glm::vec3 emissiveTint(1.0f, 1.0f, 1.0f);
                // Keep custom warm/flicker treatment narrowly scoped to kobold candle flames.
                bool koboldCandleFlame = false;
                if (colorKeyBlack) {
                    std::string modelKey = gpuModel.data.name;
                    std::transform(modelKey.begin(), modelKey.end(), modelKey.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    koboldCandleFlame =
                        (modelKey.find("kobold") != std::string::npos) &&
                        ((modelKey.find("candle") != std::string::npos) ||
                         (modelKey.find("torch") != std::string::npos) ||
                         (modelKey.find("mine") != std::string::npos));
                }
                if (unlit && koboldCandleFlame) {
                    using clock = std::chrono::steady_clock;
                    float t = std::chrono::duration<float>(clock::now().time_since_epoch()).count();
                    float phase = static_cast<float>(batch.submeshId) * 0.31f;
                    float f1 = std::sin(t * 7.9f + phase);
                    float f2 = std::sin(t * 12.7f + phase * 1.73f);
                    float f3 = std::sin(t * 4.3f + phase * 2.11f);
                    float flicker = 0.90f + 0.10f * f1 + 0.06f * f2 + 0.04f * f3;
                    flicker = std::clamp(flicker, 0.72f, 1.12f);
                    emissiveBoost = (blendMode >= 3) ? (2.4f * flicker) : (1.5f * flicker);
                    emissiveTint = glm::vec3(1.28f, 1.04f, 0.82f);
                }

                // Allocate and fill material descriptor set (set 1)
                VkDescriptorSet materialSet = VK_NULL_HANDLE;
                {
                    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                    ai.descriptorPool = materialDescPools_[frameSlot];
                    ai.descriptorSetCount = 1;
                    ai.pSetLayouts = &materialSetLayout_;
                    if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &materialSet) != VK_SUCCESS) {
                        continue; // Pool exhausted, skip this batch
                    }
                }

                // Resolve normal/height map for this texture
                VkTexture* normalMap = flatNormalTexture_.get();
                float batchHeightVariance = 0.0f;
                if (texPtr && texPtr != whiteTexture_.get()) {
                    for (const auto& ce : textureCache) {
                        if (ce.second.texture.get() == texPtr && ce.second.normalHeightMap) {
                            normalMap = ce.second.normalHeightMap.get();
                            batchHeightVariance = ce.second.heightMapVariance;
                            break;
                        }
                    }
                }

                // POM quality → sample count
                int pomSamples = 32;
                if (pomQuality_ == 0) pomSamples = 16;
                else if (pomQuality_ == 2) pomSamples = 64;

                // Create per-batch material UBO
                CharMaterialUBO matData{};
                matData.opacity = instance.opacity;
                matData.alphaTest = (blendNeedsCutout || alphaCutout) ? 1 : 0;
                matData.colorKeyBlack = (blendNeedsCutout || colorKeyBlack) ? 1 : 0;
                matData.unlit = unlit ? 1 : 0;
                matData.emissiveBoost = emissiveBoost;
                matData.emissiveTintR = emissiveTint.r;
                matData.emissiveTintG = emissiveTint.g;
                matData.emissiveTintB = emissiveTint.b;
                matData.specularIntensity = 0.5f;
                matData.enableNormalMap = normalMappingEnabled_ ? 1 : 0;
                matData.enablePOM = pomEnabled_ ? 1 : 0;
                matData.pomScale = 0.06f;
                matData.pomMaxSamples = pomSamples;
                matData.heightMapVariance = batchHeightVariance;
                matData.normalMapStrength = normalMapStrength_;

                // Create a small UBO for this batch's material
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = sizeof(CharMaterialUBO);
                bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo allocInfo{};
                ::VkBuffer matUBO = VK_NULL_HANDLE;
                VmaAllocation matUBOAlloc = VK_NULL_HANDLE;
                vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &matUBO, &matUBOAlloc, &allocInfo);
                if (allocInfo.pMappedData) {
                    memcpy(allocInfo.pMappedData, &matData, sizeof(CharMaterialUBO));
                }

                // Write descriptor set: binding 0 = texture, binding 1 = material UBO, binding 2 = normal/height map
                VkTexture* bindTex = (texPtr && texPtr->isValid()) ? texPtr : whiteTexture_.get();
                VkDescriptorImageInfo imgInfo = bindTex->descriptorInfo();
                VkDescriptorBufferInfo bufInfo{};
                bufInfo.buffer = matUBO;
                bufInfo.offset = 0;
                bufInfo.range = sizeof(CharMaterialUBO);
                VkDescriptorImageInfo nhImgInfo = normalMap->descriptorInfo();

                VkWriteDescriptorSet writes[3] = {};
                writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[0].dstSet = materialSet;
                writes[0].dstBinding = 0;
                writes[0].descriptorCount = 1;
                writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[0].pImageInfo = &imgInfo;

                writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[1].dstSet = materialSet;
                writes[1].dstBinding = 1;
                writes[1].descriptorCount = 1;
                writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                writes[1].pBufferInfo = &bufInfo;

                writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[2].dstSet = materialSet;
                writes[2].dstBinding = 2;
                writes[2].descriptorCount = 1;
                writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[2].pImageInfo = &nhImgInfo;

                vkUpdateDescriptorSets(vkCtx_->getDevice(), 3, writes, 0, nullptr);

                // Bind material descriptor set (set 1)
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 1, 1, &materialSet, 0, nullptr);

                vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);

                transientMaterialUbos_[frameSlot].emplace_back(matUBO, matUBOAlloc);
            }
        } else {
            // Draw entire model with first texture
            VkTexture* texPtr = !gpuModel.textureIds.empty() ? gpuModel.textureIds[0] : whiteTexture_.get();
            if (!texPtr || !texPtr->isValid()) texPtr = whiteTexture_.get();

            // Allocate material descriptor set
            VkDescriptorSet materialSet = VK_NULL_HANDLE;
            {
                VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                ai.descriptorPool = materialDescPools_[frameSlot];
                ai.descriptorSetCount = 1;
                ai.pSetLayouts = &materialSetLayout_;
                if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &materialSet) != VK_SUCCESS) {
                    continue;
                }
            }

            // POM quality → sample count
            int pomSamples2 = 32;
            if (pomQuality_ == 0) pomSamples2 = 16;
            else if (pomQuality_ == 2) pomSamples2 = 64;

            CharMaterialUBO matData{};
            matData.opacity = instance.opacity;
            matData.alphaTest = 0;
            matData.colorKeyBlack = 0;
            matData.unlit = 0;
            matData.emissiveBoost = 1.0f;
            matData.emissiveTintR = 1.0f;
            matData.emissiveTintG = 1.0f;
            matData.emissiveTintB = 1.0f;
            matData.specularIntensity = 0.5f;
            matData.enableNormalMap = normalMappingEnabled_ ? 1 : 0;
            matData.enablePOM = pomEnabled_ ? 1 : 0;
            matData.pomScale = 0.06f;
            matData.pomMaxSamples = pomSamples2;
            matData.heightMapVariance = 0.0f;
            matData.normalMapStrength = normalMapStrength_;

            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = sizeof(CharMaterialUBO);
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo allocInfo{};
            ::VkBuffer matUBO = VK_NULL_HANDLE;
            VmaAllocation matUBOAlloc = VK_NULL_HANDLE;
            vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &matUBO, &matUBOAlloc, &allocInfo);
            if (allocInfo.pMappedData) {
                memcpy(allocInfo.pMappedData, &matData, sizeof(CharMaterialUBO));
            }

            VkDescriptorImageInfo imgInfo = texPtr->descriptorInfo();
            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = matUBO;
            bufInfo.offset = 0;
            bufInfo.range = sizeof(CharMaterialUBO);
            VkDescriptorImageInfo nhImgInfo2 = flatNormalTexture_->descriptorInfo();

            VkWriteDescriptorSet writes[3] = {};
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = materialSet;
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[0].pImageInfo = &imgInfo;

            writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[1].dstSet = materialSet;
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[1].pBufferInfo = &bufInfo;

            writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[2].dstSet = materialSet;
            writes[2].dstBinding = 2;
            writes[2].descriptorCount = 1;
            writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[2].pImageInfo = &nhImgInfo2;

            vkUpdateDescriptorSets(vkCtx_->getDevice(), 3, writes, 0, nullptr);

            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    pipelineLayout_, 1, 1, &materialSet, 0, nullptr);

            vkCmdDrawIndexed(cmd, gpuModel.indexCount, 1, 0, 0, 0);

            transientMaterialUbos_[frameSlot].emplace_back(matUBO, matUBOAlloc);
        }
    }
}

bool CharacterRenderer::initializeShadow(VkRenderPass shadowRenderPass) {
    if (!vkCtx_ || shadowRenderPass == VK_NULL_HANDLE) return false;
    VkDevice device = vkCtx_->getDevice();

    // ShadowCharParams UBO (matches character_shadow.frag.glsl set=1 binding=1)
    struct ShadowCharParams {
        int32_t alphaTest = 0;
        int32_t colorKeyBlack = 0;
    };

    // Create ShadowCharParams UBO
    VkBufferCreateInfo bufCI{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufCI.size = sizeof(ShadowCharParams);
    bufCI.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo allocInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufCI, &allocCI,
            &shadowParamsUBO_, &shadowParamsAlloc_, &allocInfo) != VK_SUCCESS) {
        LOG_ERROR("CharacterRenderer: failed to create shadow params UBO");
        return false;
    }
    ShadowCharParams defaultParams{};
    std::memcpy(allocInfo.pMappedData, &defaultParams, sizeof(defaultParams));

    // Descriptor set layout for set 1: binding 0 = sampler2D, binding 1 = ShadowCharParams UBO
    VkDescriptorSetLayoutBinding layoutBindings[2]{};
    layoutBindings[0].binding = 0;
    layoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layoutBindings[0].descriptorCount = 1;
    layoutBindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    layoutBindings[1].binding = 1;
    layoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    layoutBindings[1].descriptorCount = 1;
    layoutBindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutCI{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutCI.bindingCount = 2;
    layoutCI.pBindings = layoutBindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCI, nullptr, &shadowParamsLayout_) != VK_SUCCESS) {
        LOG_ERROR("CharacterRenderer: failed to create shadow params layout");
        return false;
    }

    // Descriptor pool (1 set)
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[1].descriptorCount = 1;
    VkDescriptorPoolCreateInfo poolCI{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolCI.maxSets = 1;
    poolCI.poolSizeCount = 2;
    poolCI.pPoolSizes = poolSizes;
    if (vkCreateDescriptorPool(device, &poolCI, nullptr, &shadowParamsPool_) != VK_SUCCESS) {
        LOG_ERROR("CharacterRenderer: failed to create shadow params pool");
        return false;
    }

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo setAlloc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    setAlloc.descriptorPool = shadowParamsPool_;
    setAlloc.descriptorSetCount = 1;
    setAlloc.pSetLayouts = &shadowParamsLayout_;
    if (vkAllocateDescriptorSets(device, &setAlloc, &shadowParamsSet_) != VK_SUCCESS) {
        LOG_ERROR("CharacterRenderer: failed to allocate shadow params set");
        return false;
    }

    // Write descriptors (white dummy texture + ShadowCharParams UBO)
    VkDescriptorImageInfo imgInfo{};
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgInfo.imageView = whiteTexture_->getImageView();
    imgInfo.sampler = whiteTexture_->getSampler();
    VkDescriptorBufferInfo bufInfo{};
    bufInfo.buffer = shadowParamsUBO_;
    bufInfo.offset = 0;
    bufInfo.range = sizeof(ShadowCharParams);
    VkWriteDescriptorSet writes[2]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = shadowParamsSet_;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &imgInfo;
    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = shadowParamsSet_;
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[1].pBufferInfo = &bufInfo;
    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);

    // Pipeline layout: set 0 = perFrameLayout_ (dummy), set 1 = shadowParamsLayout_, set 2 = boneSetLayout_
    // Push constant: 128 bytes (lightSpaceMatrix + model), VERTEX stage
    VkDescriptorSetLayout setLayouts[] = {perFrameLayout_, shadowParamsLayout_, boneSetLayout_};
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pc.offset = 0;
    pc.size = 128;
    VkPipelineLayoutCreateInfo plCI{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plCI.setLayoutCount = 3;
    plCI.pSetLayouts = setLayouts;
    plCI.pushConstantRangeCount = 1;
    plCI.pPushConstantRanges = &pc;
    if (vkCreatePipelineLayout(device, &plCI, nullptr, &shadowPipelineLayout_) != VK_SUCCESS) {
        LOG_ERROR("CharacterRenderer: failed to create shadow pipeline layout");
        return false;
    }

    // Load character shadow shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/character_shadow.vert.spv")) {
        LOG_ERROR("CharacterRenderer: failed to load character_shadow.vert.spv");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/character_shadow.frag.spv")) {
        LOG_ERROR("CharacterRenderer: failed to load character_shadow.frag.spv");
        vertShader.destroy();
        return false;
    }

    // Character vertex format (CharVertexGPU): stride = 56 bytes
    // loc 0: vec3 aPos          (R32G32B32_SFLOAT, offset 0)
    // loc 1: vec4 aBoneWeights  (R8G8B8A8_UNORM,   offset 12)
    // loc 2: ivec4 aBoneIndices (R8G8B8A8_UINT,    offset 16)
    // loc 3: vec2 aTexCoord     (R32G32_SFLOAT,    offset 32)
    VkVertexInputBindingDescription vertBind{};
    vertBind.binding = 0;
    vertBind.stride = static_cast<uint32_t>(sizeof(CharVertexGPU));
    vertBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::vector<VkVertexInputAttributeDescription> vertAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(CharVertexGPU, position))},
        {1, 0, VK_FORMAT_R8G8B8A8_UNORM,   static_cast<uint32_t>(offsetof(CharVertexGPU, boneWeights))},
        {2, 0, VK_FORMAT_R8G8B8A8_UINT,    static_cast<uint32_t>(offsetof(CharVertexGPU, boneIndices))},
        {3, 0, VK_FORMAT_R32G32_SFLOAT,    static_cast<uint32_t>(offsetof(CharVertexGPU, texCoords))},
    };

    shadowPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({vertBind}, vertAttrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setDepthBias(0.05f, 0.20f)
        .setNoColorAttachment()
        .setLayout(shadowPipelineLayout_)
        .setRenderPass(shadowRenderPass)
        .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
        .build(device);

    vertShader.destroy();
    fragShader.destroy();

    if (!shadowPipeline_) {
        LOG_ERROR("CharacterRenderer: failed to create shadow pipeline");
        return false;
    }
    LOG_INFO("CharacterRenderer shadow pipeline initialized");
    return true;
}

void CharacterRenderer::renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix,
                                     const glm::vec3& shadowCenter, float shadowRadius) {
    if (!shadowPipeline_ || !shadowParamsSet_) return;
    if (instances.empty() || models.empty()) return;

    uint32_t frameIndex = vkCtx_->getCurrentFrame();
    VkDevice device = vkCtx_->getDevice();

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
    // Bind shadow params set at set 1
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
        1, 1, &shadowParamsSet_, 0, nullptr);

    struct ShadowPush { glm::mat4 lightSpaceMatrix; glm::mat4 model; };

    const float shadowRadiusSq = shadowRadius * shadowRadius;
    for (auto& pair : instances) {
        auto& inst = pair.second;
        if (!inst.visible) continue;

        // Distance cull against shadow frustum
        glm::vec3 diff = inst.position - shadowCenter;
        if (glm::dot(diff, diff) > shadowRadiusSq) continue;

        auto modelIt = models.find(inst.modelId);
        if (modelIt == models.end()) continue;
        const M2ModelGPU& gpuModel = modelIt->second;
        if (!gpuModel.vertexBuffer) continue;

        glm::mat4 modelMat = inst.hasOverrideModelMatrix
            ? inst.overrideModelMatrix
            : getModelMatrix(inst);

        // Ensure bone SSBO is allocated and upload bone matrices
        int numBones = std::min(static_cast<int>(inst.boneMatrices.size()), MAX_BONES);
        if (numBones > 0) {
            if (!inst.boneBuffer[frameIndex]) {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = MAX_BONES * sizeof(glm::mat4);
                bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo ai{};
                vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci,
                    &inst.boneBuffer[frameIndex], &inst.boneAlloc[frameIndex], &ai);
                inst.boneMapped[frameIndex] = ai.pMappedData;

                VkDescriptorSetAllocateInfo dsAI{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                dsAI.descriptorPool = boneDescPool_;
                dsAI.descriptorSetCount = 1;
                dsAI.pSetLayouts = &boneSetLayout_;
                VkResult dsRes = vkAllocateDescriptorSets(device, &dsAI, &inst.boneSet[frameIndex]);
                if (dsRes != VK_SUCCESS) {
                    LOG_ERROR("CharacterRenderer[shadow]: bone descriptor allocation failed (instance=",
                              inst.id, ", frame=", frameIndex, ", vk=", static_cast<int>(dsRes), ")");
                    if (inst.boneBuffer[frameIndex]) {
                        vmaDestroyBuffer(vkCtx_->getAllocator(),
                                         inst.boneBuffer[frameIndex], inst.boneAlloc[frameIndex]);
                        inst.boneBuffer[frameIndex] = VK_NULL_HANDLE;
                        inst.boneAlloc[frameIndex] = VK_NULL_HANDLE;
                        inst.boneMapped[frameIndex] = nullptr;
                    }
                }

                if (inst.boneSet[frameIndex]) {
                    VkDescriptorBufferInfo bInfo{};
                    bInfo.buffer = inst.boneBuffer[frameIndex];
                    bInfo.offset = 0;
                    bInfo.range = bci.size;
                    VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                    w.dstSet = inst.boneSet[frameIndex];
                    w.dstBinding = 0;
                    w.descriptorCount = 1;
                    w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    w.pBufferInfo = &bInfo;
                    vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
                }
            }
            if (inst.boneMapped[frameIndex]) {
                memcpy(inst.boneMapped[frameIndex], inst.boneMatrices.data(),
                       numBones * sizeof(glm::mat4));
            }
        }

        if (!inst.boneSet[frameIndex]) continue;

        // Bind bone SSBO at set 2
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
            2, 1, &inst.boneSet[frameIndex], 0, nullptr);

        ShadowPush push{lightSpaceMatrix, modelMat};
        vkCmdPushConstants(cmd, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, 128, &push);

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &gpuModel.vertexBuffer, &offset);
        vkCmdBindIndexBuffer(cmd, gpuModel.indexBuffer, 0, VK_INDEX_TYPE_UINT16);

        bool applyGeosetFilter = !inst.activeGeosets.empty();
        for (const auto& batch : gpuModel.data.batches) {
            uint16_t blendMode = 0;
            if (batch.materialIndex < gpuModel.data.materials.size()) {
                blendMode = gpuModel.data.materials[batch.materialIndex].blendMode;
            }
            if (blendMode >= 2) continue; // skip transparent
            if (applyGeosetFilter &&
                inst.activeGeosets.find(batch.submeshId) == inst.activeGeosets.end()) continue;
            vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
        }
    }
}

glm::mat4 CharacterRenderer::getModelMatrix(const CharacterInstance& instance) const {
    glm::mat4 model = glm::mat4(1.0f);

    // Apply transformations: T * R * S
    model = glm::translate(model, instance.position);

    // Apply rotation (euler angles, Z-up)
    // Convention: yaw around Z, pitch around X, roll around Y.
    model = glm::rotate(model, instance.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));  // Yaw
    model = glm::rotate(model, instance.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));  // Pitch
    model = glm::rotate(model, instance.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));  // Roll

    model = glm::scale(model, glm::vec3(instance.scale));

    return model;
}

void CharacterRenderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.position = position;
    }
}

void CharacterRenderer::setInstanceRotation(uint32_t instanceId, const glm::vec3& rotation) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.rotation = rotation;
    }
}

void CharacterRenderer::moveInstanceTo(uint32_t instanceId, const glm::vec3& destination, float durationSeconds) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;

    auto& inst = it->second;

    // Don't move dead instances (corpses shouldn't slide around)
    if (inst.isDead) return;

    auto pickMoveAnim = [&](bool preferRun) -> uint32_t {
        // Choose movement anim from estimated speed; fall back if missing.
        if (preferRun) {
            if (hasAnimation(instanceId, 5)) return 5; // Run
            if (hasAnimation(instanceId, 4)) return 4; // Walk
        } else {
            if (hasAnimation(instanceId, 4)) return 4; // Walk
            if (hasAnimation(instanceId, 5)) return 5; // Run
        }
        return 0;
    };

    float planarDist = glm::length(glm::vec2(destination.x - inst.position.x,
                                             destination.y - inst.position.y));
    bool synthesizedDuration = false;
    if (durationSeconds <= 0.0f) {
        if (planarDist < 0.01f) {
            // Stop at current location.
            inst.position = destination;
            inst.isMoving = false;
            if (inst.currentAnimationId == 4 || inst.currentAnimationId == 5) {
                playAnimation(instanceId, 0, true);
            }
            return;
        }
        // Some cores send movement-only deltas without spline duration.
        // Synthesize a tiny duration so movement anim/rotation still updates.
        durationSeconds = std::clamp(planarDist / 7.0f, 0.05f, 0.20f);
        synthesizedDuration = true;
    }

    inst.moveStart = inst.position;
    inst.moveEnd = destination;
    inst.moveDuration = durationSeconds;
    inst.moveElapsed = 0.0f;
    inst.isMoving = true;

    // Face toward destination (yaw around Z axis since Z is up)
    glm::vec3 dir = destination - inst.position;
    if (glm::length(glm::vec2(dir.x, dir.y)) > 0.001f) {
        float angle = std::atan2(dir.y, dir.x);
        inst.rotation.z = angle;
    }

    // Play movement animation while moving.
    // Prefer run only when speed is clearly above normal walk pace.
    float moveSpeed = planarDist / std::max(durationSeconds, 0.001f);
    bool preferRun = (!synthesizedDuration && moveSpeed >= 4.5f);
    uint32_t moveAnim = pickMoveAnim(preferRun);
    if (moveAnim != 0 && inst.currentAnimationId != moveAnim) {
        playAnimation(instanceId, moveAnim, true);
    }
}

const pipeline::M2Model* CharacterRenderer::getModelData(uint32_t modelId) const {
    auto it = models.find(modelId);
    if (it == models.end()) return nullptr;
    return &it->second.data;
}

void CharacterRenderer::startFadeIn(uint32_t instanceId, float durationSeconds) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;
    it->second.opacity = 0.0f;
    it->second.fadeInTime = 0.0f;
    it->second.fadeInDuration = durationSeconds;
}

void CharacterRenderer::setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.activeGeosets = geosets;
    }
}

void CharacterRenderer::setGroupTextureOverride(uint32_t instanceId, uint16_t geosetGroup, VkTexture* texture) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.groupTextureOverrides[geosetGroup] = texture;
    }
}

void CharacterRenderer::setTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot, VkTexture* texture) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.textureSlotOverrides[textureSlot] = texture;
    }
}

void CharacterRenderer::clearTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.textureSlotOverrides.erase(textureSlot);
    }
}

void CharacterRenderer::setInstanceVisible(uint32_t instanceId, bool visible) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        if (it->second.visible != visible) {
            LOG_INFO("CharacterRenderer::setInstanceVisible id=", instanceId, " visible=", visible);
        }
        it->second.visible = visible;

        // Also hide/show attached weapons (for first-person mode)
        for (const auto& wa : it->second.weaponAttachments) {
            auto weapIt = instances.find(wa.weaponInstanceId);
            if (weapIt != instances.end()) {
                weapIt->second.visible = visible;
            }
        }
    }
}

void CharacterRenderer::removeInstance(uint32_t instanceId) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;

    LOG_INFO("CharacterRenderer::removeInstance id=", instanceId,
             " pos=(", it->second.position.x, ",", it->second.position.y, ",", it->second.position.z, ")",
             " remaining=", instances.size() - 1,
             " override=", (void*)renderPassOverride_);

    // Remove child attachments first (helmets/weapons), otherwise they leak as
    // orphan render instances when the parent creature despawns.
    auto attachments = it->second.weaponAttachments;
    for (const auto& wa : attachments) {
        removeInstance(wa.weaponInstanceId);
    }

    // Destroy bone buffers for this instance
    destroyInstanceBones(it->second);

    instances.erase(it);
}

bool CharacterRenderer::getAnimationState(uint32_t instanceId, uint32_t& animationId,
                                          float& animationTimeMs, float& animationDurationMs) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    const CharacterInstance& instance = it->second;
    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    const auto& sequences = modelIt->second.data.sequences;
    if (instance.currentSequenceIndex < 0 || instance.currentSequenceIndex >= static_cast<int>(sequences.size())) {
        return false;
    }

    animationId = instance.currentAnimationId;
    animationTimeMs = instance.animationTime;
    animationDurationMs = static_cast<float>(sequences[instance.currentSequenceIndex].duration);
    return true;
}

bool CharacterRenderer::hasAnimation(uint32_t instanceId, uint32_t animationId) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    const auto& sequences = modelIt->second.data.sequences;
    for (const auto& seq : sequences) {
        if (seq.id == animationId) {
            return true;
        }
    }
    return false;
}

bool CharacterRenderer::getAnimationSequences(uint32_t instanceId, std::vector<pipeline::M2Sequence>& out) const {
    out.clear();
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    out = modelIt->second.data.sequences;
    return !out.empty();
}

bool CharacterRenderer::getInstanceModelName(uint32_t instanceId, std::string& modelName) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }
    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }
    modelName = modelIt->second.data.name;
    return !modelName.empty();
}

bool CharacterRenderer::attachWeapon(uint32_t charInstanceId, uint32_t attachmentId,
                                      const pipeline::M2Model& weaponModel, uint32_t weaponModelId,
                                      const std::string& texturePath) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) {
        core::Logger::getInstance().warning("attachWeapon: character instance ", charInstanceId, " not found");
        return false;
    }
    auto& charInstance = charIt->second;
    auto charModelIt = models.find(charInstance.modelId);
    if (charModelIt == models.end()) return false;
    const auto& charModel = charModelIt->second.data;

    // Find bone index for this attachment point
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    bool found = false;

    // Try attachment lookup first
    if (attachmentId < charModel.attachmentLookup.size()) {
        uint16_t attIdx = charModel.attachmentLookup[attachmentId];
        if (attIdx < charModel.attachments.size()) {
            boneIndex = charModel.attachments[attIdx].bone;
            offset = charModel.attachments[attIdx].position;
            found = true;
        }
    }
    // Fallback: scan attachments by id
    if (!found) {
        for (const auto& att : charModel.attachments) {
            if (att.id == attachmentId) {
                boneIndex = att.bone;
                offset = att.position;
                found = true;
                break;
            }
        }
    }
    // Fallback to key-bone lookup only for weapon hand attachment IDs.
    if (!found && (attachmentId == 1 || attachmentId == 2)) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < charModel.bones.size(); i++) {
            if (charModel.bones[i].keyBoneId == targetKeyBone) {
                boneIndex = static_cast<uint16_t>(i);
                found = true;
                break;
            }
        }
    }

    // Validate bone index (bad attachment tables should not silently bind to origin)
    if (found && boneIndex >= charModel.bones.size()) {
        found = false;
    }
    if (!found && (attachmentId == 1 || attachmentId == 2)) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < charModel.bones.size(); i++) {
            if (charModel.bones[i].keyBoneId == targetKeyBone) {
                boneIndex = static_cast<uint16_t>(i);
                offset = glm::vec3(0.0f);
                found = true;
                break;
            }
        }
    }

    if (!found) {
        core::Logger::getInstance().warning("attachWeapon: no bone found for attachment ", attachmentId);
        return false;
    }

    // Remove existing weapon at this attachment point
    detachWeapon(charInstanceId, attachmentId);

    // Load weapon model into renderer
    if (models.find(weaponModelId) == models.end()) {
        if (!loadModel(weaponModel, weaponModelId)) {
            core::Logger::getInstance().warning("attachWeapon: failed to load weapon model ", weaponModelId);
            return false;
        }
    }

    // Apply weapon texture if provided
    if (!texturePath.empty()) {
        VkTexture* texPtr = loadTexture(texturePath);
        if (texPtr != whiteTexture_.get()) {
            setModelTexture(weaponModelId, 0, texPtr);
        }
    }

    // Create weapon instance
    uint32_t weaponInstanceId = createInstance(weaponModelId, glm::vec3(0.0f));
    if (weaponInstanceId == 0) return false;

    // Mark weapon instance as override-positioned
    auto weapIt = instances.find(weaponInstanceId);
    if (weapIt != instances.end()) {
        weapIt->second.hasOverrideModelMatrix = true;
    }

    // Store attachment on parent character instance
    WeaponAttachment wa;
    wa.weaponModelId = weaponModelId;
    wa.weaponInstanceId = weaponInstanceId;
    wa.attachmentId = attachmentId;
    wa.boneIndex = boneIndex;
    wa.offset = offset;
    charInstance.weaponAttachments.push_back(wa);

    core::Logger::getInstance().debug("Attached weapon model ", weaponModelId,
        " to instance ", charInstanceId, " at attachment ", attachmentId,
        " (bone ", boneIndex, ", offset ", offset.x, ",", offset.y, ",", offset.z, ")");
    return true;
}

bool CharacterRenderer::getInstanceBounds(uint32_t instanceId, glm::vec3& outCenter, float& outRadius) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return false;
    auto mIt = models.find(it->second.modelId);
    if (mIt == models.end()) return false;

    const auto& inst = it->second;
    const auto& model = mIt->second.data;

    glm::vec3 localCenter = (model.boundMin + model.boundMax) * 0.5f;
    float radius = model.boundRadius;
    if (radius <= 0.001f) {
        radius = glm::length(model.boundMax - model.boundMin) * 0.5f;
    }

    float scale = std::max(0.001f, inst.scale);
    outCenter = inst.position + localCenter * scale;
    outRadius = std::max(0.5f, radius * scale);
    return true;
}

bool CharacterRenderer::getInstanceFootZ(uint32_t instanceId, float& outFootZ) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return false;
    auto mIt = models.find(it->second.modelId);
    if (mIt == models.end()) return false;

    const auto& inst = it->second;
    const auto& model = mIt->second.data;
    float scale = std::max(0.001f, inst.scale);
    outFootZ = inst.position.z + model.boundMin.z * scale;
    return true;
}

void CharacterRenderer::detachWeapon(uint32_t charInstanceId, uint32_t attachmentId) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) return;
    auto& attachments = charIt->second.weaponAttachments;

    for (auto it = attachments.begin(); it != attachments.end(); ++it) {
        if (it->attachmentId == attachmentId) {
            removeInstance(it->weaponInstanceId);
            attachments.erase(it);
            core::Logger::getInstance().info("Detached weapon from instance ", charInstanceId,
                " attachment ", attachmentId);
            return;
        }
    }
}

bool CharacterRenderer::getAttachmentTransform(uint32_t instanceId, uint32_t attachmentId, glm::mat4& outTransform) {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) return false;
    const auto& instance = instIt->second;

    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) return false;
    const auto& model = modelIt->second.data;

    // Find attachment point
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    bool found = false;

    // Try attachment lookup first
    if (attachmentId < model.attachmentLookup.size()) {
        uint16_t attIdx = model.attachmentLookup[attachmentId];
        if (attIdx < model.attachments.size()) {
            boneIndex = model.attachments[attIdx].bone;
            offset = model.attachments[attIdx].position;
            found = true;
        }
    }

    // Fallback: scan attachments by id
    if (!found) {
        for (const auto& att : model.attachments) {
            if (att.id == attachmentId) {
                boneIndex = att.bone;
                offset = att.position;
                found = true;
                break;
            }
        }
    }

    if (!found) return false;

    // Validate bone index; invalid indices bind attachments to origin (looks like weapons at feet).
    if (boneIndex >= model.bones.size()) {
        // Fallback: key bones (26/27) only for hand attachments.
        if (attachmentId == 1 || attachmentId == 2) {
            int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
            found = false;
            for (size_t i = 0; i < model.bones.size(); i++) {
                if (model.bones[i].keyBoneId == targetKeyBone) {
                    boneIndex = static_cast<uint16_t>(i);
                    offset = glm::vec3(0.0f);
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        } else {
            return false;
        }
    }

    // Get bone matrix
    glm::mat4 boneMat(1.0f);
    if (boneIndex < instance.boneMatrices.size()) {
        boneMat = instance.boneMatrices[boneIndex];
    }

    // Compute world transform: modelMatrix * boneMatrix * offsetTranslation
    glm::mat4 modelMat = instance.hasOverrideModelMatrix
        ? instance.overrideModelMatrix
        : getModelMatrix(instance);

    outTransform = modelMat * boneMat * glm::translate(glm::mat4(1.0f), offset);
    return true;
}

void CharacterRenderer::dumpAnimations(uint32_t instanceId) const {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) {
        core::Logger::getInstance().info("dumpAnimations: instance ", instanceId, " not found");
        return;
    }
    const auto& instance = instIt->second;

    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        core::Logger::getInstance().info("dumpAnimations: model not found for instance ", instanceId);
        return;
    }
    const auto& model = modelIt->second.data;

    core::Logger::getInstance().info("=== Animation dump for ", model.name, " ===");
    core::Logger::getInstance().info("Total animations: ", model.sequences.size());

    for (size_t i = 0; i < model.sequences.size(); i++) {
        const auto& seq = model.sequences[i];
        core::Logger::getInstance().info("  [", i, "] animId=", seq.id,
            " variation=", seq.variationIndex,
            " duration=", seq.duration, "ms",
            " speed=", seq.movingSpeed,
            " flags=0x", std::hex, seq.flags, std::dec);
    }
    core::Logger::getInstance().info("=== End animation dump ===");
}

void CharacterRenderer::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old main-pass pipelines (NOT shadow, NOT pipeline layout)
    if (opaquePipeline_)    { vkDestroyPipeline(device, opaquePipeline_, nullptr); opaquePipeline_ = VK_NULL_HANDLE; }
    if (alphaTestPipeline_) { vkDestroyPipeline(device, alphaTestPipeline_, nullptr); alphaTestPipeline_ = VK_NULL_HANDLE; }
    if (alphaPipeline_)     { vkDestroyPipeline(device, alphaPipeline_, nullptr); alphaPipeline_ = VK_NULL_HANDLE; }
    if (additivePipeline_)  { vkDestroyPipeline(device, additivePipeline_, nullptr); additivePipeline_ = VK_NULL_HANDLE; }

    // --- Load shaders ---
    rendering::VkShaderModule charVert, charFrag;
    charVert.loadFromFile(device, "assets/shaders/character.vert.spv");
    charFrag.loadFromFile(device, "assets/shaders/character.frag.spv");

    if (!charVert.isValid() || !charFrag.isValid()) {
        LOG_ERROR("CharacterRenderer::recreatePipelines: missing required shaders");
        return;
    }

    VkRenderPass mainPass = renderPassOverride_ ? renderPassOverride_ : vkCtx_->getImGuiRenderPass();
    VkSampleCountFlagBits samples = renderPassOverride_ ? msaaSamplesOverride_ : vkCtx_->getMsaaSamples();

    // --- Vertex input ---
    VkVertexInputBindingDescription charBinding{};
    charBinding.binding = 0;
    charBinding.stride = sizeof(CharVertexGPU);
    charBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> charAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(CharVertexGPU, position))},
        {1, 0, VK_FORMAT_R8G8B8A8_UNORM,   static_cast<uint32_t>(offsetof(CharVertexGPU, boneWeights))},
        {2, 0, VK_FORMAT_R8G8B8A8_UINT,     static_cast<uint32_t>(offsetof(CharVertexGPU, boneIndices))},
        {3, 0, VK_FORMAT_R32G32B32_SFLOAT,  static_cast<uint32_t>(offsetof(CharVertexGPU, normal))},
        {4, 0, VK_FORMAT_R32G32_SFLOAT,     static_cast<uint32_t>(offsetof(CharVertexGPU, texCoords))},
        {5, 0, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<uint32_t>(offsetof(CharVertexGPU, tangent))},
    };

    auto buildCharPipeline = [&](VkPipelineColorBlendAttachmentState blendState, bool depthWrite) -> VkPipeline {
        return PipelineBuilder()
            .setShaders(charVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        charFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({charBinding}, charAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, depthWrite, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(blendState)
            .setMultisample(samples)
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    };

    LOG_INFO("CharacterRenderer::recreatePipelines: renderPass=", (void*)mainPass,
             " samples=", static_cast<int>(samples),
             " pipelineLayout=", (void*)pipelineLayout_);

    opaquePipeline_ = buildCharPipeline(PipelineBuilder::blendDisabled(), true);
    alphaTestPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), true);
    alphaPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), false);
    additivePipeline_ = buildCharPipeline(PipelineBuilder::blendAdditive(), false);

    charVert.destroy();
    charFrag.destroy();

    if (!opaquePipeline_ || !alphaTestPipeline_ || !alphaPipeline_ || !additivePipeline_) {
        LOG_ERROR("CharacterRenderer::recreatePipelines FAILED: opaque=", (void*)opaquePipeline_,
                  " alphaTest=", (void*)alphaTestPipeline_,
                  " alpha=", (void*)alphaPipeline_,
                  " additive=", (void*)additivePipeline_,
                  " renderPass=", (void*)mainPass, " samples=", static_cast<int>(samples));
    } else {
        LOG_INFO("CharacterRenderer: pipelines recreated successfully (samples=",
                 static_cast<int>(samples), ")");
    }
}

} // namespace rendering
} // namespace wowee
