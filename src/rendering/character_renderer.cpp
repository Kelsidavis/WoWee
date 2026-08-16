/**
 * CharacterRenderer - GPU rendering of M2 character models with skeletal animation (Vulkan)
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
#include <atomic>
#include "rendering/character_renderer.hpp"
#include "rendering/pom_quality.hpp"
#include "rendering/shadow_params.hpp"
#include "rendering/normal_map.hpp"
#include "rendering/m2_track_sampler.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "core/thread_pool.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/bone_slots.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "rendering/m2_model_classifier.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <climits>
#include <cmath>
#include <filesystem>
#include <future>
#include <numeric>
#include <thread>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <cstring>

namespace wowee {
namespace rendering {

namespace {
size_t approxTextureBytesWithMips(int w, int h) {
    if (w <= 0 || h <= 0) return 0;
    size_t base = static_cast<size_t>(w) * static_cast<size_t>(h) * 4ull;
    return base + (base / 3);  // ~4/3 for mip chain
}

std::string normalizeTexturePathKey(std::string key) {
    std::replace(key.begin(), key.end(), '/', '\\');
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return key;
}

bool isMagentaKeyCandidate(const uint8_t* rgba) {
    const int r = rgba[0];
    const int g = rgba[1];
    const int b = rgba[2];
    const int rbDelta = (r > b) ? (r - b) : (b - r);
    return r >= 170 && b >= 170 && g <= 120 &&
           r >= g + 70 && b >= g + 70 && rbDelta <= 96;
}

bool shouldApplyMagentaKey(const std::string& normalizedPath) {
    return normalizedPath.find("character\\") == 0 ||
           normalizedPath.find("item\\texturecomponents\\") == 0 ||
           normalizedPath.find("item\\objectcomponents\\") == 0 ||
           normalizedPath.find("\\hair") != std::string::npos ||
           normalizedPath.find("hair") == 0 ||
           normalizedPath.find("\\cape\\") != std::string::npos ||
           normalizedPath.find("cape\\") == 0;
}

size_t bleedAndStripMagentaKey(std::vector<uint8_t>& rgba, int width, int height) {
    if (width <= 0 || height <= 0 || rgba.size() < static_cast<size_t>(width) * height * 4) {
        return 0;
    }

    const size_t pixelCount = static_cast<size_t>(width) * height;
    std::vector<uint8_t> source = rgba;
    std::vector<uint8_t> mask(pixelCount, 0);
    size_t stripped = 0;

    for (size_t p = 0; p < pixelCount; ++p) {
        size_t i = p * 4;
        if (!isMagentaKeyCandidate(&source[i])) continue;
        mask[p] = 1;
        ++stripped;
    }

    if (stripped == 0) return 0;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            size_t p = static_cast<size_t>(y) * width + x;
            if (!mask[p]) continue;

            uint32_t rSum = 0;
            uint32_t gSum = 0;
            uint32_t bSum = 0;
            uint32_t samples = 0;

            for (int radius = 1; radius <= 8 && samples == 0; ++radius) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    int ny = y + dy;
                    if (ny < 0 || ny >= height) continue;
                    for (int dx = -radius; dx <= radius; ++dx) {
                        if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
                        int nx = x + dx;
                        if (nx < 0 || nx >= width) continue;

                        size_t np = static_cast<size_t>(ny) * width + nx;
                        if (mask[np]) continue;
                        size_t ni = np * 4;
                        if (source[ni + 3] == 0) continue;
                        rSum += source[ni + 0];
                        gSum += source[ni + 1];
                        bSum += source[ni + 2];
                        ++samples;
                    }
                }
            }

            size_t i = p * 4;
            if (samples > 0) {
                rgba[i + 0] = static_cast<uint8_t>(rSum / samples);
                rgba[i + 1] = static_cast<uint8_t>(gSum / samples);
                rgba[i + 2] = static_cast<uint8_t>(bSum / samples);
            } else {
                rgba[i + 0] = 0;
                rgba[i + 1] = 0;
                rgba[i + 2] = 0;
            }
            rgba[i + 3] = 0;
        }
    }
    return stripped;
}

bool hasNonOpaqueAlpha(const std::vector<uint8_t>& rgba) {
    for (size_t i = 3; i < rgba.size(); i += 4) {
        if (rgba[i] != 255) {
            return true;
        }
    }
    return false;
}

void applyMagentaKeyIfNeeded(pipeline::BLPImage& image, const std::string& path) {
    if (!image.isValid()) return;
    std::string key = normalizeTexturePathKey(path);
    if (!shouldApplyMagentaKey(key)) return;
    bleedAndStripMagentaKey(image.data, image.width, image.height);
}
} // namespace

// Descriptor pool sizing
static constexpr uint32_t MAX_MATERIAL_SETS = 4096;
static constexpr uint32_t MAX_BONE_SETS = 8192;

// Texture compositing sizes (NPC skin upscale)
static constexpr int kBaseTexSize    = 256;  // NPC baked texture default
static constexpr int kUpscaleTexSize = 512;  // Target size for region compositing
static constexpr int32_t kPreviewSimpleTextureMode = -31336;

// WOWEE_SCENE_DIAG=1 - dump what each glue-scene backdrop batch is handed at draw
// time. The scene renders through the character path, so when it comes out wrong
// the question is always which texture, blend mode and shader path it actually got.
static bool sceneDiagEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("WOWEE_SCENE_DIAG");
        return v && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}

// Evaluate a batch's M2 color-alpha track at the given animation sequence/time.
// Returns 1.0 when the batch has no color slot or the track has no usable data.
// Global-sequence-timed tracks are not culled (they run on a different clock and
// typically drive pulsing glows, not visibility gating).
static float evalBatchColorAlpha(const pipeline::M2Model& model,
                                 const pipeline::M2Batch& batch,
                                 int sequenceIndex, float animationTimeMs,
                                 float globalTimeMs) {
    if (batch.colorIndex == 0xFFFF ||
        batch.colorIndex >= model.colorAlphaTracks.size()) {
        return 1.0f;
    }
    return m2_track::sampleFloat(model.colorAlphaTracks[batch.colorIndex],
                                 sequenceIndex, animationTimeMs, globalTimeMs,
                                 model.globalSequenceDurations, 1.0f);
}

// Evaluate the material transparency track selected through the skin batch's
// lookup table. Enchant cards depend on this track for their authored duty
// cycle and strength; treating the first key as a constant makes every pulse
// both brighter and several times more frequent than intended.
static float evalBatchTextureWeight(const pipeline::M2Model& model,
                                    const pipeline::M2Batch& batch,
                                    int sequenceIndex, float animationTimeMs,
                                    float globalTimeMs) {
    if (batch.transparencyIndex == 0xFFFF ||
        batch.transparencyIndex >= model.textureWeightLookup.size()) {
        return 1.0f;
    }
    const uint16_t trackIndex = model.textureWeightLookup[batch.transparencyIndex];
    if (trackIndex == 0xFFFF || trackIndex >= model.textureWeightTracks.size()) {
        return 1.0f;
    }
    return m2_track::sampleFloat(model.textureWeightTracks[trackIndex],
                                 sequenceIndex, animationTimeMs, globalTimeMs,
                                 model.globalSequenceDurations, 1.0f);
}

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
    int32_t hairMaterial;
    float _pad[1];
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

/// Builds the five main-pass pipelines from an already-loaded shader pair.
///
/// initialize() and recreatePipelines() both need exactly these five, blend
/// state apart, and each described the vertex layout and the builder for
/// itself. The layout is the part worth having once: the bone weights and
/// indices are packed bytes, so a description that drifts from the struct
/// feeds the skinning garbage rather than failing to build.
void CharacterRenderer::buildMainPassPipelines(VkDevice device, VkRenderPass mainPass,
                                               VkSampleCountFlagBits samples,
                                               wowee::rendering::VkShaderModule& charVert,
                                               wowee::rendering::VkShaderModule& charFrag) {
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
    auto buildCharPipeline = [&](VkPipelineColorBlendAttachmentState blendState,
                                  bool depthWrite, bool alphaToCoverage = false) -> VkPipeline {
        auto builder = PipelineBuilder()
            .setShaders(charVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        charFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({charBinding}, charAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, depthWrite, VK_COMPARE_OP_LESS)
            .setDepthBias(0.0f, 0.0f)
            .setColorBlendAttachment(blendState)
            .setMultisample(samples);
        if (alphaToCoverage)
            builder.setAlphaToCoverage(true);
        return builder
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_DEPTH_BIAS})
            .build(device, vkCtx_->getPipelineCache());
    };

    opaquePipeline_ = buildCharPipeline(PipelineBuilder::blendDisabled(), true);
    alphaTestPipeline_ = buildCharPipeline(PipelineBuilder::blendDisabled(), true, true);
    alphaPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), false);
    additivePipeline_ = buildCharPipeline(PipelineBuilder::blendAdditive(), false);
    translucentPipeline_ = buildCharPipeline(PipelineBuilder::blendAlpha(), true);

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
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
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
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, MAX_MATERIAL_SETS},
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

    // --- Material UBO ring buffers (one per frame slot) ---
    {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(ctx->getPhysicalDevice(), &props);
        materialUboAlignment_ = static_cast<uint32_t>(props.limits.minUniformBufferOffsetAlignment);
        if (materialUboAlignment_ < 1) materialUboAlignment_ = 1;
        // Round up UBO size to alignment
        uint32_t alignedUboSize = (sizeof(CharMaterialUBO) + materialUboAlignment_ - 1) & ~(materialUboAlignment_ - 1);
        uint32_t ringSize = alignedUboSize * MATERIAL_RING_CAPACITY;
        for (int i = 0; i < 2; i++) {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = ringSize;
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo allocInfo{};
            vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                            &materialRingBuffer_[i], &materialRingAlloc_[i], &allocInfo);
            materialRingMapped_[i] = allocInfo.pMappedData;
        }
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
    if (!charVert.loadFromFile(device, "assets/shaders/character.vert.spv") ||
        !charFrag.loadFromFile(device, "assets/shaders/character.frag.spv")) {
        LOG_ERROR("Character: Missing required shaders, cannot initialize");
        return false;
    }

    VkRenderPass mainPass = renderPassOverride_ ? renderPassOverride_ : vkCtx_->getImGuiRenderPass();
    VkSampleCountFlagBits samples = renderPassOverride_ ? msaaSamplesOverride_ : vkCtx_->getMsaaSamples();

    buildMainPassPipelines(device, mainPass, samples, charVert, charFrag);

    // Clean up shader modules
    charVert.destroy();
    charFrag.destroy();

    createFallbackTextures(device);

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

    // Wait for any in-flight background normal map generation threads
    {
        std::unique_lock<std::mutex> lock(normalMapResultsMutex_);
        normalMapDoneCV_.wait(lock, [this] {
            return pendingNormalMapCount_.load(std::memory_order_acquire) == 0;
        });
    }

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

    // Model and bone destruction above is deferred; drain it now while the
    // descriptor pools are still alive, since no further frames will run.
    vkCtx_->flushDeferredCleanup();

    // ~VkTexture is empty by design -- it has no device or allocator to free
    // with -- so clearing the map drops the unique_ptrs without destroying the
    // image, view or allocation behind each one. Both textures in an entry are
    // owned here.
    for (auto& [path, entry] : textureCache) {
        if (entry.texture) entry.texture->destroy(device, alloc);
        if (entry.normalHeightMap) entry.normalHeightMap->destroy(device, alloc);
    }
    textureCache.clear();
    // The singletons the cache never held. Same reason as above.
    if (whiteTexture_)       { whiteTexture_->destroy(device, alloc);       whiteTexture_.reset(); }
    if (transparentTexture_) { transparentTexture_->destroy(device, alloc); transparentTexture_.reset(); }
    if (flatNormalTexture_)  { flatNormalTexture_->destroy(device, alloc);  flatNormalTexture_.reset(); }
    texturePropsByPtr_.clear();
    normalMapByTexPtr_.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;

    // Clean up composite cache
    compositeCache_.clear();
    failedTextureCache_.clear();
    failedTextureRetryAt_.clear();
    textureLookupSerial_ = 0;

    whiteTexture_.reset();
    transparentTexture_.reset();
    flatNormalTexture_.reset();

    models.clear();
    instances.clear();

    // Destroy pipelines
    auto destroyPipeline = [&](VkPipeline& p) {
        destroy(device, p);
    };
    destroyPipeline(opaquePipeline_);
    destroyPipeline(alphaTestPipeline_);
    destroyPipeline(alphaPipeline_);
    destroyPipeline(additivePipeline_);
    destroyPipeline(translucentPipeline_);

    destroy(device, pipelineLayout_);

    // Destroy material ring buffers
    for (int i = 0; i < 2; i++) {
        if (materialRingBuffer_[i]) {
            vmaDestroyBuffer(alloc, materialRingBuffer_[i], materialRingAlloc_[i]);
            materialRingBuffer_[i] = VK_NULL_HANDLE;
            materialRingAlloc_[i] = VK_NULL_HANDLE;
            materialRingMapped_[i] = nullptr;
        }
        materialRingOffset_[i] = 0;
    }

    // Destroy descriptor pools and layouts
    for (int i = 0; i < 2; i++) {
        destroy(device, materialDescPools_[i]);
    }
    if (boneDescPool_) {
        if (boneDescPoolGeneration_) boneDescPoolGeneration_->fetch_add(1, std::memory_order_relaxed);
        vkDestroyDescriptorPool(device, boneDescPool_, nullptr);
        boneDescPool_ = VK_NULL_HANDLE;
    }
    destroy(device, materialSetLayout_);
    destroy(device, boneSetLayout_);

    // Shadow resources
    destroy(device, shadowPipeline_);
    destroy(device, shadowPipelineLayout_);
    destroyShadowParamsSet(device, alloc, shadowParams_);
    shadowTexSetCache_.clear();
    for (auto& pool : shadowTexPool_) {
        if (pool) { vkDestroyDescriptorPool(device, pool, nullptr); pool = VK_NULL_HANDLE; }
    }

    vkCtx_ = nullptr;
}

void CharacterRenderer::clear() {
    if (!vkCtx_) return;

    LOG_INFO("CharacterRenderer::clear instances=", instances.size(),
             " models=", models.size());

    // Wait for any in-flight background normal map generation threads
    {
        std::unique_lock<std::mutex> lock(normalMapResultsMutex_);
        normalMapDoneCV_.wait(lock, [this] {
            return pendingNormalMapCount_.load(std::memory_order_acquire) == 0;
        });
    }
    // Discard any completed results that haven't been uploaded
    {
        std::lock_guard<std::mutex> lock(normalMapResultsMutex_);
        completedNormalMaps_.clear();
    }

    vkDeviceWaitIdle(vkCtx_->getDevice());
    VkDevice device = vkCtx_->getDevice();

    // Destroy GPU resources for all models
    for (auto& pair : models) {
        destroyModelGPU(pair.second);
    }

    // Destroy bone buffers for all instances
    for (auto& pair : instances) {
        destroyInstanceBones(pair.second);
    }

    // See CharacterRenderer::shutdown: ~VkTexture frees nothing on its own.
    if (vkCtx_) {
        VkDevice dev = vkCtx_->getDevice();
        VmaAllocator vma = vkCtx_->getAllocator();
        for (auto& [path, entry] : textureCache) {
            if (entry.texture) entry.texture->destroy(dev, vma);
            if (entry.normalHeightMap) entry.normalHeightMap->destroy(dev, vma);
        }
    }
    textureCache.clear();
    texturePropsByPtr_.clear();
    normalMapByTexPtr_.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;
    loggedTextureLoadFails_.clear();
    failedTextureRetryAt_.clear();
    textureLookupSerial_ = 0;

    // Clear composite and failed caches
    compositeCache_.clear();
    failedTextureCache_.clear();

    // Recreate default textures (needed by loadModel/loadTexture fallbacks)
    whiteTexture_.reset();
    transparentTexture_.reset();
    flatNormalTexture_.reset();
    createFallbackTextures(device);

    models.clear();
    instances.clear();

    // Reset material ring buffer offsets (buffers persist, just reset write position)
    for (int i = 0; i < 2; i++) {
        materialRingOffset_[i] = 0;
    }

    // Reset descriptor pools (don't destroy - reuse for new allocations)
    for (int i = 0; i < 2; i++) {
        if (materialDescPools_[i]) {
            vkResetDescriptorPool(device, materialDescPools_[i], 0);
        }
    }
    if (boneDescPool_) {
        if (boneDescPoolGeneration_) boneDescPoolGeneration_->fetch_add(1, std::memory_order_relaxed);
        vkResetDescriptorPool(device, boneDescPool_, 0);
    }
}

void CharacterRenderer::createFallbackTextures(VkDevice device) {
    // White: default diffuse when no texture is assigned
    {
        uint8_t white[] = {255, 255, 255, 255};
        whiteTexture_ = std::make_unique<VkTexture>();
        whiteTexture_->upload(*vkCtx_, white, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        whiteTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }
    // Transparent: placeholder for optional overlay layers (e.g. hair highlights)
    {
        uint8_t transparent[] = {0, 0, 0, 0};
        transparentTexture_ = std::make_unique<VkTexture>();
        transparentTexture_->upload(*vkCtx_, transparent, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        transparentTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }
    // Flat normal: neutral normal map (128,128,255) + 0.5 height in alpha channel
    {
        uint8_t flatNormal[] = {128, 128, 255, 128};
        flatNormalTexture_ = std::make_unique<VkTexture>();
        flatNormalTexture_->upload(*vkCtx_, flatNormal, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        flatNormalTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }
}

void CharacterRenderer::destroyModelGPU(M2ModelGPU& gpuModel, bool defer) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();

    // Snapshot raw handles and null the model fields immediately
    ::VkBuffer vb = gpuModel.vertexBuffer;
    VmaAllocation vbAlloc = gpuModel.vertexAlloc;
    ::VkBuffer ib = gpuModel.indexBuffer;
    VmaAllocation ibAlloc = gpuModel.indexAlloc;
    gpuModel.vertexBuffer = VK_NULL_HANDLE;
    gpuModel.vertexAlloc = VK_NULL_HANDLE;
    gpuModel.indexBuffer = VK_NULL_HANDLE;
    gpuModel.indexAlloc = VK_NULL_HANDLE;

    if (!defer) {
        // Safe after vkDeviceWaitIdle (shutdown / clear paths)
        if (vb) vmaDestroyBuffer(alloc, vb, vbAlloc);
        if (ib) vmaDestroyBuffer(alloc, ib, ibAlloc);
    } else if (vb || ib) {
        // Streaming path: in-flight command buffers may still reference these
        vkCtx_->deferAfterAllFrameFences([alloc, vb, vbAlloc, ib, ibAlloc]() {
            if (vb) vmaDestroyBuffer(alloc, vb, vbAlloc);
            if (ib) vmaDestroyBuffer(alloc, ib, ibAlloc);
        });
    }
}

void CharacterRenderer::destroyInstanceBones(CharacterInstance& inst, bool defer) {
    if (!vkCtx_) return;
    releaseInstanceBones(*vkCtx_, boneDescPool_, boneDescPoolGeneration_, inst, defer);
}

std::unique_ptr<VkTexture> CharacterRenderer::generateNormalHeightMap(
        const uint8_t* pixels, uint32_t width, uint32_t height, float& outVariance) {
    if (!vkCtx_ || width == 0 || height == 0) return nullptr;

    // Use the CPU-only static method, then upload to GPU
    std::vector<uint8_t> dummy(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
    std::memcpy(dummy.data(), pixels, dummy.size());
    auto result = generateNormalHeightMapCPU("", std::move(dummy), width, height);
    outVariance = result.variance;

    auto tex = std::make_unique<VkTexture>();
    if (!tex->upload(*vkCtx_, result.pixels.data(), width, height, VK_FORMAT_R8G8B8A8_UNORM, true)) {
        return nullptr;
    }
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                        VK_SAMPLER_ADDRESS_MODE_REPEAT);
    return tex;
}

// Static, thread-safe CPU-only normal map generation (no GPU access)
bool CharacterRenderer::queueNormalMapGeneration(const std::string& cacheKey,
                                                std::vector<uint8_t> pixels,
                                                uint32_t width, uint32_t height) {
    // Every surface this renderer draws derives its normal map from its own
    // diffuse art, and this is the one place that starts that work. It used to
    // be spelled out inside the file-loading path alone - so a texture that
    // never came from a file never got one, and a character's body is exactly
    // that: composited in memory from a skin, a face and whatever armour is
    // worn. The largest lit surface on screen was the one surface with no
    // normal map, which reads as the lighting working everywhere except on
    // people.
    if (width < 32 || height < 32) return false;
    // Use acq_rel so the increment is visible to shutdown()'s acquire load
    // before the thread body begins (relaxed could delay visibility and cause
    // shutdown() to see 0 and proceed while a thread is still running).
    pendingNormalMapCount_.fetch_add(1, std::memory_order_acq_rel);
    auto* self = this;
    std::thread([self, ck = cacheKey, px = std::move(pixels), width, height]() mutable {
        // try-catch guarantees the counter is decremented even if the compute
        // throws (e.g., bad_alloc). Without this, shutdown() would deadlock
        // waiting for a count that never reaches zero.
        try {
            auto result = generateNormalHeightMapCPU(std::move(ck), std::move(px),
                                                     width, height);
            {
                std::lock_guard<std::mutex> lock(self->normalMapResultsMutex_);
                self->completedNormalMaps_.push_back(std::move(result));
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Normal map generation failed: ", e.what());
        }
        if (self->pendingNormalMapCount_.fetch_sub(1, std::memory_order_release) == 1) {
            self->normalMapDoneCV_.notify_one();
        }
    }).detach();
    return true;
}

CharacterRenderer::NormalMapResult CharacterRenderer::generateNormalHeightMapCPU(
        std::string cacheKey, std::vector<uint8_t> srcPixels, uint32_t width, uint32_t height) {
    NormalMapResult result;
    result.cacheKey = std::move(cacheKey);
    result.width = width;
    result.height = height;
    result.variance = 0.0f;
    // Five, where the WMO renderer asks for two: this is skin and cloth seen
    // close up, and it wants the gradient exaggerated.
    result.pixels = rendering::generateNormalHeightMap(
        srcPixels.data(), width, height, /*strength=*/5.0f, result.variance);
    return result;
}

VkTexture* CharacterRenderer::loadTexture(const std::string& path) {
    constexpr uint64_t kFailedTextureRetryLookups = 512;
    // Skip empty or whitespace-only paths (type-0 textures have no filename)
    if (path.empty()) return whiteTexture_.get();
    bool allWhitespace = true;
    for (char c : path) {
        if (c != ' ' && c != '\t' && c != '\0' && c != '\n') { allWhitespace = false; break; }
    }
    if (allWhitespace) return whiteTexture_.get();

    std::string key = normalizeTexturePathKey(path);
    const uint64_t lookupSerial = ++textureLookupSerial_;
    // The same question the M2 renderer asks, through the same answer. This
    // carried four of its eleven tokens and searched the whole path, so which
    // textures were colour-keyed depended on which renderer had loaded them.
    const bool colorKeyBlackHint = assetNameLooksLikeFlame(key);

    // Check cache
    auto it = textureCache.find(key);
    if (it != textureCache.end()) {
        it->second.lastUse = ++textureCacheCounter_;
        return it->second.texture.get();
    }
    auto failIt = failedTextureRetryAt_.find(key);
    if (failIt != failedTextureRetryAt_.end() && lookupSerial < failIt->second) {
        return whiteTexture_.get();
    }

    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture_.get();
    }

    // Check pre-decoded BLP cache first (populated by background threads)
    pipeline::BLPImage blpImage;
    if (predecodedBLPCache_) {
        auto pit = predecodedBLPCache_->find(key);
        if (pit != predecodedBLPCache_->end()) {
            blpImage = std::move(pit->second);
            predecodedBLPCache_->erase(pit);
        }
    }
    if (!blpImage.isValid()) {
        blpImage = assetManager->loadTexture(key);
    }
    if (!blpImage.isValid()) {
        // Cache misses briefly to avoid repeated expensive MPQ/disk probes.
        failedTextureCache_.insert(key);
        failedTextureRetryAt_[key] = lookupSerial + kFailedTextureRetryLookups;
        if (loggedTextureLoadFails_.insert(key).second) {
            core::Logger::getInstance().warning("Failed to load texture: ", path);
        }
        return whiteTexture_.get();
    }

    applyMagentaKeyIfNeeded(blpImage, key);

    size_t approxBytes = approxTextureBytesWithMips(blpImage.width, blpImage.height);
    if (textureCacheBytes_ + approxBytes > textureCacheBudgetBytes_) {
        static constexpr size_t kMaxFailedTextureCache = 200000;
        if (failedTextureCache_.size() < kMaxFailedTextureCache) {
            // Budget is saturated; avoid repeatedly decoding/uploading this texture.
            failedTextureCache_.insert(key);
            failedTextureRetryAt_[key] = lookupSerial + kFailedTextureRetryLookups;
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

    // Launch normal map generation on background thread - CPU work is pure compute,
    // only the GPU upload (in processPendingNormalMaps) needs the main thread (~1-2ms).
    e.normalMapPending = queueNormalMapGeneration(
        key, std::vector<uint8_t>(blpImage.data.begin(), blpImage.data.end()),
        blpImage.width, blpImage.height);

    textureCacheBytes_ += e.approxBytes;
    texturePropsByPtr_[texPtr] = {hasAlpha, colorKeyBlackHint};
    textureCache[key] = std::move(e);
    failedTextureCache_.erase(key);
    failedTextureRetryAt_.erase(key);

    core::Logger::getInstance().debug("Loaded character texture: ", path, " (", blpImage.width, "x", blpImage.height, ")");
    return texPtr;
}

void CharacterRenderer::processPendingNormalMaps(int budget) {
    if (!vkCtx_) return;

    // Collect completed results from background threads
    std::deque<NormalMapResult> ready;
    {
        std::lock_guard<std::mutex> lock(normalMapResultsMutex_);
        if (completedNormalMaps_.empty()) return;
        int count = std::min(budget, static_cast<int>(completedNormalMaps_.size()));
        for (int i = 0; i < count; i++) {
            ready.push_back(std::move(completedNormalMaps_.front()));
            completedNormalMaps_.pop_front();
        }
    }

    // GPU upload only (~1-2ms each) - CPU work already done on background thread
    for (auto& result : ready) {
        auto it = textureCache.find(result.cacheKey);
        if (it == textureCache.end()) continue;  // texture was evicted

        vkCtx_->beginUploadBatch();
        auto tex = std::make_unique<VkTexture>();
        bool ok = tex->upload(*vkCtx_, result.pixels.data(), result.width, result.height,
                              VK_FORMAT_R8G8B8A8_UNORM, true);
        if (ok) {
            tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                               VK_SAMPLER_ADDRESS_MODE_REPEAT);
            it->second.heightMapVariance = result.variance;
            it->second.approxBytes += approxTextureBytesWithMips(result.width, result.height);
            textureCacheBytes_ += approxTextureBytesWithMips(result.width, result.height);
            it->second.normalHeightMap = std::move(tex);
            if (it->second.texture) {
                normalMapByTexPtr_[it->second.texture.get()] = {
                    it->second.normalHeightMap.get(), it->second.heightMapVariance};
            }
        }
        vkCtx_->endUploadBatch();
        it->second.normalMapPending = false;
    }
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
// Blit an overlay resampled to an explicit destination size. The integer-scale
// version below only grows an overlay, and only by a whole factor, so an overlay
// that arrives larger than the region it belongs in was pasted at its own size
// and covered several regions of the atlas. Nearest-neighbour is enough here:
// these are small atlas patches, and the alternative was no resize at all.
static void blitOverlayResampled(std::vector<uint8_t>& composite, int compW, int compH,
                                 const pipeline::BLPImage& overlay,
                                 int dstX, int dstY, int dstW, int dstH) {
    if (dstW <= 0 || dstH <= 0 || overlay.width <= 0 || overlay.height <= 0) return;
    for (int y = 0; y < dstH; ++y) {
        const int dy = dstY + y;
        if (dy < 0 || dy >= compH) continue;
        const int sy = std::min(overlay.height - 1, y * overlay.height / dstH);
        for (int x = 0; x < dstW; ++x) {
            const int dx = dstX + x;
            if (dx < 0 || dx >= compW) continue;
            const int sx = std::min(overlay.width - 1, x * overlay.width / dstW);

            const size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            const uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            const size_t dstIdx = (static_cast<size_t>(dy) * compW + dx) * 4;
            if (srcA == 255) {
                composite[dstIdx + 0] = overlay.data[srcIdx + 0];
                composite[dstIdx + 1] = overlay.data[srcIdx + 1];
                composite[dstIdx + 2] = overlay.data[srcIdx + 2];
                composite[dstIdx + 3] = 255;
            } else {
                const float alpha = srcA / 255.0f;
                const float inv = 1.0f - alpha;
                for (int c = 0; c < 3; ++c) {
                    composite[dstIdx + c] = static_cast<uint8_t>(
                        overlay.data[srcIdx + c] * alpha + composite[dstIdx + c] * inv);
                }
                composite[dstIdx + 3] = std::max(composite[dstIdx + 3], srcA);
            }
        }
    }
}

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

// Nearest-neighbor downscale blit: sample every Nth pixel from overlay
static void blitOverlayDownscaleN(std::vector<uint8_t>& composite, int compW, int compH,
                                   const pipeline::BLPImage& overlay, int dstX, int dstY, int scale) {
    if (scale < 2) { blitOverlay(composite, compW, compH, overlay, dstX, dstY); return; }
    int outW = overlay.width / scale;
    int outH = overlay.height / scale;
    for (int oy = 0; oy < outH; oy++) {
        int dy = dstY + oy;
        if (dy < 0 || dy >= compH) continue;
        for (int ox = 0; ox < outW; ox++) {
            int dx = dstX + ox;
            if (dx < 0 || dx >= compW) continue;

            int sx = ox * scale;
            int sy = oy * scale;
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

namespace {

/// Where a character-texture overlay belongs on the body atlas, in the
/// coordinates of the 256x256 reference atlas everything is expressed in.
///
/// One table, consulted twice: once to work out how big the canvas has to be
/// for the art being laid on it, and once to place each layer. It was written
/// out inline at the placement site alone, and then the size question could
/// only be answered by guessing.
struct AtlasRegion256 { int x, y, w, h; bool known; };

AtlasRegion256 regionFor(const std::string& pathLower) {
    if (pathLower.find("faceupper") != std::string::npos) return {  0, 160, 128, 32, true};
    if (pathLower.find("facelower") != std::string::npos) return {  0, 192, 128, 64, true};
    if (pathLower.find("pelvis")    != std::string::npos) return {128,  96, 128, 64, true};
    if (pathLower.find("torso")     != std::string::npos) return {128,   0, 128, 64, true};
    if (pathLower.find("armupper")  != std::string::npos) return {  0,   0, 128, 64, true};
    if (pathLower.find("armlower")  != std::string::npos) return {  0,  64, 128, 64, true};
    if (pathLower.find("hand")      != std::string::npos) return {  0, 128, 128, 32, true};
    if (pathLower.find("foot")      != std::string::npos ||
        pathLower.find("feet")      != std::string::npos) return {128, 224, 128, 32, true};
    if (pathLower.find("legupper")  != std::string::npos ||
        pathLower.find("leg")       != std::string::npos) return {128, 160, 128, 64, true};
    return {0, 0, 0, 0, false};
}

std::string lowerPath(const std::string& s) {
    std::string out = s;
    for (auto& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

/// The atlas scale a layer of this size implies for its region. A face authored
/// at 512x256 belongs in a 128x64 region, so it is asking for a 1024 atlas.
int impliedScale(const AtlasRegion256& region, int overlayWidth) {
    if (!region.known || region.w <= 0 || overlayWidth <= 0) return 1;
    int scale = overlayWidth / region.w;
    return scale < 1 ? 1 : scale;
}

}  // namespace

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
    pipeline::BLPImage base;
    if (predecodedBLPCache_) {
        std::string key = layerPaths[0];
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        auto pit = predecodedBLPCache_->find(key);
        if (pit != predecodedBLPCache_->end()) {
            base = std::move(pit->second);
            predecodedBLPCache_->erase(pit);
        }
    }
    if (!base.isValid()) base = assetManager->loadTexture(layerPaths[0]);
    if (!base.isValid()) {
        core::Logger::getInstance().warning("Composite: failed to load base layer: ", layerPaths[0]);
        return whiteTexture_.get();
    }
    applyMagentaKeyIfNeeded(base, layerPaths[0]);

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

    // Load every overlay before deciding how big the canvas is.
    //
    // The body decided the atlas size on its own, and each overlay was then
    // squeezed into whatever region that gave it. An HD art set ships a body at
    // the size the stock one uses and a face at twice it - so a 512x256 face was
    // resampled down to 256x128 to fit a 512 body, which throws away exactly the
    // detail the art exists for and lands it soft next to a crisp body.
    //
    // The canvas is sized to the most demanding layer instead. Nothing changes
    // for art that agrees with its body, which is every set that shipped with
    // the game; a set that asks for more gets a bigger atlas and is placed at
    // its own resolution.
    struct LoadedOverlay { std::string path; pipeline::BLPImage image; };
    std::vector<LoadedOverlay> overlays;
    overlays.reserve(layerPaths.size());
    int requiredScale = coordScale;
    int largestRegionScale = 1;
    bool sawRegionLayer = false;
    for (size_t layer = 1; layer < layerPaths.size(); layer++) {
        if (layerPaths[layer].empty()) continue;
        pipeline::BLPImage overlay;
        if (predecodedBLPCache_) {
            std::string key = layerPaths[layer];
            std::replace(key.begin(), key.end(), '/', '\\');
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            auto pit = predecodedBLPCache_->find(key);
            if (pit != predecodedBLPCache_->end()) {
                overlay = std::move(pit->second);
                predecodedBLPCache_->erase(pit);
            }
        }
        if (!overlay.isValid()) overlay = assetManager->loadTexture(layerPaths[layer]);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("Composite: FAILED to load overlay: ",
                                                layerPaths[layer]);
            continue;
        }
        applyMagentaKeyIfNeeded(overlay, layerPaths[layer]);
        // A full-atlas layer speaks for itself and is not a region at all.
        if (overlay.width != width || overlay.height != height) {
            const AtlasRegion256 region = regionFor(lowerPath(layerPaths[layer]));
            if (region.known) {
                const int want = impliedScale(region, overlay.width);
                if (want > largestRegionScale) largestRegionScale = want;
                sawRegionLayer = true;
            }
        }
        overlays.push_back({layerPaths[layer], std::move(overlay)});
    }

    // Grow to the most demanding layer.
    //
    // A layer that does not fit its region is not merely higher resolution. A
    // draenei's HD faceLower is a front-facing head where the stock one is a
    // side profile, and a tauren's is a muzzle seen head on where the stock one
    // is the side of a head - different pictures, not larger ones, and shrinking
    // them into a region sized for the stock art puts the wrong thing on the
    // face. At 512x256 they are exactly their region on a 1024 atlas, which is
    // the size their scale is asking for.
    //
    // The cost is the layers that do fit: a body, a pelvis and a torso authored
    // for a 512 atlas get upscaled to sit beside them. That is softer, and
    // softer in the right place beats sharp in the wrong one. It applies only
    // to the four race and sex pairs that ship such a layer; every other set
    // agrees with its body and is untouched.
    if (sawRegionLayer && largestRegionScale > coordScale) {
        requiredScale = largestRegionScale;
    }

    if (requiredScale > coordScale) {
        const int newSize = 256 * requiredScale;
        std::vector<uint8_t> grown(static_cast<size_t>(newSize) * newSize * 4);
        const int factor = newSize / width;
        for (int y = 0; y < newSize; y++) {
            const int srcY = y / factor;
            for (int x = 0; x < newSize; x++) {
                const int srcIdx = (srcY * width + x / factor) * 4;
                const int dstIdx = (y * newSize + x) * 4;
                grown[dstIdx + 0] = composite[srcIdx + 0];
                grown[dstIdx + 1] = composite[srcIdx + 1];
                grown[dstIdx + 2] = composite[srcIdx + 2];
                grown[dstIdx + 3] = composite[srcIdx + 3];
            }
        }
        core::Logger::getInstance().info("Composite: body is ", width, "x", height,
                                         " but its art asks for ", newSize, "x", newSize,
                                         " - growing the atlas to keep the detail");
        composite = std::move(grown);
        width = height = newSize;
        coordScale = requiredScale;
    }

    // Alpha-blend each overlay onto the composite
    for (auto& loaded : overlays) {
        const pipeline::BLPImage& overlay = loaded.image;

        core::Logger::getInstance().info("Composite: overlay ", loaded.path,
            " (", overlay.width, "x", overlay.height, ")");

        if (overlay.width == width && overlay.height == height) {
            // Same size: full alpha-blend
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // Where this layer belongs, from the one table that also sized the
            // canvas above.
            const AtlasRegion256 region = regionFor(lowerPath(loaded.path));
            if (!region.known) {
                // Unknown -- center placement as fallback
                const int cx = (width - overlay.width) / 2;
                const int cy = (height - overlay.height) / 2;
                core::Logger::getInstance().info("Composite: UNKNOWN region for '",
                    loaded.path, "', centering at (", cx, ",", cy, ")");
                blitOverlay(composite, width, height, overlay, cx, cy);
                continue;
            }
            int dstX = region.x, dstY = region.y;
            const int expectedW256 = region.w, expectedH256 = region.h;

            // Scale coordinates from 256-base to actual canvas
            dstX *= coordScale;
            dstY *= coordScale;

            // The region is a fixed fraction of the atlas, so the overlay has to
            // land at exactly this size whatever resolution it was authored at.
            // Growing it by a whole factor was not enough, because it only ever
            // grew: an overlay arriving larger than its region was pasted at its
            // own size, spilling across neighbouring regions and dragging every
            // feature on the head to the wrong scale. These assets ship at two
            // resolutions - a 256-wide face belongs in a 128-wide slot on a
            // 256-wide atlas - so the two can meet whenever a lookup resolves
            // the body and the face from different sets.
            const int expectedW = expectedW256 * coordScale;
            const int expectedH = expectedH256 * coordScale;
            const bool needsResample =
                (overlay.width != expectedW || overlay.height != expectedH);

            if (needsResample) {
                // Resampling here means this overlay was authored for a different
                // atlas size than the body it is going onto - the two came from
                // different art sets. It will be placed correctly, but a quarter
                // resolution face stretched over an HD head is soft and muddy
                // next to a crisp body, and that reads as the face not fitting.
                core::Logger::getInstance().warning(
                    "Composite: '", loaded.path, "' is ", overlay.width, "x",
                    overlay.height, " but its region on this ", width, "x", height,
                    " body is ", expectedW, "x", expectedH,
                    " - mismatched art sets; resampling to fit");
            } else {
                core::Logger::getInstance().info("Composite: placing '", loaded.path,
                    "' (", overlay.width, "x", overlay.height,
                    ") at (", dstX, ",", dstY, ") on ", width, "x", height);
            }

            if (needsResample) {
                blitOverlayResampled(composite, width, height, overlay,
                                     dstX, dstY, expectedW, expectedH);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        }
    }

    bleedAndStripMagentaKey(composite, width, height);
    const bool hasAlpha = hasNonOpaqueAlpha(composite);

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
    e.hasAlpha = hasAlpha;
    e.colorKeyBlack = false;
    texturePropsByPtr_[texPtr] = {hasAlpha, false};
    // No derived normal map for a composited body, and this is why: the
    // derivation reads luminance as height, which holds for stone and bark and
    // does not hold for skin. Every freckle, every painted shadow under a
    // collarbone, becomes a ridge - and on a character that reads as stretch
    // marks. Art authored as a surface gets one; art authored as a person does
    // not.
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
    static constexpr int regionCoords256[][2] = {
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

    pipeline::BLPImage base;
    if (predecodedBLPCache_) {
        std::string key = basePath;
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        auto pit = predecodedBLPCache_->find(key);
        if (pit != predecodedBLPCache_->end()) {
            base = std::move(pit->second);
            predecodedBLPCache_->erase(pit);
        }
    }
    if (!base.isValid()) base = assetManager->loadTexture(basePath);
    if (!base.isValid()) {
        return whiteTexture_.get();
    }
    applyMagentaKeyIfNeeded(base, basePath);

    std::vector<uint8_t> composite;
    int width = base.width;
    int height = base.height;

    // No atlas growth on this path, and the reason is the equipment.
    //
    // Growing it here scales the region coordinates but not the art placed into
    // them: the equipment layers are authored for a 512 atlas, so on a 1024 one
    // they land at quarter size in the right corner of the right region, and a
    // fully armoured NPC comes out naked with fragments of armour scattered
    // over it. That is what growing this path did, and it is worse than what it
    // was meant to fix.
    //
    // The face it was meant to fix is a dwarf's, and a dwarf's HD faceLower is
    // a faithful 2x of the stock one - same parts in the same places - so
    // resampling it down gives back the stock picture. Softer, and right.
    // compositeTextures still grows, because it has no equipment to misplace.

    // If base texture is 256x256 (e.g., baked NPC texture), upscale to 512x512
    // so equipment regions can be composited at correct coordinates
    if (width == kBaseTexSize && height == kBaseTexSize && !regionLayers.empty()) {
        width = kUpscaleTexSize;
        height = kUpscaleTexSize;
        composite.resize(width * height * 4);
        // Simple 2x nearest-neighbor upscale
        for (int y = 0; y < kUpscaleTexSize; y++) {
            for (int x = 0; x < kUpscaleTexSize; x++) {
                int srcX = x / 2;
                int srcY = y / 2;
                int srcIdx = (srcY * kBaseTexSize + srcX) * 4;
                int dstIdx = (y * kUpscaleTexSize + x) * 4;
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
    bool upscaled = (base.width == kBaseTexSize && base.height == kBaseTexSize && width == kUpscaleTexSize);
    for (const auto& ul : baseLayers) {
        if (ul.empty()) continue;
        pipeline::BLPImage overlay;
        if (predecodedBLPCache_) {
            std::string key = ul;
            std::replace(key.begin(), key.end(), '/', '\\');
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            auto pit = predecodedBLPCache_->find(key);
            if (pit != predecodedBLPCache_->end()) {
                overlay = std::move(pit->second);
                predecodedBLPCache_->erase(pit);
            }
        }
        if (!overlay.isValid()) overlay = assetManager->loadTexture(ul);
        if (!overlay.isValid()) continue;
        applyMagentaKeyIfNeeded(overlay, ul);

        if (overlay.width == width && overlay.height == height) {
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // WoW 256-scale atlas coordinates (from CharComponentTextureSections)
            int dstX = 0, dstY = 0;
            std::string pathLower = ul;
            for (auto& c : pathLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

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
    static constexpr int regionSizes256[][2] = {
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

        pipeline::BLPImage overlay;
        if (predecodedBLPCache_) {
            std::string key = rl.second;
            std::replace(key.begin(), key.end(), '/', '\\');
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            auto pit = predecodedBLPCache_->find(key);
            if (pit != predecodedBLPCache_->end()) {
                overlay = std::move(pit->second);
                predecodedBLPCache_->erase(pit);
            }
        }
        if (!overlay.isValid()) overlay = assetManager->loadTexture(rl.second);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("compositeWithRegions: failed to load ", rl.second);
            continue;
        }
        applyMagentaKeyIfNeeded(overlay, rl.second);

        int dstX = regionCoords256[regionIdx][0] * scaleX;
        int dstY = regionCoords256[regionIdx][1] * scaleY;

        // Expected full-resolution size for this region at current atlas scale
        int expectedW = regionSizes256[regionIdx][0] * scaleX;
        int expectedH = regionSizes256[regionIdx][1] * scaleY;
        if (overlay.width == expectedW && overlay.height == expectedH) {
            // Exact match - blit 1:1
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        } else if (overlay.width * 2 == expectedW && overlay.height * 2 == expectedH) {
            // Overlay is half size - upscale 2x
            blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
        } else if (overlay.width > expectedW && overlay.height > expectedH &&
                   expectedW > 0 && expectedH > 0) {
            // Overlay is larger than region (e.g. HD textures for 1024 atlas on 512 canvas)
            // Downscale to fit
            int dsX = overlay.width / expectedW;
            int dsY = overlay.height / expectedH;
            int ds = std::min(dsX, dsY);
            if (ds >= 2) {
                blitOverlayDownscaleN(composite, width, height, overlay, dstX, dstY, ds);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        } else {
            // Size mismatch - blit at natural size (may clip or leave gap)
            core::Logger::getInstance().warning("compositeWithRegions: region ", regionIdx,
                " at (", dstX, ",", dstY, ") overlay=", overlay.width, "x", overlay.height,
                " expected=", expectedW, "x", expectedH, " from ", rl.second);
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }
    }

    bleedAndStripMagentaKey(composite, width, height);
    const bool hasAlpha = hasNonOpaqueAlpha(composite);

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
    entry.hasAlpha = hasAlpha;
    entry.colorKeyBlack = false;
    texturePropsByPtr_[texPtr] = {hasAlpha, false};
    // Skin again, with armour composited onto it. Same reason as above.
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
bool CharacterRenderer::loadModel(const pipeline::M2Model& model, uint32_t id) {
    if (!model.isValid()) {
        core::Logger::getInstance().error("Cannot load invalid M2 model");
        return false;
    }

    auto existingIt = models.find(id);
    if (existingIt != models.end()) {
        core::Logger::getInstance().warning("Model ID ", id, " already loaded, replacing");
        destroyModelGPU(existingIt->second, /*defer=*/true);
        models.erase(existingIt);
    }

    M2ModelGPU gpuModel;
    gpuModel.data = model;
    if (!model.vertices.empty()) {
        gpuModel.visualBoundMin = glm::vec3(std::numeric_limits<float>::max());
        gpuModel.visualBoundMax = glm::vec3(-std::numeric_limits<float>::max());
        for (const auto& vertex : model.vertices) {
            gpuModel.visualBoundMin = glm::min(gpuModel.visualBoundMin, vertex.position);
            gpuModel.visualBoundMax = glm::max(gpuModel.visualBoundMax, vertex.position);
        }
        gpuModel.visualBoundRadius =
            glm::length(gpuModel.visualBoundMax - gpuModel.visualBoundMin) * 0.5f;
    }
    const auto classification = classifyM2Model(
        model.name, model.boundMin, model.boundMax,
        model.vertices.size(), model.particleEmitters.size());
    gpuModel.isSkyBird = classification.isSkyBird;

    // Batch all GPU uploads (VB, IB, textures) into a single command buffer
    // submission with one fence wait, instead of one fence wait per upload.
    vkCtx_->beginUploadBatch();

    // Setup GPU buffers
    setupModelBuffers(gpuModel);

    // Calculate bind pose
    calculateBindPose(gpuModel);

    // Load textures from model
    for (const auto& tex : model.textures) {
        VkTexture* texPtr = loadTexture(tex.filename);
        gpuModel.textureIds.push_back(texPtr);
    }

    vkCtx_->endUploadBatch();

    // Precompute batch render order (priorityPlane, materialLayer). The result
    // depends only on the model, so caching it here removes the per-frame
    // per-instance allocate + sort from render().
    gpuModel.sortedBatchIndices.resize(gpuModel.data.batches.size());
    std::iota(gpuModel.sortedBatchIndices.begin(), gpuModel.sortedBatchIndices.end(), 0);
    std::stable_sort(gpuModel.sortedBatchIndices.begin(), gpuModel.sortedBatchIndices.end(),
        [&batches = gpuModel.data.batches](size_t a, size_t b) {
            const auto& ba = batches[a];
            const auto& bb = batches[b];
            if (ba.priorityPlane != bb.priorityPlane)
                return ba.priorityPlane < bb.priorityPlane;
            return ba.materialLayer < bb.materialLayer;
        });

    {
        std::string lowerName = gpuModel.data.name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.isKoboldFlame =
            (lowerName.find("kobold") != std::string::npos) &&
            ((lowerName.find("candle") != std::string::npos) ||
             (lowerName.find("torch") != std::string::npos) ||
             (lowerName.find("mine") != std::string::npos));
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
    size_t numBones = model.bones.size();
    int outOfRangeCount = 0, nonzeroWeightOOR = 0;
    for (size_t i = 0; i < vertCount; i++) {
        const auto& src = model.vertices[i];
        auto& dst = gpuVerts[i];
        dst.position = src.position;
        std::memcpy(dst.boneWeights, src.boneWeights, 4);
        std::memcpy(dst.boneIndices, src.boneIndices, 4);
        dst.normal = src.normal;
        dst.texCoords = src.texCoords[0]; // Use first UV set
        dst.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f); // default

        // Diagnostic: check bone indices
        for (int j = 0; j < 4; j++) {
            uint8_t bi = src.boneIndices[j];
            uint8_t bw = src.boneWeights[j];
            if (bi >= numBones) {
                outOfRangeCount++;
                if (bw > 0) nonzeroWeightOOR++;
            }
        }
    }
    // A bone index past the end of the bone list, which is a broken model or a
    // misread one.
    //
    // An index at or above 128 is not that: it is what any model with more than
    // 128 bones has, and the character models have 219. Warning on it fired on
    // every character in the world and reported outOfRange=0 every time - a
    // line per model saying nothing was wrong, in a log whose whole value is
    // that what is in it is.
    if (outOfRangeCount > 0) {
        LOG_WARNING("Model has bone indices past its bone list: bones=", numBones,
                    " verts=", vertCount, " outOfRange=", outOfRangeCount,
                    " (nonzeroWeight=", nonzeroWeightOOR, ")",
                    " - those vertices skin to nothing and collapse to the origin");
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
    auto modelIt = models.find(modelId);
    if (modelIt == models.end()) {
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
    auto& gpuRef = modelIt->second;
    instance.boneMatrices.resize(std::max(static_cast<size_t>(1), gpuRef.data.bones.size()), glm::mat4(1.0f));
    instance.cachedModel = &gpuRef;

    uint32_t id = instance.id;
    instances[id] = std::move(instance);
    return id;
}

void CharacterRenderer::playAnimation(uint32_t instanceId, uint32_t animationId, bool loop,
                                      uint32_t oneShotReturnAnim) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        core::Logger::getInstance().warning("Cannot play animation: instance ", instanceId, " not found");
        return;
    }

    auto& instance = it->second;
    auto& model = models[instance.modelId].data;
    instance.oneShotReturnAnim = loop ? 0 : oneShotReturnAnim;

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

    // Prefer variationIndex==0 (primary animation); fall back to first match
    int firstMatch = -1;
    for (size_t i = 0; i < model.sequences.size(); i++) {
        if (model.sequences[i].id == animationId) {
            if (firstMatch < 0) firstMatch = static_cast<int>(i);
            if (model.sequences[i].variationIndex == 0) {
                instance.currentSequenceIndex = static_cast<int>(i);
                break;
            }
        }
    }
    if (instance.currentSequenceIndex < 0 && firstMatch >= 0) {
        instance.currentSequenceIndex = firstMatch;
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
    // Creature birds are rendered by this path rather than M2Renderer.  Their
    // rapid wing motion remains conspicuous at distance, and the render radius
    // can be larger than the generic animation radius.  Use the same boundary
    // as rendering so a visible bird never holds its last bone pose.
    const float birdUpdateRadius = static_cast<float>(envSizeOrDefault("WOWEE_CHAR_RENDER_RADIUS", 130));
    const float birdUpdateRadiusSq = birdUpdateRadius * birdUpdateRadius;

    // Single pass: fade-in, movement, and animation bone collection
    toUpdate_.clear();

    for (auto& pair : instances) {
        auto& inst = pair.second;

        // Update fade-in opacity
        if (inst.fadeInDuration > 0.0f && inst.opacity < 1.0f) {
            inst.fadeInTime += deltaTime;
            inst.opacity = std::min(1.0f, inst.fadeInTime / inst.fadeInDuration);
            if (inst.opacity >= 1.0f) {
                inst.fadeInDuration = 0.0f;
            }
        }

        // Interpolate creature movement
        if (inst.isMoving) {
            inst.moveElapsed += deltaTime;
            float t = inst.moveElapsed / inst.moveDuration;
            if (t >= 1.0f) {
                inst.position = inst.moveEnd;
                inst.isMoving = false;
            } else {
                inst.position = glm::mix(inst.moveStart, inst.moveEnd, t);
            }
        }

        // Skip weapon instances for animation - their transforms are set by parent
        // bones. Enchant visuals are the exception: they are pure animated FX.
        if (inst.hasOverrideModelMatrix && !inst.isEffectModel) continue;

        float distSq = glm::distance2(inst.position, cameraPos);
        const bool isSkyBird = inst.cachedModel && inst.cachedModel->isSkyBird;
        const float updateRadiusSq = isSkyBird ? birdUpdateRadiusSq : animUpdateRadiusSq;
        if (distSq > updateRadiusSq && !inst.isSceneModel) continue;

        // Advance global sequence timer (accumulates independently of animation wrapping)
        inst.globalSequenceTime += deltaTime * 1000.0f;

        // Always advance animation time (cheap)
        if (inst.cachedModel && !inst.cachedModel->data.sequences.empty()) {
            if (inst.currentSequenceIndex < 0) {
                inst.currentSequenceIndex = 0;
                inst.currentAnimationId = inst.cachedModel->data.sequences[0].id;
            }
            const auto& seq = inst.cachedModel->data.sequences[inst.currentSequenceIndex];
            inst.animationTime += deltaTime * 1000.0f;
            if (seq.duration > 0 && inst.animationTime >= static_cast<float>(seq.duration)) {
                if (inst.animationLoop) {
                    // Subtract duration instead of fmod to preserve float precision
                    // fmod() loses precision with large animationTime values
                    inst.animationTime -= static_cast<float>(seq.duration);
                    // Clamp to [0, duration) to handle multiple loops in one frame
                    while (inst.animationTime >= static_cast<float>(seq.duration)) {
                        inst.animationTime -= static_cast<float>(seq.duration);
                    }
                } else {
                    // One-shot animation finished: return to the caller-specified
                    // resume anim (NPC state emote) or Stand, unless dead
                    if (inst.currentAnimationId != anim::DEATH) {
                        const uint32_t resumeAnim =
                            inst.oneShotReturnAnim != 0 ? inst.oneShotReturnAnim : anim::STAND;
                        playAnimation(pair.first, resumeAnim, true);
                    } else {
                        // Stay on last frame of death
                        inst.animationTime = static_cast<float>(seq.duration);
                    }
                }
            }
        }

        // Keep combat-range creatures at the render frame rate. Aggressive
        // throttling used to begin at 10 yards and reached every eighth frame
        // at 40 yards, making ordinary monster locomotion look like a slideshow.
        // Reserve frame skipping for models far enough away that bone detail is
        // much less noticeable, consistent with the generic M2 renderer.
        uint32_t boneInterval = 1;
        if (!isSkyBird) {
            if (distSq > 90.0f * 90.0f) boneInterval = 4;
            else if (distSq > 45.0f * 45.0f) boneInterval = 2;
        }

        inst.boneUpdateCounter++;
        bool needsBones = (inst.boneUpdateCounter >= boneInterval) || inst.boneMatrices.empty();
        if (needsBones) {
            inst.boneUpdateCounter = 0;
            toUpdate_.push_back(std::ref(inst));
        }
    }

    const size_t updatedCount = toUpdate_.size();

    // Thread bone matrix computation in chunks
    if (updatedCount >= 8 && numAnimThreads_ > 1) {
        static const size_t minAnimWorkPerThread = std::max<size_t>(
            8, envSizeOrDefault("WOWEE_CHAR_ANIM_WORK_PER_THREAD", 16));
        const size_t maxUsefulThreads = std::max<size_t>(
            1, (updatedCount + minAnimWorkPerThread - 1) / minAnimWorkPerThread);
        const size_t numThreads = std::min(static_cast<size_t>(numAnimThreads_), maxUsefulThreads);

        if (numThreads <= 1) {
            for (auto& instRef : toUpdate_) {
                calculateBoneMatrices(instRef.get());
            }
        } else {
            const size_t chunkSize = updatedCount / numThreads;
            const size_t remainder = updatedCount % numThreads;

            auto processRange = [this](size_t begin, size_t end) {
                for (size_t i = begin; i < end; i++) {
                    calculateBoneMatrices(toUpdate_[i].get());
                }
            };

            animFutures_.clear();
            if (animFutures_.capacity() < numThreads) {
                animFutures_.reserve(numThreads);
            }

            // Dispatch all but the last chunk to the shared pool; process the
            // last chunk on this thread so the pool is never a hard dependency
            // for finishing this frame's bone work.
            size_t start = 0;
            for (size_t t = 0; t + 1 < numThreads; t++) {
                size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                animFutures_.push_back(core::ThreadPool::frameWorkers().submit(
                    [processRange, start, end]() { processRange(start, end); }));
                start = end;
            }
            processRange(start, updatedCount);

            for (auto& f : animFutures_) {
                f.get();
            }
        }
    } else {
        for (auto& instRef : toUpdate_) {
            calculateBoneMatrices(instRef.get());
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

        for (auto& wa : instance.weaponAttachments) {
            auto weapIt = instances.find(wa.weaponInstanceId);
            if (weapIt == instances.end()) continue;

            // Get the bone matrix for the attachment bone
            glm::mat4 boneMat(1.0f);
            if (wa.boneIndex < instance.boneMatrices.size()) {
                boneMat = instance.boneMatrices[wa.boneIndex];
            }

            // Weapon model matrix = character model * bone transform * attachment
            // offset * item/sheath orientation.
            glm::mat4 weaponMat =
                charModelMat * boneMat * glm::translate(glm::mat4(1.0f), wa.offset) *
                wa.localTransform;

            // Back-sheathed weapons: the swinging left arm passes through the
            // canted blade while running. Sample the whole arm - shoulder,
            // elbow, hand attachment points plus segment midpoints - against
            // the weapon model's AABB and ease the blade outward, away from
            // the spine, so the arm pushes it instead of clipping. A single
            // sphere at the elbow joint missed forearm/upper-arm contact.
            constexpr uint32_t kAttachmentBack = 12;
            if (wa.attachmentId == kAttachmentBack && weapIt->second.cachedModel) {
                constexpr uint32_t kAttachmentHandLeft = 2;
                constexpr uint32_t kAttachmentElbowLeft = 4;
                constexpr uint32_t kAttachmentShoulderLeft = 6;
                constexpr float kArmRadius = 0.16f;      // arm mesh thickness around the joint chain
                constexpr float kPushClearance = 0.03f;  // keep the blade slightly off the sleeve
                constexpr float kMaxPush = 0.35f;

                const glm::mat4 weaponInv = glm::inverse(weaponMat);
                const auto& wm = weapIt->second.cachedModel->data;

                glm::vec3 joints[3];
                int jointCount = 0;
                for (uint32_t attId : {kAttachmentShoulderLeft, kAttachmentElbowLeft, kAttachmentHandLeft}) {
                    glm::mat4 m;
                    if (getAttachmentTransform(pair.first, attId, m)) {
                        joints[jointCount++] = glm::vec3(m[3]);
                    }
                }

                float targetPush = 0.0f;
                auto testPoint = [&](const glm::vec3& worldPt) {
                    const glm::vec3 local = glm::vec3(weaponInv * glm::vec4(worldPt, 1.0f));
                    const glm::vec3 closest = glm::clamp(local, wm.boundMin, wm.boundMax);
                    const float pen = kArmRadius - glm::distance(local, closest);
                    if (pen > 0.0f) targetPush = std::max(targetPush, pen + kPushClearance);
                };
                for (int j = 0; j < jointCount; ++j) {
                    testPoint(joints[j]);
                    if (j + 1 < jointCount) testPoint((joints[j] + joints[j + 1]) * 0.5f);
                }
                targetPush = std::min(targetPush, kMaxPush);
                if (targetPush > 0.0f && wa.sheathPush < 0.01f) {
                    core::Logger::getInstance().debug("Sheath push engaged: pen=", targetPush,
                                                      " joints=", jointCount);
                }

                wa.sheathPush += (targetPush - wa.sheathPush) * std::min(1.0f, deltaTime * 14.0f);
                if (wa.sheathPush > 0.001f) {
                    // Horizontal direction from the spine out through the weapon.
                    glm::vec3 outward = glm::vec3(weaponMat[3]) - glm::vec3(charModelMat[3]);
                    outward.z = 0.0f;
                    const float len = glm::length(outward);
                    if (len > 0.001f) {
                        weaponMat = glm::translate(glm::mat4(1.0f), (outward / len) * wa.sheathPush) * weaponMat;
                    }
                }
            }
            weapIt->second.overrideModelMatrix = weaponMat;
            weapIt->second.hasOverrideModelMatrix = true;

            // Enchant visuals ride the weapon, offset to their attachment point on it.
            for (const auto& fx : wa.effects) {
                auto fxIt = instances.find(fx.effectInstanceId);
                if (fxIt == instances.end()) continue;
                fxIt->second.overrideModelMatrix =
                    weaponMat * glm::translate(glm::mat4(1.0f), fx.offset);
                fxIt->second.hasOverrideModelMatrix = true;
                // Keep the effect near the character so animation distance culling
                // (which works off instance position) treats it like its wielder.
                fxIt->second.position = instance.position;
            }
        }
    }
}
// --- Bone transform calculation ---

constexpr int32_t kKeyBoneSpineLow = 4;

void CharacterRenderer::calculateBoneMatrices(CharacterInstance& instance) {
    if (!instance.cachedModel) return;
    auto& model = instance.cachedModel->data;

    if (model.bones.empty()) {
        return;
    }

    size_t numBones = model.bones.size();
    instance.boneMatrices.resize(numBones);

    const auto& gsd = model.globalSequenceDurations;

    // One-time diagnostic: check bone ordering (parents must precede children)
    static bool checkedBoneOrder = false;
    if (!checkedBoneOrder) {
        checkedBoneOrder = true;
        for (size_t i = 0; i < numBones; i++) {
            const auto& bone = model.bones[i];
            if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) >= i) {
                LOG_WARNING("Bone ", i, " references parent ", bone.parentBone,
                            " which comes AFTER it - will use stale matrix!");
            }
        }
    }

    // How many more calls the extreme-translation probe below runs for.
    // Declared here rather than inside the loop so it can be advanced once per
    // call, after it, instead of on a bone the loop happens to reach.
    static int diagFrames = 0;

    for (size_t i = 0; i < numBones; i++) {
        const auto& bone = model.bones[i];

        // Local transform includes pivot bracket: T(pivot)*T*R*S*T(-pivot)
        // At rest this is identity, so no separate bind pose is needed
        glm::mat4 localTransform = getBoneTransform(bone, instance.animationTime, instance.globalSequenceTime,
                                                    instance.currentSequenceIndex, gsd);

        if (bone.keyBoneId == kKeyBoneSpineLow && instance.torsoYawOverrideRad != 0.0f) {
            glm::mat4 extraYaw = glm::translate(glm::mat4(1.0f), bone.pivot)
                                * glm::rotate(glm::mat4(1.0f), instance.torsoYawOverrideRad, glm::vec3(0.0f, 0.0f, 1.0f))
                                * glm::translate(glm::mat4(1.0f), -bone.pivot);
            localTransform = extraYaw * localTransform;
        }

        // Compose with parent
        if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) < numBones) {
            instance.boneMatrices[i] = instance.boneMatrices[bone.parentBone] * localTransform;
        } else {
            instance.boneMatrices[i] = localTransform;
        }

        // Diagnostic: detect bones with extreme translation. Gated so the abs()
        // probes and the post-loop counter bump only run for the first few frames.
        if (diagFrames < 3) {
            float tx = std::abs(instance.boneMatrices[i][3][0]);
            float ty = std::abs(instance.boneMatrices[i][3][1]);
            float tz = std::abs(instance.boneMatrices[i][3][2]);
            if (tx > 50.0f || ty > 50.0f || tz > 50.0f) {
                LOG_DEBUG("BONE DIAG: bone[", i, "] keyBone=", bone.keyBoneId,
                            " flags=0x", std::hex, bone.flags, std::dec,
                            " parent=", bone.parentBone,
                            " pivot=(", bone.pivot.x, ",", bone.pivot.y, ",", bone.pivot.z, ")",
                            " mat_t=(", instance.boneMatrices[i][3][0], ",",
                            instance.boneMatrices[i][3][1], ",", instance.boneMatrices[i][3][2], ")",
                            " local_t=(", localTransform[3][0], ",", localTransform[3][1], ",",
                            localTransform[3][2], ")",
                            " animTime=", instance.animationTime,
                            " gsTime=", instance.globalSequenceTime,
                            " seqIdx=", instance.currentSequenceIndex);
            }
        }
    }
    // Once per call, not once per last bone. The bump used to sit inside the
    // loop under `i == numBones - 1`, which this loop does reach - but a
    // counter that only advances on a condition inside the block it guards is
    // the shape that had a light-volume diagnostic logging every frame of every
    // session (a5080cec), and one `continue` added above would make this the
    // same fault. tools/bounded_log_check.py reports it.
    if (diagFrames < 3) diagFrames++;
}

glm::mat4 CharacterRenderer::getBoneTransform(const pipeline::M2Bone& bone, float animTime, float globalSeqTime,
                                               int sequenceIndex, const std::vector<uint32_t>& globalSeqDurations) {
    // Resolve global sequences: bones with globalSequence >= 0 use sequence 0
    // with time wrapped at the global sequence duration, independent of the
    // character's current animation.
    glm::vec3 translation = m2_track::sampleVec3(
        bone.translation, sequenceIndex, animTime, globalSeqTime,
        globalSeqDurations, glm::vec3(0.0f));
    glm::quat rotation = m2_track::sampleQuat(
        bone.rotation, sequenceIndex, animTime, globalSeqTime,
        globalSeqDurations);
    glm::vec3 scale = m2_track::sampleVec3(
        bone.scale, sequenceIndex, animTime, globalSeqTime,
        globalSeqDurations, glm::vec3(1.0f));

    // M2 bone transform: T(pivot) * T(trans) * R(rot) * S(scale) * T(-pivot).
    // Build directly instead of chaining glm::translate/rotate/scale (each of
    // those is a full mat4 multiply). The composed matrix has:
    //   linear part      = R * diag(scale)     (3 column scales of R)
    //   translation part = pivot + trans - RS * pivot
    glm::mat3 R = glm::mat3_cast(rotation);
    glm::vec3 c0 = R[0] * scale.x;
    glm::vec3 c1 = R[1] * scale.y;
    glm::vec3 c2 = R[2] * scale.z;
    glm::vec3 t  = (bone.pivot + translation)
                   - (c0 * bone.pivot.x + c1 * bone.pivot.y + c2 * bone.pivot.z);
    glm::mat4 transform;
    transform[0] = glm::vec4(c0, 0.0f);
    transform[1] = glm::vec4(c1, 0.0f);
    transform[2] = glm::vec4(c2, 0.0f);
    transform[3] = glm::vec4(t,  1.0f);
    return transform;
}

// --- Rendering ---

void CharacterRenderer::prepareRender(uint32_t frameIndex) {
    if (instances.empty() || !opaquePipeline_) return;
    if (frameIndex >= 2 || boneDescPool_ == VK_NULL_HANDLE || boneSetLayout_ == VK_NULL_HANDLE) return;

    // Pre-allocate bone SSBOs + descriptor sets on main thread (pool ops not thread-safe)
    for (auto& [id, instance] : instances) {
        int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), MAX_BONES);
        if (numBones <= 0) continue;

        if (!instance.boneBuffer[frameIndex]) {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = MAX_BONES * sizeof(glm::mat4);
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo allocInfo{};
            if (vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci,
                            &instance.boneBuffer[frameIndex], &instance.boneAlloc[frameIndex], &allocInfo) != VK_SUCCESS) {
                instance.boneBuffer[frameIndex] = VK_NULL_HANDLE;
                instance.boneAlloc[frameIndex] = VK_NULL_HANDLE;
                instance.boneMapped[frameIndex] = nullptr;
                continue;
            }
            instance.boneMapped[frameIndex] = allocInfo.pMappedData;

            // Initialize all bone slots to identity so out-of-range indices
            // produce correct (neutral) transforms instead of GPU garbage
            if (instance.boneMapped[frameIndex]) {
                auto* dst = static_cast<glm::mat4*>(instance.boneMapped[frameIndex]);
                for (int j = 0; j < MAX_BONES; j++) dst[j] = glm::mat4(1.0f);
            }

            VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            ai.descriptorPool = boneDescPool_;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts = &boneSetLayout_;
            VkResult dsRes = vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &instance.boneSet[frameIndex]);
            if (dsRes != VK_SUCCESS) {
                LOG_ERROR("CharacterRenderer::prepareRender: bone descriptor alloc failed (instance=",
                          id, ", frame=", frameIndex, ", vk=", static_cast<int>(dsRes), ")");
                if (instance.boneBuffer[frameIndex]) {
                    vmaDestroyBuffer(vkCtx_->getAllocator(),
                                     instance.boneBuffer[frameIndex], instance.boneAlloc[frameIndex]);
                    instance.boneBuffer[frameIndex] = VK_NULL_HANDLE;
                    instance.boneAlloc[frameIndex] = VK_NULL_HANDLE;
                    instance.boneMapped[frameIndex] = nullptr;
                }
                continue;
            }

            if (instance.boneSet[frameIndex]) {
                VkDescriptorBufferInfo bufInfo{};
                bufInfo.buffer = instance.boneBuffer[frameIndex];
                bufInfo.offset = 0;
                bufInfo.range = bci.size;
                VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.dstSet = instance.boneSet[frameIndex];
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                write.pBufferInfo = &bufInfo;
                vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &write, 0, nullptr);
            }
        }
    }
}

void CharacterRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (instances.empty() || !opaquePipeline_) {
        return;
    }
    const float renderRadius = static_cast<float>(envSizeOrDefault("WOWEE_CHAR_RENDER_RADIUS", 130));
    const float renderRadiusSq = renderRadius * renderRadius;
    // Default frustum-cull radius when model bounds aren't available.
    // 4.0 covers Tauren, mounted characters, and most creature models.
    constexpr float kDefaultCharacterCullRadius = 4.0f;
    const glm::vec3 camPos = camera.getPosition();
    const float frameTimeSeconds = std::chrono::duration<float>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // Extract frustum planes for per-instance visibility testing
    Frustum frustum;
    frustum.extractFromMatrix(camera.getViewProjectionMatrix());

    uint32_t frameIndex = vkCtx_->getCurrentFrame();
    uint32_t frameSlot = frameIndex % 2u;

    // Reset material ring buffer and descriptor pool once per frame slot.
    if (lastMaterialPoolResetFrame_ != frameIndex) {
        materialRingOffset_[frameSlot] = 0;
        if (materialDescPools_[frameSlot]) {
            vkResetDescriptorPool(vkCtx_->getDevice(), materialDescPools_[frameSlot], 0);
        }
        materialDescriptorCache_[frameSlot].clear();
        lastMaterialPoolResetFrame_ = frameIndex;
    }

    // Pre-compute aligned UBO stride for ring buffer sub-allocation
    const uint32_t uboStride = (sizeof(CharMaterialUBO) + materialUboAlignment_ - 1) & ~(materialUboAlignment_ - 1);
    const uint32_t ringCapacityBytes = uboStride * MATERIAL_RING_CAPACITY;
    auto getMaterialDescriptorSet = [&](VkTexture* diffuse, VkTexture* normal) -> VkDescriptorSet {
        if (!diffuse || !normal) return VK_NULL_HANDLE;
        const VkDescriptorImageInfo diffuseInfo = diffuse->descriptorInfo();
        const VkDescriptorImageInfo normalInfo = normal->descriptorInfo();
        const MaterialDescriptorKey key{diffuseInfo.imageView, normalInfo.imageView,
                                        diffuseInfo.sampler, normalInfo.sampler};
        auto& cache = materialDescriptorCache_[frameSlot];
        if (auto it = cache.find(key); it != cache.end()) return it->second;

        VkDescriptorSet set = VK_NULL_HANDLE;
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        ai.descriptorPool = materialDescPools_[frameSlot];
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &materialSetLayout_;
        if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set) != VK_SUCCESS)
            return VK_NULL_HANDLE;

        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = materialRingBuffer_[frameSlot];
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(CharMaterialUBO);
        VkWriteDescriptorSet writes[3] = {};
        writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set, 0, 0, 1,
                     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &diffuseInfo, nullptr, nullptr};
        writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set, 1, 0, 1,
                     VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, nullptr, &bufferInfo, nullptr};
        writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set, 2, 0, 1,
                     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &normalInfo, nullptr, nullptr};
        vkUpdateDescriptorSets(vkCtx_->getDevice(), 3, writes, 0, nullptr);
        cache.emplace(key, set);
        return set;
    };

    // Bind per-frame descriptor set (set 0) -- shared across all draws
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    // Start with opaque pipeline
    VkPipeline currentPipeline = opaquePipeline_;
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, currentPipeline);

    for (auto& pair : instances) {
        auto& instance = pair.second;

        // Skip invisible instances (e.g., player in first-person mode)
        if (!instance.visible) continue;

        // Character instance culling: test both distance and frustum visibility
        if (!instance.hasOverrideModelMatrix && !instance.isSceneModel) {
            glm::vec3 toInst = instance.position - camPos;
            float distSq = glm::dot(toInst, toInst);

            // Distance cull: skip if beyond render radius
            if (distSq > renderRadiusSq) continue;

            // Compute per-instance bounding radius from model data when available.
            float cullRadius = kDefaultCharacterCullRadius;
            auto mIt = models.find(instance.modelId);
            if (mIt != models.end()) {
                float modelR = mIt->second.data.boundRadius;
                if (modelR > 0.01f)
                    cullRadius = std::max(kDefaultCharacterCullRadius, modelR * std::max(0.001f, instance.scale));
            }

            // Frustum cull: skip if outside view frustum
            if (!frustum.intersectsSphere(instance.position, cullRadius)) continue;
        }

        if (!instance.cachedModel) continue;
        const auto& gpuModel = *instance.cachedModel;

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
            // GPU allocation is performed by prepareRender() before command
            // recording. Never allocate buffers/descriptors from the draw loop.
            if (!instance.boneBuffer[frameIndex] || !instance.boneSet[frameIndex]) continue;

            // Upload bone matrices
            if (instance.boneMapped[frameIndex]) {
                memcpy(instance.boneMapped[frameIndex], instance.boneMatrices.data(),
                       numBones * sizeof(glm::mat4));
            }

            // Bind bone descriptor set (set 2)
            if (instance.boneSet[frameIndex]) {
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 2, 1, &instance.boneSet[frameIndex], 0, nullptr);
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


            auto batchUsesTextureType = [](const M2ModelGPU& gm, const pipeline::M2Batch& b, uint32_t wantedType) {
                if (b.textureIndex == 0xFFFF || gm.data.textureLookup.empty()) return false;

                uint32_t comboCount = b.textureCount ? static_cast<uint32_t>(b.textureCount) : 1u;
                comboCount = std::min<uint32_t>(comboCount, 8u);
                for (uint32_t i = 0; i < comboCount; ++i) {
                    uint32_t lookupPos = static_cast<uint32_t>(b.textureIndex) + i;
                    if (lookupPos >= gm.data.textureLookup.size()) break;
                    uint16_t texSlot = gm.data.textureLookup[lookupPos];
                    if (texSlot < gm.data.textures.size() && gm.data.textures[texSlot].type == wantedType)
                        return true;
                }
                return false;
            };

            const bool previewMainModel = renderPassOverride_ != VK_NULL_HANDLE &&
                                          !instance.hasOverrideModelMatrix;

            // Draw batches in two passes: opaque (blendMode 0) first, then
            // alpha-key/blend after.  This ensures capes and body parts write
            // depth before hair overlay, preventing hair→cape z-fight.
            auto getBatchBlendMode = [&](const pipeline::M2Batch& b) -> uint16_t {
                if (b.materialIndex < gpuModel.data.materials.size())
                    return gpuModel.data.materials[b.materialIndex].blendMode;
                return 0;
            };

            // Use precomputed batch render order (cached on gpuModel at load time;
            // depends only on static batch metadata, so per-frame re-sorting was waste).
            const auto& sortedBatchIndices = gpuModel.sortedBatchIndices;

            for (int pass = 0; pass < 2; pass++) {
            for (size_t bi : sortedBatchIndices) {
                const auto& batch = gpuModel.data.batches[bi];
                uint16_t bm = getBatchBlendMode(batch);
                if (pass == 0 && bm != 0) continue;  // pass 0: opaque only
                if (pass == 1 && bm == 0) continue;   // pass 1: non-opaque only
                if (applyGeosetFilter) {
                    if (instance.activeGeosets.find(batch.submeshId) == instance.activeGeosets.end()) {
                        continue;
                    }
                } else {
                    // Even without a geoset filter, skip eye glow (group 17)
                    // and group 18 unless explicitly opted in. These geosets are
                    // only for DK/NE eye glow and should be off by default.
                    //
                    // Group 15 joins them, for the same reason and a visible
                    // one: it is the cloak, and its variants above 1501 are
                    // cloaks that exist. A model drawn with no filter drew one
                    // whether or not the character wore anything - and with no
                    // cloak texture to bind, a white sheet. An NPC that owns a
                    // cape says so by naming the geoset; one that says nothing
                    // has none.
                    uint16_t grp = batch.submeshId / 100;
                    if (grp == 17 || grp == 18 || grp == 15) continue;
                }
                // One line per head batch, for the first few instances: which
                // texture slot it resolved to and what type that slot is. The
                // player and an NPC use the same model, the same art and the
                // same pairing, and one of them draws a face - so the answer is
                // in what each batch actually got, which nothing has yet shown.
                // Only instances that are actually a character in the world: one
                // with per-instance texture overrides is an NPC, and the player
                // is the one that draws its head detail. The first run of this
                // spent all its lines on the character-select models before a
                // single NPC had spawned.
                const bool worldCharacter =
                    !instance.textureSlotOverrides.empty() || instance.drawSkinExtra;
                if (batch.submeshId == 0 && worldCharacter && headBatchCanaryCount_ < 24) {
                    ++headBatchCanaryCount_;
                    uint32_t chosenType = 0;
                    if (batch.textureIndex != 0xFFFF && !gpuModel.data.textureLookup.empty()) {
                        uint16_t sl = gpuModel.data.textureLookup[batch.textureIndex];
                        if (sl < gpuModel.data.textures.size())
                            chosenType = gpuModel.data.textures[sl].type;
                    }
                    VkTexture* rt = resolveBatchTexture(instance, gpuModel, batch);
                    // At debug: this canary was for heads resolving to the
                    // white texture, and they resolve to a texture. Twenty-four
                    // of them a session is noise in a log read for faults.
                    core::Logger::getInstance().debug(
                        "Head batch: instance=", pair.first, " model=", instance.modelId,
                        " geoset=", batch.submeshId, " firstSlotType=", chosenType,
                        " resolved=", (rt == whiteTexture_.get() ? "WHITE"
                                       : (rt == nullptr ? "null" : "texture")),
                        " ptr=", static_cast<const void*>(rt),
                        " overrides=", instance.textureSlotOverrides.size(),
                        " drawSkinExtra=", (instance.drawSkinExtra ? 1 : 0),
                        " verts=", gpuModel.data.vertices.size());
                }
                // Note on the Skin Extra batch, since it has been misread twice:
                // it is NOT a layer over the head. On an HD character model it
                // is its own section - 703 vertices and 634 triangles of the
                // human female's head, separate from the 403 and 877 of the
                // sections beside it - carrying the eyes, the mouth, the ears
                // and the eyelashes. Skipping it does not remove a detail pass,
                // it removes a face's features. Whatever is wrong with an NPC
                // face, it is not this batch existing.
                // M2 color-alpha animation gates prop submeshes per animation -
                // e.g. the peasant lumberjack carry model has two wood-bundle
                // submeshes and only one is alpha-1 in any given animation.
                // Opaque batches can't express alpha in the shader, so cull
                // batches whose track evaluates to ~0 for the current sequence.
                const float batchColorAlpha = glm::clamp(
                    evalBatchColorAlpha(gpuModel.data, batch,
                                        instance.currentSequenceIndex,
                                        instance.animationTime,
                                        instance.globalSequenceTime) *
                    evalBatchTextureWeight(gpuModel.data, batch,
                                           instance.currentSequenceIndex,
                                           instance.animationTime,
                                           instance.globalSequenceTime),
                    0.0f, 1.0f);
                if (batchColorAlpha <= 0.01f) {
                    continue;
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

                const uint16_t submeshGroup = static_cast<uint16_t>(batch.submeshId / 100);
                const bool hairTexture = batchUsesTextureType(gpuModel, batch, 6);
                const bool hairGeoset = (submeshGroup >= 1 && submeshGroup <= 3) ||
                                        (submeshGroup == 0 && batch.submeshId > 0 && batch.submeshId <= 99);
                // Scene models have no hair, and their submesh ids are all 0, which
                // would otherwise satisfy the hair-geoset guess for every batch.
                const bool hairMaterial = !instance.isSceneModel &&
                                          (hairTexture ||
                                           (hairGeoset && (blendMode != 0 || batch.textureCount > 1)));

                // Attached weapon models can include additive FX/card batches that
                // appear as detached flat quads for some swords. Keep core geometry
                // and drop FX-style passes for weapon attachments. Enchant visuals
                // are entirely such batches, so they must survive this cull.
                if (instance.hasOverrideModelMatrix && !instance.isEffectModel && blendMode >= 3) {
                    continue;
                }

                // For body/equipment parts with white/fallback texture, use skin (type 1) texture.
                if (texPtr == whiteTexture_.get() && !instance.isSceneModel) {
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
                    auto pit = texturePropsByPtr_.find(texPtr);
                    if (pit != texturePropsByPtr_.end()) {
                        alphaCutout = pit->second.hasAlpha;
                        colorKeyBlack = pit->second.colorKeyBlack;
                    }
                }
                // A scene means what its materials say. Stormwind's walls are DXT5 with
                // an unused alpha channel - every texel below the 0.5 cutoff - so
                // inferring a cutout from "the texture has alpha" discards the whole
                // building and leaves the sky showing through it. Only an alpha-key
                // material (blendMode 1) cuts out here.
                const bool blendNeedsCutout = instance.isSceneModel
                    ? (blendMode == 1)
                    : ((blendMode == 1) ||
                       (blendMode == 0 && alphaCutout) ||
                       (blendMode >= 2 && !alphaCutout) ||
                       hairMaterial);
                // Enchant glows emit their own light; scene lighting must not tint them.
                const bool unlit = ((materialFlags & 0x01) != 0) || (blendMode >= 3) ||
                                   instance.isEffectModel;

                // Hair textures are authored as alpha-cut cards. If they use the
                // translucent pipeline they form a soft shell around the head.
                // M2Blend: 0 Opaque, 1 AlphaKey, 2 Alpha, 3 NoAlphaAdd, 4 Add,
                // 5 Mod, 6 Mod2x, 7 BlendAdd. (6/Mod2x is kept on additive as it
                // has been; the doodad path in M2Renderer already sends
                // everything above 2 there.)
                const bool additiveBlend = (blendMode == 3 || blendMode == 4 ||
                                            blendMode == 6 || blendMode == 7);

                VkPipeline desiredPipeline;
                if (instance.isEffectModel) {
                    // Enchant visuals are glow cards drawn on black. Their materials
                    // declare Mod/alpha blending, which would composite that black
                    // background as an opaque quad - force additive so only the light adds.
                    desiredPipeline = additivePipeline_;
                } else if (additiveBlend) {
                    // Decided before the fade branch below, not after it. An
                    // additive card fades by adding less light - matData.opacity
                    // already scales what it contributes - so a partial alpha is
                    // not a reason to divert it to the translucent pipeline, and
                    // diverting it composites the card's black backing as black.
                    //
                    // The Darnassus wisps are the case: their glow alpha is an
                    // animated pulse, so the diversion tripped on almost every
                    // frame the pulse was not at full, and the cards showed as
                    // black roughly half the time.
                    desiredPipeline = additivePipeline_;
                } else if (instance.opacity * batchColorAlpha < 0.999f) {
                    // Whole-instance fade (ghost form, spawn fade-in): the opaque and
                    // alpha-test pipelines have blending disabled, so the shader's
                    // texColor.a * opacity output is discarded and only hair (via
                    // alpha-to-coverage) ever looked translucent. Route every batch
                    // through the blend pipeline; the per-batch alphaTest UBO flag
                    // still handles cutout materials in the shader.
                    //
                    // Except that a batch which must not write depth still must
                    // not, fading or otherwise. translucentPipeline_ writes
                    // depth so a fading character's own solid parts keep sorting
                    // against each other; alphaPipeline_ is the same blend with
                    // the depth write off. Sending a translucent card down the
                    // writing one puts an invisible occluder in front of
                    // whatever it covers, which is how the glue screens' cloud
                    // and haze cards were punching a rectangle out of the
                    // character standing behind them - the card's own materials
                    // ask for no depth write (0x10) and the scenes animate their
                    // alpha, so almost every frame took this branch.
                    const bool noDepthWrite = (blendMode >= 2) || ((materialFlags & 0x10) != 0);
                    desiredPipeline = noDepthWrite ? alphaPipeline_ : translucentPipeline_;
                } else if (hairMaterial) {
                    desiredPipeline = alphaTestPipeline_;
                } else {
                    // additiveBlend (3/4/6/7) is handled above; only the
                    // non-additive modes reach here. 4/Add is the one that used
                    // to fall through to the alpha pipeline, which composited a
                    // glow card's black backing as an actual black quad - every
                    // material on Wisp.m2 declares it.
                    switch (blendMode) {
                        case 0: desiredPipeline = opaquePipeline_; break;
                        case 1: desiredPipeline = alphaTestPipeline_; break;
                        case 2: desiredPipeline = alphaPipeline_; break;
                        default: desiredPipeline = alphaPipeline_; break;
                    }
                }
                if (desiredPipeline != currentPipeline) {
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
                    currentPipeline = desiredPipeline;
                }

                float emissiveBoost = 1.0f;
                glm::vec3 emissiveTint(1.0f, 1.0f, 1.0f);
                const bool koboldCandleFlame = colorKeyBlack && gpuModel.isKoboldFlame;
                if (unlit && koboldCandleFlame) {
                    float phase = static_cast<float>(batch.submeshId) * 0.31f;
                    float f1 = std::sin(frameTimeSeconds * 7.9f + phase);
                    float f2 = std::sin(frameTimeSeconds * 12.7f + phase * 1.73f);
                    float f3 = std::sin(frameTimeSeconds * 4.3f + phase * 2.11f);
                    float flicker = 0.90f + 0.10f * f1 + 0.06f * f2 + 0.04f * f3;
                    flicker = std::clamp(flicker, 0.72f, 1.12f);
                    emissiveBoost = (blendMode >= 3) ? (2.4f * flicker) : (1.5f * flicker);
                    emissiveTint = glm::vec3(1.28f, 1.04f, 0.82f);
                }

                // Resolve normal/height map for this texture
                VkTexture* normalMap = flatNormalTexture_.get();
                float batchHeightVariance = 0.0f;
                if (texPtr && texPtr != whiteTexture_.get()) {
                    auto nmIt = normalMapByTexPtr_.find(texPtr);
                    if (nmIt != normalMapByTexPtr_.end()) {
                        normalMap = nmIt->second.normalMap;
                        batchHeightVariance = nmIt->second.heightMapVariance;
                    }
                }

                // POM quality → sample count
                const int pomSamples = pomSamplesFor(pomQuality_);
                const bool useAdvancedMaterials = !previewMainModel;
                const bool usePreviewSimpleShader = previewMainModel;

                // Create per-batch material UBO
                CharMaterialUBO matData{};
                matData.opacity = instance.opacity * batchColorAlpha;
                matData.alphaTest = blendNeedsCutout ? 1 : 0;
                matData.colorKeyBlack = colorKeyBlack ? 1 : 0;
                matData.unlit = unlit ? 1 : 0;
                matData.emissiveBoost = emissiveBoost;
                matData.emissiveTintR = emissiveTint.r;
                matData.emissiveTintG = emissiveTint.g;
                matData.emissiveTintB = emissiveTint.b;
                matData.specularIntensity = 0.5f;
                matData.enableNormalMap = (useAdvancedMaterials && normalMappingEnabled_) ? 1 : 0;
                // Character textures are layered/mirrored atlases, not coherent
                // height fields. Parallax offsets cross face seams and reveal
                // underlying armor through tabards, so keep POM out of the
                // character path even when it is enabled for the world.
                matData.enablePOM = 0;
                matData.pomScale = 0.06f;
                matData.pomMaxSamples = pomSamples;
                matData.heightMapVariance = useAdvancedMaterials ? batchHeightVariance : 0.0f;
                matData.normalMapStrength = normalMapStrength_;
                matData.hairMaterial = hairMaterial ? 1 : 0;

                // The base humanoid mesh samples a mirrored character atlas,
                // with the face sitting directly beside a UV seam. Parallax
                // displacement moves each mirrored half in opposite screen
                // directions, producing crossed eyes and a center-squashed
                // mouth at oblique angles. Keep normal lighting, but never
                // displace UVs on group 0 body/head surfaces.
                if (batchGroup == 0) {
                    normalMap = flatNormalTexture_.get();
                    matData.specularIntensity = 0.20f;
                    matData.enableNormalMap = 0;
                    matData.enablePOM = 0;
                    matData.heightMapVariance = 0.0f;
                }

                // Tabards are a thin cloth layer over the torso. Reusing the
                // body composite's generated height/normal response makes the
                // armor underneath appear embossed and reflective through the
                // cloth as the camera moves. Keep the diffuse tabard artwork,
                // but give group 12 a flat, low-specular cloth material.
                if (batchGroup == 12) {
                    normalMap = flatNormalTexture_.get();
                    matData.specularIntensity = 0.08f;
                    matData.enableNormalMap = 0;
                    matData.enablePOM = 0;
                    matData.heightMapVariance = 0.0f;
                }
                if (usePreviewSimpleShader) {
                    matData.enableNormalMap = 0;
                    matData.enablePOM = kPreviewSimpleTextureMode;
                    matData.heightMapVariance = 0.0f;
                }

                // WOWEE_SCENE_DIAG=1 dumps what each backdrop batch is actually told to
                // draw - texture, blend mode, shader path - once per scene model.
                static int sceneDiagLines = 0;
                if (instance.isSceneModel && sceneDiagEnabled() && sceneDiagLines++ < 40) {
                    std::string texName = "<white>";
                    if (batch.textureIndex < gpuModel.data.textureLookup.size()) {
                        uint16_t lk = gpuModel.data.textureLookup[batch.textureIndex];
                        if (lk < gpuModel.data.textures.size())
                            texName = gpuModel.data.textures[lk].filename;
                    }
                    // Warning level: the diagnostic is opt-in already, and the file log
                    // filters info out by default.
                    core::Logger::getInstance().warning(
                        "SCENE DIAG batch submesh=", batch.submeshId,
                        " blend=", blendMode, " matFlags=0x", std::hex, materialFlags, std::dec,
                        " alphaTest=", matData.alphaTest,
                        " unlit=", matData.unlit,
                        " simplePath=", (matData.enablePOM == kPreviewSimpleTextureMode ? 1 : 0),
                        " whiteFallback=", (texPtr == whiteTexture_.get() ? 1 : 0),
                        " tex=", texName);
                }

                // Sub-allocate material UBO from ring buffer
                uint32_t matOffset = materialRingOffset_[frameSlot];
                if (matOffset + uboStride > ringCapacityBytes) continue; // ring exhausted
                memcpy(static_cast<char*>(materialRingMapped_[frameSlot]) + matOffset, &matData, sizeof(CharMaterialUBO));
                materialRingOffset_[frameSlot] = matOffset + uboStride;

                VkTexture* bindTex = (texPtr && texPtr->isValid()) ? texPtr : whiteTexture_.get();
                VkDescriptorSet materialSet = getMaterialDescriptorSet(bindTex, normalMap);
                if (!materialSet) continue;

                // Bind material descriptor set (set 1)
                const uint32_t dynamicOffset = matOffset;
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 1, 1, &materialSet, 1, &dynamicOffset);

                // Per-batch depth bias from materialLayer to separate coplanar
                // armor pieces (chest/legs/gloves) that share identical depth.
                vkCmdSetDepthBias(cmd, static_cast<float>(batch.materialLayer) * 0.5f, 0.0f, 0.0f);

                vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
            }
            } // end pass loop
        } else {
            // Draw entire model with first texture
            VkTexture* texPtr = !gpuModel.textureIds.empty() ? gpuModel.textureIds[0] : whiteTexture_.get();
            if (!texPtr || !texPtr->isValid()) texPtr = whiteTexture_.get();

            // POM quality → sample count
            const int pomSamples2 = pomSamplesFor(pomQuality_);

            // Whole-model fallback inherits whatever pipeline was bound last;
            // pick it explicitly so instance fades blend here too.
            VkPipeline fallbackPipeline = (instance.opacity < 0.999f)
                ? translucentPipeline_ : opaquePipeline_;
            if (fallbackPipeline != currentPipeline) {
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, fallbackPipeline);
                currentPipeline = fallbackPipeline;
            }

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
            const bool previewMainModel = renderPassOverride_ != VK_NULL_HANDLE &&
                                          !instance.hasOverrideModelMatrix;
            const bool useAdvancedMaterials = !previewMainModel;
            const bool usePreviewSimpleShader = previewMainModel;
            matData.enableNormalMap = (useAdvancedMaterials && normalMappingEnabled_) ? 1 : 0;
            matData.enablePOM = (useAdvancedMaterials && pomEnabled_) ? 1 : 0;
            matData.pomScale = 0.06f;
            matData.pomMaxSamples = pomSamples2;
            matData.heightMapVariance = 0.0f;
            matData.normalMapStrength = normalMapStrength_;
            matData.hairMaterial = 0;
            if (usePreviewSimpleShader) {
                matData.enableNormalMap = 0;
                matData.enablePOM = kPreviewSimpleTextureMode;
            }

            // Sub-allocate material UBO from ring buffer
            uint32_t matOffset2 = materialRingOffset_[frameSlot];
            if (matOffset2 + uboStride > ringCapacityBytes) continue; // ring exhausted
            memcpy(static_cast<char*>(materialRingMapped_[frameSlot]) + matOffset2, &matData, sizeof(CharMaterialUBO));
            materialRingOffset_[frameSlot] = matOffset2 + uboStride;

            VkDescriptorSet materialSet = getMaterialDescriptorSet(texPtr, flatNormalTexture_.get());
            if (!materialSet) continue;

            const uint32_t dynamicOffset = matOffset2;
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    pipelineLayout_, 1, 1, &materialSet, 1, &dynamicOffset);

            vkCmdDrawIndexed(cmd, gpuModel.indexCount, 1, 0, 0, 0);
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

    // The same set the other three shadow passes bind - a sampler and a small
    // uniform buffer - with this pass's own params behind binding 1.
    if (!createShadowParamsSet(device, vkCtx_->getAllocator(), sizeof(ShadowCharParams),
                               whiteTexture_->getImageView(), whiteTexture_->getSampler(),
                               "CharacterRenderer", shadowParams_)) {
        return false;
    }

    // Alpha testing on, once, because it never varies here.
    //
    // The buffer is zero-filled at creation and was then never written, so the
    // flag read 0 and every caster stamped its bounding rectangle into the
    // shadow map - hair, capes and cloaks cast solid slabs instead of their own
    // outline. With it on, an opaque batch is bound to the white fallback whose
    // alpha is 1 and passes the cutoff untouched, while an alpha-keyed batch is
    // bound to its own texture below and cuts properly.
    {
        VmaAllocationInfo paramsInfo{};
        vmaGetAllocationInfo(vkCtx_->getAllocator(), shadowParams_.alloc, &paramsInfo);
        if (paramsInfo.pMappedData) {
            ShadowCharParams p{};
            p.alphaTest = 1;
            p.colorKeyBlack = 0;
            std::memcpy(paramsInfo.pMappedData, &p, sizeof(p));
        }
    }

    // One texture-set pool per frame in flight, reset at the top of each
    // frame's shadow pass.
    {
        VkDescriptorPoolSize texPoolSizes[2]{};
        texPoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        texPoolSizes[0].descriptorCount = 256;
        texPoolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        texPoolSizes[1].descriptorCount = 256;
        VkDescriptorPoolCreateInfo texPoolCI{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        texPoolCI.maxSets = 256;
        texPoolCI.poolSizeCount = 2;
        texPoolCI.pPoolSizes = texPoolSizes;
        for (uint32_t f = 0; f < kShadowTexPoolFrames; ++f) {
            if (vkCreateDescriptorPool(device, &texPoolCI, nullptr, &shadowTexPool_[f]) != VK_SUCCESS) {
                LOG_ERROR("CharacterRenderer: failed to create shadow texture pool ", f);
                return false;
            }
        }
    }

    // Pipeline layout: set 0 = shadowParams_.layout, set 1 = boneSetLayout_
    //
    // There used to be a third, perFrameLayout_, sitting at set 0 as a dummy
    // that nothing ever bound, which pushed the params to set 1 and the bones to
    // set 2. That made this the odd one out of the four passes sharing the
    // shadow render pass: terrain, WMO and M2 all bind their params at set 0
    // under a layout of their own. Binding set 1 while set 0 still carried
    // another pass's incompatible layout left the params disturbed rather than
    // bound, and the fragment shader read its alpha-test flags from nothing -
    // thousands of times a session, which GPU-assisted validation reports as
    // indexing a descriptor array of length zero. A set that is never bound has
    // no business being in the layout.
    // Push constant: 128 bytes (lightSpaceMatrix + model), VERTEX stage
    VkDescriptorSetLayout setLayouts[] = {shadowParams_.layout, boneSetLayout_};
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pc.offset = 0;
    pc.size = 128;
    VkPipelineLayoutCreateInfo plCI{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plCI.setLayoutCount = 2;
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
    // loc 2: uvec4 aBoneIndices (R8G8B8A8_UINT,    offset 16)
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

    shadowPipeline_ = buildShadowPipeline(
        device, vkCtx_->getPipelineCache(),
        vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
        fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT),
        vertBind, vertAttrs, shadowPipelineLayout_, shadowRenderPass);

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
    if (!shadowPipeline_ || !shadowParams_.set) return;
    if (instances.empty() || models.empty()) return;
    if (boneDescPool_ == VK_NULL_HANDLE || boneSetLayout_ == VK_NULL_HANDLE) return;

    uint32_t frameIndex = vkCtx_->getCurrentFrame();
    if (frameIndex >= 2) return;
    VkDevice device = vkCtx_->getDevice();

    // This frame slot's fence was waited on in beginFrame, so last time's sets
    // are finished with and the pool can be handed back whole.
    if (frameIndex < kShadowTexPoolFrames && shadowTexPool_[frameIndex]) {
        vkResetDescriptorPool(device, shadowTexPool_[frameIndex], 0);
    }
    shadowTexSetCache_.clear();

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);

    const float shadowRadiusSq = shadowRadius * shadowRadius;
    // Which set is bound at 0 right now, so a run of opaque batches does not
    // rebind the same fallback for each one.
    VkDescriptorSet currentTexSet = VK_NULL_HANDLE;
    for (auto& pair : instances) {
        auto& inst = pair.second;
        if (!inst.visible) continue;

        // Distance cull against shadow frustum
        glm::vec3 diff = inst.position - shadowCenter;
        if (glm::dot(diff, diff) > shadowRadiusSq) continue;

        if (!inst.cachedModel) continue;
        const M2ModelGPU& gpuModel = *inst.cachedModel;
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
                if (vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci,
                    &inst.boneBuffer[frameIndex], &inst.boneAlloc[frameIndex], &ai) != VK_SUCCESS) {
                    inst.boneBuffer[frameIndex] = VK_NULL_HANDLE;
                    inst.boneAlloc[frameIndex] = VK_NULL_HANDLE;
                    inst.boneMapped[frameIndex] = nullptr;
                    continue;
                }
                inst.boneMapped[frameIndex] = ai.pMappedData;

                // Initialize all bone slots to identity so out-of-range indices
                // produce correct (neutral) transforms instead of GPU garbage
                if (inst.boneMapped[frameIndex]) {
                    auto* dst = static_cast<glm::mat4*>(inst.boneMapped[frameIndex]);
                    for (int j = 0; j < MAX_BONES; j++) dst[j] = glm::mat4(1.0f);
                }

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

        // Params at set 1 and bones at set 2, together, per instance.
        //
        // Binding set 1 once before the loop was not enough. This pass runs
        // after the M2 shadow pass, which binds its own set at set 0 under a
        // different pipeline layout, and a descriptor set bound while a lower
        // set carries an incompatible layout is disturbed rather than kept. The
        // fragment shader then read set 1 binding 1 - its alpha-test flags -
        // from a set the GPU no longer considered bound, which GPU-assisted
        // validation reports as indexing a descriptor array of length zero,
        // thousands of times a session.
        VkDescriptorSet sets[2] = {shadowParams_.set, inst.boneSet[frameIndex]};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
            0, 2, sets, 0, nullptr);
        currentTexSet = shadowParams_.set;

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
            if (!applyGeosetFilter) {
                uint16_t grp = batch.submeshId / 100;
                if (grp == 17 || grp == 18) continue;
            }

            // An alpha-keyed batch casts the shape of its texture; everything
            // else keeps the white fallback and casts solid. Only the set at
            // binding 0 changes, so the bones stay bound from above.
            VkDescriptorSet texSet = shadowParams_.set;
            if (blendMode == 1) {
                VkTexture* tex = resolveBatchTexture(inst, gpuModel, batch);
                if (tex && tex != whiteTexture_.get()) {
                    texSet = shadowTexDescSet(tex, frameIndex);
                }
            }
            if (texSet != currentTexSet) {
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
                                        0, 1, &texSet, 0, nullptr);
                currentTexSet = texSet;
            }

            vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
        }
    }
}

VkDescriptorSet CharacterRenderer::shadowTexDescSet(VkTexture* tex, uint32_t frameIndex) {
    if (!tex || frameIndex >= kShadowTexPoolFrames) return shadowParams_.set;
    VkDescriptorPool pool = shadowTexPool_[frameIndex];
    if (!pool) return shadowParams_.set;

    VkImageView view = tex->getImageView();
    if (auto it = shadowTexSetCache_.find(view); it != shadowTexSetCache_.end()) return it->second;

    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = pool;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &shadowParams_.layout;
    VkDescriptorSet set = VK_NULL_HANDLE;
    if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set) != VK_SUCCESS) {
        // Out of sets for this frame: the white fallback casts a full shadow,
        // which is the old behaviour rather than a hole.
        return shadowParams_.set;
    }

    VkDescriptorImageInfo imgInfo{};
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgInfo.imageView = view;
    imgInfo.sampler = tex->getSampler();
    VkDescriptorBufferInfo bufInfo{};
    bufInfo.buffer = shadowParams_.ubo;
    bufInfo.offset = 0;
    bufInfo.range = VK_WHOLE_SIZE;
    VkWriteDescriptorSet writes[2]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = set;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &imgInfo;
    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = set;
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[1].pBufferInfo = &bufInfo;
    vkUpdateDescriptorSets(vkCtx_->getDevice(), 2, writes, 0, nullptr);

    shadowTexSetCache_[view] = set;
    return set;
}

VkTexture* CharacterRenderer::resolveBatchTexture(const CharacterInstance& inst,
                                                 const M2ModelGPU& gm,
                                                 const pipeline::M2Batch& b) const {
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
        it->second.moveStart = position;
        it->second.moveEnd = position;
        it->second.moveElapsed = 0.0f;
        it->second.moveDuration = 0.0f;
        it->second.isMoving = false;
    }
}

void CharacterRenderer::setInstanceRotation(uint32_t instanceId, const glm::vec3& rotation) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.rotation = rotation;
    }
}

void CharacterRenderer::setInstanceTorsoYaw(uint32_t instanceId, float deltaYawRad) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.torsoYawOverrideRad = deltaYawRad;
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

    float pdx = destination.x - inst.position.x;
    float pdy = destination.y - inst.position.y;
    float planarDistSq = pdx * pdx + pdy * pdy;
    bool synthesizedDuration = false;
    if (durationSeconds <= 0.0f) {
        if (planarDistSq < 1e-4f) {
            // Stop at current location.
            inst.position = destination;
            inst.isMoving = false;
            if (inst.currentAnimationId == anim::WALK || inst.currentAnimationId == anim::RUN) {
                playAnimation(instanceId, anim::STAND, true);
            }
            return;
        }
        // Some cores send movement-only deltas without spline duration.
        // Synthesize a tiny duration so movement anim/rotation still updates.
        durationSeconds = std::clamp(std::sqrt(planarDistSq) / 7.0f, 0.05f, 0.20f);
        synthesizedDuration = true;
    }

    inst.moveStart = inst.position;
    inst.moveEnd = destination;
    inst.moveDuration = durationSeconds;
    inst.moveElapsed = 0.0f;
    inst.isMoving = true;

    // Face toward destination (yaw around Z axis since Z is up)
    glm::vec3 dir = destination - inst.position;
    if (dir.x * dir.x + dir.y * dir.y > 1e-6f) {
        float angle = std::atan2(dir.y, dir.x);
        inst.rotation.z = angle;
    }

    // Play movement animation while moving.
    // Prefer run only when speed is clearly above normal walk pace.
    float moveSpeed = std::sqrt(planarDistSq) / std::max(durationSeconds, 0.001f);
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

const pipeline::M2Model* CharacterRenderer::getInstanceModelData(uint32_t instanceId) const {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) return nullptr;
    return getModelData(instIt->second.modelId);
}

void CharacterRenderer::startFadeIn(uint32_t instanceId, float durationSeconds) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;
    it->second.opacity = 0.0f;
    it->second.fadeInTime = 0.0f;
    it->second.fadeInDuration = durationSeconds;
}

void CharacterRenderer::setInstanceOpacity(uint32_t instanceId, float opacity) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        const float clampedOpacity = std::clamp(opacity, 0.0f, 1.0f);
        it->second.opacity = clampedOpacity;
        // Cancel any fade-in in progress to avoid overwriting the new opacity
        it->second.fadeInDuration = 0.0f;

        // Equipment is rendered as independent character instances. Keep the
        // whole visual together instead of leaving opaque weapons floating on
        // a translucent stealthed creature.
        for (const auto& attachment : it->second.weaponAttachments) {
            auto weaponIt = instances.find(attachment.weaponInstanceId);
            if (weaponIt != instances.end()) {
                weaponIt->second.opacity = clampedOpacity;
                weaponIt->second.fadeInDuration = 0.0f;
            }
            for (const auto& effect : attachment.effects) {
                auto effectIt = instances.find(effect.effectInstanceId);
                if (effectIt != instances.end()) {
                    effectIt->second.opacity = clampedOpacity;
                    effectIt->second.fadeInDuration = 0.0f;
                }
            }
        }
    }
}

void CharacterRenderer::setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.activeGeosets = geosets;
    }
}

void CharacterRenderer::setDrawSkinExtra(uint32_t instanceId, bool enabled) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.drawSkinExtra = enabled;
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

    // Remove child attachments first (helmets/weapons/enchant effects),
    // otherwise they leak as orphan render instances when the parent creature
    // despawns. Their models get a fresh id per attach, so unload those too
    // once the instances are gone.
    auto attachments = it->second.weaponAttachments;
    for (const auto& wa : attachments) {
        for (const auto& fx : wa.effects) removeInstance(fx.effectInstanceId);
        removeInstance(wa.weaponInstanceId);
    }
    for (const auto& wa : attachments) {
        unloadModelIfUnused(wa.weaponModelId);
        for (const auto& fx : wa.effects) unloadModelIfUnused(fx.effectModelId);
    }

    // Defer bone buffer destruction - in-flight command buffers may still
    // reference these descriptor sets.
    destroyInstanceBones(it->second, /*defer=*/true);

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

const std::vector<uint32_t>* CharacterRenderer::getFootstepEventTimes(uint32_t instanceId) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return nullptr;
    }

    const CharacterInstance& instance = it->second;
    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        return nullptr;
    }

    const auto& eventTimes = modelIt->second.data.footstepEventTimes;
    if (instance.currentSequenceIndex < 0 ||
        instance.currentSequenceIndex >= static_cast<int>(eventTimes.size()) ||
        eventTimes[instance.currentSequenceIndex].empty()) {
        return nullptr;
    }
    return &eventTimes[instance.currentSequenceIndex];
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

bool CharacterRenderer::findAttachmentBone(uint32_t modelId, uint32_t attachmentId,
                                          uint16_t& outBoneIndex, glm::vec3& outOffset) const {
    auto modelIt = models.find(modelId);
    if (modelIt == models.end()) return false;
    const auto& model = modelIt->second.data;

    outBoneIndex = 0;
    outOffset = glm::vec3(0.0f);
    bool found = false;

    // Try attachment lookup first
    if (attachmentId < model.attachmentLookup.size()) {
        uint16_t attIdx = model.attachmentLookup[attachmentId];
        if (attIdx < model.attachments.size()) {
            outBoneIndex = model.attachments[attIdx].bone;
            outOffset = model.attachments[attIdx].position;
            found = true;
        }
    }

    // Fallback: scan attachments by id
    if (!found) {
        for (const auto& att : model.attachments) {
            if (att.id == attachmentId) {
                outBoneIndex = att.bone;
                outOffset = att.position;
                found = true;
                break;
            }
        }
    }

    // Fallback: key-bone lookup for weapon hand attachment IDs (ID 1 = right hand, ID 2 = left hand)
    if (!found && (attachmentId == 1 || attachmentId == 2)) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < model.bones.size(); i++) {
            if (model.bones[i].keyBoneId == targetKeyBone) {
                outBoneIndex = static_cast<uint16_t>(i);
                outOffset = glm::vec3(0.0f);
                found = true;
                break;
            }
        }
    }

    // Fallback for head attachment (ID 11): use bone 0 if attachment not defined
    if (!found && attachmentId == 11 && model.bones.size() > 0) {
        outBoneIndex = 0;
        found = true;
    }

    // Validate bone index
    if (found && outBoneIndex >= model.bones.size()) {
        found = false;
    }

    return found;
}

bool CharacterRenderer::attachWeapon(uint32_t charInstanceId, uint32_t attachmentId,
                                      const pipeline::M2Model& weaponModel, uint32_t weaponModelId,
                                      const std::string& texturePath,
                                      const glm::mat4& localTransform) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) {
        core::Logger::getInstance().warning("attachWeapon: character instance ", charInstanceId, " not found");
        return false;
    }
    auto& charInstance = charIt->second;
    auto charModelIt = models.find(charInstance.modelId);
    if (charModelIt == models.end()) return false;

    // Find bone index for this attachment point
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    if (!findAttachmentBone(charInstance.modelId, attachmentId, boneIndex, offset)) {
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
            // Item models can keep an authored hilt texture in slot 0 and expose
            // the DBC-selected skin through one or more replaceable slots:
            // 2 = object skin, 3 = weapon blade, 4 = weapon handle.  Applying the
            // skin to slot 0 alone leaves multi-material weapons with an untextured
            // blade (notably Melris Malagan's sword).
            bool appliedReplaceableSlot = false;
            auto modelIt = models.find(weaponModelId);
            if (modelIt != models.end()) {
                const auto& textures = modelIt->second.data.textures;
                for (uint32_t slot = 0; slot < textures.size(); ++slot) {
                    const uint32_t type = textures[slot].type;
                    if (type == 2 || type == 3 || type == 4) {
                        setModelTexture(weaponModelId, slot, texPtr);
                        appliedReplaceableSlot = true;
                    }
                }
            }
            // Older/simple item models commonly expose only one unnamed slot.
            if (!appliedReplaceableSlot) {
                setModelTexture(weaponModelId, 0, texPtr);
            }
        }
    }

    // Create weapon instance
    uint32_t weaponInstanceId = createInstance(weaponModelId, glm::vec3(0.0f));
    if (weaponInstanceId == 0) return false;

    // Mark weapon instance as override-positioned
    auto weapIt = instances.find(weaponInstanceId);
    if (weapIt != instances.end()) {
        weapIt->second.hasOverrideModelMatrix = true;
        weapIt->second.opacity = charInstance.opacity;
    }

    // Store attachment on parent character instance
    WeaponAttachment wa;
    wa.weaponModelId = weaponModelId;
    wa.weaponInstanceId = weaponInstanceId;
    wa.attachmentId = attachmentId;
    wa.boneIndex = boneIndex;
    wa.offset = offset;
    wa.localTransform = localTransform;
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

    glm::vec3 boundsMin = mIt->second.visualBoundMin;
    glm::vec3 boundsMax = mIt->second.visualBoundMax;
    float radius = mIt->second.visualBoundRadius;
    if (radius <= 0.001f) {
        boundsMin = model.boundMin;
        boundsMax = model.boundMax;
        radius = model.boundRadius;
        if (radius <= 0.001f) {
            radius = glm::length(boundsMax - boundsMin) * 0.5f;
        }
    }

    float scale = std::max(0.001f, inst.scale);
    const glm::vec3 localCenter = (boundsMin + boundsMax) * 0.5f;
    glm::mat4 rotation(1.0f);
    rotation = glm::rotate(rotation, inst.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
    rotation = glm::rotate(rotation, inst.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
    rotation = glm::rotate(rotation, inst.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    outCenter = inst.position + glm::vec3(rotation * glm::vec4(localCenter * scale, 0.0f));
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

bool CharacterRenderer::getInstancePosition(uint32_t instanceId, glm::vec3& outPos) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return false;
    outPos = it->second.position;
    return true;
}

void CharacterRenderer::detachWeapon(uint32_t charInstanceId, uint32_t attachmentId) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) return;
    auto& attachments = charIt->second.weaponAttachments;

    for (auto it = attachments.begin(); it != attachments.end(); ++it) {
        if (it->attachmentId == attachmentId) {
            // Collect model ids before erasing the attachment, then unload the
            // ones no instance still uses (each attach allocates a fresh id).
            std::vector<uint32_t> modelIds;
            modelIds.push_back(it->weaponModelId);
            for (const auto& fx : it->effects) {
                removeInstance(fx.effectInstanceId);
                modelIds.push_back(fx.effectModelId);
            }
            removeInstance(it->weaponInstanceId);
            attachments.erase(it);
            for (uint32_t mid : modelIds) unloadModelIfUnused(mid);
            core::Logger::getInstance().info("Detached weapon from instance ", charInstanceId,
                " attachment ", attachmentId);
            return;
        }
    }
}

void CharacterRenderer::unloadModelIfUnused(uint32_t modelId) {
    auto modelIt = models.find(modelId);
    if (modelIt == models.end()) return;
    for (const auto& p : instances) {
        if (p.second.modelId == modelId) return;
    }
    destroyModelGPU(modelIt->second, /*defer=*/true);
    models.erase(modelIt);
}

bool CharacterRenderer::attachWeaponEffect(uint32_t charInstanceId, uint32_t attachmentId,
                                           uint32_t visualSlot,
                                           const pipeline::M2Model& effectModel,
                                           uint32_t effectModelId) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) return false;

    auto& attachments = charIt->second.weaponAttachments;
    auto waIt = std::find_if(attachments.begin(), attachments.end(),
                             [attachmentId](const WeaponAttachment& wa) {
                                 return wa.attachmentId == attachmentId;
                             });
    if (waIt == attachments.end()) return false;  // no weapon to hang the effect on

    if (models.find(effectModelId) == models.end()) {
        if (!loadModel(effectModel, effectModelId)) {
            core::Logger::getInstance().warning("attachWeaponEffect: failed to load effect model ",
                                                effectModelId);
            return false;
        }
    }

    // The ItemVisuals slot names the attachment point on the weapon model itself.
    // Weapons carry few attachments, so fall back to the weapon's origin.
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    if (!findAttachmentBone(waIt->weaponModelId, visualSlot, boneIndex, offset)) {
        offset = glm::vec3(0.0f);
    }

    uint32_t effectInstanceId = createInstance(effectModelId, glm::vec3(0.0f));
    if (effectInstanceId == 0) return false;

    auto fxIt = instances.find(effectInstanceId);
    if (fxIt == instances.end()) return false;
    fxIt->second.hasOverrideModelMatrix = true;
    fxIt->second.isEffectModel = true;
    fxIt->second.animationLoop = true;
    fxIt->second.opacity = charIt->second.opacity;

    WeaponEffectAttachment fx;
    fx.effectModelId = effectModelId;
    fx.effectInstanceId = effectInstanceId;
    fx.offset = offset;
    waIt->effects.push_back(fx);

    core::Logger::getInstance().debug("Attached enchant visual model ", effectModelId,
        " to weapon at attachment ", attachmentId, " (visual slot ", visualSlot, ")");
    return true;
}

void CharacterRenderer::setInstanceSceneModel(uint32_t instanceId, bool isScene) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) it->second.isSceneModel = isScene;
}

void CharacterRenderer::detachWeaponEffects(uint32_t charInstanceId, uint32_t attachmentId) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) return;

    for (auto& wa : charIt->second.weaponAttachments) {
        if (wa.attachmentId != attachmentId) continue;
        for (const auto& fx : wa.effects) removeInstance(fx.effectInstanceId);
        wa.effects.clear();
        return;
    }
}

bool CharacterRenderer::getAttachmentTransform(uint32_t instanceId, uint32_t attachmentId, glm::mat4& outTransform) {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) return false;
    const auto& instance = instIt->second;

    // Find attachment point using shared lookup logic
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    if (!findAttachmentBone(instance.modelId, attachmentId, boneIndex, offset)) {
        return false;
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
    destroy(device, opaquePipeline_);
    destroy(device, alphaTestPipeline_);
    destroy(device, alphaPipeline_);
    destroy(device, additivePipeline_);
    destroy(device, translucentPipeline_);

    // --- Load shaders ---
    rendering::VkShaderModule charVert, charFrag;
    if (!charVert.loadFromFile(device, "assets/shaders/character.vert.spv") ||
        !charFrag.loadFromFile(device, "assets/shaders/character.frag.spv")) {
        LOG_ERROR("CharacterRenderer::recreatePipelines: missing required shaders");
        return;
    }

    VkRenderPass mainPass = renderPassOverride_ ? renderPassOverride_ : vkCtx_->getImGuiRenderPass();
    VkSampleCountFlagBits samples = renderPassOverride_ ? msaaSamplesOverride_ : vkCtx_->getMsaaSamples();

    LOG_INFO("CharacterRenderer::recreatePipelines: renderPass=", (void*)mainPass,
             " samples=", static_cast<int>(samples),
             " pipelineLayout=", (void*)pipelineLayout_);

    buildMainPassPipelines(device, mainPass, samples, charVert, charFrag);

    charVert.destroy();
    charFrag.destroy();

    if (!opaquePipeline_ || !alphaTestPipeline_ || !alphaPipeline_ || !additivePipeline_ ||
        !translucentPipeline_) {
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
