#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <thread>
#include <unordered_set>

namespace wowee {
namespace rendering {

namespace {
size_t envSizeMBOrDefault(const char* name, size_t defMb) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) return defMb;
    char* end = nullptr;
    unsigned long long mb = std::strtoull(raw, &end, 10);
    if (end == raw || mb == 0) return defMb;
    return static_cast<size_t>(mb);
}

size_t envSizeOrDefault(const char* name, size_t defValue) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) return defValue;
    char* end = nullptr;
    unsigned long long v = std::strtoull(raw, &end, 10);
    if (end == raw || v == 0) return defValue;
    return static_cast<size_t>(v);
}
} // namespace

static void transformAABB(const glm::mat4& modelMatrix,
                          const glm::vec3& localMin,
                          const glm::vec3& localMax,
                          glm::vec3& outMin,
                          glm::vec3& outMax);

WMORenderer::WMORenderer() {
}

WMORenderer::~WMORenderer() {
    shutdown();
}

bool WMORenderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                              pipeline::AssetManager* assets) {
    if (initialized_) { assetManager = assets; return true; }
    core::Logger::getInstance().info("Initializing WMO renderer (Vulkan)...");

    vkCtx_ = ctx;
    assetManager = assets;

    if (!vkCtx_) {
        core::Logger::getInstance().error("WMORenderer: null VkContext");
        return false;
    }

    const unsigned hc = std::thread::hardware_concurrency();
    const size_t availableCores = (hc > 1u) ? static_cast<size_t>(hc - 1u) : 1ull;
    // WMO culling is lighter than animation; keep defaults conservative to reduce spikes.
    const size_t defaultCullThreads = std::max<size_t>(1, availableCores / 4);
    numCullThreads_ = static_cast<uint32_t>(std::max<size_t>(
        1, envSizeOrDefault("WOWEE_WMO_CULL_THREADS", defaultCullThreads)));
    core::Logger::getInstance().info("WMO cull threads: ", numCullThreads_);

    VkDevice device = vkCtx_->getDevice();

    // --- Create material descriptor set layout (set 1) ---
    // binding 0: sampler2D (diffuse texture)
    // binding 1: uniform buffer (WMOMaterial)
    // binding 2: sampler2D (normal+height map)
    std::vector<VkDescriptorSetLayoutBinding> materialBindings(3);
    materialBindings[0] = {};
    materialBindings[0].binding = 0;
    materialBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    materialBindings[0].descriptorCount = 1;
    materialBindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    materialBindings[1] = {};
    materialBindings[1].binding = 1;
    materialBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    materialBindings[1].descriptorCount = 1;
    materialBindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    materialBindings[2] = {};
    materialBindings[2].binding = 2;
    materialBindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    materialBindings[2].descriptorCount = 1;
    materialBindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    materialSetLayout_ = createDescriptorSetLayout(device, materialBindings);
    if (!materialSetLayout_) {
        core::Logger::getInstance().error("WMORenderer: failed to create material set layout");
        return false;
    }

    // --- Create descriptor pool ---
    VkDescriptorPoolSize poolSizes[] = {
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_MATERIAL_SETS * 2 },  // diffuse + normal/height
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_MATERIAL_SETS },
    };

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = MAX_MATERIAL_SETS;
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &materialDescPool_) != VK_SUCCESS) {
        core::Logger::getInstance().error("WMORenderer: failed to create descriptor pool");
        return false;
    }

    // --- Create pipeline layout ---
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(GPUPushConstants);

    std::vector<VkDescriptorSetLayout> setLayouts = { perFrameLayout, materialSetLayout_ };
    pipelineLayout_ = createPipelineLayout(device, setLayouts, { pushRange });
    if (!pipelineLayout_) {
        core::Logger::getInstance().error("WMORenderer: failed to create pipeline layout");
        return false;
    }

    // --- Load shaders ---
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/wmo.vert.spv")) {
        core::Logger::getInstance().error("WMORenderer: failed to load vertex shader");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/wmo.frag.spv")) {
        core::Logger::getInstance().error("WMORenderer: failed to load fragment shader");
        return false;
    }

    // --- Vertex input ---
    // WMO vertex: pos3 + normal3 + texCoord2 + color4 + tangent4 = 64 bytes
    struct WMOVertexData {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec4 color;
        glm::vec4 tangent;  // xyz=tangent dir, w=handedness ±1
    };

    VkVertexInputBindingDescription vertexBinding{};
    vertexBinding.binding = 0;
    vertexBinding.stride = sizeof(WMOVertexData);
    vertexBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> vertexAttribs(5);
    vertexAttribs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, position)) };
    vertexAttribs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, normal)) };
    vertexAttribs[2] = { 2, 0, VK_FORMAT_R32G32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, texCoord)) };
    vertexAttribs[3] = { 3, 0, VK_FORMAT_R32G32B32A32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, color)) };
    vertexAttribs[4] = { 4, 0, VK_FORMAT_R32G32B32A32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, tangent)) };

    // --- Build opaque pipeline ---
    VkRenderPass mainPass = vkCtx_->getImGuiRenderPass();

    opaquePipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    if (!opaquePipeline_) {
        core::Logger::getInstance().error("WMORenderer: failed to create opaque pipeline");
        vertShader.destroy();
        fragShader.destroy();
        return false;
    }

    // --- Build transparent pipeline ---
    transparentPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    if (!transparentPipeline_) {
        core::Logger::getInstance().warning("WMORenderer: transparent pipeline not available");
    }

    // --- Build glass pipeline (alpha blend WITH depth write for windows) ---
    glassPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    // --- Build wireframe pipeline ---
    wireframePipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_LINE, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    if (!wireframePipeline_) {
        core::Logger::getInstance().warning("WMORenderer: wireframe pipeline not available");
    }

    vertShader.destroy();
    fragShader.destroy();

    // --- Create fallback white texture ---
    whiteTexture_ = std::make_unique<VkTexture>();
    uint8_t whitePixel[4] = {255, 255, 255, 255};
    whiteTexture_->upload(*vkCtx_, whitePixel, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
    whiteTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                                  VK_SAMPLER_ADDRESS_MODE_REPEAT);

    // --- Create flat normal placeholder texture ---
    // (128,128,255,128) = flat normal pointing up (0,0,1), mid-height
    flatNormalTexture_ = std::make_unique<VkTexture>();
    uint8_t flatNormalPixel[4] = {128, 128, 255, 128};
    flatNormalTexture_->upload(*vkCtx_, flatNormalPixel, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
    flatNormalTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                                       VK_SAMPLER_ADDRESS_MODE_REPEAT);
    textureCacheBudgetBytes_ =
        envSizeMBOrDefault("WOWEE_WMO_TEX_CACHE_MB", 4096) * 1024ull * 1024ull;
    modelCacheLimit_ = envSizeMBOrDefault("WOWEE_WMO_MODEL_LIMIT", 4000);
    core::Logger::getInstance().info("WMO texture cache budget: ",
                                     textureCacheBudgetBytes_ / (1024 * 1024), " MB");
    core::Logger::getInstance().info("WMO model cache limit: ", modelCacheLimit_);

    core::Logger::getInstance().info("WMO renderer initialized (Vulkan)");
    initialized_ = true;
    return true;
}

void WMORenderer::shutdown() {
    core::Logger::getInstance().info("Shutting down WMO renderer...");

    if (!vkCtx_) {
        loadedModels.clear();
        instances.clear();
        spatialGrid.clear();
        instanceIndexById.clear();
        initialized_ = false;
        return;
    }

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator allocator = vkCtx_->getAllocator();

    vkDeviceWaitIdle(device);

    // Free all GPU resources for loaded models
    for (auto& [id, model] : loadedModels) {
        for (auto& group : model.groups) {
            destroyGroupGPU(group);
        }
    }

    // Free cached textures
    for (auto& [path, entry] : textureCache) {
        if (entry.texture) entry.texture->destroy(device, allocator);
        if (entry.normalHeightMap) entry.normalHeightMap->destroy(device, allocator);
    }
    textureCache.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;
    failedTextureCache_.clear();
    loggedTextureLoadFails_.clear();
    textureBudgetRejectWarnings_ = 0;

    // Free white texture and flat normal texture
    if (whiteTexture_) { whiteTexture_->destroy(device, allocator); whiteTexture_.reset(); }
    if (flatNormalTexture_) { flatNormalTexture_->destroy(device, allocator); flatNormalTexture_.reset(); }

    loadedModels.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();

    // Destroy pipelines
    if (opaquePipeline_) { vkDestroyPipeline(device, opaquePipeline_, nullptr); opaquePipeline_ = VK_NULL_HANDLE; }
    if (transparentPipeline_) { vkDestroyPipeline(device, transparentPipeline_, nullptr); transparentPipeline_ = VK_NULL_HANDLE; }
    if (glassPipeline_) { vkDestroyPipeline(device, glassPipeline_, nullptr); glassPipeline_ = VK_NULL_HANDLE; }
    if (wireframePipeline_) { vkDestroyPipeline(device, wireframePipeline_, nullptr); wireframePipeline_ = VK_NULL_HANDLE; }
    if (pipelineLayout_) { vkDestroyPipelineLayout(device, pipelineLayout_, nullptr); pipelineLayout_ = VK_NULL_HANDLE; }
    if (materialDescPool_) { vkDestroyDescriptorPool(device, materialDescPool_, nullptr); materialDescPool_ = VK_NULL_HANDLE; }
    if (materialSetLayout_) { vkDestroyDescriptorSetLayout(device, materialSetLayout_, nullptr); materialSetLayout_ = VK_NULL_HANDLE; }

    // Destroy shadow resources
    if (shadowPipeline_) { vkDestroyPipeline(device, shadowPipeline_, nullptr); shadowPipeline_ = VK_NULL_HANDLE; }
    if (shadowPipelineLayout_) { vkDestroyPipelineLayout(device, shadowPipelineLayout_, nullptr); shadowPipelineLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsPool_) { vkDestroyDescriptorPool(device, shadowParamsPool_, nullptr); shadowParamsPool_ = VK_NULL_HANDLE; }
    if (shadowParamsLayout_) { vkDestroyDescriptorSetLayout(device, shadowParamsLayout_, nullptr); shadowParamsLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsUBO_) { vmaDestroyBuffer(allocator, shadowParamsUBO_, shadowParamsAlloc_); shadowParamsUBO_ = VK_NULL_HANDLE; }

    vkCtx_ = nullptr;
    initialized_ = false;
}

bool WMORenderer::loadModel(const pipeline::WMOModel& model, uint32_t id) {
    if (!model.isValid()) {
        core::Logger::getInstance().error("Cannot load invalid WMO model");
        return false;
    }

    // Check if already loaded
    auto existingIt = loadedModels.find(id);
    if (existingIt != loadedModels.end()) {
        // If a model was first loaded while texture resolution failed (or before
        // assets were fully available), it can remain permanently white because
        // merged batches cache texture pointers at load time. Do a one-time reload for
        // models that have texture paths but no resolved non-white textures.
        if (assetManager && !model.textures.empty()) {
            bool hasResolvedTexture = false;
            for (VkTexture* tex : existingIt->second.textures) {
                if (tex != nullptr && tex != whiteTexture_.get()) {
                    hasResolvedTexture = true;
                    break;
                }
            }
            static std::unordered_set<uint32_t> retryReloadedModels;
            static bool retryReloadedModelsCapped = false;
            if (retryReloadedModels.size() > 8192) {
                retryReloadedModels.clear();
                if (!retryReloadedModelsCapped) {
                    core::Logger::getInstance().warning("WMO fallback-retry set exceeded 8192 entries; reset");
                    retryReloadedModelsCapped = true;
                }
            }
            if (!hasResolvedTexture && retryReloadedModels.insert(id).second) {
                core::Logger::getInstance().warning(
                    "WMO model ", id,
                    " has only fallback textures; forcing one-time reload");
                unloadModel(id);
            } else {
                return true;
            }
        } else {
            return true;
        }
    }
    if (loadedModels.size() >= modelCacheLimit_) {
        if (modelLimitRejectWarnings_ < 3) {
            core::Logger::getInstance().warning("WMO model cache full (",
                                                loadedModels.size(), "/", modelCacheLimit_,
                                                "), skipping model load: id=", id);
        }
        ++modelLimitRejectWarnings_;
        return false;
    }

    core::Logger::getInstance().debug("Loading WMO model ", id, " with ", model.groups.size(), " groups, ",
                                      model.textures.size(), " textures...");

    ModelData modelData;
    modelData.id = id;
    modelData.boundingBoxMin = model.boundingBoxMin;
    modelData.boundingBoxMax = model.boundingBoxMax;
    {
        glm::vec3 ext = model.boundingBoxMax - model.boundingBoxMin;
        float horiz = std::max(ext.x, ext.y);
        float vert = ext.z;
        modelData.isLowPlatform = (vert < 6.0f && horiz > 20.0f);
    }

    core::Logger::getInstance().debug("  WMO bounds: min=(", model.boundingBoxMin.x, ", ", model.boundingBoxMin.y, ", ", model.boundingBoxMin.z,
                                      ") max=(", model.boundingBoxMax.x, ", ", model.boundingBoxMax.y, ", ", model.boundingBoxMax.z, ")");

    // Load textures for this model
    core::Logger::getInstance().debug("  WMO has ", model.textures.size(), " texture paths, ", model.materials.size(), " materials");
    if (assetManager && !model.textures.empty()) {
        for (size_t i = 0; i < model.textures.size(); i++) {
            const auto& texPath = model.textures[i];
            core::Logger::getInstance().debug("    Loading texture ", i, ": ", texPath);
            VkTexture* tex = loadTexture(texPath);
            modelData.textures.push_back(tex);
            // Store lowercase texture name for material detection
            std::string lowerPath = texPath;
            std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            modelData.textureNames.push_back(lowerPath);
        }
        core::Logger::getInstance().debug("  Loaded ", modelData.textures.size(), " textures for WMO");
    }

    // Store material -> texture index mapping
    // IMPORTANT: mat.texture1 is a byte offset into MOTX, not an array index!
    // We need to convert it using the textureOffsetToIndex map
    core::Logger::getInstance().debug("  textureOffsetToIndex map has ", model.textureOffsetToIndex.size(), " entries");
    static int matLogCount = 0;
    auto resolveTextureIndex = [&](uint32_t textureField) -> uint32_t {
        auto it = model.textureOffsetToIndex.find(textureField);
        if (it != model.textureOffsetToIndex.end()) {
            return it->second;
        }
        // Some files may store direct index instead of MOTX byte offset.
        if (textureField < model.textures.size()) {
            return textureField;
        }
        return std::numeric_limits<uint32_t>::max();
    };

    for (size_t i = 0; i < model.materials.size(); i++) {
        const auto& mat = model.materials[i];
        uint32_t texIndex = 0;  // Default to first texture
        const uint32_t t1 = resolveTextureIndex(mat.texture1);
        const uint32_t t2 = resolveTextureIndex(mat.texture2);
        const uint32_t t3 = resolveTextureIndex(mat.texture3);

        // Prefer first valid non-empty texture among texture1/2/3.
        auto pickValid = [&](uint32_t idx) -> bool {
            if (idx == std::numeric_limits<uint32_t>::max()) return false;
            if (idx >= model.textures.size()) return false;
            if (model.textures[idx].empty()) return false;
            texIndex = idx;
            return true;
        };
        if (!pickValid(t1)) {
            if (!pickValid(t2)) {
                pickValid(t3);
            }
        }

        if (matLogCount < 20) {
            core::Logger::getInstance().debug("  Material ", i,
                ": tex1=", mat.texture1, "->", t1,
                " tex2=", mat.texture2, "->", t2,
                " tex3=", mat.texture3, "->", t3,
                " chosen=", texIndex);
            matLogCount++;
        }

        modelData.materialTextureIndices.push_back(texIndex);
        modelData.materialBlendModes.push_back(mat.blendMode);
        modelData.materialFlags.push_back(mat.flags);

    }

    // Helper: look up group name from MOGN raw data via MOGI nameOffset
    auto getGroupName = [&](uint32_t groupIdx) -> std::string {
        if (groupIdx < model.groupInfo.size()) {
            int32_t nameOff = model.groupInfo[groupIdx].nameOffset;
            if (nameOff >= 0 && static_cast<size_t>(nameOff) < model.groupNameRaw.size()) {
                const char* str = reinterpret_cast<const char*>(model.groupNameRaw.data() + nameOff);
                size_t maxLen = model.groupNameRaw.size() - nameOff;
                return std::string(str, strnlen(str, maxLen));
            }
        }
        return {};
    };

    // Create GPU resources for each group
    uint32_t loadedGroups = 0;
    for (size_t gi = 0; gi < model.groups.size(); gi++) {
        const auto& wmoGroup = model.groups[gi];
        // Skip empty groups
        if (wmoGroup.vertices.empty() || wmoGroup.indices.empty()) {
            continue;
        }

        GroupResources resources;
        if (createGroupResources(wmoGroup, resources, wmoGroup.flags)) {
            // Detect distance-only LOD/exterior shell groups:
            // 1. Very low vertex count (<100) — portal connectors, tiny shells
            // 2. ALWAYS_DRAW (0x10000) with low verts — distant LOD stand-ins
            // 3. Pure OUTDOOR groups (0x8 set, 0x2000 not set) in large WMOs —
            //    exterior cityscape shells (e.g. "city01" in Stormwind)
            bool alwaysDraw = (wmoGroup.flags & 0x10000) != 0;
            size_t nVerts = wmoGroup.vertices.size();
            bool isLargeWmo = model.nGroups > 50;
            // Detect facade groups by name (exterior face of buildings)
            std::string gname = getGroupName(static_cast<uint32_t>(gi));
            bool isFacade = false;
            bool isCityShell = false;
            if (!gname.empty()) {
                std::string lower = gname;
                std::transform(lower.begin(), lower.end(), lower.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                isFacade = lower.find("facade") != std::string::npos;
                // "city01" etc are exterior cityscape shells in large WMOs
                isCityShell = (lower.find("city") == 0 && lower.size() <= 8);
            }
            // Flag 0x80 on INDOOR groups in large WMOs = interior cathedral shell
            bool hasFlag80 = (wmoGroup.flags & 0x80) != 0;
            bool isIndoor = (wmoGroup.flags & 0x2000) != 0;
            if (nVerts < 100 || (alwaysDraw && nVerts < 5000) || (isFacade && isLargeWmo) || (isCityShell && isLargeWmo) || (hasFlag80 && isIndoor && isLargeWmo)) {
                resources.isLOD = true;
            }
            modelData.groups.push_back(resources);
            loadedGroups++;
        }
    }

    if (loadedGroups == 0) {
        core::Logger::getInstance().warning("No valid groups loaded for WMO ", id);
        return false;
    }

    // Build pre-merged batches for each group (texture-sorted for efficient rendering)
    for (auto& groupRes : modelData.groups) {
        // Use pointer value as key for batching
        struct BatchKey {
            uintptr_t texPtr;
            bool alphaTest;
            bool unlit;
            bool isWindow;
            bool operator==(const BatchKey& o) const { return texPtr == o.texPtr && alphaTest == o.alphaTest && unlit == o.unlit && isWindow == o.isWindow; }
        };
        struct BatchKeyHash {
            size_t operator()(const BatchKey& k) const {
                return std::hash<uintptr_t>()(k.texPtr) ^ (std::hash<bool>()(k.alphaTest) << 1) ^ (std::hash<bool>()(k.unlit) << 2) ^ (std::hash<bool>()(k.isWindow) << 3);
            }
        };
        std::unordered_map<BatchKey, GroupResources::MergedBatch, BatchKeyHash> batchMap;

        for (const auto& batch : groupRes.batches) {
            VkTexture* tex = whiteTexture_.get();
            bool hasTexture = false;

            if (batch.materialId < modelData.materialTextureIndices.size()) {
                uint32_t texIndex = modelData.materialTextureIndices[batch.materialId];
                if (texIndex < modelData.textures.size()) {
                    tex = modelData.textures[texIndex];
                    hasTexture = (tex != nullptr && tex != whiteTexture_.get());
                    if (!tex) tex = whiteTexture_.get();
                }
            }

            bool alphaTest = false;
            uint32_t blendMode = 0;
            if (batch.materialId < modelData.materialBlendModes.size()) {
                blendMode = modelData.materialBlendModes[batch.materialId];
                alphaTest = (blendMode == 1);
            }

            bool unlit = false;
            uint32_t matFlags = 0;
            if (batch.materialId < modelData.materialFlags.size()) {
                matFlags = modelData.materialFlags[batch.materialId];
                unlit = (matFlags & 0x01) != 0;
            }

            // Detect window/glass materials by texture name.
            // Flag 0x10 (F_SIDN) marks night-glow materials (windows AND lamps),
            // so we additionally check for "window" in the texture path to
            // distinguish actual glass from lamp post geometry.
            bool isWindow = false;
            if (batch.materialId < modelData.materialTextureIndices.size()) {
                uint32_t ti = modelData.materialTextureIndices[batch.materialId];
                if (ti < modelData.textureNames.size()) {
                    isWindow = (modelData.textureNames[ti].find("window") != std::string::npos);
                }
            }

            BatchKey key{ reinterpret_cast<uintptr_t>(tex), alphaTest, unlit, isWindow };
            auto& mb = batchMap[key];
            if (mb.draws.empty()) {
                mb.texture = tex;
                mb.hasTexture = hasTexture;
                mb.alphaTest = alphaTest;
                mb.unlit = unlit;
                mb.isTransparent = (blendMode >= 2);
                mb.isWindow = isWindow;
                // Look up normal/height map from texture cache
                if (hasTexture && tex != whiteTexture_.get()) {
                    for (const auto& [cacheKey, cacheEntry] : textureCache) {
                        if (cacheEntry.texture.get() == tex) {
                            mb.normalHeightMap = cacheEntry.normalHeightMap.get();
                            mb.heightMapVariance = cacheEntry.heightMapVariance;
                            break;
                        }
                    }
                }
            }
            GroupResources::MergedBatch::DrawRange dr;
            dr.firstIndex = batch.startIndex;
            dr.indexCount = batch.indexCount;
            mb.draws.push_back(dr);
        }

        // Allocate descriptor sets and UBOs for each merged batch
        groupRes.mergedBatches.reserve(batchMap.size());
        bool anyTextured = false;
        bool isInterior = (groupRes.groupFlags & 0x2000) != 0;
        for (auto& [key, mb] : batchMap) {
            if (mb.hasTexture) anyTextured = true;

            // Create material UBO
            VmaAllocator allocator = vkCtx_->getAllocator();
            AllocatedBuffer matBuf = createBuffer(allocator, sizeof(WMOMaterialUBO),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                VMA_MEMORY_USAGE_CPU_TO_GPU);
            mb.materialUBO = matBuf.buffer;
            mb.materialUBOAlloc = matBuf.allocation;

            // Write material params
            WMOMaterialUBO matData{};
            matData.hasTexture = mb.hasTexture ? 1 : 0;
            matData.alphaTest = mb.alphaTest ? 1 : 0;
            matData.unlit = mb.unlit ? 1 : 0;
            matData.isInterior = isInterior ? 1 : 0;
            matData.specularIntensity = 0.5f;
            matData.isWindow = mb.isWindow ? 1 : 0;
            matData.enableNormalMap = normalMappingEnabled_ ? 1 : 0;
            matData.enablePOM = pomEnabled_ ? 1 : 0;
            matData.pomScale = 0.012f;
            {
                static const int pomSampleTable[] = { 16, 32, 64 };
                matData.pomMaxSamples = pomSampleTable[std::clamp(pomQuality_, 0, 2)];
            }
            matData.heightMapVariance = mb.heightMapVariance;
            matData.normalMapStrength = normalMapStrength_;
            if (matBuf.info.pMappedData) {
                memcpy(matBuf.info.pMappedData, &matData, sizeof(matData));
            }

            // Allocate and write descriptor set
            mb.materialSet = allocateMaterialSet();
            if (mb.materialSet) {
                VkTexture* texToUse = mb.texture ? mb.texture : whiteTexture_.get();
                VkDescriptorImageInfo imgInfo = texToUse->descriptorInfo();

                VkDescriptorBufferInfo bufInfo{};
                bufInfo.buffer = mb.materialUBO;
                bufInfo.offset = 0;
                bufInfo.range = sizeof(WMOMaterialUBO);

                VkTexture* nhMap = mb.normalHeightMap ? mb.normalHeightMap : flatNormalTexture_.get();
                VkDescriptorImageInfo nhImgInfo = nhMap->descriptorInfo();

                VkWriteDescriptorSet writes[3] = {};
                writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[0].dstSet = mb.materialSet;
                writes[0].dstBinding = 0;
                writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[0].descriptorCount = 1;
                writes[0].pImageInfo = &imgInfo;

                writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[1].dstSet = mb.materialSet;
                writes[1].dstBinding = 1;
                writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                writes[1].descriptorCount = 1;
                writes[1].pBufferInfo = &bufInfo;

                writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[2].dstSet = mb.materialSet;
                writes[2].dstBinding = 2;
                writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[2].descriptorCount = 1;
                writes[2].pImageInfo = &nhImgInfo;

                vkUpdateDescriptorSets(vkCtx_->getDevice(), 3, writes, 0, nullptr);
            }

            groupRes.mergedBatches.push_back(std::move(mb));
        }
        groupRes.allUntextured = !anyTextured && !groupRes.mergedBatches.empty();
    }

    // Copy portal data for visibility culling
    modelData.portalVertices = model.portalVertices;
    for (const auto& portal : model.portals) {
        PortalData pd;
        pd.startVertex = portal.startVertex;
        pd.vertexCount = portal.vertexCount;
        // Compute portal plane from vertices if we have them
        if (portal.vertexCount >= 3 && portal.startVertex + portal.vertexCount <= model.portalVertices.size()) {
            glm::vec3 v0 = model.portalVertices[portal.startVertex];
            glm::vec3 v1 = model.portalVertices[portal.startVertex + 1];
            glm::vec3 v2 = model.portalVertices[portal.startVertex + 2];
            pd.normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
            pd.distance = glm::dot(pd.normal, v0);
        } else {
            pd.normal = glm::vec3(0.0f, 0.0f, 1.0f);
            pd.distance = 0.0f;
        }
        modelData.portals.push_back(pd);
    }
    for (const auto& ref : model.portalRefs) {
        PortalRef pr;
        pr.portalIndex = ref.portalIndex;
        pr.groupIndex = ref.groupIndex;
        pr.side = ref.side;
        modelData.portalRefs.push_back(pr);
    }
    // Build per-group portal ref ranges from WMOGroup data
    modelData.groupPortalRefs.resize(model.groups.size(), {0, 0});
    for (size_t gi = 0; gi < model.groups.size(); gi++) {
        modelData.groupPortalRefs[gi] = {model.groups[gi].portalStart, model.groups[gi].portalCount};
    }

    if (!modelData.portals.empty()) {
        core::Logger::getInstance().debug("WMO portals: ", modelData.portals.size(),
                                          " refs: ", modelData.portalRefs.size());
    }

    // Store doodad templates (M2 models placed in WMO) for instancing later
    if (!model.doodadSets.empty() && !model.doodads.empty()) {
        const auto& doodadSet = model.doodadSets[0];  // Use first doodad set
        for (uint32_t di = 0; di < doodadSet.count; di++) {
            uint32_t doodadIdx = doodadSet.startIndex + di;
            if (doodadIdx >= model.doodads.size()) break;

            const auto& doodad = model.doodads[doodadIdx];
            auto nameIt = model.doodadNames.find(doodad.nameIndex);
            if (nameIt == model.doodadNames.end()) continue;

            std::string m2Path = nameIt->second;
            if (m2Path.empty()) continue;

            // Convert .mdx/.mdl to .m2
            if (m2Path.size() > 4) {
                std::string ext = m2Path.substr(m2Path.size() - 4);
                for (char& c : ext) c = std::tolower(c);
                if (ext == ".mdx" || ext == ".mdl") {
                    m2Path = m2Path.substr(0, m2Path.size() - 4) + ".m2";
                }
            }

            // Build doodad's local transform (WoW coordinates)
            // WMO doodads use quaternion rotation (X/Y swapped for correct orientation)
            glm::quat fixedRotation(doodad.rotation.w, doodad.rotation.y, doodad.rotation.x, doodad.rotation.z);

            glm::mat4 localTransform(1.0f);
            localTransform = glm::translate(localTransform, doodad.position);
            localTransform *= glm::mat4_cast(fixedRotation);
            localTransform = glm::scale(localTransform, glm::vec3(doodad.scale));

            DoodadTemplate doodadTemplate;
            doodadTemplate.m2Path = m2Path;
            doodadTemplate.localTransform = localTransform;
            modelData.doodadTemplates.push_back(doodadTemplate);
        }

        if (!modelData.doodadTemplates.empty()) {
            core::Logger::getInstance().debug("WMO has ", modelData.doodadTemplates.size(), " doodad templates");
        }
    }

    loadedModels[id] = std::move(modelData);
    core::Logger::getInstance().debug("WMO model ", id, " loaded successfully (", loadedGroups, " groups)");
    return true;
}

void WMORenderer::unloadModel(uint32_t id) {
    auto it = loadedModels.find(id);
    if (it == loadedModels.end()) {
        return;
    }

    // Free GPU resources
    for (auto& group : it->second.groups) {
        destroyGroupGPU(group);
    }

    loadedModels.erase(it);
    core::Logger::getInstance().info("WMO model ", id, " unloaded");
}

void WMORenderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    // Find and remove models with no instances
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : loadedModels) {
        if (usedModelIds.find(id) == usedModelIds.end()) {
            toRemove.push_back(id);
        }
    }

    // Delete GPU resources and remove from map
    for (uint32_t id : toRemove) {
        unloadModel(id);
    }

    if (!toRemove.empty()) {
        core::Logger::getInstance().info("WMO cleanup: removed ", toRemove.size(), " unused models, ", loadedModels.size(), " remaining");
    }
}

uint32_t WMORenderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
    // Check if model is loaded
    if (loadedModels.find(modelId) == loadedModels.end()) {
        core::Logger::getInstance().error("Cannot create instance of unloaded WMO model ", modelId);
        return 0;
    }

    WMOInstance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();
    const ModelData& model = loadedModels[modelId];
    transformAABB(instance.modelMatrix, model.boundingBoxMin, model.boundingBoxMax,
                  instance.worldBoundsMin, instance.worldBoundsMax);

    // Pre-compute world-space group bounds to avoid per-frame transformAABB
    instance.worldGroupBounds.reserve(model.groups.size());
    for (const auto& group : model.groups) {
        glm::vec3 gMin, gMax;
        transformAABB(instance.modelMatrix, group.boundingBoxMin, group.boundingBoxMax, gMin, gMax);
        gMin -= glm::vec3(0.5f);
        gMax += glm::vec3(0.5f);
        instance.worldGroupBounds.emplace_back(gMin, gMax);
    }

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }
    core::Logger::getInstance().debug("Created WMO instance ", instance.id, " (model ", modelId, ")");
    return instance.id;
}

void WMORenderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.position = position;
    inst.updateModelMatrix();
    auto modelIt = loadedModels.find(inst.modelId);
    if (modelIt != loadedModels.end()) {
        const ModelData& model = modelIt->second;
        transformAABB(inst.modelMatrix, model.boundingBoxMin, model.boundingBoxMax,
                      inst.worldBoundsMin, inst.worldBoundsMax);
        inst.worldGroupBounds.clear();
        inst.worldGroupBounds.reserve(model.groups.size());
        for (const auto& group : model.groups) {
            glm::vec3 gMin, gMax;
            transformAABB(inst.modelMatrix, group.boundingBoxMin, group.boundingBoxMax, gMin, gMax);
            gMin -= glm::vec3(0.5f);
            gMax += glm::vec3(0.5f);
            inst.worldGroupBounds.emplace_back(gMin, gMax);
        }
    }
    rebuildSpatialIndex();
}

void WMORenderer::setInstanceTransform(uint32_t instanceId, const glm::mat4& transform) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];

    // Decompose transform to position/rotation/scale
    inst.position = glm::vec3(transform[3]);

    // Extract rotation (assuming uniform scale)
    glm::mat3 rotationMatrix(transform);
    float scaleX = glm::length(glm::vec3(transform[0]));
    float scaleY = glm::length(glm::vec3(transform[1]));
    float scaleZ = glm::length(glm::vec3(transform[2]));
    inst.scale = scaleX;  // Assume uniform scale

    if (scaleX > 0.0001f) rotationMatrix[0] /= scaleX;
    if (scaleY > 0.0001f) rotationMatrix[1] /= scaleY;
    if (scaleZ > 0.0001f) rotationMatrix[2] /= scaleZ;

    inst.rotation = glm::vec3(0.0f);  // Euler angles not directly used, so zero them

    // Update model matrix and bounds
    inst.modelMatrix = transform;
    inst.invModelMatrix = glm::inverse(transform);

    auto modelIt = loadedModels.find(inst.modelId);
    if (modelIt != loadedModels.end()) {
        const ModelData& model = modelIt->second;
        transformAABB(inst.modelMatrix, model.boundingBoxMin, model.boundingBoxMax,
                      inst.worldBoundsMin, inst.worldBoundsMax);
        inst.worldGroupBounds.clear();
        inst.worldGroupBounds.reserve(model.groups.size());
        for (const auto& group : model.groups) {
            glm::vec3 gMin, gMax;
            transformAABB(inst.modelMatrix, group.boundingBoxMin, group.boundingBoxMax, gMin, gMax);
            gMin -= glm::vec3(0.5f);
            gMax += glm::vec3(0.5f);
            inst.worldGroupBounds.emplace_back(gMin, gMax);
        }
    }

    // Propagate transform to child M2 doodads (chairs, furniture on transports)
    if (m2Renderer_ && !inst.doodads.empty()) {
        for (const auto& doodad : inst.doodads) {
            glm::mat4 worldTransform = inst.modelMatrix * doodad.localTransform;
            m2Renderer_->setInstanceTransform(doodad.m2InstanceId, worldTransform);
        }
    }

    // NOTE: Don't rebuild spatial index on every transform update - causes flickering
    // Spatial grid is only used for collision queries, render iterates all instances
    // rebuildSpatialIndex();
}

void WMORenderer::addDoodadToInstance(uint32_t instanceId, uint32_t m2InstanceId, const glm::mat4& localTransform) {
    auto it = std::find_if(instances.begin(), instances.end(),
                          [instanceId](const WMOInstance& inst) { return inst.id == instanceId; });
    if (it != instances.end()) {
        WMOInstance::DoodadInfo doodad;
        doodad.m2InstanceId = m2InstanceId;
        doodad.localTransform = localTransform;
        it->doodads.push_back(doodad);
    }
}

const std::vector<WMORenderer::DoodadTemplate>* WMORenderer::getDoodadTemplates(uint32_t modelId) const {
    auto it = loadedModels.find(modelId);
    if (it != loadedModels.end() && !it->second.doodadTemplates.empty()) {
        return &it->second.doodadTemplates;
    }
    return nullptr;
}

void WMORenderer::removeInstance(uint32_t instanceId) {
    auto it = std::find_if(instances.begin(), instances.end(),
                          [instanceId](const WMOInstance& inst) { return inst.id == instanceId; });
    if (it != instances.end()) {
        if (m2Renderer_) {
            for (const auto& doodad : it->doodads) {
                m2Renderer_->removeInstance(doodad.m2InstanceId);
            }
        }
        instances.erase(it);
        rebuildSpatialIndex();
        core::Logger::getInstance().debug("Removed WMO instance ", instanceId);
    }
}

void WMORenderer::removeInstances(const std::vector<uint32_t>& instanceIds) {
    if (instanceIds.empty() || instances.empty()) {
        return;
    }

    std::unordered_set<uint32_t> toRemove(instanceIds.begin(), instanceIds.end());
    if (m2Renderer_) {
        for (const auto& inst : instances) {
            if (toRemove.find(inst.id) == toRemove.end()) {
                continue;
            }
            for (const auto& doodad : inst.doodads) {
                m2Renderer_->removeInstance(doodad.m2InstanceId);
            }
        }
    }

    const size_t oldSize = instances.size();
    instances.erase(std::remove_if(instances.begin(), instances.end(),
                   [&toRemove](const WMOInstance& inst) {
                       return toRemove.find(inst.id) != toRemove.end();
                   }),
                   instances.end());

    if (instances.size() != oldSize) {
        rebuildSpatialIndex();
        core::Logger::getInstance().debug("Removed ", (oldSize - instances.size()),
                                          " WMO instances (batched)");
    }
}

void WMORenderer::clearInstances() {
    if (m2Renderer_) {
        for (const auto& inst : instances) {
            for (const auto& doodad : inst.doodads) {
                m2Renderer_->removeInstance(doodad.m2InstanceId);
            }
        }
    }
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    precomputedFloorGrid.clear();  // Invalidate floor cache when instances change
    core::Logger::getInstance().info("Cleared all WMO instances");
}

void WMORenderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocusEnabled = (radius > 0.0f);
    collisionFocusPos = worldPos;
    collisionFocusRadius = std::max(0.0f, radius);
    collisionFocusRadiusSq = collisionFocusRadius * collisionFocusRadius;
}

void WMORenderer::clearCollisionFocus() {
    collisionFocusEnabled = false;
}

// setLighting is now a no-op (lighting is in the per-frame UBO)

void WMORenderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
    currentFrameId++;
    // Note: precomputedFloorGrid is persistent and not cleared per-frame
}

bool WMORenderer::saveFloorCache() const {
    if (mapName_.empty()) {
        core::Logger::getInstance().warning("Cannot save floor cache: no map name set");
        return false;
    }

    std::string filepath = "cache/wmo_floor_" + mapName_ + ".bin";

    // Create directory if needed
    std::filesystem::path path(filepath);
    std::filesystem::path absPath = std::filesystem::absolute(path);
    core::Logger::getInstance().info("Saving floor cache to: ", absPath.string());

    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            core::Logger::getInstance().error("Failed to create cache directory: ", ec.message());
        }
    }

    std::ofstream file(filepath, std::ios::binary);
    if (!file) {
        core::Logger::getInstance().error("Failed to open floor cache file for writing: ", filepath);
        return false;
    }

    // Write header: magic + version + count
    const uint32_t magic = 0x574D4F46;  // "WMOF"
    const uint32_t version = 1;
    const uint64_t count = precomputedFloorGrid.size();

    file.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    file.write(reinterpret_cast<const char*>(&count), sizeof(count));

    // Write each entry: key (uint64) + height (float)
    for (const auto& [key, height] : precomputedFloorGrid) {
        file.write(reinterpret_cast<const char*>(&key), sizeof(key));
        file.write(reinterpret_cast<const char*>(&height), sizeof(height));
    }

    core::Logger::getInstance().info("Saved WMO floor cache (", mapName_, "): ", count, " entries");
    return true;
}

bool WMORenderer::loadFloorCache() {
    if (mapName_.empty()) {
        core::Logger::getInstance().warning("Cannot load floor cache: no map name set");
        return false;
    }

    std::string filepath = "cache/wmo_floor_" + mapName_ + ".bin";

    std::ifstream file(filepath, std::ios::binary);
    if (!file) {
        core::Logger::getInstance().info("No existing floor cache for map: ", mapName_);
        return false;
    }

    // Read and validate header
    uint32_t magic = 0, version = 0;
    uint64_t count = 0;

    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&count), sizeof(count));

    if (magic != 0x574D4F46 || version != 1) {
        core::Logger::getInstance().warning("Invalid floor cache file format: ", filepath);
        return false;
    }

    // Read entries
    precomputedFloorGrid.clear();
    precomputedFloorGrid.reserve(count);

    for (uint64_t i = 0; i < count; i++) {
        uint64_t key;
        float height;
        file.read(reinterpret_cast<char*>(&key), sizeof(key));
        file.read(reinterpret_cast<char*>(&height), sizeof(height));
        precomputedFloorGrid[key] = height;
    }

    core::Logger::getInstance().info("Loaded WMO floor cache (", mapName_, "): ", precomputedFloorGrid.size(), " entries");
    return true;
}

void WMORenderer::precomputeFloorCache() {
    if (instances.empty()) {
        core::Logger::getInstance().info("precomputeFloorCache: no instances to precompute");
        return;
    }

    size_t startSize = precomputedFloorGrid.size();
    size_t samplesChecked = 0;

    core::Logger::getInstance().info("Pre-computing floor cache for ", instances.size(), " WMO instances...");

    for (const auto& instance : instances) {
        // Get world bounds for this instance
        const glm::vec3& boundsMin = instance.worldBoundsMin;
        const glm::vec3& boundsMax = instance.worldBoundsMax;

        // Sample reference Z is above the structure
        float refZ = boundsMax.z + 10.0f;

        // Iterate over grid points within the bounds
        float startX = std::floor(boundsMin.x / FLOOR_GRID_CELL_SIZE) * FLOOR_GRID_CELL_SIZE;
        float startY = std::floor(boundsMin.y / FLOOR_GRID_CELL_SIZE) * FLOOR_GRID_CELL_SIZE;

        int stepsX = static_cast<int>((boundsMax.x - startX) / FLOOR_GRID_CELL_SIZE) + 1;
        int stepsY = static_cast<int>((boundsMax.y - startY) / FLOOR_GRID_CELL_SIZE) + 1;
        for (int ix = 0; ix < stepsX; ++ix) {
            float x = startX + ix * FLOOR_GRID_CELL_SIZE;
            for (int iy = 0; iy < stepsY; ++iy) {
                float y = startY + iy * FLOOR_GRID_CELL_SIZE;
                // Sample at grid cell center
                float sampleX = x + FLOOR_GRID_CELL_SIZE * 0.5f;
                float sampleY = y + FLOOR_GRID_CELL_SIZE * 0.5f;

                // Check if already cached
                uint64_t key = floorGridKey(sampleX, sampleY);
                if (precomputedFloorGrid.find(key) != precomputedFloorGrid.end()) {
                    continue;  // Already computed
                }

                samplesChecked++;

                // getFloorHeight will compute and cache the result
                getFloorHeight(sampleX, sampleY, refZ);
            }
        }
    }

    size_t newEntries = precomputedFloorGrid.size() - startSize;
    core::Logger::getInstance().info("Floor cache precompute complete: ", samplesChecked, " samples checked, ",
                                     newEntries, " new entries, total ", precomputedFloorGrid.size());
}

WMORenderer::GridCell WMORenderer::toCell(const glm::vec3& p) const {
    return GridCell{
        static_cast<int>(std::floor(p.x / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.y / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.z / SPATIAL_CELL_SIZE))
    };
}

void WMORenderer::rebuildSpatialIndex() {
    spatialGrid.clear();
    instanceIndexById.clear();
    instanceIndexById.reserve(instances.size());

    for (size_t i = 0; i < instances.size(); i++) {
        const auto& inst = instances[i];
        instanceIndexById[inst.id] = i;

        GridCell minCell = toCell(inst.worldBoundsMin);
        GridCell maxCell = toCell(inst.worldBoundsMax);
        for (int z = minCell.z; z <= maxCell.z; z++) {
            for (int y = minCell.y; y <= maxCell.y; y++) {
                for (int x = minCell.x; x <= maxCell.x; x++) {
                    spatialGrid[GridCell{x, y, z}].push_back(inst.id);
                }
            }
        }
    }
}

void WMORenderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
                                   std::vector<size_t>& outIndices) const {
    outIndices.clear();
    candidateIdScratch.clear();

    GridCell minCell = toCell(queryMin);
    GridCell maxCell = toCell(queryMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                auto it = spatialGrid.find(GridCell{x, y, z});
                if (it == spatialGrid.end()) continue;
                for (uint32_t id : it->second) {
                    if (!candidateIdScratch.insert(id).second) continue;
                    auto idxIt = instanceIndexById.find(id);
                    if (idxIt != instanceIndexById.end()) {
                        outIndices.push_back(idxIt->second);
                    }
                }
            }
        }
    }

    // Safety fallback: if the grid misses due streaming/index drift, avoid
    // tunneling by scanning all instances instead of returning no candidates.
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
}

void WMORenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (!opaquePipeline_ || instances.empty()) {
        lastDrawCalls = 0;
        return;
    }

    // Update material UBOs if settings changed
    if (materialSettingsDirty_) {
        materialSettingsDirty_ = false;
        static const int pomSampleTable[] = { 16, 32, 64 };
        int maxSamples = pomSampleTable[std::clamp(pomQuality_, 0, 2)];
        for (auto& [modelId, model] : loadedModels) {
            for (auto& group : model.groups) {
                for (auto& mb : group.mergedBatches) {
                    if (!mb.materialUBO) continue;
                    // Read existing UBO data, update normal/POM fields
                    VmaAllocationInfo allocInfo{};
                    vmaGetAllocationInfo(vkCtx_->getAllocator(), mb.materialUBOAlloc, &allocInfo);
                    if (allocInfo.pMappedData) {
                        auto* ubo = reinterpret_cast<WMOMaterialUBO*>(allocInfo.pMappedData);
                        ubo->enableNormalMap = normalMappingEnabled_ ? 1 : 0;
                        ubo->enablePOM = pomEnabled_ ? 1 : 0;
                        ubo->pomScale = 0.012f;
                        ubo->pomMaxSamples = maxSamples;
                        ubo->heightMapVariance = mb.heightMapVariance;
                        ubo->normalMapStrength = normalMapStrength_;
                    }
                }
            }
        }
    }

    lastDrawCalls = 0;

    // Extract frustum planes for proper culling
    glm::mat4 viewProj = camera.getProjectionMatrix() * camera.getViewMatrix();
    Frustum frustum;
    frustum.extractFromMatrix(viewProj);

    lastPortalCulledGroups = 0;
    lastDistanceCulledGroups = 0;

    // ── Phase 1: Parallel visibility culling ──────────────────────────
    std::vector<size_t> visibleInstances;
    visibleInstances.reserve(instances.size());
    for (size_t i = 0; i < instances.size(); ++i) {
        const auto& instance = instances[i];
        if (loadedModels.find(instance.modelId) == loadedModels.end())
            continue;
        visibleInstances.push_back(i);
    }

    glm::vec3 camPos = camera.getPosition();
    bool doPortalCull = portalCulling;
    bool doFrustumCull = false; // Temporarily disabled: can over-cull world WMOs
    bool doDistanceCull = distanceCulling;

    auto cullInstance = [&](size_t instIdx) -> InstanceDrawList {
        if (instIdx >= instances.size()) return InstanceDrawList{};
        const auto& instance = instances[instIdx];
        auto mdlIt = loadedModels.find(instance.modelId);
        if (mdlIt == loadedModels.end()) return InstanceDrawList{};
        const ModelData& model = mdlIt->second;

        InstanceDrawList result;
        result.instanceIndex = instIdx;

        // Portal-based visibility
        std::unordered_set<uint32_t> portalVisibleGroups;
        bool usePortalCulling = doPortalCull && !model.portals.empty() && !model.portalRefs.empty();
        if (usePortalCulling) {
            glm::vec4 localCamPos = instance.invModelMatrix * glm::vec4(camPos, 1.0f);
            getVisibleGroupsViaPortals(model, glm::vec3(localCamPos), frustum,
                                       instance.modelMatrix, portalVisibleGroups);
        }

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            if (usePortalCulling &&
                portalVisibleGroups.find(static_cast<uint32_t>(gi)) == portalVisibleGroups.end()) {
                result.portalCulled++;
                continue;
            }

            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];

                if (doDistanceCull) {
                    glm::vec3 closestPoint = glm::clamp(camPos, gMin, gMax);
                    float distSq = glm::dot(closestPoint - camPos, closestPoint - camPos);
                    if (distSq > 250000.0f) {
                        result.distanceCulled++;
                        continue;
                    }
                }

                if (doFrustumCull && !frustum.intersectsAABB(gMin, gMax))
                    continue;
            }

            result.visibleGroups.push_back(static_cast<uint32_t>(gi));
        }
        return result;
    };

    // Dispatch culling — parallel when enough instances, sequential otherwise.
    std::vector<InstanceDrawList> drawLists;
    drawLists.reserve(visibleInstances.size());

    static const size_t minParallelCullInstances = std::max<size_t>(
        4, envSizeOrDefault("WOWEE_WMO_CULL_MT_MIN", 128));
    if (visibleInstances.size() >= minParallelCullInstances && numCullThreads_ > 1) {
        static const size_t minCullWorkPerThread = std::max<size_t>(
            16, envSizeOrDefault("WOWEE_WMO_CULL_WORK_PER_THREAD", 64));
        const size_t maxUsefulThreads = std::max<size_t>(
            1, (visibleInstances.size() + minCullWorkPerThread - 1) / minCullWorkPerThread);
        const size_t numThreads = std::min(static_cast<size_t>(numCullThreads_), maxUsefulThreads);
        if (numThreads <= 1) {
            for (size_t idx : visibleInstances) {
                drawLists.push_back(cullInstance(idx));
            }
        } else {
            const size_t chunkSize = visibleInstances.size() / numThreads;
            const size_t remainder = visibleInstances.size() % numThreads;

            drawLists.resize(visibleInstances.size());

            cullFutures_.clear();
            if (cullFutures_.capacity() < numThreads) {
                cullFutures_.reserve(numThreads);
            }

            size_t start = 0;
            for (size_t t = 0; t < numThreads; ++t) {
                const size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                cullFutures_.push_back(std::async(std::launch::async,
                    [&, start, end]() {
                        for (size_t j = start; j < end; ++j) {
                            drawLists[j] = cullInstance(visibleInstances[j]);
                        }
                    }));
                start = end;
            }

            for (auto& f : cullFutures_) {
                f.get();
            }
        }
    } else {
        for (size_t idx : visibleInstances)
            drawLists.push_back(cullInstance(idx));
    }

    // ── Phase 2: Vulkan draw ────────────────────────────────
    // Select pipeline based on wireframe mode
    VkPipeline activePipeline = (wireframeMode && wireframePipeline_) ? wireframePipeline_ : opaquePipeline_;
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, activePipeline);

    // Bind per-frame descriptor set (set 0)
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                             0, 1, &perFrameSet, 0, nullptr);

    // Track which pipeline is currently bound: 0=opaque, 1=transparent, 2=glass
    int currentPipelineKind = 0;

    for (const auto& dl : drawLists) {
        if (dl.instanceIndex >= instances.size()) continue;
        const auto& instance = instances[dl.instanceIndex];
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) continue;
        const ModelData& model = modelIt->second;


        // Push model matrix
        GPUPushConstants push{};
        push.model = instance.modelMatrix;
        vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                            0, sizeof(GPUPushConstants), &push);

        // LOD shell groups render only beyond this distance squared (190 units)
        static constexpr float LOD_SHELL_DIST_SQ = 196.0f * 196.0f;

        // Render visible groups
        for (uint32_t gi : dl.visibleGroups) {
            const auto& group = model.groups[gi];

            // Only skip antiportal geometry
            if (group.groupFlags & 0x4000000) continue;

            // Skip distance-only LOD shell groups when camera is close to the group
            if (group.isLOD) {
                glm::vec3 groupCenter = instance.modelMatrix * glm::vec4(
                    (group.boundingBoxMin + group.boundingBoxMax) * 0.5f, 1.0f);
                float groupDistSq = glm::dot(camPos - groupCenter, camPos - groupCenter);
                if (groupDistSq < LOD_SHELL_DIST_SQ) continue;
            }

            // Bind vertex + index buffers
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &group.vertexBuffer, &offset);
            vkCmdBindIndexBuffer(cmd, group.indexBuffer, 0, VK_INDEX_TYPE_UINT16);

            // Render each merged batch
            for (const auto& mb : group.mergedBatches) {
                if (!mb.materialSet) continue;

                // Determine which pipeline this batch needs
                int neededPipeline = 0; // opaque
                if (mb.isWindow && glassPipeline_) {
                    neededPipeline = 2; // glass (alpha blend + depth write)
                } else if (mb.isTransparent && transparentPipeline_) {
                    neededPipeline = 1; // transparent (alpha blend, no depth write)
                }

                // Switch pipeline if needed (descriptor sets and push constants
                // are preserved across compatible pipeline layout switches)
                if (neededPipeline != currentPipelineKind) {
                    VkPipeline targetPipeline = activePipeline;
                    if (neededPipeline == 1) targetPipeline = transparentPipeline_;
                    else if (neededPipeline == 2) targetPipeline = glassPipeline_;

                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, targetPipeline);
                    currentPipelineKind = neededPipeline;
                }

                // Bind material descriptor set (set 1)
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                                         1, 1, &mb.materialSet, 0, nullptr);

                // Issue draw calls for each range in this merged batch
                for (const auto& dr : mb.draws) {
                    vkCmdDrawIndexed(cmd, dr.indexCount, 1, dr.firstIndex, 0, 0);
                    lastDrawCalls++;
                }
            }
        }

        lastPortalCulledGroups += dl.portalCulled;
        lastDistanceCulledGroups += dl.distanceCulled;
    }
}

bool WMORenderer::initializeShadow(VkRenderPass shadowRenderPass) {
    if (!vkCtx_ || shadowRenderPass == VK_NULL_HANDLE) return false;
    VkDevice device = vkCtx_->getDevice();

    // ShadowParams UBO: useBones, useTexture, alphaTest, foliageSway, windTime, foliageMotionDamp
    struct ShadowParamsUBO {
        int32_t useBones = 0;
        int32_t useTexture = 0;
        int32_t alphaTest = 0;
        int32_t foliageSway = 0;
        float windTime = 0.0f;
        float foliageMotionDamp = 1.0f;
    };

    // Create ShadowParams UBO
    VkBufferCreateInfo bufCI{};
    bufCI.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCI.size = sizeof(ShadowParamsUBO);
    bufCI.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo allocInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufCI, &allocCI,
            &shadowParamsUBO_, &shadowParamsAlloc_, &allocInfo) != VK_SUCCESS) {
        core::Logger::getInstance().error("WMORenderer: failed to create shadow params UBO");
        return false;
    }
    ShadowParamsUBO defaultParams{};
    std::memcpy(allocInfo.pMappedData, &defaultParams, sizeof(defaultParams));

    // Create descriptor set layout: binding 0 = sampler2D (texture), binding 1 = ShadowParams UBO
    VkDescriptorSetLayoutBinding layoutBindings[2]{};
    layoutBindings[0].binding = 0;
    layoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layoutBindings[0].descriptorCount = 1;
    layoutBindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    layoutBindings[1].binding = 1;
    layoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    layoutBindings[1].descriptorCount = 1;
    layoutBindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutCI{};
    layoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutCI.bindingCount = 2;
    layoutCI.pBindings = layoutBindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCI, nullptr, &shadowParamsLayout_) != VK_SUCCESS) {
        core::Logger::getInstance().error("WMORenderer: failed to create shadow params layout");
        return false;
    }

    // Create descriptor pool
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[1].descriptorCount = 1;
    VkDescriptorPoolCreateInfo poolCI{};
    poolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolCI.maxSets = 1;
    poolCI.poolSizeCount = 2;
    poolCI.pPoolSizes = poolSizes;
    if (vkCreateDescriptorPool(device, &poolCI, nullptr, &shadowParamsPool_) != VK_SUCCESS) {
        core::Logger::getInstance().error("WMORenderer: failed to create shadow params pool");
        return false;
    }

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo setAlloc{};
    setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    setAlloc.descriptorPool = shadowParamsPool_;
    setAlloc.descriptorSetCount = 1;
    setAlloc.pSetLayouts = &shadowParamsLayout_;
    if (vkAllocateDescriptorSets(device, &setAlloc, &shadowParamsSet_) != VK_SUCCESS) {
        core::Logger::getInstance().error("WMORenderer: failed to allocate shadow params set");
        return false;
    }

    // Write descriptors
    VkDescriptorBufferInfo bufInfo{};
    bufInfo.buffer = shadowParamsUBO_;
    bufInfo.offset = 0;
    bufInfo.range = sizeof(ShadowParamsUBO);

    VkWriteDescriptorSet writes[2]{};
    // binding 0: texture (use white fallback so binding is valid; useTexture=0 so it's not sampled)
    VkDescriptorImageInfo imgInfo{};
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgInfo.imageView = whiteTexture_->getImageView();
    imgInfo.sampler = whiteTexture_->getSampler();
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = shadowParamsSet_;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &imgInfo;
    // binding 1: params UBO
    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = shadowParamsSet_;
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[1].pBufferInfo = &bufInfo;
    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);

    // Create shadow pipeline layout: set 1 = shadowParamsLayout_, push constants = 128 bytes
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pc.offset = 0;
    pc.size = 128;  // lightSpaceMatrix (64) + model (64)
    shadowPipelineLayout_ = createPipelineLayout(device, {shadowParamsLayout_}, {pc});
    if (!shadowPipelineLayout_) {
        core::Logger::getInstance().error("WMORenderer: failed to create shadow pipeline layout");
        return false;
    }

    // Load shadow shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/shadow.vert.spv")) {
        core::Logger::getInstance().error("WMORenderer: failed to load shadow vertex shader");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/shadow.frag.spv")) {
        core::Logger::getInstance().error("WMORenderer: failed to load shadow fragment shader");
        return false;
    }

    // WMO vertex layout: pos(loc0,off0) normal(loc1,off12) texCoord(loc2,off24) color(loc3,off32) tangent(loc4,off48), stride=64
    // Shadow shader locations: 0=aPos, 1=aTexCoord, 2=aBoneWeights, 3=aBoneIndicesF
    // useBones=0 so locations 2,3 are never read; we alias them to existing data offsets
    VkVertexInputBindingDescription vertBind{};
    vertBind.binding = 0;
    vertBind.stride = 64;
    vertBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::vector<VkVertexInputAttributeDescription> vertAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT,    0},   // aPos       -> position
        {1, 0, VK_FORMAT_R32G32_SFLOAT,       24},  // aTexCoord  -> texCoord
        {2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 32},  // aBoneWeights (aliased to color, not used)
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 32},  // aBoneIndicesF (aliased to color, not used)
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
        core::Logger::getInstance().error("WMORenderer: failed to create shadow pipeline");
        return false;
    }
    core::Logger::getInstance().info("WMORenderer shadow pipeline initialized");
    return true;
}

void WMORenderer::renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix,
                               const glm::vec3& shadowCenter, float shadowRadius) {
    if (!shadowPipeline_ || !shadowParamsSet_) return;
    if (instances.empty() || loadedModels.empty()) return;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
        0, 1, &shadowParamsSet_, 0, nullptr);

    struct ShadowPush { glm::mat4 lightSpaceMatrix; glm::mat4 model; };

    const float shadowRadiusSq = shadowRadius * shadowRadius;
    for (const auto& instance : instances) {
        // Distance cull against shadow frustum
        glm::vec3 diff = instance.position - shadowCenter;
        if (glm::dot(diff, diff) > shadowRadiusSq) continue;
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) continue;
        const ModelData& model = modelIt->second;

        ShadowPush push{lightSpaceMatrix, instance.modelMatrix};
        vkCmdPushConstants(cmd, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                           0, 128, &push);

        for (const auto& group : model.groups) {
            if (group.vertexBuffer == VK_NULL_HANDLE || group.indexBuffer == VK_NULL_HANDLE) continue;

            // Skip antiportal geometry
            if (group.groupFlags & 0x4000000) continue;

            // Skip LOD groups in shadow pass (they overlap real geometry)
            if (group.isLOD) continue;

            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &group.vertexBuffer, &offset);
            vkCmdBindIndexBuffer(cmd, group.indexBuffer, 0, VK_INDEX_TYPE_UINT16);

            // Draw all batches in shadow pass.
            // WMO transparency classification is not reliable enough for caster
            // selection here and was dropping major world casters.
            for (const auto& mb : group.mergedBatches) {
                for (const auto& dr : mb.draws) {
                    vkCmdDrawIndexed(cmd, dr.indexCount, 1, dr.firstIndex, 0, 0);
                }
            }
        }
    }
}

uint32_t WMORenderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt != loadedModels.end()) {
            total += modelIt->second.getTotalTriangles();
        }
    }
    return total;
}

bool WMORenderer::createGroupResources(const pipeline::WMOGroup& group, GroupResources& resources, uint32_t groupFlags) {
    if (group.vertices.empty() || group.indices.empty()) {
        return false;
    }

    resources.groupFlags = groupFlags;

    resources.vertexCount = group.vertices.size();
    resources.indexCount = group.indices.size();
    resources.boundingBoxMin = group.boundingBoxMin;
    resources.boundingBoxMax = group.boundingBoxMax;

    // Create vertex data (position, normal, texcoord, color, tangent)
    struct VertexData {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec4 color;
        glm::vec4 tangent;  // xyz=tangent dir, w=handedness ±1
    };

    std::vector<VertexData> vertices;
    vertices.reserve(group.vertices.size());

    for (const auto& v : group.vertices) {
        VertexData vd;
        vd.position = v.position;
        vd.normal = v.normal;
        vd.texCoord = v.texCoord;
        vd.color = v.color;
        vd.tangent = glm::vec4(0.0f);
        vertices.push_back(vd);
    }

    // Compute tangents using Lengyel's method
    {
        std::vector<glm::vec3> tan1(vertices.size(), glm::vec3(0.0f));
        std::vector<glm::vec3> tan2(vertices.size(), glm::vec3(0.0f));

        const auto& indices = group.indices;
        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            uint16_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
            if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) continue;

            const glm::vec3& p0 = vertices[i0].position;
            const glm::vec3& p1 = vertices[i1].position;
            const glm::vec3& p2 = vertices[i2].position;
            const glm::vec2& uv0 = vertices[i0].texCoord;
            const glm::vec2& uv1 = vertices[i1].texCoord;
            const glm::vec2& uv2 = vertices[i2].texCoord;

            glm::vec3 dp1 = p1 - p0;
            glm::vec3 dp2 = p2 - p0;
            glm::vec2 duv1 = uv1 - uv0;
            glm::vec2 duv2 = uv2 - uv0;

            float det = duv1.x * duv2.y - duv1.y * duv2.x;
            if (std::abs(det) < 1e-8f) continue;  // degenerate UVs
            float r = 1.0f / det;

            glm::vec3 sdir = (dp1 * duv2.y - dp2 * duv1.y) * r;
            glm::vec3 tdir = (dp2 * duv1.x - dp1 * duv2.x) * r;

            tan1[i0] += sdir; tan1[i1] += sdir; tan1[i2] += sdir;
            tan2[i0] += tdir; tan2[i1] += tdir; tan2[i2] += tdir;
        }

        for (size_t i = 0; i < vertices.size(); i++) {
            glm::vec3 n = glm::normalize(vertices[i].normal);
            glm::vec3 t = tan1[i];

            if (glm::dot(t, t) < 1e-8f) {
                // Fallback: generate tangent perpendicular to normal
                glm::vec3 up = (std::abs(n.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
                t = glm::normalize(glm::cross(n, up));
            }

            // Gram-Schmidt orthogonalize
            t = glm::normalize(t - n * glm::dot(n, t));
            float w = (glm::dot(glm::cross(n, t), tan2[i]) < 0.0f) ? -1.0f : 1.0f;
            vertices[i].tangent = glm::vec4(t, w);
        }
    }

    // Upload vertex buffer to GPU
    AllocatedBuffer vertBuf = uploadBuffer(*vkCtx_, vertices.data(),
        vertices.size() * sizeof(VertexData),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    resources.vertexBuffer = vertBuf.buffer;
    resources.vertexAlloc = vertBuf.allocation;

    // Upload index buffer to GPU
    AllocatedBuffer idxBuf = uploadBuffer(*vkCtx_, group.indices.data(),
        group.indices.size() * sizeof(uint16_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    resources.indexBuffer = idxBuf.buffer;
    resources.indexAlloc = idxBuf.allocation;

    // Store collision geometry for floor raycasting
    resources.collisionVertices.reserve(group.vertices.size());
    for (const auto& v : group.vertices) {
        resources.collisionVertices.push_back(v.position);
    }
    resources.collisionIndices = group.indices;

    // Compute actual bounding box from vertices (WMO header bboxes can be unreliable)
    if (!resources.collisionVertices.empty()) {
        resources.boundingBoxMin = resources.collisionVertices[0];
        resources.boundingBoxMax = resources.collisionVertices[0];
        for (const auto& v : resources.collisionVertices) {
            resources.boundingBoxMin = glm::min(resources.boundingBoxMin, v);
            resources.boundingBoxMax = glm::max(resources.boundingBoxMax, v);
        }
    }

    // Build 2D spatial grid for fast collision triangle lookup
    resources.buildCollisionGrid();

    // Create batches
    if (!group.batches.empty()) {
        for (const auto& batch : group.batches) {
            GroupResources::Batch resBatch;
            resBatch.startIndex = batch.startIndex;
            resBatch.indexCount = batch.indexCount;
            resBatch.materialId = batch.materialId;
            resources.batches.push_back(resBatch);
        }
    } else {
        // No batches defined - render entire group as one batch
        GroupResources::Batch batch;
        batch.startIndex = 0;
        batch.indexCount = resources.indexCount;
        batch.materialId = 0;
        resources.batches.push_back(batch);
    }

    return true;
}

// renderGroup removed — draw calls are inlined in render()

void WMORenderer::destroyGroupGPU(GroupResources& group) {
    if (!vkCtx_) return;
    VmaAllocator allocator = vkCtx_->getAllocator();

    if (group.vertexBuffer) {
        vmaDestroyBuffer(allocator, group.vertexBuffer, group.vertexAlloc);
        group.vertexBuffer = VK_NULL_HANDLE;
        group.vertexAlloc = VK_NULL_HANDLE;
    }
    if (group.indexBuffer) {
        vmaDestroyBuffer(allocator, group.indexBuffer, group.indexAlloc);
        group.indexBuffer = VK_NULL_HANDLE;
        group.indexAlloc = VK_NULL_HANDLE;
    }

    // Destroy material UBOs (descriptor sets are freed when pool is reset/destroyed)
    for (auto& mb : group.mergedBatches) {
        if (mb.materialUBO) {
            vmaDestroyBuffer(allocator, mb.materialUBO, mb.materialUBOAlloc);
            mb.materialUBO = VK_NULL_HANDLE;
            mb.materialUBOAlloc = VK_NULL_HANDLE;
        }
    }
}

VkDescriptorSet WMORenderer::allocateMaterialSet() {
    if (!materialDescPool_ || !materialSetLayout_) return VK_NULL_HANDLE;

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = materialDescPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &materialSetLayout_;

    VkDescriptorSet set = VK_NULL_HANDLE;
    if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &allocInfo, &set) != VK_SUCCESS) {
        core::Logger::getInstance().warning("WMORenderer: failed to allocate material descriptor set");
        return VK_NULL_HANDLE;
    }
    return set;
}

bool WMORenderer::isGroupVisible(const GroupResources& group, const glm::mat4& modelMatrix,
                                 const Camera& camera) const {
    // Simple frustum culling using bounding box
    // Transform bounding box corners to world space
    glm::vec3 corners[8] = {
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMin.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMin.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMax.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMax.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMin.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMin.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMax.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMax.y, group.boundingBoxMax.z)
    };

    // Transform corners to world space
    for (int i = 0; i < 8; i++) {
        glm::vec4 worldPos = modelMatrix * glm::vec4(corners[i], 1.0f);
        corners[i] = glm::vec3(worldPos);
    }

    // Simple check: if all corners are behind camera, cull
    // (This is a very basic culling implementation - a full frustum test would be better)
    glm::vec3 forward = camera.getForward();
    glm::vec3 camPos = camera.getPosition();

    int behindCount = 0;
    for (int i = 0; i < 8; i++) {
        glm::vec3 toCorner = corners[i] - camPos;
        if (glm::dot(toCorner, forward) < 0.0f) {
            behindCount++;
        }
    }

    // If all corners are behind camera, cull
    return behindCount < 8;
}

int WMORenderer::findContainingGroup(const ModelData& model, const glm::vec3& localPos) const {
    // Find which group's bounding box contains the position
    // Prefer interior groups (smaller volume) when multiple match
    int bestGroup = -1;
    float bestVolume = std::numeric_limits<float>::max();

    for (size_t gi = 0; gi < model.groups.size(); gi++) {
        const auto& group = model.groups[gi];
        if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
            localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
            localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
            glm::vec3 size = group.boundingBoxMax - group.boundingBoxMin;
            float volume = size.x * size.y * size.z;
            if (volume < bestVolume) {
                bestVolume = volume;
                bestGroup = static_cast<int>(gi);
            }
        }
    }
    return bestGroup;
}

bool WMORenderer::isPortalVisible(const ModelData& model, uint16_t portalIndex,
                                   [[maybe_unused]] const glm::vec3& cameraLocalPos,
                                   const Frustum& frustum,
                                   const glm::mat4& modelMatrix) const {
    if (portalIndex >= model.portals.size()) return false;

    const auto& portal = model.portals[portalIndex];
    if (portal.vertexCount < 3) return false;
    if (portal.startVertex + portal.vertexCount > model.portalVertices.size()) return false;

    // Get portal polygon center and bounds for frustum test
    glm::vec3 center(0.0f);
    glm::vec3 pMin = model.portalVertices[portal.startVertex];
    glm::vec3 pMax = pMin;
    for (uint16_t i = 0; i < portal.vertexCount; i++) {
        const auto& v = model.portalVertices[portal.startVertex + i];
        center += v;
        pMin = glm::min(pMin, v);
        pMax = glm::max(pMax, v);
    }
    center /= static_cast<float>(portal.vertexCount);

    // Transform bounds to world space for frustum test
    glm::vec4 worldMin = modelMatrix * glm::vec4(pMin, 1.0f);
    glm::vec4 worldMax = modelMatrix * glm::vec4(pMax, 1.0f);

    // Check if portal AABB intersects frustum (more robust than point test)
    return frustum.intersectsAABB(glm::vec3(worldMin), glm::vec3(worldMax));
}

void WMORenderer::getVisibleGroupsViaPortals(const ModelData& model,
                                              const glm::vec3& cameraLocalPos,
                                              const Frustum& frustum,
                                              const glm::mat4& modelMatrix,
                                              std::unordered_set<uint32_t>& outVisibleGroups) const {
    // Find camera's containing group
    int cameraGroup = findContainingGroup(model, cameraLocalPos);

    // If camera is outside all groups, fall back to frustum culling only
    if (cameraGroup < 0) {
        // Camera outside WMO - mark all groups as potentially visible
        // (will still be frustum culled in render)
        for (size_t gi = 0; gi < model.groups.size(); gi++) {
            outVisibleGroups.insert(static_cast<uint32_t>(gi));
        }
        return;
    }

    // BFS through portals from camera's group
    std::vector<bool> visited(model.groups.size(), false);
    std::vector<uint32_t> queue;
    queue.push_back(static_cast<uint32_t>(cameraGroup));
    visited[cameraGroup] = true;
    outVisibleGroups.insert(static_cast<uint32_t>(cameraGroup));

    size_t queueIdx = 0;
    while (queueIdx < queue.size()) {
        uint32_t currentGroup = queue[queueIdx++];

        // Get portal refs for this group
        if (currentGroup >= model.groupPortalRefs.size()) continue;
        auto [portalStart, portalCount] = model.groupPortalRefs[currentGroup];

        for (uint16_t pi = 0; pi < portalCount; pi++) {
            uint16_t refIdx = portalStart + pi;
            if (refIdx >= model.portalRefs.size()) continue;

            const auto& ref = model.portalRefs[refIdx];
            uint32_t targetGroup = ref.groupIndex;

            if (targetGroup >= model.groups.size()) continue;
            if (visited[targetGroup]) continue;

            // Check if portal is visible from camera
            if (isPortalVisible(model, ref.portalIndex, cameraLocalPos, frustum, modelMatrix)) {
                visited[targetGroup] = true;
                outVisibleGroups.insert(targetGroup);
                queue.push_back(targetGroup);
            }
        }
    }
}

void WMORenderer::WMOInstance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Apply MODF placement rotation (WoW-to-GL coordinate transform)
    // WoW Ry(B)*Rx(A)*Rz(C) becomes GL Rz(B)*Ry(-A)*Rx(-C)
    // rotation stored as (-C, -A, B) in radians by caller
    // Apply in Z, Y, X order to get Rz(B) * Ry(-A) * Rx(-C)
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));

    // Cache inverse for collision detection
    invModelMatrix = glm::inverse(modelMatrix);
}

std::unique_ptr<VkTexture> WMORenderer::generateNormalHeightMap(
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

    // Step 2: Sobel 3x3 → normal map
    // Use ORIGINAL height for normals (crisp detail), blurred height for POM alpha only
    const float strength = 2.0f;
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
            // Sobel X
            float gx = -sampleH(ix-1, iy-1) - 2.0f*sampleH(ix-1, iy) - sampleH(ix-1, iy+1)
                       + sampleH(ix+1, iy-1) + 2.0f*sampleH(ix+1, iy) + sampleH(ix+1, iy+1);
            // Sobel Y
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

    // Step 3: Upload to GPU with mipmaps
    auto tex = std::make_unique<VkTexture>();
    if (!tex->upload(*vkCtx_, output.data(), width, height, VK_FORMAT_R8G8B8A8_UNORM, true)) {
        return nullptr;
    }
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                        VK_SAMPLER_ADDRESS_MODE_REPEAT);
    return tex;
}

VkTexture* WMORenderer::loadTexture(const std::string& path) {
    if (!assetManager || !vkCtx_) {
        return whiteTexture_.get();
    }

    auto normalizeKey = [](std::string key) {
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    };
    std::string key = path;
    // Some assets contain stray bytes after a NUL in path chunks.
    size_t nul = key.find('\0');
    if (nul != std::string::npos) key.resize(nul);
    key = normalizeKey(key);
    if (key.rfind(".\\", 0) == 0) key = key.substr(2);
    while (!key.empty() && key.front() == '\\') key.erase(key.begin());
    if (key.empty()) return whiteTexture_.get();

    auto hasKnownExt = [](const std::string& p) {
        if (p.size() < 4) return false;
        std::string ext = p.substr(p.size() - 4);
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return (ext == ".blp" || ext == ".tga" || ext == ".dds");
    };
    auto toBlp = [](std::string p) {
        if (p.size() >= 4) {
            std::string ext = p.substr(p.size() - 4);
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (ext == ".tga" || ext == ".dds") {
                p = p.substr(0, p.size() - 4) + ".blp";
            }
        }
        return p;
    };

    std::vector<std::string> candidates;
    auto addCandidate = [&](const std::string& raw) {
        std::string c = normalizeKey(raw);
        if (c.rfind(".\\", 0) == 0) c = c.substr(2);
        while (!c.empty() && c.front() == '\\') c.erase(c.begin());
        if (!c.empty()) candidates.push_back(c);
    };

    addCandidate(toBlp(key));
    if (!hasKnownExt(key)) addCandidate(key + ".blp");

    // Common WMO references omit folder prefix; manifest often stores these under textures\...
    std::string keyWithExt = hasKnownExt(key) ? toBlp(key) : (key + ".blp");
    if (key.find('\\') == std::string::npos) {
        addCandidate(std::string("textures\\") + keyWithExt);
    }
    if (key.rfind("texture\\", 0) == 0) {
        addCandidate(std::string("textures\\") + key.substr(8));
    }

    // De-duplicate while preserving order.
    std::vector<std::string> uniqueCandidates;
    uniqueCandidates.reserve(candidates.size());
    std::unordered_set<std::string> seen;
    for (const auto& c : candidates) {
        if (seen.insert(c).second) uniqueCandidates.push_back(c);
    }

    // Cache lookup across all candidate keys
    for (const auto& c : uniqueCandidates) {
        auto it = textureCache.find(c);
        if (it != textureCache.end()) {
            it->second.lastUse = ++textureCacheCounter_;
            return it->second.texture.get();
        }
    }

    const auto& attemptedCandidates = uniqueCandidates;

    // Try loading all candidates until one succeeds
    pipeline::BLPImage blp;
    std::string resolvedKey;
    for (const auto& c : attemptedCandidates) {
        blp = assetManager->loadTexture(c);
        if (blp.isValid()) {
            resolvedKey = c;
            break;
        }
    }
    if (!blp.isValid()) {
        if (loggedTextureLoadFails_.insert(key).second) {
            core::Logger::getInstance().warning("WMO: Failed to load texture: ", path);
        }
        // Do not cache failures as white. MPQ reads can fail transiently
        // during streaming/contention, and caching white here permanently
        // poisons the texture for this session.
        return whiteTexture_.get();
    }

    core::Logger::getInstance().debug("WMO texture: ", path, " size=", blp.width, "x", blp.height);

    size_t base = static_cast<size_t>(blp.width) * static_cast<size_t>(blp.height) * 4ull;
    size_t approxBytes = base + (base / 3);
    if (textureCacheBytes_ + approxBytes > textureCacheBudgetBytes_) {
        if (textureBudgetRejectWarnings_ < 3) {
            core::Logger::getInstance().warning(
                "WMO texture cache full (", textureCacheBytes_ / (1024 * 1024),
                " MB / ", textureCacheBudgetBytes_ / (1024 * 1024),
                " MB), rejecting texture: ", path);
        }
        ++textureBudgetRejectWarnings_;
        return whiteTexture_.get();
    }

    // Create Vulkan texture
    auto texture = std::make_unique<VkTexture>();
    if (!texture->upload(*vkCtx_, blp.data.data(), blp.width, blp.height,
                          VK_FORMAT_R8G8B8A8_UNORM, true)) {
        core::Logger::getInstance().warning("WMO: Failed to upload texture to GPU: ", path);
        return whiteTexture_.get();
    }
    texture->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                            VK_SAMPLER_ADDRESS_MODE_REPEAT);

    // Generate normal+height map from diffuse pixels
    float nhVariance = 0.0f;
    std::unique_ptr<VkTexture> nhMap;
    if (normalMappingEnabled_ || pomEnabled_) {
        nhMap = generateNormalHeightMap(blp.data.data(), blp.width, blp.height, nhVariance);
        if (nhMap) {
            approxBytes *= 2;  // account for normal map in budget
        }
    }

    // Cache it
    TextureCacheEntry e;
    VkTexture* rawPtr = texture.get();
    e.approxBytes = approxBytes;
    e.lastUse = ++textureCacheCounter_;
    e.texture = std::move(texture);
    e.normalHeightMap = std::move(nhMap);
    e.heightMapVariance = nhVariance;
    textureCacheBytes_ += e.approxBytes;
    if (!resolvedKey.empty()) {
        textureCache[resolvedKey] = std::move(e);
    } else {
        textureCache[key] = std::move(e);
    }
    core::Logger::getInstance().debug("WMO: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return rawPtr;
}

// Ray-AABB intersection (slab method)
// Returns true if the ray intersects the axis-aligned bounding box
static bool rayIntersectsAABB(const glm::vec3& origin, const glm::vec3& dir,
                               const glm::vec3& bmin, const glm::vec3& bmax) {
    float tmin = -1e30f, tmax = 1e30f;
    for (int i = 0; i < 3; i++) {
        if (std::abs(dir[i]) < 1e-8f) {
            // Ray is parallel to this slab — check if origin is inside
            if (origin[i] < bmin[i] || origin[i] > bmax[i]) return false;
        } else {
            float invD = 1.0f / dir[i];
            float t0 = (bmin[i] - origin[i]) * invD;
            float t1 = (bmax[i] - origin[i]) * invD;
            if (t0 > t1) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmin > tmax) return false;
        }
    }
    return tmax >= 0.0f;  // At least part of the ray is forward
}

static void transformAABB(const glm::mat4& modelMatrix,
                          const glm::vec3& localMin,
                          const glm::vec3& localMax,
                          glm::vec3& outMin,
                          glm::vec3& outMax) {
    const glm::vec3 corners[8] = {
        {localMin.x, localMin.y, localMin.z},
        {localMin.x, localMin.y, localMax.z},
        {localMin.x, localMax.y, localMin.z},
        {localMin.x, localMax.y, localMax.z},
        {localMax.x, localMin.y, localMin.z},
        {localMax.x, localMin.y, localMax.z},
        {localMax.x, localMax.y, localMin.z},
        {localMax.x, localMax.y, localMax.z}
    };

    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(-std::numeric_limits<float>::max());
    for (const glm::vec3& corner : corners) {
        glm::vec3 world = glm::vec3(modelMatrix * glm::vec4(corner, 1.0f));
        outMin = glm::min(outMin, world);
        outMax = glm::max(outMax, world);
    }
}

static float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax) {
    glm::vec3 q = glm::clamp(p, bmin, bmax);
    glm::vec3 d = p - q;
    return glm::dot(d, d);
}

struct QueryTimer {
    double* totalMs = nullptr;
    uint32_t* callCount = nullptr;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    QueryTimer(double* total, uint32_t* calls) : totalMs(total), callCount(calls) {}
    ~QueryTimer() {
        if (callCount) {
            (*callCount)++;
        }
        if (totalMs) {
            auto end = std::chrono::steady_clock::now();
            *totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }
    }
};

// Möller–Trumbore ray-triangle intersection
// Returns distance along ray if hit, or negative if miss
static float rayTriangleIntersect(const glm::vec3& origin, const glm::vec3& dir,
                                   const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    const float EPSILON = 1e-6f;
    glm::vec3 e1 = v1 - v0;
    glm::vec3 e2 = v2 - v0;
    glm::vec3 h = glm::cross(dir, e2);
    float a = glm::dot(e1, h);
    if (a > -EPSILON && a < EPSILON) return -1.0f;

    float f = 1.0f / a;
    glm::vec3 s = origin - v0;
    float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return -1.0f;

    glm::vec3 q = glm::cross(s, e1);
    float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return -1.0f;

    float t = f * glm::dot(e2, q);
    return t > EPSILON ? t : -1.0f;
}

// Closest point on triangle (from Real-Time Collision Detection).
static glm::vec3 closestPointOnTriangle(const glm::vec3& p, const glm::vec3& a,
                                        const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ap = p - a;
    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

// ---- Per-group 2D collision grid ----

void WMORenderer::GroupResources::buildCollisionGrid() {
    if (collisionVertices.empty() || collisionIndices.size() < 3) {
        gridCellsX = 0;
        gridCellsY = 0;
        return;
    }

    gridOrigin = glm::vec2(boundingBoxMin.x, boundingBoxMin.y);
    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;

    gridCellsX = std::max(1, static_cast<int>(std::ceil(extentX / COLLISION_CELL_SIZE)));
    gridCellsY = std::max(1, static_cast<int>(std::ceil(extentY / COLLISION_CELL_SIZE)));

    // Cap grid size to avoid excessive memory for huge groups
    if (gridCellsX > 64) gridCellsX = 64;
    if (gridCellsY > 64) gridCellsY = 64;

    size_t totalCells = gridCellsX * gridCellsY;
    cellTriangles.resize(totalCells);
    cellFloorTriangles.resize(totalCells);
    cellWallTriangles.resize(totalCells);

    size_t numTriangles = collisionIndices.size() / 3;
    triBounds.resize(numTriangles);

    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    for (size_t i = 0; i + 2 < collisionIndices.size(); i += 3) {
        const glm::vec3& v0 = collisionVertices[collisionIndices[i]];
        const glm::vec3& v1 = collisionVertices[collisionIndices[i + 1]];
        const glm::vec3& v2 = collisionVertices[collisionIndices[i + 2]];

        // Triangle XY bounding box
        float triMinX = std::min({v0.x, v1.x, v2.x});
        float triMinY = std::min({v0.y, v1.y, v2.y});
        float triMaxX = std::max({v0.x, v1.x, v2.x});
        float triMaxY = std::max({v0.y, v1.y, v2.y});

        // Per-triangle Z bounds
        float triMinZ = std::min({v0.z, v1.z, v2.z});
        float triMaxZ = std::max({v0.z, v1.z, v2.z});
        triBounds[i / 3] = { triMinZ, triMaxZ };

        // Classify floor vs wall by normal.
        // Wall threshold matches MAX_WALK_SLOPE_DOT (cos 50° ≈ 0.6428) so that
        // surfaces too steep to walk on are always tested for wall collision.
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 normal = glm::cross(edge1, edge2);
        float normalLen = glm::length(normal);
        float absNz = (normalLen > 0.001f) ? std::abs(normal.z / normalLen) : 0.0f;
        bool isFloor = (absNz >= 0.35f);  // ~70° max slope (relaxed for steep stairs)
        bool isWall = (absNz < 0.65f);  // Matches walkable slope threshold

        int cellMinX = std::max(0, static_cast<int>((triMinX - gridOrigin.x) * invCellW));
        int cellMinY = std::max(0, static_cast<int>((triMinY - gridOrigin.y) * invCellH));
        int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((triMaxX - gridOrigin.x) * invCellW));
        int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((triMaxY - gridOrigin.y) * invCellH));

        uint32_t triIdx = static_cast<uint32_t>(i);
        for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
            for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
                int cellIdx = cy * gridCellsX + cx;
                cellTriangles[cellIdx].push_back(triIdx);
                if (isFloor) cellFloorTriangles[cellIdx].push_back(triIdx);
                if (isWall) cellWallTriangles[cellIdx].push_back(triIdx);
            }
        }
    }
}

const std::vector<uint32_t>* WMORenderer::GroupResources::getTrianglesAtLocal(float localX, float localY) const {
    if (gridCellsX == 0 || gridCellsY == 0) return nullptr;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cx = static_cast<int>((localX - gridOrigin.x) * invCellW);
    int cy = static_cast<int>((localY - gridOrigin.y) * invCellH);

    if (cx < 0 || cx >= gridCellsX || cy < 0 || cy >= gridCellsY) return nullptr;

    return &cellTriangles[cy * gridCellsX + cx];
}

void WMORenderer::GroupResources::getTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    // Collect unique triangle indices from all overlapping cells
    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    // Remove duplicates (triangles spanning multiple cells)
    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

void WMORenderer::GroupResources::getFloorTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0 || cellFloorTriangles.empty()) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellFloorTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

void WMORenderer::GroupResources::getWallTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0 || cellWallTriangles.empty()) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellWallTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

std::optional<float> WMORenderer::getFloorHeight(float glX, float glY, float glZ, float* outNormalZ) const {
    // All floor caching disabled - even per-frame cache can return stale results
    // when player Z changes between queries, causing fall-through at stairs.

    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;
    float bestNormalZ = 1.0f;
    bool bestFromLowPlatform = false;

    // World-space ray: from high above, pointing straight down
    glm::vec3 worldOrigin(glX, glY, glZ + 500.0f);
    glm::vec3 worldDir(0.0f, 0.0f, -1.0f);

    // Lambda to test a single group for floor hits
    auto testGroupFloor = [&](const WMOInstance& instance, const ModelData& model,
                              const GroupResources& group,
                              const glm::vec3& localOrigin, const glm::vec3& localDir) {
        const auto& verts = group.collisionVertices;
        const auto& indices = group.collisionIndices;

        // Use unfiltered triangle list: a vertical ray naturally misses vertical
        // geometry via ray-triangle intersection, so pre-filtering by normal is
        // unnecessary and risks excluding legitimate floor geometry (steep ramps,
        // stair treads with non-trivial normals).
        group.getTrianglesInRange(
            localOrigin.x - 1.0f, localOrigin.y - 1.0f,
            localOrigin.x + 1.0f, localOrigin.y + 1.0f,
            wallTriScratch);

        for (uint32_t triStart : wallTriScratch) {
            const glm::vec3& v0 = verts[indices[triStart]];
            const glm::vec3& v1 = verts[indices[triStart + 1]];
            const glm::vec3& v2 = verts[indices[triStart + 2]];

            float t = rayTriangleIntersect(localOrigin, localDir, v0, v1, v2);
            if (t <= 0.0f) {
                t = rayTriangleIntersect(localOrigin, localDir, v0, v2, v1);
            }

            if (t > 0.0f) {
                glm::vec3 hitLocal = localOrigin + localDir * t;
                glm::vec3 hitWorld = glm::vec3(instance.modelMatrix * glm::vec4(hitLocal, 1.0f));

                float allowAbove = model.isLowPlatform ? 12.0f : 2.0f;
                if (hitWorld.z <= glZ + allowAbove) {
                    if (!bestFloor || hitWorld.z > *bestFloor) {
                        bestFloor = hitWorld.z;
                        bestFromLowPlatform = model.isLowPlatform;

                        // Compute local normal and transform to world space
                        glm::vec3 localNormal = glm::cross(v1 - v0, v2 - v0);
                        float len = glm::length(localNormal);
                        if (len > 0.001f) {
                            localNormal /= len;
                            // Ensure normal points upward
                            if (localNormal.z < 0.0f) localNormal = -localNormal;
                            glm::vec3 worldNormal = glm::normalize(
                                glm::vec3(instance.modelMatrix * glm::vec4(localNormal, 0.0f)));
                            bestNormalZ = std::abs(worldNormal.z);
                        }
                    }
                }
            }
        }
    };

    // Fast path: current active interior group and its neighbors are usually
    // the right answer for player-floor queries while moving in cities/buildings.
    if (activeGroup_.isValid() && activeGroup_.instanceIdx < instances.size()) {
        const auto& instance = instances[activeGroup_.instanceIdx];
        auto it = loadedModels.find(instance.modelId);
        if (it != loadedModels.end() && instance.modelId == activeGroup_.modelId) {
            const ModelData& model = it->second;
            glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(worldOrigin, 1.0f));
            glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(worldDir, 0.0f)));

            auto testGroupIdx = [&](uint32_t gi) {
                if (gi >= model.groups.size()) return;
                if (gi < instance.worldGroupBounds.size()) {
                    const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                    if (glX < gMin.x || glX > gMax.x ||
                        glY < gMin.y || glY > gMax.y ||
                        glZ - 4.0f > gMax.z) {
                        return;
                    }
                }
                const auto& group = model.groups[gi];
                if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                    return;
                }
                testGroupFloor(instance, model, group, localOrigin, localDir);
            };

            if (activeGroup_.groupIdx >= 0) {
                testGroupIdx(static_cast<uint32_t>(activeGroup_.groupIdx));
            }
            for (uint32_t ngi : activeGroup_.neighborGroups) {
                testGroupIdx(ngi);
            }
        }
    }

    // Full scan: test all instances (active group fast path removed to fix
    // bridge clipping where early-return missed other WMO instances)
    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 8.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 10.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;
        float zMarginDown = model.isLowPlatform ? 20.0f : 2.0f;
        float zMarginUp = model.isLowPlatform ? 20.0f : 4.0f;

        // Broad-phase reject in world space to avoid expensive matrix transforms.
        if (bestFloor && instance.worldBoundsMax.z <= (*bestFloor + 0.05f)) {
            continue;
        }
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - zMarginDown || glZ > instance.worldBoundsMax.z + zMarginUp) {
            continue;
        }

        // World-space pre-pass: check which groups' world XY bounds contain
        // the query point. For a vertical ray this eliminates most groups
        // before any local-space math.
        bool anyGroupOverlaps = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ - 4.0f <= gMax.z) {
                anyGroupOverlaps = true;
                break;
            }
        }
        if (!anyGroupOverlaps) continue;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(worldOrigin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(worldDir, 0.0f)));

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull — vertical ray at (glX, glY)
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (glX < gMin.x || glX > gMax.x ||
                    glY < gMin.y || glY > gMax.y ||
                    glZ - 4.0f > gMax.z) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                continue;
            }

            testGroupFloor(instance, model, group, localOrigin, localDir);
        }
    }

    // Persistent grid cache disabled (see above comment about stairs fall-through)

    if (bestFloor && outNormalZ) {
        *outNormalZ = bestNormalZ;
    }

    return bestFloor;
}

bool WMORenderer::checkWallCollision(const glm::vec3& from, const glm::vec3& to, glm::vec3& adjustedPos, bool insideWMO) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool blocked = false;

    glm::vec3 moveDir = to - from;
    float moveDist = glm::length(moveDir);
    if (moveDist < 0.001f) return false;

    // Player collision parameters — WoW-style horizontal cylinder
    // Tighter radius when inside for more responsive indoor collision
    const float PLAYER_RADIUS = insideWMO ? 0.45f : 0.50f;
    const float PLAYER_HEIGHT = 2.0f;       // Cylinder height for Z bounds
    const float MAX_STEP_HEIGHT = 1.0f;     // Step-up threshold

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(8.0f, 8.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(8.0f, 8.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        const float broadMargin = PLAYER_RADIUS + 1.5f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && to.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && to.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && to.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && to.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + PLAYER_HEIGHT && to.z > instance.worldBoundsMax.z + PLAYER_HEIGHT) continue;
        if (from.z + PLAYER_HEIGHT < instance.worldBoundsMin.z && to.z + PLAYER_HEIGHT < instance.worldBoundsMin.z) continue;

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // World-space pre-pass: skip instances where no groups are near the movement
        const float wallMargin = PLAYER_RADIUS + 2.0f;
        bool anyGroupNear = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (to.x >= gMin.x - wallMargin && to.x <= gMax.x + wallMargin &&
                to.y >= gMin.y - wallMargin && to.y <= gMax.y + wallMargin &&
                to.z + PLAYER_HEIGHT >= gMin.z && to.z <= gMax.z + wallMargin) {
                anyGroupNear = true;
                break;
            }
        }
        if (!anyGroupNear) continue;

        // Transform positions into local space using cached inverse
        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localTo = glm::vec3(instance.invModelMatrix * glm::vec4(to, 1.0f));
        float localFeetZ = localTo.z;
        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (to.x < gMin.x - wallMargin || to.x > gMax.x + wallMargin ||
                    to.y < gMin.y - wallMargin || to.y > gMax.y + wallMargin ||
                    to.z > gMax.z + PLAYER_HEIGHT || to.z + PLAYER_HEIGHT < gMin.z) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            // Local-space AABB check
            float margin = PLAYER_RADIUS + 2.0f;
            if (localTo.x < group.boundingBoxMin.x - margin || localTo.x > group.boundingBoxMax.x + margin ||
                localTo.y < group.boundingBoxMin.y - margin || localTo.y > group.boundingBoxMax.y + margin ||
                localTo.z < group.boundingBoxMin.z - margin || localTo.z > group.boundingBoxMax.z + margin) {
                continue;
            }

            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            // Use spatial grid: query range covering the movement segment + player radius
            float rangeMinX = std::min(localFrom.x, localTo.x) - PLAYER_RADIUS - 1.5f;
            float rangeMinY = std::min(localFrom.y, localTo.y) - PLAYER_RADIUS - 1.5f;
            float rangeMaxX = std::max(localFrom.x, localTo.x) + PLAYER_RADIUS + 1.5f;
            float rangeMaxY = std::max(localFrom.y, localTo.y) + PLAYER_RADIUS + 1.5f;
            group.getWallTrianglesInRange(rangeMinX, rangeMinY, rangeMaxX, rangeMaxY, wallTriScratch);

            for (uint32_t triStart : wallTriScratch) {
                // Use pre-computed Z bounds for fast vertical reject
                const auto& tb = group.triBounds[triStart / 3];

                // Only collide with walls in player's vertical range
                if (tb.maxZ < localFeetZ + 0.3f) continue;
                if (tb.minZ > localFeetZ + PLAYER_HEIGHT) continue;

                // Skip low geometry that can be stepped over
                if (tb.maxZ <= localFeetZ + MAX_STEP_HEIGHT) continue;

                // Skip very short vertical surfaces (stair risers)
                float triHeight = tb.maxZ - tb.minZ;
                if (triHeight < 1.0f && tb.maxZ <= localFeetZ + 1.2f) continue;

                const glm::vec3& v0 = verts[indices[triStart]];
                const glm::vec3& v1 = verts[indices[triStart + 1]];
                const glm::vec3& v2 = verts[indices[triStart + 2]];

                // Triangle normal for swept test and push fallback
                glm::vec3 edge1 = v1 - v0;
                glm::vec3 edge2 = v2 - v0;
                glm::vec3 normal = glm::cross(edge1, edge2);
                float normalLen = glm::length(normal);
                if (normalLen < 0.001f) continue;
                normal /= normalLen;

                // Recompute plane distances with current (possibly pushed) localTo
                float fromDist = glm::dot(localFrom - v0, normal);
                float toDist = glm::dot(localTo - v0, normal);

                // Swept test: prevent tunneling when crossing a wall between frames
                if ((fromDist > PLAYER_RADIUS && toDist < -PLAYER_RADIUS) ||
                    (fromDist < -PLAYER_RADIUS && toDist > PLAYER_RADIUS)) {
                    float denom = (fromDist - toDist);
                    if (std::abs(denom) > 1e-6f) {
                        float tHit = fromDist / denom;
                        if (tHit >= 0.0f && tHit <= 1.0f) {
                            glm::vec3 hitPoint = localFrom + (localTo - localFrom) * tHit;
                            glm::vec3 hitClosest = closestPointOnTriangle(hitPoint, v0, v1, v2);
                            float hitErrSq = glm::dot(hitClosest - hitPoint, hitClosest - hitPoint);
                            if (hitErrSq <= 0.15f * 0.15f) {
                                float side = fromDist > 0.0f ? 1.0f : -1.0f;
                                glm::vec3 safeLocal = hitPoint + normal * side * (PLAYER_RADIUS + 0.05f);
                                glm::vec3 pushLocal(safeLocal.x - localTo.x, safeLocal.y - localTo.y, 0.0f);
                                // Cap swept pushback so walls don't shove the player violently
                                float pushLen = glm::length(glm::vec2(pushLocal.x, pushLocal.y));
                                const float MAX_SWEPT_PUSH = 0.15f;
                                if (pushLen > MAX_SWEPT_PUSH) {
                                    float scale = MAX_SWEPT_PUSH / pushLen;
                                    pushLocal.x *= scale;
                                    pushLocal.y *= scale;
                                }
                                localTo.x += pushLocal.x;
                                localTo.y += pushLocal.y;
                                glm::vec3 pushWorld = glm::vec3(instance.modelMatrix * glm::vec4(pushLocal, 0.0f));
                                adjustedPos.x += pushWorld.x;
                                adjustedPos.y += pushWorld.y;
                                blocked = true;
                                continue;
                            }
                        }
                    }
                }

                // Horizontal cylinder collision: closest point + horizontal distance
                glm::vec3 closest = closestPointOnTriangle(localTo, v0, v1, v2);
                glm::vec3 delta = localTo - closest;
                float horizDist = glm::length(glm::vec2(delta.x, delta.y));

                if (horizDist <= PLAYER_RADIUS) {
                    // Skip floor-like surfaces — grounding handles them, not wall collision
                    float absNz = std::abs(normal.z);
                    if (absNz >= 0.35f) continue;

                    const float SKIN = 0.005f;        // small separation so we don't re-collide immediately
                    // Stronger push when inside WMO for more responsive indoor collision
                    const float MAX_PUSH = insideWMO ? 0.12f : 0.08f;
                    float penetration = (PLAYER_RADIUS - horizDist);
                    float pushDist = glm::clamp(penetration + SKIN, 0.0f, MAX_PUSH);
                    glm::vec2 pushDir2;
                    if (horizDist > 1e-4f) {
                        pushDir2 = glm::normalize(glm::vec2(delta.x, delta.y));
                    } else {
                        glm::vec2 n2(normal.x, normal.y);
                        float n2Len = glm::length(n2);
                        if (n2Len < 1e-4f) continue;
                        pushDir2 = n2 / n2Len;
                    }
                    glm::vec3 pushLocal(pushDir2.x * pushDist, pushDir2.y * pushDist, 0.0f);

                    localTo.x += pushLocal.x;
                    localTo.y += pushLocal.y;
                    glm::vec3 pushWorld = glm::vec3(instance.modelMatrix * glm::vec4(pushLocal, 0.0f));
                    adjustedPos.x += pushWorld.x;
                    adjustedPos.y += pushWorld.y;
                    blocked = true;
                }
            }
        }
    }

    return blocked;
}

void WMORenderer::updateActiveGroup(float glX, float glY, float glZ) {
    // If active group is still valid, check if player is still inside it
    if (activeGroup_.isValid() && activeGroup_.instanceIdx < instances.size()) {
        const auto& instance = instances[activeGroup_.instanceIdx];
        if (instance.modelId == activeGroup_.modelId) {
            auto it = loadedModels.find(instance.modelId);
            if (it != loadedModels.end()) {
                const ModelData& model = it->second;
                glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

                // Still inside active group?
                if (activeGroup_.groupIdx >= 0 && static_cast<size_t>(activeGroup_.groupIdx) < model.groups.size()) {
                    const auto& group = model.groups[activeGroup_.groupIdx];
                    if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                        localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                        localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                        return;  // Still in same group
                    }
                }

                // Check portal-neighbor groups
                for (uint32_t ngi : activeGroup_.neighborGroups) {
                    if (ngi < model.groups.size()) {
                        const auto& group = model.groups[ngi];
                        if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                            localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                            localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                            // Moved to a neighbor group — update
                            activeGroup_.groupIdx = static_cast<int32_t>(ngi);
                            // Rebuild neighbors for new group
                            activeGroup_.neighborGroups.clear();
                            if (ngi < model.groupPortalRefs.size()) {
                                auto [portalStart, portalCount] = model.groupPortalRefs[ngi];
                                for (uint16_t pi = 0; pi < portalCount; pi++) {
                                    uint16_t refIdx = portalStart + pi;
                                    if (refIdx < model.portalRefs.size()) {
                                        uint32_t tgt = model.portalRefs[refIdx].groupIndex;
                                        if (tgt < model.groups.size()) {
                                            activeGroup_.neighborGroups.push_back(tgt);
                                        }
                                    }
                                }
                            }
                            return;
                        }
                    }
                }
            }
        }
    }

    // Full scan: find which instance/group contains the player
    activeGroup_.invalidate();

    glm::vec3 queryMin(glX - 0.5f, glY - 0.5f, glZ - 0.5f);
    glm::vec3 queryMax(glX + 0.5f, glY + 0.5f, glZ + 0.5f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z || glZ > instance.worldBoundsMax.z) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        int gi = findContainingGroup(model, localPos);
        if (gi >= 0) {
            activeGroup_.instanceIdx = static_cast<uint32_t>(idx);
            activeGroup_.modelId = instance.modelId;
            activeGroup_.groupIdx = gi;

            // Build neighbor list from portal refs
            activeGroup_.neighborGroups.clear();
            uint32_t groupIdx = static_cast<uint32_t>(gi);
            if (groupIdx < model.groupPortalRefs.size()) {
                auto [portalStart, portalCount] = model.groupPortalRefs[groupIdx];
                for (uint16_t pi = 0; pi < portalCount; pi++) {
                    uint16_t refIdx = portalStart + pi;
                    if (refIdx < model.portalRefs.size()) {
                        uint32_t tgt = model.portalRefs[refIdx].groupIndex;
                        if (tgt < model.groups.size()) {
                            activeGroup_.neighborGroups.push_back(tgt);
                        }
                    }
                }
            }
            return;
        }
    }
}

bool WMORenderer::isInsideWMO(float glX, float glY, float glZ, uint32_t* outModelId) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    glm::vec3 queryMin(glX - 0.5f, glY - 0.5f, glZ - 0.5f);
    glm::vec3 queryMax(glX + 0.5f, glY + 0.5f, glZ + 0.5f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z || glZ > instance.worldBoundsMax.z) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // World-space pre-check: skip instance if no group's world bounds contain point
        bool anyGroupContains = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ >= gMin.z && glZ <= gMax.z) {
                anyGroupContains = true;
                break;
            }
        }
        if (!anyGroupContains) continue;

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));
        for (const auto& group : model.groups) {
            if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                if (outModelId) *outModelId = instance.modelId;
                return true;
            }
        }
    }
    return false;
}

bool WMORenderer::isInsideInteriorWMO(float glX, float glY, float glZ) const {
    glm::vec3 queryMin(glX - 0.5f, glY - 0.5f, glZ - 0.5f);
    glm::vec3 queryMax(glX + 0.5f, glY + 0.5f, glZ + 0.5f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z || glZ > instance.worldBoundsMax.z) {
            continue;
        }
        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;
        const ModelData& model = it->second;

        bool anyGroupContains = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ >= gMin.z && glZ <= gMax.z) {
                anyGroupContains = true;
                break;
            }
        }
        if (!anyGroupContains) continue;

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));
        for (const auto& group : model.groups) {
            if (!(group.groupFlags & 0x2000)) continue; // Skip exterior groups
            if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                return true;
            }
        }
    }
    return false;
}

float WMORenderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;
    // Camera collision should primarily react to walls.
    // Wall list pre-filters at abs(normal.z) < 0.55, but for camera raycast we want
    // a stricter threshold to avoid ramp/stair geometry pulling the camera in.
    constexpr float MAX_WALKABLE_ABS_NORMAL_Z = 0.20f;
    constexpr float MAX_HIT_BELOW_ORIGIN = 0.90f;
    constexpr float MAX_HIT_ABOVE_ORIGIN = 0.80f;
    constexpr float MIN_SURFACE_ALIGNMENT = 0.25f;

    glm::vec3 rayEnd = origin + direction * maxDistance;
    glm::vec3 queryMin = glm::min(origin, rayEnd) - glm::vec3(1.0f);
    glm::vec3 queryMax = glm::max(origin, rayEnd) + glm::vec3(1.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        glm::vec3 center = (instance.worldBoundsMin + instance.worldBoundsMax) * 0.5f;
        float radius = glm::length(instance.worldBoundsMax - center);
        if (glm::length(center - origin) > (maxDistance + radius + 1.0f)) {
            continue;
        }

        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.5f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.5f);
        if (!rayIntersectsAABB(origin, direction, worldMin, worldMax)) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull — skip groups whose world AABB doesn't intersect the ray
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (!rayIntersectsAABB(origin, direction, gMin, gMax)) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            // Local-space AABB cull
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                continue;
            }

            // Narrow-phase: triangle raycast using spatial grid (wall-only).
            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            // Compute local-space ray endpoint and query grid for XY range
            glm::vec3 localEnd = localOrigin + localDir * (closestHit / glm::length(
                glm::vec3(instance.modelMatrix * glm::vec4(localDir, 0.0f))));
            float rMinX = std::min(localOrigin.x, localEnd.x) - 1.0f;
            float rMinY = std::min(localOrigin.y, localEnd.y) - 1.0f;
            float rMaxX = std::max(localOrigin.x, localEnd.x) + 1.0f;
            float rMaxY = std::max(localOrigin.y, localEnd.y) + 1.0f;
            group.getWallTrianglesInRange(rMinX, rMinY, rMaxX, rMaxY, wallTriScratch);

            for (uint32_t triStart : wallTriScratch) {
                const glm::vec3& v0 = verts[indices[triStart]];
                const glm::vec3& v1 = verts[indices[triStart + 1]];
                const glm::vec3& v2 = verts[indices[triStart + 2]];
                glm::vec3 triNormal = glm::cross(v1 - v0, v2 - v0);
                float normalLenSq = glm::dot(triNormal, triNormal);
                if (normalLenSq < 1e-8f) {
                    continue;
                }
                triNormal /= std::sqrt(normalLenSq);
                // Wall list pre-filters at 0.55; apply stricter camera threshold
                if (std::abs(triNormal.z) > MAX_WALKABLE_ABS_NORMAL_Z) {
                    continue;
                }
                // Ignore near-grazing intersections that tend to come from ramps/arches
                // and cause camera pull-in even when no meaningful wall is behind the player.
                if (std::abs(glm::dot(triNormal, localDir)) < MIN_SURFACE_ALIGNMENT) {
                    continue;
                }

                float t = rayTriangleIntersect(localOrigin, localDir, v0, v1, v2);
                if (t <= 0.0f) {
                    // Two-sided collision.
                    t = rayTriangleIntersect(localOrigin, localDir, v0, v2, v1);
                }
                if (t <= 0.0f) continue;

                glm::vec3 localHit = localOrigin + localDir * t;
                glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
                // Ignore low hits; camera floor handling already keeps the camera above ground.
                // This avoids gate/ramp floor geometry pulling the camera in too aggressively.
                if (worldHit.z < origin.z - MAX_HIT_BELOW_ORIGIN) {
                    continue;
                }
                // Ignore very high hits (arches/ceilings) that should not clamp normal chase-cam distance.
                if (worldHit.z > origin.z + MAX_HIT_ABOVE_ORIGIN) {
                    continue;
                }
                float worldDist = glm::length(worldHit - origin);
                if (worldDist > 0.0f && worldDist < closestHit && worldDist <= maxDistance) {
                    closestHit = worldDist;
                }
            }
        }
    }

    return closestHit;
}

// Occlusion queries stubbed out in Vulkan (were disabled by default anyway)

void WMORenderer::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old main-pass pipelines (NOT shadow, NOT pipeline layout)
    if (opaquePipeline_)      { vkDestroyPipeline(device, opaquePipeline_, nullptr); opaquePipeline_ = VK_NULL_HANDLE; }
    if (transparentPipeline_) { vkDestroyPipeline(device, transparentPipeline_, nullptr); transparentPipeline_ = VK_NULL_HANDLE; }
    if (glassPipeline_)       { vkDestroyPipeline(device, glassPipeline_, nullptr); glassPipeline_ = VK_NULL_HANDLE; }
    if (wireframePipeline_)   { vkDestroyPipeline(device, wireframePipeline_, nullptr); wireframePipeline_ = VK_NULL_HANDLE; }

    // --- Load shaders ---
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/wmo.vert.spv") ||
        !fragShader.loadFromFile(device, "assets/shaders/wmo.frag.spv")) {
        core::Logger::getInstance().error("WMORenderer::recreatePipelines: failed to load shaders");
        return;
    }

    // --- Vertex input ---
    struct WMOVertexData {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec4 color;
        glm::vec4 tangent;
    };

    VkVertexInputBindingDescription vertexBinding{};
    vertexBinding.binding = 0;
    vertexBinding.stride = sizeof(WMOVertexData);
    vertexBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> vertexAttribs(5);
    vertexAttribs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, position)) };
    vertexAttribs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, normal)) };
    vertexAttribs[2] = { 2, 0, VK_FORMAT_R32G32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, texCoord)) };
    vertexAttribs[3] = { 3, 0, VK_FORMAT_R32G32B32A32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, color)) };
    vertexAttribs[4] = { 4, 0, VK_FORMAT_R32G32B32A32_SFLOAT,
        static_cast<uint32_t>(offsetof(WMOVertexData, tangent)) };

    VkRenderPass mainPass = vkCtx_->getImGuiRenderPass();

    opaquePipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    transparentPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    glassPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    wireframePipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({ vertexBinding }, vertexAttribs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_LINE, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(mainPass)
        .setDynamicStates({ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR })
        .build(device);

    vertShader.destroy();
    fragShader.destroy();

    core::Logger::getInstance().info("WMORenderer: pipelines recreated");
}

} // namespace rendering
} // namespace wowee
