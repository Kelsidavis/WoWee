#include "rendering/m2_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <chrono>
#include <cctype>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <unordered_set>
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <future>
#include <thread>

namespace wowee {
namespace rendering {

namespace {

bool envFlagEnabled(const char* key, bool defaultValue) {
    const char* raw = std::getenv(key);
    if (!raw || !*raw) return defaultValue;
    std::string v(raw);
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return !(v == "0" || v == "false" || v == "off" || v == "no");
}

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

static constexpr uint32_t kParticleFlagRandomized = 0x40;
static constexpr uint32_t kParticleFlagTiled = 0x80;

float computeGroundDetailDownOffset(const M2ModelGPU& model, float scale) {
    // Keep a tiny sink to avoid hovering, but cap pivot compensation so details
    // don't get pushed below the terrain on models with large positive boundMin.
    const float pivotComp = glm::clamp(std::max(0.0f, model.boundMin.z * scale), 0.0f, 0.10f);
    const float terrainSink = 0.03f;
    return pivotComp + terrainSink;
}

void getTightCollisionBounds(const M2ModelGPU& model, glm::vec3& outMin, glm::vec3& outMax) {
    glm::vec3 center = (model.boundMin + model.boundMax) * 0.5f;
    glm::vec3 half = (model.boundMax - model.boundMin) * 0.5f;

    // Per-shape collision fitting:
    // - small solid props (boxes/crates/chests): tighter than full mesh, but
    //   larger than default to prevent walk-through on narrow objects
    // - default: tighter fit (avoid oversized blockers)
    // - stepped low platforms (tree curbs/planters): wider XY + lower Z
    if (model.collisionTreeTrunk) {
        // Tree trunk: proportional cylinder at the base of the tree.
        float modelHoriz = std::max(model.boundMax.x - model.boundMin.x,
                                    model.boundMax.y - model.boundMin.y);
        float trunkHalf = std::clamp(modelHoriz * 0.05f, 0.5f, 5.0f);
        half.x = trunkHalf;
        half.y = trunkHalf;
        // Height proportional to trunk width, capped at 3.5 units.
        half.z = std::min(trunkHalf * 2.5f, 3.5f);
        // Shift center down so collision is at the base (trunk), not mid-canopy.
        center.z = model.boundMin.z + half.z;
    } else if (model.collisionNarrowVerticalProp) {
        // Tall thin props (lamps/posts): keep passable gaps near walls.
        half.x *= 0.30f;
        half.y *= 0.30f;
        half.z *= 0.96f;
    } else if (model.collisionSmallSolidProp) {
        // Keep full tight mesh bounds for small solid props to avoid clip-through.
        half.x *= 1.00f;
        half.y *= 1.00f;
        half.z *= 1.00f;
    } else if (model.collisionSteppedLowPlatform) {
        half.x *= 0.98f;
        half.y *= 0.98f;
        half.z *= 0.52f;
    } else {
        half.x *= 0.66f;
        half.y *= 0.66f;
        half.z *= 0.76f;
    }

    outMin = center - half;
    outMax = center + half;
}

float getEffectiveCollisionTopLocal(const M2ModelGPU& model,
                                    const glm::vec3& localPos,
                                    const glm::vec3& localMin,
                                    const glm::vec3& localMax) {
    if (!model.collisionSteppedFountain && !model.collisionSteppedLowPlatform) {
        return localMax.z;
    }

    glm::vec2 center((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
    glm::vec2 half((localMax.x - localMin.x) * 0.5f, (localMax.y - localMin.y) * 0.5f);
    if (half.x < 1e-4f || half.y < 1e-4f) {
        return localMax.z;
    }

    float nx = (localPos.x - center.x) / half.x;
    float ny = (localPos.y - center.y) / half.y;
    float r = std::sqrt(nx * nx + ny * ny);

    float h = localMax.z - localMin.z;
    if (model.collisionSteppedFountain) {
        if (r > 0.85f) return localMin.z + h * 0.18f;  // outer lip
        if (r > 0.65f) return localMin.z + h * 0.36f;  // mid step
        if (r > 0.45f) return localMin.z + h * 0.54f;  // inner step
        if (r > 0.28f) return localMin.z + h * 0.70f;  // center platform / statue base
        if (r > 0.14f) return localMin.z + h * 0.84f;  // statue body / sword
        return localMin.z + h * 0.96f;                  // statue head / top
    }

    // Low square curb/planter profile:
    // use edge distance (not radial) so corner blocks don't become too low and
    // clip-through at diagonals.
    float edge = std::max(std::abs(nx), std::abs(ny));
    if (edge > 0.92f) return localMin.z + h * 0.06f;
    if (edge > 0.72f) return localMin.z + h * 0.30f;
    return localMin.z + h * 0.62f;
}

bool segmentIntersectsAABB(const glm::vec3& from, const glm::vec3& to,
                           const glm::vec3& bmin, const glm::vec3& bmax,
                           float& outEnterT) {
    glm::vec3 d = to - from;
    float tEnter = 0.0f;
    float tExit = 1.0f;

    for (int axis = 0; axis < 3; axis++) {
        if (std::abs(d[axis]) < 1e-6f) {
            if (from[axis] < bmin[axis] || from[axis] > bmax[axis]) {
                return false;
            }
            continue;
        }

        float inv = 1.0f / d[axis];
        float t0 = (bmin[axis] - from[axis]) * inv;
        float t1 = (bmax[axis] - from[axis]) * inv;
        if (t0 > t1) std::swap(t0, t1);

        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) return false;
    }

    outEnterT = tEnter;
    return tExit >= 0.0f && tEnter <= 1.0f;
}

void transformAABB(const glm::mat4& modelMatrix,
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
    for (const auto& c : corners) {
        glm::vec3 wc = glm::vec3(modelMatrix * glm::vec4(c, 1.0f));
        outMin = glm::min(outMin, wc);
        outMax = glm::max(outMax, wc);
    }
}

float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax) {
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

// Möller–Trumbore ray-triangle intersection.
// Returns distance along ray if hit, negative if miss.
float rayTriangleIntersect(const glm::vec3& origin, const glm::vec3& dir,
                           const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    constexpr float EPSILON = 1e-6f;
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

// Closest point on triangle to a point (Ericson, Real-Time Collision Detection §5.1.5).
glm::vec3 closestPointOnTriangle(const glm::vec3& p,
                                  const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }
    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
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

} // namespace

void M2Instance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Rotation in radians
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));
    invModelMatrix = glm::inverse(modelMatrix);
}

M2Renderer::M2Renderer() {
}

M2Renderer::~M2Renderer() {
    shutdown();
}

bool M2Renderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                            pipeline::AssetManager* assets) {
    if (initialized_) { assetManager = assets; return true; }
    vkCtx_ = ctx;
    assetManager = assets;

    const unsigned hc = std::thread::hardware_concurrency();
    const size_t availableCores = (hc > 1u) ? static_cast<size_t>(hc - 1u) : 1ull;
    // Keep headroom for other frame tasks: M2 gets about half of non-main cores by default.
    const size_t defaultAnimThreads = std::max<size_t>(1, availableCores / 2);
    numAnimThreads_ = static_cast<uint32_t>(std::max<size_t>(
        1, envSizeOrDefault("WOWEE_M2_ANIM_THREADS", defaultAnimThreads)));
    LOG_INFO("Initializing M2 renderer (Vulkan, ", numAnimThreads_, " anim threads)...");

    VkDevice device = vkCtx_->getDevice();

    // --- Descriptor set layouts ---

    // Material set layout (set 1): binding 0 = sampler2D, binding 2 = M2Material UBO
    // (M2Params moved to push constants alongside model matrix)
    {
        VkDescriptorSetLayoutBinding bindings[2] = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[1].binding = 2;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 2;
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

    // Particle texture set layout (set 1 for particles): binding 0 = sampler2D
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 1;
        ci.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &particleTexLayout_);
    }

    // --- Descriptor pools ---
    {
        VkDescriptorPoolSize sizes[] = {
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_MATERIAL_SETS + 256},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_MATERIAL_SETS + 256},
        };
        VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets = MAX_MATERIAL_SETS + 256;
        ci.poolSizeCount = 2;
        ci.pPoolSizes = sizes;
        ci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        vkCreateDescriptorPool(device, &ci, nullptr, &materialDescPool_);
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

    // --- Pipeline layouts ---

    // Main M2 pipeline layout: set 0 = perFrame, set 1 = material, set 2 = bones
    // Push constant: mat4 model + vec2 uvOffset + int texCoordSet + int useBones = 80 bytes
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout, materialSetLayout_, boneSetLayout_};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = 84; // mat4(64) + vec2(8) + int(4) + int(4) + int(4)

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 3;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &pipelineLayout_);
    }

    // Particle pipeline layout: set 0 = perFrame, set 1 = particleTex
    // Push constant: vec2 tileCount + int alphaKey (12 bytes)
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout, particleTexLayout_};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pushRange.offset = 0;
        pushRange.size = 12; // vec2 + int

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 2;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &particlePipelineLayout_);
    }

    // Smoke pipeline layout: set 0 = perFrame
    // Push constant: float screenHeight (4 bytes)
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = 4;

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 1;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &smokePipelineLayout_);
    }

    // --- Load shaders ---
    rendering::VkShaderModule m2Vert, m2Frag;
    rendering::VkShaderModule particleVert, particleFrag;
    rendering::VkShaderModule smokeVert, smokeFrag;

    m2Vert.loadFromFile(device, "assets/shaders/m2.vert.spv");
    m2Frag.loadFromFile(device, "assets/shaders/m2.frag.spv");
    particleVert.loadFromFile(device, "assets/shaders/m2_particle.vert.spv");
    particleFrag.loadFromFile(device, "assets/shaders/m2_particle.frag.spv");
    smokeVert.loadFromFile(device, "assets/shaders/m2_smoke.vert.spv");
    smokeFrag.loadFromFile(device, "assets/shaders/m2_smoke.frag.spv");

    if (!m2Vert.isValid() || !m2Frag.isValid()) {
        LOG_ERROR("M2: Missing required shaders, cannot initialize");
        return false;
    }

    VkRenderPass mainPass = vkCtx_->getImGuiRenderPass();

    // --- Build M2 model pipelines ---
    // Vertex input: 18 floats = 72 bytes stride
    // loc 0: vec3 pos (0), loc 1: vec3 normal (12), loc 2: vec2 uv0 (24),
    // loc 5: vec2 uv1 (32), loc 3: vec4 boneWeights (40), loc 4: vec4 boneIndices (56)
    VkVertexInputBindingDescription m2Binding{};
    m2Binding.binding = 0;
    m2Binding.stride = 18 * sizeof(float);
    m2Binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> m2Attrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                     // position
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 3 * sizeof(float)},     // normal
        {2, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float)},        // texCoord0
        {5, 0, VK_FORMAT_R32G32_SFLOAT, 8 * sizeof(float)},        // texCoord1
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 10 * sizeof(float)}, // boneWeights
        {4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 14 * sizeof(float)}, // boneIndices (float)
    };

    auto buildM2Pipeline = [&](VkPipelineColorBlendAttachmentState blendState, bool depthWrite) -> VkPipeline {
        return PipelineBuilder()
            .setShaders(m2Vert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        m2Frag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({m2Binding}, m2Attrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, depthWrite, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(blendState)
            .setMultisample(vkCtx_->getMsaaSamples())
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    };

    opaquePipeline_ = buildM2Pipeline(PipelineBuilder::blendDisabled(), true);
    alphaTestPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), true);
    alphaPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), false);
    additivePipeline_ = buildM2Pipeline(PipelineBuilder::blendAdditive(), false);

    // --- Build particle pipelines ---
    if (particleVert.isValid() && particleFrag.isValid()) {
        VkVertexInputBindingDescription pBind{};
        pBind.binding = 0;
        pBind.stride = 9 * sizeof(float); // pos3 + color4 + size1 + tile1
        pBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> pAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                    // position
            {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 3 * sizeof(float)}, // color
            {2, 0, VK_FORMAT_R32_SFLOAT, 7 * sizeof(float)},          // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 8 * sizeof(float)},          // tile
        };

        auto buildParticlePipeline = [&](VkPipelineColorBlendAttachmentState blend) -> VkPipeline {
            return PipelineBuilder()
                .setShaders(particleVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                            particleFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
                .setVertexInput({pBind}, pAttrs)
                .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
                .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
                .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
                .setColorBlendAttachment(blend)
                .setMultisample(vkCtx_->getMsaaSamples())
                .setLayout(particlePipelineLayout_)
                .setRenderPass(mainPass)
                .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
                .build(device);
        };

        particlePipeline_ = buildParticlePipeline(PipelineBuilder::blendAlpha());
        particleAdditivePipeline_ = buildParticlePipeline(PipelineBuilder::blendAdditive());
    }

    // --- Build smoke pipeline ---
    if (smokeVert.isValid() && smokeFrag.isValid()) {
        VkVertexInputBindingDescription sBind{};
        sBind.binding = 0;
        sBind.stride = 6 * sizeof(float); // pos3 + lifeRatio1 + size1 + isSpark1
        sBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> sAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},           // position
            {1, 0, VK_FORMAT_R32_SFLOAT, 3 * sizeof(float)}, // lifeRatio
            {2, 0, VK_FORMAT_R32_SFLOAT, 4 * sizeof(float)}, // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 5 * sizeof(float)}, // isSpark
        };

        smokePipeline_ = PipelineBuilder()
            .setShaders(smokeVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        smokeFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({sBind}, sAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(PipelineBuilder::blendAlpha())
            .setMultisample(vkCtx_->getMsaaSamples())
            .setLayout(smokePipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    }

    // Clean up shader modules
    m2Vert.destroy(); m2Frag.destroy();
    particleVert.destroy(); particleFrag.destroy();
    smokeVert.destroy(); smokeFrag.destroy();

    // --- Create dynamic particle buffers (mapped for CPU writes) ---
    {
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo allocInfo{};

        // Smoke particle buffer
        bci.size = MAX_SMOKE_PARTICLES * 6 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &smokeVB_, &smokeVBAlloc_, &allocInfo);
        smokeVBMapped_ = allocInfo.pMappedData;

        // M2 particle buffer
        bci.size = MAX_M2_PARTICLES * 9 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &m2ParticleVB_, &m2ParticleVBAlloc_, &allocInfo);
        m2ParticleVBMapped_ = allocInfo.pMappedData;
    }

    // --- Create white fallback texture ---
    {
        uint8_t white[] = {255, 255, 255, 255};
        whiteTexture_ = std::make_unique<VkTexture>();
        whiteTexture_->upload(*vkCtx_, white, 1, 1, VK_FORMAT_R8G8B8A8_UNORM);
        whiteTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }

    // --- Generate soft radial gradient glow texture ---
    {
        static constexpr int SZ = 64;
        std::vector<uint8_t> px(SZ * SZ * 4);
        float half = SZ / 2.0f;
        for (int y = 0; y < SZ; y++) {
            for (int x = 0; x < SZ; x++) {
                float dx = (x + 0.5f - half) / half;
                float dy = (y + 0.5f - half) / half;
                float r = std::sqrt(dx * dx + dy * dy);
                float a = std::max(0.0f, 1.0f - r);
                a = a * a; // Quadratic falloff
                int idx = (y * SZ + x) * 4;
                px[idx + 0] = 255;
                px[idx + 1] = 255;
                px[idx + 2] = 255;
                px[idx + 3] = static_cast<uint8_t>(a * 255);
            }
        }
        glowTexture_ = std::make_unique<VkTexture>();
        glowTexture_->upload(*vkCtx_, px.data(), SZ, SZ, VK_FORMAT_R8G8B8A8_UNORM);
        glowTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
        // Pre-allocate glow texture descriptor set (reused every frame)
        if (particleTexLayout_ && materialDescPool_) {
            VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            ai.descriptorPool = materialDescPool_;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts = &particleTexLayout_;
            if (vkAllocateDescriptorSets(device, &ai, &glowTexDescSet_) == VK_SUCCESS) {
                VkDescriptorImageInfo imgInfo = glowTexture_->descriptorInfo();
                VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.dstSet = glowTexDescSet_;
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                write.pImageInfo = &imgInfo;
                vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
            }
        }
    }
    textureCacheBudgetBytes_ =
        envSizeMBOrDefault("WOWEE_M2_TEX_CACHE_MB", 4096) * 1024ull * 1024ull;
    modelCacheLimit_ = envSizeMBOrDefault("WOWEE_M2_MODEL_LIMIT", 6000);
    LOG_INFO("M2 texture cache budget: ", textureCacheBudgetBytes_ / (1024 * 1024), " MB");
    LOG_INFO("M2 model cache limit: ", modelCacheLimit_);

    LOG_INFO("M2 renderer initialized (Vulkan)");
    initialized_ = true;
    return true;
}

void M2Renderer::shutdown() {
    LOG_INFO("Shutting down M2 renderer...");
    if (!vkCtx_) return;

    vkDeviceWaitIdle(vkCtx_->getDevice());
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();

    // Delete model GPU resources
    for (auto& [id, model] : models) {
        destroyModelGPU(model);
    }
    models.clear();

    // Destroy instance bone buffers
    for (auto& inst : instances) {
        destroyInstanceBones(inst);
    }
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();

    // Delete cached textures
    textureCache.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;
    textureHasAlphaByPtr_.clear();
    textureColorKeyBlackByPtr_.clear();
    failedTextureCache_.clear();
    loggedTextureLoadFails_.clear();
    textureBudgetRejectWarnings_ = 0;
    whiteTexture_.reset();
    glowTexture_.reset();

    // Clean up particle buffers
    if (smokeVB_) { vmaDestroyBuffer(alloc, smokeVB_, smokeVBAlloc_); smokeVB_ = VK_NULL_HANDLE; }
    if (m2ParticleVB_) { vmaDestroyBuffer(alloc, m2ParticleVB_, m2ParticleVBAlloc_); m2ParticleVB_ = VK_NULL_HANDLE; }
    smokeParticles.clear();

    // Destroy pipelines
    auto destroyPipeline = [&](VkPipeline& p) { if (p) { vkDestroyPipeline(device, p, nullptr); p = VK_NULL_HANDLE; } };
    destroyPipeline(opaquePipeline_);
    destroyPipeline(alphaTestPipeline_);
    destroyPipeline(alphaPipeline_);
    destroyPipeline(additivePipeline_);
    destroyPipeline(particlePipeline_);
    destroyPipeline(particleAdditivePipeline_);
    destroyPipeline(smokePipeline_);

    if (pipelineLayout_) { vkDestroyPipelineLayout(device, pipelineLayout_, nullptr); pipelineLayout_ = VK_NULL_HANDLE; }
    if (particlePipelineLayout_) { vkDestroyPipelineLayout(device, particlePipelineLayout_, nullptr); particlePipelineLayout_ = VK_NULL_HANDLE; }
    if (smokePipelineLayout_) { vkDestroyPipelineLayout(device, smokePipelineLayout_, nullptr); smokePipelineLayout_ = VK_NULL_HANDLE; }

    // Destroy descriptor pools and layouts
    if (materialDescPool_) { vkDestroyDescriptorPool(device, materialDescPool_, nullptr); materialDescPool_ = VK_NULL_HANDLE; }
    if (boneDescPool_) { vkDestroyDescriptorPool(device, boneDescPool_, nullptr); boneDescPool_ = VK_NULL_HANDLE; }
    if (materialSetLayout_) { vkDestroyDescriptorSetLayout(device, materialSetLayout_, nullptr); materialSetLayout_ = VK_NULL_HANDLE; }
    if (boneSetLayout_) { vkDestroyDescriptorSetLayout(device, boneSetLayout_, nullptr); boneSetLayout_ = VK_NULL_HANDLE; }
    if (particleTexLayout_) { vkDestroyDescriptorSetLayout(device, particleTexLayout_, nullptr); particleTexLayout_ = VK_NULL_HANDLE; }

    // Destroy shadow resources
    destroyPipeline(shadowPipeline_);
    if (shadowPipelineLayout_) { vkDestroyPipelineLayout(device, shadowPipelineLayout_, nullptr); shadowPipelineLayout_ = VK_NULL_HANDLE; }
    if (shadowTexPool_) { vkDestroyDescriptorPool(device, shadowTexPool_, nullptr); shadowTexPool_ = VK_NULL_HANDLE; }
    if (shadowParamsPool_) { vkDestroyDescriptorPool(device, shadowParamsPool_, nullptr); shadowParamsPool_ = VK_NULL_HANDLE; }
    if (shadowParamsLayout_) { vkDestroyDescriptorSetLayout(device, shadowParamsLayout_, nullptr); shadowParamsLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsUBO_) { vmaDestroyBuffer(alloc, shadowParamsUBO_, shadowParamsAlloc_); shadowParamsUBO_ = VK_NULL_HANDLE; }

    initialized_ = false;
}

void M2Renderer::destroyModelGPU(M2ModelGPU& model) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();
    if (model.vertexBuffer) { vmaDestroyBuffer(alloc, model.vertexBuffer, model.vertexAlloc); model.vertexBuffer = VK_NULL_HANDLE; }
    if (model.indexBuffer) { vmaDestroyBuffer(alloc, model.indexBuffer, model.indexAlloc); model.indexBuffer = VK_NULL_HANDLE; }
    for (auto& batch : model.batches) {
        if (batch.materialUBO) { vmaDestroyBuffer(alloc, batch.materialUBO, batch.materialUBOAlloc); batch.materialUBO = VK_NULL_HANDLE; }
        // materialSet freed when pool is reset/destroyed
    }
}

void M2Renderer::destroyInstanceBones(M2Instance& inst) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();
    for (int i = 0; i < 2; i++) {
        if (inst.boneBuffer[i]) {
            vmaDestroyBuffer(alloc, inst.boneBuffer[i], inst.boneAlloc[i]);
            inst.boneBuffer[i] = VK_NULL_HANDLE;
            inst.boneMapped[i] = nullptr;
        }
        // boneSet freed when pool is reset/destroyed
    }
}

VkDescriptorSet M2Renderer::allocateMaterialSet() {
    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = materialDescPool_;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &materialSetLayout_;
    VkDescriptorSet set = VK_NULL_HANDLE;
    vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set);
    return set;
}

VkDescriptorSet M2Renderer::allocateBoneSet() {
    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = boneDescPool_;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &boneSetLayout_;
    VkDescriptorSet set = VK_NULL_HANDLE;
    vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set);
    return set;
}

// ---------------------------------------------------------------------------
// M2 collision mesh: build spatial grid + classify triangles
// ---------------------------------------------------------------------------
void M2ModelGPU::CollisionMesh::build() {
    if (indices.size() < 3 || vertices.empty()) return;
    triCount = static_cast<uint32_t>(indices.size() / 3);

    // Bounding box for grid
    glm::vec3 bmin(std::numeric_limits<float>::max());
    glm::vec3 bmax(-std::numeric_limits<float>::max());
    for (const auto& v : vertices) {
        bmin = glm::min(bmin, v);
        bmax = glm::max(bmax, v);
    }

    gridOrigin = glm::vec2(bmin.x, bmin.y);
    gridCellsX = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.x - bmin.x) / CELL_SIZE))));
    gridCellsY = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.y - bmin.y) / CELL_SIZE))));

    cellFloorTris.resize(gridCellsX * gridCellsY);
    cellWallTris.resize(gridCellsX * gridCellsY);
    triBounds.resize(triCount);

    for (uint32_t ti = 0; ti < triCount; ti++) {
        uint16_t i0 = indices[ti * 3];
        uint16_t i1 = indices[ti * 3 + 1];
        uint16_t i2 = indices[ti * 3 + 2];
        if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) continue;

        const auto& v0 = vertices[i0];
        const auto& v1 = vertices[i1];
        const auto& v2 = vertices[i2];

        triBounds[ti].minZ = std::min({v0.z, v1.z, v2.z});
        triBounds[ti].maxZ = std::max({v0.z, v1.z, v2.z});

        glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
        float normalLen = glm::length(normal);
        float absNz = (normalLen > 0.001f) ? std::abs(normal.z / normalLen) : 0.0f;
        bool isFloor = (absNz >= 0.35f);  // ~70° max slope (relaxed for steep stairs)
        bool isWall  = (absNz < 0.65f);

        float triMinX = std::min({v0.x, v1.x, v2.x});
        float triMaxX = std::max({v0.x, v1.x, v2.x});
        float triMinY = std::min({v0.y, v1.y, v2.y});
        float triMaxY = std::max({v0.y, v1.y, v2.y});

        int cxMin = std::clamp(static_cast<int>((triMinX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cxMax = std::clamp(static_cast<int>((triMaxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cyMin = std::clamp(static_cast<int>((triMinY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
        int cyMax = std::clamp(static_cast<int>((triMaxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);

        for (int cy = cyMin; cy <= cyMax; cy++) {
            for (int cx = cxMin; cx <= cxMax; cx++) {
                int ci = cy * gridCellsX + cx;
                if (isFloor) cellFloorTris[ci].push_back(ti);
                if (isWall)  cellWallTris[ci].push_back(ti);
            }
        }
    }
}

void M2ModelGPU::CollisionMesh::getFloorTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellFloorTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void M2ModelGPU::CollisionMesh::getWallTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellWallTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

bool M2Renderer::hasModel(uint32_t modelId) const {
    return models.find(modelId) != models.end();
}

bool M2Renderer::loadModel(const pipeline::M2Model& model, uint32_t modelId) {
    if (models.find(modelId) != models.end()) {
        // Already loaded
        return true;
    }
    if (models.size() >= modelCacheLimit_) {
        if (modelLimitRejectWarnings_ < 3) {
            LOG_WARNING("M2 model cache full (", models.size(), "/", modelCacheLimit_,
                        "), skipping model load: id=", modelId, " name=", model.name);
        }
        ++modelLimitRejectWarnings_;
        return false;
    }

    bool hasGeometry = !model.vertices.empty() && !model.indices.empty();
    bool hasParticles = !model.particleEmitters.empty();
    if (!hasGeometry && !hasParticles) {
        LOG_WARNING("M2 model has no geometry and no particles: ", model.name);
        return false;
    }

    M2ModelGPU gpuModel;
    gpuModel.name = model.name;

    // Detect invisible trap models (event objects that should not render or collide)
    std::string lowerName = model.name;
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool isInvisibleTrap = (lowerName.find("invisibletrap") != std::string::npos);
    gpuModel.isInvisibleTrap = isInvisibleTrap;
    if (isInvisibleTrap) {
        LOG_INFO("Loading InvisibleTrap model: ", model.name, " (will be invisible, no collision)");
    }
    // Use tight bounds from actual vertices for collision/camera occlusion.
    // Header bounds in some M2s are overly conservative.
    glm::vec3 tightMin(0.0f);
    glm::vec3 tightMax(0.0f);
    if (hasGeometry) {
        tightMin = glm::vec3(std::numeric_limits<float>::max());
        tightMax = glm::vec3(-std::numeric_limits<float>::max());
        for (const auto& v : model.vertices) {
            tightMin = glm::min(tightMin, v.position);
            tightMax = glm::max(tightMax, v.position);
        }
    }
    bool foliageOrTreeLike = false;
    bool chestName = false;
    bool groundDetailModel = false;
    {
        std::string lowerName = model.name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.collisionSteppedFountain = (lowerName.find("fountain") != std::string::npos);

        glm::vec3 dims = tightMax - tightMin;
        float horiz = std::max(dims.x, dims.y);
        float vert = std::max(0.0f, dims.z);
        bool lowWideShape = (horiz > 1.4f && vert > 0.2f && vert < horiz * 0.70f);
        bool likelyCurbName =
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos);
        bool knownStormwindPlanter =
            (lowerName.find("stormwindplanter") != std::string::npos) ||
            (lowerName.find("stormwindwindowplanter") != std::string::npos);
        bool lowPlatformShape = (horiz > 1.8f && vert > 0.2f && vert < 1.8f);
        bool bridgeName =
            (lowerName.find("bridge") != std::string::npos) ||
            (lowerName.find("plank") != std::string::npos) ||
            (lowerName.find("walkway") != std::string::npos);
        gpuModel.collisionSteppedLowPlatform = (!gpuModel.collisionSteppedFountain) &&
                                               (knownStormwindPlanter ||
                                                bridgeName ||
                                                (likelyCurbName && (lowPlatformShape || lowWideShape)));
        gpuModel.collisionBridge = bridgeName;

        bool isPlanter = (lowerName.find("planter") != std::string::npos);
        gpuModel.collisionPlanter = isPlanter;
        bool statueName =
            (lowerName.find("statue") != std::string::npos) ||
            (lowerName.find("monument") != std::string::npos) ||
            (lowerName.find("sculpture") != std::string::npos);
        gpuModel.collisionStatue = statueName;
        bool smallSolidPropName =
            statueName ||
            (lowerName.find("crate") != std::string::npos) ||
            (lowerName.find("box") != std::string::npos) ||
            (lowerName.find("chest") != std::string::npos) ||
            (lowerName.find("barrel") != std::string::npos);
        chestName = (lowerName.find("chest") != std::string::npos);
        bool foliageName =
            (lowerName.find("bush") != std::string::npos) ||
            (lowerName.find("grass") != std::string::npos) ||
            (lowerName.find("drygrass") != std::string::npos) ||
            (lowerName.find("dry_grass") != std::string::npos) ||
            (lowerName.find("dry-grass") != std::string::npos) ||
            (lowerName.find("deadgrass") != std::string::npos) ||
            (lowerName.find("dead_grass") != std::string::npos) ||
            (lowerName.find("dead-grass") != std::string::npos) ||
            ((lowerName.find("plant") != std::string::npos) && !isPlanter) ||
            (lowerName.find("flower") != std::string::npos) ||
            (lowerName.find("shrub") != std::string::npos) ||
            (lowerName.find("fern") != std::string::npos) ||
            (lowerName.find("vine") != std::string::npos) ||
            (lowerName.find("lily") != std::string::npos) ||
            (lowerName.find("weed") != std::string::npos) ||
            (lowerName.find("wheat") != std::string::npos) ||
            (lowerName.find("pumpkin") != std::string::npos) ||
            (lowerName.find("firefly") != std::string::npos) ||
            (lowerName.find("fireflies") != std::string::npos) ||
            (lowerName.find("fireflys") != std::string::npos) ||
            (lowerName.find("mushroom") != std::string::npos) ||
            (lowerName.find("fungus") != std::string::npos) ||
            (lowerName.find("toadstool") != std::string::npos) ||
            (lowerName.find("root") != std::string::npos) ||
            (lowerName.find("branch") != std::string::npos) ||
            (lowerName.find("thorn") != std::string::npos) ||
            (lowerName.find("moss") != std::string::npos) ||
            (lowerName.find("ivy") != std::string::npos) ||
            (lowerName.find("seaweed") != std::string::npos) ||
            (lowerName.find("kelp") != std::string::npos) ||
            (lowerName.find("cattail") != std::string::npos) ||
            (lowerName.find("reed") != std::string::npos) ||
            (lowerName.find("palm") != std::string::npos) ||
            (lowerName.find("bamboo") != std::string::npos) ||
            (lowerName.find("banana") != std::string::npos) ||
            (lowerName.find("coconut") != std::string::npos) ||
            (lowerName.find("canopy") != std::string::npos) ||
            (lowerName.find("hedge") != std::string::npos) ||
            (lowerName.find("cactus") != std::string::npos) ||
            (lowerName.find("leaf") != std::string::npos) ||
            (lowerName.find("leaves") != std::string::npos) ||
            (lowerName.find("stalk") != std::string::npos) ||
            (lowerName.find("corn") != std::string::npos) ||
            (lowerName.find("crop") != std::string::npos) ||
            (lowerName.find("hay") != std::string::npos) ||
            (lowerName.find("frond") != std::string::npos) ||
            (lowerName.find("algae") != std::string::npos) ||
            (lowerName.find("coral") != std::string::npos);
        bool treeLike = (lowerName.find("tree") != std::string::npos);
        foliageOrTreeLike = (foliageName || treeLike);
        groundDetailModel =
            (lowerName.find("\\nodxt\\detail\\") != std::string::npos) ||
            (lowerName.find("\\detail\\") != std::string::npos);
        bool hardTreePart =
            (lowerName.find("trunk") != std::string::npos) ||
            (lowerName.find("stump") != std::string::npos) ||
            (lowerName.find("log") != std::string::npos);
        // Only large trees (canopy > 20 model units wide) get trunk collision.
        // Small/mid trees are walkthrough to avoid getting stuck between them.
        // Only large trees get trunk collision; all smaller trees are walkthrough.
        bool treeWithTrunk = treeLike && !hardTreePart && !foliageName && horiz > 40.0f;
        bool softTree = treeLike && !hardTreePart && !treeWithTrunk;
        bool forceSolidCurb = gpuModel.collisionSteppedLowPlatform || knownStormwindPlanter || likelyCurbName || gpuModel.collisionPlanter;
        bool narrowVerticalName =
            (lowerName.find("lamp") != std::string::npos) ||
            (lowerName.find("lantern") != std::string::npos) ||
            (lowerName.find("post") != std::string::npos) ||
            (lowerName.find("pole") != std::string::npos);
        bool narrowVerticalShape =
            (horiz > 0.12f && horiz < 2.0f && vert > 2.2f && vert > horiz * 1.8f);
        gpuModel.collisionTreeTrunk = treeWithTrunk;
        gpuModel.collisionNarrowVerticalProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            (narrowVerticalName || narrowVerticalShape);
        bool genericSolidPropShape =
            (horiz > 0.6f && horiz < 6.0f && vert > 0.30f && vert < 4.0f && vert > horiz * 0.16f) ||
            statueName;
        bool curbLikeName =
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos);
        bool lowPlatformLikeShape = lowWideShape || lowPlatformShape;
        bool carpetOrRug =
            (lowerName.find("carpet") != std::string::npos) ||
            (lowerName.find("rug") != std::string::npos);
        gpuModel.collisionSmallSolidProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            !gpuModel.collisionNarrowVerticalProp &&
            !gpuModel.collisionTreeTrunk &&
            !curbLikeName &&
            !lowPlatformLikeShape &&
            (smallSolidPropName || (genericSolidPropShape && !foliageName && !softTree));
        // Disable collision for foliage, soft trees, and decorative carpets/rugs
        gpuModel.collisionNoBlock = ((foliageName || softTree || carpetOrRug) &&
                                     !forceSolidCurb);
    }
    gpuModel.boundMin = tightMin;
    gpuModel.boundMax = tightMax;
    gpuModel.boundRadius = model.boundRadius;
    gpuModel.indexCount = static_cast<uint32_t>(model.indices.size());
    gpuModel.vertexCount = static_cast<uint32_t>(model.vertices.size());

    // Store bone/sequence data for animation
    gpuModel.bones = model.bones;
    gpuModel.sequences = model.sequences;
    gpuModel.globalSequenceDurations = model.globalSequenceDurations;
    gpuModel.hasAnimation = false;
    for (const auto& bone : model.bones) {
        if (bone.translation.hasData() || bone.rotation.hasData() || bone.scale.hasData()) {
            gpuModel.hasAnimation = true;
            break;
        }
    }
    bool ambientCreature =
        (lowerName.find("firefly") != std::string::npos) ||
        (lowerName.find("fireflies") != std::string::npos) ||
        (lowerName.find("fireflys") != std::string::npos) ||
        (lowerName.find("dragonfly") != std::string::npos) ||
        (lowerName.find("dragonflies") != std::string::npos) ||
        (lowerName.find("butterfly") != std::string::npos) ||
        (lowerName.find("moth") != std::string::npos);
    gpuModel.disableAnimation = (foliageOrTreeLike && !ambientCreature) || chestName;
    gpuModel.shadowWindFoliage = foliageOrTreeLike && !ambientCreature;
    gpuModel.isFoliageLike = foliageOrTreeLike && !ambientCreature;
    gpuModel.isElvenLike =
        (lowerName.find("elf") != std::string::npos) ||
        (lowerName.find("elven") != std::string::npos) ||
        (lowerName.find("quel") != std::string::npos);
    gpuModel.isLanternLike =
        (lowerName.find("lantern") != std::string::npos) ||
        (lowerName.find("lamp") != std::string::npos) ||
        (lowerName.find("light") != std::string::npos);
    gpuModel.isKoboldFlame =
        (lowerName.find("kobold") != std::string::npos) &&
        ((lowerName.find("candle") != std::string::npos) ||
         (lowerName.find("torch") != std::string::npos) ||
         (lowerName.find("mine") != std::string::npos));
    gpuModel.isGroundDetail = groundDetailModel;
    if (groundDetailModel) {
        // Ground clutter (grass/pebbles/detail cards) should never block camera/movement.
        gpuModel.collisionNoBlock = true;
    }
    // Spell effect models: particle-dominated with minimal geometry (e.g. LevelUp.m2)
    gpuModel.isSpellEffect = hasParticles && model.vertices.size() <= 200 &&
                              model.particleEmitters.size() >= 3;
    // Water vegetation: cattails, reeds, bulrushes, kelp, seaweed, lilypad near water
    gpuModel.isWaterVegetation =
        (lowerName.find("cattail") != std::string::npos) ||
        (lowerName.find("reed") != std::string::npos) ||
        (lowerName.find("bulrush") != std::string::npos) ||
        (lowerName.find("seaweed") != std::string::npos) ||
        (lowerName.find("kelp") != std::string::npos) ||
        (lowerName.find("lilypad") != std::string::npos);
    // Ambient creature effects: particle-based glow (exempt from particle dampeners)
    gpuModel.isFireflyEffect = ambientCreature;

    // Build collision mesh + spatial grid from M2 bounding geometry
    gpuModel.collision.vertices = model.collisionVertices;
    gpuModel.collision.indices = model.collisionIndices;
    gpuModel.collision.build();
    if (gpuModel.collision.valid()) {
        core::Logger::getInstance().debug("  M2 collision mesh: ", gpuModel.collision.triCount,
            " tris, grid ", gpuModel.collision.gridCellsX, "x", gpuModel.collision.gridCellsY);
    }

    // Flag smoke models for UV scroll animation (particle emitters not implemented)
    {
        std::string smokeName = model.name;
        std::transform(smokeName.begin(), smokeName.end(), smokeName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.isSmoke = (smokeName.find("smoke") != std::string::npos);
    }

    // Identify idle variation sequences (animation ID 0 = Stand)
    for (int i = 0; i < static_cast<int>(model.sequences.size()); i++) {
        if (model.sequences[i].id == 0 && model.sequences[i].duration > 0) {
            gpuModel.idleVariationIndices.push_back(i);
        }
    }

    if (hasGeometry) {
        // Create VBO with interleaved vertex data
        // Format: position (3), normal (3), texcoord0 (2), texcoord1 (2), boneWeights (4), boneIndices (4 as float)
        const size_t floatsPerVertex = 18;
        std::vector<float> vertexData;
        vertexData.reserve(model.vertices.size() * floatsPerVertex);

        for (const auto& v : model.vertices) {
            vertexData.push_back(v.position.x);
            vertexData.push_back(v.position.y);
            vertexData.push_back(v.position.z);
            vertexData.push_back(v.normal.x);
            vertexData.push_back(v.normal.y);
            vertexData.push_back(v.normal.z);
            vertexData.push_back(v.texCoords[0].x);
            vertexData.push_back(v.texCoords[0].y);
            vertexData.push_back(v.texCoords[1].x);
            vertexData.push_back(v.texCoords[1].y);
            float w0 = v.boneWeights[0] / 255.0f;
            float w1 = v.boneWeights[1] / 255.0f;
            float w2 = v.boneWeights[2] / 255.0f;
            float w3 = v.boneWeights[3] / 255.0f;
            vertexData.push_back(w0);
            vertexData.push_back(w1);
            vertexData.push_back(w2);
            vertexData.push_back(w3);
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[0], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[1], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[2], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[3], uint8_t(127))));
        }

        // Upload vertex buffer to GPU
        {
            auto buf = uploadBuffer(*vkCtx_,
                vertexData.data(), vertexData.size() * sizeof(float),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            gpuModel.vertexBuffer = buf.buffer;
            gpuModel.vertexAlloc = buf.allocation;
        }

        // Upload index buffer to GPU
        {
            auto buf = uploadBuffer(*vkCtx_,
                model.indices.data(), model.indices.size() * sizeof(uint16_t),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
            gpuModel.indexBuffer = buf.buffer;
            gpuModel.indexAlloc = buf.allocation;
        }
    }

    // Load ALL textures from the model into a local vector.
    // textureLoadFailed[i] is true if texture[i] had a named path that failed to load.
    // Such batches are hidden (batchOpacity=0) rather than rendered white.
    std::vector<VkTexture*> allTextures;
    std::vector<bool> textureLoadFailed;
    std::vector<std::string> textureKeysLower;
    if (assetManager) {
        for (size_t ti = 0; ti < model.textures.size(); ti++) {
            const auto& tex = model.textures[ti];
            std::string texPath = tex.filename;
            // Some extracted M2 texture strings contain embedded NUL + garbage suffix.
            // Truncate at first NUL so valid paths like "...foo.blp\0junk" still resolve.
            size_t nul = texPath.find('\0');
            if (nul != std::string::npos) {
                texPath.resize(nul);
            }
            if (!texPath.empty()) {
                std::string texKey = texPath;
                std::replace(texKey.begin(), texKey.end(), '/', '\\');
                std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                VkTexture* texPtr = loadTexture(texPath, tex.flags);
                bool failed = (texPtr == whiteTexture_.get());
                if (failed) {
                    static uint32_t loggedModelTextureFails = 0;
                    static bool loggedModelTextureFailSuppressed = false;
                    if (loggedModelTextureFails < 250) {
                        LOG_WARNING("M2 model ", model.name, " texture[", ti, "] failed to load: ", texPath);
                        ++loggedModelTextureFails;
                    } else if (!loggedModelTextureFailSuppressed) {
                        LOG_WARNING("M2 model texture-failure warnings suppressed after ",
                                    loggedModelTextureFails, " entries");
                        loggedModelTextureFailSuppressed = true;
                    }
                }
                if (isInvisibleTrap) {
                    LOG_INFO("  InvisibleTrap texture[", ti, "]: ", texPath, " -> ", (failed ? "WHITE" : "OK"));
                }
                allTextures.push_back(texPtr);
                textureLoadFailed.push_back(failed);
                textureKeysLower.push_back(std::move(texKey));
            } else {
                if (isInvisibleTrap) {
                    LOG_INFO("  InvisibleTrap texture[", ti, "]: EMPTY (using white fallback)");
                }
                allTextures.push_back(whiteTexture_.get());
                textureLoadFailed.push_back(false);  // Empty filename = intentional white (type!=0)
                textureKeysLower.emplace_back();
            }
        }
    }

    static const bool kGlowDiag = envFlagEnabled("WOWEE_M2_GLOW_DIAG", false);
    if (kGlowDiag) {
        std::string lowerName = model.name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        const bool lanternLike =
            (lowerName.find("lantern") != std::string::npos) ||
            (lowerName.find("lamp") != std::string::npos) ||
            (lowerName.find("light") != std::string::npos);
        if (lanternLike) {
            for (size_t ti = 0; ti < model.textures.size(); ++ti) {
                const std::string key = (ti < textureKeysLower.size()) ? textureKeysLower[ti] : std::string();
                LOG_DEBUG("M2 GLOW TEX '", model.name, "' tex[", ti, "]='", key, "' flags=0x",
                          std::hex, model.textures[ti].flags, std::dec);
            }
        }
    }

    // Copy particle emitter data and resolve textures
    gpuModel.particleEmitters = model.particleEmitters;
    gpuModel.particleTextures.resize(model.particleEmitters.size(), whiteTexture_.get());
    for (size_t ei = 0; ei < model.particleEmitters.size(); ei++) {
        uint16_t texIdx = model.particleEmitters[ei].texture;
        if (texIdx < allTextures.size() && allTextures[texIdx] != nullptr) {
            gpuModel.particleTextures[ei] = allTextures[texIdx];
        }
    }

    // Copy texture transform data for UV animation
    gpuModel.textureTransforms = model.textureTransforms;
    gpuModel.textureTransformLookup = model.textureTransformLookup;
    gpuModel.hasTextureAnimation = false;

    // Build per-batch GPU entries
    if (!model.batches.empty()) {
        for (const auto& batch : model.batches) {
            M2ModelGPU::BatchGPU bgpu;
            bgpu.indexStart = batch.indexStart;
            bgpu.indexCount = batch.indexCount;

            // Store texture animation index from batch
            bgpu.textureAnimIndex = batch.textureAnimIndex;
            if (bgpu.textureAnimIndex != 0xFFFF) {
                gpuModel.hasTextureAnimation = true;
            }

            // Store blend mode and flags from material
            if (batch.materialIndex < model.materials.size()) {
                bgpu.blendMode = model.materials[batch.materialIndex].blendMode;
                bgpu.materialFlags = model.materials[batch.materialIndex].flags;
            }

            // Copy LOD level from batch
            bgpu.submeshLevel = batch.submeshLevel;

            // Resolve texture: batch.textureIndex → textureLookup → allTextures
            VkTexture* tex = whiteTexture_.get();
            bool texFailed = false;
            std::string batchTexKeyLower;
            if (batch.textureIndex < model.textureLookup.size()) {
                uint16_t texIdx = model.textureLookup[batch.textureIndex];
                if (texIdx < allTextures.size()) {
                    tex = allTextures[texIdx];
                    texFailed = (texIdx < textureLoadFailed.size()) && textureLoadFailed[texIdx];
                    if (texIdx < textureKeysLower.size()) {
                        batchTexKeyLower = textureKeysLower[texIdx];
                    }
                }
                if (texIdx < model.textures.size()) {
                    bgpu.texFlags = static_cast<uint8_t>(model.textures[texIdx].flags & 0x3);
                }
            } else if (!allTextures.empty()) {
                tex = allTextures[0];
                texFailed = !textureLoadFailed.empty() && textureLoadFailed[0];
                if (!textureKeysLower.empty()) {
                    batchTexKeyLower = textureKeysLower[0];
                }
            }

            if (texFailed && groundDetailModel) {
                static const std::string kDetailFallbackTexture = "World\\NoDXT\\Detail\\8des_detaildoodads01.blp";
                VkTexture* fallbackTex = loadTexture(kDetailFallbackTexture, 0);
                if (fallbackTex != nullptr && fallbackTex != whiteTexture_.get()) {
                    tex = fallbackTex;
                    texFailed = false;
                }
            }
            bgpu.texture = tex;
            const bool exactLanternGlowTexture =
                (batchTexKeyLower == "world\\expansion06\\doodads\\nightelf\\7ne_druid_streetlamp01_light.blp") ||
                (batchTexKeyLower == "world\\generic\\nightelf\\passive doodads\\lamps\\glowblue32.blp") ||
                (batchTexKeyLower == "world\\generic\\human\\passive doodads\\stormwind\\t_vfx_glow01_64.blp") ||
                (batchTexKeyLower == "world\\azeroth\\karazahn\\passivedoodads\\bonfire\\flamelicksmallblue.blp") ||
                (batchTexKeyLower == "world\\generic\\nightelf\\passive doodads\\magicalimplements\\glow.blp");
            const bool texHasGlowToken =
                (batchTexKeyLower.find("glow") != std::string::npos) ||
                (batchTexKeyLower.find("flare") != std::string::npos) ||
                (batchTexKeyLower.find("halo") != std::string::npos) ||
                (batchTexKeyLower.find("light") != std::string::npos);
            const bool texHasFlameToken =
                (batchTexKeyLower.find("flame") != std::string::npos) ||
                (batchTexKeyLower.find("fire") != std::string::npos) ||
                (batchTexKeyLower.find("flamelick") != std::string::npos) ||
                (batchTexKeyLower.find("ember") != std::string::npos);
            const bool texGlowCardToken =
                (batchTexKeyLower.find("glow") != std::string::npos) ||
                (batchTexKeyLower.find("flamelick") != std::string::npos) ||
                (batchTexKeyLower.find("lensflare") != std::string::npos) ||
                (batchTexKeyLower.find("t_vfx") != std::string::npos) ||
                (batchTexKeyLower.find("lightbeam") != std::string::npos) ||
                (batchTexKeyLower.find("glowball") != std::string::npos) ||
                (batchTexKeyLower.find("genericglow") != std::string::npos);
            const bool texLikelyFlame =
                (batchTexKeyLower.find("fire") != std::string::npos) ||
                (batchTexKeyLower.find("flame") != std::string::npos) ||
                (batchTexKeyLower.find("torch") != std::string::npos);
            const bool texLanternFamily =
                (batchTexKeyLower.find("lantern") != std::string::npos) ||
                (batchTexKeyLower.find("lamp") != std::string::npos) ||
                (batchTexKeyLower.find("elf") != std::string::npos) ||
                (batchTexKeyLower.find("silvermoon") != std::string::npos) ||
                (batchTexKeyLower.find("quel") != std::string::npos) ||
                (batchTexKeyLower.find("thalas") != std::string::npos);
            const bool modelLanternFamily =
                (lowerName.find("lantern") != std::string::npos) ||
                (lowerName.find("lamp") != std::string::npos) ||
                (lowerName.find("light") != std::string::npos);
            bgpu.lanternGlowHint =
                exactLanternGlowTexture ||
                ((texHasGlowToken || (modelLanternFamily && texHasFlameToken)) &&
                 (texLanternFamily || modelLanternFamily) &&
                 (!texLikelyFlame || modelLanternFamily));
            bgpu.glowCardLike = bgpu.lanternGlowHint && texGlowCardToken;
            const bool texCoolTint =
                (batchTexKeyLower.find("blue") != std::string::npos) ||
                (batchTexKeyLower.find("nightelf") != std::string::npos) ||
                (batchTexKeyLower.find("arcane") != std::string::npos);
            const bool texRedTint =
                (batchTexKeyLower.find("red") != std::string::npos) ||
                (batchTexKeyLower.find("scarlet") != std::string::npos) ||
                (batchTexKeyLower.find("ruby") != std::string::npos);
            bgpu.glowTint = texCoolTint ? 1 : (texRedTint ? 2 : 0);
            bool texHasAlpha = false;
            if (tex != nullptr && tex != whiteTexture_.get()) {
                auto ait = textureHasAlphaByPtr_.find(tex);
                texHasAlpha = (ait != textureHasAlphaByPtr_.end()) ? ait->second : false;
            }
            bgpu.hasAlpha = texHasAlpha;
            bool colorKeyBlack = false;
            if (tex != nullptr && tex != whiteTexture_.get()) {
                auto cit = textureColorKeyBlackByPtr_.find(tex);
                colorKeyBlack = (cit != textureColorKeyBlackByPtr_.end()) ? cit->second : false;
            }
            bgpu.colorKeyBlack = colorKeyBlack;
            // textureCoordIndex is an index into a texture coord combo table, not directly
            // a UV set selector. Most batches have index=0 (UV set 0). We always use UV set 0
            // since we don't have the full combo table — dual-UV effects are rare edge cases.
            bgpu.textureUnit = 0;

            // Batch is hidden only when its named texture failed to load (avoids white shell artifacts).
            // Do NOT bake transparency/color animation tracks here — they animate over time and
            // baking the first keyframe value causes legitimate meshes to become invisible.
            // Keep terrain clutter visible even when source texture paths are malformed.
            bgpu.batchOpacity = (texFailed && !groundDetailModel) ? 0.0f : 1.0f;

            // Compute batch center and radius for glow sprite positioning
            if ((bgpu.blendMode >= 3 || bgpu.colorKeyBlack) && batch.indexCount > 0) {
                glm::vec3 sum(0.0f);
                uint32_t counted = 0;
                for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                    if (j < model.indices.size()) {
                        uint16_t vi = model.indices[j];
                        if (vi < model.vertices.size()) {
                            sum += model.vertices[vi].position;
                            counted++;
                        }
                    }
                }
                if (counted > 0) {
                    bgpu.center = sum / static_cast<float>(counted);
                    float maxDist = 0.0f;
                    for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                        if (j < model.indices.size()) {
                            uint16_t vi = model.indices[j];
                            if (vi < model.vertices.size()) {
                                float d = glm::length(model.vertices[vi].position - bgpu.center);
                                maxDist = std::max(maxDist, d);
                            }
                        }
                    }
                    bgpu.glowSize = std::max(maxDist, 0.5f);
                }
            }

            // Optional diagnostics for glow/light batches (disabled by default).
            if (kGlowDiag &&
                (lowerName.find("light") != std::string::npos ||
                 lowerName.find("lamp") != std::string::npos ||
                 lowerName.find("lantern") != std::string::npos)) {
                LOG_DEBUG("M2 GLOW DIAG '", model.name, "' batch ", gpuModel.batches.size(),
                          ": blend=", bgpu.blendMode, " matFlags=0x",
                          std::hex, bgpu.materialFlags, std::dec,
                          " colorKey=", bgpu.colorKeyBlack ? "Y" : "N",
                          " hasAlpha=", bgpu.hasAlpha ? "Y" : "N",
                          " unlit=", (bgpu.materialFlags & 0x01) ? "Y" : "N",
                          " lanternHint=", bgpu.lanternGlowHint ? "Y" : "N",
                          " glowSize=", bgpu.glowSize,
                          " tex=", bgpu.texture,
                          " idxCount=", bgpu.indexCount);
            }
            gpuModel.batches.push_back(bgpu);
        }
    } else {
        // Fallback: single batch covering all indices with first texture
        M2ModelGPU::BatchGPU bgpu;
        bgpu.indexStart = 0;
        bgpu.indexCount = gpuModel.indexCount;
        bgpu.texture = allTextures.empty() ? whiteTexture_.get() : allTextures[0];
        bool texHasAlpha = false;
        if (bgpu.texture != nullptr && bgpu.texture != whiteTexture_.get()) {
            auto ait = textureHasAlphaByPtr_.find(bgpu.texture);
            texHasAlpha = (ait != textureHasAlphaByPtr_.end()) ? ait->second : false;
        }
        bgpu.hasAlpha = texHasAlpha;
        bool colorKeyBlack = false;
        if (bgpu.texture != nullptr && bgpu.texture != whiteTexture_.get()) {
            auto cit = textureColorKeyBlackByPtr_.find(bgpu.texture);
            colorKeyBlack = (cit != textureColorKeyBlackByPtr_.end()) ? cit->second : false;
        }
        bgpu.colorKeyBlack = colorKeyBlack;
        gpuModel.batches.push_back(bgpu);
    }

    // Detect particle emitter volume models: box mesh (24 verts, 36 indices)
    // with disproportionately large bounds. These are invisible bounding volumes
    // that only exist to spawn particles — their mesh should never be rendered.
    if (!isInvisibleTrap && !groundDetailModel &&
        gpuModel.vertexCount <= 24 && gpuModel.indexCount <= 36
        && !model.particleEmitters.empty()) {
        glm::vec3 size = gpuModel.boundMax - gpuModel.boundMin;
        float maxDim = std::max({size.x, size.y, size.z});
        if (maxDim > 5.0f) {
            gpuModel.isInvisibleTrap = true;
            LOG_DEBUG("M2 emitter volume hidden: '", model.name, "' size=(",
                      size.x, " x ", size.y, " x ", size.z, ")");
        }
    }

    // Allocate Vulkan descriptor sets and UBOs for each batch
    for (auto& bgpu : gpuModel.batches) {
        // Create combined UBO for M2Params (binding 1) + M2Material (binding 2)
        // We allocate them as separate buffers for clarity
        VmaAllocationInfo matAllocInfo{};
        {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = sizeof(M2MaterialUBO);
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &bgpu.materialUBO, &bgpu.materialUBOAlloc, &matAllocInfo);

            // Write initial material data (static per-batch — fadeAlpha/interiorDarken updated at draw time)
            M2MaterialUBO mat{};
            mat.hasTexture = (bgpu.texture != nullptr && bgpu.texture != whiteTexture_.get()) ? 1 : 0;
            mat.alphaTest = (bgpu.blendMode == 1 || (bgpu.blendMode >= 2 && !bgpu.hasAlpha)) ? 1 : 0;
            mat.colorKeyBlack = bgpu.colorKeyBlack ? 1 : 0;
            mat.colorKeyThreshold = 0.08f;
            mat.unlit = (bgpu.materialFlags & 0x01) ? 1 : 0;
            mat.blendMode = bgpu.blendMode;
            mat.fadeAlpha = 1.0f;
            mat.interiorDarken = 0.0f;
            mat.specularIntensity = 0.5f;
            memcpy(matAllocInfo.pMappedData, &mat, sizeof(mat));
            bgpu.materialUBOMapped = matAllocInfo.pMappedData;
        }

        // Allocate descriptor set and write all bindings
        bgpu.materialSet = allocateMaterialSet();
        if (bgpu.materialSet) {
            VkTexture* batchTex = bgpu.texture ? bgpu.texture : whiteTexture_.get();
            VkDescriptorImageInfo imgInfo = batchTex->descriptorInfo();

            VkDescriptorBufferInfo matBufInfo{};
            matBufInfo.buffer = bgpu.materialUBO;
            matBufInfo.offset = 0;
            matBufInfo.range = sizeof(M2MaterialUBO);

            VkWriteDescriptorSet writes[2] = {};
            // binding 0: texture
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = bgpu.materialSet;
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[0].pImageInfo = &imgInfo;
            // binding 2: M2Material UBO
            writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[1].dstSet = bgpu.materialSet;
            writes[1].dstBinding = 2;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[1].pBufferInfo = &matBufInfo;

            vkUpdateDescriptorSets(vkCtx_->getDevice(), 2, writes, 0, nullptr);
        }
    }

    models[modelId] = std::move(gpuModel);

    LOG_DEBUG("Loaded M2 model: ", model.name, " (", models[modelId].vertexCount, " vertices, ",
              models[modelId].indexCount / 3, " triangles, ", models[modelId].batches.size(), " batches)");


    return true;
}

uint32_t M2Renderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
    auto modelIt = models.find(modelId);
    if (modelIt == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }
    const auto& mdlRef = modelIt->second;

    // Ground clutter is procedurally scattered and high-count; avoid O(N) dedup
    // scans that can hitch when new tiles stream in.
    if (!mdlRef.isGroundDetail) {
        // Deduplicate: skip if same model already at nearly the same position
        for (const auto& existing : instances) {
            if (existing.modelId == modelId) {
                glm::vec3 d = existing.position - position;
                if (glm::dot(d, d) < 0.01f) {
                    return existing.id;
                }
            }
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    if (mdlRef.isGroundDetail) {
        instance.position.z -= computeGroundDetailDownOffset(mdlRef, scale);
    }
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(mdlRef, localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);

    // Initialize animation: play first sequence (usually Stand/Idle)
    const auto& mdl = mdlRef;
    if (mdl.hasAnimation && !mdl.disableAnimation && !mdl.sequences.empty()) {
        instance.currentSequenceIndex = 0;
        instance.idleSequenceIndex = 0;
        instance.animDuration = static_cast<float>(mdl.sequences[0].duration);
        instance.animTime = static_cast<float>(rand() % std::max(1u, mdl.sequences[0].duration));
        instance.variationTimer = 3000.0f + static_cast<float>(rand() % 8000);
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

    return instance.id;
}

uint32_t M2Renderer::createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                                const glm::vec3& position) {
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    // Deduplicate: skip if same model already at nearly the same position
    for (const auto& existing : instances) {
        if (existing.modelId == modelId) {
            glm::vec3 d = existing.position - position;
            if (glm::dot(d, d) < 0.01f) {
                return existing.id;
            }
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;  // Used for frustum culling
    instance.rotation = glm::vec3(0.0f);
    instance.scale = 1.0f;
    instance.modelMatrix = modelMatrix;
    instance.invModelMatrix = glm::inverse(modelMatrix);
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);
    // Initialize animation
    const auto& mdl2 = models[modelId];
    if (mdl2.hasAnimation && !mdl2.disableAnimation && !mdl2.sequences.empty()) {
        instance.currentSequenceIndex = 0;
        instance.idleSequenceIndex = 0;
        instance.animDuration = static_cast<float>(mdl2.sequences[0].duration);
        instance.animTime = static_cast<float>(rand() % std::max(1u, mdl2.sequences[0].duration));
        instance.variationTimer = 3000.0f + static_cast<float>(rand() % 8000);
    } else {
        instance.animTime = static_cast<float>(rand()) / RAND_MAX * 10000.0f;
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

    return instance.id;
}

// --- Bone animation helpers (same logic as CharacterRenderer) ---

static int findKeyframeIndex(const std::vector<uint32_t>& timestamps, float time) {
    if (timestamps.empty()) return -1;
    if (timestamps.size() == 1) return 0;
    for (size_t i = 0; i < timestamps.size() - 1; i++) {
        if (time < static_cast<float>(timestamps[i + 1])) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(timestamps.size() - 2);
}

// Resolve sequence index and time for a track, handling global sequences.
static void resolveTrackTime(const pipeline::M2AnimationTrack& track,
                              int seqIdx, float time,
                              const std::vector<uint32_t>& globalSeqDurations,
                              int& outSeqIdx, float& outTime) {
    if (track.globalSequence >= 0 &&
        static_cast<size_t>(track.globalSequence) < globalSeqDurations.size()) {
        // Global sequence: always use sub-array 0, wrap time at global duration
        outSeqIdx = 0;
        float dur = static_cast<float>(globalSeqDurations[track.globalSequence]);
        outTime = (dur > 0.0f) ? std::fmod(time, dur) : 0.0f;
    } else {
        outSeqIdx = seqIdx;
        outTime = time;
    }
}

static glm::vec3 interpVec3(const pipeline::M2AnimationTrack& track,
                             int seqIdx, float time, const glm::vec3& def,
                             const std::vector<uint32_t>& globalSeqDurations) {
    if (!track.hasData()) return def;
    int si; float t;
    resolveTrackTime(track, seqIdx, time, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return def;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.vec3Values.empty()) return def;
    auto safe = [&](const glm::vec3& v) -> glm::vec3 {
        if (std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z)) return def;
        return v;
    };
    if (keys.vec3Values.size() == 1) return safe(keys.vec3Values[0]);
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return def;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.vec3Values.size() - 1);
    if (i0 == i1) return safe(keys.vec3Values[i0]);
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return safe(glm::mix(keys.vec3Values[i0], keys.vec3Values[i1], frac));
}

static glm::quat interpQuat(const pipeline::M2AnimationTrack& track,
                              int seqIdx, float time,
                              const std::vector<uint32_t>& globalSeqDurations) {
    glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    if (!track.hasData()) return identity;
    int si; float t;
    resolveTrackTime(track, seqIdx, time, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return identity;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.quatValues.empty()) return identity;
    auto safe = [&](const glm::quat& q) -> glm::quat {
        float len = glm::length(q);
        if (len < 0.001f || std::isnan(len)) return identity;
        return q;
    };
    if (keys.quatValues.size() == 1) return safe(keys.quatValues[0]);
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return identity;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.quatValues.size() - 1);
    if (i0 == i1) return safe(keys.quatValues[i0]);
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return glm::slerp(safe(keys.quatValues[i0]), safe(keys.quatValues[i1]), frac);
}

static void computeBoneMatrices(const M2ModelGPU& model, M2Instance& instance) {
    size_t numBones = std::min(model.bones.size(), size_t(128));
    if (numBones == 0) return;
    instance.boneMatrices.resize(numBones);
    const auto& gsd = model.globalSequenceDurations;

    for (size_t i = 0; i < numBones; i++) {
        const auto& bone = model.bones[i];
        glm::vec3 trans = interpVec3(bone.translation, instance.currentSequenceIndex, instance.animTime, glm::vec3(0.0f), gsd);
        glm::quat rot = interpQuat(bone.rotation, instance.currentSequenceIndex, instance.animTime, gsd);
        glm::vec3 scl = interpVec3(bone.scale, instance.currentSequenceIndex, instance.animTime, glm::vec3(1.0f), gsd);

        // Sanity check scale to avoid degenerate matrices
        if (scl.x < 0.001f) scl.x = 1.0f;
        if (scl.y < 0.001f) scl.y = 1.0f;
        if (scl.z < 0.001f) scl.z = 1.0f;

        glm::mat4 local = glm::translate(glm::mat4(1.0f), bone.pivot);
        local = glm::translate(local, trans);
        local *= glm::toMat4(rot);
        local = glm::scale(local, scl);
        local = glm::translate(local, -bone.pivot);

        if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) < numBones) {
            instance.boneMatrices[i] = instance.boneMatrices[bone.parentBone] * local;
        } else {
            instance.boneMatrices[i] = local;
        }
    }
}

void M2Renderer::update(float deltaTime, const glm::vec3& cameraPos, const glm::mat4& viewProjection) {
    if (spatialIndexDirty_) {
        rebuildSpatialIndex();
    }

    float dtMs = deltaTime * 1000.0f;

    // Cache camera state for frustum-culling bone computation
    cachedCamPos_ = cameraPos;
    const float maxRenderDistance = (instances.size() > 2000) ? 800.0f : 2800.0f;
    cachedMaxRenderDistSq_ = maxRenderDistance * maxRenderDistance;

    // Build frustum for culling bones
    Frustum updateFrustum;
    updateFrustum.extractFromMatrix(viewProjection);

    // --- Smoke particle spawning ---
    std::uniform_real_distribution<float> distXY(-0.4f, 0.4f);
    std::uniform_real_distribution<float> distVelXY(-0.3f, 0.3f);
    std::uniform_real_distribution<float> distVelZ(3.0f, 5.0f);
    std::uniform_real_distribution<float> distLife(4.0f, 7.0f);
    std::uniform_real_distribution<float> distDrift(-0.2f, 0.2f);

    smokeEmitAccum += deltaTime;
    float emitInterval = 1.0f / 8.0f;  // 8 particles per second per emitter

    for (auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;

        if (model.isSmoke && smokeEmitAccum >= emitInterval &&
            static_cast<int>(smokeParticles.size()) < MAX_SMOKE_PARTICLES) {
            // Emission point: model origin in world space (model matrix already positions at chimney)
            glm::vec3 emitWorld = glm::vec3(instance.modelMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

            // Occasionally spawn a spark instead of smoke (~1 in 8)
            bool spark = (smokeRng() % 8 == 0);

            SmokeParticle p;
            p.position = emitWorld + glm::vec3(distXY(smokeRng), distXY(smokeRng), 0.0f);
            if (spark) {
                p.velocity = glm::vec3(distVelXY(smokeRng) * 2.0f, distVelXY(smokeRng) * 2.0f, distVelZ(smokeRng) * 1.5f);
                p.maxLife = 0.8f + static_cast<float>(smokeRng() % 100) / 100.0f * 1.2f;  // 0.8-2.0s
                p.size = 0.5f;
                p.isSpark = 1.0f;
            } else {
                p.velocity = glm::vec3(distVelXY(smokeRng), distVelXY(smokeRng), distVelZ(smokeRng));
                p.maxLife = distLife(smokeRng);
                p.size = 1.0f;
                p.isSpark = 0.0f;
            }
            p.life = 0.0f;
            p.instanceId = instance.id;
            smokeParticles.push_back(p);
        }
    }

    if (smokeEmitAccum >= emitInterval) {
        smokeEmitAccum = 0.0f;
    }

    // --- Update existing smoke particles ---
    for (auto it = smokeParticles.begin(); it != smokeParticles.end(); ) {
        it->life += deltaTime;
        if (it->life >= it->maxLife) {
            it = smokeParticles.erase(it);
            continue;
        }
        it->position += it->velocity * deltaTime;
        it->velocity.z *= 0.98f;  // Slight deceleration
        it->velocity.x += distDrift(smokeRng) * deltaTime;
        it->velocity.y += distDrift(smokeRng) * deltaTime;
        // Grow from 1.0 to 3.5 over lifetime
        float t = it->life / it->maxLife;
        it->size = 1.0f + t * 2.5f;
        ++it;
    }

    // --- Normal M2 animation update ---
    // Phase 1: Update animation state (cheap, sequential)
    // Collect indices of instances that need bone matrix computation.
    // Reuse persistent vector to avoid allocation stutter
    boneWorkIndices_.clear();
    if (boneWorkIndices_.capacity() < instances.size()) {
        boneWorkIndices_.reserve(instances.size());
    }

    for (size_t idx = 0; idx < instances.size(); ++idx) {
        auto& instance = instances[idx];
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;

        if (!model.hasAnimation || model.disableAnimation) {
            instance.animTime += dtMs;
            // Wrap animation time for models with particle emitters so emission
            // rate tracks keep looping instead of running past their keyframes.
            if (!model.particleEmitters.empty() && instance.animTime > 3333.0f) {
                instance.animTime = std::fmod(instance.animTime, 3333.0f);
            }
            continue;
        }

        instance.animTime += dtMs * instance.animSpeed;

        // Validate sequence index
        if (instance.currentSequenceIndex < 0 ||
            instance.currentSequenceIndex >= static_cast<int>(model.sequences.size())) {
            instance.currentSequenceIndex = 0;
            if (!model.sequences.empty()) {
                instance.animDuration = static_cast<float>(model.sequences[0].duration);
            }
        }

        // Handle animation looping / variation transitions
        // When animDuration is 0 (e.g. "Stand" with infinite loop) but the model
        // has particle emitters, wrap time so particle emission tracks keep looping.
        if (instance.animDuration <= 0.0f && !model.particleEmitters.empty()) {
            instance.animDuration = 3333.0f;  // ~3.3s loop for continuous particle effects
        }
        if (instance.animDuration > 0.0f && instance.animTime >= instance.animDuration) {
            if (instance.playingVariation) {
                // Variation finished — return to idle
                instance.playingVariation = false;
                instance.currentSequenceIndex = instance.idleSequenceIndex;
                if (instance.idleSequenceIndex < static_cast<int>(model.sequences.size())) {
                    instance.animDuration = static_cast<float>(model.sequences[instance.idleSequenceIndex].duration);
                }
                instance.animTime = 0.0f;
                instance.variationTimer = 4000.0f + static_cast<float>(rand() % 6000);
            } else {
                // Loop idle
                instance.animTime = std::fmod(instance.animTime, std::max(1.0f, instance.animDuration));
            }
        }

        // Idle variation timer — occasionally play a different idle sequence
        if (!instance.playingVariation && model.idleVariationIndices.size() > 1) {
            instance.variationTimer -= dtMs;
            if (instance.variationTimer <= 0.0f) {
                int pick = rand() % static_cast<int>(model.idleVariationIndices.size());
                int newSeq = model.idleVariationIndices[pick];
                if (newSeq != instance.currentSequenceIndex && newSeq < static_cast<int>(model.sequences.size())) {
                    instance.playingVariation = true;
                    instance.currentSequenceIndex = newSeq;
                    instance.animDuration = static_cast<float>(model.sequences[newSeq].duration);
                    instance.animTime = 0.0f;
                } else {
                    instance.variationTimer = 2000.0f + static_cast<float>(rand() % 4000);
                }
            }
        }

        // Frustum + distance cull: skip expensive bone computation for off-screen instances.
        // Keep thresholds aligned with render culling so visible distant ambient actors
        // (fish/seagulls/etc.) continue animating instead of freezing in idle poses.
        float worldRadius = model.boundRadius * instance.scale;
        float cullRadius = worldRadius;
        if (model.disableAnimation) {
            cullRadius = std::max(cullRadius, 3.0f);
        }
        glm::vec3 toCam = instance.position - cachedCamPos_;
        float distSq = glm::dot(toCam, toCam);
        float effectiveMaxDistSq = cachedMaxRenderDistSq_ * std::max(1.0f, cullRadius / 12.0f);
        if (model.disableAnimation) {
            effectiveMaxDistSq *= 2.6f;
        }
        if (distSq > effectiveMaxDistSq) continue;
        float paddedRadius = std::max(cullRadius * 1.5f, cullRadius + 3.0f);
        if (cullRadius > 0.0f && !updateFrustum.intersectsSphere(instance.position, paddedRadius)) continue;

        boneWorkIndices_.push_back(idx);
    }

    // Phase 2: Compute bone matrices (expensive, parallel if enough work)
    const size_t animCount = boneWorkIndices_.size();
    if (animCount > 0) {
        static const size_t minParallelAnimInstances = std::max<size_t>(
            8, envSizeOrDefault("WOWEE_M2_ANIM_MT_MIN", 96));
        if (animCount < minParallelAnimInstances || numAnimThreads_ <= 1) {
            // Sequential — not enough work to justify thread overhead
            for (size_t i : boneWorkIndices_) {
                if (i >= instances.size()) continue;
                auto& inst = instances[i];
                auto mdlIt = models.find(inst.modelId);
                if (mdlIt == models.end()) continue;
                computeBoneMatrices(mdlIt->second, inst);
            }
        } else {
            // Parallel — dispatch across worker threads
            static const size_t minAnimWorkPerThread = std::max<size_t>(
                16, envSizeOrDefault("WOWEE_M2_ANIM_WORK_PER_THREAD", 64));
            const size_t maxUsefulThreads = std::max<size_t>(
                1, (animCount + minAnimWorkPerThread - 1) / minAnimWorkPerThread);
            const size_t numThreads = std::min(static_cast<size_t>(numAnimThreads_), maxUsefulThreads);
            if (numThreads <= 1) {
                for (size_t i : boneWorkIndices_) {
                    if (i >= instances.size()) continue;
                    auto& inst = instances[i];
                    auto mdlIt = models.find(inst.modelId);
                    if (mdlIt == models.end()) continue;
                    computeBoneMatrices(mdlIt->second, inst);
                }
            } else {
                const size_t chunkSize = animCount / numThreads;
                const size_t remainder = animCount % numThreads;

                // Reuse persistent futures vector to avoid allocation
                animFutures_.clear();
                if (animFutures_.capacity() < numThreads) {
                    animFutures_.reserve(numThreads);
                }

                size_t start = 0;
                for (size_t t = 0; t < numThreads; ++t) {
                    size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                    animFutures_.push_back(std::async(std::launch::async,
                        [this, start, end]() {
                            for (size_t j = start; j < end; ++j) {
                                size_t idx = boneWorkIndices_[j];
                                if (idx >= instances.size()) continue;
                                auto& inst = instances[idx];
                                auto mdlIt = models.find(inst.modelId);
                                if (mdlIt == models.end()) continue;
                                computeBoneMatrices(mdlIt->second, inst);
                            }
                        }));
                    start = end;
                }

                for (auto& f : animFutures_) {
                    f.get();
                }
            }
        }
    }

    // Phase 3: Particle update (sequential — uses RNG, not thread-safe)
    // Run for ALL nearby instances with particle emitters, not just those in
    // boneWorkIndices_, so particles keep animating even when bone updates are culled.
    for (size_t idx = 0; idx < instances.size(); ++idx) {
        auto& instance = instances[idx];
        auto mdlIt = models.find(instance.modelId);
        if (mdlIt == models.end()) continue;
        const auto& model = mdlIt->second;
        if (model.particleEmitters.empty()) continue;
        // Distance cull: only update particles within visible range
        glm::vec3 toCam = instance.position - cachedCamPos_;
        float distSq = glm::dot(toCam, toCam);
        if (distSq > cachedMaxRenderDistSq_) continue;
        emitParticles(instance, model, deltaTime);
        updateParticles(instance, deltaTime);
    }
}

void M2Renderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (instances.empty() || !opaquePipeline_) {
        return;
    }

    // Debug: log once when we start rendering
    static bool loggedOnce = false;
    if (!loggedOnce) {
        loggedOnce = true;
        LOG_INFO("M2 render: ", instances.size(), " instances, ", models.size(), " models");
    }

    // Build frustum for culling
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix();
    Frustum frustum;
    frustum.extractFromMatrix(projection * view);

    // Reuse persistent buffers (clear instead of reallocating)
    glowSprites_.clear();

    lastDrawCallCount = 0;

    // Adaptive render distance: balanced for performance without excessive pop-in
    const float maxRenderDistance = (instances.size() > 2000) ? 350.0f : 1000.0f;
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;
    const float fadeStartFraction = 0.75f;
    const glm::vec3 camPos = camera.getPosition();

    // Build sorted visible instance list: cull then sort by modelId to batch VAO binds
    // Reuse persistent vector to avoid allocation
    sortedVisible_.clear();
    // Reserve based on expected visible count (roughly 30% of total instances in dense areas)
    const size_t expectedVisible = std::min(instances.size() / 3, size_t(600));
    if (sortedVisible_.capacity() < expectedVisible) {
        sortedVisible_.reserve(expectedVisible);
    }

    // Early distance rejection: max possible render distance (tight but safe upper bound)
    const float maxPossibleDistSq = maxRenderDistance * maxRenderDistance * 4.0f;  // 2x safety margin (reduced from 4x)

    for (uint32_t i = 0; i < static_cast<uint32_t>(instances.size()); ++i) {
        const auto& instance = instances[i];

        // Fast early rejection: skip instances that are definitely too far
        glm::vec3 toCam = instance.position - camPos;
        float distSq = glm::dot(toCam, toCam);
        if (distSq > maxPossibleDistSq) continue;  // Early out before model lookup

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;
        if (!model.isValid() || model.isSmoke || model.isInvisibleTrap) continue;
        float worldRadius = model.boundRadius * instance.scale;
        float cullRadius = worldRadius;
        if (model.disableAnimation) {
            cullRadius = std::max(cullRadius, 3.0f);
        }
        float effectiveMaxDistSq = maxRenderDistanceSq * std::max(1.0f, cullRadius / 12.0f);
        if (model.disableAnimation) {
            effectiveMaxDistSq *= 2.6f;
        }
        if (model.isGroundDetail) {
            // Keep clutter local so distant grass doesn't overdraw the scene.
            effectiveMaxDistSq *= 0.75f;
        }
        // Removed aggressive small-object distance caps to prevent city pop-out
        // Small props (barrels, lanterns, etc.) now use same distance as larger objects
        if (distSq > effectiveMaxDistSq) continue;

        // Frustum cull with moderate padding to prevent edge pop-out during camera rotation
        // Reduced from 2.5x to 1.5x for better performance
        float paddedRadius = std::max(cullRadius * 1.5f, cullRadius + 3.0f);
        if (cullRadius > 0.0f && !frustum.intersectsSphere(instance.position, paddedRadius)) continue;

        sortedVisible_.push_back({i, instance.modelId, distSq, effectiveMaxDistSq});
    }

    // Sort by modelId to minimize vertex/index buffer rebinds
    std::stable_sort(sortedVisible_.begin(), sortedVisible_.end(),
                     [](const VisibleEntry& a, const VisibleEntry& b) { return a.modelId < b.modelId; });

    uint32_t currentModelId = UINT32_MAX;
    const M2ModelGPU* currentModel = nullptr;

    // State tracking
    VkPipeline currentPipeline = VK_NULL_HANDLE;
    uint32_t frameIndex = vkCtx_->getCurrentFrame();

    // Push constants struct matching m2.vert.glsl push_constant block
    struct M2PushConstants {
        glm::mat4 model;
        glm::vec2 uvOffset;
        int texCoordSet;
        int useBones;
        int isFoliage;
    };

    // Bind per-frame descriptor set (set 0) — shared across all draws
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    // Start with opaque pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, opaquePipeline_);
    currentPipeline = opaquePipeline_;

    for (const auto& entry : sortedVisible_) {
        if (entry.index >= instances.size()) continue;
        auto& instance = instances[entry.index];

        // Bind vertex + index buffers once per model group
        if (entry.modelId != currentModelId) {
            currentModelId = entry.modelId;
            auto mdlIt = models.find(currentModelId);
            if (mdlIt == models.end()) continue;
            currentModel = &mdlIt->second;
            if (!currentModel->vertexBuffer) continue;
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &currentModel->vertexBuffer, &offset);
            vkCmdBindIndexBuffer(cmd, currentModel->indexBuffer, 0, VK_INDEX_TYPE_UINT16);
        }

        const M2ModelGPU& model = *currentModel;

        // Distance-based fade alpha for smooth pop-in (squared-distance, no sqrt)
        float fadeAlpha = 1.0f;
        float fadeFrac = model.disableAnimation ? 0.55f : fadeStartFraction;
        float fadeStartDistSq = entry.effectiveMaxDistSq * fadeFrac * fadeFrac;
        if (entry.distSq > fadeStartDistSq) {
            fadeAlpha = std::clamp((entry.effectiveMaxDistSq - entry.distSq) /
                                  (entry.effectiveMaxDistSq - fadeStartDistSq), 0.0f, 1.0f);
        }

        float instanceFadeAlpha = fadeAlpha;
        if (model.isGroundDetail) {
            instanceFadeAlpha *= 0.82f;
        }

        // Upload bone matrices to SSBO if model has skeletal animation
        bool useBones = model.hasAnimation && !model.disableAnimation && !instance.boneMatrices.empty();
        if (useBones) {
            // Lazy-allocate bone SSBO on first use
            if (!instance.boneBuffer[frameIndex]) {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = 128 * sizeof(glm::mat4); // max 128 bones
                bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo allocInfo{};
                vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci,
                                &instance.boneBuffer[frameIndex], &instance.boneAlloc[frameIndex], &allocInfo);
                instance.boneMapped[frameIndex] = allocInfo.pMappedData;

                // Allocate descriptor set for bone SSBO
                instance.boneSet[frameIndex] = allocateBoneSet();
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

            // Upload bone matrices
            if (instance.boneMapped[frameIndex]) {
                int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), 128);
                memcpy(instance.boneMapped[frameIndex], instance.boneMatrices.data(),
                       numBones * sizeof(glm::mat4));
            }

            // Bind bone descriptor set (set 2)
            if (instance.boneSet[frameIndex]) {
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 2, 1, &instance.boneSet[frameIndex], 0, nullptr);
            }
        }

        // LOD selection based on distance
        float dist = std::sqrt(entry.distSq);
        uint16_t desiredLOD = 0;
        if (dist > 150.0f) desiredLOD = 3;
        else if (dist > 80.0f) desiredLOD = 2;
        else if (dist > 40.0f) desiredLOD = 1;

        uint16_t targetLOD = desiredLOD;
        if (desiredLOD > 0) {
            bool hasDesiredLOD = false;
            for (const auto& b : model.batches) {
                if (b.submeshLevel == desiredLOD) { hasDesiredLOD = true; break; }
            }
            if (!hasDesiredLOD) targetLOD = 0;
        }

        const bool foliageLikeModel = model.isFoliageLike;
        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;
            if (!model.isGroundDetail && batch.submeshLevel != targetLOD) continue;
            if (batch.batchOpacity < 0.01f) continue;

            const bool koboldFlameCard = batch.colorKeyBlack && model.isKoboldFlame;
            const bool smallCardLikeBatch =
                (batch.glowSize <= 1.35f) ||
                (batch.lanternGlowHint && batch.glowSize <= 6.0f);
            const bool batchUnlit = (batch.materialFlags & 0x01) != 0;
            const bool elvenLikeModel = model.isElvenLike;
            const bool lanternLikeModel = model.isLanternLike;
            const bool shouldUseGlowSprite =
                !koboldFlameCard &&
                (elvenLikeModel || (lanternLikeModel && batch.lanternGlowHint)) &&
                !model.isSpellEffect &&
                smallCardLikeBatch &&
                (batch.lanternGlowHint ||
                 (batch.blendMode >= 3) ||
                 (batch.colorKeyBlack && batchUnlit && batch.blendMode >= 1));
            if (shouldUseGlowSprite) {
                if (entry.distSq < 180.0f * 180.0f) {
                    glm::vec3 worldPos = glm::vec3(instance.modelMatrix * glm::vec4(batch.center, 1.0f));
                    GlowSprite gs;
                    gs.worldPos = worldPos;
                    if (batch.glowTint == 1 || elvenLikeModel) {
                        gs.color = glm::vec4(0.48f, 0.72f, 1.0f, 1.05f);
                    } else if (batch.glowTint == 2) {
                        gs.color = glm::vec4(1.0f, 0.28f, 0.22f, 1.10f);
                    } else {
                        gs.color = glm::vec4(1.0f, 0.82f, 0.46f, 1.15f);
                    }
                    gs.size = batch.glowSize * instance.scale * 1.45f;
                    glowSprites_.push_back(gs);
                    GlowSprite halo = gs;
                    halo.color.a *= 0.42f;
                    halo.size *= 1.8f;
                    glowSprites_.push_back(halo);
                }
                const bool cardLikeSkipMesh =
                    (batch.blendMode >= 3) ||
                    batch.colorKeyBlack ||
                    ((batch.materialFlags & 0x01) != 0);
                if ((batch.glowCardLike && lanternLikeModel) ||
                    (cardLikeSkipMesh && !lanternLikeModel)) {
                    continue;
                }
            }

            // Compute UV offset for texture animation
            glm::vec2 uvOffset(0.0f, 0.0f);
            if (batch.textureAnimIndex != 0xFFFF && model.hasTextureAnimation) {
                uint16_t lookupIdx = batch.textureAnimIndex;
                if (lookupIdx < model.textureTransformLookup.size()) {
                    uint16_t transformIdx = model.textureTransformLookup[lookupIdx];
                    if (transformIdx < model.textureTransforms.size()) {
                        const auto& tt = model.textureTransforms[transformIdx];
                        glm::vec3 trans = interpVec3(tt.translation,
                            instance.currentSequenceIndex, instance.animTime,
                            glm::vec3(0.0f), model.globalSequenceDurations);
                        uvOffset = glm::vec2(trans.x, trans.y);
                    }
                }
            }

            // Foliage/card-like batches render more stably as cutout (depth-write on)
            // instead of alpha-blended sorting.
            const bool foliageCutout =
                foliageLikeModel &&
                !model.isSpellEffect &&
                batch.blendMode <= 3;
            const bool forceCutout =
                !model.isSpellEffect &&
                (model.isGroundDetail ||
                 foliageCutout ||
                 batch.blendMode == 1 ||
                 (batch.blendMode >= 2 && !batch.hasAlpha) ||
                 batch.colorKeyBlack);

            // Select pipeline based on blend mode
            uint8_t effectiveBlendMode = batch.blendMode;
            if (model.isSpellEffect && (effectiveBlendMode == 4 || effectiveBlendMode == 5)) {
                effectiveBlendMode = 3;
            }
            if (forceCutout) {
                effectiveBlendMode = 1;
            }

            VkPipeline desiredPipeline;
            if (forceCutout) {
                // Use opaque pipeline + shader discard for stable foliage cards.
                desiredPipeline = opaquePipeline_;
            } else {
                switch (effectiveBlendMode) {
                    case 0: desiredPipeline = opaquePipeline_; break;
                    case 1: desiredPipeline = alphaTestPipeline_; break;
                    case 2: desiredPipeline = alphaPipeline_; break;
                    default: desiredPipeline = additivePipeline_; break;
                }
            }
            if (desiredPipeline != currentPipeline) {
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
                currentPipeline = desiredPipeline;
            }

            // Update material UBO with per-draw dynamic values (fadeAlpha, interiorDarken)
            if (batch.materialUBOMapped) {
                auto* mat = static_cast<M2MaterialUBO*>(batch.materialUBOMapped);
                mat->fadeAlpha = instanceFadeAlpha;
                mat->interiorDarken = insideInterior ? 1.0f : 0.0f;
                if (batch.colorKeyBlack) {
                    mat->colorKeyThreshold = (effectiveBlendMode == 4 || effectiveBlendMode == 5) ? 0.7f : 0.08f;
                }
                if (forceCutout) {
                    mat->alphaTest = model.isGroundDetail ? 3 : (foliageCutout ? 2 : 1);
                    if (model.isGroundDetail) {
                        mat->unlit = 0;
                    }
                }
            }

            // Bind material descriptor set (set 1) — skip batch if missing
            // to avoid inheriting a stale descriptor set from a prior renderer
            if (!batch.materialSet) continue;
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    pipelineLayout_, 1, 1, &batch.materialSet, 0, nullptr);

            // Push constants
            M2PushConstants pc;
            pc.model = instance.modelMatrix;
            pc.uvOffset = uvOffset;
            pc.texCoordSet = static_cast<int>(batch.textureUnit);
            pc.useBones = useBones ? 1 : 0;
            pc.isFoliage = model.shadowWindFoliage ? 1 : 0;
            vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);

            vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
            lastDrawCallCount++;
        }
    }

    // Render glow sprites as billboarded additive point lights
    if (!glowSprites_.empty() && particleAdditivePipeline_ && m2ParticleVB_ && glowTexDescSet_) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, particleAdditivePipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                particlePipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                particlePipelineLayout_, 1, 1, &glowTexDescSet_, 0, nullptr);

        // Push constants for particle: tileCount(vec2) + alphaKey(int)
        struct { float tileX, tileY; int alphaKey; } particlePush = {1.0f, 1.0f, 0};
        vkCmdPushConstants(cmd, particlePipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                           sizeof(particlePush), &particlePush);

        // Build and upload vertex data
        std::vector<float> glowData;
        glowData.reserve(glowSprites_.size() * 9);
        for (const auto& gs : glowSprites_) {
            glowData.push_back(gs.worldPos.x);
            glowData.push_back(gs.worldPos.y);
            glowData.push_back(gs.worldPos.z);
            glowData.push_back(gs.color.r);
            glowData.push_back(gs.color.g);
            glowData.push_back(gs.color.b);
            glowData.push_back(gs.color.a);
            glowData.push_back(gs.size);
            glowData.push_back(0.0f);
        }

        size_t uploadCount = std::min(glowSprites_.size(), MAX_M2_PARTICLES);
        memcpy(m2ParticleVBMapped_, glowData.data(), uploadCount * 9 * sizeof(float));

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &m2ParticleVB_, &offset);
        vkCmdDraw(cmd, static_cast<uint32_t>(uploadCount), 1, 0, 0);
    }

}

bool M2Renderer::initializeShadow(VkRenderPass shadowRenderPass) {
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
        LOG_ERROR("M2Renderer: failed to create shadow params UBO");
        return false;
    }
    ShadowParamsUBO defaultParams{};
    std::memcpy(allocInfo.pMappedData, &defaultParams, sizeof(defaultParams));

    // Create descriptor set layout: binding 0 = sampler2D, binding 1 = ShadowParams UBO
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
        LOG_ERROR("M2Renderer: failed to create shadow params layout");
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
        LOG_ERROR("M2Renderer: failed to create shadow params pool");
        return false;
    }

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo setAlloc{};
    setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    setAlloc.descriptorPool = shadowParamsPool_;
    setAlloc.descriptorSetCount = 1;
    setAlloc.pSetLayouts = &shadowParamsLayout_;
    if (vkAllocateDescriptorSets(device, &setAlloc, &shadowParamsSet_) != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: failed to allocate shadow params set");
        return false;
    }

    // Write descriptors (use white fallback for binding 0)
    VkDescriptorBufferInfo bufInfo{};
    bufInfo.buffer = shadowParamsUBO_;
    bufInfo.offset = 0;
    bufInfo.range = sizeof(ShadowParamsUBO);

    VkDescriptorImageInfo imgInfo{};
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgInfo.imageView = whiteTexture_->getImageView();
    imgInfo.sampler = whiteTexture_->getSampler();

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

    // Per-frame pool for foliage shadow texture sets (reset each frame)
    {
        VkDescriptorPoolSize texPoolSizes[2]{};
        texPoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        texPoolSizes[0].descriptorCount = 256;
        texPoolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        texPoolSizes[1].descriptorCount = 256;
        VkDescriptorPoolCreateInfo texPoolCI{};
        texPoolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        texPoolCI.maxSets = 256;
        texPoolCI.poolSizeCount = 2;
        texPoolCI.pPoolSizes = texPoolSizes;
        if (vkCreateDescriptorPool(device, &texPoolCI, nullptr, &shadowTexPool_) != VK_SUCCESS) {
            LOG_ERROR("M2Renderer: failed to create shadow texture pool");
            return false;
        }
    }

    // Create shadow pipeline layout: set 1 = shadowParamsLayout_, push constants = 128 bytes
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pc.offset = 0;
    pc.size = 128;  // lightSpaceMatrix (64) + model (64)
    shadowPipelineLayout_ = createPipelineLayout(device, {shadowParamsLayout_}, {pc});
    if (!shadowPipelineLayout_) {
        LOG_ERROR("M2Renderer: failed to create shadow pipeline layout");
        return false;
    }

    // Load shadow shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/shadow.vert.spv")) {
        LOG_ERROR("M2Renderer: failed to load shadow vertex shader");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/shadow.frag.spv")) {
        LOG_ERROR("M2Renderer: failed to load shadow fragment shader");
        return false;
    }

    // M2 vertex layout: 18 floats = 72 bytes stride
    // loc0=pos(off0), loc1=normal(off12), loc2=texCoord0(off24), loc5=texCoord1(off32),
    // loc3=boneWeights(off40), loc4=boneIndices(off56)
    // Shadow shader locations: 0=aPos, 1=aTexCoord, 2=aBoneWeights, 3=aBoneIndicesF
    // useBones=0 so locations 2,3 are never used
    VkVertexInputBindingDescription vertBind{};
    vertBind.binding = 0;
    vertBind.stride = 18 * sizeof(float);
    vertBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::vector<VkVertexInputAttributeDescription> vertAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT,    0},                     // aPos       -> position
        {1, 0, VK_FORMAT_R32G32_SFLOAT,       6 * sizeof(float)},     // aTexCoord  -> texCoord0
        {2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 10 * sizeof(float)},    // aBoneWeights
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 14 * sizeof(float)},    // aBoneIndicesF
    };

    shadowPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({vertBind}, vertAttrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        // Foliage/leaf cards are effectively two-sided; front-face culling can
        // drop them from the shadow map depending on light/view orientation.
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
        LOG_ERROR("M2Renderer: failed to create shadow pipeline");
        return false;
    }
    LOG_INFO("M2Renderer shadow pipeline initialized");
    return true;
}

void M2Renderer::renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix, float globalTime,
                              const glm::vec3& shadowCenter, float shadowRadius) {
    if (!shadowPipeline_ || !shadowParamsSet_) return;
    if (instances.empty() || models.empty()) return;

    struct ShadowPush { glm::mat4 lightSpaceMatrix; glm::mat4 model; };
    struct ShadowParamsUBO {
        int32_t useBones = 0;
        int32_t useTexture = 0;
        int32_t alphaTest = 0;
        int32_t foliageSway = 0;
        float windTime = 0.0f;
        float foliageMotionDamp = 1.0f;
    };
    const float shadowRadiusSq = shadowRadius * shadowRadius;

    // Reset per-frame texture descriptor pool for foliage alpha-test sets
    if (shadowTexPool_) {
        vkResetDescriptorPool(vkCtx_->getDevice(), shadowTexPool_, 0);
    }
    // Cache: texture imageView -> allocated descriptor set (avoids duplicates within frame)
    std::unordered_map<VkImageView, VkDescriptorSet> texSetCache;

    auto getTexDescSet = [&](VkTexture* tex) -> VkDescriptorSet {
        VkImageView iv = tex->getImageView();
        auto cacheIt = texSetCache.find(iv);
        if (cacheIt != texSetCache.end()) return cacheIt->second;

        VkDescriptorSet set = VK_NULL_HANDLE;
        VkDescriptorSetAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        ai.descriptorPool = shadowTexPool_;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &shadowParamsLayout_;
        if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set) != VK_SUCCESS) {
            return shadowParamsSet_; // fallback to white texture
        }
        VkDescriptorImageInfo imgInfo{};
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imgInfo.imageView = iv;
        imgInfo.sampler = tex->getSampler();
        VkDescriptorBufferInfo bufInfo{};
        bufInfo.buffer = shadowParamsUBO_;
        bufInfo.offset = 0;
        bufInfo.range = sizeof(ShadowParamsUBO);
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
        texSetCache[iv] = set;
        return set;
    };

    // Helper lambda to draw instances with a given foliageSway setting
    auto drawPass = [&](bool foliagePass) {
        ShadowParamsUBO params{};
        params.foliageSway = foliagePass ? 1 : 0;
        params.windTime = globalTime;
        params.foliageMotionDamp = 1.0f;
        // For foliage pass: enable texture+alphaTest in UBO (per-batch textures bound below)
        if (foliagePass) {
            params.useTexture = 1;
            params.alphaTest = 1;
        }

        VmaAllocationInfo allocInfo{};
        vmaGetAllocationInfo(vkCtx_->getAllocator(), shadowParamsAlloc_, &allocInfo);
        std::memcpy(allocInfo.pMappedData, &params, sizeof(params));

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
            0, 1, &shadowParamsSet_, 0, nullptr);

        uint32_t currentModelId = UINT32_MAX;
        const M2ModelGPU* currentModel = nullptr;

        for (const auto& instance : instances) {
            // Distance cull against shadow frustum
            glm::vec3 diff = instance.position - shadowCenter;
            if (glm::dot(diff, diff) > shadowRadiusSq) continue;

            auto modelIt = models.find(instance.modelId);
            if (modelIt == models.end()) continue;
            const M2ModelGPU& model = modelIt->second;
            if (!model.isValid() || model.isSmoke || model.isInvisibleTrap) continue;

            // Filter: only draw foliage models in foliage pass, non-foliage in non-foliage pass
            if (model.shadowWindFoliage != foliagePass) continue;

            // Bind vertex/index buffers when model changes
            if (instance.modelId != currentModelId) {
                currentModelId = instance.modelId;
                currentModel = &model;
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &currentModel->vertexBuffer, &offset);
                vkCmdBindIndexBuffer(cmd, currentModel->indexBuffer, 0, VK_INDEX_TYPE_UINT16);
            }

            ShadowPush push{lightSpaceMatrix, instance.modelMatrix};
            vkCmdPushConstants(cmd, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                               0, 128, &push);

            for (const auto& batch : model.batches) {
                if (batch.submeshLevel > 0) continue;
                // For foliage: bind per-batch texture for alpha-tested shadows
                if (foliagePass && batch.hasAlpha && batch.texture) {
                    VkDescriptorSet texSet = getTexDescSet(batch.texture);
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
                        0, 1, &texSet, 0, nullptr);
                } else if (foliagePass) {
                    // Non-alpha batch: rebind default set (white texture, alpha test passes)
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
                        0, 1, &shadowParamsSet_, 0, nullptr);
                }
                vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
            }
        }
    };

    // Pass 1: non-foliage (no wind displacement)
    drawPass(false);
    // Pass 2: foliage (wind displacement enabled, per-batch alpha-tested textures)
    drawPass(true);
}

// --- M2 Particle Emitter Helpers ---

float M2Renderer::interpFloat(const pipeline::M2AnimationTrack& track, float animTime,
                                int seqIdx, const std::vector<pipeline::M2Sequence>& /*seqs*/,
                                const std::vector<uint32_t>& globalSeqDurations) {
    if (!track.hasData()) return 0.0f;
    int si; float t;
    resolveTrackTime(track, seqIdx, animTime, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return 0.0f;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.floatValues.empty()) return 0.0f;
    if (keys.floatValues.size() == 1) return keys.floatValues[0];
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return 0.0f;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.floatValues.size() - 1);
    if (i0 == i1) return keys.floatValues[i0];
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return glm::mix(keys.floatValues[i0], keys.floatValues[i1], frac);
}

float M2Renderer::interpFBlockFloat(const pipeline::M2FBlock& fb, float lifeRatio) {
    if (fb.floatValues.empty()) return 1.0f;
    if (fb.floatValues.size() == 1 || fb.timestamps.empty()) return fb.floatValues[0];
    lifeRatio = glm::clamp(lifeRatio, 0.0f, 1.0f);
    // Find surrounding timestamps
    for (size_t i = 0; i < fb.timestamps.size() - 1; i++) {
        if (lifeRatio <= fb.timestamps[i + 1]) {
            float t0 = fb.timestamps[i];
            float t1 = fb.timestamps[i + 1];
            float dur = t1 - t0;
            float frac = (dur > 0.0f) ? (lifeRatio - t0) / dur : 0.0f;
            size_t v0 = std::min(i, fb.floatValues.size() - 1);
            size_t v1 = std::min(i + 1, fb.floatValues.size() - 1);
            return glm::mix(fb.floatValues[v0], fb.floatValues[v1], frac);
        }
    }
    return fb.floatValues.back();
}

glm::vec3 M2Renderer::interpFBlockVec3(const pipeline::M2FBlock& fb, float lifeRatio) {
    if (fb.vec3Values.empty()) return glm::vec3(1.0f);
    if (fb.vec3Values.size() == 1 || fb.timestamps.empty()) return fb.vec3Values[0];
    lifeRatio = glm::clamp(lifeRatio, 0.0f, 1.0f);
    for (size_t i = 0; i < fb.timestamps.size() - 1; i++) {
        if (lifeRatio <= fb.timestamps[i + 1]) {
            float t0 = fb.timestamps[i];
            float t1 = fb.timestamps[i + 1];
            float dur = t1 - t0;
            float frac = (dur > 0.0f) ? (lifeRatio - t0) / dur : 0.0f;
            size_t v0 = std::min(i, fb.vec3Values.size() - 1);
            size_t v1 = std::min(i + 1, fb.vec3Values.size() - 1);
            return glm::mix(fb.vec3Values[v0], fb.vec3Values[v1], frac);
        }
    }
    return fb.vec3Values.back();
}

std::vector<glm::vec3> M2Renderer::getWaterVegetationPositions(const glm::vec3& camPos, float maxDist) const {
    std::vector<glm::vec3> result;
    float maxDistSq = maxDist * maxDist;
    for (const auto& inst : instances) {
        auto it = models.find(inst.modelId);
        if (it == models.end() || !it->second.isWaterVegetation) continue;
        glm::vec3 diff = inst.position - camPos;
        if (glm::dot(diff, diff) <= maxDistSq) {
            result.push_back(inst.position);
        }
    }
    return result;
}

void M2Renderer::emitParticles(M2Instance& inst, const M2ModelGPU& gpu, float dt) {
    if (inst.emitterAccumulators.size() != gpu.particleEmitters.size()) {
        inst.emitterAccumulators.resize(gpu.particleEmitters.size(), 0.0f);
    }

    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
    std::uniform_real_distribution<float> distN(-1.0f, 1.0f);
    std::uniform_int_distribution<int> distTile;

    for (size_t ei = 0; ei < gpu.particleEmitters.size(); ei++) {
        const auto& em = gpu.particleEmitters[ei];
        if (!em.enabled) continue;

        float rate = interpFloat(em.emissionRate, inst.animTime, inst.currentSequenceIndex,
                                  gpu.sequences, gpu.globalSequenceDurations);
        float life = interpFloat(em.lifespan, inst.animTime, inst.currentSequenceIndex,
                                  gpu.sequences, gpu.globalSequenceDurations);
        if (rate <= 0.0f || life <= 0.0f) continue;

        inst.emitterAccumulators[ei] += rate * dt;

        while (inst.emitterAccumulators[ei] >= 1.0f && inst.particles.size() < MAX_M2_PARTICLES) {
            inst.emitterAccumulators[ei] -= 1.0f;

            M2Particle p;
            p.emitterIndex = static_cast<int>(ei);
            p.life = 0.0f;
            p.maxLife = life;
            p.tileIndex = 0.0f;

            // Position: emitter position transformed by bone matrix
            glm::vec3 localPos = em.position;
            glm::mat4 boneXform = glm::mat4(1.0f);
            if (em.bone < inst.boneMatrices.size()) {
                boneXform = inst.boneMatrices[em.bone];
            }
            glm::vec3 worldPos = glm::vec3(inst.modelMatrix * boneXform * glm::vec4(localPos, 1.0f));
            p.position = worldPos;

            // Velocity: emission speed in upward direction + random spread
            float speed = interpFloat(em.emissionSpeed, inst.animTime, inst.currentSequenceIndex,
                                       gpu.sequences, gpu.globalSequenceDurations);
            float vRange = interpFloat(em.verticalRange, inst.animTime, inst.currentSequenceIndex,
                                        gpu.sequences, gpu.globalSequenceDurations);
            float hRange = interpFloat(em.horizontalRange, inst.animTime, inst.currentSequenceIndex,
                                        gpu.sequences, gpu.globalSequenceDurations);

            // Base direction: up in model space, transformed to world
            glm::vec3 dir(0.0f, 0.0f, 1.0f);
            // Add random spread
            dir.x += distN(particleRng_) * hRange;
            dir.y += distN(particleRng_) * hRange;
            dir.z += distN(particleRng_) * vRange;
            float len = glm::length(dir);
            if (len > 0.001f) dir /= len;

            // Transform direction by bone + model orientation (rotation only)
            glm::mat3 rotMat = glm::mat3(inst.modelMatrix * boneXform);
            p.velocity = rotMat * dir * speed;

            // When emission speed is ~0 and bone animation isn't loaded (.anim files),
            // particles pile up at the same position. Give them a drift so they
            // spread outward like a mist/spray effect instead of clustering.
            if (std::abs(speed) < 0.01f) {
                if (gpu.isFireflyEffect) {
                    // Fireflies: gentle random drift in all directions
                    p.velocity = rotMat * glm::vec3(
                        distN(particleRng_) * 0.6f,
                        distN(particleRng_) * 0.6f,
                        distN(particleRng_) * 0.3f
                    );
                } else {
                    p.velocity = rotMat * glm::vec3(
                        distN(particleRng_) * 1.0f,
                        distN(particleRng_) * 1.0f,
                        -dist01(particleRng_) * 0.5f
                    );
                }
            }

            const uint32_t tilesX = std::max<uint16_t>(em.textureCols, 1);
            const uint32_t tilesY = std::max<uint16_t>(em.textureRows, 1);
            const uint32_t totalTiles = tilesX * tilesY;
            if ((em.flags & kParticleFlagTiled) && totalTiles > 1) {
                if (em.flags & kParticleFlagRandomized) {
                    distTile = std::uniform_int_distribution<int>(0, static_cast<int>(totalTiles - 1));
                    p.tileIndex = static_cast<float>(distTile(particleRng_));
                } else {
                    p.tileIndex = 0.0f;
                }
            }

            inst.particles.push_back(p);
        }
        // Cap accumulator to avoid bursts after lag
        if (inst.emitterAccumulators[ei] > 2.0f) {
            inst.emitterAccumulators[ei] = 0.0f;
        }
    }
}

void M2Renderer::updateParticles(M2Instance& inst, float dt) {
    auto it = models.find(inst.modelId);
    if (it == models.end()) return;
    const auto& gpu = it->second;

    for (size_t i = 0; i < inst.particles.size(); ) {
        auto& p = inst.particles[i];
        p.life += dt;
        if (p.life >= p.maxLife) {
            // Swap-and-pop removal
            inst.particles[i] = inst.particles.back();
            inst.particles.pop_back();
            continue;
        }
        // Apply gravity
        if (p.emitterIndex >= 0 && p.emitterIndex < static_cast<int>(gpu.particleEmitters.size())) {
            const auto& pem = gpu.particleEmitters[p.emitterIndex];
            float grav = interpFloat(pem.gravity,
                                      inst.animTime, inst.currentSequenceIndex,
                                      gpu.sequences, gpu.globalSequenceDurations);
            // When M2 gravity is 0, apply default gravity so particles arc downward.
            // Many fountain M2s rely on bone animation (.anim files) we don't load yet.
            // Firefly/ambient glow particles intentionally have zero gravity — skip fallback.
            if (grav == 0.0f && !gpu.isFireflyEffect) {
                float emSpeed = interpFloat(pem.emissionSpeed,
                                             inst.animTime, inst.currentSequenceIndex,
                                             gpu.sequences, gpu.globalSequenceDurations);
                if (std::abs(emSpeed) > 0.1f) {
                    grav = 4.0f;  // spray particles
                } else {
                    grav = 1.5f;  // mist/drift particles - gentler fall
                }
            }
            p.velocity.z -= grav * dt;
        }
        p.position += p.velocity * dt;
        i++;
    }
}

void M2Renderer::renderM2Particles(VkCommandBuffer cmd, VkDescriptorSet perFrameSet) {
    if (!particlePipeline_ || !m2ParticleVB_) return;

    // Collect all particles from all instances, grouped by texture+blend
    struct ParticleGroupKey {
        VkTexture* texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;

        bool operator==(const ParticleGroupKey& other) const {
            return texture == other.texture &&
                   blendType == other.blendType &&
                   tilesX == other.tilesX &&
                   tilesY == other.tilesY;
        }
    };
    struct ParticleGroupKeyHash {
        size_t operator()(const ParticleGroupKey& key) const {
            size_t h1 = std::hash<uintptr_t>{}(reinterpret_cast<uintptr_t>(key.texture));
            size_t h2 = std::hash<uint32_t>{}((static_cast<uint32_t>(key.tilesX) << 16) | key.tilesY);
            size_t h3 = std::hash<uint8_t>{}(key.blendType);
            return h1 ^ (h2 * 0x9e3779b9u) ^ (h3 * 0x85ebca6bu);
        }
    };
    struct ParticleGroup {
        VkTexture* texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;
        std::vector<float> vertexData;  // 9 floats per particle
    };
    std::unordered_map<ParticleGroupKey, ParticleGroup, ParticleGroupKeyHash> groups;

    size_t totalParticles = 0;

    for (auto& inst : instances) {
        if (inst.particles.empty()) continue;
        auto it = models.find(inst.modelId);
        if (it == models.end()) continue;
        const auto& gpu = it->second;

        for (const auto& p : inst.particles) {
            if (p.emitterIndex < 0 || p.emitterIndex >= static_cast<int>(gpu.particleEmitters.size())) continue;
            const auto& em = gpu.particleEmitters[p.emitterIndex];

            float lifeRatio = p.life / std::max(p.maxLife, 0.001f);
            glm::vec3 color = interpFBlockVec3(em.particleColor, lifeRatio);
            float alpha = std::min(interpFBlockFloat(em.particleAlpha, lifeRatio), 1.0f);
            float rawScale = interpFBlockFloat(em.particleScale, lifeRatio);

            if (!gpu.isSpellEffect && !gpu.isFireflyEffect) {
                color = glm::mix(color, glm::vec3(1.0f), 0.7f);
                if (rawScale > 2.0f) alpha *= 0.02f;
                if (em.blendingType == 3 || em.blendingType == 4) alpha *= 0.05f;
            }
            float scale = (gpu.isSpellEffect || gpu.isFireflyEffect) ? rawScale : std::min(rawScale, 1.5f);

            VkTexture* tex = whiteTexture_.get();
            if (p.emitterIndex < static_cast<int>(gpu.particleTextures.size())) {
                tex = gpu.particleTextures[p.emitterIndex];
            }

            uint16_t tilesX = std::max<uint16_t>(em.textureCols, 1);
            uint16_t tilesY = std::max<uint16_t>(em.textureRows, 1);
            uint32_t totalTiles = static_cast<uint32_t>(tilesX) * static_cast<uint32_t>(tilesY);
            ParticleGroupKey key{tex, em.blendingType, tilesX, tilesY};
            auto& group = groups[key];
            group.texture = tex;
            group.blendType = em.blendingType;
            group.tilesX = tilesX;
            group.tilesY = tilesY;

            group.vertexData.push_back(p.position.x);
            group.vertexData.push_back(p.position.y);
            group.vertexData.push_back(p.position.z);
            group.vertexData.push_back(color.r);
            group.vertexData.push_back(color.g);
            group.vertexData.push_back(color.b);
            group.vertexData.push_back(alpha);
            group.vertexData.push_back(scale);
            float tileIndex = p.tileIndex;
            if ((em.flags & kParticleFlagTiled) && totalTiles > 1) {
                float animSeconds = inst.animTime / 1000.0f;
                uint32_t animFrame = static_cast<uint32_t>(std::floor(animSeconds * totalTiles)) % totalTiles;
                tileIndex = std::fmod(p.tileIndex + static_cast<float>(animFrame),
                                      static_cast<float>(totalTiles));
            }
            group.vertexData.push_back(tileIndex);
            totalParticles++;
        }
    }

    if (totalParticles == 0) return;

    // Bind per-frame set (set 0) for particle pipeline
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            particlePipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    VkDeviceSize vbOffset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &m2ParticleVB_, &vbOffset);

    VkPipeline currentPipeline = VK_NULL_HANDLE;

    for (auto& [key, group] : groups) {
        if (group.vertexData.empty()) continue;

        uint8_t blendType = group.blendType;
        VkPipeline desiredPipeline = (blendType == 3 || blendType == 4)
            ? particleAdditivePipeline_ : particlePipeline_;
        if (desiredPipeline != currentPipeline) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
            currentPipeline = desiredPipeline;
        }

        // Allocate descriptor set for this group's texture
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        ai.descriptorPool = materialDescPool_;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &particleTexLayout_;
        VkDescriptorSet texSet = VK_NULL_HANDLE;
        if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &texSet) == VK_SUCCESS) {
            VkTexture* tex = group.texture ? group.texture : whiteTexture_.get();
            VkDescriptorImageInfo imgInfo = tex->descriptorInfo();
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet = texSet;
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            write.pImageInfo = &imgInfo;
            vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &write, 0, nullptr);

            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    particlePipelineLayout_, 1, 1, &texSet, 0, nullptr);
        }

        // Push constants: tileCount + alphaKey
        struct { float tileX, tileY; int alphaKey; } pc = {
            static_cast<float>(group.tilesX), static_cast<float>(group.tilesY),
            (blendType == 1) ? 1 : 0
        };
        vkCmdPushConstants(cmd, particlePipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                           sizeof(pc), &pc);

        // Upload and draw in chunks
        size_t count = group.vertexData.size() / 9;
        size_t offset = 0;
        while (offset < count) {
            size_t batch = std::min(count - offset, MAX_M2_PARTICLES);
            memcpy(m2ParticleVBMapped_, &group.vertexData[offset * 9], batch * 9 * sizeof(float));
            vkCmdDraw(cmd, static_cast<uint32_t>(batch), 1, 0, 0);
            offset += batch;
        }
    }
}

void M2Renderer::renderSmokeParticles(VkCommandBuffer cmd, VkDescriptorSet perFrameSet) {
    if (smokeParticles.empty() || !smokePipeline_ || !smokeVB_) return;

    // Build vertex data: pos(3) + lifeRatio(1) + size(1) + isSpark(1) per particle
    size_t count = std::min(smokeParticles.size(), static_cast<size_t>(MAX_SMOKE_PARTICLES));
    float* dst = static_cast<float*>(smokeVBMapped_);
    for (size_t i = 0; i < count; i++) {
        const auto& p = smokeParticles[i];
        *dst++ = p.position.x;
        *dst++ = p.position.y;
        *dst++ = p.position.z;
        *dst++ = p.life / p.maxLife;
        *dst++ = p.size;
        *dst++ = p.isSpark;
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, smokePipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            smokePipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    // Push constant: screenHeight
    float screenHeight = static_cast<float>(vkCtx_->getSwapchainExtent().height);
    vkCmdPushConstants(cmd, smokePipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0,
                       sizeof(float), &screenHeight);

    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &smokeVB_, &offset);
    vkCmdDraw(cmd, static_cast<uint32_t>(count), 1, 0, 0);
}

void M2Renderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.position = position;
    inst.updateModelMatrix();
    auto modelIt = models.find(inst.modelId);
    if (modelIt != models.end()) {
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(modelIt->second, localMin, localMax);
        transformAABB(inst.modelMatrix, localMin, localMax, inst.worldBoundsMin, inst.worldBoundsMax);
    }
    spatialIndexDirty_ = true;
}

void M2Renderer::setInstanceAnimationFrozen(uint32_t instanceId, bool frozen) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.animSpeed = frozen ? 0.0f : 1.0f;
    if (frozen) {
        inst.animTime = 0.0f;  // Reset to bind pose
    }
}

void M2Renderer::setInstanceTransform(uint32_t instanceId, const glm::mat4& transform) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];

    // Update model matrix directly
    inst.modelMatrix = transform;
    inst.invModelMatrix = glm::inverse(transform);

    // Extract position from transform for bounds
    inst.position = glm::vec3(transform[3]);

    // Update bounds
    auto modelIt = models.find(inst.modelId);
    if (modelIt != models.end()) {
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(modelIt->second, localMin, localMax);
        transformAABB(inst.modelMatrix, localMin, localMax, inst.worldBoundsMin, inst.worldBoundsMax);
    }
    spatialIndexDirty_ = true;
}

void M2Renderer::removeInstance(uint32_t instanceId) {
    for (auto it = instances.begin(); it != instances.end(); ++it) {
        if (it->id == instanceId) {
            destroyInstanceBones(*it);
            instances.erase(it);
            rebuildSpatialIndex();
            return;
        }
    }
}

void M2Renderer::removeInstances(const std::vector<uint32_t>& instanceIds) {
    if (instanceIds.empty() || instances.empty()) {
        return;
    }

    std::unordered_set<uint32_t> toRemove(instanceIds.begin(), instanceIds.end());
    const size_t oldSize = instances.size();
    for (auto& inst : instances) {
        if (toRemove.count(inst.id)) {
            destroyInstanceBones(inst);
        }
    }
    instances.erase(std::remove_if(instances.begin(), instances.end(),
                   [&toRemove](const M2Instance& inst) {
                       return toRemove.find(inst.id) != toRemove.end();
                   }),
                   instances.end());

    if (instances.size() != oldSize) {
        rebuildSpatialIndex();
    }
}

void M2Renderer::clear() {
    if (vkCtx_) {
        vkDeviceWaitIdle(vkCtx_->getDevice());
        for (auto& [id, model] : models) {
            destroyModelGPU(model);
        }
        for (auto& inst : instances) {
            destroyInstanceBones(inst);
        }
    }
    models.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    smokeParticles.clear();
    smokeEmitAccum = 0.0f;
}

void M2Renderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocusEnabled = (radius > 0.0f);
    collisionFocusPos = worldPos;
    collisionFocusRadius = std::max(0.0f, radius);
    collisionFocusRadiusSq = collisionFocusRadius * collisionFocusRadius;
}

void M2Renderer::clearCollisionFocus() {
    collisionFocusEnabled = false;
}

void M2Renderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
}

M2Renderer::GridCell M2Renderer::toCell(const glm::vec3& p) const {
    return GridCell{
        static_cast<int>(std::floor(p.x / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.y / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.z / SPATIAL_CELL_SIZE))
    };
}

void M2Renderer::rebuildSpatialIndex() {
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
    spatialIndexDirty_ = false;
}

void M2Renderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
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

    // Safety fallback to preserve collision correctness if the spatial index
    // misses candidates (e.g. during streaming churn).
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
}

void M2Renderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    // Find and remove models with no instances
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : models) {
        if (usedModelIds.find(id) == usedModelIds.end()) {
            toRemove.push_back(id);
        }
    }

    // Delete GPU resources and remove from map
    for (uint32_t id : toRemove) {
        auto it = models.find(id);
        if (it != models.end()) {
            destroyModelGPU(it->second);
            models.erase(it);
        }
    }

    if (!toRemove.empty()) {
        LOG_INFO("M2 cleanup: removed ", toRemove.size(), " unused models, ", models.size(), " remaining");
    }
}

VkTexture* M2Renderer::loadTexture(const std::string& path, uint32_t texFlags) {
    auto normalizeKey = [](std::string key) {
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    };
    std::string key = normalizeKey(path);

    // Check cache
    auto it = textureCache.find(key);
    if (it != textureCache.end()) {
        it->second.lastUse = ++textureCacheCounter_;
        return it->second.texture.get();
    }
    // No negative cache check — allow retries for transiently missing textures

    auto containsToken = [](const std::string& haystack, const char* token) {
        return haystack.find(token) != std::string::npos;
    };
    const bool colorKeyBlackHint =
        containsToken(key, "candle") ||
        containsToken(key, "flame") ||
        containsToken(key, "fire") ||
        containsToken(key, "torch") ||
        containsToken(key, "lamp") ||
        containsToken(key, "lantern") ||
        containsToken(key, "glow") ||
        containsToken(key, "flare") ||
        containsToken(key, "brazier") ||
        containsToken(key, "campfire") ||
        containsToken(key, "bonfire");

    // Load BLP texture
    pipeline::BLPImage blp = assetManager->loadTexture(key);
    if (!blp.isValid()) {
        // Return white fallback but don't cache the failure — MPQ reads can
        // fail transiently during streaming; allow retry on next model load.
        if (loggedTextureLoadFails_.insert(key).second) {
            LOG_WARNING("M2: Failed to load texture: ", path);
        }
        return whiteTexture_.get();
    }

    size_t base = static_cast<size_t>(blp.width) * static_cast<size_t>(blp.height) * 4ull;
    size_t approxBytes = base + (base / 3);
    if (textureCacheBytes_ + approxBytes > textureCacheBudgetBytes_) {
        static constexpr size_t kMaxFailedTextureCache = 200000;
        if (failedTextureCache_.size() < kMaxFailedTextureCache) {
            // Cache budget-rejected keys too; without this we repeatedly decode/load
            // the same textures every frame once budget is saturated.
            failedTextureCache_.insert(key);
        }
        if (textureBudgetRejectWarnings_ < 3) {
            LOG_WARNING("M2 texture cache full (", textureCacheBytes_ / (1024 * 1024),
                        " MB / ", textureCacheBudgetBytes_ / (1024 * 1024),
                        " MB), rejecting texture: ", path);
        }
        ++textureBudgetRejectWarnings_;
        return whiteTexture_.get();
    }

    // Track whether the texture actually uses alpha (any pixel with alpha < 255).
    bool hasAlpha = false;
    for (size_t i = 3; i < blp.data.size(); i += 4) {
        if (blp.data[i] != 255) {
            hasAlpha = true;
            break;
        }
    }

    // Create Vulkan texture
    auto tex = std::make_unique<VkTexture>();
    tex->upload(*vkCtx_, blp.data.data(), blp.width, blp.height, VK_FORMAT_R8G8B8A8_UNORM);

    // M2Texture flags: bit 0 = WrapS (1=repeat, 0=clamp), bit 1 = WrapT
    VkSamplerAddressMode wrapS = (texFlags & 0x1) ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VkSamplerAddressMode wrapT = (texFlags & 0x2) ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, wrapS, wrapT);

    VkTexture* texPtr = tex.get();

    TextureCacheEntry e;
    e.texture = std::move(tex);
    e.approxBytes = approxBytes;
    e.hasAlpha = hasAlpha;
    e.colorKeyBlack = colorKeyBlackHint;
    e.lastUse = ++textureCacheCounter_;
    textureCacheBytes_ += e.approxBytes;
    textureCache[key] = std::move(e);
    textureHasAlphaByPtr_[texPtr] = hasAlpha;
    textureColorKeyBlackByPtr_[texPtr] = colorKeyBlackHint;
    LOG_DEBUG("M2: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return texPtr;
}

uint32_t M2Renderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it != models.end()) {
            total += it->second.indexCount / 3;
        }
    }
    return total;
}

std::optional<float> M2Renderer::getFloorHeight(float glX, float glY, float glZ, float* outNormalZ) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;
    float bestNormalZ = 1.0f;  // Default to flat

    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 6.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 8.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        if (instance.scale <= 0.001f) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock || model.isInvisibleTrap || model.isSpellEffect) continue;

        // --- Mesh-based floor: vertical ray vs collision triangles ---
        // Does NOT skip the AABB path — both contribute and highest wins.
        if (model.collision.valid()) {
            glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

            model.collision.getFloorTrisInRange(
                localPos.x - 1.0f, localPos.y - 1.0f,
                localPos.x + 1.0f, localPos.y + 1.0f,
                collisionTriScratch_);

            glm::vec3 rayOrigin(localPos.x, localPos.y, localPos.z + 5.0f);
            glm::vec3 rayDir(0.0f, 0.0f, -1.0f);
            float bestHitZ = -std::numeric_limits<float>::max();
            bool hitAny = false;

            for (uint32_t ti : collisionTriScratch_) {
                if (ti >= model.collision.triCount) continue;
                if (model.collision.triBounds[ti].maxZ < localPos.z - 10.0f ||
                    model.collision.triBounds[ti].minZ > localPos.z + 5.0f) continue;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                // Two-sided: try both windings
                float tHit = rayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2);
                if (tHit < 0.0f)
                    tHit = rayTriangleIntersect(rayOrigin, rayDir, v0, v2, v1);
                if (tHit < 0.0f) continue;

                float hitZ = rayOrigin.z - tHit;

                // Walkable normal check (world space)
                glm::vec3 worldN(0.0f, 0.0f, 1.0f);  // Default to flat
                glm::vec3 localN = glm::cross(v1 - v0, v2 - v0);
                float nLen = glm::length(localN);
                if (nLen > 0.001f) {
                    localN /= nLen;
                    if (localN.z < 0.0f) localN = -localN;
                    worldN = glm::normalize(
                        glm::vec3(instance.modelMatrix * glm::vec4(localN, 0.0f)));
                    if (std::abs(worldN.z) < 0.35f) continue; // too steep (~70° max slope)
                }

                if (hitZ <= localPos.z + 3.0f && hitZ > bestHitZ) {
                    bestHitZ = hitZ;
                    hitAny = true;
                    bestNormalZ = std::abs(worldN.z);  // Store normal for output
                }
            }

            if (hitAny) {
                glm::vec3 localHit(localPos.x, localPos.y, bestHitZ);
                glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
                if (worldHit.z <= glZ + 3.0f && (!bestFloor || worldHit.z > *bestFloor)) {
                    bestFloor = worldHit.z;
                }
            }
            // Fall through to AABB floor — both contribute, highest wins
        }

        float zMargin = model.collisionBridge ? 25.0f : 2.0f;
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - zMargin || glZ > instance.worldBoundsMax.z + zMargin) {
            continue;
        }
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Must be within doodad footprint in local XY.
        // Stepped low platforms get a small pad so walk-up snapping catches edges.
        float footprintPad = 0.0f;
        if (model.collisionSteppedLowPlatform) {
            footprintPad = model.collisionPlanter ? 0.22f : 0.16f;
            if (model.collisionBridge) {
                footprintPad = 0.35f;
            }
        }
        if (localPos.x < localMin.x - footprintPad || localPos.x > localMax.x + footprintPad ||
            localPos.y < localMin.y - footprintPad || localPos.y > localMax.y + footprintPad) {
            continue;
        }

        // Construct "top" point at queried XY in local space, then transform back.
        float localTopZ = getEffectiveCollisionTopLocal(model, localPos, localMin, localMax);
        glm::vec3 localTop(localPos.x, localPos.y, localTopZ);
        glm::vec3 worldTop = glm::vec3(instance.modelMatrix * glm::vec4(localTop, 1.0f));

        // Reachability filter: allow a bit more climb for stepped low platforms.
        float maxStepUp = 1.0f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            maxStepUp = 2.0f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 3.0f : 2.4f;
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
        }
        if (worldTop.z > glZ + maxStepUp) continue;

        if (!bestFloor || worldTop.z > *bestFloor) {
            bestFloor = worldTop.z;
        }
    }

    // Output surface normal if requested
    if (outNormalZ) {
        *outNormalZ = bestNormalZ;
    }

    return bestFloor;
}

bool M2Renderer::checkCollision(const glm::vec3& from, const glm::vec3& to,
                                 glm::vec3& adjustedPos, float playerRadius) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool collided = false;

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(7.0f, 7.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(7.0f, 7.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    // Check against all M2 instances in local space (rotation-aware).
    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        const float broadMargin = playerRadius + 1.0f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && adjustedPos.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && adjustedPos.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && adjustedPos.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && adjustedPos.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + 2.5f && adjustedPos.z > instance.worldBoundsMax.z + 2.5f) continue;
        if (from.z + 2.5f < instance.worldBoundsMin.z && adjustedPos.z + 2.5f < instance.worldBoundsMin.z) continue;

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock || model.isInvisibleTrap || model.isSpellEffect) continue;
        if (instance.scale <= 0.001f) continue;

        // --- Mesh-based wall collision: closest-point push ---
        if (model.collision.valid()) {
            glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
            glm::vec3 localPos  = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
            float localRadius = playerRadius / instance.scale;

            model.collision.getWallTrisInRange(
                std::min(localFrom.x, localPos.x) - localRadius - 1.0f,
                std::min(localFrom.y, localPos.y) - localRadius - 1.0f,
                std::max(localFrom.x, localPos.x) + localRadius + 1.0f,
                std::max(localFrom.y, localPos.y) + localRadius + 1.0f,
                collisionTriScratch_);

            constexpr float PLAYER_HEIGHT = 2.0f;
            constexpr float MAX_TOTAL_PUSH = 0.02f; // Cap total push per instance
            bool pushed = false;
            float totalPushX = 0.0f, totalPushY = 0.0f;

            for (uint32_t ti : collisionTriScratch_) {
                if (ti >= model.collision.triCount) continue;
                if (localPos.z + PLAYER_HEIGHT < model.collision.triBounds[ti].minZ ||
                    localPos.z > model.collision.triBounds[ti].maxZ) continue;

                // Step-up: only skip wall when player is rising (jumping over it)
                constexpr float MAX_STEP_UP = 1.2f;
                bool rising = (localPos.z > localFrom.z + 0.05f);
                if (rising && localPos.z + MAX_STEP_UP >= model.collision.triBounds[ti].maxZ) continue;

                // Early out if we already pushed enough this instance
                float totalPushSoFar = std::sqrt(totalPushX * totalPushX + totalPushY * totalPushY);
                if (totalPushSoFar >= MAX_TOTAL_PUSH) break;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                glm::vec3 closest = closestPointOnTriangle(localPos, v0, v1, v2);
                glm::vec3 diff = localPos - closest;
                float distXY = std::sqrt(diff.x * diff.x + diff.y * diff.y);

                if (distXY < localRadius && distXY > 1e-4f) {
                    // Gentle push — very small fraction of penetration
                    float penetration = localRadius - distXY;
                    float pushDist = std::clamp(penetration * 0.08f, 0.001f, 0.015f);
                    float dx = (diff.x / distXY) * pushDist;
                    float dy = (diff.y / distXY) * pushDist;
                    localPos.x += dx;
                    localPos.y += dy;
                    totalPushX += dx;
                    totalPushY += dy;
                    pushed = true;
                } else if (distXY < 1e-4f) {
                    // On the plane — soft push along triangle normal XY
                    glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
                    float nxyLen = std::sqrt(n.x * n.x + n.y * n.y);
                    if (nxyLen > 1e-4f) {
                        float pushDist = std::min(localRadius, 0.015f);
                        float dx = (n.x / nxyLen) * pushDist;
                        float dy = (n.y / nxyLen) * pushDist;
                        localPos.x += dx;
                        localPos.y += dy;
                        totalPushX += dx;
                        totalPushY += dy;
                        pushed = true;
                    }
                }
            }

            if (pushed) {
                glm::vec3 worldPos = glm::vec3(instance.modelMatrix * glm::vec4(localPos, 1.0f));
                adjustedPos.x = worldPos.x;
                adjustedPos.y = worldPos.y;
                collided = true;
            }
            continue;
        }

        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
        float radiusScale = model.collisionNarrowVerticalProp ? 0.45f : 1.0f;
        float localRadius = (playerRadius * radiusScale) / instance.scale;

        glm::vec3 rawMin, rawMax;
        getTightCollisionBounds(model, rawMin, rawMax);
        glm::vec3 localMin = rawMin - glm::vec3(localRadius);
        glm::vec3 localMax = rawMax + glm::vec3(localRadius);
        float effectiveTop = getEffectiveCollisionTopLocal(model, localPos, rawMin, rawMax) + localRadius;
        glm::vec2 localCenter((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
        float fromR = glm::length(glm::vec2(localFrom.x, localFrom.y) - localCenter);
        float toR = glm::length(glm::vec2(localPos.x, localPos.y) - localCenter);

        // Feet-based vertical overlap test: ignore objects fully above/below us.
        constexpr float PLAYER_HEIGHT = 2.0f;
        if (localPos.z + PLAYER_HEIGHT < localMin.z || localPos.z > effectiveTop) {
            continue;
        }

        bool fromInsideXY =
            (localFrom.x >= localMin.x && localFrom.x <= localMax.x &&
             localFrom.y >= localMin.y && localFrom.y <= localMax.y);
        bool fromInsideZ = (localFrom.z + PLAYER_HEIGHT >= localMin.z && localFrom.z <= effectiveTop);
        bool escapingOverlap = (fromInsideXY && fromInsideZ && (toR > fromR + 1e-4f));
        bool allowEscapeRelax = escapingOverlap && !model.collisionSmallSolidProp;

        // Swept hard clamp for taller blockers only.
        // Low/stepable objects should be climbable and not "shove" the player off.
        float maxStepUp = 1.20f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            // Keep box/crate-class props hard-solid to prevent phase-through.
            maxStepUp = 0.75f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 2.8f : 2.4f;
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
        }
        bool stepableLowObject = (effectiveTop <= localFrom.z + maxStepUp);
        bool climbingAttempt = (localPos.z > localFrom.z + 0.18f);
        bool nearTop = (localFrom.z >= effectiveTop - 0.30f);
        float climbAllowance = model.collisionPlanter ? 0.95f : 0.60f;
        if (model.collisionSteppedLowPlatform && !model.collisionPlanter) {
            // Let low curb/planter blocks be stepable without sticky side shoves.
            climbAllowance = 1.00f;
        }
        if (model.collisionBridge) {
            climbAllowance = 3.0f;
        }
        if (model.collisionSmallSolidProp) {
            climbAllowance = 1.05f;
        }
        bool climbingTowardTop = climbingAttempt && (localFrom.z + climbAllowance >= effectiveTop);
        bool forceHardLateral =
            model.collisionSmallSolidProp &&
            !nearTop && !climbingTowardTop;
        if ((!stepableLowObject || forceHardLateral) && !allowEscapeRelax) {
            float tEnter = 0.0f;
            glm::vec3 sweepMax = localMax;
            sweepMax.z = std::min(sweepMax.z, effectiveTop);
            if (segmentIntersectsAABB(localFrom, localPos, localMin, sweepMax, tEnter)) {
                float tSafe = std::clamp(tEnter - 0.03f, 0.0f, 1.0f);
                glm::vec3 localSafe = localFrom + (localPos - localFrom) * tSafe;
                glm::vec3 worldSafe = glm::vec3(instance.modelMatrix * glm::vec4(localSafe, 1.0f));
                adjustedPos.x = worldSafe.x;
                adjustedPos.y = worldSafe.y;
                collided = true;
                continue;
            }
        }

        if (localPos.x < localMin.x || localPos.x > localMax.x ||
            localPos.y < localMin.y || localPos.y > localMax.y) {
            continue;
        }

        float pushLeft  = localPos.x - localMin.x;
        float pushRight = localMax.x - localPos.x;
        float pushBack  = localPos.y - localMin.y;
        float pushFront = localMax.y - localPos.y;

        float minPush = std::min({pushLeft, pushRight, pushBack, pushFront});
        if (allowEscapeRelax) {
            continue;
        }
        if (stepableLowObject && localFrom.z >= effectiveTop - 0.35f) {
            // Already on/near top surface: don't apply lateral push that ejects
            // the player from the object (carpets, platforms, etc).
            continue;
        }
        // Gentle fallback push for overlapping cases.
        float pushAmount;
        if (model.collisionNarrowVerticalProp) {
            pushAmount = std::clamp(minPush * 0.10f, 0.001f, 0.010f);
        } else if (model.collisionSteppedLowPlatform) {
            if (model.collisionPlanter && stepableLowObject) {
                pushAmount = std::clamp(minPush * 0.06f, 0.001f, 0.006f);
            } else {
            pushAmount = std::clamp(minPush * 0.12f, 0.003f, 0.012f);
            }
        } else if (stepableLowObject) {
            pushAmount = std::clamp(minPush * 0.12f, 0.002f, 0.015f);
        } else {
            pushAmount = std::clamp(minPush * 0.28f, 0.010f, 0.045f);
        }
        glm::vec3 localPush(0.0f);
        if (minPush == pushLeft) {
            localPush.x = -pushAmount;
        } else if (minPush == pushRight) {
            localPush.x = pushAmount;
        } else if (minPush == pushBack) {
            localPush.y = -pushAmount;
        } else {
            localPush.y = pushAmount;
        }

        glm::vec3 worldPush = glm::vec3(instance.modelMatrix * glm::vec4(localPush, 0.0f));
        adjustedPos.x += worldPush.x;
        adjustedPos.y += worldPush.y;
        collided = true;
    }

    return collided;
}

float M2Renderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;

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

        // Cheap world-space broad-phase.
        float tEnter = 0.0f;
        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.35f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.35f);
        if (!segmentIntersectsAABB(origin, origin + direction * maxDistance, worldMin, worldMax, tEnter)) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock || model.isInvisibleTrap || model.isSpellEffect) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);
        // Skip tiny doodads for camera occlusion; they cause jitter and false hits.
        glm::vec3 extents = (localMax - localMin) * instance.scale;
        if (glm::length(extents) < 0.75f) continue;

        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));
        if (!std::isfinite(localDir.x) || !std::isfinite(localDir.y) || !std::isfinite(localDir.z)) {
            continue;
        }

        // Local-space AABB slab intersection.
        glm::vec3 invDir = 1.0f / localDir;
        glm::vec3 tMin = (localMin - localOrigin) * invDir;
        glm::vec3 tMax = (localMax - localOrigin) * invDir;
        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = std::max({t1.x, t1.y, t1.z});
        float tFar = std::min({t2.x, t2.y, t2.z});
        if (tNear > tFar || tFar <= 0.0f) continue;

        float tHit = tNear > 0.0f ? tNear : tFar;
        glm::vec3 localHit = localOrigin + localDir * tHit;
        glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
        float worldDist = glm::length(worldHit - origin);
        if (worldDist > 0.0f && worldDist < closestHit) {
            closestHit = worldDist;
        }
    }

    return closestHit;
}

void M2Renderer::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old main-pass pipelines (NOT shadow, NOT pipeline layouts)
    if (opaquePipeline_)            { vkDestroyPipeline(device, opaquePipeline_, nullptr); opaquePipeline_ = VK_NULL_HANDLE; }
    if (alphaTestPipeline_)         { vkDestroyPipeline(device, alphaTestPipeline_, nullptr); alphaTestPipeline_ = VK_NULL_HANDLE; }
    if (alphaPipeline_)             { vkDestroyPipeline(device, alphaPipeline_, nullptr); alphaPipeline_ = VK_NULL_HANDLE; }
    if (additivePipeline_)          { vkDestroyPipeline(device, additivePipeline_, nullptr); additivePipeline_ = VK_NULL_HANDLE; }
    if (particlePipeline_)          { vkDestroyPipeline(device, particlePipeline_, nullptr); particlePipeline_ = VK_NULL_HANDLE; }
    if (particleAdditivePipeline_)  { vkDestroyPipeline(device, particleAdditivePipeline_, nullptr); particleAdditivePipeline_ = VK_NULL_HANDLE; }
    if (smokePipeline_)             { vkDestroyPipeline(device, smokePipeline_, nullptr); smokePipeline_ = VK_NULL_HANDLE; }

    // --- Load shaders ---
    rendering::VkShaderModule m2Vert, m2Frag;
    rendering::VkShaderModule particleVert, particleFrag;
    rendering::VkShaderModule smokeVert, smokeFrag;

    m2Vert.loadFromFile(device, "assets/shaders/m2.vert.spv");
    m2Frag.loadFromFile(device, "assets/shaders/m2.frag.spv");
    particleVert.loadFromFile(device, "assets/shaders/m2_particle.vert.spv");
    particleFrag.loadFromFile(device, "assets/shaders/m2_particle.frag.spv");
    smokeVert.loadFromFile(device, "assets/shaders/m2_smoke.vert.spv");
    smokeFrag.loadFromFile(device, "assets/shaders/m2_smoke.frag.spv");

    if (!m2Vert.isValid() || !m2Frag.isValid()) {
        LOG_ERROR("M2Renderer::recreatePipelines: missing required shaders");
        return;
    }

    VkRenderPass mainPass = vkCtx_->getImGuiRenderPass();

    // --- M2 model vertex input ---
    VkVertexInputBindingDescription m2Binding{};
    m2Binding.binding = 0;
    m2Binding.stride = 18 * sizeof(float);
    m2Binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> m2Attrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                     // position
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 3 * sizeof(float)},     // normal
        {2, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float)},        // texCoord0
        {5, 0, VK_FORMAT_R32G32_SFLOAT, 8 * sizeof(float)},        // texCoord1
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 10 * sizeof(float)}, // boneWeights
        {4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 14 * sizeof(float)}, // boneIndices (float)
    };

    auto buildM2Pipeline = [&](VkPipelineColorBlendAttachmentState blendState, bool depthWrite) -> VkPipeline {
        return PipelineBuilder()
            .setShaders(m2Vert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        m2Frag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({m2Binding}, m2Attrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, depthWrite, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(blendState)
            .setMultisample(vkCtx_->getMsaaSamples())
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    };

    opaquePipeline_ = buildM2Pipeline(PipelineBuilder::blendDisabled(), true);
    alphaTestPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), true);
    alphaPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), false);
    additivePipeline_ = buildM2Pipeline(PipelineBuilder::blendAdditive(), false);

    // --- Particle pipelines ---
    if (particleVert.isValid() && particleFrag.isValid()) {
        VkVertexInputBindingDescription pBind{};
        pBind.binding = 0;
        pBind.stride = 9 * sizeof(float); // pos3 + color4 + size1 + tile1
        pBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> pAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                    // position
            {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 3 * sizeof(float)}, // color
            {2, 0, VK_FORMAT_R32_SFLOAT, 7 * sizeof(float)},          // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 8 * sizeof(float)},          // tile
        };

        auto buildParticlePipeline = [&](VkPipelineColorBlendAttachmentState blend) -> VkPipeline {
            return PipelineBuilder()
                .setShaders(particleVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                            particleFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
                .setVertexInput({pBind}, pAttrs)
                .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
                .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
                .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
                .setColorBlendAttachment(blend)
                .setMultisample(vkCtx_->getMsaaSamples())
                .setLayout(particlePipelineLayout_)
                .setRenderPass(mainPass)
                .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
                .build(device);
        };

        particlePipeline_ = buildParticlePipeline(PipelineBuilder::blendAlpha());
        particleAdditivePipeline_ = buildParticlePipeline(PipelineBuilder::blendAdditive());
    }

    // --- Smoke pipeline ---
    if (smokeVert.isValid() && smokeFrag.isValid()) {
        VkVertexInputBindingDescription sBind{};
        sBind.binding = 0;
        sBind.stride = 6 * sizeof(float); // pos3 + lifeRatio1 + size1 + isSpark1
        sBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> sAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},           // position
            {1, 0, VK_FORMAT_R32_SFLOAT, 3 * sizeof(float)}, // lifeRatio
            {2, 0, VK_FORMAT_R32_SFLOAT, 4 * sizeof(float)}, // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 5 * sizeof(float)}, // isSpark
        };

        smokePipeline_ = PipelineBuilder()
            .setShaders(smokeVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        smokeFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({sBind}, sAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(PipelineBuilder::blendAlpha())
            .setMultisample(vkCtx_->getMsaaSamples())
            .setLayout(smokePipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device);
    }

    m2Vert.destroy(); m2Frag.destroy();
    particleVert.destroy(); particleFrag.destroy();
    smokeVert.destroy(); smokeFrag.destroy();

    core::Logger::getInstance().info("M2Renderer: pipelines recreated");
}

} // namespace rendering
} // namespace wowee
