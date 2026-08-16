#include "rendering/clouds.hpp"
#include "rendering/sky_system.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

namespace wowee {
namespace rendering {

Clouds::Clouds() = default;

Clouds::~Clouds() {
    shutdown();
}

/// Builds the one pipeline this effect draws with.
///
/// initialize() and recreatePipelines() both need it, described identically,
/// and the vertex layout went with it: a stride and an attribute stated twice
/// in one file are two chances to disagree about what a vertex is.
void Clouds::buildPipeline(VkDevice device,
                           const VkPipelineShaderStageCreateInfo& vertStage,
                           const VkPipelineShaderStageCreateInfo& fragStage) {
    // ------------------------------------------------------------------ vertex input
    VkVertexInputBindingDescription binding{};
    binding.binding   = 0;
    binding.stride    = sizeof(glm::vec3);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription posAttr{};
    posAttr.location = 0;
    posAttr.binding  = 0;
    posAttr.format   = VK_FORMAT_R32G32B32_SFLOAT;
    posAttr.offset   = 0;

    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    // ------------------------------------------------------------------ pipeline
    pipeline_ = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, {posAttr})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());


}

bool Clouds::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    LOG_INFO("Initializing cloud system (Vulkan)");

    vkCtx_ = ctx;
    VkDevice device = vkCtx_->getDevice();

    // ------------------------------------------------------------------ shaders
    auto shaders = loadShaderPair(device, "assets/shaders/clouds.vert.spv", "assets/shaders/clouds.frag.spv", "clouds");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // ------------------------------------------------------------------ push constants
    // Fragment-only push: 3 x vec4 = 48 bytes
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset     = 0;
    pushRange.size       = sizeof(CloudPush); // 48 bytes

    // ------------------------------------------------------------------ pipeline layout
    pipelineLayout_ = createPipelineLayout(device, {perFrameLayout}, {pushRange});
    if (pipelineLayout_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create clouds pipeline layout");
        return false;
    }

    buildPipeline(device, vertStage, fragStage);

    if (pipeline_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create clouds pipeline");
        return false;
    }

    // ------------------------------------------------------------------ geometry
    generateMesh();
    createBuffers();

    LOG_INFO("Cloud system initialized: ", indexCount_ / 3, " triangles");
    return true;
}

void Clouds::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    destroy(device, pipeline_);

    auto shaders = loadShaderPair(device, "assets/shaders/clouds.vert.spv", "assets/shaders/clouds.frag.spv", "clouds");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    buildPipeline(device, vertStage, fragStage);
}

void Clouds::shutdown() {
    destroyBuffers();

    if (vkCtx_) destroyPipeline(vkCtx_->getDevice(), pipeline_, pipelineLayout_);

    vkCtx_ = nullptr;
}

// ---------------------------------------------------------------------------
// Render
// ---------------------------------------------------------------------------

void Clouds::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const SkyParams& params) {
    if (!enabled_ || pipeline_ == VK_NULL_HANDLE) {
        return;
    }

    // Derive cloud base color from DBC horizon band, slightly brightened
    glm::vec3 cloudBaseColor = params.skyBand1Color * 1.1f;
    cloudBaseColor = glm::clamp(cloudBaseColor, glm::vec3(0.0f), glm::vec3(1.0f));

    // Sun direction (opposite of light direction). Guard the hemisphere like
    // Celestial/SkySystem do - directionalDir's sign convention is not stable,
    // and a flipped vector puts the cloud scatter glow opposite the real sun.
    glm::vec3 sunDir = -glm::normalize(params.directionalDir);
    if (sunDir.z < 0.0f) sunDir = -sunDir;
    float sunAboveHorizon = glm::clamp(sunDir.z, 0.0f, 1.0f);

    // Sun intensity based on elevation
    float sunIntensity = sunAboveHorizon;

    // Ambient light - brighter during day, dimmer at night
    float ambient = glm::mix(0.3f, 0.7f, sunAboveHorizon);

    CloudPush push{};
    push.cloudColor    = glm::vec4(cloudBaseColor, 1.0f);
    push.sunDirDensity = glm::vec4(sunDir, density_);
    push.windAndLight  = glm::vec4(windOffset_, sunIntensity, ambient, 0.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);

    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
        0, 1, &perFrameSet, 0, nullptr);

    vkCmdPushConstants(cmd, pipelineLayout_,
        VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer_, &offset);
    vkCmdBindIndexBuffer(cmd, indexBuffer_, 0, VK_INDEX_TYPE_UINT32);

    vkCmdDrawIndexed(cmd, static_cast<uint32_t>(indexCount_), 1, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// Update
// ---------------------------------------------------------------------------

void Clouds::update(float deltaTime) {
    if (!enabled_) {
        return;
    }
    windOffset_ += deltaTime * windSpeed_ * 0.05f; // Slow drift
}

// ---------------------------------------------------------------------------
// Density setter
// ---------------------------------------------------------------------------

void Clouds::setDensity(float density) {
    density_ = glm::clamp(density, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Mesh generation - identical algorithm to GL version
// ---------------------------------------------------------------------------

void Clouds::generateMesh() {
    vertices_.clear();
    indices_.clear();

    // Upper hemisphere - Z-up world: altitude goes into Z, horizontal spread in X/Y
    for (int ring = 0; ring <= RINGS; ++ring) {
        float phi        = (ring / static_cast<float>(RINGS)) * (static_cast<float>(M_PI) * 0.5f);
        float altZ       = RADIUS * std::cos(phi);
        float ringRadius = RADIUS * std::sin(phi);

        for (int seg = 0; seg <= SEGMENTS; ++seg) {
            float theta = (seg / static_cast<float>(SEGMENTS)) * (2.0f * static_cast<float>(M_PI));
            float x = ringRadius * std::cos(theta);
            float y = ringRadius * std::sin(theta);
            vertices_.emplace_back(x, y, altZ);
        }
    }

    for (int ring = 0; ring < RINGS; ++ring) {
        for (int seg = 0; seg < SEGMENTS; ++seg) {
            uint32_t current = static_cast<uint32_t>(ring * (SEGMENTS + 1) + seg);
            uint32_t next    = current + static_cast<uint32_t>(SEGMENTS + 1);

            indices_.push_back(current);
            indices_.push_back(next);
            indices_.push_back(current + 1);

            indices_.push_back(current + 1);
            indices_.push_back(next);
            indices_.push_back(next + 1);
        }
    }

    indexCount_ = static_cast<int>(indices_.size());
}

// ---------------------------------------------------------------------------
// GPU buffer management
// ---------------------------------------------------------------------------

void Clouds::createBuffers() {
    AllocatedBuffer vbuf = uploadBuffer(*vkCtx_,
        vertices_.data(),
        vertices_.size() * sizeof(glm::vec3),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    vertexBuffer_ = vbuf.buffer;
    vertexAlloc_  = vbuf.allocation;

    AllocatedBuffer ibuf = uploadBuffer(*vkCtx_,
        indices_.data(),
        indices_.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    indexBuffer_ = ibuf.buffer;
    indexAlloc_  = ibuf.allocation;

    // CPU data no longer needed
    vertices_.clear();
    vertices_.shrink_to_fit();
    indices_.clear();
    indices_.shrink_to_fit();
}

void Clouds::destroyBuffers() {
    if (!vkCtx_) return;

    VmaAllocator allocator = vkCtx_->getAllocator();

    destroy(allocator, vertexBuffer_, vertexAlloc_);
    destroy(allocator, indexBuffer_, indexAlloc_);
}

} // namespace rendering
} // namespace wowee
