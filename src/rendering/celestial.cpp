#include "rendering/celestial.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <vector>

namespace wowee {
namespace rendering {

Celestial::Celestial() = default;

Celestial::~Celestial() {
    shutdown();
}

VkPipeline Celestial::buildPipeline(VkDevice device,
                                    const VkPipelineShaderStageCreateInfo& vertStage,
                                    const VkPipelineShaderStageCreateInfo& fragStage) {
    // Vertex: vec3 pos + vec2 texCoord, stride = 20 bytes
    VkVertexInputBindingDescription binding = tightVertexBinding(5 * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = positionPlusUvAttrs();
    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    return PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        // On the far plane and tested, never written: the ground is drawn before
        // the sky now, and this is what keeps the sun behind a mountain.
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAdditive())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());
}

bool Celestial::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    LOG_INFO("Initializing celestial renderer (Vulkan)");

    vkCtx_ = ctx;
    VkDevice device = vkCtx_->getDevice();

    // ------------------------------------------------------------------ shaders
    auto shaders = loadShaderPair(device, "assets/shaders/celestial.vert.spv", "assets/shaders/celestial.frag.spv", "celestial");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // ------------------------------------------------------------------ push constants
    // Layout: mat4(64) + vec4(16) + float*3(12) + pad(4) = 96 bytes
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset     = 0;
    pushRange.size       = sizeof(CelestialPush); // 96 bytes

    // ------------------------------------------------------------------ pipeline layout
    pipelineLayout_ = createPipelineLayout(device, {perFrameLayout}, {pushRange});
    if (pipelineLayout_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create celestial pipeline layout");
        return false;
    }

    // ------------------------------------------------------------------ pipeline
    pipeline_ = buildPipeline(device, vertStage, fragStage);


    if (pipeline_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create celestial pipeline");
        return false;
    }

    // ------------------------------------------------------------------ geometry
    createQuad();

    LOG_INFO("Celestial renderer initialized");
    return true;
}

void Celestial::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    destroy(device, pipeline_);

    auto shaders = loadShaderPair(device, "assets/shaders/celestial.vert.spv", "assets/shaders/celestial.frag.spv", "celestial");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    pipeline_ = buildPipeline(device, vertStage, fragStage);


    if (pipeline_ == VK_NULL_HANDLE) {
        LOG_ERROR("Celestial::recreatePipelines: failed to create pipeline");
    }
}

void Celestial::shutdown() {
    destroyQuad();

    if (vkCtx_) destroyPipeline(vkCtx_->getDevice(), pipeline_, pipelineLayout_);

    vkCtx_ = nullptr;
}

// ---------------------------------------------------------------------------
// Public render entry point
// ---------------------------------------------------------------------------

void Celestial::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet,
                       float timeOfDay,
                       const glm::vec3* sunDir, const glm::vec3* sunColor,
                       float gameTime, float nightFactor) {
    if (!renderingEnabled_ || pipeline_ == VK_NULL_HANDLE) {
        return;
    }

    // Update moon phases from server game time if provided
    if (gameTime >= 0.0f) {
        updatePhasesFromGameTime(gameTime);
    }

    // Bind pipeline and per-frame descriptor set once - reused for all draws
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
        0, 1, &perFrameSet, 0, nullptr);

    // Bind the shared quad buffers
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer_, &offset);
    vkCmdBindIndexBuffer(cmd, indexBuffer_, 0, VK_INDEX_TYPE_UINT32);

    // Draw sun, then moon(s) - each call pushes different constants
    renderSun(cmd, perFrameSet, timeOfDay, sunDir, sunColor);
    renderMoon(cmd, perFrameSet, timeOfDay, nightFactor);
    if (dualMoonMode_) {
        renderBlueChild(cmd, perFrameSet, timeOfDay, nightFactor);
    }
}

// ---------------------------------------------------------------------------
// Private per-body render helpers
// ---------------------------------------------------------------------------

void Celestial::renderSun(VkCommandBuffer cmd, VkDescriptorSet /*perFrameSet*/,
                           float timeOfDay,
                           const glm::vec3* sunDir, const glm::vec3* sunColor) {
    // Sun visible 5:00–19:00
    if (timeOfDay < 5.0f || timeOfDay >= 19.0f) {
        return;
    }

    // Resolve sun direction - prefer opposite of incoming light ray, clamp below horizon
    glm::vec3 lightDir = sunDir ? glm::normalize(*sunDir) : glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 dir = -lightDir;
    if (dir.z < 0.0f) {
        dir = lightDir;
    }

    const float sunDistance = 800.0f;
    glm::vec3 sunPos = dir * sunDistance;

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, sunPos);
    model = glm::scale(model, glm::vec3(95.0f, 95.0f, 1.0f));

    glm::vec3 color = sunColor ? *sunColor : getSunColor(timeOfDay);
    const glm::vec3 warmSun(1.0f, 0.88f, 0.55f);
    color = glm::mix(color, warmSun, 0.52f);
    float intensity = getSunIntensity(timeOfDay) * 0.92f;

    CelestialPush push{};
    push.model          = model;
    push.celestialColor = glm::vec4(color, 1.0f);
    push.intensity      = intensity;
    push.moonPhase      = 0.5f; // unused for sun
    push.animTime       = sunHazeTimer_;

    vkCmdPushConstants(cmd, pipelineLayout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    vkCmdDrawIndexed(cmd, 6, 1, 0, 0, 0);
}

void Celestial::renderMoon(VkCommandBuffer cmd, VkDescriptorSet /*perFrameSet*/,
                            float timeOfDay, float nightFactor) {
    // Moon (White Lady) visible 19:00–5:00
    if (timeOfDay >= 5.0f && timeOfDay < 19.0f) {
        return;
    }
    // Scale by actual sky darkness - the DBC sky can stay daylight-bright
    // well past 19:00, and a full-brightness moon on a blue sky reads as a
    // second sun.
    if (nightFactor < 0.01f) {
        return;
    }

    glm::vec3 moonPos = getMoonPosition(timeOfDay);

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, moonPos);
    model = glm::scale(model, glm::vec3(40.0f, 40.0f, 1.0f));

    glm::vec3 color = glm::vec3(0.8f, 0.85f, 1.0f);

    float intensity = nightFactor;
    if (timeOfDay >= 19.0f && timeOfDay < 21.0f) {
        intensity *= (timeOfDay - 19.0f) / 2.0f; // Fade in
    } else if (timeOfDay >= 3.0f && timeOfDay < 5.0f) {
        intensity *= 1.0f - (timeOfDay - 3.0f) / 2.0f; // Fade out
    }

    CelestialPush push{};
    push.model          = model;
    push.celestialColor = glm::vec4(color, 0.0f);  // w=0 marks moon for the shader
    push.intensity      = intensity;
    push.moonPhase      = whiteLadyPhase_;
    push.animTime       = sunHazeTimer_;

    vkCmdPushConstants(cmd, pipelineLayout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    vkCmdDrawIndexed(cmd, 6, 1, 0, 0, 0);
}

void Celestial::renderBlueChild(VkCommandBuffer cmd, VkDescriptorSet /*perFrameSet*/,
                                 float timeOfDay, float nightFactor) {
    // Blue Child visible 19:00–5:00
    if (timeOfDay >= 5.0f && timeOfDay < 19.0f) {
        return;
    }
    if (nightFactor < 0.01f) {
        return;
    }

    // Offset slightly from White Lady
    glm::vec3 moonPos = getMoonPosition(timeOfDay);
    moonPos.x += 80.0f;
    moonPos.z -= 40.0f;

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, moonPos);
    model = glm::scale(model, glm::vec3(30.0f, 30.0f, 1.0f));

    glm::vec3 color = glm::vec3(0.7f, 0.8f, 1.0f);

    float intensity = nightFactor;
    if (timeOfDay >= 19.0f && timeOfDay < 21.0f) {
        intensity *= (timeOfDay - 19.0f) / 2.0f;
    } else if (timeOfDay >= 3.0f && timeOfDay < 5.0f) {
        intensity *= 1.0f - (timeOfDay - 3.0f) / 2.0f;
    }
    intensity *= 0.7f; // Blue Child is dimmer

    CelestialPush push{};
    push.model          = model;
    push.celestialColor = glm::vec4(color, 0.0f);  // w=0 marks moon for the shader
    push.intensity      = intensity;
    push.moonPhase      = blueChildPhase_;
    push.animTime       = sunHazeTimer_;

    vkCmdPushConstants(cmd, pipelineLayout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    vkCmdDrawIndexed(cmd, 6, 1, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// Position / colour query helpers (identical logic to GL version)
// ---------------------------------------------------------------------------

glm::vec3 Celestial::getSunPosition(float timeOfDay) const {
    float angle = calculateCelestialAngle(timeOfDay, 6.0f, 18.0f);
    const float radius = 800.0f;
    const float height = 600.0f;
    float x = radius * std::cos(angle);
    float z = height * std::sin(angle);
    return glm::vec3(x, 0.0f, z);
}

glm::vec3 Celestial::getMoonPosition(float timeOfDay) const {
    float moonTime = timeOfDay + 12.0f;
    if (moonTime >= 24.0f) moonTime -= 24.0f;
    float angle = calculateCelestialAngle(moonTime, 6.0f, 18.0f);
    const float radius = 800.0f;
    const float height = 600.0f;
    float x = radius * std::cos(angle);
    float z = height * std::sin(angle);
    return glm::vec3(x, 0.0f, z);
}

glm::vec3 Celestial::getSunColor(float timeOfDay) const {
    if (timeOfDay >= 5.0f && timeOfDay < 7.0f) {
        return glm::vec3(1.0f, 0.6f, 0.2f); // Sunrise orange
    }
    if (timeOfDay >= 7.0f && timeOfDay < 9.0f) {
        float t = (timeOfDay - 7.0f) / 2.0f;
        return glm::mix(glm::vec3(1.0f, 0.6f, 0.2f), glm::vec3(1.0f, 1.0f, 0.9f), t);
    }
    if (timeOfDay >= 9.0f && timeOfDay < 16.0f) {
        return glm::vec3(1.0f, 1.0f, 0.9f); // Day yellow-white
    }
    if (timeOfDay >= 16.0f && timeOfDay < 18.0f) {
        float t = (timeOfDay - 16.0f) / 2.0f;
        return glm::mix(glm::vec3(1.0f, 1.0f, 0.9f), glm::vec3(1.0f, 0.5f, 0.1f), t);
    }
    return glm::vec3(1.0f, 0.4f, 0.1f); // Sunset orange
}

float Celestial::getSunIntensity(float timeOfDay) const {
    if (timeOfDay >= 5.0f && timeOfDay < 6.0f) {
        return timeOfDay - 5.0f;          // Fade in
    }
    if (timeOfDay >= 6.0f && timeOfDay < 18.0f) {
        return 1.0f;                       // Full day
    }
    if (timeOfDay >= 18.0f && timeOfDay < 19.0f) {
        return 1.0f - (timeOfDay - 18.0f); // Fade out
    }
    return 0.0f;
}

float Celestial::calculateCelestialAngle(float timeOfDay, float riseTime, float setTime) const {
    float duration = setTime - riseTime;
    float elapsed  = timeOfDay - riseTime;
    float t = elapsed / duration;
    return t * static_cast<float>(M_PI);
}

// ---------------------------------------------------------------------------
// Moon phase helpers
// ---------------------------------------------------------------------------

void Celestial::update(float deltaTime) {
    sunHazeTimer_ += deltaTime;
    // Keep timer in a range where GPU sin() precision is reliable (< ~10000).
    // The noise period repeats at multiples of 1.0 on each axis, so fmod by a
    // large integer preserves visual continuity.
    if (sunHazeTimer_ > 10000.0f) {
        sunHazeTimer_ = std::fmod(sunHazeTimer_, 10000.0f);
    }

    if (!moonPhaseCycling_) {
        return;
    }

    moonPhaseTimer_ += deltaTime;
    whiteLadyPhase_ = std::fmod(moonPhaseTimer_ / MOON_CYCLE_DURATION, 1.0f);

    constexpr float BLUE_CHILD_CYCLE = 210.0f; // Slightly faster: 3.5 minutes
    blueChildPhase_ = std::fmod(moonPhaseTimer_ / BLUE_CHILD_CYCLE, 1.0f);
}
void Celestial::setBlueChildPhase(float phase) {
    blueChildPhase_ = glm::clamp(phase, 0.0f, 1.0f);
}

float Celestial::computePhaseFromGameTime(float gameTime, float cycleDays) const {
    constexpr float SECONDS_PER_GAME_DAY = 1440.0f; // 24 real minutes
    float gameDays = gameTime / SECONDS_PER_GAME_DAY;
    float phase    = std::fmod(gameDays / cycleDays, 1.0f);
    if (phase < 0.0f) phase += 1.0f;
    return phase;
}

void Celestial::updatePhasesFromGameTime(float gameTime) {
    whiteLadyPhase_ = computePhaseFromGameTime(gameTime, WHITE_LADY_CYCLE_DAYS);
    blueChildPhase_ = computePhaseFromGameTime(gameTime, BLUE_CHILD_CYCLE_DAYS);
}

// ---------------------------------------------------------------------------
// GPU buffer management
// ---------------------------------------------------------------------------

void Celestial::createQuad() {
    // Billboard quad centred at origin, vertices: pos(vec3) + uv(vec2)
    float vertices[] = {
        // Position              TexCoord
        -0.5f,  0.5f, 0.0f,    0.0f, 1.0f, // Top-left
         0.5f,  0.5f, 0.0f,    1.0f, 1.0f, // Top-right
         0.5f, -0.5f, 0.0f,    1.0f, 0.0f, // Bottom-right
        -0.5f, -0.5f, 0.0f,    0.0f, 0.0f, // Bottom-left
    };

    uint32_t indices[] = { 0, 1, 2,  0, 2, 3 };

    AllocatedBuffer vbuf = uploadBuffer(*vkCtx_,
        vertices, sizeof(vertices),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    vertexBuffer_ = vbuf.buffer;
    vertexAlloc_  = vbuf.allocation;

    AllocatedBuffer ibuf = uploadBuffer(*vkCtx_,
        indices, sizeof(indices),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    indexBuffer_ = ibuf.buffer;
    indexAlloc_  = ibuf.allocation;
}

void Celestial::destroyQuad() {
    if (!vkCtx_) return;

    VmaAllocator allocator = vkCtx_->getAllocator();

    destroy(allocator, vertexBuffer_, vertexAlloc_);
    destroy(allocator, indexBuffer_, indexAlloc_);
}

} // namespace rendering
} // namespace wowee
