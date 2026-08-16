#include "rendering/starfield.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>
#include <cmath>
#include <random>
#include <vector>

namespace wowee {
namespace rendering {

// Day/night cycle thresholds (hours, 24h clock) for star visibility.
// Stars fade in over 2 hours at dusk, stay full during night, fade out at dawn.
static constexpr float kDuskStart = 18.0f;  // stars begin fading in
static constexpr float kNightStart = 20.0f; // full star visibility
static constexpr float kDawnStart = 4.0f;   // stars begin fading out
static constexpr float kDawnEnd = 6.0f;     // stars fully gone
static constexpr float kFadeDuration = 2.0f;

// One float more than positionPlusTwoFloatsAttrs(), which mount dust, swim
// effects and the charge effect also use - hence a local list rather than a
// fourth caller of a changed shared one.
static constexpr uint32_t kStarVertexFloats = 6;

static std::vector<VkVertexInputAttributeDescription> starVertexAttrs() {
    return {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                 // position
        {1, 0, VK_FORMAT_R32_SFLOAT,       sizeof(float) * 3},  // brightness
        {2, 0, VK_FORMAT_R32_SFLOAT,       sizeof(float) * 4},  // twinkle phase
        {3, 0, VK_FORMAT_R32_SFLOAT,       sizeof(float) * 5},  // colour temperature
    };
}

StarField::StarField() = default;

StarField::~StarField() {
    shutdown();
}

bool StarField::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    LOG_INFO("Initializing star field");

    vkCtx = ctx;
    VkDevice device = vkCtx->getDevice();

    // Load SPIR-V shaders
    auto shaders = loadShaderPair(device, "assets/shaders/starfield.vert.spv", "assets/shaders/starfield.frag.spv", "starfield");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // Push constants: float time + float intensity = 8 bytes
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(float) * 3;  // time, intensity, viewportHeight

    // Pipeline layout: set 0 = per-frame UBO, push constants
    pipelineLayout = createPipelineLayout(device, {perFrameLayout}, {pushRange});
    if (pipelineLayout == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create starfield pipeline layout");
        return false;
    }

    // Vertex input: binding 0, stride = kStarVertexFloats * sizeof(float)
    VkVertexInputBindingDescription binding =
        tightVertexBinding(kStarVertexFloats * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = starVertexAttrs();

    pipeline = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)  // depth test, no write (stars behind sky)
        .setColorBlendAttachment(PipelineBuilder::blendAdditive())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .build(device, vkCtx->getPipelineCache());


    if (pipeline == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create starfield pipeline");
        return false;
    }

    // Generate star positions and upload to GPU
    generateStars();
    createStarBuffers();

    LOG_INFO("Star field initialized: ", starCount, " stars");
    return true;
}

void StarField::recreatePipelines() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    destroy(device, pipeline);

    auto shaders = loadShaderPair(device, "assets/shaders/starfield.vert.spv", "assets/shaders/starfield.frag.spv", "starfield");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // Vertex input (same as initialize)
    VkVertexInputBindingDescription binding =
        tightVertexBinding(kStarVertexFloats * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = starVertexAttrs();

    pipeline = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAdditive())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .build(device, vkCtx->getPipelineCache());


    if (pipeline == VK_NULL_HANDLE) {
        LOG_ERROR("StarField::recreatePipelines: failed to create pipeline");
    }
}

void StarField::shutdown() {
    destroyStarBuffers();

    if (vkCtx) destroyPipeline(vkCtx->getDevice(), pipeline, pipelineLayout);

    vkCtx = nullptr;
    stars.clear();
}

void StarField::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet,
                       float timeOfDay, float cloudDensity, float fogDensity) {
    if (!renderingEnabled || pipeline == VK_NULL_HANDLE || vertexBuffer == VK_NULL_HANDLE
        || stars.empty()) {
        return;
    }

    // Compute intensity from time of day then attenuate for clouds/fog
    float intensity = getStarIntensity(timeOfDay);
    intensity *= (1.0f - glm::clamp(cloudDensity * 0.7f, 0.0f, 1.0f));
    intensity *= (1.0f - glm::clamp(fogDensity * 0.3f, 0.0f, 1.0f));

    if (intensity <= 0.01f) {
        return;
    }

    // Push constants: time and intensity
    struct StarPushConstants {
        float time;
        float intensity;
        float viewportHeight;
    };
    // The vertex shader sizes each point by angle rather than by a pixel count,
    // so it needs the height it is drawing into.
    StarPushConstants push{.time = twinkleTime, .intensity = intensity,
        .viewportHeight = static_cast<float>(vkCtx->getSwapchainExtent().height)};

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    // Bind per-frame descriptor set (set 0 - camera UBO with view/projection)
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
        0, 1, &perFrameSet, 0, nullptr);

    vkCmdPushConstants(cmd, pipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    // Bind vertex buffer
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);

    // Draw all stars as individual points
    vkCmdDraw(cmd, static_cast<uint32_t>(starCount), 1, 0, 0);
}

void StarField::update(float deltaTime) {
    twinkleTime += deltaTime;
}

void StarField::generateStars() {
    stars.clear();
    stars.reserve(starCount);

    // A fixed seed, because the night sky is the same sky. std::random_device
    // drew a new one every time the field was regenerated, so the constellations
    // were never twice the same.
    std::mt19937 gen(0x57415245u);
    std::uniform_real_distribution<float> unitDist(0.0f, 1.0f);
    std::uniform_real_distribution<float> thetaDist(0.0f, 2.0f * M_PI);  // 0-360 deg
    std::uniform_real_distribution<float> twinkleDist(0.0f, 2.0f * M_PI);

    const float radius = 900.0f;  // Slightly larger than skybox

    for (int i = 0; i < starCount; i++) {
        Star star;

        // Sampling the polar angle uniformly does not spread points evenly over
        // a sphere: the area of a band shrinks with sin(phi), so a uniform phi
        // piles them up at the pole. It put a visible clump of stars directly
        // overhead. Uniform in cos(phi) is the distribution that is actually
        // even, and costs one fewer trig call.
        float cosPhi = unitDist(gen);            // upper hemisphere, 0-90 deg
        float sinPhi = std::sqrt(1.0f - cosPhi * cosPhi);
        float theta  = thetaDist(gen);           // Azimuth angle

        float x = radius * sinPhi * std::cos(theta);
        float y = radius * sinPhi * std::sin(theta);
        float z = radius * cosPhi;

        star.position = glm::vec3(x, y, z);

        // Real fields are mostly faint with a few bright ones, not an even
        // spread from 0.3 to 1.0. The exponent is what gives a sky depth
        // rather than a uniform sprinkle of near-identical dots.
        float u = unitDist(gen);
        star.brightness = 0.14f + 0.86f * std::pow(u, 2.4f);

        star.twinklePhase = twinkleDist(gen);

        // Colour temperature, biased to white: two draws averaged, so the
        // extremes of the ramp stay rare the way they are overhead.
        star.colorTemp = (unitDist(gen) + unitDist(gen)) * 0.5f;

        stars.push_back(star);
    }

    LOG_DEBUG("Generated ", stars.size(), " stars");
}

void StarField::createStarBuffers() {
    // Interleaved: pos.x, pos.y, pos.z, brightness, twinklePhase, colorTemp
    std::vector<float> vertexData;
    vertexData.reserve(stars.size() * kStarVertexFloats);

    for (const auto& star : stars) {
        vertexData.push_back(star.position.x);
        vertexData.push_back(star.position.y);
        vertexData.push_back(star.position.z);
        vertexData.push_back(star.brightness);
        vertexData.push_back(star.twinklePhase);
        vertexData.push_back(star.colorTemp);
    }

    VkDeviceSize bufferSize = vertexData.size() * sizeof(float);

    // Upload via staging buffer to GPU-local memory
    AllocatedBuffer gpuBuf = uploadBuffer(*vkCtx, vertexData.data(), bufferSize,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

    vertexBuffer = gpuBuf.buffer;
    vertexAlloc  = gpuBuf.allocation;
}

void StarField::destroyStarBuffers() {
    if (vkCtx && vertexBuffer != VK_NULL_HANDLE) {
        vmaDestroyBuffer(vkCtx->getAllocator(), vertexBuffer, vertexAlloc);
        vertexBuffer = VK_NULL_HANDLE;
        vertexAlloc  = VK_NULL_HANDLE;
    }
}

float StarField::getStarIntensity(float timeOfDay) const {
    // Full night
    if (timeOfDay >= kNightStart || timeOfDay < kDawnStart) {
        return 1.0f;
    }
    // Fade in at dusk
    if (timeOfDay >= kDuskStart) {
        return (timeOfDay - kDuskStart) / kFadeDuration;
    }
    // Fade out at dawn
    if (timeOfDay < kDawnEnd) {
        return 1.0f - (timeOfDay - kDawnStart) / kFadeDuration;
    }
    // Daytime: no stars
    return 0.0f;
}

} // namespace rendering
} // namespace wowee
