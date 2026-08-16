#include "rendering/lens_flare.hpp"
#include "rendering/camera.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

namespace wowee {
namespace rendering {

LensFlare::LensFlare() {
}

LensFlare::~LensFlare() {
    shutdown();
}

/// Builds the one pipeline this effect draws with, vertex layout and all.
///
/// initialize() and recreatePipelines() described it identically, which is
/// two statements of what a flare vertex is in one file.
void LensFlare::buildPipelines(VkDevice device,
                               const VkPipelineShaderStageCreateInfo& vertStage,
                               const VkPipelineShaderStageCreateInfo& fragStage) {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = 4 * sizeof(float);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription posAttr{};
    posAttr.location = 0;
    posAttr.binding = 0;
    posAttr.format = VK_FORMAT_R32G32_SFLOAT;
    posAttr.offset = 0;

    VkVertexInputAttributeDescription uvAttr{};
    uvAttr.location = 1;
    uvAttr.binding = 0;
    uvAttr.format = VK_FORMAT_R32G32_SFLOAT;
    uvAttr.offset = 2 * sizeof(float);

    // Dynamic viewport and scissor
    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    pipeline = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, {posAttr, uvAttr})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAdditive())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx->getPipelineCache());

}

bool LensFlare::initialize(VkContext* ctx, VkDescriptorSetLayout /*perFrameLayout*/) {
    LOG_INFO("Initializing lens flare system");

    vkCtx = ctx;
    VkDevice device = vkCtx->getDevice();

    // Generate flare elements
    generateFlareElements();

    // Upload static quad vertex buffer (pos2 + uv2, 6 vertices)
    float quadVertices[] = {
        // Pos      UV
        -0.5f, -0.5f, 0.0f, 0.0f,
         0.5f, -0.5f, 1.0f, 0.0f,
         0.5f,  0.5f, 1.0f, 1.0f,
        -0.5f, -0.5f, 0.0f, 0.0f,
         0.5f,  0.5f, 1.0f, 1.0f,
        -0.5f,  0.5f, 0.0f, 1.0f
    };

    AllocatedBuffer vbuf = uploadBuffer(*vkCtx,
        quadVertices,
        sizeof(quadVertices),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    vertexBuffer = vbuf.buffer;
    vertexAlloc  = vbuf.allocation;

    // Load SPIR-V shaders
    auto shaders = loadShaderPair(device, "assets/shaders/lens_flare.vert.spv", "assets/shaders/lens_flare.frag.spv", "lens_flare");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // Push constant range: FlarePushConstants = 32 bytes, used by both vert and frag
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(FlarePushConstants);  // 32 bytes

    // No descriptor set layouts - lens flare only uses push constants
    pipelineLayout = createPipelineLayout(device, {}, {pushRange});
    if (pipelineLayout == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create lens flare pipeline layout");
        return false;
    }

    // Vertex input: pos2 + uv2, stride = 4 * sizeof(float)
    buildPipelines(device, vertStage, fragStage);

    // Shader modules can be freed after pipeline creation

    if (pipeline == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create lens flare pipeline");
        return false;
    }

    LOG_INFO("Lens flare system initialized: ", flareElements.size(), " elements");
    return true;
}

void LensFlare::shutdown() {
    if (vkCtx) {
        destroyParticleResources(vkCtx->getDevice(), vkCtx->getAllocator(),
                                 pipeline, pipelineLayout, vertexBuffer, vertexAlloc);
    }

    vkCtx = nullptr;
}

void LensFlare::recreatePipelines() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    // Destroy old pipeline (NOT layout)
    destroy(device, pipeline);

    auto shaders = loadShaderPair(device, "assets/shaders/lens_flare.vert.spv", "assets/shaders/lens_flare.frag.spv", "lens_flare");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    buildPipelines(device, vertStage, fragStage);
}

void LensFlare::generateFlareElements() {
    flareElements.clear();

    // Main sun glow (at sun position)
    flareElements.push_back({.position = 0.0f, .size = 0.3f, .color = glm::vec3(1.0f, 0.95f, 0.8f), .brightness = 0.8f});

    // Flare ghosts along sun-to-center axis

    // Bright white ghost near sun
    flareElements.push_back({.position = 0.2f, .size = 0.08f, .color = glm::vec3(1.0f, 1.0f, 1.0f), .brightness = 0.5f});

    // Blue-tinted ghost
    flareElements.push_back({.position = 0.4f, .size = 0.15f, .color = glm::vec3(0.4f, 0.55f, 0.9f), .brightness = 0.35f});

    // Small bright spot
    flareElements.push_back({.position = 0.6f, .size = 0.05f, .color = glm::vec3(1.0f, 0.8f, 0.6f), .brightness = 0.5f});

    // Warm amber ghost (replaced oversaturated green)
    flareElements.push_back({.position = 0.8f, .size = 0.10f, .color = glm::vec3(0.9f, 0.75f, 0.5f), .brightness = 0.2f});

    // Large halo on opposite side
    flareElements.push_back({.position = -0.5f, .size = 0.22f, .color = glm::vec3(1.0f, 0.8f, 0.5f), .brightness = 0.15f});

    // Faint blue ghost far from sun
    flareElements.push_back({.position = -0.8f, .size = 0.08f, .color = glm::vec3(0.6f, 0.5f, 0.9f), .brightness = 0.15f});

    // Small warm ghost
    flareElements.push_back({.position = -1.2f, .size = 0.05f, .color = glm::vec3(1.0f, 0.6f, 0.4f), .brightness = 0.2f});
}

glm::vec2 LensFlare::worldToScreen(const Camera& camera, const glm::vec3& worldPos) const {
    // Transform to clip space
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();
    glm::mat4 viewProj = projection * view;

    glm::vec4 clipPos = viewProj * glm::vec4(worldPos, 1.0f);

    // Perspective divide
    if (clipPos.w > 0.0f) {
        glm::vec2 ndc = glm::vec2(clipPos.x / clipPos.w, clipPos.y / clipPos.w);
        return ndc;
    }

    // Behind camera
    return glm::vec2(10.0f, 10.0f);  // Off-screen
}

float LensFlare::calculateSunVisibility(const Camera& camera, const glm::vec3& sunPosition) const {
    // Get sun position in screen space
    glm::vec2 sunScreen = worldToScreen(camera, sunPosition);

    // Check if sun is behind camera
    glm::vec3 camPos = camera.getPosition();
    glm::vec3 camForward = camera.getForward();
    glm::vec3 toSun = glm::normalize(sunPosition - camPos);
    float dotProduct = glm::dot(camForward, toSun);

    if (dotProduct < 0.0f) {
        return 0.0f;  // Sun is behind camera
    }

    // Check if sun is outside screen bounds (with some margin)
    if (std::abs(sunScreen.x) > 1.5f || std::abs(sunScreen.y) > 1.5f) {
        return 0.0f;
    }

    // Fade based on angle (stronger when looking directly at sun)
    float angleFactor = glm::smoothstep(0.3f, 1.0f, dotProduct);

    // Fade at screen edges
    float edgeFade = 1.0f;
    if (std::abs(sunScreen.x) > 0.8f) {
        edgeFade *= glm::smoothstep(1.2f, 0.8f, std::abs(sunScreen.x));
    }
    if (std::abs(sunScreen.y) > 0.8f) {
        edgeFade *= glm::smoothstep(1.2f, 0.8f, std::abs(sunScreen.y));
    }

    return angleFactor * edgeFade;
}

void LensFlare::render(VkCommandBuffer cmd, const Camera& camera, const glm::vec3& sunPosition,
                       float timeOfDay, float fogDensity, float cloudDensity,
                       float weatherIntensity) {
    if (!enabled || pipeline == VK_NULL_HANDLE) {
        return;
    }

    // Only render lens flare during daytime (when sun is visible)
    if (timeOfDay < 5.0f || timeOfDay > 19.0f) {
        return;
    }

    // Sun billboard rendering is sky-locked (view translation removed), so anchor
    // flare projection to camera position along sun direction to avoid parallax drift.
    glm::vec3 sunDir = sunPosition;
    float sunDirLenSq = glm::dot(sunDir, sunDir);
    if (sunDirLenSq < 1e-8f) {
        return;
    }
    sunDir *= glm::inversesqrt(sunDirLenSq);
    glm::vec3 anchoredSunPos = camera.getPosition() + sunDir * 800.0f;

    // Calculate sun visibility
    float visibility = calculateSunVisibility(camera, anchoredSunPos);
    if (visibility < 0.01f) {
        return;
    }

    // Sun height attenuation - flare weakens when sun is near horizon (sunrise/sunset)
    float sunHeight = sunDir.z;  // z = up in render space; 0 = horizon, 1 = zenith
    float heightFactor = glm::smoothstep(-0.05f, 0.25f, sunHeight);

    // Atmospheric attenuation - fog, clouds, and weather reduce lens flare
    float atmosphericFactor = heightFactor;
    atmosphericFactor *= (1.0f - glm::clamp(fogDensity * 0.8f, 0.0f, 0.9f));       // Heavy fog nearly kills flare
    atmosphericFactor *= (1.0f - glm::clamp(cloudDensity * 0.6f, 0.0f, 0.7f));     // Clouds attenuate
    atmosphericFactor *= (1.0f - glm::clamp(weatherIntensity * 0.9f, 0.0f, 0.95f)); // Rain/snow heavily attenuates

    if (atmosphericFactor < 0.01f) {
        return;
    }

    // Get sun screen position
    glm::vec2 sunScreen = worldToScreen(camera, anchoredSunPos);
    glm::vec2 screenCenter(0.0f, 0.0f);

    // Vector from sun to screen center
    glm::vec2 sunToCenter = screenCenter - sunScreen;

    float aspectRatio = camera.getAspectRatio();

    // Bind pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    // Bind vertex buffer
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);

    // Warm tint at sunrise/sunset - shift flare color toward orange/amber when sun is low
    float warmTint = 1.0f - glm::smoothstep(0.05f, 0.35f, sunHeight);

    // Render each flare element
    for (const auto& element : flareElements) {
        // Calculate position along sun-to-center axis
        glm::vec2 position = sunScreen + sunToCenter * element.position;

        // Apply visibility, intensity, and atmospheric attenuation
        float brightness = element.brightness * visibility * intensityMultiplier * atmosphericFactor;

        // Apply warm sunset/sunrise color shift
        glm::vec3 tintedColor = element.color;
        if (warmTint > 0.01f) {
            glm::vec3 warmColor(1.0f, 0.6f, 0.25f);  // amber/orange
            tintedColor = glm::mix(tintedColor, warmColor, warmTint * 0.5f);
        }

        // Set push constants
        FlarePushConstants push{};
        push.position = position;
        push.size = element.size;
        push.aspectRatio = aspectRatio;
        push.colorBrightness = glm::vec4(tintedColor, brightness);

        vkCmdPushConstants(cmd, pipelineLayout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(push), &push);

        // Draw quad
        vkCmdDraw(cmd, VERTICES_PER_QUAD, 1, 0, 0);
    }
}

void LensFlare::setIntensity(float intensity) {
    this->intensityMultiplier = glm::clamp(intensity, 0.0f, 2.0f);
}

} // namespace rendering
} // namespace wowee
