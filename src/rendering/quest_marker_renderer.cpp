#include "rendering/quest_marker_renderer.hpp"
#include "core/coordinates.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_utils.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <SDL2/SDL.h>
#include <cmath>
#include <cstring>

namespace wowee { namespace rendering {

// Push constant layout matching quest_marker.vert.glsl / quest_marker.frag.glsl
struct QuestMarkerPushConstants {
    glm::mat4 model;   // 64 bytes, used by vertex shader
    float alpha;        // 4 bytes, used by fragment shader
    float grayscale;    // 4 bytes: 0=colour, 1=desaturated (trivial quests)
};

QuestMarkerRenderer::QuestMarkerRenderer() {
}

QuestMarkerRenderer::~QuestMarkerRenderer() {
    shutdown();
}

bool QuestMarkerRenderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
    pipeline::AssetManager* assetManager)
{
    if (!ctx || !assetManager) {
        LOG_WARNING("QuestMarkerRenderer: Missing VkContext or AssetManager");
        return false;
    }

    // Re-initialisation is a supported path -- the renderer calls this again
    // when the asset manager arrives and after a device rebuild -- and every
    // resource below is created unconditionally. Without releasing the
    // previous set first, each re-init abandoned three textures, the pipeline,
    // its layout, the descriptor pool and the quad vertex buffer, none of
    // which any later shutdown could reach.
    if (vkCtx_) {
        LOG_INFO("QuestMarkerRenderer: re-initialising, releasing the previous resources");
        shutdown();
    }

    LOG_INFO("QuestMarkerRenderer: Initializing...");
    vkCtx_ = ctx;
    VkDevice device = vkCtx_->getDevice();

    // --- Create material descriptor set layout (set 1: combined image sampler) ---
    createDescriptorResources();

    // --- Load shaders ---
    auto shaders = loadShaderPair(device, "assets/shaders/quest_marker.vert.spv",
                                  "assets/shaders/quest_marker.frag.spv", "quest_marker");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // --- Push constant range: mat4 model (64) + float alpha (4) = 68 bytes ---
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(QuestMarkerPushConstants);

    // --- Pipeline layout: set 0 = per-frame, set 1 = material texture ---
    pipelineLayout_ = createPipelineLayout(device,
        {perFrameLayout, materialSetLayout_}, {pushRange});
    if (pipelineLayout_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create quest marker pipeline layout");
        return false;
    }

    // --- Vertex input: vec3 pos (offset 0) + vec2 uv (offset 12), stride 20 ---
    VkVertexInputBindingDescription binding = tightVertexBinding(5 * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = positionPlusUvAttrs();

    // Dynamic viewport and scissor
    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    // --- Build pipeline: alpha blending, no cull, depth test on / write off ---
    pipeline_ = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS) // depth test on, write off
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());


    if (pipeline_ == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create quest marker pipeline");
        return false;
    }

    // --- Upload quad vertex buffer ---
    createQuad();

    // --- Load BLP textures ---
    loadTextures(assetManager);

    LOG_INFO("QuestMarkerRenderer: Initialization complete");
    return true;
}

void QuestMarkerRenderer::shutdown() {
    if (!vkCtx_) return;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator allocator = vkCtx_->getAllocator();

    // Wait for device idle before destroying resources
    vkDeviceWaitIdle(device);

    // Destroy textures
    for (int i = 0; i < 3; ++i) {
        textures_[i].destroy(device, allocator);
        texDescSets_[i] = VK_NULL_HANDLE;
    }

    // Destroy descriptor pool (frees all descriptor sets allocated from it)
    destroy(device, descriptorPool_);

    // Destroy descriptor set layout
    destroy(device, materialSetLayout_);

    // Destroy pipeline
    destroy(device, pipeline_);
    destroy(device, pipelineLayout_);

    // Destroy quad vertex buffer
    destroy(allocator, quadVB_, quadVBAlloc_);

    markers_.clear();
    vkCtx_ = nullptr;
}

void QuestMarkerRenderer::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old pipeline (NOT layout)
    destroy(device, pipeline_);

    auto shaders = loadShaderPair(device, "assets/shaders/quest_marker.vert.spv", "assets/shaders/quest_marker.frag.spv", "quest_marker");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    VkVertexInputBindingDescription binding = tightVertexBinding(5 * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = positionPlusUvAttrs();

    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    pipeline_ = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());

}

void QuestMarkerRenderer::createDescriptorResources() {
    VkDevice device = vkCtx_->getDevice();

    // Material set layout: binding 0 = combined image sampler (fragment stage)
    VkDescriptorSetLayoutBinding samplerBinding{};
    samplerBinding.binding = 0;
    samplerBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    samplerBinding.descriptorCount = 1;
    samplerBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    materialSetLayout_ = createDescriptorSetLayout(device, {samplerBinding});

    // Descriptor pool: 3 combined image samplers (one per marker type)
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = 3;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = 3;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS) {
        LOG_ERROR("Failed to create quest marker descriptor pool");
        return;
    }

    // Allocate 3 descriptor sets (one per texture)
    VkDescriptorSetLayout layouts[3] = {materialSetLayout_, materialSetLayout_, materialSetLayout_};

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 3;
    allocInfo.pSetLayouts = layouts;

    if (vkAllocateDescriptorSets(device, &allocInfo, texDescSets_) != VK_SUCCESS) {
        LOG_ERROR("Failed to allocate quest marker descriptor sets");
    }
}

void QuestMarkerRenderer::createQuad() {
    // Billboard quad vertices (centered, 1 unit size) - 6 vertices for 2 triangles
    float vertices[] = {
        -0.5f, -0.5f, 0.0f,  0.0f, 1.0f,  // bottom-left
         0.5f, -0.5f, 0.0f,  1.0f, 1.0f,  // bottom-right
         0.5f,  0.5f, 0.0f,  1.0f, 0.0f,  // top-right
        -0.5f,  0.5f, 0.0f,  0.0f, 0.0f,  // top-left
        -0.5f, -0.5f, 0.0f,  0.0f, 1.0f,  // bottom-left
         0.5f,  0.5f, 0.0f,  1.0f, 0.0f   // top-right
    };

    AllocatedBuffer vbuf = uploadBuffer(*vkCtx_,
        vertices, sizeof(vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    quadVB_ = vbuf.buffer;
    quadVBAlloc_ = vbuf.allocation;
}

void QuestMarkerRenderer::loadTextures(pipeline::AssetManager* assetManager) {
    const char* paths[3] = {
        "Interface\\GossipFrame\\AvailableQuestIcon.blp",
        "Interface\\GossipFrame\\ActiveQuestIcon.blp",
        "Interface\\GossipFrame\\IncompleteQuestIcon.blp"
    };
    // The grey question mark is 2.x's art and a 1.12 client does not have it -
    // its GossipFrame holds ActiveQuestIcon and AvailableQuestIcon and nothing
    // else. Falling back to the active mark draws a quest in progress the way
    // that client draws one, rather than reporting a missing file twice and
    // leaving the head bare.
    const char* fallback[3] = {
        nullptr, nullptr, "Interface\\GossipFrame\\ActiveQuestIcon.blp"
    };

    VkDevice device = vkCtx_->getDevice();

    // The sets are allocated before this runs. Held aside and published only
    // once each has been written: an allocated set that was never written is as
    // undefined to bind as one holding a null image view, and the render pass
    // below binds by marker type without looking - so a texture that failed to
    // load was already being sampled from an unwritten set.
    VkDescriptorSet allocated[3];
    for (int i = 0; i < 3; ++i) {
        allocated[i] = texDescSets_[i];
        texDescSets_[i] = VK_NULL_HANDLE;
    }

    for (int i = 0; i < 3; ++i) {
        if (!allocated[i]) continue;

        pipeline::BLPImage blp = assetManager->loadTexture(paths[i]);
        if (!blp.isValid() && fallback[i]) {
            blp = assetManager->loadTexture(fallback[i]);
            if (blp.isValid()) {
                LOG_WARNING("Quest marker: ", paths[i], " is not in this client's "
                            "data - drawing ", fallback[i], " instead");
            }
        }
        if (!blp.isValid()) {
            LOG_WARNING("Failed to load quest marker texture: ", paths[i]);
            continue;
        }

        // Upload RGBA data to VkTexture
        if (!textures_[i].upload(*vkCtx_, blp.data.data(), blp.width, blp.height,
                VK_FORMAT_R8G8B8A8_UNORM, true)) {
            LOG_WARNING("Failed to upload quest marker texture to GPU: ", paths[i]);
            continue;
        }

        // Create sampler with clamp-to-edge
        textures_[i].createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
            VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);

        // Uploaded is not sampleable: createSampler answers nothing and the
        // image view is created without being checked, and a descriptor holding
        // a null view costs the device rather than the marker. See #123.
        if (!textures_[i].isValid()) {
            LOG_WARNING("Quest marker texture is not sampleable: ", paths[i]);
            continue;
        }

        // Write descriptor set for this texture
        VkDescriptorImageInfo imgInfo = textures_[i].descriptorInfo();

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = allocated[i];
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imgInfo;

        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        texDescSets_[i] = allocated[i];

        LOG_INFO("Loaded quest marker texture: ", paths[i]);
    }
}

void QuestMarkerRenderer::setMarker(uint64_t guid, const glm::vec3& position, int markerType,
                                    float boundingHeight, float grayscale) {
    markers_[guid] = {.position = position, .type = markerType, .boundingHeight = boundingHeight, .grayscale = grayscale};
}
void QuestMarkerRenderer::clear() {
    markers_.clear();
}

void QuestMarkerRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (markers_.empty() || pipeline_ == VK_NULL_HANDLE || quadVB_ == VK_NULL_HANDLE) return;

    // WoW-style quest marker tuning parameters
    constexpr float BASE_SIZE = 0.65f;          // Base world-space size
    constexpr float HEIGHT_OFFSET = 1.1f;       // Height above NPC bounds
    constexpr float BOB_AMPLITUDE = 0.10f;      // Bob animation amplitude
    constexpr float BOB_FREQUENCY = 1.25f;      // Bob frequency (Hz)
    constexpr float MIN_DIST = 4.0f;            // Near clamp
    constexpr float MAX_DIST = 90.0f;           // Far fade-out start
    constexpr float FADE_RANGE = 25.0f;         // Fade-out range
    constexpr float CULL_DIST = MAX_DIST + FADE_RANGE;
    constexpr float CULL_DIST_SQ = CULL_DIST * CULL_DIST;

    // Get time for bob animation
    float timeSeconds = SDL_GetTicks() / 1000.0f;

    glm::mat4 view = camera.getViewMatrix();
    glm::vec3 cameraPos = camera.getPosition();

    // Extract frustum planes for visibility testing
    Frustum frustum;
    frustum.extractFromMatrix(camera.getViewProjectionMatrix());

    // Get camera right and up vectors for billboarding
    glm::vec3 cameraRight = glm::vec3(view[0][0], view[1][0], view[2][0]);
    glm::vec3 cameraUp = glm::vec3(view[0][1], view[1][1], view[2][1]);
    const glm::vec3 cameraForward = glm::cross(cameraRight, cameraUp);

    // Bind pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);

    // Bind per-frame descriptor set (set 0)
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
        0, 1, &perFrameSet, 0, nullptr);

    // Bind quad vertex buffer
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &quadVB_, &offset);

    for (const auto& [guid, marker] : markers_) {
        if (marker.type < 0 || marker.type > 2) continue;
        if (!textures_[marker.type].isValid()) continue;

        // Calculate distance for LOD and culling
        glm::vec3 toCamera = cameraPos - marker.position;
        float distSq = glm::dot(toCamera, toCamera);
        if (distSq > CULL_DIST_SQ) continue;

        // Frustum cull quest markers (small sphere for icon)
        constexpr float markerCullRadius = 0.5f;
        if (!frustum.intersectsSphere(marker.position, markerCullRadius)) continue;

        float dist = std::sqrt(distSq);

        // Calculate fade alpha
        float fadeAlpha = 1.0f;
        if (dist > MAX_DIST) {
            float t = glm::clamp((dist - MAX_DIST) / FADE_RANGE, 0.0f, 1.0f);
            t = t * t * (3.0f - 2.0f * t); // Smoothstep
            fadeAlpha = 1.0f - t;
        }
        if (fadeAlpha <= 0.001f) continue; // Cull if fully faded

        // Distance-based scaling (mild compensation for readability)
        float distScale = 1.0f;
        if (dist > MIN_DIST) {
            float t = glm::clamp((dist - 5.0f) / 55.0f, 0.0f, 1.0f);
            distScale = 1.0f + 0.35f * t;
        }
        float size = BASE_SIZE * distScale;
        size = glm::clamp(size, BASE_SIZE * 0.9f, BASE_SIZE * 1.6f);

        // Bob animation
        float bob = std::sin(timeSeconds * BOB_FREQUENCY * core::coords::TWO_PI) * BOB_AMPLITUDE;

        // Position marker above NPC with bob
        glm::vec3 markerPos = marker.position;
        markerPos.z += marker.boundingHeight + HEIGHT_OFFSET + bob;

        // Build billboard matrix (camera-facing quad)
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, markerPos);

        // Billboard: align quad to face camera
        model[0] = glm::vec4(cameraRight * size, 0.0f);
        model[1] = glm::vec4(cameraUp * size, 0.0f);
        model[2] = glm::vec4(cameraForward, 0.0f);

        // Bind material descriptor set (set 1) for this marker's texture
        if (marker.type < 0 || marker.type >= 3 || !texDescSets_[marker.type]) continue;
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
            1, 1, &texDescSets_[marker.type], 0, nullptr);

        // Push constants: model matrix + alpha + grayscale tint
        QuestMarkerPushConstants push{};
        push.model = model;
        push.alpha = fadeAlpha;
        push.grayscale = marker.grayscale;

        vkCmdPushConstants(cmd, pipelineLayout_,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(push), &push);

        // Draw the quad (6 vertices, 2 triangles)
        vkCmdDraw(cmd, 6, 1, 0, 0);
    }
}

}} // namespace wowee::rendering
