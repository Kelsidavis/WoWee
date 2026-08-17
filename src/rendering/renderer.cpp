#include "rendering/renderer.hpp"
#include "addons/lua_api_registrations.hpp"
#include "core/env_flag.hpp"
#include "rendering/sky_params_from_lighting.hpp"
#include "core/coordinates.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "pipeline/custom_zone_discovery.hpp"
#include "rendering/performance_hud.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/weather.hpp"
#include "rendering/lightning.hpp"
#include "rendering/lighting_manager.hpp"
#include "core/profiler.hpp"
#include "core/thread_pool.hpp"
#include "rendering/sky_system.hpp"
#include "rendering/swim_effects.hpp"
#include "rendering/mount_dust.hpp"
#include "rendering/charge_effect.hpp"
#include "rendering/levelup_effect.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/grass_renderer.hpp"
#include "rendering/hiz_system.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "rendering/quest_marker_renderer.hpp"
#include "rendering/footprint_renderer.hpp"
#include "game/game_handler.hpp"
#include "pipeline/m2_loader.hpp"
#include <algorithm>
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/application.hpp"
#include "core/window.hpp"
#include "core/logger.hpp"
#include "game/world.hpp"
#include "game/zone_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/amd_fsr3_runtime.hpp"
#include "rendering/spell_visual_system.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/render_graph.hpp"
#include "rendering/overlay_system.hpp"
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cctype>
#include <cmath>
#include <chrono>
#include <filesystem>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdlib>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <future>
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace wowee {
namespace rendering {



Renderer::Renderer() = default;
Renderer::~Renderer() = default;

bool Renderer::createPerFrameResources() {
    VkDevice device = vkCtx->getDevice();

    // --- Create per-frame shadow depth images (one per in-flight frame) ---
    // Each frame slot has its own depth image so that frame N's shadow read and
    // frame N+1's shadow write cannot race on the same image.
    VkImageCreateInfo imgCI{};
    imgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgCI.imageType = VK_IMAGE_TYPE_2D;
    imgCI.format = VK_FORMAT_D32_SFLOAT;
    imgCI.extent = {.width = SHADOW_MAP_SIZE, .height = SHADOW_MAP_SIZE, .depth = 1};
    imgCI.mipLevels = 1;
    imgCI.arrayLayers = 1;
    imgCI.samples = VK_SAMPLE_COUNT_1_BIT;
    imgCI.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgCI.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    VmaAllocationCreateInfo imgAllocCI{};
    imgAllocCI.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        if (vmaCreateImage(vkCtx->getAllocator(), &imgCI, &imgAllocCI,
                &shadowDepthImage[i], &shadowDepthAlloc[i], nullptr) != VK_SUCCESS) {
            LOG_ERROR("Failed to create shadow depth image [", i, "]");
            return false;
        }
        shadowDepthLayout_[i] = VK_IMAGE_LAYOUT_UNDEFINED;
    }

    // --- Create per-frame shadow depth image views ---
    VkImageViewCreateInfo viewCI{};
    viewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewCI.format = VK_FORMAT_D32_SFLOAT;
    viewCI.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        viewCI.image = shadowDepthImage[i];
        if (vkCreateImageView(device, &viewCI, nullptr, &shadowDepthView[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create shadow depth image view [", i, "]");
            return false;
        }
    }

    // --- Create shadow sampler (shared - read-only, no per-frame needed) ---
    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    sampCI.compareEnable = VK_TRUE;
    sampCI.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    shadowSampler = vkCtx->getOrCreateSampler(sampCI);
    if (shadowSampler == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create shadow sampler");
        return false;
    }

    // --- Create shadow render pass (depth-only) ---
    VkAttachmentDescription depthAtt{};
    depthAtt.format = VK_FORMAT_D32_SFLOAT;
    depthAtt.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAtt.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAtt.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAtt.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAtt.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAtt.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthRef{};
    depthRef.attachment = 0;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    dep.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo rpCI{};
    rpCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpCI.attachmentCount = 1;
    rpCI.pAttachments = &depthAtt;
    rpCI.subpassCount = 1;
    rpCI.pSubpasses = &subpass;
    rpCI.dependencyCount = 1;
    rpCI.pDependencies = &dep;
    if (vkCreateRenderPass(device, &rpCI, nullptr, &shadowRenderPass) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow render pass");
        return false;
    }

    // --- Create per-frame shadow framebuffers ---
    VkFramebufferCreateInfo fbCI{};
    fbCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbCI.renderPass = shadowRenderPass;
    fbCI.attachmentCount = 1;
    fbCI.width = SHADOW_MAP_SIZE;
    fbCI.height = SHADOW_MAP_SIZE;
    fbCI.layers = 1;
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        fbCI.pAttachments = &shadowDepthView[i];
        if (vkCreateFramebuffer(device, &fbCI, nullptr, &shadowFramebuffer[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create shadow framebuffer [", i, "]");
            return false;
        }
    }

    // --- Create descriptor set layout for set 0 (per-frame UBO + shadow sampler) ---
    VkDescriptorSetLayoutBinding bindings[2]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    // Immutable, because this one compares. Portability implementations may
    // report VkPhysicalDevicePortabilitySubsetFeaturesKHR::mutableComparisonSamplers
    // as false -- MoltenVK does -- and then a sampler with compareEnable set is
    // only legal here, baked into the layout, rather than written into the
    // descriptor per frame. shadowSampler is created above this point.
    bindings[1].pImmutableSamplers = &shadowSampler;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &perFrameSetLayout) != VK_SUCCESS) {
        LOG_ERROR("Failed to create per-frame descriptor set layout");
        return false;
    }

    // --- Create descriptor pool for UBO + image sampler (normal frames + reflection) ---
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = MAX_FRAMES * 2; // normal frames + reflection frames
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = MAX_FRAMES * 2;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = MAX_FRAMES * 2; // normal frames + reflection frames
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &sceneDescriptorPool) != VK_SUCCESS) {
        LOG_ERROR("Failed to create scene descriptor pool");
        return false;
    }

    // --- Create per-frame UBOs and descriptor sets ---
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        // Create mapped UBO
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = sizeof(GPUPerFrameData);
        bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo mapInfo{};
        if (vmaCreateBuffer(vkCtx->getAllocator(), &bufInfo, &allocInfo,
                &perFrameUBOs[i], &perFrameUBOAllocs[i], &mapInfo) != VK_SUCCESS) {
            LOG_ERROR("Failed to create per-frame UBO ", i);
            return false;
        }
        perFrameUBOMapped[i] = mapInfo.pMappedData;

        // Allocate descriptor set
        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = sceneDescriptorPool;
        setAlloc.descriptorSetCount = 1;
        setAlloc.pSetLayouts = &perFrameSetLayout;

        if (vkAllocateDescriptorSets(device, &setAlloc, &perFrameDescSets[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to allocate per-frame descriptor set ", i);
            return false;
        }

        // Write binding 0 (UBO) and binding 1 (shadow sampler)
        VkDescriptorBufferInfo descBuf{};
        descBuf.buffer = perFrameUBOs[i];
        descBuf.offset = 0;
        descBuf.range = sizeof(GPUPerFrameData);

        VkDescriptorImageInfo shadowImgInfo{};
        // sampler is ignored: binding 1 declares it immutable in the layout.
        shadowImgInfo.imageView = shadowDepthView[i];
        shadowImgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = perFrameDescSets[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].pBufferInfo = &descBuf;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = perFrameDescSets[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].pImageInfo = &shadowImgInfo;

        vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    }

    // --- Create reflection per-frame UBO and descriptor set ---
    {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = sizeof(GPUPerFrameData);
        bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo mapInfo{};
        if (vmaCreateBuffer(vkCtx->getAllocator(), &bufInfo, &allocInfo,
                &reflPerFrameUBO, &reflPerFrameUBOAlloc, &mapInfo) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection per-frame UBO");
            return false;
        }
        reflPerFrameUBOMapped = mapInfo.pMappedData;

        VkDescriptorSetLayout layouts[MAX_FRAMES];
        for (auto& layout : layouts) layout = perFrameSetLayout;

        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = sceneDescriptorPool;
        setAlloc.descriptorSetCount = MAX_FRAMES;
        setAlloc.pSetLayouts = layouts;

        if (vkAllocateDescriptorSets(device, &setAlloc, reflPerFrameDescSet) != VK_SUCCESS) {
            LOG_ERROR("Failed to allocate reflection per-frame descriptor sets");
            return false;
        }

        // Bind each reflection descriptor to the same UBO but its own frame's shadow view
        for (uint32_t i = 0; i < MAX_FRAMES; i++) {
            VkDescriptorBufferInfo descBuf{};
            descBuf.buffer = reflPerFrameUBO;
            descBuf.offset = 0;
            descBuf.range = sizeof(GPUPerFrameData);

            VkDescriptorImageInfo shadowImgInfo{};
            // sampler is ignored: binding 1 declares it immutable in the layout.
            shadowImgInfo.imageView = shadowDepthView[i];
            shadowImgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            VkWriteDescriptorSet writes[2]{};
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = reflPerFrameDescSet[i];
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[0].pBufferInfo = &descBuf;
            writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[1].dstSet = reflPerFrameDescSet[i];
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[1].pImageInfo = &shadowImgInfo;

            vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
        }
    }

    LOG_INFO("Per-frame Vulkan resources created (shadow map ", SHADOW_MAP_SIZE, "x", SHADOW_MAP_SIZE, ")");
    return true;
}

void Renderer::destroyPerFrameResources() {
    if (!vkCtx) return;
    vkDeviceWaitIdle(vkCtx->getDevice());
    VkDevice device = vkCtx->getDevice();

    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        destroy(vkCtx->getAllocator(), perFrameUBOs[i], perFrameUBOAllocs[i]);
    }
    if (reflPerFrameUBO) {
        vmaDestroyBuffer(vkCtx->getAllocator(), reflPerFrameUBO, reflPerFrameUBOAlloc);
        reflPerFrameUBO = VK_NULL_HANDLE;
        reflPerFrameUBOMapped = nullptr;
    }
    destroy(device, sceneDescriptorPool);
    destroy(device, perFrameSetLayout);

    // Destroy per-frame shadow resources
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        if (shadowFramebuffer[i]) { vkDestroyFramebuffer(device, shadowFramebuffer[i], nullptr); shadowFramebuffer[i] = VK_NULL_HANDLE; }
        if (shadowDepthView[i]) { vkDestroyImageView(device, shadowDepthView[i], nullptr); shadowDepthView[i] = VK_NULL_HANDLE; }
        if (shadowDepthImage[i]) { vmaDestroyImage(vkCtx->getAllocator(), shadowDepthImage[i], shadowDepthAlloc[i]); shadowDepthImage[i] = VK_NULL_HANDLE; shadowDepthAlloc[i] = VK_NULL_HANDLE; }
        shadowDepthLayout_[i] = VK_IMAGE_LAYOUT_UNDEFINED;
    }
    if (shadowRenderPass) { vkDestroyRenderPass(device, shadowRenderPass, nullptr); shadowRenderPass = VK_NULL_HANDLE; }
    shadowSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
}

void Renderer::updatePerFrameUBO() {
    if (!camera) return;

    currentFrameData.view = camera->getViewMatrix();
    currentFrameData.projection = camera->getProjectionMatrix();
    currentFrameData.viewPos = glm::vec4(camera->getPosition(), 1.0f);
    currentFrameData.fogParams.z = globalTime;

    // Lighting from LightingManager
    if (lightingManager) {
        const auto& lp = lightingManager->getLightingParams();
        currentFrameData.lightDir = glm::vec4(lp.directionalDir, 0.0f);
        currentFrameData.lightColor = glm::vec4(lp.diffuseColor, 1.0f);
        currentFrameData.ambientColor = glm::vec4(lp.ambientColor, 1.0f);
        currentFrameData.fogColor = glm::vec4(lp.fogColor, 1.0f);
        currentFrameData.fogParams.x = lp.fogStart;
        currentFrameData.fogParams.y = lp.fogEnd;

        // Shift fog to blue when camera is significantly underwater (terrain water only).
        if (waterRenderer && camera) {
            glm::vec3 camPos = camera->getPosition();
            auto waterH = waterRenderer->getNearestWaterHeightAt(camPos.x, camPos.y, camPos.z);
            constexpr float MIN_SUBMERSION = 2.0f;
            if (waterH && camPos.z < (*waterH - MIN_SUBMERSION)
                       && !waterRenderer->isWmoWaterAt(camPos.x, camPos.y)) {
                float depth = *waterH - camPos.z - MIN_SUBMERSION;
                float blend = glm::clamp(1.0f - std::exp(-depth * 0.08f), 0.0f, 0.7f);
                glm::vec3 underwaterFog(0.03f, 0.09f, 0.18f);
                glm::vec3 blendedFog = glm::mix(lp.fogColor, underwaterFog, blend);
                currentFrameData.fogColor = glm::vec4(blendedFog, 1.0f);
                currentFrameData.fogParams.x = glm::mix(lp.fogStart, 20.0f, blend);
                currentFrameData.fogParams.y = glm::mix(lp.fogEnd, 200.0f, blend);
            }
        }
    }

    currentFrameData.lightSpaceMatrix = lightSpaceMatrix;
    // Scale shadow bias proportionally to ortho extent to avoid acne at close range / gaps at far range
    float shadowBias = glm::clamp(0.8f * (shadowDistance_ / 300.0f), 0.0f, 1.0f);
    currentFrameData.shadowParams = glm::vec4(shadowsEnabled ? 1.0f : 0.0f, shadowBias, 0.0f, 0.0f);

    for (uint32_t i = 0; i < MAX_LOCAL_LIGHTS; ++i) {
        currentFrameData.localLightPosRadius[i] = glm::vec4(0.0f);
        currentFrameData.localLightColorIntensity[i] = glm::vec4(0.0f);
    }
    uint32_t localLightCount = wmoRenderer
        ? wmoRenderer->gatherLavaLights(camera->getPosition(),
              currentFrameData.localLightPosRadius,
              currentFrameData.localLightColorIntensity,
              MAX_LOCAL_LIGHTS)
        : 0;
    if (m2Renderer && localLightCount < MAX_LOCAL_LIGHTS) {
        localLightCount += m2Renderer->gatherLocalLights(camera->getPosition(),
            currentFrameData.localLightPosRadius + localLightCount,
            currentFrameData.localLightColorIntensity + localLightCount,
            MAX_LOCAL_LIGHTS - localLightCount);
    }
    currentFrameData.localLightMeta = glm::ivec4(static_cast<int32_t>(localLightCount), 0, 0, 0);

    // What the local lights are actually doing, throttled to a line every few
    // seconds. These are gathered around the camera rather than the player, so
    // which ones are in the set changes as the view is orbited - and they are
    // the warm ones: braziers, torches, forges, lava. A warm cast that moves
    // with the camera and nothing else would look exactly like this, so it is
    // worth being able to read the count rather than reason about it.
    {
        static double lastLightLog = 0.0;
        if (localLightCount > 0 && (globalTime - lastLightLog) > 3.0) {
            lastLightLog = globalTime;
            const glm::vec4& c0 = currentFrameData.localLightColorIntensity[0];
            const glm::vec4& p0 = currentFrameData.localLightPosRadius[0];
            LOG_INFO("localLights: count=", localLightCount, " of ", MAX_LOCAL_LIGHTS,
                     " first rgb=(", c0.r, ",", c0.g, ",", c0.b, ") intensity=", c0.w,
                     " radius=", p0.w);
        }
    }

    // Player motion, consumed by water ripples and by the foliage brush in
    // m2.vert. Horizontal speed only: a fall shouldn't read as running.
    {
        const float dt = std::max(lastDeltaTime_, 0.0f);
        if (!playerMotionTracked_) {
            prevPlayerPos_ = characterPosition;
            playerWakePos_ = characterPosition;
            playerMotionTracked_ = true;
        }
        if (dt > 0.0f) {
            const glm::vec2 step(characterPosition.x - prevPlayerPos_.x,
                                 characterPosition.y - prevPlayerPos_.y);
            // A teleport is not a sprint. Anything past a gallop is discarded
            // rather than smoothed, or one map change flattens a whole field.
            constexpr float MAX_TRACKED_SPEED = 30.0f;
            const float rawSpeed = glm::length(step) / dt;
            playerSpeed_ = (rawSpeed > MAX_TRACKED_SPEED) ? 0.0f
                                                          : glm::mix(playerSpeed_, rawSpeed, 0.25f);

            // Exponential chase, framerate independent.
            constexpr float WAKE_TIME_CONSTANT = 0.30f;  // seconds of springback
            const float k = 1.0f - std::exp(-dt / WAKE_TIME_CONSTANT);
            playerWakePos_ += (characterPosition - playerWakePos_) * k;
            if (rawSpeed > MAX_TRACKED_SPEED) playerWakePos_ = characterPosition;
        }
        prevPlayerPos_ = characterPosition;
    }
    currentFrameData.playerPos = glm::vec4(characterPosition, playerSpeed_);
    currentFrameData.playerWake = glm::vec4(playerWakePos_, 0.0f);

    // Water ripple gate: swimming and actually moving.
    if (cameraController) {
        bool inWater = cameraController->isSwimming();
        bool moving = cameraController->isMoving();
        currentFrameData.fogParams.w = (inWater && moving) ? 1.0f : 0.0f;
    } else {
        currentFrameData.fogParams.w = 0.0f;
    }

    // Copy to current frame's mapped UBO
    uint32_t frame = vkCtx->getCurrentFrame();
    std::memcpy(perFrameUBOMapped[frame], &currentFrameData, sizeof(GPUPerFrameData));
}

bool Renderer::initialize(core::Window* win) {
    window = win;
    vkCtx = win->getVkContext();
    deferredWorldInitEnabled_ = core::envFlagEnabled("WOWEE_DEFER_WORLD_SYSTEMS", true);
    LOG_INFO("Initializing renderer (Vulkan)");

    // Create camera (in front of Stormwind gate, looking north)
    camera = std::make_unique<Camera>();
    camera->setPosition(glm::vec3(-8900.0f, -170.0f, 150.0f));
    camera->setRotation(0.0f, -5.0f);
    camera->setAspectRatio(window->getAspectRatio());
    camera->setFov(60.0f);

    // Create camera controller
    cameraController = std::make_unique<CameraController>(camera.get());
    cameraController->setUseWoWSpeed(true);  // Use realistic WoW movement speed
    cameraController->setMouseSensitivity(0.15f);

    // Create performance HUD
    performanceHUD = std::make_unique<PerformanceHUD>();
    performanceHUD->setPosition(PerformanceHUD::Position::TOP_LEFT);

    // What the player last chose for shadow quality, before the resources that
    // bake it in are built. Read from the CVar file rather than waited for:
    // the interface that would normally hand it over does not load until well
    // after this, and by then the shadow map exists at whatever size it was
    // given here. See Renderer::SHADOW_MAP_SIZE.
    {
        // The top two levels are the same size on purpose, and it is worth
        // saying why before someone raises the last one to 8192 as an easy win
        // for modern hardware. Each level doubles the side, so it quadruples
        // the image: at 4096 a depth map is 64 MB and there are two of them,
        // one per frame in flight, which is 128 MB. 8192 would be 512 MB of
        // VRAM for shadows alone - not a step a machine that can run this is
        // certain to have spare, and the allocation failing at start-up is not
        // a path this renderer handles gently. setShadowMapSize clamps to 4096
        // for the same reason.
        constexpr uint32_t kShadowSideForLevel[] = {512, 1024, 2048, 4096, 4096};
        const int level = std::clamp(
            std::atoi(addons::storedCVarValue("extShadowQuality", "3").c_str()), 0, 4);
        setShadowMapSize(kShadowSideForLevel[level]);
    }

    // Create per-frame UBO and descriptor sets
    if (!createPerFrameResources()) {
        LOG_ERROR("Failed to create per-frame Vulkan resources");
        return false;
    }

    // Initialize Vulkan sub-renderers (Phase 3)

    // Sky system (owns skybox, starfield, celestial, clouds, lens flare)
    skySystem = std::make_unique<SkySystem>();
    if (!skySystem->initialize(vkCtx, perFrameSetLayout)) {
        LOG_ERROR("Failed to initialize sky system");
        return false;
    }
    // Expose sub-components via renderer accessors
    skybox = nullptr;  // Owned by skySystem; access via skySystem->getSkybox()
    celestial = nullptr;
    starField = nullptr;
    clouds = nullptr;
    lensFlare = nullptr;

    weather = std::make_unique<Weather>();
    if (!weather->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Weather effect initialization failed (non-fatal)");

    lightning = std::make_unique<Lightning>();
    if (!lightning->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Lightning effect initialization failed (non-fatal)");

    swimEffects = std::make_unique<SwimEffects>();
    syncSwimEffectsTargetPass();
    if (!swimEffects->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Swim effect initialization failed (non-fatal)");

    mountDust = std::make_unique<MountDust>();
    if (!mountDust->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Mount dust effect initialization failed (non-fatal)");

    chargeEffect = std::make_unique<ChargeEffect>();
    if (!chargeEffect->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Charge effect initialization failed (non-fatal)");

    levelUpEffect = std::make_unique<LevelUpEffect>();

    // Non-fatal like the effects above: a device that cannot build the compute
    // pipeline still gets everything else, and isReady() gates both call sites.
    grassRenderer_ = std::make_unique<GrassRenderer>();
    if (!grassRenderer_->initialize(vkCtx, perFrameSetLayout))
        LOG_WARNING("Grass renderer initialization failed (non-fatal)");

    questMarkerRenderer = std::make_unique<QuestMarkerRenderer>();
    footprintRenderer = std::make_unique<FootprintRenderer>();

    LOG_INFO("Vulkan sub-renderers initialized (Phase 3)");

    // LightingManager doesn't use GL - initialize for data-only use
    lightingManager = std::make_unique<LightingManager>();
    auto* assetManager = core::Application::getInstance().getAssetManager();

    // Create zone manager; enrich music paths from DBC if available
    zoneManager = std::make_unique<game::ZoneManager>();
    zoneManager->initialize();
    if (assetManager) {
        zoneManager->enrichFromDBC(assetManager);
    }

    // Audio is now owned by AudioCoordinator (created by Application).
    // Renderer receives AudioCoordinator* via setAudioCoordinator().

    // Create secondary command buffer resources for multithreaded rendering
    if (!createSecondaryCommandResources()) {
        LOG_WARNING("Failed to create secondary command buffers - falling back to single-threaded rendering");
    }

    // Create PostProcessPipeline (§4.3 - owns FSR/FXAA/FSR2/FSR3/brightness)
    postProcessPipeline_ = std::make_unique<PostProcessPipeline>();
    postProcessPipeline_->initialize(vkCtx);

    // Create render graph and register virtual resources
    renderGraph_ = std::make_unique<RenderGraph>();

    // Create overlay system (selection circle + fullscreen overlay)
    overlaySystem_ = std::make_unique<OverlaySystem>(vkCtx);
    renderGraph_->registerResource("shadow_depth");
    renderGraph_->registerResource("reflection_texture");
    renderGraph_->registerResource("scene_color");
    renderGraph_->registerResource("scene_depth");
    renderGraph_->registerResource("final_image");

    LOG_INFO("Renderer initialized");
    return true;
}

void Renderer::shutdown() {
    destroySecondaryCommandResources();

    LOG_DEBUG("Renderer::shutdown - terrainManager stopWorkers...");
    if (terrainManager) {
        terrainManager->stopWorkers();
        LOG_DEBUG("Renderer::shutdown - terrainManager reset...");
        terrainManager.reset();
    }

    LOG_DEBUG("Renderer::shutdown - terrainRenderer...");
    if (terrainRenderer) {
        terrainRenderer->shutdown();
        terrainRenderer.reset();
    }

    LOG_DEBUG("Renderer::shutdown - waterRenderer...");
    if (waterRenderer) {
        waterRenderer->shutdown();
        waterRenderer.reset();
    }

    LOG_DEBUG("Renderer::shutdown - minimap...");
    if (minimap) {
        minimap->shutdown();
        minimap.reset();
    }

    LOG_DEBUG("Renderer::shutdown - worldMap...");
    if (worldMap) {
        worldMap->shutdown();
        worldMap.reset();
    }

    LOG_DEBUG("Renderer::shutdown - skySystem...");
    if (skySystem) {
        skySystem->shutdown();
        skySystem.reset();
    }

    // Individual sky components are owned by skySystem; just null the aliases
    skybox = nullptr;
    celestial = nullptr;
    starField = nullptr;
    clouds = nullptr;
    lensFlare = nullptr;

    if (weather) {
        weather.reset();
    }

    if (lightning) {
        lightning->shutdown();
        lightning.reset();
    }

    if (swimEffects) {
        swimEffects->shutdown();
        swimEffects.reset();
    }

    if (footprintRenderer) {
        footprintRenderer->shutdown();
        footprintRenderer.reset();
    }

    LOG_DEBUG("Renderer::shutdown - characterRenderer...");
    if (characterRenderer) {
        characterRenderer->shutdown();
        characterRenderer.reset();
    }

    // Shutdown AnimationController before renderers it references (§4.2)
    animationController_.reset();

    LOG_DEBUG("Renderer::shutdown - wmoRenderer...");
    if (wmoRenderer) {
        wmoRenderer->shutdown();
        wmoRenderer.reset();
    }

    // Shutdown SpellVisualSystem before M2Renderer (it holds M2Renderer pointer) (§4.4)
    if (spellVisualSystem_) {
        spellVisualSystem_->shutdown();
        spellVisualSystem_.reset();
    }

    if (grassRenderer_) {
        grassRenderer_->shutdown();
        grassRenderer_.reset();
    }

    LOG_DEBUG("Renderer::shutdown - m2Renderer...");
    if (hizSystem_) {
        hizSystem_->shutdown();
        hizSystem_.reset();
    }
    if (m2Renderer) {
        m2Renderer->shutdown();
        m2Renderer.reset();
    }
    if (skyboxModelRenderer_) {
        skyboxModelRenderer_->shutdown();
        skyboxModelRenderer_.reset();
        skyboxModelInstanceId_ = 0;
        skyboxModelPath_.clear();
    }

    // Audio shutdown is handled by AudioCoordinator (owned by Application).
    audioCoordinator_ = nullptr;

    // Cleanup selection circle + overlay resources
    if (overlaySystem_) {
        overlaySystem_->cleanup();
        overlaySystem_.reset();
    }

    // Shutdown post-process pipeline (FSR/FXAA/FSR2 resources) (§4.3)
    if (postProcessPipeline_) {
        postProcessPipeline_->shutdown();
        postProcessPipeline_.reset();
    }

    // Destroy render graph
    renderGraph_.reset();

    destroyPerFrameResources();

    zoneManager.reset();

    performanceHUD.reset();
    cameraController.reset();
    camera.reset();

    LOG_INFO("Renderer shutdown");
}

void Renderer::registerPreview(CharacterPreview* preview) {
    if (!preview) return;
    auto it = std::find(activePreviews_.begin(), activePreviews_.end(), preview);
    if (it == activePreviews_.end()) {
        activePreviews_.push_back(preview);
    }
}

void Renderer::unregisterPreview(CharacterPreview* preview) {
    auto it = std::find(activePreviews_.begin(), activePreviews_.end(), preview);
    if (it != activePreviews_.end()) {
        activePreviews_.erase(it);
    }
}

void Renderer::setWaterRefractionEnabled(bool /*enabled*/) {
    // Always on. Kept as a call rather than deleted because saved settings and
    // the CVar bridge still reach it, and a config written before this that says
    // 0 should not be able to turn it off again.
    if (waterRenderer) waterRenderer->setRefractionEnabled(true);
}
void Renderer::setMsaaSamples(VkSampleCountFlagBits samples) {
    if (!vkCtx) return;

    // FSR2 requires non-MSAA render pass - block MSAA changes while FSR2 is active
    if (postProcessPipeline_ && postProcessPipeline_->isFsr2BlockingMsaa() && samples > VK_SAMPLE_COUNT_1_BIT) return;

    // Clamp to device maximum
    VkSampleCountFlagBits maxSamples = vkCtx->getMaxUsableSampleCount();
    if (samples > maxSamples) samples = maxSamples;

    if (samples == vkCtx->getMsaaSamples()) return;

    // Defer to between frames - cannot destroy render pass/framebuffers mid-frame
    pendingMsaaSamples_ = samples;
    msaaChangePending_ = true;
}

void Renderer::applyMsaaChange() {
    VkSampleCountFlagBits samples = pendingMsaaSamples_;
    msaaChangePending_ = false;

    // FSR2 requires non-MSAA render pass - if FSR2 was enabled after the MSAA
    // change was queued (startup race), force 1x to avoid framebuffer mismatch.
    if (samples > VK_SAMPLE_COUNT_1_BIT &&
        postProcessPipeline_ && postProcessPipeline_->isFsr2BlockingMsaa()) {
        samples = VK_SAMPLE_COUNT_1_BIT;
    }

    VkSampleCountFlagBits current = vkCtx->getMsaaSamples();
    if (samples == current) return;

    // Single GPU wait - all subsequent operations are CPU-side object creation
    vkDeviceWaitIdle(vkCtx->getDevice());

    // Set new MSAA and recreate swapchain (render pass, depth, MSAA image, framebuffers)
    vkCtx->setMsaaSamples(samples);
    if (!vkCtx->recreateSwapchain(window->getWidth(), window->getHeight())) {
        LOG_ERROR("MSAA change failed - reverting to 1x");
        vkCtx->setMsaaSamples(VK_SAMPLE_COUNT_1_BIT);
        (void)vkCtx->recreateSwapchain(window->getWidth(), window->getHeight());
    }

    // Recreate all sub-renderer pipelines (they embed sample count from render pass)
    if (terrainRenderer) terrainRenderer->recreatePipelines();
    if (waterRenderer) {
        waterRenderer->recreatePipelines();
        // Under MSAA the water draws single-sampled in its own pass, after the
        // scene has resolved - it is a large alpha-blended surface whose edges
        // MSAA does nothing for, and drawing it there also keeps it out of its
        // own refraction copy.
        waterRenderer->destroyWater1xResources();
        setupWater1xPass();
    }
    if (wmoRenderer) wmoRenderer->recreatePipelines();
    if (m2Renderer) m2Renderer->recreatePipelines();
    if (skyboxModelRenderer_) skyboxModelRenderer_->recreatePipelines();
    if (characterRenderer) characterRenderer->recreatePipelines();
    if (questMarkerRenderer) questMarkerRenderer->recreatePipelines();
    if (footprintRenderer) footprintRenderer->recreatePipelines();
    if (weather) weather->recreatePipelines();
    if (lightning) lightning->recreatePipelines();
    if (swimEffects) {
        syncSwimEffectsTargetPass();
        swimEffects->recreatePipelines();
    }
    if (mountDust) mountDust->recreatePipelines();
    if (chargeEffect) chargeEffect->recreatePipelines();

    // Sky system sub-renderers
    if (skySystem) {
        if (auto* sb = skySystem->getSkybox()) sb->recreatePipelines();
        if (auto* sf = skySystem->getStarField()) sf->recreatePipelines();
        if (auto* ce = skySystem->getCelestial()) ce->recreatePipelines();
        if (auto* cl = skySystem->getClouds()) cl->recreatePipelines();
        if (auto* lf = skySystem->getLensFlare()) lf->recreatePipelines();
    }

    if (minimap) minimap->recreatePipelines();

    // Resize HiZ pyramid (depth format/MSAA may have changed)
    if (hizSystem_) {
        auto ext = vkCtx->getSwapchainExtent();
        if (!hizSystem_->resize(ext.width, ext.height)) {
            LOG_WARNING("HiZ resize failed after MSAA change");
            if (m2Renderer) m2Renderer->setHiZSystem(nullptr);
            hizSystem_->shutdown();
            hizSystem_.reset();
        }
    }

    // Selection circle + overlay + FSR use lazy init, just destroy them
    if (overlaySystem_) overlaySystem_->recreatePipelines();
    if (postProcessPipeline_) postProcessPipeline_->destroyAllResources(); // Will be lazily recreated in beginFrame()

    // ImGui is deliberately not restarted here.
    //
    // It always initialises at one sample into the overlay pass, which is
    // itself always single-sampled and depends only on the swapchain format -
    // so a change of scene anti-aliasing does not change anything ImGui built.
    // Recreating the swapchain produces a new overlay pass handle, but a
    // structurally identical one, and Vulkan requires a pipeline's render pass
    // to be compatible rather than the same object.
    //
    // Tearing the backend down destroyed its descriptor pool, and every UI
    // texture in the client - item and spell icons, raid icons, the talent
    // background, the world map layers, the widget renderer - holds a
    // descriptor set allocated from it. Nothing was told, so the next frame
    // drew with freed descriptors and the GPU was reset: the log shows the
    // swapchain and pipelines rebuilt, then the fence wait failing with
    // VK_ERROR_DEVICE_LOST a fraction of a second later. Applying a saved
    // anti-aliasing setting at startup made that look like a crash on launch.

}

void Renderer::beginFrame() {
    ZoneScopedN("Renderer::beginFrame");
    if (!vkCtx) return;
    if (vkCtx->isDeviceLost()) return;

    // Apply deferred MSAA change between frames (before any rendering state is used)
    if (msaaChangePending_) {
        applyMsaaChange();
        // The rebuild destroys and remakes the swapchain, every render pass and
        // every pipeline. The frame slots are left mid-cycle by it, and the
        // next frame would reset a fence and re-record a command buffer the
        // GPU has not finished with - which is what validation reports and the
        // driver answers by losing the device.
        if (vkCtx) vkCtx->resetFrameSyncState();
    }

    // Retire finished upload batches every frame.
    //
    // This was polled only from the terrain manager, so batches submitted by
    // anything else retired only while terrain happened to be streaming. A
    // rebuild reported 1423 submitted against 1241 retired - 182 outstanding,
    // each holding a fence, a command buffer and its staging buffers. With
    // FrameXML uploading hundreds of textures the backlog is much larger than
    // it was, and nothing bounded it.
    if (vkCtx) vkCtx->pollUploadBatches();

    // Post-process resource management (§4.3 - delegates to PostProcessPipeline)
    if (postProcessPipeline_) postProcessPipeline_->manageResources();

    // Handle swapchain recreation if needed
    if (vkCtx->isSwapchainDirty()) {
        // Skip recreation while window is minimized (0×0 extent is a Vulkan spec violation)
        if (window->getWidth() == 0 || window->getHeight() == 0) return;
        (void)vkCtx->recreateSwapchain(window->getWidth(), window->getHeight());
        // Rebuild water resources that reference swapchain extent/views
        if (waterRenderer) {
            waterRenderer->recreatePipelines();
            waterRenderer->destroyWater1xResources();
            setupWater1xPass();
        }
        // Recreate post-process resources for new swapchain dimensions
        if (postProcessPipeline_) postProcessPipeline_->handleSwapchainResize();
        // Resize HiZ depth pyramid for new swapchain dimensions
        if (hizSystem_) {
            auto ext = vkCtx->getSwapchainExtent();
            if (!hizSystem_->resize(ext.width, ext.height)) {
                LOG_WARNING("HiZ resize failed - disabling occlusion culling");
                if (m2Renderer) m2Renderer->setHiZSystem(nullptr);
                hizSystem_->shutdown();
                hizSystem_.reset();
            }
        }
    }

    // Acquire swapchain image and begin command buffer
    currentCmd = vkCtx->beginFrame(currentImageIndex);
    if (currentCmd == VK_NULL_HANDLE) {
        // Swapchain out of date, will retry next frame
        return;
    }

    // FSR2 jitter pattern (§4.3 - delegates to PostProcessPipeline)
    if (postProcessPipeline_ && camera) postProcessPipeline_->applyJitter(camera.get());

    // Compute fresh shadow matrix BEFORE UBO update so shaders get current-frame data.
    lightSpaceMatrix = computeLightSpaceMatrix();

    // Update per-frame UBO with current camera/lighting state
    updatePerFrameUBO();

    // ── Early compute: M2 frustum culling ──
    // beginFrame() has already waited for this frame slot's previous fence, so
    // its mapped visibility output is complete and safe for the CPU to reuse.
    // Read/invalidate that completed output, then record the next cull dispatch
    // directly into the normal frame command buffer. The old path submitted a
    // separate command buffer and synchronously waited on a fence every frame,
    // serializing CPU and GPU work solely to obtain same-frame cull results.
    if (m2Renderer && camera && vkCtx) {
        uint32_t frame = vkCtx->getCurrentFrame();
        m2Renderer->invalidateCullOutput(frame);
        m2Renderer->dispatchCullCompute(currentCmd, frame, *camera);
    }

    // Grass culls here too, for the same reason: a dispatch has to be recorded
    // outside a render pass. Unlike the M2 path nothing reads the result back -
    // the count it produces is consumed by the indirect draw on the GPU.
    if (grassRenderer_ && camera && vkCtx) {
        grassRenderer_->dispatchCull(currentCmd, vkCtx->getCurrentFrame(), *camera,
                                     characterPosition);
    }

    // --- Off-screen pre-passes ---
    // Build frame graph: registers pre-passes as graph nodes with dependencies.
    // compile() topologically sorts; execute() runs them with auto barriers.
    buildFrameGraph(nullptr);
    if (renderGraph_) {
        renderGraph_->execute(currentCmd);
    }

    // --- Begin render pass ---
    // Select framebuffer: PP off-screen target or swapchain (§4.3 - PostProcessPipeline)
    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = vkCtx->getImGuiRenderPass();

    VkExtent2D renderExtent;
    VkFramebuffer ppFB = postProcessPipeline_ ? postProcessPipeline_->getSceneFramebuffer() : VK_NULL_HANDLE;
    if (ppFB != VK_NULL_HANDLE) {
        rpInfo.framebuffer = ppFB;
        renderExtent = postProcessPipeline_->getSceneRenderExtent();
    } else {
        rpInfo.framebuffer = vkCtx->getSwapchainFramebuffers()[currentImageIndex];
        renderExtent = vkCtx->getSwapchainExtent();
    }

    rpInfo.renderArea.offset = {.x = 0, .y = 0};
    rpInfo.renderArea.extent = renderExtent;

    // Clear values must match attachment count: 2 (no MSAA), 3 (MSAA), or 4 (MSAA+depth resolve)
    VkClearValue clearValues[4]{};
    clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clearValues[1].depthStencil = {.depth = 1.0f, .stencil = 0};
    clearValues[2].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clearValues[3].depthStencil = {.depth = 1.0f, .stencil = 0};
    bool msaaOn = (vkCtx->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT);
    if (msaaOn) {
        bool depthRes = (vkCtx->getDepthResolveImageView() != VK_NULL_HANDLE);
        rpInfo.clearValueCount = depthRes ? 4 : 3;
    } else {
        rpInfo.clearValueCount = 2;
    }
    rpInfo.pClearValues = clearValues;

    // Cache render pass state for secondary command buffer inheritance
    activeRenderPass_ = rpInfo.renderPass;
    activeFramebuffer_ = rpInfo.framebuffer;
    activeRenderExtent_ = renderExtent;

    VkSubpassContents subpassMode = parallelRecordingEnabled_
        ? VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS
        : VK_SUBPASS_CONTENTS_INLINE;
    vkCmdBeginRenderPass(currentCmd, &rpInfo, subpassMode);

    if (!parallelRecordingEnabled_) {
        // Fallback: set dynamic viewport and scissor on primary (inline mode)
        VkViewport viewport{};
        viewport.width = static_cast<float>(renderExtent.width);
        viewport.height = static_cast<float>(renderExtent.height);
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.extent = renderExtent;
        vkCmdSetScissor(currentCmd, 0, 1, &scissor);
    }
}

void Renderer::endFrame() {
    ZoneScopedN("Renderer::endFrame");
    if (!vkCtx || currentCmd == VK_NULL_HANDLE) return;

    logViewDistanceDiag();

    // Post-process execution (§4.3 - delegates to PostProcessPipeline). Whether
    // it swapped the scene pass for an INLINE one no longer matters to the
    // caller: the UI is drawn in the overlay pass, which this function opens
    // itself once whichever pass is current has been closed.
    if (postProcessPipeline_) {
        postProcessPipeline_->executePostProcessing(
            currentCmd, currentImageIndex, camera.get(), lastDeltaTime_);
    }

    // The scene is complete: close its pass so the water refraction copy can run
    // (a copy is illegal inside a render pass), then draw the UI in the overlay
    // pass. Capturing after the UI instead is what refracted the interface into
    // the water. The overlay pass is single-sampled and colour-only, which is
    // also why the UI costs the same here whatever MSAA the scene uses.
    vkCmdEndRenderPass(currentCmd);

    // Only when water could not be moved out of the scene pass (MSAA). Otherwise
    // renderWorld already took the copy at the one point in the frame where the
    // scene is finished but the water is not yet over it; copying again here
    // would replace that with an image containing the water.
    if (!waterDrawsInContinuePass()
        && waterRenderer && waterRenderer->isRefractionEnabled() && waterRenderer->hasSurfaces()
        && currentImageIndex < vkCtx->getSwapchainImages().size()) {
        waterRenderer->captureSceneHistory(
            currentCmd,
            vkCtx->getSwapchainImages()[currentImageIndex],
            vkCtx->getDepthCopySourceImage(),
            vkCtx->getSwapchainExtent(),
            vkCtx->isDepthCopySourceMsaa(),
            vkCtx->getCurrentFrame());
    }

    const auto& overlayFbs = vkCtx->getOverlayFramebuffers();
    if (vkCtx->getOverlayRenderPass() != VK_NULL_HANDLE && currentImageIndex < overlayFbs.size()) {
        VkRenderPassBeginInfo overlayRp{};
        overlayRp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        overlayRp.renderPass = vkCtx->getOverlayRenderPass();
        overlayRp.framebuffer = overlayFbs[currentImageIndex];
        overlayRp.renderArea.extent = vkCtx->getSwapchainExtent();
        vkCmdBeginRenderPass(currentCmd, &overlayRp, VK_SUBPASS_CONTENTS_INLINE);

        VkExtent2D ext = vkCtx->getSwapchainExtent();
        VkViewport vp{};
        vp.width = static_cast<float>(ext.width);
        vp.height = static_cast<float>(ext.height);
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd, 0, 1, &vp);
        VkRect2D sc{};
        sc.extent = ext;
        vkCmdSetScissor(currentCmd, 0, 1, &sc);

        // ImGui's pipelines are built against the overlay pass, so it always
        // records inline here rather than into a scene-pass secondary buffer.
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), currentCmd);
        vkCmdEndRenderPass(currentCmd);
    } else {
        LOG_ERROR("Overlay render pass missing - UI not drawn this frame");
    }

    // Water now renders in the main pass (renderWorld), no separate 1x pass needed.

    // Submit and present
    vkCtx->endFrame(currentCmd, currentImageIndex);
    currentCmd = VK_NULL_HANDLE;
}

void Renderer::setCharacterFollow(uint32_t instanceId) {
    characterInstanceId = instanceId;
    if (cameraController && instanceId > 0) {
        cameraController->setFollowTarget(&characterPosition);
    }
    if (animationController_) animationController_->onCharacterFollow(instanceId);
}

bool Renderer::captureScreenshot(const std::string& outputPath) {
    if (!vkCtx) return false;

    VkDevice device     = vkCtx->getDevice();
    VmaAllocator alloc  = vkCtx->getAllocator();
    VkExtent2D extent   = vkCtx->getSwapchainExtent();
    const auto& images  = vkCtx->getSwapchainImages();

    if (images.empty() || currentImageIndex >= images.size()) return false;

    VkImage srcImage = images[currentImageIndex];
    uint32_t w = extent.width;
    uint32_t h = extent.height;
    VkDeviceSize bufSize = static_cast<VkDeviceSize>(w) * h * 4;

    // Stall GPU so the swapchain image is idle
    vkDeviceWaitIdle(device);

    // Create staging buffer
    VkBufferCreateInfo bufInfo{.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufInfo.size  = bufSize;
    bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_CPU_ONLY;

    VkBuffer stagingBuf = VK_NULL_HANDLE;
    VmaAllocation stagingAlloc = VK_NULL_HANDLE;
    if (vmaCreateBuffer(alloc, &bufInfo, &allocCI, &stagingBuf, &stagingAlloc, nullptr) != VK_SUCCESS) {
        LOG_WARNING("Screenshot: failed to create staging buffer");
        return false;
    }

    // Record copy commands
    VkCommandBuffer cmd = vkCtx->beginSingleTimeCommands();

    // Transition swapchain image: PRESENT_SRC → TRANSFER_SRC
    VkImageMemoryBarrier2 toTransfer{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
    toTransfer.srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    toTransfer.dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    toTransfer.srcAccessMask       = VK_ACCESS_MEMORY_READ_BIT;
    toTransfer.dstAccessMask       = VK_ACCESS_TRANSFER_READ_BIT;
    toTransfer.oldLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    toTransfer.newLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    toTransfer.image               = srcImage;
    toTransfer.subresourceRange    = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    VkDependencyInfo toTransferDep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    toTransferDep.dependencyFlags = 0;
    toTransferDep.imageMemoryBarrierCount = 1;
    toTransferDep.pImageMemoryBarriers = &toTransfer;
    cmdPipelineBarrier2(cmd, toTransferDep);

    // Copy image to buffer
    VkBufferImageCopy region{};
    region.imageSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
    region.imageExtent      = {.width = w, .height = h, .depth = 1};
    vkCmdCopyImageToBuffer(cmd, srcImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           stagingBuf, 1, &region);

    // Transition back: TRANSFER_SRC → PRESENT_SRC
    VkImageMemoryBarrier2 toPresent = toTransfer;
    toPresent.srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    toPresent.dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    toPresent.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    toPresent.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    toPresent.oldLayout     = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    toPresent.newLayout     = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkDependencyInfo toPresentDep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    toPresentDep.imageMemoryBarrierCount = 1;
    toPresentDep.pImageMemoryBarriers = &toPresent;
    cmdPipelineBarrier2(cmd, toPresentDep);

    vkCtx->endSingleTimeCommands(cmd);

    // Map and convert BGRA → RGBA
    void* mapped = nullptr;
    vmaMapMemory(alloc, stagingAlloc, &mapped);
    auto* pixels = static_cast<uint8_t*>(mapped);
    for (uint32_t i = 0; i < w * h; ++i) {
        std::swap(pixels[i * 4 + 0], pixels[i * 4 + 2]); // B ↔ R
    }

    // Ensure output directory exists
    std::filesystem::path outPath(outputPath);
    if (outPath.has_parent_path())
        std::filesystem::create_directories(outPath.parent_path());

    int ok = stbi_write_png(outputPath.c_str(),
                            static_cast<int>(w), static_cast<int>(h),
                            4, pixels, static_cast<int>(w * 4));

    vmaUnmapMemory(alloc, stagingAlloc);
    vmaDestroyBuffer(alloc, stagingBuf, stagingAlloc);

    if (ok) {
        LOG_INFO("Screenshot saved: ", outputPath);
    } else {
        LOG_WARNING("Screenshot: stbi_write_png failed for ", outputPath);
    }
    return ok != 0;
}

void Renderer::resetCombatVisualState() {
    if (animationController_) animationController_->resetCombatVisualState();
    if (spellVisualSystem_) spellVisualSystem_->reset();
}

const std::string& Renderer::getCurrentZoneName() const {
    static const std::string empty;
    return audioCoordinator_ ? audioCoordinator_->getCurrentZoneName() : empty;
}

bool Renderer::ensureSkyboxModel() {
    // Which skybox model a place uses is Light.dbc's answer, not a map id:
    // LightParams names a LightSkybox row and LightSkybox names the model, and
    // getActiveSkyboxPath already walks that for whatever map the player is
    // on. This was restricted to Outland, so every other zone that defines one
    // - Tirisfal's night sky among them - fell back to the procedural sky.
    //
    // A zone that names no skybox leaves the path empty and is unaffected.
    // WOWEE_NO_SKY_M2=1 draws the procedural sky alone.
    //
    // There are two skies over the player - this client's own gradient dome and
    // the original client's sky model on top of it - and a report about the sky
    // cannot say which. Everything measurable about the model is right: the
    // lighting inputs behind it hold still, its clock advances at wall speed
    // with no restart, and the frame time beside it is steady. So the next
    // thing worth knowing is whether taking it away takes the fault with it,
    // and that is one bit that no amount of reading the code will supply.
    static const bool noSkyM2 = std::getenv("WOWEE_NO_SKY_M2") != nullptr;
    if (noSkyM2) return false;

    if (!skyboxModelRenderer_ || !lightingManager || !cachedAssetManager ||
        !camera) {
        return false;
    }

    std::string path = lightingManager->getActiveSkyboxPath();
    if (path.empty()) return false;
    std::replace(path.begin(), path.end(), '/', '\\');
    if (path == skyboxModelPath_) return skyboxModelInstanceId_ != 0;
    if (failedSkyboxPaths_.count(path)) return skyboxModelInstanceId_ != 0;

    // The sky that is up stays up until the next one is known to be loadable.
    //
    // This used to clear the renderer and blank the path before reading a
    // byte, so any path that did not resolve left no sky at all - and the
    // instance is dropped and rebuilt on every change, which restarts the
    // model's animation. While the active path was changing as the player
    // walked, that was a sky whose clouds kept jumping back to the start and
    // vanishing in between. The path churn is fixed in LightingManager; this
    // makes the swap itself atomic, so a failure costs nothing and the old sky
    // simply stays.
    std::vector<std::string> candidates{path};
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) {
        candidates.push_back(path + ".m2");
    } else {
        std::string ext = path.substr(dot);
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (ext == ".mdx" || ext == ".mdl") candidates.push_back(path.substr(0, dot) + ".m2");
    }

    std::vector<uint8_t> modelData;
    std::string resolvedPath;
    for (const auto& candidate : candidates) {
        modelData = cachedAssetManager->readFileOptional(candidate);
        if (!modelData.empty()) {
            resolvedPath = candidate;
            break;
        }
    }
    if (modelData.empty()) {
        LOG_WARNING("Outland original skybox unavailable: ", path);
        failedSkyboxPaths_.insert(path);
        return skyboxModelInstanceId_ != 0;
    }

    pipeline::M2Model model = pipeline::M2Loader::load(modelData);
    model.name = resolvedPath + "#original-sky";
    const std::string skinPath = pipeline::skinPathForM2(resolvedPath);
    auto skinData = cachedAssetManager->readFileOptional(skinPath);
    if (!skinData.empty() && model.version >= 264) {
        pipeline::M2Loader::loadSkin(skinData, model);
    }
    if (!model.isValid()) {
        LOG_WARNING("Outland original skybox model is invalid: ", resolvedPath);
        failedSkyboxPaths_.insert(path);
        return skyboxModelInstanceId_ != 0;
    }

    // The model is good, so the old one can go now.
    skyboxModelRenderer_->clear();
    skyboxModelPath_ = path;
    skyboxModelInstanceId_ = 0;

    const uint32_t modelId = static_cast<uint32_t>(std::hash<std::string>{}(model.name));
    if (!skyboxModelRenderer_->loadModel(model, modelId)) {
        LOG_WARNING("Failed to upload Outland original skybox: ", resolvedPath);
        failedSkyboxPaths_.insert(path);
        return false;
    }
    skyboxModelInstanceId_ = skyboxModelRenderer_->createInstance(
        modelId, camera->getPosition(), glm::vec3(0.0f), 1.0f);
    if (!skyboxModelInstanceId_) {
        failedSkyboxPaths_.insert(path);
        return false;
    }
    skyboxModelRenderer_->setSkipCollision(skyboxModelInstanceId_, true);
    LOG_INFO("Outland original skybox active: ", resolvedPath);
    return true;
}

bool Renderer::isOnOutdoorPvpObjective() const {
    if (!zoneManager || !terrainManager) return false;
    if (const auto areaId = terrainManager->getAreaIdAt(
            characterPosition.x, characterPosition.y)) {
        return zoneManager->isOutdoorPvpArea(*areaId);
    }
    return false;
}

uint32_t Renderer::getCurrentZoneId() const {
    // A zone remembered from the last map is worse than none: the sticky
    // answer below would hold Duskwood's pinned midnight over a continent
    // away until the first chunk of the new one loaded.
    if (const auto* gh = core::Application::getInstance().getGameHandler()) {
        const uint32_t mapId = gh->getCurrentMapId();
        if (mapId != lastResolvedZoneMapId_) {
            lastResolvedZoneMapId_ = mapId;
            lastResolvedZoneId_ = 0;
        }
    }

    uint32_t tileZoneId = 0;
    if (zoneManager && terrainManager) {
        if (const auto areaId = terrainManager->getAreaIdAt(
                characterPosition.x, characterPosition.y)) {
            lastResolvedZoneId_ = zoneManager->resolveAreaZoneId(*areaId);
            return lastResolvedZoneId_;
        }
        const auto tile = terrainManager->getCurrentTile();
        tileZoneId = zoneManager->getZoneId(tile.x, tile.y);
    }

    // The chunk under the player did not say which area it is - either its
    // ADT is not resident yet, or its area id is zero, which a great many
    // chunks carry. That is "this chunk does not know", not "the zone
    // changed", and answering from a different source instead made the zone
    // id flip back and forth as the player walked from a chunk that knew to
    // one that did not.
    //
    // Nothing about that is quiet. The zone id picks the dark-zone ambience
    // override, which replaces the four sky colours outright, and in Duskwood
    // it also pins the visual hour to the small hours - so the sky changed
    // brightness every time a chunk boundary was crossed and held still the
    // moment the player did. Keep the last chunk that did know.
    if (lastResolvedZoneId_ != 0) return lastResolvedZoneId_;

    const auto* gh = core::Application::getInstance().getGameHandler();
    if (gh && gh->getWorldStateZoneId() != 0) {
        const uint32_t areaId = gh->getWorldStateZoneId();
        return zoneManager ? zoneManager->resolveAreaZoneId(areaId) : areaId;
    }
    if (audioCoordinator_ && audioCoordinator_->getCurrentZoneId() != 0)
        return audioCoordinator_->getCurrentZoneId();
    return tileZoneId;
}

void Renderer::update(float deltaTime) {
    ZoneScopedN("Renderer::update");
    globalTime += deltaTime;
    runDeferredWorldInitStep(deltaTime);

    auto updateStart = std::chrono::steady_clock::now();
    lastDeltaTime_ = deltaTime;

    if (wmoRenderer) wmoRenderer->resetQueryStats();
    if (m2Renderer) m2Renderer->resetQueryStats();

    if (cameraController) {
        auto cameraStart = std::chrono::steady_clock::now();
        cameraController->update(deltaTime);
        auto cameraEnd = std::chrono::steady_clock::now();
        lastCameraUpdateMs = std::chrono::duration<double, std::milli>(cameraEnd - cameraStart).count();
        if (lastCameraUpdateMs > 50.0) {
            LOG_WARNING("SLOW cameraController->update: ", lastCameraUpdateMs, "ms");
        }

        // Update 3D audio listener position/orientation to match camera.
        // getUp() internally calls getRight() which calls getForward() again,
        // and we ask for getForward() once more on the same line - that's 3
        // independent trig sequences. Cache the basis vectors once.
        if (camera) {
            const glm::vec3 fwd = camera->getForward();
            const glm::vec3 worldUp(0.0f, 0.0f, 1.0f);
            glm::vec3 right = glm::cross(fwd, worldUp);
            float rLen = glm::length(right);
            right = (rLen < 1e-6f) ? glm::vec3(1.0f, 0.0f, 0.0f) : right / rLen;
            glm::vec3 up = glm::cross(right, fwd);
            float uLen = glm::length(up);
            up = (uLen < 1e-6f) ? glm::vec3(0.0f, 0.0f, 1.0f) : up / uLen;
            audio::AudioEngine::instance().setListenerPosition(camera->getPosition());
            audio::AudioEngine::instance().setListenerOrientation(fwd, up);
        }
    } else {
        lastCameraUpdateMs = 0.0;
    }

    // Visibility hardening: ensure player instance cannot stay hidden after
    // taxi/camera transitions, but preserve first-person self-hide.
    if (characterRenderer && characterInstanceId > 0 && cameraController) {
        if ((cameraController->isThirdPerson() && !cameraController->isFirstPersonView()) || (animationController_ && animationController_->isTaxiFlight())) {
            characterRenderer->setInstanceVisible(characterInstanceId, true);
        }
    }

    // Resolve WMO containment before weather and ambience consume it. Server
    // weather remains authoritative outdoors, but particles must not follow the
    // camera through a roof into Ironforge or other enclosed WMOs.
    const bool canQueryWmo = (camera && wmoRenderer);
    const glm::vec3 camPos = camera ? camera->getPosition() : glm::vec3(0.0f);
    uint32_t insideWmoId = 0;
    const bool insideWmo = canQueryWmo &&
        wmoRenderer->isInsideWMO(camPos.x, camPos.y, camPos.z, &insideWmoId);
    // Announce the crossing. zonetext.lua and worldstateframe.lua both listen
    // for ZONE_CHANGED_INDOORS, and WoW answers the way back out with a plain
    // ZONE_CHANGED - there is no outdoors counterpart. Nothing fired either,
    // so the state was known here and never left this file.
    if (insideWmo != playerIndoors_) {
        if (auto* gh = core::Application::getInstance().getGameHandler()) {
            gh->fireAddonEvent(insideWmo ? "ZONE_CHANGED_INDOORS" : "ZONE_CHANGED", {});
        }
    }
    playerIndoors_ = insideWmo;

    // Update lighting system
    if (lightingManager) {
        const auto* gh = core::Application::getInstance().getGameHandler();
        uint32_t mapId    = gh ? gh->getCurrentMapId() : 0;
        float gameTime    = gh ? gh->getGameTime() : -1.0f;
        bool isRaining    = gh ? gh->isRaining() : false;
        bool isUnderwater = cameraController ? cameraController->isSwimming() : false;
        const uint32_t resolvedZoneId = getCurrentZoneId();

        lightingManager->update(characterPosition, mapId, resolvedZoneId,
                                gameTime, isRaining, isUnderwater);

        // Sync weather visual renderer with game state
        if (weather && gh) {
            uint32_t wType = gh->getWeatherType();
            float wInt = gh->getWeatherIntensity();
            if (resolvedZoneId == 10) {
                // Duskwood's defining effect is persistent ground fog. Some
                // realms continuously report rain here; suppress those streak
                // particles so they cannot replace the authored fog ambience.
                weather->setWeatherType(Weather::Type::NONE);
                weather->setIntensity(0.0f);
            } else if (wType != 0) {
                // Server-driven weather (SMSG_WEATHER) - authoritative
                if (wType == 1)      weather->setWeatherType(Weather::Type::RAIN);
                else if (wType == 2) weather->setWeatherType(Weather::Type::SNOW);
                else if (wType == 3) weather->setWeatherType(Weather::Type::STORM);
                else                 weather->setWeatherType(Weather::Type::NONE);
                weather->setIntensity(wInt);
            } else {
                // No server weather - use zone-based weather configuration
                weather->updateZoneWeather(getCurrentZoneId(), deltaTime);
            }
            weather->setEnabled(!insideWmo);

            // Lightning flash disabled
            if (lightning) {
                lightning->setEnabled(false);
            }
        } else if (weather) {
            // No game handler (single-player without network) - zone weather only
            weather->updateZoneWeather(getCurrentZoneId(), deltaTime);
            weather->setEnabled(!insideWmo);
        }
    }

    // Sync character model position/rotation and animation with follow target
    if (characterInstanceId > 0 && characterRenderer && cameraController) {
        characterRenderer->setInstancePosition(characterInstanceId, characterPosition);

        // Movement-facing comes from camera controller and is decoupled from LMB orbit.
        bool taxiFlight = animationController_ && animationController_->isTaxiFlight();
        bool activeStrafe = (cameraController->isStrafingLeft() || cameraController->isStrafingRight())
                             && !cameraController->isMovingBackward();
        float torsoYawDeltaDeg = 0.0f;
        if (taxiFlight) {
            characterYaw = cameraController->getFacingYaw();
        } else if (cameraController->isMoving() && activeStrafe) {
            characterYaw = cameraController->getTravelYaw();
            torsoYawDeltaDeg = cameraController->getFacingYaw() - characterYaw;
        } else if (cameraController->isMoving() || cameraController->isRightMouseHeld() ||
                   cameraController->isTurningLeft() || cameraController->isTurningRight()) {
            characterYaw = cameraController->getFacingYaw();
        } else if (animationController_ && animationController_->isInCombat() &&
                   animationController_->getTargetPosition() && !animationController_->isEmoteActive() && !(animationController_ && animationController_->isMounted())) {
            glm::vec3 toTarget = *animationController_->getTargetPosition() - characterPosition;
            if (toTarget.x * toTarget.x + toTarget.y * toTarget.y > 0.01f) {
                // Go through canonical, the way spawning and the camera do.
                // Taking atan2 of the render delta directly yields a heading in
                // a different convention - a mirror about 135 degrees - so the
                // spin looked roughly right but the frame loop then converted it
                // back to a canonical yaw that pointed somewhere else, and the
                // server rejected the cast for not facing the target.
                const glm::vec3 toTargetCanonical = ::wowee::core::coords::renderToCanonical(toTarget);
                const float canonYawToTarget =
                    std::atan2(-toTargetCanonical.y, toTargetCanonical.x);
                float targetYaw = ::wowee::core::coords::canonicalToCharacterYawDeg(canonYawToTarget);
                float diff = targetYaw - characterYaw;
                while (diff > 180.0f) diff -= 360.0f;
                while (diff < -180.0f) diff += 360.0f;
                float rotSpeed = 360.0f * deltaTime;
                if (std::abs(diff) < rotSpeed) {
                    characterYaw = targetYaw;
                } else {
                    characterYaw += (diff > 0 ? rotSpeed : -rotSpeed);
                }
            }
        }
        float yawRad = glm::radians(characterYaw);
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(0.0f, 0.0f, yawRad));

        while (torsoYawDeltaDeg > 180.0f) torsoYawDeltaDeg -= 360.0f;
        while (torsoYawDeltaDeg < -180.0f) torsoYawDeltaDeg += 360.0f;
        characterRenderer->setInstanceTorsoYaw(characterInstanceId, glm::radians(torsoYawDeltaDeg));

        // Update animation based on movement state (delegated to AnimationController §4.2)
        if (animationController_) {
            animationController_->updateMeleeTimers(deltaTime);
            animationController_->setDeltaTime(deltaTime);
            animationController_->updateCharacterAnimation();
        }
    }

    // Update terrain streaming
    if (terrainManager && camera) {
        auto terrStart = std::chrono::steady_clock::now();
        terrainManager->update(*camera, deltaTime);
        float terrMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - terrStart).count();
        if (terrMs > 50.0f) {
            LOG_WARNING("SLOW terrainManager->update: ", terrMs, "ms");
        }
    }

    // Update sky system (skybox time, star twinkle, clouds, celestial moon phases)
    if (skySystem) {
        skySystem->update(deltaTime);
    }
    if (ensureSkyboxModel() && skyboxModelRenderer_ && camera) {
        skyboxModelRenderer_->setInstancePosition(skyboxModelInstanceId_, camera->getPosition());
        skyboxModelRenderer_->update(deltaTime, camera->getPosition(),
            camera->getProjectionMatrix() * camera->getViewMatrix());
    }

    // Update weather particles
    if (weather && camera) {
        weather->update(*camera, deltaTime);
    }

    // Update lightning (storm / heavy rain)
    if (lightning && camera && lightning->isEnabled()) {
        lightning->update(deltaTime, *camera);
    }

    // Update swim effects
    if (swimEffects && camera && cameraController && waterRenderer) {
        swimEffects->update(*camera, *cameraController, *waterRenderer, deltaTime);
    }

    // Surface disturbance the character leaves in the water. The droplet spray
    // is thrown by SwimEffects above; this is the froth on the surface itself,
    // which has to come from the water shader to move and light with the water.
    if (waterRenderer && camera && cameraController) {
        glm::vec3 charPos = camera->getPosition();
        const glm::vec3* followTarget = cameraController->getFollowTarget();
        if (cameraController->isThirdPerson() && followTarget) {
            charPos = *followTarget;
        }

        const bool swimming = cameraController->isSwimming();
        float intensity = 0.0f;
        bool wading = false;

        if (cameraController->isMoving()) {
            if (auto waterH = waterRenderer->getWaterHeightAt(charPos.x, charPos.y)) {
                if (swimming) {
                    // A wake only exists where the swimmer meets the surface -
                    // diving deep leaves the surface undisturbed.
                    const float below = *waterH - charPos.z;
                    intensity = glm::clamp(1.0f - (below - 0.6f) / 1.4f, 0.0f, 1.0f);
                } else {
                    // Ankle-deep barely marks the water; thigh-deep throws the
                    // most, past which the character starts swimming anyway.
                    const float depth = *waterH - charPos.z;
                    if (depth > 0.03f && depth < 1.8f) {
                        wading = true;
                        intensity = glm::clamp(depth / 0.6f, 0.3f, 1.0f);
                    }
                }
            }
        }

        const float yawRad = glm::radians(cameraController->getYaw());
        const glm::vec2 travelDir(std::sin(yawRad), -std::cos(yawRad));
        waterRenderer->updateWake(deltaTime, glm::vec2(charPos.x, charPos.y),
                                  travelDir, intensity, wading);
    }

    // Update mount dust effects
    if (mountDust) {
        mountDust->update(deltaTime);

        // Spawn dust when mounted and moving on ground
        if ((animationController_ && animationController_->isMounted()) && camera && cameraController && !(animationController_ && animationController_->isTaxiFlight())) {
            bool isMoving = cameraController->isMoving();
            bool onGround = cameraController->isGrounded();

            if (isMoving && onGround) {
                // Calculate velocity from camera direction and speed
                glm::vec3 forward = camera->getForward();
                float speed = cameraController->getMovementSpeed();
                glm::vec3 velocity = forward * speed;
                velocity.z = 0.0f;  // Ignore vertical component

                // Spawn dust at mount's feet (slightly below character position)
                float mho = animationController_ ? animationController_->getMountHeightOffset() : 0.0f;
                glm::vec3 dustPos = characterPosition - glm::vec3(0.0f, 0.0f, mho * 0.8f);
                mountDust->spawnDust(dustPos, velocity, isMoving);
            }
        }
    }
    // Update level-up effect
    if (levelUpEffect) {
        levelUpEffect->update(deltaTime);
    }
    // Update charge effect
    if (chargeEffect) {
        chargeEffect->update(deltaTime);
    }
    // Update transient spell visual instances (delegated to SpellVisualSystem §4.4)
    if (spellVisualSystem_) spellVisualSystem_->update(deltaTime);


    // Launch M2 doodad animation on background thread (overlaps with character animation + audio)
    std::future<void> m2AnimFuture;
    bool m2AnimLaunched = false;
    if (m2Renderer && camera) {
        float m2DeltaTime = deltaTime;
        glm::vec3 m2CamPos = camera->getPosition();
        glm::mat4 m2ViewProj = camera->getProjectionMatrix() * camera->getViewMatrix();
        m2AnimFuture = core::ThreadPool::frameWorkers().submit(
            [this, m2DeltaTime, m2CamPos, m2ViewProj]() {
                m2Renderer->update(m2DeltaTime, m2CamPos, m2ViewProj);
            });
        m2AnimLaunched = true;
    }

    // Update character animations (runs in parallel with M2 animation above)
    if (characterRenderer && camera) {
        characterRenderer->update(deltaTime, camera->getPosition());
    }

    // Update AudioEngine (cleanup finished sounds, etc.)
    audio::AudioEngine::instance().update(deltaTime);

    // M2Renderer::update may rebuild the instance spatial index when streaming
    // marked it dirty. Footprint spawning below performs an M2 floor query and
    // traverses that same index. Joining only after footsteps allowed the main
    // thread to walk unordered_map nodes while the animation worker cleared and
    // rebuilt them, producing a SIGSEGV in M2Renderer::gatherCandidates. Keep
    // the useful overlap with character animation and audio above, but finish
    // structural M2 work before any main-thread collision query.
    if (m2AnimLaunched) {
        try { m2AnimFuture.get(); }
        catch (const std::exception& e) { LOG_ERROR("M2 animation worker: ", e.what()); }
        m2AnimLaunched = false;
    }

    // Footsteps: age visual prints, then let authored footfall events add new ones.
    if (footprintRenderer) footprintRenderer->update(deltaTime);
    if (animationController_) animationController_->updateFootsteps(deltaTime);

    // Activity SFX + mount ambient sounds: delegated to AnimationController (§4.2)
    if (animationController_) animationController_->updateSfxState(deltaTime);

    // Ambient environmental sounds + zone/music transitions (delegated to AudioCoordinator)
    if (audioCoordinator_) {
        audio::ZoneAudioContext zctx;
        zctx.deltaTime = deltaTime;
        zctx.cameraPosition = camPos;
        zctx.isSwimming = cameraController ? cameraController->isSwimming() : false;
        zctx.insideWmo = insideWmo;
        zctx.insideWmoId = insideWmoId;
        if (weather) {
            auto wt = weather->getWeatherType();
            if (wt == Weather::Type::RAIN)       zctx.weatherType = 1;
            else if (wt == Weather::Type::SNOW)  zctx.weatherType = 2;
            else if (wt == Weather::Type::STORM) zctx.weatherType = 3;
            zctx.weatherIntensity = weather->getIntensity();
        }
        if (lightingManager) {
            zctx.gameTimeHours = lightingManager->getVisualTimeOfDayHours();
        }
        if (terrainManager) {
            auto tile = terrainManager->getCurrentTile();
            zctx.tileX = tile.x;
            zctx.tileY = tile.y;
            zctx.hasTile = true;
        }
        // Use the precise MCNK area classification when available; this avoids
        // stale server world-state zones and whole-ADT ambiguity at river banks.
        zctx.serverZoneId = getCurrentZoneId();
        zctx.zoneManager = zoneManager.get();
        audioCoordinator_->updateZoneAudio(zctx);
    }

    // Update performance HUD
    if (performanceHUD) {
        performanceHUD->update(deltaTime);
    }

    // Periodic cache hygiene: drop model GPU data no longer referenced by active instances.
    static float modelCleanupTimer = 0.0f;
    modelCleanupTimer += deltaTime;
    if (modelCleanupTimer >= 5.0f) {
        if (wmoRenderer) {
            wmoRenderer->cleanupUnusedModels();
        }
        if (m2Renderer) {
            m2Renderer->cleanupUnusedModels();
        }
        modelCleanupTimer = 0.0f;
    }

    auto updateEnd = std::chrono::steady_clock::now();
    lastUpdateMs = std::chrono::duration<double, std::milli>(updateEnd - updateStart).count();
}

void Renderer::runDeferredWorldInitStep(float deltaTime) {
    if (!deferredWorldInitEnabled_ || !deferredWorldInitPending_ || !cachedAssetManager) return;
    if (deferredWorldInitCooldown_ > 0.0f) {
        deferredWorldInitCooldown_ = std::max(0.0f, deferredWorldInitCooldown_ - deltaTime);
        if (deferredWorldInitCooldown_ > 0.0f) return;
    }

    switch (deferredWorldInitStage_) {
        case 0:
            if (audioCoordinator_->getAmbientSoundManager()) {
                audioCoordinator_->getAmbientSoundManager()->initialize(cachedAssetManager);
            }
            if (terrainManager && audioCoordinator_->getAmbientSoundManager()) {
                terrainManager->setAmbientSoundManager(audioCoordinator_->getAmbientSoundManager());
            }
            break;
        case 1:
            if (audioCoordinator_->getUiSoundManager()) audioCoordinator_->getUiSoundManager()->initialize(cachedAssetManager);
            break;
        case 2:
            if (audioCoordinator_->getCombatSoundManager()) audioCoordinator_->getCombatSoundManager()->initialize(cachedAssetManager);
            break;
        case 3:
            if (audioCoordinator_->getSpellSoundManager()) audioCoordinator_->getSpellSoundManager()->initialize(cachedAssetManager);
            break;
        case 4:
            if (audioCoordinator_->getMovementSoundManager()) audioCoordinator_->getMovementSoundManager()->initialize(cachedAssetManager);
            break;
        case 5:
            if (questMarkerRenderer && !questMarkerRenderer->initialize(vkCtx, perFrameSetLayout, cachedAssetManager))
                LOG_WARNING("Quest marker renderer re-init failed (non-fatal)");
            if (footprintRenderer && !footprintRenderer->initialize(this, vkCtx, perFrameSetLayout, cachedAssetManager))
                LOG_WARNING("Footprint renderer re-init failed (non-fatal)");
            break;
        default:
            deferredWorldInitPending_ = false;
            return;
    }

    deferredWorldInitStage_++;
    deferredWorldInitCooldown_ = 0.12f;
}

void Renderer::setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color) {
    if (overlaySystem_) overlaySystem_->setSelectionCircle(pos, radius, color);
}

void Renderer::clearSelectionCircle() {
    if (overlaySystem_) overlaySystem_->clearSelectionCircle();
}

// ========================= PostProcessPipeline delegation stubs (§4.3) =========================

PostProcessPipeline* Renderer::getPostProcessPipeline() const {
    return postProcessPipeline_.get();
}

void Renderer::setFSREnabled(bool enabled) {
    if (!postProcessPipeline_) return;
    auto req = postProcessPipeline_->setFSREnabled(enabled);
    if (req.requested) {
        pendingMsaaSamples_ = req.samples;
        msaaChangePending_ = true;
    }
}
void Renderer::setFSR2Enabled(bool enabled) {
    if (!postProcessPipeline_) return;
    auto req = postProcessPipeline_->setFSR2Enabled(enabled, camera.get());
    if (req.requested) {
        pendingMsaaSamples_ = req.samples;
        msaaChangePending_ = true;
    }
    // If enabling FSR2 and there's already a pending MSAA change to >1x
    // (e.g. startup settings loaded MSAA before FSR2), override it to 1x.
    if (enabled && msaaChangePending_ && pendingMsaaSamples_ > VK_SAMPLE_COUNT_1_BIT) {
        pendingMsaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    }
}
void Renderer::renderWorld(game::World* world, game::GameHandler* gameHandler) {
    ZoneScopedN("Renderer::renderWorld");
    (void)world;

    // Guard against null command buffer (e.g. after VK_ERROR_DEVICE_LOST)
    if (currentCmd == VK_NULL_HANDLE) return;

    // GPU crash diagnostic: skip ALL world rendering to isolate crash source
    static const bool skipAll = (std::getenv("WOWEE_SKIP_ALL_RENDER") != nullptr);
    if (skipAll) return;

    auto renderStart = std::chrono::steady_clock::now();
    lastTerrainRenderMs = 0.0;
    lastWMORenderMs = 0.0;
    lastM2RenderMs = 0.0;

    // Cache ghost state for use in overlay and FXAA passes this frame.
    ghostMode_ = (gameHandler && gameHandler->isPlayerGhost());

    uint32_t frameIdx = vkCtx->getCurrentFrame();
    VkDescriptorSet perFrameSet = perFrameDescSets[frameIdx];
    const glm::mat4& view = camera ? camera->getViewMatrix() : glm::mat4(1.0f);
    const glm::mat4& projection = camera ? camera->getProjectionMatrix() : glm::mat4(1.0f);

    // GPU crash diagnostic: skip individual renderers to isolate which one faults
    static const bool skipWMO = (std::getenv("WOWEE_SKIP_WMO") != nullptr);
    static const bool skipChars = (std::getenv("WOWEE_SKIP_CHARS") != nullptr);
    static const bool skipM2 = (std::getenv("WOWEE_SKIP_M2") != nullptr);
    static const bool skipTerrain = (std::getenv("WOWEE_SKIP_TERRAIN") != nullptr);
    static const bool skipSky = (std::getenv("WOWEE_SKIP_SKY") != nullptr);

    // Get time of day for sky-related rendering
    auto* skybox = skySystem ? skySystem->getSkybox() : nullptr;
    float timeOfDay = lightingManager
        ? lightingManager->getVisualTimeOfDayHours()
        : (skybox ? skybox->getTimeOfDay() : 12.0f);
    const bool useOriginalSkybox =
        skyboxModelRenderer_ && skyboxModelInstanceId_ != 0;

    // ── Multithreaded secondary command buffer recording ──
    // Terrain, WMO, and M2 record on worker threads while main thread handles
    // sky, characters, water, and effects.  prepareRender() on main thread first
    // to handle thread-unsafe GPU allocations (descriptor pools, bone SSBOs).
    if (parallelRecordingEnabled_) {
        // --- Pre-compute state + GPU allocations on main thread (not thread-safe) ---
        if (m2Renderer && cameraController) {
            // Use isInsideInteriorWMO (flag 0x2000) - not isInsideWMO which includes
            // outdoor WMO groups like archways/bridges that should receive shadows.
            m2Renderer->setInsideInterior(cameraController->isInsideInteriorWMO());
            m2Renderer->setOnTaxi(cameraController->isOnTaxi());
        }
        auto prepStart = std::chrono::steady_clock::now();
        if (wmoRenderer) wmoRenderer->prepareRender();
        auto prepWmoEnd = std::chrono::steady_clock::now();
        if (m2Renderer && camera) m2Renderer->prepareRender(frameIdx, *camera);
        if (useOriginalSkybox && camera)
            skyboxModelRenderer_->prepareRender(frameIdx, *camera);
        auto prepM2End = std::chrono::steady_clock::now();
        if (characterRenderer) characterRenderer->prepareRender(frameIdx);
        auto prepEnd = std::chrono::steady_clock::now();
        const double prepWmoMs  = std::chrono::duration<double, std::milli>(prepWmoEnd - prepStart).count();
        const double prepM2Ms   = std::chrono::duration<double, std::milli>(prepM2End - prepWmoEnd).count();
        const double prepCharMs = std::chrono::duration<double, std::milli>(prepEnd - prepM2End).count();

        // --- Dispatch worker threads (terrain + WMO + M2) ---
        std::future<double> terrainFuture, wmoFuture, charFuture, m2Future, postFuture;

        if (terrainRenderer && camera && terrainEnabled && !skipTerrain) {
            terrainFuture = core::ThreadPool::frameWorkers().submit([&]() -> double {
                auto t0 = std::chrono::steady_clock::now();
                VkCommandBuffer cmd = beginSecondary(SEC_TERRAIN);
                setSecondaryViewportScissor(cmd);
                terrainRenderer->render(cmd, perFrameSet, *camera);
                // Grass rides in the terrain secondary: it sits on the ground,
                // and this buffer is executed after the sky and before WMO,
                // which is exactly the order grass wants. Recording it here
                // touches only this worker's command buffer.
                if (grassRenderer_) {
                    grassRenderer_->render(cmd, frameIdx, perFrameSet);
                }
                vkEndCommandBuffer(cmd);
                return std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
            });
        }

        if (wmoRenderer && camera && !skipWMO) {
            wmoFuture = core::ThreadPool::frameWorkers().submit([&]() -> double {
                auto t0 = std::chrono::steady_clock::now();
                VkCommandBuffer cmd = beginSecondary(SEC_WMO);
                setSecondaryViewportScissor(cmd);
                wmoRenderer->render(cmd, perFrameSet, *camera, &characterPosition);
                vkEndCommandBuffer(cmd);
                return std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
            });
        }

        if (m2Renderer && camera && !skipM2) {
            m2Future = core::ThreadPool::frameWorkers().submit([&]() -> double {
                auto t0 = std::chrono::steady_clock::now();
                VkCommandBuffer cmd = beginSecondary(SEC_M2);
                setSecondaryViewportScissor(cmd);
                m2Renderer->render(cmd, perFrameSet, *camera);
                m2Renderer->renderSmokeParticles(cmd, perFrameSet);
                m2Renderer->renderM2Particles(cmd, perFrameSet);
                m2Renderer->renderM2Ribbons(cmd, perFrameSet);
                vkEndCommandBuffer(cmd);
                return std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
            });
        }

        // --- Main thread: record sky (SEC_SKY) ---
        {
            VkCommandBuffer cmd = beginSecondary(SEC_SKY);
            setSecondaryViewportScissor(cmd);
            if (skySystem && camera && !skipSky) {
                const rendering::SkyParams skyParams = rendering::skyParamsFromLighting(
                    timeOfDay,
                    gameHandler ? gameHandler->getGameTime() : -1.0f,
                    gameHandler ? gameHandler->getWeatherIntensity() : 0.0f,
                    lightingManager ? &lightingManager->getLightingParams() : nullptr,
                    useOriginalSkybox);
                skySystem->render(cmd, perFrameSet, *camera, skyParams);
                if (useOriginalSkybox) {
                    skyboxModelRenderer_->render(cmd, perFrameSet, *camera);
                }
            }
            vkEndCommandBuffer(cmd);
        }

        // --- Main thread: record selection circle before overlay state is used by post ---
        {
            VkCommandBuffer cmd = beginSecondary(SEC_SELECTION);
            setSecondaryViewportScissor(cmd);
            if (overlaySystem_) {
                overlaySystem_->renderSelectionCircle(view, projection, cmd,
                    terrainManager ? OverlaySystem::HeightQuery2D([&](float x, float y) { return terrainManager->getHeightAt(x, y); }) : OverlaySystem::HeightQuery2D{},
                    wmoRenderer ? OverlaySystem::HeightQuery3D([&](float x, float y, float z) { return wmoRenderer->getFloorHeight(x, y, z); }) : OverlaySystem::HeightQuery3D{},
                    m2Renderer ? OverlaySystem::HeightQuery3D([&](float x, float y, float z) { return m2Renderer->getFloorHeight(x, y, z); }) : OverlaySystem::HeightQuery3D{});
            }
            vkEndCommandBuffer(cmd);
        }

        // Character recording is independent after prepareRender() and no
        // longer shares the selection-circle overlay command buffer.
        charFuture = core::ThreadPool::frameWorkers().submit([&]() -> double {
            auto t0 = std::chrono::steady_clock::now();
            VkCommandBuffer cmd = beginSecondary(SEC_CHARS);
            setSecondaryViewportScissor(cmd);
            if (characterRenderer && camera && !skipChars) {
                characterRenderer->render(cmd, perFrameSet, *camera);
            }
            vkEndCommandBuffer(cmd);
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        });

        // Post-world systems are disjoint from terrain/WMO/M2/characters. Start
        // this after selection recording so OverlaySystem is never used from
        // two threads at once.
        postFuture = core::ThreadPool::frameWorkers().submit([&]() -> double {
            auto t0 = std::chrono::steady_clock::now();
            VkCommandBuffer cmd = beginSecondary(SEC_POST);
            setSecondaryViewportScissor(cmd);
            if (waterRenderer && camera && !waterDrawsInContinuePass()) {
                waterRenderer->setRenderExtent(activeRenderExtent_);
                waterRenderer->render(cmd, perFrameSet, *camera, globalTime, false, frameIdx);
            }
            if (weather && camera) weather->render(cmd, perFrameSet);
            if (lightning && camera && lightning->isEnabled()) lightning->render(cmd, perFrameSet);
            if (swimEffects && camera && !swimEffectsDrawWithWater_) {
                swimEffects->render(cmd, perFrameSet);
            }
            if (mountDust && camera) mountDust->render(cmd, perFrameSet);
            if (chargeEffect && camera) chargeEffect->render(cmd, perFrameSet);
            if (footprintRenderer && camera) footprintRenderer->render(cmd, perFrameSet, *camera);
            if (questMarkerRenderer && camera) questMarkerRenderer->render(cmd, perFrameSet, *camera);

            renderUnderwaterOverlay(cmd);
            renderPostSceneOverlays(cmd, gameHandler);
            vkEndCommandBuffer(cmd);
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        });

        // --- Wait for workers ---
        // Guard with try-catch: future::get() re-throws any exception from the
        // async task. Without this, a single bad_alloc in a render worker would
        // propagate as an unhandled exception and terminate the process.
        try { if (terrainFuture.valid()) lastTerrainRenderMs = terrainFuture.get(); }
        catch (const std::exception& e) { LOG_ERROR("Terrain render worker: ", e.what()); }
        try { if (wmoFuture.valid()) lastWMORenderMs = wmoFuture.get(); }
        catch (const std::exception& e) { LOG_ERROR("WMO render worker: ", e.what()); }
        try { if (m2Future.valid()) lastM2RenderMs = m2Future.get(); }
        catch (const std::exception& e) { LOG_ERROR("M2 render worker: ", e.what()); }
        try { if (charFuture.valid()) (void)charFuture.get(); }
        catch (const std::exception& e) { LOG_ERROR("Character render worker: ", e.what()); }
        try { if (postFuture.valid()) (void)postFuture.get(); }
        catch (const std::exception& e) { LOG_ERROR("Post render worker: ", e.what()); }

        // prepareRender() does the GPU allocations that are not thread-safe, so it runs
        // on the main thread and is not covered by the worker timings. Name the culprit
        // when a frame runs long instead of leaving renderWorld as one opaque number.
        const double prepTotalMs = prepWmoMs + prepM2Ms + prepCharMs;
        const double worstWorkerMs = std::max({lastTerrainRenderMs, lastWMORenderMs, lastM2RenderMs});
        if (prepTotalMs + worstWorkerMs > 40.0) {
            LOG_WARNING("SLOW renderWorld breakdown: prepare=", prepTotalMs,
                        "ms (wmo=", prepWmoMs, " m2=", prepM2Ms, " char=", prepCharMs,
                        ") workers: terrain=", lastTerrainRenderMs,
                        " wmo=", lastWMORenderMs, " m2=", lastM2RenderMs,
                        // Terrain is usually the critical path here, and its cost
                        // is one descriptor bind plus one draw per surviving
                        // chunk - so the counts say whether a slow frame is draw
                        // volume or something else entirely.
                        " | terrain chunks drawn=",
                        terrainRenderer ? terrainRenderer->getRenderedChunkCount() : 0,
                        " culled=",
                        terrainRenderer ? terrainRenderer->getCulledChunkCount() : 0,
                        " resident=",
                        terrainRenderer ? terrainRenderer->getChunkCount() : 0);
        }

        // --- Execute all secondary buffers in correct draw order ---
        VkCommandBuffer validCmds[6];
        uint32_t numCmds = 0;
        validCmds[numCmds++] = secondaryCmds_[SEC_SKY][frameIdx];
        if (terrainRenderer && camera && terrainEnabled && !skipTerrain)
            validCmds[numCmds++] = secondaryCmds_[SEC_TERRAIN][frameIdx];
        if (wmoRenderer && camera && !skipWMO)
            validCmds[numCmds++] = secondaryCmds_[SEC_WMO][frameIdx];
        validCmds[numCmds++] = secondaryCmds_[SEC_SELECTION][frameIdx];
        validCmds[numCmds++] = secondaryCmds_[SEC_CHARS][frameIdx];
        if (m2Renderer && camera && !skipM2)
            validCmds[numCmds++] = secondaryCmds_[SEC_M2][frameIdx];
        validCmds[numCmds++] = secondaryCmds_[SEC_POST][frameIdx];

        vkCmdExecuteCommands(currentCmd, numCmds, validCmds);

    } else {
        // ── Fallback: single-threaded inline recording (original path) ──

        if (skySystem && camera && !skipSky) {
            const rendering::SkyParams skyParams = rendering::skyParamsFromLighting(
                timeOfDay,
                gameHandler ? gameHandler->getGameTime() : -1.0f,
                gameHandler ? gameHandler->getWeatherIntensity() : 0.0f,
                lightingManager ? &lightingManager->getLightingParams() : nullptr,
                useOriginalSkybox);
            skySystem->render(currentCmd, perFrameSet, *camera, skyParams);
            if (useOriginalSkybox) {
                skyboxModelRenderer_->prepareRender(frameIdx, *camera);
                skyboxModelRenderer_->render(currentCmd, perFrameSet, *camera);
            }
        }

        if (terrainRenderer && camera && terrainEnabled && !skipTerrain) {
            auto terrainStart = std::chrono::steady_clock::now();
            terrainRenderer->render(currentCmd, perFrameSet, *camera);
            lastTerrainRenderMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - terrainStart).count();
        }

        // After terrain, before the world's models: grass sits on the ground and
        // is occluded by everything standing on it.
        if (grassRenderer_ && vkCtx) {
            grassRenderer_->render(currentCmd, vkCtx->getCurrentFrame(), perFrameSet);
        }

        if (wmoRenderer && camera && !skipWMO) {
            wmoRenderer->prepareRender();
            auto wmoStart = std::chrono::steady_clock::now();
            wmoRenderer->render(currentCmd, perFrameSet, *camera, &characterPosition);
            lastWMORenderMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - wmoStart).count();
        }

        if (overlaySystem_) {
            overlaySystem_->renderSelectionCircle(view, projection, currentCmd,
                terrainManager ? OverlaySystem::HeightQuery2D([&](float x, float y) { return terrainManager->getHeightAt(x, y); }) : OverlaySystem::HeightQuery2D{},
                wmoRenderer ? OverlaySystem::HeightQuery3D([&](float x, float y, float z) { return wmoRenderer->getFloorHeight(x, y, z); }) : OverlaySystem::HeightQuery3D{},
                m2Renderer ? OverlaySystem::HeightQuery3D([&](float x, float y, float z) { return m2Renderer->getFloorHeight(x, y, z); }) : OverlaySystem::HeightQuery3D{});
        }

        if (characterRenderer && camera && !skipChars) {
            characterRenderer->prepareRender(frameIdx);
            characterRenderer->render(currentCmd, perFrameSet, *camera);
        }

        if (m2Renderer && camera && !skipM2) {
            if (cameraController) {
                // Use isInsideInteriorWMO (flag 0x2000) for correct indoor detection
                m2Renderer->setInsideInterior(cameraController->isInsideInteriorWMO());
                m2Renderer->setOnTaxi(cameraController->isOnTaxi());
            }
            m2Renderer->prepareRender(frameIdx, *camera);
            auto m2Start = std::chrono::steady_clock::now();
            m2Renderer->render(currentCmd, perFrameSet, *camera);
            m2Renderer->renderSmokeParticles(currentCmd, perFrameSet);
            m2Renderer->renderM2Particles(currentCmd, perFrameSet);
            m2Renderer->renderM2Ribbons(currentCmd, perFrameSet);
            lastM2RenderMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - m2Start).count();
        }

        if (waterRenderer && camera && !waterDrawsInContinuePass()) {
            waterRenderer->setRenderExtent(activeRenderExtent_);
            waterRenderer->render(currentCmd, perFrameSet, *camera, globalTime, false, frameIdx);
        }
        if (weather && camera) weather->render(currentCmd, perFrameSet);
        if (lightning && camera && lightning->isEnabled()) lightning->render(currentCmd, perFrameSet);
        if (swimEffects && camera && !swimEffectsDrawWithWater_) {
            swimEffects->render(currentCmd, perFrameSet);
        }
        if (mountDust && camera) mountDust->render(currentCmd, perFrameSet);
        if (chargeEffect && camera) chargeEffect->render(currentCmd, perFrameSet);
        if (footprintRenderer && camera) footprintRenderer->render(currentCmd, perFrameSet, *camera);
        if (questMarkerRenderer && camera) questMarkerRenderer->render(currentCmd, perFrameSet, *camera);
    }

    // Underwater overlay and minimap - in the fallback path these run inline;
    // in the parallel path they were already recorded into SEC_POST above.
    if (!parallelRecordingEnabled_) {
        renderUnderwaterOverlay(currentCmd);
        renderPostSceneOverlays(currentCmd, gameHandler);
    }

    // Water is drawn last, in a continuation of the scene pass, so that the
    // refraction copy taken just before it holds the scene WITHOUT water. Taking
    // that copy from the finished frame instead fed the water its own output:
    // a moving object left one sharp copy per frame (a train of ghosts) and the
    // brightness applied to the water compounded through the loop and pumped.
    if (waterDrawsInContinuePass() && camera) {
        vkCmdEndRenderPass(currentCmd);

        VkImage sceneColor = VK_NULL_HANDLE;
        VkImage sceneDepth = VK_NULL_HANDLE;
        VkExtent2D sceneExtent = vkCtx->getSwapchainExtent();
        bool depthIsMsaa = vkCtx->isDepthCopySourceMsaa();
        if (postProcessPipeline_ && postProcessPipeline_->getSceneFramebuffer() != VK_NULL_HANDLE) {
            sceneColor = postProcessPipeline_->getSceneColorImage();
            sceneDepth = postProcessPipeline_->getSceneDepthImage();
            sceneExtent = postProcessPipeline_->getSceneRenderExtent();
            depthIsMsaa = postProcessPipeline_->sceneDepthIsMsaa();
        } else if (currentImageIndex < vkCtx->getSwapchainImages().size()) {
            sceneColor = vkCtx->getSwapchainImages()[currentImageIndex];
            sceneDepth = vkCtx->getDepthCopySourceImage();
        }

        if (sceneColor != VK_NULL_HANDLE && waterRenderer->isRefractionEnabled()) {
            waterRenderer->captureSceneHistory(currentCmd, sceneColor, sceneDepth,
                                               sceneExtent, depthIsMsaa,
                                               vkCtx->getCurrentFrame());
        }

        // Without MSAA the water continues into the scene's own framebuffer. With
        // MSAA it draws single-sampled into the resolved image instead, which is
        // both cheaper and what lets it leave the multisampled pass at all.
        const bool msaaOn = vkCtx->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT;
        VkRenderPassBeginInfo contRp{};
        contRp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        VkExtent2D waterExtent = activeRenderExtent_;
        if (msaaOn) {
            contRp.renderPass = waterRenderer->getWater1xRenderPass();
            contRp.framebuffer = waterRenderer->getWater1xFramebuffer(currentImageIndex);
            waterExtent = vkCtx->getSwapchainExtent();
        } else {
            contRp.renderPass = vkCtx->getSceneContinueRenderPass();
            contRp.framebuffer = activeFramebuffer_;
        }
        contRp.renderArea.extent = waterExtent;
        vkCmdBeginRenderPass(currentCmd, &contRp, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport vp{};
        vp.width = static_cast<float>(waterExtent.width);
        vp.height = static_cast<float>(waterExtent.height);
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd, 0, 1, &vp);
        VkRect2D sc{};
        sc.extent = waterExtent;
        vkCmdSetScissor(currentCmd, 0, 1, &sc);

        waterRenderer->setRenderExtent(waterExtent);
        waterRenderer->render(currentCmd, perFrameSet, *camera, globalTime, msaaOn, frameIdx);

        // Spray belongs on top of the surface it is thrown off. Recorded in the
        // scene pass it went under the water instead, which the sheet then hid -
        // barely at the shore where alpha sits near its floor, completely in the
        // deeper water you swim in.
        if (swimEffects && camera && swimEffectsDrawWithWater_) {
            swimEffects->render(currentCmd, perFrameSet);
        }
    }

    auto renderEnd = std::chrono::steady_clock::now();
    lastRenderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
}

// Water can leave the scene pass only when there is a continuation pass to draw
// it in, which excludes MSAA. Everything else in the frame is unaffected.
void Renderer::syncSwimEffectsTargetPass() {
    if (!swimEffects || !vkCtx) return;

    // Default: the spray stays in the scene pass, matching its MSAA sample count.
    VkRenderPass pass = vkCtx->getImGuiRenderPass();
    VkSampleCountFlagBits samples = vkCtx->getMsaaSamples();
    swimEffectsDrawWithWater_ = false;

    if (waterDrawsInContinuePass()) {
        // Both continuation passes are single-sampled: with MSAA the water draws
        // into the resolved image, and without it there is nothing to resolve.
        if (vkCtx->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT) {
            pass = waterRenderer->getWater1xRenderPass();
        } else {
            pass = vkCtx->getSceneContinueRenderPass();
        }
        if (pass != VK_NULL_HANDLE) {
            samples = VK_SAMPLE_COUNT_1_BIT;
            swimEffectsDrawWithWater_ = true;
        } else {
            pass = vkCtx->getImGuiRenderPass();
        }
    }

    swimEffects->setTargetPass(pass, samples);
}

void Renderer::renderUnderwaterOverlay(VkCommandBuffer cmd) {
    // One implementation, called from both recording paths.
    //
    // There were two. The parallel path had this one - a waterline that
    // sweeps across the view as the eye crosses the surface - and the
    // fallback path had an older one that waited until the eye was 1.5
    // units under before tinting anything at all. Whichever path the
    // client happened to be recording with decided which of the two the
    // player got, and on the fallback path crossing the surface showed a
    // bare line with no water either side of it.
if (overlaySystem_ && waterRenderer && camera) {
        glm::vec3 camPos = camera->getPosition();
        // The default vertical reach of this query is 15 units, meant to
        // stop water on a cliff above being mistaken for water the camera
        // is in. For the underwater tint that cap is the wrong end of the
        // problem: past 15 units down the query found nothing, the
        // overlay stopped, and the scene snapped bright at a fixed depth.
        // Deep ocean is far deeper than that, so reach much further here.
        constexpr float kUnderwaterReach = 400.0f;
        auto waterH = waterRenderer->getNearestWaterHeightAt(
            camPos.x, camPos.y, camPos.z, kUnderwaterReach);
        // How far the eye is under the surface. The tint used to wait
        // until 1.5 units down and then apply to the whole screen at
        // once, so crossing the surface was a step: no tint, no tint,
        // fully tinted. Start it at the surface and let a waterline
        // sweep up the view over the crossing instead.
        // How wide the crossing really is: the half-height of the near plane in
        // world units, because that is exactly the slab of world the near plane
        // spans and therefore the only depth range over which part of it can be
        // above the surface while the rest is under. It was a flat 0.55, which
        // is a number rather than a measurement - too wide here, and wrong the
        // moment the field of view or the near plane changes.
        const float fovY = glm::radians(camera->getFovDegrees());
        const float kCrossingBand =
            std::max(0.05f, camera->getNearPlane() * std::tan(fovY * 0.5f));
        const float eyeDepth = waterH ? (*waterH - camPos.z) : -1.0f;

        // Says what it decided, so a screenshot of this can be read rather than
        // guessed at. Throttled: it is one line every few seconds, and only
        // while the eye is anywhere near the surface.
        {
            static double lastLog = 0.0;
            if (waterH && std::abs(eyeDepth) < 3.0f && (globalTime - lastLog) > 2.0) {
                lastLog = globalTime;
                LOG_INFO("underwater: camZ=", camPos.z, " waterZ=", *waterH,
                         " eyeDepth=", eyeDepth, " band=", kCrossingBand,
                         " wmoWater=", waterRenderer->isWmoWaterAt(camPos.x, camPos.y) ? 1 : 0,
                         " drawing=", (eyeDepth > 0.0f) ? 1 : 0);
            }
        }
        // From a near plane's half-height above the surface, not from the
        // surface itself.
        //
        // Above the water the near plane still cuts the surface, and the water
        // in front of that cut is not drawn at all - which is the hard band of
        // bare lake bed along the bottom of the view when standing in a lake
        // looking across it. Those pixels are looking through water and should
        // be shaded as such.
        //
        // This is only safe because the split is geometric now. It was tried
        // once against the old horizon line and had to be pulled: that test
        // could not tell a dry pixel from a wet one, so standing beside a lake
        // tinted the lower half of the view. The per-pixel test can - a pixel
        // whose ray enters the world above the surface comes out untouched -
        // so the band above the surface costs nothing where there is no water.
        if (waterH && eyeDepth > -kCrossingBand
                   && !waterRenderer->isWmoWaterAt(camPos.x, camPos.y)) {
            bool canal = false;
            if (auto lt = waterRenderer->getWaterTypeAt(camPos.x, camPos.y))
                canal = (*lt == 5 || *lt == 13 || *lt == 17);
            // Until the eye passes the surface the view is darkened by
            // looking through the water plane itself, which is strong -
            // its alpha runs up towards 0.9 with depth. Once the eye is
            // under, that plane is behind the camera and contributes
            // nothing, so this overlay is all that is left. Starting it
            // near zero made submerging brighten the scene sharply, which
            // is backwards. Begin at a strength comparable to what the
            // surface was contributing and deepen from there.
            const float depth = std::max(eyeDepth, 0.0f);
            constexpr float kSurfaceHandoff = 0.38f;  // matches the plane's own darkening
            const float depthFog = 1.0f - std::exp(-depth * (canal ? 0.25f : 0.12f));
            float fogStrength = kSurfaceHandoff + depthFog * (0.75f - kSurfaceHandoff);
            fogStrength = glm::clamp(fogStrength, kSurfaceHandoff, 0.75f);
            glm::vec4 tint = canal
                ? glm::vec4(0.01f, 0.04f, 0.10f, fogStrength)
                : glm::vec4(0.03f, 0.09f, 0.18f, fogStrength);

            // The seam is worked out per pixel from the surface height, so
            // there is no screen-space line to place here. Once the eye is
            // well under, the near plane is entirely below the water and there
            // is no seam left to draw - say so, and the whole view tints.
            const bool crossing = eyeDepth < kCrossingBand;
            overlaySystem_->renderWaterline(
                tint,
                glm::inverse(camera->getViewProjectionMatrix()),
                *waterH,
                // Softness and ripple in world units now: how thick the seam is
                // in yards of water, which does not change with where the
                // camera is looking.
                0.030f,   // meniscus half-thickness
                0.004f,   // ripple on the surface height
                globalTime, crossing, cmd);
        }
    }
}

void Renderer::renderPostSceneOverlays(VkCommandBuffer cmd,
                                       game::GameHandler* gameHandler) {
    // Ghost mode desaturation: cold blue-grey overlay when dead/ghost
    if (ghostMode_ && overlaySystem_) {
        overlaySystem_->renderOverlay(glm::vec4(0.30f, 0.35f, 0.42f, 0.45f), cmd);
    }

    // Brightness overlay, applied before the minimap so it doesn't affect UI.
    if (overlaySystem_) {
        float br = postProcessPipeline_ ? postProcessPipeline_->getBrightness() : 1.0f;
        if (br < 0.99f) {
            // Black overlay at alpha (1-br) darkens as scene*br (a true multiply).
            overlaySystem_->renderOverlay(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f - br), cmd);
        } else if (br > 1.01f) {
            // Multiply scene by br instead of lerping to white (washout). The
            // water refraction shader divides br back out of its captured
            // scene sample so this doesn't compound through the history.
            overlaySystem_->renderBrightnessScale(br, cmd);
        }
    }

    if (minimap && minimap->isEnabled() && camera && window) {
        glm::vec3 minimapCenter = camera->getPosition();
        if (cameraController && cameraController->isThirdPerson())
            minimapCenter = characterPosition;
        float minimapPlayerOrientation = 0.0f;
        bool hasMinimapPlayerOrientation = false;
        if (cameraController) {
            // Render-space character yaw faces north at 180 degrees; the
            // minimap shader arrow faces north at 0. Match the mirrored
            // minimap texture by flipping the visual arrow vertically.
            minimapPlayerOrientation = glm::radians(characterYaw);
            hasMinimapPlayerOrientation = true;
        } else if (gameHandler) {
            // movementInfo.orientation is canonical yaw: north is 0, east is +pi/2.
            // Match the mirrored minimap texture by flipping the visual
            // arrow vertically.
            minimapPlayerOrientation = glm::pi<float>() - gameHandler->getMovementInfo().orientation;
            hasMinimapPlayerOrientation = true;
        }
        minimap->render(cmd, *camera, minimapCenter,
                        window->getWidth(), window->getHeight(),
                        minimapPlayerOrientation, hasMinimapPlayerOrientation);
    }
}

bool Renderer::waterDrawsInContinuePass() const {
    if (!waterRenderer || !vkCtx) return false;
    if (vkCtx->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT) {
        // The 1x water pass targets the swapchain directly, so it cannot serve a
        // frame whose scene went to an off-screen post-processing target.
        const bool offscreenScene =
            postProcessPipeline_ && postProcessPipeline_->getSceneFramebuffer() != VK_NULL_HANDLE;
        return !offscreenScene && waterRenderer->hasWater1xPass();
    }
    return vkCtx->getSceneContinueRenderPass() != VK_NULL_HANDLE;
}

// initPostProcess(), resizePostProcess(), shutdownPostProcess() removed -
// post-process pipeline is now handled by Vulkan (Phase 6 cleanup).

bool Renderer::initializeRenderers(pipeline::AssetManager* assetManager, const std::string& mapName) {
    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    LOG_INFO("Initializing renderers for map: ", mapName);

    // Scan for custom zones on first initialization
    if (customZones_.empty()) {
        customZones_ = pipeline::CustomZoneDiscovery::scan({"custom_zones", "output"});
        if (!customZones_.empty()) {
            LOG_INFO("=== Custom Zones Available ===");
            for (const auto& z : customZones_) {
                LOG_INFO("  ", z.name, " (", z.directory, ")",
                         z.hasCreatures ? " [NPCs]" : "",
                         z.hasQuests ? " [Quests]" : "");
            }
            LOG_INFO("==============================");
        }
    }

    // Create terrain renderer if not already created
    if (!terrainRenderer) {
        terrainRenderer = std::make_unique<TerrainRenderer>();
        if (!terrainRenderer->initialize(vkCtx, perFrameSetLayout, assetManager)) {
            LOG_ERROR("Failed to initialize terrain renderer");
            terrainRenderer.reset();
            return false;
        }
        if (shadowRenderPass != VK_NULL_HANDLE) {
            terrainRenderer->initializeShadow(shadowRenderPass);
        }
    } else if (!terrainRenderer->hasShadowPipeline() && shadowRenderPass != VK_NULL_HANDLE) {
        terrainRenderer->initializeShadow(shadowRenderPass);
    }

    // Create water renderer if not already created
    if (!waterRenderer) {
        waterRenderer = std::make_unique<WaterRenderer>();
        if (!waterRenderer->initialize(vkCtx, perFrameSetLayout)) {
            LOG_ERROR("Failed to initialize water renderer");
            waterRenderer.reset();
        }
    }

    // Create minimap if not already created
    if (!minimap) {
        minimap = std::make_unique<Minimap>();
        if (!minimap->initialize(vkCtx, perFrameSetLayout)) {
            LOG_ERROR("Failed to initialize minimap");
            minimap.reset();
        }
    }

    // Create world map if not already created
    if (!worldMap) {
        worldMap = std::make_unique<WorldMap>();
        if (!worldMap->initialize(vkCtx, assetManager)) {
            LOG_ERROR("Failed to initialize world map");
            worldMap.reset();
        }
    }

    // Create M2, WMO, and Character renderers
    if (!m2Renderer) {
        m2Renderer = std::make_unique<M2Renderer>();
        if (!m2Renderer->initialize(vkCtx, perFrameSetLayout, assetManager))
            LOG_ERROR("M2Renderer initialization failed");
        if (swimEffects) {
            swimEffects->setM2Renderer(m2Renderer.get());
        }
        // Initialize SpellVisualSystem once M2Renderer is available (§4.4)
        if (!spellVisualSystem_) {
            spellVisualSystem_ = std::make_unique<SpellVisualSystem>();
            spellVisualSystem_->initialize(m2Renderer.get(), this);
        }
    }

    // The original client's skies are camera-centered M2 models selected
    // through LightParams and LightSkybox, on every map that names one - not
    // Outland alone, which is where this was built and where it stayed. The
    // lookup was opened to all maps and this was not, so it went on doing
    // nothing anywhere else: without the renderer there is nothing to draw
    // into.
    //
    // It keeps its own no-depth renderer so the sky draws behind terrain and
    // never enters world collision.
    if (!skyboxModelRenderer_) {
        skyboxModelRenderer_ = std::make_unique<M2Renderer>();
        skyboxModelRenderer_->setSkyMode(true);
        if (!skyboxModelRenderer_->initialize(vkCtx, perFrameSetLayout, assetManager)) {
            LOG_WARNING("Sky M2 renderer initialization failed");
            skyboxModelRenderer_.reset();
        }
    }

    // HiZ occlusion culling disabled - the pyramid build + blocking fence was
    // the main frame-rate bottleneck.  GPU frustum culling alone provides good
    // draw-call reduction without the per-frame GPU stall.  HiZ can be re-
    // enabled once the pyramid build is moved to an async compute queue.
    if (!wmoRenderer) {
        wmoRenderer = std::make_unique<WMORenderer>();
        if (!wmoRenderer->initialize(vkCtx, perFrameSetLayout, assetManager))
            LOG_ERROR("WMORenderer initialization failed");
        if (shadowRenderPass != VK_NULL_HANDLE) {
            if (!wmoRenderer->initializeShadow(shadowRenderPass))
                LOG_WARNING("WMO shadow pipeline initialization failed");
        }
    }

    // Renderer components can be recreated during map transitions. Restore the
    // configured view distance instead of falling back to their defaults.
    setViewDistance(viewDistance_);
    setSharpStars(sharpStars_);

    // Initialize shadow pipelines for M2 if not yet done
    if (m2Renderer && shadowRenderPass != VK_NULL_HANDLE && !m2Renderer->hasShadowPipeline()) {
        if (!m2Renderer->initializeShadow(shadowRenderPass))
            LOG_WARNING("M2 shadow pipeline initialization failed");
    }
    if (!characterRenderer) {
        characterRenderer = std::make_unique<CharacterRenderer>();
        if (!characterRenderer->initialize(vkCtx, perFrameSetLayout, assetManager))
            LOG_ERROR("CharacterRenderer initialization failed");
        if (shadowRenderPass != VK_NULL_HANDLE) {
            if (!characterRenderer->initializeShadow(shadowRenderPass))
                LOG_WARNING("Character shadow pipeline initialization failed");
        }
    }

    // Initialize AnimationController (§4.2)
    if (!animationController_) {
        animationController_ = std::make_unique<AnimationController>();
        animationController_->initialize(this);
    }

    // Create and initialize terrain manager
    if (!terrainManager) {
        terrainManager = std::make_unique<TerrainManager>();
        if (!terrainManager->initialize(assetManager, terrainRenderer.get())) {
            LOG_ERROR("Failed to initialize terrain manager");
            terrainManager.reset();
            return false;
        }
        // Set water renderer for terrain streaming
        if (waterRenderer) {
            terrainManager->setWaterRenderer(waterRenderer.get());
        }
        // Set M2 renderer for doodad loading during streaming
        if (m2Renderer) {
            terrainManager->setM2Renderer(m2Renderer.get());
        }
        // Set WMO renderer for building loading during streaming
        if (wmoRenderer) {
            terrainManager->setWMORenderer(wmoRenderer.get());
        }
        // A WMO's child M2 doodads - a ship's sails, its paddlewheel - are moved
        // and destroyed through this pointer. It was never set, so every one of
        // those paths was behind a null check that never passed: the doodads
        // were created at the origin, never given their parent's transform, and
        // so drawn at the middle of the map rather than on the ship. Static
        // world doodads were unaffected, because terrain streaming places those
        // at their world position itself and never goes through the parent.
        if (wmoRenderer && m2Renderer) {
            wmoRenderer->setM2Renderer(m2Renderer.get());
        }
        // Set ambient sound manager for environmental audio emitters
        if (audioCoordinator_->getAmbientSoundManager()) {
            terrainManager->setAmbientSoundManager(audioCoordinator_->getAmbientSoundManager());
        }
        // Pass asset manager to character renderer for texture loading
        if (characterRenderer) {
            characterRenderer->setAssetManager(assetManager);
        }
        // Wire asset manager to minimap for tile texture loading
        if (minimap) {
            minimap->setAssetManager(assetManager);
        }
        // Wire terrain manager, WMO renderer, and water renderer to camera controller
        if (cameraController) {
            cameraController->setTerrainManager(terrainManager.get());
            if (wmoRenderer) {
                cameraController->setWMORenderer(wmoRenderer.get());
            }
            if (m2Renderer) {
                cameraController->setM2Renderer(m2Renderer.get());
            }
            if (waterRenderer) {
                cameraController->setWaterRenderer(waterRenderer.get());
            }
        }
    }

    // Set map name on sub-renderers
    if (terrainManager) terrainManager->setMapName(mapName);
    if (minimap) minimap->setMapName(mapName);
    if (worldMap) worldMap->setMapName(mapName);

    // Initialize audio managers
    if (audioCoordinator_->getMusicManager() && assetManager && !cachedAssetManager) {
        audio::AudioEngine::instance().setAssetManager(assetManager);
        audioCoordinator_->getMusicManager()->initialize(assetManager);
        if (audioCoordinator_->getFootstepManager()) {
            audioCoordinator_->getFootstepManager()->initialize(assetManager);
        }
        if (audioCoordinator_->getActivitySoundManager()) {
            audioCoordinator_->getActivitySoundManager()->initialize(assetManager);
        }
        if (audioCoordinator_->getMountSoundManager()) {
            audioCoordinator_->getMountSoundManager()->initialize(assetManager);
        }
        if (audioCoordinator_->getNpcVoiceManager()) {
            audioCoordinator_->getNpcVoiceManager()->initialize(assetManager);
        }
        if (audioCoordinator_->getPlayerVoiceManager()) {
            audioCoordinator_->getPlayerVoiceManager()->initialize(assetManager);
        }
        if (!deferredWorldInitEnabled_) {
            if (audioCoordinator_->getAmbientSoundManager()) {
                audioCoordinator_->getAmbientSoundManager()->initialize(assetManager);
            }
            if (audioCoordinator_->getUiSoundManager()) {
                audioCoordinator_->getUiSoundManager()->initialize(assetManager);
            }
            if (audioCoordinator_->getCombatSoundManager()) {
                audioCoordinator_->getCombatSoundManager()->initialize(assetManager);
            }
            if (audioCoordinator_->getSpellSoundManager()) {
                audioCoordinator_->getSpellSoundManager()->initialize(assetManager);
            }
            if (audioCoordinator_->getMovementSoundManager()) {
                audioCoordinator_->getMovementSoundManager()->initialize(assetManager);
            }
            if (questMarkerRenderer) {
                if (!questMarkerRenderer->initialize(vkCtx, perFrameSetLayout, assetManager))
                    LOG_WARNING("Quest marker renderer initialization failed (non-fatal)");
            }
            if (footprintRenderer) {
                if (!footprintRenderer->initialize(this, vkCtx, perFrameSetLayout, assetManager))
                    LOG_WARNING("Footprint renderer initialization failed (non-fatal)");
            }

            if (core::envFlagEnabled("WOWEE_PREWARM_ZONE_MUSIC", false)) {
                if (zoneManager) {
                    for (const auto& musicPath : zoneManager->getAllMusicPaths()) {
                        audioCoordinator_->getMusicManager()->preloadMusic(musicPath);
                    }
                }
                static const std::vector<std::string> tavernTracks = {
                    "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance01.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance02.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern1A.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern2A.mp3",
                };
                for (const auto& musicPath : tavernTracks) {
                    audioCoordinator_->getMusicManager()->preloadMusic(musicPath);
                }
            }
        } else {
            deferredWorldInitPending_ = true;
            deferredWorldInitStage_ = 0;
            deferredWorldInitCooldown_ = 0.25f;
        }

        cachedAssetManager = assetManager;

        // Enrich zone music from DBC if not already done (e.g. asset manager was null at init).
        if (zoneManager && assetManager) {
            zoneManager->enrichFromDBC(assetManager);
        }
    }

    // Snap camera to ground
    if (cameraController) {
        cameraController->reset();
    }

    return true;
}

bool Renderer::loadTestTerrain(pipeline::AssetManager* assetManager, const std::string& adtPath) {
    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    LOG_INFO("Loading test terrain: ", adtPath);

    // Extract map name from ADT path for renderer initialization
    std::string mapName;
    {
        size_t lastSep = adtPath.find_last_of("\\/");
        if (lastSep != std::string::npos) {
            std::string filename = adtPath.substr(lastSep + 1);
            size_t firstUnderscore = filename.find('_');
            mapName = filename.substr(0, firstUnderscore != std::string::npos ? firstUnderscore : filename.size());
        }
    }

    // Initialize all sub-renderers
    if (!initializeRenderers(assetManager, mapName)) {
        return false;
    }

    // Parse tile coordinates from ADT path
    // Format: World\Maps\{MapName}\{MapName}_{X}_{Y}.adt
    int tileX = 32, tileY = 49;  // defaults
    {
        size_t lastSep = adtPath.find_last_of("\\/");
        if (lastSep != std::string::npos) {
            std::string filename = adtPath.substr(lastSep + 1);
            size_t firstUnderscore = filename.find('_');
            if (firstUnderscore != std::string::npos) {
                size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
                if (secondUnderscore != std::string::npos) {
                    size_t dot = filename.find('.', secondUnderscore);
                    if (dot != std::string::npos) {
                        try {
                            tileX = std::stoi(filename.substr(firstUnderscore + 1, secondUnderscore - firstUnderscore - 1));
                            tileY = std::stoi(filename.substr(secondUnderscore + 1, dot - secondUnderscore - 1));
                        } catch (...) {
                            LOG_WARNING("Failed to parse tile coords from: ", filename);
                        }
                    }
                }
            }
        }
    }

    LOG_INFO("Enqueuing initial tile [", tileX, ",", tileY, "] via terrain manager");

    // Enqueue the initial tile for async loading (avoids long sync stalls)
    if (!terrainManager->enqueueTile(tileX, tileY)) {
        LOG_ERROR("Failed to enqueue initial tile [", tileX, ",", tileY, "]");
        return false;
    }

    terrainLoaded = true;

    LOG_INFO("Test terrain loaded successfully!");
    LOG_INFO("  Chunks: ", terrainRenderer->getChunkCount());
    LOG_INFO("  Triangles: ", terrainRenderer->getTriangleCount());

    return true;
}

void Renderer::setWireframeMode(bool enabled) {
    if (terrainRenderer) {
        terrainRenderer->setWireframe(enabled);
    }
}

// One line naming how far each of the three actually drew this frame.
//
// "Distant objects float with no terrain" is a disagreement between two
// distances, and every attempt to settle it by reading the code picked the
// wrong one of the three places that compute it. This prints the answer:
// if terrain stops short of the doodads the fault is in loading the tiles,
// and if the doodads run past the setting the fault is in the cull. It is at
// warning because the default log is warnings only, and it prints on a change
// of half a tile rather than every frame.
void Renderer::logViewDistanceDiag() {
    static const bool enabled = std::getenv("WOWEE_VIEW_DIAG") != nullptr;
    if (!enabled) return;

    const float terrainFurthest = terrainRenderer
        ? terrainRenderer->getFurthestDrawnDistance() : 0.0f;
    const float m2Furthest = m2Renderer
        ? m2Renderer->getFurthestDrawnDistance() : 0.0f;

    if (std::abs(terrainFurthest - diagTerrainFurthest_) < 266.0f &&
        std::abs(m2Furthest - diagM2Furthest_) < 266.0f) {
        return;
    }
    diagTerrainFurthest_ = terrainFurthest;
    diagM2Furthest_ = m2Furthest;

    LOG_WARNING("view distance ", static_cast<int>(viewDistance_),
                ": terrain drew to ", static_cast<int>(terrainFurthest),
                ", doodads to ", static_cast<int>(m2Furthest),
                ", tiles loaded to ", getTerrainLoadRadius(),
                " (", static_cast<int>(getTerrainLoadRadius() *
                                       core::coords::TILE_SIZE), ")");
}

void Renderer::setSharpStars(bool enabled) {
    sharpStars_ = enabled;
    // Two halves of one switch: the sky model stops drawing its star layer and
    // the client's own point stars take its place. Setting either alone gives a
    // sky with no stars or a sky with two sets of them.
    if (m2Renderer) m2Renderer->setSuppressBakedStars(sharpStars_);
    if (skySystem) skySystem->setProceduralStarsEnabled(sharpStars_);
}

void Renderer::setViewDistance(float distance) {
    viewDistance_ = glm::clamp(distance, 400.0f, 2400.0f);

    if (terrainRenderer) terrainRenderer->setViewDistance(viewDistance_);
    if (wmoRenderer) wmoRenderer->setViewDistance(viewDistance_);
    if (m2Renderer) m2Renderer->setViewDistance(viewDistance_);
    if (terrainManager) {
        terrainManager->setLoadRadius(getTerrainLoadRadius());
        terrainManager->setUnloadRadius(getTerrainUnloadRadius());
    }
}

int Renderer::getTerrainLoadRadius() const {
    constexpr float kAdtTileSize = core::coords::TILE_SIZE;
    return glm::clamp(static_cast<int>(std::ceil(viewDistance_ / kAdtTileSize)) + 1, 2, 6);
}
void Renderer::renderHUD() {
    if (currentCmd == VK_NULL_HANDLE) return;
    if (performanceHUD && camera) {
        performanceHUD->render(this, camera.get());
    }
}

// ──────────────────────────────────────────────────────
// Shadow mapping helpers
// ──────────────────────────────────────────────────────

// initShadowMap() and compileShadowShader() removed - shadow resources now created
// in createPerFrameResources() as part of the Vulkan shadow infrastructure.

glm::mat4 Renderer::computeLightSpaceMatrix() {
    const float kShadowHalfExtent = shadowDistance_;
    const float kShadowLightDistance = shadowDistance_ * 3.0f;
    constexpr float kShadowNearPlane = 1.0f;
    const float kShadowFarPlane = shadowDistance_ * 6.5f;

    // Use active lighting direction so shadow projection matches main shading.
    // Fragment shaders derive lighting with `ldir = normalize(-lightDir.xyz)`,
    // therefore shadow rays must use -directionalDir to stay aligned.
    glm::vec3 sunDir = glm::normalize(glm::vec3(-0.3f, -0.7f, -0.6f));
    if (lightingManager) {
        const auto& lighting = lightingManager->getLightingParams();
        float ldirLenSq = glm::dot(lighting.directionalDir, lighting.directionalDir);
        if (ldirLenSq > 1e-6f) {
            sunDir = -lighting.directionalDir * glm::inversesqrt(ldirLenSq);
        }
    }
    // Shadow camera expects light rays pointing downward in render space (Z up).
    // Some profiles/opcode paths provide the opposite convention; normalize here.
    if (sunDir.z > 0.0f) {
        sunDir = -sunDir;
    }
    // Keep a minimum downward component so the frustum doesn't collapse at grazing angles.
    if (sunDir.z > -0.15f) {
        sunDir.z = -0.15f;
        sunDir = glm::normalize(sunDir);
    }

    // LightingManager already smooths the directional light every frame. Keep
    // that continuous direction for the shadow projection as well. Quantizing
    // it into 0.5-degree steps made the entire 600-yard shadow footprint rotate
    // in a single frame, producing a wide light-switch flicker whenever the
    // threshold was crossed. Translation remains stabilized by texel snapping.

    // Shadow center follows the player directly; texel snapping below prevents
    // camera translation from shimmering the projection.
    glm::vec3 desiredCenter = characterPosition;
    if (!shadowCenterInitialized) {
        if (glm::dot(desiredCenter, desiredCenter) < 1.0f) {
            return glm::mat4(0.0f);
        }
        shadowCenterInitialized = true;
    }
    shadowCenter = desiredCenter;
    glm::vec3 center = shadowCenter;

    // Snap shadow frustum to texel grid so the projection is perfectly stable
    // while moving. We compute the light's right/up axes from the sun direction
    // (these are constant per frame regardless of center) and snap center along
    // them before building the view matrix.
    float halfExtent = kShadowHalfExtent;
    float texelWorld = (2.0f * halfExtent) / static_cast<float>(SHADOW_MAP_SIZE);

    // Stable light-space axes (independent of center position)
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    if (std::abs(glm::dot(sunDir, up)) > 0.99f) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    glm::vec3 lightRight = glm::normalize(glm::cross(sunDir, up));
    glm::vec3 lightUp = glm::normalize(glm::cross(lightRight, sunDir));

    // Snap center along light's right and up axes to align with texel grid.
    // This eliminates sub-texel shifts that cause shadow shimmer.
    float dotR = glm::dot(center, lightRight);
    float dotU = glm::dot(center, lightUp);
    dotR = std::floor(dotR / texelWorld) * texelWorld;
    dotU = std::floor(dotU / texelWorld) * texelWorld;
    float dotD = glm::dot(center, sunDir);  // depth axis unchanged
    center = lightRight * dotR + lightUp * dotU + sunDir * dotD;
    shadowCenter = center;

    glm::mat4 lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);
    glm::mat4 lightProj = glm::ortho(-halfExtent, halfExtent, -halfExtent, halfExtent,
                                     kShadowNearPlane, kShadowFarPlane);
    lightProj[1][1] *= -1.0f; // Vulkan Y-flip for shadow pass

    return lightProj * lightView;
}

void Renderer::setupWater1xPass() {
    if (!waterRenderer || !vkCtx) return;
    if (vkCtx->getMsaaSamples() == VK_SAMPLE_COUNT_1_BIT) {
        refreshSwimEffectsPass();  // scene continuation pass covers the water here
        return;
    }
    VkImageView depthView = vkCtx->getDepthResolveImageView();
    if (!depthView) {
        // Without a resolved depth buffer the single-sampled water has nothing
        // to depth test against, so it has to stay in the multisampled pass.
        LOG_WARNING("No depth resolve image available - water stays in the MSAA scene pass");
        refreshSwimEffectsPass();
        return;
    }

    waterRenderer->createWater1xPass(vkCtx->getSwapchainFormat(), vkCtx->getDepthFormat());
    waterRenderer->createWater1xFramebuffers(
        vkCtx->getSwapchainImageViews(), depthView, vkCtx->getSwapchainExtent());

    // The spray follows the water into its pass, and this is the first point at
    // which that pass exists - the swim effects were built long before it, back
    // when the only choice was the scene pass.
    refreshSwimEffectsPass();
}

// Rebuild the spray's pipelines if the pass it should draw into has changed.
// Only ever called with no frames in flight; recreatePipelines() destroys the
// old pipelines outright rather than deferring them.
void Renderer::refreshSwimEffectsPass() {
    if (!swimEffects || !vkCtx) return;
    const bool wasWithWater = swimEffectsDrawWithWater_;
    syncSwimEffectsTargetPass();
    if (swimEffectsDrawWithWater_ != wasWithWater) {
        vkDeviceWaitIdle(vkCtx->getDevice());
        swimEffects->recreatePipelines();
    }
}

// ========================= Multithreaded Secondary Command Buffers =========================

bool Renderer::createSecondaryCommandResources() {
    if (!vkCtx) return false;
    VkDevice device = vkCtx->getDevice();
    uint32_t queueFamily = vkCtx->getGraphicsQueueFamily();

    VkCommandPoolCreateInfo poolCI{};
    poolCI.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolCI.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolCI.queueFamilyIndex = queueFamily;

    // Create worker command pools (one per worker thread)
    for (uint32_t w = 0; w < NUM_WORKERS; ++w) {
        if (vkCreateCommandPool(device, &poolCI, nullptr, &workerCmdPools_[w]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create worker command pool ", w);
            return false;
        }
    }

    // Create main-thread secondary command pool
    if (vkCreateCommandPool(device, &poolCI, nullptr, &mainSecondaryCmdPool_) != VK_SUCCESS) {
        LOG_ERROR("Failed to create main secondary command pool");
        return false;
    }

    // Allocate secondary command buffers
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY;
    allocInfo.commandBufferCount = 1;

    // Each concurrently recorded worker secondary owns a dedicated command pool.
    const uint32_t workerSecondaries[] = { SEC_TERRAIN, SEC_WMO, SEC_CHARS, SEC_M2, SEC_POST };
    for (uint32_t w = 0; w < NUM_WORKERS; ++w) {
        allocInfo.commandPool = workerCmdPools_[w];
        for (uint32_t f = 0; f < MAX_FRAMES; ++f) {
            if (vkAllocateCommandBuffers(device, &allocInfo, &secondaryCmds_[workerSecondaries[w]][f]) != VK_SUCCESS) {
                LOG_ERROR("Failed to allocate worker secondary buffer w=", w, " f=", f);
                return false;
            }
        }
    }

    const uint32_t mainSecondaries[] = { SEC_SKY, SEC_SELECTION, SEC_IMGUI };
    for (uint32_t idx : mainSecondaries) {
        allocInfo.commandPool = mainSecondaryCmdPool_;
        for (uint32_t f = 0; f < MAX_FRAMES; ++f) {
            if (vkAllocateCommandBuffers(device, &allocInfo, &secondaryCmds_[idx][f]) != VK_SUCCESS) {
                LOG_ERROR("Failed to allocate main secondary buffer idx=", idx, " f=", f);
                return false;
            }
        }
    }

    parallelRecordingEnabled_ = true;
    LOG_INFO("Multithreaded rendering: ", NUM_WORKERS, " worker threads, ",
             NUM_SECONDARIES, " secondary buffers [ENABLED]");
    return true;
}

void Renderer::destroySecondaryCommandResources() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    vkDeviceWaitIdle(device);

    // Secondary buffers are freed when their pool is destroyed
    for (auto& workerCmdPool : workerCmdPools_) {
        if (workerCmdPool) {
            vkDestroyCommandPool(device, workerCmdPool, nullptr);
            workerCmdPool = VK_NULL_HANDLE;
        }
    }
    if (mainSecondaryCmdPool_) {
        vkDestroyCommandPool(device, mainSecondaryCmdPool_, nullptr);
        mainSecondaryCmdPool_ = VK_NULL_HANDLE;
    }

    for (auto& arr : secondaryCmds_)
        for (auto& cmd : arr)
            cmd = VK_NULL_HANDLE;

    parallelRecordingEnabled_ = false;
}

VkCommandBuffer Renderer::beginSecondary(uint32_t secondaryIndex) {
    uint32_t frame = vkCtx->getCurrentFrame();
    VkCommandBuffer cmd = secondaryCmds_[secondaryIndex][frame];

    VkCommandBufferInheritanceInfo inheritInfo{};
    inheritInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO;
    inheritInfo.renderPass = activeRenderPass_;
    inheritInfo.subpass = 0;
    inheritInfo.framebuffer = activeFramebuffer_;

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
                    | VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT;
    beginInfo.pInheritanceInfo = &inheritInfo;

    VkResult result = vkBeginCommandBuffer(cmd, &beginInfo);
    if (result != VK_SUCCESS) {
        LOG_ERROR("vkBeginCommandBuffer failed for secondary ", secondaryIndex,
                  " frame ", frame, " result=", static_cast<int>(result));
    }
    return cmd;
}

void Renderer::setSecondaryViewportScissor(VkCommandBuffer cmd) {
    VkViewport vp{};
    vp.width = static_cast<float>(activeRenderExtent_.width);
    vp.height = static_cast<float>(activeRenderExtent_.height);
    vp.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &vp);

    VkRect2D sc{};
    sc.extent = activeRenderExtent_;
    vkCmdSetScissor(cmd, 0, 1, &sc);
}

void Renderer::renderReflectionPass() {
    if (!waterRenderer || !camera || !waterRenderer->hasReflectionPass() || !waterRenderer->hasSurfaces()) return;
    if (currentCmd == VK_NULL_HANDLE || !reflPerFrameUBOMapped) return;

    // Select the current frame's pre-bound reflection descriptor set
    // (each frame's set was bound to its own shadow depth view at init).
    uint32_t frame = vkCtx->getCurrentFrame();
    VkDescriptorSet reflDescSet = reflPerFrameDescSet[frame];

    // Reflection pass uses 1x MSAA. Scene pipelines must be render-pass-compatible,
    // which requires matching sample counts. Only render scene into reflection when MSAA is off.
    bool canRenderScene = (vkCtx->getMsaaSamples() == VK_SAMPLE_COUNT_1_BIT);

    // Find dominant water height near camera
    const glm::vec3 camPos = camera->getPosition();
    auto waterH = waterRenderer->getDominantWaterHeight(camPos);
    if (!waterH) return;

    float waterHeight = *waterH;

    // Skip reflection if camera is underwater (Z is up)
    if (camPos.z < waterHeight + 0.5f) return;

    // Compute reflected view and oblique projection
    glm::mat4 reflView = WaterRenderer::computeReflectedView(*camera, waterHeight);
    glm::mat4 reflProj = WaterRenderer::computeObliqueProjection(
        camera->getProjectionMatrix(), reflView, waterHeight);

    // Update water renderer's reflection UBO with the reflected viewProj
    waterRenderer->updateReflectionUBO(reflProj * reflView);

    // Fill the reflection per-frame UBO (same as normal but with reflected matrices)
    GPUPerFrameData reflData = currentFrameData;
    reflData.view = reflView;
    reflData.projection = reflProj;
    // Reflected camera position (Z is up)
    glm::vec3 reflPos = camPos;
    reflPos.z = 2.0f * waterHeight - reflPos.z;
    reflData.viewPos = glm::vec4(reflPos, 1.0f);
    std::memcpy(reflPerFrameUBOMapped, &reflData, sizeof(GPUPerFrameData));

    // Begin reflection render pass (clears to black; scene rendered if pipeline-compatible)
    if (!waterRenderer->beginReflectionPass(currentCmd)) return;

    if (canRenderScene) {
        // Render scene into reflection texture (sky + terrain + WMO only for perf)
        if (skySystem) {
            rendering::SkyParams skyParams;
            auto* reflSkybox = skySystem->getSkybox();
            skyParams.timeOfDay = lightingManager
                ? lightingManager->getVisualTimeOfDayHours()
                : (reflSkybox ? reflSkybox->getTimeOfDay() : 12.0f);
            if (lightingManager) {
                const auto& lp = lightingManager->getLightingParams();
                skyParams.directionalDir = lp.directionalDir;
                skyParams.sunColor = lp.diffuseColor;
                skyParams.skyTopColor = lp.skyTopColor;
                skyParams.skyMiddleColor = lp.skyMiddleColor;
                skyParams.skyBand1Color = lp.skyBand1Color;
                skyParams.skyBand2Color = lp.skyBand2Color;
                skyParams.cloudDensity = lp.cloudDensity;
                skyParams.fogDensity = lp.fogDensity;
                skyParams.horizonGlow = lp.horizonGlow;
            }
            // weatherIntensity left at default 0 for reflection pass (no game handler in scope)
            skySystem->render(currentCmd, reflDescSet, *camera, skyParams);
        }
        if (terrainRenderer && terrainEnabled) {
            terrainRenderer->render(currentCmd, reflDescSet, *camera);
        }
        if (wmoRenderer) {
            wmoRenderer->render(currentCmd, reflDescSet, *camera);
        }
    }

    waterRenderer->endReflectionPass(currentCmd);
}

void Renderer::renderShadowPass() {
    ZoneScopedN("Renderer::renderShadowPass");
    static const bool skipShadows = (std::getenv("WOWEE_SKIP_SHADOWS") != nullptr);
    if (skipShadows) return;
    if (shadowDepthImage[0] == VK_NULL_HANDLE) return;
    if (currentCmd == VK_NULL_HANDLE) return;
    // Shadows off still runs the pass, and the pass still clears the map and
    // leaves it in the layout its readers expect - it simply draws nothing
    // into it. Returning here instead left the image untransitioned while it
    // stayed bound for sampling, which is the shape of fault that takes the
    // device down rather than drawing something wrong.
    const bool drawCasters = shadowsEnabled;

    // Shadows render every frame - throttling causes visible flicker on player/NPCs

    // lightSpaceMatrix was already computed at frame start (before updatePerFrameUBO).
    // Zero matrix means character position isn't set yet - skip shadow pass entirely.
    if (lightSpaceMatrix == glm::mat4(0.0f)) return;
    uint32_t frame = vkCtx->getCurrentFrame();

    // Barrier 1: transition this frame's shadow map into writable depth layout.
    VkImageMemoryBarrier2 b1{};
    b1.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    b1.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    b1.oldLayout = shadowDepthLayout_[frame];
    b1.newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    b1.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b1.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b1.srcAccessMask = (shadowDepthLayout_[frame] == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        ? VK_ACCESS_SHADER_READ_BIT
        : 0;
    b1.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                       VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    b1.image = shadowDepthImage[frame];
    b1.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    VkPipelineStageFlags srcStage = (shadowDepthLayout_[frame] == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        ? VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT
        : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    b1.srcStageMask = srcStage;
    VkDependencyInfo b1Dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    b1Dep.dependencyFlags = 0;
    b1Dep.imageMemoryBarrierCount = 1;
    b1Dep.pImageMemoryBarriers = &b1;
    cmdPipelineBarrier2(currentCmd, b1Dep);

    // Begin shadow render pass
    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = shadowRenderPass;
    rpInfo.framebuffer = shadowFramebuffer[frame];
    rpInfo.renderArea = {.offset = {.x = 0, .y = 0}, .extent = {.width = SHADOW_MAP_SIZE, .height = SHADOW_MAP_SIZE}};
    VkClearValue clear{};
    clear.depthStencil = {.depth = 1.0f, .stencil = 0};
    rpInfo.clearValueCount = 1;
    rpInfo.pClearValues = &clear;
    vkCmdBeginRenderPass(currentCmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{.x = 0, .y = 0, .width = static_cast<float>(SHADOW_MAP_SIZE), .height = static_cast<float>(SHADOW_MAP_SIZE), .minDepth = 0.0f, .maxDepth = 1.0f};
    vkCmdSetViewport(currentCmd, 0, 1, &vp);
    VkRect2D sc{.offset = {.x = 0, .y = 0}, .extent = {.width = SHADOW_MAP_SIZE, .height = SHADOW_MAP_SIZE}};
    vkCmdSetScissor(currentCmd, 0, 1, &sc);

    // Phase 7/8: render shadow casters
    const float shadowCullRadius = shadowDistance_ * 1.35f;
    // With shadows off the pass still begins and ends, so the map is cleared
    // and left where its readers expect it; only the casters are skipped.
    if (drawCasters) {
    if (terrainRenderer) {
        terrainRenderer->renderShadow(currentCmd, lightSpaceMatrix, shadowCenter, shadowCullRadius);
    }
    if (wmoRenderer) {
        wmoRenderer->renderShadow(currentCmd, lightSpaceMatrix, shadowCenter, shadowCullRadius);
    }
    if (m2Renderer) {
        m2Renderer->renderShadow(currentCmd, lightSpaceMatrix, globalTime, shadowCenter, shadowCullRadius);
    }
    if (characterRenderer) {
        characterRenderer->renderShadow(currentCmd, lightSpaceMatrix, shadowCenter, shadowCullRadius);
    }
    }  // drawCasters

    vkCmdEndRenderPass(currentCmd);

    // Barrier 2: DEPTH_STENCIL_ATTACHMENT_OPTIMAL → SHADER_READ_ONLY_OPTIMAL
    VkImageMemoryBarrier2 b2{};
    b2.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    b2.srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    b2.dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    b2.oldLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    b2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    b2.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b2.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b2.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    b2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    b2.image = shadowDepthImage[frame];
    b2.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
    VkDependencyInfo b2Dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    b2Dep.dependencyFlags = 0;
    b2Dep.imageMemoryBarrierCount = 1;
    b2Dep.pImageMemoryBarriers = &b2;
    cmdPipelineBarrier2(currentCmd, b2Dep);
    shadowDepthLayout_[frame] = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
}

// Build the per-frame render graph for off-screen pre-passes.
// Declares passes as graph nodes with input/output dependencies.
// compile() performs topological sort; execute() runs them with auto barriers.
void Renderer::buildFrameGraph(game::GameHandler* gameHandler) {
    (void)gameHandler;
    if (!renderGraph_) return;

    renderGraph_->reset();

    auto shadowDepth = renderGraph_->findResource("shadow_depth");
    auto reflTex = renderGraph_->findResource("reflection_texture");

    // Minimap composites (no dependencies - standalone off-screen render target)
    renderGraph_->addPass("minimap_composite", {}, {},
        [this](VkCommandBuffer cmd) {
            if (minimap && minimap->isEnabled() && camera) {
                glm::vec3 minimapCenter = camera->getPosition();
                if (cameraController && cameraController->isThirdPerson())
                    minimapCenter = characterPosition;
                minimap->compositePass(cmd, minimapCenter);
            }
        });

    // World map composite (standalone)
    renderGraph_->addPass("worldmap_composite", {}, {},
        [this](VkCommandBuffer cmd) {
            if (worldMap) worldMap->compositePass(cmd);
        });

    // Character preview composites (standalone)
    renderGraph_->addPass("preview_composite", {}, {},
        [this](VkCommandBuffer cmd) {
            uint32_t frame = vkCtx->getCurrentFrame();
            for (auto* preview : activePreviews_) {
                if (preview && preview->isModelLoaded())
                    preview->compositePass(cmd, frame);
            }
        });

    // Shadow pre-pass → outputs shadow_depth
    renderGraph_->addPass("shadow_pass", {}, {shadowDepth},
        [this](VkCommandBuffer) {
            // Not gated on shadowsEnabled: renderShadowPass is what clears the
            // map and leaves it in the layout its readers expect, and it
            // already skips the casters on its own when shadows are off.
            // Declining to call it here put the transition back where it was
            // before, which is the whole of the fault this was meant to end.
            if (shadowDepthImage[0] != VK_NULL_HANDLE)
                renderShadowPass();
        });
    // Left enabled even with shadows off, as long as the image exists.
    //
    // A disabled pass is skipped whole, and that includes the image barriers
    // declared on it - so turning shadows off stopped the shadow map ever
    // being transitioned, while the passes that read it kept it bound and
    // sampled it in whatever layout it was last left in. The lambda above
    // already declines to draw anything; what has to keep happening is the
    // transition.
    renderGraph_->setPassEnabled("shadow_pass", shadowDepthImage[0] != VK_NULL_HANDLE);

    // Reflection pre-pass → outputs reflection_texture (reads scene, so after shadow)
    renderGraph_->addPass("reflection_pass", {shadowDepth}, {reflTex},
        [this](VkCommandBuffer) {
            renderReflectionPass();
        });

    renderGraph_->compile();
}

} // namespace rendering
} // namespace wowee
