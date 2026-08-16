#include "rendering/charge_effect.hpp"
#include "rendering/camera.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/m2_renderer.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <random>
#include <cmath>
#include <cstring>

namespace wowee {
namespace rendering {

static std::mt19937& rng() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}

static float randFloat(float lo, float hi) {
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(rng());
}

ChargeEffect::ChargeEffect() = default;
ChargeEffect::~ChargeEffect() { shutdown(); }

/// The ribbon pipeline: the trail a charging unit leaves behind it.
///
/// initialize() and recreatePipelines() described this and the dust pipeline
/// below identically, vertex layout included, so each was stated twice in one
/// file. The layouts are the part worth having once: these are packed floats
/// with no struct behind them, so a stride and an offset are all that say what
/// a vertex is.
void ChargeEffect::buildRibbonPipeline(VkDevice device,
                                       const VkPipelineShaderStageCreateInfo& vertStage,
                                       const VkPipelineShaderStageCreateInfo& fragStage) {
    const std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    // Vertex input: pos(vec3) + alpha(float) + heat(float) + height(float) = 6 floats, stride = 24 bytes
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = 6 * sizeof(float);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> attrs(4);
    // location 0: vec3 position
    attrs[0].location = 0;
    attrs[0].binding = 0;
    attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attrs[0].offset = 0;
    // location 1: float alpha
    attrs[1].location = 1;
    attrs[1].binding = 0;
    attrs[1].format = VK_FORMAT_R32_SFLOAT;
    attrs[1].offset = 3 * sizeof(float);
    // location 2: float heat
    attrs[2].location = 2;
    attrs[2].binding = 0;
    attrs[2].format = VK_FORMAT_R32_SFLOAT;
    attrs[2].offset = 4 * sizeof(float);
    // location 3: float height
    attrs[3].location = 3;
    attrs[3].binding = 0;
    attrs[3].format = VK_FORMAT_R32_SFLOAT;
    attrs[3].offset = 5 * sizeof(float);

    ribbonPipeline_ = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS)
        .setColorBlendAttachment(PipelineBuilder::blendAdditive())  // Additive blend for fiery glow
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(ribbonPipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());
}

/// The dust pipeline: the motes thrown up along the charge.
void ChargeEffect::buildDustPipeline(VkDevice device,
                                     const VkPipelineShaderStageCreateInfo& vertStage,
                                     const VkPipelineShaderStageCreateInfo& fragStage) {
    const std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    // Vertex input: pos(vec3) + size(float) + alpha(float) = 5 floats, stride = 20 bytes
    VkVertexInputBindingDescription binding = tightVertexBinding(5 * sizeof(float));
    std::vector<VkVertexInputAttributeDescription> attrs = positionPlusTwoFloatsAttrs();

    dustPipeline_ = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(dustPipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx_->getPipelineCache());
}

bool ChargeEffect::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    vkCtx_ = ctx;
    VkDevice device = vkCtx_->getDevice();


    // ---- Ribbon trail pipeline (TRIANGLE_STRIP) ----
    {
        auto shaders = loadShaderPair(device, "assets/shaders/charge_ribbon.vert.spv", "assets/shaders/charge_ribbon.frag.spv", "charge_ribbon");
        if (!shaders) return false;
        const auto& vertStage = shaders.vertStage;
        const auto& fragStage = shaders.fragStage;

        ribbonPipelineLayout_ = createPipelineLayout(device, {perFrameLayout}, {});
        if (ribbonPipelineLayout_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge ribbon pipeline layout");
            return false;
        }

        buildRibbonPipeline(device, vertStage, fragStage);


        if (ribbonPipeline_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge ribbon pipeline");
            return false;
        }
    }

    // ---- Dust puff pipeline (POINT_LIST) ----
    {
        auto shaders = loadShaderPair(device, "assets/shaders/charge_dust.vert.spv", "assets/shaders/charge_dust.frag.spv", "charge_dust");
        if (!shaders) return false;
        const auto& vertStage = shaders.vertStage;
        const auto& fragStage = shaders.fragStage;

        dustPipelineLayout_ = createPipelineLayout(device, {perFrameLayout}, {});
        if (dustPipelineLayout_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge dust pipeline layout");
            return false;
        }

        buildDustPipeline(device, vertStage, fragStage);


        if (dustPipeline_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge dust pipeline");
            return false;
        }
    }

    // ---- Create dynamic mapped vertex buffers ----
    // Ribbon: MAX_TRAIL_POINTS * 2 vertices * 6 floats each
    ribbonDynamicVBSize_ = MAX_TRAIL_POINTS * 2 * 6 * sizeof(float);
    {
        AllocatedBuffer buf = createBuffer(vkCtx_->getAllocator(), ribbonDynamicVBSize_,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
        ribbonDynamicVB_ = buf.buffer;
        ribbonDynamicVBAlloc_ = buf.allocation;
        ribbonDynamicVBAllocInfo_ = buf.info;
        if (ribbonDynamicVB_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge ribbon dynamic vertex buffer");
            return false;
        }
    }

    // Dust: MAX_DUST * 5 floats each
    dustDynamicVBSize_ = MAX_DUST * 5 * sizeof(float);
    {
        AllocatedBuffer buf = createBuffer(vkCtx_->getAllocator(), dustDynamicVBSize_,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
        dustDynamicVB_ = buf.buffer;
        dustDynamicVBAlloc_ = buf.allocation;
        dustDynamicVBAllocInfo_ = buf.info;
        if (dustDynamicVB_ == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create charge dust dynamic vertex buffer");
            return false;
        }
    }

    ribbonVerts_.reserve(MAX_TRAIL_POINTS * 2 * 6);
    dustVerts_.reserve(MAX_DUST * 5);
    dustPuffs_.reserve(MAX_DUST);

    return true;
}

void ChargeEffect::shutdown() {
    if (vkCtx_) {
        VkDevice device = vkCtx_->getDevice();
        VmaAllocator allocator = vkCtx_->getAllocator();

        destroy(device, ribbonPipeline_);
        destroy(device, ribbonPipelineLayout_);
        destroy(allocator, ribbonDynamicVB_, ribbonDynamicVBAlloc_);

        destroy(device, dustPipeline_);
        destroy(device, dustPipelineLayout_);
        destroy(allocator, dustDynamicVB_, dustDynamicVBAlloc_);
    }

    vkCtx_ = nullptr;
    trail_.clear();
    dustPuffs_.clear();
}

void ChargeEffect::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old pipelines (NOT layouts)
    destroy(device, ribbonPipeline_);
    destroy(device, dustPipeline_);


    // ---- Rebuild ribbon trail pipeline (TRIANGLE_STRIP) ----
    {
        auto shaders = loadShaderPair(device, "assets/shaders/charge_ribbon.vert.spv", "assets/shaders/charge_ribbon.frag.spv", "charge_ribbon");
        if (!shaders) return;
        const auto& vertStage = shaders.vertStage;
        const auto& fragStage = shaders.fragStage;

        buildRibbonPipeline(device, vertStage, fragStage);

    }

    // ---- Rebuild dust puff pipeline (POINT_LIST) ----
    {
        auto shaders = loadShaderPair(device, "assets/shaders/charge_dust.vert.spv", "assets/shaders/charge_dust.frag.spv", "charge_dust");
        if (!shaders) return;
        const auto& vertStage = shaders.vertStage;
        const auto& fragStage = shaders.fragStage;

        buildDustPipeline(device, vertStage, fragStage);

    }
}

void ChargeEffect::tryLoadM2Models(M2Renderer* m2Renderer, pipeline::AssetManager* assets) {
    if (!m2Renderer || !assets) return;
    m2Renderer_ = m2Renderer;

    const char* casterPaths[] = {
        "Spells\\Charge_Caster.m2",
        "Spells\\WarriorCharge.m2",
        "Spells\\Charge\\Charge_Caster.m2",
        "Spells\\Dust_Medium.m2",
    };
    for (const char* path : casterPaths) {
        auto m2Data = assets->readFile(path);
        if (m2Data.empty()) continue;
        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.name.empty()) model.name = path;
        if (model.vertices.empty() && model.particleEmitters.empty()) continue;
        {
            auto skinData = assets->readFile(pipeline::skinPathForM2(path));
            if (!skinData.empty() && model.version >= 264)
                pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (m2Renderer_->loadModel(model, CASTER_MODEL_ID)) {
            casterModelLoaded_ = true;
            LOG_INFO("ChargeEffect: loaded caster model from ", path);
            break;
        }
    }

    const char* impactPaths[] = {
        "Spells\\Charge_Impact.m2",
        "Spells\\Charge\\Charge_Impact.m2",
        "Spells\\ImpactDust.m2",
    };
    for (const char* path : impactPaths) {
        auto m2Data = assets->readFile(path);
        if (m2Data.empty()) continue;
        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.name.empty()) model.name = path;
        if (model.vertices.empty() && model.particleEmitters.empty()) continue;
        {
            auto skinData = assets->readFile(pipeline::skinPathForM2(path));
            if (!skinData.empty() && model.version >= 264)
                pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (m2Renderer_->loadModel(model, IMPACT_MODEL_ID)) {
            impactModelLoaded_ = true;
            LOG_INFO("ChargeEffect: loaded impact model from ", path);
            break;
        }
    }
}

void ChargeEffect::start(const glm::vec3& position, const glm::vec3& direction) {
    emitting_ = true;
    dustAccum_ = 0.0f;
    trail_.clear();
    dustPuffs_.clear();
    lastEmitPos_ = position;

    // Spawn M2 caster effect
    if (casterModelLoaded_ && m2Renderer_) {
        activeCasterInstanceId_ = m2Renderer_->createInstance(
            CASTER_MODEL_ID, position, glm::vec3(0.0f), 1.0f);
    }

    // Seed the first trail point
    emit(position, direction);
}

void ChargeEffect::emit(const glm::vec3& position, const glm::vec3& direction) {
    if (!emitting_) return;

    // Move M2 caster with player
    if (activeCasterInstanceId_ != 0 && m2Renderer_) {
        m2Renderer_->setInstancePosition(activeCasterInstanceId_, position);
    }

    // Only add a new trail point if we've moved enough
    glm::vec3 emitDelta = position - lastEmitPos_;
    float distSq = glm::dot(emitDelta, emitDelta);
    if (distSq >= TRAIL_SPAWN_DIST * TRAIL_SPAWN_DIST || trail_.empty()) {
        // Ribbon is vertical: side vector points straight up
        glm::vec3 side = glm::vec3(0.0f, 0.0f, 1.0f);

        // Trail spawns at character's mid-height (ribbon extends above and below)
        glm::vec3 trailCenter = position + glm::vec3(0.0f, 0.0f, 1.0f);

        trail_.push_back({.center = trailCenter, .side = side, .age = 0.0f});
        if (trail_.size() > MAX_TRAIL_POINTS) {
            trail_.pop_front();
        }
        lastEmitPos_ = position;
    }

    // Spawn dust puffs at feet
    glm::vec3 horizDir = glm::vec3(direction.x, direction.y, 0.0f);
    float horizLenSq = glm::dot(horizDir, horizDir);
    // Skip dust when character is nearly stationary - prevents NaN from inversesqrt(0)
    if (horizLenSq < 1e-6f) return;
    float invHorizLen = glm::inversesqrt(horizLenSq);
    glm::vec3 backDir = -horizDir * invHorizLen;
    glm::vec3 sideDir = glm::vec3(-backDir.y, backDir.x, 0.0f);

    // Accumulate ~0.48 per frame at 60fps (30 particles/sec * 16ms); emit when >= 1.0
    dustAccum_ += 30.0f * 0.016f;
    while (dustAccum_ >= 1.0f && dustPuffs_.size() < MAX_DUST) {
        dustAccum_ -= 1.0f;
        DustPuff d;
        d.position = position + backDir * randFloat(0.0f, 0.6f) +
                     sideDir * randFloat(-0.4f, 0.4f) +
                     glm::vec3(0.0f, 0.0f, 0.1f);
        d.velocity = backDir * randFloat(0.5f, 2.0f) +
                     sideDir * randFloat(-0.3f, 0.3f) +
                     glm::vec3(0.0f, 0.0f, randFloat(0.8f, 2.0f));
        d.lifetime = 0.0f;
        d.maxLifetime = randFloat(0.3f, 0.5f);
        d.size = randFloat(5.0f, 10.0f);
        d.alpha = 1.0f;
        dustPuffs_.push_back(d);
    }
}

void ChargeEffect::stop() {
    emitting_ = false;

    if (activeCasterInstanceId_ != 0 && m2Renderer_) {
        m2Renderer_->removeInstance(activeCasterInstanceId_);
        activeCasterInstanceId_ = 0;
    }
}
void ChargeEffect::update(float deltaTime) {
    // Age trail points and remove expired ones
    for (auto& tp : trail_) {
        tp.age += deltaTime;
    }
    while (!trail_.empty() && trail_.front().age >= TRAIL_LIFETIME) {
        trail_.pop_front();
    }

    // Update dust puffs
    for (auto it = dustPuffs_.begin(); it != dustPuffs_.end(); ) {
        it->lifetime += deltaTime;
        if (it->lifetime >= it->maxLifetime) {
            it = dustPuffs_.erase(it);
            continue;
        }
        it->position += it->velocity * deltaTime;
        it->velocity *= 0.93f;
        float t = it->lifetime / it->maxLifetime;
        it->alpha = 1.0f - t * t;
        it->size += deltaTime * 8.0f;
        ++it;
    }

    // Clean up expired M2 impacts
    for (auto it = activeImpacts_.begin(); it != activeImpacts_.end(); ) {
        it->elapsed += deltaTime;
        if (it->elapsed >= M2_EFFECT_DURATION) {
            if (m2Renderer_) m2Renderer_->removeInstance(it->instanceId);
            it = activeImpacts_.erase(it);
        } else {
            ++it;
        }
    }
}

void ChargeEffect::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet) {
    VkDeviceSize offset = 0;

    // ---- Render ribbon trail as triangle strip ----
    if (trail_.size() >= 2 && ribbonPipeline_ != VK_NULL_HANDLE) {
        ribbonVerts_.clear();

        int n = static_cast<int>(trail_.size());
        for (int i = 0; i < n; i++) {
            const auto& tp = trail_[i];
            float ageFrac = tp.age / TRAIL_LIFETIME;      // 0 = fresh, 1 = about to expire
            float positionFrac = static_cast<float>(i) / static_cast<float>(n - 1);  // 0 = tail, 1 = head

            // Alpha: fade out by age and also taper toward the tail end
            float alpha = (1.0f - ageFrac) * std::min(positionFrac * 3.0f, 1.0f);
            // Heat: hotter near the head (character), cooler at the tail
            float heat = positionFrac;

            // Width tapers: thin at tail, full at head
            float width = TRAIL_HALF_WIDTH * std::min(positionFrac * 2.0f, 1.0f);

            // Two vertices: bottom (center - up*width) and top (center + up*width)
            glm::vec3 bottom = tp.center - tp.side * width;
            glm::vec3 top    = tp.center + tp.side * width;

            // Bottom vertex (height=0, more transparent)
            ribbonVerts_.push_back(bottom.x);
            ribbonVerts_.push_back(bottom.y);
            ribbonVerts_.push_back(bottom.z);
            ribbonVerts_.push_back(alpha);
            ribbonVerts_.push_back(heat);
            ribbonVerts_.push_back(0.0f);  // height = bottom

            // Top vertex (height=1, redder and more opaque)
            ribbonVerts_.push_back(top.x);
            ribbonVerts_.push_back(top.y);
            ribbonVerts_.push_back(top.z);
            ribbonVerts_.push_back(alpha);
            ribbonVerts_.push_back(heat);
            ribbonVerts_.push_back(1.0f);  // height = top
        }

        // Upload to mapped buffer
        VkDeviceSize uploadSize = ribbonVerts_.size() * sizeof(float);
        if (uploadSize > 0 && ribbonDynamicVBAllocInfo_.pMappedData) {
            std::memcpy(ribbonDynamicVBAllocInfo_.pMappedData, ribbonVerts_.data(), uploadSize);
        }

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ribbonPipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ribbonPipelineLayout_,
            0, 1, &perFrameSet, 0, nullptr);
        vkCmdBindVertexBuffers(cmd, 0, 1, &ribbonDynamicVB_, &offset);
        vkCmdDraw(cmd, static_cast<uint32_t>(n * 2), 1, 0, 0);
    }

    // ---- Render dust puffs ----
    if (!dustPuffs_.empty() && dustPipeline_ != VK_NULL_HANDLE) {
        dustVerts_.clear();
        for (const auto& d : dustPuffs_) {
            dustVerts_.push_back(d.position.x);
            dustVerts_.push_back(d.position.y);
            dustVerts_.push_back(d.position.z);
            dustVerts_.push_back(d.size);
            dustVerts_.push_back(d.alpha);
        }

        // Upload to mapped buffer
        VkDeviceSize uploadSize = dustVerts_.size() * sizeof(float);
        if (uploadSize > 0 && dustDynamicVBAllocInfo_.pMappedData) {
            std::memcpy(dustDynamicVBAllocInfo_.pMappedData, dustVerts_.data(), uploadSize);
        }

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, dustPipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, dustPipelineLayout_,
            0, 1, &perFrameSet, 0, nullptr);
        vkCmdBindVertexBuffers(cmd, 0, 1, &dustDynamicVB_, &offset);
        vkCmdDraw(cmd, static_cast<uint32_t>(dustPuffs_.size()), 1, 0, 0);
    }
}

} // namespace rendering
} // namespace wowee
