#include "rendering/footprint_renderer.hpp"
#include "pipeline/m2_loader.hpp"

#include "core/logger.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/wmo_renderer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <unordered_map>

#include <glm/gtc/matrix_transform.hpp>

namespace wowee::rendering {
namespace {

constexpr float kLifetime = 12.0f;
constexpr float kFadeStart = 1.5f;
constexpr size_t kMaxPrints = 72;

struct FootprintPushConstants {
    glm::mat4 model{1.0f};
    glm::vec4 tint{0.075f, 0.065f, 0.05f, 1.0f};
};

std::string normalizeModelPath(std::string path) {
    std::transform(path.begin(), path.end(), path.begin(), [](unsigned char c) {
        if (c == '/') return '\\';
        return static_cast<char>(std::tolower(c));
    });
    // .mdx and .mdl both mean the .m2 that shipped; this knew only the first.
    return pipeline::modelPathToM2(path);
}

std::string basenameOf(const std::string& path) {
    const size_t slash = path.find_last_of("\\/");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

} // namespace

FootprintRenderer::~FootprintRenderer() {
    shutdown();
}

bool FootprintRenderer::initialize(Renderer* owner, VkContext* ctx,
                                   VkDescriptorSetLayout perFrameLayout,
                                   pipeline::AssetManager* assetManager) {
    if (!owner || !ctx || !assetManager) {
        LOG_WARNING("FootprintRenderer: missing renderer, Vulkan context, or asset manager");
        return false;
    }
    if (vkCtx_) return true;

    owner_ = owner;
    vkCtx_ = ctx;
    perFrameLayout_ = perFrameLayout;
    if (!createDescriptorResources() || !createPipeline() || !createQuad() ||
        !loadFootprintData(assetManager)) {
        LOG_WARNING("FootprintRenderer: initialization incomplete");
        shutdown();
        return false;
    }
    LOG_INFO("FootprintRenderer: loaded ", profilesByPath_.size(),
             " model profiles and ", kTextureCount, " original footprint masks");
    return true;
}

void FootprintRenderer::shutdown() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator allocator = vkCtx_->getAllocator();
    vkDeviceWaitIdle(device);

    for (auto& texture : textures_) texture.destroy(device, allocator);
    // Through the helpers, which clear each handle as they destroy it, so the
    // block of assignments that used to follow is gone.
    destroy(allocator, quadVB_, quadVBAlloc_);
    destroy(device, pipeline_);
    destroy(device, pipelineLayout_);
    destroy(device, descriptorPool_);
    destroy(device, materialSetLayout_);

    textureSets_.fill(VK_NULL_HANDLE);
    profilesByPath_.clear();
    profilesByBasename_.clear();
    prints_.clear();
    owner_ = nullptr;
    vkCtx_ = nullptr;
    perFrameLayout_ = VK_NULL_HANDLE;
}

bool FootprintRenderer::createDescriptorResources() {
    VkDevice device = vkCtx_->getDevice();
    VkDescriptorSetLayoutBinding sampler{};
    sampler.binding = 0;
    sampler.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sampler.descriptorCount = 1;
    sampler.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    materialSetLayout_ = createDescriptorSetLayout(device, {sampler});
    if (!materialSetLayout_) return false;

    VkDescriptorPoolSize size{};
    size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    size.descriptorCount = kTextureCount;
    VkDescriptorPoolCreateInfo pool{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    pool.maxSets = kTextureCount;
    pool.poolSizeCount = 1;
    pool.pPoolSizes = &size;
    if (vkCreateDescriptorPool(device, &pool, nullptr, &descriptorPool_) != VK_SUCCESS) return false;

    std::array<VkDescriptorSetLayout, kTextureCount> layouts{};
    layouts.fill(materialSetLayout_);
    VkDescriptorSetAllocateInfo alloc{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    alloc.descriptorPool = descriptorPool_;
    alloc.descriptorSetCount = kTextureCount;
    alloc.pSetLayouts = layouts.data();
    return vkAllocateDescriptorSets(device, &alloc, textureSets_.data()) == VK_SUCCESS;
}

bool FootprintRenderer::createPipeline() {
    VkDevice device = vkCtx_->getDevice();
    VkShaderModule vert;
    VkShaderModule frag;
    if (!vert.loadFromFile(device, "assets/shaders/footprint.vert.spv") ||
        !frag.loadFromFile(device, "assets/shaders/footprint.frag.spv")) {
        vert.destroy();
        frag.destroy();
        return false;
    }

    VkPushConstantRange push{};
    push.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push.size = sizeof(FootprintPushConstants);
    if (!pipelineLayout_) {
        pipelineLayout_ = createPipelineLayout(device, {perFrameLayout_, materialSetLayout_}, {push});
    }

    VkVertexInputBindingDescription binding{.binding = 0, .stride = 5 * sizeof(float), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription pos{.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0};
    VkVertexInputAttributeDescription uv{.location = 1, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = 3 * sizeof(float)};
    pipeline_ = PipelineBuilder()
        .setShaders(vert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT), frag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({binding}, {pos, uv})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(pipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());
    vert.destroy();
    frag.destroy();
    return pipeline_ != VK_NULL_HANDLE;
}

void FootprintRenderer::recreatePipelines() {
    if (!vkCtx_) return;
    if (pipeline_) {
        vkDestroyPipeline(vkCtx_->getDevice(), pipeline_, nullptr);
        pipeline_ = VK_NULL_HANDLE;
    }
    if (!createPipeline()) LOG_WARNING("FootprintRenderer: pipeline recreation failed");
}

bool FootprintRenderer::createQuad() {
    const float vertices[] = {
        -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
         0.5f, -0.5f, 0.0f, 1.0f, 1.0f,
         0.5f,  0.5f, 0.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
         0.5f,  0.5f, 0.0f, 1.0f, 0.0f,
        -0.5f,  0.5f, 0.0f, 0.0f, 0.0f
    };
    AllocatedBuffer buffer = uploadBuffer(*vkCtx_, vertices, sizeof(vertices),
                                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    quadVB_ = buffer.buffer;
    quadVBAlloc_ = buffer.allocation;
    return quadVB_ != VK_NULL_HANDLE;
}

bool FootprintRenderer::loadFootprintData(pipeline::AssetManager* assets) {
    // FootprintTextures IDs are sparse. These six rows are the original masks.
    const std::array<uint32_t, kTextureCount> textureIds{1, 4, 5, 3, 6, 7};
    std::unordered_map<uint32_t, uint8_t> indexById;
    auto textureDbc = assets->loadDBC("FootprintTextures.dbc");
    if (!textureDbc || !textureDbc->isLoaded()) return false;

    VkDevice device = vkCtx_->getDevice();
    for (size_t i = 0; i < kTextureCount; ++i) {
        const int32_t row = textureDbc->findRecordById(textureIds[i]);
        if (row < 0) return false;
        std::string path = textureDbc->getString(static_cast<uint32_t>(row), 1);
        if (path.find('.') == std::string::npos) path += ".blp";
        pipeline::BLPImage image = assets->loadTexture(path);
        if (!image.isValid() || !textures_[i].upload(*vkCtx_, image.data.data(), image.width, image.height,
                                                     VK_FORMAT_R8G8B8A8_UNORM, true)) {
            LOG_WARNING("FootprintRenderer: failed to load ", path);
            return false;
        }
        textures_[i].createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                                   VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
        VkDescriptorImageInfo imageInfo = textures_[i].descriptorInfo();
        VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = textureSets_[i];
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imageInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        indexById[textureIds[i]] = static_cast<uint8_t>(i);
    }

    auto modelDbc = assets->loadDBC("CreatureModelData.dbc");
    if (!modelDbc || !modelDbc->isLoaded() || modelDbc->getFieldCount() <= 8) return false;
    for (uint32_t row = 0; row < modelDbc->getRecordCount(); ++row) {
        auto textureIt = indexById.find(modelDbc->getUInt32(row, 6));
        if (textureIt == indexById.end()) continue;
        std::string path = normalizeModelPath(modelDbc->getString(row, 2));
        // FootprintTextureLength/Width are inches; world units are yards, so the
        // conversion is /36.  Dividing by 12 yields feet and renders every print
        // three times its real size - a human's came out a yard long.
        const float length = modelDbc->getFloat(row, 7) / 36.0f;
        const float width = modelDbc->getFloat(row, 8) / 36.0f;
        // Reject degenerate rows on the raw inches, so the /36 correction doesn't
        // also quietly raise the cutoff and drop small creatures that used to
        // resolve a profile.
        if (path.empty() || length * 36.0f <= 0.6f || width * 36.0f <= 0.6f) continue;
        Profile profile{textureIt->second, length, width};
        profilesByPath_[path] = profile;
    }
    std::unordered_map<std::string, unsigned> basenameCounts;
    for (const auto& [path, profile] : profilesByPath_) {
        (void)profile;
        ++basenameCounts[basenameOf(path)];
    }
    for (const auto& [path, profile] : profilesByPath_) {
        std::string base = basenameOf(path);
        if (basenameCounts[base] == 1) profilesByBasename_[base] = profile;
    }
    return !profilesByPath_.empty();
}

FootprintRenderer::Profile FootprintRenderer::resolveProfile(
    const std::string& modelName, FootprintFallback fallback) const {
    const std::string normalized = normalizeModelPath(modelName);
    auto exact = profilesByPath_.find(normalized);
    if (exact != profilesByPath_.end()) return exact->second;
    auto base = profilesByBasename_.find(basenameOf(normalized));
    if (base != profilesByBasename_.end()) return base->second;

    // Yards, to match the converted DBC dimensions above.
    switch (fallback) {
        case FootprintFallback::HOOF:   return {.textureIndex = 4, .length = 0.50f, .width = 0.33f};
        case FootprintFallback::PAW:    return {.textureIndex = 5, .length = 0.67f, .width = 0.50f};
        case FootprintFallback::CLAW:   return {.textureIndex = 2, .length = 0.67f, .width = 0.50f};
        case FootprintFallback::CLOVEN: return {.textureIndex = 3, .length = 0.60f, .width = 0.43f};
        case FootprintFallback::BIPED:  return {.textureIndex = 0, .length = 0.33f, .width = 0.28f};
    }
    return {};
}

float FootprintRenderer::resolveFloorHeight(const glm::vec3& position) const {
    std::optional<float> floor;
    auto consider = [&](std::optional<float> height) {
        if (!height || *height < position.z - 1.25f || *height > position.z + 0.85f) return;
        floor = floor ? std::max(*floor, *height) : *height;
    };
    if (auto* terrain = owner_->getTerrainManager())
        consider(terrain->getHeightAt(position.x, position.y));
    if (auto* wmo = owner_->getWMORenderer())
        consider(wmo->getFloorHeight(position.x, position.y, position.z + 3.0f));
    if (auto* m2 = owner_->getM2Renderer())
        consider(m2->getFloorHeight(position.x, position.y, position.z + 2.0f));
    return floor.value_or(position.z);
}

void FootprintRenderer::spawn(const std::string& modelName, const glm::vec3& basePosition,
                              float yawRadians, bool leftFoot, FootprintFallback fallback) {
    if (!vkCtx_) return;
    const Profile profile = resolveProfile(modelName, fallback);
    const glm::vec2 forward(std::cos(yawRadians), std::sin(yawRadians));
    const glm::vec2 right(-forward.y, forward.x);
    // Stance is how far each foot sits off the centreline - a body dimension,
    // roughly three quarters of a footprint's width to either side.  The
    // coefficient carries the /12-to-/36 correction so the gait stays as wide
    // as it was while the prints themselves shrink to life size.
    const float side = (leftFoot ? -1.0f : 1.0f) * profile.width * 0.72f;
    const glm::vec2 xy = glm::vec2(basePosition) + right * side - forward * profile.length * 0.16f;
    glm::vec3 position(xy.x, xy.y, basePosition.z);
    position.z = resolveFloorHeight(position) + 0.035f;

    if (prints_.size() >= kMaxPrints) prints_.erase(prints_.begin());
    // A negative width mirrors the one-foot mask laterally for the opposite foot.
    prints_.push_back({position, yawRadians, profile.length,
                       leftFoot ? -profile.width : profile.width, 0.0f,
                       profile.textureIndex});
}

void FootprintRenderer::update(float deltaTime) {
    for (Print& print : prints_) print.age += deltaTime;
    prints_.erase(std::remove_if(prints_.begin(), prints_.end(),
        [](const Print& print) { return print.age >= kLifetime; }), prints_.end());
}

void FootprintRenderer::clear() {
    prints_.clear();
}

void FootprintRenderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (prints_.empty() || !pipeline_ || !quadVB_) return;
    const glm::vec3 cameraPos = camera.getPosition();
    Frustum frustum;
    frustum.extractFromMatrix(camera.getViewProjectionMatrix());

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                            0, 1, &perFrameSet, 0, nullptr);
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &quadVB_, &offset);

    for (const Print& print : prints_) {
        const glm::vec3 delta = cameraPos - print.position;
        if (glm::dot(delta, delta) > 10000.0f ||
            !frustum.intersectsSphere(print.position, std::max(print.length, std::abs(print.signedWidth))))
            continue;

        float alpha = 1.0f;
        if (print.age > kFadeStart) {
            float t = glm::clamp((print.age - kFadeStart) / (kLifetime - kFadeStart), 0.0f, 1.0f);
            t = t * t * (3.0f - 2.0f * t);
            alpha = 1.0f - t;
        }
        if (alpha <= 0.001f || print.textureIndex >= kTextureCount) continue;

        glm::mat4 model = glm::translate(glm::mat4(1.0f), print.position);
        // The shipped masks point toward +V/local +Y; rotate that axis onto
        // the model's +X forward direction and mirror across local X.
        model = glm::rotate(model, print.yaw - 1.57079632679f, glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::scale(model, glm::vec3(print.signedWidth, print.length, 1.0f));
        FootprintPushConstants push;
        push.model = model;
        push.tint.a = alpha;
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                                1, 1, &textureSets_[print.textureIndex], 0, nullptr);
        vkCmdPushConstants(cmd, pipelineLayout_,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(push), &push);
        vkCmdDraw(cmd, 6, 1, 0, 0);
    }
}

} // namespace wowee::rendering
