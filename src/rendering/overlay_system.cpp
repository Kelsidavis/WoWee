#include "rendering/overlay_system.hpp"
#include "core/coordinates.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <vector>

namespace wowee {
namespace rendering {

OverlaySystem::OverlaySystem(VkContext* ctx)
    : vkCtx_(ctx) {}

OverlaySystem::~OverlaySystem() {
    cleanup();
}

void OverlaySystem::cleanup() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    destroy(device, selCirclePipeline_);
    destroy(device, selCirclePipelineLayout_);
    destroy(vkCtx_->getAllocator(), selCircleVertBuf_, selCircleVertAlloc_);
    destroy(vkCtx_->getAllocator(), selCircleIdxBuf_, selCircleIdxAlloc_);
    destroy(device, overlayPipeline_);
    destroy(device, brightnessPipeline_);
    destroy(device, overlayPipelineLayout_);
}

void OverlaySystem::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    // Destroy only pipelines (keep geometry buffers)
    destroy(device, selCirclePipeline_);
    destroy(device, overlayPipeline_);
    destroy(device, brightnessPipeline_);
}

void OverlaySystem::setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color) {
    selCirclePos_ = pos;
    selCircleRadius_ = radius;
    selCircleColor_ = color;
    selCircleVisible_ = true;
}

void OverlaySystem::clearSelectionCircle() {
    selCircleVisible_ = false;
}

void OverlaySystem::initSelectionCircle() {
    if (selCirclePipeline_ != VK_NULL_HANDLE) return;
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Load shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/selection_circle.vert.spv")) {
        LOG_ERROR("OverlaySystem: failed to load selection circle vertex shader");
        return;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/selection_circle.frag.spv")) {
        LOG_ERROR("OverlaySystem: failed to load selection circle fragment shader");
        vertShader.destroy();
        return;
    }

    // Pipeline layout: push constants only (mat4 mvp=64 + vec4 color=16), VERTEX|FRAGMENT
    VkPushConstantRange pcRange{};
    pcRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pcRange.offset = 0;
    pcRange.size = 80;
    selCirclePipelineLayout_ = createPipelineLayout(device, {}, {pcRange});

    // Vertex input: binding 0, stride 12, vec3 at location 0
    VkVertexInputBindingDescription vertBind{.binding = 0, .stride = 12, .inputRate = VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription vertAttr{.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0};

    // Build disc geometry as TRIANGLE_LIST (N=48 segments)
    constexpr int SEGMENTS = 48;
    std::vector<float> verts;
    verts.reserve((SEGMENTS + 1) * 3);
    // Center vertex
    verts.insert(verts.end(), {0.0f, 0.0f, 0.0f});
    // Ring vertices
    for (int i = 0; i <= SEGMENTS; ++i) {
        float angle = core::coords::TWO_PI * static_cast<float>(i) / static_cast<float>(SEGMENTS);
        verts.push_back(std::cos(angle));
        verts.push_back(std::sin(angle));
        verts.push_back(0.0f);
    }

    // Build TRIANGLE_LIST indices
    std::vector<uint16_t> indices;
    indices.reserve(SEGMENTS * 3);
    for (int i = 0; i < SEGMENTS; ++i) {
        indices.push_back(0);
        indices.push_back(static_cast<uint16_t>(i + 1));
        indices.push_back(static_cast<uint16_t>(i + 2));
    }
    selCircleVertCount_ = SEGMENTS * 3;

    // Upload vertex buffer
    if (selCircleVertBuf_ == VK_NULL_HANDLE) {
        AllocatedBuffer vbuf = uploadBuffer(*vkCtx_, verts.data(),
            verts.size() * sizeof(float), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        selCircleVertBuf_ = vbuf.buffer;
        selCircleVertAlloc_ = vbuf.allocation;

        AllocatedBuffer ibuf = uploadBuffer(*vkCtx_, indices.data(),
            indices.size() * sizeof(uint16_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        selCircleIdxBuf_ = ibuf.buffer;
        selCircleIdxAlloc_ = ibuf.allocation;
    }

    // Build pipeline
    selCirclePipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({vertBind}, {vertAttr})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(selCirclePipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());

    vertShader.destroy();
    fragShader.destroy();

    if (!selCirclePipeline_) {
        LOG_ERROR("OverlaySystem: failed to build selection circle pipeline");
    }
}

void OverlaySystem::renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection,
                                           VkCommandBuffer cmd,
                                           HeightQuery2D terrainHeight,
                                           HeightQuery3D wmoHeight,
                                           HeightQuery3D m2Height) {
    if (!selCircleVisible_) return;
    initSelectionCircle();
    if (selCirclePipeline_ == VK_NULL_HANDLE || cmd == VK_NULL_HANDLE) return;

    // Keep circle anchored near target foot Z. The floor queries are collision
    // raycasts; reuse the last result while the target stands still, refreshing
    // every 30 frames in case geometry streams in around it.
    float floorZ;
    if (selCircleFloorCacheAge_ >= 0 && selCircleFloorCacheAge_ < 30 &&
        selCirclePos_ == selCircleFloorCachePos_) {
        floorZ = selCircleFloorCacheZ_;
        selCircleFloorCacheAge_++;
    } else {
        const float baseZ = selCirclePos_.z;
        floorZ = baseZ;
        auto considerFloor = [&](std::optional<float> sample) {
            if (!sample) return;
            const float h = *sample;
            if (h < baseZ - 1.25f || h > baseZ + 0.85f) return;
            floorZ = std::max(floorZ, h);
        };

        if (terrainHeight) considerFloor(terrainHeight(selCirclePos_.x, selCirclePos_.y));
        if (wmoHeight) considerFloor(wmoHeight(selCirclePos_.x, selCirclePos_.y, selCirclePos_.z + 3.0f));
        if (m2Height) considerFloor(m2Height(selCirclePos_.x, selCirclePos_.y, selCirclePos_.z + 2.0f));

        selCircleFloorCachePos_ = selCirclePos_;
        selCircleFloorCacheZ_ = floorZ;
        selCircleFloorCacheAge_ = 0;
    }

    glm::vec3 raisedPos = selCirclePos_;
    raisedPos.z = floorZ + 0.17f;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), raisedPos);
    model = glm::scale(model, glm::vec3(selCircleRadius_));

    glm::mat4 mvp = projection * view * model;
    glm::vec4 color4(selCircleColor_, 1.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, selCirclePipeline_);
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &selCircleVertBuf_, &offset);
    vkCmdBindIndexBuffer(cmd, selCircleIdxBuf_, 0, VK_INDEX_TYPE_UINT16);
    vkCmdPushConstants(cmd, selCirclePipelineLayout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, 64, &mvp[0][0]);
    vkCmdPushConstants(cmd, selCirclePipelineLayout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        64, 16, &color4[0]);
    vkCmdDrawIndexed(cmd, static_cast<uint32_t>(selCircleVertCount_), 1, 0, 0, 0);
}

void OverlaySystem::initOverlayPipeline() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pc.offset = 0;
    pc.size = 112;  // mat4 invViewProj + vec4 colour + vec4 plane + vec4 params

    VkPipelineLayoutCreateInfo plCI{};
    plCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plCI.pushConstantRangeCount = 1;
    plCI.pPushConstantRanges = &pc;
    vkCreatePipelineLayout(device, &plCI, nullptr, &overlayPipelineLayout_);

    VkShaderModule vertMod, fragMod;
    if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
        !fragMod.loadFromFile(device, "assets/shaders/overlay.frag.spv")) {
        LOG_ERROR("OverlaySystem: failed to load overlay shaders");
        vertMod.destroy(); fragMod.destroy();
        return;
    }

    overlayPipeline_ = PipelineBuilder()
        .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({}, {})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(overlayPipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());

    vertMod.destroy(); fragMod.destroy();

    if (overlayPipeline_) LOG_INFO("OverlaySystem: overlay pipeline initialized");
}

namespace {

/// overlay.frag's push block. One definition, because there are two entry
/// points into that shader and only one of them was keeping up with it.
struct OverlayPush {
    glm::mat4 invViewProj;
    glm::vec4 color;
    glm::vec4 plane;
    glm::vec4 params;
};
static_assert(sizeof(OverlayPush) == 112, "must match overlay.frag's push block");

OverlayPush makeOverlayPush(const glm::vec4& color, const glm::mat4& invViewProj,
                            float waterZ, float softness, float rippleAmp,
                            float time, bool hasSeam) {
    OverlayPush p{};
    p.invViewProj = invViewProj;
    p.color = color;
    p.plane = glm::vec4(0.0f, 0.0f, 0.0f, waterZ);
    p.params = glm::vec4(softness, time, rippleAmp, hasSeam ? 1.0f : 0.0f);
    return p;
}

}  // namespace

void OverlaySystem::renderOverlay(const glm::vec4& color, VkCommandBuffer cmd) {
    // Whole screen: no seam, so the shader skips the surface test entirely and
    // the matrix and height below are never read.
    renderWaterline(color, glm::mat4(1.0f), 0.0f, 0.0f, 0.0f, 0.0f, false, cmd);
}

void OverlaySystem::renderWaterline(const glm::vec4& color, const glm::mat4& invViewProj,
                                    float waterZ, float softness, float rippleAmp,
                                    float time, bool hasSeam, VkCommandBuffer cmd) {
    if (!overlayPipeline_) initOverlayPipeline();
    if (!overlayPipeline_ || cmd == VK_NULL_HANDLE) return;
    OverlayPush push = makeOverlayPush(color, invViewProj, waterZ, softness, rippleAmp,
                                       time, hasSeam);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, overlayPipeline_);
    vkCmdPushConstants(cmd, overlayPipelineLayout_,
                       VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void OverlaySystem::initBrightnessPipeline() {
    if (brightnessPipeline_ != VK_NULL_HANDLE) return;
    if (!vkCtx_) return;
    // Reuse the overlay pipeline's layout (same fragment push constant); only
    // the blend state differs.
    if (!overlayPipeline_) initOverlayPipeline();
    if (!overlayPipelineLayout_) return;
    VkDevice device = vkCtx_->getDevice();

    VkShaderModule vertMod, fragMod;
    if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
        !fragMod.loadFromFile(device, "assets/shaders/overlay.frag.spv")) {
        LOG_ERROR("OverlaySystem: failed to load brightness shaders");
        vertMod.destroy(); fragMod.destroy();
        return;
    }

    // result.rgb = src.rgb * dst.rgb + dst.rgb * 1 = dst.rgb * (1 + src.rgb).
    // Pushing src.rgb = (scale - 1) yields dst.rgb * scale - a true luminance
    // multiply. Alpha is left untouched (dst passes through).
    VkPipelineColorBlendAttachmentState mul{};
    mul.blendEnable = VK_TRUE;
    mul.srcColorBlendFactor = VK_BLEND_FACTOR_DST_COLOR;
    mul.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    mul.colorBlendOp = VK_BLEND_OP_ADD;
    mul.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    mul.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    mul.alphaBlendOp = VK_BLEND_OP_ADD;
    mul.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    brightnessPipeline_ = PipelineBuilder()
        .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({}, {})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(mul)
        .setMultisample(vkCtx_->getMsaaSamples())
        .setLayout(overlayPipelineLayout_)
        .setRenderPass(vkCtx_->getImGuiRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());

    vertMod.destroy(); fragMod.destroy();
    if (brightnessPipeline_) LOG_INFO("OverlaySystem: brightness pipeline initialized");
}

void OverlaySystem::renderBrightnessScale(float scale, VkCommandBuffer cmd) {
    if (scale <= 1.0f) return; // darkening handled by the black overlay path
    if (!brightnessPipeline_) initBrightnessPipeline();
    if (!brightnessPipeline_ || cmd == VK_NULL_HANDLE) return;
    // overlay.frag outputs the pushed color; the blend multiplies it by dst.
    //
    // The whole block, not just the colour. This pushed sixteen bytes back when
    // the block started with one, and kept doing it after a mat4 was put in
    // front: the colour landed in the matrix's first row and everything the
    // shader actually reads was whatever the last draw had left there. It runs
    // whenever brightness is above neutral, so the screen flashed green or red
    // by the frame.
    const glm::vec4 tint(scale - 1.0f, scale - 1.0f, scale - 1.0f, 1.0f);
    OverlayPush push = makeOverlayPush(tint, glm::mat4(1.0f), 0.0f, 0.0f, 0.0f, 0.0f, false);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, brightnessPipeline_);
    vkCmdPushConstants(cmd, overlayPipelineLayout_,
                       VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

} // namespace rendering
} // namespace wowee
