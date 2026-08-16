#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>

#include "rendering/vertex_layout.hpp"

namespace wowee {
namespace rendering {

// Builder pattern for VkGraphicsPipeline creation.
// Usage:
//   auto pipeline = PipelineBuilder()
//       .setShaders(vertStage, fragStage)
//       .setVertexInput(bindings, attributes)
//       .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
//       .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_BACK_BIT)
//       .setDepthTest(true, true, VK_COMPARE_OP_LESS)
//       .setColorBlendAttachment(PipelineBuilder::blendAlpha())
//       .setLayout(pipelineLayout)
//       .setRenderPass(renderPass)
//       .build(device);

class PipelineBuilder {
public:
    PipelineBuilder();

    // Shader stages
    PipelineBuilder& setShaders(VkPipelineShaderStageCreateInfo vert,
        VkPipelineShaderStageCreateInfo frag);

    // Vertex input
    PipelineBuilder& setVertexInput(
        const std::vector<VkVertexInputBindingDescription>& bindings,
        const std::vector<VkVertexInputAttributeDescription>& attributes);


    // Input assembly
    PipelineBuilder& setTopology(VkPrimitiveTopology topology,
        VkBool32 primitiveRestart = VK_FALSE);

    // Rasterization
    PipelineBuilder& setRasterization(VkPolygonMode polygonMode,
        VkCullModeFlags cullMode,
        VkFrontFace frontFace = VK_FRONT_FACE_CLOCKWISE);

    // Depth test/write
    PipelineBuilder& setDepthTest(bool enable, bool writeEnable,
        VkCompareOp compareOp = VK_COMPARE_OP_LESS);

    // No depth test (default)
    PipelineBuilder& setNoDepthTest();

    // Depth bias (for shadow maps)
    PipelineBuilder& setDepthBias(float constantFactor, float slopeFactor);

    // Color blend attachment
    PipelineBuilder& setColorBlendAttachment(
        VkPipelineColorBlendAttachmentState blendState);

    // No color attachment (depth-only pass)
    PipelineBuilder& setNoColorAttachment();

    // Multisampling
    PipelineBuilder& setMultisample(VkSampleCountFlagBits samples);
    PipelineBuilder& setAlphaToCoverage(bool enable);

    // Pipeline layout
    PipelineBuilder& setLayout(VkPipelineLayout layout);

    // Render pass
    PipelineBuilder& setRenderPass(VkRenderPass renderPass, uint32_t subpass = 0);

    // Dynamic state
    PipelineBuilder& setDynamicStates(const std::vector<VkDynamicState>& states);

    // Pipeline derivatives - hint driver to share compiled state between similar pipelines
    PipelineBuilder& setFlags(VkPipelineCreateFlags flags);
    PipelineBuilder& setBasePipeline(VkPipeline basePipeline);

    // Build the pipeline (pass a VkPipelineCache for faster creation)
    VkPipeline build(VkDevice device, VkPipelineCache cache = VK_NULL_HANDLE) const;

    // Common blend states
    static VkPipelineColorBlendAttachmentState blendDisabled();
    static VkPipelineColorBlendAttachmentState blendAlpha();
    static VkPipelineColorBlendAttachmentState blendAdditive();

private:
    std::vector<VkPipelineShaderStageCreateInfo> shaderStages_;
    std::vector<VkVertexInputBindingDescription> vertexBindings_;
    std::vector<VkVertexInputAttributeDescription> vertexAttributes_;
    VkPrimitiveTopology topology_ = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkBool32 primitiveRestart_ = VK_FALSE;
    VkPolygonMode polygonMode_ = VK_POLYGON_MODE_FILL;
    VkCullModeFlags cullMode_ = VK_CULL_MODE_NONE;
    VkFrontFace frontFace_ = VK_FRONT_FACE_CLOCKWISE;
    bool depthTestEnable_ = false;
    bool depthWriteEnable_ = false;
    VkCompareOp depthCompareOp_ = VK_COMPARE_OP_LESS;
    bool depthBiasEnable_ = false;
    float depthBiasConstant_ = 0.0f;
    float depthBiasSlope_ = 0.0f;
    VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    bool alphaToCoverage_ = false;
    std::vector<VkPipelineColorBlendAttachmentState> colorBlendAttachments_;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    uint32_t subpass_ = 0;
    std::vector<VkDynamicState> dynamicStates_;
    VkPipelineCreateFlags flags_ = 0;
    VkPipeline basePipelineHandle_ = VK_NULL_HANDLE;
};

// ---------------------------------------------------------------------------
// Vertex layouts that several effects share
//
// Two layouts account for every point-and-quad effect in the renderer, and both
// happen to be twenty bytes: a position and two per-vertex floats, or a position
// and a texture coordinate. Each was written out by hand at the twelve places
// that build such a pipeline - binding, stride, then one struct per attribute
// with its location, format and offset.
//
// The offsets have to agree with the shader that reads them, and nothing checks
// that they do. A stride or offset wrong in one of twelve copies is not a
// crash; it is a vertex stream read at the wrong places, which draws something
// that looks nearly right.

/// A binding that packs vertices back to back, with nothing between them.
inline VkVertexInputBindingDescription tightVertexBinding(uint32_t strideBytes,
                                                          uint32_t binding = 0) {
    VkVertexInputBindingDescription desc{};
    desc.binding = binding;
    desc.stride = strideBytes;
    desc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return desc;
}

/// vec3 position, then two single floats - size and alpha, or brightness and
/// twinkle phase, depending on what is being drawn. Stride 20.
inline std::vector<VkVertexInputAttributeDescription> positionPlusTwoFloatsAttrs() {
    return {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        {1, 0, VK_FORMAT_R32_SFLOAT, 3 * sizeof(float)},
        {2, 0, VK_FORMAT_R32_SFLOAT, 4 * sizeof(float)},
    };
}

/// vec3 position, then a vec2 texture coordinate. Stride 20.
inline std::vector<VkVertexInputAttributeDescription> positionPlusUvAttrs() {
    return {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        {1, 0, VK_FORMAT_R32G32_SFLOAT, 3 * sizeof(float)},
    };
}

/// Viewport and scissor set per command buffer rather than baked into the
/// pipeline - which is every pipeline here, because the window can resize.
inline std::vector<VkDynamicState> viewportAndScissorDynamic() {
    return {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
}

/// A vertex attribute description, from the layout stated without Vulkan.
///
/// The component count is the whole mapping: every vertex attribute in this
/// renderer is floats, so 2, 3 and 4 are the only widths there are.
inline VkVertexInputAttributeDescription toVkAttribute(const VertexAttribute& attribute,
                                                       uint32_t binding = 0) {
    VkFormat format = VK_FORMAT_UNDEFINED;
    switch (attribute.componentCount) {
        case 2: format = VK_FORMAT_R32G32_SFLOAT; break;
        case 3: format = VK_FORMAT_R32G32B32_SFLOAT; break;
        case 4: format = VK_FORMAT_R32G32B32A32_SFLOAT; break;
        default: break;
    }
    return VkVertexInputAttributeDescription{.location = attribute.location, .binding = binding, .format = format,
                                             .offset = attribute.offset};
}

/// The same for a whole layout.
template <typename Attributes>
std::vector<VkVertexInputAttributeDescription> toVkAttributes(const Attributes& attributes,
                                                              uint32_t binding = 0) {
    std::vector<VkVertexInputAttributeDescription> out;
    out.reserve(attributes.size());
    for (const VertexAttribute& attribute : attributes) {
        out.push_back(toVkAttribute(attribute, binding));
    }
    return out;
}

/// The binding a per-vertex layout of the given stride is read through.
inline VkVertexInputBindingDescription perVertexBinding(uint32_t stride,
                                                        uint32_t binding = 0) {
    VkVertexInputBindingDescription description{};
    description.binding = binding;
    description.stride = stride;
    description.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return description;
}

/// The depth-only pipeline every renderer draws its shadow pass with.
///
/// Four of them built it: characters, M2 doodads, WMOs and terrain. The state
/// was identical in all four down to the depth bias, and only the shaders, the
/// vertex format and the layout differ, which is what this takes.
///
/// Culling is off rather than front-face, because foliage and leaf cards are
/// effectively two-sided and front-face culling drops them out of the shadow
/// map depending on where the light is.
inline VkPipeline buildShadowPipeline(
        VkDevice device, VkPipelineCache cache,
        const VkPipelineShaderStageCreateInfo& vertStage,
        const VkPipelineShaderStageCreateInfo& fragStage,
        const VkVertexInputBindingDescription& binding,
        const std::vector<VkVertexInputAttributeDescription>& attributes,
        VkPipelineLayout layout, VkRenderPass renderPass) {
    return PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, attributes)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setDepthBias(0.05f, 0.20f)
        .setNoColorAttachment()
        .setLayout(layout)
        .setRenderPass(renderPass)
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, cache);
}


// Helper to create a pipeline layout from descriptor set layouts and push constant ranges
VkPipelineLayout createPipelineLayout(VkDevice device,
    const std::vector<VkDescriptorSetLayout>& setLayouts,
    const std::vector<VkPushConstantRange>& pushConstants = {});

// Helper to create a descriptor set layout from bindings
VkDescriptorSetLayout createDescriptorSetLayout(VkDevice device,
    const std::vector<VkDescriptorSetLayoutBinding>& bindings);

} // namespace rendering
} // namespace wowee
