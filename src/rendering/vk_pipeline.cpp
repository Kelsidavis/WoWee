#include "rendering/vk_pipeline.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {

PipelineBuilder::PipelineBuilder() {
    // Default: one blend attachment with blending disabled
    colorBlendAttachments_.push_back(blendDisabled());

    // Default dynamic states: viewport + scissor (almost always dynamic)
    dynamicStates_ = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
}

PipelineBuilder& PipelineBuilder::setShaders(
    VkPipelineShaderStageCreateInfo vert, VkPipelineShaderStageCreateInfo frag)
{
    shaderStages_ = {vert, frag};
    return *this;
}

PipelineBuilder& PipelineBuilder::setVertexInput(
    const std::vector<VkVertexInputBindingDescription>& bindings,
    const std::vector<VkVertexInputAttributeDescription>& attributes)
{
    vertexBindings_ = bindings;
    vertexAttributes_ = attributes;
    return *this;
}
PipelineBuilder& PipelineBuilder::setTopology(VkPrimitiveTopology topology,
    VkBool32 primitiveRestart)
{
    topology_ = topology;
    primitiveRestart_ = primitiveRestart;
    return *this;
}

PipelineBuilder& PipelineBuilder::setRasterization(VkPolygonMode polygonMode,
    VkCullModeFlags cullMode, VkFrontFace frontFace)
{
    polygonMode_ = polygonMode;
    cullMode_ = cullMode;
    frontFace_ = frontFace;
    return *this;
}

PipelineBuilder& PipelineBuilder::setDepthTest(bool enable, bool writeEnable,
    VkCompareOp compareOp)
{
    depthTestEnable_ = enable;
    depthWriteEnable_ = writeEnable;
    depthCompareOp_ = compareOp;
    return *this;
}

PipelineBuilder& PipelineBuilder::setNoDepthTest() {
    depthTestEnable_ = false;
    depthWriteEnable_ = false;
    return *this;
}

PipelineBuilder& PipelineBuilder::setDepthBias(float constantFactor, float slopeFactor) {
    depthBiasEnable_ = true;
    depthBiasConstant_ = constantFactor;
    depthBiasSlope_ = slopeFactor;
    return *this;
}

PipelineBuilder& PipelineBuilder::setColorBlendAttachment(
    VkPipelineColorBlendAttachmentState blendState)
{
    // assign, not an initializer_list. GCC 13 at -O2 traces the list's
    // temporary array through the vector's memmove and reports a false
    // -Warray-bounds against it, which -Werror turns into a build failure.
    colorBlendAttachments_.assign(1, blendState);
    return *this;
}

PipelineBuilder& PipelineBuilder::setNoColorAttachment() {
    colorBlendAttachments_.clear();
    return *this;
}

PipelineBuilder& PipelineBuilder::setMultisample(VkSampleCountFlagBits samples) {
    msaaSamples_ = samples;
    return *this;
}

PipelineBuilder& PipelineBuilder::setAlphaToCoverage(bool enable) {
    alphaToCoverage_ = enable;
    return *this;
}

PipelineBuilder& PipelineBuilder::setLayout(VkPipelineLayout layout) {
    pipelineLayout_ = layout;
    return *this;
}

PipelineBuilder& PipelineBuilder::setRenderPass(VkRenderPass renderPass, uint32_t subpass) {
    renderPass_ = renderPass;
    subpass_ = subpass;
    // The other half of the exclusion described on setRenderingFormats.
    useDynamicRendering_ = false;
    colorFormats_.clear();
    depthFormat_ = VK_FORMAT_UNDEFINED;
    stencilFormat_ = VK_FORMAT_UNDEFINED;
    return *this;
}

PipelineBuilder& PipelineBuilder::setRenderingFormats(const std::vector<VkFormat>& colorFormats,
                                                      VkFormat depthFormat,
                                                      VkFormat stencilFormat) {
    colorFormats_ = colorFormats;
    depthFormat_ = depthFormat;
    stencilFormat_ = stencilFormat;
    useDynamicRendering_ = true;
    renderPass_ = VK_NULL_HANDLE;
    subpass_ = 0;
    return *this;
}

PipelineBuilder& PipelineBuilder::setDynamicStates(const std::vector<VkDynamicState>& states) {
    dynamicStates_ = states;
    return *this;
}

// Pipeline derivatives - hint driver to share compiled state between similar pipelines
PipelineBuilder& PipelineBuilder::setFlags(VkPipelineCreateFlags flags) {
    flags_ = flags;
    return *this;
}

PipelineBuilder& PipelineBuilder::setBasePipeline(VkPipeline basePipeline) {
    basePipelineHandle_ = basePipeline;
    return *this;
}

VkPipeline PipelineBuilder::build(VkDevice device, VkPipelineCache cache) const {
    // Vertex input
    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = static_cast<uint32_t>(vertexBindings_.size());
    vertexInput.pVertexBindingDescriptions = vertexBindings_.data();
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertexAttributes_.size());
    vertexInput.pVertexAttributeDescriptions = vertexAttributes_.data();

    // Input assembly
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = topology_;
    inputAssembly.primitiveRestartEnable = primitiveRestart_;

    // Viewport / scissor (dynamic, so just specify count)
    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    // Rasterization
    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = polygonMode_;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = cullMode_;
    rasterizer.frontFace = frontFace_;
    rasterizer.depthBiasEnable = depthBiasEnable_ ? VK_TRUE : VK_FALSE;
    rasterizer.depthBiasConstantFactor = depthBiasConstant_;
    rasterizer.depthBiasSlopeFactor = depthBiasSlope_;

    // Multisampling
    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = msaaSamples_;
    multisampling.alphaToCoverageEnable = alphaToCoverage_ ? VK_TRUE : VK_FALSE;

    // Depth/stencil
    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = depthTestEnable_ ? VK_TRUE : VK_FALSE;
    depthStencil.depthWriteEnable = depthWriteEnable_ ? VK_TRUE : VK_FALSE;
    depthStencil.depthCompareOp = depthCompareOp_;
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    // Color blending
    VkPipelineColorBlendStateCreateInfo colorBlending{};
    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.attachmentCount = static_cast<uint32_t>(colorBlendAttachments_.size());
    colorBlending.pAttachments = colorBlendAttachments_.data();

    // Dynamic state
    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates_.size());
    dynamicState.pDynamicStates = dynamicStates_.data();

    // Create pipeline
    VkGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineInfo.stageCount = static_cast<uint32_t>(shaderStages_.size());
    pipelineInfo.pStages = shaderStages_.data();
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = colorBlendAttachments_.empty() ? nullptr : &colorBlending;
    pipelineInfo.pDynamicState = dynamicStates_.empty() ? nullptr : &dynamicState;
    pipelineInfo.layout = pipelineLayout_;
    pipelineInfo.flags = flags_;
    pipelineInfo.basePipelineHandle = basePipelineHandle_;
    pipelineInfo.basePipelineIndex = -1;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = subpass_;

    // A dynamic rendering pipeline carries its attachment formats instead of a
    // render pass, and leaves renderPass VK_NULL_HANDLE. Declared out here so
    // it outlives the create call, which reads the chain.
    VkPipelineRenderingCreateInfo renderingInfo{};
    if (useDynamicRendering_) {
        renderingInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        renderingInfo.colorAttachmentCount = static_cast<uint32_t>(colorFormats_.size());
        renderingInfo.pColorAttachmentFormats =
            colorFormats_.empty() ? nullptr : colorFormats_.data();
        renderingInfo.depthAttachmentFormat = depthFormat_;
        renderingInfo.stencilAttachmentFormat = stencilFormat_;
        renderingInfo.pNext = pipelineInfo.pNext;
        pipelineInfo.pNext = &renderingInfo;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    if (vkCreateGraphicsPipelines(device, cache, 1, &pipelineInfo,
            nullptr, &pipeline) != VK_SUCCESS)
    {
        LOG_ERROR("Failed to create graphics pipeline");
        return VK_NULL_HANDLE;
    }

    return pipeline;
}

// All RGBA channels enabled - used by every blend mode since we never need to
// mask individual channels (WoW's fixed-function pipeline always writes all four).
static constexpr VkColorComponentFlags kColorWriteAll =
    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

VkPipelineColorBlendAttachmentState PipelineBuilder::blendDisabled() {
    VkPipelineColorBlendAttachmentState state{};
    state.colorWriteMask = kColorWriteAll;
    state.blendEnable = VK_FALSE;
    return state;
}

VkPipelineColorBlendAttachmentState PipelineBuilder::blendAlpha() {
    VkPipelineColorBlendAttachmentState state{};
    state.colorWriteMask = kColorWriteAll;
    state.blendEnable = VK_TRUE;
    state.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    state.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    state.colorBlendOp = VK_BLEND_OP_ADD;
    state.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    state.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    state.alphaBlendOp = VK_BLEND_OP_ADD;
    return state;
}
VkPipelineColorBlendAttachmentState PipelineBuilder::blendAdditive() {
    VkPipelineColorBlendAttachmentState state{};
    state.colorWriteMask = kColorWriteAll;
    state.blendEnable = VK_TRUE;
    state.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    state.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    state.colorBlendOp = VK_BLEND_OP_ADD;
    state.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    state.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    state.alphaBlendOp = VK_BLEND_OP_ADD;
    return state;
}

VkPipelineLayout createPipelineLayout(VkDevice device,
    const std::vector<VkDescriptorSetLayout>& setLayouts,
    const std::vector<VkPushConstantRange>& pushConstants)
{
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
    layoutInfo.pSetLayouts = setLayouts.data();
    layoutInfo.pushConstantRangeCount = static_cast<uint32_t>(pushConstants.size());
    layoutInfo.pPushConstantRanges = pushConstants.data();

    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &layout) != VK_SUCCESS) {
        LOG_ERROR("Failed to create pipeline layout");
    }

    return layout;
}

VkDescriptorSetLayout createDescriptorSetLayout(VkDevice device,
    const std::vector<VkDescriptorSetLayoutBinding>& bindings)
{
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    VkDescriptorSetLayout layout = VK_NULL_HANDLE;
    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &layout) != VK_SUCCESS) {
        LOG_ERROR("Failed to create descriptor set layout");
    }

    return layout;
}

} // namespace rendering
} // namespace wowee
