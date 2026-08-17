#include "rendering/grass_renderer.hpp"

#include <array>
#include <cstring>
#include <vector>

#include "core/logger.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"

namespace wowee {
namespace rendering {

namespace {

/// Distance beyond which a blade is not drawn, squared. A single fixed bound
/// in this phase; Phase 7 makes it a density falloff.
constexpr float kMaxDistance = 120.0f;

/// Five segments, six rows of two vertices, so a blade can curve instead of
/// hinging. The top row's width tapers to nothing, which makes its quad a
/// triangle and gives the blade a point rather than a cut-off end.
constexpr uint32_t kBladeSegments = 5;
constexpr uint32_t kBladeVertices = (kBladeSegments + 1) * 2;
constexpr uint32_t kBladeIndexCount = kBladeSegments * 6;

// The highest index the list below generates is the last vertex of the top
// row. If the two ever disagree the draw reads past the rows the vertex shader
// builds, which the shader cannot detect - it just returns garbage positions.
static_assert(kBladeSegments * 2 + 1 == kBladeVertices - 1,
              "the index list must reference exactly the rows that exist");

std::array<uint16_t, kBladeIndexCount> bladeIndices() {
    std::array<uint16_t, kBladeIndexCount> indices{};
    for (uint32_t seg = 0; seg < kBladeSegments; ++seg) {
        const auto a = static_cast<uint16_t>(seg * 2);
        const auto b = static_cast<uint16_t>(seg * 2 + 1);
        const auto c = static_cast<uint16_t>(seg * 2 + 2);
        const auto d = static_cast<uint16_t>(seg * 2 + 3);
        const uint32_t o = seg * 6;
        indices[o + 0] = a; indices[o + 1] = b; indices[o + 2] = d;
        indices[o + 3] = a; indices[o + 4] = d; indices[o + 5] = c;
    }
    return indices;
}

bool createGpuBuffer(VmaAllocator allocator, VkDeviceSize size, VkBufferUsageFlags usage,
                     VkBuffer& outBuffer, VmaAllocation& outAlloc) {
    VkBufferCreateInfo bci{.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size;
    bci.usage = usage;
    VmaAllocationCreateInfo aci{};
    aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    return vmaCreateBuffer(allocator, &bci, &aci, &outBuffer, &outAlloc, nullptr) == VK_SUCCESS;
}

bool createMappedBuffer(VmaAllocator allocator, VkDeviceSize size, VkBufferUsageFlags usage,
                        VkBuffer& outBuffer, VmaAllocation& outAlloc, void*& outMapped) {
    VkBufferCreateInfo bci{.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.size = size;
    bci.usage = usage;
    VmaAllocationCreateInfo aci{};
    aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo ai{};
    if (vmaCreateBuffer(allocator, &bci, &aci, &outBuffer, &outAlloc, &ai) != VK_SUCCESS) {
        return false;
    }
    outMapped = ai.pMappedData;
    return true;
}

} // namespace

GrassRenderer::~GrassRenderer() {
    shutdown();
}

bool GrassRenderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    if (!ctx) return false;
    vkCtx_ = ctx;

    if (!createSourceBuffer()) {
        LOG_ERROR("GrassRenderer: failed to create source buffers");
        shutdown();
        return false;
    }
    if (!createPerFrameBuffers()) {
        LOG_ERROR("GrassRenderer: failed to create per-frame buffers");
        shutdown();
        return false;
    }
    if (!createCullPipeline()) {
        LOG_ERROR("GrassRenderer: failed to create cull pipeline");
        shutdown();
        return false;
    }
    if (!createDrawPipeline(perFrameLayout)) {
        LOG_ERROR("GrassRenderer: failed to create draw pipeline");
        shutdown();
        return false;
    }

    LOG_INFO("GrassRenderer initialized (", bladeCount_, " test blades, GPU-driven)");
    return true;
}

bool GrassRenderer::createSourceBuffer() {
    // Allocated once at full capacity and never resized. The population is
    // replaced whenever the player leaves the window it was generated for, and
    // reallocating a device-local buffer on that cadence would mean stalling
    // the queue or deferring a destroy on every rebuild.
    const std::vector<GrassBladeGPU> empty(kMaxBlades);
    AllocatedBuffer source = uploadBuffer(*vkCtx_, empty.data(),
                                          sizeof(GrassBladeGPU) * kMaxBlades,
                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    if (source.buffer == VK_NULL_HANDLE) return false;
    sourceBuffer_ = source.buffer;
    sourceAlloc_ = source.allocation;
    bladeCount_ = 0;
    setObjectName(vkCtx_->getDevice(), VK_OBJECT_TYPE_BUFFER,
                  reinterpret_cast<uint64_t>(sourceBuffer_), "grass source blades");

    // One index list, shared by every blade.
    const auto indexData = bladeIndices();
    AllocatedBuffer indices = uploadBuffer(*vkCtx_, indexData.data(),
                                           sizeof(uint16_t) * indexData.size(),
                                           VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    if (indices.buffer == VK_NULL_HANDLE) return false;
    indexBuffer_ = indices.buffer;
    indexAlloc_ = indices.allocation;

    return true;
}

bool GrassRenderer::setPopulation(const pipeline::GrassBladeSample* blades, size_t count) {
    if (!vkCtx_ || sourceBuffer_ == VK_NULL_HANDLE) return false;

    const bool complete = count <= kMaxBlades;
    count = std::min<size_t>(count, kMaxBlades);
    if (count == 0) {
        bladeCount_ = 0;
        return complete;
    }

    std::vector<GrassBladeGPU> packed(count);
    for (size_t i = 0; i < count; ++i) {
        const auto& b = blades[i];
        packed[i].positionHeight = glm::vec4(b.x, b.y, b.z, b.height);
        packed[i].facingWidthPhase = glm::vec4(b.facing, b.width, 0.0f, b.phase);
    }

    const VkDeviceSize bytes = sizeof(GrassBladeGPU) * count;
    AllocatedBuffer staging = createBuffer(vkCtx_->getAllocator(), bytes,
                                           VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                           VMA_MEMORY_USAGE_CPU_ONLY);
    if (staging.buffer == VK_NULL_HANDLE) return false;

    void* mapped = nullptr;
    if (vmaMapMemory(vkCtx_->getAllocator(), staging.allocation, &mapped) != VK_SUCCESS) {
        destroyBuffer(vkCtx_->getAllocator(), staging);
        return false;
    }
    std::memcpy(mapped, packed.data(), bytes);
    vmaUnmapMemory(vkCtx_->getAllocator(), staging.allocation);

    vkCtx_->immediateSubmit([&](VkCommandBuffer cmd) {
        VkBufferCopy region{};
        region.size = bytes;
        vkCmdCopyBuffer(cmd, staging.buffer, sourceBuffer_, 1, &region);
    });
    destroyBuffer(vkCtx_->getAllocator(), staging);

    // Only after the copy: the cull dispatches over this count, so raising it
    // before the data landed would cull against whatever was there before.
    bladeCount_ = static_cast<uint32_t>(count);
    return complete;
}

bool GrassRenderer::createPerFrameBuffers() {
    VmaAllocator allocator = vkCtx_->getAllocator();
    const VkDeviceSize visibleSize = sizeof(uint32_t) * bladeCount_;

    for (uint32_t i = 0; i < kFrames; ++i) {
        if (!createMappedBuffer(allocator, sizeof(GrassCullUniformsGPU),
                                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                cullUniform_[i], cullUniformAlloc_[i], cullUniformMapped_[i])) {
            return false;
        }
        // Device-local: written by the cull dispatch and read by the draw, both
        // on the GPU. Nothing maps these.
        if (!createGpuBuffer(allocator, visibleSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                             visibleBuffer_[i], visibleAlloc_[i])) {
            return false;
        }
        // The constant fields of the draw command, written once here. Only
        // instanceCount changes afterwards, and only the GPU changes it -
        // TRANSFER_DST is for the per-frame vkCmdFillBuffer that resets it.
        VkDrawIndexedIndirectCommand command{};
        command.indexCount = kBladeIndexCount;
        command.instanceCount = 0;
        command.firstIndex = 0;
        command.vertexOffset = 0;
        command.firstInstance = 0;
        AllocatedBuffer indirect = uploadBuffer(*vkCtx_, &command, sizeof(command),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT);
        if (indirect.buffer == VK_NULL_HANDLE) return false;
        indirectBuffer_[i] = indirect.buffer;
        indirectAlloc_[i] = indirect.allocation;
    }
    return true;
}

bool GrassRenderer::createCullPipeline() {
    VkDevice device = vkCtx_->getDevice();

    // Descriptor pool covers both sets: cull (1 UBO + 3 SSBO) and draw (2 SSBO),
    // per frame in flight.
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = kFrames;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSizes[1].descriptorCount = kFrames * 5;

    VkDescriptorPoolCreateInfo poolCi{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolCi.maxSets = kFrames * 2;
    poolCi.poolSizeCount = 2;
    poolCi.pPoolSizes = poolSizes;
    if (vkCreateDescriptorPool(device, &poolCi, nullptr, &descPool_) != VK_SUCCESS) {
        return false;
    }

    VkDescriptorSetLayoutBinding bindings[4]{};
    for (uint32_t b = 0; b < 4; ++b) {
        bindings[b].binding = b;
        bindings[b].descriptorType = (b == 0) ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                                              : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[b].descriptorCount = 1;
        bindings[b].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutCi{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutCi.bindingCount = 4;
    layoutCi.pBindings = bindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCi, nullptr, &cullSetLayout_) != VK_SUCCESS) {
        return false;
    }

    VkPipelineLayoutCreateInfo plCi{.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    plCi.setLayoutCount = 1;
    plCi.pSetLayouts = &cullSetLayout_;
    if (vkCreatePipelineLayout(device, &plCi, nullptr, &cullPipelineLayout_) != VK_SUCCESS) {
        return false;
    }

    VkShaderModule cullComp;
    if (!cullComp.loadFromFile(device, "assets/shaders/grass_cull.comp.spv")) {
        LOG_ERROR("GrassRenderer: failed to load grass_cull.comp.spv");
        return false;
    }
    VkComputePipelineCreateInfo cpCi{.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    cpCi.stage = cullComp.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
    cpCi.layout = cullPipelineLayout_;
    const VkResult cullResult = vkCreateComputePipelines(
        device, vkCtx_->getPipelineCache(), 1, &cpCi, nullptr, &cullPipeline_);
    cullComp.destroy();
    if (cullResult != VK_SUCCESS) {
        cullPipeline_ = VK_NULL_HANDLE;
        return false;
    }

    for (uint32_t i = 0; i < kFrames; ++i) {
        VkDescriptorSetAllocateInfo setAi{
            .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        setAi.descriptorPool = descPool_;
        setAi.descriptorSetCount = 1;
        setAi.pSetLayouts = &cullSetLayout_;
        if (vkAllocateDescriptorSets(device, &setAi, &cullSet_[i]) != VK_SUCCESS) {
            return false;
        }

        VkDescriptorBufferInfo uboInfo{
            .buffer = cullUniform_[i], .offset = 0, .range = sizeof(GrassCullUniformsGPU)};
        VkDescriptorBufferInfo sourceInfo{
            .buffer = sourceBuffer_, .offset = 0, .range = VK_WHOLE_SIZE};
        VkDescriptorBufferInfo visibleInfo{
            .buffer = visibleBuffer_[i], .offset = 0, .range = VK_WHOLE_SIZE};
        VkDescriptorBufferInfo indirectInfo{
            .buffer = indirectBuffer_[i], .offset = 0, .range = VK_WHOLE_SIZE};
        const VkDescriptorBufferInfo* infos[4] = {&uboInfo, &sourceInfo, &visibleInfo,
                                                  &indirectInfo};

        VkWriteDescriptorSet writes[4]{};
        for (uint32_t b = 0; b < 4; ++b) {
            writes[b] = {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[b].dstSet = cullSet_[i];
            writes[b].dstBinding = b;
            writes[b].descriptorCount = 1;
            writes[b].descriptorType = (b == 0) ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                                                : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[b].pBufferInfo = infos[b];
        }
        vkUpdateDescriptorSets(device, 4, writes, 0, nullptr);
    }
    return true;
}

bool GrassRenderer::createDrawPipeline(VkDescriptorSetLayout perFrameLayout) {
    VkDevice device = vkCtx_->getDevice();

    // Set 1: the source blades and this frame's visible list. Set 0 stays the
    // per-frame UBO, as everywhere else.
    VkDescriptorSetLayoutBinding bindings[2]{};
    for (uint32_t b = 0; b < 2; ++b) {
        bindings[b].binding = b;
        bindings[b].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[b].descriptorCount = 1;
        bindings[b].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layoutCi{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutCi.bindingCount = 2;
    layoutCi.pBindings = bindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCi, nullptr, &drawSetLayout_) != VK_SUCCESS) {
        return false;
    }

    for (uint32_t i = 0; i < kFrames; ++i) {
        VkDescriptorSetAllocateInfo setAi{
            .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        setAi.descriptorPool = descPool_;
        setAi.descriptorSetCount = 1;
        setAi.pSetLayouts = &drawSetLayout_;
        if (vkAllocateDescriptorSets(device, &setAi, &drawSet_[i]) != VK_SUCCESS) {
            return false;
        }
        VkDescriptorBufferInfo sourceInfo{
            .buffer = sourceBuffer_, .offset = 0, .range = VK_WHOLE_SIZE};
        VkDescriptorBufferInfo visibleInfo{
            .buffer = visibleBuffer_[i], .offset = 0, .range = VK_WHOLE_SIZE};
        VkWriteDescriptorSet writes[2]{};
        writes[0] = {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[0].dstSet = drawSet_[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &sourceInfo;
        writes[1] = {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[1].dstSet = drawSet_[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &visibleInfo;
        vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    }

    auto shaders = loadShaderPair(device, "assets/shaders/grass.vert.spv",
                                  "assets/shaders/grass.frag.spv", "grass");
    if (!shaders) return false;

    pipelineLayout_ = createPipelineLayout(device, {perFrameLayout, drawSetLayout_}, {});
    if (pipelineLayout_ == VK_NULL_HANDLE) return false;

    // No vertex input: the shader builds the quad from gl_VertexIndex and reads
    // everything else from the two storage buffers.
    pipeline_ = PipelineBuilder()
                    .setShaders(shaders.vertStage, shaders.fragStage)
                    .setVertexInput({}, {})
                    .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
                    // Two-sided: a blade is a single quad and is seen from both
                    // faces as the camera moves around it.
                    .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
                    .setDepthTest(true, true, VK_COMPARE_OP_LESS)
                    .setColorBlendAttachment(PipelineBuilder::blendDisabled())
                    .setMultisample(vkCtx_->getMsaaSamples())
                    .setLayout(pipelineLayout_)
                    .setRenderPass(vkCtx_->getImGuiRenderPass())
                    .setDynamicStates(viewportAndScissorDynamic())
                    .build(device, vkCtx_->getPipelineCache());

    return pipeline_ != VK_NULL_HANDLE;
}

void GrassRenderer::dispatchCull(VkCommandBuffer cmd, uint32_t frameIndex, const Camera& camera) {
    if (!isReady() || frameIndex >= kFrames || bladeCount_ == 0) return;

    // Cull parameters for this frame.
    if (cullUniformMapped_[frameIndex]) {
        GrassCullUniformsGPU uniforms{};
        // The same extractor the M2 cull uploads from, rather than a second
        // derivation. Deriving the planes by hand here got the near plane wrong
        // - the textbook form assumes OpenGL's [-1,1] depth and Vulkan clips to
        // [0,1] - which culled the whole field while looking like a draw that
        // never ran.
        const glm::mat4 viewProj = camera.getProjectionMatrix() * camera.getViewMatrix();
        Frustum frustum;
        frustum.extractFromMatrix(viewProj);
        for (int i = 0; i < 6; ++i) {
            const auto& plane = frustum.getPlane(static_cast<Frustum::Side>(i));
            uniforms.frustumPlanes[i] = glm::vec4(plane.normal, plane.distance);
        }
        uniforms.cameraPos = glm::vec4(camera.getPosition(), kMaxDistance * kMaxDistance);
        uniforms.bladeCount = bladeCount_;
        std::memcpy(cullUniformMapped_[frameIndex], &uniforms, sizeof(uniforms));
    }

    // Reset the compaction counter. Only instanceCount is zeroed; the rest of
    // the command was written once at creation and never changes.
    vkCmdFillBuffer(cmd, indirectBuffer_[frameIndex],
                    offsetof(VkDrawIndexedIndirectCommand, instanceCount), sizeof(uint32_t), 0);

    // The fill has to land before the shader starts adding to it.
    {
        VkBufferMemoryBarrier2 barrier{.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2};
        barrier.srcStageMask = VK_PIPELINE_STAGE_2_CLEAR_BIT;
        barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.buffer = indirectBuffer_[frameIndex];
        barrier.offset = 0;
        barrier.size = VK_WHOLE_SIZE;
        VkDependencyInfo dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dep.bufferMemoryBarrierCount = 1;
        dep.pBufferMemoryBarriers = &barrier;
        cmdPipelineBarrier2(cmd, dep);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cullPipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cullPipelineLayout_, 0, 1,
                            &cullSet_[frameIndex], 0, nullptr);
    vkCmdDispatch(cmd, (bladeCount_ + 63) / 64, 1, 1);

    // Hand the results to the draw: the index list is read by the vertex stage,
    // the command itself by the indirect draw. No host access anywhere.
    {
        VkMemoryBarrier2 barrier{.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
        barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT |
                               VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT;
        barrier.dstAccessMask = VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT |
                                VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
        VkDependencyInfo dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dep.memoryBarrierCount = 1;
        dep.pMemoryBarriers = &barrier;
        cmdPipelineBarrier2(cmd, dep);
    }
}

void GrassRenderer::render(VkCommandBuffer cmd, uint32_t frameIndex,
                           VkDescriptorSet perFrameSet) {
    if (!isReady() || frameIndex >= kFrames || bladeCount_ == 0) return;

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_, 0, 1,
                            &perFrameSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_, 1, 1,
                            &drawSet_[frameIndex], 0, nullptr);
    vkCmdBindIndexBuffer(cmd, indexBuffer_, 0, VK_INDEX_TYPE_UINT16);
    // One command, and its instance count came from the GPU.
    vkCmdDrawIndexedIndirect(cmd, indirectBuffer_[frameIndex], 0, 1,
                             sizeof(VkDrawIndexedIndirectCommand));
}

void GrassRenderer::shutdown() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator allocator = vkCtx_->getAllocator();

    destroyPipeline(device, pipeline_, pipelineLayout_);
    destroy(device, cullPipeline_);
    destroy(device, cullPipelineLayout_);
    destroy(device, cullSetLayout_);
    destroy(device, drawSetLayout_);
    // The sets go with the pool; freeing them individually would need
    // FREE_DESCRIPTOR_SET on it.
    destroy(device, descPool_);

    for (uint32_t i = 0; i < kFrames; ++i) {
        destroy(allocator, cullUniform_[i], cullUniformAlloc_[i]);
        cullUniformMapped_[i] = nullptr;
        destroy(allocator, visibleBuffer_[i], visibleAlloc_[i]);
        destroy(allocator, indirectBuffer_[i], indirectAlloc_[i]);
        cullSet_[i] = VK_NULL_HANDLE;
        drawSet_[i] = VK_NULL_HANDLE;
    }
    destroy(allocator, sourceBuffer_, sourceAlloc_);
    destroy(allocator, indexBuffer_, indexAlloc_);

    bladeCount_ = 0;
    vkCtx_ = nullptr;
}

} // namespace rendering
} // namespace wowee
