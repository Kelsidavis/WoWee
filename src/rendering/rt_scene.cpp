#include "rendering/rt_scene.hpp"

#include "core/logger.hpp"

#include <algorithm>
#include <cstring>

namespace wowee::rendering {

namespace {

constexpr uint32_t kBindingTriangles = 0;
constexpr uint32_t kBindingNodes = 1;
constexpr uint32_t kBindingMeshes = 2;
constexpr uint32_t kBindingInstances = 3;
constexpr uint32_t kBindingTlasNodes = 4;
constexpr uint32_t kBindingTlas = 5;

constexpr uint32_t kInitialTriangles = 1u << 18;
constexpr uint32_t kInitialNodes = 1u << 17;
// Triangles built and uploaded per frame. A binned-SAH build runs at a few
// million triangles a second on one core, so this keeps a frame's share to a
// few milliseconds while the world streams in after the switch is turned on.
constexpr uint64_t kBuildBudgetTriangles = 150000;

uint32_t grownCapacity(uint32_t current, uint32_t needed) {
    uint32_t c = std::max(current, 1024u);
    while (c < needed) c += c / 2;
    return c;
}

RtAabb transformBounds(const RtAabb& b, const glm::mat4& m) {
    RtAabb out;
    for (int i = 0; i < 8; ++i) {
        const glm::vec3 corner((i & 1) ? b.bmax.x : b.bmin.x, (i & 2) ? b.bmax.y : b.bmin.y,
                               (i & 4) ? b.bmax.z : b.bmin.z);
        out.grow(glm::vec3(m * glm::vec4(corner, 1.0f)));
    }
    return out;
}

}  // namespace

RtScene::~RtScene() { shutdown(); }

bool RtScene::initialize(VkContext* ctx) {
    ctx_ = ctx;
    alive_ = std::make_shared<bool>(true);
    backend_ = ctx->hardwareRayQueryEnabled() ? Backend::Hardware : Backend::Software;
    if (backend_ == Backend::Hardware && !loadHardwareEntryPoints()) {
        LOG_WARNING("RtScene: acceleration structure entry points missing, using the compute tracer");
        backend_ = Backend::Software;
    }

    const VkBufferUsageFlags poolUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                         VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                         VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    triPool_.elementSize = sizeof(RtTriangle);
    triPool_.usage = poolUsage;
    if (backend_ == Backend::Hardware) {
        triPool_.usage |= VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                          VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    }
    nodePool_.elementSize = sizeof(RtBvhNode);
    nodePool_.usage = poolUsage;

    placeholder_ = createBuffer(ctx_->getAllocator(), 64, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                VMA_MEMORY_USAGE_CPU_TO_GPU);
    if (!placeholder_.buffer) return false;
    std::memset(placeholder_.info.pMappedData, 0, 64);

    if (!createDescriptors()) return false;
    LOG_INFO("RtScene: ", backend_ == Backend::Hardware ? "hardware" : "software", " backend");
    return true;
}

bool RtScene::loadHardwareEntryPoints() {
    VkDevice dev = ctx_->getDevice();
    auto load = [&](const char* name) { return vkGetDeviceProcAddr(dev, name); };
    pfnGetBufferDeviceAddress_ =
        reinterpret_cast<PFN_vkGetBufferDeviceAddress>(load("vkGetBufferDeviceAddress"));
    pfnCreateAs_ = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
        load("vkCreateAccelerationStructureKHR"));
    pfnDestroyAs_ = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
        load("vkDestroyAccelerationStructureKHR"));
    pfnGetAsBuildSizes_ = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
        load("vkGetAccelerationStructureBuildSizesKHR"));
    pfnCmdBuildAs_ = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
        load("vkCmdBuildAccelerationStructuresKHR"));
    pfnGetAsAddress_ = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
        load("vkGetAccelerationStructureDeviceAddressKHR"));
    if (!pfnGetBufferDeviceAddress_ || !pfnCreateAs_ || !pfnDestroyAs_ || !pfnGetAsBuildSizes_ ||
        !pfnCmdBuildAs_ || !pfnGetAsAddress_) {
        return false;
    }

    VkPhysicalDeviceAccelerationStructurePropertiesKHR asProps{};
    asProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;
    VkPhysicalDeviceProperties2 props{};
    props.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    props.pNext = &asProps;
    vkGetPhysicalDeviceProperties2(ctx_->getPhysicalDevice(), &props);
    scratchAlignment_ = std::max<VkDeviceSize>(asProps.minAccelerationStructureScratchOffsetAlignment, 1);
    return true;
}

bool RtScene::createDescriptors() {
    VkDevice dev = ctx_->getDevice();
    std::vector<VkDescriptorSetLayoutBinding> bindings;
    auto addBinding = [&](uint32_t binding, VkDescriptorType type) {
        VkDescriptorSetLayoutBinding b{};
        b.binding = binding;
        b.descriptorType = type;
        b.descriptorCount = 1;
        b.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings.push_back(b);
    };
    addBinding(kBindingTriangles, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    addBinding(kBindingMeshes, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    addBinding(kBindingInstances, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    if (backend_ == Backend::Software) {
        addBinding(kBindingNodes, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        addBinding(kBindingTlasNodes, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    } else {
        addBinding(kBindingTlas, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR);
    }

    VkDescriptorSetLayoutCreateInfo li{};
    li.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    li.bindingCount = static_cast<uint32_t>(bindings.size());
    li.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(dev, &li, nullptr, &setLayout_) != VK_SUCCESS) return false;

    std::vector<VkDescriptorPoolSize> sizes;
    sizes.push_back({VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                     (backend_ == Backend::Software ? 5u : 3u) * MAX_FRAMES_IN_FLIGHT});
    if (backend_ == Backend::Hardware) {
        sizes.push_back({VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, MAX_FRAMES_IN_FLIGHT});
    }
    VkDescriptorPoolCreateInfo pi{};
    pi.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pi.maxSets = MAX_FRAMES_IN_FLIGHT;
    pi.poolSizeCount = static_cast<uint32_t>(sizes.size());
    pi.pPoolSizes = sizes.data();
    if (vkCreateDescriptorPool(dev, &pi, nullptr, &descriptorPool_) != VK_SUCCESS) return false;

    for (auto& fr : frames_) {
        VkDescriptorSetAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        ai.descriptorPool = descriptorPool_;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &setLayout_;
        if (vkAllocateDescriptorSets(dev, &ai, &fr.set) != VK_SUCCESS) return false;
    }
    return true;
}

void RtScene::shutdown() {
    if (!ctx_) return;
    VkDevice dev = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();
    vkDeviceWaitIdle(dev);
    alive_.reset();

    for (auto& m : meshes_) destroyMeshGpu(m);
    meshes_.clear();
    instances_.clear();
    for (auto& fr : frames_) {
        destroyBuffer(alloc, fr.instances);
        destroyBuffer(alloc, fr.tlasNodes);
        destroyBuffer(alloc, fr.asInstances);
        if (fr.tlas) pfnDestroyAs_(dev, fr.tlas, nullptr);
        destroyBuffer(alloc, fr.tlasBuffer);
        destroyBuffer(alloc, fr.scratch);
        fr = FrameResources{};
    }
    destroyBuffer(alloc, triPool_.buffer);
    destroyBuffer(alloc, nodePool_.buffer);
    destroyBuffer(alloc, meshTable_);
    destroyBuffer(alloc, placeholder_);
    if (descriptorPool_) vkDestroyDescriptorPool(dev, descriptorPool_, nullptr);
    if (setLayout_) vkDestroyDescriptorSetLayout(dev, setLayout_, nullptr);
    descriptorPool_ = VK_NULL_HANDLE;
    setLayout_ = VK_NULL_HANDLE;
    ctx_ = nullptr;
}

void RtScene::destroyMeshGpu(Mesh& m) {
    if (m.blas && pfnDestroyAs_) pfnDestroyAs_(ctx_->getDevice(), m.blas, nullptr);
    m.blas = VK_NULL_HANDLE;
    destroyBuffer(ctx_->getAllocator(), m.blasBuffer);
}

// ---------------------------------------------------------------------------
// Registration

RtScene::MeshId RtScene::addMesh(MeshSource source) {
    if (!ctx_ || source.indices.size() < 3 || source.surfaces.empty()) return kInvalid;
    const size_t triCount = source.indices.size() / 3;
    if (source.surfaces.size() != 1 && source.surfaces.size() != triCount) return kInvalid;
    RtAabb bounds;
    for (uint32_t idx : source.indices) {
        if (idx >= source.positions.size()) return kInvalid;
        bounds.grow(source.positions[idx]);
    }

    MeshId id;
    if (!freeMeshes_.empty()) {
        id = freeMeshes_.back();
        freeMeshes_.pop_back();
    } else {
        id = static_cast<MeshId>(meshes_.size());
        meshes_.emplace_back();
    }
    Mesh& m = meshes_[id];
    m = Mesh{};
    m.live = true;
    m.triCount = static_cast<uint32_t>(triCount);
    m.bounds = bounds;
    m.source = std::move(source);
    ++liveMeshes_;
    liveTriangles_ += m.triCount;
    if (active_) {
        m.queued = true;
        pendingMeshes_.push_back(id);
    }
    return id;
}

void RtScene::buildMesh(Mesh& m) {
    const MeshSource& src = m.source;
    m.tris.resize(m.triCount);
    m.hasTranslucent = false;
    for (uint32_t t = 0; t < m.triCount; ++t) {
        const float surface = src.surfaces.size() == 1 ? src.surfaces[0] : src.surfaces[t];
        RtTriangle& tri = m.tris[t];
        tri.v0 = glm::vec4(src.positions[src.indices[t * 3]], surface);
        tri.v1 = glm::vec4(src.positions[src.indices[t * 3 + 1]], 0.0f);
        tri.v2 = glm::vec4(src.positions[src.indices[t * 3 + 2]], 0.0f);
        if (unpackRtSurface(surface).a < 1.0f) m.hasTranslucent = true;
    }
    RtBvh bvh = buildRtTriangleBvh(m.tris);
    m.nodeCount = static_cast<uint32_t>(bvh.nodes.size());
    m.nodes = std::move(bvh.nodes);
}

void RtScene::setActive(bool active) {
    if (active == active_ || !ctx_) return;
    active_ = active;
    if (!active) {
        releaseGpu();
        return;
    }
    for (MeshId id = 0; id < meshes_.size(); ++id) {
        Mesh& m = meshes_[id];
        if (m.live && !m.queued && !m.uploaded) {
            m.queued = true;
            pendingMeshes_.push_back(id);
        }
    }
    ++sceneVersion_;
}

void RtScene::releaseGpu() {
    VmaAllocator alloc = ctx_->getAllocator();
    VkDevice dev = ctx_->getDevice();
    ++gpuEpoch_;
    for (auto& m : meshes_) {
        destroyMeshGpu(m);
        m.uploaded = false;
        m.queued = false;
        m.triOffset = m.nodeOffset = RtRangeAllocator::kFailed;
        m.tris.clear();
        m.tris.shrink_to_fit();
        m.nodes.clear();
        m.nodes.shrink_to_fit();
    }
    pendingMeshes_.clear();
    for (auto& fr : frames_) {
        destroyBuffer(alloc, fr.instances);
        destroyBuffer(alloc, fr.tlasNodes);
        destroyBuffer(alloc, fr.asInstances);
        if (fr.tlas) pfnDestroyAs_(dev, fr.tlas, nullptr);
        destroyBuffer(alloc, fr.tlasBuffer);
        destroyBuffer(alloc, fr.scratch);
        const VkDescriptorSet set = fr.set;
        fr = FrameResources{};
        fr.set = set;
    }
    for (GpuPool* p : {&triPool_, &nodePool_}) {
        destroyBuffer(alloc, p->buffer);
        p->alloc = RtRangeAllocator{};
    }
    destroyBuffer(alloc, meshTable_);
    meshTableCapacity_ = 0;
    meshTableDirty_ = true;
    builtVersion_ = ~0ull;
    ++sceneVersion_;
    ++descriptorGeneration_;
}

void RtScene::removeMesh(MeshId id) {
    if (id >= meshes_.size() || !meshes_[id].live) return;
    Mesh& m = meshes_[id];
    m.live = false;
    --liveMeshes_;
    liveTriangles_ -= m.triCount;
    pendingMeshes_.erase(std::remove(pendingMeshes_.begin(), pendingMeshes_.end(), id),
                         pendingMeshes_.end());

    // In-flight frames may still trace this mesh. Its pool ranges, BLAS and
    // slot are released only once every frame slot has been fenced.
    const uint32_t triOffset = m.triOffset, triCount = m.triCount;
    const uint32_t nodeOffset = m.nodeOffset, nodeCount = m.nodeCount;
    VkAccelerationStructureKHR blas = m.blas;
    AllocatedBuffer blasBuffer = m.blasBuffer;
    m = Mesh{};
    // The BLAS is destroyed whether or not the scene is still alive by then;
    // the pool ranges and the slot only matter to a scene that is.
    std::weak_ptr<bool> alive = alive_;
    const uint64_t epoch = gpuEpoch_;
    VkDevice dev = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();
    auto destroyAs = pfnDestroyAs_;
    ctx_->deferAfterAllFrameFences([this, alive, epoch, dev, alloc, destroyAs, id, triOffset,
                                    triCount, nodeOffset, nodeCount, blas,
                                    blasBuffer]() mutable {
        if (blas) destroyAs(dev, blas, nullptr);
        destroyBuffer(alloc, blasBuffer);
        if (alive.expired()) return;
        if (epoch == gpuEpoch_ && triOffset != RtRangeAllocator::kFailed) triPool_.alloc.release(triOffset, triCount);
        if (epoch == gpuEpoch_ && nodeOffset != RtRangeAllocator::kFailed)
            nodePool_.alloc.release(nodeOffset, nodeCount);
        freeMeshes_.push_back(id);
    });
}

RtScene::InstanceId RtScene::addInstance(MeshId mesh, const glm::mat4& objectToWorld) {
    if (mesh >= meshes_.size() || !meshes_[mesh].live) return kInvalid;
    InstanceId id;
    if (!freeInstances_.empty()) {
        id = freeInstances_.back();
        freeInstances_.pop_back();
    } else {
        id = static_cast<InstanceId>(instances_.size());
        instances_.emplace_back();
    }
    instances_[id] = Instance{true, mesh, objectToWorld};
    ++liveInstances_;
    ++sceneVersion_;
    return id;
}

void RtScene::setInstanceTransform(InstanceId id, const glm::mat4& objectToWorld) {
    if (id >= instances_.size() || !instances_[id].live) return;
    if (instances_[id].objectToWorld == objectToWorld) return;
    instances_[id].objectToWorld = objectToWorld;
    ++sceneVersion_;
}

void RtScene::removeInstance(InstanceId id) {
    if (id >= instances_.size() || !instances_[id].live) return;
    instances_[id] = Instance{};
    freeInstances_.push_back(id);
    --liveInstances_;
    ++sceneVersion_;
}

// ---------------------------------------------------------------------------
// GPU upkeep

VkDeviceAddress RtScene::bufferAddress(VkBuffer buffer) const {
    VkBufferDeviceAddressInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    info.buffer = buffer;
    return pfnGetBufferDeviceAddress_(ctx_->getDevice(), &info);
}

void RtScene::stage(VkCommandBuffer cmd, VkBuffer dst, VkDeviceSize dstOffset, const void* data,
                    VkDeviceSize size) {
    if (size == 0) return;
    AllocatedBuffer staging = createBuffer(ctx_->getAllocator(), size,
                                           VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                           VMA_MEMORY_USAGE_CPU_ONLY);
    if (!staging.buffer) return;
    std::memcpy(staging.info.pMappedData, data, size);
    VkBufferCopy region{0, dstOffset, size};
    vkCmdCopyBuffer(cmd, staging.buffer, dst, 1, &region);
    VmaAllocator alloc = ctx_->getAllocator();
    ctx_->deferAfterFrameFence([alloc, staging]() mutable { destroyBuffer(alloc, staging); });
}

void RtScene::ensurePoolCapacity(VkCommandBuffer cmd, GpuPool& pool, uint32_t needed) {
    if (pool.buffer.buffer && pool.alloc.capacity() >= needed) return;
    const uint32_t newCap = grownCapacity(pool.alloc.capacity(), needed);
    AllocatedBuffer grown = createBuffer(ctx_->getAllocator(),
                                         VkDeviceSize(newCap) * pool.elementSize, pool.usage,
                                         VMA_MEMORY_USAGE_GPU_ONLY);
    if (!grown.buffer) return;
    if (pool.buffer.buffer && pool.alloc.capacity() > 0) {
        VkBufferCopy region{0, 0, VkDeviceSize(pool.alloc.capacity()) * pool.elementSize};
        vkCmdCopyBuffer(cmd, pool.buffer.buffer, grown.buffer, 1, &region);
        VmaAllocator alloc = ctx_->getAllocator();
        AllocatedBuffer old = pool.buffer;
        ctx_->deferAfterAllFrameFences([alloc, old]() mutable { destroyBuffer(alloc, old); });
    }
    pool.buffer = grown;
    pool.alloc.grow(newCap);
    ++descriptorGeneration_;
}

uint32_t RtScene::poolAllocate(VkCommandBuffer cmd, GpuPool& pool, uint32_t count) {
    uint32_t off = pool.alloc.allocate(count);
    if (off != RtRangeAllocator::kFailed) return off;
    ensurePoolCapacity(cmd, pool, pool.alloc.capacity() + count);
    return pool.alloc.allocate(count);
}

void RtScene::uploadPending(VkCommandBuffer cmd) {
    if (pendingMeshes_.empty()) return;
    if (!triPool_.buffer.buffer) ensurePoolCapacity(cmd, triPool_, kInitialTriangles);
    if (backend_ == Backend::Software && !nodePool_.buffer.buffer) {
        ensurePoolCapacity(cmd, nodePool_, kInitialNodes);
    }

    // Build this frame's share from the sources.
    std::vector<MeshId> batch;
    uint64_t built = 0;
    size_t consumed = 0;
    for (; consumed < pendingMeshes_.size() && built < kBuildBudgetTriangles; ++consumed) {
        const MeshId id = pendingMeshes_[consumed];
        Mesh& m = meshes_[id];
        if (!m.live || !m.queued) continue;
        m.queued = false;
        buildMesh(m);
        built += m.triCount;
        batch.push_back(id);
    }
    pendingMeshes_.erase(pendingMeshes_.begin(), pendingMeshes_.begin() + consumed);
    if (batch.empty()) return;

    // Allocate everything first: a pool growth copies the old contents, and
    // that copy has to precede the uploads into it.
    std::vector<MeshId> ready;
    for (MeshId id : batch) {
        Mesh& m = meshes_[id];
        m.triOffset = poolAllocate(cmd, triPool_, m.triCount);
        if (backend_ == Backend::Software) m.nodeOffset = poolAllocate(cmd, nodePool_, m.nodeCount);
        if (m.triOffset == RtRangeAllocator::kFailed ||
            (backend_ == Backend::Software && m.nodeOffset == RtRangeAllocator::kFailed)) {
            LOG_WARNING("RtScene: out of memory for a mesh of ", m.triCount, " triangles");
            if (m.triOffset != RtRangeAllocator::kFailed) triPool_.alloc.release(m.triOffset, m.triCount);
            if (m.nodeOffset != RtRangeAllocator::kFailed) nodePool_.alloc.release(m.nodeOffset, m.nodeCount);
            m.triOffset = m.nodeOffset = RtRangeAllocator::kFailed;
            m.tris.clear();
            m.nodes.clear();
            continue;
        }
        ready.push_back(id);
    }

    VkMemoryBarrier2 copyOrder{};
    copyOrder.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    copyOrder.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    copyOrder.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    copyOrder.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    copyOrder.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    VkDependencyInfo dep{};
    dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.memoryBarrierCount = 1;
    dep.pMemoryBarriers = &copyOrder;
    cmdPipelineBarrier2(cmd, dep);

    for (MeshId id : ready) {
        Mesh& m = meshes_[id];
        stage(cmd, triPool_.buffer.buffer, VkDeviceSize(m.triOffset) * sizeof(RtTriangle),
              m.tris.data(), m.tris.size() * sizeof(RtTriangle));
        if (backend_ == Backend::Software) {
            // Child indices are stored relative to the mesh; the shader adds
            // the mesh's node offset, so nothing here needs rewriting.
            stage(cmd, nodePool_.buffer.buffer, VkDeviceSize(m.nodeOffset) * sizeof(RtBvhNode),
                  m.nodes.data(), m.nodes.size() * sizeof(RtBvhNode));
        }
    }

    VkMemoryBarrier2 toReaders{};
    toReaders.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    toReaders.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    toReaders.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    toReaders.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    toReaders.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
    if (backend_ == Backend::Hardware) {
        toReaders.dstStageMask |= VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
        toReaders.dstAccessMask |= VK_ACCESS_2_SHADER_READ_BIT;
    }
    dep.pMemoryBarriers = &toReaders;
    cmdPipelineBarrier2(cmd, dep);

    if (backend_ == Backend::Hardware) buildPendingBlas(cmd, ready);

    for (MeshId id : ready) {
        Mesh& m = meshes_[id];
        m.uploaded = true;
        m.tris.clear();
        m.tris.shrink_to_fit();
        m.nodes.clear();
        m.nodes.shrink_to_fit();
    }
    meshTableDirty_ = true;
    ++sceneVersion_;
}

void RtScene::buildPendingBlas(VkCommandBuffer cmd, const std::vector<MeshId>& ids) {
    VkDevice dev = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();
    const VkDeviceAddress poolAddress = bufferAddress(triPool_.buffer.buffer);

    for (MeshId id : ids) {
        Mesh& m = meshes_[id];

        VkAccelerationStructureGeometryKHR geom{};
        geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        // Translucent meshes go through the candidate loop in the shader, which
        // reads the triangle's opacity from the pool.
        geom.flags = m.hasTranslucent ? 0 : VK_GEOMETRY_OPAQUE_BIT_KHR;
        auto& tri = geom.geometry.triangles;
        tri.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        tri.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        tri.vertexData.deviceAddress = poolAddress + VkDeviceAddress(m.triOffset) * sizeof(RtTriangle);
        tri.vertexStride = sizeof(glm::vec4);
        tri.maxVertex = m.triCount * 3 - 1;
        tri.indexType = VK_INDEX_TYPE_NONE_KHR;

        VkAccelerationStructureBuildGeometryInfoKHR build{};
        build.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        build.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        build.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        build.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        build.geometryCount = 1;
        build.pGeometries = &geom;

        VkAccelerationStructureBuildSizesInfoKHR sizes{};
        sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        pfnGetAsBuildSizes_(dev, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build,
                            &m.triCount, &sizes);

        m.blasBuffer = createBuffer(alloc, sizes.accelerationStructureSize,
                                    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                    VMA_MEMORY_USAGE_GPU_ONLY);
        AllocatedBuffer scratch = createBuffer(alloc, sizes.buildScratchSize + scratchAlignment_,
                                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                   VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                               VMA_MEMORY_USAGE_GPU_ONLY);
        if (!m.blasBuffer.buffer || !scratch.buffer) {
            destroyBuffer(alloc, scratch);
            continue;
        }

        VkAccelerationStructureCreateInfoKHR ci{};
        ci.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        ci.buffer = m.blasBuffer.buffer;
        ci.size = sizes.accelerationStructureSize;
        ci.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        if (pfnCreateAs_(dev, &ci, nullptr, &m.blas) != VK_SUCCESS) {
            destroyBuffer(alloc, scratch);
            continue;
        }

        const VkDeviceAddress scratchAddr = bufferAddress(scratch.buffer);
        build.dstAccelerationStructure = m.blas;
        build.scratchData.deviceAddress =
            (scratchAddr + scratchAlignment_ - 1) & ~(scratchAlignment_ - 1);
        VkAccelerationStructureBuildRangeInfoKHR range{};
        range.primitiveCount = m.triCount;
        const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
        pfnCmdBuildAs_(cmd, 1, &build, &ranges);
        ctx_->deferAfterFrameFence([alloc, scratch]() mutable { destroyBuffer(alloc, scratch); });

        VkAccelerationStructureDeviceAddressInfoKHR ai{};
        ai.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
        ai.accelerationStructure = m.blas;
        m.blasAddress = pfnGetAsAddress_(dev, &ai);
    }

    // BLAS builds must finish before the TLAS build that references them.
    VkMemoryBarrier2 mb{};
    mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    mb.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    mb.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    mb.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    mb.dstAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    VkDependencyInfo dep{};
    dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.memoryBarrierCount = 1;
    dep.pMemoryBarriers = &mb;
    cmdPipelineBarrier2(cmd, dep);
}

void RtScene::ensureMapped(AllocatedBuffer& buf, uint32_t& capacity, uint32_t needed,
                           uint32_t elementSize, VkBufferUsageFlags usage) {
    if (buf.buffer && capacity >= needed) return;
    VmaAllocator alloc = ctx_->getAllocator();
    if (buf.buffer) {
        // This frame slot's previous use has been fenced, but the descriptor
        // set of this slot still names the buffer until it is rewritten below.
        AllocatedBuffer old = buf;
        ctx_->deferAfterFrameFence([alloc, old]() mutable { destroyBuffer(alloc, old); });
    }
    capacity = grownCapacity(capacity, std::max(needed, 1u));
    buf = createBuffer(alloc, VkDeviceSize(capacity) * elementSize, usage,
                       VMA_MEMORY_USAGE_CPU_TO_GPU);
    ++descriptorGeneration_;
}

void RtScene::uploadFrame(VkCommandBuffer cmd, FrameResources& fr) {
    // Rebuild the CPU-side per-frame data once per scene version.
    if (builtVersion_ != sceneVersion_) {
        builtVersion_ = sceneVersion_;
        instanceData_.clear();
        asInstanceData_.clear();
        std::vector<RtAabb> bounds;
        for (const Instance& inst : instances_) {
            if (!inst.live) continue;
            const Mesh& m = meshes_[inst.mesh];
            if (!m.uploaded) continue;  // failed allocation, or not yet uploaded
            if (backend_ == Backend::Hardware && !m.blas) continue;
            InstanceGPU g{};
            const glm::mat4 inv = glm::inverse(inst.objectToWorld);
            const glm::mat4 invT = glm::transpose(inv);
            g.worldToObject[0] = invT[0];
            g.worldToObject[1] = invT[1];
            g.worldToObject[2] = invT[2];
            g.mesh = inst.mesh;
            instanceData_.push_back(g);
            bounds.push_back(transformBounds(m.bounds, inst.objectToWorld));

            if (backend_ == Backend::Hardware) {
                VkAccelerationStructureInstanceKHR ai{};
                const glm::mat4 t = glm::transpose(inst.objectToWorld);
                std::memcpy(&ai.transform, &t, sizeof(ai.transform));
                ai.instanceCustomIndex = static_cast<uint32_t>(instanceData_.size() - 1);
                ai.mask = 0xFF;
                ai.accelerationStructureReference = m.blasAddress;
                asInstanceData_.push_back(ai);
            }
        }
        tlasNodes_.clear();
        if (backend_ == Backend::Software && !instanceData_.empty()) {
            RtBvh top = buildRtBvh(bounds, 1);
            // Leaves index the reordered instance array; reorder to match.
            std::vector<InstanceGPU> sorted(instanceData_.size());
            for (size_t i = 0; i < top.order.size(); ++i) sorted[i] = instanceData_[top.order[i]];
            instanceData_.swap(sorted);
            tlasNodes_ = std::move(top.nodes);
        }
    }

    if (fr.uploadedVersion == sceneVersion_) return;
    fr.uploadedVersion = sceneVersion_;
    fr.instanceCount = static_cast<uint32_t>(instanceData_.size());

    ensureMapped(fr.instances, fr.instanceCapacity, fr.instanceCount, sizeof(InstanceGPU),
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    if (!instanceData_.empty()) {
        std::memcpy(fr.instances.info.pMappedData, instanceData_.data(),
                    instanceData_.size() * sizeof(InstanceGPU));
    }
    if (backend_ == Backend::Software) {
        ensureMapped(fr.tlasNodes, fr.tlasNodeCapacity, static_cast<uint32_t>(tlasNodes_.size()),
                     sizeof(RtBvhNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        if (!tlasNodes_.empty()) {
            std::memcpy(fr.tlasNodes.info.pMappedData, tlasNodes_.data(),
                        tlasNodes_.size() * sizeof(RtBvhNode));
        }
    } else {
        buildTlas(cmd, fr);
    }
}

void RtScene::buildTlas(VkCommandBuffer cmd, FrameResources& fr) {
    VkDevice dev = ctx_->getDevice();
    VmaAllocator alloc = ctx_->getAllocator();
    const uint32_t count = static_cast<uint32_t>(asInstanceData_.size());

    ensureMapped(fr.asInstances, fr.asInstanceCapacity, count,
                 sizeof(VkAccelerationStructureInstanceKHR),
                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
    if (count > 0) {
        std::memcpy(fr.asInstances.info.pMappedData, asInstanceData_.data(),
                    count * sizeof(VkAccelerationStructureInstanceKHR));
    }

    VkAccelerationStructureGeometryKHR geom{};
    geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    geom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geom.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geom.geometry.instances.data.deviceAddress = bufferAddress(fr.asInstances.buffer);

    VkAccelerationStructureBuildGeometryInfoKHR build{};
    build.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    build.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    build.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build.geometryCount = 1;
    build.pGeometries = &geom;

    VkAccelerationStructureBuildSizesInfoKHR sizes{};
    sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    pfnGetAsBuildSizes_(dev, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build, &count, &sizes);

    if (!fr.tlas || fr.tlasSize < sizes.accelerationStructureSize) {
        if (fr.tlas) {
            VkAccelerationStructureKHR oldAs = fr.tlas;
            AllocatedBuffer oldBuf = fr.tlasBuffer;
            auto destroyAs = pfnDestroyAs_;
            ctx_->deferAfterFrameFence([dev, alloc, oldAs, oldBuf, destroyAs]() mutable {
                destroyAs(dev, oldAs, nullptr);
                destroyBuffer(alloc, oldBuf);
            });
        }
        fr.tlasSize = sizes.accelerationStructureSize + sizes.accelerationStructureSize / 2;
        fr.tlasBuffer = createBuffer(alloc, fr.tlasSize,
                                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                         VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                     VMA_MEMORY_USAGE_GPU_ONLY);
        VkAccelerationStructureCreateInfoKHR ci{};
        ci.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        ci.buffer = fr.tlasBuffer.buffer;
        ci.size = fr.tlasSize;
        ci.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        fr.tlas = VK_NULL_HANDLE;
        if (pfnCreateAs_(dev, &ci, nullptr, &fr.tlas) != VK_SUCCESS) {
            fr.tlas = VK_NULL_HANDLE;
            return;
        }
        ++descriptorGeneration_;
    }
    if (!fr.scratch.buffer || fr.scratchSize < sizes.buildScratchSize) {
        if (fr.scratch.buffer) {
            AllocatedBuffer old = fr.scratch;
            ctx_->deferAfterFrameFence([alloc, old]() mutable { destroyBuffer(alloc, old); });
        }
        fr.scratchSize = sizes.buildScratchSize + sizes.buildScratchSize / 2;
        fr.scratch = createBuffer(alloc, fr.scratchSize + scratchAlignment_,
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                  VMA_MEMORY_USAGE_GPU_ONLY);
    }

    const VkDeviceAddress scratchAddr = bufferAddress(fr.scratch.buffer);
    build.dstAccelerationStructure = fr.tlas;
    build.scratchData.deviceAddress = (scratchAddr + scratchAlignment_ - 1) & ~(scratchAlignment_ - 1);
    VkAccelerationStructureBuildRangeInfoKHR range{};
    range.primitiveCount = count;
    const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
    pfnCmdBuildAs_(cmd, 1, &build, &ranges);

    VkMemoryBarrier2 mb{};
    mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    mb.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    mb.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    mb.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    mb.dstAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    VkDependencyInfo dep{};
    dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.memoryBarrierCount = 1;
    dep.pMemoryBarriers = &mb;
    cmdPipelineBarrier2(cmd, dep);
}

void RtScene::writeDescriptors(FrameResources& fr) {
    if (fr.descriptorGeneration == descriptorGeneration_) return;
    fr.descriptorGeneration = descriptorGeneration_;

    auto info = [&](const AllocatedBuffer& b) {
        return VkDescriptorBufferInfo{b.buffer ? b.buffer : placeholder_.buffer, 0, VK_WHOLE_SIZE};
    };
    VkDescriptorBufferInfo tris = info(triPool_.buffer);
    VkDescriptorBufferInfo nodes = info(nodePool_.buffer);
    VkDescriptorBufferInfo meshes = info(meshTable_);
    VkDescriptorBufferInfo insts = info(fr.instances);
    VkDescriptorBufferInfo tlasNodes = info(fr.tlasNodes);

    std::vector<VkWriteDescriptorSet> writes;
    auto add = [&](uint32_t binding, const VkDescriptorBufferInfo* bi) {
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet = fr.set;
        w.dstBinding = binding;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        w.pBufferInfo = bi;
        writes.push_back(w);
    };
    add(kBindingTriangles, &tris);
    add(kBindingMeshes, &meshes);
    add(kBindingInstances, &insts);
    VkWriteDescriptorSetAccelerationStructureKHR asWrite{};
    if (backend_ == Backend::Software) {
        add(kBindingNodes, &nodes);
        add(kBindingTlasNodes, &tlasNodes);
    } else if (fr.tlas) {
        asWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
        asWrite.accelerationStructureCount = 1;
        asWrite.pAccelerationStructures = &fr.tlas;
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.pNext = &asWrite;
        w.dstSet = fr.set;
        w.dstBinding = kBindingTlas;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        writes.push_back(w);
    }
    vkUpdateDescriptorSets(ctx_->getDevice(), static_cast<uint32_t>(writes.size()), writes.data(),
                           0, nullptr);
}

void RtScene::update(VkCommandBuffer cmd) {
    if (!ctx_ || !active_) return;
    uploadPending(cmd);

    if (meshTableDirty_) {
        meshTableDirty_ = false;
        std::vector<MeshGPU> table(meshes_.size());
        for (size_t i = 0; i < meshes_.size(); ++i) {
            table[i] = {meshes_[i].nodeOffset, meshes_[i].triOffset, 0, 0};
        }
        if (!meshTable_.buffer || meshTableCapacity_ < table.size()) {
            if (meshTable_.buffer) {
                VmaAllocator alloc = ctx_->getAllocator();
                AllocatedBuffer old = meshTable_;
                ctx_->deferAfterAllFrameFences([alloc, old]() mutable { destroyBuffer(alloc, old); });
            }
            meshTableCapacity_ = grownCapacity(meshTableCapacity_, static_cast<uint32_t>(table.size()));
            meshTable_ = createBuffer(ctx_->getAllocator(),
                                      VkDeviceSize(meshTableCapacity_) * sizeof(MeshGPU),
                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                          VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                      VMA_MEMORY_USAGE_GPU_ONLY);
            ++descriptorGeneration_;
        }
        // Rows of live meshes never change after upload and freed rows are
        // unreferenced, so rewriting the table while an earlier frame reads
        // it only changes rows no one is looking at.
        stage(cmd, meshTable_.buffer, 0, table.data(), table.size() * sizeof(MeshGPU));
        VkMemoryBarrier2 mb{};
        mb.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
        mb.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        mb.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        mb.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        mb.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
        VkDependencyInfo dep{};
        dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
        dep.memoryBarrierCount = 1;
        dep.pMemoryBarriers = &mb;
        cmdPipelineBarrier2(cmd, dep);
    }

    FrameResources& fr = frames_[ctx_->getCurrentFrame()];
    uploadFrame(cmd, fr);
    writeDescriptors(fr);
}

VkDescriptorSet RtScene::descriptorSet() const {
    return ctx_ ? frames_[ctx_->getCurrentFrame()].set : VK_NULL_HANDLE;
}

RtScene::Stats RtScene::stats() const {
    Stats s;
    s.meshes = liveMeshes_;
    s.instances = liveInstances_;
    s.triangles = liveTriangles_;
    auto sz = [](const AllocatedBuffer& b) { return b.buffer ? uint64_t(b.info.size) : 0ull; };
    s.gpuBytes = sz(triPool_.buffer) + sz(nodePool_.buffer) + sz(meshTable_);
    for (const auto& m : meshes_) s.gpuBytes += sz(m.blasBuffer);
    return s;
}

}  // namespace wowee::rendering
