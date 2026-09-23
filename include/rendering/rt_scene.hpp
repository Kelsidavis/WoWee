#pragma once

/// The world as the ray traced lighting sees it.
///
/// Renderers register geometry here as they load it - a mesh per terrain
/// chunk, WMO group or M2 model, in its own object space - and an instance for
/// every placement of it. Each frame `update` uploads what changed and brings
/// the acceleration structures up to date, and the trace shaders bind
/// `descriptorSet()` at set 1.
///
/// Two backends behind the one interface:
///  - software: a two-level BVH walked in compute (`rt_trace_sw.glsli`). The
///    bottom levels are built on the CPU when a mesh is added; the top level
///    is rebuilt on the CPU when the instance set changes.
///  - hardware: VK_KHR_acceleration_structure + ray queries
///    (`rt_trace_hw.glsli`). The BLAS of each mesh is built from the same
///    triangle pool the software path reads, so hit lookups are identical.
///
/// Geometry is kept as compact indexed source meshes whether or not the
/// lighting is on. Only while it is active are BVHs built and the pools
/// filled - a few hundred thousand triangles a frame, so switching it on
/// streams the world in rather than stalling - and switching it off frees
/// every GPU byte.
///
/// Main thread only. Removing a mesh that instances still use is a caller bug.

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include "rendering/rt_bvh.hpp"
#include "rendering/rt_range_allocator.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_utils.hpp"

namespace wowee::rendering {

class RtScene {
public:
    using MeshId = uint32_t;
    using InstanceId = uint32_t;
    static constexpr uint32_t kInvalid = ~0u;

    enum class Backend { Software, Hardware };

    RtScene() = default;
    ~RtScene();
    RtScene(const RtScene&) = delete;
    RtScene& operator=(const RtScene&) = delete;

    bool initialize(VkContext* ctx);
    void shutdown();

    [[nodiscard]] Backend backend() const { return backend_; }

    /// An indexed triangle mesh in object space. `surfaces` holds one packed
    /// surface (packRtSurface) per triangle, or a single one for all of them.
    struct MeshSource {
        std::vector<glm::vec3> positions;
        std::vector<uint32_t> indices;
        std::vector<float> surfaces;
    };
    /// Returns kInvalid for a mesh with no usable triangle.
    MeshId addMesh(MeshSource source);
    void removeMesh(MeshId mesh);

    InstanceId addInstance(MeshId mesh, const glm::mat4& objectToWorld);
    void setInstanceTransform(InstanceId instance, const glm::mat4& objectToWorld);
    void removeInstance(InstanceId instance);

    /// Build and upload while active; release everything GPU-side when not.
    /// Switching off must happen with the device idle.
    void setActive(bool active);
    [[nodiscard]] bool isActive() const { return active_; }

    /// Record uploads and acceleration structure builds for this frame.
    /// Must run outside a render pass and before any trace dispatch.
    void update(VkCommandBuffer cmd);

    [[nodiscard]] VkDescriptorSetLayout descriptorSetLayout() const { return setLayout_; }
    /// The set for the current frame slot. Valid after update().
    [[nodiscard]] VkDescriptorSet descriptorSet() const;
    /// False until at least one instance has made it into an acceleration
    /// structure; tracing an empty scene is legal but pointless.
    [[nodiscard]] bool hasContent() const { return liveInstances_ > 0; }

    struct Stats {
        uint32_t meshes = 0;
        uint32_t instances = 0;
        uint64_t triangles = 0;
        uint64_t gpuBytes = 0;
    };
    [[nodiscard]] Stats stats() const;

private:
    struct Mesh {
        bool live = false;
        uint32_t triOffset = RtRangeAllocator::kFailed;
        uint32_t triCount = 0;
        uint32_t nodeOffset = RtRangeAllocator::kFailed;
        uint32_t nodeCount = 0;
        RtAabb bounds;
        bool uploaded = false;
        bool queued = false;
        MeshSource source;
        // Built from the source for one upload; cleared once copied.
        std::vector<RtTriangle> tris;
        std::vector<RtBvhNode> nodes;
        // Hardware backend.
        AllocatedBuffer blasBuffer{};
        VkAccelerationStructureKHR blas = VK_NULL_HANDLE;
        VkDeviceAddress blasAddress = 0;
        bool hasTranslucent = false;
    };
    struct Instance {
        bool live = false;
        MeshId mesh = kInvalid;
        glm::mat4 objectToWorld{1.0f};
    };
    /// What a trace shader reads per mesh (std430).
    struct MeshGPU {
        uint32_t nodeOffset;
        uint32_t triOffset;
        uint32_t pad0;
        uint32_t pad1;
    };
    /// What a trace shader reads per instance (std430): the world-to-object
    /// transform as three rows, and the mesh it places.
    struct InstanceGPU {
        glm::vec4 worldToObject[3];
        uint32_t mesh;
        uint32_t pad[3];
    };

    struct GpuPool {
        AllocatedBuffer buffer{};
        RtRangeAllocator alloc;
        uint32_t elementSize = 0;
        VkBufferUsageFlags usage = 0;
    };
    struct FrameResources {
        AllocatedBuffer instances{};   // InstanceGPU[], mapped
        uint32_t instanceCapacity = 0;
        AllocatedBuffer tlasNodes{};   // RtBvhNode[], mapped (software)
        uint32_t tlasNodeCapacity = 0;
        // Hardware TLAS.
        AllocatedBuffer asInstances{}; // VkAccelerationStructureInstanceKHR[], mapped
        uint32_t asInstanceCapacity = 0;
        AllocatedBuffer tlasBuffer{};
        VkAccelerationStructureKHR tlas = VK_NULL_HANDLE;
        VkDeviceSize tlasSize = 0;
        AllocatedBuffer scratch{};
        VkDeviceSize scratchSize = 0;
        uint64_t uploadedVersion = ~0ull;
        uint64_t descriptorGeneration = ~0ull;
        uint32_t instanceCount = 0;
        VkDescriptorSet set = VK_NULL_HANDLE;
    };

    bool loadHardwareEntryPoints();
    bool createDescriptors();
    void ensurePoolCapacity(VkCommandBuffer cmd, GpuPool& pool, uint32_t needed);
    uint32_t poolAllocate(VkCommandBuffer cmd, GpuPool& pool, uint32_t count);
    void uploadPending(VkCommandBuffer cmd);
    void buildPendingBlas(VkCommandBuffer cmd, const std::vector<MeshId>& meshes);
    void uploadFrame(VkCommandBuffer cmd, FrameResources& fr);
    void buildTlas(VkCommandBuffer cmd, FrameResources& fr);
    void writeDescriptors(FrameResources& fr);
    void destroyMeshGpu(Mesh& m);
    void releaseGpu();
    void buildMesh(Mesh& m);
    VkDeviceAddress bufferAddress(VkBuffer buffer) const;
    void ensureMapped(AllocatedBuffer& buf, uint32_t& capacity, uint32_t needed,
                      uint32_t elementSize, VkBufferUsageFlags usage);
    void stage(VkCommandBuffer cmd, VkBuffer dst, VkDeviceSize dstOffset, const void* data,
               VkDeviceSize size);

    VkContext* ctx_ = nullptr;
    // Deferred releases check this before touching the pools.
    std::shared_ptr<bool> alive_;
    bool active_ = false;
    // Bumped by releaseGpu(); a deferred release from an older epoch refers
    // to pools that no longer exist.
    uint64_t gpuEpoch_ = 0;
    Backend backend_ = Backend::Software;

    std::vector<Mesh> meshes_;
    std::vector<MeshId> freeMeshes_;
    std::vector<MeshId> pendingMeshes_;
    std::vector<Instance> instances_;
    std::vector<InstanceId> freeInstances_;
    uint32_t liveMeshes_ = 0;
    uint32_t liveInstances_ = 0;
    uint64_t liveTriangles_ = 0;

    GpuPool triPool_;
    GpuPool nodePool_;
    AllocatedBuffer meshTable_{};
    uint32_t meshTableCapacity_ = 0;
    bool meshTableDirty_ = true;

    // Bumped whenever the instance set, a transform, or a mesh's placement in
    // the pools changes; each frame slot re-uploads when it lags behind.
    uint64_t sceneVersion_ = 0;
    // Bumped whenever a buffer a descriptor points at is replaced.
    uint64_t descriptorGeneration_ = 0;

    // CPU copies of the per-frame data, built once per version.
    uint64_t builtVersion_ = ~0ull;
    std::vector<InstanceGPU> instanceData_;
    std::vector<RtBvhNode> tlasNodes_;
    std::vector<VkAccelerationStructureInstanceKHR> asInstanceData_;

    FrameResources frames_[MAX_FRAMES_IN_FLIGHT];
    VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    // Bound where a real buffer does not exist yet, so every set is complete.
    AllocatedBuffer placeholder_{};

    VkDeviceSize scratchAlignment_ = 256;
    PFN_vkGetBufferDeviceAddress pfnGetBufferDeviceAddress_ = nullptr;
    PFN_vkCreateAccelerationStructureKHR pfnCreateAs_ = nullptr;
    PFN_vkDestroyAccelerationStructureKHR pfnDestroyAs_ = nullptr;
    PFN_vkGetAccelerationStructureBuildSizesKHR pfnGetAsBuildSizes_ = nullptr;
    PFN_vkCmdBuildAccelerationStructuresKHR pfnCmdBuildAs_ = nullptr;
    PFN_vkGetAccelerationStructureDeviceAddressKHR pfnGetAsAddress_ = nullptr;
};

}  // namespace wowee::rendering
