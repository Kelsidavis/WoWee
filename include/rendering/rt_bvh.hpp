#pragma once

/// Bounding volume hierarchy shared by the ray traced lighting backends.
///
/// One builder serves both levels of the software tracer: a bottom level over
/// the triangles of one mesh, and a top level over the world-space bounds of
/// every instance. The node layout is the GPU layout (std430, 32 bytes), so a
/// built hierarchy is uploaded as-is and `rt_trace_sw.glsli` walks the same
/// structure `traceClosest` below walks on the CPU. That function is the
/// reference the shader is checked against, not something the renderer calls.
///
/// The hardware backend does not use these nodes; the driver builds its own
/// acceleration structures from the same triangle pool.

#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

namespace wowee::rendering {

struct RtAabb {
    glm::vec3 bmin{ 1e30f};
    glm::vec3 bmax{-1e30f};

    void grow(const glm::vec3& p) {
        bmin = glm::min(bmin, p);
        bmax = glm::max(bmax, p);
    }
    void grow(const RtAabb& b) {
        bmin = glm::min(bmin, b.bmin);
        bmax = glm::max(bmax, b.bmax);
    }
    [[nodiscard]] bool valid() const {
        return bmin.x <= bmax.x && bmin.y <= bmax.y && bmin.z <= bmax.z;
    }
    [[nodiscard]] float halfArea() const {
        if (!valid()) return 0.0f;
        const glm::vec3 e = bmax - bmin;
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }
};

/// count == 0: interior node, children at `leftOrFirst` and `leftOrFirst + 1`.
/// count  > 0: leaf covering primitives [leftOrFirst, leftOrFirst + count) of
///             the reordered primitive array.
struct RtBvhNode {
    glm::vec3 bmin;
    uint32_t leftOrFirst;
    glm::vec3 bmax;
    uint32_t count;
};
static_assert(sizeof(RtBvhNode) == 32, "RtBvhNode is uploaded as std430");

/// One triangle as the tracers store it. Three vec4 so the pool can double as
/// a non-indexed R32G32B32 vertex stream (stride 16) for hardware BLAS builds.
/// v0.w carries the packed surface colour (RGBA8, alpha = opacity); the other
/// two w components are unused.
struct RtTriangle {
    glm::vec4 v0;
    glm::vec4 v1;
    glm::vec4 v2;
};
static_assert(sizeof(RtTriangle) == 48, "RtTriangle is uploaded as std430");

struct RtBvh {
    std::vector<RtBvhNode> nodes;
    /// order[i] is the input index of the primitive that sorted slot i holds.
    std::vector<uint32_t> order;
};

/// Binned-SAH build over arbitrary primitive bounds. Deterministic for a given
/// input. An empty input yields an empty hierarchy.
RtBvh buildRtBvh(std::span<const RtAabb> bounds, uint32_t maxLeafSize);

/// Build the bottom-level hierarchy of one mesh and reorder `tris` in place so
/// the leaves index it directly.
RtBvh buildRtTriangleBvh(std::vector<RtTriangle>& tris, uint32_t maxLeafSize = 4);

/// Pack a linear colour and opacity the way RtTriangle::v0.w stores them.
float packRtSurface(const glm::vec3& albedo, float opacity);
glm::vec4 unpackRtSurface(float packed);

struct RtHit {
    float t = -1.0f;
    uint32_t prim = 0;  // sorted slot of the triangle that was hit
};

/// Reference closest-hit traversal over one bottom-level hierarchy.
RtHit traceRtClosest(std::span<const RtBvhNode> nodes, std::span<const RtTriangle> tris,
                     const glm::vec3& origin, const glm::vec3& dir, float tMax);

}  // namespace wowee::rendering
