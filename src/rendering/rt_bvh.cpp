#include "rendering/rt_bvh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <numeric>

namespace wowee::rendering {

namespace {

constexpr int kBins = 16;
// Relative cost of one box test against one primitive test. Only the ratio
// matters; it decides when a leaf is cheaper than splitting it again.
constexpr float kTraversalCost = 1.0f;
constexpr float kIntersectCost = 1.0f;

struct WorkItem {
    uint32_t node;
    uint32_t first;
    uint32_t count;
};

RtAabb rangeBounds(std::span<const RtAabb> bounds, std::span<const uint32_t> order,
                   uint32_t first, uint32_t count, RtAabb* centroidBoundsOut) {
    RtAabb b;
    RtAabb cb;
    for (uint32_t i = first; i < first + count; ++i) {
        const RtAabb& pb = bounds[order[i]];
        b.grow(pb);
        cb.grow((pb.bmin + pb.bmax) * 0.5f);
    }
    if (centroidBoundsOut) *centroidBoundsOut = cb;
    return b;
}

}  // namespace

RtBvh buildRtBvh(std::span<const RtAabb> bounds, uint32_t maxLeafSize) {
    RtBvh bvh;
    const uint32_t n = static_cast<uint32_t>(bounds.size());
    if (n == 0) return bvh;
    maxLeafSize = std::max(1u, maxLeafSize);

    bvh.order.resize(n);
    std::iota(bvh.order.begin(), bvh.order.end(), 0u);
    bvh.nodes.reserve(2 * n);
    bvh.nodes.push_back({});

    std::vector<WorkItem> stack;
    stack.push_back({0, 0, n});

    while (!stack.empty()) {
        const WorkItem item = stack.back();
        stack.pop_back();

        RtAabb cb;
        const RtAabb nb = rangeBounds(bounds, bvh.order, item.first, item.count, &cb);
        RtBvhNode& node = bvh.nodes[item.node];
        node.bmin = nb.bmin;
        node.bmax = nb.bmax;

        auto makeLeaf = [&]() {
            RtBvhNode& leaf = bvh.nodes[item.node];
            leaf.leftOrFirst = item.first;
            leaf.count = item.count;
        };

        if (item.count <= maxLeafSize) {
            makeLeaf();
            continue;
        }

        // Pick the axis and bin boundary with the lowest surface area cost.
        float bestCost = 1e30f;
        int bestAxis = -1;
        int bestSplit = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const float lo = cb.bmin[axis];
            const float extent = cb.bmax[axis] - lo;
            if (!(extent > 0.0f)) continue;
            const float scale = kBins / extent;

            std::array<RtAabb, kBins> binBounds{};
            std::array<uint32_t, kBins> binCount{};
            for (uint32_t i = item.first; i < item.first + item.count; ++i) {
                const RtAabb& pb = bounds[bvh.order[i]];
                const float c = (pb.bmin[axis] + pb.bmax[axis]) * 0.5f;
                const int bin = std::min(kBins - 1, static_cast<int>((c - lo) * scale));
                binBounds[bin].grow(pb);
                ++binCount[bin];
            }

            std::array<float, kBins - 1> leftArea{};
            std::array<uint32_t, kBins - 1> leftCount{};
            RtAabb acc;
            uint32_t cnt = 0;
            for (int i = 0; i < kBins - 1; ++i) {
                acc.grow(binBounds[i]);
                cnt += binCount[i];
                leftArea[i] = acc.halfArea();
                leftCount[i] = cnt;
            }
            acc = RtAabb{};
            cnt = 0;
            for (int i = kBins - 1; i > 0; --i) {
                acc.grow(binBounds[i]);
                cnt += binCount[i];
                const uint32_t lc = leftCount[i - 1];
                if (lc == 0 || cnt == 0) continue;
                const float cost = leftArea[i - 1] * lc + acc.halfArea() * cnt;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = i;
                }
            }
        }

        const float leafCost = kIntersectCost * item.count;
        const float parentArea = nb.halfArea();
        const float splitCost = parentArea > 0.0f
            ? kTraversalCost + kIntersectCost * bestCost / parentArea
            : 1e30f;

        uint32_t mid = 0;
        if (bestAxis >= 0 && splitCost < leafCost) {
            const float lo = cb.bmin[bestAxis];
            const float scale = kBins / (cb.bmax[bestAxis] - lo);
            auto it = std::partition(
                bvh.order.begin() + item.first, bvh.order.begin() + item.first + item.count,
                [&](uint32_t idx) {
                    const RtAabb& pb = bounds[idx];
                    const float c = (pb.bmin[bestAxis] + pb.bmax[bestAxis]) * 0.5f;
                    return std::min(kBins - 1, static_cast<int>((c - lo) * scale)) < bestSplit;
                });
            mid = static_cast<uint32_t>(it - bvh.order.begin());
        } else if (item.count > maxLeafSize * 4) {
            // Coincident centroids, or splitting looks no better than a leaf,
            // but a leaf this large would make every ray through it linear in
            // the mesh. Split down the middle instead.
            mid = item.first + item.count / 2;
        } else {
            makeLeaf();
            continue;
        }

        const uint32_t left = static_cast<uint32_t>(bvh.nodes.size());
        bvh.nodes.push_back({});
        bvh.nodes.push_back({});
        bvh.nodes[item.node].leftOrFirst = left;
        bvh.nodes[item.node].count = 0;
        stack.push_back({left + 1, mid, item.first + item.count - mid});
        stack.push_back({left, item.first, mid - item.first});
    }

    return bvh;
}

RtBvh buildRtTriangleBvh(std::vector<RtTriangle>& tris, uint32_t maxLeafSize) {
    std::vector<RtAabb> bounds(tris.size());
    for (size_t i = 0; i < tris.size(); ++i) {
        bounds[i].grow(glm::vec3(tris[i].v0));
        bounds[i].grow(glm::vec3(tris[i].v1));
        bounds[i].grow(glm::vec3(tris[i].v2));
    }
    RtBvh bvh = buildRtBvh(bounds, maxLeafSize);
    std::vector<RtTriangle> sorted(tris.size());
    for (size_t i = 0; i < bvh.order.size(); ++i) sorted[i] = tris[bvh.order[i]];
    tris.swap(sorted);
    return bvh;
}

float packRtSurface(const glm::vec3& albedo, float opacity) {
    auto q = [](float v) {
        return static_cast<uint32_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
    };
    const uint32_t bits = q(albedo.r) | (q(albedo.g) << 8) | (q(albedo.b) << 16) |
                          (q(opacity) << 24);
    return std::bit_cast<float>(bits);
}

glm::vec4 unpackRtSurface(float packed) {
    const uint32_t bits = std::bit_cast<uint32_t>(packed);
    return glm::vec4(bits & 0xFFu, (bits >> 8) & 0xFFu, (bits >> 16) & 0xFFu, bits >> 24) /
           255.0f;
}

namespace {

// Slab test. Returns the entry distance, or a value past tMax for a miss.
float intersectAabb(const glm::vec3& bmin, const glm::vec3& bmax, const glm::vec3& origin,
                    const glm::vec3& invDir, float tMax) {
    const glm::vec3 t0 = (bmin - origin) * invDir;
    const glm::vec3 t1 = (bmax - origin) * invDir;
    const glm::vec3 tn = glm::min(t0, t1);
    const glm::vec3 tf = glm::max(t0, t1);
    const float enter = std::max(std::max(tn.x, tn.y), std::max(tn.z, 0.0f));
    const float exit = std::min(std::min(tf.x, tf.y), std::min(tf.z, tMax));
    return enter <= exit ? enter : 1e30f;
}

float intersectTriangle(const RtTriangle& tri, const glm::vec3& origin, const glm::vec3& dir) {
    const glm::vec3 v0(tri.v0);
    const glm::vec3 e1 = glm::vec3(tri.v1) - v0;
    const glm::vec3 e2 = glm::vec3(tri.v2) - v0;
    const glm::vec3 h = glm::cross(dir, e2);
    const float a = glm::dot(e1, h);
    if (std::abs(a) < 1e-12f) return -1.0f;
    const float f = 1.0f / a;
    const glm::vec3 s = origin - v0;
    const float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return -1.0f;
    const glm::vec3 q = glm::cross(s, e1);
    const float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return -1.0f;
    return f * glm::dot(e2, q);
}

}  // namespace

RtHit traceRtClosest(std::span<const RtBvhNode> nodes, std::span<const RtTriangle> tris,
                     const glm::vec3& origin, const glm::vec3& dir, float tMax) {
    RtHit hit;
    if (nodes.empty()) return hit;
    const glm::vec3 invDir = 1.0f / dir;  // IEEE infinities make the slab test work

    uint32_t stack[64];
    int sp = 0;
    uint32_t cur = 0;
    if (intersectAabb(nodes[0].bmin, nodes[0].bmax, origin, invDir, tMax) > tMax) return hit;

    for (;;) {
        const RtBvhNode& node = nodes[cur];
        if (node.count > 0) {
            for (uint32_t i = node.leftOrFirst; i < node.leftOrFirst + node.count; ++i) {
                const float t = intersectTriangle(tris[i], origin, dir);
                if (t > 0.0f && t < tMax) {
                    tMax = t;
                    hit.t = t;
                    hit.prim = i;
                }
            }
        } else {
            uint32_t a = node.leftOrFirst;
            uint32_t b = a + 1;
            float ta = intersectAabb(nodes[a].bmin, nodes[a].bmax, origin, invDir, tMax);
            float tb = intersectAabb(nodes[b].bmin, nodes[b].bmax, origin, invDir, tMax);
            if (ta > tb) {
                std::swap(ta, tb);
                std::swap(a, b);
            }
            if (ta <= tMax) {
                if (tb <= tMax && sp < 64) stack[sp++] = b;
                cur = a;
                continue;
            }
        }
        // Pop the next subtree that can still hold a closer hit. Its box was
        // tested against the tMax of the moment it was pushed; re-test it now.
        bool found = false;
        while (sp > 0) {
            cur = stack[--sp];
            if (intersectAabb(nodes[cur].bmin, nodes[cur].bmax, origin, invDir, tMax) <= tMax) {
                found = true;
                break;
            }
        }
        if (!found) break;
    }
    return hit;
}

}  // namespace wowee::rendering
