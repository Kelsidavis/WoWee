// The hierarchy the software ray tracer walks.
//
// Closest hits are checked against a brute-force scan of every triangle, which
// is slow and obviously correct. The structural checks are what the shader
// relies on without re-checking: every triangle reachable exactly once, and
// every box enclosing what is under it.
#include <catch_amalgamated.hpp>

#include <cmath>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "rendering/collision_geometry.hpp"
#include "rendering/rt_bvh.hpp"
#include "rendering/rt_range_allocator.hpp"

using namespace wowee::rendering;

namespace {

std::vector<RtTriangle> randomSoup(uint32_t count, uint32_t seed, float spread, float size) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> pos(-spread, spread);
    std::uniform_real_distribution<float> off(-size, size);
    std::vector<RtTriangle> tris(count);
    for (auto& t : tris) {
        const glm::vec3 c(pos(rng), pos(rng), pos(rng));
        t.v0 = glm::vec4(c + glm::vec3(off(rng), off(rng), off(rng)), 0.0f);
        t.v1 = glm::vec4(c + glm::vec3(off(rng), off(rng), off(rng)), 0.0f);
        t.v2 = glm::vec4(c + glm::vec3(off(rng), off(rng), off(rng)), 0.0f);
    }
    return tris;
}

float bruteForce(const std::vector<RtTriangle>& tris, const glm::vec3& o, const glm::vec3& d) {
    float best = -1.0f;
    for (const auto& t : tris) {
        const float h = rayTriangleIntersect(o, d, glm::vec3(t.v0), glm::vec3(t.v1), glm::vec3(t.v2));
        if (h > 0.0f && (best < 0.0f || h < best)) best = h;
    }
    return best;
}

bool contains(const RtBvhNode& outer, const glm::vec3& p) {
    constexpr float eps = 1e-4f;
    return glm::all(glm::greaterThanEqual(p, outer.bmin - eps)) &&
           glm::all(glm::lessThanEqual(p, outer.bmax + eps));
}

void checkStructure(const RtBvh& bvh, const std::vector<RtTriangle>& tris) {
    std::vector<int> seen(tris.size(), 0);
    std::vector<uint32_t> stack{0};
    while (!stack.empty()) {
        const uint32_t idx = stack.back();
        stack.pop_back();
        const RtBvhNode& n = bvh.nodes[idx];
        if (n.count > 0) {
            for (uint32_t i = n.leftOrFirst; i < n.leftOrFirst + n.count; ++i) {
                REQUIRE(i < tris.size());
                ++seen[i];
                CHECK(contains(n, glm::vec3(tris[i].v0)));
                CHECK(contains(n, glm::vec3(tris[i].v1)));
                CHECK(contains(n, glm::vec3(tris[i].v2)));
            }
        } else {
            REQUIRE(n.leftOrFirst + 1 < bvh.nodes.size());
            for (uint32_t c : {n.leftOrFirst, n.leftOrFirst + 1}) {
                CHECK(contains(n, bvh.nodes[c].bmin));
                CHECK(contains(n, bvh.nodes[c].bmax));
                stack.push_back(c);
            }
        }
    }
    for (int s : seen) CHECK(s == 1);
}

}  // namespace

TEST_CASE("RT BVH closest hit matches a brute-force scan", "[rt_bvh]") {
    for (uint32_t seed : {1u, 2u, 3u}) {
        const auto original = randomSoup(2000, seed, 50.0f, 3.0f);
        auto tris = original;
        const RtBvh bvh = buildRtTriangleBvh(tris);
        checkStructure(bvh, tris);

        std::mt19937 rng(seed * 77);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f);
        int hits = 0;
        for (int r = 0; r < 2000; ++r) {
            const glm::vec3 o(u(rng) * 60.0f, u(rng) * 60.0f, u(rng) * 60.0f);
            glm::vec3 d(u(rng), u(rng), u(rng));
            if (glm::length(d) < 1e-3f) continue;
            d = glm::normalize(d);
            const float expected = bruteForce(original, o, d);
            const RtHit got = traceRtClosest(bvh.nodes, tris, o, d, 1e30f);
            if (expected < 0.0f) {
                CHECK(got.t < 0.0f);
            } else {
                ++hits;
                REQUIRE(got.t > 0.0f);
                CHECK(std::abs(got.t - expected) <= 1e-3f * std::max(1.0f, expected));
            }
        }
        CHECK(hits > 100);  // the rays have to actually exercise the hit path
    }
}

TEST_CASE("RT BVH honours tMax", "[rt_bvh]") {
    auto tris = randomSoup(500, 9, 20.0f, 2.0f);
    const auto original = tris;
    const RtBvh bvh = buildRtTriangleBvh(tris);
    // Aim at a triangle rather than a fixed direction: the soup differs between
    // standard libraries, so a hand-picked ray can miss everything on one of them.
    const glm::vec3 o(-40.0f, 0.3f, 0.1f);
    const glm::vec3 target =
        (glm::vec3(original[0].v0) + glm::vec3(original[0].v1) + glm::vec3(original[0].v2)) / 3.0f;
    const glm::vec3 d = glm::normalize(target - o);
    const float full = bruteForce(original, o, d);
    REQUIRE(full > 0.0f);
    CHECK(traceRtClosest(bvh.nodes, tris, o, d, full * 0.5f).t < 0.0f);
    CHECK(traceRtClosest(bvh.nodes, tris, o, d, full * 1.01f).t > 0.0f);
}

TEST_CASE("RT BVH splits coincident triangles instead of one huge leaf", "[rt_bvh]") {
    // Stacked copies of one triangle - the case where no SAH split separates
    // anything. Decals and double-sided cards in the models look like this.
    std::vector<RtTriangle> tris(200);
    for (auto& t : tris) {
        t.v0 = {0, 0, 0, 0};
        t.v1 = {1, 0, 0, 0};
        t.v2 = {0, 1, 0, 0};
    }
    const RtBvh bvh = buildRtTriangleBvh(tris, 4);
    checkStructure(bvh, tris);
    for (const auto& n : bvh.nodes) CHECK(n.count <= 16);
    CHECK(traceRtClosest(bvh.nodes, tris, {0.2f, 0.2f, 5.0f}, {0, 0, -1}, 1e30f).t ==
          Catch::Approx(5.0f));
}

TEST_CASE("RT BVH of nothing traces nothing", "[rt_bvh]") {
    std::vector<RtTriangle> tris;
    const RtBvh bvh = buildRtTriangleBvh(tris);
    CHECK(bvh.nodes.empty());
    CHECK(traceRtClosest(bvh.nodes, tris, {0, 0, 0}, {0, 0, 1}, 1e30f).t < 0.0f);
}

TEST_CASE("RT surface packing round-trips to 8 bits", "[rt_bvh]") {
    const glm::vec4 v = unpackRtSurface(packRtSurface({0.25f, 0.5f, 1.0f}, 0.3f));
    CHECK(v.r == Catch::Approx(0.25f).margin(1.0 / 255));
    CHECK(v.g == Catch::Approx(0.5f).margin(1.0 / 255));
    CHECK(v.b == Catch::Approx(1.0f).margin(1.0 / 255));
    CHECK(v.a == Catch::Approx(0.3f).margin(1.0 / 255));
}


TEST_CASE("RT range allocator reuses and coalesces freed ranges", "[rt_bvh]") {
    RtRangeAllocator a(100);
    const uint32_t x = a.allocate(40);
    const uint32_t y = a.allocate(40);
    CHECK(x == 0);
    CHECK(y == 40);
    CHECK(a.allocate(30) == RtRangeAllocator::kFailed);
    CHECK(a.used() == 80);

    a.release(x, 40);
    a.release(y, 40);
    CHECK(a.used() == 0);
    CHECK(a.allocate(100) == 0);  // only possible once all three ranges merged
    a.release(0, 100);

    const uint32_t z = a.allocate(90);
    a.grow(200);
    CHECK(a.capacity() == 200);
    CHECK(a.used() == 90);
    CHECK(a.allocate(110) == 90);  // old tail and new space are one range
    a.release(z, 90);
    CHECK(a.allocate(0) == RtRangeAllocator::kFailed);
}
