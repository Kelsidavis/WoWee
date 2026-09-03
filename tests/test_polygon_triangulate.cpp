// Ear clipping, against the areas a quest blob is actually shaped like.
//
// The blob outlines come off SMSG_QUEST_POI_QUERY_RESPONSE as a ring of
// points and are regularly concave. ImGui fills polygons with
// AddConvexPolyFilled, which paints outside a concave ring, so the ring is
// cut into triangles first. What has to hold is that the triangles cover the
// ring and nothing else: their areas sum to the ring's own.
#include <catch_amalgamated.hpp>

#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#include "rendering/polygon_triangulate.hpp"

using wowee::rendering::signedAreaX2;
using wowee::rendering::triangulateSimplePolygon;

namespace {

float trianglesArea(const std::vector<glm::vec2>& pts, const std::vector<uint16_t>& idx) {
    float sum = 0.0f;
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        const glm::vec2& a = pts[idx[i]];
        const glm::vec2& b = pts[idx[i + 1]];
        const glm::vec2& c = pts[idx[i + 2]];
        sum += std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) * 0.5f;
    }
    return sum;
}

}  // namespace

TEST_CASE("a square is two triangles covering the square", "[polygon]") {
    const std::vector<glm::vec2> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const auto idx = triangulateSimplePolygon(square);
    REQUIRE(idx.size() == 6);
    CHECK(trianglesArea(square, idx) == Catch::Approx(100.0f));
}

TEST_CASE("winding does not change the answer", "[polygon]") {
    // The wire gives no promise about which way a ring is written.
    const std::vector<glm::vec2> clockwise = {{0, 10}, {10, 10}, {10, 0}, {0, 0}};
    const auto idx = triangulateSimplePolygon(clockwise);
    REQUIRE(idx.size() == 6);
    CHECK(trianglesArea(clockwise, idx) == Catch::Approx(100.0f));
}

TEST_CASE("a concave ring is covered and not overfilled", "[polygon]") {
    // An L: the case AddConvexPolyFilled gets wrong, painting the notch in.
    const std::vector<glm::vec2> ell = {
        {0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10},
    };
    const auto idx = triangulateSimplePolygon(ell);
    REQUIRE(idx.size() == (ell.size() - 2) * 3);
    // 10x4 plus 4x6: the notch is not part of it, and a convex fill would add
    // the missing 6x6 corner.
    CHECK(trianglesArea(ell, idx) == Catch::Approx(64.0f));
    CHECK(std::fabs(signedAreaX2(ell)) * 0.5f == Catch::Approx(64.0f));
}

TEST_CASE("a ring with a deep notch still comes out whole", "[polygon]") {
    // A blob that wraps a lake, roughly: two arms and a slot between them.
    const std::vector<glm::vec2> u = {
        {0, 0}, {12, 0}, {12, 10}, {9, 10}, {9, 3}, {3, 3}, {3, 10}, {0, 10},
    };
    const auto idx = triangulateSimplePolygon(u);
    REQUIRE(idx.size() == (u.size() - 2) * 3);
    CHECK(trianglesArea(u, idx) == Catch::Approx(std::fabs(signedAreaX2(u)) * 0.5f));
}

TEST_CASE("too few points make no triangles", "[polygon]") {
    CHECK(triangulateSimplePolygon({}).empty());
    CHECK(triangulateSimplePolygon({{0, 0}, {1, 1}}).empty());
}
