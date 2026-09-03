#include "rendering/polygon_triangulate.hpp"

#include <algorithm>

namespace wowee {
namespace rendering {

namespace {

/// Whether p is inside triangle abc, edges included.
bool insideTriangle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c,
                    const glm::vec2& p) {
    const auto cross = [](const glm::vec2& u, const glm::vec2& v, const glm::vec2& w) {
        return (v.x - u.x) * (w.y - u.y) - (v.y - u.y) * (w.x - u.x);
    };
    const float d1 = cross(a, b, p);
    const float d2 = cross(b, c, p);
    const float d3 = cross(c, a, p);
    const bool anyNegative = d1 < 0.0f || d2 < 0.0f || d3 < 0.0f;
    const bool anyPositive = d1 > 0.0f || d2 > 0.0f || d3 > 0.0f;
    // Inside, or on an edge: a point on one edge has one zero and the other
    // two the same sign.
    return !(anyNegative && anyPositive);
}

}  // namespace

float signedAreaX2(const std::vector<glm::vec2>& points) {
    float sum = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        const glm::vec2& a = points[i];
        const glm::vec2& b = points[(i + 1) % points.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum;
}

std::vector<uint16_t> triangulateSimplePolygon(const std::vector<glm::vec2>& points) {
    std::vector<uint16_t> out;
    const size_t n = points.size();
    if (n < 3 || n > 4096) return out;

    // Work counter-clockwise so an ear is a left turn, whichever way the ring
    // was written. The indices stay the caller's either way.
    std::vector<uint16_t> ring(n);
    for (size_t i = 0; i < n; ++i) ring[i] = static_cast<uint16_t>(i);
    if (signedAreaX2(points) < 0.0f) std::reverse(ring.begin(), ring.end());

    out.reserve((n - 2) * 3);
    size_t guard = 0;
    size_t at = 0;
    while (ring.size() > 2) {
        // One full pass with no ear taken means the ring is not simple; stop
        // rather than spin, and hand back the triangles found so far.
        if (guard++ > ring.size() * ring.size() + 8) break;

        const size_t count = ring.size();
        const uint16_t ia = ring[(at + count - 1) % count];
        const uint16_t ib = ring[at % count];
        const uint16_t ic = ring[(at + 1) % count];
        const glm::vec2& a = points[ia];
        const glm::vec2& b = points[ib];
        const glm::vec2& c = points[ic];

        const float turn = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        bool isEar = turn > 0.0f;
        if (isEar) {
            for (uint16_t idx : ring) {
                if (idx == ia || idx == ib || idx == ic) continue;
                if (insideTriangle(a, b, c, points[idx])) { isEar = false; break; }
            }
        }
        if (!isEar) {
            at = (at + 1) % count;
            continue;
        }

        out.push_back(ia);
        out.push_back(ib);
        out.push_back(ic);
        ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(at % count));
        if (!ring.empty()) at %= ring.size();
    }
    return out;
}

} // namespace rendering
} // namespace wowee
