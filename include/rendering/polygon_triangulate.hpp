#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace wowee {
namespace rendering {

/// Triangle indices for a simple polygon, by ear clipping.
///
/// Quest POI areas are the reason this exists. A blob's outline comes off the
/// wire as a ring of points and is often not convex - an area that follows a
/// valley or wraps a lake bends back on itself - and ImGui's polygon fill is
/// AddConvexPolyFilled, which on a concave ring paints outside it.
///
/// The ring may be given in either winding. Returns index triples into the
/// input, three at a time, and an empty vector for fewer than three points or
/// a ring it cannot reduce (self-intersecting, or every remaining vertex
/// reflex - which a degenerate ring can be).
std::vector<uint16_t> triangulateSimplePolygon(const std::vector<glm::vec2>& points);

/// Twice the signed area of a ring; positive when it winds counter-clockwise.
float signedAreaX2(const std::vector<glm::vec2>& points);

} // namespace rendering
} // namespace wowee
