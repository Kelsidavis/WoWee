#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

// Clearings around the built and the placed.
//
// Roads and paths ease grass out through the terrain data (grass_terrain's
// verge). Buildings and props are placements, not paint, so the terrain
// cannot see them: the renderers that own the instances report their
// footprints here, and the generator asks how wild each blade's spot is.
// Near a wall or a cartwheel the answer is 0 and grass grows short and
// sparse; a few yards out it is 1 and the meadow is its own again.

namespace wowee {
namespace pipeline {

/// One thing grass gives way to, as a 2D footprint. A point is a degenerate
/// box. Distances are yards in the same world space blades are generated in.
struct GrassClearingSource {
    float minX = 0.0f;
    float minY = 0.0f;
    float maxX = 0.0f;
    float maxY = 0.0f;
    /// Fully cleared out to this far beyond the footprint.
    float clearing = 0.0f;
    /// And easing back to wild over this much further.
    float ease = 4.0f;
};

/// The sources for one generation window, bucketed on a coarse grid so a
/// million blade samples pay for the two or three sources near them rather
/// than for every fence post in the window.
class GrassClearingField {
public:
    /// Replace the field. Sources wholly outside the window are dropped.
    void build(std::vector<GrassClearingSource> sources,
               float minX, float minY, float maxX, float maxY);
    void clear();

    /// 1 in open country, easing to 0 at (and inside) any source's cleared
    /// ring. The minimum over sources: standing between a house and a fence
    /// is as tame as the tamer of the two says.
    [[nodiscard]] float wildness(float x, float y) const;

    [[nodiscard]] size_t sourceCount() const { return sources_.size(); }

private:
    std::vector<GrassClearingSource> sources_;
    std::vector<std::vector<uint32_t>> cells_;
    float originX_ = 0.0f;
    float originY_ = 0.0f;
    float cellSize_ = 16.0f;
    int cols_ = 0;
    int rows_ = 0;
};

} // namespace pipeline
} // namespace wowee
