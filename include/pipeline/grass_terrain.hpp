#pragma once

#include <cstdint>
#include <functional>

#include <glm/glm.hpp>

#include "pipeline/adt_loader.hpp"

// Whether grass grows at a point on the terrain, and what kind.
//
// The answer is already in the ADT and must not be re-derived: each texture
// layer carries a ground-effect id, and an effect with no density is Blizzard
// saying "no vegetation here". Roads, cliffs and bare dirt already carry that.
// A classifier of our own would disagree with the map, and the map is right.
//
// The density lookup arrives as a callback rather than a TerrainManager
// reference: the table lives in the rendering layer, and this has no business
// depending on it or on a GPU to be tested.

namespace wowee {
namespace pipeline {

/// What the terrain says about one point.
struct GrassSuitability {
    /// 0 = nothing grows here, 1 = fully vegetated. Continuous, because layers
    /// blend continuously and a hard step would draw a visible seam along
    /// every texture boundary (spec §4).
    float suitability = 0.0f;
    /// The ground effect contributing most of that, or 0 when none does. What
    /// Phase 4 turns into a vegetation profile.
    uint32_t effectId = 0;
    /// 0 flat, 1 vertical. From the chunk's own normals.
    ///
    /// Only meaningful when suitability is above zero. A chunk where nothing
    /// grows answers before sampling anything, because that is most of a road
    /// or a cliff and paying for height and alpha there was most of the cost
    /// of a rebuild.
    float slope = 0.0f;
    /// World Z at the sample point, on the surface the terrain mesh draws.
    /// Only meaningful when suitability is above zero, as above.
    float rootHeight = 0.0f;
    /// The tones the ground under this point is painted in, blended by the
    /// same alpha weights the terrain shader composites with. The terrain
    /// textures depict each region's own foliage, so grass grown from these
    /// wears the region's colours without anything naming a region.
    /// Valid only when hasGroundColor.
    glm::vec3 groundShadow{0.0f};
    glm::vec3 groundHighlight{0.0f};
    bool hasGroundColor = false;
};

/// Density for a ground-effect id; 0 (or an unknown id) means no vegetation.
using GroundEffectDensityFn = std::function<uint32_t(uint32_t effectId)>;

/// Everything about a chunk that does not change between samples.
///
/// Built once per chunk and reused for every point in it. Without this,
/// sampling decoded the chunk's alpha maps again for every blade considered -
/// four kilobytes per layer per sample, which at fifty thousand candidates is
/// hundreds of megabytes of the same work.
struct ChunkGrassContext {
    /// Decoded alpha per layer; empty for layers that carry none.
    std::vector<uint8_t> alpha[4];
    /// Whether each layer's ground effect grows anything.
    bool grows[4] = {};
    uint32_t effectId[4] = {};
    size_t layerCount = 0;
    /// False when no layer's ground effect grows anything, which makes every
    /// sample in the chunk zero without looking at alpha, slope or height.
    /// Roads, rock and water are whole chunks of this.
    bool growsAnything = false;

    /// Each layer's ground texture tones, filled by the caller after build() -
    /// the texture names live on the tile, which pipeline code never sees.
    /// Left unset, blades keep their profile colours unmixed.
    glm::vec3 layerShadow[4]{};
    glm::vec3 layerHighlight[4]{};
    bool hasLayerColors = false;

    /// Returns false when nothing in this chunk grows anything, so a caller
    /// can skip it whole.
    bool build(const MapChunk& chunk, const GroundEffectDensityFn& densityFor);

    /// Stop a layer growing anything, whatever its ground effect says.
    ///
    /// For made surfaces. The caller applies it because the test is on the
    /// texture's name and the names live on the tile, which pipeline code
    /// never sees. Recomputes whether the chunk grows anything at all.
    void suppressLayer(size_t layerIdx);
};

/// Sample using a context already built for this chunk.
GrassSuitability evaluateGrass(const ChunkGrassContext& context, const MapChunk& chunk,
                               float fracX, float fracY);

/// Sample a chunk at (fracX, fracY), each in **0..8 grid units** across it.
///
/// These are the fractions `TerrainManager::findChunkAt` hands back, and the
/// same ones `chunkSurfacePoint` and `isHoleAt` take. Normalised 0..1 would be
/// tidier and is what this took first; every caller then had to remember to
/// divide, one did not, and every sample landed clamped at a chunk corner -
/// which reads as "the whole chunk is uniformly suitable" rather than as a
/// units mistake.
///
/// Builds a context for every call, so it is for single samples and tests. Use
/// the overload above when walking many points across one chunk.
GrassSuitability evaluateGrass(const MapChunk& chunk, float fracX, float fracY,
                               const GroundEffectDensityFn& densityFor);

/// One chunk is 8 grid quads across.
inline constexpr float GRASS_CHUNK_QUADS = 8.0f;

/// Above this the ground is too steep to hold grass, and below it nothing is
/// taken away. Between the two, suitability falls off smoothly rather than
/// stopping along a contour line.
inline constexpr float GRASS_SLOPE_FULL = 0.30f;  // ~25 degrees
inline constexpr float GRASS_SLOPE_NONE = 0.55f;  // ~56 degrees

} // namespace pipeline
} // namespace wowee
