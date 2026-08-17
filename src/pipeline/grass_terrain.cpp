#include "pipeline/grass_terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "core/coordinates.hpp"
#include "pipeline/adt_alpha.hpp"
#include "pipeline/terrain_mesh.hpp"

namespace wowee {
namespace pipeline {

namespace {

float clamp01(float v) {
    return std::clamp(v, 0.0f, 1.0f);
}

/// Bilinear sample of a decoded 64x64 alpha map, returned as 0..1.
///
/// Bilinear rather than nearest because this is what keeps suitability
/// continuous. Point-sampling would step in 1/64ths of a chunk - about half a
/// yard - and the grass would end along a straight line every time two
/// textures met.
float sampleAlpha(const std::vector<uint8_t>& alpha, float u, float v) {
    if (alpha.size() < ALPHA_MAP_SIZE) return 0.0f;

    const float fx = clamp01(u) * static_cast<float>(ALPHA_MAP_DIM - 1);
    const float fy = clamp01(v) * static_cast<float>(ALPHA_MAP_DIM - 1);
    const auto x0 = static_cast<size_t>(fx);
    const auto y0 = static_cast<size_t>(fy);
    const size_t x1 = std::min(x0 + 1, ALPHA_MAP_DIM - 1);
    const size_t y1 = std::min(y0 + 1, ALPHA_MAP_DIM - 1);
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);

    const float a00 = static_cast<float>(alpha[y0 * ALPHA_MAP_DIM + x0]);
    const float a10 = static_cast<float>(alpha[y0 * ALPHA_MAP_DIM + x1]);
    const float a01 = static_cast<float>(alpha[y1 * ALPHA_MAP_DIM + x0]);
    const float a11 = static_cast<float>(alpha[y1 * ALPHA_MAP_DIM + x1]);

    const float top = a00 + (a10 - a00) * tx;
    const float bottom = a01 + (a11 - a01) * tx;
    return (top + (bottom - top) * ty) / 255.0f;
}

/// Slope at (u, v) from the chunk's compressed normals, as 1 - |normal.z|.
///
/// The normals are the terrain's own, so this agrees with what the ground
/// looks like rather than with a height difference measured over some chosen
/// distance. Z is up.
float sampleSlope(const MapChunk& chunk, float fracX, float fracY) {
    // The 9x9 outer grid, in the interleaved 145-vertex layout. fracX indexes
    // the grid's x and fracY its y, the same pairing chunkSurfacePoint uses.
    constexpr int kRowStride = 17;
    const int x = std::clamp(static_cast<int>(std::lround(fracX)), 0, 8);
    const int y = std::clamp(static_cast<int>(std::lround(fracY)), 0, 8);
    const size_t idx = static_cast<size_t>(y * kRowStride + x);
    if (idx * 3 + 2 >= chunk.normals.size()) return 0.0f;

    // int8 normals are stored X, Y, Z over [-127, 127].
    const float nz = static_cast<float>(chunk.normals[idx * 3 + 2]) / 127.0f;
    return clamp01(1.0f - std::fabs(nz));
}

/// Height at (fracX, fracY), from the sampler the terrain mesh is built with.
///
/// Not a bilinear interpolation of the four outer corners. The mesh fans four
/// triangles from each quad's centre vertex, and MCVT puts that centre
/// wherever the artist needed it - commonly a yard or two off the plane of its
/// corners on a hillside. Interpolating the corners answers below the ground
/// that is drawn, and a blade is a third of a yard tall, so the entire field
/// ends up buried. terrain_manager.cpp carries the same warning, from the time
/// the player sank through slopes for the same reason.
float sampleHeight(const MapChunk& chunk, float fracX, float fracY) {
    if (!chunk.heightMap.isLoaded()) return chunk.position[2];
    constexpr float kUnitSize = core::coords::TILE_SIZE / 16.0f / GRASS_CHUNK_QUADS;
    return TerrainMeshGenerator::chunkSurfacePoint(chunk.position, chunk.heightMap,
                                                   fracX, fracY, kUnitSize).z;
}

} // namespace

bool ChunkGrassContext::build(const MapChunk& chunk, const GroundEffectDensityFn& densityFor) {
    layerCount = std::min<size_t>(chunk.layers.size(), 4);
    bool any = false;
    for (size_t i = 0; i < layerCount; ++i) {
        const auto& layer = chunk.layers[i];

        // The layer's effect id, and nothing else. An effect id of zero is
        // the map saying no vegetation here - roads and bare dirt carry
        // exactly that. There used to be a fallback through layer.textureId,
        // copied from the clutter placer; but textureId is an index into the
        // tile's own texture list, a small integer, and looking it up in a
        // table keyed by DBC ground-effect ids collides with whatever row
        // happens to share the number. Roads sprouted grass chunk by chunk
        // depending on where their texture sat in each tile's list, and the
        // clutter placer only survives the same fallback because it also
        // filters road textures by name. Elwynn's real grass resolves through
        // its effect id directly.
        effectId[i] = layer.effectId;
        const uint32_t density = (effectId[i] != 0 && densityFor) ? densityFor(effectId[i]) : 0;
        grows[i] = density != 0;
        any = any || grows[i];
        if (i == 0) continue;  // layer 0 is the base and carries no alpha map
        // Unset texels read as zero coverage: a layer whose data runs out is
        // not covering the rest of the chunk, it simply stopped.
        if (!decodeLayerAlpha(chunk, i, alpha[i], 0)) alpha[i].clear();
    }
    growsAnything = any;
    return any;
}

GrassSuitability evaluateGrass(const ChunkGrassContext& context, const MapChunk& chunk,
                               float fracX, float fracY) {
    GrassSuitability out;
    if (!context.growsAnything) return out;  // whole chunk, before any sampling

    fracX = std::clamp(fracX, 0.0f, GRASS_CHUNK_QUADS);
    fracY = std::clamp(fracY, 0.0f, GRASS_CHUNK_QUADS);

    out.slope = sampleSlope(chunk, fracX, fracY);
    out.rootHeight = sampleHeight(chunk, fracX, fracY);

    if (context.layerCount == 0) return out;

    // A hole is a cave mouth or a doorway cut through the terrain. There is no
    // ground there at all, so nothing grows regardless of what the layers say.
    // Read the quad the way isHoleAt does, so the two agree about which quads
    // are there.
    const int quadX = std::clamp(static_cast<int>(std::floor(fracX)), 0, 7);
    const int quadY = std::clamp(static_cast<int>(std::floor(fracY)), 0, 7);
    if (chunk.isHole(quadY, quadX) ) return out;

    // The alpha maps are 64x64 across the chunk, indexed the way the clutter
    // scatterer indexes them.
    const float u = fracX / GRASS_CHUNK_QUADS;
    const float v = fracY / GRASS_CHUNK_QUADS;

    // Layer 0 is the base and covers whatever the layers above it do not.
    // Layers 1..3 carry alpha maps, and their weights are taken off the base
    // in order, which is how the terrain shader composites them.
    std::array<float, 4> weight{};
    float remaining = 1.0f;
    for (size_t i = 1; i < context.layerCount; ++i) {
        if (context.alpha[i].empty()) continue;
        const float a = clamp01(sampleAlpha(context.alpha[i], u, v));
        weight[i] = remaining * a;
        remaining -= weight[i];
    }
    weight[0] = std::max(remaining, 0.0f);

    // The ground's colour is the same blend the terrain shader draws, so the
    // grass sits on the colour it is tinted by. All layers count here - the
    // ground under a blade is what it is whether or not that layer grows.
    if (context.hasLayerColors) {
        glm::vec3 ground(0.0f);
        for (size_t i = 0; i < context.layerCount; ++i) {
            ground += context.layerColor[i] * weight[i];
        }
        out.groundColor = ground;
        out.hasGroundColor = true;
    }

    // Each layer contributes its own weight if its ground effect grows
    // anything. Weighted rather than thresholded, so a texel that is half
    // grass and half road is half suitable instead of one or the other.
    float best = 0.0f;
    for (size_t i = 0; i < context.layerCount; ++i) {
        if (weight[i] <= 0.0f || !context.grows[i]) continue;
        out.suitability += weight[i];
        if (weight[i] > best) {
            best = weight[i];
            out.effectId = context.effectId[i];
        }
    }
    out.suitability = clamp01(out.suitability);

    // Steep ground holds less, and holds none at all past the point where it
    // reads as a cliff face.
    if (out.slope > GRASS_SLOPE_FULL) {
        const float t = (out.slope - GRASS_SLOPE_FULL) / (GRASS_SLOPE_NONE - GRASS_SLOPE_FULL);
        out.suitability *= clamp01(1.0f - t);
    }
    if (out.suitability <= 0.0f) out.effectId = 0;

    return out;
}

GrassSuitability evaluateGrass(const MapChunk& chunk, float fracX, float fracY,
                               const GroundEffectDensityFn& densityFor) {
    ChunkGrassContext context;
    context.build(chunk, densityFor);
    return evaluateGrass(context, chunk, fracX, fracY);
}

} // namespace pipeline
} // namespace wowee
