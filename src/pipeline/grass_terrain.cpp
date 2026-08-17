#include "pipeline/grass_terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "pipeline/adt_alpha.hpp"

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
float sampleSlope(const MapChunk& chunk, float u, float v) {
    // The 9x9 outer grid, in the interleaved 145-vertex layout.
    constexpr int kRowStride = 17;
    const int x = std::clamp(static_cast<int>(std::lround(clamp01(u) * 8.0f)), 0, 8);
    const int y = std::clamp(static_cast<int>(std::lround(clamp01(v) * 8.0f)), 0, 8);
    const size_t idx = static_cast<size_t>(y * kRowStride + x);
    if (idx * 3 + 2 >= chunk.normals.size()) return 0.0f;

    // int8 normals are stored X, Y, Z over [-127, 127].
    const float nz = static_cast<float>(chunk.normals[idx * 3 + 2]) / 127.0f;
    return clamp01(1.0f - std::fabs(nz));
}

/// Height at (u, v), bilinear over the 9x9 outer grid.
float sampleHeight(const MapChunk& chunk, float u, float v) {
    if (!chunk.heightMap.isLoaded()) return chunk.position[2];

    const float fx = clamp01(u) * 8.0f;
    const float fy = clamp01(v) * 8.0f;
    const int x0 = std::clamp(static_cast<int>(fx), 0, 8);
    const int y0 = std::clamp(static_cast<int>(fy), 0, 8);
    const int x1 = std::min(x0 + 1, 8);
    const int y1 = std::min(y0 + 1, 8);
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);

    const float h00 = chunk.heightMap.getHeight(x0, y0);
    const float h10 = chunk.heightMap.getHeight(x1, y0);
    const float h01 = chunk.heightMap.getHeight(x0, y1);
    const float h11 = chunk.heightMap.getHeight(x1, y1);

    const float top = h00 + (h10 - h00) * tx;
    const float bottom = h01 + (h11 - h01) * tx;
    return chunk.position[2] + top + (bottom - top) * ty;
}

} // namespace

bool ChunkGrassContext::build(const MapChunk& chunk, const GroundEffectDensityFn& densityFor) {
    layerCount = std::min<size_t>(chunk.layers.size(), 4);
    bool any = false;
    for (size_t i = 0; i < layerCount; ++i) {
        effectId[i] = chunk.layers[i].effectId;
        grows[i] = effectId[i] != 0 && densityFor && densityFor(effectId[i]) != 0;
        any = any || grows[i];
        if (i == 0) continue;  // layer 0 is the base and carries no alpha map
        // Unset texels read as zero coverage: a layer whose data runs out is
        // not covering the rest of the chunk, it simply stopped.
        if (!decodeLayerAlpha(chunk, i, alpha[i], 0)) alpha[i].clear();
    }
    return any;
}

GrassSuitability evaluateGrass(const ChunkGrassContext& context, const MapChunk& chunk,
                               float u, float v) {
    GrassSuitability out;
    u = clamp01(u);
    v = clamp01(v);

    out.slope = sampleSlope(chunk, u, v);
    out.rootHeight = sampleHeight(chunk, u, v);

    if (context.layerCount == 0) return out;

    // A hole is a cave mouth or a doorway cut through the terrain. There is no
    // ground there at all, so nothing grows regardless of what the layers say.
    const int quadX = std::clamp(static_cast<int>(u * 8.0f), 0, 7);
    const int quadY = std::clamp(static_cast<int>(v * 8.0f), 0, 7);
    if (chunk.isHole(quadY, quadX)) return out;

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

GrassSuitability evaluateGrass(const MapChunk& chunk, float u, float v,
                               const GroundEffectDensityFn& densityFor) {
    ChunkGrassContext context;
    context.build(chunk, densityFor);
    return evaluateGrass(context, chunk, u, v);
}

} // namespace pipeline
} // namespace wowee
