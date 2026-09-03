#include "pipeline/grass_terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "core/coordinates.hpp"
#include "pipeline/adt_alpha.hpp"
#include "pipeline/grass_profile.hpp"
#include "pipeline/terrain_mesh.hpp"

namespace wowee {
namespace pipeline {

namespace {

float clamp01(float v) {
    return std::clamp(v, 0.0f, 1.0f);
}

// The bilinear read of an alpha map now lives beside the decoder, as
// pipeline::sampleAlpha - the terrain queries want the same mapping and had
// their own nearest-texel copies of it. Bilinear rather than nearest is what
// keeps suitability continuous: point-sampling steps in 1/64ths of a chunk -
// about half a yard - and the grass would end along a straight line every time
// two textures met.

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

bool ChunkGrassContext::build(const MapChunk& chunk, const GroundEffectDensityFn& densityFor,
                              const LayerTextureNameFn& textureNameFor) {
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
        // Nothing grows out of a road, and only the name says so. The name is
        // remembered as well: the verge easing wants to know a road from
        // ground that merely grows nothing, because meadow gives way toward a
        // path but not toward every patch of forest floor.
        if (textureNameFor && isRoadLikeTexture(textureNameFor(layer.textureId))) {
            roadLike[i] = true;
            grows[i] = false;
            effectId[i] = 0;
        }
        any = any || grows[i];
        if (i == 0) continue;  // layer 0 is the base and carries no alpha map
        // Unset texels read as fully covered, which is what the terrain
        // renderer reads them as. Grass has to agree with the ground that is
        // actually painted, not with a defensible reading of its own: where an
        // RLE run ended early this decoded the road as absent and grew a full
        // field of grass down ground the renderer had already paved.
        if (!decodeLayerAlpha(chunk, i, alpha[i], 255)) alpha[i].clear();
    }
    growsAnything = any;
    return any;
}

void ChunkGrassContext::suppressLayer(size_t layerIdx) {
    if (layerIdx >= layerCount) return;
    grows[layerIdx] = false;
    effectId[layerIdx] = 0;
    growsAnything = false;
    for (size_t i = 0; i < layerCount; ++i) growsAnything = growsAnything || grows[i];
}

GrassSuitability evaluateGrass(const ChunkGrassContext& context, const MapChunk& chunk,
                               float fracX, float fracY) {
    GrassSuitability out;
    if (!context.growsAnything) return out;  // whole chunk, before any sampling

    fracX = std::clamp(fracX, 0.0f, GRASS_CHUNK_QUADS);
    fracY = std::clamp(fracY, 0.0f, GRASS_CHUNK_QUADS);

    out.areaId = chunk.areaId;
    out.slope = sampleSlope(chunk, fracX, fracY);
    out.rootHeight = sampleHeight(chunk, fracX, fracY);
    out.submerged = context.hasWater && out.rootHeight < context.waterHeight;

    if (context.layerCount == 0) return out;

    // A hole is a cave mouth or a doorway cut through the terrain. There is no
    // ground there at all, so nothing grows regardless of what the layers say.
    // Read the quad the way isHoleAt does, so the two agree about which quads
    // are there.
    const int quadX = std::clamp(static_cast<int>(std::floor(fracX)), 0, 7);
    const int quadY = std::clamp(static_cast<int>(std::floor(fracY)), 0, 7);
    if (chunk.isHole(quadY, quadX) ) return out;

    // The map's own "nothing grows here" mask. Tilled farm rows, building
    // footprints and interior floors sit on textures whose effects grow, and
    // this per-quad bit is the only thing in the data that keeps clutter off
    // them - it is how the original client kept grass out of the abbey.
    if (chunk.isEffectDisabled(quadY, quadX)) return out;

    // The alpha maps are 64x64 across the chunk, indexed the way the clutter
    // scatterer indexes them.
    const float u = fracX / GRASS_CHUNK_QUADS;
    const float v = fracY / GRASS_CHUNK_QUADS;

    // Layers paint over one another, so the topmost takes its alpha and the
    // ones beneath share what it leaves - which means walking them from the
    // top down, not the bottom up.
    //
    // Bottom up gives the lowest layer the most weight, and on a road that is
    // the dirt underneath the cobblestone: the dirt grows, so grass came up
    // through the paving no matter how correctly the cobblestone layer was
    // marked bare. Three fixes went past this because each asked whether the
    // road layer was suppressed - it was - rather than whether it was getting
    // its weight.
    std::array<float, 4> weight{};
    float remaining = 1.0f;
    for (size_t i = context.layerCount; i-- > 1;) {
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
        glm::vec3 shadow(0.0f);
        glm::vec3 highlight(0.0f);
        for (size_t i = 0; i < context.layerCount; ++i) {
            shadow += context.layerShadow[i] * weight[i];
            highlight += context.layerHighlight[i] * weight[i];
        }
        out.groundShadow = shadow;
        out.groundHighlight = highlight;
        out.hasGroundColor = true;
    }

    // The quad's own layer, not a blend of every layer that grows. The map
    // assigns each quad the layer its ground effect comes from - the
    // dominant paint - and the original client grows clutter from that layer
    // alone. Summing every growing layer's weight let a faint wash of grass
    // alpha seed ground whose dominant texture is bare dirt. The mapped
    // layer's blend weight still scales the density, so a boundary fades out
    // instead of ending on the quad grid.
    const uint32_t mapped = chunk.effectLayerFor(quadY, quadX);
    if (mapped < context.layerCount && context.grows[mapped]) {
        out.suitability = clamp01(weight[mapped]);
        out.effectId = context.effectId[mapped];
    }

    // The verge: how far this point stands from the nearest quad that is a
    // road, a path, or masked no-grow (a farm row, a building footprint).
    // The generator shortens and thins by this, so full meadow does not grow
    // to the last texel of tarmac and then stop - it gives way over about a
    // quad and a half. Chunk-local by design: a road in the next chunk over
    // eases its own side from its own quads.
    if (out.suitability > 0.0f) {
        constexpr float kVergeQuads = 1.4f;  // ~6 yards
        float nearest = kVergeQuads;
        for (int vy = quadY - 2; vy <= quadY + 2; ++vy) {
            for (int vx = quadX - 2; vx <= quadX + 2; ++vx) {
                if (vx < 0 || vx > 7 || vy < 0 || vy > 7) continue;
                bool bare = chunk.isEffectDisabled(vy, vx);
                if (!bare) {
                    const uint32_t layer = chunk.effectLayerFor(vy, vx);
                    bare = layer < context.layerCount && context.roadLike[layer];
                }
                if (!bare) continue;
                // Distance to the quad's rectangle, not its centre, so a
                // straight road edge eases evenly along its whole length.
                const float dx = std::max({static_cast<float>(vx) - fracX,
                                           fracX - static_cast<float>(vx + 1), 0.0f});
                const float dy = std::max({static_cast<float>(vy) - fracY,
                                           fracY - static_cast<float>(vy + 1), 0.0f});
                nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy));
            }
        }
        const float t = clamp01(nearest / kVergeQuads);
        out.wildness = t * t * (3.0f - 2.0f * t);
    }

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
