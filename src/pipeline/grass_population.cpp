#include "pipeline/grass_population.hpp"

#include <algorithm>
#include <cmath>

#include "core/coordinates.hpp"

namespace wowee {
namespace pipeline {

namespace {

/// Integer hash, from the lattice cell and a salt. No floating point anywhere
/// in the identity of a blade: the same cell has to hash the same on every
/// machine and in every build, or a rebuild would move blades.
uint32_t hashCell(int32_t cx, int32_t cy, uint32_t salt) {
    uint32_t h = static_cast<uint32_t>(cx) * 0x9e3779b9u;
    h ^= static_cast<uint32_t>(cy) * 0x85ebca6bu;
    h ^= salt;
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

float hashUnit(int32_t cx, int32_t cy, uint32_t salt) {
    return static_cast<float>(hashCell(cx, cy, salt) & 0xffffffu) /
           static_cast<float>(0xffffff);
}

float smoothstep01(float t) {
    return t * t * (3.0f - 2.0f * t);
}

/// Smooth value noise over a coarse world lattice, 0..1. Built on the same
/// integer hash as everything else, so it is world-anchored and identical on
/// every rebuild - a drift of deep meadow stays where it is.
float smoothNoise(float x, float y, float scale, uint32_t salt) {
    const float fx = x / scale;
    const float fy = y / scale;
    const auto cx = static_cast<int32_t>(std::floor(fx));
    const auto cy = static_cast<int32_t>(std::floor(fy));
    const float tx = smoothstep01(fx - static_cast<float>(cx));
    const float ty = smoothstep01(fy - static_cast<float>(cy));

    const float a = hashUnit(cx, cy, salt);
    const float b = hashUnit(cx + 1, cy, salt);
    const float c = hashUnit(cx, cy + 1, salt);
    const float d = hashUnit(cx + 1, cy + 1, salt);
    const float top = a + (b - a) * tx;
    const float bottom = c + (d - c) * tx;
    return top + (bottom - top) * ty;
}

} // namespace

bool populateArea(float centerX, float centerY, float radius,
                  const GrassPopulationParams& params,
                  const SuitabilitySampler& sample,
                  std::vector<GrassBladeSample>& out,
                  size_t maxBlades,
                  const ProfileLookupFn& profileFor) {
    out.clear();
    if (!sample || params.spacing <= 0.0f || radius <= 0.0f) return true;

    const float spacing = params.spacing;

    // Anchor the lattice to the world rather than to the centre. Walking east
    // must slide a window over a population that was always there, not shift
    // every blade along with the player.
    const auto minCellX = static_cast<int32_t>(std::floor((centerX - radius) / spacing));
    const auto maxCellX = static_cast<int32_t>(std::floor((centerX + radius) / spacing));
    const auto minCellY = static_cast<int32_t>(std::floor((centerY - radius) / spacing));
    const auto maxCellY = static_cast<int32_t>(std::floor((centerY + radius) / spacing));

    const float densityScale = std::max(params.densityScale, 0.0f);
    const float radiusSq = radius * radius;

    // If the window holds more candidates than the cap allows, thin every one
    // of them by the same factor rather than filling until full and stopping.
    // The loop walks south to north, so stopping deletes the northern rows
    // outright - the field simply ends on a line, and which line depends on
    // nothing the player can see. Thinning uniformly degrades the whole window
    // instead, and being hash-driven it stays stable frame to frame.
    const auto totalCells = static_cast<size_t>(maxCellX - minCellX + 1) *
                            static_cast<size_t>(maxCellY - minCellY + 1);
    const float capFactor =
        (totalCells > maxBlades)
            ? 0.95f * static_cast<float>(maxBlades) / static_cast<float>(totalCells)
            : 1.0f;

    for (int32_t cy = minCellY; cy <= maxCellY; ++cy) {
        for (int32_t cx = minCellX; cx <= maxCellX; ++cx) {
            // Off the lattice, hard enough that no lattice shows through.
            //
            // A jitter of half a cell keeps every blade inside its own cell,
            // which is stratified sampling: it enforces a minimum spacing
            // between neighbours and the field comes out in rows, like a
            // planted crop. Jittering past the cell lets blades cluster and
            // leave gaps, which is what a meadow does. Alternate rows are also
            // offset half a cell, so the columns that ran north to south have
            // no straight line left to run along.
            const float rowShift = (cy & 1) ? 0.5f : 0.0f;
            const float jx = (hashUnit(cx, cy, params.seed ^ 0x01u) - 0.5f) * 1.9f;
            const float jy = (hashUnit(cx, cy, params.seed ^ 0x02u) - 0.5f) * 1.9f;
            const float wx = (static_cast<float>(cx) + 0.5f + rowShift + jx) * spacing;
            const float wy = (static_cast<float>(cy) + 0.5f + jy) * spacing;

            // Round, not square. Grass draws within a radius of the player, so
            // the corners of a square window were a quarter of the generation
            // cost spent where nothing is ever drawn.
            const float ddx = wx - centerX;
            const float ddy = wy - centerY;
            if (ddx * ddx + ddy * ddy > radiusSq) continue;

            const GrassSuitability fit = sample(wx, wy);
            if (fit.suitability <= 0.0f) continue;

            // What grows here, from what the map plants here. Scree thins
            // further than meadow does on ground both call suitable.
            const GrassProfileRef profile =
                profileFor ? profileFor(fit.effectId) : GrassProfileRef{};

            // Thin by suitability rather than cutting at a threshold: this is
            // what makes a blend boundary fade out instead of ending on a line.
            const float keep = hashUnit(cx, cy, params.seed ^ 0x03u);
            if (keep >= fit.suitability * densityScale * profile.densityScale * capFactor) {
                continue;
            }

            if (out.size() >= maxBlades) return false;

            GrassBladeSample blade;
            blade.x = wx;
            blade.y = wy;
            blade.z = fit.rootHeight;
            // Verges are short and the open field is deep. Suitability is
            // already the distance-to-road signal - it blends down toward
            // every road and dirt texture - so grass shortens approaching one
            // instead of standing knee-high to the wheel ruts. On top of
            // that, drifts of deep meadow twenty-odd yards across, from
            // world-anchored noise, so open ground is not one uniform pile.
            const float verge = 0.55f + 0.45f * std::min(fit.suitability * 1.6f, 1.0f);
            const float depthNoise = smoothNoise(wx, wy, 23.0f, params.seed ^ 0x07u);
            const float meadowDepth =
                0.8f + 0.85f * smoothstep01(std::clamp((depthNoise - 0.4f) / 0.35f, 0.0f, 1.0f));

            blade.height = params.baseHeight * profile.heightScale * verge * meadowDepth *
                           (1.0f - params.heightVariation +
                            2.0f * params.heightVariation * hashUnit(cx, cy, params.seed ^ 0x04u));
            blade.facing = hashUnit(cx, cy, params.seed ^ 0x05u) * core::coords::TWO_PI;
            // Width varies per blade too; a sward of one width reads as
            // extruded, whatever the heights do.
            blade.width = params.baseWidth * profile.widthScale *
                          (0.7f + 0.8f * hashUnit(cx, cy, params.seed ^ 0x08u));
            blade.phase = hashUnit(cx, cy, params.seed ^ 0x06u);
            blade.profileIndex = profile.index;
            blade.groundShadow = fit.groundShadow;
            blade.groundHighlight = fit.groundHighlight;
            blade.hasGroundColor = fit.hasGroundColor;
            blade.submerged = fit.submerged;
            out.push_back(blade);
        }
    }
    return true;
}

} // namespace pipeline
} // namespace wowee
