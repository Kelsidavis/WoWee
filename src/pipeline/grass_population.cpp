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

    for (int32_t cy = minCellY; cy <= maxCellY; ++cy) {
        for (int32_t cx = minCellX; cx <= maxCellX; ++cx) {
            // Jitter off the lattice, or the field reads as a checkerboard.
            const float jx = hashUnit(cx, cy, params.seed ^ 0x01u) - 0.5f;
            const float jy = hashUnit(cx, cy, params.seed ^ 0x02u) - 0.5f;
            const float wx = (static_cast<float>(cx) + 0.5f + jx) * spacing;
            const float wy = (static_cast<float>(cy) + 0.5f + jy) * spacing;

            const GrassSuitability fit = sample(wx, wy);
            if (fit.suitability <= 0.0f) continue;

            // What grows here, from what the map plants here. Scree thins
            // further than meadow does on ground both call suitable.
            const GrassProfileRef profile =
                profileFor ? profileFor(fit.effectId) : GrassProfileRef{};

            // Thin by suitability rather than cutting at a threshold: this is
            // what makes a blend boundary fade out instead of ending on a line.
            const float keep = hashUnit(cx, cy, params.seed ^ 0x03u);
            if (keep >= fit.suitability * densityScale * profile.densityScale) continue;

            if (out.size() >= maxBlades) return false;

            GrassBladeSample blade;
            blade.x = wx;
            blade.y = wy;
            blade.z = fit.rootHeight;
            blade.height = params.baseHeight * profile.heightScale *
                           (1.0f - params.heightVariation +
                            2.0f * params.heightVariation * hashUnit(cx, cy, params.seed ^ 0x04u));
            blade.facing = hashUnit(cx, cy, params.seed ^ 0x05u) * core::coords::TWO_PI;
            blade.width = params.baseWidth * profile.widthScale;
            blade.phase = hashUnit(cx, cy, params.seed ^ 0x06u);
            blade.profileIndex = profile.index;
            blade.groundColor = fit.groundColor;
            blade.hasGroundColor = fit.hasGroundColor;
            out.push_back(blade);
        }
    }
    return true;
}

} // namespace pipeline
} // namespace wowee
