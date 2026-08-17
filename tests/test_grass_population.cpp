// Blades generated from the terrain, and the one property everything else
// rests on: a blade's identity comes from where it is in the world and from
// nothing else.
//
// The population is thrown away and rebuilt whenever the player moves far
// enough. If any part of a blade depended on the window it was generated in -
// the centre, the order, the frame - every rebuild would visibly shuffle the
// field. Spec §36. Most of the cases below are that property from a different
// angle.

#include <catch_amalgamated.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "pipeline/grass_population.hpp"

using wowee::pipeline::GrassBladeSample;
using wowee::pipeline::GrassPopulationParams;
using wowee::pipeline::GrassSuitability;
using wowee::pipeline::populateArea;

namespace {

/// Terrain that grows grass everywhere, flat, at a known height.
GrassSuitability everywhere(float, float) {
    GrassSuitability fit;
    fit.suitability = 1.0f;
    fit.effectId = 7;
    fit.rootHeight = 42.0f;
    return fit;
}

GrassSuitability nowhere(float, float) {
    return GrassSuitability{};
}

/// A boundary at x = 0: fully suitable to the west, nothing to the east, with
/// a ten-yard blend between. What a grass-to-dirt edge looks like.
GrassSuitability boundary(float x, float) {
    GrassSuitability fit;
    fit.suitability = std::clamp(0.5f - x / 20.0f, 0.0f, 1.0f);
    fit.rootHeight = 0.0f;
    return fit;
}

bool sameBlade(const GrassBladeSample& a, const GrassBladeSample& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.height == b.height &&
           a.facing == b.facing && a.width == b.width && a.phase == b.phase;
}

} // namespace

TEST_CASE("the same area generates the same blades every time", "[grass][population]") {
    const GrassPopulationParams params;
    std::vector<GrassBladeSample> first;
    std::vector<GrassBladeSample> second;

    REQUIRE(populateArea(100.0f, 200.0f, 8.0f, params, everywhere, first, 100000));
    REQUIRE(populateArea(100.0f, 200.0f, 8.0f, params, everywhere, second, 100000));

    REQUIRE(!first.empty());
    REQUIRE(first.size() == second.size());
    for (size_t i = 0; i < first.size(); ++i) {
        REQUIRE(sameBlade(first[i], second[i]));
    }
}

TEST_CASE("moving the window slides over a fixed population", "[grass][population]") {
    // The property that matters most. A window centred four yards east must
    // contain the very same blades in the overlap - same positions, same
    // heights, same facings - or the field would reshuffle every time the
    // player walked far enough to trigger a rebuild.
    const GrassPopulationParams params;
    std::vector<GrassBladeSample> here;
    std::vector<GrassBladeSample> shifted;

    REQUIRE(populateArea(0.0f, 0.0f, 10.0f, params, everywhere, here, 100000));
    REQUIRE(populateArea(4.0f, 0.0f, 10.0f, params, everywhere, shifted, 100000));

    // Every blade of the first window that falls inside the second must be
    // present there, identical.
    size_t overlapping = 0;
    for (const auto& a : here) {
        // Only blades the second window actually reaches: the window is round,
        // so a blade inside the first can sit outside the second.
        const float dx = a.x - 4.0f;
        if (dx * dx + a.y * a.y > 9.0f * 9.0f) continue;
        const bool found = std::any_of(shifted.begin(), shifted.end(),
                                       [&](const GrassBladeSample& b) { return sameBlade(a, b); });
        REQUIRE(found);
        ++overlapping;
    }
    REQUIRE(overlapping > 100);
}

TEST_CASE("terrain that grows nothing yields no blades", "[grass][population]") {
    std::vector<GrassBladeSample> blades;
    REQUIRE(populateArea(0.0f, 0.0f, 20.0f, GrassPopulationParams{}, nowhere, blades, 100000));
    REQUIRE(blades.empty());
}

TEST_CASE("density follows suitability across a boundary", "[grass][population]") {
    // Counted in three bands: well inside the grass, across the blend, and
    // well outside. It has to thin out, not stop.
    const GrassPopulationParams params;
    std::vector<GrassBladeSample> blades;
    REQUIRE(populateArea(0.0f, 0.0f, 30.0f, params, boundary, blades, 200000));

    auto countBetween = [&](float lo, float hi) {
        return std::count_if(blades.begin(), blades.end(), [&](const GrassBladeSample& b) {
            return b.x >= lo && b.x < hi;
        });
    };

    const auto west = countBetween(-30.0f, -20.0f);  // suitability 1.0
    const auto mid = countBetween(-5.0f, 5.0f);      // around 0.5
    const auto east = countBetween(20.0f, 30.0f);    // 0

    REQUIRE(west > 0);
    REQUIRE(east == 0);
    REQUIRE(mid > 0);
    REQUIRE(mid < west);
}

TEST_CASE("density scale thins the field without moving it", "[grass][population]") {
    GrassPopulationParams full;
    GrassPopulationParams half = full;
    half.densityScale = 0.5f;

    std::vector<GrassBladeSample> dense;
    std::vector<GrassBladeSample> sparse;
    REQUIRE(populateArea(0.0f, 0.0f, 15.0f, full, everywhere, dense, 100000));
    REQUIRE(populateArea(0.0f, 0.0f, 15.0f, half, everywhere, sparse, 100000));

    REQUIRE(sparse.size() < dense.size());
    REQUIRE(!sparse.empty());
    // Thinning removes blades; it must not relocate the ones that remain.
    for (const auto& b : sparse) {
        REQUIRE(std::any_of(dense.begin(), dense.end(),
                            [&](const GrassBladeSample& a) { return sameBlade(a, b); }));
    }
}

TEST_CASE("blades sit at the height the terrain reported", "[grass][population]") {
    std::vector<GrassBladeSample> blades;
    REQUIRE(populateArea(0.0f, 0.0f, 5.0f, GrassPopulationParams{}, everywhere, blades, 100000));
    REQUIRE(!blades.empty());
    for (const auto& b : blades) {
        REQUIRE(b.z == Catch::Approx(42.0f));
    }
}

TEST_CASE("the blade cap is honoured", "[grass][population]") {
    // Landing exactly on the cap is what filling-until-full does, and that is
    // the behaviour with the directional failure. Under the cap by some margin
    // is what thinning to fit looks like.
    std::vector<GrassBladeSample> blades;
    populateArea(0.0f, 0.0f, 40.0f, GrassPopulationParams{}, everywhere, blades, 64);
    REQUIRE(blades.size() <= 64);
    REQUIRE(!blades.empty());
}

TEST_CASE("generated blades are within the requested area", "[grass][population]") {
    std::vector<GrassBladeSample> blades;
    const float radius = 12.0f;
    REQUIRE(populateArea(50.0f, -30.0f, radius, GrassPopulationParams{}, everywhere,
                         blades, 100000));
    REQUIRE(!blades.empty());
    for (const auto& b : blades) {
        // Round window, with one lattice cell of slack: a cell whose centre is
        // inside can jitter its blade just outside.
        const float dx = b.x - 50.0f;
        const float dy = b.y + 30.0f;
        REQUIRE(std::sqrt(dx * dx + dy * dy) < radius + 1.0f);
    }
}

TEST_CASE("heights and facings vary between blades", "[grass][population]") {
    std::vector<GrassBladeSample> blades;
    REQUIRE(populateArea(0.0f, 0.0f, 10.0f, GrassPopulationParams{}, everywhere,
                         blades, 100000));
    REQUIRE(blades.size() > 50);

    const bool heightsDiffer = std::any_of(blades.begin(), blades.end(),
        [&](const GrassBladeSample& b) { return b.height != blades[0].height; });
    const bool facingsDiffer = std::any_of(blades.begin(), blades.end(),
        [&](const GrassBladeSample& b) { return b.facing != blades[0].facing; });
    const bool widthsDiffer = std::any_of(blades.begin(), blades.end(),
        [&](const GrassBladeSample& b) { return b.width != blades[0].width; });
    REQUIRE(heightsDiffer);
    REQUIRE(facingsDiffer);
    REQUIRE(widthsDiffer);

    for (const auto& b : blades) {
        REQUIRE(b.height > 0.0f);
        REQUIRE(b.facing >= 0.0f);
        REQUIRE(b.facing <= Catch::Approx(6.2831853f).margin(0.001));
    }
}

TEST_CASE("grass shortens toward a road and stands deep in the open",
          "[grass][population]") {
    // Suitability is the distance-to-road signal, so height follows it: verge
    // grass at a blend boundary is shorter on average than the open field,
    // instead of standing knee-high to the wheel ruts.
    const GrassPopulationParams params;
    std::vector<GrassBladeSample> blades;
    REQUIRE(populateArea(0.0f, 0.0f, 30.0f, params, boundary, blades, 200000));

    auto meanHeightBetween = [&](float lo, float hi) {
        float sum = 0.0f;
        int n = 0;
        for (const auto& b : blades) {
            if (b.x >= lo && b.x < hi) { sum += b.height; ++n; }
        }
        REQUIRE(n > 20);
        return sum / static_cast<float>(n);
    };

    // Deep field (suitability 1.0) vs the thin end of the verge (~0.15).
    const float open = meanHeightBetween(-30.0f, -15.0f);
    const float verge = meanHeightBetween(4.0f, 7.0f);
    REQUIRE(verge < open * 0.85f);
}

TEST_CASE("hitting the blade cap thins the whole window, not its far side",
          "[grass][population]") {
    // The loop walks south to north. Filling until full and stopping deletes
    // the northern rows outright, so the field ends on a line: grass behind
    // the player and none ahead, depending only on which way north is. The
    // cap has to thin everywhere instead.
    GrassPopulationParams params;
    std::vector<GrassBladeSample> blades;
    const size_t cap = 400;
    REQUIRE(populateArea(0.0f, 0.0f, 20.0f, params, everywhere, blades, cap));
    REQUIRE(blades.size() <= cap);
    REQUIRE(blades.size() > cap / 4);

    // Both halves populated, north as well as south.
    const auto north = std::count_if(blades.begin(), blades.end(),
                                     [](const GrassBladeSample& b) { return b.y > 5.0f; });
    const auto south = std::count_if(blades.begin(), blades.end(),
                                     [](const GrassBladeSample& b) { return b.y < -5.0f; });
    REQUIRE(north > 0);
    REQUIRE(south > 0);
    // Comparable, rather than one side carrying the entire population.
    REQUIRE(north * 4 > south);
    REQUIRE(south * 4 > north);
}
