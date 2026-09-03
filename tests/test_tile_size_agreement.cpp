// The size of an ADT tile, which three headers each declared for themselves.
//
// 533.33333 is not a number anyone chose: a tile is 1600/3 yards, and the
// whole 64x64 grid is 34133.33 across. Everything that turns a world position
// into a tile index divides by it, and everything that turns a tile index into
// a world position multiplies by it, so the two halves have to agree to the
// bit or a tile boundary lands in a different place depending on which
// direction you crossed it.
//
// Two files had it as 533.333, three decimals rather than five. Across the map
// that is 0.021 yards - far too small to see, and far too small to find, which
// is exactly why it should not be spelled out twice.
//
// The oracle is arithmetic rather than the code: a tile is 1600/3, and the
// world is 64 tiles.
#include <catch_amalgamated.hpp>

#include <cmath>
#include <utility>

#include "core/coordinates.hpp"
#include "pipeline/terrain_mesh.hpp"

TEST_CASE("a tile is sixteen hundred thirds of a yard", "[coordinates]") {
    // Not 533.33 and not 533.3333333: the client's own value is a five-decimal
    // truncation, and everything that shares it has to share that truncation
    // rather than a more accurate one.
    CHECK(wowee::core::coords::TILE_SIZE ==
          Catch::Approx(1600.0f / 3.0f).epsilon(1e-6));
}

TEST_CASE("a chunk is a sixteenth of a tile and a quad an eighth of that",
          "[coordinates]") {
    // The mesh generator derives its own constants from the tile size rather
    // than spelling any of them out, so pinning the tile pins the chain: 16
    // chunks to a tile, 8 quads to a chunk. A chunk is 33.33 yards and a quad
    // 4.17, which are the numbers the terrain comments quote.
    const float tile = wowee::core::coords::TILE_SIZE;
    CHECK(tile / 16.0f == Catch::Approx(33.3333f).epsilon(1e-4));
    CHECK(tile / 16.0f / 8.0f == Catch::Approx(4.16666f).epsilon(1e-4));
}

TEST_CASE("the world is sixty-four tiles across", "[coordinates]") {
    // 34133.33 in each axis, and the origin sits at the centre, which is what
    // ZEROPOINT names.
    CHECK(wowee::core::coords::ZEROPOINT ==
          Catch::Approx(32.0f * (1600.0f / 3.0f)).epsilon(1e-6));
    CHECK(64.0f * wowee::core::coords::TILE_SIZE ==
          Catch::Approx(2.0f * wowee::core::coords::ZEROPOINT).epsilon(1e-6));
}

TEST_CASE("the truncated spelling is measurably different", "[coordinates]") {
    // 533.333 was in two files. This is what it costs across the map: a fifth
    // of a millimetre per tile, two centimetres end to end. Harmless on its
    // own and the reason a second spelling survives unnoticed.
    const float truncated = 533.333f;
    CHECK(truncated != wowee::core::coords::TILE_SIZE);
    const float driftAcrossMap =
        std::abs(64.0f * (wowee::core::coords::TILE_SIZE - truncated));
    CHECK(driftAcrossMap > 0.001f);
    CHECK(driftAcrossMap < 0.05f);
}

TEST_CASE("pi is written down once", "[coordinates]") {
    // Twenty-two sites spelled it out, in four precisions:
    // 3.14159265358979323846f, 3.1415926535f, 3.14159265f and 3.14159f. The
    // first three are the same float; the fourth is a different one, off by
    // 2.6e-6, which is 0.00015 degrees of a turn.
    //
    // Nothing measurable came of that - the short spelling was in a bob phase,
    // a turn-rate default and a degrees display, none of it persisted or sent
    // - but it is the shape that made two files spell the ADT tile size to
    // three decimals and the rest to five.
    CHECK(wowee::core::coords::PI ==
          Catch::Approx(std::acos(-1.0)).epsilon(1e-6));
    CHECK(wowee::core::coords::TWO_PI ==
          Catch::Approx(2.0 * std::acos(-1.0)).epsilon(1e-6));
    CHECK(wowee::core::coords::TWO_PI == 2.0f * wowee::core::coords::PI);

    // The short spelling is a different float, which is why it counts as a
    // second value rather than a second way of writing the first.
    CHECK(3.14159f != wowee::core::coords::PI);
}

// Which render axis a tile index counts along.
//
// worldToTile takes the tile's X index from renderY and its Y index from
// renderX - the grid is turned a quarter turn from the axes it is addressed
// in. So a tile's render-X edge comes from its *Y* index, and anything that
// works out the tile's extent has to cross the axes the same way.
//
// TerrainManager::getTileBounds did not: it named its outputs for one axis and
// computed them from the other, which put every chunk-index guess a tile's
// worth of chunks away from the point being asked about. Nothing showed,
// because each caller checked its guess against the chunk's own position and
// widened the search - until the area lookup asked without checking, and
// answered with a chunk from far enough away to belong to another zone.
TEST_CASE("a tile's render-X edge comes from its Y index", "[coordinates]") {
    using wowee::core::coords::TILE_SIZE;
    using wowee::core::coords::worldToTile;

    // One tile west of the origin tile, so the two indices differ - the fault
    // is invisible on the diagonal, where x and y are the same number.
    constexpr int kTileX = 32, kTileY = 33;
    const float maxRenderX = (32 - kTileY) * TILE_SIZE;
    const float maxRenderY = (32 - kTileX) * TILE_SIZE;
    constexpr float kInside = 0.01f;

    // Both corners of that extent land in the tile it was worked out for.
    CHECK(worldToTile(maxRenderX - kInside, maxRenderY - kInside) ==
          std::pair<int, int>{kTileX, kTileY});
    CHECK(worldToTile(maxRenderX - TILE_SIZE + kInside,
                      maxRenderY - TILE_SIZE + kInside) ==
          std::pair<int, int>{kTileX, kTileY});

    // Crossing them the other way lands in a different tile, which is what
    // taking the X edge from the X index amounts to.
    CHECK(worldToTile(maxRenderY - kInside, maxRenderX - kInside) !=
          std::pair<int, int>{kTileX, kTileY});
}
