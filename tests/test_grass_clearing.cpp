// Clearings around the placed world: near a building or a prop the grass
// generator is told the ground is tame, easing back to wild a few yards out.
// These pin the geometry - box distance, the cleared ring, the ease band,
// the minimum across sources - and that an empty field suppresses nothing.

#include <catch_amalgamated.hpp>

#include "pipeline/grass_clearing.hpp"

using wowee::pipeline::GrassClearingField;
using wowee::pipeline::GrassClearingSource;

TEST_CASE("an empty field is wild everywhere", "[grass][clearing]") {
    GrassClearingField field;
    REQUIRE(field.wildness(0.0f, 0.0f) == 1.0f);

    field.build({}, -50.0f, -50.0f, 50.0f, 50.0f);
    REQUIRE(field.wildness(10.0f, -3.0f) == 1.0f);
}

TEST_CASE("wildness rises from a footprint's edge", "[grass][clearing]") {
    // A 10x10 building at the origin: cleared for 2 yards past its wall,
    // easing back to wild over 8 more.
    GrassClearingField field;
    field.build({{-5.0f, -5.0f, 5.0f, 5.0f, 2.0f, 8.0f}}, -50.0f, -50.0f, 50.0f, 50.0f);

    // Inside and against the wall: fully tame.
    REQUIRE(field.wildness(0.0f, 0.0f) == 0.0f);
    REQUIRE(field.wildness(6.5f, 0.0f) == 0.0f);
    // Midway up the ease band: some growth, not full.
    const float mid = field.wildness(11.0f, 0.0f);
    REQUIRE(mid > 0.2f);
    REQUIRE(mid < 0.8f);
    // Beyond the band: the wild.
    REQUIRE(field.wildness(16.0f, 0.0f) == 1.0f);
    // Distance is to the box, not its centre: a corner is further out than
    // the face at the same axis distance.
    REQUIRE(field.wildness(9.0f, 9.0f) > field.wildness(9.0f, 0.0f));
}

TEST_CASE("overlapping sources take the tamer answer", "[grass][clearing]") {
    GrassClearingField field;
    field.build({{-5.0f, -5.0f, 5.0f, 5.0f, 2.0f, 8.0f},
                 {12.0f, 0.0f, 12.0f, 0.0f, 1.0f, 4.0f}},
                -50.0f, -50.0f, 50.0f, 50.0f);

    // Beside the fence post at (12, 0), which sits inside the building's
    // ease band: the post's own clearing wins.
    REQUIRE(field.wildness(12.5f, 0.0f) == 0.0f);
}

TEST_CASE("sources beyond the window are dropped", "[grass][clearing]") {
    GrassClearingField field;
    field.build({{200.0f, 200.0f, 210.0f, 210.0f, 2.0f, 8.0f}},
                -50.0f, -50.0f, 50.0f, 50.0f);
    REQUIRE(field.sourceCount() == 0);
}
