// What a model's sway parameters come out as, and how tall it says it is.
//
// plantHeight used to be filled in only on the paths that sway, so everything
// else reached the vertex shader with zero and ModelHeight came out a flat 1.0.
// A fragment that wants to know how far up the model it sits - a flame card
// fading out over its own tip - had nothing to ask. A fire is not foliage and
// still has a top.
#include <catch_amalgamated.hpp>

#include "rendering/m2_sway.hpp"

using wowee::rendering::m2SwayFor;
using wowee::rendering::m2SwayAmplitude;

namespace {
// (sky, hangingCloth, windFoliage, groundDetail)
constexpr bool kNotSky = false, kNotCloth = false, kNotFoliage = false, kNotDetail = false;
}  // namespace

TEST_CASE("every model reports its own height", "[m2][sway]") {
    SECTION("a still prop") {
        // OrcBonFire's own numbers: its geometry runs from -0.30 to 8.75, and
        // 9.05 is the height the shader divides by.
        const auto sway = m2SwayFor(kNotSky, kNotCloth, kNotFoliage, kNotDetail, -0.3f, 8.75f);
        CHECK(sway.mode == 0);                      // does not sway
        CHECK(sway.plantHeight == Catch::Approx(9.05f));
    }

    SECTION("measured from the model's base, or z=0 when the base is above it") {
        // A doodad whose geometry sits below its origin is that much taller,
        // and one floating above z=0 is not measured from the ground up.
        CHECK(m2SwayFor(kNotSky, kNotCloth, kNotFoliage, kNotDetail, -4.0f, 2.0f).plantHeight
              == Catch::Approx(6.0f));
        CHECK(m2SwayFor(kNotSky, kNotCloth, kNotFoliage, kNotDetail, 3.0f, 5.0f).plantHeight
              == Catch::Approx(5.0f));
    }

    SECTION("never zero, so a fraction of it is never a divide by nothing") {
        const auto sway = m2SwayFor(kNotSky, kNotCloth, kNotFoliage, kNotDetail, 0.0f, 0.0f);
        CHECK(sway.plantHeight >= 0.05f);
    }
}

TEST_CASE("the sway modes are unchanged by that", "[m2][sway]") {
    SECTION("sky") {
        const auto sway = m2SwayFor(true, kNotCloth, kNotFoliage, kNotDetail, 0.0f, 40.0f);
        CHECK(sway.mode == -1);
    }
    SECTION("hanging cloth is held at the top") {
        const auto sway = m2SwayFor(kNotSky, true, kNotFoliage, kNotDetail, 0.0f, 6.0f);
        CHECK(sway.mode == 3);
        CHECK(sway.refHeight == Catch::Approx(6.0f));   // the top, not the span
        CHECK(sway.plantHeight == Catch::Approx(6.0f));
        CHECK(sway.amp == Catch::Approx(0.3f));         // a twentieth of the drop
    }
    SECTION("cloth on a planted pole is held at the foot instead") {
        // Same wind, same amplitude, the weight turned end for end: the shader
        // reads the mode and nothing else changes. A standard whose foot is in
        // the ground must not swing there, which mode 3 made it do hardest of
        // anywhere on the model.
        const auto sway = m2SwayFor(kNotSky, true, kNotFoliage, kNotDetail, 0.0f, 6.0f,
                                    /*standingCloth=*/true);
        CHECK(sway.mode == 4);
        CHECK(sway.refHeight == Catch::Approx(6.0f));
        CHECK(sway.plantHeight == Catch::Approx(6.0f));
        CHECK(sway.amp == Catch::Approx(0.3f));
    }
    SECTION("standing only means anything for cloth") {
        const auto sway = m2SwayFor(kNotSky, kNotCloth, true, kNotDetail, 0.0f, 20.0f,
                                    /*standingCloth=*/true);
        CHECK(sway.mode == 1);
    }
    SECTION("a tree still throws what it threw") {
        const auto sway = m2SwayFor(kNotSky, kNotCloth, true, kNotDetail, 0.0f, 20.0f);
        CHECK(sway.mode == 1);
        CHECK(sway.refHeight == Catch::Approx(20.0f));
        CHECK(sway.plantHeight == Catch::Approx(20.0f));
    }
    SECTION("ground detail sways on its own mode") {
        const auto sway = m2SwayFor(kNotSky, kNotCloth, true, true, 0.0f, 0.8f);
        CHECK(sway.mode == 2);
        CHECK(sway.plantHeight == Catch::Approx(0.8f));
    }
}

TEST_CASE("the wind diagnostic leaves other animation paths alone", "[m2][sway]") {
    const auto tree = m2SwayFor(kNotSky, kNotCloth, true, kNotDetail, 0.0f, 15.0f);
    const auto grass = m2SwayFor(kNotSky, kNotCloth, true, true, 0.0f, 0.8f);
    const auto cloth = m2SwayFor(kNotSky, true, kNotFoliage, kNotDetail, 0.0f, 6.0f);
    CHECK(m2SwayAmplitude(tree, false) == tree.amp);
    CHECK(m2SwayAmplitude(tree, true) == 0.0f);
    CHECK(m2SwayAmplitude(grass, true) == grass.amp);
    CHECK(m2SwayAmplitude(cloth, true) == cloth.amp);
}
