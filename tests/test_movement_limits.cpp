#include <catch_amalgamated.hpp>
#include "rendering/movement_limits.hpp"
#include "core/coordinates.hpp"

#include <cmath>

TEST_CASE("stock hill climbing limits are shared by all surfaces") {
    using namespace wowee::rendering::movement;
    REQUIRE(kMaxWalkableSlopeDegrees == 50.0f);
    REQUIRE(isWalkableNormal(kMinWalkableNormalZ));
    REQUIRE_FALSE(isWalkableNormal(kMinWalkableNormalZ - 0.001f));
    REQUIRE(isReachableStep(kMaxStepUp));
    REQUIRE_FALSE(isReachableStep(kMaxStepUp + 0.001f));
}

// A walkable slope can rise faster than the step-up budget allows for, which is
// why grounding cannot rely on the budget alone. At the steepest walkable angle
// a mounted player crosses more ground per frame than kMaxStepUp covers as soon
// as the frame runs long - and the floor selection rejects any surface above
// feet + budget as unreachable, so the terrain under a climbing player stops
// counting as ground and they sink into the hill.
TEST_CASE("a walkable slope out-climbs the step-up budget in a long frame") {
    using namespace wowee::rendering::movement;

    // tan(50 degrees), the rise per unit travelled along the steepest slope a
    // player may walk up.
    constexpr float kSteepestRisePerYard = 1.19175f;

    auto riseOverFrame = [](float speedYardsPerSec, float frameSeconds) {
        return speedYardsPerSec * frameSeconds * kSteepestRisePerYard;
    };

    // A smooth frame stays well inside the budget at every travel speed.
    CHECK(riseOverFrame(7.0f, 1.0f / 60.0f) < kMaxStepUp);   // running
    CHECK(riseOverFrame(14.0f, 1.0f / 60.0f) < kMaxStepUp);  // epic mount

    // A slow frame does not. This is the case that put the player inside the
    // hill, so grounding has to recover from penetration rather than assume it
    // cannot happen.
    CHECK(riseOverFrame(14.0f, 1.0f / 20.0f) > kMaxStepUp);
}

// Facing crosses two representations: the renderer holds the character's yaw in
// degrees, the game side holds canonical yaw in radians, and the frame loop
// converts render → game every frame. Both directions of that conversion were
// hand-written at four call sites, two of them inverting the other two from
// memory. If they ever disagree, facing a target writes one value and the next
// frame reads back another - which is how a cast could be accepted and then
// fail the server's arc check a second and a half later.
TEST_CASE("character yaw and canonical yaw convert back to each other") {
    using namespace wowee::core::coords;

    for (float deg = -720.0f; deg <= 720.0f; deg += 7.5f) {
        const float canonical = characterYawDegToCanonical(deg);
        const float back = canonicalToCharacterYawDeg(canonical);
        // Round-trips to the same heading, allowing for full turns.
        float delta = std::fmod(std::fabs(back - deg), 360.0f);
        if (delta > 180.0f) delta = 360.0f - delta;
        INFO("degrees: " << deg << " canonical: " << canonical << " back: " << back);
        CHECK(delta < 0.01f);
        CHECK(canonical >= -PI - 0.001f);
        CHECK(canonical <= PI + 0.001f);
    }

    // Render yaw is canonical + 90: canonicalToRender swaps x and y, and
    // canonical yaw is atan2(-dy, dx), so a heading of canonical 0 (north) has
    // render components (0, 1) and a render yaw of 90.
    CHECK(canonicalToCharacterYawDeg(0.0f) == Catch::Approx(90.0f).margin(1e-4));
    CHECK(characterYawDegToCanonical(90.0f) == Catch::Approx(0.0f).margin(1e-5));

    // And it must agree with taking the angle of the render direction directly,
    // which is how the combat auto-turn and the camera both produce it.
    for (float canon = -3.0f; canon <= 3.0f; canon += 0.25f) {
        const glm::vec3 dirCanonical(std::cos(canon), -std::sin(canon), 0.0f);
        const glm::vec3 dirRender = canonicalToRender(dirCanonical);
        const float renderYawDeg =
            std::atan2(dirRender.y, dirRender.x) * (180.0f / PI);
        float diff = std::fmod(std::fabs(renderYawDeg - canonicalToCharacterYawDeg(canon)), 360.0f);
        if (diff > 180.0f) diff = 360.0f - diff;
        INFO("canonical: " << canon);
        CHECK(diff < 0.01f);
    }
}

// Standing on the hillside over a cave used to drop the player onto the cave's
// ceiling. Being "inside" a WMO is decided by bounding-box containment, and an
// underground WMO's interior box reaches up through the ground above it - so
// the terrain veto meant for Undercity's halls fired out in the open, leaving
// the WMO as the only floor candidate and its ceiling as the nearest surface
// below.
TEST_CASE("the terrain veto only refuses ground overhead") {
    using namespace wowee::rendering::movement;
    constexpr float kStepUp = kMaxStepUp;

    SECTION("Undercity: the surface is ~113m over the halls, and refused") {
        REQUIRE(terrainIsOverheadRoof(true, 61.66f, -51.5f, kStepUp));
    }

    SECTION("the hillside over a cave is at the feet, and kept") {
        // Same containment answer, entirely different situation.
        REQUIRE_FALSE(terrainIsOverheadRoof(true, 120.0f, 120.0f, kStepUp));
        REQUIRE_FALSE(terrainIsOverheadRoof(true, 120.4f, 120.0f, kStepUp));
    }

    SECTION("ground below the feet is never a roof") {
        REQUIRE_FALSE(terrainIsOverheadRoof(true, 100.0f, 120.0f, kStepUp));
    }

    SECTION("outdoors nothing is vetoed, however far above") {
        REQUIRE_FALSE(terrainIsOverheadRoof(false, 500.0f, 0.0f, kStepUp));
    }

    SECTION("the boundary is what the player could step onto") {
        const float feet = 10.0f;
        const float edge = feet + kStepUp + 0.5f;
        REQUIRE_FALSE(terrainIsOverheadRoof(true, edge, feet, kStepUp));
        REQUIRE(terrainIsOverheadRoof(true, edge + 0.01f, feet, kStepUp));
    }
}

// A tunnel seam preferred the WMO floor whatever it was, which is right at a
// tunnel mouth and wrong where a ramp merely passes under the ground beside
// it. At the Orgrimmar valley entrance the ramp runs about 1.3 yards below
// the terrain it meets, and the preference pulled the player off the ground
// they were standing on and down inside the hillside - walking through the
// terrain with the ramp overhead.
TEST_CASE("a seam only prefers the WMO floor when it is the way in") {
    using namespace wowee::rendering::movement;
    constexpr float kStepUp = kMaxStepUp;

    SECTION("Orgrimmar's entrance: the terrain is at the feet, so it wins") {
        // The floor query's own numbers at render(1380.64, -4365.3): feet on
        // the terrain, the ramp 1.33 yards under it. Taking the ramp put the
        // player inside the hillside.
        REQUIRE_FALSE(wmoFloorIsWayIn(26.0326f, 26.0284f, kStepUp));
    }

    SECTION("a tunnel mouth: the hillside overhead cannot be the floor") {
        REQUIRE(wmoFloorIsWayIn(26.0f, 20.0f, kStepUp));
        REQUIRE(wmoFloorIsWayIn(61.66f, -51.5f, kStepUp));
    }

    SECTION("terrain below the feet is always the ground under them") {
        REQUIRE_FALSE(wmoFloorIsWayIn(20.0f, 26.0f, kStepUp));
    }

    SECTION("the boundary is what the player could step onto") {
        const float feet = 10.0f;
        const float edge = feet + kStepUp + 0.5f;
        REQUIRE_FALSE(wmoFloorIsWayIn(edge - 0.01f, feet, kStepUp));
        REQUIRE(wmoFloorIsWayIn(edge + 0.01f, feet, kStepUp));
    }

    SECTION("a gap test would have called the Orgrimmar ramp a tunnel") {
        // 1.33 yards is more than a step, which is why the rule asks where the
        // player is rather than how far apart the two surfaces are.
        REQUIRE(26.0326f - 24.698f > kStepUp);
    }
}

TEST_CASE("the floor pick anchors where the player actually stands", "[movement][floor]") {
    using wowee::rendering::movement::floorArbitrationAnchor;
    using wowee::rendering::movement::standingOnLastGround;

    SECTION("standing, the feet are the anchor") {
        // Level ground, and a step up onto a ledge: the feet are the honest
        // answer and are what lets the ledge win once they are level with it.
        CHECK(standingOnLastGround(true, 10.0f, 10.0f));
        CHECK(floorArbitrationAnchor(true, 10.0f, 10.0f) == Catch::Approx(10.0f));
        CHECK(floorArbitrationAnchor(true, 10.5f, 10.0f) == Catch::Approx(10.5f));
        // A real step down is still standing.
        CHECK(floorArbitrationAnchor(true, 9.6f, 10.0f) == Catch::Approx(9.6f));
    }

    SECTION("under the floor by more than a step is falling") {
        CHECK_FALSE(standingOnLastGround(true, 9.0f, 10.0f));
        CHECK(floorArbitrationAnchor(true, 9.0f, 10.0f) == Catch::Approx(10.0f));
    }

    SECTION("the Undercity elevator ramp") {
        // The frame the player fell through it: feet 2.81 under the landing
        // they were standing on, with the hall floor five yards below.
        constexpr float feet = -44.2411f;
        constexpr float lastGround = -41.4245f;
        constexpr float landing = -41.4273f;
        constexpr float hallFloor = -46.6305f;

        // Measured from the feet the hall floor is nearer, which is what took
        // the player through the ramp.
        CHECK(std::abs(hallFloor - feet) < std::abs(landing - feet));

        // Anchored where they were standing, the landing wins outright.
        const float anchor = floorArbitrationAnchor(true, feet, lastGround);
        CHECK(anchor == Catch::Approx(lastGround));
        CHECK(std::abs(landing - anchor) < std::abs(hallFloor - anchor));
    }

    SECTION("airborne is unchanged") {
        CHECK(floorArbitrationAnchor(false, 5.0f, 10.0f) == Catch::Approx(10.0f));
        CHECK(floorArbitrationAnchor(false, 12.0f, 10.0f) == Catch::Approx(10.0f));
    }
}

// Flying into the ground: the step stops at the surface and slides along it.
TEST_CASE("a flight step that ends in the ground slides along it", "[movement][flight]") {
    using namespace wowee::rendering::movement;
    using Height = std::optional<float>;

    // Ground rising in +x at the given slope from x = 0; flat before it.
    auto hill = [](float degrees) {
        const float rise = std::tan(glm::radians(degrees));
        return [rise](float x, float) -> Height { return x > 0.0f ? x * rise : 0.0f; };
    };
    const auto flat = [](float, float) -> Height { return 0.0f; };

    SECTION("a step clear of the ground is untouched") {
        const glm::vec3 from(-5.0f, 0.0f, 10.0f), to(-4.0f, 0.0f, 10.0f);
        CHECK(flightStepAgainstGround(hill(45.0f), from, to, 2.0f) == to);
    }

    SECTION("flying straight down ends on the ground") {
        const glm::vec3 end = flightStepAgainstGround(
            flat, glm::vec3(0.0f, 0.0f, 0.3f), glm::vec3(0.0f, 0.0f, -0.4f), 2.0f);
        CHECK(end.x == Catch::Approx(0.0f).margin(1e-4));
        CHECK(end.z == Catch::Approx(0.0f).margin(1e-3));
    }

    SECTION("level flight into a slope climbs it, slower the steeper it is") {
        // Level at the foot of the hill, with a frame's worth of travel into it
        // that out-climbs the step-up budget - the frame that used to go in.
        const glm::vec3 from(5.0f, 0.0f, 5.0f), to(6.0f, 0.0f, 5.0f);
        const glm::vec3 gentle = flightStepAgainstGround(hill(30.0f), glm::vec3(8.0f, 0.0f, 4.62f),
                                                         glm::vec3(9.0f, 0.0f, 4.62f), 2.0f);
        const glm::vec3 steep = flightStepAgainstGround(hill(45.0f), from, to, 2.0f);
        const glm::vec3 cliff = flightStepAgainstGround(hill(85.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                                                        glm::vec3(1.0f, 0.0f, 0.0f), 2.0f);
        // Never inside the ground...
        CHECK(gentle.z >= gentle.x * std::tan(glm::radians(30.0f)) - 1e-3f);
        CHECK(steep.z >= steep.x - 1e-3f);
        CHECK(cliff.z >= cliff.x * std::tan(glm::radians(85.0f)) - 1e-3f);
        // ...and less of the step gets through the steeper the face.
        CHECK(gentle.x - 8.0f > steep.x - 5.0f);
        CHECK(steep.x - 5.0f > cliff.x);
        CHECK(cliff.x < 0.05f);
    }

    SECTION("a step starting under the ground is left alone") {
        const glm::vec3 from(5.0f, 0.0f, 2.0f), to(6.0f, 0.0f, 2.0f);
        CHECK(flightStepAgainstGround(hill(45.0f), from, to, 2.0f) == to);
    }

    SECTION("no ground at the end - a hole, an unloaded tile - is no ground") {
        const auto holed = [](float x, float) -> Height {
            if (x > 0.5f) return std::nullopt;
            return 10.0f;
        };
        const glm::vec3 from(0.0f, 0.0f, 11.0f), to(1.0f, 0.0f, 5.0f);
        CHECK(flightStepAgainstGround(holed, from, to, 2.0f) == to);
    }
}
