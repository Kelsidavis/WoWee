// What grass looks like on a patch of ground, derived from the detail doodads
// the map places there.
//
// The names carry the classification. The asset set is <zone><type><n> and the
// type is one of about a dozen three-letter codes, so an effect that places
// elwgra01 is meadow and one that places blaroc03 is scree. These cases pin
// that reading, and pin that nothing here knows what a zone is - a profile
// that had to be corrected per zone would be the wrong shape entirely.

#include <catch_amalgamated.hpp>

#include <string>
#include <vector>

#include "pipeline/grass_profile.hpp"

using wowee::pipeline::categoriseDoodad;
using wowee::pipeline::deriveProfile;
using wowee::pipeline::GrassCategory;
using wowee::pipeline::GrassProfile;
using wowee::pipeline::isRoadLikeTexture;
using wowee::pipeline::profileFor;

TEST_CASE("detail doodads are read by their type code", "[grass][profile]") {
    SECTION("the four commonest codes") {
        // gra 156, bus 119, roc 71, flo 67 across the shipped detail set.
        REQUIRE(categoriseDoodad("elwgra01.m2") == GrassCategory::Meadow);
        REQUIRE(categoriseDoodad("badbus06.m2") == GrassCategory::Scrub);
        REQUIRE(categoriseDoodad("blaroc02.m2") == GrassCategory::Barren);
        REQUIRE(categoriseDoodad("redflo01.m2") == GrassCategory::Flowers);
    }

    SECTION("bones and cones are dry, not barren") {
        REQUIRE(categoriseDoodad("icebon03.m2") == GrassCategory::Dry);
        REQUIRE(categoriseDoodad("dslbon01.m2") == GrassCategory::Dry);
    }

    SECTION("a full path is read from its basename") {
        REQUIRE(categoriseDoodad("World\\NoDXT\\Detail\\ElwGra02.m2") ==
                GrassCategory::Meadow);
        REQUIRE(categoriseDoodad("world/nodxt/detail/BlaRoc01.M2") ==
                GrassCategory::Barren);
    }

    SECTION("names longer than six characters are not sliced") {
        // Reading a fixed offset out of these picks the wrong three letters.
        REQUIRE(categoriseDoodad("stranglethornfern04.m2") == GrassCategory::Scrub);
    }

    SECTION("an unrecognised name grows like grass") {
        // The largest group and the right guess for a name we do not know.
        REQUIRE(categoriseDoodad("somethingentirelynew.m2") == GrassCategory::Meadow);
        REQUIRE(categoriseDoodad("") == GrassCategory::Meadow);
    }
}

TEST_CASE("scree grows less and drier than meadow", "[grass][profile]") {
    const GrassProfile meadow = profileFor(GrassCategory::Meadow);
    const GrassProfile barren = profileFor(GrassCategory::Barren);
    const GrassProfile dry = profileFor(GrassCategory::Dry);
    const GrassProfile scrub = profileFor(GrassCategory::Scrub);

    REQUIRE(barren.densityScale < meadow.densityScale);
    REQUIRE(barren.heightScale < meadow.heightScale);
    REQUIRE(dry.densityScale < meadow.densityScale);

    // Dry growth is browner: more red than green at the tip, where meadow is
    // the other way round.
    REQUIRE(dry.tipColor.r > dry.tipColor.g);
    REQUIRE(meadow.tipColor.g > meadow.tipColor.r);

    // Scrub is the tall stiff one.
    REQUIRE(scrub.heightScale > meadow.heightScale);
    REQUIRE(scrub.stiffness > meadow.stiffness);

    // Going to seed is what drying grass does, so dry ground seeds the most;
    // blooms belong where the map plants flowers.
    const GrassProfile flowers = profileFor(GrassCategory::Flowers);
    REQUIRE(dry.seedChance > meadow.seedChance);
    REQUIRE(flowers.bloomChance > meadow.bloomChance);
    REQUIRE(barren.bloomChance == Catch::Approx(0.0f));
}

TEST_CASE("an effect's profile is its doodads mixed by weight", "[grass][profile]") {
    const std::vector<std::string> models{"elwgra01.m2", "blaroc02.m2"};

    SECTION("weighted toward grass") {
        const GrassProfile p = deriveProfile(models, {90, 10});
        REQUIRE(p.densityScale > 0.8f);
        REQUIRE(p.tipColor.g > p.tipColor.r);
    }

    SECTION("weighted toward rock") {
        const GrassProfile p = deriveProfile(models, {10, 90});
        REQUIRE(p.densityScale < 0.4f);
        REQUIRE(p.heightScale < profileFor(GrassCategory::Meadow).heightScale);
    }

    SECTION("the mix lies between its parts") {
        const GrassProfile even = deriveProfile(models, {50, 50});
        const GrassProfile meadow = profileFor(GrassCategory::Meadow);
        const GrassProfile barren = profileFor(GrassCategory::Barren);
        REQUIRE(even.densityScale < meadow.densityScale);
        REQUIRE(even.densityScale > barren.densityScale);
        REQUIRE(even.bloomChance < meadow.bloomChance);
        REQUIRE(even.seedChance > barren.seedChance);
    }
}

TEST_CASE("an effect that weights everything zero mixes evenly",
          "[grass][profile]") {
    // Some ground effects carry all-zero weights. Reading that as "none of
    // these doodads" gives the effect no profile at all; it means the four are
    // equally likely.
    const std::vector<std::string> models{"elwgra01.m2", "blaroc02.m2"};
    const GrassProfile zeroed = deriveProfile(models, {0, 0, 0, 0});
    const GrassProfile even = deriveProfile(models, {50, 50});

    REQUIRE(zeroed.densityScale == Catch::Approx(even.densityScale));
    REQUIRE(zeroed.heightScale == Catch::Approx(even.heightScale));
    REQUIRE(zeroed.stiffness == Catch::Approx(even.stiffness));
}

TEST_CASE("an effect with no doodad models still grows grass",
          "[grass][profile]") {
    // A doodad id that did not resolve to a model must not silently become
    // bare ground; the terrain still said something grows there.
    const GrassProfile p = deriveProfile({}, {});
    REQUIRE(p.densityScale == Catch::Approx(profileFor(GrassCategory::Meadow).densityScale));
}

TEST_CASE("weights shorter than the model list are treated as zero",
          "[grass][profile]") {
    const std::vector<std::string> models{"elwgra01.m2", "blaroc02.m2"};
    const GrassProfile p = deriveProfile(models, {100});
    // Only the grass carried weight, so this is meadow.
    REQUIRE(p.densityScale == Catch::Approx(profileFor(GrassCategory::Meadow).densityScale));
}

TEST_CASE("made surfaces are recognised by name", "[grass][profile]") {
    // Road textures carry real ground effects with real densities, so nothing
    // in the effect data marks them as bare. The name is the only signal, and
    // the shipped clutter placer has always relied on it alone.
    REQUIRE(isRoadLikeTexture("Tileset\\Elwynn\\ElwynnCobbleStone01.blp"));
    REQUIRE(isRoadLikeTexture("tileset/elwynn/ElwynnRoad01.blp"));
    REQUIRE(isRoadLikeTexture("Tileset\\Human\\HumanPath.blp"));
    REQUIRE(isRoadLikeTexture("StreetBrick.blp"));

    REQUIRE_FALSE(isRoadLikeTexture("Tileset\\Elwynn\\ElwynnGrass01.blp"));
    REQUIRE_FALSE(isRoadLikeTexture("Tileset\\Elwynn\\ElwynnDirt.blp"));
    REQUIRE_FALSE(isRoadLikeTexture(""));
}
