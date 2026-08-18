// The per-zone grass look table. What grows stays the map's decision; these
// pin down that the authored overrides parse, match the right ground, and
// change a profile the way the file says - multiplying the scales, replacing
// the colours - without touching anything the entry does not name.

#include <catch_amalgamated.hpp>

#include <string>

#include "pipeline/grass_biomes.hpp"

using wowee::pipeline::GrassBiomeSet;
using wowee::pipeline::GrassProfile;
using wowee::pipeline::loadGrassBiomes;

namespace {

constexpr const char* kTable = R"({
  // comments are allowed - this is an authored file
  "biomes": [
    {
      "name": "Vineyard",
      "areas": [999],
      "densityScale": 0.0
    },
    {
      "name": "Wheatland",
      "zones": [40],
      "heightScale": 1.5,
      "tipColor": [0.8, 0.6, 0.2],
      "seedChance": 0.9,
      "headColorA": [0.9, 0.8, 0.5],
      "headColorB": [0.6, 0.5, 0.2]
    },
    {
      "name": "Bare",
      "zones": [51],
      "densityScale": 0.0
    },
    { "name": "unmatchable, skipped" },
    "not even an object"
  ]
})";

} // namespace

TEST_CASE("the biome table parses, skipping malformed entries", "[grass][biomes]") {
    std::string error;
    const GrassBiomeSet set = loadGrassBiomes(kTable, error);
    REQUIRE(error.empty());
    // The unmatchable entry and the non-object are dropped, not fatal.
    REQUIRE(set.size() == 3);
    REQUIRE(set.biomes_[1].name == "Wheatland");
}

TEST_CASE("matching prefers the subzone over the zone", "[grass][biomes]") {
    std::string error;
    const GrassBiomeSet set = loadGrassBiomes(kTable, error);

    // Area 999 sits inside zone 40; the vineyard row wins over wheatland.
    REQUIRE(set.findFor(999, 40) == 1);
    // Any other area of zone 40 is wheat.
    REQUIRE(set.findFor(123, 40) == 2);
    // Ground no row names gets no biome.
    REQUIRE(set.findFor(5, 12) == 0);
}

TEST_CASE("an override multiplies scales and replaces colours", "[grass][biomes]") {
    std::string error;
    const GrassBiomeSet set = loadGrassBiomes(kTable, error);
    const auto* wheat = set.biome(2);
    REQUIRE(wheat != nullptr);

    GrassProfile p;
    p.heightScale = 0.8f;   // as a scrub effect might have derived
    const float defaultWidth = p.widthScale;
    const glm::vec3 defaultRoot = p.rootColor;

    wheat->override_.apply(p);

    // Multiplied, so the effect's own character survives the biome.
    REQUIRE(p.heightScale == Catch::Approx(0.8f * 1.5f));
    // Unnamed fields untouched.
    REQUIRE(p.widthScale == defaultWidth);
    REQUIRE(p.rootColor == defaultRoot);
    // Named colours replaced.
    REQUIRE(p.tipColor.x == Catch::Approx(0.8f));
    REQUIRE(p.seedChance == Catch::Approx(0.9f));
    REQUIRE(p.headColorA.z == Catch::Approx(0.5f));
}

TEST_CASE("a bare biome zeroes density and nothing else", "[grass][biomes]") {
    std::string error;
    const GrassBiomeSet set = loadGrassBiomes(kTable, error);
    const auto* bare = set.biome(3);
    REQUIRE(bare != nullptr);

    GrassProfile p;
    bare->override_.apply(p);
    REQUIRE(p.densityScale == Catch::Approx(0.0f));
    REQUIRE(p.heightScale == Catch::Approx(1.0f));
}

TEST_CASE("bad JSON reports rather than crashes", "[grass][biomes]") {
    std::string error;
    const GrassBiomeSet broken = loadGrassBiomes("{ not json", error);
    REQUIRE(broken.size() == 0);
    REQUIRE(!error.empty());

    const GrassBiomeSet noArray = loadGrassBiomes("{}", error);
    REQUIRE(noArray.size() == 0);
    REQUIRE(!error.empty());
}
