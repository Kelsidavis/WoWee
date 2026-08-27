// Whether one client's asset tree may answer for another's missing files.
//
// The base fallback resolves per file, and a borrowed file is an error nowhere,
// so an expansion tree extracted from one client sitting over a base extracted
// from a different one draws the wrong game and says nothing. In this checkout
// the wotlk tree holds 18,943 files and the base holds 199,468, so nine lookups
// in ten are answered by the base: whatever populated it is most of what is on
// screen. Cataclysm is where that stops being subtle, its Azeroth being the
// sundered one, so an uncovered tile is drawn as the old world.
//
// The decision is the pure half of that and lives on its own so it can be
// tested without stb_image, the profiler and the loaders behind AssetManager.
#include <catch_amalgamated.hpp>

#include "pipeline/base_fallback.hpp"

using wowee::pipeline::BaseFallbackDecision;
using wowee::pipeline::decideBaseFallback;

TEST_CASE("the same client is used without comment", "[basefallback]") {
    CHECK(decideBaseFallback("wotlk", "wotlk", false) == BaseFallbackDecision::Use);
    CHECK(decideBaseFallback("wotlk", "wotlk", true) == BaseFallbackDecision::Use);
}

TEST_CASE("a different client is refused", "[basefallback]") {
    // The case this exists for: a Cata tree over whatever populated Data/.
    CHECK(decideBaseFallback("wotlk", "cata", false) == BaseFallbackDecision::Refuse);
    CHECK(decideBaseFallback("classic", "tbc", false) == BaseFallbackDecision::Refuse);
}

TEST_CASE("a different client is used when forced", "[basefallback]") {
    CHECK(decideBaseFallback("wotlk", "cata", true) == BaseFallbackDecision::UseForced);
}

TEST_CASE("an unlabelled base is used and reported", "[basefallback]") {
    // Every manifest written before the expansion field. Refusing these would
    // break every existing install, and using them silently is the bug, so the
    // third answer is to use them and say so.
    CHECK(decideBaseFallback("", "cata", false) == BaseFallbackDecision::UseUnlabelled);
    CHECK(decideBaseFallback("", "wotlk", false) == BaseFallbackDecision::UseUnlabelled);
    CHECK(decideBaseFallback("", "", false) == BaseFallbackDecision::UseUnlabelled);
}

TEST_CASE("forcing does not silence an unlabelled base", "[basefallback]") {
    // The flag answers "use another client's tree anyway". It says nothing
    // about a tree whose client is unknown, and reading it as permission to go
    // quiet would hide the commoner of the two cases.
    CHECK(decideBaseFallback("", "cata", true) == BaseFallbackDecision::UseUnlabelled);
}

TEST_CASE("an unknown active expansion is not a mismatch", "[basefallback]") {
    // Nothing to contradict. Refusing here would break a client that has not
    // settled its profile yet, over a base that may well be the right one.
    CHECK(decideBaseFallback("wotlk", "", false) == BaseFallbackDecision::Use);
}
