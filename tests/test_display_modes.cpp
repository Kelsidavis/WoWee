#include <catch_amalgamated.hpp>

#include "ui/display_modes.hpp"

using wowee::ui::displayResolutionIndexFor;
using wowee::ui::displayResolutionsForScreen;

// A tablet's resolution list is the screen and fractions of it. The screen
// swaps width and height when the device rotates, and the settings panel
// rewrites the saved size to the entry it picked. A size that no longer fits
// fell back to a smaller fraction, and the next rotation fell back again -
// two turns were enough to go from a half-scale render target to the panel's
// full 3.87 MP size.
TEST_CASE("a tablet keeps its render scale across a rotation", "[ui][display]") {
    const auto landscape = displayResolutionsForScreen(2360, 1640);
    const auto portrait = displayResolutionsForScreen(1640, 2360);

    // Half scale on a landscape screen is 1180x820; the same half scale after
    // rotating is 820x1180, not the only portrait entry 1180x820 fits into.
    const int portraitIndex = displayResolutionIndexFor(portrait, 1180, 820);
    REQUIRE(portraitIndex >= 0);
    CHECK(portrait[static_cast<std::size_t>(portraitIndex)].width == 820);
    CHECK(portrait[static_cast<std::size_t>(portraitIndex)].height == 1180);

    // Rotating back must not ratchet up to the native screen size either.
    const int landscapeIndex = displayResolutionIndexFor(landscape, 820, 1180);
    REQUIRE(landscapeIndex >= 0);
    CHECK(landscape[static_cast<std::size_t>(landscapeIndex)].width == 1180);
    CHECK(landscape[static_cast<std::size_t>(landscapeIndex)].height == 820);
    CHECK_FALSE(landscape[static_cast<std::size_t>(landscapeIndex)].native);
}

TEST_CASE("an exact resolution still wins over a rotated one", "[ui][display]") {
    const auto list = displayResolutionsForScreen(2360, 1640);
    // Both 1180x820 and 820x1180 are entries on some screens; the exact
    // size has to be chosen before the swapped-match fallback is considered.
    const int index = displayResolutionIndexFor(list, 1180, 820);
    REQUIRE(index >= 0);
    CHECK(list[static_cast<std::size_t>(index)].width == 1180);
    CHECK(list[static_cast<std::size_t>(index)].height == 820);
}

TEST_CASE("an unmatched size still lands on the closest mode", "[ui][display]") {
    const auto list = displayResolutionsForScreen(1180, 820);
    const int index = displayResolutionIndexFor(list, 640, 480);
    REQUIRE(index >= 0);
    CHECK(list[static_cast<std::size_t>(index)].width <= 640);
    CHECK(list[static_cast<std::size_t>(index)].height <= 480);
}
