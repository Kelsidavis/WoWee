// The size a screen recording is encoded at.
//
// 4:2:0 video - what every player expects, and what the encoders here are given
// - keeps colour at half resolution both ways, so an odd side has no half and
// the encoder refuses the frame, or worse, crops a line off it. And a Retina
// window is four times the pixels of the 1080p the recording is kept to.
#include <catch_amalgamated.hpp>
#include "core/screen_recorder.hpp"

using wowee::core::recordingFrameSize;

TEST_CASE("a recording keeps a window that already fits", "[recording]") {
    const auto size = recordingFrameSize(1600, 900);
    CHECK(size.width == 1600);
    CHECK(size.height == 900);
}

TEST_CASE("a tall window is scaled to 1080 lines, shape kept, sides even", "[recording]") {
    // The Retina window this was first recorded from.
    const auto retina = recordingFrameSize(2752, 1728);
    CHECK(retina.height == 1080);
    CHECK(retina.width == 1720);

    const auto odd = recordingFrameSize(3601, 2261);
    CHECK(odd.height == 1080);
    CHECK(odd.width % 2 == 0);
    CHECK(odd.width == 1720);  // 3601 * 1080 / 2261 = 1720.07
}

TEST_CASE("an odd-sized small window loses a line rather than failing", "[recording]") {
    const auto size = recordingFrameSize(1281, 721);
    CHECK(size.width == 1280);
    CHECK(size.height == 720);
}

TEST_CASE("a window with no size records nothing", "[recording]") {
    const auto size = recordingFrameSize(0, 0);
    CHECK(size.width == 0);
    CHECK(size.height == 0);
}
