// How many lines a label takes, asked on demand.
//
// FrameXML sets a string's text and reads its height in the same breath -
// WatchFrame_SetLine writes an objective, gives it a width and asks how tall it
// came out, all before anything is drawn again. So the count cannot come from
// the last frame's measure: a row that was just reset measures zero, and the
// tracker then gave a two-line objective a one-line row and drew the rows below
// it over the second line.
//
// Measured without a font here, which is the fallback path: no ImGui context
// means half the point size per character, so at the default twelve a character
// is six units wide and the widths below read as character counts times six.
#include <catch_amalgamated.hpp>

#include "ui/interface_fonts.hpp"

using wowee::ui::interfaceTextLines;

namespace {
/// The default point size with no context, so the numbers below are readable.
constexpr float kChar = 6.0f;  // 12.0f * 0.5f
}  // namespace

TEST_CASE("A label with nothing in it takes no lines", "[fonts][wrap]") {
    REQUIRE(interfaceTextLines("", "", 0.0f, 100.0f, false) == 0);
}

TEST_CASE("A label that wraps at nothing is one line", "[fonts][wrap]") {
    // Zero width is "as wide as your own text", which is what a label sized by
    // its text asks for - it cannot wrap, however long it is.
    REQUIRE(interfaceTextLines("Return to Tavernkeep Smitts at Darkshire", "",
                               0.0f, 0.0f, false) == 1);
}

TEST_CASE("A label wraps inside the width it was given", "[fonts][wrap]") {
    // "Return to" is nine characters and fits; " the" would make thirteen and
    // does not.
    REQUIRE(interfaceTextLines("Return to the inn", "", 0.0f, 10 * kChar, false) == 2);
    // Wide enough for all seventeen, so it stays on one line.
    REQUIRE(interfaceTextLines("Return to the inn", "", 0.0f, 20 * kChar, false) == 1);
}

TEST_CASE("A newline breaks a line at any width", "[fonts][wrap]") {
    // |n is WoW's spelling of a line break, and it breaks whether or not
    // anything is wrapping.
    REQUIRE(interfaceTextLines("one|ntwo", "", 0.0f, 0.0f, false) == 2);
}
