// The chat window's saved background colour, and the repair of one saved wrong.
//
// The panel behind the chat text is a white image the interface tints, so the
// stored colour is the whole of what it looks like. This client stored white at
// full alpha before 7f052e7ff, which paints an opaque slab over the chat with
// the text lost in it. Fixing the default did not fix the installs that had
// already written that white to disk, and the fault came back as a report weeks
// after it was closed - so the reading side has to recognise it.

#include <catch_amalgamated.hpp>

#include "addons/chat_window_background.hpp"

using wowee::addons::kChatBackgroundDefault;
using wowee::addons::repairChatBackground;

TEST_CASE("the white background a pre-fix build saved is not restored",
          "[chat][interface-state]") {
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;
    REQUIRE(repairChatBackground(r, g, b, a));
    CHECK(r == kChatBackgroundDefault[0]);
    CHECK(g == kChatBackgroundDefault[1]);
    CHECK(b == kChatBackgroundDefault[2]);
    CHECK(a == kChatBackgroundDefault[3]);
}

TEST_CASE("no background this client restores hides the text",
          "[chat][interface-state]") {
    // The property the report is about, stated without reference to any one
    // wrong value: whatever comes back, the panel cannot be both white and
    // solid, because the chat draws white text on it.
    const float saved[][4] = {
        {1.0f, 1.0f, 1.0f, 1.0f},   // the pre-fix default
        {1.0f, 1.0f, 1.0f, 0.9f},   // white, near enough solid
        {1.0f, 1.0f, 1.0f, 0.0f},   // white but faded out, so latent
        {0.0f, 0.0f, 0.0f, 0.25f},  // the interface's own
        {0.1f, 0.0f, 0.3f, 0.8f},   // somebody's choice
    };
    for (const auto& c : saved) {
        float r = c[0], g = c[1], b = c[2], a = c[3];
        repairChatBackground(r, g, b, a);
        INFO("saved " << c[0] << "," << c[1] << "," << c[2] << "," << c[3]);
        const bool white = (r > 0.9f && g > 0.9f && b > 0.9f);
        CHECK_FALSE(white);
    }
}

TEST_CASE("a background somebody chose is left alone", "[chat][interface-state]") {
    // The repair is a repair and not a policy: it must not quietly overwrite a
    // colour the player picked, including a dark one at full opacity.
    const float chosen[][4] = {
        {0.0f, 0.0f, 0.0f, 0.25f},
        {0.0f, 0.0f, 0.0f, 1.0f},
        {0.2f, 0.4f, 0.6f, 0.5f},
        {1.0f, 0.0f, 0.0f, 1.0f},
        {0.9f, 0.9f, 0.9f, 1.0f},
    };
    for (const auto& c : chosen) {
        float r = c[0], g = c[1], b = c[2], a = c[3];
        INFO("saved " << c[0] << "," << c[1] << "," << c[2] << "," << c[3]);
        CHECK_FALSE(repairChatBackground(r, g, b, a));
        CHECK(r == c[0]);
        CHECK(g == c[1]);
        CHECK(b == c[2]);
        CHECK(a == c[3]);
    }
}

TEST_CASE("a white window faded out keeps the alpha it was faded to",
          "[chat][interface-state]") {
    // Alpha 0 is not the pre-fix default, so something moved it there. Only the
    // colour beneath is unset, and it matters: raising the opacity slider on a
    // window still holding white would bring the slab straight back.
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 0.0f;
    REQUIRE(repairChatBackground(r, g, b, a));
    CHECK(a == 0.0f);
    CHECK(r == kChatBackgroundDefault[0]);
}

TEST_CASE("the repair settles in one pass", "[chat][interface-state]") {
    // It runs on every load against a file it wrote itself, so a repair that
    // changed its own output would rewrite the file every session.
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;
    REQUIRE(repairChatBackground(r, g, b, a));
    CHECK_FALSE(repairChatBackground(r, g, b, a));
}
