// Which interface draws what, in the configuration a run actually gets.
//
// The transition's state is "FrameXML draws all of it", and nothing held that
// still. It used to rest on a list and a grouping rule agreeing - a default set
// naming forty-nine of the fifty-two elements, with the other three reached
// through "mainmenubar" - in a file where either could be edited alone. Both
// are gone with WOWEE_FRAMEXML_UI, and what is left to hold still is that no
// element falls out of the handover by some other route: the release net, or a
// suppression row that outlived the element it was written for.
//
// Getting it wrong is quiet, and quieter than it used to be. An element that
// stops being owned no longer means this client draws its own version instead -
// that version has been deleted for all but a handful - it means nothing draws
// it at all.

#include <catch_amalgamated.hpp>

#include "ui/framexml_takeover.hpp"

#include <cstdlib>
#include <string>
#include "core/env.hpp"

using namespace wowee::ui;

namespace {

/// WOWEE_LOAD_FRAMEXML still decides whether anything is owned at all, and it
/// is read once and cached - so it has to be cleared before the first question,
/// not inside the first test case. Otherwise a developer who happens to be
/// running with it set to 0 gets a failure that says nothing about the code.
const bool kCleanEnvironment = [] {
    wowee::core::unsetEnvVar("WOWEE_LOAD_FRAMEXML");
    return true;
}();

}  // namespace

TEST_CASE("The last element is still the last one", "[takeover]") {
    // The loop below counts up to Petition, so a new element added after it
    // would go unchecked and this is what notices. Named rather than counted,
    // because the enum has no sentinel and adding one would make every switch
    // over it non-exhaustive.
    REQUIRE(kCleanEnvironment);
    REQUIRE(uiElementName(UiElement::Petition) == "petition");
}

TEST_CASE("Every element is FrameXML's", "[takeover]") {
    REQUIRE(kCleanEnvironment);
    std::string notOwned;
    int total = 0;
    for (int i = 0; i <= static_cast<int>(UiElement::Petition); ++i) {
        const auto element = static_cast<UiElement>(i);
        ++total;
        if (!frameXmlOwns(element)) {
            notOwned += uiElementName(element);
            notOwned += ' ';
        }
    }
    // Fifty-two of them, and the count is asserted so that an element quietly
    // removed shows up as loudly as one quietly unowned.
    CHECK(total == 52);
    INFO("still drawn by this client: " << notOwned);
    CHECK(notOwned.empty());
}

TEST_CASE("The bar's pieces are owned through the whole bar", "[takeover]") {
    REQUIRE(kCleanEnvironment);
    // FrameXML draws all six as one frame - MainMenuBar, one strip of art with
    // the griffins on the ends - where this client built them as separate
    // pieces. They were owned through a grouping rule while elements were named
    // one at a time; the rule is gone with the naming, and this says the pieces
    // did not go with it.
    CHECK(frameXmlOwns(UiElement::ActionBar));
    CHECK(frameXmlOwns(UiElement::StanceBar));
    CHECK(frameXmlOwns(UiElement::XpBar));
    CHECK(frameXmlOwns(UiElement::RepBar));
    CHECK(frameXmlOwns(UiElement::BagBar));
    CHECK(frameXmlOwns(UiElement::MicroMenu));
}

TEST_CASE("Owning everything means suppressing nothing", "[takeover]") {
    REQUIRE(kCleanEnvironment);
    // The handover has two halves and the case above only pins one. Suppression
    // is what stops FrameXML's own frames being drawn while this client draws
    // its version, so with every element owned the list must be empty.
    //
    // The failure this guards is not subtle in effect and is entirely silent in
    // cause: the renderer sets shown = false on every name in that list, every
    // frame, so a suppression entry surviving for an element we own means the
    // panel simply never appears - with nothing logged, because suppressing a
    // frame is not an error.
    std::string suppressed;
    for (const std::string& name : frameXmlSuppressedFrames()) {
        suppressed += name;
        suppressed += ' ';
    }
    INFO("hidden despite being owned: " << suppressed);
    CHECK(suppressed.empty());
}
