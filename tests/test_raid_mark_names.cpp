// The raid target markers, named once and read three ways.
//
// The eight words were written out capitalised for the minimap's tooltip,
// lowercase for what /mark matches, and a third time inside that command's
// "Unknown mark. Use: ..." line. They now come from one table, lowercased
// where a lowercase word is wanted - so what /mark accepts is no longer a
// literal anyone can see, and this is what pins it.
//
// The count went the same way: GameHandler, SocialHandler and the icon
// loader each declared their own 8.
#include <catch_amalgamated.hpp>

#include <cstdint>
#include <string>

#include "game/group_defines.hpp"
#include "ui/chat/chat_utils.hpp"

using wowee::game::instanceDifficultyName;
using wowee::game::kRaidMarkCount;
using wowee::game::raidMarkName;
using wowee::ui::chat_utils::toLower;

TEST_CASE("/mark accepts the words it always accepted", "[raid-marks]") {
    // The list the command matched against before the names were shared.
    const char* kWords[] = {"star", "circle", "diamond", "triangle",
                            "moon", "square", "cross", "skull"};
    REQUIRE(kRaidMarkCount == 8);
    for (uint32_t i = 0; i < kRaidMarkCount; ++i) {
        REQUIRE(raidMarkName(static_cast<uint8_t>(i)) != nullptr);
        CHECK(toLower(raidMarkName(static_cast<uint8_t>(i))) == kWords[i]);
    }
}

TEST_CASE("a mark the server does not have has no name", "[raid-marks]") {
    // 0xFF is what getEntityRaidMark answers for an unmarked unit, and the
    // tooltip asks for its name rather than testing the value first.
    CHECK(raidMarkName(static_cast<uint8_t>(kRaidMarkCount)) == nullptr);
    CHECK(raidMarkName(0xFF) == nullptr);
}

TEST_CASE("a difficulty outside the four has no name", "[raid-marks]") {
    // Same shape, same header: the callers print the label only when there
    // is one, rather than each deciding what an unknown difficulty is called.
    CHECK(std::string(instanceDifficultyName(0)) == "Normal");
    CHECK(std::string(instanceDifficultyName(3)) == "25 Heroic");
    CHECK(instanceDifficultyName(4) == nullptr);
}
