#include <catch_amalgamated.hpp>

#include "game/quest_progress.hpp"

using wowee::game::decodeQuestObjectiveCounts;
using wowee::game::normalizeQuestObjectiveEntry;
using wowee::game::isQuestSlotComplete;
using wowee::game::questObjectiveCountFieldOffset;
using wowee::game::questObjectiveLine;

TEST_CASE("Classic quest counters use four packed 6-bit values", "[quest][progress]") {
    const uint32_t packed = 5u | (12u << 6) | (31u << 12) | (63u << 18)
                          | (1u << 24); // state byte must not leak into counts
    REQUIRE(decodeQuestObjectiveCounts(3, packed) ==
            std::array<uint32_t, 4>{5, 12, 31, 63});
    REQUIRE(questObjectiveCountFieldOffset(3) == 1);
}

TEST_CASE("TBC quest counters use four byte values", "[quest][progress]") {
    const uint32_t packed = 7u | (16u << 8) | (42u << 16) | (200u << 24);
    REQUIRE(decodeQuestObjectiveCounts(4, packed) ==
            std::array<uint32_t, 4>{7, 16, 42, 200});
    REQUIRE(questObjectiveCountFieldOffset(4) == 2);
}

TEST_CASE("WotLK quest counters use four uint16 values", "[quest][progress]") {
    const uint32_t first = 5u | (300u << 16);
    const uint32_t second = 16u | (1024u << 16);
    REQUIRE(decodeQuestObjectiveCounts(5, first, second) ==
            std::array<uint32_t, 4>{5, 300, 16, 1024});
    REQUIRE(questObjectiveCountFieldOffset(5) == 2);
}

TEST_CASE("Quest update game-object entries discard the wire marker", "[quest][progress]") {
    REQUIRE(normalizeQuestObjectiveEntry(1234u) == 1234u);
    REQUIRE(normalizeQuestObjectiveEntry(0x80000000u | 5678u) == 5678u);
}

TEST_CASE("Classic completion state does not alias the first kill count", "[quest][progress]") {
    const uint32_t oneKillNotComplete = 1u;
    const uint32_t oneKillComplete = 1u | (1u << 24);
    REQUIRE_FALSE(isQuestSlotComplete(3, oneKillNotComplete));
    REQUIRE(isQuestSlotComplete(3, oneKillComplete));
    REQUIRE(isQuestSlotComplete(4, 1u));
    REQUIRE(isQuestSlotComplete(5, 1u));
}

// The bit beside completion, in the same field.
//
// The server names them QUEST_STATE_COMPLETE = 0x0001 and QUEST_STATE_FAIL =
// 0x0002 (Player.h:619). The complete bit was read for as long as the quest
// log has existed and the fail bit next to it was dropped, so a timed quest
// that ran out looked exactly like one still running - and the tracker, which
// writes `if ( isComplete and isComplete < 0 )`, had no way to reach that
// branch.
//
// Both strides, because Classic packs the state into the top byte of a shared
// field and TBC/WotLK give it one of its own; reading the wrong end answers
// zero for everything, which is indistinguishable from nothing being failed.
TEST_CASE("A failed quest slot reads as failed, not as unfinished",
          "[quest][progress]") {
    SECTION("WotLK and TBC keep the state in its own field") {
        const uint8_t stride = 5;
        CHECK_FALSE(wowee::game::isQuestSlotFailed(stride, 0x0));
        CHECK_FALSE(wowee::game::isQuestSlotFailed(stride, 0x1));   // complete
        CHECK(wowee::game::isQuestSlotFailed(stride, 0x2));         // failed
        CHECK(wowee::game::isQuestSlotFailed(stride, 0x3));         // both
        // And completion still reads the way it did.
        CHECK(wowee::game::isQuestSlotComplete(stride, 0x1));
        CHECK_FALSE(wowee::game::isQuestSlotComplete(stride, 0x2));
        CHECK(wowee::game::isQuestSlotComplete(stride, 0x3));
    }
    SECTION("Classic packs it into the top byte") {
        const uint8_t stride = 3;
        CHECK_FALSE(wowee::game::isQuestSlotFailed(stride, 0x00000002));
        CHECK(wowee::game::isQuestSlotFailed(stride, 0x02000000));
        CHECK(wowee::game::isQuestSlotComplete(stride, 0x01000000));
        CHECK(wowee::game::isQuestSlotFailed(stride, 0x03000000));
    }
}

// An objective line names what it is about. Both the quest log and the tracker
// read this one string, and it used to say "Creature slain: 12/15" for every
// kill objective at once - fifteen of what, the line could not say.
TEST_CASE("A kill objective is named, and counted", "[quest][objective]") {
    REQUIRE(questObjectiveLine("Nightbane Worgen", false, 12, 15) ==
            "Nightbane Worgen slain: 12/15");
    // A game object is found rather than slain, which is the game's own
    // QUEST_OBJECTS_FOUND against its QUEST_MONSTERS_KILLED.
    REQUIRE(questObjectiveLine("Bundle of Wood", true, 0, 8) ==
            "Bundle of Wood: 0/8");
}

TEST_CASE("An objective with no name yet still counts", "[quest][objective]") {
    // The creature query has not answered. The count is real and belongs on
    // screen now; the name arrives behind it and rewrites the line.
    REQUIRE(questObjectiveLine("", false, 3, 8) == "Creature slain: 3/8");
    REQUIRE(questObjectiveLine("", true, 3, 8) == "Object: 3/8");
}
