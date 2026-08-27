// UNIT_NPC_FLAGS, bit by bit.
//
// A wrong one here is invisible: the field arrives, the mask is applied, and
// the answer is simply always no. NPC_FLAG_VENDOR was 0x04 - one of the two
// bits in the run that nothing sets - so no merchant in the game ever read as
// one and the vendor cursor never drew, twice reported as "the cursor does not
// change over a vendor" with the code that draws it working correctly.
//
// The values are 3.3.5a's NPCFlags enum, which is a plain run of bits: gossip,
// questgiver, two unused, three trainers, five vendors, repair, the flight
// master, the two spirits, then the rest. Written out so the run is visible -
// a gap or a swap in it is the whole failure mode.
#include <catch_amalgamated.hpp>

#include "game/protocol_constants.hpp"

using namespace wowee::game;

TEST_CASE("the npc flag run has no gaps or swaps", "[npc][flags]") {
    CHECK(NPC_FLAG_GOSSIP         == 0x00000001u);
    CHECK(NPC_FLAG_QUESTGIVER     == 0x00000002u);
    // 0x04 and 0x08 are unused. Nothing may claim them.
    CHECK(NPC_FLAG_TRAINER        == 0x00000010u);
    // 0x20 trainer-class, 0x40 trainer-profession.
    CHECK(NPC_FLAG_VENDOR         == 0x00000080u);
    CHECK(NPC_FLAG_VENDOR_AMMO    == 0x00000100u);
    CHECK(NPC_FLAG_VENDOR_FOOD    == 0x00000200u);
    CHECK(NPC_FLAG_VENDOR_POISON  == 0x00000400u);
    CHECK(NPC_FLAG_VENDOR_REAGENT == 0x00000800u);
    CHECK(NPC_FLAG_REPAIR         == 0x00001000u);
    CHECK(NPC_FLAG_FLIGHT_MASTER  == 0x00002000u);
    // The healer first. These were the other way round, which no caller could
    // see because both are only ever tested as a pair.
    CHECK(NPC_FLAG_SPIRIT_HEALER  == 0x00004000u);
    CHECK(NPC_FLAG_SPIRIT_GUIDE   == 0x00008000u);
    CHECK(NPC_FLAG_INNKEEPER      == 0x00010000u);
    CHECK(NPC_FLAG_BANKER         == 0x00020000u);
    CHECK(NPC_FLAG_AUCTIONEER     == 0x00200000u);
}

TEST_CASE("every vendor kind counts as a vendor", "[npc][flags]") {
    // A food or reagent seller usually carries the plain bit too, and a few
    // carry only their own - so the cursor asks about all five.
    CHECK((NPC_FLAG_ANY_VENDOR & NPC_FLAG_VENDOR)         != 0u);
    CHECK((NPC_FLAG_ANY_VENDOR & NPC_FLAG_VENDOR_AMMO)    != 0u);
    CHECK((NPC_FLAG_ANY_VENDOR & NPC_FLAG_VENDOR_FOOD)    != 0u);
    CHECK((NPC_FLAG_ANY_VENDOR & NPC_FLAG_VENDOR_POISON)  != 0u);
    CHECK((NPC_FLAG_ANY_VENDOR & NPC_FLAG_VENDOR_REAGENT) != 0u);

    // And nothing that is not a vendor does.
    for (uint32_t other : {NPC_FLAG_GOSSIP, NPC_FLAG_QUESTGIVER, NPC_FLAG_TRAINER,
                           NPC_FLAG_REPAIR, NPC_FLAG_FLIGHT_MASTER,
                           NPC_FLAG_SPIRIT_HEALER, NPC_FLAG_SPIRIT_GUIDE,
                           NPC_FLAG_INNKEEPER, NPC_FLAG_BANKER,
                           NPC_FLAG_AUCTIONEER}) {
        CHECK((NPC_FLAG_ANY_VENDOR & other) == 0u);
    }
}
