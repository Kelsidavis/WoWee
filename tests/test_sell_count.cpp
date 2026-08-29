#include <catch_amalgamated.hpp>

#include "game/sell_count.hpp"

using wowee::game::sellCountForStack;

// All four vendor sell sites asked for exactly one unit while the slot's own
// stack count was in hand, so selling a stack of twenty sold one and left
// nineteen - and the auto-sell sweep charged one unit's price into the total it
// reported and never came back to the slot. 3.3.5 sells the whole stack on a
// right-click, CMSG_SELL_ITEM carries the count, and the server clamps it.
TEST_CASE("a vendor sell asks for the whole stack", "[vendor][inventory]") {
    CHECK(sellCountForStack(20) == 20u);
    CHECK(sellCountForStack(5) == 5u);
    CHECK(sellCountForStack(1) == 1u);

    // A slot that never said how many it holds still holds one - answering
    // zero would sell nothing at all, which is worse than selling one.
    CHECK(sellCountForStack(0) == 1u);
}
