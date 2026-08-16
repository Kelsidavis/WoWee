// The spam filter the Social panel offers.
//
// Easy to write in a way that looks right and is wrong in one direction only:
// a filter that eats two people greeting each other. The mature language
// filter that used to sit beside it is gone - chat arrives as it was sent.
#include <catch_amalgamated.hpp>
#include "game/chat_filters.hpp"

using namespace wowee::game;

namespace {
constexpr uint64_t kAlice = 0x11;
constexpr uint64_t kBob   = 0x22;

std::deque<RecentChatLine> history() {
    return {
        {kAlice, "WTS [Thunderfury] pst", 100.0},
        {kBob,   "hi", 100.0},
    };
}
}  // namespace

TEST_CASE("the same sender pasting the same line again is spam", "[chatfilter]") {
    CHECK(repeatsRecentLine(history(), kAlice, "WTS [Thunderfury] pst", 110.0));
    // Case and spacing changed is the cheapest way past a filter that only
    // compares raw text.
    CHECK(repeatsRecentLine(history(), kAlice, "wts  [Thunderfury]   PST ", 110.0));
}

TEST_CASE("two people saying the same thing is not spam", "[chatfilter]") {
    // Bob said "hi". Alice saying "hi" is a conversation.
    CHECK_FALSE(repeatsRecentLine(history(), kAlice, "hi", 110.0));
}

TEST_CASE("the same line long enough later is not spam", "[chatfilter]") {
    CHECK_FALSE(repeatsRecentLine(history(), kAlice, "WTS [Thunderfury] pst", 200.0));
}

TEST_CASE("a line with no sender is never spam", "[chatfilter]") {
    // System lines and anything the server sends unattributed.
    CHECK_FALSE(repeatsRecentLine(history(), 0, "WTS [Thunderfury] pst", 110.0));
}
