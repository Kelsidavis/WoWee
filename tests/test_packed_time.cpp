// WoW packed-time unpacking — the one uint32 that carries a whole date.
//
// Four places in this client unpacked this field and no two agreed; every one
// of them was wrong. The values are what make it dangerous: a date read with
// the wrong shifts still comes out as a plausible day and month, so nothing
// looks broken until someone checks a date they know.
//
// The expectations here are built the way the server builds the field, from
// ByteBuffer::AppendPackedTime:
//
//     (tm_year - 100) << 24 | tm_mon << 20 | (tm_mday - 1) << 14
//                           | tm_wday << 11 | tm_hour << 6 | tm_min
#include <catch_amalgamated.hpp>
#include "game/packed_time.hpp"

using wowee::game::unpackWowPackedTime;

// Pack exactly as the server does, so a test failure means the reader is
// wrong rather than that two hand-written constants disagree.
static uint32_t pack(int year, int month, int day, int weekday, int hour, int minute) {
    return (static_cast<uint32_t>(year - 2000) << 24) |
           (static_cast<uint32_t>(month - 1) << 20) |
           (static_cast<uint32_t>(day - 1) << 14) |
           (static_cast<uint32_t>(weekday) << 11) |
           (static_cast<uint32_t>(hour) << 6) |
            static_cast<uint32_t>(minute);
}

TEST_CASE("packed time round-trips a date the server would send") {
    const auto d = unpackWowPackedTime(pack(2009, 11, 24, 2, 17, 43));
    CHECK(d.fullYear() == 2009);
    CHECK(d.yearSince2000 == 9);
    CHECK(d.month == 11);
    CHECK(d.day == 24);
    CHECK(d.weekday == 2);
    CHECK(d.hour == 17);
    CHECK(d.minute == 43);
}

TEST_CASE("zero is the first of January 2000, not an arbitrary date") {
    // Day and month are stored zero-based, so an unset field has to come back
    // as the 1st of the 1st rather than as day zero of month zero.
    const auto d = unpackWowPackedTime(0);
    CHECK(d.day == 1);
    CHECK(d.month == 1);
    CHECK(d.fullYear() == 2000);
    CHECK(d.hour == 0);
    CHECK(d.minute == 0);
}

TEST_CASE("each field stays inside its own bits") {
    // The failure mode of every wrong reader in this codebase was a field
    // bleeding into its neighbour, so set one at a time to its maximum and
    // check nothing else moves.
    SECTION("minute uses six bits and does not reach the hour") {
        const auto d = unpackWowPackedTime(pack(2000, 1, 1, 0, 0, 59));
        CHECK(d.minute == 59);
        CHECK(d.hour == 0);
    }
    SECTION("hour uses five bits and does not reach the weekday") {
        const auto d = unpackWowPackedTime(pack(2000, 1, 1, 0, 23, 0));
        CHECK(d.hour == 23);
        CHECK(d.weekday == 0);
        CHECK(d.minute == 0);
    }
    SECTION("day of month does not reach the month") {
        const auto d = unpackWowPackedTime(pack(2000, 1, 31, 0, 0, 0));
        CHECK(d.day == 31);
        CHECK(d.month == 1);
    }
    SECTION("month does not reach the year") {
        const auto d = unpackWowPackedTime(pack(2000, 12, 1, 0, 0, 0));
        CHECK(d.month == 12);
        CHECK(d.yearSince2000 == 0);
    }
    SECTION("a full field set at once keeps every part") {
        const auto d = unpackWowPackedTime(pack(2010, 12, 31, 6, 23, 59));
        CHECK(d.fullYear() == 2010);
        CHECK(d.month == 12);
        CHECK(d.day == 31);
        CHECK(d.weekday == 6);
        CHECK(d.hour == 23);
        CHECK(d.minute == 59);
    }
}

TEST_CASE("a wrong reading is plausible, which is why this is tested") {
    // The guild creation date read three separate uint32s where the server
    // sends one packed field. This is the value a guild made on 2 March 2005
    // arrives as; under the old reading its first word alone was the "day".
    const uint32_t packed = pack(2005, 3, 2, 3, 9, 15);
    const auto d = unpackWowPackedTime(packed);
    CHECK(d.day == 2);
    CHECK(d.month == 3);
    CHECK(d.fullYear() == 2005);
    // And the shifts that were being used elsewhere do not produce it.
    CHECK(static_cast<int>((packed >> 17) & 0x1F) != d.day);
    CHECK(static_cast<int>(packed & 0xFFFF) != d.fullYear());
}
