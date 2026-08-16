#pragma once

#include <cstdint>

namespace wowee {
namespace game {

/**
 * A date the server sent as a WoW "packed time" - one uint32 holding a whole
 * calendar time in bitfields.
 *
 * The layout is fixed by the server's ByteBuffer::AppendPackedTime:
 *
 *     (tm_year - 100) << 24 | tm_mon << 20 | (tm_mday - 1) << 14
 *                           | tm_wday << 11 | tm_hour << 6 | tm_min
 *
 * so, from the bottom: minute 0-5, hour 6-10, weekday 11-13, day-of-month 14-19
 * (zero-based), month 20-23 (zero-based), and year 24-31 counted from 2000.
 *
 * This exists because four places in this client unpacked it and no two agreed
 * - shifts of 3/9/17/21/25 in one, a low-sixteen-bit year in two others, and
 * three separate uint32s in the guild parser, which also left everything after
 * the date in that packet misread. Every one of them was wrong. The format is
 * not guessable from the values (a date in 2009 is a plausible-looking number
 * under several readings), so it belongs in one place with a test.
 */
struct WowDate {
    int minute = 0;         // 0-59
    int hour = 0;           // 0-23
    int weekday = 0;        // 0-6, Sunday first
    int day = 1;            // 1-31
    int month = 1;          // 1-12
    int yearSince2000 = 0;  // 0-255 - add 2000 for a full year

    /// The four-digit year. FrameXML wants the short form instead: SHORTDATE
    /// formats it "%02d", so GetAchievementInfo returns yearSince2000.
    [[nodiscard]] constexpr int fullYear() const { return yearSince2000 + 2000; }
};

/// Unpack a server packed-time field. A zero packs to 1 January 2000, which is
/// how "no date" arrives and reads as obviously unset rather than as today.
constexpr WowDate unpackWowPackedTime(uint32_t packed) {
    WowDate d;
    d.minute        = static_cast<int>( packed        & 0x3F);
    d.hour          = static_cast<int>((packed >>  6) & 0x1F);
    d.weekday       = static_cast<int>((packed >> 11) & 0x07);
    d.day           = static_cast<int>((packed >> 14) & 0x3F) + 1;
    d.month         = static_cast<int>((packed >> 20) & 0x0F) + 1;
    d.yearSince2000 = static_cast<int>((packed >> 24) & 0xFF);
    return d;
}

/// Pack a date back into the server's uint32, the exact inverse of the above.
///
/// Needed to *send* a date rather than read one - creating a calendar event
/// writes two of these. Every field is masked to its width, so a caller that
/// hands over a month of 13 writes a wrong date rather than corrupting the
/// fields above it.
constexpr uint32_t packWowPackedTime(const WowDate& d) {
    const uint32_t minute  = static_cast<uint32_t>(d.minute)        & 0x3Fu;
    const uint32_t hour    = static_cast<uint32_t>(d.hour)          & 0x1Fu;
    const uint32_t weekday = static_cast<uint32_t>(d.weekday)       & 0x07u;
    const uint32_t day     = static_cast<uint32_t>(d.day - 1)       & 0x3Fu;
    const uint32_t month   = static_cast<uint32_t>(d.month - 1)     & 0x0Fu;
    const uint32_t year    = static_cast<uint32_t>(d.yearSince2000) & 0xFFu;
    return minute | (hour << 6) | (weekday << 11) | (day << 14) |
           (month << 20) | (year << 24);
}

}  // namespace game
}  // namespace wowee
