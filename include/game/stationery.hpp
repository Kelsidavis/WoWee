#pragma once

#include <cstdint>

namespace wowee {
namespace game {

/// The letterhead a letter is written on, by its Stationery.dbc id.
///
/// FrameXML appends "1" and "2" to this and hangs the two halves behind the
/// mail body: Interface\Stationery\StationeryTest1 and ...2. Answering nil
/// left both halves empty, so every letter was read as dark ink on the black
/// of an empty frame - the font is a dark brown chosen to sit on parchment,
/// and there was no parchment.
///
/// Seven rows, from 3.3.5's own Stationery.dbc, and each name is what the
/// .blp beside it is called. An id not in the table falls back on what kind of
/// mail it is rather than on nothing: a plausible letterhead is right far more
/// often than no letterhead is, and no letterhead is unreadable.
inline const char* stationeryTexture(uint32_t stationeryId, uint8_t messageType) {
    switch (stationeryId) {
        case 1:  case 41: return "StationeryTest";
        case 61:          return "GMStationery";
        case 62:          return "AuctionStationery";
        case 64:          return "Stationery_Val";
        case 65:          return "Stationery_Chr";
        case 67:          return "Stationery_Orp";
        default: break;
    }
    // 2 is an auction house letter, which has its own paper in every client.
    return messageType == 2 ? "AuctionStationery" : "StationeryTest";
}

} // namespace game
} // namespace wowee
