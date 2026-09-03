// The letterhead behind a letter, by its Stationery.dbc id.
//
// The mail font is a dark brown chosen to sit on parchment. With no
// stationery answered, FrameXML hangs nothing behind the body and the letter
// is dark brown on black - which is what every letter looked like. These are
// the seven rows of 3.3.5's Stationery.dbc, each naming the .blp that has to
// be found under Interface\Stationery.
#include <catch_amalgamated.hpp>

#include <string>

#include "game/stationery.hpp"

using wowee::game::stationeryTexture;

TEST_CASE("each stationery id names its own art", "[mail]") {
    CHECK(std::string(stationeryTexture(1, 0))  == "StationeryTest");
    CHECK(std::string(stationeryTexture(41, 0)) == "StationeryTest");
    CHECK(std::string(stationeryTexture(61, 0)) == "GMStationery");
    CHECK(std::string(stationeryTexture(62, 0)) == "AuctionStationery");
    CHECK(std::string(stationeryTexture(64, 0)) == "Stationery_Val");
    CHECK(std::string(stationeryTexture(65, 0)) == "Stationery_Chr");
    CHECK(std::string(stationeryTexture(67, 0)) == "Stationery_Orp");
}

TEST_CASE("an id with no row still gets a letterhead", "[mail]") {
    // The point of the fallback: never nothing. A letter from a server that
    // sends an id this client has no row for still has to be readable.
    CHECK(std::string(stationeryTexture(0, 0)) == "StationeryTest");
    CHECK(std::string(stationeryTexture(9999, 0)) == "StationeryTest");
    // Message type 2 is the auction house, which has its own paper.
    CHECK(std::string(stationeryTexture(0, 2)) == "AuctionStationery");
}
