// The scale that makes a font height mean what FrameXML means by it.
//
// <FontHeight><AbsValue val="10"/></FontHeight> is an em size, the way every
// traditional font API means "font size". ImGui asks stb_truetype for
// stbtt_ScaleForPixelHeight, which fits the whole ascender-to-descender span
// into those ten pixels instead - stb's own header calls the em mapping
// "probably what traditional APIs compute". FRIZQT__ spans 1215 units of a
// 1000-unit em, so the interface was drawn and measured at 82.3% of WoW's size,
// and everything anchored to a caption's right edge landed short by 18% of that
// caption's width.
//
// The correction is one ratio read off the font's own head and hhea tables, and
// what is pinned here is the reading of them: the offsets are absolute byte
// positions into a file this client did not write, and a table directory names
// its tables in any order at any offset.
//
// Built in memory rather than read off disk. The real faces are not in the
// repository - Data/* is ignored - so a test that opened one would pass on the
// machine that has the game and be skipped everywhere else.
#include <catch_amalgamated.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "ui/interface_fonts.hpp"

using wowee::ui::fontEmSizeScale;

namespace {

void be16(std::vector<uint8_t>& out, size_t at, uint16_t v) {
    out[at]     = static_cast<uint8_t>(v >> 8);
    out[at + 1] = static_cast<uint8_t>(v & 0xFF);
}

void be32(std::vector<uint8_t>& out, size_t at, uint32_t v) {
    for (int i = 0; i < 4; ++i) out[at + static_cast<size_t>(i)] =
        static_cast<uint8_t>(v >> ((3 - i) * 8));
}

/// A font file with nothing in it but the two tables the ratio comes from.
///
/// head is placed after hhea on purpose. The directory is what says where each
/// one is, and a reader that assumed the order would still pass on a file that
/// happened to use it.
std::vector<uint8_t> makeFont(uint16_t unitsPerEm, int16_t ascent, int16_t descent) {
    constexpr size_t kDirEnd = 12 + 2 * 16;
    constexpr size_t kHheaAt = kDirEnd;
    constexpr size_t kHheaLen = 36;
    constexpr size_t kHeadAt = kHheaAt + kHheaLen;
    constexpr size_t kHeadLen = 54;

    std::vector<uint8_t> f(kHeadAt + kHeadLen, 0);
    be32(f, 0, 0x00010000u);
    be16(f, 4, 2);                       // two tables

    be32(f, 12, 0x68686561u);            // 'hhea'
    be32(f, 20, static_cast<uint32_t>(kHheaAt));
    be32(f, 24, static_cast<uint32_t>(kHheaLen));
    be32(f, 28, 0x68656164u);            // 'head'
    be32(f, 36, static_cast<uint32_t>(kHeadAt));
    be32(f, 40, static_cast<uint32_t>(kHeadLen));

    be16(f, kHheaAt + 4, static_cast<uint16_t>(ascent));
    be16(f, kHheaAt + 6, static_cast<uint16_t>(descent));
    be16(f, kHeadAt + 18, unitsPerEm);
    return f;
}

float scaleOf(const std::vector<uint8_t>& f) {
    return fontEmSizeScale(f.data(), f.size());
}

}  // namespace

TEST_CASE("the ratio is the vertical span over the em") {
    // FRIZQT__'s own numbers, which are what the auction house's rarity
    // dropdown was landing 10 pixels short by.
    const float s = scaleOf(makeFont(1000, 965, -250));
    CHECK(s == Catch::Approx(1.215f).epsilon(0.0001));
}

TEST_CASE("a face whose span is its em needs no correction") {
    CHECK(scaleOf(makeFont(2048, 1638, -410)) == Catch::Approx(1.0f).epsilon(0.0001));
}

TEST_CASE("the tables are found wherever the directory puts them") {
    // Same font, one table pushed far down the file with a gap before it. The
    // offsets that matter are the ones inside each table, and they are counted
    // from where the directory says the table starts.
    std::vector<uint8_t> f = makeFont(1000, 965, -250);
    const size_t moved = f.size() + 4096;
    f.resize(moved + 54, 0);
    be32(f, 36, static_cast<uint32_t>(moved));   // head's record: new offset
    be16(f, moved + 18, 1000);
    CHECK(scaleOf(f) == Catch::Approx(1.215f).epsilon(0.0001));
}

TEST_CASE("anything unreadable leaves ImGui's own scaling alone") {
    CHECK(fontEmSizeScale(nullptr, 0) == 1.0f);

    const std::vector<uint8_t> tiny(8, 0);
    CHECK(scaleOf(tiny) == 1.0f);

    // A directory naming tables past the end of the file.
    std::vector<uint8_t> truncated = makeFont(1000, 965, -250);
    truncated.resize(40);
    CHECK(scaleOf(truncated) == 1.0f);

    // Present and nonsense: a zero em would divide by nothing.
    CHECK(scaleOf(makeFont(0, 965, -250)) == 1.0f);
    // And a span ten times its em is a misread, not a design.
    CHECK(scaleOf(makeFont(1000, 5000, -5000)) == 1.0f);
}

TEST_CASE("a collection is read through to its first face") {
    std::vector<uint8_t> face = makeFont(1000, 965, -250);
    std::vector<uint8_t> ttc(16, 0);
    be32(ttc, 0, 0x74746366u);                    // 'ttcf'
    be32(ttc, 8, 1);                              // one face
    be32(ttc, 12, static_cast<uint32_t>(ttc.size()));
    ttc.insert(ttc.end(), face.begin(), face.end());
    // The face's own table offsets are counted from the file, so shift them.
    const uint32_t base = 16;
    be32(ttc, base + 20, base + 12 + 2 * 16);
    be32(ttc, base + 36, base + 12 + 2 * 16 + 36);
    CHECK(scaleOf(ttc) == Catch::Approx(1.215f).epsilon(0.0001));
}
