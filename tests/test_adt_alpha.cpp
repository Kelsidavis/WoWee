// MCAL alpha maps, and the row and column at the end of one that are not data.
//
// A chunk's alpha map is stored 64 texels wide and only 63 of them are
// painted: the file's last row and column carry whatever was left in them, and
// the client fills them from the row and column before. Left alone they are a
// strip of something else along two of a chunk's four sides, which puts a hard
// line between every pair of chunks - the ground textures not lining up, in a
// grid across the whole world.
//
// The chunks here are built by hand, so this needs no ADT, no device and no
// world.
#include <catch_amalgamated.hpp>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <vector>

#include "pipeline/adt_alpha.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/wdt_loader.hpp"

using wowee::pipeline::ALPHA_MAP_DIM;
using wowee::pipeline::ALPHA_MAP_PACKED;
using wowee::pipeline::ALPHA_MAP_SIZE;
using wowee::pipeline::decodeLayerAlpha;
using wowee::pipeline::MapChunk;

namespace {

/// A chunk with one base layer and one alpha layer, whose four-bit map is
/// filled by `nibbleAt(x, y)`.
template <typename F>
MapChunk packedChunk(F nibbleAt) {
    MapChunk chunk;
    chunk.layers.resize(2);
    // Layer 0 is the base and never has a map; layer 1's begins at zero.
    chunk.layers[1].flags = 0x100;   // use_alpha_map
    chunk.layers[1].offsetMCAL = 0;

    chunk.alphaMap.assign(ALPHA_MAP_PACKED, 0);
    for (size_t y = 0; y < ALPHA_MAP_DIM; ++y) {
        for (size_t x = 0; x < ALPHA_MAP_DIM; x += 2) {
            const uint8_t lo = static_cast<uint8_t>(nibbleAt(x, y) & 0x0F);
            const uint8_t hi = static_cast<uint8_t>(nibbleAt(x + 1, y) & 0x0F);
            chunk.alphaMap[(y * ALPHA_MAP_DIM + x) / 2] =
                static_cast<uint8_t>(lo | (hi << 4));
        }
    }
    return chunk;
}

uint8_t at(const std::vector<uint8_t>& alpha, size_t x, size_t y) {
    return alpha[y * ALPHA_MAP_DIM + x];
}

}  // namespace

TEST_CASE("a four-bit alpha map unpacks low nibble first", "[adt][alpha]") {
    // Every texel its own column number, so an unpack that swapped the two
    // halves of a byte would show immediately.
    const MapChunk chunk = packedChunk([](size_t x, size_t) { return x % 16; });
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 0));
    REQUIRE(alpha.size() == ALPHA_MAP_SIZE);
    CHECK(at(alpha, 0, 0) == 0);
    CHECK(at(alpha, 1, 0) == 17);
    CHECK(at(alpha, 5, 3) == 5 * 17);
}

TEST_CASE("the last row and column are taken from the ones before them",
          "[adt][alpha]") {
    // 15 everywhere the file paints, and 0 in the row and column it does not -
    // which is the shape of the fault: a strip of nothing along two sides.
    const MapChunk chunk = packedChunk([](size_t x, size_t y) {
        const bool unpainted = (x == ALPHA_MAP_DIM - 1) || (y == ALPHA_MAP_DIM - 1);
        return unpainted ? 0 : 15;
    });
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 0));

    // The strip is gone: the edge reads as what the texel inside it reads.
    for (size_t i = 0; i < ALPHA_MAP_DIM; ++i) {
        INFO("column " << i);
        CHECK(at(alpha, i, ALPHA_MAP_DIM - 1) == at(alpha, i, ALPHA_MAP_DIM - 2));
    }
    for (size_t i = 0; i < ALPHA_MAP_DIM; ++i) {
        INFO("row " << i);
        CHECK(at(alpha, ALPHA_MAP_DIM - 1, i) == at(alpha, ALPHA_MAP_DIM - 2, i));
    }
    // And what it reads is the painted value, not the leftover.
    CHECK(at(alpha, ALPHA_MAP_DIM - 1, ALPHA_MAP_DIM - 1) == 15 * 17);
}

TEST_CASE("a layer with no map of its own is left as the caller asked",
          "[adt][alpha]") {
    MapChunk chunk;
    chunk.layers.resize(1);   // the base only
    std::vector<uint8_t> alpha;
    CHECK_FALSE(decodeLayerAlpha(chunk, 0, alpha, 255));
    REQUIRE(alpha.size() == ALPHA_MAP_SIZE);
    CHECK(at(alpha, 10, 10) == 255);
}

TEST_CASE("compressed alpha counts are exact and preserve row boundaries", "[adt][alpha]") {
    MapChunk chunk;
    chunk.layers.resize(2);
    chunk.layers[1].flags = 0x100 | 0x200;
    // Two different flat rows, as produced by Blizzard's row-limited RLE.
    for (size_t y = 0; y < ALPHA_MAP_DIM; ++y) {
        chunk.alphaMap.push_back(0x80 | 64);
        chunk.alphaMap.push_back(static_cast<uint8_t>(y * 4));
    }
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 0));
    for (size_t y = 0; y < ALPHA_MAP_DIM; ++y) {
        for (size_t x = 0; x < ALPHA_MAP_DIM; ++x) {
            CHECK(at(alpha, x, y) == y * 4);
        }
    }
}

TEST_CASE("compressed alpha handles literals, empty runs and layer boundaries", "[adt][alpha]") {
    MapChunk chunk;
    chunk.layers.resize(3);
    chunk.layers[1].flags = 0x100 | 0x200;
    chunk.layers[2].flags = 0x100 | 0x200;
    // A zero-length copy consumes no value; a zero-length fill still has one
    // value byte. Layer 1 then ends partway through a literal; the next
    // layer's data must never be consumed to complete it.
    chunk.alphaMap = {0x00, 0x80, 88, 3, 11, 22, 33, 0x82, 44, 3, 55};
    chunk.layers[2].offsetMCAL = static_cast<uint32_t>(chunk.alphaMap.size());
    chunk.alphaMap.insert(chunk.alphaMap.end(), {0x84, 99});
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 7));
    CHECK(alpha[0] == 11);
    CHECK(alpha[1] == 22);
    CHECK(alpha[2] == 33);
    CHECK(alpha[3] == 44);
    CHECK(alpha[4] == 44);
    CHECK(alpha[5] == 55);
    CHECK(alpha[6] == 7);
    REQUIRE(decodeLayerAlpha(chunk, 2, alpha, 7));
    CHECK(alpha[0] == 99);
    CHECK(alpha[3] == 99);
    CHECK(alpha[4] == 7);
}

TEST_CASE("fixed four-bit alpha preserves the authored last row and column", "[adt][alpha]") {
    MapChunk chunk = packedChunk([](size_t x, size_t y) {
        return (x == ALPHA_MAP_DIM - 1 || y == ALPHA_MAP_DIM - 1) ? 0 : 15;
    });
    chunk.flags = 0x8000; // MCNK do_not_fix_alpha_map
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 0));
    CHECK(at(alpha, 62, 62) == 255);
    CHECK(at(alpha, 63, 62) == 0);
    CHECK(at(alpha, 62, 63) == 0);
}

TEST_CASE("invalid alpha layer offsets do not run past MCAL", "[adt][alpha]") {
    MapChunk chunk;
    chunk.layers.resize(3);
    chunk.layers[1].flags = chunk.layers[2].flags = 0x100;
    chunk.alphaMap.resize(ALPHA_MAP_SIZE * 2, 0);
    chunk.layers[1].offsetMCAL = ALPHA_MAP_SIZE;
    chunk.layers[2].offsetMCAL = 1;
    std::vector<uint8_t> alpha;
    CHECK_FALSE(decodeLayerAlpha(chunk, 1, alpha, 0));
    chunk.layers[2].offsetMCAL = static_cast<uint32_t>(chunk.alphaMap.size() + 1);
    CHECK_FALSE(decodeLayerAlpha(chunk, 1, alpha, 0));
}

TEST_CASE("WDT flags select the terrain alpha format", "[adt][alpha][wdt]") {
    wowee::pipeline::WDTInfo wdt;
    CHECK_FALSE(wdt.hasBigAlpha());
    wdt.mphdFlags = 0x04;
    CHECK(wdt.hasBigAlpha());
    wdt.mphdFlags = 0x80;
    CHECK(wdt.hasBigAlpha());
    wdt.mphdFlags = 0x02;
    CHECK_FALSE(wdt.hasBigAlpha());

    // MVER 18 followed by an MPHD with the actual Northrend flags. This also
    // exercises parsing, not only the bit test on an already-populated struct.
    std::vector<uint8_t> file = {
        'R', 'E', 'V', 'M', 4, 0, 0, 0, 18, 0, 0, 0,
        'D', 'H', 'P', 'M', 32, 0, 0, 0, 0x0e, 0, 0, 0,
    };
    file.resize(12 + 8 + 32, 0);
    const auto parsed = wowee::pipeline::parseWDT(file);
    CHECK(parsed.mphdFlags == 0x0e);
    CHECK(parsed.hasBigAlpha());
}

TEST_CASE("Icecrown's compressed terrain preserves the authored alpha map", "[adt][alpha][assets]") {
    // Optional regression against the build 12340 Icecrown tile. The expected
    // digest was derived independently from MCAL's documented RLE.
    const char* path = std::getenv("WOWEE_ICECROWN_ADT");
    if (!path || !*path) SKIP("Set WOWEE_ICECROWN_ADT to northrend_31_16.adt");
    std::ifstream stream(path, std::ios::binary);
    REQUIRE(stream.is_open());
    const std::vector<uint8_t> data{std::istreambuf_iterator<char>(stream), {}};
    const auto terrain = wowee::pipeline::ADTLoader::load(data);
    REQUIRE(terrain.isLoaded());
    const auto& chunk = terrain.getChunk(5, 4);
    REQUIRE(chunk.layers.size() == 4);
    REQUIRE(chunk.layers[1].compressedAlpha());
    std::vector<uint8_t> alpha;
    REQUIRE(decodeLayerAlpha(chunk, 1, alpha, 0));
    uint64_t hash = 14695981039346656037ull;
    for (const uint8_t value : alpha) hash = (hash ^ value) * 1099511628211ull;
    CHECK(hash == 0x8dec39b19c1e622full);
    CHECK(at(alpha, 0, 0) == 255);
    CHECK(at(alpha, 63, 0) == 0);
    CHECK(at(alpha, 63, 63) == 7);
}
