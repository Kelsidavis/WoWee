// What the BLP loader does with a texture that lies about its own sizes.
//
// A BLP header states width, height, a compression format, and a mip offset
// and size. Nothing makes those agree. The decompressors took a bare source
// pointer and derived how much to read from width and height:
//
//   void decompressDXT1(const uint8_t* src, uint8_t* dst, int w, int h) {
//       for (each of (w/4)*(h/4) blocks)
//           const uint8_t* block = src + (by * blockWidth + bx) * 8;
//
// so a file declaring 4096x4096 DXT1 with an eight-byte mip level read two
// megabytes out of an eight-byte buffer. A comment above the call site said
// "the decompressors have their own internal bounds"; they bounded the
// destination, never the source.
//
// The mip bounds check had its own problem:
//
//   uint32_t offset = header.mipOffsets[0];
//   uint32_t mipSize = header.mipSizes[0];
//   if (offset + mipSize > size) { ... reject ... }
//
// Both operands come from the file and both are uint32, so an offset near 4G
// plus a size wraps to something small and passes.
//
// Every case here is malformed on purpose. None should read out of bounds --
// run under --asan, where an overrun fails instead of returning a value
// nobody looks at.
#include <catch_amalgamated.hpp>

#include "pipeline/blp_loader.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

using wowee::pipeline::BLPLoader;

namespace {

// BLP2: magic[4] version compression alphaDepth alphaEncoding hasMips
//       width height mipOffsets[16] mipSizes[16] palette[256]
constexpr size_t kBlp2HeaderSize = 1172;
constexpr size_t kOffCompression = 8;
constexpr size_t kOffAlphaDepth  = 9;
constexpr size_t kOffWidth       = 12;
constexpr size_t kOffHeight      = 16;
constexpr size_t kOffMipOffsets  = 20;
constexpr size_t kOffMipSizes    = 84;

void writeU32(std::vector<uint8_t>& b, size_t at, uint32_t v) {
    REQUIRE(at + sizeof(v) <= b.size());
    std::memcpy(b.data() + at, &v, sizeof(v));
}

/// A BLP2 whose header claims `width`x`height` in `compression`, with a mip
/// level of `mipSize` bytes that the file may or may not actually contain.
std::vector<uint8_t> makeBlp2(uint32_t width, uint32_t height, uint8_t compression,
                              uint32_t mipSize, uint32_t mipOffset = kBlp2HeaderSize,
                              uint8_t alphaDepth = 8, size_t trailingBytes = 0) {
    std::vector<uint8_t> b(kBlp2HeaderSize + trailingBytes, 0u);
    std::memcpy(b.data(), "BLP2", 4);
    writeU32(b, 4, 1);  // version
    b[kOffCompression] = compression;
    b[kOffAlphaDepth]  = alphaDepth;
    writeU32(b, kOffWidth, width);
    writeU32(b, kOffHeight, height);
    writeU32(b, kOffMipOffsets, mipOffset);
    writeU32(b, kOffMipSizes, mipSize);
    return b;
}

} // namespace

TEST_CASE("BLP loader survives a truncated file", "[blp][robustness]") {
    SECTION("empty input")      { REQUIRE_NOTHROW(BLPLoader::load({})); }
    SECTION("magic only")       { REQUIRE_NOTHROW(BLPLoader::load({'B','L','P','2'})); }

    SECTION("header cut short") {
        std::vector<uint8_t> data(kBlp2HeaderSize / 2, 0u);
        std::memcpy(data.data(), "BLP2", 4);
        REQUIRE_NOTHROW(BLPLoader::load(data));
    }
}

TEST_CASE("BLP loader survives a mip level smaller than its format needs",
          "[blp][robustness]") {
    // (4096/4) * (4096/4) blocks at 8 bytes each is 2 MiB of source. The file
    // supplies eight bytes.
    SECTION("DXT1 dimensions far larger than the mip data") {
        auto data = makeBlp2(4096, 4096, /*compression=*/2, /*mipSize=*/8,
                             kBlp2HeaderSize, 8, /*trailingBytes=*/8);
        REQUIRE_NOTHROW(BLPLoader::load(data));
    }

    SECTION("DXT3 and DXT5, same shape at 16 bytes a block") {
        for (uint8_t compression : {uint8_t{3}, uint8_t{4}}) {
            auto data = makeBlp2(1024, 1024, compression, 16,
                                 kBlp2HeaderSize, 8, 16);
            REQUIRE_NOTHROW(BLPLoader::load(data));
        }
    }

    // Palette reads one index byte per pixel and then an alpha run after them,
    // so it wants width*height*2 bytes at alphaDepth 8.
    SECTION("palette with one byte of mip data") {
        auto data = makeBlp2(2048, 2048, /*compression=*/1, 1,
                             kBlp2HeaderSize, 8, 1);
        REQUIRE_NOTHROW(BLPLoader::load(data));
    }

    SECTION("palette at every alpha depth") {
        for (uint8_t depth : {uint8_t{0}, uint8_t{1}, uint8_t{4}, uint8_t{8}}) {
            auto data = makeBlp2(512, 512, 1, 4, kBlp2HeaderSize, depth, 4);
            REQUIRE_NOTHROW(BLPLoader::load(data));
        }
    }
}

TEST_CASE("BLP loader rejects a mip offset that wraps the bounds check",
          "[blp][robustness]") {
    // offset + mipSize as uint32: 0xFFFFFF00 + 0x200 is 0x100, which is inside
    // any plausible file. Widened to 64 bits it is not.
    auto data = makeBlp2(64, 64, /*compression=*/2, /*mipSize=*/0x200,
                         /*mipOffset=*/0xFFFFFF00u, 8, /*trailingBytes=*/512);
    REQUIRE_NOTHROW(BLPLoader::load(data));
    // The mip is unreachable, so this must fail rather than decode something.
    REQUIRE_FALSE(BLPLoader::load(data).isValid());
}

TEST_CASE("BLP loader survives random bytes behind a valid magic",
          "[blp][robustness]") {
    std::vector<uint8_t> data(kBlp2HeaderSize + 4096);
    uint32_t state = 0x9E3779B9u;
    for (auto& b : data) {
        state = state * 1664525u + 1013904223u;
        b = static_cast<uint8_t>(state >> 24);
    }
    std::memcpy(data.data(), "BLP2", 4);
    REQUIRE_NOTHROW(BLPLoader::load(data));
}
