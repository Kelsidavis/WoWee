// What the ADT parser does with a file that lies about its own sizes.
//
// The chunk readers used to take a bare pointer and a length as two separate
// arguments, and readUInt32/readUInt16/readFloat did a memcpy at the offset
// they were handed without checking it against anything. Every guard lived at
// the call site, so a parser was only as safe as the arithmetic of whoever
// invoked it -- and one of those sums could wrap:
//
//   if (ofsAlpha > 0 && sizeAlpha > 0 && ofsAlpha + sizeAlpha <= size) {
//       uint32_t skip = (readUInt32(data, ofsAlpha) == MCAL) ? 8 : 0;
//       parseMCAL(data + ofsAlpha + skip, sizeAlpha - skip, chunk);
//   }
//
// sizeAlpha is only known to be >= 1 there. A four-byte MCAL whose contents
// spell "MCAL" takes skip to 8, and sizeAlpha - skip wraps to ~1.8e19, which
// then became a memcpy length.
//
// These cases are all malformed on purpose. None should read out of bounds,
// and none should crash -- run this suite under --asan, where an overrun is a
// failure rather than a value nobody notices.
#include <catch_amalgamated.hpp>

#include "pipeline/adt_loader.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

using wowee::pipeline::ADTLoader;

namespace {

constexpr uint32_t kMVER = 0x4D564552;
constexpr uint32_t kMCNK = 0x4D434E4B;
constexpr uint32_t kMCAL = 0x4D43414C;

void appendU32(std::vector<uint8_t>& out, uint32_t v) {
    uint8_t b[4];
    std::memcpy(b, &v, sizeof(v));
    out.insert(out.end(), b, b + 4);
}

/// magic + declared size + payload, the shape every ADT chunk has.
void appendChunk(std::vector<uint8_t>& out, uint32_t magic,
                 const std::vector<uint8_t>& payload) {
    appendU32(out, magic);
    appendU32(out, static_cast<uint32_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
}

/// A 128-byte MCNK header with every sub-chunk offset zeroed, which the parser
/// reads as "this chunk has none of them".
std::vector<uint8_t> mcnkHeader() {
    return std::vector<uint8_t>(128, 0);
}

void writeU32At(std::vector<uint8_t>& buf, size_t offset, uint32_t v) {
    REQUIRE(offset + sizeof(v) <= buf.size());
    std::memcpy(buf.data() + offset, &v, sizeof(v));
}

} // namespace

TEST_CASE("ADT loader survives a truncated file", "[adt][robustness]") {
    SECTION("empty input") {
        REQUIRE_NOTHROW(ADTLoader::load({}));
    }

    SECTION("a header cut off mid-way") {
        // Four bytes: a magic with no size field behind it.
        std::vector<uint8_t> data;
        appendU32(data, kMVER);
        REQUIRE_NOTHROW(ADTLoader::load(data));
    }

    SECTION("a chunk whose declared size runs past the end of the file") {
        std::vector<uint8_t> data;
        appendU32(data, kMVER);
        appendU32(data, 0xFFFF);  // claims 64k of payload
        data.insert(data.end(), 16, 0u);
        REQUIRE_NOTHROW(ADTLoader::load(data));
    }

    SECTION("a declared size of 0xFFFFFFFF") {
        std::vector<uint8_t> data;
        appendU32(data, kMCNK);
        appendU32(data, 0xFFFFFFFFu);
        data.insert(data.end(), 256, 0u);
        REQUIRE_NOTHROW(ADTLoader::load(data));
    }
}

TEST_CASE("ADT loader survives a self-inconsistent MCNK", "[adt][robustness]") {
    SECTION("sub-chunk offsets pointing past the chunk") {
        auto header = mcnkHeader();
        writeU32At(header, 20, 0xFFFF0000u);    // ofsHeight (MCVT)
        writeU32At(header, 24, 0xFFFF0000u);    // ofsNormal (MCNR)
        writeU32At(header, 28, 0xFFFF0000u);    // ofsLayer  (MCLY)
        writeU32At(header, 36, 0xFFFF0000u);    // ofsAlpha  (MCAL)
        writeU32At(header, 0x60, 0xFFFF0000u);  // ofsLiquid (MCLQ)

        std::vector<uint8_t> data;
        appendChunk(data, kMCNK, header);
        REQUIRE_NOTHROW(ADTLoader::load(data));
    }

    // The wrap described at the top of this file. sizeAlpha of 4 is inside the
    // chunk, so the bounds check at the call site passes; the four bytes at
    // that offset spell MCAL, so skip becomes 8; sizeAlpha - skip is then
    // 4 - 8. As an unsigned subtraction that is 18446744073709551612.
    SECTION("an MCAL smaller than its own sub-chunk header") {
        auto header = mcnkHeader();
        const uint32_t alphaOffset = 128;
        writeU32At(header, 36, alphaOffset);  // ofsAlpha
        writeU32At(header, 40, 4);            // sizeAlpha: fits, but < the 8-byte header

        std::vector<uint8_t> payload = header;
        appendU32(payload, kMCAL);  // the four bytes at ofsAlpha spell MCAL

        std::vector<uint8_t> data;
        appendChunk(data, kMCNK, payload);
        REQUIRE_NOTHROW(ADTLoader::load(data));

        // And the same shape for every size below the 8-byte sub-chunk header.
        for (uint32_t sizeAlpha = 1; sizeAlpha < 8; ++sizeAlpha) {
            auto h = mcnkHeader();
            writeU32At(h, 36, alphaOffset);
            writeU32At(h, 40, sizeAlpha);

            std::vector<uint8_t> p = h;
            appendU32(p, kMCAL);

            std::vector<uint8_t> d;
            appendChunk(d, kMCNK, p);
            REQUIRE_NOTHROW(ADTLoader::load(d));
        }
    }

    SECTION("an MCLQ shorter than its declared layout") {
        auto header = mcnkHeader();
        writeU32At(header, 0x60, 128);  // ofsLiquid, just past the header
        writeU32At(header, 0x64, 16);   // sizeLiquid, far below the 720-byte minimum

        std::vector<uint8_t> payload = header;
        payload.insert(payload.end(), 16, 0u);

        std::vector<uint8_t> data;
        appendChunk(data, kMCNK, payload);
        REQUIRE_NOTHROW(ADTLoader::load(data));
    }
}

TEST_CASE("ADT loader rejects a file of random bytes", "[adt][robustness]") {
    // Deterministic garbage: no seed, so a failure here reproduces.
    std::vector<uint8_t> data(4096);
    uint32_t state = 0x12345678u;
    for (auto& b : data) {
        state = state * 1664525u + 1013904223u;
        b = static_cast<uint8_t>(state >> 24);
    }
    REQUIRE_NOTHROW(ADTLoader::load(data));
}
