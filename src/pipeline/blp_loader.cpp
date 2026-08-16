#include "pipeline/dxt_block.hpp"
#include "pipeline/blp_loader.hpp"

#include <span>
#include "core/logger.hpp"
#include <cstring>
#include <algorithm>

namespace wowee {
namespace pipeline {

BLPImage BLPLoader::load(const std::vector<uint8_t>& blpData) {
    if (blpData.size() < 8) {  // Minimum: magic + first field
        LOG_ERROR("BLP data too small");
        return BLPImage();
    }

    const uint8_t* data = blpData.data();
    const char* magic = reinterpret_cast<const char*>(data);

    // Check magic number
    if (std::memcmp(magic, "BLP1", 4) == 0) {
        return loadBLP1(blpData);
    } if (std::memcmp(magic, "BLP2", 4) == 0) {
        return loadBLP2(blpData);
    } else if (std::memcmp(magic, "BLP0", 4) == 0) {
        LOG_WARNING("BLP0 format not fully supported");
        return BLPImage();
    } else {
        LOG_ERROR("Invalid BLP magic: ", std::string(magic, 4));
        return BLPImage();
    }
}

BLPImage BLPLoader::loadBLP1(std::span<const uint8_t> data) {
    const size_t size = data.size();
    // Copy header to stack to avoid unaligned reinterpret_cast (UB on strict platforms)
    if (size < sizeof(BLP1Header)) {
        LOG_ERROR("BLP1 data too small for header");
        return BLPImage();
    }
    BLP1Header header;
    std::memcpy(&header, data.data(), sizeof(BLP1Header));

    BLPImage image;
    image.format = BLPFormat::BLP1;
    image.width = header.width;
    image.height = header.height;
    image.channels = 4;
    image.mipLevels = header.hasMips ? 16 : 1;

    // BLP1 compression: 0=JPEG (not used in WoW), 1=palette/indexed
    // BLP1 does NOT support DXT - only palette with optional alpha
    if (header.compression == 1) {
        image.compression = BLPCompression::PALETTE;
    } else if (header.compression == 0) {
        LOG_WARNING("BLP1 JPEG compression not supported");
        return BLPImage();
    } else {
        LOG_WARNING("BLP1 unknown compression: ", header.compression);
        return BLPImage();
    }

    LOG_DEBUG("Loading BLP1: ", image.width, "x", image.height, " ",
              getCompressionName(image.compression), " alpha=", header.alphaBits);

    // Get first mipmap (full resolution)
    uint32_t offset = header.mipOffsets[0];
    uint32_t mipSize = header.mipSizes[0];

    // Widened before adding: both fields come from the file, and as uint32
    // an offset near 4G plus a size wraps and passes this check.
    if (static_cast<uint64_t>(offset) + mipSize > size) {
        LOG_ERROR("BLP1 mipmap data out of bounds (offset=", offset, " size=", mipSize, " fileSize=", size, ")");
        return BLPImage();
    }

    const std::span<const uint8_t> mipData = data.subspan(offset, mipSize);

    if (image.width <= 0 || image.height <= 0 || image.width > 4096 || image.height > 4096) {
        LOG_ERROR("BLP1 dimensions out of range: ", image.width, "x", image.height);
        return BLPImage();
    }
    uint32_t pixelCount = static_cast<uint32_t>(image.width) * static_cast<uint32_t>(image.height);
    image.data.resize(pixelCount * 4);  // RGBA8

    decompressPalette(mipData, image.data.data(), header.palette,
                      image.width, image.height, static_cast<uint8_t>(header.alphaBits));

    return image;
}

BLPImage BLPLoader::loadBLP2(std::span<const uint8_t> data) {
    const size_t size = data.size();
    // Copy header to stack to avoid unaligned reinterpret_cast (UB on strict platforms)
    if (size < sizeof(BLP2Header)) {
        LOG_ERROR("BLP2 data too small for header");
        return BLPImage();
    }
    BLP2Header header;
    std::memcpy(&header, data.data(), sizeof(BLP2Header));

    BLPImage image;
    image.format = BLPFormat::BLP2;
    image.width = header.width;
    image.height = header.height;
    image.channels = 4;
    image.mipLevels = header.hasMips ? 16 : 1;

    // BLP2 compression types:
    //   1 = palette/uncompressed
    //   2 = DXTC (DXT1/DXT3/DXT5 based on alphaDepth + alphaEncoding)
    //   3 = plain A8R8G8B8
    if (header.compression == 1) {
        image.compression = BLPCompression::PALETTE;
    } else if (header.compression == 2) {
        // BLP2 DXTC format selection based on alphaDepth + alphaEncoding:
        //   alphaDepth=0                    → DXT1 (no alpha)
        //   alphaDepth>0, alphaEncoding=0   → DXT1 (1-bit alpha)
        //   alphaDepth>0, alphaEncoding=1   → DXT3 (explicit 4-bit alpha)
        //   alphaDepth>0, alphaEncoding=7   → DXT5 (interpolated alpha)
        if (header.alphaDepth == 0 || header.alphaEncoding == 0) {
            image.compression = BLPCompression::DXT1;
        } else if (header.alphaEncoding == 1) {
            image.compression = BLPCompression::DXT3;
        } else if (header.alphaEncoding == 7) {
            image.compression = BLPCompression::DXT5;
        } else {
            image.compression = BLPCompression::DXT1;
        }
    } else if (header.compression == 3) {
        image.compression = BLPCompression::ARGB8888;
    } else {
        image.compression = BLPCompression::ARGB8888;
    }

    LOG_DEBUG("Loading BLP2: ", image.width, "x", image.height, " ",
              getCompressionName(image.compression),
              " (comp=", static_cast<int>(header.compression), " alphaDepth=", static_cast<int>(header.alphaDepth),
              " alphaEnc=", static_cast<int>(header.alphaEncoding), " mipOfs=", header.mipOffsets[0],
              " mipSize=", header.mipSizes[0], ")");

    // Get first mipmap (full resolution)
    uint32_t offset = header.mipOffsets[0];
    uint32_t mipSize = header.mipSizes[0];

    // See loadBLP1: widened so a near-4G offset cannot wrap past this check.
    if (static_cast<uint64_t>(offset) + mipSize > size) {
        LOG_ERROR("BLP2 mipmap data out of bounds");
        return BLPImage();
    }

    const std::span<const uint8_t> mipData = data.subspan(offset, mipSize);

    if (image.width <= 0 || image.height <= 0 || image.width > 4096 || image.height > 4096) {
        LOG_ERROR("BLP2 dimensions out of range: ", image.width, "x", image.height);
        return BLPImage();
    }
    uint32_t pixelCount = static_cast<uint32_t>(image.width) * static_cast<uint32_t>(image.height);
    uint32_t requiredArgb = pixelCount * 4;
    // For ARGB8888 the source must be at least pixelCount*4 bytes; for DXT/palette
    // the source is smaller but the decompressors have their own internal bounds.
    if (image.compression == BLPCompression::ARGB8888 && mipSize < requiredArgb) {
        LOG_ERROR("BLP2 ARGB8888 mipSize (", mipSize, ") < required (", requiredArgb, ")");
        return BLPImage();
    }
    image.data.resize(requiredArgb);  // RGBA8

    switch (image.compression) {
        case BLPCompression::DXT1:
            decompressDXT1(mipData, image.data.data(), image.width, image.height);
            break;

        case BLPCompression::DXT3:
            decompressDXT3(mipData, image.data.data(), image.width, image.height);
            break;

        case BLPCompression::DXT5:
            decompressDXT5(mipData, image.data.data(), image.width, image.height);
            break;

        case BLPCompression::PALETTE:
            decompressPalette(mipData, image.data.data(), header.palette,
                              image.width, image.height, header.alphaDepth);
            break;

        case BLPCompression::ARGB8888:
            for (uint32_t i = 0; i < pixelCount; i++) {
                image.data[i * 4 + 0] = mipData[i * 4 + 2];  // R
                image.data[i * 4 + 1] = mipData[i * 4 + 1];  // G
                image.data[i * 4 + 2] = mipData[i * 4 + 0];  // B
                image.data[i * 4 + 3] = mipData[i * 4 + 3];  // A
            }
            break;

        default:
            LOG_ERROR("Unsupported BLP2 compression type");
            return BLPImage();
    }

    // Note: DXT1 may encode 1-bit transparency via the color-key mode (c0 <= c1).
    // Do not override alpha based on alphaDepth; preserve whatever the DXT decompressor produced.

    return image;
}

void BLPLoader::decompressDXT1(std::span<const uint8_t> src, uint8_t* dst, int width, int height) {
    // DXT1 decompression (8 bytes per 4x4 block)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const size_t blockOffset = static_cast<size_t>(by * blockWidth + bx) * 8;
            if (blockOffset + 8 > src.size()) continue;
            const uint8_t* block = src.data() + blockOffset;

            // DXT1 reads the endpoint order as a mode flag, which is how a
            // cut-out texture encodes its transparent pixels.
            const DxtColorBlock colors = decodeDxtColorBlock(block, /*allowPunchThrough=*/true);

            // Decompress 4x4 block
            for (int py = 0; py < 4; py++) {
                for (int px = 0; px < 4; px++) {
                    int x = bx * 4 + px;
                    int y = by * 4 + py;

                    if (x >= width || y >= height) continue;

                    const int index = colors.indexAt(px, py);
                    uint8_t* pixel = dst + (y * width + x) * 4;

                    pixel[0] = colors.rgb[index][0];
                    pixel[1] = colors.rgb[index][1];
                    pixel[2] = colors.rgb[index][2];
                    pixel[3] = (colors.index3IsTransparent && index == 3) ? 0 : 255;
                }
            }
        }
    }
}

namespace {

/// Writes one decoded 4x4 block into the image, taking each pixel's alpha from
/// the caller.
///
/// DXT3 and DXT5 differ only in where alpha comes from - four bits a pixel
/// against a three-bit index into an interpolated ramp - and each carried its
/// own copy of the walk that clips the block against the image edge and writes
/// the colour. A texture whose width is not a multiple of four relies on that
/// clip, and it was written twice.
template <typename AlphaFor>
void writeBlockPixels(const DxtColorBlock& colors, uint8_t* dst,
                      int width, int height, int bx, int by, AlphaFor&& alphaFor) {
    for (int py = 0; py < 4; py++) {
        for (int px = 0; px < 4; px++) {
            const int x = bx * 4 + px;
            const int y = by * 4 + py;
            if (x >= width || y >= height) continue;

            const int index = colors.indexAt(px, py);
            uint8_t* pixel = dst + (y * width + x) * 4;
            pixel[0] = colors.rgb[index][0];
            pixel[1] = colors.rgb[index][1];
            pixel[2] = colors.rgb[index][2];
            pixel[3] = alphaFor(px, py);
        }
    }
}

}  // namespace

void BLPLoader::decompressDXT3(std::span<const uint8_t> src, uint8_t* dst, int width, int height) {
    // DXT3 decompression (16 bytes per 4x4 block - 8 bytes alpha + 8 bytes color)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const size_t blockOffset = static_cast<size_t>(by * blockWidth + bx) * 16;
            if (blockOffset + 16 > src.size()) continue;
            const uint8_t* block = src.data() + blockOffset;

            // First 8 bytes: 4-bit alpha values
            uint64_t alphaBlock = 0;
            for (int i = 0; i < 8; i++) {
                alphaBlock |= static_cast<uint64_t>(block[i]) << (i * 8);
            }

            // The colour half is the same eight bytes DXT1 uses, at byte 8.
            // Alpha is carried separately here, so the endpoint order is not a
            // mode flag and there is no transparent index.
            const DxtColorBlock colors =
                decodeDxtColorBlock(block + 8, /*allowPunchThrough=*/false);

            // Four bits a pixel, scaled from [0..15] to [0..255].
            writeBlockPixels(colors, dst, width, height, bx, by,
                             [&](int px, int py) -> uint8_t {
                const uint8_t alpha4 = (alphaBlock >> ((py * 4 + px) * 4)) & 0xF;
                return static_cast<uint8_t>(alpha4 * 255 / 15);
            });
        }
    }
}

void BLPLoader::decompressDXT5(std::span<const uint8_t> src, uint8_t* dst, int width, int height) {
    // DXT5 decompression (16 bytes per 4x4 block - interpolated alpha + color)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const size_t blockOffset = static_cast<size_t>(by * blockWidth + bx) * 16;
            if (blockOffset + 16 > src.size()) continue;
            const uint8_t* block = src.data() + blockOffset;

            // Alpha endpoints
            uint8_t alpha0 = block[0];
            uint8_t alpha1 = block[1];

            // Build alpha lookup table
            uint8_t alphas[8];
            alphas[0] = alpha0;
            alphas[1] = alpha1;
            if (alpha0 > alpha1) {
                alphas[2] = (6*alpha0 + 1*alpha1) / 7;
                alphas[3] = (5*alpha0 + 2*alpha1) / 7;
                alphas[4] = (4*alpha0 + 3*alpha1) / 7;
                alphas[5] = (3*alpha0 + 4*alpha1) / 7;
                alphas[6] = (2*alpha0 + 5*alpha1) / 7;
                alphas[7] = (1*alpha0 + 6*alpha1) / 7;
            } else {
                alphas[2] = (4*alpha0 + 1*alpha1) / 5;
                alphas[3] = (3*alpha0 + 2*alpha1) / 5;
                alphas[4] = (2*alpha0 + 3*alpha1) / 5;
                alphas[5] = (1*alpha0 + 4*alpha1) / 5;
                alphas[6] = 0;
                alphas[7] = 255;
            }

            // Alpha indices (48 bits for 16 pixels, 3 bits each)
            uint64_t alphaIndices = 0;
            for (int i = 2; i < 8; i++) {
                alphaIndices |= static_cast<uint64_t>(block[i]) << ((i - 2) * 8);
            }

            // The colour half is the same eight bytes DXT1 uses, at byte 8.
            // Alpha is carried separately here, so the endpoint order is not a
            // mode flag and there is no transparent index.
            const DxtColorBlock colors =
                decodeDxtColorBlock(block + 8, /*allowPunchThrough=*/false);

            // A three-bit index into the eight-entry ramp built above.
            writeBlockPixels(colors, dst, width, height, bx, by,
                             [&](int px, int py) -> uint8_t {
                return alphas[(alphaIndices >> ((py * 4 + px) * 3)) & 0x7];
            });
        }
    }
}

void BLPLoader::decompressPalette(std::span<const uint8_t> src, uint8_t* dst, const uint32_t* palette, int width, int height, uint8_t alphaDepth) {
    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);

    // Palette indices come first (one byte per pixel) and the alpha data
    // follows them. Neither length is stated in the file: the mip size is,
    // and it need not agree with width * height. Read through a checked
    // accessor rather than trusting it to -- a mip of eight bytes claiming
    // 4096x4096 would otherwise walk 16M bytes off the end.
    const auto byteAt = [&src](size_t i) -> uint8_t {
        return i < src.size() ? src[i] : uint8_t{0};
    };
    const size_t alphaBase = pixelCount;

    for (size_t i = 0; i < pixelCount; i++) {
        uint8_t index = byteAt(i);
        uint32_t color = palette[index];

        // Palette stores BGR (the high byte is typically 0, not alpha)
        dst[i * 4 + 0] = (color >> 16) & 0xFF;  // R
        dst[i * 4 + 1] = (color >> 8) & 0xFF;   // G
        dst[i * 4 + 2] = color & 0xFF;           // B

        // Alpha is stored separately after the index data
        if (alphaDepth == 8) {
            dst[i * 4 + 3] = byteAt(alphaBase + i);
        } else if (alphaDepth == 4) {
            // 4-bit alpha: 2 pixels packed per byte (low nibble first).
            // Multiply by 17 to scale [0..15] → [0..255] (equivalent to n * 255 / 15).
            uint8_t alphaByte = byteAt(alphaBase + i / 2);
            dst[i * 4 + 3] = (i % 2 == 0) ? ((alphaByte & 0x0F) * 17) : ((alphaByte >> 4) * 17);
        } else if (alphaDepth == 1) {
            // 1-bit alpha: 8 pixels per byte
            uint8_t alphaByte = byteAt(alphaBase + i / 8);
            dst[i * 4 + 3] = ((alphaByte >> (i % 8)) & 1) ? 255 : 0;
        } else {
            // No alpha channel: fully opaque
            dst[i * 4 + 3] = 255;
        }
    }
}

const char* BLPLoader::getFormatName(BLPFormat format) {
    switch (format) {
        case BLPFormat::BLP0: return "BLP0";
        case BLPFormat::BLP1: return "BLP1";
        case BLPFormat::BLP2: return "BLP2";
        default: return "Unknown";
    }
}

const char* BLPLoader::getCompressionName(BLPCompression compression) {
    switch (compression) {
        case BLPCompression::NONE: return "None";
        case BLPCompression::PALETTE: return "Palette";
        case BLPCompression::DXT1: return "DXT1";
        case BLPCompression::DXT3: return "DXT3";
        case BLPCompression::DXT5: return "DXT5";
        case BLPCompression::ARGB8888: return "ARGB8888";
        default: return "Unknown";
    }
}

} // namespace pipeline
} // namespace wowee
