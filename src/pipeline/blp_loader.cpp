#include "pipeline/blp_loader.hpp"
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
        return loadBLP1(data, blpData.size());
    } else if (std::memcmp(magic, "BLP2", 4) == 0) {
        return loadBLP2(data, blpData.size());
    } else if (std::memcmp(magic, "BLP0", 4) == 0) {
        LOG_WARNING("BLP0 format not fully supported");
        return BLPImage();
    } else {
        LOG_ERROR("Invalid BLP magic: ", std::string(magic, 4));
        return BLPImage();
    }
}

BLPImage BLPLoader::loadBLP1(const uint8_t* data, size_t size) {
    // Copy header to stack to avoid unaligned reinterpret_cast (UB on strict platforms)
    if (size < sizeof(BLP1Header)) {
        LOG_ERROR("BLP1 data too small for header");
        return BLPImage();
    }
    BLP1Header header;
    std::memcpy(&header, data, sizeof(BLP1Header));

    BLPImage image;
    image.format = BLPFormat::BLP1;
    image.width = header.width;
    image.height = header.height;
    image.channels = 4;
    image.mipLevels = header.hasMips ? 16 : 1;

    // BLP1 compression: 0=JPEG (not used in WoW), 1=palette/indexed
    // BLP1 does NOT support DXT — only palette with optional alpha
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

    if (offset + mipSize > size) {
        LOG_ERROR("BLP1 mipmap data out of bounds (offset=", offset, " size=", mipSize, " fileSize=", size, ")");
        return BLPImage();
    }

    const uint8_t* mipData = data + offset;

    // Allocate output buffer
    int pixelCount = image.width * image.height;
    image.data.resize(pixelCount * 4);  // RGBA8

    decompressPalette(mipData, image.data.data(), header.palette,
                      image.width, image.height, static_cast<uint8_t>(header.alphaBits));

    return image;
}

BLPImage BLPLoader::loadBLP2(const uint8_t* data, size_t size) {
    // Copy header to stack to avoid unaligned reinterpret_cast (UB on strict platforms)
    if (size < sizeof(BLP2Header)) {
        LOG_ERROR("BLP2 data too small for header");
        return BLPImage();
    }
    BLP2Header header;
    std::memcpy(&header, data, sizeof(BLP2Header));

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
              " (comp=", (int)header.compression, " alphaDepth=", (int)header.alphaDepth,
              " alphaEnc=", (int)header.alphaEncoding, " mipOfs=", header.mipOffsets[0],
              " mipSize=", header.mipSizes[0], ")");

    // Get first mipmap (full resolution)
    uint32_t offset = header.mipOffsets[0];
    uint32_t mipSize = header.mipSizes[0];

    if (offset + mipSize > size) {
        LOG_ERROR("BLP2 mipmap data out of bounds");
        return BLPImage();
    }

    const uint8_t* mipData = data + offset;

    // Allocate output buffer
    int pixelCount = image.width * image.height;
    image.data.resize(pixelCount * 4);  // RGBA8

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
            for (int i = 0; i < pixelCount; i++) {
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

void BLPLoader::decompressDXT1(const uint8_t* src, uint8_t* dst, int width, int height) {
    // DXT1 decompression (8 bytes per 4x4 block)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const uint8_t* block = src + (by * blockWidth + bx) * 8;

            // Read color endpoints (RGB565)
            uint16_t c0 = block[0] | (block[1] << 8);
            uint16_t c1 = block[2] | (block[3] << 8);

            // Convert RGB565 to RGB888
            uint8_t r0 = ((c0 >> 11) & 0x1F) * 255 / 31;
            uint8_t g0 = ((c0 >> 5) & 0x3F) * 255 / 63;
            uint8_t b0 = (c0 & 0x1F) * 255 / 31;

            uint8_t r1 = ((c1 >> 11) & 0x1F) * 255 / 31;
            uint8_t g1 = ((c1 >> 5) & 0x3F) * 255 / 63;
            uint8_t b1 = (c1 & 0x1F) * 255 / 31;

            // Read 4x4 color indices (2 bits per pixel)
            uint32_t indices = block[4] | (block[5] << 8) | (block[6] << 16) | (block[7] << 24);

            // Decompress 4x4 block
            for (int py = 0; py < 4; py++) {
                for (int px = 0; px < 4; px++) {
                    int x = bx * 4 + px;
                    int y = by * 4 + py;

                    if (x >= width || y >= height) continue;

                    int index = (indices >> ((py * 4 + px) * 2)) & 0x3;
                    uint8_t* pixel = dst + (y * width + x) * 4;

                    // Interpolate colors based on index
                    if (c0 > c1) {
                        switch (index) {
                            case 0: pixel[0] = r0; pixel[1] = g0; pixel[2] = b0; pixel[3] = 255; break;
                            case 1: pixel[0] = r1; pixel[1] = g1; pixel[2] = b1; pixel[3] = 255; break;
                            case 2: pixel[0] = (2*r0 + r1) / 3; pixel[1] = (2*g0 + g1) / 3; pixel[2] = (2*b0 + b1) / 3; pixel[3] = 255; break;
                            case 3: pixel[0] = (r0 + 2*r1) / 3; pixel[1] = (g0 + 2*g1) / 3; pixel[2] = (b0 + 2*b1) / 3; pixel[3] = 255; break;
                        }
                    } else {
                        switch (index) {
                            case 0: pixel[0] = r0; pixel[1] = g0; pixel[2] = b0; pixel[3] = 255; break;
                            case 1: pixel[0] = r1; pixel[1] = g1; pixel[2] = b1; pixel[3] = 255; break;
                            case 2: pixel[0] = (r0 + r1) / 2; pixel[1] = (g0 + g1) / 2; pixel[2] = (b0 + b1) / 2; pixel[3] = 255; break;
                            case 3: pixel[0] = 0; pixel[1] = 0; pixel[2] = 0; pixel[3] = 0; break;  // Transparent
                        }
                    }
                }
            }
        }
    }
}

void BLPLoader::decompressDXT3(const uint8_t* src, uint8_t* dst, int width, int height) {
    // DXT3 decompression (16 bytes per 4x4 block - 8 bytes alpha + 8 bytes color)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const uint8_t* block = src + (by * blockWidth + bx) * 16;

            // First 8 bytes: 4-bit alpha values
            uint64_t alphaBlock = 0;
            for (int i = 0; i < 8; i++) {
                alphaBlock |= (uint64_t)block[i] << (i * 8);
            }

            // Color block (same as DXT1) starts at byte 8
            const uint8_t* colorBlock = block + 8;

            uint16_t c0 = colorBlock[0] | (colorBlock[1] << 8);
            uint16_t c1 = colorBlock[2] | (colorBlock[3] << 8);

            uint8_t r0 = ((c0 >> 11) & 0x1F) * 255 / 31;
            uint8_t g0 = ((c0 >> 5) & 0x3F) * 255 / 63;
            uint8_t b0 = (c0 & 0x1F) * 255 / 31;

            uint8_t r1 = ((c1 >> 11) & 0x1F) * 255 / 31;
            uint8_t g1 = ((c1 >> 5) & 0x3F) * 255 / 63;
            uint8_t b1 = (c1 & 0x1F) * 255 / 31;

            uint32_t indices = colorBlock[4] | (colorBlock[5] << 8) | (colorBlock[6] << 16) | (colorBlock[7] << 24);

            for (int py = 0; py < 4; py++) {
                for (int px = 0; px < 4; px++) {
                    int x = bx * 4 + px;
                    int y = by * 4 + py;

                    if (x >= width || y >= height) continue;

                    int index = (indices >> ((py * 4 + px) * 2)) & 0x3;
                    uint8_t* pixel = dst + (y * width + x) * 4;

                    // DXT3 always uses 4-color mode for the color portion
                    switch (index) {
                        case 0: pixel[0] = r0; pixel[1] = g0; pixel[2] = b0; break;
                        case 1: pixel[0] = r1; pixel[1] = g1; pixel[2] = b1; break;
                        case 2: pixel[0] = (2*r0 + r1) / 3; pixel[1] = (2*g0 + g1) / 3; pixel[2] = (2*b0 + b1) / 3; break;
                        case 3: pixel[0] = (r0 + 2*r1) / 3; pixel[1] = (g0 + 2*g1) / 3; pixel[2] = (b0 + 2*b1) / 3; break;
                    }

                    // Apply 4-bit alpha
                    int alphaIndex = py * 4 + px;
                    uint8_t alpha4 = (alphaBlock >> (alphaIndex * 4)) & 0xF;
                    pixel[3] = alpha4 * 255 / 15;
                }
            }
        }
    }
}

void BLPLoader::decompressDXT5(const uint8_t* src, uint8_t* dst, int width, int height) {
    // DXT5 decompression (16 bytes per 4x4 block - interpolated alpha + color)
    int blockWidth = (width + 3) / 4;
    int blockHeight = (height + 3) / 4;

    for (int by = 0; by < blockHeight; by++) {
        for (int bx = 0; bx < blockWidth; bx++) {
            const uint8_t* block = src + (by * blockWidth + bx) * 16;

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
                alphaIndices |= (uint64_t)block[i] << ((i - 2) * 8);
            }

            // Color block (same as DXT1) starts at byte 8
            const uint8_t* colorBlock = block + 8;

            uint16_t c0 = colorBlock[0] | (colorBlock[1] << 8);
            uint16_t c1 = colorBlock[2] | (colorBlock[3] << 8);

            uint8_t r0 = ((c0 >> 11) & 0x1F) * 255 / 31;
            uint8_t g0 = ((c0 >> 5) & 0x3F) * 255 / 63;
            uint8_t b0 = (c0 & 0x1F) * 255 / 31;

            uint8_t r1 = ((c1 >> 11) & 0x1F) * 255 / 31;
            uint8_t g1 = ((c1 >> 5) & 0x3F) * 255 / 63;
            uint8_t b1 = (c1 & 0x1F) * 255 / 31;

            uint32_t indices = colorBlock[4] | (colorBlock[5] << 8) | (colorBlock[6] << 16) | (colorBlock[7] << 24);

            for (int py = 0; py < 4; py++) {
                for (int px = 0; px < 4; px++) {
                    int x = bx * 4 + px;
                    int y = by * 4 + py;

                    if (x >= width || y >= height) continue;

                    int index = (indices >> ((py * 4 + px) * 2)) & 0x3;
                    uint8_t* pixel = dst + (y * width + x) * 4;

                    // DXT5 always uses 4-color mode for the color portion
                    switch (index) {
                        case 0: pixel[0] = r0; pixel[1] = g0; pixel[2] = b0; break;
                        case 1: pixel[0] = r1; pixel[1] = g1; pixel[2] = b1; break;
                        case 2: pixel[0] = (2*r0 + r1) / 3; pixel[1] = (2*g0 + g1) / 3; pixel[2] = (2*b0 + b1) / 3; break;
                        case 3: pixel[0] = (r0 + 2*r1) / 3; pixel[1] = (g0 + 2*g1) / 3; pixel[2] = (b0 + 2*b1) / 3; break;
                    }

                    // Apply interpolated alpha
                    int alphaIdx = (alphaIndices >> ((py * 4 + px) * 3)) & 0x7;
                    pixel[3] = alphas[alphaIdx];
                }
            }
        }
    }
}

void BLPLoader::decompressPalette(const uint8_t* src, uint8_t* dst, const uint32_t* palette, int width, int height, uint8_t alphaDepth) {
    int pixelCount = width * height;

    // Palette indices are first (1 byte per pixel)
    const uint8_t* indices = src;
    // Alpha data follows the palette indices
    const uint8_t* alphaData = src + pixelCount;

    for (int i = 0; i < pixelCount; i++) {
        uint8_t index = indices[i];
        uint32_t color = palette[index];

        // Palette stores BGR (the high byte is typically 0, not alpha)
        dst[i * 4 + 0] = (color >> 16) & 0xFF;  // R
        dst[i * 4 + 1] = (color >> 8) & 0xFF;   // G
        dst[i * 4 + 2] = color & 0xFF;           // B

        // Alpha is stored separately after the index data
        if (alphaDepth == 8) {
            dst[i * 4 + 3] = alphaData[i];
        } else if (alphaDepth == 4) {
            // 4-bit alpha: 2 pixels per byte
            uint8_t alphaByte = alphaData[i / 2];
            dst[i * 4 + 3] = (i % 2 == 0) ? ((alphaByte & 0x0F) * 17) : ((alphaByte >> 4) * 17);
        } else if (alphaDepth == 1) {
            // 1-bit alpha: 8 pixels per byte
            uint8_t alphaByte = alphaData[i / 8];
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
