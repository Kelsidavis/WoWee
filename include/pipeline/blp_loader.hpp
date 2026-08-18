#pragma once

#include <span>
#include <vector>
#include <cstdint>
#include <string>

namespace wowee {
namespace pipeline {

/**
 * BLP image format (Blizzard Picture)
 */
enum class BLPFormat {
    UNKNOWN = 0,
    BLP0 = 1,      // Alpha channel only
    BLP1 = 2,      // DXT compression or uncompressed
    BLP2 = 3       // DXT compression with mipmaps
};

/**
 * BLP compression type
 */
enum class BLPCompression {
    NONE = 0,
    PALETTE = 1,     // 256-color palette
    DXT1 = 2,        // DXT1 compression (no alpha or 1-bit alpha)
    DXT3 = 3,        // DXT3 compression (4-bit alpha)
    DXT5 = 4,        // DXT5 compression (interpolated alpha)
    ARGB8888 = 5     // Uncompressed 32-bit ARGB
};

/**
 * Loaded BLP image data
 */
struct BLPImage {
    int width = 0;
    int height = 0;
    int channels = 4;
    int mipLevels = 1;
    BLPFormat format = BLPFormat::UNKNOWN;
    BLPCompression compression = BLPCompression::NONE;
    std::vector<uint8_t> data;      // RGBA8 pixel data (decompressed)
    std::vector<std::vector<uint8_t>> mipmaps;  // Mipmap levels

    [[nodiscard]] bool isValid() const {
        return width > 0 && height > 0 && (!data.empty() || !mipmaps.empty());
    }
    /// True when mipmaps holds DXT blocks rather than data holding RGBA8.
    [[nodiscard]] bool isBlockCompressed() const { return !mipmaps.empty(); }

    /// Whether any texel is less than fully opaque.
    ///
    /// The same question a caller used to answer by walking every fourth byte
    /// of the decoded RGBA8, and the reason M2 textures had to be decoded at
    /// all. Answered from the blocks instead, exactly rather than by format:
    /// a DXT3 or DXT5 texture carries an alpha channel that may still be
    /// opaque everywhere, and DXT1's punch-through is per block, so the format
    /// alone would report alpha that is not there and silence an alpha test
    /// that is currently running.
    [[nodiscard]] bool hasTransparency() const;

    /// What this will occupy once uploaded, which the texture caches spend
    /// their budget against. Block-compressed is its own levels; decoded is
    /// the base plus the third a generated mip chain adds.
    [[nodiscard]] size_t approxUploadBytes() const {
        if (isBlockCompressed()) {
            size_t total = 0;
            for (const auto& level : mipmaps) total += level.size();
            return total;
        }
        const size_t base = static_cast<size_t>(width) * static_cast<size_t>(height) * 4ull;
        return base + (base / 3);
    }

    /// What this would have occupied decoded, whether or not it is.
    ///
    /// The counterfactual the block upload is measured against: RGBA8 plus the
    /// third a generated mip chain adds, which is what every one of these
    /// textures cost before they went up as blocks. Reported next to the real
    /// figure so the saving is a measurement rather than an estimate.
    [[nodiscard]] size_t approxDecodedUploadBytes() const {
        const size_t base = static_cast<size_t>(width) * static_cast<size_t>(height) * 4ull;
        return base + (base / 3);
    }
};

/**
 * BLP texture loader
 *
 * Supports BLP0, BLP1, BLP2 formats
 * Handles DXT1/3/5 compression and palette formats
 * Format specification: https://wowdev.wiki/BLP
 */
/// Whether this GPU can sample BC1/BC2/BC3, which is what a DXT BLP's blocks
/// are handed to it as.
///
/// Desktop GPUs all can. Mobile ones generally cannot: Mali and Adreno carry
/// ASTC and ETC2 instead, and asking them for a BC image gives an untextured
/// surface rather than an error. Set once from the renderer after the device is
/// chosen; true until then, which is what every desktop wants and what the
/// tools that load textures with no device at all want too.
void setBlockCompressionSupported(bool supported);
bool blockCompressionSupported();

class BLPLoader {
public:
    /**
     * Load BLP image from byte data
     * @param blpData Raw BLP file data
     * @return Loaded image (check isValid())
     */
    /// keepCompressed leaves a DXT texture in its DXT blocks: mipmaps holds
    /// every level as stored and data stays empty. The GPU samples BC1/BC2/BC3
    /// natively, so decoding to RGBA8 costs CPU time to produce four to eight
    /// times the bytes, and the file's own mip levels get thrown away and
    /// rebuilt. Only for callers that do not read the pixels - anything that
    /// composites or scans them needs the decoded form.
    ///
    /// Palette and ARGB8888 always decode; there is nothing to pass through.
    static BLPImage load(const std::vector<uint8_t>& blpData, bool keepCompressed = false);

    /// Decode a block-compressed image's base level to RGBA8.
    ///
    /// For callers that need the pixels for something other than sampling -
    /// building a normal map, scanning for a colour key - while the texture
    /// itself still uploads as blocks. Returns empty for an image that is not
    /// block-compressed, whose pixels are already in data.
    static std::vector<uint8_t> decodeBaseLevel(const BLPImage& image);

    /**
     * Get format name for debugging
     */
    static const char* getFormatName(BLPFormat format);
    static const char* getCompressionName(BLPCompression compression);

private:
    // BLP1 file header - all fields after magic are uint32
    // Used by classic WoW through WotLK for many textures
    struct BLP1Header {
        char magic[4];           // 'BLP1'
        uint32_t compression;    // 0=JPEG, 1=palette (uncompressed/indexed)
        uint32_t alphaBits;      // 0, 1, 4, or 8
        uint32_t width;
        uint32_t height;
        uint32_t extra;          // Flags/unknown (often 4 or 5)
        uint32_t hasMips;        // 0 or 1
        uint32_t mipOffsets[16];
        uint32_t mipSizes[16];
        uint32_t palette[256];   // 256-color BGRA palette (for compression=1)
    };
    static_assert(sizeof(BLP1Header) == 1180,
                  "BLP1Header is memcpy'd from the file: 1180 bytes, no padding");

    // BLP2 file header - compression fields are uint8
    // Used by WoW from TBC onwards (coexists with BLP1 in WotLK)
    struct BLP2Header {
        char magic[4];           // 'BLP2'
        uint32_t version;        // Always 1
        uint8_t compression;     // 1=uncompressed/palette, 2=DXTC, 3=A8R8G8B8
        uint8_t alphaDepth;      // 0, 1, 4, or 8
        uint8_t alphaEncoding;   // 0=DXT1, 1=DXT3, 7=DXT5
        uint8_t hasMips;         // Has mipmaps
        uint32_t width;
        uint32_t height;
        uint32_t mipOffsets[16];
        uint32_t mipSizes[16];
        uint32_t palette[256];   // 256-color BGRA palette (for compression=1)
    };
    static_assert(sizeof(BLP2Header) == 1172,
                  "BLP2Header is memcpy'd from the file: 1172 bytes, no padding");

    static BLPImage loadBLP1(std::span<const uint8_t> data);
    static BLPImage loadBLP2(std::span<const uint8_t> data, bool keepCompressed);
    static void decompressDXT1(std::span<const uint8_t> src, uint8_t* dst, int width, int height);
    static void decompressDXT3(std::span<const uint8_t> src, uint8_t* dst, int width, int height);
    static void decompressDXT5(std::span<const uint8_t> src, uint8_t* dst, int width, int height);
    static void decompressPalette(std::span<const uint8_t> src, uint8_t* dst, const uint32_t* palette, int width, int height, uint8_t alphaDepth = 8);

    /// Bytes a mip level must hold for the given format and dimensions.
    /// Returns 0 when the product would overflow.
    static size_t requiredSourceBytes(BLPCompression compression, int width, int height,
                                      uint8_t alphaDepth);
};

} // namespace pipeline
} // namespace wowee
