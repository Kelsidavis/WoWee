#include "pipeline/adt_alpha.hpp"

#include <algorithm>

namespace wowee {
namespace pipeline {

namespace {

/// The last row and column of a four-bit map are not data.
///
/// A chunk's alpha map is stored 64 wide and only 63 of those are painted: the
/// file's last row and column carry whatever was left there, and the client
/// fills them from the row and column before. Without it every chunk ends in a
/// strip of something else, and since the strip is on two of the four sides,
/// the boundary between two chunks is a hard line - which is the ground
/// textures not quite lining up, in a grid across the whole world.
///
/// Only the four-bit form. The eight-bit and compressed maps are what a map
/// with "big alpha" carries, and those are painted to the edge.
void fixLastRowAndColumn(std::vector<uint8_t>& alpha) {
    if (alpha.size() < ALPHA_MAP_SIZE) return;
    constexpr size_t kLast = ALPHA_MAP_DIM - 1;
    for (size_t i = 0; i < ALPHA_MAP_DIM; ++i) {
        alpha[kLast * ALPHA_MAP_DIM + i] = alpha[(kLast - 1) * ALPHA_MAP_DIM + i];
    }
    // After the row, so the corner takes the value the row above it just did.
    for (size_t i = 0; i < ALPHA_MAP_DIM; ++i) {
        alpha[i * ALPHA_MAP_DIM + kLast] = alpha[i * ALPHA_MAP_DIM + kLast - 1];
    }
}

}  // namespace

bool decodeLayerAlpha(const MapChunk& chunk, size_t layerIdx,
                      std::vector<uint8_t>& outAlpha, uint8_t unsetFill) {
    outAlpha.assign(ALPHA_MAP_SIZE, unsetFill);

    if (layerIdx >= chunk.layers.size()) return false;
    const auto& layer = chunk.layers[layerIdx];
    if (!layer.useAlpha() || layer.offsetMCAL >= chunk.alphaMap.size()) return false;

    const size_t offset = layer.offsetMCAL;

    // How much of MCAL belongs to this layer, taken from where the next layer
    // with an alpha map starts rather than from what is left in the blob. The
    // difference decides between the 4-bit and 8-bit forms below, so reading
    // "everything remaining" would misidentify every layer but the last.
    size_t layerSize = chunk.alphaMap.size() - offset;
    for (size_t j = layerIdx + 1; j < chunk.layers.size(); ++j) {
        if (chunk.layers[j].useAlpha()) {
            layerSize = chunk.layers[j].offsetMCAL - offset;
            break;
        }
    }

    if (layer.compressedAlpha()) {
        size_t readPos = offset;
        size_t writePos = 0;
        while (writePos < ALPHA_MAP_SIZE && readPos < chunk.alphaMap.size()) {
            const uint8_t cmd = chunk.alphaMap[readPos++];
            const bool fill = (cmd & ALPHA_FILL_FLAG) != 0;
            const int count = (cmd & ALPHA_COUNT_MASK) + 1;

            if (fill) {
                if (readPos >= chunk.alphaMap.size()) break;
                const uint8_t val = chunk.alphaMap[readPos++];
                for (int i = 0; i < count && writePos < ALPHA_MAP_SIZE; ++i) {
                    outAlpha[writePos++] = val;
                }
            } else {
                for (int i = 0;
                     i < count && writePos < ALPHA_MAP_SIZE && readPos < chunk.alphaMap.size();
                     ++i) {
                    outAlpha[writePos++] = chunk.alphaMap[readPos++];
                }
            }
        }
        return true;
    }

    if (layerSize >= ALPHA_MAP_SIZE) {
        std::copy(chunk.alphaMap.begin() + static_cast<std::ptrdiff_t>(offset),
                  chunk.alphaMap.begin() + static_cast<std::ptrdiff_t>(offset + ALPHA_MAP_SIZE),
                  outAlpha.begin());
        return true;
    }

    if (layerSize >= ALPHA_MAP_PACKED &&
        offset + ALPHA_MAP_PACKED <= chunk.alphaMap.size()) {
        // 4 bits per texel: low nibble first, scaled 0-15 to 0-255 by 17.
        for (size_t i = 0; i < ALPHA_MAP_PACKED; ++i) {
            const uint8_t v = chunk.alphaMap[offset + i];
            outAlpha[i * 2] = static_cast<uint8_t>((v & 0x0F) * 17);
            outAlpha[i * 2 + 1] = static_cast<uint8_t>((v >> 4) * 17);
        }
        fixLastRowAndColumn(outAlpha);
        return true;
    }

    return false;
}

} // namespace pipeline
} // namespace wowee
