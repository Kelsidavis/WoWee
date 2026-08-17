#include "pipeline/adt_alpha.hpp"

#include <algorithm>

namespace wowee {
namespace pipeline {

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
        return true;
    }

    return false;
}

} // namespace pipeline
} // namespace wowee
