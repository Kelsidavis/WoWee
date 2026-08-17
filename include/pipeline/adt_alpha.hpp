#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "pipeline/adt_loader.hpp"

// MCAL alpha-map decoding.
//
// This lives beside the loader rather than inside a renderer because it is
// knowledge about the file format, and because two copies of it had already
// grown: one in terrain_mesh.cpp and one in terrain_manager.cpp. The grass
// terrain adapter needed the same thing and would have made a third.

namespace wowee {
namespace pipeline {

/// 64x64 texels, one byte each, once decoded.
inline constexpr size_t ALPHA_MAP_DIM = 64;
inline constexpr size_t ALPHA_MAP_SIZE = ALPHA_MAP_DIM * ALPHA_MAP_DIM;
/// The same map at 4 bits per texel, which is how most chunks store it.
inline constexpr size_t ALPHA_MAP_PACKED = ALPHA_MAP_SIZE / 2;
static_assert(ALPHA_MAP_PACKED * 2 == ALPHA_MAP_SIZE,
              "packed alpha must unpack to full size");

/// RLE command byte: high bit selects fill over copy, low seven are the count.
inline constexpr uint8_t ALPHA_FILL_FLAG = 0x80;
inline constexpr uint8_t ALPHA_COUNT_MASK = 0x7F;

/// Decode one layer's alpha map into `outAlpha`, which is resized to
/// ALPHA_MAP_SIZE.
///
/// `unsetFill` is what texels the layer's data does not reach are left as.
/// 255 - fully covered - is what the terrain renderer uses and therefore what
/// the ground actually looks like, so anything deciding what grows where wants
/// the same. It stays a parameter for the mesh builder, which fills with 0 and
/// has always done.
///
/// Returns false when the layer has no alpha map of its own, in which case
/// `outAlpha` is left filled with `unsetFill`. Layer 0 is always the base and
/// never has one.
bool decodeLayerAlpha(const MapChunk& chunk, size_t layerIdx,
                      std::vector<uint8_t>& outAlpha, uint8_t unsetFill = 255);

} // namespace pipeline
} // namespace wowee
