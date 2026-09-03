#pragma once

#include <vulkan/vulkan.h>
#include <cstdint>

#include "game/group_defines.hpp"

namespace wowee {
namespace pipeline { class AssetManager; }
namespace ui {

/// Number of raid target markers: Star, Circle, Diamond, Triangle, Moon,
/// Square, Cross, Skull - in that order, matching the server's icon indices.
inline constexpr uint8_t kRaidTargetIconCount =
    static_cast<uint8_t>(game::kRaidMarkCount);

/**
 * Blizzard raid target marker artwork for an icon index, ready to hand to
 * ImGui's AddImage/Image.
 *
 * These are drawn as textures rather than text symbols because the ImGui font
 * has no code points for most of the marks (skull, cross, moon, ...), so a
 * glyph-based version renders as '?' boxes.
 *
 * Textures are uploaded once and cached for the process. Returns
 * VK_NULL_HANDLE if the icon index is out of range, the BLP is missing, or the
 * upload fails - a failed upload is not cached, so it is retried next frame.
 */
VkDescriptorSet getRaidTargetIcon(uint8_t icon, pipeline::AssetManager* assetManager);

} // namespace ui
} // namespace wowee
