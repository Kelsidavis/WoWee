#pragma once

/// Reading a BLP off disk and handing it to Vulkan as a UI texture.
///
/// Eight places in the interface did this: spell icons in the spellbook, the
/// talent tree and the HUD, item icons in the bags, achievement icons, raid
/// target markers, the backpack button and the talent tab backgrounds. The
/// sequence is the same every time - read the file, decode it, find the
/// context, upload - and only the parts around it differ.
///
/// Those parts are deliberately left to the callers, because they disagree
/// on purpose:
///
///   - Some cache a failure so a missing file is not retried every frame.
///     Others cache only success, because their failure is descriptor pool
///     pressure and blacklisting the icon for good would be wrong.
///   - Some take the window from UiServices, others from the Application.
///   - The bag icons log which of the two failures happened, which is why
///     this reports a reason rather than just failing.

#include <cstdint>
#include <string>
#include <unordered_map>

#include <vulkan/vulkan.h>

namespace wowee::core {
class Window;
}

namespace wowee::pipeline {
class AssetManager;
}

namespace wowee::ui {

/// Why an upload produced no texture. Ok is the only value that comes with a
/// usable descriptor set.
enum class UiTextureLoad {
    Ok,
    NotFound,      ///< no such file, or it was empty
    DecodeFailed,  ///< the file is there and is not a readable BLP
    NoContext,     ///< no window or no Vulkan context yet
};

/// Decode `path` as a BLP and upload it as a UI texture.
///
/// Returns VK_NULL_HANDLE on any failure, with `why` set when it is given.
/// Callers own the caching: this does none.
VkDescriptorSet uploadUiTextureFromBlp(pipeline::AssetManager* assetManager,
                                       const std::string& path,
                                       core::Window* window,
                                       UiTextureLoad* why = nullptr);

/// One icon, uploaded at most once and at most one per frame's budget.
///
/// The spellbook and the talent tree ask this the same way and want the same
/// three answers, and each wrote them out: a cached set is returned, a refused
/// upload budget defers *without* caching so the icon is retried next frame,
/// and an icon id with no path caches a null so a missing file is not looked
/// for again every frame the panel is open. Deferring and failing look alike
/// at the call site and must not be cached alike; that distinction is the
/// reason this is one function rather than two copies of four branches.
VkDescriptorSet cachedIconTexture(
    uint32_t iconId, pipeline::AssetManager* assetManager, core::Window* window,
    const std::unordered_map<uint32_t, std::string>& paths,
    std::unordered_map<uint32_t, VkDescriptorSet>& cache);

/// One item icon, by its ItemDisplayInfo id.
///
/// The path comes from ItemDisplayInfo.dbc rather than from a table the caller
/// built, which is the one thing item icons do differently from the spell and
/// talent ones above. The cache is shared by everything that draws an item -
/// bags, the action bar, tooltips, dialogs - because they draw the same items
/// and there is no reason for each to upload its own copy.
VkDescriptorSet itemIconTexture(uint32_t displayInfoId,
                                pipeline::AssetManager* assetManager,
                                core::Window* window);

}  // namespace wowee::ui
