#pragma once

/// The item tooltip, which is not a property of any window.
///
/// It was written inside the bag window and every other surface that shows an
/// item - the action bar, the dialogs, the HUD, the social panel - reached
/// through that window to draw one. What it needs is the item, what to compare
/// it against, and the two services it reads: the game handler for the player's
/// class, race and skills, and the asset manager for the enchantment names.

#include <cstdint>
#include <string>
#include <unordered_map>

namespace wowee {
namespace game {
class GameHandler;
class Inventory;
struct ItemDef;
struct ItemQueryResponseData;
}
namespace pipeline { class AssetManager; }

namespace ui {

/// The tooltip for an item, with the comparison against what is worn when an
/// inventory is given.
void renderItemTooltip(const game::ItemDef& item, const game::Inventory* inventory,
                       uint64_t itemGuid, game::GameHandler* gameHandler,
                       pipeline::AssetManager* assetManager);
void renderItemTooltip(const game::ItemQueryResponseData& info, const game::Inventory* inventory,
                       uint64_t itemGuid, game::GameHandler* gameHandler,
                       pipeline::AssetManager* assetManager);

/// Enchantment id to name, read once from SpellItemEnchantment.dbc.
const std::unordered_map<uint32_t, std::string>& enchantmentNames(
    pipeline::AssetManager* assetManager);

}  // namespace ui
}  // namespace wowee
