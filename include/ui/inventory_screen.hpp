#pragma once

#include "game/inventory.hpp"
#include "game/character.hpp"
#include "game/world_packets.hpp"
#include <vulkan/vulkan.h>
#include <imgui.h>
#include <algorithm>
#include <array>
#include <functional>
#include <memory>
#include <unordered_map>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering { class CharacterPreview; class CharacterRenderer; }
namespace game { class GameHandler; }
namespace ui {

class InventoryScreen {
public:
    ~InventoryScreen();

    /// Render bags window (B key). Positioned at bottom of screen.

    /// Render character screen (C key). Standalone equipment window.

    /// Draw the item-target cursor armed by using a sharpening stone / weightstone /
    /// weapon oil, and handle cancelling it. Call once per frame, after the windows.
    void renderItemTargetCursor();

    [[nodiscard]] bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

    // Separate bag window controls
    void setSeparateBags(bool sep) { separateBags_ = sep; }
    [[nodiscard]] bool isSeparateBags() const { return separateBags_; }
    void toggleCompactBags() { compactBags_ = !compactBags_; }
    [[nodiscard]] bool isCompactBags() const { return compactBags_; }
    void setShowKeyring(bool show) { showKeyring_ = show; }
    [[nodiscard]] bool isShowKeyring() const { return showKeyring_; }
    void setBagScale(float scale) { bagScale_ = std::clamp(scale, 0.75f, 1.5f); }
    [[nodiscard]] float getBagScale() const { return bagScale_; }
    static float recommendedBagScale(float displayHeight) {
        if (displayHeight >= 2000.0f) return 1.20f;
        if (displayHeight >= 1300.0f) return 1.10f;
        return 1.00f;
    }
    [[nodiscard]] bool isBackpackOpen() const { return backpackOpen_; }
    [[nodiscard]] bool isBagOpen(int idx) const { return idx >= 0 && idx < 4 ? bagOpen_[idx] : false; }

    [[nodiscard]] bool isCharacterOpen() const { return characterOpen; }
    void setCharacterOpen(bool o) { characterOpen = o; }

    void setGameHandler(game::GameHandler* handler) { gameHandler_ = handler; }

    /// Set asset manager for icon/model loading
    void setAssetManager(pipeline::AssetManager* am) { assetManager_ = am; }

    /// Store player appearance for character preview

    /// Mark the character preview as needing equipment update


    /// Returns true if equipment changed since last call, and clears the flag.
    bool consumeEquipmentDirty() { bool d = equipmentDirty; equipmentDirty = false; return d; }
    /// Returns true if any inventory slot changed since last call, and clears the flag.
    bool consumeInventoryDirty() { bool d = inventoryDirty; inventoryDirty = false; return d; }

private:
    bool open = false;
    bool characterOpen = false;
    bool separateBags_ = true;
    bool compactBags_ = false;
    bool showKeyring_ = true;
    float bagScale_ = 1.0f;
    bool backpackOpen_ = false;
    std::array<bool, 4> bagOpen_{};
    bool equipmentDirty = false;
    bool inventoryDirty = false;

    // Vendor sell mode
    game::GameHandler* gameHandler_ = nullptr;

    // Asset manager for icons and preview
    pipeline::AssetManager* assetManager_ = nullptr;

    // Item icon cache: displayInfoId -> GL texture
public:
    VkDescriptorSet getItemIcon(uint32_t displayInfoId);
    void renderItemTooltip(const game::ItemQueryResponseData& info, const game::Inventory* inventory = nullptr, uint64_t itemGuid = 0);
    void renderItemTooltip(const game::ItemDef& item, const game::Inventory* inventory = nullptr, uint64_t itemGuid = 0);
private:

    // Character model preview

    // Stored player appearance for preview


    // Drag-and-drop held item state
    bool holdingItem = false;
    game::ItemDef heldItem;
    enum class HeldSource { NONE, BACKPACK, BAG, EQUIPMENT, BANK, BANK_BAG, BANK_BAG_EQUIP, KEYRING };
    HeldSource heldSource = HeldSource::NONE;
    int heldBackpackIndex = -1;
    int heldBagIndex = -1;
    int heldBagSlotIndex = -1;
    int heldBankIndex = -1;
    int heldBankBagIndex = -1;
    int heldBankBagSlotIndex = -1;
    int heldKeyringIndex = -1;
    game::EquipSlot heldEquipSlot = game::EquipSlot::NUM_SLOTS;

    // Slot rendering with interaction support
    enum class SlotKind { BACKPACK, EQUIPMENT, KEYRING };

    // Frame the item-target cursor was armed on (-1 = not armed). Cancel input is
    // ignored on that frame so the right-click that used the item doesn't cancel it.
    int itemTargetArmedFrame_ = -1;
    /// The targeting cursor's own art. See castCursorTexture().
    VkDescriptorSet castCursorTexture_ = VK_NULL_HANDLE;
    VkDescriptorSet castCursorTexture();

    // Click-and-hold pickup tracking
    static constexpr float kPickupHoldThreshold = 0.10f; // seconds

    /// Shared footer for the backpack / All Bags windows: Sort Bags button + money display.


    // Held item helpers
    void cancelPickup(game::Inventory& inv);

    /// Where the held item came from, as the (container, slot) pair the server
    /// addresses. False when nothing usable is held.
    ///
    /// Four places worked this out and they did not agree on how many places an
    /// item can be picked up from: two knew all seven, one knew five and one
    /// six. The keyring was missing from two of them, so dragging a key onto an
    /// equipment slot or into the bank did nothing at all.
    bool heldItemWireSource(uint8_t& srcBag, uint8_t& srcSlot) const;

    // Drop confirmation (drag-outside-window destroy)

    // Destroy confirmation (Shift+right-click destroy)

    // Stack split popup state

    // Binding-on-equip confirmation. The item remains on the cursor until the
    // player explicitly accepts, including when the destination is a bag slot.

    // ImGui starts window movement before item widgets run for the frame, so
    // keep bag windows title-bar-draggable while bags are open.
    bool bagMoveConfigActive_ = false;
    bool previousMoveFromTitleBarOnly_ = false;
    void setBagMoveConfigActive(bool active);

    // Pending chat item link from shift-click
    std::string pendingChatItemLink_;

public:
    static ImVec4 getQualityColor(game::ItemQuality quality);

    /// Returns true if the user is currently holding an item (pickup cursor).
    [[nodiscard]] bool isHoldingItem() const { return holdingItem; }
    /// Where the held item came from, in the bag and slot the server names.
    /// False when nothing is held or its source cannot be addressed.
    bool heldItemSource(uint8_t& bag, uint8_t& slot) const {
        return holdingItem && heldItemWireSource(bag, slot);
    }
    /// Let go of the held item without putting it anywhere: the trade window
    /// takes it from its own slot, so the cursor is simply cleared.
    void releaseHeldItem() { holdingItem = false; }
    /// Returns the item being held (only valid when isHoldingItem() is true).
    [[nodiscard]] const game::ItemDef& getHeldItem() const { return heldItem; }
    /// Cancel the pickup, returning the item to its original slot.
    void returnHeldItem(game::Inventory& inv) { cancelPickup(inv); }
    /// Returns a WoW item link string if the user shift-clicked a bag item, then clears it.
    std::string getAndClearPendingChatLink() {
        std::string out = std::move(pendingChatItemLink_);
        pendingChatItemLink_.clear();
        return out;
    }

};

} // namespace ui
} // namespace wowee
