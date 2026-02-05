#pragma once

#include "game/inventory.hpp"
#include <imgui.h>

namespace wowee {
namespace ui {

class InventoryScreen {
public:
    void render(game::Inventory& inventory, uint64_t moneyCopper);
    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

    /// Returns true if equipment changed since last call, and clears the flag.
    bool consumeEquipmentDirty() { bool d = equipmentDirty; equipmentDirty = false; return d; }
    /// Returns true if any inventory slot changed since last call, and clears the flag.
    bool consumeInventoryDirty() { bool d = inventoryDirty; inventoryDirty = false; return d; }

private:
    bool open = false;
    bool bKeyWasDown = false;
    bool equipmentDirty = false;
    bool inventoryDirty = false;

    // Drag-and-drop held item state
    bool holdingItem = false;
    game::ItemDef heldItem;
    enum class HeldSource { NONE, BACKPACK, EQUIPMENT };
    HeldSource heldSource = HeldSource::NONE;
    int heldBackpackIndex = -1;
    game::EquipSlot heldEquipSlot = game::EquipSlot::NUM_SLOTS;

    void renderEquipmentPanel(game::Inventory& inventory);
    void renderBackpackPanel(game::Inventory& inventory);

    // Slot rendering with interaction support
    enum class SlotKind { BACKPACK, EQUIPMENT };
    void renderItemSlot(game::Inventory& inventory, const game::ItemSlot& slot,
                        float size, const char* label,
                        SlotKind kind, int backpackIndex,
                        game::EquipSlot equipSlot);
    void renderItemTooltip(const game::ItemDef& item);

    // Held item helpers
    void pickupFromBackpack(game::Inventory& inv, int index);
    void pickupFromEquipment(game::Inventory& inv, game::EquipSlot slot);
    void placeInBackpack(game::Inventory& inv, int index);
    void placeInEquipment(game::Inventory& inv, game::EquipSlot slot);
    void cancelPickup(game::Inventory& inv);
    game::EquipSlot getEquipSlotForType(uint8_t inventoryType, game::Inventory& inv);
    void renderHeldItem();

    static ImVec4 getQualityColor(game::ItemQuality quality);
};

} // namespace ui
} // namespace wowee
