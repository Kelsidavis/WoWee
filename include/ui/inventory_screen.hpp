#pragma once

#include "game/inventory.hpp"
#include "game/character.hpp"
#include "game/world_packets.hpp"
#include <GL/glew.h>
#include <imgui.h>
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
    void render(game::Inventory& inventory, uint64_t moneyCopper);

    /// Render character screen (C key). Standalone equipment window.
    void renderCharacterScreen(game::GameHandler& gameHandler);

    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

    bool isCharacterOpen() const { return characterOpen; }
    void toggleCharacter() { characterOpen = !characterOpen; }
    void setCharacterOpen(bool o) { characterOpen = o; }

    /// Enable vendor mode: right-clicking bag items sells them.
    void setVendorMode(bool enabled, game::GameHandler* handler) {
        vendorMode_ = enabled;
        gameHandler_ = handler;
    }

    /// Set asset manager for icon/model loading
    void setAssetManager(pipeline::AssetManager* am) { assetManager_ = am; }

    /// Store player appearance for character preview
    void setPlayerAppearance(game::Race race, game::Gender gender,
                             uint8_t skin, uint8_t face,
                             uint8_t hairStyle, uint8_t hairColor,
                             uint8_t facialHair);

    /// Mark the character preview as needing equipment update
    void markPreviewDirty() { previewDirty_ = true; }

    /// Update the preview animation (call each frame)
    void updatePreview(float deltaTime);

    /// Returns true if equipment changed since last call, and clears the flag.
    bool consumeEquipmentDirty() { bool d = equipmentDirty; equipmentDirty = false; return d; }
    /// Returns true if any inventory slot changed since last call, and clears the flag.
    bool consumeInventoryDirty() { bool d = inventoryDirty; inventoryDirty = false; return d; }

private:
    bool open = false;
    bool characterOpen = false;
    bool bKeyWasDown = false;
    bool cKeyWasDown = false;
    bool equipmentDirty = false;
    bool inventoryDirty = false;

    // Vendor sell mode
    bool vendorMode_ = false;
    game::GameHandler* gameHandler_ = nullptr;

    // Asset manager for icons and preview
    pipeline::AssetManager* assetManager_ = nullptr;

    // Item icon cache: displayInfoId -> GL texture
    std::unordered_map<uint32_t, GLuint> iconCache_;
public:
    GLuint getItemIcon(uint32_t displayInfoId);
private:

    // Character model preview
    std::unique_ptr<rendering::CharacterPreview> charPreview_;
    bool previewInitialized_ = false;
    bool previewDirty_ = false;

    // Stored player appearance for preview
    game::Race playerRace_ = game::Race::HUMAN;
    game::Gender playerGender_ = game::Gender::MALE;
    uint8_t playerSkin_ = 0;
    uint8_t playerFace_ = 0;
    uint8_t playerHairStyle_ = 0;
    uint8_t playerHairColor_ = 0;
    uint8_t playerFacialHair_ = 0;

    void initPreview();
    void updatePreviewEquipment(game::Inventory& inventory);

    // Drag-and-drop held item state
    bool holdingItem = false;
    game::ItemDef heldItem;
    enum class HeldSource { NONE, BACKPACK, EQUIPMENT };
    HeldSource heldSource = HeldSource::NONE;
    int heldBackpackIndex = -1;
    game::EquipSlot heldEquipSlot = game::EquipSlot::NUM_SLOTS;

    void renderEquipmentPanel(game::Inventory& inventory);
    void renderBackpackPanel(game::Inventory& inventory);
    void renderStatsPanel(game::Inventory& inventory, uint32_t playerLevel);

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

public:
    static ImVec4 getQualityColor(game::ItemQuality quality);
};

} // namespace ui
} // namespace wowee
