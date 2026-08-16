#include "game/reputation_standing.hpp"
#include "ui/ui_texture_load.hpp"
#include "ui/ui_upload_budget.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/item_tooltip.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "game/inventory_slots.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/ui_colors.hpp"
#include "ui/keybinding_manager.hpp"
#include "game/game_handler.hpp"
#include "core/application.hpp"
#include "core/world_loader.hpp"
#include "rendering/vk_context.hpp"
#include "core/input.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <SDL2/SDL.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <unordered_set>

namespace wowee {
namespace ui {

namespace {

// ITEM_FLAG_OPENABLE: the server template, not item class, is authoritative for
// loot containers. Fishing finds such as Message in a Bottle and Tightly Sealed
// Trunk are miscellaneous items that still need CMSG_OPEN_ITEM on right-click.
constexpr uint32_t kItemFlagOpenable = 0x00000004u;

// Keep the complete bag window reachable after monitor/resolution changes or
// a stale imgui.ini position. Merely checking for total off-screen placement
// leaves a narrow sliver visible and makes the title bar almost impossible to
// grab, which is worse than restoring it to the viewport edge.
bool clampCurrentWindowToMainViewport() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    if (!viewport) return false;

    ImVec2 pos = ImGui::GetWindowPos();
    const ImVec2 size = ImGui::GetWindowSize();
    const ImVec2 minPos = viewport->WorkPos;
    const ImVec2 maxPos(
        minPos.x + std::max(0.0f, viewport->WorkSize.x - size.x),
        minPos.y + std::max(0.0f, viewport->WorkSize.y - size.y));
    const ImVec2 clamped(std::clamp(pos.x, minPos.x, maxPos.x),
                         std::clamp(pos.y, minPos.y, maxPos.y));
    if (clamped.x == pos.x && clamped.y == pos.y) return false;

    ImGui::SetWindowPos(clamped);
    return true;
}





// Socket types from shared ui_colors.hpp (ui::kSocketTypes)




} // namespace

InventoryScreen::~InventoryScreen() {
    setBagMoveConfigActive(false);
}

namespace {


}  // namespace

void InventoryScreen::setBagMoveConfigActive(bool active) {
    if (!ImGui::GetCurrentContext()) {
        bagMoveConfigActive_ = false;
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    if (active) {
        if (!bagMoveConfigActive_) {
            previousMoveFromTitleBarOnly_ = io.ConfigWindowsMoveFromTitleBarOnly;
            bagMoveConfigActive_ = true;
        }
        io.ConfigWindowsMoveFromTitleBarOnly = true;
    } else if (bagMoveConfigActive_) {
        io.ConfigWindowsMoveFromTitleBarOnly = previousMoveFromTitleBarOnly_;
        bagMoveConfigActive_ = false;
    }
}

ImVec4 InventoryScreen::getQualityColor(game::ItemQuality quality) {
    return ui::getQualityColor(quality);
}

// ============================================================
// Item Icon Loading
// ============================================================

void InventoryScreen::renderItemTooltip(const game::ItemDef& item,
                                       const game::Inventory* inventory,
                                       uint64_t itemGuid) {
    ui::renderItemTooltip(item, inventory, itemGuid, gameHandler_, assetManager_);
}

void InventoryScreen::renderItemTooltip(const game::ItemQueryResponseData& info,
                                        const game::Inventory* inventory,
                                        uint64_t itemGuid) {
    ui::renderItemTooltip(info, inventory, itemGuid, gameHandler_, assetManager_);
}

VkDescriptorSet InventoryScreen::getItemIcon(uint32_t displayInfoId) {
    // The shared cache, so an item drawn here and on the action bar is
    // uploaded once. See itemIconTexture.
    return itemIconTexture(displayInfoId, assetManager_,
                           core::Application::getInstance().getWindow());
}

// ============================================================
// Character Model Preview
// ============================================================

void InventoryScreen::setPlayerAppearance(game::Race race, game::Gender gender,
                                           uint8_t skin, uint8_t face,
                                           uint8_t hairStyle, uint8_t hairColor,
                                           uint8_t facialHair) {
    playerRace_ = race;
    playerGender_ = gender;
    playerSkin_ = skin;
    playerFace_ = face;
    playerHairStyle_ = hairStyle;
    playerHairColor_ = hairColor;
    playerFacialHair_ = facialHair;
    // Force preview reload on next render
    previewInitialized_ = false;
}

void InventoryScreen::initPreview() {
    if (previewInitialized_ || !assetManager_) return;

    if (!charPreview_) {
        charPreview_ = std::make_unique<rendering::CharacterPreview>();
        if (!charPreview_->initialize(assetManager_)) {
            LOG_WARNING("InventoryScreen: failed to init CharacterPreview");
            charPreview_.reset();
            return;
        }
        auto* renderer = core::Application::getInstance().getRenderer();
        if (renderer) renderer->registerPreview(charPreview_.get());
    }

    charPreview_->loadCharacter(playerRace_, playerGender_,
                                 playerSkin_, playerFace_,
                                 playerHairStyle_, playerHairColor_,
                                 playerFacialHair_);
    previewInitialized_ = true;
    previewDirty_ = true; // apply equipment on first load
}
void InventoryScreen::updatePreviewEquipment(game::Inventory& inventory,
                                              bool showHelm, bool showCloak) {
    if (!charPreview_ || !charPreview_->isModelLoaded()) return;

    std::vector<game::EquipmentItem> equipped;
    equipped.reserve(game::Inventory::NUM_EQUIP_SLOTS);
    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto equipSlot = static_cast<game::EquipSlot>(s);
        if ((!showHelm && equipSlot == game::EquipSlot::HEAD) ||
            (!showCloak && equipSlot == game::EquipSlot::BACK)) {
            continue;
        }
        const auto& slot = inventory.getEquipSlot(equipSlot);
        if (slot.empty() || slot.item.displayInfoId == 0) continue;
        game::EquipmentItem ei;
        ei.displayModel = slot.item.displayInfoId;
        ei.inventoryType = slot.item.inventoryType;
        ei.enchantment = 0;
        equipped.push_back(ei);
    }
    charPreview_->applyEquipment(equipped);
    previewDirty_ = false;
}

// ============================================================
// Equip slot helpers
// ============================================================

game::EquipSlot InventoryScreen::getEquipSlotForType(uint8_t inventoryType, game::Inventory& inv) {
    switch (inventoryType) {
        case 1:  return game::EquipSlot::HEAD;
        case 2:  return game::EquipSlot::NECK;
        case 3:  return game::EquipSlot::SHOULDERS;
        case 4:  return game::EquipSlot::SHIRT;
        case 5:  return game::EquipSlot::CHEST;
        case 6:  return game::EquipSlot::WAIST;
        case 7:  return game::EquipSlot::LEGS;
        case 8:  return game::EquipSlot::FEET;
        case 9:  return game::EquipSlot::WRISTS;
        case 10: return game::EquipSlot::HANDS;
        case 11: {
            if (inv.getEquipSlot(game::EquipSlot::RING1).empty())
                return game::EquipSlot::RING1;
            return game::EquipSlot::RING2;
        }
        case 12: {
            if (inv.getEquipSlot(game::EquipSlot::TRINKET1).empty())
                return game::EquipSlot::TRINKET1;
            return game::EquipSlot::TRINKET2;
        }
        case 13: // One-Hand
        case 21: // Main Hand
            return game::EquipSlot::MAIN_HAND;
        case 17: // Two-Hand
            return game::EquipSlot::MAIN_HAND;
        case 14: // Shield
        case 22: // Off Hand
        case 23: // Held In Off-hand
            return game::EquipSlot::OFF_HAND;
        case 15: // Ranged (bow/gun)
        case 25: // Thrown
        case 26: // Ranged
            return game::EquipSlot::RANGED;
        case 16: return game::EquipSlot::BACK;
        case 18: {
            for (int i = 0; i < game::Inventory::NUM_BAG_SLOTS; ++i) {
                auto slot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + i);
                if (inv.getEquipSlot(slot).empty()) return slot;
            }
            return game::EquipSlot::BAG1;
        }
        case 19: return game::EquipSlot::TABARD;
        case 20: return game::EquipSlot::CHEST; // Robe
        default: return game::EquipSlot::NUM_SLOTS;
    }
}

/// The sound an item makes as it leaves the bag.
///
/// Three of these were loaded at start-up - cloth, food and gems - with no
/// method to play them and nothing choosing between them, so every pickup
/// rustled like a bag whatever it was. The item's class is what decides, as
/// it does in the real client; anything without a sound of its own keeps the
/// bag rustle rather than falling silent.
void InventoryScreen::playPickupSoundFor(const game::ItemDef& item) const {
    auto* app = &core::Application::getInstance();
    auto* ac = app ? app->getAudioCoordinator() : nullptr;
    auto* sfx = ac ? ac->getUiSoundManager() : nullptr;
    if (!sfx || !gameHandler_) return;

    // Item classes, as AzerothCore's ItemTemplate.h numbers them.
    constexpr uint32_t kClassConsumable = 0;
    constexpr uint32_t kClassGem        = 3;
    constexpr uint32_t kClassTradeGoods = 7;
    // Trade goods subclasses: cloth is its own, and leather sounds the same.
    constexpr uint32_t kTradeGoodsCloth   = 5;
    constexpr uint32_t kTradeGoodsLeather = 6;

    const auto* info = gameHandler_->getItemInfo(item.itemId);
    if (!info || !info->valid) { sfx->playPickupBag(); return; }

    if (info->itemClass == kClassGem) {
        sfx->playPickupGem();
    } else if (info->itemClass == kClassConsumable) {
        sfx->playPickupFood();
    } else if (info->itemClass == kClassTradeGoods &&
               (info->subClass == kTradeGoodsCloth ||
                info->subClass == kTradeGoodsLeather)) {
        sfx->playPickupCloth();
    } else {
        sfx->playPickupBag();
    }
}

void InventoryScreen::pickupFromBackpack(game::Inventory& inv, int index) {
    const auto& slot = inv.getBackpackSlot(index);
    if (slot.empty()) return;
    holdingItem = true;
    heldItem = slot.item;
    playPickupSoundFor(heldItem);
    heldSource = HeldSource::BACKPACK;
    heldBackpackIndex = index;
    heldEquipSlot = game::EquipSlot::NUM_SLOTS;
    inv.clearBackpackSlot(index);
    inventoryDirty = true;
}

void InventoryScreen::pickupFromBag(game::Inventory& inv, int bagIndex, int slotIndex) {
    const auto& slot = inv.getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;
    holdingItem = true;
    heldItem = slot.item;
    playPickupSoundFor(heldItem);
    heldSource = HeldSource::BAG;
    heldBackpackIndex = -1;
    heldBagIndex = bagIndex;
    heldBagSlotIndex = slotIndex;
    heldEquipSlot = game::EquipSlot::NUM_SLOTS;
    inv.clearBagSlot(bagIndex, slotIndex);
    inventoryDirty = true;
}

void InventoryScreen::pickupFromEquipment(game::Inventory& inv, game::EquipSlot slot) {
    const auto& es = inv.getEquipSlot(slot);
    if (es.empty()) return;
    holdingItem = true;
    heldItem = es.item;
    playPickupSoundFor(heldItem);
    heldSource = HeldSource::EQUIPMENT;
    heldBackpackIndex = -1;
    heldEquipSlot = slot;
    inv.clearEquipSlot(slot);
    equipmentDirty = true;
    inventoryDirty = true;
}

void InventoryScreen::pickupFromKeyring(game::Inventory& inv, int index) {
    const auto& slot = inv.getKeyringSlot(index);
    if (slot.empty()) return;
    holdingItem = true;
    heldItem = slot.item;
    playPickupSoundFor(heldItem);
    heldSource = HeldSource::KEYRING;
    heldBackpackIndex = -1;
    heldKeyringIndex = index;
    heldEquipSlot = game::EquipSlot::NUM_SLOTS;
    inv.clearKeyringSlot(index);
    inventoryDirty = true;
}




bool InventoryScreen::heldItemWireSource(uint8_t& srcBag, uint8_t& srcSlot) const {
    srcBag = 0xFF;
    srcSlot = 0;
    switch (heldSource) {
        case HeldSource::BACKPACK:
            if (heldBackpackIndex < 0) return false;
            srcSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(heldBackpackIndex));
            return true;
        case HeldSource::BAG:
            if (heldBagIndex < 0 || heldBagSlotIndex < 0) return false;
            srcBag = static_cast<uint8_t>(game::slots::wornBagContainer(heldBagIndex));
            srcSlot = static_cast<uint8_t>(heldBagSlotIndex);
            return true;
        case HeldSource::EQUIPMENT:
            if (heldEquipSlot == game::EquipSlot::NUM_SLOTS) return false;
            srcSlot = static_cast<uint8_t>(heldEquipSlot);
            return true;
        case HeldSource::BANK:
            if (heldBankIndex < 0) return false;
            srcSlot = static_cast<uint8_t>(game::slots::bankGeneralWireSlot(heldBankIndex));
            return true;
        case HeldSource::BANK_BAG:
            if (heldBankBagIndex < 0 || heldBankBagSlotIndex < 0) return false;
            srcBag = static_cast<uint8_t>(game::slots::bankBagContainer(heldBankBagIndex));
            srcSlot = static_cast<uint8_t>(heldBankBagSlotIndex);
            return true;
        case HeldSource::BANK_BAG_EQUIP:
            if (heldBankBagIndex < 0) return false;
            srcSlot = static_cast<uint8_t>(game::slots::bankBagWireSlot(heldBankBagIndex));
            return true;
        case HeldSource::KEYRING:
            if (heldKeyringIndex < 0) return false;
            srcSlot = static_cast<uint8_t>(game::slots::keyringWireSlot(heldKeyringIndex));
            return true;
        case HeldSource::NONE:
            break;
    }
    return false;
}

void InventoryScreen::placeInBackpack(game::Inventory& inv, int index) {
    if (!holdingItem) return;
    if (gameHandler_) {
        // Online mode: send server swap packet for all container moves
        uint8_t dstBag = 0xFF;
        uint8_t dstSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(index));
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (!heldItemWireSource(srcBag, srcSlot)) {
            cancelPickup(inv);
            return;
        }
        gameHandler_->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
        cancelPickup(inv);
        return;
    }
    const auto& target = inv.getBackpackSlot(index);
    if (target.empty()) {
        inv.setBackpackSlot(index, heldItem);
        holdingItem = false;
    } else {
        // Swap
        game::ItemDef targetItem = target.item;
        inv.setBackpackSlot(index, heldItem);
        heldItem = targetItem;
        heldSource = HeldSource::BACKPACK;
        heldBackpackIndex = index;
    }
    inventoryDirty = true;
}

void InventoryScreen::placeInBag(game::Inventory& inv, int bagIndex, int slotIndex) {
    if (!holdingItem) return;
    if (gameHandler_) {
        // Online mode: send server swap packet
        uint8_t dstBag = static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex));
        uint8_t dstSlot = static_cast<uint8_t>(slotIndex);
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (!heldItemWireSource(srcBag, srcSlot)) {
            cancelPickup(inv);
            return;
        }
        gameHandler_->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
        cancelPickup(inv);
        return;
    }
    const auto& target = inv.getBagSlot(bagIndex, slotIndex);
    if (target.empty()) {
        inv.setBagSlot(bagIndex, slotIndex, heldItem);
        holdingItem = false;
    } else {
        game::ItemDef targetItem = target.item;
        inv.setBagSlot(bagIndex, slotIndex, heldItem);
        heldItem = targetItem;
        heldSource = HeldSource::BAG;
        heldBagIndex = bagIndex;
        heldBagSlotIndex = slotIndex;
    }
    inventoryDirty = true;
}

void InventoryScreen::placeInKeyring(game::Inventory& inv, int index) {
    if (!holdingItem) return;
    if (gameHandler_) {
        uint8_t dstBag = 0xFF;
        uint8_t dstSlot = static_cast<uint8_t>(game::slots::keyringWireSlot(index));
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (heldSource == HeldSource::KEYRING && heldKeyringIndex >= 0) {
            srcSlot = static_cast<uint8_t>(game::slots::keyringWireSlot(heldKeyringIndex));
        } else if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
            srcSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(heldBackpackIndex));
        } else if (heldSource == HeldSource::BAG && heldBagIndex >= 0 && heldBagSlotIndex >= 0) {
            srcBag = static_cast<uint8_t>(game::slots::wornBagContainer(heldBagIndex));
            srcSlot = static_cast<uint8_t>(heldBagSlotIndex);
        } else {
            // Only keys (from bags/backpack/keyring) belong in the keyring; the
            // server rejects anything else. Other sources just cancel.
            cancelPickup(inv);
            return;
        }
        if (srcBag == dstBag && srcSlot == dstSlot) {
            cancelPickup(inv);
            return;
        }
        gameHandler_->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
        cancelPickup(inv);
        return;
    }
    const auto& target = inv.getKeyringSlot(index);
    if (target.empty()) {
        inv.setKeyringSlot(index, heldItem);
        holdingItem = false;
    } else {
        game::ItemDef targetItem = target.item;
        inv.setKeyringSlot(index, heldItem);
        heldItem = targetItem;
        heldSource = HeldSource::KEYRING;
        heldKeyringIndex = index;
    }
    inventoryDirty = true;
}

void InventoryScreen::placeInEquipment(game::Inventory& inv, game::EquipSlot slot) {
    if (!holdingItem) return;

    // Only prompt for a BoE item that has not already bound. Once soulbound, re-equipping
    // (slot swaps, unequip/re-equip) must not ask again. The rule itself lives on
    // InventoryHandler, where the path FrameXML takes can reach it too.
    if (heldItem.wouldBindOnEquip() && !equipConfirmOpen_) {
        equipConfirmOpen_ = true;
        equipConfirmAuto_ = false;
        equipConfirmSlot_ = slot;
        equipConfirmItemName_ = heldItem.name;
        return;
    }

    // Validate: check if the held item can go in this slot
    if (heldItem.inventoryType > 0) {
        bool valid = false;
        if (heldItem.inventoryType == 18) {
            valid = (slot >= game::EquipSlot::BAG1 && slot <= game::EquipSlot::BAG4);
        } else {
            game::EquipSlot validSlot = getEquipSlotForType(heldItem.inventoryType, inv);
            if (validSlot == game::EquipSlot::NUM_SLOTS) return;

            valid = (slot == validSlot);
            if (!valid) {
                if (heldItem.inventoryType == 11)
                    valid = (slot == game::EquipSlot::RING1 || slot == game::EquipSlot::RING2);
                else if (heldItem.inventoryType == 12)
                    valid = (slot == game::EquipSlot::TRINKET1 || slot == game::EquipSlot::TRINKET2);
            }
        }
        if (!valid) return;
    } else {
        return;
    }

    if (gameHandler_) {
        uint8_t dstBag = 0xFF;
        uint8_t dstSlot = static_cast<uint8_t>(slot);
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (!heldItemWireSource(srcBag, srcSlot)) {
            cancelPickup(inv);
            return;
        }

        if (srcBag == dstBag && srcSlot == dstSlot) {
            cancelPickup(inv);
            return;
        }

        gameHandler_->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
        cancelPickup(inv);
        return;
    }

    const auto& target = inv.getEquipSlot(slot);
    if (target.empty()) {
        inv.setEquipSlot(slot, heldItem);
        holdingItem = false;
    } else {
        game::ItemDef targetItem = target.item;
        inv.setEquipSlot(slot, heldItem);
        heldItem = targetItem;
        heldSource = HeldSource::EQUIPMENT;
        heldEquipSlot = slot;
    }

    // Two-handed weapon in main hand clears the off-hand slot
    if (slot == game::EquipSlot::MAIN_HAND &&
        inv.getEquipSlot(game::EquipSlot::MAIN_HAND).item.inventoryType == 17) {
        const auto& offHand = inv.getEquipSlot(game::EquipSlot::OFF_HAND);
        if (!offHand.empty()) {
            inv.addItem(offHand.item);
            inv.clearEquipSlot(game::EquipSlot::OFF_HAND);
        }
    }

    // Equipping off-hand unequips a 2H weapon from main hand
    if (slot == game::EquipSlot::OFF_HAND &&
        inv.getEquipSlot(game::EquipSlot::MAIN_HAND).item.inventoryType == 17) {
        inv.addItem(inv.getEquipSlot(game::EquipSlot::MAIN_HAND).item);
        inv.clearEquipSlot(game::EquipSlot::MAIN_HAND);
    }

    equipmentDirty = true;
    inventoryDirty = true;
}

void InventoryScreen::cancelPickup(game::Inventory& inv) {
    if (!holdingItem) return;
    if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
        if (inv.getBackpackSlot(heldBackpackIndex).empty()) {
            inv.setBackpackSlot(heldBackpackIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::BAG && heldBagIndex >= 0 && heldBagSlotIndex >= 0) {
        if (inv.getBagSlot(heldBagIndex, heldBagSlotIndex).empty()) {
            inv.setBagSlot(heldBagIndex, heldBagSlotIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::EQUIPMENT && heldEquipSlot != game::EquipSlot::NUM_SLOTS) {
        if (inv.getEquipSlot(heldEquipSlot).empty()) {
            inv.setEquipSlot(heldEquipSlot, heldItem);
            equipmentDirty = true;
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::BANK && heldBankIndex >= 0) {
        if (inv.getBankSlot(heldBankIndex).empty()) {
            inv.setBankSlot(heldBankIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::BANK_BAG && heldBankBagIndex >= 0 && heldBankBagSlotIndex >= 0) {
        if (inv.getBankBagSlot(heldBankBagIndex, heldBankBagSlotIndex).empty()) {
            inv.setBankBagSlot(heldBankBagIndex, heldBankBagSlotIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::BANK_BAG_EQUIP && heldBankBagIndex >= 0) {
        if (inv.getBankBagItem(heldBankBagIndex).empty()) {
            inv.setBankBagItem(heldBankBagIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else if (heldSource == HeldSource::KEYRING && heldKeyringIndex >= 0) {
        if (inv.getKeyringSlot(heldKeyringIndex).empty()) {
            inv.setKeyringSlot(heldKeyringIndex, heldItem);
        } else {
            inv.addItem(heldItem);
        }
    } else {
        inv.addItem(heldItem);
    }
    holdingItem = false;
    inventoryDirty = true;
}

/// Interface\\Cursor\\Cast.blp, loaded once.
///
/// Nothing in the client used the cursor art at all, so every cursor that
/// meant something drew whatever the calling code had to hand.
VkDescriptorSet InventoryScreen::castCursorTexture() {
    if (!castCursorTexture_) {
        if (assetManager_ && assetManager_->isInitialized()) {
            castCursorTexture_ = uploadUiTextureFromBlp(
                assetManager_, "Interface\\Cursor\\Cast.blp",
                core::Application::getInstance().getWindow());
        }
    }
    return castCursorTexture_;
}

void InventoryScreen::renderItemTargetCursor() {
    // Both kinds of pending use: one waiting for an item to apply to, one
    // waiting for a unit. The cursor and the way out of it are the same.
    const bool awaitingItem = gameHandler_ && gameHandler_->isAwaitingItemTarget();
    const bool awaitingUnit = gameHandler_ && gameHandler_->isAwaitingUnitTarget();
    if (!awaitingItem && !awaitingUnit) {
        itemTargetArmedFrame_ = -1;
        return;
    }
    if (itemTargetArmedFrame_ < 0) itemTargetArmedFrame_ = ImGui::GetFrameCount();

    // Escape or right-click abandons the pending use. Skipped on the arming frame,
    // where the right-click that used the item is still being handled.
    if (itemTargetArmedFrame_ != ImGui::GetFrameCount() && !ImGui::GetIO().WantTextInput) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) ||
            core::Input::getInstance().isKeyPressed(SDL_SCANCODE_ESCAPE)) {
            gameHandler_->cancelItemTargeting();
            gameHandler_->cancelUnitTargeting();
            itemTargetArmedFrame_ = -1;
            return;
        }
    }

    ImVec2 mousePos = ImGui::GetIO().MousePos;
    constexpr float size = 36.0f;
    ImVec2 pos(mousePos.x - size * 0.5f, mousePos.y - size * 0.5f);
    ImDrawList* drawList = ImGui::GetForegroundDrawList();

    // The client's own cursor art, which is what WoW puts on a cursor waiting
    // for a target. This drew the item's icon with a green box around it - the
    // picture of the thing being used rather than the instruction to pick
    // something - and Interface\\Cursor\\Cast.blp says it properly.
    VkDescriptorSet castCursor = castCursorTexture();
    if (castCursor) {
        drawList->AddImage((ImTextureID)(uintptr_t)castCursor, pos,
                           ImVec2(pos.x + size, pos.y + size));
    } else {
        // Only until the art resolves, and still readable as "pick something".
        drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size), IM_COL32(40, 35, 30, 200));
        drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                          IM_COL32(0, 220, 0, 230), 0.0f, 0, 2.0f);
    }

    const char* hint = awaitingUnit ? "Select a target" : "Select an item";
    ImVec2 hintSize = ImGui::CalcTextSize(hint);
    ImVec2 hintPos(mousePos.x - hintSize.x * 0.5f, pos.y + size + 4.0f);
    drawList->AddRectFilled(ImVec2(hintPos.x - 3.0f, hintPos.y - 2.0f),
                            ImVec2(hintPos.x + hintSize.x + 3.0f, hintPos.y + hintSize.y + 2.0f),
                            IM_COL32(0, 0, 0, 180));
    drawList->AddText(hintPos, IM_COL32(0, 255, 0, 255), hint);
}

void InventoryScreen::renderHeldItem() {
    if (!holdingItem) return;

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;
    float size = 36.0f;
    ImVec2 pos(mousePos.x - size * 0.5f, mousePos.y - size * 0.5f);

    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    ImVec4 qColor = getQualityColor(heldItem.quality);
    ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(qColor);

    // Try to show icon
    VkDescriptorSet iconTex = getItemIcon(heldItem.displayInfoId);
    if (iconTex) {
        drawList->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                           ImVec2(pos.x + size, pos.y + size));
        drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                          borderCol, 0.0f, 0, 2.0f);
    } else {
        drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size),
                                IM_COL32(40, 35, 30, 200));
        drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                          borderCol, 0.0f, 0, 2.0f);

        char abbr[4] = {};
        if (!heldItem.name.empty()) {
            abbr[0] = heldItem.name[0];
            if (heldItem.name.size() > 1) abbr[1] = heldItem.name[1];
        }
        float textW = ImGui::CalcTextSize(abbr).x;
        drawList->AddText(ImVec2(pos.x + (size - textW) * 0.5f, pos.y + 2.0f),
                          ImGui::ColorConvertFloat4ToU32(qColor), abbr);
    }

    if (heldItem.stackCount > 1) {
        char countStr[16];
        snprintf(countStr, sizeof(countStr), "%u", heldItem.stackCount);
        float cw = ImGui::CalcTextSize(countStr).x;
        drawList->AddText(ImVec2(pos.x + size - cw - 2.0f, pos.y + size - 14.0f),
                          IM_COL32(255, 255, 255, 220), countStr);
    }
}




// ============================================================
// Bags window (B key) - bottom of screen, no equipment panel
// ============================================================

void InventoryScreen::toggleBackpack() {
    backpackOpen_ = !backpackOpen_;
}

void InventoryScreen::toggleBag(int idx) {
    if (idx >= 0 && idx < 4) {
        bagOpen_[idx] = !bagOpen_[idx];
    }
}

void InventoryScreen::openAllBags() {
    backpackOpen_ = true;
    for (auto& b : bagOpen_) b = true;
}

void InventoryScreen::closeAllBags() {
    backpackOpen_ = false;
    for (auto& b : bagOpen_) b = false;
}

void InventoryScreen::render(game::Inventory& inventory, uint64_t moneyCopper) {
    // Bags toggle (B key, edge-triggered)
    bool bagsDown = KeybindingManager::getInstance().isActionPressed(
        KeybindingManager::Action::TOGGLE_BAGS, false);
    bool bToggled = bagsDown && !bKeyWasDown;
    bKeyWasDown = bagsDown;

    bool wantsTextInput = ImGui::GetIO().WantTextInput;

    if (separateBags_) {
        if (bToggled) {
            bool anyOpen = backpackOpen_;
            for (auto b : bagOpen_) anyOpen |= b;
            if (anyOpen) closeAllBags();
            else openAllBags();
        }
        open = backpackOpen_ || std::any_of(bagOpen_.begin(), bagOpen_.end(), [](bool b){ return b; });
    } else {
        if (bToggled) open = !open;
    }

    if (!open) {
        setBagMoveConfigActive(false);
        if (holdingItem) cancelPickup(inventory);
        return;
    }

    setBagMoveConfigActive(true);

    // Escape cancels held item
    if (holdingItem && !wantsTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_ESCAPE)) {
        cancelPickup(inventory);
    }

    // Right-click anywhere while holding = cancel
    if (holdingItem && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        cancelPickup(inventory);
    }

    // Cancel pending pickup if mouse released before threshold
    if (pickupPending_ && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        pickupPending_ = false;
    }

    if (separateBags_) {
        renderSeparateBags(inventory, moneyCopper);
    } else {
        renderAggregateBags(inventory, moneyCopper);
    }

    // Detect held item dropped outside inventory windows → drop confirmation
    if (holdingItem && heldItem.itemId != 6948 && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
        !ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) &&
        !ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive()) {
        dropConfirmOpen_ = true;
        dropItemName_ = heldItem.name;
    }

    // Drop item confirmation popup - positioned near cursor
    if (dropConfirmOpen_) {
        ImVec2 mousePos = ImGui::GetIO().MousePos;
        ImGui::SetNextWindowPos(ImVec2(mousePos.x - 80.0f, mousePos.y - 20.0f), ImGuiCond_Always);
        ImGui::OpenPopup("##DropItem");
        dropConfirmOpen_ = false;
    }
    if (ImGui::BeginPopup("##DropItem", ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::Text("Destroy \"%s\"?", dropItemName_.c_str());
        ImGui::Spacing();
        if (ImGui::Button("Yes", ImVec2(80, 0))) {
            if (gameHandler_) {
                // Deliberately not heldItemWireSource: that answers for all
                // seven places an item can be held, and this destroys the item.
                // Whether a bank slot or the keyring should be destroyable from
                // here is a question about the server, not about this code, so
                // widening it silently is the one thing not to do.
                uint8_t srcBag = 0xFF;
                uint8_t srcSlot = 0;
                bool haveSource = false;
                if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
                    srcSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(heldBackpackIndex));
                    haveSource = true;
                } else if (heldSource == HeldSource::BAG && heldBagIndex >= 0 && heldBagSlotIndex >= 0) {
                    srcBag = static_cast<uint8_t>(game::slots::wornBagContainer(heldBagIndex));
                    srcSlot = static_cast<uint8_t>(heldBagSlotIndex);
                    haveSource = true;
                } else if (heldSource == HeldSource::EQUIPMENT &&
                           heldEquipSlot != game::EquipSlot::NUM_SLOTS) {
                    srcSlot = static_cast<uint8_t>(heldEquipSlot);
                    haveSource = true;
                }
                if (haveSource) {
                    uint8_t destroyCount = static_cast<uint8_t>(std::clamp<uint32_t>(
                        std::max<uint32_t>(1u, heldItem.stackCount), 1u, 255u));
                    gameHandler_->destroyItem(srcBag, srcSlot, destroyCount);
                }
            }
            holdingItem = false;
            heldItem = game::ItemDef{};
            heldSource = HeldSource::NONE;
            heldBackpackIndex = -1;
            heldBagIndex = -1;
            heldBagSlotIndex = -1;
            heldEquipSlot = game::EquipSlot::NUM_SLOTS;
            inventoryDirty = true;
            dropItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("No", ImVec2(80, 0))) {
            cancelPickup(inventory);
            dropItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Shift+right-click destroy confirmation popup
    if (destroyConfirmOpen_) {
        ImVec2 mousePos = ImGui::GetIO().MousePos;
        ImGui::SetNextWindowPos(ImVec2(mousePos.x - 80.0f, mousePos.y - 20.0f), ImGuiCond_Always);
        ImGui::OpenPopup("##DestroyItem");
        destroyConfirmOpen_ = false;
    }
    if (ImGui::BeginPopup("##DestroyItem", ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::TextColored(ui::colors::kSoftRed, "Destroy");
        ImGui::TextUnformatted(destroyItemName_.c_str());
        ImGui::Spacing();
        if (ImGui::Button("Yes, Destroy", ImVec2(110, 0))) {
            if (gameHandler_) {
                gameHandler_->destroyItem(destroyBag_, destroySlot_, destroyCount_);
            }
            destroyItemName_.clear();
            inventoryDirty = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(70, 0))) {
            destroyItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Stack split popup
    if (splitConfirmOpen_) {
        ImVec2 mousePos = ImGui::GetIO().MousePos;
        ImGui::SetNextWindowPos(ImVec2(mousePos.x - 80.0f, mousePos.y - 20.0f), ImGuiCond_Always);
        ImGui::OpenPopup("##SplitStack");
        splitConfirmOpen_ = false;
    }
    if (ImGui::BeginPopup("##SplitStack", ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::Text("Split %s", splitItemName_.c_str());
        ImGui::Spacing();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::SliderInt("##splitcount", &splitCount_, 1, splitMax_ - 1);
        ImGui::Spacing();
        if (ImGui::Button("OK", ImVec2(55, 0))) {
            if (gameHandler_ && splitCount_ > 0 && splitCount_ < splitMax_) {
                gameHandler_->splitItem(splitBag_, splitSlot_, static_cast<uint8_t>(splitCount_));
            }
            splitItemName_.clear();
            inventoryDirty = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(55, 0))) {
            splitItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    renderEquipConfirmationPopup(inventory);
    // Draw held item at cursor
    renderHeldItem();
}

void InventoryScreen::renderEquipConfirmationPopup(game::Inventory& inventory) {
    if (equipConfirmOpen_) {
        ImVec2 mousePos = ImGui::GetIO().MousePos;
        ImGui::SetNextWindowPos(ImVec2(mousePos.x - 120.0f, mousePos.y - 30.0f), ImGuiCond_Always);
        ImGui::OpenPopup("##BindOnEquipConfirm");
        equipConfirmOpen_ = false;
    }
    if (ImGui::BeginPopup("##BindOnEquipConfirm", ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::TextWrapped("Equip \"%s\"?", equipConfirmItemName_.c_str());
        ImGui::TextColored(ui::colors::kSoftRed, "This item will bind to you.");
        ImGui::Spacing();
        if (ImGui::Button("Equip", ImVec2(85, 0))) {
            if (equipConfirmAuto_ && gameHandler_) {
                // Already asked, so say so: the handler prompts too, and
                // without this the answer to the prompt raises it again.
                if (equipConfirmBag_ == 0xFF)
                    gameHandler_->autoEquipItemBySlot(static_cast<int>(equipConfirmSourceSlot_ - game::Inventory::NUM_EQUIP_SLOTS), true);
                else
                    gameHandler_->autoEquipItemInBag(equipConfirmBag_, equipConfirmSourceSlot_, true);
            } else if (holdingItem) {
                equipConfirmOpen_ = true;
                placeInEquipment(inventory, equipConfirmSlot_);
            }
            equipConfirmItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(85, 0))) {
            equipConfirmItemName_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// ============================================================
// Aggregate mode - original single-window bags
// ============================================================

void InventoryScreen::renderAggregateBags(game::Inventory& inventory, uint64_t moneyCopper) {
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    const float scale = bagScale_;
    const float slotSize = 40.0f * scale;
    constexpr int columns = 6;
    int totalSlots = inventory.getBackpackSize();
    int usedSlots = 0;
    for (int slot = 0; slot < inventory.getBackpackSize(); ++slot)
        usedSlots += !inventory.getBackpackSlot(slot).empty();
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; bag++) {
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) continue;
        totalSlots += bagSize;
        for (int slot = 0; slot < bagSize; ++slot)
            usedSlots += !inventory.getBagSlot(bag, slot).empty();
    }

    int rows = (totalSlots + columns - 1) / columns;
    float bagContentH = rows * (slotSize + 4.0f * scale) + 54.0f * scale;
    int visibleKeySlots = 0;
    if (showKeyring_) {
        constexpr int keyColumns = 8;
        int lastOccupied = -1;
        for (int slot = inventory.getKeyringSize() - 1; slot >= 0; --slot) {
            if (!inventory.getKeyringSlot(slot).empty()) { lastOccupied = slot; break; }
        }
        // Always show at least one row when the keyring is enabled, so it stays
        // visible (and a drop target) even when empty.
        visibleKeySlots = ((lastOccupied < 0 ? 0 : lastOccupied) / keyColumns + 1) * keyColumns;
        if (visibleKeySlots > 0) {
            const float keySlotSize = 24.0f * scale;
            const int keyRows = (visibleKeySlots + keyColumns - 1) / keyColumns;
            bagContentH += 30.0f * scale + keyRows * (keySlotSize + 4.0f * scale);
        }
    }

    float windowW = columns * (slotSize + 4.0f * scale) + 30.0f * scale;
    float windowH = bagContentH + 50.0f * scale;

    float posX = screenW - windowW - 10.0f;
    float posY = screenH - windowH - 60.0f;

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
    if (holdingItem || pickupPending_) flags |= ImGuiWindowFlags_NoMove;

    char windowTitle[64];
    snprintf(windowTitle, sizeof(windowTitle), "All Bags (%d/%d)###Bags", usedSlots, totalSlots);
    bool windowVisible = ImGui::Begin(windowTitle, &open, flags);
    if (!windowVisible) {
        ImGui::End();
        return;
    }

    clampCurrentWindowToMainViewport();

    // Draw one uninterrupted grid while retaining each slot's real container
    // and index for pickup, use, split, destroy, and server swap operations.
    // Special containers (quivers, ammo pouches, profession bags) only accept
    // their own item type, so their slots get a label and an amber wash.
    int gridIndex = 0;
    auto renderCombinedSlot = [&](const game::ItemSlot& slot, int backpackIndex,
                                  int bagIndex, int bagSlotIndex,
                                  const char* restrictedLabel) {
        if (gridIndex % columns != 0) ImGui::SameLine();
        ImGui::PushID(gridIndex);
        renderItemSlot(inventory, slot, slotSize, restrictedLabel,
                       SlotKind::BACKPACK, backpackIndex, game::EquipSlot::NUM_SLOTS,
                       bagIndex, bagSlotIndex);
        if (restrictedLabel && slot.empty()) {
            ImVec2 mn = ImGui::GetItemRectMin();
            ImVec2 mx = ImGui::GetItemRectMax();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(mn, mx, IM_COL32(180, 140, 40, 40));
            dl->AddRect(mn, mx, IM_COL32(180, 140, 40, 130));
        }
        ImGui::PopID();
        ++gridIndex;
    };

    for (int slot = 0; slot < inventory.getBackpackSize(); ++slot)
        renderCombinedSlot(inventory.getBackpackSlot(slot), slot, -1, -1, nullptr);
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; ++bag) {
        const int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) continue;
        const char* restrictedLabel = nullptr;
        if (inventory.isBagSpecial(bag)) {
            game::EquipSlot bagEquip = static_cast<game::EquipSlot>(
                static_cast<int>(game::EquipSlot::BAG1) + bag);
            const auto& bagItem = inventory.getEquipSlot(bagEquip);
            restrictedLabel = (!bagItem.empty() && !bagItem.item.subclassName.empty())
                ? bagItem.item.subclassName.c_str()
                : "Special Bag";
        }
        for (int slot = 0; slot < bagSize; ++slot)
            renderCombinedSlot(inventory.getBagSlot(bag, slot), -1, bag, slot, restrictedLabel);
    }

    if (visibleKeySlots > 0) {
        const float keySlotSize = 24.0f * scale;
        constexpr int keyColumns = 8;
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(ui::colors::kDarkYellow, "Keyring");
        for (int slot = 0; slot < visibleKeySlots; ++slot) {
            if (slot % keyColumns != 0) ImGui::SameLine();
            ImGui::PushID(10000 + slot);
            renderItemSlot(inventory, inventory.getKeyringSlot(slot), keySlotSize, nullptr,
                           SlotKind::KEYRING, -1, game::EquipSlot::NUM_SLOTS, -1, -1, slot);
            ImGui::PopID();
        }
    }

    renderBagsFooter(moneyCopper);
    ImGui::End();
}

// ============================================================
// Separate mode - individual draggable bag windows
// ============================================================

void InventoryScreen::renderSeparateBags(game::Inventory& inventory, uint64_t moneyCopper) {
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    const float scale = bagScale_;
    const float slotSize = 40.0f * scale;
    constexpr int columns = 6;
    const float baseWindowW = columns * (slotSize + 4.0f * scale) + 30.0f * scale;

    // Each bag window is independently closable - no forced backpack constraint.

    // Anchor stack to the bag bar (bottom-right), opening upward.
    const float bagBarTop = screenH - (42.0f + 12.0f) - 10.0f;
    const float stackGap = 8.0f;
    float stackBottom = bagBarTop - stackGap;
    float stackX = screenW - baseWindowW - 10.0f;

    // Where the next bag of this height goes, starting a new column to the
    // left when it would run off the top. Without the wrap a fifth bag simply
    // went above the screen; with every bag in one column and no wrap, tall
    // bags overlapped the ones already placed.
    //
    // The real client tiles the same way - updateContainerFrameAnchors fills a
    // column upward from the bottom right and steps left when the next bag no
    // longer fits.
    auto placeNext = [&](float height) {
        if (stackBottom - height < 0.0f && stackBottom < bagBarTop - stackGap) {
            stackX -= baseWindowW + stackGap;
            stackBottom = bagBarTop - stackGap;
        }
        const float y = stackBottom - height;
        stackBottom = y - stackGap;
        return y;
    };

    // Backpack window (bottom of stack)
    if (backpackOpen_) {
        int bpTotal = inventory.getBackpackSize();
        int bpUsed = 0;
        for (int i = 0; i < bpTotal; ++i) if (!inventory.getBackpackSlot(i).empty()) ++bpUsed;
        char bpTitle[64];
        snprintf(bpTitle, sizeof(bpTitle), "Backpack (%d/%d)###backpack", bpUsed, bpTotal);
        int bpRows = (bpTotal + columns - 1) / columns;
        float bpH = bpRows * (slotSize + 4.0f * scale) + 80.0f * scale;
        if (showKeyring_) {
            const float keySlotSize = 24.0f * scale;
            constexpr int keyCols = 8;
            int lastOccupied = -1;
            for (int i = inventory.getKeyringSize() - 1; i >= 0; --i) {
                if (!inventory.getKeyringSlot(i).empty()) { lastOccupied = i; break; }
            }
            {
                int visibleKeySlots = ((lastOccupied < 0 ? 0 : lastOccupied) / keyCols + 1) * keyCols;
                int keyRows = (visibleKeySlots + keyCols - 1) / keyCols;
                bpH += 30.0f * scale + keyRows * (keySlotSize + 4.0f * scale);
            }
        }
        const float bpX = stackX;
        const float bpY = placeNext(bpH);
        renderBagWindow(bpTitle, backpackOpen_, inventory, -1, bpX, bpY, moneyCopper);
    }

    // Extra bag windows in right-to-left bag-bar order (closest to backpack first).
    constexpr int kBagOrder[game::Inventory::NUM_BAG_SLOTS] = {3, 2, 1, 0};
    for (int bag : kBagOrder) {
        if (!bagOpen_[bag]) continue;
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) {
            bagOpen_[bag] = false;
            continue;
        }
        // In separate-bag mode, never auto-hide empty bags. Players still need
        // to open empty bags to move items into them.

        int bagRows = (bagSize + columns - 1) / columns;
        float bagH = bagRows * (slotSize + 4.0f * scale) + 60.0f * scale;
        const float bagX = stackX;
        const float defaultY = placeNext(bagH);

        // Build title from equipped bag item name, with used/total slot counts
        int bagUsed = 0;
        for (int si = 0; si < bagSize; ++si) if (!inventory.getBagSlot(bag, si).empty()) ++bagUsed;
        char title[96];
        game::EquipSlot bagSlot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + bag);
        const auto& bagItem = inventory.getEquipSlot(bagSlot);
        if (!bagItem.empty() && !bagItem.item.name.empty()) {
            snprintf(title, sizeof(title), "%s (%d/%d)###bag%d", bagItem.item.name.c_str(), bagUsed, bagSize, bag);
        } else {
            snprintf(title, sizeof(title), "Bag Slot %d (%d/%d)###bag%d", bag + 1, bagUsed, bagSize, bag);
        }

        renderBagWindow(title, bagOpen_[bag], inventory, bag, bagX, defaultY, 0);
    }

    // Update open state based on individual windows
    open = backpackOpen_ || std::any_of(bagOpen_.begin(), bagOpen_.end(), [](bool b){ return b; });
}

void InventoryScreen::renderBagWindow(const char* title, bool& isOpen,
                                       game::Inventory& inventory, int bagIndex,
                                       float defaultX, float defaultY, uint64_t moneyCopper) {
    const float scale = bagScale_;
    const float slotSize = 40.0f * scale;
    constexpr int columns = 6;

    int numSlots = (bagIndex < 0) ? inventory.getBackpackSize() : inventory.getBagSize(bagIndex);
    if (numSlots <= 0) return;

    int rows = (numSlots + columns - 1) / columns;
    float contentH = rows * (slotSize + 4.0f * scale) + 10.0f * scale;
    if (bagIndex < 0) {
        // Keyring renders at 24px in 8 columns and ONLY shows rows that have
        // occupied slots (rounded up to a full row of 8) - must match the
        // render logic below or we reserve huge empty space.
        const float keySlotSize = 24.0f * scale;
        constexpr int   keyCols     = 8;
        int lastOccupied = -1;
        if (showKeyring_) {
            for (int i = inventory.getKeyringSize() - 1; i >= 0; --i) {
                if (!inventory.getKeyringSlot(i).empty()) { lastOccupied = i; break; }
            }
        }
        int visibleKeySlots = !showKeyring_ ? 0
                            : ((lastOccupied < 0 ? 0 : lastOccupied) / keyCols + 1) * keyCols;
        int keyringRows = (visibleKeySlots + keyCols - 1) / keyCols;
        contentH += 36.0f * scale;
        if (visibleKeySlots > 0) {
            contentH += 30.0f * scale + keyringRows * (keySlotSize + 4.0f * scale);
        }
    }
    float gridW = columns * (slotSize + 4.0f * scale) + 30.0f * scale;
    // Ensure window is wide enough for the title + close button
    const char* displayTitle = title;
    const char* hashPos = strstr(title, "##");
    float titleW = ImGui::CalcTextSize(displayTitle, hashPos).x + 50.0f; // close button + padding
    float windowW = std::max(gridW, titleW);
    float windowH = contentH + 40.0f * scale;

    // Always, and unmovable with it. These are tiled: the caller works out
    // where each one goes so they sit in a column above the bag bar without
    // touching. FirstUseEver meant that was only ever a starting suggestion -
    // one drag and the window kept its own position for the rest of the
    // install, sitting over whichever bag was placed there afterwards. The real
    // client does not let a bag be dragged out of its stack either.
    ImGui::SetNextWindowPos(ImVec2(defaultX, defaultY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove;

    bool windowVisible = ImGui::Begin(title, &isOpen, flags);
    if (!windowVisible) {
        ImGui::End();
        return;
    }

    // Recover stale or partially off-screen saved positions. The entire title
    // bar remains selectable instead of accepting a one-pixel visible sliver.
    ImVec2 winPos = ImGui::GetWindowPos();
    ImVec2 winSize = ImGui::GetWindowSize();
    if (clampCurrentWindowToMainViewport()) {
        winPos = ImGui::GetWindowPos();
        winSize = ImGui::GetWindowSize();
    }
    float scrH = ImGui::GetIO().DisplaySize.y;

    if (bagIndex < 0) {
        constexpr float bagBarSlotSize = 42.0f;
        constexpr float bagBarPadding = 6.0f;
        constexpr float bagBarBottomMargin = 10.0f;
        constexpr float stackGap = 8.0f;
        const float bagBarTop = scrH - (bagBarSlotSize + bagBarPadding * 2.0f) - bagBarBottomMargin;
        const float maxY = bagBarTop - stackGap - winSize.y;
        if (winPos.y > maxY) {
            ImGui::SetWindowPos(ImVec2(winPos.x, std::max(0.0f, maxY)));
        }
    }

    // Render item slots in 4-column grid
    for (int i = 0; i < numSlots; i++) {
        if (i % columns != 0) ImGui::SameLine();

        const game::ItemSlot& slot = (bagIndex < 0)
            ? inventory.getBackpackSlot(i)
            : inventory.getBagSlot(bagIndex, i);

        char id[32];
        if (bagIndex < 0) {
            snprintf(id, sizeof(id), "##sbp_%d", i);
        } else {
            snprintf(id, sizeof(id), "##sb%d_%d", bagIndex, i);
        }
        ImGui::PushID(id);

        if (bagIndex < 0) {
            // Backpack slot
            renderItemSlot(inventory, slot, slotSize, nullptr,
                           SlotKind::BACKPACK, i, game::EquipSlot::NUM_SLOTS);
        } else {
            // Bag slot - pass bag index info for interactions. Special containers
            // (quiver/ammo pouch/profession bag) get a label + amber wash so their
            // restricted slots aren't mistaken for general-purpose space.
            const char* restrictedLabel = nullptr;
            if (inventory.isBagSpecial(bagIndex)) {
                game::EquipSlot bagEquip = static_cast<game::EquipSlot>(
                    static_cast<int>(game::EquipSlot::BAG1) + bagIndex);
                const auto& bagItem = inventory.getEquipSlot(bagEquip);
                restrictedLabel = (!bagItem.empty() && !bagItem.item.subclassName.empty())
                    ? bagItem.item.subclassName.c_str()
                    : "Special Bag";
            }
            renderItemSlot(inventory, slot, slotSize, restrictedLabel,
                           SlotKind::BACKPACK, -1, game::EquipSlot::NUM_SLOTS,
                           bagIndex, i);
            if (restrictedLabel && slot.empty()) {
                ImVec2 mn = ImGui::GetItemRectMin();
                ImVec2 mx = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                dl->AddRectFilled(mn, mx, IM_COL32(180, 140, 40, 40));
                dl->AddRect(mn, mx, IM_COL32(180, 140, 40, 130));
            }
        }
        ImGui::PopID();
    }

    if (bagIndex < 0 && showKeyring_) {
        const float keySlotSize = 24.0f * scale;
        constexpr int keyCols = 8;
        // Only show rows that contain items (round up to full row)
        int lastOccupied = -1;
        for (int i = inventory.getKeyringSize() - 1; i >= 0; --i) {
            if (!inventory.getKeyringSlot(i).empty()) { lastOccupied = i; break; }
        }
        int visibleSlots = ((lastOccupied < 0 ? 0 : lastOccupied) / keyCols + 1) * keyCols;
        if (visibleSlots > 0) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ui::colors::kDarkYellow, "Keyring");
            for (int i = 0; i < visibleSlots; ++i) {
                if (i % keyCols != 0) ImGui::SameLine();
                const auto& slot = inventory.getKeyringSlot(i);
                char id[32];
                snprintf(id, sizeof(id), "##skr_%d", i);
                ImGui::PushID(id);
                renderItemSlot(inventory, slot, keySlotSize, nullptr,
                               SlotKind::KEYRING, -1, game::EquipSlot::NUM_SLOTS, -1, -1, i);
                ImGui::PopID();
            }
        }
    }

    // Footer for backpack: sort button + money display
    if (bagIndex < 0) {
        renderBagsFooter(moneyCopper);
    }

    ImGui::End();
}

void InventoryScreen::renderBagsFooter(uint64_t moneyCopper) {
    ImGui::Spacing();
    ImGui::Separator();

    // Sort Bags button. Through the game handler, which owns the one sort
    // there is: this panel used to keep a second queue of its own and drain it
    // a swap per frame. Two queues over the same bags is not a tidiness
    // question - SortBags() from a macro and this button each computed their
    // swaps from the layout as it was when they ran, so a sort started from
    // one while the other was still draining interleaved two plans and left
    // the bags arranged as neither intended, with nothing to say so. The
    // handler's own guard refuses a second sort while one is in flight, and it
    // sends a swap per network tick rather than per frame, which is the rate
    // the server will take a burst of them at.
    bool sorting = gameHandler_ && gameHandler_->isSortingItems();
    if (sorting) ImGui::BeginDisabled();
    if (ImGui::SmallButton(sorting ? "Sorting..." : "Sort Bags") && gameHandler_) {
        gameHandler_->sortBags();
    }
    if (sorting) ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Merge partial stacks, then sort every bag slot by quality\n(highest first), then by item ID, then by stack size.");
    }

    ImGui::SameLine();
    uint64_t gold   = moneyCopper / 10000;
    uint64_t silver = (moneyCopper / 100) % 100;
    uint64_t copper = moneyCopper % 100;
    ImGui::TextColored(ui::colors::kWarmGold, "%llug %llus %lluc",
                       static_cast<unsigned long long>(gold),
                       static_cast<unsigned long long>(silver),
                       static_cast<unsigned long long>(copper));
}

// ============================================================
// Character screen (C key) - equipment + model preview + stats
// ============================================================

void InventoryScreen::renderCharacterScreen(game::GameHandler& gameHandler) {
    if (!characterOpen) return;

    auto& inventory = gameHandler.getInventory();

    // Lazy-init the preview
    if (!previewInitialized_ && assetManager_) {
        initPreview();
    }

    // Update preview equipment if dirty
    if (previewDirty_ && charPreview_ && previewInitialized_) {
        updatePreviewEquipment(inventory, gameHandler.isHelmVisible(),
                                gameHandler.isCloakVisible());
    }

    // Update and render the preview FBO
    if (charPreview_ && previewInitialized_) {
        charPreview_->update(ImGui::GetIO().DeltaTime);
        charPreview_->render();
        charPreview_->requestComposite();
    }

    ImGui::SetNextWindowPos(ImVec2(20.0f, 80.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0f, 650.0f), ImGuiCond_FirstUseEver);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    if (!ImGui::Begin("Character", &characterOpen, flags)) {
        ImGui::End();
        return;
    }

    // Scale character-window contents from the usable resized area. Width and
    // height both constrain the result so controls grow into a large window
    // without overflowing a short one.
    const ImVec2 characterAvail = ImGui::GetContentRegionAvail();
    const float widthScale = characterAvail.x / 350.0f;
    const float heightScale = characterAvail.y / 450.0f;
    characterUiScale_ = std::clamp(std::min(widthScale, heightScale), 0.75f, 2.0f);
    ImGui::SetWindowFontScale(characterUiScale_);

    // Clamp window position within screen after resize
    {
        ImGuiIO& io = ImGui::GetIO();
        ImVec2 pos = ImGui::GetWindowPos();
        ImVec2 sz = ImGui::GetWindowSize();
        bool clamped = false;
        if (pos.x + sz.x > io.DisplaySize.x) { pos.x = std::max(0.0f, io.DisplaySize.x - sz.x); clamped = true; }
        if (pos.y + sz.y > io.DisplaySize.y) { pos.y = std::max(0.0f, io.DisplaySize.y - sz.y); clamped = true; }
        if (pos.x < 0.0f) { pos.x = 0.0f; clamped = true; }
        if (pos.y < 0.0f) { pos.y = 0.0f; clamped = true; }
        if (clamped) ImGui::SetWindowPos(pos);
    }

    if (ImGui::BeginTabBar("##CharacterTabs")) {
        if (ImGui::BeginTabItem("Equipment")) {
            renderEquipmentPanel(inventory);
            ImGui::Spacing();
            ImGui::Separator();
            // Appearance visibility toggles
            bool helmVis = gameHandler.isHelmVisible();
            bool cloakVis = gameHandler.isCloakVisible();
            if (ImGui::Checkbox("Show Helm", &helmVis)) {
                gameHandler.toggleHelm();
                previewDirty_ = true;
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Show Cloak", &cloakVis)) {
                gameHandler.toggleCloak();
                previewDirty_ = true;
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Stats")) {
            ImGui::Spacing();
            int32_t stats[5];
            for (int i = 0; i < 5; ++i) stats[i] = gameHandler.getPlayerStat(i);
            const int32_t* serverStats = (stats[0] >= 0) ? stats : nullptr;
            int32_t resists[6];
            for (int i = 0; i < 6; ++i) resists[i] = gameHandler.getResistance(i + 1);
            renderStatsPanel(inventory, gameHandler.getPlayerLevel(), gameHandler.getArmorRating(), serverStats, resists, &gameHandler);

            // Played time (shown if available, fetched on character screen open)
            uint32_t totalSec = gameHandler.getTotalTimePlayed();
            uint32_t levelSec = gameHandler.getLevelTimePlayed();
            if (totalSec > 0 || levelSec > 0) {
                ImGui::Separator();
                // Helper lambda to format seconds as "Xd Xh Xm"
                auto fmtTime = [](uint32_t sec) -> std::string {
                    uint32_t d = sec / 86400, h = (sec % 86400) / 3600, m = (sec % 3600) / 60;
                    char buf[48];
                    if (d > 0) snprintf(buf, sizeof(buf), "%ud %uh %um", d, h, m);
                    else if (h > 0) snprintf(buf, sizeof(buf), "%uh %um", h, m);
                    else snprintf(buf, sizeof(buf), "%um", m);
                    return buf;
                };
                ImGui::TextDisabled("Time Played");
                ImGui::Columns(2, "##playtime", false);
                ImGui::SetColumnWidth(0, 130.0f * characterUiScale_);
                ImGui::Text("Total:");    ImGui::NextColumn();
                ImGui::Text("%s", fmtTime(totalSec).c_str()); ImGui::NextColumn();
                ImGui::Text("This level:"); ImGui::NextColumn();
                ImGui::Text("%s", fmtTime(levelSec).c_str()); ImGui::NextColumn();
                ImGui::Columns(1);
            }

            // PvP Currency (TBC/WotLK only)
            uint32_t honor = gameHandler.getHonorPoints();
            uint32_t arena = gameHandler.getArenaPoints();
            if (honor > 0 || arena > 0) {
                ImGui::Separator();
                ImGui::TextDisabled("PvP Currency");
                ImGui::Columns(2, "##pvpcurrency", false);
                ImGui::SetColumnWidth(0, 130.0f * characterUiScale_);
                ImGui::Text("Honor Points:"); ImGui::NextColumn();
                ImGui::TextColored(ImVec4(0.9f, 0.75f, 0.2f, 1.0f), "%u", honor); ImGui::NextColumn();
                ImGui::Text("Arena Points:"); ImGui::NextColumn();
                ImGui::TextColored(ImVec4(0.9f, 0.75f, 0.2f, 1.0f), "%u", arena); ImGui::NextColumn();
                ImGui::Columns(1);
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Reputation")) {
            renderReputationPanel(gameHandler);
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Skills")) {
            const auto& skills = gameHandler.getPlayerSkills();
            if (skills.empty()) {
                ImGui::TextDisabled("No skill data received yet.");
            } else {
                // Group skills by SkillLine.dbc category
                struct CategoryGroup {
                    const char* label;
                    uint32_t categoryId;
                };
                static constexpr CategoryGroup groups[] = {
                    { .label = "Weapon Skills", .categoryId = 6 },
                    { .label = "Armor Skills", .categoryId = 8 },
                    { .label = "Secondary Skills", .categoryId = 10 },
                    { .label = "Professions", .categoryId = 11 },
                    { .label = "Languages", .categoryId = 9 },
                    { .label = "Other", .categoryId = 0 },
                };

                ImGui::BeginChild("##SkillsList", ImVec2(0, 0), true);

                for (const auto& group : groups) {
                    // Collect skills for this category
                    std::vector<const game::PlayerSkill*> groupSkills;
                    for (const auto& [id, skill] : skills) {
                        if (skill.value == 0 && skill.maxValue == 0) continue;
                        uint32_t cat = gameHandler.getSkillCategory(id);
                        if (group.categoryId == 0) {
                            // "Other" catches everything not in the named categories
                            if (cat != 6 && cat != 8 && cat != 9 && cat != 10 && cat != 11) {
                                groupSkills.push_back(&skill);
                            }
                        } else if (cat == group.categoryId) {
                            groupSkills.push_back(&skill);
                        }
                    }
                    if (groupSkills.empty()) continue;

                    if (ImGui::CollapsingHeader(group.label, ImGuiTreeNodeFlags_DefaultOpen)) {
                        for (const game::PlayerSkill* skill : groupSkills) {
                            const std::string& name = gameHandler.getSkillName(skill->skillId);
                            char label[128];
                            if (name.empty()) {
                                snprintf(label, sizeof(label), "Skill #%u", skill->skillId);
                            } else {
                                snprintf(label, sizeof(label), "%s", name.c_str());
                            }

                            // Effective value includes temporary and permanent bonuses
                            uint16_t effective = skill->effectiveValue();
                            uint16_t bonus = skill->bonusTemp + skill->bonusPerm;

                            // Progress bar reflects effective / max; cap visual fill at 1.0
                            float ratio = (skill->maxValue > 0)
                                ? std::min(1.0f, static_cast<float>(effective) / static_cast<float>(skill->maxValue))
                                : 0.0f;

                            char overlay[64];
                            if (bonus > 0)
                                snprintf(overlay, sizeof(overlay), "%u / %u (+%u)", effective, skill->maxValue, bonus);
                            else
                                snprintf(overlay, sizeof(overlay), "%u / %u", effective, skill->maxValue);

                            // Gold name when maxed out, cyan when buffed above base, default otherwise
                            bool isMaxed  = (effective >= skill->maxValue && skill->maxValue > 0);
                            bool isBuffed = (bonus > 0);
                            ImVec4 nameColor = isMaxed  ? ui::colors::kTooltipGold
                                             : isBuffed ? ImVec4(0.4f, 0.9f,  1.0f, 1.0f)
                                             :            ui::colors::kVeryLightGray;
                            ImGui::TextColored(nameColor, "%s", label);
                            ImGui::SameLine(180.0f * characterUiScale_);
                            ImGui::SetNextItemWidth(-1.0f);
                            // Bar color: gold when maxed, green otherwise
                            ImVec4 barColor = isMaxed ? ui::colors::kTooltipGold : ui::colors::kFriendlyGreen;
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
                            ImGui::ProgressBar(ratio, ImVec2(0, 14.0f * characterUiScale_), overlay);
                            ImGui::PopStyleColor();
                        }
                    }
                }

                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Achievements")) {
            const auto& earned = gameHandler.getEarnedAchievements();
            if (earned.empty()) {
                ImGui::Spacing();
                ImGui::TextDisabled("No achievements earned yet.");
            } else {
                static char achieveFilter[128] = {};
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputTextWithHint("##achsearch", "Search achievements...",
                                         achieveFilter, sizeof(achieveFilter));
                ImGui::Separator();

                char filterLower[128];
                for (size_t i = 0; i < sizeof(achieveFilter); ++i)
                    filterLower[i] = static_cast<char>(tolower(static_cast<unsigned char>(achieveFilter[i])));

                ImGui::BeginChild("##AchList", ImVec2(0, 0), false);
                // Sort by ID for stable ordering
                std::vector<uint32_t> sortedIds(earned.begin(), earned.end());
                std::sort(sortedIds.begin(), sortedIds.end());
                int shown = 0;
                for (uint32_t id : sortedIds) {
                    const std::string& name = gameHandler.getAchievementName(id);
                    const char* displayName = name.empty() ? nullptr : name.c_str();
                    if (displayName == nullptr) continue;  // skip unknown achievements

                    // Apply filter
                    if (filterLower[0] != '\0') {
                        // simple case-insensitive substring match
                        std::string lower;
                        lower.reserve(name.size());
                        for (char c : name) lower += static_cast<char>(tolower(static_cast<unsigned char>(c)));
                        if (lower.find(filterLower) == std::string::npos) continue;
                    }

                    ImGui::PushID(static_cast<int>(id));
                    ImGui::TextColored(ui::colors::kWarmGold, "[Achievement]");
                    ImGui::SameLine();
                    ImGui::Text("%s", displayName);
                    ImGui::PopID();
                    ++shown;
                }
                if (shown == 0 && filterLower[0] != '\0') {
                    ImGui::TextDisabled("No achievements match the filter.");
                }
                ImGui::Text("Total: %d", static_cast<int>(earned.size()));
                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("PvP")) {
            const auto& arenaStats = gameHandler.getArenaTeamStats();
            if (arenaStats.empty()) {
                ImGui::Spacing();
                ImGui::TextDisabled("Not a member of any Arena team.");
            } else {
                for (const auto& team : arenaStats) {
                    ImGui::PushID(static_cast<int>(team.teamId));
                    char header[64];
                    snprintf(header, sizeof(header), "Team ID %u  (Rating: %u)", team.teamId, team.rating);
                    if (ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen)) {
                        ImGui::Columns(2, "##arenacols", false);
                        ImGui::Text("Rating:");     ImGui::NextColumn();
                        ImGui::Text("%u", team.rating);    ImGui::NextColumn();
                        ImGui::Text("Rank:");       ImGui::NextColumn();
                        ImGui::Text("#%u", team.rank);     ImGui::NextColumn();
                        ImGui::Text("This week:");  ImGui::NextColumn();
                        ImGui::Text("%u / %u (W/G)", team.weekWins, team.weekGames); ImGui::NextColumn();
                        ImGui::Text("Season:");     ImGui::NextColumn();
                        ImGui::Text("%u / %u (W/G)", team.seasonWins, team.seasonGames); ImGui::NextColumn();
                        ImGui::Columns(1);
                    }
                    ImGui::PopID();
                }
            }
            ImGui::EndTabItem();
        }

        // Equipment Sets tab (WotLK only - requires server support)
        if (gameHandler.supportsEquipmentSets() && ImGui::BeginTabItem("Outfits")) {
            ImGui::Spacing();

            // Save current gear as new set
            static char newSetName[64] = {};
            ImGui::SetNextItemWidth(160.0f * characterUiScale_);
            ImGui::InputTextWithHint("##newsetname", "New set name...", newSetName, sizeof(newSetName));
            ImGui::SameLine();
            bool canSave = (newSetName[0] != '\0');
            if (!canSave) ImGui::BeginDisabled();
            if (ImGui::SmallButton("Save Current Gear")) {
                gameHandler.saveEquipmentSet(newSetName);
                newSetName[0] = '\0';
            }
            if (!canSave) ImGui::EndDisabled();

            ImGui::Separator();

            const auto& eqSets = gameHandler.getEquipmentSets();
            if (eqSets.empty()) {
                ImGui::TextDisabled("No saved equipment sets.");
            } else {
                ImGui::BeginChild("##EqSetsList", ImVec2(0, 0), false);
                for (const auto& es : eqSets) {
                    ImGui::PushID(static_cast<int>(es.setId));
                    const char* displayName = es.name.empty() ? "(Unnamed)" : es.name.c_str();
                    ImGui::Text("%s", displayName);
                    float btnAreaW = 150.0f * characterUiScale_;
                    ImGui::SameLine(ImGui::GetContentRegionAvail().x - btnAreaW + ImGui::GetCursorPosX());
                    if (ImGui::SmallButton("Equip")) {
                        gameHandler.useEquipmentSet(es.setId);
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Update")) {
                        gameHandler.saveEquipmentSet(es.name, es.iconName, es.setGuid, es.setId);
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Delete")) {
                        gameHandler.deleteEquipmentSet(es.setGuid);
                        ImGui::PopID();
                        break;  // Iterator invalidated
                    }
                    ImGui::PopID();
                }
                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    renderEquipConfirmationPopup(inventory);

    ImGui::End();

    // If both bags and character are open, allow drag-and-drop between them
    // (held item rendering is handled in render())
    if (open) {
        renderHeldItem();
    }
}

void InventoryScreen::renderReputationPanel(game::GameHandler& gameHandler) {
    const auto& standings = gameHandler.getFactionStandings();
    if (standings.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("No reputation data received yet.");
        ImGui::TextDisabled("Reputation updates as you kill enemies and complete quests.");
        return;
    }

    // Colours only. Where each standing begins and ends is in
    // game/reputation_standing.hpp, shared with the original interface's
    // GetFactionInfo - two tables of the same thresholds would eventually
    // disagree, and the same faction would sit at different standings
    // depending on which window was open.
    static const ImVec4 tierColors[8] = {
        ImVec4(0.6f, 0.1f, 0.1f, 1.0f),   // Hated
        ImVec4(0.8f, 0.2f, 0.1f, 1.0f),   // Hostile
        ImVec4(0.9f, 0.5f, 0.1f, 1.0f),   // Unfriendly
        ImVec4(0.8f, 0.8f, 0.2f, 1.0f),   // Neutral
        ui::colors::kFriendlyGreen,        // Friendly
        ImVec4(0.2f, 0.8f, 0.5f, 1.0f),   // Honored
        ImVec4(0.3f, 0.6f, 1.0f, 1.0f),   // Revered
        ui::colors::kWarmGold,             // Exalted
    };
    struct RepTier {
        const char* name;
        int32_t     floor;
        int32_t     ceiling;
        ImVec4      color;
    };
    auto getTier = [&](int32_t val) -> RepTier {
        const auto& band = game::reputationStandingFor(val);
        return {.name = band.name, .floor = band.floor, .ceiling = band.ceiling, .color = tierColors[band.id - 1]};
    };

    // --- Reputation controls ---
    // "Show inactive" reveals factions the player has parked on the inactive list.
    static bool showInactiveFactions = false;
    ImGui::Checkbox("Show inactive", &showInactiveFactions);
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Right-click a faction to track it on the reputation bar,\n"
                          "declare war / make peace, or set it inactive.");
    }
    // Count how many factions are currently parked as inactive.
    uint32_t inactiveCount = 0;
    for (const auto& [fid, st] : standings) {
        uint32_t rl = gameHandler.getRepListIdByFactionId(fid);
        if (rl != 0xFFFFFFFFu && gameHandler.isFactionInactive(rl)) ++inactiveCount;
    }
    if (inactiveCount > 0 && !showInactiveFactions) {
        ImGui::SameLine();
        ImGui::TextDisabled("%u inactive hidden", inactiveCount);
    }
    ImGui::Separator();

    ImGui::BeginChild("##ReputationList", ImVec2(0, 0), true);

    // Sort: watched faction first, then alphabetically by name
    uint32_t watchedFactionId = gameHandler.getWatchedFactionId();
    std::vector<std::pair<uint32_t, int32_t>> sortedFactions(standings.begin(), standings.end());
    std::sort(sortedFactions.begin(), sortedFactions.end(),
        [&](const auto& a, const auto& b) {
            if (a.first == watchedFactionId) return true;
            if (b.first == watchedFactionId) return false;
            const std::string& na = gameHandler.getFactionNamePublic(a.first);
            const std::string& nb = gameHandler.getFactionNamePublic(b.first);
            return na < nb;
        });

    for (const auto& [factionId, standing] : sortedFactions) {
        const RepTier tier = getTier(standing);

        const std::string& factionName = gameHandler.getFactionNamePublic(factionId);
        const char* displayName = factionName.empty() ? "Unknown Faction" : factionName.c_str();

        // Determine at-war / inactive status via repListId lookup
        uint32_t repListId = gameHandler.getRepListIdByFactionId(factionId);
        bool hasRepList = (repListId != 0xFFFFFFFFu);
        bool atWar = hasRepList && gameHandler.isFactionAtWar(repListId);
        bool inactive = hasRepList && gameHandler.isFactionInactive(repListId);
        bool peaceForced = hasRepList && gameHandler.isFactionPeaceForced(repListId);
        bool isWatched = (factionId == watchedFactionId);

        // Hide inactive factions unless the player opted to show them.
        if (inactive && !showInactiveFactions) continue;

        ImGui::PushID(static_cast<int>(factionId));
        // Dim inactive rows so they read as parked when shown.
        if (inactive) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

        // Faction name + tier label on same line; mark at-war and watched factions
        ImGui::TextColored(tier.color, "[%s]", tier.name);
        ImGui::SameLine(90.0f * characterUiScale_);
        if (atWar) {
            ImGui::TextColored(ui::colors::kRed, "%s", displayName);
            ImGui::SameLine();
            ImGui::TextColored(ui::colors::kRed, "(At War)");
        } else if (isWatched) {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 1.0f), "%s", displayName);
            ImGui::SameLine();
            ImGui::TextDisabled("(Tracked)");
        } else {
            ImGui::Text("%s", displayName);
        }

        // Progress bar showing position within current tier
        float ratio = 0.0f;
        char overlay[64] = "";
        if (tier.floor == 42000) {
            // Exalted - full bar
            ratio = 1.0f;
            snprintf(overlay, sizeof(overlay), "Exalted");
        } else {
            int32_t tierRange = tier.ceiling - tier.floor + 1;
            int32_t inTier    = standing - tier.floor;
            ratio = static_cast<float>(inTier) / static_cast<float>(tierRange);
            ratio = std::max(0.0f, std::min(1.0f, ratio));
            snprintf(overlay, sizeof(overlay), "%d / %d",
                     inTier < 0 ? 0 : inTier, tierRange);
        }

        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, tier.color);
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::ProgressBar(ratio, ImVec2(0, 12.0f * characterUiScale_), overlay);
        ImGui::PopStyleColor();

        // Right-click context menu on the progress bar
        if (ImGui::BeginPopupContextItem("##RepCtx")) {
            ImGui::TextDisabled("%s", displayName);
            ImGui::Separator();
            if (isWatched) {
                if (ImGui::MenuItem("Untrack"))
                    gameHandler.setWatchedFactionId(0);
            } else {
                if (ImGui::MenuItem("Track on Rep Bar"))
                    gameHandler.setWatchedFactionId(factionId);
            }
            if (hasRepList) {
                ImGui::Separator();
                // War / peace toggle - disabled when the server forces peace.
                if (peaceForced) {
                    ImGui::BeginDisabled();
                    ImGui::MenuItem("At War", nullptr, false);
                    ImGui::EndDisabled();
                } else if (ImGui::MenuItem("At War", nullptr, atWar)) {
                    gameHandler.setFactionAtWar(repListId, !atWar);
                }
                // Inactive toggle - parks the faction out of the active list.
                if (ImGui::MenuItem("Inactive", nullptr, inactive)) {
                    gameHandler.setFactionInactive(repListId, !inactive);
                }
            }
            ImGui::EndPopup();
        }

        if (inactive) ImGui::PopStyleVar();
        ImGui::Spacing();
        ImGui::PopID();
    }

    ImGui::EndChild();
}

void InventoryScreen::renderEquipmentPanel(game::Inventory& inventory) {
    ImGui::TextColored(ui::colors::kWarmGold, "Equipment");
    ImGui::Separator();

    static constexpr game::EquipSlot leftSlots[] = {
        game::EquipSlot::HEAD, game::EquipSlot::NECK,
        game::EquipSlot::SHOULDERS, game::EquipSlot::BACK,
        game::EquipSlot::CHEST, game::EquipSlot::SHIRT,
        game::EquipSlot::TABARD, game::EquipSlot::WRISTS,
    };
    static constexpr game::EquipSlot rightSlots[] = {
        game::EquipSlot::HANDS, game::EquipSlot::WAIST,
        game::EquipSlot::LEGS, game::EquipSlot::FEET,
        game::EquipSlot::RING1, game::EquipSlot::RING2,
        game::EquipSlot::TRINKET1, game::EquipSlot::TRINKET2,
    };

    const float slotSize = 36.0f * characterUiScale_;
    const float gap = 8.0f * characterUiScale_;
    const float availableW = ImGui::GetContentRegionAvail().x;
    const float minimumPreviewW = 140.0f * characterUiScale_;
    const float minimumLayoutW = slotSize * 2.0f + gap * 2.0f + minimumPreviewW;
    const float previewW = minimumPreviewW + std::max(0.0f, availableW - minimumLayoutW);

    // Calculate column positions for the 3-column layout
    float contentStartX = ImGui::GetCursorPosX();
    float rightColX = contentStartX + slotSize + gap + previewW + gap;

    int rows = 8;
    float previewStartY = ImGui::GetCursorScreenPos().y;

    for (int r = 0; r < rows; r++) {
        // Left column
        {
            const auto& slot = inventory.getEquipSlot(leftSlots[r]);
            const char* label = game::getEquipSlotName(leftSlots[r]);
            char id[64];
            snprintf(id, sizeof(id), "##eq_l_%d", r);
            ImGui::PushID(id);
            renderItemSlot(inventory, slot, slotSize, label,
                           SlotKind::EQUIPMENT, -1, leftSlots[r]);
            ImGui::PopID();
        }

        // Right column
        ImGui::SameLine(rightColX);
        {
            const auto& slot = inventory.getEquipSlot(rightSlots[r]);
            const char* label = game::getEquipSlotName(rightSlots[r]);
            char id[64];
            snprintf(id, sizeof(id), "##eq_r_%d", r);
            ImGui::PushID(id);
            renderItemSlot(inventory, slot, slotSize, label,
                           SlotKind::EQUIPMENT, -1, rightSlots[r]);
            ImGui::PopID();
        }
    }

    float previewEndY = ImGui::GetCursorScreenPos().y;

    // Draw the 3D character preview in the center column
    if (charPreview_ && previewInitialized_ && charPreview_->getTextureId()) {
        float previewX = ImGui::GetWindowPos().x + contentStartX + slotSize + gap;
        float previewH = previewEndY - previewStartY;
        // Maintain aspect ratio
        float texAspect = static_cast<float>(charPreview_->getWidth()) / static_cast<float>(charPreview_->getHeight());
        float displayW = previewW;
        float displayH = displayW / texAspect;
        if (displayH > previewH) {
            displayH = previewH;
            displayW = displayH * texAspect;
        }
        float offsetX = previewX + (previewW - displayW) * 0.5f;
        float offsetY = previewStartY + (previewH - displayH) * 0.5f;

        ImVec2 pMin(offsetX, offsetY);
        ImVec2 pMax(offsetX + displayW, offsetY + displayH);

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        // Background for preview area
        drawList->AddRectFilled(pMin, pMax, IM_COL32(13, 13, 25, 255));
        drawList->AddImage(
            reinterpret_cast<ImTextureID>(charPreview_->getTextureId()),
            pMin, pMax);
        drawList->AddRect(pMin, pMax, IM_COL32(60, 60, 80, 200));

        // Drag-to-rotate: detect mouse drag over the preview image
        ImGui::SetCursorScreenPos(pMin);
        ImGui::InvisibleButton("##charPreviewDrag", ImVec2(displayW, displayH));
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            float dx = ImGui::GetIO().MouseDelta.x;
            charPreview_->rotate(dx * 1.0f);
        }
    }

    // Weapon row - positioned to the right of left column to avoid crowding main equipment
    ImGui::Spacing();
    ImGui::Separator();

    static constexpr game::EquipSlot weaponSlots[] = {
        game::EquipSlot::MAIN_HAND,
        game::EquipSlot::OFF_HAND,
        game::EquipSlot::RANGED,
    };

    // Position weapons in center column area (after left column, 3D preview renders on top)
    ImGui::SetCursorPosX(contentStartX + slotSize + gap);
    for (int i = 0; i < 3; i++) {
        if (i > 0) ImGui::SameLine();
        const auto& slot = inventory.getEquipSlot(weaponSlots[i]);
        const char* label = game::getEquipSlotName(weaponSlots[i]);
        char id[64];
        snprintf(id, sizeof(id), "##eq_w_%d", i);
        ImGui::PushID(id);
        renderItemSlot(inventory, slot, slotSize, label,
                       SlotKind::EQUIPMENT, -1, weaponSlots[i]);
        ImGui::PopID();
    }
}

// ============================================================
// Stats Panel
// ============================================================

void InventoryScreen::renderStatsPanel(game::Inventory& inventory, uint32_t playerLevel,
                                        int32_t serverArmor, const int32_t* serverStats,
                                        const int32_t* serverResists,
                                        const game::GameHandler* gh) {
    // Sum equipment stats for item-query bonus display
    int32_t itemStr = 0, itemAgi = 0, itemSta = 0, itemInt = 0, itemSpi = 0;
    // Secondary stat sums from extraStats
    int32_t itemAP = 0, itemSP = 0, itemHit = 0, itemCrit = 0, itemHaste = 0;
    int32_t itemResil = 0, itemExpertise = 0, itemMp5 = 0, itemHp5 = 0;
    int32_t itemDefense = 0, itemDodge = 0, itemParry = 0, itemBlock = 0, itemBlockVal = 0;
    int32_t itemArmorPen = 0, itemSpellPen = 0;
    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (slot.empty()) continue;
        itemStr += slot.item.strength;
        itemAgi += slot.item.agility;
        itemSta += slot.item.stamina;
        itemInt += slot.item.intellect;
        itemSpi += slot.item.spirit;
        for (const auto& es : slot.item.extraStats) {
            switch (es.statType) {
                case 12:                             itemDefense  += es.statValue; break;
                case 13:                             itemDodge    += es.statValue; break;
                case 14:                             itemParry    += es.statValue; break;
                case 15:                             itemBlock    += es.statValue; break;
                case 16: case 17: case 18: case 31: itemHit      += es.statValue; break;
                case 19: case 20: case 21: case 32: itemCrit     += es.statValue; break;
                case 28: case 29: case 30: case 36: itemHaste    += es.statValue; break;
                case 35:                             itemResil    += es.statValue; break;
                case 37:                             itemExpertise += es.statValue; break;
                case 38: case 39:                    itemAP       += es.statValue; break;
                case 41: case 42: case 45:           itemSP       += es.statValue; break;
                case 43:                             itemMp5      += es.statValue; break;
                case 44:                             itemArmorPen += es.statValue; break;
                case 46:                             itemHp5      += es.statValue; break;
                case 47:                             itemSpellPen += es.statValue; break;
                case 48:                             itemBlockVal += es.statValue; break;
                default: break;
            }
        }
    }

    // Use server-authoritative armor from UNIT_FIELD_RESISTANCES when available.
    // Falls back to summing item query armors if server armor wasn't received yet.
    int32_t itemQueryArmor = 0;
    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (!slot.empty()) itemQueryArmor += slot.item.armor;
    }
    int32_t totalArmor = (serverArmor > 0) ? serverArmor : itemQueryArmor;

    // Average item level (exclude shirt/tabard as WoW convention)
    {
        uint32_t iLvlSum = 0;
        int iLvlCount = 0;
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            auto eslot = static_cast<game::EquipSlot>(s);
            if (eslot == game::EquipSlot::SHIRT || eslot == game::EquipSlot::TABARD) continue;
            const auto& slot = inventory.getEquipSlot(eslot);
            if (!slot.empty() && slot.item.itemLevel > 0) {
                iLvlSum += slot.item.itemLevel;
                ++iLvlCount;
            }
        }
        if (iLvlCount > 0) {
            float avg = static_cast<float>(iLvlSum) / static_cast<float>(iLvlCount);
            ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f),
                "Average Item Level: %.1f  (%d/%d slots)", avg, iLvlCount,
                game::Inventory::NUM_EQUIP_SLOTS - 2);  // -2 for shirt/tabard
        }
        ImGui::Separator();
    }

    ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    ImVec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    ImVec4 gold(1.0f, 0.84f, 0.0f, 1.0f);
    ImVec4 gray(0.6f, 0.6f, 0.6f, 1.0f);

    static constexpr const char* kStatTooltips[5] = {
        "Increases your melee attack power by 2.\nIncreases your block value.",
        "Increases your Armor.\nIncreases ranged attack power by 2.\nIncreases your chance to dodge attacks and score critical strikes.",
        "Increases Health by 10 per point.",
        "Increases your Mana pool.\nIncreases your chance to score a critical strike with spells.",
        "Increases Health and Mana regeneration."
    };

    // Armor (no base)
    ImGui::BeginGroup();
    if (totalArmor > 0) {
        ImGui::TextColored(gold, "Armor: %d", totalArmor);
    } else {
        ImGui::TextColored(gray, "Armor: 0");
    }
    ImGui::EndGroup();
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::TextWrapped("Reduces damage taken from physical attacks.");
        ImGui::EndTooltip();
    }

    if (serverStats) {
        // Server-authoritative stats from UNIT_FIELD_STAT0-4: show total and item bonus.
        // serverStats[i] is the server's effective base stat (items included, buffs excluded).
        const char* statNames[5] = {"Strength", "Agility", "Stamina", "Intellect", "Spirit"};
        const int32_t itemBonuses[5] = {itemStr, itemAgi, itemSta, itemInt, itemSpi};
        for (int i = 0; i < 5; ++i) {
            int32_t total = serverStats[i];
            int32_t bonus = itemBonuses[i];
            ImGui::BeginGroup();
            if (bonus > 0) {
                ImGui::TextColored(white, "%s: %d", statNames[i], total);
                ImGui::SameLine();
                ImGui::TextColored(green, "(+%d)", bonus);
            } else {
                ImGui::TextColored(gray, "%s: %d", statNames[i], total);
            }
            ImGui::EndGroup();
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextWrapped("%s", kStatTooltips[i]);
                ImGui::EndTooltip();
            }
        }
    } else {
        // Fallback: estimated base (20 + level) plus item query bonuses.
        int32_t baseStat = 20 + static_cast<int32_t>(playerLevel);
        auto renderStat = [&](const char* name, int32_t equipBonus, const char* tooltip) {
            int32_t total = baseStat + equipBonus;
            ImGui::BeginGroup();
            if (equipBonus > 0) {
                ImGui::TextColored(white, "%s: %d", name, total);
                ImGui::SameLine();
                ImGui::TextColored(green, "(+%d)", equipBonus);
            } else {
                ImGui::TextColored(gray, "%s: %d", name, total);
            }
            ImGui::EndGroup();
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextWrapped("%s", tooltip);
                ImGui::EndTooltip();
            }
        };
        renderStat("Strength",  itemStr, kStatTooltips[0]);
        renderStat("Agility",   itemAgi, kStatTooltips[1]);
        renderStat("Stamina",   itemSta, kStatTooltips[2]);
        renderStat("Intellect", itemInt, kStatTooltips[3]);
        renderStat("Spirit",    itemSpi, kStatTooltips[4]);
    }

    // Secondary stats from equipped items
    bool hasSecondary = itemAP || itemSP || itemHit || itemCrit || itemHaste ||
                        itemResil || itemExpertise || itemMp5 || itemHp5 ||
                        itemDefense || itemDodge || itemParry || itemBlock || itemBlockVal ||
                        itemArmorPen || itemSpellPen;
    if (hasSecondary) {
        ImGui::Spacing();
        ImGui::Separator();
        auto renderSecondary = [&](const char* name, int32_t val, const char* tooltip) {
            if (val > 0) {
                ImGui::BeginGroup();
                ImGui::TextColored(green, "+%d %s", val, name);
                ImGui::EndGroup();
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::TextWrapped("%s", tooltip);
                    ImGui::EndTooltip();
                }
            }
        };
        renderSecondary("Attack Power",     itemAP,       "Increases the damage of your melee and ranged attacks.");
        renderSecondary("Spell Power",      itemSP,       "Increases the damage and healing of your spells.");
        renderSecondary("Hit Rating",       itemHit,      "Reduces the chance your attacks will miss.");
        renderSecondary("Crit Rating",      itemCrit,     "Increases your critical strike chance.");
        renderSecondary("Haste Rating",     itemHaste,    "Increases attack speed and spell casting speed.");
        renderSecondary("Resilience",       itemResil,    "Reduces the chance you will be critically hit.\nReduces damage taken from critical hits.");
        renderSecondary("Expertise",        itemExpertise,"Reduces the chance your attacks will be dodged or parried.");
        renderSecondary("Defense Rating",   itemDefense,  "Reduces the chance enemies will critically hit you.");
        renderSecondary("Dodge Rating",     itemDodge,    "Increases your chance to dodge attacks.");
        renderSecondary("Parry Rating",     itemParry,    "Increases your chance to parry attacks.");
        renderSecondary("Block Rating",     itemBlock,    "Increases your chance to block attacks with your shield.");
        renderSecondary("Block Value",      itemBlockVal, "Increases the amount of damage your shield blocks.");
        renderSecondary("Armor Penetration",itemArmorPen, "Reduces the armor of your target.");
        renderSecondary("Spell Penetration",itemSpellPen, "Reduces your target's resistance to your spells.");
        renderSecondary("Mana per 5 sec",   itemMp5,      "Restores mana every 5 seconds, even while casting.");
        renderSecondary("Health per 5 sec", itemHp5,      "Restores health every 5 seconds.");
    }

    // Elemental resistances from server update fields
    if (serverResists) {
        bool hasResist = false;
        for (int i = 0; i < 6; ++i) {
            if (serverResists[i] > 0) { hasResist = true; break; }
        }
        if (hasResist) {
            ImGui::Spacing();
            ImGui::Separator();
            for (int i = 0; i < 6; ++i) {
                if (serverResists[i] > 0) {
                    ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f),
                        "%s: %d", game::resistanceSchoolName(static_cast<uint32_t>(i)), serverResists[i]);
                }
            }
        }
    }

    // Server-authoritative combat stats (WotLK update fields - only shown when received)
    if (gh) {
        int32_t meleeAP   = gh->getMeleeAttackPower();
        int32_t rangedAP  = gh->getRangedAttackPower();
        int32_t spellPow  = gh->getSpellPower();
        int32_t healPow   = gh->getHealingPower();
        float dodgePct    = gh->getDodgePct();
        float parryPct   = gh->getParryPct();
        float blockPct   = gh->getBlockPct();
        float critPct    = gh->getCritPct();
        float rCritPct   = gh->getRangedCritPct();
        float sCritPct   = gh->getSpellCritPct(1);  // Holy school (avg proxy for spell crit)
        // Hit rating: CR_HIT_MELEE=5, CR_HIT_RANGED=6, CR_HIT_SPELL=7
        // Haste rating: CR_HASTE_MELEE=17, CR_HASTE_RANGED=18, CR_HASTE_SPELL=19
        // Other: CR_EXPERTISE=23, CR_ARMOR_PENETRATION=24, CR_CRIT_TAKEN_MELEE=14
        int32_t hitRating     = gh->getCombatRating(5);
        int32_t hitRangedR    = gh->getCombatRating(6);
        int32_t hitSpellR     = gh->getCombatRating(7);
        int32_t expertiseR    = gh->getCombatRating(23);
        int32_t hasteR        = gh->getCombatRating(17);
        int32_t hasteRangedR  = gh->getCombatRating(18);
        int32_t hasteSpellR   = gh->getCombatRating(19);
        int32_t armorPenR     = gh->getCombatRating(24);
        int32_t resilR        = gh->getCombatRating(14);  // CR_CRIT_TAKEN_MELEE = Resilience

        bool hasAny = (meleeAP >= 0 || spellPow >= 0 || dodgePct >= 0.0f || parryPct >= 0.0f ||
                       blockPct >= 0.0f || critPct >= 0.0f || hitRating >= 0);
        if (hasAny) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ui::colors::kWarmGold, "Combat");
            ImVec4 cyan(0.5f, 0.9f, 1.0f, 1.0f);
            if (meleeAP  >= 0) ImGui::TextColored(cyan, "Attack Power: %d", meleeAP);
            if (rangedAP >= 0 && rangedAP != meleeAP)
                ImGui::TextColored(cyan, "Ranged Attack Power: %d", rangedAP);
            if (spellPow  >= 0) ImGui::TextColored(cyan, "Spell Power: %d", spellPow);
            if (healPow   >= 0 && healPow != spellPow)
                ImGui::TextColored(cyan, "Healing Power: %d", healPow);
            if (dodgePct  >= 0.0f) ImGui::TextColored(cyan, "Dodge: %.2f%%", dodgePct);
            if (parryPct  >= 0.0f) ImGui::TextColored(cyan, "Parry: %.2f%%", parryPct);
            if (blockPct  >= 0.0f) ImGui::TextColored(cyan, "Block: %.2f%%", blockPct);
            if (critPct   >= 0.0f) ImGui::TextColored(cyan, "Melee Crit: %.2f%%", critPct);
            if (rCritPct  >= 0.0f) ImGui::TextColored(cyan, "Ranged Crit: %.2f%%", rCritPct);
            if (sCritPct  >= 0.0f) ImGui::TextColored(cyan, "Spell Crit: %.2f%%", sCritPct);

            // Combat ratings with percentage conversion (WotLK level-80 divisors scaled by level).
            // Formula: pct = rating / (divisorAt80 * pow(level/80.0, 0.93))
            // Level-80 divisors derived from gtCombatRatings.dbc (well-known WotLK constants):
            //   Hit: 26.23,  Expertise: 8.19/expertise (0.25% each),
            //   Haste: 32.79,  ArmorPen: 13.99,  Resilience: 94.27
            uint32_t level = playerLevel > 0 ? playerLevel : gh->getPlayerLevel();
            if (level == 0) level = 80;
            double lvlScale = level <= 80
                ? std::pow(static_cast<double>(level) / 80.0, 0.93)
                : 1.0;

            auto ratingPct = [&](int32_t rating, double divisorAt80) -> float {
                if (rating < 0 || divisorAt80 <= 0.0) return -1.0f;
                double d = divisorAt80 * lvlScale;
                return static_cast<float>(rating / d);
            };

            if (hitRating >= 0) {
                float pct = ratingPct(hitRating, 26.23);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Hit Rating: %d (%.2f%%)", hitRating, pct);
                else
                    ImGui::TextColored(cyan, "Hit Rating: %d", hitRating);
            }
            // Show ranged/spell hit only when they differ from melee hit
            if (hitRangedR >= 0 && hitRangedR != hitRating) {
                float pct = ratingPct(hitRangedR, 26.23);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Ranged Hit Rating: %d (%.2f%%)", hitRangedR, pct);
                else
                    ImGui::TextColored(cyan, "Ranged Hit Rating: %d", hitRangedR);
            }
            if (hitSpellR >= 0 && hitSpellR != hitRating) {
                // Spell hit cap at 17% (446 rating at 80); divisor same as melee hit
                float pct = ratingPct(hitSpellR, 26.23);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Spell Hit Rating: %d (%.2f%%)", hitSpellR, pct);
                else
                    ImGui::TextColored(cyan, "Spell Hit Rating: %d", hitSpellR);
            }
            if (expertiseR >= 0) {
                // Each expertise point reduces dodge and parry chance by 0.25%
                // expertise_points = rating / 8.19
                float exp_pts = ratingPct(expertiseR, 8.19);
                if (exp_pts >= 0.0f) {
                    float exp_pct = exp_pts * 0.25f;  // % dodge/parry reduction
                    ImGui::TextColored(cyan, "Expertise: %d (%.1f / %.2f%%)",
                                       expertiseR, exp_pts, exp_pct);
                } else {
                    ImGui::TextColored(cyan, "Expertise Rating: %d", expertiseR);
                }
            }
            if (hasteR >= 0) {
                float pct = ratingPct(hasteR, 32.79);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Haste Rating: %d (%.2f%%)", hasteR, pct);
                else
                    ImGui::TextColored(cyan, "Haste Rating: %d", hasteR);
            }
            if (hasteRangedR >= 0 && hasteRangedR != hasteR) {
                float pct = ratingPct(hasteRangedR, 32.79);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Ranged Haste Rating: %d (%.2f%%)", hasteRangedR, pct);
                else
                    ImGui::TextColored(cyan, "Ranged Haste Rating: %d", hasteRangedR);
            }
            if (hasteSpellR >= 0 && hasteSpellR != hasteR) {
                float pct = ratingPct(hasteSpellR, 32.79);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Spell Haste Rating: %d (%.2f%%)", hasteSpellR, pct);
                else
                    ImGui::TextColored(cyan, "Spell Haste Rating: %d", hasteSpellR);
            }
            if (armorPenR >= 0) {
                float pct = ratingPct(armorPenR, 13.99);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Armor Pen: %d (%.2f%%)", armorPenR, pct);
                else
                    ImGui::TextColored(cyan, "Armor Penetration: %d", armorPenR);
            }
            if (resilR >= 0) {
                // Resilience: reduces crit chance against you by pct%, and crit damage by 2*pct%
                float pct = ratingPct(resilR, 94.27);
                if (pct >= 0.0f)
                    ImGui::TextColored(cyan, "Resilience: %d (%.2f%%)", resilR, pct);
                else
                    ImGui::TextColored(cyan, "Resilience: %d", resilR);
            }
        }

        // Movement speeds (always show when non-default)
        {
            constexpr float kBaseRun    = 7.0f;
            constexpr float kBaseFlight = 7.0f;
            float runSpeed    = gh->getServerRunSpeed();
            float flightSpeed = gh->getServerFlightSpeed();
            float swimSpeed   = gh->getServerSwimSpeed();

            bool showRun    = runSpeed    > 0.0f && std::fabs(runSpeed    - kBaseRun)    > 0.05f;
            bool showFlight = flightSpeed > 0.0f && std::fabs(flightSpeed - kBaseFlight) > 0.05f;
            bool showSwim   = swimSpeed   > 0.0f && std::fabs(swimSpeed   - 4.722f)      > 0.05f;

            if (showRun || showFlight || showSwim) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::TextColored(ui::colors::kWarmGold, "Movement");
                ImVec4 speedColor(0.6f, 1.0f, 0.8f, 1.0f);
                if (showRun) {
                    float pct = (runSpeed / kBaseRun) * 100.0f;
                    ImGui::TextColored(speedColor, "Run Speed: %.1f%%", pct);
                }
                if (showFlight) {
                    float pct = (flightSpeed / kBaseFlight) * 100.0f;
                    ImGui::TextColored(speedColor, "Flight Speed: %.1f%%", pct);
                }
                if (showSwim) {
                    float pct = (swimSpeed / 4.722f) * 100.0f;
                    ImGui::TextColored(speedColor, "Swim Speed: %.1f%%", pct);
                }
            }
        }
    }
}
void InventoryScreen::renderItemSlot(game::Inventory& inventory, const game::ItemSlot& slot,
                                      float size, const char* label,
                                      SlotKind kind, int backpackIndex,
                                      game::EquipSlot equipSlot,
                                      int bagIndex, int bagSlotIndex,
                                      int keyringIndex) {
    // Bag items are valid inventory slots even though backpackIndex is -1
    bool isBagSlot = (bagIndex >= 0 && bagSlotIndex >= 0);
    bool isKeyringSlot = (kind == SlotKind::KEYRING && keyringIndex >= 0);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 pos = ImGui::GetCursorScreenPos();

    bool isEmpty = slot.empty();

    // Determine if this is a valid drop target for held item
    bool validDrop = false;
    if (holdingItem) {
        if (kind == SlotKind::BACKPACK && (backpackIndex >= 0 || isBagSlot)) {
            validDrop = true;
        } else if (isKeyringSlot) {
            // The keyring only accepts keys; the server enforces this on the swap.
            validDrop = true;
        } else if (kind == SlotKind::EQUIPMENT && heldItem.inventoryType > 0) {
            if (heldItem.inventoryType == 18) {
                validDrop = (equipSlot >= game::EquipSlot::BAG1 && equipSlot <= game::EquipSlot::BAG4);
            } else {
                game::EquipSlot validSlot = getEquipSlotForType(heldItem.inventoryType, inventory);
                validDrop = (equipSlot == validSlot);
                if (!validDrop && heldItem.inventoryType == 11)
                    validDrop = (equipSlot == game::EquipSlot::RING1 || equipSlot == game::EquipSlot::RING2);
                if (!validDrop && heldItem.inventoryType == 12)
                    validDrop = (equipSlot == game::EquipSlot::TRINKET1 || equipSlot == game::EquipSlot::TRINKET2);
            }
        }
    }

    if (isEmpty) {
        ImU32 bgCol = IM_COL32(30, 30, 30, 200);
        ImU32 borderCol = IM_COL32(60, 60, 60, 200);

        if (validDrop) {
            bgCol = IM_COL32(20, 50, 20, 200);
            borderCol = IM_COL32(0, 180, 0, 200);
        }

        drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size), bgCol);
        drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size), borderCol);

        if (label) {
            char abbr[4] = {};
            abbr[0] = label[0];
            if (label[1]) abbr[1] = label[1];
            float textW = ImGui::CalcTextSize(abbr).x;
            drawList->AddText(ImVec2(pos.x + (size - textW) * 0.5f, pos.y + size * 0.3f),
                              IM_COL32(80, 80, 80, 180), abbr);
        }

        ImGui::InvisibleButton("slot", ImVec2(size, size));

        // Drop held item on mouse release over empty slot
        if (ImGui::IsItemHovered() && holdingItem && validDrop &&
            ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                placeInBackpack(inventory, backpackIndex);
            } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                placeInBag(inventory, bagIndex, bagSlotIndex);
            } else if (isKeyringSlot) {
                placeInKeyring(inventory, keyringIndex);
            } else if (kind == SlotKind::EQUIPMENT) {
                placeInEquipment(inventory, equipSlot);
            }
        }

        if (label && ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextColored(ui::colors::kDarkGray, "%s", label);
            ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "Empty");
            ImGui::EndTooltip();
        }
    } else {
        const auto& item = slot.item;
        ImVec4 qColor = getQualityColor(item.quality);
        ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(qColor);

        ImU32 bgCol = IM_COL32(40, 35, 30, 220);
        if (holdingItem && validDrop) {
            bgCol = IM_COL32(30, 55, 30, 220);
            borderCol = IM_COL32(0, 200, 0, 220);
        }

        // Try to show icon
        VkDescriptorSet iconTex = getItemIcon(item.displayInfoId);
        if (iconTex) {
            drawList->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                               ImVec2(pos.x + size, pos.y + size));
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, 2.0f);
        } else {
            drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size), bgCol);
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, 2.0f);

            char abbr[4] = {};
            if (!item.name.empty()) {
                abbr[0] = item.name[0];
                if (item.name.size() > 1) abbr[1] = item.name[1];
            }
            float textW = ImGui::CalcTextSize(abbr).x;
            drawList->AddText(ImVec2(pos.x + (size - textW) * 0.5f, pos.y + 2.0f),
                              ImGui::ColorConvertFloat4ToU32(qColor), abbr);
        }

        if (item.stackCount > 1) {
            char countStr[16];
            snprintf(countStr, sizeof(countStr), "%u", item.stackCount);
            float cw = ImGui::CalcTextSize(countStr).x;
            drawList->AddText(ImVec2(pos.x + size - cw - 2.0f, pos.y + size - 14.0f),
                              IM_COL32(255, 255, 255, 220), countStr);
        }

        // Durability bar on equipment slots (3px strip at bottom of slot icon)
        if (kind == SlotKind::EQUIPMENT && item.maxDurability > 0) {
            float durPct = static_cast<float>(item.curDurability) /
                           static_cast<float>(item.maxDurability);
            ImVec4 durRgb = ui::colors::durabilityColor(durPct);
            durRgb.w = 220.0f / 255.0f;  // the strip is drawn under the icon
            const ImU32 durCol = ImGui::ColorConvertFloat4ToU32(durRgb);
            float barW = size * durPct;
            drawList->AddRectFilled(ImVec2(pos.x, pos.y + size - 3.0f),
                                    ImVec2(pos.x + barW, pos.y + size),
                                    durCol);
        }

        ImGui::InvisibleButton("slot", ImVec2(size, size));

        // A used sharpening stone / weightstone / oil arms an item-target cursor:
        // this slot becomes the item it is applied to, and normal slot clicks are
        // suppressed until a target is chosen or the cursor is cancelled.
        const bool targetingItem = gameHandler_ && gameHandler_->isAwaitingItemTarget();

        // Left mouse: hold to pick up, release to drop/swap
        if (targetingItem && !holdingItem) {
            if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                uint64_t targetGuid = 0;
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    targetGuid = gameHandler_->getBackpackItemGuid(backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    targetGuid = gameHandler_->getBagItemGuid(bagIndex, bagSlotIndex);
                } else if (kind == SlotKind::EQUIPMENT) {
                    targetGuid = gameHandler_->getEquipSlotGuid(static_cast<int>(equipSlot));
                }
                // Empty slots are not targets - leave the cursor armed.
                if (targetGuid != 0) gameHandler_->completeItemUseOnItem(targetGuid);
            }
        } else if (!holdingItem) {
            // Start pickup tracking on mouse press
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
                pickupPending_ = true;
                pickupPressTime_ = ImGui::GetTime();
                pickupSlotKind_ = kind;
                pickupBackpackIndex_ = backpackIndex;
                pickupBagIndex_ = bagIndex;
                pickupBagSlotIndex_ = bagSlotIndex;
                pickupKeyringIndex_ = keyringIndex;
                pickupEquipSlot_ = equipSlot;
            }
            // Check if held long enough to pick up
            if (pickupPending_ && ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
                (ImGui::GetTime() - pickupPressTime_) >= kPickupHoldThreshold) {
                // Verify this is the same slot that was pressed
                bool sameSlot = (pickupSlotKind_ == kind);
                if (kind == SlotKind::BACKPACK && !isBagSlot)
                    sameSlot = sameSlot && (pickupBackpackIndex_ == backpackIndex);
                else if (kind == SlotKind::BACKPACK && isBagSlot)
                    sameSlot = sameSlot && (pickupBagIndex_ == bagIndex) && (pickupBagSlotIndex_ == bagSlotIndex);
                else if (isKeyringSlot)
                    sameSlot = sameSlot && (pickupKeyringIndex_ == keyringIndex);
                else if (kind == SlotKind::EQUIPMENT)
                    sameSlot = sameSlot && (pickupEquipSlot_ == equipSlot);

                if (sameSlot && ImGui::IsItemHovered()) {
                    pickupPending_ = false;
                    if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                        pickupFromBackpack(inventory, backpackIndex);
                    } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                        pickupFromBag(inventory, bagIndex, bagSlotIndex);
                    } else if (isKeyringSlot) {
                        pickupFromKeyring(inventory, keyringIndex);
                    } else if (kind == SlotKind::EQUIPMENT) {
                        pickupFromEquipment(inventory, equipSlot);
                    }
                }
            }
        } else {
            // Drop/swap on mouse release over a filled slot
            if (ImGui::IsItemHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    placeInBackpack(inventory, backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    placeInBag(inventory, bagIndex, bagSlotIndex);
                } else if (isKeyringSlot && validDrop) {
                    placeInKeyring(inventory, keyringIndex);
                } else if (kind == SlotKind::EQUIPMENT && validDrop) {
                    placeInEquipment(inventory, equipSlot);
                }
            }
        }

        // Shift+right-click: split stack (if stackable >1) or destroy confirmation
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
            !holdingItem && !targetingItem && ImGui::GetIO().KeyShift && item.itemId != 0) {
            if (item.stackCount > 1 && item.maxStack > 1) {
                // Open split popup for stackable items
                splitConfirmOpen_ = true;
                splitItemName_ = item.name;
                splitMax_ = static_cast<int>(item.stackCount);
                splitCount_ = splitMax_ / 2;
                if (splitCount_ < 1) splitCount_ = 1;
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    splitBag_ = 0xFF;
                    splitSlot_ = game::slots::backpackWireSlot(backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    splitBag_ = static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex));
                    splitSlot_ = static_cast<uint8_t>(bagSlotIndex);
                }
            } else if (item.bindType != 4) {
                // Destroy confirmation for non-quest, non-stackable items
                destroyConfirmOpen_ = true;
                destroyItemName_ = item.name;
                destroyCount_ = static_cast<uint8_t>(std::clamp<uint32_t>(
                    std::max<uint32_t>(1u, item.stackCount), 1u, 255u));
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    destroyBag_ = 0xFF;
                    destroySlot_ = game::slots::backpackWireSlot(backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    destroyBag_ = static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex));
                    destroySlot_ = static_cast<uint8_t>(bagSlotIndex);
                } else if (kind == SlotKind::EQUIPMENT) {
                    destroyBag_ = 0xFF;
                    destroySlot_ = static_cast<uint8_t>(equipSlot);
                }
            }
        }

        // Right-click: bank deposit (if bank open), vendor sell (if vendor mode), or auto-equip/use
        // Note: InvisibleButton only tracks left-click by default, so use IsItemHovered+IsMouseClicked
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !holdingItem && !targetingItem && !ImGui::GetIO().KeyShift && gameHandler_) {
            LOG_DEBUG("Right-click slot: kind=", static_cast<int>(kind),
                     " backpackIndex=", backpackIndex,
                     " bagIndex=", bagIndex, " bagSlotIndex=", bagSlotIndex,
                     " vendorMode=", vendorMode_,
                     " bankOpen=", gameHandler_->isBankOpen(),
                     " item='", item.name, "' invType=", static_cast<int>(item.inventoryType));
            if (gameHandler_->isMailComposeOpen() && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                gameHandler_->attachItemFromBackpack(backpackIndex);
            } else if (gameHandler_->isMailComposeOpen() && kind == SlotKind::BACKPACK && isBagSlot) {
                gameHandler_->attachItemFromBag(bagIndex, bagSlotIndex);
            } else if (gameHandler_->isBankOpen() && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                gameHandler_->depositItem(0xFF, game::slots::backpackWireSlot(backpackIndex));
            } else if (gameHandler_->isBankOpen() && kind == SlotKind::BACKPACK && isBagSlot) {
                gameHandler_->depositItem(static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex)), static_cast<uint8_t>(bagSlotIndex));
            } else if (gameHandler_->isGuildBankOpen() && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                gameHandler_->guildBankDepositFromInventory(0xFF, game::slots::backpackWireSlot(backpackIndex));
            } else if (gameHandler_->isGuildBankOpen() && kind == SlotKind::BACKPACK && isBagSlot) {
                gameHandler_->guildBankDepositFromInventory(static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex)), static_cast<uint8_t>(bagSlotIndex));
            } else if (vendorMode_ && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                gameHandler_->sellItemBySlot(backpackIndex);
            } else if (vendorMode_ && kind == SlotKind::BACKPACK && isBagSlot) {
                gameHandler_->sellItemInBag(bagIndex, bagSlotIndex);
            } else if (kind == SlotKind::EQUIPMENT) {
                LOG_INFO("UI unequip request: equipSlot=", static_cast<int>(equipSlot));
                gameHandler_->unequipToBackpack(equipSlot);
            } else if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                LOG_INFO("Right-click backpack item: name='", item.name,
                         "' inventoryType=", static_cast<int>(item.inventoryType),
                         " itemId=", item.itemId,
                         " startQuestId=", item.startQuestId);
                if (item.startQuestId != 0) {
                    uint64_t iGuid = gameHandler_->getBackpackItemGuid(backpackIndex);
                    gameHandler_->offerQuestFromItem(iGuid, item.startQuestId);
                } else if (item.inventoryType > 0) {
                    if (gameHandler_->equipWouldBindFromBackpack(backpackIndex)) {
                        equipConfirmOpen_ = true;
                        equipConfirmAuto_ = true;
                        equipConfirmBag_ = 0xFF;
                        equipConfirmSourceSlot_ = static_cast<uint8_t>(game::Inventory::NUM_EQUIP_SLOTS + backpackIndex);
                        equipConfirmItemName_ = item.name;
                    } else {
                        gameHandler_->autoEquipItemBySlot(backpackIndex);
                    }
                } else {
                    // Openable is an item flag, not an item-class guarantee. Some fishing
                    // containers are miscellaneous items and otherwise fall through to an
                    // empty CMSG_USE_ITEM that the server silently ignores.
                    auto* info = gameHandler_->getItemInfo(item.itemId);
                    if (info && info->valid && info->pageTextId != 0) {
                        gameHandler_->readItemBySlot(backpackIndex);
                    } else if (info && info->valid &&
                               ((info->itemFlags & kItemFlagOpenable) != 0 ||
                                info->itemClass == 1)) {
                        gameHandler_->openItemBySlot(backpackIndex);
                    } else {
                        // No bind warning in this window - see the loot click in
                        // window_manager.cpp for why that is said here.
                        gameHandler_->useItemBySlot(backpackIndex, true);
                    }
                }
            } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                LOG_INFO("Right-click bag item: name='", item.name,
                         "' inventoryType=", static_cast<int>(item.inventoryType),
                         " bagIndex=", bagIndex, " slotIndex=", bagSlotIndex,
                         " startQuestId=", item.startQuestId);
                if (item.startQuestId != 0) {
                    uint64_t iGuid = gameHandler_->getBagItemGuid(bagIndex, bagSlotIndex);
                    gameHandler_->offerQuestFromItem(iGuid, item.startQuestId);
                } else if (item.inventoryType > 0) {
                    if (gameHandler_->equipWouldBindFromBag(bagIndex, bagSlotIndex)) {
                        equipConfirmOpen_ = true;
                        equipConfirmAuto_ = true;
                        equipConfirmBag_ = static_cast<uint8_t>(bagIndex);
                        equipConfirmSourceSlot_ = static_cast<uint8_t>(bagSlotIndex);
                        equipConfirmItemName_ = item.name;
                    } else {
                        gameHandler_->autoEquipItemInBag(bagIndex, bagSlotIndex);
                    }
                } else {
                    auto* info = gameHandler_->getItemInfo(item.itemId);
                    if (info && info->valid && info->pageTextId != 0) {
                        gameHandler_->readItemInBag(bagIndex, bagSlotIndex);
                    } else if (info && info->valid &&
                               ((info->itemFlags & kItemFlagOpenable) != 0 ||
                                info->itemClass == 1)) {
                        gameHandler_->openItemInBag(bagIndex, bagSlotIndex);
                    } else {
                        gameHandler_->useItemInBag(bagIndex, bagSlotIndex, true);
                    }
                }
            } else if (isKeyringSlot) {
                // Right-click a key to use it (e.g. opening a nearby door/object).
                gameHandler_->useItemById(item.itemId);
            }
        }

        // Shift+left-click: insert item link into chat input
        if (ImGui::IsItemHovered() && !holdingItem && !targetingItem &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            ImGui::GetIO().KeyShift &&
            item.itemId != 0 && !item.name.empty()) {
            // A shift-click writes the same link a Lua binding would, colour
            // and all nine fields, rather than a sixth spelling of it.
            pendingChatItemLink_ = game::itemChatLink(
                item.itemId, static_cast<uint32_t>(item.quality), item.name);
        }

        if (ImGui::IsItemHovered() && !holdingItem) {
            // Pass inventory for backpack/bag items only; equipped items compare against themselves otherwise
            const game::Inventory* tooltipInv = (kind == SlotKind::EQUIPMENT) ? nullptr : &inventory;
            // ItemDef carries the exact object GUID for backpack and bag items.
            // Previously only equipment supplied a GUID, so hovered candidate
            // enchants/gems disappeared as soon as Shift comparison was used.
            uint64_t slotGuid = item.guid;
            if (slotGuid == 0 && kind == SlotKind::EQUIPMENT && gameHandler_)
                slotGuid = gameHandler_->getEquipSlotGuid(static_cast<int>(equipSlot));
            renderItemTooltip(item, tooltipInv, slotGuid);
        }
    }
}




} // namespace ui
} // namespace wowee
