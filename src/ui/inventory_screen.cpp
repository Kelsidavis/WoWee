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

// Keep the complete bag window reachable after monitor/resolution changes or
// a stale imgui.ini position. Merely checking for total off-screen placement
// leaves a narrow sliver visible and makes the title bar almost impossible to
// grab, which is worse than restoring it to the viewport edge.





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
// Equip slot helpers
// ============================================================




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




// ============================================================
// Bags window (B key) - bottom of screen, no equipment panel
// ============================================================

// ============================================================
// Aggregate mode - original single-window bags
// ============================================================

// ============================================================
// Separate mode - individual draggable bag windows
// ============================================================

// ============================================================
// Character screen (C key) - equipment + model preview + stats
// ============================================================

// ============================================================
// Stats Panel
// ============================================================




} // namespace ui
} // namespace wowee
