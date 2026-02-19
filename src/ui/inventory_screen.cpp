#include "ui/inventory_screen.hpp"
#include "game/game_handler.hpp"
#include "core/application.hpp"
#include "core/input.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/character_renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <SDL2/SDL.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <unordered_set>

namespace wowee {
namespace ui {

namespace {
const game::ItemSlot* findComparableEquipped(const game::Inventory& inventory, uint8_t inventoryType) {
    using ES = game::EquipSlot;
    auto slotPtr = [&](ES slot) -> const game::ItemSlot* {
        const auto& s = inventory.getEquipSlot(slot);
        return s.empty() ? nullptr : &s;
    };

    switch (inventoryType) {
        case 1: return slotPtr(ES::HEAD);
        case 2: return slotPtr(ES::NECK);
        case 3: return slotPtr(ES::SHOULDERS);
        case 4: return slotPtr(ES::SHIRT);
        case 5:
        case 20: return slotPtr(ES::CHEST);
        case 6: return slotPtr(ES::WAIST);
        case 7: return slotPtr(ES::LEGS);
        case 8: return slotPtr(ES::FEET);
        case 9: return slotPtr(ES::WRISTS);
        case 10: return slotPtr(ES::HANDS);
        case 11: {
            if (auto* s = slotPtr(ES::RING1)) return s;
            return slotPtr(ES::RING2);
        }
        case 12: {
            if (auto* s = slotPtr(ES::TRINKET1)) return s;
            return slotPtr(ES::TRINKET2);
        }
        case 13: // One-hand
            if (auto* s = slotPtr(ES::MAIN_HAND)) return s;
            return slotPtr(ES::OFF_HAND);
        case 14:
        case 22:
        case 23: return slotPtr(ES::OFF_HAND);
        case 15:
        case 25:
        case 26: return slotPtr(ES::RANGED);
        case 16: return slotPtr(ES::BACK);
        case 17:
        case 21: return slotPtr(ES::MAIN_HAND);
        case 18: // bag
            for (int i = 0; i < game::Inventory::NUM_BAG_SLOTS; ++i) {
                auto slot = static_cast<ES>(static_cast<int>(ES::BAG1) + i);
                if (auto* s = slotPtr(slot)) return s;
            }
            return nullptr;
        case 19: return slotPtr(ES::TABARD);
        default: return nullptr;
    }
}
} // namespace

InventoryScreen::~InventoryScreen() {
    // Clean up icon textures
    for (auto& [id, tex] : iconCache_) {
        if (tex) glDeleteTextures(1, &tex);
    }
    iconCache_.clear();
}

ImVec4 InventoryScreen::getQualityColor(game::ItemQuality quality) {
    switch (quality) {
        case game::ItemQuality::POOR:      return ImVec4(0.62f, 0.62f, 0.62f, 1.0f); // Grey
        case game::ItemQuality::COMMON:    return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);    // White
        case game::ItemQuality::UNCOMMON:  return ImVec4(0.12f, 1.0f, 0.0f, 1.0f);   // Green
        case game::ItemQuality::RARE:      return ImVec4(0.0f, 0.44f, 0.87f, 1.0f);  // Blue
        case game::ItemQuality::EPIC:      return ImVec4(0.64f, 0.21f, 0.93f, 1.0f); // Purple
        case game::ItemQuality::LEGENDARY: return ImVec4(1.0f, 0.50f, 0.0f, 1.0f);   // Orange
        default:                           return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    }
}

// ============================================================
// Item Icon Loading
// ============================================================

GLuint InventoryScreen::getItemIcon(uint32_t displayInfoId) {
    if (displayInfoId == 0 || !assetManager_) return 0;

    auto it = iconCache_.find(displayInfoId);
    if (it != iconCache_.end()) return it->second;

    // Load ItemDisplayInfo.dbc
    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) {
        iconCache_[displayInfoId] = 0;
        return 0;
    }

    int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
    if (recIdx < 0) {
        iconCache_[displayInfoId] = 0;
        return 0;
    }

    // Field 5 = inventoryIcon_1
    const auto* dispL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    std::string iconName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), dispL ? (*dispL)["InventoryIcon"] : 5);
    if (iconName.empty()) {
        iconCache_[displayInfoId] = 0;
        return 0;
    }

    std::string iconPath = "Interface\\Icons\\" + iconName + ".blp";
    auto blpData = assetManager_->readFile(iconPath);
    if (blpData.empty()) {
        iconCache_[displayInfoId] = 0;
        return 0;
    }

    auto image = pipeline::BLPLoader::load(blpData);
    if (!image.isValid()) {
        iconCache_[displayInfoId] = 0;
        return 0;
    }

    GLuint texId = 0;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.width, image.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, image.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    iconCache_[displayInfoId] = texId;
    return texId;
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
    }

    charPreview_->loadCharacter(playerRace_, playerGender_,
                                 playerSkin_, playerFace_,
                                 playerHairStyle_, playerHairColor_,
                                 playerFacialHair_);
    previewInitialized_ = true;
    previewDirty_ = true; // apply equipment on first load
}

void InventoryScreen::updatePreview(float deltaTime) {
    if (charPreview_ && previewInitialized_) {
        charPreview_->update(deltaTime);
    }
}

void InventoryScreen::updatePreviewEquipment(game::Inventory& inventory) {
    if (!charPreview_ || !charPreview_->isModelLoaded() || !assetManager_) return;

    auto* charRenderer = charPreview_->getCharacterRenderer();
    uint32_t instanceId = charPreview_->getInstanceId();
    if (!charRenderer || instanceId == 0) return;

    // --- Geosets (mirroring GameScreen::updateCharacterGeosets) ---
    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");

    auto getGeosetGroup = [&](uint32_t displayInfoId, int groupField) -> uint32_t {
        if (!displayInfoDbc || displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), 7 + groupField);
    };

    auto findEquippedDisplayId = [&](std::initializer_list<uint8_t> types) -> uint32_t {
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
            if (!slot.empty()) {
                for (uint8_t t : types) {
                    if (slot.item.inventoryType == t)
                        return slot.item.displayInfoId;
                }
            }
        }
        return 0;
    };

    auto hasEquippedType = [&](std::initializer_list<uint8_t> types) -> bool {
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
            if (!slot.empty()) {
                for (uint8_t t : types) {
                    if (slot.item.inventoryType == t) return true;
                }
            }
        }
        return false;
    };

    std::unordered_set<uint16_t> geosets;
    // Body parts (group 0: IDs 0-99, some models use up to 27)
    for (uint16_t i = 0; i <= 99; i++) geosets.insert(i);

    // Hair geoset: group 1 = 100 + hairStyle + 1
    geosets.insert(static_cast<uint16_t>(100 + playerHairStyle_ + 1));
    // Facial hair geoset: group 2 = 200 + facialHair + 1
    geosets.insert(static_cast<uint16_t>(200 + playerFacialHair_ + 1));
    geosets.insert(701);  // Ears

    // Chest/Shirt
    {
        uint32_t did = findEquippedDisplayId({4, 5, 20});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 501 + gg : 501));
        uint32_t gg3 = getGeosetGroup(did, 2);
        if (gg3 > 0) {
            geosets.insert(static_cast<uint16_t>(1301 + gg3));
        }
    }

    // Legs
    {
        uint32_t did = findEquippedDisplayId({7});
        uint32_t gg = getGeosetGroup(did, 0);
        if (geosets.count(1302) == 0 && geosets.count(1303) == 0) {
            geosets.insert(static_cast<uint16_t>(gg > 0 ? 1301 + gg : 1301));
        }
    }

    // Feet
    {
        uint32_t did = findEquippedDisplayId({8});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 401 + gg : 401));
    }

    // Gloves
    {
        uint32_t did = findEquippedDisplayId({10});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 301 + gg : 301));
    }

    // Cloak
    geosets.insert(hasEquippedType({16}) ? 1502 : 1501);

    // Tabard
    if (hasEquippedType({19})) {
        geosets.insert(1201);
    }

    charRenderer->setActiveGeosets(instanceId, geosets);

    // --- Textures (mirroring GameScreen::updateCharacterTextures) ---
    auto& app = core::Application::getInstance();
    const auto& bodySkinPath = app.getBodySkinPath();
    const auto& underwearPaths = app.getUnderwearPaths();

    if (bodySkinPath.empty() || !displayInfoDbc) return;

    static const char* componentDirs[] = {
        "ArmUpperTexture", "ArmLowerTexture", "HandTexture",
        "TorsoUpperTexture", "TorsoLowerTexture",
        "LegUpperTexture", "LegLowerTexture", "FootTexture",
    };

    std::vector<std::pair<int, std::string>> regionLayers;
    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (slot.empty() || slot.item.displayInfoId == 0) continue;

        int32_t recIdx = displayInfoDbc->findRecordById(slot.item.displayInfoId);
        if (recIdx < 0) continue;

        for (int region = 0; region < 8; region++) {
            uint32_t fieldIdx = 14 + region;
            std::string texName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), fieldIdx);
            if (texName.empty()) continue;

            std::string base = "Item\\TextureComponents\\" +
                std::string(componentDirs[region]) + "\\" + texName;
            std::string genderSuffix = (playerGender_ == game::Gender::FEMALE) ? "_F.blp" : "_M.blp";
            std::string genderPath = base + genderSuffix;
            std::string unisexPath = base + "_U.blp";
            std::string fullPath;
            if (assetManager_->fileExists(genderPath)) {
                fullPath = genderPath;
            } else if (assetManager_->fileExists(unisexPath)) {
                fullPath = unisexPath;
            } else {
                fullPath = base + ".blp";
            }
            regionLayers.emplace_back(region, fullPath);
        }
    }

    // Find the skin texture slot index in the preview model
    // The preview model uses model ID PREVIEW_MODEL_ID; find slot for type-1 (body skin)
    const auto* modelData = charRenderer->getModelData(charPreview_->getModelId());
    uint32_t skinSlot = 0;
    if (modelData) {
        for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
            if (modelData->textures[ti].type == 1) {
                skinSlot = static_cast<uint32_t>(ti);
                break;
            }
        }
    }

    GLuint newTex = charRenderer->compositeWithRegions(bodySkinPath, underwearPaths, regionLayers);
    if (newTex != 0) {
        charRenderer->setModelTexture(charPreview_->getModelId(), skinSlot, newTex);
    }

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
        case 19: return game::EquipSlot::TABARD;
        case 20: return game::EquipSlot::CHEST; // Robe
        default: return game::EquipSlot::NUM_SLOTS;
    }
}

void InventoryScreen::pickupFromBackpack(game::Inventory& inv, int index) {
    const auto& slot = inv.getBackpackSlot(index);
    if (slot.empty()) return;
    holdingItem = true;
    heldItem = slot.item;
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
    heldSource = HeldSource::EQUIPMENT;
    heldBackpackIndex = -1;
    heldEquipSlot = slot;
    inv.clearEquipSlot(slot);
    equipmentDirty = true;
    inventoryDirty = true;
}

void InventoryScreen::placeInBackpack(game::Inventory& inv, int index) {
    if (!holdingItem) return;
    if (gameHandler_) {
        // Online mode: send server swap packet for all container moves
        uint8_t dstBag = 0xFF;
        uint8_t dstSlot = static_cast<uint8_t>(23 + index);
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
            srcSlot = static_cast<uint8_t>(23 + heldBackpackIndex);
        } else if (heldSource == HeldSource::BAG) {
            srcBag = static_cast<uint8_t>(19 + heldBagIndex);
            srcSlot = static_cast<uint8_t>(heldBagSlotIndex);
        } else if (heldSource == HeldSource::EQUIPMENT) {
            srcSlot = static_cast<uint8_t>(heldEquipSlot);
        } else {
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
        uint8_t dstBag = static_cast<uint8_t>(19 + bagIndex);
        uint8_t dstSlot = static_cast<uint8_t>(slotIndex);
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
            srcSlot = static_cast<uint8_t>(23 + heldBackpackIndex);
        } else if (heldSource == HeldSource::BAG) {
            srcBag = static_cast<uint8_t>(19 + heldBagIndex);
            srcSlot = static_cast<uint8_t>(heldBagSlotIndex);
        } else if (heldSource == HeldSource::EQUIPMENT) {
            srcSlot = static_cast<uint8_t>(heldEquipSlot);
        } else {
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

void InventoryScreen::placeInEquipment(game::Inventory& inv, game::EquipSlot slot) {
    if (!holdingItem) return;
    if (gameHandler_) {
        if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
            gameHandler_->autoEquipItemBySlot(heldBackpackIndex);
            cancelPickup(inv);
            return;
        }
        if (heldSource == HeldSource::BAG) {
            gameHandler_->autoEquipItemInBag(heldBagIndex, heldBagSlotIndex);
            cancelPickup(inv);
            return;
        }
        if (heldSource == HeldSource::EQUIPMENT) {
            cancelPickup(inv);
            return;
        }
    }

    // Validate: check if the held item can go in this slot
    if (heldItem.inventoryType > 0) {
        game::EquipSlot validSlot = getEquipSlotForType(heldItem.inventoryType, inv);
        if (validSlot == game::EquipSlot::NUM_SLOTS) return;

        bool valid = (slot == validSlot);
        if (!valid) {
            if (heldItem.inventoryType == 11)
                valid = (slot == game::EquipSlot::RING1 || slot == game::EquipSlot::RING2);
            else if (heldItem.inventoryType == 12)
                valid = (slot == game::EquipSlot::TRINKET1 || slot == game::EquipSlot::TRINKET2);
        }
        if (!valid) return;
    } else {
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
    } else {
        inv.addItem(heldItem);
    }
    holdingItem = false;
    inventoryDirty = true;
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
    GLuint iconTex = getItemIcon(heldItem.displayInfoId);
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
// Bags window (B key) — bottom of screen, no equipment panel
// ============================================================

void InventoryScreen::toggleBackpack() {
    backpackOpen_ = !backpackOpen_;
}

void InventoryScreen::toggleBag(int idx) {
    if (idx >= 0 && idx < 4) {
        bagOpen_[idx] = !bagOpen_[idx];
        if (bagOpen_[idx]) {
            // Keep backpack as the anchor window at the bottom of the stack.
            backpackOpen_ = true;
        }
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

bool InventoryScreen::bagHasAnyItems(const game::Inventory& inventory, int bagIndex) const {
    int bagSize = inventory.getBagSize(bagIndex);
    if (bagSize <= 0) return false;
    for (int i = 0; i < bagSize; ++i) {
        if (!inventory.getBagSlot(bagIndex, i).empty()) return true;
    }
    return false;
}

void InventoryScreen::render(game::Inventory& inventory, uint64_t moneyCopper) {
    // B key toggle (edge-triggered)
    bool wantsTextInput = ImGui::GetIO().WantTextInput;
    bool bDown = !wantsTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_B);
    bool bToggled = bDown && !bKeyWasDown;
    bKeyWasDown = bDown;

    // C key toggle for character screen (edge-triggered)
    bool cDown = !wantsTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_C);
    if (cDown && !cKeyWasDown) {
        characterOpen = !characterOpen;
    }
    cKeyWasDown = cDown;

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
        if (holdingItem) cancelPickup(inventory);
        return;
    }

    // Escape cancels held item
    if (holdingItem && !wantsTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_ESCAPE)) {
        cancelPickup(inventory);
    }

    // Right-click anywhere while holding = cancel
    if (holdingItem && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        cancelPickup(inventory);
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

    // Drop item confirmation popup — positioned near cursor
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
            holdingItem = false;
            heldItem = game::ItemDef{};
            heldSource = HeldSource::NONE;
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

    // Draw held item at cursor
    renderHeldItem();
}

// ============================================================
// Aggregate mode — original single-window bags
// ============================================================

void InventoryScreen::renderAggregateBags(game::Inventory& inventory, uint64_t moneyCopper) {
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    constexpr float slotSize = 40.0f;
    constexpr int columns = 6;
    int rows = (inventory.getBackpackSize() + columns - 1) / columns;
    float bagContentH = rows * (slotSize + 4.0f) + 40.0f;

    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; bag++) {
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) continue;
        if (compactBags_ && !bagHasAnyItems(inventory, bag)) continue;
        int bagRows = (bagSize + columns - 1) / columns;
        bagContentH += bagRows * (slotSize + 4.0f) + 30.0f;
    }

    float windowW = columns * (slotSize + 4.0f) + 30.0f;
    float windowH = bagContentH + 50.0f;

    float posX = screenW - windowW - 10.0f;
    float posY = screenH - windowH - 60.0f;

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    if (!ImGui::Begin("Bags", &open, flags)) {
        ImGui::End();
        return;
    }

    renderBackpackPanel(inventory, compactBags_);

    ImGui::Spacing();
    uint64_t gold = moneyCopper / 10000;
    uint64_t silver = (moneyCopper / 100) % 100;
    uint64_t copper = moneyCopper % 100;
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "%llug %llus %lluc",
                       static_cast<unsigned long long>(gold),
                       static_cast<unsigned long long>(silver),
                       static_cast<unsigned long long>(copper));
    ImGui::SameLine();
    const char* collapseLabel = compactBags_ ? "Expand Empty" : "Collapse Empty";
    const float btnW = 92.0f;
    const float rightMargin = 8.0f;
    float rightX = ImGui::GetWindowContentRegionMax().x - btnW - rightMargin;
    if (rightX > ImGui::GetCursorPosX()) ImGui::SetCursorPosX(rightX);
    if (ImGui::SmallButton(collapseLabel)) {
        compactBags_ = !compactBags_;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Toggle empty bag section visibility");
    }
    ImGui::End();
}

// ============================================================
// Separate mode — individual draggable bag windows
// ============================================================

void InventoryScreen::renderSeparateBags(game::Inventory& inventory, uint64_t moneyCopper) {
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    constexpr float slotSize = 40.0f;
    constexpr int columns = 6;
    constexpr float baseWindowW = columns * (slotSize + 4.0f) + 30.0f;

    bool anyBagOpen = std::any_of(bagOpen_.begin(), bagOpen_.end(), [](bool b) { return b; });
    if (anyBagOpen && !backpackOpen_) {
        // Enforce backpack as the bottom-most stack window when any bag is open.
        backpackOpen_ = true;
    }

    // Anchor stack to the bag bar (bottom-right), opening upward.
    const float bagBarTop = screenH - (42.0f + 12.0f) - 10.0f;
    const float stackGap = 8.0f;
    float stackBottom = bagBarTop - stackGap;
    float stackX = screenW - baseWindowW - 10.0f;

    // Backpack window (bottom of stack)
    if (backpackOpen_) {
        int bpRows = (inventory.getBackpackSize() + columns - 1) / columns;
        float bpH = bpRows * (slotSize + 4.0f) + 80.0f; // header + money + padding
        float defaultY = stackBottom - bpH;
        renderBagWindow("Backpack", backpackOpen_, inventory, -1, stackX, defaultY, moneyCopper);
        stackBottom = defaultY - stackGap;
    }

    // Extra bag windows in right-to-left bag-bar order (closest to backpack first).
    constexpr int kBagOrder[game::Inventory::NUM_BAG_SLOTS] = {3, 2, 1, 0};
    for (int ord = 0; ord < game::Inventory::NUM_BAG_SLOTS; ++ord) {
        int bag = kBagOrder[ord];
        if (!bagOpen_[bag]) continue;
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) {
            bagOpen_[bag] = false;
            continue;
        }
        // In separate-bag mode, never auto-hide empty bags. Players still need
        // to open empty bags to move items into them.

        int bagRows = (bagSize + columns - 1) / columns;
        float bagH = bagRows * (slotSize + 4.0f) + 60.0f;
        float defaultY = stackBottom - bagH;
        stackBottom = defaultY - stackGap;

        // Build title from equipped bag item name
        char title[64];
        game::EquipSlot bagSlot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + bag);
        const auto& bagItem = inventory.getEquipSlot(bagSlot);
        if (!bagItem.empty() && !bagItem.item.name.empty()) {
            snprintf(title, sizeof(title), "%s##bag%d", bagItem.item.name.c_str(), bag);
        } else {
            snprintf(title, sizeof(title), "Bag Slot %d##bag%d", bag + 1, bag);
        }

        renderBagWindow(title, bagOpen_[bag], inventory, bag, stackX, defaultY, 0);
    }

    // Update open state based on individual windows
    open = backpackOpen_ || std::any_of(bagOpen_.begin(), bagOpen_.end(), [](bool b){ return b; });
}

void InventoryScreen::renderBagWindow(const char* title, bool& isOpen,
                                       game::Inventory& inventory, int bagIndex,
                                       float defaultX, float defaultY, uint64_t moneyCopper) {
    constexpr float slotSize = 40.0f;
    constexpr int columns = 6;

    int numSlots = (bagIndex < 0) ? inventory.getBackpackSize() : inventory.getBagSize(bagIndex);
    if (numSlots <= 0) return;

    int rows = (numSlots + columns - 1) / columns;
    float contentH = rows * (slotSize + 4.0f) + 10.0f;
    if (bagIndex < 0) contentH += 25.0f; // money display for backpack
    float gridW = columns * (slotSize + 4.0f) + 30.0f;
    // Ensure window is wide enough for the title + close button
    const char* displayTitle = title;
    const char* hashPos = strstr(title, "##");
    float titleW = ImGui::CalcTextSize(displayTitle, hashPos).x + 50.0f; // close button + padding
    float windowW = std::max(gridW, titleW);
    float windowH = contentH + 40.0f; // title bar + padding

    // Keep separate bag windows anchored to the bag-bar stack.
    ImGui::SetNextWindowPos(ImVec2(defaultX, defaultY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
    if (!ImGui::Begin(title, &isOpen, flags)) {
        ImGui::End();
        return;
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
            // Bag slot - pass bag index info for interactions
            renderItemSlot(inventory, slot, slotSize, nullptr,
                           SlotKind::BACKPACK, -1, game::EquipSlot::NUM_SLOTS,
                           bagIndex, i);
        }
        ImGui::PopID();
    }

    // Money display at bottom of backpack
    if (bagIndex < 0 && moneyCopper > 0) {
        ImGui::Spacing();
        uint64_t gold = moneyCopper / 10000;
        uint64_t silver = (moneyCopper / 100) % 100;
        uint64_t copper = moneyCopper % 100;
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "%llug %llus %lluc",
                           static_cast<unsigned long long>(gold),
                           static_cast<unsigned long long>(silver),
                           static_cast<unsigned long long>(copper));
    }

    ImGui::End();
}

// ============================================================
// Character screen (C key) — equipment + model preview + stats
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
        updatePreviewEquipment(inventory);
    }

    // Update and render the preview FBO
    if (charPreview_ && previewInitialized_) {
        charPreview_->update(ImGui::GetIO().DeltaTime);
        charPreview_->render();
    }

    ImGui::SetNextWindowPos(ImVec2(20.0f, 80.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0f, 650.0f), ImGuiCond_FirstUseEver);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    if (!ImGui::Begin("Character", &characterOpen, flags)) {
        ImGui::End();
        return;
    }

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
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Stats")) {
            ImGui::Spacing();
            renderStatsPanel(inventory, gameHandler.getPlayerLevel());
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
                static const CategoryGroup groups[] = {
                    { "Weapon Skills", 6 },
                    { "Armor Skills", 8 },
                    { "Secondary Skills", 10 },
                    { "Professions", 11 },
                    { "Languages", 9 },
                    { "Other", 0 },
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

                            // Show progress bar with value/max overlay
                            float ratio = (skill->maxValue > 0)
                                ? static_cast<float>(skill->value) / static_cast<float>(skill->maxValue)
                                : 0.0f;

                            char overlay[64];
                            snprintf(overlay, sizeof(overlay), "%u / %u", skill->value, skill->maxValue);

                            ImGui::Text("%s", label);
                            ImGui::SameLine(180.0f);
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::ProgressBar(ratio, ImVec2(0, 14.0f), overlay);
                        }
                    }
                }

                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();

    // If both bags and character are open, allow drag-and-drop between them
    // (held item rendering is handled in render())
    if (open) {
        renderHeldItem();
    }
}

void InventoryScreen::renderEquipmentPanel(game::Inventory& inventory) {
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Equipment");
    ImGui::Separator();

    static const game::EquipSlot leftSlots[] = {
        game::EquipSlot::HEAD, game::EquipSlot::NECK,
        game::EquipSlot::SHOULDERS, game::EquipSlot::BACK,
        game::EquipSlot::CHEST, game::EquipSlot::SHIRT,
        game::EquipSlot::TABARD, game::EquipSlot::WRISTS,
    };
    static const game::EquipSlot rightSlots[] = {
        game::EquipSlot::HANDS, game::EquipSlot::WAIST,
        game::EquipSlot::LEGS, game::EquipSlot::FEET,
        game::EquipSlot::RING1, game::EquipSlot::RING2,
        game::EquipSlot::TRINKET1, game::EquipSlot::TRINKET2,
    };

    constexpr float slotSize = 36.0f;
    constexpr float previewW = 140.0f;

    // Calculate column positions for the 3-column layout
    float contentStartX = ImGui::GetCursorPosX();
    float rightColX = contentStartX + slotSize + 8.0f + previewW + 8.0f;

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
        float previewX = ImGui::GetWindowPos().x + contentStartX + slotSize + 8.0f;
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
            (ImTextureID)(uintptr_t)charPreview_->getTextureId(),
            pMin, pMax,
            ImVec2(0, 1), ImVec2(1, 0));  // flip Y for GL
        drawList->AddRect(pMin, pMax, IM_COL32(60, 60, 80, 200));

        // Drag-to-rotate: detect mouse drag over the preview image
        ImGui::SetCursorScreenPos(pMin);
        ImGui::InvisibleButton("##charPreviewDrag", ImVec2(displayW, displayH));
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            float dx = ImGui::GetIO().MouseDelta.x;
            charPreview_->rotate(dx * 1.0f);
        }
    }

    // Weapon row
    ImGui::Spacing();
    ImGui::Separator();

    static const game::EquipSlot weaponSlots[] = {
        game::EquipSlot::MAIN_HAND,
        game::EquipSlot::OFF_HAND,
        game::EquipSlot::RANGED,
    };
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

void InventoryScreen::renderStatsPanel(game::Inventory& inventory, uint32_t playerLevel) {
    // Sum equipment stats
    int32_t totalArmor = 0;
    int32_t totalStr = 0, totalAgi = 0, totalSta = 0, totalInt = 0, totalSpi = 0;

    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (slot.empty()) continue;
        totalArmor += slot.item.armor;
        totalStr += slot.item.strength;
        totalAgi += slot.item.agility;
        totalSta += slot.item.stamina;
        totalInt += slot.item.intellect;
        totalSpi += slot.item.spirit;
    }

    // Base stats: 20 + level
    int32_t baseStat = 20 + static_cast<int32_t>(playerLevel);

    ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    ImVec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    ImVec4 gold(1.0f, 0.84f, 0.0f, 1.0f);
    ImVec4 gray(0.6f, 0.6f, 0.6f, 1.0f);

    // Armor (no base)
    if (totalArmor > 0) {
        ImGui::TextColored(gold, "Armor: %d", totalArmor);
    } else {
        ImGui::TextColored(gray, "Armor: 0");
    }

    // Helper to render a stat line
    auto renderStat = [&](const char* name, int32_t equipBonus) {
        int32_t total = baseStat + equipBonus;
        if (equipBonus > 0) {
            ImGui::TextColored(white, "%s: %d", name, total);
            ImGui::SameLine();
            ImGui::TextColored(green, "(+%d)", equipBonus);
        } else {
            ImGui::TextColored(gray, "%s: %d", name, total);
        }
    };

    renderStat("Strength", totalStr);
    renderStat("Agility", totalAgi);
    renderStat("Stamina", totalSta);
    renderStat("Intellect", totalInt);
    renderStat("Spirit", totalSpi);
}

void InventoryScreen::renderBackpackPanel(game::Inventory& inventory, bool collapseEmptySections) {
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Backpack");
    ImGui::Separator();

    constexpr float slotSize = 40.0f;
    constexpr int columns = 6;

    for (int i = 0; i < inventory.getBackpackSize(); i++) {
        if (i % columns != 0) ImGui::SameLine();

        const auto& slot = inventory.getBackpackSlot(i);
        char id[32];
        snprintf(id, sizeof(id), "##bp_%d", i);
        ImGui::PushID(id);
        renderItemSlot(inventory, slot, slotSize, nullptr,
                       SlotKind::BACKPACK, i, game::EquipSlot::NUM_SLOTS);
        ImGui::PopID();
    }

    // Show extra bags if equipped
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; bag++) {
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) continue;
        if (collapseEmptySections && !bagHasAnyItems(inventory, bag)) continue;

        ImGui::Spacing();
        ImGui::Separator();
        game::EquipSlot bagSlot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + bag);
        const auto& bagItem = inventory.getEquipSlot(bagSlot);
        std::string bagLabel = (!bagItem.empty() && !bagItem.item.name.empty())
            ? bagItem.item.name
            : ("Bag Slot " + std::to_string(bag + 1));
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.0f, 1.0f), "%s", bagLabel.c_str());

        for (int s = 0; s < bagSize; s++) {
            if (s % columns != 0) ImGui::SameLine();
            const auto& slot = inventory.getBagSlot(bag, s);
            char sid[32];
            snprintf(sid, sizeof(sid), "##bag%d_%d", bag, s);
            ImGui::PushID(sid);
            renderItemSlot(inventory, slot, slotSize, nullptr,
                           SlotKind::BACKPACK, -1, game::EquipSlot::NUM_SLOTS,
                           bag, s);
            ImGui::PopID();
        }
    }
}

void InventoryScreen::renderItemSlot(game::Inventory& inventory, const game::ItemSlot& slot,
                                      float size, const char* label,
                                      SlotKind kind, int backpackIndex,
                                      game::EquipSlot equipSlot,
                                      int bagIndex, int bagSlotIndex) {
    // Bag items are valid inventory slots even though backpackIndex is -1
    bool isBagSlot = (bagIndex >= 0 && bagSlotIndex >= 0);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 pos = ImGui::GetCursorScreenPos();

    bool isEmpty = slot.empty();

    // Determine if this is a valid drop target for held item
    bool validDrop = false;
    if (holdingItem) {
        if (kind == SlotKind::BACKPACK && (backpackIndex >= 0 || isBagSlot)) {
            validDrop = true;
        } else if (kind == SlotKind::EQUIPMENT && heldItem.inventoryType > 0) {
            game::EquipSlot validSlot = getEquipSlotForType(heldItem.inventoryType, inventory);
            validDrop = (equipSlot == validSlot);
            if (!validDrop && heldItem.inventoryType == 11)
                validDrop = (equipSlot == game::EquipSlot::RING1 || equipSlot == game::EquipSlot::RING2);
            if (!validDrop && heldItem.inventoryType == 12)
                validDrop = (equipSlot == game::EquipSlot::TRINKET1 || equipSlot == game::EquipSlot::TRINKET2);
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

        if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && holdingItem && validDrop) {
            if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                placeInBackpack(inventory, backpackIndex);
            } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                placeInBag(inventory, bagIndex, bagSlotIndex);
            } else if (kind == SlotKind::EQUIPMENT) {
                placeInEquipment(inventory, equipSlot);
            }
        }

        if (label && ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "%s", label);
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
        GLuint iconTex = getItemIcon(item.displayInfoId);
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

        ImGui::InvisibleButton("slot", ImVec2(size, size));

        // Left-click: pickup or place/swap
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            if (!holdingItem) {
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    pickupFromBackpack(inventory, backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    pickupFromBag(inventory, bagIndex, bagSlotIndex);
                } else if (kind == SlotKind::EQUIPMENT) {
                    pickupFromEquipment(inventory, equipSlot);
                }
            } else {
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    placeInBackpack(inventory, backpackIndex);
                } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                    placeInBag(inventory, bagIndex, bagSlotIndex);
                } else if (kind == SlotKind::EQUIPMENT && validDrop) {
                    placeInEquipment(inventory, equipSlot);
                }
            }
        }

        // Right-click: vendor sell (if vendor mode) or auto-equip/use
        if (ImGui::IsItemClicked(ImGuiMouseButton_Right) && !holdingItem && gameHandler_) {
            LOG_INFO("Right-click slot: kind=", (int)kind,
                     " backpackIndex=", backpackIndex,
                     " bagIndex=", bagIndex, " bagSlotIndex=", bagSlotIndex,
                     " vendorMode=", vendorMode_);
            if (vendorMode_ && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                gameHandler_->sellItemBySlot(backpackIndex);
            } else if (vendorMode_ && kind == SlotKind::BACKPACK && isBagSlot) {
                gameHandler_->sellItemInBag(bagIndex, bagSlotIndex);
            } else if (kind == SlotKind::EQUIPMENT) {
                LOG_INFO("UI unequip request: equipSlot=", (int)equipSlot);
                gameHandler_->unequipToBackpack(equipSlot);
            } else if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                if (item.inventoryType > 0) {
                    gameHandler_->autoEquipItemBySlot(backpackIndex);
                } else {
                    gameHandler_->useItemBySlot(backpackIndex);
                }
            } else if (kind == SlotKind::BACKPACK && isBagSlot) {
                if (item.inventoryType > 0) {
                    gameHandler_->autoEquipItemInBag(bagIndex, bagSlotIndex);
                } else {
                    gameHandler_->useItemInBag(bagIndex, bagSlotIndex);
                }
            }
        }

        if (ImGui::IsItemHovered() && !holdingItem) {
            renderItemTooltip(item, &inventory);
        }
    }
}

void InventoryScreen::renderItemTooltip(const game::ItemDef& item, const game::Inventory* inventory) {
    ImGui::BeginTooltip();

    ImVec4 qColor = getQualityColor(item.quality);
    ImGui::TextColored(qColor, "%s", item.name.c_str());

    if (item.itemId == 6948 && gameHandler_) {
        uint32_t mapId = 0;
        glm::vec3 pos;
        if (gameHandler_->getHomeBind(mapId, pos)) {
            const char* mapName = "Unknown";
            switch (mapId) {
                case 0:   mapName = "Eastern Kingdoms"; break;
                case 1:   mapName = "Kalimdor"; break;
                case 530:  mapName = "Outland"; break;
                case 571:  mapName = "Northrend"; break;
                case 13:   mapName = "Test"; break;
                case 169:  mapName = "Emerald Dream"; break;
            }
            ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Home: %s", mapName);
        } else {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Home: not set");
        }
        ImGui::TextDisabled("Use: Teleport home");
    }

    // Slot type
    if (item.inventoryType > 0) {
        const char* slotName = "";
        switch (item.inventoryType) {
            case 1:  slotName = "Head"; break;
            case 2:  slotName = "Neck"; break;
            case 3:  slotName = "Shoulder"; break;
            case 4:  slotName = "Shirt"; break;
            case 5:  slotName = "Chest"; break;
            case 6:  slotName = "Waist"; break;
            case 7:  slotName = "Legs"; break;
            case 8:  slotName = "Feet"; break;
            case 9:  slotName = "Wrist"; break;
            case 10: slotName = "Hands"; break;
            case 11: slotName = "Finger"; break;
            case 12: slotName = "Trinket"; break;
            case 13: slotName = "One-Hand"; break;
            case 14: slotName = "Shield"; break;
            case 15: slotName = "Ranged"; break;
            case 16: slotName = "Back"; break;
            case 17: slotName = "Two-Hand"; break;
            case 18: slotName = "Bag"; break;
            case 19: slotName = "Tabard"; break;
            case 20: slotName = "Robe"; break;
            case 21: slotName = "Main Hand"; break;
            case 22: slotName = "Off Hand"; break;
            case 23: slotName = "Held In Off-hand"; break;
            case 25: slotName = "Thrown"; break;
            case 26: slotName = "Ranged"; break;
            default: slotName = ""; break;
        }
        if (slotName[0]) {
            if (!item.subclassName.empty()) {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s  %s", slotName, item.subclassName.c_str());
            } else {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", slotName);
            }
        }
    }

    if (item.damageMax > 0.0f) {
        ImGui::Text("%.0f - %.0f Damage", item.damageMin, item.damageMax);
        if (item.delayMs > 0) {
            float speed = static_cast<float>(item.delayMs) / 1000.0f;
            float dps = ((item.damageMin + item.damageMax) * 0.5f) / speed;
            ImGui::Text("Speed %.2f", speed);
            ImGui::Text("%.1f damage per second", dps);
        }
    }

    // Armor
    if (item.armor > 0) {
        ImGui::Text("%d Armor", item.armor);
    }

    // Stats with "Equip:" prefix style
    ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    ImVec4 red(1.0f, 0.2f, 0.2f, 1.0f);

    auto renderStat = [&](int32_t val, const char* name) {
        if (val > 0) {
            ImGui::TextColored(green, "+%d %s", val, name);
        } else if (val < 0) {
            ImGui::TextColored(red, "%d %s", val, name);
        }
    };

    renderStat(item.stamina, "Stamina");
    renderStat(item.strength, "Strength");
    renderStat(item.agility, "Agility");
    renderStat(item.intellect, "Intellect");
    renderStat(item.spirit, "Spirit");

    // Stack info
    if (item.maxStack > 1) {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Stack: %u/%u", item.stackCount, item.maxStack);
    }

    // Sell price
    if (item.sellPrice > 0) {
        uint32_t g = item.sellPrice / 10000;
        uint32_t s = (item.sellPrice / 100) % 100;
        uint32_t c = item.sellPrice % 100;
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Sell Price: %ug %us %uc", g, s, c);
    }

    // Shift-hover comparison with currently equipped equivalent.
    if (inventory && ImGui::GetIO().KeyShift && item.inventoryType > 0) {
        if (const game::ItemSlot* eq = findComparableEquipped(*inventory, item.inventoryType)) {
            ImGui::Separator();
            ImGui::TextDisabled("Equipped:");
            GLuint eqIcon = getItemIcon(eq->item.displayInfoId);
            if (eqIcon) {
                ImGui::Image((ImTextureID)(uintptr_t)eqIcon, ImVec2(18.0f, 18.0f));
                ImGui::SameLine();
            }
            ImGui::TextColored(getQualityColor(eq->item.quality), "%s", eq->item.name.c_str());

            if (eq->item.damageMax > 0.0f) {
                ImGui::Text("%.0f - %.0f Damage", eq->item.damageMin, eq->item.damageMax);
            }
            if (eq->item.armor > 0) ImGui::Text("%d Armor", eq->item.armor);
            renderStat(eq->item.stamina, "Stamina");
            renderStat(eq->item.strength, "Strength");
            renderStat(eq->item.agility, "Agility");
            renderStat(eq->item.intellect, "Intellect");
            renderStat(eq->item.spirit, "Spirit");
        }
    }

    ImGui::EndTooltip();
}

} // namespace ui
} // namespace wowee
