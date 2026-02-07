#include "ui/inventory_screen.hpp"
#include "game/game_handler.hpp"
#include "core/application.hpp"
#include "core/input.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/character_renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <SDL2/SDL.h>
#include <cstdio>
#include <unordered_set>

namespace wowee {
namespace ui {

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
    std::string iconName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), 5);
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
    for (uint16_t i = 0; i <= 18; i++) geosets.insert(i);

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
            uint32_t fieldIdx = 15 + region;
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
    if (gameHandler_ && heldSource == HeldSource::EQUIPMENT) {
        // Online mode: avoid client-side unequip; wait for server update.
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

void InventoryScreen::placeInEquipment(game::Inventory& inv, game::EquipSlot slot) {
    if (!holdingItem) return;
    if (gameHandler_) {
        if (heldSource == HeldSource::BACKPACK && heldBackpackIndex >= 0) {
            // Online mode: request server auto-equip and keep local state intact.
            gameHandler_->autoEquipItemBySlot(heldBackpackIndex);
            cancelPickup(inv);
            return;
        }
        if (heldSource == HeldSource::EQUIPMENT) {
            // Online mode: avoid client-side equipment swaps.
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

void InventoryScreen::render(game::Inventory& inventory, uint64_t moneyCopper) {
    // B key toggle (edge-triggered)
    bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard;
    bool bDown = !uiWantsKeyboard && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_B);
    if (bDown && !bKeyWasDown) {
        open = !open;
    }
    bKeyWasDown = bDown;

    // C key toggle for character screen (edge-triggered)
    bool cDown = !uiWantsKeyboard && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_C);
    if (cDown && !cKeyWasDown) {
        characterOpen = !characterOpen;
    }
    cKeyWasDown = cDown;

    if (!open) {
        if (holdingItem) cancelPickup(inventory);
        return;
    }

    // Escape cancels held item
    if (holdingItem && !uiWantsKeyboard && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_ESCAPE)) {
        cancelPickup(inventory);
    }

    // Right-click anywhere while holding = cancel
    if (holdingItem && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        cancelPickup(inventory);
    }

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    // Calculate bag window size
    constexpr float slotSize = 40.0f;
    constexpr int columns = 4;
    int rows = (inventory.getBackpackSize() + columns - 1) / columns;
    float bagContentH = rows * (slotSize + 4.0f) + 40.0f; // slots + header + money

    // Check for extra bags and add space
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; bag++) {
        int bagSize = inventory.getBagSize(bag);
        if (bagSize <= 0) continue;
        int bagRows = (bagSize + columns - 1) / columns;
        bagContentH += bagRows * (slotSize + 4.0f) + 30.0f; // slots + header
    }

    float windowW = columns * (slotSize + 4.0f) + 30.0f;
    float windowH = bagContentH + 50.0f; // padding

    // Position at bottom-right of screen
    float posX = screenW - windowW - 10.0f;
    float posY = screenH - windowH - 60.0f; // above action bar area

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(windowW, windowH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    if (!ImGui::Begin("Bags", &open, flags)) {
        ImGui::End();
        return;
    }

    renderBackpackPanel(inventory);

    // Money display
    ImGui::Spacing();
    uint64_t gold = moneyCopper / 10000;
    uint64_t silver = (moneyCopper / 100) % 100;
    uint64_t copper = moneyCopper % 100;
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "%llug %llus %lluc",
                       static_cast<unsigned long long>(gold),
                       static_cast<unsigned long long>(silver),
                       static_cast<unsigned long long>(copper));
    ImGui::End();

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
            uint32_t level = gameHandler.getPlayerLevel();
            uint32_t cap = (level > 0) ? (level * 5) : 0;
            ImGui::TextDisabled("Skills (online sync pending)");
            ImGui::Spacing();
            if (ImGui::BeginTable("SkillsTable", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
                ImGui::TableSetupColumn("Skill", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableHeadersRow();

                const char* skills[] = {
                    "Unarmed", "Swords", "Axes", "Maces", "Daggers",
                    "Staves", "Polearms", "Bows", "Guns", "Crossbows"
                };
                for (const char* skill : skills) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", skill);
                    ImGui::TableSetColumnIndex(1);
                    if (cap > 0) {
                        ImGui::Text("-- / %u", cap);
                    } else {
                        ImGui::TextDisabled("--");
                    }
                }

                ImGui::EndTable();
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

void InventoryScreen::renderBackpackPanel(game::Inventory& inventory) {
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Backpack");
    ImGui::Separator();

    constexpr float slotSize = 40.0f;
    constexpr int columns = 4;

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

        ImGui::Spacing();
        ImGui::Separator();
        char bagLabel[32];
        snprintf(bagLabel, sizeof(bagLabel), "Bag %d", bag + 1);
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.0f, 1.0f), "%s", bagLabel);

        for (int s = 0; s < bagSize; s++) {
            if (s % columns != 0) ImGui::SameLine();
            const auto& slot = inventory.getBagSlot(bag, s);
            char sid[32];
            snprintf(sid, sizeof(sid), "##bag%d_%d", bag, s);
            ImGui::PushID(sid);
            renderItemSlot(inventory, slot, slotSize, nullptr,
                           SlotKind::BACKPACK, -1, game::EquipSlot::NUM_SLOTS);
            ImGui::PopID();
        }
    }
}

void InventoryScreen::renderItemSlot(game::Inventory& inventory, const game::ItemSlot& slot,
                                      float size, const char* label,
                                      SlotKind kind, int backpackIndex,
                                      game::EquipSlot equipSlot) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 pos = ImGui::GetCursorScreenPos();

    bool isEmpty = slot.empty();

    // Determine if this is a valid drop target for held item
    bool validDrop = false;
    if (holdingItem) {
        if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
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
                } else if (kind == SlotKind::EQUIPMENT) {
                    pickupFromEquipment(inventory, equipSlot);
                }
            } else {
                if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                    placeInBackpack(inventory, backpackIndex);
                } else if (kind == SlotKind::EQUIPMENT && validDrop) {
                    placeInEquipment(inventory, equipSlot);
                }
            }
        }

        // Right-click: vendor sell (if vendor mode) or auto-equip/unequip
        if (ImGui::IsItemClicked(ImGuiMouseButton_Right) && !holdingItem) {
            LOG_DEBUG("Right-click slot: kind=", (int)kind,
                      " backpackIndex=", backpackIndex,
                      " vendorMode=", vendorMode_,
                      " hasHandler=", (gameHandler_ != nullptr));
            if (vendorMode_ && gameHandler_ && kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                // Sell to vendor
                gameHandler_->sellItemBySlot(backpackIndex);
            } else if (kind == SlotKind::EQUIPMENT) {
                // Unequip: move to free backpack slot
                int freeSlot = inventory.findFreeBackpackSlot();
                if (freeSlot >= 0) {
                    inventory.setBackpackSlot(freeSlot, item);
                    inventory.clearEquipSlot(equipSlot);
                    equipmentDirty = true;
                    inventoryDirty = true;
                }
            } else if (kind == SlotKind::BACKPACK && backpackIndex >= 0) {
                if (gameHandler_) {
                    if (item.inventoryType > 0) {
                        // Auto-equip (online)
                        gameHandler_->autoEquipItemBySlot(backpackIndex);
                    } else {
                        // Use consumable (online)
                        gameHandler_->useItemBySlot(backpackIndex);
                    }
                }
            }
        }

        if (ImGui::IsItemHovered() && !holdingItem) {
            renderItemTooltip(item);
        }
    }
}

void InventoryScreen::renderItemTooltip(const game::ItemDef& item) {
    ImGui::BeginTooltip();

    ImVec4 qColor = getQualityColor(item.quality);
    ImGui::TextColored(qColor, "%s", item.name.c_str());

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

    ImGui::EndTooltip();
}

} // namespace ui
} // namespace wowee
