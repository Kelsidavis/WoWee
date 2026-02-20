#include "game/inventory.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace game {

static const ItemSlot EMPTY_SLOT{};

Inventory::Inventory() = default;

const ItemSlot& Inventory::getBackpackSlot(int index) const {
    if (index < 0 || index >= BACKPACK_SLOTS) return EMPTY_SLOT;
    return backpack[index];
}

bool Inventory::setBackpackSlot(int index, const ItemDef& item) {
    if (index < 0 || index >= BACKPACK_SLOTS) return false;
    backpack[index].item = item;
    return true;
}

bool Inventory::clearBackpackSlot(int index) {
    if (index < 0 || index >= BACKPACK_SLOTS) return false;
    backpack[index].item = ItemDef{};
    return true;
}

const ItemSlot& Inventory::getEquipSlot(EquipSlot slot) const {
    int idx = static_cast<int>(slot);
    if (idx < 0 || idx >= NUM_EQUIP_SLOTS) return EMPTY_SLOT;
    return equipment[idx];
}

bool Inventory::setEquipSlot(EquipSlot slot, const ItemDef& item) {
    int idx = static_cast<int>(slot);
    if (idx < 0 || idx >= NUM_EQUIP_SLOTS) return false;
    equipment[idx].item = item;
    return true;
}

bool Inventory::clearEquipSlot(EquipSlot slot) {
    int idx = static_cast<int>(slot);
    if (idx < 0 || idx >= NUM_EQUIP_SLOTS) return false;
    equipment[idx].item = ItemDef{};
    return true;
}

int Inventory::getBagSize(int bagIndex) const {
    if (bagIndex < 0 || bagIndex >= NUM_BAG_SLOTS) return 0;
    return bags[bagIndex].size;
}

void Inventory::setBagSize(int bagIndex, int size) {
    if (bagIndex < 0 || bagIndex >= NUM_BAG_SLOTS) return;
    bags[bagIndex].size = std::min(size, MAX_BAG_SIZE);
}

const ItemSlot& Inventory::getBagSlot(int bagIndex, int slotIndex) const {
    if (bagIndex < 0 || bagIndex >= NUM_BAG_SLOTS) return EMPTY_SLOT;
    if (slotIndex < 0 || slotIndex >= bags[bagIndex].size) return EMPTY_SLOT;
    return bags[bagIndex].slots[slotIndex];
}

bool Inventory::setBagSlot(int bagIndex, int slotIndex, const ItemDef& item) {
    if (bagIndex < 0 || bagIndex >= NUM_BAG_SLOTS) return false;
    if (slotIndex < 0 || slotIndex >= bags[bagIndex].size) return false;
    bags[bagIndex].slots[slotIndex].item = item;
    return true;
}

bool Inventory::clearBagSlot(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= NUM_BAG_SLOTS) return false;
    if (slotIndex < 0 || slotIndex >= bags[bagIndex].size) return false;
    bags[bagIndex].slots[slotIndex].item = ItemDef{};
    return true;
}

const ItemSlot& Inventory::getBankSlot(int index) const {
    if (index < 0 || index >= BANK_SLOTS) return EMPTY_SLOT;
    return bankSlots_[index];
}

bool Inventory::setBankSlot(int index, const ItemDef& item) {
    if (index < 0 || index >= BANK_SLOTS) return false;
    bankSlots_[index].item = item;
    return true;
}

bool Inventory::clearBankSlot(int index) {
    if (index < 0 || index >= BANK_SLOTS) return false;
    bankSlots_[index].item = ItemDef{};
    return true;
}

const ItemSlot& Inventory::getBankBagSlot(int bagIndex, int slotIndex) const {
    if (bagIndex < 0 || bagIndex >= BANK_BAG_SLOTS) return EMPTY_SLOT;
    if (slotIndex < 0 || slotIndex >= bankBags_[bagIndex].size) return EMPTY_SLOT;
    return bankBags_[bagIndex].slots[slotIndex];
}

bool Inventory::setBankBagSlot(int bagIndex, int slotIndex, const ItemDef& item) {
    if (bagIndex < 0 || bagIndex >= BANK_BAG_SLOTS) return false;
    if (slotIndex < 0 || slotIndex >= bankBags_[bagIndex].size) return false;
    bankBags_[bagIndex].slots[slotIndex].item = item;
    return true;
}

int Inventory::getBankBagSize(int bagIndex) const {
    if (bagIndex < 0 || bagIndex >= BANK_BAG_SLOTS) return 0;
    return bankBags_[bagIndex].size;
}

void Inventory::setBankBagSize(int bagIndex, int size) {
    if (bagIndex < 0 || bagIndex >= BANK_BAG_SLOTS) return;
    bankBags_[bagIndex].size = std::min(size, MAX_BAG_SIZE);
}

void Inventory::swapBagContents(int bagA, int bagB) {
    if (bagA < 0 || bagA >= NUM_BAG_SLOTS || bagB < 0 || bagB >= NUM_BAG_SLOTS) return;
    if (bagA == bagB) return;
    std::swap(bags[bagA], bags[bagB]);
}

int Inventory::findFreeBackpackSlot() const {
    for (int i = 0; i < BACKPACK_SLOTS; i++) {
        if (backpack[i].empty()) return i;
    }
    return -1;
}

bool Inventory::addItem(const ItemDef& item) {
    // Try stacking first
    if (item.maxStack > 1) {
        for (int i = 0; i < BACKPACK_SLOTS; i++) {
            if (!backpack[i].empty() &&
                backpack[i].item.itemId == item.itemId &&
                backpack[i].item.stackCount < backpack[i].item.maxStack) {
                uint32_t space = backpack[i].item.maxStack - backpack[i].item.stackCount;
                uint32_t toAdd = std::min(space, item.stackCount);
                backpack[i].item.stackCount += toAdd;
                if (toAdd >= item.stackCount) return true;
                // Remaining needs a new slot — fall through
            }
        }
    }

    int slot = findFreeBackpackSlot();
    if (slot < 0) return false;
    backpack[slot].item = item;
    return true;
}

void Inventory::populateTestItems() {
    // Equipment
    {
        ItemDef sword;
        sword.itemId = 25;
        sword.name = "Worn Shortsword";
        sword.quality = ItemQuality::COMMON;
        sword.inventoryType = 21; // Main Hand
        sword.strength = 1;
        sword.displayInfoId = 1542;    // Sword_1H_Short_A_02.m2
        sword.subclassName = "Sword";
        setEquipSlot(EquipSlot::MAIN_HAND, sword);
    }
    {
        ItemDef shield;
        shield.itemId = 2129;
        shield.name = "Large Round Shield";
        shield.quality = ItemQuality::COMMON;
        shield.inventoryType = 14; // Off Hand (Shield)
        shield.armor = 18;
        shield.stamina = 1;
        shield.displayInfoId = 18662;  // Shield_Round_A_01.m2
        shield.subclassName = "Shield";
        setEquipSlot(EquipSlot::OFF_HAND, shield);
    }
    // Shirt/pants/boots in backpack (character model renders bare geosets)
    {
        ItemDef shirt;
        shirt.itemId = 38;
        shirt.name = "Recruit's Shirt";
        shirt.quality = ItemQuality::COMMON;
        shirt.inventoryType = 4; // Shirt
        shirt.displayInfoId = 2163;
        addItem(shirt);
    }
    {
        ItemDef pants;
        pants.itemId = 39;
        pants.name = "Recruit's Pants";
        pants.quality = ItemQuality::COMMON;
        pants.inventoryType = 7; // Legs
        pants.armor = 4;
        pants.displayInfoId = 1883;
        addItem(pants);
    }
    {
        ItemDef boots;
        boots.itemId = 40;
        boots.name = "Recruit's Boots";
        boots.quality = ItemQuality::COMMON;
        boots.inventoryType = 8; // Feet
        boots.armor = 3;
        boots.displayInfoId = 2166;
        addItem(boots);
    }

    // Backpack items
    {
        ItemDef potion;
        potion.itemId = 118;
        potion.name = "Minor Healing Potion";
        potion.quality = ItemQuality::COMMON;
        potion.stackCount = 3;
        potion.maxStack = 5;
        addItem(potion);
    }
    {
        ItemDef hearthstone;
        hearthstone.itemId = 6948;
        hearthstone.name = "Hearthstone";
        hearthstone.quality = ItemQuality::COMMON;
        addItem(hearthstone);
    }
    {
        ItemDef leather;
        leather.itemId = 2318;
        leather.name = "Light Leather";
        leather.quality = ItemQuality::COMMON;
        leather.stackCount = 5;
        leather.maxStack = 20;
        addItem(leather);
    }
    {
        ItemDef cloth;
        cloth.itemId = 2589;
        cloth.name = "Linen Cloth";
        cloth.quality = ItemQuality::COMMON;
        cloth.stackCount = 8;
        cloth.maxStack = 20;
        addItem(cloth);
    }
    {
        ItemDef quest;
        quest.itemId = 50000;
        quest.name = "Kobold Candle";
        quest.quality = ItemQuality::COMMON;
        quest.stackCount = 4;
        quest.maxStack = 10;
        addItem(quest);
    }
    {
        ItemDef ring;
        ring.itemId = 11287;
        ring.name = "Verdant Ring";
        ring.quality = ItemQuality::UNCOMMON;
        ring.inventoryType = 11; // Ring
        ring.stamina = 3;
        ring.spirit = 2;
        addItem(ring);
    }
    {
        ItemDef cloak;
        cloak.itemId = 2570;
        cloak.name = "Linen Cloak";
        cloak.quality = ItemQuality::UNCOMMON;
        cloak.inventoryType = 16; // Back
        cloak.armor = 10;
        cloak.agility = 1;
        cloak.displayInfoId = 15055;
        addItem(cloak);
    }
    {
        ItemDef rareAxe;
        rareAxe.itemId = 15268;
        rareAxe.name = "Stoneslayer";
        rareAxe.quality = ItemQuality::RARE;
        rareAxe.inventoryType = 17; // Two-Hand
        rareAxe.strength = 8;
        rareAxe.stamina = 7;
        rareAxe.subclassName = "Axe";
        rareAxe.displayInfoId = 782;    // Axe_2H_Battle_B_01.m2
        addItem(rareAxe);
    }

    LOG_INFO("Inventory: populated test items (2 equipped, 11 backpack)");
}

const char* getQualityName(ItemQuality quality) {
    switch (quality) {
        case ItemQuality::POOR:      return "Poor";
        case ItemQuality::COMMON:    return "Common";
        case ItemQuality::UNCOMMON:  return "Uncommon";
        case ItemQuality::RARE:      return "Rare";
        case ItemQuality::EPIC:      return "Epic";
        case ItemQuality::LEGENDARY: return "Legendary";
        default:                     return "Unknown";
    }
}

const char* getEquipSlotName(EquipSlot slot) {
    switch (slot) {
        case EquipSlot::HEAD:       return "Head";
        case EquipSlot::NECK:       return "Neck";
        case EquipSlot::SHOULDERS:  return "Shoulders";
        case EquipSlot::SHIRT:      return "Shirt";
        case EquipSlot::CHEST:      return "Chest";
        case EquipSlot::WAIST:      return "Waist";
        case EquipSlot::LEGS:       return "Legs";
        case EquipSlot::FEET:       return "Feet";
        case EquipSlot::WRISTS:     return "Wrists";
        case EquipSlot::HANDS:      return "Hands";
        case EquipSlot::RING1:      return "Ring 1";
        case EquipSlot::RING2:      return "Ring 2";
        case EquipSlot::TRINKET1:   return "Trinket 1";
        case EquipSlot::TRINKET2:   return "Trinket 2";
        case EquipSlot::BACK:       return "Back";
        case EquipSlot::MAIN_HAND:  return "Main Hand";
        case EquipSlot::OFF_HAND:   return "Off Hand";
        case EquipSlot::RANGED:     return "Ranged";
        case EquipSlot::TABARD:     return "Tabard";
        case EquipSlot::BAG1:       return "Bag 1";
        case EquipSlot::BAG2:       return "Bag 2";
        case EquipSlot::BAG3:       return "Bag 3";
        case EquipSlot::BAG4:       return "Bag 4";
        default:                    return "Unknown";
    }
}

} // namespace game
} // namespace wowee
