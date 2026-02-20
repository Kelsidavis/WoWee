#pragma once

#include <cstdint>
#include <string>
#include <array>

namespace wowee {
namespace game {

enum class ItemQuality : uint8_t {
    POOR = 0,       // Grey
    COMMON = 1,     // White
    UNCOMMON = 2,   // Green
    RARE = 3,       // Blue
    EPIC = 4,       // Purple
    LEGENDARY = 5,  // Orange
};

enum class EquipSlot : uint8_t {
    HEAD = 0, NECK, SHOULDERS, SHIRT, CHEST,
    WAIST, LEGS, FEET, WRISTS, HANDS,
    RING1, RING2, TRINKET1, TRINKET2,
    BACK, MAIN_HAND, OFF_HAND, RANGED, TABARD,
    BAG1, BAG2, BAG3, BAG4,
    NUM_SLOTS  // = 23
};

struct ItemDef {
    uint32_t itemId = 0;
    std::string name;
    std::string subclassName;  // "Sword", "Mace", "Shield", etc.
    ItemQuality quality = ItemQuality::COMMON;
    uint8_t inventoryType = 0;
    uint32_t stackCount = 1;
    uint32_t maxStack = 1;
    uint32_t bagSlots = 0;
    float damageMin = 0.0f;
    float damageMax = 0.0f;
    uint32_t delayMs = 0;
    // Stats
    int32_t armor = 0;
    int32_t stamina = 0;
    int32_t strength = 0;
    int32_t agility = 0;
    int32_t intellect = 0;
    int32_t spirit = 0;
    uint32_t displayInfoId = 0;
    uint32_t sellPrice = 0;
};

struct ItemSlot {
    ItemDef item;
    bool empty() const { return item.itemId == 0; }
};

class Inventory {
public:
    static constexpr int BACKPACK_SLOTS = 16;
    static constexpr int NUM_EQUIP_SLOTS = 23;
    static constexpr int NUM_BAG_SLOTS = 4;
    static constexpr int MAX_BAG_SIZE = 36;
    static constexpr int BANK_SLOTS = 28;
    static constexpr int BANK_BAG_SLOTS = 7;

    Inventory();

    // Backpack
    const ItemSlot& getBackpackSlot(int index) const;
    bool setBackpackSlot(int index, const ItemDef& item);
    bool clearBackpackSlot(int index);
    int getBackpackSize() const { return BACKPACK_SLOTS; }

    // Equipment
    const ItemSlot& getEquipSlot(EquipSlot slot) const;
    bool setEquipSlot(EquipSlot slot, const ItemDef& item);
    bool clearEquipSlot(EquipSlot slot);

    // Extra bags
    int getBagSize(int bagIndex) const;
    void setBagSize(int bagIndex, int size);
    const ItemSlot& getBagSlot(int bagIndex, int slotIndex) const;
    bool setBagSlot(int bagIndex, int slotIndex, const ItemDef& item);
    bool clearBagSlot(int bagIndex, int slotIndex);

    // Bank slots (28 main + 7 bank bags)
    const ItemSlot& getBankSlot(int index) const;
    bool setBankSlot(int index, const ItemDef& item);
    bool clearBankSlot(int index);

    const ItemSlot& getBankBagSlot(int bagIndex, int slotIndex) const;
    bool setBankBagSlot(int bagIndex, int slotIndex, const ItemDef& item);
    int getBankBagSize(int bagIndex) const;
    void setBankBagSize(int bagIndex, int size);

    uint8_t getPurchasedBankBagSlots() const { return purchasedBankBagSlots_; }
    void setPurchasedBankBagSlots(uint8_t count) { purchasedBankBagSlots_ = count; }

    // Swap two bag slots (equip items + contents)
    void swapBagContents(int bagA, int bagB);

    // Utility
    int findFreeBackpackSlot() const;
    bool addItem(const ItemDef& item);

    // Test data
    void populateTestItems();

private:
    std::array<ItemSlot, BACKPACK_SLOTS> backpack{};
    std::array<ItemSlot, NUM_EQUIP_SLOTS> equipment{};

    struct BagData {
        int size = 0;
        std::array<ItemSlot, MAX_BAG_SIZE> slots{};
    };
    std::array<BagData, NUM_BAG_SLOTS> bags{};

    // Bank
    std::array<ItemSlot, BANK_SLOTS> bankSlots_{};
    std::array<BagData, BANK_BAG_SLOTS> bankBags_{};
    uint8_t purchasedBankBagSlots_ = 0;
};

const char* getQualityName(ItemQuality quality);
const char* getEquipSlotName(EquipSlot slot);

} // namespace game
} // namespace wowee
