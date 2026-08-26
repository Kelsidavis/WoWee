#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <vector>

namespace wowee {
namespace game {

enum class ItemQuality : uint8_t {
    POOR = 0,       // Grey
    COMMON = 1,     // White
    UNCOMMON = 2,   // Green
    RARE = 3,       // Blue
    EPIC = 4,       // Purple
    LEGENDARY = 5,  // Orange
    ARTIFACT = 6,   // Yellow (unused in 3.3.5a but valid quality value)
    HEIRLOOM = 7,   // Yellow/gold (WotLK bind-on-account heirlooms)
};

enum class EquipSlot : uint8_t {
    HEAD = 0, NECK, SHOULDERS, SHIRT, CHEST,
    WAIST, LEGS, FEET, WRISTS, HANDS,
    RING1, RING2, TRINKET1, TRINKET2,
    BACK, MAIN_HAND, OFF_HAND, RANGED, TABARD,
    BAG1, BAG2, BAG3, BAG4,
    NUM_SLOTS  // = 23
};

// WoW InventoryType field values (from ItemDisplayInfo / Item.dbc / CMSG_ITEM_QUERY)
// Used in ItemDef::inventoryType and equipment update packets.
namespace InvType {
    constexpr uint8_t NON_EQUIP     = 0;   // Not equippable / unarmed
    constexpr uint8_t HEAD          = 1;
    constexpr uint8_t NECK          = 2;
    constexpr uint8_t SHOULDERS     = 3;
    constexpr uint8_t SHIRT         = 4;
    constexpr uint8_t CHEST         = 5;   // Chest armor
    constexpr uint8_t WAIST         = 6;
    constexpr uint8_t LEGS          = 7;
    constexpr uint8_t FEET          = 8;
    constexpr uint8_t WRISTS        = 9;
    constexpr uint8_t HANDS         = 10;
    constexpr uint8_t FINGER        = 11;  // Ring
    constexpr uint8_t TRINKET       = 12;
    constexpr uint8_t ONE_HAND      = 13;  // One-handed weapon (sword, mace, dagger, fist)
    constexpr uint8_t SHIELD        = 14;
    constexpr uint8_t RANGED_BOW    = 15;  // Bow
    constexpr uint8_t BACK          = 16;  // Cloak
    constexpr uint8_t TWO_HAND      = 17;  // Two-handed weapon (also polearm/staff by inventoryType alone)
    constexpr uint8_t BAG           = 18;
    constexpr uint8_t TABARD        = 19;
    constexpr uint8_t ROBE          = 20;  // Chest (robe variant)
    constexpr uint8_t MAIN_HAND     = 21;  // Main-hand only weapon
    constexpr uint8_t OFF_HAND      = 22;  // Off-hand weapon (INVTYPE_WEAPONOFFHAND)
    constexpr uint8_t HOLDABLE      = 23;  // Off-hand holdable (books, orbs)
    constexpr uint8_t AMMO          = 24;
    constexpr uint8_t THROWN        = 25;
    constexpr uint8_t RANGED_GUN    = 26;  // Gun / Crossbow / Wand
} // namespace InvType

/// Whether an item of this INVTYPE is a weapon, for the purpose of comparing
/// two of them: damage per second is worth showing side by side, armour is not.
///
/// Held-in-off-hand and shields are deliberately not weapons here - they occupy
/// a weapon slot and have no damage to compare.
///
/// Written out twice before this, in the bags and in the chat tooltip, as a
/// switch over the bare numbers rather than the names two dozen lines above.
inline bool isWeaponInventoryType(uint8_t inventoryType) {
    switch (inventoryType) {
        case InvType::ONE_HAND:
        case InvType::RANGED_BOW:
        case InvType::TWO_HAND:
        case InvType::MAIN_HAND:
        case InvType::THROWN:
        case InvType::RANGED_GUN:
            return true;
        default:
            return false;
    }
}

/// Whether an item of this INVTYPE swings from the off hand.
///
/// Two INVTYPEs go in the off-hand slot and only these two are weapons: 13,
/// which is any one-hander and may be equipped in either hand, and 22, which
/// is the off-hand-only weapon. 23 is the held-in-off-hand book or orb and 14
/// is a shield; neither swings.
///
/// 22 was read as the book for a while - the name is close and the comment on
/// it said so - and the cost was dual wielding. Both places that ask this
/// tested 13 alone, so an off-hand-only weapon left the character with no
/// off-hand attack animation and, until the main hand was drawn, no reason to
/// unsheathe.
inline bool isOffHandWeaponInventoryType(uint8_t inventoryType) {
    return inventoryType == InvType::ONE_HAND || inventoryType == InvType::OFF_HAND;
}

/// The equipped slots an item of this INVTYPE should be compared against, in
/// the order to try them.
///
/// Usually one. Two where the item could go in either of a pair - rings,
/// trinkets, and a one-hander that may be in the off hand - and every bag slot
/// for a bag. Empty for anything that is not equipped at all.
///
/// This mapping is WoW's, not this client's, and it was written out twice: once
/// for the bags' comparison tooltip and once for the chat link's. They agreed,
/// which is luck rather than design - the stat-name table beside them in the
/// same two files did not, and was shifted by one from 34 up for long enough
/// that a resilience item read as haste.
inline std::vector<EquipSlot> comparableEquipSlots(uint8_t inventoryType) {
    using ES = EquipSlot;
    switch (inventoryType) {
        case InvType::HEAD:      return {ES::HEAD};
        case InvType::NECK:      return {ES::NECK};
        case InvType::SHOULDERS: return {ES::SHOULDERS};
        case InvType::SHIRT:     return {ES::SHIRT};
        case InvType::CHEST:
        case InvType::ROBE:      return {ES::CHEST};
        case InvType::WAIST:     return {ES::WAIST};
        case InvType::LEGS:      return {ES::LEGS};
        case InvType::FEET:      return {ES::FEET};
        case InvType::WRISTS:    return {ES::WRISTS};
        case InvType::HANDS:     return {ES::HANDS};
        case InvType::FINGER:    return {ES::RING1, ES::RING2};
        case InvType::TRINKET:   return {ES::TRINKET1, ES::TRINKET2};
        // A one-hander can be wielded in either hand, so the main hand is tried
        // first and the off hand second.
        case InvType::ONE_HAND:  return {ES::MAIN_HAND, ES::OFF_HAND};
        case InvType::SHIELD:
        case InvType::OFF_HAND:
        case InvType::HOLDABLE:  return {ES::OFF_HAND};
        case InvType::RANGED_BOW:
        case InvType::THROWN:
        case InvType::RANGED_GUN: return {ES::RANGED};
        case InvType::BACK:      return {ES::BACK};
        case InvType::TWO_HAND:
        case InvType::MAIN_HAND: return {ES::MAIN_HAND};
        case InvType::BAG: {
            // Counted off the enum rather than against Inventory::NUM_BAG_SLOTS,
            // which is declared further down this file - and which would be a
            // second place to say how many bags there are.
            std::vector<EquipSlot> bags;
            for (int s = static_cast<int>(ES::BAG1); s <= static_cast<int>(ES::BAG4); ++s) {
                bags.push_back(static_cast<ES>(s));
            }
            return bags;
        }
        case InvType::TABARD:    return {ES::TABARD};
        default:                 return {};
    }
}

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
    uint32_t curDurability = 0;
    uint32_t maxDurability = 0;
    uint32_t itemLevel = 0;
    uint32_t requiredLevel = 0;
    uint32_t bindType = 0;      // 0=none, 1=BoP, 2=BoE, 3=BoU, 4=BoQ
    // Per-instance bound state from ITEM_FIELD_FLAGS bit 0x1 (ITEM_FLAG_SOULBOUND).
    // A BoE item is bindType==2 but only prompts on equip while this is false; once the
    // server sets the soulbound bit (on equip), it is already bound and must not prompt.
    bool soulbound = false;

    /// Would equipping this bind it to the player?
    ///
    /// Bind-on-equip and not yet bound. Once the server sets the soulbound
    /// bit, moving the same piece between slots must not ask again.
    ///
    /// Here rather than on a handler because it is a fact about the item, and
    /// because it had been written out three times in inventory_screen.cpp and
    /// nowhere on the path FrameXML takes - so equipping through the interface
    /// bound the item with no prompt at all.
    [[nodiscard]] bool wouldBindOnEquip() const { return bindType == 2 && !soulbound; }
    // Per-instance ITEM_FIELD_RANDOM_PROPERTIES_ID: >0 → ItemRandomProperties.dbc (prefix),
    // <0 → ItemRandomSuffix.dbc (e.g. "of the Bear"). 0 means no random property rolled.
    int32_t randomPropertyId = 0;
    // ITEM_FIELD_PROPERTY_SEED, which scales what a suffix rolled. Kept beside
    // the id because the two only mean anything together, and because a
    // tooltip built from the item template has neither - the stats below are
    // the base item's, with the roll already folded in, so anything wanting to
    // name the roll on its own has to ask for it again.
    uint32_t suffixFactor = 0;
    std::string description;    // Flavor/lore text shown in tooltip (italic yellow)
    uint32_t pageTextId = 0;     // Non-zero: item opens readable page text
    // Generic stat pairs for non-primary stats (hit, crit, haste, AP, SP, etc.)
    struct ExtraStat { uint32_t statType = 0; int32_t statValue = 0; };
    std::vector<ExtraStat> extraStats;
    uint32_t startQuestId = 0;  // Non-zero: item begins a quest
    // Exact server object identity for this displayed inventory slot. Item IDs
    // are not unique and must never be used to guess destructive operations.
    uint64_t guid = 0;
};

struct ItemSlot {
    ItemDef item;
    [[nodiscard]] bool empty() const { return item.itemId == 0; }
};

class Inventory {
public:
    static constexpr int BACKPACK_SLOTS = 16;
    static constexpr int KEYRING_SLOTS = 32;
    // WoW slot layout: 0-22 are equipment (head, neck, ... tabard, mainhand, offhand, ranged, ammo).
    // Backpack inventory starts at slot 23 in bag 0xFF, so packet slot = NUM_EQUIP_SLOTS + backpackIndex.
    static constexpr int NUM_EQUIP_SLOTS = 23;
    // Bag containers occupy equipment slots 19-22 (bag1, bag2, bag3, bag4).
    // Packet bag byte = FIRST_BAG_EQUIP_SLOT + bagIndex.
    static constexpr int FIRST_BAG_EQUIP_SLOT = 19;
    static constexpr int NUM_BAG_SLOTS = 4;
    static constexpr int MAX_BAG_SIZE = 36;
    static constexpr int BANK_SLOTS = 28;
    static constexpr int BANK_BAG_SLOTS = 7;

    Inventory();

    // Backpack
    [[nodiscard]] const ItemSlot& getBackpackSlot(int index) const;
    bool setBackpackSlot(int index, const ItemDef& item);
    bool clearBackpackSlot(int index);
    [[nodiscard]] int getBackpackSize() const { return BACKPACK_SLOTS; }

    /// How many of an item the backpack and equipped bags hold together.
    ///
    /// A slot that is not empty holds at least one of what is in it, whatever
    /// its stack count says. Several of the item paths leave the count at zero
    /// for something that does not stack, and the five places that counted
    /// items split on what to do about it: the Lua bindings read such a slot
    /// as one and the client's own counters read it as none. A reagent could
    /// be in the bags and missing from the crafting check at the same time.
    [[nodiscard]] uint32_t countItem(uint32_t itemId) const;

    // Equipment
    [[nodiscard]] const ItemSlot& getEquipSlot(EquipSlot slot) const;
    bool setEquipSlot(EquipSlot slot, const ItemDef& item);

    // Keyring
    [[nodiscard]] const ItemSlot& getKeyringSlot(int index) const;
    bool setKeyringSlot(int index, const ItemDef& item);
    [[nodiscard]] int getKeyringSize() const { return KEYRING_SLOTS; }

    // Extra bags
    [[nodiscard]] int getBagSize(int bagIndex) const;
    void setBagSize(int bagIndex, int size);
    // Special containers (quivers, ammo pouches, profession bags) only accept
    // their own item type: sorting skips them and the UI marks their slots.
    void setBagSpecial(int bagIndex, bool special);
    [[nodiscard]] const ItemSlot& getBagSlot(int bagIndex, int slotIndex) const;
    bool setBagSlot(int bagIndex, int slotIndex, const ItemDef& item);

    // Bank slots (28 main + 7 bank bags)
    [[nodiscard]] const ItemSlot& getBankSlot(int index) const;
    bool setBankSlot(int index, const ItemDef& item);
    bool clearBankSlot(int index);

    [[nodiscard]] const ItemSlot& getBankBagSlot(int bagIndex, int slotIndex) const;
    bool setBankBagSlot(int bagIndex, int slotIndex, const ItemDef& item);
    [[nodiscard]] int getBankBagSize(int bagIndex) const;
    void setBankBagSize(int bagIndex, int size);
    [[nodiscard]] const ItemSlot& getBankBagItem(int bagIndex) const;
    void setBankBagItem(int bagIndex, const ItemDef& item);

    [[nodiscard]] uint8_t getPurchasedBankBagSlots() const { return purchasedBankBagSlots_; }
    void setPurchasedBankBagSlots(uint8_t count) { purchasedBankBagSlots_ = count; }

    // Swap two bag slots (equip items + contents)
    void swapBagContents(int bagA, int bagB);

    // Utility
    [[nodiscard]] int findFreeBackpackSlot() const;
    bool addItem(const ItemDef& item);

    // Sort all bag slots (backpack + equip bags) by quality desc → itemId asc → stackCount desc.
    // Purely client-side: reorders the local inventory struct without server interaction.
    void sortBags();

    // Sort the bank the same way (main bank slots + bank bag contents). mainSlotCount is the
    // number of usable main bank slots for the active expansion (24 Classic, 28 TBC/WotLK) so
    // sorting never spills items into slots the server doesn't have.
    void sortBank(int mainSlotCount);

    // Sort one bank bag's contents in place, leaving the rest of the bank alone.
    // Sorting the whole bank pools everything into the main slots, which is the
    // wrong tool when a bag is being kept as a deliberate category.
    void sortBankBag(int bagIndex);

    // A single swap operation using WoW bag/slot addressing (for CMSG_SWAP_ITEM).
    struct SwapOp {
        uint8_t srcBag;
        uint8_t srcSlot;
        uint8_t dstBag;
        uint8_t dstSlot;
    };

    /// One slot as a sort sees it: where it is on the wire, and enough of what
    /// is in it to order it.
    struct SortEntry {
        uint8_t bag;
        uint8_t slot;
        uint32_t itemId;
        ItemQuality quality;
        uint32_t stackCount;
    };

    /// The swaps that put a run of slots into sorted order.
    ///
    /// Bags, the main bank and a single bank bag each planned this for
    /// themselves, and the plan is the half that is hard: ordering the entries
    /// is a comparator, but turning a target permutation into a sequence of
    /// two-slot swaps that a server will accept one at a time is not, and a
    /// mistake there moves an item somewhere nobody asked for.
    ///
    /// Order is quality descending, then item id ascending, then stack count
    /// descending, with empty slots last. Ties keep their existing order, so
    /// two identical stacks are not swapped for nothing.
    ///
    /// Two empty slots are never swapped with each other: that costs a packet
    /// and changes nothing.
    static std::vector<SwapOp> swapsToSort(const std::vector<SortEntry>& entries);

    // Pour partial stacks of the same item together, so two half stacks become
    // one. Dropping a stack onto another of the same item is a swap as far as the
    // wire is concerned - the server merges what fits and leaves the rest behind -
    // so the returned ops go through the same queue as a sort.
    //
    // Unlike the sort, this both plans and applies: keeping the plan and the local
    // preview in one pass means the two cannot disagree.
    std::vector<SwapOp> mergePartialStacks();
    std::vector<SwapOp> mergeBankPartialStacks(int mainSlotCount);

    // Compute the CMSG_SWAP_ITEM operations needed to reach sorted order.
    // Does NOT modify the inventory - caller is responsible for sending packets.
    [[nodiscard]] std::vector<SwapOp> computeSortSwaps() const;
    [[nodiscard]] std::vector<SwapOp> computeBankSortSwaps(int mainSlotCount) const;
    [[nodiscard]] std::vector<SwapOp> computeBankBagSortSwaps(int bagIndex) const;

    // WoW bag/slot addressing for bank storage (used by sort + drag-drop):
    // main bank slots live in bag 0xFF at slot BANK_SLOT_START + index; each bank bag's
    // contents live in bag BANK_BAG_CONTAINER_START + bagIndex.
    static constexpr uint8_t BANK_SLOT_START = 39;
    static constexpr uint8_t BANK_BAG_CONTAINER_START = 67;


private:
    std::array<ItemSlot, BACKPACK_SLOTS> backpack{};
    std::array<ItemSlot, KEYRING_SLOTS> keyring_{};
    std::array<ItemSlot, NUM_EQUIP_SLOTS> equipment{};

    struct BagData {
        int size = 0;
        bool special = false;  // Quiver/ammo pouch/profession bag - restricted contents
        ItemSlot bagItem;  // The bag item itself (for icon/name/tooltip)
        std::array<ItemSlot, MAX_BAG_SIZE> slots{};
    };
    std::array<BagData, NUM_BAG_SLOTS> bags{};

    // Bank
    std::array<ItemSlot, BANK_SLOTS> bankSlots_{};
    std::array<BagData, BANK_BAG_SLOTS> bankBags_{};
    uint8_t purchasedBankBagSlots_ = 0;
};


} // namespace game
} // namespace wowee
