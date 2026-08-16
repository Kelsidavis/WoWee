#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Template (.wit) - novel replacement for
// Blizzard's Item.dbc + ItemDisplayInfo.dbc + the SQL
// item_template tables that AzerothCore-style servers store
// item definitions in. The 12th open format added to the
// editor.
//
// One file holds the catalog of all items in a content pack:
// weapons, armor, consumables, quest items, trade goods, etc.
// Each entry pairs the gameplay metadata (stats, level reqs,
// flags) with the display metadata (icon / model displayId,
// quality color) - the runtime needs both together to render
// inventory tooltips and equip slots.
//
// Binary layout (little-endian):
//   magic[4]            = "WITM"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes              -- catalog label
//   entryCount (uint32)
//   entries (each):
//     itemId (uint32)
//     displayId (uint32)
//     quality (uint8)             -- poor..artifact
//     itemClass (uint8)           -- consumable / weapon / armor / ...
//     itemSubClass (uint8)
//     inventoryType (uint8)       -- equip slot (0 = non-equip)
//     flags (uint32)              -- unique / BoP / BoE / quest / ...
//     requiredLevel (uint16)
//     itemLevel (uint16)
//     sellPriceCopper (uint32)
//     buyPriceCopper (uint32)
//     maxStack (uint16)
//     durability (uint16)         -- 0 for non-equippable
//     damageMin (uint32)          -- weapons only; 0 otherwise
//     damageMax (uint32)
//     attackSpeedMs (uint32)
//     statCount (uint8) + pad[3]
//     stats (statCount × {type: uint8, pad, value: int16})
//     nameLen (uint32) + name bytes
//     descLen (uint32) + description bytes
struct WoweeItem {
    enum Quality : uint8_t {
        Poor      = 0,
        Common    = 1,
        Uncommon  = 2,
        Rare      = 3,
        Epic      = 4,
        Legendary = 5,
        Artifact  = 6,
        Heirloom  = 7,
    };

    enum Class : uint8_t {
        Consumable  = 0,
        Container   = 1,
        Weapon      = 2,
        Gem         = 3,
        Armor       = 4,
        Reagent     = 5,
        Projectile  = 6,
        TradeGoods  = 7,
        Recipe      = 9,
        Quiver      = 11,
        Quest       = 12,
        Key         = 13,
        Misc        = 15,
    };

    enum InventoryType : uint8_t {
        NonEquip  = 0,
        Head      = 1,
        Neck      = 2,
        Shoulders = 3,
        Body      = 4,    // shirt
        Chest     = 5,
        Waist     = 6,
        Legs      = 7,
        Feet      = 8,
        Wrists    = 9,
        Hands     = 10,
        Finger    = 11,
        Trinket   = 12,
        Weapon1H  = 13,
        Shield    = 14,
        Ranged    = 15,
        Cloak     = 16,
        Weapon2H  = 17,
    };

    enum Flags : uint32_t {
        Unique      = 0x01,
        Conjured    = 0x02,
        Openable    = 0x04,
        Heroic      = 0x08,
        BindOnPickup = 0x10,
        BindOnEquip  = 0x20,
        QuestItem    = 0x40,
        NoSellable   = 0x80,
    };

    // Common stat types (matches WoW's ItemModType numbering
    // for the most-used subset; the format permits any uint8).
    enum StatType : uint8_t {
        StatNone      = 0,
        StatHealth    = 1,
        StatMana      = 2,
        StatAgility   = 3,
        StatStrength  = 4,
        StatIntellect = 5,
        StatSpirit    = 6,
        StatStamina   = 7,
        StatDefense   = 31,
    };

    struct Stat {
        uint8_t type = StatNone;
        int16_t value = 0;
    };

    struct Entry {
        uint32_t itemId = 0;
        uint32_t displayId = 0;
        uint8_t quality = Common;
        uint8_t itemClass = Misc;
        uint8_t itemSubClass = 0;
        uint8_t inventoryType = NonEquip;
        uint32_t flags = 0;
        uint16_t requiredLevel = 0;
        uint16_t itemLevel = 1;
        uint32_t sellPriceCopper = 0;
        uint32_t buyPriceCopper = 0;
        uint16_t maxStack = 1;
        uint16_t durability = 0;
        uint32_t damageMin = 0;
        uint32_t damageMax = 0;
        uint32_t attackSpeedMs = 0;
        std::vector<Stat> stats;     // up to 255 (uint8 count)
        std::string name;
        std::string description;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by itemId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t itemId) const;

    static const char* qualityName(uint8_t q);
    static const char* classNameOf(uint8_t c);
    static const char* slotName(uint8_t s);
    static const char* statName(uint8_t t);
};

class WoweeItemLoader {
public:
    static bool save(const WoweeItem& cat,
                     const std::string& basePath);
    static WoweeItem load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-items* variants.
    //
    //   makeStarter - a tiny demo catalog: 1 weapon + 1 chest
    //                  armor + 1 healing potion + 1 quest item.
    //   makeWeapons - 5 weapon entries spanning common,
    //                  uncommon, rare, epic; both 1H and 2H.
    //   makeArmor   - full gear set: head + chest + legs +
    //                  feet + hands + cloak.
    static WoweeItem makeStarter(const std::string& catalogName);
    static WoweeItem makeWeapons(const std::string& catalogName);
    static WoweeItem makeArmor(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
