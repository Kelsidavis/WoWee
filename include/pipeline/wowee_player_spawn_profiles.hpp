#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Player Spawn Profile catalog (.wpsp) -
// novel replacement for AzerothCore's playercreateinfo
// SQL table plus the per-class/race starting fields in
// CharStartOutfit.dbc. Defines the initial state for a
// newly created character: starting map / zone /
// position, bind point (Hearthstone destination),
// starting items, and starting spells.
//
// One entry per (class, race) combination - a Human
// Warrior spawns at Northshire Abbey with a Warrior's
// Hammer and Heroic Strike already learned, while an
// Orc Hunter spawns in Valley of Trials with a starter
// gun and Aimed Shot. Death Knights have their own
// preset spawning at lvl 55 in Acherus.
//
// Cross-references with previously-added formats:
//   WCHC: race / class fields use the same bit layout
//         as WCHC class / race IDs.
//   WMS:  mapId / bindMapId reference WMS map entries.
//   WIT:  startingItem*Id reference WIT item entries.
//   WSPL: startingSpell*Id reference WSPL spell entries.
//
// Binary layout (little-endian):
//   magic[4]            = "WPSP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     profileId (uint32)
//     nameLen + name
//     descLen + description
//     raceMask (uint32)
//     classMask (uint32)
//     mapId (uint32)
//     zoneId (uint32)
//     spawnX (float) / spawnY (float) / spawnZ (float)
//     spawnFacing (float)
//     bindMapId (uint32)
//     bindZoneId (uint32)
//     startingItem1Id (uint32) / startingItem1Count (uint32)
//     startingItem2Id (uint32) / startingItem2Count (uint32)
//     startingItem3Id (uint32) / startingItem3Count (uint32)
//     startingItem4Id (uint32) / startingItem4Count (uint32)
//     startingSpell1Id (uint32)
//     startingSpell2Id (uint32)
//     startingSpell3Id (uint32)
//     startingSpell4Id (uint32)
//     startingLevel (uint8) / pad[3]
//     iconColorRGBA (uint32)
struct WoweePlayerSpawnProfile {
    struct Entry {
        uint32_t profileId = 0;
        std::string name;
        std::string description;
        uint32_t raceMask = 0;
        uint32_t classMask = 0;
        uint32_t mapId = 0;
        uint32_t zoneId = 0;
        float spawnX = 0.0f;
        float spawnY = 0.0f;
        float spawnZ = 0.0f;
        float spawnFacing = 0.0f;
        uint32_t bindMapId = 0;
        uint32_t bindZoneId = 0;
        uint32_t startingItem1Id = 0;
        uint32_t startingItem1Count = 0;
        uint32_t startingItem2Id = 0;
        uint32_t startingItem2Count = 0;
        uint32_t startingItem3Id = 0;
        uint32_t startingItem3Count = 0;
        uint32_t startingItem4Id = 0;
        uint32_t startingItem4Count = 0;
        uint32_t startingSpell1Id = 0;
        uint32_t startingSpell2Id = 0;
        uint32_t startingSpell3Id = 0;
        uint32_t startingSpell4Id = 0;
        uint8_t startingLevel = 1;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t profileId) const;

    // Find the first profile whose race/class masks both
    // include the given bits. Used by character creation
    // to look up "Human Warrior" given race=Human(1) +
    // class=Warrior(1).
    [[nodiscard]] const Entry* findByRaceClass(uint32_t raceBit,
                                  uint32_t classBit) const;
};

class WoweePlayerSpawnProfileLoader {
public:
    static bool save(const WoweePlayerSpawnProfile& cat,
                     const std::string& basePath);
    static WoweePlayerSpawnProfile load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-psp* variants.
    //
    //   makeAlliance   - 5 Alliance combos covering each
    //                     starting zone (Human Warrior in
    //                     Northshire, Dwarf Hunter in Coldridge,
    //                     NightElf Druid in Shadowglen, Gnome
    //                     Mage in Anvilmar, Draenei Shaman
    //                     in Ammen Vale).
    //   makeHorde      - 5 Horde combos (Orc Warrior in
    //                     Valley of Trials, Tauren Druid in
    //                     Camp Narache, Undead Mage in Deathknell,
    //                     Troll Hunter in Valley of Trials,
    //                     BloodElf Priest in Sunstrider Isle).
    //   makeDeathKnight - 2 DK combos (Alliance Human DK and
    //                     Horde Orc DK) starting at lvl 55 in
    //                     Acherus, with the standard DK
    //                     starter spell set.
    static WoweePlayerSpawnProfile makeAlliance(const std::string& catalogName);
    static WoweePlayerSpawnProfile makeHorde(const std::string& catalogName);
    static WoweePlayerSpawnProfile makeDeathKnight(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
