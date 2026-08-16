#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Looking-For-Group Dungeon catalog (.wlfg) -
// novel replacement for Blizzard's LFGDungeons.dbc plus
// the AzerothCore-style dungeon-finder reward tables.
// Defines the dungeons / raids that the LFG / Dungeon
// Finder / Raid Browser presents to players, with their
// level brackets, group-size requirements, role
// requirements (tank / heal / DPS), and queue-completion
// rewards.
//
// Cross-references with previously-added formats:
//   WLFG.entry.mapId                → WMS.map.mapId
//   WLFG.entry.queueRewardItemId    → WIT.itemId
//   WLFG.entry.firstClearAchievement → WACH.achievementId
//
// Binary layout (little-endian):
//   magic[4]            = "WLFG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     dungeonId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32)
//     minLevel (uint16) / maxLevel (uint16) /
//       recommendedLevel (uint16) / minGearLevel (uint16)
//     difficulty (uint8) / groupSize (uint8) /
//       requiredRolesMask (uint8) / expansionRequired (uint8)
//     queueRewardItemId (uint32)
//     queueRewardEmblemCount (uint16) / pad[2]
//     firstClearAchievement (uint32)
struct WoweeLFGDungeon {
    enum Difficulty : uint8_t {
        Normal      = 0,
        Heroic      = 1,    // dungeon heroic + 10-man heroic raid
        Mythic      = 2,    // 25-man heroic raid
        Hardmode    = 3,    // Ulduar-style toggleable
    };

    enum ExpansionRequired : uint8_t {
        Classic = 0,
        TBC     = 1,
        WotLK   = 2,
        TurtleWoW = 3,
    };

    // requiredRolesMask bits - combine to require multiple
    // role types in the queue-formed group.
    static constexpr uint8_t kRoleTank = 0x01;
    static constexpr uint8_t kRoleHeal = 0x02;
    static constexpr uint8_t kRoleDPS  = 0x04;
    static constexpr uint8_t kRoleAll  = kRoleTank | kRoleHeal | kRoleDPS;

    struct Entry {
        uint32_t dungeonId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;            // WMS cross-ref
        uint16_t minLevel = 1;
        uint16_t maxLevel = 80;
        uint16_t recommendedLevel = 0;
        uint16_t minGearLevel = 0;     // ilvl floor for queue
        uint8_t difficulty = Normal;
        uint8_t groupSize = 5;         // 5 / 10 / 25 / 40
        uint8_t requiredRolesMask = kRoleAll;
        uint8_t expansionRequired = Classic;
        uint32_t queueRewardItemId = 0;     // WIT cross-ref
        uint16_t queueRewardEmblemCount = 0;
        uint32_t firstClearAchievement = 0; // WACH cross-ref
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t dungeonId) const;

    static const char* difficultyName(uint8_t d);
    static const char* expansionRequiredName(uint8_t e);
};

class WoweeLFGDungeonLoader {
public:
    static bool save(const WoweeLFGDungeon& cat,
                     const std::string& basePath);
    static WoweeLFGDungeon load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-lfg* variants.
    //
    //   makeStarter - 3 classic-era dungeons (Ragefire
    //                  Chasm 13-18, Wailing Caverns
    //                  17-24, Deadmines 18-23).
    //   makeHeroic  - 5 WotLK 80-level heroics with
    //                  emblem rewards (Halls of Lightning
    //                  Heroic, Halls of Stone, Utgarde
    //                  Pinnacle, Violet Hold, Old Kingdom).
    //   makeRaid    - 3 raid catalog entries (Naxxramas-25,
    //                  Ulduar-25 with Hardmode, ToC-25)
    //                  with achievement cross-refs and
    //                  larger groupSize.
    static WoweeLFGDungeon makeStarter(const std::string& catalogName);
    static WoweeLFGDungeon makeHeroic(const std::string& catalogName);
    static WoweeLFGDungeon makeRaid(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
