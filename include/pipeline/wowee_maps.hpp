#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Map / Area catalog (.wms) - novel replacement
// for Blizzard's Map.dbc + AreaTable.dbc + the AzerothCore-
// style world_zone SQL tables. The 26th open format added
// to the editor.
//
// Defines two related kinds of locator:
//   • Maps  - top-level worlds (continents, instances, BGs).
//             Each map has a friendly name, type, expansion
//             tag, and player-count cap.
//   • Areas - sub-zones within maps with friendly names,
//             parent-area chain, recommended level range,
//             faction-territory marker, exploration XP, and
//             an ambient-sound cross-reference into WSND.
//
// One file holds both arrays. The runtime uses Areas for
// minimap labels, location strings under the player frame,
// "Discover Sub-zone" XP gains, and ambient music selection.
//
// Cross-references with previously-added formats:
//   WMS.area.ambienceSoundId    → WSND.entry.soundId
//   WMS.area.parentAreaId       → WMS.area.areaId (intra-format)
//   WSPN entries are tied to WMS.area boundaries by
//                                world position (no direct id)
//
// Binary layout (little-endian):
//   magic[4]            = "WMSX"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   mapCount (uint32)
//   maps (each):
//     mapId (uint32)
//     nameLen + name
//     shortLen + shortName
//     mapType (uint8) / expansionId (uint8) / pad[2]
//     maxPlayers (uint16) / pad[2]
//   areaCount (uint32)
//   areas (each):
//     areaId (uint32)
//     mapId (uint32)
//     parentAreaId (uint32)
//     nameLen + name
//     minLevel (uint16) / maxLevel (uint16)
//     factionGroup (uint8) / pad[3]
//     explorationXP (uint32)
//     ambienceSoundId (uint32)
struct WoweeMaps {
    enum MapType : uint8_t {
        Continent    = 0,
        Instance     = 1,
        Raid         = 2,
        Battleground = 3,
        Arena        = 4,
    };

    enum ExpansionId : uint8_t {
        Classic = 0,
        Tbc     = 1,
        Wotlk   = 2,
        Cata    = 3,
        Mop     = 4,
    };

    enum FactionGroup : uint8_t {
        FactionBoth     = 0,
        FactionAlliance = 1,
        FactionHorde    = 2,
        FactionContested = 3,    // PvP-flagging zone
    };

    struct Map {
        uint32_t mapId = 0;
        std::string name;
        std::string shortName;          // e.g. "EK", "Kalim", "DM"
        uint8_t mapType = Continent;
        uint8_t expansionId = Classic;
        uint16_t maxPlayers = 0;        // 0 = unlimited (continent)
    };

    struct Area {
        uint32_t areaId = 0;
        uint32_t mapId = 0;
        uint32_t parentAreaId = 0;      // 0 = top-level
        std::string name;
        uint16_t minLevel = 1;
        uint16_t maxLevel = 1;
        uint8_t factionGroup = FactionBoth;
        uint32_t explorationXP = 0;
        uint32_t ambienceSoundId = 0;   // WSND cross-ref, 0 = none
    };

    std::string name;
    std::vector<Map> maps;
    std::vector<Area> areas;

    [[nodiscard]] bool isValid() const { return !maps.empty(); }

    [[nodiscard]] const Map* findMap(uint32_t mapId) const;
    [[nodiscard]] const Area* findArea(uint32_t areaId) const;

    static const char* mapTypeName(uint8_t t);
    static const char* expansionName(uint8_t e);
    static const char* factionGroupName(uint8_t f);
};

class WoweeMapsLoader {
public:
    static bool save(const WoweeMaps& cat,
                     const std::string& basePath);
    static WoweeMaps load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-maps* variants.
    //
    //   makeStarter - 1 map (continent) + 3 areas (capital,
    //                  starter zone, neighboring zone with
    //                  parent chain).
    //   makeClassic - 2 continents + a small dungeon instance
    //                  + 6 areas wiring sub-zones to parents
    //                  (Stormwind > City Trade District etc).
    //   makeBgArena - 2 maps showcasing Battleground (40 players)
    //                  and Arena (5v5) types.
    static WoweeMaps makeStarter(const std::string& catalogName);
    static WoweeMaps makeClassic(const std::string& catalogName);
    static WoweeMaps makeBgArena(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
