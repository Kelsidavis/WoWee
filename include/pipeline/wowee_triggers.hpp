#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Area Trigger catalog (.wtrg) - novel
// replacement for Blizzard's AreaTrigger.dbc +
// AreaTriggerTeleport.dbc + the AzerothCore-style
// areatrigger_template / areatrigger_teleport SQL tables.
// The 29th open format added to the editor.
//
// Defines proximity-based event zones - when a player
// enters a defined region (box or sphere), the runtime
// fires the trigger's action: teleport to another map,
// award exploration XP for a quest, run a server script,
// gate an instance entrance behind a key item, etc.
//
// Cross-references with previously-added formats:
//   WTRG.entry.mapId / areaId  → WMS.map.mapId / WMS.area.areaId
//   WTRG.entry.actionTarget (kind=Teleport)        → WMS.mapId
//   WTRG.entry.actionTarget (kind=QuestExploration) → WQT.questId
//   WTRG.entry.requiredQuestId → WQT.entry.questId
//   WTRG.entry.requiredItemId  → WIT.entry.itemId (key)
//
// Binary layout (little-endian):
//   magic[4]            = "WTRG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     triggerId (uint32)
//     mapId (uint32)
//     areaId (uint32)
//     nameLen + name
//     center (3 × float)
//     shape (uint8) + kind (uint8) + pad[2]
//     boxDims (3 × float)
//     radius (float)
//     actionTarget (uint32)
//     dest (3 × float)
//     destOrientation (float)
//     requiredQuestId (uint32)
//     requiredItemId (uint32)
//     minLevel (uint16) + pad[2]
struct WoweeTrigger {
    enum Shape : uint8_t {
        ShapeBox    = 0,
        ShapeSphere = 1,
    };

    enum Kind : uint8_t {
        KindTeleport         = 0,    // moves player to dest
        KindQuestExploration = 1,    // awards XP toward a quest
        KindScript           = 2,    // runs a server script
        KindInstanceEntrance = 3,    // dungeon / raid portal
        KindAreaName         = 4,    // shows "Discovered: ..." text
        KindCombatStartZone  = 5,    // marks PvP-flag boundary
        KindWaypoint         = 6,    // waypoint marker (NPCs / quests)
    };

    struct Entry {
        uint32_t triggerId = 0;
        uint32_t mapId = 0;
        uint32_t areaId = 0;          // 0 = anywhere on the map
        std::string name;
        glm::vec3 center{0};
        uint8_t shape = ShapeBox;
        uint8_t kind = KindAreaName;
        glm::vec3 boxDims{0};         // half-extents; ignored if sphere
        float radius = 0.0f;          // ignored if box
        uint32_t actionTarget = 0;
        glm::vec3 dest{0};            // teleport destination
        float destOrientation = 0.0f; // teleport facing (radians)
        uint32_t requiredQuestId = 0; // 0 = no quest gate
        uint32_t requiredItemId = 0;  // 0 = no key item required
        uint16_t minLevel = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t triggerId) const;

    static const char* shapeName(uint8_t s);
    static const char* kindName(uint8_t k);
};

class WoweeTriggerLoader {
public:
    static bool save(const WoweeTrigger& cat,
                     const std::string& basePath);
    static WoweeTrigger load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-triggers* variants.
    //
    //   makeStarter - 2 triggers: 1 area-name (player enters
    //                  Goldshire) + 1 quest exploration
    //                  (matches WQT 100 "Investigate the Camp").
    //   makeDungeon - 3 triggers around an instance: outdoor
    //                  approach area-name + portal-style
    //                  teleport into the instance + instance
    //                  exit teleport back outdoors.
    //   makeFlightPath - 2 triggers marking flight-master
    //                     proximity (Stormwind / Goldshire) so
    //                     the runtime knows when to open the
    //                     flight UI without explicit interact.
    static WoweeTrigger makeStarter(const std::string& catalogName);
    static WoweeTrigger makeDungeon(const std::string& catalogName);
    static WoweeTrigger makeFlightPath(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
