// map_resolver.cpp - Centralized map navigation resolution for the world map.
// Map folder names resolved from a built-in table matching
// Data/interface/worldmap/ - no dependency on WorldLoader.
#include "rendering/world_map/map_resolver.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <cctype>

namespace wowee {
namespace rendering {
namespace world_map {

// ── Worldmap folder table (from Data/interface/worldmap/) ────
// Each entry maps a DBC MapID to its worldmap folder name and UI display name.
// Folder names match the directories under Data/interface/worldmap/.

struct MapFolderEntry {
    uint32_t mapId;
    const char* folder;       // worldmap folder name (case as on disk)
    const char* displayName;  // UI display name
};

static constexpr MapFolderEntry kMapFolders[] = {
    // Special UI-only views (no DBC MapID - sentinel values)
    { UINT32_MAX,     "World",        "World"            },
    { UINT32_MAX - 1, "Cosmic",       "Cosmic"           },
    // Continents
    {   0, "Azeroth",      "Eastern Kingdoms" },
    {   1, "Kalimdor",     "Kalimdor"         },
    { 530, "Expansion01",  "Outland"          },
    { 571, "Northrend",    "Northrend"        },
    // Dungeons / instances with worldmap folders
    // (Data/interface/worldmap/<folder>/ exists for these)
    {  33, "Shadowfang",           "Shadowfang Keep"           },  // placeholder – no folder yet
    { 209, "Tanaris",              "Tanaris"                   },  // shared with zone
    { 534, "CoTStratholme",        "Caverns of Time"           },
    { 574, "UtgardeKeep",          "Utgarde Keep"              },
    { 575, "UtgardePinnacle",      "Utgarde Pinnacle"          },
    { 578, "Nexus80",              "The Nexus"                 },
    { 595, "ThecullingOfStratholme","Culling of Stratholme"    },
    { 599, "HallsOfLightning",     "Halls of Lightning"        },
    { 600, "HallsOfStone",         "Halls of Stone"            },
    { 601, "DrakTheron",           "Drak'Theron Keep"          },
    { 602, "GunDrak",              "Gundrak"                   },
    { 603, "Ulduar77",             "Ulduar"                    },
    { 608, "VioletHold",           "Violet Hold"               },
    { 619, "AhnKahet",             "Ahn'kahet"                 },
    { 631, "IcecrownCitadel",      "Icecrown Citadel"          },
    { 632, "TheForgeOfSouls",      "Forge of Souls"            },
    { 649, "TheArgentColiseum",    "Trial of the Crusader"     },
    { 658, "PitOfSaron",           "Pit of Saron"              },
    { 668, "HallsOfReflection",    "Halls of Reflection"       },
    { 724, "TheRubySanctum",       "Ruby Sanctum"              },
};

// ── Map folder lookup functions ──────────────────────────────

const char* mapIdToFolder(uint32_t mapId) {
    for (const auto& mapFolder : kMapFolders) {
        if (mapFolder.mapId == mapId)
            return mapFolder.folder;
    }
    return "";
}

int folderToMapId(const std::string& folder) {
    for (const auto& mapFolder : kMapFolders) {
        // Case-insensitive compare
        const char* entry = mapFolder.folder;
        if (folder.size() != std::char_traits<char>::length(entry)) continue;
        bool match = true;
        for (size_t j = 0; j < folder.size(); j++) {
            if (std::tolower(static_cast<unsigned char>(folder[j])) !=
                std::tolower(static_cast<unsigned char>(entry[j]))) {
                match = false;
                break;
            }
        }
        if (match) return static_cast<int>(mapFolder.mapId);
    }
    return -1;
}

const char* mapDisplayName(uint32_t mapId) {
    for (const auto& mapFolder : kMapFolders) {
        if (mapFolder.mapId == mapId)
            return mapFolder.displayName;
    }
    return nullptr;
}

// ── Helper: find best continent zone for a mapId ─────────────

int findContinentForMapId(const std::vector<Zone>& zones,
                           uint32_t mapId,
                           int cosmicIdx) {
    // 1) Prefer a leaf continent whose displayMapID matches the target mapId.
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx) continue;
        if (zones[i].areaID != 0) continue;
        if (isLeafContinent(zones, i) && zones[i].displayMapID == mapId)
            return i;
    }

    // 2) Find the first non-root, non-cosmic continent.
    int firstContinent = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (i == cosmicIdx) continue;
        if (zones[i].areaID != 0) continue;
        if (firstContinent < 0) firstContinent = i;
        if (!isRootContinent(zones, i)) return i;
    }

    // 3) Fallback to first continent entry
    return firstContinent;
}

// ── Resolve WORLD view region click ──────────────────────────

MapResolveResult resolveWorldRegionClick(uint32_t regionMapId,
                                          const std::vector<Zone>& zones,
                                          int currentMapId,
                                          int cosmicIdx) {
    MapResolveResult result;

    if (static_cast<int>(regionMapId) == currentMapId) {
        // Target map is already loaded - navigate to the matching continent
        // within the current zone data (no reload needed).
        int contIdx = findContinentForMapId(zones, regionMapId, cosmicIdx);
        if (contIdx >= 0) {
            result.action = MapResolveAction::NAVIGATE_CONTINENT;
            result.targetZoneIdx = contIdx;
            LOG_INFO("resolveWorldRegionClick: mapId=", regionMapId,
                     " matches current map - NAVIGATE_CONTINENT idx=", contIdx);
        } else {
            LOG_WARNING("resolveWorldRegionClick: mapId=", regionMapId,
                        " matches current but no continent found");
        }
        return result;
    }

    // Different map - need to load it
    const char* folder = mapIdToFolder(regionMapId);
    if (folder[0]) {
        result.action = MapResolveAction::LOAD_MAP;
        result.targetMapName = folder;
        LOG_INFO("resolveWorldRegionClick: mapId=", regionMapId,
                 " → LOAD_MAP '", folder, "'");
    } else {
        LOG_WARNING("resolveWorldRegionClick: unknown mapId=", regionMapId);
    }
    return result;
}

// ── Resolve CONTINENT view zone click ────────────────────────

MapResolveResult resolveZoneClick(int zoneIdx,
                                   const std::vector<Zone>& zones,
                                   int currentMapId) {
    MapResolveResult result;
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return result;

    const auto& zone = zones[zoneIdx];

    // If the zone's displayMapID differs from the current map, it belongs to
    // a different continent/map. Load that map instead.
    // Skip sentinel values (UINT32_MAX / UINT32_MAX-1) used by kMapFolders for
    // World/Cosmic; the DBC stores -1 (0xFFFFFFFF) to mean "no display map".
    if (zone.displayMapID != 0 &&
        zone.displayMapID < UINT32_MAX - 1 &&
        static_cast<int>(zone.displayMapID) != currentMapId) {
        const char* folder = mapIdToFolder(zone.displayMapID);
        if (folder[0]) {
            result.action = MapResolveAction::LOAD_MAP;
            result.targetMapName = folder;
            LOG_INFO("resolveZoneClick: zone[", zoneIdx, "] '", zone.areaName,
                     "' displayMapID=", zone.displayMapID, " → LOAD_MAP '", folder, "'");
            return result;
        }
    }

    // Normal case: enter the zone within the current map
    result.action = MapResolveAction::ENTER_ZONE;
    result.targetZoneIdx = zoneIdx;
    return result;
}

// ── Resolve COSMIC view click ────────────────────────────────

MapResolveResult resolveCosmicClick(uint32_t targetMapId) {
    MapResolveResult result;
    const char* folder = mapIdToFolder(targetMapId);
    if (folder[0]) {
        result.action = MapResolveAction::LOAD_MAP;
        result.targetMapName = folder;
    }
    return result;
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
