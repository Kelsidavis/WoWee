#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open World Locations catalog (.wloc) -
// novel unified replacement for the half-dozen
// proprietary location tables vanilla WoW
// scattered across AreaPOI.dbc (zone-discovery
// landmarks), gameobject_template.spawn rows
// (herb/mineral/fishing nodes), creature_template
// rare-spawn entries, and AreaTrigger.dbc (zone
// boundary teleports). Each WLOC entry binds one
// world coord (mapId, x, y, z) to its kind (POI /
// RareSpawn / HerbNode / MineralVein / FishingSpot
// / AreaTrigger / PortalLanding), respawn timer,
// gathering-skill gate, and on-discovery XP.
//
// Cross-references with previously-added formats:
//   WMS:   mapId references the WMS map catalog.
//   WSKL:  requiredSkillId references the WSKL
//          skill catalog (Herbalism=182,
//          Mining=186, Fishing=356).
//   WPRT:  PortalLanding-kind locations are
//          referenced as destination coords by
//          WPRT mage portal entries.
//
// Binary layout (little-endian):
//   magic[4]            = "WLOC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     locationId (uint32)
//     nameLen + name
//     mapId (uint32)
//     areaId (uint32)
//     x (float)
//     y (float)
//     z (float)
//     locKind (uint8)            - 0=POI /
//                                   1=RareSpawn /
//                                   2=HerbNode /
//                                   3=MineralVein /
//                                   4=FishingSpot /
//                                   5=AreaTrigger /
//                                   6=PortalLanding
//     iconIndex (uint8)
//     factionAccess (uint8)      - 0=Both /
//                                   1=Alliance /
//                                   2=Horde /
//                                   3=Neutral
//     pad0 (uint8)
//     respawnSec (uint32)        - 0 = static
//                                   (POI / Trigger /
//                                   Portal land)
//     discoverableXp (uint32)    - XP on first
//                                   discovery (POI
//                                   only)
//     requiredSkillId (uint16)   - 0 if not gather-
//                                   gated
//     requiredSkillLevel (uint16) - gathering skill
//                                    minimum
struct WoweeWorldLocations {
    enum LocKind : uint8_t {
        POI           = 0,
        RareSpawn     = 1,
        HerbNode      = 2,
        MineralVein   = 3,
        FishingSpot   = 4,
        AreaTrigger   = 5,
        PortalLanding = 6,
    };

    enum FactionAccess : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
        Neutral  = 3,
    };

    struct Entry {
        uint32_t locationId = 0;
        std::string name;
        uint32_t mapId = 0;
        uint32_t areaId = 0;
        float x = 0.f;
        float y = 0.f;
        float z = 0.f;
        uint8_t locKind = POI;
        uint8_t iconIndex = 0;
        uint8_t factionAccess = Both;
        uint8_t pad0 = 0;
        uint32_t respawnSec = 0;
        uint32_t discoverableXp = 0;
        uint16_t requiredSkillId = 0;
        uint16_t requiredSkillLevel = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t locationId) const;

    // Returns all locations on a map (used by the
    // world-map UI to populate per-map markers).
    [[nodiscard]] std::vector<const Entry*> findByMap(uint32_t mapId) const;

    // Returns all locations of a single kind on a
    // map. Used by the herbalism-tracking UI to
    // show only HerbNode markers without polluting
    // with POIs.
    [[nodiscard]] std::vector<const Entry*> findByMapAndKind(uint32_t mapId,
                                                  uint8_t kind) const;
};

class WoweeWorldLocationsLoader {
public:
    static bool save(const WoweeWorldLocations& cat,
                     const std::string& basePath);
    static WoweeWorldLocations load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-loc* variants.
    //
    //   makeAlliancePOIs   - 4 Alliance POIs
    //                          (Stormwind / Ironforge
    //                          / Goldshire / Sentinel
    //                          Hill) with discoverable
    //                          XP and POI-kind iconry.
    //   makeHerbalismNodes - 5 herb nodes (Peacebloom
    //                          / Silverleaf /
    //                          Briarthorn / Mageroyal
    //                          / Stranglekelp) with
    //                          required Herbalism skill
    //                          1..125 + 600s (10min)
    //                          respawn.
    //   makeRareSpawns     - 4 vanilla rare-elites
    //                          (Mor'Ladim / Princess
    //                          Tempestria / Foreman
    //                          Rigger / Lord Sakrasis)
    //                          with 1800..7200s (30-
    //                          120min) respawn.
    static WoweeWorldLocations makeAlliancePOIs(const std::string& catalogName);
    static WoweeWorldLocations makeHerbalismNodes(const std::string& catalogName);
    static WoweeWorldLocations makeRareSpawns(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
