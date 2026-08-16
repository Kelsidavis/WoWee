#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Transit Schedule catalog (.wtsc) -
// novel replacement for the implicit
// taxi/zeppelin/boat scheduling that vanilla WoW
// drove from a tangle of TaxiNodes.dbc +
// TaxiPath.dbc + per-zeppelin GameObject scripts +
// hard-coded transport interval timers in the
// server's MapManager. Each WTRN entry binds one
// scheduled passenger route to its origin /
// destination coordinates, vehicle type
// (Taxi/Zeppelin/Boat/Mount), departure interval,
// in-flight duration, capacity, and faction-access
// gate.
//
// Cross-references with previously-added formats:
//   WMS:   originMapId / destinationMapId reference
//          the WMS map catalog.
//   WTAX:  vehicleType=Taxi routes are derived from
//          (and extend) WTAX taxi-node catalog -
//          WTRN adds the scheduling layer that WTAX
//          lacked.
//
// Binary layout (little-endian):
//   magic[4]            = "WTSC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     routeId (uint32)
//     nameLen + name
//     vehicleType (uint8)        - 0=Taxi /
//                                   1=Zeppelin /
//                                   2=Boat /
//                                   3=Mount
//     factionAccess (uint8)      - 0=Both /
//                                   1=Alliance /
//                                   2=Horde /
//                                   3=Neutral
//     pad0 (uint16)
//     originLen + originName
//     originX (float)
//     originY (float)
//     originMapId (uint32)
//     destinationLen + destinationName
//     destinationX (float)
//     destinationY (float)
//     destinationMapId (uint32)
//     departureIntervalSec (uint32)  - period between
//                                       successive
//                                       departures from
//                                       origin
//     travelDurationSec (uint32)     - in-flight time
//                                       origin->dest
//     capacity (uint16)              - max simultaneous
//                                       riders (0 =
//                                       unlimited, e.g.
//                                       solo gryphon)
//     pad1 (uint16)
struct WoweeTransitSchedule {
    enum VehicleType : uint8_t {
        Taxi     = 0,
        Zeppelin = 1,
        Boat     = 2,
        Mount    = 3,    // hired riding-mount
                          // (e.g., kodo
                          //  caravan in vanilla
                          //  Barrens)
    };

    enum FactionAccess : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
        Neutral  = 3,    // Booty Bay-style
                          //  cross-faction routes
    };

    struct Entry {
        uint32_t routeId = 0;
        std::string name;
        uint8_t vehicleType = Taxi;
        uint8_t factionAccess = Both;
        uint16_t pad0 = 0;
        std::string originName;
        float originX = 0.f;
        float originY = 0.f;
        uint32_t originMapId = 0;
        std::string destinationName;
        float destinationX = 0.f;
        float destinationY = 0.f;
        uint32_t destinationMapId = 0;
        uint32_t departureIntervalSec = 0;
        uint32_t travelDurationSec = 0;
        uint16_t capacity = 0;
        uint16_t pad1 = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t routeId) const;

    // Returns all routes accessible by a given faction
    // mask (Alliance/Horde/Neutral all see Both routes;
    // a faction-specific call also includes that
    // faction's exclusive routes).
    [[nodiscard]] std::vector<const Entry*> findAccessibleByFaction(
        uint8_t faction) const;

    // Returns all routes departing from a given
    // origin map. Used by the boat-dock UI to
    // populate the "next departure" widget.
    [[nodiscard]] std::vector<const Entry*> findDeparturesFromMap(
        uint32_t mapId) const;
};

class WoweeTransitScheduleLoader {
public:
    static bool save(const WoweeTransitSchedule& cat,
                     const std::string& basePath);
    static WoweeTransitSchedule load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-trn* variants.
    //
    //   makeZeppelins - 3 vanilla zeppelin routes
    //                    (Orgrimmar<->Undercity 240s
    //                    interval / OG<->Grom'gol /
    //                    UC<->Grom'gol). All Horde-only.
    //   makeBoats     - 3 vanilla boat routes
    //                    (Auberdine<->Stormwind /
    //                    Menethil<->Theramore /
    //                    BootyBay<->Ratchet - last is
    //                    Neutral, both factions can
    //                    board).
    //   makeTaxis     - 3 taxi gryphon/wyvern routes
    //                    (Stormwind<->Ironforge
    //                    Alliance / Crossroads<->
    //                    Razor Hill Horde /
    //                    Booty Bay<->Stormwind
    //                    Neutral).
    static WoweeTransitSchedule makeZeppelins(const std::string& catalogName);
    static WoweeTransitSchedule makeBoats(const std::string& catalogName);
    static WoweeTransitSchedule makeTaxis(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
