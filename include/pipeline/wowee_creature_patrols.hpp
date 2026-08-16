#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Patrol Path catalog (.wcmr) -
// novel replacement for AzerothCore's creature_movement /
// waypoints SQL tables plus the per-spawn waypoint
// arrays. Defines named waypoint paths that creatures
// patrol along: Stormwind guards walking the city perimeter,
// AQ40 trash rotating through the chamber, ICC patrols
// circling the spire.
//
// Each entry binds a creatureGuid (server-side spawn id)
// to a sequence of (x, y, z, delayMs) waypoints. The
// pathKind controls cycling behavior (Loop / OneShot /
// Reverse / Random) and moveType controls the locomotion
// kind (Walk / Run / Fly / Swim) - a flying patrol
// ignores ground geometry, a swimming patrol stays
// underwater.
//
// Variable-length entries: each path stores its own
// waypointCount with a corresponding inline waypoint
// array (no fixed cap). Loaders advance through the file
// by reading the count first, then count*16 bytes of
// waypoint data.
//
// Cross-references with previously-added formats:
//   WCRT: creatureGuid references the spawned creature
//         instance whose AI follows this patrol.
//
// Binary layout (little-endian):
//   magic[4]            = "WCMR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     pathId (uint32)
//     nameLen + name
//     descLen + description
//     creatureGuid (uint32)
//     pathKind (uint8) / moveType (uint8) / pad[2]
//     waypointCount (uint32)
//     waypoints[count] (each 16 bytes):
//       x (float) / y (float) / z (float) / delayMs (uint32)
//     iconColorRGBA (uint32)
struct WoweeCreaturePatrol {
    enum PathKind : uint8_t {
        Loop      = 0,    // patrol forever in a circle
        OneShot   = 1,    // walk once, then idle at last waypoint
        Reverse   = 2,    // walk to end, walk back, repeat
        Random    = 3,    // pick next waypoint randomly each step
    };

    enum MoveType : uint8_t {
        Walk      = 0,    // ground walking speed
        Run       = 1,    // ground run speed
        Fly       = 2,    // airborne movement (ignores terrain)
        Swim      = 3,    // underwater movement
    };

    struct Waypoint {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        uint32_t delayMs = 0;    // dwell time at this waypoint
    };

    struct Entry {
        uint32_t pathId = 0;
        std::string name;
        std::string description;
        uint32_t creatureGuid = 0;
        uint8_t pathKind = Loop;
        uint8_t moveType = Walk;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        std::vector<Waypoint> waypoints;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t pathId) const;
    [[nodiscard]] const Entry* findByCreatureGuid(uint32_t creatureGuid) const;

    // Compute total path length in yards by summing
    // segment distances between consecutive waypoints.
    // For Loop kind, includes the closing segment back to
    // the first waypoint.
    [[nodiscard]] float pathLengthYards(uint32_t pathId) const;

    static const char* pathKindName(uint8_t k);
    static const char* moveTypeName(uint8_t m);
};

class WoweeCreaturePatrolLoader {
public:
    static bool save(const WoweeCreaturePatrol& cat,
                     const std::string& basePath);
    static WoweeCreaturePatrol load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cmr* variants.
    //
    //   makePatrol - 3 small patrols showing each pathKind
    //                 (4-pt Loop guard, 6-pt OneShot run,
    //                 8-pt Random tiger).
    //   makeCity   - 4 capital-city guard routes (Stormwind /
    //                 Orgrimmar / Ironforge / Thunder Bluff)
    //                 with 6-8 waypoints each.
    //   makeBoss   - 3 raid-zone patrols (AQ40 12-pt Loop /
    //                 Naxx 8-pt OneShot / ICC 16-pt Random)
    //                 demonstrating long-path support.
    static WoweeCreaturePatrol makePatrol(const std::string& catalogName);
    static WoweeCreaturePatrol makeCity(const std::string& catalogName);
    static WoweeCreaturePatrol makeBoss(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
