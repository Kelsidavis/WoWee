#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Taxi catalog (.wtax) - novel replacement for
// Blizzard's TaxiNodes.dbc + TaxiPath.dbc + TaxiPathNode.dbc.
// The 24th open format added to the editor.
//
// Defines the flight-master network: a set of named nodes
// (positions on the world map) plus the paths between them
// (sequences of waypoints with per-segment delay and a
// per-path gold cost). The same file holds both node and
// path lists - flat arrays keyed by id.
//
// Cross-references with previously-added formats:
//   WCRT.entry (with FlightMaster npcFlag) ≈ WTAX.entry.nodeId
//                                            (matched by world
//                                             position, not by
//                                             direct ID - the
//                                             flight-master NPC
//                                             stands at the node)
//   WTAX.path.fromNodeId / toNodeId → WTAX.entry.nodeId
//                                     (intra-format graph)
//
// Binary layout (little-endian):
//   magic[4]            = "WTAX"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   nodeCount (uint32)
//   nodes (each):
//     nodeId (uint32)
//     mapId (uint32)
//     nameLen + name
//     iconLen + iconPath
//     position (3 × float)
//     factionAlliance (uint32) / factionHorde (uint32)
//   pathCount (uint32)
//   paths (each):
//     pathId (uint32)
//     fromNodeId (uint32) / toNodeId (uint32)
//     moneyCostCopper (uint32)
//     waypointCount (uint32)
//     waypoints (waypointCount × {
//       position (3 × float)
//       delaySec (float)
//     })
struct WoweeTaxi {
    struct Node {
        uint32_t nodeId = 0;
        uint32_t mapId = 0;
        std::string name;
        std::string iconPath;
        glm::vec3 position{0};
        uint32_t factionAlliance = 0;       // 0 = available to all
        uint32_t factionHorde = 0;
    };

    struct Waypoint {
        glm::vec3 position{0};
        float delaySec = 0.0f;        // pause at this waypoint
    };

    struct Path {
        uint32_t pathId = 0;
        uint32_t fromNodeId = 0;
        uint32_t toNodeId = 0;
        uint32_t moneyCostCopper = 0;
        std::vector<Waypoint> waypoints;
    };

    std::string name;
    std::vector<Node> nodes;
    std::vector<Path> paths;

    [[nodiscard]] bool isValid() const { return !nodes.empty(); }

    // Lookup helpers.
    [[nodiscard]] const Node* findNode(uint32_t nodeId) const;
    [[nodiscard]] const Path* findPath(uint32_t pathId) const;
    // First path matching a from→to pair, or nullptr.
    [[nodiscard]] const Path* findPathBetween(uint32_t fromNodeId, uint32_t toNodeId) const;
};

class WoweeTaxiLoader {
public:
    static bool save(const WoweeTaxi& cat,
                     const std::string& basePath);
    static WoweeTaxi load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-taxi* variants.
    //
    //   makeStarter - 2 nodes + 1 path (round-trip 2 cities,
    //                  3 waypoints, 50 silver each way).
    //   makeRegion  - 4 nodes around a square (~500m apart) +
    //                  4 paths forming a connected ring
    //                  (each path is 2 waypoints).
    //   makeContinent - 6 nodes + 8 paths covering a small
    //                    continent's flight network with
    //                    cross-route shortcuts.
    static WoweeTaxi makeStarter(const std::string& catalogName);
    static WoweeTaxi makeRegion(const std::string& catalogName);
    static WoweeTaxi makeContinent(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
