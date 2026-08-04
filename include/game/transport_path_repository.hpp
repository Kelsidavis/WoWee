// include/game/transport_path_repository.hpp
// Owns and manages transport path data — DBC, taxi, and custom paths.
// Uses CatmullRomSpline for spline evaluation (replaces duplicated evalTimedCatmullRom).
// Separated from TransportManager for SOLID-S (single responsibility).
#pragma once

#include "math/spline.hpp"
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>

namespace wowee::pipeline {
    class AssetManager;
}

namespace wowee::game {

/// Whether a displayId belongs to a vehicle that travels a route — a ship, a
/// zeppelin, an icebreaker, a gunship — as opposed to an elevator or a lift.
///
/// Both are transports and both animate, but they want opposite treatment: a
/// vehicle's route is long and mostly horizontal and often comes from the
/// server, while a lift travels a few metres straight up.
///
/// Named because the numbers were being written out at three call sites and
/// each list had picked up elevator displayIds along the way. 807 and 808 are
/// Gnomeregan's two lifts, 2454 the Searing Gorge scaffold cars, 1587 a
/// GameObject named "Elevator" — every one of them was being treated as a ship
/// or drawn as an airship. Values verified against gameobject_template: these
/// are the displayIds carried by type 15 (MO_TRANSPORT) rows.
constexpr bool isVehicleTransportDisplay(uint32_t displayId) {
    switch (displayId) {
        case 3015:  // ship
        case 3031:  // zeppelin
        case 6637:  // Naxxramas
        case 7087:  // night elf ship
        case 7446:  // icebreaker
        case 7546:  // horde zeppelin
        case 7570:  // Sister Mercy
        case 7636:  // turtle
        case 8253: case 8254:   // Orgrim's Hammer / The Skybreaker
        case 9001: case 9002:   // gunships
        case 9150: case 9151:   // Icecrown airships
            return true;
        default:
            return false;
    }
}

/// Vehicles whose route is a taxi path the server assigns, rather than a
/// TransportAnimation.dbc loop. Borrowing an animation path for one of these
/// sends it underwater, so they stay docked until the route arrives.
constexpr bool isOceanGoingTransportDisplay(uint32_t displayId) {
    switch (displayId) {
        case 3015:  // ship
        case 7087:  // night elf ship
        case 7446:  // icebreaker
        case 7636:  // turtle
            return true;
        default:
            return false;
    }
}

/// Metadata + CatmullRomSpline for a transport path.
struct PathEntry {
    math::CatmullRomSpline spline;
    uint32_t pathId = 0;
    bool zOnly = false;       // Elevator/bobbing — no meaningful XY travel
    bool fromDBC = false;     // Loaded from TransportAnimation.dbc
    bool worldCoords = false; // TaxiPathNode absolute world positions (not local offsets)

    PathEntry(math::CatmullRomSpline s, uint32_t id, bool zo, bool dbc, bool wc)
        : spline(std::move(s)), pathId(id), zOnly(zo), fromDBC(dbc), worldCoords(wc) {}
};

/// Owns and manages transport path data.
class TransportPathRepository {
public:
    TransportPathRepository() = default;

    // ── DBC loading ─────────────────────────────────────────
    bool loadTransportAnimationDBC(pipeline::AssetManager* assetMgr);
    bool loadTaxiPathNodeDBC(pipeline::AssetManager* assetMgr);

    // ── Path construction ───────────────────────────────────
    void loadPathFromNodes(uint32_t pathId, const std::vector<glm::vec3>& waypoints,
                           bool looping = true, float speed = 18.0f);

    // ── Lookup ──────────────────────────────────────────────
    const PathEntry* findPath(uint32_t pathId) const;
    // Taxi paths are stored per-map: a continent-crossing boat path has nodes on
    // two maps, and only the segment on the transport's current map is valid world
    // geometry. mapId selects that segment.
    const PathEntry* findTaxiPath(uint32_t taxiPathId, uint32_t mapId) const;
    bool hasPathForEntry(uint32_t entry) const;
    bool hasTaxiPath(uint32_t taxiPathId) const;              // exists on any map
    bool hasTaxiPathForMap(uint32_t taxiPathId, uint32_t mapId) const;

    // ── Query ───────────────────────────────────────────────
    bool hasUsableMovingPathForEntry(uint32_t entry, float minXYRange = 1.0f) const;
    uint32_t inferDbcPathForSpawn(const glm::vec3& spawnWorldPos, float maxDistance,
                                  bool allowZOnly) const;
    uint32_t inferMovingPathForSpawn(const glm::vec3& spawnWorldPos,
                                     float maxDistance = 1200.0f) const;
    uint32_t pickFallbackMovingPath(uint32_t entry, uint32_t displayId) const;

    // ── Mutation ─────────────────────────────────────────────
    /// Store or overwrite a path entry (used by assignTaxiPathToTransport).
    void storePath(uint32_t pathId, PathEntry entry);

    // Build the cyclic CatmullRomSpline for one map's slice of a TaxiPathNode route.
    //
    // The transport clock samples the result modulo its duration, so the slice must be
    // position-closed (end where it began) or the hull teleport-snaps at the wrap. Two
    // topologies:
    //   - closed loop (endpoints coincide): append the first point to close the ring;
    //   - open route: append the outbound points in reverse so the slice is one continuous
    //     there-and-back ferry (offshore-in -> dock -> offshore-out -> U-turn -> back).
    // A slice is NEVER made to hold stationary offshore for the time the boat "spends" on
    // another continent: a rider aboard experiences that as the boat sitting dead at sea.
    // The cross-continent handoff is the server's SMSG_NEW_WORLD teleport at the offshore
    // node, independent of this client animation. pts are canonical world positions in
    // NodeIndex order; nodeDelaysMs is the authored per-node dock dwell in milliseconds
    // (0 where none), matched by index to pts. Static and dependency-free so the wrap
    // behaviour can be unit-tested without a DBC.
    // fullRouteCycleMs is the whole route's period across every map it touches,
    // in the same terms this function accounts in (legs twice, dwells once).
    // Anything left over after this slice's own cost is spent waiting at the
    // pier, so the boat makes one departure per server cycle rather than lapping
    // its shore until the transfer comes due. 0 disables the stretch.
    static math::CatmullRomSpline buildTaxiSegmentSpline(
        const std::vector<glm::vec3>& pts,
        const std::vector<uint32_t>& nodeDelaysMs,
        float transportSpeed = 28.0f,
        uint32_t fullRouteCycleMs = 0);

private:
    std::unordered_map<uint32_t, PathEntry> paths_;
    // taxiPathId -> mapId -> world-coordinate path segment for that map.
    std::unordered_map<uint32_t, std::unordered_map<uint32_t, PathEntry>> taxiPaths_;
};

} // namespace wowee::game
