// src/game/transport_path_repository.cpp
// Owns and manages transport path data — DBC, taxi, and custom paths.
// Ported from TransportManager (path management subset).
#include "game/transport_path_repository.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>

namespace wowee::game {

namespace {

bool isDeeprunTramPath(uint32_t transportEntry) {
    return transportEntry >= 176080u && transportEntry <= 176085u;
}

glm::vec3 transportAnimationOffsetToCanonical(uint32_t transportEntry, const glm::vec3& pos) {
    if (isDeeprunTramPath(transportEntry)) {
        // Deeprun's TransportAnimation rows are local subway-car offsets, not
        // server/world coordinates. Raw X is the tunnel travel axis; swapping it
        // through serverToCanonical makes the cars drive perpendicular to the rails.
        return glm::vec3(pos.x, pos.y, pos.z);
    }

    // TransportAnimation.dbc local offsets use a coordinate system where the travel
    // axis is negated relative to server world coords for ship/zeppelin-style paths.
    return core::coords::serverToCanonical(glm::vec3(-pos.x, -pos.y, pos.z));
}

} // namespace

// ── Simple lookup methods ──────────────────────────────────────

const PathEntry* TransportPathRepository::findPath(uint32_t pathId) const {
    auto it = paths_.find(pathId);
    return it != paths_.end() ? &it->second : nullptr;
}

const PathEntry* TransportPathRepository::findTaxiPath(uint32_t taxiPathId, uint32_t mapId) const {
    auto it = taxiPaths_.find(taxiPathId);
    if (it == taxiPaths_.end()) return nullptr;
    auto mIt = it->second.find(mapId);
    return mIt != it->second.end() ? &mIt->second : nullptr;
}

bool TransportPathRepository::hasPathForEntry(uint32_t entry) const {
    auto* e = findPath(entry);
    return e != nullptr && e->fromDBC;
}

bool TransportPathRepository::hasTaxiPath(uint32_t taxiPathId) const {
    auto it = taxiPaths_.find(taxiPathId);
    return it != taxiPaths_.end() && !it->second.empty();
}

bool TransportPathRepository::hasTaxiPathForMap(uint32_t taxiPathId, uint32_t mapId) const {
    return findTaxiPath(taxiPathId, mapId) != nullptr;
}

void TransportPathRepository::storePath(uint32_t pathId, PathEntry entry) {
    auto it = paths_.find(pathId);
    if (it != paths_.end()) {
        it->second = std::move(entry);
    } else {
        paths_.emplace(pathId, std::move(entry));
    }
}

// ── Query methods ──────────────────────────────────────────────

bool TransportPathRepository::hasUsableMovingPathForEntry(uint32_t entry, float minXYRange) const {
    auto* e = findPath(entry);
    if (!e) return false;
    if (!e->fromDBC || e->spline.keyCount() < 2 || e->spline.durationMs() == 0 || e->zOnly) {
        return false;
    }
    return e->spline.hasXYMovement(minXYRange);
}

uint32_t TransportPathRepository::inferDbcPathForSpawn(const glm::vec3& spawnWorldPos,
                                                        float maxDistance,
                                                        bool allowZOnly) const {
    float bestD2 = maxDistance * maxDistance;
    uint32_t bestPathId = 0;

    for (const auto& [pathId, entry] : paths_) {
        if (!entry.fromDBC || entry.spline.durationMs() == 0 || entry.spline.keyCount() == 0) {
            continue;
        }
        if (!allowZOnly && entry.zOnly) {
            continue;
        }

        // Find nearest waypoint on this path to spawn
        size_t nearIdx = entry.spline.findNearestKey(spawnWorldPos);
        glm::vec3 diff = entry.spline.keys()[nearIdx].position - spawnWorldPos;
        float d2 = glm::dot(diff, diff);
        if (d2 < bestD2) {
            bestD2 = d2;
            bestPathId = pathId;
        }
    }

    if (bestPathId != 0) {
        LOG_INFO("TransportPathRepository: Inferred DBC path ", bestPathId,
                 " (allowZOnly=", allowZOnly ? "yes" : "no",
                 ") for spawn at (", spawnWorldPos.x, ", ", spawnWorldPos.y, ", ", spawnWorldPos.z,
                 "), dist=", std::sqrt(bestD2));
    }

    return bestPathId;
}

uint32_t TransportPathRepository::inferMovingPathForSpawn(const glm::vec3& spawnWorldPos, float maxDistance) const {
    return inferDbcPathForSpawn(spawnWorldPos, maxDistance, /*allowZOnly=*/false);
}

uint32_t TransportPathRepository::pickFallbackMovingPath(uint32_t entry, uint32_t displayId) const {
    auto isUsableMovingPath = [this](uint32_t pathId) -> bool {
        auto* e = findPath(pathId);
        if (!e) return false;
        return e->fromDBC && !e->zOnly && e->spline.durationMs() > 0 && e->spline.keyCount() > 1;
    };

    // Known AzerothCore transport entry remaps (WotLK): server entry -> moving DBC path id.
    // Zeppelins are client-animated and map to real zeppelin TransportAnimation paths.
    // The continent-crossing SHIPS (Maiden's Fancy, Bravery, Black Princess, and the
    // icebreakers) were previously remapped to Deeprun Tram paths (176080-176085) here
    // — a straight ~2482-unit underground line — which sailed them underwater to
    // nowhere. Those ships are server-driven MO_TRANSPORT objects with no
    // TransportAnimation.dbc entry; they get their route from their taxi path
    // (GO template data[0] -> TaxiPathNode.dbc), assigned by the GO-query hook. So
    // they are intentionally NOT listed here — a ship with no real/taxi path stays
    // docked rather than borrowing an unrelated route (see the looksLikeShip guard below).
    static const std::unordered_map<uint32_t, uint32_t> kEntryRemap = {
        {164871u, 193182u}, // The Thundercaller (zeppelin)
        {176495u, 193183u}, // The Purple Princess (zeppelin)
        {175080u, 193182u}, // The Iron Eagle (zeppelin)
        {181689u, 193183u}, // Cloudkisser (zeppelin)
        {186238u, 193182u}, // The Mighty Wind (zeppelin)
    };

    auto itMapped = kEntryRemap.find(entry);
    if (itMapped != kEntryRemap.end() && isUsableMovingPath(itMapped->second)) {
        return itMapped->second;
    }

    if (displayId == 3831u) {
        static constexpr uint32_t kDeeprunTramCandidates[] = {
            176080u, 176081u, 176082u, 176083u, 176084u, 176085u
        };
        for (uint32_t id : kDeeprunTramCandidates) {
            if (id == entry && isUsableMovingPath(id)) return id;
        }
        for (uint32_t id : kDeeprunTramCandidates) {
            if (isUsableMovingPath(id)) {
                LOG_WARNING("TransportPathRepository: remapped Deeprun tram displayId 3831 entry ",
                            entry, " to DBC path ", id);
                return id;
            }
        }
    }

    // Fallback by display model family.
    const bool looksLikeShip = isOceanGoingTransportDisplay(displayId);
    if (looksLikeShip) {
        // Continent-crossing ships are server-driven MO_TRANSPORT objects: their
        // route comes from their taxi path (TaxiPathNode.dbc via GO template data[0]),
        // not from TransportAnimation.dbc. There is no correct TransportAnimation
        // fallback for them — borrowing any (previously the Deeprun Tram paths) sent
        // them underwater. Return 0 so the ship stays docked until the GO-query hook
        // assigns its taxi path; never fabricate a path from an unrelated route.
        return 0;
    }

    // Zeppelins used to fall back to the first usable id in a candidate list,
    // which is one route — so every zeppelin flew it. Three of them are already
    // remapped to 193182 above, and the display families cover five more ids, so
    // in practice the whole fleet traced the same line regardless of where it
    // spawned or where it was meant to go.
    //
    // The reasoning already written down for ships applies just as well here: a
    // transport whose route is not known stays docked. It is the same object the
    // server carries the player on, so an invented route does not merely look
    // wrong, it disagrees with where the server says the player is.
    // No last-resort "any moving path" either. Handing a transport an unrelated
    // route is what sailed the continent ships underwater, and picking the first
    // entry of an unordered map made it arbitrary as well as wrong.
    return 0;
}

// ── Path construction from waypoints ───────────────────────────

void TransportPathRepository::loadPathFromNodes(uint32_t pathId, const std::vector<glm::vec3>& waypoints, bool looping, float speed) {
    if (waypoints.empty()) {
        LOG_ERROR("TransportPathRepository: Cannot load empty path ", pathId);
        return;
    }

    bool isZOnly = false;  // Manually loaded paths are assumed to have XY movement

    // Helper: compute segment duration from distance and speed
    auto segMsFromDist = [&](float dist) -> uint32_t {
        if (speed <= 0.0f) return 1000;
        return static_cast<uint32_t>((dist / speed) * 1000.0f);
    };

    // Single point = stationary (durationMs = 0)
    if (waypoints.size() == 1) {
        std::vector<math::SplineKey> keys;
        keys.push_back({0, waypoints[0]});
        math::CatmullRomSpline spline(std::move(keys), false);
        // Runtime fallbacks may replace stale runtime/taxi copies left under the
        // same entry, but must never overwrite an authentic expansion DBC path.
        const auto* existing = findPath(pathId);
        if (!existing || !existing->fromDBC) {
            storePath(pathId, PathEntry(std::move(spline), pathId, isZOnly, false, false));
        }
        LOG_INFO("TransportPathRepository: Loaded stationary path ", pathId);
        return;
    }

    // Multiple points: calculate cumulative time based on distance and speed
    std::vector<math::SplineKey> keys;
    keys.reserve(waypoints.size() + (looping ? 1 : 0));
    uint32_t cumulativeMs = 0;
    keys.push_back({0, waypoints[0]});

    for (size_t i = 1; i < waypoints.size(); i++) {
        float dist = glm::distance(waypoints[i-1], waypoints[i]);
        cumulativeMs += glm::max(1u, segMsFromDist(dist));
        keys.push_back({cumulativeMs, waypoints[i]});
    }

    // Add explicit wrap segment (last → first) for looping paths.
    // By duplicating the first point at the end with cumulative time, the path
    // becomes time-closed and CatmullRomSpline handles wrap via modular time
    // without requiring special-case index wrapping during evaluation.
    if (looping && waypoints.size() >= 2) {
        float wrapDist = glm::distance(waypoints.back(), waypoints.front());
        cumulativeMs += glm::max(1u, segMsFromDist(wrapDist));
        keys.push_back({cumulativeMs, waypoints[0]});
    }

    math::CatmullRomSpline spline(std::move(keys), false);
    // Runtime fallbacks may replace stale runtime/taxi copies left under the
    // same entry, but must never overwrite an authentic expansion DBC path.
    const auto* existing = findPath(pathId);
    if (!existing || !existing->fromDBC) {
        storePath(pathId, PathEntry(std::move(spline), pathId, isZOnly, false, false));
    }

    auto* stored = findPath(pathId);
    LOG_INFO("TransportPathRepository: Loaded path ", pathId,
             " with ", waypoints.size(), " waypoints",
             (looping ? " + wrap segment" : ""),
             ", duration=", stored ? stored->spline.durationMs() : 0, "ms, speed=", speed);
}

// ── DBC: TransportAnimation ────────────────────────────────────

bool TransportPathRepository::loadTransportAnimationDBC(pipeline::AssetManager* assetMgr) {
    LOG_INFO("Loading TransportAnimation.dbc...");

    if (!assetMgr) {
        LOG_ERROR("AssetManager is null");
        return false;
    }

    // Load DBC file
    auto dbcData = assetMgr->readFile("DBFilesClient\\TransportAnimation.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("TransportAnimation.dbc not found - transports will use fallback paths");
        return false;
    }

    pipeline::DBCFile dbc;
    if (!dbc.load(dbcData)) {
        LOG_ERROR("Failed to parse TransportAnimation.dbc");
        return false;
    }

    LOG_INFO("TransportAnimation.dbc: ", dbc.getRecordCount(), " records, ",
             dbc.getFieldCount(), " fields per record");

    // Debug: dump first 3 records to see all field values
    for (uint32_t i = 0; i < std::min(3u, dbc.getRecordCount()); i++) {
        LOG_INFO("  DEBUG Record ", i, ": ",
                 " [0]=", dbc.getUInt32(i, 0),
                 " [1]=", dbc.getUInt32(i, 1),
                 " [2]=", dbc.getUInt32(i, 2),
                 " [3]=", dbc.getFloat(i, 3),
                 " [4]=", dbc.getFloat(i, 4),
                 " [5]=", dbc.getFloat(i, 5),
                 " [6]=", dbc.getUInt32(i, 6));
    }

    // Group waypoints by transportEntry
    std::map<uint32_t, std::vector<std::pair<uint32_t, glm::vec3>>> waypointsByTransport;

    for (uint32_t i = 0; i < dbc.getRecordCount(); i++) {
        // uint32_t id = dbc.getUInt32(i, 0);  // Not needed
        uint32_t transportEntry = dbc.getUInt32(i, 1);
        uint32_t timeIndex = dbc.getUInt32(i, 2);
        float posX = dbc.getFloat(i, 3);
        float posY = dbc.getFloat(i, 4);
        float posZ = dbc.getFloat(i, 5);
        // uint32_t sequenceId = dbc.getUInt32(i, 6);  // Not needed for basic paths

        // RAW FLOAT SANITY CHECK: Log first 10 records to see if DBC has real data
        if (i < 10) {
            uint32_t ux = dbc.getUInt32(i, 3);
            uint32_t uy = dbc.getUInt32(i, 4);
            uint32_t uz = dbc.getUInt32(i, 5);
            LOG_INFO("TA raw rec ", i,
                     " entry=", transportEntry,
                     " t=", timeIndex,
                     " raw=(", posX, ",", posY, ",", posZ, ")",
                     " u32=(", ux, ",", uy, ",", uz, ")");
        }

        // DIAGNOSTIC: Log ALL records for problematic ferries (20655, 20657, 149046)
        // AND first few records for known-good transports to verify DBC reading
        if (i < 5 || transportEntry == 2074 ||
            transportEntry == 20655 || transportEntry == 20657 || transportEntry == 149046) {
            LOG_INFO("RAW DBC [", i, "] entry=", transportEntry, " t=", timeIndex,
                     " raw=(", posX, ",", posY, ",", posZ, ")");
        }

        waypointsByTransport[transportEntry].push_back({timeIndex, glm::vec3(posX, posY, posZ)});
    }

    // Create time-indexed paths from waypoints
    int pathsLoaded = 0;
    for (const auto& [transportEntry, waypoints] : waypointsByTransport) {
        if (waypoints.empty()) continue;

        // Sort by timeIndex
        auto sortedWaypoints = waypoints;
        std::sort(sortedWaypoints.begin(), sortedWaypoints.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // CRITICAL: Normalize timeIndex to start at 0 (DBC records don't start at 0!)
        // This makes evaluatePosition(0) valid and stabilizes basePosition seeding
        uint32_t t0 = sortedWaypoints.front().first;

        // Build SplineKey array with normalized time indices
        std::vector<math::SplineKey> keys;
        keys.reserve(sortedWaypoints.size() + 1);  // +1 for wrap point

        // Log DBC waypoints for tram entries
        if (transportEntry >= 176080 && transportEntry <= 176085) {
            size_t mid = sortedWaypoints.size() / 4;  // ~quarter through
            size_t mid2 = sortedWaypoints.size() / 2; // ~halfway
            LOG_DEBUG("DBC path entry=", transportEntry, " nPts=", sortedWaypoints.size(),
                       " [0] t=", sortedWaypoints[0].first, " raw=(", sortedWaypoints[0].second.x, ",", sortedWaypoints[0].second.y, ",", sortedWaypoints[0].second.z, ")",
                       " [", mid, "] t=", sortedWaypoints[mid].first, " raw=(", sortedWaypoints[mid].second.x, ",", sortedWaypoints[mid].second.y, ",", sortedWaypoints[mid].second.z, ")",
                       " [", mid2, "] t=", sortedWaypoints[mid2].first, " raw=(", sortedWaypoints[mid2].second.x, ",", sortedWaypoints[mid2].second.y, ",", sortedWaypoints[mid2].second.z, ")");
        }

        // Deeprun tram frame normalization (data-driven, not expansion-gated):
        // vanilla/TBC author the tram's local path along +X ([0, +2482]), which
        // the identity mapping below was tuned against. WotLK re-exported the
        // same paths along -Y ([-2482, 0], X≈0), which made the cars drive
        // perpendicular to the tunnel. Detect the travel axis from the data
        // extents and rotate the Y-major variant into the X-major frame.
        bool tramYMajor = false;
        if (isDeeprunTramPath(transportEntry)) {
            glm::vec3 mn = sortedWaypoints.front().second;
            glm::vec3 mx = mn;
            for (const auto& wp : sortedWaypoints) {
                mn = glm::min(mn, wp.second);
                mx = glm::max(mx, wp.second);
            }
            tramYMajor = (mx.y - mn.y) > (mx.x - mn.x);
            if (tramYMajor) {
                LOG_INFO("Tram entry ", transportEntry,
                         " uses Y-major local frame (WotLK export) — rotating to X-major");
            }
        }
        auto tramNormalize = [tramYMajor](const glm::vec3& p) {
            return tramYMajor ? glm::vec3(-p.y, p.x, p.z) : p;
        };

        for (size_t idx = 0; idx < sortedWaypoints.size(); idx++) {
            const auto& [tMs, rawPos] = sortedWaypoints[idx];
            const glm::vec3 pos = tramNormalize(rawPos);

            glm::vec3 canonical = transportAnimationOffsetToCanonical(transportEntry, pos);

            // Skip waypoints where serverToCanonical zeroes nonzero inputs
            if ((pos.x != 0.0f || pos.y != 0.0f || pos.z != 0.0f) &&
                (canonical.x == 0.0f && canonical.y == 0.0f && canonical.z == 0.0f)) {
                LOG_ERROR("serverToCanonical ZEROED — skipping waypoint! entry=", transportEntry,
                          " server=(", pos.x, ",", pos.y, ",", pos.z, ")",
                          " → canon=(", canonical.x, ",", canonical.y, ",", canonical.z, ")");
                continue;
            }

            // Debug waypoint conversion for first transport (entry 2074)
            if (transportEntry == 2074 && idx < 5) {
                LOG_INFO("COORD CONVERT: entry=", transportEntry, " t=", tMs,
                         " serverPos=(", pos.x, ", ", pos.y, ", ", pos.z, ")",
                         " → canonical=(", canonical.x, ", ", canonical.y, ", ", canonical.z, ")");
            }

            // DIAGNOSTIC: Log ALL conversions for problematic ferries
            if (transportEntry == 20655 || transportEntry == 20657 || transportEntry == 149046) {
                LOG_INFO("CONVERT ", transportEntry, " t=", tMs,
                         " server=(", pos.x, ",", pos.y, ",", pos.z, ")",
                         " → canon=(", canonical.x, ",", canonical.y, ",", canonical.z, ")");
            }

            keys.push_back({tMs - t0, canonical});  // Normalize: subtract first timeIndex
        }

        // Get base duration from last normalized timeIndex
        uint32_t lastTimeMs = sortedWaypoints.back().first - t0;

        // Calculate wrap duration (last → first segment)
        // Use average segment duration as wrap duration
        uint32_t totalDelta = 0;
        int segmentCount = 0;
        for (size_t i = 1; i < sortedWaypoints.size(); i++) {
            uint32_t delta = sortedWaypoints[i].first - sortedWaypoints[i-1].first;
            if (delta > 0) {
                totalDelta += delta;
                segmentCount++;
            }
        }
        uint32_t wrapMs = (segmentCount > 0) ? (totalDelta / segmentCount) : 1000;

        // Add duplicate first point at end with wrap duration
        // This makes the wrap segment (last → first) have proper duration
        const glm::vec3 fp = tramNormalize(sortedWaypoints.front().second);
        glm::vec3 firstCanonical = transportAnimationOffsetToCanonical(transportEntry, fp);
        keys.push_back({lastTimeMs + wrapMs, firstCanonical});

        // Build the spline (time-closed=false because we added explicit wrap point)
        math::CatmullRomSpline spline(std::move(keys), false);

        // Detect Z-only paths (elevator/bobbing animation, not real XY travel)
        const auto& sk = spline.keys();
        float minX = sk[0].position.x, maxX = minX;
        float minY = sk[0].position.y, maxY = minY;
        float minZ = sk[0].position.z, maxZ = minZ;
        for (const auto& k : sk) {
            minX = std::min(minX, k.position.x); maxX = std::max(maxX, k.position.x);
            minY = std::min(minY, k.position.y); maxY = std::max(maxY, k.position.y);
            minZ = std::min(minZ, k.position.z); maxZ = std::max(maxZ, k.position.z);
        }
        float rangeX = maxX - minX;
        float rangeY = maxY - minY;
        float rangeZ = maxZ - minZ;
        float rangeXY = std::max(rangeX, rangeY);
        // Some elevator paths have tiny XY jitter. Treat them as z-only when horizontal travel
        // is negligible compared to vertical motion.
        bool isZOnly = (rangeXY < 0.01f) || (rangeXY < 1.0f && rangeZ > 2.0f);

        // Log first, middle, and last points to verify path data
        glm::vec3 firstOffset = sk[0].position;
        size_t midIdx = sk.size() / 2;
        glm::vec3 midOffset = sk[midIdx].position;
        glm::vec3 lastOffset = sk[sk.size() - 2].position;  // -2 to skip wrap duplicate
        uint32_t durationMs = spline.durationMs();
        LOG_INFO("  Transport ", transportEntry, ": ", sk.size() - 1, " waypoints + wrap, ",
                 durationMs, "ms duration (wrap=", wrapMs, "ms, t0_normalized=", sk[0].timeMs, "ms)",
                 " rangeXY=(", rangeX, ",", rangeY, ") rangeZ=", rangeZ, " ",
                 (isZOnly ? "[Z-ONLY]" : "[XY-PATH]"),
                 " firstOffset=(", firstOffset.x, ", ", firstOffset.y, ", ", firstOffset.z, ")",
                 " midOffset=(", midOffset.x, ", ", midOffset.y, ", ", midOffset.z, ")",
                 " lastOffset=(", lastOffset.x, ", ", lastOffset.y, ", ", lastOffset.z, ")");

        // Store path
        paths_.emplace(transportEntry, PathEntry(std::move(spline), transportEntry, isZOnly, true, false));
        pathsLoaded++;
    }

    LOG_INFO("Loaded ", pathsLoaded, " transport paths from TransportAnimation.dbc");
    return pathsLoaded > 0;
}

// ── DBC: TaxiPathNode ──────────────────────────────────────────

math::CatmullRomSpline TransportPathRepository::buildTaxiSegmentSpline(
    const std::vector<glm::vec3>& pts,
    const std::vector<uint32_t>& nodeDelaysMs,
    float transportSpeed,
    uint32_t fullRouteCycleMs)
{
    auto legMs = [transportSpeed](float dist) {
        return std::max<uint32_t>(100u, static_cast<uint32_t>(dist / transportSpeed * 1000.0f));
    };

    std::vector<math::SplineKey> keys;
    if (pts.size() < 2) {
        if (!pts.empty()) keys.push_back({0u, pts.front()});
        return math::CatmullRomSpline(std::move(keys), false);
    }

    const float endGap = glm::distance(pts.front(), pts.back());
    const bool closedLoop = endGap < 60.0f;

    // What this slice costs on its own, in the same terms the caller measured the
    // whole route in: legs travelled twice for the there-and-back, dwells once.
    uint32_t sliceLegMs = 0, sliceDelayMs = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        sliceDelayMs += (i < nodeDelaysMs.size()) ? nodeDelaysMs[i] : 0u;
        if (i + 1 < pts.size()) sliceLegMs += legMs(glm::distance(pts[i], pts[i + 1]));
    }
    const uint32_t sliceCycleMs = sliceLegMs * 2u + sliceDelayMs;

    // The rest of the route belongs to the other map, and the boat has to account
    // for that time somewhere or it simply laps this shore while the server's
    // schedule catches up. It waits at the pier: visibly stopped, still boardable,
    // and never adrift offshore. The pier is the node with the longest authored
    // dwell, which is what a dwell in TaxiPathNode means.
    uint32_t pierSurplusMs = 0;
    size_t pierIndex = 0;
    if (fullRouteCycleMs > sliceCycleMs) {
        pierSurplusMs = fullRouteCycleMs - sliceCycleMs;
        uint32_t longestDwell = 0;
        for (size_t i = 0; i < pts.size() && i < nodeDelaysMs.size(); ++i) {
            if (nodeDelaysMs[i] > longestDwell) {
                longestDwell = nodeDelaysMs[i];
                pierIndex = i;
            }
        }
        // No authored dwell anywhere: hold at the node furthest from either end,
        // which for an offshore-in/dock/offshore-out slice is the dock.
        if (longestDwell == 0) pierIndex = pts.size() / 2;
    }

    // Forward pass through the authored nodes, holding each node's dock dwell as a
    // repeated-position key (CatmullRomSpline evaluates those as exact stops).
    keys.reserve(pts.size() * 2 + 1);
    uint32_t cumulativeMs = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        keys.push_back({cumulativeMs, pts[i]});
        uint32_t delayMs = (i < nodeDelaysMs.size()) ? nodeDelaysMs[i] : 0u;
        if (i == pierIndex) delayMs += pierSurplusMs;
        if (delayMs != 0) {
            cumulativeMs += delayMs;
            keys.push_back({cumulativeMs, pts[i]});
        }
        if (i + 1 < pts.size()) {
            cumulativeMs += legMs(glm::distance(pts[i], pts[i + 1]));
        }
    }

    if (closedLoop) {
        // Endpoints already coincide: close the ring back to the first node.
        cumulativeMs += legMs(endGap);
        keys.push_back({cumulativeMs, pts.front()});
    } else {
        // Open route: the boat oscillates. Append the outbound points in reverse so the
        // cycle is one continuous there-and-back that is position-closed (ends where it
        // began), so the modulo phase wrap is seamless. This covers both a single-map
        // harbour shuttle AND one map's slice of a continent route (nodes run
        // offshore-in -> dock -> offshore-out): the hull sails out, U-turns at the far
        // node, and sails back through its dock. It must NEVER hold stationary offshore
        // for the time it "spends" on the other continent — a rider aboard experiences
        // that hold as the boat sitting dead at sea. The real cross-continent handoff is
        // the server's SMSG_NEW_WORLD teleport at the offshore node, independent of this
        // client animation; the boat just keeps ferrying while it waits for passengers.
        for (size_t i = pts.size() - 1; i-- > 0; ) {
            cumulativeMs += legMs(glm::distance(pts[i + 1], pts[i]));
            keys.push_back({cumulativeMs, pts[i]});
        }
    }

    return math::CatmullRomSpline(std::move(keys), false);
}

bool TransportPathRepository::loadTaxiPathNodeDBC(pipeline::AssetManager* assetMgr) {
    LOG_INFO("Loading TaxiPathNode.dbc...");

    if (!assetMgr) {
        LOG_ERROR("AssetManager is null");
        return false;
    }

    auto dbcData = assetMgr->readFile("DBFilesClient\\TaxiPathNode.dbc");
    if (dbcData.empty()) {
        LOG_WARNING("TaxiPathNode.dbc not found - MO_TRANSPORT will use fallback paths");
        return false;
    }

    pipeline::DBCFile dbc;
    if (!dbc.load(dbcData)) {
        LOG_ERROR("Failed to parse TaxiPathNode.dbc");
        return false;
    }

    LOG_INFO("TaxiPathNode.dbc: ", dbc.getRecordCount(), " records, ",
             dbc.getFieldCount(), " fields per record");

    // Group nodes by (PathID, MapID), storing (NodeIndex, X, Y, Z).
    // Paths are split per map: a continent-crossing boat path (e.g. Menethil ->
    // Valgarde) has nodes on two maps, but only the segment on the transport's
    // current map is valid world geometry. The old loader skipped any multi-map
    // path entirely (to filter flight-master routes), which dropped every
    // cross-continent boat. Instead, keep each map's segment separately and let
    // the assignment pick the one matching the transport's map.
    struct TaxiNode {
        uint32_t nodeIndex;
        float x, y, z;
        uint32_t delaySeconds;
    };
    std::map<std::pair<uint32_t, uint32_t>, std::vector<TaxiNode>> nodesByPathMap;

    for (uint32_t i = 0; i < dbc.getRecordCount(); i++) {
        uint32_t pathId = dbc.getUInt32(i, 1);    // PathID
        uint32_t nodeIdx = dbc.getUInt32(i, 2);   // NodeIndex
        uint32_t mapId = dbc.getUInt32(i, 3);     // MapID
        float posX = dbc.getFloat(i, 4);          // X (server coords)
        float posY = dbc.getFloat(i, 5);          // Y (server coords)
        float posZ = dbc.getFloat(i, 6);          // Z (server coords)
        uint32_t delaySeconds = dbc.getUInt32(i, 8); // Dock dwell time

        nodesByPathMap[{pathId, mapId}].push_back({nodeIdx, posX, posY, posZ, delaySeconds});
    }

    for (auto& [key, nodes] : nodesByPathMap) {
        std::sort(nodes.begin(), nodes.end(),
                  [](const TaxiNode& a, const TaxiNode& b) { return a.nodeIndex < b.nodeIndex; });
    }

    constexpr float transportSpeed = 28.0f;  // units per second

    auto legMsFor = [](float dist) {
        return std::max<uint32_t>(100u, static_cast<uint32_t>(dist / transportSpeed * 1000.0f));
    };

    // A cross-continent route is split into one slice per map, and each slice is
    // animated on its own. Sized from its own nodes alone, a slice's cycle is far
    // shorter than the server's, so the boat completed several round trips of the
    // Borean shore before the server's transfer came due — reported live as the
    // Kraken doing a couple of circuits instead of leaving.
    //
    // Measure the whole route first, across every map it touches, so a slice can
    // stretch its cycle to match. Same units as buildTaxiSegmentSpline's own
    // accounting: the legs are travelled twice per there-and-back, the authored
    // dwells once.
    std::unordered_map<uint32_t, uint32_t> routeCycleMs;
    for (const auto& [key, nodes] : nodesByPathMap) {
        if (nodes.size() < 2) continue;
        uint32_t legs = 0, delays = 0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            delays += nodes[i].delaySeconds * 1000u;
            if (i + 1 < nodes.size()) {
                legs += legMsFor(glm::distance(
                    glm::vec3(nodes[i].x, nodes[i].y, nodes[i].z),
                    glm::vec3(nodes[i + 1].x, nodes[i + 1].y, nodes[i + 1].z)));
            }
        }
        routeCycleMs[key.first] += legs * 2u + delays;
    }

    // Build world-coordinate transport paths, one segment per (pathId, mapId).
    int pathsLoaded = 0;
    for (auto& [key, nodes] : nodesByPathMap) {
        const uint32_t pathId = key.first;
        const uint32_t mapId = key.second;
        if (nodes.size() < 2) continue;

        // Convert nodes to canonical world positions and collect their authored dwells.
        std::vector<glm::vec3> pts;
        std::vector<uint32_t> nodeDelaysMs;
        pts.reserve(nodes.size());
        nodeDelaysMs.reserve(nodes.size());
        for (const auto& node : nodes) {
            pts.push_back(core::coords::serverToCanonical(glm::vec3(node.x, node.y, node.z)));
            nodeDelaysMs.push_back(node.delaySeconds * 1000u);
        }

        // Each map's slice is animated independently as a continuous there-and-back
        // ferry (see buildTaxiSegmentSpline). The cross-continent handoff is the server's
        // SMSG_NEW_WORLD teleport at the offshore node, not this client animation, so a
        // slice must never park offshore waiting on the other map — that reads as the boat
        // sitting dead at sea to anyone aboard. The surplus is spent at the pier instead,
        // where the boat is both plainly waiting and still boardable.
        const auto cycleIt = routeCycleMs.find(pathId);
        const uint32_t fullCycleMs = (cycleIt != routeCycleMs.end()) ? cycleIt->second : 0u;
        math::CatmullRomSpline spline = buildTaxiSegmentSpline(
            pts, nodeDelaysMs, transportSpeed, fullCycleMs);
        // TaxiPathNode is a separate, per-map route source. Do not label it as
        // TransportAnimation DBC data: copied taxi segments live temporarily in
        // paths_ for the active transport, and fromDBC=true made later inference
        // offer Bravery's cached route to unrelated icebreakers after zoning.
        taxiPaths_[pathId].emplace(mapId, PathEntry(std::move(spline), pathId, false, false, true));
        pathsLoaded++;
    }

    LOG_INFO("Loaded ", pathsLoaded, " TaxiPathNode transport path segments (",
             taxiPaths_.size(), " distinct taxi paths across all maps)");
    return pathsLoaded > 0;
}

} // namespace wowee::game
