// src/game/transport_animator.cpp
// Path evaluation, Z clamping, and orientation for transports.
// Extracted from TransportManager::updateTransportMovement (Phase 3b of spline refactoring).
#include "game/transport_animator.hpp"
#include "game/transport_manager.hpp"
#include "game/transport_path_repository.hpp"
#include "math/spline.hpp"
#include "core/logger.hpp"
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace wowee::game {

namespace {

// GO entries whose TaxiPath berth runs parallel to the pier: the ship holds a broadside
// (side-on) heading through its dock dwell instead of its bow-first approach yaw. Kept
// entry-scoped on purpose — this is route geometry, not a model trait, so an unrelated
// ship that happens to reuse one of these display ids keeps its own docking orientation.
bool berthRunsParallel(uint32_t entry) {
    return entry == 176310u ||  // The Bravery — Stormwind Harbor
           entry == 176244u ||  // The Moonspray — Auberdine
           entry == 181646u;    // Elune's Blessing — Auberdine
}

}  // namespace

void TransportAnimator::evaluateAndApply(
    ActiveTransport& transport,
    const PathEntry& pathEntry,
    uint32_t pathTimeMs) const
{
    const auto& spline = pathEntry.spline;

    // Evaluate position from time via CatmullRomSpline (path is local offsets, add base position)
    glm::vec3 pathOffset = spline.evaluatePosition(pathTimeMs);

    // Catmull-Rom splines aren't constrained to the convex hull of their control
    // points. TransportAnimation.dbc uses sparse keyframes near each Deeprun Tram
    // station, and live position polling showed the evaluated path visibly
    // overshooting past the authored stop keyframe before correcting back to it
    // (observed ~12 units of dip-and-recover on final approach) - reported live
    // as cars not quite lining up with the platform ramps. Clamp X/Y to the
    // authored keyframe extents (in raw, pre-mirror spline space) to remove the
    // overshoot without touching the keyframe values themselves. Z has its own
    // clampZOffset() below for the same underlying spline behavior.
    bool xyClamped = false;
    // Populated for tram entries below; also decides the 176085 mirror.
    glm::vec3 keyMin(std::numeric_limits<float>::max());
    glm::vec3 keyMax(std::numeric_limits<float>::lowest());
    if (TransportManager::isDeeprunTramTransport(transport) && !spline.keys().empty()) {
        for (const auto& key : spline.keys()) {
            keyMin = glm::min(keyMin, key.position);
            keyMax = glm::max(keyMax, key.position);
        }
        const float clampedX = std::clamp(pathOffset.x, keyMin.x, keyMax.x);
        const float clampedY = std::clamp(pathOffset.y, keyMin.y, keyMax.y);
        xyClamped = (clampedX != pathOffset.x) || (clampedY != pathOffset.y);
        pathOffset.x = clampedX;
        pathOffset.y = clampedY;
    }

    // Entry 176085's TransportAnimation.dbc path data is mirrored relative to its real
    // train siblings (176080, 176081): diagnostic dump of all six entries' raw key
    // extents showed 176080/176081 spanning local X=[0,+2482] (real-world X correctly
    // increasing toward Stormwind, matching their spawn near Ironforge), while 176085
    // spans local X=[-2482,0] - the same negative-direction convention as the genuinely
    // Stormwind-side cars (176082-176084, correct for THEM since their real-world X
    // needs to decrease toward Ironforge). 176085 spawns at real-world X=-11 (Ironforge
    // side) but its path pulls it further NEGATIVE instead of toward Stormwind's
    // positive coordinates, driving it off the edge of the modeled tunnel entirely -
    // "took me outside of map bounds and back" reported live, on the one car
    // consistently observed going the wrong way. Y is negligible for all six paths
    // (confirmed ~0 in the same dump), so a straight X negation is enough to mirror
    // this one entry's local path back into the same real-world direction as its
    // siblings, without needing a general per-transport reverse/mirror flag for what's
    // so far a single-entry data quirk.
    // Only mirror when this entry's data actually uses the negative-X
    // convention (vanilla/TBC). WotLK's re-export gives 176085 the same
    // positive-X frame as its siblings (after the loader's Y-major rotation),
    // so an unconditional entry check would drive it off-tunnel there.
    const bool tramMirroredData = keyMin.x < -100.0f && keyMax.x < 100.0f;
    if (transport.entry == 176085u && tramMirroredData) {
        pathOffset.x = -pathOffset.x;
    }

    pathOffset.z = clampZOffset(
        pathOffset.z,
        pathEntry.worldCoords,
        transport.useClientAnimation,
        transport.serverUpdateCount,
        transport.hasServerClock);

    transport.position = transport.basePosition + pathOffset;

    // A TaxiPath route encodes a dock wait as two keys at the same position with
    // time between them. Hold the authored position for that stop and blend the
    // heading over the five seconds either side of it.
    //
    // This used to run only for the three entries in berthRunsParallel, because
    // it was written for their broadside berths. But the position hold is not a
    // berth-specific nicety: a Catmull-Rom spline is not constrained to the hull
    // of its control points, so evaluating through a repeated key overshoots and
    // recovers — the ship sails past its dock and comes back, repeatedly, for
    // the whole length of the wait. That is the same overshoot already
    // documented and clamped for the tram, and it applies to every ship.
    // berthRunsParallel now decides only what it is about: the heading.
    float shipDockBlend = 0.0f;
    glm::vec3 shipApproach(0.0f);
    bool shipAtDockDwell = false;
    glm::vec3 shipDockPosition(0.0f);
    const bool needsSideOnDock = berthRunsParallel(transport.entry);
    if (pathEntry.worldCoords && !transport.isM2) {
        constexpr uint32_t kDockTurnMs = 5000u;
        const auto& keys = spline.keys();
        for (size_t i = 1; i + 1 < keys.size(); ++i) {
            const glm::vec3 dwellDelta = keys[i].position - keys[i + 1].position;
            if (glm::dot(dwellDelta, dwellDelta) > 0.01f) continue;
            const uint32_t dwellStart = keys[i].timeMs;
            const uint32_t dwellEnd = keys[i + 1].timeMs;
            const uint32_t turnStart = dwellStart > kDockTurnMs
                ? dwellStart - kDockTurnMs : 0u;
            if (pathTimeMs < turnStart || pathTimeMs > dwellEnd + kDockTurnMs) {
                continue;
            }

            // A berth heading is the direction the hull lies while alongside, and
            // a route generally turns as it passes through its dock: the Maiden's
            // Fancy comes into Menethil on a bearing 26 degrees off the one it
            // leaves on. Taking the arrival leg alone therefore parked the hull
            // half that turn out — 13 degrees, which over a hundred-unit hull is
            // enough to walk the gangway off the plank. Use the chord through the
            // berth, from the node before the stop to the node after it, which is
            // the line the boat is lying on rather than either end of the turn.
            const glm::vec3 berthFrom = keys[i - 1].position;
            const glm::vec3 berthTo =
                (i + 2 < keys.size()) ? keys[i + 2].position : keys[i].position;
            shipApproach = berthTo - berthFrom;
            shipApproach.z = 0.0f;
            float approachLen = glm::length(shipApproach);
            if (approachLen <= 0.001f) {
                // Nothing after the stop (route ends here): fall back to arrival.
                shipApproach = keys[i].position - keys[i - 1].position;
                shipApproach.z = 0.0f;
                approachLen = glm::length(shipApproach);
            }
            if (approachLen <= 0.001f) break;
            shipApproach /= approachLen;

            if (pathTimeMs < dwellStart) {
                shipDockBlend = static_cast<float>(
                    pathTimeMs - turnStart) / static_cast<float>(kDockTurnMs);
            } else if (pathTimeMs <= dwellEnd) {
                shipDockBlend = 1.0f;
                shipAtDockDwell = true;
                shipDockPosition = keys[i].position;
            } else {
                shipDockBlend = 1.0f - static_cast<float>(
                    pathTimeMs - dwellEnd) / static_cast<float>(kDockTurnMs);
            }
            shipDockBlend = std::clamp(shipDockBlend, 0.0f, 1.0f);
            shipDockBlend = shipDockBlend * shipDockBlend *
                            (3.0f - 2.0f * shipDockBlend);
            break;
        }
    }
    // Record the stop so the hull's machinery can follow it. A paddlewheel
    // turning while the ship sits at the pier is what happens otherwise.
    transport.atDockDwell = shipAtDockDwell;
    if (shipAtDockDwell) {
        // Catmull-Rom evaluation can sit slightly off a repeated-position key
        // even during its authored hold. Pin the actual dwell to the TaxiPath
        // node so the gangway does not retain a visible one-unit gap.
        transport.position = transport.basePosition + shipDockPosition;
    }

    // Server yaw is authoritative only while the server is also driving position.
    //
    // hasServerYaw is set by every server update, including the ones that arrive
    // for a ship the client animates itself. Taking it here pinned such a ship's
    // facing to whichever orientation the server last reported — its berth
    // heading — and held it there for the whole voyage while the position ran
    // along the route underneath. That is a ship sailing sideways or stern-first
    // and lying across its pier on arrival, and it also made everything below
    // (route yaw, the bow offset, the broadside dock hold) unreachable for any
    // transport the server had ever mentioned.
    //
    // When the client owns the animation the route tangent is what facing has to
    // follow, because the client owns the phase the server's snapshot knows
    // nothing about.
    if (pathEntry.zOnly) {
        // A lift travels straight up and down, so its tangent is vertical and
        // carries no heading at all: atan2 of nothing is zero, and
        // orientationFromTangent falls into its near-vertical branch and picks
        // an arbitrary basis. Both were being used, which is why an elevator
        // stood at right angles to the shaft it runs in. It keeps the yaw it
        // was placed at, which is the only heading it ever has.
        transport.rotation =
            glm::angleAxis(transport.spawnYaw, glm::vec3(0.0f, 0.0f, 1.0f));
    } else if (transport.hasServerYaw && !transport.useClientAnimation) {
        float effectiveYaw = transport.serverYaw +
            (transport.serverYawFlipped180 ? glm::pi<float>() : 0.0f);
        transport.rotation = glm::angleAxis(effectiveYaw, glm::vec3(0.0f, 0.0f, 1.0f));
    } else if (xyClamped) {
        // The tangent below comes from a separate, unclamped evaluation of the same
        // spline - it doesn't know the position is currently pinned at the keyframe
        // boundary above. Near the station, the raw (unclamped) path loops through a
        // little overshoot-and-recover S-curve, and its tangent sweeps through whatever
        // instantaneous directions that loop produces - including briefly near-
        // perpendicular to the direction of travel - while the rendered position sits
        // still. Reported live as cars "turning sideways for a brief period before
        // leaving the station." Leave transport.rotation at its last computed value
        // while the position is clamped; there's no real facing change to reflect since
        // the car isn't actually moving from the rider's point of view during that
        // window.
    } else {
        auto result = spline.evaluate(pathTimeMs);
        glm::vec3 tangent = result.tangent;
        // Mirror the tangent's X to match the position mirror above, so facing
        // direction stays consistent with this entry's (corrected) direction of travel.
        if (transport.entry == 176085u && tramMirroredData) {
            tangent.x = -tangent.x;
        }
        // orientationFromTangent orients along the full 3D tangent, pitching/banking to
        // match vertical slope - correct for something like a boat cresting swells, but
        // a subway car shouldn't nose-dive on a downhill grade the way this made it look
        // ("angling downwards instead of staying flat" reported live). Flattening
        // tangent.z to 0 fixed that, but a subsequent live test reported the level car
        // visually clipping into the sloped tunnel floor on grade changes, so that was
        // changed to a clamped partial pitch instead of a full flatten. Confirmed via a
        // later live comparison against the real game client that this was the wrong
        // trade-off: in the real client, tram cars stay level (parallel to the ground)
        // through elevation changes - any clamped tilt is visibly wrong regardless of
        // whether it also reduces clipping. Reverted to a full flatten to match the
        // real client's confirmed behavior; other transports keep full tangent-based
        // orientation.
        if (TransportManager::isDeeprunTramTransport(transport)) {
            tangent.z = 0.0f;
        }
        const float tangentLenSq = glm::dot(tangent, tangent);
        if (tangentLenSq <= 1e-6f && shipDockBlend > 0.0f) {
            tangent = shipApproach;
        }
        const float effectiveTangentLenSq = glm::dot(tangent, tangent);
        if (effectiveTangentLenSq > 1e-6f) {
            if (pathEntry.worldCoords && !transport.isM2) {
                // TaxiPathNode coordinates were converted server -> canonical by
                // swapping X/Y. WMO transport models face server-space +X, so derive
                // the same yaw the server would send from the canonical tangent.
                // The generic spline helper uses a different local-forward convention
                // and mirrored ship yaw, producing sideways/backwards sailing.
                // Facing = direction of travel + the hull's bow offset. Every
                // transport hull in the data is authored bow-at--X, so that
                // offset is PI for all of them; see
                // TransportManager::transportModelBowOffset for the measurements.
                float routeYaw = std::atan2(tangent.x, tangent.y) +
                                 TransportManager::transportModelBowOffset(transport.displayId);
                // A GO query reports the transport's orientation at the instant it is
                // received, not a persistent heading for every berth, so using that
                // snapshot as the dock yaw made the result depend on where the ship
                // happened to be when the player logged in. berthRunsParallel routes run
                // parallel to their piers, so the corrected route heading is also the
                // stable broadside dock heading — the dock dwell holds routeYaw directly.
                const float effectiveYaw = routeYaw;
                transport.rotation = glm::angleAxis(
                    effectiveYaw, glm::vec3(0.0f, 0.0f, 1.0f));
                if (shipDockBlend > 0.999f && needsSideOnDock) {
                    static std::unordered_set<uint64_t> loggedDockGuids;
                    if (loggedDockGuids.insert(transport.guid).second) {
                        LOG_DEBUG("SHIP DOCK DIAG entry=", transport.entry,
                                    " guid=0x", std::hex, transport.guid, std::dec,
                                    " pathTime=", pathTimeMs,
                                    " position=(", transport.position.x, ",",
                                    transport.position.y, ",", transport.position.z, ")",
                                    " tangent=(", tangent.x, ",", tangent.y, ")",
                                    " routeYaw=", routeYaw,
                                    " hasDockYaw=", transport.hasDockYaw,
                                    " dockYaw=", transport.dockYaw,
                                    " effectiveYaw=", effectiveYaw);
                    }
                }
            } else {
                transport.rotation = math::CatmullRomSpline::orientationFromTangent(tangent);
            }
        }
        // A TaxiPathNode route encodes a dock wait as repeated positions, so the
        // tangent vanishes and there is no heading to derive. The ship keeps its
        // corrected arrival rotation through the dwell.
        //
        // This used to restore the GO's authored spawn orientation instead, for
        // hulls whose bow offset was zero. That spawn yaw is a snapshot from
        // whenever the GO query happened to answer rather than a heading for the
        // berth, and restoring it made a ship swing round for the stop and back
        // again on departure. Now that the offset is PI for every hull the
        // condition could not fire at all, so the branch is gone rather than
        // left sitting there looking live.
    }
}

float TransportAnimator::clampZOffset(float z, bool worldCoords, bool clientAnim,
                                       int serverUpdateCount, bool hasServerClock)
{
    // Skip Z clamping for world-coordinate paths (TaxiPathNode) where values are absolute positions.
    if (worldCoords) return z;

    constexpr float kMinFallbackZOffset = -2.0f;
    constexpr float kMaxFallbackZOffset =  8.0f;

    // Clamp fallback Z offsets for non-world-coordinate paths to prevent transport
    // models from sinking below sea level on paths derived only from spawn-time data
    // (notably icebreaker routes where the DBC path has steep vertical curves).
    if (clientAnim && serverUpdateCount <= 1) {
        z = std::max(z, kMinFallbackZOffset);
    }
    if (!clientAnim && !hasServerClock) {
        z = std::clamp(z, kMinFallbackZOffset, kMaxFallbackZOffset);
    }
    return z;
}

} // namespace wowee::game
