// src/game/spline_packet.cpp
// Consolidated spline packet parsing - replaces 7 duplicated parsing locations.
// Ported from: world_packets.cpp, world_packets_entity.cpp, packet_parsers_classic.cpp,
//              packet_parsers_tbc.cpp, movement_handler.cpp.
#include "game/spline_packet.hpp"
#include "core/logger.hpp"
#include <cmath>
#include <string>

namespace wowee::game {

// ── Packed-delta decoding ───────────────────────────────────────

glm::vec3 decodePackedDelta(uint32_t packed, const glm::vec3& midpoint) {
    // 11-bit signed X, 11-bit signed Y, 10-bit signed Z
    // Scaled by 0.25, subtracted from midpoint
    int32_t sx = static_cast<int32_t>(packed & 0x7FF);
    if (sx & 0x400) sx |= static_cast<int32_t>(0xFFFFF800);   // sign-extend 11-bit

    int32_t sy = static_cast<int32_t>((packed >> 11) & 0x7FF);
    if (sy & 0x400) sy |= static_cast<int32_t>(0xFFFFF800);   // sign-extend 11-bit

    int32_t sz = static_cast<int32_t>((packed >> 22) & 0x3FF);
    if (sz & 0x200) sz |= static_cast<int32_t>(0xFFFFFC00);   // sign-extend 10-bit

    return glm::vec3(
        midpoint.x - static_cast<float>(sx) * 0.25f,
        midpoint.y - static_cast<float>(sy) * 0.25f,
        midpoint.z - static_cast<float>(sz) * 0.25f
    );
}

// ── MonsterMove spline body (post-splineFlags) ─────────────────

namespace {

/// The head every MonsterMove spline body starts with, whatever expansion
/// wrote it: an optional animation block, the duration, an optional parabolic
/// block, then the point count.
///
/// Written out twice, once in each of the two parsers below, and the only
/// difference between the copies was which bit means animation. That bit moved
/// in WotLK, from 0x00400000 to 0x00200000, so the caller passes the one its
/// generation uses; honouring the other reads five bytes that are not there
/// and shifts every field after it.
///
/// This is the order AzerothCore's WriteCommonMonsterMovePart writes.
///
/// Answers false only on a truncated packet. A point count of zero is a
/// complete body, not a failure: the server sends one for a spline that just
/// turns a creature on the spot.
bool readSplineBodyHead(network::Packet& packet, SplineBlockData& out,
                        uint32_t splineFlags, uint32_t animationBit,
                        uint32_t& outPointCount) {
    out.splineFlags = splineFlags;
    outPointCount = 0;

    // Animation: uint8 animType + uint32 animStartTime
    if (splineFlags & animationBit) {
        if (!packet.hasRemaining(5)) return false;
        out.hasAnimation = true;
        out.animationType = packet.readUInt8();
        out.animationStartTime = packet.readUInt32();
    }

    if (!packet.hasRemaining(4)) return false;
    out.duration = packet.readUInt32();

    // Parabolic: float vertAccel + uint32 startTime
    if (splineFlags & SplineFlag::PARABOLIC_MM) {
        if (!packet.hasRemaining(8)) return false;
        out.hasParabolic = true;
        out.verticalAcceleration = packet.readFloat();
        out.parabolicStartTime = packet.readUInt32();
    }

    if (!packet.hasRemaining(4)) return false;
    const uint32_t pointCount = packet.readUInt32();
    // A count read out of the wrong bytes is usually enormous, and reserving
    // for it is how a malformed packet becomes an allocation failure.
    if (pointCount > 1000) return false;
    outPointCount = pointCount;
    return true;
}

}  // namespace

bool parseMonsterMoveSplineBody(
    network::Packet& packet,
    SplineBlockData& out,
    uint32_t splineFlags,
    const glm::vec3& startPos,
    bool useTbcUncompressedMask,
    SplineFlagSet flagSet)
{
    const bool wotlk = (flagSet == SplineFlagSet::Wotlk);
    const uint32_t animationBit = wotlk ? SplineFlagWotlk::ANIMATION
                                        : SplineFlag::ANIMATION;

    uint32_t pointCount = 0;
    if (!readSplineBodyHead(packet, out, splineFlags, animationBit, pointCount)) return false;
    if (pointCount == 0) return true;

    // Determine compressed vs uncompressed
    uint32_t uncompMask = useTbcUncompressedMask
        ? SplineFlag::UNCOMPRESSED_MASK_TBC
        : (wotlk ? SplineFlagWotlk::UNCOMPRESSED_MASK
                 : SplineFlag::UNCOMPRESSED_MASK);
    bool uncompressed = (splineFlags & uncompMask) != 0;

    if (uncompressed) {
        // All waypoints as absolute float3, last one is destination
        for (uint32_t i = 0; i + 1 < pointCount; ++i) {
            if (!packet.hasRemaining(12)) return true; // Partial parse OK
            float wx = packet.readFloat();
            float wy = packet.readFloat();
            float wz = packet.readFloat();
            out.waypoints.emplace_back(wx, wy, wz);
        }
        if (!packet.hasRemaining(12)) return true;
        out.destination.x = packet.readFloat();
        out.destination.y = packet.readFloat();
        out.destination.z = packet.readFloat();
        out.hasDest = true;
    } else {
        // Compressed: first float3 is destination, rest are packed deltas from midpoint
        if (!packet.hasRemaining(12)) return true;
        out.destination.x = packet.readFloat();
        out.destination.y = packet.readFloat();
        out.destination.z = packet.readFloat();
        out.hasDest = true;

        if (pointCount > 1) {
            glm::vec3 mid = (startPos + out.destination) * 0.5f;
            for (uint32_t i = 0; i + 1 < pointCount; ++i) {
                if (!packet.hasRemaining(4)) break;
                uint32_t packed = packet.readUInt32();
                out.waypoints.push_back(decodePackedDelta(packed, mid));
            }
        }
    }

    return true;
}

// ── Vanilla MonsterMove spline body (always compressed) ─────────

bool parseMonsterMoveFacing(network::Packet& packet, MonsterMoveData& data, bool& stopped) {
    stopped = false;
    if (!packet.hasRemaining(1)) return false;
    data.moveType = packet.readUInt8();

    // Stop: the packet ends here, and where it stopped is where it already is.
    if (data.moveType == 1) {
        data.destX = data.x;
        data.destY = data.y;
        data.destZ = data.z;
        data.hasDest = false;
        stopped = true;
        return true;
    }

    if (data.moveType == 2) {
        // FacingSpot - a point to look at, which nothing here needs; read past.
        if (!packet.hasRemaining(12)) return false;
        packet.readFloat();
        packet.readFloat();
        packet.readFloat();
    } else if (data.moveType == 3) {
        // FacingTarget
        if (!packet.hasRemaining(8)) return false;
        data.facingTarget = packet.readUInt64();
    } else if (data.moveType == 4) {
        // FacingAngle
        if (!packet.hasRemaining(4)) return false;
        data.facingAngle = packet.readFloat();
    }
    return true;
}

bool parseMonsterMoveSplineBodyVanilla(
    network::Packet& packet,
    SplineBlockData& out,
    uint32_t splineFlags,
    const glm::vec3& startPos)
{
    uint32_t pointCount = 0;
    if (!readSplineBodyHead(packet, out, splineFlags, SplineFlag::ANIMATION, pointCount)) {
        return false;
    }
    if (pointCount == 0) return true;

    // Always compressed in Vanilla: dest (12 bytes) + packed deltas (4 bytes each)
    size_t requiredBytes = 12;
    if (pointCount > 1) requiredBytes += static_cast<size_t>(pointCount - 1) * 4ull;
    if (!packet.hasRemaining(requiredBytes)) return false;

    out.destination.x = packet.readFloat();
    out.destination.y = packet.readFloat();
    out.destination.z = packet.readFloat();
    out.hasDest = true;

    if (pointCount > 1) {
        glm::vec3 mid = (startPos + out.destination) * 0.5f;
        for (uint32_t i = 0; i + 1 < pointCount; ++i) {
            uint32_t packed = packet.readUInt32();
            out.waypoints.push_back(decodePackedDelta(packed, mid));
        }
    }

    return true;
}

// ── Classic/Turtle movement update spline block ─────────────────

bool parseClassicMoveUpdateSpline(
    network::Packet& packet,
    SplineBlockData& out)
{
    // splineFlags
    if (!packet.hasRemaining(4)) return false;
    out.splineFlags = packet.readUInt32();
    LOG_DEBUG("  [Classic] Spline: flags=0x", std::hex, out.splineFlags, std::dec);

    // FINAL_POINT / FINAL_TARGET / FINAL_ANGLE
    if (out.splineFlags & SplineFlag::FINAL_POINT) {
        if (!packet.hasRemaining(12)) return false;
        out.hasFinalPoint = true;
        out.finalPoint.x = packet.readFloat();
        out.finalPoint.y = packet.readFloat();
        out.finalPoint.z = packet.readFloat();
    } else if (out.splineFlags & SplineFlag::FINAL_TARGET) {
        if (!packet.hasRemaining(8)) return false;
        out.hasFinalTarget = true;
        out.finalTarget = packet.readUInt64();
    } else if (out.splineFlags & SplineFlag::FINAL_ANGLE) {
        if (!packet.hasRemaining(4)) return false;
        out.hasFinalAngle = true;
        out.finalAngle = packet.readFloat();
    }

    // timePassed + duration + splineId + pointCount = 16 bytes
    if (!packet.hasRemaining(16)) return false;
    out.timePassed = packet.readUInt32();
    out.duration = packet.readUInt32();
    out.splineId = packet.readUInt32();

    uint32_t pointCount = packet.readUInt32();
    if (pointCount > 256) return false;

    // All points uncompressed (12 bytes each) + endPoint (12 bytes)
    // Classic: NO splineMode byte
    if (!packet.hasRemaining(static_cast<size_t>(pointCount) * 12 + 12)) return false;
    for (uint32_t i = 0; i < pointCount; ++i) {
        float px = packet.readFloat();
        float py = packet.readFloat();
        float pz = packet.readFloat();
        out.waypoints.emplace_back(px, py, pz);
    }

    out.endPoint.x = packet.readFloat();
    out.endPoint.y = packet.readFloat();
    out.endPoint.z = packet.readFloat();
    out.hasEndPoint = true;

    return true;
}

// ── WotLK movement update spline block ──────────────────────────
// Complex multi-try parser for different server variations.

bool parseWotlkMoveUpdateSpline(
    network::Packet& packet,
    SplineBlockData& out,
    const glm::vec3& entityPos)
{
    auto bytesAvailable = [&](size_t n) -> bool { return packet.hasRemaining(n); };

    // splineFlags
    if (!bytesAvailable(4)) return false;
    out.splineFlags = packet.readUInt32();
    LOG_DEBUG("  Spline: flags=0x", std::hex, out.splineFlags, std::dec);

    // FINAL_POINT / FINAL_TARGET / FINAL_ANGLE
    if (out.splineFlags & SplineFlagWotlk::FINAL_POINT) {
        if (!bytesAvailable(12)) return false;
        out.hasFinalPoint = true;
        out.finalPoint.x = packet.readFloat();
        out.finalPoint.y = packet.readFloat();
        out.finalPoint.z = packet.readFloat();
    } else if (out.splineFlags & SplineFlagWotlk::FINAL_TARGET) {
        if (!bytesAvailable(8)) return false;
        out.hasFinalTarget = true;
        out.finalTarget = packet.readUInt64();
    } else if (out.splineFlags & SplineFlagWotlk::FINAL_ANGLE) {
        if (!bytesAvailable(4)) return false;
        out.hasFinalAngle = true;
        out.finalAngle = packet.readFloat();
    }

    // timePassed + duration + splineId
    if (!bytesAvailable(12)) return false;
    out.timePassed = packet.readUInt32();
    out.duration = packet.readUInt32();
    out.splineId = packet.readUInt32();

    // ── Helper: try to parse spline points + splineMode + endPoint ──
    // WotLK uses compressed points by default (first=12 bytes, rest=4 bytes packed).
    // Why the best-aligned attempt gave up. Every layout below can fail for a
    // different reason and the log only ever said that all of them did, which
    // is the one thing already known. The first attempt is the authoritative
    // AzerothCore layout, so its reason is the one worth keeping.
    std::string firstReason;
    auto note = [&](const std::string& why) {
        if (firstReason.empty()) firstReason = why;
    };

    auto tryParseSplinePoints = [&](bool compressed, const char* tag) -> bool {
        if (!bytesAvailable(4)) { note("no room for the node count"); return false; }
        size_t prePointCount = packet.getReadPos();
        uint32_t pc = packet.readUInt32();
        if (pc > 256) {
            note("node count " + std::to_string(pc) + " is beyond any real path");
            packet.setReadPos(prePointCount);
            return false;
        }
        // AzerothCore's WriteCreate always appends splineMode and endPoint,
        // even when the path contains no nodes.  Leaving those 13 bytes in
        // the packet desynchronizes the next UPDATE_OBJECT block.
        size_t pointsBytes;
        if (compressed && pc > 0) {
            // First point = 3 floats (12 bytes), rest = packed uint32 (4 bytes each)
            pointsBytes = 12ull + (pc > 1 ? static_cast<size_t>(pc - 1) * 4ull : 0ull);
        } else {
            // All uncompressed: 3 floats each
            pointsBytes = static_cast<size_t>(pc) * 12ull;
        }
        size_t needed = pointsBytes + 13ull; // + splineMode(1) + endPoint(12)
        if (!bytesAvailable(needed)) {
            note("needs " + std::to_string(needed) + " bytes for " +
                 std::to_string(pc) + " nodes, has " +
                 std::to_string(packet.getRemainingSize()));
            packet.setReadPos(prePointCount);
            return false;
        }
        packet.setReadPos(packet.getReadPos() + pointsBytes);
        uint8_t mode = packet.readUInt8();
        if (mode > 3) {
            note("nodes=" + std::to_string(pc) + " then mode byte " +
                 std::to_string(static_cast<int>(mode)) + ", which is not an "
                 "interpolation mode");
            packet.setReadPos(prePointCount);
            return false;
        }
        float epX = packet.readFloat();
        float epY = packet.readFloat();
        float epZ = packet.readFloat();
        // Validate endPoint: garbage bytes rarely produce finite world coords
        if (!std::isfinite(epX) || !std::isfinite(epY) || !std::isfinite(epZ) ||
            std::fabs(epX) > 65000.0f || std::fabs(epY) > 65000.0f ||
            std::fabs(epZ) > 65000.0f) {
            note("nodes=" + std::to_string(pc) + " then an end point of (" +
                 std::to_string(epX) + "," + std::to_string(epY) + "," +
                 std::to_string(epZ) + "), which is not a world coordinate");
            packet.setReadPos(prePointCount);
            return false;
        }
        // Proximity check: if entity position is known (not the default 0,0,0
        // sentinel), reject endpoints that are implausibly far from it.
        float posLenSq = entityPos.x * entityPos.x + entityPos.y * entityPos.y + entityPos.z * entityPos.z;
        if (posLenSq > 1.0f) {
            float dx = epX - entityPos.x;
            float dy = epY - entityPos.y;
            float dz = epZ - entityPos.z;
            float distSq = dx * dx + dy * dy + dz * dz;
            // Generous, because the longest legitimate spline is a flight
            // path and those cross a continent: the limit is here to reject
            // bytes that happen to decode as finite numbers, not to have an
            // opinion about how far a unit may travel. At five thousand it had
            // one, and a taxi is further than that.
            constexpr float kFarthestPlausible = 20000.0f;
            if (distSq > kFarthestPlausible * kFarthestPlausible) {
                note("nodes=" + std::to_string(pc) + " then an end point " +
                     std::to_string(static_cast<int>(std::sqrt(distSq))) +
                     " away, which is further than any spline runs");
                packet.setReadPos(prePointCount);
                return false;
            }
        }
        out.splineMode = mode;
        out.endPoint = glm::vec3(epX, epY, epZ);
        out.hasEndPoint = true;
        LOG_DEBUG("  Spline pointCount=", pc, " compressed=", compressed,
                  " endPt=(", epX, ",", epY, ",", epZ, ") (", tag, ")");
        return true;
    };

    // Save position before WotLK spline header for fallback
    size_t beforeSplineHeader = packet.getReadPos();

    // Try 1: WotLK format (durationMod+durationModNext+[ANIMATION]+vertAccel+effectStart+points)
    // Some servers (ChromieCraft) always write vertAccel+effectStart unconditionally.
    bool splineParsed = false;
    if (bytesAvailable(8)) {
        /*float durationMod =*/ packet.readFloat();
        /*float durationModNext =*/ packet.readFloat();
        bool wotlkOk = true;
        if (out.splineFlags & SplineFlagWotlk::ANIMATION) {
            if (!bytesAvailable(5)) { wotlkOk = false; }
            else {
                out.hasAnimation = true;
                out.animationType = packet.readUInt8();
                out.animationStartTime = packet.readUInt32();
            }
        }
        // Unconditional vertAccel+effectStart (ChromieCraft/some AzerothCore builds)
        if (wotlkOk) {
            if (!bytesAvailable(8)) { wotlkOk = false; }
            else {
                /*float vertAccel =*/ packet.readFloat();
                /*uint32_t effectStart =*/ packet.readUInt32();
            }
        }
        if (wotlkOk) {
            // AzerothCore WriteCreate serializes every path node as Vector3.
            // Try that authoritative layout first: a compressed interpretation
            // is shorter and can otherwise pass endpoint validation by chance,
            // leaving unread node bytes before the next object block.
            splineParsed = tryParseSplinePoints(false, "wotlk-uncompressed");
            if (!splineParsed) {
                bool useCompressed = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
                splineParsed = tryParseSplinePoints(useCompressed, "wotlk-compressed-fallback");
            }
        }
    }

    // Some WotLK cores serialize verticalAcceleration for every create spline,
    // but omit effectStartTime unless the spline is actually parabolic. Taxi
    // splines commonly use that layout. Treating the following pointCount as
    // effectStartTime shifts the path and rejects the whole player update.
    if (!splineParsed && !(out.splineFlags & SplineFlagWotlk::PARABOLIC)) {
        packet.setReadPos(beforeSplineHeader);
        if (bytesAvailable(12)) {
            packet.readFloat(); // durationMod
            packet.readFloat(); // durationModNext
            packet.readFloat(); // verticalAcceleration
            splineParsed = tryParseSplinePoints(false, "wotlk-vertical-no-effect-start");
            if (!splineParsed) {
                bool useCompressed = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
                splineParsed = tryParseSplinePoints(useCompressed,
                                                    "wotlk-vertical-no-effect-start-compressed");
            }
        }
    }

    // Try 2: ANIMATION present but vertAccel+effectStart gated by PARABOLIC
    if (!splineParsed && (out.splineFlags & SplineFlagWotlk::ANIMATION)) {
        packet.setReadPos(beforeSplineHeader);
        out.hasAnimation = false; // Reset from failed try
        if (bytesAvailable(8)) {
            packet.readFloat(); // durationMod
            packet.readFloat(); // durationModNext
            bool ok = true;
            if (!bytesAvailable(5)) { ok = false; }
            else {
                out.hasAnimation = true;
                out.animationType = packet.readUInt8();
                out.animationStartTime = packet.readUInt32();
            }
            if (ok && (out.splineFlags & SplineFlagWotlk::PARABOLIC)) {
                if (!bytesAvailable(8)) { ok = false; }
                else { packet.readFloat(); packet.readUInt32(); }
            }
            if (ok) {
                bool useCompressed = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
                splineParsed = tryParseSplinePoints(useCompressed, "wotlk-anim-conditional");
                if (!splineParsed) {
                    splineParsed = tryParseSplinePoints(false, "wotlk-anim-conditional-uncomp");
                }
            }
        }
    }

    // Try 3: No ANIMATION - vertAccel+effectStart only when PARABOLIC set
    if (!splineParsed) {
        packet.setReadPos(beforeSplineHeader);
        out.hasAnimation = false;
        if (bytesAvailable(8)) {
            packet.readFloat(); // durationMod
            packet.readFloat(); // durationModNext
            bool ok = true;
            if (out.splineFlags & SplineFlagWotlk::PARABOLIC) {
                if (!bytesAvailable(8)) { ok = false; }
                else { packet.readFloat(); packet.readUInt32(); }
            }
            if (ok) {
                bool useCompressed = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
                splineParsed = tryParseSplinePoints(useCompressed, "wotlk-parabolic-gated");
                if (!splineParsed) {
                    splineParsed = tryParseSplinePoints(false, "wotlk-parabolic-gated-uncomp");
                }
            }
        }
    }

    // Try 4: No header at all - just durationMod+durationModNext then points
    if (!splineParsed) {
        packet.setReadPos(beforeSplineHeader);
        if (bytesAvailable(8)) {
            packet.readFloat(); // durationMod
            packet.readFloat(); // durationModNext
            splineParsed = tryParseSplinePoints(false, "wotlk-no-parabolic");
            if (!splineParsed) {
                bool useComp = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
                splineParsed = tryParseSplinePoints(useComp, "wotlk-no-parabolic-compressed");
            }
        }
    }

    // Try 5: bare points (no WotLK header at all - some spline types skip everything)
    if (!splineParsed) {
        packet.setReadPos(beforeSplineHeader);
        splineParsed = tryParseSplinePoints(false, "bare-uncompressed");
        if (!splineParsed) {
            packet.setReadPos(beforeSplineHeader);
            bool useComp = (out.splineFlags & SplineFlagWotlk::UNCOMPRESSED_MASK) == 0;
            splineParsed = tryParseSplinePoints(useComp, "bare-compressed");
        }
    }

    if (!splineParsed) {
        // Dump first 5 uint32s at beforeSplineHeader for format diagnosis
        packet.setReadPos(beforeSplineHeader);
        uint32_t d[5] = {};
        for (int di = 0; di < 5 && packet.hasRemaining(4); ++di)
            d[di] = packet.readUInt32();
        packet.setReadPos(beforeSplineHeader);
        LOG_WARNING("WotLK spline parse failed"
                    " splineFlags=0x", std::hex, out.splineFlags, std::dec,
                    " remaining=", packet.getRemainingSize(),
                    " header=[0x", std::hex, d[0], " 0x", d[1], " 0x", d[2],
                    " 0x", d[3], " 0x", d[4], "]", std::dec,
                    " firstReason=", firstReason.empty() ? "(none recorded)" : firstReason);
        return false;
    }

    return true;
}

} // namespace wowee::game
