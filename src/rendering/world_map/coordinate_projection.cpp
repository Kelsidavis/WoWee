// coordinate_projection.cpp - Pure coordinate math for world map UV projection.
// Extracted from WorldMap::renderPosToMapUV, findBestContinentForPlayer,
// findZoneForPlayer, zoneBelongsToContinent, getContinentProjectionBounds,
// isRootContinent, isLeafContinent (Phase 2 of refactoring plan).
#include "rendering/world_map/coordinate_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {
namespace world_map {

// ── Continent classification helpers ─────────────────────────

bool isRootContinent(const std::vector<Zone>& zones, int idx) {
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return false;
    const auto& c = zones[idx];
    if (c.areaID != 0 || c.wmaID == 0) return false;
    for (const auto& z : zones) {
        if (z.areaID == 0 && z.parentWorldMapID == c.wmaID) {
            return true;
        }
    }
    return false;
}

bool isLeafContinent(const std::vector<Zone>& zones, int idx) {
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return false;
    const auto& c = zones[idx];
    if (c.areaID != 0) return false;
    return c.parentWorldMapID != 0;
}

// ── UV projection ────────────────────────────────────────────

glm::vec2 renderPosToMapUV(const glm::vec3& renderPos,
                            const ZoneBounds& bounds,
                            bool isContinent) {
    float wowX = renderPos.y;
    float wowY = renderPos.x;

    float denom_h = bounds.locLeft - bounds.locRight;
    float denom_v = bounds.locTop - bounds.locBottom;
    if (std::abs(denom_h) < 0.001f || std::abs(denom_v) < 0.001f)
        return glm::vec2(0.5f, 0.5f);

    float u = (bounds.locLeft - wowX) / denom_h;
    float v = (bounds.locTop  - wowY) / denom_v;

    (void)isContinent;
    return glm::vec2(u, v);
}

// ── Continent projection bounds ──────────────────────────────

ZoneBounds projectionBoundsFor(const std::vector<Zone>& zones, int zoneIdx,
                               bool& isContinent) {
    isContinent = false;
    if (zoneIdx < 0 || static_cast<size_t>(zoneIdx) >= zones.size()) return {};

    const Zone& zone = zones[static_cast<size_t>(zoneIdx)];
    ZoneBounds bounds = zone.bounds;
    isContinent = (zone.areaID == 0);
    if (isContinent) {
        float left, right, top, bottom;
        if (getContinentProjectionBounds(zones, zoneIdx, left, right, top, bottom)) {
            bounds = {.locLeft = left, .locRight = right, .locTop = top, .locBottom = bottom};
        }
    }
    return bounds;
}

bool getContinentProjectionBounds(const std::vector<Zone>& zones,
                                   int contIdx,
                                   float& left, float& right,
                                   float& top, float& bottom) {
    if (contIdx < 0 || contIdx >= static_cast<int>(zones.size())) return false;
    const auto& cont = zones[contIdx];
    if (cont.areaID != 0) return false;

    if (std::abs(cont.bounds.locLeft - cont.bounds.locRight) > 0.001f &&
        std::abs(cont.bounds.locTop - cont.bounds.locBottom) > 0.001f) {
        left = cont.bounds.locLeft; right = cont.bounds.locRight;
        top = cont.bounds.locTop; bottom = cont.bounds.locBottom;
        return true;
    }

    std::vector<float> northEdges, southEdges, westEdges, eastEdges;
    for (int zi = 0; zi < static_cast<int>(zones.size()); zi++) {
        if (!zoneBelongsToContinent(zones, zi, contIdx)) continue;
        const auto& z = zones[zi];
        if (std::abs(z.bounds.locLeft - z.bounds.locRight) < 0.001f ||
            std::abs(z.bounds.locTop - z.bounds.locBottom) < 0.001f) continue;
        northEdges.push_back(std::max(z.bounds.locLeft, z.bounds.locRight));
        southEdges.push_back(std::min(z.bounds.locLeft, z.bounds.locRight));
        westEdges.push_back(std::max(z.bounds.locTop, z.bounds.locBottom));
        eastEdges.push_back(std::min(z.bounds.locTop, z.bounds.locBottom));
    }

    if (northEdges.size() < 3) {
        left = cont.bounds.locLeft; right = cont.bounds.locRight;
        top = cont.bounds.locTop; bottom = cont.bounds.locBottom;
        return std::abs(left - right) > 0.001f && std::abs(top - bottom) > 0.001f;
    }

    left = *std::max_element(northEdges.begin(), northEdges.end());
    right = *std::min_element(southEdges.begin(), southEdges.end());
    top = *std::max_element(westEdges.begin(), westEdges.end());
    bottom = *std::min_element(eastEdges.begin(), eastEdges.end());

    if (left <= right || top <= bottom) {
        left = cont.bounds.locLeft; right = cont.bounds.locRight;
        top = cont.bounds.locTop; bottom = cont.bounds.locBottom;
    }
    return std::abs(left - right) > 0.001f && std::abs(top - bottom) > 0.001f;
}

// ── Player position lookups ──────────────────────────────────

int findBestContinentForPlayer(const std::vector<Zone>& zones,
                                const glm::vec3& playerRenderPos) {
    float wowX = playerRenderPos.y;
    float wowY = playerRenderPos.x;

    int bestIdx = -1;
    float bestArea = std::numeric_limits<float>::max();
    float bestCenterDist2 = std::numeric_limits<float>::max();

    bool hasLeaf = false;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (zones[i].areaID == 0 && !isRootContinent(zones, i)) {
            hasLeaf = true;
            break;
        }
    }

    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        const auto& z = zones[i];
        if (z.areaID != 0) continue;
        if (hasLeaf && isRootContinent(zones, i)) continue;

        float minX = std::min(z.bounds.locLeft, z.bounds.locRight);
        float maxX = std::max(z.bounds.locLeft, z.bounds.locRight);
        float minY = std::min(z.bounds.locTop, z.bounds.locBottom);
        float maxY = std::max(z.bounds.locTop, z.bounds.locBottom);
        float spanX = maxX - minX;
        float spanY = maxY - minY;
        if (spanX < 0.001f || spanY < 0.001f) continue;

        bool contains = (wowX >= minX && wowX <= maxX && wowY >= minY && wowY <= maxY);
        float area = spanX * spanY;
        if (contains) {
            if (area < bestArea) { bestArea = area; bestIdx = i; }
        } else if (bestIdx < 0) {
            float cx = (minX + maxX) * 0.5f, cy = (minY + maxY) * 0.5f;
            float dist2 = (wowX - cx) * (wowX - cx) + (wowY - cy) * (wowY - cy);
            if (dist2 < bestCenterDist2) { bestCenterDist2 = dist2; bestIdx = i; }
        }
    }
    return bestIdx;
}

int findZoneByAreaId(const std::vector<Zone>& zones, uint32_t areaId) {
    if (areaId == 0) return -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (zones[i].areaID == areaId) return i;
    }
    return -1;
}

int findZoneForPlayer(const std::vector<Zone>& zones,
                       const glm::vec3& playerRenderPos,
                       uint32_t authoritativeZoneId) {
    // The server tells us which zone the player is in. When that names a zone on
    // this map, it is the answer and there is nothing to work out.
    const int known = findZoneByAreaId(zones, authoritativeZoneId);
    if (known >= 0) return known;

    float wowX = playerRenderPos.y;
    float wowY = playerRenderPos.x;

    // WorldMapArea bounds are axis-aligned rectangles around irregular zones, so
    // adjacent zones' boxes overlap heavily (e.g. Darkshore's box clips Felwood's and
    // Winterspring's). Picking the smallest-area containing box mis-resolved a player
    // standing deep in a large zone to a smaller neighbor whose box merely reached them,
    // which left the real zone fogged. Instead pick the box the player sits deepest
    // inside - the largest normalized distance to the nearest edge - which favors the
    // zone genuinely containing them over one they only clip at the border. Smaller area
    // breaks ties so a subzone still wins over its parent when equally central.
    int bestIdx = -1;
    float bestCentrality = -1.0f;
    float bestArea = std::numeric_limits<float>::max();

    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        const auto& z = zones[i];
        if (z.areaID == 0) continue;

        float minX = std::min(z.bounds.locLeft, z.bounds.locRight);
        float maxX = std::max(z.bounds.locLeft, z.bounds.locRight);
        float minY = std::min(z.bounds.locTop, z.bounds.locBottom);
        float maxY = std::max(z.bounds.locTop, z.bounds.locBottom);
        float spanX = maxX - minX, spanY = maxY - minY;
        if (spanX < 0.001f || spanY < 0.001f) continue;

        if (wowX >= minX && wowX <= maxX && wowY >= minY && wowY <= maxY) {
            float cx = std::min(wowX - minX, maxX - wowX) / spanX;  // [0, 0.5]
            float cy = std::min(wowY - minY, maxY - wowY) / spanY;
            float centrality = std::min(cx, cy);
            float area = spanX * spanY;
            if (centrality > bestCentrality + 1e-4f ||
                (centrality > bestCentrality - 1e-4f && area < bestArea)) {
                bestCentrality = centrality;
                bestArea = area;
                bestIdx = i;
            }
        }
    }
    return bestIdx;
}

// ── Zone–continent relationship ──────────────────────────────

bool zoneBelongsToContinent(const std::vector<Zone>& zones,
                             int zoneIdx, int contIdx) {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return false;
    if (contIdx < 0 || contIdx >= static_cast<int>(zones.size())) return false;

    const auto& z = zones[zoneIdx];
    const auto& cont = zones[contIdx];
    if (z.areaID == 0) return false;

    // Prefer explicit parent link if available
    if (z.parentWorldMapID != 0 && cont.wmaID != 0)
        return z.parentWorldMapID == cont.wmaID;

    // Fallback: spatial overlap heuristic
    auto rectMinX = [](const Zone& a) { return std::min(a.bounds.locLeft, a.bounds.locRight); };
    auto rectMaxX = [](const Zone& a) { return std::max(a.bounds.locLeft, a.bounds.locRight); };
    auto rectMinY = [](const Zone& a) { return std::min(a.bounds.locTop, a.bounds.locBottom); };
    auto rectMaxY = [](const Zone& a) { return std::max(a.bounds.locTop, a.bounds.locBottom); };

    float zMinX = rectMinX(z), zMaxX = rectMaxX(z);
    float zMinY = rectMinY(z), zMaxY = rectMaxY(z);
    if ((zMaxX - zMinX) < 0.001f || (zMaxY - zMinY) < 0.001f) return false;

    int bestContIdx = -1;
    float bestOverlap = 0.0f;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        const auto& c = zones[i];
        if (c.areaID != 0) continue;
        float cMinX = rectMinX(c), cMaxX = rectMaxX(c);
        float cMinY = rectMinY(c), cMaxY = rectMaxY(c);
        if ((cMaxX - cMinX) < 0.001f || (cMaxY - cMinY) < 0.001f) continue;

        float ox = std::max(0.0f, std::min(zMaxX, cMaxX) - std::max(zMinX, cMinX));
        float oy = std::max(0.0f, std::min(zMaxY, cMaxY) - std::max(zMinY, cMinY));
        float overlap = ox * oy;
        if (overlap > bestOverlap) { bestOverlap = overlap; bestContIdx = i; }
    }
    if (bestContIdx >= 0) return bestContIdx == contIdx;

    float centerX = (z.bounds.locLeft + z.bounds.locRight) * 0.5f;
    float centerY = (z.bounds.locTop + z.bounds.locBottom) * 0.5f;
    return centerX >= rectMinX(cont) && centerX <= rectMaxX(cont) &&
           centerY >= rectMinY(cont) && centerY <= rectMaxY(cont);
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
