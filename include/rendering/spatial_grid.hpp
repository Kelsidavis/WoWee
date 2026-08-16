#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace rendering {

/// The uniform grid the M2 and WMO renderers narrow a collision query with.
///
/// Both kept their own copy of this, identical down to the hash constants and
/// the cell size, because both answer the same question: which of the tens of
/// thousands of instances in a zone could possibly overlap this box. Scanning
/// them all is what it replaces.
///
/// A miss here does not crash. It hands the caller too few candidates and the
/// player walks through a wall, so both callers keep a fallback that scans
/// everything when the grid answers nothing, and the arithmetic below is
/// pinned by tests/test_spatial_grid.cpp.

/// 64 world units, which is a little under an ADT chunk.
inline constexpr float kSpatialCellSize = 64.0f;

struct GridCell {
    int x;
    int y;
    int z;
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridCellHash {
    size_t operator()(const GridCell& c) const {
        size_t h1 = std::hash<int>()(c.x);
        size_t h2 = std::hash<int>()(c.y);
        size_t h3 = std::hash<int>()(c.z);
        return h1 ^ (h2 * 0x9e3779b9u) ^ (h3 * 0x85ebca6bu);
    }
};

/// Cell to the instance ids that touch it.
using SpatialGrid = std::unordered_map<GridCell, std::vector<uint32_t>, GridCellHash>;

/// The cell a world position falls in.
///
/// std::floor rather than a cast to int, and that is the whole trap: a cast
/// truncates towards zero, so every position between -64 and 0 would land in
/// cell 0 alongside the positions just above it, and the cells eastward of the
/// map origin would each be double width.
inline GridCell toSpatialCell(const glm::vec3& p) {
    return GridCell{
        static_cast<int>(std::floor(p.x / kSpatialCellSize)),
        static_cast<int>(std::floor(p.y / kSpatialCellSize)),
        static_cast<int>(std::floor(p.z / kSpatialCellSize))
    };
}

/// Files an instance under every cell its world bounds reach into.
inline void insertBounds(SpatialGrid& grid, const glm::vec3& boundsMin,
                         const glm::vec3& boundsMax, uint32_t id) {
    const GridCell minCell = toSpatialCell(boundsMin);
    const GridCell maxCell = toSpatialCell(boundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                grid[GridCell{x, y, z}].push_back(id);
            }
        }
    }
}

/// Takes an instance back out of every cell its bounds reached.
///
/// Called with the bounds the instance had when it was filed, not its current
/// ones: an instance that has moved is erased with its old box and filed again
/// with its new one, and passing the new box to both leaves it listed in cells
/// it has left.
///
/// A cell emptied this way is left in place rather than erased. Zones churn
/// instances constantly and the cell will be filled again.
inline void eraseBounds(SpatialGrid& grid, const glm::vec3& boundsMin,
                        const glm::vec3& boundsMax, uint32_t id) {
    const GridCell minCell = toSpatialCell(boundsMin);
    const GridCell maxCell = toSpatialCell(boundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                auto it = grid.find(GridCell{x, y, z});
                if (it == grid.end()) continue;
                auto& ids = it->second;
                ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
            }
        }
    }
}

/// Moves an instance from the box it was filed under to its current one.
///
/// An instance that has moved but still covers the same run of cells needs no
/// work at all, and that is the common case: units and doodads move a few
/// units per frame inside cells 64 wide. Refiling anyway is not merely wasted
/// work, it is a linear erase across every cell the instance touches, once per
/// moving instance per frame.
inline void refileBounds(SpatialGrid& grid,
                         const glm::vec3& oldMin, const glm::vec3& oldMax,
                         const glm::vec3& newMin, const glm::vec3& newMax,
                         uint32_t id) {
    if (toSpatialCell(oldMin) == toSpatialCell(newMin) &&
        toSpatialCell(oldMax) == toSpatialCell(newMax)) {
        return;
    }
    eraseBounds(grid, oldMin, oldMax, id);
    insertBounds(grid, newMin, newMax, id);
}

/// The distinct ids filed in any cell the query box reaches, in the order the
/// grid walks them. An instance spanning several cells is offered once.
///
/// `seen` is the caller's scratch set, passed in because both callers hold a
/// thread_local one: these queries run on the camera's async threads.
inline void gatherIds(const SpatialGrid& grid, const glm::vec3& queryMin,
                      const glm::vec3& queryMax, std::unordered_set<uint32_t>& seen,
                      const std::function<void(uint32_t)>& onId) {
    seen.clear();
    const GridCell minCell = toSpatialCell(queryMin);
    const GridCell maxCell = toSpatialCell(queryMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                auto it = grid.find(GridCell{x, y, z});
                if (it == grid.end()) continue;
                for (uint32_t id : it->second) {
                    if (!seen.insert(id).second) continue;
                    onId(id);
                }
            }
        }
    }
}


/// A rectangle of cells in a dense grid, inclusive at both ends.
struct CellRange {
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;

    [[nodiscard]] size_t count() const {
        return static_cast<size_t>(maxX - minX + 1) * static_cast<size_t>(maxY - minY + 1);
    }
};

/// Which cells of a dense grid a query box covers, or nothing when it misses.
///
/// This is the other kind of grid in the renderer: where SpatialGrid above is a
/// hash of cells over a whole zone, a WMO group carries a dense grid of its own
/// triangles, sized to its bounding box. Three queries walked it, for any
/// triangle, for floors and for walls, and each computed the cell rectangle
/// itself.
///
/// A cell missed here is a triangle never tested, so the failure is falling
/// through a floor or walking through a wall, and it happens only at whatever
/// spot the arithmetic goes wrong.
///
/// The extents are guarded rather than trusted: a group whose bounding box is
/// flat in one axis would divide by zero, and a degenerate group is a real
/// thing, a doorway or a railing modelled as a single plane.
inline std::optional<CellRange> cellRangeCovering(
        int cellsX, int cellsY, float extentX, float extentY,
        const glm::vec2& gridOrigin,
        float minX, float minY, float maxX, float maxY) {
    if (cellsX <= 0 || cellsY <= 0) return std::nullopt;

    const float invCellW = static_cast<float>(cellsX) / std::max(0.01f, extentX);
    const float invCellH = static_cast<float>(cellsY) / std::max(0.01f, extentY);

    CellRange range;
    range.minX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    range.minY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    range.maxX = std::min(cellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    range.maxY = std::min(cellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (range.minX > range.maxX || range.minY > range.maxY) return std::nullopt;
    return range;
}

}  // namespace rendering
}  // namespace wowee
