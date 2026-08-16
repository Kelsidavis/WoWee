// exploration_state.hpp - Fog of war / exploration tracking (pure domain logic).
// Extracted from WorldMap::updateExploration (Phase 3 of refactoring plan).
// No rendering or GPU dependencies - fully testable standalone.
#pragma once

#include "rendering/world_map/world_map_types.hpp"
#include <glm/glm.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace rendering {
namespace world_map {

class ExplorationState {
public:
    void setServerMask(const std::vector<uint32_t>& masks, bool hasData);
    [[nodiscard]] bool hasServerMask() const { return hasServerMask_; }

    /// Recompute explored zones and overlays for given player position.
    /// @param zones          All loaded zones
    /// @param playerRenderPos Player position in render-space
    /// @param currentZoneIdx  Currently viewed zone index
    /// @param exploreFlagByAreaId  AreaID → ExploreFlag mapping from AreaTable.dbc
    /// @param playerZoneId The zone the server says the player is in (0 if unknown),
    ///        used in preference to deriving it from overlapping map geometry.
    void update(const std::vector<Zone>& zones,
                const glm::vec3& playerRenderPos,
                int currentZoneIdx,
                const std::unordered_map<uint32_t, uint32_t>& exploreFlagByAreaId,
                uint32_t playerZoneId = 0);

    [[nodiscard]] const std::unordered_set<int>& exploredZones() const { return exploredZones_; }
    [[nodiscard]] const std::unordered_set<int>& exploredOverlays() const { return exploredOverlays_; }

    /// Returns true if the explored overlay set changed since last update.
    [[nodiscard]] bool overlaysChanged() const { return overlaysChanged_; }

    /// Clear accumulated local exploration data.
    void clearLocal() { locallyExploredZones_.clear(); }

private:
    [[nodiscard]] bool isBitSet(uint32_t bitIndex) const;

    std::vector<uint32_t> serverMask_;
    bool hasServerMask_ = false;
    std::unordered_set<int> exploredZones_;
    std::unordered_set<int> exploredOverlays_;
    std::unordered_set<int> locallyExploredZones_;
    bool overlaysChanged_ = false;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
