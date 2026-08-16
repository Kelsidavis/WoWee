// view_state_machine.hpp - Navigation state and transitions for the world map.
// Extracted from WorldMap zoom/enter methods (Phase 6 of refactoring plan).
// SRP - pure state machine, no rendering or input code.
#pragma once

#include "rendering/world_map/world_map_types.hpp"

namespace wowee {
namespace rendering {
namespace world_map {

/// Manages the current view level and transitions between views.
class ViewStateMachine {
public:
    [[nodiscard]] ViewLevel currentLevel() const { return level_; }
    [[nodiscard]] const TransitionState& transition() const { return transition_; }

    [[nodiscard]] int continentIdx() const { return continentIdx_; }
    [[nodiscard]] int currentZoneIdx() const { return currentIdx_; }
    [[nodiscard]] bool cosmicEnabled() const { return cosmicEnabled_; }

    void setContinentIdx(int idx) { continentIdx_ = idx; }
    void setCurrentZoneIdx(int idx) { currentIdx_ = idx; }
    void setCosmicEnabled(bool enabled) { cosmicEnabled_ = enabled; }
    void setLevel(ViewLevel level) { level_ = level; }

    /// Result of a zoom/navigate operation.
    struct ZoomResult {
        bool changed = false;
        ViewLevel newLevel = ViewLevel::ZONE;
        int targetIdx = -1;   // zone index to load/composite
    };

    /// Attempt to zoom in. hoveredZoneIdx is the zone under the cursor (-1 if none).
    /// playerZoneIdx is the zone the player is standing in (-1 if none).
    ZoomResult zoomIn(int hoveredZoneIdx, int playerZoneIdx);

    /// Attempt to zoom out one level.
    ZoomResult zoomOut();

    /// Navigate to world view. Returns the root/fallback continent index to composite.
    ZoomResult enterWorldView();

    /// Navigate to cosmic view.
    ZoomResult enterCosmicView();

    /// Navigate directly into a zone from continent view.
    ZoomResult enterZone(int zoneIdx);

    /// Advance transition animation. Returns true while animating.
    bool updateTransition(float deltaTime);

private:
    void startTransition(ViewLevel from, ViewLevel to, float duration = 0.3f);

    ViewLevel level_ = ViewLevel::CONTINENT;
    TransitionState transition_;
    int continentIdx_ = -1;
    int currentIdx_ = -1;
    bool cosmicEnabled_ = true;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
