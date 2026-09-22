// input_handler.hpp - Input processing for the world map.
// Extracted from WorldMap::render (Phase 9 of refactoring plan).
// SRP - input interpretation separated from state changes and rendering.
#pragma once

#include "rendering/world_map/world_map_types.hpp"

namespace wowee {
namespace rendering {
namespace world_map {

enum class InputAction {
    NONE,
    CLOSE,
    ZOOM_IN,
    ZOOM_OUT,
    CLICK_ZONE,          // left-click on continent view zone
    CLICK_COSMIC_REGION, // left-click on cosmic landmass
    RIGHT_CLICK_BACK,    // right-click to go back
};

struct InputResult {
    InputAction action = InputAction::NONE;
    int targetIdx = -1;  // zone or cosmic region index
};

class InputHandler {
public:
    /// Process input for current frame. Returns the highest-priority action.
    ///
    /// `mouseOverMap` is whether the cursor is over the map image. The wheel
    /// and the right-click step are the map's only while it is: FrameXML's
    /// quest list sits beside the map in the same frame, and scrolling it
    /// zoomed the map out as well.
    InputResult process(ViewLevel currentLevel,
                        int hoveredZoneIdx,
                        bool cosmicEnabled,
                        bool mouseOverMap);
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
