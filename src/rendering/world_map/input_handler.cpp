// input_handler.cpp - Input processing for the world map.
// Extracted from WorldMap::render (Phase 9 of refactoring plan).
#include "rendering/world_map/input_handler.hpp"
#include "core/input.hpp"
#include <imgui.h>
#include <cmath>

namespace wowee {
namespace rendering {
namespace world_map {

InputResult InputHandler::process(ViewLevel currentLevel,
                                   int hoveredZoneIdx,
                                   [[maybe_unused]] bool cosmicEnabled) {
    InputResult result;
    auto& input = core::Input::getInstance();

    // ESC closes the map
    if (input.isKeyJustPressed(SDL_SCANCODE_ESCAPE)) {
        result.action = InputAction::CLOSE;
        return result;
    }

    // Scroll wheel zoom
    auto& io = ImGui::GetIO();
    float wheelDelta = io.MouseWheel;
    if (wheelDelta > 0.0f) {
        result.action = InputAction::ZOOM_IN;
        return result;
    } if (wheelDelta < 0.0f) {
        result.action = InputAction::ZOOM_OUT;
        return result;
    }

    // Continent view: left-click on hovered zone (from previous frame)
    if (currentLevel == ViewLevel::CONTINENT && hoveredZoneIdx >= 0 &&
        input.isMouseButtonJustPressed(1)) {
        result.action = InputAction::CLICK_ZONE;
        result.targetIdx = hoveredZoneIdx;
        return result;
    }

    // Right-click to go back (zone → continent; continent → world)
    if (io.MouseClicked[1]) {
        result.action = InputAction::RIGHT_CLICK_BACK;
        return result;
    }

    return result;
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
