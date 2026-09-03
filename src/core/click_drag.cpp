#include "core/click_drag.hpp"

#include <algorithm>

#include <imgui.h>

namespace wowee {
namespace core {

float clickDragThreshold() {
    const ImGuiIO& io = ImGui::GetIO();
    const float shorter = std::min(io.DisplaySize.x, io.DisplaySize.y);
    return std::max(5.0f, shorter * 0.008f);
}

} // namespace core
} // namespace wowee
