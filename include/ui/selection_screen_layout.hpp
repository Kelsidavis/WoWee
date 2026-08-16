#pragma once

#include <imgui.h>
#include <algorithm>

namespace wowee::ui {

// Shared shell for the realm and character selection screens. Keeping their
// maximum logical size and footer geometry in one place prevents controls from
// jumping when moving between login stages or using an ultrawide/high-DPI view.
struct SelectionScreenLayout {
    float scale = 1.0f;
    ImVec2 windowPos{};
    ImVec2 windowSize{};

    [[nodiscard]] float footerHeight() const { return 88.0f * scale; }
    [[nodiscard]] float gap() const { return 10.0f * scale; }
    [[nodiscard]] ImVec2 button(float width = 120.0f) const {
        return ImVec2(width * scale, 38.0f * scale);
    }
    [[nodiscard]] ImVec2 primaryButton(float width = 180.0f) const {
        return ImVec2(width * scale, 42.0f * scale);
    }
};

inline SelectionScreenLayout makeSelectionScreenLayout(const ImGuiViewport& vp) {
    SelectionScreenLayout layout;
    const float resolutionScale = std::min(vp.Size.x / 1920.0f,
                                           vp.Size.y / 1080.0f);
    layout.scale = std::clamp(resolutionScale, 1.0f, 1.5f);

    const float margin = 28.0f * layout.scale;
    const ImVec2 available(std::max(720.0f, vp.Size.x - margin * 2.0f),
                           std::max(540.0f, vp.Size.y - margin * 2.0f));
    const ImVec2 preferred(1600.0f * layout.scale, 900.0f * layout.scale);
    layout.windowSize = ImVec2(std::min(available.x, preferred.x),
                               std::min(available.y, preferred.y));
    layout.windowPos = ImVec2(vp.Pos.x + (vp.Size.x - layout.windowSize.x) * 0.5f,
                              vp.Pos.y + (vp.Size.y - layout.windowSize.y) * 0.5f);
    return layout;
}

} // namespace wowee::ui
