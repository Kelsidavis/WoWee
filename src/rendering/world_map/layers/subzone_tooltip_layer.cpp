// subzone_tooltip_layer.cpp - Overlay area hover labels in zone view.
// Extracted from WorldMap::renderImGuiOverlay (Phase 8 of refactoring plan).
#include "rendering/world_map/layers/subzone_tooltip_layer.hpp"
#include <imgui.h>
#include <limits>

namespace wowee {
namespace rendering {
namespace world_map {

void SubzoneTooltipLayer::render(const LayerContext& ctx) {
    if (ctx.viewLevel != ViewLevel::ZONE) return;
    if (ctx.currentZoneIdx < 0 || !ctx.zones) return;

    ImVec2 mp = ImGui::GetIO().MousePos;
    if (mp.x < ctx.imgMin.x || mp.x > ctx.imgMin.x + ctx.displayW ||
        mp.y < ctx.imgMin.y || mp.y > ctx.imgMin.y + ctx.displayH)
        return;

    float mu = (mp.x - ctx.imgMin.x) / ctx.displayW;
    float mv = (mp.y - ctx.imgMin.y) / ctx.displayH;

    const auto& zone = (*ctx.zones)[ctx.currentZoneIdx];
    std::string hoveredName;
    bool hoveredExplored = false;
    float bestArea = std::numeric_limits<float>::max();

    float fboW = static_cast<float>(ctx.fboW);
    float fboH = static_cast<float>(ctx.fboH);

    // Mouse position in FBO pixel coordinates (used for HitRect AABB test)
    float pixelX = mu * fboW;
    float pixelY = mv * fboH;

    for (int oi = 0; oi < static_cast<int>(zone.overlays.size()); oi++) {
        const auto& ov = zone.overlays[oi];

        // ── Hybrid Approach: Zone view uses HitRect AABB pre-filter ──
        // WorldMapOverlay.dbc fields 13-16 define a hit-test rectangle.
        // Only overlays whose HitRect contains the mouse need further testing.
        // This is Blizzard's optimization to avoid sampling every overlay.
        bool hasHitRect = (ov.hitRectRight > ov.hitRectLeft &&
                           ov.hitRectBottom > ov.hitRectTop);
        if (hasHitRect) {
            if (pixelX < static_cast<float>(ov.hitRectLeft) ||
                pixelX > static_cast<float>(ov.hitRectRight) ||
                pixelY < static_cast<float>(ov.hitRectTop) ||
                pixelY > static_cast<float>(ov.hitRectBottom)) {
                continue;  // Mouse outside HitRect - skip this overlay
            }
        } else {
            // Fallback: use overlay offset+size AABB (old behaviour)
            float ovLeft   = static_cast<float>(ov.offsetX) / fboW;
            float ovTop    = static_cast<float>(ov.offsetY) / fboH;
            float ovRight  = static_cast<float>(ov.offsetX + ov.texWidth)  / fboW;
            float ovBottom = static_cast<float>(ov.offsetY + ov.texHeight) / fboH;

            if (mu < ovLeft || mu > ovRight || mv < ovTop || mv > ovBottom)
                continue;
        }

        float area = static_cast<float>(ov.texWidth) * static_cast<float>(ov.texHeight);
        if (area < bestArea) {
            bestArea = area;
            // Find display name from the first valid area ID
            for (unsigned int areaID : ov.areaIDs) {
                if (areaID == 0) continue;
                if (ctx.areaNameByAreaId) {
                    auto nameIt = ctx.areaNameByAreaId->find(areaID);
                    if (nameIt != ctx.areaNameByAreaId->end()) {
                        hoveredName = nameIt->second;
                        break;
                    }
                }
            }
            hoveredExplored = ctx.exploredOverlays &&
                              ctx.exploredOverlays->count(oi) > 0;
        }
    }

    if (!hoveredName.empty()) {
        std::string label = hoveredName;
        if (!hoveredExplored)
            label += "  (Unexplored)";

        ImVec2 labelSz = ImGui::CalcTextSize(label.c_str());
        float lx = ctx.imgMin.x + (ctx.displayW - labelSz.x) * 0.5f;
        float ly = ctx.imgMin.y + 6.0f;
        ImU32 labelCol = hoveredExplored
            ? IM_COL32(255, 230, 150, 240)
            : IM_COL32(160, 160, 160, 200);
        ctx.drawList->AddText(ImVec2(lx + 1.0f, ly + 1.0f),
                              IM_COL32(0, 0, 0, 200), label.c_str());
        ctx.drawList->AddText(ImVec2(lx, ly), labelCol, label.c_str());
    }
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
