// quest_poi_layer.cpp - Quest objective markers on the world map.
// Extracted from WorldMap::renderImGuiOverlay (Phase 8 of refactoring plan).
#include "rendering/world_map/layers/quest_poi_layer.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "core/coordinates.hpp"
#include <imgui.h>

namespace wowee {
namespace rendering {
namespace world_map {

void QuestPOILayer::render(const LayerContext& ctx) {
    if (!pois_ || pois_->empty()) return;
    const auto projection = currentProjection(ctx);
    if (!projection) return;

    ImVec2 mp = ImGui::GetMousePos();
    ImFont* qFont = ImGui::GetFont();
    for (const auto& qp : *pois_) {
        glm::vec3 rPos = core::coords::canonicalToRender(
            glm::vec3(qp.wowX, qp.wowY, 0.0f));
        glm::vec2 uv = renderPosToMapUV(rPos, projection->bounds, projection->isContinent);
        if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) continue;

        float px = ctx.imgMin.x + uv.x * ctx.displayW;
        float py = ctx.imgMin.y + uv.y * ctx.displayH;

        const char* marker = nullptr;
        ImU32 fill = IM_COL32(0, 210, 255, 220);
        ImU32 outline = IM_COL32(255, 215, 0, 220);
        const char* description = "Quest Objective";
        switch (qp.kind) {
            case QuestPOI::Kind::AVAILABLE:
                marker = "!";
                fill = IM_COL32(255, 210, 0, 255);
                outline = IM_COL32(80, 55, 0, 230);
                description = "Has a quest for you";
                break;
            case QuestPOI::Kind::AVAILABLE_LOW:
                marker = "!";
                fill = IM_COL32(160, 160, 160, 255);
                outline = IM_COL32(50, 50, 50, 230);
                description = "Has a low-level quest for you";
                break;
            case QuestPOI::Kind::REWARD:
                marker = "?";
                fill = IM_COL32(255, 210, 0, 255);
                outline = IM_COL32(80, 55, 0, 230);
                description = "Quest ready to turn in";
                break;
            case QuestPOI::Kind::INCOMPLETE:
                marker = "?";
                fill = IM_COL32(160, 160, 160, 255);
                outline = IM_COL32(50, 50, 50, 230);
                description = "Quest in progress";
                break;
            case QuestPOI::Kind::OBJECTIVE:
                break;
        }

        ctx.drawList->AddCircleFilled(ImVec2(px, py), 5.0f, fill);
        ctx.drawList->AddCircle(ImVec2(px, py), 5.0f, outline, 0, 1.5f);
        if (marker) {
            const float markerSize = ImGui::GetFontSize() * 0.8f;
            ImVec2 markerSz = qFont->CalcTextSizeA(markerSize, FLT_MAX, 0.0f, marker);
            ctx.drawList->AddText(qFont, markerSize,
                                  ImVec2(px - markerSz.x * 0.5f, py - markerSz.y * 0.5f),
                                  IM_COL32(0, 0, 0, 255), marker);
        }

        // The name goes under an objective and nowhere else.
        //
        // Every marker used to carry one, so a zone with a dozen quest givers
        // in it was a wall of yellow NPC names over the map - and the names
        // were the least of what the marker already said: the ! and the ? say
        // what the NPC is for, and hovering one still names it. What a marker
        // cannot say by its shape is which quest an objective circle belongs
        // to, so that is the one that keeps its label.
        if (qp.kind == QuestPOI::Kind::OBJECTIVE && !qp.name.empty()) {
            ImVec2 nameSz = qFont->CalcTextSizeA(ImGui::GetFontSize() * 0.85f, FLT_MAX, 0.0f, qp.name.c_str());
            float tx = px - nameSz.x * 0.5f;
            float ty = py - nameSz.y - 7.0f;
            ctx.drawList->AddText(qFont, ImGui::GetFontSize() * 0.85f,
                                  ImVec2(tx + 1.0f, ty + 1.0f), IM_COL32(0, 0, 0, 180), qp.name.c_str());
            ctx.drawList->AddText(qFont, ImGui::GetFontSize() * 0.85f,
                                  ImVec2(tx, ty), IM_COL32(255, 230, 100, 230), qp.name.c_str());
        }
        float mdx = mp.x - px, mdy = mp.y - py;
        if (mdx * mdx + mdy * mdy < 49.0f && !qp.name.empty()) {
            ImGui::SetTooltip("%s\n(%s)", qp.name.c_str(), description);
        }
    }
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
