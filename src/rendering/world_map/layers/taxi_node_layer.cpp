// taxi_node_layer.cpp - Flight master markers on the world map.
// Passive mode: small diamonds with name tooltips (normal world map).
// Flight-map mode: interactive taxi selection - green marker at the current
// node, gold markers at reachable destinations, hover shows the dotted route
// and the flight cost, click activates the flight.
#include "rendering/world_map/layers/taxi_node_layer.hpp"
#include "game/item_text.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "core/coordinates.hpp"
#include <imgui.h>
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace wowee {
namespace rendering {
namespace world_map {

namespace {

void drawDiamond(ImDrawList* dl, float px, float py, float h,
                 ImU32 fill, ImU32 border, float borderThickness) {
    ImVec2 top(px, py - h);
    ImVec2 right(px + h, py);
    ImVec2 bot(px, py + h);
    ImVec2 left(px - h, py);
    dl->AddQuadFilled(top, right, bot, left, fill);
    dl->AddQuad(top, right, bot, left, border, borderThickness);
}

// WoW-style dotted flight route segment.
void drawDottedSegment(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 color) {
    float dx = b.x - a.x, dy = b.y - a.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f) return;
    constexpr float kDotSpacing = 9.0f;
    constexpr float kDotRadius = 1.8f;
    int dots = static_cast<int>(len / kDotSpacing);
    for (int i = 1; i <= dots; i++) {
        float t = (i * kDotSpacing) / len;
        dl->AddCircleFilled(ImVec2(a.x + dx * t, a.y + dy * t), kDotRadius, color);
    }
}

void formatCost(uint32_t copperTotal, char* buf, size_t bufSize) {
    std::snprintf(buf, bufSize, "%s", game::formatCopperPrice(copperTotal).c_str());
}

bool projectNodeToDisplayedMap(const TaxiNode& node, const LayerContext& ctx,
                               glm::vec3& outRenderPos) {
    if (static_cast<int>(node.mapId) == ctx.currentMapId) {
        outRenderPos = core::coords::canonicalToRender(
            glm::vec3(node.wowX, node.wowY, node.wowZ));
        return true;
    }
    if (!ctx.zones || ctx.currentMapId < 0) return false;

    // TaxiNodes retain their physical MapID. Draenei-island nodes therefore
    // say 530 even though WorldMapArea displays those islands under Kalimdor
    // (map 1). Admit only nodes that fall inside a virtual child area of the
    // displayed map; this includes Azuremyst without leaking the rest of the
    // Outland flight network onto Kalimdor.
    for (const Zone& zone : *ctx.zones) {
        if (zone.areaID == 0 || zone.mapID != node.mapId ||
            static_cast<int>(zone.displayMapID) != ctx.currentMapId) {
            continue;
        }
        const float minX = std::min(zone.bounds.locLeft, zone.bounds.locRight);
        const float maxX = std::max(zone.bounds.locLeft, zone.bounds.locRight);
        const float minY = std::min(zone.bounds.locTop, zone.bounds.locBottom);
        const float maxY = std::max(zone.bounds.locTop, zone.bounds.locBottom);
        const float displayWowX = node.wowX + zone.virtualOffsetWowX;
        const float displayWowY = node.wowY + zone.virtualOffsetWowY;
        if (displayWowX >= minX && displayWowX <= maxX &&
            displayWowY >= minY && displayWowY <= maxY) {
            outRenderPos = core::coords::canonicalToRender(
                glm::vec3(displayWowX, displayWowY, node.wowZ));
            return true;
        }
    }
    return false;
}

} // namespace

void TaxiNodeLayer::render(const LayerContext& ctx) {
    if (!nodes_ || nodes_->empty()) return;
    const auto projection = currentProjection(ctx);
    if (!projection) return;

    if (taxiMode_) {
        renderFlightMap(ctx, projection->bounds, projection->isContinent);
    } else {
        renderWorldMapMarkers(ctx, projection->bounds, projection->isContinent);
    }
}

void TaxiNodeLayer::renderWorldMapMarkers(const LayerContext& ctx,
                                          const ZoneBounds& bounds,
                                          bool isContinent) {
    ImVec2 mp = ImGui::GetMousePos();
    for (const auto& node : *nodes_) {
        glm::vec3 rPos(0.0f);
        if (!projectNodeToDisplayedMap(node, ctx, rPos)) continue;
        glm::vec2 uv = renderPosToMapUV(rPos, bounds, isContinent);
        if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) continue;

        float px = ctx.imgMin.x + uv.x * ctx.displayW;
        float py = ctx.imgMin.y + uv.y * ctx.displayH;

        // Discovered nodes are gold; undiscovered flight masters are drawn in
        // green so the player can see where flight paths exist before visiting
        // them (matches the green flight-point icon from the game).
        if (node.known) {
            drawDiamond(ctx.drawList, px, py, 5.0f,
                        IM_COL32(255, 215, 0, 230), IM_COL32(80, 50, 0, 200), 1.2f);
        } else {
            drawDiamond(ctx.drawList, px, py, 4.5f,
                        IM_COL32(70, 200, 90, 210), IM_COL32(15, 60, 25, 200), 1.2f);
        }

        if (!node.name.empty()) {
            float mdx = mp.x - px, mdy = mp.y - py;
            if (mdx * mdx + mdy * mdy < 49.0f) {
                if (node.known) {
                    ImGui::SetTooltip("%s\n(Flight Master)", node.name.c_str());
                } else {
                    ImGui::SetTooltip("%s\n(Flight Master - not yet discovered)",
                                      node.name.c_str());
                }
            }
        }
    }
}

void TaxiNodeLayer::renderFlightMap(const LayerContext& ctx,
                                    const ZoneBounds& bounds,
                                    bool isContinent) {
    ImVec2 mp = ImGui::GetMousePos();
    ImDrawList* dl = ctx.drawList;

    // Project every visible node once; the hovered route needs positions by id.
    struct Projected {
        const TaxiNode* node;
        float px, py;
    };
    std::vector<Projected> visible;
    std::unordered_map<uint32_t, ImVec2> posById;
    visible.reserve(nodes_->size());
    for (const auto& node : *nodes_) {
        if (!node.known) continue;
        if (!node.current && !node.reachable) continue;
        glm::vec3 rPos(0.0f);
        if (!projectNodeToDisplayedMap(node, ctx, rPos)) continue;
        glm::vec2 uv = renderPosToMapUV(rPos, bounds, isContinent);
        if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) continue;

        float px = ctx.imgMin.x + uv.x * ctx.displayW;
        float py = ctx.imgMin.y + uv.y * ctx.displayH;
        visible.push_back({.node = &node, .px = px, .py = py});
        posById[node.id] = ImVec2(px, py);
    }
    if (visible.empty()) return;

    // Hover detection (nearest node within pickup radius wins)
    constexpr float kPickRadiusSq = 81.0f;  // 9 px
    const TaxiNode* hovered = nullptr;
    float hoveredDistSq = kPickRadiusSq;
    for (const auto& p : visible) {
        float mdx = mp.x - p.px, mdy = mp.y - p.py;
        float distSq = mdx * mdx + mdy * mdy;
        if (distSq < hoveredDistSq) {
            hoveredDistSq = distSq;
            hovered = p.node;
        }
    }

    // Dotted route to the hovered destination, drawn under the markers
    if (hovered && !hovered->current && routeProvider_) {
        std::vector<uint32_t> route = routeProvider_(hovered->id);
        for (size_t i = 0; i + 1 < route.size(); i++) {
            auto a = posById.find(route[i]);
            auto b = posById.find(route[i + 1]);
            // Hops through nodes on another continent have no projection here.
            if (a == posById.end() || b == posById.end()) continue;
            drawDottedSegment(dl, a->second, b->second, IM_COL32(255, 235, 130, 220));
        }
    }

    // Markers
    for (const auto& p : visible) {
        const bool isHovered = (p.node == hovered);
        if (p.node->current) {
            // Current node: green marker with a white core
            drawDiamond(dl, p.px, p.py, isHovered ? 9.0f : 7.0f,
                        IM_COL32(60, 210, 60, 240), IM_COL32(10, 60, 10, 220), 1.5f);
            dl->AddCircleFilled(ImVec2(p.px, p.py), 2.0f, IM_COL32(235, 255, 235, 255));
        } else {
            drawDiamond(dl, p.px, p.py, isHovered ? 9.0f : 6.5f,
                        isHovered ? IM_COL32(255, 240, 120, 255)
                                  : IM_COL32(255, 215, 0, 235),
                        IM_COL32(80, 50, 0, 220), 1.5f);
        }
    }

    // Tooltip + click handling
    if (hovered) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(hovered->name.c_str());
        if (hovered->current) {
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "You are here");
        } else {
            char costBuf[48];
            formatCost(hovered->costCopper, costBuf, sizeof(costBuf));
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "Cost: %s", costBuf);
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Click to fly");
        }
        ImGui::EndTooltip();

        if (!hovered->current && onSelect_ && ImGui::GetIO().MouseClicked[0]) {
            onSelect_(hovered->id);
        }
    }
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
