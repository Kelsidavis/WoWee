// zone_highlight_layer.cpp - Continent view zone rectangles + hover effects.
// Extracted from WorldMap::renderZoneHighlights (Phase 8 of refactoring plan).
#include "core/coordinates.hpp"
#include "rendering/imgui_texture.hpp"
#include "rendering/world_map/layers/zone_highlight_layer.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <backends/imgui_impl_vulkan.h>
#include <algorithm>
#include <cmath>

namespace wowee {
namespace rendering {
namespace world_map {

ZoneHighlightLayer::~ZoneHighlightLayer() {
    // At shutdown vkDeviceWaitIdle has been called, so immediate cleanup is safe.
    if (vkCtx_) {
        VkDevice device = vkCtx_->getDevice();
        VmaAllocator alloc = vkCtx_->getAllocator();
        for (auto& [name, entry] : highlights_) {
            removeImGuiTexture(entry.imguiDS);
            if (entry.texture) entry.texture->destroy(device, alloc);
        }
    }
    highlights_.clear();
    missingHighlights_.clear();
}

void ZoneHighlightLayer::initialize(VkContext* ctx, pipeline::AssetManager* am) {
    vkCtx_ = ctx;
    assetManager_ = am;
}

void ZoneHighlightLayer::clearTextures() {
    if (vkCtx_ && !highlights_.empty()) {
        // Defer destruction until all in-flight frames complete.
        // The previous frame's command buffer may still reference these ImGui
        // descriptor sets and texture image views from highlight draw commands.
        VkDevice device = vkCtx_->getDevice();
        VmaAllocator alloc = vkCtx_->getAllocator();

        struct DeferredHighlight {
            std::unique_ptr<VkTexture> texture;
            VkDescriptorSet imguiDS;
        };
        auto captured = std::make_shared<std::vector<DeferredHighlight>>();
        for (auto& [name, entry] : highlights_) {
            DeferredHighlight dh;
            dh.texture = std::move(entry.texture);
            dh.imguiDS = entry.imguiDS;
            captured->push_back(std::move(dh));
        }
        vkCtx_->deferAfterAllFrameFences([device, alloc, captured]() {
            for (auto& dh : *captured) {
                removeImGuiTexture(dh.imguiDS);
                if (dh.texture) dh.texture->destroy(device, alloc);
            }
        });
    }
    highlights_.clear();
    missingHighlights_.clear();
}

void ZoneHighlightLayer::ensureHighlight(const std::string& key,
                                          const std::string& customPath) {
    if (!vkCtx_ || !assetManager_) return;
    if (key.empty()) return;
    if (highlights_.count(key) || missingHighlights_.count(key)) return;

    // Determine BLP path
    std::string path;
    if (!customPath.empty()) {
        path = customPath;
    } else {
        std::string lower = key;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        path = "Interface\\WorldMap\\" + key + "\\" + lower + "highlight.blp";
    }

    auto blpImage = assetManager_->loadTexture(path);
    if (!blpImage.isValid()) {
        LOG_WARNING("ZoneHighlightLayer: highlight not found for key='", key, "' path='", path, "'");
        missingHighlights_.insert(key);
        return;
    }

    LOG_INFO("ZoneHighlightLayer: loaded highlight key='", key, "' path='", path,
             "' ", blpImage.width, "x", blpImage.height, " dataSize=", blpImage.data.size());

    // WoW highlight BLPs with alphaDepth=0 use additive blending (white=glow, black=invisible).
    // Convert to alpha-blend compatible: set alpha = max(R,G,B) for fully opaque textures.
    {
        bool allOpaque = true;
        for (size_t i = 3; i < blpImage.data.size(); i += 4) {
            if (blpImage.data[i] < 255) { allOpaque = false; break; }
        }
        if (allOpaque) {
            for (size_t i = 0; i < blpImage.data.size(); i += 4) {
                uint8_t r = blpImage.data[i], g = blpImage.data[i + 1], b = blpImage.data[i + 2];
                blpImage.data[i + 3] = std::max({r, g, b});
            }
        }
    }

    auto loaded = makeImGuiTexture(*vkCtx_, blpImage);
    if (!loaded) {
        missingHighlights_.insert(key);
        return;
    }
    auto tex = std::move(loaded.texture);
    VkDescriptorSet ds = loaded.descriptorSet;

    HighlightEntry entry;
    entry.texture = std::move(tex);
    entry.imguiDS = ds;
    highlights_[key] = std::move(entry);
}

ImTextureID ZoneHighlightLayer::getHighlightTexture(const std::string& key,
                                                     const std::string& customPath) {
    ensureHighlight(key, customPath);
    auto it = highlights_.find(key);
    if (it != highlights_.end() && it->second.imguiDS) {
        return reinterpret_cast<ImTextureID>(it->second.imguiDS);
    }
    return 0;
}

void ZoneHighlightLayer::render(const LayerContext& ctx) {
    if (ctx.viewLevel != ViewLevel::CONTINENT || ctx.continentIdx < 0) return;
    if (!ctx.zones) return;

    // The same bounds every marker layer projects against. This spelled the
    // fallback out instead - seeding the four from the continent's own bounds
    // and letting the call leave them alone on failure - which is correct, and
    // is the eleventh place to say so.
    bool isContinent = false;
    const ZoneBounds cb = projectionBoundsFor(*ctx.zones, ctx.continentIdx, isContinent);
    const float cLeft = cb.locLeft, cRight = cb.locRight;
    const float cTop = cb.locTop, cBottom = cb.locBottom;
    float cDenomU = cLeft - cRight;
    float cDenomV = cTop - cBottom;

    if (std::abs(cDenomU) < 0.001f || std::abs(cDenomV) < 0.001f) return;

    hoveredZone_ = -1;
    ImVec2 mousePos = ImGui::GetMousePos();

    // ── ZMP pixel-accurate hover detection ──
    // The ZMP is a 128x128 grid covering the full world (64×64 ADTs of 533.333 each).
    // Convert mouse screen position → world coordinates → ZMP grid cell → areaID → zone.
    int zmpHoveredZone = -1;
    if (ctx.hasZmpData && ctx.zmpGrid && ctx.zmpResolveZoneIdx && ctx.zmpRepoPtr) {
        float mu = (mousePos.x - ctx.imgMin.x) / ctx.displayW;
        float mv = (mousePos.y - ctx.imgMin.y) / ctx.displayH;

        if (mu >= 0.0f && mu <= 1.0f && mv >= 0.0f && mv <= 1.0f) {
            // Screen UV → world coordinates
            float wowX = cLeft - mu * cDenomU;
            float wowY = cTop  - mv * cDenomV;

            // World coordinates → ZMP UV (0.5 = world center)
            constexpr float kWorldSize = 64.0f * core::coords::TILE_SIZE;  // 34133.312
            float zmpX = 0.5f - wowX / kWorldSize;
            float zmpY = 0.5f - wowY / kWorldSize;

            if (zmpX >= 0.0f && zmpX < 1.0f && zmpY >= 0.0f && zmpY < 1.0f) {
                int col = static_cast<int>(zmpX * 128.0f);
                int row = static_cast<int>(zmpY * 128.0f);
                col = std::clamp(col, 0, 127);
                row = std::clamp(row, 0, 127);
                uint32_t areaId = (*ctx.zmpGrid)[row * 128 + col];
                if (areaId != 0) {
                    int zi = ctx.zmpResolveZoneIdx(ctx.zmpRepoPtr, areaId);
                    if (zi >= 0 && zoneBelongsToContinent(*ctx.zones, zi, ctx.continentIdx)) {
                        zmpHoveredZone = zi;
                    }
                }
            }
        }
    }

    // ── Render zone rectangles ──
    for (int zi = 0; zi < static_cast<int>(ctx.zones->size()); zi++) {
        if (!zoneBelongsToContinent(*ctx.zones, zi, ctx.continentIdx)) continue;
        const auto& z = (*ctx.zones)[zi];
        if (std::abs(z.bounds.locLeft - z.bounds.locRight) < 0.001f ||
            std::abs(z.bounds.locTop - z.bounds.locBottom) < 0.001f) continue;

        // Project from WorldMapArea.dbc world coords
        float zuMin = (cLeft - z.bounds.locLeft) / cDenomU;
        float zuMax = (cLeft - z.bounds.locRight) / cDenomU;
        float zvMin = (cTop - z.bounds.locTop) / cDenomV;
        float zvMax = (cTop - z.bounds.locBottom) / cDenomV;

        constexpr float kOverlayShrink = 0.92f;
        float cu = (zuMin + zuMax) * 0.5f, cv = (zvMin + zvMax) * 0.5f;
        float hu = (zuMax - zuMin) * 0.5f * kOverlayShrink;
        float hv = (zvMax - zvMin) * 0.5f * kOverlayShrink;
        zuMin = cu - hu; zuMax = cu + hu;
        zvMin = cv - hv; zvMax = cv + hv;

        zuMin = std::clamp(zuMin, 0.0f, 1.0f);
        zuMax = std::clamp(zuMax, 0.0f, 1.0f);
        zvMin = std::clamp(zvMin, 0.0f, 1.0f);
        zvMax = std::clamp(zvMax, 0.0f, 1.0f);
        if (zuMax - zuMin < 0.001f || zvMax - zvMin < 0.001f) continue;

        float titleBarH = ImGui::GetFrameHeight();
        float sx0 = ctx.imgMin.x + zuMin * ctx.displayW;
        float sy0 = ctx.imgMin.y + zvMin * ctx.displayH + titleBarH;
        float sx1 = ctx.imgMin.x + zuMax * ctx.displayW;
        float sy1 = ctx.imgMin.y + zvMax * ctx.displayH + titleBarH;

        bool explored = !ctx.exploredZones ||
                        ctx.exploredZones->empty() ||
                        ctx.exploredZones->count(zi) > 0;
        // Use ZMP pixel-accurate hover when available; fall back to AABB
        bool hovered = (zmpHoveredZone >= 0)
            ? (zi == zmpHoveredZone)
            : (mousePos.x >= sx0 && mousePos.x <= sx1 &&
               mousePos.y >= sy0 && mousePos.y <= sy1);

        if (hovered) {
            hoveredZone_ = zi;

            if (prevHoveredZone_ == zi) {
                hoverHighlightAlpha_ = std::min(hoverHighlightAlpha_ + 0.08f, 1.0f);
            } else {
                hoverHighlightAlpha_ = 0.3f;
            }

            // Draw the highlight BLP texture within the zone's bounding rectangle.
            auto it = highlights_.find(z.areaName);
            if (it == highlights_.end()) ensureHighlight(z.areaName, "");
            it = highlights_.find(z.areaName);
            if (it != highlights_.end() && it->second.imguiDS) {
                uint8_t imgAlpha = static_cast<uint8_t>(255.0f * hoverHighlightAlpha_);
                // Draw twice for a very bright glow effect
                ctx.drawList->AddImage(
                    reinterpret_cast<ImTextureID>(it->second.imguiDS),
                    ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                    ImVec2(0, 0), ImVec2(1, 1),
                    IM_COL32(255, 255, 255, imgAlpha));
                ctx.drawList->AddImage(
                    reinterpret_cast<ImTextureID>(it->second.imguiDS),
                    ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                    ImVec2(0, 0), ImVec2(1, 1),
                    IM_COL32(255, 255, 200, imgAlpha));
            } else {
                // Fallback: bright colored rectangle if no highlight texture
                uint8_t fillAlpha = static_cast<uint8_t>(100.0f * hoverHighlightAlpha_);
                ctx.drawList->AddRectFilled(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                            IM_COL32(255, 235, 50, fillAlpha));
            }

            uint8_t borderAlpha = static_cast<uint8_t>(200.0f * hoverHighlightAlpha_);
            ctx.drawList->AddRect(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                  IM_COL32(255, 225, 50, borderAlpha), 0, 0, 2.0f);
        } else if (explored) {
            ctx.drawList->AddRect(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                  IM_COL32(255, 255, 255, 30), 0.0f, 0, 1.0f);
        }

        // Zone name label
        bool zoneExplored = explored;
        if (!z.areaName.empty()) {
            const ZoneMeta* meta = metadata_ ? metadata_->find(z.areaName) : nullptr;
            std::string label = ZoneMetadata::formatLabel(z.areaName, meta);

            ImFont* font = ImGui::GetFont();
            float fontSize = ImGui::GetFontSize() * 0.75f;
            ImVec2 labelSz = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, label.c_str());
            float zoneCx = (sx0 + sx1) * 0.5f;
            float zoneCy = (sy0 + sy1) * 0.5f;
            float lx = zoneCx - labelSz.x * 0.5f;
            float ly = zoneCy - labelSz.y * 0.5f;

            if (labelSz.x < (sx1 - sx0) * 1.1f && labelSz.y < (sy1 - sy0) * 0.8f) {
                ImU32 textColor;
                if (!zoneExplored) {
                    textColor = IM_COL32(140, 140, 140, 130);
                } else if (meta) {
                    switch (meta->faction) {
                        case ZoneFaction::Alliance:  textColor = IM_COL32(100, 160, 255, 200); break;
                        case ZoneFaction::Horde:     textColor = IM_COL32(255, 100, 100, 200); break;
                        case ZoneFaction::Contested:  textColor = IM_COL32(255, 215, 0, 190); break;
                        default:                      textColor = IM_COL32(255, 230, 180, 180); break;
                    }
                } else {
                    textColor = IM_COL32(255, 230, 180, 180);
                }
                ctx.drawList->AddText(font, fontSize,
                                      ImVec2(lx + 1.0f, ly + 1.0f),
                                      IM_COL32(0, 0, 0, 140), label.c_str());
                ctx.drawList->AddText(font, fontSize,
                                      ImVec2(lx, ly), textColor, label.c_str());
            }
        }
    }

    prevHoveredZone_ = hoveredZone_;
    if (hoveredZone_ < 0) {
        hoverHighlightAlpha_ = 0.0f;
    }
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
