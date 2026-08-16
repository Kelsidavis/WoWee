// zone_highlight_layer.hpp - Continent view zone rectangles + hover effects.
#pragma once
#include "rendering/world_map/overlay_renderer.hpp"
#include "rendering/world_map/zone_metadata.hpp"
#include "rendering/vk_texture.hpp"
#include <vulkan/vulkan.h>
#include <unordered_map>
#include <memory>

namespace wowee {
namespace rendering {
class VkContext;
}
namespace pipeline { class AssetManager; }
namespace rendering {
namespace world_map {

class ZoneHighlightLayer : public IOverlayLayer {
public:
    ~ZoneHighlightLayer() override;

    void setMetadata(const ZoneMetadata* metadata) { metadata_ = metadata; }
    void initialize(VkContext* ctx, pipeline::AssetManager* am);
    void clearTextures();
    void render(const LayerContext& ctx) override;
    [[nodiscard]] int hoveredZone() const { return hoveredZone_; }

    /// Get the ImGui texture ID for a highlight BLP, loading lazily.
    /// key is used as cache key; customPath overrides the default path if non-empty.
    ImTextureID getHighlightTexture(const std::string& key,
                                     const std::string& customPath = "");

private:
    /// Load the highlight BLP and register it with ImGui.
    void ensureHighlight(const std::string& key, const std::string& customPath);

    const ZoneMetadata* metadata_ = nullptr;
    VkContext* vkCtx_ = nullptr;
    pipeline::AssetManager* assetManager_ = nullptr;

    struct HighlightEntry {
        std::unique_ptr<VkTexture> texture;
        VkDescriptorSet imguiDS = VK_NULL_HANDLE;  // ImGui texture ID
    };
    std::unordered_map<std::string, HighlightEntry> highlights_;
    std::unordered_set<std::string> missingHighlights_;  // areas with no highlight file

    int hoveredZone_ = -1;
    int prevHoveredZone_ = -1;
    float hoverHighlightAlpha_ = 0.0f;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
