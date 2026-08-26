// composite_renderer.hpp - Vulkan off-screen composite rendering for the world map.
// Extracted from WorldMap (Phase 7 of refactoring plan).
// SRP - all GPU resource management separated from domain logic.
#pragma once

#include "rendering/world_map/world_map_types.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <unordered_set>

namespace wowee {
namespace rendering {
class VkContext;
class VkTexture;
class VkRenderTarget;
}
namespace pipeline { class AssetManager; }
namespace rendering {
namespace world_map {

/// Push constant for world map tile composite vertex shader.
struct WorldMapTilePush {
    glm::vec2 gridOffset;  // 8 bytes
    float gridCols;          // 4 bytes
    float gridRows;          // 4 bytes
};  // 16 bytes

/// Push constant for the overlay/fog pipeline (vertex + fragment stages).
struct OverlayPush {
    glm::vec2 gridOffset;  // 8 bytes  (vertex)
    float gridCols;          // 4 bytes  (vertex)
    float gridRows;          // 4 bytes  (vertex)
    glm::vec4 tintColor;   // 16 bytes (fragment)
};  // 32 bytes

class CompositeRenderer {
public:
    CompositeRenderer();
    ~CompositeRenderer();

    bool initialize(VkContext* ctx, pipeline::AssetManager* am);
    void shutdown();

    /// Load base tile textures for a zone.
    void loadZoneTextures(int zoneIdx, std::vector<Zone>& zones, const std::string& mapName);

    /// Load exploration overlay textures for a zone.
    void loadOverlayTextures(int zoneIdx, std::vector<Zone>& zones);

    /// Request a composite for the given zone (deferred to compositePass).
    void requestComposite(int zoneIdx);

    /// Execute the off-screen composite pass.
    void compositePass(VkCommandBuffer cmd,
                       const std::vector<Zone>& zones,
                       const std::unordered_set<int>& exploredOverlays,
                       bool hasServerMask);

    /// Descriptor set for ImGui display of the composite.
    [[nodiscard]] VkDescriptorSet displayDescriptorSet() const { return imguiDisplaySet; }

    /// Destroy all loaded zone textures (on map change).

    /// Detach zone textures for deferred GPU destruction.
    /// Clears CPU tracking immediately but moves GPU texture objects to a stale
    /// list so they can be freed later when no in-flight frames reference them.
    void detachZoneTextures();

    /// Free any GPU textures previously moved to the stale list by detachZoneTextures.
    /// Calls vkDeviceWaitIdle internally to ensure no in-flight work references them.
    void flushStaleTextures();

    /// Index of the zone currently composited (-1 if none).
    [[nodiscard]] int compositedIdx() const { return compositedIdx_; }
    /// Whether the composite image has ever been rendered into.
    ///
    /// Distinct from compositedIdx(), which says whether it is *current*:
    /// invalidateComposite() clears that on every zone change while the image
    /// still holds a perfectly good picture. Only the first use is the one
    /// where its contents are undefined.
    [[nodiscard]] bool everComposited() const { return everComposited_; }

    /// Reset composited index to force re-composite.
    void invalidateComposite() { compositedIdx_ = -1; }

    /// Check whether a zone has any loaded tile textures.
    [[nodiscard]] bool hasAnyTile(int zoneIdx) const;

    // FBO dimensions (public for overlay coordinate math)
    static constexpr int GRID_COLS = 4;
    static constexpr int GRID_ROWS = 3;
    static constexpr int TILE_PX = 256;
    static constexpr int FBO_W = GRID_COLS * TILE_PX;
    static constexpr int FBO_H = GRID_ROWS * TILE_PX;

    // WoW's WorldMapDetailFrame is 1002x668 - the visible map content area.
    // The FBO is 1024x768 so we crop UVs to show only the actual map region.
    static constexpr int MAP_W = 1002;
    static constexpr int MAP_H = 668;
    static constexpr float MAP_U_MAX = static_cast<float>(MAP_W) / static_cast<float>(FBO_W);
    static constexpr float MAP_V_MAX = static_cast<float>(MAP_H) / static_cast<float>(FBO_H);

private:
    VkContext* vkCtx = nullptr;
    pipeline::AssetManager* assetManager = nullptr;
    bool initialized = false;

    std::unique_ptr<VkRenderTarget> compositeTarget;

    // Quad vertex buffer (pos2 + uv2)
    ::VkBuffer quadVB = VK_NULL_HANDLE;
    VmaAllocation quadVBAlloc = VK_NULL_HANDLE;

    // Descriptor resources
    VkDescriptorSetLayout samplerSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool descPool = VK_NULL_HANDLE;
    static constexpr uint32_t MAX_DESC_SETS = 192;
    static constexpr uint32_t MAX_OVERLAY_TILES = 48;

    // Tile composite pipeline
    VkPipeline tilePipeline = VK_NULL_HANDLE;
    VkPipelineLayout tilePipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSet tileDescSets[2][12] = {};  // [frameInFlight][tileSlot]

    // Alpha-blended overlay pipeline (fog + explored area overlays)
    VkPipeline overlayPipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout overlayPipelineLayout_ = VK_NULL_HANDLE;
    std::unique_ptr<VkTexture> fogTexture_;      // 1×1 white pixel for fog quad
    VkDescriptorSet fogDescSet_ = VK_NULL_HANDLE;
    VkDescriptorSet overlayDescSets_[2][MAX_OVERLAY_TILES] = {};

    // ImGui display descriptor set (points to composite render target)
    VkDescriptorSet imguiDisplaySet = VK_NULL_HANDLE;

    // Texture storage (owns all VkTexture objects for zone tiles)
    std::vector<std::unique_ptr<VkTexture>> zoneTextures;

    int compositedIdx_ = -1;
    bool everComposited_ = false;
    int pendingCompositeIdx_ = -1;

    // Per-zone tile texture pointers (indexed by zone, then by tile slot)
    // Stored separately since Zone struct is now Vulkan-free
    struct ZoneTextureSlots {
        VkTexture* tileTextures[12] = {};
        bool tilesLoaded = false;
        // Per-overlay tile textures
        struct OverlaySlots {
            std::vector<VkTexture*> tiles;
            bool tilesLoaded = false;
        };
        std::vector<OverlaySlots> overlays;
    };
    std::vector<ZoneTextureSlots> zoneTextureSlots_;

    void ensureTextureSlots(size_t zoneCount, const std::vector<Zone>& zones);
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
