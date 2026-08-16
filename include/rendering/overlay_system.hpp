#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <functional>
#include <optional>

namespace wowee {
namespace rendering {

class VkContext;

/// Manages selection circle and fullscreen overlay Vulkan pipelines.
/// Extracted from Renderer to isolate overlay rendering resources.
class OverlaySystem {
public:
    /// Height query callable: returns floor height at (x, y) or (x, y, probeZ).
    using HeightQuery2D = std::function<std::optional<float>(float x, float y)>;
    using HeightQuery3D = std::function<std::optional<float>(float x, float y, float probeZ)>;

    explicit OverlaySystem(VkContext* ctx);
    ~OverlaySystem();

    OverlaySystem(const OverlaySystem&) = delete;
    OverlaySystem& operator=(const OverlaySystem&) = delete;

    // Selection circle
    void setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color);
    void clearSelectionCircle();
    void renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection,
                               VkCommandBuffer cmd,
                               const HeightQuery2D& terrainHeight,
                               const HeightQuery3D& wmoHeight,
                               const HeightQuery3D& m2Height);

    // Fullscreen color overlay (underwater tint, etc.)
    void renderOverlay(const glm::vec4& color, VkCommandBuffer cmd);

    /// Underwater tint bounded by a waterline that sweeps across the view as the
    /// camera crosses the surface. lineNdc runs from +1 (line off the bottom, so
    /// nothing is tinted) to -1 (off the top, so everything is), softness spans
    /// the meniscus and rippleAmp bends the edge.
    void renderWaterline(const glm::vec4& color, const glm::mat4& invViewProj,
                         float waterZ, float softness, float rippleAmp,
                         float time, bool hasSeam, VkCommandBuffer cmd);

    // Fullscreen multiplicative brightness scale (scene.rgb *= scale). Uses a
    // dst-color blend so brightness > 1 truly scales luminance instead of
    // lerping toward white (which washed everything out). scale > 1 only.
    void renderBrightnessScale(float scale, VkCommandBuffer cmd);

    /// Destroy all Vulkan resources (called before VkContext teardown).
    void cleanup();

    /// Recreate pipelines after swapchain resize / MSAA change.
    void recreatePipelines();

private:
    void initSelectionCircle();
    void initOverlayPipeline();
    void initBrightnessPipeline();

    VkContext* vkCtx_ = nullptr;

    // Selection circle resources
    VkPipeline selCirclePipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout selCirclePipelineLayout_ = VK_NULL_HANDLE;
    ::VkBuffer selCircleVertBuf_ = VK_NULL_HANDLE;
    VmaAllocation selCircleVertAlloc_ = VK_NULL_HANDLE;
    ::VkBuffer selCircleIdxBuf_ = VK_NULL_HANDLE;
    VmaAllocation selCircleIdxAlloc_ = VK_NULL_HANDLE;
    int selCircleVertCount_ = 0;
    glm::vec3 selCirclePos_{0.0f};
    glm::vec3 selCircleColor_{1.0f, 0.0f, 0.0f};
    float selCircleRadius_ = 1.5f;
    bool selCircleVisible_ = false;

    // Floor-snap cache: the terrain/WMO/M2 floor queries are collision
    // raycasts, so reuse the result while the target stands still. Refreshed
    // periodically in case world geometry streams in around the target.
    glm::vec3 selCircleFloorCachePos_{0.0f};
    float selCircleFloorCacheZ_ = 0.0f;
    int selCircleFloorCacheAge_ = -1;  // -1 = invalid; counts frames since query

    // Fullscreen overlay resources
    VkPipeline overlayPipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout overlayPipelineLayout_ = VK_NULL_HANDLE;
    // Multiplicative brightness pipeline (shares overlayPipelineLayout_).
    VkPipeline brightnessPipeline_ = VK_NULL_HANDLE;
};

} // namespace rendering
} // namespace wowee
