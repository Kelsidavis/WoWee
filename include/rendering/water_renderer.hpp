#pragma once

#include <vector>
#include <memory>
#include <optional>
#include <cstdint>
#include <functional>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>

namespace wowee {
namespace pipeline {
    struct ADTTerrain;
    struct LiquidData;
    struct WMOLiquid;
}

namespace rendering {

class Camera;
class VkContext;

/**
 * Water surface for a single map chunk
 */
struct WaterSurface {
    glm::vec3 position;
    glm::vec3 origin;
    glm::vec3 stepX;
    glm::vec3 stepY;
    float minHeight;
    float maxHeight;
    uint16_t liquidType;

    int tileX = -1, tileY = -1;
    uint32_t wmoId = 0;

    uint8_t xOffset = 0;
    uint8_t yOffset = 0;
    uint8_t width = 8;
    uint8_t height = 8;

    std::vector<float> heights;
    std::vector<uint8_t> mask;

    // Vulkan render data
    ::VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VmaAllocation vertexAlloc = VK_NULL_HANDLE;
    ::VkBuffer indexBuffer = VK_NULL_HANDLE;
    VmaAllocation indexAlloc = VK_NULL_HANDLE;
    int indexCount = 0;

    // Per-surface material UBO
    ::VkBuffer materialUBO = VK_NULL_HANDLE;
    VmaAllocation materialAlloc = VK_NULL_HANDLE;

    // Material descriptor set (set 1)
    VkDescriptorSet materialSet = VK_NULL_HANDLE;

    bool hasHeightData() const { return !heights.empty(); }
};

/**
 * Water renderer (Vulkan) with planar reflections, Gerstner waves,
 * GGX specular, shoreline foam, and subsurface scattering.
 */
class WaterRenderer {
public:
    WaterRenderer();
    ~WaterRenderer();

    bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);
    void shutdown();

    void loadFromTerrain(const pipeline::ADTTerrain& terrain, bool append = false,
                         int tileX = -1, int tileY = -1);

    void loadFromWMO(const pipeline::WMOLiquid& liquid, const glm::mat4& modelMatrix, uint32_t wmoId);
    void removeWMO(uint32_t wmoId);
    void removeTile(int tileX, int tileY);
    void clear();

    void recreatePipelines();

    // Separate 1x pass for MSAA mode — water rendered after MSAA resolve
    bool createWater1xPass(VkFormat colorFormat, VkFormat depthFormat);
    void createWater1xFramebuffers(const std::vector<VkImageView>& swapViews,
                                    VkImageView depthView, VkExtent2D extent);
    void destroyWater1xResources();
    bool beginWater1xPass(VkCommandBuffer cmd, uint32_t imageIndex, VkExtent2D extent);
    void endWater1xPass(VkCommandBuffer cmd);
    bool hasWater1xPass() const { return water1xRenderPass != VK_NULL_HANDLE; }
    VkRenderPass getWater1xRenderPass() const { return water1xRenderPass; }

    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera, float time, bool use1x = false);
    void captureSceneHistory(VkCommandBuffer cmd,
                             VkImage srcColorImage,
                             VkImage srcDepthImage,
                             VkExtent2D srcExtent,
                             bool srcDepthIsMsaa);

    // --- Planar reflection pass ---
    // Call sequence: beginReflectionPass → [render scene] → endReflectionPass
    bool beginReflectionPass(VkCommandBuffer cmd);
    void endReflectionPass(VkCommandBuffer cmd);

    // Get the dominant water height near a position (for reflection plane)
    std::optional<float> getDominantWaterHeight(const glm::vec3& cameraPos) const;

    // Compute reflected view matrix for a given water height
    static glm::mat4 computeReflectedView(const Camera& camera, float waterHeight);
    // Compute oblique clip projection to clip below-water geometry in reflection
    static glm::mat4 computeObliqueProjection(const glm::mat4& proj, const glm::mat4& view, float waterHeight);

    // Update the reflection UBO with reflected viewProj matrix
    void updateReflectionUBO(const glm::mat4& reflViewProj);

    VkRenderPass getReflectionRenderPass() const { return reflectionRenderPass; }
    VkExtent2D getReflectionExtent() const { return {REFLECTION_WIDTH, REFLECTION_HEIGHT}; }
    bool hasReflectionPass() const { return reflectionRenderPass != VK_NULL_HANDLE; }
    bool hasSurfaces() const { return !surfaces.empty(); }

    void setEnabled(bool enabled) { renderingEnabled = enabled; }
    bool isEnabled() const { return renderingEnabled; }

    std::optional<float> getWaterHeightAt(float glX, float glY) const;
    /// Like getWaterHeightAt but only returns water surfaces whose height is
    /// close to the query Z (within maxAbove units above). Avoids false
    /// underwater detection from elevated WMO water far above the camera.
    std::optional<float> getNearestWaterHeightAt(float glX, float glY, float queryZ, float maxAbove = 15.0f) const;
    std::optional<uint16_t> getWaterTypeAt(float glX, float glY) const;
    bool isWmoWaterAt(float glX, float glY) const;

    int getSurfaceCount() const { return static_cast<int>(surfaces.size()); }

private:
    void createWaterMesh(WaterSurface& surface);
    void destroyWaterMesh(WaterSurface& surface);

    glm::vec4 getLiquidColor(uint16_t liquidType) const;
    float getLiquidAlpha(uint16_t liquidType) const;

    void updateMaterialUBO(WaterSurface& surface);
    VkDescriptorSet allocateMaterialSet();
    void createSceneHistoryResources(VkExtent2D extent, VkFormat colorFormat, VkFormat depthFormat);
    void destroySceneHistoryResources();

    // Reflection pass resources
    void createReflectionResources();
    void destroyReflectionResources();

    VkContext* vkCtx = nullptr;

    // Pipeline
    VkPipeline waterPipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout materialSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool materialDescPool = VK_NULL_HANDLE;
    VkDescriptorSetLayout sceneSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneDescPool = VK_NULL_HANDLE;
    VkDescriptorSet sceneSet = VK_NULL_HANDLE;
    static constexpr uint32_t MAX_WATER_SETS = 16384;

    VkSampler sceneColorSampler = VK_NULL_HANDLE;
    VkSampler sceneDepthSampler = VK_NULL_HANDLE;
    VkImage sceneColorImage = VK_NULL_HANDLE;
    VmaAllocation sceneColorAlloc = VK_NULL_HANDLE;
    VkImageView sceneColorView = VK_NULL_HANDLE;
    VkImage sceneDepthImage = VK_NULL_HANDLE;
    VmaAllocation sceneDepthAlloc = VK_NULL_HANDLE;
    VkImageView sceneDepthView = VK_NULL_HANDLE;
    VkExtent2D sceneHistoryExtent = {0, 0};
    bool sceneHistoryReady = false;

    // Planar reflection resources
    static constexpr uint32_t REFLECTION_WIDTH = 512;
    static constexpr uint32_t REFLECTION_HEIGHT = 512;
    VkRenderPass reflectionRenderPass = VK_NULL_HANDLE;
    VkFramebuffer reflectionFramebuffer = VK_NULL_HANDLE;
    VkImage reflectionColorImage = VK_NULL_HANDLE;
    VmaAllocation reflectionColorAlloc = VK_NULL_HANDLE;
    VkImageView reflectionColorView = VK_NULL_HANDLE;
    VkImage reflectionDepthImage = VK_NULL_HANDLE;
    VmaAllocation reflectionDepthAlloc = VK_NULL_HANDLE;
    VkImageView reflectionDepthView = VK_NULL_HANDLE;
    VkSampler reflectionSampler = VK_NULL_HANDLE;
    VkImageLayout reflectionColorLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    // Reflection UBO (mat4 reflViewProj)
    ::VkBuffer reflectionUBO = VK_NULL_HANDLE;
    VmaAllocation reflectionUBOAlloc = VK_NULL_HANDLE;
    void* reflectionUBOMapped = nullptr;

    // Separate 1x water pass (used when MSAA is active)
    VkRenderPass water1xRenderPass = VK_NULL_HANDLE;
    VkPipeline water1xPipeline = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> water1xFramebuffers;

    std::vector<WaterSurface> surfaces;
    bool renderingEnabled = true;
};

} // namespace rendering
} // namespace wowee
