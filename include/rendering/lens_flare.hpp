#pragma once

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <vector>

namespace wowee {
namespace rendering {

class Camera;
class VkContext;

/**
 * @brief Renders lens flare effect when looking at the sun
 *
 * Features:
 * - Multiple flare elements (ghosts) along sun-to-center axis
 * - Sun glow at sun position
 * - Colored flare elements (chromatic aberration simulation)
 * - Intensity based on sun visibility and angle
 * - Additive blending for realistic light artifacts
 */
class LensFlare {
public:
    LensFlare();
    ~LensFlare();

    /**
     * @brief Initialize lens flare system
     * @param ctx Vulkan context
     * @param perFrameLayout Per-frame descriptor set layout (unused, kept for API consistency)
     * @return true if initialization succeeded
     */
    bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);

    /**
     * @brief Destroy Vulkan resources
     */
    void shutdown();

    void recreatePipelines();
    /// The pipeline state both initialize() and
    /// recreatePipelines() need, described once.
    void buildPipelines(VkDevice device,
                        const VkPipelineShaderStageCreateInfo& vertStage,
                        const VkPipelineShaderStageCreateInfo& fragStage);

    /**
     * @brief Render lens flare effect
     * @param cmd Command buffer to record into
     * @param camera The camera to render from
     * @param sunPosition World-space sun position
     * @param timeOfDay Current time (0-24 hours)
     * @param fogDensity Fog density 0-1 (attenuates flare)
     * @param cloudDensity Cloud density 0-1 (attenuates flare)
     * @param weatherIntensity Weather intensity 0-1 (rain/snow attenuates flare)
     */
    void render(VkCommandBuffer cmd, const Camera& camera, const glm::vec3& sunPosition,
                float timeOfDay, float fogDensity = 0.0f, float cloudDensity = 0.0f,
                float weatherIntensity = 0.0f);

    /**
     * @brief Enable or disable lens flare rendering
     */
    void setEnabled(bool enabled) { this->enabled = enabled; }
    [[nodiscard]] bool isEnabled() const { return enabled; }

    /**
     * @brief Set flare intensity multiplier
     */
    void setIntensity(float intensity);
    [[nodiscard]] float getIntensity() const { return intensityMultiplier; }

private:
    struct FlareElement {
        float position;      // Position along sun-center axis (-1 to 1, 0 = center)
        float size;          // Size in screen space
        glm::vec3 color;     // RGB color
        float brightness;    // Brightness multiplier
    };

    struct FlarePushConstants {
        glm::vec2 position;     // Screen-space position (-1 to 1)
        float size;             // Size in screen space
        float aspectRatio;      // Viewport aspect ratio
        glm::vec4 colorBrightness; // RGB color + brightness in w
    };

    void generateFlareElements();
    [[nodiscard]] float calculateSunVisibility(const Camera& camera, const glm::vec3& sunPosition) const;
    [[nodiscard]] glm::vec2 worldToScreen(const Camera& camera, const glm::vec3& worldPos) const;

    VkContext* vkCtx = nullptr;

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VmaAllocation vertexAlloc = VK_NULL_HANDLE;

    // Flare elements
    std::vector<FlareElement> flareElements;

    // Parameters
    bool enabled = true;
    float intensityMultiplier = 1.0f;

    // Quad vertices for rendering flare sprites
    static constexpr int VERTICES_PER_QUAD = 6;
};

} // namespace rendering
} // namespace wowee
