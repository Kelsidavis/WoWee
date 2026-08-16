#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace wowee {
namespace rendering {

class VkContext;
struct SkyParams;

/**
 * Skybox renderer
 *
 * Renders an atmospheric sky gradient using a fullscreen triangle.
 * No vertex buffer: 3 vertices cover the entire screen via gl_VertexIndex.
 * World-space ray direction is reconstructed from the inverse view+projection.
 *
 * Sky colors are driven by DBC data (Light.dbc / LightIntBand.dbc) via SkyParams,
 * with Rayleigh/Mie atmospheric scattering for realistic appearance.
 */
class Skybox {
public:
    Skybox();
    ~Skybox();

    bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);
    void shutdown();
    void recreatePipelines();

    /**
     * Render the skybox using DBC-driven sky colors.
     * @param cmd Command buffer to record into
     * @param perFrameSet Per-frame descriptor set (set 0, contains camera UBO)
     * @param params Sky parameters with DBC colors and sun direction
     */
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const SkyParams& params);

    /**
     * Enable/disable skybox rendering
     */
    void setEnabled(bool enabled) { renderingEnabled = enabled; }
    [[nodiscard]] bool isEnabled() const { return renderingEnabled; }

    /**
     * Set time of day (0-24 hours)
     * 0 = midnight, 6 = dawn, 12 = noon, 18 = dusk, 24 = midnight
     */
    void setTimeOfDay(float time);
    [[nodiscard]] float getTimeOfDay() const { return timeOfDay; }

    /**
     * Enable/disable time progression
     */
    void setTimeProgression(bool enabled) { timeProgressionEnabled = enabled; }
    [[nodiscard]] bool isTimeProgressionEnabled() const { return timeProgressionEnabled; }

    /**
     * Update time progression
     */
    void update(float deltaTime);

private:
    VkContext* vkCtx = nullptr;

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

    float timeOfDay = 12.0f;  // Default: noon
    float timeSpeed = 1.0f;   // 1.0 = 1 hour per real second
    bool timeProgressionEnabled = false;
    bool renderingEnabled = true;
};

} // namespace rendering
} // namespace wowee
