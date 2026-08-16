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
 * Lightning system for thunder storm effects
 *
 * Features:
 * - Random lightning strikes during rain
 * - Screen flash effect
 * - Procedural lightning bolts with branches
 * - Thunder timing (light then sound delay)
 * - Intensity scaling with weather
 */
class Lightning {
public:
    Lightning();
    ~Lightning();

    [[nodiscard]] bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);
    void shutdown();
    void recreatePipelines();
    /// The two pipelines both initialize() and recreatePipelines() need.
    void buildBoltPipeline(VkDevice device,
                           const VkPipelineShaderStageCreateInfo& vertStage,
                           const VkPipelineShaderStageCreateInfo& fragStage);
    void buildFlashPipeline(VkDevice device,
                            const VkPipelineShaderStageCreateInfo& vertStage,
                            const VkPipelineShaderStageCreateInfo& fragStage);

    void update(float deltaTime, const Camera& camera);
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);

    // Control
    void setEnabled(bool enabled);
    [[nodiscard]] bool isEnabled() const { return enabled; }

    void setIntensity(float intensity);  // 0.0 - 1.0 (affects frequency)
    [[nodiscard]] float getIntensity() const { return intensity; }

    // Trigger manual strike (for testing or scripted events)
    void triggerStrike(const glm::vec3& position);

private:
    struct LightningBolt {
        glm::vec3 startPos;
        glm::vec3 endPos;
        float lifetime;
        float maxLifetime;
        std::vector<glm::vec3> segments;  // Bolt path
        std::vector<glm::vec3> branches;  // Branch points
        float brightness;
        bool active;
    };

    struct Flash {
        float intensity;  // 0.0 - 1.0
        float lifetime;
        float maxLifetime;
        bool active;
    };

    void generateLightningBolt(LightningBolt& bolt);
    void generateBoltSegments(const glm::vec3& start, const glm::vec3& end,
                             std::vector<glm::vec3>& segments, int depth = 0);
    void updateBolts(float deltaTime);
    void updateFlash(float deltaTime);
    void spawnRandomStrike(const glm::vec3& cameraPos);

    void renderBolts(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);
    void renderFlash(VkCommandBuffer cmd);

    bool enabled = true;
    float intensity = 0.5f;  // Strike frequency multiplier

    // Timing
    float strikeTimer = 0.0f;
    float nextStrikeTime = 0.0f;

    // Active effects
    std::vector<LightningBolt> bolts;
    Flash flash;

    // Vulkan objects
    VkContext* vkCtx = nullptr;

    // Bolt pipeline + dynamic buffer
    VkPipeline boltPipeline = VK_NULL_HANDLE;
    VkPipelineLayout boltPipelineLayout = VK_NULL_HANDLE;
    ::VkBuffer boltDynamicVB = VK_NULL_HANDLE;
    VmaAllocation boltDynamicVBAlloc = VK_NULL_HANDLE;
    VmaAllocationInfo boltDynamicVBAllocInfo{};
    VkDeviceSize boltDynamicVBSize = 0;

    // Flash pipeline + static quad buffer
    VkPipeline flashPipeline = VK_NULL_HANDLE;
    VkPipelineLayout flashPipelineLayout = VK_NULL_HANDLE;
    ::VkBuffer flashQuadVB = VK_NULL_HANDLE;
    VmaAllocation flashQuadVBAlloc = VK_NULL_HANDLE;

    // Configuration
    static constexpr int MAX_BOLTS = 3;
    static constexpr float MIN_STRIKE_INTERVAL = 2.0f;
    static constexpr float MAX_STRIKE_INTERVAL = 8.0f;
    static constexpr float BOLT_LIFETIME = 0.15f;  // Quick flash
    static constexpr float FLASH_LIFETIME = 0.3f;
    static constexpr float STRIKE_DISTANCE = 200.0f;  // From camera
    static constexpr int MAX_SEGMENTS = 64;
    static constexpr float BRANCH_PROBABILITY = 0.3f;
};

} // namespace rendering
} // namespace wowee
