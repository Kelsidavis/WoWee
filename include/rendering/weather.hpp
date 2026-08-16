#pragma once

#include <algorithm>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <vector>
#include <unordered_map>

namespace wowee {
namespace rendering {

class Camera;
class VkContext;

/**
 * @brief Weather particle system for rain and snow
 *
 * Features:
 * - Rain particles (fast vertical drops)
 * - Snow particles (slow floating flakes)
 * - Particle recycling for efficiency
 * - Camera-relative positioning (follows player)
 * - Adjustable intensity (light, medium, heavy)
 * - Vulkan point-sprite rendering
 */
class Weather {
public:
    enum class Type {
        NONE,
        RAIN,
        SNOW,
        STORM
    };

    Weather();
    ~Weather();

    /**
     * @brief Initialize weather system
     * @param ctx Vulkan context
     * @param perFrameLayout Descriptor set layout for the per-frame UBO (set 0)
     * @return true if initialization succeeded
     */
    [[nodiscard]] bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout);
    void recreatePipelines();
    /// The pipeline state both initialize() and recreatePipelines() need.
    void buildPipelines(VkDevice device,
                        const VkPipelineShaderStageCreateInfo& vertStage,
                        const VkPipelineShaderStageCreateInfo& fragStage);

    /**
     * @brief Update weather particles
     * @param camera Camera for particle positioning
     * @param deltaTime Time since last frame
     */
    void update(const Camera& camera, float deltaTime);

    /**
     * @brief Render weather particles
     * @param cmd Command buffer to record into
     * @param perFrameSet Per-frame descriptor set (set 0, contains camera UBO)
     */
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);

    /**
     * @brief Set weather type
     */
    void setWeatherType(Type type) { weatherType = type; }
    [[nodiscard]] Type getWeatherType() const { return weatherType; }

    /**
     * @brief Set weather intensity (0.0 = none, 1.0 = heavy)
     */
    void setIntensity(float intensity);
    [[nodiscard]] float getIntensity() const { return intensity; }

    /// How much of the weather to draw, as a fraction of what the current
    /// weather would use. The game's Weather Detail setting, which is a
    /// quality knob rather than a forecast: it says how heavily to draw the
    /// rain, not how hard it is raining.
    ///
    /// Intensity is the weather itself and comes from the server; this
    /// multiplies it, so turning the setting down thins every storm without
    /// making any of them a different storm.
    void setDensityScale(float scale) {
        densityScale_ = std::clamp(scale, 0.0f, 1.0f);
    }
    [[nodiscard]] float densityScale() const { return densityScale_; }

    /**
     * @brief Enable or disable weather
     */
    void setEnabled(bool enabled) { this->enabled = enabled; }
    [[nodiscard]] bool isEnabled() const { return enabled; }

    /**
     * @brief Get active particle count
     */
    [[nodiscard]] int getParticleCount() const;

    /**
     * @brief Zone weather configuration
     * Provides default weather per zone for single-player mode.
     * When connected to a server, SMSG_WEATHER overrides these.
     */
    struct ZoneWeather {
        Type type = Type::NONE;
        float minIntensity = 0.0f;     // Min intensity (varies over time)
        float maxIntensity = 0.0f;     // Max intensity
        float probability = 0.0f;      // Chance of weather being active (0-1)
    };

    /**
     * @brief Set weather for a zone (used for zone-based weather configuration)
     */
    void setZoneWeather(uint32_t zoneId, Type type, float minIntensity, float maxIntensity, float probability);

    /**
     * @brief Update weather based on current zone (single-player mode)
     * @param zoneId Current zone ID
     * @param deltaTime Time since last frame
     */
    void updateZoneWeather(uint32_t zoneId, float deltaTime);

    /**
     * @brief Initialize default zone weather table
     */
    void initializeZoneWeatherDefaults();

    /**
     * @brief Clean up Vulkan resources
     */
    void shutdown();

private:
    struct Particle {
        glm::vec3 position;
        glm::vec3 velocity;
        float lifetime;
        float maxLifetime;
    };

    void resetParticles(const Camera& camera);
    void updateParticle(Particle& particle, const glm::vec3& cameraPos, float deltaTime);
    [[nodiscard]] glm::vec3 getRandomPosition(const glm::vec3& center) const;

    // Vulkan objects
    VkContext* vkCtx = nullptr;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

    // Dynamic mapped buffer for particle positions (updated every frame)
    ::VkBuffer dynamicVB = VK_NULL_HANDLE;
    VmaAllocation dynamicVBAlloc = VK_NULL_HANDLE;
    VmaAllocationInfo dynamicVBAllocInfo{};
    VkDeviceSize dynamicVBSize = 0;

    // Particles
    std::vector<Particle> particles;
    std::vector<glm::vec3> particlePositions;  // For rendering

    // Weather parameters
    bool enabled = false;
    Type weatherType = Type::NONE;
    float intensity = 0.5f;
    float densityScale_ = 1.0f;

    // Particle system parameters
    static constexpr int MAX_PARTICLES = 2000;
    static constexpr float SPAWN_VOLUME_SIZE = 100.0f;  // Size of spawn area around camera
    static constexpr float SPAWN_HEIGHT = 80.0f;        // Height above camera to spawn

    // Zone-based weather
    std::unordered_map<uint32_t, ZoneWeather> zoneWeatherTable_;
    uint32_t currentWeatherZone_ = 0;
    float zoneWeatherTimer_ = 0.0f;         // Time accumulator for weather cycling
    float zoneWeatherCycleDuration_ = 0.0f;  // Current cycle length
    bool zoneWeatherActive_ = false;         // Is zone weather currently active?
    float targetIntensity_ = 0.0f;           // Target intensity for smooth transitions
    bool zoneWeatherInitialized_ = false;
};

} // namespace rendering
} // namespace wowee
