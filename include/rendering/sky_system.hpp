#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

namespace wowee {
namespace rendering {

class Camera;
class VkContext;
class Skybox;
class Celestial;
class StarField;
class Clouds;
class LensFlare;
class LightingManager;

/**
 * Sky rendering parameters (extracted from LightingManager)
 */
struct SkyParams {
    // Sun/moon positioning
    glm::vec3 directionalDir{0.0f, -1.0f, 0.3f};
    glm::vec3 sunColor{1.0f, 1.0f, 0.9f};

    // Sky colors (for skybox tinting/blending)
    glm::vec3 skyTopColor{0.5f, 0.7f, 1.0f};
    glm::vec3 skyMiddleColor{0.7f, 0.85f, 1.0f};
    glm::vec3 skyBand1Color{0.9f, 0.95f, 1.0f};
    glm::vec3 skyBand2Color{1.0f, 0.98f, 0.9f};

    // Atmospheric effects
    float cloudDensity = 0.0f;      // 0-1
    float fogDensity = 0.0f;        // 0-1
    float horizonGlow = 0.3f;       // 0-1
    float weatherIntensity = 0.0f;  // 0-1 (rain/snow intensity, attenuates lens flare)

    // Time
    float timeOfDay = 12.0f;    // 0-24 hours
    float gameTime = -1.0f;     // Server game time, hours since midnight (-1 = use fallback)

    // Skybox selection (future: from LightSkybox.dbc)
    uint32_t skyboxModelId = 0;
    bool skyboxHasStars = false;  // Does loaded skybox include baked stars?
    bool useOriginalSkybox = false; // Original camera-centered client M2 is active
};

/**
 * Unified sky rendering system
 *
 * Coordinates skybox (authoritative), celestial bodies (sun + 2 moons),
 * and fallback procedural stars. Driven by lighting system data.
 *
 * Architecture:
 * - Skybox is PRIMARY (includes baked stars from M2 models)
 * - Celestial renders sun + White Lady + Blue Child
 * - StarField is DEBUG/FALLBACK only (disabled when skybox has stars)
 */
class SkySystem {
public:
    SkySystem();
    ~SkySystem();

    /**
     * Initialize sky system components.
     * @param ctx            Vulkan context (required for Vulkan renderers)
     * @param perFrameLayout Descriptor set layout for set 0 (camera UBO)
     */
    bool initialize(VkContext* ctx = nullptr, VkDescriptorSetLayout perFrameLayout = VK_NULL_HANDLE);
    void shutdown();

    /**
     * Update sky system (time, moon phases, etc.)
     */
    void update(float deltaTime);

    /**
     * Render complete sky.
     * @param cmd         Active Vulkan command buffer
     * @param perFrameSet Per-frame descriptor set (set 0, camera UBO)
     * @param camera      Camera for legacy sub-renderers (lens flare, etc.)
     * @param params      Sky parameters from lighting system
     */
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet,
                const Camera& camera, const SkyParams& params);

    /**
     * Enable/disable procedural stars (DEBUG/FALLBACK)
     * Default: OFF (stars come from skybox)
     */
    void setProceduralStarsEnabled(bool enabled) { proceduralStarsEnabled_ = enabled; }
    bool isProceduralStarsEnabled() const { return proceduralStarsEnabled_; }

    /**
     * Enable/disable debug sky mode (forces procedural stars even with skybox)
     */
    void setDebugSkyMode(bool enabled) { debugSkyMode_ = enabled; }
    bool isDebugSkyMode() const { return debugSkyMode_; }

    /**
     * Get sun position in world space (for lens flare, shadows, etc.)
     */
    glm::vec3 getSunPosition(const SkyParams& params) const;

    /**
     * Enable/disable moon phase cycling
     */
    void setMoonPhaseCycling(bool enabled);

    /**
     * Set moon phases manually (0.0-1.0 each)
     */
    void setWhiteLadyPhase(float phase);
    void setBlueChildPhase(float phase);

    float getWhiteLadyPhase() const;
    float getBlueChildPhase() const;

    // Component accessors (for direct control if needed)
    Skybox*    getSkybox()    const { return skybox_.get(); }
    Celestial* getCelestial() const { return celestial_.get(); }
    StarField* getStarField() const { return starField_.get(); }
    Clouds*    getClouds()    const { return clouds_.get(); }
    LensFlare* getLensFlare() const { return lensFlare_.get(); }

private:
    std::unique_ptr<Skybox>    skybox_;      // Authoritative sky
    std::unique_ptr<Celestial> celestial_;   // Sun + 2 moons
    std::unique_ptr<StarField> starField_;   // Fallback procedural stars
    std::unique_ptr<Clouds>    clouds_;      // Cloud layer
    std::unique_ptr<LensFlare> lensFlare_;   // Sun lens flare

    bool proceduralStarsEnabled_ = false;
    bool debugSkyMode_ = false;
    bool initialized_ = false;
};

} // namespace rendering
} // namespace wowee
