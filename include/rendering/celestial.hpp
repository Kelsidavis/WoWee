#pragma once

#include <memory>
#include <glm/glm.hpp>

namespace wowee {
namespace rendering {

class Shader;
class Camera;

/**
 * Celestial body renderer
 *
 * Renders sun and moon that move across the sky based on time of day.
 * Sun rises at dawn, sets at dusk. Moon is visible at night.
 */
class Celestial {
public:
    Celestial();
    ~Celestial();

    bool initialize();
    void shutdown();

    /**
     * Render celestial bodies (sun and moon)
     * @param camera Camera for view matrix
     * @param timeOfDay Time of day in hours (0-24)
     * @param sunDir Optional sun direction from lighting system (normalized)
     * @param sunColor Optional sun color from lighting system
     * @param gameTime Optional server game time in seconds (for deterministic moon phases)
     */
    void render(const Camera& camera, float timeOfDay,
                const glm::vec3* sunDir = nullptr,
                const glm::vec3* sunColor = nullptr,
                float gameTime = -1.0f);

    /**
     * Enable/disable celestial rendering
     */
    void setEnabled(bool enabled) { renderingEnabled = enabled; }
    bool isEnabled() const { return renderingEnabled; }

    /**
     * Update celestial bodies (for moon phase cycling)
     */
    void update(float deltaTime);

    /**
     * Set White Lady phase (primary moon, 0.0 = new, 0.5 = full, 1.0 = new)
     */
    void setMoonPhase(float phase);
    float getMoonPhase() const { return whiteLadyPhase_; }

    /**
     * Set Blue Child phase (secondary moon, 0.0 = new, 0.5 = full, 1.0 = new)
     */
    void setBlueChildPhase(float phase);
    float getBlueChildPhase() const { return blueChildPhase_; }

    /**
     * Enable/disable automatic moon phase cycling
     */
    void setMoonPhaseCycling(bool enabled) { moonPhaseCycling = enabled; }
    bool isMoonPhaseCycling() const { return moonPhaseCycling; }

    /**
     * Enable/disable two-moon rendering (White Lady + Blue Child)
     */
    void setDualMoonMode(bool enabled) { dualMoonMode_ = enabled; }
    bool isDualMoonMode() const { return dualMoonMode_; }

    /**
     * Get sun position in world space
     */
    glm::vec3 getSunPosition(float timeOfDay) const;

    /**
     * Get moon position in world space
     */
    glm::vec3 getMoonPosition(float timeOfDay) const;

    /**
     * Get sun color (changes with time of day)
     */
    glm::vec3 getSunColor(float timeOfDay) const;

    /**
     * Get sun intensity (0-1, fades at dawn/dusk)
     */
    float getSunIntensity(float timeOfDay) const;

private:
    void createCelestialQuad();
    void destroyCelestialQuad();

    void renderSun(const Camera& camera, float timeOfDay,
                   const glm::vec3* sunDir = nullptr,
                   const glm::vec3* sunColor = nullptr);
    void renderMoon(const Camera& camera, float timeOfDay);  // White Lady (primary)
    void renderBlueChild(const Camera& camera, float timeOfDay);  // Blue Child (secondary)

    float calculateCelestialAngle(float timeOfDay, float riseTime, float setTime) const;

    /**
     * Compute moon phase from game time (deterministic)
     * @param gameTime Server game time in seconds
     * @param cycleDays Lunar cycle length in game days
     * @return Phase 0.0-1.0 (0=new, 0.5=full, 1.0=new)
     */
    float computePhaseFromGameTime(float gameTime, float cycleDays) const;

    /**
     * Update moon phases from game time (server-driven)
     */
    void updatePhasesFromGameTime(float gameTime);

    std::unique_ptr<Shader> celestialShader;

    uint32_t vao = 0;
    uint32_t vbo = 0;
    uint32_t ebo = 0;

    bool renderingEnabled = true;

    // Moon phase system (two moons in Azeroth lore)
    float whiteLadyPhase_ = 0.5f;  // 0.0-1.0 (0=new, 0.5=full) - primary moon
    float blueChildPhase_ = 0.25f;  // 0.0-1.0 (0=new, 0.5=full) - secondary moon
    bool moonPhaseCycling = true;
    float moonPhaseTimer = 0.0f;  // Fallback for deltaTime mode (development)
    float sunHazeTimer_ = 0.0f;   // Always-running timer for sun haze animation
    bool dualMoonMode_ = true;  // Default: render both moons (Azeroth-specific)

    // WoW lunar cycle constants (in game days)
    // WoW day = 24 real minutes, so these are ~realistic game-world cycles
    static constexpr float WHITE_LADY_CYCLE_DAYS = 30.0f;    // ~12 real hours for full cycle
    static constexpr float BLUE_CHILD_CYCLE_DAYS = 27.0f;    // ~10.8 real hours (slightly faster)
    static constexpr float MOON_CYCLE_DURATION = 240.0f;     // Fallback: 4 minutes (deltaTime mode)
};

} // namespace rendering
} // namespace wowee
