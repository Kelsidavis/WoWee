#pragma once

#include <string>
#include <deque>

namespace wowee {

namespace rendering {
class Renderer;
class Camera;
}

namespace rendering {

/**
 * Performance HUD for displaying real-time statistics
 *
 * Shows FPS, frame time, rendering stats, and terrain info
 */
class PerformanceHUD {
public:
    PerformanceHUD();
    ~PerformanceHUD();

    /**
     * Update HUD with latest frame time
     * @param deltaTime Time since last frame in seconds
     */
    void update(float deltaTime);

    /**
     * Render HUD using ImGui
     * @param renderer Renderer for accessing stats
     * @param camera Camera for position info
     */
    void render(const Renderer* renderer, const Camera* camera);

    /**
     * Enable/disable HUD display
     */
    void setEnabled(bool enabled) { this->enabled = enabled; }
    [[nodiscard]] bool isEnabled() const { return enabled; }

    /**
     * Toggle HUD visibility
     */
    void toggle() { enabled = !enabled; }

    /**
     * Set HUD position
     */
    enum class Position {
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT
    };
    void setPosition(Position pos) { position = pos; }

    /**
     * Enable/disable specific sections
     */
    void setShowFPS(bool show) { showFPS = show; }
    void setShowRenderer(bool show) { showRenderer = show; }
    void setShowTerrain(bool show) { showTerrain = show; }
    void setShowCamera(bool show) { showCamera = show; }
    void setShowControls(bool show) { showControls = show; }

private:
    /**
     * Calculate average FPS from frame time history
     */
    void calculateFPS();

    bool enabled = false;  // Disabled by default, press F1 to toggle
    Position position = Position::TOP_LEFT;

    // Section visibility
    bool showFPS = true;
    bool showRenderer = true;
    bool showTerrain = true;
    bool showCamera = true;
    bool showControls = true;

    // FPS tracking
    std::deque<float> frameTimeHistory;
    static constexpr size_t MAX_FRAME_HISTORY = 120;  // 2 seconds at 60 FPS
    float currentFPS = 0.0f;
    float averageFPS = 0.0f;
    float minFPS = 0.0f;
    float maxFPS = 0.0f;
    float frameTime = 0.0f;

    // Update timing
    float updateTimer = 0.0f;
    static constexpr float UPDATE_INTERVAL = 0.1f;  // Update stats every 0.1s
};

} // namespace rendering
} // namespace wowee
