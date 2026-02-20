#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <deque>
#include <cstdint>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {

class Camera;
class Shader;
class M2Renderer;

/// Renders a red-orange ribbon streak trailing behind the warrior during Charge,
/// plus small dust puffs at ground level.
class ChargeEffect {
public:
    ChargeEffect();
    ~ChargeEffect();

    bool initialize();
    void shutdown();

    /// Try to load M2 spell models (Charge_Caster.m2, etc.)
    void tryLoadM2Models(M2Renderer* m2Renderer, pipeline::AssetManager* assets);

    /// Start the trail (call once when charge begins)
    void start(const glm::vec3& position, const glm::vec3& direction);

    /// Feed current position each frame while charging
    void emit(const glm::vec3& position, const glm::vec3& direction);

    /// Stop adding trail points (existing ribbon fades out)
    void stop();

    /// Spawn M2 impact burst at target position
    void triggerImpact(const glm::vec3& position);

    void update(float deltaTime);
    void render(const Camera& camera);

    bool isActive() const { return emitting_ || !trail_.empty() || !dustPuffs_.empty(); }

private:
    // --- Ribbon trail ---
    struct TrailPoint {
        glm::vec3 center;      // World position of trail spine
        glm::vec3 side;        // Perpendicular direction (for ribbon width)
        float age;             // Seconds since spawned
    };

    static constexpr int MAX_TRAIL_POINTS = 64;
    static constexpr float TRAIL_LIFETIME = 0.5f;    // Seconds before trail point fades
    static constexpr float TRAIL_HALF_WIDTH = 0.8f;   // Half-width of ribbon
    static constexpr float TRAIL_SPAWN_DIST = 0.4f;   // Min distance between trail points
    std::deque<TrailPoint> trail_;

    GLuint ribbonVao_ = 0;
    GLuint ribbonVbo_ = 0;
    std::unique_ptr<Shader> ribbonShader_;
    std::vector<float> ribbonVerts_;  // pos(3) + alpha(1) + heat(1) = 5 floats per vert

    // --- Dust puffs (small point sprites at feet) ---
    struct DustPuff {
        glm::vec3 position;
        glm::vec3 velocity;
        float lifetime;
        float maxLifetime;
        float size;
        float alpha;
    };

    static constexpr int MAX_DUST = 80;
    std::vector<DustPuff> dustPuffs_;

    GLuint dustVao_ = 0;
    GLuint dustVbo_ = 0;
    std::unique_ptr<Shader> dustShader_;
    std::vector<float> dustVerts_;

    bool emitting_ = false;
    glm::vec3 lastEmitPos_{0.0f};
    float dustAccum_ = 0.0f;

    // --- M2 spell effect models (optional) ---
    static constexpr uint32_t CASTER_MODEL_ID = 999800;
    static constexpr uint32_t IMPACT_MODEL_ID = 999801;
    static constexpr float M2_EFFECT_DURATION = 2.0f;

    M2Renderer* m2Renderer_ = nullptr;
    bool casterModelLoaded_ = false;
    bool impactModelLoaded_ = false;

    uint32_t activeCasterInstanceId_ = 0;
    struct ActiveM2 {
        uint32_t instanceId;
        float elapsed;
    };
    std::vector<ActiveM2> activeImpacts_;
};

} // namespace rendering
} // namespace wowee
