#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace wowee {
namespace rendering {

class M2Renderer;

/// Manages spawning the real LevelUp.m2 spell effect at world positions.
/// The M2 model contains particle emitters that produce the golden pillar/ring effect.
class LevelUpEffect {
public:
    LevelUpEffect();
    ~LevelUpEffect();

    /// Load the LevelUp.m2 model (call once after M2Renderer is ready)
    /// @param m2Renderer The M2 renderer to register the model with
    /// @param m2FileData Raw bytes of Spell/LevelUp/LevelUp.m2
    /// @param skinFileData Raw bytes of Spell/LevelUp/LevelUp00.skin
    /// @return true if model loaded successfully
    bool loadModel(M2Renderer* m2Renderer,
                   const std::vector<uint8_t>& m2FileData,
                   const std::vector<uint8_t>& skinFileData);

    /// Trigger the level-up effect at a world position (render coords)
    void trigger(const glm::vec3& position);

    /// Remove expired effect instances
    void update(float deltaTime);

    bool isModelLoaded() const { return modelLoaded_; }

private:
    static constexpr float EFFECT_DURATION = 3.5f;
    static constexpr uint32_t MODEL_ID = 999900;  // Unique model ID for level-up effect

    struct ActiveEffect {
        uint32_t instanceId;
        float elapsed;
    };

    M2Renderer* m2Renderer_ = nullptr;
    bool modelLoaded_ = false;
    std::vector<ActiveEffect> activeEffects_;
};

} // namespace rendering
} // namespace wowee
