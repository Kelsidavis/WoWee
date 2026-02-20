#include "rendering/levelup_effect.hpp"
#include "rendering/m2_renderer.hpp"
#include "pipeline/m2_loader.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {

LevelUpEffect::LevelUpEffect() = default;
LevelUpEffect::~LevelUpEffect() = default;

bool LevelUpEffect::loadModel(M2Renderer* m2Renderer,
                               const std::vector<uint8_t>& m2FileData,
                               const std::vector<uint8_t>& skinFileData) {
    if (!m2Renderer || m2FileData.empty()) return false;

    m2Renderer_ = m2Renderer;

    pipeline::M2Model model = pipeline::M2Loader::load(m2FileData);
    // Spell effect M2s may have no geometry (particle-only), so don't require isValid()
    if (model.vertices.empty() && model.particleEmitters.empty()) {
        LOG_WARNING("LevelUpEffect: M2 has no vertices and no particle emitters");
        return false;
    }

    if (!skinFileData.empty() && model.version >= 264) {
        pipeline::M2Loader::loadSkin(skinFileData, model);
    }

    if (!m2Renderer_->loadModel(model, MODEL_ID)) {
        LOG_WARNING("LevelUpEffect: failed to load model to GPU");
        return false;
    }

    modelLoaded_ = true;
    LOG_INFO("LevelUpEffect: loaded LevelUp.m2 — vertices=", model.vertices.size(),
             " indices=", model.indices.size(),
             " emitters=", model.particleEmitters.size(),
             " batches=", model.batches.size());
    return true;
}

void LevelUpEffect::trigger(const glm::vec3& position) {
    if (!modelLoaded_ || !m2Renderer_) return;

    uint32_t instanceId = m2Renderer_->createInstance(MODEL_ID, position,
                                                       glm::vec3(0.0f), 1.0f);
    if (instanceId == 0) {
        LOG_WARNING("LevelUpEffect: failed to create instance");
        return;
    }

    activeEffects_.push_back({instanceId, 0.0f});
}

void LevelUpEffect::update(float deltaTime) {
    if (activeEffects_.empty() || !m2Renderer_) return;

    for (auto it = activeEffects_.begin(); it != activeEffects_.end(); ) {
        it->elapsed += deltaTime;
        if (it->elapsed >= EFFECT_DURATION) {
            m2Renderer_->removeInstance(it->instanceId);
            it = activeEffects_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace rendering
} // namespace wowee
