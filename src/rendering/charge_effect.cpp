#include "rendering/charge_effect.hpp"
#include "rendering/camera.hpp"
#include "rendering/shader.hpp"
#include "rendering/m2_renderer.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <random>
#include <cmath>

namespace wowee {
namespace rendering {

static std::mt19937& rng() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}

static float randFloat(float lo, float hi) {
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(rng());
}

ChargeEffect::ChargeEffect() = default;
ChargeEffect::~ChargeEffect() { shutdown(); }

bool ChargeEffect::initialize() {
    // ---- Ribbon trail shader ----
    ribbonShader_ = std::make_unique<Shader>();

    const char* ribbonVS = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in float aAlpha;
        layout (location = 2) in float aHeat;
        layout (location = 3) in float aHeight;

        uniform mat4 uView;
        uniform mat4 uProjection;

        out float vAlpha;
        out float vHeat;
        out float vHeight;

        void main() {
            gl_Position = uProjection * uView * vec4(aPos, 1.0);
            vAlpha = aAlpha;
            vHeat = aHeat;
            vHeight = aHeight;
        }
    )";

    const char* ribbonFS = R"(
        #version 330 core
        in float vAlpha;
        in float vHeat;
        in float vHeight;
        out vec4 FragColor;

        void main() {
            // Vertical gradient: top is red/opaque, bottom is transparent
            vec3 topColor  = vec3(0.9, 0.15, 0.05);  // Deep red at top
            vec3 midColor  = vec3(1.0, 0.5, 0.1);    // Orange in middle
            vec3 color = mix(midColor, topColor, vHeight);
            // Mix with heat (head vs tail along length)
            vec3 hotColor = vec3(1.0, 0.6, 0.15);
            color = mix(color, hotColor, vHeat * 0.4);

            // Bottom fades to transparent, top is opaque
            float vertAlpha = smoothstep(0.0, 0.4, vHeight);
            FragColor = vec4(color, vAlpha * vertAlpha * 0.7);
        }
    )";

    if (!ribbonShader_->loadFromSource(ribbonVS, ribbonFS)) {
        LOG_ERROR("Failed to create charge ribbon shader");
        return false;
    }

    glGenVertexArrays(1, &ribbonVao_);
    glGenBuffers(1, &ribbonVbo_);
    glBindVertexArray(ribbonVao_);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVbo_);
    // pos(3) + alpha(1) + heat(1) + height(1) = 6 floats
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(5 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glBindVertexArray(0);

    ribbonVerts_.reserve(MAX_TRAIL_POINTS * 2 * 6);

    // ---- Dust puff shader (small point sprites) ----
    dustShader_ = std::make_unique<Shader>();

    const char* dustVS = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in float aSize;
        layout (location = 2) in float aAlpha;

        uniform mat4 uView;
        uniform mat4 uProjection;

        out float vAlpha;

        void main() {
            gl_Position = uProjection * uView * vec4(aPos, 1.0);
            gl_PointSize = aSize;
            vAlpha = aAlpha;
        }
    )";

    const char* dustFS = R"(
        #version 330 core
        in float vAlpha;
        out vec4 FragColor;

        void main() {
            vec2 coord = gl_PointCoord - vec2(0.5);
            float dist = length(coord);
            if (dist > 0.5) discard;
            float alpha = smoothstep(0.5, 0.0, dist) * vAlpha;
            vec3 dustColor = vec3(0.65, 0.55, 0.40);
            FragColor = vec4(dustColor, alpha * 0.45);
        }
    )";

    if (!dustShader_->loadFromSource(dustVS, dustFS)) {
        LOG_ERROR("Failed to create charge dust shader");
        return false;
    }

    glGenVertexArrays(1, &dustVao_);
    glGenBuffers(1, &dustVbo_);
    glBindVertexArray(dustVao_);
    glBindBuffer(GL_ARRAY_BUFFER, dustVbo_);
    // pos(3) + size(1) + alpha(1) = 5 floats
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

    dustVerts_.reserve(MAX_DUST * 5);
    dustPuffs_.reserve(MAX_DUST);

    return true;
}

void ChargeEffect::shutdown() {
    if (ribbonVao_) glDeleteVertexArrays(1, &ribbonVao_);
    if (ribbonVbo_) glDeleteBuffers(1, &ribbonVbo_);
    ribbonVao_ = 0; ribbonVbo_ = 0;
    if (dustVao_) glDeleteVertexArrays(1, &dustVao_);
    if (dustVbo_) glDeleteBuffers(1, &dustVbo_);
    dustVao_ = 0; dustVbo_ = 0;
    trail_.clear();
    dustPuffs_.clear();
    ribbonShader_.reset();
    dustShader_.reset();
}

void ChargeEffect::tryLoadM2Models(M2Renderer* m2Renderer, pipeline::AssetManager* assets) {
    if (!m2Renderer || !assets) return;
    m2Renderer_ = m2Renderer;

    const char* casterPaths[] = {
        "Spells\\Charge_Caster.m2",
        "Spells\\WarriorCharge.m2",
        "Spells\\Charge\\Charge_Caster.m2",
        "Spells\\Dust_Medium.m2",
    };
    for (const char* path : casterPaths) {
        auto m2Data = assets->readFile(path);
        if (m2Data.empty()) continue;
        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.vertices.empty() && model.particleEmitters.empty()) continue;
        std::string skinPath = std::string(path);
        auto dotPos = skinPath.rfind('.');
        if (dotPos != std::string::npos) {
            std::string skinFile = skinPath.substr(0, dotPos) + "00.skin";
            auto skinData = assets->readFile(skinFile);
            if (!skinData.empty() && model.version >= 264)
                pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (m2Renderer_->loadModel(model, CASTER_MODEL_ID)) {
            casterModelLoaded_ = true;
            LOG_INFO("ChargeEffect: loaded caster model from ", path);
            break;
        }
    }

    const char* impactPaths[] = {
        "Spells\\Charge_Impact.m2",
        "Spells\\Charge\\Charge_Impact.m2",
        "Spells\\ImpactDust.m2",
    };
    for (const char* path : impactPaths) {
        auto m2Data = assets->readFile(path);
        if (m2Data.empty()) continue;
        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.vertices.empty() && model.particleEmitters.empty()) continue;
        std::string skinPath = std::string(path);
        auto dotPos = skinPath.rfind('.');
        if (dotPos != std::string::npos) {
            std::string skinFile = skinPath.substr(0, dotPos) + "00.skin";
            auto skinData = assets->readFile(skinFile);
            if (!skinData.empty() && model.version >= 264)
                pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (m2Renderer_->loadModel(model, IMPACT_MODEL_ID)) {
            impactModelLoaded_ = true;
            LOG_INFO("ChargeEffect: loaded impact model from ", path);
            break;
        }
    }
}

void ChargeEffect::start(const glm::vec3& position, const glm::vec3& direction) {
    emitting_ = true;
    dustAccum_ = 0.0f;
    trail_.clear();
    dustPuffs_.clear();
    lastEmitPos_ = position;

    // Spawn M2 caster effect
    if (casterModelLoaded_ && m2Renderer_) {
        activeCasterInstanceId_ = m2Renderer_->createInstance(
            CASTER_MODEL_ID, position, glm::vec3(0.0f), 1.0f);
    }

    // Seed the first trail point
    emit(position, direction);
}

void ChargeEffect::emit(const glm::vec3& position, const glm::vec3& direction) {
    if (!emitting_) return;

    // Move M2 caster with player
    if (activeCasterInstanceId_ != 0 && m2Renderer_) {
        m2Renderer_->setInstancePosition(activeCasterInstanceId_, position);
    }

    // Only add a new trail point if we've moved enough
    float dist = glm::length(position - lastEmitPos_);
    if (dist >= TRAIL_SPAWN_DIST || trail_.empty()) {
        // Ribbon is vertical: side vector points straight up
        glm::vec3 side = glm::vec3(0.0f, 0.0f, 1.0f);

        // Trail spawns at character's mid-height (ribbon extends above and below)
        glm::vec3 trailCenter = position + glm::vec3(0.0f, 0.0f, 1.0f);

        trail_.push_back({trailCenter, side, 0.0f});
        if (trail_.size() > MAX_TRAIL_POINTS) {
            trail_.pop_front();
        }
        lastEmitPos_ = position;
    }

    // Spawn dust puffs at feet
    glm::vec3 horizDir = glm::vec3(direction.x, direction.y, 0.0f);
    float horizLen = glm::length(horizDir);
    if (horizLen < 0.001f) return;
    glm::vec3 backDir = -horizDir / horizLen;
    glm::vec3 sideDir = glm::vec3(-backDir.y, backDir.x, 0.0f);

    dustAccum_ += 30.0f * 0.016f;
    while (dustAccum_ >= 1.0f && dustPuffs_.size() < MAX_DUST) {
        dustAccum_ -= 1.0f;
        DustPuff d;
        d.position = position + backDir * randFloat(0.0f, 0.6f) +
                     sideDir * randFloat(-0.4f, 0.4f) +
                     glm::vec3(0.0f, 0.0f, 0.1f);
        d.velocity = backDir * randFloat(0.5f, 2.0f) +
                     sideDir * randFloat(-0.3f, 0.3f) +
                     glm::vec3(0.0f, 0.0f, randFloat(0.8f, 2.0f));
        d.lifetime = 0.0f;
        d.maxLifetime = randFloat(0.3f, 0.5f);
        d.size = randFloat(5.0f, 10.0f);
        d.alpha = 1.0f;
        dustPuffs_.push_back(d);
    }
}

void ChargeEffect::stop() {
    emitting_ = false;

    if (activeCasterInstanceId_ != 0 && m2Renderer_) {
        m2Renderer_->removeInstance(activeCasterInstanceId_);
        activeCasterInstanceId_ = 0;
    }
}

void ChargeEffect::triggerImpact(const glm::vec3& position) {
    if (!impactModelLoaded_ || !m2Renderer_) return;
    uint32_t instanceId = m2Renderer_->createInstance(
        IMPACT_MODEL_ID, position, glm::vec3(0.0f), 1.0f);
    if (instanceId != 0) {
        activeImpacts_.push_back({instanceId, 0.0f});
    }
}

void ChargeEffect::update(float deltaTime) {
    // Age trail points and remove expired ones
    for (auto& tp : trail_) {
        tp.age += deltaTime;
    }
    while (!trail_.empty() && trail_.front().age >= TRAIL_LIFETIME) {
        trail_.pop_front();
    }

    // Update dust puffs
    for (auto it = dustPuffs_.begin(); it != dustPuffs_.end(); ) {
        it->lifetime += deltaTime;
        if (it->lifetime >= it->maxLifetime) {
            it = dustPuffs_.erase(it);
            continue;
        }
        it->position += it->velocity * deltaTime;
        it->velocity *= 0.93f;
        float t = it->lifetime / it->maxLifetime;
        it->alpha = 1.0f - t * t;
        it->size += deltaTime * 8.0f;
        ++it;
    }

    // Clean up expired M2 impacts
    for (auto it = activeImpacts_.begin(); it != activeImpacts_.end(); ) {
        it->elapsed += deltaTime;
        if (it->elapsed >= M2_EFFECT_DURATION) {
            if (m2Renderer_) m2Renderer_->removeInstance(it->instanceId);
            it = activeImpacts_.erase(it);
        } else {
            ++it;
        }
    }
}

void ChargeEffect::render(const Camera& camera) {
    // ---- Render ribbon trail as triangle strip ----
    if (trail_.size() >= 2 && ribbonShader_) {
        ribbonVerts_.clear();

        int n = static_cast<int>(trail_.size());
        for (int i = 0; i < n; i++) {
            const auto& tp = trail_[i];
            float ageFrac = tp.age / TRAIL_LIFETIME;      // 0 = fresh, 1 = about to expire
            float positionFrac = static_cast<float>(i) / static_cast<float>(n - 1);  // 0 = tail, 1 = head

            // Alpha: fade out by age and also taper toward the tail end
            float alpha = (1.0f - ageFrac) * std::min(positionFrac * 3.0f, 1.0f);
            // Heat: hotter near the head (character), cooler at the tail
            float heat = positionFrac;

            // Width tapers: thin at tail, full at head
            float width = TRAIL_HALF_WIDTH * std::min(positionFrac * 2.0f, 1.0f);

            // Two vertices: bottom (center - up*width) and top (center + up*width)
            glm::vec3 bottom = tp.center - tp.side * width;
            glm::vec3 top    = tp.center + tp.side * width;

            // Bottom vertex (height=0, more transparent)
            ribbonVerts_.push_back(bottom.x);
            ribbonVerts_.push_back(bottom.y);
            ribbonVerts_.push_back(bottom.z);
            ribbonVerts_.push_back(alpha);
            ribbonVerts_.push_back(heat);
            ribbonVerts_.push_back(0.0f);  // height = bottom

            // Top vertex (height=1, redder and more opaque)
            ribbonVerts_.push_back(top.x);
            ribbonVerts_.push_back(top.y);
            ribbonVerts_.push_back(top.z);
            ribbonVerts_.push_back(alpha);
            ribbonVerts_.push_back(heat);
            ribbonVerts_.push_back(1.0f);  // height = top
        }

        glBindBuffer(GL_ARRAY_BUFFER, ribbonVbo_);
        glBufferData(GL_ARRAY_BUFFER, ribbonVerts_.size() * sizeof(float),
                     ribbonVerts_.data(), GL_DYNAMIC_DRAW);

        ribbonShader_->use();
        ribbonShader_->setUniform("uView", camera.getViewMatrix());
        ribbonShader_->setUniform("uProjection", camera.getProjectionMatrix());

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // Additive blend for fiery glow
        glDepthMask(GL_FALSE);

        glBindVertexArray(ribbonVao_);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, static_cast<GLsizei>(n * 2));
        glBindVertexArray(0);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_TRUE);
    }

    // ---- Render dust puffs ----
    if (!dustPuffs_.empty() && dustShader_) {
        dustVerts_.clear();
        for (const auto& d : dustPuffs_) {
            dustVerts_.push_back(d.position.x);
            dustVerts_.push_back(d.position.y);
            dustVerts_.push_back(d.position.z);
            dustVerts_.push_back(d.size);
            dustVerts_.push_back(d.alpha);
        }

        glBindBuffer(GL_ARRAY_BUFFER, dustVbo_);
        glBufferData(GL_ARRAY_BUFFER, dustVerts_.size() * sizeof(float),
                     dustVerts_.data(), GL_DYNAMIC_DRAW);

        dustShader_->use();
        dustShader_->setUniform("uView", camera.getViewMatrix());
        dustShader_->setUniform("uProjection", camera.getProjectionMatrix());

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glEnable(GL_PROGRAM_POINT_SIZE);

        glBindVertexArray(dustVao_);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(dustPuffs_.size()));
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDisable(GL_PROGRAM_POINT_SIZE);
    }
}

} // namespace rendering
} // namespace wowee
