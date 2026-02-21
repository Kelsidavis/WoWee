#include "rendering/sky_system.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/camera.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {

SkySystem::SkySystem() = default;

SkySystem::~SkySystem() {
    shutdown();
}

bool SkySystem::initialize() {
    if (initialized_) {
        LOG_WARNING("SkySystem already initialized");
        return true;
    }

    LOG_INFO("Initializing sky system");

    // Initialize skybox (authoritative)
    skybox_ = std::make_unique<Skybox>();
    if (!skybox_->initialize()) {
        LOG_ERROR("Failed to initialize skybox");
        return false;
    }

    // Initialize celestial bodies (sun + 2 moons)
    celestial_ = std::make_unique<Celestial>();
    if (!celestial_->initialize()) {
        LOG_ERROR("Failed to initialize celestial bodies");
        return false;
    }

    // Initialize procedural stars (FALLBACK only)
    starField_ = std::make_unique<StarField>();
    if (!starField_->initialize()) {
        LOG_ERROR("Failed to initialize star field");
        return false;
    }
    // Default: disabled (skybox is authoritative)
    starField_->setEnabled(false);

    // Initialize clouds
    clouds_ = std::make_unique<Clouds>();
    if (!clouds_->initialize()) {
        LOG_ERROR("Failed to initialize clouds");
        return false;
    }

    // Initialize lens flare
    lensFlare_ = std::make_unique<LensFlare>();
    if (!lensFlare_->initialize()) {
        LOG_ERROR("Failed to initialize lens flare");
        return false;
    }

    initialized_ = true;
    LOG_INFO("Sky system initialized successfully");
    return true;
}

void SkySystem::shutdown() {
    if (!initialized_) {
        return;
    }

    LOG_INFO("Shutting down sky system");

    // Shutdown components that have explicit shutdown methods
    if (starField_) starField_->shutdown();
    if (celestial_) celestial_->shutdown();
    if (skybox_) skybox_->shutdown();

    // Reset all (destructors handle cleanup for clouds/lensFlare)
    lensFlare_.reset();
    clouds_.reset();
    starField_.reset();
    celestial_.reset();
    skybox_.reset();

    initialized_ = false;
}

void SkySystem::update(float deltaTime) {
    if (!initialized_) {
        return;
    }

    // Update time-based systems
    if (skybox_) skybox_->update(deltaTime);
    if (celestial_) celestial_->update(deltaTime);
    if (starField_) starField_->update(deltaTime);
}

void SkySystem::render(const Camera& camera, const SkyParams& params) {
    if (!initialized_) {
        return;
    }

    // Render skybox first (authoritative, includes baked stars)
    if (skybox_) {
        skybox_->render(camera, params.timeOfDay);
    }

    // Decide whether to render procedural stars
    bool renderProceduralStars = false;
    if (debugSkyMode_) {
        // Debug mode: always show procedural stars
        renderProceduralStars = true;
    } else if (proceduralStarsEnabled_) {
        // Fallback mode: show only if skybox doesn't have stars
        renderProceduralStars = !params.skyboxHasStars;
    }

    // Render procedural stars (FALLBACK or DEBUG only)
    if (renderProceduralStars && starField_) {
        starField_->setEnabled(true);
        starField_->render(camera, params.timeOfDay, params.cloudDensity, params.fogDensity);
    } else if (starField_) {
        starField_->setEnabled(false);
    }

    // Render celestial bodies (sun + White Lady + Blue Child)
    // Pass gameTime for deterministic moon phases
    if (celestial_) {
        celestial_->render(camera, params.timeOfDay, &params.directionalDir, &params.sunColor, params.gameTime);
    }

    // Render clouds
    if (clouds_) {
        clouds_->render(camera, params.timeOfDay);
    }

    // Render lens flare (sun glow effect)
    if (lensFlare_) {
        glm::vec3 sunPos = getSunPosition(params);
        lensFlare_->render(camera, sunPos, params.timeOfDay);
    }
}

glm::vec3 SkySystem::getSunPosition(const SkyParams& params) const {
    glm::vec3 dir = glm::normalize(params.directionalDir);
    if (glm::length(dir) < 0.0001f) {
        dir = glm::vec3(0.0f, 0.0f, -1.0f);
    }
    glm::vec3 sunDir = -dir;
    if (sunDir.z < 0.0f) {
        sunDir = dir;
    }
    glm::vec3 pos = sunDir * 800.0f;
    return pos;
}


void SkySystem::setMoonPhaseCycling(bool enabled) {
    if (celestial_) {
        celestial_->setMoonPhaseCycling(enabled);
    }
}

void SkySystem::setWhiteLadyPhase(float phase) {
    if (celestial_) {
        celestial_->setMoonPhase(phase);  // White Lady is primary moon
    }
}

void SkySystem::setBlueChildPhase(float phase) {
    if (celestial_) {
        celestial_->setBlueChildPhase(phase);
    }
}

float SkySystem::getWhiteLadyPhase() const {
    return celestial_ ? celestial_->getMoonPhase() : 0.5f;
}

float SkySystem::getBlueChildPhase() const {
    return celestial_ ? celestial_->getBlueChildPhase() : 0.25f;
}

} // namespace rendering
} // namespace wowee
