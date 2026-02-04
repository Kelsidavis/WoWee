#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/scene.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/performance_hud.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/weather.hpp"
#include "rendering/swim_effects.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/shader.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/window.hpp"
#include "core/logger.hpp"
#include "game/world.hpp"
#include "game/zone_manager.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cctype>
#include <cmath>
#include <chrono>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace wowee {
namespace rendering {

struct EmoteInfo {
    uint32_t animId;
    bool loop;
    std::string text;
};

// AnimationData.dbc IDs for WotLK HumanMale emotes
// Reference: https://wowdev.wiki/M2/AnimationList
static const std::unordered_map<std::string, EmoteInfo> EMOTE_TABLE = {
    {"wave",    {67,  false, "waves."}},
    {"bow",     {66,  false, "bows down graciously."}},
    {"laugh",   {70,  false, "laughs."}},
    {"point",   {84,  false, "points over there."}},
    {"cheer",   {68,  false, "cheers!"}},
    {"dance",   {69,  true,  "begins to dance."}},
    {"kneel",   {75,  false, "kneels down."}},
    {"applaud", {80,  false, "applauds."}},
    {"shout",   {81,  false, "shouts."}},
    {"chicken", {78,  false, "clucks like a chicken."}},
    {"cry",     {77,  false, "cries."}},
    {"kiss",    {76,  false, "blows a kiss."}},
    {"roar",    {74,  false, "roars with bestial vigor."}},
    {"salute",  {113, false, "salutes."}},
    {"rude",    {73,  false, "makes a rude gesture."}},
    {"flex",    {82,  false, "flexes muscles."}},
    {"shy",     {83,  false, "acts shy."}},
    {"beg",     {79,  false, "begs everyone around."}},
    {"eat",     {61,  false, "begins to eat."}},
};

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

bool Renderer::initialize(core::Window* win) {
    window = win;
    LOG_INFO("Initializing renderer");

    // Create camera (in front of Stormwind gate, looking north)
    camera = std::make_unique<Camera>();
    camera->setPosition(glm::vec3(-8900.0f, -170.0f, 150.0f));
    camera->setRotation(0.0f, -5.0f);
    camera->setAspectRatio(window->getAspectRatio());
    camera->setFov(60.0f);

    // Create camera controller
    cameraController = std::make_unique<CameraController>(camera.get());
    cameraController->setUseWoWSpeed(true);  // Use realistic WoW movement speed
    cameraController->setMouseSensitivity(0.15f);

    // Create scene
    scene = std::make_unique<Scene>();

    // Create performance HUD
    performanceHUD = std::make_unique<PerformanceHUD>();
    performanceHUD->setPosition(PerformanceHUD::Position::TOP_LEFT);

    // Create water renderer
    waterRenderer = std::make_unique<WaterRenderer>();
    if (!waterRenderer->initialize()) {
        LOG_WARNING("Failed to initialize water renderer");
        waterRenderer.reset();
    }

    // Create skybox
    skybox = std::make_unique<Skybox>();
    if (!skybox->initialize()) {
        LOG_WARNING("Failed to initialize skybox");
        skybox.reset();
    } else {
        skybox->setTimeOfDay(12.0f);  // Start at noon
    }

    // Create celestial renderer (sun and moon)
    celestial = std::make_unique<Celestial>();
    if (!celestial->initialize()) {
        LOG_WARNING("Failed to initialize celestial renderer");
        celestial.reset();
    }

    // Create star field
    starField = std::make_unique<StarField>();
    if (!starField->initialize()) {
        LOG_WARNING("Failed to initialize star field");
        starField.reset();
    }

    // Create clouds
    clouds = std::make_unique<Clouds>();
    if (!clouds->initialize()) {
        LOG_WARNING("Failed to initialize clouds");
        clouds.reset();
    } else {
        clouds->setDensity(0.5f);  // Medium cloud coverage
    }

    // Create lens flare
    lensFlare = std::make_unique<LensFlare>();
    if (!lensFlare->initialize()) {
        LOG_WARNING("Failed to initialize lens flare");
        lensFlare.reset();
    }

    // Create weather system
    weather = std::make_unique<Weather>();
    if (!weather->initialize()) {
        LOG_WARNING("Failed to initialize weather");
        weather.reset();
    }

    // Create swim effects
    swimEffects = std::make_unique<SwimEffects>();
    if (!swimEffects->initialize()) {
        LOG_WARNING("Failed to initialize swim effects");
        swimEffects.reset();
    }

    // Create character renderer
    characterRenderer = std::make_unique<CharacterRenderer>();
    if (!characterRenderer->initialize()) {
        LOG_WARNING("Failed to initialize character renderer");
        characterRenderer.reset();
    }

    // Create WMO renderer
    wmoRenderer = std::make_unique<WMORenderer>();
    if (!wmoRenderer->initialize()) {
        LOG_WARNING("Failed to initialize WMO renderer");
        wmoRenderer.reset();
    }

    // Create minimap
    minimap = std::make_unique<Minimap>();
    if (!minimap->initialize(200)) {
        LOG_WARNING("Failed to initialize minimap");
        minimap.reset();
    }

    // Create M2 renderer (for doodads)
    m2Renderer = std::make_unique<M2Renderer>();
    // Note: M2 renderer needs asset manager, will be initialized when terrain loads

    // Create zone manager
    zoneManager = std::make_unique<game::ZoneManager>();
    zoneManager->initialize();

    // Create music manager (initialized later with asset manager)
    musicManager = std::make_unique<audio::MusicManager>();
    footstepManager = std::make_unique<audio::FootstepManager>();
    activitySoundManager = std::make_unique<audio::ActivitySoundManager>();

    // Underwater full-screen tint overlay (applies to all world geometry).
    underwaterOverlayShader = std::make_unique<Shader>();
    const char* overlayVS = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        void main() { gl_Position = vec4(aPos, 0.0, 1.0); }
    )";
    const char* overlayFS = R"(
        #version 330 core
        uniform vec4 uTint;
        out vec4 FragColor;
        void main() { FragColor = uTint; }
    )";
    if (!underwaterOverlayShader->loadFromSource(overlayVS, overlayFS)) {
        LOG_WARNING("Failed to initialize underwater overlay shader");
        underwaterOverlayShader.reset();
    } else {
        const float quadVerts[] = {
            -1.0f, -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f,  1.0f,  1.0f
        };
        glGenVertexArrays(1, &underwaterOverlayVAO);
        glGenBuffers(1, &underwaterOverlayVBO);
        glBindVertexArray(underwaterOverlayVAO);
        glBindBuffer(GL_ARRAY_BUFFER, underwaterOverlayVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    LOG_INFO("Renderer initialized");
    return true;
}

void Renderer::shutdown() {
    if (terrainManager) {
        terrainManager->unloadAll();
        terrainManager.reset();
    }

    if (terrainRenderer) {
        terrainRenderer->shutdown();
        terrainRenderer.reset();
    }

    if (waterRenderer) {
        waterRenderer->shutdown();
        waterRenderer.reset();
    }

    if (skybox) {
        skybox->shutdown();
        skybox.reset();
    }

    if (celestial) {
        celestial->shutdown();
        celestial.reset();
    }

    if (starField) {
        starField->shutdown();
        starField.reset();
    }

    if (clouds) {
        clouds.reset();
    }

    if (lensFlare) {
        lensFlare.reset();
    }

    if (weather) {
        weather.reset();
    }

    if (swimEffects) {
        swimEffects->shutdown();
        swimEffects.reset();
    }

    if (characterRenderer) {
        characterRenderer->shutdown();
        characterRenderer.reset();
    }

    if (wmoRenderer) {
        wmoRenderer->shutdown();
        wmoRenderer.reset();
    }

    if (m2Renderer) {
        m2Renderer->shutdown();
        m2Renderer.reset();
    }

    if (musicManager) {
        musicManager->shutdown();
        musicManager.reset();
    }
    if (footstepManager) {
        footstepManager->shutdown();
        footstepManager.reset();
    }
    if (activitySoundManager) {
        activitySoundManager->shutdown();
        activitySoundManager.reset();
    }
    if (underwaterOverlayVAO) {
        glDeleteVertexArrays(1, &underwaterOverlayVAO);
        underwaterOverlayVAO = 0;
    }
    if (underwaterOverlayVBO) {
        glDeleteBuffers(1, &underwaterOverlayVBO);
        underwaterOverlayVBO = 0;
    }
    underwaterOverlayShader.reset();

    zoneManager.reset();

    performanceHUD.reset();
    scene.reset();
    cameraController.reset();
    camera.reset();

    LOG_INFO("Renderer shutdown");
}

void Renderer::beginFrame() {
    // Black background (skybox will render over it)
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::endFrame() {
    // Nothing needed here for now
}

void Renderer::setCharacterFollow(uint32_t instanceId) {
    characterInstanceId = instanceId;
    if (cameraController && instanceId > 0) {
        cameraController->setFollowTarget(&characterPosition);
    }
}

void Renderer::updateCharacterAnimation() {
    // WoW WotLK AnimationData.dbc IDs
    constexpr uint32_t ANIM_STAND      = 0;
    constexpr uint32_t ANIM_WALK       = 4;
    constexpr uint32_t ANIM_RUN        = 5;
    // Candidate locomotion clips by common WotLK IDs.
    constexpr uint32_t ANIM_STRAFE_RUN_RIGHT  = 92;
    constexpr uint32_t ANIM_STRAFE_RUN_LEFT   = 93;
    constexpr uint32_t ANIM_STRAFE_WALK_LEFT  = 11;
    constexpr uint32_t ANIM_STRAFE_WALK_RIGHT = 12;
    constexpr uint32_t ANIM_BACKPEDAL         = 13;
    constexpr uint32_t ANIM_JUMP_START = 37;
    constexpr uint32_t ANIM_JUMP_MID   = 38;
    constexpr uint32_t ANIM_JUMP_END   = 39;
    constexpr uint32_t ANIM_SIT_DOWN   = 97;  // SitGround — transition to sitting
    constexpr uint32_t ANIM_SITTING    = 97;  // Hold on same animation (no separate idle)
    constexpr uint32_t ANIM_SWIM_IDLE  = 41;  // Treading water (SwimIdle)
    constexpr uint32_t ANIM_SWIM       = 42;  // Swimming forward (Swim)

    CharAnimState newState = charAnimState;

    bool moving = cameraController->isMoving();
    bool movingBackward = cameraController->isMovingBackward();
    bool strafeLeft = cameraController->isStrafingLeft();
    bool strafeRight = cameraController->isStrafingRight();
    bool anyStrafeLeft = strafeLeft && !strafeRight;
    bool anyStrafeRight = strafeRight && !strafeLeft;
    bool grounded = cameraController->isGrounded();
    bool jumping = cameraController->isJumping();
    bool sprinting = cameraController->isSprinting();
    bool sitting = cameraController->isSitting();
    bool swim = cameraController->isSwimming();

    switch (charAnimState) {
        case CharAnimState::IDLE:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (sitting && grounded) {
                newState = CharAnimState::SIT_DOWN;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            }
            break;

        case CharAnimState::WALK:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (!moving) {
                newState = CharAnimState::IDLE;
            } else if (sprinting) {
                newState = CharAnimState::RUN;
            }
            break;

        case CharAnimState::RUN:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (!moving) {
                newState = CharAnimState::IDLE;
            } else if (!sprinting) {
                newState = CharAnimState::WALK;
            }
            break;

        case CharAnimState::JUMP_START:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (grounded) {
                newState = CharAnimState::JUMP_END;
            } else {
                newState = CharAnimState::JUMP_MID;
            }
            break;

        case CharAnimState::JUMP_MID:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (grounded) {
                newState = CharAnimState::JUMP_END;
            }
            break;

        case CharAnimState::JUMP_END:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::SIT_DOWN:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!sitting) {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::SITTING:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!sitting) {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::EMOTE:
            if (swim) {
                cancelEmote();
                newState = CharAnimState::SWIM_IDLE;
            } else if (jumping || !grounded) {
                cancelEmote();
                newState = CharAnimState::JUMP_START;
            } else if (moving) {
                cancelEmote();
                newState = sprinting ? CharAnimState::RUN : CharAnimState::WALK;
            } else if (sitting) {
                cancelEmote();
                newState = CharAnimState::SIT_DOWN;
            }
            break;

        case CharAnimState::SWIM_IDLE:
            if (!swim) {
                newState = moving ? CharAnimState::WALK : CharAnimState::IDLE;
            } else if (moving) {
                newState = CharAnimState::SWIM;
            }
            break;

        case CharAnimState::SWIM:
            if (!swim) {
                newState = moving ? CharAnimState::WALK : CharAnimState::IDLE;
            } else if (!moving) {
                newState = CharAnimState::SWIM_IDLE;
            }
            break;
    }

    if (newState != charAnimState) {
        charAnimState = newState;
    }

    auto pickFirstAvailable = [&](std::initializer_list<uint32_t> candidates, uint32_t fallback) -> uint32_t {
        for (uint32_t id : candidates) {
            if (characterRenderer->hasAnimation(characterInstanceId, id)) {
                return id;
            }
        }
        return fallback;
    };

    uint32_t animId = ANIM_STAND;
    bool loop = true;

    switch (charAnimState) {
        case CharAnimState::IDLE:       animId = ANIM_STAND;      loop = true;  break;
        case CharAnimState::WALK:
            if (movingBackward) {
                animId = pickFirstAvailable({ANIM_BACKPEDAL}, ANIM_WALK);
            } else if (anyStrafeLeft) {
                animId = pickFirstAvailable({ANIM_STRAFE_WALK_LEFT, ANIM_STRAFE_RUN_LEFT}, ANIM_WALK);
            } else if (anyStrafeRight) {
                animId = pickFirstAvailable({ANIM_STRAFE_WALK_RIGHT, ANIM_STRAFE_RUN_RIGHT}, ANIM_WALK);
            } else {
                animId = ANIM_WALK;
            }
            loop = true;
            break;
        case CharAnimState::RUN:
            if (movingBackward) {
                animId = pickFirstAvailable({ANIM_BACKPEDAL}, ANIM_WALK);
            } else if (anyStrafeLeft) {
                animId = pickFirstAvailable({ANIM_STRAFE_RUN_LEFT}, ANIM_RUN);
            } else if (anyStrafeRight) {
                animId = pickFirstAvailable({ANIM_STRAFE_RUN_RIGHT}, ANIM_RUN);
            } else {
                animId = ANIM_RUN;
            }
            loop = true;
            break;
        case CharAnimState::JUMP_START: animId = ANIM_JUMP_START; loop = false; break;
        case CharAnimState::JUMP_MID:   animId = ANIM_JUMP_MID;   loop = false; break;
        case CharAnimState::JUMP_END:   animId = ANIM_JUMP_END;   loop = false; break;
        case CharAnimState::SIT_DOWN:   animId = ANIM_SIT_DOWN;   loop = false; break;
        case CharAnimState::SITTING:    animId = ANIM_SITTING;    loop = true;  break;
        case CharAnimState::EMOTE:      animId = emoteAnimId;     loop = emoteLoop; break;
        case CharAnimState::SWIM_IDLE:  animId = ANIM_SWIM_IDLE;  loop = true;  break;
        case CharAnimState::SWIM:       animId = ANIM_SWIM;       loop = true;  break;
    }

    uint32_t currentAnimId = 0;
    float currentAnimTimeMs = 0.0f;
    float currentAnimDurationMs = 0.0f;
    bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
    if (!haveState || currentAnimId != animId) {
        characterRenderer->playAnimation(characterInstanceId, animId, loop);
    }
}

void Renderer::playEmote(const std::string& emoteName) {
    auto it = EMOTE_TABLE.find(emoteName);
    if (it == EMOTE_TABLE.end()) return;

    const auto& info = it->second;
    emoteActive = true;
    emoteAnimId = info.animId;
    emoteLoop = info.loop;
    charAnimState = CharAnimState::EMOTE;

    if (characterRenderer && characterInstanceId > 0) {
        characterRenderer->playAnimation(characterInstanceId, emoteAnimId, emoteLoop);
    }
}

void Renderer::cancelEmote() {
    emoteActive = false;
    emoteAnimId = 0;
    emoteLoop = false;
}

std::string Renderer::getEmoteText(const std::string& emoteName) {
    auto it = EMOTE_TABLE.find(emoteName);
    if (it != EMOTE_TABLE.end()) {
        return it->second.text;
    }
    return "";
}

void Renderer::setTargetPosition(const glm::vec3* pos) {
    targetPosition = pos;
}

bool Renderer::isMoving() const {
    return cameraController && cameraController->isMoving();
}

bool Renderer::isFootstepAnimationState() const {
    return charAnimState == CharAnimState::WALK || charAnimState == CharAnimState::RUN;
}

bool Renderer::shouldTriggerFootstepEvent(uint32_t animationId, float animationTimeMs, float animationDurationMs) {
    if (animationDurationMs <= 1.0f) {
        footstepNormInitialized = false;
        return false;
    }

    float norm = std::fmod(animationTimeMs, animationDurationMs) / animationDurationMs;
    if (norm < 0.0f) norm += 1.0f;

    if (animationId != footstepLastAnimationId) {
        footstepLastAnimationId = animationId;
        footstepLastNormTime = norm;
        footstepNormInitialized = true;
        return false;
    }

    if (!footstepNormInitialized) {
        footstepNormInitialized = true;
        footstepLastNormTime = norm;
        return false;
    }

    auto crossed = [&](float eventNorm) {
        if (footstepLastNormTime <= norm) {
            return footstepLastNormTime < eventNorm && eventNorm <= norm;
        }
        return footstepLastNormTime < eventNorm || eventNorm <= norm;
    };

    bool trigger = crossed(0.22f) || crossed(0.72f);
    footstepLastNormTime = norm;
    return trigger;
}

audio::FootstepSurface Renderer::resolveFootstepSurface() const {
    if (!cameraController || !cameraController->isThirdPerson()) {
        return audio::FootstepSurface::STONE;
    }

    const glm::vec3& p = characterPosition;

    if (cameraController->isSwimming()) {
        return audio::FootstepSurface::WATER;
    }

    if (waterRenderer) {
        auto waterH = waterRenderer->getWaterHeightAt(p.x, p.y);
        if (waterH && p.z < (*waterH + 0.25f)) {
            return audio::FootstepSurface::WATER;
        }
    }

    if (wmoRenderer) {
        auto wmoFloor = wmoRenderer->getFloorHeight(p.x, p.y, p.z + 1.5f);
        auto terrainFloor = terrainManager ? terrainManager->getHeightAt(p.x, p.y) : std::nullopt;
        if (wmoFloor && (!terrainFloor || *wmoFloor >= *terrainFloor - 0.1f)) {
            return audio::FootstepSurface::STONE;
        }
    }

    if (terrainManager) {
        auto texture = terrainManager->getDominantTextureAt(p.x, p.y);
        if (texture) {
            std::string t = *texture;
            for (char& c : t) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (t.find("snow") != std::string::npos || t.find("ice") != std::string::npos) return audio::FootstepSurface::SNOW;
            if (t.find("grass") != std::string::npos || t.find("moss") != std::string::npos || t.find("leaf") != std::string::npos) return audio::FootstepSurface::GRASS;
            if (t.find("sand") != std::string::npos || t.find("dirt") != std::string::npos || t.find("mud") != std::string::npos) return audio::FootstepSurface::DIRT;
            if (t.find("wood") != std::string::npos || t.find("timber") != std::string::npos) return audio::FootstepSurface::WOOD;
            if (t.find("metal") != std::string::npos || t.find("iron") != std::string::npos) return audio::FootstepSurface::METAL;
            if (t.find("stone") != std::string::npos || t.find("rock") != std::string::npos || t.find("cobble") != std::string::npos || t.find("brick") != std::string::npos) return audio::FootstepSurface::STONE;
        }
    }

    return audio::FootstepSurface::STONE;
}

void Renderer::update(float deltaTime) {
    auto updateStart = std::chrono::steady_clock::now();
    if (wmoRenderer) wmoRenderer->resetQueryStats();
    if (m2Renderer) m2Renderer->resetQueryStats();

    if (cameraController) {
        auto cameraStart = std::chrono::steady_clock::now();
        cameraController->update(deltaTime);
        auto cameraEnd = std::chrono::steady_clock::now();
        lastCameraUpdateMs = std::chrono::duration<double, std::milli>(cameraEnd - cameraStart).count();
    } else {
        lastCameraUpdateMs = 0.0;
    }

    // Sync character model position/rotation and animation with follow target
    if (characterInstanceId > 0 && characterRenderer && cameraController && cameraController->isThirdPerson()) {
        characterRenderer->setInstancePosition(characterInstanceId, characterPosition);
        if (activitySoundManager) {
            std::string modelName;
            if (characterRenderer->getInstanceModelName(characterInstanceId, modelName)) {
                activitySoundManager->setCharacterVoiceProfile(modelName);
            }
        }

        // Movement-facing comes from camera controller and is decoupled from LMB orbit.
        if (cameraController->isMoving() || cameraController->isRightMouseHeld()) {
            characterYaw = cameraController->getFacingYaw();
        } else if (targetPosition && !emoteActive && !cameraController->isMoving()) {
            // Face target when idle
            glm::vec3 toTarget = *targetPosition - characterPosition;
            if (glm::length(glm::vec2(toTarget.x, toTarget.y)) > 0.1f) {
                float targetYaw = glm::degrees(std::atan2(toTarget.y, toTarget.x));
                // Smooth rotation toward target
                float diff = targetYaw - characterYaw;
                while (diff > 180.0f) diff -= 360.0f;
                while (diff < -180.0f) diff += 360.0f;
                float rotSpeed = 360.0f * deltaTime;
                if (std::abs(diff) < rotSpeed) {
                    characterYaw = targetYaw;
                } else {
                    characterYaw += (diff > 0 ? rotSpeed : -rotSpeed);
                }
            }
        }
        float yawRad = glm::radians(characterYaw);
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(0.0f, 0.0f, yawRad));

        // Update animation based on movement state
        updateCharacterAnimation();
    }

    // Update terrain streaming
    if (terrainManager && camera) {
        terrainManager->update(*camera, deltaTime);
    }

    // Update skybox time progression
    if (skybox) {
        skybox->update(deltaTime);
    }

    // Update star field twinkle
    if (starField) {
        starField->update(deltaTime);
    }

    // Update clouds animation
    if (clouds) {
        clouds->update(deltaTime);
    }

    // Update celestial (moon phase cycling)
    if (celestial) {
        celestial->update(deltaTime);
    }

    // Update weather particles
    if (weather && camera) {
        weather->update(*camera, deltaTime);
    }

    // Update swim effects
    if (swimEffects && camera && cameraController && waterRenderer) {
        swimEffects->update(*camera, *cameraController, *waterRenderer, deltaTime);
    }

    // Update character animations
    if (characterRenderer) {
        characterRenderer->update(deltaTime);
    }

    // Footsteps: animation-event driven + surface query at event time.
    if (footstepManager) {
        footstepManager->update(deltaTime);
        if (characterRenderer && characterInstanceId > 0 &&
            cameraController && cameraController->isThirdPerson() &&
            isFootstepAnimationState() && cameraController->isGrounded() &&
            !cameraController->isSwimming()) {
            uint32_t animId = 0;
            float animTimeMs = 0.0f;
            float animDurationMs = 0.0f;
            if (characterRenderer->getAnimationState(characterInstanceId, animId, animTimeMs, animDurationMs) &&
                shouldTriggerFootstepEvent(animId, animTimeMs, animDurationMs)) {
                footstepManager->playFootstep(resolveFootstepSurface(), cameraController->isSprinting());
            }
        } else {
            footstepNormInitialized = false;
        }
    }

    // Activity SFX: animation/state-driven jump, landing, and swim loops/splashes.
    if (activitySoundManager) {
        activitySoundManager->update(deltaTime);
        if (cameraController && cameraController->isThirdPerson()) {
            bool grounded = cameraController->isGrounded();
            bool jumping = cameraController->isJumping();
            bool falling = cameraController->isFalling();
            bool swimming = cameraController->isSwimming();
            bool moving = cameraController->isMoving();

            if (!sfxStateInitialized) {
                sfxPrevGrounded = grounded;
                sfxPrevJumping = jumping;
                sfxPrevFalling = falling;
                sfxPrevSwimming = swimming;
                sfxStateInitialized = true;
            }

            if (jumping && !sfxPrevJumping && !swimming) {
                activitySoundManager->playJump();
            }

            if (grounded && !sfxPrevGrounded) {
                bool hardLanding = sfxPrevFalling;
                activitySoundManager->playLanding(resolveFootstepSurface(), hardLanding);
            }

            if (swimming && !sfxPrevSwimming) {
                activitySoundManager->playWaterEnter();
            } else if (!swimming && sfxPrevSwimming) {
                activitySoundManager->playWaterExit();
            }

            activitySoundManager->setSwimmingState(swimming, moving);

            sfxPrevGrounded = grounded;
            sfxPrevJumping = jumping;
            sfxPrevFalling = falling;
            sfxPrevSwimming = swimming;
        } else {
            activitySoundManager->setSwimmingState(false, false);
            sfxStateInitialized = false;
        }
    }

    // Update M2 doodad animations
    if (m2Renderer) {
        m2Renderer->update(deltaTime);
    }

    // Update zone detection and music
    if (zoneManager && musicManager && terrainManager && camera) {
        // First check tile-based zone
        auto tile = terrainManager->getCurrentTile();
        uint32_t zoneId = zoneManager->getZoneId(tile.x, tile.y);



        // Override with WMO-based detection (e.g., inside Stormwind)
        if (wmoRenderer) {
            glm::vec3 camPos = camera->getPosition();
            uint32_t wmoModelId = 0;
            if (wmoRenderer->isInsideWMO(camPos.x, camPos.y, camPos.z, &wmoModelId)) {
                // Check if inside Stormwind WMO (model ID 10047)
                if (wmoModelId == 10047) {
                    zoneId = 1519;  // Stormwind City
                }
            }
        }

        if (zoneId != currentZoneId && zoneId != 0) {
            currentZoneId = zoneId;
            auto* info = zoneManager->getZoneInfo(zoneId);
            if (info) {
                currentZoneName = info->name;
                LOG_INFO("Entered zone: ", info->name);
                std::string music = zoneManager->getRandomMusic(zoneId);
                if (!music.empty()) {
                    musicManager->crossfadeTo(music);
                }
            }
        }

        musicManager->update(deltaTime);
    }

    // Update performance HUD
    if (performanceHUD) {
        performanceHUD->update(deltaTime);
    }

    auto updateEnd = std::chrono::steady_clock::now();
    lastUpdateMs = std::chrono::duration<double, std::milli>(updateEnd - updateStart).count();
}

void Renderer::renderWorld(game::World* world) {
    auto renderStart = std::chrono::steady_clock::now();
    lastTerrainRenderMs = 0.0;
    lastWMORenderMs = 0.0;
    lastM2RenderMs = 0.0;

    (void)world;  // Unused for now

    // Get time of day for sky-related rendering
    float timeOfDay = skybox ? skybox->getTimeOfDay() : 12.0f;
    bool underwater = false;
    bool canalUnderwater = false;

    // Render skybox first (furthest back)
    if (skybox && camera) {
        skybox->render(*camera, timeOfDay);
    }

    // Render stars after skybox
    if (starField && camera) {
        starField->render(*camera, timeOfDay);
    }

    // Render celestial bodies (sun/moon) after stars
    if (celestial && camera) {
        celestial->render(*camera, timeOfDay);
    }

    // Render clouds after celestial bodies
    if (clouds && camera) {
        clouds->render(*camera, timeOfDay);
    }

    // Render lens flare (screen-space effect, render after celestial bodies)
    if (lensFlare && camera && celestial) {
        glm::vec3 sunPosition = celestial->getSunPosition(timeOfDay);
        lensFlare->render(*camera, sunPosition, timeOfDay);
    }

    // Render terrain if loaded and enabled
    if (terrainEnabled && terrainLoaded && terrainRenderer && camera) {
        // Check if camera/character is underwater for fog override
        if (cameraController && cameraController->isSwimming() && waterRenderer && camera) {
            glm::vec3 camPos = camera->getPosition();
            auto waterH = waterRenderer->getWaterHeightAt(camPos.x, camPos.y);
            constexpr float MAX_UNDERWATER_DEPTH = 12.0f;
            // Require camera to be meaningfully below the surface before
            // underwater fog/tint kicks in (avoids "wrong plane" near surface).
            constexpr float UNDERWATER_ENTER_EPS = 0.45f;
            if (waterH &&
                camPos.z < (*waterH - UNDERWATER_ENTER_EPS) &&
                (*waterH - camPos.z) <= MAX_UNDERWATER_DEPTH) {
                underwater = true;
            }
        }

        if (underwater) {
            glm::vec3 camPos = camera->getPosition();
            std::optional<uint16_t> liquidType = waterRenderer ? waterRenderer->getWaterTypeAt(camPos.x, camPos.y) : std::nullopt;
            if (!liquidType && cameraController) {
                const glm::vec3* followTarget = cameraController->getFollowTarget();
                if (followTarget && waterRenderer) {
                    liquidType = waterRenderer->getWaterTypeAt(followTarget->x, followTarget->y);
                }
            }
            bool canalWater = liquidType && (*liquidType == 5 || *liquidType == 13 || *liquidType == 17);
            canalUnderwater = canalWater;

            float fogColor[3] = {0.04f, 0.12f, 0.22f};
            float fogStart = 8.0f;
            float fogEnd = 140.0f;
            if (canalWater) {
                fogColor[0] = 0.012f;
                fogColor[1] = 0.055f;
                fogColor[2] = 0.12f;
                fogStart = 2.5f;
                fogEnd = 55.0f;
            }
            terrainRenderer->setFog(fogColor, fogStart, fogEnd);
            glClearColor(fogColor[0], fogColor[1], fogColor[2], 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);  // Re-clear with underwater color
        } else if (skybox) {
            // Update terrain fog based on time of day (match sky color)
            glm::vec3 horizonColor = skybox->getHorizonColor(timeOfDay);
            float fogColorArray[3] = {horizonColor.r, horizonColor.g, horizonColor.b};
            terrainRenderer->setFog(fogColorArray, 400.0f, 1200.0f);
        }

        auto terrainStart = std::chrono::steady_clock::now();
        terrainRenderer->render(*camera);
        auto terrainEnd = std::chrono::steady_clock::now();
        lastTerrainRenderMs = std::chrono::duration<double, std::milli>(terrainEnd - terrainStart).count();

    }

    // Render weather particles (after terrain/water, before characters)
    if (weather && camera) {
        weather->render(*camera);
    }

    // Render swim effects (ripples and bubbles)
    if (swimEffects && camera) {
        swimEffects->render(*camera);
    }

    // Render characters (after weather)
    if (characterRenderer && camera) {
        glm::mat4 view = camera->getViewMatrix();
        glm::mat4 projection = camera->getProjectionMatrix();
        characterRenderer->render(*camera, view, projection);
    }

    // Render WMO buildings (after characters, before UI)
    if (wmoRenderer && camera) {
        glm::mat4 view = camera->getViewMatrix();
        glm::mat4 projection = camera->getProjectionMatrix();
        auto wmoStart = std::chrono::steady_clock::now();
        wmoRenderer->render(*camera, view, projection);
        auto wmoEnd = std::chrono::steady_clock::now();
        lastWMORenderMs = std::chrono::duration<double, std::milli>(wmoEnd - wmoStart).count();
    }

    // Render M2 doodads (trees, rocks, etc.)
    if (m2Renderer && camera) {
        glm::mat4 view = camera->getViewMatrix();
        glm::mat4 projection = camera->getProjectionMatrix();
        auto m2Start = std::chrono::steady_clock::now();
        m2Renderer->render(*camera, view, projection);
        auto m2End = std::chrono::steady_clock::now();
        lastM2RenderMs = std::chrono::duration<double, std::milli>(m2End - m2Start).count();
    }

    // Render water after opaque terrain/WMO/M2 so transparent surfaces remain visible.
    if (waterRenderer && camera) {
        static float time = 0.0f;
        time += 0.016f;  // Approximate frame time
        waterRenderer->render(*camera, time);
    }

    // Full-screen underwater tint so WMO/M2/characters also feel submerged.
    if (underwater && underwaterOverlayShader && underwaterOverlayVAO) {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        underwaterOverlayShader->use();
        if (canalUnderwater) {
            underwaterOverlayShader->setUniform("uTint", glm::vec4(0.01f, 0.05f, 0.11f, 0.50f));
        } else {
            underwaterOverlayShader->setUniform("uTint", glm::vec4(0.02f, 0.08f, 0.15f, 0.30f));
        }
        glBindVertexArray(underwaterOverlayVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    // Render minimap overlay
    if (minimap && camera && window) {
        minimap->render(*camera, window->getWidth(), window->getHeight());
    }

    auto renderEnd = std::chrono::steady_clock::now();
    lastRenderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
}

bool Renderer::loadTestTerrain(pipeline::AssetManager* assetManager, const std::string& adtPath) {
    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    LOG_INFO("Loading test terrain: ", adtPath);

    // Create terrain renderer if not already created
    if (!terrainRenderer) {
        terrainRenderer = std::make_unique<TerrainRenderer>();
        if (!terrainRenderer->initialize(assetManager)) {
            LOG_ERROR("Failed to initialize terrain renderer");
            terrainRenderer.reset();
            return false;
        }
    }

    // Create and initialize terrain manager
    if (!terrainManager) {
        terrainManager = std::make_unique<TerrainManager>();
        if (!terrainManager->initialize(assetManager, terrainRenderer.get())) {
            LOG_ERROR("Failed to initialize terrain manager");
            terrainManager.reset();
            return false;
        }
        // Set water renderer for terrain streaming
        if (waterRenderer) {
            terrainManager->setWaterRenderer(waterRenderer.get());
        }
        // Set M2 renderer for doodad loading during streaming
        if (m2Renderer) {
            terrainManager->setM2Renderer(m2Renderer.get());
        }
        // Set WMO renderer for building loading during streaming
        if (wmoRenderer) {
            terrainManager->setWMORenderer(wmoRenderer.get());
        }
        // Pass asset manager to character renderer for texture loading
        if (characterRenderer) {
            characterRenderer->setAssetManager(assetManager);
        }
        // Wire terrain renderer to minimap
        if (minimap) {
            minimap->setTerrainRenderer(terrainRenderer.get());
        }
        // Wire terrain manager, WMO renderer, and water renderer to camera controller
        if (cameraController) {
            cameraController->setTerrainManager(terrainManager.get());
            if (wmoRenderer) {
                cameraController->setWMORenderer(wmoRenderer.get());
            }
            if (m2Renderer) {
                cameraController->setM2Renderer(m2Renderer.get());
            }
            if (waterRenderer) {
                cameraController->setWaterRenderer(waterRenderer.get());
            }
        }
    }

    // Parse tile coordinates from ADT path
    // Format: World\Maps\{MapName}\{MapName}_{X}_{Y}.adt
    int tileX = 32, tileY = 49;  // defaults
    {
        // Find last path separator
        size_t lastSep = adtPath.find_last_of("\\/");
        if (lastSep != std::string::npos) {
            std::string filename = adtPath.substr(lastSep + 1);
            // Find first underscore after map name
            size_t firstUnderscore = filename.find('_');
            if (firstUnderscore != std::string::npos) {
                size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
                if (secondUnderscore != std::string::npos) {
                    size_t dot = filename.find('.', secondUnderscore);
                    if (dot != std::string::npos) {
                        tileX = std::stoi(filename.substr(firstUnderscore + 1, secondUnderscore - firstUnderscore - 1));
                        tileY = std::stoi(filename.substr(secondUnderscore + 1, dot - secondUnderscore - 1));
                    }
                }
            }
            // Extract map name
            std::string mapName = filename.substr(0, firstUnderscore != std::string::npos ? firstUnderscore : filename.size());
            terrainManager->setMapName(mapName);
        }
    }

    LOG_INFO("Loading initial tile [", tileX, ",", tileY, "] via terrain manager");

    // Load the initial tile through TerrainManager (properly tracked for streaming)
    if (!terrainManager->loadTile(tileX, tileY)) {
        LOG_ERROR("Failed to load initial tile [", tileX, ",", tileY, "]");
        return false;
    }

    terrainLoaded = true;

    // Initialize music manager with asset manager
    if (musicManager && assetManager && !cachedAssetManager) {
        musicManager->initialize(assetManager);
        if (footstepManager) {
            footstepManager->initialize(assetManager);
        }
        if (activitySoundManager) {
            activitySoundManager->initialize(assetManager);
        }
        cachedAssetManager = assetManager;
    }

    // Snap camera to ground now that terrain is loaded
    if (cameraController) {
        cameraController->reset();
    }

    LOG_INFO("Test terrain loaded successfully!");
    LOG_INFO("  Chunks: ", terrainRenderer->getChunkCount());
    LOG_INFO("  Triangles: ", terrainRenderer->getTriangleCount());

    return true;
}

void Renderer::setWireframeMode(bool enabled) {
    if (terrainRenderer) {
        terrainRenderer->setWireframe(enabled);
    }
}

bool Renderer::loadTerrainArea(const std::string& mapName, int centerX, int centerY, int radius) {
    // Create terrain renderer if not already created
    if (!terrainRenderer) {
        LOG_ERROR("Terrain renderer not initialized");
        return false;
    }

    // Create terrain manager if not already created
    if (!terrainManager) {
        terrainManager = std::make_unique<TerrainManager>();
        // Wire terrain manager to camera controller for grounding
        if (cameraController) {
            cameraController->setTerrainManager(terrainManager.get());
        }
    }

    LOG_INFO("Loading terrain area: ", mapName, " [", centerX, ",", centerY, "] radius=", radius);

    terrainManager->setMapName(mapName);
    terrainManager->setLoadRadius(radius);
    terrainManager->setUnloadRadius(radius + 1);

    // Load tiles in radius
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            int tileX = centerX + dx;
            int tileY = centerY + dy;

            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                terrainManager->loadTile(tileX, tileY);
            }
        }
    }

    terrainLoaded = true;

    // Initialize music manager with asset manager (if available from loadTestTerrain)
    if (musicManager && cachedAssetManager) {
        if (!musicManager->isInitialized()) {
            musicManager->initialize(cachedAssetManager);
        }
    }
    if (footstepManager && cachedAssetManager) {
        if (!footstepManager->isInitialized()) {
            footstepManager->initialize(cachedAssetManager);
        }
    }
    if (activitySoundManager && cachedAssetManager) {
        if (!activitySoundManager->isInitialized()) {
            activitySoundManager->initialize(cachedAssetManager);
        }
    }

    // Wire WMO, M2, and water renderer to camera controller
    if (cameraController && wmoRenderer) {
        cameraController->setWMORenderer(wmoRenderer.get());
    }
    if (cameraController && m2Renderer) {
        cameraController->setM2Renderer(m2Renderer.get());
    }
    if (cameraController && waterRenderer) {
        cameraController->setWaterRenderer(waterRenderer.get());
    }

    // Snap camera to ground now that terrain is loaded
    if (cameraController) {
        cameraController->reset();
    }

    LOG_INFO("Terrain area loaded: ", terrainManager->getLoadedTileCount(), " tiles");

    return true;
}

void Renderer::setTerrainStreaming(bool enabled) {
    if (terrainManager) {
        terrainManager->setStreamingEnabled(enabled);
        LOG_INFO("Terrain streaming: ", enabled ? "ON" : "OFF");
    }
}

void Renderer::renderHUD() {
    if (performanceHUD && camera) {
        performanceHUD->render(this, camera.get());
    }
}

} // namespace rendering
} // namespace wowee
