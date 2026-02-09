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
#include "rendering/mount_dust.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/shader.hpp"
#include "pipeline/m2_loader.hpp"
#include <algorithm>
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/application.hpp"
#include "core/window.hpp"
#include "core/logger.hpp"
#include "game/world.hpp"
#include "game/zone_manager.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
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
    uint32_t animId = 0;
    bool loop = false;
    std::string textNoTarget;
    std::string textTarget;
    std::string command;
};

static std::unordered_map<std::string, EmoteInfo> EMOTE_TABLE;
static bool emoteTableLoaded = false;

static std::vector<std::string> parseEmoteCommands(const std::string& raw) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : raw) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
            cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        } else if (!cur.empty()) {
            out.push_back(cur);
            cur.clear();
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static bool isLoopingEmote(const std::string& command) {
    static const std::unordered_set<std::string> kLooping = {
        "dance",
        "train",
    };
    return kLooping.find(command) != kLooping.end();
}

static void loadFallbackEmotes() {
    if (!EMOTE_TABLE.empty()) return;
    EMOTE_TABLE = {
        {"wave",    {67,  false, "You wave.", "You wave at %s.", "wave"}},
        {"bow",     {66,  false, "You bow down graciously.", "You bow down before %s.", "bow"}},
        {"laugh",   {70,  false, "You laugh.", "You laugh at %s.", "laugh"}},
        {"point",   {84,  false, "You point over yonder.", "You point at %s.", "point"}},
        {"cheer",   {68,  false, "You cheer!", "You cheer at %s.", "cheer"}},
        {"dance",   {69,  true,  "You burst into dance.", "You dance with %s.", "dance"}},
        {"kneel",   {75,  false, "You kneel down.", "You kneel before %s.", "kneel"}},
        {"applaud", {80,  false, "You applaud. Bravo!", "You applaud at %s. Bravo!", "applaud"}},
        {"shout",   {81,  false, "You shout.", "You shout at %s.", "shout"}},
        {"chicken", {78,  false, "With arms flapping, you strut around. Cluck, Cluck, Chicken!",
                     "With arms flapping, you strut around %s. Cluck, Cluck, Chicken!", "chicken"}},
        {"cry",     {77,  false, "You cry.", "You cry on %s's shoulder.", "cry"}},
        {"kiss",    {76,  false, "You blow a kiss into the wind.", "You blow a kiss to %s.", "kiss"}},
        {"roar",    {74,  false, "You roar with bestial vigor. So fierce!", "You roar with bestial vigor at %s. So fierce!", "roar"}},
        {"salute",  {113, false, "You salute.", "You salute %s with respect.", "salute"}},
        {"rude",    {73,  false, "You make a rude gesture.", "You make a rude gesture at %s.", "rude"}},
        {"flex",    {82,  false, "You flex your muscles. Oooooh so strong!", "You flex at %s. Oooooh so strong!", "flex"}},
        {"shy",     {83,  false, "You smile shyly.", "You smile shyly at %s.", "shy"}},
        {"beg",     {79,  false, "You beg everyone around you. How pathetic.", "You beg %s. How pathetic.", "beg"}},
        {"eat",     {61,  false, "You begin to eat.", "You begin to eat in front of %s.", "eat"}},
    };
}

static std::string replacePlaceholders(const std::string& text, const std::string* targetName) {
    if (text.empty()) return text;
    std::string out;
    out.reserve(text.size() + 16);
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 1 < text.size() && text[i + 1] == 's') {
            if (targetName && !targetName->empty()) out += *targetName;
            i++;
        } else {
            out.push_back(text[i]);
        }
    }
    return out;
}

static void loadEmotesFromDbc() {
    if (emoteTableLoaded) return;
    emoteTableLoaded = true;

    auto* assetManager = core::Application::getInstance().getAssetManager();
    if (!assetManager) {
        LOG_WARNING("Emotes: no AssetManager");
        loadFallbackEmotes();
        return;
    }

    auto emotesTextDbc = assetManager->loadDBC("EmotesText.dbc");
    auto emotesTextDataDbc = assetManager->loadDBC("EmotesTextData.dbc");
    if (!emotesTextDbc || !emotesTextDataDbc || !emotesTextDbc->isLoaded() || !emotesTextDataDbc->isLoaded()) {
        LOG_WARNING("Emotes: DBCs not available (EmotesText/EmotesTextData)");
        loadFallbackEmotes();
        return;
    }

    std::unordered_map<uint32_t, std::string> textData;
    textData.reserve(emotesTextDataDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDataDbc->getRecordCount(); ++r) {
        uint32_t id = emotesTextDataDbc->getUInt32(r, 0);
        std::string text = emotesTextDataDbc->getString(r, 1);
        if (!text.empty()) textData.emplace(id, std::move(text));
    }

    std::unordered_map<uint32_t, uint32_t> emoteIdToAnim;
    if (auto emotesDbc = assetManager->loadDBC("Emotes.dbc"); emotesDbc && emotesDbc->isLoaded()) {
        emoteIdToAnim.reserve(emotesDbc->getRecordCount());
        for (uint32_t r = 0; r < emotesDbc->getRecordCount(); ++r) {
            uint32_t emoteId = emotesDbc->getUInt32(r, 0);
            uint32_t animId = emotesDbc->getUInt32(r, 2);
            if (animId != 0) emoteIdToAnim[emoteId] = animId;
        }
    }

    EMOTE_TABLE.clear();
    EMOTE_TABLE.reserve(emotesTextDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDbc->getRecordCount(); ++r) {
        std::string cmdRaw = emotesTextDbc->getString(r, 1);
        if (cmdRaw.empty()) continue;

        uint32_t emoteRef = emotesTextDbc->getUInt32(r, 2);
        uint32_t animId = 0;
        auto animIt = emoteIdToAnim.find(emoteRef);
        if (animIt != emoteIdToAnim.end()) {
            animId = animIt->second;
        } else {
            animId = emoteRef;  // fallback if EmotesText stores animation id directly
        }

        uint32_t senderTargetTextId = emotesTextDbc->getUInt32(r, 5);  // unisex, target, sender
        uint32_t senderNoTargetTextId = emotesTextDbc->getUInt32(r, 9); // unisex, no target, sender

        std::string textTarget;
        std::string textNoTarget;
        if (auto it = textData.find(senderTargetTextId); it != textData.end()) {
            textTarget = it->second;
        }
        if (auto it = textData.find(senderNoTargetTextId); it != textData.end()) {
            textNoTarget = it->second;
        }

        for (const std::string& cmd : parseEmoteCommands(cmdRaw)) {
            if (cmd.empty()) continue;
            EmoteInfo info;
            info.animId = animId;
            info.loop = isLoopingEmote(cmd);
            info.textNoTarget = textNoTarget;
            info.textTarget = textTarget;
            info.command = cmd;
            EMOTE_TABLE.emplace(cmd, std::move(info));
        }
    }

    if (EMOTE_TABLE.empty()) {
        LOG_WARNING("Emotes: DBC loaded but no commands parsed, using fallback list");
        loadFallbackEmotes();
    } else {
        LOG_INFO("Emotes: loaded ", EMOTE_TABLE.size(), " commands from DBC");
    }
}

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

    // Create mount dust effects
    mountDust = std::make_unique<MountDust>();
    if (!mountDust->initialize()) {
        LOG_WARNING("Failed to initialize mount dust effects");
        mountDust.reset();
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

    // Initialize AudioEngine (singleton)
    if (!audio::AudioEngine::instance().initialize()) {
        LOG_WARNING("Failed to initialize AudioEngine - audio will be disabled");
    }

    // Create music manager (initialized later with asset manager)
    musicManager = std::make_unique<audio::MusicManager>();
    footstepManager = std::make_unique<audio::FootstepManager>();
    activitySoundManager = std::make_unique<audio::ActivitySoundManager>();
    mountSoundManager = std::make_unique<audio::MountSoundManager>();

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

    // Initialize post-process FBO pipeline
    initPostProcess(window->getWidth(), window->getHeight());

    // Initialize shadow map
    initShadowMap();

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

    // Shutdown AudioEngine singleton
    audio::AudioEngine::instance().shutdown();

    if (underwaterOverlayVAO) {
        glDeleteVertexArrays(1, &underwaterOverlayVAO);
        underwaterOverlayVAO = 0;
    }
    if (underwaterOverlayVBO) {
        glDeleteBuffers(1, &underwaterOverlayVBO);
        underwaterOverlayVBO = 0;
    }
    underwaterOverlayShader.reset();

    // Cleanup shadow map resources
    if (shadowFBO) { glDeleteFramebuffers(1, &shadowFBO); shadowFBO = 0; }
    if (shadowDepthTex) { glDeleteTextures(1, &shadowDepthTex); shadowDepthTex = 0; }
    if (shadowShaderProgram) { glDeleteProgram(shadowShaderProgram); shadowShaderProgram = 0; }

    shutdownPostProcess();

    zoneManager.reset();

    performanceHUD.reset();
    scene.reset();
    cameraController.reset();
    camera.reset();

    LOG_INFO("Renderer shutdown");
}

void Renderer::beginFrame() {
    // Resize post-process FBO if window size changed
    int w = window->getWidth();
    int h = window->getHeight();
    if (w != fbWidth || h != fbHeight) {
        resizePostProcess(w, h);
    }

    // Clear default framebuffer (login screen renders here directly)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
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

void Renderer::setMounted(uint32_t mountInstId, uint32_t mountDisplayId, float heightOffset) {
    mountInstanceId_ = mountInstId;
    mountHeightOffset_ = heightOffset;
    charAnimState = CharAnimState::MOUNT;
    if (cameraController) {
        cameraController->setMounted(true);
        cameraController->setMountHeightOffset(heightOffset);
    }

    // Notify mount sound manager
    if (mountSoundManager) {
        bool isFlying = taxiFlight_;  // Taxi flights are flying mounts
        mountSoundManager->onMount(mountDisplayId, isFlying);
    }
}

void Renderer::clearMount() {
    mountInstanceId_ = 0;
    mountHeightOffset_ = 0.0f;
    mountPitch_ = 0.0f;
    mountRoll_ = 0.0f;
    charAnimState = CharAnimState::IDLE;
    if (cameraController) {
        cameraController->setMounted(false);
        cameraController->setMountHeightOffset(0.0f);
    }

    // Notify mount sound manager
    if (mountSoundManager) {
        mountSoundManager->onDismount();
    }
}

uint32_t Renderer::resolveMeleeAnimId() {
    if (!characterRenderer || characterInstanceId == 0) {
        meleeAnimId = 0;
        meleeAnimDurationMs = 0.0f;
        return 0;
    }

    if (meleeAnimId != 0 && characterRenderer->hasAnimation(characterInstanceId, meleeAnimId)) {
        return meleeAnimId;
    }

    std::vector<pipeline::M2Sequence> sequences;
    if (!characterRenderer->getAnimationSequences(characterInstanceId, sequences)) {
        meleeAnimId = 0;
        meleeAnimDurationMs = 0.0f;
        return 0;
    }

    auto findDuration = [&](uint32_t id) -> float {
        for (const auto& seq : sequences) {
            if (seq.id == id && seq.duration > 0) {
                return static_cast<float>(seq.duration);
            }
        }
        return 0.0f;
    };

    // Select animation priority based on equipped weapon type
    // WoW inventory types: 17 = 2H weapon, 13/21 = 1H, 0 = unarmed
    // WoW anim IDs: 16 = unarmed, 17 = 1H attack, 18 = 2H attack
    const uint32_t* attackCandidates;
    size_t candidateCount;
    static const uint32_t candidates2H[] = {18, 17, 16, 19, 20, 21};
    static const uint32_t candidates1H[] = {17, 18, 16, 19, 20, 21};
    static const uint32_t candidatesUnarmed[] = {16, 17, 18, 19, 20, 21};
    if (equippedWeaponInvType_ == 17) { // INVTYPE_2HWEAPON
        attackCandidates = candidates2H;
        candidateCount = 6;
    } else if (equippedWeaponInvType_ == 0) {
        attackCandidates = candidatesUnarmed;
        candidateCount = 6;
    } else {
        attackCandidates = candidates1H;
        candidateCount = 6;
    }
    for (size_t ci = 0; ci < candidateCount; ci++) {
        uint32_t id = attackCandidates[ci];
        if (characterRenderer->hasAnimation(characterInstanceId, id)) {
            meleeAnimId = id;
            meleeAnimDurationMs = findDuration(id);
            return meleeAnimId;
        }
    }

    const uint32_t avoidIds[] = {0, 1, 4, 5, 11, 12, 13, 37, 38, 39, 41, 42, 97};
    auto isAvoid = [&](uint32_t id) -> bool {
        for (uint32_t avoid : avoidIds) {
            if (id == avoid) return true;
        }
        return false;
    };

    uint32_t bestId = 0;
    uint32_t bestDuration = 0;
    for (const auto& seq : sequences) {
        if (seq.duration == 0) continue;
        if (isAvoid(seq.id)) continue;
        if (seq.movingSpeed > 0.1f) continue;
        if (seq.duration < 150 || seq.duration > 2000) continue;
        if (bestId == 0 || seq.duration < bestDuration) {
            bestId = seq.id;
            bestDuration = seq.duration;
        }
    }

    if (bestId == 0) {
        for (const auto& seq : sequences) {
            if (seq.duration == 0) continue;
            if (isAvoid(seq.id)) continue;
            if (bestId == 0 || seq.duration < bestDuration) {
                bestId = seq.id;
                bestDuration = seq.duration;
            }
        }
    }

    meleeAnimId = bestId;
    meleeAnimDurationMs = static_cast<float>(bestDuration);
    return meleeAnimId;
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
    constexpr uint32_t ANIM_MOUNT      = 91;  // Seated on mount

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
    bool forceMelee = meleeSwingTimer > 0.0f && grounded && !swim;

    // When mounted, force MOUNT state and skip normal transitions
    if (isMounted()) {
        newState = CharAnimState::MOUNT;
        charAnimState = newState;

        // Play seated animation on player
        uint32_t currentAnimId = 0;
        float currentAnimTimeMs = 0.0f, currentAnimDurationMs = 0.0f;
        bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
        if (!haveState || currentAnimId != ANIM_MOUNT) {
            characterRenderer->playAnimation(characterInstanceId, ANIM_MOUNT, true);
        }

        // Sync mount instance position and rotation
        float mountBob = 0.0f;
        if (mountInstanceId_ > 0) {
            characterRenderer->setInstancePosition(mountInstanceId_, characterPosition);
            float yawRad = glm::radians(characterYaw);
            // Apply pitch (up/down), roll (banking), and yaw for realistic flight
            characterRenderer->setInstanceRotation(mountInstanceId_, glm::vec3(mountPitch_, mountRoll_, yawRad));

            // Drive mount model animation: idle when still, run when moving
            auto pickMountAnim = [&](std::initializer_list<uint32_t> candidates, uint32_t fallback) -> uint32_t {
                for (uint32_t id : candidates) {
                    if (characterRenderer->hasAnimation(mountInstanceId_, id)) {
                        return id;
                    }
                }
                return fallback;
            };

            uint32_t mountAnimId = ANIM_STAND;
            if (moving) {
                if (anyStrafeLeft) {
                    mountAnimId = pickMountAnim({ANIM_STRAFE_RUN_LEFT, ANIM_STRAFE_WALK_LEFT, ANIM_RUN}, ANIM_RUN);
                } else if (anyStrafeRight) {
                    mountAnimId = pickMountAnim({ANIM_STRAFE_RUN_RIGHT, ANIM_STRAFE_WALK_RIGHT, ANIM_RUN}, ANIM_RUN);
                } else if (movingBackward) {
                    mountAnimId = pickMountAnim({ANIM_BACKPEDAL}, ANIM_RUN);
                } else {
                    mountAnimId = ANIM_RUN;
                }
            }
            uint32_t curMountAnim = 0;
            float curMountTime = 0, curMountDur = 0;
            bool haveMountState = characterRenderer->getAnimationState(mountInstanceId_, curMountAnim, curMountTime, curMountDur);
            if (!haveMountState || curMountAnim != mountAnimId) {
                characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
            }

            // Rider bob: sinusoidal motion synced to mount's run animation
            if (moving && haveMountState && curMountDur > 1.0f) {
                float norm = std::fmod(curMountTime, curMountDur) / curMountDur;
                // One bounce per stride cycle
                float bobSpeed = taxiFlight_ ? 2.0f : 1.0f;
                mountBob = std::sin(norm * 2.0f * 3.14159f * bobSpeed) * 0.12f;
            }
        }

        // Character follows mount's full rotation (pitch, roll, yaw)
        // This keeps the character "glued" to the mount during banking/climbing
        float yawRad = glm::radians(characterYaw);

        // Create rotation matrix from mount's orientation
        glm::mat4 mountRotation = glm::mat4(1.0f);
        mountRotation = glm::rotate(mountRotation, yawRad, glm::vec3(0.0f, 0.0f, 1.0f));       // Yaw (Z)
        mountRotation = glm::rotate(mountRotation, mountRoll_, glm::vec3(1.0f, 0.0f, 0.0f));   // Roll (X)
        mountRotation = glm::rotate(mountRotation, mountPitch_, glm::vec3(0.0f, 1.0f, 0.0f));  // Pitch (Y)

        // Offset in mount's local space (rider sits above mount)
        glm::vec3 localOffset(0.0f, 0.0f, mountHeightOffset_ + mountBob);

        // Transform offset through mount's rotation to get world-space offset
        glm::vec3 worldOffset = glm::vec3(mountRotation * glm::vec4(localOffset, 0.0f));

        // Character position = mount position + rotated offset
        glm::vec3 playerPos = characterPosition + worldOffset;
        characterRenderer->setInstancePosition(characterInstanceId, playerPos);

        // Character rotates with mount (same pitch, roll, yaw)
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(mountPitch_, mountRoll_, yawRad));
        return;
    }

    if (!forceMelee) switch (charAnimState) {
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

        case CharAnimState::MELEE_SWING:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else if (sitting) {
                newState = CharAnimState::SIT_DOWN;
            } else {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::MOUNT:
            // If we got here, the mount state was cleared externally but the
            // animation state hasn't been reset yet. Fall back to normal logic.
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
            } else {
                newState = CharAnimState::IDLE;
            }
            break;
    }

    if (forceMelee) {
        newState = CharAnimState::MELEE_SWING;
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
                animId = pickFirstAvailable({ANIM_WALK, ANIM_RUN}, ANIM_STAND);
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
                animId = pickFirstAvailable({ANIM_RUN, ANIM_WALK}, ANIM_STAND);
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
        case CharAnimState::MELEE_SWING:
            animId = resolveMeleeAnimId();
            if (animId == 0) {
                animId = ANIM_STAND;
            }
            loop = false;
            break;
        case CharAnimState::MOUNT:      animId = ANIM_MOUNT;      loop = true;  break;
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
    loadEmotesFromDbc();
    auto it = EMOTE_TABLE.find(emoteName);
    if (it == EMOTE_TABLE.end()) return;

    const auto& info = it->second;
    if (info.animId == 0) return;
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

void Renderer::triggerMeleeSwing() {
    if (!characterRenderer || characterInstanceId == 0) return;
    if (meleeSwingCooldown > 0.0f) return;
    if (emoteActive) {
        cancelEmote();
    }
    resolveMeleeAnimId();
    meleeSwingCooldown = 0.1f;
    float durationSec = meleeAnimDurationMs > 0.0f ? meleeAnimDurationMs / 1000.0f : 0.6f;
    if (durationSec < 0.25f) durationSec = 0.25f;
    if (durationSec > 1.0f) durationSec = 1.0f;
    meleeSwingTimer = durationSec;
    if (activitySoundManager) {
        activitySoundManager->playMeleeSwing();
    }
}

std::string Renderer::getEmoteText(const std::string& emoteName, const std::string* targetName) {
    loadEmotesFromDbc();
    auto it = EMOTE_TABLE.find(emoteName);
    if (it != EMOTE_TABLE.end()) {
        const auto& info = it->second;
        const std::string& base = (targetName ? info.textTarget : info.textNoTarget);
        if (!base.empty()) {
            return replacePlaceholders(base, targetName);
        }
        if (targetName && !targetName->empty()) {
            return "You " + info.command + " at " + *targetName + ".";
        }
        return "You " + info.command + ".";
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

    // Cache footstep surface to avoid expensive queries every step
    // Only update if moved >1.5 units or timer expired (0.5s)
    float distSq = glm::dot(p - cachedFootstepPosition, p - cachedFootstepPosition);
    if (distSq < 2.25f && cachedFootstepUpdateTimer < 0.5f) {
        return cachedFootstepSurface;
    }

    // Update cache
    cachedFootstepPosition = p;
    cachedFootstepUpdateTimer = 0.0f;

    if (cameraController->isSwimming()) {
        cachedFootstepSurface = audio::FootstepSurface::WATER;
        return audio::FootstepSurface::WATER;
    }

    if (waterRenderer) {
        auto waterH = waterRenderer->getWaterHeightAt(p.x, p.y);
        if (waterH && p.z < (*waterH + 0.25f)) {
            cachedFootstepSurface = audio::FootstepSurface::WATER;
            return audio::FootstepSurface::WATER;
        }
    }

    if (wmoRenderer) {
        auto wmoFloor = wmoRenderer->getFloorHeight(p.x, p.y, p.z + 1.5f);
        auto terrainFloor = terrainManager ? terrainManager->getHeightAt(p.x, p.y) : std::nullopt;
        if (wmoFloor && (!terrainFloor || *wmoFloor >= *terrainFloor - 0.1f)) {
            cachedFootstepSurface = audio::FootstepSurface::STONE;
            return audio::FootstepSurface::STONE;
        }
    }

    // Determine surface type (expensive - only done when cache needs update)
    audio::FootstepSurface surface = audio::FootstepSurface::STONE;

    if (terrainManager) {
        auto texture = terrainManager->getDominantTextureAt(p.x, p.y);
        if (texture) {
            std::string t = *texture;
            for (char& c : t) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (t.find("snow") != std::string::npos || t.find("ice") != std::string::npos) surface = audio::FootstepSurface::SNOW;
            else if (t.find("grass") != std::string::npos || t.find("moss") != std::string::npos || t.find("leaf") != std::string::npos) surface = audio::FootstepSurface::GRASS;
            else if (t.find("sand") != std::string::npos || t.find("dirt") != std::string::npos || t.find("mud") != std::string::npos) surface = audio::FootstepSurface::DIRT;
            else if (t.find("wood") != std::string::npos || t.find("timber") != std::string::npos) surface = audio::FootstepSurface::WOOD;
            else if (t.find("metal") != std::string::npos || t.find("iron") != std::string::npos) surface = audio::FootstepSurface::METAL;
            else if (t.find("stone") != std::string::npos || t.find("rock") != std::string::npos || t.find("cobble") != std::string::npos || t.find("brick") != std::string::npos) surface = audio::FootstepSurface::STONE;
        }
    }

    cachedFootstepSurface = surface;
    return surface;
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

        // Update 3D audio listener position/orientation to match camera
        if (camera) {
            audio::AudioEngine::instance().setListenerPosition(camera->getPosition());
            audio::AudioEngine::instance().setListenerOrientation(camera->getForward(), camera->getUp());
        }
    } else {
        lastCameraUpdateMs = 0.0;
    }

    // Sync character model position/rotation and animation with follow target
    if (characterInstanceId > 0 && characterRenderer && cameraController && cameraController->isThirdPerson()) {
        if (meleeSwingCooldown > 0.0f) {
            meleeSwingCooldown = std::max(0.0f, meleeSwingCooldown - deltaTime);
        }
        if (meleeSwingTimer > 0.0f) {
            meleeSwingTimer = std::max(0.0f, meleeSwingTimer - deltaTime);
        }

        characterRenderer->setInstancePosition(characterInstanceId, characterPosition);
        if (activitySoundManager) {
            std::string modelName;
            if (characterRenderer->getInstanceModelName(characterInstanceId, modelName)) {
                activitySoundManager->setCharacterVoiceProfile(modelName);
            }
        }

        // Movement-facing comes from camera controller and is decoupled from LMB orbit.
        // During taxi flights, orientation is controlled by the flight path (not player input)
        if (taxiFlight_) {
            // Taxi flight: use orientation from flight path
            characterYaw = cameraController->getFacingYaw();
        } else if (cameraController->isMoving() || cameraController->isRightMouseHeld()) {
            characterYaw = cameraController->getFacingYaw();
        } else if (inCombat_ && targetPosition && !emoteActive && !isMounted()) {
            // Face target when in combat and idle
            glm::vec3 toTarget = *targetPosition - characterPosition;
            if (glm::length(glm::vec2(toTarget.x, toTarget.y)) > 0.1f) {
                float targetYaw = glm::degrees(std::atan2(toTarget.y, toTarget.x));
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

    // Update mount dust effects
    if (mountDust) {
        mountDust->update(deltaTime);

        // Spawn dust when mounted and moving on ground
        if (isMounted() && cameraController && !taxiFlight_) {
            bool isMoving = cameraController->isMoving();
            bool onGround = cameraController->isGrounded();

            if (isMoving && onGround) {
                // Calculate velocity from camera direction and speed
                glm::vec3 forward = camera->getForward();
                float speed = cameraController->getMovementSpeed();
                glm::vec3 velocity = forward * speed;
                velocity.z = 0.0f;  // Ignore vertical component

                // Spawn dust at mount's feet (slightly below character position)
                glm::vec3 dustPos = characterPosition - glm::vec3(0.0f, 0.0f, mountHeightOffset_ * 0.8f);
                mountDust->spawnDust(dustPos, velocity, isMoving);
            }
        }
    }

    // Update character animations
    if (characterRenderer) {
        characterRenderer->update(deltaTime);
    }

    // Update AudioEngine (cleanup finished sounds, etc.)
    audio::AudioEngine::instance().update(deltaTime);

    // Footsteps: animation-event driven + surface query at event time.
    if (footstepManager) {
        footstepManager->update(deltaTime);
        cachedFootstepUpdateTimer += deltaTime;  // Update surface cache timer
        bool canPlayFootsteps = characterRenderer && characterInstanceId > 0 &&
            cameraController && cameraController->isThirdPerson() &&
            cameraController->isGrounded() && !cameraController->isSwimming();

        if (canPlayFootsteps && isMounted() && mountInstanceId_ > 0 && !taxiFlight_) {
            // Mount footsteps: use mount's animation for timing
            uint32_t animId = 0;
            float animTimeMs = 0.0f, animDurationMs = 0.0f;
            if (characterRenderer->getAnimationState(mountInstanceId_, animId, animTimeMs, animDurationMs) &&
                animDurationMs > 1.0f && cameraController->isMoving()) {
                float norm = std::fmod(animTimeMs, animDurationMs) / animDurationMs;
                if (norm < 0.0f) norm += 1.0f;

                if (animId != mountFootstepLastAnimId) {
                    mountFootstepLastAnimId = animId;
                    mountFootstepLastNormTime = norm;
                    mountFootstepNormInitialized = true;
                } else if (!mountFootstepNormInitialized) {
                    mountFootstepNormInitialized = true;
                    mountFootstepLastNormTime = norm;
                } else {
                    // Mount gait: 2 hoofbeats per cycle (synced with animation)
                    auto crossed = [&](float eventNorm) {
                        if (mountFootstepLastNormTime <= norm) {
                            return mountFootstepLastNormTime < eventNorm && eventNorm <= norm;
                        }
                        return mountFootstepLastNormTime < eventNorm || eventNorm <= norm;
                    };
                    if (crossed(0.25f) || crossed(0.75f)) {
                        footstepManager->playFootstep(resolveFootstepSurface(), true);
                    }
                    mountFootstepLastNormTime = norm;
                }
            } else {
                mountFootstepNormInitialized = false;
            }
            footstepNormInitialized = false;  // Reset player footstep tracking
        } else if (canPlayFootsteps && isFootstepAnimationState()) {
            uint32_t animId = 0;
            float animTimeMs = 0.0f;
            float animDurationMs = 0.0f;
            if (characterRenderer->getAnimationState(characterInstanceId, animId, animTimeMs, animDurationMs) &&
                shouldTriggerFootstepEvent(animId, animTimeMs, animDurationMs)) {
                footstepManager->playFootstep(resolveFootstepSurface(), cameraController->isSprinting());
            }
            mountFootstepNormInitialized = false;
        } else {
            footstepNormInitialized = false;
            mountFootstepNormInitialized = false;
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

    // Mount ambient sounds: wing flaps, breathing, etc.
    if (mountSoundManager) {
        mountSoundManager->update(deltaTime);
        if (cameraController && isMounted()) {
            bool moving = cameraController->isMoving();
            bool flying = taxiFlight_ || !cameraController->isGrounded();  // Flying if taxi or airborne
            mountSoundManager->setMoving(moving);
            mountSoundManager->setFlying(flying);
        }
    }

    // Update M2 doodad animations (pass camera for frustum-culling bone computation)
    if (m2Renderer && camera) {
        m2Renderer->update(deltaTime, camera->getPosition(),
                           camera->getProjectionMatrix() * camera->getViewMatrix());
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

// ============================================================
// Selection Circle
// ============================================================

void Renderer::initSelectionCircle() {
    if (selCircleVAO) return;

    // Minimal shader: position + uniform MVP + color
    const char* vsSrc = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uMVP;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
        }
    )";
    const char* fsSrc = R"(
        #version 330 core
        uniform vec3 uColor;
        out vec4 FragColor;
        void main() {
            FragColor = vec4(uColor, 0.6);
        }
    )";

    auto compile = [](GLenum type, const char* src) -> GLuint {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        return s;
    };

    GLuint vs = compile(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fsSrc);
    selCircleShader = glCreateProgram();
    glAttachShader(selCircleShader, vs);
    glAttachShader(selCircleShader, fs);
    glLinkProgram(selCircleShader);
    glDeleteShader(vs);
    glDeleteShader(fs);

    // Build ring vertices (two concentric circles forming a strip)
    constexpr int SEGMENTS = 48;
    constexpr float INNER = 0.85f;
    constexpr float OUTER = 1.0f;
    std::vector<float> verts;
    for (int i = 0; i <= SEGMENTS; i++) {
        float angle = 2.0f * 3.14159265f * static_cast<float>(i) / static_cast<float>(SEGMENTS);
        float c = std::cos(angle), s = std::sin(angle);
        // Outer vertex
        verts.push_back(c * OUTER);
        verts.push_back(s * OUTER);
        verts.push_back(0.0f);
        // Inner vertex
        verts.push_back(c * INNER);
        verts.push_back(s * INNER);
        verts.push_back(0.0f);
    }
    selCircleVertCount = static_cast<int>((SEGMENTS + 1) * 2);

    glGenVertexArrays(1, &selCircleVAO);
    glGenBuffers(1, &selCircleVBO);
    glBindVertexArray(selCircleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, selCircleVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Renderer::setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color) {
    selCirclePos = pos;
    selCircleRadius = radius;
    selCircleColor = color;
    selCircleVisible = true;
}

void Renderer::clearSelectionCircle() {
    selCircleVisible = false;
}

void Renderer::renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection) {
    if (!selCircleVisible) return;
    initSelectionCircle();

    // Small Z offset to prevent clipping under terrain
    glm::vec3 raisedPos = selCirclePos;
    raisedPos.z += 0.15f;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), raisedPos);
    model = glm::scale(model, glm::vec3(selCircleRadius));

    glm::mat4 mvp = projection * view * model;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    glUseProgram(selCircleShader);
    glUniformMatrix4fv(glGetUniformLocation(selCircleShader, "uMVP"), 1, GL_FALSE, &mvp[0][0]);
    glUniform3fv(glGetUniformLocation(selCircleShader, "uColor"), 1, &selCircleColor[0]);

    glBindVertexArray(selCircleVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, selCircleVertCount);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
}

void Renderer::renderWorld(game::World* world) {
    auto renderStart = std::chrono::steady_clock::now();
    lastTerrainRenderMs = 0.0;
    lastWMORenderMs = 0.0;
    lastM2RenderMs = 0.0;

    // Shadow pass (before main scene)
    if (shadowsEnabled && shadowFBO && shadowShaderProgram && terrainLoaded) {
        renderShadowPass();
    } else {
        // Clear shadow maps when disabled
        if (terrainRenderer) terrainRenderer->clearShadowMap();
        if (wmoRenderer) wmoRenderer->clearShadowMap();
        if (m2Renderer) m2Renderer->clearShadowMap();
        if (characterRenderer) characterRenderer->clearShadowMap();
    }

    // Bind HDR scene framebuffer for world rendering
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFBO);
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    // Update fog across all renderers based on time of day (match sky color)
    if (skybox) {
        glm::vec3 horizonColor = skybox->getHorizonColor(timeOfDay);
        if (wmoRenderer) wmoRenderer->setFog(horizonColor, 100.0f, 600.0f);
        if (m2Renderer) m2Renderer->setFog(horizonColor, 100.0f, 600.0f);
        if (characterRenderer) characterRenderer->setFog(horizonColor, 100.0f, 600.0f);
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
            constexpr float UNDERWATER_ENTER_EPS = 1.10f;
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
            canalUnderwater = liquidType && (*liquidType == 5 || *liquidType == 13 || *liquidType == 17);
        }

        if (skybox) {
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
    // Skip during taxi flights for performance and visual clarity
    bool onTaxi = cameraController && cameraController->isOnTaxi();
    if (weather && camera && !onTaxi) {
        weather->render(*camera);
    }

    // Render swim effects (ripples and bubbles)
    if (swimEffects && camera) {
        swimEffects->render(*camera);
    }

    // Render mount dust effects
    if (mountDust && camera) {
        mountDust->render(*camera);
    }

    // Compute view/projection once for all sub-renderers
    const glm::mat4& view = camera ? camera->getViewMatrix() : glm::mat4(1.0f);
    const glm::mat4& projection = camera ? camera->getProjectionMatrix() : glm::mat4(1.0f);

    // Render characters (after weather)
    if (characterRenderer && camera) {
        characterRenderer->render(*camera, view, projection);
    }

    // Render selection circle under targeted creature
    renderSelectionCircle(view, projection);

    // Render WMO buildings (after characters, before UI)
    if (wmoRenderer && camera) {
        auto wmoStart = std::chrono::steady_clock::now();
        wmoRenderer->render(*camera, view, projection);
        auto wmoEnd = std::chrono::steady_clock::now();
        lastWMORenderMs = std::chrono::duration<double, std::milli>(wmoEnd - wmoStart).count();
    }

    // Render M2 doodads (trees, rocks, etc.)
    if (m2Renderer && camera) {
        // Dim M2 lighting when player is inside a WMO
        if (cameraController) {
            m2Renderer->setInsideInterior(cameraController->isInsideWMO());
            m2Renderer->setOnTaxi(cameraController->isOnTaxi());
        }
        auto m2Start = std::chrono::steady_clock::now();
        m2Renderer->render(*camera, view, projection);
        // Skip particle fog during taxi (expensive and visually distracting)
        if (!onTaxi) {
            m2Renderer->renderSmokeParticles(*camera, view, projection);
            m2Renderer->renderM2Particles(view, projection);
        }
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
    if (false && underwater && underwaterOverlayShader && underwaterOverlayVAO) {
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

    // --- Resolve MSAA → non-MSAA texture ---
    glBindFramebuffer(GL_READ_FRAMEBUFFER, sceneFBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolveFBO);
    glBlitFramebuffer(0, 0, fbWidth, fbHeight, 0, 0, fbWidth, fbHeight,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

    // --- Post-process: tonemap via fullscreen quad ---
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, window->getWidth(), window->getHeight());
    glDisable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    if (postProcessShader && screenQuadVAO) {
        postProcessShader->use();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, resolveColorTex);
        postProcessShader->setUniform("uScene", 0);
        glBindVertexArray(screenQuadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        postProcessShader->unuse();
    }

    // Render minimap overlay (after post-process so it's not overwritten)
    if (minimap && camera && window) {
        glm::vec3 minimapCenter = camera->getPosition();
        if (cameraController && cameraController->isThirdPerson()) {
            minimapCenter = characterPosition;
        }
        minimap->render(*camera, minimapCenter, window->getWidth(), window->getHeight());
    }

    glEnable(GL_DEPTH_TEST);

    auto renderEnd = std::chrono::steady_clock::now();
    lastRenderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
}

// ──────────────────────────────────────────────────────
// Post-process FBO helpers
// ──────────────────────────────────────────────────────

void Renderer::initPostProcess(int w, int h) {
    fbWidth = w;
    fbHeight = h;
    constexpr int SAMPLES = 4;

    // --- MSAA FBO (render target) ---
    glGenRenderbuffers(1, &sceneColorRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneColorRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_RGBA16F, w, h);

    glGenRenderbuffers(1, &sceneDepthRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneDepthRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_DEPTH_COMPONENT24, w, h);

    glGenFramebuffers(1, &sceneFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, sceneColorRBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, sceneDepthRBO);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("MSAA scene FBO incomplete!");
    }

    // --- Resolve FBO (non-MSAA, for post-process sampling) ---
    glGenTextures(1, &resolveColorTex);
    glBindTexture(GL_TEXTURE_2D, resolveColorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &resolveDepthTex);
    glBindTexture(GL_TEXTURE_2D, resolveDepthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenFramebuffers(1, &resolveFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveColorTex, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, resolveDepthTex, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("Resolve FBO incomplete!");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --- Fullscreen quad (triangle strip, pos + UV) ---
    const float quadVerts[] = {
        // pos (x,y)   uv (u,v)
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
    };
    glGenVertexArrays(1, &screenQuadVAO);
    glGenBuffers(1, &screenQuadVBO);
    glBindVertexArray(screenQuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, screenQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    // --- Post-process shader (Reinhard tonemap + gamma 2.2) ---
    const char* ppVS = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aUV;
        out vec2 vUV;
        void main() {
            vUV = aUV;
            gl_Position = vec4(aPos, 0.0, 1.0);
        }
    )";
    const char* ppFS = R"(
        #version 330 core
        in vec2 vUV;
        uniform sampler2D uScene;
        out vec4 FragColor;
        void main() {
            vec3 color = texture(uScene, vUV).rgb;
            // Shoulder tonemap: identity below 0.9, soft rolloff above
            vec3 excess = max(color - 0.9, 0.0);
            vec3 mapped = min(color, vec3(0.9)) + 0.1 * excess / (excess + 0.1);
            FragColor = vec4(mapped, 1.0);
        }
    )";
    postProcessShader = std::make_unique<Shader>();
    if (!postProcessShader->loadFromSource(ppVS, ppFS)) {
        LOG_ERROR("Failed to compile post-process shader");
        postProcessShader.reset();
    }

    LOG_INFO("Post-process FBO initialized (", w, "x", h, ")");
}

void Renderer::resizePostProcess(int w, int h) {
    if (w <= 0 || h <= 0) return;
    fbWidth = w;
    fbHeight = h;
    constexpr int SAMPLES = 4;

    // Resize MSAA renderbuffers
    glBindRenderbuffer(GL_RENDERBUFFER, sceneColorRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_RGBA16F, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneDepthRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_DEPTH_COMPONENT24, w, h);

    // Resize resolve textures
    glBindTexture(GL_TEXTURE_2D, resolveColorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
    glBindTexture(GL_TEXTURE_2D, resolveDepthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

    LOG_INFO("Post-process FBO resized (", w, "x", h, ")");
}

void Renderer::shutdownPostProcess() {
    if (sceneFBO) {
        glDeleteFramebuffers(1, &sceneFBO);
        sceneFBO = 0;
    }
    if (sceneColorRBO) {
        glDeleteRenderbuffers(1, &sceneColorRBO);
        sceneColorRBO = 0;
    }
    if (sceneDepthRBO) {
        glDeleteRenderbuffers(1, &sceneDepthRBO);
        sceneDepthRBO = 0;
    }
    if (resolveFBO) {
        glDeleteFramebuffers(1, &resolveFBO);
        resolveFBO = 0;
    }
    if (resolveColorTex) {
        glDeleteTextures(1, &resolveColorTex);
        resolveColorTex = 0;
    }
    if (resolveDepthTex) {
        glDeleteTextures(1, &resolveDepthTex);
        resolveDepthTex = 0;
    }
    if (screenQuadVAO) {
        glDeleteVertexArrays(1, &screenQuadVAO);
        screenQuadVAO = 0;
    }
    if (screenQuadVBO) {
        glDeleteBuffers(1, &screenQuadVBO);
        screenQuadVBO = 0;
    }
    postProcessShader.reset();
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
        // Wire asset manager to minimap for tile texture loading
        if (minimap) {
            minimap->setAssetManager(assetManager);
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
            if (minimap) {
                minimap->setMapName(mapName);
            }
        }
    }

    LOG_INFO("Enqueuing initial tile [", tileX, ",", tileY, "] via terrain manager");

    // Enqueue the initial tile for async loading (avoids long sync stalls)
    if (!terrainManager->enqueueTile(tileX, tileY)) {
        LOG_ERROR("Failed to enqueue initial tile [", tileX, ",", tileY, "]");
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

// ──────────────────────────────────────────────────────
// Shadow mapping helpers
// ──────────────────────────────────────────────────────

void Renderer::initShadowMap() {
    // Compile shadow shader
    shadowShaderProgram = compileShadowShader();
    if (!shadowShaderProgram) {
        LOG_ERROR("Failed to compile shadow shader");
        return;
    }

    // Create depth texture
    glGenTextures(1, &shadowDepthTex);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                 SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create depth-only FBO
    glGenFramebuffers(1, &shadowFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowDepthTex, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("Shadow FBO incomplete!");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    LOG_INFO("Shadow map initialized (", SHADOW_MAP_SIZE, "x", SHADOW_MAP_SIZE, ")");
}

uint32_t Renderer::compileShadowShader() {
    const char* vertSrc = R"(
        #version 330 core
        uniform mat4 uLightSpaceMatrix;
        uniform mat4 uModel;
        layout(location = 0) in vec3 aPos;
        layout(location = 2) in vec2 aTexCoord;
        layout(location = 3) in vec4 aBoneWeights;
        layout(location = 4) in vec4 aBoneIndicesF;
        uniform bool uUseBones;
        uniform mat4 uBones[200];
        out vec2 vTexCoord;
        void main() {
            vec3 pos = aPos;
            if (uUseBones) {
                ivec4 bi = ivec4(aBoneIndicesF);
                mat4 boneTransform = uBones[bi.x] * aBoneWeights.x
                                   + uBones[bi.y] * aBoneWeights.y
                                   + uBones[bi.z] * aBoneWeights.z
                                   + uBones[bi.w] * aBoneWeights.w;
                pos = vec3(boneTransform * vec4(aPos, 1.0));
            }
            vTexCoord = aTexCoord;
            gl_Position = uLightSpaceMatrix * uModel * vec4(pos, 1.0);
        }
    )";
    const char* fragSrc = R"(
        #version 330 core
        in vec2 vTexCoord;
        uniform bool uUseTexture;
        uniform sampler2D uTexture;
        uniform bool uAlphaTest;
        uniform float uShadowOpacity;

        float hash12(vec2 p) {
            vec3 p3 = fract(vec3(p.xyx) * 0.1031);
            p3 += dot(p3, p3.yzx + 33.33);
            return fract((p3.x + p3.y) * p3.z);
        }

        void main() {
            float opacity = clamp(uShadowOpacity, 0.0, 1.0);
            if (uUseTexture) {
                vec4 tex = texture(uTexture, vTexCoord);
                if (uAlphaTest && tex.a < 0.5) discard;
                opacity *= tex.a;
            }

            // Stochastic alpha for soft/translucent shadow casters (foliage).
            // Use UV-space hash so pattern stays stable with camera movement.
            if (opacity < 0.999) {
                float d = hash12(floor(vTexCoord * 4096.0));
                if (d > opacity) discard;
            }
        }
    )";

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertSrc, nullptr);
    glCompileShader(vs);
    GLint success;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(vs, 512, nullptr, log);
        LOG_ERROR("Shadow vertex shader error: ", log);
        glDeleteShader(vs);
        return 0;
    }

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragSrc, nullptr);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(fs, 512, nullptr, log);
        LOG_ERROR("Shadow fragment shader error: ", log);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, 512, nullptr, log);
        LOG_ERROR("Shadow shader link error: ", log);
        glDeleteProgram(program);
        program = 0;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

glm::mat4 Renderer::computeLightSpaceMatrix() {
    constexpr float kShadowHalfExtent = 180.0f;
    constexpr float kShadowLightDistance = 280.0f;
    constexpr float kShadowNearPlane = 1.0f;
    constexpr float kShadowFarPlane = 600.0f;

    // Sun direction matching WMO light dir
    glm::vec3 sunDir = glm::normalize(glm::vec3(-0.3f, -0.7f, -0.6f));

    // Keep a stable shadow focus center and only recentre occasionally.
    glm::vec3 desiredCenter = characterPosition;
    if (!shadowCenterInitialized) {
        shadowCenter = desiredCenter;
        shadowCenterInitialized = true;
    } else {
        constexpr float recenterThreshold = 30.0f; // world units
        if (std::abs(desiredCenter.x - shadowCenter.x) > recenterThreshold ||
            std::abs(desiredCenter.y - shadowCenter.y) > recenterThreshold) {
            shadowCenter.x = desiredCenter.x;
            shadowCenter.y = desiredCenter.y;
        }
        // Avoid vertical jitter from tiny terrain/camera height changes.
        if (std::abs(desiredCenter.z - shadowCenter.z) > 4.0f) {
            shadowCenter.z = desiredCenter.z;
        }
    }
    glm::vec3 center = shadowCenter;

    // Texel snapping: round center to shadow texel boundaries to prevent shimmer
    float halfExtent = kShadowHalfExtent;
    float texelWorld = (2.0f * halfExtent) / static_cast<float>(SHADOW_MAP_SIZE);

    // Build light view to get stable axes
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    // If sunDir is nearly parallel to up, pick a different up vector
    if (std::abs(glm::dot(sunDir, up)) > 0.99f) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    glm::mat4 lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);

    // Snap center in light space to texel grid
    glm::vec4 centerLS = lightView * glm::vec4(center, 1.0f);
    centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
    centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
    glm::vec4 snappedCenter = glm::inverse(lightView) * centerLS;
    center = glm::vec3(snappedCenter);
    shadowCenter = center;

    // Rebuild with snapped center
    lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);
    glm::mat4 lightProj = glm::ortho(-halfExtent, halfExtent, -halfExtent, halfExtent,
                                     kShadowNearPlane, kShadowFarPlane);

    return lightProj * lightView;
}

void Renderer::renderShadowPass() {
    constexpr float kShadowHalfExtent = 180.0f;
    constexpr float kShadowLightDistance = 280.0f;
    constexpr float kShadowNearPlane = 1.0f;
    constexpr float kShadowFarPlane = 600.0f;

    // Compute light space matrix
    lightSpaceMatrix = computeLightSpaceMatrix();

    // Bind shadow FBO
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO);
    glViewport(0, 0, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
    glClear(GL_DEPTH_BUFFER_BIT);

    // Caster-side bias: front-face culling + polygon offset
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    // Use shadow shader
    glUseProgram(shadowShaderProgram);
    GLint lsmLoc = glGetUniformLocation(shadowShaderProgram, "uLightSpaceMatrix");
    glUniformMatrix4fv(lsmLoc, 1, GL_FALSE, &lightSpaceMatrix[0][0]);
    GLint useTexLoc = glGetUniformLocation(shadowShaderProgram, "uUseTexture");
    GLint texLoc = glGetUniformLocation(shadowShaderProgram, "uTexture");
    GLint alphaTestLoc = glGetUniformLocation(shadowShaderProgram, "uAlphaTest");
    GLint opacityLoc = glGetUniformLocation(shadowShaderProgram, "uShadowOpacity");
    GLint useBonesLoc = glGetUniformLocation(shadowShaderProgram, "uUseBones");
    if (useTexLoc >= 0) glUniform1i(useTexLoc, 0);
    if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, 0);
    if (opacityLoc >= 0) glUniform1f(opacityLoc, 1.0f);
    if (useBonesLoc >= 0) glUniform1i(useBonesLoc, 0);
    if (texLoc >= 0) glUniform1i(texLoc, 0);

    // Render terrain into shadow map
    if (terrainRenderer) {
        terrainRenderer->renderShadow(shadowShaderProgram);
    }

    // Render WMO into shadow map
    if (wmoRenderer) {
        // WMO renderShadow takes separate view/proj matrices and a Shader ref.
        // We need to decompose our lightSpaceMatrix or use the raw shader program.
        // Since WMO::renderShadow sets uModel per instance, we use the shadow shader
        // directly by calling renderShadow with the light view/proj split.
        // For simplicity, compute the split:
        glm::vec3 sunDir = glm::normalize(glm::vec3(-0.3f, -0.7f, -0.6f));
        glm::vec3 center = shadowCenterInitialized ? shadowCenter : characterPosition;
        float halfExtent = kShadowHalfExtent;
        glm::vec3 up(0.0f, 0.0f, 1.0f);
        if (std::abs(glm::dot(sunDir, up)) > 0.99f) up = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::mat4 lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);
        glm::mat4 lightProj = glm::ortho(-halfExtent, halfExtent, -halfExtent, halfExtent,
                                         kShadowNearPlane, kShadowFarPlane);

        // WMO renderShadow needs a Shader reference — but it only uses setUniform("uModel", ...)
        // We'll create a thin wrapper. Actually, WMO's renderShadow takes a Shader& and calls
        // shadowShader.setUniform("uModel", ...). We need a Shader object wrapping our program.
        // Instead, let's use the lower-level approach: WMO renderShadow uses the shader passed in.
        // We need to temporarily wrap our GL program in a Shader object.
        Shader shadowShaderWrapper;
        shadowShaderWrapper.setProgram(shadowShaderProgram);
        wmoRenderer->renderShadow(lightView, lightProj, shadowShaderWrapper);
        shadowShaderWrapper.releaseProgram();  // Don't let wrapper delete our program
    }

    // Render M2 doodads into shadow map
    if (m2Renderer) {
        m2Renderer->renderShadow(shadowShaderProgram);
    }

    // Render characters into shadow map
    if (characterRenderer) {
        // Character shadows need less caster bias to avoid "floating" away from feet.
        glDisable(GL_POLYGON_OFFSET_FILL);
        glCullFace(GL_BACK);
        characterRenderer->renderShadow(lightSpaceMatrix);
        glCullFace(GL_FRONT);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.0f, 4.0f);
    }

    // Restore state
    glDisable(GL_POLYGON_OFFSET_FILL);
    glCullFace(GL_BACK);

    // Restore main viewport
    glViewport(0, 0, fbWidth, fbHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Distribute shadow map to all receivers
    if (terrainRenderer) terrainRenderer->setShadowMap(shadowDepthTex, lightSpaceMatrix);
    if (wmoRenderer) wmoRenderer->setShadowMap(shadowDepthTex, lightSpaceMatrix);
    if (m2Renderer) m2Renderer->setShadowMap(shadowDepthTex, lightSpaceMatrix);
    if (characterRenderer) characterRenderer->setShadowMap(shadowDepthTex, lightSpaceMatrix);
}

} // namespace rendering
} // namespace wowee
