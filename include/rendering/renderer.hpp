#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

namespace wowee {
namespace core { class Window; }
namespace game { class World; class ZoneManager; class GameHandler; }
namespace audio { class MusicManager; class FootstepManager; class ActivitySoundManager; class MountSoundManager; class NpcVoiceManager; class AmbientSoundManager; class UiSoundManager; class CombatSoundManager; class SpellSoundManager; class MovementSoundManager; enum class FootstepSurface : uint8_t; enum class VoiceType; }
namespace pipeline { class AssetManager; }

namespace rendering {

class Camera;
class CameraController;
class Scene;
class TerrainRenderer;
class TerrainManager;
class PerformanceHUD;
class WaterRenderer;
class Skybox;
class Celestial;
class StarField;
class Clouds;
class LensFlare;
class Weather;
class LightingManager;
class SkySystem;
class SwimEffects;
class MountDust;
class CharacterRenderer;
class WMORenderer;
class M2Renderer;
class Minimap;
class QuestMarkerRenderer;
class Shader;

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize(core::Window* window);
    void shutdown();

    void beginFrame();
    void endFrame();

    void renderWorld(game::World* world, game::GameHandler* gameHandler = nullptr);

    /**
     * Update renderer (camera, etc.)
     */
    void update(float deltaTime);

    /**
     * Load test terrain for debugging
     * @param assetManager Asset manager to load terrain data
     * @param adtPath Path to ADT file (e.g., "World\\Maps\\Azeroth\\Azeroth_32_49.adt")
     */
    bool loadTestTerrain(pipeline::AssetManager* assetManager, const std::string& adtPath);

    /**
     * Enable/disable terrain rendering
     */
    void setTerrainEnabled(bool enabled) { terrainEnabled = enabled; }

    /**
     * Enable/disable wireframe mode
     */
    void setWireframeMode(bool enabled);

    /**
     * Load terrain tiles around position
     * @param mapName Map name (e.g., "Azeroth", "Kalimdor")
     * @param centerX Center tile X coordinate
     * @param centerY Center tile Y coordinate
     * @param radius Load radius in tiles
     */
    bool loadTerrainArea(const std::string& mapName, int centerX, int centerY, int radius = 1);

    /**
     * Enable/disable terrain streaming
     */
    void setTerrainStreaming(bool enabled);

    /**
     * Render performance HUD
     */
    void renderHUD();

    Camera* getCamera() { return camera.get(); }
    CameraController* getCameraController() { return cameraController.get(); }
    Scene* getScene() { return scene.get(); }
    TerrainRenderer* getTerrainRenderer() const { return terrainRenderer.get(); }
    TerrainManager* getTerrainManager() const { return terrainManager.get(); }
    PerformanceHUD* getPerformanceHUD() { return performanceHUD.get(); }
    WaterRenderer* getWaterRenderer() const { return waterRenderer.get(); }
    Skybox* getSkybox() const { return skybox.get(); }
    Celestial* getCelestial() const { return celestial.get(); }
    StarField* getStarField() const { return starField.get(); }
    Clouds* getClouds() const { return clouds.get(); }
    LensFlare* getLensFlare() const { return lensFlare.get(); }
    Weather* getWeather() const { return weather.get(); }
    CharacterRenderer* getCharacterRenderer() const { return characterRenderer.get(); }
    WMORenderer* getWMORenderer() const { return wmoRenderer.get(); }
    M2Renderer* getM2Renderer() const { return m2Renderer.get(); }
    Minimap* getMinimap() const { return minimap.get(); }
    QuestMarkerRenderer* getQuestMarkerRenderer() const { return questMarkerRenderer.get(); }
    SkySystem* getSkySystem() const { return skySystem.get(); }
    const std::string& getCurrentZoneName() const { return currentZoneName; }

    // Third-person character follow
    void setCharacterFollow(uint32_t instanceId);
    glm::vec3& getCharacterPosition() { return characterPosition; }
    uint32_t getCharacterInstanceId() const { return characterInstanceId; }
    float getCharacterYaw() const { return characterYaw; }
    void setCharacterYaw(float yawDeg) { characterYaw = yawDeg; }

    // Emote support
    void playEmote(const std::string& emoteName);
    void cancelEmote();
    bool isEmoteActive() const { return emoteActive; }
    static std::string getEmoteText(const std::string& emoteName, const std::string* targetName = nullptr);

    // Targeting support
    void setTargetPosition(const glm::vec3* pos);
    void setInCombat(bool combat) { inCombat_ = combat; }
    bool isMoving() const;
    void triggerMeleeSwing();
    void setEquippedWeaponType(uint32_t inventoryType) { equippedWeaponInvType_ = inventoryType; meleeAnimId = 0; }

    // Mount rendering
    void setMounted(uint32_t mountInstId, uint32_t mountDisplayId, float heightOffset);
    void setTaxiFlight(bool onTaxi) { taxiFlight_ = onTaxi; }
    void setMountPitchRoll(float pitch, float roll) { mountPitch_ = pitch; mountRoll_ = roll; }
    void clearMount();
    bool isMounted() const { return mountInstanceId_ != 0; }

    // Selection circle for targeted entity
    void setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color);
    void clearSelectionCircle();

    // CPU timing stats (milliseconds, last frame).
    double getLastUpdateMs() const { return lastUpdateMs; }
    double getLastRenderMs() const { return lastRenderMs; }
    double getLastCameraUpdateMs() const { return lastCameraUpdateMs; }
    double getLastTerrainRenderMs() const { return lastTerrainRenderMs; }
    double getLastWMORenderMs() const { return lastWMORenderMs; }
    double getLastM2RenderMs() const { return lastM2RenderMs; }
    audio::MusicManager* getMusicManager() { return musicManager.get(); }
    audio::FootstepManager* getFootstepManager() { return footstepManager.get(); }
    audio::ActivitySoundManager* getActivitySoundManager() { return activitySoundManager.get(); }
    audio::MountSoundManager* getMountSoundManager() { return mountSoundManager.get(); }
    audio::NpcVoiceManager* getNpcVoiceManager() { return npcVoiceManager.get(); }
    audio::AmbientSoundManager* getAmbientSoundManager() { return ambientSoundManager.get(); }
    audio::UiSoundManager* getUiSoundManager() { return uiSoundManager.get(); }
    audio::CombatSoundManager* getCombatSoundManager() { return combatSoundManager.get(); }
    audio::SpellSoundManager* getSpellSoundManager() { return spellSoundManager.get(); }
    audio::MovementSoundManager* getMovementSoundManager() { return movementSoundManager.get(); }
    LightingManager* getLightingManager() { return lightingManager.get(); }

private:
    core::Window* window = nullptr;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<CameraController> cameraController;
    std::unique_ptr<Scene> scene;
    std::unique_ptr<TerrainRenderer> terrainRenderer;
    std::unique_ptr<TerrainManager> terrainManager;
    std::unique_ptr<PerformanceHUD> performanceHUD;
    std::unique_ptr<WaterRenderer> waterRenderer;
    std::unique_ptr<Skybox> skybox;
    std::unique_ptr<Celestial> celestial;
    std::unique_ptr<StarField> starField;
    std::unique_ptr<Clouds> clouds;
    std::unique_ptr<LensFlare> lensFlare;
    std::unique_ptr<Weather> weather;
    std::unique_ptr<LightingManager> lightingManager;
    std::unique_ptr<SkySystem> skySystem;  // Coordinator for sky rendering
    std::unique_ptr<SwimEffects> swimEffects;
    std::unique_ptr<MountDust> mountDust;
    std::unique_ptr<CharacterRenderer> characterRenderer;
    std::unique_ptr<WMORenderer> wmoRenderer;
    std::unique_ptr<M2Renderer> m2Renderer;
    std::unique_ptr<Minimap> minimap;
    std::unique_ptr<QuestMarkerRenderer> questMarkerRenderer;
    std::unique_ptr<audio::MusicManager> musicManager;
    std::unique_ptr<audio::FootstepManager> footstepManager;
    std::unique_ptr<audio::ActivitySoundManager> activitySoundManager;
    std::unique_ptr<audio::MountSoundManager> mountSoundManager;
    std::unique_ptr<audio::NpcVoiceManager> npcVoiceManager;
    std::unique_ptr<audio::AmbientSoundManager> ambientSoundManager;
    std::unique_ptr<audio::UiSoundManager> uiSoundManager;
    std::unique_ptr<audio::CombatSoundManager> combatSoundManager;
    std::unique_ptr<audio::SpellSoundManager> spellSoundManager;
    std::unique_ptr<audio::MovementSoundManager> movementSoundManager;
    std::unique_ptr<game::ZoneManager> zoneManager;
    std::unique_ptr<Shader> underwaterOverlayShader;
    uint32_t underwaterOverlayVAO = 0;
    uint32_t underwaterOverlayVBO = 0;

    // Post-process FBO pipeline (HDR MSAA → resolve → tonemap)
    uint32_t sceneFBO = 0;          // MSAA render target
    uint32_t sceneColorRBO = 0;     // GL_RGBA16F multisampled renderbuffer
    uint32_t sceneDepthRBO = 0;     // GL_DEPTH_COMPONENT24 multisampled renderbuffer
    uint32_t resolveFBO = 0;        // Non-MSAA resolve target
    uint32_t resolveColorTex = 0;   // GL_RGBA16F resolved texture (sampled by post-process)
    uint32_t resolveDepthTex = 0;   // GL_DEPTH_COMPONENT24 resolved texture (for future SSAO)
    uint32_t screenQuadVAO = 0;
    uint32_t screenQuadVBO = 0;
    std::unique_ptr<Shader> postProcessShader;
    int fbWidth = 0, fbHeight = 0;

    void initPostProcess(int w, int h);
    void resizePostProcess(int w, int h);
    void shutdownPostProcess();

    // Shadow mapping
    static constexpr int SHADOW_MAP_SIZE = 2048;
    uint32_t shadowFBO = 0;
    uint32_t shadowDepthTex = 0;
    uint32_t shadowShaderProgram = 0;
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
    glm::vec3 shadowCenter = glm::vec3(0.0f);
    bool shadowCenterInitialized = false;
    bool shadowsEnabled = false;

public:
    void setShadowsEnabled(bool enabled) { shadowsEnabled = enabled; }
    bool areShadowsEnabled() const { return shadowsEnabled; }

private:
    void initShadowMap();
    void renderShadowPass();
    uint32_t compileShadowShader();
    glm::mat4 computeLightSpaceMatrix();

    pipeline::AssetManager* cachedAssetManager = nullptr;
    uint32_t currentZoneId = 0;
    std::string currentZoneName;
    bool inTavern_ = false;
    bool inBlacksmith_ = false;
    float musicSwitchCooldown_ = 0.0f;

    // Third-person character state
    glm::vec3 characterPosition = glm::vec3(0.0f);
    uint32_t characterInstanceId = 0;
    float characterYaw = 0.0f;

    // Character animation state
    enum class CharAnimState { IDLE, WALK, RUN, JUMP_START, JUMP_MID, JUMP_END, SIT_DOWN, SITTING, EMOTE, SWIM_IDLE, SWIM, MELEE_SWING, MOUNT };
    CharAnimState charAnimState = CharAnimState::IDLE;
    void updateCharacterAnimation();
    bool isFootstepAnimationState() const;
    bool shouldTriggerFootstepEvent(uint32_t animationId, float animationTimeMs, float animationDurationMs);
    audio::FootstepSurface resolveFootstepSurface() const;
    uint32_t resolveMeleeAnimId();

    // Emote state
    bool emoteActive = false;
    uint32_t emoteAnimId = 0;
    bool emoteLoop = false;

    // Target facing
    const glm::vec3* targetPosition = nullptr;
    bool inCombat_ = false;

    // Selection circle rendering
    uint32_t selCircleVAO = 0;
    uint32_t selCircleVBO = 0;
    uint32_t selCircleShader = 0;
    int selCircleVertCount = 0;
    void initSelectionCircle();
    void renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection);
    glm::vec3 selCirclePos{0.0f};
    glm::vec3 selCircleColor{1.0f, 0.0f, 0.0f};
    float selCircleRadius = 1.5f;
    bool selCircleVisible = false;

    // Footstep event tracking (animation-driven)
    uint32_t footstepLastAnimationId = 0;
    float footstepLastNormTime = 0.0f;
    bool footstepNormInitialized = false;

    // Footstep surface cache (avoid expensive queries every step)
    mutable audio::FootstepSurface cachedFootstepSurface{};
    mutable glm::vec3 cachedFootstepPosition{0.0f, 0.0f, 0.0f};
    mutable float cachedFootstepUpdateTimer{999.0f};  // Force initial query

    // Mount footstep tracking (separate from player's)
    uint32_t mountFootstepLastAnimId = 0;
    float mountFootstepLastNormTime = 0.0f;
    bool mountFootstepNormInitialized = false;
    bool sfxStateInitialized = false;
    bool sfxPrevGrounded = true;
    bool sfxPrevJumping = false;
    bool sfxPrevFalling = false;
    bool sfxPrevSwimming = false;

    float meleeSwingTimer = 0.0f;
    float meleeSwingCooldown = 0.0f;
    float meleeAnimDurationMs = 0.0f;
    uint32_t meleeAnimId = 0;
    uint32_t equippedWeaponInvType_ = 0;

    // Mount state
    // Mount animation capabilities (discovered at mount time, varies per model)
    struct MountAnimSet {
        uint32_t jumpStart = 0;  // Jump start animation
        uint32_t jumpLoop = 0;   // Jump airborne loop
        uint32_t jumpEnd = 0;    // Jump landing
        uint32_t rearUp = 0;     // Rear-up / special flourish
        uint32_t run = 0;        // Run animation (discovered, don't assume)
        uint32_t stand = 0;      // Stand animation (discovered)
        std::vector<uint32_t> fidgets;  // Idle fidget animations (head turn, tail swish, etc.)
    };

    enum class MountAction { None, Jump, RearUp };

    uint32_t mountInstanceId_ = 0;
    float mountHeightOffset_ = 0.0f;
    float mountPitch_ = 0.0f;  // Up/down tilt (radians)
    float mountRoll_ = 0.0f;   // Left/right banking (radians)
    int mountSeatAttachmentId_ = -1;  // -1 unknown, -2 unavailable
    glm::vec3 smoothedMountSeatPos_ = glm::vec3(0.0f);
    bool mountSeatSmoothingInit_ = false;
    float prevMountYaw_ = 0.0f; // Previous yaw for turn rate calculation (procedural lean)
    float lastDeltaTime_ = 0.0f; // Cached for use in updateCharacterAnimation()
    MountAction mountAction_ = MountAction::None;  // Current mount action (jump/rear-up)
    uint32_t mountActionPhase_ = 0;  // 0=start, 1=loop, 2=end (for jump chaining)
    MountAnimSet mountAnims_;  // Cached animation IDs for current mount
    float mountIdleFidgetTimer_ = 0.0f;  // Timer for random idle fidgets
    float mountIdleSoundTimer_ = 0.0f;   // Timer for ambient idle sounds
    uint32_t mountActiveFidget_ = 0;     // Currently playing fidget animation ID (0 = none)
    bool taxiFlight_ = false;

    bool terrainEnabled = true;
    bool terrainLoaded = false;

    // CPU timing stats (last frame/update).
    double lastUpdateMs = 0.0;
    double lastRenderMs = 0.0;
    double lastCameraUpdateMs = 0.0;
    double lastTerrainRenderMs = 0.0;
    double lastWMORenderMs = 0.0;
    double lastM2RenderMs = 0.0;
};

} // namespace rendering
} // namespace wowee
