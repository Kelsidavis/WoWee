#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include "rendering/vk_frame_data.hpp"
#include "rendering/sky_system.hpp"

namespace wowee {
namespace core { class Window; }
namespace rendering { class VkContext; }
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
class SwimEffects;
class MountDust;
class LevelUpEffect;
class ChargeEffect;
class CharacterRenderer;
class WMORenderer;
class M2Renderer;
class Minimap;
class WorldMap;
class QuestMarkerRenderer;
class CharacterPreview;
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
    Skybox* getSkybox() const { return skySystem ? skySystem->getSkybox() : nullptr; }
    Celestial* getCelestial() const { return skySystem ? skySystem->getCelestial() : nullptr; }
    StarField* getStarField() const { return skySystem ? skySystem->getStarField() : nullptr; }
    Clouds* getClouds() const { return skySystem ? skySystem->getClouds() : nullptr; }
    LensFlare* getLensFlare() const { return skySystem ? skySystem->getLensFlare() : nullptr; }
    Weather* getWeather() const { return weather.get(); }
    CharacterRenderer* getCharacterRenderer() const { return characterRenderer.get(); }
    WMORenderer* getWMORenderer() const { return wmoRenderer.get(); }
    M2Renderer* getM2Renderer() const { return m2Renderer.get(); }
    Minimap* getMinimap() const { return minimap.get(); }
    WorldMap* getWorldMap() const { return worldMap.get(); }
    QuestMarkerRenderer* getQuestMarkerRenderer() const { return questMarkerRenderer.get(); }
    SkySystem* getSkySystem() const { return skySystem.get(); }
    const std::string& getCurrentZoneName() const { return currentZoneName; }
    VkContext* getVkContext() const { return vkCtx; }
    VkDescriptorSetLayout getPerFrameSetLayout() const { return perFrameSetLayout; }
    VkRenderPass getShadowRenderPass() const { return shadowRenderPass; }

    // Third-person character follow
    void setCharacterFollow(uint32_t instanceId);
    glm::vec3& getCharacterPosition() { return characterPosition; }
    uint32_t getCharacterInstanceId() const { return characterInstanceId; }
    float getCharacterYaw() const { return characterYaw; }
    void setCharacterYaw(float yawDeg) { characterYaw = yawDeg; }

    // Emote support
    void playEmote(const std::string& emoteName);
    void triggerLevelUpEffect(const glm::vec3& position);
    void cancelEmote();
    bool isEmoteActive() const { return emoteActive; }
    static std::string getEmoteText(const std::string& emoteName, const std::string* targetName = nullptr);
    static uint32_t getEmoteDbcId(const std::string& emoteName);
    static std::string getEmoteTextByDbcId(uint32_t dbcId, const std::string& senderName, const std::string* targetName = nullptr);
    static uint32_t getEmoteAnimByDbcId(uint32_t dbcId);

    // Targeting support
    void setTargetPosition(const glm::vec3* pos);
    void setInCombat(bool combat) { inCombat_ = combat; }
    bool isMoving() const;
    void triggerMeleeSwing();
    void setEquippedWeaponType(uint32_t inventoryType) { equippedWeaponInvType_ = inventoryType; meleeAnimId = 0; }
    void setCharging(bool charging) { charging_ = charging; }
    bool isCharging() const { return charging_; }
    void startChargeEffect(const glm::vec3& position, const glm::vec3& direction);
    void emitChargeEffect(const glm::vec3& position, const glm::vec3& direction);
    void stopChargeEffect();

    // Mount rendering
    void setMounted(uint32_t mountInstId, uint32_t mountDisplayId, float heightOffset, const std::string& modelPath = "");
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
    game::ZoneManager* getZoneManager() { return zoneManager.get(); }
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
    void runDeferredWorldInitStep(float deltaTime);

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
    std::unique_ptr<LevelUpEffect> levelUpEffect;
    std::unique_ptr<ChargeEffect> chargeEffect;
    std::unique_ptr<CharacterRenderer> characterRenderer;
    std::unique_ptr<WMORenderer> wmoRenderer;
    std::unique_ptr<M2Renderer> m2Renderer;
    std::unique_ptr<Minimap> minimap;
    std::unique_ptr<WorldMap> worldMap;
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
    // Shadow mapping (Vulkan)
    static constexpr uint32_t SHADOW_MAP_SIZE = 4096;
    VkImage shadowDepthImage = VK_NULL_HANDLE;
    VmaAllocation shadowDepthAlloc = VK_NULL_HANDLE;
    VkImageView shadowDepthView = VK_NULL_HANDLE;
    VkSampler shadowSampler = VK_NULL_HANDLE;
    VkRenderPass shadowRenderPass = VK_NULL_HANDLE;
    VkFramebuffer shadowFramebuffer = VK_NULL_HANDLE;
    VkImageLayout shadowDepthLayout_ = VK_IMAGE_LAYOUT_UNDEFINED;
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
    glm::vec3 shadowCenter = glm::vec3(0.0f);
    bool shadowCenterInitialized = false;
    bool shadowsEnabled = true;
    uint32_t shadowFrameCounter_ = 0;


public:
    // Character preview registration (for off-screen composite pass)
    void registerPreview(CharacterPreview* preview);
    void unregisterPreview(CharacterPreview* preview);

    void setShadowsEnabled(bool enabled) { shadowsEnabled = enabled; }
    bool areShadowsEnabled() const { return shadowsEnabled; }
    void setMsaaSamples(VkSampleCountFlagBits samples);

private:
    void applyMsaaChange();
    VkSampleCountFlagBits pendingMsaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    bool msaaChangePending_ = false;
    void renderShadowPass();
    glm::mat4 computeLightSpaceMatrix();

    pipeline::AssetManager* cachedAssetManager = nullptr;
    uint32_t currentZoneId = 0;
    std::string currentZoneName;
    bool inTavern_ = false;
    bool inBlacksmith_ = false;
    float musicSwitchCooldown_ = 0.0f;
    bool deferredWorldInitEnabled_ = true;
    bool deferredWorldInitPending_ = false;
    uint8_t deferredWorldInitStage_ = 0;
    float deferredWorldInitCooldown_ = 0.0f;

    // Third-person character state
    glm::vec3 characterPosition = glm::vec3(0.0f);
    uint32_t characterInstanceId = 0;
    float characterYaw = 0.0f;

    // Character animation state
    enum class CharAnimState { IDLE, WALK, RUN, JUMP_START, JUMP_MID, JUMP_END, SIT_DOWN, SITTING, EMOTE, SWIM_IDLE, SWIM, MELEE_SWING, MOUNT, CHARGE, COMBAT_IDLE };
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

    // Selection circle rendering (Vulkan)
    VkPipeline selCirclePipeline = VK_NULL_HANDLE;
    VkPipelineLayout selCirclePipelineLayout = VK_NULL_HANDLE;
    ::VkBuffer selCircleVertBuf = VK_NULL_HANDLE;
    VmaAllocation selCircleVertAlloc = VK_NULL_HANDLE;
    ::VkBuffer selCircleIdxBuf = VK_NULL_HANDLE;
    VmaAllocation selCircleIdxAlloc = VK_NULL_HANDLE;
    int selCircleVertCount = 0;
    void initSelectionCircle();
    void renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection);
    glm::vec3 selCirclePos{0.0f};
    glm::vec3 selCircleColor{1.0f, 0.0f, 0.0f};
    float selCircleRadius = 1.5f;
    bool selCircleVisible = false;

    // Fullscreen color overlay (underwater tint)
    VkPipeline overlayPipeline = VK_NULL_HANDLE;
    VkPipelineLayout overlayPipelineLayout = VK_NULL_HANDLE;
    void initOverlayPipeline();
    void renderOverlay(const glm::vec4& color);

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

    bool charging_ = false;
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
    bool taxiAnimsLogged_ = false;

    // Vulkan frame state
    VkContext* vkCtx = nullptr;
    VkCommandBuffer currentCmd = VK_NULL_HANDLE;
    uint32_t currentImageIndex = 0;

    // Per-frame UBO + descriptors (set 0)
    static constexpr uint32_t MAX_FRAMES = 2;
    VkDescriptorSetLayout perFrameSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneDescriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet perFrameDescSets[MAX_FRAMES] = {};
    VkBuffer perFrameUBOs[MAX_FRAMES] = {};
    VmaAllocation perFrameUBOAllocs[MAX_FRAMES] = {};
    void* perFrameUBOMapped[MAX_FRAMES] = {};
    GPUPerFrameData currentFrameData{};
    float globalTime = 0.0f;

    // Per-frame reflection UBO (mirrors camera for planar reflections)
    VkBuffer reflPerFrameUBO = VK_NULL_HANDLE;
    VmaAllocation reflPerFrameUBOAlloc = VK_NULL_HANDLE;
    void* reflPerFrameUBOMapped = nullptr;
    VkDescriptorSet reflPerFrameDescSet = VK_NULL_HANDLE;

    bool createPerFrameResources();
    void destroyPerFrameResources();
    void updatePerFrameUBO();
    void setupWater1xPass();
    void renderReflectionPass();

    // Active character previews for off-screen rendering
    std::vector<CharacterPreview*> activePreviews_;

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
