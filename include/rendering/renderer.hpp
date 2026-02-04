#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include <glm/glm.hpp>

namespace wowee {
namespace core { class Window; }
namespace game { class World; class ZoneManager; }
namespace audio { class MusicManager; class FootstepManager; class ActivitySoundManager; enum class FootstepSurface : uint8_t; }
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
class SwimEffects;
class CharacterRenderer;
class WMORenderer;
class M2Renderer;
class Minimap;
class Shader;

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize(core::Window* window);
    void shutdown();

    void beginFrame();
    void endFrame();

    void renderWorld(game::World* world);

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
    const std::string& getCurrentZoneName() const { return currentZoneName; }

    // Third-person character follow
    void setCharacterFollow(uint32_t instanceId);
    glm::vec3& getCharacterPosition() { return characterPosition; }
    uint32_t getCharacterInstanceId() const { return characterInstanceId; }
    float getCharacterYaw() const { return characterYaw; }

    // Emote support
    void playEmote(const std::string& emoteName);
    void cancelEmote();
    bool isEmoteActive() const { return emoteActive; }
    static std::string getEmoteText(const std::string& emoteName);

    // Targeting support
    void setTargetPosition(const glm::vec3* pos);
    bool isMoving() const;

    // CPU timing stats (milliseconds, last frame).
    double getLastUpdateMs() const { return lastUpdateMs; }
    double getLastRenderMs() const { return lastRenderMs; }
    double getLastCameraUpdateMs() const { return lastCameraUpdateMs; }
    double getLastTerrainRenderMs() const { return lastTerrainRenderMs; }
    double getLastWMORenderMs() const { return lastWMORenderMs; }
    double getLastM2RenderMs() const { return lastM2RenderMs; }

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
    std::unique_ptr<SwimEffects> swimEffects;
    std::unique_ptr<CharacterRenderer> characterRenderer;
    std::unique_ptr<WMORenderer> wmoRenderer;
    std::unique_ptr<M2Renderer> m2Renderer;
    std::unique_ptr<Minimap> minimap;
    std::unique_ptr<audio::MusicManager> musicManager;
    std::unique_ptr<audio::FootstepManager> footstepManager;
    std::unique_ptr<audio::ActivitySoundManager> activitySoundManager;
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

    pipeline::AssetManager* cachedAssetManager = nullptr;
    uint32_t currentZoneId = 0;
    std::string currentZoneName;

    // Third-person character state
    glm::vec3 characterPosition = glm::vec3(0.0f);
    uint32_t characterInstanceId = 0;
    float characterYaw = 0.0f;

    // Character animation state
    enum class CharAnimState { IDLE, WALK, RUN, JUMP_START, JUMP_MID, JUMP_END, SIT_DOWN, SITTING, EMOTE, SWIM_IDLE, SWIM };
    CharAnimState charAnimState = CharAnimState::IDLE;
    void updateCharacterAnimation();
    bool isFootstepAnimationState() const;
    bool shouldTriggerFootstepEvent(uint32_t animationId, float animationTimeMs, float animationDurationMs);
    audio::FootstepSurface resolveFootstepSurface() const;

    // Emote state
    bool emoteActive = false;
    uint32_t emoteAnimId = 0;
    bool emoteLoop = false;

    // Target facing
    const glm::vec3* targetPosition = nullptr;

    // Footstep event tracking (animation-driven)
    uint32_t footstepLastAnimationId = 0;
    float footstepLastNormTime = 0.0f;
    bool footstepNormInitialized = false;
    bool sfxStateInitialized = false;
    bool sfxPrevGrounded = true;
    bool sfxPrevJumping = false;
    bool sfxPrevFalling = false;
    bool sfxPrevSwimming = false;

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
