#pragma once

#include "rendering/vk_frame_data.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/camera.hpp"
#include "editor_water.hpp"
#include "editor_markers.hpp"
#include "transform_gizmo.hpp"
#include "object_placer.hpp"
#include "npc_spawner.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering { class VkContext; class VkTexture; class VolumetricFog; }

namespace editor {

class EditorViewport {
public:
    EditorViewport();
    ~EditorViewport();

    bool initialize(rendering::VkContext* ctx, pipeline::AssetManager* am, rendering::Camera* cam);
    void shutdown();

    bool loadTerrain(const pipeline::TerrainMesh& mesh,
                     const std::vector<std::string>& texturePaths,
                     int tileX, int tileY);
    void clearTerrain();

    void updateWater(const pipeline::ADTTerrain& terrain, int tileX, int tileY);
    void updateMarkers(const std::vector<PlacedObject>& objects);
    void updateNpcMarkers(const std::vector<CreatureSpawn>& npcs);
    void placeM2(const std::string& path, const glm::vec3& pos, const glm::vec3& rot, float scale);
    void placeWMO(const std::string& path, const glm::vec3& pos, const glm::vec3& rot);
    void clearObjects();
    void rebuildObjects(const std::vector<PlacedObject>& objects,
                        const std::vector<CreatureSpawn>& npcs = {});

    void update(float deltaTime);
    void render(VkCommandBuffer cmd);

    // Ghost preview for placement
    void setGhostPreview(const std::string& path, const glm::vec3& pos,
                         const glm::vec3& rotDeg, float scale);
    void clearGhostPreview();

    TransformGizmo& getGizmo() { return gizmo_; }
    void setBrushIndicator(const glm::vec3& center, float radius, bool active);
    void setPathPreview(const glm::vec3& start, const glm::vec3& end, float width, bool visible);
    /** Show a multi-segment patrol path as a ribbon. Empty `points` clears it. */
    void setPatrolPath(const std::vector<glm::vec3>& points, float width = 1.5f);
    void clearPatrolPath() { setPatrolPath({}); }

    void setWireframe(bool enabled);
    void setShowNpcMarkers(bool show) { showNpcMarkers_ = show; }
    bool getShowNpcMarkers() const { return showNpcMarkers_; }

    // Nameplates: when true, only show the floating name label on the
    // currently-selected NPC; otherwise show on all of them. Default true
    // to keep the viewport quiet - was previously always-on, which got
    // noisy fast on populated zones.
    void setNpcNameplatesSelectedOnly(bool sel) { npcNameplatesSelectedOnly_ = sel; }
    bool getNpcNameplatesSelectedOnly() const { return npcNameplatesSelectedOnly_; }

    bool isWireframe() const { return wireframe_; }

    void setClearColor(float r, float g, float b) { clearR_=r; clearG_=g; clearB_=b; }
    void getClearColor(float& r, float& g, float& b) const { r=clearR_; g=clearG_; b=clearB_; }
    void setLightDir(const glm::vec3& d) { lightDir_ = d; }
    glm::vec3 getLightDir() const { return lightDir_; }

    void setTimeOfDay(float t);
    float getTimeOfDay() const { return timeOfDay_; }
    glm::vec3& getLightColor() { return lightColor_; }
    glm::vec3& getAmbientColor() { return ambientColor_; }
    glm::vec3& getFogColor() { return fogColor_; }
    float& getFogNear() { return fogNear_; }
    float& getFogFar() { return fogFar_; }

    rendering::TerrainRenderer* getTerrainRenderer() { return terrainRenderer_.get(); }
    rendering::M2Renderer* getM2Renderer() { return m2Renderer_.get(); }

    /** Set the active map name so WOM/WOB lookups can probe per-zone roots first. */
    void setActiveMapName(const std::string& name) { activeMapName_ = name; }

private:
    bool createPerFrameResources();
    void destroyPerFrameResources();
    void updatePerFrameUBO();

    rendering::VkContext* vkCtx_ = nullptr;
    pipeline::AssetManager* assetManager_ = nullptr;
    rendering::Camera* camera_ = nullptr;

    std::unique_ptr<rendering::TerrainRenderer> terrainRenderer_;
    std::unique_ptr<rendering::M2Renderer> m2Renderer_;
    std::unique_ptr<rendering::WMORenderer> wmoRenderer_;
    EditorWater waterRenderer_;
    EditorMarkers markerRenderer_;
    TransformGizmo gizmo_;

    static constexpr uint32_t MAX_FRAMES = 2;
    VkDescriptorSetLayout perFrameSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool sceneDescPool_ = VK_NULL_HANDLE;
    VkDescriptorSet perFrameDescSets_[MAX_FRAMES] = {};
    VkBuffer perFrameUBOs_[MAX_FRAMES] = {};
    VmaAllocation perFrameUBOAllocs_[MAX_FRAMES] = {};
    void* perFrameUBOMapped_[MAX_FRAMES] = {};

    std::unique_ptr<rendering::VkTexture> dummyShadowTexture_;
    VkSampler shadowSampler_ = VK_NULL_HANDLE;
    /// Only for its neutral volume and sampler: the world shaders read a fog
    /// volume at set 0 binding 2, and the editor builds none.
    std::unique_ptr<rendering::VolumetricFog> fogVolume_;

    bool wireframe_ = false;
    float clearR_ = 0.15f, clearG_ = 0.15f, clearB_ = 0.2f;
    glm::vec3 lightDir_ = glm::normalize(glm::vec3(0.5f, -1.0f, 0.3f));
    glm::vec3 lightColor_ = glm::vec3(1.0f, 0.95f, 0.85f);
    glm::vec3 ambientColor_ = glm::vec3(0.3f, 0.3f, 0.35f);
    glm::vec3 fogColor_ = glm::vec3(0.6f, 0.7f, 0.8f);
    float fogNear_ = 5000.0f, fogFar_ = 10000.0f;
    float timeOfDay_ = 12.0f;

    // Persistent path -> renderer model ID maps. Keeping these across rebuilds
    // lets the renderer skip re-uploading models that are still in its cache.
    std::unordered_map<std::string, uint32_t> persistentM2ModelIds_;
    std::unordered_map<std::string, uint32_t> persistentWMOModelIds_;
    uint32_t nextPersistentModelId_ = 1;

    // Active map name used to build per-zone WOM/WOB prefixes so per-zone
    // overrides win over global custom_zones/ assets.
    std::string activeMapName_;

    // Ghost preview state
    std::string ghostModelPath_;
    uint32_t ghostModelId_ = 0;
    uint32_t ghostInstanceId_ = 0;
    bool ghostActive_ = false;

    // Brush indicator
    VkBuffer brushVB_ = VK_NULL_HANDLE;
    VmaAllocation brushVBAlloc_ = VK_NULL_HANDLE;
    uint32_t brushVertCount_ = 0;
    bool brushVisible_ = false;

    // NPC position markers
    bool showNpcMarkers_ = true;
    bool npcNameplatesSelectedOnly_ = true;
    VkBuffer npcMarkerVB_ = VK_NULL_HANDLE;
    VmaAllocation npcMarkerVBAlloc_ = VK_NULL_HANDLE;
    uint32_t npcMarkerVertCount_ = 0;

    // Path preview line
    VkBuffer pathVB_ = VK_NULL_HANDLE;
    VmaAllocation pathVBAlloc_ = VK_NULL_HANDLE;
    uint32_t pathVertCount_ = 0;
    bool pathVisible_ = false;

    // Patrol path ribbon (selected NPC)
    VkBuffer patrolVB_ = VK_NULL_HANDLE;
    VmaAllocation patrolVBAlloc_ = VK_NULL_HANDLE;
    uint32_t patrolVertCount_ = 0;
};

} // namespace editor
} // namespace wowee
