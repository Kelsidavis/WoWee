#pragma once

#include "pipeline/blp_loader.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <optional>
#include <future>
#include <algorithm>

namespace wowee {
namespace pipeline {
    struct WMOModel;
    struct WMOGroup;
    class AssetManager;
}

namespace rendering {

class Camera;
class Frustum;
class M2Renderer;
class VkContext;
class VkTexture;

/**
 * WMO (World Model Object) Renderer (Vulkan)
 *
 * Renders buildings, dungeons, and large structures from WMO files.
 * Features:
 * - Multi-material rendering
 * - Batched rendering per group
 * - Frustum culling
 * - Portal visibility (future)
 */
class WMORenderer {
public:
    WMORenderer();
    ~WMORenderer();

    /**
     * Initialize renderer (Vulkan)
     * @param ctx Vulkan context
     * @param perFrameLayout Descriptor set layout for set 0 (per-frame UBO)
     * @param assetManager Asset manager for loading textures (optional)
     */
    [[nodiscard]] bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                    pipeline::AssetManager* assetManager = nullptr);

    /**
     * Cleanup GPU resources
     */
    void shutdown();

    /**
     * Set M2 renderer for hierarchical transform updates (doodads follow parent WMO)
     */
    void setM2Renderer(M2Renderer* renderer) { m2Renderer_ = renderer; }

    /**
     * Load WMO model and create GPU resources
     * @param model WMO model with geometry data
     * @param id Unique identifier for this WMO instance
     * @return True if successful
     */
    bool loadModel(const pipeline::WMOModel& model, uint32_t id);

    enum class ModelLoadResult { Complete, InProgress, Failed };

    /// Upload a model a few groups at a time. InProgress means call again with
    /// the same arguments; a single large model can otherwise take over a
    /// hundred milliseconds, which lands as a visible hitch when terrain
    /// streaming finalises a tile.
    ModelLoadResult loadModelIncremental(const pipeline::WMOModel& model, uint32_t id, float budgetMs);

    /**
     * Check if a WMO model is currently resident in the renderer
     * @param id WMO model identifier
     */
    bool isModelLoaded(uint32_t id) const;
    /// Whether this instance's model has finished uploading its groups, and so
    /// whether a failed floor query means "nothing under you" rather than "not
    /// loaded yet". The two need telling apart: a rider held in place waiting
    /// for collision that is already there waits forever.
    bool instanceHasCollisionGeometry(uint32_t instanceId) const;

    /**
     * Check if a WMO instance is still live in the renderer. Owners that cache
     * instance IDs use this to detect an instance dropped underneath them
     * (e.g. by a renderer-wide clear) instead of addressing a dead handle.
     * @param instanceId Instance identifier returned by createInstance
     */
    bool hasInstance(uint32_t instanceId) const;

    /**
     * Unload WMO model and free GPU resources
     * @param id WMO model identifier
     */
    void unloadModel(uint32_t id);

    /**
     * Create a WMO instance in the world
     * @param modelId WMO model to instantiate
     * @param position World position
     * @param rotation Rotation (euler angles in radians)
     * @param scale Uniform scale
     * @return Instance ID
     */
    uint32_t createInstance(uint32_t modelId, const glm::vec3& position,
                           const glm::vec3& rotation = glm::vec3(0.0f),
                           float scale = 1.0f);

    /**
     * Update the world position of an existing instance (e.g., for transports)
     * @param instanceId Instance to update
     * @param position New world position
     */
    void setInstancePosition(uint32_t instanceId, const glm::vec3& position);

    /**
     * Update the full transform of an existing instance (for moving transports)
     * @param instanceId Instance to update
     * @param transform World transform matrix
     */
    void setInstanceTransform(uint32_t instanceId, const glm::mat4& transform);

    /**
     * Add doodad (child M2) to WMO instance
     * @param instanceId WMO instance to add doodad to
     * @param m2InstanceId M2 instance ID of the doodad
     * @param localTransform Local transform relative to WMO origin
     */
    void addDoodadToInstance(uint32_t instanceId, uint32_t m2InstanceId, const glm::mat4& localTransform);

    // Forward declare DoodadTemplate for public API
    struct DoodadTemplate {
        std::string m2Path;
        glm::mat4 localTransform;
    };

    /**
     * Get doodad templates for a WMO model
     * @param modelId WMO model ID
     * @return Vector of doodad templates (empty if no doodads or model not found)
     */
    /// Set an animation on every child doodad of an instance. Returns how many
    /// doodads were addressed, so a caller streaming them in over several frames
    /// can tell when the set has grown and re-apply. A doodad lacking the
    /// sequence is left as it is.
    size_t setInstanceDoodadAnimation(uint32_t instanceId, uint32_t animationId, bool loop);
    const std::vector<DoodadTemplate>* getDoodadTemplates(uint32_t modelId) const;

    /**
     * Remove WMO instance
     * @param instanceId Instance to remove
     */
    void removeInstance(uint32_t instanceId);
    /**
     * Remove multiple WMO instances with a single spatial-index rebuild.
     */
    void removeInstances(const std::vector<uint32_t>& instanceIds);

    /**
     * Remove all instances
     */
    void clearInstances();

    /**
     * Clear all instances, loaded models, and texture cache (for map transitions)
     */
    void clearAll();

    /**
     * Render all WMO instances (Vulkan)
     * @param cmd Command buffer to record into
     * @param perFrameSet Per-frame descriptor set (set 0)
     * @param camera Camera for frustum culling
     */
    /** Pre-update mutable state (frame ID, material UBOs) on main thread before parallel render. */
    void prepareRender();
    /// viewerPos is the character; portal culling seeds from it as well as from
    /// the camera, because at a doorway the two stand in different groups and
    /// neither alone is reliably the right place to start.
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera,
                const glm::vec3* viewerPos = nullptr);

    /**
     * Initialize shadow pipeline (Phase 7)
     */
    [[nodiscard]] bool initializeShadow(VkRenderPass shadowRenderPass);

    /**
     * Render depth-only for shadow casting
     */
    void renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix,
                      const glm::vec3& shadowCenter = glm::vec3(0), float shadowRadius = 1e9f);

    /**
     * Get number of loaded models
     */
    void recreatePipelines();
    bool isInitialized() const { return initialized_; }
    uint32_t getModelCount() const { return loadedModels.size(); }

    /**
     * Get number of active instances
     */
    uint32_t getInstanceCount() const { return instances.size(); }
    size_t getLoadedModelCount() const { return loadedModels.size(); }

    /**
     * Remove models that have no instances referencing them
     * Call periodically to free GPU memory
     */
    void cleanupUnusedModels();

    /**
     * Get total triangle count (all instances)
     */
    uint32_t getTotalTriangleCount() const;

    /**
     * Get total draw call count (last frame)
     */
    uint32_t getDrawCallCount() const { return lastDrawCalls; }

    /**
     * Normal mapping / Parallax Occlusion Mapping settings
     */
    void setNormalMappingEnabled(bool enabled) { normalMappingEnabled_ = enabled; materialSettingsDirty_ = true; }
    void setNormalMapStrength(float s) { normalMapStrength_ = s; materialSettingsDirty_ = true; }
    void setPOMEnabled(bool enabled) { pomEnabled_ = enabled; materialSettingsDirty_ = true; }
    void setPOMQuality(int q) { pomQuality_ = q; materialSettingsDirty_ = true; }
    void setWMOOnlyMap(bool v) { wmoOnlyMap_ = v; materialSettingsDirty_ = true; }
    bool isNormalMappingEnabled() const { return normalMappingEnabled_; }
    float getNormalMapStrength() const { return normalMapStrength_; }
    bool isPOMEnabled() const { return pomEnabled_; }
    int getPOMQuality() const { return pomQuality_; }

    /**
     * Enable/disable wireframe rendering
     */
    void setWireframeMode(bool enabled) { wireframeMode = enabled; }

    /**
     * Enable/disable frustum culling
     */
    void setFrustumCulling(bool enabled) { frustumCulling = enabled; }

    /**
     * Enable/disable portal-based visibility culling
     */
    void setPortalCulling(bool enabled) { portalCulling = enabled; }
    bool isPortalCullingEnabled() const { return portalCulling; }

    /**
     * Enable/disable distance-based group culling
     */
    void setDistanceCulling(bool enabled, float maxDistance = 500.0f) {
        distanceCulling = enabled;
        maxGroupDistance = maxDistance;
        maxGroupDistanceSq = maxDistance * maxDistance;
    }
    bool isDistanceCullingEnabled() const { return distanceCulling; }
    float getMaxGroupDistance() const { return maxGroupDistance; }
    void setViewDistance(float distance) { viewDistance_ = std::clamp(distance, 400.0f, 2400.0f); }

    /**
     * Get number of groups culled by portals last frame
     */
    uint32_t getPortalCulledGroups() const { return lastPortalCulledGroups; }

    /**
     * Get number of groups culled by distance last frame
     */
    uint32_t getDistanceCulledGroups() const { return lastDistanceCulledGroups; }

    /**
     * Enable/disable GPU occlusion query culling (stubbed in Vulkan)
     */
    void setOcclusionCulling(bool /*enabled*/) { /* stubbed */ }
    bool isOcclusionCullingEnabled() const { return false; }

    /**
     * Get number of groups culled by occlusion queries last frame
     */
    uint32_t getOcclusionCulledGroups() const { return 0; }

    // Lighting/fog/shadow are now in the per-frame UBO; these are no-ops for API compat
    void setFog(const glm::vec3& /*color*/, float /*start*/, float /*end*/) {}
    void setLighting(const float /*lightDir*/[3], const float /*lightColor*/[3],
                     const float /*ambientColor*/[3]) {}
    void setShadowMap(uint32_t /*depthTex*/, const glm::mat4& /*lightSpace*/) {}
    void clearShadowMap() {}

    /**
     * Get floor height at a GL position via ray-triangle intersection.
     * @param outNormalZ If not null, receives the Z component of the floor surface normal
     *                   (1.0 = flat, 0.0 = vertical). Useful for slope walkability checks.
     */
    std::optional<float> getFloorHeight(float glX, float glY, float glZ, float* outNormalZ = nullptr) const;

    /** Query floor collision from one moving WMO instance. */
    std::optional<float> getInstanceFloorHeight(uint32_t instanceId,
                                                float glX, float glY, float glZ,
                                                float* outNormalZ = nullptr) const;

    /** Dump diagnostic info about WMO groups overlapping a position */
    void debugDumpGroupsAtPosition(float glX, float glY, float glZ) const;

    /**
     * Check wall collision and adjust position
     * @param from Starting position
     * @param to Desired position
     * @param adjustedPos Output adjusted position (pushed away from walls)
     * @param insideWMO If true, use tighter collision for indoor precision
     * @return true if collision occurred
     */
    bool checkWallCollision(const glm::vec3& from, const glm::vec3& to, glm::vec3& adjustedPos, bool insideWMO = false) const;

    /**
     * Check if a position is inside any WMO
     * @param outModelId If not null, receives the model ID of the WMO
     * @return true if inside a WMO
     */
    bool isInsideWMO(float glX, float glY, float glZ, uint32_t* outModelId = nullptr) const;

    /**
     * Check if a position is inside an interior WMO group (flag 0x2000).
     * Used to dim M2 lighting for doodads placed indoors.
     */
    bool isInsideInteriorWMO(float glX, float glY, float glZ) const;

    /** Gather local orange point lights derived from visible lava materials. */
    uint32_t gatherLavaLights(const glm::vec3& cameraPos,
                              glm::vec4* outPosRadius,
                              glm::vec4* outColorIntensity,
                              uint32_t maxLights) const;

    /**
     * Raycast against WMO bounding boxes for camera collision
     * @param origin Ray origin (e.g., character head position)
     * @param direction Ray direction (normalized)
     * @param maxDistance Maximum ray distance to check
     * @return Distance to first intersection, or maxDistance if no hit
     */
    float raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const;

    /**
     * Limit expensive collision/raycast queries to objects near a focus point.
     */
    void setCollisionFocus(const glm::vec3& worldPos, float radius);
    void clearCollisionFocus();

    void resetQueryStats();
    double getQueryTimeMs() const { return queryTimeMs; }
    uint32_t getQueryCallCount() const { return queryCallCount; }

    /**
     * Update the tracked active WMO group based on player position.
     * Called at low frequency (every ~10 frames or on significant movement).
     */
    void updateActiveGroup(float glX, float glY, float glZ);

    // Floor cache persistence (zone-specific files)
    void setMapName(const std::string& name) { mapName_ = name; }
    const std::string& getMapName() const { return mapName_; }
    bool saveFloorCache() const;  // Saves to cache/wmo_floor_<mapName>.bin
    bool loadFloorCache();        // Loads from cache/wmo_floor_<mapName>.bin
    size_t getFloorCacheSize() const { return precomputedFloorGrid.size(); }

    // Pre-compute floor cache for all loaded WMO instances
    void precomputeFloorCache();

    // Pre-decoded BLP cache: set before calling loadModel() to skip main-thread BLP decode
    void setPredecodedBLPCache(std::unordered_map<std::string, pipeline::BLPImage>* cache) { predecodedBLPCache_ = cache; }

    // Normal/height pixels may be generated by terrain workers alongside the
    // decoded diffuse textures. loadTexture() then only performs the bounded GPU
    // upload while building the WMO model on the main thread.
    void setPredecodedNormalMapCache(
        std::unordered_map<std::string, pipeline::BLPImage>* cache,
        std::unordered_map<std::string, float>* variances) {
        predecodedNormalMapCache_ = cache;
        predecodedNormalMapVariances_ = variances;
    }
    static pipeline::BLPImage generateNormalHeightMapPixels(
        const uint8_t* pixels, uint32_t width, uint32_t height, float& outVariance);

    // Defer normal/height map generation during streaming to avoid CPU stalls
    void setDeferNormalMaps(bool defer) { deferNormalMaps_ = defer; }

private:
    // WMO material UBO — matches WMOMaterial in wmo.frag.glsl
    struct WMOMaterialUBO {
        int32_t hasTexture;        // 0
        int32_t alphaTest;         // 4
        int32_t unlit;             // 8
        int32_t isInterior;        // 12
        float specularIntensity;   // 16
        int32_t isWindow;          // 20
        int32_t enableNormalMap;   // 24
        int32_t enablePOM;         // 28
        float pomScale;            // 32 (height scale)
        int32_t pomMaxSamples;     // 36 (max ray-march steps)
        float heightMapVariance;   // 40 (low variance = skip POM)
        float normalMapStrength;   // 44 (0=flat, 1=full, 2=exaggerated)
        int32_t isLava;            // 48 (1=lava/magma UV scroll)
        float wmoAmbientR;         // 52 (interior ambient color R)
        float wmoAmbientG;         // 56 (interior ambient color G)
        float wmoAmbientB;         // 60 (interior ambient color B)
        int32_t emissive;           // 64 (0 none, 1 lamp glass, 2 firelit)
        int32_t padding0;           // 68
        int32_t padding1;           // 72
        int32_t padding2;           // 76
    };  // 80 bytes total

    /**
     * WMO group GPU resources
     */
    struct GroupResources {
        ::VkBuffer vertexBuffer = VK_NULL_HANDLE;
        VmaAllocation vertexAlloc = VK_NULL_HANDLE;
        ::VkBuffer indexBuffer = VK_NULL_HANDLE;
        VmaAllocation indexAlloc = VK_NULL_HANDLE;
        uint32_t indexCount = 0;
        uint32_t vertexCount = 0;
        glm::vec3 boundingBoxMin;
        glm::vec3 boundingBoxMax;

        uint32_t groupFlags = 0;
        bool allUntextured = false;  // True if ALL batches use fallback white texture (collision/placeholder group)
        bool isLOD = false;          // Distance-only group (skip when camera is close)

        // Material batches (start index, count, material ID)
        struct Batch {
            uint32_t startIndex;   // First index in EBO
            uint32_t indexCount;   // Number of indices to draw
            uint8_t materialId;    // Material/texture reference
        };
        std::vector<Batch> batches;

        // Pre-merged batches for efficient rendering (computed at load time)
        struct MergedBatch {
            VkTexture* texture = nullptr;   // from cache, NOT owned
            VkTexture* normalHeightMap = nullptr;  // generated from diffuse, NOT owned
            float heightMapVariance = 0.0f; // variance of height map (low = flat texture)
            VkDescriptorSet materialSet = VK_NULL_HANDLE;  // set 1
            ::VkBuffer materialUBO = VK_NULL_HANDLE;
            VmaAllocation materialUBOAlloc = VK_NULL_HANDLE;
            bool hasTexture = false;
            bool alphaTest = false;
            bool unlit = false;
            bool isTransparent = false;     // blendMode >= 2
            bool isWindow = false;          // F_SIDN or F_WINDOW material
            bool isLava = false;            // lava/magma texture (UV scroll)
            // 0 = not luminous, 1 = authored lamp glass (bright), 2 = firelit
            // from behind with a flicker (clock faces). See wmo.frag's emissive
            // branch for how each level is shaded.
            uint8_t emissiveLevel = 0;
            // For multi-draw: store index ranges
            struct DrawRange { uint32_t firstIndex; uint32_t indexCount; };
            std::vector<DrawRange> draws;
        };
        std::vector<MergedBatch> mergedBatches;
        // Local-space center/radius for each authored lava draw range.
        std::vector<glm::vec4> lavaLights;

        // Collision geometry (positions only, for floor raycasting)
        std::vector<glm::vec3> collisionVertices;
        std::vector<uint16_t> collisionIndices;

        // 2D spatial grid for fast triangle lookup (built at load time).
        // Bins triangles by their XY bounding box into grid cells.
        static constexpr float COLLISION_CELL_SIZE = 4.0f;
        int gridCellsX = 0;
        int gridCellsY = 0;
        glm::vec2 gridOrigin;  // XY of bounding box min
        // cellTriangles[cellY * gridCellsX + cellX] = list of triangle start indices
        std::vector<std::vector<uint32_t>> cellTriangles;

        // Pre-classified triangle lists per cell (built at load time)
        std::vector<std::vector<uint32_t>> cellFloorTriangles;  // abs(normal.z) >= 0.35
        std::vector<std::vector<uint32_t>> cellWallTriangles;   // abs(normal.z) < 0.35

        // Pre-computed per-triangle Z bounds for fast vertical reject
        struct TriBounds { float minZ; float maxZ; };
        std::vector<TriBounds> triBounds;  // indexed by triStart/3

        // Pre-computed per-triangle normals (unit length, indexed by triStart/3)
        std::vector<glm::vec3> triNormals;

        // Per-collision-triangle MOPY flags (indexed by collision tri index, i.e. triStart/3)
        std::vector<uint8_t> triMopyFlags;

        // Scratch bitset for deduplicating triangle queries (sized to numTriangles)
        mutable std::vector<uint8_t> triVisited;

        // Build the spatial grid from collision geometry
        void buildCollisionGrid();

        // Get triangle indices for a local-space XY point
        const std::vector<uint32_t>* getTrianglesAtLocal(float localX, float localY) const;

        // Get triangle indices for a local-space XY range (for wall collision)
        void getTrianglesInRange(float minX, float minY, float maxX, float maxY,
                                 std::vector<uint32_t>& out) const;

        // Get pre-classified floor/wall triangles in range
        void getFloorTrianglesInRange(float minX, float minY, float maxX, float maxY,
                                      std::vector<uint32_t>& out) const;
        void getWallTrianglesInRange(float minX, float minY, float maxX, float maxY,
                                     std::vector<uint32_t>& out) const;
    };

    /**
     * Portal data for visibility culling
     */
    struct PortalData {
        uint16_t startVertex;
        uint16_t vertexCount;
        glm::vec3 normal;
        float distance;
    };

    struct PortalRef {
        uint16_t portalIndex;
        uint16_t groupIndex;
        int16_t side;
    };

    /**
     * Loaded WMO model data
     */
    struct ModelData {
        uint32_t id;
        std::vector<GroupResources> groups;
        glm::vec3 boundingBoxMin;
        glm::vec3 boundingBoxMax;
        glm::vec3 wmoAmbientColor{0.5f, 0.5f, 0.5f};  // From MOHD, used for interior lighting
        bool isLowPlatform = false;

        // Doodad templates (M2 models placed in WMO, stored for instancing)
        // Uses the public DoodadTemplate struct defined above
        std::vector<DoodadTemplate> doodadTemplates;

        // Texture handles for this model (indexed by texture path order)
        std::vector<VkTexture*> textures;  // non-owning, from cache
        std::vector<std::string> textureNames;  // lowercase texture paths (parallel to textures)

        // Material texture indices (materialId -> texture index)
        std::vector<uint32_t> materialTextureIndices;

        // Material blend modes (materialId -> blendMode; 1 = alpha-test cutout)
        std::vector<uint32_t> materialBlendModes;

        // Material flags (materialId -> flags; 0x01 = unlit)
        std::vector<uint32_t> materialFlags;

        // Portal visibility data
        std::vector<PortalData> portals;
        std::vector<glm::vec3> portalVertices;
        std::vector<PortalRef> portalRefs;
        // For each group: which portal refs belong to it (start index, count)
        std::vector<std::pair<uint16_t, uint16_t>> groupPortalRefs;

        // Set once the textures and materials below have been populated, so a
        // resumed load skips straight to the groups it has left.
        bool setupDone = false;
        // Next group to upload. A large model's groups are spread across several
        // calls under a time budget rather than uploaded in one stall.
        // Next texture to upload. Uploading them all at once cost 40ms on a
        // transport — the images are large, and the expense is the GPU upload
        // rather than the decode, which the worker already did.
        size_t nextTextureIndex = 0;
        size_t nextGroupIndex = 0;
        uint32_t loadedGroups = 0;

        uint32_t getTotalTriangles() const {
            uint32_t total = 0;
            for (const auto& group : groups) {
                total += group.indexCount / 3;
            }
            return total;
        }
    };

    /**
     * WMO instance in the world
     */
    struct WMOInstance {
        uint32_t id;
        uint32_t modelId;
        glm::vec3 position;
        glm::vec3 rotation;  // Euler angles (radians)
        float scale;
        glm::mat4 modelMatrix;
        glm::mat4 invModelMatrix;  // Cached inverse for collision
        glm::vec3 worldBoundsMin;
        glm::vec3 worldBoundsMax;
        std::vector<std::pair<glm::vec3, glm::vec3>> worldGroupBounds;

        // Doodad tracking: M2 instances that are children of this WMO
        struct DoodadInfo {
            uint32_t m2InstanceId;       // ID of the M2 instance
            glm::mat4 localTransform;    // Local transform relative to WMO origin
        };
        std::vector<DoodadInfo> doodads;

        void updateModelMatrix();
    };

    /**
     * Create GPU resources for a WMO group
     */
    bool createGroupResources(const pipeline::WMOGroup& group, GroupResources& resources, uint32_t groupFlags = 0);

    /**
     * Check if group is visible in frustum
     */
    bool isGroupVisible(const GroupResources& group, const glm::mat4& modelMatrix,
                       const Camera& camera) const;

    /**
     * Find which group index contains a position (model space)
     * @return Group index or -1 if outside all groups
     */
    int findContainingGroup(const ModelData& model, const glm::vec3& localPos) const;

    /**
     * Get visible groups via portal traversal
     */
    /// Seeds from both the camera's group and the character's: at a doorway the
    /// two are in different rooms, and walking from only one while judging every
    /// door against the camera's frustum empties the interior.
    void getVisibleGroupsViaPortals(const ModelData& model,
                                     const glm::vec3& cameraLocalPos,
                                     const glm::vec3& viewerLocalPos,
                                     const Frustum& frustum,
                                     const glm::mat4& modelMatrix,
                                     std::unordered_set<uint32_t>& outVisibleGroups) const;

    /**
     * Test if a portal polygon is visible from a position through a frustum
     */
    bool isPortalVisible(const ModelData& model, uint16_t portalIndex,
                         const glm::vec3& cameraLocalPos,
                         const Frustum& frustum,
                         const glm::mat4& modelMatrix) const;

    /**
     * Load a texture from path
     */
    VkTexture* loadTexture(const std::string& path);
    std::unordered_map<std::string, pipeline::BLPImage>* predecodedBLPCache_ = nullptr;
    std::unordered_map<std::string, pipeline::BLPImage>* predecodedNormalMapCache_ = nullptr;
    std::unordered_map<std::string, float>* predecodedNormalMapVariances_ = nullptr;

    /**
     * Generate normal+height map from diffuse RGBA8 pixels
     * @param pixels RGBA8 pixel data
     * @param width Texture width
     * @param height Texture height
     * @param outVariance Receives height map variance (for POM threshold)
     * @return Generated VkTexture (RGBA8: RGB=normal, A=height)
     */
    std::unique_ptr<VkTexture> generateNormalHeightMap(const uint8_t* pixels, uint32_t width, uint32_t height, float& outVariance);

    /**
     * Allocate a material descriptor set from the pool
     */
    VkDescriptorSet allocateMaterialSet();

    /**
     * Destroy GPU resources for a single group.
     * When defer=true, destruction is scheduled via deferAfterFrameFence
     * so in-flight command buffers are not invalidated.
     */
    void destroyGroupGPU(GroupResources& group, bool defer = false);

    struct GridCell {
        int x;
        int y;
        int z;
        bool operator==(const GridCell& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct GridCellHash {
        size_t operator()(const GridCell& c) const {
            size_t h1 = std::hash<int>()(c.x);
            size_t h2 = std::hash<int>()(c.y);
            size_t h3 = std::hash<int>()(c.z);
            return h1 ^ (h2 * 0x9e3779b9u) ^ (h3 * 0x85ebca6bu);
        }
    };

    GridCell toCell(const glm::vec3& p) const;
    void rebuildSpatialIndex();
    void gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax, std::vector<size_t>& outIndices) const;

    // Vulkan context
    VkContext* vkCtx_ = nullptr;

    // Asset manager for loading textures
    pipeline::AssetManager* assetManager = nullptr;

    // M2 renderer for hierarchical transforms (doodads following WMO parent)
    M2Renderer* m2Renderer_ = nullptr;

    // Current map name for zone-specific floor cache
    std::string mapName_;

    // Vulkan pipelines
    VkPipeline opaquePipeline_ = VK_NULL_HANDLE;
    VkPipeline transparentPipeline_ = VK_NULL_HANDLE;
    VkPipeline glassPipeline_ = VK_NULL_HANDLE;      // alpha blend + depth write (windows)
    VkPipeline wireframePipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;

    // Shadow rendering (Phase 7)
    VkPipeline shadowPipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout shadowPipelineLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout shadowParamsLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool shadowParamsPool_ = VK_NULL_HANDLE;
    VkDescriptorSet shadowParamsSet_ = VK_NULL_HANDLE;
    ::VkBuffer shadowParamsUBO_ = VK_NULL_HANDLE;
    VmaAllocation shadowParamsAlloc_ = VK_NULL_HANDLE;

    // Descriptor set layouts
    VkDescriptorSetLayout materialSetLayout_ = VK_NULL_HANDLE;

    // Descriptor pool for material sets
    VkDescriptorPool materialDescPool_ = VK_NULL_HANDLE;
    static constexpr uint32_t MAX_MATERIAL_SETS = 32768;

    // Texture cache (path -> VkTexture)
    struct TextureCacheEntry {
        std::unique_ptr<VkTexture> texture;
        std::unique_ptr<VkTexture> normalHeightMap;  // generated normal+height from diffuse
        float heightMapVariance = 0.0f;  // variance of generated height map
        size_t approxBytes = 0;
        uint64_t lastUse = 0;
    };
    std::unordered_map<std::string, TextureCacheEntry> textureCache;
    size_t textureCacheBytes_ = 0;
    uint64_t textureCacheCounter_ = 0;
    size_t textureCacheBudgetBytes_ = 8192ull * 1024 * 1024;  // 8 GB default, overridden at init
    std::unordered_set<std::string> failedTextureCache_;
    std::unordered_map<std::string, uint64_t> failedTextureRetryAt_;
    std::unordered_set<std::string> loggedTextureLoadFails_;
    uint64_t textureLookupSerial_ = 0;
    uint32_t textureBudgetRejectWarnings_ = 0;

    // Default white texture
    std::unique_ptr<VkTexture> whiteTexture_;

    // Flat normal placeholder (128,128,255,128) = up-pointing normal, mid-height
    std::unique_ptr<VkTexture> flatNormalTexture_;

    // Loaded models (modelId -> ModelData)
    std::unordered_map<uint32_t, ModelData> loadedModels;
    // Models part-way through an incremental load; moved into loadedModels once
    // every group is uploaded, so a half-built model is never rendered.
    std::unordered_map<uint32_t, ModelData> loadingModels_;
    size_t modelCacheLimit_ = 4000;
    uint32_t modelLimitRejectWarnings_ = 0;

    // Active instances
    std::vector<WMOInstance> instances;
    uint32_t nextInstanceId = 1;

    bool initialized_ = false;

    // Normal mapping / POM settings
    bool normalMappingEnabled_ = true;   // on by default
    bool deferNormalMaps_ = false;       // skip normal map gen during streaming
    float normalMapStrength_ = 0.8f;     // 0.0 = flat, 1.0 = full, 2.0 = exaggerated
    bool pomEnabled_ = true;             // on by default
    int pomQuality_ = 1;                 // 0=Low(16), 1=Medium(32), 2=High(64)
    bool materialSettingsDirty_ = false; // rebuild UBOs when settings change
    bool wmoOnlyMap_ = false;            // true for dungeon/instance WMO-only maps

    // Rendering state
    bool wireframeMode = false;
    bool frustumCulling = true;  // dead: every instance is included, the group cull does the work
    bool portalCulling = true;   // AABB transform bug fixed; conservative frustum test (no plane-side check) is visually safe
    bool distanceCulling = false;  // Disabled - causes ground to disappear

    /// Master switch, and it is off: no WMO group is dropped for any reason.
    ///
    /// Buildings kept disappearing from angles that had no business hiding
    /// them. Distance culling was turned off years ago for the same complaint
    /// ("causes ground to disappear") and it did not settle the matter, because
    /// the distance test below runs whether that flag is set or not — the flag
    /// only chooses which of two distances to use, so everything past
    /// viewDistance_ vanished regardless.
    ///
    /// The set of instances is already bounded by which tiles are loaded, so
    /// what this costs is the groups of loaded buildings that are behind the
    /// camera or too far to matter, not an unbounded scene.
    ///
    /// WOWEE_WMO_CULL=1 puts portal and distance culling back, for measuring
    /// what it costs or for chasing this again.
    bool cullingEnabled_ = false;
    float maxGroupDistance = 500.0f;
    float maxGroupDistanceSq = 250000.0f;  // maxGroupDistance^2
    float viewDistance_ = 1200.0f;
    uint32_t lastDrawCalls = 0;
    mutable uint32_t lastPortalCulledGroups = 0;
    mutable uint32_t lastDistanceCulledGroups = 0;

    // Optional query-space culling for collision/raycast hot paths.
    bool collisionFocusEnabled = false;
    glm::vec3 collisionFocusPos = glm::vec3(0.0f);
    float collisionFocusRadius = 0.0f;
    float collisionFocusRadiusSq = 0.0f;

    // Uniform grid for fast local collision queries.
    static constexpr float SPATIAL_CELL_SIZE = 64.0f;
    std::unordered_map<GridCell, std::vector<uint32_t>, GridCellHash> spatialGrid;
    std::unordered_map<uint32_t, size_t> instanceIndexById;
    // Collision scratch buffers are thread_local (see wmo_renderer.cpp) for thread-safety.

    // Parallel visibility culling
    uint32_t numCullThreads_ = 1;

    struct InstanceDrawList {
        size_t instanceIndex;
        const ModelData* model = nullptr;     // cached pointer; saves a hashmap find per instance per frame
        std::vector<uint32_t> visibleGroups;  // group indices that passed culling
        uint32_t portalCulled = 0;
        uint32_t distanceCulled = 0;
    };
    std::vector<std::future<void>> cullFutures_;
    std::vector<size_t> visibleInstances_;      // reused per frame
    std::vector<InstanceDrawList> drawLists_;    // reused per frame
    std::unordered_set<uint32_t> portalVisibleGroupSet_; // reused per frame (portal culling scratch)

    // Collision query profiling — atomic because getFloorHeight is dispatched
    // on async threads from camera_controller while the main thread reads these.
    mutable std::atomic<double> queryTimeMs{0.0};
    mutable std::atomic<uint32_t> queryCallCount{0};

    // Floor height cache - persistent precomputed grid
    static constexpr float FLOOR_GRID_CELL_SIZE = 2.0f;  // 2 unit grid cells
    mutable std::unordered_map<uint64_t, float> precomputedFloorGrid;  // key -> floor height
    mutable bool floorGridDirty = true;  // Rebuild when instances change
    mutable uint32_t currentFrameId = 0;

    uint64_t floorGridKey(float x, float y) const {
        int32_t ix = static_cast<int32_t>(std::floor(x / FLOOR_GRID_CELL_SIZE));
        int32_t iy = static_cast<int32_t>(std::floor(y / FLOOR_GRID_CELL_SIZE));
        return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(iy));
    }

    // Compute floor height for a single cell (expensive, done at load time)
    std::optional<float> computeFloorHeightSlow(float x, float y, float refZ) const;

    // Active WMO group tracking — reduces per-query group iteration
    struct ActiveGroupInfo {
        uint32_t instanceIdx = UINT32_MAX;
        uint32_t modelId = 0;
        int32_t groupIdx = -1;
        std::vector<uint32_t> neighborGroups;  // portal-connected groups
        bool isValid() const { return instanceIdx != UINT32_MAX && groupIdx >= 0; }
        void invalidate() { instanceIdx = UINT32_MAX; groupIdx = -1; neighborGroups.clear(); }
    };
    mutable ActiveGroupInfo activeGroup_;

    // Per-frame floor height dedup cache (same XY queried 3-5x per frame)
    struct FrameFloorCache {
        static constexpr size_t CAPACITY = 16;
        struct Entry { uint64_t key; float resultZ; float normalZ; uint32_t frameId; };
        Entry entries[CAPACITY] = {};

        uint64_t makeKey(float x, float y) const {
            // 0.5-unit quantized grid
            int32_t ix = static_cast<int32_t>(std::floor(x * 2.0f));
            int32_t iy = static_cast<int32_t>(std::floor(y * 2.0f));
            return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
                   static_cast<uint64_t>(static_cast<uint32_t>(iy));
        }

        std::optional<float> get(float x, float y, uint32_t frame, float* outNormalZ = nullptr) const {
            uint64_t k = makeKey(x, y);
            size_t slot = k % CAPACITY;
            const auto& e = entries[slot];
            if (e.frameId == frame && e.key == k) {
                if (outNormalZ) *outNormalZ = e.normalZ;
                return e.resultZ;
            }
            return std::nullopt;
        }

        void put(float x, float y, float result, float normalZ, uint32_t frame) {
            uint64_t k = makeKey(x, y);
            size_t slot = k % CAPACITY;
            entries[slot] = { k, result, normalZ, frame };
        }
    };
    mutable FrameFloorCache frameFloorCache_;
};

} // namespace rendering
} // namespace wowee
