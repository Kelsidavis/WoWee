#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <optional>

namespace wowee {
namespace pipeline {
    struct WMOModel;
    struct WMOGroup;
    class AssetManager;
}

namespace rendering {

class Camera;
class Shader;
class Frustum;

/**
 * WMO (World Model Object) Renderer
 *
 * Renders buildings, dungeons, and large structures from WMO files.
 * Features:
 * - Multi-material rendering
 * - Batched rendering per group
 * - Frustum culling
 * - Portal visibility (future)
 * - Dynamic lighting support (future)
 */
class WMORenderer {
public:
    WMORenderer();
    ~WMORenderer();

    /**
     * Initialize renderer and create shaders
     * @param assetManager Asset manager for loading textures (optional)
     */
    bool initialize(pipeline::AssetManager* assetManager = nullptr);

    /**
     * Cleanup GPU resources
     */
    void shutdown();

    /**
     * Load WMO model and create GPU resources
     * @param model WMO model with geometry data
     * @param id Unique identifier for this WMO instance
     * @return True if successful
     */
    bool loadModel(const pipeline::WMOModel& model, uint32_t id);

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
     * Remove WMO instance
     * @param instanceId Instance to remove
     */
    void removeInstance(uint32_t instanceId);

    /**
     * Remove all instances
     */
    void clearInstances();

    /**
     * Render all WMO instances
     * @param camera Camera for view/projection matrices
     * @param view View matrix
     * @param projection Projection matrix
     */
    void render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);

    /**
     * Get number of loaded models
     */
    uint32_t getModelCount() const { return loadedModels.size(); }

    /**
     * Get number of active instances
     */
    uint32_t getInstanceCount() const { return instances.size(); }

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

    /**
     * Get number of groups culled by portals last frame
     */
    uint32_t getPortalCulledGroups() const { return lastPortalCulledGroups; }

    /**
     * Get number of groups culled by distance last frame
     */
    uint32_t getDistanceCulledGroups() const { return lastDistanceCulledGroups; }

    /**
     * Enable/disable GPU occlusion query culling
     */
    void setOcclusionCulling(bool enabled) { occlusionCulling = enabled; }
    bool isOcclusionCullingEnabled() const { return occlusionCulling; }

    /**
     * Get number of groups culled by occlusion queries last frame
     */
    uint32_t getOcclusionCulledGroups() const { return lastOcclusionCulledGroups; }

    void setFog(const glm::vec3& color, float start, float end) {
        fogColor = color; fogStart = start; fogEnd = end;
    }

    void setShadowMap(GLuint depthTex, const glm::mat4& lightSpace) {
        shadowDepthTex = depthTex; lightSpaceMatrix = lightSpace; shadowEnabled = true;
    }
    void clearShadowMap() { shadowEnabled = false; }

    /**
     * Render depth-only for shadow casting (reuses VAOs)
     */
    void renderShadow(const glm::mat4& lightView, const glm::mat4& lightProj, Shader& shadowShader);

    /**
     * Get floor height at a GL position via ray-triangle intersection.
     * @param outNormalZ If not null, receives the Z component of the floor surface normal
     *                   (1.0 = flat, 0.0 = vertical). Useful for slope walkability checks.
     */
    std::optional<float> getFloorHeight(float glX, float glY, float glZ, float* outNormalZ = nullptr) const;

    /**
     * Check wall collision and adjust position
     * @param from Starting position
     * @param to Desired position
     * @param adjustedPos Output adjusted position (pushed away from walls)
     * @return true if collision occurred
     */
    bool checkWallCollision(const glm::vec3& from, const glm::vec3& to, glm::vec3& adjustedPos) const;

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

private:
    /**
     * WMO group GPU resources
     */
    struct GroupResources {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ebo = 0;
        uint32_t indexCount = 0;
        uint32_t vertexCount = 0;
        glm::vec3 boundingBoxMin;
        glm::vec3 boundingBoxMax;

        uint32_t groupFlags = 0;

        // Material batches (start index, count, material ID)
        struct Batch {
            uint32_t startIndex;   // First index in EBO
            uint32_t indexCount;   // Number of indices to draw
            uint8_t materialId;    // Material/texture reference
        };
        std::vector<Batch> batches;

        // Pre-merged batches for efficient rendering (computed at load time)
        struct MergedBatch {
            GLuint texId;
            bool hasTexture;
            bool alphaTest;
            bool unlit = false;
            uint32_t blendMode = 0;
            std::vector<GLsizei> counts;
            std::vector<const void*> offsets;
        };
        std::vector<MergedBatch> mergedBatches;

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
        std::vector<std::vector<uint32_t>> cellFloorTriangles;  // abs(normal.z) >= 0.45
        std::vector<std::vector<uint32_t>> cellWallTriangles;   // abs(normal.z) < 0.55

        // Pre-computed per-triangle Z bounds for fast vertical reject
        struct TriBounds { float minZ; float maxZ; };
        std::vector<TriBounds> triBounds;  // indexed by triStart/3

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
        bool isLowPlatform = false;

        // Texture handles for this model (indexed by texture path order)
        std::vector<GLuint> textures;

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

        void updateModelMatrix();
    };

    /**
     * Create GPU resources for a WMO group
     */
    bool createGroupResources(const pipeline::WMOGroup& group, GroupResources& resources, uint32_t groupFlags = 0);

    /**
     * Render a single group
     */
    void renderGroup(const GroupResources& group, const ModelData& model,
                    const glm::mat4& modelMatrix,
                    const glm::mat4& view, const glm::mat4& projection);

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
     * @param model The WMO model data
     * @param cameraLocalPos Camera position in model space
     * @param frustum Frustum for portal visibility testing
     * @param modelMatrix Transform for world-space frustum test
     * @param outVisibleGroups Output set of visible group indices
     */
    void getVisibleGroupsViaPortals(const ModelData& model,
                                     const glm::vec3& cameraLocalPos,
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
    GLuint loadTexture(const std::string& path);

    /**
     * Initialize occlusion query resources (bbox VAO, shader)
     */
    void initOcclusionResources();

    /**
     * Run occlusion query pre-pass for an instance
     */
    void runOcclusionQueries(const WMOInstance& instance, const ModelData& model,
                              const glm::mat4& view, const glm::mat4& projection);

    /**
     * Check if a group passed occlusion test (uses previous frame results)
     */
    bool isGroupOccluded(uint32_t instanceId, uint32_t groupIndex) const;

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

    // Shader
    std::unique_ptr<Shader> shader;

    // Asset manager for loading textures
    pipeline::AssetManager* assetManager = nullptr;

    // Current map name for zone-specific floor cache
    std::string mapName_;

    // Texture cache (path -> texture ID)
    std::unordered_map<std::string, GLuint> textureCache;

    // Default white texture
    GLuint whiteTexture = 0;

    // Loaded models (modelId -> ModelData)
    std::unordered_map<uint32_t, ModelData> loadedModels;

    // Active instances
    std::vector<WMOInstance> instances;
    uint32_t nextInstanceId = 1;

    // Rendering state
    bool wireframeMode = false;
    bool frustumCulling = true;
    bool portalCulling = false;  // Disabled by default - needs debugging
    bool distanceCulling = false;  // Disabled - causes ground to disappear
    bool occlusionCulling = false;  // GPU occlusion queries - disabled, adds overhead
    float maxGroupDistance = 500.0f;
    float maxGroupDistanceSq = 250000.0f;  // maxGroupDistance^2
    uint32_t lastDrawCalls = 0;
    mutable uint32_t lastPortalCulledGroups = 0;
    mutable uint32_t lastDistanceCulledGroups = 0;
    mutable uint32_t lastOcclusionCulledGroups = 0;

    // Occlusion query resources
    GLuint bboxVao = 0;
    GLuint bboxVbo = 0;
    std::unique_ptr<Shader> occlusionShader;
    // Query objects per (instance, group) - reused each frame
    // Key: (instanceId << 16) | groupIndex
    mutable std::unordered_map<uint32_t, GLuint> occlusionQueries;
    // Results from previous frame (1 frame latency to avoid GPU stalls)
    mutable std::unordered_map<uint32_t, bool> occlusionResults;

    // Fog parameters
    glm::vec3 fogColor = glm::vec3(0.5f, 0.6f, 0.7f);
    float fogStart = 400.0f;
    float fogEnd = 1200.0f;

    // Shadow mapping
    GLuint shadowDepthTex = 0;
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
    bool shadowEnabled = false;

    // Optional query-space culling for collision/raycast hot paths.
    bool collisionFocusEnabled = false;
    glm::vec3 collisionFocusPos = glm::vec3(0.0f);
    float collisionFocusRadius = 0.0f;
    float collisionFocusRadiusSq = 0.0f;

    // Uniform grid for fast local collision queries.
    static constexpr float SPATIAL_CELL_SIZE = 64.0f;
    std::unordered_map<GridCell, std::vector<uint32_t>, GridCellHash> spatialGrid;
    std::unordered_map<uint32_t, size_t> instanceIndexById;
    mutable std::vector<size_t> candidateScratch;
    mutable std::vector<uint32_t> wallTriScratch;  // Scratch for wall collision grid queries
    mutable std::unordered_set<uint32_t> candidateIdScratch;

    // Parallel visibility culling
    uint32_t numCullThreads_ = 1;

    struct InstanceDrawList {
        size_t instanceIndex;
        std::vector<uint32_t> visibleGroups;  // group indices that passed culling
        uint32_t portalCulled = 0;
        uint32_t distanceCulled = 0;
        uint32_t occlusionCulled = 0;
    };

    // Collision query profiling (per frame).
    mutable double queryTimeMs = 0.0;
    mutable uint32_t queryCallCount = 0;

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
