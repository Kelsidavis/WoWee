#pragma once

#include "pipeline/m2_loader.hpp"
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
    class AssetManager;
}

namespace rendering {

class Shader;
class Camera;

/**
 * GPU representation of an M2 model
 */
struct M2ModelGPU {
    struct BatchGPU {
        GLuint texture = 0;
        uint32_t indexStart = 0;   // offset in indices (not bytes)
        uint32_t indexCount = 0;
        bool hasAlpha = false;
    };

    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    uint32_t indexCount = 0;
    uint32_t vertexCount = 0;
    std::vector<BatchGPU> batches;

    glm::vec3 boundMin;
    glm::vec3 boundMax;
    float boundRadius = 0.0f;

    std::string name;

    bool isValid() const { return vao != 0 && indexCount > 0; }
};

/**
 * Instance of an M2 model in the world
 */
struct M2Instance {
    uint32_t id = 0;     // Unique instance ID
    uint32_t modelId;
    glm::vec3 position;
    glm::vec3 rotation;  // Euler angles in degrees
    float scale;
    glm::mat4 modelMatrix;
    glm::mat4 invModelMatrix;
    glm::vec3 worldBoundsMin;
    glm::vec3 worldBoundsMax;

    // Animation state
    float animTime = 0.0f;       // Current animation time
    float animSpeed = 1.0f;      // Animation playback speed
    uint32_t animId = 0;         // Current animation sequence

    void updateModelMatrix();
};

/**
 * M2 Model Renderer
 *
 * Handles rendering of M2 models (doodads like trees, rocks, bushes)
 */
class M2Renderer {
public:
    M2Renderer();
    ~M2Renderer();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    /**
     * Load an M2 model to GPU
     * @param model Parsed M2 model data
     * @param modelId Unique ID for this model
     * @return True if successful
     */
    bool loadModel(const pipeline::M2Model& model, uint32_t modelId);

    /**
     * Create an instance of a loaded model
     * @param modelId ID of the loaded model
     * @param position World position
     * @param rotation Rotation in degrees (x, y, z)
     * @param scale Scale factor (1.0 = normal)
     * @return Instance ID
     */
    uint32_t createInstance(uint32_t modelId, const glm::vec3& position,
                            const glm::vec3& rotation = glm::vec3(0.0f),
                            float scale = 1.0f);

    /**
     * Create an instance with a pre-computed model matrix
     * Used for WMO doodads where the full transform is computed externally
     */
    uint32_t createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                       const glm::vec3& position);

    /**
     * Update animation state for all instances
     * @param deltaTime Time since last frame
     */
    void update(float deltaTime);

    /**
     * Render all visible instances
     */
    void render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);

    /**
     * Remove a specific instance by ID
     * @param instanceId Instance ID returned by createInstance()
     */
    void removeInstance(uint32_t instanceId);

    /**
     * Clear all models and instances
     */
    void clear();

    /**
     * Remove models that have no instances referencing them
     * Call periodically to free GPU memory
     */
    void cleanupUnusedModels();

    /**
     * Check collision with M2 objects and adjust position
     * @param from Starting position
     * @param to Desired position
     * @param adjustedPos Output adjusted position
     * @param playerRadius Collision radius of player
     * @return true if collision occurred
     */
    bool checkCollision(const glm::vec3& from, const glm::vec3& to,
                        glm::vec3& adjustedPos, float playerRadius = 0.5f) const;

    /**
     * Approximate top surface height for standing/jumping on doodads.
     * @param glX World X
     * @param glY World Y
     * @param glZ Query/reference Z (used to ignore unreachable tops)
     */
    std::optional<float> getFloorHeight(float glX, float glY, float glZ) const;

    /**
     * Raycast against M2 bounding boxes for camera collision
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

    // Stats
    uint32_t getModelCount() const { return static_cast<uint32_t>(models.size()); }
    uint32_t getInstanceCount() const { return static_cast<uint32_t>(instances.size()); }
    uint32_t getTotalTriangleCount() const;
    uint32_t getDrawCallCount() const { return lastDrawCallCount; }

private:
    pipeline::AssetManager* assetManager = nullptr;
    std::unique_ptr<Shader> shader;

    std::unordered_map<uint32_t, M2ModelGPU> models;
    std::vector<M2Instance> instances;

    uint32_t nextInstanceId = 1;
    uint32_t lastDrawCallCount = 0;

    GLuint loadTexture(const std::string& path);
    std::unordered_map<std::string, GLuint> textureCache;
    GLuint whiteTexture = 0;

    // Lighting uniforms
    glm::vec3 lightDir = glm::vec3(0.5f, 0.5f, 1.0f);
    glm::vec3 ambientColor = glm::vec3(0.4f, 0.4f, 0.45f);

    // Optional query-space culling for collision/raycast hot paths.
    bool collisionFocusEnabled = false;
    glm::vec3 collisionFocusPos = glm::vec3(0.0f);
    float collisionFocusRadius = 0.0f;
    float collisionFocusRadiusSq = 0.0f;

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

    static constexpr float SPATIAL_CELL_SIZE = 64.0f;
    std::unordered_map<GridCell, std::vector<uint32_t>, GridCellHash> spatialGrid;
    std::unordered_map<uint32_t, size_t> instanceIndexById;
    mutable std::vector<size_t> candidateScratch;
    mutable std::unordered_set<uint32_t> candidateIdScratch;

    // Collision query profiling (per frame).
    mutable double queryTimeMs = 0.0;
    mutable uint32_t queryCallCount = 0;
};

} // namespace rendering
} // namespace wowee
