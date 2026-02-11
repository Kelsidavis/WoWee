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
#include <random>
#include <future>

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
        uint16_t textureAnimIndex = 0xFFFF; // 0xFFFF = no texture animation
        uint16_t blendMode = 0;   // 0=Opaque, 1=AlphaKey, 2=Alpha, 3=Add, etc.
        uint16_t materialFlags = 0; // M2 material flags (0x01=Unlit, 0x04=TwoSided, 0x10=NoDepthWrite)
        uint16_t submeshLevel = 0; // LOD level: 0=base, 1=LOD1, 2=LOD2, 3=LOD3
        glm::vec3 center = glm::vec3(0.0f); // Center of batch geometry (model space)
        float glowSize = 1.0f;              // Approx radius of batch geometry
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
    bool collisionSteppedFountain = false;
    bool collisionSteppedLowPlatform = false;
    bool collisionPlanter = false;
    bool collisionBridge = false;
    bool collisionSmallSolidProp = false;
    bool collisionNarrowVerticalProp = false;
    bool collisionTreeTrunk = false;
    bool collisionNoBlock = false;
    bool collisionStatue = false;
    bool isSmallFoliage = false;  // Small foliage (bushes, grass, plants) - skip during taxi
    bool isInvisibleTrap = false; // Invisible trap objects (don't render, no collision)

    // Collision mesh with spatial grid (from M2 bounding geometry)
    struct CollisionMesh {
        std::vector<glm::vec3> vertices;
        std::vector<uint16_t> indices;
        uint32_t triCount = 0;

        struct TriBounds { float minZ, maxZ; };
        std::vector<TriBounds> triBounds;

        static constexpr float CELL_SIZE = 4.0f;
        glm::vec2 gridOrigin{0.0f};
        int gridCellsX = 0, gridCellsY = 0;
        std::vector<std::vector<uint32_t>> cellFloorTris;
        std::vector<std::vector<uint32_t>> cellWallTris;

        void build();
        void getFloorTrisInRange(float minX, float minY, float maxX, float maxY,
                                 std::vector<uint32_t>& out) const;
        void getWallTrisInRange(float minX, float minY, float maxX, float maxY,
                                std::vector<uint32_t>& out) const;
        bool valid() const { return triCount > 0; }
    };
    CollisionMesh collision;

    std::string name;

    // Skeletal animation data (kept from M2Model for bone computation)
    std::vector<pipeline::M2Bone> bones;
    std::vector<pipeline::M2Sequence> sequences;
    std::vector<uint32_t> globalSequenceDurations;  // Loop durations for global sequence tracks
    bool hasAnimation = false;  // True if any bone has keyframes
    bool isSmoke = false;       // True for smoke models (UV scroll animation)
    bool disableAnimation = false; // Keep foliage/tree doodads visually stable
    bool hasTextureAnimation = false; // True if any batch has UV animation

    // Particle emitter data (kept from M2Model)
    std::vector<pipeline::M2ParticleEmitter> particleEmitters;
    std::vector<GLuint> particleTextures;  // Resolved GL textures per emitter

    // Texture transform data for UV animation
    std::vector<pipeline::M2TextureTransform> textureTransforms;
    std::vector<uint16_t> textureTransformLookup;
    std::vector<int> idleVariationIndices;  // Sequence indices for idle variations (animId 0)

    bool isValid() const { return vao != 0 && indexCount > 0; }
};

/**
 * A single M2 particle emitted from a particle emitter
 */
struct M2Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    float life;        // current age in seconds
    float maxLife;     // total lifespan
    int emitterIndex;  // which emitter spawned this
    float tileIndex = 0.0f; // texture atlas tile index
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
    float animTime = 0.0f;       // Current animation time (ms)
    float animSpeed = 1.0f;      // Animation playback speed
    int currentSequenceIndex = 0;// Index into sequences array
    float animDuration = 0.0f;   // Duration of current animation (ms)
    std::vector<glm::mat4> boneMatrices;

    // Idle variation state
    int idleSequenceIndex = 0;   // Default idle sequence index
    float variationTimer = 0.0f; // Time until next variation attempt (ms)
    bool playingVariation = false;// Currently playing a one-shot variation

    // Particle emitter state
    std::vector<float> emitterAccumulators;  // fractional particle counter per emitter
    std::vector<M2Particle> particles;

    // Frame-skip optimization (update distant animations less frequently)
    uint8_t frameSkipCounter = 0;

    void updateModelMatrix();
};

/**
 * A single smoke particle emitted from a chimney or similar M2 model
 */
struct SmokeParticle {
    glm::vec3 position;
    glm::vec3 velocity;
    float life = 0.0f;
    float maxLife = 3.0f;
    float size = 1.0f;
    float isSpark = 0.0f;  // 0 = smoke, 1 = ember/spark
    uint32_t instanceId = 0;
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
     * Check if a model is already loaded
     * @param modelId ID to check
     * @return True if model is loaded
     */
    bool hasModel(uint32_t modelId) const;

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
     * @param cameraPos Camera world position (for frustum-culling bones)
     * @param viewProjection Combined view*projection matrix
     */
    void update(float deltaTime, const glm::vec3& cameraPos, const glm::mat4& viewProjection);

    /**
     * Render all visible instances
     */
    void render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);

    /**
     * Render depth-only pass for shadow casting
     */
    void renderShadow(GLuint shadowShaderProgram);

    /**
     * Render smoke particles (call after render())
     */
    void renderSmokeParticles(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);

    /**
     * Render M2 particle emitter particles (call after renderSmokeParticles())
     */
    void renderM2Particles(const glm::mat4& view, const glm::mat4& proj);

    /**
     * Update the world position of an existing instance (e.g., for transports)
     * @param instanceId Instance ID returned by createInstance()
     * @param position New world position
     */
    void setInstancePosition(uint32_t instanceId, const glm::vec3& position);

    /**
     * Update the full transform of an existing instance (e.g., for WMO doodads following parent WMO)
     * @param instanceId Instance ID returned by createInstance()
     * @param transform New world transform matrix
     */
    void setInstanceTransform(uint32_t instanceId, const glm::mat4& transform);

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
    std::optional<float> getFloorHeight(float glX, float glY, float glZ, float* outNormalZ = nullptr) const;

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

    void setFog(const glm::vec3& color, float start, float end) {
        fogColor = color; fogStart = start; fogEnd = end;
    }

    void setLighting(const float lightDirIn[3], const float lightColorIn[3],
                     const float ambientColorIn[3]) {
        lightDir = glm::vec3(lightDirIn[0], lightDirIn[1], lightDirIn[2]);
        lightColor = glm::vec3(lightColorIn[0], lightColorIn[1], lightColorIn[2]);
        ambientColor = glm::vec3(ambientColorIn[0], ambientColorIn[1], ambientColorIn[2]);
    }

    void setShadowMap(GLuint depthTex, const glm::mat4& lightSpace) {
        shadowDepthTex = depthTex; lightSpaceMatrix = lightSpace; shadowEnabled = true;
    }
    void clearShadowMap() { shadowEnabled = false; }

    void setInsideInterior(bool inside) { insideInterior = inside; }
    void setOnTaxi(bool onTaxi) { onTaxi_ = onTaxi; }

private:
    bool insideInterior = false;
    bool onTaxi_ = false;
    pipeline::AssetManager* assetManager = nullptr;
    std::unique_ptr<Shader> shader;

    std::unordered_map<uint32_t, M2ModelGPU> models;
    std::vector<M2Instance> instances;

    uint32_t nextInstanceId = 1;
    uint32_t lastDrawCallCount = 0;

    GLuint loadTexture(const std::string& path);
    std::unordered_map<std::string, GLuint> textureCache;
    GLuint whiteTexture = 0;
    GLuint glowTexture = 0;  // Soft radial gradient for glow sprites

    // Lighting uniforms
    glm::vec3 lightDir = glm::vec3(0.5f, 0.5f, 1.0f);
    glm::vec3 lightColor = glm::vec3(1.5f, 1.4f, 1.3f);
    glm::vec3 ambientColor = glm::vec3(0.4f, 0.4f, 0.45f);

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
    mutable std::vector<uint32_t> collisionTriScratch_;

    // Collision query profiling (per frame).
    mutable double queryTimeMs = 0.0;
    mutable uint32_t queryCallCount = 0;

    // Persistent render buffers (avoid per-frame allocation/deallocation)
    struct VisibleEntry {
        uint32_t index;
        uint32_t modelId;
        float distSq;
        float effectiveMaxDistSq;
    };
    std::vector<VisibleEntry> sortedVisible_;  // Reused each frame
    struct GlowSprite {
        glm::vec3 worldPos;
        glm::vec4 color;
        float size;
    };
    std::vector<GlowSprite> glowSprites_;  // Reused each frame

    // Animation update buffers (avoid per-frame allocation)
    std::vector<size_t> boneWorkIndices_;        // Reused each frame
    std::vector<std::future<void>> animFutures_; // Reused each frame

    // Smoke particle system
    std::vector<SmokeParticle> smokeParticles;
    GLuint smokeVAO = 0;
    GLuint smokeVBO = 0;
    std::unique_ptr<Shader> smokeShader;
    static constexpr int MAX_SMOKE_PARTICLES = 1000;
    float smokeEmitAccum = 0.0f;
    std::mt19937 smokeRng{42};

    // M2 particle emitter system
    GLuint m2ParticleShader_ = 0;
    GLuint m2ParticleVAO_ = 0;
    GLuint m2ParticleVBO_ = 0;
    static constexpr size_t MAX_M2_PARTICLES = 4000;
    std::mt19937 particleRng_{123};

    // Cached camera state from update() for frustum-culling bones
    glm::vec3 cachedCamPos_ = glm::vec3(0.0f);
    float cachedMaxRenderDistSq_ = 0.0f;

    // Thread count for parallel bone animation
    uint32_t numAnimThreads_ = 1;

    float interpFloat(const pipeline::M2AnimationTrack& track, float animTime, int seqIdx,
                      const std::vector<pipeline::M2Sequence>& seqs,
                      const std::vector<uint32_t>& globalSeqDurations);
    float interpFBlockFloat(const pipeline::M2FBlock& fb, float lifeRatio);
    glm::vec3 interpFBlockVec3(const pipeline::M2FBlock& fb, float lifeRatio);
    void emitParticles(M2Instance& inst, const M2ModelGPU& gpu, float dt);
    void updateParticles(M2Instance& inst, float dt);
};

} // namespace rendering
} // namespace wowee
