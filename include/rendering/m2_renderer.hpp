#pragma once

#include "rendering/collision_geometry.hpp"
#include "rendering/spatial_grid.hpp"
#include "rendering/shadow_params.hpp"

#include "pipeline/m2_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "rendering/m2_model_classifier.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <string>
#include <optional>
#include <random>
#include <chrono>
#include <future>
#include <algorithm>

namespace wowee {

namespace pipeline {
    class AssetManager;
}

namespace rendering {

class Camera;
class VkContext;
class VkTexture;
class HiZSystem;

// Ceiling on the bone matrices computed and uploaded for one M2 instance.
// WoW models run well past a hundred bones - the largest shipped model has
// 315 - and a vertex weighted to a bone above this ceiling would read outside
// its instance's range in the mega bone buffer, which is what threw stray
// spikes out of water elementals and other high-bone-count creatures.
inline constexpr uint32_t kMaxBonesPerInstance = 512;

/**
 * GPU representation of an M2 model
 */
struct M2ModelGPU {
    struct BatchGPU {
        VkTexture* texture = nullptr;  // from cache, NOT owned
        VkDescriptorSet materialSet = VK_NULL_HANDLE;  // set 1
        ::VkBuffer materialUBO = VK_NULL_HANDLE;
        VmaAllocation materialUBOAlloc = VK_NULL_HANDLE;
        void* materialUBOMapped = nullptr;  // cached mapped pointer (avoids per-frame vmaGetAllocationInfo)
        uint32_t indexStart = 0;   // offset in indices (not bytes)
        uint32_t indexCount = 0;
        bool hasAlpha = false;
        bool colorKeyBlack = false;
        glm::vec3 tint{1.0f};  ///< the batch's authored colour
        uint16_t textureAnimIndex = 0xFFFF; // 0xFFFF = no texture animation
        uint16_t blendMode = 0;   // 0=Opaque, 1=AlphaKey, 2=Alpha, 3=Add, etc.
        uint16_t materialFlags = 0; // M2 material flags (0x01=Unlit, 0x04=TwoSided, 0x10=NoDepthWrite)
        uint16_t submeshLevel = 0; // LOD level: 0=base, 1=LOD1, 2=LOD2, 3=LOD3
        uint8_t textureUnit = 0;  // UV set index (0=texCoords[0], 1=texCoords[1])
        uint8_t texFlags = 0;     // M2Texture.flags (bit0=WrapS, bit1=WrapT)
        bool lanternGlowHint = false; // Texture/model hints this batch is a glow-card billboard
        bool glowCardLike = false; // Batch likely is a flat emissive card that should be sprite-replaced
        bool preserveGlowMesh = false; // Keep emissive glass/fixture mesh below its glow sprite
        // Forge fire card: the flame/coals/glow batches of a forge, as opposed
        // to the masonry and ironwork the rest of the model is made of.
        bool forgeFireCard = false;
        // A sky model's star-point layer. Suppressed when the client draws its
        // own stars instead: the authored layer is a 256x256 compressed texture
        // magnified across the whole dome, which is soft at any resolution and
        // softer at a high one.
        bool starLayer = false;
        uint8_t glowTint = 0; // 0=warm, 1=cool, 2=red
        float batchOpacity = 1.0f; // Resolved texture weight opacity (0=transparent, skip batch)
        glm::vec3 center = glm::vec3(0.0f); // Center of batch geometry (model space)
        float glowSize = 1.0f;              // Approx radius of batch geometry

        struct LightBoneAnchor {
            uint16_t bone = 0;
            // Sum of (vertex position * skin weight, skin weight), divided by
            // sampled vertex count. Transforming and summing these reproduces
            // the animated center of the skinned glow geometry.
            glm::vec4 weightedPoint{0.0f};
        };
        std::vector<LightBoneAnchor> lightBoneAnchors;
        uint16_t lightSuspensionBone = UINT16_MAX;
        glm::vec3 lightSuspensionPoint{0.0f};
    };

    ::VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VmaAllocation vertexAlloc = VK_NULL_HANDLE;
    ::VkBuffer indexBuffer = VK_NULL_HANDLE;
    VmaAllocation indexAlloc = VK_NULL_HANDLE;
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
    bool isSmallFoliage = false;    // Small foliage (bushes, grass, plants) - skip during taxi
    bool isInvisibleTrap = false;   // Invisible trap objects (don't render, no collision)
    bool isGroundDetail = false;    // Ground clutter/detail doodads (special fallback render path)
    bool isWaterVegetation = false; // Cattails, reeds, kelp etc. near water (insect spawning)
    bool isFireflyEffect = false;   // Firefly/fireflies M2 (exempt from particle dampeners)
    bool isWaterfall = false;       // Waterfall model (ambient sound + splash particles)
    bool isBrazierOrFire = false;   // Brazier / campfire / bonfire model
    bool isGroundFire = false;      // Ground fire whose halo follows its lowest flame emitter
    bool isTorch = false;           // Wall-mounted or standing torch
    bool isForge = false;           // Smithy forge (contained fire, lights its surroundings)
    AmbientEmitterType ambientEmitterType = AmbientEmitterType::None;

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
        /// The triangles of one of the two cell arrays that a query box
        /// reaches. The two queries below differ only in which they pass.
        void gatherTrisInRange(const std::vector<std::vector<uint32_t>>& cells,
                               float minX, float minY, float maxX, float maxY,
                               std::vector<uint32_t>& out) const;

        void getFloorTrisInRange(float minX, float minY, float maxX, float maxY,
                                 std::vector<uint32_t>& out) const;
        void getWallTrisInRange(float minX, float minY, float maxX, float maxY,
                                std::vector<uint32_t>& out) const;
        [[nodiscard]] bool valid() const { return triCount > 0; }
    };
    CollisionMesh collision;

    std::string name;

    // Skeletal animation data (kept from M2Model for bone computation)
    std::vector<pipeline::M2Bone> bones;
    std::vector<pipeline::M2Sequence> sequences;
    std::vector<uint32_t> globalSequenceDurations;  // Loop durations for global sequence tracks
    bool hasAnimation = false;  // True if any bone has keyframes
    bool isSmoke = false;       // True for smoke models (UV scroll animation)
    bool isSpellEffect = false;  // True for spell effect models (skip particle dampeners)
    bool isInstancePortal = false; // Instance portal model (spin + glow)
    bool disableAnimation = false; // Keep foliage/tree doodads visually stable
    bool shadowWindFoliage = false; // Apply wind sway in shadow pass for foliage/tree cards
    bool isFoliageLike = false;     // Model name matches foliage/tree/bush/grass etc (precomputed)
    bool isElvenLike = false;       // Model name matches elf/elven/quel (precomputed)
    bool isLanternLike = false;     // Model name matches lantern/lamp/light (precomputed)
    bool isKoboldFlame = false;     // Model name matches kobold+(candle/torch/mine) (precomputed)
    bool isLavaModel = false;       // Model name contains lava/molten/magma (UV scroll fallback)
    bool isSkyBird = false;         // Flying bird/bat doodad - hide until animation range
    bool isLightBeam = false;       // Lighthouse/light-ray beam - distant bones must keep updating
    bool isTransportDoodad = false; // Animated ship sail/paddle child
    bool hasTextureAnimation = false; // True if any batch has UV animation
    bool hasTransparentBatches = false; // True if any batch uses alpha-blend or additive (blendMode >= 2)
    uint8_t availableLODs = 0;  // Bitmask: bit N set if any batch has submeshLevel==N

    // Particle emitter data (kept from M2Model)
    std::vector<pipeline::M2ParticleEmitter> particleEmitters;
    std::vector<VkTexture*> particleTextures;    // Resolved Vulkan textures per emitter
    std::vector<VkDescriptorSet> particleTexSets; // Pre-allocated descriptor sets per emitter (stable, avoids per-frame alloc)

    // Ribbon emitter data (kept from M2Model)
    std::vector<pipeline::M2RibbonEmitter> ribbonEmitters;
    std::vector<VkTexture*> ribbonTextures;       // Resolved texture per ribbon emitter
    std::vector<VkDescriptorSet> ribbonTexSets;   // Descriptor sets per ribbon emitter

    // Texture transform data for UV animation
    std::vector<pipeline::M2TextureTransform> textureTransforms;
    std::vector<uint16_t> textureTransformLookup;
    std::vector<int> idleVariationIndices;  // Sequence indices for idle variations (animId 0)

    [[nodiscard]] bool isValid() const { return vertexBuffer != VK_NULL_HANDLE && indexCount > 0; }
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
    float globalSequenceTime = 0.0f; // Independent clock for global animation tracks
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

    // Ribbon emitter state
    struct RibbonEdge {
        glm::vec3 worldPos;   // Spine world position when this edge was born
        glm::vec3 color;      // Interpolated color at birth
        float     alpha;      // Interpolated alpha at birth
        float     heightAbove;// Half-width above spine
        float     heightBelow;// Half-width below spine
        float     age;        // Seconds since spawned
    };
    // One deque of edges per ribbon emitter on this instance
    std::vector<std::deque<RibbonEdge>> ribbonEdges;
    std::vector<float> ribbonEdgeAccumulators; // fractional edge counter per emitter

    // Cached model flags (set at creation to avoid per-frame hash lookups)
    bool cachedHasAnimation = false;
    bool cachedDisableAnimation = false;
    bool cachedIsSmoke = false;
    bool cachedHasParticleEmitters = false;
    bool cachedIsGroundDetail = false;
    bool cachedIsInvisibleTrap = false;
    bool cachedIsInstancePortal = false;
    bool cachedIsSkyBird = false;
    bool cachedIsLightBeam = false;
    bool cachedIsTransportDoodad = false;
    bool cachedIsValid = false;
    bool skipCollision = false;    // Fully non-collidable visual/effect instance
    bool skipWallCollision = false; // Keep authored floors, suppress only wall blocking
    float cachedBoundRadius = 0.0f;
    glm::vec3 cachedCullCenter{0.0f};              // transformed visual-bounds center
    float cachedVisualRadius = 0.0f;               // transformed visual-bounds half diagonal
    // Pre-computed per-instance cull factors (depend only on static flags + scale +
    // bound radius), populated by recomputeCachedCullFactors(). The per-frame SSBO
    // upload just multiplies by the smoothed render distance and packs the rest.
    float cachedEffectiveMaxDistSqFactor = 1.0f;  // multiplied by maxRenderDistanceSq each frame
    float cachedPaddedRadius = 0.0f;              // sphere radius used by the cull compute
    float portalSpinAngle = 0.0f;  // Accumulated spin angle for portal rotation
    const M2ModelGPU* cachedModel = nullptr;  // Avoid per-frame hash lookups

    // Result of the most recent GPU cull dispatch that actually covered this
    // instance, matched back by instance ID (never by array index - see
    // cullSubmittedIds_).  Defaults to visible so an instance created after the
    // in-flight dispatch draws immediately instead of waiting ~2 frames for its
    // first cull result.
    uint8_t lastCullVisible = 1;
    // Consecutive frames this instance has been culled (capped at 3), used for
    // the HiZ `previouslyVisible` hysteresis.  Defaults to 2 ("not recently
    // visible") so a freshly spawned instance skips the HiZ test rather than
    // being judged against depth data that never contained it.
    uint8_t hizPrevCulledFrames = 2;
    // Server game object (mailbox, chest, node, door): exempt from the adaptive
    // ambient-doodad distance, which collapses to ~200 units in a city and hid
    // interactable objects long before the server stopped sending them.
    // Set by the spawner; see M2_GAME_OBJECT_MIN_RENDER_DISTANCE.
    bool isGameObject = false;

    void recomputeCachedCullFactors();

    // Frame-skip optimization (update distant animations less frequently)
    uint8_t frameSkipCounter = 0;
    bool bonesDirty[2] = {false, false};  // Per-frame-index: set when bones recomputed, cleared after upload
    // Mega-bone slot this instance's bones were last uploaded to, per frame
    // index (0 = never uploaded; valid slots start at 1). Lets prepareRender
    // skip the memcpy when the bones are unchanged and still at the same slot.
    uint32_t megaBoneUploadedSlot[2] = {0, 0};

    // Per-instance bone SSBO (double-buffered) - legacy; see mega bone SSBO in M2Renderer
    ::VkBuffer boneBuffer[2] = {};
    VmaAllocation boneAlloc[2] = {};
    void* boneMapped[2] = {};
    VkDescriptorSet boneSet[2] = {};

    // Mega bone SSBO offset - base bone index for this instance (set per-frame in prepareRender)
    uint32_t megaBoneOffset = 0;

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

// M2 material UBO - matches M2Material in m2.frag.glsl (set 1, binding 2)
struct M2MaterialUBO {
    int32_t hasTexture;
    int32_t alphaTest;
    int32_t colorKeyBlack;
    float colorKeyThreshold;
    int32_t unlit;
    int32_t blendMode;
    float fadeAlpha;
    float interiorDarken;
    float specularIntensity;
    float emissiveBoost;
    // The batch's authored colour, from the M2's colour track, with any
    // flicker already folded in. Three scalars rather than a vec3 so the
    // std140 rules stay trivial.
    float tintR;
    float tintG;
    float tintB;
};

// M2 params UBO - matches M2Params in m2.vert.glsl (set 1, binding 1)
struct M2ParamsUBO {
    float uvOffsetX;
    float uvOffsetY;
    int32_t texCoordSet;
    int32_t useBones;
};

/**
 * M2 Model Renderer (Vulkan)
 *
 * Handles rendering of M2 models (doodads like trees, rocks, bushes)
 */
class M2Renderer {
public:
    M2Renderer();
    ~M2Renderer();

    [[nodiscard]] bool initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                    pipeline::AssetManager* assets);
    /** Configure this renderer for camera-centered sky M2s before initialize(). */
    void setSkyMode(bool enabled) { skyMode_ = enabled; }
    void shutdown();

    [[nodiscard]] bool hasModel(uint32_t modelId) const;
    bool loadModel(const pipeline::M2Model& model, uint32_t modelId);
    /** Force-remove a model and all its GPU resources. Caller must ensure no instances reference it. */
    void unloadModel(uint32_t modelId);
    /** Mark a loaded model as a spell effect (full-brightness particles, no collision). */
    void markModelAsSpellEffect(uint32_t modelId);

    /// Instances of the same model at the same spot are treated as one placement,
    /// which is right for the static world and wrong for anything whose position
    /// arrives later from a parent. Pass allowPositionDedup=false for a child
    /// instance created at a placeholder position - see the note in
    /// setInstanceTransform on why the placeholder key outlives the placement.
    uint32_t createInstance(uint32_t modelId, const glm::vec3& position,
                            const glm::vec3& rotation = glm::vec3(0.0f),
                            float scale = 1.0f,
                            bool allowPositionDedup = true);
    uint32_t createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                       const glm::vec3& position);

    void update(float deltaTime, const glm::vec3& cameraPos, const glm::mat4& viewProjection);

    /**
     * Render all visible instances (Vulkan)
     */
    /** Pre-allocate GPU resources (bone SSBOs, descriptors) on main thread before parallel render. */
    void prepareRender(uint32_t frameIndex, const Camera& camera);
    /** Dispatch GPU frustum culling compute shader on primary cmd before render pass. */
    void dispatchCullCompute(VkCommandBuffer cmd, uint32_t frameIndex, const Camera& camera);
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera);

    /** Gather the nearest authored glow cards as inexpensive scene point lights. */
    uint32_t gatherLocalLights(const glm::vec3& cameraPos,
                               glm::vec4* outPosRadius,
                               glm::vec4* outColorIntensity,
                               uint32_t maxLights) const;

    /** Set the HiZ system for occlusion culling (Phase 6.3). nullptr disables HiZ. */
    void setHiZSystem(HiZSystem* hiz) { hizSystem_ = hiz; }
    void setForceNoCull(bool v) { forceNoCull_ = v; }

    /** Ensure GPU→CPU cull output is visible to the host after a fence wait.
     *  Call after the early compute submission finishes (endSingleTimeCommands). */
    void invalidateCullOutput(uint32_t frameIndex);

    /**
     * Initialize shadow pipeline (Phase 7)
     */
    [[nodiscard]] bool initializeShadow(VkRenderPass shadowRenderPass);
    [[nodiscard]] bool hasShadowPipeline() const { return shadowPipeline_ != VK_NULL_HANDLE; }

    /**
     * Render depth-only pass for shadow casting
     */
    void renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix, float globalTime = 0.0f,
                      const glm::vec3& shadowCenter = glm::vec3(0), float shadowRadius = 1e9f);

    /**
     * Render M2 particle emitters (point sprites)
     */
    void renderM2Particles(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);

    /**
     * Render smoke particles from chimneys etc.
     */
    void renderSmokeParticles(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);

    /**
     * Render M2 ribbon emitters (spell trails / wing effects)
     */
    void renderM2Ribbons(VkCommandBuffer cmd, VkDescriptorSet perFrameSet);

    void setInstancePosition(uint32_t instanceId, const glm::vec3& position);
    void setInstanceTransform(uint32_t instanceId, const glm::mat4& transform);
    void setInstanceAnimationFrozen(uint32_t instanceId, bool frozen);
    /// Set the animation sequence by animation ID (e.g. anim::OPEN, anim::CLOSE).
    /// Finds the first sequence with matching ID. Unfreezes the instance and resets time.
    void setInstanceAnimation(uint32_t instanceId, uint32_t animationId, bool loop = true);
    /// Check if a model instance has a specific animation ID in its sequence table.
    [[nodiscard]] bool hasAnimation(uint32_t instanceId, uint32_t animationId) const;
    [[nodiscard]] float getInstanceAnimDuration(uint32_t instanceId) const;
    /// World-space visual bounding sphere of an instance (center + radius),
    /// used for cursor picking and selection circles. Returns false for an
    /// unknown instance or a degenerate (zero-radius) model.
    bool getInstanceBounds(uint32_t instanceId, glm::vec3& outCenter, float& outRadius) const;
    /// True while the instance is still live in the renderer. Owners that cache
    /// instance IDs (game objects, transports) use this to notice an instance
    /// that was dropped underneath them - e.g. by a renderer-wide clear - and
    /// respawn it instead of silently addressing a dead handle forever.
    [[nodiscard]] bool hasInstance(uint32_t instanceId) const {
        return instanceIndexById.find(instanceId) != instanceIndexById.end();
    }
    void removeInstance(uint32_t instanceId);
    void removeInstances(const std::vector<uint32_t>& instanceIds);
    /// Mark an instance as a server game object so the adaptive doodad render
    /// distance can't cull it while the server still considers it visible.
    void setInstanceIsGameObject(uint32_t instanceId, bool isGameObject);
    void setSkipCollision(uint32_t instanceId, bool skip);
    void setSkipWallCollision(uint32_t instanceId, bool skip);
    void clear();
    /** Drop all instances but keep models in GPU memory. Cheap path for the
     *  editor's rebuild loop where the same model is re-instanced repeatedly. */
    void clearInstances();
    void cleanupUnusedModels();
    /// Keep a model resident even while it has no instances. Game object models
    /// are a small, bounded set (one per display ID actually encountered) that
    /// goes instance-free every time the player leaves an area and comes back -
    /// exactly the reap/reload churn seen in the field. Pinning them keeps that
    /// cycle out of the picture entirely; ambient doodads still get reaped.
    void setModelPinned(uint32_t modelId, bool pinned);
    /** Return and clear the model IDs evicted by cleanupUnusedModels() since the
     *  last call. Lets callers that track uploaded models drop stale entries. */
    std::vector<uint32_t> drainReapedModelIds();

    bool checkCollision(const glm::vec3& from, const glm::vec3& to,
                        glm::vec3& adjustedPos, float playerRadius = 0.5f) const;
    std::optional<float> getFloorHeight(float glX, float glY, float glZ, float* outNormalZ = nullptr) const;
    [[nodiscard]] float raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const;
    void setCollisionFocus(const glm::vec3& worldPos, float radius);

    void resetQueryStats();
    [[nodiscard]] double getQueryTimeMs() const { return queryTimeMs; }
    [[nodiscard]] uint32_t getQueryCallCount() const { return queryCallCount; }

    void recreatePipelines();

    /// Build the nine main-pass pipelines. Called by initialize() and again by
    /// recreatePipelines() after a device loss, which is the reason it exists:
    /// the two used to be separate copies of the same 190 lines.
    bool buildMainPassPipelines(VkDescriptorSetLayout perFrameLayout);

    // Stats
    [[nodiscard]] bool isInitialized() const { return initialized_; }
    [[nodiscard]] uint32_t getModelCount() const { return static_cast<uint32_t>(models.size()); }
    [[nodiscard]] uint32_t getInstanceCount() const { return static_cast<uint32_t>(instances.size()); }
    [[nodiscard]] uint32_t getTotalTriangleCount() const;
    [[nodiscard]] uint32_t getDrawCallCount() const { return lastDrawCallCount; }

    // Lighting/fog/shadow are now in per-frame UBO; these are no-ops for API compat
    void setFog(const glm::vec3& /*color*/, float /*start*/, float /*end*/) {}
    void setLighting(const float /*lightDirIn*/[3], const float /*lightColorIn*/[3],
                     const float /*ambientColorIn*/[3]) {}
    void setShadowMap(uint32_t /*depthTex*/, const glm::mat4& /*lightSpace*/) {}
    void clearShadowMap() {}

    /// How far the furthest doodad drawn this frame was. See
    /// Renderer::logViewDistanceDiag.
    [[nodiscard]] float getFurthestDrawnDistance() const { return std::sqrt(furthestDrawnSq_); }
    /// How many particles to emit, as a fraction of what a model asks for -
    /// the game's Particle Density.
    ///
    /// Applied to the authored rate before the floor that holds flames
    /// together, so turning it down thins smoke, dust and spell effects while
    /// a candle keeps enough particles alive to still read as a flame. See the
    /// note in m2_renderer_particles.cpp: below that floor a fire does not get
    /// smaller, it disappears.
    void setParticleDensity(float density) {
        particleDensity_ = std::clamp(density, 0.05f, 1.0f);
    }
    [[nodiscard]] float particleDensity() const { return particleDensity_; }

    void setSuppressBakedStars(bool suppress) { suppressBakedStars_ = suppress; }
    void setInsideInterior(bool inside) { insideInterior = inside; }
    void setOnTaxi(bool onTaxi) { onTaxi_ = onTaxi; }
    void setViewDistance(float distance) {
        viewDistanceRaw_ = std::clamp(distance, 400.0f, 2400.0f);
        // And the distance itself, as a ceiling. The scale multiplies a
        // constant tuned for scene density - 2800 yards where models are
        // sparse - so at 2400 it put doodads 5600 yards out while the terrain
        // and the WMOs both stop at the 2400 the player asked for. Distant
        // trees and buildings then stood on nothing.
        viewDistanceAbsolute_ = viewDistanceRaw_;
        recomputeViewDistanceScale();
    }

    /// How far the world's clutter is drawn, against how far the world is -
    /// the game's Environment Detail.
    ///
    /// View distance is how far there is anything to see; this is how much of
    /// the scenery inside that is worth drawing, and it only ever takes away.
    /// Above 1 it changes nothing: the ceiling above is the terrain's own, and
    /// a doodad past that is a tree standing on nothing, which is the fault
    /// that ceiling was added for.
    /// How far the ground cover is drawn - the game's Ground Clutter Radius.
    ///
    /// Its own distance rather than a share of the doodad one: grass stops
    /// between 70 and 140 yards in the original client while doodads run to
    /// the horizon, and the setting names those yards outright.
    void setGroundDetailDistance(float yards) {
        groundDetailMaxDistance_ = std::clamp(yards, 0.0f, 500.0f);
    }
    [[nodiscard]] float groundDetailDistance() const { return groundDetailMaxDistance_; }

    void setEnvironmentDetail(float detail) {
        environmentDetail_ = std::clamp(detail, 0.25f, 1.5f);
        recomputeViewDistanceScale();
    }
    [[nodiscard]] float environmentDetail() const { return environmentDetail_; }

    [[nodiscard]] std::vector<glm::vec3> getWaterVegetationPositions(const glm::vec3& camPos, float maxDist) const;

    // Pre-decoded BLP cache: set by terrain manager before calling loadModel()
    // so loadTexture() can skip the expensive assetManager->loadTexture() call.
    void setPredecodedBLPCache(std::unordered_map<std::string, pipeline::BLPImage>* cache) { predecodedBLPCache_ = cache; }

private:
    bool initialized_ = false;
    bool insideInterior = false;
    bool onTaxi_ = false;
    pipeline::AssetManager* assetManager = nullptr;

    // Vulkan context
    VkContext* vkCtx_ = nullptr;

    // Vulkan pipelines (one per blend mode)
    VkPipeline opaquePipeline_ = VK_NULL_HANDLE;       // blend mode 0
    VkPipeline alphaTestPipeline_ = VK_NULL_HANDLE;     // blend mode 1
    VkPipeline alphaPipeline_ = VK_NULL_HANDLE;         // blend mode 2
    VkPipeline additivePipeline_ = VK_NULL_HANDLE;      // blend mode 3+
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;

    // Shadow rendering (Phase 7)
    VkPipeline shadowPipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout shadowPipelineLayout_ = VK_NULL_HANDLE;
    /// The set the shadow pass binds. Five separate members before,
    /// built and torn down here and in three other renderers.
    ShadowParamsSet shadowParams_;
    // Per-frame pools for foliage shadow texture descriptor sets (one per frame-in-flight)
    static constexpr uint32_t kShadowTexPoolFrames = 2;
    VkDescriptorPool shadowTexPool_[kShadowTexPoolFrames] = {};

    // Particle pipelines
    VkPipeline particlePipeline_ = VK_NULL_HANDLE;       // M2 emitter particles
    VkPipeline particleAdditivePipeline_ = VK_NULL_HANDLE; // Additive particle blend
    VkPipelineLayout particlePipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline smokePipeline_ = VK_NULL_HANDLE;           // Smoke particles
    VkPipelineLayout smokePipelineLayout_ = VK_NULL_HANDLE;

    // Ribbon pipelines (additive + alpha-blend)
    VkPipeline ribbonPipeline_ = VK_NULL_HANDLE;          // Alpha-blend ribbons
    VkPipeline ribbonAdditivePipeline_ = VK_NULL_HANDLE;  // Additive ribbons
    VkPipelineLayout ribbonPipelineLayout_ = VK_NULL_HANDLE;
    /// The per-frame set layout initialize() was given. recreatePipelines()
    /// runs long after that call and needs the same one.
    VkDescriptorSetLayout perFrameLayout_ = VK_NULL_HANDLE;

    // Descriptor set layouts
    VkDescriptorSetLayout materialSetLayout_ = VK_NULL_HANDLE;  // set 1
    VkDescriptorSetLayout boneSetLayout_ = VK_NULL_HANDLE;      // set 2
    VkDescriptorSetLayout particleTexLayout_ = VK_NULL_HANDLE;  // particle set 1 (texture only)

    // Descriptor pools
    VkDescriptorPool materialDescPool_ = VK_NULL_HANDLE;
    VkDescriptorPool boneDescPool_ = VK_NULL_HANDLE;
    std::shared_ptr<std::atomic<uint64_t>> boneDescPoolGeneration_ =
        std::make_shared<std::atomic<uint64_t>>(0);
    static constexpr uint32_t MAX_MATERIAL_SETS = 16384;
    static constexpr uint32_t MAX_BONE_SETS = 16384;

    // Dummy identity bone buffer + descriptor set for non-animated models.
    // The pipeline layout declares set 2 (bones) and some drivers (Intel ANV)
    // require all declared sets to be bound even when the shader doesn't access them.
    ::VkBuffer dummyBoneBuffer_ = VK_NULL_HANDLE;
    VmaAllocation dummyBoneAlloc_ = VK_NULL_HANDLE;
    VkDescriptorSet dummyBoneSet_ = VK_NULL_HANDLE;

    // Mega bone SSBO - consolidates all per-instance bone matrices into a single buffer per frame.
    // Replaces per-instance bone SSBOs for fewer descriptor binds and enables GPU instancing.
    static constexpr uint32_t MEGA_BONE_MAX_INSTANCES = 4096;
    // Per-instance bone ranges are packed at their model's true bone count
    // rather than at a fixed stride, so the buffer is sized in matrices. The
    // budget matches the old 4096 x 128 slots; models under the old stride
    // now leave room for the ones above it.
    static constexpr uint32_t MEGA_BONE_MATRIX_CAPACITY = MEGA_BONE_MAX_INSTANCES * 128;
    ::VkBuffer megaBoneBuffer_[2] = {};
    VmaAllocation megaBoneAlloc_[2] = {};
    void* megaBoneMapped_[2] = {};
    VkDescriptorSet megaBoneSet_[2] = {};

    // GPU instance data SSBO - per-instance transforms, fade, bones for instanced draws.
    // Shader reads instanceData[push.instanceDataOffset + gl_InstanceIndex].
    struct M2InstanceGPU {
        glm::mat4 model;           // 64 bytes @ offset 0
        glm::vec2 uvOffset;        //  8 bytes @ offset 64
        float fadeAlpha;           //  4 bytes @ offset 72
        int32_t useBones;          //  4 bytes @ offset 76
        int32_t boneBase;          //  4 bytes @ offset 80
        int32_t boneCount;         //  4 bytes @ offset 84 - clamps skinning reads
        int32_t _pad[2] = {};      //  8 bytes @ offset 88 - align to 96 (std430)
    };
    // How many instances one frame may hand the GPU, not how many exist. Ground
    // clutter is what fills it: it is drawn by the thousand and every tuft
    // takes a slot of its own, so at 16384 a dwarf standing in Dun Morogh
    // grass was already losing whole models for a frame at a time - which reads
    // as clutter blinking rather than as anything being over budget. Two
    // buffers of 96 bytes a slot, so this costs 6 MB rather than 3.
    static constexpr uint32_t MAX_INSTANCE_DATA = 32768;
    VkDescriptorSetLayout instanceSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool instanceDescPool_ = VK_NULL_HANDLE;
    ::VkBuffer instanceBuffer_[2] = {};
    VmaAllocation instanceAlloc_[2] = {};
    void* instanceMapped_[2] = {};
    VkDescriptorSet instanceSet_[2] = {};
    uint32_t instanceDataCount_ = 0; // reset each frame in render()

    // GPU Frustum Culling via Compute Shader
    // Compute shader tests each M2 instance against frustum planes + distance, writes visibility[].
    // CPU reads back visibility to build sortedVisible_ without per-instance frustum/distance tests.
    struct CullInstanceGPU {        // matches CullInstance in m2_cull.comp.glsl (32 bytes, std430)
        glm::vec4 sphere;           // xyz = world position, w = padded radius
        float effectiveMaxDistSq;   // adaptive distance cull threshold
        uint32_t flags;             // bit 0 = valid, bit 1 = smoke, bit 2 = invisibleTrap
        float _pad[2] = {};
    };
    struct CullUniformsGPU {        // matches CullUniforms in m2_cull_hiz.comp.glsl (std140)
        glm::vec4 frustumPlanes[6]; // xyz = normal, w = distance         (96 bytes)
        glm::vec4 cameraPos;        // xyz = camera position, w = maxPossibleDistSq (16 bytes)
        uint32_t instanceCount;     //                                    (4 bytes)
        uint32_t hizEnabled;        // 1 = HiZ occlusion active           (4 bytes)
        uint32_t hizMipLevels;      // mip levels in HiZ pyramid          (4 bytes)
        uint32_t _pad2 = {};        //                                    (4 bytes)
        glm::vec4 hizParams;        // x=pyramidW, y=pyramidH, z=nearPlane, w=unused (16 bytes)
        glm::mat4 viewProj;         // current frame view-projection                 (64 bytes)
        glm::mat4 prevViewProj;     // previous frame VP for HiZ reprojection        (64 bytes)
    };                              // Total: 272 bytes
    static constexpr uint32_t MAX_CULL_INSTANCES = 24576;
    VkPipeline cullPipeline_ = VK_NULL_HANDLE;           // frustum-only (fallback)
    VkPipeline cullHiZPipeline_ = VK_NULL_HANDLE;        // frustum + HiZ occlusion
    VkPipelineLayout cullPipelineLayout_ = VK_NULL_HANDLE;  // frustum-only layout (set 0)
    VkPipelineLayout cullHiZPipelineLayout_ = VK_NULL_HANDLE; // HiZ layout (set 0 + set 1)
    VkDescriptorSetLayout cullSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool cullDescPool_ = VK_NULL_HANDLE;
    VkDescriptorSet cullSet_[2] = {};               // double-buffered
    ::VkBuffer cullUniformBuffer_[2] = {};           // frustum planes + camera + HiZ params (UBO)
    VmaAllocation cullUniformAlloc_[2] = {};
    void* cullUniformMapped_[2] = {};
    ::VkBuffer cullInputBuffer_[2] = {};             // per-instance bounding sphere + flags (SSBO)
    VmaAllocation cullInputAlloc_[2] = {};
    void* cullInputMapped_[2] = {};
    ::VkBuffer cullOutputBuffer_[2] = {};            // uint visibility[] (SSBO, host-readable)
    VmaAllocation cullOutputAlloc_[2] = {};
    void* cullOutputMapped_[2] = {};

    // HiZ occlusion culling (Phase 6.3) - optional, driven by Renderer
    HiZSystem* hizSystem_ = nullptr;

    // Previous frame's view-projection for temporal reprojection in HiZ culling.
    // Stored each frame so the cull shader can project into the same screen space
    // as the depth buffer the HiZ pyramid was built from.
    glm::mat4 prevVP_{1.0f};

    // Instance IDs, in dispatch order, for the cull results a frame slot holds.
    //
    // The visibility SSBO is read back one full slot cycle after it is written:
    // dispatchCullCompute() records a dispatch into this frame's command buffer,
    // and render() reads the mapped output of the dispatch recorded on the SAME
    // slot ~2 frames earlier.  Array indices are not stable across that gap -
    // createInstance() appends and removeInstance() swap-removes, so slot i can
    // easily describe a different object by the time it is read.  Recording the
    // IDs lets render() match each result back to the instance it was computed
    // for.  cullSubmittedIds_ is the dispatch just recorded (results not ready);
    // cullReadableIds_ is the previous one, matching the mapped data now.
    std::vector<uint32_t> cullSubmittedIds_[2];
    std::vector<uint32_t> cullReadableIds_[2];

    // Dynamic ribbon vertex buffer (CPU-written triangle strip)
    static constexpr size_t MAX_RIBBON_VERTS = 2048;  // 9 floats each
    ::VkBuffer ribbonVB_ = VK_NULL_HANDLE;
    VmaAllocation ribbonVBAlloc_ = VK_NULL_HANDLE;
    void* ribbonVBMapped_ = nullptr;

    // Dynamic particle buffers
    ::VkBuffer smokeVB_ = VK_NULL_HANDLE;
    VmaAllocation smokeVBAlloc_ = VK_NULL_HANDLE;
    void* smokeVBMapped_ = nullptr;
    ::VkBuffer m2ParticleVB_ = VK_NULL_HANDLE;
    VmaAllocation m2ParticleVBAlloc_ = VK_NULL_HANDLE;
    void* m2ParticleVBMapped_ = nullptr;
    // Dedicated glow sprite vertex buffer (separate from particle VB to avoid data race)
    static constexpr size_t MAX_GLOW_SPRITES = 8000;
    ::VkBuffer glowVB_ = VK_NULL_HANDLE;
    VmaAllocation glowVBAlloc_ = VK_NULL_HANDLE;
    void* glowVBMapped_ = nullptr;

    std::unordered_map<uint32_t, M2ModelGPU> models;
    // Grace period for model cleanup: track when a model first became instanceless.
    // Models are only evicted after 60 seconds with no instances.
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> modelUnusedSince_;
    std::unordered_set<uint32_t> pinnedModelIds_;
    // Model IDs evicted by cleanupUnusedModels() since the last drain. Owners that
    // cache "already uploaded" model IDs (e.g. TerrainManager) reconcile against
    // this so a reaped model is re-loaded instead of skipped as a stale hit.
    std::vector<uint32_t> reapedModelIds_;
    std::vector<M2Instance> instances;

    // O(1) dedup: key = (modelId, quantized x, quantized y, quantized z) → instanceId
    struct DedupKey {
        uint32_t modelId;
        int32_t qx, qy, qz; // position quantized to 0.1 units
        bool operator==(const DedupKey& o) const {
            return modelId == o.modelId && qx == o.qx && qy == o.qy && qz == o.qz;
        }
    };
    struct DedupHash {
        size_t operator()(const DedupKey& k) const {
            size_t h = std::hash<uint32_t>()(k.modelId);
            h ^= std::hash<int32_t>()(k.qx) * 2654435761u;
            h ^= std::hash<int32_t>()(k.qy) * 40503u;
            h ^= std::hash<int32_t>()(k.qz) * 12289u;
            return h;
        }
    };
    std::unordered_map<DedupKey, uint32_t, DedupHash> instanceDedupMap_;

    uint32_t nextInstanceId = 1;
    uint32_t lastDrawCallCount = 0;
    size_t modelCacheLimit_ = 6000;
    uint32_t modelLimitRejectWarnings_ = 0;

    VkTexture* loadTexture(const std::string& path, uint32_t texFlags = 0);
    std::unordered_map<std::string, pipeline::BLPImage>* predecodedBLPCache_ = nullptr;

    struct TextureCacheEntry {
        std::unique_ptr<VkTexture> texture;
        size_t approxBytes = 0;
        uint64_t lastUse = 0;
        bool hasAlpha = true;
        bool colorKeyBlack = false;
    };
    std::unordered_map<std::string, TextureCacheEntry> textureCache;
    struct TextureProperties {
        bool hasAlpha = false;
        bool colorKeyBlack = false;
    };
    std::unordered_map<VkTexture*, TextureProperties> texturePropsByPtr_;
    size_t textureCacheBytes_ = 0;
    uint64_t textureCacheCounter_ = 0;
    size_t textureCacheBudgetBytes_ = 2048ull * 1024 * 1024;
    std::unordered_set<std::string> failedTextureCache_;
    std::unordered_map<std::string, uint64_t> failedTextureRetryAt_;
    std::unordered_set<std::string> loggedTextureLoadFails_;
    uint64_t textureLookupSerial_ = 0;
    uint32_t textureBudgetRejectWarnings_ = 0;
    std::unique_ptr<VkTexture> whiteTexture_;
    std::unique_ptr<VkTexture> glowTexture_;
    VkDescriptorSet glowTexDescSet_ = VK_NULL_HANDLE;  // cached glow texture descriptor (allocated once)

    // Optional query-space culling for collision/raycast hot paths.
    CollisionFocus collisionFocus;

    void rebuildSpatialIndex();
    void gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax, std::vector<size_t>& outIndices) const;

    SpatialGrid spatialGrid;
    std::unordered_map<uint32_t, size_t> instanceIndexById;
    // Instance to copy bone matrices from when spawning another of the same
    // model. Finding one by scanning every instance is O(n) per spawn, which
    // became 18ms for a single createInstance once a zone held tens of
    // thousands of them.
    std::unordered_map<uint32_t, uint32_t> boneSeedInstanceByModel_;
    // Collision scratch buffers are thread_local (see m2_renderer.cpp) for thread-safety.

    // Collision query profiling - atomic because getFloorHeight is dispatched
    // on async threads from camera_controller while the main thread reads these.
    mutable std::atomic<double> queryTimeMs{0.0};
    mutable std::atomic<uint32_t> queryCallCount{0};

    // Persistent render buffers (avoid per-frame allocation/deallocation)
    struct VisibleEntry {
        uint32_t index;
        uint32_t modelId;
        float distSq;
        float effectiveMaxDistSq;
    };
    std::vector<VisibleEntry> sortedVisible_;  // Reused each frame
    std::vector<VisibleEntry> transparentVisible_; // Only models needing pass 2
    struct GlowSprite {
        glm::vec3 worldPos;
        glm::vec4 color;
        float size;
    };
    std::vector<GlowSprite> glowSprites_;  // Reused each frame

    // Shadow-pass texture descriptor cache (reused each frame, cleared via pool reset)
    std::unordered_map<VkImageView, VkDescriptorSet> shadowTexSetCache_;

    // Ribbon draw-call list (reused each frame)
    struct RibbonDrawCall {
        VkDescriptorSet texSet;
        VkPipeline      pipeline;
        uint32_t        firstVertex;
        uint32_t        vertexCount;
    };
    std::vector<RibbonDrawCall> ribbonDraws_;

    // Particle group structures (reused each frame)
    struct ParticleGroupKey {
        VkTexture* texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;
        bool operator==(const ParticleGroupKey& other) const {
            return texture == other.texture &&
                   blendType == other.blendType &&
                   tilesX == other.tilesX &&
                   tilesY == other.tilesY;
        }
    };
    struct ParticleGroupKeyHash {
        size_t operator()(const ParticleGroupKey& key) const {
            size_t h1 = std::hash<uintptr_t>{}(reinterpret_cast<uintptr_t>(key.texture));
            size_t h2 = std::hash<uint32_t>{}((static_cast<uint32_t>(key.tilesX) << 16) | key.tilesY);
            size_t h3 = std::hash<uint8_t>{}(key.blendType);
            return h1 ^ (h2 * 0x9e3779b9u) ^ (h3 * 0x85ebca6bu);
        }
    };
    struct ParticleGroup {
        VkTexture* texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;
        VkDescriptorSet preAllocSet = VK_NULL_HANDLE;
        std::vector<float> vertexData;
    };
    std::unordered_map<ParticleGroupKey, ParticleGroup, ParticleGroupKeyHash> particleGroups_;

    // Animation update buffers (avoid per-frame allocation)
    std::vector<size_t> boneWorkIndices_;        // Reused each frame
    std::vector<std::future<void>> animFutures_; // Reused each frame
    bool spatialIndexDirty_ = false;

    // Fast-path instance index lists (rebuilt in rebuildSpatialIndex / on create)
    std::vector<size_t> animatedInstanceIndices_;   // hasAnimation && !disableAnimation
    std::vector<size_t> particleOnlyInstanceIndices_; // !hasAnimation && hasParticleEmitters
    std::vector<size_t> particleInstanceIndices_;    // ALL instances with particle emitters

    // Smoke particle system
    std::vector<SmokeParticle> smokeParticles;
    std::vector<size_t> smokeInstanceIndices_;  // Indices into instances[] for smoke emitters
    std::vector<size_t> portalInstanceIndices_; // Indices into instances[] for spinning portals
    static constexpr int MAX_SMOKE_PARTICLES = 1000;
    float smokeEmitAccum = 0.0f;
    std::mt19937 smokeRng{42};

    // M2 particle emitter system
    static constexpr size_t MAX_M2_PARTICLES = 4000;
    std::mt19937 particleRng_{123};
    bool skyMode_ = false;
    // What the sky-model clock diagnostic last reported, so it prints on a
    // restart or once a second rather than every frame. See M2Renderer::update.
    uint32_t skyDiagInstanceId_ = 0;
    float skyDiagAnimTime_ = 0.0f;
    /// Whether the sky model was drawn last frame, so the cull
    /// diagnostic prints on a change rather than every frame.
    bool skyDiagWasDrawn_ = false;
    /// Sky batches that reached a draw call this frame, and the last counts
    /// reported, so the line prints on a change rather than every frame.
    uint32_t skyDiagDrawsOpaque_ = 0;
    uint32_t skyDiagDrawsTransparent_ = 0;
    uint32_t skyDiagLastOpaque_ = 0xFFFFFFFFu;
    uint32_t skyDiagLastTransparent_ = 0xFFFFFFFFu;

    // Cached camera state from update() for frustum-culling bones
    glm::vec3 cachedCamPos_ = glm::vec3(0.0f);
    float cachedMaxRenderDistSq_ = 0.0f;
    float smoothedRenderDist_ = 1000.0f;  // Smoothed render distance to prevent flickering
    /// The distance as asked for, kept so the scale can be rebuilt when the
    /// detail setting changes without the caller having to say it again.
    float viewDistanceRaw_ = 1200.0f;
    float environmentDetail_ = 1.0f;
    float groundDetailMaxDistance_ = 0.0f;   // 0 = no cap of its own
    void recomputeViewDistanceScale() {
        viewDistanceScale_ = (viewDistanceRaw_ / 1200.0f) * environmentDetail_;
    }
    float viewDistanceScale_ = 1.0f;
    float viewDistanceAbsolute_ = 1200.0f;
    /// Drop the sky model's baked star layer, because something else is
    /// drawing stars. See BatchGPU::starLayer.
    bool suppressBakedStars_ = false;
    float particleDensity_ = 1.0f;
    float furthestDrawnSq_ = 0.0f;
    bool forceNoCull_ = false;

    // Thread count for parallel bone animation
    uint32_t numAnimThreads_ = 1;

    float interpFloat(const pipeline::M2AnimationTrack& track, float animTime,
                      float globalTime, int seqIdx,
                      const std::vector<uint32_t>& globalSeqDurations);
    float interpFBlockFloat(const pipeline::M2FBlock& fb, float lifeRatio);
    glm::vec3 interpFBlockVec3(const pipeline::M2FBlock& fb, float lifeRatio);
    void emitParticles(M2Instance& inst, const M2ModelGPU& gpu, float dt);
    void updateParticles(M2Instance& inst, float dt);
    void updateRibbons(M2Instance& inst, const M2ModelGPU& gpu, float dt);

    // Helper to allocate descriptor sets
    VkDescriptorSet allocateMaterialSet();
    VkDescriptorSet allocateBoneSet();

    // Helper to destroy model GPU resources
    void destroyModelGPU(M2ModelGPU& model);
    // Helper to destroy instance bone buffers.
    // When defer=true, destruction is scheduled via deferAfterFrameFence so
    // in-flight command buffers are not invalidated (use for streaming unload).
    /// Starts a new instance's animation and seeds its bones from a
    /// sibling of the same model, so it draws on the frame it spawns.
    /// Both spawn paths need it and each used to have its own copy.
    void seedInstanceAnimation(const M2ModelGPU& model, uint32_t modelId,
                               M2Instance& instance);

    void destroyInstanceBones(M2Instance& inst, bool defer = false);
};

} // namespace rendering
} // namespace wowee
