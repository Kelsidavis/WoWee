#pragma once

#include "pipeline/m2_loader.hpp"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {

// Forward declarations
class Shader;
class Texture;
class Camera;

// Weapon attached to a character instance at a bone attachment point
struct WeaponAttachment {
    uint32_t weaponModelId;
    uint32_t weaponInstanceId;
    uint32_t attachmentId;     // 1=RightHand, 2=LeftHand
    uint16_t boneIndex;
    glm::vec3 offset;
};

/**
 * Character renderer for M2 models with skeletal animation
 *
 * Features:
 * - Skeletal animation with bone transformations
 * - Keyframe interpolation (linear position/scale, slerp rotation)
 * - Vertex skinning (GPU-accelerated)
 * - Texture loading from BLP via AssetManager
 */
class CharacterRenderer {
public:
    CharacterRenderer();
    ~CharacterRenderer();

    bool initialize();
    void shutdown();

    void setAssetManager(pipeline::AssetManager* am) { assetManager = am; }

    bool loadModel(const pipeline::M2Model& model, uint32_t id);

    uint32_t createInstance(uint32_t modelId, const glm::vec3& position,
                           const glm::vec3& rotation = glm::vec3(0.0f),
                           float scale = 1.0f);

    void playAnimation(uint32_t instanceId, uint32_t animationId, bool loop = true);

    void update(float deltaTime, const glm::vec3& cameraPos = glm::vec3(0.0f));

    void render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);
    void renderShadow(const glm::mat4& lightSpaceMatrix);

    void setInstancePosition(uint32_t instanceId, const glm::vec3& position);
    void setInstanceRotation(uint32_t instanceId, const glm::vec3& rotation);
    void moveInstanceTo(uint32_t instanceId, const glm::vec3& destination, float durationSeconds);
    void startFadeIn(uint32_t instanceId, float durationSeconds);
    const pipeline::M2Model* getModelData(uint32_t modelId) const;
    void setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets);
    void setGroupTextureOverride(uint32_t instanceId, uint16_t geosetGroup, GLuint textureId);
    void setTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot, GLuint textureId);
    void clearTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot);
    void setInstanceVisible(uint32_t instanceId, bool visible);
    void removeInstance(uint32_t instanceId);
    bool getAnimationState(uint32_t instanceId, uint32_t& animationId, float& animationTimeMs, float& animationDurationMs) const;
    bool hasAnimation(uint32_t instanceId, uint32_t animationId) const;
    bool getAnimationSequences(uint32_t instanceId, std::vector<pipeline::M2Sequence>& out) const;
    bool getInstanceModelName(uint32_t instanceId, std::string& modelName) const;
    bool getInstanceBounds(uint32_t instanceId, glm::vec3& outCenter, float& outRadius) const;

    /** Debug: Log all available animations for an instance */
    void dumpAnimations(uint32_t instanceId) const;

    /** Attach a weapon model to a character instance at the given attachment point. */
    bool attachWeapon(uint32_t charInstanceId, uint32_t attachmentId,
                      const pipeline::M2Model& weaponModel, uint32_t weaponModelId,
                      const std::string& texturePath);

    /** Detach a weapon from the given attachment point. */
    void detachWeapon(uint32_t charInstanceId, uint32_t attachmentId);

    /** Get the world-space transform of an attachment point on an instance.
     *  Used for mount seats, weapon positions, etc.
     *  @param instanceId The character/mount instance
     *  @param attachmentId The attachment point ID (0=Mount, 1=RightHand, 2=LeftHand, etc.)
     *  @param outTransform The resulting world-space transform matrix
     *  @return true if attachment found and matrix computed
     */
    bool getAttachmentTransform(uint32_t instanceId, uint32_t attachmentId, glm::mat4& outTransform);

    size_t getInstanceCount() const { return instances.size(); }

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

private:
    // GPU representation of M2 model
    struct M2ModelGPU {
        uint32_t vao = 0;
        uint32_t vbo = 0;
        uint32_t ebo = 0;

        pipeline::M2Model data;  // Original model data
        std::vector<glm::mat4> bindPose;  // Inverse bind pose matrices

        // Textures loaded from BLP (indexed by texture array position)
        std::vector<GLuint> textureIds;
    };

    // Character instance
    struct CharacterInstance {
        uint32_t id;
        uint32_t modelId;

        glm::vec3 position;
        glm::vec3 rotation;
        float scale;
        bool visible = true;  // For first-person camera hiding

        // Animation state
        uint32_t currentAnimationId = 0;
        int currentSequenceIndex = -1;  // Index into M2Model::sequences
        float animationTime = 0.0f;
        bool animationLoop = true;
        bool isDead = false;  // Prevents movement while in death state
        std::vector<glm::mat4> boneMatrices;  // Current bone transforms

        // Geoset visibility — which submesh IDs to render
        // Empty = render all (for non-character models)
        std::unordered_set<uint16_t> activeGeosets;

        // Per-geoset-group texture overrides (group → GL texture ID)
        std::unordered_map<uint16_t, GLuint> groupTextureOverrides;

        // Per-texture-slot overrides (slot → GL texture ID)
        std::unordered_map<uint16_t, GLuint> textureSlotOverrides;

        // Weapon attachments (weapons parented to this instance's bones)
        std::vector<WeaponAttachment> weaponAttachments;

        // Opacity (for fade-in)
        float opacity = 1.0f;
        float fadeInTime = 0.0f;     // elapsed fade time (seconds)
        float fadeInDuration = 0.0f; // total fade duration (0 = no fade)

        // Movement interpolation
        bool isMoving = false;
        glm::vec3 moveStart{0.0f};
        glm::vec3 moveEnd{0.0f};
        float moveDuration = 0.0f;   // seconds
        float moveElapsed = 0.0f;

        // Override model matrix (used for weapon instances positioned by parent bone)
        bool hasOverrideModelMatrix = false;
        glm::mat4 overrideModelMatrix{1.0f};
    };

    void setupModelBuffers(M2ModelGPU& gpuModel);
    void calculateBindPose(M2ModelGPU& gpuModel);
    void updateAnimation(CharacterInstance& instance, float deltaTime);
    void calculateBoneMatrices(CharacterInstance& instance);
    glm::mat4 getBoneTransform(const pipeline::M2Bone& bone, float time, int sequenceIndex);
    glm::mat4 getModelMatrix(const CharacterInstance& instance) const;

    // Keyframe interpolation helpers
    static int findKeyframeIndex(const std::vector<uint32_t>& timestamps, float time);
    static glm::vec3 interpolateVec3(const pipeline::M2AnimationTrack& track,
                                      int seqIdx, float time, const glm::vec3& defaultVal);
    static glm::quat interpolateQuat(const pipeline::M2AnimationTrack& track,
                                      int seqIdx, float time);

public:
    /**
     * Build a composited character skin texture by alpha-blending overlay
     * layers (e.g. underwear) onto a base skin BLP. Each overlay is placed
     * at the correct CharComponentTextureSections region based on its
     * filename (pelvis, torso, etc.). Returns the resulting GL texture ID.
     */
    GLuint compositeTextures(const std::vector<std::string>& layerPaths);

    /**
     * Build a composited character skin with explicit region-based equipment overlays.
     * @param basePath Body skin texture path
     * @param baseLayers Underwear overlay paths (placed by filename keyword)
     * @param regionLayers Pairs of (region_index, blp_path) for equipment textures
     * @return GL texture ID of the composited result
     */
    GLuint compositeWithRegions(const std::string& basePath,
                                const std::vector<std::string>& baseLayers,
                                const std::vector<std::pair<int, std::string>>& regionLayers);

    /** Load a BLP texture from MPQ and return the GL texture ID (cached). */
    GLuint loadTexture(const std::string& path);

    /** Replace a loaded model's texture at the given slot with a new GL texture. */
    void setModelTexture(uint32_t modelId, uint32_t textureSlot, GLuint textureId);

    /** Reset a model's texture slot back to white fallback. */
    void resetModelTexture(uint32_t modelId, uint32_t textureSlot);


private:
    std::unique_ptr<Shader> characterShader;
    GLuint shadowCasterProgram = 0;
    pipeline::AssetManager* assetManager = nullptr;

    // Fog parameters
    glm::vec3 fogColor = glm::vec3(0.5f, 0.6f, 0.7f);
    float fogStart = 400.0f;
    float fogEnd = 1200.0f;

    // Lighting parameters
    glm::vec3 lightDir = glm::vec3(0.0f, -1.0f, 0.3f);
    glm::vec3 lightColor = glm::vec3(1.5f, 1.4f, 1.3f);
    glm::vec3 ambientColor = glm::vec3(0.4f, 0.4f, 0.45f);

    // Shadow mapping
    GLuint shadowDepthTex = 0;
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
    bool shadowEnabled = false;

    // Texture cache
    struct TextureCacheEntry {
        GLuint id = 0;
        size_t approxBytes = 0;
        uint64_t lastUse = 0;
    };
    std::unordered_map<std::string, TextureCacheEntry> textureCache;
    std::unordered_map<std::string, GLuint> compositeCache_;  // key → GPU texture for reuse
    size_t textureCacheBytes_ = 0;
    uint64_t textureCacheCounter_ = 0;
    size_t textureCacheBudgetBytes_ = 1024ull * 1024 * 1024;  // Default, overridden at init
    GLuint whiteTexture = 0;

    std::unordered_map<uint32_t, M2ModelGPU> models;
    std::unordered_map<uint32_t, CharacterInstance> instances;

    uint32_t nextInstanceId = 1;

    // Maximum bones supported (GPU uniform limit)
    static constexpr int MAX_BONES = 200;
};

} // namespace rendering
} // namespace wowee
