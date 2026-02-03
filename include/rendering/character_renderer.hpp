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

    void update(float deltaTime);

    void render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection);

    void setInstancePosition(uint32_t instanceId, const glm::vec3& position);
    void setInstanceRotation(uint32_t instanceId, const glm::vec3& rotation);
    void setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets);
    void setInstanceVisible(uint32_t instanceId, bool visible);
    void removeInstance(uint32_t instanceId);

    /** Attach a weapon model to a character instance at the given attachment point. */
    bool attachWeapon(uint32_t charInstanceId, uint32_t attachmentId,
                      const pipeline::M2Model& weaponModel, uint32_t weaponModelId,
                      const std::string& texturePath);

    /** Detach a weapon from the given attachment point. */
    void detachWeapon(uint32_t charInstanceId, uint32_t attachmentId);

    size_t getInstanceCount() const { return instances.size(); }

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
        std::vector<glm::mat4> boneMatrices;  // Current bone transforms

        // Geoset visibility — which submesh IDs to render
        // Empty = render all (for non-character models)
        std::unordered_set<uint16_t> activeGeosets;

        // Weapon attachments (weapons parented to this instance's bones)
        std::vector<WeaponAttachment> weaponAttachments;

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
    pipeline::AssetManager* assetManager = nullptr;

    // Texture cache
    std::unordered_map<std::string, GLuint> textureCache;
    GLuint whiteTexture = 0;

    std::unordered_map<uint32_t, M2ModelGPU> models;
    std::unordered_map<uint32_t, CharacterInstance> instances;

    uint32_t nextInstanceId = 1;

    // Maximum bones supported (GPU uniform limit)
    static constexpr int MAX_BONES = 200;
};

} // namespace rendering
} // namespace wowee
