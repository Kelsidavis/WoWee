#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <cstdint>

namespace wowee {
namespace pipeline {

struct M2Model;

// Wowee Open Model format (.wom) - novel format, no Blizzard IP
// WOM1: static geometry | WOM2: + bones + animations | WOM3: + multi-batch materials
struct WoweeModel {
    struct Vertex {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        uint8_t boneWeights[4] = {255, 0, 0, 0};
        uint8_t boneIndices[4] = {0, 0, 0, 0};
    };

    struct Bone {
        int32_t keyBoneId = -1;
        int16_t parentBone = -1;
        glm::vec3 pivot{0};
        uint32_t flags = 0;
    };

    struct AnimKeyframe {
        uint32_t timeMs;
        glm::vec3 translation;
        glm::quat rotation;
        glm::vec3 scale;
    };

    struct Animation {
        uint32_t id = 0;
        uint32_t durationMs = 0;
        float movingSpeed = 0;
        std::vector<std::vector<AnimKeyframe>> boneKeyframes; // [boneIdx][keyframe]
    };

    // WOM3: a contiguous slice of indices that draws with one material/texture.
    // Most M2 models have multiple submeshes (body, hair, eyes, etc.) - each
    // becomes one Batch in WOM3.
    struct Batch {
        uint32_t indexStart = 0;       // first index in the global index buffer
        uint32_t indexCount = 0;       // number of indices to draw
        uint32_t textureIndex = 0;     // index into texturePaths
        uint16_t blendMode = 0;        // 0=opaque, 1=alpha-test, 2=alpha, 3=add
        uint16_t flags = 0;            // bit 0 = unlit, bit 1 = two-sided, bit 2 = no z-write
    };

    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<std::string> texturePaths;
    std::vector<Bone> bones;
    std::vector<Animation> animations;
    std::vector<Batch> batches;        // empty in WOM1/WOM2, populated in WOM3
    float boundRadius = 1.0f;
    glm::vec3 boundMin{0}, boundMax{0};
    uint32_t version = 1; // 1=WOM1(static), 2=WOM2(animated), 3=WOM3(multi-batch)

    [[nodiscard]] bool isValid() const { return !vertices.empty() && !indices.empty(); }
    [[nodiscard]] bool hasAnimation() const { return !bones.empty() && !animations.empty(); }
    [[nodiscard]] bool hasBatches() const { return !batches.empty(); }
};

class WoweeModelLoader {
public:
    // Load from .wom file (binary) + .wom.json (metadata)
    static WoweeModel load(const std::string& basePath);

    // Save to .wom + .wom.json
    static bool save(const WoweeModel& model, const std::string& basePath);

    // Convert an M2 model to WoweeModel (static geometry only, no animation)
    static WoweeModel fromM2(const std::string& m2Path, class AssetManager* am);

    // Same as fromM2() but takes parsed M2 bytes + optional skin bytes
    // directly. Lets the asset extractor convert during MPQ→loose-files
    // without standing up an AssetManager.
    static WoweeModel fromM2Bytes(const std::vector<uint8_t>& m2Data,
                                   const std::vector<uint8_t>& skinData = {});

    // Check if a .wom exists
    static bool exists(const std::string& basePath);

    // Convert WoweeModel to an in-memory M2Model so the M2Renderer can consume it.
    // Single batch, single material - sufficient for static and simple animated meshes.
    static M2Model toM2(const WoweeModel& wom);

    // Convenience: try loading <path-without-ext>.wom from the standard editor
    // search paths (custom_zones/models/, output/models/). Returns valid model on hit.
    // `extraPrefixes` are tried before the defaults - pass per-zone roots like
    // {"output/<map>/models/", "custom_zones/<map>/models/"} when the caller
    // knows the active zone.
    static WoweeModel tryLoadByGamePath(
        const std::string& gamePath,
        const std::vector<std::string>& extraPrefixes = {});
};

} // namespace pipeline
} // namespace wowee
