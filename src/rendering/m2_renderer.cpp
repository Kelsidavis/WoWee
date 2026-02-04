#include "rendering/m2_renderer.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <chrono>
#include <cctype>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {

namespace {

void getTightCollisionBounds(const M2ModelGPU& model, glm::vec3& outMin, glm::vec3& outMax) {
    glm::vec3 center = (model.boundMin + model.boundMax) * 0.5f;
    glm::vec3 half = (model.boundMax - model.boundMin) * 0.5f;

    // Per-shape collision fitting:
    // - small solid props (boxes/crates/chests): tighter than full mesh, but
    //   larger than default to prevent walk-through on narrow objects
    // - default: tighter fit (avoid oversized blockers)
    // - stepped low platforms (tree curbs/planters): wider XY + lower Z
    if (model.collisionNarrowVerticalProp) {
        // Tall thin props (lamps/posts): keep passable gaps near walls.
        half.x *= 0.30f;
        half.y *= 0.30f;
        half.z *= 0.96f;
    } else if (model.collisionSmallSolidProp) {
        // Keep full tight mesh bounds for small solid props to avoid clip-through.
        half.x *= 1.00f;
        half.y *= 1.00f;
        half.z *= 1.00f;
    } else if (model.collisionSteppedLowPlatform) {
        half.x *= 0.98f;
        half.y *= 0.98f;
        half.z *= 0.52f;
    } else {
        half.x *= 0.66f;
        half.y *= 0.66f;
        half.z *= 0.76f;
    }

    outMin = center - half;
    outMax = center + half;
}

float getEffectiveCollisionTopLocal(const M2ModelGPU& model,
                                    const glm::vec3& localPos,
                                    const glm::vec3& localMin,
                                    const glm::vec3& localMax) {
    if (!model.collisionSteppedFountain && !model.collisionSteppedLowPlatform) {
        return localMax.z;
    }

    glm::vec2 center((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
    glm::vec2 half((localMax.x - localMin.x) * 0.5f, (localMax.y - localMin.y) * 0.5f);
    if (half.x < 1e-4f || half.y < 1e-4f) {
        return localMax.z;
    }

    float nx = (localPos.x - center.x) / half.x;
    float ny = (localPos.y - center.y) / half.y;
    float r = std::sqrt(nx * nx + ny * ny);

    float h = localMax.z - localMin.z;
    if (model.collisionSteppedFountain) {
        if (r > 0.85f) return localMin.z + h * 0.18f;  // outer lip
        if (r > 0.65f) return localMin.z + h * 0.36f;  // mid step
        if (r > 0.45f) return localMin.z + h * 0.54f;  // inner step
        if (r > 0.28f) return localMin.z + h * 0.70f;  // center platform / statue base
        if (r > 0.14f) return localMin.z + h * 0.84f;  // statue body / sword
        return localMin.z + h * 0.96f;                  // statue head / top
    }

    // Low square curb/planter profile:
    // use edge distance (not radial) so corner blocks don't become too low and
    // clip-through at diagonals.
    float edge = std::max(std::abs(nx), std::abs(ny));
    if (edge > 0.92f) return localMin.z + h * 0.06f;
    if (edge > 0.72f) return localMin.z + h * 0.30f;
    return localMin.z + h * 0.62f;
}

bool segmentIntersectsAABB(const glm::vec3& from, const glm::vec3& to,
                           const glm::vec3& bmin, const glm::vec3& bmax,
                           float& outEnterT) {
    glm::vec3 d = to - from;
    float tEnter = 0.0f;
    float tExit = 1.0f;

    for (int axis = 0; axis < 3; axis++) {
        if (std::abs(d[axis]) < 1e-6f) {
            if (from[axis] < bmin[axis] || from[axis] > bmax[axis]) {
                return false;
            }
            continue;
        }

        float inv = 1.0f / d[axis];
        float t0 = (bmin[axis] - from[axis]) * inv;
        float t1 = (bmax[axis] - from[axis]) * inv;
        if (t0 > t1) std::swap(t0, t1);

        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) return false;
    }

    outEnterT = tEnter;
    return tExit >= 0.0f && tEnter <= 1.0f;
}

void transformAABB(const glm::mat4& modelMatrix,
                   const glm::vec3& localMin,
                   const glm::vec3& localMax,
                   glm::vec3& outMin,
                   glm::vec3& outMax) {
    const glm::vec3 corners[8] = {
        {localMin.x, localMin.y, localMin.z},
        {localMin.x, localMin.y, localMax.z},
        {localMin.x, localMax.y, localMin.z},
        {localMin.x, localMax.y, localMax.z},
        {localMax.x, localMin.y, localMin.z},
        {localMax.x, localMin.y, localMax.z},
        {localMax.x, localMax.y, localMin.z},
        {localMax.x, localMax.y, localMax.z}
    };

    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(-std::numeric_limits<float>::max());
    for (const auto& c : corners) {
        glm::vec3 wc = glm::vec3(modelMatrix * glm::vec4(c, 1.0f));
        outMin = glm::min(outMin, wc);
        outMax = glm::max(outMax, wc);
    }
}

float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax) {
    glm::vec3 q = glm::clamp(p, bmin, bmax);
    glm::vec3 d = p - q;
    return glm::dot(d, d);
}

struct QueryTimer {
    double* totalMs = nullptr;
    uint32_t* callCount = nullptr;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    QueryTimer(double* total, uint32_t* calls) : totalMs(total), callCount(calls) {}
    ~QueryTimer() {
        if (callCount) {
            (*callCount)++;
        }
        if (totalMs) {
            auto end = std::chrono::steady_clock::now();
            *totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }
    }
};

} // namespace

void M2Instance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Rotation in radians
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));
    invModelMatrix = glm::inverse(modelMatrix);
}

M2Renderer::M2Renderer() {
}

M2Renderer::~M2Renderer() {
    shutdown();
}

bool M2Renderer::initialize(pipeline::AssetManager* assets) {
    assetManager = assets;

    LOG_INFO("Initializing M2 renderer...");

    // Create M2 shader with simple animation support
    const char* vertexSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec2 aTexCoord;

        uniform mat4 uModel;
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform float uTime;
        uniform float uAnimScale;  // 0 = no animation, 1 = full animation

        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;

        void main() {
            vec3 pos = aPos;

            // Simple swaying animation for vegetation/doodads
            // Only animate vertices above ground level (positive Y in model space)
            if (uAnimScale > 0.0 && pos.z > 0.5) {
                float sway = sin(uTime * 2.0 + pos.x * 0.5 + pos.y * 0.3) * 0.1;
                float heightFactor = clamp((pos.z - 0.5) / 3.0, 0.0, 1.0);
                pos.x += sway * heightFactor * uAnimScale;
                pos.y += sway * 0.5 * heightFactor * uAnimScale;
            }

            vec4 worldPos = uModel * vec4(pos, 1.0);
            FragPos = worldPos.xyz;
            Normal = mat3(uModel) * aNormal;
            TexCoord = aTexCoord;

            gl_Position = uProjection * uView * worldPos;
        }
    )";

    const char* fragmentSrc = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;

        uniform vec3 uLightDir;
        uniform vec3 uAmbientColor;
        uniform sampler2D uTexture;
        uniform bool uHasTexture;
        uniform bool uAlphaTest;
        uniform float uFadeAlpha;

        out vec4 FragColor;

        void main() {
            vec4 texColor;
            if (uHasTexture) {
                texColor = texture(uTexture, TexCoord);
            } else {
                texColor = vec4(0.6, 0.5, 0.4, 1.0);  // Fallback brownish
            }

            // Alpha test for leaves, fences, etc.
            if (uAlphaTest && texColor.a < 0.5) {
                discard;
            }

            // Distance fade - discard nearly invisible fragments
            float finalAlpha = texColor.a * uFadeAlpha;
            if (finalAlpha < 0.02) {
                discard;
            }

            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Two-sided lighting for foliage
            float diff = max(abs(dot(normal, lightDir)), 0.3);

            vec3 ambient = uAmbientColor * texColor.rgb;
            vec3 diffuse = diff * texColor.rgb;

            vec3 result = ambient + diffuse;
            FragColor = vec4(result, finalAlpha);
        }
    )";

    shader = std::make_unique<Shader>();
    if (!shader->loadFromSource(vertexSrc, fragmentSrc)) {
        LOG_ERROR("Failed to create M2 shader");
        return false;
    }

    // Create white fallback texture
    uint8_t white[] = {255, 255, 255, 255};
    glGenTextures(1, &whiteTexture);
    glBindTexture(GL_TEXTURE_2D, whiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    LOG_INFO("M2 renderer initialized");
    return true;
}

void M2Renderer::shutdown() {
    LOG_INFO("Shutting down M2 renderer...");

    // Delete GPU resources
    for (auto& [id, model] : models) {
        if (model.vao != 0) glDeleteVertexArrays(1, &model.vao);
        if (model.vbo != 0) glDeleteBuffers(1, &model.vbo);
        if (model.ebo != 0) glDeleteBuffers(1, &model.ebo);
    }
    models.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();

    // Delete cached textures
    for (auto& [path, texId] : textureCache) {
        if (texId != 0 && texId != whiteTexture) {
            glDeleteTextures(1, &texId);
        }
    }
    textureCache.clear();
    if (whiteTexture != 0) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }

    shader.reset();
}

bool M2Renderer::loadModel(const pipeline::M2Model& model, uint32_t modelId) {
    if (models.find(modelId) != models.end()) {
        // Already loaded
        return true;
    }

    if (model.vertices.empty() || model.indices.empty()) {
        LOG_WARNING("M2 model has no geometry: ", model.name);
        return false;
    }

    M2ModelGPU gpuModel;
    gpuModel.name = model.name;
    // Use tight bounds from actual vertices for collision/camera occlusion.
    // Header bounds in some M2s are overly conservative.
    glm::vec3 tightMin( std::numeric_limits<float>::max());
    glm::vec3 tightMax(-std::numeric_limits<float>::max());
    for (const auto& v : model.vertices) {
        tightMin = glm::min(tightMin, v.position);
        tightMax = glm::max(tightMax, v.position);
    }
    {
        std::string lowerName = model.name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.collisionSteppedFountain = (lowerName.find("fountain") != std::string::npos);

        glm::vec3 dims = tightMax - tightMin;
        float horiz = std::max(dims.x, dims.y);
        float vert = std::max(0.0f, dims.z);
        bool lowWideShape = (horiz > 1.4f && vert > 0.2f && vert < horiz * 0.70f);
        bool likelyCurbName =
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos);
        bool knownStormwindPlanter =
            (lowerName.find("stormwindplanter") != std::string::npos) ||
            (lowerName.find("stormwindwindowplanter") != std::string::npos);
        bool lowPlatformShape = (horiz > 1.8f && vert > 0.2f && vert < 1.8f);
        gpuModel.collisionSteppedLowPlatform = (!gpuModel.collisionSteppedFountain) &&
                                               (knownStormwindPlanter ||
                                                (likelyCurbName && (lowPlatformShape || lowWideShape)));

        bool isPlanter = (lowerName.find("planter") != std::string::npos);
        gpuModel.collisionPlanter = isPlanter;
        bool statueName =
            (lowerName.find("statue") != std::string::npos) ||
            (lowerName.find("monument") != std::string::npos) ||
            (lowerName.find("sculpture") != std::string::npos);
        gpuModel.collisionStatue = statueName;
        bool smallSolidPropName =
            statueName ||
            (lowerName.find("crate") != std::string::npos) ||
            (lowerName.find("box") != std::string::npos) ||
            (lowerName.find("chest") != std::string::npos) ||
            (lowerName.find("barrel") != std::string::npos);
        bool foliageName =
            (lowerName.find("bush") != std::string::npos) ||
            (lowerName.find("grass") != std::string::npos) ||
            ((lowerName.find("plant") != std::string::npos) && !isPlanter) ||
            (lowerName.find("flower") != std::string::npos) ||
            (lowerName.find("shrub") != std::string::npos) ||
            (lowerName.find("fern") != std::string::npos) ||
            (lowerName.find("vine") != std::string::npos);
        bool canopyLike =
            (lowerName.find("canopy") != std::string::npos) ||
            (lowerName.find("leaf") != std::string::npos) ||
            (lowerName.find("leaves") != std::string::npos);
        bool treeLike = (lowerName.find("tree") != std::string::npos);
        bool hardTreePart =
            (lowerName.find("trunk") != std::string::npos) ||
            (lowerName.find("stump") != std::string::npos) ||
            (lowerName.find("log") != std::string::npos);
        bool softTree = treeLike && !hardTreePart && (canopyLike || vert > horiz * 1.35f);
        bool smallSoftShape = (horiz < 2.2f && vert < 2.4f);
        bool mediumFoliageShape = (horiz < 4.5f && vert < 4.5f);
        bool forceSolidCurb = gpuModel.collisionSteppedLowPlatform || knownStormwindPlanter || likelyCurbName || gpuModel.collisionPlanter;
        bool narrowVerticalName =
            (lowerName.find("lamp") != std::string::npos) ||
            (lowerName.find("lantern") != std::string::npos) ||
            (lowerName.find("post") != std::string::npos) ||
            (lowerName.find("pole") != std::string::npos);
        bool narrowVerticalShape =
            (horiz > 0.12f && horiz < 2.0f && vert > 2.2f && vert > horiz * 1.8f);
        gpuModel.collisionNarrowVerticalProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            (narrowVerticalName || narrowVerticalShape);
        bool genericSolidPropShape =
            (horiz > 0.6f && horiz < 6.0f && vert > 0.30f && vert < 4.0f && vert > horiz * 0.16f) ||
            statueName;
        bool curbLikeName =
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos);
        bool lowPlatformLikeShape = lowWideShape || lowPlatformShape;
        gpuModel.collisionSmallSolidProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            !gpuModel.collisionNarrowVerticalProp &&
            !curbLikeName &&
            !lowPlatformLikeShape &&
            (smallSolidPropName || (genericSolidPropShape && !foliageName && !softTree));
        gpuModel.collisionNoBlock = ((((foliageName && smallSoftShape) || (foliageName && mediumFoliageShape)) || softTree) &&
                                     !forceSolidCurb);
    }
    gpuModel.boundMin = tightMin;
    gpuModel.boundMax = tightMax;
    gpuModel.boundRadius = model.boundRadius;
    gpuModel.indexCount = static_cast<uint32_t>(model.indices.size());
    gpuModel.vertexCount = static_cast<uint32_t>(model.vertices.size());

    // Create VAO
    glGenVertexArrays(1, &gpuModel.vao);
    glBindVertexArray(gpuModel.vao);

    // Create VBO with interleaved vertex data
    // Format: position (3), normal (3), texcoord (2)
    std::vector<float> vertexData;
    vertexData.reserve(model.vertices.size() * 8);

    for (const auto& v : model.vertices) {
        vertexData.push_back(v.position.x);
        vertexData.push_back(v.position.y);
        vertexData.push_back(v.position.z);
        vertexData.push_back(v.normal.x);
        vertexData.push_back(v.normal.y);
        vertexData.push_back(v.normal.z);
        vertexData.push_back(v.texCoords[0].x);
        vertexData.push_back(v.texCoords[0].y);
    }

    glGenBuffers(1, &gpuModel.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, gpuModel.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float),
                 vertexData.data(), GL_STATIC_DRAW);

    // Create EBO
    glGenBuffers(1, &gpuModel.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuModel.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.indices.size() * sizeof(uint16_t),
                 model.indices.data(), GL_STATIC_DRAW);

    // Set up vertex attributes
    const size_t stride = 8 * sizeof(float);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));

    // TexCoord
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));

    glBindVertexArray(0);

    // Load ALL textures from the model into a local vector
    std::vector<GLuint> allTextures;
    if (assetManager) {
        for (const auto& tex : model.textures) {
            if (!tex.filename.empty()) {
                allTextures.push_back(loadTexture(tex.filename));
            } else {
                allTextures.push_back(whiteTexture);
            }
        }
    }

    // Build per-batch GPU entries
    if (!model.batches.empty()) {
        for (const auto& batch : model.batches) {
            M2ModelGPU::BatchGPU bgpu;
            bgpu.indexStart = batch.indexStart;
            bgpu.indexCount = batch.indexCount;

            // Resolve texture: batch.textureIndex → textureLookup → allTextures
            GLuint tex = whiteTexture;
            if (batch.textureIndex < model.textureLookup.size()) {
                uint16_t texIdx = model.textureLookup[batch.textureIndex];
                if (texIdx < allTextures.size()) {
                    tex = allTextures[texIdx];
                }
            } else if (!allTextures.empty()) {
                tex = allTextures[0];
            }
            bgpu.texture = tex;
            bgpu.hasAlpha = (tex != 0 && tex != whiteTexture);
            gpuModel.batches.push_back(bgpu);
        }
    } else {
        // Fallback: single batch covering all indices with first texture
        M2ModelGPU::BatchGPU bgpu;
        bgpu.indexStart = 0;
        bgpu.indexCount = gpuModel.indexCount;
        bgpu.texture = allTextures.empty() ? whiteTexture : allTextures[0];
        bgpu.hasAlpha = (bgpu.texture != 0 && bgpu.texture != whiteTexture);
        gpuModel.batches.push_back(bgpu);
    }

    models[modelId] = std::move(gpuModel);

    LOG_DEBUG("Loaded M2 model: ", model.name, " (", models[modelId].vertexCount, " vertices, ",
              models[modelId].indexCount / 3, " triangles, ", models[modelId].batches.size(), " batches)");

    return true;
}

uint32_t M2Renderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

uint32_t M2Renderer::createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                                const glm::vec3& position) {
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;  // Used for frustum culling
    instance.rotation = glm::vec3(0.0f);
    instance.scale = 1.0f;
    instance.modelMatrix = modelMatrix;
    instance.invModelMatrix = glm::inverse(modelMatrix);
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);
    instance.animTime = static_cast<float>(rand()) / RAND_MAX * 10.0f;  // Random start time

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

void M2Renderer::update(float deltaTime) {
    // Advance animation time for all instances
    for (auto& instance : instances) {
        instance.animTime += deltaTime * instance.animSpeed;
    }
}

void M2Renderer::render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection) {
    (void)camera;  // unused for now

    if (instances.empty() || !shader) {
        return;
    }

    // Debug: log once when we start rendering
    static bool loggedOnce = false;
    if (!loggedOnce) {
        loggedOnce = true;
        LOG_INFO("M2 render: ", instances.size(), " instances, ", models.size(), " models");
    }

    // Set up GL state for M2 rendering
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);   // Some M2 geometry is single-sided

    // Build frustum for culling
    Frustum frustum;
    frustum.extractFromMatrix(projection * view);

    shader->use();
    shader->setUniform("uView", view);
    shader->setUniform("uProjection", projection);
    shader->setUniform("uLightDir", lightDir);
    shader->setUniform("uAmbientColor", ambientColor);

    lastDrawCallCount = 0;

    // Distance-based culling threshold for M2 models
    const float maxRenderDistance = 180.0f;  // Aggressive culling for city performance
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;
    const float fadeStartFraction = 0.75f;   // Start fading at 75% of max distance
    const glm::vec3 camPos = camera.getPosition();

    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (!model.isValid()) continue;

        // Distance culling for small objects (scaled by object size)
        glm::vec3 toCam = instance.position - camPos;
        float distSq = glm::dot(toCam, toCam);
        float worldRadius = model.boundRadius * instance.scale;
        // Cull small objects (radius < 20) at distance, keep larger objects visible longer
        float effectiveMaxDistSq = maxRenderDistanceSq * std::max(1.0f, worldRadius / 12.0f);
        if (worldRadius < 0.8f) {
            effectiveMaxDistSq = std::min(effectiveMaxDistSq, 65.0f * 65.0f);
        } else if (worldRadius < 1.5f) {
            effectiveMaxDistSq = std::min(effectiveMaxDistSq, 95.0f * 95.0f);
        }
        if (distSq > effectiveMaxDistSq) {
            continue;
        }

        // Frustum cull: test bounding sphere in world space
        if (worldRadius > 0.0f && !frustum.intersectsSphere(instance.position, worldRadius)) {
            continue;
        }

        // Distance-based fade alpha for smooth pop-in
        float fadeAlpha = 1.0f;
        float fadeStartDistSq = effectiveMaxDistSq * fadeStartFraction * fadeStartFraction;
        if (distSq > fadeStartDistSq) {
            float dist = std::sqrt(distSq);
            float effectiveMaxDist = std::sqrt(effectiveMaxDistSq);
            float fadeStartDist = effectiveMaxDist * fadeStartFraction;
            fadeAlpha = std::clamp((effectiveMaxDist - dist) / (effectiveMaxDist - fadeStartDist), 0.0f, 1.0f);
        }

        shader->setUniform("uModel", instance.modelMatrix);
        shader->setUniform("uTime", instance.animTime);
        shader->setUniform("uAnimScale", 0.0f);  // Disabled - proper M2 animation needs bone/particle systems
        shader->setUniform("uFadeAlpha", fadeAlpha);

        // Disable depth writes for fading objects to avoid z-fighting
        if (fadeAlpha < 1.0f) {
            glDepthMask(GL_FALSE);
        }

        glBindVertexArray(model.vao);

        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;

            bool hasTexture = (batch.texture != 0);
            shader->setUniform("uHasTexture", hasTexture);
            shader->setUniform("uAlphaTest", batch.hasAlpha);

            if (hasTexture) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, batch.texture);
                shader->setUniform("uTexture", 0);
            }

            glDrawElements(GL_TRIANGLES, batch.indexCount, GL_UNSIGNED_SHORT,
                           (void*)(batch.indexStart * sizeof(uint16_t)));

            lastDrawCallCount++;
        }

        glBindVertexArray(0);

        if (fadeAlpha < 1.0f) {
            glDepthMask(GL_TRUE);
        }
    }

    // Restore state
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void M2Renderer::removeInstance(uint32_t instanceId) {
    for (auto it = instances.begin(); it != instances.end(); ++it) {
        if (it->id == instanceId) {
            instances.erase(it);
            rebuildSpatialIndex();
            return;
        }
    }
}

void M2Renderer::clear() {
    for (auto& [id, model] : models) {
        if (model.vao != 0) glDeleteVertexArrays(1, &model.vao);
        if (model.vbo != 0) glDeleteBuffers(1, &model.vbo);
        if (model.ebo != 0) glDeleteBuffers(1, &model.ebo);
    }
    models.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
}

void M2Renderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocusEnabled = (radius > 0.0f);
    collisionFocusPos = worldPos;
    collisionFocusRadius = std::max(0.0f, radius);
    collisionFocusRadiusSq = collisionFocusRadius * collisionFocusRadius;
}

void M2Renderer::clearCollisionFocus() {
    collisionFocusEnabled = false;
}

void M2Renderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
}

M2Renderer::GridCell M2Renderer::toCell(const glm::vec3& p) const {
    return GridCell{
        static_cast<int>(std::floor(p.x / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.y / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.z / SPATIAL_CELL_SIZE))
    };
}

void M2Renderer::rebuildSpatialIndex() {
    spatialGrid.clear();
    instanceIndexById.clear();
    instanceIndexById.reserve(instances.size());

    for (size_t i = 0; i < instances.size(); i++) {
        const auto& inst = instances[i];
        instanceIndexById[inst.id] = i;

        GridCell minCell = toCell(inst.worldBoundsMin);
        GridCell maxCell = toCell(inst.worldBoundsMax);
        for (int z = minCell.z; z <= maxCell.z; z++) {
            for (int y = minCell.y; y <= maxCell.y; y++) {
                for (int x = minCell.x; x <= maxCell.x; x++) {
                    spatialGrid[GridCell{x, y, z}].push_back(inst.id);
                }
            }
        }
    }
}

void M2Renderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
                                  std::vector<size_t>& outIndices) const {
    outIndices.clear();
    candidateIdScratch.clear();

    GridCell minCell = toCell(queryMin);
    GridCell maxCell = toCell(queryMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                auto it = spatialGrid.find(GridCell{x, y, z});
                if (it == spatialGrid.end()) continue;
                for (uint32_t id : it->second) {
                    if (!candidateIdScratch.insert(id).second) continue;
                    auto idxIt = instanceIndexById.find(id);
                    if (idxIt != instanceIndexById.end()) {
                        outIndices.push_back(idxIt->second);
                    }
                }
            }
        }
    }

    // Safety fallback to preserve collision correctness if the spatial index
    // misses candidates (e.g. during streaming churn).
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
}

void M2Renderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    // Find and remove models with no instances
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : models) {
        if (usedModelIds.find(id) == usedModelIds.end()) {
            toRemove.push_back(id);
        }
    }

    // Delete GPU resources and remove from map
    for (uint32_t id : toRemove) {
        auto it = models.find(id);
        if (it != models.end()) {
            if (it->second.vao != 0) glDeleteVertexArrays(1, &it->second.vao);
            if (it->second.vbo != 0) glDeleteBuffers(1, &it->second.vbo);
            if (it->second.ebo != 0) glDeleteBuffers(1, &it->second.ebo);
            models.erase(it);
        }
    }

    if (!toRemove.empty()) {
        LOG_INFO("M2 cleanup: removed ", toRemove.size(), " unused models, ", models.size(), " remaining");
    }
}

GLuint M2Renderer::loadTexture(const std::string& path) {
    // Check cache
    auto it = textureCache.find(path);
    if (it != textureCache.end()) {
        return it->second;
    }

    // Load BLP texture
    pipeline::BLPImage blp = assetManager->loadTexture(path);
    if (!blp.isValid()) {
        LOG_WARNING("M2: Failed to load texture: ", path);
        textureCache[path] = whiteTexture;
        return whiteTexture;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 blp.width, blp.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blp.data.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    textureCache[path] = textureID;
    LOG_DEBUG("M2: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return textureID;
}

uint32_t M2Renderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it != models.end()) {
            total += it->second.indexCount / 3;
        }
    }
    return total;
}

std::optional<float> M2Renderer::getFloorHeight(float glX, float glY, float glZ) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;

    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 6.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 8.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - 2.0f || glZ > instance.worldBoundsMax.z + 2.0f) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        if (instance.scale <= 0.001f) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Must be within doodad footprint in local XY.
        // Stepped low platforms get a small pad so walk-up snapping catches edges.
        float footprintPad = 0.0f;
        if (model.collisionSteppedLowPlatform) {
            footprintPad = model.collisionPlanter ? 0.22f : 0.16f;
        }
        if (localPos.x < localMin.x - footprintPad || localPos.x > localMax.x + footprintPad ||
            localPos.y < localMin.y - footprintPad || localPos.y > localMax.y + footprintPad) {
            continue;
        }

        // Construct "top" point at queried XY in local space, then transform back.
        float localTopZ = getEffectiveCollisionTopLocal(model, localPos, localMin, localMax);
        glm::vec3 localTop(localPos.x, localPos.y, localTopZ);
        glm::vec3 worldTop = glm::vec3(instance.modelMatrix * glm::vec4(localTop, 1.0f));

        // Reachability filter: allow a bit more climb for stepped low platforms.
        float maxStepUp = 1.0f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            maxStepUp = 2.0f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 3.0f : 2.4f;
        }
        if (worldTop.z > glZ + maxStepUp) continue;

        if (!bestFloor || worldTop.z > *bestFloor) {
            bestFloor = worldTop.z;
        }
    }

    return bestFloor;
}

bool M2Renderer::checkCollision(const glm::vec3& from, const glm::vec3& to,
                                 glm::vec3& adjustedPos, float playerRadius) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool collided = false;

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(7.0f, 7.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(7.0f, 7.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    // Check against all M2 instances in local space (rotation-aware).
    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        const float broadMargin = playerRadius + 1.0f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && adjustedPos.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && adjustedPos.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && adjustedPos.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && adjustedPos.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + 2.5f && adjustedPos.z > instance.worldBoundsMax.z + 2.5f) continue;
        if (from.z + 2.5f < instance.worldBoundsMin.z && adjustedPos.z + 2.5f < instance.worldBoundsMin.z) continue;

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        if (instance.scale <= 0.001f) continue;

        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
        float radiusScale = model.collisionNarrowVerticalProp ? 0.45f : 1.0f;
        float localRadius = (playerRadius * radiusScale) / instance.scale;

        glm::vec3 rawMin, rawMax;
        getTightCollisionBounds(model, rawMin, rawMax);
        glm::vec3 localMin = rawMin - glm::vec3(localRadius);
        glm::vec3 localMax = rawMax + glm::vec3(localRadius);
        float effectiveTop = getEffectiveCollisionTopLocal(model, localPos, rawMin, rawMax) + localRadius;
        glm::vec2 localCenter((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
        float fromR = glm::length(glm::vec2(localFrom.x, localFrom.y) - localCenter);
        float toR = glm::length(glm::vec2(localPos.x, localPos.y) - localCenter);

        // Feet-based vertical overlap test: ignore objects fully above/below us.
        constexpr float PLAYER_HEIGHT = 2.0f;
        if (localPos.z + PLAYER_HEIGHT < localMin.z || localPos.z > effectiveTop) {
            continue;
        }

        bool fromInsideXY =
            (localFrom.x >= localMin.x && localFrom.x <= localMax.x &&
             localFrom.y >= localMin.y && localFrom.y <= localMax.y);
        bool fromInsideZ = (localFrom.z + PLAYER_HEIGHT >= localMin.z && localFrom.z <= effectiveTop);
        bool escapingOverlap = (fromInsideXY && fromInsideZ && (toR > fromR + 1e-4f));
        bool allowEscapeRelax = escapingOverlap && !model.collisionSmallSolidProp;

        // Swept hard clamp for taller blockers only.
        // Low/stepable objects should be climbable and not "shove" the player off.
        float maxStepUp = 1.20f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            // Keep box/crate-class props hard-solid to prevent phase-through.
            maxStepUp = 0.75f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 2.8f : 2.4f;
        }
        bool stepableLowObject = (effectiveTop <= localFrom.z + maxStepUp);
        bool climbingAttempt = (localPos.z > localFrom.z + 0.18f);
        bool nearTop = (localFrom.z >= effectiveTop - 0.30f);
        float climbAllowance = model.collisionPlanter ? 0.95f : 0.60f;
        if (model.collisionSteppedLowPlatform && !model.collisionPlanter) {
            // Let low curb/planter blocks be stepable without sticky side shoves.
            climbAllowance = 1.00f;
        }
        if (model.collisionSmallSolidProp) {
            climbAllowance = 1.05f;
        }
        bool climbingTowardTop = climbingAttempt && (localFrom.z + climbAllowance >= effectiveTop);
        bool forceHardLateral =
            model.collisionSmallSolidProp &&
            !nearTop && !climbingTowardTop;
        if ((!stepableLowObject || forceHardLateral) && !allowEscapeRelax) {
            float tEnter = 0.0f;
            glm::vec3 sweepMax = localMax;
            sweepMax.z = std::min(sweepMax.z, effectiveTop);
            if (segmentIntersectsAABB(localFrom, localPos, localMin, sweepMax, tEnter)) {
                float tSafe = std::clamp(tEnter - 0.03f, 0.0f, 1.0f);
                glm::vec3 localSafe = localFrom + (localPos - localFrom) * tSafe;
                glm::vec3 worldSafe = glm::vec3(instance.modelMatrix * glm::vec4(localSafe, 1.0f));
                adjustedPos.x = worldSafe.x;
                adjustedPos.y = worldSafe.y;
                collided = true;
                continue;
            }
        }

        if (localPos.x < localMin.x || localPos.x > localMax.x ||
            localPos.y < localMin.y || localPos.y > localMax.y) {
            continue;
        }

        float pushLeft  = localPos.x - localMin.x;
        float pushRight = localMax.x - localPos.x;
        float pushBack  = localPos.y - localMin.y;
        float pushFront = localMax.y - localPos.y;

        float minPush = std::min({pushLeft, pushRight, pushBack, pushFront});
        if (allowEscapeRelax) {
            continue;
        }
        if ((model.collisionSteppedLowPlatform || model.collisionSteppedFountain) && stepableLowObject) {
            // Already on/near top surface: don't apply lateral push that ejects
            // the player from the object when landing.
            continue;
        }
        // Gentle fallback push for overlapping cases.
        float pushAmount;
        if (model.collisionNarrowVerticalProp) {
            pushAmount = std::clamp(minPush * 0.10f, 0.001f, 0.010f);
        } else if (model.collisionSteppedLowPlatform) {
            if (model.collisionPlanter && stepableLowObject) {
                pushAmount = std::clamp(minPush * 0.06f, 0.001f, 0.006f);
            } else {
            pushAmount = std::clamp(minPush * 0.12f, 0.003f, 0.012f);
            }
        } else if (stepableLowObject) {
            pushAmount = std::clamp(minPush * 0.12f, 0.002f, 0.015f);
        } else {
            pushAmount = std::clamp(minPush * 0.28f, 0.010f, 0.045f);
        }
        glm::vec3 localPush(0.0f);
        if (minPush == pushLeft) {
            localPush.x = -pushAmount;
        } else if (minPush == pushRight) {
            localPush.x = pushAmount;
        } else if (minPush == pushBack) {
            localPush.y = -pushAmount;
        } else {
            localPush.y = pushAmount;
        }

        glm::vec3 worldPush = glm::vec3(instance.modelMatrix * glm::vec4(localPush, 0.0f));
        adjustedPos.x += worldPush.x;
        adjustedPos.y += worldPush.y;
        collided = true;
    }

    return collided;
}

float M2Renderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;

    glm::vec3 rayEnd = origin + direction * maxDistance;
    glm::vec3 queryMin = glm::min(origin, rayEnd) - glm::vec3(1.0f);
    glm::vec3 queryMax = glm::max(origin, rayEnd) + glm::vec3(1.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        // Cheap world-space broad-phase.
        float tEnter = 0.0f;
        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.35f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.35f);
        if (!segmentIntersectsAABB(origin, origin + direction * maxDistance, worldMin, worldMax, tEnter)) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);
        // Skip tiny doodads for camera occlusion; they cause jitter and false hits.
        glm::vec3 extents = (localMax - localMin) * instance.scale;
        if (glm::length(extents) < 0.75f) continue;

        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));
        if (!std::isfinite(localDir.x) || !std::isfinite(localDir.y) || !std::isfinite(localDir.z)) {
            continue;
        }

        // Local-space AABB slab intersection.
        glm::vec3 invDir = 1.0f / localDir;
        glm::vec3 tMin = (localMin - localOrigin) * invDir;
        glm::vec3 tMax = (localMax - localOrigin) * invDir;
        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = std::max({t1.x, t1.y, t1.z});
        float tFar = std::min({t2.x, t2.y, t2.z});
        if (tNear > tFar || tFar <= 0.0f) continue;

        float tHit = tNear > 0.0f ? tNear : tFar;
        glm::vec3 localHit = localOrigin + localDir * tHit;
        glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
        float worldDist = glm::length(worldHit - origin);
        if (worldDist > 0.0f && worldDist < closestHit) {
            closestHit = worldDist;
        }
    }

    return closestHit;
}

} // namespace rendering
} // namespace wowee
