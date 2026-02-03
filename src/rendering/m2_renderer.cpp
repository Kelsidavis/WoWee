#include "rendering/m2_renderer.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_set>
#include <algorithm>

namespace wowee {
namespace rendering {

void M2Instance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Rotation in radians
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));
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

            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Two-sided lighting for foliage
            float diff = max(abs(dot(normal, lightDir)), 0.3);

            vec3 ambient = uAmbientColor * texColor.rgb;
            vec3 diffuse = diff * texColor.rgb;

            vec3 result = ambient + diffuse;
            FragColor = vec4(result, texColor.a);
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
    gpuModel.boundMin = model.boundMin;
    gpuModel.boundMax = model.boundMax;
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

    instances.push_back(instance);

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
    instance.animTime = static_cast<float>(rand()) / RAND_MAX * 10.0f;  // Random start time

    instances.push_back(instance);

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
    glDisable(GL_BLEND);       // No blend leaking from prior renderers
    glDisable(GL_CULL_FACE);   // Some M2 geometry is single-sided

    // Make models render with a bright color for debugging
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);  // Wireframe mode

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
    const float maxRenderDistance = 400.0f;  // Balance between performance and visibility
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;
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
        float effectiveMaxDistSq = maxRenderDistanceSq * std::max(1.0f, worldRadius / 10.0f);
        if (distSq > effectiveMaxDistSq) {
            continue;
        }

        // Frustum cull: test bounding sphere in world space
        if (worldRadius > 0.0f && !frustum.intersectsSphere(instance.position, worldRadius)) {
            continue;
        }

        shader->setUniform("uModel", instance.modelMatrix);
        shader->setUniform("uTime", instance.animTime);
        shader->setUniform("uAnimScale", 0.0f);  // Disabled - proper M2 animation needs bone/particle systems

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

        // Check for GL errors (only first draw)
        static bool checkedOnce = false;
        if (!checkedOnce) {
            checkedOnce = true;
            GLenum err = glGetError();
            if (err != GL_NO_ERROR) {
                LOG_ERROR("GL error after M2 draw: ", err);
            } else {
                LOG_INFO("M2 draw successful: ", model.indexCount, " indices");
            }
        }

        glBindVertexArray(0);
    }

    // Restore cull face state
    glEnable(GL_CULL_FACE);
}

void M2Renderer::removeInstance(uint32_t instanceId) {
    for (auto it = instances.begin(); it != instances.end(); ++it) {
        if (it->id == instanceId) {
            instances.erase(it);
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

bool M2Renderer::checkCollision(const glm::vec3& from, const glm::vec3& to,
                                 glm::vec3& adjustedPos, float playerRadius) const {
    adjustedPos = to;
    bool collided = false;

    // Check against all M2 instances using their bounding boxes
    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;

        // Transform model bounds to world space (approximate with scaled AABB)
        glm::vec3 worldMin = instance.position + model.boundMin * instance.scale;
        glm::vec3 worldMax = instance.position + model.boundMax * instance.scale;

        // Ensure min/max are correct
        glm::vec3 actualMin = glm::min(worldMin, worldMax);
        glm::vec3 actualMax = glm::max(worldMin, worldMax);

        // Expand bounds by player radius
        actualMin -= glm::vec3(playerRadius);
        actualMax += glm::vec3(playerRadius);

        // Check if player position is inside expanded bounds (XY only for walking)
        if (adjustedPos.x >= actualMin.x && adjustedPos.x <= actualMax.x &&
            adjustedPos.y >= actualMin.y && adjustedPos.y <= actualMax.y &&
            adjustedPos.z >= actualMin.z && adjustedPos.z <= actualMax.z) {

            // Push player out of the object
            // Find the shortest push direction (XY only)
            float pushLeft = adjustedPos.x - actualMin.x;
            float pushRight = actualMax.x - adjustedPos.x;
            float pushBack = adjustedPos.y - actualMin.y;
            float pushFront = actualMax.y - adjustedPos.y;

            float minPush = std::min({pushLeft, pushRight, pushBack, pushFront});

            if (minPush == pushLeft) {
                adjustedPos.x = actualMin.x - 0.01f;
            } else if (minPush == pushRight) {
                adjustedPos.x = actualMax.x + 0.01f;
            } else if (minPush == pushBack) {
                adjustedPos.y = actualMin.y - 0.01f;
            } else {
                adjustedPos.y = actualMax.y + 0.01f;
            }

            collided = true;
        }
    }

    return collided;
}

float M2Renderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    float closestHit = maxDistance;

    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;

        // Transform model bounds to world space (approximate with scaled AABB)
        glm::vec3 worldMin = instance.position + model.boundMin * instance.scale;
        glm::vec3 worldMax = instance.position + model.boundMax * instance.scale;

        // Ensure min/max are correct
        glm::vec3 actualMin = glm::min(worldMin, worldMax);
        glm::vec3 actualMax = glm::max(worldMin, worldMax);

        // Ray-AABB intersection (slab method)
        glm::vec3 invDir = 1.0f / direction;
        glm::vec3 tMin = (actualMin - origin) * invDir;
        glm::vec3 tMax = (actualMax - origin) * invDir;

        // Handle negative direction components
        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = std::max({t1.x, t1.y, t1.z});
        float tFar = std::min({t2.x, t2.y, t2.z});

        // Check if ray intersects the box
        if (tNear <= tFar && tFar > 0.0f) {
            float hitDist = tNear > 0.0f ? tNear : tFar;
            if (hitDist > 0.0f && hitDist < closestHit) {
                closestHit = hitDist;
            }
        }
    }

    return closestHit;
}

} // namespace rendering
} // namespace wowee
