#include "rendering/wmo_renderer.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <unordered_set>

namespace wowee {
namespace rendering {

WMORenderer::WMORenderer() {
}

WMORenderer::~WMORenderer() {
    shutdown();
}

bool WMORenderer::initialize(pipeline::AssetManager* assets) {
    core::Logger::getInstance().info("Initializing WMO renderer...");

    assetManager = assets;

    // Create WMO shader with texture support
    const char* vertexSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec2 aTexCoord;
        layout (location = 3) in vec4 aColor;

        uniform mat4 uModel;
        uniform mat4 uView;
        uniform mat4 uProjection;

        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;
        out vec4 VertexColor;

        void main() {
            vec4 worldPos = uModel * vec4(aPos, 1.0);
            FragPos = worldPos.xyz;
            // Use mat3(uModel) directly - avoids expensive inverse() per vertex
            // This works correctly for uniform scale transforms
            Normal = mat3(uModel) * aNormal;
            TexCoord = aTexCoord;
            VertexColor = aColor;

            gl_Position = uProjection * uView * worldPos;
        }
    )";

    const char* fragmentSrc = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;
        in vec4 VertexColor;

        uniform vec3 uLightDir;
        uniform vec3 uViewPos;
        uniform vec3 uAmbientColor;
        uniform sampler2D uTexture;
        uniform bool uHasTexture;
        uniform bool uAlphaTest;

        out vec4 FragColor;

        void main() {
            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Diffuse lighting
            float diff = max(dot(normal, lightDir), 0.0);
            vec3 diffuse = diff * vec3(1.0);

            // Ambient
            vec3 ambient = uAmbientColor;

            // Sample texture or use vertex color
            vec4 texColor;
            if (uHasTexture) {
                texColor = texture(uTexture, TexCoord);
                // Alpha test only for cutout materials (lattice, grating, etc.)
                if (uAlphaTest && texColor.a < 0.5) discard;
            } else {
                // MOCV vertex color alpha is a lighting blend factor, not transparency
                texColor = vec4(VertexColor.rgb, 1.0);
            }

            // Combine lighting with texture
            vec3 result = (ambient + diffuse) * texColor.rgb;
            FragColor = vec4(result, 1.0);
        }
    )";

    shader = std::make_unique<Shader>();
    if (!shader->loadFromSource(vertexSrc, fragmentSrc)) {
        core::Logger::getInstance().error("Failed to create WMO shader");
        return false;
    }

    // Create default white texture for fallback
    uint8_t whitePixel[4] = {255, 255, 255, 255};
    glGenTextures(1, &whiteTexture);
    glBindTexture(GL_TEXTURE_2D, whiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, whitePixel);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    core::Logger::getInstance().info("WMO renderer initialized");
    return true;
}

void WMORenderer::shutdown() {
    core::Logger::getInstance().info("Shutting down WMO renderer...");

    // Free all GPU resources
    for (auto& [id, model] : loadedModels) {
        for (auto& group : model.groups) {
            if (group.vao != 0) glDeleteVertexArrays(1, &group.vao);
            if (group.vbo != 0) glDeleteBuffers(1, &group.vbo);
            if (group.ebo != 0) glDeleteBuffers(1, &group.ebo);
        }
    }

    // Free cached textures
    for (auto& [path, texId] : textureCache) {
        if (texId != 0 && texId != whiteTexture) {
            glDeleteTextures(1, &texId);
        }
    }
    textureCache.clear();

    // Free white texture
    if (whiteTexture != 0) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }

    loadedModels.clear();
    instances.clear();
    shader.reset();
}

bool WMORenderer::loadModel(const pipeline::WMOModel& model, uint32_t id) {
    if (!model.isValid()) {
        core::Logger::getInstance().error("Cannot load invalid WMO model");
        return false;
    }

    // Check if already loaded
    if (loadedModels.find(id) != loadedModels.end()) {
        core::Logger::getInstance().warning("WMO model ", id, " already loaded");
        return true;
    }

    core::Logger::getInstance().info("Loading WMO model ", id, " with ", model.groups.size(), " groups, ",
                                     model.textures.size(), " textures...");

    ModelData modelData;
    modelData.id = id;
    modelData.boundingBoxMin = model.boundingBoxMin;
    modelData.boundingBoxMax = model.boundingBoxMax;

    core::Logger::getInstance().info("  WMO bounds: min=(", model.boundingBoxMin.x, ", ", model.boundingBoxMin.y, ", ", model.boundingBoxMin.z,
                                     ") max=(", model.boundingBoxMax.x, ", ", model.boundingBoxMax.y, ", ", model.boundingBoxMax.z, ")");

    // Load textures for this model
    core::Logger::getInstance().info("  WMO has ", model.textures.size(), " texture paths, ", model.materials.size(), " materials");
    if (assetManager && !model.textures.empty()) {
        for (size_t i = 0; i < model.textures.size(); i++) {
            const auto& texPath = model.textures[i];
            core::Logger::getInstance().debug("    Loading texture ", i, ": ", texPath);
            GLuint texId = loadTexture(texPath);
            modelData.textures.push_back(texId);
        }
        core::Logger::getInstance().info("  Loaded ", modelData.textures.size(), " textures for WMO");
    }

    // Store material -> texture index mapping
    // IMPORTANT: mat.texture1 is a byte offset into MOTX, not an array index!
    // We need to convert it using the textureOffsetToIndex map
    core::Logger::getInstance().info("  textureOffsetToIndex map has ", model.textureOffsetToIndex.size(), " entries");
    static int matLogCount = 0;
    for (size_t i = 0; i < model.materials.size(); i++) {
        const auto& mat = model.materials[i];
        uint32_t texIndex = 0;  // Default to first texture

        auto it = model.textureOffsetToIndex.find(mat.texture1);
        if (it != model.textureOffsetToIndex.end()) {
            texIndex = it->second;
            if (matLogCount < 20) {
                core::Logger::getInstance().info("  Material ", i, ": texture1 offset ", mat.texture1, " -> texture index ", texIndex);
                matLogCount++;
            }
        } else if (mat.texture1 < model.textures.size()) {
            // Fallback: maybe it IS an index in some files?
            texIndex = mat.texture1;
            if (matLogCount < 20) {
                core::Logger::getInstance().info("  Material ", i, ": using texture1 as direct index: ", texIndex);
                matLogCount++;
            }
        } else {
            if (matLogCount < 20) {
                core::Logger::getInstance().info("  Material ", i, ": texture1 offset ", mat.texture1, " NOT FOUND, using default");
                matLogCount++;
            }
        }

        modelData.materialTextureIndices.push_back(texIndex);
        modelData.materialBlendModes.push_back(mat.blendMode);
    }

    // Create GPU resources for each group
    uint32_t loadedGroups = 0;
    for (const auto& wmoGroup : model.groups) {
        // Skip empty groups
        if (wmoGroup.vertices.empty() || wmoGroup.indices.empty()) {
            continue;
        }

        GroupResources resources;
        if (createGroupResources(wmoGroup, resources)) {
            modelData.groups.push_back(resources);
            loadedGroups++;
        }
    }

    if (loadedGroups == 0) {
        core::Logger::getInstance().warning("No valid groups loaded for WMO ", id);
        return false;
    }

    loadedModels[id] = std::move(modelData);
    core::Logger::getInstance().info("WMO model ", id, " loaded successfully (", loadedGroups, " groups)");
    return true;
}

void WMORenderer::unloadModel(uint32_t id) {
    auto it = loadedModels.find(id);
    if (it == loadedModels.end()) {
        return;
    }

    // Free GPU resources
    for (auto& group : it->second.groups) {
        if (group.vao != 0) glDeleteVertexArrays(1, &group.vao);
        if (group.vbo != 0) glDeleteBuffers(1, &group.vbo);
        if (group.ebo != 0) glDeleteBuffers(1, &group.ebo);
    }

    loadedModels.erase(it);
    core::Logger::getInstance().info("WMO model ", id, " unloaded");
}

void WMORenderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    // Find and remove models with no instances
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : loadedModels) {
        if (usedModelIds.find(id) == usedModelIds.end()) {
            toRemove.push_back(id);
        }
    }

    // Delete GPU resources and remove from map
    for (uint32_t id : toRemove) {
        unloadModel(id);
    }

    if (!toRemove.empty()) {
        core::Logger::getInstance().info("WMO cleanup: removed ", toRemove.size(), " unused models, ", loadedModels.size(), " remaining");
    }
}

uint32_t WMORenderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
    // Check if model is loaded
    if (loadedModels.find(modelId) == loadedModels.end()) {
        core::Logger::getInstance().error("Cannot create instance of unloaded WMO model ", modelId);
        return 0;
    }

    WMOInstance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();

    instances.push_back(instance);
    core::Logger::getInstance().info("Created WMO instance ", instance.id, " (model ", modelId, ")");
    return instance.id;
}

void WMORenderer::removeInstance(uint32_t instanceId) {
    auto it = std::find_if(instances.begin(), instances.end(),
                          [instanceId](const WMOInstance& inst) { return inst.id == instanceId; });
    if (it != instances.end()) {
        instances.erase(it);
        core::Logger::getInstance().info("Removed WMO instance ", instanceId);
    }
}

void WMORenderer::clearInstances() {
    instances.clear();
    core::Logger::getInstance().info("Cleared all WMO instances");
}

void WMORenderer::render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection) {
    if (!shader || instances.empty()) {
        lastDrawCalls = 0;
        return;
    }

    lastDrawCalls = 0;

    // Set shader uniforms
    shader->use();
    shader->setUniform("uView", view);
    shader->setUniform("uProjection", projection);
    shader->setUniform("uViewPos", camera.getPosition());
    shader->setUniform("uLightDir", glm::vec3(-0.3f, -0.7f, -0.6f));  // Default sun direction
    shader->setUniform("uAmbientColor", glm::vec3(0.4f, 0.4f, 0.5f));

    // Enable wireframe if requested
    if (wireframeMode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    // WMOs are opaque — ensure blending is off (alpha test via discard in shader)
    glDisable(GL_BLEND);

    // Disable backface culling for WMOs (some faces may have wrong winding)
    glDisable(GL_CULL_FACE);

    // Extract frustum planes for proper culling
    Frustum frustum;
    frustum.extractFromMatrix(projection * view);

    // Render all instances with instance-level culling
    const glm::vec3 camPos = camera.getPosition();
    const float maxRenderDistance = 500.0f;  // Reduced for performance
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;

    for (const auto& instance : instances) {
        // Instance-level distance culling
        glm::vec3 toCam = instance.position - camPos;
        float distSq = glm::dot(toCam, toCam);
        if (distSq > maxRenderDistanceSq) {
            continue;  // Skip instances that are too far
        }

        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) {
            continue;
        }

        const ModelData& model = modelIt->second;
        shader->setUniform("uModel", instance.modelMatrix);

        // Render all groups
        for (const auto& group : model.groups) {
            // Proper frustum culling using AABB test
            if (frustumCulling) {
                // Transform group bounding box to world space
                glm::vec3 worldMin = glm::vec3(instance.modelMatrix * glm::vec4(group.boundingBoxMin, 1.0f));
                glm::vec3 worldMax = glm::vec3(instance.modelMatrix * glm::vec4(group.boundingBoxMax, 1.0f));
                // Ensure min/max are correct after transform (rotation can swap them)
                glm::vec3 actualMin = glm::min(worldMin, worldMax);
                glm::vec3 actualMax = glm::max(worldMin, worldMax);
                if (!frustum.intersectsAABB(actualMin, actualMax)) {
                    continue;
                }
            }

            renderGroup(group, model, instance.modelMatrix, view, projection);
        }
    }

    // Restore polygon mode
    if (wireframeMode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Re-enable backface culling
    glEnable(GL_CULL_FACE);
}

uint32_t WMORenderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt != loadedModels.end()) {
            total += modelIt->second.getTotalTriangles();
        }
    }
    return total;
}

bool WMORenderer::createGroupResources(const pipeline::WMOGroup& group, GroupResources& resources) {
    if (group.vertices.empty() || group.indices.empty()) {
        return false;
    }

    resources.vertexCount = group.vertices.size();
    resources.indexCount = group.indices.size();
    resources.boundingBoxMin = group.boundingBoxMin;
    resources.boundingBoxMax = group.boundingBoxMax;

    // Create vertex data (position, normal, texcoord, color)
    struct VertexData {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec4 color;
    };

    std::vector<VertexData> vertices;
    vertices.reserve(group.vertices.size());

    for (const auto& v : group.vertices) {
        VertexData vd;
        vd.position = v.position;
        vd.normal = v.normal;
        vd.texCoord = v.texCoord;
        vd.color = v.color;
        vertices.push_back(vd);
    }

    // Create VAO/VBO/EBO
    glGenVertexArrays(1, &resources.vao);
    glGenBuffers(1, &resources.vbo);
    glGenBuffers(1, &resources.ebo);

    glBindVertexArray(resources.vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, resources.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(VertexData),
                vertices.data(), GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, resources.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, group.indices.size() * sizeof(uint16_t),
                group.indices.data(), GL_STATIC_DRAW);

    // Vertex attributes
    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         (void*)offsetof(VertexData, position));

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         (void*)offsetof(VertexData, normal));

    // TexCoord
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         (void*)offsetof(VertexData, texCoord));

    // Color
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         (void*)offsetof(VertexData, color));

    glBindVertexArray(0);

    // Store collision geometry for floor raycasting
    resources.collisionVertices.reserve(group.vertices.size());
    for (const auto& v : group.vertices) {
        resources.collisionVertices.push_back(v.position);
    }
    resources.collisionIndices = group.indices;

    // Compute actual bounding box from vertices (WMO header bboxes can be unreliable)
    if (!resources.collisionVertices.empty()) {
        resources.boundingBoxMin = resources.collisionVertices[0];
        resources.boundingBoxMax = resources.collisionVertices[0];
        for (const auto& v : resources.collisionVertices) {
            resources.boundingBoxMin = glm::min(resources.boundingBoxMin, v);
            resources.boundingBoxMax = glm::max(resources.boundingBoxMax, v);
        }
    }

    // Create batches
    if (!group.batches.empty()) {
        for (const auto& batch : group.batches) {
            GroupResources::Batch resBatch;
            resBatch.startIndex = batch.startIndex;
            resBatch.indexCount = batch.indexCount;
            resBatch.materialId = batch.materialId;
            resources.batches.push_back(resBatch);
        }
    } else {
        // No batches defined - render entire group as one batch
        GroupResources::Batch batch;
        batch.startIndex = 0;
        batch.indexCount = resources.indexCount;
        batch.materialId = 0;
        resources.batches.push_back(batch);
    }

    return true;
}

void WMORenderer::renderGroup(const GroupResources& group, const ModelData& model,
                              [[maybe_unused]] const glm::mat4& modelMatrix,
                              [[maybe_unused]] const glm::mat4& view,
                              [[maybe_unused]] const glm::mat4& projection) {
    glBindVertexArray(group.vao);

    static int debugLogCount = 0;

    // Render each batch
    for (const auto& batch : group.batches) {
        // Bind texture for this batch's material
        // materialId -> materialTextureIndices[materialId] -> textures[texIndex]
        GLuint texId = whiteTexture;
        bool hasTexture = false;

        if (batch.materialId < model.materialTextureIndices.size()) {
            uint32_t texIndex = model.materialTextureIndices[batch.materialId];
            if (texIndex < model.textures.size()) {
                texId = model.textures[texIndex];
                hasTexture = (texId != 0 && texId != whiteTexture);

                if (debugLogCount < 10) {
                    core::Logger::getInstance().debug("  Batch: materialId=", (int)batch.materialId,
                        " -> texIndex=", texIndex, " -> texId=", texId, " hasTexture=", hasTexture);
                    debugLogCount++;
                }
            }
        }

        // Determine if this material uses alpha-test cutout (blendMode 1)
        bool alphaTest = false;
        if (batch.materialId < model.materialBlendModes.size()) {
            alphaTest = (model.materialBlendModes[batch.materialId] == 1);
        }

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texId);
        shader->setUniform("uTexture", 0);
        shader->setUniform("uHasTexture", hasTexture);
        shader->setUniform("uAlphaTest", alphaTest);

        glDrawElements(GL_TRIANGLES, batch.indexCount, GL_UNSIGNED_SHORT,
                      (void*)(batch.startIndex * sizeof(uint16_t)));
        lastDrawCalls++;
    }

    glBindVertexArray(0);
}

bool WMORenderer::isGroupVisible(const GroupResources& group, const glm::mat4& modelMatrix,
                                 const Camera& camera) const {
    // Simple frustum culling using bounding box
    // Transform bounding box corners to world space
    glm::vec3 corners[8] = {
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMin.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMin.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMax.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMax.y, group.boundingBoxMin.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMin.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMin.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMin.x, group.boundingBoxMax.y, group.boundingBoxMax.z),
        glm::vec3(group.boundingBoxMax.x, group.boundingBoxMax.y, group.boundingBoxMax.z)
    };

    // Transform corners to world space
    for (int i = 0; i < 8; i++) {
        glm::vec4 worldPos = modelMatrix * glm::vec4(corners[i], 1.0f);
        corners[i] = glm::vec3(worldPos);
    }

    // Simple check: if all corners are behind camera, cull
    // (This is a very basic culling implementation - a full frustum test would be better)
    glm::vec3 forward = camera.getForward();
    glm::vec3 camPos = camera.getPosition();

    int behindCount = 0;
    for (int i = 0; i < 8; i++) {
        glm::vec3 toCorner = corners[i] - camPos;
        if (glm::dot(toCorner, forward) < 0.0f) {
            behindCount++;
        }
    }

    // If all corners are behind camera, cull
    return behindCount < 8;
}

void WMORenderer::WMOInstance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Apply MODF placement rotation (WoW-to-GL coordinate transform)
    // WoW Ry(B)*Rx(A)*Rz(C) becomes GL Rz(B)*Ry(-A)*Rx(-C)
    // rotation stored as (-C, -A, B) in radians by caller
    // Apply in Z, Y, X order to get Rz(B) * Ry(-A) * Rx(-C)
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));

    // Cache inverse for collision detection
    invModelMatrix = glm::inverse(modelMatrix);
}

GLuint WMORenderer::loadTexture(const std::string& path) {
    if (!assetManager) {
        return whiteTexture;
    }

    // Check cache first
    auto it = textureCache.find(path);
    if (it != textureCache.end()) {
        return it->second;
    }

    // Load BLP texture
    pipeline::BLPImage blp = assetManager->loadTexture(path);
    if (!blp.isValid()) {
        core::Logger::getInstance().warning("WMO: Failed to load texture: ", path);
        textureCache[path] = whiteTexture;
        return whiteTexture;
    }

    core::Logger::getInstance().debug("WMO texture: ", path, " size=", blp.width, "x", blp.height);

    // Create OpenGL texture
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Upload texture data (BLP loader outputs RGBA8)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 blp.width, blp.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blp.data.data());

    // Set texture parameters with mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindTexture(GL_TEXTURE_2D, 0);

    // Cache it
    textureCache[path] = textureID;
    core::Logger::getInstance().debug("WMO: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return textureID;
}

// Ray-AABB intersection (slab method)
// Returns true if the ray intersects the axis-aligned bounding box
static bool rayIntersectsAABB(const glm::vec3& origin, const glm::vec3& dir,
                               const glm::vec3& bmin, const glm::vec3& bmax) {
    float tmin = -1e30f, tmax = 1e30f;
    for (int i = 0; i < 3; i++) {
        if (std::abs(dir[i]) < 1e-8f) {
            // Ray is parallel to this slab — check if origin is inside
            if (origin[i] < bmin[i] || origin[i] > bmax[i]) return false;
        } else {
            float invD = 1.0f / dir[i];
            float t0 = (bmin[i] - origin[i]) * invD;
            float t1 = (bmax[i] - origin[i]) * invD;
            if (t0 > t1) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmin > tmax) return false;
        }
    }
    return tmax >= 0.0f;  // At least part of the ray is forward
}

// Möller–Trumbore ray-triangle intersection
// Returns distance along ray if hit, or negative if miss
static float rayTriangleIntersect(const glm::vec3& origin, const glm::vec3& dir,
                                   const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    const float EPSILON = 1e-6f;
    glm::vec3 e1 = v1 - v0;
    glm::vec3 e2 = v2 - v0;
    glm::vec3 h = glm::cross(dir, e2);
    float a = glm::dot(e1, h);
    if (a > -EPSILON && a < EPSILON) return -1.0f;

    float f = 1.0f / a;
    glm::vec3 s = origin - v0;
    float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return -1.0f;

    glm::vec3 q = glm::cross(s, e1);
    float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return -1.0f;

    float t = f * glm::dot(e2, q);
    return t > EPSILON ? t : -1.0f;
}

std::optional<float> WMORenderer::getFloorHeight(float glX, float glY, float glZ) const {
    std::optional<float> bestFloor;

    // World-space ray: from high above, pointing straight down
    glm::vec3 worldOrigin(glX, glY, glZ + 500.0f);
    glm::vec3 worldDir(0.0f, 0.0f, -1.0f);

    // Debug: log when no instances
    static int debugCounter = 0;
    if (instances.empty() && (debugCounter++ % 300 == 0)) {
        core::Logger::getInstance().warning("WMO getFloorHeight: no instances loaded!");
    }

    for (const auto& instance : instances) {
        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(worldOrigin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(worldDir, 0.0f)));

        int groupsChecked = 0;
        int groupsSkipped = 0;
        int trianglesHit = 0;

        for (const auto& group : model.groups) {
            // Quick bounding box check
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                groupsSkipped++;
                continue;
            }
            groupsChecked++;

            // Raycast against triangles
            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                const glm::vec3& v0 = verts[indices[i]];
                const glm::vec3& v1 = verts[indices[i + 1]];
                const glm::vec3& v2 = verts[indices[i + 2]];

                // Try both winding orders (two-sided collision)
                float t = rayTriangleIntersect(localOrigin, localDir, v0, v1, v2);
                if (t <= 0.0f) {
                    // Try reverse winding
                    t = rayTriangleIntersect(localOrigin, localDir, v0, v2, v1);
                }

                if (t > 0.0f) {
                    trianglesHit++;
                    // Hit point in local space -> world space
                    glm::vec3 hitLocal = localOrigin + localDir * t;
                    glm::vec3 hitWorld = glm::vec3(instance.modelMatrix * glm::vec4(hitLocal, 1.0f));

                    // Only use floors below or near the query point
                    if (hitWorld.z <= glZ + 2.0f) {
                        if (!bestFloor || hitWorld.z > *bestFloor) {
                            bestFloor = hitWorld.z;
                        }
                    }
                }
            }
        }

        // Debug logging (every ~5 seconds at 60fps)
        static int logCounter = 0;
        if ((logCounter++ % 300 == 0) && (groupsChecked > 0 || groupsSkipped > 0)) {
            core::Logger::getInstance().debug("Floor check: ", groupsChecked, " groups checked, ",
                groupsSkipped, " skipped, ", trianglesHit, " hits, best=",
                bestFloor ? std::to_string(*bestFloor) : "none");
        }
    }

    return bestFloor;
}

bool WMORenderer::checkWallCollision(const glm::vec3& from, const glm::vec3& to, glm::vec3& adjustedPos) const {
    adjustedPos = to;
    bool blocked = false;

    glm::vec3 moveDir = to - from;
    float moveDistXY = glm::length(glm::vec2(moveDir.x, moveDir.y));
    if (moveDistXY < 0.001f) return false;

    // Player collision parameters
    const float PLAYER_RADIUS = 0.6f;       // Character collision radius
    const float PLAYER_HEIGHT = 2.0f;       // Player height for wall checks

    // Debug logging
    static int wallDebugCounter = 0;
    int groupsChecked = 0;
    int wallsHit = 0;

    for (const auto& instance : instances) {
        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Transform positions into local space using cached inverse
        glm::vec3 localTo = glm::vec3(instance.invModelMatrix * glm::vec4(to, 1.0f));
        float localFeetZ = localTo.z;

        for (const auto& group : model.groups) {
            // Quick bounding box check
            float margin = PLAYER_RADIUS + 2.0f;
            if (localTo.x < group.boundingBoxMin.x - margin || localTo.x > group.boundingBoxMax.x + margin ||
                localTo.y < group.boundingBoxMin.y - margin || localTo.y > group.boundingBoxMax.y + margin ||
                localTo.z < group.boundingBoxMin.z - margin || localTo.z > group.boundingBoxMax.z + margin) {
                continue;
            }
            groupsChecked++;

            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                const glm::vec3& v0 = verts[indices[i]];
                const glm::vec3& v1 = verts[indices[i + 1]];
                const glm::vec3& v2 = verts[indices[i + 2]];

                // Get triangle normal
                glm::vec3 edge1 = v1 - v0;
                glm::vec3 edge2 = v2 - v0;
                glm::vec3 normal = glm::cross(edge1, edge2);
                float normalLen = glm::length(normal);
                if (normalLen < 0.001f) continue;
                normal /= normalLen;

                // Skip mostly-horizontal triangles (floors/ceilings)
                // Only collide with walls (vertical surfaces)
                if (std::abs(normal.z) > 0.5f) continue;

                // Get triangle Z range
                float triMinZ = std::min({v0.z, v1.z, v2.z});
                float triMaxZ = std::max({v0.z, v1.z, v2.z});

                // Only collide with walls in player's vertical range
                if (triMaxZ < localFeetZ + 0.3f) continue;
                if (triMinZ > localFeetZ + PLAYER_HEIGHT) continue;

                // Signed distance from player to triangle plane
                float planeDist = glm::dot(localTo - v0, normal);
                float absPlaneDist = std::abs(planeDist);
                if (absPlaneDist > PLAYER_RADIUS) continue;

                // Project point onto plane
                glm::vec3 projected = localTo - normal * planeDist;

                // Check if projected point is inside triangle (or near edge)
                float d0 = glm::dot(glm::cross(v1 - v0, projected - v0), normal);
                float d1 = glm::dot(glm::cross(v2 - v1, projected - v1), normal);
                float d2 = glm::dot(glm::cross(v0 - v2, projected - v2), normal);

                // Allow small negative values for edge tolerance
                const float edgeTolerance = -0.1f;
                bool insideTriangle = (d0 >= edgeTolerance && d1 >= edgeTolerance && d2 >= edgeTolerance);

                if (insideTriangle) {
                    wallsHit++;
                    // Push player away from wall (horizontal only)
                    float pushDist = PLAYER_RADIUS - absPlaneDist;
                    if (pushDist > 0.0f) {
                        float sign = planeDist > 0.0f ? 1.0f : -1.0f;
                        glm::vec3 pushLocal = normal * sign * pushDist;

                        // Transform push vector back to world space
                        glm::vec3 pushWorld = glm::vec3(instance.modelMatrix * glm::vec4(pushLocal, 0.0f));

                        // Only horizontal push
                        adjustedPos.x += pushWorld.x;
                        adjustedPos.y += pushWorld.y;
                        blocked = true;
                    }
                }
            }
        }
    }

    // Debug logging every ~5 seconds
    if ((wallDebugCounter++ % 300 == 0) && !instances.empty()) {
        core::Logger::getInstance().debug("Wall collision: ", instances.size(), " instances, ",
            groupsChecked, " groups checked, ", wallsHit, " walls hit, blocked=", blocked);
    }

    return blocked;
}

bool WMORenderer::isInsideWMO(float glX, float glY, float glZ, uint32_t* outModelId) const {
    for (const auto& instance : instances) {
        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Check if inside any group's bounding box
        for (const auto& group : model.groups) {
            if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                if (outModelId) *outModelId = instance.modelId;
                return true;
            }
        }
    }
    return false;
}

float WMORenderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    float closestHit = maxDistance;

    for (const auto& instance : instances) {
        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));

        for (const auto& group : model.groups) {
            // Ray-AABB intersection (slab method)
            glm::vec3 tMin = (group.boundingBoxMin - localOrigin) / localDir;
            glm::vec3 tMax = (group.boundingBoxMax - localOrigin) / localDir;

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
    }

    return closestHit;
}

} // namespace rendering
} // namespace wowee
