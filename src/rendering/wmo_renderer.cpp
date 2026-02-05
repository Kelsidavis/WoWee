#include "rendering/wmo_renderer.hpp"
#include "rendering/texture.hpp"
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
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace wowee {
namespace rendering {

static void transformAABB(const glm::mat4& modelMatrix,
                          const glm::vec3& localMin,
                          const glm::vec3& localMax,
                          glm::vec3& outMin,
                          glm::vec3& outMax);

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
        uniform vec3 uLightColor;
        uniform float uSpecularIntensity;
        uniform vec3 uViewPos;
        uniform vec3 uAmbientColor;
        uniform sampler2D uTexture;
        uniform bool uHasTexture;
        uniform bool uAlphaTest;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        uniform sampler2DShadow uShadowMap;
        uniform mat4 uLightSpaceMatrix;
        uniform bool uShadowEnabled;
        uniform float uShadowStrength;

        out vec4 FragColor;

        void main() {
            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Diffuse lighting
            float diff = max(dot(normal, lightDir), 0.0);
            vec3 diffuse = diff * vec3(1.0);

            // Ambient
            vec3 ambient = uAmbientColor;

            // Blinn-Phong specular
            vec3 viewDir = normalize(uViewPos - FragPos);
            vec3 halfDir = normalize(lightDir + viewDir);
            float spec = pow(max(dot(normal, halfDir), 0.0), 32.0);
            vec3 specular = spec * uLightColor * uSpecularIntensity;

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

            // Shadow mapping
            float shadow = 1.0;
            if (uShadowEnabled) {
                vec4 lsPos = uLightSpaceMatrix * vec4(FragPos, 1.0);
                vec3 proj = lsPos.xyz / lsPos.w * 0.5 + 0.5;
                if (proj.z <= 1.0 && proj.x >= 0.0 && proj.x <= 1.0 && proj.y >= 0.0 && proj.y <= 1.0) {
                    float edgeDist = max(abs(proj.x - 0.5), abs(proj.y - 0.5));
                    float coverageFade = 1.0 - smoothstep(0.40, 0.49, edgeDist);
                    float bias = max(0.005 * (1.0 - dot(normal, lightDir)), 0.001);
                    shadow = 0.0;
                    vec2 texelSize = vec2(1.0 / 2048.0);
                    for (int sx = -1; sx <= 1; sx++) {
                        for (int sy = -1; sy <= 1; sy++) {
                            shadow += texture(uShadowMap, vec3(proj.xy + vec2(sx, sy) * texelSize, proj.z - bias));
                        }
                    }
                    shadow /= 9.0;
                    shadow = mix(1.0, shadow, coverageFade);
                }
            }
            shadow = mix(1.0, shadow, clamp(uShadowStrength, 0.0, 1.0));

            // Combine lighting with texture
            vec3 result = (ambient + (diffuse + specular) * shadow) * texColor.rgb;

            // Fog
            float fogDist = length(uViewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            result = mix(uFogColor, result, fogFactor);

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

    // Initialize occlusion query resources
    initOcclusionResources();

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
    spatialGrid.clear();
    instanceIndexById.clear();
    shader.reset();

    // Free occlusion query resources
    for (auto& [key, query] : occlusionQueries) {
        glDeleteQueries(1, &query);
    }
    occlusionQueries.clear();
    occlusionResults.clear();
    if (bboxVao != 0) { glDeleteVertexArrays(1, &bboxVao); bboxVao = 0; }
    if (bboxVbo != 0) { glDeleteBuffers(1, &bboxVbo); bboxVbo = 0; }
    occlusionShader.reset();
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

    // Copy portal data for visibility culling
    modelData.portalVertices = model.portalVertices;
    for (const auto& portal : model.portals) {
        PortalData pd;
        pd.startVertex = portal.startVertex;
        pd.vertexCount = portal.vertexCount;
        // Compute portal plane from vertices if we have them
        if (portal.vertexCount >= 3 && portal.startVertex + portal.vertexCount <= model.portalVertices.size()) {
            glm::vec3 v0 = model.portalVertices[portal.startVertex];
            glm::vec3 v1 = model.portalVertices[portal.startVertex + 1];
            glm::vec3 v2 = model.portalVertices[portal.startVertex + 2];
            pd.normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
            pd.distance = glm::dot(pd.normal, v0);
        } else {
            pd.normal = glm::vec3(0.0f, 0.0f, 1.0f);
            pd.distance = 0.0f;
        }
        modelData.portals.push_back(pd);
    }
    for (const auto& ref : model.portalRefs) {
        PortalRef pr;
        pr.portalIndex = ref.portalIndex;
        pr.groupIndex = ref.groupIndex;
        pr.side = ref.side;
        modelData.portalRefs.push_back(pr);
    }
    // Build per-group portal ref ranges from WMOGroup data
    modelData.groupPortalRefs.resize(model.groups.size(), {0, 0});
    for (size_t gi = 0; gi < model.groups.size(); gi++) {
        modelData.groupPortalRefs[gi] = {model.groups[gi].portalStart, model.groups[gi].portalCount};
    }

    if (!modelData.portals.empty()) {
        core::Logger::getInstance().info("WMO portals: ", modelData.portals.size(),
                                         " refs: ", modelData.portalRefs.size());
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
    const ModelData& model = loadedModels[modelId];
    transformAABB(instance.modelMatrix, model.boundingBoxMin, model.boundingBoxMax,
                  instance.worldBoundsMin, instance.worldBoundsMax);

    // Pre-compute world-space group bounds to avoid per-frame transformAABB
    instance.worldGroupBounds.reserve(model.groups.size());
    for (const auto& group : model.groups) {
        glm::vec3 gMin, gMax;
        transformAABB(instance.modelMatrix, group.boundingBoxMin, group.boundingBoxMax, gMin, gMax);
        gMin -= glm::vec3(0.5f);
        gMax += glm::vec3(0.5f);
        instance.worldGroupBounds.emplace_back(gMin, gMax);
    }

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
    core::Logger::getInstance().info("Created WMO instance ", instance.id, " (model ", modelId, ")");
    return instance.id;
}

void WMORenderer::removeInstance(uint32_t instanceId) {
    auto it = std::find_if(instances.begin(), instances.end(),
                          [instanceId](const WMOInstance& inst) { return inst.id == instanceId; });
    if (it != instances.end()) {
        instances.erase(it);
        rebuildSpatialIndex();
        core::Logger::getInstance().info("Removed WMO instance ", instanceId);
    }
}

void WMORenderer::clearInstances() {
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    core::Logger::getInstance().info("Cleared all WMO instances");
}

void WMORenderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocusEnabled = (radius > 0.0f);
    collisionFocusPos = worldPos;
    collisionFocusRadius = std::max(0.0f, radius);
    collisionFocusRadiusSq = collisionFocusRadius * collisionFocusRadius;
}

void WMORenderer::clearCollisionFocus() {
    collisionFocusEnabled = false;
}

void WMORenderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
}

WMORenderer::GridCell WMORenderer::toCell(const glm::vec3& p) const {
    return GridCell{
        static_cast<int>(std::floor(p.x / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.y / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.z / SPATIAL_CELL_SIZE))
    };
}

void WMORenderer::rebuildSpatialIndex() {
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

void WMORenderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
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

    // Safety fallback: if the grid misses due streaming/index drift, avoid
    // tunneling by scanning all instances instead of returning no candidates.
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
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
    shader->setUniform("uLightColor", glm::vec3(1.5f, 1.4f, 1.3f));
    shader->setUniform("uSpecularIntensity", 0.5f);
    shader->setUniform("uAmbientColor", glm::vec3(0.4f, 0.4f, 0.5f));
    shader->setUniform("uFogColor", fogColor);
    shader->setUniform("uFogStart", fogStart);
    shader->setUniform("uFogEnd", fogEnd);
    shader->setUniform("uShadowEnabled", shadowEnabled ? 1 : 0);
    shader->setUniform("uShadowStrength", 0.65f);
    if (shadowEnabled) {
        shader->setUniform("uLightSpaceMatrix", lightSpaceMatrix);
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, shadowDepthTex);
        shader->setUniform("uShadowMap", 7);
    }

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

    lastPortalCulledGroups = 0;
    lastDistanceCulledGroups = 0;
    lastOcclusionCulledGroups = 0;

    // Collect occlusion query results from previous frame (non-blocking)
    if (occlusionCulling) {
        for (auto& [queryKey, query] : occlusionQueries) {
            GLuint available = 0;
            glGetQueryObjectuiv(query, GL_QUERY_RESULT_AVAILABLE, &available);
            if (available) {
                GLuint result = 0;
                glGetQueryObjectuiv(query, GL_QUERY_RESULT, &result);
                occlusionResults[queryKey] = (result > 0);
            }
        }
    }

    // Render all instances with instance-level culling
    for (const auto& instance : instances) {
        // NOTE: Disabled hard instance-distance culling for WMOs.
        // Large city WMOs can have instance origins far from local camera position,
        // causing whole city sections to disappear unexpectedly.

        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) {
            continue;
        }

        if (frustumCulling) {
            glm::vec3 instMin = instance.worldBoundsMin - glm::vec3(0.5f);
            glm::vec3 instMax = instance.worldBoundsMax + glm::vec3(0.5f);
            if (!frustum.intersectsAABB(instMin, instMax)) {
                continue;
            }
        }

        const ModelData& model = modelIt->second;

        // Run occlusion queries for this instance (pre-pass)
        if (occlusionCulling && occlusionShader && bboxVao != 0) {
            runOcclusionQueries(instance, model, view, projection);
            // Re-bind main shader after occlusion pass
            shader->use();
        }

        shader->setUniform("uModel", instance.modelMatrix);

        // Portal-based visibility culling
        std::unordered_set<uint32_t> portalVisibleGroups;
        bool usePortalCulling = portalCulling && !model.portals.empty() && !model.portalRefs.empty();

        if (usePortalCulling) {
            // Transform camera position to model's local space
            glm::vec4 localCamPos = instance.invModelMatrix * glm::vec4(camera.getPosition(), 1.0f);
            glm::vec3 cameraLocalPos(localCamPos);

            getVisibleGroupsViaPortals(model, cameraLocalPos, frustum, instance.modelMatrix, portalVisibleGroups);
        }

        // Render all groups using cached world-space bounds
        glm::vec3 camPos = camera.getPosition();
        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // Portal culling check
            if (usePortalCulling && portalVisibleGroups.find(static_cast<uint32_t>(gi)) == portalVisibleGroups.end()) {
                lastPortalCulledGroups++;
                continue;
            }

            // Occlusion culling check (uses previous frame results)
            if (occlusionCulling && isGroupOccluded(instance.id, static_cast<uint32_t>(gi))) {
                lastOcclusionCulledGroups++;
                continue;
            }

            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];

                // Distance culling: skip groups whose center is too far
                if (distanceCulling) {
                    glm::vec3 groupCenter = (gMin + gMax) * 0.5f;
                    float distSq = glm::dot(groupCenter - camPos, groupCenter - camPos);
                    if (distSq > maxGroupDistanceSq) {
                        lastDistanceCulledGroups++;
                        continue;
                    }
                }

                // Frustum culling
                if (frustumCulling && !frustum.intersectsAABB(gMin, gMax)) {
                    continue;
                }
            }

            renderGroup(model.groups[gi], model, instance.modelMatrix, view, projection);
        }
    }

    // Restore polygon mode
    if (wireframeMode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Re-enable backface culling
    glEnable(GL_CULL_FACE);
}

void WMORenderer::renderShadow(const glm::mat4& lightView, const glm::mat4& lightProj, Shader& shadowShader) {
    if (instances.empty()) return;
    Frustum frustum;
    frustum.extractFromMatrix(lightProj * lightView);
    for (const auto& instance : instances) {
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) continue;
        if (frustumCulling) {
            glm::vec3 instMin = instance.worldBoundsMin - glm::vec3(0.5f);
            glm::vec3 instMax = instance.worldBoundsMax + glm::vec3(0.5f);
            if (!frustum.intersectsAABB(instMin, instMax)) continue;
        }
        const ModelData& model = modelIt->second;
        shadowShader.setUniform("uModel", instance.modelMatrix);
        for (const auto& group : model.groups) {
            glBindVertexArray(group.vao);
            glDrawElements(GL_TRIANGLES, group.indexCount, GL_UNSIGNED_SHORT, 0);
            glBindVertexArray(0);
        }
    }
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

    // Render each batch in original order (sorting breaks depth/alpha)
    for (const auto& batch : group.batches) {
        // Bind texture for this batch's material
        GLuint texId = whiteTexture;
        bool hasTexture = false;

        if (batch.materialId < model.materialTextureIndices.size()) {
            uint32_t texIndex = model.materialTextureIndices[batch.materialId];
            if (texIndex < model.textures.size()) {
                texId = model.textures[texIndex];
                hasTexture = (texId != 0 && texId != whiteTexture);
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

int WMORenderer::findContainingGroup(const ModelData& model, const glm::vec3& localPos) const {
    // Find which group's bounding box contains the position
    // Prefer interior groups (smaller volume) when multiple match
    int bestGroup = -1;
    float bestVolume = std::numeric_limits<float>::max();

    for (size_t gi = 0; gi < model.groups.size(); gi++) {
        const auto& group = model.groups[gi];
        if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
            localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
            localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
            glm::vec3 size = group.boundingBoxMax - group.boundingBoxMin;
            float volume = size.x * size.y * size.z;
            if (volume < bestVolume) {
                bestVolume = volume;
                bestGroup = static_cast<int>(gi);
            }
        }
    }
    return bestGroup;
}

bool WMORenderer::isPortalVisible(const ModelData& model, uint16_t portalIndex,
                                   [[maybe_unused]] const glm::vec3& cameraLocalPos,
                                   const Frustum& frustum,
                                   const glm::mat4& modelMatrix) const {
    if (portalIndex >= model.portals.size()) return false;

    const auto& portal = model.portals[portalIndex];
    if (portal.vertexCount < 3) return false;
    if (portal.startVertex + portal.vertexCount > model.portalVertices.size()) return false;

    // Get portal polygon center and bounds for frustum test
    glm::vec3 center(0.0f);
    glm::vec3 pMin = model.portalVertices[portal.startVertex];
    glm::vec3 pMax = pMin;
    for (uint16_t i = 0; i < portal.vertexCount; i++) {
        const auto& v = model.portalVertices[portal.startVertex + i];
        center += v;
        pMin = glm::min(pMin, v);
        pMax = glm::max(pMax, v);
    }
    center /= static_cast<float>(portal.vertexCount);

    // Transform bounds to world space for frustum test
    glm::vec4 worldMin = modelMatrix * glm::vec4(pMin, 1.0f);
    glm::vec4 worldMax = modelMatrix * glm::vec4(pMax, 1.0f);

    // Check if portal AABB intersects frustum (more robust than point test)
    return frustum.intersectsAABB(glm::vec3(worldMin), glm::vec3(worldMax));
}

void WMORenderer::getVisibleGroupsViaPortals(const ModelData& model,
                                              const glm::vec3& cameraLocalPos,
                                              const Frustum& frustum,
                                              const glm::mat4& modelMatrix,
                                              std::unordered_set<uint32_t>& outVisibleGroups) const {
    // Find camera's containing group
    int cameraGroup = findContainingGroup(model, cameraLocalPos);

    // If camera is outside all groups, fall back to frustum culling only
    if (cameraGroup < 0) {
        // Camera outside WMO - mark all groups as potentially visible
        // (will still be frustum culled in render)
        for (size_t gi = 0; gi < model.groups.size(); gi++) {
            outVisibleGroups.insert(static_cast<uint32_t>(gi));
        }
        return;
    }

    // BFS through portals from camera's group
    std::vector<bool> visited(model.groups.size(), false);
    std::vector<uint32_t> queue;
    queue.push_back(static_cast<uint32_t>(cameraGroup));
    visited[cameraGroup] = true;
    outVisibleGroups.insert(static_cast<uint32_t>(cameraGroup));

    size_t queueIdx = 0;
    while (queueIdx < queue.size()) {
        uint32_t currentGroup = queue[queueIdx++];

        // Get portal refs for this group
        if (currentGroup >= model.groupPortalRefs.size()) continue;
        auto [portalStart, portalCount] = model.groupPortalRefs[currentGroup];

        for (uint16_t pi = 0; pi < portalCount; pi++) {
            uint16_t refIdx = portalStart + pi;
            if (refIdx >= model.portalRefs.size()) continue;

            const auto& ref = model.portalRefs[refIdx];
            uint32_t targetGroup = ref.groupIndex;

            if (targetGroup >= model.groups.size()) continue;
            if (visited[targetGroup]) continue;

            // Check if portal is visible from camera
            if (isPortalVisible(model, ref.portalIndex, cameraLocalPos, frustum, modelMatrix)) {
                visited[targetGroup] = true;
                outVisibleGroups.insert(targetGroup);
                queue.push_back(targetGroup);
            }
        }
    }
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
    applyAnisotropicFiltering();
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

static void transformAABB(const glm::mat4& modelMatrix,
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
    for (const glm::vec3& corner : corners) {
        glm::vec3 world = glm::vec3(modelMatrix * glm::vec4(corner, 1.0f));
        outMin = glm::min(outMin, world);
        outMax = glm::max(outMax, world);
    }
}

static float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax) {
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

// Closest point on triangle (from Real-Time Collision Detection).
static glm::vec3 closestPointOnTriangle(const glm::vec3& p, const glm::vec3& a,
                                        const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ap = p - a;
    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

std::optional<float> WMORenderer::getFloorHeight(float glX, float glY, float glZ) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;

    // World-space ray: from high above, pointing straight down
    glm::vec3 worldOrigin(glX, glY, glZ + 500.0f);
    glm::vec3 worldDir(0.0f, 0.0f, -1.0f);

    // Debug: log when no instances
    static int debugCounter = 0;
    if (instances.empty() && (debugCounter++ % 300 == 0)) {
        core::Logger::getInstance().warning("WMO getFloorHeight: no instances loaded!");
    }

    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 8.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 10.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        // Broad-phase reject in world space to avoid expensive matrix transforms.
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - 2.0f || glZ > instance.worldBoundsMax.z + 4.0f) {
            continue;
        }

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

                    // Only use floors below or near the query point.
                    // Callers already elevate glZ by +5..+6; keep buffer small
                    // to avoid selecting ceilings above the player.
                    if (hitWorld.z <= glZ + 0.5f) {
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
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool blocked = false;

    glm::vec3 moveDir = to - from;
    float moveDistXY = glm::length(glm::vec2(moveDir.x, moveDir.y));
    if (moveDistXY < 0.001f) return false;

    // Player collision parameters
    const float PLAYER_RADIUS = 0.50f;      // Slightly narrower to pass tight doorways/interiors
    const float PLAYER_HEIGHT = 2.0f;       // Player height for wall checks
    const float MAX_STEP_HEIGHT = 0.85f;    // Balanced step-up without wall pass-through

    // Debug logging
    static int wallDebugCounter = 0;
    int groupsChecked = 0;
    int wallsHit = 0;

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(8.0f, 8.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(8.0f, 8.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        const float broadMargin = PLAYER_RADIUS + 1.5f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && to.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && to.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && to.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && to.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + PLAYER_HEIGHT && to.z > instance.worldBoundsMax.z + PLAYER_HEIGHT) continue;
        if (from.z + PLAYER_HEIGHT < instance.worldBoundsMin.z && to.z + PLAYER_HEIGHT < instance.worldBoundsMin.z) continue;

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Transform positions into local space using cached inverse
        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localTo = glm::vec3(instance.invModelMatrix * glm::vec4(to, 1.0f));
        float localFeetZ = localTo.z;
        // Use a tiny Z threshold so ramp-side logic still triggers with
        // smoothed ground snapping and small per-step vertical deltas.
        bool steppingUp = (localTo.z > localFrom.z + 0.005f);
        bool steppingDown = (localTo.z < localFrom.z - 0.005f);

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

                // Skip near-horizontal triangles (floors/ceilings).
                if (std::abs(normal.z) > 0.85f) continue;

                // Get triangle Z range
                float triMinZ = std::min({v0.z, v1.z, v2.z});
                float triMaxZ = std::max({v0.z, v1.z, v2.z});
                float fromDist = glm::dot(localFrom - v0, normal);
                float toDist = glm::dot(localTo - v0, normal);
                bool towardWallMotion = (std::abs(toDist) + 1e-4f < std::abs(fromDist));
                bool awayFromWallMotion = !towardWallMotion;

                // Only collide with walls in player's vertical range
                if (triMaxZ < localFeetZ + 0.3f) continue;
                if (triMinZ > localFeetZ + PLAYER_HEIGHT) continue;

                // Lower parts of ramps should be stepable from the side.
                // Allow a larger step-up budget for ramp-like triangles.
                // Allow running off/onto lower ramp side geometry without invisible wall blocks.
                if (normal.z > 0.30f && triMaxZ <= localFeetZ + 0.95f) continue;
                // Ignore short near-vertical side strips around ramps/edges.
                // These commonly act like invisible side guard rails.
                float triHeight = triMaxZ - triMinZ;
                bool likelyRealWall =
                    (std::abs(normal.z) < 0.20f) &&
                    (triHeight > 2.4f || triMaxZ > localFeetZ + 2.6f);
                bool structuralWall =
                    (triHeight > 1.6f) &&
                    (triMaxZ > localFeetZ + 1.8f);
                if (std::abs(normal.z) < 0.25f &&
                    triHeight < 1.8f &&
                    triMaxZ <= localFeetZ + 1.4f) {
                    continue;
                }
                // Motion-aware permissive ramp side strips:
                // keeps side entry/exit from behaving like invisible rails.
                bool rampSideStrip = false;
                if (steppingUp) {
                    rampSideStrip =
                        (std::abs(normal.z) > 0.02f && std::abs(normal.z) < 0.45f) &&
                        triMinZ <= localFeetZ + 0.30f &&
                        triHeight < 3.6f &&
                        triMaxZ <= localFeetZ + 2.8f;
                } else if (steppingDown) {
                    rampSideStrip =
                        (std::abs(normal.z) > 0.02f && std::abs(normal.z) < 0.65f) &&
                        triMinZ <= localFeetZ + 0.45f &&
                        triHeight < 4.5f &&
                        triMaxZ <= localFeetZ + 3.8f;
                }
                // High on ramps, side triangles can span very tall strips and
                // still behave like side rails. If we're stepping down and
                // moving away from the wall, don't let them trap movement.
                if (!rampSideStrip &&
                    steppingDown &&
                    awayFromWallMotion &&
                    std::abs(normal.z) > 0.02f && std::abs(normal.z) < 0.70f &&
                    triMinZ <= localFeetZ + 0.60f &&
                    triMaxZ <= localFeetZ + 4.5f &&
                    localFeetZ >= triMinZ + 0.80f) {
                    rampSideStrip = true;
                }
                if (rampSideStrip && !likelyRealWall && !structuralWall) {
                    continue;
                }
                // Let players run off ramp sides: ignore lower side-wall strips
                // that sit around foot height and are not true tall building walls.
                if (std::abs(normal.z) < 0.45f &&
                    std::abs(normal.z) > 0.05f &&
                    triMinZ <= localFeetZ + 0.20f &&
                    triHeight < 4.0f &&
                    triMaxZ <= localFeetZ + 4.0f &&
                    !likelyRealWall && !structuralWall) {
                    continue;
                }

                float stepHeightLimit = MAX_STEP_HEIGHT;
                if (triMaxZ <= localFeetZ + stepHeightLimit) continue;  // Treat as step-up, not hard wall

                // Swept test: prevent tunneling when crossing a wall between frames.
                bool shortRampEdgeStrip =
                    (steppingUp || steppingDown) &&
                    (std::abs(normal.z) > 0.01f && std::abs(normal.z) < (steppingDown ? 0.50f : 0.30f)) &&
                    triMinZ <= localFeetZ + (steppingDown ? 0.45f : 0.35f) &&
                    triHeight < (steppingDown ? 4.2f : 3.0f) &&
                    triMaxZ <= localFeetZ + (steppingDown ? 3.8f : 3.2f);
                if ((fromDist > PLAYER_RADIUS && toDist < -PLAYER_RADIUS) ||
                    (fromDist < -PLAYER_RADIUS && toDist > PLAYER_RADIUS)) {
                    // For true wall-like faces, always block segment crossing.
                    // Motion-direction heuristics are only for ramp-side stickiness.
                    if (!towardWallMotion && !likelyRealWall && !structuralWall) {
                        continue;
                    }
                    if (shortRampEdgeStrip && !likelyRealWall && !structuralWall) {
                        continue;
                    }
                    float denom = (fromDist - toDist);
                    if (std::abs(denom) > 1e-6f) {
                        float tHit = fromDist / denom;  // Segment param [0,1]
                        if (tHit >= 0.0f && tHit <= 1.0f) {
                            glm::vec3 hitPoint = localFrom + (localTo - localFrom) * tHit;
                            glm::vec3 hitClosest = closestPointOnTriangle(hitPoint, v0, v1, v2);
                            float hitErrSq = glm::dot(hitClosest - hitPoint, hitClosest - hitPoint);
                            bool insideHit = (hitErrSq <= 0.04f * 0.04f);
                            if (insideHit) {
                                float side = fromDist > 0.0f ? 1.0f : -1.0f;
                                glm::vec3 safeLocal = hitPoint + normal * side * (PLAYER_RADIUS + 0.03f);
                                glm::vec3 safeWorld = glm::vec3(instance.modelMatrix * glm::vec4(safeLocal, 1.0f));
                                adjustedPos.x = safeWorld.x;
                                adjustedPos.y = safeWorld.y;
                                blocked = true;
                                continue;
                            }
                        }
                    }
                }

                glm::vec3 closest = closestPointOnTriangle(localTo, v0, v1, v2);
                glm::vec3 delta = localTo - closest;
                float horizDist = glm::length(glm::vec2(delta.x, delta.y));
                if (horizDist <= PLAYER_RADIUS) {
                    wallsHit++;
                    // Push player away from wall (horizontal only, from closest point).
                    float pushDist = PLAYER_RADIUS - horizDist;
                    if (pushDist > 0.0f) {
                        glm::vec2 pushDir2;
                        if (horizDist > 1e-4f) {
                            pushDir2 = glm::normalize(glm::vec2(delta.x, delta.y));
                        } else {
                            glm::vec2 n2(normal.x, normal.y);
                            if (glm::length(n2) < 1e-4f) continue;
                            pushDir2 = glm::normalize(n2);
                        }

                        // Softer push when stepping up near ramp side edges.
                        bool rampEdgeLike = (std::abs(normal.z) < 0.45f && triHeight < 4.0f);
                        if (!towardWallMotion && !likelyRealWall && !structuralWall) continue;
                        if (shortRampEdgeStrip &&
                            !likelyRealWall && !structuralWall &&
                            std::abs(toDist) >= std::abs(fromDist) - PLAYER_RADIUS * 0.25f) continue;
                        float pushScale = 0.35f;
                        float pushCap = 0.06f;
                        if (rampEdgeLike && (steppingUp || steppingDown)) {
                            pushScale = steppingDown ? 0.08f : 0.12f;
                            pushCap = steppingDown ? 0.015f : 0.022f;
                        }
                        pushDist = std::min(pushCap, pushDist * pushScale);
                        if (pushDist <= 0.0f) continue;
                        glm::vec3 pushLocal(pushDir2.x * pushDist, pushDir2.y * pushDist, 0.0f);

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
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    glm::vec3 queryMin(glX - 0.5f, glY - 0.5f, glZ - 0.5f);
    glm::vec3 queryMax(glX + 0.5f, glY + 0.5f, glZ + 0.5f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z || glZ > instance.worldBoundsMax.z) {
            continue;
        }

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
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;
    // Camera collision should primarily react to walls.
    // Treat near-horizontal triangles as floor/ceiling and ignore them here so
    // ramps/stairs don't constantly pull the camera in and clip the floor view.
    constexpr float MAX_WALKABLE_ABS_NORMAL_Z = 0.20f;
    constexpr float MAX_HIT_BELOW_ORIGIN = 0.90f;
    constexpr float MAX_HIT_ABOVE_ORIGIN = 0.80f;
    constexpr float MIN_SURFACE_ALIGNMENT = 0.25f;

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

        glm::vec3 center = (instance.worldBoundsMin + instance.worldBoundsMax) * 0.5f;
        float radius = glm::length(instance.worldBoundsMax - center);
        if (glm::length(center - origin) > (maxDistance + radius + 1.0f)) {
            continue;
        }

        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.5f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.5f);
        if (!rayIntersectsAABB(origin, direction, worldMin, worldMax)) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));

        for (const auto& group : model.groups) {
            // Broad-phase cull with local AABB first.
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                continue;
            }

            // Narrow-phase: triangle raycast for accurate camera collision.
            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;
            for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                const glm::vec3& v0 = verts[indices[i]];
                const glm::vec3& v1 = verts[indices[i + 1]];
                const glm::vec3& v2 = verts[indices[i + 2]];
                glm::vec3 triNormal = glm::cross(v1 - v0, v2 - v0);
                float normalLenSq = glm::dot(triNormal, triNormal);
                if (normalLenSq < 1e-8f) {
                    continue;
                }
                triNormal /= std::sqrt(normalLenSq);
                if (std::abs(triNormal.z) > MAX_WALKABLE_ABS_NORMAL_Z) {
                    continue;
                }
                // Ignore near-grazing intersections that tend to come from ramps/arches
                // and cause camera pull-in even when no meaningful wall is behind the player.
                if (std::abs(glm::dot(triNormal, localDir)) < MIN_SURFACE_ALIGNMENT) {
                    continue;
                }

                float t = rayTriangleIntersect(localOrigin, localDir, v0, v1, v2);
                if (t <= 0.0f) {
                    // Two-sided collision.
                    t = rayTriangleIntersect(localOrigin, localDir, v0, v2, v1);
                }
                if (t <= 0.0f) continue;

                glm::vec3 localHit = localOrigin + localDir * t;
                glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
                // Ignore low hits; camera floor handling already keeps the camera above ground.
                // This avoids gate/ramp floor geometry pulling the camera in too aggressively.
                if (worldHit.z < origin.z - MAX_HIT_BELOW_ORIGIN) {
                    continue;
                }
                // Ignore very high hits (arches/ceilings) that should not clamp normal chase-cam distance.
                if (worldHit.z > origin.z + MAX_HIT_ABOVE_ORIGIN) {
                    continue;
                }
                float worldDist = glm::length(worldHit - origin);
                if (worldDist > 0.0f && worldDist < closestHit && worldDist <= maxDistance) {
                    closestHit = worldDist;
                }
            }
        }
    }

    return closestHit;
}

void WMORenderer::initOcclusionResources() {
    // Simple vertex shader for bounding box rendering
    const char* occVertSrc = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uMVP;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
        }
    )";

    // Fragment shader that writes nothing (depth-only)
    const char* occFragSrc = R"(
        #version 330 core
        void main() {
            // No color output - depth only
        }
    )";

    occlusionShader = std::make_unique<Shader>();
    if (!occlusionShader->loadFromSource(occVertSrc, occFragSrc)) {
        core::Logger::getInstance().warning("Failed to create occlusion shader");
        occlusionCulling = false;
        return;
    }

    // Create unit cube vertices (will be scaled to group bounds)
    float cubeVerts[] = {
        // Front face
        0,0,1, 1,0,1, 1,1,1, 0,0,1, 1,1,1, 0,1,1,
        // Back face
        1,0,0, 0,0,0, 0,1,0, 1,0,0, 0,1,0, 1,1,0,
        // Left face
        0,0,0, 0,0,1, 0,1,1, 0,0,0, 0,1,1, 0,1,0,
        // Right face
        1,0,1, 1,0,0, 1,1,0, 1,0,1, 1,1,0, 1,1,1,
        // Top face
        0,1,1, 1,1,1, 1,1,0, 0,1,1, 1,1,0, 0,1,0,
        // Bottom face
        0,0,0, 1,0,0, 1,0,1, 0,0,0, 1,0,1, 0,0,1,
    };

    glGenVertexArrays(1, &bboxVao);
    glGenBuffers(1, &bboxVbo);

    glBindVertexArray(bboxVao);
    glBindBuffer(GL_ARRAY_BUFFER, bboxVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVerts), cubeVerts, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    core::Logger::getInstance().info("Occlusion query resources initialized");
}

void WMORenderer::runOcclusionQueries(const WMOInstance& instance, const ModelData& model,
                                       const glm::mat4& view, const glm::mat4& projection) {
    if (!occlusionShader || bboxVao == 0) return;

    occlusionShader->use();
    glBindVertexArray(bboxVao);

    // Disable color writes, keep depth test
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);  // Don't write depth

    for (size_t gi = 0; gi < model.groups.size(); ++gi) {
        const auto& group = model.groups[gi];

        // Create query key
        uint32_t queryKey = (instance.id << 16) | static_cast<uint32_t>(gi);

        // Get or create query object
        GLuint query;
        auto it = occlusionQueries.find(queryKey);
        if (it == occlusionQueries.end()) {
            glGenQueries(1, &query);
            occlusionQueries[queryKey] = query;
        } else {
            query = it->second;
        }

        // Compute MVP for this group's bounding box
        glm::vec3 bboxSize = group.boundingBoxMax - group.boundingBoxMin;
        glm::mat4 bboxModel = instance.modelMatrix;
        bboxModel = glm::translate(bboxModel, group.boundingBoxMin);
        bboxModel = glm::scale(bboxModel, bboxSize);
        glm::mat4 mvp = projection * view * bboxModel;

        occlusionShader->setUniform("uMVP", mvp);

        // Run occlusion query
        glBeginQuery(GL_ANY_SAMPLES_PASSED, query);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glEndQuery(GL_ANY_SAMPLES_PASSED);
    }

    // Restore state
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glBindVertexArray(0);
}

bool WMORenderer::isGroupOccluded(uint32_t instanceId, uint32_t groupIndex) const {
    uint32_t queryKey = (instanceId << 16) | groupIndex;

    // Check previous frame's result
    auto resultIt = occlusionResults.find(queryKey);
    if (resultIt != occlusionResults.end()) {
        return !resultIt->second;  // Return true if NOT visible
    }

    // No result yet - assume visible
    return false;
}

} // namespace rendering
} // namespace wowee
