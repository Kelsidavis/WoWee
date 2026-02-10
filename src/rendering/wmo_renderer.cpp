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
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <thread>
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

    numCullThreads_ = std::min(4u, std::max(1u, std::thread::hardware_concurrency() - 1));

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
        uniform bool uUnlit;
        uniform bool uIsInterior;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        uniform sampler2DShadow uShadowMap;
        uniform mat4 uLightSpaceMatrix;
        uniform bool uShadowEnabled;
        uniform float uShadowStrength;

        out vec4 FragColor;

        void main() {
            // Sample texture or use vertex color
            vec4 texColor;
            float alpha = 1.0;
            if (uHasTexture) {
                texColor = texture(uTexture, TexCoord);
                // Alpha test only for cutout materials (lattice, grating, etc.)
                if (uAlphaTest && texColor.a < 0.5) discard;
                alpha = texColor.a;
                // Don't multiply texture by vertex color here - it zeros out black MOCV areas
                // Vertex colors will be applied as AO modulation after lighting
            } else {
                // MOCV vertex color alpha is a lighting blend factor, not transparency
                texColor = vec4(VertexColor.rgb, 1.0);
            }

            // Unlit materials (windows, lamps) — emit texture color directly
            if (uUnlit) {
                // Apply fog only
                float fogDist = length(uViewPos - FragPos);
                float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
                vec3 result = mix(uFogColor, texColor.rgb, fogFactor);
                FragColor = vec4(result, alpha);
                return;
            }

            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            vec3 litColor;
            if (uIsInterior) {
                // Interior: MOCV vertex colors are baked lighting.
                // Use them directly as the light multiplier on the texture.
                vec3 vertLight = VertexColor.rgb * 2.4 + 0.35;
                // Subtle directional fill so geometry reads
                float diff = max(dot(normal, lightDir), 0.0);
                vertLight += diff * 0.10;
                litColor = texColor.rgb * vertLight;
            } else {
                // Exterior: standard diffuse + specular lighting
                vec3 ambient = uAmbientColor;

                float diff = max(dot(normal, lightDir), 0.0);
                vec3 diffuse = diff * vec3(1.0);

                vec3 viewDir = normalize(uViewPos - FragPos);
                vec3 halfDir = normalize(lightDir + viewDir);
                float spec = pow(max(dot(normal, halfDir), 0.0), 32.0);
                vec3 specular = spec * uLightColor * uSpecularIntensity;

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

                litColor = (ambient + (diffuse + specular) * shadow) * texColor.rgb;

                // Apply vertex color as ambient occlusion (AO) with minimum to prevent blackout
                // MOCV values of (0,0,0) are clamped to 0.5 to keep areas visible
                vec3 ao = max(VertexColor.rgb, vec3(0.5));
                litColor *= ao;
            }

            // Fog
            float fogDist = length(uViewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            vec3 result = mix(uFogColor, litColor, fogFactor);

            FragColor = vec4(result, alpha);
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
    {
        glm::vec3 ext = model.boundingBoxMax - model.boundingBoxMin;
        float horiz = std::max(ext.x, ext.y);
        float vert = ext.z;
        modelData.isLowPlatform = (vert < 6.0f && horiz > 20.0f);
    }

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
        modelData.materialFlags.push_back(mat.flags);
    }

    // Create GPU resources for each group
    uint32_t loadedGroups = 0;
    for (const auto& wmoGroup : model.groups) {
        // Skip empty groups
        if (wmoGroup.vertices.empty() || wmoGroup.indices.empty()) {
            continue;
        }

        GroupResources resources;
        if (createGroupResources(wmoGroup, resources, wmoGroup.flags)) {
            modelData.groups.push_back(resources);
            loadedGroups++;
        }
    }

    if (loadedGroups == 0) {
        core::Logger::getInstance().warning("No valid groups loaded for WMO ", id);
        return false;
    }

    // Build pre-merged batches for each group (texture-sorted for efficient rendering)
    for (auto& groupRes : modelData.groups) {
        std::unordered_map<uint64_t, GroupResources::MergedBatch> batchMap;

        for (const auto& batch : groupRes.batches) {
            GLuint texId = whiteTexture;
            bool hasTexture = false;

            if (batch.materialId < modelData.materialTextureIndices.size()) {
                uint32_t texIndex = modelData.materialTextureIndices[batch.materialId];
                if (texIndex < modelData.textures.size()) {
                    texId = modelData.textures[texIndex];
                    hasTexture = (texId != 0 && texId != whiteTexture);
                }
            }

            bool alphaTest = false;
            uint32_t blendMode = 0;
            if (batch.materialId < modelData.materialBlendModes.size()) {
                blendMode = modelData.materialBlendModes[batch.materialId];
                alphaTest = (blendMode == 1);
            }

            bool unlit = false;
            if (batch.materialId < modelData.materialFlags.size()) {
                unlit = (modelData.materialFlags[batch.materialId] & 0x01) != 0;
            }

            // Merge key: texture ID + alphaTest + unlit (unlit batches must not merge with lit)
            uint64_t key = (static_cast<uint64_t>(texId) << 2)
                         | (alphaTest ? 1ULL : 0ULL)
                         | (unlit ? 2ULL : 0ULL);
            auto& mb = batchMap[key];
            if (mb.counts.empty()) {
                mb.texId = texId;
                mb.hasTexture = hasTexture;
                mb.alphaTest = alphaTest;
                mb.unlit = unlit;
                mb.blendMode = blendMode;
            }
            mb.counts.push_back(static_cast<GLsizei>(batch.indexCount));
            mb.offsets.push_back(reinterpret_cast<const void*>(batch.startIndex * sizeof(uint16_t)));
        }

        groupRes.mergedBatches.reserve(batchMap.size());
        for (auto& [key, mb] : batchMap) {
            groupRes.mergedBatches.push_back(std::move(mb));
        }
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

void WMORenderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.position = position;
    inst.updateModelMatrix();
    auto modelIt = loadedModels.find(inst.modelId);
    if (modelIt != loadedModels.end()) {
        const ModelData& model = modelIt->second;
        transformAABB(inst.modelMatrix, model.boundingBoxMin, model.boundingBoxMax,
                      inst.worldBoundsMin, inst.worldBoundsMax);
        inst.worldGroupBounds.clear();
        inst.worldGroupBounds.reserve(model.groups.size());
        for (const auto& group : model.groups) {
            glm::vec3 gMin, gMax;
            transformAABB(inst.modelMatrix, group.boundingBoxMin, group.boundingBoxMax, gMin, gMax);
            gMin -= glm::vec3(0.5f);
            gMax += glm::vec3(0.5f);
            inst.worldGroupBounds.emplace_back(gMin, gMax);
        }
    }
    rebuildSpatialIndex();
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
    precomputedFloorGrid.clear();  // Invalidate floor cache when instances change
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
    currentFrameId++;
    // Note: precomputedFloorGrid is persistent and not cleared per-frame
}

bool WMORenderer::saveFloorCache() const {
    if (mapName_.empty()) {
        core::Logger::getInstance().warning("Cannot save floor cache: no map name set");
        return false;
    }

    std::string filepath = "cache/wmo_floor_" + mapName_ + ".bin";

    // Create directory if needed
    std::filesystem::path path(filepath);
    std::filesystem::path absPath = std::filesystem::absolute(path);
    core::Logger::getInstance().info("Saving floor cache to: ", absPath.string());

    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            core::Logger::getInstance().error("Failed to create cache directory: ", ec.message());
        }
    }

    std::ofstream file(filepath, std::ios::binary);
    if (!file) {
        core::Logger::getInstance().error("Failed to open floor cache file for writing: ", filepath);
        return false;
    }

    // Write header: magic + version + count
    const uint32_t magic = 0x574D4F46;  // "WMOF"
    const uint32_t version = 1;
    const uint64_t count = precomputedFloorGrid.size();

    file.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    file.write(reinterpret_cast<const char*>(&count), sizeof(count));

    // Write each entry: key (uint64) + height (float)
    for (const auto& [key, height] : precomputedFloorGrid) {
        file.write(reinterpret_cast<const char*>(&key), sizeof(key));
        file.write(reinterpret_cast<const char*>(&height), sizeof(height));
    }

    core::Logger::getInstance().info("Saved WMO floor cache (", mapName_, "): ", count, " entries");
    return true;
}

bool WMORenderer::loadFloorCache() {
    if (mapName_.empty()) {
        core::Logger::getInstance().warning("Cannot load floor cache: no map name set");
        return false;
    }

    std::string filepath = "cache/wmo_floor_" + mapName_ + ".bin";

    std::ifstream file(filepath, std::ios::binary);
    if (!file) {
        core::Logger::getInstance().info("No existing floor cache for map: ", mapName_);
        return false;
    }

    // Read and validate header
    uint32_t magic = 0, version = 0;
    uint64_t count = 0;

    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&count), sizeof(count));

    if (magic != 0x574D4F46 || version != 1) {
        core::Logger::getInstance().warning("Invalid floor cache file format: ", filepath);
        return false;
    }

    // Read entries
    precomputedFloorGrid.clear();
    precomputedFloorGrid.reserve(count);

    for (uint64_t i = 0; i < count; i++) {
        uint64_t key;
        float height;
        file.read(reinterpret_cast<char*>(&key), sizeof(key));
        file.read(reinterpret_cast<char*>(&height), sizeof(height));
        precomputedFloorGrid[key] = height;
    }

    core::Logger::getInstance().info("Loaded WMO floor cache (", mapName_, "): ", precomputedFloorGrid.size(), " entries");
    return true;
}

void WMORenderer::precomputeFloorCache() {
    if (instances.empty()) {
        core::Logger::getInstance().info("precomputeFloorCache: no instances to precompute");
        return;
    }

    size_t startSize = precomputedFloorGrid.size();
    size_t samplesChecked = 0;

    core::Logger::getInstance().info("Pre-computing floor cache for ", instances.size(), " WMO instances...");

    for (const auto& instance : instances) {
        // Get world bounds for this instance
        const glm::vec3& boundsMin = instance.worldBoundsMin;
        const glm::vec3& boundsMax = instance.worldBoundsMax;

        // Sample reference Z is above the structure
        float refZ = boundsMax.z + 10.0f;

        // Iterate over grid points within the bounds
        float startX = std::floor(boundsMin.x / FLOOR_GRID_CELL_SIZE) * FLOOR_GRID_CELL_SIZE;
        float startY = std::floor(boundsMin.y / FLOOR_GRID_CELL_SIZE) * FLOOR_GRID_CELL_SIZE;

        for (float x = startX; x <= boundsMax.x; x += FLOOR_GRID_CELL_SIZE) {
            for (float y = startY; y <= boundsMax.y; y += FLOOR_GRID_CELL_SIZE) {
                // Sample at grid cell center
                float sampleX = x + FLOOR_GRID_CELL_SIZE * 0.5f;
                float sampleY = y + FLOOR_GRID_CELL_SIZE * 0.5f;

                // Check if already cached
                uint64_t key = floorGridKey(sampleX, sampleY);
                if (precomputedFloorGrid.find(key) != precomputedFloorGrid.end()) {
                    continue;  // Already computed
                }

                samplesChecked++;

                // getFloorHeight will compute and cache the result
                getFloorHeight(sampleX, sampleY, refZ);
            }
        }
    }

    size_t newEntries = precomputedFloorGrid.size() - startSize;
    core::Logger::getInstance().info("Floor cache precompute complete: ", samplesChecked, " samples checked, ",
                                     newEntries, " new entries, total ", precomputedFloorGrid.size());
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
    shader->setUniform("uAmbientColor", glm::vec3(0.55f, 0.55f, 0.6f));
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

    // Set up texture unit 0 for diffuse textures (set once per frame)
    glActiveTexture(GL_TEXTURE0);
    shader->setUniform("uTexture", 0);

    // Initialize new uniforms to defaults
    shader->setUniform("uUnlit", false);
    shader->setUniform("uIsInterior", false);

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

    // ── Phase 1: Parallel visibility culling ──────────────────────────
    // Build list of instances that pass the coarse instance-level frustum test.
    std::vector<size_t> visibleInstances;
    visibleInstances.reserve(instances.size());
    for (size_t i = 0; i < instances.size(); ++i) {
        const auto& instance = instances[i];
        if (loadedModels.find(instance.modelId) == loadedModels.end())
            continue;

        if (frustumCulling) {
            glm::vec3 instMin = instance.worldBoundsMin - glm::vec3(0.5f);
            glm::vec3 instMax = instance.worldBoundsMax + glm::vec3(0.5f);
            if (!frustum.intersectsAABB(instMin, instMax))
                continue;
        }
        visibleInstances.push_back(i);
    }

    // Per-instance cull lambda — produces an InstanceDrawList for one instance.
    // Reads only const data; each invocation writes to its own output.
    glm::vec3 camPos = camera.getPosition();
    bool doPortalCull = portalCulling;
    bool doOcclusionCull = occlusionCulling;
    bool doFrustumCull = frustumCulling;

    auto cullInstance = [&](size_t instIdx) -> InstanceDrawList {
        if (instIdx >= instances.size()) return InstanceDrawList{};
        const auto& instance = instances[instIdx];
        auto mdlIt = loadedModels.find(instance.modelId);
        if (mdlIt == loadedModels.end()) return InstanceDrawList{};
        const ModelData& model = mdlIt->second;

        InstanceDrawList result;
        result.instanceIndex = instIdx;

        // Portal-based visibility
        std::unordered_set<uint32_t> portalVisibleGroups;
        bool usePortalCulling = doPortalCull && !model.portals.empty() && !model.portalRefs.empty();
        if (usePortalCulling) {
            glm::vec4 localCamPos = instance.invModelMatrix * glm::vec4(camPos, 1.0f);
            getVisibleGroupsViaPortals(model, glm::vec3(localCamPos), frustum,
                                       instance.modelMatrix, portalVisibleGroups);
        }

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // Portal culling
            if (usePortalCulling &&
                portalVisibleGroups.find(static_cast<uint32_t>(gi)) == portalVisibleGroups.end()) {
                result.portalCulled++;
                continue;
            }

            // Occlusion culling (reads previous-frame results, read-only map)
            if (doOcclusionCull && isGroupOccluded(instance.id, static_cast<uint32_t>(gi))) {
                result.occlusionCulled++;
                continue;
            }

            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];

                // Hard distance cutoff
                glm::vec3 closestPoint = glm::clamp(camPos, gMin, gMax);
                float distSq = glm::dot(closestPoint - camPos, closestPoint - camPos);
                if (distSq > 25600.0f) {
                    result.distanceCulled++;
                    continue;
                }

                // Frustum culling
                if (doFrustumCull && !frustum.intersectsAABB(gMin, gMax))
                    continue;
            }

            result.visibleGroups.push_back(static_cast<uint32_t>(gi));
        }
        return result;
    };

    // Dispatch culling — parallel when enough instances, sequential otherwise.
    std::vector<InstanceDrawList> drawLists;
    drawLists.reserve(visibleInstances.size());

    if (visibleInstances.size() >= 4 && numCullThreads_ > 1) {
        const size_t numThreads = std::min(static_cast<size_t>(numCullThreads_),
                                           visibleInstances.size());
        const size_t chunkSize = visibleInstances.size() / numThreads;
        const size_t remainder = visibleInstances.size() % numThreads;

        // Each future returns a vector of InstanceDrawList for its chunk.
        std::vector<std::future<std::vector<InstanceDrawList>>> futures;
        futures.reserve(numThreads);

        size_t start = 0;
        for (size_t t = 0; t < numThreads; ++t) {
            size_t end = start + chunkSize + (t < remainder ? 1 : 0);
            futures.push_back(std::async(std::launch::async,
                [&, start, end]() {
                    std::vector<InstanceDrawList> chunk;
                    chunk.reserve(end - start);
                    for (size_t j = start; j < end; ++j)
                        chunk.push_back(cullInstance(visibleInstances[j]));
                    return chunk;
                }));
            start = end;
        }

        for (auto& f : futures) {
            auto chunk = f.get();
            for (auto& dl : chunk)
                drawLists.push_back(std::move(dl));
        }
    } else {
        for (size_t idx : visibleInstances)
            drawLists.push_back(cullInstance(idx));
    }

    // ── Phase 2: Sequential GL draw ────────────────────────────────
    for (const auto& dl : drawLists) {
        if (dl.instanceIndex >= instances.size()) continue;
        const auto& instance = instances[dl.instanceIndex];
        auto modelIt = loadedModels.find(instance.modelId);
        if (modelIt == loadedModels.end()) continue;
        const ModelData& model = modelIt->second;

        // Occlusion query pre-pass (GL calls — must be main thread)
        if (occlusionCulling && occlusionShader && bboxVao != 0) {
            runOcclusionQueries(instance, model, view, projection);
            shader->use();
        }

        shader->setUniform("uModel", instance.modelMatrix);

        // Debug logging for STORMWIND.WMO groups to identify LOD shell
        static bool loggedStormwindGroups = false;
        if (!loggedStormwindGroups && instance.modelId == 10047) {
            glm::vec3 cameraPos = camera.getPosition();
            float distToWMO = glm::length(cameraPos - instance.position);
            LOG_INFO("=== STORMWIND.WMO Group Rendering (dist=", distToWMO, ") ===");
            for (uint32_t gi : dl.visibleGroups) {
                const auto& group = model.groups[gi];
                glm::vec3 groupCenter = (group.boundingBoxMin + group.boundingBoxMax) * 0.5f;
                glm::vec4 worldCenter = instance.modelMatrix * glm::vec4(groupCenter, 1.0f);
                float distToGroup = glm::length(cameraPos - glm::vec3(worldCenter));
                LOG_INFO("  Group ", gi, ": flags=0x", std::hex, group.groupFlags, std::dec,
                         " dist=", distToGroup, " verts=", group.vertexCount);
            }
            loggedStormwindGroups = true;  // Only log once to avoid spam
        }

        for (uint32_t gi : dl.visibleGroups)
            renderGroup(model.groups[gi], model, instance.modelMatrix, view, projection);

        lastPortalCulledGroups += dl.portalCulled;
        lastDistanceCulledGroups += dl.distanceCulled;
        lastOcclusionCulledGroups += dl.occlusionCulled;
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

bool WMORenderer::createGroupResources(const pipeline::WMOGroup& group, GroupResources& resources, uint32_t groupFlags) {
    if (group.vertices.empty() || group.indices.empty()) {
        return false;
    }

    resources.groupFlags = groupFlags;

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

    // Build 2D spatial grid for fast collision triangle lookup
    resources.buildCollisionGrid();

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

void WMORenderer::renderGroup(const GroupResources& group, [[maybe_unused]] const ModelData& model,
                              [[maybe_unused]] const glm::mat4& modelMatrix,
                              [[maybe_unused]] const glm::mat4& view,
                              [[maybe_unused]] const glm::mat4& projection) {
    glBindVertexArray(group.vao);

    // Set interior flag once per group (0x2000 = interior)
    bool isInterior = (group.groupFlags & 0x2000) != 0;
    shader->setUniform("uIsInterior", isInterior);

    // Use pre-computed merged batches (built at load time)
    // Track bound state to avoid redundant GL calls
    static GLuint lastBoundTex = 0;
    static bool lastHasTexture = false;
    static bool lastAlphaTest = false;
    static bool lastUnlit = false;

    for (const auto& mb : group.mergedBatches) {
        if (mb.texId != lastBoundTex) {
            glBindTexture(GL_TEXTURE_2D, mb.texId);
            lastBoundTex = mb.texId;
        }
        if (mb.hasTexture != lastHasTexture) {
            shader->setUniform("uHasTexture", mb.hasTexture);
            lastHasTexture = mb.hasTexture;
        }
        if (mb.alphaTest != lastAlphaTest) {
            shader->setUniform("uAlphaTest", mb.alphaTest);
            lastAlphaTest = mb.alphaTest;
        }
        if (mb.unlit != lastUnlit) {
            shader->setUniform("uUnlit", mb.unlit);
            lastUnlit = mb.unlit;
        }

        // Enable alpha blending for translucent materials (blendMode >= 2)
        bool needsBlend = (mb.blendMode >= 2);
        if (needsBlend) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        glMultiDrawElements(GL_TRIANGLES, mb.counts.data(), GL_UNSIGNED_SHORT,
                            mb.offsets.data(), static_cast<GLsizei>(mb.counts.size()));
        lastDrawCalls++;

        if (needsBlend) {
            glDisable(GL_BLEND);
        }
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

// ---- Per-group 2D collision grid ----

void WMORenderer::GroupResources::buildCollisionGrid() {
    if (collisionVertices.empty() || collisionIndices.size() < 3) {
        gridCellsX = 0;
        gridCellsY = 0;
        return;
    }

    gridOrigin = glm::vec2(boundingBoxMin.x, boundingBoxMin.y);
    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;

    gridCellsX = std::max(1, static_cast<int>(std::ceil(extentX / COLLISION_CELL_SIZE)));
    gridCellsY = std::max(1, static_cast<int>(std::ceil(extentY / COLLISION_CELL_SIZE)));

    // Cap grid size to avoid excessive memory for huge groups
    if (gridCellsX > 64) gridCellsX = 64;
    if (gridCellsY > 64) gridCellsY = 64;

    size_t totalCells = gridCellsX * gridCellsY;
    cellTriangles.resize(totalCells);
    cellFloorTriangles.resize(totalCells);
    cellWallTriangles.resize(totalCells);

    size_t numTriangles = collisionIndices.size() / 3;
    triBounds.resize(numTriangles);

    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    for (size_t i = 0; i + 2 < collisionIndices.size(); i += 3) {
        const glm::vec3& v0 = collisionVertices[collisionIndices[i]];
        const glm::vec3& v1 = collisionVertices[collisionIndices[i + 1]];
        const glm::vec3& v2 = collisionVertices[collisionIndices[i + 2]];

        // Triangle XY bounding box
        float triMinX = std::min({v0.x, v1.x, v2.x});
        float triMinY = std::min({v0.y, v1.y, v2.y});
        float triMaxX = std::max({v0.x, v1.x, v2.x});
        float triMaxY = std::max({v0.y, v1.y, v2.y});

        // Per-triangle Z bounds
        float triMinZ = std::min({v0.z, v1.z, v2.z});
        float triMaxZ = std::max({v0.z, v1.z, v2.z});
        triBounds[i / 3] = { triMinZ, triMaxZ };

        // Classify floor vs wall by normal.
        // Wall threshold matches MAX_WALK_SLOPE_DOT (cos 50° ≈ 0.6428) so that
        // surfaces too steep to walk on are always tested for wall collision.
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 normal = glm::cross(edge1, edge2);
        float normalLen = glm::length(normal);
        float absNz = (normalLen > 0.001f) ? std::abs(normal.z / normalLen) : 0.0f;
        bool isFloor = (absNz >= 0.35f);  // ~70° max slope (relaxed for steep stairs)
        bool isWall = (absNz < 0.65f);  // Matches walkable slope threshold

        int cellMinX = std::max(0, static_cast<int>((triMinX - gridOrigin.x) * invCellW));
        int cellMinY = std::max(0, static_cast<int>((triMinY - gridOrigin.y) * invCellH));
        int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((triMaxX - gridOrigin.x) * invCellW));
        int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((triMaxY - gridOrigin.y) * invCellH));

        uint32_t triIdx = static_cast<uint32_t>(i);
        for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
            for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
                int cellIdx = cy * gridCellsX + cx;
                cellTriangles[cellIdx].push_back(triIdx);
                if (isFloor) cellFloorTriangles[cellIdx].push_back(triIdx);
                if (isWall) cellWallTriangles[cellIdx].push_back(triIdx);
            }
        }
    }
}

const std::vector<uint32_t>* WMORenderer::GroupResources::getTrianglesAtLocal(float localX, float localY) const {
    if (gridCellsX == 0 || gridCellsY == 0) return nullptr;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cx = static_cast<int>((localX - gridOrigin.x) * invCellW);
    int cy = static_cast<int>((localY - gridOrigin.y) * invCellH);

    if (cx < 0 || cx >= gridCellsX || cy < 0 || cy >= gridCellsY) return nullptr;

    return &cellTriangles[cy * gridCellsX + cx];
}

void WMORenderer::GroupResources::getTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    // Collect unique triangle indices from all overlapping cells
    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    // Remove duplicates (triangles spanning multiple cells)
    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

void WMORenderer::GroupResources::getFloorTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0 || cellFloorTriangles.empty()) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellFloorTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

void WMORenderer::GroupResources::getWallTrianglesInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0 || cellWallTriangles.empty()) return;

    float extentX = boundingBoxMax.x - boundingBoxMin.x;
    float extentY = boundingBoxMax.y - boundingBoxMin.y;
    float invCellW = gridCellsX / std::max(0.01f, extentX);
    float invCellH = gridCellsY / std::max(0.01f, extentY);

    int cellMinX = std::max(0, static_cast<int>((minX - gridOrigin.x) * invCellW));
    int cellMinY = std::max(0, static_cast<int>((minY - gridOrigin.y) * invCellH));
    int cellMaxX = std::min(gridCellsX - 1, static_cast<int>((maxX - gridOrigin.x) * invCellW));
    int cellMaxY = std::min(gridCellsY - 1, static_cast<int>((maxY - gridOrigin.y) * invCellH));

    if (cellMinX > cellMaxX || cellMinY > cellMaxY) return;

    for (int cy = cellMinY; cy <= cellMaxY; ++cy) {
        for (int cx = cellMinX; cx <= cellMaxX; ++cx) {
            const auto& cell = cellWallTriangles[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    if (cellMinX != cellMaxX || cellMinY != cellMaxY) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

std::optional<float> WMORenderer::getFloorHeight(float glX, float glY, float glZ, float* outNormalZ) const {
    // All floor caching disabled - even per-frame cache can return stale results
    // when player Z changes between queries, causing fall-through at stairs.

    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;
    float bestNormalZ = 1.0f;
    bool bestFromLowPlatform = false;

    // World-space ray: from high above, pointing straight down
    glm::vec3 worldOrigin(glX, glY, glZ + 500.0f);
    glm::vec3 worldDir(0.0f, 0.0f, -1.0f);

    // Lambda to test a single group for floor hits
    auto testGroupFloor = [&](const WMOInstance& instance, const ModelData& model,
                              const GroupResources& group,
                              const glm::vec3& localOrigin, const glm::vec3& localDir) {
        const auto& verts = group.collisionVertices;
        const auto& indices = group.collisionIndices;

        // Use unfiltered triangle list: a vertical ray naturally misses vertical
        // geometry via ray-triangle intersection, so pre-filtering by normal is
        // unnecessary and risks excluding legitimate floor geometry (steep ramps,
        // stair treads with non-trivial normals).
        group.getTrianglesInRange(
            localOrigin.x - 1.0f, localOrigin.y - 1.0f,
            localOrigin.x + 1.0f, localOrigin.y + 1.0f,
            wallTriScratch);

        for (uint32_t triStart : wallTriScratch) {
            const glm::vec3& v0 = verts[indices[triStart]];
            const glm::vec3& v1 = verts[indices[triStart + 1]];
            const glm::vec3& v2 = verts[indices[triStart + 2]];

            float t = rayTriangleIntersect(localOrigin, localDir, v0, v1, v2);
            if (t <= 0.0f) {
                t = rayTriangleIntersect(localOrigin, localDir, v0, v2, v1);
            }

            if (t > 0.0f) {
                glm::vec3 hitLocal = localOrigin + localDir * t;
                glm::vec3 hitWorld = glm::vec3(instance.modelMatrix * glm::vec4(hitLocal, 1.0f));

                float allowAbove = model.isLowPlatform ? 12.0f : 2.0f;
                if (hitWorld.z <= glZ + allowAbove) {
                    if (!bestFloor || hitWorld.z > *bestFloor) {
                        bestFloor = hitWorld.z;
                        bestFromLowPlatform = model.isLowPlatform;

                        // Compute local normal and transform to world space
                        glm::vec3 localNormal = glm::cross(v1 - v0, v2 - v0);
                        float len = glm::length(localNormal);
                        if (len > 0.001f) {
                            localNormal /= len;
                            // Ensure normal points upward
                            if (localNormal.z < 0.0f) localNormal = -localNormal;
                            glm::vec3 worldNormal = glm::normalize(
                                glm::vec3(instance.modelMatrix * glm::vec4(localNormal, 0.0f)));
                            bestNormalZ = std::abs(worldNormal.z);
                        }
                    }
                }
            }
        }
    };

    // Full scan: test all instances (active group fast path removed to fix
    // bridge clipping where early-return missed other WMO instances)
    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 8.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 10.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;
        float zMarginDown = model.isLowPlatform ? 20.0f : 2.0f;
        float zMarginUp = model.isLowPlatform ? 20.0f : 4.0f;

        // Broad-phase reject in world space to avoid expensive matrix transforms.
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - zMarginDown || glZ > instance.worldBoundsMax.z + zMarginUp) {
            continue;
        }

        // World-space pre-pass: check which groups' world XY bounds contain
        // the query point. For a vertical ray this eliminates most groups
        // before any local-space math.
        bool anyGroupOverlaps = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ - 4.0f <= gMax.z) {
                anyGroupOverlaps = true;
                break;
            }
        }
        if (!anyGroupOverlaps) continue;

        // Use cached inverse matrix
        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(worldOrigin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(worldDir, 0.0f)));

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull — vertical ray at (glX, glY)
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (glX < gMin.x || glX > gMax.x ||
                    glY < gMin.y || glY > gMax.y ||
                    glZ - 4.0f > gMax.z) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                continue;
            }

            testGroupFloor(instance, model, group, localOrigin, localDir);
        }
    }

    // Persistent grid cache disabled (see above comment about stairs fall-through)

    if (bestFloor && outNormalZ) {
        *outNormalZ = bestNormalZ;
    }

    return bestFloor;
}

bool WMORenderer::checkWallCollision(const glm::vec3& from, const glm::vec3& to, glm::vec3& adjustedPos, bool insideWMO) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool blocked = false;

    glm::vec3 moveDir = to - from;
    float moveDist = glm::length(moveDir);
    if (moveDist < 0.001f) return false;

    // Player collision parameters — WoW-style horizontal cylinder
    // Tighter radius when inside for more responsive indoor collision
    const float PLAYER_RADIUS = insideWMO ? 0.45f : 0.50f;
    const float PLAYER_HEIGHT = 2.0f;       // Cylinder height for Z bounds
    const float MAX_STEP_HEIGHT = 1.0f;     // Step-up threshold

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

        // World-space pre-pass: skip instances where no groups are near the movement
        const float wallMargin = PLAYER_RADIUS + 2.0f;
        bool anyGroupNear = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (to.x >= gMin.x - wallMargin && to.x <= gMax.x + wallMargin &&
                to.y >= gMin.y - wallMargin && to.y <= gMax.y + wallMargin &&
                to.z + PLAYER_HEIGHT >= gMin.z && to.z <= gMax.z + wallMargin) {
                anyGroupNear = true;
                break;
            }
        }
        if (!anyGroupNear) continue;

        // Transform positions into local space using cached inverse
        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localTo = glm::vec3(instance.invModelMatrix * glm::vec4(to, 1.0f));
        float localFeetZ = localTo.z;
        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (to.x < gMin.x - wallMargin || to.x > gMax.x + wallMargin ||
                    to.y < gMin.y - wallMargin || to.y > gMax.y + wallMargin ||
                    to.z > gMax.z + PLAYER_HEIGHT || to.z + PLAYER_HEIGHT < gMin.z) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            // Local-space AABB check
            float margin = PLAYER_RADIUS + 2.0f;
            if (localTo.x < group.boundingBoxMin.x - margin || localTo.x > group.boundingBoxMax.x + margin ||
                localTo.y < group.boundingBoxMin.y - margin || localTo.y > group.boundingBoxMax.y + margin ||
                localTo.z < group.boundingBoxMin.z - margin || localTo.z > group.boundingBoxMax.z + margin) {
                continue;
            }

            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            // Use spatial grid: query range covering the movement segment + player radius
            float rangeMinX = std::min(localFrom.x, localTo.x) - PLAYER_RADIUS - 1.5f;
            float rangeMinY = std::min(localFrom.y, localTo.y) - PLAYER_RADIUS - 1.5f;
            float rangeMaxX = std::max(localFrom.x, localTo.x) + PLAYER_RADIUS + 1.5f;
            float rangeMaxY = std::max(localFrom.y, localTo.y) + PLAYER_RADIUS + 1.5f;
            group.getWallTrianglesInRange(rangeMinX, rangeMinY, rangeMaxX, rangeMaxY, wallTriScratch);

            for (uint32_t triStart : wallTriScratch) {
                // Use pre-computed Z bounds for fast vertical reject
                const auto& tb = group.triBounds[triStart / 3];

                // Only collide with walls in player's vertical range
                if (tb.maxZ < localFeetZ + 0.3f) continue;
                if (tb.minZ > localFeetZ + PLAYER_HEIGHT) continue;

                // Skip low geometry that can be stepped over
                if (tb.maxZ <= localFeetZ + MAX_STEP_HEIGHT) continue;

                // Skip very short vertical surfaces (stair risers)
                float triHeight = tb.maxZ - tb.minZ;
                if (triHeight < 1.0f && tb.maxZ <= localFeetZ + 1.2f) continue;

                const glm::vec3& v0 = verts[indices[triStart]];
                const glm::vec3& v1 = verts[indices[triStart + 1]];
                const glm::vec3& v2 = verts[indices[triStart + 2]];

                // Triangle normal for swept test and push fallback
                glm::vec3 edge1 = v1 - v0;
                glm::vec3 edge2 = v2 - v0;
                glm::vec3 normal = glm::cross(edge1, edge2);
                float normalLen = glm::length(normal);
                if (normalLen < 0.001f) continue;
                normal /= normalLen;

                // Recompute plane distances with current (possibly pushed) localTo
                float fromDist = glm::dot(localFrom - v0, normal);
                float toDist = glm::dot(localTo - v0, normal);

                // Swept test: prevent tunneling when crossing a wall between frames
                if ((fromDist > PLAYER_RADIUS && toDist < -PLAYER_RADIUS) ||
                    (fromDist < -PLAYER_RADIUS && toDist > PLAYER_RADIUS)) {
                    float denom = (fromDist - toDist);
                    if (std::abs(denom) > 1e-6f) {
                        float tHit = fromDist / denom;
                        if (tHit >= 0.0f && tHit <= 1.0f) {
                            glm::vec3 hitPoint = localFrom + (localTo - localFrom) * tHit;
                            glm::vec3 hitClosest = closestPointOnTriangle(hitPoint, v0, v1, v2);
                            float hitErrSq = glm::dot(hitClosest - hitPoint, hitClosest - hitPoint);
                            if (hitErrSq <= 0.15f * 0.15f) {
                                float side = fromDist > 0.0f ? 1.0f : -1.0f;
                                glm::vec3 safeLocal = hitPoint + normal * side * (PLAYER_RADIUS + 0.05f);
                                glm::vec3 pushLocal(safeLocal.x - localTo.x, safeLocal.y - localTo.y, 0.0f);
                                // Cap swept pushback so walls don't shove the player violently
                                float pushLen = glm::length(glm::vec2(pushLocal.x, pushLocal.y));
                                const float MAX_SWEPT_PUSH = 0.15f;
                                if (pushLen > MAX_SWEPT_PUSH) {
                                    float scale = MAX_SWEPT_PUSH / pushLen;
                                    pushLocal.x *= scale;
                                    pushLocal.y *= scale;
                                }
                                localTo.x += pushLocal.x;
                                localTo.y += pushLocal.y;
                                glm::vec3 pushWorld = glm::vec3(instance.modelMatrix * glm::vec4(pushLocal, 0.0f));
                                adjustedPos.x += pushWorld.x;
                                adjustedPos.y += pushWorld.y;
                                blocked = true;
                                continue;
                            }
                        }
                    }
                }

                // Horizontal cylinder collision: closest point + horizontal distance
                glm::vec3 closest = closestPointOnTriangle(localTo, v0, v1, v2);
                glm::vec3 delta = localTo - closest;
                float horizDist = glm::length(glm::vec2(delta.x, delta.y));

                if (horizDist <= PLAYER_RADIUS) {
                    // Skip floor-like surfaces — grounding handles them, not wall collision
                    float absNz = std::abs(normal.z);
                    if (absNz >= 0.35f) continue;

                    const float SKIN = 0.005f;        // small separation so we don't re-collide immediately
                    // Stronger push when inside WMO for more responsive indoor collision
                    const float MAX_PUSH = insideWMO ? 0.12f : 0.08f;
                    float penetration = (PLAYER_RADIUS - horizDist);
                    float pushDist = glm::clamp(penetration + SKIN, 0.0f, MAX_PUSH);
                    glm::vec2 pushDir2;
                    if (horizDist > 1e-4f) {
                        pushDir2 = glm::normalize(glm::vec2(delta.x, delta.y));
                    } else {
                        glm::vec2 n2(normal.x, normal.y);
                        float n2Len = glm::length(n2);
                        if (n2Len < 1e-4f) continue;
                        pushDir2 = n2 / n2Len;
                    }
                    glm::vec3 pushLocal(pushDir2.x * pushDist, pushDir2.y * pushDist, 0.0f);

                    localTo.x += pushLocal.x;
                    localTo.y += pushLocal.y;
                    glm::vec3 pushWorld = glm::vec3(instance.modelMatrix * glm::vec4(pushLocal, 0.0f));
                    adjustedPos.x += pushWorld.x;
                    adjustedPos.y += pushWorld.y;
                    blocked = true;
                }
            }
        }
    }

    return blocked;
}

void WMORenderer::updateActiveGroup(float glX, float glY, float glZ) {
    // If active group is still valid, check if player is still inside it
    if (activeGroup_.isValid() && activeGroup_.instanceIdx < instances.size()) {
        const auto& instance = instances[activeGroup_.instanceIdx];
        if (instance.modelId == activeGroup_.modelId) {
            auto it = loadedModels.find(instance.modelId);
            if (it != loadedModels.end()) {
                const ModelData& model = it->second;
                glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

                // Still inside active group?
                if (activeGroup_.groupIdx >= 0 && static_cast<size_t>(activeGroup_.groupIdx) < model.groups.size()) {
                    const auto& group = model.groups[activeGroup_.groupIdx];
                    if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                        localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                        localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                        return;  // Still in same group
                    }
                }

                // Check portal-neighbor groups
                for (uint32_t ngi : activeGroup_.neighborGroups) {
                    if (ngi < model.groups.size()) {
                        const auto& group = model.groups[ngi];
                        if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                            localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                            localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
                            // Moved to a neighbor group — update
                            activeGroup_.groupIdx = static_cast<int32_t>(ngi);
                            // Rebuild neighbors for new group
                            activeGroup_.neighborGroups.clear();
                            if (ngi < model.groupPortalRefs.size()) {
                                auto [portalStart, portalCount] = model.groupPortalRefs[ngi];
                                for (uint16_t pi = 0; pi < portalCount; pi++) {
                                    uint16_t refIdx = portalStart + pi;
                                    if (refIdx < model.portalRefs.size()) {
                                        uint32_t tgt = model.portalRefs[refIdx].groupIndex;
                                        if (tgt < model.groups.size()) {
                                            activeGroup_.neighborGroups.push_back(tgt);
                                        }
                                    }
                                }
                            }
                            return;
                        }
                    }
                }
            }
        }
    }

    // Full scan: find which instance/group contains the player
    activeGroup_.invalidate();

    glm::vec3 queryMin(glX - 0.5f, glY - 0.5f, glZ - 0.5f);
    glm::vec3 queryMax(glX + 0.5f, glY + 0.5f, glZ + 0.5f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z || glZ > instance.worldBoundsMax.z) {
            continue;
        }

        auto it = loadedModels.find(instance.modelId);
        if (it == loadedModels.end()) continue;

        const ModelData& model = it->second;
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        int gi = findContainingGroup(model, localPos);
        if (gi >= 0) {
            activeGroup_.instanceIdx = static_cast<uint32_t>(idx);
            activeGroup_.modelId = instance.modelId;
            activeGroup_.groupIdx = gi;

            // Build neighbor list from portal refs
            activeGroup_.neighborGroups.clear();
            uint32_t groupIdx = static_cast<uint32_t>(gi);
            if (groupIdx < model.groupPortalRefs.size()) {
                auto [portalStart, portalCount] = model.groupPortalRefs[groupIdx];
                for (uint16_t pi = 0; pi < portalCount; pi++) {
                    uint16_t refIdx = portalStart + pi;
                    if (refIdx < model.portalRefs.size()) {
                        uint32_t tgt = model.portalRefs[refIdx].groupIndex;
                        if (tgt < model.groups.size()) {
                            activeGroup_.neighborGroups.push_back(tgt);
                        }
                    }
                }
            }
            return;
        }
    }
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

        // World-space pre-check: skip instance if no group's world bounds contain point
        bool anyGroupContains = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ >= gMin.z && glZ <= gMax.z) {
                anyGroupContains = true;
                break;
            }
        }
        if (!anyGroupContains) continue;

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));
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

bool WMORenderer::isInsideInteriorWMO(float glX, float glY, float glZ) const {
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

        bool anyGroupContains = false;
        for (size_t gi = 0; gi < model.groups.size() && gi < instance.worldGroupBounds.size(); ++gi) {
            const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
            if (glX >= gMin.x && glX <= gMax.x &&
                glY >= gMin.y && glY <= gMax.y &&
                glZ >= gMin.z && glZ <= gMax.z) {
                anyGroupContains = true;
                break;
            }
        }
        if (!anyGroupContains) continue;

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));
        for (const auto& group : model.groups) {
            if (!(group.groupFlags & 0x2000)) continue; // Skip exterior groups
            if (localPos.x >= group.boundingBoxMin.x && localPos.x <= group.boundingBoxMax.x &&
                localPos.y >= group.boundingBoxMin.y && localPos.y <= group.boundingBoxMax.y &&
                localPos.z >= group.boundingBoxMin.z && localPos.z <= group.boundingBoxMax.z) {
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
    // Wall list pre-filters at abs(normal.z) < 0.55, but for camera raycast we want
    // a stricter threshold to avoid ramp/stair geometry pulling the camera in.
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

        for (size_t gi = 0; gi < model.groups.size(); ++gi) {
            // World-space group cull — skip groups whose world AABB doesn't intersect the ray
            if (gi < instance.worldGroupBounds.size()) {
                const auto& [gMin, gMax] = instance.worldGroupBounds[gi];
                if (!rayIntersectsAABB(origin, direction, gMin, gMax)) {
                    continue;
                }
            }

            const auto& group = model.groups[gi];
            // Local-space AABB cull
            if (!rayIntersectsAABB(localOrigin, localDir, group.boundingBoxMin, group.boundingBoxMax)) {
                continue;
            }

            // Narrow-phase: triangle raycast using spatial grid (wall-only).
            const auto& verts = group.collisionVertices;
            const auto& indices = group.collisionIndices;

            // Compute local-space ray endpoint and query grid for XY range
            glm::vec3 localEnd = localOrigin + localDir * (closestHit / glm::length(
                glm::vec3(instance.modelMatrix * glm::vec4(localDir, 0.0f))));
            float rMinX = std::min(localOrigin.x, localEnd.x) - 1.0f;
            float rMinY = std::min(localOrigin.y, localEnd.y) - 1.0f;
            float rMaxX = std::max(localOrigin.x, localEnd.x) + 1.0f;
            float rMaxY = std::max(localOrigin.y, localEnd.y) + 1.0f;
            group.getWallTrianglesInRange(rMinX, rMinY, rMaxX, rMaxY, wallTriScratch);

            for (uint32_t triStart : wallTriScratch) {
                const glm::vec3& v0 = verts[indices[triStart]];
                const glm::vec3& v1 = verts[indices[triStart + 1]];
                const glm::vec3& v2 = verts[indices[triStart + 2]];
                glm::vec3 triNormal = glm::cross(v1 - v0, v2 - v0);
                float normalLenSq = glm::dot(triNormal, triNormal);
                if (normalLenSq < 1e-8f) {
                    continue;
                }
                triNormal /= std::sqrt(normalLenSq);
                // Wall list pre-filters at 0.55; apply stricter camera threshold
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
