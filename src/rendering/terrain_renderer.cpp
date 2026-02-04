#include "rendering/terrain_renderer.hpp"
#include "rendering/texture.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <limits>

namespace wowee {
namespace rendering {

TerrainRenderer::TerrainRenderer() {
}

TerrainRenderer::~TerrainRenderer() {
    shutdown();
}

bool TerrainRenderer::initialize(pipeline::AssetManager* assets) {
    assetManager = assets;

    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    LOG_INFO("Initializing terrain renderer");

    // Load terrain shader
    shader = std::make_unique<Shader>();
    if (!shader->loadFromFile("assets/shaders/terrain.vert", "assets/shaders/terrain.frag")) {
        LOG_ERROR("Failed to load terrain shader");
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

    LOG_INFO("Terrain renderer initialized");
    return true;
}

void TerrainRenderer::shutdown() {
    LOG_INFO("Shutting down terrain renderer");

    clear();

    // Delete white texture
    if (whiteTexture) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }

    // Delete cached textures
    for (auto& pair : textureCache) {
        glDeleteTextures(1, &pair.second);
    }
    textureCache.clear();

    shader.reset();
}

bool TerrainRenderer::loadTerrain(const pipeline::TerrainMesh& mesh,
                                   const std::vector<std::string>& texturePaths,
                                   int tileX, int tileY) {
    LOG_INFO("Loading terrain mesh: ", mesh.validChunkCount, " chunks");

    // Upload each chunk to GPU
    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            const auto& chunk = mesh.getChunk(x, y);

            if (!chunk.isValid()) {
                continue;
            }

            TerrainChunkGPU gpuChunk = uploadChunk(chunk);

            if (!gpuChunk.isValid()) {
                LOG_WARNING("Failed to upload chunk [", x, ",", y, "]");
                continue;
            }

            // Calculate bounding sphere for frustum culling
            calculateBoundingSphere(gpuChunk, chunk);

            // Load textures for this chunk
            if (!chunk.layers.empty()) {
                // Base layer (always present)
                uint32_t baseTexId = chunk.layers[0].textureId;
                if (baseTexId < texturePaths.size()) {
                    gpuChunk.baseTexture = loadTexture(texturePaths[baseTexId]);
                } else {
                    gpuChunk.baseTexture = whiteTexture;
                }

                // Additional layers (with alpha blending)
                for (size_t i = 1; i < chunk.layers.size() && i < 4; i++) {
                    const auto& layer = chunk.layers[i];

                    // Load layer texture
                    GLuint layerTex = whiteTexture;
                    if (layer.textureId < texturePaths.size()) {
                        layerTex = loadTexture(texturePaths[layer.textureId]);
                    }
                    gpuChunk.layerTextures.push_back(layerTex);

                    // Create alpha texture
                    GLuint alphaTex = 0;
                    if (!layer.alphaData.empty()) {
                        alphaTex = createAlphaTexture(layer.alphaData);
                    }
                    gpuChunk.alphaTextures.push_back(alphaTex);
                }
            } else {
                // No layers, use default white texture
                gpuChunk.baseTexture = whiteTexture;
            }

            gpuChunk.tileX = tileX;
            gpuChunk.tileY = tileY;
            chunks.push_back(gpuChunk);
        }
    }

    LOG_INFO("Loaded ", chunks.size(), " terrain chunks to GPU");
    return !chunks.empty();
}

TerrainChunkGPU TerrainRenderer::uploadChunk(const pipeline::ChunkMesh& chunk) {
    TerrainChunkGPU gpuChunk;

    gpuChunk.worldX = chunk.worldX;
    gpuChunk.worldY = chunk.worldY;
    gpuChunk.worldZ = chunk.worldZ;
    gpuChunk.indexCount = static_cast<uint32_t>(chunk.indices.size());

    // Debug: verify Z values in uploaded vertices
    static int uploadLogCount = 0;
    if (uploadLogCount < 3 && !chunk.vertices.empty()) {
        float minZ = 999999.0f, maxZ = -999999.0f;
        for (const auto& v : chunk.vertices) {
            if (v.position[2] < minZ) minZ = v.position[2];
            if (v.position[2] > maxZ) maxZ = v.position[2];
        }
        LOG_DEBUG("GPU upload Z range: [", minZ, ", ", maxZ, "] delta=", maxZ - minZ);
        uploadLogCount++;
    }

    // Create VAO
    glGenVertexArrays(1, &gpuChunk.vao);
    glBindVertexArray(gpuChunk.vao);

    // Create VBO
    glGenBuffers(1, &gpuChunk.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, gpuChunk.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 chunk.vertices.size() * sizeof(pipeline::TerrainVertex),
                 chunk.vertices.data(),
                 GL_STATIC_DRAW);

    // Create IBO
    glGenBuffers(1, &gpuChunk.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuChunk.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 chunk.indices.size() * sizeof(pipeline::TerrainIndex),
                 chunk.indices.data(),
                 GL_STATIC_DRAW);

    // Set up vertex attributes
    // Location 0: Position (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                         sizeof(pipeline::TerrainVertex),
                         (void*)offsetof(pipeline::TerrainVertex, position));

    // Location 1: Normal (vec3)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                         sizeof(pipeline::TerrainVertex),
                         (void*)offsetof(pipeline::TerrainVertex, normal));

    // Location 2: TexCoord (vec2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                         sizeof(pipeline::TerrainVertex),
                         (void*)offsetof(pipeline::TerrainVertex, texCoord));

    // Location 3: LayerUV (vec2)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE,
                         sizeof(pipeline::TerrainVertex),
                         (void*)offsetof(pipeline::TerrainVertex, layerUV));

    glBindVertexArray(0);

    return gpuChunk;
}

GLuint TerrainRenderer::loadTexture(const std::string& path) {
    // Check cache first
    auto it = textureCache.find(path);
    if (it != textureCache.end()) {
        return it->second;
    }

    // Load BLP texture
    pipeline::BLPImage blp = assetManager->loadTexture(path);
    if (!blp.isValid()) {
        LOG_WARNING("Failed to load texture: ", path);
        textureCache[path] = whiteTexture;
        return whiteTexture;
    }

    // Create OpenGL texture
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Upload texture data (BLP loader outputs RGBA8)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 blp.width, blp.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blp.data.data());

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Generate mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);
    applyAnisotropicFiltering();

    glBindTexture(GL_TEXTURE_2D, 0);

    // Cache texture
    textureCache[path] = textureID;

    LOG_DEBUG("Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return textureID;
}

GLuint TerrainRenderer::createAlphaTexture(const std::vector<uint8_t>& alphaData) {
    if (alphaData.empty()) {
        return 0;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Alpha data is always expanded to 4096 bytes (64x64 at 8-bit) by terrain_mesh
    int width = 64;
    int height = static_cast<int>(alphaData.size()) / 64;

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED,
                 width, height, 0,
                 GL_RED, GL_UNSIGNED_BYTE, alphaData.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);

    return textureID;
}

void TerrainRenderer::render(const Camera& camera) {
    if (chunks.empty() || !shader) {
        return;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Disable backface culling temporarily to debug flashing
    glDisable(GL_CULL_FACE);
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);

    // Wireframe mode
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Use shader
    shader->use();

    // Bind sampler uniforms to texture units (constant, only needs to be set once per use)
    shader->setUniform("uBaseTexture", 0);
    shader->setUniform("uLayer1Texture", 1);
    shader->setUniform("uLayer2Texture", 2);
    shader->setUniform("uLayer3Texture", 3);
    shader->setUniform("uLayer1Alpha", 4);
    shader->setUniform("uLayer2Alpha", 5);
    shader->setUniform("uLayer3Alpha", 6);

    // Set view/projection matrices
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();
    glm::mat4 model = glm::mat4(1.0f);

    shader->setUniform("uModel", model);
    shader->setUniform("uView", view);
    shader->setUniform("uProjection", projection);

    // Set lighting
    shader->setUniform("uLightDir", glm::vec3(lightDir[0], lightDir[1], lightDir[2]));
    shader->setUniform("uLightColor", glm::vec3(lightColor[0], lightColor[1], lightColor[2]));
    shader->setUniform("uSpecularIntensity", 0.5f);
    shader->setUniform("uAmbientColor", glm::vec3(ambientColor[0], ambientColor[1], ambientColor[2]));

    // Set camera position
    glm::vec3 camPos = camera.getPosition();
    shader->setUniform("uViewPos", camPos);

    // Set fog (disable by setting very far distances)
    shader->setUniform("uFogColor", glm::vec3(fogColor[0], fogColor[1], fogColor[2]));
    if (fogEnabled) {
        shader->setUniform("uFogStart", fogStart);
        shader->setUniform("uFogEnd", fogEnd);
    } else {
        shader->setUniform("uFogStart", 100000.0f);  // Very far
        shader->setUniform("uFogEnd", 100001.0f);    // Effectively disabled
    }

    // Extract frustum for culling
    Frustum frustum;
    if (frustumCullingEnabled) {
        glm::mat4 viewProj = projection * view;
        frustum.extractFromMatrix(viewProj);
    }

    // Render each chunk — track last-bound textures to skip redundant binds
    renderedChunks = 0;
    culledChunks = 0;
    GLuint lastBound[7] = {0, 0, 0, 0, 0, 0, 0};
    int lastLayerConfig = -1; // track hasLayer1|hasLayer2|hasLayer3 bitmask

    for (const auto& chunk : chunks) {
        if (!chunk.isValid()) {
            continue;
        }

        // Frustum culling
        if (frustumCullingEnabled && !isChunkVisible(chunk, frustum)) {
            culledChunks++;
            continue;
        }

        // Bind base texture (slot 0) — skip if same as last chunk
        if (chunk.baseTexture != lastBound[0]) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, chunk.baseTexture);
            lastBound[0] = chunk.baseTexture;
        }

        // Layer configuration
        bool hasLayer1 = chunk.layerTextures.size() > 0;
        bool hasLayer2 = chunk.layerTextures.size() > 1;
        bool hasLayer3 = chunk.layerTextures.size() > 2;
        int layerConfig = (hasLayer1 ? 1 : 0) | (hasLayer2 ? 2 : 0) | (hasLayer3 ? 4 : 0);

        if (layerConfig != lastLayerConfig) {
            shader->setUniform("uHasLayer1", hasLayer1 ? 1 : 0);
            shader->setUniform("uHasLayer2", hasLayer2 ? 1 : 0);
            shader->setUniform("uHasLayer3", hasLayer3 ? 1 : 0);
            lastLayerConfig = layerConfig;
        }

        if (hasLayer1) {
            if (chunk.layerTextures[0] != lastBound[1]) {
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, chunk.layerTextures[0]);
                lastBound[1] = chunk.layerTextures[0];
            }
            if (chunk.alphaTextures[0] != lastBound[4]) {
                glActiveTexture(GL_TEXTURE4);
                glBindTexture(GL_TEXTURE_2D, chunk.alphaTextures[0]);
                lastBound[4] = chunk.alphaTextures[0];
            }
        }

        if (hasLayer2) {
            if (chunk.layerTextures[1] != lastBound[2]) {
                glActiveTexture(GL_TEXTURE2);
                glBindTexture(GL_TEXTURE_2D, chunk.layerTextures[1]);
                lastBound[2] = chunk.layerTextures[1];
            }
            if (chunk.alphaTextures[1] != lastBound[5]) {
                glActiveTexture(GL_TEXTURE5);
                glBindTexture(GL_TEXTURE_2D, chunk.alphaTextures[1]);
                lastBound[5] = chunk.alphaTextures[1];
            }
        }

        if (hasLayer3) {
            if (chunk.layerTextures[2] != lastBound[3]) {
                glActiveTexture(GL_TEXTURE3);
                glBindTexture(GL_TEXTURE_2D, chunk.layerTextures[2]);
                lastBound[3] = chunk.layerTextures[2];
            }
            if (chunk.alphaTextures[2] != lastBound[6]) {
                glActiveTexture(GL_TEXTURE6);
                glBindTexture(GL_TEXTURE_2D, chunk.alphaTextures[2]);
                lastBound[6] = chunk.alphaTextures[2];
            }
        }

        // Draw chunk
        glBindVertexArray(chunk.vao);
        glDrawElements(GL_TRIANGLES, chunk.indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        renderedChunks++;
    }

    // Reset wireframe
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void TerrainRenderer::removeTile(int tileX, int tileY) {
    int removed = 0;
    auto it = chunks.begin();
    while (it != chunks.end()) {
        if (it->tileX == tileX && it->tileY == tileY) {
            if (it->vao) glDeleteVertexArrays(1, &it->vao);
            if (it->vbo) glDeleteBuffers(1, &it->vbo);
            if (it->ibo) glDeleteBuffers(1, &it->ibo);
            for (GLuint alpha : it->alphaTextures) {
                if (alpha) glDeleteTextures(1, &alpha);
            }
            it = chunks.erase(it);
            removed++;
        } else {
            ++it;
        }
    }
    if (removed > 0) {
        LOG_DEBUG("Removed ", removed, " terrain chunks for tile [", tileX, ",", tileY, "]");
    }
}

void TerrainRenderer::clear() {
    // Delete all GPU resources
    for (auto& chunk : chunks) {
        if (chunk.vao) glDeleteVertexArrays(1, &chunk.vao);
        if (chunk.vbo) glDeleteBuffers(1, &chunk.vbo);
        if (chunk.ibo) glDeleteBuffers(1, &chunk.ibo);

        // Delete alpha textures (not cached)
        for (GLuint alpha : chunk.alphaTextures) {
            if (alpha) glDeleteTextures(1, &alpha);
        }
    }

    chunks.clear();
    renderedChunks = 0;
}

void TerrainRenderer::setLighting(const float lightDirIn[3], const float lightColorIn[3],
                                   const float ambientColorIn[3]) {
    lightDir[0] = lightDirIn[0];
    lightDir[1] = lightDirIn[1];
    lightDir[2] = lightDirIn[2];

    lightColor[0] = lightColorIn[0];
    lightColor[1] = lightColorIn[1];
    lightColor[2] = lightColorIn[2];

    ambientColor[0] = ambientColorIn[0];
    ambientColor[1] = ambientColorIn[1];
    ambientColor[2] = ambientColorIn[2];
}

void TerrainRenderer::setFog(const float fogColorIn[3], float fogStartIn, float fogEndIn) {
    fogColor[0] = fogColorIn[0];
    fogColor[1] = fogColorIn[1];
    fogColor[2] = fogColorIn[2];
    fogStart = fogStartIn;
    fogEnd = fogEndIn;
}

int TerrainRenderer::getTriangleCount() const {
    int total = 0;
    for (const auto& chunk : chunks) {
        total += chunk.indexCount / 3;
    }
    return total;
}

bool TerrainRenderer::isChunkVisible(const TerrainChunkGPU& chunk, const Frustum& frustum) {
    // Test bounding sphere against frustum
    return frustum.intersectsSphere(chunk.boundingSphereCenter, chunk.boundingSphereRadius);
}

void TerrainRenderer::calculateBoundingSphere(TerrainChunkGPU& gpuChunk,
                                                const pipeline::ChunkMesh& meshChunk) {
    if (meshChunk.vertices.empty()) {
        gpuChunk.boundingSphereRadius = 0.0f;
        gpuChunk.boundingSphereCenter = glm::vec3(0.0f);
        return;
    }

    // Calculate AABB first
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(std::numeric_limits<float>::lowest());

    for (const auto& vertex : meshChunk.vertices) {
        glm::vec3 pos(vertex.position[0], vertex.position[1], vertex.position[2]);
        min = glm::min(min, pos);
        max = glm::max(max, pos);
    }

    // Center is midpoint of AABB
    gpuChunk.boundingSphereCenter = (min + max) * 0.5f;

    // Radius is distance from center to furthest vertex
    float maxDistSq = 0.0f;
    for (const auto& vertex : meshChunk.vertices) {
        glm::vec3 pos(vertex.position[0], vertex.position[1], vertex.position[2]);
        glm::vec3 diff = pos - gpuChunk.boundingSphereCenter;
        float distSq = glm::dot(diff, diff);
        maxDistSq = std::max(maxDistSq, distSq);
    }

    gpuChunk.boundingSphereRadius = std::sqrt(maxDistSq);
}

} // namespace rendering
} // namespace wowee
