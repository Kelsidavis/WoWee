#pragma once

#include "pipeline/terrain_mesh.hpp"
#include "rendering/shader.hpp"
#include "rendering/texture.hpp"
#include "rendering/camera.hpp"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>
#include <string>

namespace wowee {

// Forward declarations
namespace pipeline { class AssetManager; }

namespace rendering {

class Frustum;

/**
 * GPU-side terrain chunk data
 */
struct TerrainChunkGPU {
    GLuint vao = 0;           // Vertex array object
    GLuint vbo = 0;           // Vertex buffer
    GLuint ibo = 0;           // Index buffer
    uint32_t indexCount = 0;  // Number of indices to draw

    // Texture IDs for this chunk
    GLuint baseTexture = 0;
    std::vector<GLuint> layerTextures;
    std::vector<GLuint> alphaTextures;

    // World position for culling
    float worldX = 0.0f;
    float worldY = 0.0f;
    float worldZ = 0.0f;

    // Owning tile coordinates (for per-tile removal)
    int tileX = -1, tileY = -1;

    // Bounding sphere for frustum culling
    float boundingSphereRadius = 0.0f;
    glm::vec3 boundingSphereCenter = glm::vec3(0.0f);

    bool isValid() const { return vao != 0 && vbo != 0 && ibo != 0; }
};

/**
 * Terrain renderer
 *
 * Handles uploading terrain meshes to GPU and rendering them
 */
class TerrainRenderer {
public:
    TerrainRenderer();
    ~TerrainRenderer();

    /**
     * Initialize terrain renderer
     * @param assetManager Asset manager for loading textures
     */
    bool initialize(pipeline::AssetManager* assetManager);

    /**
     * Shutdown and cleanup GPU resources
     */
    void shutdown();

    /**
     * Load terrain mesh and upload to GPU
     * @param mesh Terrain mesh to load
     * @param texturePaths Texture file paths from ADT
     * @param tileX Tile X coordinate for tracking ownership (-1 = untracked)
     * @param tileY Tile Y coordinate for tracking ownership (-1 = untracked)
     */
    bool loadTerrain(const pipeline::TerrainMesh& mesh,
                     const std::vector<std::string>& texturePaths,
                     int tileX = -1, int tileY = -1);

    /**
     * Remove all chunks belonging to a specific tile
     * @param tileX Tile X coordinate
     * @param tileY Tile Y coordinate
     */
    void removeTile(int tileX, int tileY);

    /**
     * Render loaded terrain
     * @param camera Camera for view/projection matrices
     */
    void render(const Camera& camera);

    /**
     * Clear all loaded terrain
     */
    void clear();

    /**
     * Set lighting parameters
     */
    void setLighting(const float lightDir[3], const float lightColor[3],
                     const float ambientColor[3]);

    /**
     * Set fog parameters
     */
    void setFog(const float fogColor[3], float fogStart, float fogEnd);

    /**
     * Enable/disable wireframe rendering
     */
    void setWireframe(bool enabled) { wireframe = enabled; }

    /**
     * Enable/disable frustum culling
     */
    void setFrustumCulling(bool enabled) { frustumCullingEnabled = enabled; }

    /**
     * Enable/disable distance fog
     */
    void setFogEnabled(bool enabled) { fogEnabled = enabled; }
    bool isFogEnabled() const { return fogEnabled; }

    /**
     * Render terrain geometry into shadow depth map
     */
    void renderShadow(GLuint shaderProgram);

    /**
     * Set shadow map for receiving shadows
     */
    void setShadowMap(GLuint depthTex, const glm::mat4& lightSpaceMat) {
        shadowDepthTex = depthTex; lightSpaceMatrix = lightSpaceMat; shadowEnabled = true;
    }
    void clearShadowMap() { shadowEnabled = false; }

    /**
     * Get statistics
     */
    int getChunkCount() const { return static_cast<int>(chunks.size()); }
    int getRenderedChunkCount() const { return renderedChunks; }
    int getCulledChunkCount() const { return culledChunks; }
    int getTriangleCount() const;

private:
    /**
     * Upload single chunk to GPU
     */
    TerrainChunkGPU uploadChunk(const pipeline::ChunkMesh& chunk);

    /**
     * Load texture from asset manager
     */
    GLuint loadTexture(const std::string& path);

    /**
     * Create alpha texture from raw alpha data
     */
    GLuint createAlphaTexture(const std::vector<uint8_t>& alphaData);

    /**
     * Check if chunk is in view frustum
     */
    bool isChunkVisible(const TerrainChunkGPU& chunk, const Frustum& frustum);

    /**
     * Calculate bounding sphere for chunk
     */
    void calculateBoundingSphere(TerrainChunkGPU& chunk, const pipeline::ChunkMesh& meshChunk);

    pipeline::AssetManager* assetManager = nullptr;
    std::unique_ptr<Shader> shader;

    // Loaded terrain chunks
    std::vector<TerrainChunkGPU> chunks;

    // Texture cache (path -> GL texture ID)
    std::unordered_map<std::string, GLuint> textureCache;

    // Lighting parameters
    float lightDir[3] = {-0.5f, -1.0f, -0.5f};
    float lightColor[3] = {1.0f, 1.0f, 0.9f};
    float ambientColor[3] = {0.3f, 0.3f, 0.35f};

    // Fog parameters
    float fogColor[3] = {0.5f, 0.6f, 0.7f};
    float fogStart = 400.0f;
    float fogEnd = 800.0f;

    // Rendering state
    bool wireframe = false;
    bool frustumCullingEnabled = true;
    bool fogEnabled = true;
    int renderedChunks = 0;
    int culledChunks = 0;

    // Default white texture (fallback)
    GLuint whiteTexture = 0;

    // Shadow mapping (receiving)
    GLuint shadowDepthTex = 0;
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
    bool shadowEnabled = false;
};

} // namespace rendering
} // namespace wowee
