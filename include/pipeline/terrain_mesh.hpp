#pragma once

#include "core/coordinates.hpp"
#include "pipeline/adt_loader.hpp"

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace wowee {
namespace pipeline {

/**
 * Vertex format for terrain rendering
 */
struct TerrainVertex {
    float position[3];     // X, Y, Z
    float normal[3];       // Normal vector
    float texCoord[2];     // Base texture coordinates
    float layerUV[2];      // Layer texture coordinates
    uint8_t chunkIndex = 0;    // Which chunk this vertex belongs to

    TerrainVertex()  {
        position[0] = position[1] = position[2] = 0.0f;
        normal[0] = normal[1] = normal[2] = 0.0f;
        texCoord[0] = texCoord[1] = 0.0f;
        layerUV[0] = layerUV[1] = 0.0f;
    }
};

/**
 * Triangle index (3 vertices)
 */
using TerrainIndex = uint32_t;

/**
 * Renderable terrain mesh for a single map chunk
 */
struct ChunkMesh {
    std::vector<TerrainVertex> vertices;
    std::vector<TerrainIndex> indices;

    // Chunk position in world space
    float worldX;
    float worldY;
    float worldZ;

    // Chunk grid coordinates
    int chunkX;
    int chunkY;

    // Texture layer info
    struct LayerInfo {
        uint32_t textureId;
        uint32_t flags;
        std::vector<uint8_t> alphaData;  // 64x64 alpha map
    };
    std::vector<LayerInfo> layers;

    [[nodiscard]] bool isValid() const { return !vertices.empty() && !indices.empty(); }
    [[nodiscard]] size_t getVertexCount() const { return vertices.size(); }
    [[nodiscard]] size_t getTriangleCount() const { return indices.size() / 3; }
};

/**
 * Complete terrain tile mesh (16x16 chunks)
 */
struct TerrainMesh {
    std::array<ChunkMesh, 256> chunks;  // 16x16 grid
    std::vector<std::string> textures;   // Texture filenames

    int validChunkCount = 0;

    [[nodiscard]] const ChunkMesh& getChunk(int x, int y) const { return chunks[y * 16 + x]; }
    ChunkMesh& getChunk(int x, int y) { return chunks[y * 16 + x]; }
};

/**
 * Terrain mesh generator
 *
 * Converts ADT heightmap data into renderable triangle meshes
 */
class TerrainMeshGenerator {
public:
    /**
     * Generate terrain mesh from ADT data
     * @param terrain Loaded ADT terrain data
     * @return Generated mesh (check validChunkCount)
     */
    static TerrainMesh generate(const ADTTerrain& terrain);

    /**
     * Where a point inside a chunk sits in the world.
     *
     * `fracX` and `fracY` are cell coordinates within the chunk, 0 to 8, and
     * need not be whole: the height between the four surrounding grid points
     * is interpolated bilinearly. This is what the ground-clutter and doodad
     * scatterers ask when they drop something at a random spot on a chunk,
     * and both had their own copy of it.
     *
     * The axes cross on purpose. A chunk's world X runs against the grid's
     * Y and its world Y against the grid's X, which is the terrain axis
     * pairing this codebase uses throughout; swapping them back lays every
     * scattered object out mirrored, which reads as the doodad data being
     * wrong rather than the sampling.
     *
     * Out-of-range coordinates clamp to the chunk rather than reading past
     * it, so a caller that rounds slightly past 8 gets the edge height
     * instead of a zero that would bury the object.
     */
    static glm::vec3 chunkSurfacePoint(const float chunkPosition[3],
                                       const HeightMap& heightMap,
                                       float fracX, float fracY, float unitSize);

    /**
     * Where a world position falls inside a chunk, in the 0..8 grid fractions
     * chunkSurfacePoint and isHole take. False if the position is outside it.
     *
     * The axes cross here too: world X gives fracY and world Y gives fracX.
     *
     * Separated so a caller walking many points can test the chunk it already
     * has before searching for another. The search is a tile lookup and a 3x3
     * probe, and running it per sample dominated grass generation - a hundred
     * thousand candidates in a row nearly all land in the chunk the last one
     * did.
     */
    static bool chunkFractionsAt(const float chunkPosition[3], float glX, float glY,
                                 float unitSize, float& fracX, float& fracY);

private:
    /**
     * Generate mesh for a single map chunk
     */
    static ChunkMesh generateChunkMesh(const MapChunk& chunk, int chunkX, int chunkY, int tileX, int tileY);

    /**
     * Generate vertices from heightmap
     * WoW heightmap layout: 9x9 outer + 8x8 inner vertices (145 total)
     */
    static std::vector<TerrainVertex> generateVertices(const MapChunk& chunk, int chunkX, int chunkY, int tileX, int tileY);

    /**
     * Generate triangle indices
     * Creates triangles that connect the heightmap vertices
     * Skips quads that are marked as holes in the chunk
     */
    static std::vector<TerrainIndex> generateIndices(const MapChunk& chunk);


    /**
     * Convert WoW's compressed normals to float
     */
    static void decompressNormal(const int8_t* compressedNormal, float* normal);

    /**
     * Get height at grid position from WoW's 9x9+8x8 layout
     */
    static float getHeightAt(const HeightMap& heightMap, int x, int y);



    // Terrain constants
    // WoW terrain: 64x64 tiles, each tile = 533.33 yards, each chunk = 33.33 yards
    // One ADT tile, from the coordinate header rather than spelled again:
    // the value is a truncation of 1600/3, and a second spelling of a
    // truncation is a second answer to where a tile boundary is.
    static constexpr float TILE_SIZE = core::coords::TILE_SIZE;
    static constexpr float CHUNK_SIZE = TILE_SIZE / 16.0f; // One chunk = 33.33 yards (16 chunks per tile)
    static constexpr float GRID_STEP = CHUNK_SIZE / 8.0f;  // 8 quads per chunk = 4.17 yards per vertex
};

} // namespace pipeline
} // namespace wowee
