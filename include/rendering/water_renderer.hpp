#pragma once

#include <vector>
#include <memory>
#include <optional>
#include <cstdint>
#include <glm/glm.hpp>

namespace wowee {
namespace pipeline {
    struct ADTTerrain;
    struct LiquidData;
    struct WMOLiquid;
}

namespace rendering {

class Camera;
class Shader;

/**
 * Water surface for a single map chunk
 */
struct WaterSurface {
    glm::vec3 position;          // World position
    glm::vec3 origin;            // Mesh origin (world)
    glm::vec3 stepX;             // Mesh X step vector in world space
    glm::vec3 stepY;             // Mesh Y step vector in world space
    float minHeight;             // Minimum water height
    float maxHeight;             // Maximum water height
    uint16_t liquidType;         // LiquidType.dbc ID (WotLK)

    // Owning tile coordinates (for per-tile removal)
    int tileX = -1, tileY = -1;

    // Owning WMO instance ID (for WMO liquid removal, 0 = terrain water)
    uint32_t wmoId = 0;

    // Water layer dimensions within chunk (0-7 offset, 1-8 size)
    uint8_t xOffset = 0;
    uint8_t yOffset = 0;
    uint8_t width = 8;           // Width in tiles (1-8)
    uint8_t height = 8;          // Height in tiles (1-8)

    // Height map for water surface ((width+1) x (height+1) vertices)
    std::vector<float> heights;

    // Render mask (which tiles have water)
    std::vector<uint8_t> mask;

    // Render data
    uint32_t vao = 0;
    uint32_t vbo = 0;
    uint32_t ebo = 0;
    int indexCount = 0;

    bool hasHeightData() const { return !heights.empty(); }
};

/**
 * Water renderer
 *
 * Renders water surfaces with transparency and animation.
 * Supports multiple liquid types (water, ocean, magma, slime).
 */
class WaterRenderer {
public:
    WaterRenderer();
    ~WaterRenderer();

    bool initialize();
    void shutdown();

    /**
     * Load water surfaces from ADT terrain
     * @param terrain The ADT terrain data
     * @param append If true, add to existing water instead of replacing
     * @param tileX Tile X coordinate for tracking ownership (-1 = untracked)
     * @param tileY Tile Y coordinate for tracking ownership (-1 = untracked)
     */
    void loadFromTerrain(const pipeline::ADTTerrain& terrain, bool append = false,
                         int tileX = -1, int tileY = -1);

    /**
     * Load water surface from WMO liquid data
     * @param liquid WMO liquid data from MLIQ chunk
     * @param modelMatrix WMO instance model matrix for transforming to world space
     * @param wmoId WMO instance ID for tracking ownership
     */
    void loadFromWMO(const pipeline::WMOLiquid& liquid, const glm::mat4& modelMatrix, uint32_t wmoId);

    /**
     * Remove all water surfaces belonging to a specific WMO instance
     * @param wmoId WMO instance ID
     */
    void removeWMO(uint32_t wmoId);

    /**
     * Remove all water surfaces belonging to a specific tile
     * @param tileX Tile X coordinate
     * @param tileY Tile Y coordinate
     */
    void removeTile(int tileX, int tileY);

    /**
     * Clear all water surfaces
     */
    void clear();

    /**
     * Render all water surfaces
     */
    void render(const Camera& camera, float time);

    /**
     * Enable/disable water rendering
     */
    void setEnabled(bool enabled) { renderingEnabled = enabled; }
    bool isEnabled() const { return renderingEnabled; }

    /**
     * Query the water height at a given world position.
     * Returns the highest water surface height at that XY, or nullopt if no water.
     */
    std::optional<float> getWaterHeightAt(float glX, float glY) const;
    std::optional<uint16_t> getWaterTypeAt(float glX, float glY) const;

    /**
     * Get water surface count
     */
    int getSurfaceCount() const { return static_cast<int>(surfaces.size()); }

private:
    void createWaterMesh(WaterSurface& surface);
    void destroyWaterMesh(WaterSurface& surface);

    glm::vec4 getLiquidColor(uint16_t liquidType) const;
    float getLiquidAlpha(uint16_t liquidType) const;

    std::unique_ptr<Shader> waterShader;
    std::vector<WaterSurface> surfaces;
    bool renderingEnabled = true;
};

} // namespace rendering
} // namespace wowee
