#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {

class Shader;

struct WorldMapZone {
    uint32_t wmaID = 0;
    uint32_t areaID = 0;       // 0 = continent level
    std::string areaName;      // texture folder name (from DBC)
    float locLeft = 0, locRight = 0, locTop = 0, locBottom = 0;
    uint32_t displayMapID = 0;
    uint32_t parentWorldMapID = 0;

    // Per-zone cached textures
    GLuint tileTextures[12] = {};
    bool tilesLoaded = false;
};

class WorldMap {
public:
    WorldMap();
    ~WorldMap();

    void initialize(pipeline::AssetManager* assetManager);
    void render(const glm::vec3& playerRenderPos, int screenWidth, int screenHeight);
    void setMapName(const std::string& name);
    bool isOpen() const { return open; }
    void close() { open = false; }

private:
    enum class ViewLevel { WORLD, CONTINENT, ZONE };

    void createFBO();
    void createTileShader();
    void createQuad();
    void enterWorldView();
    void loadZonesFromDBC();
    int findBestContinentForPlayer(const glm::vec3& playerRenderPos) const;
    bool zoneBelongsToContinent(int zoneIdx, int contIdx) const;
    bool getContinentProjectionBounds(int contIdx, float& left, float& right,
                                      float& top, float& bottom) const;
    void loadZoneTextures(int zoneIdx);
    void compositeZone(int zoneIdx);
    void renderImGuiOverlay(const glm::vec3& playerRenderPos, int screenWidth, int screenHeight);

    // World pos → map UV using a specific zone's bounds
    glm::vec2 renderPosToMapUV(const glm::vec3& renderPos, int zoneIdx) const;

    pipeline::AssetManager* assetManager = nullptr;
    bool initialized = false;
    bool open = false;

    std::string mapName = "Azeroth";

    // All zones for current map
    std::vector<WorldMapZone> zones;
    int continentIdx = -1;   // index of AreaID=0 entry in zones
    int currentIdx = -1;     // currently displayed zone index
    ViewLevel viewLevel = ViewLevel::CONTINENT;
    int compositedIdx = -1;  // which zone is currently composited in FBO

    // FBO for composited map (4x3 tiles = 1024x768)
    static constexpr int GRID_COLS = 4;
    static constexpr int GRID_ROWS = 3;
    static constexpr int TILE_PX = 256;
    static constexpr int FBO_W = GRID_COLS * TILE_PX;  // 1024
    static constexpr int FBO_H = GRID_ROWS * TILE_PX;  // 768

    GLuint fbo = 0;
    GLuint fboTexture = 0;
    std::unique_ptr<Shader> tileShader;
    GLuint tileQuadVAO = 0;
    GLuint tileQuadVBO = 0;
};

} // namespace rendering
} // namespace wowee
