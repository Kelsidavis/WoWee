#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {

class Shader;
class Camera;

class Minimap {
public:
    Minimap();
    ~Minimap();

    bool initialize(int size = 200);
    void shutdown();

    void setAssetManager(pipeline::AssetManager* am) { assetManager = am; }
    void setMapName(const std::string& name);

    void render(const Camera& playerCamera, const glm::vec3& centerWorldPos,
                int screenWidth, int screenHeight);

    void setEnabled(bool enabled) { this->enabled = enabled; }
    bool isEnabled() const { return enabled; }
    void toggle() { enabled = !enabled; }

    void setViewRadius(float radius) { viewRadius = radius; }

    // Public accessors for WorldMap
    GLuint getOrLoadTileTexture(int tileX, int tileY);
    void ensureTRSParsed() { if (!trsParsed) parseTRS(); }
    GLuint getTileQuadVAO() const { return tileQuadVAO; }
    const std::string& getMapName() const { return mapName; }

private:
    void parseTRS();
    void compositeTilesToFBO(const glm::vec3& centerWorldPos);
    void renderQuad(const Camera& playerCamera, const glm::vec3& centerWorldPos,
                    int screenWidth, int screenHeight);

    pipeline::AssetManager* assetManager = nullptr;
    std::string mapName = "Azeroth";

    // TRS lookup: "Azeroth\map32_49" → "e7f0dea73ee6baca78231aaf4b7e772a"
    std::unordered_map<std::string, std::string> trsLookup;
    bool trsParsed = false;

    // Tile texture cache: hash → GL texture ID
    std::unordered_map<std::string, GLuint> tileTextureCache;
    GLuint noDataTexture = 0;  // dark fallback for missing tiles

    // Composite FBO (3x3 tiles = 768x768)
    GLuint compositeFBO = 0;
    GLuint compositeTexture = 0;
    static constexpr int TILE_PX = 256;
    static constexpr int COMPOSITE_PX = TILE_PX * 3;  // 768

    // Tile compositing quad
    GLuint tileQuadVAO = 0;
    GLuint tileQuadVBO = 0;
    std::unique_ptr<Shader> tileShader;

    // Screen quad
    GLuint quadVAO = 0;
    GLuint quadVBO = 0;
    std::unique_ptr<Shader> quadShader;

    int mapSize = 200;
    float viewRadius = 400.0f;  // world units visible in minimap radius
    bool enabled = true;

    // Throttling
    float updateIntervalSec = 0.25f;
    float updateDistance = 6.0f;
    std::chrono::steady_clock::time_point lastUpdateTime{};
    glm::vec3 lastUpdatePos{0.0f};
    bool hasCachedFrame = false;

    // Tile tracking
    int lastCenterTileX = -1;
    int lastCenterTileY = -1;
};

} // namespace rendering
} // namespace wowee
