#include "rendering/minimap.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <sstream>
#include <cmath>

namespace wowee {
namespace rendering {

Minimap::Minimap() = default;

Minimap::~Minimap() {
    shutdown();
}

bool Minimap::initialize(int size) {
    mapSize = size;

    // --- Composite FBO (3x3 tiles = 768x768) ---
    glGenFramebuffers(1, &compositeFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, compositeFBO);

    glGenTextures(1, &compositeTexture);
    glBindTexture(GL_TEXTURE_2D, compositeTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, COMPOSITE_PX, COMPOSITE_PX, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, compositeTexture, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("Minimap composite FBO incomplete");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --- Unit quad for tile compositing ---
    float quadVerts[] = {
        // pos (x,y), uv (u,v)
        0.0f, 0.0f,  0.0f, 0.0f,
        1.0f, 0.0f,  1.0f, 0.0f,
        1.0f, 1.0f,  1.0f, 1.0f,
        0.0f, 0.0f,  0.0f, 0.0f,
        1.0f, 1.0f,  1.0f, 1.0f,
        0.0f, 1.0f,  0.0f, 1.0f,
    };

    glGenVertexArrays(1, &tileQuadVAO);
    glGenBuffers(1, &tileQuadVBO);
    glBindVertexArray(tileQuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, tileQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    // --- Tile compositing shader ---
    const char* tileVertSrc = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aUV;

        uniform vec2 uGridOffset;  // (col, row) in 0-2

        out vec2 TexCoord;

        void main() {
            vec2 gridPos = (uGridOffset + aPos) / 3.0;
            gl_Position = vec4(gridPos * 2.0 - 1.0, 0.0, 1.0);
            TexCoord = aUV;
        }
    )";

    const char* tileFragSrc = R"(
        #version 330 core
        in vec2 TexCoord;

        uniform sampler2D uTileTexture;

        out vec4 FragColor;

        void main() {
            // BLP minimap tiles have same axis transposition as ADT terrain:
            // tile U (cols) = north-south, tile V (rows) = west-east
            // Composite grid: TexCoord.x = west-east, TexCoord.y = north-south
            // So swap to match
            FragColor = texture(uTileTexture, vec2(TexCoord.y, TexCoord.x));
        }
    )";

    tileShader = std::make_unique<Shader>();
    if (!tileShader->loadFromSource(tileVertSrc, tileFragSrc)) {
        LOG_ERROR("Failed to create minimap tile compositing shader");
        return false;
    }

    // --- Screen quad ---
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    // --- Screen quad shader with rotation + circular mask ---
    const char* quadVertSrc = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aUV;

        uniform vec4 uRect;  // x, y, w, h in 0..1 screen space

        out vec2 TexCoord;

        void main() {
            vec2 pos = uRect.xy + aUV * uRect.zw;
            gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
            TexCoord = aUV;
        }
    )";

    const char* quadFragSrc = R"(
        #version 330 core
        in vec2 TexCoord;

        uniform sampler2D uComposite;
        uniform vec2 uPlayerUV;
        uniform float uRotation;
        uniform float uArrowRotation;
        uniform float uZoomRadius;
        uniform bool uSquareShape;

        out vec4 FragColor;

        bool pointInTriangle(vec2 p, vec2 a, vec2 b, vec2 c) {
            vec2 v0 = c - a, v1 = b - a, v2 = p - a;
            float d00 = dot(v0, v0);
            float d01 = dot(v0, v1);
            float d02 = dot(v0, v2);
            float d11 = dot(v1, v1);
            float d12 = dot(v1, v2);
            float inv = 1.0 / (d00 * d11 - d01 * d01);
            float u = (d11 * d02 - d01 * d12) * inv;
            float v = (d00 * d12 - d01 * d02) * inv;
            return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);
        }

        vec2 rot2(vec2 v, float ang) {
            float c = cos(ang);
            float s = sin(ang);
            return vec2(v.x * c - v.y * s, v.x * s + v.y * c);
        }

        void main() {
            vec2 centered = TexCoord - 0.5;
            float dist = length(centered);
            float maxDist = uSquareShape ? max(abs(centered.x), abs(centered.y)) : dist;
            if (maxDist > 0.5) discard;

            // Rotate screen coords → composite UV offset
            // Composite: U increases east, V increases north
            // Screen: +X=right, +Y=up
            float c = cos(uRotation);
            float s = sin(uRotation);
            float scale = uZoomRadius * 2.0;

            vec2 offset = vec2(
                centered.x * c + centered.y * s,
                -centered.x * s + centered.y * c
            ) * scale;

            vec2 uv = uPlayerUV + offset;
            vec3 color = texture(uComposite, uv).rgb;

            // Thin dark border at edge
            if (maxDist > 0.49) {
                color = mix(color, vec3(0.08), smoothstep(0.49, 0.5, maxDist));
            }

            // Player arrow at center (always points up = forward)
            vec2 ap = rot2(centered, -(uArrowRotation + 3.14159265));
            vec2 tip = vec2(0.0, 0.035);
            vec2 lt  = vec2(-0.018, -0.016);
            vec2 rt  = vec2(0.018, -0.016);
            vec2 nL  = vec2(-0.006, -0.006);
            vec2 nR  = vec2(0.006, -0.006);
            vec2 nB  = vec2(0.0, 0.006);

            bool inArrow = pointInTriangle(ap, tip, lt, rt)
                        && !pointInTriangle(ap, nL, nR, nB);

            if (inArrow) {
                color = vec3(0.0, 0.0, 0.0);
            }

            FragColor = vec4(color, 0.8);
        }
    )";

    quadShader = std::make_unique<Shader>();
    if (!quadShader->loadFromSource(quadVertSrc, quadFragSrc)) {
        LOG_ERROR("Failed to create minimap screen quad shader");
        return false;
    }

    // --- No-data fallback texture (dark blue-gray) ---
    glGenTextures(1, &noDataTexture);
    glBindTexture(GL_TEXTURE_2D, noDataTexture);
    uint8_t darkPixel[4] = { 12, 20, 30, 255 };
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, darkPixel);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    LOG_INFO("Minimap initialized (", mapSize, "x", mapSize, " screen, ",
             COMPOSITE_PX, "x", COMPOSITE_PX, " composite)");
    return true;
}

void Minimap::shutdown() {
    if (compositeFBO) { glDeleteFramebuffers(1, &compositeFBO); compositeFBO = 0; }
    if (compositeTexture) { glDeleteTextures(1, &compositeTexture); compositeTexture = 0; }
    if (tileQuadVAO) { glDeleteVertexArrays(1, &tileQuadVAO); tileQuadVAO = 0; }
    if (tileQuadVBO) { glDeleteBuffers(1, &tileQuadVBO); tileQuadVBO = 0; }
    if (quadVAO) { glDeleteVertexArrays(1, &quadVAO); quadVAO = 0; }
    if (quadVBO) { glDeleteBuffers(1, &quadVBO); quadVBO = 0; }
    if (noDataTexture) { glDeleteTextures(1, &noDataTexture); noDataTexture = 0; }

    // Delete cached tile textures
    for (auto& [hash, tex] : tileTextureCache) {
        if (tex) glDeleteTextures(1, &tex);
    }
    tileTextureCache.clear();

    tileShader.reset();
    quadShader.reset();
}

void Minimap::setMapName(const std::string& name) {
    if (mapName != name) {
        mapName = name;
        hasCachedFrame = false;
        lastCenterTileX = -1;
        lastCenterTileY = -1;
    }
}

// --------------------------------------------------------
// TRS parsing
// --------------------------------------------------------

void Minimap::parseTRS() {
    if (trsParsed || !assetManager) return;
    trsParsed = true;

    auto data = assetManager->getMPQManager().readFile("Textures\\Minimap\\md5translate.trs");
    if (data.empty()) {
        LOG_WARNING("Failed to load md5translate.trs");
        return;
    }

    std::string content(reinterpret_cast<const char*>(data.data()), data.size());
    std::istringstream stream(content);
    std::string line;
    int count = 0;

    while (std::getline(stream, line)) {
        // Remove \r
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Skip "dir:" lines and empty lines
        if (line.empty() || line.substr(0, 4) == "dir:") continue;

        // Format: "Azeroth\map32_49.blp\t<hash>.blp"
        auto tabPos = line.find('\t');
        if (tabPos == std::string::npos) continue;

        std::string key = line.substr(0, tabPos);
        std::string hashFile = line.substr(tabPos + 1);

        // Strip .blp from key: "Azeroth\map32_49"
        if (key.size() > 4 && key.substr(key.size() - 4) == ".blp") {
            key = key.substr(0, key.size() - 4);
        }
        // Strip .blp from hash to get just the md5: "e7f0dea73ee6baca78231aaf4b7e772a"
        if (hashFile.size() > 4 && hashFile.substr(hashFile.size() - 4) == ".blp") {
            hashFile = hashFile.substr(0, hashFile.size() - 4);
        }

        trsLookup[key] = hashFile;
        count++;
    }

    LOG_INFO("Parsed md5translate.trs: ", count, " entries");
}

// --------------------------------------------------------
// Tile texture loading
// --------------------------------------------------------

GLuint Minimap::getOrLoadTileTexture(int tileX, int tileY) {
    // Build TRS key: "Azeroth\map32_49"
    std::string key = mapName + "\\map" + std::to_string(tileX) + "_" + std::to_string(tileY);

    auto trsIt = trsLookup.find(key);
    if (trsIt == trsLookup.end()) {
        return noDataTexture;
    }

    const std::string& hash = trsIt->second;

    // Check texture cache
    auto cacheIt = tileTextureCache.find(hash);
    if (cacheIt != tileTextureCache.end()) {
        return cacheIt->second;
    }

    // Load from MPQ
    std::string blpPath = "Textures\\Minimap\\" + hash + ".blp";
    auto blpImage = assetManager->loadTexture(blpPath);
    if (!blpImage.isValid()) {
        tileTextureCache[hash] = noDataTexture;
        return noDataTexture;
    }

    // Create GL texture
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, blpImage.width, blpImage.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blpImage.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    tileTextureCache[hash] = tex;
    return tex;
}

// --------------------------------------------------------
// Composite 3x3 tiles into FBO
// --------------------------------------------------------

void Minimap::compositeTilesToFBO(const glm::vec3& centerWorldPos) {
    // centerWorldPos is in render coords (renderX=wowY, renderY=wowX)
    auto [tileX, tileY] = core::coords::worldToTile(centerWorldPos.x, centerWorldPos.y);

    // Save GL state
    GLint prevFBO = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, compositeFBO);
    glViewport(0, 0, COMPOSITE_PX, COMPOSITE_PX);
    glClearColor(0.05f, 0.08f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    tileShader->use();
    tileShader->setUniform("uTileTexture", 0);

    glBindVertexArray(tileQuadVAO);

    // Draw 3x3 tile grid into composite FBO.
    // BLP first row → GL V=0 (bottom) = north edge of tile.
    // So north tile (dr=-1) goes to row 0 (bottom), south (dr=+1) to row 2 (top).
    // West tile (dc=-1) goes to col 0 (left), east (dc=+1) to col 2 (right).
    // Result: composite U=0→west, U=1→east, V=0→north, V=1→south.
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int tx = tileX + dr;
            int ty = tileY + dc;

            GLuint tileTex = getOrLoadTileTexture(tx, ty);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tileTex);

            // Grid position: dr=-1 (north) → row 0, dr=0 → row 1, dr=+1 (south) → row 2
            float col = static_cast<float>(dc + 1);  // 0, 1, 2
            float row = static_cast<float>(dr + 1);  // 0, 1, 2

            tileShader->setUniform("uGridOffset", glm::vec2(col, row));
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }
    }

    glBindVertexArray(0);

    // Restore GL state
    glBindFramebuffer(GL_FRAMEBUFFER, prevFBO);
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);

    lastCenterTileX = tileX;
    lastCenterTileY = tileY;
}

// --------------------------------------------------------
// Main render
// --------------------------------------------------------

void Minimap::render(const Camera& playerCamera, const glm::vec3& centerWorldPos,
                     int screenWidth, int screenHeight) {
    if (!enabled || !assetManager || !compositeFBO) return;

    // Lazy-parse TRS on first use
    if (!trsParsed) parseTRS();

    // Check if composite needs refresh
    const auto now = std::chrono::steady_clock::now();
    bool needsRefresh = !hasCachedFrame;
    if (!needsRefresh) {
        float moved = glm::length(glm::vec2(centerWorldPos.x - lastUpdatePos.x,
                                             centerWorldPos.y - lastUpdatePos.y));
        float elapsed = std::chrono::duration<float>(now - lastUpdateTime).count();
        needsRefresh = (moved >= updateDistance) || (elapsed >= updateIntervalSec);
    }

    // Also refresh if player crossed a tile boundary
    auto [curTileX, curTileY] = core::coords::worldToTile(centerWorldPos.x, centerWorldPos.y);
    if (curTileX != lastCenterTileX || curTileY != lastCenterTileY) {
        needsRefresh = true;
    }

    if (needsRefresh) {
        compositeTilesToFBO(centerWorldPos);
        lastUpdateTime = now;
        lastUpdatePos = centerWorldPos;
        hasCachedFrame = true;
    }

    // Draw screen quad
    renderQuad(playerCamera, centerWorldPos, screenWidth, screenHeight);
}

void Minimap::renderQuad(const Camera& playerCamera, const glm::vec3& centerWorldPos,
                         int screenWidth, int screenHeight) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    quadShader->use();

    // Position minimap in top-right corner
    float margin = 10.0f;
    float pixelW = static_cast<float>(mapSize) / screenWidth;
    float pixelH = static_cast<float>(mapSize) / screenHeight;
    float x = 1.0f - pixelW - margin / screenWidth;
    float y = 1.0f - pixelH - margin / screenHeight;
    quadShader->setUniform("uRect", glm::vec4(x, y, pixelW, pixelH));

    // Compute player's UV in the composite texture
    // Render coords: renderX = wowY (west axis), renderY = wowX (north axis)
    constexpr float TILE_SIZE = core::coords::TILE_SIZE;
    auto [tileX, tileY] = core::coords::worldToTile(centerWorldPos.x, centerWorldPos.y);

    // Fractional position within center tile
    // tileX = floor(32 - wowX/TILE_SIZE), wowX = renderY
    // fracNS: 0 = north edge of tile, 1 = south edge
    float fracNS = 32.0f - static_cast<float>(tileX) - centerWorldPos.y / TILE_SIZE;
    // fracEW: 0 = west edge of tile, 1 = east edge
    float fracEW = 32.0f - static_cast<float>(tileY) - centerWorldPos.x / TILE_SIZE;

    // Composite UV: center tile is grid slot (1,1) → UV range [1/3, 2/3]
    // Composite orientation: U=0→west, U=1→east, V=0→north, V=1→south
    float playerU = (1.0f + fracEW) / 3.0f;
    float playerV = (1.0f + fracNS) / 3.0f;

    quadShader->setUniform("uPlayerUV", glm::vec2(playerU, playerV));

    // Zoom: convert view radius from world units to composite UV fraction
    float zoomRadius = viewRadius / (TILE_SIZE * 3.0f);
    quadShader->setUniform("uZoomRadius", zoomRadius);

    // Rotation: compass bearing from north, clockwise
    // renderX = wowY (west), renderY = wowX (north)
    // Facing north: fwd=(0,1,0) → bearing=0
    // Facing east:  fwd=(-1,0,0) → bearing=π/2
    float rotation = 0.0f;
    if (rotateWithCamera) {
        glm::vec3 fwd = playerCamera.getForward();
        rotation = std::atan2(-fwd.x, fwd.y);
    }
    quadShader->setUniform("uRotation", rotation);
    float arrowRotation = 0.0f;
    if (!rotateWithCamera) {
        glm::vec3 fwd = playerCamera.getForward();
        arrowRotation = std::atan2(-fwd.x, fwd.y);
    }
    quadShader->setUniform("uArrowRotation", arrowRotation);
    quadShader->setUniform("uSquareShape", squareShape);

    quadShader->setUniform("uComposite", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, compositeTexture);

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

} // namespace rendering
} // namespace wowee
