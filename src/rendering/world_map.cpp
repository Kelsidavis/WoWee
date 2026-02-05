#include "rendering/world_map.hpp"
#include "rendering/shader.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/coordinates.hpp"
#include "core/input.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <imgui.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace wowee {
namespace rendering {

namespace {
bool isRootContinent(const std::vector<WorldMapZone>& zones, int idx) {
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return false;
    const auto& c = zones[idx];
    if (c.areaID != 0 || c.wmaID == 0) return false;
    for (const auto& z : zones) {
        if (z.areaID == 0 && z.parentWorldMapID == c.wmaID) {
            return true;
        }
    }
    return false;
}

bool isLeafContinent(const std::vector<WorldMapZone>& zones, int idx) {
    if (idx < 0 || idx >= static_cast<int>(zones.size())) return false;
    const auto& c = zones[idx];
    if (c.areaID != 0) return false;
    return c.parentWorldMapID != 0;
}
} // namespace

WorldMap::WorldMap() = default;

WorldMap::~WorldMap() {
    if (fbo) glDeleteFramebuffers(1, &fbo);
    if (fboTexture) glDeleteTextures(1, &fboTexture);
    if (tileQuadVAO) glDeleteVertexArrays(1, &tileQuadVAO);
    if (tileQuadVBO) glDeleteBuffers(1, &tileQuadVBO);
    for (auto& zone : zones) {
        for (auto& tex : zone.tileTextures) {
            if (tex) glDeleteTextures(1, &tex);
        }
    }
    tileShader.reset();
}

void WorldMap::initialize(pipeline::AssetManager* am) {
    if (initialized) return;
    assetManager = am;
    createFBO();
    createTileShader();
    createQuad();
    initialized = true;
    LOG_INFO("WorldMap initialized (", FBO_W, "x", FBO_H, " FBO)");
}

void WorldMap::setMapName(const std::string& name) {
    if (mapName == name && !zones.empty()) return;
    mapName = name;
    // Clear old zone data
    for (auto& zone : zones) {
        for (auto& tex : zone.tileTextures) {
            if (tex) { glDeleteTextures(1, &tex); tex = 0; }
        }
    }
    zones.clear();
    continentIdx = -1;
    currentIdx = -1;
    compositedIdx = -1;
    viewLevel = ViewLevel::WORLD;
}

// --------------------------------------------------------
// GL resource creation
// --------------------------------------------------------

void WorldMap::createFBO() {
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, FBO_W, FBO_H, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("WorldMap FBO incomplete");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void WorldMap::createTileShader() {
    const char* vertSrc = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aUV;

        uniform vec2 uGridOffset;  // (col, row) in grid
        uniform float uGridCols;
        uniform float uGridRows;

        out vec2 TexCoord;

        void main() {
            vec2 gridPos = vec2(
                (uGridOffset.x + aPos.x) / uGridCols,
                (uGridOffset.y + aPos.y) / uGridRows
            );
            gl_Position = vec4(gridPos * 2.0 - 1.0, 0.0, 1.0);
            TexCoord = aUV;
        }
    )";

    const char* fragSrc = R"(
        #version 330 core
        in vec2 TexCoord;

        uniform sampler2D uTileTexture;

        out vec4 FragColor;

        void main() {
            FragColor = texture(uTileTexture, TexCoord);
        }
    )";

    tileShader = std::make_unique<Shader>();
    if (!tileShader->loadFromSource(vertSrc, fragSrc)) {
        LOG_ERROR("Failed to create WorldMap tile shader");
    }
}

void WorldMap::createQuad() {
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
}

// --------------------------------------------------------
// DBC zone loading
// --------------------------------------------------------

void WorldMap::loadZonesFromDBC() {
    if (!zones.empty() || !assetManager) return;

    // Step 1: Resolve mapID from Map.dbc
    int mapID = -1;
    auto mapDbc = assetManager->loadDBC("Map.dbc");
    if (mapDbc && mapDbc->isLoaded()) {
        for (uint32_t i = 0; i < mapDbc->getRecordCount(); i++) {
            std::string dir = mapDbc->getString(i, 1);
            if (dir == mapName) {
                mapID = static_cast<int>(mapDbc->getUInt32(i, 0));
                LOG_INFO("WorldMap: Map.dbc '", mapName, "' -> mapID=", mapID);
                break;
            }
        }
    }

    if (mapID < 0) {
        if (mapName == "Azeroth") mapID = 0;
        else if (mapName == "Kalimdor") mapID = 1;
        else if (mapName == "Expansion01") mapID = 530;
        else if (mapName == "Northrend") mapID = 571;
        else {
            LOG_WARNING("WorldMap: unknown map '", mapName, "'");
            return;
        }
    }

    // Step 2: Load ALL WorldMapArea records for this mapID
    auto wmaDbc = assetManager->loadDBC("WorldMapArea.dbc");
    if (!wmaDbc || !wmaDbc->isLoaded()) {
        LOG_WARNING("WorldMap: WorldMapArea.dbc not found");
        return;
    }

    LOG_INFO("WorldMap: WorldMapArea.dbc has ", wmaDbc->getFieldCount(),
             " fields, ", wmaDbc->getRecordCount(), " records");

    // WorldMapArea.dbc layout (11 fields, no localized strings):
    //   0: ID, 1: MapID, 2: AreaID, 3: AreaName (stringref)
    //   4: locLeft, 5: locRight, 6: locTop, 7: locBottom
    //   8: displayMapID, 9: defaultDungeonFloor, 10: parentWorldMapID

    for (uint32_t i = 0; i < wmaDbc->getRecordCount(); i++) {
        uint32_t recMapID = wmaDbc->getUInt32(i, 1);
        if (static_cast<int>(recMapID) != mapID) continue;

        WorldMapZone zone;
        zone.wmaID   = wmaDbc->getUInt32(i, 0);
        zone.areaID  = wmaDbc->getUInt32(i, 2);
        zone.areaName = wmaDbc->getString(i, 3);
        zone.locLeft   = wmaDbc->getFloat(i, 4);
        zone.locRight  = wmaDbc->getFloat(i, 5);
        zone.locTop    = wmaDbc->getFloat(i, 6);
        zone.locBottom = wmaDbc->getFloat(i, 7);
        zone.displayMapID = wmaDbc->getUInt32(i, 8);
        zone.parentWorldMapID = wmaDbc->getUInt32(i, 10);

        int idx = static_cast<int>(zones.size());

        // Debug: also log raw uint32 values for bounds fields
        uint32_t raw4 = wmaDbc->getUInt32(i, 4);
        uint32_t raw5 = wmaDbc->getUInt32(i, 5);
        uint32_t raw6 = wmaDbc->getUInt32(i, 6);
        uint32_t raw7 = wmaDbc->getUInt32(i, 7);

        LOG_INFO("WorldMap: zone[", idx, "] areaID=", zone.areaID,
                 " '", zone.areaName, "' L=", zone.locLeft,
                 " R=", zone.locRight, " T=", zone.locTop,
                 " B=", zone.locBottom,
                 " (raw4=", raw4, " raw5=", raw5,
                 " raw6=", raw6, " raw7=", raw7, ")");

        if (zone.areaID == 0 && continentIdx < 0) {
            continentIdx = idx;
        }

        zones.push_back(std::move(zone));
    }

    // For each continent entry with missing bounds, derive bounds from its child zones only.
    for (int ci = 0; ci < static_cast<int>(zones.size()); ci++) {
        auto& cont = zones[ci];
        if (cont.areaID != 0) continue;

        if (std::abs(cont.locLeft) > 0.001f || std::abs(cont.locRight) > 0.001f ||
            std::abs(cont.locTop) > 0.001f || std::abs(cont.locBottom) > 0.001f) {
            continue;
        }

        bool first = true;
        for (const auto& z : zones) {
            if (z.areaID == 0) continue;
            if (std::abs(z.locLeft - z.locRight) < 0.001f ||
                std::abs(z.locTop - z.locBottom) < 0.001f) {
                continue;
            }

            // Prefer explicit parent linkage when deriving continent extents.
            if (z.parentWorldMapID != 0 && cont.wmaID != 0 && z.parentWorldMapID != cont.wmaID) {
                continue;
            }

            if (first) {
                cont.locLeft = z.locLeft;
                cont.locRight = z.locRight;
                cont.locTop = z.locTop;
                cont.locBottom = z.locBottom;
                first = false;
            } else {
                cont.locLeft = std::max(cont.locLeft, z.locLeft);
                cont.locRight = std::min(cont.locRight, z.locRight);
                cont.locTop = std::max(cont.locTop, z.locTop);
                cont.locBottom = std::min(cont.locBottom, z.locBottom);
            }
        }

        if (!first) {
            LOG_INFO("WorldMap: computed bounds for continent '", cont.areaName, "': L=", cont.locLeft,
                     " R=", cont.locRight, " T=", cont.locTop, " B=", cont.locBottom);
        }
    }

    LOG_INFO("WorldMap: loaded ", zones.size(), " zones for mapID=", mapID,
             ", continentIdx=", continentIdx);
}

int WorldMap::findBestContinentForPlayer(const glm::vec3& playerRenderPos) const {
    float wowX = playerRenderPos.y;  // north/south
    float wowY = playerRenderPos.x;  // west/east

    int bestIdx = -1;
    float bestArea = std::numeric_limits<float>::max();
    float bestCenterDist2 = std::numeric_limits<float>::max();

    bool hasLeafContinent = false;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (zones[i].areaID == 0 && !isRootContinent(zones, i)) {
            hasLeafContinent = true;
            break;
        }
    }

    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        const auto& z = zones[i];
        if (z.areaID != 0) continue;
        if (hasLeafContinent && isRootContinent(zones, i)) continue;

        float minX = std::min(z.locLeft, z.locRight);
        float maxX = std::max(z.locLeft, z.locRight);
        float minY = std::min(z.locTop, z.locBottom);
        float maxY = std::max(z.locTop, z.locBottom);
        float spanX = maxX - minX;
        float spanY = maxY - minY;
        if (spanX < 0.001f || spanY < 0.001f) continue;

        bool contains = (wowX >= minX && wowX <= maxX && wowY >= minY && wowY <= maxY);
        float area = spanX * spanY;
        if (contains) {
            if (area < bestArea) {
                bestArea = area;
                bestIdx = i;
            }
        } else if (bestIdx < 0) {
            // Fallback if player isn't inside any continent bounds: nearest center.
            float cx = (minX + maxX) * 0.5f;
            float cy = (minY + maxY) * 0.5f;
            float dx = wowX - cx;
            float dy = wowY - cy;
            float dist2 = dx * dx + dy * dy;
            if (dist2 < bestCenterDist2) {
                bestCenterDist2 = dist2;
                bestIdx = i;
            }
        }
    }

    return bestIdx;
}

bool WorldMap::zoneBelongsToContinent(int zoneIdx, int contIdx) const {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return false;
    if (contIdx < 0 || contIdx >= static_cast<int>(zones.size())) return false;

    const auto& z = zones[zoneIdx];
    const auto& cont = zones[contIdx];

    if (z.areaID == 0) return false;

    // Prefer explicit parent linkage from WorldMapArea.dbc.
    if (z.parentWorldMapID != 0 && cont.wmaID != 0) {
        return z.parentWorldMapID == cont.wmaID;
    }

    auto rectMinX = [](const WorldMapZone& a) { return std::min(a.locLeft, a.locRight); };
    auto rectMaxX = [](const WorldMapZone& a) { return std::max(a.locLeft, a.locRight); };
    auto rectMinY = [](const WorldMapZone& a) { return std::min(a.locTop, a.locBottom); };
    auto rectMaxY = [](const WorldMapZone& a) { return std::max(a.locTop, a.locBottom); };

    float zMinX = rectMinX(z), zMaxX = rectMaxX(z);
    float zMinY = rectMinY(z), zMaxY = rectMaxY(z);
    if ((zMaxX - zMinX) < 0.001f || (zMaxY - zMinY) < 0.001f) return false;

    // Fallback: assign zone to the continent with highest overlap area.
    int bestContIdx = -1;
    float bestOverlap = 0.0f;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        const auto& c = zones[i];
        if (c.areaID != 0) continue;

        float cMinX = rectMinX(c), cMaxX = rectMaxX(c);
        float cMinY = rectMinY(c), cMaxY = rectMaxY(c);
        if ((cMaxX - cMinX) < 0.001f || (cMaxY - cMinY) < 0.001f) continue;

        float ox = std::max(0.0f, std::min(zMaxX, cMaxX) - std::max(zMinX, cMinX));
        float oy = std::max(0.0f, std::min(zMaxY, cMaxY) - std::max(zMinY, cMinY));
        float overlap = ox * oy;
        if (overlap > bestOverlap) {
            bestOverlap = overlap;
            bestContIdx = i;
        }
    }

    if (bestContIdx >= 0) {
        return bestContIdx == contIdx;
    }

    // Last resort: center-point containment.
    float centerX = (z.locLeft + z.locRight) * 0.5f;
    float centerY = (z.locTop + z.locBottom) * 0.5f;
    float cMinX = rectMinX(cont), cMaxX = rectMaxX(cont);
    float cMinY = rectMinY(cont), cMaxY = rectMaxY(cont);
    return centerX >= cMinX && centerX <= cMaxX &&
           centerY >= cMinY && centerY <= cMaxY;
}

bool WorldMap::getContinentProjectionBounds(int contIdx, float& left, float& right,
                                            float& top, float& bottom) const {
    if (contIdx < 0 || contIdx >= static_cast<int>(zones.size())) return false;

    const auto& cont = zones[contIdx];
    if (cont.areaID != 0) return false;

    // Prefer authored continent bounds from DBC when available.
    if (std::abs(cont.locLeft - cont.locRight) > 0.001f &&
        std::abs(cont.locTop - cont.locBottom) > 0.001f) {
        left = cont.locLeft;
        right = cont.locRight;
        top = cont.locTop;
        bottom = cont.locBottom;
        return true;
    }

    std::vector<float> northEdges;
    std::vector<float> southEdges;
    std::vector<float> westEdges;
    std::vector<float> eastEdges;

    for (int zi = 0; zi < static_cast<int>(zones.size()); zi++) {
        if (!zoneBelongsToContinent(zi, contIdx)) continue;
        const auto& z = zones[zi];
        if (std::abs(z.locLeft - z.locRight) < 0.001f ||
            std::abs(z.locTop - z.locBottom) < 0.001f) {
            continue;
        }

        northEdges.push_back(std::max(z.locLeft, z.locRight));
        southEdges.push_back(std::min(z.locLeft, z.locRight));
        westEdges.push_back(std::max(z.locTop, z.locBottom));
        eastEdges.push_back(std::min(z.locTop, z.locBottom));
    }

    if (northEdges.size() < 3) {
        left = cont.locLeft;
        right = cont.locRight;
        top = cont.locTop;
        bottom = cont.locBottom;
        return std::abs(left - right) > 0.001f && std::abs(top - bottom) > 0.001f;
    }

    // Fallback: derive full extents from child zones.
    left = *std::max_element(northEdges.begin(), northEdges.end());
    right = *std::min_element(southEdges.begin(), southEdges.end());
    top = *std::max_element(westEdges.begin(), westEdges.end());
    bottom = *std::min_element(eastEdges.begin(), eastEdges.end());

    if (left <= right || top <= bottom) {
        left = cont.locLeft;
        right = cont.locRight;
        top = cont.locTop;
        bottom = cont.locBottom;
    }

    return std::abs(left - right) > 0.001f && std::abs(top - bottom) > 0.001f;
}

// --------------------------------------------------------
// Per-zone texture loading
// --------------------------------------------------------

void WorldMap::loadZoneTextures(int zoneIdx) {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return;
    auto& zone = zones[zoneIdx];
    if (zone.tilesLoaded) return;
    zone.tilesLoaded = true;

    const std::string& folder = zone.areaName;
    if (folder.empty()) return;

    std::vector<std::string> candidateFolders;
    candidateFolders.push_back(folder);
    if (zone.areaID == 0 && mapName == "Azeroth") {
        if (folder != "Azeroth") candidateFolders.push_back("Azeroth");
        if (folder != "EasternKingdoms") candidateFolders.push_back("EasternKingdoms");
    }

    int loaded = 0;
    for (int i = 0; i < 12; i++) {
        pipeline::BLPImage blpImage;
        bool found = false;
        for (const auto& testFolder : candidateFolders) {
            std::string path = "Interface\\WorldMap\\" + testFolder + "\\" +
                               testFolder + std::to_string(i + 1) + ".blp";
            blpImage = assetManager->loadTexture(path);
            if (blpImage.isValid()) {
                found = true;
                break;
            }
        }

        if (!found) {
            zone.tileTextures[i] = 0;
            continue;
        }

        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, blpImage.width, blpImage.height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, blpImage.data.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        zone.tileTextures[i] = tex;
        loaded++;
    }

    LOG_INFO("WorldMap: loaded ", loaded, "/12 tiles for '", folder, "'");
}

// --------------------------------------------------------
// Composite a zone's tiles into the FBO
// --------------------------------------------------------

void WorldMap::compositeZone(int zoneIdx) {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return;
    if (compositedIdx == zoneIdx) return;

    const auto& zone = zones[zoneIdx];

    // Save GL state
    GLint prevFBO = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    GLboolean prevBlend = glIsEnabled(GL_BLEND);
    GLboolean prevDepthTest = glIsEnabled(GL_DEPTH_TEST);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, FBO_W, FBO_H);
    glClearColor(0.05f, 0.08f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    tileShader->use();
    tileShader->setUniform("uTileTexture", 0);
    tileShader->setUniform("uGridCols", static_cast<float>(GRID_COLS));
    tileShader->setUniform("uGridRows", static_cast<float>(GRID_ROWS));

    glBindVertexArray(tileQuadVAO);

    // Tiles 1-12 in a 4x3 grid: tile N at col=(N-1)%4, row=(N-1)/4
    // Row 0 (tiles 1-4) = top of image (north) → placed at FBO bottom (GL y=0)
    // ImGui::Image maps GL (0,0) → widget top-left → north at top ✓
    for (int i = 0; i < 12; i++) {
        if (zone.tileTextures[i] == 0) continue;

        int col = i % GRID_COLS;
        int row = i / GRID_COLS;

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, zone.tileTextures[i]);

        tileShader->setUniform("uGridOffset", glm::vec2(
            static_cast<float>(col), static_cast<float>(row)));
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    glBindVertexArray(0);

    // Restore GL state
    glBindFramebuffer(GL_FRAMEBUFFER, prevFBO);
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
    if (prevBlend) glEnable(GL_BLEND);
    if (prevDepthTest) glEnable(GL_DEPTH_TEST);

    compositedIdx = zoneIdx;
}

void WorldMap::enterWorldView() {
    viewLevel = ViewLevel::WORLD;

    int rootIdx = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (isRootContinent(zones, i)) {
            rootIdx = i;
            break;
        }
    }

    if (rootIdx >= 0) {
        loadZoneTextures(rootIdx);
        bool hasAnyTile = false;
        for (GLuint tex : zones[rootIdx].tileTextures) {
            if (tex != 0) { hasAnyTile = true; break; }
        }
        if (hasAnyTile) {
            compositeZone(rootIdx);
            currentIdx = rootIdx;
            return;
        }
    }

    // Fallback: use first leaf continent as world-view backdrop.
    int fallbackContinent = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); i++) {
        if (isLeafContinent(zones, i)) {
            fallbackContinent = i;
            break;
        }
    }
    if (fallbackContinent < 0) {
        for (int i = 0; i < static_cast<int>(zones.size()); i++) {
            if (zones[i].areaID == 0 && !isRootContinent(zones, i)) {
                fallbackContinent = i;
                break;
            }
        }
    }
    if (fallbackContinent >= 0) {
        loadZoneTextures(fallbackContinent);
        compositeZone(fallbackContinent);
        currentIdx = fallbackContinent;
        return;
    }

    // No root world texture available: clear to neutral background.
    currentIdx = -1;
    compositedIdx = -1;
    GLint prevFBO = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, FBO_W, FBO_H);
    glClearColor(0.05f, 0.08f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebuffer(GL_FRAMEBUFFER, prevFBO);
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
}

// --------------------------------------------------------
// Coordinate projection
// --------------------------------------------------------

glm::vec2 WorldMap::renderPosToMapUV(const glm::vec3& renderPos, int zoneIdx) const {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size()))
        return glm::vec2(0.5f, 0.5f);

    const auto& zone = zones[zoneIdx];

    // renderPos: x = wowY (west axis), y = wowX (north axis)
    float wowX = renderPos.y;  // north
    float wowY = renderPos.x;  // west

    float left = zone.locLeft;
    float right = zone.locRight;
    float top = zone.locTop;
    float bottom = zone.locBottom;
    if (zone.areaID == 0) {
        float l, r, t, b;
        if (getContinentProjectionBounds(zoneIdx, l, r, t, b)) {
            left = l; right = r; top = t; bottom = b;
        }
    }

    // WorldMapArea.dbc axis mapping:
    //   locLeft/locRight  contain wowX values (N/S), locLeft=north > locRight=south
    //   locTop/locBottom  contain wowY values (W/E), locTop=west  > locBottom=east
    // World map textures are laid out with axes transposed, so horizontal uses wowX.
    float denom_h = left - right;    // wowX span (N-S) → horizontal
    float denom_v = top - bottom;    // wowY span (W-E) → vertical

    if (std::abs(denom_h) < 0.001f || std::abs(denom_v) < 0.001f)
        return glm::vec2(0.5f, 0.5f);

    float u = (left - wowX) / denom_h;
    float v = (top  - wowY) / denom_v;

    // Continent overlay calibration: shift overlays/player marker upward.
    if (zone.areaID == 0) {
        constexpr float kVScale = 1.0f;
        constexpr float kVOffset = -0.15f; // ~15% upward total
        v = (v - 0.5f) * kVScale + 0.5f + kVOffset;
    }
    return glm::vec2(u, v);
}

// --------------------------------------------------------
// Main render
// --------------------------------------------------------

void WorldMap::render(const glm::vec3& playerRenderPos, int screenWidth, int screenHeight) {
    if (!initialized || !assetManager) return;

    auto& input = core::Input::getInstance();

    // When map is open, always allow M/Escape to close (bypass ImGui keyboard capture)
    if (open) {
        if (input.isKeyJustPressed(SDL_SCANCODE_M) ||
            input.isKeyJustPressed(SDL_SCANCODE_ESCAPE)) {
            open = false;
            return;
        }
    } else {
        auto& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard && input.isKeyJustPressed(SDL_SCANCODE_M)) {
            open = true;

            // Lazy-load zone data on first open
            if (zones.empty()) loadZonesFromDBC();

            int bestContinent = findBestContinentForPlayer(playerRenderPos);
            if (bestContinent >= 0 && bestContinent != continentIdx) {
                continentIdx = bestContinent;
                compositedIdx = -1;
            }

            // Ensure continent textures are loaded and composited
            if (continentIdx >= 0) {
                loadZoneTextures(continentIdx);
                compositeZone(continentIdx);
                currentIdx = continentIdx;
                viewLevel = ViewLevel::CONTINENT;
            }
        }
    }

    if (!open) return;

    renderImGuiOverlay(playerRenderPos, screenWidth, screenHeight);
}

// --------------------------------------------------------
// ImGui overlay
// --------------------------------------------------------

void WorldMap::renderImGuiOverlay(const glm::vec3& playerRenderPos, int screenWidth, int screenHeight) {
    float sw = static_cast<float>(screenWidth);
    float sh = static_cast<float>(screenHeight);

    // Full-screen dark background
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(sw, sh));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoBringToFrontOnFocus |
                             ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.75f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    if (ImGui::Begin("##WorldMap", nullptr, flags)) {
        // Map display area: maintain 4:3 aspect ratio, fit within ~85% of screen
        float mapAspect = static_cast<float>(FBO_W) / static_cast<float>(FBO_H);  // 1.333
        float availW = sw * 0.85f;
        float availH = sh * 0.85f;
        float displayW, displayH;
        if (availW / availH > mapAspect) {
            displayH = availH;
            displayW = availH * mapAspect;
        } else {
            displayW = availW;
            displayH = availW / mapAspect;
        }

        float mapX = (sw - displayW) / 2.0f;
        float mapY = (sh - displayH) / 2.0f;

        ImGui::SetCursorPos(ImVec2(mapX, mapY));
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(fboTexture)),
                     ImVec2(displayW, displayH), ImVec2(0, 0), ImVec2(1, 1));

        ImVec2 imgMin = ImGui::GetItemRectMin();
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        std::vector<int> continentIndices;
        bool hasLeafContinents = false;
        for (int i = 0; i < static_cast<int>(zones.size()); i++) {
            if (isLeafContinent(zones, i)) { hasLeafContinents = true; break; }
        }
        for (int i = 0; i < static_cast<int>(zones.size()); i++) {
            if (zones[i].areaID != 0) continue;
            if (hasLeafContinents) {
                if (isLeafContinent(zones, i)) continentIndices.push_back(i);
            } else if (!isRootContinent(zones, i)) {
                continentIndices.push_back(i);
            }
        }
        // If we have multiple continent choices, hide the root/world alias entry
        // (commonly "Azeroth") so picker only shows real continents.
        if (continentIndices.size() > 1) {
            std::vector<int> filtered;
            filtered.reserve(continentIndices.size());
            for (int idx : continentIndices) {
                if (zones[idx].areaName == mapName) continue;
                filtered.push_back(idx);
            }
            if (!filtered.empty()) continentIndices = std::move(filtered);
        }
        if (continentIndices.empty()) {
            for (int i = 0; i < static_cast<int>(zones.size()); i++) {
                if (zones[i].areaID == 0) continentIndices.push_back(i);
            }
        }

        // World-level continent selection UI.
        if (viewLevel == ViewLevel::WORLD && !continentIndices.empty()) {
            ImVec2 titleSz = ImGui::CalcTextSize("World");
            ImGui::SetCursorPos(ImVec2((sw - titleSz.x) * 0.5f, mapY + 8.0f));
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 0.95f), "World");

            ImGui::SetCursorPos(ImVec2(mapX + 8.0f, mapY + 32.0f));
            for (size_t i = 0; i < continentIndices.size(); i++) {
                int ci = continentIndices[i];
                if (i > 0) ImGui::SameLine();
                const bool selected = (ci == continentIdx);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.25f, 0.05f, 0.9f));
                }

                    std::string rawName = zones[ci].areaName.empty() ? "Continent" : zones[ci].areaName;
                    if (rawName == "Azeroth") rawName = "Eastern Kingdoms";
                    std::string label = rawName + "##" + std::to_string(ci);
                if (ImGui::Button(label.c_str())) {
                    continentIdx = ci;
                    loadZoneTextures(continentIdx);
                    compositeZone(continentIdx);
                    currentIdx = continentIdx;
                    viewLevel = ViewLevel::CONTINENT;
                }
                if (selected) ImGui::PopStyleColor();
            }
        } else if (viewLevel == ViewLevel::CONTINENT && continentIndices.size() > 1) {
            ImGui::SetCursorPos(ImVec2(mapX + 8.0f, mapY + 8.0f));
            for (size_t i = 0; i < continentIndices.size(); i++) {
                int ci = continentIndices[i];
                if (i > 0) ImGui::SameLine();
                const bool selected = (ci == continentIdx);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.25f, 0.05f, 0.9f));
                }

                std::string rawName = zones[ci].areaName.empty() ? "Continent" : zones[ci].areaName;
                if (rawName == "Azeroth") rawName = "Eastern Kingdoms";
                std::string label = rawName + "##" + std::to_string(ci);
                if (ImGui::Button(label.c_str())) {
                    continentIdx = ci;
                    loadZoneTextures(continentIdx);
                    compositeZone(continentIdx);
                    currentIdx = continentIdx;
                }
                if (selected) ImGui::PopStyleColor();
            }
        }

        // Player marker on current view
        if (currentIdx >= 0 && viewLevel != ViewLevel::WORLD) {
            glm::vec2 playerUV = renderPosToMapUV(playerRenderPos, currentIdx);

            if (playerUV.x >= 0.0f && playerUV.x <= 1.0f &&
                playerUV.y >= 0.0f && playerUV.y <= 1.0f) {
                float px = imgMin.x + playerUV.x * displayW;
                float py = imgMin.y + playerUV.y * displayH;

                drawList->AddCircleFilled(ImVec2(px, py), 6.0f,
                                          IM_COL32(255, 40, 40, 255));
                drawList->AddCircle(ImVec2(px, py), 6.0f,
                                    IM_COL32(0, 0, 0, 200), 0, 2.0f);
            }
        }

        // --- Continent view: show clickable zone overlays ---
        if (viewLevel == ViewLevel::CONTINENT && continentIdx >= 0) {
            const auto& cont = zones[continentIdx];
            // World map textures are transposed; match the same axis mapping as player UV.
            float cLeft = cont.locLeft;
            float cRight = cont.locRight;
            float cTop = cont.locTop;
            float cBottom = cont.locBottom;
            getContinentProjectionBounds(continentIdx, cLeft, cRight, cTop, cBottom);
            float cDenomU = cLeft - cRight;    // wowX span (N-S)
            float cDenomV = cTop - cBottom;    // wowY span (W-E)

            ImVec2 mousePos = ImGui::GetMousePos();
            int hoveredZone = -1;

            if (std::abs(cDenomU) > 0.001f && std::abs(cDenomV) > 0.001f) {
                for (int zi = 0; zi < static_cast<int>(zones.size()); zi++) {
                    if (!zoneBelongsToContinent(zi, continentIdx)) continue;

                    const auto& z = zones[zi];

                    // Skip zones with zero-size bounds
                    if (std::abs(z.locLeft - z.locRight) < 0.001f ||
                        std::abs(z.locTop - z.locBottom) < 0.001f) continue;

                    // Project zone bounds to continent UV
                    // u axis (left->right): north->south
                    float zuMin = (cLeft - z.locLeft) / cDenomU;      // zone north edge
                    float zuMax = (cLeft - z.locRight) / cDenomU;     // zone south edge
                    // v axis (top->bottom): west->east
                    float zvMin = (cTop - z.locTop) / cDenomV;        // zone west edge
                    float zvMax = (cTop - z.locBottom) / cDenomV;     // zone east edge

                    // Slightly shrink DBC AABB overlays to reduce heavy overlap.
                    constexpr float kOverlayShrink = 0.92f;
                    float cu = (zuMin + zuMax) * 0.5f;
                    float cv = (zvMin + zvMax) * 0.5f;
                    float hu = (zuMax - zuMin) * 0.5f * kOverlayShrink;
                    float hv = (zvMax - zvMin) * 0.5f * kOverlayShrink;
                    zuMin = cu - hu;
                    zuMax = cu + hu;
                    zvMin = cv - hv;
                    zvMax = cv + hv;

                    // Continent overlay calibration (matches player marker calibration).
                    constexpr float kVScale = 1.0f;
                    constexpr float kVOffset = -0.15f;
                    zvMin = (zvMin - 0.5f) * kVScale + 0.5f + kVOffset;
                    zvMax = (zvMax - 0.5f) * kVScale + 0.5f + kVOffset;

                    // Clamp to [0,1]
                    zuMin = std::clamp(zuMin, 0.0f, 1.0f);
                    zuMax = std::clamp(zuMax, 0.0f, 1.0f);
                    zvMin = std::clamp(zvMin, 0.0f, 1.0f);
                    zvMax = std::clamp(zvMax, 0.0f, 1.0f);

                    // Skip tiny or degenerate zones
                    if (zuMax - zuMin < 0.001f || zvMax - zvMin < 0.001f) continue;

                    // Convert to screen coordinates
                    float sx0 = imgMin.x + zuMin * displayW;
                    float sy0 = imgMin.y + zvMin * displayH;
                    float sx1 = imgMin.x + zuMax * displayW;
                    float sy1 = imgMin.y + zvMax * displayH;

                    // Check hover
                    bool hovered = (mousePos.x >= sx0 && mousePos.x <= sx1 &&
                                    mousePos.y >= sy0 && mousePos.y <= sy1);

                    if (hovered) {
                        hoveredZone = zi;
                        drawList->AddRectFilled(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                                IM_COL32(255, 255, 200, 40));
                        drawList->AddRect(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                          IM_COL32(255, 215, 0, 180), 0.0f, 0, 2.0f);
                    } else {
                        drawList->AddRect(ImVec2(sx0, sy0), ImVec2(sx1, sy1),
                                          IM_COL32(255, 255, 255, 30), 0.0f, 0, 1.0f);
                    }
                }
            }

            // Zone name tooltip
            if (hoveredZone >= 0) {
                ImGui::SetTooltip("%s", zones[hoveredZone].areaName.c_str());

                // Click to zoom into zone
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    loadZoneTextures(hoveredZone);
                    compositeZone(hoveredZone);
                    currentIdx = hoveredZone;
                    viewLevel = ViewLevel::ZONE;
                }
            }
        }

        // --- Zone view: back to continent ---
        if (viewLevel == ViewLevel::ZONE && continentIdx >= 0) {
            // Right-click or Back button
            auto& io = ImGui::GetIO();
            bool goBack = io.MouseClicked[1]; // right-click (direct IO check)

            // "< Back" button in top-left of map area
            ImGui::SetCursorPos(ImVec2(mapX + 8.0f, mapY + 8.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));
            if (ImGui::Button("< Back")) goBack = true;
            ImGui::PopStyleColor(3);

            if (goBack) {
                compositeZone(continentIdx);
                currentIdx = continentIdx;
                viewLevel = ViewLevel::CONTINENT;
            }

            // Zone name header
            const char* zoneName = zones[currentIdx].areaName.c_str();
            ImVec2 nameSize = ImGui::CalcTextSize(zoneName);
            float nameY = mapY - nameSize.y - 8.0f;
            if (nameY > 0.0f) {
                ImGui::SetCursorPos(ImVec2((sw - nameSize.x) / 2.0f, nameY));
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 0.9f), "%s", zoneName);
            }
        }

        // --- Continent view: back to world ---
        if (viewLevel == ViewLevel::CONTINENT) {
            auto& io = ImGui::GetIO();
            bool goWorld = io.MouseClicked[1];

            float worldBtnY = mapY + (continentIndices.size() > 1 ? 40.0f : 8.0f);
            ImGui::SetCursorPos(ImVec2(mapX + 8.0f, worldBtnY));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));
            if (ImGui::Button("< World")) goWorld = true;
            ImGui::PopStyleColor(3);

            if (goWorld) enterWorldView();
        }

        // Help text
        const char* helpText;
        if (viewLevel == ViewLevel::ZONE) {
            helpText = "Right-click to zoom out | M or Escape to close";
        } else if (viewLevel == ViewLevel::WORLD) {
            helpText = "Select a continent | M or Escape to close";
        } else {
            helpText = "Click a zone to zoom in | Right-click for World | M or Escape to close";
        }
        ImVec2 textSize = ImGui::CalcTextSize(helpText);
        float textY = mapY + displayH + 8.0f;
        if (textY + textSize.y < sh) {
            ImGui::SetCursorPos(ImVec2((sw - textSize.x) / 2.0f, textY));
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.8f), "%s", helpText);
        }
    }
    ImGui::End();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

} // namespace rendering
} // namespace wowee
