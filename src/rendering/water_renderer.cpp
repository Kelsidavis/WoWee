#include "rendering/water_renderer.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {

WaterRenderer::WaterRenderer() = default;

WaterRenderer::~WaterRenderer() {
    shutdown();
}

bool WaterRenderer::initialize() {
    LOG_INFO("Initializing water renderer");

    // Create water shader
    waterShader = std::make_unique<Shader>();

    // Vertex shader
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec2 aTexCoord;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        uniform float time;
        uniform float waveAmp;
        uniform float waveFreq;
        uniform float waveSpeed;

        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;
        out float WaveOffset;

        void main() {
            vec3 pos = aPos;

            // Pseudo-random phase offsets to break up regular pattern
            float hash1 = fract(sin(dot(aPos.xy, vec2(12.9898, 78.233))) * 43758.5453);
            float hash2 = fract(sin(dot(aPos.xy, vec2(93.9898, 67.345))) * 27153.5328);

            // Multiple wave octaves with randomized phases for natural variation
            float w1 = sin((aPos.x + time * waveSpeed + hash1 * 6.28) * waveFreq) * waveAmp;
            float w2 = cos((aPos.y - time * (waveSpeed * 0.78) + hash2 * 6.28) * (waveFreq * 0.82)) * (waveAmp * 0.72);

            // Add higher frequency detail waves (smaller amplitude)
            float w3 = sin((aPos.x * 1.7 - time * waveSpeed * 1.3 + hash1 * 3.14) * waveFreq * 2.1) * (waveAmp * 0.35);
            float w4 = cos((aPos.y * 1.4 + time * waveSpeed * 0.9 + hash2 * 3.14) * waveFreq * 1.8) * (waveAmp * 0.28);

            float wave = w1 + w2 + w3 + w4;
            pos.z += wave;

            FragPos = vec3(model * vec4(pos, 1.0));
            // Use mat3(model) directly - avoids expensive inverse() per vertex
            Normal = mat3(model) * aNormal;
            TexCoord = aTexCoord;
            WaveOffset = wave;

            gl_Position = projection * view * vec4(FragPos, 1.0);
        }
    )";

    // Fragment shader
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;
        in float WaveOffset;

        uniform vec3 viewPos;
        uniform vec4 waterColor;
        uniform float waterAlpha;
        uniform float time;
        uniform float shimmerStrength;
        uniform float alphaScale;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        out vec4 FragColor;

        void main() {
            // Normalize interpolated normal
            vec3 norm = normalize(Normal);

            // Simple directional light (sun)
            vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
            float diff = max(dot(norm, lightDir), 0.0);

            // Specular highlights (shininess for water)
            vec3 viewDir = normalize(viewPos - FragPos);
            vec3 reflectDir = reflect(-lightDir, norm);
            float specBase = pow(max(dot(viewDir, reflectDir), 0.0), mix(64.0, 180.0, shimmerStrength));
            float sparkle = 0.65 + 0.35 * sin((TexCoord.x + TexCoord.y + time * 0.4) * 80.0);
            float spec = specBase * mix(1.0, sparkle, shimmerStrength);

            // Animated texture coordinates for flowing effect
            vec2 uv1 = TexCoord + vec2(time * 0.02, time * 0.01);
            vec2 uv2 = TexCoord + vec2(-time * 0.01, time * 0.015);

            // Combine lighting
            vec3 ambient = vec3(0.3) * waterColor.rgb;
            vec3 diffuse = vec3(0.6) * diff * waterColor.rgb;
            vec3 specular = vec3(1.0) * spec;

            // Add wave offset to brightness
            float brightness = 1.0 + WaveOffset * 0.1;

            vec3 result = (ambient + diffuse + specular) * brightness;
            // Add a subtle sky tint and luminance floor so large ocean sheets
            // never turn black at grazing angles.
            float horizon = pow(1.0 - max(dot(norm, viewDir), 0.0), 1.6);
            vec3 skyTint = vec3(0.22, 0.35, 0.48) * (0.25 + 0.55 * shimmerStrength) * horizon;
            result += skyTint;
            result = max(result, waterColor.rgb * 0.24);

            // Subtle foam on wave crests only (no grid artifacts)
            float wavePeak = smoothstep(0.35, 0.6, WaveOffset);  // Only highest peaks
            float foam = wavePeak * 0.25;  // Subtle white highlight
            result += vec3(foam);

            // Slight fresnel: more reflective/opaque at grazing angles.
            float fresnel = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);

            // Distance-based opacity: distant water is more opaque to hide underwater objects
            float dist = length(viewPos - FragPos);
            float distFade = smoothstep(40.0, 300.0, dist);  // Start at 40 units, full opaque at 300
            float distAlpha = mix(0.0, 0.75, distFade);  // Add up to 75% opacity at distance

            float alpha = clamp(waterAlpha * alphaScale * (0.80 + fresnel * 0.45) + distAlpha, 0.20, 0.98);

            // Apply distance fog
            float fogDist = length(viewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            vec3 finalColor = mix(uFogColor, result, fogFactor);

            FragColor = vec4(finalColor, alpha);
        }
    )";

    if (!waterShader->loadFromSource(vertexShaderSource, fragmentShaderSource)) {
        LOG_ERROR("Failed to create water shader");
        return false;
    }

    LOG_INFO("Water renderer initialized");
    return true;
}

void WaterRenderer::shutdown() {
    clear();
    waterShader.reset();
}

void WaterRenderer::loadFromTerrain(const pipeline::ADTTerrain& terrain, bool append,
                                     int tileX, int tileY) {
    constexpr float TILE_SIZE = 33.33333f / 8.0f;

    if (!append) {
        LOG_INFO("Loading water from terrain (replacing)");
        clear();
    } else {
        LOG_INFO("Loading water from terrain (appending)");
    }

    // Load water surfaces from MH2O data
    int totalLayers = 0;

    for (int chunkIdx = 0; chunkIdx < 256; chunkIdx++) {
        const auto& chunkWater = terrain.waterData[chunkIdx];

        if (!chunkWater.hasWater()) {
            continue;
        }

        // Get the terrain chunk for position reference
        int chunkX = chunkIdx % 16;
        int chunkY = chunkIdx / 16;
        const auto& terrainChunk = terrain.getChunk(chunkX, chunkY);

        // Process each water layer in this chunk
        for (const auto& layer : chunkWater.layers) {
            WaterSurface surface;

            // Use the chunk base position - layer offsets will be applied in mesh generation
            // to match terrain's coordinate transformation
            surface.position = glm::vec3(
                terrainChunk.position[0],
                terrainChunk.position[1],
                layer.minHeight
            );
            surface.origin = glm::vec3(
                surface.position.x - (static_cast<float>(layer.y) * TILE_SIZE),
                surface.position.y - (static_cast<float>(layer.x) * TILE_SIZE),
                layer.minHeight
            );
            surface.stepX = glm::vec3(0.0f, -TILE_SIZE, 0.0f);
            surface.stepY = glm::vec3(-TILE_SIZE, 0.0f, 0.0f);

            // Debug log first few water surfaces
            if (totalLayers < 5) {
                LOG_DEBUG("Water layer ", totalLayers, ": chunk=", chunkIdx,
                         " liquidType=", layer.liquidType,
                         " offset=(", (int)layer.x, ",", (int)layer.y, ")",
                         " size=", (int)layer.width, "x", (int)layer.height,
                         " height range=[", layer.minHeight, ",", layer.maxHeight, "]");
            }

            surface.minHeight = layer.minHeight;
            surface.maxHeight = layer.maxHeight;
            surface.liquidType = layer.liquidType;

            // Store dimensions
            surface.xOffset = layer.x;
            surface.yOffset = layer.y;
            surface.width = layer.width;
            surface.height = layer.height;

            // Prefer per-vertex terrain water heights when sane; fall back to flat
            // minHeight if data looks malformed (prevents sky-stretch artifacts).
            size_t numVertices = (layer.width + 1) * (layer.height + 1);
            bool useFlat = true;
            if (layer.heights.size() == numVertices) {
                bool sane = true;
                for (float h : layer.heights) {
                    if (!std::isfinite(h) || std::abs(h) > 50000.0f) {
                        sane = false;
                        break;
                    }
                    // Conservative acceptance window around MH2O min/max metadata.
                    if (h < layer.minHeight - 8.0f || h > layer.maxHeight + 8.0f) {
                        sane = false;
                        break;
                    }
                }
                if (sane) {
                    useFlat = false;
                    surface.heights = layer.heights;
                }
            }
            if (useFlat) {
                surface.heights.resize(numVertices, layer.minHeight);
            }

            // Lower all terrain water in Stormwind area to prevent it from showing in tunnels/buildings/parks
            // Only apply to Stormwind to avoid affecting water elsewhere
            // Expanded bounds to cover all of Stormwind including outlying areas and park
            bool isStormwindArea = (tileX >= 28 && tileX <= 50 && tileY >= 28 && tileY <= 52);
            // Only lower high water (canal level >94) to avoid affecting moonwell and other low features
            if (isStormwindArea && layer.minHeight > 94.0f) {
                // Calculate approximate world position from tile coordinates
                float tileWorldX = (32.0f - tileX) * 533.33333f;
                float tileWorldY = (32.0f - tileY) * 533.33333f;

                // Exclude moonwell area at (-8755.9, 1108.9) - don't lower water within 50 units
                glm::vec3 moonwellPos(-8755.9f, 1108.9f, 96.1f);
                float distToMoonwell = glm::distance(glm::vec2(tileWorldX, tileWorldY),
                                                      glm::vec2(moonwellPos.x, moonwellPos.y));

                if (distToMoonwell > 300.0f) {  // Terrain tiles are large, use bigger exclusion radius
                    LOG_INFO("  -> LOWERING water at tile (", tileX, ",", tileY, ") from height ", layer.minHeight, " by 1 unit");
                    for (float& h : surface.heights) {
                        h -= 1.0f;
                    }
                    surface.minHeight -= 1.0f;
                    surface.maxHeight -= 1.0f;
                } else {
                    LOG_INFO("  -> SKIPPING tile (", tileX, ",", tileY, ") - moonwell exclusion (dist: ", distToMoonwell, ")");
                }
            }

            // Copy render mask
            surface.mask = layer.mask;

            surface.tileX = tileX;
            surface.tileY = tileY;
            createWaterMesh(surface);
            surfaces.push_back(surface);
            totalLayers++;
        }
    }

    LOG_INFO("Loaded ", totalLayers, " water layers from MH2O data");
}

void WaterRenderer::removeTile(int tileX, int tileY) {
    int removed = 0;
    auto it = surfaces.begin();
    while (it != surfaces.end()) {
        if (it->tileX == tileX && it->tileY == tileY) {
            destroyWaterMesh(*it);
            it = surfaces.erase(it);
            removed++;
        } else {
            ++it;
        }
    }
    if (removed > 0) {
        LOG_DEBUG("Removed ", removed, " water surfaces for tile [", tileX, ",", tileY, "]");
    }
}

void WaterRenderer::loadFromWMO([[maybe_unused]] const pipeline::WMOLiquid& liquid,
                                 [[maybe_unused]] const glm::mat4& modelMatrix,
                                 [[maybe_unused]] uint32_t wmoId) {
    if (!liquid.hasLiquid() || liquid.xTiles == 0 || liquid.yTiles == 0) {
        return;
    }
    if (liquid.xVerts < 2 || liquid.yVerts < 2) {
        return;
    }
    if (liquid.xTiles != liquid.xVerts - 1 || liquid.yTiles != liquid.yVerts - 1) {
        return;
    }
    if (liquid.xTiles > 64 || liquid.yTiles > 64) {
        return;
    }

    WaterSurface surface;
    surface.tileX = -1;
    surface.tileY = -1;
    surface.wmoId = wmoId;
    surface.liquidType = liquid.materialId;
    surface.xOffset = 0;
    surface.yOffset = 0;
    surface.width = static_cast<uint8_t>(std::min<uint32_t>(255, liquid.xTiles));
    surface.height = static_cast<uint8_t>(std::min<uint32_t>(255, liquid.yTiles));

    constexpr float WMO_LIQUID_TILE_SIZE = 4.1666625f;
    const glm::vec3 localBase(liquid.basePosition.x, liquid.basePosition.y, liquid.basePosition.z);
    const glm::vec3 localStepX(WMO_LIQUID_TILE_SIZE, 0.0f, 0.0f);
    const glm::vec3 localStepY(0.0f, WMO_LIQUID_TILE_SIZE, 0.0f);

    surface.origin = glm::vec3(modelMatrix * glm::vec4(localBase, 1.0f));
    surface.stepX = glm::vec3(modelMatrix * glm::vec4(localStepX, 0.0f));
    surface.stepY = glm::vec3(modelMatrix * glm::vec4(localStepY, 0.0f));
    surface.position = surface.origin;
    // Guard against malformed transforms that produce giant/vertical sheets.
    float stepXLen = glm::length(surface.stepX);
    float stepYLen = glm::length(surface.stepY);
    glm::vec3 planeN = glm::cross(surface.stepX, surface.stepY);
    float nz = (glm::length(planeN) > 1e-4f) ? std::abs(glm::normalize(planeN).z) : 0.0f;
    float spanX = stepXLen * static_cast<float>(surface.width);
    float spanY = stepYLen * static_cast<float>(surface.height);
    if (stepXLen < 0.2f || stepXLen > 12.0f ||
        stepYLen < 0.2f || stepYLen > 12.0f ||
        nz < 0.60f ||
        spanX > 450.0f || spanY > 450.0f) {
        return;
    }

    const int gridWidth = static_cast<int>(surface.width) + 1;
    const int gridHeight = static_cast<int>(surface.height) + 1;
    const int vertexCount = gridWidth * gridHeight;
    // Keep WMO liquid flat for stability; some files use variant payload layouts
    // that can produce invalid per-vertex heights if interpreted generically.
    surface.heights.assign(vertexCount, surface.origin.z);
    surface.minHeight = surface.origin.z;
    surface.maxHeight = surface.origin.z;

    // Lower WMO water in Stormwind area to prevent it from showing in tunnels/buildings/parks
    // Calculate tile coordinates from world position
    int tileX = static_cast<int>(std::floor((32.0f - surface.origin.x / 533.33333f)));
    int tileY = static_cast<int>(std::floor((32.0f - surface.origin.y / 533.33333f)));

    // Log all WMO water to debug park issue
    LOG_INFO("WMO water at pos=(", surface.origin.x, ",", surface.origin.y, ",", surface.origin.z,
             ") tile=(", tileX, ",", tileY, ") wmoId=", wmoId);

    // Expanded bounds to cover all of Stormwind including outlying areas and park
    bool isStormwindArea = (tileX >= 28 && tileX <= 50 && tileY >= 28 && tileY <= 52);

    // Only lower high WMO water (canal level >94) to avoid affecting moonwell and other low features
    if (isStormwindArea && surface.origin.z > 94.0f) {
        // Exclude moonwell area at (-8755.9, 1108.9) - don't lower water within 20 units
        glm::vec3 moonwellPos(-8755.9f, 1108.9f, 96.1f);
        float distToMoonwell = glm::distance(glm::vec2(surface.origin.x, surface.origin.y),
                                              glm::vec2(moonwellPos.x, moonwellPos.y));

        if (distToMoonwell > 20.0f) {
            LOG_INFO("  -> LOWERING by 1 unit (dist to moonwell: ", distToMoonwell, ")");
            for (float& h : surface.heights) {
                h -= 1.0f;
            }
            surface.minHeight -= 1.0f;
            surface.maxHeight -= 1.0f;
        } else {
            LOG_INFO("  -> SKIPPING (moonwell exclusion zone, dist: ", distToMoonwell, ")");
        }
    }

    // Skip WMO water that's clearly invalid (extremely high - above 300 units)
    // This is a conservative global filter that won't affect normal gameplay
    if (surface.origin.z > 300.0f) {
        LOG_INFO("WMO water filtered: height=", surface.origin.z, " wmoId=", wmoId, " (too high)");
        return;
    }

    // Skip WMO water that's extremely low (deep underground where it shouldn't be)
    if (surface.origin.z < -100.0f) {
        LOG_INFO("WMO water filtered: height=", surface.origin.z, " wmoId=", wmoId, " (too low)");
        return;
    }

    size_t tileCount = static_cast<size_t>(surface.width) * static_cast<size_t>(surface.height);
    size_t maskBytes = (tileCount + 7) / 8;
    // WMO liquid flags vary across files; for now treat all WMO liquid tiles as
    // visible for rendering. Swim/gameplay queries already ignore WMO surfaces.
    surface.mask.assign(maskBytes, 0xFF);

    createWaterMesh(surface);
    if (surface.indexCount > 0) {
        surfaces.push_back(surface);
    }
}

void WaterRenderer::removeWMO(uint32_t wmoId) {
    if (wmoId == 0) {
        return;
    }

    auto it = surfaces.begin();
    while (it != surfaces.end()) {
        if (it->wmoId == wmoId) {
            destroyWaterMesh(*it);
            it = surfaces.erase(it);
        } else {
            ++it;
        }
    }
}

void WaterRenderer::clear() {
    for (auto& surface : surfaces) {
        destroyWaterMesh(surface);
    }
    surfaces.clear();
}

void WaterRenderer::render(const Camera& camera, float time) {
    if (!renderingEnabled || surfaces.empty() || !waterShader) {
        return;
    }

    glDisable(GL_CULL_FACE);

    // Enable alpha blending for transparent water
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Disable depth writing so terrain shows through water
    glDepthMask(GL_FALSE);

    waterShader->use();

    // Set uniforms
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();

    waterShader->setUniform("view", view);
    waterShader->setUniform("projection", projection);
    waterShader->setUniform("viewPos", camera.getPosition());
    waterShader->setUniform("time", time);
    waterShader->setUniform("uFogColor", fogColor);
    waterShader->setUniform("uFogStart", fogStart);
    waterShader->setUniform("uFogEnd", fogEnd);

    // Render each water surface
    for (const auto& surface : surfaces) {
        if (surface.vao == 0) {
            continue;
        }

        // Model matrix (identity, position already in vertices)
        glm::mat4 model = glm::mat4(1.0f);
        waterShader->setUniform("model", model);

        // Set liquid-specific color and alpha
        glm::vec4 color = getLiquidColor(surface.liquidType);
        float alpha = getLiquidAlpha(surface.liquidType);

        // City/canal liquid profile: clearer water + stronger ripples/sun shimmer.
        // Stormwind canals typically use LiquidType 5 in this data set.
        bool canalProfile = (surface.wmoId != 0) || (surface.liquidType == 5);
        // Reduced wave amplitude to prevent tile seam gaps (tiles don't share wave state)
        float waveAmp = canalProfile ? 0.04f : 0.06f;      // Subtle waves to avoid boundary gaps
        float waveFreq = canalProfile ? 0.30f : 0.22f;     // Frequency maintained for visual
        float waveSpeed = canalProfile ? 1.20f : 2.00f;    // Speed maintained for animation
        float shimmerStrength = canalProfile ? 0.95f : 0.50f;
        float alphaScale = canalProfile ? 0.90f : 1.00f;   // Increased from 0.72 to make canal water less transparent

        waterShader->setUniform("waterColor", color);
        waterShader->setUniform("waterAlpha", alpha);
        waterShader->setUniform("waveAmp", waveAmp);
        waterShader->setUniform("waveFreq", waveFreq);
        waterShader->setUniform("waveSpeed", waveSpeed);
        waterShader->setUniform("shimmerStrength", shimmerStrength);
        waterShader->setUniform("alphaScale", alphaScale);

        // Render
        glBindVertexArray(surface.vao);
        glDrawElements(GL_TRIANGLES, surface.indexCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    // Restore state
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void WaterRenderer::createWaterMesh(WaterSurface& surface) {
    // Variable-size grid based on water layer dimensions
    const int gridWidth = surface.width + 1;   // Vertices = tiles + 1
    const int gridHeight = surface.height + 1;
    constexpr float VISUAL_WATER_Z_BIAS = 0.02f;  // Small bias to avoid obvious overdraw on city meshes

    std::vector<float> vertices;
    std::vector<uint32_t> indices;

    // Generate vertices
    for (int y = 0; y < gridHeight; y++) {
        for (int x = 0; x < gridWidth; x++) {
            int index = y * gridWidth + x;

            // Use per-vertex height data if available, otherwise flat at minHeight
            float height;
            if (index < static_cast<int>(surface.heights.size())) {
                height = surface.heights[index];
            } else {
                height = surface.minHeight;
            }

            glm::vec3 pos = surface.origin +
                            surface.stepX * static_cast<float>(x) +
                            surface.stepY * static_cast<float>(y);
            pos.z = height + VISUAL_WATER_Z_BIAS;

            // Debug first surface's corner vertices
            static int debugCount = 0;
            if (debugCount < 4 && (x == 0 || x == gridWidth-1) && (y == 0 || y == gridHeight-1)) {
                LOG_DEBUG("Water vertex: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                debugCount++;
            }

            vertices.push_back(pos.x);
            vertices.push_back(pos.y);
            vertices.push_back(pos.z);

            // Normal (pointing up for water surface)
            vertices.push_back(0.0f);
            vertices.push_back(0.0f);
            vertices.push_back(1.0f);

            // Texture coordinates
            vertices.push_back(static_cast<float>(x) / std::max(1, gridWidth - 1));
            vertices.push_back(static_cast<float>(y) / std::max(1, gridHeight - 1));
        }
    }

    // Generate indices (triangles), respecting the render mask
    for (int y = 0; y < gridHeight - 1; y++) {
        for (int x = 0; x < gridWidth - 1; x++) {
            // Check render mask - each bit represents a tile
            // Also render edge tiles to blend coastlines (avoid square gaps)
            bool renderTile = true;
            if (!surface.mask.empty()) {
                int tileIndex;
                if (surface.wmoId == 0 && surface.mask.size() >= 8) {
                    // Terrain MH2O mask is chunk-wide 8x8.
                    int cx = static_cast<int>(surface.xOffset) + x;
                    int cy = static_cast<int>(surface.yOffset) + y;
                    tileIndex = cy * 8 + cx;
                } else {
                    // Local mask indexing (WMO/custom).
                    tileIndex = y * surface.width + x;
                }
                int byteIndex = tileIndex / 8;
                int bitIndex = tileIndex % 8;
                if (byteIndex < static_cast<int>(surface.mask.size())) {
                    uint8_t maskByte = surface.mask[byteIndex];
                    bool lsbOrder = (maskByte & (1 << bitIndex)) != 0;
                    bool msbOrder = (maskByte & (1 << (7 - bitIndex))) != 0;
                    renderTile = lsbOrder || msbOrder;

                    // If this tile is masked out, check neighbors to fill gaps
                    if (!renderTile && x > 0 && y > 0 && x < gridWidth-2 && y < gridHeight-2) {
                        // Check adjacent tiles - render if any neighbor is water (blend coastline)
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                if (dx == 0 && dy == 0) continue;
                                int neighborIdx = (y + dy) * surface.width + (x + dx);
                                int nByteIdx = neighborIdx / 8;
                                int nBitIdx = neighborIdx % 8;
                                if (nByteIdx < static_cast<int>(surface.mask.size())) {
                                    uint8_t nMask = surface.mask[nByteIdx];
                                    if ((nMask & (1 << nBitIdx)) || (nMask & (1 << (7 - nBitIdx)))) {
                                        renderTile = true;
                                        goto found_neighbor;
                                    }
                                }
                            }
                        }
                        found_neighbor:;
                    }
                }
            }

            if (!renderTile) {
                continue;  // Skip this tile
            }

            int topLeft = y * gridWidth + x;
            int topRight = topLeft + 1;
            int bottomLeft = (y + 1) * gridWidth + x;
            int bottomRight = bottomLeft + 1;

            // First triangle
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            // Second triangle
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    if (indices.empty() && surface.wmoId == 0) {
        // Terrain MH2O masks can be inconsistent in some tiles. If a terrain layer
        // produces no visible tiles, fall back to its full local rect for rendering.
        for (int y = 0; y < gridHeight - 1; y++) {
            for (int x = 0; x < gridWidth - 1; x++) {
                int topLeft = y * gridWidth + x;
                int topRight = topLeft + 1;
                int bottomLeft = (y + 1) * gridWidth + x;
                int bottomRight = bottomLeft + 1;
                indices.push_back(topLeft);
                indices.push_back(bottomLeft);
                indices.push_back(topRight);
                indices.push_back(topRight);
                indices.push_back(bottomLeft);
                indices.push_back(bottomRight);
            }
        }
    }

    if (indices.empty()) return;

    surface.indexCount = static_cast<int>(indices.size());

    // Create OpenGL buffers
    glGenVertexArrays(1, &surface.vao);
    glGenBuffers(1, &surface.vbo);
    glGenBuffers(1, &surface.ebo);

    glBindVertexArray(surface.vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, surface.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, surface.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    // Set vertex attributes
    // Position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Texture coordinates
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void WaterRenderer::destroyWaterMesh(WaterSurface& surface) {
    if (surface.vao != 0) {
        glDeleteVertexArrays(1, &surface.vao);
        surface.vao = 0;
    }
    if (surface.vbo != 0) {
        glDeleteBuffers(1, &surface.vbo);
        surface.vbo = 0;
    }
    if (surface.ebo != 0) {
        glDeleteBuffers(1, &surface.ebo);
        surface.ebo = 0;
    }
}

std::optional<float> WaterRenderer::getWaterHeightAt(float glX, float glY) const {
    std::optional<float> best;

    for (size_t si = 0; si < surfaces.size(); si++) {
        const auto& surface = surfaces[si];
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 stepX(surface.stepX.x, surface.stepX.y);
        glm::vec2 stepY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(stepX, stepX);
        float lenSqY = glm::dot(stepY, stepY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) {
            continue;
        }
        float gx = glm::dot(rel, stepX) / lenSqX;
        float gy = glm::dot(rel, stepY) / lenSqY;

        if (gx < 0.0f || gx > static_cast<float>(surface.width) ||
            gy < 0.0f || gy > static_cast<float>(surface.height)) {
            continue;
        }

        int gridWidth = surface.width + 1;

        // Bilinear interpolation
        int ix = static_cast<int>(gx);
        int iy = static_cast<int>(gy);
        float fx = gx - ix;
        float fy = gy - iy;

        // Clamp to valid vertex range
        if (ix >= surface.width) { ix = surface.width - 1; fx = 1.0f; }
        if (iy >= surface.height) { iy = surface.height - 1; fy = 1.0f; }
        if (ix < 0 || iy < 0) {
            continue;
        }

        // Respect per-tile mask so holes/non-liquid tiles do not count as swimmable.
        if (!surface.mask.empty()) {
            int tileIndex;
            if (surface.wmoId == 0 && surface.mask.size() >= 8) {
                int cx = static_cast<int>(surface.xOffset) + ix;
                int cy = static_cast<int>(surface.yOffset) + iy;
                tileIndex = cy * 8 + cx;
            } else {
                tileIndex = iy * surface.width + ix;
            }
            int byteIndex = tileIndex / 8;
            int bitIndex = tileIndex % 8;
            if (byteIndex < static_cast<int>(surface.mask.size())) {
                uint8_t maskByte = surface.mask[byteIndex];
                bool lsbOrder = (maskByte & (1 << bitIndex)) != 0;
                bool msbOrder = (maskByte & (1 << (7 - bitIndex))) != 0;
                bool renderTile = lsbOrder || msbOrder;
                if (!renderTile) {
                    continue;
                }
            }
        }

        int idx00 = iy * gridWidth + ix;
        int idx10 = idx00 + 1;
        int idx01 = idx00 + gridWidth;
        int idx11 = idx01 + 1;

        int total = static_cast<int>(surface.heights.size());
        if (idx11 >= total) continue;

        float h00 = surface.heights[idx00];
        float h10 = surface.heights[idx10];
        float h01 = surface.heights[idx01];
        float h11 = surface.heights[idx11];

        float h = h00 * (1-fx) * (1-fy) + h10 * fx * (1-fy) +
                  h01 * (1-fx) * fy     + h11 * fx * fy;

        if (!best || h > *best) {
            best = h;
        }
    }

    return best;
}

std::optional<uint16_t> WaterRenderer::getWaterTypeAt(float glX, float glY) const {
    std::optional<float> bestHeight;
    std::optional<uint16_t> bestType;

    for (const auto& surface : surfaces) {
        glm::vec2 rel(glX - surface.origin.x, glY - surface.origin.y);
        glm::vec2 stepX(surface.stepX.x, surface.stepX.y);
        glm::vec2 stepY(surface.stepY.x, surface.stepY.y);
        float lenSqX = glm::dot(stepX, stepX);
        float lenSqY = glm::dot(stepY, stepY);
        if (lenSqX < 1e-6f || lenSqY < 1e-6f) {
            continue;
        }

        float gx = glm::dot(rel, stepX) / lenSqX;
        float gy = glm::dot(rel, stepY) / lenSqY;
        if (gx < 0.0f || gx > static_cast<float>(surface.width) ||
            gy < 0.0f || gy > static_cast<float>(surface.height)) {
            continue;
        }

        int ix = static_cast<int>(gx);
        int iy = static_cast<int>(gy);
        if (ix >= surface.width) ix = surface.width - 1;
        if (iy >= surface.height) iy = surface.height - 1;
        if (ix < 0 || iy < 0) continue;

        if (!surface.mask.empty()) {
            int tileIndex;
            if (surface.wmoId == 0 && surface.mask.size() >= 8) {
                int cx = static_cast<int>(surface.xOffset) + ix;
                int cy = static_cast<int>(surface.yOffset) + iy;
                tileIndex = cy * 8 + cx;
            } else {
                tileIndex = iy * surface.width + ix;
            }
            int byteIndex = tileIndex / 8;
            int bitIndex = tileIndex % 8;
            if (byteIndex < static_cast<int>(surface.mask.size())) {
                uint8_t maskByte = surface.mask[byteIndex];
                bool lsbOrder = (maskByte & (1 << bitIndex)) != 0;
                bool msbOrder = (maskByte & (1 << (7 - bitIndex))) != 0;
                bool renderTile = lsbOrder || msbOrder;
                if (!renderTile) continue;
            }
        }

        // Use minHeight as stable selector for "topmost surface at XY".
        float h = surface.minHeight;
        if (!bestHeight || h > *bestHeight) {
            bestHeight = h;
            bestType = surface.liquidType;
        }
    }

    return bestType;
}

glm::vec4 WaterRenderer::getLiquidColor(uint16_t liquidType) const {
    // WoW 3.3.5a LiquidType.dbc IDs:
    // 1,5,9,13,17 = Water variants (still, slow, fast)
    // 2,6,10,14   = Ocean
    // 3,7,11,15   = Magma
    // 4,8,12      = Slime
    // Map to basic type using (id - 1) % 4 for standard IDs, or handle ranges
    uint8_t basicType;
    if (liquidType == 0) {
        basicType = 0;  // Water (fallback)
    } else {
        basicType = ((liquidType - 1) % 4);
    }

    switch (basicType) {
        case 0:  // Water
            return glm::vec4(0.2f, 0.4f, 0.6f, 1.0f);
        case 1:  // Ocean
            return glm::vec4(0.06f, 0.18f, 0.34f, 1.0f);
        case 2:  // Magma
            return glm::vec4(0.9f, 0.3f, 0.05f, 1.0f);
        case 3:  // Slime
            return glm::vec4(0.2f, 0.6f, 0.1f, 1.0f);
        default:
            return glm::vec4(0.2f, 0.4f, 0.6f, 1.0f);  // Water fallback
    }
}

float WaterRenderer::getLiquidAlpha(uint16_t liquidType) const {
    uint8_t basicType = (liquidType == 0) ? 0 : ((liquidType - 1) % 4);
    switch (basicType) {
        case 1:  return 0.68f;  // Ocean
        case 2:  return 0.72f;  // Magma
        case 3:  return 0.62f;  // Slime
        default: return 0.38f;  // Water
    }
}

} // namespace rendering
} // namespace wowee
