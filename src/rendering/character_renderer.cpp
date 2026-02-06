/**
 * CharacterRenderer — GPU rendering of M2 character models with skeletal animation
 *
 * Handles:
 *  - Uploading M2 vertex/index data to OpenGL VAO/VBO/EBO
 *  - Per-frame bone matrix computation (hierarchical, with keyframe interpolation)
 *  - GPU vertex skinning via a bone-matrix uniform array in the vertex shader
 *  - Per-batch texture binding through the M2 texture-lookup indirection
 *  - Geoset filtering (activeGeosets) to show/hide body part groups
 *  - CPU texture compositing for character skins (base skin + underwear overlays)
 *
 * The character texture compositing uses the WoW CharComponentTextureSections
 * layout, placing region overlays (pelvis, torso, etc.) at their correct pixel
 * positions on the 512×512 body skin atlas. Region coordinates sourced from
 * the original WoW Model Viewer (charcontrol.h, REGION_FAC=2).
 */
#include "rendering/character_renderer.hpp"
#include "rendering/shader.hpp"
#include "rendering/texture.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace wowee {
namespace rendering {

CharacterRenderer::CharacterRenderer() {
}

CharacterRenderer::~CharacterRenderer() {
    shutdown();
}

bool CharacterRenderer::initialize() {
    core::Logger::getInstance().info("Initializing character renderer...");

    // Create character shader with skeletal animation
    const char* vertexSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec4 aBoneWeights;
        layout (location = 2) in ivec4 aBoneIndices;
        layout (location = 3) in vec3 aNormal;
        layout (location = 4) in vec2 aTexCoord;

        uniform mat4 uModel;
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform mat4 uBones[200];

        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;

        void main() {
            // Skinning: blend bone transformations
            mat4 boneTransform = mat4(0.0);
            boneTransform += uBones[aBoneIndices.x] * aBoneWeights.x;
            boneTransform += uBones[aBoneIndices.y] * aBoneWeights.y;
            boneTransform += uBones[aBoneIndices.z] * aBoneWeights.z;
            boneTransform += uBones[aBoneIndices.w] * aBoneWeights.w;

            // Transform position and normal
            vec4 skinnedPos = boneTransform * vec4(aPos, 1.0);
            vec4 worldPos = uModel * skinnedPos;

            FragPos = worldPos.xyz;
            // Use mat3 directly - avoid expensive inverse() in shader
            // Works correctly for uniform scaling; normalize in fragment shader handles the rest
            Normal = mat3(uModel) * mat3(boneTransform) * aNormal;
            TexCoord = aTexCoord;

            gl_Position = uProjection * uView * worldPos;
        }
    )";

    const char* fragmentSrc = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;

        uniform sampler2D uTexture0;
        uniform vec3 uLightDir;
        uniform vec3 uLightColor;
        uniform float uSpecularIntensity;
        uniform vec3 uViewPos;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        uniform sampler2DShadow uShadowMap;
        uniform mat4 uLightSpaceMatrix;
        uniform int uShadowEnabled;
        uniform float uShadowStrength;

        out vec4 FragColor;

        void main() {
            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Diffuse lighting
            float diff = max(dot(normal, lightDir), 0.0);

            // Blinn-Phong specular
            vec3 viewDir = normalize(uViewPos - FragPos);
            vec3 halfDir = normalize(lightDir + viewDir);
            float spec = pow(max(dot(normal, halfDir), 0.0), 32.0);
            vec3 specular = spec * uLightColor * uSpecularIntensity;

            // Shadow mapping
            float shadow = 1.0;
            if (uShadowEnabled != 0) {
                vec4 lsPos = uLightSpaceMatrix * vec4(FragPos, 1.0);
                vec3 proj = lsPos.xyz / lsPos.w * 0.5 + 0.5;
                if (proj.z <= 1.0 && proj.x >= 0.0 && proj.x <= 1.0 && proj.y >= 0.0 && proj.y <= 1.0) {
                    float edgeDist = max(abs(proj.x - 0.5), abs(proj.y - 0.5));
                    float coverageFade = 1.0 - smoothstep(0.40, 0.49, edgeDist);
                    float bias = max(0.005 * (1.0 - abs(dot(normal, lightDir))), 0.001);
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

            // Ambient
            vec3 ambient = vec3(0.3);

            // Sample texture
            vec4 texColor = texture(uTexture0, TexCoord);

            // Combine
            vec3 result = (ambient + (diff * vec3(1.0) + specular) * shadow) * texColor.rgb;

            // Fog
            float fogDist = length(uViewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            result = mix(uFogColor, result, fogFactor);

            // Force alpha=1 for opaque character rendering (baked NPC textures may have alpha=0)
            FragColor = vec4(result, 1.0);
        }
    )";

    // Log GPU uniform limit
    GLint maxComponents = 0;
    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &maxComponents);
    core::Logger::getInstance().info("GPU max vertex uniform components: ", maxComponents,
                                     " (supports ~", maxComponents / 16, " mat4)");

    characterShader = std::make_unique<Shader>();
    if (!characterShader->loadFromSource(vertexSrc, fragmentSrc)) {
        core::Logger::getInstance().error("Failed to create character shader");
        return false;
    }

    const char* shadowVertSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec4 aBoneWeights;
        layout (location = 2) in ivec4 aBoneIndices;
        layout (location = 4) in vec2 aTexCoord;

        uniform mat4 uLightSpaceMatrix;
        uniform mat4 uModel;
        uniform mat4 uBones[200];

        out vec2 vTexCoord;

        void main() {
            mat4 boneTransform = mat4(0.0);
            boneTransform += uBones[aBoneIndices.x] * aBoneWeights.x;
            boneTransform += uBones[aBoneIndices.y] * aBoneWeights.y;
            boneTransform += uBones[aBoneIndices.z] * aBoneWeights.z;
            boneTransform += uBones[aBoneIndices.w] * aBoneWeights.w;
            vec4 skinnedPos = boneTransform * vec4(aPos, 1.0);
            vTexCoord = aTexCoord;
            gl_Position = uLightSpaceMatrix * uModel * skinnedPos;
        }
    )";

    const char* shadowFragSrc = R"(
        #version 330 core
        in vec2 vTexCoord;
        uniform sampler2D uTexture;
        uniform bool uAlphaTest;
        void main() {
            if (uAlphaTest && texture(uTexture, vTexCoord).a < 0.5) discard;
        }
    )";

    auto compileStage = [](GLenum type, const char* src) -> GLuint {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            LOG_ERROR("Character shadow shader compile error: ", log);
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    };

    GLuint shVs = compileStage(GL_VERTEX_SHADER, shadowVertSrc);
    GLuint shFs = compileStage(GL_FRAGMENT_SHADER, shadowFragSrc);
    if (!shVs || !shFs) {
        if (shVs) glDeleteShader(shVs);
        if (shFs) glDeleteShader(shFs);
        return false;
    }

    shadowCasterProgram = glCreateProgram();
    glAttachShader(shadowCasterProgram, shVs);
    glAttachShader(shadowCasterProgram, shFs);
    glLinkProgram(shadowCasterProgram);
    GLint linked = 0;
    glGetProgramiv(shadowCasterProgram, GL_LINK_STATUS, &linked);
    glDeleteShader(shVs);
    glDeleteShader(shFs);
    if (!linked) {
        char log[512];
        glGetProgramInfoLog(shadowCasterProgram, sizeof(log), nullptr, log);
        LOG_ERROR("Character shadow shader link error: ", log);
        glDeleteProgram(shadowCasterProgram);
        shadowCasterProgram = 0;
        return false;
    }

    // Create 1x1 white fallback texture
    uint8_t white[] = { 255, 255, 255, 255 };
    glGenTextures(1, &whiteTexture);
    glBindTexture(GL_TEXTURE_2D, whiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    core::Logger::getInstance().info("Character renderer initialized");
    return true;
}

void CharacterRenderer::shutdown() {
    // Clean up GPU resources
    for (auto& pair : models) {
        auto& gpuModel = pair.second;
        if (gpuModel.vao) {
            glDeleteVertexArrays(1, &gpuModel.vao);
            glDeleteBuffers(1, &gpuModel.vbo);
            glDeleteBuffers(1, &gpuModel.ebo);
        }
        for (GLuint texId : gpuModel.textureIds) {
            if (texId && texId != whiteTexture) {
                glDeleteTextures(1, &texId);
            }
        }
    }

    // Clean up texture cache
    for (auto& pair : textureCache) {
        if (pair.second && pair.second != whiteTexture) {
            glDeleteTextures(1, &pair.second);
        }
    }
    textureCache.clear();

    if (whiteTexture) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }

    models.clear();
    instances.clear();
    characterShader.reset();
    if (shadowCasterProgram) {
        glDeleteProgram(shadowCasterProgram);
        shadowCasterProgram = 0;
    }
}

GLuint CharacterRenderer::loadTexture(const std::string& path) {
    // Skip empty or whitespace-only paths (type-0 textures have no filename)
    if (path.empty()) return whiteTexture;
    bool allWhitespace = true;
    for (char c : path) {
        if (c != ' ' && c != '\t' && c != '\0' && c != '\n') { allWhitespace = false; break; }
    }
    if (allWhitespace) return whiteTexture;

    // Check cache
    auto it = textureCache.find(path);
    if (it != textureCache.end()) return it->second;

    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture;
    }

    auto blpImage = assetManager->loadTexture(path);
    if (!blpImage.isValid()) {
        core::Logger::getInstance().warning("Failed to load texture: ", path);
        textureCache[path] = whiteTexture;
        return whiteTexture;
    }

    GLuint texId;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, blpImage.width, blpImage.height,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, blpImage.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);
    applyAnisotropicFiltering();
    glBindTexture(GL_TEXTURE_2D, 0);

    textureCache[path] = texId;
    core::Logger::getInstance().info("Loaded character texture: ", path, " (", blpImage.width, "x", blpImage.height, ")");
    return texId;
}

// Alpha-blend overlay onto composite at (dstX, dstY)
static void blitOverlay(std::vector<uint8_t>& composite, int compW, int compH,
                         const pipeline::BLPImage& overlay, int dstX, int dstY) {
    for (int sy = 0; sy < overlay.height; sy++) {
        int dy = dstY + sy;
        if (dy < 0 || dy >= compH) continue;
        for (int sx = 0; sx < overlay.width; sx++) {
            int dx = dstX + sx;
            if (dx < 0 || dx >= compW) continue;

            size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            size_t dstIdx = (static_cast<size_t>(dy) * compW + dx) * 4;

            uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            if (srcA == 255) {
                composite[dstIdx + 0] = overlay.data[srcIdx + 0];
                composite[dstIdx + 1] = overlay.data[srcIdx + 1];
                composite[dstIdx + 2] = overlay.data[srcIdx + 2];
                composite[dstIdx + 3] = 255;
            } else {
                float alpha = srcA / 255.0f;
                float invAlpha = 1.0f - alpha;
                composite[dstIdx + 0] = static_cast<uint8_t>(overlay.data[srcIdx + 0] * alpha + composite[dstIdx + 0] * invAlpha);
                composite[dstIdx + 1] = static_cast<uint8_t>(overlay.data[srcIdx + 1] * alpha + composite[dstIdx + 1] * invAlpha);
                composite[dstIdx + 2] = static_cast<uint8_t>(overlay.data[srcIdx + 2] * alpha + composite[dstIdx + 2] * invAlpha);
                composite[dstIdx + 3] = std::max(composite[dstIdx + 3], srcA);
            }
        }
    }
}

// Nearest-neighbor 2x scale blit of overlay onto composite at (dstX, dstY)
static void blitOverlayScaled2x(std::vector<uint8_t>& composite, int compW, int compH,
                                 const pipeline::BLPImage& overlay, int dstX, int dstY) {
    for (int sy = 0; sy < overlay.height; sy++) {
        for (int sx = 0; sx < overlay.width; sx++) {
            size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            // Write to 2x2 block of destination pixels
            for (int dy2 = 0; dy2 < 2; dy2++) {
                int dy = dstY + sy * 2 + dy2;
                if (dy < 0 || dy >= compH) continue;
                for (int dx2 = 0; dx2 < 2; dx2++) {
                    int dx = dstX + sx * 2 + dx2;
                    if (dx < 0 || dx >= compW) continue;

                    size_t dstIdx = (static_cast<size_t>(dy) * compW + dx) * 4;
                    if (srcA == 255) {
                        composite[dstIdx + 0] = overlay.data[srcIdx + 0];
                        composite[dstIdx + 1] = overlay.data[srcIdx + 1];
                        composite[dstIdx + 2] = overlay.data[srcIdx + 2];
                        composite[dstIdx + 3] = 255;
                    } else {
                        float alpha = srcA / 255.0f;
                        float invAlpha = 1.0f - alpha;
                        composite[dstIdx + 0] = static_cast<uint8_t>(overlay.data[srcIdx + 0] * alpha + composite[dstIdx + 0] * invAlpha);
                        composite[dstIdx + 1] = static_cast<uint8_t>(overlay.data[srcIdx + 1] * alpha + composite[dstIdx + 1] * invAlpha);
                        composite[dstIdx + 2] = static_cast<uint8_t>(overlay.data[srcIdx + 2] * alpha + composite[dstIdx + 2] * invAlpha);
                        composite[dstIdx + 3] = std::max(composite[dstIdx + 3], srcA);
                    }
                }
            }
        }
    }
}

GLuint CharacterRenderer::compositeTextures(const std::vector<std::string>& layerPaths) {
    if (layerPaths.empty() || !assetManager || !assetManager->isInitialized()) {
        return whiteTexture;
    }

    // Load base layer
    auto base = assetManager->loadTexture(layerPaths[0]);
    if (!base.isValid()) {
        core::Logger::getInstance().warning("Composite: failed to load base layer: ", layerPaths[0]);
        return whiteTexture;
    }

    // Copy base pixel data as our working buffer
    std::vector<uint8_t> composite = base.data;
    int width = base.width;
    int height = base.height;

    core::Logger::getInstance().info("Composite: base layer ", width, "x", height, " from ", layerPaths[0]);

    // Alpha-blend each overlay onto the composite
    for (size_t layer = 1; layer < layerPaths.size(); layer++) {
        if (layerPaths[layer].empty()) continue;

        auto overlay = assetManager->loadTexture(layerPaths[layer]);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("Composite: failed to load overlay: ", layerPaths[layer]);
            continue;
        }

        core::Logger::getInstance().info("Composite: overlay ", layerPaths[layer],
            " (", overlay.width, "x", overlay.height, ")");

        // Debug: save overlay to disk
        {
            std::string fname = (std::filesystem::temp_directory_path() / ("overlay_debug_" + std::to_string(layer) + ".rgba")).string();
            FILE* f = fopen(fname.c_str(), "wb");
            if (f) {
                fwrite(&overlay.width, 4, 1, f);
                fwrite(&overlay.height, 4, 1, f);
                fwrite(overlay.data.data(), 1, overlay.data.size(), f);
                fclose(f);
            }
            // Check alpha values
            int opaquePixels = 0, transPixels = 0, semiPixels = 0;
            size_t pxCount = static_cast<size_t>(overlay.width) * overlay.height;
            for (size_t p = 0; p < pxCount; p++) {
                uint8_t a = overlay.data[p * 4 + 3];
                if (a == 255) opaquePixels++;
                else if (a == 0) transPixels++;
                else semiPixels++;
            }
            core::Logger::getInstance().info("  Overlay alpha stats: opaque=", opaquePixels,
                " transparent=", transPixels, " semi=", semiPixels);
        }

        if (overlay.width == width && overlay.height == height) {
            // Same size: full alpha-blend
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // WoW character texture layout (512x512, from CharComponentTextureSections):
            // Region               X    Y    W    H
            // 0  Base              0    0    512  512
            // 1  Arm Upper         0    0    256  128
            // 2  Arm Lower         0    128  256  128
            // 3  Hand              0    256  256  64
            // 4  Face Upper        0    320  256  64
            // 5  Face Lower        0    384  256  128
            // 6  Torso Upper       256  0    256  128
            // 7  Torso Lower       256  128  256  64
            // 8  Pelvis Upper      256  192  256  128
            // 9  Pelvis Lower      256  320  256  128
            // 10 Foot              256  448  256  64
            //
            // Determine region by filename keywords
            int dstX = 0, dstY = 0;
            std::string pathLower = layerPaths[layer];
            for (auto& c : pathLower) c = std::tolower(c);

            if (pathLower.find("pelvis") != std::string::npos) {
                // Pelvis Upper: (256, 192) 256x128
                dstX = 256;
                dstY = 192;
                core::Logger::getInstance().info("Composite: placing pelvis region at (", dstX, ",", dstY, ")");
            } else if (pathLower.find("torso") != std::string::npos) {
                // Torso Upper: (256, 0) 256x128
                dstX = 256;
                dstY = 0;
                core::Logger::getInstance().info("Composite: placing torso region at (", dstX, ",", dstY, ")");
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 128;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 256;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 256; dstY = 448;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 256; dstY = 320;
            } else {
                // Unknown — center placement as fallback
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
                core::Logger::getInstance().info("Composite: unknown region '", layerPaths[layer], "', placing at (", dstX, ",", dstY, ")");
            }

            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }
    }

    // Debug: save composite as raw RGBA file
    {
        std::string dbgPath = (std::filesystem::temp_directory_path() / "composite_debug.rgba").string();
        FILE* f = fopen(dbgPath.c_str(), "wb");
        if (f) {
            // Write width, height as 4 bytes each, then pixel data
            fwrite(&width, 4, 1, f);
            fwrite(&height, 4, 1, f);
            fwrite(composite.data(), 1, composite.size(), f);
            fclose(f);
            core::Logger::getInstance().info("DEBUG: saved composite to ", dbgPath);
        }
    }

    // Upload composite to GPU
    GLuint texId;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, composite.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);
    applyAnisotropicFiltering();
    glBindTexture(GL_TEXTURE_2D, 0);

    core::Logger::getInstance().info("Composite texture created: ", width, "x", height, " from ", layerPaths.size(), " layers");
    return texId;
}

GLuint CharacterRenderer::compositeWithRegions(const std::string& basePath,
                                                const std::vector<std::string>& baseLayers,
                                                const std::vector<std::pair<int, std::string>>& regionLayers) {
    // Region index → pixel coordinates on the 512x512 atlas
    static const int regionCoords[][2] = {
        {   0,   0 },  // 0 = ArmUpper
        {   0, 128 },  // 1 = ArmLower
        {   0, 256 },  // 2 = Hand
        { 256,   0 },  // 3 = TorsoUpper
        { 256, 128 },  // 4 = TorsoLower
        { 256, 192 },  // 5 = LegUpper
        { 256, 320 },  // 6 = LegLower
        { 256, 448 },  // 7 = Foot
    };

    // First, build base skin + underwear using existing compositeTextures
    std::vector<std::string> layers;
    layers.push_back(basePath);
    for (const auto& ul : baseLayers) {
        layers.push_back(ul);
    }
    // Load base composite into CPU buffer
    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture;
    }

    auto base = assetManager->loadTexture(basePath);
    if (!base.isValid()) {
        return whiteTexture;
    }

    std::vector<uint8_t> composite;
    int width = base.width;
    int height = base.height;

    // If base texture is 256x256 (e.g., baked NPC texture), upscale to 512x512
    // so equipment regions can be composited at correct coordinates
    if (width == 256 && height == 256 && !regionLayers.empty()) {
        width = 512;
        height = 512;
        composite.resize(width * height * 4);
        // Simple 2x nearest-neighbor upscale
        for (int y = 0; y < 512; y++) {
            for (int x = 0; x < 512; x++) {
                int srcX = x / 2;
                int srcY = y / 2;
                int srcIdx = (srcY * 256 + srcX) * 4;
                int dstIdx = (y * 512 + x) * 4;
                composite[dstIdx + 0] = base.data[srcIdx + 0];
                composite[dstIdx + 1] = base.data[srcIdx + 1];
                composite[dstIdx + 2] = base.data[srcIdx + 2];
                composite[dstIdx + 3] = base.data[srcIdx + 3];
            }
        }
        core::Logger::getInstance().info("compositeWithRegions: upscaled 256x256 to 512x512");
    } else {
        composite = base.data;
    }

    // Blend underwear overlays (same logic as compositeTextures)
    for (const auto& ul : baseLayers) {
        if (ul.empty()) continue;
        auto overlay = assetManager->loadTexture(ul);
        if (!overlay.isValid()) continue;

        if (overlay.width == width && overlay.height == height) {
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            int dstX = 0, dstY = 0;
            std::string pathLower = ul;
            for (auto& c : pathLower) c = std::tolower(c);

            if (pathLower.find("pelvis") != std::string::npos) {
                dstX = 256; dstY = 192;
            } else if (pathLower.find("torso") != std::string::npos) {
                dstX = 256; dstY = 0;
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 128;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 256;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 256; dstY = 448;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 256; dstY = 320;
            } else {
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
            }
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }
    }

    // Expected region sizes on the 512x512 atlas
    static const int regionSizes[][2] = {
        { 256, 128 },  // 0 = ArmUpper
        { 256, 128 },  // 1 = ArmLower
        { 256,  64 },  // 2 = Hand
        { 256, 128 },  // 3 = TorsoUpper
        { 256,  64 },  // 4 = TorsoLower
        { 256, 128 },  // 5 = LegUpper
        { 256, 128 },  // 6 = LegLower
        { 256,  64 },  // 7 = Foot
    };

    // Now blit equipment region textures at explicit coordinates
    for (const auto& rl : regionLayers) {
        int regionIdx = rl.first;
        if (regionIdx < 0 || regionIdx >= 8) continue;

        auto overlay = assetManager->loadTexture(rl.second);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("compositeWithRegions: failed to load ", rl.second);
            continue;
        }

        int dstX = regionCoords[regionIdx][0];
        int dstY = regionCoords[regionIdx][1];

        // Component textures are stored at half resolution — scale 2x if needed
        int expectedW = regionSizes[regionIdx][0];
        int expectedH = regionSizes[regionIdx][1];
        if (overlay.width * 2 == expectedW && overlay.height * 2 == expectedH) {
            blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
        } else {
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }

        core::Logger::getInstance().info("compositeWithRegions: region ", regionIdx,
            " at (", dstX, ",", dstY, ") ", overlay.width, "x", overlay.height, " from ", rl.second);
    }

    // Upload to GPU
    GLuint texId;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, composite.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);
    applyAnisotropicFiltering();
    glBindTexture(GL_TEXTURE_2D, 0);

    core::Logger::getInstance().info("compositeWithRegions: created ", width, "x", height,
        " texture with ", regionLayers.size(), " equipment regions");
    return texId;
}

void CharacterRenderer::setModelTexture(uint32_t modelId, uint32_t textureSlot, GLuint textureId) {
    auto it = models.find(modelId);
    if (it == models.end()) {
        core::Logger::getInstance().warning("setModelTexture: model ", modelId, " not found");
        return;
    }

    auto& gpuModel = it->second;
    if (textureSlot >= gpuModel.textureIds.size()) {
        core::Logger::getInstance().warning("setModelTexture: slot ", textureSlot, " out of range (", gpuModel.textureIds.size(), " textures)");
        return;
    }

    // Delete old texture if it's not shared and not in the texture cache
    GLuint oldTex = gpuModel.textureIds[textureSlot];
    if (oldTex && oldTex != whiteTexture) {
        bool cached = false;
        for (const auto& [k, v] : textureCache) {
            if (v == oldTex) { cached = true; break; }
        }
        if (!cached) {
            glDeleteTextures(1, &oldTex);
        }
    }

    gpuModel.textureIds[textureSlot] = textureId;
    core::Logger::getInstance().info("Replaced model ", modelId, " texture slot ", textureSlot, " with composited texture");
}

void CharacterRenderer::resetModelTexture(uint32_t modelId, uint32_t textureSlot) {
    setModelTexture(modelId, textureSlot, whiteTexture);
}

bool CharacterRenderer::loadModel(const pipeline::M2Model& model, uint32_t id) {
    if (!model.isValid()) {
        core::Logger::getInstance().error("Cannot load invalid M2 model");
        return false;
    }

    if (models.find(id) != models.end()) {
        core::Logger::getInstance().warning("Model ID ", id, " already loaded, replacing");
        auto& old = models[id];
        if (old.vao) {
            glDeleteVertexArrays(1, &old.vao);
            glDeleteBuffers(1, &old.vbo);
            glDeleteBuffers(1, &old.ebo);
        }
    }

    M2ModelGPU gpuModel;
    gpuModel.data = model;

    // Setup GPU buffers
    setupModelBuffers(gpuModel);

    // Calculate bind pose
    calculateBindPose(gpuModel);

    // Load textures from model
    for (const auto& tex : model.textures) {
        GLuint texId = loadTexture(tex.filename);
        gpuModel.textureIds.push_back(texId);
    }

    models[id] = std::move(gpuModel);

    core::Logger::getInstance().info("Loaded M2 model ", id, " (", model.vertices.size(),
                      " verts, ", model.bones.size(), " bones, ", model.sequences.size(),
                      " anims, ", model.textures.size(), " textures)");

    return true;
}

void CharacterRenderer::setupModelBuffers(M2ModelGPU& gpuModel) {
    auto& model = gpuModel.data;

    glGenVertexArrays(1, &gpuModel.vao);
    glGenBuffers(1, &gpuModel.vbo);
    glGenBuffers(1, &gpuModel.ebo);

    glBindVertexArray(gpuModel.vao);

    // Interleaved vertex data
    glBindBuffer(GL_ARRAY_BUFFER, gpuModel.vbo);
    glBufferData(GL_ARRAY_BUFFER, model.vertices.size() * sizeof(pipeline::M2Vertex),
                model.vertices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pipeline::M2Vertex),
                         (void*)offsetof(pipeline::M2Vertex, position));

    // Bone weights (normalize uint8 to float)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(pipeline::M2Vertex),
                         (void*)offsetof(pipeline::M2Vertex, boneWeights));

    // Bone indices
    glEnableVertexAttribArray(2);
    glVertexAttribIPointer(2, 4, GL_UNSIGNED_BYTE, sizeof(pipeline::M2Vertex),
                          (void*)offsetof(pipeline::M2Vertex, boneIndices));

    // Normal
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(pipeline::M2Vertex),
                         (void*)offsetof(pipeline::M2Vertex, normal));

    // TexCoord (first UV set)
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(pipeline::M2Vertex),
                         (void*)offsetof(pipeline::M2Vertex, texCoords));

    // Index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuModel.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.indices.size() * sizeof(uint16_t),
                model.indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
}

void CharacterRenderer::calculateBindPose(M2ModelGPU& gpuModel) {
    auto& bones = gpuModel.data.bones;
    size_t numBones = bones.size();
    gpuModel.bindPose.resize(numBones);

    // Compute full hierarchical rest pose, then invert.
    // Each bone's rest position is T(pivot), composed with its parent chain.
    std::vector<glm::mat4> restPose(numBones);
    for (size_t i = 0; i < numBones; i++) {
        glm::mat4 local = glm::translate(glm::mat4(1.0f), bones[i].pivot);
        if (bones[i].parentBone >= 0 && static_cast<size_t>(bones[i].parentBone) < numBones) {
            restPose[i] = restPose[bones[i].parentBone] * local;
        } else {
            restPose[i] = local;
        }
        gpuModel.bindPose[i] = glm::inverse(restPose[i]);
    }
}

uint32_t CharacterRenderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                           const glm::vec3& rotation, float scale) {
    if (models.find(modelId) == models.end()) {
        core::Logger::getInstance().error("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    CharacterInstance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;

    // Initialize bone matrices to identity
    auto& model = models[modelId].data;
    instance.boneMatrices.resize(std::max(static_cast<size_t>(1), model.bones.size()), glm::mat4(1.0f));

    instances[instance.id] = instance;

    core::Logger::getInstance().info("Created character instance ", instance.id, " from model ", modelId);

    return instance.id;
}

void CharacterRenderer::playAnimation(uint32_t instanceId, uint32_t animationId, bool loop) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        core::Logger::getInstance().warning("Cannot play animation: instance ", instanceId, " not found");
        return;
    }

    auto& instance = it->second;
    auto& model = models[instance.modelId].data;

    // Find animation sequence index by ID
    instance.currentAnimationId = animationId;
    instance.currentSequenceIndex = -1;
    instance.animationTime = 0.0f;
    instance.animationLoop = loop;

    for (size_t i = 0; i < model.sequences.size(); i++) {
        if (model.sequences[i].id == animationId) {
            instance.currentSequenceIndex = static_cast<int>(i);
            break;
        }
    }

    if (instance.currentSequenceIndex < 0) {
        // Fall back to first sequence
        if (!model.sequences.empty()) {
            instance.currentSequenceIndex = 0;
            instance.currentAnimationId = model.sequences[0].id;
        }
        core::Logger::getInstance().warning("Animation ", animationId, " not found, using default");
        // Dump available animation IDs for debugging
        std::string ids;
        for (size_t i = 0; i < model.sequences.size(); i++) {
            if (!ids.empty()) ids += ", ";
            ids += std::to_string(model.sequences[i].id);
        }
        core::Logger::getInstance().info("Available animation IDs (", model.sequences.size(), "): ", ids);
    }
}

void CharacterRenderer::update(float deltaTime) {
    for (auto& pair : instances) {
        updateAnimation(pair.second, deltaTime);
    }

    // Update weapon attachment transforms (after all bone matrices are computed)
    for (auto& pair : instances) {
        auto& instance = pair.second;
        if (instance.weaponAttachments.empty()) continue;

        glm::mat4 charModelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);

        for (const auto& wa : instance.weaponAttachments) {
            auto weapIt = instances.find(wa.weaponInstanceId);
            if (weapIt == instances.end()) continue;

            // Get the bone matrix for the attachment bone
            glm::mat4 boneMat(1.0f);
            if (wa.boneIndex < instance.boneMatrices.size()) {
                boneMat = instance.boneMatrices[wa.boneIndex];
            }

            // Weapon model matrix = character model * bone transform * offset translation
            weapIt->second.overrideModelMatrix =
                charModelMat * boneMat * glm::translate(glm::mat4(1.0f), wa.offset);
            weapIt->second.hasOverrideModelMatrix = true;
        }
    }
}

void CharacterRenderer::updateAnimation(CharacterInstance& instance, float deltaTime) {
    auto& model = models[instance.modelId].data;

    if (model.sequences.empty()) {
        return;
    }

    // Resolve sequence index if not set
    if (instance.currentSequenceIndex < 0) {
        instance.currentSequenceIndex = 0;
        instance.currentAnimationId = model.sequences[0].id;
    }

    const auto& sequence = model.sequences[instance.currentSequenceIndex];

    // Update animation time (convert to milliseconds)
    instance.animationTime += deltaTime * 1000.0f;

    if (sequence.duration > 0 && instance.animationTime >= static_cast<float>(sequence.duration)) {
        if (instance.animationLoop) {
            instance.animationTime = std::fmod(instance.animationTime, static_cast<float>(sequence.duration));
        } else {
            instance.animationTime = static_cast<float>(sequence.duration);
        }
    }

    // Update bone matrices
    calculateBoneMatrices(instance);
}

// --- Keyframe interpolation helpers ---

int CharacterRenderer::findKeyframeIndex(const std::vector<uint32_t>& timestamps, float time) {
    if (timestamps.empty()) return -1;
    if (timestamps.size() == 1) return 0;

    // Binary search for the keyframe bracket
    for (size_t i = 0; i < timestamps.size() - 1; i++) {
        if (time < static_cast<float>(timestamps[i + 1])) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(timestamps.size() - 2);
}

glm::vec3 CharacterRenderer::interpolateVec3(const pipeline::M2AnimationTrack& track,
                                              int seqIdx, float time, const glm::vec3& defaultVal) {
    if (!track.hasData()) return defaultVal;
    if (seqIdx < 0 || seqIdx >= static_cast<int>(track.sequences.size())) return defaultVal;

    const auto& keys = track.sequences[seqIdx];
    if (keys.timestamps.empty() || keys.vec3Values.empty()) return defaultVal;

    auto safeVec3 = [&](const glm::vec3& v) -> glm::vec3 {
        if (std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z)) return defaultVal;
        return v;
    };

    if (keys.vec3Values.size() == 1) return safeVec3(keys.vec3Values[0]);

    int idx = findKeyframeIndex(keys.timestamps, time);
    if (idx < 0) return defaultVal;

    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.vec3Values.size() - 1);

    if (i0 == i1) return safeVec3(keys.vec3Values[i0]);

    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float duration = t1 - t0;
    float t = (duration > 0.0f) ? glm::clamp((time - t0) / duration, 0.0f, 1.0f) : 0.0f;

    return safeVec3(glm::mix(keys.vec3Values[i0], keys.vec3Values[i1], t));
}

glm::quat CharacterRenderer::interpolateQuat(const pipeline::M2AnimationTrack& track,
                                              int seqIdx, float time) {
    glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    if (!track.hasData()) return identity;
    if (seqIdx < 0 || seqIdx >= static_cast<int>(track.sequences.size())) return identity;

    const auto& keys = track.sequences[seqIdx];
    if (keys.timestamps.empty() || keys.quatValues.empty()) return identity;

    auto safeQuat = [&](const glm::quat& q) -> glm::quat {
        float len = glm::length(q);
        if (len < 0.001f || std::isnan(len)) return identity;
        return q;
    };

    if (keys.quatValues.size() == 1) return safeQuat(keys.quatValues[0]);

    int idx = findKeyframeIndex(keys.timestamps, time);
    if (idx < 0) return identity;

    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.quatValues.size() - 1);

    if (i0 == i1) return safeQuat(keys.quatValues[i0]);

    glm::quat q0 = safeQuat(keys.quatValues[i0]);
    glm::quat q1 = safeQuat(keys.quatValues[i1]);

    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float duration = t1 - t0;
    float t = (duration > 0.0f) ? glm::clamp((time - t0) / duration, 0.0f, 1.0f) : 0.0f;

    return glm::slerp(q0, q1, t);
}

// --- Bone transform calculation ---

void CharacterRenderer::calculateBoneMatrices(CharacterInstance& instance) {
    auto& model = models[instance.modelId].data;

    if (model.bones.empty()) {
        return;
    }

    size_t numBones = model.bones.size();
    instance.boneMatrices.resize(numBones);

    static bool dumpedOnce = false;

    for (size_t i = 0; i < numBones; i++) {
        const auto& bone = model.bones[i];

        // Local transform includes pivot bracket: T(pivot)*T*R*S*T(-pivot)
        // At rest this is identity, so no separate bind pose is needed
        glm::mat4 localTransform = getBoneTransform(bone, instance.animationTime, instance.currentSequenceIndex);

        // Debug: dump first frame bone data
        if (!dumpedOnce && i < 5) {
            glm::vec3 t = interpolateVec3(bone.translation, instance.currentSequenceIndex, instance.animationTime, glm::vec3(0.0f));
            glm::quat r = interpolateQuat(bone.rotation, instance.currentSequenceIndex, instance.animationTime);
            glm::vec3 s = interpolateVec3(bone.scale, instance.currentSequenceIndex, instance.animationTime, glm::vec3(1.0f));
            core::Logger::getInstance().info("Bone ", i, " parent=", bone.parentBone,
                " pivot=(", bone.pivot.x, ",", bone.pivot.y, ",", bone.pivot.z, ")",
                " t=(", t.x, ",", t.y, ",", t.z, ")",
                " r=(", r.w, ",", r.x, ",", r.y, ",", r.z, ")",
                " s=(", s.x, ",", s.y, ",", s.z, ")",
                " seqIdx=", instance.currentSequenceIndex);
        }

        // Compose with parent
        if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) < numBones) {
            instance.boneMatrices[i] = instance.boneMatrices[bone.parentBone] * localTransform;
        } else {
            instance.boneMatrices[i] = localTransform;
        }
    }
    if (!dumpedOnce) {
        dumpedOnce = true;
        // Dump final matrix for bone 0
        auto& m = instance.boneMatrices[0];
        core::Logger::getInstance().info("Bone 0 final matrix row0=(", m[0][0], ",", m[1][0], ",", m[2][0], ",", m[3][0], ")");
    }
}

glm::mat4 CharacterRenderer::getBoneTransform(const pipeline::M2Bone& bone, float time, int sequenceIndex) {
    glm::vec3 translation = interpolateVec3(bone.translation, sequenceIndex, time, glm::vec3(0.0f));
    glm::quat rotation = interpolateQuat(bone.rotation, sequenceIndex, time);
    glm::vec3 scale = interpolateVec3(bone.scale, sequenceIndex, time, glm::vec3(1.0f));

    // M2 bone transform: T(pivot) * T(trans) * R(rot) * S(scale) * T(-pivot)
    // At rest (no animation): T(pivot) * I * I * I * T(-pivot) = identity
    glm::mat4 transform = glm::translate(glm::mat4(1.0f), bone.pivot);
    transform = glm::translate(transform, translation);
    transform *= glm::toMat4(rotation);
    transform = glm::scale(transform, scale);
    transform = glm::translate(transform, -bone.pivot);

    return transform;
}

// --- Rendering ---

void CharacterRenderer::render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection) {
    if (instances.empty()) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);  // M2 models have mixed winding; render both sides

    characterShader->use();
    characterShader->setUniform("uView", view);
    characterShader->setUniform("uProjection", projection);
    characterShader->setUniform("uLightDir", glm::vec3(0.0f, -1.0f, 0.3f));
    characterShader->setUniform("uLightColor", glm::vec3(1.5f, 1.4f, 1.3f));
    characterShader->setUniform("uSpecularIntensity", 0.5f);
    characterShader->setUniform("uViewPos", camera.getPosition());

    // Fog
    characterShader->setUniform("uFogColor", fogColor);
    characterShader->setUniform("uFogStart", fogStart);
    characterShader->setUniform("uFogEnd", fogEnd);

    // Shadows
    characterShader->setUniform("uShadowEnabled", shadowEnabled ? 1 : 0);
    characterShader->setUniform("uShadowStrength", 0.65f);
    if (shadowEnabled) {
        characterShader->setUniform("uLightSpaceMatrix", lightSpaceMatrix);
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, shadowDepthTex);
        characterShader->setUniform("uShadowMap", 7);
    }

    for (const auto& pair : instances) {
        const auto& instance = pair.second;

        // Skip invisible instances (e.g., player in first-person mode)
        if (!instance.visible) continue;

        const auto& gpuModel = models[instance.modelId];

        // Set model matrix (use override for weapon instances)
        glm::mat4 modelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);
        characterShader->setUniform("uModel", modelMat);

        // Set bone matrices (upload all at once for performance)
        int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), MAX_BONES);
        if (numBones > 0) {
            characterShader->setUniformMatrixArray("uBones[0]", instance.boneMatrices.data(), numBones);
        }

        // Bind VAO and draw
        glBindVertexArray(gpuModel.vao);

        if (!gpuModel.data.batches.empty()) {
            // One-time debug dump of rendered batches
            static bool dumpedBatches = false;
            if (!dumpedBatches) {
                dumpedBatches = true;
                int bIdx = 0;
                int rendered = 0, skipped = 0;
                for (const auto& b : gpuModel.data.batches) {
                    bool filtered = !instance.activeGeosets.empty() &&
                        instance.activeGeosets.find(b.submeshId) == instance.activeGeosets.end();

                    GLuint resolvedTex = whiteTexture;
                    std::string texInfo = "white(fallback)";
                    if (b.textureIndex != 0xFFFF && b.textureIndex < gpuModel.data.textureLookup.size()) {
                        uint16_t lk = gpuModel.data.textureLookup[b.textureIndex];
                        if (lk < gpuModel.textureIds.size()) {
                            resolvedTex = gpuModel.textureIds[lk];
                            texInfo = "lookup[" + std::to_string(b.textureIndex) + "]->tex[" + std::to_string(lk) + "]=GL" + std::to_string(resolvedTex);
                        } else {
                            texInfo = "lookup[" + std::to_string(b.textureIndex) + "]->OOB(" + std::to_string(lk) + ")";
                        }
                    } else if (b.textureIndex == 0xFFFF) {
                        texInfo = "texIdx=FFFF";
                    } else {
                        texInfo = "texIdx=" + std::to_string(b.textureIndex) + " OOB(lookupSz=" + std::to_string(gpuModel.data.textureLookup.size()) + ")";
                    }

                    if (filtered) skipped++; else rendered++;
                    LOG_INFO("Batch ", bIdx, ": submesh=", b.submeshId,
                             " level=", b.submeshLevel,
                             " idxStart=", b.indexStart, " idxCount=", b.indexCount,
                             " tex=", texInfo,
                             filtered ? " [SKIP]" : " [RENDER]");
                    bIdx++;
                }
                LOG_INFO("Batch summary: ", rendered, " rendered, ", skipped, " skipped, ",
                         gpuModel.textureIds.size(), " textures loaded, ",
                         gpuModel.data.textureLookup.size(), " in lookup table");
                for (size_t t = 0; t < gpuModel.data.textures.size(); t++) {
                    LOG_INFO("  Texture[", t, "]: type=", gpuModel.data.textures[t].type,
                             " file=", gpuModel.data.textures[t].filename,
                             " glId=", (t < gpuModel.textureIds.size() ? std::to_string(gpuModel.textureIds[t]) : "N/A"));
                }
            }

            // Draw batches (submeshes) with per-batch textures
            for (const auto& batch : gpuModel.data.batches) {
                // Filter by active geosets (if set)
                if (!instance.activeGeosets.empty() &&
                    instance.activeGeosets.find(batch.submeshId) == instance.activeGeosets.end()) {
                    continue;
                }

                // Resolve texture for this batch
                GLuint texId = whiteTexture;
                if (batch.textureIndex < gpuModel.data.textureLookup.size()) {
                    uint16_t lookupIdx = gpuModel.data.textureLookup[batch.textureIndex];
                    if (lookupIdx < gpuModel.textureIds.size()) {
                        texId = gpuModel.textureIds[lookupIdx];
                    }
                }

                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, texId);

                glDrawElements(GL_TRIANGLES,
                               batch.indexCount,
                               GL_UNSIGNED_SHORT,
                               (void*)(batch.indexStart * sizeof(uint16_t)));
            }
        } else {
            // Draw entire model with first texture
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, !gpuModel.textureIds.empty() ? gpuModel.textureIds[0] : whiteTexture);

            glDrawElements(GL_TRIANGLES,
                          static_cast<GLsizei>(gpuModel.data.indices.size()),
                          GL_UNSIGNED_SHORT,
                          0);
        }
    }

    glBindVertexArray(0);
    glEnable(GL_CULL_FACE);  // Restore culling for other renderers
}

void CharacterRenderer::renderShadow(const glm::mat4& lightSpaceMatrix) {
    if (instances.empty() || shadowCasterProgram == 0) {
        return;
    }

    glUseProgram(shadowCasterProgram);

    GLint lightSpaceLoc = glGetUniformLocation(shadowCasterProgram, "uLightSpaceMatrix");
    GLint modelLoc = glGetUniformLocation(shadowCasterProgram, "uModel");
    GLint texLoc = glGetUniformLocation(shadowCasterProgram, "uTexture");
    GLint alphaTestLoc = glGetUniformLocation(shadowCasterProgram, "uAlphaTest");
    GLint bonesLoc = glGetUniformLocation(shadowCasterProgram, "uBones[0]");
    if (lightSpaceLoc < 0 || modelLoc < 0) {
        return;
    }

    glUniformMatrix4fv(lightSpaceLoc, 1, GL_FALSE, &lightSpaceMatrix[0][0]);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    if (texLoc >= 0) glUniform1i(texLoc, 0);
    glActiveTexture(GL_TEXTURE0);

    for (const auto& [_, instance] : instances) {
        auto modelIt = models.find(instance.modelId);
        if (modelIt == models.end()) continue;
        const auto& gpuModel = modelIt->second;

        glm::mat4 modelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &modelMat[0][0]);

        if (!instance.boneMatrices.empty() && bonesLoc >= 0) {
            int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), MAX_BONES);
            glUniformMatrix4fv(bonesLoc, numBones, GL_FALSE, &instance.boneMatrices[0][0][0]);
        }

        glBindVertexArray(gpuModel.vao);

        if (!gpuModel.data.batches.empty()) {
            for (const auto& batch : gpuModel.data.batches) {
                GLuint texId = whiteTexture;
                if (batch.textureIndex < gpuModel.data.textureLookup.size()) {
                    uint16_t lookupIdx = gpuModel.data.textureLookup[batch.textureIndex];
                    if (lookupIdx < gpuModel.textureIds.size()) {
                        texId = gpuModel.textureIds[lookupIdx];
                    }
                }

                bool alphaCutout = (texId != 0 && texId != whiteTexture);
                if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, alphaCutout ? 1 : 0);
                glBindTexture(GL_TEXTURE_2D, texId ? texId : whiteTexture);

                glDrawElements(GL_TRIANGLES,
                               batch.indexCount,
                               GL_UNSIGNED_SHORT,
                               (void*)(batch.indexStart * sizeof(uint16_t)));
            }
        } else {
            if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, 0);
            glBindTexture(GL_TEXTURE_2D, whiteTexture);
            glDrawElements(GL_TRIANGLES,
                           static_cast<GLsizei>(gpuModel.data.indices.size()),
                           GL_UNSIGNED_SHORT,
                           0);
        }
    }

    glBindVertexArray(0);
    glCullFace(GL_BACK);
}

glm::mat4 CharacterRenderer::getModelMatrix(const CharacterInstance& instance) const {
    glm::mat4 model = glm::mat4(1.0f);

    // Apply transformations: T * R * S
    model = glm::translate(model, instance.position);

    // Apply rotation (euler angles)
    model = glm::rotate(model, instance.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));  // Yaw
    model = glm::rotate(model, instance.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));  // Pitch
    model = glm::rotate(model, instance.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));  // Roll

    model = glm::scale(model, glm::vec3(instance.scale));

    return model;
}

void CharacterRenderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.position = position;
    }
}

void CharacterRenderer::setInstanceRotation(uint32_t instanceId, const glm::vec3& rotation) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.rotation = rotation;
    }
}

void CharacterRenderer::setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.activeGeosets = geosets;
    }
}

void CharacterRenderer::setInstanceVisible(uint32_t instanceId, bool visible) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.visible = visible;

        // Also hide/show attached weapons (for first-person mode)
        for (const auto& wa : it->second.weaponAttachments) {
            auto weapIt = instances.find(wa.weaponInstanceId);
            if (weapIt != instances.end()) {
                weapIt->second.visible = visible;
            }
        }
    }
}

void CharacterRenderer::removeInstance(uint32_t instanceId) {
    instances.erase(instanceId);
}

bool CharacterRenderer::getAnimationState(uint32_t instanceId, uint32_t& animationId,
                                          float& animationTimeMs, float& animationDurationMs) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    const CharacterInstance& instance = it->second;
    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    const auto& sequences = modelIt->second.data.sequences;
    if (instance.currentSequenceIndex < 0 || instance.currentSequenceIndex >= static_cast<int>(sequences.size())) {
        return false;
    }

    animationId = instance.currentAnimationId;
    animationTimeMs = instance.animationTime;
    animationDurationMs = static_cast<float>(sequences[instance.currentSequenceIndex].duration);
    return true;
}

bool CharacterRenderer::hasAnimation(uint32_t instanceId, uint32_t animationId) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    const auto& sequences = modelIt->second.data.sequences;
    for (const auto& seq : sequences) {
        if (seq.id == animationId) {
            return true;
        }
    }
    return false;
}

bool CharacterRenderer::getAnimationSequences(uint32_t instanceId, std::vector<pipeline::M2Sequence>& out) const {
    out.clear();
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }

    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }

    out = modelIt->second.data.sequences;
    return !out.empty();
}

bool CharacterRenderer::getInstanceModelName(uint32_t instanceId, std::string& modelName) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) {
        return false;
    }
    auto modelIt = models.find(it->second.modelId);
    if (modelIt == models.end()) {
        return false;
    }
    modelName = modelIt->second.data.name;
    return !modelName.empty();
}

bool CharacterRenderer::attachWeapon(uint32_t charInstanceId, uint32_t attachmentId,
                                      const pipeline::M2Model& weaponModel, uint32_t weaponModelId,
                                      const std::string& texturePath) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) {
        core::Logger::getInstance().warning("attachWeapon: character instance ", charInstanceId, " not found");
        return false;
    }
    auto& charInstance = charIt->second;
    auto charModelIt = models.find(charInstance.modelId);
    if (charModelIt == models.end()) return false;
    const auto& charModel = charModelIt->second.data;

    // Find bone index for this attachment point
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    bool found = false;

    // Try attachment lookup first
    if (attachmentId < charModel.attachmentLookup.size()) {
        uint16_t attIdx = charModel.attachmentLookup[attachmentId];
        if (attIdx < charModel.attachments.size()) {
            boneIndex = charModel.attachments[attIdx].bone;
            offset = charModel.attachments[attIdx].position;
            found = true;
        }
    }
    // Fallback: scan attachments by id
    if (!found) {
        for (const auto& att : charModel.attachments) {
            if (att.id == attachmentId) {
                boneIndex = att.bone;
                offset = att.position;
                found = true;
                break;
            }
        }
    }
    // Fallback: scan bones for keyBoneId 26 (right hand) / 27 (left hand)
    if (!found) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < charModel.bones.size(); i++) {
            if (charModel.bones[i].keyBoneId == targetKeyBone) {
                boneIndex = static_cast<uint16_t>(i);
                found = true;
                break;
            }
        }
    }

    if (!found) {
        core::Logger::getInstance().warning("attachWeapon: no bone found for attachment ", attachmentId);
        return false;
    }

    // Remove existing weapon at this attachment point
    detachWeapon(charInstanceId, attachmentId);

    // Load weapon model into renderer
    if (models.find(weaponModelId) == models.end()) {
        if (!loadModel(weaponModel, weaponModelId)) {
            core::Logger::getInstance().warning("attachWeapon: failed to load weapon model ", weaponModelId);
            return false;
        }
    }

    // Apply weapon texture if provided
    if (!texturePath.empty()) {
        GLuint texId = loadTexture(texturePath);
        if (texId != whiteTexture) {
            setModelTexture(weaponModelId, 0, texId);
        }
    }

    // Create weapon instance
    uint32_t weaponInstanceId = createInstance(weaponModelId, glm::vec3(0.0f));
    if (weaponInstanceId == 0) return false;

    // Mark weapon instance as override-positioned
    auto weapIt = instances.find(weaponInstanceId);
    if (weapIt != instances.end()) {
        weapIt->second.hasOverrideModelMatrix = true;
    }

    // Store attachment on parent character instance
    WeaponAttachment wa;
    wa.weaponModelId = weaponModelId;
    wa.weaponInstanceId = weaponInstanceId;
    wa.attachmentId = attachmentId;
    wa.boneIndex = boneIndex;
    wa.offset = offset;
    charInstance.weaponAttachments.push_back(wa);

    core::Logger::getInstance().info("Attached weapon model ", weaponModelId,
        " to instance ", charInstanceId, " at attachment ", attachmentId,
        " (bone ", boneIndex, ", offset ", offset.x, ",", offset.y, ",", offset.z, ")");
    return true;
}

void CharacterRenderer::detachWeapon(uint32_t charInstanceId, uint32_t attachmentId) {
    auto charIt = instances.find(charInstanceId);
    if (charIt == instances.end()) return;
    auto& attachments = charIt->second.weaponAttachments;

    for (auto it = attachments.begin(); it != attachments.end(); ++it) {
        if (it->attachmentId == attachmentId) {
            removeInstance(it->weaponInstanceId);
            attachments.erase(it);
            core::Logger::getInstance().info("Detached weapon from instance ", charInstanceId,
                " attachment ", attachmentId);
            return;
        }
    }
}

} // namespace rendering
} // namespace wowee
