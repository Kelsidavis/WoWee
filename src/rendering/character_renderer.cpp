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
#include <future>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <limits>

namespace wowee {
namespace rendering {

namespace {
size_t envSizeMBOrDefault(const char* name, size_t defMb) {
    const char* v = std::getenv(name);
    if (!v || !*v) return defMb;
    char* end = nullptr;
    unsigned long long mb = std::strtoull(v, &end, 10);
    if (end == v || mb == 0) return defMb;
    if (mb > (std::numeric_limits<size_t>::max() / (1024ull * 1024ull))) return defMb;
    return static_cast<size_t>(mb);
}

size_t approxTextureBytesWithMips(int w, int h) {
    if (w <= 0 || h <= 0) return 0;
    size_t base = static_cast<size_t>(w) * static_cast<size_t>(h) * 4ull;
    return base + (base / 3);  // ~4/3 for mip chain
}
} // namespace

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
        uniform mat4 uBones[240];

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
        uniform float uOpacity;
        uniform int uAlphaTest;
        uniform int uColorKeyBlack;
        uniform int uUnlit;
        uniform float uEmissiveBoost;
        uniform vec3 uEmissiveTint;

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
                    // Single hardware PCF tap — GL_LINEAR + compare mode gives 2×2 bilinear PCF for free
                    shadow = texture(uShadowMap, vec3(proj.xy, proj.z - bias));
                    shadow = mix(1.0, shadow, coverageFade);
                }
            }
            shadow = mix(1.0, shadow, clamp(uShadowStrength, 0.0, 1.0));

            // Ambient
            vec3 ambient = vec3(0.3);

            // Sample texture
            vec4 texColor = texture(uTexture0, TexCoord);
            if (uAlphaTest != 0 && texColor.a < 0.5) discard;
            if (uColorKeyBlack != 0) {
                float key = max(texColor.r, max(texColor.g, texColor.b));
                // Soft black-key: fade fringe instead of hard-cut to avoid dark halo.
                float keyAlpha = smoothstep(0.12, 0.30, key);
                texColor.a *= keyAlpha;
                if (texColor.a < 0.02) discard;
            }

            // Combine
            vec3 litResult = (ambient + (diff * vec3(1.0) + specular) * shadow) * texColor.rgb;
            vec3 warmBase = vec3(
                max(texColor.r, texColor.g * 0.92),
                texColor.g * 0.90,
                texColor.b * 0.45
            );
            vec3 emissiveResult = warmBase * uEmissiveTint * uEmissiveBoost;
            vec3 result = (uUnlit != 0) ? emissiveResult : litResult;

            // Fog
            float fogDist = length(uViewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            result = mix(uFogColor, result, fogFactor);

            // Apply texture alpha and instance opacity (for fade-in effects)
            float finalAlpha = texColor.a * uOpacity;
            if (finalAlpha < 0.02) discard;
            FragColor = vec4(result, finalAlpha);
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
        uniform mat4 uBones[240];

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
        uniform bool uColorKeyBlack;
        void main() {
            vec4 tex = texture(uTexture, vTexCoord);
            if (uAlphaTest && tex.a < 0.5) discard;
            if (uColorKeyBlack) {
                float key = max(tex.r, max(tex.g, tex.b));
                if (key < 0.14) discard;
            }
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

    // Create 1x1 transparent fallback texture for hidden texture slots.
    uint8_t transparent[] = { 0, 0, 0, 0 };
    glGenTextures(1, &transparentTexture);
    glBindTexture(GL_TEXTURE_2D, transparentTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, transparent);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Diagnostics-only: cache lifetime is currently tied to renderer lifetime.
    textureCacheBudgetBytes_ = envSizeMBOrDefault("WOWEE_CHARACTER_TEX_CACHE_MB", 2048) * 1024ull * 1024ull;

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
        GLuint texId = pair.second.id;
        if (texId && texId != whiteTexture) {
            glDeleteTextures(1, &texId);
        }
    }
    textureCache.clear();
    textureHasAlphaById_.clear();
    textureColorKeyBlackById_.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;

    if (whiteTexture) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }
    if (transparentTexture) {
        glDeleteTextures(1, &transparentTexture);
        transparentTexture = 0;
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

    auto normalizeKey = [](std::string key) {
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    };
    std::string key = normalizeKey(path);
    auto containsToken = [](const std::string& haystack, const char* token) {
        return haystack.find(token) != std::string::npos;
    };
    const bool colorKeyBlackHint =
        containsToken(key, "candle") ||
        containsToken(key, "flame") ||
        containsToken(key, "fire") ||
        containsToken(key, "torch");

    // Check cache
    auto it = textureCache.find(key);
    if (it != textureCache.end()) {
        it->second.lastUse = ++textureCacheCounter_;
        return it->second.id;
    }

    if (!assetManager || !assetManager->isInitialized()) {
        return whiteTexture;
    }

    // Check negative cache to avoid repeated file I/O for textures that don't exist
    if (failedTextureCache_.count(key)) {
        return whiteTexture;
    }

    auto blpImage = assetManager->loadTexture(key);
    if (!blpImage.isValid()) {
        core::Logger::getInstance().warning("Failed to load texture: ", path);
        failedTextureCache_.insert(key);
        return whiteTexture;
    }

    bool hasAlpha = false;
    for (size_t i = 3; i < blpImage.data.size(); i += 4) {
        if (blpImage.data[i] != 255) {
            hasAlpha = true;
            break;
        }
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

    TextureCacheEntry e;
    e.id = texId;
    e.approxBytes = approxTextureBytesWithMips(blpImage.width, blpImage.height);
    e.lastUse = ++textureCacheCounter_;
    e.hasAlpha = hasAlpha;
    e.colorKeyBlack = colorKeyBlackHint;
    textureCacheBytes_ += e.approxBytes;
    textureCache[key] = e;
    textureHasAlphaById_[texId] = hasAlpha;
    textureColorKeyBlackById_[texId] = colorKeyBlackHint;
    if (textureCacheBytes_ > textureCacheBudgetBytes_) {
        core::Logger::getInstance().warning(
            "Character texture cache over budget: ",
            textureCacheBytes_ / (1024 * 1024), " MB > ",
            textureCacheBudgetBytes_ / (1024 * 1024), " MB (textures=", textureCache.size(), ")");
    }
    core::Logger::getInstance().debug("Loaded character texture: ", path, " (", blpImage.width, "x", blpImage.height, ")");
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

// Nearest-neighbor NxN scale blit of overlay onto composite at (dstX, dstY)
static void blitOverlayScaledN(std::vector<uint8_t>& composite, int compW, int compH,
                                const pipeline::BLPImage& overlay, int dstX, int dstY, int scale) {
    if (scale < 1) scale = 1;
    for (int sy = 0; sy < overlay.height; sy++) {
        for (int sx = 0; sx < overlay.width; sx++) {
            size_t srcIdx = (static_cast<size_t>(sy) * overlay.width + sx) * 4;
            uint8_t srcA = overlay.data[srcIdx + 3];
            if (srcA == 0) continue;

            // Write to scale×scale block of destination pixels
            for (int dy2 = 0; dy2 < scale; dy2++) {
                int dy = dstY + sy * scale + dy2;
                if (dy < 0 || dy >= compH) continue;
                for (int dx2 = 0; dx2 < scale; dx2++) {
                    int dx = dstX + sx * scale + dx2;
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

// Legacy 2x wrapper
static void blitOverlayScaled2x(std::vector<uint8_t>& composite, int compW, int compH,
                                 const pipeline::BLPImage& overlay, int dstX, int dstY) {
    blitOverlayScaledN(composite, compW, compH, overlay, dstX, dstY, 2);
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

    // WoW character texture atlas regions (from WoW Model Viewer / CharComponentTextureSections)
    // Coordinates at 256x256 base resolution:
    // Region          X    Y    W    H
    // Base            0    0    256  256
    // Arm Upper       0    0    128  64
    // Arm Lower       0    64   128  64
    // Hand            0    128  128  32
    // Face Upper      0    160  128  32
    // Face Lower      0    192  128  64
    // Torso Upper     128  0    128  64
    // Torso Lower     128  64   128  32
    // Pelvis Upper    128  96   128  64
    // Pelvis Lower    128  160  128  64
    // Foot            128  224  128  32

    // Scale factor: base texture may be larger than the 256x256 reference atlas
    int coordScale = width / 256;
    if (coordScale < 1) coordScale = 1;

    // Atlas region sizes at 256x256 base (w, h) for known regions
    struct AtlasRegion { int x, y, w, h; };
    static const AtlasRegion faceLowerRegion256 = {0, 192, 128, 64};
    static const AtlasRegion faceUpperRegion256 = {0, 160, 128, 32};

    // Alpha-blend each overlay onto the composite
    for (size_t layer = 1; layer < layerPaths.size(); layer++) {
        if (layerPaths[layer].empty()) continue;

        auto overlay = assetManager->loadTexture(layerPaths[layer]);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("Composite: FAILED to load overlay: ", layerPaths[layer]);
            continue;
        }

        core::Logger::getInstance().info("Composite: overlay ", layerPaths[layer],
            " (", overlay.width, "x", overlay.height, ")");

        if (overlay.width == width && overlay.height == height) {
            // Same size: full alpha-blend
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // Determine region by filename keywords
            // Coordinates scale with base texture size (256x256 is reference)
            int dstX = 0, dstY = 0;
            int expectedW256 = 0, expectedH256 = 0; // Expected size at 256-base
            std::string pathLower = layerPaths[layer];
            for (auto& c : pathLower) c = std::tolower(c);

            if (pathLower.find("faceupper") != std::string::npos) {
                dstX = faceUpperRegion256.x; dstY = faceUpperRegion256.y;
                expectedW256 = faceUpperRegion256.w; expectedH256 = faceUpperRegion256.h;
            } else if (pathLower.find("facelower") != std::string::npos) {
                dstX = faceLowerRegion256.x; dstY = faceLowerRegion256.y;
                expectedW256 = faceLowerRegion256.w; expectedH256 = faceLowerRegion256.h;
            } else if (pathLower.find("pelvis") != std::string::npos) {
                dstX = 128; dstY = 96;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("torso") != std::string::npos) {
                dstX = 128; dstY = 0;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 64;
                expectedW256 = 128; expectedH256 = 64;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 128;
                expectedW256 = 128; expectedH256 = 32;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 128; dstY = 224;
                expectedW256 = 128; expectedH256 = 32;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 128; dstY = 160;
                expectedW256 = 128; expectedH256 = 64;
            } else {
                // Unknown — center placement as fallback
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
                core::Logger::getInstance().info("Composite: UNKNOWN region for '",
                    layerPaths[layer], "', centering at (", dstX, ",", dstY, ")");
                blitOverlay(composite, width, height, overlay, dstX, dstY);
                continue;
            }

            // Scale coordinates from 256-base to actual canvas
            dstX *= coordScale;
            dstY *= coordScale;

            // If overlay is 256-base sized but canvas is larger, scale the overlay up
            int expectedW = expectedW256 * coordScale;
            int expectedH = expectedH256 * coordScale;
            bool needsScale = (coordScale > 1 &&
                               overlay.width == expectedW256 && overlay.height == expectedH256);

            core::Logger::getInstance().info("Composite: placing '", layerPaths[layer],
                "' (", overlay.width, "x", overlay.height,
                ") at (", dstX, ",", dstY, ") on ", width, "x", height,
                " expected=", expectedW, "x", expectedH,
                needsScale ? " [SCALING]" : "");

            if (needsScale) {
                blitOverlayScaledN(composite, width, height, overlay, dstX, dstY, coordScale);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        }
    }

    // Debug: dump composite to /tmp for visual inspection
    {
        std::string dumpPath = "/tmp/wowee_composite_debug_" +
            std::to_string(width) + "x" + std::to_string(height) + ".raw";
        std::ofstream dump(dumpPath, std::ios::binary);
        if (dump) {
            dump.write(reinterpret_cast<const char*>(composite.data()),
                       static_cast<std::streamsize>(composite.size()));
            core::Logger::getInstance().info("Composite debug dump: ", dumpPath,
                " (", width, "x", height, ", ", composite.size(), " bytes)");
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

void CharacterRenderer::clearCompositeCache() {
    // Just clear the lookup map so next compositeWithRegions() creates fresh textures.
    // Don't delete GPU textures — they may still be referenced by models or instances.
    // Orphaned textures will be cleaned up when their model/instance is destroyed.
    compositeCache_.clear();
}

GLuint CharacterRenderer::compositeWithRegions(const std::string& basePath,
                                                const std::vector<std::string>& baseLayers,
                                                const std::vector<std::pair<int, std::string>>& regionLayers) {
    // Build cache key from all inputs to avoid redundant compositing
    std::string cacheKey = basePath;
    for (const auto& bl : baseLayers) { cacheKey += '|'; cacheKey += bl; }
    cacheKey += '#';
    for (const auto& rl : regionLayers) {
        cacheKey += std::to_string(rl.first);
        cacheKey += ':';
        cacheKey += rl.second;
        cacheKey += ',';
    }
    auto cacheIt = compositeCache_.find(cacheKey);
    if (cacheIt != compositeCache_.end() && cacheIt->second != 0) {
        return cacheIt->second;
    }

    // Region index → pixel coordinates on the 256x256 base atlas
    // These are scaled up by (width/256, height/256) for larger textures (512x512, 1024x1024)
    static const int regionCoords256[][2] = {
        {   0,   0 },  // 0 = ArmUpper
        {   0,  64 },  // 1 = ArmLower
        {   0, 128 },  // 2 = Hand
        { 128,   0 },  // 3 = TorsoUpper
        { 128,  64 },  // 4 = TorsoLower
        { 128,  96 },  // 5 = LegUpper
        { 128, 160 },  // 6 = LegLower
        { 128, 224 },  // 7 = Foot
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
        core::Logger::getInstance().debug("compositeWithRegions: upscaled 256x256 to 512x512");
    } else {
        composite = base.data;
    }

    // Blend face + underwear overlays
    // If we upscaled from 256→512, scale coords and texels with blitOverlayScaled2x.
    // For native 512/1024 textures, face overlays are full atlas size (hit width==width branch).
    bool upscaled = (base.width == 256 && base.height == 256 && width == 512);
    for (const auto& ul : baseLayers) {
        if (ul.empty()) continue;
        auto overlay = assetManager->loadTexture(ul);
        if (!overlay.isValid()) continue;

        if (overlay.width == width && overlay.height == height) {
            blitOverlay(composite, width, height, overlay, 0, 0);
        } else {
            // WoW 256-scale atlas coordinates (from CharComponentTextureSections)
            int dstX = 0, dstY = 0;
            std::string pathLower = ul;
            for (auto& c : pathLower) c = std::tolower(c);

            // Scale factor from 256-base coordinates to actual canvas size
            int coordScale = width / 256;
            if (coordScale < 1) coordScale = 1;
            bool useScale = true;

            if (pathLower.find("faceupper") != std::string::npos) {
                dstX = 0; dstY = 160;
            } else if (pathLower.find("facelower") != std::string::npos) {
                dstX = 0; dstY = 192;
            } else if (pathLower.find("pelvis") != std::string::npos) {
                dstX = 128; dstY = 96;
            } else if (pathLower.find("torso") != std::string::npos) {
                dstX = 128; dstY = 0;
            } else if (pathLower.find("armupper") != std::string::npos) {
                dstX = 0; dstY = 0;
            } else if (pathLower.find("armlower") != std::string::npos) {
                dstX = 0; dstY = 64;
            } else if (pathLower.find("hand") != std::string::npos) {
                dstX = 0; dstY = 128;
            } else if (pathLower.find("foot") != std::string::npos || pathLower.find("feet") != std::string::npos) {
                dstX = 128; dstY = 224;
            } else if (pathLower.find("legupper") != std::string::npos || pathLower.find("leg") != std::string::npos) {
                dstX = 128; dstY = 160;
            } else {
                // Fallback: center overlay on canvas (already in canvas coords)
                dstX = (width - overlay.width) / 2;
                dstY = (height - overlay.height) / 2;
                useScale = false;
            }

            if (useScale) {
                dstX *= coordScale;
                dstY *= coordScale;
            }

            if (upscaled) {
                // Overlay is 256-base sized, needs 2x texel scaling for 512 canvas
                blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
            } else {
                blitOverlay(composite, width, height, overlay, dstX, dstY);
            }
        }
    }

    // Expected region sizes on the 256x256 base atlas (scaled like coords)
    static const int regionSizes256[][2] = {
        { 128,  64 },  // 0 = ArmUpper
        { 128,  64 },  // 1 = ArmLower
        { 128,  32 },  // 2 = Hand
        { 128,  64 },  // 3 = TorsoUpper
        { 128,  32 },  // 4 = TorsoLower
        { 128,  64 },  // 5 = LegUpper
        { 128,  64 },  // 6 = LegLower
        { 128,  32 },  // 7 = Foot
    };

    // Scale factor from 256-base to actual texture size
    int scaleX = width / 256;
    int scaleY = height / 256;
    if (scaleX < 1) scaleX = 1;
    if (scaleY < 1) scaleY = 1;

    // Now blit equipment region textures at explicit coordinates
    for (const auto& rl : regionLayers) {
        int regionIdx = rl.first;
        if (regionIdx < 0 || regionIdx >= 8) continue;

        auto overlay = assetManager->loadTexture(rl.second);
        if (!overlay.isValid()) {
            core::Logger::getInstance().warning("compositeWithRegions: failed to load ", rl.second);
            continue;
        }

        int dstX = regionCoords256[regionIdx][0] * scaleX;
        int dstY = regionCoords256[regionIdx][1] * scaleY;

        // Expected full-resolution size for this region at current atlas scale
        int expectedW = regionSizes256[regionIdx][0] * scaleX;
        int expectedH = regionSizes256[regionIdx][1] * scaleY;
        if (overlay.width * 2 == expectedW && overlay.height * 2 == expectedH) {
            blitOverlayScaled2x(composite, width, height, overlay, dstX, dstY);
        } else {
            blitOverlay(composite, width, height, overlay, dstX, dstY);
        }

        core::Logger::getInstance().debug("compositeWithRegions: region ", regionIdx,
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

    core::Logger::getInstance().debug("compositeWithRegions: created ", width, "x", height,
        " texture with ", regionLayers.size(), " equipment regions");
    compositeCache_[cacheKey] = texId;
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
            if (v.id == oldTex) { cached = true; break; }
        }
        if (!cached) {
            glDeleteTextures(1, &oldTex);
        }
    }

    gpuModel.textureIds[textureSlot] = textureId;
    core::Logger::getInstance().debug("Replaced model ", modelId, " texture slot ", textureSlot, " with composited texture");
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

    core::Logger::getInstance().debug("Loaded M2 model ", id, " (", model.vertices.size(),
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

    // Track death state for preventing movement while dead
    if (animationId == 1) {
        instance.isDead = true;
    } else if (instance.isDead && animationId == 0) {
        instance.isDead = false;  // Respawned
    }

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

        // Only log missing animation once per model (reduce spam)
        static std::unordered_map<uint32_t, std::unordered_set<uint32_t>> loggedMissingAnims;
        uint32_t modelId = instance.modelId;  // Use modelId as identifier
        if (loggedMissingAnims[modelId].insert(animationId).second) {
            // First time seeing this missing animation for this model
            LOG_WARNING("Animation ", animationId, " not found in model ", modelId, ", using default");
        }
    }
}

void CharacterRenderer::update(float deltaTime, const glm::vec3& cameraPos) {
    // Distance culling for animation updates (150 unit radius)
    const float animUpdateRadiusSq = 150.0f * 150.0f;

    // Update fade-in opacity
    for (auto& [id, inst] : instances) {
        if (inst.fadeInDuration > 0.0f && inst.opacity < 1.0f) {
            inst.fadeInTime += deltaTime;
            inst.opacity = std::min(1.0f, inst.fadeInTime / inst.fadeInDuration);
            if (inst.opacity >= 1.0f) {
                inst.fadeInDuration = 0.0f;
            }
        }
    }

    // Interpolate creature movement
    for (auto& [id, inst] : instances) {
        if (inst.isMoving) {
            inst.moveElapsed += deltaTime;
            float t = inst.moveElapsed / inst.moveDuration;
            if (t >= 1.0f) {
                inst.position = inst.moveEnd;
                inst.isMoving = false;
                // Return to idle when movement completes
                if (inst.currentAnimationId == 4 || inst.currentAnimationId == 5) {
                    playAnimation(id, 0, true);
                }
            } else {
                inst.position = glm::mix(inst.moveStart, inst.moveEnd, t);
            }
        }
    }

    // Only update animations for nearby characters (performance optimization)
    // Collect instances that need updates
    std::vector<std::reference_wrapper<CharacterInstance>> toUpdate;
    toUpdate.reserve(instances.size());

    for (auto& pair : instances) {
        float distSq = glm::distance2(pair.second.position, cameraPos);
        if (distSq < animUpdateRadiusSq) {
            toUpdate.push_back(std::ref(pair.second));
        }
    }

    int updatedCount = toUpdate.size();

    // Thread bone calculations if we have many characters (4+)
    if (updatedCount >= 4) {
        std::vector<std::future<void>> futures;
        futures.reserve(updatedCount);

        for (auto& instRef : toUpdate) {
            futures.push_back(std::async(std::launch::async, [this, &instRef, deltaTime]() {
                updateAnimation(instRef.get(), deltaTime);
            }));
        }

        // Wait for all to complete
        for (auto& f : futures) {
            f.get();
        }
    } else {
        // Sequential for small counts (avoid thread overhead)
        for (auto& instRef : toUpdate) {
            updateAnimation(instRef.get(), deltaTime);
        }
    }

    static int logCounter = 0;
    if (++logCounter >= 300) {  // Log every 10 seconds at 30fps
        LOG_DEBUG("CharacterRenderer: ", updatedCount, "/", instances.size(), " instances updated (",
                 instances.size() - updatedCount, " culled)");
        logCounter = 0;
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
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    characterShader->use();
    characterShader->setUniform("uView", view);
    characterShader->setUniform("uProjection", projection);
    characterShader->setUniform("uLightDir", lightDir);
    characterShader->setUniform("uLightColor", lightColor);
    characterShader->setUniform("uSpecularIntensity", 0.5f);
    characterShader->setUniform("uViewPos", camera.getPosition());

    // Fog
    characterShader->setUniform("uFogColor", fogColor);
    characterShader->setUniform("uFogStart", fogStart);
    characterShader->setUniform("uFogEnd", fogEnd);

    // Shadows
    characterShader->setUniform("uShadowEnabled", shadowEnabled ? 1 : 0);
    characterShader->setUniform("uShadowStrength", 0.68f);
    characterShader->setUniform("uTexture0", 0);
    characterShader->setUniform("uAlphaTest", 0);
    characterShader->setUniform("uColorKeyBlack", 0);
    characterShader->setUniform("uUnlit", 0);
    characterShader->setUniform("uEmissiveBoost", 1.0f);
    characterShader->setUniform("uEmissiveTint", glm::vec3(1.0f));
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

        // Skip fully transparent instances
        if (instance.opacity <= 0.0f) continue;

        // Set model matrix (use override for weapon instances)
        glm::mat4 modelMat = instance.hasOverrideModelMatrix
            ? instance.overrideModelMatrix
            : getModelMatrix(instance);
        characterShader->setUniform("uModel", modelMat);
        characterShader->setUniform("uOpacity", instance.opacity);

        // Set bone matrices (upload all at once for performance)
        int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), MAX_BONES);
        if (numBones > 0) {
            characterShader->setUniformMatrixArray("uBones[0]", instance.boneMatrices.data(), numBones);
        }

        // Bind VAO and draw
        glBindVertexArray(gpuModel.vao);

	        if (!gpuModel.data.batches.empty()) {
	            bool applyGeosetFilter = !instance.activeGeosets.empty();
	            if (applyGeosetFilter) {
	                bool hasRenderableGeoset = false;
	                for (const auto& batch : gpuModel.data.batches) {
	                    if (instance.activeGeosets.find(batch.submeshId) != instance.activeGeosets.end()) {
	                        hasRenderableGeoset = true;
	                        break;
	                    }
	                }
                if (!hasRenderableGeoset) {
                    static std::unordered_set<uint32_t> loggedGeosetFallback;
                    if (loggedGeosetFallback.insert(instance.id).second) {
                        LOG_WARNING("Geoset filter matched no batches for instance ",
                                    instance.id, " (model ", instance.modelId,
                                    "); rendering all batches as fallback");
                    }
	                    applyGeosetFilter = false;
	                }
	            }

	            auto resolveBatchTexture = [&](const CharacterInstance& inst, const M2ModelGPU& gm, const pipeline::M2Batch& b) -> GLuint {
	                // A skin batch can reference multiple textures (b.textureCount) starting at b.textureIndex.
	                // We currently bind only a single texture, so pick the most appropriate one.
	                //
	                // This matters for hair: the first texture in the combo can be a mask/empty slot,
	                // causing the hair to render as solid white.
	                if (b.textureIndex == 0xFFFF) return whiteTexture;
	                if (gm.data.textureLookup.empty() || gm.textureIds.empty()) return whiteTexture;

	                uint32_t comboCount = b.textureCount ? static_cast<uint32_t>(b.textureCount) : 1u;
	                comboCount = std::min<uint32_t>(comboCount, 8u);

	                struct Candidate { GLuint id; uint32_t type; };
	                Candidate first{whiteTexture, 0};
	                bool hasFirst = false;
	                Candidate firstNonWhite{whiteTexture, 0};
	                bool hasFirstNonWhite = false;

	                for (uint32_t i = 0; i < comboCount; i++) {
	                    uint32_t lookupPos = static_cast<uint32_t>(b.textureIndex) + i;
	                    if (lookupPos >= gm.data.textureLookup.size()) break;
	                    uint16_t texSlot = gm.data.textureLookup[lookupPos];
	                    if (texSlot >= gm.textureIds.size()) continue;

	                    GLuint texId = gm.textureIds[texSlot];
	                    uint32_t texType = (texSlot < gm.data.textures.size()) ? gm.data.textures[texSlot].type : 0;
                        // Apply texture slot overrides.
                        // For type-1 (skin) overrides, only apply to skin-group batches
                        // to prevent the skin composite from bleeding onto cloak/hair.
                        {
                            auto itO = inst.textureSlotOverrides.find(texSlot);
                            if (itO != inst.textureSlotOverrides.end() && itO->second != 0) {
                                if (texType == 1) {
                                    // Only apply skin override to skin groups
                                    uint16_t grp = b.submeshId / 100;
                                    bool isSkinGroup = (grp == 0 || grp == 3 || grp == 4 || grp == 5 ||
                                                        grp == 8 || grp == 9 || grp == 13 || grp == 20);
                                    if (isSkinGroup) texId = itO->second;
                                } else {
                                    texId = itO->second;
                                }
                            }
                        }

	                    if (!hasFirst) {
	                        first = {texId, texType};
	                        hasFirst = true;
	                    }

	                    if (texId == 0 || texId == whiteTexture) continue;

	                    // Prefer the hair texture slot (type 6) whenever present in the combo.
	                    // Humanoid scalp meshes can live in group 0, so group-based checks are insufficient.
	                    if (texType == 6) {
	                        return texId;
	                    }

	                    if (!hasFirstNonWhite) {
	                        firstNonWhite = {texId, texType};
	                        hasFirstNonWhite = true;
	                    }
	                }

	                if (hasFirstNonWhite) return firstNonWhite.id;
	                if (hasFirst && first.id != 0) return first.id;
	                return whiteTexture;
	            };

	            // One-time debug dump of rendered batches per model
	            static std::unordered_set<uint32_t> dumpedModels;
	            if (dumpedModels.find(instance.modelId) == dumpedModels.end()) {
                dumpedModels.insert(instance.modelId);
                int bIdx = 0;
                int rendered = 0, skipped = 0;
                for (const auto& b : gpuModel.data.batches) {
                    bool filtered = applyGeosetFilter &&
                        (b.submeshId / 100 != 0) &&
                        instance.activeGeosets.find(b.submeshId) == instance.activeGeosets.end();

	                    GLuint resolvedTex = resolveBatchTexture(instance, gpuModel, b);
	                    std::string texInfo = "GL" + std::to_string(resolvedTex);

	                    if (filtered) skipped++; else rendered++;
	                    LOG_DEBUG("Batch ", bIdx, ": submesh=", b.submeshId,
                              " level=", b.submeshLevel,
                              " idxStart=", b.indexStart, " idxCount=", b.indexCount,
                              " tex=", texInfo,
                              filtered ? " [SKIP]" : " [RENDER]");
                    bIdx++;
                }
                LOG_DEBUG("Batch summary: ", rendered, " rendered, ", skipped, " skipped, ",
                          gpuModel.textureIds.size(), " textures loaded, ",
                          gpuModel.data.textureLookup.size(), " in lookup table");
                for (size_t t = 0; t < gpuModel.data.textures.size(); t++) {
	                }
	            }

	            // Draw batches (submeshes) with per-batch textures
	            // Geoset filtering: skip batches whose submeshId is not in activeGeosets.
	            // For character models, group 0 (body/scalp) is also filtered so that only
	            // the correct scalp mesh renders (not all overlapping variants).
            for (const auto& batch : gpuModel.data.batches) {
                if (applyGeosetFilter) {
                    if (instance.activeGeosets.find(batch.submeshId) == instance.activeGeosets.end()) {
                        continue;
                    }
                }

	                // Resolve texture for this batch (prefer hair textures for hair geosets).
	                GLuint texId = resolveBatchTexture(instance, gpuModel, batch);
                const uint16_t batchGroup = static_cast<uint16_t>(batch.submeshId / 100);
                auto groupTexIt = instance.groupTextureOverrides.find(batchGroup);
                if (groupTexIt != instance.groupTextureOverrides.end() && groupTexIt->second != 0) {
                    texId = groupTexIt->second;
                }

                // Respect M2 material blend mode for creature/character submeshes.
                uint16_t blendMode = 0;
                uint16_t materialFlags = 0;
                if (batch.materialIndex < gpuModel.data.materials.size()) {
                    blendMode = gpuModel.data.materials[batch.materialIndex].blendMode;
                    materialFlags = gpuModel.data.materials[batch.materialIndex].flags;
                }

                // Attached weapon models can include additive FX/card batches that
                // appear as detached flat quads for some swords. Keep core geometry
                // and drop FX-style passes for weapon attachments.
                if (instance.hasOverrideModelMatrix && blendMode >= 3) {
                    continue;
                }
                switch (blendMode) {
                    case 0: glBlendFunc(GL_ONE, GL_ZERO); break;                      // Opaque
                    case 1: glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break; // AlphaKey
                    case 2: glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break; // Alpha
                    case 3: glBlendFunc(GL_SRC_ALPHA, GL_ONE); break;                 // Additive
                    case 4: glBlendFunc(GL_DST_COLOR, GL_ZERO); break;                // Mod
                    case 5: glBlendFunc(GL_DST_COLOR, GL_SRC_COLOR); break;           // Mod2x
                    case 6: glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); break;       // BlendAdd
                    default: glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break;
                }


                // For body/equipment parts with white/fallback texture, use skin (type 1) texture.
                // Groups that share the body skin atlas: 0=body, 3=gloves, 4=boots, 5=chest,
                // 8=wristbands, 9=pelvis, 13=pants. Hair (group 1) and facial hair (group 2) do NOT.
                if (texId == whiteTexture) {
                    uint16_t group = batchGroup;
                    bool isSkinGroup = (group == 0 || group == 3 || group == 4 || group == 5 ||
                                        group == 8 || group == 9 || group == 13);
                    if (isSkinGroup) {
                        // Check if this batch's texture slot is a hair type (don't override hair)
                        uint32_t texType = 0;
                        if (batch.textureIndex < gpuModel.data.textureLookup.size()) {
                            uint16_t lk = gpuModel.data.textureLookup[batch.textureIndex];
                            if (lk < gpuModel.data.textures.size()) {
                                texType = gpuModel.data.textures[lk].type;
                            }
                        }
                        // Do NOT apply skin composite to hair (type 6) batches
                        if (texType != 6) {
                            for (size_t ti = 0; ti < gpuModel.textureIds.size(); ti++) {
                                GLuint candidate = gpuModel.textureIds[ti];
                                auto itO = instance.textureSlotOverrides.find(static_cast<uint16_t>(ti));
                                if (itO != instance.textureSlotOverrides.end() && itO->second != 0) {
                                    candidate = itO->second;
                                }
                                if (candidate != whiteTexture && candidate != 0) {
                                    // Only use type 1 (skin) textures as fallback
                                    if (ti < gpuModel.data.textures.size() &&
                                        (gpuModel.data.textures[ti].type == 1 || gpuModel.data.textures[ti].type == 11)) {
                                        texId = candidate;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, texId);
                bool alphaCutout = false;
                bool colorKeyBlack = false;
                if (texId != 0 && texId != whiteTexture) {
                    auto ait = textureHasAlphaById_.find(texId);
                    alphaCutout = (ait != textureHasAlphaById_.end()) ? ait->second : false;
                    auto cit = textureColorKeyBlackById_.find(texId);
                    colorKeyBlack = (cit != textureColorKeyBlackById_.end()) ? cit->second : false;
                }
                const bool blendNeedsCutout = (blendMode == 1) || (blendMode >= 2 && !alphaCutout);
                characterShader->setUniform("uAlphaTest", (blendNeedsCutout || alphaCutout) ? 1 : 0);
                characterShader->setUniform("uColorKeyBlack", (blendNeedsCutout || colorKeyBlack) ? 1 : 0);
                const bool unlit = ((materialFlags & 0x01) != 0) || (blendMode >= 3);
                characterShader->setUniform("uUnlit", unlit ? 1 : 0);
                float emissiveBoost = 1.0f;
                glm::vec3 emissiveTint(1.0f, 1.0f, 1.0f);
                // Keep custom warm/flicker treatment narrowly scoped to kobold candle flames.
                bool koboldCandleFlame = false;
                if (colorKeyBlack) {
                    std::string modelKey = gpuModel.data.name;
                    std::transform(modelKey.begin(), modelKey.end(), modelKey.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    koboldCandleFlame =
                        (modelKey.find("kobold") != std::string::npos) &&
                        ((modelKey.find("candle") != std::string::npos) ||
                         (modelKey.find("torch") != std::string::npos) ||
                         (modelKey.find("mine") != std::string::npos));
                }
                if (unlit && koboldCandleFlame) {
                    using clock = std::chrono::steady_clock;
                    float t = std::chrono::duration<float>(clock::now().time_since_epoch()).count();
                    float phase = static_cast<float>(batch.submeshId) * 0.31f;
                    float f1 = std::sin(t * 7.9f + phase);
                    float f2 = std::sin(t * 12.7f + phase * 1.73f);
                    float f3 = std::sin(t * 4.3f + phase * 2.11f);
                    float flicker = 0.90f + 0.10f * f1 + 0.06f * f2 + 0.04f * f3;
                    flicker = std::clamp(flicker, 0.72f, 1.12f);
                    emissiveBoost = (blendMode >= 3) ? (2.4f * flicker) : (1.5f * flicker);
                    // Warm flame bias to avoid green cast from source textures.
                    emissiveTint = glm::vec3(1.28f, 1.04f, 0.82f);
                }
                characterShader->setUniform("uEmissiveBoost", emissiveBoost);
                characterShader->setUniform("uEmissiveTint", emissiveTint);

                glDrawElements(GL_TRIANGLES,
                               batch.indexCount,
                               GL_UNSIGNED_SHORT,
                               (void*)(batch.indexStart * sizeof(uint16_t)));
            }
        } else {
            // Draw entire model with first texture
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, !gpuModel.textureIds.empty() ? gpuModel.textureIds[0] : whiteTexture);
            characterShader->setUniform("uAlphaTest", 0);
            characterShader->setUniform("uColorKeyBlack", 0);
            characterShader->setUniform("uUnlit", 0);
            characterShader->setUniform("uEmissiveBoost", 1.0f);
            characterShader->setUniform("uEmissiveTint", glm::vec3(1.0f));

            glDrawElements(GL_TRIANGLES,
                          static_cast<GLsizei>(gpuModel.data.indices.size()),
                          GL_UNSIGNED_SHORT,
                          0);
        }
    }

    glBindVertexArray(0);
    glDisable(GL_BLEND);
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
    GLint colorKeyLoc = glGetUniformLocation(shadowCasterProgram, "uColorKeyBlack");
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
                        auto itO = instance.textureSlotOverrides.find(lookupIdx);
                        if (itO != instance.textureSlotOverrides.end() && itO->second != 0) {
                            texId = itO->second;
                        }
                    }
                }

                bool alphaCutout = false;
                bool colorKeyBlack = false;
                if (texId != 0 && texId != whiteTexture) {
                    auto itA = textureHasAlphaById_.find(texId);
                    alphaCutout = (itA != textureHasAlphaById_.end()) ? itA->second : false;
                    auto itC = textureColorKeyBlackById_.find(texId);
                    colorKeyBlack = (itC != textureColorKeyBlackById_.end()) ? itC->second : false;
                }
                if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, alphaCutout ? 1 : 0);
                if (colorKeyLoc >= 0) glUniform1i(colorKeyLoc, colorKeyBlack ? 1 : 0);
                glBindTexture(GL_TEXTURE_2D, texId ? texId : whiteTexture);

                glDrawElements(GL_TRIANGLES,
                               batch.indexCount,
                               GL_UNSIGNED_SHORT,
                               (void*)(batch.indexStart * sizeof(uint16_t)));
            }
        } else {
            if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, 0);
            if (colorKeyLoc >= 0) glUniform1i(colorKeyLoc, 0);
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

    // Apply rotation (euler angles, Z-up)
    // Convention: yaw around Z, pitch around X, roll around Y.
    model = glm::rotate(model, instance.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));  // Yaw
    model = glm::rotate(model, instance.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));  // Pitch
    model = glm::rotate(model, instance.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));  // Roll

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

void CharacterRenderer::moveInstanceTo(uint32_t instanceId, const glm::vec3& destination, float durationSeconds) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;

    auto& inst = it->second;

    // Don't move dead instances (corpses shouldn't slide around)
    if (inst.isDead) return;

    auto pickMoveAnim = [&](bool preferRun) -> uint32_t {
        // Choose movement anim from estimated speed; fall back if missing.
        if (preferRun) {
            if (hasAnimation(instanceId, 5)) return 5; // Run
            if (hasAnimation(instanceId, 4)) return 4; // Walk
        } else {
            if (hasAnimation(instanceId, 4)) return 4; // Walk
            if (hasAnimation(instanceId, 5)) return 5; // Run
        }
        return 0;
    };

    float planarDist = glm::length(glm::vec2(destination.x - inst.position.x,
                                             destination.y - inst.position.y));
    bool synthesizedDuration = false;
    if (durationSeconds <= 0.0f) {
        if (planarDist < 0.01f) {
            // Stop at current location.
            inst.position = destination;
            inst.isMoving = false;
            if (inst.currentAnimationId == 4 || inst.currentAnimationId == 5) {
                playAnimation(instanceId, 0, true);
            }
            return;
        }
        // Some cores send movement-only deltas without spline duration.
        // Synthesize a tiny duration so movement anim/rotation still updates.
        durationSeconds = std::clamp(planarDist / 7.0f, 0.05f, 0.20f);
        synthesizedDuration = true;
    }

    inst.moveStart = inst.position;
    inst.moveEnd = destination;
    inst.moveDuration = durationSeconds;
    inst.moveElapsed = 0.0f;
    inst.isMoving = true;

    // Face toward destination (yaw around Z axis since Z is up)
    glm::vec3 dir = destination - inst.position;
    if (glm::length(glm::vec2(dir.x, dir.y)) > 0.001f) {
        float angle = std::atan2(dir.y, dir.x);
        inst.rotation.z = angle;
    }

    // Play movement animation while moving.
    // Prefer run only when speed is clearly above normal walk pace.
    float moveSpeed = planarDist / std::max(durationSeconds, 0.001f);
    bool preferRun = (!synthesizedDuration && moveSpeed >= 4.5f);
    uint32_t moveAnim = pickMoveAnim(preferRun);
    if (moveAnim != 0 && inst.currentAnimationId != moveAnim) {
        playAnimation(instanceId, moveAnim, true);
    }
}

const pipeline::M2Model* CharacterRenderer::getModelData(uint32_t modelId) const {
    auto it = models.find(modelId);
    if (it == models.end()) return nullptr;
    return &it->second.data;
}

void CharacterRenderer::startFadeIn(uint32_t instanceId, float durationSeconds) {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;
    it->second.opacity = 0.0f;
    it->second.fadeInTime = 0.0f;
    it->second.fadeInDuration = durationSeconds;
}

void CharacterRenderer::setActiveGeosets(uint32_t instanceId, const std::unordered_set<uint16_t>& geosets) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.activeGeosets = geosets;
    }
}

void CharacterRenderer::setGroupTextureOverride(uint32_t instanceId, uint16_t geosetGroup, GLuint textureId) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.groupTextureOverrides[geosetGroup] = textureId;
    }
}

void CharacterRenderer::setTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot, GLuint textureId) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.textureSlotOverrides[textureSlot] = textureId;
    }
}

void CharacterRenderer::clearTextureSlotOverride(uint32_t instanceId, uint16_t textureSlot) {
    auto it = instances.find(instanceId);
    if (it != instances.end()) {
        it->second.textureSlotOverrides.erase(textureSlot);
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
    auto it = instances.find(instanceId);
    if (it == instances.end()) return;

    // Remove child attachments first (helmets/weapons), otherwise they leak as
    // orphan render instances when the parent creature despawns.
    auto attachments = it->second.weaponAttachments;
    for (const auto& wa : attachments) {
        removeInstance(wa.weaponInstanceId);
    }

    instances.erase(it);
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
    // Fallback to key-bone lookup only for weapon hand attachment IDs.
    if (!found && (attachmentId == 1 || attachmentId == 2)) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < charModel.bones.size(); i++) {
            if (charModel.bones[i].keyBoneId == targetKeyBone) {
                boneIndex = static_cast<uint16_t>(i);
                found = true;
                break;
            }
        }
    }

    // Validate bone index (bad attachment tables should not silently bind to origin)
    if (found && boneIndex >= charModel.bones.size()) {
        found = false;
    }
    if (!found && (attachmentId == 1 || attachmentId == 2)) {
        int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
        for (size_t i = 0; i < charModel.bones.size(); i++) {
            if (charModel.bones[i].keyBoneId == targetKeyBone) {
                boneIndex = static_cast<uint16_t>(i);
                offset = glm::vec3(0.0f);
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

    core::Logger::getInstance().debug("Attached weapon model ", weaponModelId,
        " to instance ", charInstanceId, " at attachment ", attachmentId,
        " (bone ", boneIndex, ", offset ", offset.x, ",", offset.y, ",", offset.z, ")");
    return true;
}

bool CharacterRenderer::getInstanceBounds(uint32_t instanceId, glm::vec3& outCenter, float& outRadius) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return false;
    auto mIt = models.find(it->second.modelId);
    if (mIt == models.end()) return false;

    const auto& inst = it->second;
    const auto& model = mIt->second.data;

    glm::vec3 localCenter = (model.boundMin + model.boundMax) * 0.5f;
    float radius = model.boundRadius;
    if (radius <= 0.001f) {
        radius = glm::length(model.boundMax - model.boundMin) * 0.5f;
    }

    float scale = std::max(0.001f, inst.scale);
    outCenter = inst.position + localCenter * scale;
    outRadius = std::max(0.5f, radius * scale);
    return true;
}

bool CharacterRenderer::getInstanceFootZ(uint32_t instanceId, float& outFootZ) const {
    auto it = instances.find(instanceId);
    if (it == instances.end()) return false;
    auto mIt = models.find(it->second.modelId);
    if (mIt == models.end()) return false;

    const auto& inst = it->second;
    const auto& model = mIt->second.data;
    float scale = std::max(0.001f, inst.scale);
    outFootZ = inst.position.z + model.boundMin.z * scale;
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

bool CharacterRenderer::getAttachmentTransform(uint32_t instanceId, uint32_t attachmentId, glm::mat4& outTransform) {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) return false;
    const auto& instance = instIt->second;

    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) return false;
    const auto& model = modelIt->second.data;

    // Find attachment point
    uint16_t boneIndex = 0;
    glm::vec3 offset(0.0f);
    bool found = false;

    // Try attachment lookup first
    if (attachmentId < model.attachmentLookup.size()) {
        uint16_t attIdx = model.attachmentLookup[attachmentId];
        if (attIdx < model.attachments.size()) {
            boneIndex = model.attachments[attIdx].bone;
            offset = model.attachments[attIdx].position;
            found = true;
        }
    }

    // Fallback: scan attachments by id
    if (!found) {
        for (const auto& att : model.attachments) {
            if (att.id == attachmentId) {
                boneIndex = att.bone;
                offset = att.position;
                found = true;
                break;
            }
        }
    }

    if (!found) return false;

    // Validate bone index; invalid indices bind attachments to origin (looks like weapons at feet).
    if (boneIndex >= model.bones.size()) {
        // Fallback: key bones (26/27) only for hand attachments.
        if (attachmentId == 1 || attachmentId == 2) {
            int32_t targetKeyBone = (attachmentId == 1) ? 26 : 27;
            found = false;
            for (size_t i = 0; i < model.bones.size(); i++) {
                if (model.bones[i].keyBoneId == targetKeyBone) {
                    boneIndex = static_cast<uint16_t>(i);
                    offset = glm::vec3(0.0f);
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        } else {
            return false;
        }
    }

    // Get bone matrix
    glm::mat4 boneMat(1.0f);
    if (boneIndex < instance.boneMatrices.size()) {
        boneMat = instance.boneMatrices[boneIndex];
    }

    // Compute world transform: modelMatrix * boneMatrix * offsetTranslation
    glm::mat4 modelMat = instance.hasOverrideModelMatrix
        ? instance.overrideModelMatrix
        : getModelMatrix(instance);

    outTransform = modelMat * boneMat * glm::translate(glm::mat4(1.0f), offset);
    return true;
}

void CharacterRenderer::dumpAnimations(uint32_t instanceId) const {
    auto instIt = instances.find(instanceId);
    if (instIt == instances.end()) {
        core::Logger::getInstance().info("dumpAnimations: instance ", instanceId, " not found");
        return;
    }
    const auto& instance = instIt->second;

    auto modelIt = models.find(instance.modelId);
    if (modelIt == models.end()) {
        core::Logger::getInstance().info("dumpAnimations: model not found for instance ", instanceId);
        return;
    }
    const auto& model = modelIt->second.data;

    core::Logger::getInstance().info("=== Animation dump for ", model.name, " ===");
    core::Logger::getInstance().info("Total animations: ", model.sequences.size());

    for (size_t i = 0; i < model.sequences.size(); i++) {
        const auto& seq = model.sequences[i];
        core::Logger::getInstance().info("  [", i, "] animId=", seq.id,
            " variation=", seq.variationIndex,
            " duration=", seq.duration, "ms",
            " speed=", seq.movingSpeed,
            " flags=0x", std::hex, seq.flags, std::dec);
    }
    core::Logger::getInstance().info("=== End animation dump ===");
}

} // namespace rendering
} // namespace wowee
