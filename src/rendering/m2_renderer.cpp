#include "rendering/m2_renderer.hpp"
#include "rendering/texture.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <chrono>
#include <cctype>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wowee {
namespace rendering {

namespace {

void getTightCollisionBounds(const M2ModelGPU& model, glm::vec3& outMin, glm::vec3& outMax) {
    glm::vec3 center = (model.boundMin + model.boundMax) * 0.5f;
    glm::vec3 half = (model.boundMax - model.boundMin) * 0.5f;

    // Per-shape collision fitting:
    // - small solid props (boxes/crates/chests): tighter than full mesh, but
    //   larger than default to prevent walk-through on narrow objects
    // - default: tighter fit (avoid oversized blockers)
    // - stepped low platforms (tree curbs/planters): wider XY + lower Z
    if (model.collisionTreeTrunk) {
        // Tree trunk: proportional cylinder at the base of the tree.
        float modelHoriz = std::max(model.boundMax.x - model.boundMin.x,
                                    model.boundMax.y - model.boundMin.y);
        float trunkHalf = std::clamp(modelHoriz * 0.05f, 0.5f, 5.0f);
        half.x = trunkHalf;
        half.y = trunkHalf;
        // Height proportional to trunk width, capped at 3.5 units.
        half.z = std::min(trunkHalf * 2.5f, 3.5f);
        // Shift center down so collision is at the base (trunk), not mid-canopy.
        center.z = model.boundMin.z + half.z;
    } else if (model.collisionNarrowVerticalProp) {
        // Tall thin props (lamps/posts): keep passable gaps near walls.
        half.x *= 0.30f;
        half.y *= 0.30f;
        half.z *= 0.96f;
    } else if (model.collisionSmallSolidProp) {
        // Keep full tight mesh bounds for small solid props to avoid clip-through.
        half.x *= 1.00f;
        half.y *= 1.00f;
        half.z *= 1.00f;
    } else if (model.collisionSteppedLowPlatform) {
        half.x *= 0.98f;
        half.y *= 0.98f;
        half.z *= 0.52f;
    } else {
        half.x *= 0.66f;
        half.y *= 0.66f;
        half.z *= 0.76f;
    }

    outMin = center - half;
    outMax = center + half;
}

float getEffectiveCollisionTopLocal(const M2ModelGPU& model,
                                    const glm::vec3& localPos,
                                    const glm::vec3& localMin,
                                    const glm::vec3& localMax) {
    if (!model.collisionSteppedFountain && !model.collisionSteppedLowPlatform) {
        return localMax.z;
    }

    glm::vec2 center((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
    glm::vec2 half((localMax.x - localMin.x) * 0.5f, (localMax.y - localMin.y) * 0.5f);
    if (half.x < 1e-4f || half.y < 1e-4f) {
        return localMax.z;
    }

    float nx = (localPos.x - center.x) / half.x;
    float ny = (localPos.y - center.y) / half.y;
    float r = std::sqrt(nx * nx + ny * ny);

    float h = localMax.z - localMin.z;
    if (model.collisionSteppedFountain) {
        if (r > 0.85f) return localMin.z + h * 0.18f;  // outer lip
        if (r > 0.65f) return localMin.z + h * 0.36f;  // mid step
        if (r > 0.45f) return localMin.z + h * 0.54f;  // inner step
        if (r > 0.28f) return localMin.z + h * 0.70f;  // center platform / statue base
        if (r > 0.14f) return localMin.z + h * 0.84f;  // statue body / sword
        return localMin.z + h * 0.96f;                  // statue head / top
    }

    // Low square curb/planter profile:
    // use edge distance (not radial) so corner blocks don't become too low and
    // clip-through at diagonals.
    float edge = std::max(std::abs(nx), std::abs(ny));
    if (edge > 0.92f) return localMin.z + h * 0.06f;
    if (edge > 0.72f) return localMin.z + h * 0.30f;
    return localMin.z + h * 0.62f;
}

bool segmentIntersectsAABB(const glm::vec3& from, const glm::vec3& to,
                           const glm::vec3& bmin, const glm::vec3& bmax,
                           float& outEnterT) {
    glm::vec3 d = to - from;
    float tEnter = 0.0f;
    float tExit = 1.0f;

    for (int axis = 0; axis < 3; axis++) {
        if (std::abs(d[axis]) < 1e-6f) {
            if (from[axis] < bmin[axis] || from[axis] > bmax[axis]) {
                return false;
            }
            continue;
        }

        float inv = 1.0f / d[axis];
        float t0 = (bmin[axis] - from[axis]) * inv;
        float t1 = (bmax[axis] - from[axis]) * inv;
        if (t0 > t1) std::swap(t0, t1);

        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) return false;
    }

    outEnterT = tEnter;
    return tExit >= 0.0f && tEnter <= 1.0f;
}

void transformAABB(const glm::mat4& modelMatrix,
                   const glm::vec3& localMin,
                   const glm::vec3& localMax,
                   glm::vec3& outMin,
                   glm::vec3& outMax) {
    const glm::vec3 corners[8] = {
        {localMin.x, localMin.y, localMin.z},
        {localMin.x, localMin.y, localMax.z},
        {localMin.x, localMax.y, localMin.z},
        {localMin.x, localMax.y, localMax.z},
        {localMax.x, localMin.y, localMin.z},
        {localMax.x, localMin.y, localMax.z},
        {localMax.x, localMax.y, localMin.z},
        {localMax.x, localMax.y, localMax.z}
    };

    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(-std::numeric_limits<float>::max());
    for (const auto& c : corners) {
        glm::vec3 wc = glm::vec3(modelMatrix * glm::vec4(c, 1.0f));
        outMin = glm::min(outMin, wc);
        outMax = glm::max(outMax, wc);
    }
}

float pointAABBDistanceSq(const glm::vec3& p, const glm::vec3& bmin, const glm::vec3& bmax) {
    glm::vec3 q = glm::clamp(p, bmin, bmax);
    glm::vec3 d = p - q;
    return glm::dot(d, d);
}

struct QueryTimer {
    double* totalMs = nullptr;
    uint32_t* callCount = nullptr;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    QueryTimer(double* total, uint32_t* calls) : totalMs(total), callCount(calls) {}
    ~QueryTimer() {
        if (callCount) {
            (*callCount)++;
        }
        if (totalMs) {
            auto end = std::chrono::steady_clock::now();
            *totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }
    }
};

} // namespace

void M2Instance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Rotation in radians
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));
    invModelMatrix = glm::inverse(modelMatrix);
}

M2Renderer::M2Renderer() {
}

M2Renderer::~M2Renderer() {
    shutdown();
}

bool M2Renderer::initialize(pipeline::AssetManager* assets) {
    assetManager = assets;

    LOG_INFO("Initializing M2 renderer...");

    // Create M2 shader with skeletal animation support
    const char* vertexSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec2 aTexCoord;
        layout (location = 3) in vec4 aBoneWeights;
        layout (location = 4) in vec4 aBoneIndicesF;

        uniform mat4 uModel;
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform bool uUseBones;
        uniform mat4 uBones[128];
        uniform vec2 uUVOffset;
        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;

        void main() {
            vec3 pos = aPos;
            vec3 norm = aNormal;

            if (uUseBones) {
                ivec4 bi = ivec4(aBoneIndicesF);
                mat4 boneTransform = uBones[bi.x] * aBoneWeights.x
                                   + uBones[bi.y] * aBoneWeights.y
                                   + uBones[bi.z] * aBoneWeights.z
                                   + uBones[bi.w] * aBoneWeights.w;
                pos = vec3(boneTransform * vec4(aPos, 1.0));
                norm = mat3(boneTransform) * aNormal;
            }

            vec4 worldPos = uModel * vec4(pos, 1.0);
            FragPos = worldPos.xyz;
            Normal = mat3(uModel) * norm;
            TexCoord = aTexCoord + uUVOffset;

            gl_Position = uProjection * uView * worldPos;
        }
    )";

    const char* fragmentSrc = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;

        uniform vec3 uLightDir;
        uniform vec3 uLightColor;
        uniform float uSpecularIntensity;
        uniform vec3 uAmbientColor;
        uniform vec3 uViewPos;
        uniform sampler2D uTexture;
        uniform bool uHasTexture;
        uniform bool uAlphaTest;
        uniform float uFadeAlpha;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        uniform sampler2DShadow uShadowMap;
        uniform mat4 uLightSpaceMatrix;
        uniform bool uShadowEnabled;
        uniform float uShadowStrength;

        out vec4 FragColor;

        void main() {
            vec4 texColor;
            if (uHasTexture) {
                texColor = texture(uTexture, TexCoord);
            } else {
                texColor = vec4(0.6, 0.5, 0.4, 1.0);  // Fallback brownish
            }

            // Alpha test for leaves, fences, etc.
            if (uAlphaTest && texColor.a < 0.5) {
                discard;
            }

            // Distance fade - discard nearly invisible fragments
            float finalAlpha = texColor.a * uFadeAlpha;
            if (finalAlpha < 0.02) {
                discard;
            }

            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            // Two-sided lighting for foliage
            float diff = max(abs(dot(normal, lightDir)), 0.3);

            // Blinn-Phong specular
            vec3 viewDir = normalize(uViewPos - FragPos);
            vec3 halfDir = normalize(lightDir + viewDir);
            float spec = pow(max(dot(normal, halfDir), 0.0), 32.0);
            vec3 specular = spec * uLightColor * uSpecularIntensity;

            // Shadow mapping
            float shadow = 1.0;
            if (uShadowEnabled) {
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

            vec3 ambient = uAmbientColor * texColor.rgb;
            vec3 diffuse = diff * texColor.rgb;

            vec3 result = ambient + (diffuse + specular) * shadow;

            // Fog
            float fogDist = length(uViewPos - FragPos);
            float fogFactor = clamp((uFogEnd - fogDist) / (uFogEnd - uFogStart), 0.0, 1.0);
            result = mix(uFogColor, result, fogFactor);

            FragColor = vec4(result, finalAlpha);
        }
    )";

    shader = std::make_unique<Shader>();
    if (!shader->loadFromSource(vertexSrc, fragmentSrc)) {
        LOG_ERROR("Failed to create M2 shader");
        return false;
    }

    // Create smoke particle shader
    const char* smokeVertSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in float aLifeRatio;
        layout (location = 2) in float aSize;
        layout (location = 3) in float aIsSpark;

        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform float uScreenHeight;

        out float vLifeRatio;
        out float vIsSpark;

        void main() {
            vec4 viewPos = uView * vec4(aPos, 1.0);
            gl_Position = uProjection * viewPos;
            float dist = -viewPos.z;
            float scale = (aIsSpark > 0.5) ? 0.12 : 0.3;
            gl_PointSize = clamp(aSize * (uScreenHeight * scale) / max(dist, 1.0), 2.0, 200.0);
            vLifeRatio = aLifeRatio;
            vIsSpark = aIsSpark;
        }
    )";

    const char* smokeFragSrc = R"(
        #version 330 core
        in float vLifeRatio;
        in float vIsSpark;
        out vec4 FragColor;

        void main() {
            vec2 coord = gl_PointCoord - vec2(0.5);
            float dist = length(coord) * 2.0;

            if (vIsSpark > 0.5) {
                // Ember/spark: bright hot dot, fades quickly
                float circle = 1.0 - smoothstep(0.3, 0.8, dist);
                float fade = 1.0 - smoothstep(0.0, 1.0, vLifeRatio);
                float alpha = circle * fade;
                vec3 color = mix(vec3(1.0, 0.6, 0.1), vec3(1.0, 0.2, 0.0), vLifeRatio);
                FragColor = vec4(color, alpha);
            } else {
                // Smoke: soft gray circle
                float circle = 1.0 - smoothstep(0.5, 1.0, dist);
                float fadeIn = smoothstep(0.0, 0.1, vLifeRatio);
                float fadeOut = 1.0 - smoothstep(0.4, 1.0, vLifeRatio);
                float alpha = circle * fadeIn * fadeOut * 0.5;
                vec3 color = mix(vec3(0.5, 0.5, 0.53), vec3(0.65, 0.65, 0.68), vLifeRatio);
                FragColor = vec4(color, alpha);
            }
        }
    )";

    smokeShader = std::make_unique<Shader>();
    if (!smokeShader->loadFromSource(smokeVertSrc, smokeFragSrc)) {
        LOG_ERROR("Failed to create smoke particle shader (non-fatal)");
        smokeShader.reset();
    }

    // Create smoke particle VAO/VBO (only if shader compiled)
    if (smokeShader) {
        glGenVertexArrays(1, &smokeVAO);
        glGenBuffers(1, &smokeVBO);
        glBindVertexArray(smokeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, smokeVBO);
        // 5 floats per particle: pos(3) + lifeRatio(1) + size(1)
        // 6 floats per particle: pos(3) + lifeRatio(1) + size(1) + isSpark(1)
        glBufferData(GL_ARRAY_BUFFER, MAX_SMOKE_PARTICLES * 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        // Life ratio
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        // Size
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(4 * sizeof(float)));
        // IsSpark
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(5 * sizeof(float)));
        glBindVertexArray(0);
    }

    // Create white fallback texture
    uint8_t white[] = {255, 255, 255, 255};
    glGenTextures(1, &whiteTexture);
    glBindTexture(GL_TEXTURE_2D, whiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    LOG_INFO("M2 renderer initialized");
    return true;
}

void M2Renderer::shutdown() {
    LOG_INFO("Shutting down M2 renderer...");

    // Delete GPU resources
    for (auto& [id, model] : models) {
        if (model.vao != 0) glDeleteVertexArrays(1, &model.vao);
        if (model.vbo != 0) glDeleteBuffers(1, &model.vbo);
        if (model.ebo != 0) glDeleteBuffers(1, &model.ebo);
    }
    models.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();

    // Delete cached textures
    for (auto& [path, texId] : textureCache) {
        if (texId != 0 && texId != whiteTexture) {
            glDeleteTextures(1, &texId);
        }
    }
    textureCache.clear();
    if (whiteTexture != 0) {
        glDeleteTextures(1, &whiteTexture);
        whiteTexture = 0;
    }

    shader.reset();

    // Clean up smoke particle resources
    if (smokeVAO != 0) { glDeleteVertexArrays(1, &smokeVAO); smokeVAO = 0; }
    if (smokeVBO != 0) { glDeleteBuffers(1, &smokeVBO); smokeVBO = 0; }
    smokeShader.reset();
    smokeParticles.clear();
}

bool M2Renderer::loadModel(const pipeline::M2Model& model, uint32_t modelId) {
    if (models.find(modelId) != models.end()) {
        // Already loaded
        return true;
    }

    if (model.vertices.empty() || model.indices.empty()) {
        LOG_WARNING("M2 model has no geometry: ", model.name);
        return false;
    }

    M2ModelGPU gpuModel;
    gpuModel.name = model.name;
    // Use tight bounds from actual vertices for collision/camera occlusion.
    // Header bounds in some M2s are overly conservative.
    glm::vec3 tightMin( std::numeric_limits<float>::max());
    glm::vec3 tightMax(-std::numeric_limits<float>::max());
    for (const auto& v : model.vertices) {
        tightMin = glm::min(tightMin, v.position);
        tightMax = glm::max(tightMax, v.position);
    }
    bool foliageOrTreeLike = false;
    {
        std::string lowerName = model.name;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.collisionSteppedFountain = (lowerName.find("fountain") != std::string::npos);

        glm::vec3 dims = tightMax - tightMin;
        float horiz = std::max(dims.x, dims.y);
        float vert = std::max(0.0f, dims.z);
        bool lowWideShape = (horiz > 1.4f && vert > 0.2f && vert < horiz * 0.70f);
        bool likelyCurbName =
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos);
        bool knownStormwindPlanter =
            (lowerName.find("stormwindplanter") != std::string::npos) ||
            (lowerName.find("stormwindwindowplanter") != std::string::npos);
        bool lowPlatformShape = (horiz > 1.8f && vert > 0.2f && vert < 1.8f);
        gpuModel.collisionSteppedLowPlatform = (!gpuModel.collisionSteppedFountain) &&
                                               (knownStormwindPlanter ||
                                                (likelyCurbName && (lowPlatformShape || lowWideShape)));

        bool isPlanter = (lowerName.find("planter") != std::string::npos);
        gpuModel.collisionPlanter = isPlanter;
        bool statueName =
            (lowerName.find("statue") != std::string::npos) ||
            (lowerName.find("monument") != std::string::npos) ||
            (lowerName.find("sculpture") != std::string::npos);
        gpuModel.collisionStatue = statueName;
        bool smallSolidPropName =
            statueName ||
            (lowerName.find("crate") != std::string::npos) ||
            (lowerName.find("box") != std::string::npos) ||
            (lowerName.find("chest") != std::string::npos) ||
            (lowerName.find("barrel") != std::string::npos);
        bool foliageName =
            (lowerName.find("bush") != std::string::npos) ||
            (lowerName.find("grass") != std::string::npos) ||
            ((lowerName.find("plant") != std::string::npos) && !isPlanter) ||
            (lowerName.find("flower") != std::string::npos) ||
            (lowerName.find("shrub") != std::string::npos) ||
            (lowerName.find("fern") != std::string::npos) ||
            (lowerName.find("vine") != std::string::npos) ||
            (lowerName.find("lily") != std::string::npos) ||
            (lowerName.find("weed") != std::string::npos);
        bool treeLike = (lowerName.find("tree") != std::string::npos);
        foliageOrTreeLike = (foliageName || treeLike);
        bool hardTreePart =
            (lowerName.find("trunk") != std::string::npos) ||
            (lowerName.find("stump") != std::string::npos) ||
            (lowerName.find("log") != std::string::npos);
        // Only large trees (canopy > 20 model units wide) get trunk collision.
        // Small/mid trees are walkthrough to avoid getting stuck between them.
        // Only large trees get trunk collision; all smaller trees are walkthrough.
        bool treeWithTrunk = treeLike && !hardTreePart && !foliageName && horiz > 40.0f;
        bool softTree = treeLike && !hardTreePart && !treeWithTrunk;
        bool forceSolidCurb = gpuModel.collisionSteppedLowPlatform || knownStormwindPlanter || likelyCurbName || gpuModel.collisionPlanter;
        bool narrowVerticalName =
            (lowerName.find("lamp") != std::string::npos) ||
            (lowerName.find("lantern") != std::string::npos) ||
            (lowerName.find("post") != std::string::npos) ||
            (lowerName.find("pole") != std::string::npos);
        bool narrowVerticalShape =
            (horiz > 0.12f && horiz < 2.0f && vert > 2.2f && vert > horiz * 1.8f);
        gpuModel.collisionTreeTrunk = treeWithTrunk;
        gpuModel.collisionNarrowVerticalProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            (narrowVerticalName || narrowVerticalShape);
        bool genericSolidPropShape =
            (horiz > 0.6f && horiz < 6.0f && vert > 0.30f && vert < 4.0f && vert > horiz * 0.16f) ||
            statueName;
        bool curbLikeName =
            (lowerName.find("curb") != std::string::npos) ||
            (lowerName.find("planter") != std::string::npos) ||
            (lowerName.find("ring") != std::string::npos) ||
            (lowerName.find("well") != std::string::npos) ||
            (lowerName.find("base") != std::string::npos);
        bool lowPlatformLikeShape = lowWideShape || lowPlatformShape;
        gpuModel.collisionSmallSolidProp =
            !gpuModel.collisionSteppedFountain &&
            !gpuModel.collisionSteppedLowPlatform &&
            !gpuModel.collisionNarrowVerticalProp &&
            !gpuModel.collisionTreeTrunk &&
            !curbLikeName &&
            !lowPlatformLikeShape &&
            (smallSolidPropName || (genericSolidPropShape && !foliageName && !softTree));
        gpuModel.collisionNoBlock = ((foliageName || softTree) &&
                                     !forceSolidCurb);
    }
    gpuModel.boundMin = tightMin;
    gpuModel.boundMax = tightMax;
    gpuModel.boundRadius = model.boundRadius;
    gpuModel.indexCount = static_cast<uint32_t>(model.indices.size());
    gpuModel.vertexCount = static_cast<uint32_t>(model.vertices.size());

    // Create VAO
    glGenVertexArrays(1, &gpuModel.vao);
    glBindVertexArray(gpuModel.vao);

    // Store bone/sequence data for animation
    gpuModel.bones = model.bones;
    gpuModel.sequences = model.sequences;
    gpuModel.globalSequenceDurations = model.globalSequenceDurations;
    gpuModel.hasAnimation = false;
    for (const auto& bone : model.bones) {
        if (bone.translation.hasData() || bone.rotation.hasData() || bone.scale.hasData()) {
            gpuModel.hasAnimation = true;
            break;
        }
    }
    gpuModel.disableAnimation = foliageOrTreeLike;

    // Flag smoke models for UV scroll animation (particle emitters not implemented)
    {
        std::string smokeName = model.name;
        std::transform(smokeName.begin(), smokeName.end(), smokeName.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        gpuModel.isSmoke = (smokeName.find("smoke") != std::string::npos);
    }

    // Identify idle variation sequences (animation ID 0 = Stand)
    for (int i = 0; i < static_cast<int>(model.sequences.size()); i++) {
        if (model.sequences[i].id == 0 && model.sequences[i].duration > 0) {
            gpuModel.idleVariationIndices.push_back(i);
        }
    }

    // Create VBO with interleaved vertex data
    // Format: position (3), normal (3), texcoord (2), boneWeights (4), boneIndices (4 as float)
    const size_t floatsPerVertex = 16;
    std::vector<float> vertexData;
    vertexData.reserve(model.vertices.size() * floatsPerVertex);

    for (const auto& v : model.vertices) {
        vertexData.push_back(v.position.x);
        vertexData.push_back(v.position.y);
        vertexData.push_back(v.position.z);
        vertexData.push_back(v.normal.x);
        vertexData.push_back(v.normal.y);
        vertexData.push_back(v.normal.z);
        vertexData.push_back(v.texCoords[0].x);
        vertexData.push_back(v.texCoords[0].y);
        // Bone weights (normalized 0-1)
        float w0 = v.boneWeights[0] / 255.0f;
        float w1 = v.boneWeights[1] / 255.0f;
        float w2 = v.boneWeights[2] / 255.0f;
        float w3 = v.boneWeights[3] / 255.0f;
        vertexData.push_back(w0);
        vertexData.push_back(w1);
        vertexData.push_back(w2);
        vertexData.push_back(w3);
        // Bone indices (clamped to max 127 for uniform array)
        vertexData.push_back(static_cast<float>(std::min(v.boneIndices[0], uint8_t(127))));
        vertexData.push_back(static_cast<float>(std::min(v.boneIndices[1], uint8_t(127))));
        vertexData.push_back(static_cast<float>(std::min(v.boneIndices[2], uint8_t(127))));
        vertexData.push_back(static_cast<float>(std::min(v.boneIndices[3], uint8_t(127))));
    }

    glGenBuffers(1, &gpuModel.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, gpuModel.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float),
                 vertexData.data(), GL_STATIC_DRAW);

    // Create EBO
    glGenBuffers(1, &gpuModel.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuModel.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.indices.size() * sizeof(uint16_t),
                 model.indices.data(), GL_STATIC_DRAW);

    // Set up vertex attributes
    const size_t stride = floatsPerVertex * sizeof(float);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));

    // TexCoord
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));

    // Bone Weights
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, (void*)(8 * sizeof(float)));

    // Bone Indices (as integer attribute)
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride, (void*)(12 * sizeof(float)));

    glBindVertexArray(0);

    // Load ALL textures from the model into a local vector
    std::vector<GLuint> allTextures;
    if (assetManager) {
        for (const auto& tex : model.textures) {
            if (!tex.filename.empty()) {
                allTextures.push_back(loadTexture(tex.filename));
            } else {
                allTextures.push_back(whiteTexture);
            }
        }
    }

    // Copy texture transform data for UV animation
    gpuModel.textureTransforms = model.textureTransforms;
    gpuModel.textureTransformLookup = model.textureTransformLookup;
    gpuModel.hasTextureAnimation = false;

    // Build per-batch GPU entries
    if (!model.batches.empty()) {
        for (const auto& batch : model.batches) {
            M2ModelGPU::BatchGPU bgpu;
            bgpu.indexStart = batch.indexStart;
            bgpu.indexCount = batch.indexCount;

            // Store texture animation index from batch
            bgpu.textureAnimIndex = batch.textureAnimIndex;
            if (bgpu.textureAnimIndex != 0xFFFF) {
                gpuModel.hasTextureAnimation = true;
            }

            // Resolve texture: batch.textureIndex → textureLookup → allTextures
            GLuint tex = whiteTexture;
            if (batch.textureIndex < model.textureLookup.size()) {
                uint16_t texIdx = model.textureLookup[batch.textureIndex];
                if (texIdx < allTextures.size()) {
                    tex = allTextures[texIdx];
                }
            } else if (!allTextures.empty()) {
                tex = allTextures[0];
            }
            bgpu.texture = tex;
            bgpu.hasAlpha = (tex != 0 && tex != whiteTexture);
            gpuModel.batches.push_back(bgpu);
        }
    } else {
        // Fallback: single batch covering all indices with first texture
        M2ModelGPU::BatchGPU bgpu;
        bgpu.indexStart = 0;
        bgpu.indexCount = gpuModel.indexCount;
        bgpu.texture = allTextures.empty() ? whiteTexture : allTextures[0];
        bgpu.hasAlpha = (bgpu.texture != 0 && bgpu.texture != whiteTexture);
        gpuModel.batches.push_back(bgpu);
    }

    models[modelId] = std::move(gpuModel);

    LOG_DEBUG("Loaded M2 model: ", model.name, " (", models[modelId].vertexCount, " vertices, ",
              models[modelId].indexCount / 3, " triangles, ", models[modelId].batches.size(), " batches)");

    return true;
}

uint32_t M2Renderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    // Deduplicate: skip if same model already at nearly the same position
    for (const auto& existing : instances) {
        if (existing.modelId == modelId) {
            glm::vec3 d = existing.position - position;
            if (glm::dot(d, d) < 0.01f) {
                return existing.id;
            }
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);

    // Initialize animation: play first sequence (usually Stand/Idle)
    const auto& mdl = models[modelId];
    if (mdl.hasAnimation && !mdl.disableAnimation && !mdl.sequences.empty()) {
        instance.currentSequenceIndex = 0;
        instance.idleSequenceIndex = 0;
        instance.animDuration = static_cast<float>(mdl.sequences[0].duration);
        instance.animTime = static_cast<float>(rand() % std::max(1u, mdl.sequences[0].duration));
        instance.variationTimer = 3000.0f + static_cast<float>(rand() % 8000);
    }

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

uint32_t M2Renderer::createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                                const glm::vec3& position) {
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }

    // Deduplicate: skip if same model already at nearly the same position
    for (const auto& existing : instances) {
        if (existing.modelId == modelId) {
            glm::vec3 d = existing.position - position;
            if (glm::dot(d, d) < 0.01f) {
                return existing.id;
            }
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;  // Used for frustum culling
    instance.rotation = glm::vec3(0.0f);
    instance.scale = 1.0f;
    instance.modelMatrix = modelMatrix;
    instance.invModelMatrix = glm::inverse(modelMatrix);
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);
    // Initialize animation
    const auto& mdl2 = models[modelId];
    if (mdl2.hasAnimation && !mdl2.disableAnimation && !mdl2.sequences.empty()) {
        instance.currentSequenceIndex = 0;
        instance.idleSequenceIndex = 0;
        instance.animDuration = static_cast<float>(mdl2.sequences[0].duration);
        instance.animTime = static_cast<float>(rand() % std::max(1u, mdl2.sequences[0].duration));
        instance.variationTimer = 3000.0f + static_cast<float>(rand() % 8000);
    } else {
        instance.animTime = static_cast<float>(rand()) / RAND_MAX * 10000.0f;
    }

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

// --- Bone animation helpers (same logic as CharacterRenderer) ---

static int findKeyframeIndex(const std::vector<uint32_t>& timestamps, float time) {
    if (timestamps.empty()) return -1;
    if (timestamps.size() == 1) return 0;
    for (size_t i = 0; i < timestamps.size() - 1; i++) {
        if (time < static_cast<float>(timestamps[i + 1])) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(timestamps.size() - 2);
}

// Resolve sequence index and time for a track, handling global sequences.
static void resolveTrackTime(const pipeline::M2AnimationTrack& track,
                              int seqIdx, float time,
                              const std::vector<uint32_t>& globalSeqDurations,
                              int& outSeqIdx, float& outTime) {
    if (track.globalSequence >= 0 &&
        static_cast<size_t>(track.globalSequence) < globalSeqDurations.size()) {
        // Global sequence: always use sub-array 0, wrap time at global duration
        outSeqIdx = 0;
        float dur = static_cast<float>(globalSeqDurations[track.globalSequence]);
        outTime = (dur > 0.0f) ? std::fmod(time, dur) : 0.0f;
    } else {
        outSeqIdx = seqIdx;
        outTime = time;
    }
}

static glm::vec3 interpVec3(const pipeline::M2AnimationTrack& track,
                             int seqIdx, float time, const glm::vec3& def,
                             const std::vector<uint32_t>& globalSeqDurations) {
    if (!track.hasData()) return def;
    int si; float t;
    resolveTrackTime(track, seqIdx, time, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return def;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.vec3Values.empty()) return def;
    auto safe = [&](const glm::vec3& v) -> glm::vec3 {
        if (std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z)) return def;
        return v;
    };
    if (keys.vec3Values.size() == 1) return safe(keys.vec3Values[0]);
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return def;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.vec3Values.size() - 1);
    if (i0 == i1) return safe(keys.vec3Values[i0]);
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return safe(glm::mix(keys.vec3Values[i0], keys.vec3Values[i1], frac));
}

static glm::quat interpQuat(const pipeline::M2AnimationTrack& track,
                              int seqIdx, float time,
                              const std::vector<uint32_t>& globalSeqDurations) {
    glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    if (!track.hasData()) return identity;
    int si; float t;
    resolveTrackTime(track, seqIdx, time, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return identity;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.quatValues.empty()) return identity;
    auto safe = [&](const glm::quat& q) -> glm::quat {
        float len = glm::length(q);
        if (len < 0.001f || std::isnan(len)) return identity;
        return q;
    };
    if (keys.quatValues.size() == 1) return safe(keys.quatValues[0]);
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return identity;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.quatValues.size() - 1);
    if (i0 == i1) return safe(keys.quatValues[i0]);
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return glm::slerp(safe(keys.quatValues[i0]), safe(keys.quatValues[i1]), frac);
}

static void computeBoneMatrices(const M2ModelGPU& model, M2Instance& instance) {
    size_t numBones = std::min(model.bones.size(), size_t(128));
    if (numBones == 0) return;
    instance.boneMatrices.resize(numBones);
    const auto& gsd = model.globalSequenceDurations;

    for (size_t i = 0; i < numBones; i++) {
        const auto& bone = model.bones[i];
        glm::vec3 trans = interpVec3(bone.translation, instance.currentSequenceIndex, instance.animTime, glm::vec3(0.0f), gsd);
        glm::quat rot = interpQuat(bone.rotation, instance.currentSequenceIndex, instance.animTime, gsd);
        glm::vec3 scl = interpVec3(bone.scale, instance.currentSequenceIndex, instance.animTime, glm::vec3(1.0f), gsd);

        // Sanity check scale to avoid degenerate matrices
        if (scl.x < 0.001f) scl.x = 1.0f;
        if (scl.y < 0.001f) scl.y = 1.0f;
        if (scl.z < 0.001f) scl.z = 1.0f;

        glm::mat4 local = glm::translate(glm::mat4(1.0f), bone.pivot);
        local = glm::translate(local, trans);
        local *= glm::toMat4(rot);
        local = glm::scale(local, scl);
        local = glm::translate(local, -bone.pivot);

        if (bone.parentBone >= 0 && static_cast<size_t>(bone.parentBone) < numBones) {
            instance.boneMatrices[i] = instance.boneMatrices[bone.parentBone] * local;
        } else {
            instance.boneMatrices[i] = local;
        }
    }
}

void M2Renderer::update(float deltaTime) {
    float dtMs = deltaTime * 1000.0f;

    // --- Smoke particle spawning ---
    std::uniform_real_distribution<float> distXY(-0.4f, 0.4f);
    std::uniform_real_distribution<float> distVelXY(-0.3f, 0.3f);
    std::uniform_real_distribution<float> distVelZ(3.0f, 5.0f);
    std::uniform_real_distribution<float> distLife(4.0f, 7.0f);
    std::uniform_real_distribution<float> distDrift(-0.2f, 0.2f);

    smokeEmitAccum += deltaTime;
    float emitInterval = 1.0f / 8.0f;  // 8 particles per second per emitter

    for (auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;

        if (model.isSmoke && smokeEmitAccum >= emitInterval &&
            static_cast<int>(smokeParticles.size()) < MAX_SMOKE_PARTICLES) {
            // Emission point: model origin in world space (model matrix already positions at chimney)
            glm::vec3 emitWorld = glm::vec3(instance.modelMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

            // Occasionally spawn a spark instead of smoke (~1 in 8)
            bool spark = (smokeRng() % 8 == 0);

            SmokeParticle p;
            p.position = emitWorld + glm::vec3(distXY(smokeRng), distXY(smokeRng), 0.0f);
            if (spark) {
                p.velocity = glm::vec3(distVelXY(smokeRng) * 2.0f, distVelXY(smokeRng) * 2.0f, distVelZ(smokeRng) * 1.5f);
                p.maxLife = 0.8f + static_cast<float>(smokeRng() % 100) / 100.0f * 1.2f;  // 0.8-2.0s
                p.size = 0.5f;
                p.isSpark = 1.0f;
            } else {
                p.velocity = glm::vec3(distVelXY(smokeRng), distVelXY(smokeRng), distVelZ(smokeRng));
                p.maxLife = distLife(smokeRng);
                p.size = 1.0f;
                p.isSpark = 0.0f;
            }
            p.life = 0.0f;
            p.instanceId = instance.id;
            smokeParticles.push_back(p);
        }
    }

    if (smokeEmitAccum >= emitInterval) {
        smokeEmitAccum = 0.0f;
    }

    // --- Update existing smoke particles ---
    for (auto it = smokeParticles.begin(); it != smokeParticles.end(); ) {
        it->life += deltaTime;
        if (it->life >= it->maxLife) {
            it = smokeParticles.erase(it);
            continue;
        }
        it->position += it->velocity * deltaTime;
        it->velocity.z *= 0.98f;  // Slight deceleration
        it->velocity.x += distDrift(smokeRng) * deltaTime;
        it->velocity.y += distDrift(smokeRng) * deltaTime;
        // Grow from 1.0 to 3.5 over lifetime
        float t = it->life / it->maxLife;
        it->size = 1.0f + t * 2.5f;
        ++it;
    }

    // --- Normal M2 animation update ---
    for (auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;

        if (!model.hasAnimation || model.disableAnimation) {
            instance.animTime += dtMs;
            continue;
        }

        instance.animTime += dtMs * instance.animSpeed;

        // Validate sequence index
        if (instance.currentSequenceIndex < 0 ||
            instance.currentSequenceIndex >= static_cast<int>(model.sequences.size())) {
            instance.currentSequenceIndex = 0;
            if (!model.sequences.empty()) {
                instance.animDuration = static_cast<float>(model.sequences[0].duration);
            }
        }

        // Handle animation looping / variation transitions
        if (instance.animDuration > 0.0f && instance.animTime >= instance.animDuration) {
            if (instance.playingVariation) {
                // Variation finished — return to idle
                instance.playingVariation = false;
                instance.currentSequenceIndex = instance.idleSequenceIndex;
                if (instance.idleSequenceIndex < static_cast<int>(model.sequences.size())) {
                    instance.animDuration = static_cast<float>(model.sequences[instance.idleSequenceIndex].duration);
                }
                instance.animTime = 0.0f;
                instance.variationTimer = 4000.0f + static_cast<float>(rand() % 6000);
            } else {
                // Loop idle
                instance.animTime = std::fmod(instance.animTime, std::max(1.0f, instance.animDuration));
            }
        }

        // Idle variation timer — occasionally play a different idle sequence
        if (!instance.playingVariation && model.idleVariationIndices.size() > 1) {
            instance.variationTimer -= dtMs;
            if (instance.variationTimer <= 0.0f) {
                int pick = rand() % static_cast<int>(model.idleVariationIndices.size());
                int newSeq = model.idleVariationIndices[pick];
                if (newSeq != instance.currentSequenceIndex && newSeq < static_cast<int>(model.sequences.size())) {
                    instance.playingVariation = true;
                    instance.currentSequenceIndex = newSeq;
                    instance.animDuration = static_cast<float>(model.sequences[newSeq].duration);
                    instance.animTime = 0.0f;
                } else {
                    instance.variationTimer = 2000.0f + static_cast<float>(rand() % 4000);
                }
            }
        }

        computeBoneMatrices(model, instance);
    }
}

void M2Renderer::render(const Camera& camera, const glm::mat4& view, const glm::mat4& projection) {
    if (instances.empty() || !shader) {
        return;
    }

    // Debug: log once when we start rendering
    static bool loggedOnce = false;
    if (!loggedOnce) {
        loggedOnce = true;
        LOG_INFO("M2 render: ", instances.size(), " instances, ", models.size(), " models");
    }

    // Set up GL state for M2 rendering
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);   // Some M2 geometry is single-sided

    // Build frustum for culling
    Frustum frustum;
    frustum.extractFromMatrix(projection * view);

    shader->use();
    shader->setUniform("uView", view);
    shader->setUniform("uProjection", projection);
    shader->setUniform("uLightDir", lightDir);
    shader->setUniform("uLightColor", glm::vec3(1.5f, 1.4f, 1.3f));
    shader->setUniform("uSpecularIntensity", 0.5f);
    shader->setUniform("uAmbientColor", ambientColor);
    shader->setUniform("uViewPos", camera.getPosition());
    shader->setUniform("uFogColor", fogColor);
    shader->setUniform("uFogStart", fogStart);
    shader->setUniform("uFogEnd", fogEnd);
    shader->setUniform("uShadowEnabled", shadowEnabled ? 1 : 0);
    shader->setUniform("uShadowStrength", 0.65f);
    if (shadowEnabled) {
        shader->setUniform("uLightSpaceMatrix", lightSpaceMatrix);
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, shadowDepthTex);
        shader->setUniform("uShadowMap", 7);
    }

    lastDrawCallCount = 0;

    // Adaptive render distance: keep longer tree/foliage visibility to reduce pop-in.
    const float maxRenderDistance = (instances.size() > 600) ? 320.0f : 2800.0f;
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;
    const float fadeStartFraction = 0.75f;
    const glm::vec3 camPos = camera.getPosition();

    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (!model.isValid()) continue;

        // Skip smoke models — replaced by particle emitters
        if (model.isSmoke) continue;

        // Distance culling for small objects (scaled by object size)
        glm::vec3 toCam = instance.position - camPos;
        float distSq = glm::dot(toCam, toCam);
        float worldRadius = model.boundRadius * instance.scale;
        float cullRadius = worldRadius;
        if (model.disableAnimation) {
            // Many bushes/foliage M2s have conservative tiny bounds; pad to reduce pop-in.
            cullRadius = std::max(cullRadius, 3.0f);
        }
        // Cull small objects (radius < 20) at distance, keep larger objects visible longer
        float effectiveMaxDistSq = maxRenderDistanceSq * std::max(1.0f, cullRadius / 12.0f);
        if (model.disableAnimation) {
            // Trees/foliage keep a much larger horizon before culling.
            effectiveMaxDistSq *= 2.6f;
        }
        if (!model.disableAnimation) {
            if (worldRadius < 0.8f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 95.0f * 95.0f);
            } else if (worldRadius < 1.5f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 140.0f * 140.0f);
            }
        }
        if (distSq > effectiveMaxDistSq) {
            continue;
        }

        // Frustum cull: test bounding sphere in world space
        if (cullRadius > 0.0f && !frustum.intersectsSphere(instance.position, cullRadius)) {
            continue;
        }

        // Distance-based fade alpha for smooth pop-in
        float fadeAlpha = 1.0f;
        float fadeFrac = model.disableAnimation ? 0.55f : fadeStartFraction;
        float fadeStartDistSq = effectiveMaxDistSq * fadeFrac * fadeFrac;
        if (distSq > fadeStartDistSq) {
            float dist = std::sqrt(distSq);
            float effectiveMaxDist = std::sqrt(effectiveMaxDistSq);
            float fadeStartDist = effectiveMaxDist * fadeFrac;
            fadeAlpha = std::clamp((effectiveMaxDist - dist) / (effectiveMaxDist - fadeStartDist), 0.0f, 1.0f);
        }

        shader->setUniform("uModel", instance.modelMatrix);
        shader->setUniform("uFadeAlpha", fadeAlpha);

        // Upload bone matrices if model has skeletal animation
        bool useBones = model.hasAnimation && !model.disableAnimation && !instance.boneMatrices.empty();
        shader->setUniform("uUseBones", useBones);
        if (useBones) {
            int numBones = std::min(static_cast<int>(instance.boneMatrices.size()), 128);
            shader->setUniformMatrixArray("uBones[0]", instance.boneMatrices.data(), numBones);
        }

        // Disable depth writes for fading objects to avoid z-fighting
        if (fadeAlpha < 1.0f) {
            glDepthMask(GL_FALSE);
        }

        glBindVertexArray(model.vao);

        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;

            // Compute UV offset for texture animation
            glm::vec2 uvOffset(0.0f, 0.0f);
            if (batch.textureAnimIndex != 0xFFFF && model.hasTextureAnimation) {
                uint16_t lookupIdx = batch.textureAnimIndex;
                if (lookupIdx < model.textureTransformLookup.size()) {
                    uint16_t transformIdx = model.textureTransformLookup[lookupIdx];
                    if (transformIdx < model.textureTransforms.size()) {
                        const auto& tt = model.textureTransforms[transformIdx];
                        glm::vec3 trans = interpVec3(tt.translation,
                            instance.currentSequenceIndex, instance.animTime,
                            glm::vec3(0.0f), model.globalSequenceDurations);
                        uvOffset = glm::vec2(trans.x, trans.y);
                    }
                }
            }
            shader->setUniform("uUVOffset", uvOffset);

            bool hasTexture = (batch.texture != 0);
            shader->setUniform("uHasTexture", hasTexture);
            shader->setUniform("uAlphaTest", batch.hasAlpha);

            if (hasTexture) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, batch.texture);
                shader->setUniform("uTexture", 0);
            }

            glDrawElements(GL_TRIANGLES, batch.indexCount, GL_UNSIGNED_SHORT,
                           (void*)(batch.indexStart * sizeof(uint16_t)));

            lastDrawCallCount++;
        }

        glBindVertexArray(0);

        if (fadeAlpha < 1.0f) {
            glDepthMask(GL_TRUE);
        }
    }

    // Restore state
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void M2Renderer::renderShadow(GLuint shadowShaderProgram) {
    if (instances.empty() || shadowShaderProgram == 0) {
        return;
    }

    GLint modelLoc = glGetUniformLocation(shadowShaderProgram, "uModel");
    GLint useTexLoc = glGetUniformLocation(shadowShaderProgram, "uUseTexture");
    GLint texLoc = glGetUniformLocation(shadowShaderProgram, "uTexture");
    GLint alphaTestLoc = glGetUniformLocation(shadowShaderProgram, "uAlphaTest");
    GLint opacityLoc = glGetUniformLocation(shadowShaderProgram, "uShadowOpacity");
    if (modelLoc < 0) {
        return;
    }

    if (useTexLoc >= 0) glUniform1i(useTexLoc, 0);
    if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, 0);
    if (opacityLoc >= 0) glUniform1f(opacityLoc, 1.0f);
    if (texLoc >= 0) glUniform1i(texLoc, 0);
    glActiveTexture(GL_TEXTURE0);

    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (!model.isValid() || model.isSmoke) continue;

        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &instance.modelMatrix[0][0]);
        glBindVertexArray(model.vao);

        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;
            bool useTexture = (batch.texture != 0);
            bool alphaCutout = batch.hasAlpha;

            // Foliage/leaf cutout batches cast softer shadows than opaque trunk geometry.
            float shadowOpacity = alphaCutout ? 0.55f : 1.0f;

            if (useTexLoc >= 0) glUniform1i(useTexLoc, useTexture ? 1 : 0);
            if (alphaTestLoc >= 0) glUniform1i(alphaTestLoc, alphaCutout ? 1 : 0);
            if (opacityLoc >= 0) glUniform1f(opacityLoc, shadowOpacity);
            if (useTexture) {
                glBindTexture(GL_TEXTURE_2D, batch.texture);
            }
            glDrawElements(GL_TRIANGLES, batch.indexCount, GL_UNSIGNED_SHORT,
                           (void*)(batch.indexStart * sizeof(uint16_t)));
        }
    }

    glBindVertexArray(0);
}

void M2Renderer::renderSmokeParticles(const Camera& /*camera*/, const glm::mat4& view, const glm::mat4& projection) {
    if (smokeParticles.empty() || !smokeShader || smokeVAO == 0) return;

    // Build vertex data: pos(3) + lifeRatio(1) + size(1) + isSpark(1) per particle
    std::vector<float> data;
    data.reserve(smokeParticles.size() * 6);
    for (const auto& p : smokeParticles) {
        data.push_back(p.position.x);
        data.push_back(p.position.y);
        data.push_back(p.position.z);
        data.push_back(p.life / p.maxLife);
        data.push_back(p.size);
        data.push_back(p.isSpark);
    }

    // Upload to VBO
    glBindBuffer(GL_ARRAY_BUFFER, smokeVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, data.size() * sizeof(float), data.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Set GL state
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);  // Occlude behind buildings
    glDepthMask(GL_FALSE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_CULL_FACE);

    smokeShader->use();
    smokeShader->setUniform("uView", view);
    smokeShader->setUniform("uProjection", projection);

    // Get viewport height for point size scaling
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    smokeShader->setUniform("uScreenHeight", static_cast<float>(viewport[3]));

    glBindVertexArray(smokeVAO);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(smokeParticles.size()));
    glBindVertexArray(0);

    // Restore state
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_TRUE);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void M2Renderer::removeInstance(uint32_t instanceId) {
    for (auto it = instances.begin(); it != instances.end(); ++it) {
        if (it->id == instanceId) {
            instances.erase(it);
            rebuildSpatialIndex();
            return;
        }
    }
}

void M2Renderer::clear() {
    for (auto& [id, model] : models) {
        if (model.vao != 0) glDeleteVertexArrays(1, &model.vao);
        if (model.vbo != 0) glDeleteBuffers(1, &model.vbo);
        if (model.ebo != 0) glDeleteBuffers(1, &model.ebo);
    }
    models.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    smokeParticles.clear();
    smokeEmitAccum = 0.0f;
}

void M2Renderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocusEnabled = (radius > 0.0f);
    collisionFocusPos = worldPos;
    collisionFocusRadius = std::max(0.0f, radius);
    collisionFocusRadiusSq = collisionFocusRadius * collisionFocusRadius;
}

void M2Renderer::clearCollisionFocus() {
    collisionFocusEnabled = false;
}

void M2Renderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
}

M2Renderer::GridCell M2Renderer::toCell(const glm::vec3& p) const {
    return GridCell{
        static_cast<int>(std::floor(p.x / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.y / SPATIAL_CELL_SIZE)),
        static_cast<int>(std::floor(p.z / SPATIAL_CELL_SIZE))
    };
}

void M2Renderer::rebuildSpatialIndex() {
    spatialGrid.clear();
    instanceIndexById.clear();
    instanceIndexById.reserve(instances.size());

    for (size_t i = 0; i < instances.size(); i++) {
        const auto& inst = instances[i];
        instanceIndexById[inst.id] = i;

        GridCell minCell = toCell(inst.worldBoundsMin);
        GridCell maxCell = toCell(inst.worldBoundsMax);
        for (int z = minCell.z; z <= maxCell.z; z++) {
            for (int y = minCell.y; y <= maxCell.y; y++) {
                for (int x = minCell.x; x <= maxCell.x; x++) {
                    spatialGrid[GridCell{x, y, z}].push_back(inst.id);
                }
            }
        }
    }
}

void M2Renderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
                                  std::vector<size_t>& outIndices) const {
    outIndices.clear();
    candidateIdScratch.clear();

    GridCell minCell = toCell(queryMin);
    GridCell maxCell = toCell(queryMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                auto it = spatialGrid.find(GridCell{x, y, z});
                if (it == spatialGrid.end()) continue;
                for (uint32_t id : it->second) {
                    if (!candidateIdScratch.insert(id).second) continue;
                    auto idxIt = instanceIndexById.find(id);
                    if (idxIt != instanceIndexById.end()) {
                        outIndices.push_back(idxIt->second);
                    }
                }
            }
        }
    }

    // Safety fallback to preserve collision correctness if the spatial index
    // misses candidates (e.g. during streaming churn).
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
}

void M2Renderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    // Find and remove models with no instances
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : models) {
        if (usedModelIds.find(id) == usedModelIds.end()) {
            toRemove.push_back(id);
        }
    }

    // Delete GPU resources and remove from map
    for (uint32_t id : toRemove) {
        auto it = models.find(id);
        if (it != models.end()) {
            if (it->second.vao != 0) glDeleteVertexArrays(1, &it->second.vao);
            if (it->second.vbo != 0) glDeleteBuffers(1, &it->second.vbo);
            if (it->second.ebo != 0) glDeleteBuffers(1, &it->second.ebo);
            models.erase(it);
        }
    }

    if (!toRemove.empty()) {
        LOG_INFO("M2 cleanup: removed ", toRemove.size(), " unused models, ", models.size(), " remaining");
    }
}

GLuint M2Renderer::loadTexture(const std::string& path) {
    // Check cache
    auto it = textureCache.find(path);
    if (it != textureCache.end()) {
        return it->second;
    }

    // Load BLP texture
    pipeline::BLPImage blp = assetManager->loadTexture(path);
    if (!blp.isValid()) {
        LOG_WARNING("M2: Failed to load texture: ", path);
        textureCache[path] = whiteTexture;
        return whiteTexture;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 blp.width, blp.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blp.data.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);
    applyAnisotropicFiltering();

    glBindTexture(GL_TEXTURE_2D, 0);

    textureCache[path] = textureID;
    LOG_DEBUG("M2: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return textureID;
}

uint32_t M2Renderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        auto it = models.find(instance.modelId);
        if (it != models.end()) {
            total += it->second.indexCount / 3;
        }
    }
    return total;
}

std::optional<float> M2Renderer::getFloorHeight(float glX, float glY, float glZ) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;

    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 6.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 8.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - 2.0f || glZ > instance.worldBoundsMax.z + 2.0f) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        if (instance.scale <= 0.001f) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Must be within doodad footprint in local XY.
        // Stepped low platforms get a small pad so walk-up snapping catches edges.
        float footprintPad = 0.0f;
        if (model.collisionSteppedLowPlatform) {
            footprintPad = model.collisionPlanter ? 0.22f : 0.16f;
        }
        if (localPos.x < localMin.x - footprintPad || localPos.x > localMax.x + footprintPad ||
            localPos.y < localMin.y - footprintPad || localPos.y > localMax.y + footprintPad) {
            continue;
        }

        // Construct "top" point at queried XY in local space, then transform back.
        float localTopZ = getEffectiveCollisionTopLocal(model, localPos, localMin, localMax);
        glm::vec3 localTop(localPos.x, localPos.y, localTopZ);
        glm::vec3 worldTop = glm::vec3(instance.modelMatrix * glm::vec4(localTop, 1.0f));

        // Reachability filter: allow a bit more climb for stepped low platforms.
        float maxStepUp = 1.0f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            maxStepUp = 2.0f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 3.0f : 2.4f;
        }
        if (worldTop.z > glZ + maxStepUp) continue;

        if (!bestFloor || worldTop.z > *bestFloor) {
            bestFloor = worldTop.z;
        }
    }

    return bestFloor;
}

bool M2Renderer::checkCollision(const glm::vec3& from, const glm::vec3& to,
                                 glm::vec3& adjustedPos, float playerRadius) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool collided = false;

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(7.0f, 7.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(7.0f, 7.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    // Check against all M2 instances in local space (rotation-aware).
    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        const float broadMargin = playerRadius + 1.0f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && adjustedPos.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && adjustedPos.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && adjustedPos.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && adjustedPos.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + 2.5f && adjustedPos.z > instance.worldBoundsMax.z + 2.5f) continue;
        if (from.z + 2.5f < instance.worldBoundsMin.z && adjustedPos.z + 2.5f < instance.worldBoundsMin.z) continue;

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        if (instance.scale <= 0.001f) continue;

        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
        float radiusScale = model.collisionNarrowVerticalProp ? 0.45f : 1.0f;
        float localRadius = (playerRadius * radiusScale) / instance.scale;

        glm::vec3 rawMin, rawMax;
        getTightCollisionBounds(model, rawMin, rawMax);
        glm::vec3 localMin = rawMin - glm::vec3(localRadius);
        glm::vec3 localMax = rawMax + glm::vec3(localRadius);
        float effectiveTop = getEffectiveCollisionTopLocal(model, localPos, rawMin, rawMax) + localRadius;
        glm::vec2 localCenter((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
        float fromR = glm::length(glm::vec2(localFrom.x, localFrom.y) - localCenter);
        float toR = glm::length(glm::vec2(localPos.x, localPos.y) - localCenter);

        // Feet-based vertical overlap test: ignore objects fully above/below us.
        constexpr float PLAYER_HEIGHT = 2.0f;
        if (localPos.z + PLAYER_HEIGHT < localMin.z || localPos.z > effectiveTop) {
            continue;
        }

        bool fromInsideXY =
            (localFrom.x >= localMin.x && localFrom.x <= localMax.x &&
             localFrom.y >= localMin.y && localFrom.y <= localMax.y);
        bool fromInsideZ = (localFrom.z + PLAYER_HEIGHT >= localMin.z && localFrom.z <= effectiveTop);
        bool escapingOverlap = (fromInsideXY && fromInsideZ && (toR > fromR + 1e-4f));
        bool allowEscapeRelax = escapingOverlap && !model.collisionSmallSolidProp;

        // Swept hard clamp for taller blockers only.
        // Low/stepable objects should be climbable and not "shove" the player off.
        float maxStepUp = 1.20f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            // Keep box/crate-class props hard-solid to prevent phase-through.
            maxStepUp = 0.75f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 2.8f : 2.4f;
        }
        bool stepableLowObject = (effectiveTop <= localFrom.z + maxStepUp);
        bool climbingAttempt = (localPos.z > localFrom.z + 0.18f);
        bool nearTop = (localFrom.z >= effectiveTop - 0.30f);
        float climbAllowance = model.collisionPlanter ? 0.95f : 0.60f;
        if (model.collisionSteppedLowPlatform && !model.collisionPlanter) {
            // Let low curb/planter blocks be stepable without sticky side shoves.
            climbAllowance = 1.00f;
        }
        if (model.collisionSmallSolidProp) {
            climbAllowance = 1.05f;
        }
        bool climbingTowardTop = climbingAttempt && (localFrom.z + climbAllowance >= effectiveTop);
        bool forceHardLateral =
            model.collisionSmallSolidProp &&
            !nearTop && !climbingTowardTop;
        if ((!stepableLowObject || forceHardLateral) && !allowEscapeRelax) {
            float tEnter = 0.0f;
            glm::vec3 sweepMax = localMax;
            sweepMax.z = std::min(sweepMax.z, effectiveTop);
            if (segmentIntersectsAABB(localFrom, localPos, localMin, sweepMax, tEnter)) {
                float tSafe = std::clamp(tEnter - 0.03f, 0.0f, 1.0f);
                glm::vec3 localSafe = localFrom + (localPos - localFrom) * tSafe;
                glm::vec3 worldSafe = glm::vec3(instance.modelMatrix * glm::vec4(localSafe, 1.0f));
                adjustedPos.x = worldSafe.x;
                adjustedPos.y = worldSafe.y;
                collided = true;
                continue;
            }
        }

        if (localPos.x < localMin.x || localPos.x > localMax.x ||
            localPos.y < localMin.y || localPos.y > localMax.y) {
            continue;
        }

        float pushLeft  = localPos.x - localMin.x;
        float pushRight = localMax.x - localPos.x;
        float pushBack  = localPos.y - localMin.y;
        float pushFront = localMax.y - localPos.y;

        float minPush = std::min({pushLeft, pushRight, pushBack, pushFront});
        if (allowEscapeRelax) {
            continue;
        }
        if ((model.collisionSteppedLowPlatform || model.collisionSteppedFountain) && stepableLowObject) {
            // Already on/near top surface: don't apply lateral push that ejects
            // the player from the object when landing.
            continue;
        }
        // Gentle fallback push for overlapping cases.
        float pushAmount;
        if (model.collisionNarrowVerticalProp) {
            pushAmount = std::clamp(minPush * 0.10f, 0.001f, 0.010f);
        } else if (model.collisionSteppedLowPlatform) {
            if (model.collisionPlanter && stepableLowObject) {
                pushAmount = std::clamp(minPush * 0.06f, 0.001f, 0.006f);
            } else {
            pushAmount = std::clamp(minPush * 0.12f, 0.003f, 0.012f);
            }
        } else if (stepableLowObject) {
            pushAmount = std::clamp(minPush * 0.12f, 0.002f, 0.015f);
        } else {
            pushAmount = std::clamp(minPush * 0.28f, 0.010f, 0.045f);
        }
        glm::vec3 localPush(0.0f);
        if (minPush == pushLeft) {
            localPush.x = -pushAmount;
        } else if (minPush == pushRight) {
            localPush.x = pushAmount;
        } else if (minPush == pushBack) {
            localPush.y = -pushAmount;
        } else {
            localPush.y = pushAmount;
        }

        glm::vec3 worldPush = glm::vec3(instance.modelMatrix * glm::vec4(localPush, 0.0f));
        adjustedPos.x += worldPush.x;
        adjustedPos.y += worldPush.y;
        collided = true;
    }

    return collided;
}

float M2Renderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;

    glm::vec3 rayEnd = origin + direction * maxDistance;
    glm::vec3 queryMin = glm::min(origin, rayEnd) - glm::vec3(1.0f);
    glm::vec3 queryMax = glm::max(origin, rayEnd) + glm::vec3(1.0f);
    gatherCandidates(queryMin, queryMax, candidateScratch);

    for (size_t idx : candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocusEnabled &&
            pointAABBDistanceSq(collisionFocusPos, instance.worldBoundsMin, instance.worldBoundsMax) > collisionFocusRadiusSq) {
            continue;
        }

        // Cheap world-space broad-phase.
        float tEnter = 0.0f;
        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.35f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.35f);
        if (!segmentIntersectsAABB(origin, origin + direction * maxDistance, worldMin, worldMax, tEnter)) {
            continue;
        }

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);
        // Skip tiny doodads for camera occlusion; they cause jitter and false hits.
        glm::vec3 extents = (localMax - localMin) * instance.scale;
        if (glm::length(extents) < 0.75f) continue;

        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));
        if (!std::isfinite(localDir.x) || !std::isfinite(localDir.y) || !std::isfinite(localDir.z)) {
            continue;
        }

        // Local-space AABB slab intersection.
        glm::vec3 invDir = 1.0f / localDir;
        glm::vec3 tMin = (localMin - localOrigin) * invDir;
        glm::vec3 tMax = (localMax - localOrigin) * invDir;
        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = std::max({t1.x, t1.y, t1.z});
        float tFar = std::min({t2.x, t2.y, t2.z});
        if (tNear > tFar || tFar <= 0.0f) continue;

        float tHit = tNear > 0.0f ? tNear : tFar;
        glm::vec3 localHit = localOrigin + localDir * tHit;
        glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
        float worldDist = glm::length(worldHit - origin);
        if (worldDist > 0.0f && worldDist < closestHit) {
            closestHit = worldDist;
        }
    }

    return closestHit;
}

} // namespace rendering
} // namespace wowee
