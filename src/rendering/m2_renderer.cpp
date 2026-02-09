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
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <future>
#include <thread>

namespace wowee {
namespace rendering {

namespace {

static constexpr uint32_t kParticleFlagRandomized = 0x40;
static constexpr uint32_t kParticleFlagTiled = 0x80;

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

// Möller–Trumbore ray-triangle intersection.
// Returns distance along ray if hit, negative if miss.
float rayTriangleIntersect(const glm::vec3& origin, const glm::vec3& dir,
                           const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    constexpr float EPSILON = 1e-6f;
    glm::vec3 e1 = v1 - v0;
    glm::vec3 e2 = v2 - v0;
    glm::vec3 h = glm::cross(dir, e2);
    float a = glm::dot(e1, h);
    if (a > -EPSILON && a < EPSILON) return -1.0f;
    float f = 1.0f / a;
    glm::vec3 s = origin - v0;
    float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return -1.0f;
    glm::vec3 q = glm::cross(s, e1);
    float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return -1.0f;
    float t = f * glm::dot(e2, q);
    return t > EPSILON ? t : -1.0f;
}

// Closest point on triangle to a point (Ericson, Real-Time Collision Detection §5.1.5).
glm::vec3 closestPointOnTriangle(const glm::vec3& p,
                                  const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }
    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

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

    numAnimThreads_ = std::min(4u, std::max(1u, std::thread::hardware_concurrency() - 1));
    LOG_INFO("Initializing M2 renderer (", numAnimThreads_, " anim threads)...");

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
        uniform bool uUnlit;
        uniform float uFadeAlpha;

        uniform vec3 uFogColor;
        uniform float uFogStart;
        uniform float uFogEnd;

        uniform sampler2DShadow uShadowMap;
        uniform mat4 uLightSpaceMatrix;
        uniform bool uShadowEnabled;
        uniform float uShadowStrength;
        uniform bool uInteriorDarken;

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

            // Unlit path: emit texture color directly (glow effects, emissive surfaces)
            if (uUnlit) {
                FragColor = vec4(texColor.rgb, finalAlpha);
                return;
            }

            vec3 normal = normalize(Normal);
            vec3 lightDir = normalize(uLightDir);

            vec3 result;
            if (uInteriorDarken) {
                // Interior: dim ambient, minimal directional light
                float diff = max(abs(dot(normal, lightDir)), 0.0) * 0.15;
                result = texColor.rgb * (0.55 + diff);
            } else {
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

                result = ambient + (diffuse + specular) * shadow;
            }

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

    // Create M2 particle emitter shader
    {
        const char* particleVertSrc = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec4 aColor;
            layout (location = 2) in float aSize;
            layout (location = 3) in float aTile;

            uniform mat4 uView;
            uniform mat4 uProjection;

            out vec4 vColor;
            out float vTile;

            void main() {
                vec4 viewPos = uView * vec4(aPos, 1.0);
                gl_Position = uProjection * viewPos;
                float dist = max(-viewPos.z, 1.0);
                gl_PointSize = clamp(aSize * 800.0 / dist, 1.0, 256.0);
                vColor = aColor;
                vTile = aTile;
            }
        )";

        const char* particleFragSrc = R"(
            #version 330 core
            in vec4 vColor;
            in float vTile;
            uniform sampler2D uTexture;
            uniform vec2 uTileCount;
            out vec4 FragColor;

            void main() {
                vec2 tileCount = max(uTileCount, vec2(1.0));
                float tilesX = tileCount.x;
                float tilesY = tileCount.y;
                float tileMax = max(tilesX * tilesY - 1.0, 0.0);
                float tile = clamp(vTile, 0.0, tileMax);
                float col = mod(tile, tilesX);
                float row = floor(tile / tilesX);
                vec2 tileSize = vec2(1.0 / tilesX, 1.0 / tilesY);
                vec2 uv = gl_PointCoord * tileSize + vec2(col, row) * tileSize;
                vec4 texColor = texture(uTexture, uv);
                FragColor = texColor * vColor;
                if (FragColor.a < 0.01) discard;
            }
        )";

        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &particleVertSrc, nullptr);
        glCompileShader(vs);

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &particleFragSrc, nullptr);
        glCompileShader(fs);

        m2ParticleShader_ = glCreateProgram();
        glAttachShader(m2ParticleShader_, vs);
        glAttachShader(m2ParticleShader_, fs);
        glLinkProgram(m2ParticleShader_);
        glDeleteShader(vs);
        glDeleteShader(fs);

        // Create particle VAO/VBO: 9 floats per particle (pos3 + rgba4 + size1 + tile1)
        glGenVertexArrays(1, &m2ParticleVAO_);
        glGenBuffers(1, &m2ParticleVBO_);
        glBindVertexArray(m2ParticleVAO_);
        glBindBuffer(GL_ARRAY_BUFFER, m2ParticleVBO_);
        glBufferData(GL_ARRAY_BUFFER, MAX_M2_PARTICLES * 9 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position (3f)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
        // Color (4f)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
        // Size (1f)
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(7 * sizeof(float)));
        // Tile index (1f)
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(8 * sizeof(float)));
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

    // Generate soft radial gradient glow texture for light sprites
    {
        static constexpr int SZ = 64;
        std::vector<uint8_t> px(SZ * SZ * 4);
        float half = SZ / 2.0f;
        for (int y = 0; y < SZ; y++) {
            for (int x = 0; x < SZ; x++) {
                float dx = (x + 0.5f - half) / half;
                float dy = (y + 0.5f - half) / half;
                float r = std::sqrt(dx * dx + dy * dy);
                float a = std::max(0.0f, 1.0f - r);
                a = a * a; // Quadratic falloff
                int idx = (y * SZ + x) * 4;
                px[idx + 0] = 255;
                px[idx + 1] = 255;
                px[idx + 2] = 255;
                px[idx + 3] = static_cast<uint8_t>(a * 255);
            }
        }
        glGenTextures(1, &glowTexture);
        glBindTexture(GL_TEXTURE_2D, glowTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SZ, SZ, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

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
    if (glowTexture != 0) {
        glDeleteTextures(1, &glowTexture);
        glowTexture = 0;
    }

    shader.reset();

    // Clean up smoke particle resources
    if (smokeVAO != 0) { glDeleteVertexArrays(1, &smokeVAO); smokeVAO = 0; }
    if (smokeVBO != 0) { glDeleteBuffers(1, &smokeVBO); smokeVBO = 0; }
    smokeShader.reset();
    smokeParticles.clear();

    // Clean up M2 particle resources
    if (m2ParticleVAO_ != 0) { glDeleteVertexArrays(1, &m2ParticleVAO_); m2ParticleVAO_ = 0; }
    if (m2ParticleVBO_ != 0) { glDeleteBuffers(1, &m2ParticleVBO_); m2ParticleVBO_ = 0; }
    if (m2ParticleShader_ != 0) { glDeleteProgram(m2ParticleShader_); m2ParticleShader_ = 0; }
}

// ---------------------------------------------------------------------------
// M2 collision mesh: build spatial grid + classify triangles
// ---------------------------------------------------------------------------
void M2ModelGPU::CollisionMesh::build() {
    if (indices.size() < 3 || vertices.empty()) return;
    triCount = static_cast<uint32_t>(indices.size() / 3);

    // Bounding box for grid
    glm::vec3 bmin(std::numeric_limits<float>::max());
    glm::vec3 bmax(-std::numeric_limits<float>::max());
    for (const auto& v : vertices) {
        bmin = glm::min(bmin, v);
        bmax = glm::max(bmax, v);
    }

    gridOrigin = glm::vec2(bmin.x, bmin.y);
    gridCellsX = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.x - bmin.x) / CELL_SIZE))));
    gridCellsY = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.y - bmin.y) / CELL_SIZE))));

    cellFloorTris.resize(gridCellsX * gridCellsY);
    cellWallTris.resize(gridCellsX * gridCellsY);
    triBounds.resize(triCount);

    for (uint32_t ti = 0; ti < triCount; ti++) {
        uint16_t i0 = indices[ti * 3];
        uint16_t i1 = indices[ti * 3 + 1];
        uint16_t i2 = indices[ti * 3 + 2];
        if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) continue;

        const auto& v0 = vertices[i0];
        const auto& v1 = vertices[i1];
        const auto& v2 = vertices[i2];

        triBounds[ti].minZ = std::min({v0.z, v1.z, v2.z});
        triBounds[ti].maxZ = std::max({v0.z, v1.z, v2.z});

        glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
        float normalLen = glm::length(normal);
        float absNz = (normalLen > 0.001f) ? std::abs(normal.z / normalLen) : 0.0f;
        bool isFloor = (absNz >= 0.40f);  // ~66° max slope (relaxed for steep stairs)
        bool isWall  = (absNz < 0.65f);

        float triMinX = std::min({v0.x, v1.x, v2.x});
        float triMaxX = std::max({v0.x, v1.x, v2.x});
        float triMinY = std::min({v0.y, v1.y, v2.y});
        float triMaxY = std::max({v0.y, v1.y, v2.y});

        int cxMin = std::clamp(static_cast<int>((triMinX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cxMax = std::clamp(static_cast<int>((triMaxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cyMin = std::clamp(static_cast<int>((triMinY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
        int cyMax = std::clamp(static_cast<int>((triMaxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);

        for (int cy = cyMin; cy <= cyMax; cy++) {
            for (int cx = cxMin; cx <= cxMax; cx++) {
                int ci = cy * gridCellsX + cx;
                if (isFloor) cellFloorTris[ci].push_back(ti);
                if (isWall)  cellWallTris[ci].push_back(ti);
            }
        }
    }
}

void M2ModelGPU::CollisionMesh::getFloorTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellFloorTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void M2ModelGPU::CollisionMesh::getWallTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellWallTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
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
    bool chestName = false;
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
        bool bridgeName =
            (lowerName.find("bridge") != std::string::npos) ||
            (lowerName.find("plank") != std::string::npos) ||
            (lowerName.find("walkway") != std::string::npos);
        gpuModel.collisionSteppedLowPlatform = (!gpuModel.collisionSteppedFountain) &&
                                               (knownStormwindPlanter ||
                                                bridgeName ||
                                                (likelyCurbName && (lowPlatformShape || lowWideShape)));
        gpuModel.collisionBridge = bridgeName;

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
        chestName = (lowerName.find("chest") != std::string::npos);
        bool foliageName =
            (lowerName.find("bush") != std::string::npos) ||
            (lowerName.find("grass") != std::string::npos) ||
            (lowerName.find("drygrass") != std::string::npos) ||
            (lowerName.find("dry_grass") != std::string::npos) ||
            (lowerName.find("dry-grass") != std::string::npos) ||
            (lowerName.find("deadgrass") != std::string::npos) ||
            (lowerName.find("dead_grass") != std::string::npos) ||
            (lowerName.find("dead-grass") != std::string::npos) ||
            ((lowerName.find("plant") != std::string::npos) && !isPlanter) ||
            (lowerName.find("flower") != std::string::npos) ||
            (lowerName.find("shrub") != std::string::npos) ||
            (lowerName.find("fern") != std::string::npos) ||
            (lowerName.find("vine") != std::string::npos) ||
            (lowerName.find("lily") != std::string::npos) ||
            (lowerName.find("weed") != std::string::npos) ||
            (lowerName.find("wheat") != std::string::npos) ||
            (lowerName.find("pumpkin") != std::string::npos) ||
            (lowerName.find("firefly") != std::string::npos) ||
            (lowerName.find("fireflies") != std::string::npos) ||
            (lowerName.find("fireflys") != std::string::npos) ||
            (lowerName.find("mushroom") != std::string::npos) ||
            (lowerName.find("fungus") != std::string::npos) ||
            (lowerName.find("toadstool") != std::string::npos) ||
            (lowerName.find("root") != std::string::npos) ||
            (lowerName.find("branch") != std::string::npos) ||
            (lowerName.find("thorn") != std::string::npos) ||
            (lowerName.find("moss") != std::string::npos) ||
            (lowerName.find("ivy") != std::string::npos) ||
            (lowerName.find("seaweed") != std::string::npos) ||
            (lowerName.find("kelp") != std::string::npos) ||
            (lowerName.find("cattail") != std::string::npos) ||
            (lowerName.find("reed") != std::string::npos);
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
    gpuModel.disableAnimation = foliageOrTreeLike || chestName;

    // Build collision mesh + spatial grid from M2 bounding geometry
    gpuModel.collision.vertices = model.collisionVertices;
    gpuModel.collision.indices = model.collisionIndices;
    gpuModel.collision.build();
    if (gpuModel.collision.valid()) {
        core::Logger::getInstance().debug("  M2 collision mesh: ", gpuModel.collision.triCount,
            " tris, grid ", gpuModel.collision.gridCellsX, "x", gpuModel.collision.gridCellsY);
    }

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

    // Copy particle emitter data and resolve textures
    gpuModel.particleEmitters = model.particleEmitters;
    gpuModel.particleTextures.resize(model.particleEmitters.size(), whiteTexture);
    for (size_t ei = 0; ei < model.particleEmitters.size(); ei++) {
        uint16_t texIdx = model.particleEmitters[ei].texture;
        if (texIdx < allTextures.size() && allTextures[texIdx] != 0) {
            gpuModel.particleTextures[ei] = allTextures[texIdx];
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

            // Store blend mode and flags from material
            if (batch.materialIndex < model.materials.size()) {
                bgpu.blendMode = model.materials[batch.materialIndex].blendMode;
                bgpu.materialFlags = model.materials[batch.materialIndex].flags;
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

            // Compute batch center and radius for glow sprite positioning
            if (bgpu.blendMode >= 3 && batch.indexCount > 0) {
                glm::vec3 sum(0.0f);
                uint32_t counted = 0;
                for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                    if (j < model.indices.size()) {
                        uint16_t vi = model.indices[j];
                        if (vi < model.vertices.size()) {
                            sum += model.vertices[vi].position;
                            counted++;
                        }
                    }
                }
                if (counted > 0) {
                    bgpu.center = sum / static_cast<float>(counted);
                    float maxDist = 0.0f;
                    for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                        if (j < model.indices.size()) {
                            uint16_t vi = model.indices[j];
                            if (vi < model.vertices.size()) {
                                float d = glm::length(model.vertices[vi].position - bgpu.center);
                                maxDist = std::max(maxDist, d);
                            }
                        }
                    }
                    bgpu.glowSize = std::max(maxDist, 0.5f);
                }
            }

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

void M2Renderer::update(float deltaTime, const glm::vec3& cameraPos, const glm::mat4& viewProjection) {
    float dtMs = deltaTime * 1000.0f;

    // Cache camera state for frustum-culling bone computation
    cachedCamPos_ = cameraPos;
    const float maxRenderDistance = (instances.size() > 600) ? 320.0f : 2800.0f;
    cachedMaxRenderDistSq_ = maxRenderDistance * maxRenderDistance;

    // Build frustum for culling bones
    Frustum updateFrustum;
    updateFrustum.extractFromMatrix(viewProjection);

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
    // Phase 1: Update animation state (cheap, sequential)
    // Collect indices of instances that need bone matrix computation.
    std::vector<size_t> boneWorkIndices;
    boneWorkIndices.reserve(instances.size());

    for (size_t idx = 0; idx < instances.size(); ++idx) {
        auto& instance = instances[idx];
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

        // Frustum + distance cull: skip expensive bone computation for off-screen instances
        float worldRadius = model.boundRadius * instance.scale;
        float cullRadius = worldRadius;
        glm::vec3 toCam = instance.position - cachedCamPos_;
        float distSq = glm::dot(toCam, toCam);
        float effectiveMaxDistSq = cachedMaxRenderDistSq_ * std::max(1.0f, cullRadius / 12.0f);
        if (!model.disableAnimation) {
            if (worldRadius < 0.8f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 95.0f * 95.0f);
            } else if (worldRadius < 1.5f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 140.0f * 140.0f);
            }
        }
        if (distSq > effectiveMaxDistSq) continue;
        if (cullRadius > 0.0f && !updateFrustum.intersectsSphere(instance.position, cullRadius)) continue;

        boneWorkIndices.push_back(idx);
    }

    // Phase 2: Compute bone matrices (expensive, parallel if enough work)
    const size_t animCount = boneWorkIndices.size();
    if (animCount > 0) {
        if (animCount < 32 || numAnimThreads_ <= 1) {
            // Sequential — not enough work to justify thread overhead
            for (size_t i : boneWorkIndices) {
                if (i >= instances.size()) continue;
                auto& inst = instances[i];
                auto mdlIt = models.find(inst.modelId);
                if (mdlIt == models.end()) continue;
                computeBoneMatrices(mdlIt->second, inst);
            }
        } else {
            // Parallel — dispatch across worker threads
            const size_t numThreads = std::min(static_cast<size_t>(numAnimThreads_), animCount);
            const size_t chunkSize = animCount / numThreads;
            const size_t remainder = animCount % numThreads;

            std::vector<std::future<void>> futures;
            futures.reserve(numThreads);

            size_t start = 0;
            for (size_t t = 0; t < numThreads; ++t) {
                size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                futures.push_back(std::async(std::launch::async,
                    [this, &boneWorkIndices, start, end]() {
                        for (size_t j = start; j < end; ++j) {
                            size_t idx = boneWorkIndices[j];
                            if (idx >= instances.size()) continue;
                            auto& inst = instances[idx];
                            auto mdlIt = models.find(inst.modelId);
                            if (mdlIt == models.end()) continue;
                            computeBoneMatrices(mdlIt->second, inst);
                        }
                    }));
                start = end;
            }

            for (auto& f : futures) {
                f.get();
            }
        }
    }

    // Phase 3: Particle update (sequential — uses RNG, not thread-safe)
    for (size_t idx : boneWorkIndices) {
        if (idx >= instances.size()) continue;
        auto& instance = instances[idx];
        auto mdlIt = models.find(instance.modelId);
        if (mdlIt == models.end()) continue;
        const auto& model = mdlIt->second;
        if (!model.particleEmitters.empty()) {
            emitParticles(instance, model, deltaTime);
            updateParticles(instance, deltaTime);
        }
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

    // Collect glow sprites from additive/mod batches for deferred rendering
    struct GlowSprite {
        glm::vec3 worldPos;
        glm::vec4 color; // RGBA
        float size;
    };
    std::vector<GlowSprite> glowSprites;

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

    // Build sorted visible instance list: cull then sort by modelId to batch VAO binds
    struct VisibleEntry {
        uint32_t index;
        uint32_t modelId;
        float distSq;
        float effectiveMaxDistSq;
    };
    std::vector<VisibleEntry> sortedVisible;
    sortedVisible.reserve(instances.size() / 2);

    for (uint32_t i = 0; i < static_cast<uint32_t>(instances.size()); ++i) {
        const auto& instance = instances[i];
        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        const M2ModelGPU& model = it->second;
        if (!model.isValid() || model.isSmoke) continue;

        glm::vec3 toCam = instance.position - camPos;
        float distSq = glm::dot(toCam, toCam);
        float worldRadius = model.boundRadius * instance.scale;
        float cullRadius = worldRadius;
        if (model.disableAnimation) {
            cullRadius = std::max(cullRadius, 3.0f);
        }
        float effectiveMaxDistSq = maxRenderDistanceSq * std::max(1.0f, cullRadius / 12.0f);
        if (model.disableAnimation) {
            effectiveMaxDistSq *= 2.6f;
        }
        if (!model.disableAnimation) {
            if (worldRadius < 0.8f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 95.0f * 95.0f);
            } else if (worldRadius < 1.5f) {
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, 140.0f * 140.0f);
            }
        }
        if (distSq > effectiveMaxDistSq) continue;
        if (cullRadius > 0.0f && !frustum.intersectsSphere(instance.position, cullRadius)) continue;

        sortedVisible.push_back({i, instance.modelId, distSq, effectiveMaxDistSq});
    }

    // Sort by modelId to minimize VAO rebinds
    std::sort(sortedVisible.begin(), sortedVisible.end(),
              [](const VisibleEntry& a, const VisibleEntry& b) { return a.modelId < b.modelId; });

    uint32_t currentModelId = UINT32_MAX;
    const M2ModelGPU* currentModel = nullptr;

    for (const auto& entry : sortedVisible) {
        if (entry.index >= instances.size()) continue;
        const auto& instance = instances[entry.index];

        // Bind VAO once per model group
        if (entry.modelId != currentModelId) {
            if (currentModel) glBindVertexArray(0);
            currentModelId = entry.modelId;
            auto mdlIt = models.find(currentModelId);
            if (mdlIt == models.end()) continue;
            currentModel = &mdlIt->second;
            glBindVertexArray(currentModel->vao);
        }

        const M2ModelGPU& model = *currentModel;

        // Distance-based fade alpha for smooth pop-in (squared-distance, no sqrt)
        float fadeAlpha = 1.0f;
        float fadeFrac = model.disableAnimation ? 0.55f : fadeStartFraction;
        float fadeStartDistSq = entry.effectiveMaxDistSq * fadeFrac * fadeFrac;
        if (entry.distSq > fadeStartDistSq) {
            fadeAlpha = std::clamp((entry.effectiveMaxDistSq - entry.distSq) /
                                  (entry.effectiveMaxDistSq - fadeStartDistSq), 0.0f, 1.0f);
        }

        shader->setUniform("uModel", instance.modelMatrix);
        shader->setUniform("uFadeAlpha", fadeAlpha);
        shader->setUniform("uInteriorDarken", insideInterior);

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

        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;

            // Additive/mod batches (glow halos, light effects): collect as glow sprites
            // instead of rendering the mesh geometry which appears as flat orange disks.
            if (batch.blendMode >= 3) {
                if (entry.distSq < 120.0f * 120.0f) { // Only render glow within 120 units
                    glm::vec3 worldPos = glm::vec3(instance.modelMatrix * glm::vec4(batch.center, 1.0f));
                    GlowSprite gs;
                    gs.worldPos = worldPos;
                    gs.color = glm::vec4(1.0f, 0.75f, 0.35f, 0.85f);
                    gs.size = batch.glowSize * instance.scale;
                    glowSprites.push_back(gs);
                }
                continue;
            }

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

            // Apply per-batch blend mode from M2 material
            // 0=Opaque, 1=AlphaKey, 2=Alpha, 3=Add, 4=Mod, 5=Mod2x, 6=BlendAdd, 7=Screen
            bool batchTransparent = false;
            switch (batch.blendMode) {
                case 0: // Opaque
                    glBlendFunc(GL_ONE, GL_ZERO);
                    break;
                case 1: // Alpha Key (alpha test, handled by uAlphaTest)
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    break;
                case 2: // Alpha
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    batchTransparent = true;
                    break;
                case 3: // Additive
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                    batchTransparent = true;
                    break;
                case 4: // Mod
                    glBlendFunc(GL_DST_COLOR, GL_ZERO);
                    batchTransparent = true;
                    break;
                case 5: // Mod2x
                    glBlendFunc(GL_DST_COLOR, GL_SRC_COLOR);
                    batchTransparent = true;
                    break;
                case 6: // BlendAdd
                    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
                    batchTransparent = true;
                    break;
                default: // Fallback
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    break;
            }

            // Disable depth writes for transparent/additive batches
            if (batchTransparent && fadeAlpha >= 1.0f) {
                glDepthMask(GL_FALSE);
            }


            // Unlit: material flag 0x01
            bool unlit = (batch.materialFlags & 0x01) != 0;
            shader->setUniform("uUnlit", unlit);

            bool hasTexture = (batch.texture != 0);
            shader->setUniform("uHasTexture", hasTexture);
            shader->setUniform("uAlphaTest", batch.blendMode == 1);

            if (hasTexture) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, batch.texture);
                shader->setUniform("uTexture", 0);
            }

            glDrawElements(GL_TRIANGLES, batch.indexCount, GL_UNSIGNED_SHORT,
                           (void*)(batch.indexStart * sizeof(uint16_t)));

            // Restore depth writes and blend func after transparent batch
            if (batchTransparent && fadeAlpha >= 1.0f) {
                glDepthMask(GL_TRUE);
            }
            if (batch.blendMode != 0 && batch.blendMode != 2) {
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }

            lastDrawCallCount++;
        }

        if (fadeAlpha < 1.0f) {
            glDepthMask(GL_TRUE);
        }
    }

    if (currentModel) glBindVertexArray(0);

    // Render glow sprites as billboarded additive point lights
    if (!glowSprites.empty() && m2ParticleShader_ != 0 && m2ParticleVAO_ != 0) {
        glUseProgram(m2ParticleShader_);

        GLint viewLoc = glGetUniformLocation(m2ParticleShader_, "uView");
        GLint projLoc = glGetUniformLocation(m2ParticleShader_, "uProjection");
        GLint texLoc = glGetUniformLocation(m2ParticleShader_, "uTexture");
        GLint tileLoc = glGetUniformLocation(m2ParticleShader_, "uTileCount");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        glUniform1i(texLoc, 0);
        glUniform2f(tileLoc, 1.0f, 1.0f);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive blending
        glDepthMask(GL_FALSE);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glDisable(GL_CULL_FACE);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, glowTexture);

        // Build vertex data: pos(3) + color(4) + size(1) + tile(1) = 9 floats per sprite
        std::vector<float> glowData;
        glowData.reserve(glowSprites.size() * 9);
        for (const auto& gs : glowSprites) {
            glowData.push_back(gs.worldPos.x);
            glowData.push_back(gs.worldPos.y);
            glowData.push_back(gs.worldPos.z);
            glowData.push_back(gs.color.r);
            glowData.push_back(gs.color.g);
            glowData.push_back(gs.color.b);
            glowData.push_back(gs.color.a);
            glowData.push_back(gs.size);
            glowData.push_back(0.0f);
        }

        glBindVertexArray(m2ParticleVAO_);
        glBindBuffer(GL_ARRAY_BUFFER, m2ParticleVBO_);
        size_t uploadCount = std::min(glowSprites.size(), MAX_M2_PARTICLES);
        glBufferSubData(GL_ARRAY_BUFFER, 0, uploadCount * 9 * sizeof(float), glowData.data());
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(uploadCount));
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDisable(GL_PROGRAM_POINT_SIZE);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

// --- M2 Particle Emitter Helpers ---

float M2Renderer::interpFloat(const pipeline::M2AnimationTrack& track, float animTime,
                                int seqIdx, const std::vector<pipeline::M2Sequence>& /*seqs*/,
                                const std::vector<uint32_t>& globalSeqDurations) {
    if (!track.hasData()) return 0.0f;
    int si; float t;
    resolveTrackTime(track, seqIdx, animTime, globalSeqDurations, si, t);
    if (si < 0 || si >= static_cast<int>(track.sequences.size())) return 0.0f;
    const auto& keys = track.sequences[si];
    if (keys.timestamps.empty() || keys.floatValues.empty()) return 0.0f;
    if (keys.floatValues.size() == 1) return keys.floatValues[0];
    int idx = findKeyframeIndex(keys.timestamps, t);
    if (idx < 0) return 0.0f;
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, keys.floatValues.size() - 1);
    if (i0 == i1) return keys.floatValues[i0];
    float t0 = static_cast<float>(keys.timestamps[i0]);
    float t1 = static_cast<float>(keys.timestamps[i1]);
    float dur = t1 - t0;
    float frac = (dur > 0.0f) ? glm::clamp((t - t0) / dur, 0.0f, 1.0f) : 0.0f;
    return glm::mix(keys.floatValues[i0], keys.floatValues[i1], frac);
}

float M2Renderer::interpFBlockFloat(const pipeline::M2FBlock& fb, float lifeRatio) {
    if (fb.floatValues.empty()) return 1.0f;
    if (fb.floatValues.size() == 1 || fb.timestamps.empty()) return fb.floatValues[0];
    lifeRatio = glm::clamp(lifeRatio, 0.0f, 1.0f);
    // Find surrounding timestamps
    for (size_t i = 0; i < fb.timestamps.size() - 1; i++) {
        if (lifeRatio <= fb.timestamps[i + 1]) {
            float t0 = fb.timestamps[i];
            float t1 = fb.timestamps[i + 1];
            float dur = t1 - t0;
            float frac = (dur > 0.0f) ? (lifeRatio - t0) / dur : 0.0f;
            size_t v0 = std::min(i, fb.floatValues.size() - 1);
            size_t v1 = std::min(i + 1, fb.floatValues.size() - 1);
            return glm::mix(fb.floatValues[v0], fb.floatValues[v1], frac);
        }
    }
    return fb.floatValues.back();
}

glm::vec3 M2Renderer::interpFBlockVec3(const pipeline::M2FBlock& fb, float lifeRatio) {
    if (fb.vec3Values.empty()) return glm::vec3(1.0f);
    if (fb.vec3Values.size() == 1 || fb.timestamps.empty()) return fb.vec3Values[0];
    lifeRatio = glm::clamp(lifeRatio, 0.0f, 1.0f);
    for (size_t i = 0; i < fb.timestamps.size() - 1; i++) {
        if (lifeRatio <= fb.timestamps[i + 1]) {
            float t0 = fb.timestamps[i];
            float t1 = fb.timestamps[i + 1];
            float dur = t1 - t0;
            float frac = (dur > 0.0f) ? (lifeRatio - t0) / dur : 0.0f;
            size_t v0 = std::min(i, fb.vec3Values.size() - 1);
            size_t v1 = std::min(i + 1, fb.vec3Values.size() - 1);
            return glm::mix(fb.vec3Values[v0], fb.vec3Values[v1], frac);
        }
    }
    return fb.vec3Values.back();
}

void M2Renderer::emitParticles(M2Instance& inst, const M2ModelGPU& gpu, float dt) {
    if (inst.emitterAccumulators.size() != gpu.particleEmitters.size()) {
        inst.emitterAccumulators.resize(gpu.particleEmitters.size(), 0.0f);
    }

    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
    std::uniform_real_distribution<float> distN(-1.0f, 1.0f);
    std::uniform_int_distribution<int> distTile;

    for (size_t ei = 0; ei < gpu.particleEmitters.size(); ei++) {
        const auto& em = gpu.particleEmitters[ei];
        if (!em.enabled) continue;

        float rate = interpFloat(em.emissionRate, inst.animTime, inst.currentSequenceIndex,
                                  gpu.sequences, gpu.globalSequenceDurations);
        float life = interpFloat(em.lifespan, inst.animTime, inst.currentSequenceIndex,
                                  gpu.sequences, gpu.globalSequenceDurations);
        if (rate <= 0.0f || life <= 0.0f) continue;

        inst.emitterAccumulators[ei] += rate * dt;

        while (inst.emitterAccumulators[ei] >= 1.0f && inst.particles.size() < MAX_M2_PARTICLES) {
            inst.emitterAccumulators[ei] -= 1.0f;

            M2Particle p;
            p.emitterIndex = static_cast<int>(ei);
            p.life = 0.0f;
            p.maxLife = life;
            p.tileIndex = 0.0f;

            // Position: emitter position transformed by bone matrix
            glm::vec3 localPos = em.position;
            glm::mat4 boneXform = glm::mat4(1.0f);
            if (em.bone < inst.boneMatrices.size()) {
                boneXform = inst.boneMatrices[em.bone];
            }
            glm::vec3 worldPos = glm::vec3(inst.modelMatrix * boneXform * glm::vec4(localPos, 1.0f));
            p.position = worldPos;

            // Velocity: emission speed in upward direction + random spread
            float speed = interpFloat(em.emissionSpeed, inst.animTime, inst.currentSequenceIndex,
                                       gpu.sequences, gpu.globalSequenceDurations);
            float vRange = interpFloat(em.verticalRange, inst.animTime, inst.currentSequenceIndex,
                                        gpu.sequences, gpu.globalSequenceDurations);
            float hRange = interpFloat(em.horizontalRange, inst.animTime, inst.currentSequenceIndex,
                                        gpu.sequences, gpu.globalSequenceDurations);

            // Base direction: up in model space, transformed to world
            glm::vec3 dir(0.0f, 0.0f, 1.0f);
            // Add random spread
            dir.x += distN(particleRng_) * hRange;
            dir.y += distN(particleRng_) * hRange;
            dir.z += distN(particleRng_) * vRange;
            float len = glm::length(dir);
            if (len > 0.001f) dir /= len;

            // Transform direction by bone + model orientation (rotation only)
            glm::mat3 rotMat = glm::mat3(inst.modelMatrix * boneXform);
            p.velocity = rotMat * dir * speed;

            const uint32_t tilesX = std::max<uint16_t>(em.textureCols, 1);
            const uint32_t tilesY = std::max<uint16_t>(em.textureRows, 1);
            const uint32_t totalTiles = tilesX * tilesY;
            if ((em.flags & kParticleFlagTiled) && totalTiles > 1) {
                if (em.flags & kParticleFlagRandomized) {
                    distTile = std::uniform_int_distribution<int>(0, static_cast<int>(totalTiles - 1));
                    p.tileIndex = static_cast<float>(distTile(particleRng_));
                } else {
                    p.tileIndex = 0.0f;
                }
            }

            inst.particles.push_back(p);
        }
        // Cap accumulator to avoid bursts after lag
        if (inst.emitterAccumulators[ei] > 2.0f) {
            inst.emitterAccumulators[ei] = 0.0f;
        }
    }
}

void M2Renderer::updateParticles(M2Instance& inst, float dt) {
    auto it = models.find(inst.modelId);
    if (it == models.end()) return;
    const auto& gpu = it->second;

    for (size_t i = 0; i < inst.particles.size(); ) {
        auto& p = inst.particles[i];
        p.life += dt;
        if (p.life >= p.maxLife) {
            // Swap-and-pop removal
            inst.particles[i] = inst.particles.back();
            inst.particles.pop_back();
            continue;
        }
        // Apply gravity
        if (p.emitterIndex >= 0 && p.emitterIndex < static_cast<int>(gpu.particleEmitters.size())) {
            float grav = interpFloat(gpu.particleEmitters[p.emitterIndex].gravity,
                                      inst.animTime, inst.currentSequenceIndex,
                                      gpu.sequences, gpu.globalSequenceDurations);
            p.velocity.z -= grav * dt;
        }
        p.position += p.velocity * dt;
        i++;
    }
}

void M2Renderer::renderM2Particles(const glm::mat4& view, const glm::mat4& proj) {
    if (m2ParticleShader_ == 0 || m2ParticleVAO_ == 0) return;

    // Collect all particles from all instances, grouped by texture+blend
    struct ParticleGroupKey {
        GLuint texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;

        bool operator==(const ParticleGroupKey& other) const {
            return texture == other.texture &&
                   blendType == other.blendType &&
                   tilesX == other.tilesX &&
                   tilesY == other.tilesY;
        }
    };
    struct ParticleGroupKeyHash {
        size_t operator()(const ParticleGroupKey& key) const {
            size_t h1 = std::hash<uint32_t>{}(key.texture);
            size_t h2 = std::hash<uint32_t>{}((static_cast<uint32_t>(key.tilesX) << 16) | key.tilesY);
            size_t h3 = std::hash<uint8_t>{}(key.blendType);
            return h1 ^ (h2 * 0x9e3779b9u) ^ (h3 * 0x85ebca6bu);
        }
    };
    struct ParticleGroup {
        GLuint texture;
        uint8_t blendType;
        uint16_t tilesX;
        uint16_t tilesY;
        std::vector<float> vertexData;  // 9 floats per particle
    };
    std::unordered_map<ParticleGroupKey, ParticleGroup, ParticleGroupKeyHash> groups;

    size_t totalParticles = 0;

    for (auto& inst : instances) {
        if (inst.particles.empty()) continue;
        auto it = models.find(inst.modelId);
        if (it == models.end()) continue;
        const auto& gpu = it->second;

        for (const auto& p : inst.particles) {
            if (p.emitterIndex < 0 || p.emitterIndex >= static_cast<int>(gpu.particleEmitters.size())) continue;
            const auto& em = gpu.particleEmitters[p.emitterIndex];

            float lifeRatio = p.life / std::max(p.maxLife, 0.001f);
            glm::vec3 color = interpFBlockVec3(em.particleColor, lifeRatio);
            float alpha = interpFBlockFloat(em.particleAlpha, lifeRatio);
            float scale = interpFBlockFloat(em.particleScale, lifeRatio);

            GLuint tex = whiteTexture;
            if (p.emitterIndex < static_cast<int>(gpu.particleTextures.size())) {
                tex = gpu.particleTextures[p.emitterIndex];
            }

            uint16_t tilesX = std::max<uint16_t>(em.textureCols, 1);
            uint16_t tilesY = std::max<uint16_t>(em.textureRows, 1);
            uint32_t totalTiles = static_cast<uint32_t>(tilesX) * static_cast<uint32_t>(tilesY);
            ParticleGroupKey key{tex, em.blendingType, tilesX, tilesY};
            auto& group = groups[key];
            group.texture = tex;
            group.blendType = em.blendingType;
            group.tilesX = tilesX;
            group.tilesY = tilesY;

            group.vertexData.push_back(p.position.x);
            group.vertexData.push_back(p.position.y);
            group.vertexData.push_back(p.position.z);
            group.vertexData.push_back(color.r);
            group.vertexData.push_back(color.g);
            group.vertexData.push_back(color.b);
            group.vertexData.push_back(alpha);
            group.vertexData.push_back(scale);
            float tileIndex = p.tileIndex;
            if ((em.flags & kParticleFlagTiled) && totalTiles > 1) {
                float animSeconds = inst.animTime / 1000.0f;
                uint32_t animFrame = static_cast<uint32_t>(std::floor(animSeconds * totalTiles)) % totalTiles;
                tileIndex = std::fmod(p.tileIndex + static_cast<float>(animFrame),
                                      static_cast<float>(totalTiles));
            }
            group.vertexData.push_back(tileIndex);
            totalParticles++;
        }
    }

    if (totalParticles == 0) return;

    // Set up GL state
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_CULL_FACE);

    glUseProgram(m2ParticleShader_);

    GLint viewLoc = glGetUniformLocation(m2ParticleShader_, "uView");
    GLint projLoc = glGetUniformLocation(m2ParticleShader_, "uProjection");
    GLint texLoc = glGetUniformLocation(m2ParticleShader_, "uTexture");
    GLint tileLoc = glGetUniformLocation(m2ParticleShader_, "uTileCount");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));
    glUniform1i(texLoc, 0);
    glActiveTexture(GL_TEXTURE0);

    glBindVertexArray(m2ParticleVAO_);

    for (auto& [key, group] : groups) {
        if (group.vertexData.empty()) continue;

        // Set blend mode
        if (group.blendType == 4) {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // Additive
        } else {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Alpha
        }

        glBindTexture(GL_TEXTURE_2D, group.texture);
        glUniform2f(tileLoc, static_cast<float>(group.tilesX), static_cast<float>(group.tilesY));

        // Upload and draw in chunks of MAX_M2_PARTICLES
        size_t count = group.vertexData.size() / 9;
        size_t offset = 0;
        while (offset < count) {
            size_t batch = std::min(count - offset, MAX_M2_PARTICLES);
            glBindBuffer(GL_ARRAY_BUFFER, m2ParticleVBO_);
            glBufferSubData(GL_ARRAY_BUFFER, 0, batch * 9 * sizeof(float),
                            &group.vertexData[offset * 9]);
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(batch));
            offset += batch;
        }
    }

    glBindVertexArray(0);

    // Restore state
    glDepthMask(GL_TRUE);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
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

void M2Renderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.position = position;
    inst.updateModelMatrix();
    auto modelIt = models.find(inst.modelId);
    if (modelIt != models.end()) {
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(modelIt->second, localMin, localMax);
        transformAABB(inst.modelMatrix, localMin, localMax, inst.worldBoundsMin, inst.worldBoundsMax);
    }
    rebuildSpatialIndex();
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
        // Don't cache failures — transient StormLib thread contention can
        // cause reads to fail; next loadModel call will retry.
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

        auto it = models.find(instance.modelId);
        if (it == models.end()) continue;
        if (instance.scale <= 0.001f) continue;

        const M2ModelGPU& model = it->second;
        if (model.collisionNoBlock) continue;

        // --- Mesh-based floor: vertical ray vs collision triangles ---
        // Does NOT skip the AABB path — both contribute and highest wins.
        if (model.collision.valid()) {
            glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

            model.collision.getFloorTrisInRange(
                localPos.x - 1.0f, localPos.y - 1.0f,
                localPos.x + 1.0f, localPos.y + 1.0f,
                collisionTriScratch_);

            glm::vec3 rayOrigin(localPos.x, localPos.y, localPos.z + 5.0f);
            glm::vec3 rayDir(0.0f, 0.0f, -1.0f);
            float bestHitZ = -std::numeric_limits<float>::max();
            bool hitAny = false;

            for (uint32_t ti : collisionTriScratch_) {
                if (ti >= model.collision.triCount) continue;
                if (model.collision.triBounds[ti].maxZ < localPos.z - 10.0f ||
                    model.collision.triBounds[ti].minZ > localPos.z + 5.0f) continue;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                // Two-sided: try both windings
                float tHit = rayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2);
                if (tHit < 0.0f)
                    tHit = rayTriangleIntersect(rayOrigin, rayDir, v0, v2, v1);
                if (tHit < 0.0f) continue;

                float hitZ = rayOrigin.z - tHit;

                // Walkable normal check (world space)
                glm::vec3 localN = glm::cross(v1 - v0, v2 - v0);
                float nLen = glm::length(localN);
                if (nLen > 0.001f) {
                    localN /= nLen;
                    if (localN.z < 0.0f) localN = -localN;
                    glm::vec3 worldN = glm::normalize(
                        glm::vec3(instance.modelMatrix * glm::vec4(localN, 0.0f)));
                    if (std::abs(worldN.z) < 0.40f) continue; // too steep (~66° max slope)
                }

                if (hitZ <= localPos.z + 3.0f && hitZ > bestHitZ) {
                    bestHitZ = hitZ;
                    hitAny = true;
                }
            }

            if (hitAny) {
                glm::vec3 localHit(localPos.x, localPos.y, bestHitZ);
                glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
                if (worldHit.z <= glZ + 3.0f && (!bestFloor || worldHit.z > *bestFloor)) {
                    bestFloor = worldHit.z;
                }
            }
            // Fall through to AABB floor — both contribute, highest wins
        }

        float zMargin = model.collisionBridge ? 25.0f : 2.0f;
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - zMargin || glZ > instance.worldBoundsMax.z + zMargin) {
            continue;
        }
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Must be within doodad footprint in local XY.
        // Stepped low platforms get a small pad so walk-up snapping catches edges.
        float footprintPad = 0.0f;
        if (model.collisionSteppedLowPlatform) {
            footprintPad = model.collisionPlanter ? 0.22f : 0.16f;
            if (model.collisionBridge) {
                footprintPad = 0.35f;
            }
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
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
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

        // --- Mesh-based wall collision: closest-point push ---
        if (model.collision.valid()) {
            glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
            glm::vec3 localPos  = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
            float localRadius = playerRadius / instance.scale;

            model.collision.getWallTrisInRange(
                std::min(localFrom.x, localPos.x) - localRadius - 1.0f,
                std::min(localFrom.y, localPos.y) - localRadius - 1.0f,
                std::max(localFrom.x, localPos.x) + localRadius + 1.0f,
                std::max(localFrom.y, localPos.y) + localRadius + 1.0f,
                collisionTriScratch_);

            constexpr float PLAYER_HEIGHT = 2.0f;
            constexpr float MAX_TOTAL_PUSH = 0.02f; // Cap total push per instance
            bool pushed = false;
            float totalPushX = 0.0f, totalPushY = 0.0f;

            for (uint32_t ti : collisionTriScratch_) {
                if (ti >= model.collision.triCount) continue;
                if (localPos.z + PLAYER_HEIGHT < model.collision.triBounds[ti].minZ ||
                    localPos.z > model.collision.triBounds[ti].maxZ) continue;

                // Step-up: only skip wall when player is rising (jumping over it)
                constexpr float MAX_STEP_UP = 1.2f;
                bool rising = (localPos.z > localFrom.z + 0.05f);
                if (rising && localPos.z + MAX_STEP_UP >= model.collision.triBounds[ti].maxZ) continue;

                // Early out if we already pushed enough this instance
                float totalPushSoFar = std::sqrt(totalPushX * totalPushX + totalPushY * totalPushY);
                if (totalPushSoFar >= MAX_TOTAL_PUSH) break;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                glm::vec3 closest = closestPointOnTriangle(localPos, v0, v1, v2);
                glm::vec3 diff = localPos - closest;
                float distXY = std::sqrt(diff.x * diff.x + diff.y * diff.y);

                if (distXY < localRadius && distXY > 1e-4f) {
                    // Gentle push — very small fraction of penetration
                    float penetration = localRadius - distXY;
                    float pushDist = std::clamp(penetration * 0.08f, 0.001f, 0.015f);
                    float dx = (diff.x / distXY) * pushDist;
                    float dy = (diff.y / distXY) * pushDist;
                    localPos.x += dx;
                    localPos.y += dy;
                    totalPushX += dx;
                    totalPushY += dy;
                    pushed = true;
                } else if (distXY < 1e-4f) {
                    // On the plane — soft push along triangle normal XY
                    glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
                    float nxyLen = std::sqrt(n.x * n.x + n.y * n.y);
                    if (nxyLen > 1e-4f) {
                        float pushDist = std::min(localRadius, 0.015f);
                        float dx = (n.x / nxyLen) * pushDist;
                        float dy = (n.y / nxyLen) * pushDist;
                        localPos.x += dx;
                        localPos.y += dy;
                        totalPushX += dx;
                        totalPushY += dy;
                        pushed = true;
                    }
                }
            }

            if (pushed) {
                glm::vec3 worldPos = glm::vec3(instance.modelMatrix * glm::vec4(localPos, 1.0f));
                adjustedPos.x = worldPos.x;
                adjustedPos.y = worldPos.y;
                collided = true;
            }
            continue;
        }

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
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
        }
        bool stepableLowObject = (effectiveTop <= localFrom.z + maxStepUp);
        bool climbingAttempt = (localPos.z > localFrom.z + 0.18f);
        bool nearTop = (localFrom.z >= effectiveTop - 0.30f);
        float climbAllowance = model.collisionPlanter ? 0.95f : 0.60f;
        if (model.collisionSteppedLowPlatform && !model.collisionPlanter) {
            // Let low curb/planter blocks be stepable without sticky side shoves.
            climbAllowance = 1.00f;
        }
        if (model.collisionBridge) {
            climbAllowance = 3.0f;
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
        if (stepableLowObject && localFrom.z >= effectiveTop - 0.35f) {
            // Already on/near top surface: don't apply lateral push that ejects
            // the player from the object (carpets, platforms, etc).
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
