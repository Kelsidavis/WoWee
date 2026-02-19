#include "rendering/clouds.hpp"
#include "rendering/camera.hpp"
#include "rendering/shader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>

namespace wowee {
namespace rendering {

Clouds::Clouds() {
}

Clouds::~Clouds() {
    cleanup();
}

bool Clouds::initialize() {
    LOG_INFO("Initializing cloud system");

    // Generate cloud dome mesh
    generateMesh();

    // Create VAO
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices.size() * sizeof(glm::vec3),
                 vertices.data(),
                 GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 indices.size() * sizeof(unsigned int),
                 indices.data(),
                 GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    // Create shader
    shader = std::make_unique<Shader>();

    // Cloud vertex shader
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;

        uniform mat4 uView;
        uniform mat4 uProjection;

        out vec3 WorldPos;
        out vec3 LocalPos;

        void main() {
            LocalPos = aPos;
            WorldPos = aPos;

            // Remove translation from view matrix (billboard effect)
            mat4 viewNoTranslation = uView;
            viewNoTranslation[3][0] = 0.0;
            viewNoTranslation[3][1] = 0.0;
            viewNoTranslation[3][2] = 0.0;

            vec4 pos = uProjection * viewNoTranslation * vec4(aPos, 1.0);
            gl_Position = pos.xyww;  // Put at far plane
        }
    )";

    // Cloud fragment shader with procedural noise
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec3 WorldPos;
        in vec3 LocalPos;

        uniform vec3 uCloudColor;
        uniform float uDensity;
        uniform float uWindOffset;

        out vec4 FragColor;

        // Simple 3D noise function
        float hash(vec3 p) {
            p = fract(p * vec3(0.1031, 0.1030, 0.0973));
            p += dot(p, p.yxz + 19.19);
            return fract((p.x + p.y) * p.z);
        }

        float noise(vec3 p) {
            vec3 i = floor(p);
            vec3 f = fract(p);
            f = f * f * (3.0 - 2.0 * f);

            return mix(
                mix(mix(hash(i + vec3(0,0,0)), hash(i + vec3(1,0,0)), f.x),
                    mix(hash(i + vec3(0,1,0)), hash(i + vec3(1,1,0)), f.x), f.y),
                mix(mix(hash(i + vec3(0,0,1)), hash(i + vec3(1,0,1)), f.x),
                    mix(hash(i + vec3(0,1,1)), hash(i + vec3(1,1,1)), f.x), f.y),
                f.z);
        }

        // Fractal Brownian Motion for cloud-like patterns
        float fbm(vec3 p) {
            float value = 0.0;
            float amplitude = 0.5;
            float frequency = 1.0;

            for (int i = 0; i < 4; i++) {
                value += amplitude * noise(p * frequency);
                frequency *= 2.0;
                amplitude *= 0.5;
            }

            return value;
        }

        void main() {
            // Normalize position for noise sampling
            vec3 pos = normalize(LocalPos);

            // Only render on upper hemisphere
            if (pos.y < 0.1) {
                discard;
            }

            // Apply wind offset to x coordinate
            vec3 samplePos = vec3(pos.x + uWindOffset, pos.y, pos.z) * 3.0;

            // Generate two cloud layers
            float clouds1 = fbm(samplePos * 1.0);
            float clouds2 = fbm(samplePos * 2.0 + vec3(100.0));

            // Combine layers
            float cloudPattern = clouds1 * 0.6 + clouds2 * 0.4;

            // Apply density threshold to create cloud shapes
            float cloudMask = smoothstep(0.4 + (1.0 - uDensity) * 0.3, 0.7, cloudPattern);

            // Add some variation to cloud edges
            float edgeNoise = noise(samplePos * 5.0);
            cloudMask *= smoothstep(0.3, 0.7, edgeNoise);

            // Fade clouds near horizon
            float horizonFade = smoothstep(0.0, 0.3, pos.y);
            cloudMask *= horizonFade;

            // Final alpha
            float alpha = cloudMask * 0.85;

            if (alpha < 0.05) {
                discard;
            }

            FragColor = vec4(uCloudColor, alpha);
        }
    )";

    if (!shader->loadFromSource(vertexShaderSource, fragmentShaderSource)) {
        LOG_ERROR("Failed to create cloud shader");
        return false;
    }

    LOG_INFO("Cloud system initialized: ", triangleCount, " triangles");
    return true;
}

void Clouds::generateMesh() {
    vertices.clear();
    indices.clear();

    // Generate hemisphere mesh for clouds
    for (int ring = 0; ring <= RINGS; ++ring) {
        float phi = (ring / static_cast<float>(RINGS)) * (M_PI * 0.5f);  // 0 to π/2
        float y = RADIUS * cosf(phi);
        float ringRadius = RADIUS * sinf(phi);

        for (int segment = 0; segment <= SEGMENTS; ++segment) {
            float theta = (segment / static_cast<float>(SEGMENTS)) * (2.0f * M_PI);
            float x = ringRadius * cosf(theta);
            float z = ringRadius * sinf(theta);

            vertices.push_back(glm::vec3(x, y, z));
        }
    }

    // Generate indices
    for (int ring = 0; ring < RINGS; ++ring) {
        for (int segment = 0; segment < SEGMENTS; ++segment) {
            int current = ring * (SEGMENTS + 1) + segment;
            int next = current + SEGMENTS + 1;

            // Two triangles per quad
            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    triangleCount = static_cast<int>(indices.size()) / 3;
}

void Clouds::update(float deltaTime) {
    if (!enabled) {
        return;
    }

    // Accumulate wind movement
    windOffset += deltaTime * windSpeed * 0.05f;  // Slow drift
}

glm::vec3 Clouds::getCloudColor(float timeOfDay) const {
    // Base cloud color (white/light gray)
    glm::vec3 dayColor(0.95f, 0.95f, 1.0f);

    // Dawn clouds (orange tint)
    if (timeOfDay >= 5.0f && timeOfDay < 7.0f) {
        float t = (timeOfDay - 5.0f) / 2.0f;
        glm::vec3 dawnColor(1.0f, 0.7f, 0.5f);
        return glm::mix(dawnColor, dayColor, t);
    }
    // Dusk clouds (orange/pink tint)
    else if (timeOfDay >= 17.0f && timeOfDay < 19.0f) {
        float t = (timeOfDay - 17.0f) / 2.0f;
        glm::vec3 duskColor(1.0f, 0.6f, 0.4f);
        return glm::mix(dayColor, duskColor, t);
    }
    // Night clouds (dark blue-gray)
    else if (timeOfDay >= 20.0f || timeOfDay < 5.0f) {
        return glm::vec3(0.15f, 0.15f, 0.25f);
    }

    return dayColor;
}

void Clouds::render(const Camera& camera, float timeOfDay) {
    if (!enabled || !shader) {
        return;
    }

    // Enable blending for transparent clouds
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Disable depth write (clouds are in sky)
    glDepthMask(GL_FALSE);

    // Enable depth test so clouds are behind skybox
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    shader->use();

    // Set matrices
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();

    shader->setUniform("uView", view);
    shader->setUniform("uProjection", projection);

    // Set cloud parameters
    glm::vec3 cloudColor = getCloudColor(timeOfDay);
    shader->setUniform("uCloudColor", cloudColor);
    shader->setUniform("uDensity", density);
    shader->setUniform("uWindOffset", windOffset);

    // Render
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // Restore state
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
}

void Clouds::setDensity(float density) {
    this->density = glm::clamp(density, 0.0f, 1.0f);
}

void Clouds::cleanup() {
    if (vao) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    if (vbo) {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (ebo) {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
}

} // namespace rendering
} // namespace wowee
