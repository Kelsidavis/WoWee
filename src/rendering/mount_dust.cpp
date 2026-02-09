#include "rendering/mount_dust.hpp"
#include "rendering/camera.hpp"
#include "rendering/shader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <random>
#include <cmath>

namespace wowee {
namespace rendering {

static std::mt19937& rng() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}

static float randFloat(float lo, float hi) {
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(rng());
}

MountDust::MountDust() = default;
MountDust::~MountDust() { shutdown(); }

bool MountDust::initialize() {
    LOG_INFO("Initializing mount dust effects");

    // Dust particle shader (brownish/tan dust clouds)
    shader = std::make_unique<Shader>();

    const char* dustVS = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in float aSize;
        layout (location = 2) in float aAlpha;

        uniform mat4 uView;
        uniform mat4 uProjection;

        out float vAlpha;

        void main() {
            gl_Position = uProjection * uView * vec4(aPos, 1.0);
            gl_PointSize = aSize;
            vAlpha = aAlpha;
        }
    )";

    const char* dustFS = R"(
        #version 330 core
        in float vAlpha;
        out vec4 FragColor;

        void main() {
            vec2 coord = gl_PointCoord - vec2(0.5);
            float dist = length(coord);
            if (dist > 0.5) discard;
            // Soft dust cloud with brownish/tan color
            float alpha = smoothstep(0.5, 0.0, dist) * vAlpha;
            vec3 dustColor = vec3(0.7, 0.65, 0.55);  // Tan/brown dust
            FragColor = vec4(dustColor, alpha * 0.4);  // Semi-transparent
        }
    )";

    if (!shader->loadFromSource(dustVS, dustFS)) {
        LOG_ERROR("Failed to create mount dust shader");
        return false;
    }

    // Create VAO/VBO
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Position (vec3) + Size (float) + Alpha (float) = 5 floats per vertex
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    particles.reserve(MAX_DUST_PARTICLES);
    vertexData.reserve(MAX_DUST_PARTICLES * 5);

    LOG_INFO("Mount dust effects initialized");
    return true;
}

void MountDust::shutdown() {
    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
    vao = 0;
    vbo = 0;
    particles.clear();
    shader.reset();
}

void MountDust::spawnDust(const glm::vec3& position, const glm::vec3& velocity, bool isMoving) {
    if (!isMoving) {
        spawnAccum = 0.0f;
        return;
    }

    // Spawn rate based on speed
    float speed = glm::length(velocity);
    if (speed < 0.1f) return;

    // Spawn dust particles at a rate proportional to speed
    float spawnRate = speed * 8.0f;  // More dust at higher speeds
    spawnAccum += spawnRate * 0.016f;  // Assume ~60 FPS

    while (spawnAccum >= 1.0f && particles.size() < MAX_DUST_PARTICLES) {
        spawnAccum -= 1.0f;

        Particle p;
        // Spawn slightly behind and to the sides of the mount
        p.position = position + glm::vec3(
            randFloat(-0.3f, 0.3f),
            randFloat(-0.3f, 0.3f),
            randFloat(-0.1f, 0.1f)
        );

        // Dust rises up and drifts backward slightly
        p.velocity = glm::vec3(
            randFloat(-0.2f, 0.2f),
            randFloat(-0.2f, 0.2f),
            randFloat(0.5f, 1.2f)  // Rise upward
        ) - velocity * 0.2f;  // Drift backward relative to movement

        p.lifetime = 0.0f;
        p.maxLifetime = randFloat(0.4f, 0.8f);
        p.size = randFloat(8.0f, 16.0f);
        p.alpha = 1.0f;

        particles.push_back(p);
    }
}

void MountDust::update(float deltaTime) {
    // Update existing particles
    for (auto it = particles.begin(); it != particles.end(); ) {
        it->lifetime += deltaTime;

        if (it->lifetime >= it->maxLifetime) {
            it = particles.erase(it);
            continue;
        }

        // Update position
        it->position += it->velocity * deltaTime;

        // Slow down velocity (friction)
        it->velocity *= 0.96f;

        // Fade out
        float t = it->lifetime / it->maxLifetime;
        it->alpha = 1.0f - t;

        // Grow slightly as they fade
        it->size += deltaTime * 12.0f;

        ++it;
    }
}

void MountDust::render(const Camera& camera) {
    if (particles.empty() || !shader) return;

    // Build vertex data
    vertexData.clear();
    for (const auto& p : particles) {
        vertexData.push_back(p.position.x);
        vertexData.push_back(p.position.y);
        vertexData.push_back(p.position.z);
        vertexData.push_back(p.size);
        vertexData.push_back(p.alpha);
    }

    // Upload to GPU
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_DYNAMIC_DRAW);

    // Render
    shader->use();
    shader->setUniform("uView", camera.getViewMatrix());
    shader->setUniform("uProjection", camera.getProjectionMatrix());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);  // Don't write to depth buffer
    glEnable(GL_PROGRAM_POINT_SIZE);

    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(particles.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_PROGRAM_POINT_SIZE);
}

} // namespace rendering
} // namespace wowee
