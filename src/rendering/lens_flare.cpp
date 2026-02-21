#include "rendering/lens_flare.hpp"
#include "rendering/camera.hpp"
#include "rendering/shader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>

namespace wowee {
namespace rendering {

LensFlare::LensFlare() {
}

LensFlare::~LensFlare() {
    cleanup();
}

bool LensFlare::initialize() {
    LOG_INFO("Initializing lens flare system");

    // Generate flare elements
    generateFlareElements();

    // Create VAO and VBO for quad rendering
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Position (x, y) and UV (u, v) for a quad
    float quadVertices[] = {
        // Pos      UV
        -0.5f, -0.5f, 0.0f, 0.0f,
         0.5f, -0.5f, 1.0f, 0.0f,
         0.5f,  0.5f, 1.0f, 1.0f,
        -0.5f, -0.5f, 0.0f, 0.0f,
         0.5f,  0.5f, 1.0f, 1.0f,
        -0.5f,  0.5f, 0.0f, 1.0f
    };

    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // UV attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    // Create shader
    shader = std::make_unique<Shader>();

    // Lens flare vertex shader (2D screen-space rendering)
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aUV;

        uniform vec2 uPosition;  // Screen-space position (-1 to 1)
        uniform float uSize;      // Size in screen space
        uniform float uAspectRatio;

        out vec2 TexCoord;

        void main() {
            // Scale by size and aspect ratio
            vec2 scaledPos = aPos * uSize;
            scaledPos.x /= uAspectRatio;

            // Translate to position
            vec2 finalPos = scaledPos + uPosition;

            gl_Position = vec4(finalPos, 0.0, 1.0);
            TexCoord = aUV;
        }
    )";

    // Lens flare fragment shader (circular gradient)
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec2 TexCoord;

        uniform vec3 uColor;
        uniform float uBrightness;

        out vec4 FragColor;

        void main() {
            // Distance from center
            vec2 center = vec2(0.5);
            float dist = distance(TexCoord, center);

            // Circular gradient with soft edges
            float alpha = smoothstep(0.5, 0.0, dist);

            // Add some variation - brighter in center
            float centerGlow = smoothstep(0.5, 0.0, dist * 2.0);
            alpha = max(alpha * 0.3, centerGlow);

            // Apply brightness
            alpha *= uBrightness;

            if (alpha < 0.01) {
                discard;
            }

            FragColor = vec4(uColor, alpha);
        }
    )";

    if (!shader->loadFromSource(vertexShaderSource, fragmentShaderSource)) {
        LOG_ERROR("Failed to create lens flare shader");
        return false;
    }

    LOG_INFO("Lens flare system initialized: ", flareElements.size(), " elements");
    return true;
}

void LensFlare::generateFlareElements() {
    flareElements.clear();

    // Main sun glow (at sun position)
    flareElements.push_back({0.0f, 0.3f, glm::vec3(1.0f, 0.95f, 0.8f), 0.8f});

    // Flare ghosts along sun-to-center axis
    // These appear at various positions between sun and opposite side

    // Bright white ghost near sun
    flareElements.push_back({0.2f, 0.08f, glm::vec3(1.0f, 1.0f, 1.0f), 0.5f});

    // Blue-tinted ghost
    flareElements.push_back({0.4f, 0.15f, glm::vec3(0.3f, 0.5f, 1.0f), 0.4f});

    // Small bright spot
    flareElements.push_back({0.6f, 0.05f, glm::vec3(1.0f, 0.8f, 0.6f), 0.6f});

    // Green-tinted ghost (chromatic aberration)
    flareElements.push_back({0.8f, 0.12f, glm::vec3(0.4f, 1.0f, 0.5f), 0.3f});

    // Large halo on opposite side
    flareElements.push_back({-0.5f, 0.25f, glm::vec3(1.0f, 0.7f, 0.4f), 0.2f});

    // Purple ghost far from sun
    flareElements.push_back({-0.8f, 0.1f, glm::vec3(0.8f, 0.4f, 1.0f), 0.25f});

    // Small red ghost
    flareElements.push_back({-1.2f, 0.06f, glm::vec3(1.0f, 0.3f, 0.3f), 0.3f});
}

glm::vec2 LensFlare::worldToScreen(const Camera& camera, const glm::vec3& worldPos) const {
    // Transform to clip space
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();
    glm::mat4 viewProj = projection * view;

    glm::vec4 clipPos = viewProj * glm::vec4(worldPos, 1.0f);

    // Perspective divide
    if (clipPos.w > 0.0f) {
        glm::vec2 ndc = glm::vec2(clipPos.x / clipPos.w, clipPos.y / clipPos.w);
        return ndc;
    }

    // Behind camera
    return glm::vec2(10.0f, 10.0f);  // Off-screen
}

float LensFlare::calculateSunVisibility(const Camera& camera, const glm::vec3& sunPosition) const {
    // Get sun position in screen space
    glm::vec2 sunScreen = worldToScreen(camera, sunPosition);

    // Check if sun is behind camera
    glm::vec3 camPos = camera.getPosition();
    glm::vec3 camForward = camera.getForward();
    glm::vec3 toSun = glm::normalize(sunPosition - camPos);
    float dotProduct = glm::dot(camForward, toSun);

    if (dotProduct < 0.0f) {
        return 0.0f;  // Sun is behind camera
    }

    // Check if sun is outside screen bounds (with some margin)
    if (std::abs(sunScreen.x) > 1.5f || std::abs(sunScreen.y) > 1.5f) {
        return 0.0f;
    }

    // Fade based on angle (stronger when looking directly at sun)
    float angleFactor = glm::smoothstep(0.3f, 1.0f, dotProduct);

    // Fade at screen edges
    float edgeFade = 1.0f;
    if (std::abs(sunScreen.x) > 0.8f) {
        edgeFade *= glm::smoothstep(1.2f, 0.8f, std::abs(sunScreen.x));
    }
    if (std::abs(sunScreen.y) > 0.8f) {
        edgeFade *= glm::smoothstep(1.2f, 0.8f, std::abs(sunScreen.y));
    }

    return angleFactor * edgeFade;
}

void LensFlare::render(const Camera& camera, const glm::vec3& sunPosition, float timeOfDay) {
    if (!enabled || !shader) {
        return;
    }

    // Only render lens flare during daytime (when sun is visible)
    if (timeOfDay < 5.0f || timeOfDay > 19.0f) {
        return;
    }

    // Sun billboard rendering is sky-locked (view translation removed), so anchor
    // flare projection to camera position along sun direction to avoid parallax drift.
    glm::vec3 sunDir = sunPosition;
    if (glm::length(sunDir) < 0.0001f) {
        return;
    }
    sunDir = glm::normalize(sunDir);
    glm::vec3 anchoredSunPos = camera.getPosition() + sunDir * 800.0f;

    // Calculate sun visibility
    float visibility = calculateSunVisibility(camera, anchoredSunPos);
    if (visibility < 0.01f) {
        return;
    }

    // Get sun screen position
    glm::vec2 sunScreen = worldToScreen(camera, anchoredSunPos);
    glm::vec2 screenCenter(0.0f, 0.0f);

    // Vector from sun to screen center
    glm::vec2 sunToCenter = screenCenter - sunScreen;

    // Enable additive blending for flare effect
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // Additive blending

    // Disable depth test (render on top)
    glDisable(GL_DEPTH_TEST);

    shader->use();

    // Set aspect ratio
    float aspectRatio = camera.getAspectRatio();
    shader->setUniform("uAspectRatio", aspectRatio);

    glBindVertexArray(vao);

    // Render each flare element
    for (const auto& element : flareElements) {
        // Calculate position along sun-to-center axis
        glm::vec2 position = sunScreen + sunToCenter * element.position;

        // Set uniforms
        shader->setUniform("uPosition", position);
        shader->setUniform("uSize", element.size);
        shader->setUniform("uColor", element.color);

        // Apply visibility and intensity
        float brightness = element.brightness * visibility * intensityMultiplier;
        shader->setUniform("uBrightness", brightness);

        // Render quad
        glDrawArrays(GL_TRIANGLES, 0, VERTICES_PER_QUAD);
    }

    glBindVertexArray(0);

    // Restore state
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Restore standard blending
}

void LensFlare::setIntensity(float intensity) {
    this->intensityMultiplier = glm::clamp(intensity, 0.0f, 2.0f);
}

void LensFlare::cleanup() {
    if (vao) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    if (vbo) {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
}

} // namespace rendering
} // namespace wowee
