#include "rendering/celestial.hpp"
#include "rendering/shader.hpp"
#include "rendering/camera.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

namespace wowee {
namespace rendering {

Celestial::Celestial() = default;

Celestial::~Celestial() {
    shutdown();
}

bool Celestial::initialize() {
    LOG_INFO("Initializing celestial renderer");

    // Create celestial shader
    celestialShader = std::make_unique<Shader>();

    // Vertex shader - billboard facing camera (sky dome locked)
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTexCoord;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        out vec2 TexCoord;

        void main() {
            TexCoord = aTexCoord;

            // Sky object: remove translation, keep rotation (skybox technique)
            mat4 viewNoTranslation = mat4(mat3(view));

            gl_Position = projection * viewNoTranslation * model * vec4(aPos, 1.0);
        }
    )";

    // Fragment shader - disc with glow and moon phase support
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec2 TexCoord;

        uniform vec3 celestialColor;
        uniform float intensity;
        uniform float moonPhase;  // 0.0 = new moon, 0.5 = full moon, 1.0 = new moon
        uniform float uAnimTime;

        out vec4 FragColor;

        float hash(vec2 p) {
            return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
        }

        float noise(vec2 p) {
            vec2 i = floor(p);
            vec2 f = fract(p);
            f = f * f * (3.0 - 2.0 * f);
            float a = hash(i + vec2(0.0, 0.0));
            float b = hash(i + vec2(1.0, 0.0));
            float c = hash(i + vec2(0.0, 1.0));
            float d = hash(i + vec2(1.0, 1.0));
            return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
        }

        void main() {
            // Create circular disc
            vec2 center = vec2(0.5, 0.5);
            float dist = distance(TexCoord, center);

            // Core disc + glow with explicit radial mask to avoid square billboard artifact.
            float disc = smoothstep(0.50, 0.38, dist);
            float glow = smoothstep(0.64, 0.00, dist) * 0.24;
            float radialMask = 1.0 - smoothstep(0.58, 0.70, dist);

            float alpha = (disc + glow) * radialMask * intensity;
            vec3 outColor = celestialColor;

            // Very faint animated haze over sun disc/glow (no effect for moon).
            if (intensity > 0.5) {
                vec2 uv = (TexCoord - vec2(0.5)) * 3.0;
                // Slow flow field for atmospheric-like turbulence drift.
                vec2 flow = vec2(
                    noise(uv * 0.9 + vec2(uAnimTime * 0.012, -uAnimTime * 0.009)),
                    noise(uv * 0.9 + vec2(-uAnimTime * 0.010, uAnimTime * 0.011))
                ) - vec2(0.5);
                vec2 warped = uv + flow * 0.42;
                float n1 = noise(warped * 1.7 + vec2(uAnimTime * 0.016, -uAnimTime * 0.013));
                float n2 = noise(warped * 3.0 + vec2(-uAnimTime * 0.021, uAnimTime * 0.017));
                float haze = mix(n1, n2, 0.35);
                float hazeMask = clamp(disc * 0.75 + glow * 0.28, 0.0, 1.0);
                float hazeMix = hazeMask * 0.55;
                float lumaMod = mix(1.0, 0.93 + haze * 0.10, hazeMix);
                outColor *= lumaMod;
                alpha *= mix(1.0, 0.94 + haze * 0.06, hazeMix);
            }

            // Apply moon phase shadow (only for moon, indicated by low intensity)
            if (intensity < 0.5) {  // Moon has lower intensity than sun
                // Calculate phase position (-1 to 1, where 0 is center)
                float phasePos = (moonPhase - 0.5) * 2.0;

                // Distance from phase terminator line
                float x = (TexCoord.x - 0.5) * 2.0;  // -1 to 1

                // Create shadow using smoothstep
                float shadow = 1.0;

                if (moonPhase < 0.5) {
                    // Waning (right to left shadow)
                    shadow = smoothstep(phasePos - 0.1, phasePos + 0.1, x);
                } else {
                    // Waxing (left to right shadow)
                    shadow = smoothstep(phasePos - 0.1, phasePos + 0.1, -x);
                }

                // Apply elliptical terminator for 3D effect
                float y = (TexCoord.y - 0.5) * 2.0;
                float ellipse = sqrt(max(0.0, 1.0 - y * y));
                float terminatorX = phasePos / ellipse;

                if (moonPhase < 0.5) {
                    shadow = smoothstep(terminatorX - 0.15, terminatorX + 0.15, x);
                } else {
                    shadow = smoothstep(terminatorX - 0.15, terminatorX + 0.15, -x);
                }

                // Darken shadowed area (not completely black, slight glow remains)
                alpha *= mix(0.05, 1.0, shadow);
            }

            if (alpha < 0.01) {
                discard;
            }

            FragColor = vec4(outColor, alpha);
        }
    )";

    if (!celestialShader->loadFromSource(vertexShaderSource, fragmentShaderSource)) {
        LOG_ERROR("Failed to create celestial shader");
        return false;
    }

    // Create billboard quad
    createCelestialQuad();

    LOG_INFO("Celestial renderer initialized");
    return true;
}

void Celestial::shutdown() {
    destroyCelestialQuad();
    celestialShader.reset();
}

void Celestial::render(const Camera& camera, float timeOfDay,
                       const glm::vec3* sunDir, const glm::vec3* sunColor, float gameTime) {
    if (!renderingEnabled || vao == 0 || !celestialShader) {
        return;
    }

    // Update moon phases from game time if available (deterministic)
    if (gameTime >= 0.0f) {
        updatePhasesFromGameTime(gameTime);
    }

    // Enable additive blending for celestial glow (brighter against sky)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // Additive blending for brightness

    // Disable depth testing entirely - celestial bodies render "on" the sky
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    // Disable culling - billboards can face either way
    glDisable(GL_CULL_FACE);

    // Render sun with alpha blending (avoids additive white clipping).
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    renderSun(camera, timeOfDay, sunDir, sunColor);

    // Render moons additively for glow.
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    renderMoon(camera, timeOfDay);  // White Lady (primary moon)

    if (dualMoonMode_) {
        renderBlueChild(camera, timeOfDay);  // Blue Child (secondary moon)
    }

    // Restore state
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}

void Celestial::renderSun(const Camera& camera, float timeOfDay,
                          const glm::vec3* sunDir, const glm::vec3* sunColor) {
    // Sun visible from 5:00 to 19:00
    if (timeOfDay < 5.0f || timeOfDay >= 19.0f) {
        return;
    }

    celestialShader->use();

    glm::vec3 dir = sunDir ? glm::normalize(*sunDir) : glm::vec3(0.0f, 0.0f, 1.0f);

    // Place sun on sky sphere at fixed distance
    const float sunDistance = 800.0f;
    glm::vec3 sunPos = dir * sunDistance;

    // Create model matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, sunPos);
    model = glm::scale(model, glm::vec3(95.0f, 95.0f, 1.0f));  // Match WotLK-like apparent size

    // Set uniforms
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();

    celestialShader->setUniform("model", model);
    celestialShader->setUniform("view", view);
    celestialShader->setUniform("projection", projection);

    // Sun color and intensity (use lighting color if provided)
    glm::vec3 color = sunColor ? *sunColor : getSunColor(timeOfDay);
    // Force strong warm/yellow tint; avoid white blowout.
    const glm::vec3 warmSun(1.0f, 0.88f, 0.55f);
    color = glm::mix(color, warmSun, 0.52f);
    float intensity = getSunIntensity(timeOfDay) * 0.92f;

    celestialShader->setUniform("celestialColor", color);
    celestialShader->setUniform("intensity", intensity);
    celestialShader->setUniform("moonPhase", 0.5f);  // Sun doesn't use this, but shader expects it
    celestialShader->setUniform("uAnimTime", sunHazeTimer_);

    // Render quad
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Celestial::renderMoon(const Camera& camera, float timeOfDay) {
    // Moon visible from 19:00 to 5:00 (night)
    if (timeOfDay >= 5.0f && timeOfDay < 19.0f) {
        return;
    }

    celestialShader->use();

    // Get moon position
    glm::vec3 moonPos = getMoonPosition(timeOfDay);

    // Create model matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, moonPos);
    model = glm::scale(model, glm::vec3(40.0f, 40.0f, 1.0f));  // 40 unit diameter (smaller than sun)

    // Set uniforms
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();

    celestialShader->setUniform("model", model);
    celestialShader->setUniform("view", view);
    celestialShader->setUniform("projection", projection);

    // Moon color (pale blue-white) and intensity
    glm::vec3 color = glm::vec3(0.8f, 0.85f, 1.0f);

    // Fade in/out at transitions
    float intensity = 1.0f;
    if (timeOfDay >= 19.0f && timeOfDay < 21.0f) {
        // Fade in (19:00-21:00)
        intensity = (timeOfDay - 19.0f) / 2.0f;
    }
    else if (timeOfDay >= 3.0f && timeOfDay < 5.0f) {
        // Fade out (3:00-5:00)
        intensity = 1.0f - (timeOfDay - 3.0f) / 2.0f;
    }

    celestialShader->setUniform("celestialColor", color);
    celestialShader->setUniform("intensity", intensity);
    celestialShader->setUniform("moonPhase", whiteLadyPhase_);
    celestialShader->setUniform("uAnimTime", sunHazeTimer_);

    // Render quad
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Celestial::renderBlueChild(const Camera& camera, float timeOfDay) {
    // Blue Child visible from 19:00 to 5:00 (night, same as White Lady)
    if (timeOfDay >= 5.0f && timeOfDay < 19.0f) {
        return;
    }

    celestialShader->use();

    // Get moon position (offset slightly from White Lady)
    glm::vec3 moonPos = getMoonPosition(timeOfDay);
    // Offset Blue Child to the right and slightly lower
    moonPos.x += 80.0f;   // Right offset
    moonPos.z -= 40.0f;   // Slightly lower

    // Create model matrix (smaller than White Lady)
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, moonPos);
    model = glm::scale(model, glm::vec3(30.0f, 30.0f, 1.0f));  // 30 unit diameter (smaller)

    // Set uniforms
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();

    celestialShader->setUniform("model", model);
    celestialShader->setUniform("view", view);
    celestialShader->setUniform("projection", projection);

    // Blue Child color (pale blue tint)
    glm::vec3 color = glm::vec3(0.7f, 0.8f, 1.0f);

    // Fade in/out at transitions (same as White Lady)
    float intensity = 1.0f;
    if (timeOfDay >= 19.0f && timeOfDay < 21.0f) {
        // Fade in (19:00-21:00)
        intensity = (timeOfDay - 19.0f) / 2.0f;
    }
    else if (timeOfDay >= 3.0f && timeOfDay < 5.0f) {
        // Fade out (3:00-5:00)
        intensity = 1.0f - (timeOfDay - 3.0f) / 2.0f;
    }

    // Blue Child is dimmer than White Lady
    intensity *= 0.7f;

    celestialShader->setUniform("celestialColor", color);
    celestialShader->setUniform("intensity", intensity);
    celestialShader->setUniform("moonPhase", blueChildPhase_);
    celestialShader->setUniform("uAnimTime", sunHazeTimer_);

    // Render quad
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

glm::vec3 Celestial::getSunPosition(float timeOfDay) const {
    // Sun rises at 6:00, peaks at 12:00, sets at 18:00
    float angle = calculateCelestialAngle(timeOfDay, 6.0f, 18.0f);

    const float radius = 800.0f;  // Horizontal distance
    const float height = 600.0f;  // Maximum height at zenith

    // Arc across sky (angle 0→π maps to sunrise→noon→sunset)
    // Z is vertical (matches skybox: Altitude = aPos.z)
    // At angle=0: x=radius, z=0 (east horizon)
    // At angle=π/2: x=0, z=height (zenith, directly overhead)
    // At angle=π: x=-radius, z=0 (west horizon)
    float x = radius * std::cos(angle);  // Horizontal position (E→W)
    float y = 0.0f;  // Y is north-south (keep at 0)
    float z = height * std::sin(angle);  // Vertical position (Z is UP, matches skybox)

    return glm::vec3(x, y, z);
}

glm::vec3 Celestial::getMoonPosition(float timeOfDay) const {
    // Moon rises at 18:00, peaks at 0:00 (24:00), sets at 6:00
    // Adjust time for moon (opposite to sun)
    float moonTime = timeOfDay + 12.0f;
    if (moonTime >= 24.0f) moonTime -= 24.0f;

    float angle = calculateCelestialAngle(moonTime, 6.0f, 18.0f);

    const float radius = 800.0f;
    const float height = 600.0f;

    // Same arc formula as sun (Z is vertical, matches skybox)
    float x = radius * std::cos(angle);
    float y = 0.0f;
    float z = height * std::sin(angle);

    return glm::vec3(x, y, z);
}

glm::vec3 Celestial::getSunColor(float timeOfDay) const {
    // Sunrise/sunset: orange/red
    // Midday: bright yellow-white

    if (timeOfDay >= 5.0f && timeOfDay < 7.0f) {
        // Sunrise: orange
        return glm::vec3(1.0f, 0.6f, 0.2f);
    }
    else if (timeOfDay >= 7.0f && timeOfDay < 9.0f) {
        // Morning: blend to yellow
        float t = (timeOfDay - 7.0f) / 2.0f;
        glm::vec3 orange = glm::vec3(1.0f, 0.6f, 0.2f);
        glm::vec3 yellow = glm::vec3(1.0f, 1.0f, 0.9f);
        return glm::mix(orange, yellow, t);
    }
    else if (timeOfDay >= 9.0f && timeOfDay < 16.0f) {
        // Day: bright yellow-white
        return glm::vec3(1.0f, 1.0f, 0.9f);
    }
    else if (timeOfDay >= 16.0f && timeOfDay < 18.0f) {
        // Evening: blend to orange
        float t = (timeOfDay - 16.0f) / 2.0f;
        glm::vec3 yellow = glm::vec3(1.0f, 1.0f, 0.9f);
        glm::vec3 orange = glm::vec3(1.0f, 0.5f, 0.1f);
        return glm::mix(yellow, orange, t);
    }
    else {
        // Sunset: deep orange/red
        return glm::vec3(1.0f, 0.4f, 0.1f);
    }
}

float Celestial::getSunIntensity(float timeOfDay) const {
    // Fade in at sunrise (5:00-6:00)
    if (timeOfDay >= 5.0f && timeOfDay < 6.0f) {
        return (timeOfDay - 5.0f);  // 0 to 1
    }
    // Full intensity during day (6:00-18:00)
    else if (timeOfDay >= 6.0f && timeOfDay < 18.0f) {
        return 1.0f;
    }
    // Fade out at sunset (18:00-19:00)
    else if (timeOfDay >= 18.0f && timeOfDay < 19.0f) {
        return 1.0f - (timeOfDay - 18.0f);  // 1 to 0
    }
    else {
        return 0.0f;
    }
}

float Celestial::calculateCelestialAngle(float timeOfDay, float riseTime, float setTime) const {
    // Map time to angle (0 to PI)
    // riseTime: 0 radians (horizon east)
    // (riseTime + setTime) / 2: PI/2 radians (zenith)
    // setTime: PI radians (horizon west)

    float duration = setTime - riseTime;
    float elapsed = timeOfDay - riseTime;

    // Normalize to 0-1
    float t = elapsed / duration;

    // Map to 0 to PI (arc from east to west)
    return t * M_PI;
}

void Celestial::createCelestialQuad() {
    // Simple quad centered at origin
    float vertices[] = {
        // Position         // TexCoord
        -0.5f,  0.5f, 0.0f,  0.0f, 1.0f,  // Top-left
         0.5f,  0.5f, 0.0f,  1.0f, 1.0f,  // Top-right
         0.5f, -0.5f, 0.0f,  1.0f, 0.0f,  // Bottom-right
        -0.5f, -0.5f, 0.0f,  0.0f, 0.0f   // Bottom-left
    };

    uint32_t indices[] = {
        0, 1, 2,  // First triangle
        0, 2, 3   // Second triangle
    };

    // Create OpenGL buffers
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Set vertex attributes
    // Position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture coordinates
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void Celestial::destroyCelestialQuad() {
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (ebo != 0) {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
}

void Celestial::update(float deltaTime) {
    sunHazeTimer_ += deltaTime;

    if (!moonPhaseCycling) {
        return;
    }

    // Update moon phase timer
    moonPhaseTimer += deltaTime;

    // White Lady completes full cycle in MOON_CYCLE_DURATION seconds
    whiteLadyPhase_ = std::fmod(moonPhaseTimer / MOON_CYCLE_DURATION, 1.0f);

    // Blue Child has a different cycle rate (slightly faster, 3.5 minutes)
    constexpr float BLUE_CHILD_CYCLE = 210.0f;
    blueChildPhase_ = std::fmod(moonPhaseTimer / BLUE_CHILD_CYCLE, 1.0f);
}

void Celestial::setMoonPhase(float phase) {
    // Set White Lady phase (primary moon)
    whiteLadyPhase_ = glm::clamp(phase, 0.0f, 1.0f);

    // Update timer to match White Lady phase
    moonPhaseTimer = whiteLadyPhase_ * MOON_CYCLE_DURATION;
}

void Celestial::setBlueChildPhase(float phase) {
    // Set Blue Child phase (secondary moon)
    blueChildPhase_ = glm::clamp(phase, 0.0f, 1.0f);
}

float Celestial::computePhaseFromGameTime(float gameTime, float cycleDays) const {
    // WoW game time: 1 game day = 24 real minutes = 1440 seconds
    constexpr float SECONDS_PER_GAME_DAY = 1440.0f;

    // Convert game time to game days
    float gameDays = gameTime / SECONDS_PER_GAME_DAY;

    // Compute phase as fraction of lunar cycle (0.0-1.0)
    float phase = std::fmod(gameDays / cycleDays, 1.0f);

    // Ensure positive (fmod can return negative for negative input)
    if (phase < 0.0f) {
        phase += 1.0f;
    }

    return phase;
}

void Celestial::updatePhasesFromGameTime(float gameTime) {
    // Compute deterministic phases from server game time
    whiteLadyPhase_ = computePhaseFromGameTime(gameTime, WHITE_LADY_CYCLE_DAYS);
    blueChildPhase_ = computePhaseFromGameTime(gameTime, BLUE_CHILD_CYCLE_DAYS);
}

} // namespace rendering
} // namespace wowee
