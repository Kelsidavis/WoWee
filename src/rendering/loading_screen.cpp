#include "rendering/loading_screen.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>
#include <random>
#include <chrono>
#include <cstdio>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace wowee {
namespace rendering {

LoadingScreen::LoadingScreen() {
    imagePaths.push_back("assets/loading1.jpeg");
    imagePaths.push_back("assets/loading2.jpeg");
}

LoadingScreen::~LoadingScreen() {
    shutdown();
}

bool LoadingScreen::initialize() {
    LOG_INFO("Initializing loading screen");

    // Background image shader (textured quad)
    const char* vertexSrc = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        layout (location = 1) in vec2 aTexCoord;
        out vec2 TexCoord;
        void main() {
            gl_Position = vec4(aPos, 0.0, 1.0);
            TexCoord = aTexCoord;
        }
    )";

    const char* fragmentSrc = R"(
        #version 330 core
        in vec2 TexCoord;
        out vec4 FragColor;
        uniform sampler2D screenTexture;
        void main() {
            FragColor = texture(screenTexture, TexCoord);
        }
    )";

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexSrc, nullptr);
    glCompileShader(vertexShader);

    GLint success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        LOG_ERROR("Loading screen vertex shader compilation failed: ", infoLog);
        return false;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentSrc, nullptr);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        LOG_ERROR("Loading screen fragment shader compilation failed: ", infoLog);
        return false;
    }

    shaderId = glCreateProgram();
    glAttachShader(shaderId, vertexShader);
    glAttachShader(shaderId, fragmentShader);
    glLinkProgram(shaderId);

    glGetProgramiv(shaderId, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderId, 512, nullptr, infoLog);
        LOG_ERROR("Loading screen shader linking failed: ", infoLog);
        return false;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Simple solid-color shader for progress bar
    const char* barVertSrc = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        void main() {
            gl_Position = vec4(aPos, 0.0, 1.0);
        }
    )";

    const char* barFragSrc = R"(
        #version 330 core
        out vec4 FragColor;
        uniform vec4 uColor;
        void main() {
            FragColor = uColor;
        }
    )";

    GLuint bv = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(bv, 1, &barVertSrc, nullptr);
    glCompileShader(bv);
    GLuint bf = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(bf, 1, &barFragSrc, nullptr);
    glCompileShader(bf);

    barShaderId = glCreateProgram();
    glAttachShader(barShaderId, bv);
    glAttachShader(barShaderId, bf);
    glLinkProgram(barShaderId);

    glDeleteShader(bv);
    glDeleteShader(bf);

    createQuad();
    createBarQuad();
    selectRandomImage();

    LOG_INFO("Loading screen initialized");
    return true;
}

void LoadingScreen::shutdown() {
    if (textureId) {
        glDeleteTextures(1, &textureId);
        textureId = 0;
    }
    if (vao) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    if (vbo) {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (shaderId) {
        glDeleteProgram(shaderId);
        shaderId = 0;
    }
    if (barVao) {
        glDeleteVertexArrays(1, &barVao);
        barVao = 0;
    }
    if (barVbo) {
        glDeleteBuffers(1, &barVbo);
        barVbo = 0;
    }
    if (barShaderId) {
        glDeleteProgram(barShaderId);
        barShaderId = 0;
    }
}

void LoadingScreen::selectRandomImage() {
    if (imagePaths.empty()) return;

    unsigned seed = static_cast<unsigned>(
        std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(0, imagePaths.size() - 1);

    currentImageIndex = distribution(generator);
    LOG_INFO("Selected loading screen: ", imagePaths[currentImageIndex]);

    loadImage(imagePaths[currentImageIndex]);
}

bool LoadingScreen::loadImage(const std::string& path) {
    if (textureId) {
        glDeleteTextures(1, &textureId);
        textureId = 0;
    }

    int channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path.c_str(), &imageWidth, &imageHeight, &channels, 4);

    if (!data) {
        LOG_ERROR("Failed to load loading screen image: ", path);
        return false;
    }

    LOG_INFO("Loaded loading screen image: ", imageWidth, "x", imageHeight);

    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageWidth, imageHeight, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, data);

    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
}

void LoadingScreen::createQuad() {
    float vertices[] = {
        // Position    // TexCoord
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void LoadingScreen::createBarQuad() {
    // Dynamic quad — vertices updated each frame via glBufferSubData
    glGenVertexArrays(1, &barVao);
    glGenBuffers(1, &barVbo);

    glBindVertexArray(barVao);
    glBindBuffer(GL_ARRAY_BUFFER, barVbo);
    glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void LoadingScreen::render() {
    if (!vao || !shaderId) return;

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);

    // Draw background image
    if (textureId) {
        glUseProgram(shaderId);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureId);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
    }

    // Draw progress bar at bottom center
    if (barVao && barShaderId) {
        // Bar dimensions in NDC: centered, near bottom
        const float barWidth = 0.6f;   // half-width in NDC (total 1.2 of 2.0 range = 60% of screen)
        const float barHeight = 0.015f;
        const float barY = -0.82f;     // near bottom

        float left = -barWidth;
        float right = -barWidth + 2.0f * barWidth * loadProgress;
        float top = barY + barHeight;
        float bottom = barY - barHeight;

        // Background (dark)
        {
            float bgVerts[] = {
                -barWidth, top,
                -barWidth, bottom,
                 barWidth, bottom,
                -barWidth, top,
                 barWidth, bottom,
                 barWidth, top,
            };
            glUseProgram(barShaderId);
            GLint colorLoc = glGetUniformLocation(barShaderId, "uColor");
            glUniform4f(colorLoc, 0.1f, 0.1f, 0.1f, 0.8f);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            glBindVertexArray(barVao);
            glBindBuffer(GL_ARRAY_BUFFER, barVbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(bgVerts), bgVerts);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        // Filled portion (gold/amber like WoW)
        if (loadProgress > 0.001f) {
            float fillVerts[] = {
                left, top,
                left, bottom,
                right, bottom,
                left, top,
                right, bottom,
                right, top,
            };
            GLint colorLoc = glGetUniformLocation(barShaderId, "uColor");
            glUniform4f(colorLoc, 0.78f, 0.61f, 0.13f, 1.0f);

            glBindBuffer(GL_ARRAY_BUFFER, barVbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(fillVerts), fillVerts);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        // Border (thin bright outline)
        {
            const float borderInset = 0.002f;
            float borderLeft = -barWidth - borderInset;
            float borderRight = barWidth + borderInset;
            float borderTop = top + borderInset;
            float borderBottom = bottom - borderInset;

            // Draw 4 thin border edges as line strip
            glUseProgram(barShaderId);
            GLint colorLoc = glGetUniformLocation(barShaderId, "uColor");
            glUniform4f(colorLoc, 0.55f, 0.43f, 0.1f, 1.0f);

            float borderVerts[] = {
                borderLeft, borderTop,
                borderRight, borderTop,
                borderRight, borderBottom,
                borderLeft, borderBottom,
                borderLeft, borderTop,
            };
            glBindBuffer(GL_ARRAY_BUFFER, barVbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(borderVerts), borderVerts);
            glDrawArrays(GL_LINE_STRIP, 0, 5);
        }

        glBindVertexArray(0);
        glDisable(GL_BLEND);
    }

    // Draw status text and percentage with ImGui overlay
    {
        ImGuiIO& io = ImGui::GetIO();
        float screenW = io.DisplaySize.x;
        float screenH = io.DisplaySize.y;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Invisible fullscreen window for text overlay
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(screenW, screenH));
        ImGui::Begin("##LoadingOverlay", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground |
            ImGuiWindowFlags_NoBringToFrontOnFocus);

        // Percentage text centered above bar
        char pctBuf[32];
        snprintf(pctBuf, sizeof(pctBuf), "%d%%", static_cast<int>(loadProgress * 100.0f));

        float barCenterY = screenH * (1.0f - ((-0.82f + 1.0f) / 2.0f)); // NDC -0.82 to screen Y
        float textY = barCenterY - 30.0f;

        ImVec2 pctSize = ImGui::CalcTextSize(pctBuf);
        ImGui::SetCursorPos(ImVec2((screenW - pctSize.x) * 0.5f, textY));
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "%s", pctBuf);

        // Status text centered below bar
        float statusY = barCenterY + 16.0f;
        ImVec2 statusSize = ImGui::CalcTextSize(statusText.c_str());
        ImGui::SetCursorPos(ImVec2((screenW - statusSize.x) * 0.5f, statusY));
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "%s", statusText.c_str());

        ImGui::End();
        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    glEnable(GL_DEPTH_TEST);
}

} // namespace rendering
} // namespace wowee
