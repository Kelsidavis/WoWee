#pragma once

#include <GL/glew.h>
#include <string>
#include <vector>

namespace wowee {
namespace rendering {

class LoadingScreen {
public:
    LoadingScreen();
    ~LoadingScreen();

    bool initialize();
    void shutdown();

    // Select a random loading screen image
    void selectRandomImage();

    // Render the loading screen with progress bar and status text
    void render();

    // Update loading progress (0.0 to 1.0)
    void setProgress(float progress) { loadProgress = progress; }

    // Set loading status text
    void setStatus(const std::string& status) { statusText = status; }

private:
    bool loadImage(const std::string& path);
    void createQuad();
    void createBarQuad();

    GLuint textureId = 0;
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint shaderId = 0;

    // Progress bar GL objects
    GLuint barVao = 0;
    GLuint barVbo = 0;
    GLuint barShaderId = 0;

    std::vector<std::string> imagePaths;
    int currentImageIndex = 0;

    float loadProgress = 0.0f;
    std::string statusText = "Loading...";

    int imageWidth = 0;
    int imageHeight = 0;
};

} // namespace rendering
} // namespace wowee
