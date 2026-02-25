#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <vector>

struct SDL_Window;

namespace wowee {
namespace rendering {

class VkContext;

class LoadingScreen {
public:
    LoadingScreen();
    ~LoadingScreen();

    bool initialize();
    void shutdown();

    void selectRandomImage();

    // Render the loading screen with progress bar and status text (pure ImGui)
    void render();

    void setProgress(float progress) { loadProgress = progress; }
    void setStatus(const std::string& status) { statusText = status; }

    // Must be set before initialize() for Vulkan texture upload
    void setVkContext(VkContext* ctx) { vkCtx = ctx; }
    void setSDLWindow(SDL_Window* win) { sdlWindow = win; }

private:
    bool loadImage(const std::string& path);

    VkContext* vkCtx = nullptr;
    SDL_Window* sdlWindow = nullptr;

    // Vulkan texture for background image
    VkImage bgImage = VK_NULL_HANDLE;
    VkDeviceMemory bgMemory = VK_NULL_HANDLE;
    VkImageView bgImageView = VK_NULL_HANDLE;
    VkSampler bgSampler = VK_NULL_HANDLE;
    VkDescriptorSet bgDescriptorSet = VK_NULL_HANDLE; // ImGui texture handle

    std::vector<std::string> imagePaths;
    int currentImageIndex = 0;

    float loadProgress = 0.0f;
    std::string statusText = "Loading...";

    int imageWidth = 0;
    int imageHeight = 0;
};

} // namespace rendering
} // namespace wowee
