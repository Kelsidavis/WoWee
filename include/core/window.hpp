#pragma once

#include <string>
#include <memory>
#include <SDL2/SDL.h>
#include <vulkan/vulkan.h>

namespace wowee {
namespace rendering { class VkContext; }

namespace core {

struct WindowConfig {
    std::string title = "Wowee Native";
    int width = 1920;
    int height = 1080;
    bool fullscreen = false;
    bool vsync = true;
    bool resizable = true;
};

class Window {
public:
    explicit Window(const WindowConfig& config);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool initialize();
    void shutdown();

private:
    /// Give the window its own icon rather than the toolkit's default.
    void setWindowIcon();

public:

    void swapBuffers() {} // No-op: Vulkan presents in Renderer::endFrame()

    [[nodiscard]] bool shouldClose() const { return shouldCloseFlag; }
    void setShouldClose(bool value) { shouldCloseFlag = value; }

    /// The window, in the units the desktop places windows in.
    ///
    /// Points, not pixels, and on a high density display the two differ: a
    /// 14-inch MacBook Pro is 3024x1964 pixels and 1512x982 points. Every
    /// mouse position and every interface coordinate is in these, so this is
    /// what the interface asks for - and what a window can be sized to.
    [[nodiscard]] int getWidth() const { return width; }
    [[nodiscard]] int getHeight() const { return height; }
    /// The surface behind it, in pixels. What the swapchain has to be.
    ///
    /// Equal to the size above wherever a point is a pixel, which is every
    /// platform this runs on but macOS. There it is twice it, and rendering
    /// at the point size means drawing a 1512-wide picture and letting the
    /// display stretch it over 3024 - which is what this client did.
    [[nodiscard]] int getDrawableWidth() const { return drawableWidth; }
    [[nodiscard]] int getDrawableHeight() const { return drawableHeight; }
    void setSize(int w, int h) { width = w; height = h; refreshDrawableSize(); }
    [[nodiscard]] float getAspectRatio() const { return static_cast<float>(width) / height; }
    [[nodiscard]] bool isFullscreen() const { return fullscreen; }
    [[nodiscard]] bool isVsyncEnabled() const { return vsync; }

    /// Frames per second the main loop paces itself to, or 0 for uncapped.
    ///
    /// Lives here beside vsync because it is the same question asked a
    /// different way - how often to present - and a client drawing a 2004 game
    /// will otherwise run as fast as the hardware allows, which on a laptop is
    /// heat and fan noise for frames nobody sees.
    void setFrameCap(int fps) { frameCapFps_ = (fps > 0) ? fps : 0; }
    [[nodiscard]] int frameCap() const { return frameCapFps_; }
    void setFullscreen(bool enable);
    void setVsync(bool enable);
    void applyResolution(int w, int h);

    [[nodiscard]] SDL_Window* getSDLWindow() const { return window; }

    // Vulkan context access
    [[nodiscard]] rendering::VkContext* getVkContext() const { return vkContext.get(); }

private:
    WindowConfig config;
    SDL_Window* window = nullptr;
    std::unique_ptr<rendering::VkContext> vkContext;

    int width;
    int height;
    /// Refreshed with the size, never set by hand: SDL is the only thing
    /// that knows what the surface behind the window came out as.
    int drawableWidth = 0;
    int drawableHeight = 0;
    void refreshDrawableSize();
    int windowedWidth = 0;
    int windowedHeight = 0;
    bool fullscreen = false;
    int frameCapFps_ = 0;   // 0 = uncapped
    bool vsync = true;
    bool shouldCloseFlag = false;
};

} // namespace core
} // namespace wowee
