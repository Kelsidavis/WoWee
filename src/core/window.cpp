#include "core/window.hpp"

#include <cmath>
#include "core/env.hpp"
#include "core/logger.hpp"
#include "core/config_paths.hpp"
#include "stb_image.h"
#include "rendering/vk_context.hpp"
#include <SDL2/SDL_vulkan.h>
#include <cstdlib>
#ifdef __APPLE__
#include "core/macos_platform.hpp"
#include <filesystem>
#include <mach-o/dyld.h>
#include <vector>
#endif

namespace wowee {
namespace core {

#ifdef __APPLE__
namespace {

std::string bundledMoltenVkManifest() {
    uint32_t pathSize = 0;
    _NSGetExecutablePath(nullptr, &pathSize);
    if (pathSize == 0) return {};

    std::vector<char> executablePath(pathSize + 1, '\0');
    if (_NSGetExecutablePath(executablePath.data(), &pathSize) != 0) return {};

    std::error_code ec;
    auto executable = std::filesystem::weakly_canonical(executablePath.data(), ec);
    if (ec) return {};

    auto candidate = executable.parent_path().parent_path()
        / "Resources" / "vulkan" / "icd.d" / "MoltenVK_icd.json";
    return std::filesystem::exists(candidate) ? candidate.string() : std::string{};
}

} // namespace
#endif

Window::Window(const WindowConfig& config)
    : config(config)
    , width(config.width)
    , height(config.height)
    , windowedWidth(config.width)
    , windowedHeight(config.height)
    , fullscreen(config.fullscreen)
    , vsync(config.vsync) {
}

Window::~Window() {
    shutdown();
}

bool Window::initialize() {
    LOG_INFO("Initializing window: ", config.title);

#ifdef __APPLE__
    // Before SDL_Init spins up NSApplication: holding a key should repeat it,
    // not open the accent chooser over the game.
    disablePressAndHoldAccents();
#endif

#ifdef __ANDROID__
    // Without this the manifest's screenOrientation does not survive: SDL calls
    // setOrientation itself when it creates the window, and with no hint and a
    // resizable window it asks for FULL_USER, which follows the phone's own
    // rotation lock. That is portrait, and the interface is laid out for a
    // landscape screen. Naming both landscape orientations leaves the phone
    // free to flip between them.
    SDL_SetHint(SDL_HINT_ORIENTATIONS, "LandscapeLeft LandscapeRight");
#endif

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        LOG_ERROR("Failed to initialize SDL: ", SDL_GetError());
        return false;
    }

    // Explicitly load the Vulkan library before creating the window.
    // SDL_CreateWindow with SDL_WINDOW_VULKAN fails on some platforms/drivers
    // if the Vulkan loader hasn't been located yet; calling this first gives a
    // clear error and avoids the misleading "not configured in SDL" message.
    // SDL 2.28+ uses LoadLibraryExW(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS) which does
    // not search System32, so fall back to the explicit path on Windows if needed.
    //
    // On macOS, MoltenVK is a Vulkan "portability" driver.  The Vulkan loader
    // hides portability drivers (and their extensions like VK_KHR_surface) from
    // pre-instance enumeration unless told otherwise.  Setting this env var
    // makes the loader include portability ICDs so SDL's VK_KHR_surface check
    // succeeds.
#ifdef __APPLE__
    setEnvVar("VK_LOADER_ENABLE_PORTABILITY_DRIVERS", "1", /*overwrite=*/false);
    // Probe for MoltenVK's ICD JSON if VK_ICD_FILENAMES isn't already set.
    // Without it the Vulkan loader can't find MoltenVK and SDL's pre-instance
    // VK_KHR_surface check fails - the typical symptom when building with the
    // LunarG SDK without sourcing setup-env.sh first.  Check $VULKAN_SDK
    // (LunarG SDK) before falling back to the two common Homebrew prefixes.
    if (!std::getenv("VK_ICD_FILENAMES")) {
        // Prefer the app-bundled driver so a redistributed build never depends
        // on a developer's Homebrew or LunarG SDK installation.
        std::string foundIcd = bundledMoltenVkManifest();
        if (const char* sdk = std::getenv("VULKAN_SDK"); sdk && *sdk) {
            if (foundIcd.empty()) {
                std::string candidate = std::string(sdk) + "/share/vulkan/icd.d/MoltenVK_icd.json";
                if (std::filesystem::exists(candidate)) foundIcd = candidate;
            }
        }
        if (foundIcd.empty()) {
            for (const char* p : {
                    "/opt/homebrew/share/vulkan/icd.d/MoltenVK_icd.json",
                    "/usr/local/share/vulkan/icd.d/MoltenVK_icd.json"}) {
                if (std::filesystem::exists(p)) { foundIcd = p; break; }
            }
        }
        if (!foundIcd.empty()) {
            setEnvVar("VK_ICD_FILENAMES", foundIcd.c_str());
            LOG_INFO("Auto-detected MoltenVK ICD: ", foundIcd);
        }
    }
#endif
    bool vulkanLoaded = (SDL_Vulkan_LoadLibrary(nullptr) == 0);
#ifdef _WIN32
    if (!vulkanLoaded) {
        const char* sysRoot = std::getenv("SystemRoot");
        if (sysRoot && *sysRoot) {
            std::string fallbackPath = std::string(sysRoot) + "\\System32\\vulkan-1.dll";
            vulkanLoaded = (SDL_Vulkan_LoadLibrary(fallbackPath.c_str()) == 0);
            if (vulkanLoaded) {
                LOG_INFO("Loaded Vulkan library via explicit path: ", fallbackPath);
            }
        }
    }
#endif
    if (!vulkanLoaded) {
        LOG_ERROR("Failed to load Vulkan library: ", SDL_GetError());
#ifdef __APPLE__
        LOG_ERROR("On macOS, install Vulkan via Homebrew:  brew install vulkan-loader molten-vk");
        LOG_ERROR("Or source the LunarG SDK setup script before running:  source $VULKAN_SDK/setup-env.sh");
#else
        LOG_ERROR("Ensure the Vulkan runtime (vulkan-1.dll) is installed. "
                  "Install the latest GPU drivers or the Vulkan Runtime from https://vulkan.lunarg.com/");
#endif
        SDL_Quit();
        return false;
    }

    // Create Vulkan window (no GL attributes needed)
    Uint32 flags = SDL_WINDOW_VULKAN | SDL_WINDOW_SHOWN;
    if (config.fullscreen) {
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    }
#ifdef __ANDROID__
    // A phone has no windows to be one of. Fullscreen is also what makes SDL
    // put the activity in immersive mode, which is what hides the navigation
    // bar; without it the client draws into 2272x954 of a 2424x1080 panel and
    // the rest is system chrome.
    flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
#endif
    if (config.resizable) {
        flags |= SDL_WINDOW_RESIZABLE;
    }

    window = SDL_CreateWindow(
        config.title.c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width,
        height,
        flags
    );

    if (!window) {
        LOG_ERROR("Failed to create window: ", SDL_GetError());
        return false;
    }

    setWindowIcon();

    // Initialize Vulkan context
    vkContext = std::make_unique<rendering::VkContext>();
    vkContext->setVsync(vsync);
    if (!vkContext->initialize(window)) {
        LOG_ERROR("Failed to initialize Vulkan context");
        return false;
    }

    // SDL and the Vulkan driver leave the working directory at /system/bin on
    // Android, and everything after this opens its files relative to it: the
    // skybox shader was the first to fail, one call after this returned.
    core::enterResourceRoot();

    LOG_INFO("Window initialized successfully (Vulkan)");
    return true;
}

/// The icon the window and the task switcher show.
///
/// The build installs assets/Wowee.png as a hicolor icon and writes a .desktop
/// file pointing at it, which is what a packaged copy uses. Nothing ever told
/// the window itself, so a client run from the build directory - which is every
/// run during development - had the toolkit's blank default.
///
/// Not fatal, and quiet about it: a missing or unreadable icon costs the window
/// nothing but the icon.
void Window::setWindowIcon() {
    static constexpr const char* kIconPath = "assets/Wowee.png";
    int w = 0, h = 0, channels = 0;
    unsigned char* pixels = stbi_load(kIconPath, &w, &h, &channels, 4);
    if (!pixels) {
        LOG_DEBUG("Window icon not loaded from ", kIconPath, ": ", stbi_failure_reason());
        return;
    }

    // RGBA in memory order, which is what stb_image gives whatever the file
    // held. The masks say so explicitly rather than relying on the byte order
    // of the machine.
    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
        pixels, w, h, 32, w * 4,
        0x000000FFu, 0x0000FF00u, 0x00FF0000u, 0xFF000000u);
    if (surface) {
        SDL_SetWindowIcon(window, surface);
        SDL_FreeSurface(surface);
    } else {
        LOG_DEBUG("Window icon surface failed: ", SDL_GetError());
    }
    // After SDL_SetWindowIcon, which copies what it needs.
    stbi_image_free(pixels);
}

void Window::shutdown() {
    LOG_DEBUG("Window::shutdown - vkContext...");
    if (vkContext) {
        vkContext->shutdown();
        vkContext.reset();
    }

    LOG_DEBUG("Window::shutdown - SDL_DestroyWindow...");
    if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }

    LOG_DEBUG("Window::shutdown - SDL_Quit...");
    SDL_Vulkan_UnloadLibrary();
    SDL_Quit();
    LOG_DEBUG("Window shutdown complete");
}
void Window::setFullscreen(bool enable) {
    if (!window) return;
    if (enable == fullscreen) return;
    if (enable) {
        windowedWidth = width;
        windowedHeight = height;
        if (SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP) != 0) {
            LOG_WARNING("Failed to enter fullscreen: ", SDL_GetError());
            return;
        }
        fullscreen = true;
        SDL_GetWindowSize(window, &width, &height);
    } else {
        if (SDL_SetWindowFullscreen(window, 0) != 0) {
            LOG_WARNING("Failed to exit fullscreen: ", SDL_GetError());
            return;
        }
        fullscreen = false;
        SDL_SetWindowSize(window, windowedWidth, windowedHeight);
        width = windowedWidth;
        height = windowedHeight;
    }
    if (vkContext) {
        vkContext->markSwapchainDirty();
    }
}

void Window::setVsync(bool enable) {
    vsync = enable;
    if (vkContext) {
        vkContext->setVsync(enable);
        vkContext->markSwapchainDirty();
    }
    LOG_INFO("VSync ", enable ? "enabled" : "disabled");
}

void Window::applyResolution(int w, int h) {
    if (!window) return;
    if (w <= 0 || h <= 0) return;
    if (fullscreen) {
        const int displayIndex = SDL_GetWindowDisplayIndex(window);
        if (displayIndex < 0) {
            LOG_WARNING("Could not determine display for fullscreen resolution ",
                        w, "x", h, ": ", SDL_GetError());
            return;
        }

        // A mode whose shape does not match the display is not worth taking.
        //
        // Maximising a window gives the desktop's own aspect and looks right;
        // going fullscreen then forced whatever resolution the selector held,
        // and on a display that is not that shape the result is stretched or
        // letterboxed with the field of view fighting it. If the chosen
        // resolution is not the display's shape, stay on the desktop mode -
        // which is the shape the player just had - rather than honouring a
        // number at the cost of the picture.
        SDL_DisplayMode desktop{};
        if (SDL_GetDesktopDisplayMode(displayIndex, &desktop) == 0 &&
            desktop.w > 0 && desktop.h > 0) {
            const float wantAspect = static_cast<float>(w) / static_cast<float>(h);
            const float haveAspect =
                static_cast<float>(desktop.w) / static_cast<float>(desktop.h);
            if (std::abs(wantAspect - haveAspect) > haveAspect * 0.02f) {
                LOG_INFO("Fullscreen keeps the desktop mode ", desktop.w, "x", desktop.h,
                         ": the chosen ", w, "x", h, " is a different shape");
                if (SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP) == 0) {
                    SDL_GetWindowSize(window, &width, &height);
                    if (vkContext) vkContext->markSwapchainDirty();
                }
                return;
            }
        }

        SDL_DisplayMode requested{};
        requested.w = w;
        requested.h = h;
        SDL_DisplayMode closest{};
        if (!SDL_GetClosestDisplayMode(displayIndex, &requested, &closest)) {
            LOG_WARNING("No fullscreen display mode available near ", w, "x", h,
                        ": ", SDL_GetError());
            return;
        }
        if (SDL_SetWindowDisplayMode(window, &closest) != 0) {
            LOG_WARNING("Failed to select fullscreen display mode ", closest.w,
                        "x", closest.h, ": ", SDL_GetError());
            return;
        }
        // FULLSCREEN_DESKTOP always uses the desktop mode and was silently
        // ignoring the resolution selector (especially visible on macOS).
        if (SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN) != 0) {
            LOG_WARNING("Failed to apply fullscreen resolution ", closest.w,
                        "x", closest.h, ": ", SDL_GetError());
            return;
        }
        SDL_GetWindowSize(window, &width, &height);
        if (vkContext) {
            vkContext->markSwapchainDirty();
        }
        LOG_INFO("Fullscreen resolution applied: ", width, "x", height);
        return;
    }
    SDL_SetWindowSize(window, w, h);
    SDL_GetWindowSize(window, &width, &height);
    windowedWidth = w;
    windowedHeight = h;
    if (vkContext) {
        vkContext->markSwapchainDirty();
    }
}

} // namespace core
} // namespace wowee
