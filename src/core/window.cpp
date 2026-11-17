#include "core/window.hpp"

#include <algorithm>

#include <cmath>
#include "core/env.hpp"
#include "core/logger.hpp"
#include "core/config_paths.hpp"
#include "stb_image.h"
#include "rendering/vk_context.hpp"
#include <SDL3/SDL_vulkan.h>
#include <cstdlib>
#ifdef __APPLE__
#include "core/macos_platform.hpp"
#include <filesystem>
#include <mach-o/dyld.h>
#include <vector>
#endif
#if defined(WOWEE_IOS)
#include "core/ios_platform.hpp"
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
#if defined(WOWEE_IOS)
    // iOS platform prep (safe areas, keyboard, etc.).
    core::iosPreparePlatform();
#else
    // Before SDL_Init spins up NSApplication: holding a key should repeat it,
    // not open the accent chooser over the game.
    disablePressAndHoldAccents();
#endif
#endif

#if defined(__ANDROID__)
    // Force landscape on phones/tablets.
    SDL_SetHint(SDL_HINT_ORIENTATIONS, "LandscapeLeft LandscapeRight");

    // By default SDL parks the thread that called SDL_main for as long as the
    // activity is in the background. That thread is the one that reads the
    // socket, so a few seconds behind the home button and the server has timed
    // the session out. Letting it run keeps the connection; the surface is
    // released separately and the frame is skipped while it is gone.
    SDL_SetHint(SDL_HINT_ANDROID_BLOCK_ON_PAUSE, "0");
#elif defined(WOWEE_IOS)
    // Landscape, like a notebook.  Portrait used to be forced here because
    // a landscape window presented black - but that was the missing-scene
    // bug (the window was never displayed at all), not an orientation
    // problem.  Info.plist agrees: landscape-only, no portrait.
    SDL_SetHint(SDL_HINT_ORIENTATIONS, "LandscapeLeft LandscapeRight");
#endif

    // Initialize SDL
    // SDL3 answers true on success where SDL2 answered 0: this test is
    // inverted from what it was, not renamed.
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
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
    bool vulkanLoaded = SDL_Vulkan_LoadLibrary(nullptr);
#ifdef _WIN32
    if (!vulkanLoaded) {
        const char* sysRoot = std::getenv("SystemRoot");
        if (sysRoot && *sysRoot) {
            std::string fallbackPath = std::string(sysRoot) + "\\System32\\vulkan-1.dll";
            vulkanLoaded = SDL_Vulkan_LoadLibrary(fallbackPath.c_str());
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
    // SDL3 shows a window by default, so there is no SHOWN flag.
    Uint32 flags = SDL_WINDOW_VULKAN;
#ifdef __APPLE__
    // Draw at the display's own pixels rather than at its points.
    //
    // Without this a window on a Retina panel gets a surface the size of the
    // window in points and the display stretches it: on a 14-inch MacBook Pro
    // the whole client rendered into 1512x982 and was blown up to 3024x1964,
    // which is every edge and every glyph softened, permanently, with no
    // setting to say so. With it the surface is the full 3024x1964.
    //
    // Apple only on purpose. It is the platform where a point is not a pixel
    // for this client; the swapchain now asks SDL for the drawable size
    // either way, so the two agree wherever they are already equal.
    flags |= SDL_WINDOW_HIGH_PIXEL_DENSITY;
#endif
    if (config.fullscreen) {
        flags |= SDL_WINDOW_FULLSCREEN;
    }
#if defined(__ANDROID__) || defined(WOWEE_IOS)
    // No window chrome on a device; fullscreen is the natural state.
    flags |= SDL_WINDOW_FULLSCREEN;
#endif
    if (config.resizable) {
        flags |= SDL_WINDOW_RESIZABLE;
    }

    // SDL3 takes no position: a new window is placed by the platform, and
    // SDL_SetWindowPosition is the way to say otherwise.
    window = SDL_CreateWindow(
        config.title.c_str(),
        width,
        height,
        flags
    );

    if (!window) {
        LOG_ERROR("Failed to create window: ", SDL_GetError());
        return false;
    }

    setWindowIcon();

    // Whether this platform's window can be resized at all.
    //
    // Asked of SDL rather than decided by an #ifdef, because the answer is a
    // property of the video backend rather than of the platform: SDL's UIKit
    // backend has no SetWindowSize at all, so the call there is answered by
    // doing nothing and the window stays the size the system gave it. On
    // Android it does have one, and the window can be resized.
    //
    // Found by asking for the size the window already is: a request that
    // cannot be honoured is refused, and a request that asks for what is
    // already true is granted without changing anything.
    {
        int currentW = 0, currentH = 0;
        SDL_GetWindowSize(window, &currentW, &currentH);
        canResizeWindow_ = SDL_SetWindowSize(window, currentW, currentH);
        if (!canResizeWindow_) {
            LOG_INFO("This platform cannot resize the window: a resolution "
                     "request will change the size the world is drawn at, not "
                     "the size of the window.");
        }
    }

    // Before the swapchain, which is sized from it.
    SDL_GetWindowSize(window, &width, &height);
    refreshDrawableSize();
    if (drawableWidth != width || drawableHeight != height) {
        LOG_WARNING("Window is ", width, "x", height, " points on a ",
                    drawableWidth, "x", drawableHeight,
                    " pixel surface - the world is drawn at the pixel size.");
    }

    // The resolutions this device offers, built now that both the screen's
    // size and the answer to whether the window can be resized are known.
    //
    // The screen's size is the surface's, in pixels - the surface is what
    // the world is drawn on, so a resolution that does not fit it is a
    // resolution the device does not have. Where the window cannot be
    // resized this is the whole list the player can choose from.
    {
        const int screenW = std::max(drawableWidth, width);
        const int screenH = std::max(drawableHeight, height);
        displayResolutions_ = canResizeWindow_
            ? ui::displayResolutionsForDesktop()
            : ui::displayResolutionsForScreen(screenW, screenH);
        if (!canResizeWindow_) {
            LOG_INFO("Resolutions offered:");
            for (std::size_t i = 0; i < displayResolutions_.size(); ++i) {
                LOG_INFO("  ", displayResolutions_[i].width, "x",
                         displayResolutions_[i].height,
                         displayResolutions_[i].native ? " (the screen itself)" : "");
            }
        }
    }
#if defined(WOWEE_IOS)
    // SDL 3.2.24 creates its UIWindow with initWithFrame: and never gives
    // it a scene; a scene-based app never displays such a window (black
    // screen).  Attach it to the connected scene, then show what we got.
    core::iosAttachWindowToScene(window);
    core::iosLogSdlWindow(window);
    // What is on screen right now, and (via a delayed sample) once settled.
    core::iosLogWindowStatus("after scene attach");
    LOG_INFO("iOS window: ", width, "x", height, " points, drawable ",
             drawableWidth, "x", drawableHeight);
    if (const char* r = std::getenv("WOWEE_RESOURCE_ROOT")) LOG_INFO("iOS resource root: ", r);
    if (const char* d = std::getenv("WOW_DATA_PATH")) LOG_INFO("iOS data path: ", d);
#endif

    // Initialize Vulkan context
    vkContext = std::make_unique<rendering::VkContext>();
    vkContext->setVsync(vsync);
    if (!vkContext->initialize(window)) {
        LOG_ERROR("Failed to initialize Vulkan context");
        return false;
    }

#if defined(__ANDROID__) || defined(WOWEE_IOS)
    // On a device the working directory is not where the resources live.
    core::enterResourceRoot();
#endif

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
    // SDL3 names the format instead of taking four masks. ABGR8888 is the
    // one whose bytes are R,G,B,A in memory, which is what stb_image gives
    // whatever the file held.
    SDL_Surface* surface = SDL_CreateSurfaceFrom(
        w, h, SDL_PIXELFORMAT_ABGR8888, pixels, w * 4);
    if (surface) {
        SDL_SetWindowIcon(window, surface);
        SDL_DestroySurface(surface);
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
        // SDL3 answers true on success where SDL2 answered 0, so this test
        // is inverted from what it was rather than renamed.
        if (!SDL_SetWindowFullscreen(window, true)) {
            LOG_WARNING("Failed to enter fullscreen: ", SDL_GetError());
            return;
        }
        fullscreen = true;
        SDL_GetWindowSize(window, &width, &height);
        refreshDrawableSize();
    } else {
        if (!SDL_SetWindowFullscreen(window, false)) {
            LOG_WARNING("Failed to exit fullscreen: ", SDL_GetError());
            return;
        }
        fullscreen = false;
        SDL_SetWindowSize(window, windowedWidth, windowedHeight);
        width = windowedWidth;
        height = windowedHeight;
    }
    // Said, as vsync changes are: several things can switch the window between
    // full screen and not, and a report that it "did not stay" is otherwise a
    // guess at which of them did.
    LOG_WARNING("Window: full screen ", fullscreen ? "on" : "off", ", ", width, "x", height);
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
    // WARNING, not INFO: this build logs INFO nowhere, so the one line that
    // would say who turned vsync off never appeared. Vsync is set from four
    // places and a stale one overriding a fresh one is invisible otherwise.
    LOG_WARNING("VSync ", enable ? "enabled" : "disabled",
                " (requested by the caller; swapchain marked dirty)");
}

void Window::refreshDrawableSize() {
    if (!window) { drawableWidth = width; drawableHeight = height; return; }
    int dw = 0, dh = 0;
    SDL_GetWindowSizeInPixels(window, &dw, &dh);
    // A minimised window answers zero, and a swapchain of zero is a spec
    // violation - so the last good size stands until there is a real one.
    if (dw > 0 && dh > 0) {
        drawableWidth = dw;
        drawableHeight = dh;
    }
}

void Window::applyResolution(int w, int h) {
    if (!window) return;
    if (w <= 0 || h <= 0) return;

    // A window that cannot be resized cannot be given a resolution - the
    // request would be accepted and change nothing, and the player would see
    // a dropdown that says one thing and does another.
    //
    // On such a platform the resolution means the size the world is drawn at
    // instead, and the upscaler stretches it onto the screen. That is the
    // only way a resolution selector can mean anything here, and it is what
    // makes a lower resolution worth choosing at all: fewer pixels drawn,
    // then scaled up.
    if (!canResizeWindow_) {
        setRenderResolution(w, h);
        return;
    }

    if (fullscreen) {
        const int displayIndex = SDL_GetDisplayForWindow(window);
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
        // SDL3 hands back a pointer to the mode it owns rather than filling
        // in a caller's copy and answering 0 for success.
        const SDL_DisplayMode* desktopMode = SDL_GetDesktopDisplayMode(displayIndex);
        if (desktopMode != nullptr && desktopMode->w > 0 && desktopMode->h > 0) {
            const SDL_DisplayMode& desktop = *desktopMode;
            const float wantAspect = static_cast<float>(w) / static_cast<float>(h);
            const float haveAspect =
                static_cast<float>(desktop.w) / static_cast<float>(desktop.h);
            if (std::abs(wantAspect - haveAspect) > haveAspect * 0.02f) {
                LOG_INFO("Fullscreen keeps the desktop mode ", desktop.w, "x", desktop.h,
                         ": the chosen ", w, "x", h, " is a different shape");
                if (SDL_SetWindowFullscreen(window, true)) {
                    SDL_GetWindowSize(window, &width, &height);
        refreshDrawableSize();
                    if (vkContext) vkContext->markSwapchainDirty();
                }
                return;
            }
        }

        SDL_DisplayMode closest{};
        // The wanted size is arguments now, not a half-filled mode. 0 for the
        // refresh rate means "whatever this display does", and false leaves
        // out the high-density modes, which are the same picture at a scale
        // this client does its own accounting for.
        if (!SDL_GetClosestFullscreenDisplayMode(displayIndex, w, h, 0.0f, false, &closest)) {
            LOG_WARNING("No fullscreen display mode available near ", w, "x", h,
                        ": ", SDL_GetError());
            return;
        }
        if (!SDL_SetWindowFullscreenMode(window, &closest)) {
            LOG_WARNING("Failed to select fullscreen display mode ", closest.w,
                        "x", closest.h, ": ", SDL_GetError());
            return;
        }
        // FULLSCREEN_DESKTOP always uses the desktop mode and was silently
        // ignoring the resolution selector (especially visible on macOS).
        if (!SDL_SetWindowFullscreen(window, true)) {
            LOG_WARNING("Failed to apply fullscreen resolution ", closest.w,
                        "x", closest.h, ": ", SDL_GetError());
            return;
        }
        SDL_GetWindowSize(window, &width, &height);
        refreshDrawableSize();
        if (vkContext) {
            vkContext->markSwapchainDirty();
        }
        // At warning: this switches the display's mode, which on a desktop
        // with more than one monitor is visible on all of them.
        LOG_WARNING("Window: full screen display mode ", closest.w, "x", closest.h,
                    " for a requested ", w, "x", h);
        return;
    }
    // Windowed. What is asked for and what is granted are not the same
    // thing, and this used to record the request either way.
    //
    // A window is sized in screen points, not pixels, and macOS will not
    // make one larger than the space it has: on a laptop whose desktop is
    // 1728x1117 points, asking for 1920x1080 gives a window clamped to the
    // usable area, and asking for 2560x1440 gives the same clamped window
    // again. The panel then wrote the number it had asked for into the
    // config, so the dropdown showed a resolution the window never had,
    // every larger choice looked identical, and the setting read as broken.
    // Nothing said so: this path logged nothing at all.
    //
    // Clamped here rather than left to the platform, so the number stored is
    // one the window can actually be, and said out loud when it bites.
    int wantW = w;
    int wantH = h;
    const int displayIndex = SDL_GetDisplayForWindow(window);
    SDL_Rect usable{};
    if (displayIndex >= 0 && SDL_GetDisplayUsableBounds(displayIndex, &usable) &&
        usable.w > 0 && usable.h > 0) {
        wantW = std::min(wantW, usable.w);
        wantH = std::min(wantH, usable.h);
    }
    SDL_SetWindowSize(window, wantW, wantH);
    SDL_GetWindowSize(window, &width, &height);
    refreshDrawableSize();
    // What the window actually is, not what was wanted.
    windowedWidth = width;
    windowedHeight = height;
    if (width != w || height != h) {
        LOG_WARNING("Window resolution ", w, "x", h, " was not available: the window is ",
                    width, "x", height, ". A window is sized in screen points, and this "
                    "display offers ", usable.w, "x", usable.h, " of them - on a high "
                    "density screen that is half its pixels or fewer.");
    } else {
        LOG_INFO("Window resolution applied: ", width, "x", height);
    }
    if (vkContext) {
        vkContext->markSwapchainDirty();
    }
}

void Window::setRenderResolution(int w, int h) {
    if (w <= 0 || h <= 0) return;
    if (renderResolutionWidth == w && renderResolutionHeight == h) return;
    renderResolutionWidth = w;
    renderResolutionHeight = h;
    // Said out loud, because on this platform nothing about the window
    // changes and the picture is the only sign that the setting did anything.
    LOG_WARNING("The world will be drawn at ", w, "x", h, " and scaled up to the ",
                drawableWidth, "x", drawableHeight, " screen");
}

int Window::getDisplayResolutionIndex() const {
    return displayResolutionIndex_;
}

void Window::setDisplayResolutionIndex(int index) {
    if (index < 0 || index >= static_cast<int>(displayResolutions_.size())) return;
    displayResolutionIndex_ = index;
    const auto& resolution = displayResolutions_[static_cast<std::size_t>(index)];
    applyResolution(resolution.width, resolution.height);
}

} // namespace core
} // namespace wowee
