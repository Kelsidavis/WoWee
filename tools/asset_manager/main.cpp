/// WoWee Asset Manager - a native window for building this client's assets.
///
///     wowee_assets [game folder] [a later client] [where to put it]
///
/// Three questions in the order somebody actually has answers for: where is the
/// game, what do you want it to look like, and go.
///
/// This is a C++ program because the thing it replaced was not: a Tkinter window
/// driving shell scripts that drove Python scripts, each needing an interpreter
/// with the right modules built into it. On one machine `python3` had no Tkinter
/// and the window simply did not appear - a perfectly good interpreter sat one
/// directory away and nothing said so. A tool that installs the game's assets
/// should not itself need an install.
///
/// It uses SDL2 and Dear ImGui, both of which the client already carries, and
/// calls Extractor::run in this process. Nothing is shelled out to.

#include <SDL.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

// The implementation is already compiled into open_format_emitter.cpp.
#include "stb_image_write.h"

#include "core/data_paths.hpp"
#include "pipeline/asset_inventory.hpp"
#include "core/local_time.hpp"
#include "ui/imgui_theme.hpp"

#include "casc.hpp"
#include "pack.hpp"
#include "install_scan.hpp"
#include "folder_picker.hpp"
#include "job.hpp"
#include "profiles.hpp"

#include "panel.hpp"

namespace {

namespace fs = std::filesystem;
using namespace wowee::assets;

/// ImGui's built-in face is drawn at thirteen pixels. Everything the panel
/// lays out was sized against that, so it is the height the atlas is scaled
/// from rather than a size worth changing.
constexpr float kBaseFontSize = 13.0f;

/// FRIZQT draws smaller than ImGui's built-in face at the same nominal height,
/// so asking for thirteen of it puts noticeably smaller text into controls
/// sized for thirteen. This is the height that matches.
constexpr float kFrizqtRatio = 1.25f;

/// How many frames to let settle before the picture is taken. ImGui sizes a
/// good deal of its layout from what it measured last frame, and hides a window
/// outright for its first frame or two while it fits itself, so an early one
/// is not what the window looks like.
constexpr int kShotFrame = 8;

/// The renderer's own pixels, as a PNG.
void writeScreenshot(SDL_Renderer* renderer, const char* path) {
    int width = 0;
    int height = 0;
    if (SDL_GetRendererOutputSize(renderer, &width, &height) != 0) return;
    std::vector<uint8_t> pixels(std::size_t(width) * std::size_t(height) * 4);
    if (SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ABGR8888,
                             pixels.data(), width * 4) != 0) {
        std::fprintf(stderr, "could not read the window back: %s\n", SDL_GetError());
        return;
    }
    if (stbi_write_png(path, width, height, 4, pixels.data(), width * 4) == 0) {
        std::fprintf(stderr, "could not write %s\n", path);
        return;
    }
    std::printf("wrote %s (%dx%d)\n", path, width, height);
}

/// The game's own typeface, if any extraction on this machine has produced it.
///
/// A window that installs the game's assets should look like the game, and the
/// face is most of that. It is looked for where the client itself reads from
/// and beside whatever this window is writing to - which covers the common case
/// of running this a second time, to add something to a tree already built.
/// Before any extraction there is nothing to find, and ImGui's built-in face is
/// what the layout was measured against anyway.
fs::path gameFont(const std::string& outputDir, const char* name) {
    std::vector<fs::path> roots;
    if (!outputDir.empty()) roots.emplace_back(outputDir);
    // Where the client reads from on this platform, which is not the same
    // directory on all three - written out by hand it was the macOS one, and
    // the window went without the game's face everywhere else.
    if (const fs::path userData = wowee::core::userDataRoot(); !userData.empty()) {
        roots.emplace_back(userData);
    }

    std::error_code ec;
    for (const fs::path& root : roots) {
        // Directly under the root for a single-expansion tree, and one level
        // into expansions/ for the layout the profiles write.
        std::vector<fs::path> here{root};
        const fs::path expansions = root / "expansions";
        for (fs::directory_iterator it(expansions, ec), end; it != end && !ec; it.increment(ec)) {
            here.push_back(it->path());
        }
        for (const fs::path& at : here) {
            for (const char* dir : {"fonts", "misc/fonts"}) {
                const fs::path file = at / dir / name;
                if (fs::is_regular_file(file, ec)) return file;
            }
        }
    }
    return {};
}

/// How many pixels the renderer puts down for each point the window is measured
/// in. One on an ordinary display, two on a Retina one.
float displayScale(SDL_Window* window, SDL_Renderer* renderer) {
    int windowWidth = 0;
    int pixelWidth = 0;
    SDL_GetWindowSize(window, &windowWidth, nullptr);
    if (SDL_GetRendererOutputSize(renderer, &pixelWidth, nullptr) != 0) return 1.0f;
    if (windowWidth <= 0 || pixelWidth <= 0) return 1.0f;
    return std::max(1.0f, float(pixelWidth) / float(windowWidth));
}

/// A startup failure, said somewhere it will be seen.
///
/// This is a windowed program on Windows, so it has no console to print to:
/// stderr goes nowhere and a double-click on a machine that cannot open a
/// window looked exactly like nothing happening at all. SDL's own box needs no
/// platform code and works before SDL_Init, so the one case that has no window
/// yet is covered too.
void startupFailure(const char* what) {
    std::fprintf(stderr, "%s\n", what);
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "WoWee Asset Manager", what, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        startupFailure((std::string("SDL could not start: ") + SDL_GetError()).c_str());
        return 1;
    }

    // As much of the page as the display will show at once, since what is not
    // shown has to be scrolled past. Clamped to the space the desktop actually
    // leaves, so a laptop does not open a window taller than its screen.
    int wide = 980;
    int high = 900;
    if (SDL_Rect usable; SDL_GetDisplayUsableBounds(0, &usable) == 0) {
        wide = std::min(wide, std::max(640, usable.w - 80));
        high = std::min(high, std::max(520, usable.h - 80));
    }

    SDL_Window* window = SDL_CreateWindow(
        "WoWee Asset Manager", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        wide, high, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (window == nullptr) {
        startupFailure((std::string("Could not open a window: ") + SDL_GetError()).c_str());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        // Software is slower and perfectly adequate for a form with a log in it.
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (renderer == nullptr) {
        startupFailure((std::string("Could not draw: ") + SDL_GetError()).c_str());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    wowee::ui::applyWoweeStyle(ImGui::GetStyle());

    // On a Retina display the window is 900 points wide and the thing drawn
    // into is 1800 pixels. SDL_Renderer does not bridge that on its own, so
    // without this the interface is drawn at point coordinates into a pixel
    // buffer: half size, in the top-left quarter of its own window, with the
    // rest left empty.
    //
    // The renderer is told to scale, which puts the layout back where it
    // belongs. The font atlas is then built at the pixel size it will actually
    // occupy and laid out as though it were half that, so the glyphs land one
    // texel to a pixel rather than being magnified along with everything else.
    const float scale = displayScale(window, renderer);
    ImGuiIO& io = ImGui::GetIO();

    App app;
    // Where the client itself looks when nobody has said otherwise. Data/ beside
    // the terminal's working directory was eight minutes of extraction the
    // client would never find, with nothing on screen saying where it went.
    initPanelDefaults(app);

    // The same folders the window takes by drag and drop, for anyone who
    // already has the paths in a terminal.
    if (argc > 1) std::snprintf(app.gameDir, sizeof(app.gameDir), "%s", argv[1]);
    if (argc > 2) std::snprintf(app.secondDir, sizeof(app.secondDir), "%s", argv[2]);
    if (argc > 3) std::snprintf(app.outputDir, sizeof(app.outputDir), "%s", argv[3]);

    // FRIZQT is what the game writes its interface in. A little larger than
    // ImGui's built-in face at the same nominal height, so it is asked for at a
    // size that keeps the controls around it the size they were laid out at.
    const fs::path face = gameFont(app.outputDir, "frizqt__.ttf");
    if (!face.empty()) {
        ImFontConfig config;
        config.SizePixels = kBaseFontSize * kFrizqtRatio * scale;
        io.Fonts->AddFontFromFileTTF(face.string().c_str(), config.SizePixels);
    } else if (scale > 1.0f) {
        ImFontConfig config;
        config.SizePixels = kBaseFontSize * scale;
        io.Fonts->AddFontDefault(&config);
    }
    io.FontGlobalScale = 1.0f / scale;

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    rescan(app);

    const char* shotPath = std::getenv("WOWEE_ASSETS_SCREENSHOT");
    int frames = 0;

    bool quit = false;
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) quit = true;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                quit = true;
            }
            if (event.type == SDL_DROPFILE && event.drop.file != nullptr) {
                std::snprintf(app.gameDir, sizeof(app.gameDir), "%s", event.drop.file);
                SDL_free(event.drop.file);
                rescan(app);
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("WoWee Asset Manager", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
        ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
        ImGui::TextUnformatted(
            "Builds the assets this client reads out of a World of Warcraft install you "
            "own. Nothing is written into the install itself.");
        ImGui::PopTextWrapPos();
        ImGui::PopStyleColor();
        ImGui::Spacing();

        drawPanel(app);

        ImGui::End();

        ImGui::Render();
        // Re-read every frame: a window dragged between a Retina display and an
        // ordinary one changes this without resizing.
        const float now = displayScale(window, renderer);
        SDL_RenderSetScale(renderer, now, now);
        SDL_SetRenderDrawColor(renderer, 24, 24, 28, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        // A picture of the window, written once and then done with. What this
        // looks like cannot be checked by reading it, and a bug report about a
        // window is a screenshot; this is how one gets taken on a machine where
        // the person hitting the problem cannot easily take one.
        //
        // Read before the frame is presented, not after: presenting leaves the
        // buffer it came from undefined, and on Metal reading it back then
        // gives a blank image rather than an error.
        if (shotPath != nullptr && ++frames == kShotFrame) {
            writeScreenshot(renderer, shotPath);
            quit = true;
        }

        SDL_RenderPresent(renderer);
    }

    // Both workers are told to stop here and waited for in ~App, so they wind
    // down while SDL is being torn down rather than after it.
    app.job.cancel();
    app.packCancel.store(true);
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
