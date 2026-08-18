#include "ui/ui_manager.hpp"
#include <cstring>
#include "pipeline/asset_manager.hpp"
#include "ui/interface_fonts.hpp"

#include <SDL2/SDL.h>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include "core/window.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "rendering/vk_context.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_vulkan.h>

namespace wowee {
namespace ui {

UIManager::UIManager() {
    // Create screen instances
    authScreen = std::make_unique<AuthScreen>();
    realmScreen = std::make_unique<RealmScreen>();
    characterCreateScreen = std::make_unique<CharacterCreateScreen>();
    characterScreen = std::make_unique<CharacterScreen>();
    gameScreen = std::make_unique<GameScreen>();
}

UIManager::~UIManager() = default;

namespace {

/// How much bigger the interface has to be drawn than on a desktop monitor.
///
/// Android reports its own density as dots per inch over a 160 baseline, which
/// is the same factor the platform scales its own interface by, so a phone
/// reporting 420 dpi wants 2.625. The client's panels were laid out in pixels
/// against a desktop monitor, and at 1:1 on a 420 dpi phone the login dialog is
/// too small to read and far too small to hit.
///
/// 1.0 everywhere else, where the layouts are already the right size.
#ifdef __ANDROID__
/// The shortest the interface can be, in the units its layouts are written in.
///
/// The client's dialogs are sized in pixels against a desktop monitor, and the
/// tallest of them - character selection, with its list and its row of buttons
/// underneath - needs about this much. Scaling past the point where the screen
/// holds that many units pushes the bottom row off the display, where it cannot
/// be pressed at all.
constexpr float kMinLogicalHeight = 620.0f;
#endif

float interfaceScale([[maybe_unused]] SDL_Window* window) {
#ifdef __ANDROID__
    float diagonalDpi = 0.0f;
    float density = 2.0f;  // No answer from SDL; a phone is still not a monitor.
    if (SDL_GetDisplayDPI(0, &diagonalDpi, nullptr, nullptr) == 0 && diagonalDpi > 0.0f) {
        density = diagonalDpi / 160.0f;  // Android's own baseline for 1x.
    }

    // Density is what makes the text legible and the controls big enough to
    // hit. It is not free: every unit of it takes a unit of layout room away,
    // and a phone in landscape has very little height to give.
    int height = 0;
    if (window) {
        SDL_GetWindowSize(window, nullptr, &height);
    }
    if (height > 0) {
        density = std::min(density, static_cast<float>(height) / kMinLogicalHeight);
    }
    return std::max(1.0f, density);
#else
    return 1.0f;
#endif
}

}  // namespace

bool UIManager::initialize(core::Window* win) {
    window = win;
    LOG_INFO("Initializing UI manager");

    auto* vkCtx = window->getVkContext();
    if (!vkCtx) {
        LOG_ERROR("No Vulkan context available for ImGui initialization");
        return false;
    }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    // Setup ImGui style
    ImGui::StyleColorsDark();

    // Customize style for better WoW feel
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;

    // WoW-inspired colors
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.08f, 0.12f, 0.94f);
    // ImGui uses PopupBg for hover tooltips. Keep their text and item details
    // fully legible over the 3D scene.
    colors[ImGuiCol_PopupBg] = ImVec4(0.06f, 0.06f, 0.09f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.10f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.15f, 0.15f, 0.25f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.20f, 0.25f, 0.40f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.25f, 0.30f, 0.50f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.15f, 0.20f, 0.35f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.20f, 0.25f, 0.40f, 0.55f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.25f, 0.30f, 0.50f, 0.80f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.25f, 0.45f, 1.00f);

    // Padding, rounding, scrollbars and the rest, before the backend starts.
    // Fonts are scaled where the atlas is built, so they stay crisp rather than
    // being magnified from a desktop-sized atlas.
    interfaceScale_ = interfaceScale(window->getSDLWindow());
    if (const float scale = interfaceScale_; scale > 1.0f) {
        style.ScaleAllSizes(scale);
        LOG_INFO("Interface scaled by ", scale, " for this display");
    }

    // Initialize ImGui for SDL2 + Vulkan
    ImGui_ImplSDL2_InitForVulkan(window->getSDLWindow());

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion = VK_API_VERSION_1_1;
    initInfo.Instance = vkCtx->getInstance();
    initInfo.PhysicalDevice = vkCtx->getPhysicalDevice();
    initInfo.Device = vkCtx->getDevice();
    initInfo.QueueFamily = vkCtx->getGraphicsQueueFamily();
    initInfo.Queue = vkCtx->getGraphicsQueue();
    initInfo.DescriptorPool = vkCtx->getImGuiDescriptorPool();
    initInfo.MinImageCount = 2;
    initInfo.ImageCount = vkCtx->getSwapchainImageCount();
    // The UI renders in the overlay pass, which is single-sampled on purpose:
    // ImGui draws axis-aligned rects and pre-antialiased glyphs, so MSAA buys
    // almost nothing there and costs fill rate at the sample count the scene uses.
    initInfo.PipelineInfoMain.RenderPass = vkCtx->getOverlayRenderPass();
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.CheckVkResultFn = [](VkResult err) {
        if (err != VK_SUCCESS)
            LOG_ERROR("ImGui Vulkan error: ", static_cast<int>(err));
    };

    ImGui_ImplVulkan_Init(&initInfo);

    imguiInitialized = true;

    LOG_INFO("UI manager initialized successfully (Vulkan)");
    return true;
}

void UIManager::loadInterfaceFont(const std::string& dataRoot,
                                  pipeline::AssetManager* assets) {
    if (!imguiInitialized) return;
    // Called more than once on purpose: with the archives open and again after
    // in case they never opened, and against more than one root because the
    // fonts do not sit in the same place in every install. Whichever gets
    // there first takes the face, and the atlas can only be added to before
    // the first frame anyway.
    //
    // The flag latches on success only. It used to be set on the way in, so a
    // root with no fonts under it stopped every later attempt: pointing this
    // at the expansion overlay left an install that keeps its fonts in the
    // base Data on the built-in face, and the second call could not save it.
    if (interfaceFontsLoaded_) return;
    if (dataRoot.empty()) {
        // Nothing to search is not the same as searching and finding nothing,
        // and both end up in the built-in face.
        LOG_WARNING("No data directory to load interface fonts from - keeping "
                    "the built-in face");
        return;
    }

    namespace fs = std::filesystem;
    std::error_code ec;

    // Extracted data does not agree with itself about case, and this path is
    // reached directly rather than through the asset manager's manifest.
    //
    // Matched a component at a time rather than against a list of spellings.
    // The list only held four, so an install writing Misc/fonts or MISC/FONTS
    // matched none of them, the built-in face was kept, and the only trace was
    // an info line the log does not carry.
    auto childIgnoringCase = [&](const fs::path& base, const std::string& name) {
        fs::path exact = base / name;
        if (fs::exists(exact, ec)) return exact;
        auto lower = [](std::string v) {
            for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return v;
        };
        const std::string wanted = lower(name);
        for (const auto& entry : fs::directory_iterator(base, ec)) {
            if (lower(entry.path().filename().string()) == wanted) return entry.path();
        }
        return fs::path();
    };

    fs::path fontDir;
    for (const char* rel : { "misc/fonts", "fonts" }) {
        fs::path at(dataRoot);
        for (const auto& part : fs::path(rel)) {
            at = childIgnoringCase(at, part.string());
            if (at.empty()) break;
        }
        if (!at.empty() && fs::is_directory(at, ec)) { fontDir = at; break; }
    }
    if (fontDir.empty()) {
        // Said out loud: the client still runs, in a face that is not the
        // game's, and nothing else reports why.
        LOG_WARNING("No interface fonts under ", dataRoot,
                    " - keeping the built-in face, so text will not look right");
        return;
    }

    // Built at a size above what the interface mostly asks for. A font string
    // carries its own height and is drawn scaled from its face, and scaling
    // down from a larger atlas reads better than up from a smaller.
    // The same figure the style was scaled by. Building the atlas at a
    // different one puts text of one size into controls sized for another.
    const float atlasScale = interfaceScale_;
    const float kAtlasSize = 18.0f * atlasScale;

    // What the client's own panels are drawn at. They were laid out against
    // ImGui's built-in face, which is 13 pixels tall, and FRIZQT at the atlas
    // size above would push text out of buttons sized for it. Close enough to
    // the old metrics to keep those layouts intact, in the game's typeface,
    // which is the point.
    const float kClientSize = 15.0f * atlasScale;

    ImGuiIO& io = ImGui::GetIO();

    // Case is not agreed on here either, so look for the file rather than
    // assuming the spelling the manifest happens to use.
    auto resolve = [&](const char* name) {
        fs::path file = fontDir / name;
        if (fs::exists(file, ec)) return file;
        for (const auto& entry : fs::directory_iterator(fontDir, ec)) {
            std::string have = entry.path().filename().string();
            std::transform(have.begin(), have.end(), have.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (have == name) return entry.path();
        }
        return fs::path();
    };

    // FRIZQT at the client's size, first, because ImGui draws with whichever
    // face was added first and that is what everything without an opinion gets
    // - this client's own windows included. The same face is added again below
    // at the atlas size for the interface, which asks for it by name.
    // A font the archives hold, handed to ImGui as bytes. An install that never
    // extracted its data keeps every font inside the MPQs, where the directory
    // walk above sees nothing at all - which is why the same build found them
    // on one machine and not another. The bytes are copied because ImGui takes
    // ownership of the buffer it is given and frees it with its own allocator.
    // Weight, not size. A font string carries its own height and most of
    // FrameXML asks for ten to fourteen, so nearly every label is drawn scaled
    // down from the atlas above - and downscaling an antialiased glyph spreads
    // each stroke over fewer pixels, which reads as faint rather than small.
    // Brightening the rasterized coverage puts the weight back; it is what
    // ImGui offers for exactly this and costs nothing at draw time.
    auto interfaceFontConfig = [] {
        ImFontConfig cfg;
        cfg.RasterizerMultiply = 1.35f;
        return cfg;
    };

    auto addFromArchive = [&](const char* name, float size) -> ImFont* {
        if (!assets) return nullptr;
        auto data = assets->readFileOptional(std::string("Fonts\\") + name);
        if (data.empty()) return nullptr;
        void* owned = IM_ALLOC(data.size());
        std::memcpy(owned, data.data(), data.size());
        ImFontConfig cfg = interfaceFontConfig();
        cfg.FontDataOwnedByAtlas = true;
        return io.Fonts->AddFontFromMemoryTTF(owned, static_cast<int>(data.size()),
                                              size, &cfg);
    };

    const fs::path frizqt = resolve("frizqt__.ttf");
    if (frizqt.empty() && addFromArchive("FRIZQT__.TTF", kClientSize)) {
        LOG_INFO("Interface font read from the archives rather than from disk");
    } else if (!frizqt.empty()) {
        ImFontConfig clientCfg = interfaceFontConfig();
        if (!io.Fonts->AddFontFromFileTTF(frizqt.string().c_str(), kClientSize,
                                          &clientCfg)) {
            // Found and refused is a different problem from not found, and
            // reads identically on screen.
            LOG_WARNING("Could not read the interface font at ", frizqt.string(),
                        " - keeping the built-in face");
            io.Fonts->AddFontDefault();
        }
    } else {
        LOG_WARNING("No frizqt__.ttf in ", fontDir.string(),
                    " - keeping the built-in face");
        io.Fonts->AddFontDefault();
    }

    // The faces FrameXML's font objects name: body text in FRIZQT, headings in
    // MORPHEUS, damage in SKURRI, condensed numbers in ARIALN.
    const char* faces[] = {
        "frizqt__.ttf", "morpheus.ttf", "skurri.ttf", "arialn.ttf", "friends.ttf"
    };
    int loaded = 0;
    for (const char* name : faces) {
        const fs::path file = resolve(name);
        ImFontConfig faceCfg = interfaceFontConfig();
        ImFont* f = file.empty()
            ? nullptr
            : io.Fonts->AddFontFromFileTTF(file.string().c_str(), kAtlasSize,
                                           &faceCfg);
        if (!f) {
            // The archives spell them in upper case, which matters on a
            // filesystem that cares and costs nothing on one that does not.
            std::string upper(name);
            for (char& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            f = addFromArchive(upper.c_str(), kAtlasSize);
        }
        if (f) {
            registerInterfaceFace(name, f);
            ++loaded;
        }
    }
    LOG_WARNING("Interface fonts loaded: ", loaded, " of 5 from ", fontDir.string());
    if (loaded > 0) interfaceFontsLoaded_ = true;
}

void UIManager::shutdown() {
    if (imguiInitialized) {
        auto* vkCtx = window ? window->getVkContext() : nullptr;
        if (vkCtx) {
            vkDeviceWaitIdle(vkCtx->getDevice());
        }

        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        imguiInitialized = false;
    }
    LOG_INFO("UI manager shutdown");
}

void UIManager::update([[maybe_unused]] float deltaTime) {
    if (!imguiInitialized) return;

    // Start ImGui frame
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
}

void UIManager::render(core::AppState appState, auth::AuthHandler* authHandler, game::GameHandler* gameHandler) {
    if (!imguiInitialized) return;

    // Two ~150-200ms spikes land here every launch, before login. Decoding the
    // auth background off the main thread did not move them, so report which
    // application state was being drawn when one happens - that narrows it to a
    // screen before anyone goes looking inside one.
    const auto uiRenderStart = std::chrono::steady_clock::now();
    struct StateReport {
        std::chrono::steady_clock::time_point start;
        core::AppState state;
        ~StateReport() {
            const float ms = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (ms > 50.0f) {
                LOG_WARNING("SLOW UI screen render: ", ms, "ms in appState=",
                            static_cast<int>(state));
            }
        }
    } stateReport{.start = uiRenderStart, .state = appState};

    // Render appropriate screen based on application state
    switch (appState) {
        case core::AppState::AUTHENTICATION:
            if (authHandler) {
                authScreen->render(*authHandler);
            }
            break;

        case core::AppState::REALM_SELECTION:
            authScreen->stopLoginMusic();
            if (authHandler) {
                realmScreen->render(*authHandler);
            }
            break;

        case core::AppState::CHARACTER_CREATION:
            authScreen->stopLoginMusic();
            if (gameHandler) {
                characterCreateScreen->render(*gameHandler);
            }
            break;

        case core::AppState::CHARACTER_SELECTION:
            authScreen->stopLoginMusic();
            if (gameHandler) {
                characterScreen->render(*gameHandler);
            }
            break;

        case core::AppState::IN_GAME:
            authScreen->stopLoginMusic();
            if (gameHandler) {
                gameScreen->render(*gameHandler);
            }
            break;

        case core::AppState::DISCONNECTED:
            authScreen->stopLoginMusic();
            ImGui::SetNextWindowSize(ImVec2(400, 150), ImGuiCond_Always);
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 200,
                                           ImGui::GetIO().DisplaySize.y * 0.5f - 75),
                                    ImGuiCond_Always);
            ImGui::Begin("Disconnected", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
            ImGui::TextWrapped("You have been disconnected from the server.");
            ImGui::Spacing();
            if (ImGui::Button("Return to Login", ImVec2(-1, 0))) {
                // Will be handled by application
            }
            ImGui::End();
            break;
    }

}

void UIManager::finishImGuiFrame() {
    // Finalize ImGui draw data (actual rendering happens in the command buffer).
    //
    // Split out of render() so the application can put something between the
    // two. FrameXML's panels draw into the same background list the nameplates
    // and minimap blips use, and the last thing added to that list is on top,
    // so the panels have to go in after this stage has drawn the world's
    // overlays - and before the draw data is closed, which is here.
    if (!imguiInitialized) return;
#ifdef __ANDROID__
    // The SDL backend stopped calling these deliberately (imgui #6306), because
    // on a desktop they only pertain to IME. On Android they are what raises
    // and lowers the on-screen keyboard, so without them a text box takes
    // focus, shows a caret, and there is no way to type into it.
    //
    // Read after the frame is built, so it reflects the box the player just
    // touched rather than the one they touched last frame.
    if (const bool wantsText = ImGui::GetIO().WantTextInput; wantsText != softKeyboardUp_) {
        if (wantsText) {
            SDL_StartTextInput();
        } else {
            SDL_StopTextInput();
        }
        softKeyboardUp_ = wantsText;
    }
#endif

    ImGui::Render();
}

void UIManager::processEvent(const SDL_Event& event) {
    if (imguiInitialized) {
        ImGui_ImplSDL2_ProcessEvent(&event);
    }
}

} // namespace ui
} // namespace wowee
