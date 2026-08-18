#pragma once

#include "ui/auth_screen.hpp"
#include "ui/realm_screen.hpp"
#include "ui/character_create_screen.hpp"
#include "ui/character_screen.hpp"
#include "ui/game_screen.hpp"
#include "ui/ui_services.hpp"
#include <memory>
#include <string>

// Forward declare SDL_Event
union SDL_Event;

namespace wowee {
namespace pipeline { class AssetManager; }

// Forward declarations
namespace core { class Window; class AppearanceComposer; enum class AppState; }
namespace auth { class AuthHandler; }
namespace game { class GameHandler; }

namespace ui {

/**
 * UIManager - Manages all UI screens and ImGui rendering
 *
 * Coordinates screen transitions and rendering based on application state
 */
class UIManager {
public:
    UIManager();
    ~UIManager();

    /**
     * Initialize ImGui and UI screens
     * @param window Window instance for ImGui initialization
     */
    bool initialize(core::Window* window);

    /// Loads the game's own interface font, if it is in the data.
    ///
    /// Separate from initialize because the asset path is not settled until
    /// after the expansion profile is chosen, and separate from drawing because
    /// the glyph atlas is built once, before the first frame - adding a face
    /// afterwards means tearing the font texture down and rebuilding it, which
    /// cannot happen while a frame is in flight.
    /// Load the interface typefaces, from loose files or from the archives.
    ///
    /// `assets` may be null, and is only consulted when nothing was found on
    /// disk: an install that never extracted its data keeps the fonts inside
    /// the MPQs, where std::filesystem cannot see them. That is why this used
    /// to work on one machine and not another with the same build - the case
    /// of the directory was never the whole story.
    void loadInterfaceFont(const std::string& dataRoot,
                           pipeline::AssetManager* assets = nullptr);
    /// Whether a face has already been taken; a second call is a no-op.
    bool interfaceFontsLoaded_ = false;

    /**
     * Shutdown ImGui and cleanup
     */
    void shutdown();

    /**
     * Update UI state
     * @param deltaTime Time since last frame in seconds
     */
    void update(float deltaTime);

    /**
     * Render UI based on current application state
     * @param appState Current application state
     * @param authHandler Authentication handler reference
     * @param gameHandler Game handler reference
     */
    void render(core::AppState appState, auth::AuthHandler* authHandler, game::GameHandler* gameHandler);

    /**
     * Process SDL event for ImGui
     * @param event SDL event to process
     */
    /// Close the ImGui frame. Separate from render() so the application can
    /// draw FrameXML's panels between the two - they belong over the world
    /// overlays that render() puts in the same draw list.
    void finishImGuiFrame();

    void processEvent(const SDL_Event& event);

    /**
     * Get screen instances for callback setup
     */
    AuthScreen& getAuthScreen() { return *authScreen; }
    RealmScreen& getRealmScreen() { return *realmScreen; }
    CharacterCreateScreen& getCharacterCreateScreen() { return *characterCreateScreen; }
    CharacterScreen& getCharacterScreen() { return *characterScreen; }
    GameScreen& getGameScreen() { return *gameScreen; }

    // Dependency injection forwarding (Phase A singleton breaking)
    void setAppearanceComposer(core::AppearanceComposer* ac) {
        if (gameScreen) gameScreen->setAppearanceComposer(ac);
    }

    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services) {
        services_ = services;
        if (gameScreen) gameScreen->setServices(services);
        if (authScreen) authScreen->setServices(services);
        if (characterScreen) characterScreen->setServices(services);
    }
    [[nodiscard]] const UIServices& getServices() const { return services_; }

private:
#ifdef __ANDROID__
    /// Whether the on-screen keyboard is up, so it is raised and lowered once
    /// per change rather than every frame. Declared only where it is read:
    /// clang makes an unused private field an error, and the desktop builds
    /// that use gcc did not say so.
    bool softKeyboardUp_ = false;
#endif

    /// How much bigger than a desktop layout this display needs the interface
    /// drawn. Decided once at start-up; the style and the font atlas both use
    /// it, and they have to agree. 1.0 off Android.
    float interfaceScale_ = 1.0f;
    core::Window* window = nullptr;
    UIServices services_;  // Injected services

    // UI Screens
    std::unique_ptr<AuthScreen> authScreen;
    std::unique_ptr<RealmScreen> realmScreen;
    std::unique_ptr<CharacterCreateScreen> characterCreateScreen;
    std::unique_ptr<CharacterScreen> characterScreen;
    std::unique_ptr<GameScreen> gameScreen;

    // ImGui state
    bool imguiInitialized = false;
};

} // namespace ui
} // namespace wowee
