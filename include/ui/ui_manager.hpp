#pragma once

#include "ui/auth_screen.hpp"
#include "ui/realm_screen.hpp"
#include "ui/character_create_screen.hpp"
#include "ui/character_screen.hpp"
#include "ui/game_screen.hpp"
#include <memory>

// Forward declare SDL_Event
union SDL_Event;

namespace wowee {

// Forward declarations
namespace core { class Window; enum class AppState; }
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
    void processEvent(const SDL_Event& event);

    /**
     * Get screen instances for callback setup
     */
    AuthScreen& getAuthScreen() { return *authScreen; }
    RealmScreen& getRealmScreen() { return *realmScreen; }
    CharacterCreateScreen& getCharacterCreateScreen() { return *characterCreateScreen; }
    CharacterScreen& getCharacterScreen() { return *characterScreen; }
    GameScreen& getGameScreen() { return *gameScreen; }

private:
    core::Window* window = nullptr;

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
