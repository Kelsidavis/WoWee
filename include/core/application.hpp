#pragma once

#include "core/window.hpp"
#include "core/input.hpp"
#include "game/character.hpp"
#include <memory>
#include <string>
#include <vector>

namespace wowee {

// Forward declarations
namespace rendering { class Renderer; }
namespace ui { class UIManager; }
namespace auth { class AuthHandler; }
namespace game { class GameHandler; class World; class NpcManager; }
namespace pipeline { class AssetManager; }

namespace core {

enum class AppState {
    AUTHENTICATION,
    REALM_SELECTION,
    CHARACTER_CREATION,
    CHARACTER_SELECTION,
    IN_GAME,
    DISCONNECTED
};

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    bool initialize();
    void run();
    void shutdown();

    // State management
    AppState getState() const { return state; }
    void setState(AppState newState);

    // Accessors
    Window* getWindow() { return window.get(); }
    rendering::Renderer* getRenderer() { return renderer.get(); }
    ui::UIManager* getUIManager() { return uiManager.get(); }
    auth::AuthHandler* getAuthHandler() { return authHandler.get(); }
    game::GameHandler* getGameHandler() { return gameHandler.get(); }
    game::World* getWorld() { return world.get(); }
    pipeline::AssetManager* getAssetManager() { return assetManager.get(); }

    // Singleton access
    static Application& getInstance() { return *instance; }

    // Single-player mode
    void startSinglePlayer();
    bool isSinglePlayer() const { return singlePlayerMode; }

    // Weapon loading (called at spawn and on equipment change)
    void loadEquippedWeapons();

    // Teleport to a spawn preset location (single-player only)
    void teleportTo(int presetIndex);

    // Character skin composite state (saved at spawn for re-compositing on equipment change)
    const std::string& getBodySkinPath() const { return bodySkinPath_; }
    const std::vector<std::string>& getUnderwearPaths() const { return underwearPaths_; }
    uint32_t getSkinTextureSlotIndex() const { return skinTextureSlotIndex_; }
    uint32_t getCloakTextureSlotIndex() const { return cloakTextureSlotIndex_; }

private:
    void update(float deltaTime);
    void render();
    void setupUICallbacks();
    void spawnPlayerCharacter();
    void spawnNpcs();
    std::string getPlayerModelPath() const;
    static const char* mapIdToName(uint32_t mapId);

    static Application* instance;

    std::unique_ptr<Window> window;
    std::unique_ptr<rendering::Renderer> renderer;
    std::unique_ptr<ui::UIManager> uiManager;
    std::unique_ptr<auth::AuthHandler> authHandler;
    std::unique_ptr<game::GameHandler> gameHandler;
    std::unique_ptr<game::World> world;
    std::unique_ptr<game::NpcManager> npcManager;
    std::unique_ptr<pipeline::AssetManager> assetManager;

    AppState state = AppState::AUTHENTICATION;
    bool running = false;
    bool singlePlayerMode = false;
    bool playerCharacterSpawned = false;
    bool npcsSpawned = false;
    bool spawnSnapToGround = true;
    float lastFrameTime = 0.0f;
    float movementHeartbeatTimer = 0.0f;
    game::Race spRace_ = game::Race::HUMAN;
    game::Gender spGender_ = game::Gender::MALE;
    game::Class spClass_ = game::Class::WARRIOR;
    uint32_t spMapId_ = 0;
    uint32_t spZoneId_ = 0;
    glm::vec3 spSpawnCanonical_ = glm::vec3(62.0f, -9464.0f, 200.0f);
    float spYawDeg_ = 0.0f;
    float spPitchDeg_ = -5.0f;

    // Weapon model ID counter (starting high to avoid collision with character model IDs)
    uint32_t nextWeaponModelId_ = 1000;

    // Saved at spawn for skin re-compositing
    std::string bodySkinPath_;
    std::vector<std::string> underwearPaths_;
    uint32_t skinTextureSlotIndex_ = 0;
    uint32_t cloakTextureSlotIndex_ = 0;
};

} // namespace core
} // namespace wowee
