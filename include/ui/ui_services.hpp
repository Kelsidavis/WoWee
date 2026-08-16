#pragma once

namespace wowee {

// Forward declarations
namespace core { 
    class Window;
    class EntitySpawner;
    class AppearanceComposer;
    class WorldLoader;
}
namespace rendering { class Renderer; }
namespace pipeline { class AssetManager; }
namespace game { 
    class GameHandler;
    class ExpansionRegistry;
}
namespace addons { class AddonManager; }
namespace audio { class AudioCoordinator; }

namespace ui {

/**
 * UI Services - Dependency injection container for UI components.
 * 
 * Break the singleton Phase B
 * 
 * Replaces Application::getInstance() calls throughout UI code.
 * Application creates this struct and injects it into UIManager,
 * which propagates it to GameScreen and all child UI components.
 * 
 * Owned by Application, shared as const pointers (non-owning).
 */
struct UIServices {
    core::Window* window = nullptr;
    rendering::Renderer* renderer = nullptr;
    pipeline::AssetManager* assetManager = nullptr;
    game::GameHandler* gameHandler = nullptr;
    game::ExpansionRegistry* expansionRegistry = nullptr;
    addons::AddonManager* addonManager = nullptr;
    audio::AudioCoordinator* audioCoordinator = nullptr;
    
    // Extracted classes (also available individually for Phase A compatibility)
    core::EntitySpawner* entitySpawner = nullptr;
    core::AppearanceComposer* appearanceComposer = nullptr;
    core::WorldLoader* worldLoader = nullptr;
    
    // Helper to check if core services are wired
    [[nodiscard]] bool isValid() const {
        return window && renderer && assetManager && gameHandler;
    }
};

} // namespace ui
} // namespace wowee
