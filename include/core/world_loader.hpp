#pragma once

#include <functional>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <memory>
#include <atomic>
#include <thread>
#include <cstdint>

namespace wowee {

namespace rendering { class Renderer; class LoadingScreen; }
namespace pipeline { class AssetManager; class DBCLayout; }
namespace game { class GameHandler; class World; }
namespace addons { class AddonManager; }

namespace core {

class Application;
class EntitySpawner;
class AppearanceComposer;
class Window;

/// Handles terrain streaming, map transitions, world preloading,
/// and coordinate-aware tile management for online world entry.
class WorldLoader {
public:
    WorldLoader(Application& app,
                rendering::Renderer* renderer,
                pipeline::AssetManager* assetManager,
                game::GameHandler* gameHandler,
                EntitySpawner* entitySpawner,
                AppearanceComposer* appearanceComposer,
                Window* window,
                game::World* world,
                addons::AddonManager* addonManager);
    ~WorldLoader();

    // Main terrain loading - drives loading screen, WMO/ADT detection, player spawn
    void loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z);

    // Process deferred world entry (called from Application::update each frame)
    void processPendingEntry();

    // Map name utilities
    static const char* mapIdToName(uint32_t mapId);
    static const char* mapDisplayName(uint32_t mapId);

    // Background preloading - warms AssetManager file cache
    void startWorldPreload(uint32_t mapId, const std::string& mapName,
                           float serverX, float serverY);
    void cancelWorldPreload();

    // Persistent world info for session-to-session preloading
    void saveLastWorldInfo(uint32_t mapId, const std::string& mapName,
                           float serverX, float serverY);
    struct LastWorldInfo {
        uint32_t mapId = 0;
        std::string mapName;
        float x = 0, y = 0;
        bool valid = false;
    };
    [[nodiscard]] LastWorldInfo loadLastWorldInfo() const;

    // State accessors
    [[nodiscard]] uint32_t getLoadedMapId() const { return loadedMapId_; }
    [[nodiscard]] bool isLoadingWorld() const { return loadingWorld_; }
    [[nodiscard]] bool hasPendingEntry() const { return pendingWorldEntry_.has_value(); }

    // Get cached map name by ID (returns empty string if not found)
    [[nodiscard]] std::string getMapNameById(uint32_t mapId) const {
        auto it = mapNameById_.find(mapId);
        return (it != mapNameById_.end()) ? it->second : std::string{};
    }

    // Set pending world entry for deferred processing via processPendingEntry()
    void setPendingEntry(uint32_t mapId, float x, float y, float z) {
        pendingWorldEntry_ = PendingWorldEntry{.mapId = mapId, .x = x, .y = y, .z = z};
    }

    // Reset methods (for logout / character switch)
    void resetLoadedMap() { loadedMapId_ = 0xFFFFFFFF; }
    void resetMapNameCache() { mapNameCacheLoaded_ = false; mapNameById_.clear(); }

private:
    /// The loading screen, while a world is being loaded into.
    ///
    /// Loading a map takes seconds, and the screen has to keep drawing through
    /// it or the window stops answering the compositor and the desktop paints it
    /// as hung. So every slow step pumps it, and pumping was written out as
    /// `if (ok) { screen.render(); window->swapBuffers(); }` wherever a step was
    /// slow enough to need it.
    ///
    /// `ok` is false when the screen could not be set up at all - the world
    /// still loads, it is just loaded blind.
    struct LoadingUi {
        rendering::LoadingScreen* screen = nullptr;
        Window* window = nullptr;
        bool ok = false;
        std::function<void(const char*, float)> showProgress;

        /// Draw one frame of the loading screen, if there is one.
        void pump() const;
    };

    /// Read the map's WDT, then load either its root WMO or its terrain tiles,
    /// and precompute what the player will stand on. The longest step of a world
    /// load by far, which is why it takes the loading screen with it.
    void loadMapGeometry(uint32_t mapId, const std::string& mapName,
                         const glm::vec3& spawnCanonical, const glm::vec3& spawnRender,
                         const LoadingUi& ui);

    Application& app_;
    rendering::Renderer* renderer_;
    pipeline::AssetManager* assetManager_;
    game::GameHandler* gameHandler_;
    EntitySpawner* entitySpawner_;
    AppearanceComposer* appearanceComposer_;
    Window* window_;
    game::World* world_;
    addons::AddonManager* addonManager_;

    uint32_t loadedMapId_ = 0xFFFFFFFF;  // Map ID of currently loaded terrain (0xFFFFFFFF = none)
    bool loadingWorld_ = false;          // True while loadOnlineWorldTerrain is running

    struct PendingWorldEntry {
        uint32_t mapId; float x, y, z;
    };
    std::optional<PendingWorldEntry> pendingWorldEntry_;

    // Map.dbc name cache (loaded once per session)
    bool mapNameCacheLoaded_ = false;
    std::unordered_map<uint32_t, std::string> mapNameById_;

    // Background world preloader - warms AssetManager file cache for the
    // expected world before the user clicks Enter World.
    struct WorldPreload {
        uint32_t mapId = 0;
        std::string mapName;
        int centerTileX = 0;
        int centerTileY = 0;
        std::atomic<bool> cancel{false};
        std::vector<std::thread> workers;
    };
    std::unique_ptr<WorldPreload> worldPreload_;
};

} // namespace core
} // namespace wowee
