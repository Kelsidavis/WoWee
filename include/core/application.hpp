#pragma once

#include "core/window.hpp"
#include "core/input.hpp"
#include "game/character.hpp"
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

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

    // Weapon loading (called at spawn and on equipment change)
    void loadEquippedWeapons();

    // Logout to login screen
    void logoutToLogin();

    // Render bounds lookup (for click targeting / selection)
    bool getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const;

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
    void loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z);
    void buildFactionHostilityMap(uint8_t playerRace);
    void spawnOnlineCreature(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation);
    void despawnOnlineCreature(uint64_t guid);
    void buildCreatureDisplayLookups();
    std::string getModelPathForDisplayId(uint32_t displayId) const;

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
    bool playerCharacterSpawned = false;
    bool npcsSpawned = false;
    bool spawnSnapToGround = true;
    float lastFrameTime = 0.0f;
    float movementHeartbeatTimer = 0.0f;

    // Weapon model ID counter (starting high to avoid collision with character model IDs)
    uint32_t nextWeaponModelId_ = 1000;

    // Saved at spawn for skin re-compositing
    std::string bodySkinPath_;
    std::vector<std::string> underwearPaths_;
    uint32_t skinTextureSlotIndex_ = 0;
    uint32_t cloakTextureSlotIndex_ = 0;

    // Online creature model spawning
    struct CreatureDisplayData {
        uint32_t modelId = 0;
        std::string skin1, skin2, skin3;  // Texture names from CreatureDisplayInfo.dbc
        uint32_t extraDisplayId = 0;      // Link to CreatureDisplayInfoExtra.dbc
    };
    struct HumanoidDisplayExtra {
        uint8_t raceId = 0;
        uint8_t sexId = 0;
        uint8_t skinId = 0;
        uint8_t faceId = 0;
        uint8_t hairStyleId = 0;
        uint8_t hairColorId = 0;
        uint8_t facialHairId = 0;
        std::string bakeName;  // Pre-baked texture path if available
        // Equipment display IDs (from columns 8-18)
        // 0=helm, 1=shoulder, 2=shirt, 3=chest, 4=belt, 5=legs, 6=feet, 7=wrist, 8=hands, 9=tabard, 10=cape
        uint32_t equipDisplayId[11] = {0};
    };
    std::unordered_map<uint32_t, CreatureDisplayData> displayDataMap_;  // displayId → display data
    std::unordered_map<uint32_t, HumanoidDisplayExtra> humanoidExtraMap_;  // extraDisplayId → humanoid data
    std::unordered_map<uint32_t, std::string> modelIdToPath_;   // modelId → M2 path (from CreatureModelData.dbc)
    // CharHairGeosets.dbc: key = (raceId<<16)|(sexId<<8)|variationId → geosetId (skinSectionId)
    std::unordered_map<uint32_t, uint16_t> hairGeosetMap_;
    // CharFacialHairStyles.dbc: key = (raceId<<16)|(sexId<<8)|variationId → {geoset100, geoset300, geoset200}
    struct FacialHairGeosets { uint16_t geoset100 = 0; uint16_t geoset300 = 0; uint16_t geoset200 = 0; };
    std::unordered_map<uint32_t, FacialHairGeosets> facialHairGeosetMap_;
    std::unordered_map<uint64_t, uint32_t> creatureInstances_;  // guid → render instanceId
    std::unordered_map<uint64_t, uint32_t> creatureModelIds_;   // guid → loaded modelId
    std::unordered_map<uint32_t, uint32_t> displayIdModelCache_; // displayId → modelId (model caching)
    uint32_t nextCreatureModelId_ = 5000;  // Model IDs for online creatures
    bool creatureLookupsBuilt_ = false;

    // Deferred creature spawn queue (throttles spawning to avoid hangs)
    struct PendingCreatureSpawn {
        uint64_t guid;
        uint32_t displayId;
        float x, y, z, orientation;
    };
    std::vector<PendingCreatureSpawn> pendingCreatureSpawns_;
    static constexpr int MAX_SPAWNS_PER_FRAME = 2;
    void processCreatureSpawnQueue();
};

} // namespace core
} // namespace wowee
