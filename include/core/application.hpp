#pragma once

#include "core/window.hpp"
#include "core/input.hpp"
#include "game/character.hpp"
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <array>

namespace wowee {

// Forward declarations
namespace rendering { class Renderer; }
namespace ui { class UIManager; }
namespace auth { class AuthHandler; }
namespace game { class GameHandler; class World; class ExpansionRegistry; }
namespace pipeline { class AssetManager; class DBCLayout; }
namespace audio { enum class VoiceType; }

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
    game::ExpansionRegistry* getExpansionRegistry() { return expansionRegistry_.get(); }
    pipeline::DBCLayout* getDBCLayout() { return dbcLayout_.get(); }
    void reloadExpansionData(); // Reload DBC layouts, opcodes, etc. after expansion change

    // Singleton access
    static Application& getInstance() { return *instance; }

    // Weapon loading (called at spawn and on equipment change)
    void loadEquippedWeapons();

    // Logout to login screen
    void logoutToLogin();

    // Render bounds lookup (for click targeting / selection)
    bool getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const;
    bool getRenderFootZForGuid(uint64_t guid, float& outFootZ) const;

    // Character skin composite state (saved at spawn for re-compositing on equipment change)
    const std::string& getBodySkinPath() const { return bodySkinPath_; }
    const std::vector<std::string>& getUnderwearPaths() const { return underwearPaths_; }
    uint32_t getSkinTextureSlotIndex() const { return skinTextureSlotIndex_; }
    uint32_t getCloakTextureSlotIndex() const { return cloakTextureSlotIndex_; }
    uint32_t getGryphonDisplayId() const { return gryphonDisplayId_; }
    uint32_t getWyvernDisplayId() const { return wyvernDisplayId_; }

private:
    void update(float deltaTime);
    void render();
    void setupUICallbacks();
    void spawnPlayerCharacter();
    std::string getPlayerModelPath() const;
    static const char* mapIdToName(uint32_t mapId);
    void loadOnlineWorldTerrain(uint32_t mapId, float x, float y, float z);
    void buildFactionHostilityMap(uint8_t playerRace);
    void spawnOnlineCreature(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation);
    void despawnOnlineCreature(uint64_t guid);
    bool tryAttachCreatureVirtualWeapons(uint64_t guid, uint32_t instanceId);
    void spawnOnlinePlayer(uint64_t guid,
                           uint8_t raceId,
                           uint8_t genderId,
                           uint32_t appearanceBytes,
                           uint8_t facialFeatures,
                           float x, float y, float z, float orientation);
    void setOnlinePlayerEquipment(uint64_t guid,
                                  const std::array<uint32_t, 19>& displayInfoIds,
                                  const std::array<uint8_t, 19>& inventoryTypes);
    void despawnOnlinePlayer(uint64_t guid);
    void buildCreatureDisplayLookups();
    std::string getModelPathForDisplayId(uint32_t displayId) const;
    void spawnOnlineGameObject(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation);
    void despawnOnlineGameObject(uint64_t guid);
    void buildGameObjectDisplayLookups();
    std::string getGameObjectModelPathForDisplayId(uint32_t displayId) const;
    audio::VoiceType detectVoiceTypeFromDisplayId(uint32_t displayId) const;
    void setupTestTransport();  // Test transport boat for development

    static Application* instance;

    std::unique_ptr<Window> window;
    std::unique_ptr<rendering::Renderer> renderer;
    std::unique_ptr<ui::UIManager> uiManager;
    std::unique_ptr<auth::AuthHandler> authHandler;
    std::unique_ptr<game::GameHandler> gameHandler;
    std::unique_ptr<game::World> world;
    std::unique_ptr<pipeline::AssetManager> assetManager;
    std::unique_ptr<game::ExpansionRegistry> expansionRegistry_;
    std::unique_ptr<pipeline::DBCLayout> dbcLayout_;

    AppState state = AppState::AUTHENTICATION;
    bool running = false;
    std::string pendingCreatedCharacterName_;  // Auto-select after character creation
    bool playerCharacterSpawned = false;
    bool npcsSpawned = false;
    bool spawnSnapToGround = true;
    float lastFrameTime = 0.0f;

    // Player character info (for model spawning)
    game::Race playerRace_ = game::Race::HUMAN;
    game::Gender playerGender_ = game::Gender::MALE;
    game::Class playerClass_ = game::Class::WARRIOR;
    uint64_t spawnedPlayerGuid_ = 0;
    uint32_t spawnedAppearanceBytes_ = 0;
    uint8_t spawnedFacialFeatures_ = 0;

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
    std::unordered_map<uint64_t, glm::vec3> creatureRenderPosCache_; // guid -> last synced render position
    std::unordered_set<uint64_t> creatureWeaponsAttached_;       // guid set when NPC virtual weapons attached
    std::unordered_map<uint64_t, uint8_t> creatureWeaponAttachAttempts_; // guid -> attach attempts
    std::unordered_set<uint64_t> deadCreatureGuids_;            // GUIDs that should spawn in corpse/death pose
    std::unordered_map<uint32_t, uint32_t> displayIdModelCache_; // displayId → modelId (model caching)
    uint32_t nextCreatureModelId_ = 5000;  // Model IDs for online creatures
    uint32_t gryphonDisplayId_ = 0;
    uint32_t wyvernDisplayId_ = 0;
    bool lastTaxiFlight_ = false;
    uint32_t loadedMapId_ = 0xFFFFFFFF;  // Map ID of currently loaded terrain (0xFFFFFFFF = none)
    float taxiLandingClampTimer_ = 0.0f;
    float worldEntryMovementGraceTimer_ = 0.0f;
    float facingSendCooldown_ = 0.0f;        // Rate-limits MSG_MOVE_SET_FACING
    float lastSentCanonicalYaw_ = 1000.0f;   // Sentinel — triggers first send
    float taxiStreamCooldown_ = 0.0f;
    bool idleYawned_ = false;

    // Charge rush state
    bool chargeActive_ = false;
    float chargeTimer_ = 0.0f;
    float chargeDuration_ = 0.0f;
    glm::vec3 chargeStartPos_{0.0f};  // Render coordinates
    glm::vec3 chargeEndPos_{0.0f};    // Render coordinates
    uint64_t chargeTargetGuid_ = 0;

    // Online gameobject model spawning
    struct GameObjectInstanceInfo {
        uint32_t modelId = 0;
        uint32_t instanceId = 0;
        bool isWmo = false;
    };
    std::unordered_map<uint32_t, std::string> gameObjectDisplayIdToPath_;
    std::unordered_map<uint32_t, uint32_t> gameObjectDisplayIdModelCache_; // displayId → M2 modelId
    std::unordered_map<uint32_t, uint32_t> gameObjectDisplayIdWmoCache_;   // displayId → WMO modelId
    std::unordered_map<uint64_t, GameObjectInstanceInfo> gameObjectInstances_; // guid → instance info
    struct PendingTransportMove {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float orientation = 0.0f;
    };
    std::unordered_map<uint64_t, PendingTransportMove> pendingTransportMoves_; // guid -> latest pre-registration move
    uint32_t nextGameObjectModelId_ = 20000;
    uint32_t nextGameObjectWmoModelId_ = 40000;
    bool testTransportSetup_ = false;
    bool gameObjectLookupsBuilt_ = false;

    // Mount model tracking
    uint32_t mountInstanceId_ = 0;
    uint32_t mountModelId_ = 0;
    uint32_t pendingMountDisplayId_ = 0;  // Deferred mount load (0 = none pending)
    bool weaponsSheathed_ = false;
    bool wasAutoAttacking_ = false;
    void processPendingMount();
    bool creatureLookupsBuilt_ = false;
    bool mapNameCacheLoaded_ = false;
    std::unordered_map<uint32_t, std::string> mapNameById_;

    // Deferred creature spawn queue (throttles spawning to avoid hangs)
    struct PendingCreatureSpawn {
        uint64_t guid;
        uint32_t displayId;
        float x, y, z, orientation;
    };
    std::vector<PendingCreatureSpawn> pendingCreatureSpawns_;
    static constexpr int MAX_SPAWNS_PER_FRAME = 8;
    static constexpr int MAX_NEW_CREATURE_MODELS_PER_FRAME = 1;
    static constexpr uint16_t MAX_CREATURE_SPAWN_RETRIES = 300;
    std::unordered_set<uint64_t> pendingCreatureSpawnGuids_;
    std::unordered_map<uint64_t, uint16_t> creatureSpawnRetryCounts_;
    std::unordered_set<uint32_t> nonRenderableCreatureDisplayIds_;

    // Online player instances (separate from creatures so we can apply per-player skin/hair textures).
    std::unordered_map<uint64_t, uint32_t> playerInstances_;  // guid → render instanceId
    struct OnlinePlayerAppearanceState {
        uint32_t instanceId = 0;
        uint32_t modelId = 0;
        uint8_t raceId = 0;
        uint8_t genderId = 0;
        uint32_t appearanceBytes = 0;
        uint8_t facialFeatures = 0;
        std::string bodySkinPath;
        std::vector<std::string> underwearPaths;
    };
    std::unordered_map<uint64_t, OnlinePlayerAppearanceState> onlinePlayerAppearance_;
    std::unordered_map<uint64_t, std::pair<std::array<uint32_t, 19>, std::array<uint8_t, 19>>> pendingOnlinePlayerEquipment_;
    // Deferred equipment compositing queue — processes max 1 per frame to avoid stutter
    std::vector<std::pair<uint64_t, std::pair<std::array<uint32_t, 19>, std::array<uint8_t, 19>>>> deferredEquipmentQueue_;
    void processDeferredEquipmentQueue();
    // Cache base player model geometry by (raceId, genderId)
    std::unordered_map<uint32_t, uint32_t> playerModelCache_; // key=(race<<8)|gender → modelId
    struct PlayerTextureSlots { int skin = -1; int hair = -1; int underwear = -1; };
    std::unordered_map<uint32_t, PlayerTextureSlots> playerTextureSlotsByModelId_;
    uint32_t nextPlayerModelId_ = 60000;
    struct PendingPlayerSpawn {
        uint64_t guid;
        uint8_t raceId;
        uint8_t genderId;
        uint32_t appearanceBytes;
        uint8_t facialFeatures;
        float x, y, z, orientation;
    };
    std::vector<PendingPlayerSpawn> pendingPlayerSpawns_;
    std::unordered_set<uint64_t> pendingPlayerSpawnGuids_;
    void processPlayerSpawnQueue();
    std::unordered_set<uint64_t> creaturePermanentFailureGuids_;
    void processCreatureSpawnQueue();

    struct PendingGameObjectSpawn {
        uint64_t guid;
        uint32_t entry;
        uint32_t displayId;
        float x, y, z, orientation;
    };
    std::vector<PendingGameObjectSpawn> pendingGameObjectSpawns_;
    void processGameObjectSpawnQueue();
    struct PendingTransportDoodadBatch {
        uint64_t guid = 0;
        uint32_t modelId = 0;
        uint32_t instanceId = 0;
        size_t nextIndex = 0;
        size_t doodadBudget = 0;
        size_t spawnedDoodads = 0;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float orientation = 0.0f;
    };
    std::vector<PendingTransportDoodadBatch> pendingTransportDoodadBatches_;
    static constexpr size_t MAX_TRANSPORT_DOODADS_PER_FRAME = 4;
    void processPendingTransportDoodads();

    // Quest marker billboard sprites (above NPCs)
    void loadQuestMarkerModels();  // Now loads BLP textures
    void updateQuestMarkers();     // Updates billboard positions
};

} // namespace core
} // namespace wowee
