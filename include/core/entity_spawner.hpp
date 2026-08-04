#pragma once

#include "game/character.hpp"
#include "game/game_services.hpp"
#include "pipeline/blp_loader.hpp"
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <chrono>
#include <optional>
#include <future>
#include <mutex>
#include <glm/glm.hpp>

namespace wowee {

// Forward declarations
namespace rendering { class Renderer; }
namespace pipeline { class AssetManager; class DBCLayout; struct M2Model; struct WMOModel; }
namespace audio { enum class VoiceType; }
namespace game { class GameHandler; }

namespace core {

class Application;

class EntitySpawner {
public:
    EntitySpawner(rendering::Renderer* renderer,
                  pipeline::AssetManager* assetManager,
                  game::GameHandler* gameHandler,
                  pipeline::DBCLayout* dbcLayout,
                  game::GameServices* gameServices);
    ~EntitySpawner();

    EntitySpawner(const EntitySpawner&) = delete;
    EntitySpawner& operator=(const EntitySpawner&) = delete;

    // Lifecycle
    void initialize();   // Build DBC lookups
    void update();       // Process all spawn/despawn queues (called from Application::update)
    void shutdown();     // Clear all instances and queues

    // Queue-based spawn API (called from GameHandler callbacks)
    void queueCreatureSpawn(uint64_t guid, uint32_t displayId,
                            float x, float y, float z, float orientation, float scale = 1.0f);
    void queuePlayerSpawn(uint64_t guid, uint8_t raceId, uint8_t genderId,
                          uint32_t appearanceBytes, uint8_t facialFeatures,
                          float x, float y, float z, float orientation);
    void queueGameObjectSpawn(uint64_t guid, uint32_t entry, uint32_t displayId,
                              float x, float y, float z, float orientation, float scale = 1.0f);
    void queuePlayerEquipment(uint64_t guid,
                              const std::array<uint32_t, 19>& displayInfoIds,
                              const std::array<uint8_t, 19>& inventoryTypes);

    // Immediate despawn
    void despawnCreature(uint64_t guid);
    void despawnPlayer(uint64_t guid);
    void despawnGameObject(uint64_t guid);

    // Re-evaluate the animation policy for spawned game objects of this entry.
    // Called when a GAMEOBJECT_QUERY_RESPONSE finally supplies the type that the
    // policy depends on; harmless if nothing of that entry is waiting.
    void onGameObjectInfoReceived(uint32_t entry);

private:
    // Freeze or animate a spawned game object based on its type, deferring the
    // decision when the type has not been queried yet.
    void applyGameObjectAnimationPolicy(uint64_t guid, uint32_t entry, uint32_t instanceId);

public:

    // Clear all queues and instances (logout, reconnect)
    void clearAllQueues();
    void despawnAllCreatures();
    void despawnAllPlayers();
    void despawnAllGameObjects();

    // Full reset — despawns entities, waits for async loads, clears all state.
    // Used by logoutToLogin() and loadOnlineWorldTerrain() for clean slate.
    void resetAllState();

    // Rebuild lookup tables (for reloadExpansionData after expansion change)
    void rebuildLookups();

    // Check if any spawn/async work is still pending
    bool hasWorkPending() const;

    // Status queries
    bool isCreatureSpawned(uint64_t guid) const { return creatureInstances_.count(guid) > 0; }
    bool isCreaturePending(uint64_t guid) const { return pendingCreatureSpawnGuids_.count(guid) > 0; }
    bool isPlayerSpawned(uint64_t guid) const { return playerInstances_.count(guid) > 0; }
    bool isPlayerPending(uint64_t guid) const { return pendingPlayerSpawnGuids_.count(guid) > 0; }
    bool isGameObjectSpawned(uint64_t guid) const { return gameObjectInstances_.count(guid) > 0; }

    // Quick instance ID lookups (returns 0 if not found)
    uint32_t getCreatureInstanceId(uint64_t guid) const {
        auto it = creatureInstances_.find(guid); return (it != creatureInstances_.end()) ? it->second : 0;
    }
    uint32_t getPlayerInstanceId(uint64_t guid) const {
        auto it = playerInstances_.find(guid); return (it != playerInstances_.end()) ? it->second : 0;
    }

    // Render bounds/position queries (used by click targeting, etc.)
    bool getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const;
    bool getRenderFootZForGuid(uint64_t guid, float& outFootZ) const;
    bool getRenderPositionForGuid(uint64_t guid, glm::vec3& outPos) const;

    // Display data lookups
    bool areCreatureLookupsBuilt() const { return creatureLookupsBuilt_; }
    bool areGameObjectLookupsBuilt() const { return gameObjectLookupsBuilt_; }
    std::string getModelPathForDisplayId(uint32_t displayId) const;
    std::string getGameObjectModelPathForDisplayId(uint32_t displayId) const;

    /// The hull a transport is drawn with, or "" to use the display lookup.
    ///
    /// A preloaded transport can be spawned before its displayId is known, so
    /// the vehicle has to be recognisable from its GameObject entry too.
    ///
    /// One function because there were two copies of this table, in
    /// entity_spawner_player.cpp and entity_spawner_processing.cpp, and the
    /// queued path in the second is the one most spawns actually take —
    /// so fixing the first left the bug on screen.
    static std::string transportModelPath(uint32_t entry, uint32_t displayId);
    audio::VoiceType detectVoiceTypeFromDisplayId(uint32_t displayId) const;

    // Attempts one deferred attachment and owns retry bookkeeping. Returns true
    // only when this call consumed the caller's per-frame attachment budget.
    bool retryCreatureVirtualWeapons(uint64_t guid, uint32_t instanceId,
                                     uint8_t maxAttempts);

    // Mount
    void setMountDisplayId(uint32_t displayId) { pendingMountDisplayId_ = displayId; }
    uint32_t getMountInstanceId() const { return mountInstanceId_; }
    uint32_t getMountModelId() const { return mountModelId_; }

    struct RemotePlayerMount {
        uint32_t displayId = 0;
        uint32_t modelId = 0;
        uint32_t instanceId = 0;
        float riderHeight = 0.0f;
    };
    void setRemotePlayerMountDisplayId(uint64_t guid, uint32_t displayId);
    const RemotePlayerMount* getRemotePlayerMount(uint64_t guid) const {
        auto it = remotePlayerMounts_.find(guid);
        return it != remotePlayerMounts_.end() ? &it->second : nullptr;
    }

    // Local player GUID exclusion (EntitySpawner skips spawning the local player)
    void setLocalPlayerGuid(uint64_t guid) { spawnedPlayerGuid_ = guid; }

    // Weapon model IDs (shared counter for creature weapons + equipment helm/weapon models)
    uint32_t allocateWeaponModelId() { return nextWeaponModelId_++; }

    // Maximum weapon attachment operations per tick (used by Application update loop)
    static constexpr int MAX_WEAPON_ATTACHES_PER_TICK = 2;

    // Dead creature tracking (spawn in corpse/death pose)
    void markCreatureDead(uint64_t guid) { deadCreatureGuids_.insert(guid); }
    void unmarkCreatureDead(uint64_t guid) { deadCreatureGuids_.erase(guid); }
    void clearDeadCreatureGuids() { deadCreatureGuids_.clear(); }

    // Mount state management
    void clearMountState();

    // Transport registration API (used by setupUICallbacks transport spawn callback)
    void queueTransportRegistration(uint64_t guid, uint32_t entry, uint32_t displayId,
                                     float x, float y, float z, float orientation);
    void setTransportPendingMove(uint64_t guid, float x, float y, float z, float orientation);
    bool hasTransportRegistrationPending(uint64_t guid) const;
    void updateTransportRegistration(uint64_t guid, uint32_t displayId,
                                      float x, float y, float z, float orientation);

    // Test transport setup flag
    bool isTestTransportSetup() const { return testTransportSetup_; }
    void setTestTransportSetup(bool v) { testTransportSetup_ = v; }

    // Creature animation state accessors (for movement sync in Application::update)
    std::unordered_map<uint64_t, uint32_t>& getCreatureInstances() { return creatureInstances_; }
    const std::unordered_map<uint64_t, uint32_t>& getCreatureInstances() const { return creatureInstances_; }
    std::unordered_map<uint64_t, uint32_t>& getCreatureModelIds() { return creatureModelIds_; }
    const std::unordered_map<uint64_t, uint32_t>& getCreatureDisplayIds() const { return creatureDisplayIds_; }
    std::unordered_map<uint64_t, glm::vec3>& getCreatureRenderPosCache() { return creatureRenderPosCache_; }
    std::unordered_map<uint64_t, bool>& getCreatureWasMoving() { return creatureWasMoving_; }
    std::unordered_map<uint64_t, bool>& getCreatureWasSwimming() { return creatureWasSwimming_; }
    std::unordered_map<uint64_t, bool>& getCreatureWasFlying() { return creatureWasFlying_; }
    std::unordered_map<uint64_t, bool>& getCreatureWasWalking() { return creatureWasWalking_; }
    std::unordered_map<uint64_t, bool>& getCreatureSwimmingState() { return creatureSwimmingState_; }
    std::unordered_map<uint64_t, bool>& getCreatureWalkingState() { return creatureWalkingState_; }
    std::unordered_map<uint64_t, bool>& getCreatureFlyingState() { return creatureFlyingState_; }
    std::unordered_map<uint64_t, uint32_t>& getCreatureActiveEmotes() { return creatureActiveEmotes_; }
    std::unordered_map<uint32_t, bool>& getModelIdIsWolfLike() { return modelIdIsWolfLike_; }

    // Player instance accessors (for movement sync in Application::update)
    std::unordered_map<uint64_t, uint32_t>& getPlayerInstances() { return playerInstances_; }
    const std::unordered_map<uint64_t, uint32_t>& getPlayerInstances() const { return playerInstances_; }

    // GameObject instance accessors
    auto& getGameObjectInstances() { return gameObjectInstances_; }
    const auto& getGameObjectInstances() const { return gameObjectInstances_; }

    // Display data accessors (needed by Application for gryphon/wyvern display IDs)
    const auto& getDisplayDataMap() const { return displayDataMap_; }
    uint32_t getGryphonDisplayId() const { return gryphonDisplayId_; }
    uint32_t getWyvernDisplayId() const { return wyvernDisplayId_; }

    // Character section lookups (needed for player character skin compositing)
    std::string lookupCharSection(uint8_t race, uint8_t sex, uint8_t section,
                                  uint8_t variation, uint8_t color, int texIndex = 0) const;
    const std::unordered_map<uint32_t, uint16_t>& getHairGeosetMap() const { return hairGeosetMap_; }

    struct FacialHairGeosets { uint16_t geoset100 = 0; uint16_t geoset300 = 0; uint16_t geoset200 = 0; };
    const std::unordered_map<uint32_t, FacialHairGeosets>& getFacialHairGeosetMap() const { return facialHairGeosetMap_; }

    // Creature M2 sync loader (used by spawnPlayerCharacter in Application)

private:
    // Dependencies (non-owning)
    rendering::Renderer* renderer_;
    pipeline::AssetManager* assetManager_;
    game::GameHandler* gameHandler_;
    pipeline::DBCLayout* dbcLayout_;
    game::GameServices* gameServices_;

    // --- Creature display data (from DBC files) ---
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
    std::unordered_map<uint32_t, FacialHairGeosets> facialHairGeosetMap_;
    bool creatureLookupsBuilt_ = false;
    bool tryAttachCreatureVirtualWeapons(uint64_t guid, uint32_t instanceId);

    // CharSections.dbc lookup cache
    std::unordered_map<uint64_t, std::string> charSectionsCache_;
    bool charSectionsCacheBuilt_ = false;
    void buildCharSectionsCache();

    // Creature display lookup building
    void buildCreatureDisplayLookups();

    // Weapon M2 loading helper
    bool loadWeaponM2(const std::string& m2Path, pipeline::M2Model& outModel);

    // GameObject display lookups
    std::unordered_map<uint32_t, std::string> gameObjectDisplayIdToPath_;
    bool gameObjectLookupsBuilt_ = false;
    void buildGameObjectDisplayLookups();

    // --- Creature instances and tracking ---
    // Drop the pending-spawn marker for guid unless a queue entry still references it.
    void erasePendingGuidIfUnqueued(uint64_t guid);

    std::unordered_map<uint64_t, uint32_t> creatureInstances_;  // guid → render instanceId
    std::unordered_map<uint64_t, uint32_t> creatureModelIds_;   // guid → loaded modelId
    std::unordered_map<uint64_t, uint32_t> creatureDisplayIds_; // guid → active displayId
    // Latest server-requested display, including creatures whose async model load
    // has not completed yet. This prevents an older queued display from winning.
    std::unordered_map<uint64_t, uint32_t> requestedCreatureDisplayIds_;
    std::unordered_map<uint64_t, glm::vec3> creatureRenderPosCache_; // guid → last synced render position
    std::unordered_map<uint64_t, bool> creatureWasMoving_;
    std::unordered_map<uint64_t, bool> creatureWasSwimming_;
    std::unordered_map<uint64_t, bool> creatureWasFlying_;
    std::unordered_map<uint64_t, bool> creatureWasWalking_;
    std::unordered_map<uint64_t, bool> creatureSwimmingState_;
    std::unordered_map<uint64_t, bool> creatureWalkingState_;
    std::unordered_map<uint64_t, bool> creatureFlyingState_;
    // Server waypoint scripts hold work/chop emotes for many seconds. Locomotion
    // temporarily overrides them, then the idle transition restores this loop.
    std::unordered_map<uint64_t, uint32_t> creatureActiveEmotes_;
    std::unordered_map<uint64_t, bool> creatureWasStealthed_;
    uint32_t stealthSyncFrameCounter_ = 0;  // throttles syncCreatureStealthVisuals()
    std::unordered_set<uint64_t> creatureWeaponsAttached_;
    std::unordered_map<uint64_t, uint8_t> creatureWeaponAttachAttempts_;
    std::unordered_map<uint32_t, bool> modelIdIsWolfLike_;

    void syncCreatureStealthVisuals();

    // --- Creature async loads ---
    struct PreparedCreatureModel {
        uint64_t guid;
        uint32_t displayId;
        uint32_t modelId;
        float x, y, z, orientation;
        float scale = 1.0f;
        std::shared_ptr<pipeline::M2Model> model;
        std::unordered_map<std::string, pipeline::BLPImage> predecodedTextures;
        bool valid = false;
        bool permanent_failure = false;
    };
    struct AsyncCreatureLoad {
        std::future<PreparedCreatureModel> future;
    };
    std::vector<AsyncCreatureLoad> asyncCreatureLoads_;
    std::unordered_set<uint32_t> asyncCreatureDisplayLoads_;
    void processAsyncCreatureResults(bool unlimited = false);
    static constexpr int MAX_ASYNC_CREATURE_LOADS = 4;
    std::unordered_set<uint64_t> deadCreatureGuids_;

    // --- NPC equipment attachment models (helm, shoulders) ---
    // Equipment is shared across NPCs, so a crowd wearing the same gear would
    // otherwise re-read, re-parse and re-upload the same M2 once per spawn. Cache
    // it by resolved path. A modelId of 0 negative-caches a path that has no model,
    // so a missing file is not retried against the disk on every spawn.
    struct CachedAttachmentModel {
        uint32_t modelId = 0;                      // 0 = no usable model
        std::shared_ptr<pipeline::M2Model> model;  // shared_ptr: M2Model is incomplete here
    };
    // Parsed geometry, keyed by path. A null value negative-caches a missing file so
    // it is not probed against the disk on every spawn.
    std::unordered_map<std::string, std::shared_ptr<pipeline::M2Model>> attachmentModelData_;
    // Renderer model id, keyed by "path|texture". attachWeapon() binds the texture to
    // the model id, so two recolours of one mesh must not share an id or the last
    // NPC spawned would retexture every other NPC wearing that mesh.
    std::unordered_map<std::string, uint32_t> attachmentModelIds_;
    /// Resolve the first candidate path with a usable model, reading and parsing it
    /// at most once. Returns modelId == 0 when no candidate yields a valid model.
    CachedAttachmentModel getOrLoadAttachmentModel(
        const std::vector<std::string>& candidatePaths, const std::string& texturePath);

    std::unordered_map<uint32_t, uint32_t> displayIdModelCache_;
    std::unordered_set<uint32_t> displayIdTexturesApplied_;
    std::unordered_map<uint32_t, std::unordered_map<std::string, pipeline::BLPImage>> displayIdPredecodedTextures_;
    mutable std::unordered_set<uint32_t> warnedMissingDisplayDataIds_;
    mutable std::unordered_set<uint32_t> warnedMissingModelPathIds_;
    uint32_t nextCreatureModelId_ = 5000;
    uint32_t gryphonDisplayId_ = 0;
    uint32_t wyvernDisplayId_ = 0;

    // --- Creature spawn queue ---
    struct PendingCreatureSpawn {
        uint64_t guid;
        uint32_t displayId;
        float x, y, z, orientation;
        float scale = 1.0f;
    };
    std::deque<PendingCreatureSpawn> pendingCreatureSpawns_;
    static constexpr int MAX_SPAWNS_PER_FRAME = 3;
    static constexpr int MAX_NEW_CREATURE_MODELS_PER_FRAME = 1;
    static constexpr auto CREATURE_SPAWN_RETRY_WINDOW = std::chrono::seconds(5);
    std::unordered_set<uint64_t> pendingCreatureSpawnGuids_;
    std::unordered_map<uint64_t, std::chrono::steady_clock::time_point>
        creatureSpawnRetryDeadlines_;
    std::unordered_set<uint32_t> nonRenderableCreatureDisplayIds_;
    std::unordered_set<uint64_t> creaturePermanentFailureGuids_;
    void processCreatureSpawnQueue(bool unlimited = false);

    // --- NPC composite loads ---
    struct DeferredNpcComposite {
        uint32_t modelId;
        uint32_t displayId;
        std::string basePath;
        std::vector<std::string> overlayPaths;
        std::vector<std::pair<int, std::string>> regionLayers;
        std::vector<uint32_t> skinTextureSlots;
        bool hasComposite = false;
        bool hasSimpleSkin = false;
        std::string bakedSkinPath;
        bool hasBakedSkin = false;
        std::vector<uint32_t> hairTextureSlots;
        std::string hairTexturePath;
        bool useBakedForHair = false;
    };
    struct PreparedNpcComposite {
        DeferredNpcComposite info;
        std::unordered_map<std::string, pipeline::BLPImage> predecodedTextures;
    };
    struct AsyncNpcCompositeLoad {
        std::future<PreparedNpcComposite> future;
    };
    std::vector<AsyncNpcCompositeLoad> asyncNpcCompositeLoads_;
    void processAsyncNpcCompositeResults(bool unlimited = false);

    // --- Player instances ---
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
    std::deque<std::pair<uint64_t, std::pair<std::array<uint32_t, 19>, std::array<uint8_t, 19>>>> deferredEquipmentQueue_;
    void processDeferredEquipmentQueue();
    struct PreparedEquipmentUpdate {
        uint64_t guid;
        std::array<uint32_t, 19> displayInfoIds;
        std::array<uint8_t, 19> inventoryTypes;
        std::unordered_map<std::string, pipeline::BLPImage> predecodedTextures;
    };
    struct AsyncEquipmentLoad {
        std::future<PreparedEquipmentUpdate> future;
    };
    std::vector<AsyncEquipmentLoad> asyncEquipmentLoads_;
    void processAsyncEquipmentResults();
    std::vector<std::string> resolveEquipmentTexturePaths(uint64_t guid,
        const std::array<uint32_t, 19>& displayInfoIds,
        const std::array<uint8_t, 19>& inventoryTypes) const;
    std::unordered_map<uint32_t, uint32_t> playerModelCache_;
    struct PlayerTextureSlots { int skin = -1; int hair = -1; int underwear = -1; };
    std::unordered_map<uint32_t, PlayerTextureSlots> playerTextureSlotsByModelId_;
    uint32_t nextPlayerModelId_ = 60000;

    // --- Player spawn queue ---
    struct PendingPlayerSpawn {
        uint64_t guid;
        uint8_t raceId;
        uint8_t genderId;
        uint32_t appearanceBytes;
        uint8_t facialFeatures;
        float x, y, z, orientation;
    };
    std::deque<PendingPlayerSpawn> pendingPlayerSpawns_;
    std::unordered_set<uint64_t> pendingPlayerSpawnGuids_;
    void processPlayerSpawnQueue();

    // --- GameObject instances ---
    struct GameObjectInstanceInfo {
        uint32_t modelId = 0;
        uint32_t instanceId = 0;
        bool isWmo = false;
    };
    // Game objects spawned before their type was known, keyed by entry. Each is
    // frozen on the conservative assumption that it is state-driven until the
    // query response says otherwise.
    std::unordered_map<uint32_t, std::vector<uint32_t>> gameObjectPendingAnimPolicy_;

    std::unordered_map<uint32_t, uint32_t> gameObjectDisplayIdModelCache_;
    std::unordered_set<uint32_t> gameObjectDisplayIdFailedCache_;
    std::unordered_map<uint32_t, uint32_t> gameObjectDisplayIdWmoCache_;
    std::unordered_map<uint64_t, GameObjectInstanceInfo> gameObjectInstances_;
    struct PendingTransportMove {
        float x = 0.0f, y = 0.0f, z = 0.0f, orientation = 0.0f;
    };
    struct PendingTransportRegistration {
        uint64_t guid = 0;
        uint32_t entry = 0;
        uint32_t displayId = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f, orientation = 0.0f;
        int retryFrames = 0;  // waiting for the GO's render instance to spawn
    };
    std::unordered_map<uint64_t, PendingTransportMove> pendingTransportMoves_;
    std::deque<PendingTransportRegistration> pendingTransportRegistrations_;
    uint32_t nextGameObjectModelId_ = 20000;
    uint32_t nextGameObjectWmoModelId_ = 40000;
    bool testTransportSetup_ = false;

    // --- GameObject spawn queue ---
    struct PendingGameObjectSpawn {
        uint64_t guid;
        uint32_t entry;
        uint32_t displayId;
        float x, y, z, orientation;
        float scale = 1.0f;
    };
    std::deque<PendingGameObjectSpawn> pendingGameObjectSpawns_;
    void processGameObjectSpawnQueue();

    // --- Async WMO loading for game objects ---
    struct PreparedGameObjectWMO {
        uint64_t guid;
        uint32_t entry;
        uint32_t displayId;
        float x, y, z, orientation;
        float scale = 1.0f;
        std::shared_ptr<pipeline::WMOModel> wmoModel;
        std::unordered_map<std::string, pipeline::BLPImage> predecodedTextures;
        bool valid = false;
        bool isWmo = false;
        std::string modelPath;
    };
    struct AsyncGameObjectLoad {
        std::future<PreparedGameObjectWMO> future;
    };

    // A parsed WMO whose GPU upload is being spread across frames. Uploading a
    // transport in one go cost ~40ms — its textures are large, and that upload
    // has to happen on the main thread even though a worker decoded them. The
    // result is held here because it owns the model and the decoded pixels.
    struct PendingWmoUpload {
        PreparedGameObjectWMO result;
        uint32_t modelId = 0;
    };
    std::vector<PendingWmoUpload> pendingWmoUploads_;
    void processPendingWmoUploads();
    void finishWmoSpawn(const PreparedGameObjectWMO& result, uint32_t modelId);
    std::vector<AsyncGameObjectLoad> asyncGameObjectLoads_;
    void processAsyncGameObjectResults();
    struct PendingTransportDoodadBatch {
        uint64_t guid = 0;
        uint32_t modelId = 0;
        uint32_t instanceId = 0;
        size_t nextIndex = 0;
        size_t doodadBudget = 0;
        size_t spawnedDoodads = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f, orientation = 0.0f;
    };
    std::vector<PendingTransportDoodadBatch> pendingTransportDoodadBatches_;
    static constexpr size_t MAX_TRANSPORT_DOODADS_PER_FRAME = 4;
    void processPendingTransportRegistrations();
    void processPendingTransportDoodads();

    // --- Mount ---
    uint32_t mountInstanceId_ = 0;
    uint32_t mountModelId_ = 0;
    uint32_t pendingMountDisplayId_ = 0;
    void processPendingMount();
    std::unordered_map<uint64_t, RemotePlayerMount> remotePlayerMounts_;
    std::unordered_map<uint64_t, uint32_t> pendingRemotePlayerMounts_;
    void processPendingRemotePlayerMounts();
    bool loadRemoteMountModel(uint32_t displayId, uint32_t& modelId,
                              std::string& modelPath, float& riderHeight);
    void removeRemotePlayerMount(uint64_t guid);

    // --- Local player GUID exclusion ---
    uint64_t spawnedPlayerGuid_ = 0;

    // --- Weapon model ID counter ---
    // Weapon model ids share CharacterRenderer's models map with NPC
    // composites, which are keyed by creature displayId (1..~35000). Starting
    // at 1000 collided with loaded NPC models once enough weapon reloads
    // advanced the counter — attachWeapon then instanced an NPC mesh as the
    // weapon. Reserve a range no displayId can reach.
    uint32_t nextWeaponModelId_ = 0x40000000u;

    // --- Spawn internal methods ---
    void spawnOnlineCreature(uint64_t guid, uint32_t displayId,
                             float x, float y, float z, float orientation, float scale = 1.0f);
    void spawnOnlinePlayer(uint64_t guid, uint8_t raceId, uint8_t genderId,
                           uint32_t appearanceBytes, uint8_t facialFeatures,
                           float x, float y, float z, float orientation);
    void setOnlinePlayerEquipment(uint64_t guid,
                                  const std::array<uint32_t, 19>& displayInfoIds,
                                  const std::array<uint8_t, 19>& inventoryTypes);
    void spawnOnlineGameObject(uint64_t guid, uint32_t entry, uint32_t displayId,
                               float x, float y, float z, float orientation, float scale = 1.0f);
};

} // namespace core
} // namespace wowee
