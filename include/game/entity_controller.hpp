#pragma once

#include "game/world_packets.hpp"
#include "game/entity.hpp"
#include "game/opcode_table.hpp"
#include "network/packet.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <glm/vec3.hpp>

namespace wowee {
namespace game {

class GameHandler;

class EntityController {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit EntityController(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Entity Manager access ---
    EntityManager& getEntityManager() { return entityManager; }
    const EntityManager& getEntityManager() const { return entityManager; }

    // --- Name / info cache queries ---
    void queryPlayerName(uint64_t guid);
    void queryCreatureInfo(uint32_t entry, uint64_t guid);
    void queryGameObjectInfo(uint32_t entry, uint64_t guid);
    std::string getCachedPlayerName(uint64_t guid) const;
    std::string getCachedCreatureName(uint32_t entry) const;
    void invalidatePlayerName(uint64_t guid) { playerNameCache.erase(guid); }

    // Read-only cache access for other handlers
    const std::unordered_map<uint64_t, std::string>& getPlayerNameCache() const { return playerNameCache; }
    const std::unordered_map<uint32_t, CreatureQueryResponseData>& getCreatureInfoCache() const { return creatureInfoCache; }
    std::string getCachedCreatureSubName(uint32_t entry) const {
        auto it = creatureInfoCache.find(entry);
        return (it != creatureInfoCache.end()) ? it->second.subName : "";
    }
    int getCreatureRank(uint32_t entry) const {
        auto it = creatureInfoCache.find(entry);
        return (it != creatureInfoCache.end()) ? static_cast<int>(it->second.rank) : -1;
    }
    uint32_t getCreatureType(uint32_t entry) const {
        auto it = creatureInfoCache.find(entry);
        return (it != creatureInfoCache.end()) ? it->second.creatureType : 0;
    }
    uint32_t getCreatureFamily(uint32_t entry) const {
        auto it = creatureInfoCache.find(entry);
        return (it != creatureInfoCache.end()) ? it->second.family : 0;
    }
    const GameObjectQueryResponseData* getCachedGameObjectInfo(uint32_t entry) const {
        auto it = gameObjectInfoCache_.find(entry);
        return (it != gameObjectInfoCache_.end()) ? &it->second : nullptr;
    }

    // Name lookup (checks cache then entity manager)
    const std::string& lookupName(uint64_t guid) const {
        static const std::string kEmpty;
        auto it = playerNameCache.find(guid);
        if (it != playerNameCache.end()) return it->second;
        auto entity = entityManager.getEntity(guid);
        if (entity && entity->isUnit()) {
            // isUnit() is the type-tag check; skip dynamic_cast since the tag
            // already guarantees the Unit subclass.
            const auto* unit = static_cast<const Unit*>(entity.get());
            if (!unit->getName().empty()) return unit->getName();
        }
        return kEmpty;
    }
    uint8_t lookupPlayerClass(uint64_t guid) const {
        auto it = playerClassRaceCache_.find(guid);
        return it != playerClassRaceCache_.end() ? it->second.classId : 0;
    }
    uint8_t lookupPlayerRace(uint64_t guid) const {
        auto it = playerClassRaceCache_.find(guid);
        return it != playerClassRaceCache_.end() ? it->second.raceId : 0;
    }

    // --- Transport GUID tracking ---
    bool isTransportGuid(uint64_t guid) const { return transportGuids_.count(guid) > 0; }
    bool hasServerTransportUpdate(uint64_t guid) const { return serverUpdatedTransportGuids_.count(guid) > 0; }

    // --- Update object work queue ---
    void enqueueUpdateObjectWork(UpdateObjectData&& data);
    void processPendingUpdateObjectWork(const std::chrono::steady_clock::time_point& start,
                                        float budgetMs);
    bool hasPendingUpdateObjectWork() const { return !pendingUpdateObjectWork_.empty(); }

    // --- Reset all state (called on disconnect / character switch) ---
    void clearAll();

private:
    GameHandler& owner_;

    // --- Entity tracking ---
    EntityManager entityManager;              // Manages all entities in view

    // ---- Name caches ----
    std::unordered_map<uint64_t, std::string> playerNameCache;
    // Class/race cache from SMSG_NAME_QUERY_RESPONSE (guid → {classId, raceId})
    struct PlayerClassRace { uint8_t classId = 0; uint8_t raceId = 0; };
    std::unordered_map<uint64_t, PlayerClassRace> playerClassRaceCache_;
    std::unordered_set<uint64_t> pendingNameQueries;
    std::unordered_map<uint32_t, CreatureQueryResponseData> creatureInfoCache;
    std::unordered_set<uint32_t> pendingCreatureQueries;
    std::unordered_map<uint32_t, GameObjectQueryResponseData> gameObjectInfoCache_;
    std::unordered_set<uint32_t> pendingGameObjectQueries_;

    // --- Update Object work queue ---
    struct PendingUpdateObjectWork {
        UpdateObjectData data;
        size_t nextBlockIndex = 0;
        bool outOfRangeProcessed = false;
        bool newItemCreated = false;
    };
    std::deque<PendingUpdateObjectWork> pendingUpdateObjectWork_;

    // --- Transport GUID tracking ---
    std::unordered_set<uint64_t> transportGuids_;  // GUIDs of known transport GameObjects
    std::unordered_set<uint64_t> serverUpdatedTransportGuids_;

    // --- Packet handlers ---
    void handleUpdateObject(network::Packet& packet);
    void handleCompressedUpdateObject(network::Packet& packet);
    void handleDestroyObject(network::Packet& packet);
    void handleNameQueryResponse(network::Packet& packet);
    void handleCreatureQueryResponse(network::Packet& packet);
    void handleGameObjectQueryResponse(network::Packet& packet);
    void handleGameObjectPageText(network::Packet& packet);
    void handlePageTextQueryResponse(network::Packet& packet);

    // --- Entity lifecycle ---
    void processOutOfRangeObjects(const std::vector<uint64_t>& guids);
    void applyUpdateObjectBlock(const UpdateBlock& block, bool& newItemCreated);
    void finalizeUpdateObjectBatch(bool newItemCreated);

    bool extractPlayerAppearance(const FlatFieldMap& fields,
                                 uint8_t& outRace, uint8_t& outGender,
                                 uint32_t& outAppearanceBytes, uint8_t& outFacial) const;
    void maybeDetectCoinageIndex(const FlatFieldMap& oldFields,
                                 const FlatFieldMap& newFields);

    void handleCreateObject(const UpdateBlock& block, bool& newItemCreated);
    void handleValuesUpdate(const UpdateBlock& block);
    void handleMovementUpdate(const UpdateBlock& block);
    /// Seed a transport's route phase from the server's own path clock, so the
    /// client stops animating on a period it invented for itself.
    void applyTransportRouteClock(const UpdateBlock& block);
    /// Note which non-combat companion is out and what called it, so casting
    /// that spell again dismisses instead of summoning another.
    void trackActiveCritter(const UpdateBlock& block);

    // Update transport-relative child attachment (non-player entities).
    //     Consolidates identical logic from CREATE/VALUES/MOVEMENT handlers.
    void updateNonPlayerTransportAttachment(const UpdateBlock& block,
                                            const std::shared_ptr<Entity>& entity,
                                            ObjectType entityType);
    // Rebuild playerAuras_ from UNIT_FIELD_AURAS (Classic/TBC-era clients).
    //     Consolidates identical logic from CREATE and VALUES handlers.
    void syncPreWotlkAurasFromFields(const std::shared_ptr<Entity>& entity);
    // Detect mount/dismount from UNIT_FIELD_MOUNTDISPLAYID changes (self-player only).
    //     Consolidates identical logic from CREATE and VALUES handlers.
    void detectPlayerMountChange(uint32_t newMountDisplayId,
                                 const FlatFieldMap& blockFields);

    // Shared player-death handler: caches corpse position, sets death state.
    void markPlayerDead(const char* source);

    // Cached field indices resolved once per handler call to avoid repeated lookups.
    struct UnitFieldIndices {
        uint16_t health, maxHealth, powerBase, maxPowerBase;
        uint16_t level, faction, flags, dynFlags, auraState;
        uint16_t displayId, mountDisplayId, npcFlags, npcEmoteState;
        uint16_t bytes0, bytes1;
        static UnitFieldIndices resolve();
    };
    struct PlayerFieldIndices {
        uint16_t xp, nextXp, restedXp, level;
        uint16_t coinage, honor, arena;
        uint16_t playerFlags, armor;
        uint16_t pBytes, pBytes2, chosenTitle;
        uint16_t stats[5];
        uint16_t meleeAP, rangedAP;
        uint16_t spDmg1, healBonus;
        uint16_t blockPct, dodgePct, parryPct, critPct, rangedCritPct;
        uint16_t sCrit1, rating1;
        static PlayerFieldIndices resolve();
    };
    struct UnitFieldUpdateResult {
        bool healthChanged = false;
        bool powerChanged = false;
        bool displayIdChanged = false;
        bool npcDeathNotified = false;
        bool npcRespawnNotified = false;
        // Set when UNIT_DYNFLAG_LOOTABLE went away this update — the corpse has
        // been looted empty. Acted on after the field loop, never inside it.
        bool lootableCleared = false;
        uint32_t oldDisplayId = 0;
    };

    // Entity factory — creates the correct Entity subclass for the given block.
    std::shared_ptr<Entity> createEntityFromBlock(const UpdateBlock& block);
    // Track player-on-transport state from movement blocks.
    /// Take the server's position for the player when the two have diverged
    /// beyond anything lag can explain. Does nothing in the ordinary case,
    /// where the client owns its position and the server echoes it back.
    void resyncPlayerIfFarFromServer(const glm::vec3& serverCanonicalPos);

    void applyPlayerTransportState(const UpdateBlock& block,
                                    const std::shared_ptr<Entity>& entity,
                                    const glm::vec3& canonicalPos, float oCanonical,
                                    bool updateMovementInfoPos);
    // Apply unit fields during CREATE — returns true if entity is initially dead.
    bool applyUnitFieldsOnCreate(const UpdateBlock& block,
                                  std::shared_ptr<Unit>& unit,
                                  const UnitFieldIndices& ufi);
    // Apply unit fields during VALUES — returns change tracking result.
    UnitFieldUpdateResult applyUnitFieldsOnUpdate(const UpdateBlock& block,
                                                    const std::shared_ptr<Entity>& entity,
                                                    std::shared_ptr<Unit>& unit,
                                                    const UnitFieldIndices& ufi);
    // Apply player stat fields (XP, inventory, skills, etc.). isCreate=true for CREATE path.
    bool applyPlayerStatFields(const FlatFieldMap& fields,
                                const PlayerFieldIndices& pfi, bool isCreate);
    // Dispatch spawn callbacks (creature/player) — deduplicates CREATE and VALUES paths.
    void dispatchEntitySpawn(uint64_t guid, ObjectType objectType,
                              const std::shared_ptr<Entity>& entity,
                              const std::shared_ptr<Unit>& unit, bool isDead);
    // Track item/container on CREATE.
    void trackItemOnCreate(const UpdateBlock& block, bool& newItemCreated);
    // Update item fields on VALUES update.
    void updateItemOnValuesUpdate(const UpdateBlock& block,
                                   const std::shared_ptr<Entity>& entity);

    // Allows extending object-type handling without modifying handler dispatch.
    struct IObjectTypeHandler {
        virtual ~IObjectTypeHandler() = default;
        virtual void onCreate(const UpdateBlock& /*block*/, std::shared_ptr<Entity>& /*entity*/,
                              bool& /*newItemCreated*/) {}
        virtual void onValuesUpdate(const UpdateBlock& /*block*/, std::shared_ptr<Entity>& /*entity*/) {}
        virtual void onMovementUpdate(const UpdateBlock& /*block*/, std::shared_ptr<Entity>& /*entity*/) {}
    };
    struct UnitTypeHandler;
    struct PlayerTypeHandler;
    struct GameObjectTypeHandler;
    struct ItemTypeHandler;
    struct CorpseTypeHandler;
    std::unordered_map<uint8_t, std::unique_ptr<IObjectTypeHandler>> typeHandlers_;
    void initTypeHandlers();
    IObjectTypeHandler* getTypeHandler(ObjectType type) const;

    void onCreateUnit(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onCreatePlayer(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onCreateGameObject(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onCreateItem(const UpdateBlock& block, bool& newItemCreated);
    void onCreateCorpse(const UpdateBlock& block);
    void handleDisplayIdChange(const UpdateBlock& block,
                               const std::shared_ptr<Entity>& entity,
                               const std::shared_ptr<Unit>& unit,
                               const UnitFieldUpdateResult& result);
    void onValuesUpdateUnit(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onValuesUpdatePlayer(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onValuesUpdateItem(const UpdateBlock& block, std::shared_ptr<Entity>& entity);
    void onValuesUpdateGameObject(const UpdateBlock& block, std::shared_ptr<Entity>& entity);

    // Collects addon events during block processing, flushes at the end.
    struct PendingEvents {
        std::vector<std::pair<std::string, std::vector<std::string>>> events;
        void emit(const std::string& name, const std::vector<std::string>& args = {}) {
            events.emplace_back(name, args);
        }
        void clear() { events.clear(); }
    };
    PendingEvents pendingEvents_;
    void flushPendingEvents();
};

} // namespace game
} // namespace wowee
