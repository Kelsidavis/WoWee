#pragma once

#include "game/protocol_constants.hpp"
#include "game/update_field_table.hpp"

#include <cstdint>
#include <cmath>
#include <string>
#include <array>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <optional>
#include <mutex>
#include <chrono>
#include "math/spline.hpp"
#include "game/flat_field_map.hpp"

namespace wowee {
namespace game {

/**
 * Object type IDs for WoW 3.3.5a
 */
enum class ObjectType : uint8_t {
    OBJECT = 0,
    ITEM = 1,
    CONTAINER = 2,
    UNIT = 3,
    PLAYER = 4,
    GAMEOBJECT = 5,
    DYNAMICOBJECT = 6,
    CORPSE = 7
};

/**
 * Object type masks for update packets
 */
enum class TypeMask : uint16_t {
    OBJECT = 0x0001,
    ITEM = 0x0002,
    CONTAINER = 0x0004,
    UNIT = 0x0008,
    PLAYER = 0x0010,
    GAMEOBJECT = 0x0020,
    DYNAMICOBJECT = 0x0040,
    CORPSE = 0x0080
};

// UNIT_DYNAMIC_FLAGS values shared by the supported legacy expansions.
// Keep these named: 0x08 is TAPPED_BY_PLAYER, while the actual DEAD bit is
// 0x20. Confusing the two leaves pre-existing corpses standing after login.
inline constexpr uint32_t UNIT_DYNFLAG_LOOTABLE         = 0x00000001;
inline constexpr uint32_t UNIT_DYNFLAG_TAPPED           = 0x00000004;
inline constexpr uint32_t UNIT_DYNFLAG_TAPPED_BY_PLAYER = 0x00000008;
inline constexpr uint32_t UNIT_DYNFLAG_DEAD             = 0x00000020;

/// CREATE updates may omit zero-valued health. In that case the dynamic corpse
/// flags are the authoritative indication that the unit must spawn dead.
inline bool isUnitCorpseState(uint32_t health, uint32_t maxHealth, uint32_t dynamicFlags) {
    return (maxHealth > 0 && health == 0) ||
           (dynamicFlags & (UNIT_DYNFLAG_DEAD | UNIT_DYNFLAG_LOOTABLE)) != 0;
}

/**
 * Update types for SMSG_UPDATE_OBJECT
 */
enum class UpdateType : uint8_t {
    VALUES = 0,              // Partial update (changed fields only)
    MOVEMENT = 1,            // Movement update
    CREATE_OBJECT = 2,       // Create new object (full data)
    CREATE_OBJECT2 = 3,      // Create new object (alternate format)
    OUT_OF_RANGE_OBJECTS = 4, // Objects left view range
    NEAR_OBJECTS = 5         // Objects entered view range
};

/// The update type's own name, for a log line.
///
/// Beside the enum rather than in the two parsers that print it - the WotLK one
/// and the vanilla one had an identical copy each, and they are the two files
/// where a value added to this list would need noticing.
inline const char* updateTypeName(UpdateType type) {
    switch (type) {
        case UpdateType::VALUES:               return "VALUES";
        case UpdateType::MOVEMENT:             return "MOVEMENT";
        case UpdateType::CREATE_OBJECT:        return "CREATE_OBJECT";
        case UpdateType::CREATE_OBJECT2:       return "CREATE_OBJECT2";
        case UpdateType::OUT_OF_RANGE_OBJECTS: return "OUT_OF_RANGE_OBJECTS";
        case UpdateType::NEAR_OBJECTS:         return "NEAR_OBJECTS";
    }
    return "UNKNOWN";
}

/**
 * Base entity class for all game objects
 */
class Entity {
public:
    Entity() = default;
    explicit Entity(uint64_t guid) : guid(guid) {}
    virtual ~Entity() = default;

    // GUID access
    [[nodiscard]] uint64_t getGuid() const { return guid; }
    void setGuid(uint64_t g) { guid = g; }

    // Position
    [[nodiscard]] float getX() const { return x; }
    [[nodiscard]] float getY() const { return y; }
    [[nodiscard]] float getZ() const { return z; }
    [[nodiscard]] float getOrientation() const { return orientation; }
    // Update orientation only, without disrupting an in-progress movement interpolation.
    void setOrientation(float o) { orientation = o; }

    void setPosition(float px, float py, float pz, float o) {
        x = px;
        y = py;
        z = pz;
        orientation = o;
        isMoving_ = false; // Instant position set cancels interpolation
        usePathMode_ = false;
        activeSpline_.reset();
    }

    // Multi-segment path movement (Catmull-Rom spline interpolation)
    void startMoveAlongPath(const std::vector<std::array<float, 3>>& path, float destO, float totalDuration) {
        if (path.empty()) return;
        if (path.size() == 1 || totalDuration <= 0.0f) {
            startMoveTo(path.back()[0], path.back()[1], path.back()[2], destO, totalDuration);
            return;
        }
        // Build cumulative distances for proportional time assignment.
        // (Stored in a tiny stack/heap vector - typical N is <=15 waypoints,
        // and keeping float precision matters for the timeMs rescale below.)
        std::vector<float> cumDist(path.size(), 0.0f);
        float totalDist = 0.0f;
        for (size_t i = 1; i < path.size(); i++) {
            float dx = path[i][0] - path[i - 1][0];
            float dy = path[i][1] - path[i - 1][1];
            float dz = path[i][2] - path[i - 1][2];
            totalDist += std::sqrt(dx * dx + dy * dy + dz * dz);
            cumDist[i] = totalDist;
        }
        if (totalDist < 0.001f) {
            startMoveTo(path.back()[0], path.back()[1], path.back()[2], destO, totalDuration);
            return;
        }
        // Build SplineKeys with distance-proportional time
        uint32_t durationMs = static_cast<uint32_t>(totalDuration * 1000.0f);
        const float invTotalDist = static_cast<float>(durationMs) / totalDist;
        std::vector<math::SplineKey> keys(path.size());
        for (size_t i = 0; i < path.size(); i++) {
            keys[i].timeMs = static_cast<uint32_t>(cumDist[i] * invTotalDist);
            keys[i].position = {path[i][0], path[i][1], path[i][2]};
        }
        activeSpline_.emplace(std::move(keys), /*timeClosed=*/false);
        splineDurationMs_ = durationMs;

        // Snap position if in overrun phase
        if (isMoving_ && moveElapsed_ >= moveDuration_) {
            x = moveEndX_; y = moveEndY_; z = moveEndZ_;
        }
        moveEndX_ = path.back()[0]; moveEndY_ = path.back()[1]; moveEndZ_ = path.back()[2];
        moveDuration_ = totalDuration;
        moveElapsed_ = 0.0f;
        orientation = destO;
        isMoving_ = true;
        usePathMode_ = true;
        // Velocity for dead-reckoning after path completes
        float fromX = isMoving_ ? moveEndX_ : x;
        float fromY = isMoving_ ? moveEndY_ : y;
        float impliedVX = (path.back()[0] - fromX) / totalDuration;
        float impliedVY = (path.back()[1] - fromY) / totalDuration;
        float impliedVZ = (path.back()[2] - path[0][2]) / totalDuration;
        const float alpha = 0.65f;
        velX_ = alpha * impliedVX + (1.0f - alpha) * velX_;
        velY_ = alpha * impliedVY + (1.0f - alpha) * velY_;
        velZ_ = alpha * impliedVZ + (1.0f - alpha) * velZ_;
    }

    // Movement interpolation (syncs entity position with renderer during movement)
    void startMoveTo(float destX, float destY, float destZ, float destO, float durationSec) {
        usePathMode_ = false;
        activeSpline_.reset();
        if (durationSec <= 0.0f) {
            setPosition(destX, destY, destZ, destO);
            return;
        }
        // Movement heartbeats and repeated spline destinations can carry a
        // positive duration without any actual displacement. Treat these as
        // authoritative stops; otherwise isActivelyMoving() drives Run/Walk
        // while the model has nowhere to move. This applies equally to nearby
        // players and creatures.
        const float remainingX = destX - x;
        const float remainingY = destY - y;
        const float remainingZ = destZ - z;
        constexpr float kNoOpMoveDistanceSq = 0.02f * 0.02f;
        if (remainingX * remainingX + remainingY * remainingY +
                remainingZ * remainingZ <= kNoOpMoveDistanceSq) {
            setPosition(destX, destY, destZ, destO);
            return;
        }
        // If we're in the dead-reckoning overrun phase, snap x/y/z back to the
        // destination before using them as the new start.  The renderer was showing
        // the entity at moveEnd (via getLatest) during overrun, so the new
        // interpolation must start there to avoid a visible teleport.
        if (isMoving_ && moveElapsed_ >= moveDuration_) {
            x = moveEndX_;
            y = moveEndY_;
            z = moveEndZ_;
        }
        // Derive velocity from the displacement this packet implies.
        // Use the previous destination (not current lerped pos) as the "from" so
        // variable network timing doesn't inflate/shrink the implied speed.
        float fromX = isMoving_ ? moveEndX_ : x;
        float fromY = isMoving_ ? moveEndY_ : y;
        float fromZ = isMoving_ ? moveEndZ_ : z;
        float impliedVX = (destX - fromX) / durationSec;
        float impliedVY = (destY - fromY) / durationSec;
        float impliedVZ = (destZ - fromZ) / durationSec;
        // Exponential moving average on velocity - 65% new sample, 35% previous.
        // Smooths out jitter from irregular server update intervals (~200-600ms)
        // without introducing visible lag on direction changes.
        const float alpha = 0.65f;
        velX_ = alpha * impliedVX + (1.0f - alpha) * velX_;
        velY_ = alpha * impliedVY + (1.0f - alpha) * velY_;
        velZ_ = alpha * impliedVZ + (1.0f - alpha) * velZ_;

        moveStartX_ = x; moveStartY_ = y; moveStartZ_ = z;
        moveEndX_ = destX; moveEndY_ = destY; moveEndZ_ = destZ;
        moveDuration_ = durationSec;
        moveElapsed_ = 0.0f;
        orientation = destO;
        isMoving_ = true;
    }

    void updateMovement(float deltaTime) {
        if (!isMoving_) return;
        moveElapsed_ += deltaTime;
        if (moveElapsed_ < moveDuration_) {
            if (usePathMode_ && activeSpline_) {
                // Catmull-Rom spline interpolation
                uint32_t pathTimeMs = static_cast<uint32_t>(moveElapsed_ * 1000.0f);
                if (pathTimeMs >= splineDurationMs_) pathTimeMs = splineDurationMs_ - 1;
                glm::vec3 pos = activeSpline_->evaluatePosition(pathTimeMs);
                x = pos.x;
                y = pos.y;
                z = pos.z;
            } else {
                // Single-segment linear interpolation
                float t = moveElapsed_ / moveDuration_;
                x = moveStartX_ + (moveEndX_ - moveStartX_) * t;
                y = moveStartY_ + (moveEndY_ - moveStartY_) * t;
                z = moveStartZ_ + (moveEndZ_ - moveStartZ_) * t;
            }
        } else {
            // Past the interpolation window: dead-reckon at the smoothed velocity
            // rather than freezing in place. Cap to one extra interval so we don't
            // drift endlessly if the entity stops sending packets.
            float overrun = moveElapsed_ - moveDuration_;
            if (overrun < moveDuration_) {
                x = moveEndX_ + velX_ * overrun;
                y = moveEndY_ + velY_ * overrun;
                z = moveEndZ_ + velZ_ * overrun;
            } else {
                // Two intervals with no update - entity has probably stopped.
                x = moveEndX_; y = moveEndY_; z = moveEndZ_;
                velX_ = 0.0f; velY_ = 0.0f; velZ_ = 0.0f;
                isMoving_ = false;
            }
        }
    }

    [[nodiscard]] bool isEntityMoving() const { return isMoving_; }

    /// True only during the active interpolation phase (before reaching destination).
    /// Unlike isEntityMoving(), this does NOT include the dead-reckoning overrun window,
    /// so animations (Run/Walk) should use this to avoid "running in place" after arrival.
    [[nodiscard]] bool isActivelyMoving() const {
        return isMoving_ && moveElapsed_ < moveDuration_;
    }

    // Returns the latest server-authoritative position: destination if moving, current if not.
    // Unlike getX/Y/Z (which only update via updateMovement), this always reflects the
    // last known server position regardless of distance culling.
    [[nodiscard]] float getLatestX() const { return isMoving_ ? moveEndX_ : x; }
    [[nodiscard]] float getLatestY() const { return isMoving_ ? moveEndY_ : y; }
    [[nodiscard]] float getLatestZ() const { return isMoving_ ? moveEndZ_ : z; }

    // Object type
    [[nodiscard]] ObjectType getType() const { return type; }
    void setType(ObjectType t) { type = t; }

    /// True if this entity is a Unit or Player (both derive from Unit).
    [[nodiscard]] bool isUnit() const { return type == ObjectType::UNIT || type == ObjectType::PLAYER; }

    // Fields (for update values)
    void setField(uint16_t index, uint32_t value) {
        fields[index] = value;
    }

    [[nodiscard]] uint32_t getField(uint16_t index) const {
        auto it = fields.find(index);
        return (it != fields.end()) ? it->second : 0;
    }

    [[nodiscard]] bool hasField(uint16_t index) const {
        return fields.find(index) != fields.end();
    }

    [[nodiscard]] const FlatFieldMap& getFields() const {
        return fields;
    }

protected:
    uint64_t guid = 0;
    ObjectType type = ObjectType::OBJECT;

    // Position
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float orientation = 0.0f;

    // Update fields (dynamic values) - flat sorted vector. See FlatFieldMap docs.
    FlatFieldMap fields;

    // Movement interpolation state
    bool isMoving_ = false;
    bool usePathMode_ = false;
    float moveStartX_ = 0, moveStartY_ = 0, moveStartZ_ = 0;
    float moveEndX_ = 0, moveEndY_ = 0, moveEndZ_ = 0;
    float moveDuration_ = 0;
    float moveElapsed_ = 0;
    float velX_ = 0, velY_ = 0, velZ_ = 0;  // Smoothed velocity for dead reckoning
    // CatmullRom spline for multi-segment path movement (replaces linear pathPoints_/pathSegDists_)
    std::optional<math::CatmullRomSpline> activeSpline_;
    uint32_t splineDurationMs_ = 0;
};

/**
 * Unit entity (NPCs, creatures, players)
 */
class Unit : public Entity {
public:
    Unit() { type = ObjectType::UNIT; }
    explicit Unit(uint64_t guid) : Entity(guid) { type = ObjectType::UNIT; }

    // Name
    [[nodiscard]] const std::string& getName() const { return name; }
    void setName(const std::string& n) { name = n; }

    // Health
    [[nodiscard]] uint32_t getHealth() const { return health; }
    void setHealth(uint32_t h) { health = h; }

    [[nodiscard]] uint32_t getMaxHealth() const { return maxHealth; }
    void setMaxHealth(uint32_t h) { maxHealth = h; }

    // Power (mana/rage/energy) - indexed by power type (0-6)
    [[nodiscard]] uint32_t getPower() const { return powers[powerType < 7 ? powerType : 0]; }
    void setPower(uint32_t p) { powers[powerType < 7 ? powerType : 0] = p; }
    void setPowerByType(uint8_t type, uint32_t p) { if (type < 7) powers[type] = p; }

    [[nodiscard]] uint32_t getMaxPower() const { return maxPowers[powerType < 7 ? powerType : 0]; }
    void setMaxPower(uint32_t p) { maxPowers[powerType < 7 ? powerType : 0] = p; }
    void setMaxPowerByType(uint8_t type, uint32_t p) { if (type < 7) maxPowers[type] = p; }

    [[nodiscard]] uint32_t getPowerByType(uint8_t type) const { return type < 7 ? powers[type] : 0; }
    [[nodiscard]] uint32_t getMaxPowerByType(uint8_t type) const { return type < 7 ? maxPowers[type] : 0; }

    [[nodiscard]] uint8_t getPowerType() const { return powerType; }
    void setPowerType(uint8_t t) { powerType = t; }

    [[nodiscard]] uint32_t getAuraState() const { return auraState; }
    void setAuraState(uint32_t state) { auraState = state; }

    // Level
    [[nodiscard]] uint32_t getLevel() const { return level; }
    void setLevel(uint32_t l) { level = l; }

    // Entry ID (creature template entry)
    [[nodiscard]] uint32_t getEntry() const { return entry; }
    void setEntry(uint32_t e) { entry = e; }

    // Display ID (model display)
    [[nodiscard]] uint32_t getDisplayId() const { return displayId; }
    void setDisplayId(uint32_t id) { displayId = id; }
    /// How far this unit's edge is from its centre, and how far past that it
    /// can reach. Range between two units is measured edge to edge, so both
    /// come off the centre distance - see UNIT_FIELD_COMBATREACH.
    [[nodiscard]] float getCombatReach() const { return combatReach; }
    void setCombatReach(float r) { combatReach = r; }
    [[nodiscard]] float getBoundingRadius() const { return boundingRadius; }
    void setBoundingRadius(float r) { boundingRadius = r; }

    // Mount display ID (UNIT_FIELD_MOUNTDISPLAYID, index 69)
    [[nodiscard]] uint32_t getMountDisplayId() const { return mountDisplayId; }
    void setMountDisplayId(uint32_t id) { mountDisplayId = id; }

    // Unit flags (UNIT_FIELD_FLAGS, index 59)
    [[nodiscard]] uint32_t getUnitFlags() const { return unitFlags; }
    void setUnitFlags(uint32_t f) { unitFlags = f; }

    // Client presentation flags (byte 2 of UNIT_FIELD_BYTES_1).
    [[nodiscard]] uint8_t getVisibilityFlags() const { return visibilityFlags; }
    void setVisibilityFlags(uint8_t f) { visibilityFlags = f; }
    [[nodiscard]] bool hasCreepVisibility() const { return (visibilityFlags & UNIT_VIS_FLAG_CREEP) != 0; }
    void clearCreepVisibility() {
        visibilityFlags &= static_cast<uint8_t>(~UNIT_VIS_FLAG_CREEP);
    }

    // Dynamic flags (UNIT_DYNAMIC_FLAGS, index 147)
    [[nodiscard]] uint32_t getDynamicFlags() const { return dynamicFlags; }
    void setDynamicFlags(uint32_t f) { dynamicFlags = f; }

    // NPC flags (UNIT_NPC_FLAGS, index 82)
    [[nodiscard]] uint32_t getNpcFlags() const { return npcFlags; }
    void setNpcFlags(uint32_t f) { npcFlags = f; }

    // NPC emote state (UNIT_NPC_EMOTESTATE) - persistent looping animation for NPCs
    [[nodiscard]] uint32_t getNpcEmoteState() const { return npcEmoteState; }
    void setNpcEmoteState(uint32_t e) { npcEmoteState = e; }

    // Returns true if NPC has interaction flags (gossip/vendor/quest/trainer)
    [[nodiscard]] bool isInteractable() const { return npcFlags != 0; }

    // Faction-based hostility
    [[nodiscard]] uint32_t getFactionTemplate() const { return factionTemplate; }
    void setFactionTemplate(uint32_t f) { factionTemplate = f; }
    [[nodiscard]] bool isHostile() const { return hostile; }
    void setHostile(bool h) { hostile = h; }

protected:
    std::string name;
    uint32_t health = 0;
    uint32_t maxHealth = 0;
    uint32_t powers[7] = {};     // Indexed by power type (0=mana,1=rage,2=focus,3=energy,4=happiness,5=runes,6=runic)
    uint32_t maxPowers[7] = {};  // Max values per power type
    uint8_t powerType = 0;       // Active power type
    uint32_t auraState = 0;      // UNIT_FIELD_AURASTATE reactive opportunity mask
    uint32_t level = 1;
    uint32_t entry = 0;
    uint32_t displayId = 0;
    // Zero until the server says otherwise, which is the right default: it
    // makes an edge-to-edge distance fall back to centre to centre rather
    // than inventing reach for a unit whose size has not arrived.
    float combatReach = 0.0f;
    float boundingRadius = 0.0f;
    uint32_t mountDisplayId = 0;
    uint32_t unitFlags = 0;
    uint8_t visibilityFlags = 0;
    uint32_t dynamicFlags = 0;
    uint32_t npcFlags = 0;
    uint32_t npcEmoteState = 0;
    uint32_t factionTemplate = 0;
    bool hostile = false;
};

/**
 * Player entity
 * Name is inherited from Unit - do NOT redeclare it here or the
 * shadowed field will diverge from Unit::name, causing nameplates
 * and other Unit*-based lookups to read an empty string.
 */
class Player : public Unit {
public:
    Player() { type = ObjectType::PLAYER; }
    explicit Player(uint64_t guid) : Unit(guid) { type = ObjectType::PLAYER; }
};

/**
 * GameObject entity (doors, chests, etc.)
 */
class GameObject : public Entity {
public:
    GameObject() { type = ObjectType::GAMEOBJECT; }
    explicit GameObject(uint64_t guid) : Entity(guid) { type = ObjectType::GAMEOBJECT; }

    [[nodiscard]] const std::string& getName() const { return name; }
    void setName(const std::string& n) { name = n; }

    [[nodiscard]] uint32_t getEntry() const { return entry; }
    void setEntry(uint32_t e) { entry = e; }

    [[nodiscard]] uint32_t getDisplayId() const { return displayId; }
    void setDisplayId(uint32_t id) { displayId = id; }

protected:
    std::string name;
    uint32_t entry = 0;
    uint32_t displayId = 0;
};

/// What to call this entity on screen, whatever kind it turns out to be.
///
/// The name lives on Unit and on GameObject and not on Entity, so anything
/// wanting to write one down has to ask which kind it is holding first. Five
/// places did that for themselves - the four files GameScreen was split into
/// and the chat commands - in identical copies, and two of the four never
/// called their own.
///
/// "Unknown" rather than an empty string, because every caller is putting this
/// in a sentence: an entity whose name has not arrived yet reads as a gap in
/// the line otherwise, with nothing to say a name was expected.
inline std::string entityDisplayName(const std::shared_ptr<Entity>& entity) {
    if (!entity) return "Unknown";
    if (entity->getType() == ObjectType::PLAYER) {
        auto player = std::static_pointer_cast<Player>(entity);
        if (!player->getName().empty()) return player->getName();
    } else if (entity->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(entity);
        if (!unit->getName().empty()) return unit->getName();
    } else if (entity->getType() == ObjectType::GAMEOBJECT) {
        auto go = std::static_pointer_cast<GameObject>(entity);
        if (!go->getName().empty()) return go->getName();
    }
    return "Unknown";
}

/**
 * Entity manager for tracking all entities in view
 */
class EntityManager {
public:
    // Add entity
    void addEntity(uint64_t guid, std::shared_ptr<Entity> entity);

    // Remove entity
    void removeEntity(uint64_t guid);

    // Get entity
    std::shared_ptr<Entity> getEntity(uint64_t guid) const;

    // Check if entity exists
    bool hasEntity(uint64_t guid) const;

    // Main-thread spatial query. The cell index is refreshed at most four times
    // per second, avoiding a full entity-map distance scan every rendered frame.
    std::vector<std::shared_ptr<Entity>> getEntitiesNear(float x, float y, float radius) const;
    void getEntitiesNear(float x, float y, float radius,
                         std::vector<std::shared_ptr<Entity>>& out) const;

    // Get all entities. MAIN-THREAD-ONLY: mutations happen via dispatchQueuedPackets()
    // on the main thread, and this reference is not lock-protected. Callers on any
    // other thread (e.g. the headless HTTP API thread) must use snapshotEntities()
    // instead, which is safe to call from anywhere.
    const std::unordered_map<uint64_t, std::shared_ptr<Entity>>& getEntities() const {
        return entities;
    }

    // Thread-safe copy of all tracked entities, safe to call from any thread.
    std::vector<std::shared_ptr<Entity>> snapshotEntities() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::shared_ptr<Entity>> snapshot;
        snapshot.reserve(entities.size());
        for (const auto& [guid, entity] : entities) {
            snapshot.push_back(entity);
        }
        return snapshot;
    }

    // Clear all entities
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        entities.clear();
        spatialCells_.clear();
        spatialDirty_ = true;
    }

    // Get entity count
    size_t getEntityCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entities.size();
    }

private:
    // MAIN-THREAD-ONLY for getEntities(): all entity map mutations happen via
    // dispatchQueuedPackets() on the main thread. mutex_ guards addEntity/removeEntity/
    // getEntity/hasEntity/snapshotEntities so those are safe to call cross-thread (the
    // headless HTTP API thread reads player HP and nearby entities this way); it is not
    // taken by the unlocked getEntities() reference accessor above, which remains
    // main-thread-only.
    mutable std::mutex mutex_;
    std::unordered_map<uint64_t, std::shared_ptr<Entity>> entities;
    static constexpr float kSpatialCellSize = 64.0f;
    mutable std::unordered_map<int64_t, std::vector<std::shared_ptr<Entity>>> spatialCells_;
    mutable std::chrono::steady_clock::time_point lastSpatialRebuild_{};
    mutable bool spatialDirty_ = true;
};

/// Who a unit has selected, from the two halves the wire splits the guid into.
///
/// UNIT_FIELD_TARGET arrives as a low and a high update field, and reading it
/// was written out separately in six files - the combat handler, two unit
/// APIs, the target frames, the nameplates and a slash command. Six copies of
/// a two-field read is how one of them comes to be missing its high half, and
/// a guid missing its high half matches nothing on a server that uses one.
inline uint64_t unitTargetGuid(const Entity& entity) {
    const auto& fields = entity.getFields();
    auto lo = fields.find(fieldIndex(UF::UNIT_FIELD_TARGET_LO));
    if (lo == fields.end()) return 0;
    uint64_t guid = lo->second;
    auto hi = fields.find(fieldIndex(UF::UNIT_FIELD_TARGET_HI));
    if (hi != fields.end()) guid |= (static_cast<uint64_t>(hi->second) << 32);
    return guid;
}

inline uint64_t unitTargetGuid(const std::shared_ptr<Entity>& entity) {
    return entity ? unitTargetGuid(*entity) : 0;
}

/// Who summoned this unit, or 0 for a unit nobody did.
///
/// Answers 0 on an expansion whose table has no entry for the field, which is
/// the same answer as "nobody summoned it" on purpose: the callers use it to
/// decide whether a unit is somebody's pet, and not knowing has to read as no.
inline uint64_t unitSummonedByGuid(const Entity& entity) {
    const auto& fields = entity.getFields();
    auto lo = fields.find(fieldIndex(UF::UNIT_FIELD_SUMMONEDBY_LO));
    if (lo == fields.end()) return 0;
    uint64_t guid = lo->second;
    auto hi = fields.find(fieldIndex(UF::UNIT_FIELD_SUMMONEDBY_HI));
    if (hi != fields.end()) guid |= (static_cast<uint64_t>(hi->second) << 32);
    return guid;
}

} // namespace game
} // namespace wowee
