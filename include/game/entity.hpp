#pragma once

#include <cstdint>
#include <string>
#include <map>
#include <memory>

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

/**
 * Base entity class for all game objects
 */
class Entity {
public:
    Entity() = default;
    explicit Entity(uint64_t guid) : guid(guid) {}
    virtual ~Entity() = default;

    // GUID access
    uint64_t getGuid() const { return guid; }
    void setGuid(uint64_t g) { guid = g; }

    // Position
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    float getOrientation() const { return orientation; }

    void setPosition(float px, float py, float pz, float o) {
        x = px;
        y = py;
        z = pz;
        orientation = o;
    }

    // Object type
    ObjectType getType() const { return type; }
    void setType(ObjectType t) { type = t; }

    // Fields (for update values)
    void setField(uint16_t index, uint32_t value) {
        fields[index] = value;
    }

    uint32_t getField(uint16_t index) const {
        auto it = fields.find(index);
        return (it != fields.end()) ? it->second : 0;
    }

    bool hasField(uint16_t index) const {
        return fields.find(index) != fields.end();
    }

    const std::map<uint16_t, uint32_t>& getFields() const {
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

    // Update fields (dynamic values)
    std::map<uint16_t, uint32_t> fields;
};

/**
 * Unit entity (NPCs, creatures, players)
 */
class Unit : public Entity {
public:
    Unit() { type = ObjectType::UNIT; }
    explicit Unit(uint64_t guid) : Entity(guid) { type = ObjectType::UNIT; }

    // Name
    const std::string& getName() const { return name; }
    void setName(const std::string& n) { name = n; }

    // Health
    uint32_t getHealth() const { return health; }
    void setHealth(uint32_t h) { health = h; }

    uint32_t getMaxHealth() const { return maxHealth; }
    void setMaxHealth(uint32_t h) { maxHealth = h; }

    // Power (mana/rage/energy)
    uint32_t getPower() const { return power; }
    void setPower(uint32_t p) { power = p; }

    uint32_t getMaxPower() const { return maxPower; }
    void setMaxPower(uint32_t p) { maxPower = p; }

    uint8_t getPowerType() const { return powerType; }
    void setPowerType(uint8_t t) { powerType = t; }

    // Level
    uint32_t getLevel() const { return level; }
    void setLevel(uint32_t l) { level = l; }

    // Entry ID (creature template entry)
    uint32_t getEntry() const { return entry; }
    void setEntry(uint32_t e) { entry = e; }

    // Display ID (model display)
    uint32_t getDisplayId() const { return displayId; }
    void setDisplayId(uint32_t id) { displayId = id; }

protected:
    std::string name;
    uint32_t health = 0;
    uint32_t maxHealth = 0;
    uint32_t power = 0;
    uint32_t maxPower = 0;
    uint8_t powerType = 0;   // 0=mana, 1=rage, 2=focus, 3=energy
    uint32_t level = 1;
    uint32_t entry = 0;
    uint32_t displayId = 0;
};

/**
 * Player entity
 */
class Player : public Unit {
public:
    Player() { type = ObjectType::PLAYER; }
    explicit Player(uint64_t guid) : Unit(guid) { type = ObjectType::PLAYER; }

    // Name
    const std::string& getName() const { return name; }
    void setName(const std::string& n) { name = n; }

protected:
    std::string name;
};

/**
 * GameObject entity (doors, chests, etc.)
 */
class GameObject : public Entity {
public:
    GameObject() { type = ObjectType::GAMEOBJECT; }
    explicit GameObject(uint64_t guid) : Entity(guid) { type = ObjectType::GAMEOBJECT; }

    uint32_t getDisplayId() const { return displayId; }
    void setDisplayId(uint32_t id) { displayId = id; }

protected:
    uint32_t displayId = 0;
};

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

    // Get all entities
    const std::map<uint64_t, std::shared_ptr<Entity>>& getEntities() const {
        return entities;
    }

    // Clear all entities
    void clear() {
        entities.clear();
    }

    // Get entity count
    size_t getEntityCount() const {
        return entities.size();
    }

private:
    std::map<uint64_t, std::shared_ptr<Entity>> entities;
};

} // namespace game
} // namespace wowee
