#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace wowee {
namespace game {

/**
 * Logical update field identifiers (expansion-agnostic).
 * Wire indices are loaded at runtime from JSON.
 */
enum class UF : uint16_t {
    // Object fields
    OBJECT_FIELD_ENTRY,

    // Unit fields
    UNIT_FIELD_TARGET_LO,
    UNIT_FIELD_TARGET_HI,
    UNIT_FIELD_BYTES_0,
    UNIT_FIELD_HEALTH,
    UNIT_FIELD_POWER1,
    UNIT_FIELD_MAXHEALTH,
    UNIT_FIELD_MAXPOWER1,
    UNIT_FIELD_LEVEL,
    UNIT_FIELD_FACTIONTEMPLATE,
    UNIT_FIELD_FLAGS,
    UNIT_FIELD_FLAGS_2,
    UNIT_FIELD_DISPLAYID,
    UNIT_FIELD_MOUNTDISPLAYID,
    UNIT_FIELD_AURAS,           // Start of aura spell ID array (48 consecutive uint32 slots, classic/vanilla only)
    UNIT_NPC_FLAGS,
    UNIT_DYNAMIC_FLAGS,
    UNIT_FIELD_RESISTANCES,   // Physical armor (index 0 of the resistance array)
    UNIT_END,

    // Player fields
    PLAYER_FLAGS,
    PLAYER_BYTES,
    PLAYER_BYTES_2,
    PLAYER_XP,
    PLAYER_NEXT_LEVEL_XP,
    PLAYER_FIELD_COINAGE,
    PLAYER_QUEST_LOG_START,
    PLAYER_FIELD_INV_SLOT_HEAD,
    PLAYER_FIELD_PACK_SLOT_1,
    PLAYER_FIELD_BANK_SLOT_1,
    PLAYER_FIELD_BANKBAG_SLOT_1,
    PLAYER_SKILL_INFO_START,
    PLAYER_EXPLORED_ZONES_START,

    // GameObject fields
    GAMEOBJECT_DISPLAYID,

    // Item fields
    ITEM_FIELD_STACK_COUNT,

    // Container fields
    CONTAINER_FIELD_NUM_SLOTS,
    CONTAINER_FIELD_SLOT_1,

    COUNT
};

/**
 * Maps logical update field names to expansion-specific wire indices.
 * Loaded from JSON (e.g. Data/expansions/wotlk/update_fields.json).
 */
class UpdateFieldTable {
public:
    /** Load from JSON file. Returns true if successful. */
    bool loadFromJson(const std::string& path);

    /** Get the wire index for a logical field. Returns 0xFFFF if unknown. */
    uint16_t index(UF field) const;

    /** Override a wire index at runtime (used for auto-detecting custom field layouts). */
    void setIndex(UF field, uint16_t idx) { fieldMap_[static_cast<uint16_t>(field)] = idx; }

    /** Check if a field is mapped. */
    bool hasField(UF field) const;

    /** Number of mapped fields. */
    size_t size() const { return fieldMap_.size(); }

private:
    std::unordered_map<uint16_t, uint16_t> fieldMap_;  // UF enum → wire index
};

/**
 * Global active update field table (set by Application at startup).
 */
void setActiveUpdateFieldTable(const UpdateFieldTable* table);
const UpdateFieldTable* getActiveUpdateFieldTable();

/** Convenience: get wire index for a logical field. */
inline uint16_t fieldIndex(UF field) {
    const auto* t = getActiveUpdateFieldTable();
    return t ? t->index(field) : 0xFFFF;
}

} // namespace game
} // namespace wowee
