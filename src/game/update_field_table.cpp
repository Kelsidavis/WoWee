#include "game/update_field_table.hpp"
#include "game/json_table_scan.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace wowee {
namespace game {

static const UpdateFieldTable* g_activeUpdateFieldTable = nullptr;

void setActiveUpdateFieldTable(const UpdateFieldTable* table) { g_activeUpdateFieldTable = table; }
const UpdateFieldTable* getActiveUpdateFieldTable() { return g_activeUpdateFieldTable; }

struct UFNameEntry {
    const char* name;
    UF field;
};

static const UFNameEntry kUFNames[] = {
    {.name = "OBJECT_FIELD_ENTRY", .field = UF::OBJECT_FIELD_ENTRY},
    {.name = "OBJECT_FIELD_SCALE_X", .field = UF::OBJECT_FIELD_SCALE_X},
    {.name = "UNIT_FIELD_TARGET_LO", .field = UF::UNIT_FIELD_TARGET_LO},
    {.name = "UNIT_FIELD_TARGET_HI", .field = UF::UNIT_FIELD_TARGET_HI},
    {.name = "UNIT_FIELD_BYTES_0", .field = UF::UNIT_FIELD_BYTES_0},
    {.name = "UNIT_FIELD_BYTES_1", .field = UF::UNIT_FIELD_BYTES_1},
    {.name = "UNIT_FIELD_HEALTH", .field = UF::UNIT_FIELD_HEALTH},
    {.name = "UNIT_FIELD_POWER1", .field = UF::UNIT_FIELD_POWER1},
    {.name = "UNIT_FIELD_MAXHEALTH", .field = UF::UNIT_FIELD_MAXHEALTH},
    {.name = "UNIT_FIELD_MAXPOWER1", .field = UF::UNIT_FIELD_MAXPOWER1},
    {.name = "UNIT_FIELD_LEVEL", .field = UF::UNIT_FIELD_LEVEL},
    {.name = "UNIT_FIELD_FACTIONTEMPLATE", .field = UF::UNIT_FIELD_FACTIONTEMPLATE},
    {.name = "UNIT_FIELD_FLAGS", .field = UF::UNIT_FIELD_FLAGS},
    {.name = "UNIT_FIELD_FLAGS_2", .field = UF::UNIT_FIELD_FLAGS_2},
    {.name = "UNIT_FIELD_AURASTATE", .field = UF::UNIT_FIELD_AURASTATE},
    {.name = "UNIT_FIELD_BOUNDINGRADIUS", .field = UF::UNIT_FIELD_BOUNDINGRADIUS},
    {.name = "UNIT_FIELD_COMBATREACH", .field = UF::UNIT_FIELD_COMBATREACH},
    {.name = "UNIT_FIELD_DISPLAYID", .field = UF::UNIT_FIELD_DISPLAYID},
    {.name = "UNIT_FIELD_MOUNTDISPLAYID", .field = UF::UNIT_FIELD_MOUNTDISPLAYID},
    {.name = "UNIT_FIELD_AURAS", .field = UF::UNIT_FIELD_AURAS},
    {.name = "UNIT_FIELD_AURAFLAGS", .field = UF::UNIT_FIELD_AURAFLAGS},
    {.name = "UNIT_NPC_FLAGS", .field = UF::UNIT_NPC_FLAGS},
    {.name = "UNIT_NPC_EMOTESTATE", .field = UF::UNIT_NPC_EMOTESTATE},
    {.name = "UNIT_DYNAMIC_FLAGS", .field = UF::UNIT_DYNAMIC_FLAGS},
    {.name = "UNIT_FIELD_MINDAMAGE", .field = UF::UNIT_FIELD_MINDAMAGE},
    {.name = "UNIT_FIELD_MAXDAMAGE", .field = UF::UNIT_FIELD_MAXDAMAGE},
    {.name = "UNIT_FIELD_PETEXPERIENCE", .field = UF::UNIT_FIELD_PETEXPERIENCE},
    {.name = "UNIT_FIELD_PETNEXTLEVELEXP", .field = UF::UNIT_FIELD_PETNEXTLEVELEXP},
    {.name = "UNIT_FIELD_RESISTANCES", .field = UF::UNIT_FIELD_RESISTANCES},
    {.name = "UNIT_FIELD_STAT0", .field = UF::UNIT_FIELD_STAT0},
    {.name = "UNIT_FIELD_STAT1", .field = UF::UNIT_FIELD_STAT1},
    {.name = "UNIT_FIELD_STAT2", .field = UF::UNIT_FIELD_STAT2},
    {.name = "UNIT_FIELD_STAT3", .field = UF::UNIT_FIELD_STAT3},
    {.name = "UNIT_FIELD_STAT4", .field = UF::UNIT_FIELD_STAT4},
    {.name = "UNIT_END", .field = UF::UNIT_END},
    {.name = "UNIT_FIELD_ATTACK_POWER", .field = UF::UNIT_FIELD_ATTACK_POWER},
    {.name = "UNIT_FIELD_RANGED_ATTACK_POWER", .field = UF::UNIT_FIELD_RANGED_ATTACK_POWER},
    {.name = "UNIT_FIELD_POWER_REGEN_FLAT_MODIFIER", .field = UF::UNIT_FIELD_POWER_REGEN_FLAT_MODIFIER},
    {.name = "UNIT_FIELD_POWER_REGEN_INTERRUPTED_FLAT_MODIFIER", .field = UF::UNIT_FIELD_POWER_REGEN_INTERRUPTED_FLAT_MODIFIER},
    {.name = "PLAYER_FLAGS", .field = UF::PLAYER_FLAGS},
    {.name = "PLAYER_BYTES", .field = UF::PLAYER_BYTES},
    {.name = "PLAYER_BYTES_2", .field = UF::PLAYER_BYTES_2},
    {.name = "PLAYER_XP", .field = UF::PLAYER_XP},
    {.name = "PLAYER_NEXT_LEVEL_XP", .field = UF::PLAYER_NEXT_LEVEL_XP},
    {.name = "PLAYER_FIELD_COINAGE", .field = UF::PLAYER_FIELD_COINAGE},
    {.name = "PLAYER_QUEST_LOG_START", .field = UF::PLAYER_QUEST_LOG_START},
    {.name = "PLAYER_FIELD_INV_SLOT_HEAD", .field = UF::PLAYER_FIELD_INV_SLOT_HEAD},
    {.name = "PLAYER_FIELD_PACK_SLOT_1", .field = UF::PLAYER_FIELD_PACK_SLOT_1},
    {.name = "PLAYER_FIELD_KEYRING_SLOT_1", .field = UF::PLAYER_FIELD_KEYRING_SLOT_1},
    {.name = "PLAYER_FIELD_BANK_SLOT_1", .field = UF::PLAYER_FIELD_BANK_SLOT_1},
    {.name = "PLAYER_FIELD_BANKBAG_SLOT_1", .field = UF::PLAYER_FIELD_BANKBAG_SLOT_1},
    {.name = "PLAYER_SKILL_INFO_START", .field = UF::PLAYER_SKILL_INFO_START},
    {.name = "PLAYER_EXPLORED_ZONES_START", .field = UF::PLAYER_EXPLORED_ZONES_START},
    {.name = "UNIT_FIELD_CRITTER", .field = UF::UNIT_FIELD_CRITTER},
    {.name = "UNIT_FIELD_CHARM", .field = UF::UNIT_FIELD_CHARM},
    {.name = "UNIT_FIELD_CHARMEDBY", .field = UF::UNIT_FIELD_CHARMEDBY},
    {.name = "UNIT_FIELD_SUMMONEDBY_LO", .field = UF::UNIT_FIELD_SUMMONEDBY_LO},
    {.name = "UNIT_FIELD_SUMMONEDBY_HI", .field = UF::UNIT_FIELD_SUMMONEDBY_HI},
    {.name = "GAMEOBJECT_DISPLAYID", .field = UF::GAMEOBJECT_DISPLAYID},
    {.name = "GAMEOBJECT_BYTES_1", .field = UF::GAMEOBJECT_BYTES_1},
    {.name = "GAMEOBJECT_DYNAMIC", .field = UF::GAMEOBJECT_DYNAMIC},
    {.name = "GAMEOBJECT_LEVEL", .field = UF::GAMEOBJECT_LEVEL},
    {.name = "ITEM_FIELD_STACK_COUNT", .field = UF::ITEM_FIELD_STACK_COUNT},
    {.name = "ITEM_FIELD_DURABILITY", .field = UF::ITEM_FIELD_DURABILITY},
    {.name = "ITEM_FIELD_MAXDURABILITY", .field = UF::ITEM_FIELD_MAXDURABILITY},
    {.name = "PLAYER_REST_STATE_EXPERIENCE", .field = UF::PLAYER_REST_STATE_EXPERIENCE},
    {.name = "PLAYER_CHOSEN_TITLE", .field = UF::PLAYER_CHOSEN_TITLE},
    {.name = "PLAYER_FIELD_MOD_DAMAGE_DONE_POS", .field = UF::PLAYER_FIELD_MOD_DAMAGE_DONE_POS},
    {.name = "PLAYER_FIELD_MOD_HEALING_DONE_POS", .field = UF::PLAYER_FIELD_MOD_HEALING_DONE_POS},
    {.name = "PLAYER_BLOCK_PERCENTAGE", .field = UF::PLAYER_BLOCK_PERCENTAGE},
    {.name = "PLAYER_DODGE_PERCENTAGE", .field = UF::PLAYER_DODGE_PERCENTAGE},
    {.name = "PLAYER_PARRY_PERCENTAGE", .field = UF::PLAYER_PARRY_PERCENTAGE},
    {.name = "PLAYER_CRIT_PERCENTAGE", .field = UF::PLAYER_CRIT_PERCENTAGE},
    {.name = "PLAYER_RANGED_CRIT_PERCENTAGE", .field = UF::PLAYER_RANGED_CRIT_PERCENTAGE},
    {.name = "PLAYER_SPELL_CRIT_PERCENTAGE1", .field = UF::PLAYER_SPELL_CRIT_PERCENTAGE1},
    {.name = "PLAYER_FIELD_COMBAT_RATING_1", .field = UF::PLAYER_FIELD_COMBAT_RATING_1},
    {.name = "PLAYER_EXPERTISE", .field = UF::PLAYER_EXPERTISE},
    {.name = "PLAYER_OFFHAND_EXPERTISE", .field = UF::PLAYER_OFFHAND_EXPERTISE},
    {.name = "PLAYER_FIELD_HONOR_CURRENCY", .field = UF::PLAYER_FIELD_HONOR_CURRENCY},
    {.name = "PLAYER_FIELD_ARENA_CURRENCY", .field = UF::PLAYER_FIELD_ARENA_CURRENCY},
    {.name = "CONTAINER_FIELD_NUM_SLOTS", .field = UF::CONTAINER_FIELD_NUM_SLOTS},
    {.name = "CONTAINER_FIELD_SLOT_1", .field = UF::CONTAINER_FIELD_SLOT_1},
};

static constexpr size_t kUFNameCount = sizeof(kUFNames) / sizeof(kUFNames[0]);

bool UpdateFieldTable::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("UpdateFieldTable: cannot open ", path);
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    fieldMap_.clear();
    size_t loaded = 0;

    forEachJsonKeyValue(json, [&](const std::string& key, const std::string& valStr) {
        uint32_t parsed = 0;
        // Skipped rather than stored when it is not wholly a number. Storing
        // what a partial parse returned would put index 0 - the object GUID -
        // under a name that reads correctly everywhere it is used.
        if (!parseTableNumber(valStr, parsed)) return;
        const uint16_t idx = static_cast<uint16_t>(parsed);

        for (size_t i = 0; i < kUFNameCount; ++i) {
            if (key == kUFNames[i].name) {
                fieldMap_[static_cast<uint16_t>(kUFNames[i].field)] = idx;
                ++loaded;
                break;
            }
        }
    });

    if (loaded == 0) {
        LOG_WARNING("UpdateFieldTable: no fields loaded from ", path);
        return false;
    }

    LOG_INFO("UpdateFieldTable: loaded ", loaded, " fields from ", path);
    return true;
}

uint16_t UpdateFieldTable::index(UF field) const {
    auto it = fieldMap_.find(static_cast<uint16_t>(field));
    return (it != fieldMap_.end()) ? it->second : 0xFFFF;
}

bool UpdateFieldTable::hasField(UF field) const {
    return fieldMap_.count(static_cast<uint16_t>(field)) > 0;
}

} // namespace game
} // namespace wowee
