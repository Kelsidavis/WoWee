#include "game/update_field_table.hpp"
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
    {"OBJECT_FIELD_ENTRY", UF::OBJECT_FIELD_ENTRY},
    {"UNIT_FIELD_TARGET_LO", UF::UNIT_FIELD_TARGET_LO},
    {"UNIT_FIELD_TARGET_HI", UF::UNIT_FIELD_TARGET_HI},
    {"UNIT_FIELD_BYTES_0", UF::UNIT_FIELD_BYTES_0},
    {"UNIT_FIELD_HEALTH", UF::UNIT_FIELD_HEALTH},
    {"UNIT_FIELD_POWER1", UF::UNIT_FIELD_POWER1},
    {"UNIT_FIELD_MAXHEALTH", UF::UNIT_FIELD_MAXHEALTH},
    {"UNIT_FIELD_MAXPOWER1", UF::UNIT_FIELD_MAXPOWER1},
    {"UNIT_FIELD_LEVEL", UF::UNIT_FIELD_LEVEL},
    {"UNIT_FIELD_FACTIONTEMPLATE", UF::UNIT_FIELD_FACTIONTEMPLATE},
    {"UNIT_FIELD_FLAGS", UF::UNIT_FIELD_FLAGS},
    {"UNIT_FIELD_FLAGS_2", UF::UNIT_FIELD_FLAGS_2},
    {"UNIT_FIELD_DISPLAYID", UF::UNIT_FIELD_DISPLAYID},
    {"UNIT_FIELD_MOUNTDISPLAYID", UF::UNIT_FIELD_MOUNTDISPLAYID},
    {"UNIT_FIELD_AURAS", UF::UNIT_FIELD_AURAS},
    {"UNIT_NPC_FLAGS", UF::UNIT_NPC_FLAGS},
    {"UNIT_DYNAMIC_FLAGS", UF::UNIT_DYNAMIC_FLAGS},
    {"UNIT_FIELD_RESISTANCES", UF::UNIT_FIELD_RESISTANCES},
    {"UNIT_END", UF::UNIT_END},
    {"PLAYER_FLAGS", UF::PLAYER_FLAGS},
    {"PLAYER_BYTES", UF::PLAYER_BYTES},
    {"PLAYER_BYTES_2", UF::PLAYER_BYTES_2},
    {"PLAYER_XP", UF::PLAYER_XP},
    {"PLAYER_NEXT_LEVEL_XP", UF::PLAYER_NEXT_LEVEL_XP},
    {"PLAYER_FIELD_COINAGE", UF::PLAYER_FIELD_COINAGE},
    {"PLAYER_QUEST_LOG_START", UF::PLAYER_QUEST_LOG_START},
    {"PLAYER_FIELD_INV_SLOT_HEAD", UF::PLAYER_FIELD_INV_SLOT_HEAD},
    {"PLAYER_FIELD_PACK_SLOT_1", UF::PLAYER_FIELD_PACK_SLOT_1},
    {"PLAYER_FIELD_BANK_SLOT_1", UF::PLAYER_FIELD_BANK_SLOT_1},
    {"PLAYER_FIELD_BANKBAG_SLOT_1", UF::PLAYER_FIELD_BANKBAG_SLOT_1},
    {"PLAYER_SKILL_INFO_START", UF::PLAYER_SKILL_INFO_START},
    {"PLAYER_EXPLORED_ZONES_START", UF::PLAYER_EXPLORED_ZONES_START},
    {"GAMEOBJECT_DISPLAYID", UF::GAMEOBJECT_DISPLAYID},
    {"ITEM_FIELD_STACK_COUNT", UF::ITEM_FIELD_STACK_COUNT},
    {"CONTAINER_FIELD_NUM_SLOTS", UF::CONTAINER_FIELD_NUM_SLOTS},
    {"CONTAINER_FIELD_SLOT_1", UF::CONTAINER_FIELD_SLOT_1},
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
    size_t pos = 0;

    while (pos < json.size()) {
        size_t keyStart = json.find('"', pos);
        if (keyStart == std::string::npos) break;
        size_t keyEnd = json.find('"', keyStart + 1);
        if (keyEnd == std::string::npos) break;
        std::string key = json.substr(keyStart + 1, keyEnd - keyStart - 1);

        size_t colon = json.find(':', keyEnd);
        if (colon == std::string::npos) break;

        size_t valStart = colon + 1;
        while (valStart < json.size() && (json[valStart] == ' ' || json[valStart] == '\t' ||
               json[valStart] == '\r' || json[valStart] == '\n'))
            ++valStart;

        size_t valEnd = json.find_first_of(",}\r\n", valStart);
        if (valEnd == std::string::npos) valEnd = json.size();
        std::string valStr = json.substr(valStart, valEnd - valStart);
        // Trim whitespace
        while (!valStr.empty() && (valStr.back() == ' ' || valStr.back() == '\t'))
            valStr.pop_back();

        uint16_t idx = 0;
        try { idx = static_cast<uint16_t>(std::stoul(valStr)); } catch (...) {
            pos = valEnd + 1;
            continue;
        }

        // Find matching UF enum
        for (size_t i = 0; i < kUFNameCount; ++i) {
            if (key == kUFNames[i].name) {
                fieldMap_[static_cast<uint16_t>(kUFNames[i].field)] = idx;
                ++loaded;
                break;
            }
        }

        pos = valEnd + 1;
    }

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
