#include "game/game_handler.hpp"
#include "game/opcodes.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <random>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <sqlite3.h>
#include <zlib.h>

namespace wowee {
namespace game {

namespace {

struct LootEntryRow {
    uint32_t item = 0;
    float chance = 0.0f;
    uint16_t lootmode = 0;
    uint8_t groupid = 0;
    int32_t mincountOrRef = 0;
    uint8_t maxcount = 1;
};

struct CreatureTemplateRow {
    uint32_t lootId = 0;
    uint32_t minGold = 0;
    uint32_t maxGold = 0;
};

struct ItemTemplateRow {
    uint32_t itemId = 0;
    std::string name;
    uint32_t displayId = 0;
    uint8_t quality = 0;
    uint8_t inventoryType = 0;
    int32_t maxStack = 1;
    uint32_t sellPrice = 0;
    int32_t armor = 0;
    int32_t stamina = 0;
    int32_t strength = 0;
    int32_t agility = 0;
    int32_t intellect = 0;
    int32_t spirit = 0;
};

struct SinglePlayerLootDb {
    bool loaded = false;
    std::string basePath;
    std::unordered_map<uint32_t, CreatureTemplateRow> creatureTemplates;
    std::unordered_map<uint32_t, std::vector<LootEntryRow>> creatureLoot;
    std::unordered_map<uint32_t, std::vector<LootEntryRow>> referenceLoot;
    std::unordered_map<uint32_t, ItemTemplateRow> itemTemplates;
};

struct SinglePlayerCreateDb {
    bool loaded = false;
    std::unordered_map<uint16_t, GameHandler::SinglePlayerCreateInfo> rows;
};

struct SinglePlayerStartDb {
    bool loaded = false;
    struct StartItemRow {
        uint8_t race = 0;
        uint8_t cls = 0;
        uint32_t itemId = 0;
        int32_t amount = 1;
    };
    struct StartSpellRow {
        uint32_t raceMask = 0;
        uint32_t classMask = 0;
        uint32_t spellId = 0;
    };
    struct StartActionRow {
        uint8_t race = 0;
        uint8_t cls = 0;
        uint16_t button = 0;
        uint32_t action = 0;
        uint16_t type = 0;
    };
    std::vector<StartItemRow> items;
    std::vector<StartSpellRow> spells;
    std::vector<StartActionRow> actions;
};

struct SinglePlayerSqlite {
    sqlite3* db = nullptr;
    std::filesystem::path path;

    bool open() {
        if (db) return true;
        path = std::filesystem::path("saves");
        std::error_code ec;
        std::filesystem::create_directories(path, ec);
        path /= "singleplayer.db";
        if (sqlite3_open(path.string().c_str(), &db) != SQLITE_OK) {
            db = nullptr;
            return false;
        }
        sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
        sqlite3_exec(db, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);
        return true;
    }

    void close() {
        if (db) {
            sqlite3_close(db);
            db = nullptr;
        }
    }

    bool exec(const char* sql) const {
        if (!db) return false;
        char* err = nullptr;
        int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
        if (err) sqlite3_free(err);
        return rc == SQLITE_OK;
    }

    bool ensureSchema() const {
        static const char* kSchema =
            "CREATE TABLE IF NOT EXISTS characters ("
            " guid INTEGER PRIMARY KEY,"
            " name TEXT,"
            " race INTEGER,"
            " \"class\" INTEGER,"
            " gender INTEGER,"
            " level INTEGER,"
            " appearance_bytes INTEGER,"
            " facial_features INTEGER,"
            " zone INTEGER,"
            " map INTEGER,"
            " position_x REAL,"
            " position_y REAL,"
            " position_z REAL,"
            " orientation REAL,"
            " money INTEGER,"
            " xp INTEGER,"
            " health INTEGER,"
            " max_health INTEGER,"
            " has_state INTEGER DEFAULT 0"
            ");"
            "CREATE TABLE IF NOT EXISTS character_inventory ("
            " guid INTEGER,"
            " location INTEGER,"
            " slot INTEGER,"
            " item_id INTEGER,"
            " name TEXT,"
            " quality INTEGER,"
            " inventory_type INTEGER,"
            " stack_count INTEGER,"
            " max_stack INTEGER,"
            " bag_slots INTEGER,"
            " armor INTEGER,"
            " stamina INTEGER,"
            " strength INTEGER,"
            " agility INTEGER,"
            " intellect INTEGER,"
            " spirit INTEGER,"
            " display_info_id INTEGER,"
            " subclass_name TEXT,"
            " sell_price INTEGER DEFAULT 0,"
            " PRIMARY KEY (guid, location, slot)"
            ");"
            "CREATE TABLE IF NOT EXISTS character_spell ("
            " guid INTEGER,"
            " spell INTEGER,"
            " PRIMARY KEY (guid, spell)"
            ");"
            "CREATE TABLE IF NOT EXISTS character_action ("
            " guid INTEGER,"
            " slot INTEGER,"
            " type INTEGER,"
            " action INTEGER,"
            " PRIMARY KEY (guid, slot)"
            ");"
            "CREATE TABLE IF NOT EXISTS character_aura ("
            " guid INTEGER,"
            " slot INTEGER,"
            " spell INTEGER,"
            " flags INTEGER,"
            " level INTEGER,"
            " charges INTEGER,"
            " duration_ms INTEGER,"
            " max_duration_ms INTEGER,"
            " caster_guid INTEGER,"
            " PRIMARY KEY (guid, slot)"
            ");"
            "CREATE TABLE IF NOT EXISTS character_queststatus ("
            " guid INTEGER,"
            " quest INTEGER,"
            " status INTEGER,"
            " progress INTEGER,"
            " PRIMARY KEY (guid, quest)"
            ");"
            "CREATE TABLE IF NOT EXISTS character_settings ("
            " guid INTEGER PRIMARY KEY,"
            " fullscreen INTEGER,"
            " vsync INTEGER,"
            " shadows INTEGER,"
            " res_w INTEGER,"
            " res_h INTEGER,"
            " music_volume INTEGER,"
            " sfx_volume INTEGER,"
            " mouse_sensitivity REAL,"
            " invert_mouse INTEGER"
            ");";
        if (!exec(kSchema)) return false;
        // Migration: add sell_price column to existing saves
        exec("ALTER TABLE character_inventory ADD COLUMN sell_price INTEGER DEFAULT 0;");
        return true;
    }
};

static SinglePlayerSqlite& getSinglePlayerSqlite() {
    static SinglePlayerSqlite sp;
    if (!sp.db) {
        if (sp.open()) {
            sp.ensureSchema();
        }
    }
    return sp;
}

static uint32_t removeItemsFromInventory(Inventory& inventory, uint32_t itemId, uint32_t amount) {
    if (itemId == 0 || amount == 0) return 0;
    uint32_t remaining = amount;

    for (int i = 0; i < Inventory::BACKPACK_SLOTS && remaining > 0; i++) {
        const ItemSlot& slot = inventory.getBackpackSlot(i);
        if (slot.empty() || slot.item.itemId != itemId) continue;
        if (slot.item.stackCount <= remaining) {
            remaining -= slot.item.stackCount;
            inventory.clearBackpackSlot(i);
        } else {
            ItemDef updated = slot.item;
            updated.stackCount -= remaining;
            inventory.setBackpackSlot(i, updated);
            remaining = 0;
        }
    }

    for (int i = 0; i < Inventory::NUM_EQUIP_SLOTS && remaining > 0; i++) {
        EquipSlot slotId = static_cast<EquipSlot>(i);
        const ItemSlot& slot = inventory.getEquipSlot(slotId);
        if (slot.empty() || slot.item.itemId != itemId) continue;
        if (slot.item.stackCount <= remaining) {
            remaining -= slot.item.stackCount;
            inventory.clearEquipSlot(slotId);
        } else {
            ItemDef updated = slot.item;
            updated.stackCount -= remaining;
            inventory.setEquipSlot(slotId, updated);
            remaining = 0;
        }
    }

    for (int bag = 0; bag < Inventory::NUM_BAG_SLOTS && remaining > 0; bag++) {
        int bagSize = inventory.getBagSize(bag);
        for (int slotIndex = 0; slotIndex < bagSize && remaining > 0; slotIndex++) {
            const ItemSlot& slot = inventory.getBagSlot(bag, slotIndex);
            if (slot.empty() || slot.item.itemId != itemId) continue;
            if (slot.item.stackCount <= remaining) {
                remaining -= slot.item.stackCount;
                inventory.setBagSlot(bag, slotIndex, ItemDef{});
            } else {
                ItemDef updated = slot.item;
                updated.stackCount -= remaining;
                inventory.setBagSlot(bag, slotIndex, updated);
                remaining = 0;
            }
        }
    }

    return amount - remaining;
}

static std::string trimSql(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) b++;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) e--;
    return s.substr(b, e - b);
}

static bool parseInsertTuples(const std::string& line, std::vector<std::string>& outTuples) {
    outTuples.clear();
    size_t valuesPos = line.find("VALUES");
    if (valuesPos == std::string::npos) valuesPos = line.find("values");
    if (valuesPos == std::string::npos) return false;

    bool inQuote = false;
    int depth = 0;
    size_t tupleStart = std::string::npos;
    for (size_t i = valuesPos; i < line.size(); i++) {
        char c = line[i];
        if (c == '\'' && (i == 0 || line[i - 1] != '\\')) inQuote = !inQuote;
        if (inQuote) continue;
        if (c == '(') {
            if (depth == 0) tupleStart = i + 1;
            depth++;
        } else if (c == ')') {
            depth--;
            if (depth == 0 && tupleStart != std::string::npos && i > tupleStart) {
                outTuples.push_back(line.substr(tupleStart, i - tupleStart));
                tupleStart = std::string::npos;
            }
        }
    }
    return !outTuples.empty();
}

static std::vector<std::string> splitCsvTuple(const std::string& tuple) {
    std::vector<std::string> cols;
    std::string cur;
    bool inQuote = false;
    for (size_t i = 0; i < tuple.size(); i++) {
        char c = tuple[i];
        if (c == '\'' && (i == 0 || tuple[i - 1] != '\\')) {
            inQuote = !inQuote;
            cur.push_back(c);
            continue;
        }
        if (c == ',' && !inQuote) {
            cols.push_back(trimSql(cur));
            cur.clear();
            continue;
        }
        cur.push_back(c);
    }
    if (!cur.empty()) cols.push_back(trimSql(cur));
    return cols;
}

static std::string unquoteSqlString(const std::string& s) {
    if (s.size() >= 2 && s.front() == '\'' && s.back() == '\'') {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

static std::vector<std::string> loadCreateTableColumns(const std::filesystem::path& path) {
    std::vector<std::string> columns;
    std::ifstream in(path);
    if (!in) return columns;
    std::string line;
    bool inCreate = false;
    while (std::getline(in, line)) {
        if (!inCreate) {
            if (line.find("CREATE TABLE") != std::string::npos ||
                line.find("create table") != std::string::npos) {
                inCreate = true;
            }
            continue;
        }
        auto trimmed = trimSql(line);
        if (trimmed.empty()) continue;
        if (trimmed[0] == ')') break;
        size_t b = trimmed.find('`');
        if (b == std::string::npos) continue;
        size_t e = trimmed.find('`', b + 1);
        if (e == std::string::npos) continue;
        columns.push_back(trimmed.substr(b + 1, e - b - 1));
    }
    return columns;
}

static int columnIndex(const std::vector<std::string>& cols, const std::string& name) {
    for (size_t i = 0; i < cols.size(); i++) {
        if (cols[i] == name) return static_cast<int>(i);
    }
    return -1;
}

static std::filesystem::path resolveDbBasePath() {
    if (const char* dbBase = std::getenv("WOW_DB_BASE_PATH")) {
        std::filesystem::path base(dbBase);
        if (std::filesystem::exists(base)) return base;
    }
    if (std::filesystem::exists("assets/sql")) {
        return std::filesystem::path("assets/sql");
    }
    return {};
}

static void processInsertStatements(
    std::ifstream& in,
    const std::function<void(const std::vector<std::string>&)>& onTuple) {
    std::string line;
    std::string stmt;
    std::vector<std::string> tuples;
    while (std::getline(in, line)) {
        if (stmt.empty()) {
            if (line.find("INSERT INTO") == std::string::npos &&
                line.find("insert into") == std::string::npos) {
                continue;
            }
        }
        if (!stmt.empty()) stmt.push_back('\n');
        stmt += line;
        if (line.find(';') == std::string::npos) continue;

        if (parseInsertTuples(stmt, tuples)) {
            for (const auto& t : tuples) {
                onTuple(splitCsvTuple(t));
            }
        }
        stmt.clear();
    }
}

static SinglePlayerLootDb& getSinglePlayerLootDb() {
    static SinglePlayerLootDb db;
    if (db.loaded) return db;

    auto base = resolveDbBasePath();
    if (base.empty()) {
        db.loaded = true;
        return db;
    }

    std::filesystem::path basePath = base;
    std::filesystem::path creatureTemplatePath = basePath / "creature_template.sql";
    std::filesystem::path creatureLootPath = basePath / "creature_loot_template.sql";
    std::filesystem::path referenceLootPath = basePath / "reference_loot_template.sql";
    std::filesystem::path itemTemplatePath = basePath / "item_template.sql";

    if (!std::filesystem::exists(creatureTemplatePath)) {
        auto alt = basePath / "base";
        if (std::filesystem::exists(alt / "creature_template.sql")) {
            basePath = alt;
            creatureTemplatePath = basePath / "creature_template.sql";
            creatureLootPath = basePath / "creature_loot_template.sql";
            referenceLootPath = basePath / "reference_loot_template.sql";
            itemTemplatePath = basePath / "item_template.sql";
        }
    }

    db.basePath = basePath.string();

    // creature_template: entry, lootid, mingold, maxgold
    {
        auto cols = loadCreateTableColumns(creatureTemplatePath);
        int idxEntry = columnIndex(cols, "entry");
        int idxLoot = columnIndex(cols, "lootid");
        int idxMinGold = columnIndex(cols, "mingold");
        int idxMaxGold = columnIndex(cols, "maxgold");
        if (idxEntry >= 0 && std::filesystem::exists(creatureTemplatePath)) {
            std::ifstream in(creatureTemplatePath);
            processInsertStatements(in, [&](const std::vector<std::string>& row) {
                if (idxEntry >= static_cast<int>(row.size())) return;
                try {
                    uint32_t entry = static_cast<uint32_t>(std::stoul(row[idxEntry]));
                    CreatureTemplateRow tr;
                    if (idxLoot >= 0 && idxLoot < static_cast<int>(row.size())) {
                        tr.lootId = static_cast<uint32_t>(std::stoul(row[idxLoot]));
                    }
                    if (idxMinGold >= 0 && idxMinGold < static_cast<int>(row.size())) {
                        tr.minGold = static_cast<uint32_t>(std::stoul(row[idxMinGold]));
                    }
                    if (idxMaxGold >= 0 && idxMaxGold < static_cast<int>(row.size())) {
                        tr.maxGold = static_cast<uint32_t>(std::stoul(row[idxMaxGold]));
                    }
                    db.creatureTemplates[entry] = tr;
                } catch (const std::exception&) {
                }
            });
        }
    }

    auto loadLootTable = [&](const std::filesystem::path& path,
                             std::unordered_map<uint32_t, std::vector<LootEntryRow>>& out) {
        if (!std::filesystem::exists(path)) return;
        std::ifstream in(path);
        processInsertStatements(in, [&](const std::vector<std::string>& row) {
            if (row.size() < 7) return;
            try {
                uint32_t entry = static_cast<uint32_t>(std::stoul(row[0]));
                LootEntryRow lr;
                lr.item = static_cast<uint32_t>(std::stoul(row[1]));
                lr.chance = std::stof(row[2]);
                lr.lootmode = static_cast<uint16_t>(std::stoul(row[3]));
                lr.groupid = static_cast<uint8_t>(std::stoul(row[4]));
                lr.mincountOrRef = static_cast<int32_t>(std::stol(row[5]));
                lr.maxcount = static_cast<uint8_t>(std::stoul(row[6]));
                out[entry].push_back(lr);
            } catch (const std::exception&) {
            }
        });
    };

    loadLootTable(creatureLootPath, db.creatureLoot);
    loadLootTable(referenceLootPath, db.referenceLoot);

    // item_template
    {
        auto cols = loadCreateTableColumns(itemTemplatePath);
        int idxEntry = columnIndex(cols, "entry");
        int idxName = columnIndex(cols, "name");
        int idxDisplay = columnIndex(cols, "displayid");
        int idxQuality = columnIndex(cols, "Quality");
        int idxInvType = columnIndex(cols, "InventoryType");
        int idxStack = columnIndex(cols, "stackable");
        int idxSellPrice = columnIndex(cols, "SellPrice");
        int idxArmor = columnIndex(cols, "armor");
        // stat_type/stat_value pairs (up to 10)
        int idxStatType[10], idxStatVal[10];
        for (int si = 0; si < 10; si++) {
            idxStatType[si] = columnIndex(cols, "stat_type" + std::to_string(si + 1));
            idxStatVal[si] = columnIndex(cols, "stat_value" + std::to_string(si + 1));
        }
        if (idxEntry >= 0 && std::filesystem::exists(itemTemplatePath)) {
            std::ifstream in(itemTemplatePath);
            processInsertStatements(in, [&](const std::vector<std::string>& row) {
                if (idxEntry >= static_cast<int>(row.size())) return;
                try {
                    ItemTemplateRow ir;
                    ir.itemId = static_cast<uint32_t>(std::stoul(row[idxEntry]));
                    if (idxName >= 0 && idxName < static_cast<int>(row.size())) {
                        ir.name = unquoteSqlString(row[idxName]);
                    }
                    if (idxDisplay >= 0 && idxDisplay < static_cast<int>(row.size())) {
                        ir.displayId = static_cast<uint32_t>(std::stoul(row[idxDisplay]));
                    }
                    if (idxQuality >= 0 && idxQuality < static_cast<int>(row.size())) {
                        ir.quality = static_cast<uint8_t>(std::stoul(row[idxQuality]));
                    }
                    if (idxInvType >= 0 && idxInvType < static_cast<int>(row.size())) {
                        ir.inventoryType = static_cast<uint8_t>(std::stoul(row[idxInvType]));
                    }
                    if (idxStack >= 0 && idxStack < static_cast<int>(row.size())) {
                        ir.maxStack = static_cast<int32_t>(std::stol(row[idxStack]));
                        if (ir.maxStack <= 0) ir.maxStack = 1;
                    }
                    if (idxSellPrice >= 0 && idxSellPrice < static_cast<int>(row.size())) {
                        ir.sellPrice = static_cast<uint32_t>(std::stoul(row[idxSellPrice]));
                    }
                    if (idxArmor >= 0 && idxArmor < static_cast<int>(row.size())) {
                        ir.armor = static_cast<int32_t>(std::stol(row[idxArmor]));
                    }
                    // Parse stat_type/stat_value pairs (protected from parse errors)
                    for (int si = 0; si < 10; si++) {
                        try {
                            if (idxStatType[si] < 0 || idxStatVal[si] < 0) continue;
                            if (idxStatType[si] >= static_cast<int>(row.size())) continue;
                            if (idxStatVal[si] >= static_cast<int>(row.size())) continue;
                            int stype = std::stoi(row[idxStatType[si]]);
                            int sval = std::stoi(row[idxStatVal[si]]);
                            if (sval == 0) continue;
                            switch (stype) {
                                case 3: ir.agility += sval; break;
                                case 4: ir.strength += sval; break;
                                case 5: ir.intellect += sval; break;
                                case 6: ir.spirit += sval; break;
                                case 7: ir.stamina += sval; break;
                            }
                        } catch (...) {}
                    }
                    db.itemTemplates[ir.itemId] = std::move(ir);
                } catch (const std::exception&) {
                }
            });
        }
    }

    db.loaded = true;
    LOG_INFO("Single-player loot DB loaded from ", db.basePath,
             " (creatures=", db.creatureTemplates.size(),
             ", loot=", db.creatureLoot.size(),
             ", reference=", db.referenceLoot.size(),
             ", items=", db.itemTemplates.size(), ")");
    return db;
}

static SinglePlayerCreateDb& getSinglePlayerCreateDb() {
    static SinglePlayerCreateDb db;
    if (db.loaded) return db;

    auto base = resolveDbBasePath();
    if (base.empty()) {
        db.loaded = true;
        return db;
    }

    std::filesystem::path basePath = base;
    std::filesystem::path createInfoPath = basePath / "playercreateinfo.sql";
    if (!std::filesystem::exists(createInfoPath)) {
        auto alt = basePath / "base";
        if (std::filesystem::exists(alt / "playercreateinfo.sql")) {
            basePath = alt;
            createInfoPath = basePath / "playercreateinfo.sql";
        }
    }

    if (!std::filesystem::exists(createInfoPath)) {
        db.loaded = true;
        return db;
    }

    auto cols = loadCreateTableColumns(createInfoPath);
    int idxRace = columnIndex(cols, "race");
    int idxClass = columnIndex(cols, "class");
    int idxMap = columnIndex(cols, "map");
    int idxZone = columnIndex(cols, "zone");
    int idxX = columnIndex(cols, "position_x");
    int idxY = columnIndex(cols, "position_y");
    int idxZ = columnIndex(cols, "position_z");
    int idxO = columnIndex(cols, "orientation");

    std::ifstream in(createInfoPath);
    processInsertStatements(in, [&](const std::vector<std::string>& row) {
        if (idxRace < 0 || idxClass < 0 || idxMap < 0 || idxZone < 0 ||
            idxX < 0 || idxY < 0 || idxZ < 0 || idxO < 0) {
            return;
        }
        if (idxRace >= static_cast<int>(row.size()) || idxClass >= static_cast<int>(row.size())) return;
        try {
            uint32_t race = static_cast<uint32_t>(std::stoul(row[idxRace]));
            uint32_t cls = static_cast<uint32_t>(std::stoul(row[idxClass]));
            GameHandler::SinglePlayerCreateInfo info;
            info.mapId = static_cast<uint32_t>(std::stoul(row[idxMap]));
            info.zoneId = static_cast<uint32_t>(std::stoul(row[idxZone]));
            info.x = std::stof(row[idxX]);
            info.y = std::stof(row[idxY]);
            info.z = std::stof(row[idxZ]);
            info.orientation = std::stof(row[idxO]);
            uint16_t key = static_cast<uint16_t>((race << 8) | cls);
            db.rows[key] = info;
        } catch (const std::exception&) {
        }
    });

    db.loaded = true;
    LOG_INFO("Single-player create DB loaded from ", createInfoPath.string(),
             " (rows=", db.rows.size(), ")");
    return db;
}

static SinglePlayerStartDb& getSinglePlayerStartDb() {
    static SinglePlayerStartDb db;
    if (db.loaded) return db;

    auto base = resolveDbBasePath();
    if (base.empty()) {
        db.loaded = true;
        return db;
    }

    std::filesystem::path basePath = base;
    std::filesystem::path itemPath = basePath / "playercreateinfo_item.sql";
    std::filesystem::path spellPath = basePath / "playercreateinfo_spell.sql";
    std::filesystem::path actionPath = basePath / "playercreateinfo_action.sql";
    if (!std::filesystem::exists(itemPath) || !std::filesystem::exists(spellPath) || !std::filesystem::exists(actionPath)) {
        auto alt = basePath / "base";
        if (std::filesystem::exists(alt / "playercreateinfo_item.sql")) {
            basePath = alt;
            itemPath = basePath / "playercreateinfo_item.sql";
            spellPath = basePath / "playercreateinfo_spell.sql";
            actionPath = basePath / "playercreateinfo_action.sql";
        }
    }

    if (std::filesystem::exists(itemPath)) {
        std::ifstream in(itemPath);
        processInsertStatements(in, [&](const std::vector<std::string>& row) {
            if (row.size() < 4) return;
            try {
                SinglePlayerStartDb::StartItemRow r;
                r.race = static_cast<uint8_t>(std::stoul(row[0]));
                r.cls = static_cast<uint8_t>(std::stoul(row[1]));
                r.itemId = static_cast<uint32_t>(std::stoul(row[2]));
                r.amount = static_cast<int32_t>(std::stol(row[3]));
                db.items.push_back(r);
            } catch (const std::exception&) {
            }
        });
    }

    if (std::filesystem::exists(spellPath)) {
        std::ifstream in(spellPath);
        processInsertStatements(in, [&](const std::vector<std::string>& row) {
            if (row.size() < 3) return;
            try {
                SinglePlayerStartDb::StartSpellRow r;
                r.raceMask = static_cast<uint32_t>(std::stoul(row[0]));
                r.classMask = static_cast<uint32_t>(std::stoul(row[1]));
                r.spellId = static_cast<uint32_t>(std::stoul(row[2]));
                db.spells.push_back(r);
            } catch (const std::exception&) {
            }
        });
    }

    if (std::filesystem::exists(actionPath)) {
        std::ifstream in(actionPath);
        processInsertStatements(in, [&](const std::vector<std::string>& row) {
            if (row.size() < 5) return;
            try {
                SinglePlayerStartDb::StartActionRow r;
                r.race = static_cast<uint8_t>(std::stoul(row[0]));
                r.cls = static_cast<uint8_t>(std::stoul(row[1]));
                r.button = static_cast<uint16_t>(std::stoul(row[2]));
                r.action = static_cast<uint32_t>(std::stoul(row[3]));
                r.type = static_cast<uint16_t>(std::stoul(row[4]));
                db.actions.push_back(r);
            } catch (const std::exception&) {
            }
        });
    }

    db.loaded = true;
    LOG_INFO("Single-player start DB loaded (items=", db.items.size(),
             ", spells=", db.spells.size(), ", actions=", db.actions.size(), ")");
    return db;
}

} // namespace

GameHandler::GameHandler() {
    LOG_DEBUG("GameHandler created");

    // Default spells always available
    knownSpells.push_back(6603);  // Attack
    knownSpells.push_back(8690);  // Hearthstone

    // Default action bar layout
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;   // Attack in slot 1
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone in slot 12
}

GameHandler::~GameHandler() {
    disconnect();
}

bool GameHandler::connect(const std::string& host,
                          uint16_t port,
                          const std::vector<uint8_t>& sessionKey,
                          const std::string& accountName,
                          uint32_t build) {

    if (sessionKey.size() != 40) {
        LOG_ERROR("Invalid session key size: ", sessionKey.size(), " (expected 40)");
        fail("Invalid session key");
        return false;
    }

    LOG_INFO("========================================");
    LOG_INFO("   CONNECTING TO WORLD SERVER");
    LOG_INFO("========================================");
    LOG_INFO("Host: ", host);
    LOG_INFO("Port: ", port);
    LOG_INFO("Account: ", accountName);
    LOG_INFO("Build: ", build);

    // Store authentication data
    this->sessionKey = sessionKey;
    this->accountName = accountName;
    this->build = build;

    // Generate random client seed
    this->clientSeed = generateClientSeed();
    LOG_DEBUG("Generated client seed: 0x", std::hex, clientSeed, std::dec);

    // Create world socket
    socket = std::make_unique<network::WorldSocket>();

    // Set up packet callback
    socket->setPacketCallback([this](const network::Packet& packet) {
        network::Packet mutablePacket = packet;
        handlePacket(mutablePacket);
    });

    // Connect to world server
    setState(WorldState::CONNECTING);

    if (!socket->connect(host, port)) {
        LOG_ERROR("Failed to connect to world server");
        fail("Connection failed");
        return false;
    }

    setState(WorldState::CONNECTED);
    LOG_INFO("Connected to world server, waiting for SMSG_AUTH_CHALLENGE...");

    return true;
}

void GameHandler::disconnect() {
    if (singlePlayerMode_) {
        flushSinglePlayerSave();
    }
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
    activeCharacterGuid_ = 0;
    playerNameCache.clear();
    pendingNameQueries.clear();
    setState(WorldState::DISCONNECTED);
    LOG_INFO("Disconnected from world server");
}

bool GameHandler::isConnected() const {
    return socket && socket->isConnected();
}

void GameHandler::update(float deltaTime) {
    // Fire deferred char-create callback (outside ImGui render)
    if (pendingCharCreateResult_) {
        pendingCharCreateResult_ = false;
        if (charCreateCallback_) {
            charCreateCallback_(pendingCharCreateSuccess_, pendingCharCreateMsg_);
        }
    }

    if (!socket && !singlePlayerMode_) {
        return;
    }

    // Update socket (processes incoming data and triggers callbacks)
    if (socket) {
        socket->update();
    }

    // Validate target still exists
    if (targetGuid != 0 && !entityManager.hasEntity(targetGuid)) {
        clearTarget();
    }

    // Send periodic heartbeat if in world
    if (state == WorldState::IN_WORLD || singlePlayerMode_) {
        timeSinceLastPing += deltaTime;

        if (timeSinceLastPing >= pingInterval) {
            if (socket) {
                sendPing();
            }
            timeSinceLastPing = 0.0f;
        }

        // Update cast timer (Phase 3)
        if (casting && castTimeRemaining > 0.0f) {
            castTimeRemaining -= deltaTime;
            if (castTimeRemaining <= 0.0f) {
                casting = false;
                currentCastSpellId = 0;
                castTimeRemaining = 0.0f;
            }
        }

        // Update spell cooldowns (Phase 3)
        for (auto it = spellCooldowns.begin(); it != spellCooldowns.end(); ) {
            it->second -= deltaTime;
            if (it->second <= 0.0f) {
                it = spellCooldowns.erase(it);
            } else {
                ++it;
            }
        }

        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.cooldownRemaining > 0.0f) {
                slot.cooldownRemaining -= deltaTime;
                if (slot.cooldownRemaining < 0.0f) slot.cooldownRemaining = 0.0f;
            }
        }

        // Update combat text (Phase 2)
        updateCombatText(deltaTime);

        // Update entity movement interpolation (keeps targeting in sync with visuals)
        for (auto& [guid, entity] : entityManager.getEntities()) {
            entity->updateMovement(deltaTime);
        }

        // Single-player local combat
        if (singlePlayerMode_) {
            updateLocalCombat(deltaTime);
            updateNpcAggro(deltaTime);
        }

        // Online mode: maintain auto-attack by periodically re-sending CMSG_ATTACKSWING
        if (!singlePlayerMode_ && autoAttacking && autoAttackTarget != 0 && socket) {
            auto target = entityManager.getEntity(autoAttackTarget);
            if (!target) {
                // Target gone
                stopAutoAttack();
            } else if (target->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(target);
                if (unit->getHealth() == 0) {
                    stopAutoAttack();
                } else {
                    // Out-of-range notice (melee)
                    constexpr float MELEE_RANGE = 5.0f;
                    float dx = target->getX() - movementInfo.x;
                    float dy = target->getY() - movementInfo.y;
                    float dz = target->getZ() - movementInfo.z;
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    bool outOfRange = dist > MELEE_RANGE;
                    if (outOfRange && !autoAttackOutOfRange_) {
                        addSystemChatMessage("Target is out of range.");
                        autoAttackOutOfRange_ = true;
                    } else if (!outOfRange && autoAttackOutOfRange_) {
                        autoAttackOutOfRange_ = false;
                    }

                    // Re-send attack swing every 2 seconds to keep server combat alive
                    swingTimer_ += deltaTime;
                    if (swingTimer_ >= 2.0f) {
                        auto pkt = AttackSwingPacket::build(autoAttackTarget);
                        socket->send(pkt);
                        swingTimer_ = 0.0f;
                    }
                }
            }
        }
    }

    if (singlePlayerMode_) {
        if (spDirtyFlags_ != SP_DIRTY_NONE) {
            spDirtyTimer_ += deltaTime;
            spPeriodicTimer_ += deltaTime;
            bool due = false;
            if (spDirtyHighPriority_ && spDirtyTimer_ >= 0.5f) {
                due = true;
            } else if (spPeriodicTimer_ >= 30.0f) {
                due = true;
            }
            if (due) {
                saveSinglePlayerCharacterState(false);
            }
        }
    }
}

void GameHandler::handlePacket(network::Packet& packet) {
    if (packet.getSize() < 1) {
        LOG_WARNING("Received empty packet");
        return;
    }

    uint16_t opcode = packet.getOpcode();

    LOG_DEBUG("Received world packet: opcode=0x", std::hex, opcode, std::dec,
              " size=", packet.getSize(), " bytes");

    // Route packet based on opcode
    Opcode opcodeEnum = static_cast<Opcode>(opcode);

    switch (opcodeEnum) {
        case Opcode::SMSG_AUTH_CHALLENGE:
            if (state == WorldState::CONNECTED) {
                handleAuthChallenge(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_CHALLENGE in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_AUTH_RESPONSE:
            if (state == WorldState::AUTH_SENT) {
                handleAuthResponse(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_RESPONSE in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_CHAR_CREATE:
            handleCharCreateResponse(packet);
            break;

        case Opcode::SMSG_CHAR_DELETE: {
            uint8_t result = packet.readUInt8();
            lastCharDeleteResult_ = result;
            bool success = (result == 0x00 || result == 0x47); // Common success codes
            LOG_INFO("SMSG_CHAR_DELETE result: ", (int)result, success ? " (success)" : " (failed)");
            requestCharacterList();
            if (charDeleteCallback_) charDeleteCallback_(success);
            break;
        }

        case Opcode::SMSG_CHAR_ENUM:
            if (state == WorldState::CHAR_LIST_REQUESTED) {
                handleCharEnum(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_CHAR_ENUM in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_LOGIN_VERIFY_WORLD:
            if (state == WorldState::ENTERING_WORLD) {
                handleLoginVerifyWorld(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_LOGIN_VERIFY_WORLD in state: ", (int)state);
            }
            break;

        case Opcode::SMSG_ACCOUNT_DATA_TIMES:
            // Can be received at any time after authentication
            handleAccountDataTimes(packet);
            break;

        case Opcode::SMSG_MOTD:
            // Can be received at any time after entering world
            handleMotd(packet);
            break;

        case Opcode::SMSG_PONG:
            // Can be received at any time after entering world
            handlePong(packet);
            break;

        case Opcode::SMSG_UPDATE_OBJECT:
            LOG_INFO("Received SMSG_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleUpdateObject(packet);
            }
            break;

        case Opcode::SMSG_COMPRESSED_UPDATE_OBJECT:
            LOG_INFO("Received SMSG_COMPRESSED_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Compressed version of UPDATE_OBJECT
            if (state == WorldState::IN_WORLD) {
                handleCompressedUpdateObject(packet);
            }
            break;

        case Opcode::SMSG_DESTROY_OBJECT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleDestroyObject(packet);
            }
            break;

        case Opcode::SMSG_MESSAGECHAT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleMessageChat(packet);
            }
            break;

        // ---- Phase 1: Foundation ----
        case Opcode::SMSG_NAME_QUERY_RESPONSE:
            handleNameQueryResponse(packet);
            break;

        case Opcode::SMSG_CREATURE_QUERY_RESPONSE:
            handleCreatureQueryResponse(packet);
            break;

        case Opcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE:
            handleItemQueryResponse(packet);
            break;

        // ---- XP ----
        case Opcode::SMSG_LOG_XPGAIN:
            handleXpGain(packet);
            break;

        // ---- Creature Movement ----
        case Opcode::SMSG_MONSTER_MOVE:
            handleMonsterMove(packet);
            break;

        // ---- Phase 2: Combat ----
        case Opcode::SMSG_ATTACKSTART:
            handleAttackStart(packet);
            break;
        case Opcode::SMSG_ATTACKSTOP:
            handleAttackStop(packet);
            break;
        case Opcode::SMSG_ATTACKERSTATEUPDATE:
            handleAttackerStateUpdate(packet);
            break;
        case Opcode::SMSG_SPELLNONMELEEDAMAGELOG:
            handleSpellDamageLog(packet);
            break;
        case Opcode::SMSG_SPELLHEALLOG:
            handleSpellHealLog(packet);
            break;

        // ---- Phase 3: Spells ----
        case Opcode::SMSG_INITIAL_SPELLS:
            handleInitialSpells(packet);
            break;
        case Opcode::SMSG_CAST_FAILED:
            handleCastFailed(packet);
            break;
        case Opcode::SMSG_SPELL_START:
            handleSpellStart(packet);
            break;
        case Opcode::SMSG_SPELL_GO:
            handleSpellGo(packet);
            break;
        case Opcode::SMSG_SPELL_FAILURE:
            // Spell failed mid-cast
            casting = false;
            currentCastSpellId = 0;
            break;
        case Opcode::SMSG_SPELL_COOLDOWN:
            handleSpellCooldown(packet);
            break;
        case Opcode::SMSG_COOLDOWN_EVENT:
            handleCooldownEvent(packet);
            break;
        case Opcode::SMSG_AURA_UPDATE:
            handleAuraUpdate(packet, false);
            break;
        case Opcode::SMSG_AURA_UPDATE_ALL:
            handleAuraUpdate(packet, true);
            break;
        case Opcode::SMSG_LEARNED_SPELL:
            handleLearnedSpell(packet);
            break;
        case Opcode::SMSG_REMOVED_SPELL:
            handleRemovedSpell(packet);
            break;

        // ---- Phase 4: Group ----
        case Opcode::SMSG_GROUP_INVITE:
            handleGroupInvite(packet);
            break;
        case Opcode::SMSG_GROUP_DECLINE:
            handleGroupDecline(packet);
            break;
        case Opcode::SMSG_GROUP_LIST:
            handleGroupList(packet);
            break;
        case Opcode::SMSG_GROUP_UNINVITE:
            handleGroupUninvite(packet);
            break;
        case Opcode::SMSG_PARTY_COMMAND_RESULT:
            handlePartyCommandResult(packet);
            break;

        // ---- Phase 5: Loot/Gossip/Vendor ----
        case Opcode::SMSG_LOOT_RESPONSE:
            handleLootResponse(packet);
            break;
        case Opcode::SMSG_LOOT_RELEASE_RESPONSE:
            handleLootReleaseResponse(packet);
            break;
        case Opcode::SMSG_LOOT_REMOVED:
            handleLootRemoved(packet);
            break;
        case Opcode::SMSG_GOSSIP_MESSAGE:
            handleGossipMessage(packet);
            break;
        case Opcode::SMSG_GOSSIP_COMPLETE:
            handleGossipComplete(packet);
            break;
        case Opcode::SMSG_LIST_INVENTORY:
            handleListInventory(packet);
            break;

        // Silently ignore common packets we don't handle yet
        case Opcode::SMSG_FEATURE_SYSTEM_STATUS:
        case Opcode::SMSG_SET_FLAT_SPELL_MODIFIER:
        case Opcode::SMSG_SET_PCT_SPELL_MODIFIER:
        case Opcode::SMSG_SPELL_DELAYED:
        case Opcode::SMSG_UPDATE_AURA_DURATION:
        case Opcode::SMSG_PERIODICAURALOG:
        case Opcode::SMSG_SPELLENERGIZELOG:
        case Opcode::SMSG_ENVIRONMENTALDAMAGELOG:
        case Opcode::SMSG_LOOT_MONEY_NOTIFY: {
            // uint32 money + uint8 soleLooter
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t amount = packet.readUInt32();
                playerMoneyCopper_ += amount;
                LOG_INFO("Looted ", amount, " copper (total: ", playerMoneyCopper_, ")");
            }
            break;
        }
        case Opcode::SMSG_LOOT_CLEAR_MONEY:
        case Opcode::SMSG_NPC_TEXT_UPDATE:
            break;
        case Opcode::SMSG_SELL_ITEM: {
            // uint64 vendorGuid, uint64 itemGuid, uint8 result
            if ((packet.getSize() - packet.getReadPos()) >= 17) {
                packet.readUInt64(); // vendorGuid
                packet.readUInt64(); // itemGuid
                uint8_t result = packet.readUInt8();
                if (result != 0) {
                    static const char* sellErrors[] = {
                        "OK", "Can't find item", "Can't sell item",
                        "Can't find vendor", "You don't own that item",
                        "Unknown error", "Only empty bag"
                    };
                    const char* msg = (result < 7) ? sellErrors[result] : "Unknown sell error";
                    addSystemChatMessage(std::string("Sell failed: ") + msg);
                    LOG_WARNING("SMSG_SELL_ITEM error: ", (int)result, " (", msg, ")");
                }
            }
            break;
        }
        case Opcode::SMSG_INVENTORY_CHANGE_FAILURE: {
            if ((packet.getSize() - packet.getReadPos()) >= 1) {
                uint8_t error = packet.readUInt8();
                if (error != 0) {
                    LOG_WARNING("SMSG_INVENTORY_CHANGE_FAILURE: error=", (int)error);
                    // Common error codes
                    std::string msg;
                    switch (error) {
                        case 1: msg = "Item slots occupied."; break;
                        case 4: msg = "Item doesn't go there."; break;
                        case 5: msg = "Bag is full."; break;
                        case 14: msg = "Can't equip that."; break;
                        case 23: msg = "Can't equip with two-handed weapon."; break;
                        case 26: msg = "Inventory full."; break;
                        case 29: msg = "Item is locked."; break;
                        default: msg = "Inventory error (" + std::to_string(error) + ")."; break;
                    }
                    addSystemChatMessage(msg);
                }
            }
            break;
        }
        case Opcode::SMSG_BUY_FAILED:
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
        case Opcode::MSG_RAID_TARGET_UPDATE:
            break;
        case Opcode::SMSG_QUESTGIVER_STATUS: {
            // uint64 npcGuid + uint8 status
            if (packet.getSize() - packet.getReadPos() >= 9) {
                uint64_t npcGuid = packet.readUInt64();
                uint8_t status = packet.readUInt8();
                npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS: guid=0x", std::hex, npcGuid, std::dec, " status=", (int)status);
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_STATUS_MULTIPLE: {
            // uint32 count, then count * (uint64 guid + uint8 status)
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t count = packet.readUInt32();
                for (uint32_t i = 0; i < count; ++i) {
                    if (packet.getSize() - packet.getReadPos() < 9) break;
                    uint64_t npcGuid = packet.readUInt64();
                    uint8_t status = packet.readUInt8();
                    npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                }
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS_MULTIPLE: ", count, " entries");
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_QUEST_DETAILS:
            handleQuestDetails(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_QUEST_COMPLETE: {
            // Mark quest as complete in local log
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t questId = packet.readUInt32();
                for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
                    if (it->questId == questId) {
                        questLog_.erase(it);
                        break;
                    }
                }
            }
            // Re-query all nearby quest giver NPCs so markers refresh
            if (socket) {
                for (const auto& [guid, entity] : entityManager.getEntities()) {
                    if (entity->getType() != ObjectType::UNIT) continue;
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    if (unit->getNpcFlags() & 0x02) {
                        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                        qsPkt.writeUInt64(guid);
                        socket->send(qsPkt);
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_REQUEST_ITEMS:
            handleQuestRequestItems(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_OFFER_REWARD:
            handleQuestOfferReward(packet);
            break;
        case Opcode::SMSG_GROUP_SET_LEADER:
            LOG_DEBUG("Ignoring known opcode: 0x", std::hex, opcode, std::dec);
            break;

        default:
            LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec);
            break;
    }
}

void GameHandler::handleAuthChallenge(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_CHALLENGE");

    AuthChallengeData challenge;
    if (!AuthChallengeParser::parse(packet, challenge)) {
        fail("Failed to parse SMSG_AUTH_CHALLENGE");
        return;
    }

    if (!challenge.isValid()) {
        fail("Invalid auth challenge data");
        return;
    }

    // Store server seed
    serverSeed = challenge.serverSeed;
    LOG_DEBUG("Server seed: 0x", std::hex, serverSeed, std::dec);

    setState(WorldState::CHALLENGE_RECEIVED);

    // Send authentication session
    sendAuthSession();
}

void GameHandler::sendAuthSession() {
    LOG_INFO("Sending CMSG_AUTH_SESSION");

    // Build authentication packet
    auto packet = AuthSessionPacket::build(
        build,
        accountName,
        clientSeed,
        sessionKey,
        serverSeed
    );

    LOG_DEBUG("CMSG_AUTH_SESSION packet size: ", packet.getSize(), " bytes");

    // Send packet (unencrypted - this is the last unencrypted packet)
    socket->send(packet);

    // Enable encryption IMMEDIATELY after sending AUTH_SESSION
    // AzerothCore enables encryption before sending AUTH_RESPONSE,
    // so we need to be ready to decrypt the response
    LOG_INFO("Enabling encryption immediately after AUTH_SESSION");
    socket->initEncryption(sessionKey);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption enabled, waiting for AUTH_RESPONSE...");
}

void GameHandler::handleAuthResponse(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_RESPONSE");

    AuthResponseData response;
    if (!AuthResponseParser::parse(packet, response)) {
        fail("Failed to parse SMSG_AUTH_RESPONSE");
        return;
    }

    if (!response.isSuccess()) {
        std::string reason = std::string("Authentication failed: ") +
                           getAuthResultString(response.result);
        fail(reason);
        return;
    }

    // Encryption was already enabled after sending AUTH_SESSION
    LOG_INFO("AUTH_RESPONSE OK - world authentication successful");

    setState(WorldState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   WORLD AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("Connected to world server");
    LOG_INFO("Ready for character operations");

    setState(WorldState::READY);

    // Request character list automatically
    requestCharacterList();

    // Call success callback
    if (onSuccess) {
        onSuccess();
    }
}

void GameHandler::requestCharacterList() {
    if (singlePlayerMode_) {
        loadSinglePlayerCharacters();
        setState(WorldState::CHAR_LIST_RECEIVED);
        return;
    }
    if (state != WorldState::READY && state != WorldState::AUTHENTICATED &&
        state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot request character list in state: ", (int)state);
        return;
    }

    LOG_INFO("Requesting character list from server...");

    // Build CMSG_CHAR_ENUM packet (no body, just opcode)
    auto packet = CharEnumPacket::build();

    // Send packet
    socket->send(packet);

    setState(WorldState::CHAR_LIST_REQUESTED);
    LOG_INFO("CMSG_CHAR_ENUM sent, waiting for character list...");
}

void GameHandler::handleCharEnum(network::Packet& packet) {
    LOG_INFO("Handling SMSG_CHAR_ENUM");

    CharEnumResponse response;
    if (!CharEnumParser::parse(packet, response)) {
        fail("Failed to parse SMSG_CHAR_ENUM");
        return;
    }

    // Store characters
    characters = response.characters;

    setState(WorldState::CHAR_LIST_RECEIVED);

    LOG_INFO("========================================");
    LOG_INFO("   CHARACTER LIST RECEIVED");
    LOG_INFO("========================================");
    LOG_INFO("Found ", characters.size(), " character(s)");

    if (characters.empty()) {
        LOG_INFO("No characters on this account");
    } else {
        LOG_INFO("Characters:");
        for (size_t i = 0; i < characters.size(); ++i) {
            const auto& character = characters[i];
            LOG_INFO("  [", i + 1, "] ", character.name);
            LOG_INFO("      GUID: 0x", std::hex, character.guid, std::dec);
            LOG_INFO("      ", getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            LOG_INFO("      Level ", (int)character.level);
        }
    }

    LOG_INFO("Ready to select character");
}

void GameHandler::createCharacter(const CharCreateData& data) {
    if (singlePlayerMode_) {
        // Create character locally
        Character ch;
        uint64_t nextGuid = 0x0000000100000001ULL;
        for (const auto& existing : characters) {
            nextGuid = std::max(nextGuid, existing.guid + 1);
        }
        ch.guid = nextGuid;
        ch.name = data.name;
        ch.race = data.race;
        ch.characterClass = data.characterClass;
        ch.gender = data.gender;
        ch.level = 1;
        ch.appearanceBytes = (static_cast<uint32_t>(data.skin)) |
                             (static_cast<uint32_t>(data.face) << 8) |
                             (static_cast<uint32_t>(data.hairStyle) << 16) |
                             (static_cast<uint32_t>(data.hairColor) << 24);
        ch.facialFeatures = data.facialHair;
        SinglePlayerCreateInfo createInfo;
        if (getSinglePlayerCreateInfo(data.race, data.characterClass, createInfo)) {
            ch.zoneId = createInfo.zoneId;
            ch.mapId = createInfo.mapId;
            ch.x = createInfo.x;
            ch.y = createInfo.y;
            ch.z = createInfo.z;
        } else {
            ch.zoneId = 12;   // Elwynn Forest default
            ch.mapId = 0;
            ch.x = -8949.95f;
            ch.y = -132.493f;
            ch.z = 83.5312f;
        }
        ch.guildId = 0;
        ch.flags = 0;
        ch.pet = {};
        characters.push_back(ch);
        spHasState_[ch.guid] = false;
        spSavedOrientation_[ch.guid] = 0.0f;

        // Persist to single-player DB
        auto& sp = getSinglePlayerSqlite();
        if (sp.db) {
            const char* sql =
                "INSERT OR REPLACE INTO characters "
                "(guid, name, race, \"class\", gender, level, appearance_bytes, facial_features, zone, map, "
                "position_x, position_y, position_z, orientation, money, xp, health, max_health, has_state) "
                "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);";
            sqlite3_stmt* stmt = nullptr;
            if (sqlite3_prepare_v2(sp.db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
                sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(ch.guid));
                sqlite3_bind_text(stmt, 2, ch.name.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_int(stmt, 3, static_cast<int>(ch.race));
                sqlite3_bind_int(stmt, 4, static_cast<int>(ch.characterClass));
                sqlite3_bind_int(stmt, 5, static_cast<int>(ch.gender));
                sqlite3_bind_int(stmt, 6, static_cast<int>(ch.level));
                sqlite3_bind_int(stmt, 7, static_cast<int>(ch.appearanceBytes));
                sqlite3_bind_int(stmt, 8, static_cast<int>(ch.facialFeatures));
                sqlite3_bind_int(stmt, 9, static_cast<int>(ch.zoneId));
                sqlite3_bind_int(stmt, 10, static_cast<int>(ch.mapId));
                sqlite3_bind_double(stmt, 11, ch.x);
                sqlite3_bind_double(stmt, 12, ch.y);
                sqlite3_bind_double(stmt, 13, ch.z);
                sqlite3_bind_double(stmt, 14, 0.0);
                sqlite3_bind_int64(stmt, 15, 0);
                sqlite3_bind_int(stmt, 16, 0);
                sqlite3_bind_int(stmt, 17, 0);
                sqlite3_bind_int(stmt, 18, 0);
                sqlite3_bind_int(stmt, 19, 0);
                sqlite3_step(stmt);
                sqlite3_finalize(stmt);
            }
        }

        if (activeCharacterGuid_ == 0) {
            activeCharacterGuid_ = ch.guid;
        }

        LOG_INFO("Single-player character created: ", ch.name);
        // Defer callback to next update() so ImGui frame completes first
        pendingCharCreateResult_ = true;
        pendingCharCreateSuccess_ = true;
        pendingCharCreateMsg_ = "Character created!";
        return;
    }

    // Online mode: send packet to server
    if (!socket) {
        LOG_WARNING("Cannot create character: not connected");
        if (charCreateCallback_) {
            charCreateCallback_(false, "Not connected to server");
        }
        return;
    }

    auto packet = CharCreatePacket::build(data);
    socket->send(packet);
    LOG_INFO("CMSG_CHAR_CREATE sent for: ", data.name);
}

void GameHandler::handleCharCreateResponse(network::Packet& packet) {
    CharCreateResponseData data;
    if (!CharCreateResponseParser::parse(packet, data)) {
        LOG_ERROR("Failed to parse SMSG_CHAR_CREATE");
        return;
    }

    if (data.result == CharCreateResult::SUCCESS) {
        LOG_INFO("Character created successfully");
        requestCharacterList();
        if (charCreateCallback_) {
            charCreateCallback_(true, "Character created!");
        }
    } else {
        std::string msg;
        switch (data.result) {
            case CharCreateResult::ERROR: msg = "Server error"; break;
            case CharCreateResult::FAILED: msg = "Creation failed"; break;
            case CharCreateResult::NAME_IN_USE: msg = "Name already in use"; break;
            case CharCreateResult::DISABLED: msg = "Character creation disabled"; break;
            case CharCreateResult::PVP_TEAMS_VIOLATION: msg = "PvP faction violation"; break;
            case CharCreateResult::SERVER_LIMIT: msg = "Server character limit reached"; break;
            case CharCreateResult::ACCOUNT_LIMIT: msg = "Account character limit reached"; break;
            case CharCreateResult::SERVER_QUEUE: msg = "Server is queued"; break;
            case CharCreateResult::ONLY_EXISTING: msg = "Only existing characters allowed"; break;
            case CharCreateResult::EXPANSION: msg = "Expansion required"; break;
            case CharCreateResult::EXPANSION_CLASS: msg = "Expansion required for this class"; break;
            case CharCreateResult::LEVEL_REQUIREMENT: msg = "Level requirement not met"; break;
            case CharCreateResult::UNIQUE_CLASS_LIMIT: msg = "Unique class limit reached"; break;
            case CharCreateResult::RESTRICTED_RACECLASS: msg = "Race/class combination not allowed"; break;
            // Name validation errors
            case CharCreateResult::NAME_FAILURE: msg = "Invalid name"; break;
            case CharCreateResult::NAME_NO_NAME: msg = "Please enter a name"; break;
            case CharCreateResult::NAME_TOO_SHORT: msg = "Name is too short"; break;
            case CharCreateResult::NAME_TOO_LONG: msg = "Name is too long"; break;
            case CharCreateResult::NAME_INVALID_CHARACTER: msg = "Name contains invalid characters"; break;
            case CharCreateResult::NAME_MIXED_LANGUAGES: msg = "Name mixes languages"; break;
            case CharCreateResult::NAME_PROFANE: msg = "Name contains profanity"; break;
            case CharCreateResult::NAME_RESERVED: msg = "Name is reserved"; break;
            case CharCreateResult::NAME_INVALID_APOSTROPHE: msg = "Invalid apostrophe in name"; break;
            case CharCreateResult::NAME_MULTIPLE_APOSTROPHES: msg = "Name has multiple apostrophes"; break;
            case CharCreateResult::NAME_THREE_CONSECUTIVE: msg = "Name has 3+ consecutive same letters"; break;
            case CharCreateResult::NAME_INVALID_SPACE: msg = "Invalid space in name"; break;
            case CharCreateResult::NAME_CONSECUTIVE_SPACES: msg = "Name has consecutive spaces"; break;
            default: msg = "Unknown error (code " + std::to_string(static_cast<int>(data.result)) + ")"; break;
        }
        LOG_WARNING("Character creation failed: ", msg, " (code=", static_cast<int>(data.result), ")");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
    }
}

void GameHandler::deleteCharacter(uint64_t characterGuid) {
    if (singlePlayerMode_) {
        // Remove from local list
        characters.erase(
            std::remove_if(characters.begin(), characters.end(),
                           [characterGuid](const Character& c) { return c.guid == characterGuid; }),
            characters.end());
        // Remove from database
        auto& sp = getSinglePlayerSqlite();
        if (sp.db) {
            const char* sql = "DELETE FROM characters WHERE guid=?";
            sqlite3_stmt* stmt = nullptr;
            if (sqlite3_prepare_v2(sp.db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
                sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(characterGuid));
                sqlite3_step(stmt);
                sqlite3_finalize(stmt);
            }
            const char* sql2 = "DELETE FROM character_inventory WHERE guid=?";
            if (sqlite3_prepare_v2(sp.db, sql2, -1, &stmt, nullptr) == SQLITE_OK) {
                sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(characterGuid));
                sqlite3_step(stmt);
                sqlite3_finalize(stmt);
            }
        }
        if (charDeleteCallback_) charDeleteCallback_(true);
        return;
    }

    if (!socket) {
        if (charDeleteCallback_) charDeleteCallback_(false);
        return;
    }

    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CHAR_DELETE));
    packet.writeUInt64(characterGuid);
    socket->send(packet);
    LOG_INFO("CMSG_CHAR_DELETE sent for GUID: 0x", std::hex, characterGuid, std::dec);
}

const Character* GameHandler::getActiveCharacter() const {
    if (activeCharacterGuid_ == 0) return nullptr;
    for (const auto& ch : characters) {
        if (ch.guid == activeCharacterGuid_) return &ch;
    }
    return nullptr;
}

const Character* GameHandler::getFirstCharacter() const {
    if (characters.empty()) return nullptr;
    return &characters.front();
}

void GameHandler::setSinglePlayerCharListReady() {
    loadSinglePlayerCharacters();
    setState(WorldState::CHAR_LIST_RECEIVED);
}

bool GameHandler::getSinglePlayerSettings(SinglePlayerSettings& out) const {
    if (!singlePlayerMode_ || !spSettingsLoaded_) return false;
    out = spSettings_;
    return true;
}

void GameHandler::setSinglePlayerSettings(const SinglePlayerSettings& settings) {
    if (!singlePlayerMode_) return;
    spSettings_ = settings;
    spSettingsLoaded_ = true;
    markSinglePlayerDirty(SP_DIRTY_SETTINGS, true);
}

bool GameHandler::getSinglePlayerCreateInfo(Race race, Class cls, SinglePlayerCreateInfo& out) const {
    auto& db = getSinglePlayerCreateDb();
    uint16_t key = static_cast<uint16_t>((static_cast<uint32_t>(race) << 8) |
                                         static_cast<uint32_t>(cls));
    auto it = db.rows.find(key);
    if (it == db.rows.end()) return false;
    out = it->second;
    return true;
}

void GameHandler::notifyInventoryChanged() {
    markSinglePlayerDirty(SP_DIRTY_INVENTORY, true);
}

void GameHandler::notifyEquipmentChanged() {
    markSinglePlayerDirty(SP_DIRTY_INVENTORY, true);
    markSinglePlayerDirty(SP_DIRTY_STATS, true);
}

void GameHandler::notifyQuestStateChanged() {
    markSinglePlayerDirty(SP_DIRTY_QUESTS, true);
}

void GameHandler::markSinglePlayerDirty(uint32_t flags, bool highPriority) {
    if (!singlePlayerMode_) return;
    spDirtyFlags_ |= flags;
    if (highPriority) {
        spDirtyHighPriority_ = true;
        spDirtyTimer_ = 0.0f;
    }
}

void GameHandler::loadSinglePlayerCharacters() {
    if (!singlePlayerMode_) return;
    auto& sp = getSinglePlayerSqlite();
    if (!sp.db) return;

    characters.clear();
    spHasState_.clear();
    spSavedOrientation_.clear();

    const char* sql =
        "SELECT guid, name, race, \"class\", gender, level, appearance_bytes, facial_features, "
        "zone, map, position_x, position_y, position_z, orientation, has_state "
        "FROM characters ORDER BY guid;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(sp.db, sql, -1, &stmt, nullptr) != SQLITE_OK) return;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Character ch;
        ch.guid = static_cast<uint64_t>(sqlite3_column_int64(stmt, 0));
        const unsigned char* nameText = sqlite3_column_text(stmt, 1);
        ch.name = nameText ? reinterpret_cast<const char*>(nameText) : "";
        ch.race = static_cast<Race>(sqlite3_column_int(stmt, 2));
        ch.characterClass = static_cast<Class>(sqlite3_column_int(stmt, 3));
        ch.gender = static_cast<Gender>(sqlite3_column_int(stmt, 4));
        ch.level = static_cast<uint8_t>(sqlite3_column_int(stmt, 5));
        ch.appearanceBytes = static_cast<uint32_t>(sqlite3_column_int(stmt, 6));
        ch.facialFeatures = static_cast<uint8_t>(sqlite3_column_int(stmt, 7));
        ch.zoneId = static_cast<uint32_t>(sqlite3_column_int(stmt, 8));
        ch.mapId = static_cast<uint32_t>(sqlite3_column_int(stmt, 9));
        ch.x = static_cast<float>(sqlite3_column_double(stmt, 10));
        ch.y = static_cast<float>(sqlite3_column_double(stmt, 11));
        ch.z = static_cast<float>(sqlite3_column_double(stmt, 12));
        float orientation = static_cast<float>(sqlite3_column_double(stmt, 13));
        int hasState = sqlite3_column_int(stmt, 14);

        characters.push_back(ch);
        spHasState_[ch.guid] = (hasState != 0);
        spSavedOrientation_[ch.guid] = orientation;
    }
    sqlite3_finalize(stmt);
}

bool GameHandler::loadSinglePlayerCharacterState(uint64_t guid) {
    if (!singlePlayerMode_) return false;
    auto& sp = getSinglePlayerSqlite();
    if (!sp.db) return false;

    spSettingsLoaded_ = false;

    const char* sqlChar =
        "SELECT level, zone, map, position_x, position_y, position_z, orientation, money, xp, health, max_health, has_state "
        "FROM characters WHERE guid=?;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(sp.db, sqlChar, -1, &stmt, nullptr) != SQLITE_OK) return false;
    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
    if (sqlite3_step(stmt) != SQLITE_ROW) {
        sqlite3_finalize(stmt);
        return false;
    }

    uint32_t level = static_cast<uint32_t>(sqlite3_column_int(stmt, 0));
    uint32_t zone = static_cast<uint32_t>(sqlite3_column_int(stmt, 1));
    uint32_t map = static_cast<uint32_t>(sqlite3_column_int(stmt, 2));
    float posX = static_cast<float>(sqlite3_column_double(stmt, 3));
    float posY = static_cast<float>(sqlite3_column_double(stmt, 4));
    float posZ = static_cast<float>(sqlite3_column_double(stmt, 5));
    float orientation = static_cast<float>(sqlite3_column_double(stmt, 6));
    uint64_t money = static_cast<uint64_t>(sqlite3_column_int64(stmt, 7));
    uint32_t xp = static_cast<uint32_t>(sqlite3_column_int(stmt, 8));
    uint32_t health = static_cast<uint32_t>(sqlite3_column_int(stmt, 9));
    uint32_t maxHealth = static_cast<uint32_t>(sqlite3_column_int(stmt, 10));
    bool hasState = sqlite3_column_int(stmt, 11) != 0;
    sqlite3_finalize(stmt);

    spHasState_[guid] = hasState;
    spSavedOrientation_[guid] = orientation;
    if (!hasState) return false;

    // Update movementInfo so startSinglePlayer can use it for spawning
    movementInfo.x = posX;
    movementInfo.y = posY;
    movementInfo.z = posZ;
    movementInfo.orientation = orientation;

    // Update character list entry
    for (auto& ch : characters) {
        if (ch.guid == guid) {
            ch.level = static_cast<uint8_t>(std::max<uint32_t>(1, level));
            ch.zoneId = zone;
            ch.mapId = map;
            ch.x = posX;
            ch.y = posY;
            ch.z = posZ;
            break;
        }
    }

    // Load inventory
    inventory = Inventory();
    const char* sqlInv =
        "SELECT location, slot, item_id, name, quality, inventory_type, stack_count, max_stack, bag_slots, "
        "armor, stamina, strength, agility, intellect, spirit, display_info_id, subclass_name, "
        "COALESCE(sell_price, 0) "
        "FROM character_inventory WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, sqlInv, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int location = sqlite3_column_int(stmt, 0);
            int slot = sqlite3_column_int(stmt, 1);
            ItemDef def;
            def.itemId = static_cast<uint32_t>(sqlite3_column_int(stmt, 2));
            const unsigned char* nameText = sqlite3_column_text(stmt, 3);
            def.name = nameText ? reinterpret_cast<const char*>(nameText) : "";
            def.quality = static_cast<ItemQuality>(sqlite3_column_int(stmt, 4));
            def.inventoryType = static_cast<uint8_t>(sqlite3_column_int(stmt, 5));
            def.stackCount = static_cast<uint32_t>(sqlite3_column_int(stmt, 6));
            def.maxStack = static_cast<uint32_t>(sqlite3_column_int(stmt, 7));
            def.bagSlots = static_cast<uint32_t>(sqlite3_column_int(stmt, 8));
            def.armor = static_cast<int32_t>(sqlite3_column_int(stmt, 9));
            def.stamina = static_cast<int32_t>(sqlite3_column_int(stmt, 10));
            def.strength = static_cast<int32_t>(sqlite3_column_int(stmt, 11));
            def.agility = static_cast<int32_t>(sqlite3_column_int(stmt, 12));
            def.intellect = static_cast<int32_t>(sqlite3_column_int(stmt, 13));
            def.spirit = static_cast<int32_t>(sqlite3_column_int(stmt, 14));
            def.displayInfoId = static_cast<uint32_t>(sqlite3_column_int(stmt, 15));
            const unsigned char* subclassText = sqlite3_column_text(stmt, 16);
            def.subclassName = subclassText ? reinterpret_cast<const char*>(subclassText) : "";
            def.sellPrice = static_cast<uint32_t>(sqlite3_column_int(stmt, 17));

            // Fill missing data from item template DB (for old saves)
            if (def.itemId != 0) {
                auto& itemDb = getSinglePlayerLootDb().itemTemplates;
                auto itTpl = itemDb.find(def.itemId);
                if (itTpl != itemDb.end()) {
                    if (def.sellPrice == 0) def.sellPrice = itTpl->second.sellPrice;
                    if (def.displayInfoId == 0) def.displayInfoId = itTpl->second.displayId;
                    if (def.armor == 0) def.armor = itTpl->second.armor;
                    if (def.stamina == 0) def.stamina = itTpl->second.stamina;
                    if (def.strength == 0) def.strength = itTpl->second.strength;
                    if (def.agility == 0) def.agility = itTpl->second.agility;
                    if (def.intellect == 0) def.intellect = itTpl->second.intellect;
                    if (def.spirit == 0) def.spirit = itTpl->second.spirit;
                }
            }

            if (location == 0) {
                inventory.setBackpackSlot(slot, def);
            } else if (location == 1) {
                inventory.setEquipSlot(static_cast<EquipSlot>(slot), def);
            } else if (location == 2) {
                int bagIndex = slot / Inventory::MAX_BAG_SIZE;
                int bagSlot = slot % Inventory::MAX_BAG_SIZE;
                inventory.setBagSlot(bagIndex, bagSlot, def);
            }
        }
        sqlite3_finalize(stmt);
    }

    // Load spells
    knownSpells.clear();
    const char* sqlSpell = "SELECT spell FROM character_spell WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, sqlSpell, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            uint32_t spellId = static_cast<uint32_t>(sqlite3_column_int(stmt, 0));
            if (spellId != 0) knownSpells.push_back(spellId);
        }
        sqlite3_finalize(stmt);
    }
    if (std::find(knownSpells.begin(), knownSpells.end(), 6603) == knownSpells.end()) {
        knownSpells.push_back(6603);
    }
    if (std::find(knownSpells.begin(), knownSpells.end(), 8690) == knownSpells.end()) {
        knownSpells.push_back(8690);
    }

    // Load action bar
    for (auto& slot : actionBar) slot = ActionBarSlot{};
    bool hasActionRows = false;
    const char* sqlAction = "SELECT slot, type, action FROM character_action WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, sqlAction, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int slot = sqlite3_column_int(stmt, 0);
            if (slot < 0 || slot >= static_cast<int>(actionBar.size())) continue;
            actionBar[slot].type = static_cast<ActionBarSlot::Type>(sqlite3_column_int(stmt, 1));
            actionBar[slot].id = static_cast<uint32_t>(sqlite3_column_int(stmt, 2));
            hasActionRows = true;
        }
        sqlite3_finalize(stmt);
    }
    if (!hasActionRows) {
        actionBar[0].type = ActionBarSlot::SPELL;
        actionBar[0].id = 6603;
        actionBar[11].type = ActionBarSlot::SPELL;
        actionBar[11].id = 8690;
        int slot = 1;
        for (uint32_t spellId : knownSpells) {
            if (spellId == 6603 || spellId == 8690) continue;
            if (slot >= 11) break;
            actionBar[slot].type = ActionBarSlot::SPELL;
            actionBar[slot].id = spellId;
            slot++;
        }
    }

    // Load auras
    playerAuras.clear();
    const char* sqlAura =
        "SELECT slot, spell, flags, level, charges, duration_ms, max_duration_ms, caster_guid "
        "FROM character_aura WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, sqlAura, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            uint8_t slot = static_cast<uint8_t>(sqlite3_column_int(stmt, 0));
            AuraSlot aura;
            aura.spellId = static_cast<uint32_t>(sqlite3_column_int(stmt, 1));
            aura.flags = static_cast<uint8_t>(sqlite3_column_int(stmt, 2));
            aura.level = static_cast<uint8_t>(sqlite3_column_int(stmt, 3));
            aura.charges = static_cast<uint8_t>(sqlite3_column_int(stmt, 4));
            aura.durationMs = static_cast<int32_t>(sqlite3_column_int(stmt, 5));
            aura.maxDurationMs = static_cast<int32_t>(sqlite3_column_int(stmt, 6));
            aura.casterGuid = static_cast<uint64_t>(sqlite3_column_int64(stmt, 7));
            while (playerAuras.size() <= slot) playerAuras.push_back(AuraSlot{});
            playerAuras[slot] = aura;
        }
        sqlite3_finalize(stmt);
    }

    // Apply money, xp, stats
    playerMoneyCopper_ = money;
    playerXp_ = xp;
    localPlayerLevel_ = std::max<uint32_t>(1, level);
    localPlayerHealth_ = std::max<uint32_t>(1, health);
    localPlayerMaxHealth_ = std::max<uint32_t>(localPlayerHealth_, maxHealth);
    playerNextLevelXp_ = xpForLevel(localPlayerLevel_);

    // Seed movement info for spawn (canonical coords in DB)
    movementInfo.x = posX;
    movementInfo.y = posY;
    movementInfo.z = posZ;
    movementInfo.orientation = orientation;

    spLastDirtyX_ = movementInfo.x;
    spLastDirtyY_ = movementInfo.y;
    spLastDirtyZ_ = movementInfo.z;
    spLastDirtyOrientation_ = movementInfo.orientation;

    const char* sqlSettings =
        "SELECT fullscreen, vsync, shadows, res_w, res_h, music_volume, sfx_volume, mouse_sensitivity, invert_mouse "
        "FROM character_settings WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, sqlSettings, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(guid));
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            spSettings_.fullscreen = sqlite3_column_int(stmt, 0) != 0;
            spSettings_.vsync = sqlite3_column_int(stmt, 1) != 0;
            spSettings_.shadows = sqlite3_column_int(stmt, 2) != 0;
            spSettings_.resWidth = sqlite3_column_int(stmt, 3);
            spSettings_.resHeight = sqlite3_column_int(stmt, 4);
            spSettings_.musicVolume = sqlite3_column_int(stmt, 5);
            spSettings_.sfxVolume = sqlite3_column_int(stmt, 6);
            spSettings_.mouseSensitivity = static_cast<float>(sqlite3_column_double(stmt, 7));
            spSettings_.invertMouse = sqlite3_column_int(stmt, 8) != 0;
            spSettingsLoaded_ = true;
        }
        sqlite3_finalize(stmt);
    }

    return true;
}

void GameHandler::applySinglePlayerStartData(Race race, Class cls) {
    inventory = Inventory();
    knownSpells.clear();
    knownSpells.push_back(6603);  // Attack
    knownSpells.push_back(8690);  // Hearthstone

    for (auto& slot : actionBar) {
        slot = ActionBarSlot{};
    }
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;

    auto& startDb = getSinglePlayerStartDb();
    auto& itemDb = getSinglePlayerLootDb().itemTemplates;

    uint8_t raceVal = static_cast<uint8_t>(race);
    uint8_t classVal = static_cast<uint8_t>(cls);
    bool addedItem = false;

    for (const auto& row : startDb.items) {
        if (row.itemId == 0 || row.amount == 0) continue;
        if (row.race != 0 && row.race != raceVal) continue;
        if (row.cls != 0 && row.cls != classVal) continue;
        if (row.amount < 0) continue;

        ItemDef def;
        def.itemId = row.itemId;
        def.stackCount = static_cast<uint32_t>(row.amount);
        def.maxStack = def.stackCount;

        auto itTpl = itemDb.find(row.itemId);
        if (itTpl != itemDb.end()) {
            def.name = itTpl->second.name.empty()
                ? ("Item " + std::to_string(row.itemId))
                : itTpl->second.name;
            def.quality = static_cast<ItemQuality>(itTpl->second.quality);
            def.inventoryType = itTpl->second.inventoryType;
            def.maxStack = std::max(def.maxStack, static_cast<uint32_t>(itTpl->second.maxStack));
            def.sellPrice = itTpl->second.sellPrice;
            def.displayInfoId = itTpl->second.displayId;
            def.armor = itTpl->second.armor;
            def.stamina = itTpl->second.stamina;
            def.strength = itTpl->second.strength;
            def.agility = itTpl->second.agility;
            def.intellect = itTpl->second.intellect;
            def.spirit = itTpl->second.spirit;
        } else {
            def.name = "Item " + std::to_string(row.itemId);
        }

        if (inventory.addItem(def)) {
            addedItem = true;
        }
    }

    for (const auto& row : startDb.items) {
        if (row.itemId == 0 || row.amount >= 0) continue;
        if (row.race != 0 && row.race != raceVal) continue;
        if (row.cls != 0 && row.cls != classVal) continue;
        removeItemsFromInventory(inventory, row.itemId, static_cast<uint32_t>(-row.amount));
    }

    if (!addedItem && startDb.items.empty()) {
        addSystemChatMessage("No starting items found in playercreateinfo_item.sql.");
    }

    uint32_t raceMask = 1u << (raceVal > 0 ? (raceVal - 1) : 0);
    uint32_t classMask = 1u << (classVal > 0 ? (classVal - 1) : 0);
    for (const auto& row : startDb.spells) {
        if (row.spellId == 0) continue;
        if (row.raceMask != 0 && (row.raceMask & raceMask) == 0) continue;
        if (row.classMask != 0 && (row.classMask & classMask) == 0) continue;
        if (std::find(knownSpells.begin(), knownSpells.end(), row.spellId) == knownSpells.end()) {
            knownSpells.push_back(row.spellId);
        }
    }

    bool hasActionRows = false;
    for (const auto& row : startDb.actions) {
        if (row.button >= actionBar.size()) continue;
        if (row.race != 0 && row.race != raceVal) continue;
        if (row.cls != 0 && row.cls != classVal) continue;

        ActionBarSlot::Type type = ActionBarSlot::EMPTY;
        switch (row.type) {
            case 0: type = ActionBarSlot::SPELL; break;
            case 1: type = ActionBarSlot::ITEM; break;
            case 2: type = ActionBarSlot::MACRO; break;
            default: break;
        }
        if (type == ActionBarSlot::EMPTY || row.action == 0) continue;

        actionBar[row.button].type = type;
        actionBar[row.button].id = row.action;
        hasActionRows = true;

        if (type == ActionBarSlot::SPELL &&
            std::find(knownSpells.begin(), knownSpells.end(), row.action) == knownSpells.end()) {
            knownSpells.push_back(row.action);
        }
    }

    if (!hasActionRows) {
        // Leave slots 1-10 empty; player assigns from spellbook
    }

    markSinglePlayerDirty(SP_DIRTY_INVENTORY | SP_DIRTY_SPELLS | SP_DIRTY_ACTIONBAR |
                          SP_DIRTY_STATS | SP_DIRTY_XP | SP_DIRTY_MONEY, true);
}

void GameHandler::saveSinglePlayerCharacterState(bool force) {
    if (!singlePlayerMode_) return;
    if (activeCharacterGuid_ == 0) return;
    if (!force && spDirtyFlags_ == SP_DIRTY_NONE) return;

    auto& sp = getSinglePlayerSqlite();
    if (!sp.db) return;

    const Character* active = getActiveCharacter();
    if (!active) return;

    sqlite3_exec(sp.db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

    const char* updateCharSql =
        "UPDATE characters SET level=?, zone=?, map=?, position_x=?, position_y=?, position_z=?, orientation=?, "
        "money=?, xp=?, health=?, max_health=?, has_state=1 WHERE guid=?;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(sp.db, updateCharSql, -1, &stmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int(stmt, 1, static_cast<int>(localPlayerLevel_));
        sqlite3_bind_int(stmt, 2, static_cast<int>(active->zoneId));
        sqlite3_bind_int(stmt, 3, static_cast<int>(active->mapId));
        sqlite3_bind_double(stmt, 4, movementInfo.x);
        sqlite3_bind_double(stmt, 5, movementInfo.y);
        sqlite3_bind_double(stmt, 6, movementInfo.z);
        sqlite3_bind_double(stmt, 7, movementInfo.orientation);
        sqlite3_bind_int64(stmt, 8, static_cast<sqlite3_int64>(playerMoneyCopper_));
        sqlite3_bind_int(stmt, 9, static_cast<int>(playerXp_));
        sqlite3_bind_int(stmt, 10, static_cast<int>(localPlayerHealth_));
        sqlite3_bind_int(stmt, 11, static_cast<int>(localPlayerMaxHealth_));
        sqlite3_bind_int64(stmt, 12, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }

    spHasState_[activeCharacterGuid_] = true;
    spSavedOrientation_[activeCharacterGuid_] = movementInfo.orientation;

    if (spSettingsLoaded_ && (force || (spDirtyFlags_ & SP_DIRTY_SETTINGS))) {
        const char* upsertSettings =
            "INSERT INTO character_settings "
            "(guid, fullscreen, vsync, shadows, res_w, res_h, music_volume, sfx_volume, mouse_sensitivity, invert_mouse) "
            "VALUES (?,?,?,?,?,?,?,?,?,?) "
            "ON CONFLICT(guid) DO UPDATE SET "
            "fullscreen=excluded.fullscreen, vsync=excluded.vsync, shadows=excluded.shadows, "
            "res_w=excluded.res_w, res_h=excluded.res_h, music_volume=excluded.music_volume, "
            "sfx_volume=excluded.sfx_volume, mouse_sensitivity=excluded.mouse_sensitivity, "
            "invert_mouse=excluded.invert_mouse;";
        if (sqlite3_prepare_v2(sp.db, upsertSettings, -1, &stmt, nullptr) == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, spSettings_.fullscreen ? 1 : 0);
            sqlite3_bind_int(stmt, 3, spSettings_.vsync ? 1 : 0);
            sqlite3_bind_int(stmt, 4, spSettings_.shadows ? 1 : 0);
            sqlite3_bind_int(stmt, 5, spSettings_.resWidth);
            sqlite3_bind_int(stmt, 6, spSettings_.resHeight);
            sqlite3_bind_int(stmt, 7, spSettings_.musicVolume);
            sqlite3_bind_int(stmt, 8, spSettings_.sfxVolume);
            sqlite3_bind_double(stmt, 9, spSettings_.mouseSensitivity);
            sqlite3_bind_int(stmt, 10, spSettings_.invertMouse ? 1 : 0);
            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }
    }

    sqlite3_stmt* del = nullptr;
    const char* delInv = "DELETE FROM character_inventory WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, delInv, -1, &del, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(del, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(del);
        sqlite3_finalize(del);
    }
    const char* delSpell = "DELETE FROM character_spell WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, delSpell, -1, &del, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(del, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(del);
        sqlite3_finalize(del);
    }
    const char* delAction = "DELETE FROM character_action WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, delAction, -1, &del, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(del, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(del);
        sqlite3_finalize(del);
    }
    const char* delAura = "DELETE FROM character_aura WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, delAura, -1, &del, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(del, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(del);
        sqlite3_finalize(del);
    }
    const char* delQuest = "DELETE FROM character_queststatus WHERE guid=?;";
    if (sqlite3_prepare_v2(sp.db, delQuest, -1, &del, nullptr) == SQLITE_OK) {
        sqlite3_bind_int64(del, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
        sqlite3_step(del);
        sqlite3_finalize(del);
    }

    const char* insInv =
        "INSERT INTO character_inventory "
        "(guid, location, slot, item_id, name, quality, inventory_type, stack_count, max_stack, bag_slots, "
        "armor, stamina, strength, agility, intellect, spirit, display_info_id, subclass_name, sell_price) "
        "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);";
    if (sqlite3_prepare_v2(sp.db, insInv, -1, &stmt, nullptr) == SQLITE_OK) {
        for (int i = 0; i < Inventory::BACKPACK_SLOTS; i++) {
            const ItemSlot& slot = inventory.getBackpackSlot(i);
            if (slot.empty()) continue;
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, 0);
            sqlite3_bind_int(stmt, 3, i);
            sqlite3_bind_int(stmt, 4, static_cast<int>(slot.item.itemId));
            sqlite3_bind_text(stmt, 5, slot.item.name.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int(stmt, 6, static_cast<int>(slot.item.quality));
            sqlite3_bind_int(stmt, 7, static_cast<int>(slot.item.inventoryType));
            sqlite3_bind_int(stmt, 8, static_cast<int>(slot.item.stackCount));
            sqlite3_bind_int(stmt, 9, static_cast<int>(slot.item.maxStack));
            sqlite3_bind_int(stmt, 10, static_cast<int>(slot.item.bagSlots));
            sqlite3_bind_int(stmt, 11, static_cast<int>(slot.item.armor));
            sqlite3_bind_int(stmt, 12, static_cast<int>(slot.item.stamina));
            sqlite3_bind_int(stmt, 13, static_cast<int>(slot.item.strength));
            sqlite3_bind_int(stmt, 14, static_cast<int>(slot.item.agility));
            sqlite3_bind_int(stmt, 15, static_cast<int>(slot.item.intellect));
            sqlite3_bind_int(stmt, 16, static_cast<int>(slot.item.spirit));
            sqlite3_bind_int(stmt, 17, static_cast<int>(slot.item.displayInfoId));
            sqlite3_bind_text(stmt, 18, slot.item.subclassName.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int(stmt, 19, static_cast<int>(slot.item.sellPrice));
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        for (int i = 0; i < Inventory::NUM_EQUIP_SLOTS; i++) {
            EquipSlot eq = static_cast<EquipSlot>(i);
            const ItemSlot& slot = inventory.getEquipSlot(eq);
            if (slot.empty()) continue;
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, 1);
            sqlite3_bind_int(stmt, 3, i);
            sqlite3_bind_int(stmt, 4, static_cast<int>(slot.item.itemId));
            sqlite3_bind_text(stmt, 5, slot.item.name.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int(stmt, 6, static_cast<int>(slot.item.quality));
            sqlite3_bind_int(stmt, 7, static_cast<int>(slot.item.inventoryType));
            sqlite3_bind_int(stmt, 8, static_cast<int>(slot.item.stackCount));
            sqlite3_bind_int(stmt, 9, static_cast<int>(slot.item.maxStack));
            sqlite3_bind_int(stmt, 10, static_cast<int>(slot.item.bagSlots));
            sqlite3_bind_int(stmt, 11, static_cast<int>(slot.item.armor));
            sqlite3_bind_int(stmt, 12, static_cast<int>(slot.item.stamina));
            sqlite3_bind_int(stmt, 13, static_cast<int>(slot.item.strength));
            sqlite3_bind_int(stmt, 14, static_cast<int>(slot.item.agility));
            sqlite3_bind_int(stmt, 15, static_cast<int>(slot.item.intellect));
            sqlite3_bind_int(stmt, 16, static_cast<int>(slot.item.spirit));
            sqlite3_bind_int(stmt, 17, static_cast<int>(slot.item.displayInfoId));
            sqlite3_bind_text(stmt, 18, slot.item.subclassName.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int(stmt, 19, static_cast<int>(slot.item.sellPrice));
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        sqlite3_finalize(stmt);
    }

    const char* insSpell = "INSERT INTO character_spell (guid, spell) VALUES (?,?);";
    if (sqlite3_prepare_v2(sp.db, insSpell, -1, &stmt, nullptr) == SQLITE_OK) {
        for (uint32_t spellId : knownSpells) {
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, static_cast<int>(spellId));
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        sqlite3_finalize(stmt);
    }

    const char* insAction = "INSERT INTO character_action (guid, slot, type, action) VALUES (?,?,?,?);";
    if (sqlite3_prepare_v2(sp.db, insAction, -1, &stmt, nullptr) == SQLITE_OK) {
        for (int i = 0; i < static_cast<int>(actionBar.size()); i++) {
            const auto& slot = actionBar[i];
            if (slot.type == ActionBarSlot::EMPTY || slot.id == 0) continue;
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, i);
            sqlite3_bind_int(stmt, 3, static_cast<int>(slot.type));
            sqlite3_bind_int(stmt, 4, static_cast<int>(slot.id));
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        sqlite3_finalize(stmt);
    }

    const char* insAura =
        "INSERT INTO character_aura (guid, slot, spell, flags, level, charges, duration_ms, max_duration_ms, caster_guid) "
        "VALUES (?,?,?,?,?,?,?,?,?);";
    if (sqlite3_prepare_v2(sp.db, insAura, -1, &stmt, nullptr) == SQLITE_OK) {
        for (size_t i = 0; i < playerAuras.size(); i++) {
            const auto& aura = playerAuras[i];
            if (aura.spellId == 0) continue;
            sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(activeCharacterGuid_));
            sqlite3_bind_int(stmt, 2, static_cast<int>(i));
            sqlite3_bind_int(stmt, 3, static_cast<int>(aura.spellId));
            sqlite3_bind_int(stmt, 4, static_cast<int>(aura.flags));
            sqlite3_bind_int(stmt, 5, static_cast<int>(aura.level));
            sqlite3_bind_int(stmt, 6, static_cast<int>(aura.charges));
            sqlite3_bind_int(stmt, 7, static_cast<int>(aura.durationMs));
            sqlite3_bind_int(stmt, 8, static_cast<int>(aura.maxDurationMs));
            sqlite3_bind_int64(stmt, 9, static_cast<sqlite3_int64>(aura.casterGuid));
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        sqlite3_finalize(stmt);
    }

    sqlite3_exec(sp.db, "COMMIT;", nullptr, nullptr, nullptr);

    spDirtyFlags_ = SP_DIRTY_NONE;
    spDirtyHighPriority_ = false;
    spDirtyTimer_ = 0.0f;
    spPeriodicTimer_ = 0.0f;

    // Update cached character list position/level for UI.
    for (auto& ch : characters) {
        if (ch.guid == activeCharacterGuid_) {
            ch.level = static_cast<uint8_t>(localPlayerLevel_);
            ch.x = movementInfo.x;
            ch.y = movementInfo.y;
            ch.z = movementInfo.z;
            break;
        }
    }
}

void GameHandler::flushSinglePlayerSave() {
    saveSinglePlayerCharacterState(true);
}

void GameHandler::selectCharacter(uint64_t characterGuid) {
    if (state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot select character in state: ", (int)state);
        return;
    }

    LOG_INFO("========================================");
    LOG_INFO("   ENTERING WORLD");
    LOG_INFO("========================================");
    LOG_INFO("Character GUID: 0x", std::hex, characterGuid, std::dec);

    // Find character name for logging
    for (const auto& character : characters) {
        if (character.guid == characterGuid) {
            LOG_INFO("Character: ", character.name);
            LOG_INFO("Level ", (int)character.level, " ",
                     getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            break;
        }
    }

    // Store player GUID
    playerGuid = characterGuid;

    // Build CMSG_PLAYER_LOGIN packet
    auto packet = PlayerLoginPacket::build(characterGuid);

    // Send packet
    socket->send(packet);

    setState(WorldState::ENTERING_WORLD);
    LOG_INFO("CMSG_PLAYER_LOGIN sent, entering world...");
}

void GameHandler::handleLoginVerifyWorld(network::Packet& packet) {
    LOG_INFO("Handling SMSG_LOGIN_VERIFY_WORLD");

    LoginVerifyWorldData data;
    if (!LoginVerifyWorldParser::parse(packet, data)) {
        fail("Failed to parse SMSG_LOGIN_VERIFY_WORLD");
        return;
    }

    if (!data.isValid()) {
        fail("Invalid world entry data");
        return;
    }

    // Successfully entered the world!
    setState(WorldState::IN_WORLD);

    LOG_INFO("========================================");
    LOG_INFO("   SUCCESSFULLY ENTERED WORLD!");
    LOG_INFO("========================================");
    LOG_INFO("Map ID: ", data.mapId);
    LOG_INFO("Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("Orientation: ", data.orientation, " radians");
    LOG_INFO("Player is now in the game world");
    addSystemChatMessage("You have entered the world.");

    // Initialize movement info with world entry position (server → canonical)
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(data.x, data.y, data.z));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = data.orientation;
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    movementInfo.time = 0;

    // Send CMSG_SET_ACTIVE_MOVER (required by some servers)
    if (playerGuid != 0 && socket) {
        auto activeMoverPacket = SetActiveMoverPacket::build(playerGuid);
        socket->send(activeMoverPacket);
        LOG_INFO("Sent CMSG_SET_ACTIVE_MOVER for player 0x", std::hex, playerGuid, std::dec);
    }

    // Notify application to load terrain for this map/position (online mode)
    if (worldEntryCallback_) {
        worldEntryCallback_(data.mapId, data.x, data.y, data.z);
    }
}

void GameHandler::handleAccountDataTimes(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_ACCOUNT_DATA_TIMES");

    AccountDataTimesData data;
    if (!AccountDataTimesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACCOUNT_DATA_TIMES");
        return;
    }

    LOG_DEBUG("Account data times received (server time: ", data.serverTime, ")");
}

void GameHandler::handleMotd(network::Packet& packet) {
    LOG_INFO("Handling SMSG_MOTD");

    MotdData data;
    if (!MotdParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MOTD");
        return;
    }

    if (!data.isEmpty()) {
        LOG_INFO("========================================");
        LOG_INFO("   MESSAGE OF THE DAY");
        LOG_INFO("========================================");
        for (const auto& line : data.lines) {
            LOG_INFO(line);
            addSystemChatMessage(std::string("MOTD: ") + line);
        }
        LOG_INFO("========================================");
    }
}

void GameHandler::sendPing() {
    if (state != WorldState::IN_WORLD) {
        return;
    }

    // Increment sequence number
    pingSequence++;

    LOG_DEBUG("Sending CMSG_PING (heartbeat)");
    LOG_DEBUG("  Sequence: ", pingSequence);

    // Build and send ping packet
    auto packet = PingPacket::build(pingSequence, lastLatency);
    socket->send(packet);
}

void GameHandler::handlePong(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_PONG");

    PongData data;
    if (!PongParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_PONG");
        return;
    }

    // Verify sequence matches
    if (data.sequence != pingSequence) {
        LOG_WARNING("SMSG_PONG sequence mismatch: expected ", pingSequence,
                    ", got ", data.sequence);
        return;
    }

    LOG_DEBUG("Heartbeat acknowledged (sequence: ", data.sequence, ")");
}

void GameHandler::sendMovement(Opcode opcode) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send movement in state: ", (int)state);
        return;
    }

    // Use real millisecond timestamp (server validates for anti-cheat)
    static auto startTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    movementInfo.time = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count());

    // Update movement flags based on opcode
    switch (opcode) {
        case Opcode::CMSG_MOVE_START_FORWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FORWARD);
            break;
        case Opcode::CMSG_MOVE_START_BACKWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::BACKWARD);
            break;
        case Opcode::CMSG_MOVE_STOP:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                    static_cast<uint32_t>(MovementFlags::BACKWARD));
            break;
        case Opcode::CMSG_MOVE_START_STRAFE_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_LEFT);
            break;
        case Opcode::CMSG_MOVE_START_STRAFE_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
            break;
        case Opcode::CMSG_MOVE_STOP_STRAFE:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT));
            break;
        case Opcode::CMSG_MOVE_JUMP:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::CMSG_MOVE_START_TURN_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_LEFT);
            break;
        case Opcode::CMSG_MOVE_START_TURN_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_RIGHT);
            break;
        case Opcode::CMSG_MOVE_STOP_TURN:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
            break;
        case Opcode::CMSG_MOVE_FALL_LAND:
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::CMSG_MOVE_HEARTBEAT:
            // No flag changes — just sends current position
            break;
        default:
            break;
    }

    LOG_DEBUG("Sending movement packet: opcode=0x", std::hex,
              static_cast<uint16_t>(opcode), std::dec);

    // Convert canonical → server coordinates for the wire
    MovementInfo wireInfo = movementInfo;
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wireInfo.x, wireInfo.y, wireInfo.z));
    wireInfo.x = serverPos.x;
    wireInfo.y = serverPos.y;
    wireInfo.z = serverPos.z;

    // Build and send movement packet
    auto packet = MovementPacket::build(opcode, wireInfo, playerGuid);
    socket->send(packet);
}

void GameHandler::setPosition(float x, float y, float z) {
    movementInfo.x = x;
    movementInfo.y = y;
    movementInfo.z = z;
    if (singlePlayerMode_) {
        float dx = x - spLastDirtyX_;
        float dy = y - spLastDirtyY_;
        float dz = z - spLastDirtyZ_;
        float distSq = dx * dx + dy * dy + dz * dz;
        if (distSq >= 1.0f) {
            spLastDirtyX_ = x;
            spLastDirtyY_ = y;
            spLastDirtyZ_ = z;
            markSinglePlayerDirty(SP_DIRTY_POSITION, false);
        }
    }
}

void GameHandler::setOrientation(float orientation) {
    movementInfo.orientation = orientation;
    if (singlePlayerMode_) {
        float diff = std::fabs(orientation - spLastDirtyOrientation_);
        if (diff >= 0.1f) {
            spLastDirtyOrientation_ = orientation;
            markSinglePlayerDirty(SP_DIRTY_POSITION, false);
        }
    }
}

void GameHandler::handleUpdateObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_UPDATE_OBJECT");

    UpdateObjectData data;
    if (!UpdateObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_UPDATE_OBJECT");
        return;
    }

    // Process out-of-range objects first
    for (uint64_t guid : data.outOfRangeGuids) {
        if (entityManager.hasEntity(guid)) {
            LOG_INFO("Entity went out of range: 0x", std::hex, guid, std::dec);
            // Trigger creature despawn callback before removing entity
            if (creatureDespawnCallback_) {
                creatureDespawnCallback_(guid);
            }
            entityManager.removeEntity(guid);
        }
    }

    // Process update blocks
    for (const auto& block : data.blocks) {
        switch (block.updateType) {
            case UpdateType::CREATE_OBJECT:
            case UpdateType::CREATE_OBJECT2: {
                // Create new entity
                std::shared_ptr<Entity> entity;

                switch (block.objectType) {
                    case ObjectType::PLAYER:
                        entity = std::make_shared<Player>(block.guid);
                        LOG_INFO("Created player entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    case ObjectType::UNIT:
                        entity = std::make_shared<Unit>(block.guid);
                        LOG_INFO("Created unit entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    case ObjectType::GAMEOBJECT:
                        entity = std::make_shared<GameObject>(block.guid);
                        LOG_INFO("Created gameobject entity: 0x", std::hex, block.guid, std::dec);
                        break;

                    default:
                        entity = std::make_shared<Entity>(block.guid);
                        entity->setType(block.objectType);
                        LOG_INFO("Created generic entity: 0x", std::hex, block.guid, std::dec,
                                 ", type=", static_cast<int>(block.objectType));
                        break;
                }

                // Set position from movement block (server → canonical)
                if (block.hasMovement) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("  Position: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                }

                // Set fields
                for (const auto& field : block.fields) {
                    entity->setField(field.first, field.second);
                }

                // Add to manager
                entityManager.addEntity(block.guid, entity);

                // Auto-query names (Phase 1)
                if (block.objectType == ObjectType::PLAYER) {
                    queryPlayerName(block.guid);
                } else if (block.objectType == ObjectType::UNIT) {
                    // Extract creature entry from fields (UNIT_FIELD_ENTRY = index 54 in 3.3.5a,
                    // but the OBJECT_FIELD_ENTRY is at index 3)
                    auto it = block.fields.find(3); // OBJECT_FIELD_ENTRY
                    if (it != block.fields.end() && it->second != 0) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        unit->setEntry(it->second);
                        // Set name from cache immediately if available
                        std::string cached = getCachedCreatureName(it->second);
                        if (!cached.empty()) {
                            unit->setName(cached);
                        }
                        queryCreatureInfo(it->second, block.guid);
                    }
                }

                // Extract health/mana/power from fields (Phase 2) — single pass
                if (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) {
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    for (const auto& [key, val] : block.fields) {
                        switch (key) {
                            case 24:
                                unit->setHealth(val);
                                // Detect dead player on login
                                if (block.guid == playerGuid && val == 0) {
                                    playerDead_ = true;
                                    LOG_INFO("Player logged in dead");
                                }
                                break;
                            case 25: unit->setPower(val); break;
                            case 32: unit->setMaxHealth(val); break;
                            case 33: unit->setMaxPower(val); break;
                            case 55: unit->setFactionTemplate(val); break; // UNIT_FIELD_FACTIONTEMPLATE
                            case 59: unit->setUnitFlags(val); break;   // UNIT_FIELD_FLAGS
                            case 54: unit->setLevel(val); break;
                            case 67: unit->setDisplayId(val); break;  // UNIT_FIELD_DISPLAYID
                            case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                            default: break;
                        }
                    }
                    // Determine hostility from faction template for online creatures
                    if (unit->getFactionTemplate() != 0) {
                        unit->setHostile(isHostileFaction(unit->getFactionTemplate()));
                    }
                    // Trigger creature spawn callback for units with displayId
                    if (block.objectType == ObjectType::UNIT && unit->getDisplayId() != 0) {
                        if (creatureSpawnCallback_) {
                            creatureSpawnCallback_(block.guid, unit->getDisplayId(),
                                unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                        }
                        // Query quest giver status for NPCs with questgiver flag (0x02)
                        if ((unit->getNpcFlags() & 0x02) && socket) {
                            network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                            qsPkt.writeUInt64(block.guid);
                            socket->send(qsPkt);
                        }
                    }
                }
                // Track online item objects
                if (block.objectType == ObjectType::ITEM) {
                    auto entryIt = block.fields.find(3);  // OBJECT_FIELD_ENTRY
                    auto stackIt = block.fields.find(14); // ITEM_FIELD_STACK_COUNT
                    if (entryIt != block.fields.end() && entryIt->second != 0) {
                        OnlineItemInfo info;
                        info.entry = entryIt->second;
                        info.stackCount = (stackIt != block.fields.end()) ? stackIt->second : 1;
                        onlineItems_[block.guid] = info;
                        queryItemInfo(info.entry, block.guid);
                    }
                }

                // Extract XP / inventory slot fields for player entity
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    lastPlayerFields_ = block.fields;
                    detectInventorySlotBases(block.fields);
                    bool slotsChanged = false;
                    for (const auto& [key, val] : block.fields) {
                        if (key == 634) { playerXp_ = val; }                // PLAYER_XP
                        else if (key == 635) { playerNextLevelXp_ = val; }  // PLAYER_NEXT_LEVEL_XP
                        else if (key == 54) {
                            serverPlayerLevel_ = val;                        // UNIT_FIELD_LEVEL
                            for (auto& ch : characters) {
                                if (ch.guid == playerGuid) { ch.level = val; break; }
                            }
                        }
                        else if (key == 1170) { playerMoneyCopper_ = val; }  // PLAYER_FIELD_COINAGE
                    }
                    if (applyInventoryFields(block.fields)) slotsChanged = true;
                    if (slotsChanged) rebuildOnlineInventory();
                }
                break;
            }

            case UpdateType::VALUES: {
                // Update existing entity fields
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    for (const auto& field : block.fields) {
                        entity->setField(field.first, field.second);
                    }

                    // Update cached health/mana/power values (Phase 2) — single pass
                    if (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        for (const auto& [key, val] : block.fields) {
                            switch (key) {
                                case 24: {
                                    uint32_t oldHealth = unit->getHealth();
                                    unit->setHealth(val);
                                    if (val == 0) {
                                        if (block.guid == autoAttackTarget) {
                                            stopAutoAttack();
                                        }
                                        hostileAttackers_.erase(block.guid);
                                        // Player death
                                        if (block.guid == playerGuid) {
                                            playerDead_ = true;
                                            stopAutoAttack();
                                            LOG_INFO("Player died!");
                                        }
                                        // Trigger death animation for NPC units
                                        if (entity->getType() == ObjectType::UNIT && npcDeathCallback_) {
                                            npcDeathCallback_(block.guid);
                                        }
                                    } else if (oldHealth == 0 && val > 0) {
                                        // Player resurrection
                                        if (block.guid == playerGuid) {
                                            playerDead_ = false;
                                            LOG_INFO("Player resurrected!");
                                        }
                                        // Respawn: health went from 0 to >0, reset animation
                                        if (entity->getType() == ObjectType::UNIT && npcRespawnCallback_) {
                                            npcRespawnCallback_(block.guid);
                                        }
                                    }
                                    break;
                                }
                                case 25: unit->setPower(val); break;
                                case 32: unit->setMaxHealth(val); break;
                                case 33: unit->setMaxPower(val); break;
                                case 59: unit->setUnitFlags(val); break;   // UNIT_FIELD_FLAGS
                                case 54: unit->setLevel(val); break;
                                case 55:  // UNIT_FIELD_FACTIONTEMPLATE
                                    unit->setFactionTemplate(val);
                                    unit->setHostile(isHostileFaction(val));
                                    break;
                                case 82: unit->setNpcFlags(val); break;   // UNIT_NPC_FLAGS
                                default: break;
                            }
                        }
                    }
                    // Update XP / inventory slot fields for player entity
                    if (block.guid == playerGuid) {
                        for (const auto& [key, val] : block.fields) {
                            lastPlayerFields_[key] = val;
                        }
                        detectInventorySlotBases(block.fields);
                        bool slotsChanged = false;
                        for (const auto& [key, val] : block.fields) {
                            if (key == 634) {
                                playerXp_ = val;
                                LOG_INFO("XP updated: ", val);
                            }
                            else if (key == 635) {
                                playerNextLevelXp_ = val;
                                LOG_INFO("Next level XP updated: ", val);
                            }
                            else if (key == 54) {
                                serverPlayerLevel_ = val;
                                LOG_INFO("Level updated: ", val);
                                // Update Character struct for character selection screen
                                for (auto& ch : characters) {
                                    if (ch.guid == playerGuid) {
                                        ch.level = val;
                                        break;
                                    }
                                }
                            }
                            else if (key == 1170) {
                                playerMoneyCopper_ = val;
                                LOG_INFO("Money updated via VALUES: ", val, " copper");
                            }
                        }
                        if (applyInventoryFields(block.fields)) slotsChanged = true;
                        if (slotsChanged) rebuildOnlineInventory();
                    }

                    // Update item stack count for online items
                    if (entity->getType() == ObjectType::ITEM) {
                        for (const auto& [key, val] : block.fields) {
                            if (key == 14) { // ITEM_FIELD_STACK_COUNT
                                auto it = onlineItems_.find(block.guid);
                                if (it != onlineItems_.end()) it->second.stackCount = val;
                            }
                        }
                        rebuildOnlineInventory();
                    }

                    LOG_DEBUG("Updated entity fields: 0x", std::hex, block.guid, std::dec);
                } else {
                    LOG_WARNING("VALUES update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            case UpdateType::MOVEMENT: {
                // Update entity position (server → canonical)
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    entity->setPosition(pos.x, pos.y, pos.z, block.orientation);
                    LOG_DEBUG("Updated entity position: 0x", std::hex, block.guid, std::dec);
                } else {
                    LOG_WARNING("MOVEMENT update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            default:
                break;
        }
    }

    tabCycleStale = true;
    LOG_INFO("Entity count: ", entityManager.getEntityCount());

    // Late inventory base detection once items are known
    if (playerGuid != 0 && invSlotBase_ < 0 && !lastPlayerFields_.empty() && !onlineItems_.empty()) {
        detectInventorySlotBases(lastPlayerFields_);
        if (invSlotBase_ >= 0) {
            if (applyInventoryFields(lastPlayerFields_)) {
                rebuildOnlineInventory();
            }
        }
    }
}

void GameHandler::handleCompressedUpdateObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_COMPRESSED_UPDATE_OBJECT, packet size: ", packet.getSize());

    // First 4 bytes = decompressed size
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_COMPRESSED_UPDATE_OBJECT too small");
        return;
    }

    uint32_t decompressedSize = packet.readUInt32();
    LOG_INFO("  Decompressed size: ", decompressedSize);

    if (decompressedSize == 0 || decompressedSize > 1024 * 1024) {
        LOG_WARNING("Invalid decompressed size: ", decompressedSize);
        return;
    }

    // Remaining data is zlib compressed
    size_t compressedSize = packet.getSize() - packet.getReadPos();
    const uint8_t* compressedData = packet.getData().data() + packet.getReadPos();

    // Decompress
    std::vector<uint8_t> decompressed(decompressedSize);
    uLongf destLen = decompressedSize;
    int ret = uncompress(decompressed.data(), &destLen, compressedData, compressedSize);

    if (ret != Z_OK) {
        LOG_WARNING("Failed to decompress UPDATE_OBJECT: zlib error ", ret);
        return;
    }

    LOG_DEBUG("  Decompressed ", compressedSize, " -> ", destLen, " bytes");

    // Create packet from decompressed data and parse it
    network::Packet decompressedPacket(static_cast<uint16_t>(Opcode::SMSG_UPDATE_OBJECT), decompressed);
    handleUpdateObject(decompressedPacket);
}

void GameHandler::handleDestroyObject(network::Packet& packet) {
    LOG_INFO("Handling SMSG_DESTROY_OBJECT");

    DestroyObjectData data;
    if (!DestroyObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_DESTROY_OBJECT");
        return;
    }

    // Remove entity
    if (entityManager.hasEntity(data.guid)) {
        entityManager.removeEntity(data.guid);
        LOG_INFO("Destroyed entity: 0x", std::hex, data.guid, std::dec,
                 " (", (data.isDeath ? "death" : "despawn"), ")");
    } else {
        LOG_WARNING("Destroy object for unknown entity: 0x", std::hex, data.guid, std::dec);
    }

    // Clean up auto-attack and target if destroyed entity was our target
    if (data.guid == autoAttackTarget) {
        stopAutoAttack();
    }
    if (data.guid == targetGuid) {
        targetGuid = 0;
    }
    hostileAttackers_.erase(data.guid);

    // Remove online item tracking
    if (onlineItems_.erase(data.guid)) {
        rebuildOnlineInventory();
    }

    // Clean up quest giver status
    npcQuestStatus_.erase(data.guid);

    tabCycleStale = true;
    LOG_INFO("Entity count: ", entityManager.getEntityCount());
}

void GameHandler::sendChatMessage(ChatType type, const std::string& message, const std::string& target) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send chat in state: ", (int)state);
        return;
    }

    if (message.empty()) {
        LOG_WARNING("Cannot send empty chat message");
        return;
    }

    LOG_INFO("Sending chat message: [", getChatTypeString(type), "] ", message);

    // Determine language based on character (for now, use COMMON)
    ChatLanguage language = ChatLanguage::COMMON;

    // Build and send packet
    auto packet = MessageChatPacket::build(type, language, message, target);
    socket->send(packet);
}

void GameHandler::handleMessageChat(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_MESSAGECHAT");

    MessageChatData data;
    if (!MessageChatParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MESSAGECHAT");
        return;
    }

    // Add to chat history
    chatHistory.push_back(data);

    // Limit chat history size
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.erase(chatHistory.begin());
    }

    // Log the message
    std::string senderInfo;
    if (!data.senderName.empty()) {
        senderInfo = data.senderName;
    } else if (data.senderGuid != 0) {
        // Try to find entity name
        auto entity = entityManager.getEntity(data.senderGuid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::dynamic_pointer_cast<Player>(entity);
            if (player && !player->getName().empty()) {
                senderInfo = player->getName();
            } else {
                senderInfo = "Player-" + std::to_string(data.senderGuid);
            }
        } else {
            senderInfo = "Unknown-" + std::to_string(data.senderGuid);
        }
    } else {
        senderInfo = "System";
    }

    std::string channelInfo;
    if (!data.channelName.empty()) {
        channelInfo = "[" + data.channelName + "] ";
    }

    LOG_INFO("========================================");
    LOG_INFO(" CHAT [", getChatTypeString(data.type), "]");
    LOG_INFO("========================================");
    LOG_INFO(channelInfo, senderInfo, ": ", data.message);
    LOG_INFO("========================================");
}

void GameHandler::setTarget(uint64_t guid) {
    if (guid == targetGuid) return;
    targetGuid = guid;

    // Inform server of target selection (Phase 1)
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = SetSelectionPacket::build(guid);
        socket->send(packet);
    }

    if (guid != 0) {
        LOG_INFO("Target set: 0x", std::hex, guid, std::dec);
    }
}

void GameHandler::clearTarget() {
    if (targetGuid != 0) {
        LOG_INFO("Target cleared");
    }
    targetGuid = 0;
    tabCycleIndex = -1;
    tabCycleStale = true;
}

std::shared_ptr<Entity> GameHandler::getTarget() const {
    if (targetGuid == 0) return nullptr;
    return entityManager.getEntity(targetGuid);
}

void GameHandler::releaseSpirit() {
    if (!playerDead_) return;
    if (socket && state == WorldState::IN_WORLD) {
        auto packet = RepopRequestPacket::build();
        socket->send(packet);
        LOG_INFO("Sent CMSG_REPOP_REQUEST (Release Spirit)");
    }
}

void GameHandler::activateSpiritHealer(uint64_t npcGuid) {
    if (!playerDead_) return;
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SpiritHealerActivatePacket::build(npcGuid);
    socket->send(packet);
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE to 0x", std::hex, npcGuid, std::dec);
}

void GameHandler::tabTarget(float playerX, float playerY, float playerZ) {
    // Rebuild cycle list if stale
    if (tabCycleStale) {
        tabCycleList.clear();
        tabCycleIndex = -1;

        struct EntityDist {
            uint64_t guid;
            float distance;
        };
        std::vector<EntityDist> sortable;

        for (const auto& [guid, entity] : entityManager.getEntities()) {
            auto t = entity->getType();
            if (t != ObjectType::UNIT && t != ObjectType::PLAYER) continue;
            if (guid == playerGuid) continue;  // Don't tab-target self
            float dx = entity->getX() - playerX;
            float dy = entity->getY() - playerY;
            float dz = entity->getZ() - playerZ;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            sortable.push_back({guid, dist});
        }

        std::sort(sortable.begin(), sortable.end(),
                  [](const EntityDist& a, const EntityDist& b) { return a.distance < b.distance; });

        for (const auto& ed : sortable) {
            tabCycleList.push_back(ed.guid);
        }
        tabCycleStale = false;
    }

    if (tabCycleList.empty()) {
        clearTarget();
        return;
    }

    tabCycleIndex = (tabCycleIndex + 1) % static_cast<int>(tabCycleList.size());
    setTarget(tabCycleList[tabCycleIndex]);
}

void GameHandler::addLocalChatMessage(const MessageChatData& msg) {
    chatHistory.push_back(msg);
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.pop_front();
    }
}

// ============================================================
// Phase 1: Name Queries
// ============================================================

void GameHandler::queryPlayerName(uint64_t guid) {
    if (playerNameCache.count(guid) || pendingNameQueries.count(guid)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingNameQueries.insert(guid);
    auto packet = NameQueryPacket::build(guid);
    socket->send(packet);
}

void GameHandler::queryCreatureInfo(uint32_t entry, uint64_t guid) {
    if (creatureInfoCache.count(entry) || pendingCreatureQueries.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingCreatureQueries.insert(entry);
    auto packet = CreatureQueryPacket::build(entry, guid);
    socket->send(packet);
}

std::string GameHandler::getCachedPlayerName(uint64_t guid) const {
    auto it = playerNameCache.find(guid);
    return (it != playerNameCache.end()) ? it->second : "";
}

std::string GameHandler::getCachedCreatureName(uint32_t entry) const {
    auto it = creatureInfoCache.find(entry);
    return (it != creatureInfoCache.end()) ? it->second.name : "";
}

void GameHandler::handleNameQueryResponse(network::Packet& packet) {
    NameQueryResponseData data;
    if (!NameQueryResponseParser::parse(packet, data)) return;

    pendingNameQueries.erase(data.guid);

    if (data.isValid()) {
        playerNameCache[data.guid] = data.name;
        // Update entity name
        auto entity = entityManager.getEntity(data.guid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<Player>(entity);
            player->setName(data.name);
        }
    }
}

void GameHandler::handleCreatureQueryResponse(network::Packet& packet) {
    CreatureQueryResponseData data;
    if (!CreatureQueryResponseParser::parse(packet, data)) return;

    pendingCreatureQueries.erase(data.entry);

    if (data.isValid()) {
        creatureInfoCache[data.entry] = data;
        // Update all unit entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(entity);
                if (unit->getEntry() == data.entry) {
                    unit->setName(data.name);
                }
            }
        }
    }
}

// ============================================================
// Item Query
// ============================================================

void GameHandler::queryItemInfo(uint32_t entry, uint64_t guid) {
    if (itemInfoCache_.count(entry) || pendingItemQueries_.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingItemQueries_.insert(entry);
    auto packet = ItemQueryPacket::build(entry, guid);
    socket->send(packet);
}

void GameHandler::handleItemQueryResponse(network::Packet& packet) {
    ItemQueryResponseData data;
    if (!ItemQueryResponseParser::parse(packet, data)) return;

    pendingItemQueries_.erase(data.entry);

    if (data.valid) {
        itemInfoCache_[data.entry] = data;
        rebuildOnlineInventory();
    }
}

uint64_t GameHandler::resolveOnlineItemGuid(uint32_t itemId) const {
    if (itemId == 0) return 0;
    uint64_t found = 0;
    for (const auto& [guid, info] : onlineItems_) {
        if (info.entry != itemId) continue;
        if (found != 0) {
            return 0; // Ambiguous
        }
        found = guid;
    }
    return found;
}

void GameHandler::detectInventorySlotBases(const std::map<uint16_t, uint32_t>& fields) {
    if (invSlotBase_ >= 0 && packSlotBase_ >= 0) return;
    if (onlineItems_.empty() || fields.empty()) return;

    std::vector<uint16_t> matchingPairs;
    matchingPairs.reserve(32);

    for (const auto& [idx, low] : fields) {
        if ((idx % 2) != 0) continue;
        auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
        if (itHigh == fields.end()) continue;
        uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
        if (guid == 0) continue;
        if (onlineItems_.count(guid)) {
            matchingPairs.push_back(idx);
        }
    }

    if (matchingPairs.empty()) return;
    std::sort(matchingPairs.begin(), matchingPairs.end());

    if (invSlotBase_ < 0) {
        // The lowest matching field is the first EQUIPPED slot (not necessarily HEAD).
        // With 2+ matches we can derive the true base: all matches must be at
        // even offsets from the base, spaced 2 fields per slot.
        // Use the known 3.3.5a default (324) and verify matches align to it.
        constexpr int knownBase = 324;
        constexpr int slotStride = 2;
        bool allAlign = true;
        for (uint16_t p : matchingPairs) {
            if (p < knownBase || (p - knownBase) % slotStride != 0) {
                allAlign = false;
                break;
            }
        }
        if (allAlign) {
            invSlotBase_ = knownBase;
        } else {
            // Fallback: if we have 2+ matches, derive base from their spacing
            if (matchingPairs.size() >= 2) {
                uint16_t lo = matchingPairs[0];
                // lo must be base + 2*slotN, and slotN is 0..22
                // Try each possible slot for 'lo' and see if all others also land on valid slots
                for (int s = 0; s <= 22; s++) {
                    int candidate = lo - s * slotStride;
                    if (candidate < 0) break;
                    bool ok = true;
                    for (uint16_t p : matchingPairs) {
                        int off = p - candidate;
                        if (off < 0 || off % slotStride != 0 || off / slotStride > 22) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        invSlotBase_ = candidate;
                        break;
                    }
                }
                if (invSlotBase_ < 0) invSlotBase_ = knownBase;
            } else {
                invSlotBase_ = knownBase;
            }
        }
        packSlotBase_ = invSlotBase_ + (game::Inventory::NUM_EQUIP_SLOTS * 2);
        LOG_INFO("Detected inventory field base: equip=", invSlotBase_,
                 " pack=", packSlotBase_);
    }
}

bool GameHandler::applyInventoryFields(const std::map<uint16_t, uint32_t>& fields) {
    bool slotsChanged = false;
    // WoW 3.3.5a: PLAYER_FIELD_INV_SLOT_HEAD = UNIT_END + 0x00B0 = 324
    //             PLAYER_FIELD_PACK_SLOT_1  = UNIT_END + 0x00DE = 370
    int equipBase = (invSlotBase_ >= 0) ? invSlotBase_ : 324;
    int packBase = (packSlotBase_ >= 0) ? packSlotBase_ : 370;

    for (const auto& [key, val] : fields) {
        if (key >= equipBase && key <= equipBase + (game::Inventory::NUM_EQUIP_SLOTS * 2 - 1)) {
            int slotIndex = (key - equipBase) / 2;
            bool isLow = ((key - equipBase) % 2 == 0);
            if (slotIndex < static_cast<int>(equipSlotGuids_.size())) {
                uint64_t& guid = equipSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        } else if (key >= packBase && key <= packBase + (game::Inventory::BACKPACK_SLOTS * 2 - 1)) {
            int slotIndex = (key - packBase) / 2;
            bool isLow = ((key - packBase) % 2 == 0);
            if (slotIndex < static_cast<int>(backpackSlotGuids_.size())) {
                uint64_t& guid = backpackSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }
    }

    return slotsChanged;
}

void GameHandler::rebuildOnlineInventory() {
    if (singlePlayerMode_) return;

    inventory = Inventory();

    // Equipment slots
    for (int i = 0; i < 23; i++) {
        uint64_t guid = equipSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }
        if (def.itemId != 0) {
            auto& db = getSinglePlayerLootDb();
            auto itTpl = db.itemTemplates.find(def.itemId);
            if (itTpl != db.itemTemplates.end()) {
                if (def.name.empty() || def.name.rfind("Item ", 0) == 0) def.name = itTpl->second.name;
                if (def.quality == ItemQuality::COMMON && itTpl->second.quality != 0) {
                    def.quality = static_cast<ItemQuality>(itTpl->second.quality);
                }
                if (def.inventoryType == 0 && itTpl->second.inventoryType != 0) {
                    def.inventoryType = itTpl->second.inventoryType;
                }
                if (def.maxStack <= 1 && itTpl->second.maxStack > 1) {
                    def.maxStack = static_cast<uint32_t>(itTpl->second.maxStack);
                }
                if (def.displayInfoId == 0 && itTpl->second.displayId != 0) {
                    def.displayInfoId = itTpl->second.displayId;
                }
                if (def.armor == 0 && itTpl->second.armor > 0) def.armor = itTpl->second.armor;
                if (def.stamina == 0 && itTpl->second.stamina > 0) def.stamina = itTpl->second.stamina;
                if (def.strength == 0 && itTpl->second.strength > 0) def.strength = itTpl->second.strength;
                if (def.agility == 0 && itTpl->second.agility > 0) def.agility = itTpl->second.agility;
                if (def.intellect == 0 && itTpl->second.intellect > 0) def.intellect = itTpl->second.intellect;
                if (def.spirit == 0 && itTpl->second.spirit > 0) def.spirit = itTpl->second.spirit;
            }
        }

        inventory.setEquipSlot(static_cast<EquipSlot>(i), def);
    }

    // Backpack slots
    for (int i = 0; i < 16; i++) {
        uint64_t guid = backpackSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }
        if (def.itemId != 0) {
            auto& db = getSinglePlayerLootDb();
            auto itTpl = db.itemTemplates.find(def.itemId);
            if (itTpl != db.itemTemplates.end()) {
                if (def.name.empty() || def.name.rfind("Item ", 0) == 0) def.name = itTpl->second.name;
                if (def.quality == ItemQuality::COMMON && itTpl->second.quality != 0) {
                    def.quality = static_cast<ItemQuality>(itTpl->second.quality);
                }
                if (def.inventoryType == 0 && itTpl->second.inventoryType != 0) {
                    def.inventoryType = itTpl->second.inventoryType;
                }
                if (def.maxStack <= 1 && itTpl->second.maxStack > 1) {
                    def.maxStack = static_cast<uint32_t>(itTpl->second.maxStack);
                }
                if (def.displayInfoId == 0 && itTpl->second.displayId != 0) {
                    def.displayInfoId = itTpl->second.displayId;
                }
                if (def.armor == 0 && itTpl->second.armor > 0) def.armor = itTpl->second.armor;
                if (def.stamina == 0 && itTpl->second.stamina > 0) def.stamina = itTpl->second.stamina;
                if (def.strength == 0 && itTpl->second.strength > 0) def.strength = itTpl->second.strength;
                if (def.agility == 0 && itTpl->second.agility > 0) def.agility = itTpl->second.agility;
                if (def.intellect == 0 && itTpl->second.intellect > 0) def.intellect = itTpl->second.intellect;
                if (def.spirit == 0 && itTpl->second.spirit > 0) def.spirit = itTpl->second.spirit;
            }
        }

        inventory.setBackpackSlot(i, def);
    }

    onlineEquipDirty_ = true;

    LOG_DEBUG("Rebuilt online inventory: equip=", [&](){
        int c = 0; for (auto g : equipSlotGuids_) if (g) c++; return c;
    }(), " backpack=", [&](){
        int c = 0; for (auto g : backpackSlotGuids_) if (g) c++; return c;
    }());
}

// ============================================================
// Phase 2: Combat
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    autoAttacking = true;
    autoAttackTarget = targetGuid;
    autoAttackOutOfRange_ = false;
    swingTimer_ = 0.0f;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackSwingPacket::build(targetGuid);
        socket->send(packet);
    }
    LOG_INFO("Starting auto-attack on 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopAutoAttack() {
    if (!autoAttacking) return;
    autoAttacking = false;
    autoAttackTarget = 0;
    autoAttackOutOfRange_ = false;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackStopPacket::build();
        socket->send(packet);
    }
    LOG_INFO("Stopping auto-attack");
}

void GameHandler::addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId, bool isPlayerSource) {
    CombatTextEntry entry;
    entry.type = type;
    entry.amount = amount;
    entry.spellId = spellId;
    entry.age = 0.0f;
    entry.isPlayerSource = isPlayerSource;
    combatText.push_back(entry);
}

void GameHandler::updateCombatText(float deltaTime) {
    for (auto& entry : combatText) {
        entry.age += deltaTime;
    }
    combatText.erase(
        std::remove_if(combatText.begin(), combatText.end(),
                       [](const CombatTextEntry& e) { return e.isExpired(); }),
        combatText.end());
}

void GameHandler::handleAttackStart(network::Packet& packet) {
    AttackStartData data;
    if (!AttackStartParser::parse(packet, data)) return;

    if (data.attackerGuid == playerGuid) {
        autoAttacking = true;
        autoAttackTarget = data.victimGuid;
    }
}

void GameHandler::handleAttackStop(network::Packet& packet) {
    AttackStopData data;
    if (!AttackStopParser::parse(packet, data)) return;

    // Don't clear autoAttacking on SMSG_ATTACKSTOP - the server sends this
    // when the attack loop pauses (out of range, etc). The player's intent
    // to attack persists until target dies or player explicitly cancels.
    // We'll re-send CMSG_ATTACKSWING periodically in the update loop.
    if (data.attackerGuid == playerGuid) {
        LOG_DEBUG("SMSG_ATTACKSTOP received (keeping auto-attack intent)");
    } else if (data.victimGuid == playerGuid) {
        hostileAttackers_.erase(data.attackerGuid);
    }
}

void GameHandler::handleMonsterMove(network::Packet& packet) {
    MonsterMoveData data;
    if (!MonsterMoveParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MONSTER_MOVE");
        return;
    }

    // Update entity position in entity manager
    auto entity = entityManager.getEntity(data.guid);
    if (entity) {
        if (data.hasDest) {
            // Convert destination from server to canonical coords
            glm::vec3 destCanonical = core::coords::serverToCanonical(
                glm::vec3(data.destX, data.destY, data.destZ));

            // Calculate facing angle
            float orientation = entity->getOrientation();
            if (data.moveType == 4) {
                // FacingAngle - server specifies exact angle
                orientation = data.facingAngle;
            } else if (data.moveType == 3) {
                // FacingTarget - face toward the target entity
                auto target = entityManager.getEntity(data.facingTarget);
                if (target) {
                    float dx = target->getX() - entity->getX();
                    float dy = target->getY() - entity->getY();
                    if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                        orientation = std::atan2(dy, dx);
                    }
                }
            } else {
                // Normal move - face toward destination
                float dx = destCanonical.x - entity->getX();
                float dy = destCanonical.y - entity->getY();
                if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                    orientation = std::atan2(dy, dx);
                }
            }

            // Interpolate entity position alongside renderer (so targeting matches visual)
            entity->startMoveTo(destCanonical.x, destCanonical.y, destCanonical.z,
                                orientation, data.duration / 1000.0f);

            // Notify renderer to smoothly move the creature
            if (creatureMoveCallback_) {
                creatureMoveCallback_(data.guid,
                    destCanonical.x, destCanonical.y, destCanonical.z,
                    data.duration);
            }
        } else if (data.moveType == 1) {
            // Stop at current position
            glm::vec3 posCanonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            entity->setPosition(posCanonical.x, posCanonical.y, posCanonical.z,
                                entity->getOrientation());

            if (creatureMoveCallback_) {
                creatureMoveCallback_(data.guid,
                    posCanonical.x, posCanonical.y, posCanonical.z, 0);
            }
        }
    }
}

void GameHandler::handleAttackerStateUpdate(network::Packet& packet) {
    AttackerStateUpdateData data;
    if (!AttackerStateUpdateParser::parse(packet, data)) return;

    bool isPlayerAttacker = (data.attackerGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);
    if (isPlayerAttacker && meleeSwingCallback_) {
        meleeSwingCallback_();
    }
    if (!isPlayerAttacker && npcSwingCallback_) {
        npcSwingCallback_(data.attackerGuid);
    }

    if (isPlayerTarget && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
    }

    if (data.isMiss()) {
        addCombatText(CombatTextEntry::MISS, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 1) {
        addCombatText(CombatTextEntry::DODGE, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 2) {
        addCombatText(CombatTextEntry::PARRY, 0, 0, isPlayerAttacker);
    } else {
        auto type = data.isCrit() ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::MELEE_DAMAGE;
        addCombatText(type, data.totalDamage, 0, isPlayerAttacker);
    }

    (void)isPlayerTarget; // Used for future incoming damage display
}

void GameHandler::handleSpellDamageLog(network::Packet& packet) {
    SpellDamageLogData data;
    if (!SpellDamageLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.attackerGuid == playerGuid);
    auto type = data.isCrit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::SPELL_DAMAGE;
    addCombatText(type, static_cast<int32_t>(data.damage), data.spellId, isPlayerSource);
}

void GameHandler::handleSpellHealLog(network::Packet& packet) {
    SpellHealLogData data;
    if (!SpellHealLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.casterGuid == playerGuid);
    auto type = data.isCrit ? CombatTextEntry::CRIT_HEAL : CombatTextEntry::HEAL;
    addCombatText(type, static_cast<int32_t>(data.heal), data.spellId, isPlayerSource);
}

// ============================================================
// Phase 3: Spells
// ============================================================

void GameHandler::castSpell(uint32_t spellId, uint64_t targetGuid) {
    // Hearthstone (8690) — handle locally when no server connection (single-player)
    if (spellId == 8690 && hearthstoneCallback) {
        LOG_INFO("Hearthstone: teleporting home");
        hearthstoneCallback();
        return;
    }

    // Attack (6603) routes to auto-attack instead of cast (works without server)
    if (spellId == 6603) {
        uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
        if (target != 0) {
            if (autoAttacking) {
                stopAutoAttack();
            } else {
                startAutoAttack(target);
            }
        }
        return;
    }

    if (state != WorldState::IN_WORLD || !socket) return;

    if (casting) return; // Already casting

    uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
    auto packet = CastSpellPacket::build(spellId, target, ++castCount);
    socket->send(packet);
    LOG_INFO("Casting spell: ", spellId, " on 0x", std::hex, target, std::dec);
}

void GameHandler::cancelCast() {
    if (!casting) return;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = CancelCastPacket::build(currentCastSpellId);
        socket->send(packet);
    }
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;
}

void GameHandler::cancelAura(uint32_t spellId) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = CancelAuraPacket::build(spellId);
    socket->send(packet);
}

void GameHandler::setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id) {
    if (slot < 0 || slot >= ACTION_BAR_SLOTS) return;
    actionBar[slot].type = type;
    actionBar[slot].id = id;
    markSinglePlayerDirty(SP_DIRTY_ACTIONBAR, true);
}

float GameHandler::getSpellCooldown(uint32_t spellId) const {
    auto it = spellCooldowns.find(spellId);
    return (it != spellCooldowns.end()) ? it->second : 0.0f;
}

void GameHandler::handleInitialSpells(network::Packet& packet) {
    InitialSpellsData data;
    if (!InitialSpellsParser::parse(packet, data)) return;

    knownSpells = data.spellIds;

    // Ensure Attack (6603) and Hearthstone (8690) are always present
    if (std::find(knownSpells.begin(), knownSpells.end(), 6603u) == knownSpells.end()) {
        knownSpells.insert(knownSpells.begin(), 6603u);
    }
    if (std::find(knownSpells.begin(), knownSpells.end(), 8690u) == knownSpells.end()) {
        knownSpells.push_back(8690u);
    }

    // Set initial cooldowns
    for (const auto& cd : data.cooldowns) {
        if (cd.cooldownMs > 0) {
            spellCooldowns[cd.spellId] = cd.cooldownMs / 1000.0f;
        }
    }

    // Auto-populate action bar: Attack in slot 1, Hearthstone in slot 12
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone

    LOG_INFO("Learned ", knownSpells.size(), " spells");
}

void GameHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    if (!CastFailedParser::parse(packet, data)) return;

    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Add system message about failed cast with readable reason
    const char* reason = getSpellCastResultString(data.result);
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    if (reason) {
        msg.message = reason;
    } else {
        msg.message = "Spell cast failed (error " + std::to_string(data.result) + ")";
    }
    addLocalChatMessage(msg);
}

void GameHandler::handleSpellStart(network::Packet& packet) {
    SpellStartData data;
    if (!SpellStartParser::parse(packet, data)) return;

    // If this is the player's own cast, start cast bar
    if (data.casterUnit == playerGuid && data.castTime > 0) {
        casting = true;
        currentCastSpellId = data.spellId;
        castTimeTotal = data.castTime / 1000.0f;
        castTimeRemaining = castTimeTotal;
    }
}

void GameHandler::handleSpellGo(network::Packet& packet) {
    SpellGoData data;
    if (!SpellGoParser::parse(packet, data)) return;

    // Cast completed
    if (data.casterUnit == playerGuid) {
        casting = false;
        currentCastSpellId = 0;
        castTimeRemaining = 0.0f;
    }
}

void GameHandler::handleSpellCooldown(network::Packet& packet) {
    SpellCooldownData data;
    if (!SpellCooldownParser::parse(packet, data)) return;

    for (const auto& [spellId, cooldownMs] : data.cooldowns) {
        float seconds = cooldownMs / 1000.0f;
        spellCooldowns[spellId] = seconds;
        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
                slot.cooldownTotal = seconds;
                slot.cooldownRemaining = seconds;
            }
        }
    }
}

void GameHandler::handleCooldownEvent(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    // Cooldown finished
    spellCooldowns.erase(spellId);
    for (auto& slot : actionBar) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
            slot.cooldownRemaining = 0.0f;
        }
    }
}

void GameHandler::handleAuraUpdate(network::Packet& packet, bool isAll) {
    AuraUpdateData data;
    if (!AuraUpdateParser::parse(packet, data, isAll)) return;

    // Determine which aura list to update
    std::vector<AuraSlot>* auraList = nullptr;
    if (data.guid == playerGuid) {
        auraList = &playerAuras;
    } else if (data.guid == targetGuid) {
        auraList = &targetAuras;
    }

    if (auraList) {
        for (const auto& [slot, aura] : data.updates) {
            // Ensure vector is large enough
            while (auraList->size() <= slot) {
                auraList->push_back(AuraSlot{});
            }
            (*auraList)[slot] = aura;
        }
    }
    if (singlePlayerMode_ && data.guid == playerGuid) {
        markSinglePlayerDirty(SP_DIRTY_AURAS, true);
    }
}

void GameHandler::handleLearnedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.push_back(spellId);
    markSinglePlayerDirty(SP_DIRTY_SPELLS, true);
    LOG_INFO("Learned spell: ", spellId);
}

void GameHandler::handleRemovedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.erase(
        std::remove(knownSpells.begin(), knownSpells.end(), spellId),
        knownSpells.end());
    markSinglePlayerDirty(SP_DIRTY_SPELLS, true);
    LOG_INFO("Removed spell: ", spellId);
}

// ============================================================
// Phase 4: Group/Party
// ============================================================

void GameHandler::inviteToGroup(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupInvitePacket::build(playerName);
    socket->send(packet);
    LOG_INFO("Inviting ", playerName, " to group");
}

void GameHandler::acceptGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupAcceptPacket::build();
    socket->send(packet);
    LOG_INFO("Accepted group invite");
}

void GameHandler::declineGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupDeclinePacket::build();
    socket->send(packet);
    LOG_INFO("Declined group invite");
}

void GameHandler::leaveGroup() {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupDisbandPacket::build();
    socket->send(packet);
    partyData = GroupListData{};
    LOG_INFO("Left group");
}

void GameHandler::handleGroupInvite(network::Packet& packet) {
    GroupInviteResponseData data;
    if (!GroupInviteResponseParser::parse(packet, data)) return;

    pendingGroupInvite = true;
    pendingInviterName = data.inviterName;
    LOG_INFO("Group invite from: ", data.inviterName);
    if (!data.inviterName.empty()) {
        addSystemChatMessage(data.inviterName + " has invited you to a group.");
    }
}

void GameHandler::handleGroupDecline(network::Packet& packet) {
    GroupDeclineData data;
    if (!GroupDeclineResponseParser::parse(packet, data)) return;

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = data.playerName + " has declined your group invitation.";
    addLocalChatMessage(msg);
}

void GameHandler::handleGroupList(network::Packet& packet) {
    if (!GroupListParser::parse(packet, partyData)) return;

    if (partyData.isEmpty()) {
        LOG_INFO("No longer in a group");
        addSystemChatMessage("You are no longer in a group.");
    } else {
        LOG_INFO("In group with ", partyData.memberCount, " members");
        addSystemChatMessage("You are now in a group with " + std::to_string(partyData.memberCount) + " members.");
    }
}

void GameHandler::handleGroupUninvite(network::Packet& packet) {
    (void)packet;
    partyData = GroupListData{};
    LOG_INFO("Removed from group");

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "You have been removed from the group.";
    addLocalChatMessage(msg);
}

void GameHandler::handlePartyCommandResult(network::Packet& packet) {
    PartyCommandResultData data;
    if (!PartyCommandResultParser::parse(packet, data)) return;

    if (data.result != PartyResult::OK) {
        MessageChatData msg;
        msg.type = ChatType::SYSTEM;
        msg.language = ChatLanguage::UNIVERSAL;
        msg.message = "Party command failed (error " + std::to_string(static_cast<uint32_t>(data.result)) + ")";
        if (!data.name.empty()) msg.message += " for " + data.name;
        addLocalChatMessage(msg);
    }
}

// ============================================================
// Phase 5: Loot, Gossip, Vendor
// ============================================================

void GameHandler::lootTarget(uint64_t guid) {
    if (singlePlayerMode_) {
        auto entity = entityManager.getEntity(guid);
        if (!entity || entity->getType() != ObjectType::UNIT) return;
        auto unit = std::static_pointer_cast<Unit>(entity);
        if (unit->getHealth() != 0) return;

        auto it = localLootState_.find(guid);
        if (it == localLootState_.end()) {
            LocalLootState state;
            state.data = generateLocalLoot(guid);
            it = localLootState_.emplace(guid, std::move(state)).first;
        }
        if (it->second.data.items.empty() && it->second.data.gold == 0) {
            addSystemChatMessage("No loot.");
            return;
        }
        simulateLootResponse(it->second.data);
        return;
    }

    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = LootPacket::build(guid);
    socket->send(packet);
}

void GameHandler::lootItem(uint8_t slotIndex) {
    if (singlePlayerMode_) {
        if (!lootWindowOpen) return;
        auto it = std::find_if(currentLoot.items.begin(), currentLoot.items.end(),
                               [slotIndex](const LootItem& item) { return item.slotIndex == slotIndex; });
        if (it == currentLoot.items.end()) return;

        auto& db = getSinglePlayerLootDb();
        ItemDef def;
        def.itemId = it->itemId;
        def.stackCount = it->count;
        def.maxStack = it->count;

        auto itTpl = db.itemTemplates.find(it->itemId);
        if (itTpl != db.itemTemplates.end()) {
            def.name = itTpl->second.name.empty()
                ? ("Item " + std::to_string(it->itemId))
                : itTpl->second.name;
            def.quality = static_cast<ItemQuality>(itTpl->second.quality);
            def.inventoryType = itTpl->second.inventoryType;
            def.maxStack = std::max(def.maxStack, static_cast<uint32_t>(itTpl->second.maxStack));
            def.sellPrice = itTpl->second.sellPrice;
            def.displayInfoId = itTpl->second.displayId;
            def.armor = itTpl->second.armor;
            def.stamina = itTpl->second.stamina;
            def.strength = itTpl->second.strength;
            def.agility = itTpl->second.agility;
            def.intellect = itTpl->second.intellect;
            def.spirit = itTpl->second.spirit;
        } else {
            def.name = "Item " + std::to_string(it->itemId);
        }

        if (inventory.addItem(def)) {
            simulateLootRemove(slotIndex);
            addSystemChatMessage("You receive item: " + def.name + " x" + std::to_string(def.stackCount) + ".");
            markSinglePlayerDirty(SP_DIRTY_INVENTORY, true);
            if (currentLoot.lootGuid != 0) {
                auto st = localLootState_.find(currentLoot.lootGuid);
                if (st != localLootState_.end()) {
                    auto& items = st->second.data.items;
                    items.erase(std::remove_if(items.begin(), items.end(),
                                               [slotIndex](const LootItem& item) {
                                                   return item.slotIndex == slotIndex;
                                               }),
                                items.end());
                }
            }
        } else {
            addSystemChatMessage("Inventory is full.");
        }
        return;
    }
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = AutostoreLootItemPacket::build(slotIndex);
    socket->send(packet);
}

void GameHandler::closeLoot() {
    if (!lootWindowOpen) return;
    lootWindowOpen = false;
    if (currentLoot.lootGuid != 0 && targetGuid == currentLoot.lootGuid) {
        clearTarget();
    }
    if (singlePlayerMode_ && currentLoot.lootGuid != 0) {
        auto st = localLootState_.find(currentLoot.lootGuid);
        if (st != localLootState_.end()) {
            if (!st->second.moneyTaken && st->second.data.gold > 0) {
                addMoneyCopper(st->second.data.gold);
                st->second.moneyTaken = true;
                st->second.data.gold = 0;
            }
        }
        currentLoot.gold = 0;
        simulateLootRelease();
        return;
    }
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = LootReleasePacket::build(currentLoot.lootGuid);
        socket->send(packet);
    }
    currentLoot = LootResponseData{};
}

void GameHandler::interactWithNpc(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GossipHelloPacket::build(guid);
    socket->send(packet);
}

void GameHandler::selectGossipOption(uint32_t optionId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, currentGossip.menuId, optionId);
    socket->send(packet);
}

void GameHandler::selectGossipQuest(uint32_t questId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    auto packet = QuestgiverQueryQuestPacket::build(currentGossip.npcGuid, questId);
    socket->send(packet);
    gossipWindowOpen = false;
}

void GameHandler::handleQuestDetails(network::Packet& packet) {
    QuestDetailsData data;
    if (!QuestDetailsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_QUEST_DETAILS");
        return;
    }
    currentQuestDetails = data;
    questDetailsOpen = true;
    gossipWindowOpen = false;
}

void GameHandler::acceptQuest() {
    if (!questDetailsOpen || state != WorldState::IN_WORLD || !socket) return;
    uint64_t npcGuid = currentQuestDetails.npcGuid;
    auto packet = QuestgiverAcceptQuestPacket::build(
        npcGuid, currentQuestDetails.questId);
    socket->send(packet);

    // Add to quest log
    bool alreadyInLog = false;
    for (const auto& q : questLog_) {
        if (q.questId == currentQuestDetails.questId) { alreadyInLog = true; break; }
    }
    if (!alreadyInLog) {
        QuestLogEntry entry;
        entry.questId = currentQuestDetails.questId;
        entry.title = currentQuestDetails.title;
        entry.objectives = currentQuestDetails.objectives;
        questLog_.push_back(entry);
    }

    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};

    // Re-query quest giver status so marker updates (! → ?)
    if (npcGuid) {
        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::declineQuest() {
    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};
}

void GameHandler::abandonQuest(uint32_t questId) {
    // Find the quest's index in our local log
    for (size_t i = 0; i < questLog_.size(); i++) {
        if (questLog_[i].questId == questId) {
            // Tell server to remove it (slot index in server quest log)
            // We send the local index; server maps it via PLAYER_QUEST_LOG fields
            if (state == WorldState::IN_WORLD && socket) {
                network::Packet pkt(static_cast<uint16_t>(Opcode::CMSG_QUESTLOG_REMOVE_QUEST));
                pkt.writeUInt8(static_cast<uint8_t>(i));
                socket->send(pkt);
            }
            questLog_.erase(questLog_.begin() + static_cast<ptrdiff_t>(i));
            return;
        }
    }
}

void GameHandler::handleQuestRequestItems(network::Packet& packet) {
    QuestRequestItemsData data;
    if (!QuestRequestItemsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_REQUEST_ITEMS");
        return;
    }
    currentQuestRequestItems_ = data;
    questRequestItemsOpen_ = true;
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for required items
    for (const auto& item : data.requiredItems) {
        queryItemInfo(item.itemId, 0);
    }
}

void GameHandler::handleQuestOfferReward(network::Packet& packet) {
    QuestOfferRewardData data;
    if (!QuestOfferRewardParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_OFFER_REWARD");
        return;
    }
    currentQuestOfferReward_ = data;
    questOfferRewardOpen_ = true;
    questRequestItemsOpen_ = false;
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for reward items
    for (const auto& item : data.choiceRewards)
        queryItemInfo(item.itemId, 0);
    for (const auto& item : data.fixedRewards)
        queryItemInfo(item.itemId, 0);
}

void GameHandler::completeQuest() {
    if (!questRequestItemsOpen_ || state != WorldState::IN_WORLD || !socket) return;
    auto packet = QuestgiverCompleteQuestPacket::build(
        currentQuestRequestItems_.npcGuid, currentQuestRequestItems_.questId);
    socket->send(packet);
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::closeQuestRequestItems() {
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::chooseQuestReward(uint32_t rewardIndex) {
    if (!questOfferRewardOpen_ || state != WorldState::IN_WORLD || !socket) return;
    uint64_t npcGuid = currentQuestOfferReward_.npcGuid;
    auto packet = QuestgiverChooseRewardPacket::build(
        npcGuid, currentQuestOfferReward_.questId, rewardIndex);
    socket->send(packet);
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};

    // Re-query quest giver status so markers update
    if (npcGuid) {
        network::Packet qsPkt(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::closeQuestOfferReward() {
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};
}

void GameHandler::closeGossip() {
    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::openVendor(uint64_t npcGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = ListInventoryPacket::build(npcGuid);
    socket->send(packet);
}

void GameHandler::closeVendor() {
    vendorWindowOpen = false;
    currentVendorItems = ListInventoryData{};
}

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = BuyItemPacket::build(vendorGuid, itemId, slot, count);
    socket->send(packet);
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    socket->send(packet);
}

void GameHandler::sellItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (singlePlayerMode_) {
        if (slot.item.sellPrice > 0) {
            addMoneyCopper(slot.item.sellPrice);
            std::string msg = "You sold " + slot.item.name + ".";
            addSystemChatMessage(msg);
        } else {
            addSystemChatMessage("You can't sell " + slot.item.name + ".");
            return;
        }
        inventory.clearBackpackSlot(backpackIndex);
        notifyInventoryChanged();
    } else {
        uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
        if (itemGuid == 0) {
            itemGuid = resolveOnlineItemGuid(slot.item.itemId);
        }
        LOG_DEBUG("sellItemBySlot: slot=", backpackIndex,
                  " item=", slot.item.name,
                  " itemGuid=0x", std::hex, itemGuid, std::dec,
                  " vendorGuid=0x", std::hex, currentVendorItems.vendorGuid, std::dec);
        if (itemGuid != 0 && currentVendorItems.vendorGuid != 0) {
            sellItem(currentVendorItems.vendorGuid, itemGuid, 1);
        } else if (itemGuid == 0) {
            addSystemChatMessage("Cannot sell: item not found in inventory.");
            LOG_WARNING("Sell failed: missing item GUID for slot ", backpackIndex);
        } else {
            addSystemChatMessage("Cannot sell: no vendor.");
        }
    }
}

void GameHandler::autoEquipItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (singlePlayerMode_) {
        // Fall back to local equip logic (UI already handles this).
        return;
    }

    if (state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = AutoEquipItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex));
        socket->send(packet);
    }
}

void GameHandler::useItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (singlePlayerMode_) {
        // Single-player consumable use not implemented yet.
        return;
    }

    uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }
    if (itemGuid != 0 && state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = UseItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex), itemGuid);
        socket->send(packet);
    } else if (itemGuid == 0) {
        LOG_WARNING("Use item failed: missing item GUID for slot ", backpackIndex);
    }
}

void GameHandler::useItemById(uint32_t itemId) {
    if (itemId == 0) return;
    for (int i = 0; i < inventory.getBackpackSize(); i++) {
        const auto& slot = inventory.getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            useItemBySlot(i);
            return;
        }
    }
}

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;
    if (currentLoot.gold > 0) {
        if (singlePlayerMode_) {
            addMoneyCopper(currentLoot.gold);
            currentLoot.gold = 0;
        } else if (state == WorldState::IN_WORLD && socket) {
            // Auto-loot gold by sending CMSG_LOOT_MONEY (server handles the rest)
            auto pkt = LootMoneyPacket::build();
            socket->send(pkt);
            currentLoot.gold = 0;
        }
    }
}

void GameHandler::handleLootReleaseResponse(network::Packet& packet) {
    (void)packet;
    lootWindowOpen = false;
    currentLoot = LootResponseData{};
}

void GameHandler::handleLootRemoved(network::Packet& packet) {
    uint8_t slotIndex = packet.readUInt8();
    for (auto it = currentLoot.items.begin(); it != currentLoot.items.end(); ++it) {
        if (it->slotIndex == slotIndex) {
            currentLoot.items.erase(it);
            break;
        }
    }
}

void GameHandler::handleGossipMessage(network::Packet& packet) {
    if (!GossipMessageParser::parse(packet, currentGossip)) return;
    if (questDetailsOpen) return; // Don't reopen gossip while viewing quest
    gossipWindowOpen = true;
    vendorWindowOpen = false; // Close vendor if gossip opens
}

void GameHandler::handleGossipComplete(network::Packet& packet) {
    (void)packet;
    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::handleListInventory(network::Packet& packet) {
    if (!ListInventoryParser::parse(packet, currentVendorItems)) return;
    vendorWindowOpen = true;
    gossipWindowOpen = false; // Close gossip if vendor opens

    // Query item info for all vendor items so we can show names
    for (const auto& item : currentVendorItems.items) {
        queryItemInfo(item.itemId, 0);
    }
}

// ============================================================
// Single-player local combat
// ============================================================

void GameHandler::updateLocalCombat(float deltaTime) {
    if (!autoAttacking || autoAttackTarget == 0) return;

    auto entity = entityManager.getEntity(autoAttackTarget);
    if (!entity || entity->getType() != ObjectType::UNIT) {
        stopAutoAttack();
        return;
    }
    auto unit = std::static_pointer_cast<Unit>(entity);
    if (unit->getHealth() == 0) {
        stopAutoAttack();
        return;
    }

    // Check melee range (~8 units squared distance)
    float dx = unit->getX() - movementInfo.x;
    float dy = unit->getY() - movementInfo.y;
    float dz = unit->getZ() - movementInfo.z;
    float distSq = dx * dx + dy * dy + dz * dz;
    if (distSq > 64.0f) return; // 8^2 = 64

    swingTimer_ += deltaTime;
    while (swingTimer_ >= SWING_SPEED) {
        swingTimer_ -= SWING_SPEED;
        performPlayerSwing();
    }
}

void GameHandler::performPlayerSwing() {
    if (autoAttackTarget == 0) return;
    auto entity = entityManager.getEntity(autoAttackTarget);
    if (!entity || entity->getType() != ObjectType::UNIT) return;
    auto unit = std::static_pointer_cast<Unit>(entity);
    if (unit->getHealth() == 0) return;

    if (meleeSwingCallback_) {
        meleeSwingCallback_();
    }

    // Aggro the target
    aggroNpc(autoAttackTarget);

    // 5% miss chance
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> roll(0.0f, 1.0f);
    if (roll(rng) < 0.05f) {
        addCombatText(CombatTextEntry::MISS, 0, 0, true);
        return;
    }

    // Damage calculation
    int32_t baseDamage = 5 + static_cast<int32_t>(localPlayerLevel_) * 3;
    std::uniform_real_distribution<float> dmgRange(0.8f, 1.2f);
    int32_t damage = static_cast<int32_t>(baseDamage * dmgRange(rng));

    // 10% crit chance (2x damage)
    bool crit = roll(rng) < 0.10f;
    if (crit) damage *= 2;

    // Apply damage
    uint32_t hp = unit->getHealth();
    if (static_cast<uint32_t>(damage) >= hp) {
        unit->setHealth(0);
        handleNpcDeath(autoAttackTarget);
    } else {
        unit->setHealth(hp - static_cast<uint32_t>(damage));
    }

    addCombatText(crit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::MELEE_DAMAGE,
                  damage, 0, true);
}

void GameHandler::handleNpcDeath(uint64_t guid) {
    // Award XP from kill
    auto entity = entityManager.getEntity(guid);
    if (entity && entity->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(entity);
        awardLocalXp(guid, unit->getLevel());
    }

    // Remove from aggro list
    aggroList_.erase(
        std::remove_if(aggroList_.begin(), aggroList_.end(),
                       [guid](const NpcAggroEntry& e) { return e.guid == guid; }),
        aggroList_.end());

    // Stop auto-attack if target was this NPC
    if (autoAttackTarget == guid) {
        stopAutoAttack();
    }

    // Notify death callback (plays death animation)
    if (npcDeathCallback_) {
        npcDeathCallback_(guid);
    }
}

void GameHandler::aggroNpc(uint64_t guid) {
    if (!isNpcAggroed(guid)) {
        aggroList_.push_back({guid, 0.0f});
    }
}

bool GameHandler::isNpcAggroed(uint64_t guid) const {
    for (const auto& e : aggroList_) {
        if (e.guid == guid) return true;
    }
    return false;
}

void GameHandler::updateNpcAggro(float deltaTime) {
    // Remove dead/missing NPCs and NPCs out of leash range
    for (auto it = aggroList_.begin(); it != aggroList_.end(); ) {
        auto entity = entityManager.getEntity(it->guid);
        if (!entity || entity->getType() != ObjectType::UNIT) {
            it = aggroList_.erase(it);
            continue;
        }
        auto unit = std::static_pointer_cast<Unit>(entity);
        if (unit->getHealth() == 0) {
            it = aggroList_.erase(it);
            continue;
        }

        // Leash range: 40 units
        float dx = unit->getX() - movementInfo.x;
        float dy = unit->getY() - movementInfo.y;
        float distSq = dx * dx + dy * dy;
        if (distSq > 1600.0f) { // 40^2
            it = aggroList_.erase(it);
            continue;
        }

        // Melee range: 8 units — NPC attacks player
        float dz = unit->getZ() - movementInfo.z;
        float fullDistSq = distSq + dz * dz;
        if (fullDistSq <= 64.0f) { // 8^2
            it->swingTimer += deltaTime;
            if (it->swingTimer >= SWING_SPEED) {
                it->swingTimer -= SWING_SPEED;
                performNpcSwing(it->guid);
            }
        }
        ++it;
    }
}

void GameHandler::performNpcSwing(uint64_t guid) {
    if (localPlayerHealth_ == 0) return;

    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::UNIT) return;
    auto unit = std::static_pointer_cast<Unit>(entity);

    // Auto-target the attacker if player has no current target
    if (targetGuid == 0) {
        setTarget(guid);
    }

    if (npcSwingCallback_) {
        npcSwingCallback_(guid);
    }

    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> roll(0.0f, 1.0f);

    // 5% miss
    if (roll(rng) < 0.05f) {
        addCombatText(CombatTextEntry::MISS, 0, 0, false);
        return;
    }

    // Damage: 3 + npcLevel * 2
    int32_t baseDamage = 3 + static_cast<int32_t>(unit->getLevel()) * 2;
    std::uniform_real_distribution<float> dmgRange(0.8f, 1.2f);
    int32_t damage = static_cast<int32_t>(baseDamage * dmgRange(rng));

    // 5% crit (2x)
    bool crit = roll(rng) < 0.05f;
    if (crit) damage *= 2;

    // Apply to local player health
    if (static_cast<uint32_t>(damage) >= localPlayerHealth_) {
        localPlayerHealth_ = 0;
    } else {
        localPlayerHealth_ -= static_cast<uint32_t>(damage);
    }

    addCombatText(crit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::MELEE_DAMAGE,
                  damage, 0, false);
}

// ============================================================
// XP tracking
// ============================================================

// WotLK 3.3.5a XP-to-next-level table (from player_xp_for_level)
static const uint32_t XP_TABLE[] = {
    0,       // level 0 (unused)
    400,     900,     1400,    2100,    2800,    3600,    4500,    5400,    6500,    7600,     // 1-10
    8700,    9800,    11000,   12300,   13600,   15000,   16400,   17800,   19300,   20800,    // 11-20
    22400,   24000,   25500,   27200,   28900,   30500,   32200,   33900,   36300,   38800,    // 21-30
    41600,   44600,   48000,   51400,   55000,   58700,   62400,   66200,   70200,   74300,    // 31-40
    78500,   82800,   87100,   91600,   96300,   101000,  105800,  110700,  115700,  120900,   // 41-50
    126100,  131500,  137000,  142500,  148200,  154000,  159900,  165800,  172000,  290000,   // 51-60
    317000,  349000,  386000,  428000,  475000,  527000,  585000,  648000,  717000,  1523800,  // 61-70
    1539600, 1555700, 1571800, 1587900, 1604200, 1620700, 1637400, 1653900, 1670800           // 71-79
};
static constexpr uint32_t XP_TABLE_SIZE = sizeof(XP_TABLE) / sizeof(XP_TABLE[0]);

uint32_t GameHandler::xpForLevel(uint32_t level) {
    if (level == 0 || level >= XP_TABLE_SIZE) return 0;
    return XP_TABLE[level];
}

uint32_t GameHandler::killXp(uint32_t playerLevel, uint32_t victimLevel) {
    if (playerLevel == 0 || victimLevel == 0) return 0;

    // Gray level check (too low = 0 XP)
    int32_t grayLevel;
    if (playerLevel <= 5)        grayLevel = 0;
    else if (playerLevel <= 39)  grayLevel = static_cast<int32_t>(playerLevel) - 5 - static_cast<int32_t>(playerLevel) / 10;
    else if (playerLevel <= 59)  grayLevel = static_cast<int32_t>(playerLevel) - 1 - static_cast<int32_t>(playerLevel) / 5;
    else                         grayLevel = static_cast<int32_t>(playerLevel) - 9;

    if (static_cast<int32_t>(victimLevel) <= grayLevel) return 0;

    // Base XP = 45 + 5 * victimLevel (WoW-like ZeroDifference formula)
    uint32_t baseXp = 45 + 5 * victimLevel;

    // Level difference multiplier
    int32_t diff = static_cast<int32_t>(victimLevel) - static_cast<int32_t>(playerLevel);
    float multiplier = 1.0f + diff * 0.05f;
    if (multiplier < 0.1f) multiplier = 0.1f;
    if (multiplier > 2.0f) multiplier = 2.0f;

    return static_cast<uint32_t>(baseXp * multiplier);
}

void GameHandler::awardLocalXp(uint64_t victimGuid, uint32_t victimLevel) {
    if (localPlayerLevel_ >= 80) return; // Level cap

    uint32_t xp = killXp(localPlayerLevel_, victimLevel);
    if (xp == 0) return;

    playerXp_ += xp;
    markSinglePlayerDirty(SP_DIRTY_XP, true);

    // Show XP gain in combat text as a heal-type (gold text)
    addCombatText(CombatTextEntry::HEAL, static_cast<int32_t>(xp), 0, true);
    simulateXpGain(victimGuid, xp);

    LOG_INFO("XP gained: +", xp, " (total: ", playerXp_, "/", playerNextLevelXp_, ")");

    // Check for level-up
    while (playerXp_ >= playerNextLevelXp_ && localPlayerLevel_ < 80) {
        playerXp_ -= playerNextLevelXp_;
        levelUp();
    }
}

void GameHandler::levelUp() {
    localPlayerLevel_++;
    playerNextLevelXp_ = xpForLevel(localPlayerLevel_);

    // Scale HP with level
    uint32_t newMaxHp = 20 + localPlayerLevel_ * 10;
    localPlayerMaxHealth_ = newMaxHp;
    localPlayerHealth_ = newMaxHp; // Full heal on level-up
    markSinglePlayerDirty(SP_DIRTY_STATS | SP_DIRTY_XP, true);

    LOG_INFO("LEVEL UP! Now level ", localPlayerLevel_,
             " (HP: ", newMaxHp, ", next level: ", playerNextLevelXp_, " XP)");

    // Announce in chat
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "You have reached level " + std::to_string(localPlayerLevel_) + "!";
    addLocalChatMessage(msg);
}

void GameHandler::handleXpGain(network::Packet& packet) {
    XpGainData data;
    if (!XpGainParser::parse(packet, data)) return;

    // Server already updates PLAYER_XP via update fields,
    // but we can show combat text for XP gains
    addCombatText(CombatTextEntry::HEAL, static_cast<int32_t>(data.totalXp), 0, true);

    std::string msg = "You gain " + std::to_string(data.totalXp) + " experience.";
    if (data.groupBonus > 0) {
        msg += " (+" + std::to_string(data.groupBonus) + " group bonus)";
    }
    addSystemChatMessage(msg);
}

LootResponseData GameHandler::generateLocalLoot(uint64_t guid) {
    LootResponseData data;
    data.lootGuid = guid;
    data.lootType = 0;
    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::UNIT) return data;
    auto unit = std::static_pointer_cast<Unit>(entity);
    uint32_t entry = unit->getEntry();
    if (entry == 0) return data;

    auto& db = getSinglePlayerLootDb();

    uint32_t lootId = entry;
    auto itTemplate = db.creatureTemplates.find(entry);
    if (itTemplate != db.creatureTemplates.end()) {
        if (itTemplate->second.lootId != 0) lootId = itTemplate->second.lootId;
        if (itTemplate->second.maxGold > 0) {
            std::uniform_int_distribution<uint32_t> goldDist(
                itTemplate->second.minGold, itTemplate->second.maxGold);
            static std::mt19937 rng(std::random_device{}());
            data.gold = goldDist(rng);
        }
    }

    auto itLoot = db.creatureLoot.find(lootId);
    if (itLoot == db.creatureLoot.end() && lootId != entry) {
        itLoot = db.creatureLoot.find(entry);
    }
    if (itLoot == db.creatureLoot.end()) return data;

    std::unordered_map<uint8_t, std::vector<LootEntryRow>> groups;
    std::vector<LootEntryRow> ungroupped;
    for (const auto& row : itLoot->second) {
        if (row.groupid == 0) {
            ungroupped.push_back(row);
        } else {
            groups[row.groupid].push_back(row);
        }
    }

    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> roll(0.0f, 100.0f);

    auto addItem = [&](uint32_t itemId, uint32_t count) {
        LootItem li;
        li.slotIndex = static_cast<uint8_t>(data.items.size());
        li.itemId = itemId;
        li.count = count;
        auto itItem = db.itemTemplates.find(itemId);
        if (itItem != db.itemTemplates.end()) {
            li.displayInfoId = itItem->second.displayId;
        }
        data.items.push_back(li);
    };

    std::function<void(const std::vector<LootEntryRow>&, bool)> processLootTable;
    processLootTable = [&](const std::vector<LootEntryRow>& rows, bool grouped) {
        if (rows.empty()) return;
        if (grouped) {
            float total = 0.0f;
            for (const auto& r : rows) total += std::abs(r.chance);
            if (total <= 0.0f) return;
            float r = roll(rng);
            if (total < 100.0f && r > total) return;
            float pick = (total < 100.0f)
                ? r
                : std::uniform_real_distribution<float>(0.0f, total)(rng);
            float acc = 0.0f;
            for (const auto& row : rows) {
                acc += std::abs(row.chance);
                if (pick <= acc) {
                    if (row.mincountOrRef < 0) {
                        auto refIt = db.referenceLoot.find(static_cast<uint32_t>(-row.mincountOrRef));
                        if (refIt != db.referenceLoot.end()) {
                            processLootTable(refIt->second, false);
                        }
                    } else {
                        uint32_t minc = static_cast<uint32_t>(std::max(1, row.mincountOrRef));
                        uint32_t maxc = std::max(minc, static_cast<uint32_t>(row.maxcount));
                        std::uniform_int_distribution<uint32_t> cnt(minc, maxc);
                        addItem(row.item, cnt(rng));
                    }
                    break;
                }
            }
            return;
        }

        for (const auto& row : rows) {
            float chance = std::abs(row.chance);
            if (chance <= 0.0f) continue;
            if (roll(rng) > chance) continue;
            if (row.mincountOrRef < 0) {
                auto refIt = db.referenceLoot.find(static_cast<uint32_t>(-row.mincountOrRef));
                if (refIt != db.referenceLoot.end()) {
                    processLootTable(refIt->second, false);
                }
                continue;
            }
            uint32_t minc = static_cast<uint32_t>(std::max(1, row.mincountOrRef));
            uint32_t maxc = std::max(minc, static_cast<uint32_t>(row.maxcount));
            std::uniform_int_distribution<uint32_t> cnt(minc, maxc);
            addItem(row.item, cnt(rng));
        }
    };

    processLootTable(ungroupped, false);
    for (const auto& [gid, rows] : groups) {
        processLootTable(rows, true);
    }

    return data;
}

void GameHandler::simulateLootResponse(const LootResponseData& data) {
    network::Packet packet(static_cast<uint16_t>(Opcode::SMSG_LOOT_RESPONSE));
    packet.writeUInt64(data.lootGuid);
    packet.writeUInt8(data.lootType);
    packet.writeUInt32(data.gold);
    packet.writeUInt8(static_cast<uint8_t>(data.items.size()));
    for (const auto& item : data.items) {
        packet.writeUInt8(item.slotIndex);
        packet.writeUInt32(item.itemId);
        packet.writeUInt32(item.count);
        packet.writeUInt32(item.displayInfoId);
        packet.writeUInt32(item.randomSuffix);
        packet.writeUInt32(item.randomPropertyId);
        packet.writeUInt8(item.lootSlotType);
    }
    handleLootResponse(packet);
}

void GameHandler::simulateLootRelease() {
    network::Packet packet(static_cast<uint16_t>(Opcode::SMSG_LOOT_RELEASE_RESPONSE));
    handleLootReleaseResponse(packet);
    currentLoot = LootResponseData{};
}

void GameHandler::simulateLootRemove(uint8_t slotIndex) {
    if (!lootWindowOpen) return;
    network::Packet packet(static_cast<uint16_t>(Opcode::SMSG_LOOT_REMOVED));
    packet.writeUInt8(slotIndex);
    handleLootRemoved(packet);
}

void GameHandler::simulateXpGain(uint64_t victimGuid, uint32_t totalXp) {
    network::Packet packet(static_cast<uint16_t>(Opcode::SMSG_LOG_XPGAIN));
    packet.writeUInt64(victimGuid);
    packet.writeUInt32(totalXp);
    packet.writeUInt8(0); // kill XP
    packet.writeFloat(1.0f); // group rate (1.0 = solo, no bonus)
    packet.writeUInt8(0);    // RAF flag
    handleXpGain(packet);
}

void GameHandler::simulateMotd(const std::vector<std::string>& lines) {
    network::Packet packet(static_cast<uint16_t>(Opcode::SMSG_MOTD));
    packet.writeUInt32(static_cast<uint32_t>(lines.size()));
    for (const auto& line : lines) {
        packet.writeString(line);
    }
    handleMotd(packet);
}

void GameHandler::addMoneyCopper(uint32_t amount) {
    if (amount == 0) return;
    playerMoneyCopper_ += amount;
    markSinglePlayerDirty(SP_DIRTY_MONEY, true);
    uint32_t gold = amount / 10000;
    uint32_t silver = (amount / 100) % 100;
    uint32_t copper = amount % 100;
    std::string msg = "You receive ";
    msg += std::to_string(gold) + "g ";
    msg += std::to_string(silver) + "s ";
    msg += std::to_string(copper) + "c.";
    addSystemChatMessage(msg);
}

void GameHandler::addSystemChatMessage(const std::string& message) {
    if (message.empty()) return;
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = message;
    addLocalChatMessage(msg);
}

uint32_t GameHandler::generateClientSeed() {
    // Generate cryptographically random seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(1, 0xFFFFFFFF);
    return dis(gen);
}

void GameHandler::setState(WorldState newState) {
    if (state != newState) {
        LOG_DEBUG("World state: ", (int)state, " -> ", (int)newState);
        state = newState;
    }
}

void GameHandler::fail(const std::string& reason) {
    LOG_ERROR("World connection failed: ", reason);
    setState(WorldState::FAILED);

    if (onFailure) {
        onFailure(reason);
    }
}

std::string GameHandler::getItemTemplateName(uint32_t itemId) const {
    auto& db = getSinglePlayerLootDb();
    auto it = db.itemTemplates.find(itemId);
    if (it != db.itemTemplates.end()) return it->second.name;
    return {};
}

ItemQuality GameHandler::getItemTemplateQuality(uint32_t itemId) const {
    auto& db = getSinglePlayerLootDb();
    auto it = db.itemTemplates.find(itemId);
    if (it != db.itemTemplates.end()) return static_cast<ItemQuality>(it->second.quality);
    return ItemQuality::COMMON;
}

} // namespace game
} // namespace wowee
