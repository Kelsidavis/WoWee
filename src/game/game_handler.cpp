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
    std::vector<StartItemRow> items;
    std::vector<StartSpellRow> spells;
};

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
    if (!std::filesystem::exists(itemPath) || !std::filesystem::exists(spellPath)) {
        auto alt = basePath / "base";
        if (std::filesystem::exists(alt / "playercreateinfo_item.sql")) {
            basePath = alt;
            itemPath = basePath / "playercreateinfo_item.sql";
            spellPath = basePath / "playercreateinfo_spell.sql";
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

    db.loaded = true;
    LOG_INFO("Single-player start DB loaded (items=", db.items.size(),
             ", spells=", db.spells.size(), ")");
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
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
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

        // Single-player local combat
        if (singlePlayerMode_) {
            updateLocalCombat(deltaTime);
            updateNpcAggro(deltaTime);
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
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleUpdateObject(packet);
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

        // ---- XP ----
        case Opcode::SMSG_LOG_XPGAIN:
            handleXpGain(packet);
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
        case Opcode::SMSG_LOOT_MONEY_NOTIFY:
        case Opcode::SMSG_LOOT_CLEAR_MONEY:
        case Opcode::SMSG_NPC_TEXT_UPDATE:
        case Opcode::SMSG_SELL_ITEM:
        case Opcode::SMSG_BUY_FAILED:
        case Opcode::SMSG_INVENTORY_CHANGE_FAILURE:
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
        case Opcode::MSG_RAID_TARGET_UPDATE:
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

    // Send packet (NOT encrypted yet)
    socket->send(packet);

    // CRITICAL: Initialize encryption AFTER sending AUTH_SESSION
    // but BEFORE receiving AUTH_RESPONSE
    LOG_INFO("Initializing RC4 header encryption...");
    socket->initEncryption(sessionKey);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption initialized, waiting for response...");
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

    // Authentication successful!
    setState(WorldState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   WORLD AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("Connected to world server");
    LOG_INFO("Ready for character operations");

    setState(WorldState::READY);

    // Call success callback
    if (onSuccess) {
        onSuccess();
    }
}

void GameHandler::requestCharacterList() {
    if (state != WorldState::READY && state != WorldState::AUTHENTICATED) {
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
        ch.guid = 0x0000000100000001ULL + characters.size();
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
            case CharCreateResult::NAME_IN_USE: msg = "Name already in use"; break;
            case CharCreateResult::DISABLED: msg = "Character creation disabled"; break;
            case CharCreateResult::SERVER_LIMIT: msg = "Server character limit reached"; break;
            case CharCreateResult::ACCOUNT_LIMIT: msg = "Account character limit reached"; break;
            default: msg = "Character creation failed"; break;
        }
        LOG_WARNING("Character creation failed: ", msg);
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
    }
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
    setState(WorldState::CHAR_LIST_RECEIVED);
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
        } else {
            def.name = "Item " + std::to_string(row.itemId);
        }

        inventory.addItem(def);
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

    // Auto-populate action bar with known spells
    int slot = 1;
    for (uint32_t spellId : knownSpells) {
        if (spellId == 6603 || spellId == 8690) continue;
        if (slot >= 11) break;
        actionBar[slot].type = ActionBarSlot::SPELL;
        actionBar[slot].id = spellId;
        slot++;
    }
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

    // Update movement time
    movementInfo.time = ++movementTime;

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
    auto packet = MovementPacket::build(opcode, wireInfo);
    socket->send(packet);
}

void GameHandler::setPosition(float x, float y, float z) {
    movementInfo.x = x;
    movementInfo.y = y;
    movementInfo.z = z;
}

void GameHandler::setOrientation(float orientation) {
    movementInfo.orientation = orientation;
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
                        queryCreatureInfo(it->second, block.guid);
                    }
                }

                // Extract health/mana/power from fields (Phase 2) — single pass
                if (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) {
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    for (const auto& [key, val] : block.fields) {
                        switch (key) {
                            case 24: unit->setHealth(val); break;
                            case 25: unit->setPower(val); break;
                            case 32: unit->setMaxHealth(val); break;
                            case 33: unit->setMaxPower(val); break;
                            case 54: unit->setLevel(val); break;
                            default: break;
                        }
                    }
                }
                // Extract XP fields for player entity
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    for (const auto& [key, val] : block.fields) {
                        switch (key) {
                            case 634: playerXp_ = val; break;           // PLAYER_XP
                            case 635: playerNextLevelXp_ = val; break;  // PLAYER_NEXT_LEVEL_XP
                            case 54:  serverPlayerLevel_ = val; break;  // UNIT_FIELD_LEVEL
                            default: break;
                        }
                    }
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
                                case 24: unit->setHealth(val); break;
                                case 25: unit->setPower(val); break;
                                case 32: unit->setMaxHealth(val); break;
                                case 33: unit->setMaxPower(val); break;
                                case 54: unit->setLevel(val); break;
                                default: break;
                            }
                        }
                    }
                    // Update XP fields for player entity
                    if (block.guid == playerGuid) {
                        for (const auto& [key, val] : block.fields) {
                            switch (key) {
                                case 634: playerXp_ = val; break;           // PLAYER_XP
                                case 635: playerNextLevelXp_ = val; break;  // PLAYER_NEXT_LEVEL_XP
                                case 54:  serverPlayerLevel_ = val; break;  // UNIT_FIELD_LEVEL
                                default: break;
                            }
                        }
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
// Phase 2: Combat
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    autoAttacking = true;
    autoAttackTarget = targetGuid;
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

    if (data.attackerGuid == playerGuid) {
        autoAttacking = false;
        autoAttackTarget = 0;
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

    uint64_t target = targetGuid != 0 ? targetGuid : targetGuid;
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

    // Auto-populate action bar: Attack in slot 1, Hearthstone in slot 12, rest filled with known spells
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone
    int slot = 1;
    for (int i = 0; i < static_cast<int>(knownSpells.size()) && slot < 11; ++i) {
        if (knownSpells[i] == 6603 || knownSpells[i] == 8690) continue;
        actionBar[slot].type = ActionBarSlot::SPELL;
        actionBar[slot].id = knownSpells[i];
        slot++;
    }

    LOG_INFO("Learned ", knownSpells.size(), " spells");
}

void GameHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    if (!CastFailedParser::parse(packet, data)) return;

    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Add system message about failed cast
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "Spell cast failed (error " + std::to_string(data.result) + ")";
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
}

void GameHandler::handleLearnedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.push_back(spellId);
    LOG_INFO("Learned spell: ", spellId);
}

void GameHandler::handleRemovedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.erase(
        std::remove(knownSpells.begin(), knownSpells.end(), spellId),
        knownSpells.end());
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
        } else {
            def.name = "Item " + std::to_string(it->itemId);
        }

        if (inventory.addItem(def)) {
            simulateLootRemove(slotIndex);
            addSystemChatMessage("You receive item: " + def.name + " x" + std::to_string(def.stackCount) + ".");
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
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, optionId);
    socket->send(packet);
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

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = BuyItemPacket::build(vendorGuid, itemId, slot, count);
    socket->send(packet);
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    socket->send(packet);
}

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;
    if (currentLoot.gold > 0) {
        std::string msg = "You loot ";
        msg += std::to_string(currentLoot.getGold()) + "g ";
        msg += std::to_string(currentLoot.getSilver()) + "s ";
        msg += std::to_string(currentLoot.getCopper()) + "c.";
        addSystemChatMessage(msg);
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
    packet.writeFloat(0.0f);
    packet.writeUInt32(0); // group bonus
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

} // namespace game
} // namespace wowee
