#include "game/opcode_table.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <string_view>

namespace wowee {
namespace game {

// Global active opcode table pointer
static const OpcodeTable* g_activeOpcodeTable = nullptr;

void setActiveOpcodeTable(const OpcodeTable* table) { g_activeOpcodeTable = table; }
const OpcodeTable* getActiveOpcodeTable() { return g_activeOpcodeTable; }

// Name ↔ LogicalOpcode mapping table (generated from the enum)
struct OpcodeNameEntry {
    const char* name;
    LogicalOpcode op;
};

// Expansion/core naming aliases -> canonical LogicalOpcode names used by implementation.
struct OpcodeAliasEntry {
    const char* alias;
    const char* canonical;
};

// clang-format off
static const OpcodeAliasEntry kOpcodeAliases[] = {
#include "game/opcode_aliases_generated.inc"
};

static const OpcodeNameEntry kOpcodeNames[] = {
#include "game/opcode_names_generated.inc"
};
// clang-format on

static constexpr size_t kOpcodeNameCount = sizeof(kOpcodeNames) / sizeof(kOpcodeNames[0]);
static constexpr size_t kOpcodeAliasCount = sizeof(kOpcodeAliases) / sizeof(kOpcodeAliases[0]);

static std::string_view canonicalOpcodeName(std::string_view name) {
    for (size_t i = 0; i < kOpcodeAliasCount; ++i) {
        if (name == kOpcodeAliases[i].alias) return kOpcodeAliases[i].canonical;
    }
    return name;
}

std::optional<LogicalOpcode> OpcodeTable::nameToLogical(const std::string& name) {
    const std::string_view canonical = canonicalOpcodeName(name);
    for (size_t i = 0; i < kOpcodeNameCount; ++i) {
        if (canonical == kOpcodeNames[i].name) return kOpcodeNames[i].op;
    }
    return std::nullopt;
}

const char* OpcodeTable::logicalToName(LogicalOpcode op) {
    uint16_t val = static_cast<uint16_t>(op);
    for (size_t i = 0; i < kOpcodeNameCount; ++i) {
        if (static_cast<uint16_t>(kOpcodeNames[i].op) == val) return kOpcodeNames[i].name;
    }
    return "UNKNOWN";
}

bool OpcodeTable::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("OpcodeTable: cannot open ", path, ", using defaults");
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    // Start fresh — JSON is the single source of truth for opcode mappings.
    logicalToWire_.clear();
    wireToLogical_.clear();

    // Parse simple JSON: { "NAME": "0xHEX", ... } or { "NAME": 123, ... }
    size_t pos = 0;
    size_t loaded = 0;
    while (pos < json.size()) {
        // Find next quoted key
        size_t keyStart = json.find('"', pos);
        if (keyStart == std::string::npos) break;
        size_t keyEnd = json.find('"', keyStart + 1);
        if (keyEnd == std::string::npos) break;
        std::string key = json.substr(keyStart + 1, keyEnd - keyStart - 1);

        // Find colon then value
        size_t colon = json.find(':', keyEnd);
        if (colon == std::string::npos) break;

        // Skip whitespace
        size_t valStart = colon + 1;
        while (valStart < json.size() && (json[valStart] == ' ' || json[valStart] == '\t' ||
               json[valStart] == '\r' || json[valStart] == '\n' || json[valStart] == '"'))
            ++valStart;

        size_t valEnd = json.find_first_of(",}\"\r\n", valStart);
        if (valEnd == std::string::npos) valEnd = json.size();
        std::string valStr = json.substr(valStart, valEnd - valStart);

        // Parse hex or decimal value
        uint16_t wire = 0;
        try {
            if (valStr.size() > 2 && (valStr[0] == '0' && (valStr[1] == 'x' || valStr[1] == 'X'))) {
                wire = static_cast<uint16_t>(std::stoul(valStr, nullptr, 16));
            } else {
                wire = static_cast<uint16_t>(std::stoul(valStr));
            }
        } catch (...) {
            pos = valEnd + 1;
            continue;
        }

        auto logOp = nameToLogical(key);
        if (logOp) {
            uint16_t logIdx = static_cast<uint16_t>(*logOp);
            logicalToWire_[logIdx] = wire;
            wireToLogical_[wire] = logIdx;
            ++loaded;
        }

        pos = valEnd + 1;
    }

    if (loaded == 0) {
        LOG_WARNING("OpcodeTable: no opcodes loaded from ", path);
        return false;
    }

    LOG_INFO("OpcodeTable: loaded ", loaded, " opcodes from ", path);
    return true;
}

uint16_t OpcodeTable::toWire(LogicalOpcode op) const {
    auto it = logicalToWire_.find(static_cast<uint16_t>(op));
    return (it != logicalToWire_.end()) ? it->second : 0xFFFF;
}

std::optional<LogicalOpcode> OpcodeTable::fromWire(uint16_t wireValue) const {
    auto it = wireToLogical_.find(wireValue);
    if (it != wireToLogical_.end()) {
        return static_cast<LogicalOpcode>(it->second);
    }
    return std::nullopt;
}

bool OpcodeTable::hasOpcode(LogicalOpcode op) const {
    return logicalToWire_.count(static_cast<uint16_t>(op)) > 0;
}

} // namespace game
} // namespace wowee
