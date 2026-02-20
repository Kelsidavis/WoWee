#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <optional>

namespace wowee {
namespace game {

/**
 * Logical opcode identifiers (expansion-agnostic).
 *
 * These are compile-time enum values used in switch statements.
 * The actual wire values depend on the active expansion and are
 * loaded from JSON at runtime via OpcodeTable.
 */
enum class LogicalOpcode : uint16_t {
    // Generated from Data/opcodes/canonical.json
#include "game/opcode_enum_generated.inc"

    // Sentinel
    COUNT
};

/**
 * Maps LogicalOpcode ↔ expansion-specific wire values.
 *
 * Loaded from JSON (e.g. Data/expansions/wotlk/opcodes.json).
 * Used for sending packets (toWire) and receiving them (fromWire).
 */
class OpcodeTable {
public:
    /**
     * Load opcode mappings from a JSON file.
     * Format: { "CMSG_PING": "0x1DC", "SMSG_AUTH_CHALLENGE": "0x1EC", ... }
     */
    bool loadFromJson(const std::string& path);

    /** LogicalOpcode → wire value for sending packets. Returns 0xFFFF if unknown. */
    uint16_t toWire(LogicalOpcode op) const;

    /** Wire value → LogicalOpcode for receiving packets. Returns nullopt if unknown. */
    std::optional<LogicalOpcode> fromWire(uint16_t wireValue) const;

    /** Check if a logical opcode has a wire mapping. */
    bool hasOpcode(LogicalOpcode op) const;

    /** Number of mapped opcodes. */
    size_t size() const { return logicalToWire_.size(); }

private:
    std::unordered_map<uint16_t, uint16_t> logicalToWire_;   // LogicalOpcode → wire
    std::unordered_map<uint16_t, uint16_t> wireToLogical_;   // wire → LogicalOpcode

    static std::optional<LogicalOpcode> nameToLogical(const std::string& name);
    static const char* logicalToName(LogicalOpcode op);
};

/**
 * Global active opcode table pointer (set by GameHandler at startup).
 * Used by world_packets.cpp and other code that needs to send packets
 * without direct access to a GameHandler instance.
 */
void setActiveOpcodeTable(const OpcodeTable* table);
const OpcodeTable* getActiveOpcodeTable();

/**
 * Get the wire value for a logical opcode using the active table.
 * Convenience helper for packet construction code.
 */
inline uint16_t wireOpcode(LogicalOpcode op) {
    const auto* table = getActiveOpcodeTable();
    return table ? table->toWire(op) : 0xFFFF;
}

} // namespace game
} // namespace wowee
