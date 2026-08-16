#pragma once

#include "game/opcode_table.hpp"
#include "network/packet.hpp"
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;
class WardenCrypto;
class WardenMemory;
class WardenModule;
class WardenModuleManager;

class WardenHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit WardenHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Public API ---

    /** Reset all warden state (called on connect / disconnect). */
    void reset();

    /** Initialize warden module manager (called once from GameHandler ctor). */
    void initModuleManager();

    /** Whether the server requires Warden (gates char enum / create). */
    [[nodiscard]] bool requiresWarden() const { return requiresWarden_; }
    void setRequiresWarden(bool v) { requiresWarden_ = v; }

    [[nodiscard]] bool wardenGateSeen() const { return wardenGateSeen_; }

    /** Increment packet-after-gate counter (called from handlePacket). */
    void notifyPacketAfterGate() { ++wardenPacketsAfterGate_; }

    [[nodiscard]] bool wardenCharEnumBlockedLogged() const { return wardenCharEnumBlockedLogged_; }
    void setWardenCharEnumBlockedLogged(bool v) { wardenCharEnumBlockedLogged_ = v; }

    /** Called from GameHandler::update() to drain async warden response + log gate timing. */
    void update(float deltaTime);

private:
    void handleWardenData(network::Packet& packet);
    bool loadWardenCRFile(const std::string& moduleHashHex);

    GameHandler& owner_;

    // --- Warden state ---
    bool requiresWarden_ = false;
    bool wardenGateSeen_ = false;
    float wardenGateElapsed_ = 0.0f;
    float wardenGateNextStatusLog_ = 2.0f;
    uint32_t wardenPacketsAfterGate_ = 0;
    bool wardenCharEnumBlockedLogged_ = false;
    std::unique_ptr<WardenCrypto> wardenCrypto_;
    std::unique_ptr<WardenMemory> wardenMemory_;
    std::unique_ptr<WardenModuleManager> wardenModuleManager_;

    // Warden module download state
    enum class WardenState {
        WAIT_MODULE_USE,     // Waiting for first SMSG (MODULE_USE)
        WAIT_MODULE_CACHE,   // Sent MODULE_MISSING, receiving module chunks
        WAIT_HASH_REQUEST,   // Module received, waiting for HASH_REQUEST
        WAIT_CHECKS,         // Hash sent, waiting for check requests
    };
    WardenState wardenState_ = WardenState::WAIT_MODULE_USE;
    std::vector<uint8_t> wardenModuleHash_;    // 16 bytes MD5
    std::vector<uint8_t> wardenModuleKey_;     // 16 bytes RC4
    uint32_t wardenModuleSize_ = 0;
    std::vector<uint8_t> wardenModuleData_;    // Downloaded module chunks
    std::shared_ptr<WardenModule> wardenLoadedModule_; // Loaded Warden module

    // Pre-computed challenge/response entries from .cr file
    struct WardenCREntry {
        uint8_t seed[16];
        uint8_t reply[20];
        uint8_t clientKey[16];  // Encrypt key (client→server)
        uint8_t serverKey[16]; // Decrypt key (server→client)
    };
    std::vector<WardenCREntry> wardenCREntries_;
    // Module-specific check type opcodes [9]: MEM, PAGE_A, PAGE_B, MPQ, LUA, DRIVER, TIMING, PROC, MODULE
    uint8_t wardenCheckOpcodes_[9] = {};

    // Async Warden response: avoids 5-second main-loop stalls from PAGE_A/PAGE_B code pattern searches
    std::future<std::vector<uint8_t>> wardenPendingEncrypted_;  // encrypted response bytes
    bool wardenResponsePending_ = false;
};

} // namespace game
} // namespace wowee
