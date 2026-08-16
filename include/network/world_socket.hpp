#pragma once

#include "network/socket.hpp"
#include "network/packet.hpp"
#include "network/net_platform.hpp"
#include "auth/rc4.hpp"
#include "auth/vanilla_crypt.hpp"
#include <functional>
#include <vector>
#include <deque>
#include <cstdint>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <array>

namespace wowee {
namespace network {

/**
 * World Server Socket
 *
 * Handles WoW world server protocol with header encryption.
 * Supports vanilla/classic raw XOR, CMaNGOS TBC HMAC-derived XOR, and WotLK RC4.
 *
 * Key Differences from Auth Server:
 * - Outgoing: 6-byte header (2 bytes size + 4 bytes opcode, big-endian)
 * - Incoming: 4-byte header (2 bytes size + 2 bytes opcode)
 * - Headers are encrypted after CMSG_AUTH_SESSION
 * - Packet bodies remain unencrypted
 * - Size field includes opcode bytes (payloadLen = size - 2)
 */
class WorldSocket : public Socket {
public:
    WorldSocket();
    ~WorldSocket() override;

    bool connect(const std::string& host, uint16_t port) override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    /**
     * Send a world packet
     * Automatically encrypts 6-byte header if encryption is enabled
     *
     * @param packet Packet to send
     */
    void send(const Packet& packet) override;

    /**
     * Update socket - receive data and parse packets
     * Should be called regularly (e.g., each frame)
     */
    void update() override;

    /**
     * Set callback for complete packets
     *
     * @param callback Function to call when packet is received
     */
    void setPacketCallback(std::function<void(const Packet&)> callback) {
        packetCallback = callback;
    }

    /**
     * Initialize header encryption for packet headers
     * Must be called after CMSG_AUTH_SESSION before further communication
     *
     * @param sessionKey 40-byte session key from auth server
     * @param build Client build number (determines cipher family)
     */
    void initEncryption(const std::vector<uint8_t>& sessionKey, uint32_t build = 12340);

    void tracePacketsFor(std::chrono::milliseconds duration, const std::string& reason);

    /**
     * Check if header encryption is enabled
     */
    [[nodiscard]] bool isEncryptionEnabled() const { return encryptionEnabled; }

private:
    /**
     * Try to parse complete packets from receive buffer
     */
    void tryParsePackets();
    void pumpNetworkIO();
    void dispatchQueuedPackets();
    void asyncPumpLoop();
    void startAsyncPump();
    void stopAsyncPump();
    void closeSocketNoJoin();
    void recordRecentPacket(bool outbound, uint16_t opcode, uint16_t payloadLen);
    void dumpRecentPacketHistoryLocked(const char* reason, size_t bufferedBytes);

    socket_t sockfd = INVALID_SOCK;           // THREAD-SAFE: protected by ioMutex_
    // Read every frame by GameHandler while the async receive pump may hold
    // ioMutex_ for a packet burst. Atomic state avoids turning that harmless
    // status check into a 50ms main-thread mutex stall.
    std::atomic<bool> connected{false};
    bool encryptionEnabled = false;            // THREAD-SAFE: protected by ioMutex_
    bool useVanillaCrypt = false;  // true = XOR cipher, false = RC4
    bool useAsyncPump_ = true;
    std::thread asyncPumpThread_;
    std::atomic<bool> asyncPumpStop_{false};   // THREAD-SAFE: atomic
    std::atomic<bool> asyncPumpRunning_{false}; // THREAD-SAFE: atomic
    // Guards sockfd, encryptionEnabled, receiveBuffer, cipher state,
    // headerBytesDecrypted, and recentPacketHistory_.
    mutable std::mutex ioMutex_;
    // Guards pendingPacketCallbacks_ (asyncPumpThread_ produces, main thread consumes).
    mutable std::mutex callbackMutex_;

    // WotLK RC4 ciphers for header encryption/decryption
    auth::RC4 encryptCipher;
    auth::RC4 decryptCipher;

    // Vanilla/TBC XOR+addition cipher
    auth::VanillaCrypt vanillaCrypt;

    // THREAD-SAFE: protected by ioMutex_
    std::vector<uint8_t> receiveBuffer;
    size_t receiveReadOffset_ = 0;
    // Optional reused packet queue (feature-gated) to reduce per-update allocations.
    std::vector<Packet> parsedPacketsScratch_;
    // THREAD-SAFE: protected by callbackMutex_.
    // Parsed packets waiting for callback dispatch; drained with a strict per-update budget.
    std::deque<Packet> pendingPacketCallbacks_;

    // Runtime-gated network optimization toggles (default off).
    bool useFastRecvAppend_ = false;
    bool useParseScratchQueue_ = false;

    // Track how many header bytes have been decrypted (0-4)
    // This prevents re-decrypting the same header when waiting for more data
    size_t headerBytesDecrypted = 0;

    // Debug-only tracing window for post-auth packet framing verification.
    int headerTracePacketsLeft = 0;
    std::chrono::steady_clock::time_point packetTraceStart_{};
    std::chrono::steady_clock::time_point packetTraceUntil_{};
    std::string packetTraceReason_;

    struct RecentPacketTrace {
        std::chrono::steady_clock::time_point when{};
        bool outbound = false;
        uint16_t opcode = 0;
        uint16_t payloadLen = 0;
    };
    std::deque<RecentPacketTrace> recentPacketHistory_;

    // Packet callback
    std::function<void(const Packet&)> packetCallback;
};

} // namespace network
} // namespace wowee
