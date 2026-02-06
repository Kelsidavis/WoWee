#pragma once

#include "network/socket.hpp"
#include "network/packet.hpp"
#include "network/net_platform.hpp"
#include "auth/rc4.hpp"
#include <functional>
#include <vector>
#include <cstdint>

namespace wowee {
namespace network {

/**
 * World Server Socket
 *
 * Handles WoW 3.3.5a world server protocol with RC4 header encryption.
 *
 * Key Differences from Auth Server:
 * - Outgoing: 6-byte header (2 bytes size + 4 bytes opcode, big-endian)
 * - Incoming: 4-byte header (2 bytes size + 2 bytes opcode, big-endian)
 * - Headers are RC4-encrypted after CMSG_AUTH_SESSION
 * - Packet bodies remain unencrypted
 * - Size field is payload size only (does NOT include header)
 */
class WorldSocket : public Socket {
public:
    WorldSocket();
    ~WorldSocket() override;

    bool connect(const std::string& host, uint16_t port) override;
    void disconnect() override;
    bool isConnected() const override;

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
    void update();

    /**
     * Set callback for complete packets
     *
     * @param callback Function to call when packet is received
     */
    void setPacketCallback(std::function<void(const Packet&)> callback) {
        packetCallback = callback;
    }

    /**
     * Initialize RC4 encryption for packet headers
     * Must be called after CMSG_AUTH_SESSION before further communication
     *
     * @param sessionKey 40-byte session key from auth server
     */
    void initEncryption(const std::vector<uint8_t>& sessionKey);

    /**
     * Check if header encryption is enabled
     */
    bool isEncryptionEnabled() const { return encryptionEnabled; }

private:
    /**
     * Try to parse complete packets from receive buffer
     */
    void tryParsePackets();

    socket_t sockfd = INVALID_SOCK;
    bool connected = false;
    bool encryptionEnabled = false;

    // RC4 ciphers for header encryption/decryption
    auth::RC4 encryptCipher;  // For outgoing headers
    auth::RC4 decryptCipher;  // For incoming headers

    // Receive buffer
    std::vector<uint8_t> receiveBuffer;

    // Track how many header bytes have been decrypted (0-4)
    // This prevents re-decrypting the same header when waiting for more data
    size_t headerBytesDecrypted = 0;

    // Packet callback
    std::function<void(const Packet&)> packetCallback;
};

} // namespace network
} // namespace wowee
