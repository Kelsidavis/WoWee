#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "network/net_platform.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace network {

// WoW 3.3.5a RC4 encryption keys (hardcoded in client)
static const uint8_t ENCRYPT_KEY[] = {
    0xC2, 0xB3, 0x72, 0x3C, 0xC6, 0xAE, 0xD9, 0xB5,
    0x34, 0x3C, 0x53, 0xEE, 0x2F, 0x43, 0x67, 0xCE
};

static const uint8_t DECRYPT_KEY[] = {
    0xCC, 0x98, 0xAE, 0x04, 0xE8, 0x97, 0xEA, 0xCA,
    0x12, 0xDD, 0xC0, 0x93, 0x42, 0x91, 0x53, 0x57
};

WorldSocket::WorldSocket() {
    net::ensureInit();
}

WorldSocket::~WorldSocket() {
    disconnect();
}

bool WorldSocket::connect(const std::string& host, uint16_t port) {
    LOG_INFO("Connecting to world server: ", host, ":", port);

    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == INVALID_SOCK) {
        LOG_ERROR("Failed to create socket");
        return false;
    }

    // Set non-blocking
    net::setNonBlocking(sockfd);

    // Resolve host
    struct hostent* server = gethostbyname(host.c_str());
    if (server == nullptr) {
        LOG_ERROR("Failed to resolve host: ", host);
        net::closeSocket(sockfd);
        sockfd = INVALID_SOCK;
        return false;
    }

    // Connect
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    memcpy(&serverAddr.sin_addr.s_addr, server->h_addr, server->h_length);
    serverAddr.sin_port = htons(port);

    int result = ::connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (result < 0) {
        int err = net::lastError();
        if (!net::isInProgress(err)) {
            LOG_ERROR("Failed to connect: ", net::errorString(err));
            net::closeSocket(sockfd);
            sockfd = INVALID_SOCK;
            return false;
        }
    }

    connected = true;
    LOG_INFO("Connected to world server: ", host, ":", port);
    return true;
}

void WorldSocket::disconnect() {
    if (sockfd != INVALID_SOCK) {
        net::closeSocket(sockfd);
        sockfd = INVALID_SOCK;
    }
    connected = false;
    encryptionEnabled = false;
    receiveBuffer.clear();
    LOG_INFO("Disconnected from world server");
}

bool WorldSocket::isConnected() const {
    return connected;
}

void WorldSocket::send(const Packet& packet) {
    if (!connected) return;

    const auto& data = packet.getData();
    uint16_t opcode = packet.getOpcode();
    uint16_t payloadLen = static_cast<uint16_t>(data.size());

    // WotLK 3.3.5 CMSG header (6 bytes total):
    // - size (2 bytes, big-endian) = payloadLen + 4 (opcode is 4 bytes for CMSG)
    // - opcode (4 bytes, little-endian)
    // Note: Client-to-server uses 4-byte opcode, server-to-client uses 2-byte
    uint16_t sizeField = payloadLen + 4;

    std::vector<uint8_t> sendData;
    sendData.reserve(6 + payloadLen);

    // Size (2 bytes, big-endian)
    uint8_t size_hi = (sizeField >> 8) & 0xFF;
    uint8_t size_lo = sizeField & 0xFF;
    sendData.push_back(size_hi);
    sendData.push_back(size_lo);

    // Opcode (4 bytes, little-endian)
    sendData.push_back(opcode & 0xFF);
    sendData.push_back((opcode >> 8) & 0xFF);
    sendData.push_back(0);  // High bytes are 0 for all WoW opcodes
    sendData.push_back(0);

    // Debug: log encryption state and header
    LOG_DEBUG("SEND opcode=0x", std::hex, opcode, std::dec,
              " encryptionEnabled=", encryptionEnabled,
              " header=[", std::hex,
              (int)sendData[0], " ", (int)sendData[1], " ",
              (int)sendData[2], " ", (int)sendData[3], " ",
              (int)sendData[4], " ", (int)sendData[5], std::dec, "]",
              " sizeField=", sizeField, " payloadLen=", payloadLen);

    // Encrypt header if encryption is enabled (all 6 bytes)
    if (encryptionEnabled) {
        uint8_t plainHeader[6];
        memcpy(plainHeader, sendData.data(), 6);
        encryptCipher.process(sendData.data(), 6);
        LOG_DEBUG("Encrypted header: plain=[", std::hex,
                  (int)plainHeader[0], " ", (int)plainHeader[1], " ",
                  (int)plainHeader[2], " ", (int)plainHeader[3], " ",
                  (int)plainHeader[4], " ", (int)plainHeader[5], "] -> enc=[",
                  (int)sendData[0], " ", (int)sendData[1], " ",
                  (int)sendData[2], " ", (int)sendData[3], " ",
                  (int)sendData[4], " ", (int)sendData[5], "]", std::dec);
    }

    // Add payload (unencrypted)
    sendData.insert(sendData.end(), data.begin(), data.end());

    LOG_DEBUG("Sending world packet: opcode=0x", std::hex, opcode, std::dec,
              " payload=", payloadLen, " bytes (", sendData.size(), " total)");

    // Debug: dump packet bytes for AUTH_SESSION
    if (opcode == 0x1ED) {
        std::string hexDump = "AUTH_SESSION raw bytes: ";
        for (size_t i = 0; i < sendData.size(); ++i) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x ", sendData[i]);
            hexDump += buf;
            if ((i + 1) % 32 == 0) hexDump += "\n";
        }
        LOG_DEBUG(hexDump);
    }

    // Send complete packet
    ssize_t sent = net::portableSend(sockfd, sendData.data(), sendData.size());
    if (sent < 0) {
        LOG_ERROR("Send failed: ", net::errorString(net::lastError()));
    } else {
        LOG_DEBUG("Actually sent ", sent, " bytes to server");
        if (static_cast<size_t>(sent) != sendData.size()) {
            LOG_WARNING("Partial send: ", sent, " of ", sendData.size(), " bytes");
        }
    }
}

void WorldSocket::update() {
    if (!connected) return;

    // Receive data into buffer
    uint8_t buffer[4096];
    ssize_t received = net::portableRecv(sockfd, buffer, sizeof(buffer));

    if (received > 0) {
        LOG_DEBUG("Received ", received, " bytes from world server");
        receiveBuffer.insert(receiveBuffer.end(), buffer, buffer + received);

        // Try to parse complete packets from buffer
        tryParsePackets();
    }
    else if (received == 0) {
        LOG_INFO("World server connection closed");
        disconnect();
    }
    else {
        int err = net::lastError();
        if (!net::isWouldBlock(err)) {
            LOG_ERROR("Receive failed: ", net::errorString(err));
            disconnect();
        }
    }
}

void WorldSocket::tryParsePackets() {
    // World server packets have 4-byte incoming header: size(2) + opcode(2)
    while (receiveBuffer.size() >= 4) {
        // Copy header for decryption
        uint8_t header[4];
        memcpy(header, receiveBuffer.data(), 4);

        // Decrypt header if encryption is enabled
        if (encryptionEnabled) {
            decryptCipher.process(header, 4);
        }

        // Parse header
        // Size: 2 bytes big-endian (includes opcode, so payload = size - 2)
        uint16_t size = (header[0] << 8) | header[1];
        // Opcode: 2 bytes little-endian
        uint16_t opcode = header[2] | (header[3] << 8);

        LOG_DEBUG("RECV encryptionEnabled=", encryptionEnabled,
                  " header=[", std::hex, (int)header[0], " ", (int)header[1], " ",
                  (int)header[2], " ", (int)header[3], std::dec, "]",
                  " -> size=", size, " opcode=0x", std::hex, opcode, std::dec);

        // Total packet size: size field (2) + size value (which includes opcode + payload)
        size_t totalSize = 2 + size;

        if (receiveBuffer.size() < totalSize) {
            // Not enough data yet
            LOG_DEBUG("Waiting for more data: have ", receiveBuffer.size(),
                     " bytes, need ", totalSize);
            break;
        }

        // We have a complete packet!
        LOG_DEBUG("Parsing world packet: opcode=0x", std::hex, opcode, std::dec,
                 " size=", size, " totalSize=", totalSize, " bytes");

        // Extract payload (skip header)
        std::vector<uint8_t> packetData(receiveBuffer.begin() + 4,
                                        receiveBuffer.begin() + totalSize);

        // Log first few bytes of payload
        if (!packetData.empty()) {
            std::string payloadHex;
            for (size_t i = 0; i < std::min(packetData.size(), size_t(16)); i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02x ", packetData[i]);
                payloadHex += buf;
            }
            LOG_DEBUG("Payload (first 16 bytes): ", payloadHex);
        }

        // Create packet with opcode and payload
        Packet packet(opcode, packetData);

        // Remove parsed data from buffer
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + totalSize);

        // Call callback if set
        if (packetCallback) {
            packetCallback(packet);
        }
    }
}

void WorldSocket::initEncryption(const std::vector<uint8_t>& sessionKey) {
    if (sessionKey.size() != 40) {
        LOG_ERROR("Invalid session key size: ", sessionKey.size(), " (expected 40)");
        return;
    }

    LOG_INFO(">>> ENABLING ENCRYPTION - encryptionEnabled will become true <<<");

    // Convert hardcoded keys to vectors
    std::vector<uint8_t> encryptKey(ENCRYPT_KEY, ENCRYPT_KEY + 16);
    std::vector<uint8_t> decryptKey(DECRYPT_KEY, DECRYPT_KEY + 16);

    // Compute HMAC-SHA1(seed, sessionKey) for each cipher
    // The 16-byte seed is the HMAC key, session key is the message
    std::vector<uint8_t> encryptHash = auth::Crypto::hmacSHA1(encryptKey, sessionKey);
    std::vector<uint8_t> decryptHash = auth::Crypto::hmacSHA1(decryptKey, sessionKey);

    LOG_DEBUG("Encrypt hash: ", encryptHash.size(), " bytes");
    LOG_DEBUG("Decrypt hash: ", decryptHash.size(), " bytes");

    // Initialize RC4 ciphers with HMAC results
    encryptCipher.init(encryptHash);
    decryptCipher.init(decryptHash);

    // Drop first 1024 bytes of keystream (WoW protocol requirement)
    encryptCipher.drop(1024);
    decryptCipher.drop(1024);

    encryptionEnabled = true;
    LOG_INFO("World server encryption initialized successfully");
}

} // namespace network
} // namespace wowee
