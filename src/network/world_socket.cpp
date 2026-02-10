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
    headerBytesDecrypted = 0;
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

    // Debug logging disabled - too spammy

    // Encrypt header if encryption is enabled (all 6 bytes)
    if (encryptionEnabled) {
        encryptCipher.process(sendData.data(), 6);
    }

    // Add payload (unencrypted)
    sendData.insert(sendData.end(), data.begin(), data.end());


    // Debug: dump first few movement packets
    {
        static int moveDump = 3;
        bool isMove = (opcode >= 0xB5 && opcode <= 0xBE) || opcode == 0xC9 || opcode == 0xDA || opcode == 0xEE;
        if (isMove && moveDump-- > 0) {
            std::string hex = "MOVE PKT dump opcode=0x";
            char buf[8]; snprintf(buf, sizeof(buf), "%03x", opcode); hex += buf;
            hex += " payload=" + std::to_string(payloadLen) + " bytes: ";
            for (size_t i = 6; i < sendData.size() && i < 6 + 48; i++) {
                char b[4]; snprintf(b, sizeof(b), "%02x ", sendData[i]);
                hex += b;
            }
            LOG_INFO(hex);
        }
    }

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
        // Decrypt header bytes in-place if encryption is enabled
        // Only decrypt bytes we haven't already decrypted
        if (encryptionEnabled && headerBytesDecrypted < 4) {
            size_t toDecrypt = 4 - headerBytesDecrypted;
            decryptCipher.process(receiveBuffer.data() + headerBytesDecrypted, toDecrypt);
            headerBytesDecrypted = 4;
        }

        // Parse header (now decrypted in-place)
        // Size: 2 bytes big-endian (includes opcode, so payload = size - 2)
        uint16_t size = (receiveBuffer[0] << 8) | receiveBuffer[1];
        // Opcode: 2 bytes little-endian
        uint16_t opcode = receiveBuffer[2] | (receiveBuffer[3] << 8);

        // Total packet size: size field (2) + size value (which includes opcode + payload)
        size_t totalSize = 2 + size;

        // DEBUG: Log packet boundary details for quest-related opcodes
        if (opcode == 0x18F || opcode == 0x18D || opcode == 0x188 || opcode == 0x186) {
            char hexBuf[256];
            snprintf(hexBuf, sizeof(hexBuf),
                     "PACKET BOUNDARY: opcode=0x%04X size=%u totalSize=%zu bufferSize=%zu",
                     opcode, size, totalSize, receiveBuffer.size());
            core::Logger::getInstance().info(hexBuf);

            // Dump header bytes
            snprintf(hexBuf, sizeof(hexBuf),
                     "  Header: %02x %02x %02x %02x",
                     receiveBuffer[0], receiveBuffer[1], receiveBuffer[2], receiveBuffer[3]);
            core::Logger::getInstance().info(hexBuf);

            // Dump first 16 bytes of payload (if available)
            if (totalSize <= receiveBuffer.size()) {
                std::string payloadHex = "  Payload: ";
                for (size_t i = 4; i < std::min(totalSize, size_t(20)); ++i) {
                    char buf[4];
                    snprintf(buf, sizeof(buf), "%02x ", receiveBuffer[i]);
                    payloadHex += buf;
                }
                core::Logger::getInstance().info(payloadHex);
            }

            // Dump what comes after this packet (next header preview)
            if (receiveBuffer.size() > totalSize && receiveBuffer.size() >= totalSize + 4) {
                snprintf(hexBuf, sizeof(hexBuf),
                         "  Next header: %02x %02x %02x %02x",
                         receiveBuffer[totalSize], receiveBuffer[totalSize+1],
                         receiveBuffer[totalSize+2], receiveBuffer[totalSize+3]);
                core::Logger::getInstance().info(hexBuf);
            }
        }

        if (receiveBuffer.size() < totalSize) {
            // Not enough data yet - header stays decrypted in buffer
            break;
        }

        // Extract payload (skip header)
        std::vector<uint8_t> packetData(receiveBuffer.begin() + 4,
                                        receiveBuffer.begin() + totalSize);

        // Create packet with opcode and payload
        Packet packet(opcode, packetData);

        // Remove parsed data from buffer and reset header decryption counter
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + totalSize);
        headerBytesDecrypted = 0;

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
