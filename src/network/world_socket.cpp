#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "network/net_platform.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <fstream>

namespace {
constexpr size_t kMaxReceiveBufferBytes = 8 * 1024 * 1024;

inline bool isLoginPipelineSmsg(uint16_t opcode) {
    switch (opcode) {
        case 0x1EC: // SMSG_AUTH_CHALLENGE
        case 0x1EE: // SMSG_AUTH_RESPONSE
        case 0x03B: // SMSG_CHAR_ENUM
        case 0x03A: // SMSG_CHAR_CREATE
        case 0x03C: // SMSG_CHAR_DELETE
        case 0x4AB: // SMSG_CLIENTCACHE_VERSION
        case 0x0FD: // SMSG_TUTORIAL_FLAGS
        case 0x2E6: // SMSG_WARDEN_DATA
            return true;
        default:
            return false;
    }
}

inline bool isLoginPipelineCmsg(uint16_t opcode) {
    switch (opcode) {
        case 0x1ED: // CMSG_AUTH_SESSION
        case 0x037: // CMSG_CHAR_ENUM
        case 0x036: // CMSG_CHAR_CREATE
        case 0x038: // CMSG_CHAR_DELETE
        case 0x03D: // CMSG_PLAYER_LOGIN
            return true;
        default:
            return false;
    }
}
} // namespace

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

    // Debug: parse and log character-create payload fields (helps diagnose appearance issues).
    if (opcode == 0x036) { // CMSG_CHAR_CREATE
        size_t pos = 0;
        std::string name;
        while (pos < data.size()) {
            uint8_t c = data[pos++];
            if (c == 0) break;
            name.push_back(static_cast<char>(c));
        }
        auto rd8 = [&](uint8_t& out) -> bool {
            if (pos >= data.size()) return false;
            out = data[pos++];
            return true;
        };
        uint8_t race = 0, cls = 0, gender = 0;
        uint8_t skin = 0, face = 0, hairStyle = 0, hairColor = 0, facial = 0, outfit = 0;
        bool ok =
            rd8(race) && rd8(cls) && rd8(gender) &&
            rd8(skin) && rd8(face) && rd8(hairStyle) && rd8(hairColor) && rd8(facial) && rd8(outfit);
        if (ok) {
            LOG_INFO("CMSG_CHAR_CREATE payload: name='", name,
                     "' race=", (int)race, " class=", (int)cls, " gender=", (int)gender,
                     " skin=", (int)skin, " face=", (int)face,
                     " hairStyle=", (int)hairStyle, " hairColor=", (int)hairColor,
                     " facial=", (int)facial, " outfit=", (int)outfit,
                     " payloadLen=", payloadLen);
            // Persist to disk so we can compare TX vs DB even if the console scrolls away.
            std::ofstream f("charcreate_payload.log", std::ios::app);
            if (f.is_open()) {
                f << "name='" << name << "'"
                  << " race=" << (int)race
                  << " class=" << (int)cls
                  << " gender=" << (int)gender
                  << " skin=" << (int)skin
                  << " face=" << (int)face
                  << " hairStyle=" << (int)hairStyle
                  << " hairColor=" << (int)hairColor
                  << " facial=" << (int)facial
                  << " outfit=" << (int)outfit
                  << " payloadLen=" << payloadLen
                  << "\n";
            }
        } else {
            LOG_WARNING("CMSG_CHAR_CREATE payload too short to parse (name='", name,
                        "' payloadLen=", payloadLen, " pos=", pos, ")");
        }
    }

    if (opcode == 0x10C || opcode == 0x10D) { // CMSG_SWAP_ITEM / CMSG_SWAP_INV_ITEM
        std::string hex;
        for (size_t i = 0; i < data.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x ", data[i]);
            hex += buf;
        }
        LOG_INFO("WS TX opcode=0x", std::hex, opcode, std::dec, " payloadLen=", payloadLen, " data=[", hex, "]");
    }

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
    if (isLoginPipelineCmsg(opcode)) {
        LOG_INFO("WS TX LOGIN opcode=0x", std::hex, opcode, std::dec,
                 " payload=", payloadLen, " enc=", encryptionEnabled ? "yes" : "no");
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

    // Drain the socket. Some servers send an auth response and immediately close; a single recv()
    // may read the response, and a subsequent recv() can return 0 (FIN). If we disconnect right
    // away we lose the buffered response and the UI ends up with a generic "no characters" symptom.
    bool sawClose = false;
    bool receivedAny = false;
    size_t bytesReadThisTick = 0;
    int readOps = 0;
    while (connected) {
        uint8_t buffer[4096];
        ssize_t received = net::portableRecv(sockfd, buffer, sizeof(buffer));

        if (received > 0) {
            receivedAny = true;
            ++readOps;
            bytesReadThisTick += static_cast<size_t>(received);
            receiveBuffer.insert(receiveBuffer.end(), buffer, buffer + received);
            if (receiveBuffer.size() > kMaxReceiveBufferBytes) {
                LOG_ERROR("World socket receive buffer overflow (", receiveBuffer.size(),
                          " bytes). Disconnecting to recover framing.");
                disconnect();
                return;
            }
            continue;
        }

        if (received == 0) {
            sawClose = true;
            break;
        }

        int err = net::lastError();
        if (net::isWouldBlock(err)) {
            break;
        }

        LOG_ERROR("Receive failed: ", net::errorString(err));
        disconnect();
        return;
    }

    if (receivedAny) {
        LOG_DEBUG("World socket read ", bytesReadThisTick, " bytes in ", readOps,
                  " recv call(s), buffered=", receiveBuffer.size());
        tryParsePackets();
        if (connected && !receiveBuffer.empty()) {
            LOG_DEBUG("World socket parse left ", receiveBuffer.size(),
                      " bytes buffered (awaiting complete packet)");
        }
    }

    if (sawClose) {
        LOG_INFO("World server connection closed");
        disconnect();
        return;
    }
}

void WorldSocket::tryParsePackets() {
    // World server packets have 4-byte incoming header: size(2) + opcode(2)
    while (receiveBuffer.size() >= 4) {
        uint8_t rawHeader[4] = {0, 0, 0, 0};
        std::memcpy(rawHeader, receiveBuffer.data(), 4);

        // Decrypt header bytes in-place if encryption is enabled
        // Only decrypt bytes we haven't already decrypted
        if (encryptionEnabled && headerBytesDecrypted < 4) {
            size_t toDecrypt = 4 - headerBytesDecrypted;
            decryptCipher.process(receiveBuffer.data() + headerBytesDecrypted, toDecrypt);
            headerBytesDecrypted = 4;
        }

        // Parse header (now decrypted in-place).
        // Size: 2 bytes big-endian. For world packets, this includes opcode bytes.
        uint16_t size = (receiveBuffer[0] << 8) | receiveBuffer[1];
        // Opcode: 2 bytes little-endian.
        uint16_t opcode = receiveBuffer[2] | (receiveBuffer[3] << 8);
        if (size < 2) {
            LOG_ERROR("World packet framing desync: invalid size=", size,
                      " rawHdr=", std::hex,
                      static_cast<int>(rawHeader[0]), " ",
                      static_cast<int>(rawHeader[1]), " ",
                      static_cast<int>(rawHeader[2]), " ",
                      static_cast<int>(rawHeader[3]), std::dec,
                      " enc=", encryptionEnabled, ". Disconnecting to recover stream.");
            disconnect();
            return;
        }
        constexpr uint16_t kMaxWorldPacketSize = 0x4000;
        if (size > kMaxWorldPacketSize) {
            LOG_ERROR("World packet framing desync: oversized packet size=", size,
                      " rawHdr=", std::hex,
                      static_cast<int>(rawHeader[0]), " ",
                      static_cast<int>(rawHeader[1]), " ",
                      static_cast<int>(rawHeader[2]), " ",
                      static_cast<int>(rawHeader[3]), std::dec,
                      " enc=", encryptionEnabled, ". Disconnecting to recover stream.");
            disconnect();
            return;
        }

        const uint16_t payloadLen = size - 2;
        const size_t totalSize = 4 + payloadLen;

        if (headerTracePacketsLeft > 0) {
            LOG_INFO("WS HDR TRACE raw=",
                     std::hex,
                     static_cast<int>(rawHeader[0]), " ",
                     static_cast<int>(rawHeader[1]), " ",
                     static_cast<int>(rawHeader[2]), " ",
                     static_cast<int>(rawHeader[3]),
                     " dec=",
                     static_cast<int>(receiveBuffer[0]), " ",
                     static_cast<int>(receiveBuffer[1]), " ",
                     static_cast<int>(receiveBuffer[2]), " ",
                     static_cast<int>(receiveBuffer[3]),
                     std::dec,
                     " size=", size,
                     " payload=", payloadLen,
                     " opcode=0x", std::hex, opcode, std::dec,
                     " buffered=", receiveBuffer.size());
            --headerTracePacketsLeft;
        }
        if (isLoginPipelineSmsg(opcode)) {
            LOG_INFO("WS RX LOGIN opcode=0x", std::hex, opcode, std::dec,
                     " size=", size, " payload=", payloadLen,
                     " buffered=", receiveBuffer.size(),
                     " enc=", encryptionEnabled ? "yes" : "no");
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
    headerTracePacketsLeft = 24;
    LOG_INFO("World server encryption initialized successfully");
}

} // namespace network
} // namespace wowee
