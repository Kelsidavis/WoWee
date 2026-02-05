#include "network/tcp_socket.hpp"
#include "network/packet.hpp"
#include "network/net_platform.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace network {

TCPSocket::TCPSocket() {
    net::ensureInit();
}

TCPSocket::~TCPSocket() {
    disconnect();
}

bool TCPSocket::connect(const std::string& host, uint16_t port) {
    LOG_INFO("Connecting to ", host, ":", port);

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

        // Non-blocking connect in progress — wait for it to complete
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sockfd, &writefds);

        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        int selectResult = ::select(static_cast<int>(sockfd) + 1, nullptr, &writefds, nullptr, &tv);
        if (selectResult <= 0) {
            LOG_ERROR("Connection timed out to ", host, ":", port);
            net::closeSocket(sockfd);
            sockfd = INVALID_SOCK;
            return false;
        }

        // Check if the connection actually succeeded
        int sockErr = 0;
        socklen_t errLen = sizeof(sockErr);
        getsockopt(sockfd, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&sockErr), &errLen);
        if (sockErr != 0) {
            LOG_ERROR("Connection failed: ", net::errorString(sockErr));
            net::closeSocket(sockfd);
            sockfd = INVALID_SOCK;
            return false;
        }
    }

    connected = true;
    LOG_INFO("Connected to ", host, ":", port);
    return true;
}

void TCPSocket::disconnect() {
    if (sockfd != INVALID_SOCK) {
        net::closeSocket(sockfd);
        sockfd = INVALID_SOCK;
    }
    connected = false;
    receiveBuffer.clear();
}

void TCPSocket::send(const Packet& packet) {
    if (!connected) return;

    // Build complete packet with opcode
    std::vector<uint8_t> sendData;

    // Add opcode (1 byte) - always little-endian, but it's just 1 byte so doesn't matter
    sendData.push_back(static_cast<uint8_t>(packet.getOpcode() & 0xFF));

    // Add packet data
    const auto& data = packet.getData();
    sendData.insert(sendData.end(), data.begin(), data.end());

    LOG_DEBUG("Sending packet: opcode=0x", std::hex, packet.getOpcode(), std::dec,
              " size=", sendData.size(), " bytes");

    // Send complete packet
    ssize_t sent = net::portableSend(sockfd, sendData.data(), sendData.size());
    if (sent < 0) {
        LOG_ERROR("Send failed: ", net::errorString(net::lastError()));
    } else if (static_cast<size_t>(sent) != sendData.size()) {
        LOG_WARNING("Partial send: ", sent, " of ", sendData.size(), " bytes");
    }
}

void TCPSocket::update() {
    if (!connected) return;

    // Receive data into buffer
    uint8_t buffer[4096];
    ssize_t received = net::portableRecv(sockfd, buffer, sizeof(buffer));

    if (received > 0) {
        LOG_DEBUG("Received ", received, " bytes from server");
        receiveBuffer.insert(receiveBuffer.end(), buffer, buffer + received);

        // Try to parse complete packets from buffer
        tryParsePackets();
    }
    else if (received == 0) {
        LOG_INFO("Connection closed by server");
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

void TCPSocket::tryParsePackets() {
    // For auth packets, we need at least 1 byte (opcode)
    while (receiveBuffer.size() >= 1) {
        uint8_t opcode = receiveBuffer[0];

        // Determine expected packet size based on opcode
        // This is specific to authentication protocol
        size_t expectedSize = getExpectedPacketSize(opcode);

        if (expectedSize == 0) {
            // Unknown opcode or need more data to determine size
            LOG_WARNING("Unknown opcode or indeterminate size: 0x", std::hex, (int)opcode, std::dec);
            break;
        }

        if (receiveBuffer.size() < expectedSize) {
            // Not enough data yet
            LOG_DEBUG("Waiting for more data: have ", receiveBuffer.size(),
                     " bytes, need ", expectedSize);
            break;
        }

        // We have a complete packet!
        LOG_DEBUG("Parsing packet: opcode=0x", std::hex, (int)opcode, std::dec,
                 " size=", expectedSize, " bytes");

        // Create packet from buffer data
        std::vector<uint8_t> packetData(receiveBuffer.begin(),
                                        receiveBuffer.begin() + expectedSize);

        Packet packet(opcode, packetData);

        // Remove parsed data from buffer
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + expectedSize);

        // Call callback if set
        if (packetCallback) {
            packetCallback(packet);
        }
    }
}

size_t TCPSocket::getExpectedPacketSize(uint8_t opcode) {
    // Authentication packet sizes (WoW 3.3.5a)
    // Note: These are minimum sizes. Some packets are variable length.

    switch (opcode) {
        case 0x00:  // LOGON_CHALLENGE response
            // Need to read status byte to determine success/failure
            if (receiveBuffer.size() >= 3) {
                uint8_t status = receiveBuffer[2];
                if (status == 0x00) {
                    // Success: opcode(1) + unk(1) + status(1) + B(32) + gLen(1) + g(gLen) +
                    //          nLen(1) + N(nLen) + salt(32) + crcHash(16) + securityFlags(1)
                    //          + optional security flag data
                    if (receiveBuffer.size() >= 36) {  // enough to read g_len
                        uint8_t gLen = receiveBuffer[35];
                        size_t minSize = 36 + gLen + 1;  // up to N_len
                        if (receiveBuffer.size() >= minSize) {
                            uint8_t nLen = receiveBuffer[36 + gLen];
                            size_t baseSize = 36 + gLen + 1 + nLen + 32 + 16 + 1;
                            // Need to read securityFlags to account for extra data
                            if (receiveBuffer.size() >= baseSize) {
                                uint8_t secFlags = receiveBuffer[baseSize - 1];
                                size_t extra = 0;
                                if (secFlags & 0x01) extra += 20;  // PIN: seed(4) + salt(16)
                                if (secFlags & 0x02) extra += 12;  // Matrix: w(1)+h(1)+digits(1)+challenges(1)+seed(8)
                                if (secFlags & 0x04) extra += 1;   // Authenticator: required(1)
                                return baseSize + extra;
                            }
                        }
                    }
                    return 0;  // Need more data
                } else {
                    // Failure - just opcode + unknown + status
                    return 3;
                }
            }
            return 0;  // Need more data to determine

        case 0x01:  // LOGON_PROOF response
            // Success: opcode(1) + status(1) + M2(20) + accountFlags(4) + surveyId(4) + loginFlags(2) = 32
            // Failure: varies by server — minimum 2 bytes (opcode + status), some send 4
            if (receiveBuffer.size() >= 2) {
                uint8_t status = receiveBuffer[1];
                if (status == 0x00) {
                    return 32;  // Success (WotLK 3.3.5a format)
                } else {
                    // Consume up to 4 bytes if available, minimum 2
                    return (receiveBuffer.size() >= 4) ? 4 : 2;
                }
            }
            return 0;  // Need more data

        case 0x10:  // REALM_LIST response
            // Variable length - format: opcode(1) + size(2) + payload(size)
            // Need to read size field (little-endian uint16 at offset 1-2)
            if (receiveBuffer.size() >= 3) {
                uint16_t size = receiveBuffer[1] | (receiveBuffer[2] << 8);
                // Total packet size is: opcode(1) + size field(2) + payload(size)
                return 1 + 2 + size;
            }
            return 0;  // Need more data to read size field

        default:
            LOG_WARNING("Unknown auth packet opcode: 0x", std::hex, (int)opcode, std::dec);
            return 0;
    }
}

} // namespace network
} // namespace wowee
