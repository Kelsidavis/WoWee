#pragma once

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

namespace wowee {
namespace network {

class Packet;

class Socket {
public:
    virtual ~Socket() = default;

    virtual bool connect(const std::string& host, uint16_t port) = 0;
    virtual void disconnect() = 0;
    [[nodiscard]] virtual bool isConnected() const = 0;

    virtual void send(const Packet& packet) = 0;
    virtual void update() = 0;

    using PacketCallback = std::function<void(const Packet&)>;
    void setPacketCallback(PacketCallback callback) { packetCallback = callback; }

protected:
    PacketCallback packetCallback;
};

} // namespace network
} // namespace wowee
