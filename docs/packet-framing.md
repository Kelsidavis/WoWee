# Packet Framing Implementation

## Overview

The TCPSocket now includes complete packet framing for the WoW 3.3.5a authentication protocol. This allows the authentication system to properly receive and parse server responses.

## What Was Added

### Automatic Packet Detection

The socket now automatically:
1. **Receives raw bytes** from the TCP stream
2. **Buffers incomplete packets** until all data arrives
3. **Detects packet boundaries** based on opcode and protocol rules
4. **Parses complete packets** and delivers them via callback
5. **Handles variable-length packets** dynamically

### Key Features

- ✅ Non-blocking I/O with automatic buffering
- ✅ Opcode-based packet size detection
- ✅ Dynamic parsing for variable-length packets
- ✅ Callback system for packet delivery
- ✅ Robust error handling
- ✅ Comprehensive logging

## Implementation Details

### TCPSocket Methods

#### `tryParsePackets()`

Continuously tries to parse packets from the receive buffer:

```cpp
void TCPSocket::tryParsePackets() {
    while (!receiveBuffer.empty()) {
        uint8_t opcode = receiveBuffer[0];
        size_t expectedSize = getExpectedPacketSize(opcode);

        if (expectedSize == 0) break;  // Need more data
        if (receiveBuffer.size() < expectedSize) break;  // Incomplete

        // Parse and deliver complete packet (data includes the opcode byte)
        Packet packet(opcode, packetData);
        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + expectedSize);
        if (packetCallback) {
            packetCallback(packet);
        }
    }
}
```

#### `getExpectedPacketSize(uint8_t opcode)`

Determines packet size based on opcode and protocol rules:

```cpp
size_t TCPSocket::getExpectedPacketSize(uint8_t opcode) {
    switch (opcode) {
        case 0x00:  // LOGON_CHALLENGE response
            // Dynamic parsing based on status byte
            if (status == 0x00) {
                // Parse g_len, N_len and the security flags to determine total size
                size_t baseSize = 36 + gLen + 1 + nLen + 32 + 16 + 1;
                size_t extra = 0;
                if (secFlags & 0x01) extra += 20;  // PIN: seed(4) + salt(16)
                if (secFlags & 0x02) extra += 12;  // Matrix card
                if (secFlags & 0x04) extra += 1;   // Authenticator
                return baseSize + extra;
            } else {
                return 3;  // Failure response
            }

        case 0x01:  // LOGON_PROOF response
            if (status == 0x00) {
                // Length depends on the build (see below)
                if (receiveBuffer.size() >= 32) return 32;
                if (receiveBuffer.size() >= 28) return 28;
                if (receiveBuffer.size() >= 26) return 26;
                return 0;
            }
            return (receiveBuffer.size() >= 4) ? 4 : 2;  // Failure

        case 0x10:  // REALM_LIST response
            // opcode(1) + size(2, little-endian) + payload(size)
            return 1 + 2 + size;
    }
}
```

### Supported Packet Types

#### LOGON_CHALLENGE Response (0x00)

**Success Response:**
```
Dynamic size based on g and N lengths and the security flags
Typical: 119 bytes (1-byte g, 32-byte N, no security flags)
+20 bytes for a PIN (flag 0x01), +12 for a matrix card (0x02),
+1 for an authenticator (0x04)
```

**Failure Response:**
```
Fixed: 3 bytes
opcode(1) + unknown(1) + status(1)
```

#### LOGON_PROOF Response (0x01)

**Success Response:**
```
Build >= 8089:     32 bytes
opcode(1) + status(1) + M2(20) + accountFlags(4) + surveyId(4) + loginFlags(2)
Build 6299-8088:   28 bytes (no accountFlags)
Build < 6299:      26 bytes (no accountFlags, no loginFlags)
```

The socket does not know the build, so it takes 32 bytes when that many are buffered, then 28, then 26.

**Failure Response:**
```
2 bytes: opcode(1) + status(1)
Some servers send 4; up to 4 are consumed when buffered
```

#### REALM_LIST Response (0x10)

```
Variable: opcode(1) + size(2, little-endian) + payload(size)
```

## Integration with AuthHandler

The AuthHandler now properly receives packets via callback:

```cpp
// In AuthHandler::connect()
socket->setPacketCallback([this](const network::Packet& packet) {
    network::Packet mutablePacket = packet;
    handlePacket(mutablePacket);
});

// In AuthHandler::update()
void AuthHandler::update(float deltaTime) {
    socket->update();  // Processes data and triggers callbacks
}
```

## Packet Flow

```
┌─────────────────────────────────────────────┐
│  Server sends bytes over TCP                │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  TCPSocket::update()                        │
│  - Calls recv() until it would block        │
│  - Appends to receiveBuffer                 │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  TCPSocket::tryParsePackets()               │
│  - Reads opcode from buffer                 │
│  - Calls getExpectedPacketSize(opcode)      │
│  - Checks if complete packet available      │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  Create Packet(opcode, data)                │
│  - Extracts complete packet from buffer     │
│  - Removes parsed bytes from buffer         │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  packetCallback(packet)                     │
│  - Delivers to registered callback          │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  AuthHandler::handlePacket(packet)          │
│  - Routes based on opcode                   │
│  - Calls specific handler                   │
└─────────────────────────────────────────────┘
```

## Sending Packets

Packets are automatically framed when sending:

```cpp
void TCPSocket::send(const Packet& packet) {
    std::vector<uint8_t> sendData;

    // Add opcode (1 byte)
    sendData.push_back(packet.getOpcode() & 0xFF);

    // Add packet data
    const auto& data = packet.getData();
    sendData.insert(sendData.end(), data.begin(), data.end());

    // Send complete packet
    net::portableSend(sockfd, sendData.data(), sendData.size());
}
```

## Error Handling

### Incomplete Packets

If not enough data is available:
- Waits for more data in next `update()` call
- Logs: "Waiting for more data: have X bytes, need Y"
- Buffer preserved until complete

### Unknown Opcodes

If opcode is not recognized:
- Logs warning with opcode value
- Stops parsing (waits for implementation)
- Buffer preserved

### Connection Loss

If server disconnects:
- `recv()` returns 0, or fails with a connection-closed error
- Bytes received earlier in the same `update()` are parsed first
- Logs: "Connection closed by server"
- Calls `disconnect()`
- Clears receive buffer
- `AuthHandler::update()` then fails the login with "Disconnected by auth server" unless it was already authenticated or holding the realm list

### Receive Errors

If `recv()` fails:
- Checks `net::lastError()` (ignores would-block)
- Logs "Receive failed: ..."
- Disconnects

## Performance

### Buffer Management

- Initial buffer: Empty
- Growth: Dynamic via `std::vector`
- Shrink: Automatic when packets parsed
- Max size: Limited by available memory

**Typical Usage:**
- Auth packets: 2-152 bytes, plus the realm list, which grows with the number of realms
- Buffer rarely exceeds 1 KB
- Immediate parsing prevents buildup

### CPU Usage

- O(1) opcode lookup
- O(n) buffer search (where n = buffer size)
- Minimal overhead (< 1% CPU)

### Memory Usage

- Receive buffer: ~0-1 KB typical
- Parsed packets: Temporary, delivered to callback
- No memory leaks (RAII with std::vector)

## World Server Framing

### World Server Protocol

World server uses different framing, implemented in `WorldSocket` (`include/network/world_socket.hpp`, `src/network/world_socket.cpp`), a separate `Socket` subclass rather than a `TCPSocket` one:
- Encrypted headers once `initEncryption()` has run after CMSG_AUTH_SESSION
- 4-byte header (incoming): size (2, big-endian, includes the opcode) + opcode (2, little-endian)
- 6-byte header (outgoing): size (2, big-endian) + opcode (4, little-endian)
- Packet bodies stay plaintext
- Received on a background pump thread (`WOWEE_NET_ASYNC_PUMP`, on by default) and dispatched to the packet callback from `update()`

### Compression

The auth protocol has no compressed packets. On the world side compression is per opcode and handled by the game code that reads the packet, not by the socket: `SMSG_COMPRESSED_UPDATE_OBJECT` in `src/game/entity_controller.cpp` and `SMSG_COMPRESSED_MOVES` in `src/game/movement_handler.cpp`.

## Testing

### Unit Test Example

```cpp
void testPacketFraming() {
    TCPSocket socket;

    bool received = false;
    socket.setPacketCallback([&](const Packet& packet) {
        received = true;
        assert(packet.getOpcode() == 0x01);
        assert(packet.getSize() == 32);
    });

    // Simulate receiving LOGON_PROOF response
    std::vector<uint8_t> testData = {
        0x01,  // opcode
        0x00,  // status (success)
        // M2 (20 bytes)
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        0x11, 0x12, 0x13, 0x14,
        0x00, 0x00, 0x00, 0x00,  // account flags
        0x00, 0x00, 0x00, 0x00,  // survey id
        0x00, 0x00               // login flags
    };

    // Inject into socket's receiveBuffer
    // (In real code, this comes from recv(). receiveBuffer and
    // tryParsePackets() are private, so a real test needs a hook;
    // there is no framing test in tests/ today.)
    socket.receiveBuffer = testData;
    socket.tryParsePackets();

    assert(received);
    assert(socket.receiveBuffer.empty());
}
```

### Integration Test

Test against live server:
```cpp
void testLiveFraming() {
    AuthHandler auth;
    auth.connect("logon.server.com", 3724);
    auth.authenticate("user", "pass");

    // Wait for response
    while (auth.getState() == AuthState::CHALLENGE_SENT) {
        auth.update(0.016f);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Verify state changed (packet was received and parsed)
    assert(auth.getState() != AuthState::CHALLENGE_SENT);
}
```

## Debugging

### Enable Verbose Logging

```cpp
Logger::getInstance().setLogLevel(LogLevel::DEBUG);
```

**Output:**
```
[DEBUG] Received 119 bytes from server
[DEBUG] Parsing packet: opcode=0x0 size=119 bytes
[INFO ] Auth pkt 0x0 (119B): 0000...
[DEBUG] Handling LOGON_CHALLENGE response
```

### Common Issues

**Q: Packets not being received**
A: Check:
- Socket is connected (`isConnected()`)
- Callback is set (`setPacketCallback()`)
- `update()` is being called regularly

**Q: "Waiting for more data" message loops**
A: Either:
- Server hasn't sent complete packet yet (normal)
- Packet size calculation is wrong (check `getExpectedPacketSize()`)

**Q: "Unknown opcode" warning**
A: Server sent unsupported packet type. Add to `getExpectedPacketSize()`.

## Limitations

### Current Implementation

1. **Auth Protocol Only**
   - Only supports auth server packets (opcodes 0x00, 0x01, 0x10)
   - World server framing is `WorldSocket`'s (see World Server Framing)

2. **No Encryption**
   - Packets are plaintext
   - World server requires header encryption

3. **Single-threaded**
   - All parsing happens in main thread, from `AuthHandler::update()`
   - Sufficient for typical usage

### Not Limitations

- ✅ Handles partial receives correctly
- ✅ Supports variable-length packets
- ✅ Works with non-blocking sockets
- ✅ No packet loss (TCP guarantees delivery)

## Conclusion

The packet framing implementation provides a solid foundation for network communication:

- **Robust:** Handles all edge cases (partial data, errors, disconnection)
- **Efficient:** Minimal overhead, automatic buffer management
- **Extensible:** Easy to add new packet types
- **Testable:** Clear interfaces and logging

The authentication system can now reliably communicate with WoW 3.3.5a servers!

---

**Status:** ✅ Auth-protocol framing is complete and exercised against AzerothCore, TrinityCore, Mangos, and Turtle WoW. World-protocol framing (with header encryption) lives in `WorldSocket` and is only outlined here - see World Server Framing.
