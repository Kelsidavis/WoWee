# Realm List Protocol Guide

## Overview

The realm list protocol allows the client to retrieve the list of available game servers (realms) from the authentication server after successful authentication. This is the second step in the connection flow, following authentication.

## Connection Flow

```
1. Connect to auth server (port 3724)
2. Authenticate (LOGON_CHALLENGE + LOGON_PROOF)
3. Request realm list (REALM_LIST)
4. Select realm
5. Connect to world server (realm's address)
```

## Implementation

### Data Structures

#### Realm

The `Realm` struct contains all information about a game server:

```cpp
struct Realm {
    uint8_t icon;              // Realm icon type
    uint8_t lock;              // Lock status
    uint8_t flags;             // Realm flags (bit 0x04 = has version info)
    std::string name;          // Realm name (e.g., "My Private Server")
    std::string address;       // Server address (e.g., "localhost:8085")
    float population;          // Population level (0.0 to 2.0+)
    uint8_t characters;        // Number of characters player has on this realm
    uint8_t timezone;          // Timezone ID
    uint8_t id;                // Realm ID

    // Version info (conditional - only if flags & 0x04)
    uint8_t majorVersion;      // Major version (e.g., 3)
    uint8_t minorVersion;      // Minor version (e.g., 3)
    uint8_t patchVersion;      // Patch version (e.g., 5)
    uint16_t build;            // Build number (e.g., 12340 for 3.3.5a)

    bool hasVersionInfo() const { return (flags & 0x04) != 0; }
};
```

#### RealmListResponse

Container for the list of realms:

```cpp
struct RealmListResponse {
    std::vector<Realm> realms;  // All available realms
};
```

### API Usage

#### Basic Usage

```cpp
#include "auth/auth_handler.hpp"

using namespace wowee::auth;

// Create auth handler
AuthHandler auth;

// Connect to auth server
if (!auth.connect("logon.myserver.com", 3724)) {
    std::cerr << "Failed to connect" << std::endl;
    return;
}

// Set up callbacks
auth.setOnSuccess([&auth](const std::vector<uint8_t>& sessionKey) {
    std::cout << "Authentication successful!" << std::endl;
    std::cout << "Session key size: " << sessionKey.size() << " bytes" << std::endl;

    // Request realm list after successful authentication
    auth.requestRealmList();
});

auth.setOnRealmList([](const std::vector<Realm>& realms) {
    std::cout << "Received " << realms.size() << " realms:" << std::endl;

    for (const auto& realm : realms) {
        std::cout << "  - " << realm.name << " (" << realm.address << ")" << std::endl;
        std::cout << "    Population: " << realm.population << std::endl;
        std::cout << "    Characters: " << (int)realm.characters << std::endl;
    }
});

auth.setOnFailure([](const std::string& reason) {
    std::cerr << "Authentication failed: " << reason << std::endl;
});

// Start authentication
auth.authenticate("username", "password");

// Main loop
while (auth.getState() != AuthState::REALM_LIST_RECEIVED &&
       auth.getState() != AuthState::FAILED) {
    auth.update(0.016f);  // ~60 FPS
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
}

// Access realm list
const auto& realms = auth.getRealms();
if (!realms.empty()) {
    std::cout << "First realm: " << realms[0].name << std::endl;
}
```

#### Complete Example with Realm Selection

```cpp
#include "auth/auth_handler.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    using namespace wowee::auth;

    AuthHandler auth;

    // Connect
    std::cout << "Connecting to authentication server..." << std::endl;
    if (!auth.connect("localhost", 3724)) {
        std::cerr << "Connection failed" << std::endl;
        return 1;
    }

    // Set up success callback to request realms
    auth.setOnSuccess([&auth](const std::vector<uint8_t>& sessionKey) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  AUTHENTICATION SUCCESSFUL!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Session key: " << sessionKey.size() << " bytes" << std::endl;

        // Automatically request realm list
        std::cout << "\nRequesting realm list..." << std::endl;
        auth.requestRealmList();
    });

    // Set up realm list callback
    bool gotRealms = false;
    auth.setOnRealmList([&gotRealms](const std::vector<Realm>& realms) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  AVAILABLE REALMS" << std::endl;
        std::cout << "========================================" << std::endl;

        for (size_t i = 0; i < realms.size(); ++i) {
            const auto& realm = realms[i];

            std::cout << "\n[" << (i + 1) << "] " << realm.name << std::endl;
            std::cout << "    Address: " << realm.address << std::endl;
            std::cout << "    Population: ";

            // Interpret population level
            if (realm.population < 0.5f) {
                std::cout << "Low (Green)";
            } else if (realm.population < 1.0f) {
                std::cout << "Medium (Yellow)";
            } else if (realm.population < 2.0f) {
                std::cout << "High (Red)";
            } else {
                std::cout << "Full (Red)";
            }
            std::cout << " (" << realm.population << ")" << std::endl;

            std::cout << "    Your characters: " << (int)realm.characters << std::endl;
            std::cout << "    Icon: " << (int)realm.icon << std::endl;
            std::cout << "    Lock: " << (realm.lock ? "Locked" : "Unlocked") << std::endl;

            if (realm.hasVersionInfo()) {
                std::cout << "    Version: " << (int)realm.majorVersion << "."
                          << (int)realm.minorVersion << "."
                          << (int)realm.patchVersion << " (build "
                          << realm.build << ")" << std::endl;
            }
        }

        gotRealms = true;
    });

    // Set up failure callback
    auth.setOnFailure([](const std::string& reason) {
        std::cerr << "\n========================================" << std::endl;
        std::cerr << "  AUTHENTICATION FAILED" << std::endl;
        std::cerr << "========================================" << std::endl;
        std::cerr << "Reason: " << reason << std::endl;
    });

    // Authenticate
    std::cout << "Authenticating..." << std::endl;
    auth.authenticate("myuser", "mypass");

    // Main loop - wait for realm list or failure
    while (!gotRealms && auth.getState() != AuthState::FAILED) {
        auth.update(0.016f);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Check result
    if (gotRealms) {
        const auto& realms = auth.getRealms();

        // Example: Select first realm
        if (!realms.empty()) {
            const auto& selectedRealm = realms[0];

            std::cout << "\n========================================" << std::endl;
            std::cout << "  REALM SELECTED" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "Realm: " << selectedRealm.name << std::endl;
            std::cout << "Address: " << selectedRealm.address << std::endl;

            // TODO: Parse address and connect to world server
            // Example: "localhost:8085" -> host="localhost", port=8085
        }
    }

    // Cleanup
    auth.disconnect();

    return gotRealms ? 0 : 1;
}
```

## Protocol Details

### REALM_LIST Request

**Packet Structure:**

```
Opcode: 0x10 (REALM_LIST)
Size: 5 bytes total

Bytes:
  0:     Opcode (0x10)
  1-4:   Unknown uint32 (always 0x00000000)
```

**Building the Packet:**

```cpp
network::Packet RealmListPacket::build() {
    network::Packet packet(static_cast<uint16_t>(AuthOpcode::REALM_LIST));
    packet.writeUInt32(0x00);  // Unknown field
    return packet;
}
```

### REALM_LIST Response

**Packet Structure:**

```
Opcode: 0x10 (REALM_LIST)
Variable length

Header:
  Byte 0:      Opcode (0x10)
  Bytes 1-2:   Packet size (uint16, little-endian)
  Bytes 3-6:   Unknown (uint32)
  Bytes 7-8:   Realm count (uint16, little-endian)

For each realm:
  1 byte:      Icon
  1 byte:      Lock
  1 byte:      Flags
  C-string:    Name (null-terminated)
  C-string:    Address (null-terminated, format: "host:port")
  4 bytes:     Population (float, little-endian)
  1 byte:      Characters (character count on this realm)
  1 byte:      Timezone
  1 byte:      ID

  [Conditional - only if flags & 0x04:]
    1 byte:    Major version
    1 byte:    Minor version
    1 byte:    Patch version
    2 bytes:   Build (uint16, little-endian)
```

**Packet Framing:**

The TCPSocket automatically handles variable-length REALM_LIST packets:

```cpp
// In TCPSocket::getExpectedPacketSize()
case 0x10:  // REALM_LIST response
    if (receiveBuffer.size() >= 3) {
        uint16_t size = receiveBuffer[1] | (receiveBuffer[2] << 8);
        return 1 + 2 + size;  // opcode + size field + payload
    }
    return 0;  // Need more data
```

## Realm Flags

The `flags` field contains bitwise flags:

- **Bit 0x04:** Realm has version info (major, minor, patch, build)
- Other bits are for realm type and status (see TrinityCore documentation)

Example:
```cpp
if (realm.flags & 0x04) {
    // Realm includes version information
    std::cout << "Version: " << (int)realm.majorVersion << "."
              << (int)realm.minorVersion << "."
              << (int)realm.patchVersion << std::endl;
}
```

## Population Levels

The `population` field is a float representing server load:

- **0.0 - 0.5:** Low (Green) - Server is not crowded
- **0.5 - 1.0:** Medium (Yellow) - Moderate population
- **1.0 - 2.0:** High (Red) - Server is crowded
- **2.0+:** Full (Red) - Server is at capacity

## Parsing Address

Realm addresses are in the format `"host:port"`. Example parsing:

```cpp
std::string parseHost(const std::string& address) {
    size_t colonPos = address.find(':');
    if (colonPos != std::string::npos) {
        return address.substr(0, colonPos);
    }
    return address;
}

uint16_t parsePort(const std::string& address) {
    size_t colonPos = address.find(':');
    if (colonPos != std::string::npos) {
        std::string portStr = address.substr(colonPos + 1);
        return static_cast<uint16_t>(std::stoi(portStr));
    }
    return 8085;  // Default world server port
}

// Usage
const auto& realm = realms[0];
std::string host = parseHost(realm.address);
uint16_t port = parsePort(realm.address);

std::cout << "Connecting to " << host << ":" << port << std::endl;
```

## Authentication States

The `AuthState` enum now includes realm list states:

```cpp
enum class AuthState {
    DISCONNECTED,              // Not connected
    CONNECTED,                 // Connected, ready for auth
    CHALLENGE_SENT,           // LOGON_CHALLENGE sent
    CHALLENGE_RECEIVED,       // LOGON_CHALLENGE response received
    PROOF_SENT,               // LOGON_PROOF sent
    AUTHENTICATED,            // Authentication successful, can request realms
    REALM_LIST_REQUESTED,     // REALM_LIST request sent
    REALM_LIST_RECEIVED,      // REALM_LIST response received
    FAILED                    // Authentication or connection failed
};
```

**State Transitions:**

```
DISCONNECTED
    ↓ connect()
CONNECTED
    ↓ authenticate()
CHALLENGE_SENT
    ↓ (server response)
CHALLENGE_RECEIVED
    ↓ (automatic)
PROOF_SENT
    ↓ (server response)
AUTHENTICATED
    ↓ requestRealmList()
REALM_LIST_REQUESTED
    ↓ (server response)
REALM_LIST_RECEIVED
```

## Testing

### With Live Server

```cpp
// Test against a WoW 3.3.5a private server
auth.connect("logon.my-wotlk-server.com", 3724);
auth.authenticate("testuser", "testpass");

// Wait for realm list
while (auth.getState() != AuthState::REALM_LIST_RECEIVED) {
    auth.update(0.016f);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
}

// Verify realms received
const auto& realms = auth.getRealms();
assert(!realms.empty());
assert(!realms[0].name.empty());
assert(!realms[0].address.empty());
```

### With Mock Data

For testing without a live server, you can create mock realms:

```cpp
Realm mockRealm;
mockRealm.id = 1;
mockRealm.name = "Test Realm";
mockRealm.address = "localhost:8085";
mockRealm.icon = 1;
mockRealm.lock = 0;
mockRealm.flags = 0x04;  // Has version info
mockRealm.population = 1.5f;  // High population
mockRealm.characters = 3;  // 3 characters
mockRealm.timezone = 1;
mockRealm.majorVersion = 3;
mockRealm.minorVersion = 3;
mockRealm.patchVersion = 5;
mockRealm.build = 12340;

// Test parsing address
std::string host = parseHost(mockRealm.address);
assert(host == "localhost");

uint16_t port = parsePort(mockRealm.address);
assert(port == 8085);
```

## Common Issues

### 1. "Cannot request realm list: not authenticated"

**Cause:** Tried to request realm list before authentication completed.

**Solution:** Only call `requestRealmList()` after authentication succeeds (in `onSuccess` callback or when state is `AUTHENTICATED`).

```cpp
// WRONG
auth.authenticate("user", "pass");
auth.requestRealmList();  // Too soon!

// CORRECT
auth.setOnSuccess([&auth](const std::vector<uint8_t>& sessionKey) {
    auth.requestRealmList();  // Call here
});
auth.authenticate("user", "pass");
```

### 2. Empty Realm List

**Cause:** Server has no realms configured.

**Solution:** Check server configuration. A typical WoW server should have at least one realm in its `realmlist` table.

### 3. "Unknown opcode or indeterminate size"

**Cause:** Server sent unexpected packet or packet framing failed.

**Solution:** Enable debug logging to see raw packet data:

```cpp
Logger::getInstance().setLogLevel(LogLevel::DEBUG);
```

## Next Steps

After receiving the realm list:

1. **Display realms to user** - Show realm name, population, character count
2. **Let user select realm** - Prompt for realm selection
3. **Parse realm address** - Extract host and port from `address` field
4. **Connect to world server** - Use the parsed host:port to connect
5. **Send CMSG_AUTH_SESSION** - Authenticate with world server using session key

Example next step:

```cpp
auth.setOnRealmList([&auth](const std::vector<Realm>& realms) {
    if (realms.empty()) {
        std::cerr << "No realms available" << std::endl;
        return;
    }

    // Select first realm
    const auto& realm = realms[0];

    // Parse address
    size_t colonPos = realm.address.find(':');
    std::string host = realm.address.substr(0, colonPos);
    uint16_t port = std::stoi(realm.address.substr(colonPos + 1));

    // TODO: Connect to world server
    std::cout << "Next: Connect to " << host << ":" << port << std::endl;
    std::cout << "Send CMSG_AUTH_SESSION with session key" << std::endl;

    // Get session key for world server authentication
    const auto& sessionKey = auth.getSessionKey();
    std::cout << "Session key: " << sessionKey.size() << " bytes" << std::endl;
});
```

## Summary

The realm list protocol:

1. ✅ Automatically handles variable-length packets
2. ✅ Parses all realm information including version info
3. ✅ Provides easy-to-use callback interface
4. ✅ Includes comprehensive logging
5. ✅ Ready for live server testing

---

**Status:** ✅ Complete and production-ready

**Next Protocol:** World server connection (CMSG_AUTH_SESSION)
