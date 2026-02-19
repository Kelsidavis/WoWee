# Complete Authentication Guide - Auth Server to World Server

## Overview

This guide demonstrates the complete authentication flow in wowee, from connecting to the auth server through world server authentication. This represents the complete implementation of WoW 3.3.5a client authentication.

## Complete Authentication Flow

```
┌─────────────────────────────────────────────┐
│ 1. AUTH SERVER AUTHENTICATION               │
│    ✅ Connect to auth server (3724)         │
│    ✅ LOGON_CHALLENGE / LOGON_PROOF         │
│    ✅ SRP6a cryptography                    │
│    ✅ Get 40-byte session key               │
└─────────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────────┐
│ 2. REALM LIST RETRIEVAL                     │
│    ✅ REALM_LIST request                    │
│    ✅ Parse realm data                      │
│    ✅ Select realm                          │
└─────────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────────┐
│ 3. WORLD SERVER CONNECTION                  │
│    ✅ Connect to world server (realm port)  │
│    ✅ SMSG_AUTH_CHALLENGE                   │
│    ✅ CMSG_AUTH_SESSION                     │
│    ✅ Initialize RC4 encryption             │
│    ✅ SMSG_AUTH_RESPONSE                    │
└─────────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────────┐
│ 4. READY FOR CHARACTER OPERATIONS           │
│    🎯 CMSG_CHAR_ENUM (next step)            │
│    🎯 Character selection                   │
│    🎯 CMSG_PLAYER_LOGIN                     │
└─────────────────────────────────────────────┘
```

## Complete Code Example

```cpp
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "core/logger.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace wowee;

int main() {
    // Enable debug logging
    core::Logger::getInstance().setLogLevel(core::LogLevel::DEBUG);

    // ========================================
    // PHASE 1: AUTH SERVER AUTHENTICATION
    // ========================================

    std::cout << "\n=== PHASE 1: AUTH SERVER AUTHENTICATION ===" << std::endl;

    auth::AuthHandler authHandler;

    // Stored data for world server
    std::vector<uint8_t> sessionKey;
    std::string accountName = "MYACCOUNT";
    std::string selectedRealmAddress;
    uint16_t selectedRealmPort;

    // Connect to auth server
    if (!authHandler.connect("logon.myserver.com", 3724)) {
        std::cerr << "Failed to connect to auth server" << std::endl;
        return 1;
    }

    // Set up auth success callback
    bool authSuccess = false;
    authHandler.setOnSuccess([&](const std::vector<uint8_t>& key) {
        std::cout << "\n[SUCCESS] Authenticated with auth server!" << std::endl;
        std::cout << "Session key: " << key.size() << " bytes" << std::endl;

        // Store session key for world server
        sessionKey = key;
        authSuccess = true;

        // Request realm list
        std::cout << "\nRequesting realm list..." << std::endl;
        authHandler.requestRealmList();
    });

    // Set up realm list callback
    bool gotRealms = false;
    authHandler.setOnRealmList([&](const std::vector<auth::Realm>& realms) {
        std::cout << "\n[SUCCESS] Received realm list!" << std::endl;
        std::cout << "Available realms: " << realms.size() << std::endl;

        // Display realms
        for (size_t i = 0; i < realms.size(); ++i) {
            const auto& realm = realms[i];
            std::cout << "\n[" << (i + 1) << "] " << realm.name << std::endl;
            std::cout << "    Address: " << realm.address << std::endl;
            std::cout << "    Population: " << realm.population << std::endl;
            std::cout << "    Characters: " << (int)realm.characters << std::endl;
        }

        // Select first realm
        if (!realms.empty()) {
            const auto& realm = realms[0];
            std::cout << "\n[SELECTED] " << realm.name << std::endl;

            // Parse realm address (format: "host:port")
            size_t colonPos = realm.address.find(':');
            if (colonPos != std::string::npos) {
                std::string host = realm.address.substr(0, colonPos);
                uint16_t port = std::stoi(realm.address.substr(colonPos + 1));

                selectedRealmAddress = host;
                selectedRealmPort = port;
                gotRealms = true;
            } else {
                std::cerr << "Invalid realm address format" << std::endl;
            }
        }
    });

    // Set up failure callback
    authHandler.setOnFailure([](const std::string& reason) {
        std::cerr << "\n[FAILED] Authentication failed: " << reason << std::endl;
    });

    // Start authentication
    std::cout << "Authenticating as: " << accountName << std::endl;
    authHandler.authenticate(accountName, "mypassword");

    // Wait for auth and realm list
    while (!gotRealms &&
           authHandler.getState() != auth::AuthState::FAILED) {
        authHandler.update(0.016f);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Check if authentication succeeded
    if (!authSuccess || sessionKey.empty()) {
        std::cerr << "Authentication failed" << std::endl;
        return 1;
    }

    if (!gotRealms) {
        std::cerr << "Failed to get realm list" << std::endl;
        return 1;
    }

    // ========================================
    // PHASE 2: WORLD SERVER CONNECTION
    // ========================================

    std::cout << "\n=== PHASE 2: WORLD SERVER CONNECTION ===" << std::endl;
    std::cout << "Connecting to: " << selectedRealmAddress << ":"
              << selectedRealmPort << std::endl;

    game::GameHandler gameHandler;

    // Set up world connection callbacks
    bool worldSuccess = false;
    gameHandler.setOnSuccess([&worldSuccess]() {
        std::cout << "\n[SUCCESS] Connected to world server!" << std::endl;
        std::cout << "Ready for character operations" << std::endl;
        worldSuccess = true;
    });

    gameHandler.setOnFailure([](const std::string& reason) {
        std::cerr << "\n[FAILED] World connection failed: " << reason << std::endl;
    });

    // Connect to world server with session key from auth server
    if (!gameHandler.connect(
            selectedRealmAddress,
            selectedRealmPort,
            sessionKey,         // 40-byte session key from auth server
            accountName,        // Same account name
            12340               // WoW 3.3.5a build
        )) {
        std::cerr << "Failed to initiate world server connection" << std::endl;
        return 1;
    }

    // Wait for world authentication to complete
    while (!worldSuccess &&
           gameHandler.getState() != game::WorldState::FAILED) {
        gameHandler.update(0.016f);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Check result
    if (!worldSuccess) {
        std::cerr << "World server connection failed" << std::endl;
        return 1;
    }

    // ========================================
    // PHASE 3: READY FOR GAME
    // ========================================

    std::cout << "\n=== PHASE 3: READY FOR CHARACTER OPERATIONS ===" << std::endl;
    std::cout << "✅ Auth server: Authenticated" << std::endl;
    std::cout << "✅ Realm list: Received" << std::endl;
    std::cout << "✅ World server: Connected" << std::endl;
    std::cout << "✅ Encryption: Initialized" << std::endl;
    std::cout << "\n🎮 Ready to request character list!" << std::endl;

    // TODO: Next steps:
    // - Send CMSG_CHAR_ENUM
    // - Receive SMSG_CHAR_ENUM
    // - Display characters
    // - Send CMSG_PLAYER_LOGIN
    // - Enter world!

    // Keep connection alive
    std::cout << "\nPress Ctrl+C to exit..." << std::endl;
    while (true) {
        gameHandler.update(0.016f);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
```

## Step-by-Step Explanation

### Phase 1: Auth Server Authentication

#### 1.1 Connect to Auth Server

```cpp
auth::AuthHandler authHandler;
authHandler.connect("logon.myserver.com", 3724);
```

**What happens:**
- TCP connection to auth server port 3724
- Connection state changes to `CONNECTED`

#### 1.2 Authenticate with SRP6a

```cpp
authHandler.authenticate("MYACCOUNT", "mypassword");
```

**What happens:**
- Sends `LOGON_CHALLENGE` packet
- Server responds with B, g, N, salt
- Computes SRP6a proof using password
- Sends `LOGON_PROOF` packet
- Server verifies and returns M2
- Session key (40 bytes) is generated

**Session Key Computation:**
```
S = (B - k*g^x)^(a + u*x) mod N
K = Interleave(SHA1(even_bytes(S)), SHA1(odd_bytes(S)))
  = 40 bytes
```

#### 1.3 Request Realm List

```cpp
authHandler.requestRealmList();
```

**What happens:**
- Sends `REALM_LIST` packet (5 bytes)
- Server responds with realm data
- Parses realm name, address, population, etc.

### Phase 2: Realm Selection

#### 2.1 Parse Realm Address

```cpp
const auto& realm = realms[0];
size_t colonPos = realm.address.find(':');
std::string host = realm.address.substr(0, colonPos);
uint16_t port = std::stoi(realm.address.substr(colonPos + 1));
```

**Realm address format:** `"localhost:8085"`

### Phase 3: World Server Connection

#### 3.1 Connect to World Server

```cpp
game::GameHandler gameHandler;
gameHandler.connect(
    host,           // e.g., "localhost"
    port,           // e.g., 8085
    sessionKey,     // 40 bytes from auth server
    accountName,    // Same account
    12340           // Build number
);
```

**What happens:**
- TCP connection to world server
- Generates random client seed
- Waits for `SMSG_AUTH_CHALLENGE`

#### 3.2 Handle SMSG_AUTH_CHALLENGE

**Server sends (unencrypted):**
```
Opcode: 0x01EC (SMSG_AUTH_CHALLENGE)
Data:
  uint32 unknown1 (always 1)
  uint32 serverSeed (random)
```

**Client receives:**
- Parses server seed
- Prepares to send authentication

#### 3.3 Send CMSG_AUTH_SESSION

**Client builds packet:**
```
Opcode: 0x01ED (CMSG_AUTH_SESSION)
Data:
  uint32 build (12340)
  uint32 unknown (0)
  string account (null-terminated, uppercase)
  uint32 unknown (0)
  uint32 clientSeed (random)
  uint32 unknown (0) x5
  uint8  authHash[20] (SHA1)
  uint32 addonCRC (0)
```

**Auth hash computation (CRITICAL):**
```cpp
SHA1(
    account_name +
    [0, 0, 0, 0] +
    client_seed (4 bytes, little-endian) +
    server_seed (4 bytes, little-endian) +
    session_key (40 bytes)
)
```

**Client sends:**
- Packet sent unencrypted

#### 3.4 Initialize Encryption

**IMMEDIATELY after sending CMSG_AUTH_SESSION:**

```cpp
socket->initEncryption(sessionKey);
```

**What happens:**
```
1. encryptHash = HMAC-SHA1(ENCRYPT_KEY, sessionKey)  // 20 bytes
2. decryptHash = HMAC-SHA1(DECRYPT_KEY, sessionKey)  // 20 bytes

3. encryptCipher = RC4(encryptHash)
4. decryptCipher = RC4(decryptHash)

5. encryptCipher.drop(1024)  // Drop first 1024 bytes
6. decryptCipher.drop(1024)  // Drop first 1024 bytes

7. encryptionEnabled = true
```

**Hardcoded Keys (WoW 3.3.5a):**
```cpp
ENCRYPT_KEY = {0xC2, 0xB3, 0x72, 0x3C, 0xC6, 0xAE, 0xD9, 0xB5,
               0x34, 0x3C, 0x53, 0xEE, 0x2F, 0x43, 0x67, 0xCE};

DECRYPT_KEY = {0xCC, 0x98, 0xAE, 0x04, 0xE8, 0x97, 0xEA, 0xCA,
               0x12, 0xDD, 0xC0, 0x93, 0x42, 0x91, 0x53, 0x57};
```

#### 3.5 Handle SMSG_AUTH_RESPONSE

**Server sends (ENCRYPTED header):**
```
Header (4 bytes, encrypted):
  uint16 size (big-endian)
  uint16 opcode 0x01EE (big-endian)

Body (1 byte, plaintext):
  uint8 result (0x00 = success)
```

**Client receives:**
- Decrypts header with RC4
- Parses result code
- If 0x00: SUCCESS!
- Otherwise: Error message

### Phase 4: Ready for Game

At this point:
- ✅ Session established
- ✅ Encryption active
- ✅ All future packets have encrypted headers
- 🎯 Ready for character operations

## Error Handling

### Auth Server Errors

```cpp
authHandler.setOnFailure([](const std::string& reason) {
    // Possible reasons:
    // - "ACCOUNT_INVALID"
    // - "PASSWORD_INVALID"
    // - "ALREADY_ONLINE"
    // - "BUILD_INVALID"
    // etc.
});
```

### World Server Errors

```cpp
gameHandler.setOnFailure([](const std::string& reason) {
    // Possible reasons:
    // - "Connection failed"
    // - "Authentication failed: ALREADY_LOGGING_IN"
    // - "Authentication failed: SESSION_EXPIRED"
    // etc.
});
```

## Testing

### Unit Test Example

```cpp
void testCompleteAuthFlow() {
    // Mock auth server
    MockAuthServer authServer(3724);

    // Real auth handler
    auth::AuthHandler auth;
    auth.connect("localhost", 3724);

    bool success = false;
    std::vector<uint8_t> key;

    auth.setOnSuccess([&](const std::vector<uint8_t>& sessionKey) {
        success = true;
        key = sessionKey;
    });

    auth.authenticate("TEST", "TEST");

    // Wait for result
    while (auth.getState() == auth::AuthState::CHALLENGE_SENT ||
           auth.getState() == auth::AuthState::PROOF_SENT) {
        auth.update(0.016f);
    }

    assert(success);
    assert(key.size() == 40);

    // Now test world server
    MockWorldServer worldServer(8085);

    game::GameHandler game;
    game.connect("localhost", 8085, key, "TEST", 12340);

    bool worldSuccess = false;
    game.setOnSuccess([&worldSuccess]() {
        worldSuccess = true;
    });

    while (game.getState() != game::WorldState::READY &&
           game.getState() != game::WorldState::FAILED) {
        game.update(0.016f);
    }

    assert(worldSuccess);
}
```

## Common Issues

### 1. "Invalid session key size"

**Cause:** Session key from auth server is not 40 bytes

**Solution:** Verify SRP implementation. Session key must be exactly 40 bytes (interleaved SHA1 hashes).

### 2. "Authentication failed: ALREADY_LOGGING_IN"

**Cause:** Character already logged in on world server

**Solution:** Wait or restart world server.

### 3. Encryption Mismatch

**Symptoms:** World server disconnects after CMSG_AUTH_SESSION

**Cause:** Encryption initialized at wrong time or with wrong key

**Solution:** Ensure encryption is initialized AFTER sending CMSG_AUTH_SESSION but BEFORE receiving SMSG_AUTH_RESPONSE.

### 4. Auth Hash Mismatch

**Symptoms:** SMSG_AUTH_RESPONSE returns error code

**Cause:** SHA1 hash computed incorrectly

**Solution:** Verify hash computation:
```cpp
// Must be exact order:
1. Account name (string bytes)
2. Four null bytes [0,0,0,0]
3. Client seed (4 bytes, little-endian)
4. Server seed (4 bytes, little-endian)
5. Session key (40 bytes)
```

## Next Steps

After successful world authentication:

1. **Character Enumeration**
   ```cpp
   // Send CMSG_CHAR_ENUM (0x0037)
   // Receive SMSG_CHAR_ENUM (0x003B)
   // Display character list
   ```

2. **Enter World**
   ```cpp
   // Send CMSG_PLAYER_LOGIN (0x003D) with character GUID
   // Receive SMSG_LOGIN_VERIFY_WORLD (0x0236)
   // Now in game!
   ```

3. **Game Packets**
   - Movement (CMSG_MOVE_*)
   - Chat (CMSG_MESSAGECHAT)
   - Spells (CMSG_CAST_SPELL)
   - etc.

## Summary

This guide demonstrates the **complete authentication flow** from auth server to world server:

1. ✅ **Auth Server:** SRP6a authentication → Session key
2. ✅ **Realm List:** Request and parse realm data
3. ✅ **World Server:** RC4-encrypted authentication
4. ✅ **Ready:** All protocols implemented and working

The client is now ready for character operations and world entry! 🎮

---

**Implementation Status:** 100% Complete for authentication
**Next Milestone:** Character enumeration and world entry
