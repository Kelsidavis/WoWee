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
│ 4. CHARACTER OPERATIONS                     │
│    ✅ CMSG_CHAR_ENUM (sent on AUTH_OK)      │
│    ✅ Character selection                   │
│    ✅ CMSG_PLAYER_LOGIN                     │
└─────────────────────────────────────────────┘
```

## Complete Code Example

```cpp
#include "auth/auth_handler.hpp"
#include "game/game_handler.hpp"
#include "game/game_services.hpp"
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
    uint32_t selectedRealmId = 0;

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
                selectedRealmId = realm.id;
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

    // Renderer, audio and asset manager; the client's Application fills these
    game::GameServices services;
    game::GameHandler gameHandler(services);

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
            12340,              // WoW 3.3.5a build
            selectedRealmId     // Realm id from the realm list
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

    // GameHandler sent CMSG_CHAR_ENUM itself on AUTH_OK. Once the state
    // reaches CHAR_LIST_RECEIVED, gameHandler.getCharacters() holds the list
    // and gameHandler.selectCharacter(guid) sends CMSG_PLAYER_LOGIN.

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

If the challenge's security flags ask for a PIN (0x01) or an authenticator code (0x04), the handler stops in `PIN_REQUIRED` or `AUTHENTICATOR_REQUIRED` until `submitSecurityCode()` supplies it, or takes it up front from the `authenticate(user, pass, pin)` overload.

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
game::GameHandler gameHandler(services);
gameHandler.connect(
    host,           // e.g., "localhost"
    port,           // e.g., 8085
    sessionKey,     // 40 bytes from auth server
    accountName,    // Same account
    12340,          // Build number
    realm.id        // Realm id; some servers reject 0
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
  uint8  seeds[32] (not used)
```

`AuthChallengeParser` also accepts the 4-byte TBC form and the 36-byte classic form, both of which start with the server seed.

**Client receives:**
- Parses server seed
- Prepares to send authentication

#### 3.3 Send CMSG_AUTH_SESSION

**Client builds packet:**
```
Opcode: 0x01ED (CMSG_AUTH_SESSION)
Data:
  uint32 build (12340)
  uint32 loginServerId (0)
  string account (null-terminated, uppercase)
  uint32 loginServerType (0)
  uint32 clientSeed (random)
  uint32 regionId (0)
  uint32 battlegroupId (0)
  uint32 realmId (from the realm list)
  uint64 dosResponse (0)
  uint8  authHash[20] (SHA1)
  uint32 addonInfoSize (uncompressed size)
  uint8  addonInfo[] (zlib: uint32 addonCount 0, uint32 clientTime 0)
```

Builds up to 8606 (TBC) send the shorter layout: build, realm id, account, client seed, hash, addon info.

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
socket->initEncryption(sessionKey, build);
```

The build picks the header cipher: 5875 and below use the vanilla XOR cipher, 5876 to 8606 the CMaNGOS TBC HMAC-XOR cipher, and anything later RC4 as below.

**What happens (RC4):**
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
  uint16 size (big-endian, includes the opcode)
  uint16 opcode 0x01EE (little-endian)

Body (plaintext):
  uint8 result (0x0C = AUTH_OK)
  ... (billing and expansion fields, not read)
```

**Client receives:**
- Decrypts header with RC4
- Parses result code
- If 0x0C (AUTH_OK): state goes to `READY` and `CMSG_CHAR_ENUM` is sent
- Otherwise: Error message

### Phase 4: Ready for Game

At this point:
- ✅ Session established
- ✅ Encryption active
- ✅ All future packets have encrypted headers
- ✅ Character list requested

## Error Handling

### Auth Server Errors

```cpp
authHandler.setOnFailure([](const std::string& reason) {
    // Possible reasons:
    // - "LOGON_CHALLENGE failed: Account not found - check your username"
    // - "LOGON_CHALLENGE failed: This account is already logged in"
    // - "LOGON_CHALLENGE failed: version mismatch (client v3.3.5 build 12340, auth protocol 8)"
    // - "Login failed: <result>" (LOGON_PROOF rejected, e.g. a wrong password)
    // - "Disconnected by auth server"
    // etc.
});
```

`lastFailureWasProtocol()` is true when the failure looks like an auth protocol mismatch rather than bad credentials, so the caller can retry with another protocol version.

### World Server Errors

```cpp
gameHandler.setOnFailure([](const std::string& reason) {
    // Possible reasons:
    // - "Connection failed"
    // - "Authentication failed: ALREADY_LOGGING_IN - Already logging in"
    // - "Authentication failed: SESSION_EXPIRED - Session has expired"
    // etc.
});
```

## Testing

### Unit Tests

There is no mock auth or world server in the tree; the handlers are exercised against live servers. The pieces they are built from have tests of their own:

- `tests/test_srp.cpp` (`ctest -R srp`): SRP6a sizes, a non-zero A, different passwords giving different M1, and M2 rejection
- `tests/test_realm_list.cpp` (`ctest -R realm_list`): REALM_LIST parsing across the vanilla and TBC/WotLK layouts, and the LOGON_PROOF legacy and PIN layouts

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
   // CMSG_CHAR_ENUM (0x0037) is sent by GameHandler on AUTH_OK
   // SMSG_CHAR_ENUM (0x003B) fills gameHandler.getCharacters()
   // requestCharacterList() asks again
   ```

2. **Enter World**
   ```cpp
   // gameHandler.selectCharacter(guid) sends CMSG_PLAYER_LOGIN (0x003D)
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

**Implementation Status:** Complete - authentication, character enumeration, and world entry all working.
