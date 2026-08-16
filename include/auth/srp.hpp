#pragma once

#include "auth/big_num.hpp"
#include <vector>
#include <cstdint>
#include <string>

namespace wowee {
namespace auth {

// SRP6a implementation for World of Warcraft authentication
// Based on the original wowee JavaScript implementation
class SRP {
public:
    SRP();
    ~SRP() = default;

    // Initialize with username and password
    void initialize(const std::string& username, const std::string& password);

    // Initialize with username and pre-computed auth hash (SHA1(UPPER(user):UPPER(pass)))
    void initializeWithHash(const std::string& username, const std::vector<uint8_t>& authHash);

    // Feed server challenge data (B, g, N, salt)
    void feed(const std::vector<uint8_t>& B,
              const std::vector<uint8_t>& g,
              const std::vector<uint8_t>& N,
              const std::vector<uint8_t>& salt);

    // Some SRP implementations use k = H(N|g) instead of the WoW-specific k=3.
    // Default is false (k=3).
    void setUseHashedK(bool enabled) { useHashedK_ = enabled; }

    // Controls how SHA1 outputs are interpreted when converted to big integers (x, u, optionally k).
    // Many SRP implementations treat hash outputs as big-endian integers.
    // Default is false (treat hash outputs as little-endian integers).
    void setHashBigEndian(bool enabled) { hashBigEndian_ = enabled; }

    // Get client public ephemeral (A) - send to server
    [[nodiscard]] std::vector<uint8_t> getA() const;

    // Get client proof (M1) - send to server
    [[nodiscard]] std::vector<uint8_t> getM1() const;

    // Verify server proof (M2)
    [[nodiscard]] bool verifyServerProof(const std::vector<uint8_t>& serverM2) const;

    // Get session key (K) - used for encryption
    [[nodiscard]] std::vector<uint8_t> getSessionKey() const;

    // Securely erase stored plaintext credentials from memory.
    // Called automatically at the end of feed() once the SRP values are computed.
    void clearCredentials();

private:
    // WoW-specific SRP multiplier (k = 3)
    static constexpr uint32_t K_VALUE = 3;

    // Helper methods
    [[nodiscard]] std::vector<uint8_t> computeAuthHash(const std::string& username,
                                          const std::string& password) const;
    void computeClientEphemeral();
    void computeSessionKey();
    void computeProofs(const std::string& username);

    // SRP values
    BigNum g;         // Generator
    BigNum N;         // Prime modulus
    BigNum k;         // Multiplier (3 for WoW)
    BigNum s;         // Salt
    BigNum a;         // Client private ephemeral
    BigNum A;         // Client public ephemeral
    BigNum B;         // Server public ephemeral
    BigNum x;         // Salted password hash
    BigNum u;         // Scrambling parameter
    BigNum S;         // Shared session key (raw)

    // Derived values
    std::vector<uint8_t> K;   // Interleaved session key (40 bytes)
    std::vector<uint8_t> M1;  // Client proof (20 bytes)
    std::vector<uint8_t> M2;  // Expected server proof (20 bytes)

    // Stored credentials
    std::string stored_username;
    std::string stored_password;
    std::vector<uint8_t> stored_auth_hash;  // Pre-computed SHA1(UPPER(user):UPPER(pass))

    bool initialized = false;
    bool useHashedK_ = false;
    bool hashBigEndian_ = false;
};

} // namespace auth
} // namespace wowee
