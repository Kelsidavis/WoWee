#pragma once

#include "auth/auth_opcodes.hpp"
#include "network/packet.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <array>

namespace wowee {
namespace auth {

// Client build and version information
struct ClientInfo {
    uint8_t majorVersion = 3;
    uint8_t minorVersion = 3;
    uint8_t patchVersion = 5;
    uint16_t build = 12340;  // 3.3.5a
    uint8_t protocolVersion = 8; // SRP auth protocol version
    std::string game = "WoW";
    std::string platform = "x86";
    std::string os = "Win";
    std::string locale = "enUS";
    uint32_t timezone = 0;
};

// LOGON_CHALLENGE packet builder
class LogonChallengePacket {
public:
    static network::Packet build(const std::string& account, const ClientInfo& info = ClientInfo());
};

// LOGON_CHALLENGE response data
struct LogonChallengeResponse {
    AuthResult result;
    std::vector<uint8_t> B;     // Server public ephemeral (32 bytes)
    std::vector<uint8_t> g;     // Generator (variable, usually 1 byte)
    std::vector<uint8_t> N;     // Prime modulus (variable, usually 256 bytes)
    std::vector<uint8_t> salt;  // Salt (32 bytes)
    uint8_t securityFlags;

    // PIN extension (securityFlags & 0x01)
    uint32_t pinGridSeed = 0;
    std::array<uint8_t, 16> pinSalt{};

    // Authenticator extension (securityFlags & 0x04)
    uint8_t authenticatorRequired = 0;

    bool isSuccess() const { return result == AuthResult::SUCCESS; }
};

// LOGON_CHALLENGE response parser
class LogonChallengeResponseParser {
public:
    static bool parse(network::Packet& packet, LogonChallengeResponse& response);
};

// LOGON_PROOF packet builder
class LogonProofPacket {
public:
    static network::Packet build(const std::vector<uint8_t>& A,
                                  const std::vector<uint8_t>& M1);
    // Legacy (protocol < 8): A(32) + M1(20) + crc(20) + number_of_keys(1). No securityFlags byte.
    static network::Packet buildLegacy(const std::vector<uint8_t>& A,
                                       const std::vector<uint8_t>& M1);
    static network::Packet build(const std::vector<uint8_t>& A,
                                  const std::vector<uint8_t>& M1,
                                  uint8_t securityFlags,
                                  const std::array<uint8_t, 16>* pinClientSalt,
                                  const std::array<uint8_t, 20>* pinHash);
};

// AUTHENTICATOR token packet builder (opcode 0x04 on many TrinityCore-derived servers)
class AuthenticatorTokenPacket {
public:
    static network::Packet build(const std::string& token);
};

// LOGON_PROOF response data
struct LogonProofResponse {
    uint8_t status;
    std::vector<uint8_t> M2;  // Server proof (20 bytes)

    bool isSuccess() const { return status == 0; }
};

// LOGON_PROOF response parser
class LogonProofResponseParser {
public:
    static bool parse(network::Packet& packet, LogonProofResponse& response);
};

// Realm data structure
struct Realm {
    uint8_t icon;
    uint8_t lock;
    uint8_t flags;
    std::string name;
    std::string address;
    float population;
    uint8_t characters;
    uint8_t timezone;
    uint8_t id;

    // Version info (conditional - only if flags & 0x04)
    uint8_t majorVersion = 0;
    uint8_t minorVersion = 0;
    uint8_t patchVersion = 0;
    uint16_t build = 0;

    bool hasVersionInfo() const { return (flags & 0x04) != 0; }
};

// REALM_LIST packet builder
class RealmListPacket {
public:
    static network::Packet build();
};

// REALM_LIST response data
struct RealmListResponse {
    std::vector<Realm> realms;
};

// REALM_LIST response parser
class RealmListResponseParser {
public:
    static bool parse(network::Packet& packet, RealmListResponse& response);
};

} // namespace auth
} // namespace wowee
