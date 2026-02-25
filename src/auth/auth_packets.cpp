#include "auth/auth_packets.hpp"
#include "core/logger.hpp"
#include "network/net_platform.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>
#include <array>

namespace wowee {
namespace auth {

namespace {
bool detectOutboundIPv4(std::array<uint8_t, 4>& outIp) {
    net::ensureInit();

    socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCK) {
        return false;
    }

    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(53);
    if (inet_pton(AF_INET, "1.1.1.1", &remote.sin_addr) != 1) {
        net::closeSocket(s);
        return false;
    }

    if (::connect(s, reinterpret_cast<sockaddr*>(&remote), sizeof(remote)) != 0) {
        net::closeSocket(s);
        return false;
    }

    sockaddr_in local{};
#ifdef _WIN32
    int localLen = sizeof(local);
#else
    socklen_t localLen = sizeof(local);
#endif
    if (::getsockname(s, reinterpret_cast<sockaddr*>(&local), &localLen) != 0) {
        net::closeSocket(s);
        return false;
    }

    net::closeSocket(s);

    const uint32_t ip = ntohl(local.sin_addr.s_addr);
    outIp[0] = static_cast<uint8_t>((ip >> 24) & 0xFF);
    outIp[1] = static_cast<uint8_t>((ip >> 16) & 0xFF);
    outIp[2] = static_cast<uint8_t>((ip >> 8) & 0xFF);
    outIp[3] = static_cast<uint8_t>(ip & 0xFF);

    return (ip != 0);
}
} // namespace

network::Packet LogonChallengePacket::build(const std::string& account, const ClientInfo& info) {
    // Convert account to uppercase
    std::string upperAccount = account;
    std::transform(upperAccount.begin(), upperAccount.end(), upperAccount.begin(), ::toupper);

    // Calculate payload size (everything after cmd + error + size)
    // game(4) + version(3) + build(2) + platform(4) + os(4) + locale(4) +
    // timezone(4) + ip(4) + accountLen(1) + account(N)
    uint16_t payloadSize = 30 + upperAccount.length();

    network::Packet packet(static_cast<uint16_t>(AuthOpcode::LOGON_CHALLENGE));

    // Protocol version (e.g. 8 for WoW 3.3.5a build 12340)
    packet.writeUInt8(info.protocolVersion);

    // Payload size
    packet.writeUInt16(payloadSize);

    // Write a 4-byte FourCC field with reversed string characters.
    // The auth server reads these as a C-string (stops at first null), then
    // reverses the string.  So we must send the characters reversed and
    // null-padded on the right.  E.g. "Win" → bytes ['n','i','W',0x00].
    // Server reads "niW", reverses → "Win", stores "Win".
    auto writeFourCC = [&packet](const std::string& str) {
        uint8_t buf[4] = {0, 0, 0, 0};
        size_t len = std::min<size_t>(4, str.length());
        // Write string characters in reverse order, then null-pad
        for (size_t i = 0; i < len; ++i) {
            buf[i] = static_cast<uint8_t>(str[len - 1 - i]);
        }
        for (int i = 0; i < 4; ++i) {
            packet.writeUInt8(buf[i]);
        }
    };

    // Game name (4 bytes, reversed FourCC)
    // "WoW" → bytes ['W','o','W',0x00] on the wire
    writeFourCC(info.game);

    // Version (3 bytes)
    packet.writeUInt8(info.majorVersion);
    packet.writeUInt8(info.minorVersion);
    packet.writeUInt8(info.patchVersion);

    // Build (2 bytes)
    packet.writeUInt16(info.build);

    // Platform (4 bytes)
    writeFourCC(info.platform);

    // OS (4 bytes)
    writeFourCC(info.os);

    // Locale (4 bytes)
    writeFourCC(info.locale);

    // Timezone
    packet.writeUInt32(info.timezone);

    // Client IP: use the real outbound local IPv4 when detectable.
    // Fallback to 0.0.0.0 if detection fails.
    {
        std::array<uint8_t, 4> localIp{0, 0, 0, 0};
        if (detectOutboundIPv4(localIp)) {
            packet.writeUInt8(localIp[0]);
            packet.writeUInt8(localIp[1]);
            packet.writeUInt8(localIp[2]);
            packet.writeUInt8(localIp[3]);
        } else {
            packet.writeUInt32(0);
            LOG_DEBUG("LOGON_CHALLENGE client IP detection failed; using 0.0.0.0 fallback");
        }
    }

    // Account length and name
    packet.writeUInt8(static_cast<uint8_t>(upperAccount.length()));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(upperAccount.c_str()),
                      upperAccount.length());

    LOG_DEBUG("Built LOGON_CHALLENGE packet for account: ", upperAccount);
    LOG_DEBUG("  Payload size: ", payloadSize, " bytes");
    LOG_DEBUG("  Total size: ", packet.getSize(), " bytes");

    return packet;
}

bool LogonChallengeResponseParser::parse(network::Packet& packet, LogonChallengeResponse& response) {
    // Note: opcode byte already consumed by handlePacket()

    // Unknown/protocol byte
    packet.readUInt8();

    // Status
    response.result = static_cast<AuthResult>(packet.readUInt8());

    LOG_INFO("LOGON_CHALLENGE response: ", getAuthResultString(response.result));

    if (response.result != AuthResult::SUCCESS) {
        return true;  // Valid packet, but authentication failed
    }

    // B (server public ephemeral) - 32 bytes
    response.B.resize(32);
    for (int i = 0; i < 32; ++i) {
        response.B[i] = packet.readUInt8();
    }

    // g length and value
    uint8_t gLen = packet.readUInt8();
    response.g.resize(gLen);
    for (uint8_t i = 0; i < gLen; ++i) {
        response.g[i] = packet.readUInt8();
    }

    // N length and value
    uint8_t nLen = packet.readUInt8();
    response.N.resize(nLen);
    for (uint8_t i = 0; i < nLen; ++i) {
        response.N[i] = packet.readUInt8();
    }

    // Salt - 32 bytes
    response.salt.resize(32);
    for (int i = 0; i < 32; ++i) {
        response.salt[i] = packet.readUInt8();
    }

    // Integrity salt / CRC salt - 16 bytes
    for (size_t i = 0; i < response.checksumSalt.size(); ++i) {
        response.checksumSalt[i] = packet.readUInt8();
    }

    // Security flags
    response.securityFlags = packet.readUInt8();

    // Optional security extensions (protocol v8+)
    if (response.securityFlags & 0x01) {
        // PIN required: u32 pin_grid_seed + u8[16] pin_salt
        response.pinGridSeed = packet.readUInt32();
        for (size_t i = 0; i < response.pinSalt.size(); ++i) {
            response.pinSalt[i] = packet.readUInt8();
        }
    }
    if (response.securityFlags & 0x04) {
        // Authenticator required (TrinityCore): u8 requiredFlag (usually 1)
        response.authenticatorRequired = packet.readUInt8();
    }

    LOG_DEBUG("Parsed LOGON_CHALLENGE response:");
    LOG_DEBUG("  B size: ", response.B.size(), " bytes");
    LOG_DEBUG("  g size: ", response.g.size(), " bytes");
    LOG_DEBUG("  N size: ", response.N.size(), " bytes");
    LOG_DEBUG("  salt size: ", response.salt.size(), " bytes");
    LOG_DEBUG("  Security flags: ", (int)response.securityFlags);
    if (response.securityFlags & 0x01) {
        LOG_DEBUG("  PIN grid seed: ", response.pinGridSeed);
    }

    return true;
}

network::Packet LogonProofPacket::build(const std::vector<uint8_t>& A,
                                         const std::vector<uint8_t>& M1) {
    return build(A, M1, 0, nullptr, nullptr, nullptr);
}

network::Packet LogonProofPacket::buildLegacy(const std::vector<uint8_t>& A,
                                              const std::vector<uint8_t>& M1) {
    return buildLegacy(A, M1, nullptr);
}

network::Packet LogonProofPacket::buildLegacy(const std::vector<uint8_t>& A,
                                              const std::vector<uint8_t>& M1,
                                              const std::array<uint8_t, 20>* crcHash) {
    if (A.size() != 32) {
        LOG_ERROR("Invalid A size: ", A.size(), " (expected 32)");
    }
    if (M1.size() != 20) {
        LOG_ERROR("Invalid M1 size: ", M1.size(), " (expected 20)");
    }

    network::Packet packet(static_cast<uint16_t>(AuthOpcode::LOGON_PROOF));
    packet.writeBytes(A.data(), A.size());
    packet.writeBytes(M1.data(), M1.size());
    if (crcHash) {
        packet.writeBytes(crcHash->data(), crcHash->size());
    } else {
        for (int i = 0; i < 20; ++i) packet.writeUInt8(0); // CRC hash
    }
    packet.writeUInt8(0); // number of keys
    packet.writeUInt8(0); // security flags
    return packet;
}

network::Packet LogonProofPacket::build(const std::vector<uint8_t>& A,
                                         const std::vector<uint8_t>& M1,
                                         uint8_t securityFlags,
                                         const std::array<uint8_t, 20>* crcHash,
                                         const std::array<uint8_t, 16>* pinClientSalt,
                                         const std::array<uint8_t, 20>* pinHash) {
    if (A.size() != 32) {
        LOG_ERROR("Invalid A size: ", A.size(), " (expected 32)");
    }
    if (M1.size() != 20) {
        LOG_ERROR("Invalid M1 size: ", M1.size(), " (expected 20)");
    }

    network::Packet packet(static_cast<uint16_t>(AuthOpcode::LOGON_PROOF));

    // A (client public ephemeral) - 32 bytes
    packet.writeBytes(A.data(), A.size());

    // M1 (client proof) - 20 bytes
    packet.writeBytes(M1.data(), M1.size());

    // CRC hash / integrity hash - 20 bytes
    if (crcHash) {
        packet.writeBytes(crcHash->data(), crcHash->size());
    } else {
        for (int i = 0; i < 20; ++i) packet.writeUInt8(0);
    }

    // Number of keys
    packet.writeUInt8(0);

    // Security flags
    packet.writeUInt8(securityFlags);

    if (securityFlags & 0x01) {
        if (!pinClientSalt || !pinHash) {
            LOG_ERROR("LOGON_PROOF: PIN flag set but PIN data missing");
        } else {
            // PIN: u8[16] client_salt + u8[20] pin_hash
            packet.writeBytes(pinClientSalt->data(), pinClientSalt->size());
            packet.writeBytes(pinHash->data(), pinHash->size());
        }
    }

    LOG_DEBUG("Built LOGON_PROOF packet:");
    LOG_DEBUG("  A size: ", A.size(), " bytes");
    LOG_DEBUG("  M1 size: ", M1.size(), " bytes");
    LOG_DEBUG("  Total size: ", packet.getSize(), " bytes");

    return packet;
}

network::Packet AuthenticatorTokenPacket::build(const std::string& token) {
    network::Packet packet(static_cast<uint16_t>(AuthOpcode::AUTHENTICATOR));
    // TrinityCore expects: u8 len + ascii token bytes (not null-terminated)
    uint8_t len = static_cast<uint8_t>(std::min<size_t>(255, token.size()));
    packet.writeUInt8(len);
    if (len > 0) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(token.data()), len);
    }
    return packet;
}

bool LogonProofResponseParser::parse(network::Packet& packet, LogonProofResponse& response) {
    // Note: opcode byte already consumed by handlePacket()

    // Status
    response.status = packet.readUInt8();

    LOG_INFO("LOGON_PROOF response status: ", (int)response.status);

    if (response.status != 0) {
        LOG_ERROR("LOGON_PROOF failed with status: ", (int)response.status);
        return true;  // Valid packet, but proof failed
    }

    // M2 (server proof) - 20 bytes
    response.M2.resize(20);
    for (int i = 0; i < 20; ++i) {
        response.M2[i] = packet.readUInt8();
    }

    LOG_DEBUG("Parsed LOGON_PROOF response:");
    LOG_DEBUG("  M2 size: ", response.M2.size(), " bytes");

    return true;
}

network::Packet RealmListPacket::build() {
    network::Packet packet(static_cast<uint16_t>(AuthOpcode::REALM_LIST));

    // Unknown uint32 (per WoWDev documentation)
    packet.writeUInt32(0x00);

    LOG_DEBUG("Built REALM_LIST request packet");
    LOG_DEBUG("  Total size: ", packet.getSize(), " bytes");

    return packet;
}

bool RealmListResponseParser::parse(network::Packet& packet, RealmListResponse& response, uint8_t protocolVersion) {
    // Note: opcode byte already consumed by handlePacket()
    const bool isVanilla = (protocolVersion < 8);

    // Packet size (2 bytes) - we already know the size, skip it
    uint16_t packetSize = packet.readUInt16();
    LOG_DEBUG("REALM_LIST response packet size: ", packetSize, " bytes");

    // Unknown uint32
    packet.readUInt32();

    // Realm count: uint8 for vanilla/classic, uint16 for WotLK+
    uint16_t realmCount;
    if (isVanilla) {
        realmCount = packet.readUInt8();
    } else {
        realmCount = packet.readUInt16();
    }
    LOG_INFO("REALM_LIST response: ", realmCount, " realms");

    response.realms.clear();
    response.realms.reserve(realmCount);

    for (uint16_t i = 0; i < realmCount; ++i) {
        Realm realm;

        // Icon/type: uint32 for vanilla, uint8 for WotLK
        if (isVanilla) {
            realm.icon = static_cast<uint8_t>(packet.readUInt32());
        } else {
            realm.icon = packet.readUInt8();
        }

        // Lock: not present in vanilla (added in TBC)
        if (!isVanilla) {
            realm.lock = packet.readUInt8();
        } else {
            realm.lock = 0;
        }

        // Flags
        realm.flags = packet.readUInt8();

        // Name (C-string)
        realm.name = packet.readString();

        // Address (C-string)
        realm.address = packet.readString();

        // Population (float)
        // Read 4 bytes as little-endian float
        uint32_t populationBits = packet.readUInt32();
        std::memcpy(&realm.population, &populationBits, sizeof(float));

        // Characters
        realm.characters = packet.readUInt8();

        // Timezone
        realm.timezone = packet.readUInt8();

        // ID
        realm.id = packet.readUInt8();

        // Version info (conditional - only if flags & 0x04)
        if (realm.hasVersionInfo()) {
            realm.majorVersion = packet.readUInt8();
            realm.minorVersion = packet.readUInt8();
            realm.patchVersion = packet.readUInt8();
            realm.build = packet.readUInt16();

            LOG_DEBUG("  Realm ", (int)i, " (", realm.name, ") version: ",
                      (int)realm.majorVersion, ".", (int)realm.minorVersion, ".",
                      (int)realm.patchVersion, " (", realm.build, ")");
        } else {
            LOG_DEBUG("  Realm ", (int)i, " (", realm.name, ") - no version info");
        }

        LOG_DEBUG("  Realm ", (int)i, " details:");
        LOG_DEBUG("    Name: ", realm.name);
        LOG_DEBUG("    Address: ", realm.address);
        LOG_DEBUG("    ID: ", (int)realm.id);
        LOG_DEBUG("    Icon: ", (int)realm.icon);
        LOG_DEBUG("    Lock: ", (int)realm.lock);
        LOG_DEBUG("    Flags: ", (int)realm.flags);
        LOG_DEBUG("    Population: ", realm.population);
        LOG_DEBUG("    Characters: ", (int)realm.characters);
        LOG_DEBUG("    Timezone: ", (int)realm.timezone);

        response.realms.push_back(realm);
    }

    LOG_INFO("Parsed ", response.realms.size(), " realms successfully");

    return true;
}

} // namespace auth
} // namespace wowee
