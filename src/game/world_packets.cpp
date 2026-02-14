#include "game/world_packets.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>

namespace wowee {
namespace game {

network::Packet AuthSessionPacket::build(uint32_t build,
                                          const std::string& accountName,
                                          uint32_t clientSeed,
                                          const std::vector<uint8_t>& sessionKey,
                                          uint32_t serverSeed,
                                          uint32_t realmId) {
    if (sessionKey.size() != 40) {
        LOG_ERROR("Invalid session key size: ", sessionKey.size(), " (expected 40)");
    }

    // Convert account name to uppercase
    std::string upperAccount = accountName;
    std::transform(upperAccount.begin(), upperAccount.end(),
                   upperAccount.begin(), ::toupper);

    LOG_INFO("Building CMSG_AUTH_SESSION for account: ", upperAccount);

    // Compute authentication hash
    auto authHash = computeAuthHash(upperAccount, clientSeed, serverSeed, sessionKey);

    LOG_DEBUG("  Build: ", build);
    LOG_DEBUG("  Client seed: 0x", std::hex, clientSeed, std::dec);
    LOG_DEBUG("  Server seed: 0x", std::hex, serverSeed, std::dec);
    LOG_DEBUG("  Auth hash: ", authHash.size(), " bytes");

    // Create packet (opcode will be added by WorldSocket)
    network::Packet packet(wireOpcode(Opcode::CMSG_AUTH_SESSION));

    bool isTbc = (build <= 8606);  // TBC 2.4.3 = 8606, WotLK starts at 11159+

    if (isTbc) {
        // TBC 2.4.3 format (6 fields):
        // Build, ServerID, Account, ClientSeed, Digest, AddonInfo
        packet.writeUInt32(build);
        packet.writeUInt32(realmId);           // server_id
        packet.writeString(upperAccount);
        packet.writeUInt32(clientSeed);
    } else {
        // WotLK 3.3.5a format (11 fields):
        // Build, LoginServerID, Account, LoginServerType, LocalChallenge,
        // RegionID, BattlegroupID, RealmID, DosResponse, Digest, AddonInfo
        packet.writeUInt32(build);
        packet.writeUInt32(0);                 // LoginServerID
        packet.writeString(upperAccount);
        packet.writeUInt32(0);                 // LoginServerType
        packet.writeUInt32(clientSeed);
        // AzerothCore ignores these fields; other cores may validate them.
        // Use 0 for maximum compatibility.
        packet.writeUInt32(0);                 // RegionID
        packet.writeUInt32(0);                 // BattlegroupID
        packet.writeUInt32(realmId);           // RealmID
        LOG_DEBUG("  Realm ID: ", realmId);
        packet.writeUInt32(0);                 // DOS response (uint64)
        packet.writeUInt32(0);
    }

    // Authentication hash/digest (20 bytes)
    packet.writeBytes(authHash.data(), authHash.size());

    // Addon info - compressed block with 0 addons
    // AzerothCore format: uint32 decompressedSize + zlib compressed data
    // Decompressed format: uint32 addonCount + [addons...] + uint32 clientTime
    uint8_t addonData[8] = {
        0, 0, 0, 0,  // addon count = 0
        0, 0, 0, 0   // client time = 0
    };
    uint32_t decompressedSize = 8;

    // Compress with zlib
    uLongf compressedSize = compressBound(decompressedSize);
    std::vector<uint8_t> compressed(compressedSize);
    int ret = compress(compressed.data(), &compressedSize, addonData, decompressedSize);
    if (ret == Z_OK) {
        compressed.resize(compressedSize);
        // Write decompressedSize, then compressed bytes
        packet.writeUInt32(decompressedSize);
        packet.writeBytes(compressed.data(), compressed.size());
        LOG_DEBUG("Addon info: decompressedSize=", decompressedSize,
                  " compressedSize=", compressedSize);
    } else {
        LOG_ERROR("zlib compress failed with code: ", ret);
        packet.writeUInt32(0);
    }

    LOG_INFO("CMSG_AUTH_SESSION packet built: ", packet.getSize(), " bytes");

    // Dump full packet for protocol debugging
    const auto& data = packet.getData();
    std::string hexDump;
    for (size_t i = 0; i < data.size(); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02x ", data[i]);
        hexDump += buf;
        if ((i + 1) % 16 == 0) hexDump += "\n";
    }
    LOG_DEBUG("CMSG_AUTH_SESSION full dump:\n", hexDump);

    return packet;
}

std::vector<uint8_t> AuthSessionPacket::computeAuthHash(
    const std::string& accountName,
    uint32_t clientSeed,
    uint32_t serverSeed,
    const std::vector<uint8_t>& sessionKey) {

    // Build hash input:
    // account_name + [0,0,0,0] + client_seed + server_seed + session_key

    std::vector<uint8_t> hashInput;
    hashInput.reserve(accountName.size() + 4 + 4 + 4 + sessionKey.size());

    // Account name (as bytes)
    hashInput.insert(hashInput.end(), accountName.begin(), accountName.end());

    // 4 null bytes
    for (int i = 0; i < 4; ++i) {
        hashInput.push_back(0);
    }

    // Client seed (little-endian)
    hashInput.push_back(clientSeed & 0xFF);
    hashInput.push_back((clientSeed >> 8) & 0xFF);
    hashInput.push_back((clientSeed >> 16) & 0xFF);
    hashInput.push_back((clientSeed >> 24) & 0xFF);

    // Server seed (little-endian)
    hashInput.push_back(serverSeed & 0xFF);
    hashInput.push_back((serverSeed >> 8) & 0xFF);
    hashInput.push_back((serverSeed >> 16) & 0xFF);
    hashInput.push_back((serverSeed >> 24) & 0xFF);

    // Session key (40 bytes)
    hashInput.insert(hashInput.end(), sessionKey.begin(), sessionKey.end());

    // Diagnostic: dump auth hash inputs for debugging AUTH_REJECT
    {
        auto toHex = [](const uint8_t* data, size_t len) {
            std::string s;
            for (size_t i = 0; i < len; ++i) {
                char buf[4]; snprintf(buf, sizeof(buf), "%02x", data[i]); s += buf;
            }
            return s;
        };
        LOG_INFO("AUTH HASH: account='", accountName, "' clientSeed=0x", std::hex, clientSeed,
                 " serverSeed=0x", serverSeed, std::dec);
        LOG_INFO("AUTH HASH: sessionKey=", toHex(sessionKey.data(), sessionKey.size()));
        LOG_INFO("AUTH HASH: input(", hashInput.size(), ")=", toHex(hashInput.data(), hashInput.size()));
    }

    // Compute SHA1 hash
    auto result = auth::Crypto::sha1(hashInput);

    {
        auto toHex = [](const uint8_t* data, size_t len) {
            std::string s;
            for (size_t i = 0; i < len; ++i) {
                char buf[4]; snprintf(buf, sizeof(buf), "%02x", data[i]); s += buf;
            }
            return s;
        };
        LOG_INFO("AUTH HASH: digest=", toHex(result.data(), result.size()));
    }

    return result;
}

bool AuthChallengeParser::parse(network::Packet& packet, AuthChallengeData& data) {
    // SMSG_AUTH_CHALLENGE format varies by expansion:
    //   TBC 2.4.3:    uint32 serverSeed                      (4 bytes)
    //   WotLK 3.3.5a: uint32 one + uint32 serverSeed + seeds (40 bytes)

    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_AUTH_CHALLENGE packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    if (packet.getSize() < 8) {
        // TBC format: just the server seed (4 bytes)
        data.unknown1 = 0;
        data.serverSeed = packet.readUInt32();
        LOG_INFO("Parsed SMSG_AUTH_CHALLENGE (TBC format):");
    } else {
        // WotLK format: unknown1 + serverSeed + encryption seeds
        data.unknown1 = packet.readUInt32();
        data.serverSeed = packet.readUInt32();
        LOG_INFO("Parsed SMSG_AUTH_CHALLENGE (WotLK format):");
        LOG_INFO("  Unknown1: 0x", std::hex, data.unknown1, std::dec);
    }

    LOG_INFO("  Server seed: 0x", std::hex, data.serverSeed, std::dec);

    return true;
}

bool AuthResponseParser::parse(network::Packet& packet, AuthResponseData& response) {
    // SMSG_AUTH_RESPONSE format:
    // uint8 result

    if (packet.getSize() < 1) {
        LOG_ERROR("SMSG_AUTH_RESPONSE packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    uint8_t resultCode = packet.readUInt8();
    response.result = static_cast<AuthResult>(resultCode);

    LOG_INFO("Parsed SMSG_AUTH_RESPONSE: ", getAuthResultString(response.result));

    return true;
}

const char* getAuthResultString(AuthResult result) {
    switch (result) {
        case AuthResult::OK:
            return "OK - Authentication successful";
        case AuthResult::FAILED:
            return "FAILED - Authentication failed";
        case AuthResult::REJECT:
            return "REJECT - Connection rejected";
        case AuthResult::BAD_SERVER_PROOF:
            return "BAD_SERVER_PROOF - Invalid server proof";
        case AuthResult::UNAVAILABLE:
            return "UNAVAILABLE - Server unavailable";
        case AuthResult::SYSTEM_ERROR:
            return "SYSTEM_ERROR - System error occurred";
        case AuthResult::BILLING_ERROR:
            return "BILLING_ERROR - Billing error";
        case AuthResult::BILLING_EXPIRED:
            return "BILLING_EXPIRED - Subscription expired";
        case AuthResult::VERSION_MISMATCH:
            return "VERSION_MISMATCH - Client version mismatch";
        case AuthResult::UNKNOWN_ACCOUNT:
            return "UNKNOWN_ACCOUNT - Account not found";
        case AuthResult::INCORRECT_PASSWORD:
            return "INCORRECT_PASSWORD - Wrong password";
        case AuthResult::SESSION_EXPIRED:
            return "SESSION_EXPIRED - Session has expired";
        case AuthResult::SERVER_SHUTTING_DOWN:
            return "SERVER_SHUTTING_DOWN - Server is shutting down";
        case AuthResult::ALREADY_LOGGING_IN:
            return "ALREADY_LOGGING_IN - Already logging in";
        case AuthResult::LOGIN_SERVER_NOT_FOUND:
            return "LOGIN_SERVER_NOT_FOUND - Can't contact login server";
        case AuthResult::WAIT_QUEUE:
            return "WAIT_QUEUE - Waiting in queue";
        case AuthResult::BANNED:
            return "BANNED - Account is banned";
        case AuthResult::ALREADY_ONLINE:
            return "ALREADY_ONLINE - Character already logged in";
        case AuthResult::NO_TIME:
            return "NO_TIME - No game time remaining";
        case AuthResult::DB_BUSY:
            return "DB_BUSY - Database is busy";
        case AuthResult::SUSPENDED:
            return "SUSPENDED - Account is suspended";
        case AuthResult::PARENTAL_CONTROL:
            return "PARENTAL_CONTROL - Parental controls active";
        case AuthResult::LOCKED_ENFORCED:
            return "LOCKED_ENFORCED - Account is locked";
        default:
            return "UNKNOWN - Unknown result code";
    }
}

// ============================================================
// Character Creation
// ============================================================

network::Packet CharCreatePacket::build(const CharCreateData& data) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CHAR_CREATE));

    // Convert nonbinary gender to server-compatible value (servers only support male/female)
    Gender serverGender = toServerGender(data.gender);

    packet.writeString(data.name);  // null-terminated name
    packet.writeUInt8(static_cast<uint8_t>(data.race));
    packet.writeUInt8(static_cast<uint8_t>(data.characterClass));
    packet.writeUInt8(static_cast<uint8_t>(serverGender));
    packet.writeUInt8(data.skin);
    packet.writeUInt8(data.face);
    packet.writeUInt8(data.hairStyle);
    packet.writeUInt8(data.hairColor);
    packet.writeUInt8(data.facialHair);
    packet.writeUInt8(0);  // outfitId, always 0

    LOG_DEBUG("Built CMSG_CHAR_CREATE: name=", data.name,
              " race=", static_cast<int>(data.race),
              " class=", static_cast<int>(data.characterClass),
              " gender=", static_cast<int>(data.gender),
              " (server gender=", static_cast<int>(serverGender), ")",
              " skin=", static_cast<int>(data.skin),
              " face=", static_cast<int>(data.face),
              " hair=", static_cast<int>(data.hairStyle),
              " hairColor=", static_cast<int>(data.hairColor),
              " facial=", static_cast<int>(data.facialHair));

    // Dump full packet for protocol debugging
    const auto& pktData = packet.getData();
    std::string hexDump;
    for (size_t i = 0; i < pktData.size(); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02x ", pktData[i]);
        hexDump += buf;
    }
    LOG_DEBUG("CMSG_CHAR_CREATE full dump: ", hexDump);

    return packet;
}

bool CharCreateResponseParser::parse(network::Packet& packet, CharCreateResponseData& data) {
    data.result = static_cast<CharCreateResult>(packet.readUInt8());
    LOG_INFO("SMSG_CHAR_CREATE result: ", static_cast<int>(data.result));
    return true;
}

network::Packet CharEnumPacket::build() {
    // CMSG_CHAR_ENUM has no body - just the opcode
    network::Packet packet(wireOpcode(Opcode::CMSG_CHAR_ENUM));

    LOG_DEBUG("Built CMSG_CHAR_ENUM packet (no body)");

    return packet;
}

bool CharEnumParser::parse(network::Packet& packet, CharEnumResponse& response) {
    // Read character count
    uint8_t count = packet.readUInt8();

    LOG_INFO("Parsing SMSG_CHAR_ENUM: ", (int)count, " characters");

    response.characters.clear();
    response.characters.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        Character character;

        // Read GUID (8 bytes, little-endian)
        character.guid = packet.readUInt64();

        // Read name (null-terminated string)
        character.name = packet.readString();

        // Read race, class, gender
        character.race = static_cast<Race>(packet.readUInt8());
        character.characterClass = static_cast<Class>(packet.readUInt8());
        character.gender = static_cast<Gender>(packet.readUInt8());

        // Read appearance data
        character.appearanceBytes = packet.readUInt32();
        character.facialFeatures = packet.readUInt8();

        // Read level
        character.level = packet.readUInt8();

        // Read location
        character.zoneId = packet.readUInt32();
        character.mapId = packet.readUInt32();
        character.x = packet.readFloat();
        character.y = packet.readFloat();
        character.z = packet.readFloat();

        // Read affiliations
        character.guildId = packet.readUInt32();

        // Read flags
        character.flags = packet.readUInt32();

        // Skip customization flag (uint32) and unknown byte
        packet.readUInt32();  // Customization
        packet.readUInt8();   // Unknown

        // Read pet data (always present, even if no pet)
        character.pet.displayModel = packet.readUInt32();
        character.pet.level = packet.readUInt32();
        character.pet.family = packet.readUInt32();

        // Read equipment (23 items)
        character.equipment.reserve(23);
        for (int j = 0; j < 23; ++j) {
            EquipmentItem item;
            item.displayModel = packet.readUInt32();
            item.inventoryType = packet.readUInt8();
            item.enchantment = packet.readUInt32();
            character.equipment.push_back(item);
        }

        LOG_INFO("  Character ", (int)(i + 1), ": ", character.name);
        LOG_INFO("    GUID: 0x", std::hex, character.guid, std::dec);
        LOG_INFO("    ", getRaceName(character.race), " ",
                 getClassName(character.characterClass), " (",
                 getGenderName(character.gender), ")");
        LOG_INFO("    Level: ", (int)character.level);
        LOG_INFO("    Location: Zone ", character.zoneId, ", Map ", character.mapId);
        LOG_INFO("    Position: (", character.x, ", ", character.y, ", ", character.z, ")");
        if (character.hasGuild()) {
            LOG_INFO("    Guild ID: ", character.guildId);
        }
        if (character.hasPet()) {
            LOG_INFO("    Pet: Model ", character.pet.displayModel,
                     ", Level ", character.pet.level);
        }

        response.characters.push_back(character);
    }

    LOG_INFO("Successfully parsed ", response.characters.size(), " characters");

    return true;
}

network::Packet PlayerLoginPacket::build(uint64_t characterGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PLAYER_LOGIN));

    // Write character GUID (8 bytes, little-endian)
    packet.writeUInt64(characterGuid);

    LOG_INFO("Built CMSG_PLAYER_LOGIN packet");
    LOG_INFO("  Character GUID: 0x", std::hex, characterGuid, std::dec);

    return packet;
}

bool LoginVerifyWorldParser::parse(network::Packet& packet, LoginVerifyWorldData& data) {
    // SMSG_LOGIN_VERIFY_WORLD format (WoW 3.3.5a):
    // uint32 mapId
    // float x, y, z (position)
    // float orientation

    if (packet.getSize() < 20) {
        LOG_ERROR("SMSG_LOGIN_VERIFY_WORLD packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.mapId = packet.readUInt32();
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();
    data.orientation = packet.readFloat();

    LOG_INFO("Parsed SMSG_LOGIN_VERIFY_WORLD:");
    LOG_INFO("  Map ID: ", data.mapId);
    LOG_INFO("  Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("  Orientation: ", data.orientation, " radians");

    return true;
}

bool AccountDataTimesParser::parse(network::Packet& packet, AccountDataTimesData& data) {
    // SMSG_ACCOUNT_DATA_TIMES format (WoW 3.3.5a):
    // uint32 serverTime (Unix timestamp)
    // uint8 unknown (always 1?)
    // uint32[8] accountDataTimes (timestamps for each data slot)

    if (packet.getSize() < 37) {
        LOG_ERROR("SMSG_ACCOUNT_DATA_TIMES packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.serverTime = packet.readUInt32();
    data.unknown = packet.readUInt8();

    LOG_DEBUG("Parsed SMSG_ACCOUNT_DATA_TIMES:");
    LOG_DEBUG("  Server time: ", data.serverTime);
    LOG_DEBUG("  Unknown: ", (int)data.unknown);

    for (int i = 0; i < 8; ++i) {
        data.accountDataTimes[i] = packet.readUInt32();
        if (data.accountDataTimes[i] != 0) {
            LOG_DEBUG("  Data slot ", i, ": ", data.accountDataTimes[i]);
        }
    }

    return true;
}

bool MotdParser::parse(network::Packet& packet, MotdData& data) {
    // SMSG_MOTD format (WoW 3.3.5a):
    // uint32 lineCount
    // string[lineCount] lines (null-terminated strings)

    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_MOTD packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    uint32_t lineCount = packet.readUInt32();

    LOG_INFO("Parsed SMSG_MOTD:");
    LOG_INFO("  Line count: ", lineCount);

    data.lines.clear();
    data.lines.reserve(lineCount);

    for (uint32_t i = 0; i < lineCount; ++i) {
        std::string line = packet.readString();
        data.lines.push_back(line);
        LOG_INFO("  [", i + 1, "] ", line);
    }

    return true;
}

network::Packet PingPacket::build(uint32_t sequence, uint32_t latency) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PING));

    // Write sequence number (uint32, little-endian)
    packet.writeUInt32(sequence);

    // Write latency (uint32, little-endian, in milliseconds)
    packet.writeUInt32(latency);

    LOG_DEBUG("Built CMSG_PING packet");
    LOG_DEBUG("  Sequence: ", sequence);
    LOG_DEBUG("  Latency: ", latency, " ms");

    return packet;
}

bool PongParser::parse(network::Packet& packet, PongData& data) {
    // SMSG_PONG format (WoW 3.3.5a):
    // uint32 sequence (echoed from CMSG_PING)

    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_PONG packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.sequence = packet.readUInt32();

    LOG_DEBUG("Parsed SMSG_PONG:");
    LOG_DEBUG("  Sequence: ", data.sequence);

    return true;
}

void MovementPacket::writePackedGuid(network::Packet& packet, uint64_t guid) {
    uint8_t mask = 0;
    uint8_t guidBytes[8];
    int guidByteCount = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t byte = static_cast<uint8_t>((guid >> (i * 8)) & 0xFF);
        if (byte != 0) {
            mask |= (1 << i);
            guidBytes[guidByteCount++] = byte;
        }
    }
    packet.writeUInt8(mask);
    for (int i = 0; i < guidByteCount; i++) {
        packet.writeUInt8(guidBytes[i]);
    }
}

void MovementPacket::writeMovementPayload(network::Packet& packet, const MovementInfo& info) {
    // Movement packet format (WoW 3.3.5a) payload:
    // uint32 flags
    // uint16 flags2
    // uint32 time
    // float x, y, z
    // float orientation

    // Write movement flags
    packet.writeUInt32(info.flags);
    packet.writeUInt16(info.flags2);

    // Write timestamp
    packet.writeUInt32(info.time);

    // Write position
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.x), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.y), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.z), sizeof(float));

    // Write orientation
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.orientation), sizeof(float));

    // Write transport data if on transport.
    // 3.3.5a ordering: transport block appears before pitch/fall/jump.
    if (info.hasFlag(MovementFlags::ONTRANSPORT)) {
        // Write packed transport GUID
        uint8_t transMask = 0;
        uint8_t transGuidBytes[8];
        int transGuidByteCount = 0;
        for (int i = 0; i < 8; i++) {
            uint8_t byte = static_cast<uint8_t>((info.transportGuid >> (i * 8)) & 0xFF);
            if (byte != 0) {
                transMask |= (1 << i);
                transGuidBytes[transGuidByteCount++] = byte;
            }
        }
        packet.writeUInt8(transMask);
        for (int i = 0; i < transGuidByteCount; i++) {
            packet.writeUInt8(transGuidBytes[i]);
        }

        // Write transport local position
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportX), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportY), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportZ), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportO), sizeof(float));

        // Write transport time
        packet.writeUInt32(info.transportTime);

        // Transport seat is always present in ONTRANSPORT movement info.
        packet.writeUInt8(static_cast<uint8_t>(info.transportSeat));

        // Optional second transport time for interpolated movement.
        if (info.flags2 & 0x0200) {
            packet.writeUInt32(info.transportTime2);
        }
    }

    // Write pitch if swimming/flying
    if (info.hasFlag(MovementFlags::SWIMMING) || info.hasFlag(MovementFlags::FLYING)) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.pitch), sizeof(float));
    }

    // Fall time is ALWAYS present in the packet (server reads it unconditionally).
    // Jump velocity/angle data is only present when FALLING flag is set.
    packet.writeUInt32(info.fallTime);

    if (info.hasFlag(MovementFlags::FALLING)) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpVelocity), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpSinAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpCosAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpXYSpeed), sizeof(float));
    }
}

network::Packet MovementPacket::build(Opcode opcode, const MovementInfo& info, uint64_t playerGuid) {
    network::Packet packet(wireOpcode(opcode));

    // Movement packet format (WoW 3.3.5a):
    // packed GUID + movement payload
    writePackedGuid(packet, playerGuid);
    writeMovementPayload(packet, info);

    // Detailed hex dump for debugging
    static int mvLog = 5;
    if (mvLog-- > 0) {
        const auto& raw = packet.getData();
        std::string hex;
        for (size_t i = 0; i < raw.size(); i++) {
            char b[4]; snprintf(b, sizeof(b), "%02x ", raw[i]);
            hex += b;
        }
        LOG_INFO("MOVEPKT opcode=0x", std::hex, wireOpcode(opcode), std::dec,
                 " guid=0x", std::hex, playerGuid, std::dec,
                 " payload=", raw.size(), " bytes",
                 " flags=0x", std::hex, info.flags, std::dec,
                 " flags2=0x", std::hex, info.flags2, std::dec,
                 " pos=(", info.x, ",", info.y, ",", info.z, ",", info.orientation, ")",
                 " fallTime=", info.fallTime,
                 (info.hasFlag(MovementFlags::ONTRANSPORT) ?
                  " ONTRANSPORT guid=0x" + std::to_string(info.transportGuid) +
                  " localPos=(" + std::to_string(info.transportX) + "," +
                  std::to_string(info.transportY) + "," + std::to_string(info.transportZ) + ")" : ""));
        LOG_INFO("MOVEPKT hex: ", hex);
    }

    return packet;
}

uint64_t UpdateObjectParser::readPackedGuid(network::Packet& packet) {
    // Read packed GUID format:
    // First byte is a mask indicating which bytes are present
    uint8_t mask = packet.readUInt8();

    if (mask == 0) {
        return 0;
    }

    uint64_t guid = 0;
    for (int i = 0; i < 8; ++i) {
        if (mask & (1 << i)) {
            uint8_t byte = packet.readUInt8();
            guid |= (static_cast<uint64_t>(byte) << (i * 8));
        }
    }

    return guid;
}

bool UpdateObjectParser::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    // WoW 3.3.5a UPDATE_OBJECT movement block structure:
    // 1. UpdateFlags (1 byte, sometimes 2)
    // 2. Movement data depends on update flags

    // Update flags (3.3.5a uses 2 bytes for flags)
    uint16_t updateFlags = packet.readUInt16();
    block.updateFlags = updateFlags;

    LOG_DEBUG("  UpdateFlags: 0x", std::hex, updateFlags, std::dec);

    // Log transport-related flag combinations
    if (updateFlags & 0x0002) { // UPDATEFLAG_TRANSPORT
        LOG_INFO("  Transport flags detected: 0x", std::hex, updateFlags, std::dec,
                 " (TRANSPORT=", !!(updateFlags & 0x0002),
                 ", POSITION=", !!(updateFlags & 0x0100),
                 ", ROTATION=", !!(updateFlags & 0x0200),
                 ", STATIONARY=", !!(updateFlags & 0x0040), ")");
    }

    // UpdateFlags bit meanings:
    // 0x0001 = UPDATEFLAG_SELF
    // 0x0002 = UPDATEFLAG_TRANSPORT
    // 0x0004 = UPDATEFLAG_HAS_TARGET
    // 0x0008 = UPDATEFLAG_LOWGUID
    // 0x0010 = UPDATEFLAG_HIGHGUID
    // 0x0020 = UPDATEFLAG_LIVING
    // 0x0040 = UPDATEFLAG_STATIONARY_POSITION
    // 0x0080 = UPDATEFLAG_VEHICLE
    // 0x0100 = UPDATEFLAG_POSITION (transport)
    // 0x0200 = UPDATEFLAG_ROTATION

    const uint16_t UPDATEFLAG_LIVING = 0x0020;
    const uint16_t UPDATEFLAG_STATIONARY_POSITION = 0x0040;
    const uint16_t UPDATEFLAG_HAS_TARGET = 0x0004;
    const uint16_t UPDATEFLAG_TRANSPORT = 0x0002;
    const uint16_t UPDATEFLAG_POSITION = 0x0100;
    const uint16_t UPDATEFLAG_VEHICLE = 0x0080;
    const uint16_t UPDATEFLAG_ROTATION = 0x0200;
    const uint16_t UPDATEFLAG_LOWGUID = 0x0008;
    const uint16_t UPDATEFLAG_HIGHGUID = 0x0010;

    if (updateFlags & UPDATEFLAG_LIVING) {
        // Full movement block for living units
        uint32_t moveFlags = packet.readUInt32();
        uint16_t moveFlags2 = packet.readUInt16();
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  LIVING movement: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport data (if on transport)
        if (moveFlags & 0x00000200) { // MOVEMENTFLAG_ONTRANSPORT
            block.onTransport = true;
            block.transportGuid = readPackedGuid(packet);
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t tTime =*/ packet.readUInt32();
            /*int8_t tSeat =*/ packet.readUInt8();

            LOG_DEBUG("  OnTransport: guid=0x", std::hex, block.transportGuid, std::dec,
                      " offset=(", block.transportX, ", ", block.transportY, ", ", block.transportZ, ")");

            if (moveFlags2 & 0x0200) { // MOVEMENTFLAG2_INTERPOLATED_MOVEMENT
                /*uint32_t tTime2 =*/ packet.readUInt32();
            }
        }

        // Swimming/flying pitch
        if ((moveFlags & 0x02000000) || (moveFlags2 & 0x0010)) { // MOVEMENTFLAG_SWIMMING or MOVEMENTFLAG2_ALWAYS_ALLOW_PITCHING
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping
        if (moveFlags & 0x00001000) { // MOVEMENTFLAG_FALLING
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & 0x04000000) { // MOVEMENTFLAG_SPLINE_ELEVATION
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (7 speed values)
        /*float walkSpeed =*/ packet.readFloat();
        float runSpeed = packet.readFloat();
        /*float runBackSpeed =*/ packet.readFloat();
        /*float swimSpeed =*/ packet.readFloat();
        /*float swimBackSpeed =*/ packet.readFloat();
        /*float flightSpeed =*/ packet.readFloat();
        /*float flightBackSpeed =*/ packet.readFloat();
        /*float turnRate =*/ packet.readFloat();
        /*float pitchRate =*/ packet.readFloat();

        block.runSpeed = runSpeed;

        // Spline data
        if (moveFlags & 0x08000000) { // MOVEMENTFLAG_SPLINE_ENABLED
            uint32_t splineFlags = packet.readUInt32();
            LOG_DEBUG("  Spline: flags=0x", std::hex, splineFlags, std::dec);

            if (splineFlags & 0x00010000) { // SPLINEFLAG_FINAL_POINT
                /*float finalX =*/ packet.readFloat();
                /*float finalY =*/ packet.readFloat();
                /*float finalZ =*/ packet.readFloat();
            } else if (splineFlags & 0x00020000) { // SPLINEFLAG_FINAL_TARGET
                /*uint64_t finalTarget =*/ packet.readUInt64();
            } else if (splineFlags & 0x00040000) { // SPLINEFLAG_FINAL_ANGLE
                /*float finalAngle =*/ packet.readFloat();
            }

            /*uint32_t timePassed =*/ packet.readUInt32();
            /*uint32_t duration =*/ packet.readUInt32();
            /*uint32_t splineId =*/ packet.readUInt32();

            /*float durationMod =*/ packet.readFloat();
            /*float durationModNext =*/ packet.readFloat();

            /*float verticalAccel =*/ packet.readFloat();
            /*uint32_t effectStartTime =*/ packet.readUInt32();

            uint32_t pointCount = packet.readUInt32();
            if (pointCount > 256) {
                LOG_WARNING("  Spline pointCount=", pointCount, " exceeds maximum, capping at 0 (readPos=",
                            packet.getReadPos(), "/", packet.getSize(), ")");
                pointCount = 0;
            } else {
                LOG_DEBUG("  Spline pointCount=", pointCount);
            }
            for (uint32_t i = 0; i < pointCount; i++) {
                /*float px =*/ packet.readFloat();
                /*float py =*/ packet.readFloat();
                /*float pz =*/ packet.readFloat();
            }

            /*uint8_t splineMode =*/ packet.readUInt8();
            /*float endPointX =*/ packet.readFloat();
            /*float endPointY =*/ packet.readFloat();
            /*float endPointZ =*/ packet.readFloat();
        }
    }
    else if (updateFlags & UPDATEFLAG_POSITION) {
        // Transport position update (UPDATEFLAG_POSITION = 0x0100)
        uint64_t transportGuid = readPackedGuid(packet);
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.onTransport = (transportGuid != 0);
        block.transportGuid = transportGuid;
        float tx = packet.readFloat();
        float ty = packet.readFloat();
        float tz = packet.readFloat();
        if (block.onTransport) {
            block.transportX = tx;
            block.transportY = ty;
            block.transportZ = tz;
        } else {
            block.transportX = 0.0f;
            block.transportY = 0.0f;
            block.transportZ = 0.0f;
        }
        block.orientation = packet.readFloat();
        /*float corpseOrientation =*/ packet.readFloat();
        block.hasMovement = true;

        if (block.onTransport) {
            LOG_INFO("  TRANSPORT POSITION UPDATE: guid=0x", std::hex, transportGuid, std::dec,
                     " pos=(", block.x, ", ", block.y, ", ", block.z, "), o=", block.orientation,
                     " offset=(", block.transportX, ", ", block.transportY, ", ", block.transportZ, ")");
        }
    }
    else if (updateFlags & UPDATEFLAG_STATIONARY_POSITION) {
        // Simple stationary position (4 floats)
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  STATIONARY: (", block.x, ", ", block.y, ", ", block.z, "), o=", block.orientation);
    }

    // Target GUID (for units with target)
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        /*uint64_t targetGuid =*/ readPackedGuid(packet);
    }

    // Transport time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    // Vehicle
    if (updateFlags & UPDATEFLAG_VEHICLE) {
        /*uint32_t vehicleId =*/ packet.readUInt32();
        /*float vehicleOrientation =*/ packet.readFloat();
    }

    // Rotation (GameObjects)
    if (updateFlags & UPDATEFLAG_ROTATION) {
        /*int64_t rotation =*/ packet.readUInt64();
    }

    // Low GUID
    if (updateFlags & UPDATEFLAG_LOWGUID) {
        /*uint32_t lowGuid =*/ packet.readUInt32();
    }

    // High GUID
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        /*uint32_t highGuid =*/ packet.readUInt32();
    }

    return true;
}

bool UpdateObjectParser::parseUpdateFields(network::Packet& packet, UpdateBlock& block) {
    size_t startPos = packet.getReadPos();

    // Read number of blocks (each block is 32 fields = 32 bits)
    uint8_t blockCount = packet.readUInt8();

    if (blockCount == 0) {
        return true; // No fields to update
    }

    uint32_t fieldsCapacity = blockCount * 32;
    LOG_DEBUG("  UPDATE MASK PARSE:");
    LOG_DEBUG("    maskBlockCount = ", (int)blockCount);
    LOG_DEBUG("    fieldsCapacity (blocks * 32) = ", fieldsCapacity);

    // Read update mask
    std::vector<uint32_t> updateMask(blockCount);
    for (int i = 0; i < blockCount; ++i) {
        updateMask[i] = packet.readUInt32();
    }

    // Find highest set bit
    uint16_t highestSetBit = 0;
    uint32_t valuesReadCount = 0;

    // Read field values for each bit set in mask
    for (int blockIdx = 0; blockIdx < blockCount; ++blockIdx) {
        uint32_t mask = updateMask[blockIdx];

        for (int bit = 0; bit < 32; ++bit) {
            if (mask & (1 << bit)) {
                uint16_t fieldIndex = blockIdx * 32 + bit;
                if (fieldIndex > highestSetBit) {
                    highestSetBit = fieldIndex;
                }
                uint32_t value = packet.readUInt32();
                block.fields[fieldIndex] = value;
                valuesReadCount++;

                LOG_DEBUG("    Field[", fieldIndex, "] = 0x", std::hex, value, std::dec);
            }
        }
    }

    size_t endPos = packet.getReadPos();
    size_t bytesUsed = endPos - startPos;
    size_t bytesRemaining = packet.getSize() - endPos;

    LOG_DEBUG("    highestSetBitIndex = ", highestSetBit);
    LOG_DEBUG("    valuesReadCount = ", valuesReadCount);
    LOG_DEBUG("    bytesUsedForFields = ", bytesUsed);
    LOG_DEBUG("    bytesRemainingInPacket = ", bytesRemaining);
    LOG_DEBUG("  Parsed ", block.fields.size(), " fields");

    return true;
}

bool UpdateObjectParser::parseUpdateBlock(network::Packet& packet, UpdateBlock& block) {
    // Read update type
    uint8_t updateTypeVal = packet.readUInt8();
    block.updateType = static_cast<UpdateType>(updateTypeVal);

    LOG_DEBUG("Update block: type=", (int)updateTypeVal);

    switch (block.updateType) {
        case UpdateType::VALUES: {
            // Partial update - changed fields only
            block.guid = readPackedGuid(packet);
            LOG_DEBUG("  VALUES update for GUID: 0x", std::hex, block.guid, std::dec);

            return parseUpdateFields(packet, block);
        }

        case UpdateType::MOVEMENT: {
            // Movement update
            block.guid = readPackedGuid(packet);
            LOG_DEBUG("  MOVEMENT update for GUID: 0x", std::hex, block.guid, std::dec);

            return parseMovementBlock(packet, block);
        }

        case UpdateType::CREATE_OBJECT:
        case UpdateType::CREATE_OBJECT2: {
            // Create new object with full data
            block.guid = readPackedGuid(packet);
            LOG_DEBUG("  CREATE_OBJECT for GUID: 0x", std::hex, block.guid, std::dec);

            // Read object type
            uint8_t objectTypeVal = packet.readUInt8();
            block.objectType = static_cast<ObjectType>(objectTypeVal);
            LOG_DEBUG("  Object type: ", (int)objectTypeVal);

            // Parse movement if present
            bool hasMovement = parseMovementBlock(packet, block);
            if (!hasMovement) {
                return false;
            }

            // Parse update fields
            return parseUpdateFields(packet, block);
        }

        case UpdateType::OUT_OF_RANGE_OBJECTS: {
            // Objects leaving view range - handled differently
            LOG_DEBUG("  OUT_OF_RANGE_OBJECTS (skipping in block parser)");
            return true;
        }

        case UpdateType::NEAR_OBJECTS: {
            // Objects entering view range - handled differently
            LOG_DEBUG("  NEAR_OBJECTS (skipping in block parser)");
            return true;
        }

        default:
            LOG_WARNING("Unknown update type: ", (int)updateTypeVal);
            return false;
    }
}

bool UpdateObjectParser::parse(network::Packet& packet, UpdateObjectData& data) {

    // Read block count
    data.blockCount = packet.readUInt32();

    LOG_DEBUG("SMSG_UPDATE_OBJECT:");
    LOG_DEBUG("  objectCount = ", data.blockCount);
    LOG_DEBUG("  packetSize = ", packet.getSize());

    // Check for out-of-range objects first
    if (packet.getReadPos() + 1 <= packet.getSize()) {
        uint8_t firstByte = packet.readUInt8();

        if (firstByte == static_cast<uint8_t>(UpdateType::OUT_OF_RANGE_OBJECTS)) {
            // Read out-of-range GUID count
            uint32_t count = packet.readUInt32();

            for (uint32_t i = 0; i < count; ++i) {
                uint64_t guid = readPackedGuid(packet);
                data.outOfRangeGuids.push_back(guid);
                LOG_DEBUG("    Out of range: 0x", std::hex, guid, std::dec);
            }

            // Done - packet may have more blocks after this
            // Reset read position to after the first byte if needed
        } else {
            // Not out-of-range, rewind
            packet.setReadPos(packet.getReadPos() - 1);
        }
    }

    // Parse update blocks
    data.blocks.reserve(data.blockCount);

    for (uint32_t i = 0; i < data.blockCount; ++i) {
        LOG_DEBUG("Parsing block ", i + 1, " / ", data.blockCount);

        UpdateBlock block;
        if (!parseUpdateBlock(packet, block)) {
            LOG_ERROR("Failed to parse update block ", i + 1);
            return false;
        }

        data.blocks.push_back(block);
    }


    return true;
}

bool DestroyObjectParser::parse(network::Packet& packet, DestroyObjectData& data) {
    // SMSG_DESTROY_OBJECT format:
    // uint64 guid
    // uint8 isDeath (0 = despawn, 1 = death) — WotLK only; vanilla/TBC omit this

    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_DESTROY_OBJECT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.guid = packet.readUInt64();
    // WotLK adds isDeath byte; vanilla/TBC packets are exactly 8 bytes
    if (packet.getReadPos() < packet.getSize()) {
        data.isDeath = (packet.readUInt8() != 0);
    } else {
        data.isDeath = false;
    }

    LOG_INFO("Parsed SMSG_DESTROY_OBJECT:");
    LOG_INFO("  GUID: 0x", std::hex, data.guid, std::dec);
    LOG_INFO("  Is death: ", data.isDeath ? "yes" : "no");

    return true;
}

network::Packet MessageChatPacket::build(ChatType type,
                                          ChatLanguage language,
                                          const std::string& message,
                                          const std::string& target) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MESSAGECHAT));

    // Write chat type
    packet.writeUInt32(static_cast<uint32_t>(type));

    // Write language
    packet.writeUInt32(static_cast<uint32_t>(language));

    // Write target (for whispers) or channel name
    if (type == ChatType::WHISPER) {
        packet.writeString(target);
    } else if (type == ChatType::CHANNEL) {
        packet.writeString(target);  // Channel name
    }

    // Write message
    packet.writeString(message);

    LOG_DEBUG("Built CMSG_MESSAGECHAT packet");
    LOG_DEBUG("  Type: ", static_cast<int>(type));
    LOG_DEBUG("  Language: ", static_cast<int>(language));
    LOG_DEBUG("  Message: ", message);

    return packet;
}

bool MessageChatParser::parse(network::Packet& packet, MessageChatData& data) {
    // SMSG_MESSAGECHAT format (WoW 3.3.5a):
    // uint8 type
    // uint32 language
    // uint64 senderGuid
    // uint32 unknown (always 0)
    // [type-specific data]
    // uint32 messageLength
    // string message
    // uint8 chatTag

    if (packet.getSize() < 15) {
        LOG_ERROR("SMSG_MESSAGECHAT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    // Read chat type
    uint8_t typeVal = packet.readUInt8();
    data.type = static_cast<ChatType>(typeVal);

    // Read language
    uint32_t langVal = packet.readUInt32();
    data.language = static_cast<ChatLanguage>(langVal);

    // Read sender GUID
    data.senderGuid = packet.readUInt64();

    // Read unknown field
    packet.readUInt32();

    // Type-specific data
    // WoW 3.3.5 SMSG_MESSAGECHAT format: after senderGuid+unk, most types
    // have a receiverGuid (uint64). Some types have extra fields before it.
    switch (data.type) {
        case ChatType::MONSTER_SAY:
        case ChatType::MONSTER_YELL:
        case ChatType::MONSTER_EMOTE: {
            // Read sender name length + name
            uint32_t nameLen = packet.readUInt32();
            if (nameLen > 0 && nameLen < 256) {
                data.senderName.resize(nameLen);
                for (uint32_t i = 0; i < nameLen; ++i) {
                    data.senderName[i] = static_cast<char>(packet.readUInt8());
                }
            }
            // Read receiver GUID
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::CHANNEL: {
            // Read channel name, then receiver GUID
            data.channelName = packet.readString();
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::ACHIEVEMENT:
        case ChatType::GUILD_ACHIEVEMENT: {
            // Read target GUID
            data.receiverGuid = packet.readUInt64();
            break;
        }

        default:
            // SAY, GUILD, PARTY, YELL, WHISPER, WHISPER_INFORM, RAID, etc.
            // All have receiverGuid (typically senderGuid repeated)
            data.receiverGuid = packet.readUInt64();
            break;
    }

    // Read message length
    uint32_t messageLen = packet.readUInt32();

    // Read message
    if (messageLen > 0 && messageLen < 8192) {
        data.message.resize(messageLen);
        for (uint32_t i = 0; i < messageLen; ++i) {
            data.message[i] = static_cast<char>(packet.readUInt8());
        }
    }

    // Read chat tag
    data.chatTag = packet.readUInt8();

    LOG_DEBUG("Parsed SMSG_MESSAGECHAT:");
    LOG_DEBUG("  Type: ", getChatTypeString(data.type));
    LOG_DEBUG("  Language: ", static_cast<int>(data.language));
    LOG_DEBUG("  Sender GUID: 0x", std::hex, data.senderGuid, std::dec);
    if (!data.senderName.empty()) {
        LOG_DEBUG("  Sender name: ", data.senderName);
    }
    if (!data.channelName.empty()) {
        LOG_DEBUG("  Channel: ", data.channelName);
    }
    LOG_DEBUG("  Message: ", data.message);
    LOG_DEBUG("  Chat tag: 0x", std::hex, (int)data.chatTag, std::dec);

    return true;
}

const char* getChatTypeString(ChatType type) {
    switch (type) {
        case ChatType::SAY: return "SAY";
        case ChatType::PARTY: return "PARTY";
        case ChatType::RAID: return "RAID";
        case ChatType::GUILD: return "GUILD";
        case ChatType::OFFICER: return "OFFICER";
        case ChatType::YELL: return "YELL";
        case ChatType::WHISPER: return "WHISPER";
        case ChatType::WHISPER_INFORM: return "WHISPER_INFORM";
        case ChatType::EMOTE: return "EMOTE";
        case ChatType::TEXT_EMOTE: return "TEXT_EMOTE";
        case ChatType::SYSTEM: return "SYSTEM";
        case ChatType::MONSTER_SAY: return "MONSTER_SAY";
        case ChatType::MONSTER_YELL: return "MONSTER_YELL";
        case ChatType::MONSTER_EMOTE: return "MONSTER_EMOTE";
        case ChatType::CHANNEL: return "CHANNEL";
        case ChatType::CHANNEL_JOIN: return "CHANNEL_JOIN";
        case ChatType::CHANNEL_LEAVE: return "CHANNEL_LEAVE";
        case ChatType::CHANNEL_LIST: return "CHANNEL_LIST";
        case ChatType::CHANNEL_NOTICE: return "CHANNEL_NOTICE";
        case ChatType::CHANNEL_NOTICE_USER: return "CHANNEL_NOTICE_USER";
        case ChatType::AFK: return "AFK";
        case ChatType::DND: return "DND";
        case ChatType::IGNORED: return "IGNORED";
        case ChatType::SKILL: return "SKILL";
        case ChatType::LOOT: return "LOOT";
        case ChatType::BATTLEGROUND: return "BATTLEGROUND";
        case ChatType::BATTLEGROUND_LEADER: return "BATTLEGROUND_LEADER";
        case ChatType::RAID_LEADER: return "RAID_LEADER";
        case ChatType::RAID_WARNING: return "RAID_WARNING";
        case ChatType::ACHIEVEMENT: return "ACHIEVEMENT";
        case ChatType::GUILD_ACHIEVEMENT: return "GUILD_ACHIEVEMENT";
        default: return "UNKNOWN";
    }
}

// ============================================================
// Text Emotes
// ============================================================

network::Packet TextEmotePacket::build(uint32_t textEmoteId, uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_TEXT_EMOTE));
    packet.writeUInt32(textEmoteId);
    packet.writeUInt32(0);  // emoteNum (unused)
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_TEXT_EMOTE: emoteId=", textEmoteId, " target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

bool TextEmoteParser::parse(network::Packet& packet, TextEmoteData& data) {
    size_t bytesLeft = packet.getSize() - packet.getReadPos();
    if (bytesLeft < 20) {
        LOG_WARNING("SMSG_TEXT_EMOTE too short: ", bytesLeft, " bytes");
        return false;
    }
    data.senderGuid = packet.readUInt64();
    data.textEmoteId = packet.readUInt32();
    data.emoteNum = packet.readUInt32();
    uint32_t nameLen = packet.readUInt32();
    if (nameLen > 0 && nameLen <= 256) {
        data.targetName = packet.readString();
    } else if (nameLen > 0) {
        // Skip garbage
        return false;
    }
    return true;
}

// ============================================================
// Channel System
// ============================================================

network::Packet JoinChannelPacket::build(const std::string& channelName, const std::string& password) {
    network::Packet packet(wireOpcode(Opcode::CMSG_JOIN_CHANNEL));
    packet.writeUInt32(0);  // channelId (unused)
    packet.writeUInt8(0);   // hasVoice
    packet.writeUInt8(0);   // joinedByZone
    packet.writeString(channelName);
    packet.writeString(password);
    LOG_DEBUG("Built CMSG_JOIN_CHANNEL: channel=", channelName);
    return packet;
}

network::Packet LeaveChannelPacket::build(const std::string& channelName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEAVE_CHANNEL));
    packet.writeUInt32(0);  // channelId (unused)
    packet.writeString(channelName);
    LOG_DEBUG("Built CMSG_LEAVE_CHANNEL: channel=", channelName);
    return packet;
}

bool ChannelNotifyParser::parse(network::Packet& packet, ChannelNotifyData& data) {
    size_t bytesLeft = packet.getSize() - packet.getReadPos();
    if (bytesLeft < 2) {
        LOG_WARNING("SMSG_CHANNEL_NOTIFY too short");
        return false;
    }
    data.notifyType = static_cast<ChannelNotifyType>(packet.readUInt8());
    data.channelName = packet.readString();
    // Some notification types have additional fields (guid, etc.)
    bytesLeft = packet.getSize() - packet.getReadPos();
    if (bytesLeft >= 8) {
        data.senderGuid = packet.readUInt64();
    }
    return true;
}

// ============================================================
// Phase 1: Foundation — Targeting, Name Queries
// ============================================================

network::Packet SetSelectionPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_SELECTION));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_SET_SELECTION: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet SetActiveMoverPacket::build(uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_ACTIVE_MOVER));
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_SET_ACTIVE_MOVER: guid=0x", std::hex, guid, std::dec);
    return packet;
}

network::Packet InspectPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_INSPECT));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_INSPECT: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

// ============================================================
// Server Info Commands
// ============================================================

network::Packet QueryTimePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUERY_TIME));
    LOG_DEBUG("Built CMSG_QUERY_TIME");
    return packet;
}

bool QueryTimeResponseParser::parse(network::Packet& packet, QueryTimeResponseData& data) {
    data.serverTime = packet.readUInt32();
    data.timeOffset = packet.readUInt32();
    LOG_DEBUG("Parsed SMSG_QUERY_TIME_RESPONSE: time=", data.serverTime, " offset=", data.timeOffset);
    return true;
}

network::Packet RequestPlayedTimePacket::build(bool sendToChat) {
    network::Packet packet(wireOpcode(Opcode::CMSG_REQUEST_PLAYED_TIME));
    packet.writeUInt8(sendToChat ? 1 : 0);
    LOG_DEBUG("Built CMSG_REQUEST_PLAYED_TIME: sendToChat=", sendToChat);
    return packet;
}

bool PlayedTimeParser::parse(network::Packet& packet, PlayedTimeData& data) {
    data.totalTimePlayed = packet.readUInt32();
    data.levelTimePlayed = packet.readUInt32();
    data.triggerMessage = packet.readUInt8() != 0;
    LOG_DEBUG("Parsed SMSG_PLAYED_TIME: total=", data.totalTimePlayed, " level=", data.levelTimePlayed);
    return true;
}

network::Packet WhoPacket::build(uint32_t minLevel, uint32_t maxLevel,
                                 const std::string& playerName,
                                 const std::string& guildName,
                                 uint32_t raceMask, uint32_t classMask,
                                 uint32_t zones) {
    network::Packet packet(wireOpcode(Opcode::CMSG_WHO));
    packet.writeUInt32(minLevel);
    packet.writeUInt32(maxLevel);
    packet.writeString(playerName);
    packet.writeString(guildName);
    packet.writeUInt32(raceMask);
    packet.writeUInt32(classMask);
    packet.writeUInt32(zones);  // Number of zones
    LOG_DEBUG("Built CMSG_WHO: player=", playerName);
    return packet;
}

// ============================================================
// Social Commands
// ============================================================

network::Packet AddFriendPacket::build(const std::string& playerName, const std::string& note) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ADD_FRIEND));
    packet.writeString(playerName);
    packet.writeString(note);
    LOG_DEBUG("Built CMSG_ADD_FRIEND: player=", playerName);
    return packet;
}

network::Packet DelFriendPacket::build(uint64_t friendGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_DEL_FRIEND));
    packet.writeUInt64(friendGuid);
    LOG_DEBUG("Built CMSG_DEL_FRIEND: guid=0x", std::hex, friendGuid, std::dec);
    return packet;
}

network::Packet SetContactNotesPacket::build(uint64_t friendGuid, const std::string& note) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_CONTACT_NOTES));
    packet.writeUInt64(friendGuid);
    packet.writeString(note);
    LOG_DEBUG("Built CMSG_SET_CONTACT_NOTES: guid=0x", std::hex, friendGuid, std::dec);
    return packet;
}

bool FriendStatusParser::parse(network::Packet& packet, FriendStatusData& data) {
    data.status = packet.readUInt8();
    data.guid = packet.readUInt64();
    if (data.status == 1) {  // Online
        data.note = packet.readString();
        data.chatFlag = packet.readUInt8();
    }
    LOG_DEBUG("Parsed SMSG_FRIEND_STATUS: status=", (int)data.status, " guid=0x", std::hex, data.guid, std::dec);
    return true;
}

network::Packet AddIgnorePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ADD_IGNORE));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_ADD_IGNORE: player=", playerName);
    return packet;
}

network::Packet DelIgnorePacket::build(uint64_t ignoreGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_DEL_IGNORE));
    packet.writeUInt64(ignoreGuid);
    LOG_DEBUG("Built CMSG_DEL_IGNORE: guid=0x", std::hex, ignoreGuid, std::dec);
    return packet;
}

// ============================================================
// Logout Commands
// ============================================================

network::Packet LogoutRequestPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOGOUT_REQUEST));
    LOG_DEBUG("Built CMSG_LOGOUT_REQUEST");
    return packet;
}

network::Packet LogoutCancelPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOGOUT_CANCEL));
    LOG_DEBUG("Built CMSG_LOGOUT_CANCEL");
    return packet;
}

bool LogoutResponseParser::parse(network::Packet& packet, LogoutResponseData& data) {
    data.result = packet.readUInt32();
    data.instant = packet.readUInt8();
    LOG_DEBUG("Parsed SMSG_LOGOUT_RESPONSE: result=", data.result, " instant=", (int)data.instant);
    return true;
}

// ============================================================
// Stand State
// ============================================================

network::Packet StandStateChangePacket::build(uint8_t state) {
    network::Packet packet(wireOpcode(Opcode::CMSG_STAND_STATE_CHANGE));
    packet.writeUInt32(state);
    LOG_DEBUG("Built CMSG_STAND_STATE_CHANGE: state=", (int)state);
    return packet;
}

// ============================================================
// Display Toggles
// ============================================================

network::Packet ShowingHelmPacket::build(bool show) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SHOWING_HELM));
    packet.writeUInt8(show ? 1 : 0);
    LOG_DEBUG("Built CMSG_SHOWING_HELM: show=", show);
    return packet;
}

network::Packet ShowingCloakPacket::build(bool show) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SHOWING_CLOAK));
    packet.writeUInt8(show ? 1 : 0);
    LOG_DEBUG("Built CMSG_SHOWING_CLOAK: show=", show);
    return packet;
}

// ============================================================
// PvP
// ============================================================

network::Packet TogglePvpPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_TOGGLE_PVP));
    LOG_DEBUG("Built CMSG_TOGGLE_PVP");
    return packet;
}

// ============================================================
// Guild Commands
// ============================================================

network::Packet GuildInfoPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_INFO));
    LOG_DEBUG("Built CMSG_GUILD_INFO");
    return packet;
}

network::Packet GuildRosterPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_GET_ROSTER));
    LOG_DEBUG("Built CMSG_GUILD_GET_ROSTER");
    return packet;
}

network::Packet GuildMotdPacket::build(const std::string& motd) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_MOTD));
    packet.writeString(motd);
    LOG_DEBUG("Built CMSG_GUILD_MOTD: ", motd);
    return packet;
}

network::Packet GuildPromotePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_PROMOTE_MEMBER));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_PROMOTE_MEMBER: ", playerName);
    return packet;
}

network::Packet GuildDemotePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DEMOTE_MEMBER));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_DEMOTE_MEMBER: ", playerName);
    return packet;
}

network::Packet GuildLeavePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_LEAVE));
    LOG_DEBUG("Built CMSG_GUILD_LEAVE");
    return packet;
}

network::Packet GuildInvitePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_INVITE));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_INVITE: ", playerName);
    return packet;
}

network::Packet GuildQueryPacket::build(uint32_t guildId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_QUERY));
    packet.writeUInt32(guildId);
    LOG_DEBUG("Built CMSG_GUILD_QUERY: guildId=", guildId);
    return packet;
}

network::Packet GuildRemovePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_REMOVE));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_REMOVE: ", playerName);
    return packet;
}

network::Packet GuildAcceptPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_ACCEPT));
    LOG_DEBUG("Built CMSG_GUILD_ACCEPT");
    return packet;
}

network::Packet GuildDeclineInvitationPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DECLINE_INVITATION));
    LOG_DEBUG("Built CMSG_GUILD_DECLINE_INVITATION");
    return packet;
}

bool GuildQueryResponseParser::parse(network::Packet& packet, GuildQueryResponseData& data) {
    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_GUILD_QUERY_RESPONSE too small: ", packet.getSize());
        return false;
    }
    data.guildId = packet.readUInt32();
    data.guildName = packet.readString();
    for (int i = 0; i < 10; ++i) {
        data.rankNames[i] = packet.readString();
    }
    data.emblemStyle = packet.readUInt32();
    data.emblemColor = packet.readUInt32();
    data.borderStyle = packet.readUInt32();
    data.borderColor = packet.readUInt32();
    data.backgroundColor = packet.readUInt32();
    if ((packet.getSize() - packet.getReadPos()) >= 4) {
        data.rankCount = packet.readUInt32();
    }
    LOG_INFO("Parsed SMSG_GUILD_QUERY_RESPONSE: guild=", data.guildName, " id=", data.guildId);
    return true;
}

bool GuildInfoParser::parse(network::Packet& packet, GuildInfoData& data) {
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_GUILD_INFO too small: ", packet.getSize());
        return false;
    }
    data.guildName = packet.readString();
    data.creationDay = packet.readUInt32();
    data.creationMonth = packet.readUInt32();
    data.creationYear = packet.readUInt32();
    data.numMembers = packet.readUInt32();
    data.numAccounts = packet.readUInt32();
    LOG_INFO("Parsed SMSG_GUILD_INFO: ", data.guildName, " members=", data.numMembers);
    return true;
}

bool GuildRosterParser::parse(network::Packet& packet, GuildRosterData& data) {
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_GUILD_ROSTER too small: ", packet.getSize());
        return false;
    }
    uint32_t numMembers = packet.readUInt32();
    data.motd = packet.readString();
    data.guildInfo = packet.readString();

    uint32_t rankCount = packet.readUInt32();
    data.ranks.resize(rankCount);
    for (uint32_t i = 0; i < rankCount; ++i) {
        data.ranks[i].rights = packet.readUInt32();
        data.ranks[i].goldLimit = packet.readUInt32();
        // 6 bank tab flags + 6 bank tab items per day
        for (int t = 0; t < 6; ++t) {
            packet.readUInt32(); // tabFlags
            packet.readUInt32(); // tabItemsPerDay
        }
    }

    data.members.resize(numMembers);
    for (uint32_t i = 0; i < numMembers; ++i) {
        auto& m = data.members[i];
        m.guid = packet.readUInt64();
        m.online = (packet.readUInt8() != 0);
        m.name = packet.readString();
        m.rankIndex = packet.readUInt32();
        m.level = packet.readUInt8();
        m.classId = packet.readUInt8();
        m.gender = packet.readUInt8();
        m.zoneId = packet.readUInt32();
        if (!m.online) {
            m.lastOnline = packet.readFloat();
        }
        m.publicNote = packet.readString();
        m.officerNote = packet.readString();
    }
    LOG_INFO("Parsed SMSG_GUILD_ROSTER: ", numMembers, " members, motd=", data.motd);
    return true;
}

bool GuildEventParser::parse(network::Packet& packet, GuildEventData& data) {
    if (packet.getSize() < 2) {
        LOG_ERROR("SMSG_GUILD_EVENT too small: ", packet.getSize());
        return false;
    }
    data.eventType = packet.readUInt8();
    data.numStrings = packet.readUInt8();
    for (uint8_t i = 0; i < data.numStrings && i < 3; ++i) {
        data.strings[i] = packet.readString();
    }
    if ((packet.getSize() - packet.getReadPos()) >= 8) {
        data.guid = packet.readUInt64();
    }
    LOG_INFO("Parsed SMSG_GUILD_EVENT: type=", (int)data.eventType, " strings=", (int)data.numStrings);
    return true;
}

bool GuildInviteResponseParser::parse(network::Packet& packet, GuildInviteResponseData& data) {
    if (packet.getSize() < 2) {
        LOG_ERROR("SMSG_GUILD_INVITE too small: ", packet.getSize());
        return false;
    }
    data.inviterName = packet.readString();
    data.guildName = packet.readString();
    LOG_INFO("Parsed SMSG_GUILD_INVITE: from=", data.inviterName, " guild=", data.guildName);
    return true;
}

bool GuildCommandResultParser::parse(network::Packet& packet, GuildCommandResultData& data) {
    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_GUILD_COMMAND_RESULT too small: ", packet.getSize());
        return false;
    }
    data.command = packet.readUInt32();
    data.name = packet.readString();
    data.errorCode = packet.readUInt32();
    LOG_INFO("Parsed SMSG_GUILD_COMMAND_RESULT: cmd=", data.command, " error=", data.errorCode);
    return true;
}

// ============================================================
// Ready Check
// ============================================================

network::Packet ReadyCheckPacket::build() {
    network::Packet packet(wireOpcode(Opcode::MSG_RAID_READY_CHECK));
    LOG_DEBUG("Built MSG_RAID_READY_CHECK");
    return packet;
}

network::Packet ReadyCheckConfirmPacket::build(bool ready) {
    network::Packet packet(wireOpcode(Opcode::MSG_RAID_READY_CHECK_CONFIRM));
    packet.writeUInt8(ready ? 1 : 0);
    LOG_DEBUG("Built MSG_RAID_READY_CHECK_CONFIRM: ready=", ready);
    return packet;
}

// ============================================================
// Duel
// ============================================================

network::Packet DuelCancelPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_DUEL_CANCELLED));
    LOG_DEBUG("Built CMSG_DUEL_CANCELLED");
    return packet;
}

// ============================================================
// Party/Raid Management
// ============================================================

network::Packet GroupUninvitePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_UNINVITE_GUID));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GROUP_UNINVITE_GUID for player: ", playerName);
    return packet;
}

network::Packet GroupDisbandPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_DISBAND));
    LOG_DEBUG("Built CMSG_GROUP_DISBAND");
    return packet;
}

network::Packet RaidTargetUpdatePacket::build(uint8_t targetIndex, uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::MSG_RAID_TARGET_UPDATE));
    packet.writeUInt8(targetIndex);
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built MSG_RAID_TARGET_UPDATE, index: ", (uint32_t)targetIndex, ", guid: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet RequestRaidInfoPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_REQUEST_RAID_INFO));
    LOG_DEBUG("Built CMSG_REQUEST_RAID_INFO");
    return packet;
}

// ============================================================
// Combat and Trade
// ============================================================

network::Packet DuelProposedPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_DUEL_PROPOSED));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_DUEL_PROPOSED for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet InitiateTradePacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_INITIATE_TRADE));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_INITIATE_TRADE for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AttackSwingPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ATTACKSWING));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_ATTACKSWING for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AttackStopPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_ATTACKSTOP));
    LOG_DEBUG("Built CMSG_ATTACKSTOP");
    return packet;
}

network::Packet CancelCastPacket::build(uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CANCEL_CAST));
    packet.writeUInt32(0); // cast count/sequence
    packet.writeUInt32(spellId);
    LOG_DEBUG("Built CMSG_CANCEL_CAST for spell: ", spellId);
    return packet;
}

// ============================================================
// Random Roll
// ============================================================

network::Packet RandomRollPacket::build(uint32_t minRoll, uint32_t maxRoll) {
    network::Packet packet(wireOpcode(Opcode::MSG_RANDOM_ROLL));
    packet.writeUInt32(minRoll);
    packet.writeUInt32(maxRoll);
    LOG_DEBUG("Built MSG_RANDOM_ROLL: ", minRoll, "-", maxRoll);
    return packet;
}

bool RandomRollParser::parse(network::Packet& packet, RandomRollData& data) {
    data.rollerGuid = packet.readUInt64();
    data.targetGuid = packet.readUInt64();
    data.minRoll = packet.readUInt32();
    data.maxRoll = packet.readUInt32();
    data.result = packet.readUInt32();
    LOG_DEBUG("Parsed SMSG_RANDOM_ROLL: roller=0x", std::hex, data.rollerGuid, std::dec,
              " result=", data.result, " (", data.minRoll, "-", data.maxRoll, ")");
    return true;
}

network::Packet NameQueryPacket::build(uint64_t playerGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_NAME_QUERY));
    packet.writeUInt64(playerGuid);
    LOG_DEBUG("Built CMSG_NAME_QUERY: guid=0x", std::hex, playerGuid, std::dec);
    return packet;
}

bool NameQueryResponseParser::parse(network::Packet& packet, NameQueryResponseData& data) {
    // 3.3.5a: packedGuid, uint8 found
    // If found==0: CString name, CString realmName, uint8 race, uint8 gender, uint8 classId
    data.guid = UpdateObjectParser::readPackedGuid(packet);
    data.found = packet.readUInt8();

    if (data.found != 0) {
        LOG_DEBUG("Name query: player not found for GUID 0x", std::hex, data.guid, std::dec);
        return true; // Valid response, just not found
    }

    data.name = packet.readString();
    data.realmName = packet.readString();
    data.race = packet.readUInt8();
    data.gender = packet.readUInt8();
    data.classId = packet.readUInt8();

    LOG_INFO("Name query response: ", data.name, " (race=", (int)data.race,
             " class=", (int)data.classId, ")");
    return true;
}

network::Packet CreatureQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CREATURE_QUERY));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_CREATURE_QUERY: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

bool CreatureQueryResponseParser::parse(network::Packet& packet, CreatureQueryResponseData& data) {
    data.entry = packet.readUInt32();

    // High bit set means creature not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("Creature query: entry ", data.entry, " not found");
        data.name = "";
        return true;
    }

    // 4 name strings (only first is usually populated)
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4
    data.subName = packet.readString();
    data.iconName = packet.readString();
    data.typeFlags = packet.readUInt32();
    data.creatureType = packet.readUInt32();
    data.family = packet.readUInt32();
    data.rank = packet.readUInt32();

    // Skip remaining fields (kill credits, display IDs, modifiers, quest items, etc.)
    // We've got what we need for display purposes

    LOG_DEBUG("Creature query response: ", data.name, " (type=", data.creatureType,
             " rank=", data.rank, ")");
    return true;
}

// ---- GameObject Query ----

network::Packet GameObjectQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GAMEOBJECT_QUERY));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_GAMEOBJECT_QUERY: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

bool GameObjectQueryResponseParser::parse(network::Packet& packet, GameObjectQueryResponseData& data) {
    data.entry = packet.readUInt32();

    // High bit set means gameobject not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("GameObject query: entry ", data.entry, " not found");
        data.name = "";
        return true;
    }

    data.type = packet.readUInt32();       // GameObjectType
    /*uint32_t displayId =*/ packet.readUInt32();
    // 4 name strings (only first is usually populated)
    data.name = packet.readString();
    // name2, name3, name4
    packet.readString();
    packet.readString();
    packet.readString();

    LOG_DEBUG("GameObject query response: ", data.name, " (type=", data.type, " entry=", data.entry, ")");
    return true;
}

// ---- Item Query ----

network::Packet ItemQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_QUERY_SINGLE));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_ITEM_QUERY_SINGLE: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

static const char* getItemSubclassName(uint32_t itemClass, uint32_t subClass) {
    if (itemClass == 2) { // Weapon
        switch (subClass) {
            case 0: return "Axe"; case 1: return "Axe";
            case 2: return "Bow"; case 3: return "Gun";
            case 4: return "Mace"; case 5: return "Mace";
            case 6: return "Polearm"; case 7: return "Sword";
            case 8: return "Sword"; case 9: return "Obsolete";
            case 10: return "Staff"; case 13: return "Fist Weapon";
            case 15: return "Dagger"; case 16: return "Thrown";
            case 18: return "Crossbow"; case 19: return "Wand";
            case 20: return "Fishing Pole";
            default: return "Weapon";
        }
    }
    if (itemClass == 4) { // Armor
        switch (subClass) {
            case 0: return "Miscellaneous"; case 1: return "Cloth";
            case 2: return "Leather"; case 3: return "Mail";
            case 4: return "Plate"; case 6: return "Shield";
            default: return "Armor";
        }
    }
    return "";
}

bool ItemQueryResponseParser::parse(network::Packet& packet, ItemQueryResponseData& data) {
    data.entry = packet.readUInt32();

    // High bit set means item not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("Item query: entry ", data.entry, " not found");
        return true;
    }

    uint32_t itemClass = packet.readUInt32();
    uint32_t subClass = packet.readUInt32();
    packet.readUInt32(); // SoundOverrideSubclass

    data.subclassName = getItemSubclassName(itemClass, subClass);

    // 4 name strings
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4

    data.displayInfoId = packet.readUInt32();
    data.quality = packet.readUInt32();

    packet.readUInt32(); // Flags
    packet.readUInt32(); // Flags2
    packet.readUInt32(); // BuyPrice
    data.sellPrice = packet.readUInt32(); // SellPrice

    data.inventoryType = packet.readUInt32();

    packet.readUInt32(); // AllowableClass
    packet.readUInt32(); // AllowableRace
    packet.readUInt32(); // ItemLevel
    packet.readUInt32(); // RequiredLevel
    packet.readUInt32(); // RequiredSkill
    packet.readUInt32(); // RequiredSkillRank
    packet.readUInt32(); // RequiredSpell
    packet.readUInt32(); // RequiredHonorRank
    packet.readUInt32(); // RequiredCityRank
    packet.readUInt32(); // RequiredReputationFaction
    packet.readUInt32(); // RequiredReputationRank
    packet.readUInt32(); // MaxCount
    data.maxStack = static_cast<int32_t>(packet.readUInt32()); // Stackable
    data.containerSlots = packet.readUInt32();

    uint32_t statsCount = packet.readUInt32();
    // Server always sends 10 stat pairs; statsCount tells how many are meaningful
    for (uint32_t i = 0; i < 10; i++) {
        uint32_t statType = packet.readUInt32();
        int32_t statValue = static_cast<int32_t>(packet.readUInt32());
        if (i < statsCount) {
            switch (statType) {
                case 3: data.agility = statValue; break;
                case 4: data.strength = statValue; break;
                case 5: data.intellect = statValue; break;
                case 6: data.spirit = statValue; break;
                case 7: data.stamina = statValue; break;
                default: break;
            }
        }
    }

    packet.readUInt32(); // ScalingStatDistribution
    packet.readUInt32(); // ScalingStatValue

    // 5 damage types
    for (int i = 0; i < 5; i++) {
        packet.readFloat();   // DamageMin
        packet.readFloat();   // DamageMax
        packet.readUInt32();  // DamageType
    }

    data.armor = static_cast<int32_t>(packet.readUInt32());

    data.valid = !data.name.empty();
    LOG_INFO("Item query response: ", data.name, " (quality=", data.quality,
             " invType=", data.inventoryType, " stack=", data.maxStack, ")");
    return true;
}

// ============================================================
// Creature Movement
// ============================================================

bool MonsterMoveParser::parse(network::Packet& packet, MonsterMoveData& data) {
    // PackedGuid
    data.guid = UpdateObjectParser::readPackedGuid(packet);
    if (data.guid == 0) return false;

    // uint8 unk (toggle for MOVEMENTFLAG2_UNK7)
    if (packet.getReadPos() >= packet.getSize()) return false;
    packet.readUInt8();

    // Current position (server coords: float x, y, z)
    if (packet.getReadPos() + 12 > packet.getSize()) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();

    // uint32 splineId
    if (packet.getReadPos() + 4 > packet.getSize()) return false;
    packet.readUInt32();

    // uint8 moveType
    if (packet.getReadPos() >= packet.getSize()) return false;
    data.moveType = packet.readUInt8();

    if (data.moveType == 1) {
        // Stop - no more required data
        data.destX = data.x;
        data.destY = data.y;
        data.destZ = data.z;
        data.hasDest = false;
        return true;
    }

    // Read facing data based on move type
    if (data.moveType == 2) {
        // FacingSpot: float x, y, z
        if (packet.getReadPos() + 12 > packet.getSize()) return false;
        packet.readFloat(); packet.readFloat(); packet.readFloat();
    } else if (data.moveType == 3) {
        // FacingTarget: uint64 guid
        if (packet.getReadPos() + 8 > packet.getSize()) return false;
        data.facingTarget = packet.readUInt64();
    } else if (data.moveType == 4) {
        // FacingAngle: float angle
        if (packet.getReadPos() + 4 > packet.getSize()) return false;
        data.facingAngle = packet.readFloat();
    }

    // uint32 splineFlags
    if (packet.getReadPos() + 4 > packet.getSize()) return false;
    data.splineFlags = packet.readUInt32();

    // Check for animation flag (0x00000100)
    if (data.splineFlags & 0x00000100) {
        if (packet.getReadPos() + 8 > packet.getSize()) return false;
        packet.readUInt32(); // animId
        packet.readUInt32(); // effectStartTime
    }

    // uint32 duration
    if (packet.getReadPos() + 4 > packet.getSize()) return false;
    data.duration = packet.readUInt32();

    // Check for parabolic flag (0x00000200)
    if (data.splineFlags & 0x00000200) {
        if (packet.getReadPos() + 8 > packet.getSize()) return false;
        packet.readFloat(); // vertAccel
        packet.readUInt32(); // effectStartTime
    }

    // uint32 pointCount
    if (packet.getReadPos() + 4 > packet.getSize()) return false;
    uint32_t pointCount = packet.readUInt32();

    if (pointCount == 0) return true;

    // Read destination point(s)
    // If UncompressedPath flag (0x00040000): all points are full float x,y,z
    // Otherwise: first is packed destination, rest are packed deltas
    bool uncompressed = (data.splineFlags & 0x00040000) != 0;

    if (uncompressed) {
        // Read last point as destination
        // Skip to last point: each point is 12 bytes
        for (uint32_t i = 0; i < pointCount - 1; i++) {
            if (packet.getReadPos() + 12 > packet.getSize()) return true;
            packet.readFloat(); packet.readFloat(); packet.readFloat();
        }
        if (packet.getReadPos() + 12 > packet.getSize()) return true;
        data.destX = packet.readFloat();
        data.destY = packet.readFloat();
        data.destZ = packet.readFloat();
        data.hasDest = true;
    } else {
        // Compressed: first 3 floats are the destination (final point)
        if (packet.getReadPos() + 12 > packet.getSize()) return true;
        data.destX = packet.readFloat();
        data.destY = packet.readFloat();
        data.destZ = packet.readFloat();
        data.hasDest = true;
    }

    LOG_DEBUG("MonsterMove: guid=0x", std::hex, data.guid, std::dec,
              " type=", (int)data.moveType, " dur=", data.duration, "ms",
              " dest=(", data.destX, ",", data.destY, ",", data.destZ, ")");

    return true;
}

// ============================================================
// Phase 2: Combat Core
// ============================================================

bool AttackStartParser::parse(network::Packet& packet, AttackStartData& data) {
    if (packet.getSize() < 16) return false;
    data.attackerGuid = packet.readUInt64();
    data.victimGuid = packet.readUInt64();
    LOG_INFO("Attack started: 0x", std::hex, data.attackerGuid,
             " -> 0x", data.victimGuid, std::dec);
    return true;
}

bool AttackStopParser::parse(network::Packet& packet, AttackStopData& data) {
    data.attackerGuid = UpdateObjectParser::readPackedGuid(packet);
    data.victimGuid = UpdateObjectParser::readPackedGuid(packet);
    if (packet.getReadPos() < packet.getSize()) {
        data.unknown = packet.readUInt32();
    }
    LOG_INFO("Attack stopped: 0x", std::hex, data.attackerGuid, std::dec);
    return true;
}

bool AttackerStateUpdateParser::parse(network::Packet& packet, AttackerStateUpdateData& data) {
    data.hitInfo = packet.readUInt32();
    data.attackerGuid = UpdateObjectParser::readPackedGuid(packet);
    data.targetGuid = UpdateObjectParser::readPackedGuid(packet);
    data.totalDamage = static_cast<int32_t>(packet.readUInt32());
    data.subDamageCount = packet.readUInt8();

    for (uint8_t i = 0; i < data.subDamageCount; ++i) {
        SubDamage sub;
        sub.schoolMask = packet.readUInt32();
        sub.damage = packet.readFloat();
        sub.intDamage = packet.readUInt32();
        sub.absorbed = packet.readUInt32();
        sub.resisted = packet.readUInt32();
        data.subDamages.push_back(sub);
    }

    data.victimState = packet.readUInt32();
    data.overkill = static_cast<int32_t>(packet.readUInt32());

    // Read blocked amount
    if (packet.getReadPos() < packet.getSize()) {
        data.blocked = packet.readUInt32();
    }

    LOG_INFO("Melee hit: ", data.totalDamage, " damage",
             data.isCrit() ? " (CRIT)" : "",
             data.isMiss() ? " (MISS)" : "");
    return true;
}

bool SpellDamageLogParser::parse(network::Packet& packet, SpellDamageLogData& data) {
    data.targetGuid = UpdateObjectParser::readPackedGuid(packet);
    data.attackerGuid = UpdateObjectParser::readPackedGuid(packet);
    data.spellId = packet.readUInt32();
    data.damage = packet.readUInt32();
    data.overkill = packet.readUInt32();
    data.schoolMask = packet.readUInt8();
    data.absorbed = packet.readUInt32();
    data.resisted = packet.readUInt32();

    // Skip remaining fields
    uint8_t periodicLog = packet.readUInt8();
    (void)periodicLog;
    packet.readUInt8(); // unused
    packet.readUInt32(); // blocked
    uint32_t flags = packet.readUInt32();
    (void)flags;
    // Check crit flag
    data.isCrit = (flags & 0x02) != 0;

    LOG_INFO("Spell damage: spellId=", data.spellId, " dmg=", data.damage,
             data.isCrit ? " CRIT" : "");
    return true;
}

bool SpellHealLogParser::parse(network::Packet& packet, SpellHealLogData& data) {
    data.targetGuid = UpdateObjectParser::readPackedGuid(packet);
    data.casterGuid = UpdateObjectParser::readPackedGuid(packet);
    data.spellId = packet.readUInt32();
    data.heal = packet.readUInt32();
    data.overheal = packet.readUInt32();
    data.absorbed = packet.readUInt32();
    uint8_t critFlag = packet.readUInt8();
    data.isCrit = (critFlag != 0);

    LOG_INFO("Spell heal: spellId=", data.spellId, " heal=", data.heal,
             data.isCrit ? " CRIT" : "");
    return true;
}

// ============================================================
// XP Gain
// ============================================================

bool XpGainParser::parse(network::Packet& packet, XpGainData& data) {
    data.victimGuid = packet.readUInt64();
    data.totalXp = packet.readUInt32();
    data.type = packet.readUInt8();
    if (data.type == 0) {
        // Kill XP: float groupRate (1.0 = solo) + uint8 RAF flag
        float groupRate = packet.readFloat();
        packet.readUInt8(); // RAF bonus flag
        // Group bonus = total - (total / rate); only if grouped (rate > 1)
        if (groupRate > 1.0f) {
            data.groupBonus = data.totalXp - static_cast<uint32_t>(data.totalXp / groupRate);
        }
    }
    LOG_INFO("XP gain: ", data.totalXp, " xp (type=", static_cast<int>(data.type), ")");
    return data.totalXp > 0;
}

// ============================================================
// Phase 3: Spells, Action Bar, Auras
// ============================================================

bool InitialSpellsParser::parse(network::Packet& packet, InitialSpellsData& data) {
    size_t packetSize = packet.getSize();
    data.talentSpec = packet.readUInt8();
    uint16_t spellCount = packet.readUInt16();

    // Detect vanilla (uint16 spellId) vs WotLK (uint32 spellId) format
    // Vanilla: 4 bytes/spell (uint16 id + uint16 slot), WotLK: 6 bytes/spell (uint32 id + uint16 unk)
    size_t remainingAfterHeader = packetSize - 3; // subtract talentSpec(1) + spellCount(2)
    bool vanillaFormat = remainingAfterHeader < static_cast<size_t>(spellCount) * 6 + 2;

    LOG_INFO("SMSG_INITIAL_SPELLS: packetSize=", packetSize, " bytes, spellCount=", spellCount,
             vanillaFormat ? " (vanilla uint16 format)" : " (WotLK uint32 format)");

    data.spellIds.reserve(spellCount);
    for (uint16_t i = 0; i < spellCount; ++i) {
        uint32_t spellId;
        if (vanillaFormat) {
            spellId = packet.readUInt16();
            packet.readUInt16(); // slot
        } else {
            spellId = packet.readUInt32();
            packet.readUInt16(); // unknown (always 0)
        }
        if (spellId != 0) {
            data.spellIds.push_back(spellId);
        }
    }

    uint16_t cooldownCount = packet.readUInt16();
    data.cooldowns.reserve(cooldownCount);
    for (uint16_t i = 0; i < cooldownCount; ++i) {
        SpellCooldownEntry entry;
        if (vanillaFormat) {
            entry.spellId = packet.readUInt16();
        } else {
            entry.spellId = packet.readUInt32();
        }
        entry.itemId = packet.readUInt16();
        entry.categoryId = packet.readUInt16();
        entry.cooldownMs = packet.readUInt32();
        entry.categoryCooldownMs = packet.readUInt32();
        data.cooldowns.push_back(entry);
    }

    LOG_INFO("Initial spells parsed: ", data.spellIds.size(), " spells, ",
             data.cooldowns.size(), " cooldowns");

    // Log first 10 spell IDs for debugging
    if (!data.spellIds.empty()) {
        std::string first10;
        for (size_t i = 0; i < std::min(size_t(10), data.spellIds.size()); ++i) {
            if (!first10.empty()) first10 += ", ";
            first10 += std::to_string(data.spellIds[i]);
        }
        LOG_INFO("First spells: ", first10);
    }

    return true;
}

network::Packet CastSpellPacket::build(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CAST_SPELL));
    packet.writeUInt8(castCount);
    packet.writeUInt32(spellId);
    packet.writeUInt8(0x00); // castFlags = 0 for normal cast

    // SpellCastTargets
    if (targetGuid != 0) {
        packet.writeUInt32(0x02); // TARGET_FLAG_UNIT

        // Write packed GUID
        uint8_t mask = 0;
        uint8_t bytes[8];
        int byteCount = 0;
        uint64_t g = targetGuid;
        for (int i = 0; i < 8; ++i) {
            uint8_t b = g & 0xFF;
            if (b != 0) {
                mask |= (1 << i);
                bytes[byteCount++] = b;
            }
            g >>= 8;
        }
        packet.writeUInt8(mask);
        for (int i = 0; i < byteCount; ++i) {
            packet.writeUInt8(bytes[i]);
        }
    } else {
        packet.writeUInt32(0x00); // TARGET_FLAG_SELF
    }

    LOG_DEBUG("Built CMSG_CAST_SPELL: spell=", spellId, " target=0x",
              std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet CancelAuraPacket::build(uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CANCEL_AURA));
    packet.writeUInt32(spellId);
    return packet;
}

bool CastFailedParser::parse(network::Packet& packet, CastFailedData& data) {
    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.result = packet.readUInt8();
    LOG_INFO("Cast failed: spell=", data.spellId, " result=", (int)data.result);
    return true;
}

bool SpellStartParser::parse(network::Packet& packet, SpellStartData& data) {
    data.casterGuid = UpdateObjectParser::readPackedGuid(packet);
    data.casterUnit = UpdateObjectParser::readPackedGuid(packet);
    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.castFlags = packet.readUInt32();
    data.castTime = packet.readUInt32();

    // Read target flags and target (simplified)
    if (packet.getReadPos() < packet.getSize()) {
        uint32_t targetFlags = packet.readUInt32();
        if (targetFlags & 0x02) { // TARGET_FLAG_UNIT
            data.targetGuid = UpdateObjectParser::readPackedGuid(packet);
        }
    }

    LOG_INFO("Spell start: spell=", data.spellId, " castTime=", data.castTime, "ms");
    return true;
}

bool SpellGoParser::parse(network::Packet& packet, SpellGoData& data) {
    data.casterGuid = UpdateObjectParser::readPackedGuid(packet);
    data.casterUnit = UpdateObjectParser::readPackedGuid(packet);
    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.castFlags = packet.readUInt32();
    // Timestamp in 3.3.5a
    packet.readUInt32();

    data.hitCount = packet.readUInt8();
    data.hitTargets.reserve(data.hitCount);
    for (uint8_t i = 0; i < data.hitCount; ++i) {
        data.hitTargets.push_back(packet.readUInt64());
    }

    data.missCount = packet.readUInt8();
    // Skip miss details for now

    LOG_INFO("Spell go: spell=", data.spellId, " hits=", (int)data.hitCount,
             " misses=", (int)data.missCount);
    return true;
}

bool AuraUpdateParser::parse(network::Packet& packet, AuraUpdateData& data, bool isAll) {
    data.guid = UpdateObjectParser::readPackedGuid(packet);

    while (packet.getReadPos() < packet.getSize()) {
        uint8_t slot = packet.readUInt8();
        uint32_t spellId = packet.readUInt32();

        AuraSlot aura;
        if (spellId != 0) {
            aura.spellId = spellId;
            aura.flags = packet.readUInt8();
            aura.level = packet.readUInt8();
            aura.charges = packet.readUInt8();

            if (!(aura.flags & 0x08)) { // NOT_CASTER flag
                aura.casterGuid = UpdateObjectParser::readPackedGuid(packet);
            }

            if (aura.flags & 0x20) { // DURATION
                aura.maxDurationMs = static_cast<int32_t>(packet.readUInt32());
                aura.durationMs = static_cast<int32_t>(packet.readUInt32());
            }

            if (aura.flags & 0x40) { // EFFECT_AMOUNTS
                // Only read amounts for active effect indices (flags 0x01, 0x02, 0x04)
                for (int i = 0; i < 3; ++i) {
                    if (aura.flags & (1 << i)) {
                        if (packet.getReadPos() < packet.getSize()) {
                            packet.readUInt32();
                        }
                    }
                }
            }
        }

        data.updates.push_back({slot, aura});

        // For single update, only one entry
        if (!isAll) break;
    }

    LOG_DEBUG("Aura update for 0x", std::hex, data.guid, std::dec,
              ": ", data.updates.size(), " slots");
    return true;
}

bool SpellCooldownParser::parse(network::Packet& packet, SpellCooldownData& data) {
    data.guid = packet.readUInt64();
    data.flags = packet.readUInt8();

    while (packet.getReadPos() + 8 <= packet.getSize()) {
        uint32_t spellId = packet.readUInt32();
        uint32_t cooldownMs = packet.readUInt32();
        data.cooldowns.push_back({spellId, cooldownMs});
    }

    LOG_DEBUG("Spell cooldowns: ", data.cooldowns.size(), " entries");
    return true;
}

// ============================================================
// Phase 4: Group/Party System
// ============================================================

network::Packet GroupInvitePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_INVITE));
    packet.writeString(playerName);
    packet.writeUInt32(0); // unused
    LOG_DEBUG("Built CMSG_GROUP_INVITE: ", playerName);
    return packet;
}

bool GroupInviteResponseParser::parse(network::Packet& packet, GroupInviteResponseData& data) {
    data.canAccept = packet.readUInt8();
    data.inviterName = packet.readString();
    LOG_INFO("Group invite from: ", data.inviterName, " (canAccept=", (int)data.canAccept, ")");
    return true;
}

network::Packet GroupAcceptPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_ACCEPT));
    packet.writeUInt32(0); // unused in 3.3.5a
    return packet;
}

network::Packet GroupDeclinePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_DECLINE));
    return packet;
}

bool GroupListParser::parse(network::Packet& packet, GroupListData& data) {
    data.groupType = packet.readUInt8();
    data.subGroup = packet.readUInt8();
    data.flags = packet.readUInt8();
    data.roles = packet.readUInt8();

    // Skip LFG data if present
    if (data.groupType & 0x04) {
        packet.readUInt8(); // lfg state
        packet.readUInt32(); // lfg entry
        packet.readUInt8(); // lfg flags (3.3.5a may not have this)
    }

    packet.readUInt64(); // group GUID
    packet.readUInt32(); // counter

    data.memberCount = packet.readUInt32();
    data.members.reserve(data.memberCount);

    for (uint32_t i = 0; i < data.memberCount; ++i) {
        GroupMember member;
        member.name = packet.readString();
        member.guid = packet.readUInt64();
        member.isOnline = packet.readUInt8();
        member.subGroup = packet.readUInt8();
        member.flags = packet.readUInt8();
        member.roles = packet.readUInt8();
        data.members.push_back(member);
    }

    data.leaderGuid = packet.readUInt64();

    if (data.memberCount > 0 && packet.getReadPos() < packet.getSize()) {
        data.lootMethod = packet.readUInt8();
        data.looterGuid = packet.readUInt64();
        data.lootThreshold = packet.readUInt8();
        data.difficultyId = packet.readUInt8();
        data.raidDifficultyId = packet.readUInt8();
        if (packet.getReadPos() < packet.getSize()) {
            packet.readUInt8(); // unknown byte
        }
    }

    LOG_INFO("Group list: ", data.memberCount, " members, leader=0x",
             std::hex, data.leaderGuid, std::dec);
    return true;
}

bool PartyCommandResultParser::parse(network::Packet& packet, PartyCommandResultData& data) {
    data.command = static_cast<PartyCommand>(packet.readUInt32());
    data.name = packet.readString();
    data.result = static_cast<PartyResult>(packet.readUInt32());
    LOG_INFO("Party command result: ", (int)data.result);
    return true;
}

bool GroupDeclineResponseParser::parse(network::Packet& packet, GroupDeclineData& data) {
    data.playerName = packet.readString();
    LOG_INFO("Group decline from: ", data.playerName);
    return true;
}

// ============================================================
// Phase 5: Loot System
// ============================================================

network::Packet LootPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_LOOT: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AutostoreLootItemPacket::build(uint8_t slotIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_AUTOSTORE_LOOT_ITEM));
    packet.writeUInt8(slotIndex);
    return packet;
}

network::Packet UseItemPacket::build(uint8_t bagIndex, uint8_t slotIndex, uint64_t itemGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_USE_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    packet.writeUInt8(0);  // cast count
    packet.writeUInt32(0); // spell id
    packet.writeUInt64(itemGuid);
    packet.writeUInt32(0); // glyph index
    packet.writeUInt8(0);  // cast flags
    // SpellCastTargets: self
    packet.writeUInt32(0x00);
    return packet;
}

network::Packet AutoEquipItemPacket::build(uint8_t srcBag, uint8_t srcSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_AUTOEQUIP_ITEM));
    packet.writeUInt8(srcBag);
    packet.writeUInt8(srcSlot);
    return packet;
}

network::Packet SwapItemPacket::build(uint8_t dstBag, uint8_t dstSlot, uint8_t srcBag, uint8_t srcSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SWAP_ITEM));
    packet.writeUInt8(dstBag);
    packet.writeUInt8(dstSlot);
    packet.writeUInt8(srcBag);
    packet.writeUInt8(srcSlot);
    return packet;
}

network::Packet SwapInvItemPacket::build(uint8_t srcSlot, uint8_t dstSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SWAP_INV_ITEM));
    packet.writeUInt8(srcSlot);
    packet.writeUInt8(dstSlot);
    return packet;
}

network::Packet LootMoneyPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_MONEY));
    return packet;
}

network::Packet LootReleasePacket::build(uint64_t lootGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_RELEASE));
    packet.writeUInt64(lootGuid);
    return packet;
}

bool LootResponseParser::parse(network::Packet& packet, LootResponseData& data) {
    data.lootGuid = packet.readUInt64();
    data.lootType = packet.readUInt8();
    data.gold = packet.readUInt32();
    uint8_t itemCount = packet.readUInt8();

    data.items.reserve(itemCount);
    for (uint8_t i = 0; i < itemCount; ++i) {
        LootItem item;
        item.slotIndex = packet.readUInt8();
        item.itemId = packet.readUInt32();
        item.count = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        item.randomSuffix = packet.readUInt32();
        item.randomPropertyId = packet.readUInt32();
        item.lootSlotType = packet.readUInt8();
        data.items.push_back(item);
    }

    LOG_INFO("Loot response: ", (int)itemCount, " items, ", data.gold, " copper");
    return true;
}

// ============================================================
// Phase 5: NPC Gossip
// ============================================================

network::Packet GossipHelloPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GOSSIP_HELLO));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet QuestgiverHelloPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_HELLO));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet GossipSelectOptionPacket::build(uint64_t npcGuid, uint32_t menuId, uint32_t optionId, const std::string& code) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GOSSIP_SELECT_OPTION));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(menuId);
    packet.writeUInt32(optionId);
    if (!code.empty()) {
        packet.writeString(code);
    }
    return packet;
}

network::Packet QuestgiverQueryQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_QUERY_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt8(1);  // isDialogContinued = 1 (from gossip)
    return packet;
}

network::Packet QuestgiverAcceptQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt32(0);  // unused
    return packet;
}

bool QuestDetailsParser::parse(network::Packet& packet, QuestDetailsData& data) {
    if (packet.getSize() < 28) return false;
    data.npcGuid = packet.readUInt64();
    /*informUnit*/ packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = packet.readString();
    data.details = packet.readString();
    data.objectives = packet.readString();

    if (packet.getReadPos() + 10 > packet.getSize()) {
        LOG_INFO("Quest details (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    /*activateAccept*/ packet.readUInt8();
    /*flags*/ packet.readUInt32();
    data.suggestedPlayers = packet.readUInt32();
    /*isFinished*/ packet.readUInt8();

    // Reward choice items: server always writes 6 entries (QUEST_REWARD_CHOICES_COUNT)
    if (packet.getReadPos() + 4 <= packet.getSize()) {
        /*choiceCount*/ packet.readUInt32();
        for (int i = 0; i < 6; i++) {
            if (packet.getReadPos() + 12 > packet.getSize()) break;
            packet.readUInt32(); // itemId
            packet.readUInt32(); // count
            packet.readUInt32(); // displayInfo
        }
    }

    // Reward items: server always writes 4 entries (QUEST_REWARDS_COUNT)
    if (packet.getReadPos() + 4 <= packet.getSize()) {
        /*rewardCount*/ packet.readUInt32();
        for (int i = 0; i < 4; i++) {
            if (packet.getReadPos() + 12 > packet.getSize()) break;
            packet.readUInt32(); // itemId
            packet.readUInt32(); // count
            packet.readUInt32(); // displayInfo
        }
    }

    // Money and XP rewards
    if (packet.getReadPos() + 4 <= packet.getSize())
        data.rewardMoney = packet.readUInt32();
    if (packet.getReadPos() + 4 <= packet.getSize())
        data.rewardXp = packet.readUInt32();

    LOG_INFO("Quest details: id=", data.questId, " title='", data.title, "'");
    return true;
}

bool GossipMessageParser::parse(network::Packet& packet, GossipMessageData& data) {
    data.npcGuid = packet.readUInt64();
    data.menuId = packet.readUInt32();
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

    data.options.clear();
    data.options.reserve(optionCount);
    for (uint32_t i = 0; i < optionCount; ++i) {
        GossipOption opt;
        opt.id = packet.readUInt32();
        opt.icon = packet.readUInt8();
        opt.isCoded = (packet.readUInt8() != 0);
        opt.boxMoney = packet.readUInt32();
        opt.text = packet.readString();
        opt.boxText = packet.readString();
        data.options.push_back(opt);
    }

    uint32_t questCount = packet.readUInt32();
    data.quests.clear();
    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        GossipQuestItem quest;
        quest.questId = packet.readUInt32();
        quest.questIcon = packet.readUInt32();
        quest.questLevel = static_cast<int32_t>(packet.readUInt32());
        quest.questFlags = packet.readUInt32();
        quest.isRepeatable = packet.readUInt8();
        quest.title = packet.readString();
        data.quests.push_back(quest);
    }

    LOG_INFO("Gossip: ", optionCount, " options, ", questCount, " quests");
    return true;
}

// ============================================================
// Bind Point (Hearthstone)
// ============================================================

network::Packet BinderActivatePacket::build(uint64_t npcGuid) {
    network::Packet pkt(wireOpcode(Opcode::CMSG_BINDER_ACTIVATE));
    pkt.writeUInt64(npcGuid);
    return pkt;
}

bool BindPointUpdateParser::parse(network::Packet& packet, BindPointUpdateData& data) {
    if (packet.getSize() < 20) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();
    data.mapId = packet.readUInt32();
    data.zoneId = packet.readUInt32();
    return true;
}

bool QuestRequestItemsParser::parse(network::Packet& packet, QuestRequestItemsData& data) {
    if (packet.getSize() - packet.getReadPos() < 20) return false;
    data.npcGuid = packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = packet.readString();
    data.completionText = packet.readString();

    if (packet.getReadPos() + 20 > packet.getSize()) {
        LOG_INFO("Quest request items (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    /*emoteDelay*/ packet.readUInt32();
    /*emote*/ packet.readUInt32();
    /*autoCloseOnCancel*/ packet.readUInt32();
    /*flags*/ packet.readUInt32();
    /*suggestedPlayers*/ packet.readUInt32();

    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    data.requiredMoney = packet.readUInt32();

    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    uint32_t requiredItemCount = packet.readUInt32();
    for (uint32_t i = 0; i < requiredItemCount; ++i) {
        if (packet.getReadPos() + 12 > packet.getSize()) break;
        QuestRewardItem item;
        item.itemId = packet.readUInt32();
        item.count = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        if (item.itemId > 0)
            data.requiredItems.push_back(item);
    }

    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    data.completableFlags = packet.readUInt32();

    LOG_INFO("Quest request items: id=", data.questId, " title='", data.title,
             "' items=", data.requiredItems.size(), " completable=", data.isCompletable());
    return true;
}

bool QuestOfferRewardParser::parse(network::Packet& packet, QuestOfferRewardData& data) {
    if (packet.getSize() - packet.getReadPos() < 20) return false;
    data.npcGuid = packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = packet.readString();
    data.rewardText = packet.readString();

    if (packet.getReadPos() + 10 > packet.getSize()) {
        LOG_INFO("Quest offer reward (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    /*autoFinish*/ packet.readUInt8();
    /*flags*/ packet.readUInt32();
    /*suggestedPlayers*/ packet.readUInt32();

    // Emotes
    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    uint32_t emoteCount = packet.readUInt32();
    for (uint32_t i = 0; i < emoteCount; ++i) {
        if (packet.getReadPos() + 8 > packet.getSize()) break;
        packet.readUInt32(); // delay
        packet.readUInt32(); // emote
    }

    // Choice reward items (pick one): count + 6 * (id, count, displayInfo)
    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    /*choiceCount*/ packet.readUInt32();
    for (uint32_t i = 0; i < 6; ++i) {
        if (packet.getReadPos() + 12 > packet.getSize()) break;
        QuestRewardItem item;
        item.itemId = packet.readUInt32();
        item.count = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        if (item.itemId > 0)
            data.choiceRewards.push_back(item);
    }

    // Fixed reward items: count + 4 * (id, count, displayInfo)
    if (packet.getReadPos() + 4 > packet.getSize()) return true;
    /*rewardCount*/ packet.readUInt32();
    for (uint32_t i = 0; i < 4; ++i) {
        if (packet.getReadPos() + 12 > packet.getSize()) break;
        QuestRewardItem item;
        item.itemId = packet.readUInt32();
        item.count = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        if (item.itemId > 0)
            data.fixedRewards.push_back(item);
    }

    // Money and XP
    if (packet.getReadPos() + 4 <= packet.getSize())
        data.rewardMoney = packet.readUInt32();
    if (packet.getReadPos() + 4 <= packet.getSize())
        data.rewardXp = packet.readUInt32();

    LOG_INFO("Quest offer reward: id=", data.questId, " title='", data.title,
             "' choices=", data.choiceRewards.size(), " fixed=", data.fixedRewards.size());
    return true;
}

network::Packet QuestgiverCompleteQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_COMPLETE_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    return packet;
}

network::Packet QuestgiverChooseRewardPacket::build(uint64_t npcGuid, uint32_t questId, uint32_t rewardIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_CHOOSE_REWARD));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt32(rewardIndex);
    return packet;
}

// ============================================================
// Phase 5: Vendor
// ============================================================

network::Packet ListInventoryPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LIST_INVENTORY));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet BuyItemPacket::build(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
    network::Packet packet(wireOpcode(Opcode::CMSG_BUY_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(itemId);
    packet.writeUInt32(slot);
    packet.writeUInt32(count);
    packet.writeUInt8(0);  // bag slot (0 = find any available bag slot)
    return packet;
}

network::Packet SellItemPacket::build(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SELL_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt64(itemGuid);
    packet.writeUInt32(count);
    return packet;
}

bool ListInventoryParser::parse(network::Packet& packet, ListInventoryData& data) {
    data.vendorGuid = packet.readUInt64();
    uint8_t itemCount = packet.readUInt8();

    if (itemCount == 0) {
        LOG_INFO("Vendor has nothing for sale");
        return true;
    }

    data.items.reserve(itemCount);
    for (uint8_t i = 0; i < itemCount; ++i) {
        VendorItem item;
        item.slot = packet.readUInt32();
        item.itemId = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        item.maxCount = static_cast<int32_t>(packet.readUInt32());
        item.buyPrice = packet.readUInt32();
        item.durability = packet.readUInt32();
        item.stackCount = packet.readUInt32();
        item.extendedCost = packet.readUInt32();
        data.items.push_back(item);
    }

    LOG_INFO("Vendor inventory: ", (int)itemCount, " items");
    return true;
}

// ============================================================
// Trainer
// ============================================================

bool TrainerListParser::parse(network::Packet& packet, TrainerListData& data) {
    data = TrainerListData{};
    data.trainerGuid = packet.readUInt64();
    data.trainerType = packet.readUInt32();
    uint32_t spellCount = packet.readUInt32();

    if (spellCount > 1000) {
        LOG_ERROR("TrainerListParser: unreasonable spell count ", spellCount);
        return false;
    }

    data.spells.reserve(spellCount);
    for (uint32_t i = 0; i < spellCount; ++i) {
        TrainerSpell spell;
        spell.spellId    = packet.readUInt32();
        spell.state      = packet.readUInt8();
        spell.spellCost  = packet.readUInt32();
        spell.profDialog = packet.readUInt32();
        spell.profButton = packet.readUInt32();
        spell.reqLevel   = packet.readUInt8();
        spell.reqSkill   = packet.readUInt32();
        spell.reqSkillValue = packet.readUInt32();
        spell.chainNode1 = packet.readUInt32();
        spell.chainNode2 = packet.readUInt32();
        spell.chainNode3 = packet.readUInt32();
        data.spells.push_back(spell);
    }

    data.greeting = packet.readString();

    LOG_INFO("Trainer list: ", spellCount, " spells, type=", data.trainerType,
             ", greeting=\"", data.greeting, "\"");
    return true;
}

network::Packet TrainerBuySpellPacket::build(uint64_t trainerGuid, uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_TRAINER_BUY_SPELL));
    packet.writeUInt64(trainerGuid);
    packet.writeUInt32(spellId);
    return packet;
}

// ============================================================
// Talents
// ============================================================

bool TalentsInfoParser::parse(network::Packet& packet, TalentsInfoData& data) {
    // SMSG_TALENTS_INFO format (AzerothCore variant):
    // uint8  activeSpec
    // uint8  unspentPoints
    // be32   talentCount (metadata, may not match entry count)
    // be16   entryCount (actual number of id+rank entries)
    // Entry[entryCount]: { le32 id, uint8 rank }
    // le32   glyphSlots
    // le16   glyphIds[glyphSlots]

    const size_t startPos = packet.getReadPos();
    const size_t remaining = packet.getSize() - startPos;

    if (remaining < 2 + 4 + 2) {
        LOG_ERROR("SMSG_TALENTS_INFO: packet too short (remaining=", remaining, ")");
        return false;
    }

    data = TalentsInfoData{};

    // Read header
    data.talentSpec = packet.readUInt8();
    data.unspentPoints = packet.readUInt8();

    // These two counts are big-endian (network byte order)
    uint32_t talentCountBE = packet.readUInt32();
    uint32_t talentCount = __builtin_bswap32(talentCountBE);

    uint16_t entryCountBE = packet.readUInt16();
    uint16_t entryCount = __builtin_bswap16(entryCountBE);

    // Sanity check: prevent corrupt packets from allocating excessive memory
    if (entryCount > 64) {
        LOG_ERROR("SMSG_TALENTS_INFO: entryCount too large (", entryCount, "), rejecting packet");
        return false;
    }

    LOG_INFO("SMSG_TALENTS_INFO: spec=", (int)data.talentSpec,
             " unspent=", (int)data.unspentPoints,
             " talentCount=", talentCount,
             " entryCount=", entryCount);

    // Parse learned entries (id + rank pairs)
    // These may be talents, glyphs, or other learned abilities
    data.talents.clear();
    data.talents.reserve(entryCount);

    for (uint16_t i = 0; i < entryCount; ++i) {
        if (packet.getSize() - packet.getReadPos() < 5) {
            LOG_ERROR("SMSG_TALENTS_INFO: truncated entry list at i=", i);
            return false;
        }
        uint32_t id = packet.readUInt32();  // LE
        uint8_t rank = packet.readUInt8();
        data.talents.push_back({id, rank});

        LOG_INFO("  Entry: id=", id, " rank=", (int)rank);
    }

    // Parse glyph tail: glyphSlots + glyphIds[]
    if (packet.getSize() - packet.getReadPos() < 1) {
        LOG_WARNING("SMSG_TALENTS_INFO: no glyph tail data");
        return true;  // Not fatal, older formats may not have glyphs
    }

    uint8_t glyphSlots = packet.readUInt8();

    // Sanity check: Wrath has 6 glyph slots, cap at 12 for safety
    if (glyphSlots > 12) {
        LOG_WARNING("SMSG_TALENTS_INFO: glyphSlots too large (", (int)glyphSlots, "), clamping to 12");
        glyphSlots = 12;
    }

    LOG_INFO("  GlyphSlots: ", (int)glyphSlots);

    data.glyphs.clear();
    data.glyphs.reserve(glyphSlots);

    for (uint8_t i = 0; i < glyphSlots; ++i) {
        if (packet.getSize() - packet.getReadPos() < 2) {
            LOG_ERROR("SMSG_TALENTS_INFO: truncated glyph list at i=", i);
            return false;
        }
        uint16_t glyphId = packet.readUInt16();  // LE
        data.glyphs.push_back(glyphId);
        if (glyphId != 0) {
            LOG_INFO("    Glyph slot ", i, ": ", glyphId);
        }
    }

    LOG_INFO("SMSG_TALENTS_INFO: bytesConsumed=", (packet.getReadPos() - startPos),
             " bytesRemaining=", (packet.getSize() - packet.getReadPos()));

    return true;
}

network::Packet LearnTalentPacket::build(uint32_t talentId, uint32_t requestedRank) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEARN_TALENT));
    packet.writeUInt32(talentId);
    packet.writeUInt32(requestedRank);
    return packet;
}

network::Packet TalentWipeConfirmPacket::build(bool accept) {
    network::Packet packet(wireOpcode(Opcode::MSG_TALENT_WIPE_CONFIRM));
    packet.writeUInt32(accept ? 1 : 0);
    return packet;
}

// ============================================================
// Death/Respawn
// ============================================================

network::Packet RepopRequestPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_REPOP_REQUEST));
    packet.writeUInt8(1);  // request release (1 = manual)
    return packet;
}

network::Packet SpiritHealerActivatePacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SPIRIT_HEALER_ACTIVATE));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet ResurrectResponsePacket::build(uint64_t casterGuid, bool accept) {
    network::Packet packet(wireOpcode(Opcode::CMSG_RESURRECT_RESPONSE));
    packet.writeUInt64(casterGuid);
    packet.writeUInt8(accept ? 1 : 0);
    return packet;
}

// ============================================================
// Taxi / Flight Paths
// ============================================================

bool ShowTaxiNodesParser::parse(network::Packet& packet, ShowTaxiNodesData& data) {
    if (packet.getSize() - packet.getReadPos() < 4 + 8 + 4 + TLK_TAXI_MASK_SIZE * 4) {
        LOG_ERROR("ShowTaxiNodesParser: packet too short");
        return false;
    }
    data.windowInfo = packet.readUInt32();
    data.npcGuid = packet.readUInt64();
    data.nearestNode = packet.readUInt32();
    for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
        data.nodeMask[i] = packet.readUInt32();
    }
    LOG_INFO("ShowTaxiNodes: window=", data.windowInfo, " npc=0x", std::hex, data.npcGuid, std::dec,
             " nearest=", data.nearestNode);
    return true;
}

bool ActivateTaxiReplyParser::parse(network::Packet& packet, ActivateTaxiReplyData& data) {
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining >= 4) {
        data.result = packet.readUInt32();
    } else if (remaining >= 1) {
        data.result = packet.readUInt8();
    } else {
        LOG_ERROR("ActivateTaxiReplyParser: packet too short");
        return false;
    }
    LOG_INFO("ActivateTaxiReply: result=", data.result);
    return true;
}

network::Packet ActivateTaxiExpressPacket::build(uint64_t npcGuid, uint32_t totalCost, const std::vector<uint32_t>& pathNodes) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ACTIVATETAXIEXPRESS));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(totalCost);
    packet.writeUInt32(static_cast<uint32_t>(pathNodes.size()));
    for (uint32_t nodeId : pathNodes) {
        packet.writeUInt32(nodeId);
    }
    LOG_INFO("ActivateTaxiExpress: npc=0x", std::hex, npcGuid, std::dec,
             " cost=", totalCost, " nodes=", pathNodes.size());
    return packet;
}

network::Packet ActivateTaxiPacket::build(uint64_t npcGuid, uint32_t srcNode, uint32_t destNode) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ACTIVATETAXI));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(srcNode);
    packet.writeUInt32(destNode);
    return packet;
}

network::Packet GameObjectUsePacket::build(uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GAMEOBJECT_USE));
    packet.writeUInt64(guid);
    return packet;
}

} // namespace game
} // namespace wowee
