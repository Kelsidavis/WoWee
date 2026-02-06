#include "game/world_packets.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_AUTH_SESSION));

    // AzerothCore 3.3.5a expects this exact field order:
    // Build, LoginServerID, Account, LoginServerType, LocalChallenge,
    // RegionID, BattlegroupID, RealmID, DosResponse, Digest, AddonInfo

    // Build number (uint32, little-endian)
    packet.writeUInt32(build);

    // Login server ID (uint32, always 0)
    packet.writeUInt32(0);

    // Account name (null-terminated string)
    packet.writeString(upperAccount);

    // Login server type (uint32, always 0)
    packet.writeUInt32(0);

    // LocalChallenge / Client seed (uint32, little-endian)
    packet.writeUInt32(clientSeed);

    // Region ID (uint32)
    packet.writeUInt32(0);

    // Battlegroup ID (uint32)
    packet.writeUInt32(0);

    // Realm ID (uint32)
    packet.writeUInt32(realmId);
    LOG_DEBUG("  Realm ID: ", realmId);

    // DOS response (uint64) - required for 3.x
    packet.writeUInt32(0);
    packet.writeUInt32(0);

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

    LOG_DEBUG("Auth hash input: ", hashInput.size(), " bytes");

    // Compute SHA1 hash
    return auth::Crypto::sha1(hashInput);
}

bool AuthChallengeParser::parse(network::Packet& packet, AuthChallengeData& data) {
    // SMSG_AUTH_CHALLENGE format (WoW 3.3.5a):
    // uint32 unknown1 (always 1?)
    // uint32 serverSeed

    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_AUTH_CHALLENGE packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.unknown1 = packet.readUInt32();
    data.serverSeed = packet.readUInt32();

    LOG_INFO("Parsed SMSG_AUTH_CHALLENGE:");
    LOG_INFO("  Unknown1: 0x", std::hex, data.unknown1, std::dec);
    LOG_INFO("  Server seed: 0x", std::hex, data.serverSeed, std::dec);

    // Note: 3.3.5a has additional data after this (seed2, etc.)
    // but we only need the first seed for authentication

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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CHAR_CREATE));

    packet.writeString(data.name);  // null-terminated name
    packet.writeUInt8(static_cast<uint8_t>(data.race));
    packet.writeUInt8(static_cast<uint8_t>(data.characterClass));
    packet.writeUInt8(static_cast<uint8_t>(data.gender));
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CHAR_ENUM));

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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_PLAYER_LOGIN));

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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_PING));

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

network::Packet MovementPacket::build(Opcode opcode, const MovementInfo& info, uint64_t playerGuid) {
    network::Packet packet(static_cast<uint16_t>(opcode));

    // Movement packet format (WoW 3.3.5a):
    // packed GUID
    // uint32 flags
    // uint16 flags2
    // uint32 time
    // float x, y, z
    // float orientation

    // Write packed GUID
    uint8_t mask = 0;
    uint8_t guidBytes[8];
    int guidByteCount = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t byte = static_cast<uint8_t>((playerGuid >> (i * 8)) & 0xFF);
        if (byte != 0) {
            mask |= (1 << i);
            guidBytes[guidByteCount++] = byte;
        }
    }
    packet.writeUInt8(mask);
    for (int i = 0; i < guidByteCount; i++) {
        packet.writeUInt8(guidBytes[i]);
    }

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

    static int mvLog = 10;
    if (mvLog-- > 0) {
        LOG_INFO("Movement pkt: opcode=0x", std::hex, static_cast<uint16_t>(opcode), std::dec,
                 " size=", packet.getSize(), " flags=0x", std::hex, info.flags, std::dec,
                 " pos=(", info.x, ",", info.y, ",", info.z, ")");
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

    LOG_DEBUG("  UpdateFlags: 0x", std::hex, updateFlags, std::dec);

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
            /*uint64_t transportGuid =*/ readPackedGuid(packet);
            /*float tX =*/ packet.readFloat();
            /*float tY =*/ packet.readFloat();
            /*float tZ =*/ packet.readFloat();
            /*float tO =*/ packet.readFloat();
            /*uint32_t tTime =*/ packet.readUInt32();
            /*int8_t tSeat =*/ packet.readUInt8();

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
        /*float runSpeed =*/ packet.readFloat();
        /*float runBackSpeed =*/ packet.readFloat();
        /*float swimSpeed =*/ packet.readFloat();
        /*float swimBackSpeed =*/ packet.readFloat();
        /*float flightSpeed =*/ packet.readFloat();
        /*float flightBackSpeed =*/ packet.readFloat();
        /*float turnRate =*/ packet.readFloat();
        /*float pitchRate =*/ packet.readFloat();

        // Spline data
        if (moveFlags & 0x08000000) { // MOVEMENTFLAG_SPLINE_ENABLED
            // Skip spline data for now - complex structure
            uint32_t splineFlags = packet.readUInt32();

            if (splineFlags & 0x00010000) { // has final point
                /*float finalX =*/ packet.readFloat();
                /*float finalY =*/ packet.readFloat();
                /*float finalZ =*/ packet.readFloat();
            } else if (splineFlags & 0x00020000) { // has final target
                /*uint64_t finalTarget =*/ packet.readUInt64();
            } else if (splineFlags & 0x00040000) { // has final angle
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
        // Transport position update
        /*uint64_t transportGuid =*/ readPackedGuid(packet);
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        /*float transportOffsetX =*/ packet.readFloat();
        /*float transportOffsetY =*/ packet.readFloat();
        /*float transportOffsetZ =*/ packet.readFloat();
        block.orientation = packet.readFloat();
        /*float corpseOrientation =*/ packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  POSITION: (", block.x, ", ", block.y, ", ", block.z, "), o=", block.orientation);
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
    // Read number of blocks (each block is 32 fields = 32 bits)
    uint8_t blockCount = packet.readUInt8();

    if (blockCount == 0) {
        return true; // No fields to update
    }

    LOG_DEBUG("  Parsing ", (int)blockCount, " field blocks");

    // Read update mask
    std::vector<uint32_t> updateMask(blockCount);
    for (int i = 0; i < blockCount; ++i) {
        updateMask[i] = packet.readUInt32();
    }

    // Read field values for each bit set in mask
    for (int blockIdx = 0; blockIdx < blockCount; ++blockIdx) {
        uint32_t mask = updateMask[blockIdx];

        for (int bit = 0; bit < 32; ++bit) {
            if (mask & (1 << bit)) {
                uint16_t fieldIndex = blockIdx * 32 + bit;
                uint32_t value = packet.readUInt32();
                block.fields[fieldIndex] = value;

                LOG_DEBUG("    Field[", fieldIndex, "] = 0x", std::hex, value, std::dec);
            }
        }
    }

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
    LOG_INFO("Parsing SMSG_UPDATE_OBJECT");

    // Read block count
    data.blockCount = packet.readUInt32();
    LOG_INFO("  Block count: ", data.blockCount);

    // Check for out-of-range objects first
    if (packet.getReadPos() + 1 <= packet.getSize()) {
        uint8_t firstByte = packet.readUInt8();

        if (firstByte == static_cast<uint8_t>(UpdateType::OUT_OF_RANGE_OBJECTS)) {
            // Read out-of-range GUID count
            uint32_t count = packet.readUInt32();
            LOG_INFO("  Out-of-range objects: ", count);

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

    LOG_INFO("Successfully parsed ", data.blocks.size(), " update blocks");

    return true;
}

bool DestroyObjectParser::parse(network::Packet& packet, DestroyObjectData& data) {
    // SMSG_DESTROY_OBJECT format:
    // uint64 guid
    // uint8 isDeath (0 = despawn, 1 = death)

    if (packet.getSize() < 9) {
        LOG_ERROR("SMSG_DESTROY_OBJECT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.guid = packet.readUInt64();
    data.isDeath = (packet.readUInt8() != 0);

    LOG_INFO("Parsed SMSG_DESTROY_OBJECT:");
    LOG_INFO("  GUID: 0x", std::hex, data.guid, std::dec);
    LOG_INFO("  Is death: ", data.isDeath ? "yes" : "no");

    return true;
}

network::Packet MessageChatPacket::build(ChatType type,
                                          ChatLanguage language,
                                          const std::string& message,
                                          const std::string& target) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_MESSAGECHAT));

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

            // Read receiver GUID (usually 0 for monsters)
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::WHISPER_INFORM: {
            // Read receiver name
            data.receiverName = packet.readString();
            break;
        }

        case ChatType::CHANNEL: {
            // Read channel name
            data.channelName = packet.readString();
            break;
        }

        case ChatType::ACHIEVEMENT:
        case ChatType::GUILD_ACHIEVEMENT: {
            // Read achievement ID
            packet.readUInt32();
            break;
        }

        default:
            // No additional data for most types
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
// Phase 1: Foundation — Targeting, Name Queries
// ============================================================

network::Packet SetSelectionPacket::build(uint64_t targetGuid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_SET_SELECTION));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_SET_SELECTION: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet SetActiveMoverPacket::build(uint64_t guid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_SET_ACTIVE_MOVER));
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_SET_ACTIVE_MOVER: guid=0x", std::hex, guid, std::dec);
    return packet;
}

network::Packet NameQueryPacket::build(uint64_t playerGuid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_NAME_QUERY));
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CREATURE_QUERY));
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

    LOG_INFO("Creature query response: ", data.name, " (type=", data.creatureType,
             " rank=", data.rank, ")");
    return true;
}

// ---- Item Query ----

network::Packet ItemQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_ITEM_QUERY_SINGLE));
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
    packet.readUInt32(); // SellPrice

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
    for (uint32_t i = 0; i < statsCount && i < 10; i++) {
        uint32_t statType = packet.readUInt32();
        int32_t statValue = static_cast<int32_t>(packet.readUInt32());
        switch (statType) {
            case 3: data.agility = statValue; break;
            case 4: data.strength = statValue; break;
            case 5: data.intellect = statValue; break;
            case 6: data.spirit = statValue; break;
            case 7: data.stamina = statValue; break;
            default: break;
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
// Phase 2: Combat Core
// ============================================================

network::Packet AttackSwingPacket::build(uint64_t targetGuid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_ATTACKSWING));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_ATTACKSWING: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AttackStopPacket::build() {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_ATTACKSTOP));
    LOG_DEBUG("Built CMSG_ATTACKSTOP");
    return packet;
}

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
        // Kill XP: has group bonus float (unused) + group bonus uint32
        packet.readFloat();
        data.groupBonus = packet.readUInt32();
    }
    LOG_INFO("XP gain: ", data.totalXp, " xp (type=", static_cast<int>(data.type), ")");
    return data.totalXp > 0;
}

// ============================================================
// Phase 3: Spells, Action Bar, Auras
// ============================================================

bool InitialSpellsParser::parse(network::Packet& packet, InitialSpellsData& data) {
    data.talentSpec = packet.readUInt8();
    uint16_t spellCount = packet.readUInt16();

    data.spellIds.reserve(spellCount);
    for (uint16_t i = 0; i < spellCount; ++i) {
        uint32_t spellId = packet.readUInt32();
        packet.readUInt16(); // unknown (always 0)
        if (spellId != 0) {
            data.spellIds.push_back(spellId);
        }
    }

    uint16_t cooldownCount = packet.readUInt16();
    data.cooldowns.reserve(cooldownCount);
    for (uint16_t i = 0; i < cooldownCount; ++i) {
        SpellCooldownEntry entry;
        entry.spellId = packet.readUInt32();
        entry.itemId = packet.readUInt16();
        entry.categoryId = packet.readUInt16();
        entry.cooldownMs = packet.readUInt32();
        entry.categoryCooldownMs = packet.readUInt32();
        data.cooldowns.push_back(entry);
    }

    LOG_INFO("Initial spells: ", data.spellIds.size(), " spells, ",
             data.cooldowns.size(), " cooldowns");
    return true;
}

network::Packet CastSpellPacket::build(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CAST_SPELL));
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

network::Packet CancelCastPacket::build(uint32_t spellId) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CANCEL_CAST));
    packet.writeUInt32(0); // sequence
    packet.writeUInt32(spellId);
    return packet;
}

network::Packet CancelAuraPacket::build(uint32_t spellId) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_CANCEL_AURA));
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

            if (aura.flags & 0x40) { // EFFECT_AMOUNTS - skip
                // 3 effect amounts
                for (int i = 0; i < 3; ++i) {
                    if (packet.getReadPos() < packet.getSize()) {
                        packet.readUInt32();
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GROUP_INVITE));
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GROUP_ACCEPT));
    packet.writeUInt32(0); // unused in 3.3.5a
    return packet;
}

network::Packet GroupDeclinePacket::build() {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GROUP_DECLINE));
    return packet;
}

network::Packet GroupDisbandPacket::build() {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GROUP_DISBAND));
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_LOOT));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_LOOT: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AutostoreLootItemPacket::build(uint8_t slotIndex) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_AUTOSTORE_LOOT_ITEM));
    packet.writeUInt8(slotIndex);
    return packet;
}

network::Packet LootReleasePacket::build(uint64_t lootGuid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_LOOT_RELEASE));
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
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GOSSIP_HELLO));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet GossipSelectOptionPacket::build(uint64_t npcGuid, uint32_t menuId, uint32_t optionId, const std::string& code) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_GOSSIP_SELECT_OPTION));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(menuId);
    packet.writeUInt32(optionId);
    if (!code.empty()) {
        packet.writeString(code);
    }
    return packet;
}

network::Packet QuestgiverQueryQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_QUERY_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt8(1);  // isDialogContinued = 1 (from gossip)
    return packet;
}

network::Packet QuestgiverAcceptQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt32(0);  // unused
    return packet;
}

bool GossipMessageParser::parse(network::Packet& packet, GossipMessageData& data) {
    data.npcGuid = packet.readUInt64();
    data.menuId = packet.readUInt32();
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

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
// Phase 5: Vendor
// ============================================================

network::Packet ListInventoryPacket::build(uint64_t npcGuid) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_LIST_INVENTORY));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet BuyItemPacket::build(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_BUY_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(itemId);
    packet.writeUInt32(slot);
    packet.writeUInt8(count);
    return packet;
}

network::Packet SellItemPacket::build(uint64_t vendorGuid, uint64_t itemGuid, uint8_t count) {
    network::Packet packet(static_cast<uint16_t>(Opcode::CMSG_SELL_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt64(itemGuid);
    packet.writeUInt8(count);
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

} // namespace game
} // namespace wowee
