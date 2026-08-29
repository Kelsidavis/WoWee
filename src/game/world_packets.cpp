#include "game/world_packets.hpp"
#include "game/packet_parsers.hpp"
#include "game/spline_packet.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>

namespace wowee {
namespace game {

bool AttackerStateUpdateData::readCommonHead(network::Packet& packet, size_t startPos) {
    auto rem = [&]() { return packet.getRemainingSize(); };

    hitInfo = packet.readUInt32();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    attackerGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    targetGuid = packet.readPackedGuid();

    // int32 totalDamage + uint8 subDamageCount
    if (rem() < 5) {
        packet.setReadPos(startPos);
        return false;
    }
    totalDamage = static_cast<int32_t>(packet.readUInt32());
    subDamageCount = packet.readUInt8();

    // A count the packet cannot back up is a count from a misread, so it is
    // capped by what is actually left rather than trusted.
    const uint8_t maxSubDamageCount = static_cast<uint8_t>(std::min<size_t>(rem() / 20, 64));
    if (subDamageCount > maxSubDamageCount) {
        subDamageCount = maxSubDamageCount;
    }

    subDamages.reserve(subDamageCount);
    for (uint8_t i = 0; i < subDamageCount; ++i) {
        if (rem() < 20) {
            packet.setReadPos(startPos);
            return false;
        }
        SubDamage sub;
        sub.schoolMask = packet.readUInt32();
        sub.damage     = packet.readFloat();
        sub.intDamage  = packet.readUInt32();
        sub.absorbed   = packet.readUInt32();
        sub.resisted   = packet.readUInt32();
        subDamages.push_back(sub);
    }
    subDamageCount = static_cast<uint8_t>(subDamages.size());
    return true;
}

bool ItemQueryResponseData::readCommonRequirements(network::Packet& packet) {
    // Out of line rather than in the header beside the struct, and not for
    // compile time: read_never_written_check takes its field names from
    // world_packets.hpp and then skips that file when it looks for writers,
    // so that a default initialiser is not mistaken for one. A parser method
    // living there is invisible to it the same way - the moment these five
    // fields moved into the header they were reported as read and never
    // written, all five of them, and the ceiling is zero.
    if (!packet.hasRemaining(14 * 4)) return false;
    allowableClass = packet.readUInt32();
    allowableRace = packet.readUInt32();
    itemLevel = packet.readUInt32();
    requiredLevel = packet.readUInt32();
    requiredSkill = packet.readUInt32();
    requiredSkillRank = packet.readUInt32();
    packet.readUInt32();  // RequiredSpell
    packet.readUInt32();  // RequiredHonorRank
    packet.readUInt32();  // RequiredCityRank
    requiredReputationFaction = packet.readUInt32();
    requiredReputationRank = packet.readUInt32();
    maxCount = static_cast<int32_t>(packet.readUInt32());  // 1 = Unique
    maxStack = static_cast<int32_t>(packet.readUInt32());
    containerSlots = packet.readUInt32();
    return true;
}


std::string normalizeWowTextTokens(std::string text, const std::string& playerName) {
    if (text.empty()) return text;

    size_t pos = 0;
    while ((pos = text.find('$', pos)) != std::string::npos) {
        if (pos + 1 >= text.size()) break;
        const char code = text[pos + 1];
        if (code == 'b' || code == 'B') {
            text.replace(pos, 2, "\n");
            ++pos;
        } else if ((code == 'n' || code == 'N') && !playerName.empty()) {
            text.replace(pos, 2, playerName);
            pos += playerName.size();
        } else {
            ++pos;
        }
    }

    pos = 0;
    while ((pos = text.find("|n", pos)) != std::string::npos) {
        text.replace(pos, 2, "\n");
        ++pos;
    }
    pos = 0;
    while ((pos = text.find("|N", pos)) != std::string::npos) {
        text.replace(pos, 2, "\n");
        ++pos;
    }

    return text;
}

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
                   upperAccount.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

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

    // Addon info - compressed block
    // Format differs between expansions:
    //   Vanilla/TBC (CMaNGOS): while-loop of {string name, uint8 flags, uint32 modulusCRC, uint32 urlCRC}
    //   WotLK (AzerothCore): uint32 addonCount + {string name, uint8 enabled, uint32 crc, uint32 unk} + uint32 clientTime
    std::vector<uint8_t> addonData;
    if (isTbc) {
        // Vanilla/TBC: each addon entry = null-terminated name + uint8 flags + uint32 modulusCRC + uint32 urlCRC
        // Send standard Blizzard addons that CMaNGOS anticheat expects for fingerprinting
        static const char* vanillaAddons[] = {
            "Blizzard_AuctionUI", "Blizzard_BattlefieldMinimap", "Blizzard_BindingUI",
            "Blizzard_CombatText", "Blizzard_CraftUI", "Blizzard_GMSurveyUI",
            "Blizzard_InspectUI", "Blizzard_MacroUI", "Blizzard_RaidUI",
            "Blizzard_TalentUI", "Blizzard_TradeSkillUI", "Blizzard_TrainerUI"
        };
        static constexpr uint32_t standardModulusCRC = 0x4C1C776D;
        for (const char* name : vanillaAddons) {
            // string (null-terminated)
            size_t len = strlen(name);
            addonData.insert(addonData.end(), reinterpret_cast<const uint8_t*>(name),
                             reinterpret_cast<const uint8_t*>(name) + len + 1);
            // uint8 flags = 1 (enabled)
            addonData.push_back(0x01);
            // uint32 modulusCRC (little-endian)
            addonData.push_back(static_cast<uint8_t>(standardModulusCRC & 0xFF));
            addonData.push_back(static_cast<uint8_t>((standardModulusCRC >> 8) & 0xFF));
            addonData.push_back(static_cast<uint8_t>((standardModulusCRC >> 16) & 0xFF));
            addonData.push_back(static_cast<uint8_t>((standardModulusCRC >> 24) & 0xFF));
            // uint32 urlCRC = 0
            addonData.push_back(0); addonData.push_back(0);
            addonData.push_back(0); addonData.push_back(0);
        }
    } else {
        // WotLK: uint32 addonCount + entries + uint32 clientTime
        // Send 0 addons
        addonData = { 0, 0, 0, 0,  // addonCount = 0
                      0, 0, 0, 0 }; // clientTime = 0
    }
    uint32_t decompressedSize = static_cast<uint32_t>(addonData.size());

    // Compress with zlib
    uLongf compressedSize = compressBound(decompressedSize);
    std::vector<uint8_t> compressed(compressedSize);
    int ret = compress(compressed.data(), &compressedSize, addonData.data(), decompressedSize);
    if (ret == Z_OK) {
        compressed.resize(compressedSize);
        // Write decompressedSize, then compressed bytes
        packet.writeUInt32(decompressedSize);
        packet.writeBytes(compressed.data(), compressed.size());
        LOG_DEBUG("Addon info: decompressedSize=", decompressedSize,
                  " compressedSize=", compressedSize, " addons=",
                  isTbc ? "12 vanilla" : "0 wotlk");
    } else {
        LOG_ERROR("zlib compress failed with code: ", ret);
        packet.writeUInt32(0);
    }

    LOG_INFO("CMSG_AUTH_SESSION packet built: ", packet.getSize(), " bytes");

    // Dump full packet for protocol debugging
    LOG_DEBUG("CMSG_AUTH_SESSION full dump:\n",
              core::toHexString(packet.getData().data(), packet.getData().size(), true));

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

    // Everything an AUTH_REJECT needs, and nothing that authenticates. The
    // session key is a credential (see GameHandler::onAuthSuccess) and
    // hashInput ends with all 40 of its bytes, so neither can be logged even at
    // DEBUG - a debug log is still a file that gets attached to bug reports.
    // The digest is what goes out on the wire, so it is safe and is the value
    // actually worth comparing against the server's.
    LOG_DEBUG("AUTH HASH: account='", accountName, "' clientSeed=0x", std::hex, clientSeed,
              " serverSeed=0x", serverSeed, std::dec, " sessionKey=", sessionKey.size(),
              " bytes input=", hashInput.size(), " bytes");

    // Compute SHA1 hash
    auto result = auth::Crypto::sha1(hashInput);
    LOG_DEBUG("AUTH HASH: digest=", core::toHexString(result.data(), result.size()));

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

    if (packet.getSize() <= 4) {
        // Original vanilla/TBC format: just the server seed (4 bytes)
        data.unknown1 = 0;
        data.serverSeed = packet.readUInt32();
        LOG_INFO("SMSG_AUTH_CHALLENGE: TBC format (", packet.getSize(), " bytes)");
    } else if (packet.getSize() < 40) {
        // Vanilla with encryption seeds (36 bytes): serverSeed + 32 bytes seeds
        // No "unknown1" prefix - first uint32 IS the server seed
        data.unknown1 = 0;
        data.serverSeed = packet.readUInt32();
        LOG_INFO("SMSG_AUTH_CHALLENGE: Classic+seeds format (", packet.getSize(), " bytes)");
    } else {
        // WotLK format (40+ bytes): unknown1 + serverSeed + 32 bytes encryption seeds
        data.unknown1 = packet.readUInt32();
        data.serverSeed = packet.readUInt32();
        LOG_INFO("SMSG_AUTH_CHALLENGE: WotLK format (", packet.getSize(), " bytes)");
        LOG_DEBUG("  Unknown1: 0x", std::hex, data.unknown1, std::dec);
    }

    LOG_DEBUG("  Server seed: 0x", std::hex, data.serverSeed, std::dec);

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
    // Turtle WoW / 1.12.1 clients send 4 extra zero bytes after outfitId.
    // Servers may validate packet length and silently drop undersized packets.
    packet.writeUInt32(0);

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
    LOG_DEBUG("CMSG_CHAR_CREATE full dump: ",
              core::toHexString(packet.getData().data(), packet.getData().size(), true));

    return packet;
}

bool CharCreateResponseParser::parse(network::Packet& packet, CharCreateResponseData& data) {
    // Validate minimum packet size: result(1)
    if (!packet.hasRemaining(1)) {
        LOG_WARNING("SMSG_CHAR_CREATE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

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

/// SMSG_CHAR_ENUM as Classic and TBC send it.
///
/// The two differ in exactly one thing: a TBC equipment entry carries a
/// trailing uint32 enchantment and a Classic one does not, which also moves
/// the minimum entry size from 20*5 to 20*9. Everything else, down to the
/// count cap and the order of the fields, was written out twice.
///
/// WotLK is deliberately not folded in here. CharEnumParser::parse guards
/// every field individually and substitutes a default when one is short,
/// where these two check the whole entry once and then read straight through.
/// That is a different answer to a malformed packet, not a different layout,
/// and choosing one for all three is a decision rather than a tidy-up.
bool parseCharEnumPreWotlk(network::Packet& packet, CharEnumResponse& response,
                           bool hasEnchantment, const char* tag) {
    if (packet.getSize() < 1) {
        LOG_ERROR(tag, " SMSG_CHAR_ENUM packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    uint8_t count = packet.readUInt8();

    // Cap count to prevent excessive memory allocation
    constexpr uint8_t kMaxCharacters = 32;
    if (count > kMaxCharacters) {
        LOG_WARNING(tag, " Character count ", static_cast<int>(count), " exceeds max ",
                    static_cast<int>(kMaxCharacters), ", capping");
        count = kMaxCharacters;
    }

    LOG_INFO(tag, " Parsing SMSG_CHAR_ENUM: ", static_cast<int>(count), " characters");

    response.characters.clear();
    response.characters.reserve(count);

    // guid(8) + name(1) + race(1) + class(1) + gender(1) + appearance(4)
    // + facialFeatures(1) + level(1) + zone(4) + map(4) + pos(12) + guild(4)
    // + flags(4) + firstLogin(1) + pet(12) + 20 equipment entries.
    const size_t bytesPerItem = hasEnchantment ? 9 : 5;
    const size_t minCharacterSize =
        8 + 1 + 1 + 1 + 1 + 4 + 1 + 1 + 4 + 4 + 12 + 4 + 4 + 1 + 12 + (20 * bytesPerItem);

    for (uint8_t i = 0; i < count; ++i) {
        if (!packet.hasRemaining(minCharacterSize)) {
            LOG_WARNING(tag, " Character enum packet truncated at character ",
                        static_cast<int>(i + 1), ", pos=", packet.getReadPos(),
                        " needed=", minCharacterSize, " size=", packet.getSize());
            break;
        }

        Character character;
        character.guid = packet.readUInt64();
        character.name = packet.readString();
        character.race = static_cast<Race>(packet.readUInt8());
        character.characterClass = static_cast<Class>(packet.readUInt8());
        character.gender = static_cast<Gender>(packet.readUInt8());
        // Appearance (skin, face, hairStyle, hairColor packed) then facial hair.
        character.appearanceBytes = packet.readUInt32();
        character.facialFeatures = packet.readUInt8();
        character.level = packet.readUInt8();
        character.zoneId = packet.readUInt32();
        character.mapId = packet.readUInt32();
        character.x = packet.readFloat();
        character.y = packet.readFloat();
        character.z = packet.readFloat();
        character.guildId = packet.readUInt32();
        character.flags = packet.readUInt32();
        // One byte here, where WotLK sends uint32 customization + uint8 unknown.
        /*uint8_t firstLogin =*/ packet.readUInt8();

        // Pet data, always present even with no pet.
        character.pet.displayModel = packet.readUInt32();
        character.pet.level = packet.readUInt32();
        character.pet.family = packet.readUInt32();

        // Twenty items, where WotLK has twenty-three.
        character.equipment.reserve(20);
        for (int j = 0; j < 20; ++j) {
            EquipmentItem item;
            item.displayModel = packet.readUInt32();
            item.inventoryType = packet.readUInt8();
            item.enchantment = hasEnchantment ? packet.readUInt32() : 0;
            character.equipment.push_back(item);
        }

        LOG_DEBUG("  Character ", static_cast<int>(i + 1), ": ", character.name,
                  " (", getRaceName(character.race), " ", getClassName(character.characterClass),
                  " level ", static_cast<int>(character.level), " zone ", character.zoneId, ")");

        response.characters.push_back(character);
    }

    LOG_INFO(tag, " Parsed ", response.characters.size(), " characters");
    return true;
}

bool CharEnumParser::parse(network::Packet& packet, CharEnumResponse& response) {
    // Upfront validation: count(1) + at least minimal character data
    if (!packet.hasRemaining(1)) return false;

    // Read character count
    uint8_t count = packet.readUInt8();

    LOG_INFO("Parsing SMSG_CHAR_ENUM: ", static_cast<int>(count), " characters");

    response.characters.clear();
    response.characters.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        Character character;

        // Validate minimum bytes for this character entry before reading:
        // GUID(8) + name(>=1 for empty string) + race(1) + class(1) + gender(1) +
        // appearanceBytes(4) + facialFeatures(1) + level(1) + zoneId(4) + mapId(4) +
        // x(4) + y(4) + z(4) + guildId(4) + flags(4) + customization(4) + unknown(1) +
        // petDisplayModel(4) + petLevel(4) + petFamily(4) + 23items*(dispModel(4)+invType(1)+enchant(4)) = 207 bytes
        const size_t minCharacterSize = 8 + 1 + 1 + 1 + 1 + 4 + 1 + 1 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1 + 4 + 4 + 4 + (23 * 9);
        if (!packet.hasRemaining(minCharacterSize)) {
            LOG_WARNING("CharEnumParser: truncated character at index ", static_cast<int>(i));
            break;
        }

        // Read GUID (8 bytes, little-endian)
        character.guid = packet.readUInt64();

        // Read name (null-terminated string) - validate before reading
        if (!packet.hasData()) {
            LOG_WARNING("CharEnumParser: no bytes for name at index ", static_cast<int>(i));
            break;
        }
        character.name = packet.readString();

        // Validate remaining bytes before reading fixed-size fields
        if (!packet.hasRemaining(1)) {
            LOG_WARNING("CharEnumParser: truncated before race/class/gender at index ", static_cast<int>(i));
            character.race = Race::HUMAN;
            character.characterClass = Class::WARRIOR;
            character.gender = Gender::MALE;
        } else {
            // Read race, class, gender
            character.race = static_cast<Race>(packet.readUInt8());
            if (!packet.hasRemaining(1)) {
                character.characterClass = Class::WARRIOR;
                character.gender = Gender::MALE;
            } else {
                character.characterClass = static_cast<Class>(packet.readUInt8());
                if (!packet.hasRemaining(1)) {
                    character.gender = Gender::MALE;
                } else {
                    character.gender = static_cast<Gender>(packet.readUInt8());
                }
            }
        }

        // Validate before reading appearance data
        if (!packet.hasRemaining(4)) {
            character.appearanceBytes = 0;
            character.facialFeatures = 0;
        } else {
            // Read appearance data
            character.appearanceBytes = packet.readUInt32();
            if (!packet.hasRemaining(1)) {
                character.facialFeatures = 0;
            } else {
                character.facialFeatures = packet.readUInt8();
            }
        }

        // Read level
        if (!packet.hasRemaining(1)) {
            character.level = 1;
        } else {
            character.level = packet.readUInt8();
        }

        // Read location
        if (!packet.hasRemaining(12)) {
            character.zoneId = 0;
            character.mapId = 0;
            character.x = 0.0f;
            character.y = 0.0f;
            character.z = 0.0f;
        } else {
            character.zoneId = packet.readUInt32();
            character.mapId = packet.readUInt32();
            character.x = packet.readFloat();
            character.y = packet.readFloat();
            character.z = packet.readFloat();
        }

        // Read affiliations
        if (!packet.hasRemaining(4)) {
            character.guildId = 0;
        } else {
            character.guildId = packet.readUInt32();
        }

        // Read flags
        if (!packet.hasRemaining(4)) {
            character.flags = 0;
        } else {
            character.flags = packet.readUInt32();
        }

        // Skip customization flag (uint32) and unknown byte
        if (!packet.hasRemaining(4)) {
            // Customization missing, skip unknown
        } else {
            packet.readUInt32();  // Customization
            if (!packet.hasRemaining(1)) {
                // Unknown missing
            } else {
                packet.readUInt8();   // Unknown
            }
        }

        // Read pet data (always present, even if no pet)
        if (!packet.hasRemaining(12)) {
            character.pet.displayModel = 0;
            character.pet.level = 0;
            character.pet.family = 0;
        } else {
            character.pet.displayModel = packet.readUInt32();
            character.pet.level = packet.readUInt32();
            character.pet.family = packet.readUInt32();
        }

        // Read equipment (23 items)
        character.equipment.reserve(23);
        for (int j = 0; j < 23; ++j) {
            if (!packet.hasRemaining(9)) break;
            EquipmentItem item;
            item.displayModel = packet.readUInt32();
            item.inventoryType = packet.readUInt8();
            item.enchantment = packet.readUInt32();
            character.equipment.push_back(item);
        }

        LOG_DEBUG("  Character ", static_cast<int>(i + 1), ": ", character.name,
                  " (", getRaceName(character.race), " ", getClassName(character.characterClass),
                  " level ", static_cast<int>(character.level), " zone ", character.zoneId, ")");

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
    // Common layouts seen in the wild:
    // - WotLK-like: uint32 serverTime, uint8 unk, uint32 mask, uint32[up to 8] slotTimes
    // - Older/variant: uint32 serverTime, uint8 unk, uint32[up to 8] slotTimes
    // Some servers only send a subset of slots.
    if (packet.getSize() < 5) {
        LOG_ERROR("SMSG_ACCOUNT_DATA_TIMES packet too small: ", packet.getSize(),
                  " bytes (need at least 5)");
        return false;
    }

    for (uint32_t& t : data.accountDataTimes) {
        t = 0;
    }
    data.serverTime = packet.readUInt32();
    data.unknown = packet.readUInt8();

    size_t remaining = packet.getRemainingSize();
    uint32_t mask = 0xFF;
    if (remaining >= 4 && ((remaining - 4) % 4) == 0) {
        // Treat first dword as slot mask when payload shape matches.
        mask = packet.readUInt32();
    }
    remaining = packet.getRemainingSize();
    size_t slotWords = std::min<size_t>(8, remaining / 4);

    LOG_DEBUG("Parsed SMSG_ACCOUNT_DATA_TIMES:");
    LOG_DEBUG("  Server time: ", data.serverTime);
    LOG_DEBUG("  Unknown: ", static_cast<int>(data.unknown));
    LOG_DEBUG("  Mask: 0x", std::hex, mask, std::dec, " slotsInPacket=", slotWords);

    for (size_t i = 0; i < slotWords; ++i) {
        data.accountDataTimes[i] = packet.readUInt32();
        if (data.accountDataTimes[i] != 0 || ((mask & (1u << i)) != 0)) {
            LOG_DEBUG("  Data slot ", i, ": ", data.accountDataTimes[i]);
        }
    }
    if (packet.getReadPos() != packet.getSize()) {
        LOG_DEBUG("  AccountDataTimes trailing bytes: ", packet.getRemainingSize());
        packet.skipAll();
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

    // Cap lineCount to prevent unbounded memory allocation
    const uint32_t MAX_MOTD_LINES = 64;
    if (lineCount > MAX_MOTD_LINES) {
        LOG_WARNING("MotdParser: lineCount capped (requested=", lineCount, ")");
        lineCount = MAX_MOTD_LINES;
    }

    LOG_INFO("Parsed SMSG_MOTD: ", lineCount, " line(s)");

    data.lines.clear();
    data.lines.reserve(lineCount);

    for (uint32_t i = 0; i < lineCount; ++i) {
        // Validate at least 1 byte available for the string
        if (!packet.hasData()) {
            LOG_WARNING("MotdParser: truncated at line ", i + 1);
            break;
        }
        std::string line = packet.readString();
        data.lines.push_back(line);
        LOG_DEBUG("  MOTD[", i + 1, "]: ", line);
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
    packet.writeFloat(info.x);
    packet.writeFloat(info.y);
    packet.writeFloat(info.z);

    // Write orientation
    packet.writeFloat(info.orientation);

    // Write transport data if on transport.
    // 3.3.5a ordering: transport block appears before pitch/fall/jump.
    if (info.hasFlag(MovementFlags::ONTRANSPORT)) {
        // Write packed transport GUID
        packet.writePackedGuid(info.transportGuid);

        // Write transport local position
        packet.writeFloat(info.transportX);
        packet.writeFloat(info.transportY);
        packet.writeFloat(info.transportZ);
        packet.writeFloat(info.transportO);

        // Write transport time
        packet.writeUInt32(info.transportTime);

        // Transport seat is always present in ONTRANSPORT movement info.
        packet.writeUInt8(static_cast<uint8_t>(info.transportSeat));

        // Optional second transport time for interpolated movement.
        if (info.flags2 & 0x0400) { // MOVEMENTFLAG2_INTERPOLATED_MOVEMENT
            packet.writeUInt32(info.transportTime2);
        }
    }

    // Write pitch if swimming/flying
    if (info.hasFlag(MovementFlags::SWIMMING) || info.hasFlag(MovementFlags::FLYING)) {
        packet.writeFloat(info.pitch);
    }

    // Fall time is ALWAYS present in the packet (server reads it unconditionally).
    // Jump velocity/angle data is only present when FALLING flag is set.
    packet.writeUInt32(info.fallTime);

    if (info.hasFlag(MovementFlags::FALLING)) {
        packet.writeFloat(info.jumpVelocity);
        packet.writeFloat(info.jumpSinAngle);
        packet.writeFloat(info.jumpCosAngle);
        packet.writeFloat(info.jumpXYSpeed);
    }
}

network::Packet MovementPacket::build(Opcode opcode, const MovementInfo& info, uint64_t playerGuid) {
    network::Packet packet(wireOpcode(opcode));

    // Movement packet format (WoW 3.3.5a):
    // packed GUID + movement payload
    packet.writePackedGuid(playerGuid);
    writeMovementPayload(packet, info);

    // Detailed hex dump for debugging
    static int mvLog = 5;
    if (mvLog-- > 0) {
        const auto& raw = packet.getData();
        std::string hex;
        for (uint8_t byte : raw) {
            char b[4]; snprintf(b, sizeof(b), "%02x ", byte);
            hex += b;
        }
        LOG_DEBUG("MOVEPKT opcode=0x", std::hex, wireOpcode(opcode), std::dec,
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
        LOG_DEBUG("MOVEPKT hex: ", hex);
    }

    return packet;
}

bool UpdateObjectParser::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    // WoW 3.3.5a UPDATE_OBJECT movement block structure:
    // 1. UpdateFlags (1 byte, sometimes 2)
    // 2. Movement data depends on update flags

    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    if (rem() < 2) return false;

    // Update flags (3.3.5a uses 2 bytes for flags)
    uint16_t updateFlags = packet.readUInt16();
    block.updateFlags = updateFlags;

    LOG_DEBUG("  UpdateFlags: 0x", std::hex, updateFlags, std::dec);

    // Log transport-related flag combinations
    if (updateFlags & 0x0002) { // UPDATEFLAG_TRANSPORT
        static int transportFlagLogCount = 0;
        if (transportFlagLogCount < 12) {
            LOG_INFO("  Transport flags detected: 0x", std::hex, updateFlags, std::dec,
                     " (TRANSPORT=", !!(updateFlags & 0x0002),
                     ", POSITION=", !!(updateFlags & 0x0100),
                     ", ROTATION=", !!(updateFlags & 0x0200),
                     ", STATIONARY=", !!(updateFlags & 0x0040), ")");
            transportFlagLogCount++;
        } else {
            LOG_DEBUG("  Transport flags detected: 0x", std::hex, updateFlags, std::dec);
        }
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
        // Minimum: moveFlags(4)+moveFlags2(2)+time(4)+position(16)+fallTime(4)+speeds(36) = 66
        if (rem() < 66) return false;

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
            if (rem() < 1) return false;
            block.onTransport = true;
            block.transportGuid = packet.readPackedGuid();
            if (rem() < 21) return false; // 4 floats + uint32 + uint8
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t tTime =*/ packet.readUInt32();
            /*int8_t tSeat =*/ packet.readUInt8();

            LOG_DEBUG("  OnTransport: guid=0x", std::hex, block.transportGuid, std::dec,
                      " offset=(", block.transportX, ", ", block.transportY, ", ", block.transportZ, ")");

            if (moveFlags2 & 0x0400) { // MOVEMENTFLAG2_INTERPOLATED_MOVEMENT
                if (rem() < 4) return false;
                /*uint32_t tTime2 =*/ packet.readUInt32();
            }
        }

        // Swimming/flying pitch
        // WotLK 3.3.5a movement flags (wire format):
        //   SWIMMING          = 0x00200000
        //   CAN_FLY           = 0x01000000  (ability to fly - no pitch field)
        //   FLYING            = 0x02000000  (actively flying - has pitch field)
        //   SPLINE_ELEVATION  = 0x04000000  (smooth vertical spline offset)
        // MovementFlags2:
        //   MOVEMENTFLAG2_ALWAYS_ALLOW_PITCHING = 0x0020
        //
        // Pitch is present when SWIMMING or FLYING are set, or the always-allow flag is set.
        // Note: CAN_FLY (0x01000000) does NOT gate pitch; only FLYING (0x02000000) does.
        // (TBC uses 0x01000000 for FLYING - see TbcMoveFlags in packet_parsers_tbc.cpp.)
        if ((moveFlags & 0x00200000) /* SWIMMING */ ||
            (moveFlags & 0x02000000) /* FLYING */ ||
            (moveFlags2 & 0x0020)    /* MOVEMENTFLAG2_ALWAYS_ALLOW_PITCHING */) {
            if (rem() < 4) return false;
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time
        if (rem() < 4) return false;
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping
        if (moveFlags & 0x00001000) { // MOVEMENTFLAG_FALLING
            if (rem() < 16) return false;
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & 0x04000000) { // MOVEMENTFLAG_SPLINE_ELEVATION
            if (rem() < 4) return false;
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (9 values in WotLK: walk/run/runBack/swim/swimBack/flight/flightBack/turn/pitch)
        if (rem() < 36) return false;
        block.walkSpeed       = packet.readFloat();
        block.runSpeed        = packet.readFloat();
        block.runBackSpeed    = packet.readFloat();
        block.swimSpeed       = packet.readFloat();
        block.swimBackSpeed   = packet.readFloat();
        block.flightSpeed     = packet.readFloat();
        block.flightBackSpeed = packet.readFloat();
        block.turnRate        = packet.readFloat();
        block.pitchRate       = packet.readFloat();
        block.moveFlags = moveFlags;

        // Spline data
        if (moveFlags & 0x08000000) { // MOVEMENTFLAG_SPLINE_ENABLED
            SplineBlockData splineData;
            glm::vec3 entityPos(block.x, block.y, block.z);
            if (!parseWotlkMoveUpdateSpline(packet, splineData, entityPos)) {
                LOG_WARNING("WotLK spline parse failed for guid=0x", std::hex, block.guid, std::dec);
                return false;
            }
        }
    }
    else if (updateFlags & UPDATEFLAG_POSITION) {
        // Transport position update (UPDATEFLAG_POSITION = 0x0100)
        if (rem() < 1) return false;
        uint64_t transportGuid = packet.readPackedGuid();
        if (rem() < 32) return false; // 8 floats
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
            LOG_DEBUG("  TRANSPORT POSITION UPDATE: guid=0x", std::hex, transportGuid, std::dec,
                      " pos=(", block.x, ", ", block.y, ", ", block.z, "), o=", block.orientation,
                      " offset=(", block.transportX, ", ", block.transportY, ", ", block.transportZ, ")");
        }
    }
    else if (updateFlags & UPDATEFLAG_STATIONARY_POSITION) {
        if (rem() < 16) return false;
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  STATIONARY: (", block.x, ", ", block.y, ", ", block.z, "), o=", block.orientation);
    }

    // Target GUID (for units with target)
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        if (rem() < 1) return false;
        /*uint64_t targetGuid =*/ packet.readPackedGuid();
    }

    // Transport time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        if (rem() < 4) return false;
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    // Vehicle
    if (updateFlags & UPDATEFLAG_VEHICLE) {
        if (rem() < 8) return false;
        /*uint32_t vehicleId =*/ packet.readUInt32();
        /*float vehicleOrientation =*/ packet.readFloat();
    }

    // Rotation (GameObjects)
    if (updateFlags & UPDATEFLAG_ROTATION) {
        if (rem() < 8) return false;
        /*int64_t rotation =*/ packet.readUInt64();
    }

    // Low GUID
    if (updateFlags & UPDATEFLAG_LOWGUID) {
        if (rem() < 4) return false;
        /*uint32_t lowGuid =*/ packet.readUInt32();
    }

    // High GUID
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        if (rem() < 4) return false;
        /*uint32_t highGuid =*/ packet.readUInt32();
    }

    return true;
}

bool UpdateObjectParser::parseUpdateFields(network::Packet& packet, UpdateBlock& block) {
    size_t startPos = packet.getReadPos();

    if (!packet.hasData()) return false;

    // Read number of blocks (each block is 32 fields = 32 bits)
    uint8_t blockCount = packet.readUInt8();

    if (blockCount == 0) {
        return true; // No fields to update
    }

    // Sanity check: UNIT_END=148 needs 5 mask blocks, PLAYER_END=1472 needs 46.
    // VALUES updates don't carry objectType (defaults to 0), so allow up to 55
    // for any VALUES update (could be a PLAYER). Only flag CREATE_OBJECT blocks
    // with genuinely excessive block counts.
    bool isCreateBlock = (block.updateType == UpdateType::CREATE_OBJECT ||
                          block.updateType == UpdateType::CREATE_OBJECT2);
    uint8_t maxExpectedBlocks = isCreateBlock
        ? ((block.objectType == ObjectType::PLAYER) ? 55 : 10)
        : 55;  // VALUES: allow PLAYER-sized masks
    if (blockCount > maxExpectedBlocks) {
        LOG_WARNING("UpdateObjectParser: suspicious maskBlockCount=", static_cast<int>(blockCount),
                    " for objectType=", static_cast<int>(block.objectType),
                    " guid=0x", std::hex, block.guid, std::dec,
                    " updateFlags=0x", std::hex, block.updateFlags, std::dec,
                    " moveFlags=0x", std::hex, block.moveFlags, std::dec,
                    " readPos=", packet.getReadPos(), " size=", packet.getSize());
        // Movement data likely consumed wrong number of bytes, causing blockCount
        // to be read from a misaligned position. Bail out rather than reading garbage.
        if (isCreateBlock) return false;
    }

    uint32_t fieldsCapacity = blockCount * 32;
    LOG_DEBUG("  UPDATE MASK PARSE:");
    LOG_DEBUG("    maskBlockCount = ", static_cast<int>(blockCount));
    LOG_DEBUG("    fieldsCapacity (blocks * 32) = ", fieldsCapacity);

    // Read update mask into a reused scratch buffer to avoid per-block allocations.
    static thread_local std::vector<uint32_t> updateMask;
    updateMask.resize(blockCount);
    for (int i = 0; i < blockCount; ++i) {
        // Validate 4 bytes available before each block read
        if (!packet.hasRemaining(4)) {
            LOG_WARNING("UpdateObjectParser: truncated update mask at block ", i,
                        " type=", wowee::game::updateTypeName(block.updateType),
                        " objectType=", static_cast<int>(block.objectType),
                        " guid=0x", std::hex, block.guid, std::dec,
                        " readPos=", packet.getReadPos(),
                        " size=", packet.getSize(),
                        " maskBlockCount=", static_cast<int>(blockCount));
            return false;
        }
        updateMask[i] = packet.readUInt32();
    }

    // Find highest set bit
    uint16_t highestSetBit = 0;
    uint32_t valuesReadCount = 0;

    // Pre-reserve the field vector based on popcount of the mask so the
    // monotonic append loop below doesn't reallocate as it grows.
    uint32_t totalSetBits = 0;
    for (int blockIdx = 0; blockIdx < blockCount; ++blockIdx) {
#if defined(__GNUC__) || defined(__clang__)
        totalSetBits += static_cast<uint32_t>(__builtin_popcount(updateMask[blockIdx]));
#else
        uint32_t v = updateMask[blockIdx];
        while (v) { totalSetBits += (v & 1u); v >>= 1u; }
#endif
    }
    if (totalSetBits > 0) block.fields.reserve(totalSetBits);

    // Read only set bits in each mask block (faster than scanning all 32 bits).
    for (int blockIdx = 0; blockIdx < blockCount; ++blockIdx) {
        uint32_t mask = updateMask[blockIdx];
        while (mask != 0) {
            const uint16_t fieldIndex =
#if defined(__GNUC__) || defined(__clang__)
                static_cast<uint16_t>(blockIdx * 32 + __builtin_ctz(mask));
#else
                static_cast<uint16_t>(blockIdx * 32 + [] (uint32_t v) -> uint16_t {
                    uint16_t b = 0;
                    while ((v & 1u) == 0u) { v >>= 1u; ++b; }
                    return b;
                }(mask));
#endif
            if (fieldIndex > highestSetBit) {
                highestSetBit = fieldIndex;
            }
            // Validate 4 bytes available before reading field value
            if (!packet.hasRemaining(4)) {
                LOG_WARNING("UpdateObjectParser: truncated field value at field ", fieldIndex,
                            " type=", wowee::game::updateTypeName(block.updateType),
                            " objectType=", static_cast<int>(block.objectType),
                            " guid=0x", std::hex, block.guid, std::dec,
                            " readPos=", packet.getReadPos(),
                            " size=", packet.getSize(),
                            " maskBlockIndex=", blockIdx,
                            " maskBlock=0x", std::hex, updateMask[blockIdx], std::dec);
                return false;
            }
            uint32_t value = packet.readUInt32();
            // fieldIndex is monotonically increasing here - append directly to the
            // sorted flat vector (no tree-node allocation per field anymore).
            block.fields.append_sorted(fieldIndex, value);
            valuesReadCount++;

            LOG_DEBUG("    Field[", fieldIndex, "] = 0x", std::hex, value, std::dec);
            mask &= (mask - 1u);
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
    if (!packet.hasData()) return false;

    // Read update type
    uint8_t updateTypeVal = packet.readUInt8();
    block.updateType = static_cast<UpdateType>(updateTypeVal);

    LOG_DEBUG("Update block: type=", static_cast<int>(updateTypeVal));

    switch (block.updateType) {
        case UpdateType::VALUES: {
            // Partial update - changed fields only
            if (!packet.hasData()) return false;
            block.guid = packet.readPackedGuid();
            LOG_DEBUG("  VALUES update for GUID: 0x", std::hex, block.guid, std::dec);

            return parseUpdateFields(packet, block);
        }

        case UpdateType::MOVEMENT: {
            // Movement update - WotLK 3.3.5a uses PackedGuid (NOT full uint64)
            if (!packet.hasData()) return false;
            block.guid = packet.readPackedGuid();
            LOG_DEBUG("  MOVEMENT update for GUID: 0x", std::hex, block.guid, std::dec);

            return parseMovementBlock(packet, block);
        }

        case UpdateType::CREATE_OBJECT:
        case UpdateType::CREATE_OBJECT2: {
            // Create new object with full data
            if (!packet.hasData()) return false;
            block.guid = packet.readPackedGuid();
            LOG_DEBUG("  CREATE_OBJECT for GUID: 0x", std::hex, block.guid, std::dec);

            // Read object type
            if (!packet.hasData()) return false;
            uint8_t objectTypeVal = packet.readUInt8();
            block.objectType = static_cast<ObjectType>(objectTypeVal);
            LOG_DEBUG("  Object type: ", static_cast<int>(objectTypeVal));

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
            LOG_WARNING("Unknown update type: ", static_cast<int>(updateTypeVal));
            return false;
    }
}

bool UpdateObjectParser::parse(network::Packet& packet, UpdateObjectData& data) {
    // Keep worst-case packet parsing bounded. Extremely large counts are typically
    // malformed/desynced and can stall a frame long enough to trigger disconnects.
    constexpr uint32_t kMaxReasonableUpdateBlocks = 1024;
    constexpr uint32_t kMaxReasonableOutOfRangeGuids = 4096;

    // Read block count
    data.blockCount = packet.readUInt32();
    if (data.blockCount > kMaxReasonableUpdateBlocks) {
        LOG_ERROR("SMSG_UPDATE_OBJECT rejected: unreasonable blockCount=", data.blockCount,
                  " packetSize=", packet.getSize());
        return false;
    }

    LOG_DEBUG("SMSG_UPDATE_OBJECT:");
    LOG_DEBUG("  objectCount = ", data.blockCount);
    LOG_DEBUG("  packetSize = ", packet.getSize());

    uint32_t remainingBlockCount = data.blockCount;

    // Check for out-of-range objects first
    if (packet.hasRemaining(1)) {
        uint8_t firstByte = packet.readUInt8();

        if (firstByte == static_cast<uint8_t>(UpdateType::OUT_OF_RANGE_OBJECTS)) {
            if (remainingBlockCount == 0) {
                LOG_ERROR("SMSG_UPDATE_OBJECT rejected: OUT_OF_RANGE_OBJECTS with zero blockCount");
                return false;
            }
            --remainingBlockCount;
            // Read out-of-range GUID count
            uint32_t count = packet.readUInt32();
            if (count > kMaxReasonableOutOfRangeGuids) {
                LOG_ERROR("SMSG_UPDATE_OBJECT rejected: unreasonable outOfRange count=", count,
                          " packetSize=", packet.getSize());
                return false;
            }

            data.outOfRangeGuids.reserve(count);
            for (uint32_t i = 0; i < count; ++i) {
                uint64_t guid = packet.readPackedGuid();
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
    data.blockCount = remainingBlockCount;
    data.blocks.reserve(data.blockCount);

    // Track last block state for desync diagnostics
    uint8_t prevUpdateType = 0;
    uint8_t prevObjectType = 0;
    uint16_t prevUpdateFlags = 0;
    uint32_t prevMoveFlags = 0;
    uint64_t prevGuid = 0;
    size_t prevReadPos = packet.getReadPos();

    for (uint32_t i = 0; i < data.blockCount; ++i) {
        LOG_DEBUG("Parsing block ", i + 1, " / ", data.blockCount);

        size_t blockStartPos = packet.getReadPos();
        UpdateBlock block;
        if (!parseUpdateBlock(packet, block)) {
            static int parseBlockErrors = 0;
            const uint32_t lostBlocks = data.blockCount - i;
            if (++parseBlockErrors <= 10) {
                LOG_ERROR("Failed to parse update block ", i + 1, " of ", data.blockCount,
                          " (", i, " blocks parsed, ", lostBlocks, " blocks LOST",
                          ", remaining=", packet.getRemainingSize(), " bytes)");
                LOG_ERROR("  blockStartPos=", blockStartPos, " packetSize=", packet.getSize());
                // The failing block is partially parsed: parseMovementBlock stores its
                // updateFlags/moveFlags into `block` before it can fail, and the object
                // type is read first. Log them here so the FIRST block failing (i==0, no
                // prevBlock context) still reveals which movement layout desynced - the
                // moveFlags identify which optional field (transport/spline/falling/pitch)
                // consumed the wrong byte count and misaligned the update-field mask.
                LOG_ERROR("  failedBlock: updateType=", static_cast<int>(block.updateType),
                          " objType=", static_cast<int>(block.objectType),
                          " updateFlags=0x", std::hex, block.updateFlags,
                          " moveFlags=0x", block.moveFlags,
                          " guid=0x", block.guid, std::dec,
                          " consumedBeforeFail=", packet.getSize() - packet.getRemainingSize() - blockStartPos);
                if (i > 0) {
                    LOG_ERROR("  prevBlock: type=", static_cast<int>(prevUpdateType),
                              " objType=", static_cast<int>(prevObjectType),
                              " updateFlags=0x", std::hex, prevUpdateFlags,
                              " moveFlags=0x", prevMoveFlags,
                              " guid=0x", prevGuid, std::dec,
                              " startPos=", prevReadPos,
                              " consumed=", blockStartPos - prevReadPos, " bytes");
                }
                // Peek at the failing byte(s) for format diagnosis
                packet.setReadPos(blockStartPos);
                uint8_t peekBytes[8] = {};
                size_t peekCount = std::min<size_t>(8, packet.getRemainingSize());
                for (size_t p = 0; p < peekCount; ++p)
                    peekBytes[p] = packet.readUInt8();
                LOG_ERROR("  failBytes: ",
                          std::hex, static_cast<int>(peekBytes[0]), " ",
                          static_cast<int>(peekBytes[1]), " ",
                          static_cast<int>(peekBytes[2]), " ",
                          static_cast<int>(peekBytes[3]), " ",
                          static_cast<int>(peekBytes[4]), " ",
                          static_cast<int>(peekBytes[5]), " ",
                          static_cast<int>(peekBytes[6]), " ",
                          static_cast<int>(peekBytes[7]), std::dec);
                if (parseBlockErrors == 10)
                    LOG_ERROR("(suppressing further update block parse errors)");
            }
            // Cannot reliably re-sync to the next block after a parse failure,
            // but still return true so the blocks already parsed are processed.
            break;
        }

        prevUpdateType = static_cast<uint8_t>(block.updateType);
        prevObjectType = static_cast<uint8_t>(block.objectType);
        prevUpdateFlags = block.updateFlags;
        prevMoveFlags = block.moveFlags;
        prevGuid = block.guid;
        prevReadPos = blockStartPos;
        data.blocks.emplace_back(std::move(block));
    }


    return true;
}

bool DestroyObjectParser::parse(network::Packet& packet, DestroyObjectData& data) {
    // SMSG_DESTROY_OBJECT format:
    // uint64 guid
    // uint8 isDeath (0 = despawn, 1 = death) - WotLK only; vanilla/TBC omit this

    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_DESTROY_OBJECT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    data.guid = packet.readUInt64();
    // WotLK adds isDeath byte; vanilla/TBC packets are exactly 8 bytes
    if (packet.hasData()) {
        data.isDeath = (packet.readUInt8() != 0);
    } else {
        data.isDeath = false;
    }

    LOG_DEBUG("Parsed SMSG_DESTROY_OBJECT:");
    LOG_DEBUG("  GUID: 0x", std::hex, data.guid, std::dec);
    LOG_DEBUG("  Is death: ", data.isDeath ? "yes" : "no");

    return true;
}

} // namespace game
} // namespace wowee
