#include "game/world_packets.hpp"
#include "game/packed_time.hpp"
#include "game/packet_parsers.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "game/game_utils.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>

namespace wowee {
namespace game {

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

bool decodeAddonChatPayload(const MessageChatData& data,
                            std::string& prefix,
                            std::string& payload) {
    prefix.clear();
    payload.clear();

    const size_t separator = data.message.find('\t');
    const bool hasEnvelope = separator != std::string::npos &&
                             separator > 0 && separator <= 16 &&
                             data.message.substr(0, separator).find(' ') == std::string::npos;
    const bool supportsLegacyEnvelope =
        data.type != ChatType::SAY && data.type != ChatType::YELL &&
        data.type != ChatType::EMOTE && data.type != ChatType::TEXT_EMOTE &&
        data.type != ChatType::MONSTER_SAY && data.type != ChatType::MONSTER_YELL;
    if (data.language != ChatLanguage::ADDON &&
        (!supportsLegacyEnvelope || !hasEnvelope)) return false;

    if (hasEnvelope) {
        prefix = data.message.substr(0, separator);
        payload = data.message.substr(separator + 1);
    }
    return true;
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
        case ChatType::MONSTER_EMOTE:
        case ChatType::MONSTER_WHISPER:
        case ChatType::MONSTER_PARTY:
        case ChatType::RAID_BOSS_EMOTE:
        case ChatType::RAID_BOSS_WHISPER: {
            // Read sender name (SizedCString: uint32 len including null + chars)
            uint32_t nameLen = packet.readUInt32();
            if (nameLen > packet.getRemainingSize()) return false;
            if (nameLen > 0 && nameLen < 256) {
                data.senderName.resize(nameLen);
                for (uint32_t i = 0; i < nameLen; ++i) {
                    data.senderName[i] = static_cast<char>(packet.readUInt8());
                }
                // Strip trailing null (server includes it in nameLen)
                if (!data.senderName.empty() && data.senderName.back() == '\0') {
                    data.senderName.pop_back();
                }
            }
            // Read receiver GUID (NamedGuid: guid + optional name for non-player targets)
            data.receiverGuid = packet.readUInt64();
            if (data.receiverGuid != 0) {
                // WoW GUID type encoding: bits 48-63 identify entity type.
                // Players have highGuid=0x0000. Pets use 0xF040 (active pet) or
                // 0xF014 (creature treated as pet). Mask 0xF0FF isolates the type
                // nibbles while ignoring the server-specific middle bits.
                constexpr uint16_t kGuidTypeMask   = 0xF0FF;
                constexpr uint16_t kGuidTypePet     = 0xF040;
                constexpr uint16_t kGuidTypeVehicle = 0xF014;
                uint16_t highGuid = static_cast<uint16_t>(data.receiverGuid >> 48);
                bool isPlayer = (highGuid == 0x0000);
                bool isPet = ((highGuid & kGuidTypeMask) == kGuidTypePet) ||
                             ((highGuid & kGuidTypeMask) == kGuidTypeVehicle);
                if (!isPlayer && !isPet) {
                    // Read receiver name (SizedCString)
                    uint32_t recvNameLen = packet.readUInt32();
                    if (recvNameLen > 0 && recvNameLen < 256) {
                        packet.setReadPos(packet.getReadPos() + recvNameLen);
                    }
                }
            }
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

        case ChatType::WHISPER:
        case ChatType::WHISPER_INFORM: {
            // TBC and WotLK both place the receiver GUID directly before
            // messageLen. Names are resolved separately from player GUIDs.
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::BG_SYSTEM_NEUTRAL:
        case ChatType::BG_SYSTEM_ALLIANCE:
        case ChatType::BG_SYSTEM_HORDE:
            // BG/Arena system messages — no sender GUID or name field, just message.
            // Reclassify as SYSTEM for consistent display.
            data.type = ChatType::SYSTEM;
            break;

        default:
            // SAY, GUILD, PARTY, YELL, WHISPER, WHISPER_INFORM, RAID, etc.
            // All have receiverGuid (typically senderGuid repeated)
            data.receiverGuid = packet.readUInt64();
            break;
    }

    // Read message length
    uint32_t messageLen = packet.readUInt32();
    if (messageLen > packet.getRemainingSize()) return false;

    // Read message
    if (messageLen > 0 && messageLen < 8192) {
        data.message.resize(messageLen);
        for (uint32_t i = 0; i < messageLen; ++i) {
            data.message[i] = static_cast<char>(packet.readUInt8());
        }
        // Strip trailing null terminator (servers include it in messageLen)
        if (!data.message.empty() && data.message.back() == '\0') {
            data.message.pop_back();
        }
    }

    // Read chat tag
    data.chatTag = packet.readUInt8();

    // WotLK appends the achievement ID after chatTag. The message itself is a
    // client-format template ("%s has earned the achievement $a"), so dropping
    // this field leaves the client unable to expand $a.
    if ((data.type == ChatType::ACHIEVEMENT ||
         data.type == ChatType::GUILD_ACHIEVEMENT) && packet.hasRemaining(4)) {
        data.achievementId = packet.readUInt32();
    }

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
    LOG_DEBUG("  Chat tag: 0x", std::hex, static_cast<int>(data.chatTag), std::dec);

    return true;
}

const char* getChatTypeString(ChatType type) {
    switch (type) {
        case ChatType::SYSTEM: return "SYSTEM";
        case ChatType::SAY: return "SAY";
        case ChatType::PARTY: return "PARTY";
        case ChatType::RAID: return "RAID";
        case ChatType::GUILD: return "GUILD";
        case ChatType::OFFICER: return "OFFICER";
        case ChatType::YELL: return "YELL";
        case ChatType::WHISPER: return "WHISPER";
        case ChatType::WHISPER_FOREIGN: return "WHISPER_FOREIGN";
        case ChatType::WHISPER_INFORM: return "WHISPER_INFORM";
        case ChatType::EMOTE: return "EMOTE";
        case ChatType::TEXT_EMOTE: return "TEXT_EMOTE";
        case ChatType::MONSTER_SAY: return "MONSTER_SAY";
        case ChatType::MONSTER_PARTY: return "MONSTER_PARTY";
        case ChatType::MONSTER_YELL: return "MONSTER_YELL";
        case ChatType::MONSTER_WHISPER: return "MONSTER_WHISPER";
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
        case ChatType::BG_SYSTEM_NEUTRAL: return "BG_SYSTEM_NEUTRAL";
        case ChatType::BG_SYSTEM_ALLIANCE: return "BG_SYSTEM_ALLIANCE";
        case ChatType::BG_SYSTEM_HORDE: return "BG_SYSTEM_HORDE";
        case ChatType::RAID_LEADER: return "RAID_LEADER";
        case ChatType::RAID_WARNING: return "RAID_WARNING";
        case ChatType::RAID_BOSS_EMOTE: return "RAID_BOSS_EMOTE";
        case ChatType::RAID_BOSS_WHISPER: return "RAID_BOSS_WHISPER";
        case ChatType::BATTLEGROUND: return "BATTLEGROUND";
        case ChatType::BATTLEGROUND_LEADER: return "BATTLEGROUND_LEADER";
        case ChatType::MONEY: return "MONEY";
        case ChatType::OPENING: return "OPENING";
        case ChatType::TRADESKILLS: return "TRADESKILLS";
        case ChatType::PET_INFO: return "PET_INFO";
        case ChatType::COMBAT_MISC_INFO: return "COMBAT_MISC_INFO";
        case ChatType::COMBAT_XP_GAIN: return "COMBAT_XP_GAIN";
        case ChatType::COMBAT_HONOR_GAIN: return "COMBAT_HONOR_GAIN";
        case ChatType::COMBAT_FACTION_CHANGE: return "COMBAT_FACTION_CHANGE";
        case ChatType::FILTERED: return "FILTERED";
        case ChatType::RESTRICTED: return "RESTRICTED";
        case ChatType::ACHIEVEMENT: return "ACHIEVEMENT";
        case ChatType::GUILD_ACHIEVEMENT: return "GUILD_ACHIEVEMENT";
        case ChatType::PARTY_LEADER: return "PARTY_LEADER";
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

bool TextEmoteParser::parse(network::Packet& packet, TextEmoteData& data, bool legacyFormat) {
    size_t bytesLeft = packet.getRemainingSize();
    if (bytesLeft < 20) {
        LOG_WARNING("SMSG_TEXT_EMOTE too short: ", bytesLeft, " bytes");
        return false;
    }

    if (legacyFormat) {
        // Classic 1.12: textEmoteId(u32) + emoteNum(u32) + senderGuid(u64)
        data.textEmoteId = packet.readUInt32();
        data.emoteNum    = packet.readUInt32();
        data.senderGuid  = packet.readUInt64();
    } else {
        // TBC/WotLK: senderGuid(u64) + textEmoteId(u32) + emoteNum(u32)
        data.senderGuid  = packet.readUInt64();
        data.textEmoteId = packet.readUInt32();
        data.emoteNum    = packet.readUInt32();
    }

    uint32_t nameLen = packet.readUInt32();
    if (nameLen > 0 && nameLen <= 256) {
        data.targetName = packet.readString();
    } else if (nameLen > 0) {
        // Implausible name length — misaligned read
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
    size_t bytesLeft = packet.getRemainingSize();
    if (bytesLeft < 2) {
        LOG_WARNING("SMSG_CHANNEL_NOTIFY too short");
        return false;
    }
    data.notifyType = static_cast<ChannelNotifyType>(packet.readUInt8());
    data.channelName = packet.readString();
    // Some notification types have additional fields (guid, etc.)
    bytesLeft = packet.getRemainingSize();
    if (bytesLeft >= 8) {
        data.senderGuid = packet.readUInt64();
    }
    return true;
}

// ============================================================
// Foundation — Targeting, Name Queries
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
    if (isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle")) {
        packet.writeUInt64(targetGuid);
    } else {
        packet.writePackedGuid(targetGuid);
    }
    LOG_DEBUG("Built CMSG_INSPECT: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet QueryInspectAchievementsPacket::build(uint64_t targetGuid) {
    // CMSG_QUERY_INSPECT_ACHIEVEMENTS: PackedGuid targetGuid
    network::Packet packet(wireOpcode(Opcode::CMSG_QUERY_INSPECT_ACHIEVEMENTS));
    packet.writePackedGuid(targetGuid);
    LOG_DEBUG("Built CMSG_QUERY_INSPECT_ACHIEVEMENTS: target=0x", std::hex, targetGuid, std::dec);
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
    // Validate minimum packet size: serverTime(4) + timeOffset(4)
    if (!packet.hasRemaining(8)) {
        LOG_WARNING("SMSG_QUERY_TIME_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.serverTime = packet.readUInt32();
    data.timeOffset = packet.readUInt32();
    LOG_DEBUG("Parsed SMSG_QUERY_TIME_RESPONSE: time=", data.serverTime, " offset=", data.timeOffset);
    return true;
}

network::Packet RequestPlayedTimePacket::build(bool sendToChat) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PLAYED_TIME));
    packet.writeUInt8(sendToChat ? 1 : 0);
    LOG_DEBUG("Built CMSG_PLAYED_TIME: sendToChat=", sendToChat);
    return packet;
}

bool PlayedTimeParser::parse(network::Packet& packet, PlayedTimeData& data) {
    // Classic/Turtle may omit the trailing trigger-message byte and send only
    // totalTime(4) + levelTime(4). Later expansions append triggerMsg(1).
    if (!packet.hasRemaining(8)) {
        LOG_WARNING("SMSG_PLAYED_TIME: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.totalTimePlayed = packet.readUInt32();
    data.levelTimePlayed = packet.readUInt32();
    data.triggerMessage = (packet.hasRemaining(1)) && (packet.readUInt8() != 0);
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
    packet.writeUInt32(zones);    // Number of zone IDs (0 = no zone filter)
    // Zone ID array would go here if zones > 0
    packet.writeUInt32(0);        // stringCount (number of search strings)
    // String array would go here if stringCount > 0
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
    // Validate minimum packet size: status(1) + guid(8)
    if (!packet.hasRemaining(9)) {
        LOG_WARNING("SMSG_FRIEND_STATUS: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.status = packet.readUInt8();
    data.guid = packet.readUInt64();
    if (data.status == 1) {  // Online
        // Conditional: note (string) + chatFlag (1)
        if (packet.hasData()) {
            data.note = packet.readString();
            if (packet.hasRemaining(1)) {
                data.chatFlag = packet.readUInt8();
            }
        }
    }
    LOG_DEBUG("Parsed SMSG_FRIEND_STATUS: status=", static_cast<int>(data.status), " guid=0x", std::hex, data.guid, std::dec);
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

network::Packet ComplainPacket::build(uint64_t targetGuid, const std::string& reason) {
    network::Packet packet(wireOpcode(Opcode::CMSG_COMPLAIN));
    packet.writeUInt8(1);               // complaintType: 1 = spam
    packet.writeUInt64(targetGuid);
    packet.writeUInt32(0);              // unk
    packet.writeUInt32(0);              // messageType
    packet.writeUInt32(0);              // channelId
    packet.writeUInt32(static_cast<uint32_t>(time(nullptr))); // timestamp
    packet.writeString(reason);
    LOG_DEBUG("Built CMSG_COMPLAIN: target=0x", std::hex, targetGuid, std::dec, " reason=", reason);
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
    // Validate minimum packet size: result(4) + instant(1)
    if (!packet.hasRemaining(5)) {
        LOG_WARNING("SMSG_LOGOUT_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.result = packet.readUInt32();
    data.instant = packet.readUInt8();
    LOG_DEBUG("Parsed SMSG_LOGOUT_RESPONSE: result=", data.result, " instant=", static_cast<int>(data.instant));
    return true;
}

// ============================================================
// Stand State
// ============================================================

network::Packet StandStateChangePacket::build(uint8_t state) {
    network::Packet packet(wireOpcode(Opcode::CMSG_STANDSTATECHANGE));
    packet.writeUInt32(state);
    LOG_DEBUG("Built CMSG_STANDSTATECHANGE: state=", static_cast<int>(state));
    return packet;
}

// ============================================================
// Action Bar
// ============================================================

network::Packet SetActionButtonPacket::build(uint8_t button, uint8_t type, uint32_t id, bool isClassic) {
    // Classic/Turtle (1.12): uint8 button + uint16 id + uint8 type + uint8 misc(0)
    //   type encoding: 0=spell, 1=item, 64=macro
    // TBC/WotLK:       uint8 button + uint32 packed (type<<24 | id)
    //   type encoding: 0x00=spell, 0x80=item, 0x40=macro
    //   packed=0 means clear the slot
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_ACTION_BUTTON));
    packet.writeUInt8(button);
    if (isClassic) {
        // Classic: 16-bit id, 8-bit type code, 8-bit misc
        // Map ActionBarSlot::Type (0=EMPTY,1=SPELL,2=ITEM,3=MACRO) → classic type byte
        uint8_t classicType = 0; // 0 = spell
        if (type == 2 /* ITEM */)  classicType = 1;
        if (type == 3 /* MACRO */) classicType = 64;
        packet.writeUInt16(static_cast<uint16_t>(id));
        packet.writeUInt8(classicType);
        packet.writeUInt8(0); // misc
        LOG_DEBUG("Built CMSG_SET_ACTION_BUTTON (Classic): button=", static_cast<int>(button),
                  " id=", id, " type=", static_cast<int>(classicType));
    } else {
        // TBC/WotLK: type in bits 24–31, id in bits 0–23; packed=0 clears slot
        uint8_t packedType = 0x00; // spell
        if (type == 2 /* ITEM */)  packedType = 0x80;
        if (type == 3 /* MACRO */) packedType = 0x40;
        uint32_t packed = (id == 0) ? 0 : (static_cast<uint32_t>(packedType) << 24) | (id & 0x00FFFFFF);
        packet.writeUInt32(packed);
        LOG_DEBUG("Built CMSG_SET_ACTION_BUTTON (TBC/WotLK): button=", static_cast<int>(button),
                  " packed=0x", std::hex, packed, std::dec);
    }
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
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_ROSTER));
    LOG_DEBUG("Built CMSG_GUILD_ROSTER");
    return packet;
}

network::Packet GuildMotdPacket::build(const std::string& motd) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_MOTD));
    packet.writeString(motd);
    LOG_DEBUG("Built CMSG_GUILD_MOTD: ", motd);
    return packet;
}

network::Packet GuildPromotePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_PROMOTE));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_PROMOTE: ", playerName);
    return packet;
}

network::Packet GuildDemotePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DEMOTE));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_DEMOTE: ", playerName);
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

network::Packet GuildDisbandPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DISBAND));
    LOG_DEBUG("Built CMSG_GUILD_DISBAND");
    return packet;
}

network::Packet GuildLeaderPacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_LEADER));
    packet.writeString(playerName);
    LOG_DEBUG("Built CMSG_GUILD_LEADER: ", playerName);
    return packet;
}

network::Packet GuildSetPublicNotePacket::build(const std::string& playerName, const std::string& note) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_SET_PUBLIC_NOTE));
    packet.writeString(playerName);
    packet.writeString(note);
    LOG_DEBUG("Built CMSG_GUILD_SET_PUBLIC_NOTE: ", playerName, " -> ", note);
    return packet;
}

network::Packet GuildSetOfficerNotePacket::build(const std::string& playerName, const std::string& note) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_SET_OFFICER_NOTE));
    packet.writeString(playerName);
    packet.writeString(note);
    LOG_DEBUG("Built CMSG_GUILD_SET_OFFICER_NOTE: ", playerName, " -> ", note);
    return packet;
}

network::Packet GuildAcceptPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_ACCEPT));
    LOG_DEBUG("Built CMSG_GUILD_ACCEPT");
    return packet;
}

network::Packet GuildDeclineInvitationPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DECLINE));
    LOG_DEBUG("Built CMSG_GUILD_DECLINE");
    return packet;
}

network::Packet GuildCreatePacket::build(const std::string& guildName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_CREATE));
    packet.writeString(guildName);
    LOG_DEBUG("Built CMSG_GUILD_CREATE: ", guildName);
    return packet;
}

network::Packet GuildAddRankPacket::build(const std::string& rankName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_ADD_RANK));
    packet.writeString(rankName);
    LOG_DEBUG("Built CMSG_GUILD_ADD_RANK: ", rankName);
    return packet;
}

network::Packet GuildDelRankPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_DEL_RANK));
    LOG_DEBUG("Built CMSG_GUILD_DEL_RANK");
    return packet;
}

network::Packet PetitionShowlistPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PETITION_SHOWLIST));
    packet.writeUInt64(npcGuid);
    LOG_DEBUG("Built CMSG_PETITION_SHOWLIST: guid=", npcGuid);
    return packet;
}

network::Packet PetitionBuyPacket::build(uint64_t npcGuid, const std::string& guildName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PETITION_BUY));
    packet.writeUInt64(npcGuid);          // NPC GUID
    packet.writeUInt32(0);                // unk
    packet.writeUInt64(0);                // unk
    packet.writeString(guildName);        // guild name
    packet.writeUInt32(0);                // body text (empty)
    packet.writeUInt32(0);                // min sigs
    packet.writeUInt32(0);                // max sigs
    packet.writeUInt32(0);                // unk
    packet.writeUInt32(0);                // unk
    packet.writeUInt32(0);                // unk
    packet.writeUInt32(0);                // unk
    packet.writeUInt16(0);                // unk
    packet.writeUInt32(0);                // unk
    packet.writeUInt32(0);                // unk index
    packet.writeUInt32(0);                // unk
    LOG_DEBUG("Built CMSG_PETITION_BUY: npcGuid=", npcGuid, " name=", guildName);
    return packet;
}

bool PetitionShowlistParser::parse(network::Packet& packet, PetitionShowlistData& data) {
    if (packet.getSize() < 12) {
        LOG_ERROR("SMSG_PETITION_SHOWLIST too small: ", packet.getSize());
        return false;
    }
    data.npcGuid = packet.readUInt64();
    uint32_t count = packet.readUInt32();
    if (count > 0) {
        data.itemId = packet.readUInt32();
        data.displayId = packet.readUInt32();
        data.cost = packet.readUInt32();
        // Skip unused fields if present
        if (packet.hasRemaining(8)) {
            data.charterType = packet.readUInt32();
            data.requiredSigs = packet.readUInt32();
        }
    }
    LOG_INFO("Parsed SMSG_PETITION_SHOWLIST: npcGuid=", data.npcGuid, " cost=", data.cost);
    return true;
}

bool TurnInPetitionResultsParser::parse(network::Packet& packet, uint32_t& result) {
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_TURN_IN_PETITION_RESULTS too small: ", packet.getSize());
        return false;
    }
    result = packet.readUInt32();
    LOG_INFO("Parsed SMSG_TURN_IN_PETITION_RESULTS: result=", result);
    return true;
}

bool GuildQueryResponseParser::parse(network::Packet& packet, GuildQueryResponseData& data) {
    if (packet.getSize() < 8) {
        LOG_ERROR("SMSG_GUILD_QUERY_RESPONSE too small: ", packet.getSize());
        return false;
    }
    data.guildId = packet.readUInt32();

    // Validate before reading guild name
    if (!packet.hasData()) {
        LOG_WARNING("GuildQueryResponseParser: truncated before guild name");
        data.guildName.clear();
        return true;
    }
    data.guildName = packet.readString();

    // Read 10 rank names with validation
    for (int i = 0; i < 10; ++i) {
        if (!packet.hasData()) {
            LOG_WARNING("GuildQueryResponseParser: truncated at rank name ", i);
            data.rankNames[i].clear();
        } else {
            data.rankNames[i] = packet.readString();
        }
    }

    // Validate before reading emblem fields (5 uint32s = 20 bytes)
    if (!packet.hasRemaining(20)) {
        LOG_WARNING("GuildQueryResponseParser: truncated before emblem data");
        data.emblemStyle = 0;
        data.emblemColor = 0;
        data.borderStyle = 0;
        data.borderColor = 0;
        data.backgroundColor = 0;
        return true;
    }

    data.emblemStyle = packet.readUInt32();
    data.emblemColor = packet.readUInt32();
    data.borderStyle = packet.readUInt32();
    data.borderColor = packet.readUInt32();
    data.backgroundColor = packet.readUInt32();

    if (packet.hasRemaining(4)) {
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
    // One packed uint32, not three. Guild::SendInfo writes the creation date
    // with AppendPackedTime, so reading a day, a month and a year took twelve
    // bytes where the server sent four — the date was nonsense and the two
    // counts after it were read from the wrong offset as well.
    const WowDate created = unpackWowPackedTime(packet.readUInt32());
    data.creationDay = static_cast<uint32_t>(created.day);
    data.creationMonth = static_cast<uint32_t>(created.month);
    data.creationYear = static_cast<uint32_t>(created.fullYear());
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

    // Cap members and ranks to prevent unbounded memory allocation
    const uint32_t MAX_GUILD_MEMBERS = 1000;
    if (numMembers > MAX_GUILD_MEMBERS) {
        LOG_WARNING("GuildRosterParser: numMembers capped (requested=", numMembers, ")");
        numMembers = MAX_GUILD_MEMBERS;
    }

    data.motd = packet.readString();
    data.guildInfo = packet.readString();

    if (!packet.hasRemaining(4)) {
        LOG_WARNING("GuildRosterParser: truncated before rankCount");
        data.ranks.clear();
        data.members.clear();
        return true;
    }

    uint32_t rankCount = packet.readUInt32();

    // Cap rank count to prevent unbounded allocation
    const uint32_t MAX_GUILD_RANKS = 20;
    if (rankCount > MAX_GUILD_RANKS) {
        LOG_WARNING("GuildRosterParser: rankCount capped (requested=", rankCount, ")");
        rankCount = MAX_GUILD_RANKS;
    }

    data.ranks.resize(rankCount);
    for (uint32_t i = 0; i < rankCount; ++i) {
        // Validate 4 bytes before each rank rights read
        if (!packet.hasRemaining(4)) {
            LOG_WARNING("GuildRosterParser: truncated rank at index ", i);
            break;
        }
        data.ranks[i].rights = packet.readUInt32();
        if (!packet.hasRemaining(4)) {
            data.ranks[i].goldLimit = 0;
        } else {
            data.ranks[i].goldLimit = packet.readUInt32();
        }
        // 6 bank tab flags + 6 bank tab items per day
        for (int t = 0; t < 6; ++t) {
            if (!packet.hasRemaining(8)) break;
            packet.readUInt32(); // tabFlags
            packet.readUInt32(); // tabItemsPerDay
        }
    }

    data.members.resize(numMembers);
    for (uint32_t i = 0; i < numMembers; ++i) {
        // Validate minimum bytes before reading member (guid+online+name at minimum is 9+ bytes)
        if (!packet.hasRemaining(9)) {
            LOG_WARNING("GuildRosterParser: truncated member at index ", i);
            break;
        }
        auto& m = data.members[i];
        m.guid = packet.readUInt64();
        m.online = (packet.readUInt8() != 0);

        // Validate before reading name string
        if (!packet.hasData()) {
            m.name.clear();
        } else {
            m.name = packet.readString();
        }

        // Validate before reading rank/level/class/gender/zone
        if (!packet.hasRemaining(1)) {
            m.rankIndex = 0;
            m.level = 1;
            m.classId = 0;
            m.gender = 0;
            m.zoneId = 0;
        } else {
            m.rankIndex = packet.readUInt32();
            if (!packet.hasRemaining(3)) {
                m.level = 1;
                m.classId = 0;
                m.gender = 0;
            } else {
                m.level = packet.readUInt8();
                m.classId = packet.readUInt8();
                m.gender = packet.readUInt8();
            }
            if (!packet.hasRemaining(4)) {
                m.zoneId = 0;
            } else {
                m.zoneId = packet.readUInt32();
            }
        }

        // Online status affects next fields
        if (!m.online) {
            if (!packet.hasRemaining(4)) {
                m.lastOnline = 0.0f;
            } else {
                m.lastOnline = packet.readFloat();
            }
        }

        // Read notes
        if (!packet.hasData()) {
            m.publicNote.clear();
            m.officerNote.clear();
        } else {
            m.publicNote = packet.readString();
            if (!packet.hasData()) {
                m.officerNote.clear();
            } else {
                m.officerNote = packet.readString();
            }
        }
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
    if (packet.hasRemaining(8)) {
        data.guid = packet.readUInt64();
    }
    LOG_INFO("Parsed SMSG_GUILD_EVENT: type=", static_cast<int>(data.eventType), " strings=", static_cast<int>(data.numStrings));
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

network::Packet DuelAcceptPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_DUEL_ACCEPTED));
    LOG_DEBUG("Built CMSG_DUEL_ACCEPTED");
    return packet;
}

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

network::Packet GroupRaidConvertPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_RAID_CONVERT));
    LOG_DEBUG("Built CMSG_GROUP_RAID_CONVERT");
    return packet;
}

network::Packet SetLootMethodPacket::build(uint32_t method, uint32_t threshold, uint64_t masterLooterGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_METHOD));
    packet.writeUInt32(method);
    packet.writeUInt32(threshold);
    packet.writeUInt64(masterLooterGuid);
    LOG_DEBUG("Built CMSG_LOOT_METHOD: method=", method, " threshold=", threshold,
              " masterLooter=0x", std::hex, masterLooterGuid, std::dec);
    return packet;
}

network::Packet RaidTargetUpdatePacket::build(uint8_t targetIndex, uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::MSG_RAID_TARGET_UPDATE));
    packet.writeUInt8(targetIndex);
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built MSG_RAID_TARGET_UPDATE, index: ", static_cast<uint32_t>(targetIndex), ", guid: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

bool RaidTargetUpdateParser::parse(network::Packet& packet, RaidTargetUpdateData& data) {
    data = RaidTargetUpdateData{};
    if (!packet.hasRemaining(1)) return false;

    const uint8_t type = packet.readUInt8();
    if (type == 1) {
        // Full list — variable length, only the icons that are set.
        data.fullList = true;
        while (packet.hasRemaining(9)) {
            const uint8_t  icon = packet.readUInt8();
            const uint64_t guid = packet.readUInt64();
            data.marks.emplace_back(icon, guid);
        }
        return true;
    }

    // Single set. WotLK: whoGuid + icon + targetGuid (17 bytes); classic/TBC:
    // icon + targetGuid (9). Size tells them apart, so no expansion check is
    // needed and a server that differs from its era still decodes.
    const size_t remaining = packet.getRemainingSize();
    if (remaining >= 17) {
        packet.readUInt64();  // whoGuid — who placed the mark, not needed
    } else if (remaining < 9) {
        return false;
    }
    const uint8_t  icon = packet.readUInt8();
    const uint64_t guid = packet.readUInt64();
    data.marks.emplace_back(icon, guid);
    return true;
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
    // Duels are initiated via CMSG_CAST_SPELL with spell 7266 (Duel) targeted at the opponent.
    // There is no separate CMSG_DUEL_PROPOSED opcode in WoW.
    auto packet = CastSpellPacket::build(7266, targetGuid, 0);
    LOG_DEBUG("Built duel request (spell 7266) for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet BeginTradePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_BEGIN_TRADE));
    LOG_DEBUG("Built CMSG_BEGIN_TRADE");
    return packet;
}

network::Packet CancelTradePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_CANCEL_TRADE));
    LOG_DEBUG("Built CMSG_CANCEL_TRADE");
    return packet;
}

network::Packet AcceptTradePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_ACCEPT_TRADE));
    LOG_DEBUG("Built CMSG_ACCEPT_TRADE");
    return packet;
}

network::Packet SetTradeItemPacket::build(uint8_t tradeSlot, uint8_t bag, uint8_t bagSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_TRADE_ITEM));
    packet.writeUInt8(tradeSlot);
    packet.writeUInt8(bag);
    packet.writeUInt8(bagSlot);
    LOG_DEBUG("Built CMSG_SET_TRADE_ITEM slot=", static_cast<int>(tradeSlot), " bag=", static_cast<int>(bag), " bagSlot=", static_cast<int>(bagSlot));
    return packet;
}

network::Packet ClearTradeItemPacket::build(uint8_t tradeSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CLEAR_TRADE_ITEM));
    packet.writeUInt8(tradeSlot);
    LOG_DEBUG("Built CMSG_CLEAR_TRADE_ITEM slot=", static_cast<int>(tradeSlot));
    return packet;
}

network::Packet SetTradeGoldPacket::build(uint64_t copper) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_TRADE_GOLD));
    packet.writeUInt64(copper);
    LOG_DEBUG("Built CMSG_SET_TRADE_GOLD copper=", copper);
    return packet;
}

network::Packet UnacceptTradePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_UNACCEPT_TRADE));
    LOG_DEBUG("Built CMSG_UNACCEPT_TRADE");
    return packet;
}

network::Packet InitiateTradePacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_INITIATE_TRADE));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_INITIATE_TRADE for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

} // namespace game
} // namespace wowee
