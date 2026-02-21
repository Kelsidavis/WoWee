#include "game/packet_parsers.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace game {

// ============================================================================
// Classic 1.12.1 movement flag constants
// Key differences from TBC:
// - SPLINE_ENABLED at 0x00400000 (TBC/WotLK: 0x08000000)
// - No FLYING flag (flight was added in TBC)
// - ONTRANSPORT at 0x02000000 (not used for pitch in Classic)
// Same as TBC: ON_TRANSPORT=0x200, JUMPING=0x2000, SWIMMING=0x200000,
//              SPLINE_ELEVATION=0x04000000
// ============================================================================
namespace ClassicMoveFlags {
    constexpr uint32_t ONTRANSPORT      = 0x02000000;  // Gates transport data (vmangos authoritative)
    constexpr uint32_t JUMPING          = 0x00002000;  // Gates jump data
    constexpr uint32_t SWIMMING         = 0x00200000;  // Gates pitch
    constexpr uint32_t SPLINE_ENABLED   = 0x00400000;  // TBC/WotLK: 0x08000000
    constexpr uint32_t SPLINE_ELEVATION = 0x04000000;  // Same as TBC
}

// ============================================================================
// Classic parseMovementBlock
// Key differences from TBC:
// - NO moveFlags2 (TBC reads u8, WotLK reads u16)
// - SPLINE_ENABLED at 0x00400000 (not 0x08000000)
// - Transport data: NO timestamp (TBC adds u32 timestamp)
// - Pitch: only SWIMMING (no ONTRANSPORT secondary pitch, no FLYING)
// Same as TBC: u8 UpdateFlags, JUMPING=0x2000, 8 speeds, no pitchRate
// ============================================================================
bool ClassicPacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    // Classic: UpdateFlags is uint8 (same as TBC)
    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [Classic] UpdateFlags: 0x", std::hex, (int)updateFlags, std::dec);

    const uint8_t UPDATEFLAG_LIVING       = 0x20;
    const uint8_t UPDATEFLAG_HAS_POSITION = 0x40;
    const uint8_t UPDATEFLAG_HAS_TARGET   = 0x04;
    const uint8_t UPDATEFLAG_TRANSPORT    = 0x02;
    const uint8_t UPDATEFLAG_LOWGUID      = 0x08;
    const uint8_t UPDATEFLAG_HIGHGUID     = 0x10;

    if (updateFlags & UPDATEFLAG_LIVING) {
        // Movement flags (u32 only — NO extra flags byte in Classic)
        uint32_t moveFlags = packet.readUInt32();
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Classic] LIVING: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport data (Classic: ONTRANSPORT=0x02000000, no timestamp)
        if (moveFlags & ClassicMoveFlags::ONTRANSPORT) {
            block.onTransport = true;
            block.transportGuid = UpdateObjectParser::readPackedGuid(packet);
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            // Classic: NO transport timestamp (TBC adds u32 timestamp)
            // Classic: NO transport seat byte
        }

        // Pitch (Classic: only SWIMMING, no FLYING or ONTRANSPORT pitch)
        if (moveFlags & ClassicMoveFlags::SWIMMING) {
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping (Classic: JUMPING=0x2000, same as TBC)
        if (moveFlags & ClassicMoveFlags::JUMPING) {
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & ClassicMoveFlags::SPLINE_ELEVATION) {
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (Classic: 6 values — no flight speeds, no pitchRate)
        // TBC added flying_speed + backwards_flying_speed (8 total)
        // WotLK added pitchRate (9 total)
        /*float walkSpeed =*/ packet.readFloat();
        float runSpeed = packet.readFloat();
        /*float runBackSpeed =*/ packet.readFloat();
        /*float swimSpeed =*/ packet.readFloat();
        /*float swimBackSpeed =*/ packet.readFloat();
        /*float turnRate =*/ packet.readFloat();

        block.runSpeed = runSpeed;

        // Spline data (Classic: SPLINE_ENABLED=0x00400000)
        if (moveFlags & ClassicMoveFlags::SPLINE_ENABLED) {
            uint32_t splineFlags = packet.readUInt32();
            LOG_DEBUG("  [Classic] Spline: flags=0x", std::hex, splineFlags, std::dec);

            if (splineFlags & 0x00010000) { // FINAL_POINT
                /*float finalX =*/ packet.readFloat();
                /*float finalY =*/ packet.readFloat();
                /*float finalZ =*/ packet.readFloat();
            } else if (splineFlags & 0x00020000) { // FINAL_TARGET
                /*uint64_t finalTarget =*/ packet.readUInt64();
            } else if (splineFlags & 0x00040000) { // FINAL_ANGLE
                /*float finalAngle =*/ packet.readFloat();
            }

            // Classic spline: timePassed, duration, id, nodes, finalNode (same as TBC)
            /*uint32_t timePassed =*/ packet.readUInt32();
            /*uint32_t duration =*/ packet.readUInt32();
            /*uint32_t splineId =*/ packet.readUInt32();

            uint32_t pointCount = packet.readUInt32();
            if (pointCount > 256) {
                LOG_WARNING("  [Classic] Spline pointCount=", pointCount, " exceeds max, capping");
                pointCount = 0;
            }
            for (uint32_t i = 0; i < pointCount; i++) {
                /*float px =*/ packet.readFloat();
                /*float py =*/ packet.readFloat();
                /*float pz =*/ packet.readFloat();
            }

            // Classic: NO splineMode byte
            /*float endPointX =*/ packet.readFloat();
            /*float endPointY =*/ packet.readFloat();
            /*float endPointZ =*/ packet.readFloat();
        }
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Classic] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }

    // Target GUID
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        /*uint64_t targetGuid =*/ UpdateObjectParser::readPackedGuid(packet);
    }

    // Transport time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        /*uint32_t transportTime =*/ packet.readUInt32();
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

// ============================================================================
// Classic writeMovementPayload
// Key differences from TBC:
// - NO flags2 byte (TBC writes u8)
// - Transport data: NO timestamp
// - Pitch: only SWIMMING (no ONTRANSPORT pitch)
// ============================================================================
void ClassicPacketParsers::writeMovementPayload(network::Packet& packet, const MovementInfo& info) {
    // Movement flags (uint32)
    packet.writeUInt32(info.flags);

    // Classic: NO flags2 byte (TBC has u8, WotLK has u16)

    // Timestamp
    packet.writeUInt32(info.time);

    // Position
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.x), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.y), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.z), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.orientation), sizeof(float));

    // Transport data (Classic ONTRANSPORT = 0x02000000, no timestamp)
    if (info.flags & ClassicMoveFlags::ONTRANSPORT) {
        // Packed transport GUID
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

        // Transport local position
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportX), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportY), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportZ), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportO), sizeof(float));

        // Classic: NO transport timestamp
        // Classic: NO transport seat byte
    }

    // Pitch (Classic: only SWIMMING)
    if (info.flags & ClassicMoveFlags::SWIMMING) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.pitch), sizeof(float));
    }

    // Fall time (always present)
    packet.writeUInt32(info.fallTime);

    // Jump data (Classic JUMPING = 0x2000)
    if (info.flags & ClassicMoveFlags::JUMPING) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpVelocity), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpSinAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpCosAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpXYSpeed), sizeof(float));
    }
}

// ============================================================================
// Classic buildMovementPacket
// Classic/TBC: client movement packets do NOT include PackedGuid prefix
// (WotLK added PackedGuid to client packets)
// ============================================================================
network::Packet ClassicPacketParsers::buildMovementPacket(LogicalOpcode opcode,
                                                           const MovementInfo& info,
                                                           uint64_t /*playerGuid*/) {
    network::Packet packet(wireOpcode(opcode));

    // Classic: NO PackedGuid prefix for client packets
    writeMovementPayload(packet, info);

    return packet;
}

// ============================================================================
// Classic buildCastSpell
// Vanilla 1.12.x: NO castCount prefix, NO castFlags byte
// Format: uint32 spellId + uint16 targetFlags + [PackedGuid if unit target]
// ============================================================================
network::Packet ClassicPacketParsers::buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t /*castCount*/) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_CAST_SPELL));

    packet.writeUInt32(spellId);

    // SpellCastTargets — vanilla/CMaNGOS uses uint16 target mask (WotLK uses uint32)
    if (targetGuid != 0) {
        packet.writeUInt16(0x02); // TARGET_FLAG_UNIT

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
        packet.writeUInt16(0x00); // TARGET_FLAG_SELF
    }

    return packet;
}

// ============================================================================
// Classic CMSG_USE_ITEM
// Vanilla 1.12.x: bag(u8) + slot(u8) + spellIndex(u8) + SpellCastTargets(u16)
// NO spellId, itemGuid, glyphIndex, or castFlags fields (those are WotLK)
// ============================================================================
network::Packet ClassicPacketParsers::buildUseItem(uint8_t bagIndex, uint8_t slotIndex, uint64_t /*itemGuid*/) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_USE_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    packet.writeUInt8(0);       // spell_index (which item spell to trigger, usually 0)
    packet.writeUInt16(0x0000); // SpellCastTargets: TARGET_FLAG_SELF
    return packet;
}

// ============================================================================
// Classic SMSG_CAST_FAILED: no castCount byte (added in TBC/WotLK)
// Format: spellId(u32) + result(u8)
// ============================================================================
bool ClassicPacketParsers::parseCastFailed(network::Packet& packet, CastFailedData& data) {
    data.castCount = 0;
    data.spellId = packet.readUInt32();
    uint8_t vanillaResult = packet.readUInt8();
    // Vanilla enum starts at 0=AFFECTING_COMBAT (no SUCCESS entry).
    // WotLK enum starts at 0=SUCCESS, 1=AFFECTING_COMBAT.
    // Shift +1 to align with WotLK result strings.
    data.result = vanillaResult + 1;
    LOG_DEBUG("[Classic] Cast failed: spell=", data.spellId, " vanillaResult=", (int)vanillaResult);
    return true;
}

// ============================================================================
// Classic 1.12.1 parseCharEnum
// Differences from TBC:
// - Equipment: 20 items, but NO enchantment field per slot
//   Classic: displayId(u32) + inventoryType(u8) = 5 bytes/slot
//   TBC/WotLK: displayId(u32) + inventoryType(u8) + enchant(u32) = 9 bytes/slot
// - After flags: uint8 firstLogin (same as TBC)
// ============================================================================
bool ClassicPacketParsers::parseCharEnum(network::Packet& packet, CharEnumResponse& response) {
    uint8_t count = packet.readUInt8();

    LOG_INFO("[Classic] Parsing SMSG_CHAR_ENUM: ", (int)count, " characters");

    response.characters.clear();
    response.characters.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        Character character;

        // GUID (8 bytes)
        character.guid = packet.readUInt64();

        // Name (null-terminated string)
        character.name = packet.readString();

        // Race, class, gender
        character.race = static_cast<Race>(packet.readUInt8());
        character.characterClass = static_cast<Class>(packet.readUInt8());
        character.gender = static_cast<Gender>(packet.readUInt8());

        // Appearance (5 bytes: skin, face, hairStyle, hairColor packed + facialFeatures)
        character.appearanceBytes = packet.readUInt32();
        character.facialFeatures = packet.readUInt8();

        // Level
        character.level = packet.readUInt8();

        // Location
        character.zoneId = packet.readUInt32();
        character.mapId = packet.readUInt32();
        character.x = packet.readFloat();
        character.y = packet.readFloat();
        character.z = packet.readFloat();

        // Guild ID
        character.guildId = packet.readUInt32();

        // Flags
        character.flags = packet.readUInt32();

        // Classic: uint8 firstLogin (same as TBC)
        /*uint8_t firstLogin =*/ packet.readUInt8();

        // Pet data (always present)
        character.pet.displayModel = packet.readUInt32();
        character.pet.level = packet.readUInt32();
        character.pet.family = packet.readUInt32();

        // Equipment (Classic: 20 items, NO enchantment field)
        character.equipment.reserve(20);
        for (int j = 0; j < 20; ++j) {
            EquipmentItem item;
            item.displayModel = packet.readUInt32();
            item.inventoryType = packet.readUInt8();
            item.enchantment = 0;  // Classic has no enchant field in char enum
            character.equipment.push_back(item);
        }

        LOG_INFO("  Character ", (int)(i + 1), ": ", character.name);
        LOG_INFO("    GUID: 0x", std::hex, character.guid, std::dec);
        LOG_INFO("    ", getRaceName(character.race), " ",
                 getClassName(character.characterClass), " (",
                 getGenderName(character.gender), ")");
        LOG_INFO("    Level: ", (int)character.level);
        LOG_INFO("    Location: Zone ", character.zoneId, ", Map ", character.mapId);

        response.characters.push_back(character);
    }

    LOG_INFO("[Classic] Parsed ", response.characters.size(), " characters");
    return true;
}

// ============================================================================
// Classic 1.12.1 parseMessageChat
// Differences from WotLK:
// - NO uint32 unknown field after senderGuid
// - CHANNEL type: channelName + rank(u32) + senderGuid (not just channelName)
// - No ACHIEVEMENT/GUILD_ACHIEVEMENT types
// ============================================================================
bool ClassicPacketParsers::parseMessageChat(network::Packet& packet, MessageChatData& data) {
    if (packet.getSize() < 10) {
        LOG_ERROR("[Classic] SMSG_MESSAGECHAT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    // Read chat type
    uint8_t typeVal = packet.readUInt8();
    data.type = static_cast<ChatType>(typeVal);

    // Read language
    uint32_t langVal = packet.readUInt32();
    data.language = static_cast<ChatLanguage>(langVal);

    // Classic: NO uint32 unknown field here (WotLK has one)

    // Type-specific data (matches CMaNGOS-Classic BuildChatPacket)
    switch (data.type) {
        case ChatType::MONSTER_EMOTE: {
            // nameLen(u32) + name + targetGuid(u64)
            uint32_t nameLen = packet.readUInt32();
            if (nameLen > 0 && nameLen < 256) {
                data.senderName.resize(nameLen);
                for (uint32_t i = 0; i < nameLen; ++i) {
                    data.senderName[i] = static_cast<char>(packet.readUInt8());
                }
                if (!data.senderName.empty() && data.senderName.back() == '\0') {
                    data.senderName.pop_back();
                }
            }
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::SAY:
        case ChatType::PARTY:
        case ChatType::YELL:
        case ChatType::WHISPER:
        case ChatType::WHISPER_INFORM:
        case ChatType::GUILD:
        case ChatType::OFFICER:
        case ChatType::RAID:
        case ChatType::RAID_LEADER:
        case ChatType::RAID_WARNING:
        case ChatType::EMOTE:
        case ChatType::TEXT_EMOTE: {
            // senderGuid(u64) + senderGuid(u64) — written twice by server
            data.senderGuid = packet.readUInt64();
            /*duplicateGuid*/ packet.readUInt64();
            break;
        }

        case ChatType::MONSTER_SAY:
        case ChatType::MONSTER_YELL: {
            // senderGuid(u64) + nameLen(u32) + name + targetGuid(u64)
            data.senderGuid = packet.readUInt64();
            uint32_t nameLen = packet.readUInt32();
            if (nameLen > 0 && nameLen < 256) {
                data.senderName.resize(nameLen);
                for (uint32_t i = 0; i < nameLen; ++i) {
                    data.senderName[i] = static_cast<char>(packet.readUInt8());
                }
                if (!data.senderName.empty() && data.senderName.back() == '\0') {
                    data.senderName.pop_back();
                }
            }
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::CHANNEL: {
            // channelName(string) + rank(u32) + senderGuid(u64)
            data.channelName = packet.readString();
            /*uint32_t rank =*/ packet.readUInt32();
            data.senderGuid = packet.readUInt64();
            break;
        }

        default: {
            // All other types: senderGuid(u64) + senderGuid(u64) — written twice
            data.senderGuid = packet.readUInt64();
            /*duplicateGuid*/ packet.readUInt64();
            break;
        }
    }

    // Read message length
    uint32_t messageLen = packet.readUInt32();

    // Read message
    if (messageLen > 0 && messageLen < 8192) {
        data.message.resize(messageLen);
        for (uint32_t i = 0; i < messageLen; ++i) {
            data.message[i] = static_cast<char>(packet.readUInt8());
        }
        // Remove null terminator if present
        if (!data.message.empty() && data.message.back() == '\0') {
            data.message.pop_back();
        }
    }

    // Read chat tag
    if (packet.getReadPos() < packet.getSize()) {
        data.chatTag = packet.readUInt8();
    }

    LOG_DEBUG("[Classic] SMSG_MESSAGECHAT: type=", getChatTypeString(data.type),
             " sender=", data.senderName.empty() ? std::to_string(data.senderGuid) : data.senderName);

    return true;
}

// ============================================================================
// Classic CMSG_JOIN_CHANNEL / CMSG_LEAVE_CHANNEL
// Classic format: just string channelName + string password (no channelId/hasVoice/joinedByZone)
// ============================================================================

network::Packet ClassicPacketParsers::buildJoinChannel(const std::string& channelName, const std::string& password) {
    network::Packet packet(wireOpcode(Opcode::CMSG_JOIN_CHANNEL));
    packet.writeString(channelName);
    packet.writeString(password);
    LOG_DEBUG("[Classic] Built CMSG_JOIN_CHANNEL: channel=", channelName);
    return packet;
}

network::Packet ClassicPacketParsers::buildLeaveChannel(const std::string& channelName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEAVE_CHANNEL));
    packet.writeString(channelName);
    LOG_DEBUG("[Classic] Built CMSG_LEAVE_CHANNEL: channel=", channelName);
    return packet;
}

// ============================================================================
// Classic guild roster parser
// Differences from WotLK:
// - No rankCount field (fixed 10 ranks, read rights only)
// - No per-rank bank tab data
// - No gender byte per member
// ============================================================================

bool ClassicPacketParsers::parseGuildRoster(network::Packet& packet, GuildRosterData& data) {
    if (packet.getSize() < 4) {
        LOG_ERROR("Classic SMSG_GUILD_ROSTER too small: ", packet.getSize());
        return false;
    }
    uint32_t numMembers = packet.readUInt32();
    data.motd = packet.readString();
    data.guildInfo = packet.readString();

    // Classic: fixed 10 ranks, just uint32 rights each (no goldLimit, no bank tabs)
    data.ranks.resize(10);
    for (int i = 0; i < 10; ++i) {
        data.ranks[i].rights = packet.readUInt32();
        data.ranks[i].goldLimit = 0;
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
        // Classic: NO gender byte
        m.gender = 0;
        m.zoneId = packet.readUInt32();
        if (!m.online) {
            m.lastOnline = packet.readFloat();
        }
        m.publicNote = packet.readString();
        m.officerNote = packet.readString();
    }
    LOG_INFO("Parsed Classic SMSG_GUILD_ROSTER: ", numMembers, " members");
    return true;
}

// ============================================================================
// Classic guild query response parser
// Differences from WotLK:
// - No trailing rankCount uint32
// ============================================================================

bool ClassicPacketParsers::parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) {
    if (packet.getSize() < 8) {
        LOG_ERROR("Classic SMSG_GUILD_QUERY_RESPONSE too small: ", packet.getSize());
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
    // Classic: NO trailing rankCount
    data.rankCount = 10;
    LOG_INFO("Parsed Classic SMSG_GUILD_QUERY_RESPONSE: guild=", data.guildName);
    return true;
}

// ============================================================================
// GameObject Query — Classic has no extra strings before data[]
// WotLK has iconName + castBarCaption + unk1 between names and data[].
// Vanilla: entry, type, displayId, name[4], data[24]
// ============================================================================

bool ClassicPacketParsers::parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) {
    data.entry = packet.readUInt32();

    // High bit set means gameobject not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        data.name = "";
        return true;
    }

    data.type = packet.readUInt32();
    data.displayId = packet.readUInt32();
    // 4 name strings
    data.name = packet.readString();
    packet.readString();
    packet.readString();
    packet.readString();

    // Classic: data[24] comes immediately after names (no extra strings)
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining >= 24 * 4) {
        for (int i = 0; i < 24; i++) {
            data.data[i] = packet.readUInt32();
        }
        data.hasData = true;
    }

    if (data.type == 15) { // MO_TRANSPORT
        LOG_DEBUG("Classic GO query: MO_TRANSPORT entry=", data.entry,
                  " name=\"", data.name, "\" displayId=", data.displayId,
                  " taxiPathId=", data.data[0], " moveSpeed=", data.data[1]);
    } else {
        LOG_DEBUG("Classic GO query: ", data.name, " type=", data.type, " entry=", data.entry);
    }
    return true;
}

// ============================================================================
// Gossip — Classic has no menuId, and quest items lack questFlags + isRepeatable
// ============================================================================

bool ClassicPacketParsers::parseGossipMessage(network::Packet& packet, GossipMessageData& data) {
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining < 8 + 4 + 4) {
        LOG_ERROR("Classic SMSG_GOSSIP_MESSAGE too small: ", remaining, " bytes");
        return false;
    }

    data.npcGuid = packet.readUInt64();
    // Classic: NO menuId field (WotLK adds uint32 menuId here)
    data.menuId = 0;
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

    data.options.clear();
    data.options.reserve(optionCount);
    for (uint32_t i = 0; i < optionCount; ++i) {
        GossipOption opt;
        opt.id = packet.readUInt32();
        opt.icon = packet.readUInt8();
        opt.isCoded = (packet.readUInt8() != 0);
        // Classic/Vanilla: NO boxMoney or boxText fields (commented out in mangoszero)
        opt.boxMoney = 0;
        opt.text = packet.readString();
        opt.boxText = "";
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
        // Classic: NO questFlags, NO isRepeatable
        quest.questFlags = 0;
        quest.isRepeatable = 0;
        quest.title = normalizeWowTextTokens(packet.readString());
        data.quests.push_back(quest);
    }

    LOG_INFO("Classic Gossip: ", optionCount, " options, ", questCount, " quests");
    return true;
}

// ============================================================================
// Classic CMSG_SEND_MAIL — Vanilla 1.12 format
// Differences from WotLK:
// - Single uint64 itemGuid instead of uint8 attachmentCount + item array
// - Trailing uint64 unk3 + uint8 unk4 (clients > 1.9.4)
// ============================================================================
network::Packet ClassicPacketParsers::buildSendMail(uint64_t mailboxGuid,
                                                     const std::string& recipient,
                                                     const std::string& subject,
                                                     const std::string& body,
                                                     uint32_t money, uint32_t cod) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SEND_MAIL));
    packet.writeUInt64(mailboxGuid);
    packet.writeString(recipient);
    packet.writeString(subject);
    packet.writeString(body);
    packet.writeUInt32(0);       // stationery
    packet.writeUInt32(0);       // unknown
    packet.writeUInt64(0);       // item GUID (0 = no attachment, single item only in Vanilla)
    packet.writeUInt32(money);
    packet.writeUInt32(cod);
    packet.writeUInt64(0);       // unk3 (clients > 1.9.4)
    packet.writeUInt8(0);        // unk4 (clients > 1.9.4)
    return packet;
}

// ============================================================================
// Classic SMSG_MAIL_LIST_RESULT — Vanilla 1.12 format (per vmangos)
// Key differences from WotLK:
// - uint8 count (not uint32 totalCount + uint8 shownCount)
// - No msgSize prefix per entry
// - Subject comes before item data
// - Single inline item (not attachment count + array)
// - uint8 stackCount (not uint32)
// - No enchantment array (single permanentEnchant uint32)
// ============================================================================
bool ClassicPacketParsers::parseMailList(network::Packet& packet,
                                         std::vector<MailMessage>& inbox) {
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining < 1) return false;

    uint8_t count = packet.readUInt8();
    LOG_INFO("SMSG_MAIL_LIST_RESULT (Classic): count=", (int)count);

    inbox.clear();
    inbox.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        remaining = packet.getSize() - packet.getReadPos();
        if (remaining < 5) {
            LOG_WARNING("Classic mail entry ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        MailMessage msg;

        // vmangos HandleGetMailList format:
        // u32 messageId, u8 messageType, sender (guid or u32),
        // string subject, u32 itemTextId, u32 package, u32 stationery,
        // item fields (entry, enchant, randomProp, suffixFactor,
        //              u8 stackCount, u32 charges, u32 maxDur, u32 dur),
        // u32 money, u32 cod, u32 flags, float expirationTime,
        // u32 mailTemplateId (build-dependent)
        msg.messageId = packet.readUInt32();
        msg.messageType = packet.readUInt8();

        switch (msg.messageType) {
            case 0: msg.senderGuid = packet.readUInt64(); break;
            default: msg.senderEntry = packet.readUInt32(); break;
        }

        msg.subject = packet.readString();

        uint32_t itemTextId = packet.readUInt32();
        (void)itemTextId;
        packet.readUInt32(); // package (unused)
        msg.stationeryId = packet.readUInt32();

        // Single inline item (Vanilla: one item per mail)
        uint32_t itemEntry = packet.readUInt32();
        uint32_t permanentEnchant = packet.readUInt32();
        uint32_t randomPropertyId = packet.readUInt32();
        uint32_t suffixFactor = packet.readUInt32();
        uint8_t stackCount = packet.readUInt8();
        packet.readUInt32(); // charges
        uint32_t maxDurability = packet.readUInt32();
        uint32_t durability = packet.readUInt32();

        if (itemEntry != 0) {
            MailAttachment att;
            att.slot = 0;
            att.itemGuidLow = 0; // Not provided in Vanilla list
            att.itemId = itemEntry;
            att.enchantId = permanentEnchant;
            att.randomPropertyId = randomPropertyId;
            att.randomSuffix = suffixFactor;
            att.stackCount = stackCount;
            att.chargesOrDurability = durability;
            att.maxDurability = maxDurability;
            msg.attachments.push_back(att);
        }

        msg.money = packet.readUInt32();
        msg.cod = packet.readUInt32();
        msg.flags = packet.readUInt32();
        msg.expirationTime = packet.readFloat();
        msg.mailTemplateId = packet.readUInt32();

        msg.read = (msg.flags & 0x01) != 0;
        inbox.push_back(std::move(msg));
    }

    LOG_INFO("Parsed ", inbox.size(), " mail messages");
    return true;
}

// ============================================================================
// Classic CMSG_MAIL_TAKE_ITEM — Vanilla only sends mailboxGuid + mailId
// (no itemSlot — Vanilla only supports 1 item per mail)
// ============================================================================
network::Packet ClassicPacketParsers::buildMailTakeItem(uint64_t mailboxGuid,
                                                         uint32_t mailId,
                                                         uint32_t /*itemSlot*/) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_TAKE_ITEM));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

// ============================================================================
// Classic CMSG_MAIL_DELETE — Vanilla only sends mailboxGuid + mailId
// (no mailTemplateId field)
// ============================================================================
network::Packet ClassicPacketParsers::buildMailDelete(uint64_t mailboxGuid,
                                                       uint32_t mailId,
                                                       uint32_t /*mailTemplateId*/) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_DELETE));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

// ============================================================================
// Classic SMSG_ITEM_QUERY_SINGLE_RESPONSE
// Vanilla has NO SoundOverrideSubclass, NO Flags2, NO ScalingStatDistribution,
// NO ScalingStatValue, and only 2 damage types (not 5).
// ============================================================================
network::Packet ClassicPacketParsers::buildItemQuery(uint32_t entry, uint64_t guid) {
    // Vanilla CMSG_ITEM_QUERY_SINGLE: uint32 entry + uint64 guid (same as WotLK)
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_QUERY_SINGLE));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("[Classic] Built CMSG_ITEM_QUERY_SINGLE: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

bool ClassicPacketParsers::parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) {
    data.entry = packet.readUInt32();

    // High bit set means item not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        return true;
    }

    uint32_t itemClass = packet.readUInt32();
    uint32_t subClass = packet.readUInt32();
    // Vanilla: NO SoundOverrideSubclass

    data.itemClass = itemClass;
    data.subClass = subClass;
    data.subclassName = "";
    if (itemClass == 2) { // Weapon
        switch (subClass) {
            case 0: data.subclassName = "Axe"; break;
            case 1: data.subclassName = "Axe"; break;
            case 2: data.subclassName = "Bow"; break;
            case 3: data.subclassName = "Gun"; break;
            case 4: data.subclassName = "Mace"; break;
            case 5: data.subclassName = "Mace"; break;
            case 6: data.subclassName = "Polearm"; break;
            case 7: data.subclassName = "Sword"; break;
            case 8: data.subclassName = "Sword"; break;
            case 10: data.subclassName = "Staff"; break;
            case 13: data.subclassName = "Fist Weapon"; break;
            case 15: data.subclassName = "Dagger"; break;
            case 16: data.subclassName = "Thrown"; break;
            case 18: data.subclassName = "Crossbow"; break;
            case 19: data.subclassName = "Wand"; break;
            case 20: data.subclassName = "Fishing Pole"; break;
            default: data.subclassName = "Weapon"; break;
        }
    } else if (itemClass == 4) { // Armor
        switch (subClass) {
            case 0: data.subclassName = "Miscellaneous"; break;
            case 1: data.subclassName = "Cloth"; break;
            case 2: data.subclassName = "Leather"; break;
            case 3: data.subclassName = "Mail"; break;
            case 4: data.subclassName = "Plate"; break;
            case 6: data.subclassName = "Shield"; break;
            default: data.subclassName = "Armor"; break;
        }
    }

    // 4 name strings
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4

    data.displayInfoId = packet.readUInt32();
    data.quality = packet.readUInt32();

    packet.readUInt32(); // Flags
    // Vanilla: NO Flags2
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

    // Vanilla: 10 stat pairs, NO statsCount prefix
    for (uint32_t i = 0; i < 10; i++) {
        uint32_t statType = packet.readUInt32();
        int32_t statValue = static_cast<int32_t>(packet.readUInt32());
        if (statType != 0) {
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

    // Vanilla: NO ScalingStatDistribution, NO ScalingStatValue

    // Vanilla: 5 damage types (same count as WotLK)
    bool haveWeaponDamage = false;
    for (int i = 0; i < 5; i++) {
        float dmgMin = packet.readFloat();
        float dmgMax = packet.readFloat();
        uint32_t damageType = packet.readUInt32();
        if (!haveWeaponDamage && dmgMax > 0.0f) {
            // Prefer physical damage (type 0) when present.
            if (damageType == 0 || data.damageMax <= 0.0f) {
                data.damageMin = dmgMin;
                data.damageMax = dmgMax;
                haveWeaponDamage = (damageType == 0);
            }
        }
    }

    data.armor = static_cast<int32_t>(packet.readUInt32());

    // Remaining tail can vary by core. Read resistances + delay when present.
    if (packet.getSize() - packet.getReadPos() >= 28) {
        packet.readUInt32(); // HolyRes
        packet.readUInt32(); // FireRes
        packet.readUInt32(); // NatureRes
        packet.readUInt32(); // FrostRes
        packet.readUInt32(); // ShadowRes
        packet.readUInt32(); // ArcaneRes
        data.delayMs = packet.readUInt32();
    }

    data.valid = !data.name.empty();
    LOG_DEBUG("[Classic] Item query response: ", data.name, " (quality=", data.quality,
             " invType=", data.inventoryType, " stack=", data.maxStack, ")");
    return true;
}

// ============================================================================
// Turtle WoW (build 7234) parseMovementBlock
//
// Turtle WoW is a heavily modified vanilla (1.12.1) server.  Through hex dump
// analysis the wire format is nearly identical to Classic with one key addition:
//
//   LIVING section:
//     moveFlags       u32     (NO moveFlags2 — confirmed by position alignment)
//     time            u32
//     position        4×float
//     transport       guarded by moveFlags & 0x02000000 (Classic flag)
//                     packed GUID + 4 floats + u32 timestamp (TBC-style addition)
//     pitch           guarded by SWIMMING (0x200000)
//     fallTime        u32
//     jump data       guarded by JUMPING  (0x2000)
//     splineElev      guarded by 0x04000000
//     speeds          6 floats (walk/run/runBack/swim/swimBack/turnRate)
//     spline          guarded by 0x00400000 (Classic flag) OR 0x08000000 (TBC flag)
//
//   Tail (same as Classic):
//     LOWGUID  → 1×u32
//     HIGHGUID → 1×u32
//
// The ONLY confirmed difference from pure Classic is:
//   Transport data includes a u32 timestamp after the 4 transport floats
//   (Classic omits this; TBC/WotLK include it).  Without this, entities on
//   transports cause a 4-byte desync that cascades to later blocks.
// ============================================================================
namespace TurtleMoveFlags {
    constexpr uint32_t ONTRANSPORT      = 0x02000000;  // Classic transport flag
    constexpr uint32_t JUMPING          = 0x00002000;
    constexpr uint32_t SWIMMING         = 0x00200000;
    constexpr uint32_t SPLINE_ELEVATION = 0x04000000;
    constexpr uint32_t SPLINE_CLASSIC   = 0x00400000;  // Classic spline enabled
    constexpr uint32_t SPLINE_TBC       = 0x08000000;  // TBC spline enabled
}

bool TurtlePacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [Turtle] UpdateFlags: 0x", std::hex, (int)updateFlags, std::dec);

    const uint8_t UPDATEFLAG_LIVING       = 0x20;
    const uint8_t UPDATEFLAG_HAS_POSITION = 0x40;
    const uint8_t UPDATEFLAG_HAS_TARGET   = 0x04;
    const uint8_t UPDATEFLAG_TRANSPORT    = 0x02;
    const uint8_t UPDATEFLAG_LOWGUID      = 0x08;
    const uint8_t UPDATEFLAG_HIGHGUID     = 0x10;

    if (updateFlags & UPDATEFLAG_LIVING) {
        size_t livingStart = packet.getReadPos();

        uint32_t moveFlags = packet.readUInt32();
        // Turtle: NO moveFlags2 (confirmed by hex dump — positions are only correct without it)
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Turtle] LIVING: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport — Classic flag position 0x02000000
        if (moveFlags & TurtleMoveFlags::ONTRANSPORT) {
            block.onTransport = true;
            block.transportGuid = UpdateObjectParser::readPackedGuid(packet);
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t transportTime =*/ packet.readUInt32();  // Turtle adds TBC-style timestamp
        }

        // Pitch (swimming only, Classic-style)
        if (moveFlags & TurtleMoveFlags::SWIMMING) {
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jump data
        if (moveFlags & TurtleMoveFlags::JUMPING) {
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & TurtleMoveFlags::SPLINE_ELEVATION) {
            /*float splineElevation =*/ packet.readFloat();
        }

        // Turtle: 6 speeds (same as Classic — no flight speeds)
        float walkSpeed = packet.readFloat();
        float runSpeed = packet.readFloat();
        float runBackSpeed = packet.readFloat();
        float swimSpeed = packet.readFloat();
        float swimBackSpeed = packet.readFloat();
        float turnRate = packet.readFloat();

        block.runSpeed = runSpeed;

        LOG_DEBUG("  [Turtle] Speeds: walk=", walkSpeed, " run=", runSpeed,
                  " runBack=", runBackSpeed, " swim=", swimSpeed,
                  " swimBack=", swimBackSpeed, " turn=", turnRate);

        // Spline data — check both Classic (0x00400000) and TBC (0x08000000) flag positions
        bool hasSpline = (moveFlags & TurtleMoveFlags::SPLINE_CLASSIC) ||
                         (moveFlags & TurtleMoveFlags::SPLINE_TBC);
        if (hasSpline) {
            uint32_t splineFlags = packet.readUInt32();
            LOG_DEBUG("  [Turtle] Spline: flags=0x", std::hex, splineFlags, std::dec);

            if (splineFlags & 0x00010000) {
                packet.readFloat(); packet.readFloat(); packet.readFloat();
            } else if (splineFlags & 0x00020000) {
                packet.readUInt64();
            } else if (splineFlags & 0x00040000) {
                packet.readFloat();
            }

            /*uint32_t timePassed =*/ packet.readUInt32();
            /*uint32_t duration =*/ packet.readUInt32();
            /*uint32_t splineId =*/ packet.readUInt32();

            uint32_t pointCount = packet.readUInt32();
            if (pointCount > 256) {
                LOG_WARNING("  [Turtle] Spline pointCount=", pointCount, " exceeds max, capping");
                pointCount = 0;
            }
            for (uint32_t i = 0; i < pointCount; i++) {
                packet.readFloat(); packet.readFloat(); packet.readFloat();
            }

            // End point
            packet.readFloat(); packet.readFloat(); packet.readFloat();
        }

        LOG_DEBUG("  [Turtle] LIVING block consumed ", packet.getReadPos() - livingStart,
                  " bytes, readPos now=", packet.getReadPos());
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Turtle] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }

    // Target GUID
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        /*uint64_t targetGuid =*/ UpdateObjectParser::readPackedGuid(packet);
    }

    // Transport time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    // Low GUID — Classic-style: 1×u32 (NOT TBC's 2×u32)
    if (updateFlags & UPDATEFLAG_LOWGUID) {
        /*uint32_t lowGuid =*/ packet.readUInt32();
    }

    // High GUID — 1×u32
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        /*uint32_t highGuid =*/ packet.readUInt32();
    }

    return true;
}

// ============================================================================
// Classic/Vanilla quest giver status
//
// Vanilla sends status as uint32 with different enum values:
//   0=NONE, 1=UNAVAILABLE, 2=CHAT, 3=INCOMPLETE, 4=REWARD_REP, 5=AVAILABLE
// WotLK uses uint8 with:
//   0=NONE, 1=UNAVAILABLE, 5=INCOMPLETE, 6=REWARD_REP, 7=AVAILABLE_LOW, 8=AVAILABLE, 10=REWARD
//
// Read uint32, translate to WotLK enum values.
// ============================================================================
uint8_t ClassicPacketParsers::readQuestGiverStatus(network::Packet& packet) {
    uint32_t vanillaStatus = packet.readUInt32();
    switch (vanillaStatus) {
        case 0: return 0;  // NONE
        case 1: return 1;  // UNAVAILABLE
        case 2: return 0;  // CHAT → NONE (no marker)
        case 3: return 5;  // INCOMPLETE → WotLK INCOMPLETE
        case 4: return 6;  // REWARD_REP → WotLK REWARD_REP
        case 5: return 8;  // AVAILABLE → WotLK AVAILABLE
        case 6: return 10; // REWARD → WotLK REWARD
        default: return 0;
    }
}

// ============================================================================
// Classic CMSG_QUESTGIVER_QUERY_QUEST — Vanilla 1.12 format
// WotLK appends a trailing unk1 byte; Vanilla servers don't expect it and
// some reject or misparse the 13-byte packet, preventing quest details from
// being sent back.  Classic format: guid(8) + questId(4) = 12 bytes.
// ============================================================================
network::Packet ClassicPacketParsers::buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_QUERY_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    // No trailing unk byte (WotLK-only field)
    return packet;
}

network::Packet ClassicPacketParsers::buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    // Classic/Turtle: no trailing unk1 uint32
    return packet;
}

// ============================================================================
// Classic SMSG_QUESTGIVER_QUEST_DETAILS — Vanilla 1.12 format
// WotLK inserts an informUnit GUID (8 bytes) between npcGuid and questId.
// Vanilla has: npcGuid(8) + questId(4) + title + details + objectives + ...
// ============================================================================
bool ClassicPacketParsers::parseQuestDetails(network::Packet& packet, QuestDetailsData& data) {
    if (packet.getSize() < 16) return false;

    data.npcGuid = packet.readUInt64();
    // Vanilla: questId follows immediately — no informUnit GUID
    data.questId = packet.readUInt32();
    data.title      = normalizeWowTextTokens(packet.readString());
    data.details    = normalizeWowTextTokens(packet.readString());
    data.objectives = normalizeWowTextTokens(packet.readString());

    if (packet.getReadPos() + 5 > packet.getSize()) {
        LOG_INFO("Quest details classic (short): id=", data.questId, " title='", data.title, "'");
        return !data.title.empty() || data.questId != 0;
    }

    /*activateAccept*/ packet.readUInt8();
    data.suggestedPlayers = packet.readUInt32();

    // Choice reward items: variable count + 3 uint32s each
    if (packet.getReadPos() + 4 <= packet.getSize()) {
        uint32_t choiceCount = packet.readUInt32();
        for (uint32_t i = 0; i < choiceCount && packet.getReadPos() + 12 <= packet.getSize(); ++i) {
            packet.readUInt32(); // itemId
            packet.readUInt32(); // count
            packet.readUInt32(); // displayInfo
        }
    }

    // Fixed reward items: variable count + 3 uint32s each
    if (packet.getReadPos() + 4 <= packet.getSize()) {
        uint32_t rewardCount = packet.readUInt32();
        for (uint32_t i = 0; i < rewardCount && packet.getReadPos() + 12 <= packet.getSize(); ++i) {
            packet.readUInt32(); // itemId
            packet.readUInt32(); // count
            packet.readUInt32(); // displayInfo
        }
    }

    if (packet.getReadPos() + 4 <= packet.getSize())
        data.rewardMoney = packet.readUInt32();

    LOG_INFO("Quest details classic: id=", data.questId, " title='", data.title, "'");
    return true;
}

} // namespace game
} // namespace wowee
