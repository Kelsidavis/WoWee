#pragma once

#include "game/world_packets.hpp"
#include <memory>
#include <string>

namespace wowee {
namespace game {

/**
 * PacketParsers - Polymorphic interface for expansion-specific packet parsing.
 *
 * Binary packet formats differ significantly between WoW expansions
 * (movement flags, update fields, character enum layout, etc.).
 * Each expansion implements this interface with its specific parsing logic.
 *
 * The base PacketParsers delegates to the existing static parser classes
 * in world_packets.hpp. Expansion subclasses override the methods that
 * differ from WotLK.
 */
class PacketParsers {
public:
    virtual ~PacketParsers() = default;

    // Size of MovementInfo.flags2 in bytes for MSG_MOVE_* payloads.
    // Classic: none, TBC: u8, WotLK: u16.
    virtual uint8_t movementFlags2Size() const { return 2; }

    // --- Movement ---

    /** Parse movement block from SMSG_UPDATE_OBJECT */
    virtual bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
        return UpdateObjectParser::parseMovementBlock(packet, block);
    }

    /** Write movement payload for CMSG_MOVE_* packets */
    virtual void writeMovementPayload(network::Packet& packet, const MovementInfo& info) {
        MovementPacket::writeMovementPayload(packet, info);
    }

    /** Build a complete movement packet with packed GUID + payload */
    virtual network::Packet buildMovementPacket(LogicalOpcode opcode,
                                                 const MovementInfo& info,
                                                 uint64_t playerGuid = 0) {
        return MovementPacket::build(opcode, info, playerGuid);
    }

    /** Build CMSG_CAST_SPELL (WotLK default: castCount + spellId + castFlags + targets) */
    virtual network::Packet buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
        return CastSpellPacket::build(spellId, targetGuid, castCount);
    }

    /** Build CMSG_USE_ITEM (WotLK default: bag + slot + castCount + spellId + itemGuid + glyphIndex + castFlags + targets) */
    virtual network::Packet buildUseItem(uint8_t bagIndex, uint8_t slotIndex, uint64_t itemGuid) {
        return UseItemPacket::build(bagIndex, slotIndex, itemGuid);
    }

    // --- Character Enumeration ---

    /** Parse SMSG_CHAR_ENUM */
    virtual bool parseCharEnum(network::Packet& packet, CharEnumResponse& response) {
        return CharEnumParser::parse(packet, response);
    }

    // --- Update Object ---

    /** Parse a full SMSG_UPDATE_OBJECT packet */
    virtual bool parseUpdateObject(network::Packet& packet, UpdateObjectData& data) {
        return UpdateObjectParser::parse(packet, data);
    }

    /** Parse update fields block (value mask + field values) */
    virtual bool parseUpdateFields(network::Packet& packet, UpdateBlock& block) {
        return UpdateObjectParser::parseUpdateFields(packet, block);
    }

    // --- Monster Movement ---

    /** Parse SMSG_MONSTER_MOVE */
    virtual bool parseMonsterMove(network::Packet& packet, MonsterMoveData& data) {
        return MonsterMoveParser::parse(packet, data);
    }

    // --- Combat ---

    /** Parse SMSG_ATTACKERSTATEUPDATE */
    virtual bool parseAttackerStateUpdate(network::Packet& packet, AttackerStateUpdateData& data) {
        return AttackerStateUpdateParser::parse(packet, data);
    }

    /** Parse SMSG_SPELLNONMELEEDAMAGELOG */
    virtual bool parseSpellDamageLog(network::Packet& packet, SpellDamageLogData& data) {
        return SpellDamageLogParser::parse(packet, data);
    }

    // --- Spells ---

    /** Parse SMSG_INITIAL_SPELLS */
    virtual bool parseInitialSpells(network::Packet& packet, InitialSpellsData& data) {
        return InitialSpellsParser::parse(packet, data);
    }

    /** Parse SMSG_CAST_FAILED */
    virtual bool parseCastFailed(network::Packet& packet, CastFailedData& data) {
        return CastFailedParser::parse(packet, data);
    }

    /** Parse SMSG_AURA_UPDATE / SMSG_AURA_UPDATE_ALL */
    virtual bool parseAuraUpdate(network::Packet& packet, AuraUpdateData& data, bool isAll = false) {
        return AuraUpdateParser::parse(packet, data, isAll);
    }

    // --- Chat ---

    /** Parse SMSG_MESSAGECHAT */
    virtual bool parseMessageChat(network::Packet& packet, MessageChatData& data) {
        return MessageChatParser::parse(packet, data);
    }

    /** Parse SMSG_NAME_QUERY_RESPONSE */
    virtual bool parseNameQueryResponse(network::Packet& packet, NameQueryResponseData& data) {
        return NameQueryResponseParser::parse(packet, data);
    }

    // --- Item Query ---

    /** Build CMSG_ITEM_QUERY_SINGLE */
    virtual network::Packet buildItemQuery(uint32_t entry, uint64_t guid) {
        return ItemQueryPacket::build(entry, guid);
    }

    /** Parse SMSG_ITEM_QUERY_SINGLE_RESPONSE */
    virtual bool parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) {
        return ItemQueryResponseParser::parse(packet, data);
    }

    // --- GameObject Query ---

    /** Parse SMSG_GAMEOBJECT_QUERY_RESPONSE */
    virtual bool parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) {
        return GameObjectQueryResponseParser::parse(packet, data);
    }

    // --- Gossip ---

    /** Parse SMSG_GOSSIP_MESSAGE */
    virtual bool parseGossipMessage(network::Packet& packet, GossipMessageData& data) {
        return GossipMessageParser::parse(packet, data);
    }

    // --- Quest details ---

    /** Build CMSG_QUESTGIVER_QUERY_QUEST.
     *  WotLK appends a trailing unk1 byte; Vanilla/Classic do not. */
    virtual network::Packet buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) {
        return QuestgiverQueryQuestPacket::build(npcGuid, questId);  // includes unk1
    }

    /** Build CMSG_QUESTGIVER_ACCEPT_QUEST.
     *  WotLK/AzerothCore expects trailing unk1 uint32; older expansions may not. */
    virtual network::Packet buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) {
        return QuestgiverAcceptQuestPacket::build(npcGuid, questId);
    }

    /** Parse SMSG_QUESTGIVER_QUEST_DETAILS.
     *  WotLK has an extra informUnit GUID before questId; Vanilla/Classic do not. */
    virtual bool parseQuestDetails(network::Packet& packet, QuestDetailsData& data) {
        return QuestDetailsParser::parse(packet, data);  // WotLK auto-detect
    }

    /** Stride of PLAYER_QUEST_LOG fields in update-object blocks.
     *  WotLK: 5 fields per slot, Classic/Vanilla: 3. */
    virtual uint8_t questLogStride() const { return 5; }

    // --- Quest Giver Status ---

    /** Read quest giver status from packet.
     *  WotLK: uint8, vanilla/classic: uint32 with different enum values.
     *  Returns the status value normalized to WotLK enum values. */
    virtual uint8_t readQuestGiverStatus(network::Packet& packet) {
        return packet.readUInt8();
    }

    // --- Destroy Object ---

    /** Parse SMSG_DESTROY_OBJECT */
    virtual bool parseDestroyObject(network::Packet& packet, DestroyObjectData& data) {
        return DestroyObjectParser::parse(packet, data);
    }

    // --- Guild ---

    /** Parse SMSG_GUILD_ROSTER */
    virtual bool parseGuildRoster(network::Packet& packet, GuildRosterData& data) {
        return GuildRosterParser::parse(packet, data);
    }

    /** Parse SMSG_GUILD_QUERY_RESPONSE */
    virtual bool parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) {
        return GuildQueryResponseParser::parse(packet, data);
    }

    // --- Channels ---

    /** Build CMSG_JOIN_CHANNEL */
    virtual network::Packet buildJoinChannel(const std::string& channelName, const std::string& password) {
        return JoinChannelPacket::build(channelName, password);
    }

    /** Build CMSG_LEAVE_CHANNEL */
    virtual network::Packet buildLeaveChannel(const std::string& channelName) {
        return LeaveChannelPacket::build(channelName);
    }

    // --- Mail ---

    /** Build CMSG_SEND_MAIL */
    virtual network::Packet buildSendMail(uint64_t mailboxGuid, const std::string& recipient,
                                           const std::string& subject, const std::string& body,
                                           uint32_t money, uint32_t cod) {
        return SendMailPacket::build(mailboxGuid, recipient, subject, body, money, cod);
    }

    /** Parse SMSG_MAIL_LIST_RESULT into a vector of MailMessage */
    virtual bool parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox);

    /** Build CMSG_MAIL_TAKE_ITEM */
    virtual network::Packet buildMailTakeItem(uint64_t mailboxGuid, uint32_t mailId, uint32_t itemSlot) {
        return MailTakeItemPacket::build(mailboxGuid, mailId, itemSlot);
    }

    /** Build CMSG_MAIL_DELETE */
    virtual network::Packet buildMailDelete(uint64_t mailboxGuid, uint32_t mailId, uint32_t mailTemplateId) {
        return MailDeletePacket::build(mailboxGuid, mailId, mailTemplateId);
    }

    // --- Utility ---

    /** Read a packed GUID from the packet */
    virtual uint64_t readPackedGuid(network::Packet& packet) {
        return UpdateObjectParser::readPackedGuid(packet);
    }

    /** Write a packed GUID to the packet */
    virtual void writePackedGuid(network::Packet& packet, uint64_t guid) {
        MovementPacket::writePackedGuid(packet, guid);
    }
};

/**
 * WotLK 3.3.5a packet parsers.
 *
 * Uses the default implementations which delegate to the existing
 * static parser classes. All current parsing code is WotLK-specific,
 * so no overrides are needed.
 */
class WotlkPacketParsers : public PacketParsers {
    // All methods use the defaults from PacketParsers base class,
    // which delegate to the existing WotLK static parsers.
};

/**
 * TBC 2.4.3 packet parsers.
 *
 * Overrides methods where TBC binary format differs from WotLK:
 * - SMSG_UPDATE_OBJECT: u8 has_transport after blockCount (WotLK removed it)
 * - UpdateFlags is u8 (not u16), no VEHICLE/ROTATION/POSITION flags
 * - Movement flags2 is u8 (not u16), no transport seat byte
 * - Movement flags: JUMPING=0x2000 gates jump data (WotLK: FALLING=0x1000)
 * - SPLINE_ENABLED=0x08000000, SPLINE_ELEVATION=0x04000000 (same as WotLK)
 * - Pitch: SWIMMING or else ONTRANSPORT(0x02000000)
 * - CharEnum: uint8 firstLogin (not uint32+uint8), 20 equipment items (not 23)
 * - Aura updates use inline update fields, not SMSG_AURA_UPDATE
 */
class TbcPacketParsers : public PacketParsers {
public:
    uint8_t movementFlags2Size() const override { return 1; }
    bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) override;
    void writeMovementPayload(network::Packet& packet, const MovementInfo& info) override;
    network::Packet buildMovementPacket(LogicalOpcode opcode,
                                         const MovementInfo& info,
                                         uint64_t playerGuid = 0) override;
    bool parseUpdateObject(network::Packet& packet, UpdateObjectData& data) override;
    bool parseCharEnum(network::Packet& packet, CharEnumResponse& response) override;
    bool parseAuraUpdate(network::Packet& packet, AuraUpdateData& data, bool isAll = false) override;
    bool parseNameQueryResponse(network::Packet& packet, NameQueryResponseData& data) override;
    bool parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) override;
    network::Packet buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) override;
};

/**
 * Classic 1.12.1 packet parsers.
 *
 * Inherits from TBC (shared: u8 UpdateFlags, has_transport byte).
 *
 * Differences from TBC:
 * - No moveFlags2 byte (TBC has u8, Classic has none)
 * - Only 6 speed fields (no flight speeds — flying added in TBC)
 * - SPLINE_ENABLED at 0x00400000 (TBC/WotLK: 0x08000000)
 * - Transport data has no timestamp (TBC adds u32 timestamp)
 * - Pitch: only SWIMMING (no ONTRANSPORT secondary pitch)
 * - CharEnum: no enchantment field per equipment slot
 * - No SMSG_AURA_UPDATE (uses update fields, same as TBC)
 */
class ClassicPacketParsers : public TbcPacketParsers {
public:
    uint8_t movementFlags2Size() const override { return 0; }
    bool parseCharEnum(network::Packet& packet, CharEnumResponse& response) override;
    bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) override;
    void writeMovementPayload(network::Packet& packet, const MovementInfo& info) override;
    network::Packet buildMovementPacket(LogicalOpcode opcode,
                                         const MovementInfo& info,
                                         uint64_t playerGuid = 0) override;
    network::Packet buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) override;
    network::Packet buildUseItem(uint8_t bagIndex, uint8_t slotIndex, uint64_t itemGuid) override;
    bool parseCastFailed(network::Packet& packet, CastFailedData& data) override;
    bool parseMessageChat(network::Packet& packet, MessageChatData& data) override;
    bool parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) override;
    bool parseGossipMessage(network::Packet& packet, GossipMessageData& data) override;
    bool parseGuildRoster(network::Packet& packet, GuildRosterData& data) override;
    bool parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) override;
    network::Packet buildJoinChannel(const std::string& channelName, const std::string& password) override;
    network::Packet buildLeaveChannel(const std::string& channelName) override;
    network::Packet buildSendMail(uint64_t mailboxGuid, const std::string& recipient,
                                   const std::string& subject, const std::string& body,
                                   uint32_t money, uint32_t cod) override;
    bool parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox) override;
    network::Packet buildMailTakeItem(uint64_t mailboxGuid, uint32_t mailId, uint32_t itemSlot) override;
    network::Packet buildMailDelete(uint64_t mailboxGuid, uint32_t mailId, uint32_t mailTemplateId) override;
    network::Packet buildItemQuery(uint32_t entry, uint64_t guid) override;
    bool parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) override;
    uint8_t readQuestGiverStatus(network::Packet& packet) override;
    network::Packet buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) override;
    network::Packet buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) override;
    bool parseQuestDetails(network::Packet& packet, QuestDetailsData& data) override;
    uint8_t questLogStride() const override { return 3; }
    bool parseMonsterMove(network::Packet& packet, MonsterMoveData& data) override {
        return MonsterMoveParser::parseVanilla(packet, data);
    }
};

/**
 * Turtle WoW (build 7234) packet parsers.
 *
 * Turtle WoW is a heavily modified vanilla server that sends TBC-style
 * movement blocks (moveFlags2, transport timestamps, 8 speeds including flight)
 * while keeping all other Classic packet formats.
 *
 * Inherits all Classic overrides (charEnum, chat, gossip, mail, items, etc.)
 * but delegates movement block parsing to TBC format.
 */
class TurtlePacketParsers : public ClassicPacketParsers {
public:
    uint8_t movementFlags2Size() const override { return 0; }
    bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) override;
};

/**
 * Factory function to create the right parser set for an expansion.
 */
inline std::unique_ptr<PacketParsers> createPacketParsers(const std::string& expansionId) {
    if (expansionId == "classic") return std::make_unique<ClassicPacketParsers>();
    if (expansionId == "turtle") return std::make_unique<TurtlePacketParsers>();
    if (expansionId == "tbc") return std::make_unique<TbcPacketParsers>();
    return std::make_unique<WotlkPacketParsers>();
}

} // namespace game
} // namespace wowee
