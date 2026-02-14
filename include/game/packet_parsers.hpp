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
    bool parseMessageChat(network::Packet& packet, MessageChatData& data) override;
    bool parseGuildRoster(network::Packet& packet, GuildRosterData& data) override;
    bool parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) override;
};

/**
 * Factory function to create the right parser set for an expansion.
 */
inline std::unique_ptr<PacketParsers> createPacketParsers(const std::string& expansionId) {
    if (expansionId == "classic" || expansionId == "turtle") return std::make_unique<ClassicPacketParsers>();
    if (expansionId == "tbc") return std::make_unique<TbcPacketParsers>();
    return std::make_unique<WotlkPacketParsers>();
}

} // namespace game
} // namespace wowee
