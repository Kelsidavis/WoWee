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
/// SMSG_CHAR_ENUM as Classic and TBC send it; see world_packets.cpp for why
/// WotLK is not folded in. hasEnchantment is the whole difference between the
/// two: TBC equipment entries carry a trailing uint32, Classic ones do not.
bool parseCharEnumPreWotlk(network::Packet& packet, CharEnumResponse& response,
                           bool hasEnchantment, const char* tag);

/// SMSG_ITEM_QUERY_SINGLE_RESPONSE as classic and TBC send it. The two differ
/// in one field: TBC carries a SoundOverrideSubclass after subClass.
bool parseItemQueryPreWotlk(network::Packet& packet, ItemQueryResponseData& data,
                            bool hasSoundOverrideSubclass, const char* tag);

/// The two questgiver packets as Classic and TBC send them: the NPC's GUID and
/// the quest id, and nothing else. WotLK appends a uint32 to each, so the
/// whole of the pre-WotLK case is what is absent.
///
/// Unlike the movement packet, which reads the same in both parsers and is not
/// the same, these two really are identical: they write their fields directly
/// rather than through a per-expansion payload writer.
network::Packet buildQueryQuestPacketPreWotlk(uint64_t npcGuid, uint32_t questId);
network::Packet buildAcceptQuestPacketPreWotlk(uint64_t npcGuid, uint32_t questId);

/// SMSG_SPELLNONMELEEDAMAGELOG as Classic and TBC send it.
///
/// The expansions differ by one field: WotLK carries an overkill amount after
/// the damage and these do not, so the pre-WotLK readers set it to zero
/// themselves. Between Classic and TBC the packet is byte for byte the same,
/// which is why one reader serves both - checked field by field rather than
/// assumed, since reading four bytes that are not there would take the school
/// mask out of the middle of the absorbed amount.
bool parseSpellDamageLogPreWotlk(network::Packet& packet, SpellDamageLogData& data,
                                 const char* tag);

class PacketParsers {
public:
    virtual ~PacketParsers() = default;

    // Size of MovementInfo.flags2 in bytes for MSG_MOVE_* payloads.
    // Classic: none, TBC: u8, WotLK: u16.
    [[nodiscard]] virtual uint8_t movementFlags2Size() const { return 2; }

    // Wire-format movement flag that gates transport data in MSG_MOVE_* payloads.
    // WotLK/TBC: 0x200, Classic/Turtle: 0x02000000.
    [[nodiscard]] virtual uint32_t wireOnTransportFlag() const { return 0x00000200; }

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

    /** Build CMSG_CAST_SPELL with SpellCastTargets targeting a game object. */
    virtual network::Packet buildCastGameObjectSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
        return CastSpellPacket::buildGameObjectTarget(spellId, targetGuid, castCount);
    }

    /** Build CMSG_CAST_SPELL with SpellCastTargets targeting an item - Disenchant,
     *  Prospecting, Milling and the enchant formulas. */
    virtual network::Packet buildCastSpellOnItem(uint32_t spellId, uint64_t itemGuid) {
        return CastSpellPacket::buildItemTarget(spellId, itemGuid, 0);
    }

    /** Build CMSG_USE_ITEM (WotLK default: bag + slot + castCount + spellId + itemGuid + glyphIndex + castFlags + targets)
     *  itemTargetGuid selects TARGET_FLAG_ITEM for spells that enchant another item. */
    virtual network::Packet buildUseItem(uint8_t bagIndex, uint8_t slotIndex,
                                         uint64_t itemGuid, uint32_t spellId = 0,
                                         uint64_t targetGuid = 0, uint64_t itemTargetGuid = 0) {
        return UseItemPacket::build(bagIndex, slotIndex, itemGuid, spellId, targetGuid, itemTargetGuid);
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

    /** Parse SMSG_SPELLHEALLOG */
    virtual bool parseSpellHealLog(network::Packet& packet, SpellHealLogData& data) {
        return SpellHealLogParser::parse(packet, data);
    }

    // --- Spells ---

    /** Parse SMSG_INITIAL_SPELLS */
    virtual bool parseInitialSpells(network::Packet& packet, InitialSpellsData& data) {
        return InitialSpellsParser::parse(packet, data);
    }

    /** Parse SMSG_SPELL_START */
    virtual bool parseSpellStart(network::Packet& packet, SpellStartData& data) {
        return SpellStartParser::parse(packet, data);
    }

    /** Parse SMSG_SPELL_GO */
    virtual bool parseSpellGo(network::Packet& packet, SpellGoData& data) {
        return SpellGoParser::parse(packet, data);
    }

    /** Parse SMSG_CAST_FAILED */
    virtual bool parseCastFailed(network::Packet& packet, CastFailedData& data) {
        return CastFailedParser::parse(packet, data);
    }

    /** Parse SMSG_CAST_RESULT header (spellId + result), expansion-aware.
     *  WotLK: castCount(u8) + spellId(u32) + result(u8)
     *  TBC/Classic: spellId(u32) + result(u8)  (no castCount prefix).
     *  Classic/TBC result enums have no SUCCESS entry, so parsers shift +1.
     *  miscArg/miscArg2 receive the trailing ids of spell-focus and totem
     *  failures (0 otherwise) - see readCastResultArgs.
     */
    virtual bool parseCastResult(network::Packet& packet, uint32_t& spellId, uint8_t& result,
                                 uint32_t& miscArg, uint32_t& miscArg2) {
        // WotLK default: skip castCount, read spellId + result
        if (packet.getSize() - packet.getReadPos() < 6) return false;
        packet.readUInt8();  // castCount
        spellId = packet.readUInt32();
        result  = packet.readUInt8();
        readCastResultArgs(packet, result, miscArg, miscArg2);
        return true;
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

    // --- Creature Query ---

    /** Parse SMSG_CREATURE_QUERY_RESPONSE */
    virtual bool parseCreatureQueryResponse(network::Packet& packet, CreatureQueryResponseData& data) {
        return CreatureQueryResponseParser::parse(packet, data);
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
    [[nodiscard]] virtual uint8_t questLogStride() const { return 5; }

    /** Number of PLAYER_EXPLORED_ZONES uint32 fields in update-object blocks.
     *  Classic/Vanilla/Turtle: 64 (bit-packs up to zone ID 2047).
     *  TBC/WotLK: 128 (covers Outland/Northrend zone IDs up to 4095). */
    [[nodiscard]] virtual uint8_t exploredZonesCount() const { return 128; }

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
                                           uint64_t money, uint64_t cod,
                                           const std::vector<uint64_t>& itemGuids = {}) {
        return SendMailPacket::build(mailboxGuid, recipient, subject, body, money, cod, itemGuids);
    }

    /** Parse SMSG_MAIL_LIST_RESULT into a vector of MailMessage */
    virtual bool parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox);

    /** Build CMSG_MAIL_TAKE_ITEM */
    virtual network::Packet buildMailTakeItem(uint64_t mailboxGuid, uint32_t mailId, uint32_t itemGuidLow) {
        return MailTakeItemPacket::build(mailboxGuid, mailId, itemGuidLow);
    }

    /** Build CMSG_MAIL_DELETE */
    virtual network::Packet buildMailDelete(uint64_t mailboxGuid, uint32_t mailId, uint32_t mailTemplateId) {
        return MailDeletePacket::build(mailboxGuid, mailId, mailTemplateId);
    }

    // --- Utility ---

    /** Read a packed GUID from the packet */
    virtual uint64_t readPackedGuid(network::Packet& packet) {
        return packet.readPackedGuid();
    }

    /** Write a packed GUID to the packet */
    virtual void writePackedGuid(network::Packet& packet, uint64_t guid) {
        packet.writePackedGuid(guid);
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
    [[nodiscard]] uint8_t movementFlags2Size() const override { return 1; }
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
    // TBC 2.4.3 CMSG_CAST_SPELL has no castFlags byte (WotLK added it)
    network::Packet buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) override;
    network::Packet buildCastGameObjectSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) override;
    // TBC 2.4.3 CMSG_USE_ITEM uses spellIndex + castCount + itemGuid + targets.
    network::Packet buildUseItem(uint8_t bagIndex, uint8_t slotIndex,
                                 uint64_t itemGuid, uint32_t spellId = 0,
                                 uint64_t targetGuid = 0, uint64_t itemTargetGuid = 0) override;
    // TBC 2.4.3 SMSG_MONSTER_MOVE has no unk byte after packed GUID (WotLK added it)
    bool parseMonsterMove(network::Packet& packet, MonsterMoveData& data) override;
    // TBC 2.4.3 SMSG_GOSSIP_MESSAGE quests lack questFlags(u32)+isRepeatable(u8) (WotLK added them)
    bool parseGossipMessage(network::Packet& packet, GossipMessageData& data) override;
    // TBC 2.4.3 SMSG_CAST_RESULT: spellId(u32) + result(u8) + castCount(u8)
    bool parseCastResult(network::Packet& packet, uint32_t& spellId, uint8_t& result,
                         uint32_t& miscArg, uint32_t& miscArg2) override;
    // TBC 2.4.3 SMSG_CAST_FAILED: spellId(u32) + result(u8) + castCount(u8)
    bool parseCastFailed(network::Packet& packet, CastFailedData& data) override;
    // TBC 2.4.3 SMSG_INITIAL_SPELLS: uint16 spellId + uint16 unk per entry.
    bool parseInitialSpells(network::Packet& packet, InitialSpellsData& data) override {
        return InitialSpellsParser::parse(packet, data, /*vanillaFormat=*/true);
    }
    // TBC 2.4.3 SMSG_SPELL_START: packed GUIDs + uint16 castFlags.
    bool parseSpellStart(network::Packet& packet, SpellStartData& data) override;
    // TBC 2.4.3 SMSG_SPELL_GO: packed caster GUIDs + uint16 castFlags + timestamp.
    bool parseSpellGo(network::Packet& packet, SpellGoData& data) override;
    // TBC 2.4.3 SMSG_MAIL_LIST_RESULT: uint8 count (not uint32+uint8), no body field,
    // attachment uses uint64 itemGuid (not uint32), enchants are 7×u32 id-only (not 7×{id+dur+charges})
    bool parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox) override;
    // TBC 2.4.3 SMSG_ATTACKERSTATEUPDATE uses packed GUIDs.
    bool parseAttackerStateUpdate(network::Packet& packet, AttackerStateUpdateData& data) override;
    // TBC 2.4.3 SMSG_SPELLNONMELEEDAMAGELOG uses packed GUIDs.
    bool parseSpellDamageLog(network::Packet& packet, SpellDamageLogData& data) override;
    // TBC 2.4.3 SMSG_SPELLHEALLOG uses packed GUIDs.
    bool parseSpellHealLog(network::Packet& packet, SpellHealLogData& data) override;
    // TBC 2.4.3 quest log has 4 update fields per slot (questId, state, counts, timer)
    // WotLK expands this to 5 (splits counts into two fields).
    [[nodiscard]] uint8_t questLogStride() const override { return 4; }
    // TBC 2.4.3 CMSG_QUESTGIVER_QUERY_QUEST: guid(8) + questId(4) - no trailing
    // isDialogContinued byte that WotLK added
    network::Packet buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) override;
    // TBC 2.4.3 SMSG_QUESTGIVER_QUEST_DETAILS (cmangos-tbc GossipDef.cpp):
    // u32 activateAccept + u32 suggestedPlayers, variable reward arrays, money,
    // then honor/spell/title trailing and the emote block LAST.
    bool parseQuestDetails(network::Packet& packet, QuestDetailsData& data) override {
        return parseQuestDetailsPreWotlk(packet, data, /*hasSuggestedPlayers=*/true);
    }
    // Shared vanilla/TBC quest-details layout; vanilla omits suggestedPlayers.
    static bool parseQuestDetailsPreWotlk(network::Packet& packet, QuestDetailsData& data,
                                          bool hasSuggestedPlayers);
    // TBC 2.4.3 SMSG_GUILD_ROSTER: same rank structure as WotLK (variable rankCount +
    // goldLimit + bank tabs), but NO gender byte per member (WotLK added it)
    bool parseGuildRoster(network::Packet& packet, GuildRosterData& data) override;
    // TBC 2.4.3 SMSG_GUILD_QUERY_RESPONSE has the Classic-era shape with no
    // trailing rankCount field.
    bool parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) override;
    // TBC 2.4.3 SMSG_QUESTGIVER_STATUS: uint32 status (WotLK uses uint8)
    uint8_t readQuestGiverStatus(network::Packet& packet) override;
    // TBC 2.4.3 SMSG_MESSAGECHAT shares the senderGuid + unknown + receiverGuid
    // whisper layout used by WotLK.
    bool parseMessageChat(network::Packet& packet, MessageChatData& data) override;
    // TBC 2.4.3 SMSG_GAMEOBJECT_QUERY_RESPONSE: 2 extra strings after names
    // (iconName + castBarCaption); WotLK has 3 (adds unk1)
    bool parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) override;
    // TBC 2.4.3 CMSG_JOIN_CHANNEL: name+password only (WotLK prepends channelId+hasVoice+joinedByZone)
    network::Packet buildJoinChannel(const std::string& channelName, const std::string& password) override;
    // TBC 2.4.3 CMSG_LEAVE_CHANNEL: name only (WotLK prepends channelId)
    network::Packet buildLeaveChannel(const std::string& channelName) override;
};

/**
 * Classic 1.12.1 packet parsers.
 *
 * Inherits from TBC (shared: u8 UpdateFlags, has_transport byte).
 *
 * Differences from TBC:
 * - No moveFlags2 byte (TBC has u8, Classic has none)
 * - Only 6 speed fields (no flight speeds - flying added in TBC)
 * - SPLINE_ENABLED at 0x00400000 (TBC/WotLK: 0x08000000)
 * - Transport data has no timestamp (TBC adds u32 timestamp)
 * - Pitch: only SWIMMING (no ONTRANSPORT secondary pitch)
 * - CharEnum: no enchantment field per equipment slot
 * - No SMSG_AURA_UPDATE (uses update fields, same as TBC)
 */
class ClassicPacketParsers : public TbcPacketParsers {
public:
    [[nodiscard]] uint8_t movementFlags2Size() const override { return 0; }
    [[nodiscard]] uint32_t wireOnTransportFlag() const override { return 0x02000000; }
    bool parseCharEnum(network::Packet& packet, CharEnumResponse& response) override;
    bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) override;
    void writeMovementPayload(network::Packet& packet, const MovementInfo& info) override;
    network::Packet buildMovementPacket(LogicalOpcode opcode,
                                         const MovementInfo& info,
                                         uint64_t playerGuid = 0) override;
    network::Packet buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) override;
    network::Packet buildCastGameObjectSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) override;
    network::Packet buildUseItem(uint8_t bagIndex, uint8_t slotIndex,
                                 uint64_t itemGuid, uint32_t spellId = 0,
                                 uint64_t targetGuid = 0, uint64_t itemTargetGuid = 0) override;
    bool parseCastFailed(network::Packet& packet, CastFailedData& data) override;
    bool parseCastResult(network::Packet& packet, uint32_t& spellId, uint8_t& result,
                         uint32_t& miscArg, uint32_t& miscArg2) override;
    bool parseMessageChat(network::Packet& packet, MessageChatData& data) override;
    bool parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) override;
    // Classic 1.12 SMSG_CREATURE_QUERY_RESPONSE lacks the iconName string that TBC/WotLK include
    bool parseCreatureQueryResponse(network::Packet& packet, CreatureQueryResponseData& data) override;
    bool parseGossipMessage(network::Packet& packet, GossipMessageData& data) override;
    bool parseGuildRoster(network::Packet& packet, GuildRosterData& data) override;
    bool parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) override;
    network::Packet buildJoinChannel(const std::string& channelName, const std::string& password) override;
    network::Packet buildLeaveChannel(const std::string& channelName) override;
    network::Packet buildSendMail(uint64_t mailboxGuid, const std::string& recipient,
                                   const std::string& subject, const std::string& body,
                                   uint64_t money, uint64_t cod,
                                   const std::vector<uint64_t>& itemGuids = {}) override;
    bool parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox) override;
    network::Packet buildMailTakeItem(uint64_t mailboxGuid, uint32_t mailId, uint32_t itemGuidLow) override;
    network::Packet buildMailDelete(uint64_t mailboxGuid, uint32_t mailId, uint32_t mailTemplateId) override;
    network::Packet buildItemQuery(uint32_t entry, uint64_t guid) override;
    bool parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) override;
    uint8_t readQuestGiverStatus(network::Packet& packet) override;
    network::Packet buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) override;
    network::Packet buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) override;
    // Classic 1.12 SMSG_QUESTGIVER_QUEST_DETAILS (vmangos Quest.cpp): like TBC
    // but WITHOUT the suggestedPlayers field after activateAccept.
    bool parseQuestDetails(network::Packet& packet, QuestDetailsData& data) override {
        return parseQuestDetailsPreWotlk(packet, data, /*hasSuggestedPlayers=*/false);
    }
    [[nodiscard]] uint8_t questLogStride() const override { return 3; }
    // Classic 1.12 has 64 explored-zone uint32 fields (zone IDs fit in 2048 bits).
    // TBC/WotLK use 128 (needed for Outland/Northrend zone IDs up to 4095).
    [[nodiscard]] uint8_t exploredZonesCount() const override { return 64; }
    bool parseMonsterMove(network::Packet& packet, MonsterMoveData& data) override {
        return MonsterMoveParser::parseVanilla(packet, data);
    }
    // Classic 1.12 SMSG_INITIAL_SPELLS: uint16 spellId + uint16 slot per entry (not uint32 + uint16)
    bool parseInitialSpells(network::Packet& packet, InitialSpellsData& data) override {
        return InitialSpellsParser::parse(packet, data, /*vanillaFormat=*/true);
    }
    // Classic 1.12 uses PackedGuid (not full uint64) and uint16 castFlags (not uint32)
    bool parseSpellStart(network::Packet& packet, SpellStartData& data) override;
    bool parseSpellGo(network::Packet& packet, SpellGoData& data) override;
    // Classic 1.12 melee/spell log packets use PackedGuid (not full uint64)
    bool parseAttackerStateUpdate(network::Packet& packet, AttackerStateUpdateData& data) override;
    bool parseSpellDamageLog(network::Packet& packet, SpellDamageLogData& data) override;
    bool parseSpellHealLog(network::Packet& packet, SpellHealLogData& data) override;
    // Classic 1.12 has SMSG_AURA_UPDATE (unlike TBC which doesn't);
    // format differs from WotLK: no caster GUID, DURATION flag is 0x10 not 0x20
    bool parseAuraUpdate(network::Packet& packet, AuraUpdateData& data, bool isAll = false) override;
    // Classic 1.12 SMSG_NAME_QUERY_RESPONSE: full uint64 guid + name + realmName CString +
    // uint32 race + uint32 gender + uint32 class (TBC Variant A skips the realmName CString)
    bool parseNameQueryResponse(network::Packet& packet, NameQueryResponseData& data) override;
};

/**
 * Turtle WoW packet parsers.
 *
 * Turtle is Classic-based but not wire-identical to vanilla MaNGOS. It keeps
 * most Classic packet formats, while overriding the movement-bearing paths that
 * have proven to vary in live traffic:
 * - update-object movement blocks use a Turtle-specific hybrid layout
 * - update-object parsing falls back through Classic/TBC/WotLK movement layouts
 * - monster-move parsing falls back through Vanilla, TBC, and guarded WotLK layouts
 *
 * Everything else inherits the Classic parser behavior.
 */
class TurtlePacketParsers : public ClassicPacketParsers {
public:
    [[nodiscard]] uint8_t movementFlags2Size() const override { return 0; }
    bool parseUpdateObject(network::Packet& packet, UpdateObjectData& data) override;
    bool parseMovementBlock(network::Packet& packet, UpdateBlock& block) override;
    bool parseMonsterMove(network::Packet& packet, MonsterMoveData& data) override;
};

/**
 * The parser set for an expansion id, or null if there is not one.
 *
 * Null rather than a WotLK fallback. This used to end in WotLK's parsers for
 * every id it did not recognise, so a profile whose wire format is not WotLK's
 * would authenticate, log in, and then misparse movement and update-object
 * rather than failing where the gap actually is. An expansion is a parser set
 * as much as it is a data directory, and this is the one place that says so.
 *
 * The caller decides what to do about it, because deciding here would mean
 * logging, and this header is pure enough that ten standalone tests include it.
 */
inline std::unique_ptr<PacketParsers> createPacketParsers(const std::string& expansionId) {
    if (expansionId == "classic") return std::make_unique<ClassicPacketParsers>();
    if (expansionId == "turtle") return std::make_unique<TurtlePacketParsers>();
    if (expansionId == "tbc") return std::make_unique<TbcPacketParsers>();
    if (expansionId == "wotlk") return std::make_unique<WotlkPacketParsers>();
    return nullptr;
}

} // namespace game
} // namespace wowee
