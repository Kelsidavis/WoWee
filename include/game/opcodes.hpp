#pragma once

#include <cstdint>

namespace wowee {
namespace game {

// World of Warcraft 3.3.5a opcodes
// Values derived from community reverse-engineering efforts
// Reference: https://wowdev.wiki/World_Packet
enum class Opcode : uint16_t {
    // ---- Client to Server (Core) ----
    CMSG_PING                       = 0x1DC,
    CMSG_AUTH_SESSION                = 0x1ED,
    CMSG_CHAR_CREATE                 = 0x036,
    CMSG_CHAR_ENUM                   = 0x037,
    CMSG_CHAR_DELETE                 = 0x038,
    CMSG_PLAYER_LOGIN                = 0x03D,

    // ---- Movement ----
    CMSG_MOVE_START_FORWARD          = 0x0B5,
    CMSG_MOVE_START_BACKWARD         = 0x0B6,
    CMSG_MOVE_STOP                   = 0x0B7,
    CMSG_MOVE_START_STRAFE_LEFT      = 0x0B8,
    CMSG_MOVE_START_STRAFE_RIGHT     = 0x0B9,
    CMSG_MOVE_STOP_STRAFE            = 0x0BA,
    CMSG_MOVE_JUMP                   = 0x0BB,
    CMSG_MOVE_START_TURN_LEFT        = 0x0BC,
    CMSG_MOVE_START_TURN_RIGHT       = 0x0BD,
    CMSG_MOVE_STOP_TURN              = 0x0BE,
    CMSG_MOVE_SET_FACING             = 0x0DA,
    CMSG_MOVE_FALL_LAND              = 0x0C9,
    CMSG_MOVE_START_SWIM             = 0x0CA,
    CMSG_MOVE_STOP_SWIM              = 0x0CB,
    CMSG_MOVE_HEARTBEAT              = 0x0EE,

    // ---- Server to Client (Core) ----
    SMSG_AUTH_CHALLENGE              = 0x1EC,
    SMSG_AUTH_RESPONSE               = 0x1EE,
    SMSG_CHAR_CREATE                 = 0x03A,
    SMSG_CHAR_ENUM                   = 0x03B,
    SMSG_CHAR_DELETE                 = 0x03C,
    SMSG_PONG                        = 0x1DD,
    SMSG_LOGIN_VERIFY_WORLD          = 0x236,
    SMSG_ACCOUNT_DATA_TIMES          = 0x209,
    SMSG_FEATURE_SYSTEM_STATUS       = 0x3ED,
    SMSG_MOTD                        = 0x33D,

    // ---- Entity/Object updates ----
    SMSG_UPDATE_OBJECT               = 0x0A9,
    SMSG_COMPRESSED_UPDATE_OBJECT    = 0x1F6,
    SMSG_DESTROY_OBJECT              = 0x0AA,

    // ---- Chat ----
    CMSG_MESSAGECHAT                 = 0x095,
    SMSG_MESSAGECHAT                 = 0x096,

    // ---- Phase 1: Foundation (Targeting, Queries) ----
    CMSG_SET_SELECTION               = 0x13D,
    CMSG_NAME_QUERY                  = 0x050,
    SMSG_NAME_QUERY_RESPONSE         = 0x051,
    CMSG_CREATURE_QUERY              = 0x060,
    SMSG_CREATURE_QUERY_RESPONSE     = 0x061,
    CMSG_GAMEOBJECT_QUERY            = 0x05E,
    SMSG_GAMEOBJECT_QUERY_RESPONSE   = 0x05F,
    CMSG_SET_ACTIVE_MOVER            = 0x26A,

    // ---- XP ----
    SMSG_LOG_XPGAIN                  = 0x1D0,

    // ---- Phase 2: Combat Core ----
    CMSG_ATTACKSWING                 = 0x141,
    CMSG_ATTACKSTOP                  = 0x142,
    SMSG_ATTACKSTART                 = 0x143,
    SMSG_ATTACKSTOP                  = 0x144,
    SMSG_ATTACKERSTATEUPDATE         = 0x14A,
    SMSG_SPELLNONMELEEDAMAGELOG      = 0x250,
    SMSG_SPELLHEALLOG                = 0x150,
    SMSG_SPELLENERGIZELOG            = 0x25B,
    SMSG_PERIODICAURALOG             = 0x24E,
    SMSG_ENVIRONMENTALDAMAGELOG      = 0x1FC,

    // ---- Phase 3: Spells, Action Bar, Auras ----
    CMSG_CAST_SPELL                  = 0x12E,
    CMSG_CANCEL_CAST                 = 0x12F,
    CMSG_CANCEL_AURA                 = 0x033,
    SMSG_CAST_FAILED                 = 0x130,
    SMSG_SPELL_START                 = 0x131,
    SMSG_SPELL_GO                    = 0x132,
    SMSG_SPELL_FAILURE               = 0x133,
    SMSG_SPELL_COOLDOWN              = 0x134,
    SMSG_COOLDOWN_EVENT              = 0x135,
    SMSG_UPDATE_AURA_DURATION        = 0x137,
    SMSG_INITIAL_SPELLS              = 0x12A,
    SMSG_LEARNED_SPELL               = 0x12B,
    SMSG_REMOVED_SPELL               = 0x203,
    SMSG_SPELL_DELAYED               = 0x1E2,
    SMSG_AURA_UPDATE                 = 0x3FA,
    SMSG_AURA_UPDATE_ALL             = 0x495,
    SMSG_SET_FLAT_SPELL_MODIFIER     = 0x266,
    SMSG_SET_PCT_SPELL_MODIFIER      = 0x267,

    // ---- Phase 4: Group/Party ----
    CMSG_GROUP_INVITE                = 0x06E,
    SMSG_GROUP_INVITE                = 0x06F,
    CMSG_GROUP_ACCEPT                = 0x072,
    CMSG_GROUP_DECLINE               = 0x073,
    SMSG_GROUP_DECLINE               = 0x074,
    CMSG_GROUP_UNINVITE_GUID         = 0x076,
    SMSG_GROUP_UNINVITE              = 0x077,
    CMSG_GROUP_SET_LEADER            = 0x078,
    SMSG_GROUP_SET_LEADER            = 0x079,
    CMSG_GROUP_DISBAND               = 0x07B,
    SMSG_GROUP_LIST                  = 0x07D,
    SMSG_PARTY_COMMAND_RESULT        = 0x07E,
    MSG_RAID_TARGET_UPDATE           = 0x321,

    // ---- Phase 5: Loot ----
    CMSG_AUTOSTORE_LOOT_ITEM         = 0x108,
    CMSG_LOOT                        = 0x15D,
    CMSG_LOOT_MONEY                  = 0x15E,
    CMSG_LOOT_RELEASE                = 0x15F,
    SMSG_LOOT_RESPONSE               = 0x160,
    SMSG_LOOT_RELEASE_RESPONSE       = 0x161,
    SMSG_LOOT_REMOVED                = 0x162,
    SMSG_LOOT_MONEY_NOTIFY           = 0x163,
    SMSG_LOOT_CLEAR_MONEY            = 0x165,

    // ---- Phase 5: NPC Gossip ----
    CMSG_GOSSIP_HELLO                = 0x17B,
    CMSG_GOSSIP_SELECT_OPTION        = 0x17C,
    SMSG_GOSSIP_MESSAGE              = 0x17D,
    SMSG_GOSSIP_COMPLETE             = 0x17E,
    SMSG_NPC_TEXT_UPDATE             = 0x180,

    // ---- Phase 5: Vendor ----
    CMSG_LIST_INVENTORY              = 0x19E,
    SMSG_LIST_INVENTORY              = 0x19F,
    CMSG_SELL_ITEM                   = 0x1A0,
    SMSG_SELL_ITEM                   = 0x1A1,
    CMSG_BUY_ITEM                    = 0x1A2,
    SMSG_BUY_FAILED                  = 0x1A5,

    // ---- Phase 5: Item/Equip ----
    CMSG_ITEM_QUERY_SINGLE           = 0x056,
    SMSG_ITEM_QUERY_SINGLE_RESPONSE  = 0x058,
    CMSG_AUTOEQUIP_ITEM              = 0x10A,
    SMSG_INVENTORY_CHANGE_FAILURE    = 0x112,
};

} // namespace game
} // namespace wowee
