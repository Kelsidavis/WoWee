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

    // ---- Server Info Commands ----
    CMSG_WHO                         = 0x062,
    SMSG_WHO                         = 0x063,
    CMSG_REQUEST_PLAYED_TIME         = 0x1CC,
    SMSG_PLAYED_TIME                 = 0x1CD,
    CMSG_QUERY_TIME                  = 0x1CE,
    SMSG_QUERY_TIME_RESPONSE         = 0x1CF,

    // ---- Social Commands ----
    SMSG_FRIEND_STATUS               = 0x068,
    CMSG_ADD_FRIEND                  = 0x069,
    CMSG_DEL_FRIEND                  = 0x06A,
    CMSG_SET_CONTACT_NOTES           = 0x06B,
    CMSG_ADD_IGNORE                  = 0x06C,
    CMSG_DEL_IGNORE                  = 0x06D,

    // ---- Logout Commands ----
    CMSG_PLAYER_LOGOUT               = 0x04A,
    CMSG_LOGOUT_REQUEST              = 0x04B,
    CMSG_LOGOUT_CANCEL               = 0x04E,
    SMSG_LOGOUT_RESPONSE             = 0x04C,
    SMSG_LOGOUT_COMPLETE             = 0x04D,

    // ---- Stand State ----
    CMSG_STAND_STATE_CHANGE          = 0x101,

    // ---- Display Toggles ----
    CMSG_SHOWING_HELM                = 0x2B9,
    CMSG_SHOWING_CLOAK               = 0x2BA,

    // ---- PvP ----
    CMSG_TOGGLE_PVP                  = 0x253,

    // ---- Guild ----
    CMSG_GUILD_INVITE                = 0x082,
    CMSG_GUILD_ACCEPT                = 0x084,
    CMSG_GUILD_DECLINE_INVITATION    = 0x085,
    CMSG_GUILD_INFO                  = 0x087,
    CMSG_GUILD_GET_ROSTER            = 0x089,
    CMSG_GUILD_PROMOTE_MEMBER        = 0x08B,
    CMSG_GUILD_DEMOTE_MEMBER         = 0x08C,
    CMSG_GUILD_LEAVE                 = 0x08D,
    CMSG_GUILD_MOTD                  = 0x091,
    SMSG_GUILD_INFO                  = 0x088,
    SMSG_GUILD_ROSTER                = 0x08A,

    // ---- Ready Check ----
    MSG_RAID_READY_CHECK             = 0x322,
    MSG_RAID_READY_CHECK_CONFIRM     = 0x3AE,

    // ---- Duel ----
    CMSG_DUEL_PROPOSED               = 0x166,
    CMSG_DUEL_ACCEPTED               = 0x16C,
    CMSG_DUEL_CANCELLED              = 0x16D,
    SMSG_DUEL_REQUESTED              = 0x167,

    // ---- Trade ----
    CMSG_INITIATE_TRADE              = 0x116,

    // ---- Random Roll ----
    MSG_RANDOM_ROLL                  = 0x1FB,

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

    // ---- Creature Movement ----
    SMSG_MONSTER_MOVE                = 0x0DD,

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
    CMSG_REQUEST_RAID_INFO           = 0x2CD,
    SMSG_RAID_INSTANCE_INFO          = 0x2CC,

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

    // ---- Phase 5: Taxi / Flight Paths ----
    CMSG_ACTIVATETAXI                = 0x19D,

    // ---- Phase 5: NPC Gossip ----
    CMSG_GOSSIP_HELLO                = 0x17B,
    CMSG_GOSSIP_SELECT_OPTION        = 0x17C,
    SMSG_GOSSIP_MESSAGE              = 0x17D,
    SMSG_GOSSIP_COMPLETE             = 0x17E,
    SMSG_NPC_TEXT_UPDATE             = 0x180,

    // ---- Phase 5: GameObject ----
    CMSG_GAMEOBJECT_USE              = 0x01B,

    // ---- Phase 5: Quests ----
    CMSG_QUESTGIVER_STATUS_QUERY     = 0x182,
    SMSG_QUESTGIVER_STATUS           = 0x183,
    SMSG_QUESTGIVER_STATUS_MULTIPLE  = 0x198,
    CMSG_QUESTGIVER_HELLO            = 0x184,
    CMSG_QUESTGIVER_QUERY_QUEST      = 0x186,
    SMSG_QUESTGIVER_QUEST_DETAILS    = 0x188,
    CMSG_QUESTGIVER_ACCEPT_QUEST     = 0x189,
    CMSG_QUESTGIVER_COMPLETE_QUEST   = 0x18A,
    SMSG_QUESTGIVER_REQUEST_ITEMS    = 0x18B,
    CMSG_QUESTGIVER_REQUEST_REWARD   = 0x18C,
    SMSG_QUESTGIVER_OFFER_REWARD     = 0x18D,
    CMSG_QUESTGIVER_CHOOSE_REWARD    = 0x18E,
    SMSG_QUESTGIVER_QUEST_COMPLETE   = 0x191,
    CMSG_QUESTLOG_REMOVE_QUEST       = 0x194,

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
    CMSG_USE_ITEM                    = 0x00AB,
    CMSG_AUTOEQUIP_ITEM              = 0x10A,
    SMSG_INVENTORY_CHANGE_FAILURE    = 0x112,
    CMSG_INSPECT                     = 0x114,
    SMSG_INSPECT_RESULTS             = 0x115,

    // ---- Death/Respawn ----
    CMSG_REPOP_REQUEST               = 0x015A,
    CMSG_SPIRIT_HEALER_ACTIVATE      = 0x0176,
    SMSG_SPIRIT_HEALER_CONFIRM       = 0x0222,

    // ---- Teleport / Transfer ----
    MSG_MOVE_TELEPORT_ACK            = 0x0C7,
    SMSG_TRANSFER_PENDING            = 0x003F,

    // ---- Speed Changes ----
    SMSG_FORCE_RUN_SPEED_CHANGE      = 0x00E2,
    CMSG_FORCE_RUN_SPEED_CHANGE_ACK  = 0x00E3,

    // ---- Mount ----
    CMSG_CANCEL_MOUNT_AURA           = 0x0375,

    // ---- Taxi / Flight Paths ----
    SMSG_SHOWTAXINODES               = 0x01A9,
    SMSG_ACTIVATETAXIREPLY           = 0x01AE,
    SMSG_NEW_TAXI_PATH               = 0x01AF,
    CMSG_ACTIVATETAXIEXPRESS         = 0x0312,
};

} // namespace game
} // namespace wowee
