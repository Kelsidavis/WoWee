#include "game/opcode_table.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace wowee {
namespace game {

// Global active opcode table pointer
static const OpcodeTable* g_activeOpcodeTable = nullptr;

void setActiveOpcodeTable(const OpcodeTable* table) { g_activeOpcodeTable = table; }
const OpcodeTable* getActiveOpcodeTable() { return g_activeOpcodeTable; }

// Name ↔ LogicalOpcode mapping table (generated from the enum)
struct OpcodeNameEntry {
    const char* name;
    LogicalOpcode op;
};

// clang-format off
static const OpcodeNameEntry kOpcodeNames[] = {
    {"CMSG_PING", LogicalOpcode::CMSG_PING},
    {"CMSG_AUTH_SESSION", LogicalOpcode::CMSG_AUTH_SESSION},
    {"CMSG_CHAR_CREATE", LogicalOpcode::CMSG_CHAR_CREATE},
    {"CMSG_CHAR_ENUM", LogicalOpcode::CMSG_CHAR_ENUM},
    {"CMSG_CHAR_DELETE", LogicalOpcode::CMSG_CHAR_DELETE},
    {"CMSG_PLAYER_LOGIN", LogicalOpcode::CMSG_PLAYER_LOGIN},
    {"CMSG_MOVE_START_FORWARD", LogicalOpcode::CMSG_MOVE_START_FORWARD},
    {"CMSG_MOVE_START_BACKWARD", LogicalOpcode::CMSG_MOVE_START_BACKWARD},
    {"CMSG_MOVE_STOP", LogicalOpcode::CMSG_MOVE_STOP},
    {"CMSG_MOVE_START_STRAFE_LEFT", LogicalOpcode::CMSG_MOVE_START_STRAFE_LEFT},
    {"CMSG_MOVE_START_STRAFE_RIGHT", LogicalOpcode::CMSG_MOVE_START_STRAFE_RIGHT},
    {"CMSG_MOVE_STOP_STRAFE", LogicalOpcode::CMSG_MOVE_STOP_STRAFE},
    {"CMSG_MOVE_JUMP", LogicalOpcode::CMSG_MOVE_JUMP},
    {"CMSG_MOVE_START_TURN_LEFT", LogicalOpcode::CMSG_MOVE_START_TURN_LEFT},
    {"CMSG_MOVE_START_TURN_RIGHT", LogicalOpcode::CMSG_MOVE_START_TURN_RIGHT},
    {"CMSG_MOVE_STOP_TURN", LogicalOpcode::CMSG_MOVE_STOP_TURN},
    {"CMSG_MOVE_SET_FACING", LogicalOpcode::CMSG_MOVE_SET_FACING},
    {"CMSG_MOVE_FALL_LAND", LogicalOpcode::CMSG_MOVE_FALL_LAND},
    {"CMSG_MOVE_START_SWIM", LogicalOpcode::CMSG_MOVE_START_SWIM},
    {"CMSG_MOVE_STOP_SWIM", LogicalOpcode::CMSG_MOVE_STOP_SWIM},
    {"CMSG_MOVE_HEARTBEAT", LogicalOpcode::CMSG_MOVE_HEARTBEAT},
    {"SMSG_AUTH_CHALLENGE", LogicalOpcode::SMSG_AUTH_CHALLENGE},
    {"SMSG_AUTH_RESPONSE", LogicalOpcode::SMSG_AUTH_RESPONSE},
    {"SMSG_CHAR_CREATE", LogicalOpcode::SMSG_CHAR_CREATE},
    {"SMSG_CHAR_ENUM", LogicalOpcode::SMSG_CHAR_ENUM},
    {"SMSG_CHAR_DELETE", LogicalOpcode::SMSG_CHAR_DELETE},
    {"SMSG_PONG", LogicalOpcode::SMSG_PONG},
    {"SMSG_LOGIN_VERIFY_WORLD", LogicalOpcode::SMSG_LOGIN_VERIFY_WORLD},
    {"SMSG_LOGIN_SETTIMESPEED", LogicalOpcode::SMSG_LOGIN_SETTIMESPEED},
    {"SMSG_TUTORIAL_FLAGS", LogicalOpcode::SMSG_TUTORIAL_FLAGS},
    {"SMSG_WARDEN_DATA", LogicalOpcode::SMSG_WARDEN_DATA},
    {"CMSG_WARDEN_DATA", LogicalOpcode::CMSG_WARDEN_DATA},
    {"SMSG_ACCOUNT_DATA_TIMES", LogicalOpcode::SMSG_ACCOUNT_DATA_TIMES},
    {"SMSG_CLIENTCACHE_VERSION", LogicalOpcode::SMSG_CLIENTCACHE_VERSION},
    {"SMSG_FEATURE_SYSTEM_STATUS", LogicalOpcode::SMSG_FEATURE_SYSTEM_STATUS},
    {"SMSG_MOTD", LogicalOpcode::SMSG_MOTD},
    {"SMSG_UPDATE_OBJECT", LogicalOpcode::SMSG_UPDATE_OBJECT},
    {"SMSG_COMPRESSED_UPDATE_OBJECT", LogicalOpcode::SMSG_COMPRESSED_UPDATE_OBJECT},
    {"SMSG_MONSTER_MOVE_TRANSPORT", LogicalOpcode::SMSG_MONSTER_MOVE_TRANSPORT},
    {"SMSG_DESTROY_OBJECT", LogicalOpcode::SMSG_DESTROY_OBJECT},
    {"CMSG_MESSAGECHAT", LogicalOpcode::CMSG_MESSAGECHAT},
    {"SMSG_MESSAGECHAT", LogicalOpcode::SMSG_MESSAGECHAT},
    {"CMSG_WHO", LogicalOpcode::CMSG_WHO},
    {"SMSG_WHO", LogicalOpcode::SMSG_WHO},
    {"CMSG_REQUEST_PLAYED_TIME", LogicalOpcode::CMSG_REQUEST_PLAYED_TIME},
    {"SMSG_PLAYED_TIME", LogicalOpcode::SMSG_PLAYED_TIME},
    {"CMSG_QUERY_TIME", LogicalOpcode::CMSG_QUERY_TIME},
    {"SMSG_QUERY_TIME_RESPONSE", LogicalOpcode::SMSG_QUERY_TIME_RESPONSE},
    {"SMSG_FRIEND_STATUS", LogicalOpcode::SMSG_FRIEND_STATUS},
    {"CMSG_ADD_FRIEND", LogicalOpcode::CMSG_ADD_FRIEND},
    {"CMSG_DEL_FRIEND", LogicalOpcode::CMSG_DEL_FRIEND},
    {"CMSG_SET_CONTACT_NOTES", LogicalOpcode::CMSG_SET_CONTACT_NOTES},
    {"CMSG_ADD_IGNORE", LogicalOpcode::CMSG_ADD_IGNORE},
    {"CMSG_DEL_IGNORE", LogicalOpcode::CMSG_DEL_IGNORE},
    {"CMSG_PLAYER_LOGOUT", LogicalOpcode::CMSG_PLAYER_LOGOUT},
    {"CMSG_LOGOUT_REQUEST", LogicalOpcode::CMSG_LOGOUT_REQUEST},
    {"CMSG_LOGOUT_CANCEL", LogicalOpcode::CMSG_LOGOUT_CANCEL},
    {"SMSG_LOGOUT_RESPONSE", LogicalOpcode::SMSG_LOGOUT_RESPONSE},
    {"SMSG_LOGOUT_COMPLETE", LogicalOpcode::SMSG_LOGOUT_COMPLETE},
    {"CMSG_STAND_STATE_CHANGE", LogicalOpcode::CMSG_STAND_STATE_CHANGE},
    {"CMSG_SHOWING_HELM", LogicalOpcode::CMSG_SHOWING_HELM},
    {"CMSG_SHOWING_CLOAK", LogicalOpcode::CMSG_SHOWING_CLOAK},
    {"CMSG_TOGGLE_PVP", LogicalOpcode::CMSG_TOGGLE_PVP},
    {"CMSG_GUILD_INVITE", LogicalOpcode::CMSG_GUILD_INVITE},
    {"CMSG_GUILD_ACCEPT", LogicalOpcode::CMSG_GUILD_ACCEPT},
    {"CMSG_GUILD_DECLINE_INVITATION", LogicalOpcode::CMSG_GUILD_DECLINE_INVITATION},
    {"CMSG_GUILD_INFO", LogicalOpcode::CMSG_GUILD_INFO},
    {"CMSG_GUILD_GET_ROSTER", LogicalOpcode::CMSG_GUILD_GET_ROSTER},
    {"CMSG_GUILD_PROMOTE_MEMBER", LogicalOpcode::CMSG_GUILD_PROMOTE_MEMBER},
    {"CMSG_GUILD_DEMOTE_MEMBER", LogicalOpcode::CMSG_GUILD_DEMOTE_MEMBER},
    {"CMSG_GUILD_LEAVE", LogicalOpcode::CMSG_GUILD_LEAVE},
    {"CMSG_GUILD_MOTD", LogicalOpcode::CMSG_GUILD_MOTD},
    {"SMSG_GUILD_INFO", LogicalOpcode::SMSG_GUILD_INFO},
    {"SMSG_GUILD_ROSTER", LogicalOpcode::SMSG_GUILD_ROSTER},
    {"CMSG_GUILD_QUERY", LogicalOpcode::CMSG_GUILD_QUERY},
    {"SMSG_GUILD_QUERY_RESPONSE", LogicalOpcode::SMSG_GUILD_QUERY_RESPONSE},
    {"SMSG_GUILD_INVITE", LogicalOpcode::SMSG_GUILD_INVITE},
    {"CMSG_GUILD_REMOVE", LogicalOpcode::CMSG_GUILD_REMOVE},
    {"SMSG_GUILD_EVENT", LogicalOpcode::SMSG_GUILD_EVENT},
    {"SMSG_GUILD_COMMAND_RESULT", LogicalOpcode::SMSG_GUILD_COMMAND_RESULT},
    {"MSG_RAID_READY_CHECK", LogicalOpcode::MSG_RAID_READY_CHECK},
    {"MSG_RAID_READY_CHECK_CONFIRM", LogicalOpcode::MSG_RAID_READY_CHECK_CONFIRM},
    {"CMSG_DUEL_PROPOSED", LogicalOpcode::CMSG_DUEL_PROPOSED},
    {"CMSG_DUEL_ACCEPTED", LogicalOpcode::CMSG_DUEL_ACCEPTED},
    {"CMSG_DUEL_CANCELLED", LogicalOpcode::CMSG_DUEL_CANCELLED},
    {"SMSG_DUEL_REQUESTED", LogicalOpcode::SMSG_DUEL_REQUESTED},
    {"CMSG_INITIATE_TRADE", LogicalOpcode::CMSG_INITIATE_TRADE},
    {"MSG_RANDOM_ROLL", LogicalOpcode::MSG_RANDOM_ROLL},
    {"CMSG_SET_SELECTION", LogicalOpcode::CMSG_SET_SELECTION},
    {"CMSG_NAME_QUERY", LogicalOpcode::CMSG_NAME_QUERY},
    {"SMSG_NAME_QUERY_RESPONSE", LogicalOpcode::SMSG_NAME_QUERY_RESPONSE},
    {"CMSG_CREATURE_QUERY", LogicalOpcode::CMSG_CREATURE_QUERY},
    {"SMSG_CREATURE_QUERY_RESPONSE", LogicalOpcode::SMSG_CREATURE_QUERY_RESPONSE},
    {"CMSG_GAMEOBJECT_QUERY", LogicalOpcode::CMSG_GAMEOBJECT_QUERY},
    {"SMSG_GAMEOBJECT_QUERY_RESPONSE", LogicalOpcode::SMSG_GAMEOBJECT_QUERY_RESPONSE},
    {"CMSG_SET_ACTIVE_MOVER", LogicalOpcode::CMSG_SET_ACTIVE_MOVER},
    {"CMSG_BINDER_ACTIVATE", LogicalOpcode::CMSG_BINDER_ACTIVATE},
    {"SMSG_LOG_XPGAIN", LogicalOpcode::SMSG_LOG_XPGAIN},
    {"SMSG_MONSTER_MOVE", LogicalOpcode::SMSG_MONSTER_MOVE},
    {"CMSG_ATTACKSWING", LogicalOpcode::CMSG_ATTACKSWING},
    {"CMSG_ATTACKSTOP", LogicalOpcode::CMSG_ATTACKSTOP},
    {"SMSG_ATTACKSTART", LogicalOpcode::SMSG_ATTACKSTART},
    {"SMSG_ATTACKSTOP", LogicalOpcode::SMSG_ATTACKSTOP},
    {"SMSG_ATTACKERSTATEUPDATE", LogicalOpcode::SMSG_ATTACKERSTATEUPDATE},
    {"SMSG_SPELLNONMELEEDAMAGELOG", LogicalOpcode::SMSG_SPELLNONMELEEDAMAGELOG},
    {"SMSG_SPELLHEALLOG", LogicalOpcode::SMSG_SPELLHEALLOG},
    {"SMSG_SPELLENERGIZELOG", LogicalOpcode::SMSG_SPELLENERGIZELOG},
    {"SMSG_PERIODICAURALOG", LogicalOpcode::SMSG_PERIODICAURALOG},
    {"SMSG_ENVIRONMENTALDAMAGELOG", LogicalOpcode::SMSG_ENVIRONMENTALDAMAGELOG},
    {"CMSG_CAST_SPELL", LogicalOpcode::CMSG_CAST_SPELL},
    {"CMSG_CANCEL_CAST", LogicalOpcode::CMSG_CANCEL_CAST},
    {"CMSG_CANCEL_AURA", LogicalOpcode::CMSG_CANCEL_AURA},
    {"SMSG_CAST_FAILED", LogicalOpcode::SMSG_CAST_FAILED},
    {"SMSG_SPELL_START", LogicalOpcode::SMSG_SPELL_START},
    {"SMSG_SPELL_GO", LogicalOpcode::SMSG_SPELL_GO},
    {"SMSG_SPELL_FAILURE", LogicalOpcode::SMSG_SPELL_FAILURE},
    {"SMSG_SPELL_COOLDOWN", LogicalOpcode::SMSG_SPELL_COOLDOWN},
    {"SMSG_COOLDOWN_EVENT", LogicalOpcode::SMSG_COOLDOWN_EVENT},
    {"SMSG_UPDATE_AURA_DURATION", LogicalOpcode::SMSG_UPDATE_AURA_DURATION},
    {"SMSG_INITIAL_SPELLS", LogicalOpcode::SMSG_INITIAL_SPELLS},
    {"SMSG_LEARNED_SPELL", LogicalOpcode::SMSG_LEARNED_SPELL},
    {"SMSG_SUPERCEDED_SPELL", LogicalOpcode::SMSG_SUPERCEDED_SPELL},
    {"SMSG_REMOVED_SPELL", LogicalOpcode::SMSG_REMOVED_SPELL},
    {"SMSG_SEND_UNLEARN_SPELLS", LogicalOpcode::SMSG_SEND_UNLEARN_SPELLS},
    {"SMSG_SPELL_DELAYED", LogicalOpcode::SMSG_SPELL_DELAYED},
    {"SMSG_AURA_UPDATE", LogicalOpcode::SMSG_AURA_UPDATE},
    {"SMSG_AURA_UPDATE_ALL", LogicalOpcode::SMSG_AURA_UPDATE_ALL},
    {"SMSG_SET_FLAT_SPELL_MODIFIER", LogicalOpcode::SMSG_SET_FLAT_SPELL_MODIFIER},
    {"SMSG_SET_PCT_SPELL_MODIFIER", LogicalOpcode::SMSG_SET_PCT_SPELL_MODIFIER},
    {"SMSG_TALENTS_INFO", LogicalOpcode::SMSG_TALENTS_INFO},
    {"CMSG_LEARN_TALENT", LogicalOpcode::CMSG_LEARN_TALENT},
    {"MSG_TALENT_WIPE_CONFIRM", LogicalOpcode::MSG_TALENT_WIPE_CONFIRM},
    {"CMSG_GROUP_INVITE", LogicalOpcode::CMSG_GROUP_INVITE},
    {"SMSG_GROUP_INVITE", LogicalOpcode::SMSG_GROUP_INVITE},
    {"CMSG_GROUP_ACCEPT", LogicalOpcode::CMSG_GROUP_ACCEPT},
    {"CMSG_GROUP_DECLINE", LogicalOpcode::CMSG_GROUP_DECLINE},
    {"SMSG_GROUP_DECLINE", LogicalOpcode::SMSG_GROUP_DECLINE},
    {"CMSG_GROUP_UNINVITE_GUID", LogicalOpcode::CMSG_GROUP_UNINVITE_GUID},
    {"SMSG_GROUP_UNINVITE", LogicalOpcode::SMSG_GROUP_UNINVITE},
    {"CMSG_GROUP_SET_LEADER", LogicalOpcode::CMSG_GROUP_SET_LEADER},
    {"SMSG_GROUP_SET_LEADER", LogicalOpcode::SMSG_GROUP_SET_LEADER},
    {"CMSG_GROUP_DISBAND", LogicalOpcode::CMSG_GROUP_DISBAND},
    {"SMSG_GROUP_LIST", LogicalOpcode::SMSG_GROUP_LIST},
    {"SMSG_PARTY_COMMAND_RESULT", LogicalOpcode::SMSG_PARTY_COMMAND_RESULT},
    {"MSG_RAID_TARGET_UPDATE", LogicalOpcode::MSG_RAID_TARGET_UPDATE},
    {"CMSG_REQUEST_RAID_INFO", LogicalOpcode::CMSG_REQUEST_RAID_INFO},
    {"SMSG_RAID_INSTANCE_INFO", LogicalOpcode::SMSG_RAID_INSTANCE_INFO},
    {"CMSG_AUTOSTORE_LOOT_ITEM", LogicalOpcode::CMSG_AUTOSTORE_LOOT_ITEM},
    {"CMSG_LOOT", LogicalOpcode::CMSG_LOOT},
    {"CMSG_LOOT_MONEY", LogicalOpcode::CMSG_LOOT_MONEY},
    {"CMSG_LOOT_RELEASE", LogicalOpcode::CMSG_LOOT_RELEASE},
    {"SMSG_LOOT_RESPONSE", LogicalOpcode::SMSG_LOOT_RESPONSE},
    {"SMSG_LOOT_RELEASE_RESPONSE", LogicalOpcode::SMSG_LOOT_RELEASE_RESPONSE},
    {"SMSG_LOOT_REMOVED", LogicalOpcode::SMSG_LOOT_REMOVED},
    {"SMSG_LOOT_MONEY_NOTIFY", LogicalOpcode::SMSG_LOOT_MONEY_NOTIFY},
    {"SMSG_LOOT_CLEAR_MONEY", LogicalOpcode::SMSG_LOOT_CLEAR_MONEY},
    {"CMSG_ACTIVATETAXI", LogicalOpcode::CMSG_ACTIVATETAXI},
    {"CMSG_GOSSIP_HELLO", LogicalOpcode::CMSG_GOSSIP_HELLO},
    {"CMSG_GOSSIP_SELECT_OPTION", LogicalOpcode::CMSG_GOSSIP_SELECT_OPTION},
    {"SMSG_GOSSIP_MESSAGE", LogicalOpcode::SMSG_GOSSIP_MESSAGE},
    {"SMSG_GOSSIP_COMPLETE", LogicalOpcode::SMSG_GOSSIP_COMPLETE},
    {"SMSG_NPC_TEXT_UPDATE", LogicalOpcode::SMSG_NPC_TEXT_UPDATE},
    {"CMSG_GAMEOBJECT_USE", LogicalOpcode::CMSG_GAMEOBJECT_USE},
    {"CMSG_QUESTGIVER_STATUS_QUERY", LogicalOpcode::CMSG_QUESTGIVER_STATUS_QUERY},
    {"SMSG_QUESTGIVER_STATUS", LogicalOpcode::SMSG_QUESTGIVER_STATUS},
    {"SMSG_QUESTGIVER_STATUS_MULTIPLE", LogicalOpcode::SMSG_QUESTGIVER_STATUS_MULTIPLE},
    {"CMSG_QUESTGIVER_HELLO", LogicalOpcode::CMSG_QUESTGIVER_HELLO},
    {"CMSG_QUESTGIVER_QUERY_QUEST", LogicalOpcode::CMSG_QUESTGIVER_QUERY_QUEST},
    {"SMSG_QUESTGIVER_QUEST_DETAILS", LogicalOpcode::SMSG_QUESTGIVER_QUEST_DETAILS},
    {"CMSG_QUESTGIVER_ACCEPT_QUEST", LogicalOpcode::CMSG_QUESTGIVER_ACCEPT_QUEST},
    {"CMSG_QUESTGIVER_COMPLETE_QUEST", LogicalOpcode::CMSG_QUESTGIVER_COMPLETE_QUEST},
    {"SMSG_QUESTGIVER_REQUEST_ITEMS", LogicalOpcode::SMSG_QUESTGIVER_REQUEST_ITEMS},
    {"CMSG_QUESTGIVER_REQUEST_REWARD", LogicalOpcode::CMSG_QUESTGIVER_REQUEST_REWARD},
    {"SMSG_QUESTGIVER_OFFER_REWARD", LogicalOpcode::SMSG_QUESTGIVER_OFFER_REWARD},
    {"CMSG_QUESTGIVER_CHOOSE_REWARD", LogicalOpcode::CMSG_QUESTGIVER_CHOOSE_REWARD},
    {"SMSG_QUESTGIVER_QUEST_INVALID", LogicalOpcode::SMSG_QUESTGIVER_QUEST_INVALID},
    {"SMSG_QUESTGIVER_QUEST_COMPLETE", LogicalOpcode::SMSG_QUESTGIVER_QUEST_COMPLETE},
    {"CMSG_QUESTLOG_REMOVE_QUEST", LogicalOpcode::CMSG_QUESTLOG_REMOVE_QUEST},
    {"SMSG_QUESTUPDATE_ADD_KILL", LogicalOpcode::SMSG_QUESTUPDATE_ADD_KILL},
    {"SMSG_QUESTUPDATE_COMPLETE", LogicalOpcode::SMSG_QUESTUPDATE_COMPLETE},
    {"CMSG_QUEST_QUERY", LogicalOpcode::CMSG_QUEST_QUERY},
    {"SMSG_QUEST_QUERY_RESPONSE", LogicalOpcode::SMSG_QUEST_QUERY_RESPONSE},
    {"SMSG_QUESTLOG_FULL", LogicalOpcode::SMSG_QUESTLOG_FULL},
    {"CMSG_LIST_INVENTORY", LogicalOpcode::CMSG_LIST_INVENTORY},
    {"SMSG_LIST_INVENTORY", LogicalOpcode::SMSG_LIST_INVENTORY},
    {"CMSG_SELL_ITEM", LogicalOpcode::CMSG_SELL_ITEM},
    {"SMSG_SELL_ITEM", LogicalOpcode::SMSG_SELL_ITEM},
    {"CMSG_BUY_ITEM", LogicalOpcode::CMSG_BUY_ITEM},
    {"SMSG_BUY_FAILED", LogicalOpcode::SMSG_BUY_FAILED},
    {"CMSG_TRAINER_LIST", LogicalOpcode::CMSG_TRAINER_LIST},
    {"SMSG_TRAINER_LIST", LogicalOpcode::SMSG_TRAINER_LIST},
    {"CMSG_TRAINER_BUY_SPELL", LogicalOpcode::CMSG_TRAINER_BUY_SPELL},
    {"SMSG_TRAINER_BUY_FAILED", LogicalOpcode::SMSG_TRAINER_BUY_FAILED},
    {"CMSG_ITEM_QUERY_SINGLE", LogicalOpcode::CMSG_ITEM_QUERY_SINGLE},
    {"SMSG_ITEM_QUERY_SINGLE_RESPONSE", LogicalOpcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE},
    {"CMSG_USE_ITEM", LogicalOpcode::CMSG_USE_ITEM},
    {"CMSG_AUTOEQUIP_ITEM", LogicalOpcode::CMSG_AUTOEQUIP_ITEM},
    {"CMSG_SWAP_ITEM", LogicalOpcode::CMSG_SWAP_ITEM},
    {"CMSG_SWAP_INV_ITEM", LogicalOpcode::CMSG_SWAP_INV_ITEM},
    {"SMSG_INVENTORY_CHANGE_FAILURE", LogicalOpcode::SMSG_INVENTORY_CHANGE_FAILURE},
    {"CMSG_INSPECT", LogicalOpcode::CMSG_INSPECT},
    {"SMSG_INSPECT_RESULTS", LogicalOpcode::SMSG_INSPECT_RESULTS},
    {"CMSG_REPOP_REQUEST", LogicalOpcode::CMSG_REPOP_REQUEST},
    {"SMSG_RESURRECT_REQUEST", LogicalOpcode::SMSG_RESURRECT_REQUEST},
    {"CMSG_RESURRECT_RESPONSE", LogicalOpcode::CMSG_RESURRECT_RESPONSE},
    {"CMSG_SPIRIT_HEALER_ACTIVATE", LogicalOpcode::CMSG_SPIRIT_HEALER_ACTIVATE},
    {"SMSG_SPIRIT_HEALER_CONFIRM", LogicalOpcode::SMSG_SPIRIT_HEALER_CONFIRM},
    {"SMSG_RESURRECT_CANCEL", LogicalOpcode::SMSG_RESURRECT_CANCEL},
    {"MSG_MOVE_TELEPORT_ACK", LogicalOpcode::MSG_MOVE_TELEPORT_ACK},
    {"SMSG_TRANSFER_PENDING", LogicalOpcode::SMSG_TRANSFER_PENDING},
    {"SMSG_NEW_WORLD", LogicalOpcode::SMSG_NEW_WORLD},
    {"MSG_MOVE_WORLDPORT_ACK", LogicalOpcode::MSG_MOVE_WORLDPORT_ACK},
    {"SMSG_TRANSFER_ABORTED", LogicalOpcode::SMSG_TRANSFER_ABORTED},
    {"SMSG_FORCE_RUN_SPEED_CHANGE", LogicalOpcode::SMSG_FORCE_RUN_SPEED_CHANGE},
    {"CMSG_FORCE_RUN_SPEED_CHANGE_ACK", LogicalOpcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK},
    {"CMSG_CANCEL_MOUNT_AURA", LogicalOpcode::CMSG_CANCEL_MOUNT_AURA},
    {"SMSG_SHOWTAXINODES", LogicalOpcode::SMSG_SHOWTAXINODES},
    {"SMSG_ACTIVATETAXIREPLY", LogicalOpcode::SMSG_ACTIVATETAXIREPLY},
    {"SMSG_ACTIVATETAXIREPLY_ALT", LogicalOpcode::SMSG_ACTIVATETAXIREPLY_ALT},
    {"SMSG_NEW_TAXI_PATH", LogicalOpcode::SMSG_NEW_TAXI_PATH},
    {"CMSG_ACTIVATETAXIEXPRESS", LogicalOpcode::CMSG_ACTIVATETAXIEXPRESS},
    {"SMSG_BATTLEFIELD_PORT_DENIED", LogicalOpcode::SMSG_BATTLEFIELD_PORT_DENIED},
    {"SMSG_REMOVED_FROM_PVP_QUEUE", LogicalOpcode::SMSG_REMOVED_FROM_PVP_QUEUE},
    {"SMSG_TRAINER_BUY_SUCCEEDED", LogicalOpcode::SMSG_TRAINER_BUY_SUCCEEDED},
    {"SMSG_BINDPOINTUPDATE", LogicalOpcode::SMSG_BINDPOINTUPDATE},
    {"CMSG_BATTLEFIELD_LIST", LogicalOpcode::CMSG_BATTLEFIELD_LIST},
    {"SMSG_BATTLEFIELD_LIST", LogicalOpcode::SMSG_BATTLEFIELD_LIST},
    {"CMSG_BATTLEFIELD_JOIN", LogicalOpcode::CMSG_BATTLEFIELD_JOIN},
    {"CMSG_BATTLEFIELD_STATUS", LogicalOpcode::CMSG_BATTLEFIELD_STATUS},
    {"SMSG_BATTLEFIELD_STATUS", LogicalOpcode::SMSG_BATTLEFIELD_STATUS},
    {"CMSG_BATTLEFIELD_PORT", LogicalOpcode::CMSG_BATTLEFIELD_PORT},
    {"CMSG_BATTLEMASTER_HELLO", LogicalOpcode::CMSG_BATTLEMASTER_HELLO},
    {"MSG_PVP_LOG_DATA", LogicalOpcode::MSG_PVP_LOG_DATA},
    {"CMSG_LEAVE_BATTLEFIELD", LogicalOpcode::CMSG_LEAVE_BATTLEFIELD},
    {"SMSG_GROUP_JOINED_BATTLEGROUND", LogicalOpcode::SMSG_GROUP_JOINED_BATTLEGROUND},
    {"MSG_BATTLEGROUND_PLAYER_POSITIONS", LogicalOpcode::MSG_BATTLEGROUND_PLAYER_POSITIONS},
    {"SMSG_BATTLEGROUND_PLAYER_JOINED", LogicalOpcode::SMSG_BATTLEGROUND_PLAYER_JOINED},
    {"SMSG_BATTLEGROUND_PLAYER_LEFT", LogicalOpcode::SMSG_BATTLEGROUND_PLAYER_LEFT},
    {"CMSG_BATTLEMASTER_JOIN", LogicalOpcode::CMSG_BATTLEMASTER_JOIN},
    {"SMSG_JOINED_BATTLEGROUND_QUEUE", LogicalOpcode::SMSG_JOINED_BATTLEGROUND_QUEUE},
    {"CMSG_ARENA_TEAM_CREATE", LogicalOpcode::CMSG_ARENA_TEAM_CREATE},
    {"SMSG_ARENA_TEAM_COMMAND_RESULT", LogicalOpcode::SMSG_ARENA_TEAM_COMMAND_RESULT},
    {"CMSG_ARENA_TEAM_QUERY", LogicalOpcode::CMSG_ARENA_TEAM_QUERY},
    {"SMSG_ARENA_TEAM_QUERY_RESPONSE", LogicalOpcode::SMSG_ARENA_TEAM_QUERY_RESPONSE},
    {"CMSG_ARENA_TEAM_ROSTER", LogicalOpcode::CMSG_ARENA_TEAM_ROSTER},
    {"SMSG_ARENA_TEAM_ROSTER", LogicalOpcode::SMSG_ARENA_TEAM_ROSTER},
    {"CMSG_ARENA_TEAM_INVITE", LogicalOpcode::CMSG_ARENA_TEAM_INVITE},
    {"SMSG_ARENA_TEAM_INVITE", LogicalOpcode::SMSG_ARENA_TEAM_INVITE},
    {"CMSG_ARENA_TEAM_ACCEPT", LogicalOpcode::CMSG_ARENA_TEAM_ACCEPT},
    {"CMSG_ARENA_TEAM_DECLINE", LogicalOpcode::CMSG_ARENA_TEAM_DECLINE},
    {"CMSG_ARENA_TEAM_LEAVE", LogicalOpcode::CMSG_ARENA_TEAM_LEAVE},
    {"CMSG_ARENA_TEAM_REMOVE", LogicalOpcode::CMSG_ARENA_TEAM_REMOVE},
    {"CMSG_ARENA_TEAM_DISBAND", LogicalOpcode::CMSG_ARENA_TEAM_DISBAND},
    {"CMSG_ARENA_TEAM_LEADER", LogicalOpcode::CMSG_ARENA_TEAM_LEADER},
    {"SMSG_ARENA_TEAM_EVENT", LogicalOpcode::SMSG_ARENA_TEAM_EVENT},
    {"CMSG_BATTLEMASTER_JOIN_ARENA", LogicalOpcode::CMSG_BATTLEMASTER_JOIN_ARENA},
    {"SMSG_ARENA_TEAM_STATS", LogicalOpcode::SMSG_ARENA_TEAM_STATS},
    {"SMSG_ARENA_ERROR", LogicalOpcode::SMSG_ARENA_ERROR},
    {"MSG_INSPECT_ARENA_TEAMS", LogicalOpcode::MSG_INSPECT_ARENA_TEAMS},
};
// clang-format on

static constexpr size_t kOpcodeNameCount = sizeof(kOpcodeNames) / sizeof(kOpcodeNames[0]);

std::optional<LogicalOpcode> OpcodeTable::nameToLogical(const std::string& name) {
    for (size_t i = 0; i < kOpcodeNameCount; ++i) {
        if (name == kOpcodeNames[i].name) return kOpcodeNames[i].op;
    }
    return std::nullopt;
}

const char* OpcodeTable::logicalToName(LogicalOpcode op) {
    uint16_t val = static_cast<uint16_t>(op);
    for (size_t i = 0; i < kOpcodeNameCount; ++i) {
        if (static_cast<uint16_t>(kOpcodeNames[i].op) == val) return kOpcodeNames[i].name;
    }
    return "UNKNOWN";
}

void OpcodeTable::loadWotlkDefaults() {
    // WotLK 3.3.5a wire values — matches the original hardcoded Opcode enum
    struct { LogicalOpcode op; uint16_t wire; } defaults[] = {
        {LogicalOpcode::CMSG_PING, 0x1DC},
        {LogicalOpcode::CMSG_AUTH_SESSION, 0x1ED},
        {LogicalOpcode::CMSG_CHAR_CREATE, 0x036},
        {LogicalOpcode::CMSG_CHAR_ENUM, 0x037},
        {LogicalOpcode::CMSG_CHAR_DELETE, 0x038},
        {LogicalOpcode::CMSG_PLAYER_LOGIN, 0x03D},
        {LogicalOpcode::CMSG_MOVE_START_FORWARD, 0x0B5},
        {LogicalOpcode::CMSG_MOVE_START_BACKWARD, 0x0B6},
        {LogicalOpcode::CMSG_MOVE_STOP, 0x0B7},
        {LogicalOpcode::CMSG_MOVE_START_STRAFE_LEFT, 0x0B8},
        {LogicalOpcode::CMSG_MOVE_START_STRAFE_RIGHT, 0x0B9},
        {LogicalOpcode::CMSG_MOVE_STOP_STRAFE, 0x0BA},
        {LogicalOpcode::CMSG_MOVE_JUMP, 0x0BB},
        {LogicalOpcode::CMSG_MOVE_START_TURN_LEFT, 0x0BC},
        {LogicalOpcode::CMSG_MOVE_START_TURN_RIGHT, 0x0BD},
        {LogicalOpcode::CMSG_MOVE_STOP_TURN, 0x0BE},
        {LogicalOpcode::CMSG_MOVE_SET_FACING, 0x0DA},
        {LogicalOpcode::CMSG_MOVE_FALL_LAND, 0x0C9},
        {LogicalOpcode::CMSG_MOVE_START_SWIM, 0x0CA},
        {LogicalOpcode::CMSG_MOVE_STOP_SWIM, 0x0CB},
        {LogicalOpcode::CMSG_MOVE_HEARTBEAT, 0x0EE},
        {LogicalOpcode::SMSG_AUTH_CHALLENGE, 0x1EC},
        {LogicalOpcode::SMSG_AUTH_RESPONSE, 0x1EE},
        {LogicalOpcode::SMSG_CHAR_CREATE, 0x03A},
        {LogicalOpcode::SMSG_CHAR_ENUM, 0x03B},
        {LogicalOpcode::SMSG_CHAR_DELETE, 0x03C},
        {LogicalOpcode::SMSG_PONG, 0x1DD},
        {LogicalOpcode::SMSG_LOGIN_VERIFY_WORLD, 0x236},
        {LogicalOpcode::SMSG_LOGIN_SETTIMESPEED, 0x042},
        {LogicalOpcode::SMSG_TUTORIAL_FLAGS, 0x0FD},
        {LogicalOpcode::SMSG_WARDEN_DATA, 0x2E6},
        {LogicalOpcode::CMSG_WARDEN_DATA, 0x2E7},
        {LogicalOpcode::SMSG_ACCOUNT_DATA_TIMES, 0x209},
        {LogicalOpcode::SMSG_CLIENTCACHE_VERSION, 0x4AB},
        {LogicalOpcode::SMSG_FEATURE_SYSTEM_STATUS, 0x3ED},
        {LogicalOpcode::SMSG_MOTD, 0x33D},
        {LogicalOpcode::SMSG_UPDATE_OBJECT, 0x0A9},
        {LogicalOpcode::SMSG_COMPRESSED_UPDATE_OBJECT, 0x1F6},
        {LogicalOpcode::SMSG_MONSTER_MOVE_TRANSPORT, 0x2AE},
        {LogicalOpcode::SMSG_DESTROY_OBJECT, 0x0AA},
        {LogicalOpcode::CMSG_MESSAGECHAT, 0x095},
        {LogicalOpcode::SMSG_MESSAGECHAT, 0x096},
        {LogicalOpcode::CMSG_WHO, 0x062},
        {LogicalOpcode::SMSG_WHO, 0x063},
        {LogicalOpcode::CMSG_REQUEST_PLAYED_TIME, 0x1CC},
        {LogicalOpcode::SMSG_PLAYED_TIME, 0x1CD},
        {LogicalOpcode::CMSG_QUERY_TIME, 0x1CE},
        {LogicalOpcode::SMSG_QUERY_TIME_RESPONSE, 0x1CF},
        {LogicalOpcode::SMSG_FRIEND_STATUS, 0x068},
        {LogicalOpcode::CMSG_ADD_FRIEND, 0x069},
        {LogicalOpcode::CMSG_DEL_FRIEND, 0x06A},
        {LogicalOpcode::CMSG_SET_CONTACT_NOTES, 0x06B},
        {LogicalOpcode::CMSG_ADD_IGNORE, 0x06C},
        {LogicalOpcode::CMSG_DEL_IGNORE, 0x06D},
        {LogicalOpcode::CMSG_PLAYER_LOGOUT, 0x04A},
        {LogicalOpcode::CMSG_LOGOUT_REQUEST, 0x04B},
        {LogicalOpcode::CMSG_LOGOUT_CANCEL, 0x04E},
        {LogicalOpcode::SMSG_LOGOUT_RESPONSE, 0x04C},
        {LogicalOpcode::SMSG_LOGOUT_COMPLETE, 0x04D},
        {LogicalOpcode::CMSG_STAND_STATE_CHANGE, 0x101},
        {LogicalOpcode::CMSG_SHOWING_HELM, 0x2B9},
        {LogicalOpcode::CMSG_SHOWING_CLOAK, 0x2BA},
        {LogicalOpcode::CMSG_TOGGLE_PVP, 0x253},
        {LogicalOpcode::CMSG_GUILD_INVITE, 0x082},
        {LogicalOpcode::CMSG_GUILD_ACCEPT, 0x084},
        {LogicalOpcode::CMSG_GUILD_DECLINE_INVITATION, 0x085},
        {LogicalOpcode::CMSG_GUILD_INFO, 0x087},
        {LogicalOpcode::CMSG_GUILD_GET_ROSTER, 0x089},
        {LogicalOpcode::CMSG_GUILD_PROMOTE_MEMBER, 0x08B},
        {LogicalOpcode::CMSG_GUILD_DEMOTE_MEMBER, 0x08C},
        {LogicalOpcode::CMSG_GUILD_LEAVE, 0x08D},
        {LogicalOpcode::CMSG_GUILD_MOTD, 0x091},
        {LogicalOpcode::SMSG_GUILD_INFO, 0x088},
        {LogicalOpcode::SMSG_GUILD_ROSTER, 0x08A},
        {LogicalOpcode::CMSG_GUILD_QUERY, 0x051},
        {LogicalOpcode::SMSG_GUILD_QUERY_RESPONSE, 0x052},
        {LogicalOpcode::SMSG_GUILD_INVITE, 0x083},
        {LogicalOpcode::CMSG_GUILD_REMOVE, 0x08E},
        {LogicalOpcode::SMSG_GUILD_EVENT, 0x092},
        {LogicalOpcode::SMSG_GUILD_COMMAND_RESULT, 0x093},
        {LogicalOpcode::MSG_RAID_READY_CHECK, 0x322},
        {LogicalOpcode::MSG_RAID_READY_CHECK_CONFIRM, 0x3AE},
        {LogicalOpcode::CMSG_DUEL_PROPOSED, 0x166},
        {LogicalOpcode::CMSG_DUEL_ACCEPTED, 0x16C},
        {LogicalOpcode::CMSG_DUEL_CANCELLED, 0x16D},
        {LogicalOpcode::SMSG_DUEL_REQUESTED, 0x167},
        {LogicalOpcode::CMSG_INITIATE_TRADE, 0x116},
        {LogicalOpcode::MSG_RANDOM_ROLL, 0x1FB},
        {LogicalOpcode::CMSG_SET_SELECTION, 0x13D},
        {LogicalOpcode::CMSG_NAME_QUERY, 0x050},
        {LogicalOpcode::SMSG_NAME_QUERY_RESPONSE, 0x051},
        {LogicalOpcode::CMSG_CREATURE_QUERY, 0x060},
        {LogicalOpcode::SMSG_CREATURE_QUERY_RESPONSE, 0x061},
        {LogicalOpcode::CMSG_GAMEOBJECT_QUERY, 0x05E},
        {LogicalOpcode::SMSG_GAMEOBJECT_QUERY_RESPONSE, 0x05F},
        {LogicalOpcode::CMSG_SET_ACTIVE_MOVER, 0x26A},
        {LogicalOpcode::CMSG_BINDER_ACTIVATE, 0x1B5},
        {LogicalOpcode::SMSG_LOG_XPGAIN, 0x1D0},
        {LogicalOpcode::SMSG_MONSTER_MOVE, 0x0DD},
        {LogicalOpcode::CMSG_ATTACKSWING, 0x141},
        {LogicalOpcode::CMSG_ATTACKSTOP, 0x142},
        {LogicalOpcode::SMSG_ATTACKSTART, 0x143},
        {LogicalOpcode::SMSG_ATTACKSTOP, 0x144},
        {LogicalOpcode::SMSG_ATTACKERSTATEUPDATE, 0x14A},
        {LogicalOpcode::SMSG_SPELLNONMELEEDAMAGELOG, 0x250},
        {LogicalOpcode::SMSG_SPELLHEALLOG, 0x150},
        {LogicalOpcode::SMSG_SPELLENERGIZELOG, 0x25B},
        {LogicalOpcode::SMSG_PERIODICAURALOG, 0x24E},
        {LogicalOpcode::SMSG_ENVIRONMENTALDAMAGELOG, 0x1FC},
        {LogicalOpcode::CMSG_CAST_SPELL, 0x12E},
        {LogicalOpcode::CMSG_CANCEL_CAST, 0x12F},
        {LogicalOpcode::CMSG_CANCEL_AURA, 0x033},
        {LogicalOpcode::SMSG_CAST_FAILED, 0x130},
        {LogicalOpcode::SMSG_SPELL_START, 0x131},
        {LogicalOpcode::SMSG_SPELL_GO, 0x132},
        {LogicalOpcode::SMSG_SPELL_FAILURE, 0x133},
        {LogicalOpcode::SMSG_SPELL_COOLDOWN, 0x134},
        {LogicalOpcode::SMSG_COOLDOWN_EVENT, 0x135},
        {LogicalOpcode::SMSG_UPDATE_AURA_DURATION, 0x137},
        {LogicalOpcode::SMSG_INITIAL_SPELLS, 0x12A},
        {LogicalOpcode::SMSG_LEARNED_SPELL, 0x12B},
        {LogicalOpcode::SMSG_SUPERCEDED_SPELL, 0x12C},
        {LogicalOpcode::SMSG_REMOVED_SPELL, 0x203},
        {LogicalOpcode::SMSG_SEND_UNLEARN_SPELLS, 0x41F},
        {LogicalOpcode::SMSG_SPELL_DELAYED, 0x1E2},
        {LogicalOpcode::SMSG_AURA_UPDATE, 0x3FA},
        {LogicalOpcode::SMSG_AURA_UPDATE_ALL, 0x495},
        {LogicalOpcode::SMSG_SET_FLAT_SPELL_MODIFIER, 0x266},
        {LogicalOpcode::SMSG_SET_PCT_SPELL_MODIFIER, 0x267},
        {LogicalOpcode::SMSG_TALENTS_INFO, 0x4C0},
        {LogicalOpcode::CMSG_LEARN_TALENT, 0x251},
        {LogicalOpcode::MSG_TALENT_WIPE_CONFIRM, 0x2AB},
        {LogicalOpcode::CMSG_GROUP_INVITE, 0x06E},
        {LogicalOpcode::SMSG_GROUP_INVITE, 0x06F},
        {LogicalOpcode::CMSG_GROUP_ACCEPT, 0x072},
        {LogicalOpcode::CMSG_GROUP_DECLINE, 0x073},
        {LogicalOpcode::SMSG_GROUP_DECLINE, 0x074},
        {LogicalOpcode::CMSG_GROUP_UNINVITE_GUID, 0x076},
        {LogicalOpcode::SMSG_GROUP_UNINVITE, 0x077},
        {LogicalOpcode::CMSG_GROUP_SET_LEADER, 0x078},
        {LogicalOpcode::SMSG_GROUP_SET_LEADER, 0x079},
        {LogicalOpcode::CMSG_GROUP_DISBAND, 0x07B},
        {LogicalOpcode::SMSG_GROUP_LIST, 0x07D},
        {LogicalOpcode::SMSG_PARTY_COMMAND_RESULT, 0x07E},
        {LogicalOpcode::MSG_RAID_TARGET_UPDATE, 0x321},
        {LogicalOpcode::CMSG_REQUEST_RAID_INFO, 0x2CD},
        {LogicalOpcode::SMSG_RAID_INSTANCE_INFO, 0x2CC},
        {LogicalOpcode::CMSG_AUTOSTORE_LOOT_ITEM, 0x108},
        {LogicalOpcode::CMSG_LOOT, 0x15D},
        {LogicalOpcode::CMSG_LOOT_MONEY, 0x15E},
        {LogicalOpcode::CMSG_LOOT_RELEASE, 0x15F},
        {LogicalOpcode::SMSG_LOOT_RESPONSE, 0x160},
        {LogicalOpcode::SMSG_LOOT_RELEASE_RESPONSE, 0x161},
        {LogicalOpcode::SMSG_LOOT_REMOVED, 0x162},
        {LogicalOpcode::SMSG_LOOT_MONEY_NOTIFY, 0x163},
        {LogicalOpcode::SMSG_LOOT_CLEAR_MONEY, 0x165},
        {LogicalOpcode::CMSG_ACTIVATETAXI, 0x19D},
        {LogicalOpcode::CMSG_GOSSIP_HELLO, 0x17B},
        {LogicalOpcode::CMSG_GOSSIP_SELECT_OPTION, 0x17C},
        {LogicalOpcode::SMSG_GOSSIP_MESSAGE, 0x17D},
        {LogicalOpcode::SMSG_GOSSIP_COMPLETE, 0x17E},
        {LogicalOpcode::SMSG_NPC_TEXT_UPDATE, 0x180},
        {LogicalOpcode::CMSG_GAMEOBJECT_USE, 0x01B},
        {LogicalOpcode::CMSG_QUESTGIVER_STATUS_QUERY, 0x182},
        {LogicalOpcode::SMSG_QUESTGIVER_STATUS, 0x183},
        {LogicalOpcode::SMSG_QUESTGIVER_STATUS_MULTIPLE, 0x198},
        {LogicalOpcode::CMSG_QUESTGIVER_HELLO, 0x184},
        {LogicalOpcode::CMSG_QUESTGIVER_QUERY_QUEST, 0x186},
        {LogicalOpcode::SMSG_QUESTGIVER_QUEST_DETAILS, 0x188},
        {LogicalOpcode::CMSG_QUESTGIVER_ACCEPT_QUEST, 0x189},
        {LogicalOpcode::CMSG_QUESTGIVER_COMPLETE_QUEST, 0x18A},
        {LogicalOpcode::SMSG_QUESTGIVER_REQUEST_ITEMS, 0x18B},
        {LogicalOpcode::CMSG_QUESTGIVER_REQUEST_REWARD, 0x18C},
        {LogicalOpcode::SMSG_QUESTGIVER_OFFER_REWARD, 0x18D},
        {LogicalOpcode::CMSG_QUESTGIVER_CHOOSE_REWARD, 0x18E},
        {LogicalOpcode::SMSG_QUESTGIVER_QUEST_INVALID, 0x18F},
        {LogicalOpcode::SMSG_QUESTGIVER_QUEST_COMPLETE, 0x191},
        {LogicalOpcode::CMSG_QUESTLOG_REMOVE_QUEST, 0x194},
        {LogicalOpcode::SMSG_QUESTUPDATE_ADD_KILL, 0x196},
        {LogicalOpcode::SMSG_QUESTUPDATE_COMPLETE, 0x195},
        {LogicalOpcode::CMSG_QUEST_QUERY, 0x05C},
        {LogicalOpcode::SMSG_QUEST_QUERY_RESPONSE, 0x05D},
        {LogicalOpcode::SMSG_QUESTLOG_FULL, 0x1A3},
        {LogicalOpcode::CMSG_LIST_INVENTORY, 0x19E},
        {LogicalOpcode::SMSG_LIST_INVENTORY, 0x19F},
        {LogicalOpcode::CMSG_SELL_ITEM, 0x1A0},
        {LogicalOpcode::SMSG_SELL_ITEM, 0x1A1},
        {LogicalOpcode::CMSG_BUY_ITEM, 0x1A2},
        {LogicalOpcode::SMSG_BUY_FAILED, 0x1A5},
        {LogicalOpcode::CMSG_TRAINER_LIST, 0x01B0},
        {LogicalOpcode::SMSG_TRAINER_LIST, 0x01B1},
        {LogicalOpcode::CMSG_TRAINER_BUY_SPELL, 0x01B2},
        {LogicalOpcode::SMSG_TRAINER_BUY_FAILED, 0x01B4},
        {LogicalOpcode::CMSG_ITEM_QUERY_SINGLE, 0x056},
        {LogicalOpcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE, 0x058},
        {LogicalOpcode::CMSG_USE_ITEM, 0x00AB},
        {LogicalOpcode::CMSG_AUTOEQUIP_ITEM, 0x10A},
        {LogicalOpcode::CMSG_SWAP_ITEM, 0x10C},
        {LogicalOpcode::CMSG_SWAP_INV_ITEM, 0x10D},
        {LogicalOpcode::SMSG_INVENTORY_CHANGE_FAILURE, 0x112},
        {LogicalOpcode::CMSG_INSPECT, 0x114},
        {LogicalOpcode::SMSG_INSPECT_RESULTS, 0x115},
        {LogicalOpcode::CMSG_REPOP_REQUEST, 0x015A},
        {LogicalOpcode::SMSG_RESURRECT_REQUEST, 0x015B},
        {LogicalOpcode::CMSG_RESURRECT_RESPONSE, 0x015C},
        {LogicalOpcode::CMSG_SPIRIT_HEALER_ACTIVATE, 0x021C},
        {LogicalOpcode::SMSG_SPIRIT_HEALER_CONFIRM, 0x0222},
        {LogicalOpcode::SMSG_RESURRECT_CANCEL, 0x0390},
        {LogicalOpcode::MSG_MOVE_TELEPORT_ACK, 0x0C7},
        {LogicalOpcode::SMSG_TRANSFER_PENDING, 0x003F},
        {LogicalOpcode::SMSG_NEW_WORLD, 0x003E},
        {LogicalOpcode::MSG_MOVE_WORLDPORT_ACK, 0x00DC},
        {LogicalOpcode::SMSG_TRANSFER_ABORTED, 0x0040},
        {LogicalOpcode::SMSG_FORCE_RUN_SPEED_CHANGE, 0x00E2},
        {LogicalOpcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK, 0x00E3},
        {LogicalOpcode::CMSG_CANCEL_MOUNT_AURA, 0x0375},
        {LogicalOpcode::SMSG_SHOWTAXINODES, 0x01A9},
        {LogicalOpcode::SMSG_ACTIVATETAXIREPLY, 0x01AE},
        {LogicalOpcode::SMSG_ACTIVATETAXIREPLY_ALT, 0x029D},
        {LogicalOpcode::SMSG_NEW_TAXI_PATH, 0x01AF},
        {LogicalOpcode::CMSG_ACTIVATETAXIEXPRESS, 0x0312},
        {LogicalOpcode::SMSG_BATTLEFIELD_PORT_DENIED, 0x014B},
        {LogicalOpcode::SMSG_REMOVED_FROM_PVP_QUEUE, 0x0170},
        {LogicalOpcode::SMSG_TRAINER_BUY_SUCCEEDED, 0x01B3},
        {LogicalOpcode::SMSG_BINDPOINTUPDATE, 0x0155},
        {LogicalOpcode::CMSG_BATTLEFIELD_LIST, 0x023C},
        {LogicalOpcode::SMSG_BATTLEFIELD_LIST, 0x023D},
        {LogicalOpcode::CMSG_BATTLEFIELD_JOIN, 0x023E},
        {LogicalOpcode::CMSG_BATTLEFIELD_STATUS, 0x02D3},
        {LogicalOpcode::SMSG_BATTLEFIELD_STATUS, 0x02D4},
        {LogicalOpcode::CMSG_BATTLEFIELD_PORT, 0x02D5},
        {LogicalOpcode::CMSG_BATTLEMASTER_HELLO, 0x02D7},
        {LogicalOpcode::MSG_PVP_LOG_DATA, 0x02E0},
        {LogicalOpcode::CMSG_LEAVE_BATTLEFIELD, 0x02E1},
        {LogicalOpcode::SMSG_GROUP_JOINED_BATTLEGROUND, 0x02E8},
        {LogicalOpcode::MSG_BATTLEGROUND_PLAYER_POSITIONS, 0x02E9},
        {LogicalOpcode::SMSG_BATTLEGROUND_PLAYER_JOINED, 0x02EC},
        {LogicalOpcode::SMSG_BATTLEGROUND_PLAYER_LEFT, 0x02ED},
        {LogicalOpcode::CMSG_BATTLEMASTER_JOIN, 0x02EE},
        {LogicalOpcode::SMSG_JOINED_BATTLEGROUND_QUEUE, 0x038A},
        {LogicalOpcode::CMSG_ARENA_TEAM_CREATE, 0x0348},
        {LogicalOpcode::SMSG_ARENA_TEAM_COMMAND_RESULT, 0x0349},
        {LogicalOpcode::CMSG_ARENA_TEAM_QUERY, 0x034B},
        {LogicalOpcode::SMSG_ARENA_TEAM_QUERY_RESPONSE, 0x034C},
        {LogicalOpcode::CMSG_ARENA_TEAM_ROSTER, 0x034D},
        {LogicalOpcode::SMSG_ARENA_TEAM_ROSTER, 0x034E},
        {LogicalOpcode::CMSG_ARENA_TEAM_INVITE, 0x034F},
        {LogicalOpcode::SMSG_ARENA_TEAM_INVITE, 0x0350},
        {LogicalOpcode::CMSG_ARENA_TEAM_ACCEPT, 0x0351},
        {LogicalOpcode::CMSG_ARENA_TEAM_DECLINE, 0x0352},
        {LogicalOpcode::CMSG_ARENA_TEAM_LEAVE, 0x0353},
        {LogicalOpcode::CMSG_ARENA_TEAM_REMOVE, 0x0354},
        {LogicalOpcode::CMSG_ARENA_TEAM_DISBAND, 0x0355},
        {LogicalOpcode::CMSG_ARENA_TEAM_LEADER, 0x0356},
        {LogicalOpcode::SMSG_ARENA_TEAM_EVENT, 0x0357},
        {LogicalOpcode::CMSG_BATTLEMASTER_JOIN_ARENA, 0x0358},
        {LogicalOpcode::SMSG_ARENA_TEAM_STATS, 0x035B},
        {LogicalOpcode::SMSG_ARENA_ERROR, 0x0376},
        {LogicalOpcode::MSG_INSPECT_ARENA_TEAMS, 0x0377},
    };

    logicalToWire_.clear();
    wireToLogical_.clear();
    for (auto& d : defaults) {
        uint16_t logIdx = static_cast<uint16_t>(d.op);
        logicalToWire_[logIdx] = d.wire;
        wireToLogical_[d.wire] = logIdx;
    }
    LOG_INFO("OpcodeTable: loaded ", logicalToWire_.size(), " WotLK default opcodes");
}

bool OpcodeTable::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("OpcodeTable: cannot open ", path, ", using defaults");
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    // Save old tables so we can restore on failure
    auto savedLogicalToWire = logicalToWire_;
    auto savedWireToLogical = wireToLogical_;

    logicalToWire_.clear();
    wireToLogical_.clear();

    // Parse simple JSON: { "NAME": "0xHEX", ... } or { "NAME": 123, ... }
    size_t pos = 0;
    size_t loaded = 0;
    while (pos < json.size()) {
        // Find next quoted key
        size_t keyStart = json.find('"', pos);
        if (keyStart == std::string::npos) break;
        size_t keyEnd = json.find('"', keyStart + 1);
        if (keyEnd == std::string::npos) break;
        std::string key = json.substr(keyStart + 1, keyEnd - keyStart - 1);

        // Find colon then value
        size_t colon = json.find(':', keyEnd);
        if (colon == std::string::npos) break;

        // Skip whitespace
        size_t valStart = colon + 1;
        while (valStart < json.size() && (json[valStart] == ' ' || json[valStart] == '\t' ||
               json[valStart] == '\r' || json[valStart] == '\n' || json[valStart] == '"'))
            ++valStart;

        size_t valEnd = json.find_first_of(",}\"\r\n", valStart);
        if (valEnd == std::string::npos) valEnd = json.size();
        std::string valStr = json.substr(valStart, valEnd - valStart);

        // Parse hex or decimal value
        uint16_t wire = 0;
        try {
            if (valStr.size() > 2 && (valStr[0] == '0' && (valStr[1] == 'x' || valStr[1] == 'X'))) {
                wire = static_cast<uint16_t>(std::stoul(valStr, nullptr, 16));
            } else {
                wire = static_cast<uint16_t>(std::stoul(valStr));
            }
        } catch (...) {
            pos = valEnd + 1;
            continue;
        }

        auto logOp = nameToLogical(key);
        if (logOp) {
            uint16_t logIdx = static_cast<uint16_t>(*logOp);
            logicalToWire_[logIdx] = wire;
            wireToLogical_[wire] = logIdx;
            ++loaded;
        }

        pos = valEnd + 1;
    }

    if (loaded == 0) {
        LOG_WARNING("OpcodeTable: no opcodes loaded from ", path, ", restoring previous tables");
        logicalToWire_ = std::move(savedLogicalToWire);
        wireToLogical_ = std::move(savedWireToLogical);
        return false;
    }

    LOG_INFO("OpcodeTable: loaded ", loaded, " opcodes from ", path);
    return true;
}

uint16_t OpcodeTable::toWire(LogicalOpcode op) const {
    auto it = logicalToWire_.find(static_cast<uint16_t>(op));
    return (it != logicalToWire_.end()) ? it->second : 0xFFFF;
}

std::optional<LogicalOpcode> OpcodeTable::fromWire(uint16_t wireValue) const {
    auto it = wireToLogical_.find(wireValue);
    if (it != wireToLogical_.end()) {
        return static_cast<LogicalOpcode>(it->second);
    }
    return std::nullopt;
}

bool OpcodeTable::hasOpcode(LogicalOpcode op) const {
    return logicalToWire_.count(static_cast<uint16_t>(op)) > 0;
}

} // namespace game
} // namespace wowee
