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
    {"SMSG_CHARACTER_LOGIN_FAILED", LogicalOpcode::SMSG_CHARACTER_LOGIN_FAILED},
    {"SMSG_PONG", LogicalOpcode::SMSG_PONG},
    {"SMSG_LOGIN_VERIFY_WORLD", LogicalOpcode::SMSG_LOGIN_VERIFY_WORLD},
    {"SMSG_INIT_WORLD_STATES", LogicalOpcode::SMSG_INIT_WORLD_STATES},
    {"SMSG_LOGIN_SETTIMESPEED", LogicalOpcode::SMSG_LOGIN_SETTIMESPEED},
    {"SMSG_TUTORIAL_FLAGS", LogicalOpcode::SMSG_TUTORIAL_FLAGS},
    {"SMSG_INITIALIZE_FACTIONS", LogicalOpcode::SMSG_INITIALIZE_FACTIONS},
    {"SMSG_WARDEN_DATA", LogicalOpcode::SMSG_WARDEN_DATA},
    {"CMSG_WARDEN_DATA", LogicalOpcode::CMSG_WARDEN_DATA},
    {"SMSG_ACCOUNT_DATA_TIMES", LogicalOpcode::SMSG_ACCOUNT_DATA_TIMES},
    {"SMSG_CLIENTCACHE_VERSION", LogicalOpcode::SMSG_CLIENTCACHE_VERSION},
    {"SMSG_FEATURE_SYSTEM_STATUS", LogicalOpcode::SMSG_FEATURE_SYSTEM_STATUS},
    {"SMSG_MOTD", LogicalOpcode::SMSG_MOTD},
    {"SMSG_NOTIFICATION", LogicalOpcode::SMSG_NOTIFICATION},
    {"SMSG_UPDATE_OBJECT", LogicalOpcode::SMSG_UPDATE_OBJECT},
    {"SMSG_COMPRESSED_UPDATE_OBJECT", LogicalOpcode::SMSG_COMPRESSED_UPDATE_OBJECT},
    {"SMSG_UNKNOWN_1F5", LogicalOpcode::SMSG_UNKNOWN_1F5},
    {"SMSG_MONSTER_MOVE_TRANSPORT", LogicalOpcode::SMSG_MONSTER_MOVE_TRANSPORT},
    {"SMSG_SPLINE_MOVE_SET_WALK_MODE", LogicalOpcode::SMSG_SPLINE_MOVE_SET_WALK_MODE},
    {"SMSG_SPLINE_MOVE_SET_RUN_MODE", LogicalOpcode::SMSG_SPLINE_MOVE_SET_RUN_MODE},
    {"SMSG_SPLINE_MOVE_SET_RUN_SPEED", LogicalOpcode::SMSG_SPLINE_MOVE_SET_RUN_SPEED},
    {"SMSG_SPLINE_MOVE_SET_RUN_BACK_SPEED", LogicalOpcode::SMSG_SPLINE_MOVE_SET_RUN_BACK_SPEED},
    {"SMSG_SPLINE_MOVE_SET_SWIM_SPEED", LogicalOpcode::SMSG_SPLINE_MOVE_SET_SWIM_SPEED},
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
    {"SMSG_CONTACT_LIST", LogicalOpcode::SMSG_CONTACT_LIST},
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
    {"CMSG_GUILD_DISBAND", LogicalOpcode::CMSG_GUILD_DISBAND},
    {"CMSG_GUILD_LEADER", LogicalOpcode::CMSG_GUILD_LEADER},
    {"CMSG_GUILD_SET_PUBLIC_NOTE", LogicalOpcode::CMSG_GUILD_SET_PUBLIC_NOTE},
    {"CMSG_GUILD_SET_OFFICER_NOTE", LogicalOpcode::CMSG_GUILD_SET_OFFICER_NOTE},
    {"SMSG_GUILD_EVENT", LogicalOpcode::SMSG_GUILD_EVENT},
    {"SMSG_GUILD_COMMAND_RESULT", LogicalOpcode::SMSG_GUILD_COMMAND_RESULT},
    {"MSG_RAID_READY_CHECK", LogicalOpcode::MSG_RAID_READY_CHECK},
    {"MSG_RAID_READY_CHECK_CONFIRM", LogicalOpcode::MSG_RAID_READY_CHECK_CONFIRM},
    {"SMSG_ITEM_PUSH_RESULT", LogicalOpcode::SMSG_ITEM_PUSH_RESULT},
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
    {"SMSG_COMPRESSED_MOVES", LogicalOpcode::SMSG_COMPRESSED_MOVES},
    {"CMSG_ATTACKSWING", LogicalOpcode::CMSG_ATTACKSWING},
    {"CMSG_ATTACKSTOP", LogicalOpcode::CMSG_ATTACKSTOP},
    {"SMSG_ATTACKSTART", LogicalOpcode::SMSG_ATTACKSTART},
    {"SMSG_ATTACKSTOP", LogicalOpcode::SMSG_ATTACKSTOP},
    {"SMSG_ATTACKERSTATEUPDATE", LogicalOpcode::SMSG_ATTACKERSTATEUPDATE},
    {"SMSG_AI_REACTION", LogicalOpcode::SMSG_AI_REACTION},
    {"SMSG_SPELLNONMELEEDAMAGELOG", LogicalOpcode::SMSG_SPELLNONMELEEDAMAGELOG},
    {"SMSG_PLAY_SPELL_VISUAL", LogicalOpcode::SMSG_PLAY_SPELL_VISUAL},
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
    {"SMSG_CANCEL_AUTO_REPEAT", LogicalOpcode::SMSG_CANCEL_AUTO_REPEAT},
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
    {"SMSG_QUESTGIVER_QUEST_LIST", LogicalOpcode::SMSG_QUESTGIVER_QUEST_LIST},
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
    {"SMSG_QUESTUPDATE_ADD_ITEM", LogicalOpcode::SMSG_QUESTUPDATE_ADD_ITEM},
    {"SMSG_QUESTUPDATE_COMPLETE", LogicalOpcode::SMSG_QUESTUPDATE_COMPLETE},
    {"SMSG_QUEST_FORCE_REMOVE", LogicalOpcode::SMSG_QUEST_FORCE_REMOVE},
    {"CMSG_QUEST_QUERY", LogicalOpcode::CMSG_QUEST_QUERY},
    {"SMSG_QUEST_QUERY_RESPONSE", LogicalOpcode::SMSG_QUEST_QUERY_RESPONSE},
    {"SMSG_QUESTLOG_FULL", LogicalOpcode::SMSG_QUESTLOG_FULL},
    {"CMSG_LIST_INVENTORY", LogicalOpcode::CMSG_LIST_INVENTORY},
    {"SMSG_LIST_INVENTORY", LogicalOpcode::SMSG_LIST_INVENTORY},
    {"CMSG_SELL_ITEM", LogicalOpcode::CMSG_SELL_ITEM},
    {"SMSG_SELL_ITEM", LogicalOpcode::SMSG_SELL_ITEM},
    {"CMSG_BUY_ITEM", LogicalOpcode::CMSG_BUY_ITEM},
    {"CMSG_BUYBACK_ITEM", LogicalOpcode::CMSG_BUYBACK_ITEM},
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
    {"SMSG_CLIENT_CONTROL_UPDATE", LogicalOpcode::SMSG_CLIENT_CONTROL_UPDATE},
    {"CMSG_FORCE_RUN_SPEED_CHANGE_ACK", LogicalOpcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK},
    {"CMSG_CANCEL_MOUNT_AURA", LogicalOpcode::CMSG_CANCEL_MOUNT_AURA},
    {"SMSG_SHOWTAXINODES", LogicalOpcode::SMSG_SHOWTAXINODES},
    {"SMSG_ACTIVATETAXIREPLY", LogicalOpcode::SMSG_ACTIVATETAXIREPLY},
    {"SMSG_STANDSTATE_UPDATE", LogicalOpcode::SMSG_STANDSTATE_UPDATE},
    {"SMSG_NEW_TAXI_PATH", LogicalOpcode::SMSG_NEW_TAXI_PATH},
    {"CMSG_ACTIVATETAXIEXPRESS", LogicalOpcode::CMSG_ACTIVATETAXIEXPRESS},
    {"SMSG_BATTLEFIELD_PORT_DENIED", LogicalOpcode::SMSG_BATTLEFIELD_PORT_DENIED},
    {"SMSG_REMOVED_FROM_PVP_QUEUE", LogicalOpcode::SMSG_REMOVED_FROM_PVP_QUEUE},
    {"SMSG_TRAINER_BUY_SUCCEEDED", LogicalOpcode::SMSG_TRAINER_BUY_SUCCEEDED},
    {"SMSG_BINDPOINTUPDATE", LogicalOpcode::SMSG_BINDPOINTUPDATE},
    {"SMSG_SET_PROFICIENCY", LogicalOpcode::SMSG_SET_PROFICIENCY},
    {"SMSG_ACTION_BUTTONS", LogicalOpcode::SMSG_ACTION_BUTTONS},
    {"SMSG_LEVELUP_INFO", LogicalOpcode::SMSG_LEVELUP_INFO},
    {"CMSG_TAXINODE_STATUS_QUERY", LogicalOpcode::CMSG_TAXINODE_STATUS_QUERY},
    {"SMSG_TAXINODE_STATUS", LogicalOpcode::SMSG_TAXINODE_STATUS},
    {"CMSG_UPDATE_ACCOUNT_DATA", LogicalOpcode::CMSG_UPDATE_ACCOUNT_DATA},
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
    {"SMSG_WEATHER", LogicalOpcode::SMSG_WEATHER},
    {"CMSG_EMOTE", LogicalOpcode::CMSG_EMOTE},
    {"SMSG_EMOTE", LogicalOpcode::SMSG_EMOTE},
    {"CMSG_TEXT_EMOTE", LogicalOpcode::CMSG_TEXT_EMOTE},
    {"SMSG_TEXT_EMOTE", LogicalOpcode::SMSG_TEXT_EMOTE},
    {"CMSG_JOIN_CHANNEL", LogicalOpcode::CMSG_JOIN_CHANNEL},
    {"CMSG_LEAVE_CHANNEL", LogicalOpcode::CMSG_LEAVE_CHANNEL},
    {"SMSG_CHANNEL_NOTIFY", LogicalOpcode::SMSG_CHANNEL_NOTIFY},
    {"CMSG_CHANNEL_LIST", LogicalOpcode::CMSG_CHANNEL_LIST},
    {"SMSG_CHANNEL_LIST", LogicalOpcode::SMSG_CHANNEL_LIST},
    {"SMSG_INSPECT_TALENT", LogicalOpcode::SMSG_INSPECT_TALENT},
    // Mail
    {"SMSG_SHOW_MAILBOX", LogicalOpcode::SMSG_SHOW_MAILBOX},
    {"CMSG_GET_MAIL_LIST", LogicalOpcode::CMSG_GET_MAIL_LIST},
    {"SMSG_MAIL_LIST_RESULT", LogicalOpcode::SMSG_MAIL_LIST_RESULT},
    {"CMSG_SEND_MAIL", LogicalOpcode::CMSG_SEND_MAIL},
    {"SMSG_SEND_MAIL_RESULT", LogicalOpcode::SMSG_SEND_MAIL_RESULT},
    {"CMSG_MAIL_TAKE_MONEY", LogicalOpcode::CMSG_MAIL_TAKE_MONEY},
    {"CMSG_MAIL_TAKE_ITEM", LogicalOpcode::CMSG_MAIL_TAKE_ITEM},
    {"CMSG_MAIL_DELETE", LogicalOpcode::CMSG_MAIL_DELETE},
    {"CMSG_MAIL_MARK_AS_READ", LogicalOpcode::CMSG_MAIL_MARK_AS_READ},
    {"SMSG_RECEIVED_MAIL", LogicalOpcode::SMSG_RECEIVED_MAIL},
    {"MSG_QUERY_NEXT_MAIL_TIME", LogicalOpcode::MSG_QUERY_NEXT_MAIL_TIME},
    // Bank
    {"CMSG_BANKER_ACTIVATE", LogicalOpcode::CMSG_BANKER_ACTIVATE},
    {"SMSG_SHOW_BANK", LogicalOpcode::SMSG_SHOW_BANK},
    {"CMSG_BUY_BANK_SLOT", LogicalOpcode::CMSG_BUY_BANK_SLOT},
    {"SMSG_BUY_BANK_SLOT_RESULT", LogicalOpcode::SMSG_BUY_BANK_SLOT_RESULT},
    {"CMSG_AUTOBANK_ITEM", LogicalOpcode::CMSG_AUTOBANK_ITEM},
    {"CMSG_AUTOSTORE_BANK_ITEM", LogicalOpcode::CMSG_AUTOSTORE_BANK_ITEM},
    // Guild Bank
    {"CMSG_GUILD_BANKER_ACTIVATE", LogicalOpcode::CMSG_GUILD_BANKER_ACTIVATE},
    {"CMSG_GUILD_BANK_QUERY_TAB", LogicalOpcode::CMSG_GUILD_BANK_QUERY_TAB},
    {"SMSG_GUILD_BANK_LIST", LogicalOpcode::SMSG_GUILD_BANK_LIST},
    {"CMSG_GUILD_BANK_SWAP_ITEMS", LogicalOpcode::CMSG_GUILD_BANK_SWAP_ITEMS},
    {"CMSG_GUILD_BANK_BUY_TAB", LogicalOpcode::CMSG_GUILD_BANK_BUY_TAB},
    {"CMSG_GUILD_BANK_UPDATE_TAB", LogicalOpcode::CMSG_GUILD_BANK_UPDATE_TAB},
    {"CMSG_GUILD_BANK_DEPOSIT_MONEY", LogicalOpcode::CMSG_GUILD_BANK_DEPOSIT_MONEY},
    {"CMSG_GUILD_BANK_WITHDRAW_MONEY", LogicalOpcode::CMSG_GUILD_BANK_WITHDRAW_MONEY},
    // Auction House
    {"MSG_AUCTION_HELLO", LogicalOpcode::MSG_AUCTION_HELLO},
    {"CMSG_AUCTION_SELL_ITEM", LogicalOpcode::CMSG_AUCTION_SELL_ITEM},
    {"CMSG_AUCTION_REMOVE_ITEM", LogicalOpcode::CMSG_AUCTION_REMOVE_ITEM},
    {"CMSG_AUCTION_LIST_ITEMS", LogicalOpcode::CMSG_AUCTION_LIST_ITEMS},
    {"CMSG_AUCTION_LIST_OWNER_ITEMS", LogicalOpcode::CMSG_AUCTION_LIST_OWNER_ITEMS},
    {"CMSG_AUCTION_PLACE_BID", LogicalOpcode::CMSG_AUCTION_PLACE_BID},
    {"SMSG_AUCTION_COMMAND_RESULT", LogicalOpcode::SMSG_AUCTION_COMMAND_RESULT},
    {"SMSG_AUCTION_LIST_RESULT", LogicalOpcode::SMSG_AUCTION_LIST_RESULT},
    {"SMSG_AUCTION_OWNER_LIST_RESULT", LogicalOpcode::SMSG_AUCTION_OWNER_LIST_RESULT},
    {"SMSG_AUCTION_BIDDER_LIST_RESULT", LogicalOpcode::SMSG_AUCTION_BIDDER_LIST_RESULT},
    {"SMSG_AUCTION_OWNER_NOTIFICATION", LogicalOpcode::SMSG_AUCTION_OWNER_NOTIFICATION},
    {"SMSG_AUCTION_BIDDER_NOTIFICATION", LogicalOpcode::SMSG_AUCTION_BIDDER_NOTIFICATION},
    {"CMSG_AUCTION_LIST_BIDDER_ITEMS", LogicalOpcode::CMSG_AUCTION_LIST_BIDDER_ITEMS},
    {"SMSG_UNKNOWN_319", LogicalOpcode::SMSG_UNKNOWN_319},
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

bool OpcodeTable::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("OpcodeTable: cannot open ", path, ", using defaults");
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    // Start fresh — JSON is the single source of truth for opcode mappings.
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
        LOG_WARNING("OpcodeTable: no opcodes loaded from ", path);
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
