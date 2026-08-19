#include "ui/framexml_takeover.hpp"
#include "core/env_flag.hpp"
#include "game/game_handler.hpp"
#include "game/achievement_criteria.hpp"
#include "game/packed_time.hpp"
#include "game/protocol_constants.hpp"
#include "game/game_utils.hpp"
#include "game/chat_handler.hpp"
#include "game/movement_handler.hpp"
#include "game/combat_handler.hpp"
#include "game/spell_handler.hpp"
#include "game/inventory_handler.hpp"
#include "game/social_handler.hpp"
#include "game/quest_handler.hpp"
#include "game/warden_handler.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
#include "game/packet_parsers.hpp"
#include "game/transport_manager.hpp"
#include "game/opcodes.hpp"
#include "game/update_field_table.hpp"
#include "game/expansion_profile.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/spell_visual_system.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "auth/crypto.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include "rendering/animation/animation_ids.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <random>
#include <zlib.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <array>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <openssl/sha.h>
#include <openssl/hmac.h>


namespace wowee {
namespace game {

namespace {

bool isAuthCharPipelineOpcode(LogicalOpcode op) {
    switch (op) {
        case Opcode::SMSG_AUTH_CHALLENGE:
        case Opcode::SMSG_AUTH_RESPONSE:
        case Opcode::SMSG_CLIENTCACHE_VERSION:
        case Opcode::SMSG_TUTORIAL_FLAGS:
        case Opcode::SMSG_WARDEN_DATA:
        case Opcode::SMSG_CHAR_ENUM:
        case Opcode::SMSG_CHAR_CREATE:
        case Opcode::SMSG_CHAR_DELETE:
            return true;
        default:
            return false;
    }
}

int incomingPacketsBudgetPerUpdate(WorldState state) {
    static const int inWorldBudget =
        core::envIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKETS", 24, 1, 512);
    static const int loginBudget =
        core::envIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKETS_LOGIN", 96, 1, 512);
    return state == WorldState::IN_WORLD ? inWorldBudget : loginBudget;
}

float incomingPacketBudgetMs(WorldState state) {
    static const int inWorldBudgetMs =
        core::envIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKET_MS", 2, 1, 50);
    static const int loginBudgetMs =
        core::envIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKET_MS_LOGIN", 8, 1, 50);
    return static_cast<float>(state == WorldState::IN_WORLD ? inWorldBudgetMs : loginBudgetMs);
}

float slowPacketLogThresholdMs() {
    static const int thresholdMs =
        core::envIntClamped("WOWEE_NET_SLOW_PACKET_LOG_MS", 10, 1, 60000);
    return static_cast<float>(thresholdMs);
}

constexpr size_t kMaxQueuedInboundPackets = 4096;

} // end anonymous namespace

// The rune bar draws itself from these two events and from nothing else. All
// three rune packets stored their state and announced none of it, so a death
// knight's runes were drawn once at login and then stood still.
//
// "usable" is sent as a present argument or no argument at all rather than as
// one and zero, because zero is true in Lua: an argument saying the rune is on
// cooldown would read as one saying it is ready.
void GameHandler::fireRuneUpdate(uint32_t index) {
    if (index >= playerRunes_.size()) return;
    const std::string oneBased = std::to_string(index + 1);
    fireAddonEvent("RUNE_TYPE_UPDATE", {oneBased});
    if (playerRunes_[index].ready) {
        fireAddonEvent("RUNE_POWER_UPDATE", {oneBased, "1"});
    } else {
        fireAddonEvent("RUNE_POWER_UPDATE", {oneBased});
    }
}

// The opcodes this handler answers itself, grouped by what they are about: the
// session handshake, XP and exploration, corpses and combat state, guilds and
// loot and vendors, teleports and taxis and battlegrounds, world states, action
// buttons, levelling.
void GameHandler::registerCoreOpcodes() {
    // -----------------------------------------------------------------------
    // Auth / session / pre-world handshake
    // -----------------------------------------------------------------------
    dispatchTable_[Opcode::SMSG_AUTH_CHALLENGE] = [this](network::Packet& packet) {
        if (state == WorldState::CONNECTED)
            handleAuthChallenge(packet);
        else
            LOG_WARNING("Unexpected SMSG_AUTH_CHALLENGE in state: ", worldStateName(state));
    };
    dispatchTable_[Opcode::SMSG_AUTH_RESPONSE] = [this](network::Packet& packet) {
        if (state == WorldState::AUTH_SENT)
            handleAuthResponse(packet);
        else
            LOG_WARNING("Unexpected SMSG_AUTH_RESPONSE in state: ", worldStateName(state));
    };
    dispatchTable_[Opcode::SMSG_CHAR_CREATE] = [this](network::Packet& packet) {
        handleCharCreateResponse(packet);
    };
    dispatchTable_[Opcode::SMSG_CHAR_DELETE] = [this](network::Packet& packet) {
        uint8_t result = packet.readUInt8();
        lastCharDeleteResult_ = result;
        pendingCharDeleteResponse_ = false;
        bool success = (result == 0x00 || result == 0x47);
        LOG_INFO("SMSG_CHAR_DELETE result: ", static_cast<int>(result), success ? " (success)" : " (failed)");
        requestCharacterList();
        std::string msg;
        if (success) {
            msg = "Character deleted.";
        } else {
            // Map known CHAR_DELETE_* result codes to user-friendly messages
            switch (result) {
                case 0x31: msg = "Delete failed: character is a guild leader. Transfer leadership first."; break;
                case 0x32: msg = "Delete failed: character is in an arena team."; break;
                case 0x3A: msg = "Delete failed: character has mail. Check mailbox first."; break;
                default:   msg = "Delete failed (server error code " + std::to_string(static_cast<int>(result)) + ")."; break;
            }
        }
        if (charDeleteCallback_) charDeleteCallback_(success, msg);
    };
    dispatchTable_[Opcode::SMSG_CHAR_ENUM] = [this](network::Packet& packet) {
        if (state == WorldState::CHAR_LIST_REQUESTED)
            handleCharEnum(packet);
        else
            LOG_WARNING("Unexpected SMSG_CHAR_ENUM in state: ", worldStateName(state));
    };
    registerHandler(Opcode::SMSG_CHARACTER_LOGIN_FAILED, &GameHandler::handleCharLoginFailed);
    dispatchTable_[Opcode::SMSG_LOGIN_VERIFY_WORLD] = [this](network::Packet& packet) {
        if (state == WorldState::ENTERING_WORLD || state == WorldState::IN_WORLD)
            handleLoginVerifyWorld(packet);
        else
            LOG_WARNING("Unexpected SMSG_LOGIN_VERIFY_WORLD in state: ", worldStateName(state));
    };
    registerHandler(Opcode::SMSG_LOGIN_SETTIMESPEED, &GameHandler::handleLoginSetTimeSpeed);
    registerHandler(Opcode::SMSG_CLIENTCACHE_VERSION, &GameHandler::handleClientCacheVersion);
    registerHandler(Opcode::SMSG_TUTORIAL_FLAGS, &GameHandler::handleTutorialFlags);
    registerHandler(Opcode::SMSG_ACCOUNT_DATA_TIMES, &GameHandler::handleAccountDataTimes);
    registerHandler(Opcode::SMSG_MOTD, &GameHandler::handleMotd);
    registerHandler(Opcode::SMSG_NOTIFICATION, &GameHandler::handleNotification);
    registerHandler(Opcode::SMSG_PONG, &GameHandler::handlePong);

    // -----------------------------------------------------------------------
    // World object updates + entity queries (delegated to EntityController)
    // -----------------------------------------------------------------------
    entityController_->registerOpcodes(dispatchTable_);

    // -----------------------------------------------------------------------
    // Item push / logout
    // -----------------------------------------------------------------------
    registerSkipHandler(Opcode::SMSG_ADDON_INFO);
    registerSkipHandler(Opcode::SMSG_EXPECTED_SPAM_RECORDS);

    // -----------------------------------------------------------------------
    // XP / exploration
    // -----------------------------------------------------------------------
    registerHandler(Opcode::SMSG_LOG_XPGAIN, &GameHandler::handleXpGain);
    dispatchTable_[Opcode::SMSG_EXPLORATION_EXPERIENCE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint32_t areaId   = packet.readUInt32();
            uint32_t xpGained = packet.readUInt32();
            if (xpGained > 0) {
                std::string areaName = getAreaName(areaId);
                std::string msg;
                if (!areaName.empty()) {
                    msg = "Discovered " + areaName + "! Gained " + std::to_string(xpGained) + " experience.";
                } else {
                    char buf[128];
                    std::snprintf(buf, sizeof(buf), "Discovered new area! Gained %u experience.", xpGained);
                    msg = buf;
                }
                addLocalChatLine(game::ChatType::COMBAT_XP_GAIN, msg);
                addCombatText(CombatTextEntry::XP_GAIN, static_cast<int32_t>(xpGained), 0, true);
                if (areaDiscoveryCallback_) areaDiscoveryCallback_(areaName, xpGained);
            }
        }
    };

    // SMSG_PET_NAME_QUERY_RESPONSE is handled in SpellHandler now, which is
    // where the request that provokes it is sent. It was skipped because
    // nothing asked, and nothing asked because it was skipped.

    // -----------------------------------------------------------------------
    // Entity delta updates: health / power / world state / combo / timers / PvP
    // (SMSG_HEALTH_UPDATE, SMSG_POWER_UPDATE, SMSG_UPDATE_COMBO_POINTS,
    //  SMSG_PVP_CREDIT, SMSG_PROCRESIST → moved to CombatHandler)
    // -----------------------------------------------------------------------
    dispatchTable_[Opcode::SMSG_UPDATE_WORLD_STATE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint32_t field = packet.readUInt32();
        uint32_t value = packet.readUInt32();
        worldStates_[field] = value;
        LOG_DEBUG("SMSG_UPDATE_WORLD_STATE: field=", field, " value=", value);
        fireAddonEvent("UPDATE_WORLD_STATES", {});
    };
    dispatchTable_[Opcode::SMSG_WORLD_STATE_UI_TIMER_UPDATE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t serverTime = packet.readUInt32();
            // The world state timer bar counts from this and had nothing to
            // count from - it was read and logged and dropped.
            if (addonEventCallback_)
                addonEventCallback_("WORLD_STATE_UI_TIMER_UPDATE", {std::to_string(serverTime)});
            LOG_DEBUG("SMSG_WORLD_STATE_UI_TIMER_UPDATE: serverTime=", serverTime);
        }
    };
    dispatchTable_[Opcode::SMSG_START_MIRROR_TIMER] = [this](network::Packet& packet) {
        // type(4) + value(4) + maxValue(4) + scale(4) + paused(1) + spellId(4).
        // The last two were being read the other way round: harmless while the
        // spell id is 0, but a non-zero one would land its high byte in paused and
        // freeze the bar.
        if (!packet.hasRemaining(21)) return;
        uint32_t type  = packet.readUInt32();
        int32_t  value = static_cast<int32_t>(packet.readUInt32());
        int32_t  maxV  = static_cast<int32_t>(packet.readUInt32());
        int32_t  scale = static_cast<int32_t>(packet.readUInt32());
        uint8_t  paused = packet.readUInt8();
        /*uint32_t spellId =*/ packet.readUInt32();
        if (type < 3) {
            mirrorTimers_[type].value     = value;
            mirrorTimers_[type].maxValue  = maxV;
            mirrorTimers_[type].scale     = scale;
            mirrorTimers_[type].paused    = (paused != 0);
            mirrorTimers_[type].active    = true;
            mirrorTimers_[type].pendingMs = 0.0f;  // server re-sync; drop the local carry
            // The interface names its timers rather than numbering them, and
            // wants a label to draw beside the bar: MirrorTimer_Show(timer,
            // value, maxvalue, scale, paused, label). Sending the type as a
            // number and omitting the label left it matching no timer and
            // dividing a nil.
            // The names WoW passes, not the wire's own ordering words - see
            // kMirrorTimerNames, which every path that names a timer shares.
            const char* timerName = (type < 3) ? kMirrorTimerNames[type] : "BREATH";
            const char* timerLabel = (type < 3) ? kMirrorTimerLabels[type] : "Breath";
            fireAddonEvent("MIRROR_TIMER_START", {
                    timerName, std::to_string(value),
                    std::to_string(maxV), std::to_string(scale),
                    paused ? "1" : "0", timerLabel});
        }
    };
    dispatchTable_[Opcode::SMSG_STOP_MIRROR_TIMER] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t type = packet.readUInt32();
        if (type < 3) {
            mirrorTimers_[type].active = false;
            mirrorTimers_[type].value  = 0;
            // Named, like the start above: MirrorTimer_Hide matches on the
            // timer's name, so a number hides nothing.
            fireAddonEvent("MIRROR_TIMER_STOP",
                           {type < 3 ? kMirrorTimerNames[type] : "BREATH"});
        }
    };
    dispatchTable_[Opcode::SMSG_PAUSE_MIRROR_TIMER] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(5)) return;
        uint32_t type   = packet.readUInt32();
        uint8_t  paused = packet.readUInt8();
        if (type < 3) {
            mirrorTimers_[type].paused = (paused != 0);
            fireAddonEvent("MIRROR_TIMER_PAUSE", {paused ? "1" : "0"});
        }
    };

    // -----------------------------------------------------------------------
    // Cast result / spell proc
    // (SMSG_CAST_RESULT, SMSG_SPELL_FAILED_OTHER → moved to SpellHandler)
    // (SMSG_PROCRESIST → moved to CombatHandler)
    // -----------------------------------------------------------------------

    // -----------------------------------------------------------------------
    // Pet stable
    // -----------------------------------------------------------------------
    dispatchTable_[Opcode::MSG_LIST_STABLED_PETS] = [this](network::Packet& packet) {
        if (state == WorldState::IN_WORLD) handleListStabledPets(packet);
    };
    dispatchTable_[Opcode::SMSG_STABLE_RESULT] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(1)) return;
        uint8_t result = packet.readUInt8();
        const char* msg = nullptr;
        switch (result) {
            case 0x01: msg = "Pet stored in stable."; break;
            case 0x06: msg = "Pet retrieved from stable."; break;
            case 0x07: msg = "Stable slot purchased."; break;
            case 0x08: msg = "Stable list updated."; break;
            case 0x09: msg = "Stable failed: not enough money or other error."; addUIError(msg); break;
            default: break;
        }
        if (msg) addSystemChatMessage(msg);
        LOG_INFO("SMSG_STABLE_RESULT: result=", static_cast<int>(result));
        if (stableWindowOpen_ && stableMasterGuid_ != 0 && socket && result <= 0x08) {
            auto refreshPkt = ListStabledPetsPacket::build(stableMasterGuid_);
            socket->send(refreshPkt);
        }
    };

    // -----------------------------------------------------------------------
    // Titles / achievements / character services
    // -----------------------------------------------------------------------
    dispatchTable_[Opcode::SMSG_TITLE_EARNED] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint32_t titleBit = packet.readUInt32();
        uint32_t isLost   = packet.readUInt32();
        loadTitleNameCache();
        std::string titleStr;
        auto tit = titleNameCache_.find(titleBit);
        if (tit != titleNameCache_.end() && !tit->second.empty()) {
            const auto& ln = lookupName(playerGuid);
            const std::string& pName = ln.empty() ? std::string("you") : ln;
            const std::string& fmt = tit->second;
            size_t pos = fmt.find("%s");
            if (pos != std::string::npos)
                titleStr = fmt.substr(0, pos) + pName + fmt.substr(pos + 2);
            else
                titleStr = fmt;
        }
        std::string msg;
        if (!titleStr.empty()) {
            msg = isLost ? ("Title removed: " + titleStr + ".") : ("Title earned: " + titleStr + "!");
        } else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), isLost ? "Title removed (bit %u)." : "Title earned (bit %u)!", titleBit);
            msg = buf;
        }
        if (isLost) knownTitleBits_.erase(titleBit);
        else        knownTitleBits_.insert(titleBit);
        // And announce it. The set is read by IsTitleKnown, which the character
        // sheet's title dropdown builds itself from - but only when told to
        // rebuild, and this is what tells it. Without it a title earned mid
        // session did not appear in the list until the sheet was reopened.
        fireAddonEvent("KNOWN_TITLES_UPDATE", {});
        addSystemChatMessage(msg);
        LOG_INFO("SMSG_TITLE_EARNED: bit=", titleBit, " lost=", isLost, " title='", titleStr, "'");
    };
    dispatchTable_[Opcode::SMSG_LEARNED_DANCE_MOVES] = [](network::Packet& packet) {
        LOG_DEBUG("SMSG_LEARNED_DANCE_MOVES: ignored (size=", packet.getSize(), ")");
    };
    dispatchTable_[Opcode::SMSG_CHAR_RENAME] = [this](network::Packet& packet) {
        // uint8 result, and the guid and the new name only when it succeeded.
        //
        // This read a four-byte result and required thirteen bytes. A refusal
        // is one byte, so no rename error was ever shown; a success carries the
        // name, so it usually cleared the guard and then took three bytes of
        // the guid as part of the result - which read as an enormous number,
        // fell past the error table and reported the rename as failed.
        if (packet.hasRemaining(1)) {
            const uint8_t result = packet.readUInt8();
            std::string newName;
            if (result == 0 && packet.hasRemaining(9)) {
                /*uint64_t guid =*/ packet.readUInt64();
                newName = packet.readString();
            }
            if (result == 0) {
                addSystemChatMessage("Character name changed to: " + newName);
            } else {
                static const char* kRenameErrors[] = {
                    nullptr, "Name already in use.", "Name too short.", "Name too long.",
                    "Name contains invalid characters.", "Name contains a profanity.",
                    "Name is reserved.", "Character name does not meet requirements.",
                };
                const char* errMsg = (result < 8) ? kRenameErrors[result] : nullptr;
                std::string renameErr = errMsg ? std::string("Rename failed: ") + errMsg : "Character rename failed.";
                addUIError(renameErr); addSystemChatMessage(renameErr);
            }
            LOG_INFO("SMSG_CHAR_RENAME: result=", static_cast<int>(result),
                     " newName=", newName);
        }
    };

    // -----------------------------------------------------------------------
    // Bind / heartstone / phase / barber / corpse
    // -----------------------------------------------------------------------
    dispatchTable_[Opcode::SMSG_PLAYERBOUND] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(16)) return;
        /*uint64_t binderGuid =*/ packet.readUInt64();
        uint32_t mapId  = packet.readUInt32();
        uint32_t zoneId = packet.readUInt32();
        bool changed = !hasHomeBind_ || homeBindMapId_ != mapId || homeBindZoneId_ != zoneId;
        homeBindMapId_  = mapId;
        homeBindZoneId_ = zoneId;
        if (!changed) return;
        std::string pbMsg = "Your home location has been set";
        std::string zoneName = getAreaName(zoneId);
        if (!zoneName.empty()) pbMsg += " to " + zoneName;
        pbMsg += '.';
        addSystemChatMessage(pbMsg);
    };
    // The innkeeper asking whether to make this the player's home. It was
    // skipped because this client's own gossip window never waited for it - it
    // matches the option's English text and sends the activate itself. FrameXML
    // draws the gossip window now and follows the real flow: the server asks,
    // a popup accepts, and the reply goes back.
    dispatchTable_[Opcode::SMSG_BINDER_CONFIRM] = [this](network::Packet& packet) {
        binderGuid_ = packet.hasRemaining(8) ? packet.readUInt64() : 0;
        // Who is asking. StaticPopup_Show("CONFIRM_BINDER", arg1) puts the name
        // into "Do you want to make <name> your new home?" - without it the
        // question had a hole where the innkeeper should be.
        fireAddonEvent("CONFIRM_BINDER", {lookupName(binderGuid_)});
    };
    registerSkipHandler(Opcode::SMSG_SET_PHASE_SHIFT);
    dispatchTable_[Opcode::SMSG_TOGGLE_XP_GAIN] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(1)) return;
        uint8_t enabled = packet.readUInt8();
        addSystemChatMessage(enabled ? "XP gain enabled." : "XP gain disabled.");
    };
    dispatchTable_[Opcode::SMSG_BINDZONEREPLY] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t result = packet.readUInt32();
            if (result == 0) addSystemChatMessage("Your home is now set to this location.");
            else { raiseUiError("You are too far from the innkeeper."); }
        }
    };
    dispatchTable_[Opcode::SMSG_CHANGEPLAYER_DIFFICULTY_RESULT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t result = packet.readUInt32();
            if (result == 0) {
                addSystemChatMessage("Difficulty changed.");
            } else {
                static const char* reasons[] = {
                    "", "Error", "Too many members", "Already in dungeon",
                    "You are in a battleground", "Raid not allowed in heroic",
                    "You must be in a raid group", "Player not in group"
                };
                const char* msg = (result < 8) ? reasons[result] : "Difficulty change failed.";
                addUIError(std::string("Cannot change difficulty: ") + msg);
                addSystemChatMessage(std::string("Cannot change difficulty: ") + msg);
            }
        }
    };
    dispatchTable_[Opcode::SMSG_CORPSE_NOT_IN_INSTANCE] = [this](network::Packet& /*packet*/) {
        addUIError("Your corpse is outside this instance.");
        addSystemChatMessage("Your corpse is outside this instance. Release spirit to retrieve it.");
    };
    dispatchTable_[Opcode::SMSG_CROSSED_INEBRIATION_THRESHOLD] = [this](network::Packet& packet) {
        if (packet.hasRemaining(16)) {
            uint64_t guid      = packet.readUInt64();
            uint32_t threshold = packet.readUInt32();
            uint32_t itemId    = packet.readUInt32();
            if (guid == playerGuid) {
                const float amount = static_cast<float>(std::min(threshold, 3u)) / 3.0f;
                if (threshold == 1) addSystemChatMessage("You feel tipsy.");
                else if (threshold == 2) addSystemChatMessage("You feel drunk.");
                else if (threshold >= 3) addSystemChatMessage("You feel completely smashed.");
                else addSystemChatMessage("You feel sober again.");
                if (auto* renderer = services_.renderer) {
                    if (auto* camera = renderer->getCameraController()) {
                        camera->setIntoxication(amount);
                    }
                    if (auto* post = renderer->getPostProcessPipeline()) {
                        post->setIntoxication(amount);
                    }
                }
            }
            LOG_DEBUG("SMSG_CROSSED_INEBRIATION_THRESHOLD: guid=0x", std::hex, guid,
                      std::dec, " threshold=", threshold, " itemId=", itemId);
        }
    };
    dispatchTable_[Opcode::SMSG_CLEAR_FAR_SIGHT_IMMEDIATE] = [](network::Packet& /*packet*/) {
        LOG_DEBUG("SMSG_CLEAR_FAR_SIGHT_IMMEDIATE");
    };
    registerSkipHandler(Opcode::SMSG_COMBAT_EVENT_FAILED);
    dispatchTable_[Opcode::SMSG_FORCE_ANIM] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint64_t animGuid = packet.readPackedGuid();
            if (packet.hasRemaining(4)) {
                uint32_t animId = packet.readUInt32();
                if (emoteAnimCallback_) emoteAnimCallback_(animGuid, animId, /*isState=*/false);
            }
        }
    };
    // Consume silently - opcodes we receive but don't need to act on
    for (auto op : {
        Opcode::SMSG_FLIGHT_SPLINE_SYNC, Opcode::SMSG_FORCE_DISPLAY_UPDATE,
        Opcode::SMSG_FORCE_SEND_QUEUED_PACKETS, Opcode::SMSG_FORCE_SET_VEHICLE_REC_ID,
        Opcode::SMSG_CORPSE_MAP_POSITION_QUERY_RESPONSE, Opcode::SMSG_DAMAGE_CALC_LOG,
        Opcode::SMSG_DYNAMIC_DROP_ROLL_RESULT, Opcode::SMSG_DESTRUCTIBLE_BUILDING_DAMAGE,
    }) { registerSkipHandler(op); }

    // Game object despawn animation - reset state to closed before actual despawn
    dispatchTable_[Opcode::SMSG_GAMEOBJECT_DESPAWN_ANIM] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint64_t guid = packet.readUInt64();
        // Trigger a CLOSE animation / freeze before the object is removed
        if (gameObjectStateCallback_) gameObjectStateCallback_(guid, 0);
    };
    // Game object reset state - return to READY(closed) state
    dispatchTable_[Opcode::SMSG_GAMEOBJECT_RESET_STATE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint64_t guid = packet.readUInt64();
        if (gameObjectStateCallback_) gameObjectStateCallback_(guid, 0);
    };
    dispatchTable_[Opcode::SMSG_FORCED_DEATH_UPDATE] = [this](network::Packet& packet) {
        playerDead_ = true;
        corpseX_ = movementInfo.y;
        corpseY_ = movementInfo.x;
        corpseZ_ = movementInfo.z;
        corpseMapId_ = currentMapId_;
        corpsePositionValid_ = true;
        if (ghostStateCallback_) ghostStateCallback_(false);
        fireAddonEvent("PLAYER_DEAD", {});
        addSystemChatMessage("You have been killed.");
        LOG_INFO("SMSG_FORCED_DEATH_UPDATE: player force-killed");
        packet.skipAll();
    };
    // SMSG_DEFENSE_MESSAGE - moved to ChatHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_CORPSE_RECLAIM_DELAY] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t delayMs = packet.readUInt32();
            auto nowMs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            corpseReclaimAvailableMs_ = nowMs + delayMs;
            LOG_INFO("SMSG_CORPSE_RECLAIM_DELAY: ", delayMs, "ms");
        }
    };
    dispatchTable_[Opcode::SMSG_DEATH_RELEASE_LOC] = [this](network::Packet& packet) {
        if (packet.hasRemaining(16)) {
            uint32_t relMapId = packet.readUInt32();
            float relX = packet.readFloat(), relY = packet.readFloat(), relZ = packet.readFloat();
            // Read and logged and nothing else, until now: this is where the
            // server says the nearest spirit healer is, and a ghost with no
            // marker for it has to go looking.
            //
            // Map id -1 is the withdrawal. ResurrectPlayer sends it to take the
            // marker down, so it is not a graveyard at (0,0,0).
            if (relMapId == 0xFFFFFFFFu) {
                deathReleaseValid_ = false;
                LOG_INFO("SMSG_DEATH_RELEASE_LOC: spirit healer position cleared");
            } else {
                deathReleaseValid_ = true;
                deathReleaseMapId_ = relMapId;
                deathReleaseCanonical_ =
                    core::coords::serverToCanonical(glm::vec3(relX, relY, relZ));
                LOG_INFO("SMSG_DEATH_RELEASE_LOC (graveyard spawn): map=", relMapId,
                         " x=", relX, " y=", relY, " z=", relZ);
            }
        }
    };
    dispatchTable_[Opcode::SMSG_ENABLE_BARBER_SHOP] = [this](network::Packet& /*packet*/) {
        LOG_INFO("SMSG_ENABLE_BARBER_SHOP: barber shop available");
        barberShopOpen_ = true;
        fireAddonEvent("BARBER_SHOP_OPEN", {});
    };

    // ---- Batch 3: Corpse/gametime, combat clearing, mount, loot notify,
    //                movement/speed/flags, attack, spells, group ----

    dispatchTable_[Opcode::MSG_CORPSE_QUERY] = [this](network::Packet& packet) {
        // found(1) + travelMapId(4) + x,y,z(12) + corpseMapId(4) + unk(4).
        //
        // Two map ids, and they are not the same question. The first is where
        // the coordinates are: for a corpse left in a dungeon the server
        // answers with the instance's *entrance* on the outdoor map, because
        // that is where a ghost has to walk to. The second is the map the
        // corpse is actually lying on.
        //
        // This paired the coordinates with the second, and everything that
        // reads corpseMapId_ compares it against the map the player is
        // standing on - every other writer sets it to exactly that. So a
        // corpse in a dungeon gave a position on the outdoor map filed under
        // the dungeon's id, and the check `currentMapId_ != corpseMapId_`
        // refused to show it precisely where it would have been useful.
        if (!packet.hasRemaining(1)) return;
        const uint8_t found = packet.readUInt8();
        if (!found || !packet.hasRemaining(20)) return;
        const uint32_t travelMapId = packet.readUInt32();
        const float cx = packet.readFloat();
        const float cy = packet.readFloat();
        const float cz = packet.readFloat();
        const uint32_t corpseMapId = packet.readUInt32();

        corpseX_ = cx;
        corpseY_ = cy;
        corpseZ_ = cz;
        corpseMapId_ = travelMapId;
        corpsePositionValid_ = true;
        LOG_INFO("MSG_CORPSE_QUERY: walk to (", cx, ",", cy, ",", cz,
                 ") on map ", travelMapId,
                 corpseMapId != travelMapId
                     ? " (the corpse itself is inside map " + std::to_string(corpseMapId) + ")"
                     : "");
    };
    dispatchTable_[Opcode::SMSG_FEIGN_DEATH_RESISTED] = [this](network::Packet& /*packet*/) {
        addUIError("Your Feign Death was resisted.");
        addSystemChatMessage("Your Feign Death attempt was resisted.");
    };
    dispatchTable_[Opcode::SMSG_CHANNEL_MEMBER_COUNT] = [](network::Packet& packet) {
        std::string chanName = packet.readString();
        if (packet.hasRemaining(5)) {
            /*uint8_t flags =*/ packet.readUInt8();
            uint32_t count = packet.readUInt32();
            LOG_DEBUG("SMSG_CHANNEL_MEMBER_COUNT: channel=", chanName, " members=", count);
        }
    };
    for (auto op : { Opcode::SMSG_GAMETIME_SET, Opcode::SMSG_GAMETIME_UPDATE }) {
        dispatchTable_[op] = [this](network::Packet& packet) {
            if (packet.hasRemaining(4)) {
                const WowDate t = unpackWowPackedTime(packet.readUInt32());
                gameTime_ = static_cast<float>(t.hour) +
                            static_cast<float>(t.minute) / 60.0f;
                // Said out loud, because this is where realm time actually
                // comes from on servers that never send SMSG_LOGIN_SETTIMESPEED
                // - which logged its arrival and so looked like the only
                // source. Chasing a frozen sky through a log with no line for
                // this in it led to exactly the wrong conclusion.
                LOG_INFO("Realm time: ", t.hour, ":", t.minute,
                         " (gameTime=", gameTime_, "h)");
            }
            packet.skipAll();
        };
    }
    dispatchTable_[Opcode::SMSG_GAMESPEED_SET] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            const WowDate t = unpackWowPackedTime(packet.readUInt32());
            float timeSpeed = packet.readFloat();
            gameTime_ = static_cast<float>(t.hour) +
                        static_cast<float>(t.minute) / 60.0f;
            timeSpeed_ = timeSpeed;
        }
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_GAMETIMEBIAS_SET] = [](network::Packet& packet) {
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_ACHIEVEMENT_DELETED] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t achId = packet.readUInt32();
            earnedAchievements_.erase(achId);
            achievementDates_.erase(achId);
        }
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_CRITERIA_DELETED] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t critId = packet.readUInt32();
            criteriaProgress_.erase(critId);
        }
        packet.skipAll();
    };

    // Combat clearing
    dispatchTable_[Opcode::SMSG_BREAK_TARGET] = [this](network::Packet& packet) {
        // Packed, not plain: Unit::BreakTarget sizes the packet with
        // GetPackGUID().size() and writes GetPackGUID(). Read as a uint64 the
        // guid never matched the target, so the target was never dropped.
        if (packet.hasRemaining(1)) {
            uint64_t bGuid = packet.readPackedGuid();
            if (bGuid == targetGuid) targetGuid = 0;
        }
    };
    dispatchTable_[Opcode::SMSG_CLEAR_TARGET] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint64_t cGuid = packet.readUInt64();
            if (cGuid == 0 || cGuid == targetGuid) targetGuid = 0;
        }
    };

    // Mount/dismount
    dispatchTable_[Opcode::SMSG_DISMOUNT] = [this](network::Packet& packet) {
        // TBC/WotLK identify the dismounting unit with a packed GUID. Applying
        // every nearby unit's dismount to the local player made an enemy
        // dismounting to attack knock us off our own mount. Classic variants
        // may send the legacy empty packet, which is implicitly local.
        uint64_t dismountGuid = playerGuid;
        if (packet.hasRemaining(1)) {
            dismountGuid = packet.readPackedGuid();
            if (dismountGuid == 0) {
                LOG_WARNING("Ignoring SMSG_DISMOUNT with an invalid packed GUID");
                return;
            }
        }
        if (dismountGuid != playerGuid) {
            LOG_DEBUG("Remote SMSG_DISMOUNT: guid=0x", std::hex,
                      dismountGuid, std::dec);
            if (otherPlayerMountCallback_) {
                otherPlayerMountCallback_(dismountGuid, 0);
            }
            return;
        }

        // Live-confirmed: CMaNGOS sends this partway through a taxi flight (its
        // own server-side flight-completion estimate firing early, well before
        // the client-simulated path actually finishes) - obeying it unconditionally
        // cancelled the taxi mount animation while updateClientTaxi() kept flying
        // the real path for several more seconds, seen as "walking in the air".
        // But other cores (e.g. AzerothCore) finalize taxi flights server-side -
        // dismounting/stopping the player is the *authoritative* completion
        // signal there, not a premature estimate - so a blanket "ignore while
        // flying" guard could let the client fly past a real landing if local
        // spline timing ever drifts from the server's. Distinguish the two using
        // the server's own UNIT_FLAG_TAXI_FLIGHT: still set means this is the
        // premature-estimate quirk (ignore, let updateClientTaxi() finish
        // naturally); already cleared means the server considers the flight
        // genuinely over (honor it now instead of waiting on our own spline).
        const bool onTaxiFlight = movementHandler_ && movementHandler_->isOnTaxiFlight();
        bool serverStillTaxiing = false;
        if (onTaxiFlight) {
            auto playerEntity = entityController_->getEntityManager().getEntity(playerGuid);
            auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
            if (playerUnit) {
                serverStillTaxiing = (playerUnit->getUnitFlags() & UNIT_FLAG_TAXI_FLIGHT) != 0;
            }
        }
        const bool nearDestination = onTaxiFlight && movementHandler_ &&
                                     movementHandler_->isNearTaxiDestination();
        LOG_INFO("SMSG_DISMOUNT received: onTaxiFlight=", onTaxiFlight,
                 " serverStillTaxiing=", serverStillTaxiing,
                 " nearDestination=", nearDestination);
        if (onTaxiFlight && (serverStillTaxiing || !nearDestination)) {
            movementHandler_->deferServerTaxiCompletion();
            LOG_WARNING("Deferring premature SMSG_DISMOUNT until taxi landing zone");
            return;
        }
        if (onTaxiFlight && movementHandler_) {
            // Authoritative server completion ahead of our own spline - stop the
            // client flight now rather than snapping to a final waypoint that may
            // not match where the server actually stopped us.
            movementHandler_->finishClientTaxiFlight(/*snapToFinalWaypoint=*/false);
            return;
        }

        // UNIT_FIELD_MOUNTDISPLAYID is the authoritative persistent mount
        // state.  Some realms emit an isolated SMSG_DISMOUNT while processing
        // damage (notably periodic poison) without actually removing the mount
        // aura or clearing the update field.  Treating that transient packet as
        // state made the local model dismount even though the player remained
        // mounted server-side.  A real dismount also clears the update field;
        // let that values update drive the visual when the two signals disagree.
        if (isActiveExpansion("wotlk")) {
            const uint16_t mountField = fieldIndex(UF::UNIT_FIELD_MOUNTDISPLAYID);
            auto playerEntity = entityController_->getEntityManager().getEntity(playerGuid);
            const uint32_t serverMountDisplay =
                (playerEntity && mountField != 0xFFFF && playerEntity->hasField(mountField))
                    ? playerEntity->getField(mountField)
                    : 0;
            // ...unless the player asked to dismount, in which case a still-set
            // field is exactly the staleness this packet is arriving to correct.
            // Discarding it there threw away the one authoritative confirmation
            // the dismount had.
            const bool awaitingOurOwnDismount =
                movementHandler_ && movementHandler_->isDismountPending();
            if (serverMountDisplay != 0 && !awaitingOurOwnDismount) {
                LOG_WARNING("Ignoring transient SMSG_DISMOUNT while authoritative mount field is ",
                            serverMountDisplay,
                            " casting=", isCasting(),
                            " channeling=", isChanneling(),
                            " spell=", getCurrentCastSpellId());
                return;
            }
            if (movementHandler_) movementHandler_->clearDismountPending();
        }
        currentMountDisplayId_ = 0;
        if (mountCallback_) mountCallback_(0);
    };
    dispatchTable_[Opcode::SMSG_MOUNTRESULT] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t result = packet.readUInt32();
        if (result != 4) {
            const char* msgs[] = { "Cannot mount here.", "Invalid mount spell.",
                                   "Too far away to mount.", "Already mounted." };
            std::string mountErr = result < 4 ? msgs[result] : "Cannot mount.";
            addUIError(mountErr);
            addSystemChatMessage(mountErr);
        }
    };
    dispatchTable_[Opcode::SMSG_DISMOUNTRESULT] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t result = packet.readUInt32();
        if (result != 0) {
            addUIError("Cannot dismount here.");
            raiseUiError("Cannot dismount here.");
        }
    };

    // Camera shake
    dispatchTable_[Opcode::SMSG_CAMERA_SHAKE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint32_t shakeId   = packet.readUInt32();
            uint32_t shakeType = packet.readUInt32();
            (void)shakeType;
            float magnitude = (shakeId < 50) ? 0.04f : 0.08f;
            if (cameraShakeCallback_)
                cameraShakeCallback_(magnitude, 18.0f, 0.5f);
        }
    };

    // (SMSG_PLAY_SPELL_VISUAL, SMSG_CLEAR_COOLDOWN, SMSG_MODIFY_COOLDOWN → moved to SpellHandler)

    // ---- Batch 4: Ready check, duels, guild, loot/gossip/vendor, factions, spell mods ----

    // Guild
    registerHandler(Opcode::SMSG_PET_SPELLS, &GameHandler::handlePetSpells);

    // Loot/gossip/vendor delegates
    registerHandler(Opcode::SMSG_SUMMON_REQUEST, &GameHandler::handleSummonRequest);
    dispatchTable_[Opcode::SMSG_SUMMON_CANCEL] = [this](network::Packet& /*packet*/) {
        pendingSummonRequest_ = false;
        // social_handler raised the prompt with CONFIRM_SUMMON; uiparent.lua
        // takes this one down again. Told only in chat, the dialog stays on
        // screen offering a summon the server has already withdrawn.
        fireAddonEvent("CANCEL_SUMMON", {});
        addSystemChatMessage("Summon cancelled.");
    };

    // Bind point
    dispatchTable_[Opcode::SMSG_BINDPOINTUPDATE] = [this](network::Packet& packet) {
        BindPointUpdateData data;
        if (BindPointUpdateParser::parse(packet, data)) {
            glm::vec3 canonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            bool wasSet = hasHomeBind_;
            bool changed =
                !hasHomeBind_ ||
                homeBindMapId_ != data.mapId ||
                homeBindZoneId_ != data.zoneId ||
                glm::length(homeBindPos_ - canonical) > 0.5f;
            hasHomeBind_ = true;
            homeBindMapId_ = data.mapId;
            homeBindZoneId_ = data.zoneId;
            homeBindPos_ = canonical;
            if (bindPointCallback_)
                bindPointCallback_(data.mapId, canonical.x, canonical.y, canonical.z);
            if (wasSet && changed) {
                std::string bindMsg = "Your home has been set";
                std::string zoneName = getAreaName(data.zoneId);
                if (!zoneName.empty()) bindMsg += " to " + zoneName;
                bindMsg += '.';
                addSystemChatMessage(bindMsg);
            }
        }
    };

    // Spirit healer / resurrect
    dispatchTable_[Opcode::SMSG_SPIRIT_HEALER_CONFIRM] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint64_t npcGuid = packet.readUInt64();
        if (npcGuid) {
            resurrectCasterGuid_ = npcGuid;
            resurrectCasterName_ = "";
            resurrectIsSpiritHealer_ = true;
            resurrectRequestPending_ = true;
            // The spirit healer's prompt is CONFIRM_XP_LOSS, not
            // RESURRECT_REQUEST - uiparent.lua answers it with the durability
            // and sickness warning and its Accept calls AcceptXPLoss. Nothing
            // fired it, and the only thing that could reach
            // activateSpiritHealer was this client's own gossip window, so with
            // gossip handed over there was no way left to accept at all.
            fireAddonEvent("CONFIRM_XP_LOSS", {});
        }
    };
    dispatchTable_[Opcode::SMSG_RESURRECT_REQUEST] = [this](network::Packet& packet) {
        // Spell::SendResurrectRequest writes:
        //
        //   uint64 casterGuid
        //   uint32 nameLength      (the name plus its terminator)
        //   char[] name            with that terminator
        //   uint8  0               a second one, written explicitly
        //   uint8  sickness        1 from a creature, 0 from a player
        //   uint32 0               only when the spell overrides the timer
        //
        // The length was never read, so readString took the first byte of it
        // as the whole name - a control character, for any name shorter than
        // two hundred and fifty-five - and the dialog was raised offering a
        // resurrection from that. Everything after it was out of step too, so
        // the sickness flag was never read at all.
        if (!packet.hasRemaining(12)) { packet.skipAll(); return; }
        const uint64_t casterGuid = packet.readUInt64();
        /*uint32_t nameLength =*/ packet.readUInt32();
        std::string casterName = packet.readString();
        if (packet.hasRemaining(1)) packet.readUInt8();   // the second terminator
        const bool sickness = packet.hasRemaining(1) && packet.readUInt8() != 0;
        // The trailing word is present only when the spell waives the reclaim
        // delay, so having one means there is no timer to wait out.
        const bool hasTimer = !packet.hasRemaining(4);
        packet.skipAll();

        if (!casterGuid) return;
        resurrectCasterGuid_ = casterGuid;
        // A creature offering this is a spirit healer, and that is what the
        // sickness flag says - it is set for every non-player caster.
        resurrectIsSpiritHealer_ = false;
        resurrectHasSickness_ = sickness;
        resurrectHasTimer_ = hasTimer;
        resurrectCasterName_ = casterName.empty() ? lookupName(casterGuid) : casterName;
        resurrectRequestPending_ = true;
        fireAddonEvent("RESURRECT_REQUEST", {resurrectCasterName_});
    };

    // Time sync
    dispatchTable_[Opcode::SMSG_TIME_SYNC_REQ] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t counter = packet.readUInt32();
        if (socket) {
            network::Packet resp(wireOpcode(Opcode::CMSG_TIME_SYNC_RESP));
            resp.writeUInt32(counter);
            resp.writeUInt32(nextMovementTimestampMs());
            socket->send(resp);
        }
    };

    // (SMSG_TRAINER_BUY_SUCCEEDED, SMSG_TRAINER_BUY_FAILED → moved to InventoryHandler)

    // Minimap ping
    dispatchTable_[Opcode::MSG_MINIMAP_PING] = [this](network::Packet& packet) {
        const bool mmTbcLike = isPreWotlk();
        if (!packet.hasRemaining(mmTbcLike ? 8u : 1u) ) return;
        uint64_t senderGuid = mmTbcLike
            ? packet.readUInt64() : packet.readPackedGuid();
        if (!packet.hasRemaining(8)) return;
        float pingX = packet.readFloat();
        float pingY = packet.readFloat();
        MinimapPing ping;
        ping.senderGuid = senderGuid;
        ping.wowX = pingY;
        ping.wowY = pingX;
        ping.age  = 0.0f;
        minimapPings_.push_back(ping);
        if (senderGuid != playerGuid) {
                            withSoundManager(&audio::AudioCoordinator::getUiSoundManager, [](auto* sfx) { sfx->playMinimapPing(); });
        }
        // The ping is drawn by this client from minimapPings_ and was never
        // announced, so a handed-over minimap heard the sound and showed
        // nothing. The interface wants the unit that pinged and the position,
        // and marks the spot itself.
        //
        // x then y, in that order, which is the reverse of how they are stored
        // above: the wire sends them the other way round and this client swaps
        // them on the way in. Passing the stored pair straight through would
        // put every ping at its own mirror image.
        std::string unitId = guidToUnitId(senderGuid);
        if (unitId.empty()) unitId = "player";
        fireAddonEvent("MINIMAP_PING", {unitId,
                                        std::to_string(ping.wowY),
                                        std::to_string(ping.wowX)});
    };
    dispatchTable_[Opcode::SMSG_ZONE_UNDER_ATTACK] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t areaId = packet.readUInt32();
            std::string areaName = getAreaName(areaId);
            // The zone name goes with it: the interface writes its own warning
            // from arg1 rather than reusing the chat line.
            if (addonEventCallback_) addonEventCallback_("ZONE_UNDER_ATTACK", {areaName});
            // ...which is why this client only writes the line when nothing
            // else will. chatframe.lua's own ZONE_UNDER_ATTACK branch calls
            // AddMessage with ZONE_UNDER_ATTACK formatted from arg1, so with
            // the interface in charge the player read every attack twice in
            // chat and a third time on the error line - which the real client
            // never puts it on at all. Booty Bay under sustained attack filled
            // the window.
            if (!ui::frameXmlOwns(ui::UiElement::Chat)) {
                const std::string msg = areaName.empty()
                    ? std::string("A zone is under attack!")
                    : (areaName + " is under attack!");
                addUIError(msg);
                addSystemChatMessage(msg);
            }
        }
    };

    // Spirit healer time / durability
    dispatchTable_[Opcode::SMSG_AREA_SPIRIT_HEALER_TIME] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(12)) { packet.skipAll(); return; }
        const uint64_t guid = packet.readUInt64();
        const uint32_t timeMs = packet.readUInt32();
        areaSpiritHealerGuid_ = guid;
        areaSpiritHealerSeconds_ = static_cast<float>(timeMs) / 1000.0f;

        char buf[128];
        std::snprintf(buf, sizeof(buf), "You will be able to resurrect in %u seconds.",
                      timeMs / 1000);
        addSystemChatMessage(buf);

        // Parsed and said in chat and announced to nobody. UIParent_OnEvent
        // answers this by joining the queue and raising the countdown dialog,
        // and it was never told - so the reply arrived, the line was printed,
        // and the resurrection it was counting down to was one this player had
        // not joined.
        fireAddonEvent("AREA_SPIRIT_HEALER_IN_RANGE", {});
    };
    dispatchTable_[Opcode::SMSG_DURABILITY_DAMAGE_DEATH] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t pct = packet.readUInt32();
            char buf[80];
            std::snprintf(buf, sizeof(buf),
                "You have lost %u%% of your gear's durability due to death.", pct);
            addUIError(buf);
            addSystemChatMessage(buf);
        }
    };

    // (SMSG_INITIALIZE_FACTIONS, SMSG_SET_FACTION_STANDING,
    //  SMSG_SET_FACTION_ATWAR, SMSG_SET_FACTION_VISIBLE → moved to SocialHandler)
    dispatchTable_[Opcode::SMSG_FEATURE_SYSTEM_STATUS] = [](network::Packet& packet) {
        packet.skipAll();
    };

    // (SMSG_SET_FLAT_SPELL_MODIFIER, SMSG_SET_PCT_SPELL_MODIFIER, SMSG_SPELL_DELAYED → moved to SpellHandler)

    // Proficiency
    dispatchTable_[Opcode::SMSG_SET_PROFICIENCY] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(5)) return;
        uint8_t  itemClass = packet.readUInt8();
        uint32_t mask      = packet.readUInt32();
        if (itemClass == 2) weaponProficiency_ = mask;
        else if (itemClass == 4) armorProficiency_ = mask;
    };

    // Loot money / misc consume
    for (auto op : { Opcode::SMSG_LOOT_CLEAR_MONEY, Opcode::SMSG_NPC_TEXT_UPDATE }) {
        dispatchTable_[op] = [](network::Packet& /*packet*/) {};
    }

    // Play sound
    dispatchTable_[Opcode::SMSG_PLAY_SOUND] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t soundId = packet.readUInt32();
            if (playSoundCallback_) playSoundCallback_(soundId);
        }
    };

    // SMSG_SERVER_MESSAGE - moved to ChatHandler::registerOpcodes
    // SMSG_CHAT_SERVER_MESSAGE - moved to ChatHandler::registerOpcodes
    // SMSG_AREA_TRIGGER_MESSAGE - moved to ChatHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_TRIGGER_CINEMATIC] = [this](network::Packet& packet) {
        packet.skipAll();
        network::Packet ack(wireOpcode(Opcode::CMSG_NEXT_CINEMATIC_CAMERA));
        socket->send(ack);
    };

    // ---- Batch 5: Teleport, taxi, BG, LFG, arena, movement relay, mail, bank, auction, quests ----

    // Teleport
    dispatchTable_[Opcode::SMSG_TRANSFER_PENDING] = [](network::Packet& packet) {
        uint32_t pendingMapId = packet.readUInt32();
        if (packet.hasRemaining(8)) {
            packet.readUInt32(); // transportEntry
            packet.readUInt32(); // transportMapId
        }
        (void)pendingMapId;
    };
    dispatchTable_[Opcode::SMSG_TRANSFER_ABORTED] = [this](network::Packet& packet) {
        // uint32 mapId + uint8 reason, and a further uint8 for exactly three of
        // them - the expansion level, the difficulty and the unique message.
        //
        // Every code from 0x01 on named the wrong failure. The table here was
        // shifted against Player.h's own enum: 0x02 is a full instance and was
        // read as a missing expansion, 0x06 is an encounter in progress and was
        // read as a full instance, 0x07 is the missing expansion and was read
        // as the encounter. Nothing above 0x09 was known at all, which
        // includes the one a map refusing entry outright sends.
        //
        // So a refusal always said something, and what it said was not why.
        if (!packet.hasRemaining(4)) { packet.skipAll(); return; }
        /*uint32_t mapId =*/ packet.readUInt32();
        const uint8_t reason = packet.hasRemaining(1) ? packet.readUInt8() : 0;
        const char* abortMsg = nullptr;
        switch (reason) {
            case 0x01: abortMsg = "Transfer aborted: an error occurred."; break;
            case 0x02: abortMsg = "Transfer aborted: instance is full."; break;
            case 0x03: abortMsg = "Transfer aborted: instance not found."; break;
            case 0x04: abortMsg = "You have entered too many instances recently."; break;
            case 0x06: abortMsg = "Unable to zone in while an encounter is in progress."; break;
            case 0x07: abortMsg = "You do not have the expansion required to enter this area."; break;
            case 0x08: abortMsg = "That difficulty is not available for this instance."; break;
            case 0x09: abortMsg = "You cannot leave this place yet."; break;
            case 0x0A: abortMsg = "Additional instances cannot be launched. Please try again later."; break;
            case 0x0B: abortMsg = "Transfer aborted: you must be in a group to enter."; break;
            case 0x0C:
            case 0x0D:
            case 0x0E: abortMsg = "Transfer aborted: instance not found."; break;
            case 0x0F: abortMsg = "Transfer aborted: everyone in the party must be on the same realm."; break;
            case 0x10: abortMsg = "This map cannot be entered at this time."; break;
            default:   abortMsg = "Transfer aborted."; break;
        }
        packet.skipAll();
        addUIError(abortMsg);
        addSystemChatMessage(abortMsg);
        LOG_WARNING("SMSG_TRANSFER_ABORTED: reason 0x", std::hex,
                    static_cast<int>(reason), std::dec, " - ", abortMsg);
    };

    // Taxi
    dispatchTable_[Opcode::SMSG_STANDSTATE_UPDATE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            const uint8_t newStandState = packet.readUInt8();
            if (newStandState == standState_) {
                return;
            }
            standState_ = newStandState;
            // 0=stand, 1=sit, 2-6=sit variants, 7=dead, 8=kneel. Logged because a
            // wrong state here is indistinguishable, in-game, from a wrong animation.
            // At warning level: the file log filters info out by default.
            LOG_WARNING("SMSG_STANDSTATE_UPDATE: standState=", static_cast<int>(standState_));
            if (standStateCallback_) standStateCallback_(standState_);
        }
    };
    dispatchTable_[Opcode::SMSG_NEW_TAXI_PATH] = [this](network::Packet& /*packet*/) {
        addSystemChatMessage("New flight path discovered!");
    };

    // (MSG_TALENT_WIPE_CONFIRM → registered by SpellHandler, which owns the
    // pending-wipe state the confirm dialog reads. Handling it here wrote to
    // dead duplicate members and the dialog never appeared.)

    // (SMSG_CHANNEL_LIST → moved to ChatHandler)
    // (SMSG_GROUP_SET_LEADER → moved to SocialHandler)

    // Gameobject / page text (entity queries moved to EntityController::registerOpcodes)
    dispatchTable_[Opcode::SMSG_GAMEOBJECT_CUSTOM_ANIM] = [this](network::Packet& packet) {
        if (packet.getSize() < 12) return;
        uint64_t guid = packet.readUInt64();
        uint32_t animId = packet.readUInt32();
        bool ownedFishingBite = false;
        auto goEnt = entityController_->getEntityManager().getEntity(guid);
        if (goEnt && goEnt->getType() == ObjectType::GAMEOBJECT) {
            auto go = std::static_pointer_cast<GameObject>(goEnt);
            // Only treat custom animation as a bite when this is our bobber.
            // AzerothCore sends the GO's animation progress (normally 100), not
            // M2 animation id 0, so the packet payload itself cannot identify it.
            uint64_t createdBy = static_cast<uint64_t>(go->getField(6))
                               | (static_cast<uint64_t>(go->getField(7)) << 32);
            auto* info = getCachedGameObjectInfo(go->getEntry());
            ownedFishingBite = createdBy == playerGuid && (!info || info->type == 17);
            if (ownedFishingBite) {
                hookedFishingBobberGuid_ = guid;
                // G_FishingBobber.m2 sequence 153 is its authored 1.33s bite/splash
                // animation; sequence 0 is the normal three-second idle bob.
                constexpr uint32_t kFishingBobberBiteAnimation = 153;
                if (gameObjectCustomAnimCallback_)
                    gameObjectCustomAnimCallback_(guid, kFishingBobberBiteAnimation);
                addUIError("A fish is on your line! Right-click to reel it in.");
                addSystemChatMessage("A fish is on your line! Right-click to reel it in.");
                withSoundManager(&audio::AudioCoordinator::getUiSoundManager,
                                 [](auto* sfx) { sfx->playFishingBite(); });
                LOG_WARNING("Fishing bite ready: guid=0x", std::hex, guid, std::dec,
                            " serverAnimProgress=", animId);
            }
        }
        if (!ownedFishingBite && gameObjectCustomAnimCallback_)
            gameObjectCustomAnimCallback_(guid, animId);
    };

    // Item refund / socket gems / item time
    dispatchTable_[Opcode::SMSG_ITEM_REFUND_RESULT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            packet.readUInt64(); // itemGuid
            uint32_t result = packet.readUInt32();
            addSystemChatMessage(result == 0 ? "Item returned. Refund processed."
                                             : "Could not return item for refund.");
        }
    };
    dispatchTable_[Opcode::SMSG_SOCKET_GEMS_RESULT] = [this](network::Packet& packet) {
        // There is no result field. Item::SendUpdateSockets writes the item's
        // guid and then the four enchantment ids now in its sockets, and sends
        // it only when the sockets were actually changed - so arriving *is* the
        // success. This read a uint32 at offset zero and called it the result,
        // which is the low half of the item's guid: never zero, so socketing a
        // gem always reported that it had failed.
        if (!packet.hasRemaining(8)) return;
        const uint64_t itemGuid = packet.readUInt64();
        addSystemChatMessage("Gems socketed successfully.");
        LOG_DEBUG("SMSG_SOCKET_GEMS_RESULT: itemGuid=0x", std::hex, itemGuid, std::dec);
    };
    dispatchTable_[Opcode::SMSG_ITEM_TIME_UPDATE] = [](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            packet.readUInt64(); // itemGuid
            packet.readUInt32(); // durationMs
        }
    };

    // ---- Batch 6: Spell miss / env damage / control / spell failure ----


    // ---- Achievement / fishing delegates ----
    dispatchTable_[Opcode::SMSG_ALL_ACHIEVEMENT_DATA] = [this](network::Packet& packet) {
        handleAllAchievementData(packet);
    };
    // Both are errors rather than remarks, and the real client shows them on
    // the error line rather than in the chat log - ERR_FISH_NOT_HOOKED and
    // ERR_FISH_ESCAPED are globalstrings. raiseUiError puts them there, fires
    // UI_ERROR_MESSAGE so the interface's own error frame draws them, and
    // keeps the chat line this client has always written.
    dispatchTable_[Opcode::SMSG_FISH_NOT_HOOKED] = [this](network::Packet& /*packet*/) {
        hookedFishingBobberGuid_ = 0;
        raiseUiError("Your fish got away.");
    };
    dispatchTable_[Opcode::SMSG_FISH_ESCAPED] = [this](network::Packet& /*packet*/) {
        hookedFishingBobberGuid_ = 0;
        raiseUiError("Your fish escaped!");
    };

    // ---- Auto-repeat / auras / dispel / totem ----
    dispatchTable_[Opcode::SMSG_CANCEL_AUTO_REPEAT] = [this](network::Packet& /*packet*/) {
        // The server saying a repeating spell has stopped. Nothing in this
        // client's own interface reads it, which is why it did nothing - but
        // the action button flashes for as long as one is running and stops on
        // exactly this.
        fireAddonEvent("STOP_AUTOREPEAT_SPELL", {});
    };


    // ---- Batch 7: World states, action buttons, level-up, vendor, inventory ----

    // ---- SMSG_INIT_WORLD_STATES ----
    dispatchTable_[Opcode::SMSG_INIT_WORLD_STATES] = [this](network::Packet& packet) {
        // WotLK/TBC format: uint32 mapId, uint32 zoneId, uint32 areaId, uint16 count, N*(uint32 key, uint32 val)
        // Classic format: uint32 mapId, uint32 zoneId, uint16 count, N*(uint32 key, uint32 val)
        if (!packet.hasRemaining(10)) {
            LOG_WARNING("SMSG_INIT_WORLD_STATES too short: ", packet.getSize(), " bytes");
            return;
        }
        worldStateMapId_ = packet.readUInt32();
        {
            uint32_t newZoneId = packet.readUInt32();
            if (newZoneId != worldStateZoneId_ && newZoneId != 0) {
                worldStateZoneId_ = newZoneId;
                    fireAddonEvent("ZONE_CHANGED_NEW_AREA", {});
                    fireAddonEvent("ZONE_CHANGED", {});
            } else {
                worldStateZoneId_ = newZoneId;
            }
        }
        // TBC added areaId in 2.1.0; WotLK kept it. Classic/Turtle use the shorter format.
        size_t remaining = packet.getRemainingSize();
        bool hasAreaId = isActiveExpansion("tbc") || isActiveExpansion("wotlk");
        if (hasAreaId && remaining >= 6) {
            packet.readUInt32(); // areaId
        }
        uint16_t count = packet.readUInt16();
        size_t needed = static_cast<size_t>(count) * 8;
        size_t available = packet.getRemainingSize();
        if (available < needed) {
            // Be tolerant across expansion/private-core variants: if packet shape
            // still looks like N*(key,val) dwords, parse what is present.
            if ((available % 8) == 0) {
                uint16_t adjustedCount = static_cast<uint16_t>(available / 8);
                LOG_WARNING("SMSG_INIT_WORLD_STATES count mismatch: header=", count,
                            " adjusted=", adjustedCount, " (available=", available, ")");
                count = adjustedCount;
                needed = available;
            } else {
                LOG_WARNING("SMSG_INIT_WORLD_STATES truncated: expected ", needed,
                            " bytes of state pairs, got ", available);
                packet.skipAll();
                return;
            }
        }
        worldStates_.clear();
        worldStates_.reserve(count);
        for (uint16_t i = 0; i < count; ++i) {
            uint32_t key = packet.readUInt32();
            uint32_t val = packet.readUInt32();
            worldStates_[key] = val;
        }
    };

    // ---- SMSG_ACTION_BUTTONS ----
    dispatchTable_[Opcode::SMSG_ACTION_BUTTONS] = [this](network::Packet& packet) {
        // Slot encoding differs by expansion:
        //   Classic/Turtle: uint16 actionId + uint8 type + uint8 misc
        //     type: 0=spell, 1=item, 64=macro
        //   TBC/WotLK: uint32 packed = actionId | (type << 24)
        //     type: 0x00=spell, 0x80=item, 0x40=macro
        // Format differences:
        //   Classic 1.12: no mode byte, 120 slots (480 bytes)
        //   TBC 2.4.3:    no mode byte, 132 slots (528 bytes)
        //   WotLK 3.3.5a: uint8 mode + 144 slots (577 bytes)
        size_t rem = packet.getRemainingSize();
        const bool hasModeByteExp = isActiveExpansion("wotlk");
        int serverBarSlots;
        if (isClassicLikeExpansion()) {
            serverBarSlots = 120;
        } else if (isActiveExpansion("tbc")) {
            serverBarSlots = 132;
        } else {
            serverBarSlots = 144;
        }
        if (hasModeByteExp) {
            if (rem < 1) return;
            /*uint8_t mode =*/ packet.readUInt8();
            rem--;
        }
        for (int i = 0; i < serverBarSlots; ++i) {
            if (rem < 4) return;
            uint32_t packed = packet.readUInt32();
            rem -= 4;
            if (i >= ACTION_BAR_SLOTS) continue;
            if (packed == 0) {
                // Empty slot - only clear if not already set to Attack/Hearthstone defaults
                // so we don't wipe hardcoded fallbacks when the server sends zeros.
                continue;
            }
            uint8_t type = 0;
            uint32_t id = 0;
            if (isClassicLikeExpansion()) {
                id = packed & 0x0000FFFFu;
                type = static_cast<uint8_t>((packed >> 16) & 0xFF);
            } else {
                type = static_cast<uint8_t>((packed >> 24) & 0xFF);
                id = packed & 0x00FFFFFFu;
            }
            if (id == 0) continue;
            ActionBarSlot slot;
            switch (type) {
                case 0x00: slot.type = ActionBarSlot::SPELL; slot.id = id; break;
                case 0x01: slot.type = ActionBarSlot::ITEM;  slot.id = id; break;  // Classic item
                case 0x80: slot.type = ActionBarSlot::ITEM;  slot.id = id; break;  // TBC/WotLK item
                case 0x40: slot.type = ActionBarSlot::MACRO; slot.id = id; break;  // macro (all expansions)
                default:   continue;  // unknown - leave as-is
            }
            actionBar[i] = slot;
        }
        // Apply any pending cooldowns from spellHandler's cooldowns to newly populated slots.
        // SMSG_SPELL_COOLDOWN often arrives before SMSG_ACTION_BUTTONS during login,
        // so the per-slot cooldownRemaining would be 0 without this sync.
        if (spellHandler_) {
            const auto& cooldowns = spellHandler_->getSpellCooldowns();
            for (auto& slot : actionBar) {
                if (slot.type == ActionBarSlot::SPELL && slot.id != 0) {
                    auto cdIt = cooldowns.find(slot.id);
                    if (cdIt != cooldowns.end() && cdIt->second > 0.0f) {
                        slot.cooldownRemaining = cdIt->second;
                        slot.cooldownTotal     = cdIt->second;
                    }
                } else if (slot.type == ActionBarSlot::ITEM && slot.id != 0) {
                    // Items (potions, trinkets): look up the item's on-use spell
                    // and check if that spell has a pending cooldown.
                    const auto* qi = getItemInfo(slot.id);
                    if (qi && qi->valid) {
                        for (const auto& sp : qi->spells) {
                            if (sp.spellId == 0) continue;
                            auto cdIt = cooldowns.find(sp.spellId);
                            if (cdIt != cooldowns.end() && cdIt->second > 0.0f) {
                                slot.cooldownRemaining = cdIt->second;
                                slot.cooldownTotal     = cdIt->second;
                                break;
                            }
                        }
                    }
                }
            }
        }
        LOG_INFO("SMSG_ACTION_BUTTONS: populated action bar from server");
        // Zero means every slot, which is what the whole bar arriving at
        // once is. Sent with no argument the buttons compared nil against
        // zero and against their own action, matched neither, and the bar
        // the server had just sent drew nothing.
        fireAddonEvent("ACTIONBAR_SLOT_CHANGED", {"0"});
        packet.skipAll();
    };

    // ---- SMSG_LEVELUP_INFO / SMSG_LEVELUP_INFO_ALT (shared body) ----
    for (auto op : {Opcode::SMSG_LEVELUP_INFO, Opcode::SMSG_LEVELUP_INFO_ALT}) {
        dispatchTable_[op] = [this](network::Packet& packet) {
            // Server-authoritative level-up event.
            // WotLK layout: uint32 newLevel + uint32 hpDelta + uint32 manaDelta + 5x uint32 statDeltas
            if (packet.hasRemaining(4)) {
                uint32_t newLevel = packet.readUInt32();
                if (newLevel > 0) {
                    // Parse stat deltas (WotLK layout has 7 more uint32s)
                    lastLevelUpDeltas_ = {};
                    if (packet.hasRemaining(28)) {
                        lastLevelUpDeltas_.hp    = packet.readUInt32();
                        lastLevelUpDeltas_.mana  = packet.readUInt32();
                        lastLevelUpDeltas_.str   = packet.readUInt32();
                        lastLevelUpDeltas_.agi   = packet.readUInt32();
                        lastLevelUpDeltas_.sta   = packet.readUInt32();
                        lastLevelUpDeltas_.intel = packet.readUInt32();
                        lastLevelUpDeltas_.spi   = packet.readUInt32();
                    }
                    uint32_t oldLevel = serverPlayerLevel_;
                    serverPlayerLevel_ = std::max(serverPlayerLevel_, newLevel);
                    // Update the character-list entry so the selection screen
                    // shows the correct level if the player logs out and back.
                    for (auto& ch : characters) {
                        if (ch.guid == playerGuid) {
                            ch.level = serverPlayerLevel_;
                            break;  // was 'return' - must NOT exit here or level-up notification is skipped
                        }
                    }
                    if (newLevel > oldLevel) {
                        // Not when the interface writes it: chatframe.lua's
                        // PLAYER_LEVEL_UP branch formats LEVEL_UP from arg1
                        // and adds the health and mana gains under it, so this
                        // line was a second, shorter copy above its own.
                        if (!ui::frameXmlOwns(ui::UiElement::Chat)) {
                            addSystemChatMessage("You have reached level " +
                                                 std::to_string(newLevel) + "!");
                        }
                        withSoundManager(&audio::AudioCoordinator::getUiSoundManager, [](auto* sfx) { sfx->playLevelUp(); });
                        if (levelUpCallback_) levelUpCallback_(newLevel);
                        // All nine, in the order the interface reads them:
                        // level, health, power, talent points, then the five
                        // stats. The chat frame tests the third against zero
                        // to decide whether to mention mana, so sending only
                        // the level was an error on every level gained - and
                        // every one of these was already parsed above.
                        //
                        // Talent points are zero: the packet carries no such
                        // delta, and zero is what stops the interface claiming
                        // a point was gained.
                        fireAddonEvent("PLAYER_LEVEL_UP", {
                            std::to_string(newLevel),
                            std::to_string(lastLevelUpDeltas_.hp),
                            std::to_string(lastLevelUpDeltas_.mana),
                            "0",
                            std::to_string(lastLevelUpDeltas_.str),
                            std::to_string(lastLevelUpDeltas_.agi),
                            std::to_string(lastLevelUpDeltas_.sta),
                            std::to_string(lastLevelUpDeltas_.intel),
                            std::to_string(lastLevelUpDeltas_.spi),
                        });
                    }
                }
            }
            packet.skipAll();
        };
    }

    // ---- MSG_RAID_TARGET_UPDATE ----
    dispatchTable_[Opcode::MSG_RAID_TARGET_UPDATE] = [this](network::Packet& packet) {
        RaidTargetUpdateData rtu;
        if (!RaidTargetUpdateParser::parse(packet, rtu)) return;

        if (socialHandler_) {
            // The full list is authoritative, so icons absent from it are gone.
            if (rtu.fullList) {
                for (uint32_t i = 0; i < kRaidMarkCount; ++i)
                    socialHandler_->setRaidTargetGuid(static_cast<uint8_t>(i), 0);
            }
            for (const auto& [icon, guid] : rtu.marks)
                socialHandler_->setRaidTargetGuid(icon, guid);
        }
        LOG_DEBUG("MSG_RAID_TARGET_UPDATE: fullList=", rtu.fullList,
                  " marks=", rtu.marks.size());
        fireAddonEvent("RAID_TARGET_UPDATE", {});
    };

    // ---- SMSG_CRITERIA_UPDATE ----
    dispatchTable_[Opcode::SMSG_CRITERIA_UPDATE] = [this](network::Packet& packet) {
        // The record's shape is in readCriteriaProgressTail, shared with the
        // criteria half of SMSG_ALL_ACHIEVEMENT_DATA.
        if (packet.hasRemaining(4)) {
            uint32_t criteriaId = packet.readUInt32();
            CriteriaProgressRecord rec;
            if (readCriteriaProgressTail(packet, rec)) {
                // A timed criteria whose window ran out reads as zero, which is
                // what the server means by the flag and what the panel shows.
                const uint64_t progress = (rec.flags == 1) ? 0 : rec.counter;
                uint64_t oldProgress = 0;
                auto cpit = criteriaProgress_.find(criteriaId);
                if (cpit != criteriaProgress_.end()) oldProgress = cpit->second;
                criteriaProgress_[criteriaId] = progress;
                LOG_DEBUG("SMSG_CRITERIA_UPDATE: id=", criteriaId, " progress=", progress);
                // Fire addon event for achievement tracking addons
                if (progress != oldProgress)
                    fireAddonEvent("CRITERIA_UPDATE", {std::to_string(criteriaId), std::to_string(progress)});

                // A timed criteria also drives the tracker's countdown. The
                // message says how far into the window the player is; how long
                // the window is comes from the DBC, so the two have to be put
                // together here - the interface reads the pair and works out a
                // start time from them.
                //
                // Fired for any timed criteria rather than only tracked ones.
                // The tracker stores what it is told and draws a timer only
                // inside its tracked-achievement loop, so entries for the rest
                // cost a table row and show nothing; the gain is that tracking
                // an achievement part-way through its timer still shows the
                // right countdown, instead of nothing until the next update.
                ensureAchievementCriteriaLoaded();
                const auto idx = getAchievementCriterionIndex(criteriaId);
                if (idx.timeLimit != 0 && idx.achievementId != 0) {
                    fireAddonEvent("TRACKED_ACHIEVEMENT_UPDATE",
                                   {std::to_string(idx.achievementId),
                                    std::to_string(criteriaId),
                                    std::to_string(rec.timeElapsed),
                                    std::to_string(idx.timeLimit)});
                }
            }
        }
    };

    // ---- SMSG_BARBER_SHOP_RESULT ----
    dispatchTable_[Opcode::SMSG_BARBER_SHOP_RESULT] = [this](network::Packet& packet) {
        // uint32 result: 0=success, 1/3=not enough money, 2=not seated at barber
        if (packet.hasRemaining(4)) {
            uint32_t result = packet.readUInt32();
            if (result == 0) {
                addSystemChatMessage("Hairstyle changed.");
                barberShopOpen_ = false;
                fireAddonEvent("BARBER_SHOP_CLOSE", {});
            } else {
                const char* msg = (result == 1 || result == 3) ? "Not enough money for new hairstyle."
                                : (result == 2) ? "You are not at a barber shop."
                                : "Barber shop unavailable.";
                addUIError(msg);
                addSystemChatMessage(msg);
            }
            LOG_DEBUG("SMSG_BARBER_SHOP_RESULT: result=", result);
        }
    };

}

// Everything else - inspects, quests, auctions, spells, calendars,
// battlefields, voice, and a long tail that is consumed and ignored.
//
// Named for what it is rather than given a theme it does not have: it arrived
// as five batches of whatever was left over, and the honest description of that
// is 'the rest'.
void GameHandler::registerRemainingOpcodes() {
    // -----------------------------------------------------------------------
    // Batch 8-12: Remaining opcodes (inspects, quests, auctions, spells,
    //             calendars, battlefields, voice, misc consume-only)
    // -----------------------------------------------------------------------
    // uint32 currentZoneLightId + uint32 overrideLightId + uint32 transitionMs
    dispatchTable_[Opcode::SMSG_OVERRIDE_LIGHT] = [this](network::Packet& packet) {
        // uint32 currentZoneLightId + uint32 overrideLightId + uint32 transitionMs
        if (packet.hasRemaining(12)) {
            uint32_t zoneLightId     = packet.readUInt32();
            uint32_t overrideLightId = packet.readUInt32();
            uint32_t transitionMs    = packet.readUInt32();
            overrideLightId_      = overrideLightId;
            overrideLightTransMs_ = transitionMs;
            LOG_DEBUG("SMSG_OVERRIDE_LIGHT: zone=", zoneLightId,
                      " override=", overrideLightId, " transition=", transitionMs, "ms");
        }
    };
    // Classic 1.12: uint32 weatherType + float intensity (8 bytes, no isAbrupt)
    // TBC 2.4.3 / WotLK 3.3.5a: uint32 weatherType + float intensity + uint8 isAbrupt (9 bytes)
    dispatchTable_[Opcode::SMSG_WEATHER] = [this](network::Packet& packet) {
        // Classic 1.12: uint32 weatherType + float intensity (8 bytes, no isAbrupt)
        // TBC 2.4.3 / WotLK 3.3.5a: uint32 weatherType + float intensity + uint8 isAbrupt (9 bytes)
        if (packet.hasRemaining(8)) {
            uint32_t wType = packet.readUInt32();
            float wIntensity = packet.readFloat();
            if (packet.hasRemaining(1))
                /*uint8_t isAbrupt =*/ packet.readUInt8();
            uint32_t prevWeatherType = weatherType_;
            weatherType_ = wType;
            weatherIntensity_ = wIntensity;
            const char* typeName = (wType == 1) ? "Rain" : (wType == 2) ? "Snow" : (wType == 3) ? "Storm" : "Clear";
            LOG_INFO("Weather changed: type=", wType, " (", typeName, "), intensity=", wIntensity);
            // Announce weather changes (including initial zone weather)
            if (wType != prevWeatherType) {
                const char* weatherMsg = nullptr;
                if (wIntensity < 0.05f || wType == 0) {
                    if (prevWeatherType != 0)
                        weatherMsg = "The weather clears.";
                } else if (wType == 1) {
                    weatherMsg = "It begins to rain.";
                } else if (wType == 2) {
                    weatherMsg = "It begins to snow.";
                } else if (wType == 3) {
                    weatherMsg = "A storm rolls in.";
                }
                if (weatherMsg) addSystemChatMessage(weatherMsg);
            }
            // Notify addons of weather change
                            fireAddonEvent("WEATHER_CHANGED", {std::to_string(wType), std::to_string(wIntensity)});
            // Storm transition: trigger a low-frequency thunder rumble shake
            if (wType == 3 && wIntensity > 0.3f && cameraShakeCallback_) {
                float mag = 0.03f + wIntensity * 0.04f; // 0.03–0.07 units
                cameraShakeCallback_(mag, 6.0f, 0.6f);
            }
        }
    };
    // Server-script text message - display in system chat
    dispatchTable_[Opcode::SMSG_SCRIPT_MESSAGE] = [this](network::Packet& packet) {
        // Server-script text message - display in system chat
        std::string msg = packet.readString();
        if (!msg.empty()) {
            addSystemChatMessage(msg);
            LOG_INFO("SMSG_SCRIPT_MESSAGE: ", msg);
        }
    };
    // uint64 targetGuid + uint64 casterGuid + uint32 spellId + uint32 displayId + uint32 animType
    dispatchTable_[Opcode::SMSG_ENCHANTMENTLOG] = [this](network::Packet& packet) {
        // uint64 targetGuid + uint64 casterGuid + uint32 spellId + uint32 displayId + uint32 animType
        if (packet.hasRemaining(28)) {
            uint64_t enchTargetGuid = packet.readUInt64();
            uint64_t enchCasterGuid = packet.readUInt64();
            uint32_t enchSpellId = packet.readUInt32();
            /*uint32_t displayId =*/ packet.readUInt32();
            /*uint32_t animType =*/ packet.readUInt32();
            LOG_DEBUG("SMSG_ENCHANTMENTLOG: spellId=", enchSpellId);
            // Show enchant message if the player is involved
            if (enchTargetGuid == playerGuid || enchCasterGuid == playerGuid) {
                const std::string& enchName = getSpellName(enchSpellId);
                std::string casterName = lookupName(enchCasterGuid);
                if (!enchName.empty()) {
                    std::string msg;
                    if (enchCasterGuid == playerGuid)
                        msg = "You enchant with " + enchName + ".";
                    else if (!casterName.empty())
                        msg = casterName + " enchants your item with " + enchName + ".";
                    else
                        msg = "Your item has been enchanted with " + enchName + ".";
                    addSystemChatMessage(msg);
                }
            }
        }
    };
    // WotLK: uint64 playerGuid + uint8 teamCount + per-team fields
    dispatchTable_[Opcode::MSG_INSPECT_ARENA_TEAMS] = [this](network::Packet& packet) {
        // WotLK: uint64 playerGuid + uint8 teamCount + per-team fields
        if (!packet.hasRemaining(9)) {
            packet.skipAll();
            return;
        }
        uint64_t inspGuid  = packet.readUInt64();
        uint8_t  teamCount = packet.readUInt8();
        if (teamCount > 3) teamCount = 3; // 2v2, 3v3, 5v5
        if (socialHandler_) {
            auto& ir = socialHandler_->mutableInspectResult();
            if (inspGuid == ir.guid || ir.guid == 0) {
                ir.guid = inspGuid;
                ir.arenaTeams.clear();
                for (uint8_t t = 0; t < teamCount; ++t) {
                    if (!packet.hasRemaining(21)) break;
                    SocialHandler::InspectArenaTeam team;
                    team.teamId         = packet.readUInt32();
                    team.type           = packet.readUInt8();
                    team.weekGames      = packet.readUInt32();
                    team.weekWins       = packet.readUInt32();
                    team.seasonGames    = packet.readUInt32();
                    team.seasonWins     = packet.readUInt32();
                    team.name           = packet.readString();
                    if (!packet.hasRemaining(4)) break;
                    team.personalRating = packet.readUInt32();
                    ir.arenaTeams.push_back(std::move(team));
                }
            }
        }
        LOG_DEBUG("MSG_INSPECT_ARENA_TEAMS: guid=0x", std::hex, inspGuid, std::dec,
                  " teams=", static_cast<int>(teamCount));
    };
    // auctionId(u32) + action(u32) + error(u32) + itemEntry(u32) + randomPropertyId(u32) + ...
    // action: 0=sold/won, 1=expired, 2=bid placed on your auction
    // auctionHouseId(u32) + auctionId(u32) + bidderGuid(u64) + bidAmount(u32) + outbidAmount(u32) + itemEntry(u32) + randomPropertyId(u32)
    // uint32 auctionId + uint32 itemEntry + uint32 itemRandom - auction expired/cancelled
    // uint64 containerGuid - tells client to open this container
    // The actual items come via update packets; we just log this.
    // PackedGuid (player guid) + uint32 vehicleId
    // vehicleId == 0 means the player left the vehicle
    dispatchTable_[Opcode::SMSG_PLAYER_VEHICLE_DATA] = [this](network::Packet& packet) {
        // PackedGuid (player guid) + uint32 vehicleId
        // vehicleId == 0 means the player left the vehicle
        if (packet.hasRemaining(1)) {
            (void)packet.readPackedGuid(); // player guid (unused)
        }
        uint32_t newVehicleId = 0;
        if (packet.hasRemaining(4)) {
            newVehicleId = packet.readUInt32();
        }
        bool wasInVehicle = vehicleId_ != 0;
        bool nowInVehicle = newVehicleId != 0;
        vehicleId_ = newVehicleId;
        if (wasInVehicle != nowInVehicle) {
            if (vehicleStateCallback_) vehicleStateCallback_(nowInVehicle, newVehicleId);
            // Announced as well as stored. The state was kept and handed to
            // this client's own bar, and the interface heard nothing - so the
            // leave-vehicle button, which redraws on VEHICLE_UPDATE, never
            // learned there was a vehicle to leave.
            fireAddonEvent("VEHICLE_UPDATE", {});
            fireAddonEvent(nowInVehicle ? "UNIT_ENTERED_VEHICLE"
                                        : "UNIT_EXITED_VEHICLE", {"player"});
            fireAddonEvent("UPDATE_BONUS_ACTIONBAR", {});
        }
    };
    // guid(8) + status(1): status 1 = NPC has available/new routes for this player
    dispatchTable_[Opcode::SMSG_TAXINODE_STATUS] = [this](network::Packet& packet) {
        // guid(8) + status(1): status 1 = NPC has available/new routes for this player
        if (packet.hasRemaining(9)) {
            uint64_t npcGuid = packet.readUInt64();
            uint8_t  status  = packet.readUInt8();
            taxiNpcHasRoutes_[npcGuid] = (status != 0);
        }
    };
    // SMSG_GUILD_DECLINE - moved to SocialHandler::registerOpcodes
    // Clear cached talent data so the talent screen reflects the reset.
    dispatchTable_[Opcode::SMSG_TALENTS_INVOLUNTARILY_RESET] = [this](network::Packet& packet) {
        // Clear cached talent data so the talent screen reflects the reset.
        if (spellHandler_) spellHandler_->resetTalentState();
        addUIError("Your talents have been reset by the server.");
        addSystemChatMessage("Your talents have been reset by the server.");
        // The talent frame rebuilds from this; the cached state was cleared
        // above and nothing told it to redraw, so it kept the old trees.
        if (addonEventCallback_) addonEventCallback_("TALENTS_INVOLUNTARILY_RESET", {});
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_SET_REST_START] = [](network::Packet& packet) {
        // The rest-XP accumulation start time, not a statement about where the
        // player is standing: the server sends it on login wherever that is.
        // Resting itself comes from PLAYER_FLAGS_RESTING, which entity_controller
        // watches, because that is the one that clears again on leaving an inn.
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_UPDATE_AURA_DURATION] = [this](network::Packet& packet) {
        if (packet.hasRemaining(5)) {
            uint8_t slot       = packet.readUInt8();
            uint32_t durationMs = packet.readUInt32();
            handleUpdateAuraDuration(slot, durationMs);
        }
    };
    dispatchTable_[Opcode::SMSG_ITEM_NAME_QUERY_RESPONSE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t itemId = packet.readUInt32();
            std::string name = packet.readString();
            if (!itemInfoCache_.count(itemId) && !name.empty()) {
                ItemQueryResponseData stub;
                stub.entry = itemId;
                stub.name  = std::move(name);
                stub.valid = true;
                itemInfoCache_[itemId] = std::move(stub);
            }
        }
        packet.skipAll();
    };
    // A plain guid: Unit::SendMountSpecialAnim writes GetGUID(), not the
    // packed form, so reading it packed consumed the wrong number of bytes.
    dispatchTable_[Opcode::SMSG_MOUNTSPECIAL_ANIM] = [](network::Packet& packet) {
        if (packet.hasRemaining(8)) (void)packet.readUInt64();
    };
    dispatchTable_[Opcode::SMSG_CHAR_CUSTOMIZE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t result = packet.readUInt8();
            addSystemChatMessage(result == 0 ? "Character customization complete."
                                             : "Character customization failed.");
        }
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_CHAR_FACTION_CHANGE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t result = packet.readUInt8();
            addSystemChatMessage(result == 0 ? "Faction change complete."
                                             : "Faction change failed.");
        }
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_INVALIDATE_PLAYER] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint64_t guid = packet.readUInt64();
            entityController_->invalidatePlayerName(guid);
        }
    };
    // uint32 movieId - we don't play movies; acknowledge immediately.
    dispatchTable_[Opcode::SMSG_TRIGGER_MOVIE] = [this](network::Packet& packet) {
        // uint32 movieId - we don't play movies; acknowledge immediately.
        packet.skipAll();
        // WotLK servers expect CMSG_COMPLETE_MOVIE after the movie finishes;
        // without it, the server may hang or disconnect the client.
        uint16_t wire = wireOpcode(Opcode::CMSG_COMPLETE_MOVIE);
        if (wire != 0xFFFF) {
            network::Packet ack(wire);
            socket->send(ack);
            LOG_DEBUG("SMSG_TRIGGER_MOVIE: skipped, sent CMSG_COMPLETE_MOVIE");
        }
    };
    // Server-side LFG invite timed out (no response within time limit)
    dispatchTable_[Opcode::SMSG_LFG_TIMEDOUT] = [this](network::Packet& packet) {
        // Server-side LFG invite timed out (no response within time limit)
        addSystemChatMessage("Dungeon Finder: Invite timed out.");
        if (openLfgCallback_) openLfgCallback_();
        packet.skipAll();
    };
    // Another party member failed to respond to a LFG role-check in time
    dispatchTable_[Opcode::SMSG_LFG_OTHER_TIMEDOUT] = [this](network::Packet& packet) {
        // Another party member failed to respond to a LFG role-check in time
        addSystemChatMessage("Dungeon Finder: Another player's invite timed out.");
        if (openLfgCallback_) openLfgCallback_();
        packet.skipAll();
    };
    // uint32 result - LFG auto-join attempt failed (player selected auto-join at queue time)
    dispatchTable_[Opcode::SMSG_LFG_AUTOJOIN_FAILED] = [this](network::Packet& packet) {
        // uint32 result - LFG auto-join attempt failed (player selected auto-join at queue time)
        if (packet.hasRemaining(4)) {
            uint32_t result = packet.readUInt32();
            (void)result;
        }
        addUIError("Dungeon Finder: Auto-join failed.");
        addSystemChatMessage("Dungeon Finder: Auto-join failed.");
        packet.skipAll();
    };
    // No eligible players found for auto-join
    dispatchTable_[Opcode::SMSG_LFG_AUTOJOIN_FAILED_NO_PLAYER] = [this](network::Packet& packet) {
        // No eligible players found for auto-join
        addUIError("Dungeon Finder: No players available for auto-join.");
        addSystemChatMessage("Dungeon Finder: No players available for auto-join.");
        packet.skipAll();
    };
    // Party leader is currently set to Looking for More (LFM) mode
    dispatchTable_[Opcode::SMSG_LFG_LEADER_IS_LFM] = [this](network::Packet& packet) {
        // Party leader is currently set to Looking for More (LFM) mode
        addSystemChatMessage("Your party leader is currently Looking for More.");
        packet.skipAll();
    };
    // uint32 zoneId + uint8 level_min + uint8 level_max - player queued for meeting stone
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_SETQUEUE] = [this](network::Packet& packet) {
        // uint32 zoneId + uint8 level_min + uint8 level_max - player queued for meeting stone
        if (packet.hasRemaining(6)) {
            uint32_t zoneId   = packet.readUInt32();
            uint8_t  levelMin = packet.readUInt8();
            uint8_t  levelMax = packet.readUInt8();
            char buf[128];
            std::string zoneName = getAreaName(zoneId);
            if (!zoneName.empty())
                std::snprintf(buf, sizeof(buf),
                    "You are now in the Meeting Stone queue for %s (levels %u-%u).",
                    zoneName.c_str(), levelMin, levelMax);
            else
                std::snprintf(buf, sizeof(buf),
                    "You are now in the Meeting Stone queue for zone %u (levels %u-%u).",
                    zoneId, levelMin, levelMax);
            addSystemChatMessage(buf);
            LOG_INFO("SMSG_MEETINGSTONE_SETQUEUE: zone=", zoneId,
                     " levels=", static_cast<int>(levelMin), "-", static_cast<int>(levelMax));
        }
        packet.skipAll();
    };
    // Server confirms group found and teleport summon is ready
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_COMPLETE] = [this](network::Packet& packet) {
        // Server confirms group found and teleport summon is ready
        addSystemChatMessage("Meeting Stone: Your group is ready! Use the Meeting Stone to summon.");
        LOG_INFO("SMSG_MEETINGSTONE_COMPLETE");
        packet.skipAll();
    };
    // Meeting stone search is still ongoing
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_IN_PROGRESS] = [this](network::Packet& packet) {
        // Meeting stone search is still ongoing
        addSystemChatMessage("Meeting Stone: Searching for group members...");
        LOG_DEBUG("SMSG_MEETINGSTONE_IN_PROGRESS");
        packet.skipAll();
    };
    // uint64 memberGuid - a player was added to your group via meeting stone
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_MEMBER_ADDED] = [this](network::Packet& packet) {
        // uint64 memberGuid - a player was added to your group via meeting stone
        if (packet.hasRemaining(8)) {
            uint64_t memberGuid = packet.readUInt64();
            const auto& memberName = lookupName(memberGuid);
            if (!memberName.empty()) {
                addSystemChatMessage("Meeting Stone: " + memberName +
                                     " has been added to your group.");
            } else {
                addSystemChatMessage("Meeting Stone: A new player has been added to your group.");
            }
            LOG_INFO("SMSG_MEETINGSTONE_MEMBER_ADDED: guid=0x", std::hex, memberGuid, std::dec);
        }
    };
    // uint8 reason - failed to join group via meeting stone
    // 0=target_not_in_lfg, 1=target_in_party, 2=target_invalid_map, 3=target_not_available
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_JOINFAILED] = [this](network::Packet& packet) {
        // uint8 reason - failed to join group via meeting stone
        // 0=target_not_in_lfg, 1=target_in_party, 2=target_invalid_map, 3=target_not_available
        static const char* kMeetingstoneErrors[] = {
            "Target player is not using the Meeting Stone.",
            "Target player is already in a group.",
            "You are not in a valid zone for that Meeting Stone.",
            "Target player is not available.",
        };
        if (packet.hasRemaining(1)) {
            uint8_t reason = packet.readUInt8();
            const char* msg = (reason < 4) ? kMeetingstoneErrors[reason]
                                           : "Meeting Stone: Could not join group.";
            addSystemChatMessage(msg);
            LOG_INFO("SMSG_MEETINGSTONE_JOINFAILED: reason=", static_cast<int>(reason));
        }
    };
    // Player was removed from the meeting stone queue (left, or group disbanded)
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_LEAVE] = [this](network::Packet& packet) {
        // Player was removed from the meeting stone queue (left, or group disbanded)
        addSystemChatMessage("You have left the Meeting Stone queue.");
        LOG_DEBUG("SMSG_MEETINGSTONE_LEAVE");
        packet.skipAll();
    };
    // All three of these carry a uint32 from the same enum, and all three read
    // a single byte of it. Little-endian kept the comparison working for the
    // delete, whose value is nine, and broke the other two outright: creating a
    // ticket answers 2 and updating one answers 4, and both were compared
    // against 1 - so a ticket that went through reported that it had not.
    constexpr uint32_t kTicketAlreadyExists = 1, kTicketCreated = 2,
                       kTicketUpdated = 4, kTicketDeleted = 9;
    dispatchTable_[Opcode::SMSG_GMTICKET_CREATE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            const uint32_t res = packet.readUInt32();
            addSystemChatMessage(
                res == kTicketCreated       ? "GM ticket submitted."
              : res == kTicketAlreadyExists ? "You already have a GM ticket open."
                                            : "Failed to submit GM ticket.");
        }
    };
    dispatchTable_[Opcode::SMSG_GMTICKET_UPDATETEXT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            const uint32_t res = packet.readUInt32();
            addSystemChatMessage(res == kTicketUpdated ? "GM ticket updated."
                                                       : "Failed to update GM ticket.");
        }
    };
    dispatchTable_[Opcode::SMSG_GMTICKET_DELETETICKET] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            const uint32_t res = packet.readUInt32();
            addSystemChatMessage(res == kTicketDeleted ? "GM ticket deleted."
                                                       : "No ticket to delete.");
        }
    };
    // WotLK 3.3.5a format:
    //   uint8  status  - 6=has open ticket, 3=closed, anything else none
    // If status == 6 (GMTICKET_STATUS_HASTEXT):
    //   cstring ticketText
    //   uint32  ticketAge       (seconds old)
    //   uint32  daysUntilOld    (days remaining before escalation)
    //   float   waitTimeHours   (estimated GM wait time)
    dispatchTable_[Opcode::SMSG_GMTICKET_GETTICKET] = [this](network::Packet& packet) {
        // WotLK 3.3.5a format:
        //   uint8  status  - 6=has open ticket, 3=closed, anything else none
        // If status == 6 (GMTICKET_STATUS_HASTEXT):
        //   cstring ticketText
        //   uint32  ticketAge       (seconds old)
        //   uint32  daysUntilOld    (days remaining before escalation)
        //   float   waitTimeHours   (estimated GM wait time)
        if (!packet.hasRemaining(1)) { packet.skipAll(); return; }
        uint8_t gmStatus = packet.readUInt8();
        // Status 6 = GMTICKET_STATUS_HASTEXT - open ticket with text
        if (gmStatus == 6 && packet.hasRemaining(1)) {
            gmTicketText_    = packet.readString();
            uint32_t ageSec  = (packet.hasRemaining(4)) ? packet.readUInt32() : 0;
            /*uint32_t daysLeft =*/ (packet.hasRemaining(4)) ? packet.readUInt32() : 0;
            gmTicketWaitHours_ = (packet.hasRemaining(4))
                ? packet.readFloat() : 0.0f;
            gmTicketActive_ = true;
            char buf[256];
            if (ageSec < 60) {
                std::snprintf(buf, sizeof(buf),
                    "You have an open GM ticket (submitted %us ago). Estimated wait: %.1f hours.",
                    ageSec, gmTicketWaitHours_);
            } else {
                uint32_t ageMin = ageSec / 60;
                std::snprintf(buf, sizeof(buf),
                    "You have an open GM ticket (submitted %um ago). Estimated wait: %.1f hours.",
                    ageMin, gmTicketWaitHours_);
            }
            addSystemChatMessage(buf);
            LOG_INFO("SMSG_GMTICKET_GETTICKET: open ticket age=", ageSec,
                     "s wait=", gmTicketWaitHours_, "h");

            // The interface measures all of this in days and works the wait out
            // itself, as (oldestTicketTime - ticketAge). This packet gives the
            // wait directly, so the oldest time is the age plus it - which
            // arrives at the same number the server meant.
            //
            // updateTime says how stale the estimate is; anything over an hour
            // is treated as no estimate at all, and this one was computed from
            // the packet that just arrived.
            const double ageDays = static_cast<double>(ageSec) / 86400.0;
            auto days = [](double v) {
                char out[32];
                std::snprintf(out, sizeof(out), "%.6f", v);
                return std::string(out);
            };
            fireAddonEvent("UPDATE_TICKET", {
                "1",                                            // category: has one
                gmTicketText_,
                days(ageDays),
                days(ageDays + gmTicketWaitHours_ / 24.0),
                "0",                                            // just measured
                "0",                                            // not assigned
                "0",                                            // not opened by a GM
            });
        } else if (gmStatus == 3) {
            gmTicketActive_ = false;
            gmTicketText_.clear();
            addSystemChatMessage("Your GM ticket has been closed.");
            LOG_INFO("SMSG_GMTICKET_GETTICKET: ticket closed");
            // No arguments at all is how the interface is told there is no
            // ticket: it reads the first one and stops if it is absent.
            fireAddonEvent("UPDATE_TICKET", {});
        } else {
            // Everything else is "no ticket", including 10.
            //
            // 10 was read as "suspended" and announced in chat, and this is
            // asked on every login - so an account that had never filed a
            // ticket was told at each one that the one it did not have had
            // been suspended. 0x0A is GMTICKET_STATUS_DEFAULT, the answer a
            // server gives when the player has no ticket at all, which is why
            // it arrived every time and why nothing else ever followed it.
            gmTicketActive_ = false;
            gmTicketText_.clear();
            LOG_DEBUG("SMSG_GMTICKET_GETTICKET: no open ticket (status=", static_cast<int>(gmStatus), ")");
            fireAddonEvent("UPDATE_TICKET", {});
        }
        packet.skipAll();
    };
    // uint32 status: 1 = GM support available, 0 = offline/unavailable
    dispatchTable_[Opcode::SMSG_GMTICKET_SYSTEMSTATUS] = [this](network::Packet& packet) {
        // uint32 status: 1 = GM support available, 0 = offline/unavailable
        if (packet.hasRemaining(4)) {
            uint32_t sysStatus = packet.readUInt32();
            gmSupportAvailable_ = (sysStatus != 0);
            addSystemChatMessage(gmSupportAvailable_
                ? "GM support is currently available."
                : "GM support is currently unavailable.");
            LOG_INFO("SMSG_GMTICKET_SYSTEMSTATUS: available=", gmSupportAvailable_);
            // The help frame disables its "open a ticket" button on anything
            // but GMTICKET_QUEUE_STATUS_ENABLED, which is the number 1 - and
            // gets it from here. Parsed and stored and never said, so a closed
            // queue looked open until the ticket was refused.
            fireAddonEvent("UPDATE_GM_STATUS", {gmSupportAvailable_ ? "1" : "0"});
        }
        packet.skipAll();
    };
    // uint8 runeIndex + uint8 newRuneType (0=Blood,1=Unholy,2=Frost,3=Death)
    dispatchTable_[Opcode::SMSG_CONVERT_RUNE] = [this](network::Packet& packet) {
        // uint8 runeIndex + uint8 newRuneType (0=Blood,1=Unholy,2=Frost,3=Death)
        if (!packet.hasRemaining(2)) {
            packet.skipAll();
            return;
        }
        uint8_t idx  = packet.readUInt8();
        uint8_t type = packet.readUInt8();
        if (idx < 6) {
            playerRunes_[idx].type = static_cast<RuneType>(type & 0x3);
            // Stored and never announced: the rune bar redraws a rune on this
            // event and on nothing else, so a rune converted to Death kept the
            // colour it was drawn with at login.
            fireAddonEvent("RUNE_TYPE_UPDATE", {std::to_string(idx + 1)});
        }
    };
    // uint32 count, then per rune: uint8 type, uint8 elapsed.
    //
    // This read a one-byte "ready mask" and six cooldowns, which is neither the
    // shape nor the meaning. Player::ResyncRunes writes a count first, so the
    // mask was the low byte of that count and every cooldown after it was
    // half a type and half an elapsed value.
    //
    // The polarity was inverted as well. The server sends
    // 255 - cooldown * 51, so 255 means the cooldown has fully elapsed and the
    // rune is ready, and 0 means it was just spent - the opposite of what
    // `1 - cd/255` computed.
    //
    // The type is in here too and was never read, so a rune converted to Death
    // kept whatever it was at login.
    dispatchTable_[Opcode::SMSG_RESYNC_RUNES] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) {
            packet.skipAll();
            return;
        }
        const uint32_t count = packet.readUInt32();
        for (uint32_t i = 0; i < count && i < playerRunes_.size(); ++i) {
            if (!packet.hasRemaining(2)) break;
            const uint8_t type    = packet.readUInt8();
            const uint8_t elapsed = packet.readUInt8();
            playerRunes_[i].type = static_cast<RuneType>(type & 0x3);
            playerRunes_[i].readyFraction = elapsed / 255.0f;
            playerRunes_[i].ready = (elapsed >= 255);
            fireRuneUpdate(i);
        }
        packet.skipAll();
    };
    // uint32 runeMask (bit i=1 → rune i just became ready)
    dispatchTable_[Opcode::SMSG_ADD_RUNE_POWER] = [this](network::Packet& packet) {
        // uint32 runeMask (bit i=1 → rune i just became ready)
        if (!packet.hasRemaining(4)) {
            packet.skipAll();
            return;
        }
        uint32_t runeMask = packet.readUInt32();
        for (int i = 0; i < 6; i++) {
            if (runeMask & (1u << i)) {
                playerRunes_[i].ready = true;
                playerRunes_[i].readyFraction = 1.0f;
                fireRuneUpdate(static_cast<uint32_t>(i));
            }
        }
    };

    // uint8 result: 0=success, 1=failed, 2=disabled
    dispatchTable_[Opcode::SMSG_COMPLAIN_RESULT] = [this](network::Packet& packet) {
        // uint8 result: 0=success, 1=failed, 2=disabled
        if (packet.hasRemaining(1)) {
            uint8_t result = packet.readUInt8();
            if (result == 0)
                addSystemChatMessage("Your complaint has been submitted.");
            else if (result == 2)
                addUIError("Report a Player is currently disabled.");
        }
        packet.skipAll();
    };
    // uint32 slot + packed_guid unit (0 packed = clear slot)
    dispatchTable_[Opcode::SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT] = [this](network::Packet& packet) {
        // uint32 type, then a body that depends on it. The first three types
        // carry a packed guid and a one-byte frame priority; the objective and
        // timer types carry one or two bytes; a refresh carries nothing and is
        // four bytes on the wire.
        //
        // What was here read the type as a slot index and the rest as a guid,
        // which put every engaging boss in slot zero - the value of
        // ENCOUNTER_FRAME_ENGAGE - and wrote a disengaging one into slot one
        // instead of clearing it. The five-byte guard also dropped a refresh
        // outright.
        enum : uint32_t {
            kEngage = 0, kDisengage = 1, kUpdatePriority = 2,
            kAddTimer = 3, kEnableObjective = 4, kUpdateObjective = 5,
            kDisableObjective = 6, kRefreshFrames = 7,
        };
        if (!packet.hasRemaining(4)) { packet.skipAll(); return; }
        const uint32_t type = packet.readUInt32();
        if (!socialHandler_) { packet.skipAll(); return; }

        const uint32_t kSlots = game::SocialHandler::kMaxEncounterSlots;

        switch (type) {
        case kEngage:
        case kDisengage:
        case kUpdatePriority: {
            if (!packet.hasRemaining(1)) { packet.skipAll(); return; }
            const uint64_t unit = packet.readPackedGuid();
            // Non-zero means the encounter wants this unit in a particular
            // frame - Halion's two bodies ask for one and two. Zero, which is
            // the default and what most scripts send, means anywhere.
            const uint8_t priority = packet.hasRemaining(1) ? packet.readUInt8() : 0;
            if (!unit) break;

            uint32_t existing = kSlots;
            uint32_t firstFree = kSlots;
            for (uint32_t i = 0; i < kSlots; ++i) {
                const uint64_t held = socialHandler_->getEncounterUnitGuid(i);
                if (held == unit) existing = i;
                else if (!held && firstFree == kSlots) firstFree = i;
            }

            if (type == kDisengage) {
                if (existing < kSlots) socialHandler_->setEncounterUnitGuid(existing, 0);
                break;
            }

            uint32_t slot = existing;
            if (priority >= 1 && priority <= kSlots) {
                slot = priority - 1u;
            } else if (slot == kSlots) {
                slot = firstFree;
            }
            if (slot >= kSlots) break;   // all five frames already taken
            if (existing < kSlots && existing != slot) {
                socialHandler_->setEncounterUnitGuid(existing, 0);
            }
            socialHandler_->setEncounterUnitGuid(slot, unit);
            LOG_DEBUG("SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT: ",
                      type == kEngage ? "engage" : "priority",
                      " slot=", slot, " guid=0x", std::hex, unit, std::dec);
            break;
        }
        case kRefreshFrames:
            // Nothing changed; the frames are asked to draw themselves again
            // after a unit was destroyed client-side, so the event below is
            // the whole of it.
            break;
        case kAddTimer:
        case kEnableObjective:
        case kDisableObjective:
        case kUpdateObjective:
            // Encounter objective and timer widgets, which nothing draws yet.
            packet.skipAll();
            return;
        default:
            packet.skipAll();
            return;
        }

        // The boss frames redraw all five slots on this event, which is why it
        // carries no argument. Stored and never announced, so an encounter
        // filled its slots and no boss frame appeared.
        fireAddonEvent("INSTANCE_ENCOUNTER_ENGAGE_UNIT", {});
    };
    // charName (cstring) + guid (uint64) + achievementId (uint32) + ...
    dispatchTable_[Opcode::SMSG_SERVER_FIRST_ACHIEVEMENT] = [this](network::Packet& packet) {
        // charName (cstring) + guid (uint64) + achievementId (uint32) + ...
        if (packet.hasData()) {
            std::string charName = packet.readString();
            if (packet.hasRemaining(12)) {
                /*uint64_t guid =*/ packet.readUInt64();
                uint32_t achievementId = packet.readUInt32();
                loadAchievementNameCache();
                auto nit = achievementNameCache_.find(achievementId);
                char buf[256];
                if (nit != achievementNameCache_.end() && !nit->second.empty()) {
                    std::snprintf(buf, sizeof(buf),
                        "%s is the first on the realm to earn: %s!",
                        charName.c_str(), nit->second.c_str());
                } else {
                    std::snprintf(buf, sizeof(buf),
                        "%s is the first on the realm to earn achievement #%u!",
                        charName.c_str(), achievementId);
                }
                addSystemChatMessage(buf);
            }
        }
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_SUSPEND_COMMS] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t seqIdx = packet.readUInt32();
            if (socket) {
                network::Packet ack(wireOpcode(Opcode::CMSG_SUSPEND_COMMS_ACK));
                ack.writeUInt32(seqIdx);
                socket->send(ack);
            }
        }
    };
    // SMSG_PRE_RESURRECT: packed GUID of the player who can self-resurrect.
    // Sent when the dead player has Reincarnation (Shaman), Twisting Nether (Warlock),
    // or Deathpact (Death Knight passive). The client must send CMSG_SELF_RES to accept.
    dispatchTable_[Opcode::SMSG_PRE_RESURRECT] = [this](network::Packet& packet) {
        // SMSG_PRE_RESURRECT: packed GUID of the player who can self-resurrect.
        // Sent when the dead player has Reincarnation (Shaman), Twisting Nether (Warlock),
        // or Deathpact (Death Knight passive). The client must send CMSG_SELF_RES to accept.
        uint64_t targetGuid = packet.readPackedGuid();
        if (targetGuid == playerGuid || targetGuid == 0) {
            selfResAvailable_ = true;
            LOG_INFO("SMSG_PRE_RESURRECT: self-resurrection available (guid=0x",
                     std::hex, targetGuid, std::dec, ")");
        }
    };
    dispatchTable_[Opcode::SMSG_PLAYERBINDERROR] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t error = packet.readUInt32();
            if (error == 0) {
                addUIError("Your hearthstone is not bound.");
                addSystemChatMessage("Your hearthstone is not bound.");
            } else {
                addUIError("Hearthstone bind failed.");
                addSystemChatMessage("Hearthstone bind failed.");
            }
        }
    };
    // The instance boot timer, which this read nothing of.
    //
    // Two uint32s: how long is left in milliseconds, and why. The whole
    // countdown arrives once and nothing more is sent until it stops, so the
    // deadline is worked out here and the remainder counted from it.
    //
    // It also said the wrong thing. The message was "you must be in a raid
    // group to enter this instance", and that is not what the server sends
    // this for: UpdateHomebindTime raises it when the player is standing in an
    // instance they are not valid for - saved to a different id, or in a party
    // inside a raid - and the sixty seconds is until they are teleported to a
    // graveyard. A player got one line of the wrong explanation and no
    // countdown at all.
    dispatchTable_[Opcode::SMSG_RAID_GROUP_ONLY] = [this](network::Packet& packet) {
        uint32_t timerMs = 0;
        if (packet.hasRemaining(8)) {
            timerMs = packet.readUInt32();
            packet.readUInt32();   // reason; only 0 and 1 are ever sent
        }
        packet.skipAll();

        if (timerMs == 0) {
            instanceBootDeadline_.reset();
            fireAddonEvent("INSTANCE_BOOT_STOP", {});
            return;
        }
        instanceBootDeadline_ = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(timerMs);
        const std::string msg = "You are not in this instance's group. You will be "
                                "teleported out in " + std::to_string(timerMs / 1000) +
                                " seconds.";
        addUIError(msg);
        addSystemChatMessage(msg);
        fireAddonEvent("INSTANCE_BOOT_START", {});
    };
    dispatchTable_[Opcode::SMSG_RAID_READY_CHECK_ERROR] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t err = packet.readUInt8();
            if (err == 0) { addUIError("Ready check failed: not in a group."); addSystemChatMessage("Ready check failed: not in a group."); }
            else if (err == 1) { addUIError("Ready check failed: in instance."); addSystemChatMessage("Ready check failed: in instance."); }
            else { addUIError("Ready check failed."); addSystemChatMessage("Ready check failed."); }
        }
    };
    dispatchTable_[Opcode::SMSG_RESET_FAILED_NOTIFY] = [this](network::Packet& packet) {
        addUIError("Cannot reset instance: another player is still inside.");
        raiseUiError("Cannot reset instance: another player is still inside.");
        packet.skipAll();
    };
    // uint32 splitType + uint32 deferTime + string realmName
    // Client must respond with CMSG_REALM_SPLIT to avoid session timeout on some servers.
    dispatchTable_[Opcode::SMSG_REALM_SPLIT] = [this](network::Packet& packet) {
        // uint32 splitType + uint32 deferTime + string realmName
        // Client must respond with CMSG_REALM_SPLIT to avoid session timeout on some servers.
        uint32_t splitType = 0;
        if (packet.hasRemaining(4))
            splitType = packet.readUInt32();
        packet.skipAll();
        if (socket) {
            network::Packet resp(wireOpcode(Opcode::CMSG_REALM_SPLIT));
            resp.writeUInt32(splitType);
            resp.writeString("3.3.5");
            socket->send(resp);
            LOG_DEBUG("SMSG_REALM_SPLIT splitType=", splitType, " - sent CMSG_REALM_SPLIT ack");
        }
    };
    dispatchTable_[Opcode::SMSG_REAL_GROUP_UPDATE] = [this](network::Packet& packet) {
        auto rem = [&]() { return packet.getRemainingSize(); };
        if (rem() < 1) return;
        uint8_t newGroupType = packet.readUInt8();
        if (rem() < 4) return;
        uint32_t newMemberFlags = packet.readUInt32();
        if (rem() < 8) return;
        uint64_t newLeaderGuid = packet.readUInt64();

        if (socialHandler_) {
            auto& pd = socialHandler_->mutablePartyData();
            pd.groupType = newGroupType;
            pd.leaderGuid = newLeaderGuid;

            // Update local player's flags in the member list
            uint64_t localGuid = playerGuid;
            for (auto& m : pd.members) {
                if (m.guid == localGuid) {
                    m.flags = static_cast<uint8_t>(newMemberFlags & 0xFF);
                    break;
                }
            }
        }
        LOG_DEBUG("SMSG_REAL_GROUP_UPDATE groupType=", static_cast<int>(newGroupType),
                  " memberFlags=0x", std::hex, newMemberFlags, std::dec,
                  " leaderGuid=", newLeaderGuid);
        fireAddonEvent("PARTY_LEADER_CHANGED", {});
        fireAddonEvent("GROUP_ROSTER_UPDATE", {});
    };
    dispatchTable_[Opcode::SMSG_PLAY_MUSIC] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t soundId = packet.readUInt32();
            if (playMusicCallback_) playMusicCallback_(soundId);
        }
    };
    dispatchTable_[Opcode::SMSG_PLAY_OBJECT_SOUND] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            // uint32 soundId + uint64 sourceGuid
            uint32_t soundId = packet.readUInt32();
            uint64_t srcGuid = packet.readUInt64();
            LOG_DEBUG("SMSG_PLAY_OBJECT_SOUND: id=", soundId, " src=0x", std::hex, srcGuid, std::dec);
            if (playPositionalSoundCallback_) playPositionalSoundCallback_(soundId, srcGuid);
            else if (playSoundCallback_) playSoundCallback_(soundId);
        } else if (packet.hasRemaining(4)) {
            uint32_t soundId = packet.readUInt32();
            if (playSoundCallback_) playSoundCallback_(soundId);
        }
    };
    // uint64 targetGuid + uint32 visualId (same structure as SMSG_PLAY_SPELL_VISUAL)
    dispatchTable_[Opcode::SMSG_PLAY_SPELL_IMPACT] = [this](network::Packet& packet) {
        // uint64 targetGuid + uint32 visualId (same structure as SMSG_PLAY_SPELL_VISUAL)
        if (!packet.hasRemaining(12)) {
            packet.skipAll(); return;
        }
        uint64_t impTargetGuid = packet.readUInt64();
        uint32_t impVisualId   = packet.readUInt32();
        if (impVisualId == 0) return;
        auto* renderer = services_.renderer;
        if (!renderer) return;
        glm::vec3 spawnPos;
        if (impTargetGuid == playerGuid) {
            spawnPos = renderer->getCharacterPosition();
        } else {
            auto entity = entityController_->getEntityManager().getEntity(impTargetGuid);
            if (!entity) return;
            glm::vec3 canonical(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
            spawnPos = core::coords::canonicalToRender(canonical);
        }
        if (auto* sv = renderer->getSpellVisualSystem()) sv->playSpellVisual(impVisualId, spawnPos, /*useImpactKit=*/true);
    };
    // SMSG_READ_ITEM_OK - moved to InventoryHandler::registerOpcodes
    // SMSG_READ_ITEM_FAILED - moved to InventoryHandler::registerOpcodes
    // SMSG_QUERY_QUESTS_COMPLETED_RESPONSE - moved to QuestHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_NPC_WONT_TALK] = [this](network::Packet& packet) {
        addUIError("That creature can't talk to you right now.");
        raiseUiError("That creature can't talk to you right now.");
        packet.skipAll();
    };

    // (SMSG_PET_UNLEARN_CONFIRM → registered by SpellHandler, which owns the
    // pending-unlearn state the confirm dialog reads. Handling it here wrote
    // to dead duplicate members and the dialog never appeared.)
    // These pet opcodes have incompatible formats - just consume the packet.
    // Previously they shared the unlearn handler, which misinterpreted sound IDs
    // or GUID lists as unlearn costs and could trigger a bogus unlearn dialog.
    for (auto op : { Opcode::SMSG_PET_GUIDS, Opcode::SMSG_PET_DISMISS_SOUND,
                     Opcode::SMSG_PET_ACTION_SOUND }) {
        dispatchTable_[op] = [](network::Packet& packet) { packet.skipAll(); };
    }
    // Server signals that the pet can now be named (first tame)
    dispatchTable_[Opcode::SMSG_PET_RENAMEABLE] = [this](network::Packet& packet) {
        // Server signals that the pet can now be named (first tame)
        petRenameablePending_ = true;
        // The naming dialog opens on this. The flag alone is read by this
        // client's own UI, so the interface's version never appeared.
        if (addonEventCallback_) addonEventCallback_("PET_RENAMEABLE", {});
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_PET_NAME_INVALID] = [this](network::Packet& packet) {
        addUIError("That pet name is invalid. Please choose a different name.");
        raiseUiError("That pet name is invalid. Please choose a different name.");
        packet.skipAll();
    };
    // Classic 1.12: PackedGUID + 19×uint32 itemEntries (EQUIPMENT_SLOT_END=19)
    // This opcode is only reachable on Classic servers; TBC/WotLK wire 0x115 maps to
    // SMSG_INSPECT_RESULTS_UPDATE which is handled separately.
    dispatchTable_[Opcode::SMSG_INSPECT] = [this](network::Packet& packet) {
        // Classic 1.12: PackedGUID + 19×uint32 itemEntries (EQUIPMENT_SLOT_END=19)
        // This opcode is only reachable on Classic servers; TBC/WotLK wire 0x115 maps to
        // SMSG_INSPECT_RESULTS_UPDATE which is handled separately.
        if (!packet.hasRemaining(2)) {
            packet.skipAll(); return;
        }
        uint64_t guid = packet.readPackedGuid();
        if (guid == 0) { packet.skipAll(); return; }

        constexpr int kGearSlots = 19;
        size_t needed = kGearSlots * sizeof(uint32_t);
        if (!packet.hasRemaining(needed)) {
            packet.skipAll(); return;
        }

        std::array<uint32_t, 19> items{};
        for (int s = 0; s < kGearSlots; ++s)
            items[s] = packet.readUInt32();

        // Resolve player name
        auto ent = entityController_->getEntityManager().getEntity(guid);
        std::string playerName = "Target";
        if (ent) {
            auto pl = std::dynamic_pointer_cast<Player>(ent);
            if (pl && !pl->getName().empty()) playerName = pl->getName();
        }

        // Populate inspect result immediately (no talent data in Classic SMSG_INSPECT)
        if (socialHandler_) {
            auto& ir = socialHandler_->mutableInspectResult();
            ir.guid           = guid;
            ir.playerName     = playerName;
            ir.totalTalents   = 0;
            ir.unspentTalents = 0;
            ir.talentGroups   = 0;
            ir.activeTalentGroup = 0;
            ir.itemEntries    = items;
            ir.enchantIds     = {};
        }

        cacheInspectedPlayerEquipment(guid, items);

        LOG_INFO("SMSG_INSPECT (Classic): ", playerName, " has gear in ",
                 std::count_if(items.begin(), items.end(),
                               [](uint32_t e) { return e != 0; }), "/19 slots");
        if (addonEventCallback_) {
            char guidBuf[32];
            snprintf(guidBuf, sizeof(guidBuf), "0x%016llX", (unsigned long long)guid);
            fireAddonEvent("INSPECT_READY", {guidBuf});
            // INSPECT_READY is a later expansion's name and nothing in this
            // interface listens for it. The inspect paperdoll refreshes on
            // UNIT_INVENTORY_CHANGED for the unit it is showing, so the gear
            // arrived and the window was never told - it kept whatever it had
            // when it opened. Fired for the unit token this guid answers to,
            // and only when it answers to one: an inspected player who is not
            // the target has no token for the interface to match against.
            const std::string inspectUnit = guidToUnitId(guid);
            if (!inspectUnit.empty()) {
                fireAddonEvent("UNIT_INVENTORY_CHANGED", {inspectUnit});
            }
        }
    };
    // Same wire format as SMSG_COMPRESSED_MOVES: uint8 size + uint16 opcode + payload[]
    dispatchTable_[Opcode::SMSG_MULTIPLE_MOVES] = [this](network::Packet& packet) {
        // Same wire format as SMSG_COMPRESSED_MOVES: uint8 size + uint16 opcode + payload[]
        if (movementHandler_) movementHandler_->handleCompressedMoves(packet);
    };
    // Each sub-packet uses the standard WotLK server wire format:
    //   uint16_be subSize  (includes the 2-byte opcode; payload = subSize - 2)
    //   uint16_le subOpcode
    //   payload  (subSize - 2 bytes)
    dispatchTable_[Opcode::SMSG_MULTIPLE_PACKETS] = [this](network::Packet& packet) {
        // Each sub-packet uses the standard WotLK server wire format:
        //   uint16_be subSize  (includes the 2-byte opcode; payload = subSize - 2)
        //   uint16_le subOpcode
        //   payload  (subSize - 2 bytes)
        const auto& pdata = packet.getData();
        size_t dataLen = pdata.size();
        size_t pos = packet.getReadPos();
        static uint32_t multiPktWarnCount = 0;
        std::vector<network::Packet> subPackets;
        while (pos + 4 <= dataLen) {
            uint16_t subSize = static_cast<uint16_t>(
                (static_cast<uint16_t>(pdata[pos]) << 8) | pdata[pos + 1]);
            if (subSize < 2) break;
            size_t payloadLen = subSize - 2;
            if (pos + 4 + payloadLen > dataLen) {
                if (++multiPktWarnCount <= 10) {
                    LOG_WARNING("SMSG_MULTIPLE_PACKETS: sub-packet overruns buffer at pos=",
                                pos, " subSize=", subSize, " dataLen=", dataLen);
                }
                break;
            }
            uint16_t subOpcode = static_cast<uint16_t>(pdata[pos + 2]) |
                                 (static_cast<uint16_t>(pdata[pos + 3]) << 8);
            std::vector<uint8_t> subPayload(pdata.begin() + pos + 4,
                                            pdata.begin() + pos + 4 + payloadLen);
            subPackets.emplace_back(subOpcode, std::move(subPayload));
            pos += 4 + payloadLen;
        }
        for (auto& subPacket : std::views::reverse(subPackets)) {
            enqueueIncomingPacketFront(std::move(subPacket));
        }
        packet.skipAll();
    };
    // Recruit-A-Friend: a mentor is offering to grant you a level
    dispatchTable_[Opcode::SMSG_PROPOSE_LEVEL_GRANT] = [this](network::Packet& packet) {
        // Recruit-A-Friend: a mentor is offering to grant you a level.
        // HandleGrantLevel writes GetPackGUID(), so this is packed.
        if (packet.hasRemaining(1)) {
            uint64_t mentorGuid = packet.readPackedGuid();
            std::string mentorName;
            auto ent = entityController_->getEntityManager().getEntity(mentorGuid);
            if (auto* unit = dynamic_cast<Unit*>(ent.get())) mentorName = unit->getName();
            if (mentorName.empty()) mentorName = lookupName(mentorGuid);
            addSystemChatMessage(mentorName.empty()
                ? "A player is offering to grant you a level."
                : (mentorName + " is offering to grant you a level."));
        }
        packet.skipAll();
    };
    // SMSG_REFER_A_FRIEND_EXPIRED - moved to SocialHandler::registerOpcodes
    // SMSG_REFER_A_FRIEND_FAILURE - moved to SocialHandler::registerOpcodes
    // SMSG_REPORT_PVP_AFK_RESULT - moved to SocialHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_RESPOND_INSPECT_ACHIEVEMENTS] = [this](network::Packet& packet) {
        loadAchievementNameCache();
        if (!packet.hasRemaining(1)) return;
        uint64_t inspectedGuid = packet.readPackedGuid();
        if (inspectedGuid == 0) { packet.skipAll(); return; }
        std::unordered_map<uint32_t, uint32_t> achievements;
        while (packet.hasRemaining(4)) {
            uint32_t id = packet.readUInt32();
            if (id == 0xFFFFFFFF) break;
            if (!packet.hasRemaining(4)) break;
            // Kept rather than skipped: the comparison tab prints the date each
            // row was earned beside it.
            achievements[id] = packet.readUInt32();
        }
        while (packet.hasRemaining(4)) {
            uint32_t id = packet.readUInt32();
            if (id == 0xFFFFFFFF) break;
            if (!packet.hasRemaining(16)) break;
            packet.readUInt64(); packet.readUInt32(); packet.readUInt32();
        }
        inspectedPlayerAchievements_[inspectedGuid] = std::move(achievements);
        LOG_INFO("SMSG_RESPOND_INSPECT_ACHIEVEMENTS: guid=0x", std::hex, inspectedGuid, std::dec,
                 " achievements=", inspectedPlayerAchievements_[inspectedGuid].size());
        // The comparison tab reads its totals on this and nothing else, so
        // without it the panel sat on whatever it had when it opened.
        fireAddonEvent("INSPECT_ACHIEVEMENT_READY", {});
    };
    dispatchTable_[Opcode::SMSG_ON_CANCEL_EXPECTED_RIDE_VEHICLE_AURA] = [this](network::Packet& packet) {
        const bool wasRiding = vehicleId_ != 0;
        vehicleId_ = 0;  // Vehicle ride cancelled; clear UI
        if (vehicleStateCallback_) {
            vehicleStateCallback_(false, 0);
        }
        if (wasRiding) {
            fireAddonEvent("VEHICLE_UPDATE", {});
            fireAddonEvent("UNIT_EXITED_VEHICLE", {"player"});
            fireAddonEvent("UPDATE_BONUS_ACTIONBAR", {});
        }
        packet.skipAll();
    };
    // uint32 type (0=normal, 1=heavy, 2=tired/restricted) + uint32 minutes played
    dispatchTable_[Opcode::SMSG_PLAY_TIME_WARNING] = [this](network::Packet& packet) {
        // uint32 type (0=normal, 1=heavy, 2=tired/restricted) + uint32 minutes played
        if (packet.hasRemaining(4)) {
            uint32_t warnType = packet.readUInt32();
            uint32_t minutesPlayed = (packet.hasRemaining(4))
                ? packet.readUInt32() : 0;
            const char* severity = (warnType >= 2) ? "[Tired] " : "[Play Time] ";
            char buf[128];
            if (minutesPlayed > 0) {
                uint32_t h = minutesPlayed / 60;
                uint32_t m = minutesPlayed % 60;
                if (h > 0)
                    std::snprintf(buf, sizeof(buf), "%sYou have been playing for %uh %um.", severity, h, m);
                else
                    std::snprintf(buf, sizeof(buf), "%sYou have been playing for %um.", severity, m);
            } else {
                std::snprintf(buf, sizeof(buf), "%sYou have been playing for a long time.", severity);
            }
            addSystemChatMessage(buf);
            addUIError(buf);
        }
    };
    // WotLK 3.3.5a format:
    //   uint64 mirrorGuid - GUID of the mirror image unit
    //   uint32 displayId  - display ID to render the image with
    //   uint8  raceId     - race of caster
    //   uint8  genderFlag - gender of caster
    //   uint8  classId    - class of caster
    //   uint64 casterGuid - GUID of the player who cast the spell
    //   Followed by equipped item display IDs (11 × uint32) if casterGuid != 0
    // Purpose: tells client how to render the image (same appearance as caster).
    // We parse the GUIDs so units render correctly via their existing display IDs.
    dispatchTable_[Opcode::SMSG_MIRRORIMAGE_DATA] = [this](network::Packet& packet) {
        // WotLK 3.3.5a format:
        //   uint64 mirrorGuid - GUID of the mirror image unit
        //   uint32 displayId  - display ID to render the image with
        //   uint8  raceId     - race of caster
        //   uint8  genderFlag - gender of caster
        //   uint8  classId    - class of caster
        //   uint64 casterGuid - GUID of the player who cast the spell
        //   Followed by equipped item display IDs (11 × uint32) if casterGuid != 0
        // Purpose: tells client how to render the image (same appearance as caster).
        // We parse the GUIDs so units render correctly via their existing display IDs.
        if (!packet.hasRemaining(8)) return;
        uint64_t mirrorGuid = packet.readUInt64();
        if (!packet.hasRemaining(4)) return;
        uint32_t displayId  = packet.readUInt32();
        if (!packet.hasRemaining(3)) return;
        /*uint8_t raceId   =*/ packet.readUInt8();
        /*uint8_t gender   =*/ packet.readUInt8();
        /*uint8_t classId  =*/ packet.readUInt8();
        // Apply display ID to the mirror image unit so it renders correctly
        if (mirrorGuid != 0 && displayId != 0) {
            auto entity = entityController_->getEntityManager().getEntity(mirrorGuid);
            if (entity) {
                auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
                if (unit && unit->getDisplayId() == 0)
                    unit->setDisplayId(displayId);
            }
        }
        LOG_DEBUG("SMSG_MIRRORIMAGE_DATA: mirrorGuid=0x", std::hex, mirrorGuid,
                  " displayId=", std::dec, displayId);
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint32 zoneId + uint64 expireUnixTime (seconds)
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_ENTRY_INVITE] = [this](network::Packet& packet) {
        // uint32 battleId + uint32 zoneId + uint32 acceptTime - twelve bytes.
        //
        // This required twenty and read a 64-bit guid and a 64-bit time that
        // the server does not send, so it returned on the length check every
        // time and the handler never ran once. The event was written down as
        // never fired, which was true and was the symptom rather than the
        // cause.
        if (!packet.hasRemaining(12)) {
            packet.skipAll(); return;
        }
        const uint32_t bfBattleId = packet.readUInt32();
        const uint32_t bfZoneId   = packet.readUInt32();
        const uint32_t expireTime = packet.readUInt32();
        (void)expireTime;
        bfMgrBattleId_ = bfBattleId;
        // Store the invitation so the UI can show a prompt
        bfMgrInvitePending_ = true;
        bfMgrZoneId_        = bfZoneId;
        char buf[128];
        std::string bfZoneName = getAreaName(bfZoneId);
        if (!bfZoneName.empty())
            std::snprintf(buf, sizeof(buf),
                "You are invited to the outdoor battlefield in %s. Click to enter.",
                bfZoneName.c_str());
        else
            std::snprintf(buf, sizeof(buf),
                "You are invited to the outdoor battlefield in zone %u. Click to enter.",
                bfZoneId);
        addSystemChatMessage(buf);
        // The prompt to enter is a static popup the interface raises from this.
        // The zone id goes with it as the battle id: every handler in the group
        // opens `local battleID = ...` and passes it back when the player
        // answers, so a bare fire would answer for no battlefield.
        if (addonEventCallback_)
            addonEventCallback_("BATTLEFIELD_MGR_ENTRY_INVITE", {std::to_string(bfBattleId)});
        LOG_INFO("SMSG_BATTLEFIELD_MGR_ENTRY_INVITE: zoneId=", bfZoneId);
    };
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_ENTERED] = [this](network::Packet& packet) {
        // uint32 battleId, then three flags - seven bytes. This read a 64-bit
        // guid the server does not send and guarded eight, so on a seven-byte
        // packet it never ran: entering Wintergrasp announced nothing.
        if (packet.hasRemaining(7)) {
            const uint32_t bfBattleId = packet.readUInt32();
            bfMgrBattleId_ = bfBattleId;
            packet.readUInt8();   // unk, always one
            uint8_t isSafe  = packet.readUInt8();
            uint8_t onQueue = packet.readUInt8();
            bfMgrInvitePending_ = false;
            bfMgrActive_        = true;
            addSystemChatMessage(isSafe ? "You are in the battlefield zone (safe area)."
                                        : "You have entered the battlefield!");
            if (onQueue) addSystemChatMessage("You are in the battlefield queue.");
            if (addonEventCallback_)
                addonEventCallback_("BATTLEFIELD_MGR_ENTERED", {std::to_string(bfMgrZoneId_)});
            LOG_INFO("SMSG_BATTLEFIELD_MGR_ENTERED: isSafe=", static_cast<int>(isSafe), " onQueue=", static_cast<int>(onQueue));
        }
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint32 battlefieldId + uint64 expireTime
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_QUEUE_INVITE] = [this](network::Packet& packet) {
        // uint32 battleId + uint8 warmup - five bytes, and this required
        // twenty. Same shape as the entry invite above: never ran, and the
        // event it fires was recorded as one the server never sends.
        if (!packet.hasRemaining(5)) {
            packet.skipAll(); return;
        }
        const uint32_t bfId = packet.readUInt32();
        const bool warmup = packet.readUInt8() != 0;
        bfMgrBattleId_ = bfId;
        bfMgrInvitePending_ = true;
        bfMgrZoneId_        = bfId;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "A spot has opened in the battlefield queue (battlefield %u).", bfId);
        addSystemChatMessage(buf);
        if (addonEventCallback_)
            // The warmup flag picks between two dialogs and was read off the
            // wire and thrown away, so the warmup one could never appear.
            addonEventCallback_("BATTLEFIELD_MGR_QUEUE_INVITE",
                                {std::to_string(bfId), eventBool(warmup)});
        LOG_INFO("SMSG_BATTLEFIELD_MGR_QUEUE_INVITE: bfId=", bfId);
    };
    // uint32 battlefieldId + uint32 teamId + uint8 accepted + uint8 loggingEnabled + uint8 result
    // result: 0=queued, 1=not_in_group, 2=too_high_level, 3=too_low_level,
    //         4=in_cooldown, 5=queued_other_bf, 6=bf_full
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_QUEUE_REQUEST_RESPONSE] = [this](network::Packet& packet) {
        // uint32 battlefieldId + uint32 teamId + uint8 accepted + uint8 loggingEnabled + uint8 result
        // result: 0=queued, 1=not_in_group, 2=too_high_level, 3=too_low_level,
        //         4=in_cooldown, 5=queued_other_bf, 6=bf_full
        if (!packet.hasRemaining(11)) {
            packet.skipAll(); return;
        }
        uint32_t bfId2    = packet.readUInt32();
        /*uint32_t teamId =*/ packet.readUInt32();
        uint8_t accepted  = packet.readUInt8();
        /*uint8_t logging =*/ packet.readUInt8();
        uint8_t result    = packet.readUInt8();
        (void)bfId2;
        if (accepted) {
            addSystemChatMessage("You have joined the battlefield queue.");
        } else {
            static const char* kBfQueueErrors[] = {
                "Queued for battlefield.", "Not in a group.", "Level too high.",
                "Level too low.", "Battlefield in cooldown.", "Already queued for another battlefield.",
                "Battlefield is full."
            };
            const char* msg = (result < 7) ? kBfQueueErrors[result]
                                           : "Battlefield queue request failed.";
            addSystemChatMessage(std::string("Battlefield: ") + msg);
        }
        // The battle id and whether the queue took, in that order - the
        // interface reads both to say which battlefield answered and how.
        if (addonEventCallback_) {
            addonEventCallback_("BATTLEFIELD_MGR_QUEUE_REQUEST_RESPONSE",
                                {std::to_string(bfId2), std::to_string(accepted)});
        }
        LOG_INFO("SMSG_BATTLEFIELD_MGR_QUEUE_REQUEST_RESPONSE: accepted=", static_cast<int>(accepted),
                 " result=", static_cast<int>(result));
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint8 remove
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_EJECT_PENDING] = [this](network::Packet& packet) {
        // A battle id and nothing else. This read a packed pair of a guid and a
        // flag across nine bytes, so the guard alone kept it from running.
        //
        // AzerothCore declares this opcode and never sends it, so nothing here
        // has been exercised - but a wrong reading is worse than an untested
        // one, and the frame that answers it is live.
        if (!packet.hasRemaining(4)) { packet.skipAll(); return; }
        const uint32_t battleId = packet.readUInt32();
        packet.skipAll();

        addSystemChatMessage("You will be removed from the battlefield shortly.");
        LOG_INFO("SMSG_BATTLEFIELD_MGR_EJECT_PENDING: battle ", battleId);
        // The second argument picks between two dialogs and nothing here knows
        // which; nil is the local one, which is what a player being ejected
        // from the battlefield they are standing in should see.
        fireAddonEvent("BATTLEFIELD_MGR_EJECT_PENDING",
                       {std::to_string(battleId), kEventNil});
    };
    // uint64 battlefieldGuid + uint32 reason + uint32 battleStatus + uint8 relocated
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_EJECTED] = [this](network::Packet& packet) {
        // Seven bytes: a battle id, then three single bytes. SendBfLeaveMessage
        // writes exactly that.
        //
        // This guarded seventeen and read a guid, two thirty-two-bit fields and
        // a flag, so it never ran once - and the reason it treated as a small
        // enum is a bit mask, which would have named the wrong thing if it had.
        if (!packet.hasRemaining(7)) { packet.skipAll(); return; }
        const uint32_t battleId = packet.readUInt32();
        const uint8_t reason = packet.readUInt8();
        const uint8_t battleStatus = packet.readUInt8();
        const uint8_t relocated = packet.readUInt8();
        packet.skipAll();

        constexpr uint8_t kClose = 0x01, kExited = 0x08, kLowLevel = 0x10;
        const bool exited = (reason & kExited) != 0;
        const bool lowLevel = (reason & kLowLevel) != 0;

        if (lowLevel) {
            raiseUiError("You are not high enough level for this battlefield.");
        } else if (reason & kClose) {
            addSystemChatMessage("The battlefield has closed.");
        } else if (exited) {
            addSystemChatMessage("You have left the battlefield.");
        } else {
            addSystemChatMessage("You have been removed from the battlefield.");
        }

        LOG_INFO("SMSG_BATTLEFIELD_MGR_EJECTED: battle ", battleId,
                 " reason 0x", std::hex, static_cast<int>(reason), std::dec,
                 " status ", static_cast<int>(battleStatus),
                 relocated ? " (relocated)" : "");

        bfMgrActive_        = false;
        bfMgrInvitePending_ = false;

        // battleID, playerExited, relocated, battleActive, lowLevel. The four
        // flags are booleans in the middle of the list, so they go through
        // eventBool - a zero here would read as true and pick the wrong dialog.
        fireAddonEvent("BATTLEFIELD_MGR_EJECTED",
                       {std::to_string(battleId),
                        eventBool(exited),
                        eventBool(relocated != 0),
                        eventBool(battleStatus != 0),
                        eventBool(lowLevel)});
    };
    // uint32 oldState + uint32 newState
    // States: 0=Waiting, 1=Starting, 2=InProgress, 3=Ending, 4=Cooldown
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_STATE_CHANGE] = [this](network::Packet& packet) {
        // uint32 oldState + uint32 newState
        // States: 0=Waiting, 1=Starting, 2=InProgress, 3=Ending, 4=Cooldown
        if (packet.hasRemaining(8)) {
            /*uint32_t oldState =*/ packet.readUInt32();
            uint32_t newState   = packet.readUInt32();
            static const char* kBfStates[] = {
                "waiting", "starting", "in progress", "ending", "in cooldown"
            };
            const char* stateStr = (newState < 5) ? kBfStates[newState] : "unknown state";
            char buf[128];
            std::snprintf(buf, sizeof(buf), "Battlefield is now %s.", stateStr);
            addSystemChatMessage(buf);
            LOG_INFO("SMSG_BATTLEFIELD_MGR_STATE_CHANGE: newState=", newState);
        }
        packet.skipAll();
    };
    // One event in full, in answer to opening it.
    dispatchTable_[Opcode::SMSG_CALENDAR_SEND_EVENT] = [this](network::Packet& packet) {
        CalendarEventDetail parsed;
        if (!parseCalendarSendEvent(packet, parsed)) {
            LOG_WARNING("SMSG_CALENDAR_SEND_EVENT: could not be read whole; "
                        "keeping the previous event rather than a partial one");
            packet.skipAll();
            return;
        }
        calendarEventDetail_ = std::move(parsed);
        LOG_INFO("SMSG_CALENDAR_SEND_EVENT: '", calendarEventDetail_.title,
                 "' with ", calendarEventDetail_.invitees.size(), " invitee(s)");
        if (addonEventCallback_) addonEventCallback_("CALENDAR_UPDATE_EVENT", {});
        packet.skipAll();
    };

    // The whole calendar, in answer to a request for it.
    //
    // Six lists back to back with no length prefix on any of them, so this is
    // parsed in one place with a test rather than read inline - a row taken
    // one field wrong slides everything after it and the next count is read
    // out of the middle of a string, which fails silently and completely.
    // See parseCalendarSendCalendar.
    dispatchTable_[Opcode::SMSG_CALENDAR_SEND_CALENDAR] = [this](network::Packet& packet) {
        CalendarData parsed;
        if (!parseCalendarSendCalendar(packet, parsed)) {
            LOG_WARNING("SMSG_CALENDAR_SEND_CALENDAR: could not be read whole; "
                        "keeping the previous calendar rather than a partial one");
            packet.skipAll();
            return;
        }
        calendarData_ = std::move(parsed);
        LOG_INFO("SMSG_CALENDAR_SEND_CALENDAR: ", calendarData_.events.size(),
                 " event(s), ", calendarData_.invites.size(), " invite(s), ",
                 calendarData_.lockouts.size(), " raid lockout(s), ",
                 calendarData_.holidays.size(), " holiday(s)");
        // The addon reads the whole thing on this event and nothing else
        // announces that it arrived.
        if (addonEventCallback_) addonEventCallback_("CALENDAR_UPDATE_EVENT_LIST", {});
        packet.skipAll();
    };
    // uint32 numPending - number of unacknowledged calendar invites
    dispatchTable_[Opcode::SMSG_CALENDAR_SEND_NUM_PENDING] = [this](network::Packet& packet) {
        // uint32 numPending - number of unacknowledged calendar invites
        if (packet.hasRemaining(4)) {
            uint32_t numPending = packet.readUInt32();
            const bool countChanged = (calendarPendingInvites_ != numPending);
            calendarPendingInvites_ = numPending;
            // The minimap's date button carries the indicator: gametime.lua
            // shows GameTimeCalendarInvitesTexture when the count rises and
            // hides it at zero, and it only ever asks on this event and on
            // PLAYER_ENTERING_WORLD. Firing nothing left it correct at login
            // and stale for the rest of the session.
            if (countChanged && addonEventCallback_)
                addonEventCallback_("CALENDAR_UPDATE_PENDING_INVITES", {});
            if (numPending > 0) {
                char buf[64];
                std::snprintf(buf, sizeof(buf),
                    "You have %u pending calendar invite%s.",
                    numPending, numPending == 1 ? "" : "s");
                addSystemChatMessage(buf);
            }
            LOG_DEBUG("SMSG_CALENDAR_SEND_NUM_PENDING: ", numPending, " pending invites");
        }
    };
    // uint32 command + uint8 result + cstring info
    // result 0 = success; non-zero = error code
    // command values: 0=add,1=get,2=guild_filter,3=arena_team,4=update,5=remove,
    //                 6=copy,7=invite,8=rsvp,9=remove_invite,10=status,11=moderator_status
    dispatchTable_[Opcode::SMSG_CALENDAR_COMMAND_RESULT] = [this](network::Packet& packet) {
        // uint32 command + uint8 result + cstring info
        // result 0 = success; non-zero = error code
        // command values: 0=add,1=get,2=guild_filter,3=arena_team,4=update,5=remove,
        //                 6=copy,7=invite,8=rsvp,9=remove_invite,10=status,11=moderator_status
        if (!packet.hasRemaining(5)) {
            packet.skipAll(); return;
        }
        /*uint32_t command =*/ packet.readUInt32();
        uint8_t result    = packet.readUInt8();
        std::string info  = (packet.hasData()) ? packet.readString() : "";
        if (result != 0) {
            // Map common calendar error codes to friendly strings
            static const char* kCalendarErrors[] = {
                "",
                "Calendar: Internal error.",           // 1 = CALENDAR_ERROR_INTERNAL
                "Calendar: Guild event limit reached.",// 2
                "Calendar: Event limit reached.",      // 3
                "Calendar: You cannot invite that player.", // 4
                "Calendar: No invites remaining.",     // 5
                "Calendar: Invalid date.",             // 6
                "Calendar: Cannot invite yourself.",   // 7
                "Calendar: Cannot modify this event.", // 8
                "Calendar: Not invited.",              // 9
                "Calendar: Already invited.",          // 10
                "Calendar: Player not found.",         // 11
                "Calendar: Not enough focus.",         // 12
                "Calendar: Event locked.",             // 13
                "Calendar: Event deleted.",            // 14
                "Calendar: Not a moderator.",          // 15
            };
            const char* errMsg = (result < 16) ? kCalendarErrors[result]
                                               : "Calendar: Command failed.";
            if (errMsg && errMsg[0] != '\0') addSystemChatMessage(errMsg);
            else if (!info.empty()) addSystemChatMessage("Calendar: " + info);
        }
        packet.skipAll();
    };
    // Rich notification: eventId(8) + title(cstring) + eventTime(8) + flags(4) +
    //                   eventType(1) + dungeonId(4) + inviteId(8) + status(1) + rank(1) +
    //                   isGuildEvent(1) + inviterGuid(8)
    dispatchTable_[Opcode::SMSG_CALENDAR_EVENT_INVITE_ALERT] = [this](network::Packet& packet) {
        // eventId(8) + title(cstring) + eventTime(4, packed) + flags(4) +
        // eventType(4) + dungeonId(4, signed) + inviteId(8) + status(1) +
        // rank(1) + creatorGuid(packed) + senderGuid(packed).
        //
        // Only the id and the title are read; the rest is skipped. The widths
        // written here before were a different shape - an eight-byte time, a
        // one-byte type, an isGuildEvent that is not sent, and two full guids
        // where the wire has packed ones - which would misread everything past
        // the title for whoever went to use it.
        if (!packet.hasRemaining(9)) {
            packet.skipAll(); return;
        }
        /*uint64_t eventId =*/ packet.readUInt64();
        std::string title = (packet.hasData()) ? packet.readString() : "";
        packet.skipAll(); // consume remaining fields
        if (!title.empty()) {
            addSystemChatMessage("Calendar invite: " + title);
        } else {
            addSystemChatMessage("You have a new calendar invite.");
        }
        if (calendarPendingInvites_ < 255) ++calendarPendingInvites_;
        // Same indicator, and this is the path that matters most: an invite
        // arriving mid-session is exactly when the button has to light up.
        if (addonEventCallback_)
            addonEventCallback_("CALENDAR_UPDATE_PENDING_INVITES", {});
        LOG_INFO("SMSG_CALENDAR_EVENT_INVITE_ALERT: title='", title, "'");
    };
    // Sent when an event invite's RSVP status changes for the local player
    // Format: inviteId(8) + eventId(8) + eventType(1) + flags(4) +
    //         inviteTime(8) + status(1) + rank(1) + isGuildEvent(1) + title(cstring)
    dispatchTable_[Opcode::SMSG_CALENDAR_EVENT_STATUS] = [this](network::Packet& packet) {
        // Sent when an event invite's RSVP status changes for the local player
        // Format: inviteId(8) + eventId(8) + eventType(1) + flags(4) +
        //         inviteTime(8) + status(1) + rank(1) + isGuildEvent(1) + title(cstring)
        if (!packet.hasRemaining(31)) {
            packet.skipAll(); return;
        }
        /*uint64_t inviteId =*/ packet.readUInt64();
        /*uint64_t eventId  =*/ packet.readUInt64();
        /*uint8_t  evType   =*/ packet.readUInt8();
        /*uint32_t flags    =*/ packet.readUInt32();
        /*uint64_t invTime  =*/ packet.readUInt64();
        uint8_t status     = packet.readUInt8();
        /*uint8_t rank      =*/ packet.readUInt8();
        /*uint8_t isGuild   =*/ packet.readUInt8();
        std::string evTitle = (packet.hasData()) ? packet.readString() : "";
        // status: 0=Invited,1=Accepted,2=Declined,3=Confirmed,4=Out,5=Standby,6=SignedUp,7=Not Signed Up,8=Tentative
        static const char* kRsvpStatus[] = {
            "invited", "accepted", "declined", "confirmed",
            "out", "on standby", "signed up", "not signed up", "tentative"
        };
        const char* statusStr = (status < 9) ? kRsvpStatus[status] : "unknown";
        if (!evTitle.empty()) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Calendar event '%s': your RSVP is %s.",
                          evTitle.c_str(), statusStr);
            addSystemChatMessage(buf);
        }
        // The chat line is the whole of it, deliberately. framexml_event_gap
        // pairs this message with CALENDAR_UPDATE_EVENT on the words they
        // share, but an RSVP changing is CALENDAR_UPDATE_INVITE_LIST in
        // retail - and both are read by a calendar that would re-read nothing,
        // since no event list is kept here for either to draw from.
        packet.skipAll();
    };
    // uint64 inviteId + uint64 eventId + uint32 mapId + uint32 difficulty + uint64 resetTime
    dispatchTable_[Opcode::SMSG_CALENDAR_RAID_LOCKOUT_ADDED] = [this](network::Packet& packet) {
        // uint64 inviteId + uint64 eventId + uint32 mapId + uint32 difficulty + uint64 resetTime
        if (packet.hasRemaining(28)) {
            /*uint64_t inviteId =*/ packet.readUInt64();
            /*uint64_t eventId  =*/ packet.readUInt64();
            uint32_t mapId     = packet.readUInt32();
            uint32_t difficulty = packet.readUInt32();
            /*uint64_t resetTime =*/ packet.readUInt64();
            std::string mapLabel = getMapName(mapId);
            if (mapLabel.empty()) mapLabel = "map #" + std::to_string(mapId);
            static const char* kDiff[] = {"Normal","Heroic","25-Man","25-Man Heroic"};
            const char* diffStr = (difficulty < 4) ? kDiff[difficulty] : nullptr;
            std::string msg = "Calendar: Raid lockout added for " + mapLabel;
            if (diffStr) msg += std::string(" (") + diffStr + ")";
            msg += '.';
            addSystemChatMessage(msg);
            LOG_DEBUG("SMSG_CALENDAR_RAID_LOCKOUT_ADDED: mapId=", mapId, " difficulty=", difficulty);
        }
        packet.skipAll();
    };
    // uint64 inviteId + uint64 eventId + uint32 mapId + uint32 difficulty
    dispatchTable_[Opcode::SMSG_CALENDAR_RAID_LOCKOUT_REMOVED] = [this](network::Packet& packet) {
        // uint64 inviteId + uint64 eventId + uint32 mapId + uint32 difficulty
        if (packet.hasRemaining(20)) {
            /*uint64_t inviteId =*/ packet.readUInt64();
            /*uint64_t eventId  =*/ packet.readUInt64();
            uint32_t mapId     = packet.readUInt32();
            uint32_t difficulty = packet.readUInt32();
            std::string mapLabel = getMapName(mapId);
            if (mapLabel.empty()) mapLabel = "map #" + std::to_string(mapId);
            static const char* kDiff[] = {"Normal","Heroic","25-Man","25-Man Heroic"};
            const char* diffStr = (difficulty < 4) ? kDiff[difficulty] : nullptr;
            std::string msg = "Calendar: Raid lockout removed for " + mapLabel;
            if (diffStr) msg += std::string(" (") + diffStr + ")";
            msg += '.';
            addSystemChatMessage(msg);
            LOG_DEBUG("SMSG_CALENDAR_RAID_LOCKOUT_REMOVED: mapId=", mapId,
                      " difficulty=", difficulty);
        }
        packet.skipAll();
    };
    // uint32 unixTime - server's current unix timestamp; use to sync gameTime_
    dispatchTable_[Opcode::SMSG_SERVERTIME] = [](network::Packet& packet) {
        // uint32 unixTime - server's current unix timestamp; use to sync gameTime_
        // A unix timestamp, which is not a time of day. It used to be stored
        // in gameTime_ alongside values in three other units, so whichever
        // packet arrived last decided what the sky thought the hour was.
        // Logged and dropped: nothing here needs the server's wall clock, and
        // the game clock has its own opcodes.
        if (packet.hasRemaining(4)) {
            const uint32_t srvTime = packet.readUInt32();
            LOG_DEBUG("SMSG_SERVERTIME: serverTime=", srvTime);
        }
    };
    // uint64 kickerGuid + uint32 kickReasonType + null-terminated reason string
    // kickReasonType: 0=other, 1=afk, 2=vote kick
    dispatchTable_[Opcode::SMSG_KICK_REASON] = [this](network::Packet& packet) {
        // uint64 kickerGuid + uint32 kickReasonType + null-terminated reason string
        // kickReasonType: 0=other, 1=afk, 2=vote kick
        if (!packet.hasRemaining(12)) {
            packet.skipAll();
            return;
        }
        uint64_t kickerGuid   = packet.readUInt64();
        uint32_t reasonType   = packet.readUInt32();
        std::string reason;
        if (packet.hasData())
            reason = packet.readString();
        (void)kickerGuid;  // not displayed; reasonType IS used below
        std::string msg = "You have been removed from the group.";
        if (!reason.empty())
            msg = "You have been removed from the group: " + reason;
        else if (reasonType == 1)
            msg = "You have been removed from the group for being AFK.";
        else if (reasonType == 2)
            msg = "You have been removed from the group by vote.";
        addSystemChatMessage(msg);
        addUIError(msg);
        LOG_INFO("SMSG_KICK_REASON: reasonType=", reasonType,
                 " reason='", reason, "'");
    };
    // uint32 throttleMs - rate-limited group action; notify the player
    dispatchTable_[Opcode::SMSG_GROUPACTION_THROTTLED] = [this](network::Packet& packet) {
        // uint32 throttleMs - rate-limited group action; notify the player
        if (packet.hasRemaining(4)) {
            uint32_t throttleMs = packet.readUInt32();
            char buf[128];
            if (throttleMs > 0) {
                std::snprintf(buf, sizeof(buf),
                              "Group action throttled. Please wait %.1f seconds.",
                              throttleMs / 1000.0f);
            } else {
                std::snprintf(buf, sizeof(buf), "Group action throttled.");
            }
            addSystemChatMessage(buf);
            LOG_DEBUG("SMSG_GROUPACTION_THROTTLED: throttleMs=", throttleMs);
        }
    };
    // WotLK 3.3.5a: uint32 ticketId + string subject + string body + uint32 count
    //   per count: string responseText
    dispatchTable_[Opcode::SMSG_GMRESPONSE_RECEIVED] = [this](network::Packet& packet) {
        // WotLK 3.3.5a: uint32 ticketId + string subject + string body + uint32 count
        //   per count: string responseText
        if (!packet.hasRemaining(4)) {
            packet.skipAll();
            return;
        }
        uint32_t ticketId = packet.readUInt32();
        std::string subject;
        std::string body;
        if (packet.hasData()) subject = packet.readString();
        if (packet.hasData()) body    = packet.readString();
        uint32_t responseCount = 0;
        if (packet.hasRemaining(4))
            responseCount = packet.readUInt32();
        std::string responseText;
        for (uint32_t i = 0; i < responseCount && i < 10; ++i) {
            if (packet.hasData()) {
                std::string t = packet.readString();
                if (i == 0) responseText = t;
            }
        }
        (void)ticketId;
        std::string msg;
        if (!responseText.empty())
            msg = "[GM Response] " + responseText;
        else if (!body.empty())
            msg = "[GM Response] " + body;
        else if (!subject.empty())
            msg = "[GM Response] " + subject;
        else
            msg = "[GM Response] Your ticket has been answered.";
        addSystemChatMessage(msg);
        addUIError(msg);
        // The help frame takes the ticket's own text and the reply, and opens
        // the panel that lets the answer be accepted or more help asked for.
        // Both strings are parsed right here and were only ever said in chat,
        // so the reply arrived and the frame that deals with it never opened.
        //
        // The description first, then the response - the order it unpacks them
        // - and the body stands in for a description the server did not send,
        // since an empty first argument leaves the panel captionless.
        fireAddonEvent("GMRESPONSE_RECEIVED",
                       {subject.empty() ? body : subject, responseText});
        LOG_INFO("SMSG_GMRESPONSE_RECEIVED: ticketId=", ticketId,
                 " subject='", subject, "'");
    };
    dispatchTable_[Opcode::SMSG_GMRESPONSE_STATUS_UPDATE] = [this](network::Packet& packet) {
        // One byte, and it is not a status: SendGMResponse writes
        // uint8(getSurvey), which asks whether to offer the survey now that the
        // ticket is answered. This read a four-byte ticket id and a status
        // across five bytes, so it never ran once.
        if (packet.hasRemaining(1)) {
            const uint32_t ticketId = 0;
            const uint8_t  status   = packet.readUInt8() ? 2u : 1u;
            const char* statusStr = (status == 1) ? "open"
                                  : (status == 2) ? "answered"
                                  : (status == 3) ? "needs more info"
                                  : "updated";
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "[GM Ticket #%u] Status: %s.", ticketId, statusStr);
            addSystemChatMessage(buf);
            LOG_DEBUG("SMSG_GMRESPONSE_STATUS_UPDATE: ticketId=", ticketId,
                      " status=", static_cast<int>(status));
        }
    };
    // GM ticket status (new/updated); no ticket UI yet
    registerSkipHandler(Opcode::SMSG_GM_TICKET_STATUS_UPDATE);
    // Broadcast of another player's collision height change - cosmetic only.
    registerSkipHandler(Opcode::MSG_MOVE_SET_COLLISION_HGT);
    // Client uses this outbound; treat inbound variant as no-op for robustness.
    registerSkipHandler(Opcode::MSG_MOVE_WORLDPORT_ACK);
    // Observed custom server packet (8 bytes). Safe-consume for now.
    registerSkipHandler(Opcode::MSG_MOVE_TIME_SKIPPED);
    // loggingOut_ already cleared by cancelLogout(); this is server's confirmation
    // Not skipped: this is the server confirming a logout was called off, and
    // the interface hides its countdown on it. The client already cancels
    // optimistically in cancelLogout(), so the ack changes no state here - it
    // is the only thing that tells the interface the countdown is over.
    dispatchTable_[Opcode::SMSG_LOGOUT_CANCEL_ACK] = [this](network::Packet& packet) {
        if (addonEventCallback_) addonEventCallback_("LOGOUT_CANCEL", {});
        packet.skipAll();
    };
    // Someone has taken the player's insignia, so the corpse is gone and any
    // resurrect offer against it is void. The body carries a free-repop flag
    // the interface reads only in code Blizzard commented out, so it is not
    // parsed here - the message and the cancelled offers are the whole effect.
    dispatchTable_[Opcode::SMSG_PLAYER_SKINNED] = [this](network::Packet& packet) {
        addUIError("Insignia Taken - You can only resurrect at the graveyard");
        if (addonEventCallback_) addonEventCallback_("PLAYER_SKINNED", {});
        packet.skipAll();
    };
    // These packets are not damage-shield events. Consume them without
    // synthesizing reflected damage entries or misattributing GUIDs.
    registerSkipHandler(Opcode::SMSG_AURACASTLOG);
    // These packets are not damage-shield events. Consume them without
    // synthesizing reflected damage entries or misattributing GUIDs.
    registerSkipHandler(Opcode::SMSG_SPELLBREAKLOG);
    // Consume silently - informational, no UI action needed
    // What an item cost and how long is left to hand it back.
    //
    // Skipped until now, which is why GetContainerItemPurchaseInfo answered
    // nil and the refund lock never appeared. The client was never sent this
    // unasked - it is a reply to CMSG_ITEM_REFUND_INFO, and nothing asked.
    dispatchTable_[Opcode::SMSG_ITEM_REFUND_INFO_RESPONSE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8 + 4 * 3)) return;
        const uint64_t itemGuid = packet.readUInt64();
        ItemRefundInfo info;
        info.money = packet.readUInt32();
        info.honor = packet.readUInt32();
        info.arena = packet.readUInt32();
        // Five cost slots, id and count together - the same interleaving the
        // item query's sockets use, and the same trap if read as two runs.
        for (auto& slot : info.items) {
            if (!packet.hasRemaining(8)) break;
            slot.first  = packet.readUInt32();
            slot.second = packet.readUInt32();
        }
        if (packet.hasRemaining(8)) {
            packet.readUInt32();                       // always zero
            info.playedSincePurchase = packet.readUInt32();
        }
        packet.skipAll();
        itemRefundInfo_[itemGuid] = info;
        fireAddonEvent("UPDATE_INVENTORY_ALERTS", {});
    };
    // Consume silently - informational, no UI action needed
    registerSkipHandler(Opcode::SMSG_LOOT_LIST);
    // Same format as LOCKOUT_ADDED; consume
    registerSkipHandler(Opcode::SMSG_CALENDAR_RAID_LOCKOUT_UPDATED);
    // Sent one line before SMSG_DESTROY_OBJECT and carrying the same guid, and
    // only inside an arena - Object::DestroyForPlayer writes both. The unit is
    // removed by the second one whatever happens here, so this is a decision
    // rather than a gap: the real client uses it to tell an arena opponent who
    // died from one who merely went out of range, and nothing here draws that
    // distinction.
    registerSkipHandler(Opcode::SMSG_ARENA_UNIT_DESTROYED);
    // Consume - remaining server notifications not yet parsed
    for (auto op : {
        Opcode::SMSG_AFK_MONITOR_INFO_RESPONSE,
        Opcode::SMSG_AUCTION_LIST_PENDING_SALES,
        Opcode::SMSG_AVAILABLE_VOICE_CHANNEL,
        Opcode::SMSG_CALENDAR_ARENA_TEAM,
        Opcode::SMSG_CALENDAR_CLEAR_PENDING_ACTION,
        Opcode::SMSG_CALENDAR_EVENT_INVITE,
        Opcode::SMSG_CALENDAR_EVENT_INVITE_NOTES,
        Opcode::SMSG_CALENDAR_EVENT_INVITE_NOTES_ALERT,
        Opcode::SMSG_CALENDAR_EVENT_INVITE_REMOVED,
        Opcode::SMSG_CALENDAR_EVENT_INVITE_REMOVED_ALERT,
        Opcode::SMSG_CALENDAR_EVENT_INVITE_STATUS_ALERT,
        Opcode::SMSG_CALENDAR_EVENT_MODERATOR_STATUS_ALERT,
        Opcode::SMSG_CALENDAR_EVENT_REMOVED_ALERT,
        Opcode::SMSG_CALENDAR_EVENT_UPDATED_ALERT,
        Opcode::SMSG_CALENDAR_FILTER_GUILD,
        Opcode::SMSG_CHEAT_DUMP_ITEMS_DEBUG_ONLY_RESPONSE,
        Opcode::SMSG_CHEAT_DUMP_ITEMS_DEBUG_ONLY_RESPONSE_WRITE_FILE,
        Opcode::SMSG_CHEAT_PLAYER_LOOKUP,
        Opcode::SMSG_CHECK_FOR_BOTS,
        Opcode::SMSG_COMMENTATOR_GET_PLAYER_INFO,
        Opcode::SMSG_COMMENTATOR_MAP_INFO,
        Opcode::SMSG_COMMENTATOR_PLAYER_INFO,
        Opcode::SMSG_COMMENTATOR_SKIRMISH_QUEUE_RESULT1,
        Opcode::SMSG_COMMENTATOR_SKIRMISH_QUEUE_RESULT2,
        Opcode::SMSG_COMMENTATOR_STATE_CHANGED,
        Opcode::SMSG_COOLDOWN_CHEAT,
        Opcode::SMSG_DANCE_QUERY_RESPONSE,
        Opcode::SMSG_DBLOOKUP,
        Opcode::SMSG_DEBUGAURAPROC,
        Opcode::SMSG_DEBUG_AISTATE,
        Opcode::SMSG_DEBUG_LIST_TARGETS,
        Opcode::SMSG_DEBUG_SERVER_GEO,
        Opcode::SMSG_DUMP_OBJECTS_DATA,
        Opcode::SMSG_FORCEACTIONSHOW,
        Opcode::SMSG_GM_PLAYER_INFO,
        Opcode::SMSG_GODMODE,
        Opcode::SMSG_IGNORE_DIMINISHING_RETURNS_CHEAT,
        Opcode::SMSG_IGNORE_REQUIREMENTS_CHEAT,
        Opcode::SMSG_INVALIDATE_DANCE,
        Opcode::SMSG_LFG_PENDING_INVITE,
        Opcode::SMSG_LFG_PENDING_MATCH,
        Opcode::SMSG_LFG_PENDING_MATCH_DONE,
        Opcode::SMSG_LFG_UPDATE,
        Opcode::SMSG_LFG_UPDATE_LFG,
        Opcode::SMSG_LFG_UPDATE_LFM,
        Opcode::SMSG_LFG_UPDATE_QUEUED,
        Opcode::SMSG_MOVE_CHARACTER_CHEAT,
        Opcode::SMSG_NOTIFY_DANCE,
        Opcode::SMSG_NOTIFY_DEST_LOC_SPELL_CAST,
        Opcode::SMSG_PETGODMODE,
        Opcode::SMSG_PET_UPDATE_COMBO_POINTS,
        Opcode::SMSG_PLAY_DANCE,
        Opcode::SMSG_PROFILEDATA_RESPONSE,
        Opcode::SMSG_PVP_QUEUE_STATS,
        Opcode::SMSG_QUERY_OBJECT_POSITION,
        Opcode::SMSG_QUERY_OBJECT_ROTATION,
        Opcode::SMSG_REDIRECT_CLIENT,
        Opcode::SMSG_RESET_RANGED_COMBAT_TIMER,
        Opcode::SMSG_SEND_ALL_COMBAT_LOG,
        Opcode::SMSG_SET_EXTRA_AURA_INFO_NEED_UPDATE,
        Opcode::SMSG_SET_PLAYER_DECLINED_NAMES_RESULT,
        Opcode::SMSG_SET_PROJECTILE_POSITION,
        Opcode::SMSG_SPELL_CHANCE_RESIST_PUSHBACK,
        Opcode::SMSG_SPELL_UPDATE_CHAIN_TARGETS,
        Opcode::SMSG_STOP_DANCE,
        Opcode::SMSG_TEST_DROP_RATE_RESULT,
        Opcode::SMSG_UPDATE_ACCOUNT_DATA,
        Opcode::SMSG_UPDATE_ACCOUNT_DATA_COMPLETE,
        Opcode::SMSG_UPDATE_INSTANCE_OWNERSHIP,
        Opcode::SMSG_UPDATE_LAST_INSTANCE,
        Opcode::SMSG_VOICESESSION_FULL,
        Opcode::SMSG_VOICE_CHAT_STATUS,
        Opcode::SMSG_VOICE_PARENTAL_CONTROLS,
        Opcode::SMSG_VOICE_SESSION_ADJUST_PRIORITY,
        Opcode::SMSG_VOICE_SESSION_ENABLE,
        Opcode::SMSG_VOICE_SESSION_LEAVE,
        Opcode::SMSG_VOICE_SESSION_ROSTER_UPDATE,
        Opcode::SMSG_VOICE_SET_TALKER_MUTED
    }) { registerSkipHandler(op); }

}

// The domain handlers registering their own.
//
// This used to be described as overriding duplicate entries above, which has
// not been true since those were moved out. The two sets are disjoint - checked
// opcode by opcode, 182 here against 320 there, no overlap - so the order
// between them carries no meaning and a reader need not look for one.
void GameHandler::registerDomainOpcodes() {
    chatHandler_->registerOpcodes(dispatchTable_);
    movementHandler_->registerOpcodes(dispatchTable_);
    combatHandler_->registerOpcodes(dispatchTable_);
    spellHandler_->registerOpcodes(dispatchTable_);
    inventoryHandler_->registerOpcodes(dispatchTable_);
    socialHandler_->registerOpcodes(dispatchTable_);
    questHandler_->registerOpcodes(dispatchTable_);
    wardenHandler_->registerOpcodes(dispatchTable_);
}

void GameHandler::registerOpcodeHandlers() {
    registerCoreOpcodes();
    registerRemainingOpcodes();
    registerDomainOpcodes();
}

void GameHandler::handlePacket(network::Packet& packet) {
    // Do NOT drop packets with an empty body. getSize() is the payload length and the
    // opcode is carried separately, so for the many opcodes that have no payload the
    // opcode *is* the message. Dropping them here silently swallowed
    // SMSG_LOGOUT_COMPLETE - the server logged the character out and moved on while
    // the client waited forever, so the countdown ended and nothing happened.
    uint16_t opcode = packet.getOpcode();

    try {

    const bool allowVanillaAliases = isPreWotlk();

    // Vanilla compatibility aliases:
    // - 0x006B: can be SMSG_COMPRESSED_MOVES on some vanilla-family servers
    //           and SMSG_WEATHER on others
    // - 0x0103: SMSG_PLAY_MUSIC (some vanilla-family servers)
    //
    // We gate these by payload shape so expansion-native mappings remain intact.
    if (allowVanillaAliases && opcode == 0x006B) {
        // Try compressed movement batch first:
        // [u8 subSize][u16 subOpcode][subPayload...] ...
        // where subOpcode is typically SMSG_MONSTER_MOVE / SMSG_MONSTER_MOVE_TRANSPORT.
        const auto& data = packet.getData();
        if (packet.getReadPos() + 3 <= data.size()) {
            size_t pos = packet.getReadPos();
            uint8_t subSize = data[pos];
            if (subSize >= 2 && pos + 1 + subSize <= data.size()) {
                uint16_t subOpcode = static_cast<uint16_t>(data[pos + 1]) |
                                     (static_cast<uint16_t>(data[pos + 2]) << 8);
                uint16_t monsterMoveWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE);
                uint16_t monsterMoveTransportWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE_TRANSPORT);
                if ((monsterMoveWire != 0xFFFF && subOpcode == monsterMoveWire) ||
                    (monsterMoveTransportWire != 0xFFFF && subOpcode == monsterMoveTransportWire)) {
                    LOG_INFO("Opcode 0x006B interpreted as SMSG_COMPRESSED_MOVES (subOpcode=0x",
                             std::hex, subOpcode, std::dec, ")");
                    if (movementHandler_) movementHandler_->handleCompressedMoves(packet);
                    return;
                }
            }
        }

        // Expected weather payload: uint32 weatherType, float intensity, uint8 abrupt
        if (packet.hasRemaining(9)) {
            uint32_t wType = packet.readUInt32();
            float wIntensity = packet.readFloat();
            uint8_t abrupt = packet.readUInt8();
            bool plausibleWeather =
                (wType <= 3) &&
                std::isfinite(wIntensity) &&
                (wIntensity >= 0.0f && wIntensity <= 1.5f) &&
                (abrupt <= 1);
            if (plausibleWeather) {
                weatherType_ = wType;
                weatherIntensity_ = wIntensity;
                const char* typeName =
                    (wType == 1) ? "Rain" :
                    (wType == 2) ? "Snow" :
                    (wType == 3) ? "Storm" : "Clear";
                LOG_INFO("Weather changed (0x006B alias): type=", wType,
                         " (", typeName, "), intensity=", wIntensity,
                         ", abrupt=", static_cast<int>(abrupt));
                return;
            }
            // Not weather-shaped: rewind and fall through to normal opcode table handling.
            packet.setReadPos(0);
        }
    } else if (allowVanillaAliases && opcode == 0x0103) {
        // Expected play-music payload: uint32 sound/music id
        if (packet.getRemainingSize() == 4) {
            uint32_t soundId = packet.readUInt32();
            LOG_INFO("SMSG_PLAY_MUSIC (0x0103 alias): soundId=", soundId);
            if (playMusicCallback_) playMusicCallback_(soundId);
            return;
        }
    }

    auto preLogicalOp = opcodeTable_.fromWire(opcode);
    if (preLogicalOp && isAuthCharPipelineOpcode(*preLogicalOp)) {
        LOG_DEBUG("AUTH/CHAR RX opcode=0x", std::hex, opcode, std::dec,
                 " logical=", static_cast<uint32_t>(*preLogicalOp),
                 " state=", worldStateName(state),
                 " size=", packet.getSize());
    }

    LOG_DEBUG("Received world packet: opcode=0x", std::hex, opcode, std::dec,
              " size=", packet.getSize(), " bytes");

    auto logicalOp = preLogicalOp;

    if (!logicalOp) {
        // Login-critical opcodes share the same wire values across all expansions.
        // Fall back to hardcoded mapping so the auth/char pipeline works even when
        // the expansion opcode table failed to load (wrong CWD, missing Data/, etc.).
        switch (opcode) {
            case 0x1EC: logicalOp = Opcode::SMSG_AUTH_CHALLENGE; break;
            case 0x1EE: logicalOp = Opcode::SMSG_AUTH_RESPONSE;  break;
            case 0x03B: logicalOp = Opcode::SMSG_CHAR_ENUM;      break;
            case 0x03A: logicalOp = Opcode::SMSG_CHAR_CREATE;     break;
            case 0x03C: logicalOp = Opcode::SMSG_CHAR_DELETE;     break;
            case 0x2E6: logicalOp = Opcode::SMSG_WARDEN_DATA;     break;
            default: break;
        }
        if (logicalOp) {
            static bool loggedFallback = false;
            if (!loggedFallback) {
                loggedFallback = true;
                LOG_WARNING("Opcode table lookup failed for login-critical opcode 0x",
                            std::hex, opcode, std::dec,
                            " (table has ", opcodeTable_.size(), " entries). "
                            "Using hardcoded fallback. Check that Data/expansions/<id>/opcodes.json is loadable.");
            }
        } else {
            static std::unordered_set<uint16_t> loggedUnknownWireOpcodes;
            if (loggedUnknownWireOpcodes.insert(opcode).second) {
                LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec,
                            " state=", static_cast<int>(state),
                            " size=", packet.getSize());
            }
            return;
        }
    }

    // Dispatch via the opcode handler table
    auto it = dispatchTable_.find(*logicalOp);
    if (it != dispatchTable_.end()) {
        it->second(packet);
    } else {
        // In pre-world states we need full visibility (char create/login handshakes).
        // In-world we keep de-duplication to avoid heavy log I/O in busy areas.
        if (state != WorldState::IN_WORLD) {
            static std::unordered_set<uint32_t> loggedUnhandledByState;
            const uint32_t key = (static_cast<uint32_t>(static_cast<uint8_t>(state)) << 16) |
                                 static_cast<uint32_t>(opcode);
            if (loggedUnhandledByState.insert(key).second) {
                LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec,
                            " state=", static_cast<int>(state),
                            " size=", packet.getSize());
                const auto& data = packet.getData();
                std::string hex;
                size_t limit = std::min<size_t>(data.size(), 48);
                hex.reserve(limit * 3);
                for (size_t i = 0; i < limit; ++i) {
                    char b[4];
                    snprintf(b, sizeof(b), "%02x ", data[i]);
                    hex += b;
                }
                LOG_INFO("Unhandled opcode payload hex (first ", limit, " bytes): ", hex);
            }
        } else {
            static std::unordered_set<uint16_t> loggedUnhandledOpcodes;
            if (loggedUnhandledOpcodes.insert(static_cast<uint16_t>(opcode)).second) {
                LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec);
            }
        }
    }
    } catch (const std::bad_alloc& e) {
        LOG_ERROR("OOM while handling world opcode=0x", std::hex, opcode, std::dec,
                  " state=", worldStateName(state),
                  " size=", packet.getSize(),
                  " readPos=", packet.getReadPos(),
                  " what=", e.what());
        if (socket && state == WorldState::IN_WORLD) {
            disconnect();
            fail("Out of memory while parsing world packet");
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Exception while handling world opcode=0x", std::hex, opcode, std::dec,
                  " state=", worldStateName(state),
                  " size=", packet.getSize(),
                  " readPos=", packet.getReadPos(),
                  " what=", e.what());
    }
}

void GameHandler::enqueueIncomingPacket(const network::Packet& packet) {
    if (pendingIncomingPackets_.size() >= kMaxQueuedInboundPackets) {
        LOG_ERROR("Inbound packet queue overflow (", pendingIncomingPackets_.size(),
                  " packets); dropping oldest packet to preserve responsiveness");
        pendingIncomingPackets_.pop_front();
    }
    pendingIncomingPackets_.push_back(packet);
    lastRxTime_ = std::chrono::steady_clock::now();
    rxSilenceLogged_ = false;
    rxSilence15sLogged_ = false;
}

void GameHandler::enqueueIncomingPacketFront(network::Packet&& packet) {
    if (pendingIncomingPackets_.size() >= kMaxQueuedInboundPackets) {
        LOG_ERROR("Inbound packet queue overflow while prepending (", pendingIncomingPackets_.size(),
                  " packets); dropping newest queued packet to preserve ordering");
        pendingIncomingPackets_.pop_back();
    }
    pendingIncomingPackets_.emplace_front(std::move(packet));
}

// enqueueUpdateObjectWork and processPendingUpdateObjectWork moved to EntityController

void GameHandler::processQueuedIncomingPackets() {
    if (pendingIncomingPackets_.empty() && !entityController_->hasPendingUpdateObjectWork()) {
        return;
    }

    const int maxPacketsThisUpdate = incomingPacketsBudgetPerUpdate(state);
    const float budgetMs = incomingPacketBudgetMs(state);
    const auto start = std::chrono::steady_clock::now();
    int processed = 0;

    while (processed < maxPacketsThisUpdate) {
        float elapsedMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsedMs >= budgetMs) {
            break;
        }

        if (entityController_->hasPendingUpdateObjectWork()) {
            entityController_->processPendingUpdateObjectWork(start, budgetMs);
            if (entityController_->hasPendingUpdateObjectWork()) {
                break;
            }
            continue;
        }

        if (pendingIncomingPackets_.empty()) {
            break;
        }

        network::Packet packet = std::move(pendingIncomingPackets_.front());
        pendingIncomingPackets_.pop_front();
        const uint16_t wireOp = packet.getOpcode();
        const auto logicalOp = opcodeTable_.fromWire(wireOp);
        auto packetHandleStart = std::chrono::steady_clock::now();
        handlePacket(packet);
        float packetMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - packetHandleStart).count();
        if (packetMs > slowPacketLogThresholdMs()) {
            const char* logicalName = logicalOp
                ? OpcodeTable::logicalToName(*logicalOp)
                : "UNKNOWN";
            LOG_WARNING("SLOW packet handler: ", packetMs,
                        "ms wire=0x", std::hex, wireOp, std::dec,
                        " logical=", logicalName,
                        " size=", packet.getSize(),
                        " state=", worldStateName(state));
        }
        ++processed;
    }

    if (entityController_->hasPendingUpdateObjectWork()) {
        return;
    }

    if (!pendingIncomingPackets_.empty()) {
        LOG_DEBUG("GameHandler packet budget reached (processed=", processed,
                  ", remaining=", pendingIncomingPackets_.size(),
                  ", state=", worldStateName(state), ")");
    }
}


} // namespace game
} // namespace wowee
