#include "game/game_handler.hpp"
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
#include "game/packet_parsers.hpp"
#include "game/transport_manager.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
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

const char* worldStateName(WorldState state) {
    switch (state) {
        case WorldState::DISCONNECTED: return "DISCONNECTED";
        case WorldState::CONNECTING: return "CONNECTING";
        case WorldState::CONNECTED: return "CONNECTED";
        case WorldState::CHALLENGE_RECEIVED: return "CHALLENGE_RECEIVED";
        case WorldState::AUTH_SENT: return "AUTH_SENT";
        case WorldState::AUTHENTICATED: return "AUTHENTICATED";
        case WorldState::READY: return "READY";
        case WorldState::CHAR_LIST_REQUESTED: return "CHAR_LIST_REQUESTED";
        case WorldState::CHAR_LIST_RECEIVED: return "CHAR_LIST_RECEIVED";
        case WorldState::ENTERING_WORLD: return "ENTERING_WORLD";
        case WorldState::IN_WORLD: return "IN_WORLD";
        case WorldState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

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

int parseEnvIntClamped(const char* key, int defaultValue, int minValue, int maxValue) {
    const char* raw = std::getenv(key);
    if (!raw || !*raw) return defaultValue;
    char* end = nullptr;
    long parsed = std::strtol(raw, &end, 10);
    if (end == raw) return defaultValue;
    return static_cast<int>(std::clamp<long>(parsed, minValue, maxValue));
}

int incomingPacketsBudgetPerUpdate(WorldState state) {
    static const int inWorldBudget =
        parseEnvIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKETS", 24, 1, 512);
    static const int loginBudget =
        parseEnvIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKETS_LOGIN", 96, 1, 512);
    return state == WorldState::IN_WORLD ? inWorldBudget : loginBudget;
}

float incomingPacketBudgetMs(WorldState state) {
    static const int inWorldBudgetMs =
        parseEnvIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKET_MS", 2, 1, 50);
    static const int loginBudgetMs =
        parseEnvIntClamped("WOWEE_NET_MAX_GAMEHANDLER_PACKET_MS_LOGIN", 8, 1, 50);
    return static_cast<float>(state == WorldState::IN_WORLD ? inWorldBudgetMs : loginBudgetMs);
}

float slowPacketLogThresholdMs() {
    static const int thresholdMs =
        parseEnvIntClamped("WOWEE_NET_SLOW_PACKET_LOG_MS", 10, 1, 60000);
    return static_cast<float>(thresholdMs);
}

constexpr size_t kMaxQueuedInboundPackets = 4096;

} // end anonymous namespace

void GameHandler::registerOpcodeHandlers() {
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
                addSystemChatMessage(msg);
                addCombatText(CombatTextEntry::XP_GAIN, static_cast<int32_t>(xpGained), 0, true);
                if (areaDiscoveryCallback_) areaDiscoveryCallback_(areaName, xpGained);
                                    fireAddonEvent("CHAT_MSG_COMBAT_XP_GAIN", {msg, std::to_string(xpGained)});
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
    dispatchTable_[Opcode::SMSG_WORLD_STATE_UI_TIMER_UPDATE] = [](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t serverTime = packet.readUInt32();
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
                            fireAddonEvent("MIRROR_TIMER_START", {
                    std::to_string(type), std::to_string(value),
                    std::to_string(maxV), std::to_string(scale),
                    paused ? "1" : "0"});
        }
    };
    dispatchTable_[Opcode::SMSG_STOP_MIRROR_TIMER] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t type = packet.readUInt32();
        if (type < 3) {
            mirrorTimers_[type].active = false;
            mirrorTimers_[type].value  = 0;
            fireAddonEvent("MIRROR_TIMER_STOP", {std::to_string(type)});
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
        addSystemChatMessage(msg);
        LOG_INFO("SMSG_TITLE_EARNED: bit=", titleBit, " lost=", isLost, " title='", titleStr, "'");
    };
    dispatchTable_[Opcode::SMSG_LEARNED_DANCE_MOVES] = [](network::Packet& packet) {
        LOG_DEBUG("SMSG_LEARNED_DANCE_MOVES: ignored (size=", packet.getSize(), ")");
    };
    dispatchTable_[Opcode::SMSG_CHAR_RENAME] = [this](network::Packet& packet) {
        if (packet.hasRemaining(13)) {
            uint32_t result = packet.readUInt32();
            /*uint64_t guid =*/ packet.readUInt64();
            std::string newName = packet.readString();
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
            LOG_INFO("SMSG_CHAR_RENAME: result=", result, " newName=", newName);
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
    registerSkipHandler(Opcode::SMSG_BINDER_CONFIRM);
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
            else { addUIError("You are too far from the innkeeper."); addSystemChatMessage("You are too far from the innkeeper."); }
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
    // Consume silently — opcodes we receive but don't need to act on
    for (auto op : {
        Opcode::SMSG_FLIGHT_SPLINE_SYNC, Opcode::SMSG_FORCE_DISPLAY_UPDATE,
        Opcode::SMSG_FORCE_SEND_QUEUED_PACKETS, Opcode::SMSG_FORCE_SET_VEHICLE_REC_ID,
        Opcode::SMSG_CORPSE_MAP_POSITION_QUERY_RESPONSE, Opcode::SMSG_DAMAGE_CALC_LOG,
        Opcode::SMSG_DYNAMIC_DROP_ROLL_RESULT, Opcode::SMSG_DESTRUCTIBLE_BUILDING_DAMAGE,
    }) { registerSkipHandler(op); }

    // Game object despawn animation — reset state to closed before actual despawn
    dispatchTable_[Opcode::SMSG_GAMEOBJECT_DESPAWN_ANIM] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint64_t guid = packet.readUInt64();
        // Trigger a CLOSE animation / freeze before the object is removed
        if (gameObjectStateCallback_) gameObjectStateCallback_(guid, 0);
    };
    // Game object reset state — return to READY(closed) state
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
    // SMSG_DEFENSE_MESSAGE — moved to ChatHandler::registerOpcodes
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
    dispatchTable_[Opcode::SMSG_DEATH_RELEASE_LOC] = [](network::Packet& packet) {
        if (packet.hasRemaining(16)) {
            uint32_t relMapId = packet.readUInt32();
            float relX = packet.readFloat(), relY = packet.readFloat(), relZ = packet.readFloat();
            LOG_INFO("SMSG_DEATH_RELEASE_LOC (graveyard spawn): map=", relMapId, " x=", relX, " y=", relY, " z=", relZ);
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
        if (!packet.hasRemaining(1)) return;
        uint8_t found = packet.readUInt8();
        if (found && packet.hasRemaining(20)) {
            /*uint32_t mapId =*/ packet.readUInt32();
            float cx = packet.readFloat();
            float cy = packet.readFloat();
            float cz = packet.readFloat();
            uint32_t corpseMapId = packet.readUInt32();
            corpseX_ = cx;
            corpseY_ = cy;
            corpseZ_ = cz;
            corpseMapId_ = corpseMapId;
            corpsePositionValid_ = true;
            LOG_INFO("MSG_CORPSE_QUERY: corpse at (", cx, ",", cy, ",", cz, ") map=", corpseMapId);
        }
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
        if (packet.hasRemaining(8)) {
            uint64_t bGuid = packet.readUInt64();
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
            addSystemChatMessage("Cannot dismount here.");
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
        }
    };
    dispatchTable_[Opcode::SMSG_RESURRECT_REQUEST] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint64_t casterGuid = packet.readUInt64();
        std::string casterName;
        if (packet.hasData())
            casterName = packet.readString();
        if (casterGuid) {
            resurrectCasterGuid_ = casterGuid;
            resurrectIsSpiritHealer_ = false;
            if (!casterName.empty()) {
                resurrectCasterName_ = casterName;
            } else {
                resurrectCasterName_ = lookupName(casterGuid);
            }
            resurrectRequestPending_ = true;
                            fireAddonEvent("RESURRECT_REQUEST", {resurrectCasterName_});
        }
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
    };
    dispatchTable_[Opcode::SMSG_ZONE_UNDER_ATTACK] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t areaId = packet.readUInt32();
            std::string areaName = getAreaName(areaId);
            std::string msg = areaName.empty()
                ? std::string("A zone is under attack!")
                : (areaName + " is under attack!");
            addUIError(msg);
            addSystemChatMessage(msg);
        }
    };

    // Spirit healer time / durability
    dispatchTable_[Opcode::SMSG_AREA_SPIRIT_HEALER_TIME] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            /*uint64_t guid =*/ packet.readUInt64();
            uint32_t timeMs = packet.readUInt32();
            uint32_t secs = timeMs / 1000;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "You will be able to resurrect in %u seconds.", secs);
            addSystemChatMessage(buf);
        }
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

    // SMSG_SERVER_MESSAGE — moved to ChatHandler::registerOpcodes
    // SMSG_CHAT_SERVER_MESSAGE — moved to ChatHandler::registerOpcodes
    // SMSG_AREA_TRIGGER_MESSAGE — moved to ChatHandler::registerOpcodes
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
        uint32_t mapId = packet.readUInt32();
        uint8_t reason = (packet.hasData()) ? packet.readUInt8() : 0;
        (void)mapId;
        const char* abortMsg = nullptr;
        switch (reason) {
            case 0x01: abortMsg = "Transfer aborted: difficulty unavailable."; break;
            case 0x02: abortMsg = "Transfer aborted: expansion required."; break;
            case 0x03: abortMsg = "Transfer aborted: instance not found."; break;
            case 0x04: abortMsg = "Transfer aborted: too many instances. Please wait before entering a new instance."; break;
            case 0x06: abortMsg = "Transfer aborted: instance is full."; break;
            case 0x07: abortMsg = "Transfer aborted: zone is in combat."; break;
            case 0x08: abortMsg = "Transfer aborted: you are already in this instance."; break;
            case 0x09: abortMsg = "Transfer aborted: not enough players."; break;
            default:   abortMsg = "Transfer aborted."; break;
        }
        addUIError(abortMsg);
        addSystemChatMessage(abortMsg);
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
        if (packet.hasRemaining(4)) {
            uint32_t result = packet.readUInt32();
            if (result == 0) addSystemChatMessage("Gems socketed successfully.");
            else addSystemChatMessage("Failed to socket gems.");
        }
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
    dispatchTable_[Opcode::SMSG_FISH_NOT_HOOKED] = [this](network::Packet& /*packet*/) {
        hookedFishingBobberGuid_ = 0;
        addSystemChatMessage("Your fish got away.");
    };
    dispatchTable_[Opcode::SMSG_FISH_ESCAPED] = [this](network::Packet& /*packet*/) {
        hookedFishingBobberGuid_ = 0;
        addSystemChatMessage("Your fish escaped!");
    };

    // ---- Auto-repeat / auras / dispel / totem ----
    dispatchTable_[Opcode::SMSG_CANCEL_AUTO_REPEAT] = [](network::Packet& /*packet*/) {
        // Server signals to stop a repeating spell (wand/shoot); no client action needed
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
                // Empty slot — only clear if not already set to Attack/Hearthstone defaults
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
                default:   continue;  // unknown — leave as-is
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
        fireAddonEvent("ACTIONBAR_SLOT_CHANGED", {});
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
                            break;  // was 'return' — must NOT exit here or level-up notification is skipped
                        }
                    }
                    if (newLevel > oldLevel) {
                        addSystemChatMessage("You have reached level " + std::to_string(newLevel) + "!");
                        withSoundManager(&audio::AudioCoordinator::getUiSoundManager, [](auto* sfx) { sfx->playLevelUp(); });
                        if (levelUpCallback_) levelUpCallback_(newLevel);
                        fireAddonEvent("PLAYER_LEVEL_UP", {std::to_string(newLevel)});
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
        // uint32 criteriaId + uint64 progress + uint32 elapsedTime + uint32 creationTime
        if (packet.hasRemaining(20)) {
            uint32_t criteriaId    = packet.readUInt32();
            uint64_t progress      = packet.readUInt64();
            packet.readUInt32(); // elapsedTime
            packet.readUInt32(); // creationTime
            uint64_t oldProgress = 0;
            auto cpit = criteriaProgress_.find(criteriaId);
            if (cpit != criteriaProgress_.end()) oldProgress = cpit->second;
            criteriaProgress_[criteriaId] = progress;
            LOG_DEBUG("SMSG_CRITERIA_UPDATE: id=", criteriaId, " progress=", progress);
            // Fire addon event for achievement tracking addons
            if (progress != oldProgress)
                fireAddonEvent("CRITERIA_UPDATE", {std::to_string(criteriaId), std::to_string(progress)});
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
    // Server-script text message — display in system chat
    dispatchTable_[Opcode::SMSG_SCRIPT_MESSAGE] = [this](network::Packet& packet) {
        // Server-script text message — display in system chat
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
    // uint32 auctionId + uint32 itemEntry + uint32 itemRandom — auction expired/cancelled
    // uint64 containerGuid — tells client to open this container
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
        if (wasInVehicle != nowInVehicle && vehicleStateCallback_) {
            vehicleStateCallback_(nowInVehicle, newVehicleId);
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
    // SMSG_GUILD_DECLINE — moved to SocialHandler::registerOpcodes
    // Clear cached talent data so the talent screen reflects the reset.
    dispatchTable_[Opcode::SMSG_TALENTS_INVOLUNTARILY_RESET] = [this](network::Packet& packet) {
        // Clear cached talent data so the talent screen reflects the reset.
        if (spellHandler_) spellHandler_->resetTalentState();
        addUIError("Your talents have been reset by the server.");
        addSystemChatMessage("Your talents have been reset by the server.");
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_SET_REST_START] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t restTrigger = packet.readUInt32();
            const bool nowResting = (restTrigger > 0);
            // The server pushes SMSG_SET_REST_START periodically while in
            // a rest area, so only emit chat / fire the addon event on
            // actual transitions — otherwise "You are now resting." spams
            // the chat log every tick.
            if (nowResting != isResting_) {
                isResting_ = nowResting;
                addSystemChatMessage(isResting_ ? "You are now resting."
                                                : "You are no longer resting.");
                fireAddonEvent("PLAYER_UPDATE_RESTING", {});
            }
        }
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
    dispatchTable_[Opcode::SMSG_MOUNTSPECIAL_ANIM] = [](network::Packet& packet) { (void)packet.readPackedGuid(); };
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
    // uint32 movieId — we don't play movies; acknowledge immediately.
    dispatchTable_[Opcode::SMSG_TRIGGER_MOVIE] = [this](network::Packet& packet) {
        // uint32 movieId — we don't play movies; acknowledge immediately.
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
    // uint32 result — LFG auto-join attempt failed (player selected auto-join at queue time)
    dispatchTable_[Opcode::SMSG_LFG_AUTOJOIN_FAILED] = [this](network::Packet& packet) {
        // uint32 result — LFG auto-join attempt failed (player selected auto-join at queue time)
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
    // uint32 zoneId + uint8 level_min + uint8 level_max — player queued for meeting stone
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_SETQUEUE] = [this](network::Packet& packet) {
        // uint32 zoneId + uint8 level_min + uint8 level_max — player queued for meeting stone
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
    // uint64 memberGuid — a player was added to your group via meeting stone
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_MEMBER_ADDED] = [this](network::Packet& packet) {
        // uint64 memberGuid — a player was added to your group via meeting stone
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
    // uint8 reason — failed to join group via meeting stone
    // 0=target_not_in_lfg, 1=target_in_party, 2=target_invalid_map, 3=target_not_available
    dispatchTable_[Opcode::SMSG_MEETINGSTONE_JOINFAILED] = [this](network::Packet& packet) {
        // uint8 reason — failed to join group via meeting stone
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
    dispatchTable_[Opcode::SMSG_GMTICKET_CREATE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t res = packet.readUInt8();
            addSystemChatMessage(res == 1 ? "GM ticket submitted."
                                          : "Failed to submit GM ticket.");
        }
    };
    dispatchTable_[Opcode::SMSG_GMTICKET_UPDATETEXT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t res = packet.readUInt8();
            addSystemChatMessage(res == 1 ? "GM ticket updated."
                                          : "Failed to update GM ticket.");
        }
    };
    dispatchTable_[Opcode::SMSG_GMTICKET_DELETETICKET] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t res = packet.readUInt8();
            addSystemChatMessage(res == 9 ? "GM ticket deleted."
                                          : "No ticket to delete.");
        }
    };
    // WotLK 3.3.5a format:
    //   uint8  status  — 1=no ticket, 6=has open ticket, 3=closed, 10=suspended
    // If status == 6 (GMTICKET_STATUS_HASTEXT):
    //   cstring ticketText
    //   uint32  ticketAge       (seconds old)
    //   uint32  daysUntilOld    (days remaining before escalation)
    //   float   waitTimeHours   (estimated GM wait time)
    dispatchTable_[Opcode::SMSG_GMTICKET_GETTICKET] = [this](network::Packet& packet) {
        // WotLK 3.3.5a format:
        //   uint8  status  — 1=no ticket, 6=has open ticket, 3=closed, 10=suspended
        // If status == 6 (GMTICKET_STATUS_HASTEXT):
        //   cstring ticketText
        //   uint32  ticketAge       (seconds old)
        //   uint32  daysUntilOld    (days remaining before escalation)
        //   float   waitTimeHours   (estimated GM wait time)
        if (!packet.hasRemaining(1)) { packet.skipAll(); return; }
        uint8_t gmStatus = packet.readUInt8();
        // Status 6 = GMTICKET_STATUS_HASTEXT — open ticket with text
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
        } else if (gmStatus == 3) {
            gmTicketActive_ = false;
            gmTicketText_.clear();
            addSystemChatMessage("Your GM ticket has been closed.");
            LOG_INFO("SMSG_GMTICKET_GETTICKET: ticket closed");
        } else if (gmStatus == 10) {
            gmTicketActive_ = false;
            gmTicketText_.clear();
            addSystemChatMessage("Your GM ticket has been suspended.");
            LOG_INFO("SMSG_GMTICKET_GETTICKET: ticket suspended");
        } else {
            // Status 1 = no open ticket (default/no ticket)
            gmTicketActive_ = false;
            gmTicketText_.clear();
            LOG_DEBUG("SMSG_GMTICKET_GETTICKET: no open ticket (status=", static_cast<int>(gmStatus), ")");
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
        if (idx < 6) playerRunes_[idx].type = static_cast<RuneType>(type & 0x3);
    };
    // uint8 runeReadyMask (bit i=1 → rune i is ready)
    // uint8[6] cooldowns (0=ready, 255=just used → readyFraction = 1 - val/255)
    dispatchTable_[Opcode::SMSG_RESYNC_RUNES] = [this](network::Packet& packet) {
        // uint8 runeReadyMask (bit i=1 → rune i is ready)
        // uint8[6] cooldowns (0=ready, 255=just used → readyFraction = 1 - val/255)
        if (!packet.hasRemaining(7)) {
            packet.skipAll();
            return;
        }
        uint8_t readyMask = packet.readUInt8();
        for (int i = 0; i < 6; i++) {
            uint8_t cd = packet.readUInt8();
            playerRunes_[i].ready = (readyMask & (1u << i)) != 0;
            playerRunes_[i].readyFraction = 1.0f - cd / 255.0f;
            if (playerRunes_[i].ready) playerRunes_[i].readyFraction = 1.0f;
        }
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
        // uint32 slot + packed_guid unit (0 packed = clear slot)
        if (!packet.hasRemaining(5)) {
            packet.skipAll();
            return;
        }
        uint32_t slot = packet.readUInt32();
        uint64_t unit = packet.readPackedGuid();
        if (socialHandler_) {
            socialHandler_->setEncounterUnitGuid(slot, unit);
            LOG_DEBUG("SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT: slot=", slot,
                      " guid=0x", std::hex, unit, std::dec);
        }
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
    dispatchTable_[Opcode::SMSG_RAID_GROUP_ONLY] = [this](network::Packet& packet) {
        addUIError("You must be in a raid group to enter this instance.");
        addSystemChatMessage("You must be in a raid group to enter this instance.");
        packet.skipAll();
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
        addSystemChatMessage("Cannot reset instance: another player is still inside.");
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
            LOG_DEBUG("SMSG_REALM_SPLIT splitType=", splitType, " — sent CMSG_REALM_SPLIT ack");
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
    // SMSG_READ_ITEM_OK — moved to InventoryHandler::registerOpcodes
    // SMSG_READ_ITEM_FAILED — moved to InventoryHandler::registerOpcodes
    // SMSG_QUERY_QUESTS_COMPLETED_RESPONSE — moved to QuestHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_NPC_WONT_TALK] = [this](network::Packet& packet) {
        addUIError("That creature can't talk to you right now.");
        addSystemChatMessage("That creature can't talk to you right now.");
        packet.skipAll();
    };

    // (SMSG_PET_UNLEARN_CONFIRM → registered by SpellHandler, which owns the
    // pending-unlearn state the confirm dialog reads. Handling it here wrote
    // to dead duplicate members and the dialog never appeared.)
    // These pet opcodes have incompatible formats — just consume the packet.
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
        packet.skipAll();
    };
    dispatchTable_[Opcode::SMSG_PET_NAME_INVALID] = [this](network::Packet& packet) {
        addUIError("That pet name is invalid. Please choose a different name.");
        addSystemChatMessage("That pet name is invalid. Please choose a different name.");
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
        for (auto it = subPackets.rbegin(); it != subPackets.rend(); ++it) {
            enqueueIncomingPacketFront(std::move(*it));
        }
        packet.skipAll();
    };
    // Recruit-A-Friend: a mentor is offering to grant you a level
    dispatchTable_[Opcode::SMSG_PROPOSE_LEVEL_GRANT] = [this](network::Packet& packet) {
        // Recruit-A-Friend: a mentor is offering to grant you a level
        if (packet.hasRemaining(8)) {
            uint64_t mentorGuid = packet.readUInt64();
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
    // SMSG_REFER_A_FRIEND_EXPIRED — moved to SocialHandler::registerOpcodes
    // SMSG_REFER_A_FRIEND_FAILURE — moved to SocialHandler::registerOpcodes
    // SMSG_REPORT_PVP_AFK_RESULT — moved to SocialHandler::registerOpcodes
    dispatchTable_[Opcode::SMSG_RESPOND_INSPECT_ACHIEVEMENTS] = [this](network::Packet& packet) {
        loadAchievementNameCache();
        if (!packet.hasRemaining(1)) return;
        uint64_t inspectedGuid = packet.readPackedGuid();
        if (inspectedGuid == 0) { packet.skipAll(); return; }
        std::unordered_set<uint32_t> achievements;
        while (packet.hasRemaining(4)) {
            uint32_t id = packet.readUInt32();
            if (id == 0xFFFFFFFF) break;
            if (!packet.hasRemaining(4)) break;
            /*date*/ packet.readUInt32();
            achievements.insert(id);
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
    };
    dispatchTable_[Opcode::SMSG_ON_CANCEL_EXPECTED_RIDE_VEHICLE_AURA] = [this](network::Packet& packet) {
        vehicleId_ = 0;  // Vehicle ride cancelled; clear UI
        if (vehicleStateCallback_) {
            vehicleStateCallback_(false, 0);
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
    //   uint64 mirrorGuid — GUID of the mirror image unit
    //   uint32 displayId  — display ID to render the image with
    //   uint8  raceId     — race of caster
    //   uint8  genderFlag — gender of caster
    //   uint8  classId    — class of caster
    //   uint64 casterGuid — GUID of the player who cast the spell
    //   Followed by equipped item display IDs (11 × uint32) if casterGuid != 0
    // Purpose: tells client how to render the image (same appearance as caster).
    // We parse the GUIDs so units render correctly via their existing display IDs.
    dispatchTable_[Opcode::SMSG_MIRRORIMAGE_DATA] = [this](network::Packet& packet) {
        // WotLK 3.3.5a format:
        //   uint64 mirrorGuid — GUID of the mirror image unit
        //   uint32 displayId  — display ID to render the image with
        //   uint8  raceId     — race of caster
        //   uint8  genderFlag — gender of caster
        //   uint8  classId    — class of caster
        //   uint64 casterGuid — GUID of the player who cast the spell
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
        // uint64 battlefieldGuid + uint32 zoneId + uint64 expireUnixTime (seconds)
        if (!packet.hasRemaining(20)) {
            packet.skipAll(); return;
        }
        uint64_t bfGuid    = packet.readUInt64();
        uint32_t bfZoneId  = packet.readUInt32();
        uint64_t expireTime = packet.readUInt64();
        (void)bfGuid; (void)expireTime;
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
        LOG_INFO("SMSG_BATTLEFIELD_MGR_ENTRY_INVITE: zoneId=", bfZoneId);
    };
    // uint64 battlefieldGuid + uint8 isSafe (1=pvp zones enabled) + uint8 onQueue
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_ENTERED] = [this](network::Packet& packet) {
        // uint64 battlefieldGuid + uint8 isSafe (1=pvp zones enabled) + uint8 onQueue
        if (packet.hasRemaining(8)) {
            uint64_t bfGuid2 = packet.readUInt64();
            (void)bfGuid2;
            uint8_t isSafe  = (packet.hasRemaining(1)) ? packet.readUInt8() : 0;
            uint8_t onQueue = (packet.hasRemaining(1)) ? packet.readUInt8() : 0;
            bfMgrInvitePending_ = false;
            bfMgrActive_        = true;
            addSystemChatMessage(isSafe ? "You are in the battlefield zone (safe area)."
                                        : "You have entered the battlefield!");
            if (onQueue) addSystemChatMessage("You are in the battlefield queue.");
            LOG_INFO("SMSG_BATTLEFIELD_MGR_ENTERED: isSafe=", static_cast<int>(isSafe), " onQueue=", static_cast<int>(onQueue));
        }
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint32 battlefieldId + uint64 expireTime
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_QUEUE_INVITE] = [this](network::Packet& packet) {
        // uint64 battlefieldGuid + uint32 battlefieldId + uint64 expireTime
        if (!packet.hasRemaining(20)) {
            packet.skipAll(); return;
        }
        uint64_t bfGuid3   = packet.readUInt64();
        uint32_t bfId      = packet.readUInt32();
        uint64_t expTime   = packet.readUInt64();
        (void)bfGuid3; (void)expTime;
        bfMgrInvitePending_ = true;
        bfMgrZoneId_        = bfId;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "A spot has opened in the battlefield queue (battlefield %u).", bfId);
        addSystemChatMessage(buf);
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
        LOG_INFO("SMSG_BATTLEFIELD_MGR_QUEUE_REQUEST_RESPONSE: accepted=", static_cast<int>(accepted),
                 " result=", static_cast<int>(result));
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint8 remove
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_EJECT_PENDING] = [this](network::Packet& packet) {
        // uint64 battlefieldGuid + uint8 remove
        if (packet.hasRemaining(9)) {
            uint64_t bfGuid4 = packet.readUInt64();
            uint8_t  remove  = packet.readUInt8();
            (void)bfGuid4;
            if (remove) {
                addSystemChatMessage("You will be removed from the battlefield shortly.");
            }
            LOG_INFO("SMSG_BATTLEFIELD_MGR_EJECT_PENDING: remove=", static_cast<int>(remove));
        }
        packet.skipAll();
    };
    // uint64 battlefieldGuid + uint32 reason + uint32 battleStatus + uint8 relocated
    dispatchTable_[Opcode::SMSG_BATTLEFIELD_MGR_EJECTED] = [this](network::Packet& packet) {
        // uint64 battlefieldGuid + uint32 reason + uint32 battleStatus + uint8 relocated
        if (packet.hasRemaining(17)) {
            uint64_t bfGuid5    = packet.readUInt64();
            uint32_t reason     = packet.readUInt32();
            /*uint32_t status  =*/ packet.readUInt32();
            uint8_t relocated   = packet.readUInt8();
            (void)bfGuid5;
            static const char* kEjectReasons[] = {
                "Removed from battlefield.", "Transported from battlefield.",
                "Left battlefield voluntarily.", "Offline.",
            };
            const char* msg = (reason < 4) ? kEjectReasons[reason]
                                           : "You have been ejected from the battlefield.";
            addSystemChatMessage(msg);
            if (relocated) addSystemChatMessage("You have been relocated outside the battlefield.");
            LOG_INFO("SMSG_BATTLEFIELD_MGR_EJECTED: reason=", reason, " relocated=", static_cast<int>(relocated));
        }
        bfMgrActive_        = false;
        bfMgrInvitePending_ = false;
        packet.skipAll();
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
    // uint32 numPending — number of unacknowledged calendar invites
    dispatchTable_[Opcode::SMSG_CALENDAR_SEND_NUM_PENDING] = [this](network::Packet& packet) {
        // uint32 numPending — number of unacknowledged calendar invites
        if (packet.hasRemaining(4)) {
            uint32_t numPending = packet.readUInt32();
            calendarPendingInvites_ = numPending;
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
        // Rich notification: eventId(8) + title(cstring) + eventTime(8) + flags(4) +
        //                   eventType(1) + dungeonId(4) + inviteId(8) + status(1) + rank(1) +
        //                   isGuildEvent(1) + inviterGuid(8)
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
    // uint32 unixTime — server's current unix timestamp; use to sync gameTime_
    dispatchTable_[Opcode::SMSG_SERVERTIME] = [this](network::Packet& packet) {
        // uint32 unixTime — server's current unix timestamp; use to sync gameTime_
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
    // uint32 throttleMs — rate-limited group action; notify the player
    dispatchTable_[Opcode::SMSG_GROUPACTION_THROTTLED] = [this](network::Packet& packet) {
        // uint32 throttleMs — rate-limited group action; notify the player
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
        LOG_INFO("SMSG_GMRESPONSE_RECEIVED: ticketId=", ticketId,
                 " subject='", subject, "'");
    };
    // uint32 ticketId + uint8 status (1=open, 2=surveyed, 3=need_more_help)
    dispatchTable_[Opcode::SMSG_GMRESPONSE_STATUS_UPDATE] = [this](network::Packet& packet) {
        // uint32 ticketId + uint8 status (1=open, 2=surveyed, 3=need_more_help)
        if (packet.hasRemaining(5)) {
            uint32_t ticketId = packet.readUInt32();
            uint8_t  status   = packet.readUInt8();
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
    // Broadcast of another player's collision height change — cosmetic only.
    registerSkipHandler(Opcode::MSG_MOVE_SET_COLLISION_HGT);
    // Client uses this outbound; treat inbound variant as no-op for robustness.
    registerSkipHandler(Opcode::MSG_MOVE_WORLDPORT_ACK);
    // Observed custom server packet (8 bytes). Safe-consume for now.
    registerSkipHandler(Opcode::MSG_MOVE_TIME_SKIPPED);
    // loggingOut_ already cleared by cancelLogout(); this is server's confirmation
    registerSkipHandler(Opcode::SMSG_LOGOUT_CANCEL_ACK);
    // These packets are not damage-shield events. Consume them without
    // synthesizing reflected damage entries or misattributing GUIDs.
    registerSkipHandler(Opcode::SMSG_AURACASTLOG);
    // These packets are not damage-shield events. Consume them without
    // synthesizing reflected damage entries or misattributing GUIDs.
    registerSkipHandler(Opcode::SMSG_SPELLBREAKLOG);
    // Consume silently — informational, no UI action needed
    registerSkipHandler(Opcode::SMSG_ITEM_REFUND_INFO_RESPONSE);
    // Consume silently — informational, no UI action needed
    registerSkipHandler(Opcode::SMSG_LOOT_LIST);
    // Same format as LOCKOUT_ADDED; consume
    registerSkipHandler(Opcode::SMSG_CALENDAR_RAID_LOCKOUT_UPDATED);
    // Consume — remaining server notifications not yet parsed
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
        Opcode::SMSG_CALENDAR_SEND_CALENDAR,
        Opcode::SMSG_CALENDAR_SEND_EVENT,
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
        Opcode::SMSG_PLAYER_SKINNED,
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

    // -----------------------------------------------------------------------
    // Domain handler registrations (override duplicate entries above)
    // -----------------------------------------------------------------------
    chatHandler_->registerOpcodes(dispatchTable_);
    movementHandler_->registerOpcodes(dispatchTable_);
    combatHandler_->registerOpcodes(dispatchTable_);
    spellHandler_->registerOpcodes(dispatchTable_);
    inventoryHandler_->registerOpcodes(dispatchTable_);
    socialHandler_->registerOpcodes(dispatchTable_);
    questHandler_->registerOpcodes(dispatchTable_);
    wardenHandler_->registerOpcodes(dispatchTable_);
}

void GameHandler::handlePacket(network::Packet& packet) {
    // Do NOT drop packets with an empty body. getSize() is the payload length and the
    // opcode is carried separately, so for the many opcodes that have no payload the
    // opcode *is* the message. Dropping them here silently swallowed
    // SMSG_LOGOUT_COMPLETE — the server logged the character out and moved on while
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
    if (wardenGateSeen_ && (!preLogicalOp || *preLogicalOp != Opcode::SMSG_WARDEN_DATA)) {
        ++wardenPacketsAfterGate_;
    }
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
