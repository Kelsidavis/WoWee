#include "ui/macro_text.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_utils.hpp"
#include "ui/chat/macro_evaluator.hpp"
#include "ui/chat/game_state_adapter.hpp"
#include "ui/chat/input_modifier_adapter.hpp"
#include "ui/chat/gm_command_data.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/ui_colors.hpp"
#include "rendering/vk_context.hpp"
#include "core/application.hpp"
#include "addons/addon_manager.hpp"
#include "core/coordinates.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "game/expansion_profile.hpp"
#include "game/character.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cctype>
#include <sstream>
#include <cstdlib>
#include <cctype>
#include <chrono>
#include <ctime>
#include <unordered_set>
#include <unordered_map>
#include "core/local_time.hpp"
#include "ui/framexml_takeover.hpp"

namespace wowee { namespace ui {

ChatPanel::ChatPanel() {
    registerAllCommands();
}







// --- Command registration (calls into each command group file) ---
// Forward declarations of registration functions from command files
void registerSystemCommands(ChatCommandRegistry& reg);
void registerSocialCommands(ChatCommandRegistry& reg);
void registerChannelCommands(ChatCommandRegistry& reg);
void registerCombatCommands(ChatCommandRegistry& reg);
void registerGroupCommands(ChatCommandRegistry& reg);
void registerGuildCommands(ChatCommandRegistry& reg);
void registerTargetCommands(ChatCommandRegistry& reg);
void registerEmoteCommands(ChatCommandRegistry& reg);
void registerMiscCommands(ChatCommandRegistry& reg);
void registerHelpCommands(ChatCommandRegistry& reg);
void registerGmCommands(ChatCommandRegistry& reg);

void ChatPanel::registerAllCommands() {
    registerSystemCommands(commandRegistry_);
    registerSocialCommands(commandRegistry_);
    registerChannelCommands(commandRegistry_);
    registerCombatCommands(commandRegistry_);
    registerGroupCommands(commandRegistry_);
    registerGuildCommands(commandRegistry_);
    registerTargetCommands(commandRegistry_);
    registerEmoteCommands(commandRegistry_);
    registerMiscCommands(commandRegistry_);
    registerHelpCommands(commandRegistry_);
    registerGmCommands(commandRegistry_);
}

// renderBubbles delegates to ChatBubbleManager (Phase 1.4)
void ChatPanel::renderBubbles(game::GameHandler& gameHandler) {
    bubbleManager_.render(gameHandler, services_);
}

// setupCallbacks delegates to ChatBubbleManager (Phase 1.4)
void ChatPanel::setupCallbacks(game::GameHandler& gameHandler) {
    bubbleManager_.setupCallback(gameHandler);
    // Also here, not only in render(): with the chat window handed to FrameXML
    // this panel is never drawn, and the handler cached there was left null for
    // the entry points below that still have callers.
    cachedGameHandler_ = &gameHandler;
}

namespace {

/// A string as Lua source, so a link's own punctuation cannot end it early.
std::string asLuaString(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '\\' || c == '"') out += '\\';
        out += c;
    }
    out += '"';
    return out;
}

}  // namespace

void ChatPanel::insertChatLink(const std::string& link) {
    // ChatEdit_InsertLink is what the interface uses for the same gesture: it
    // finds the active edit box and inserts there, answering false when none
    // is active - which on its own would drop the click, hence the open behind
    // it. There is no box on this side to append to any more.
    if (!cachedGameHandler_) return;
    cachedGameHandler_->runInterfaceCommand(
        "local l = " + asLuaString(link) +
        " if not ChatEdit_InsertLink(l) then ChatFrame_OpenChat(l) end");
}



void ChatPanel::setWhisperTarget(const std::string& name) {
    selectedChatType_ = 4;  // WHISPER
    strncpy(whisperTargetBuffer_, name.c_str(), sizeof(whisperTargetBuffer_) - 1);
    whisperTargetBuffer_[sizeof(whisperTargetBuffer_) - 1] = '\0';
}

ChatPanel::SlashCommands ChatPanel::consumeSlashCommands() {
    SlashCommands result = slashCmds_;
    slashCmds_ = {};
    return result;
}

// Execute all non-comment lines of a macro body in sequence.
void ChatPanel::executeMacroText(game::GameHandler& gameHandler,
                                  const std::string& macroText) {
    macroStopped_ = false;
    for (const auto& cmd : macroCommandLines(macroText)) {
        strncpy(chatInputBuffer_, cmd.c_str(), sizeof(chatInputBuffer_) - 1);
        chatInputBuffer_[sizeof(chatInputBuffer_) - 1] = '\0';
        sendChatMessage(gameHandler);
        if (macroStopped_) break;
    }
    macroStopped_ = false;
}

std::vector<std::string> ChatPanel::registryCommandNames() const {
    return commandRegistry_.getCompletions("");
}

bool ChatPanel::runRegistryCommand(game::GameHandler& gameHandler,
                                   const std::string& alias, const std::string& args) {
    std::string lower = alias;
    for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    ChatCommandContext ctx{.gameHandler = gameHandler, .services = services_, .panel = *this, .args = args, .fullCommand = lower};
    return commandRegistry_.dispatch(lower, ctx).handled;
}

void ChatPanel::sendChatMessage(game::GameHandler& gameHandler) {
    if (strlen(chatInputBuffer_) == 0) return;
    std::string input(chatInputBuffer_);

    // Save to sent-message history (skip pure whitespace, cap at 50 entries)
    {
        bool allSpace = true;
        for (char c : input) { if (!std::isspace(static_cast<unsigned char>(c))) { allSpace = false; break; } }
        if (!allSpace) {
            if (chatSentHistory_.empty() || chatSentHistory_.back() != input) {
                chatSentHistory_.push_back(input);
                if (chatSentHistory_.size() > 50)
                    chatSentHistory_.erase(chatSentHistory_.begin());
            }
        }
    }

    game::ChatType type = game::ChatType::SAY;
    std::string message = input;
    std::string target;

    // GM dot-prefix commands (.gm, .tele, .additem, etc.)
    if (input.size() > 1 && input[0] == '.') {
        LOG_INFO("GM command: '", input, "' - sending as SAY to server");
        gameHandler.sendChatMessage(game::ChatType::SAY, input, "");

        std::string dotCmd = input;
        size_t sp = dotCmd.find(' ');
        std::string cmdPart = (sp != std::string::npos)
            ? dotCmd.substr(1, sp - 1) : dotCmd.substr(1);
        for (char& c : cmdPart) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        std::string feedback;
        for (const auto& entry : kGmCommands) {
            if (entry.name == cmdPart) {
                feedback = "Sent: " + input + "  (" + std::string(entry.help) + ")";
                break;
            }
        }
        if (feedback.empty())
            feedback = "Sent: " + input
                + "  (requires GM access - server console: account set gmlevel <user> 3 -1)";
        gameHandler.addLocalChatMessage(chat_utils::makeSystemMessage(feedback));
        chatInputBuffer_[0] = '\0';
        return;
    }

    // Slash commands
    if (input.size() > 1 && input[0] == '/') {
        std::string command = input.substr(1);
        size_t spacePos = command.find(' ');
        std::string cmd = (spacePos != std::string::npos) ? command.substr(0, spacePos) : command;
        std::string cmdLower = cmd;
        for (char& c : cmdLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        // /run <lua code>
        if ((cmdLower == "run" || cmdLower == "script") && spacePos != std::string::npos) {
            std::string luaCode = command.substr(spacePos + 1);
            auto* am = services_.addonManager;
            if (am) {
                am->runScript(luaCode);
            } else {
                gameHandler.addUIError("Addon system not initialized.");
            }
            chatInputBuffer_[0] = '\0';
            return;
        }

        // /dump <expression>
        if ((cmdLower == "dump" || cmdLower == "print") && spacePos != std::string::npos) {
            std::string expr = command.substr(spacePos + 1);
            auto* am = services_.addonManager;
            if (am && am->isInitialized()) {
                std::string wrapped = "local __v = " + expr +
                    "; if type(__v) == 'table' then "
                    "  local parts = {} "
                    "  for k,v in pairs(__v) do parts[#parts+1] = tostring(k)..'='..tostring(v) end "
                    "  print('{' .. table.concat(parts, ', ') .. '}') "
                    "else print(tostring(__v)) end";
                am->runScript(wrapped);
            } else {
                game::MessageChatData errMsg;
                errMsg.type = game::ChatType::SYSTEM;
                errMsg.language = game::ChatLanguage::UNIVERSAL;
                errMsg.message = "Addon system not initialized.";
                gameHandler.addLocalChatMessage(errMsg);
            }
            chatInputBuffer_[0] = '\0';
            return;
        }

        // Addon slash commands (SlashCmdList)
        {
            auto* am = services_.addonManager;
            if (am && am->isInitialized()) {
                std::string slashCmd = "/" + cmdLower;
                std::string slashArgs;
                if (spacePos != std::string::npos) slashArgs = command.substr(spacePos + 1);
                if (am->getLuaEngine()->dispatchSlashCommand(slashCmd, slashArgs)) {
                    chatInputBuffer_[0] = '\0';
                    return;
                }
            }
        }

        // Dispatch through command registry (Phase 3.11)
        std::string args;
        if (spacePos != std::string::npos)
            args = command.substr(spacePos + 1);

        ChatCommandContext ctx{.gameHandler = gameHandler, .services = services_, .panel = *this, .args = args, .fullCommand = cmdLower};
        ChatCommandResult result = commandRegistry_.dispatch(cmdLower, ctx);
        if (result.handled) {
            if (result.clearInput)
                chatInputBuffer_[0] = '\0';
            return;
        }

        // Emote fallthrough - dynamic DBC lookup for emote text.
        {
            std::string targetName;
            const std::string* targetNamePtr = nullptr;
            if (gameHandler.hasTarget()) {
                auto targetEntity = gameHandler.getTarget();
                if (targetEntity) {
                    targetName = game::entityDisplayName(targetEntity);
                    if (!targetName.empty()) targetNamePtr = &targetName;
                }
            }

            std::string emoteText = rendering::AnimationController::getEmoteText(cmdLower, targetNamePtr);
            if (!emoteText.empty()) {
                auto* renderer = services_.renderer;
                if (renderer) {
                    if (auto* ac = renderer->getAnimationController()) ac->playEmote(cmdLower);
                }

                uint32_t dbcId = rendering::AnimationController::getEmoteDbcId(cmdLower);
                if (dbcId != 0) {
                    uint64_t targetGuid = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.sendTextEmote(dbcId, targetGuid);
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::TEXT_EMOTE;
                msg.language = game::ChatLanguage::COMMON;
                msg.message = emoteText;
                gameHandler.addLocalChatMessage(msg);

                chatInputBuffer_[0] = '\0';
                return;
            }
        }

        // Unrecognized slash command - fall through to dropdown chat type
        message = input;
    }

    // Determine chat type from dropdown selection
    switch (selectedChatType_) {
        case 0: type = game::ChatType::SAY; break;
        case 1: type = game::ChatType::YELL; break;
        case 2: type = game::ChatType::PARTY; break;
        case 3: type = game::ChatType::GUILD; break;
        case 4: type = game::ChatType::WHISPER; target = whisperTargetBuffer_; break;
        case 5: type = game::ChatType::RAID; break;
        case 6: type = game::ChatType::OFFICER; break;
        case 7: type = game::ChatType::BATTLEGROUND; break;
        case 8: type = game::ChatType::RAID_WARNING; break;
        case 9: type = game::ChatType::PARTY; break;
        case 10: {
            const auto& chans = gameHandler.getJoinedChannels();
            if (!chans.empty() && selectedChannelIdx_ < static_cast<int>(chans.size())) {
                type = game::ChatType::CHANNEL;
                target = chans[selectedChannelIdx_];
            } else { type = game::ChatType::SAY; }
            break;
        }
        default: type = game::ChatType::SAY; break;
    }

    // PortBot whisper interception
    if (type == game::ChatType::WHISPER && chat_utils::isPortBotTarget(target)) {
        std::string cmd = chat_utils::portBotCommandFor(message);
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        if (cmd.empty() || cmd == "__help__") {
            msg.message = chat_utils::portBotHelpText();
            gameHandler.addLocalChatMessage(msg);
            chatInputBuffer_[0] = '\0';
            return;
        }

        gameHandler.sendChatMessage(game::ChatType::SAY, cmd, "");
        msg.message = "PortBot executed: " + cmd;
        gameHandler.addLocalChatMessage(msg);
        chatInputBuffer_[0] = '\0';
        return;
    }

    // Validate whisper has a target
    if (type == game::ChatType::WHISPER && target.empty()) {
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "You must specify a player name for whisper.";
        gameHandler.addLocalChatMessage(msg);
        chatInputBuffer_[0] = '\0';
        return;
    }

    if (!message.empty()) {
        gameHandler.sendChatMessage(type, message, target);
    }
    chatInputBuffer_[0] = '\0';
}

} // namespace ui
} // namespace wowee
