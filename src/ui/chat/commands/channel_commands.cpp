// Channel commands: /s, /y, /p, /g, /raid, /rw, /o, /bg, /i, /join, /leave, /wts, /wtb, /1-9, /w, /r
// Moved from ChatPanel::sendChatMessage() channel dispatch section (Phase 3).
// These commands send messages to specific chat channels and/or switch the
// chat-type dropdown on the panel.
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_utils.hpp"
#include "game/game_handler.hpp"
#include <algorithm>
#include <cctype>

using wowee::ui::chat_utils::trim;
using wowee::ui::chat_utils::toLower;
namespace chat_utils = wowee::ui::chat_utils;

namespace {


// Send a whisper, intercepting PortBot targets for GM teleport commands.
// Returns true if the whisper was handled (PortBot or normal send), false if empty.
bool sendWhisperOrPortBot(wowee::game::GameHandler& gameHandler,
                          const std::string& target,
                          const std::string& message) {
    if (chat_utils::isPortBotTarget(target)) {
        std::string cmd = chat_utils::portBotCommandFor(message);
        wowee::game::MessageChatData msg;
        msg.type = wowee::game::ChatType::SYSTEM;
        msg.language = wowee::game::ChatLanguage::UNIVERSAL;
        if (cmd.empty() || cmd == "__help__") {
            msg.message = chat_utils::portBotHelpText();
            gameHandler.addLocalChatMessage(msg);
            return true;
        }
        gameHandler.sendChatMessage(wowee::game::ChatType::SAY, cmd, "");
        msg.message = "PortBot executed: " + cmd;
        gameHandler.addLocalChatMessage(msg);
        return true;
    }
    if (!message.empty()) {
        gameHandler.sendChatMessage(wowee::game::ChatType::WHISPER, message, target);
    }
    return true;
}

} // anonymous namespace

namespace wowee { namespace ui {

// --- Helper: send a message via a specific chat type + optionally switch dropdown ---
static ChatCommandResult sendAndSwitch(ChatCommandContext& ctx,
                                        game::ChatType chatType,
                                        int switchIdx,
                                        const std::string& target = "") {
    if (!ctx.args.empty())
        ctx.gameHandler.sendChatMessage(chatType, ctx.args, target);
    ctx.panel.setSelectedChatType(switchIdx);
    return {};
}

// --- /s, /say ---
class SayCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::SAY, 0);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"s", "say"}; }
    [[nodiscard]] std::string helpText() const override { return "Say to nearby players"; }
};

// --- /y, /yell, /shout ---
class YellCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::YELL, 1);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"y", "yell", "shout"}; }
    [[nodiscard]] std::string helpText() const override { return "Yell to a wider area"; }
};

// --- /p, /party ---
class PartyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::PARTY, 2);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"p", "party"}; }
    [[nodiscard]] std::string helpText() const override { return "Party chat"; }
};

// --- /g, /guild ---
class GuildChatCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::GUILD, 3);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"g", "guild"}; }
    [[nodiscard]] std::string helpText() const override { return "Guild chat"; }
};

// --- /raid, /rsay, /ra ---
class RaidChatCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::RAID, 5);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"raid", "rsay", "ra"}; }
    [[nodiscard]] std::string helpText() const override { return "Raid chat"; }
};

// --- /raidwarning, /rw ---
class RaidWarningCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::RAID_WARNING, 8);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"raidwarning", "rw"}; }
    [[nodiscard]] std::string helpText() const override { return "Raid warning"; }
};

// --- /officer, /o, /osay ---
class OfficerCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::OFFICER, 6);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"officer", "o", "osay"}; }
    [[nodiscard]] std::string helpText() const override { return "Guild officer chat"; }
};

// --- /battleground, /bg ---
class BattlegroundChatCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::BATTLEGROUND, 7);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"battleground", "bg"}; }
    [[nodiscard]] std::string helpText() const override { return "Battleground chat"; }
};

// --- /instance, /i ---
class InstanceChatCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        return sendAndSwitch(ctx, game::ChatType::PARTY, 9);
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"instance", "i"}; }
    [[nodiscard]] std::string helpText() const override { return "Instance chat"; }
};

// --- /join ---
class JoinCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty() && ctx.gameHandler.hasPendingBgInvite()) {
            ctx.gameHandler.acceptBattlefield();
            return {};
        }
        if (!ctx.args.empty()) {
            size_t pwStart = ctx.args.find(' ');
            std::string channelName = (pwStart != std::string::npos) ? ctx.args.substr(0, pwStart) : ctx.args;
            std::string password = (pwStart != std::string::npos) ? ctx.args.substr(pwStart + 1) : "";
            ctx.gameHandler.joinChannel(channelName, password);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"join"}; }
    [[nodiscard]] std::string helpText() const override { return "Join a chat channel"; }
};

// --- /leave (channel) ---
// Note: /leave without args is handled by group_commands (leave party).
// This command only triggers with args (channel name).
class LeaveChannelCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.leaveChannel(ctx.args);
        }
        // If no args, the group LeaveCommand will handle /leave (leave party)
        // so we return not-handled to allow fallthrough
        if (ctx.args.empty()) return {false, false};
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"leavechannel"}; }
    [[nodiscard]] std::string helpText() const override { return "Leave a chat channel"; }
};

// --- /wts, /wtb ---
class TradeChannelCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};
        const std::string tag = (ctx.fullCommand == "wts") ? "[WTS] " : "[WTB] ";
        std::string tradeChan;
        for (const auto& ch : ctx.gameHandler.getJoinedChannels()) {
            std::string chLow = ch;
            for (char& c : chLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (chLow.rfind("trade", 0) == 0) { tradeChan = ch; break; }
        }
        if (tradeChan.empty()) {
            game::MessageChatData errMsg;
            errMsg.type = game::ChatType::SYSTEM;
            errMsg.language = game::ChatLanguage::UNIVERSAL;
            errMsg.message = "You are not in the Trade channel.";
            ctx.gameHandler.addLocalChatMessage(errMsg);
            return {};
        }
        ctx.gameHandler.sendChatMessage(game::ChatType::CHANNEL, tag + ctx.args, tradeChan);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"wts", "wtb"}; }
    [[nodiscard]] std::string helpText() const override { return "Send to Trade channel ([WTS]/[WTB] prefix)"; }
};

// --- /1 through /9 - channel shortcuts ---
class ChannelNumberCommand : public IChatCommand {
public:
    explicit ChannelNumberCommand(int num) : num_(num), alias_(std::to_string(num)) {}
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        std::string channelName = ctx.gameHandler.getChannelByIndex(num_);
        if (channelName.empty()) {
            game::MessageChatData errMsg;
            errMsg.type = game::ChatType::SYSTEM;
            errMsg.message = "You are not in channel " + std::to_string(num_) + ".";
            ctx.gameHandler.addLocalChatMessage(errMsg);
            return {};
        }
        if (!ctx.args.empty()) {
            ctx.gameHandler.sendChatMessage(game::ChatType::CHANNEL, ctx.args, channelName);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {alias_}; }
    [[nodiscard]] std::string helpText() const override { return "Send to channel " + alias_; }
private:
    int num_;
    std::string alias_;
};

// --- /w, /whisper, /tell, /t ---
class WhisperCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.setSelectedChatType(4);  // Switch to whisper mode
        if (!ctx.args.empty()) {
            size_t msgStart = ctx.args.find(' ');
            if (msgStart != std::string::npos) {
                // /w PlayerName message - send whisper immediately (PortBot-aware)
                std::string target = ctx.args.substr(0, msgStart);
                std::string message = ctx.args.substr(msgStart + 1);
                sendWhisperOrPortBot(ctx.gameHandler, target, message);
                // Set whisper target for future messages
                char* buf = ctx.panel.getWhisperTargetBuffer();
                size_t sz = ctx.panel.getWhisperTargetBufferSize();
                strncpy(buf, target.c_str(), sz - 1);
                buf[sz - 1] = '\0';
            } else {
                // /w PlayerName - switch to whisper mode with target set
                char* buf = ctx.panel.getWhisperTargetBuffer();
                size_t sz = ctx.panel.getWhisperTargetBufferSize();
                strncpy(buf, ctx.args.c_str(), sz - 1);
                buf[sz - 1] = '\0';
            }
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"w", "whisper", "tell", "t"}; }
    [[nodiscard]] std::string helpText() const override { return "Whisper to a player"; }
};

// --- /r, /reply ---
class ReplyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.setSelectedChatType(4);
        std::string lastSender = ctx.gameHandler.getLastWhisperSender();
        if (lastSender.empty()) {
            game::MessageChatData sysMsg;
            sysMsg.type = game::ChatType::SYSTEM;
            sysMsg.language = game::ChatLanguage::UNIVERSAL;
            sysMsg.message = "No one has whispered you yet.";
            ctx.gameHandler.addLocalChatMessage(sysMsg);
            return {};
        }
        char* buf = ctx.panel.getWhisperTargetBuffer();
        size_t sz = ctx.panel.getWhisperTargetBufferSize();
        strncpy(buf, lastSender.c_str(), sz - 1);
        buf[sz - 1] = '\0';
        if (!ctx.args.empty()) {
            // PortBot-aware whisper send
            sendWhisperOrPortBot(ctx.gameHandler, lastSender, ctx.args);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"r", "reply"}; }
    [[nodiscard]] std::string helpText() const override { return "Reply to last whisper"; }
};

// --- Registration ---
void registerChannelCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<SayCommand>());
    reg.registerCommand(std::make_unique<YellCommand>());
    reg.registerCommand(std::make_unique<PartyCommand>());
    reg.registerCommand(std::make_unique<GuildChatCommand>());
    reg.registerCommand(std::make_unique<RaidChatCommand>());
    reg.registerCommand(std::make_unique<RaidWarningCommand>());
    reg.registerCommand(std::make_unique<OfficerCommand>());
    reg.registerCommand(std::make_unique<BattlegroundChatCommand>());
    reg.registerCommand(std::make_unique<InstanceChatCommand>());
    reg.registerCommand(std::make_unique<JoinCommand>());
    reg.registerCommand(std::make_unique<LeaveChannelCommand>());
    reg.registerCommand(std::make_unique<TradeChannelCommand>());
    for (int n = 1; n <= 9; ++n)
        reg.registerCommand(std::make_unique<ChannelNumberCommand>(n));
    reg.registerCommand(std::make_unique<WhisperCommand>());
    reg.registerCommand(std::make_unique<ReplyCommand>());
}

} // namespace ui
} // namespace wowee
