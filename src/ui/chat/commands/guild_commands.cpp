// Guild commands: /ginfo, /groster, /gmotd, /gpromote, /gdemote, /gquit,
//                 /ginvite, /gkick, /gcreate, /gdisband, /gleader
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"

namespace wowee { namespace ui {

// --- /ginfo, /guildinfo ---
class GuildInfoCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestGuildInfo();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"ginfo", "guildinfo"}; }
    [[nodiscard]] std::string helpText() const override { return "Show guild info"; }
};

// --- /groster, /guildroster ---
class GuildRosterCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestGuildRoster();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"groster", "guildroster"}; }
    [[nodiscard]] std::string helpText() const override { return "Show guild roster"; }
};

// --- /gmotd, /guildmotd ---
class GuildMotdCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.setGuildMotd(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gmotd <message>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gmotd", "guildmotd"}; }
    [[nodiscard]] std::string helpText() const override { return "Set guild message of the day"; }
};

// --- /gpromote, /guildpromote ---
class GuildPromoteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.promoteGuildMember(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gpromote <player>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gpromote", "guildpromote"}; }
    [[nodiscard]] std::string helpText() const override { return "Promote guild member"; }
};

// --- /gdemote, /guilddemote ---
class GuildDemoteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.demoteGuildMember(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gdemote <player>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gdemote", "guilddemote"}; }
    [[nodiscard]] std::string helpText() const override { return "Demote guild member"; }
};

// --- /gquit, /guildquit, /leaveguild ---
class GuildQuitCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.leaveGuild();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gquit", "guildquit", "leaveguild"}; }
    [[nodiscard]] std::string helpText() const override { return "Leave guild"; }
};

// --- /ginvite, /guildinvite ---
class GuildInviteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.inviteToGuild(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /ginvite <player>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"ginvite", "guildinvite"}; }
    [[nodiscard]] std::string helpText() const override { return "Invite player to guild"; }
};

// --- /gkick, /guildkick ---
class GuildKickCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.kickGuildMember(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gkick <player>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gkick", "guildkick"}; }
    [[nodiscard]] std::string helpText() const override { return "Kick player from guild"; }
};

// --- /gcreate, /guildcreate ---
class GuildCreateCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.createGuild(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gcreate <guild name>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gcreate", "guildcreate"}; }
    [[nodiscard]] std::string helpText() const override { return "Create a guild"; }
};

// --- /gdisband, /guilddisband ---
class GuildDisbandCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.disbandGuild();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gdisband", "guilddisband"}; }
    [[nodiscard]] std::string helpText() const override { return "Disband guild"; }
};

// --- /gleader, /guildleader ---
class GuildLeaderCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.setGuildLeader(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /gleader <player>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gleader", "guildleader"}; }
    [[nodiscard]] std::string helpText() const override { return "Transfer guild leadership"; }
};

// --- Registration ---
void registerGuildCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<GuildInfoCommand>());
    reg.registerCommand(std::make_unique<GuildRosterCommand>());
    reg.registerCommand(std::make_unique<GuildMotdCommand>());
    reg.registerCommand(std::make_unique<GuildPromoteCommand>());
    reg.registerCommand(std::make_unique<GuildDemoteCommand>());
    reg.registerCommand(std::make_unique<GuildQuitCommand>());
    reg.registerCommand(std::make_unique<GuildInviteCommand>());
    reg.registerCommand(std::make_unique<GuildKickCommand>());
    reg.registerCommand(std::make_unique<GuildCreateCommand>());
    reg.registerCommand(std::make_unique<GuildDisbandCommand>());
    reg.registerCommand(std::make_unique<GuildLeaderCommand>());
}

} // namespace ui
} // namespace wowee
