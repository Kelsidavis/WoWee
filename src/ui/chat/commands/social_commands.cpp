// Social commands: /friend, /removefriend, /ignore, /unignore, /invite, /inspect, /who
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include <algorithm>
#include <cctype>

namespace wowee { namespace ui {

// --- /invite ---
class InviteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};
        ctx.gameHandler.inviteToGroup(ctx.args);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"invite"}; }
    [[nodiscard]] std::string helpText() const override { return "Invite player to group"; }
};

// --- /inspect ---
class InspectCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.inspectTarget();
        ctx.panel.getSlashCmds().showInspect = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"inspect"}; }
    [[nodiscard]] std::string helpText() const override { return "Inspect target's equipment"; }
};

// --- /friend, /addfriend ---
class FriendCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            size_t subCmdSpace = ctx.args.find(' ');
            if (ctx.fullCommand == "friend" && subCmdSpace != std::string::npos) {
                std::string subCmd = ctx.args.substr(0, subCmdSpace);
                std::transform(subCmd.begin(), subCmd.end(), subCmd.begin(), ::tolower);
                if (subCmd == "add") {
                    ctx.gameHandler.addFriend(ctx.args.substr(subCmdSpace + 1));
                    return {};
                } if (subCmd == "remove" || subCmd == "delete" || subCmd == "rem") {
                    ctx.gameHandler.removeFriend(ctx.args.substr(subCmdSpace + 1));
                    return {};
                }
            } else {
                // /addfriend name or /friend name (assume add)
                ctx.gameHandler.addFriend(ctx.args);
                return {};
            }
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /friend add <name> or /friend remove <name>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"friend", "addfriend"}; }
    [[nodiscard]] std::string helpText() const override { return "Add/remove friend"; }
};

// --- /removefriend, /delfriend, /remfriend ---
class RemoveFriendCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.removeFriend(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /removefriend <name>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"removefriend", "delfriend", "remfriend"}; }
    [[nodiscard]] std::string helpText() const override { return "Remove friend"; }
};

// --- /ignore ---
class IgnoreCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.addIgnore(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /ignore <name>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"ignore"}; }
    [[nodiscard]] std::string helpText() const override { return "Ignore player messages"; }
};

// --- /unignore ---
class UnignoreCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.removeIgnore(ctx.args);
            return {};
        }
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = "Usage: /unignore <name>";
        ctx.gameHandler.addLocalChatMessage(msg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"unignore"}; }
    [[nodiscard]] std::string helpText() const override { return "Unignore player"; }
};

// --- /who, /whois, /online, /players ---
class WhoCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        std::string query = ctx.args;
        // Trim
        size_t first = query.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) query.clear();
        else { size_t last = query.find_last_not_of(" \t\r\n"); query = query.substr(first, last - first + 1); }

        if (ctx.fullCommand == "whois" && query.empty()) {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "Usage: /whois <playerName>";
            ctx.gameHandler.addLocalChatMessage(msg);
            return {};
        }
        if (ctx.fullCommand == "who" && (query == "help" || query == "?")) {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "Who commands: /who [name/filter], /whois <name>, /online";
            ctx.gameHandler.addLocalChatMessage(msg);
            return {};
        }
        ctx.gameHandler.queryWho(query);
        ctx.panel.getSlashCmds().showWho = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"who", "whois", "online", "players"}; }
    [[nodiscard]] std::string helpText() const override { return "List online players"; }
};

// --- /duel ---
class DuelCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.gameHandler.hasTarget()) {
            ctx.gameHandler.proposeDuel(ctx.gameHandler.getTargetGuid());
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must target a player to challenge to a duel.";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"duel"}; }
    [[nodiscard]] std::string helpText() const override { return "Challenge target to duel"; }
};

// --- /trade ---
class TradeCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.gameHandler.hasTarget()) {
            ctx.gameHandler.initiateTrade(ctx.gameHandler.getTargetGuid());
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must target a player to trade with.";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"trade"}; }
    [[nodiscard]] std::string helpText() const override { return "Initiate trade with target"; }
};

// --- Registration ---
void registerSocialCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<InviteCommand>());
    reg.registerCommand(std::make_unique<InspectCommand>());
    reg.registerCommand(std::make_unique<FriendCommand>());
    reg.registerCommand(std::make_unique<RemoveFriendCommand>());
    reg.registerCommand(std::make_unique<IgnoreCommand>());
    reg.registerCommand(std::make_unique<UnignoreCommand>());
    reg.registerCommand(std::make_unique<WhoCommand>());
    reg.registerCommand(std::make_unique<DuelCommand>());
    reg.registerCommand(std::make_unique<TradeCommand>());
}

} // namespace ui
} // namespace wowee
