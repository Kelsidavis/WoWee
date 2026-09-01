// Group commands: /readycheck, /ready, /notready, /yield, /afk, /dnd,
//                 /uninvite, /leave, /maintank, /mainassist, /clearmaintank,
//                 /clearmainassist, /raidinfo, /raidconvert, /lootmethod,
//                 /lootthreshold, /mark, /roll
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "game/group_defines.hpp"
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include <algorithm>
#include <cctype>
#include <ctime>
#include <string>

namespace wowee { namespace ui {

// --- /readycheck, /rc ---
class ReadyCheckCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.initiateReadyCheck();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"readycheck", "rc"}; }
    [[nodiscard]] std::string helpText() const override { return "Initiate ready check"; }
};

// --- /ready ---
class ReadyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.respondToReadyCheck(true);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"ready"}; }
    [[nodiscard]] std::string helpText() const override { return "Respond yes to ready check"; }
};

// --- /notready, /nr ---
class NotReadyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.respondToReadyCheck(false);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"notready", "nr"}; }
    [[nodiscard]] std::string helpText() const override { return "Respond no to ready check"; }
};

// --- /yield, /forfeit, /surrender ---
class YieldCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.forfeitDuel();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"yield", "forfeit", "surrender"}; }
    [[nodiscard]] std::string helpText() const override { return "Forfeit current duel"; }
};

// --- /afk, /away ---
class AfkCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.toggleAfk(ctx.args);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"afk", "away"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle AFK status"; }
};

// --- /dnd, /busy ---
class DndCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.toggleDnd(ctx.args);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"dnd", "busy"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle Do Not Disturb"; }
};

// --- /uninvite, /kick ---
class UninviteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            ctx.gameHandler.uninvitePlayer(ctx.args);
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "Usage: /uninvite <player name>";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"uninvite", "kick"}; }
    [[nodiscard]] std::string helpText() const override { return "Remove player from group"; }
};

// --- /leave, /leaveparty ---
// /leave - leave party (no args) or leave channel (with args, WoW-style overload)
class LeavePartyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            // /leave ChannelName - leave a chat channel
            ctx.gameHandler.leaveChannel(ctx.args);
        } else {
            ctx.gameHandler.leaveParty();
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"leave", "leaveparty"}; }
    [[nodiscard]] std::string helpText() const override { return "Leave party/raid or channel"; }
};

// --- /maintank, /mt ---
class MainTankCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.gameHandler.hasTarget()) {
            ctx.gameHandler.setMainTank(ctx.gameHandler.getTargetGuid());
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must target a player to set as main tank.";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"maintank", "mt"}; }
    [[nodiscard]] std::string helpText() const override { return "Set target as main tank"; }
};

// --- /mainassist, /ma ---
class MainAssistCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.gameHandler.hasTarget()) {
            ctx.gameHandler.setMainAssist(ctx.gameHandler.getTargetGuid());
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must target a player to set as main assist.";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"mainassist", "ma"}; }
    [[nodiscard]] std::string helpText() const override { return "Set target as main assist"; }
};

// --- /clearmaintank ---
class ClearMainTankCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.clearMainTank();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"clearmaintank"}; }
    [[nodiscard]] std::string helpText() const override { return "Clear main tank assignment"; }
};

// --- /clearmainassist ---
class ClearMainAssistCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.clearMainAssist();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"clearmainassist"}; }
    [[nodiscard]] std::string helpText() const override { return "Clear main assist assignment"; }
};

// --- /raidinfo ---
class RaidInfoCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        // Asked again first, for the next time this is run. The reply is a
        // packet and arrives well after this returns, so what is printed is
        // what the last request brought back - the one the interface makes at
        // login, which is why there is something to print at all.
        ctx.gameHandler.requestRaidInfo();
        const auto& lockouts = ctx.gameHandler.getInstanceLockouts();
        if (lockouts.empty()) {
            ctx.gameHandler.addSystemChatMessage("You are not saved to any instances.");
            return {};
        }
        ctx.gameHandler.addSystemChatMessage("Saved instances:");
        const auto now = static_cast<uint64_t>(std::time(nullptr));
        for (const auto& lo : lockouts) {
            std::string name = ctx.gameHandler.getMapName(lo.mapId);
            if (name.empty()) name = "Instance " + std::to_string(lo.mapId);
            static const char* kDifficulties[] = {"Normal", "Heroic",
                                                  "25-Man Normal", "25-Man Heroic"};
            if (lo.difficulty < 4) {
                name += " (";
                name += kDifficulties[lo.difficulty];
                name += ")";
            }
            // Days and hours, which is the granularity a lockout is read at.
            // A reset already past is possible - the server sends the time it
            // was set to, and nothing here is told when it comes round.
            std::string when = "expired";
            if (lo.resetTime > now) {
                const uint64_t left = lo.resetTime - now;
                const uint64_t days = left / 86400;
                const uint64_t hours = (left % 86400) / 3600;
                when = days > 0 ? std::to_string(days) + "d " + std::to_string(hours) + "h"
                                : std::to_string(hours) + "h";
            }
            ctx.gameHandler.addSystemChatMessage(
                "  " + name + " - resets in " + when +
                (lo.extended ? " (extended)" : ""));
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"raidinfo"}; }
    [[nodiscard]] std::string helpText() const override { return "Show raid instance lockouts"; }
};

// --- /raidconvert ---
class RaidConvertCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.convertToRaid();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"raidconvert"}; }
    [[nodiscard]] std::string helpText() const override { return "Convert party to raid"; }
};

// --- /lootmethod, /grouploot, /setloot ---
class LootMethodCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.gameHandler.isInGroup()) {
            ctx.gameHandler.addUIError("You are not in a group.");
            return {};
        }
        if (ctx.args.empty()) {
            const auto& pd = ctx.gameHandler.getPartyData();
            const char* cur = game::lootMethodName(pd.lootMethod);
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = std::string("Current loot method: ") + cur;
            ctx.gameHandler.addLocalChatMessage(msg);
            msg.message = "Usage: /lootmethod ffa|roundrobin|master|group|needbeforegreed";
            ctx.gameHandler.addLocalChatMessage(msg);
            return {};
        }
        std::string arg = ctx.args;
        for (auto& c : arg) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        uint32_t method = 0xFFFFFFFF;
        if (arg == "ffa" || arg == "freeforall")         method = 0;
        else if (arg == "roundrobin" || arg == "rr")     method = 1;
        else if (arg == "master" || arg == "masterloot") method = 2;
        else if (arg == "group" || arg == "grouploot")   method = 3;
        else if (arg == "needbeforegreed" || arg == "nbg" || arg == "need") method = 4;

        if (method == 0xFFFFFFFF) {
            ctx.gameHandler.addUIError("Unknown loot method. Use: ffa, roundrobin, master, group, needbeforegreed");
        } else {
            const auto& pd = ctx.gameHandler.getPartyData();
            uint64_t masterGuid = (method == 2) ? ctx.gameHandler.getPlayerGuid() : 0;
            ctx.gameHandler.sendSetLootMethod(method, pd.lootThreshold, masterGuid);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"lootmethod", "grouploot", "setloot"}; }
    [[nodiscard]] std::string helpText() const override { return "Set loot method"; }
};

// --- /lootthreshold ---
class LootThresholdCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.gameHandler.isInGroup()) {
            ctx.gameHandler.addUIError("You are not in a group.");
            return {};
        }
        if (ctx.args.empty()) {
            const auto& pd = ctx.gameHandler.getPartyData();
            static constexpr const char* kQualityNames[] = {
                "Poor (grey)", "Common (white)", "Uncommon (green)",
                "Rare (blue)", "Epic (purple)", "Legendary (orange)"
            };
            const char* cur = (pd.lootThreshold < 6) ? kQualityNames[pd.lootThreshold] : "Unknown";
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = std::string("Current loot threshold: ") + cur;
            ctx.gameHandler.addLocalChatMessage(msg);
            msg.message = "Usage: /lootthreshold <0-5> (0=Poor, 1=Common, 2=Uncommon, 3=Rare, 4=Epic, 5=Legendary)";
            ctx.gameHandler.addLocalChatMessage(msg);
            return {};
        }
        std::string arg = ctx.args;
        while (!arg.empty() && arg.front() == ' ') arg.erase(arg.begin());
        uint32_t threshold = 0xFFFFFFFF;
        if (arg.size() == 1 && arg[0] >= '0' && arg[0] <= '5') {
            threshold = static_cast<uint32_t>(arg[0] - '0');
        } else {
            for (auto& c : arg) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (arg == "poor" || arg == "grey" || arg == "gray") threshold = 0;
            else if (arg == "common" || arg == "white")          threshold = 1;
            else if (arg == "uncommon" || arg == "green")        threshold = 2;
            else if (arg == "rare" || arg == "blue")             threshold = 3;
            else if (arg == "epic" || arg == "purple")           threshold = 4;
            else if (arg == "legendary" || arg == "orange")      threshold = 5;
        }
        if (threshold == 0xFFFFFFFF) {
            ctx.gameHandler.addUIError("Invalid threshold. Use 0-5 or: poor, common, uncommon, rare, epic, legendary");
        } else {
            const auto& pd = ctx.gameHandler.getPartyData();
            uint64_t masterGuid = (pd.lootMethod == 2) ? ctx.gameHandler.getPlayerGuid() : 0;
            ctx.gameHandler.sendSetLootMethod(pd.lootMethod, threshold, masterGuid);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"lootthreshold"}; }
    [[nodiscard]] std::string helpText() const override { return "Set loot quality threshold"; }
};

// --- /mark, /marktarget, /raidtarget ---
class MarkCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.gameHandler.hasTarget()) {
            game::MessageChatData noTgt;
            noTgt.type = game::ChatType::SYSTEM;
            noTgt.language = game::ChatLanguage::UNIVERSAL;
            noTgt.message = "No target selected.";
            ctx.gameHandler.addLocalChatMessage(noTgt);
            return {};
        }
        static constexpr const char* kMarkWords[] = {
            "star", "circle", "diamond", "triangle", "moon", "square", "cross", "skull"
        };
        uint8_t icon = 7; // default: skull
        if (!ctx.args.empty()) {
            std::string argLow = ctx.args;
            for (auto& c : argLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            while (!argLow.empty() && argLow.front() == ' ') argLow.erase(argLow.begin());
            if (argLow == "clear" || argLow == "0" || argLow == "none") {
                ctx.gameHandler.setRaidMark(ctx.gameHandler.getTargetGuid(), 0xFF);
                return {};
            }
            bool found = false;
            for (int mi = 0; mi < 8; ++mi) {
                if (argLow == kMarkWords[mi]) { icon = static_cast<uint8_t>(mi); found = true; break; }
            }
            if (!found && !argLow.empty() && argLow[0] >= '1' && argLow[0] <= '8') {
                icon = static_cast<uint8_t>(argLow[0] - '1');
                found = true;
            }
            if (!found) {
                game::MessageChatData badArg;
                badArg.type = game::ChatType::SYSTEM;
                badArg.language = game::ChatLanguage::UNIVERSAL;
                badArg.message = "Unknown mark. Use: star circle diamond triangle moon square cross skull";
                ctx.gameHandler.addLocalChatMessage(badArg);
                return {};
            }
        }
        ctx.gameHandler.setRaidMark(ctx.gameHandler.getTargetGuid(), icon);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"mark", "marktarget", "raidtarget"}; }
    [[nodiscard]] std::string helpText() const override { return "Set raid target mark on target"; }
};

// --- /roll, /random, /rnd ---
class RollCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        uint32_t minRoll = 1;
        uint32_t maxRoll = 100;
        if (!ctx.args.empty()) {
            size_t dashPos = ctx.args.find('-');
            size_t spacePos = ctx.args.find(' ');
            if (dashPos != std::string::npos) {
                try {
                    minRoll = std::stoul(ctx.args.substr(0, dashPos));
                    maxRoll = std::stoul(ctx.args.substr(dashPos + 1));
                } catch (...) {}
            } else if (spacePos != std::string::npos) {
                try {
                    minRoll = std::stoul(ctx.args.substr(0, spacePos));
                    maxRoll = std::stoul(ctx.args.substr(spacePos + 1));
                } catch (...) {}
            } else {
                try { maxRoll = std::stoul(ctx.args); } catch (...) {}
            }
        }
        ctx.gameHandler.randomRoll(minRoll, maxRoll);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"roll", "random", "rnd"}; }
    [[nodiscard]] std::string helpText() const override { return "Random dice roll"; }
};

// --- Registration ---
void registerGroupCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<ReadyCheckCommand>());
    reg.registerCommand(std::make_unique<ReadyCommand>());
    reg.registerCommand(std::make_unique<NotReadyCommand>());
    reg.registerCommand(std::make_unique<YieldCommand>());
    reg.registerCommand(std::make_unique<AfkCommand>());
    reg.registerCommand(std::make_unique<DndCommand>());
    reg.registerCommand(std::make_unique<UninviteCommand>());
    reg.registerCommand(std::make_unique<LeavePartyCommand>());
    reg.registerCommand(std::make_unique<MainTankCommand>());
    reg.registerCommand(std::make_unique<MainAssistCommand>());
    reg.registerCommand(std::make_unique<ClearMainTankCommand>());
    reg.registerCommand(std::make_unique<ClearMainAssistCommand>());
    reg.registerCommand(std::make_unique<RaidInfoCommand>());
    reg.registerCommand(std::make_unique<RaidConvertCommand>());
    reg.registerCommand(std::make_unique<LootMethodCommand>());
    reg.registerCommand(std::make_unique<LootThresholdCommand>());
    reg.registerCommand(std::make_unique<MarkCommand>());
    reg.registerCommand(std::make_unique<RollCommand>());
}

} // namespace ui
} // namespace wowee
