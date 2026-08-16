// Misc commands: /time, /loc, /zone, /played, /screenshot, /ticket, /score,
//                /threat, /combatlog, /helm, /cloak, /follow, /stopfollow,
//                /assist, /pvp, /unstuck*, /transport
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include "game/transport_manager.hpp"
#include "game/entity.hpp"
#include "rendering/renderer.hpp"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <limits>
#include <memory>
#include <glm/vec3.hpp>

namespace wowee { namespace ui {

// Forward declaration of evaluateMacroConditionals (still in chat_panel_commands.cpp)
std::string evaluateMacroConditionals(const std::string& rawArg,
                                       game::GameHandler& gameHandler,
                                       uint64_t& targetOverride);

// --- /transportinfo ---
// Dumps active transport state (entry, path, clock mode, position) so
// stationary/invisible transports can be diagnosed live instead of guessing.
class TransportInfoCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        auto say = [&ctx](const std::string& text) {
            game::MessageChatData m;
            m.type = game::ChatType::SYSTEM;
            m.language = game::ChatLanguage::UNIVERSAL;
            m.message = text;
            ctx.gameHandler.addLocalChatMessage(m);
        };
        auto* tm = ctx.gameHandler.getTransportManager();
        if (!tm) { say("Transport manager unavailable."); return {}; }
        const auto& transports = tm->getTransports();
        say("Active transports: " + std::to_string(transports.size()));
        for (const auto& [guid, t] : transports) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "entry=%u disp=%u path=%u pos=(%.1f, %.1f, %.1f) %s clock=%s updates=%d inst=%u%s",
                     t.entry, t.displayId, t.pathId,
                     t.position.x, t.position.y, t.position.z,
                     t.isM2 ? "M2" : "WMO",
                     t.useClientAnimation ? (t.hasServerClock ? "client+sync" : "client")
                                          : "server",
                     t.serverUpdateCount,
                     t.wmoInstanceId,
                     t.wmoInstanceId == 0 ? " (NO RENDER INSTANCE)" : "");
            say(buf);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"transportinfo"}; }
    [[nodiscard]] std::string helpText() const override { return "List active transports and their state"; }
};

// --- /time ---
class TimeCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.queryServerTime(true);  // typed by the player, so say it
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"time"}; }
    [[nodiscard]] std::string helpText() const override { return "Query server time"; }
};

// --- /loc, /coords, /whereami ---
class LocCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        const auto& pmi = ctx.gameHandler.getMovementInfo();
        std::string zoneName;
        if (auto* rend = ctx.services.renderer)
            zoneName = rend->getCurrentZoneName();
        char buf[256];
        snprintf(buf, sizeof(buf), "%.1f, %.1f, %.1f%s%s",
                 pmi.x, pmi.y, pmi.z,
                 zoneName.empty() ? "" : " - ",
                 zoneName.c_str());
        game::MessageChatData sysMsg;
        sysMsg.type = game::ChatType::SYSTEM;
        sysMsg.language = game::ChatLanguage::UNIVERSAL;
        sysMsg.message = buf;
        ctx.gameHandler.addLocalChatMessage(sysMsg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"loc", "coords", "whereami"}; }
    [[nodiscard]] std::string helpText() const override { return "Print player coordinates"; }
};

// --- /zone ---
class ZoneCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        std::string zoneName;
        if (auto* rend = ctx.services.renderer)
            zoneName = rend->getCurrentZoneName();
        game::MessageChatData sysMsg;
        sysMsg.type = game::ChatType::SYSTEM;
        sysMsg.language = game::ChatLanguage::UNIVERSAL;
        sysMsg.message = zoneName.empty() ? "You are not in a known zone." : "You are in: " + zoneName;
        ctx.gameHandler.addLocalChatMessage(sysMsg);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"zone"}; }
    [[nodiscard]] std::string helpText() const override { return "Show current zone"; }
};

// --- /played ---
class PlayedCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestPlayedTime();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"played"}; }
    [[nodiscard]] std::string helpText() const override { return "Show time played"; }
};

// --- /screenshot, /ss ---
class ScreenshotCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.getSlashCmds().takeScreenshot = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"screenshot", "ss"}; }
    [[nodiscard]] std::string helpText() const override { return "Take a screenshot"; }
};

// --- /ticket, /gmticket, /gm ---
class TicketCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.getSlashCmds().showGmTicket = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"ticket", "gmticket", "gm"}; }
    [[nodiscard]] std::string helpText() const override { return "Open GM ticket"; }
};

// --- /score ---
class ScoreCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestPvpLog();
        ctx.panel.getSlashCmds().showBgScore = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"score"}; }
    [[nodiscard]] std::string helpText() const override { return "Show BG scoreboard"; }
};

// --- /threat ---
class ThreatCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.getSlashCmds().toggleThreat = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"threat"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle threat display"; }
};

// --- /combatlog, /cl ---
class CombatLogCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.panel.getSlashCmds().toggleCombatLog = true;
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"combatlog", "cl"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle combat log"; }
};

// --- /helm, /helmet, /showhelm ---
class HelmCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.toggleHelm();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"helm", "helmet", "showhelm"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle helmet visibility"; }
};

// --- /cloak, /showcloak ---
class CloakCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.toggleCloak();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"cloak", "showcloak"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle cloak visibility"; }
};

// --- /follow, /f ---
class FollowCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.followTarget();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"follow", "f"}; }
    [[nodiscard]] std::string helpText() const override { return "Walk toward and camera-follow your current target"; }
};

// --- /stopfollow ---
class StopFollowCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.cancelFollow();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"stopfollow"}; }
    [[nodiscard]] std::string helpText() const override { return "Stop following"; }
};

// --- /assist ---
class AssistCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        auto assistEntityTarget = [&](uint64_t srcGuid) {
            auto srcEnt = ctx.gameHandler.getEntityManager().getEntity(srcGuid);
            if (!srcEnt) { ctx.gameHandler.assistTarget(); return; }
            const uint64_t atkGuid = game::unitTargetGuid(*srcEnt);
            if (atkGuid != 0) {
                ctx.gameHandler.setTarget(atkGuid);
            } else {
                std::string sn = game::entityDisplayName(srcEnt);
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = (sn.empty() ? "Target" : sn) + " has no target.";
                ctx.gameHandler.addLocalChatMessage(msg);
            }
        };

        if (!ctx.args.empty()) {
            std::string assistArg = ctx.args;
            while (!assistArg.empty() && assistArg.front() == ' ') assistArg.erase(assistArg.begin());

            // Evaluate conditionals if present
            uint64_t assistOver = static_cast<uint64_t>(-1);
            if (!assistArg.empty() && assistArg.front() == '[') {
                assistArg = evaluateMacroConditionals(assistArg, ctx.gameHandler, assistOver);
                if (assistArg.empty() && assistOver == static_cast<uint64_t>(-1)) return {};
                while (!assistArg.empty() && assistArg.front() == ' ') assistArg.erase(assistArg.begin());
                while (!assistArg.empty() && assistArg.back()  == ' ') assistArg.pop_back();
            }

            if (assistOver != static_cast<uint64_t>(-1) && assistOver != 0) {
                assistEntityTarget(assistOver);
            } else if (!assistArg.empty()) {
                // Name search
                std::string argLow = assistArg;
                for (char& c : argLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                uint64_t bestGuid = 0; float bestDist = std::numeric_limits<float>::max();
                const auto& pmi = ctx.gameHandler.getMovementInfo();
                for (const auto& [guid, ent] : ctx.gameHandler.getEntityManager().getEntities()) {
                    if (!ent || ent->getType() == game::ObjectType::OBJECT) continue;
                    std::string nm = game::entityDisplayName(ent);
                    std::string nml = nm;
                    for (char& c : nml) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (nml.find(argLow) != 0) continue;
                    float d2 = (ent->getX()-pmi.x)*(ent->getX()-pmi.x)
                             + (ent->getY()-pmi.y)*(ent->getY()-pmi.y);
                    if (d2 < bestDist) { bestDist = d2; bestGuid = guid; }
                }
                if (bestGuid) assistEntityTarget(bestGuid);
                else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "No unit matching '" + assistArg + "' found.";
                    ctx.gameHandler.addLocalChatMessage(msg);
                }
            } else {
                ctx.gameHandler.assistTarget();
            }
        } else {
            ctx.gameHandler.assistTarget();
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"assist"}; }
    [[nodiscard]] std::string helpText() const override { return "Assist target (target their target)"; }
};

// --- /pvp ---
class PvpCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.togglePvp();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"pvp"}; }
    [[nodiscard]] std::string helpText() const override { return "Toggle PvP flag"; }
};

// --- /unstuck ---
class UnstuckCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.unstuck();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"unstuck"}; }
    [[nodiscard]] std::string helpText() const override { return "Reset position to floor height"; }
};

// --- /unstuckgy ---
class UnstuckGyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.unstuckGy();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"unstuckgy"}; }
    [[nodiscard]] std::string helpText() const override { return "Move to nearest graveyard"; }
};

// --- /unstuckhearth ---
class UnstuckHearthCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.unstuckHearth();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"unstuckhearth"}; }
    [[nodiscard]] std::string helpText() const override { return "Teleport to hearthstone bind point"; }
};

// --- /transport board ---
class TransportBoardCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        // This is invoked via the "transport" alias. Check args for sub-command.
        std::string sub = ctx.args;
        while (!sub.empty() && sub.front() == ' ') sub.erase(sub.begin());
        for (char& c : sub) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        if (sub == "board") {
            auto* tm = ctx.gameHandler.getTransportManager();
            if (tm) {
                uint64_t testTransportGuid = 0x1000000000000001ULL;
                glm::vec3 deckCenter(0.0f, 0.0f, 5.0f);
                ctx.gameHandler.setPlayerOnTransport(testTransportGuid, deckCenter);
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Boarded test transport. Use '/transport leave' to disembark.";
                ctx.gameHandler.addLocalChatMessage(msg);
            } else {
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Transport system not available.";
                ctx.gameHandler.addLocalChatMessage(msg);
            }
            return {};
        } if (sub == "leave") {
            if (ctx.gameHandler.isOnTransport()) {
                ctx.gameHandler.clearPlayerTransport();
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Disembarked from transport.";
                ctx.gameHandler.addLocalChatMessage(msg);
            } else {
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "You are not on a transport.";
                ctx.gameHandler.addLocalChatMessage(msg);
            }
            return {};
        }
        // Unrecognized sub-command
        return {.handled = false, .clearInput = false};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"transport"}; }
    [[nodiscard]] std::string helpText() const override { return "Transport: /transport board|leave"; }
};

// --- Registration ---
void registerMiscCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<TimeCommand>());
    reg.registerCommand(std::make_unique<LocCommand>());
    reg.registerCommand(std::make_unique<ZoneCommand>());
    reg.registerCommand(std::make_unique<PlayedCommand>());
    reg.registerCommand(std::make_unique<ScreenshotCommand>());
    reg.registerCommand(std::make_unique<TicketCommand>());
    reg.registerCommand(std::make_unique<ScoreCommand>());
    reg.registerCommand(std::make_unique<ThreatCommand>());
    reg.registerCommand(std::make_unique<CombatLogCommand>());
    reg.registerCommand(std::make_unique<HelmCommand>());
    reg.registerCommand(std::make_unique<CloakCommand>());
    reg.registerCommand(std::make_unique<FollowCommand>());
    reg.registerCommand(std::make_unique<StopFollowCommand>());
    reg.registerCommand(std::make_unique<AssistCommand>());
    reg.registerCommand(std::make_unique<PvpCommand>());
    reg.registerCommand(std::make_unique<UnstuckCommand>());
    reg.registerCommand(std::make_unique<UnstuckGyCommand>());
    reg.registerCommand(std::make_unique<UnstuckHearthCommand>());
    reg.registerCommand(std::make_unique<TransportBoardCommand>());
    reg.registerCommand(std::make_unique<TransportInfoCommand>());
}

} // namespace ui
} // namespace wowee
