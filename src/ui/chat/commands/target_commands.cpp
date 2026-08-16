// Target commands: /target, /cleartarget, /targetenemy, /targetfriend,
//                  /targetlasttarget, /targetlastenemy, /targetlastfriend,
//                  /focus, /clearfocus
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat/macro_evaluator.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include "game/entity.hpp"
#include <algorithm>
#include <cctype>
#include <limits>
#include <memory>

namespace wowee { namespace ui {

namespace {

// Trim leading/trailing whitespace.
inline void trimInPlace(std::string& s) {
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back()  == ' ') s.pop_back();
}

// Search nearby visible entities by name (case-insensitive prefix match).
// Returns the GUID of the nearest matching unit, or 0 if none found.
uint64_t findNearestByName(game::GameHandler& gh, const std::string& targetArgLower) {
    uint64_t bestGuid = 0;
    float bestDist = std::numeric_limits<float>::max();
    const auto& pmi = gh.getMovementInfo();
    for (const auto& [guid, entity] : gh.getEntityManager().getEntities()) {
        if (!entity || entity->getType() == game::ObjectType::OBJECT) continue;
        std::string name;
        if (entity->getType() == game::ObjectType::PLAYER ||
            entity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            name = unit->getName();
        }
        if (name.empty()) continue;
        std::string nameLower = name;
        for (char& c : nameLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (nameLower.find(targetArgLower) == 0) {
            float dx = entity->getX() - pmi.x;
            float dy = entity->getY() - pmi.y;
            float dz = entity->getZ() - pmi.z;
            float dist = dx*dx + dy*dy + dz*dz;
            if (dist < bestDist) {
                bestDist = dist;
                bestGuid = guid;
            }
        }
    }
    return bestGuid;
}

} // anon namespace

// --- /target ---
class TargetCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {.handled = false, .clearInput = false};

        std::string targetArg = ctx.args;

        // Evaluate conditionals if present
        uint64_t targetCmdOverride = static_cast<uint64_t>(-1);
        if (!targetArg.empty() && targetArg.front() == '[') {
            targetArg = evaluateMacroConditionals(targetArg, ctx.gameHandler, targetCmdOverride);
            if (targetArg.empty() && targetCmdOverride == static_cast<uint64_t>(-1)) return {};
            trimInPlace(targetArg);
        }

        // If conditionals resolved to a specific GUID, target it directly
        if (targetCmdOverride != static_cast<uint64_t>(-1) && targetCmdOverride != 0) {
            ctx.gameHandler.setTarget(targetCmdOverride);
            return {};
        }

        if (targetArg.empty()) return {};

        std::string targetArgLower = targetArg;
        for (char& c : targetArgLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        uint64_t bestGuid = findNearestByName(ctx.gameHandler, targetArgLower);
        if (bestGuid) {
            ctx.gameHandler.setTarget(bestGuid);
        } else {
            game::MessageChatData sysMsg;
            sysMsg.type = game::ChatType::SYSTEM;
            sysMsg.language = game::ChatLanguage::UNIVERSAL;
            sysMsg.message = "No target matching '" + targetArg + "' found.";
            ctx.gameHandler.addLocalChatMessage(sysMsg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"target"}; }
    [[nodiscard]] std::string helpText() const override { return "Target unit by name"; }
};

// --- /cleartarget ---
class ClearTargetCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        bool condPass = true;
        if (!ctx.args.empty()) {
            std::string ctArg = ctx.args;
            trimInPlace(ctArg);
            if (!ctArg.empty() && ctArg.front() == '[') {
                uint64_t ctOver = static_cast<uint64_t>(-1);
                std::string res = evaluateMacroConditionals(ctArg, ctx.gameHandler, ctOver);
                condPass = !(res.empty() && ctOver == static_cast<uint64_t>(-1));
            }
        }
        if (condPass) ctx.gameHandler.clearTarget();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"cleartarget"}; }
    [[nodiscard]] std::string helpText() const override { return "Clear current target"; }
};

// --- /targetenemy ---
class TargetEnemyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.targetEnemy(false);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"targetenemy"}; }
    [[nodiscard]] std::string helpText() const override { return "Cycle to next enemy"; }
};

// --- /targetfriend ---
class TargetFriendCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.targetFriend(false);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"targetfriend"}; }
    [[nodiscard]] std::string helpText() const override { return "Cycle to next friendly unit"; }
};

// --- /targetlasttarget, /targetlast ---
class TargetLastTargetCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.targetLastTarget();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"targetlasttarget", "targetlast"}; }
    [[nodiscard]] std::string helpText() const override { return "Target previous target"; }
};

// --- /targetlastenemy ---
class TargetLastEnemyCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.targetEnemy(true);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"targetlastenemy"}; }
    [[nodiscard]] std::string helpText() const override { return "Cycle to previous enemy"; }
};

// --- /targetlastfriend ---
class TargetLastFriendCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.targetFriend(true);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"targetlastfriend"}; }
    [[nodiscard]] std::string helpText() const override { return "Cycle to previous friendly unit"; }
};

// --- /focus ---
class FocusCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (!ctx.args.empty()) {
            std::string focusArg = ctx.args;

            // Evaluate conditionals if present
            uint64_t focusCmdOverride = static_cast<uint64_t>(-1);
            if (!focusArg.empty() && focusArg.front() == '[') {
                focusArg = evaluateMacroConditionals(focusArg, ctx.gameHandler, focusCmdOverride);
                if (focusArg.empty() && focusCmdOverride == static_cast<uint64_t>(-1)) return {};
                trimInPlace(focusArg);
            }

            if (focusCmdOverride != static_cast<uint64_t>(-1) && focusCmdOverride != 0) {
                ctx.gameHandler.setFocus(focusCmdOverride);
            } else if (!focusArg.empty()) {
                std::string focusArgLower = focusArg;
                for (char& c : focusArgLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                uint64_t bestGuid = findNearestByName(ctx.gameHandler, focusArgLower);
                if (bestGuid) {
                    ctx.gameHandler.setFocus(bestGuid);
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "No unit matching '" + focusArg + "' found.";
                    ctx.gameHandler.addLocalChatMessage(msg);
                }
            }
        } else if (ctx.gameHandler.hasTarget()) {
            ctx.gameHandler.setFocus(ctx.gameHandler.getTargetGuid());
        } else {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must target a unit to set as focus.";
            ctx.gameHandler.addLocalChatMessage(msg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"focus"}; }
    [[nodiscard]] std::string helpText() const override { return "Set focus target"; }
};

// --- /clearfocus ---
class ClearFocusCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.clearFocus();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"clearfocus"}; }
    [[nodiscard]] std::string helpText() const override { return "Clear focus target"; }
};

// --- Registration ---
void registerTargetCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<TargetCommand>());
    reg.registerCommand(std::make_unique<ClearTargetCommand>());
    reg.registerCommand(std::make_unique<TargetEnemyCommand>());
    reg.registerCommand(std::make_unique<TargetFriendCommand>());
    reg.registerCommand(std::make_unique<TargetLastTargetCommand>());
    reg.registerCommand(std::make_unique<TargetLastEnemyCommand>());
    reg.registerCommand(std::make_unique<TargetLastFriendCommand>());
    reg.registerCommand(std::make_unique<FocusCommand>());
    reg.registerCommand(std::make_unique<ClearFocusCommand>());
}

} // namespace ui
} // namespace wowee
