// Emote/stance commands: /sit, /stand, /kneel, /dismount, /cancelform,
//                        /cancelaura, /cancellogout, /logout, /camp, /quit, /exit,
//                        pet commands (/petattack, /petfollow, etc.)
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include "game/pet_action.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include <algorithm>
#include <cctype>

namespace wowee { namespace ui {

// --- /sit ---
class SitCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.setStandState(1);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"sit"}; }
    [[nodiscard]] std::string helpText() const override { return "Sit down"; }
};

// --- /stand ---
class StandCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.setStandState(0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"stand"}; }
    [[nodiscard]] std::string helpText() const override { return "Stand up"; }
};

// --- /kneel ---
class KneelCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.setStandState(8);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"kneel"}; }
    [[nodiscard]] std::string helpText() const override { return "Kneel"; }
};

// --- /logout, /camp ---
class LogoutEmoteCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestLogout(false);
        return {};
    }
    // aliases() is the complete name list - "logout" itself was missing here, so
    // /logout was never actually a command despite /help advertising it.
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"logout", "camp"}; }
    [[nodiscard]] std::string helpText() const override { return "Logout to character select"; }
};

// --- /quit, /exit ---
// Same logout handshake as /logout, but leaves the game once the server confirms
// the character is out of the world, rather than returning to character select.
class QuitCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.requestLogout(true);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"quit", "exit"}; }
    [[nodiscard]] std::string helpText() const override { return "Logout and quit the game"; }
};

// --- /cancellogout ---
class CancelLogoutCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.cancelLogout();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"cancellogout"}; }
    [[nodiscard]] std::string helpText() const override { return "Cancel pending logout"; }
};

// --- /dismount ---
class DismountCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.dismount();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"dismount"}; }
    [[nodiscard]] std::string helpText() const override { return "Dismount"; }
};

// --- /cancelform, /cancelshapeshift ---
class CancelFormCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        for (const auto& aura : ctx.gameHandler.getPlayerAuras()) {
            if (aura.spellId == 0) continue;
            if (aura.flags & 0x20) {
                ctx.gameHandler.cancelAura(aura.spellId);
                break;
            }
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"cancelform", "cancelshapeshift"}; }
    [[nodiscard]] std::string helpText() const override { return "Cancel shapeshift form"; }
};

// --- /cancelaura ---
class CancelAuraCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {.handled = false, .clearInput = false};
        std::string auraArg = ctx.args;
        while (!auraArg.empty() && auraArg.front() == ' ') auraArg.erase(auraArg.begin());
        while (!auraArg.empty() && auraArg.back()  == ' ') auraArg.pop_back();
        // Try numeric ID first
        {
            std::string numStr = auraArg;
            if (!numStr.empty() && numStr.front() == '#') numStr.erase(numStr.begin());
            bool isNum = !numStr.empty() &&
                std::all_of(numStr.begin(), numStr.end(),
                            [](unsigned char c){ return std::isdigit(c); });
            if (isNum) {
                uint32_t spellId = 0;
                try { spellId = static_cast<uint32_t>(std::stoul(numStr)); } catch (...) {}
                if (spellId) ctx.gameHandler.cancelAura(spellId);
                return {};
            }
        }
        // Name match against player auras
        std::string argLow = auraArg;
        for (char& c : argLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for (const auto& aura : ctx.gameHandler.getPlayerAuras()) {
            if (aura.spellId == 0) continue;
            std::string sn = ctx.gameHandler.getSpellName(aura.spellId);
            for (char& c : sn) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (sn == argLow) {
                ctx.gameHandler.cancelAura(aura.spellId);
                break;
            }
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"cancelaura"}; }
    [[nodiscard]] std::string helpText() const override { return "Cancel a specific aura/buff"; }
};

// --- Pet commands ---
class PetAttackCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        uint64_t target = ctx.gameHandler.hasTarget() ? ctx.gameHandler.getTargetGuid() : 0;
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kAttack), target);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petattack"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: attack target"; }
};

class PetFollowCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kFollow), 0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petfollow"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: follow owner"; }
};

class PetStayCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kStay), 0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petstay", "pethalt"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: stay"; }
};

class PetPassiveCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kPassive), 0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petpassive"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: passive mode"; }
};

class PetDefensiveCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kDefensive), 0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petdefensive"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: defensive mode"; }
};

class PetAggressiveCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.sendPetAction(
            game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kAggressive), 0);
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petaggressive"}; }
    [[nodiscard]] std::string helpText() const override { return "Pet: aggressive mode"; }
};

class PetDismissCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.dismissPet();
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"petdismiss"}; }
    [[nodiscard]] std::string helpText() const override { return "Dismiss pet"; }
};

// --- Registration ---
void registerEmoteCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<SitCommand>());
    reg.registerCommand(std::make_unique<StandCommand>());
    reg.registerCommand(std::make_unique<KneelCommand>());
    reg.registerCommand(std::make_unique<LogoutEmoteCommand>());
    reg.registerCommand(std::make_unique<QuitCommand>());
    reg.registerCommand(std::make_unique<CancelLogoutCommand>());
    reg.registerCommand(std::make_unique<DismountCommand>());
    reg.registerCommand(std::make_unique<CancelFormCommand>());
    reg.registerCommand(std::make_unique<CancelAuraCommand>());
    reg.registerCommand(std::make_unique<PetAttackCommand>());
    reg.registerCommand(std::make_unique<PetFollowCommand>());
    reg.registerCommand(std::make_unique<PetStayCommand>());
    reg.registerCommand(std::make_unique<PetPassiveCommand>());
    reg.registerCommand(std::make_unique<PetDefensiveCommand>());
    reg.registerCommand(std::make_unique<PetAggressiveCommand>());
    reg.registerCommand(std::make_unique<PetDismissCommand>());
}

} // namespace ui
} // namespace wowee
