// GM commands: /gmhelp, /gmcommands - local help for server-side dot-prefix commands.
// Also provides the gm_commands::getCompletions() function used by tab-completion.
// The actual GM commands (.gm, .tele, etc.) are sent to the server as SAY messages;
// the server (AzerothCore) does the real work.  This file just adds discoverability.
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat/chat_command_registry.hpp"
#include "ui/chat/gm_command_data.hpp"
#include "ui/chat/chat_utils.hpp"
#include "game/game_handler.hpp"
#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace wowee { namespace ui {

// ---------------------------------------------------------------------------
// gm_commands namespace - GM command lookup helpers used by tab-completion
// and the /gmhelp command.
// ---------------------------------------------------------------------------
namespace gm_commands {

std::vector<std::string> getCompletions(const std::string& prefix) {
    // The logic lives with the table, in gm_command_data.hpp, so a test can
    // reach it without linking this file and the game handler behind it.
    return gmCompletionsFor(prefix);
}

const GmCommandEntry* find(const std::string& name) {
    for (const auto& cmd : kGmCommands) {
        if (cmd.name == name) return &cmd;
    }
    return nullptr;
}

} // namespace gm_commands

// ---------------------------------------------------------------------------
// /gmhelp [filter] - display GM command reference locally.
// ---------------------------------------------------------------------------
class GmHelpCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        std::string filter = ctx.args;
        for (char& c : filter) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        // Trim leading/trailing whitespace
        while (!filter.empty() && std::isspace(static_cast<unsigned char>(filter.front()))) filter.erase(filter.begin());
        while (!filter.empty() && std::isspace(static_cast<unsigned char>(filter.back()))) filter.pop_back();

        // If filter matches a specific command name, show detailed help
        if (!filter.empty()) {
            // Strip leading dot if user typed /gmhelp .gm
            if (filter.front() == '.') filter = filter.substr(1);

            bool found = false;
            for (const auto& cmd : kGmCommands) {
                std::string name(cmd.name);
                if (name == filter || name.compare(0, filter.size(), filter) == 0) {
                    std::string line = std::string(cmd.syntax) + "  - " + std::string(cmd.help)
                        + "  [sec:" + std::to_string(cmd.security) + "]";
                    ctx.gameHandler.addLocalChatMessage(chat_utils::makeSystemMessage(line));
                    found = true;
                }
            }
            if (!found) {
                ctx.gameHandler.addLocalChatMessage(
                    chat_utils::makeSystemMessage("No GM commands matching '" + filter + "'."));
            }
            return {};
        }

        // No filter - print category overview
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("--- GM Commands (dot-prefix, sent to server) ---"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("GM Mode:   .gm on/off  .gm fly  .gm visible"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Teleport:  .tele <loc>  .go xyz  .appear  .summon"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Character: .levelup  .additem  .learn  .maxskill  .pinfo"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Combat:    .revive  .die  .damage  .freeze  .respawn"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Modify:    .modify money/hp/mana/speed  .morph  .modify scale"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Cheats:    .cheat god/casttime/cooldown/power/taxi/explore"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Spells:    .cast  .aura  .unaura  .cooldown  .setskill"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Quests:    .quest add/complete/remove/reward/status"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("NPC:       .npc add/delete/info/near/say/move"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Objects:   .gobject add/delete/info/near/target"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Lookup:    .lookup item/spell/creature/quest/area/teleport"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Admin:     .ban  .kick  .mute  .announce  .reload"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Server:    .server info  .server motd  .save  .commands  .help"));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Use /gmhelp <command> for details (e.g. /gmhelp tele)."));
        ctx.gameHandler.addLocalChatMessage(
            chat_utils::makeSystemMessage("Tab-complete works with dot-prefix (type .te<Tab>)."));
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"gmhelp", "gmcommands"}; }
    [[nodiscard]] std::string helpText() const override { return "List GM dot-commands (server-side)"; }
};

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------
void registerGmCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<GmHelpCommand>());
}

} // namespace ui
} // namespace wowee
