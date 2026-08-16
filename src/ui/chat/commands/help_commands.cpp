// Help commands: /help, /chathelp, /macrohelp
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"

namespace wowee { namespace ui {

// --- /help, /? ---
class HelpCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        static constexpr const char* kHelpLines[] = {
            "--- Wowee Slash Commands ---",
            "Chat: /s /y /p /g /raid /rw /o /bg /w <name> /r  /join /leave",
            "Social: /who  /friend add/remove  /ignore  /unignore",
            "Party: /invite  /uninvite  /leave  /readycheck  /mark  /roll",
            "       /maintank  /mainassist  /raidconvert  /raidinfo",
            "       /lootmethod  /lootthreshold",
            "Guild: /ginvite  /gkick  /gquit  /gpromote  /gdemote  /gmotd",
            "       /gleader  /groster  /ginfo  /gcreate  /gdisband",
            "Combat: /cast  /castsequence  /use  /startattack  /stopattack",
            "        /stopcasting  /duel  /forfeit  /pvp  /assist",
            "        /follow  /stopfollow  /threat  /combatlog",
            "Items: /use <item>  /equip <item>  /equipset [name]",
            "Target: /target  /cleartarget  /focus  /clearfocus  /inspect",
            "Movement: /sit  /stand  /kneel  /dismount",
            "Misc: /played  /time  /zone  /loc  /afk  /dnd  /helm  /cloak",
            "      /trade  /score  /unstuck  /logout  /quit  /exit  /ticket",
            "      /screenshot  /difficulty",
            "      /macrohelp  /chathelp  /help  /gmhelp",
            "GM:    .command (dot-prefix sent to server, /gmhelp for list)",
        };
        for (const char* line : kHelpLines) {
            game::MessageChatData helpMsg;
            helpMsg.type = game::ChatType::SYSTEM;
            helpMsg.language = game::ChatLanguage::UNIVERSAL;
            helpMsg.message = line;
            ctx.gameHandler.addLocalChatMessage(helpMsg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"help", "?"}; }
    [[nodiscard]] std::string helpText() const override { return "List all slash commands"; }
};

// --- /chathelp ---
class ChatHelpCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        static constexpr const char* kChatHelp[] = {
            "--- Chat Channel Commands ---",
            "/s [msg]          Say to nearby players",
            "/y [msg]          Yell to a wider area",
            "/w <name> [msg]   Whisper to player",
            "/r [msg]          Reply to last whisper",
            "/p [msg]          Party chat",
            "/g [msg]          Guild chat",
            "/o [msg]          Guild officer chat",
            "/raid [msg]       Raid chat",
            "/rw [msg]         Raid warning",
            "/bg [msg]         Battleground chat",
            "/1 [msg]          General channel",
            "/2 [msg]          Trade channel  (also /wts /wtb)",
            "/<N> [msg]        Channel by number",
            "/join <chan>      Join a channel",
            "/leave <chan>     Leave a channel",
            "/afk [msg]        Set AFK status",
            "/dnd [msg]        Set Do Not Disturb",
        };
        for (const char* line : kChatHelp) {
            game::MessageChatData helpMsg;
            helpMsg.type = game::ChatType::SYSTEM;
            helpMsg.language = game::ChatLanguage::UNIVERSAL;
            helpMsg.message = line;
            ctx.gameHandler.addLocalChatMessage(helpMsg);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"chathelp"}; }
    [[nodiscard]] std::string helpText() const override { return "List chat channel commands"; }
};

// --- /macrohelp ---
class MacroHelpCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        static constexpr const char* kMacroHelp[] = {
            "--- Macro Conditionals ---",
            "Usage: /cast [cond1,cond2] Spell1; [cond3] Spell2; Default",
            "State:   [combat] [mounted] [swimming] [flying] [stealthed]",
            "         [channeling] [pet] [group] [raid] [indoors] [outdoors]",
            "Spec:    [spec:1] [spec:2]  (active talent spec, 1-based)",
            "         (prefix no- to negate any condition)",
            "Target:  [harm] [help] [exists] [noexists] [dead] [nodead]",
            "         [target=focus] [target=pet] [target=mouseover] [target=player]",
            "         (also: @focus, @pet, @mouseover, @player, @target)",
            "Form:    [noform] [nostance] [form:0]",
            "Keys:    [mod:shift] [mod:ctrl] [mod:alt]",
            "Aura:    [buff:Name] [nobuff:Name] [debuff:Name] [nodebuff:Name]",
            "Other:   #showtooltip, /stopmacro [cond], /castsequence",
        };
        for (const char* line : kMacroHelp) {
            game::MessageChatData m;
            m.type = game::ChatType::SYSTEM;
            m.language = game::ChatLanguage::UNIVERSAL;
            m.message = line;
            ctx.gameHandler.addLocalChatMessage(m);
        }
        return {};
    }
    [[nodiscard]] std::vector<std::string> aliases() const override { return {"macrohelp"}; }
    [[nodiscard]] std::string helpText() const override { return "List macro conditionals"; }
};

// --- Registration ---
void registerHelpCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<HelpCommand>());
    reg.registerCommand(std::make_unique<ChatHelpCommand>());
    reg.registerCommand(std::make_unique<MacroHelpCommand>());
}

} // namespace ui
} // namespace wowee
