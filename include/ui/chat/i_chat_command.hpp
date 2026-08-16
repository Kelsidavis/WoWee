// IChatCommand - interface for all slash commands.
// Phase 3.1 of chat_panel_ref.md.
#pragma once

#include <string>
#include <vector>

namespace wowee {

// Forward declarations
namespace game { class GameHandler; }
namespace ui { struct UIServices; class ChatPanel; }

namespace ui {

/**
 * Context passed to every command's execute() method.
 * Provides everything a command needs without coupling to ChatPanel.
 */
struct ChatCommandContext {
    game::GameHandler&  gameHandler;
    UIServices&         services;
    ChatPanel&          panel;          // for input buffer access, macro state
    std::string         args;           // everything after "/cmd "
    std::string         fullCommand;    // the original command name (lowercase)
};

/**
 * Result returned by a command to tell the dispatcher what to do next.
 */
struct ChatCommandResult {
    bool handled = true;           // false → command not recognized, fall through
    bool clearInput = true;        // clear the input buffer after execution
};

/**
 * Interface for all chat slash commands.
 *
 * Adding a new command = create a class implementing this interface,
 * register it in ChatCommandRegistry. Zero edits to existing code. (OCP)
 */
class IChatCommand {
public:
    virtual ~IChatCommand() = default;

    /** Execute the command. */
    virtual ChatCommandResult execute(ChatCommandContext& ctx) = 0;

    /** Return all aliases for this command (e.g. {"w", "whisper", "tell", "t"}). */
    [[nodiscard]] virtual std::vector<std::string> aliases() const = 0;

    /** Optional help text shown by /help. */
    [[nodiscard]] virtual std::string helpText() const { return ""; }
};

} // namespace ui
} // namespace wowee
