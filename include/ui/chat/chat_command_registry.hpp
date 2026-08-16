// ChatCommandRegistry - command registration + dispatch.
// Replaces the 500-line if/else chain in sendChatMessage() (Phase 3.1).
#pragma once

#include "ui/chat/i_chat_command.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace ui {

/**
 * Registry of all slash commands.
 *
 * dispatch() looks up the command by alias and calls execute().
 * getCompletions() provides tab-completion for /command prefixes.
 */
class ChatCommandRegistry {
public:
    /** Register a command (takes ownership). All aliases are mapped. */
    void registerCommand(std::unique_ptr<IChatCommand> cmd);

    /**
     * Dispatch a slash command.
     * @param cmdLower  lowercase command name (e.g. "cast", "whisper")
     * @param ctx       context with args, gameHandler, services, etc.
     * @return result indicating if handled and whether to clear input
     */
    ChatCommandResult dispatch(const std::string& cmdLower, ChatCommandContext& ctx);

    /** Get all command aliases matching a prefix (for tab completion). */
    [[nodiscard]] std::vector<std::string> getCompletions(const std::string& prefix) const;



private:
    // alias → raw pointer (non-owning, commands_ owns the objects)
    std::unordered_map<std::string, IChatCommand*> commandMap_;
    // Ownership of all registered commands
    std::vector<std::unique_ptr<IChatCommand>> commands_;
};

} // namespace ui
} // namespace wowee
