// ChatCommandRegistry - command registration + dispatch.
// Moved from the if/else chain in ChatPanel::sendChatMessage() (Phase 3.1).
#include "ui/chat/chat_command_registry.hpp"
#include <algorithm>

#include "core/logger.hpp"

namespace wowee { namespace ui {

void ChatCommandRegistry::registerCommand(std::unique_ptr<IChatCommand> cmd) {
    IChatCommand* raw = cmd.get();
    for (const auto& alias : raw->aliases()) {
        // Two commands claiming one name is always a mistake, and the one that
        // wins depends on nothing more than the order the register functions are
        // called in. /logout was claimed twice - once talking to the server and
        // once tearing the world down locally - and which you got was decided by
        // a line ordering in registerAllCommands().
        auto [it, inserted] = commandMap_.try_emplace(alias, raw);
        if (!inserted) {
            core::Logger::getInstance().error(
                "Chat command /", alias, " is registered twice; keeping the first. "
                "Its help text is \"", raw->helpText(), "\"");
            continue;
        }
    }
    commands_.push_back(std::move(cmd));
}

ChatCommandResult ChatCommandRegistry::dispatch(const std::string& cmdLower,
                                                  ChatCommandContext& ctx) {
    auto it = commandMap_.find(cmdLower);
    if (it == commandMap_.end()) {
        return {.handled = false, .clearInput = false};  // not handled
    }
    return it->second->execute(ctx);
}

std::vector<std::string> ChatCommandRegistry::getCompletions(const std::string& prefix) const {
    std::vector<std::string> results;
    for (const auto& [alias, cmd] : commandMap_) {
        if (alias.size() >= prefix.size() &&
            alias.compare(0, prefix.size(), prefix) == 0) {
            results.push_back(alias);
        }
    }
    std::sort(results.begin(), results.end());
    return results;
}
} // namespace ui
} // namespace wowee
