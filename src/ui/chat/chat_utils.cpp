// chat_utils.cpp - Shared chat utility functions.
// Extracted from chat_panel_utils.cpp (Phase 6.6 of chat_panel_ref.md).

#include "ui/chat/chat_utils.hpp"
#include "game/game_handler.hpp"
#include "game/character.hpp"
#include "game/text_tokens.hpp"
#include <vector>

namespace wowee { namespace ui { namespace chat_utils {

std::string replaceGenderPlaceholders(const std::string& text,
                                       game::GameHandler& gameHandler) {
    // Moved to the game layer: the chat handler has to resolve these before a
    // line reaches the interface, and it cannot reach into the UI to do it.
    return game::resolveTextTokens(text, gameHandler);
}

namespace {

/// Where PortBot can send you, and the words that ask for it.
///
/// One table rather than a run of `if`s, because the help line is built from
/// it too: the two copies of this that existed both listed their aliases in a
/// hand-written sentence, and an alias added to the branches would not have
/// reached either sentence.
struct PortBotDestination {
    const char* shortAlias;
    const char* fullName;
};

constexpr PortBotDestination kPortBotDestinations[] = {
    {.shortAlias = "sw",    .fullName = "stormwind"},
    {.shortAlias = "if",    .fullName = "ironforge"},
    {.shortAlias = "darn",  .fullName = "darnassus"},
    {.shortAlias = "org",   .fullName = "orgrimmar"},
    {.shortAlias = "tb",    .fullName = "thunderbluff"},
    {.shortAlias = "uc",    .fullName = "undercity"},
    {.shortAlias = "shatt", .fullName = "shattrath"},
    {.shortAlias = "dal",   .fullName = "dalaran"},
};

}  // namespace

bool isPortBotTarget(const std::string& target) {
    return toLower(trim(target)) == "portbot";
}

std::string portBotCommandFor(const std::string& rawInput) {
    std::string input = trim(rawInput);
    if (input.empty()) return "";
    const std::string lower = toLower(input);
    if (lower == "help" || lower == "?") return "__help__";

    // Already a command: passed through as typed, so anything the server
    // understands and this table does not is still reachable.
    if (lower.rfind(".tele ", 0) == 0 || lower.rfind(".go ", 0) == 0) return input;
    if (lower.rfind("xyz ", 0) == 0) return ".go " + input;

    for (const auto& dest : kPortBotDestinations) {
        if (lower == dest.shortAlias || lower == dest.fullName) {
            return std::string(".tele ") + dest.fullName;
        }
    }
    // Not an alias we know: handed to the server as a destination name, which
    // is what makes every city this table does not list still work.
    return ".tele " + input;
}

std::string portBotHelpText() {
    std::string aliases;
    for (const auto& dest : kPortBotDestinations) {
        if (!aliases.empty()) aliases += " ";
        aliases += dest.shortAlias;
    }
    return "PortBot: /w PortBot <dest>. Aliases: " + aliases +
           ". Also supports '.tele ...' or 'xyz x y z [map [o]]'.";
}

} // namespace chat_utils
} // namespace ui
} // namespace wowee
