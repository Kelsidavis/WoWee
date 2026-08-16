// MacroEvaluator - WoW macro conditional parser and evaluator.
// Extracted from evaluateMacroConditionals() in chat_panel_commands.cpp.
// Phase 4.4 of chat_panel_ref.md.
#pragma once

#include <cstdint>
#include <string>

namespace wowee {
namespace game { class GameHandler; }
namespace ui {

class IGameState;
class IModifierState;

/**
 * Evaluates WoW-style macro conditional expressions.
 *
 * Syntax: [cond1,cond2] Spell1; [cond3] Spell2; DefaultSpell
 *
 * The first alternative whose conditions all evaluate true is returned.
 * If no conditions match, returns "".
 *
 * @p targetOverride is set to a specific GUID if [target=X] or [@X]
 * was in the matching conditions, or left as UINT64_MAX for "use normal target".
 */
class MacroEvaluator {
public:
    MacroEvaluator(IGameState& gameState, IModifierState& modState);

    /**
     * Evaluate a macro conditional string.
     * @param rawArg         The conditional text (e.g. "[combat] Spell1; Spell2")
     * @param targetOverride Output: set to target GUID if specified, or -1
     * @return               The matched argument text, or "" if nothing matched
     */
    std::string evaluate(const std::string& rawArg, uint64_t& targetOverride) const;

private:
    /** Evaluate a single condition token (e.g. "combat", "mod:shift", "@focus"). */
    bool evalCondition(const std::string& cond, uint64_t& tgt) const;

    /** Resolve effective target GUID (follows @/target= overrides). */
    [[nodiscard]] uint64_t resolveEffectiveTarget(uint64_t tgt) const;

    IGameState& gameState_;
    IModifierState& modState_;
};

// Convenience free function - thin wrapper over MacroEvaluator.
// Used by command modules (combat_commands, system_commands, target_commands).
std::string evaluateMacroConditionals(const std::string& rawArg,
                                      game::GameHandler& gameHandler,
                                      uint64_t& targetOverride);

} // namespace ui
} // namespace wowee
