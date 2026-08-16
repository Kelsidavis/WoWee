// InputModifierAdapter - wraps core::Input to implement IModifierState.
// Phase 4.3 of chat_panel_ref.md.
#pragma once

#include "ui/chat/i_modifier_state.hpp"

namespace wowee {
namespace ui {

/**
 * Concrete adapter from core::Input → IModifierState.
 * Reads real keyboard state from SDL.
 */
class InputModifierAdapter : public IModifierState {
public:
    [[nodiscard]] bool isShiftHeld() const override;
    [[nodiscard]] bool isCtrlHeld() const override;
    [[nodiscard]] bool isAltHeld() const override;
};

} // namespace ui
} // namespace wowee
