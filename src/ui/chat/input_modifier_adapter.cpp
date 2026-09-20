// InputModifierAdapter - concrete IModifierState wrapping core::Input.
// Phase 4.3 of chat_panel_ref.md.
#include "ui/chat/input_modifier_adapter.hpp"
#include "core/input.hpp"
#include <SDL3/SDL_scancode.h>

namespace wowee { namespace ui {

bool InputModifierAdapter::isShiftHeld() const {
    auto& input = core::Input::getInstance();
    return input.isKeyPressed(SDL_SCANCODE_LSHIFT) ||
           input.isKeyPressed(SDL_SCANCODE_RSHIFT);
}

bool InputModifierAdapter::isCtrlHeld() const {
    auto& input = core::Input::getInstance();
    return input.isKeyPressed(SDL_SCANCODE_LCTRL) ||
           input.isKeyPressed(SDL_SCANCODE_RCTRL);
}

bool InputModifierAdapter::isAltHeld() const {
    auto& input = core::Input::getInstance();
    return input.isKeyPressed(SDL_SCANCODE_LALT) ||
           input.isKeyPressed(SDL_SCANCODE_RALT);
}

} // namespace ui
} // namespace wowee
