#pragma once

/// SDL text input started with the platform's typing assists switched off.
///
/// SDL's plain SDL_StartTextInput passes no properties at all, and SDL's
/// defaults for the two that matter are autocorrect on and capitalization
/// by sentence - the right behaviour for a text editor and wrong for a
/// game. On iPadOS autocorrect rewrites what the player types in chat
/// while the hidden UIKit text field behind SDL's text input has focus.
///
/// SDL applies these on every SDL_StartTextInputWithProperties call, even
/// while input is already running, which is the only public way to correct
/// a session someone else started plain - the ImGui backend starts one with
/// SDL_StartTextInput and no properties. SDL copies the properties it is
/// given, so the ID returned here outlives the call; it is built once and
/// never destroyed because every caller wants the same two answers.

#include <SDL3/SDL.h>

namespace wowee::core {

/// Properties for text input with no autocorrect, no spell checking and no
/// auto-capitalization.
inline SDL_PropertiesID textInputNoAssistProperties() {
    static const SDL_PropertiesID props = [] {
        SDL_PropertiesID p = SDL_CreateProperties();
        if (p) {
            SDL_SetBooleanProperty(p, SDL_PROP_TEXTINPUT_AUTOCORRECT_BOOLEAN, false);
            SDL_SetNumberProperty(p, SDL_PROP_TEXTINPUT_CAPITALIZATION_NUMBER,
                                  SDL_CAPITALIZE_NONE);
        }
        return p;
    }();
    return props;
}

}  // namespace wowee::core
