#include "ui/gamepad_controls.hpp"

#include "core/gamepad.hpp"

#include <span>
#include <string>

namespace wowee {
namespace ui {

namespace {

// The scheme. Read down the list and it is the whole of what a pad's buttons
// do, apart from Escape - which is on B and on Start, and goes through ImGui
// rather than through a scancode, because that is where the interface reads
// it.
//
// On its own in this file, and not beside the code that applies it, so that a
// test can check the table without linking the camera and ImGui behind it. It
// is data; the only questions worth asking of it - is a button bound twice, is
// a key, is every key one the client actually polls - are questions about the
// data alone.
constexpr PadBinding kBindings[] = {
    {SDL_GAMEPAD_BUTTON_SOUTH,             SDL_SCANCODE_SPACE,        "Jump", "JUMP"},
    {SDL_GAMEPAD_BUTTON_WEST,             SDL_SCANCODE_1,            "Action 1", "ACTIONBUTTON1"},
    {SDL_GAMEPAD_BUTTON_NORTH,             SDL_SCANCODE_2,            "Action 2", "ACTIONBUTTON2"},
    {SDL_GAMEPAD_BUTTON_DPAD_UP,       SDL_SCANCODE_3,            "Action 3", "ACTIONBUTTON3"},
    {SDL_GAMEPAD_BUTTON_DPAD_RIGHT,    SDL_SCANCODE_4,            "Action 4", "ACTIONBUTTON4"},
    {SDL_GAMEPAD_BUTTON_DPAD_DOWN,     SDL_SCANCODE_5,            "Action 5", "ACTIONBUTTON5"},
    {SDL_GAMEPAD_BUTTON_DPAD_LEFT,     SDL_SCANCODE_6,            "Action 6", "ACTIONBUTTON6"},
    // Shift is not an action of its own: the client already reads it as "the
    // bottom-left bar", which is where actions 7 to 12 live on a keyboard
    // too. So the modifier costs nothing to implement and behaves exactly as
    // a player expects it to.
    {SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,  SDL_SCANCODE_LSHIFT,       "Hold: actions 7-12"},
    {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, SDL_SCANCODE_TAB,          "Target nearest enemy", "TARGETNEARESTENEMY"},
    {SDL_GAMEPAD_BUTTON_LEFT_STICK,     SDL_SCANCODE_NUMLOCKCLEAR, "Autorun", "TOGGLEAUTORUN"},
    // The last button every pad has and nothing was using. X sits the
    // character down, and dives while swimming - both of them things a
    // keyboard could do and a controller could not reach at all.
    {SDL_GAMEPAD_BUTTON_RIGHT_STICK,    SDL_SCANCODE_X,            "Sit down, or dive"},
};

}  // namespace

namespace {

/// The names an Xbox pad puts on its buttons, which is the shape SDL resolves
/// every other pad to and so the fallback for anything with no names of its
/// own.
const char* xboxLabel(SDL_GamepadButton button) {
    switch (button) {
        case SDL_GAMEPAD_BUTTON_SOUTH:             return "A";
        case SDL_GAMEPAD_BUTTON_EAST:             return "B";
        case SDL_GAMEPAD_BUTTON_WEST:             return "X";
        case SDL_GAMEPAD_BUTTON_NORTH:             return "Y";
        case SDL_GAMEPAD_BUTTON_BACK:          return "Back";
        case SDL_GAMEPAD_BUTTON_START:         return "Start";
        case SDL_GAMEPAD_BUTTON_LEFT_STICK:     return "Left stick click";
        case SDL_GAMEPAD_BUTTON_RIGHT_STICK:    return "Right stick click";
        case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:  return "Left bumper";
        case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return "Right bumper";
        case SDL_GAMEPAD_BUTTON_DPAD_UP:       return "D-pad up";
        case SDL_GAMEPAD_BUTTON_DPAD_DOWN:     return "D-pad down";
        case SDL_GAMEPAD_BUTTON_DPAD_LEFT:     return "D-pad left";
        case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:    return "D-pad right";
        case SDL_GAMEPAD_BUTTON_MISC1:         return "Share";
        // SDL names the paddles by where they sit on the back of the pad
        // rather than by the letters on them: 1 is upper left, 2 upper right,
        // 3 lower left, 4 lower right. An Elite pad prints P1, P3, P2, P4 in
        // that order, which is why these read out of sequence.
        case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1:       return "Paddle P1";
        case SDL_GAMEPAD_BUTTON_LEFT_PADDLE1:       return "Paddle P3";
        case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2:       return "Paddle P2";
        case SDL_GAMEPAD_BUTTON_LEFT_PADDLE2:       return "Paddle P4";
        case SDL_GAMEPAD_BUTTON_TOUCHPAD:      return "Touchpad";
        default:                                  return "";
    }
}

}  // namespace

const char* padButtonLabel(SDL_GamepadButton button, core::Gamepad::Kind kind) {
    using Kind = core::Gamepad::Kind;
    // Only what differs. The face buttons are read by position - see the hint
    // set where the subsystem starts - so the button named here is always the
    // same button under the same thumb, whatever is printed on it.
    switch (kind) {
        case Kind::PlayStation:
            switch (button) {
                case SDL_GAMEPAD_BUTTON_SOUTH:             return "Cross";
                case SDL_GAMEPAD_BUTTON_EAST:             return "Circle";
                case SDL_GAMEPAD_BUTTON_WEST:             return "Square";
                case SDL_GAMEPAD_BUTTON_NORTH:             return "Triangle";
                case SDL_GAMEPAD_BUTTON_BACK:          return "Share";
                case SDL_GAMEPAD_BUTTON_START:         return "Options";
                case SDL_GAMEPAD_BUTTON_GUIDE:         return "PS button";
                case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:  return "L1";
                case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return "R1";
                case SDL_GAMEPAD_BUTTON_LEFT_STICK:     return "L3";
                case SDL_GAMEPAD_BUTTON_RIGHT_STICK:    return "R3";
                case SDL_GAMEPAD_BUTTON_MISC1:         return "Microphone";
                default: break;
            }
            break;
        case Kind::Nintendo:
            // Nintendo prints its letters the other way round: the bottom
            // button is B and the right one is A, the left is Y and the top
            // is X. Read by position and named by what is printed there, so
            // the button a player is told to press is the one under their
            // thumb.
            switch (button) {
                case SDL_GAMEPAD_BUTTON_SOUTH:             return "B";
                case SDL_GAMEPAD_BUTTON_EAST:             return "A";
                case SDL_GAMEPAD_BUTTON_WEST:             return "Y";
                case SDL_GAMEPAD_BUTTON_NORTH:             return "X";
                case SDL_GAMEPAD_BUTTON_BACK:          return "Minus";
                case SDL_GAMEPAD_BUTTON_START:         return "Plus";
                case SDL_GAMEPAD_BUTTON_GUIDE:         return "Home";
                case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:  return "L";
                case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return "R";
                case SDL_GAMEPAD_BUTTON_LEFT_STICK:     return "Left stick click";
                case SDL_GAMEPAD_BUTTON_RIGHT_STICK:    return "Right stick click";
                case SDL_GAMEPAD_BUTTON_MISC1:         return "Capture";
                default: break;
            }
            break;
        case Kind::SteamDeck:
            // Xbox letters on the face, and four buttons on the back that
            // SDL reports in its own geometric order - upper left, upper
            // right, lower left, lower right - which on a Deck is L4, R4, L5,
            // R5.
            switch (button) {
                case SDL_GAMEPAD_BUTTON_BACK:          return "View";
                case SDL_GAMEPAD_BUTTON_START:         return "Menu";
                case SDL_GAMEPAD_BUTTON_GUIDE:         return "Steam";
                case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:  return "L1";
                case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return "R1";
                case SDL_GAMEPAD_BUTTON_LEFT_STICK:     return "L3";
                case SDL_GAMEPAD_BUTTON_RIGHT_STICK:    return "R3";
                case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1:       return "L4";
                case SDL_GAMEPAD_BUTTON_LEFT_PADDLE1:       return "R4";
                case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2:       return "L5";
                case SDL_GAMEPAD_BUTTON_LEFT_PADDLE2:       return "R5";
                case SDL_GAMEPAD_BUTTON_MISC1:         return "Quick access";
                default: break;
            }
            break;
        case Kind::Luna:
            if (button == SDL_GAMEPAD_BUTTON_MISC1) return "Microphone";
            break;
        case Kind::Xbox:
        case Kind::Stadia:
        case Kind::Shield:
        case Kind::Virtual:
        case Kind::Unknown:
            break;
    }
    return xboxLabel(button);
}

const char* padKeyName(SDL_GamepadButton button) {
    switch (button) {
        case SDL_GAMEPAD_BUTTON_SOUTH:             return "PAD1";
        case SDL_GAMEPAD_BUTTON_EAST:             return "PAD2";
        case SDL_GAMEPAD_BUTTON_WEST:             return "PAD3";
        case SDL_GAMEPAD_BUTTON_NORTH:             return "PAD4";
        case SDL_GAMEPAD_BUTTON_BACK:          return "PADBACK";
        case SDL_GAMEPAD_BUTTON_GUIDE:         return "PADSYSTEM";
        case SDL_GAMEPAD_BUTTON_START:         return "PADFORWARD";
        case SDL_GAMEPAD_BUTTON_LEFT_STICK:     return "PADLSTICK";
        case SDL_GAMEPAD_BUTTON_RIGHT_STICK:    return "PADRSTICK";
        case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:  return "PADLSHOULDER";
        case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return "PADRSHOULDER";
        case SDL_GAMEPAD_BUTTON_DPAD_UP:       return "PADDUP";
        case SDL_GAMEPAD_BUTTON_DPAD_DOWN:     return "PADDDOWN";
        case SDL_GAMEPAD_BUTTON_DPAD_LEFT:     return "PADDLEFT";
        case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:    return "PADDRIGHT";
        case SDL_GAMEPAD_BUTTON_MISC1:         return "PADSOCIAL";
        case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1:       return "PADPADDLE1";
        case SDL_GAMEPAD_BUTTON_LEFT_PADDLE1:       return "PADPADDLE2";
        case SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2:       return "PADPADDLE3";
        case SDL_GAMEPAD_BUTTON_LEFT_PADDLE2:       return "PADPADDLE4";
        default:                                  return "";
    }
}

SDL_Scancode padClientKeyFor(const std::string& command) {
    // The poll sites' own keys: movement and jumping in the camera controller,
    // the action bar, Tab and Print Screen in GameScreen. The order of the
    // action buttons is the bar's slot order.
    static constexpr struct {
        const char* command;
        SDL_Scancode key;
    } kPolled[] = {
        {"MOVEFORWARD",        SDL_SCANCODE_W},
        {"MOVEBACKWARD",       SDL_SCANCODE_S},
        {"TURNLEFT",           SDL_SCANCODE_A},
        {"TURNRIGHT",          SDL_SCANCODE_D},
        {"STRAFELEFT",         SDL_SCANCODE_Q},
        {"STRAFERIGHT",        SDL_SCANCODE_E},
        {"JUMP",               SDL_SCANCODE_SPACE},
        {"TOGGLEAUTORUN",      SDL_SCANCODE_NUMLOCKCLEAR},
        {"TARGETNEARESTENEMY", SDL_SCANCODE_TAB},
        {"SCREENSHOT",         SDL_SCANCODE_PRINTSCREEN},
        {"ACTIONBUTTON1",      SDL_SCANCODE_1},
        {"ACTIONBUTTON2",      SDL_SCANCODE_2},
        {"ACTIONBUTTON3",      SDL_SCANCODE_3},
        {"ACTIONBUTTON4",      SDL_SCANCODE_4},
        {"ACTIONBUTTON5",      SDL_SCANCODE_5},
        {"ACTIONBUTTON6",      SDL_SCANCODE_6},
        {"ACTIONBUTTON7",      SDL_SCANCODE_7},
        {"ACTIONBUTTON8",      SDL_SCANCODE_8},
        {"ACTIONBUTTON9",      SDL_SCANCODE_9},
        {"ACTIONBUTTON10",     SDL_SCANCODE_0},
        {"ACTIONBUTTON11",     SDL_SCANCODE_MINUS},
        {"ACTIONBUTTON12",     SDL_SCANCODE_EQUALS},
    };
    for (const auto& polled : kPolled) {
        if (command == polled.command) return polled.key;
    }
    return SDL_SCANCODE_UNKNOWN;
}

namespace {

// The buttons only some pads carry, offered to the pads that have them.
//
// Four paddles and a share button are worth a great deal to a client whose
// action bar has twelve slots and whose pad reaches six of them without a
// modifier: a Steam Deck's four back buttons, or an Elite pad's, take the
// next four slots with nothing held down. Bound by SDL's own geometric order
// - upper left, upper right, lower left, lower right - so the top pair are
// the two nearest the index fingers on every pad that has them.
//
// Nothing here is in the base table, because a button that is not on the pad
// is a row in a settings list that does not exist and a key that can never be
// pressed.
constexpr PadBinding kExtras[] = {
    {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1, SDL_SCANCODE_7,          "Action 7", "ACTIONBUTTON7"},
    {SDL_GAMEPAD_BUTTON_LEFT_PADDLE1, SDL_SCANCODE_8,          "Action 8", "ACTIONBUTTON8"},
    {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2, SDL_SCANCODE_9,          "Action 9", "ACTIONBUTTON9"},
    {SDL_GAMEPAD_BUTTON_LEFT_PADDLE2, SDL_SCANCODE_0,          "Action 10", "ACTIONBUTTON10"},
    // The share, capture and microphone buttons are all one button to SDL,
    // and on every pad that has one it is the button for keeping a moment.
    {SDL_GAMEPAD_BUTTON_MISC1,   SDL_SCANCODE_PRINTSCREEN, "Screenshot", "SCREENSHOT"},
};

}  // namespace

std::span<const PadBinding> padExtraBindings() { return kExtras; }

std::span<const PadBinding> padBindings() { return kBindings; }

}  // namespace ui
}  // namespace wowee
