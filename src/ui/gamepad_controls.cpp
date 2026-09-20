#include "ui/gamepad_controls.hpp"

#include "core/gamepad.hpp"
#include "core/input.hpp"
#include "core/logger.hpp"
#include "rendering/camera_controller.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>
#include <vector>

namespace wowee {
namespace ui {

namespace {

/// Whether a direction is pushed far enough to count, given whether it was
/// already. The release threshold is lower than the press one, so a thumb
/// resting on the edge does not flicker between walking and standing.
bool pushed(float axis, float threshold, bool wasOn) {
    const float release = std::max(threshold - 0.10f, 0.05f);
    return axis > (wasOn ? release : threshold);
}

}  // namespace

GamepadControls& gamepadControls() {
    static GamepadControls instance;
    return instance;
}

void GamepadControls::setInWorld(bool inWorld) {
    if (inWorld_ == inWorld) return;
    inWorld_ = inWorld;
    if (!inWorld_) reset();
}

void GamepadControls::setEnabled(bool enabled) {
    if (enabled_ == enabled) return;
    enabled_ = enabled;
    if (!enabled_) reset();
}

void GamepadControls::setLookDegreesPerSecond(float degrees) {
    lookDegreesPerSecond_ = std::clamp(degrees, 30.0f, 720.0f);
}

void GamepadControls::holdKey(SDL_Scancode key, bool held, std::uint8_t source) {
    if (key <= SDL_SCANCODE_UNKNOWN || key >= SDL_SCANCODE_COUNT) return;
    const auto i = static_cast<std::size_t>(key);
    // A key this is not holding is left alone. Two things can drive the same
    // virtual key - a phone's on-screen stick is the other - and whichever of
    // them is idle must not switch the other one off. The same goes for the
    // stick and the buttons here, which can both want W.
    if (!held && (heldKeys_[i] & source) == 0) return;
    heldKeys_[i] = held ? static_cast<std::uint8_t>(heldKeys_[i] | source)
                        : static_cast<std::uint8_t>(heldKeys_[i] & ~source);
    core::Input::getInstance().setVirtualKey(key, heldKeys_[i] != 0);
}

void GamepadControls::releaseKeys() {
    for (std::size_t i = 0; i < heldKeys_.size(); ++i) {
        if (heldKeys_[i] == 0) continue;
        heldKeys_[i] = 0;
        core::Input::getInstance().setVirtualKey(static_cast<SDL_Scancode>(i), false);
    }
    if (escapeDown_) {
        ImGui::GetIO().AddKeyEvent(ImGuiKey_Escape, false);
        escapeDown_ = false;
    }
    for (int key : imguiKeysDown_) ImGui::GetIO().AddKeyEvent(static_cast<ImGuiKey>(key), false);
    imguiKeysDown_.clear();
    // Forgotten as well as released, so a button still held when the pad is
    // live again is looked up afresh rather than answered from before.
    routes_.fill(ButtonRoute{});
    buttonWasDown_.fill(false);
    steering_ = false;
    zoomRemainder_ = 0.0f;
}

void GamepadControls::reset() {
    releaseKeys();
    for (std::size_t i = 0; i < heldMouseButtons_.size(); ++i) {
        if (!heldMouseButtons_[i]) continue;
        heldMouseButtons_[i] = false;
        const int button = static_cast<int>(i);
        core::Input::getInstance().setVirtualMouseButton(button, false);
        ImGui::GetIO().AddMouseButtonEvent(button == SDL_BUTTON_RIGHT ? 1 : 0, false);
    }
    touchTrail_.reset();
    touchClickButton_ = 0;
}

void GamepadControls::applyMovement(float x, float y) {
    // SDL's Y is positive downwards, and pushing the stick up means forward.
    const auto stickHolds = [this](SDL_Scancode key) {
        return (heldKeys_[static_cast<std::size_t>(key)] & kFromStick) != 0;
    };
    const bool forward = pushed(-y, kWalkThreshold, stickHolds(SDL_SCANCODE_W));
    const bool back = pushed(y, kWalkThreshold, stickHolds(SDL_SCANCODE_S));
    // Q and E rather than A and D, as the on-screen stick does it: with no
    // right mouse button held this client turns the character on A and D and
    // strafes on Q and E, and a stick pushed sideways should sidestep rather
    // than swing the view - the right stick is what swings the view.
    const bool left = pushed(-x, kStrafeThreshold, stickHolds(SDL_SCANCODE_Q));
    const bool right = pushed(x, kStrafeThreshold, stickHolds(SDL_SCANCODE_E));

    holdKey(SDL_SCANCODE_W, forward, kFromStick);
    holdKey(SDL_SCANCODE_S, back, kFromStick);
    holdKey(SDL_SCANCODE_Q, left, kFromStick);
    holdKey(SDL_SCANCODE_E, right, kFromStick);

    // Facing follows the camera while the stick is pushed, which is what the
    // right mouse button does on a desktop. Without it the character walks
    // sideways across the screen while still facing wherever they last were.
    steering_ = forward || back || left || right;
}

void GamepadControls::applyLook(float x, float y, float deltaTime) {
    if (!camera_) return;
    if (x == 0.0f && y == 0.0f) return;
    // applyLookDelta takes mouse pixels and multiplies by the mouse's own
    // sensitivity. Dividing it out here keeps the pad's turn rate in degrees
    // a second, so slowing the mouse does not slow the pad.
    const float mouseSensitivity = std::max(camera_->getMouseSensitivity(), 0.0001f);
    const float degrees = lookDegreesPerSecond_ * deltaTime;
    const float pixels = degrees / mouseSensitivity;
    const float up = invertLook_ ? -y : y;
    camera_->applyLookDelta(x * pixels, up * pixels);
}

void GamepadControls::applyZoom(float in, float out, float deltaTime) {
    if (!camera_) return;
    const float pull = in - out;
    if (pull == 0.0f) {
        zoomRemainder_ = 0.0f;
        return;
    }
    // Carried between frames. A trigger held a third of the way is worth two
    // notches a second, which is less than one in any single frame - rounded
    // away each time, it would be a trigger that does nothing.
    zoomRemainder_ += pull * kZoomNotchesPerSecond * deltaTime;
    const float whole = std::trunc(zoomRemainder_);
    if (whole == 0.0f) return;
    zoomRemainder_ -= whole;
    camera_->processMouseWheel(whole);
}

void GamepadControls::holdMouseButton(int button, bool held) {
    if (button < 0 || button >= static_cast<int>(heldMouseButtons_.size())) return;
    const auto i = static_cast<std::size_t>(button);
    // As with the keys: a button this is not holding is left alone, so a real
    // mouse and the pad cannot switch each other off.
    if (!held && !heldMouseButtons_[i]) return;
    if (held == heldMouseButtons_[i]) return;
    heldMouseButtons_[i] = held;
    // Both channels, because both are read. The world's own targeting polls
    // core::Input, and every panel asks ImGui.
    core::Input::getInstance().setVirtualMouseButton(button, held);
    ImGui::GetIO().AddMouseButtonEvent(button == SDL_BUTTON_RIGHT ? 1 : 0, held);
}

void GamepadControls::setPointerMode(bool on) {
    if (pointerMode_ == on) return;
    pointerMode_ = on;
    // Nothing is released here. Every path that turns the pointer off either
    // resets all held buttons or goes on to applyClicks, which stops counting
    // A and X - and a release here would drop a click the touchpad still holds.
    if (!on) return;
    // Starts where the pointer already is rather than at the middle of the
    // screen, so turning it on twice does not throw away where it was left.
    float fx = 0.0f;
    float fy = 0.0f;
    SDL_GetMouseState(&fx, &fy);
    const int x = static_cast<int>(fx);
    const int y = static_cast<int>(fy);
    pointerX_ = fx;
    pointerY_ = fy;
    if (window_ && (x == 0 && y == 0)) {
        int w = 0;
        int h = 0;
        SDL_GetWindowSize(window_, &w, &h);
        pointerX_ = static_cast<float>(w) * 0.5f;
        pointerY_ = static_cast<float>(h) * 0.5f;
    }
}

void GamepadControls::applyPointer(float deltaTime) {
    if (!window_) return;
    int w = 0;
    int h = 0;
    SDL_GetWindowSize(window_, &w, &h);
    if (w <= 0 || h <= 0) return;

    const auto& pad = core::gamepad();
    const glm::vec2 step = pointerStep(pad.rightStick().x, pad.rightStick().y, deltaTime);
    if (step.x != 0.0f || step.y != 0.0f) {
        pointerX_ = std::clamp(pointerX_ + step.x, 0.0f, static_cast<float>(w - 1));
        pointerY_ = std::clamp(pointerY_ + step.y, 0.0f, static_cast<float>(h - 1));
        SDL_WarpMouseInWindow(window_, static_cast<int>(pointerX_), static_cast<int>(pointerY_));
    }
}

void GamepadControls::applyTouchpad() {
    const auto& finger = core::gamepad().touch(0);
    // Followed every frame, before anything can return, so a finger that
    // was down while there was no window is not read as one long slide.
    const glm::vec2 slide = touchTrail_.follow(finger.down, finger.position);
    if (!window_ || (slide.x == 0.0f && slide.y == 0.0f)) return;

    int w = 0;
    int h = 0;
    SDL_GetWindowSize(window_, &w, &h);
    if (w <= 0 || h <= 0) return;

    // Starts from where the cursor really is. The touchpad works outside
    // pointer mode too, so the stored position can be stale - left from the
    // last time the stick moved it, with a real mouse used since.
    float fx = 0.0f;
    float fy = 0.0f;
    SDL_GetMouseState(&fx, &fy);
    if (std::abs(fx - pointerX_) > 1.0f || std::abs(fy - pointerY_) > 1.0f) {
        pointerX_ = fx;
        pointerY_ = fy;
    }
    const glm::vec2 step = touchStep(slide, static_cast<float>(w));
    pointerX_ = std::clamp(pointerX_ + step.x, 0.0f, static_cast<float>(w - 1));
    pointerY_ = std::clamp(pointerY_ + step.y, 0.0f, static_cast<float>(h - 1));
    SDL_WarpMouseInWindow(window_, static_cast<int>(pointerX_), static_cast<int>(pointerY_));
}

void GamepadControls::applyClicks() {
    const auto& pad = core::gamepad();
    if (!pad.held(SDL_GAMEPAD_BUTTON_TOUCHPAD)) {
        touchClickButton_ = 0;
    } else if (touchClickButton_ == 0) {
        touchClickButton_ = pad.touch(1).down ? SDL_BUTTON_RIGHT : SDL_BUTTON_LEFT;
    }

    // A is the click, as it is on every console, and X is the right click -
    // which in this game opens a corpse, uses a door and brings up a unit's
    // menu, so a pad without one cannot loot. Those two only count while the
    // pointer is up. Both channels are set once, from every source together,
    // so one source letting go cannot release a button another still holds.
    // held rather than schemeHeld: while the pointer is up these two are the
    // pointer's, whatever they are bound to. The default scheme's own entries
    // for them live in the binding table now, so asking whether they are
    // still unbound would answer no and leave the pointer with no click.
    const bool left = (pointerMode_ && pad.held(SDL_GAMEPAD_BUTTON_SOUTH)) ||
                      touchClickButton_ == SDL_BUTTON_LEFT;
    const bool right = (pointerMode_ && pad.held(SDL_GAMEPAD_BUTTON_WEST)) ||
                       touchClickButton_ == SDL_BUTTON_RIGHT;
    holdMouseButton(SDL_BUTTON_LEFT, left);
    holdMouseButton(SDL_BUTTON_RIGHT, right);
}

bool GamepadControls::schemeHeld(SDL_GamepadButton button) const {
    if (button < 0 || button >= SDL_GAMEPAD_BUTTON_COUNT) return false;
    return core::gamepad().held(button) &&
           routes_[static_cast<std::size_t>(button)].kind == PadKeyAnswer::Kind::Unbound;
}

void GamepadControls::routeButtons() {
    const auto& pad = core::gamepad();
    for (int b = 0; b < SDL_GAMEPAD_BUTTON_COUNT; ++b) {
        const auto button = static_cast<SDL_GamepadButton>(b);
        const auto i = static_cast<std::size_t>(b);
        const bool down = pad.held(button);
        const bool wasDown = buttonWasDown_[i];
        buttonWasDown_[i] = down;
        if (!down) {
            routes_[i] = ButtonRoute{};
            continue;
        }
        if (wasDown || !keyRouter_) continue;
        const char* name = padKeyName(button);
        if (name[0] == '\0') continue;

        // Asked once, on the press. The interface may take the press outright
        // - the key binding panel capturing it, or a binding script - and then
        // nothing here acts on it.
        const PadKeyAnswer answer = keyRouter_(name);
        ButtonRoute route;
        route.kind = answer.kind;
        if (answer.kind == PadKeyAnswer::Kind::Command) {
            route.key = padClientKeyFor(answer.command);
            route.imguiKey = answer.imguiKey;
            route.escape = answer.command == "TOGGLEGAMEMENU";
            if (route.key == SDL_SCANCODE_UNKNOWN && route.imguiKey == 0 && !route.escape) {
                // Bound to something the client does without a key it polls -
                // opening chat, sheathing. The press does nothing rather than
                // falling back to the default scheme, which would be the
                // button doing what the player just bound it away from.
                LOG_WARNING("Gamepad: ", name, " is bound to ", answer.command,
                            ", which a controller cannot trigger yet");
            }
        }
        routes_[i] = route;
    }
}

void GamepadControls::applyButtons() {
    const auto& pad = core::gamepad();
    // Worked out whole before any key is held, because two buttons can want
    // the same key - X by default, and a D-pad bound to ACTIONBUTTON1 - and
    // one of them letting go must not release the other.
    std::array<bool, SDL_SCANCODE_COUNT> wanted{};
    for (const PadBinding& row : padBindings()) {
        // A and X are the pointer's two clicks while it is up. Jumping and
        // casting from the same press would fire a spell at whatever was
        // under the cursor every time a window was clicked.
        if (pointerMode_ && (row.button == SDL_GAMEPAD_BUTTON_SOUTH ||
                             row.button == SDL_GAMEPAD_BUTTON_WEST)) {
            continue;
        }
        if (schemeHeld(row.button)) wanted[static_cast<std::size_t>(row.key)] = true;
    }

    // The paddles and the share button, on the pads that have them. No test
    // for whether this pad does: a button it has not got is never held, and
    // asking SDL once a frame per button to learn the same thing is work for
    // nothing. The listing in the settings panel asks, because a row there is
    // a promise.
    for (const PadBinding& row : padExtraBindings()) {
        if (schemeHeld(row.button)) wanted[static_cast<std::size_t>(row.key)] = true;
    }

    // Escape is the interface's, not the game's: it is read through
    // KeybindingManager, which asks ImGui. A virtual scancode never reaches
    // it, so this one goes on ImGui's own queue - once down, once up, because
    // ImGui counts the repeats itself.
    bool escape = schemeHeld(SDL_GAMEPAD_BUTTON_EAST) || schemeHeld(SDL_GAMEPAD_BUTTON_START);
    // The panels KeybindingManager answers ask ImGui too, for the same reason.
    std::vector<int> imguiWanted;

    for (int b = 0; b < SDL_GAMEPAD_BUTTON_COUNT; ++b) {
        const auto button = static_cast<SDL_GamepadButton>(b);
        // The pointer's two clicks outrank whatever the button is bound to,
        // for as long as the pointer is up. A and X are the click and the
        // right click there, and the defaults now sit in the binding table
        // like any other binding - so without this, raising the pointer and
        // clicking would jump and cast as well.
        if (pointerMode_ && (button == SDL_GAMEPAD_BUTTON_SOUTH ||
                             button == SDL_GAMEPAD_BUTTON_WEST)) {
            continue;
        }
        const ButtonRoute& route = routes_[static_cast<std::size_t>(b)];
        if (route.kind != PadKeyAnswer::Kind::Command) continue;
        if (!pad.held(button)) continue;
        if (route.key != SDL_SCANCODE_UNKNOWN) wanted[static_cast<std::size_t>(route.key)] = true;
        if (route.escape) escape = true;
        if (route.imguiKey != 0 &&
            std::find(imguiWanted.begin(), imguiWanted.end(), route.imguiKey) == imguiWanted.end()) {
            imguiWanted.push_back(route.imguiKey);
        }
    }

    for (std::size_t k = 0; k < wanted.size(); ++k) {
        if (!wanted[k] && (heldKeys_[k] & kFromButtons) == 0) continue;
        holdKey(static_cast<SDL_Scancode>(k), wanted[k], kFromButtons);
    }

    if (escape != escapeDown_) {
        ImGui::GetIO().AddKeyEvent(ImGuiKey_Escape, escape);
        escapeDown_ = escape;
    }

    ImGuiIO& io = ImGui::GetIO();
    for (int key : imguiKeysDown_) {
        if (std::find(imguiWanted.begin(), imguiWanted.end(), key) == imguiWanted.end()) {
            io.AddKeyEvent(static_cast<ImGuiKey>(key), false);
        }
    }
    for (int key : imguiWanted) {
        if (std::find(imguiKeysDown_.begin(), imguiKeysDown_.end(), key) == imguiKeysDown_.end()) {
            io.AddKeyEvent(static_cast<ImGuiKey>(key), true);
        }
    }
    imguiKeysDown_ = std::move(imguiWanted);
}

void GamepadControls::update(float deltaTime) {
    auto& pad = core::gamepad();
    ImGuiIO& io = ImGui::GetIO();
    // A pad is only read while the window has focus. SDL does not update a
    // controller for an unfocused window unless it is told to, so whatever
    // the stick last said stays said - and alt-tabbing mid-stride would leave
    // the character walking north for as long as the player was away.
    const bool focused =
        !window_ || (SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) != 0;
    // Two tiers, because a pad is not only a way to play: it is the only
    // input device some players have in front of them.
    //
    // The world's controls - the sticks, the action buttons, every key -
    // belong to the world. The touchpad does not: a login screen is a screen
    // with buttons on it, and a trackpad that goes dead until the player is
    // already in the world is dead exactly where someone holding a pad needs
    // it, with no way to press Login at all.
    const bool usable = enabled_ && pad.isConnected() && focused;
    const bool live = usable && inWorld_;

    // ImGui navigates its own windows with a pad, which is what the login and
    // character screens want and the opposite of what the world wants: in the
    // world the D-pad casts spells, and a panel left open would take those
    // presses as "move the highlight" as well. So the nav flag follows who is
    // driving.
    if (live) {
        io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad;
    } else {
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    }

    if (!pad.isConnected()) announced_ = false;
    if (!usable) {
        // Losing the pad or switching it off ends pointer mode; losing focus
        // for a moment does not. Coming back from another window to find the
        // pointer gone would be a small betrayal of the mode the player left
        // it in.
        if (!enabled_ || !pad.isConnected()) setPointerMode(false);
        reset();
        return;
    }
    // Pointer mode is the right stick's, and the right stick is the world's.
    // The touchpad below needs no mode.
    if (!inWorld_) setPointerMode(false);

    if (!announced_) {
        announced_ = true;
        // Once per connection, at warning, because a player who has plugged a
        // pad in and is wondering whether the client saw it has exactly one
        // place to look, and that log is warnings only.
        const auto kind = pad.kind();
        const auto name = [kind](SDL_GamepadButton button) {
            return padButtonLabel(button, kind);
        };
        LOG_WARNING("Gamepad: ", pad.describe(),
                    " - left stick moves, right stick looks, triggers zoom, ",
                    name(SDL_GAMEPAD_BUTTON_SOUTH), " jumps, ",
                    name(SDL_GAMEPAD_BUTTON_EAST), " closes, ",
                    name(SDL_GAMEPAD_BUTTON_WEST), "/", name(SDL_GAMEPAD_BUTTON_NORTH),
                    " and the D-pad are actions 1-6, hold ",
                    name(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER), " for 7-12, ",
                    name(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER), " targets, ",
                    name(SDL_GAMEPAD_BUTTON_LEFT_STICK), " autoruns, ",
                    name(SDL_GAMEPAD_BUTTON_BACK), " gives you a pointer",
                    pad.hasButton(SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1)
                        ? ", the back buttons are actions 7-10"
                        : "",
                    pad.hasTouchpad() ? ", and the touchpad is a trackpad - click it, or click "
                                        "with two fingers to right-click"
                                      : "");
    }

    // Typing takes the keys, not the pointer. A stick nudged while typing
    // should not walk the character out of town - but a finger sliding on the
    // touchpad should still move the cursor, which is what it does on the
    // laptop this is being typed on, and at the login screen the box being
    // typed into is the very thing a pointer is needed to leave.
    const bool typing = io.WantTextInput;

    if (!live || typing) {
        releaseKeys();
        applyTouchpad();
        applyClicks();
        return;
    }

    // Bindings first. A button the player has bound is no longer the default
    // scheme's, and that includes Back's pointer below and B's Escape.
    routeButtons();

    // Back switches the right stick between the view and the pointer. On its
    // edge rather than while held: it is a mode, and a mode that lasted only
    // as long as a thumb could hold a button would be no use for buying from
    // a vendor.
    const bool toggle = schemeHeld(SDL_GAMEPAD_BUTTON_BACK);
    if (toggle && !pointerToggleWasDown_) {
        setPointerMode(!pointerMode_);
        // At warning, because the two modes look identical apart from the
        // cursor, and a player wondering why the stick stopped turning the
        // view has one place to look.
        LOG_WARNING("Gamepad: the right stick now ",
                    pointerMode_ ? "moves the pointer" : "turns the view");
    }
    pointerToggleWasDown_ = toggle;

    // Walking and zooming work in both modes. A pointer that stopped the
    // player from stepping back out of a fire would be worse than none.
    applyMovement(pad.leftStick().x, pad.leftStick().y);
    applyZoom(pad.rightTrigger(), pad.leftTrigger(), deltaTime);
    if (pointerMode_) {
        applyPointer(deltaTime);
    } else {
        applyLook(pad.rightStick().x, pad.rightStick().y, deltaTime);
    }
    applyTouchpad();
    applyClicks();
    applyButtons();
}

}  // namespace ui
}  // namespace wowee
