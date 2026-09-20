#pragma once

/**
 * gamepad_controls.hpp - what a controller's sticks and buttons mean.
 *
 * The device itself is core/gamepad.hpp. This is the half that decides a push
 * of the left stick is W held down and a press of A is the space bar, and then
 * says so through the two channels the client already listens on:
 *
 *   core::Input's virtual keys, for everything polled by scancode - movement,
 *   jumping, the action bar, targeting, autorun. The on-screen stick a phone
 *   draws has used them since it was written, which is the proof they carry a
 *   press the whole way: through the movement chain, the animations and the
 *   packets that announce it, with nothing in between knowing where the press
 *   came from.
 *
 *   ImGui's key queue, for the few the interface reads rather than the game -
 *   Escape above all, which goes through KeybindingManager and so through
 *   ImGui::IsKeyPressed. A virtual scancode never reaches it.
 *
 * Nothing here is platform-specific: SDL resolves a pad on Windows, macOS,
 * Linux and Android to the same Xbox-shaped button set, so the table below is
 * the table everywhere.
 *
 * Looting a corpse, talking to an NPC and buying from a vendor are clicks
 * rather than keys, so the pad has a pointer as well: Back switches the right
 * stick from turning the view to moving the mouse. The mouse it moves is the
 * real one - warped, so that everything which asks where the pointer is gets
 * the truth, from ImGui down to the client's own picking - and only the button
 * press is synthesised.
 *
 * A pad with a touchpad also gets it as a trackpad, in either mode: a slide
 * moves the same pointer, a click is a left click, and a click with two
 * fingers down is a right click.
 */

#include <SDL3/SDL.h>

#include "core/gamepad.hpp"
#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <span>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace wowee {
namespace rendering { class CameraController; }
namespace ui {

/// One button, and the key the client already answers on.
///
/// The mapping is a table rather than a chain of ifs because it is data: the
/// settings panel can show it, and a reader can see the whole scheme at once.
/// It is the default only - a button the player has bound in the interface's
/// key binding panel does what that binding says instead.
struct PadBinding {
    SDL_GamepadButton button;
    SDL_Scancode key;      ///< what core::Input is told is held
    const char* what;      ///< what a person reads on the settings panel
    /// The interface's own name for what this does - "ACTIONBUTTON3" - or ""
    /// where there is none.
    ///
    /// These are seeded into the binding table so the game's own Key Bindings
    /// panel shows the pad's defaults beside the keyboard's, which is where a
    /// player looks to find out what a button does and the only place they
    /// can change it. Here rather than in a list of their own, because a
    /// second copy of this mapping would be wrong the moment either side
    /// moved.
    ///
    /// Empty for the three the interface has no command for: the bumper that
    /// is only a modifier, the button that raises the pointer, and the stick
    /// click that sits the character down.
    const char* command = "";
};

/// The default scheme, in the order it should be read.
///
/// Chosen against what this client actually answers to - the keys were taken
/// from the poll sites rather than from a list of what WoW binds - and against
/// what a pad has room for. Twelve action slots do not fit on a controller, so
/// six sit on the face and the pad, and the left bumper is held to reach the
/// other six: the client already reads shift as "the bottom-left bar", so the
/// modifier costs nothing and behaves exactly as it does on a keyboard.
/// Returned as a span rather than a pointer and an out-parameter: the count
/// travelling separately is a trap, and it sprang - `check(padExtraBindings(n),
/// n)` reads n before the call that sets it under one compiler and after it
/// under another, so the five extras were walked as ten and the test
/// segfaulted on Linux x86-64 while passing everywhere else.
[[nodiscard]] std::span<const PadBinding> padBindings();

/// What a player calls this button.
///
/// SDL maps every pad to an Xbox-shaped one, so these are the Xbox names - a
/// PlayStation pad's cross is reported as A and is named A here. SDL's own
/// strings are "a", "dpup", "leftshoulder", which is a protocol rather than
/// something to put on a settings panel. Returns an empty string for a button
/// nothing binds, which is how the settings panel knows not to list it.
[[nodiscard]] const char* padButtonLabel(SDL_GamepadButton button,
                                         core::Gamepad::Kind kind);

/// The buttons only some pads have, and what they do when they are there.
///
/// Paddles and the share button. Applied only when SDL says the pad in hand
/// actually has the button, so nothing here is bound into the air on a pad
/// without them.
[[nodiscard]] std::span<const PadBinding> padExtraBindings();

/// The name WoW's binding tables give a pad button - "PAD1", "PADDUP",
/// "PADLSHOULDER" - or an empty string for one that cannot be bound.
///
/// Retail's spelling, since the names are saved in bindings.cfg and compared
/// as strings. The touchpad click has none: it is the pointer's click, and a
/// binding on it would leave a trackpad that cannot click.
[[nodiscard]] const char* padKeyName(SDL_GamepadButton button);

/// The key the client polls for a binding command it performs itself, or
/// SDL_SCANCODE_UNKNOWN when it does not read that command from a scancode.
///
/// A button bound to one of these holds that key down, because the poll sites
/// read fixed scancodes rather than the binding table - pressing the key is
/// the only way in.
[[nodiscard]] SDL_Scancode padClientKeyFor(const std::string& command);

/// What the interface made of a pad button, asked once as it goes down.
struct PadKeyAnswer {
    enum class Kind {
        Unbound,  ///< nothing is bound to it: the default scheme applies
        Taken,    ///< the interface handled it - a binding being captured, or a script
        Command,  ///< bound to a command the client performs itself
    };
    Kind kind = Kind::Unbound;
    std::string command;  ///< for Command
    /// For Command, the ImGui key the client listens on when the command is
    /// one of KeybindingManager's panels; 0 otherwise.
    int imguiKey = 0;
};

/// Looks a pad button's binding name up in the interface.
using PadKeyRouter = std::function<PadKeyAnswer(const char* padKey)>;

/// A finger on the touchpad, followed from one frame to the next.
///
/// The pointer moves by how far the finger slides, not to where it lands, so
/// the frame a finger comes down is worth nothing. Otherwise lifting at the
/// right edge and landing at the left would throw the pointer across the
/// window, which is the opposite of how a trackpad is used.
class TouchTrail {
public:
    /// This frame's reading. Returns the slide since the last frame, in the
    /// touchpad's own 0-1 units, or zero if the finger has just landed or is
    /// not down.
    [[nodiscard]] glm::vec2 follow(bool down, glm::vec2 position) {
        const glm::vec2 slide = (down && wasDown_) ? position - last_ : glm::vec2(0.0f);
        wasDown_ = down;
        last_ = position;
        return slide;
    }

    /// Forgets the finger, so the next reading counts as a landing.
    void reset() { wasDown_ = false; }

private:
    bool wasDown_ = false;
    glm::vec2 last_{0.0f};
};

/**
 * The controller, applied to the client, once a frame.
 *
 * Owns the keys it sets. A key this has not held down is never cleared by it,
 * so on a phone with a pad attached the on-screen stick and the pad can both
 * drive W without one of them switching the other off.
 */
class GamepadControls {
public:
    /// The sticks only steer in the world, and so do the action buttons and
    /// every key. On the login and character screens those are left to
    /// ImGui, which navigates its windows with a pad - but the touchpad still
    /// moves the pointer and still clicks, because a login screen is a screen
    /// with buttons on it and a player holding a pad has to be able to press
    /// them.
    void setInWorld(bool inWorld);
    [[nodiscard]] bool isInWorld() const { return inWorld_; }

    /// The camera the right stick turns and the triggers zoom. Without one
    /// the sticks still move the character.
    void setCameraController(rendering::CameraController* camera) { camera_ = camera; }

    /// The window the pointer is moved inside. Without one there is no
    /// pointer mode, because there is nowhere to put the cursor.
    void setWindow(SDL_Window* window) { window_ = window; }

    /// Where a button's binding is looked up. Without one, every button keeps
    /// the default scheme.
    void setKeyRouter(PadKeyRouter router) { keyRouter_ = std::move(router); }

    /// How far the pointer travels this frame for a given push of the stick.
    ///
    /// Squared, so the one stick does both jobs a pointer needs: a nudge
    /// creeps across a button at a few pixels a second, a full push crosses
    /// the window in under two. Linear, the speed that lands on a 20 pixel
    /// button is too slow to cross a screen and the one that crosses a screen
    /// cannot land on the button.
    ///
    /// Static and taking everything it uses, so the curve can be checked
    /// without a window, a pad or a frame.
    [[nodiscard]] static glm::vec2 pointerStep(float stickX, float stickY, float deltaTime) {
        const float magnitude = std::sqrt(stickX * stickX + stickY * stickY);
        if (magnitude <= 0.0f || deltaTime <= 0.0f) return glm::vec2(0.0f);
        const float clamped = std::min(magnitude, 1.0f);
        const float speed = clamped * clamped * kPointerPointsPerSecond;
        return glm::vec2(stickX / magnitude, stickY / magnitude) * (speed * deltaTime);
    }

    /// How far the pointer travels for a slide across the touchpad.
    ///
    /// A slide the full width of the touchpad crosses the full width of the
    /// window. SDL reports both axes as 0 to 1 although the touchpad is about
    /// twice as wide as it is tall, so the vertical slide is scaled by that
    /// ratio to make the same finger travel move the pointer the same distance
    /// in every direction.
    [[nodiscard]] static glm::vec2 touchStep(glm::vec2 slide, float windowWidth) {
        if (windowWidth <= 0.0f) return glm::vec2(0.0f);
        return glm::vec2(slide.x * windowWidth, slide.y * windowWidth / kTouchpadAspect);
    }

    /// Reads the pad and applies it. Call once a frame, before Input::update,
    /// so that a button pressed this frame reads as just-pressed this frame
    /// rather than next.
    void update(float deltaTime);

    /// Releases every key this is holding. On leaving the world, on losing the
    /// pad, and when the interface takes the keyboard.
    void reset();

    /// True while the stick is being pushed, which is when the character
    /// should face where the camera looks - the same thing the right mouse
    /// button means on a desktop and a dragging finger means on a phone.
    [[nodiscard]] bool isSteering() const { return steering_; }

    /// Whether a connected pad is allowed to drive anything.
    void setEnabled(bool enabled);
    [[nodiscard]] bool isEnabled() const { return enabled_; }

    /// How fast the right stick turns the view, in degrees a second at full
    /// deflection. Independent of the mouse's own sensitivity: the two are
    /// different instruments and a player who slows one has said nothing
    /// about the other.
    void setLookDegreesPerSecond(float degrees);
    [[nodiscard]] float lookDegreesPerSecond() const { return lookDegreesPerSecond_; }

    /// Pushing the stick up looks up, unless this is set.
    void setInvertLook(bool invert) { invertLook_ = invert; }
    [[nodiscard]] bool isInvertLook() const { return invertLook_; }

private:
    /// What is holding a virtual key, as bits, so one source letting go does
    /// not release a key another still holds - the stick's W and the W of a
    /// button bound to MOVEFORWARD.
    static constexpr std::uint8_t kFromStick = 1;
    static constexpr std::uint8_t kFromButtons = 2;

    /// Releases the keys and the ImGui keys this is holding, but not the
    /// pointer's mouse buttons: the touchpad drives those, and it goes on
    /// working where the rest of the pad does not - outside the world, and
    /// while the player is typing.
    void releaseKeys();

    /// Sets a virtual key on behalf of one source and remembers which.
    void holdKey(SDL_Scancode key, bool held, std::uint8_t source);

    /// The left stick, as the four keys the client walks on.
    void applyMovement(float x, float y);
    /// The right stick, as a turn of the view.
    void applyLook(float x, float y, float deltaTime);
    /// The triggers, as notches of the wheel.
    void applyZoom(float in, float out, float deltaTime);
    /// The buttons: a bound one as its command, the rest as the keys in the
    /// table.
    void applyButtons();
    /// Looks up each button's binding as it goes down, and forgets it as it
    /// comes up.
    void routeButtons();
    /// Whether a button is down with nothing bound to it, so the default
    /// scheme's meaning for it applies.
    [[nodiscard]] bool schemeHeld(SDL_GamepadButton button) const;
    /// The right stick, as the mouse pointer, and two buttons as its clicks.
    void applyPointer(float deltaTime);
    /// Holds a mouse button and remembers that this is what is holding it.
    void holdMouseButton(int button, bool held);
    /// Turns the pointer on or off, putting the cursor somewhere sensible.
    void setPointerMode(bool on);
    /// The touchpad, as a trackpad: a slide moves the pointer.
    void applyTouchpad();
    /// Both mouse buttons, from every control that clicks: A and X while the
    /// pointer is up, and the touchpad in either mode.
    void applyClicks();

    rendering::CameraController* camera_ = nullptr;
    SDL_Window* window_ = nullptr;
    bool inWorld_ = false;
    /// The right stick moves the pointer instead of the view.
    bool pointerMode_ = false;
    /// Back is read here rather than through the table, because it toggles
    /// rather than holds and the edge is the whole of what it means.
    bool pointerToggleWasDown_ = false;
    /// Where the pointer is, in window coordinates. Kept as floats because a
    /// stick pushed gently is worth a fraction of a pixel a frame, and an int
    /// would round every one of them to nothing.
    float pointerX_ = 0.0f;
    float pointerY_ = 0.0f;
    /// Which mouse buttons this is holding, so letting go clears those and
    /// only those. Index is SDL's, which is one-based.
    std::array<bool, 8> heldMouseButtons_{};
    TouchTrail touchTrail_;
    /// The mouse button a touchpad click is holding, or 0. Chosen when the
    /// click goes down and kept until it comes up, so lifting the second
    /// finger mid-click does not swap the right button for the left.
    int touchClickButton_ = 0;
    bool enabled_ = true;
    bool steering_ = false;
    bool invertLook_ = false;
    bool announced_ = false;

    /// Which sources are holding each key down, so that letting go clears
    /// those and only those.
    std::array<std::uint8_t, SDL_SCANCODE_COUNT> heldKeys_{};

    /// How a held button is being answered, decided as it went down and kept
    /// until it comes up - a binding changed mid-press applies from the next.
    struct ButtonRoute {
        PadKeyAnswer::Kind kind = PadKeyAnswer::Kind::Unbound;
        SDL_Scancode key = SDL_SCANCODE_UNKNOWN;  ///< a command the client polls by scancode
        int imguiKey = 0;                          ///< a command the client reads through ImGui
        bool escape = false;                       ///< the game menu, which has a chain of its own
    };
    std::array<ButtonRoute, SDL_GAMEPAD_BUTTON_COUNT> routes_{};
    /// Which buttons were down last frame, so each press is looked up once.
    std::array<bool, SDL_GAMEPAD_BUTTON_COUNT> buttonWasDown_{};
    /// The ImGui keys routed buttons are holding, released as they let go.
    std::vector<int> imguiKeysDown_;
    PadKeyRouter keyRouter_;

    /// Whether the interface's Escape is currently down, so the press and the
    /// release are each sent once. ImGui counts repeats itself.
    bool escapeDown_ = false;

    float lookDegreesPerSecond_ = 180.0f;
    /// Left over from a zoom that did not reach a whole notch, so a trigger
    /// held lightly still zooms rather than rounding to nothing every frame.
    float zoomRemainder_ = 0.0f;

    /// Held before the direction lets go, so a thumb resting near the edge of
    /// the deadzone does not stutter between walking and standing. The
    /// numbers are the on-screen stick's, which were tuned against a thumb on
    /// a real control and behave the same way here.
    static constexpr float kWalkThreshold = 0.30f;
    static constexpr float kStrafeThreshold = 0.45f;
    static constexpr float kReleaseHysteresis = 0.10f;
    /// Notches of the mouse wheel a fully pulled trigger is worth per second.
    static constexpr float kZoomNotchesPerSecond = 6.0f;
    /// How fast the pointer crosses the window at full deflection, in window
    /// points a second. Fast enough to cross a 1280-wide window in a second
    /// and a half, which is about as slow as a pointer can be before it feels
    /// broken.
    static constexpr float kPointerPointsPerSecond = 900.0f;
    /// Width over height of a DualShock 4 touchpad (1920 by 943 in its own
    /// units). A DualSense's is slightly squarer, not enough to show.
    static constexpr float kTouchpadAspect = 2.0f;
};

/// The one instance, reached the way the touch controls are.
GamepadControls& gamepadControls();

}  // namespace ui
}  // namespace wowee
