#pragma once

/**
 * gamepad.hpp - the controller SDL found, and what it currently reads.
 *
 * This is the device half only: open, close, hot-plug, and the state of the
 * sticks, triggers and buttons this frame and last. What any of it *means* is
 * ui/gamepad_controls.hpp, which turns it into the keys and camera motion the
 * rest of the client already answers to.
 *
 * The split is the point. Every platform this client runs on reaches its
 * controllers through the same SDL game controller layer - Windows through
 * XInput and RawInput, macOS through GameController.framework, Linux through
 * evdev, Android through its own HID stack - and SDL resolves each of them to
 * one abstract pad with an Xbox-shaped button set. So there is nothing here
 * that is true of one platform and not another, and nothing below this line
 * needs a platform test.
 */

#include <SDL3/SDL.h>
#include <glm/glm.hpp>

#include <array>
#include <string>

namespace wowee {
namespace core {

/// One axis as a fraction of full deflection, from SDL's raw reading.
///
/// SDL's range is not symmetric - it runs from -32768 to 32767 - so dividing
/// by either end gives a number slightly outside -1..1 at the other. Clamped,
/// because everything downstream multiplies by it.
[[nodiscard]] float axisFraction(int raw);

/// A stick's deflection with its slack removed, as a vector no longer than 1.
///
/// The deadzone is taken radially - on the length of the vector, not on each
/// axis - and what is left is rescaled so that the first movement outside it
/// starts from zero rather than from the deadzone. A per-axis deadzone is the
/// obvious way to write this and is wrong in two visible ways: a stick pushed
/// diagonally passes the test on both axes at once and jumps to full speed,
/// and a stick pushed straight up reads a little sideways, which on a camera
/// is a slow drift that never settles.
[[nodiscard]] glm::vec2 stickVector(float rawX, float rawY, float deadzone);

/// A trigger's pull, 0 to 1, with its slack removed the same way.
///
/// Triggers rest at zero rather than centred, so only one end needs the
/// treatment, and the rescale matters more: a trigger's first millimetre is
/// where a camera zoom wants its finest control.
[[nodiscard]] float triggerFraction(float raw, float deadzone);

/// The controller the client is listening to, if any.
///
/// One at a time. A second pad plugged in while the first is connected is
/// ignored rather than merged: two people's sticks fighting over one character
/// is not a feature, and the first-in rule is what every console does.
class Gamepad {
public:
    static Gamepad& getInstance();

    /// Brings up SDL's controller subsystem and opens whatever is already
    /// plugged in. Safe to call more than once. Returns false only if the
    /// subsystem itself will not start, which leaves every read below
    /// answering as though nothing is connected.
    bool init();

    /// The device events, which arrive on the same queue as everything else.
    /// Anything that is not a controller event is ignored.
    void handleEvent(const SDL_Event& event);

    /// Samples the pad. Once a frame, before anything reads it, so that the
    /// just-pressed edges below describe this frame and not the last one.
    void update();

    void shutdown();

    [[nodiscard]] bool isConnected() const { return pad_ != nullptr; }
    /// What the pad calls itself, for the log and the settings panel. Empty
    /// when nothing is connected.
    [[nodiscard]] const std::string& name() const { return name_; }

    /// The sticks, deadzoned. Y is positive down, as SDL reports it.
    [[nodiscard]] glm::vec2 leftStick() const { return leftStick_; }
    [[nodiscard]] glm::vec2 rightStick() const { return rightStick_; }
    [[nodiscard]] float leftTrigger() const { return leftTrigger_; }
    [[nodiscard]] float rightTrigger() const { return rightTrigger_; }

    /// Whether a button is down. There is no just-pressed here on purpose:
    /// every button this client answers goes through core::Input's virtual
    /// keys, and that is where an edge is worked out - a second copy of the
    /// same reasoning would be one more thing to keep in step.
    [[nodiscard]] bool held(SDL_GamepadButton button) const;

    /// One finger on the pad's touch surface. Position runs 0 to 1 across and
    /// down from the top-left corner, as SDL reports it.
    struct TouchFinger {
        bool down = false;
        glm::vec2 position{0.0f};
    };

    /// Which family of pad this is, for the names on its buttons and for the
    /// few extras only some of them have.
    ///
    /// SDL resolves every pad to one shape, so this changes no behaviour that
    /// all of them share: it decides what a button is *called* - the same
    /// button is A on an Xbox pad, Cross on a PlayStation one and B on a
    /// Switch one - and which of the extra buttons are there to be used.
    enum class Kind {
        Unknown,
        Xbox,
        PlayStation,
        Nintendo,
        SteamDeck,
        Luna,
        Stadia,
        Shield,
        Virtual,
    };
    [[nodiscard]] Kind kind() const { return kind_; }

    /// Whether this pad actually has a button. SDL's mapping says so, which
    /// is how the paddles and the share button are offered only to the pads
    /// that have them rather than bound into the air on the ones that do not.
    [[nodiscard]] bool hasButton(SDL_GamepadButton button) const;

    /// Whether the pad has a touch surface. PlayStation pads do; most others
    /// do not, and read as no finger down.
    [[nodiscard]] bool hasTouchpad() const { return touchFingers_ > 0; }
    /// The first or second finger on the touch surface. A finger the pad
    /// cannot track reads as not down.
    [[nodiscard]] const TouchFinger& touch(int finger) const;

    /// How much of each stick counts as rest. A worn stick that no longer
    /// centres needs a wider one, which is why this is a setting and not a
    /// constant.
    void setStickDeadzone(float fraction);
    [[nodiscard]] float stickDeadzone() const { return stickDeadzone_; }

    /// A pad's own mapping file, for a controller SDL does not already know.
    /// Returns how many mappings the file added, or -1 if it could not be
    /// read. SDL ships a table covering the common pads, so this is for the
    /// ones it does not cover rather than for the ones it does.
    int addMappingsFromFile(const std::string& path);

    /// Everything a report needs to say about the pad in one line.
    [[nodiscard]] std::string describe() const;

private:
    Gamepad() = default;
    ~Gamepad() = default;
    Gamepad(const Gamepad&) = delete;
    Gamepad& operator=(const Gamepad&) = delete;

    /// Opens the device at a joystick index, if nothing is open already.
    void openDevice(SDL_JoystickID joystickIndex);
    /// Closes whatever is open and forgets its state, so a disconnected pad
    /// cannot leave a button held down forever.
    void closeDevice();
    /// Looks for anything already plugged in. Used at startup and after a
    /// disconnect, so unplugging the second pad of two leaves the first live.
    void openFirstAvailable();

    /// Works out the family from SDL's own type, falling back to the USB
    /// vendor and product for the pads SDL has no type for.
    [[nodiscard]] static Kind kindOf(SDL_Gamepad* pad);

    SDL_Gamepad* pad_ = nullptr;
    Kind kind_ = Kind::Unknown;
    SDL_JoystickID instanceId_ = -1;
    std::string name_;

    static constexpr int kButtonCount = SDL_GAMEPAD_BUTTON_COUNT;
    std::array<bool, kButtonCount> current_{};

    /// The first touch surface's fingers. Two, because that is what a
    /// PlayStation pad tracks, and a two-finger click is the right click.
    static constexpr int kTouchFingers = 2;
    std::array<TouchFinger, kTouchFingers> touch_{};
    /// How many of those the connected pad actually tracks.
    int touchFingers_ = 0;
    /// Which of the pad's touch surfaces is the pointer's - the right-hand
    /// one, where there is more than one.
    int touchpad_ = 0;

    glm::vec2 leftStick_{0.0f};
    glm::vec2 rightStick_{0.0f};
    float leftTrigger_ = 0.0f;
    float rightTrigger_ = 0.0f;

    /// SDL's own suggested resting slack is about a quarter of the range for
    /// a worn stick and an eighth for a new one. This sits between them: low
    /// enough that a small push registers, high enough that a pad resting on
    /// a desk does not walk the character into a wall over a lunch break.
    float stickDeadzone_ = 0.18f;
    /// Triggers rest cleanly and their travel is all useful, so this only has
    /// to reject the last fraction of a millimetre of spring. A constant
    /// rather than a setting: nothing has yet wanted it moved, and a knob
    /// nobody turns is a knob that is wrong when someone finally does.
    static constexpr float kTriggerDeadzone = 0.08f;

    bool initialised_ = false;
};

/// The one instance, reached the way the touch controls are.
Gamepad& gamepad();

}  // namespace core
}  // namespace wowee
