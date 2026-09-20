// The controller's stick arithmetic, and the scheme its buttons carry.
//
// The device half cannot be tested without a device. These are the parts that
// decide what a reading means, which is where a stick feels wrong or right:
// the deadzone, the rescale, and whether two buttons have been given the same
// job by accident.
#include <catch_amalgamated.hpp>

#include "core/gamepad.hpp"
#include "ui/gamepad_controls.hpp"

#include <cmath>
#include <map>
#include <set>
#include <span>
#include <string>

using wowee::core::axisFraction;
using wowee::core::stickVector;
using wowee::core::triggerFraction;

namespace {
float length(const glm::vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
}  // namespace

TEST_CASE("an axis reading is a fraction of full deflection") {
    CHECK(axisFraction(0) == Catch::Approx(0.0f));
    CHECK(axisFraction(32767) == Catch::Approx(1.0f));
    // SDL's range is asymmetric: -32768 is one step past the positive end.
    // Without the clamp this reads -1.00003, and everything downstream
    // multiplies by it.
    CHECK(axisFraction(-32768) == Catch::Approx(-1.0f));
    CHECK(axisFraction(16384) == Catch::Approx(0.5f).margin(0.001f));
}

TEST_CASE("a resting stick reads as centred") {
    CHECK(length(stickVector(0.0f, 0.0f, 0.2f)) == Catch::Approx(0.0f));
    CHECK(length(stickVector(0.15f, 0.0f, 0.2f)) == Catch::Approx(0.0f));
    // Inside the circle, not inside the square: 0.15 on each axis is 0.21
    // away from centre, which a per-axis deadzone would call rest.
    CHECK(length(stickVector(0.15f, 0.15f, 0.2f)) > 0.0f);
}

TEST_CASE("the deadzone is taken radially, not per axis") {
    // A stick pushed diagonally to its corner must not read longer than one,
    // or a character walks faster north-east than north.
    CHECK(length(stickVector(1.0f, 1.0f, 0.2f)) == Catch::Approx(1.0f));
    CHECK(length(stickVector(-1.0f, 1.0f, 0.2f)) == Catch::Approx(1.0f));
    // And the direction of the push is kept.
    const glm::vec2 diagonal = stickVector(0.7f, 0.7f, 0.2f);
    CHECK(diagonal.x == Catch::Approx(diagonal.y));
}

TEST_CASE("a straight push stays straight") {
    // A worn stick reads a little sideways when pushed forward. On a camera
    // that is a drift that never settles, so the cross axis has to come out
    // of the deadzone as zero.
    const glm::vec2 forward = stickVector(0.0f, -0.8f, 0.2f);
    CHECK(forward.x == Catch::Approx(0.0f));
    CHECK(forward.y < 0.0f);
}

TEST_CASE("what is left of the range starts at zero and ends at one") {
    // The first movement outside the deadzone must be the slowest one, not a
    // jump to a fifth of full speed.
    const float justOutside = length(stickVector(0.0f, 0.21f, 0.2f));
    CHECK(justOutside > 0.0f);
    CHECK(justOutside < 0.05f);
    CHECK(length(stickVector(0.0f, 1.0f, 0.2f)) == Catch::Approx(1.0f));
    // Half way along what is left is half speed.
    CHECK(length(stickVector(0.0f, 0.6f, 0.2f)) == Catch::Approx(0.5f).margin(0.001f));
}

TEST_CASE("a deadzone of zero leaves the reading alone") {
    CHECK(length(stickVector(0.0f, 0.3f, 0.0f)) == Catch::Approx(0.3f));
}

TEST_CASE("a trigger rests at nothing and pulls to one") {
    CHECK(triggerFraction(0.0f, 0.08f) == Catch::Approx(0.0f));
    CHECK(triggerFraction(0.05f, 0.08f) == Catch::Approx(0.0f));
    CHECK(triggerFraction(1.0f, 0.08f) == Catch::Approx(1.0f));
    CHECK(triggerFraction(0.54f, 0.08f) == Catch::Approx(0.5f).margin(0.001f));
    // A trigger cannot be pushed the other way, whatever the driver says.
    CHECK(triggerFraction(-0.4f, 0.08f) == Catch::Approx(0.0f));
}

TEST_CASE("no button is given two jobs and no job two buttons") {
    const auto bindings = wowee::ui::padBindings();
    REQUIRE(!bindings.empty());

    std::set<int> buttons;
    std::set<int> keys;
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        const auto& binding = bindings[i];
        INFO(binding.what);
        // A button bound twice runs whichever line is later and looks like a
        // dead button; a key bound twice is two pad buttons that do the same
        // thing, which is a wasted button on a device that has ten.
        CHECK(buttons.insert(static_cast<int>(binding.button)).second);
        CHECK(keys.insert(static_cast<int>(binding.key)).second);
        CHECK(binding.button >= 0);
        CHECK(binding.button < SDL_GAMEPAD_BUTTON_COUNT);
        CHECK(binding.key > SDL_SCANCODE_UNKNOWN);
        CHECK(binding.key < SDL_SCANCODE_COUNT);
        // Every row is shown to a player, so every row needs a name.
        REQUIRE(binding.what != nullptr);
        CHECK(binding.what[0] != '\0');
    }
}

TEST_CASE("the scheme reaches the whole action bar") {
    // Six slots on the pad and six more behind the shift the left bumper
    // holds. Without the modifier in the table the other six are unreachable,
    // which is the kind of gap that is only noticed at level 40.
    const auto bindings = wowee::ui::padBindings();
    int actionKeys = 0;
    bool hasModifier = false;
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        const SDL_Scancode key = bindings[i].key;
        if (key >= SDL_SCANCODE_1 && key <= SDL_SCANCODE_6) ++actionKeys;
        if (key == SDL_SCANCODE_LSHIFT) hasModifier = true;
    }
    CHECK(actionKeys == 6);
    CHECK(hasModifier);
}

TEST_CASE("the keys the pad holds are ones the client answers") {
    // The client polls these by scancode. A binding to a key nothing reads is
    // a button that does nothing, and the table is the only place that would
    // say so.
    const auto bindings = wowee::ui::padBindings();
    const std::set<int> answered = {
        SDL_SCANCODE_SPACE, SDL_SCANCODE_TAB, SDL_SCANCODE_NUMLOCKCLEAR,
        SDL_SCANCODE_LSHIFT, SDL_SCANCODE_X,
        SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3,
        SDL_SCANCODE_4, SDL_SCANCODE_5, SDL_SCANCODE_6,
    };
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        INFO(bindings[i].what);
        CHECK(answered.count(static_cast<int>(bindings[i].key)) == 1);
    }
}

TEST_CASE("the pointer does not move on its own") {
    const glm::vec2 still = wowee::ui::GamepadControls::pointerStep(0.0f, 0.0f, 0.016f);
    CHECK(still.x == Catch::Approx(0.0f));
    CHECK(still.y == Catch::Approx(0.0f));
    // A frame that took no time moves it nowhere either, rather than dividing
    // by it.
    const glm::vec2 frozen = wowee::ui::GamepadControls::pointerStep(1.0f, 0.0f, 0.0f);
    CHECK(frozen.x == Catch::Approx(0.0f));
}

TEST_CASE("a gentle push moves the pointer much more slowly than a full one") {
    // The reason for the curve. Linear, the speed that can land on a small
    // button cannot cross a window, and the speed that crosses a window
    // cannot land on the button.
    const float dt = 1.0f;
    const float slow = wowee::ui::GamepadControls::pointerStep(0.25f, 0.0f, dt).x;
    const float fast = wowee::ui::GamepadControls::pointerStep(1.0f, 0.0f, dt).x;
    CHECK(slow > 0.0f);
    // A quarter of the stick is a sixteenth of the speed.
    CHECK(fast / slow == Catch::Approx(16.0f).margin(0.1f));
    // And a full push crosses a 1280 wide window in under two seconds.
    CHECK(fast > 640.0f);
}

TEST_CASE("the pointer goes where the stick points") {
    const glm::vec2 diagonal = wowee::ui::GamepadControls::pointerStep(0.5f, 0.5f, 0.1f);
    CHECK(diagonal.x == Catch::Approx(diagonal.y));
    CHECK(diagonal.x > 0.0f);
    const glm::vec2 up = wowee::ui::GamepadControls::pointerStep(0.0f, -0.8f, 0.1f);
    CHECK(up.x == Catch::Approx(0.0f));
    CHECK(up.y < 0.0f);
}

TEST_CASE("a stick reading past its corner does not outrun the curve") {
    // stickVector caps at one, but this is a static taking whatever it is
    // given, and a driver that reports 1.4 on the diagonal would otherwise
    // move the pointer twice as fast diagonally as straight.
    const float straight = wowee::ui::GamepadControls::pointerStep(1.0f, 0.0f, 0.1f).x;
    const glm::vec2 corner = wowee::ui::GamepadControls::pointerStep(1.0f, 1.0f, 0.1f);
    const float diagonal = std::sqrt(corner.x * corner.x + corner.y * corner.y);
    CHECK(diagonal == Catch::Approx(straight).margin(0.001f));
}

TEST_CASE("the pointer travels the same distance however the frame is cut") {
    // A step proportional to the frame time, so a 144Hz screen and a 30Hz one
    // move the pointer at the same speed.
    const float oneStep = wowee::ui::GamepadControls::pointerStep(0.6f, 0.0f, 0.2f).x;
    float many = 0.0f;
    for (int i = 0; i < 10; ++i) many += wowee::ui::GamepadControls::pointerStep(0.6f, 0.0f, 0.02f).x;
    CHECK(many == Catch::Approx(oneStep).margin(0.001f));
}

TEST_CASE("the extras are extra, and reach the rest of the bar") {
    // The base scheme reaches six action slots without a modifier. A pad with
    // four back buttons should reach four more with nothing held, which is
    // most of what a paddle is for.
    const auto base = wowee::ui::padBindings();
    const auto extras = wowee::ui::padExtraBindings();
    REQUIRE(!extras.empty());

    std::set<int> buttons;
    std::set<int> keys;
    for (std::size_t i = 0; i < base.size(); ++i) {
        buttons.insert(static_cast<int>(base[i].button));
        keys.insert(static_cast<int>(base[i].key));
    }
    for (std::size_t i = 0; i < extras.size(); ++i) {
        INFO(extras[i].what);
        // Nothing here may repeat the base: a button bound twice runs
        // whichever line is later, and a key bound twice is a wasted button.
        CHECK(buttons.insert(static_cast<int>(extras[i].button)).second);
        CHECK(keys.insert(static_cast<int>(extras[i].key)).second);
        CHECK(extras[i].what[0] != '\0');
    }

    // Slots 7 to 10 of the main bar, which the base scheme cannot reach at all.
    int paddleSlots = 0;
    for (std::size_t i = 0; i < extras.size(); ++i) {
        const SDL_Scancode key = extras[i].key;
        if (key == SDL_SCANCODE_7 || key == SDL_SCANCODE_8 ||
            key == SDL_SCANCODE_9 || key == SDL_SCANCODE_0) {
            ++paddleSlots;
        }
    }
    CHECK(paddleSlots == 4);
}

TEST_CASE("a pad is told what its own buttons are called") {
    using Kind = wowee::core::Gamepad::Kind;
    using wowee::ui::padButtonLabel;

    // The same button under the same thumb, named as that pad prints it.
    // Nintendo is the one that matters: its letters are laid out the other
    // way round, so the bottom button - the one this client jumps on - is
    // printed B, and telling a Switch player to press A would send them to
    // the button that closes windows.
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_SOUTH, Kind::Xbox)) == "A");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_SOUTH, Kind::PlayStation)) == "Cross");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_SOUTH, Kind::Nintendo)) == "B");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_EAST, Kind::Nintendo)) == "A");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_WEST, Kind::Nintendo)) == "Y");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_NORTH, Kind::Nintendo)) == "X");

    // The Deck keeps the Xbox letters and adds four on the back.
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_SOUTH, Kind::SteamDeck)) == "A");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1, Kind::SteamDeck)) == "L4");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_LEFT_PADDLE1, Kind::SteamDeck)) == "R4");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2, Kind::SteamDeck)) == "L5");
    CHECK(std::string(padButtonLabel(SDL_GAMEPAD_BUTTON_LEFT_PADDLE2, Kind::SteamDeck)) == "R5");

    // A pad SDL has no family for still gets a name for every button, or the
    // settings panel would list a scheme with holes in it.
    for (const Kind kind : {Kind::Unknown, Kind::Xbox, Kind::PlayStation, Kind::Nintendo,
                            Kind::SteamDeck, Kind::Luna, Kind::Stadia, Kind::Shield,
                            Kind::Virtual}) {
        for (const wowee::ui::PadBinding& row : wowee::ui::padBindings()) {
            INFO(static_cast<int>(kind) << " " << row.what);
            CHECK(padButtonLabel(row.button, kind)[0] != '\0');
        }
        const auto extraRows = wowee::ui::padExtraBindings();
        for (std::size_t i = 0; i < extraRows.size(); ++i) {
            INFO(static_cast<int>(kind) << " " << extraRows[i].what);
            CHECK(padButtonLabel(extraRows[i].button, kind)[0] != '\0');
        }
    }
}

TEST_CASE("every bound button has a name a player would recognise") {
    // The settings panel lists the scheme off this table. A button with no
    // label is silently dropped from that list, which is how a control scheme
    // comes to be missing the one line someone was looking for.
    const auto bindings = wowee::ui::padBindings();
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        INFO(bindings[i].what);
        const char* label = wowee::ui::padButtonLabel(bindings[i].button,
                                                     wowee::core::Gamepad::Kind::Xbox);
        REQUIRE(label != nullptr);
        CHECK(label[0] != '\0');
    }
    // And the two that are not in the table, because they go through ImGui
    // rather than through a scancode, are still named.
    CHECK(std::string(wowee::ui::padButtonLabel(SDL_GAMEPAD_BUTTON_EAST,
                                               wowee::core::Gamepad::Kind::Xbox)) == "B");
    CHECK(std::string(wowee::ui::padButtonLabel(SDL_GAMEPAD_BUTTON_START,
                                               wowee::core::Gamepad::Kind::Xbox)) == "Start");
    CHECK(std::string(wowee::ui::padButtonLabel(SDL_GAMEPAD_BUTTON_BACK,
                                               wowee::core::Gamepad::Kind::Xbox)) == "Back");
}

TEST_CASE("a finger landing on the touchpad does not move the pointer") {
    // Lifting at one edge and landing at the other is how a trackpad crosses
    // a screen, so a landing must be worth nothing.
    wowee::ui::TouchTrail trail;
    const glm::vec2 landed = trail.follow(true, glm::vec2(0.9f, 0.5f));
    CHECK(landed.x == Catch::Approx(0.0f));
    CHECK(landed.y == Catch::Approx(0.0f));

    const glm::vec2 slid = trail.follow(true, glm::vec2(0.7f, 0.6f));
    CHECK(slid.x == Catch::Approx(-0.2f));
    CHECK(slid.y == Catch::Approx(0.1f));

    // Lifted, then down somewhere else: a new landing, not a slide.
    CHECK(trail.follow(false, glm::vec2(0.7f, 0.6f)).x == Catch::Approx(0.0f));
    CHECK(trail.follow(true, glm::vec2(0.1f, 0.5f)).x == Catch::Approx(0.0f));
}

TEST_CASE("a reset touch trail treats the next reading as a landing") {
    wowee::ui::TouchTrail trail;
    (void)trail.follow(true, glm::vec2(0.2f, 0.2f));
    trail.reset();
    CHECK(trail.follow(true, glm::vec2(0.8f, 0.8f)).x == Catch::Approx(0.0f));
}

TEST_CASE("a touchpad slide moves the pointer the same distance in every direction") {
    // The full width of the touchpad is the full width of the window.
    const glm::vec2 across = wowee::ui::GamepadControls::touchStep(glm::vec2(1.0f, 0.0f), 1280.0f);
    CHECK(across.x == Catch::Approx(1280.0f));
    CHECK(across.y == Catch::Approx(0.0f));

    // A quarter of the touchpad's width is half its height, since it is twice
    // as wide as tall; that same finger travel must move the pointer equally.
    const glm::vec2 right = wowee::ui::GamepadControls::touchStep(glm::vec2(0.25f, 0.0f), 1280.0f);
    const glm::vec2 down = wowee::ui::GamepadControls::touchStep(glm::vec2(0.0f, 0.5f), 1280.0f);
    CHECK(down.y == Catch::Approx(right.x));

    // No window, nowhere to move.
    CHECK(wowee::ui::GamepadControls::touchStep(glm::vec2(1.0f, 1.0f), 0.0f).x == Catch::Approx(0.0f));
}

TEST_CASE("every pad button has a binding name of its own") {
    // The names are saved in bindings.cfg and compared as strings, so two
    // buttons sharing one would be bound together.
    std::set<std::string> names;
    for (int b = 0; b < SDL_GAMEPAD_BUTTON_TOUCHPAD; ++b) {
        INFO(b);
        const std::string name = wowee::ui::padKeyName(static_cast<SDL_GamepadButton>(b));
        REQUIRE_FALSE(name.empty());
        CHECK(name.rfind("PAD", 0) == 0);
        CHECK(names.insert(name).second);
    }
    // The touchpad click is the pointer's, and is not bindable.
    CHECK(std::string(wowee::ui::padKeyName(SDL_GAMEPAD_BUTTON_TOUCHPAD)).empty());
    // Retail's spelling.
    CHECK(std::string(wowee::ui::padKeyName(SDL_GAMEPAD_BUTTON_SOUTH)) == "PAD1");
    CHECK(std::string(wowee::ui::padKeyName(SDL_GAMEPAD_BUTTON_NORTH)) == "PAD4");
    CHECK(std::string(wowee::ui::padKeyName(SDL_GAMEPAD_BUTTON_DPAD_UP)) == "PADDUP");
    CHECK(std::string(wowee::ui::padKeyName(SDL_GAMEPAD_BUTTON_START)) == "PADFORWARD");
}

TEST_CASE("a button bound to a command the client polls presses the key it polls") {
    const SDL_Scancode bar[] = {
        SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4,
        SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
        SDL_SCANCODE_9, SDL_SCANCODE_0, SDL_SCANCODE_MINUS, SDL_SCANCODE_EQUALS,
    };
    for (int i = 0; i < 12; ++i) {
        const std::string command = "ACTIONBUTTON" + std::to_string(i + 1);
        INFO(command);
        CHECK(wowee::ui::padClientKeyFor(command) == bar[i]);
    }
    CHECK(wowee::ui::padClientKeyFor("MOVEFORWARD") == SDL_SCANCODE_W);
    CHECK(wowee::ui::padClientKeyFor("JUMP") == SDL_SCANCODE_SPACE);
    // A command the interface performs has no key: its script runs instead.
    CHECK(wowee::ui::padClientKeyFor("TOGGLEFRIENDSTAB") == SDL_SCANCODE_UNKNOWN);
    CHECK(wowee::ui::padClientKeyFor("") == SDL_SCANCODE_UNKNOWN);
}

TEST_CASE("binding a button to what the default scheme gave it changes nothing") {
    // A player who binds X to ACTIONBUTTON1 in the panel has asked for what X
    // already did, and must get the same key rather than a different one.
    const std::map<SDL_GamepadButton, std::string> sameCommand = {
        {SDL_GAMEPAD_BUTTON_SOUTH,             "JUMP"},
        {SDL_GAMEPAD_BUTTON_WEST,             "ACTIONBUTTON1"},
        {SDL_GAMEPAD_BUTTON_NORTH,             "ACTIONBUTTON2"},
        {SDL_GAMEPAD_BUTTON_DPAD_UP,       "ACTIONBUTTON3"},
        {SDL_GAMEPAD_BUTTON_DPAD_RIGHT,    "ACTIONBUTTON4"},
        {SDL_GAMEPAD_BUTTON_DPAD_DOWN,     "ACTIONBUTTON5"},
        {SDL_GAMEPAD_BUTTON_DPAD_LEFT,     "ACTIONBUTTON6"},
        {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, "TARGETNEARESTENEMY"},
        {SDL_GAMEPAD_BUTTON_LEFT_STICK,     "TOGGLEAUTORUN"},
    };
    const auto bindings = wowee::ui::padBindings();
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        const auto it = sameCommand.find(bindings[i].button);
        if (it == sameCommand.end()) continue;
        INFO(bindings[i].what);
        CHECK(wowee::ui::padClientKeyFor(it->second) == bindings[i].key);
    }
}

TEST_CASE("a row's command and its key say the same thing") {
    // The scheme's rows now carry the interface's name for what they do, so
    // that the binding table can be seeded from this one table rather than a
    // second copy of it. The two halves of a row have to agree: the command
    // is looked up through padClientKeyFor when the press arrives as a
    // binding, and if that answers a different key from the one the row
    // presses directly, the same button does two different things depending
    // on whether the player has ever opened the Key Bindings panel.
    const auto check = [](std::span<const wowee::ui::PadBinding> rows) {
        for (const wowee::ui::PadBinding& row : rows) {
            const char* command = row.command;
            if (!command || !*command) continue;
            INFO(row.what << " is " << command);
            CHECK(wowee::ui::padClientKeyFor(command) == row.key);
        }
    };
    check(wowee::ui::padBindings());
    check(wowee::ui::padExtraBindings());
}

TEST_CASE("the rows with no command are the ones the interface has no word for") {
    // Three: the bumper that is only a modifier, the button that raises the
    // pointer, and the stick click that sits the character down. Everything
    // else a pad does has a name in the binding table and belongs in it, and
    // a row that quietly loses its command would vanish from the Key Bindings
    // panel without anything else noticing.
    std::set<int> commandless;
    for (const wowee::ui::PadBinding& row : wowee::ui::padBindings()) {
        if (row.command == nullptr || row.command[0] == '\0') {
            commandless.insert(static_cast<int>(row.button));
        }
    }
    CHECK(commandless.count(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) == 1);
    CHECK(commandless.count(SDL_GAMEPAD_BUTTON_RIGHT_STICK) == 1);
    CHECK(commandless.size() == 2);

    // And every extra has one: a paddle is an action slot and the share
    // button is a screenshot, both of which the interface can name.
    for (const wowee::ui::PadBinding& row : wowee::ui::padExtraBindings()) {
        INFO(row.what);
        REQUIRE(row.command != nullptr);
        CHECK(row.command[0] != '\0');
    }
}
