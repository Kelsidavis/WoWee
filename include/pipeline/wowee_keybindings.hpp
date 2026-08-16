#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Keybinding catalog (.wkbd) - novel replacement
// for Blizzard's KeyBinding.dbc plus the AzerothCore-style
// default-keybind SQL data. Defines the key bindings shipped
// with the game: movement (W/A/S/D), targeting (Tab),
// action bars (1-9, 0, -, =), UI panels (C/I/B/P/N/L), chat
// (Enter), camera control (Insert/Delete), macro slots, etc.
//
// Each binding has an internal action name ("MOVE_FORWARD"),
// a primary key, an optional alternate key, a category for
// the keybindings UI grouping, and a flag indicating whether
// the user can override it. Hardcoded engine bindings (alt-
// F4, ESC) set isUserOverridable=0 so the rebind dialog
// can't accidentally break them.
//
// This catalog has no cross-references to other formats -
// it's a self-contained binding map between strings and
// keys, consumed directly by the input layer.
//
// Binary layout (little-endian):
//   magic[4]            = "WKBD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     bindingId (uint32)
//     actionLen + actionName
//     descLen + description
//     primaryLen + defaultKey
//     altLen + alternateKey
//     category (uint8) / isUserOverridable (uint8) /
//       sortOrder (uint8) / pad[1]
struct WoweeKeyBinding {
    enum Category : uint8_t {
        Movement     = 0,    // WASD, jump, walk
        Combat       = 1,    // attack, spell cast, action bars
        Targeting    = 2,    // tab, focus, assist
        Camera       = 3,    // mouse look, zoom, pitch
        UIPanels     = 4,    // character/inv/bags/spellbook
        Chat         = 5,    // enter, slash, reply
        Macro        = 6,    // macro slots
        Bar          = 7,    // bar shift / page
        Other        = 8,    // misc / scripted
    };

    struct Entry {
        uint32_t bindingId = 0;
        std::string actionName;       // "MOVE_FORWARD"
        std::string description;
        std::string defaultKey;       // "W"
        std::string alternateKey;     // "" if none
        uint8_t category = Movement;
        uint8_t isUserOverridable = 1;
        uint8_t sortOrder = 0;        // display order within category
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bindingId) const;
    [[nodiscard]] const Entry* findByActionName(const std::string& actionName) const;

    static const char* categoryName(uint8_t c);
};

class WoweeKeyBindingLoader {
public:
    static bool save(const WoweeKeyBinding& cat,
                     const std::string& basePath);
    static WoweeKeyBinding load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-kbd* variants.
    //
    //   makeStarter   - 3 essential bindings (MOVE_FORWARD,
    //                    TARGET_NEAREST_ENEMY, TOGGLE_CHARACTER).
    //   makeMovement  - 8 movement bindings (4-directional,
    //                    JUMP, TOGGLE_AUTORUN, TOGGLE_WALK,
    //                    SIT_OR_STAND).
    //   makeUIPanels  - 10 UI toggle bindings
    //                    (Character/Inventory/Bags/Spellbook/
    //                    Talents/QuestLog/Friends/Guild/
    //                    MainMenu/Calendar).
    static WoweeKeyBinding makeStarter(const std::string& catalogName);
    static WoweeKeyBinding makeMovement(const std::string& catalogName);
    static WoweeKeyBinding makeUIPanels(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
