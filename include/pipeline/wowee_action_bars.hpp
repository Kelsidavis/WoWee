#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Action Bar Layout catalog (.wact) - novel
// replacement for the hardcoded per-class default action
// bar bindings in the WoW client. Defines which abilities
// auto-populate which action button slots when a new
// character is created or a class is reset.
//
// Each entry binds one (classMask, buttonSlot) pair to a
// spell or item. A Warrior's button 1 might bind Heroic
// Strike, button 2 Charge, button 3 Battle Shout, etc.
// New characters of that class get those buttons pre-
// populated so the action bar isn't empty on first login.
//
// Distinct from WKBD (Keybindings) which maps physical
// keys to action button slots - WACT maps action button
// slots to abilities. The two together complete the
// default-control configuration: Key 1 -> Action Slot 1
// (WKBD) -> Heroic Strike (WACT).
//
// Cross-references with previously-added formats:
//   WCHC: classMask uses the same bit layout as WCHC
//         class IDs (Warrior=0x01, Paladin=0x02, ...).
//   WSPL: spellId references the WSPL spell entry that
//         the button casts when triggered.
//   WIT:  itemId references a WIT item entry for item
//         macro bindings (Hearthstone in slot 12, etc.).
//
// Binary layout (little-endian):
//   magic[4]            = "WACT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     bindingId (uint32)
//     nameLen + name
//     descLen + description
//     classMask (uint32)
//     spellId (uint32)
//     itemId (uint32)
//     buttonSlot (uint8) / barMode (uint8) / pad[2]
//     iconColorRGBA (uint32)
struct WoweeActionBar {
    enum BarMode : uint8_t {
        Main      = 0,    // standard 12-button main bar (slots 0-11)
        Pet       = 1,    // hunter/warlock pet action bar
        Vehicle   = 2,    // mounted/vehicle action bar
        Stance1   = 3,    // warrior battle / druid bear stance
        Stance2   = 4,    // warrior defensive / druid cat
        Stance3   = 5,    // warrior berserker / druid tree
        Custom    = 6,    // server-custom bar overlay
    };

    struct Entry {
        uint32_t bindingId = 0;
        std::string name;
        std::string description;
        uint32_t classMask = 0;
        uint32_t spellId = 0;
        uint32_t itemId = 0;            // 0 if spell-only
        uint8_t buttonSlot = 0;         // 0..143 (12 bars × 12 slots)
        uint8_t barMode = Main;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bindingId) const;

    // Return all entries for a given class on a specific
    // bar mode, in buttonSlot order. Used by character
    // creation to populate the action bar with defaults.
    [[nodiscard]] std::vector<const Entry*> findByClass(uint32_t classBit,
                                           uint8_t barMode) const;

    static const char* barModeName(uint8_t m);
};

class WoweeActionBarLoader {
public:
    static bool save(const WoweeActionBar& cat,
                     const std::string& basePath);
    static WoweeActionBar load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-act* variants.
    //
    //   makeWarrior - 10 Warrior starter bindings on the
    //                  Main bar (Heroic Strike, Charge,
    //                  Rend, Thunder Clap, Battle Shout,
    //                  Sunder Armor, Mocking Blow, etc).
    //   makeMage    - 10 Mage starter bindings on the Main
    //                  bar (Fireball, Frostbolt, Frost
    //                  Nova, Polymorph, Mage Armor, etc).
    //   makeHunterPet - 10 Hunter Pet-bar bindings using
    //                  barMode=Pet (Attack, Follow, Stay,
    //                  Aggressive/Defensive/Passive
    //                  stances, Bite, Claw, etc).
    static WoweeActionBar makeWarrior(const std::string& catalogName);
    static WoweeActionBar makeMage(const std::string& catalogName);
    static WoweeActionBar makeHunterPet(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
