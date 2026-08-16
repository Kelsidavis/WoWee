#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Combat Maneuver Group catalog (.wcmg) -
// novel replacement for the hardcoded class-mutex tables
// that the WoW client uses to grey out incompatible
// action-bar buttons (only one Warrior stance active at
// a time, only one Hunter aspect, only one DK presence,
// only one Druid form). Each entry is one mutually-
// exclusive spell group with its member spell IDs.
//
// Cross-references with previously-added formats:
//   WCHC: classMask uses the WCHC class-bit convention
//         (1=Warrior / 2=Paladin / 4=Hunter / 8=Rogue /
//         16=Priest / 32=DK / 64=Shaman / 128=Mage /
//         256=Warlock / 1024=Druid).
//   WSPL: members[] entries are spell IDs that must
//         resolve in the WSPL catalog. Validator can
//         optionally cross-check with --catalog-pluck if
//         WSPL is available.
//   WACT: WACT button entries with spellId in any
//         exclusive WCMG group will display the "active
//         state" outline when that spell is the
//         currently-applied one for the player.
//
// Binary layout (little-endian):
//   magic[4]            = "WCMG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     groupId (uint32)
//     nameLen + name
//     descLen + description
//     classMask (uint32)
//     categoryKind (uint8)       - Stance / Form / Aspect /
//                                   Presence / Posture /
//                                   Sigil
//     exclusive (uint8)          - 0/1 bool, only one
//                                   active at a time
//     pad0 (uint8) / pad1 (uint8)
//     iconColorRGBA (uint32)
//     memberCount (uint32)
//     members (memberCount × uint32 spell IDs)
struct WoweeCombatManeuvers {
    enum CategoryKind : uint8_t {
        Stance   = 0,    // Warrior Battle/Def/Berserker
        Form     = 1,    // Druid Bear/Cat/Tree/Moonkin
        Aspect   = 2,    // Hunter Hawk/Cheetah/Pack/Viper
        Presence = 3,    // DK Frost/Unholy/Blood
        Posture  = 4,    // generic posture (e.g. siege
                          // engine driver/gunner)
        Sigil    = 5,    // DK Sigils (one ground-anchored
                          // at a time)
    };

    struct Entry {
        uint32_t groupId = 0;
        std::string name;
        std::string description;
        uint32_t classMask = 0;
        uint8_t categoryKind = Stance;
        uint8_t exclusive = 1;     // default: mutex group
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
        std::vector<uint32_t> members;   // spell IDs
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t groupId) const;

    // Returns all maneuver groups available to a class
    // (used by the action-bar UI to compute which spells
    // share a mutex bucket and grey accordingly).
    [[nodiscard]] std::vector<const Entry*> findByClass(uint32_t classBit) const;

    // Returns the group containing the given spell ID, if
    // any. Used by the action-bar update path to know
    // whether casting a spell should clear a "currently
    // active" outline elsewhere on the bar.
    [[nodiscard]] const Entry* findGroupForSpell(uint32_t spellId) const;
};

class WoweeCombatManeuversLoader {
public:
    static bool save(const WoweeCombatManeuvers& cat,
                     const std::string& basePath);
    static WoweeCombatManeuvers load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cmg* variants.
    //
    //   makeWarrior   - 1 entry: WarriorStances (3 spells:
    //                    Battle / Defensive / Berserker)
    //                    classMask=1, exclusive.
    //   makeDruid     - 2 entries: DruidShapeshiftForms
    //                    (5 spells: Bear / Cat / Travel /
    //                    Tree of Life / Moonkin) and
    //                    DruidFlightForms (2 spells:
    //                    Flight Form / Swift Flight Form)
    //                    classMask=1024, both exclusive.
    //   makeAllMutex  - 4 entries spanning all classes
    //                    that have mutex groups: Warrior
    //                    stances / Hunter aspects / DK
    //                    presences / Druid forms (one
    //                    representative group per class).
    static WoweeCombatManeuvers makeWarrior(const std::string& catalogName);
    static WoweeCombatManeuvers makeDruid(const std::string& catalogName);
    static WoweeCombatManeuvers makeAllMutex(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
