#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Unit Movement Type catalog (.wumv) - novel
// replacement for Blizzard's UnitMovement.dbc plus the
// movement-modifier portions of CreatureModelData.dbc.
// Defines movement speed types (walk / run / swim / flight
// / fly / pitch) and their per-creature multipliers, plus
// the temp speed buffs that stack on top (Sprint, Aspect of
// the Cheetah, Travel Form).
//
// baseMultiplier 1.0 = canonical default (walk = 2.5y/s,
// run = 7.0y/s, swim = 4.7y/s, flight = 7.0y/s, fly =
// 14.0y/s). maxMultiplier caps stacking - Sprint capped at
// 1.4 means even with Sprint + Aspect of the Cheetah you
// don't exceed 1.4× run speed.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the
//   movement subsystem. Spell auras (WSPL) reference
//   moveTypeId to know which speed multiplier they modify.
//
// Binary layout (little-endian):
//   magic[4]            = "WUMV"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     moveTypeId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     movementCategory (uint8) / requiresFlight (uint8) /
//       canStackBuffs (uint8) / pad[1]
//     baseSpeed (float)         - yards per second
//     baseMultiplier (float)
//     maxMultiplier (float)
//     defaultDurationMs (uint32)
//     stackingPriority (uint8) / pad[3]
struct WoweeUnitMovement {
    enum MovementCategory : uint8_t {
        Walk        = 0,    // baseline walk speed
        Run         = 1,    // baseline run speed
        Backward    = 2,    // backward movement (slower)
        Swim        = 3,    // underwater swim
        SwimBack    = 4,    // backward swim
        Turn        = 5,    // turn rate (radians/sec)
        Flight      = 6,    // ground-level flight (gryphon)
        FlightBack  = 7,    // backward flight
        Pitch       = 8,    // pitch rate (radians/sec)
        Fly         = 9,    // free-flight (drake mount)
        FlyBack     = 10,   // backward free-flight
        TempBuff    = 11,   // temporary speed modifier (Sprint)
    };

    struct Entry {
        uint32_t moveTypeId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t movementCategory = Run;
        uint8_t requiresFlight = 0;       // 1 = needs Flight Master skill
        uint8_t canStackBuffs = 1;        // 0 = exclusive (replaces existing)
        float baseSpeed = 7.0f;           // yards / second
        float baseMultiplier = 1.0f;
        float maxMultiplier = 1.4f;       // hard cap
        uint32_t defaultDurationMs = 0;   // 0 = permanent
        uint8_t stackingPriority = 0;     // higher wins on conflict
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t moveTypeId) const;

    static const char* movementCategoryName(uint8_t c);
};

class WoweeUnitMovementLoader {
public:
    static bool save(const WoweeUnitMovement& cat,
                     const std::string& basePath);
    static WoweeUnitMovement load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-umv* variants.
    //
    //   makeStarter   - 4 baseline movement types (Walk
    //                    2.5y/s, Run 7.0y/s, Swim 4.7y/s,
    //                    Turn 3.14rad/s) at canonical
    //                    WoW vanilla speeds.
    //   makeFlight    - 5 flight-related entries (Fly free
    //                    flight 14y/s, Flight ground-rail
    //                    7y/s, Pitch rate, FlyBackward,
    //                    FlightBackward).
    //   makeBuffs     - 5 temporary speed buffs (Sprint
    //                    1.4×, Aspect of the Cheetah 1.3×,
    //                    Travel Form 1.4×, Crusader Aura
    //                    1.2×, Wind Walk 1.5×).
    static WoweeUnitMovement makeStarter(const std::string& catalogName);
    static WoweeUnitMovement makeFlight(const std::string& catalogName);
    static WoweeUnitMovement makeBuffs(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
