#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Player Movement-to-Animation Map (.wphm)
// - novel replacement for the implicit
// movementState->animation mapping vanilla WoW baked
// into per-race M2 model files. Each WPHM entry binds
// one (raceId, genderId, movementState) tuple to a
// base animation sequence ID + an optional variant
// (e.g. wounded run, drunk walk) and a transition
// blend duration.
//
// Cross-references with previously-added formats:
//   WCDB: raceId references the playable-race
//         catalog (currently 1..10 in vanilla:
//         Human=1, Orc=2 ... Troll=8, Goblin=9,
//         BloodElf=10).
//   None to M2 binary directly - animation index
//   numbers come from the standard WoW M2 animation
//   table (Stand=0, Walk=4, Run=5, Death=1...).
//
// Binary layout (little-endian):
//   magic[4]            = "WPHM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     mapId (uint32)         - surrogate primary key
//                               for cross-format
//                               --catalog-find lookups
//     raceId (uint8)         - 1..10 vanilla race
//     genderId (uint8)       - 0=male, 1=female
//     movementState (uint8)  - 0=Idle / 1=Walk / 2=Run
//                               / 3=Swim / 4=Fly /
//                               5=Sit / 6=Mount /
//                               7=Death
//     pad0 (uint8)
//     baseAnimId (uint32)    - M2 anim sequence id
//     variantAnimId (uint32) - alternate sequence
//                               (0 = no variant)
//     transitionMs (uint16)  - blend duration to enter
//     pad1 (uint16)
struct WoweePlayerMovementAnim {
    enum MovementState : uint8_t {
        StateIdle  = 0,
        StateWalk  = 1,
        StateRun   = 2,
        StateSwim  = 3,
        StateFly   = 4,
        StateSit   = 5,
        StateMount = 6,
        StateDeath = 7,
    };

    struct Entry {
        uint32_t mapId = 0;
        uint8_t raceId = 0;
        uint8_t genderId = 0;
        uint8_t movementState = StateIdle;
        uint8_t pad0 = 0;
        uint32_t baseAnimId = 0;
        uint32_t variantAnimId = 0;
        uint16_t transitionMs = 0;
        uint16_t pad1 = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t mapId) const;

    // Returns the binding for a specific (race, gender,
    // state) - the canonical lookup the renderer uses
    // each frame to decide which animation sequence to
    // play.
    [[nodiscard]] const Entry* find(uint8_t raceId, uint8_t genderId,
                       uint8_t state) const;

    // Returns all bindings for a race+gender combo.
    // Used by character-preview UIs to show the full
    // state machine for one model.
    [[nodiscard]] std::vector<const Entry*> findByRaceGender(uint8_t raceId,
                                                 uint8_t genderId) const;
};

class WoweePlayerMovementAnimLoader {
public:
    static bool save(const WoweePlayerMovementAnim& cat,
                     const std::string& basePath);
    static WoweePlayerMovementAnim load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-phm* variants.
    //
    //   makeHumanMovement  - full 8-state machine for
    //                          Human male + female =
    //                          16 entries. Walk-while-
    //                          drunk variant for State
    //                          Walk only.
    //   makeOrcMovement    - same shape for Orc male
    //                          + female. Orc Run uses
    //                          a more aggressive
    //                          variant for war-stance
    //                          flavor.
    //   makeUndeadMovement - Undead male + female,
    //                          with canonical
    //                          "shambling-when-wounded"
    //                          variantAnimId on Run
    //                          state (low-health
    //                          renderer override).
    static WoweePlayerMovementAnim makeHumanMovement(const std::string& catalogName);
    static WoweePlayerMovementAnim makeOrcMovement(const std::string& catalogName);
    static WoweePlayerMovementAnim makeUndeadMovement(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
