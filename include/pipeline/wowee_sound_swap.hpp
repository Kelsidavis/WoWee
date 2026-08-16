#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Sound Swap Rules catalog (.wswp) -
// novel format covering a need vanilla WoW lacked
// entirely: priority-based sound substitution
// (Blizzard had no formal mechanism for swapping a
// stock SoundEntry for a custom replacement
// conditionally on zone/class/race; the closest
// equivalents were patch-level SoundEntries.dbc
// edits with no condition support). Each WSWP
// entry binds one (originalSoundId, condition)
// trigger to a replacementSoundId, a priority
// index for tie-breaking (higher wins), and an
// optional gain adjustment in 0.1 dB units.
//
// Cross-references with previously-added formats:
//   WSND: originalSoundId AND replacementSoundId
//         both reference the WSND sound entry
//         catalog.
//   WMS:  conditionKind=ZoneOnly conditionValue
//         is a WMS mapId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSWP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     ruleId (uint32)
//     nameLen + name
//     originalSoundId (uint32)     - WSND ref
//     replacementSoundId (uint32)  - WSND ref
//     conditionKind (uint8)        - 0=Always /
//                                     1=ZoneOnly /
//                                     2=ClassOnly /
//                                     3=RaceOnly /
//                                     4=GenderOnly
//     priorityIndex (uint8)        - higher wins
//                                     when multiple
//                                     rules match
//                                     (1..255; 0 =
//                                     never picked)
//     gainAdjustDb_x10 (int16)     - gain in 0.1 dB
//                                     units, range
//                                     [-300..+300] =
//                                     -30..+30 dB
//     conditionValue (uint32)      - interpretation
//                                     depends on
//                                     conditionKind
//                                     (mapId / classId
//                                     / raceId /
//                                     genderId; 0 for
//                                     Always)
struct WoweeSoundSwap {
    enum ConditionKind : uint8_t {
        Always       = 0,
        ZoneOnly     = 1,
        ClassOnly    = 2,
        RaceOnly     = 3,
        GenderOnly   = 4,
    };

    struct Entry {
        uint32_t ruleId = 0;
        std::string name;
        uint32_t originalSoundId = 0;
        uint32_t replacementSoundId = 0;
        uint8_t conditionKind = Always;
        uint8_t priorityIndex = 0;
        int16_t gainAdjustDb_x10 = 0;
        uint32_t conditionValue = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t ruleId) const;

    // Returns all rules that target a given
    // originalSoundId - used by the audio dispatch
    // hot path to walk candidate replacements for an
    // about-to-play sound.
    [[nodiscard]] std::vector<const Entry*> findByOriginalSound(uint32_t soundId) const;
};

class WoweeSoundSwapLoader {
public:
    static bool save(const WoweeSoundSwap& cat,
                     const std::string& basePath);
    static WoweeSoundSwap load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-swp* variants.
    //
    //   makeBossOverrides   - 3 raid-boss sound
    //                          swaps (Onyxia roar /
    //                          Ragnaros emerge /
    //                          Nefarian shout) all
    //                          ZoneOnly to their
    //                          respective raid
    //                          mapIds.
    //   makeRaceVoices      - 3 race-specific voice
    //                          replacements
    //                          (BloodElf priest cast
    //                          / Tauren shaman cast /
    //                          Undead warlock cast)
    //                          conditionKind=
    //                          RaceOnly.
    //   makeGlobalUI        - 3 always-on UI sound
    //                          replacements
    //                          (level-up / quest-
    //                          complete / mount-up)
    //                          with gainAdjust=+30
    //                          (+3 dB to make custom
    //                          sounds slightly
    //                          louder than default).
    static WoweeSoundSwap makeBossOverrides(const std::string& catalogName);
    static WoweeSoundSwap makeRaceVoices(const std::string& catalogName);
    static WoweeSoundSwap makeGlobalUI(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
