#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Cinematic catalog (.wcms) - novel replacement
// for Blizzard's Movie.dbc + CinematicCamera.dbc +
// CinematicSequences.dbc + the AzerothCore-style
// cinematic_camera SQL tables. The 41st open format added
// to the editor.
//
// Defines cinematics: pre-rendered videos, in-engine camera
// flythroughs, text crawls, and still images. Each
// cinematic has a media path (video file or M2 model
// reference for in-engine camera), duration, a "skippable"
// flag, and a trigger that fires the cinematic from a
// gameplay event (quest accepted, class first-login,
// dungeon cleared, achievement earned).
//
// Cross-references with previously-added formats - the
// triggerTargetId field is polymorphic by triggerKind:
//   triggerKind=QuestStart/End  → WQT.entry.questId
//   triggerKind=ZoneEntry       → WMS.area.areaId
//   triggerKind=ClassStart      → WCHC.class.classId
//   triggerKind=DungeonClear    → WMS.map.mapId
//   triggerKind=AchievementGained → WACH.entry.achievementId
//   WCMS.soundtrackId            → WSND.entry.soundId
//
// Binary layout (little-endian):
//   magic[4]            = "WCMS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     cinematicId (uint32)
//     nameLen + name
//     descLen + description
//     mediaLen + mediaPath
//     kind (uint8) / triggerKind (uint8) / skippable (uint8) / pad[1]
//     durationSeconds (uint32)
//     triggerTargetId (uint32)
//     soundtrackId (uint32)
struct WoweeCinematic {
    enum Kind : uint8_t {
        PreRenderedVideo = 0,    // .ogv / .bik file
        CameraFlythrough = 1,    // in-engine camera spline
        TextCrawl        = 2,    // Star Wars-style scrolling text
        StillImage       = 3,    // single image with audio
        Slideshow        = 4,    // multi-image with timing
    };

    enum TriggerKind : uint8_t {
        Manual            = 0,    // played by script call
        QuestStart        = 1,
        QuestEnd          = 2,
        ClassStart        = 3,    // shown on first login of new char
        ZoneEntry         = 4,
        DungeonClear      = 5,    // boss kill in instance
        Login             = 6,    // every login
        AchievementGained = 7,
        LevelUp           = 8,    // milestone levels (10, 20, 60, ...)
    };

    struct Entry {
        uint32_t cinematicId = 0;
        std::string name;
        std::string description;
        std::string mediaPath;
        uint8_t kind = PreRenderedVideo;
        uint8_t triggerKind = Manual;
        uint8_t skippable = 1;          // 1 = ESC dismisses
        uint32_t durationSeconds = 0;   // 0 = unknown / video-determined
        uint32_t triggerTargetId = 0;
        uint32_t soundtrackId = 0;       // WSND cross-ref, 0 = none
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t cinematicId) const;

    static const char* kindName(uint8_t k);
    static const char* triggerKindName(uint8_t t);
};

class WoweeCinematicLoader {
public:
    static bool save(const WoweeCinematic& cat,
                     const std::string& basePath);
    static WoweeCinematic load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cinematics* variants.
    //
    //   makeStarter - 3 cinematics: 1 pre-rendered intro,
    //                  1 in-engine quest cutscene, 1 still
    //                  image with audio (login splash).
    //   makeIntros  - 4 class-intro cinematics for Warrior /
    //                  Mage / Hunter / Rogue, each shown on
    //                  first character login.
    //   makeQuestCinematics - 3 quest-triggered cinematics
    //                          referencing WQT questIds 1 /
    //                          100 / 102 from the demo
    //                          content stack.
    static WoweeCinematic makeStarter(const std::string& catalogName);
    static WoweeCinematic makeIntros(const std::string& catalogName);
    static WoweeCinematic makeQuestCinematics(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
