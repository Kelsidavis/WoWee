#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Holiday catalog (.whol) - novel replacement for
// Blizzard's Holidays.dbc + HolidayDescriptions.dbc +
// HolidayNames.dbc plus the AzerothCore-style game_event SQL
// tables. The 44th open format added to the editor.
//
// Defines time-gated world events: seasonal holidays (Hallow's
// End, Brewfest, Winter Veil), weekly call-to-arms bonuses
// (BG / dungeon weekend buffs), one-shot special events, and
// recurring world PvP encouragement windows. Each holiday has
// a recurrence rule, a calendar window, optional cross-refs
// to a feature creature (e.g. Headless Horseman), an intro
// quest, and a token / item reward issued during the window.
//
// Cross-references with previously-added formats:
//   WHOL.entry.holidayQuestId  → WQT.entry.questId
//   WHOL.entry.bossCreatureId  → WCRT.creatureId
//   WHOL.entry.itemRewardId    → WIT.itemId
//   WHOL.entry.areaIdGate      → WMS.area.areaId
//   WHOL.entry.mapIdGate       → WMS.map.mapId
//
// Binary layout (little-endian):
//   magic[4]            = "WHOL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     holidayId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     holidayKind (uint8) / recurrence (uint8) /
//       startMonth (uint8) / startDay (uint8)
//     durationHours (uint16) / pad[2]
//     holidayQuestId (uint32)
//     bossCreatureId (uint32)
//     itemRewardId (uint32)
//     areaIdGate (uint32)
//     mapIdGate (uint32)
struct WoweeHoliday {
    enum HolidayKind : uint8_t {
        Seasonal      = 0,    // Hallow's End, Brewfest, ...
        Weekly        = 1,    // call-to-arms BG bonuses
        Daily         = 2,    // daily quest reset window
        WorldPvp      = 3,    // Wintergrasp / Tol Barad windows
        OneShot       = 4,    // anniversary, beta launch
        Special       = 5,    // Children's Week, anniversary
    };

    enum Recurrence : uint8_t {
        Annual       = 0,    // once per year
        Monthly      = 1,    // once per month
        WeeklyRecur  = 2,    // every week
        OneTime      = 3,    // single occurrence
    };

    struct Entry {
        uint32_t holidayId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t holidayKind = Seasonal;
        uint8_t recurrence = Annual;
        uint8_t startMonth = 1;        // 1-12 (0 = unused)
        uint8_t startDay = 1;          // 1-31
        uint16_t durationHours = 168;  // default 1 week
        uint32_t holidayQuestId = 0;   // WQT cross-ref
        uint32_t bossCreatureId = 0;   // WCRT cross-ref
        uint32_t itemRewardId = 0;     // WIT cross-ref
        uint32_t areaIdGate = 0;       // WMS cross-ref
        uint32_t mapIdGate = 0;        // WMS cross-ref
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t holidayId) const;

    static const char* holidayKindName(uint8_t k);
    static const char* recurrenceName(uint8_t r);
};

class WoweeHolidayLoader {
public:
    static bool save(const WoweeHoliday& cat,
                     const std::string& basePath);
    static WoweeHoliday load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-holidays* variants.
    //
    //   makeStarter  - 3 seasonal holidays (Hallow's End /
    //                   Brewfest / Winter Veil) with boss /
    //                   reward / quest cross-refs.
    //   makeWeekly   - 3 weekly call-to-arms windows
    //                   (Warsong Gulch / Arathi Basin / Eye
    //                   of the Storm BG bonuses).
    //   makeSpecial  - 3 special / world-PvP events
    //                   (Wintergrasp / Lunar Festival /
    //                   Children's Week).
    static WoweeHoliday makeStarter(const std::string& catalogName);
    static WoweeHoliday makeWeekly(const std::string& catalogName);
    static WoweeHoliday makeSpecial(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
