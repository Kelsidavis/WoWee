#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Anniversary & Recurring Event catalog
// (.wanv) - novel replacement for the implicit
// recurring-event scheduler vanilla WoW encoded across
// the GameEvent SQL table + the per-holiday script
// hooks. Each entry binds one calendar-driven recurring
// event (holiday like Hallow's End, anniversary, double-
// XP weekend, brewfest) to its scheduling rule (yearly
// on a fixed date, monthly on a fixed day, weekly on a
// weekday) and its payload (a spell buff applied to all
// online players, a gift item granted on first login
// during the event window).
//
// Cross-references with previously-added formats:
//   WSPL: payloadSpellId references the WSPL spell
//         catalog (buff applied to online players for
//         the event duration).
//   WIT:  payloadItemId references the WIT item
//         catalog (gift item granted on first event-
//         window login).
//
// Binary layout (little-endian):
//   magic[4]            = "WANV"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     eventId (uint32)
//     nameLen + name
//     descLen + description
//     eventKind (uint8)          - Holiday / Anniversary
//                                   / DoubleXP /
//                                   DoubleHonor /
//                                   PetBattleWeekend /
//                                   BattlegroundBonus /
//                                   SeasonalQuest /
//                                   Misc
//     recurrenceKind (uint8)     - Yearly / Monthly /
//                                   Weekly / OneOff
//     startMonth (uint8)         - 1..12 for Yearly /
//                                   Monthly; ignored for
//                                   Weekly (use startDay
//                                   = weekday 0..6)
//     startDay (uint8)           - 1..31 for Yearly /
//                                   Monthly; 0..6 for
//                                   Weekly (Sun..Sat)
//     durationDays (uint16)      - event window length
//     pad0 (uint8) / pad1 (uint8)
//     payloadSpellId (uint32)    - 0 if no buff
//     payloadItemId (uint32)     - 0 if no gift item
//     iconColorRGBA (uint32)
struct WoweeAnniversaryEvents {
    enum EventKind : uint8_t {
        Holiday            = 0,    // seasonal real-world
                                    // holiday tie-in
        Anniversary        = 1,    // game-launch
                                    // anniversary
        DoubleXP           = 2,    // experience boost
        DoubleHonor        = 3,    // PvP boost
        PetBattleWeekend   = 4,    // pet-battle bonus
        BattlegroundBonus  = 5,    // BG honor / token
                                    // boost
        SeasonalQuest      = 6,    // limited-time quest
                                    // chain
        Misc               = 255,
    };

    enum RecurrenceKind : uint8_t {
        Yearly  = 0,    // same date every year
                         // (e.g. Hallow's End Oct 18)
        Monthly = 1,    // same day every month
                         // (e.g. monthly tribute day)
        Weekly  = 2,    // same weekday every week
                         // (e.g. Tuesday maintenance)
        OneOff  = 3,    // single occurrence at the
                         // specified date - Anniversary
                         // events stay this way until
                         // the next year manually
                         // re-schedules
    };

    struct Entry {
        uint32_t eventId = 0;
        std::string name;
        std::string description;
        uint8_t eventKind = Holiday;
        uint8_t recurrenceKind = Yearly;
        uint8_t startMonth = 1;
        uint8_t startDay = 1;
        uint16_t durationDays = 7;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t payloadSpellId = 0;
        uint32_t payloadItemId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t eventId) const;

    // Returns all events of one kind. Used by the event
    // scheduler to dispatch per-kind handlers (Holiday
    // events spawn cosmetic NPCs, DoubleXP events
    // multiply XP rates, BattlegroundBonus events boost
    // honor accrual).
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t eventKind) const;
};

class WoweeAnniversaryEventsLoader {
public:
    static bool save(const WoweeAnniversaryEvents& cat,
                     const std::string& basePath);
    static WoweeAnniversaryEvents load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-anv* variants.
    //
    //   makeStandardHolidays - 5 yearly holidays
    //                           (Hallow's End / Winter
    //                           Veil / Lunar Festival /
    //                           Children's Week /
    //                           Brewfest).
    //   makeBonusEvents      - 4 weekly bonus events
    //                           (Double XP Weekend /
    //                           Double Honor / Pet
    //                           Battle Weekend / BG
    //                           Bonus).
    //   makeAnniversary      - 3 game-launch
    //                           anniversaries (WoW Nov
    //                           23 / TBC Jan 16 / WotLK
    //                           Nov 13).
    static WoweeAnniversaryEvents makeStandardHolidays(const std::string& catalogName);
    static WoweeAnniversaryEvents makeBonusEvents(const std::string& catalogName);
    static WoweeAnniversaryEvents makeAnniversary(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
