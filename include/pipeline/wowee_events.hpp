#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Seasonal Event catalog (.wsea) - novel
// replacement for Blizzard's GameEvents.dbc + the
// AzerothCore-style game_event / game_event_creature /
// game_event_gameobject SQL tables. The 31st open format
// added to the editor.
//
// Calendar-based content: holidays (Hallow's End, Winter's
// Veil), recurring promotional events (Children's Week,
// Lunar Festival, Brewfest), one-time anniversaries, and
// XP-bonus weekends. Each event has a start date, duration,
// optional recurrence, faction restriction, optional XP
// bonus, and a reward currency cross-reference into WTKN.
//
// Cross-references with previously-added formats:
//   WSEA.entry.tokenIdReward → WTKN.entry.tokenId
//                              (the seasonal currency the
//                               event hands out - Tricky
//                               Treats during Hallow's End,
//                               Brewfest Tokens during
//                               Brewfest, etc.)
//
// Binary layout (little-endian):
//   magic[4]            = "WSEA"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     eventId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     announceLen + announceMessage
//     startDate (uint64)               -- Unix timestamp seconds
//     duration_seconds (uint32)
//     recurrenceDays (uint16)
//     holidayKind (uint8) / factionGroup (uint8)
//     bonusXpPercent (uint8) / pad[3]
//     tokenIdReward (uint32)
struct WoweeEvent {
    enum HolidayKind : uint8_t {
        Combat       = 0,
        Collection   = 1,
        Racial       = 2,
        Anniversary  = 3,
        Fishing      = 4,
        Cosmetic     = 5,
        WorldEvent   = 6,    // server-wide invasion / boss event
    };

    enum FactionGroup : uint8_t {
        FactionBoth     = 0,
        FactionAlliance = 1,
        FactionHorde    = 2,
    };

    struct Entry {
        uint32_t eventId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        std::string announceMessage;
        uint64_t startDate = 0;          // Unix epoch seconds
        uint32_t duration_seconds = 0;
        uint16_t recurrenceDays = 0;     // 0 = one-shot
        uint8_t holidayKind = Combat;
        uint8_t factionGroup = FactionBoth;
        uint8_t bonusXpPercent = 0;      // 0 = no bonus
        uint32_t tokenIdReward = 0;      // WTKN cross-ref, 0 = none
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t eventId) const;

    static const char* holidayKindName(uint8_t k);
    static const char* factionGroupName(uint8_t f);
};

class WoweeEventLoader {
public:
    static bool save(const WoweeEvent& cat,
                     const std::string& basePath);
    static WoweeEvent load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-events* variants.
    //
    //   makeStarter - 3 events covering Combat / Collection /
    //                  Anniversary holidayKind categories.
    //   makeYearly  - 4 yearly recurring holidays with WTKN
    //                  reward cross-refs (Hallow's End ->
    //                  Tricky Treats, Brewfest -> Brewfest
    //                  Tokens, Lunar Festival -> Coin of
    //                  Ancestry, Winter's Veil -> Stranger's
    //                  Gift).
    //   makeBonusWeekends - 3 short XP-bonus weekend events
    //                        (50% / 100% / 200% bonus tiers).
    static WoweeEvent makeStarter(const std::string& catalogName);
    static WoweeEvent makeYearly(const std::string& catalogName);
    static WoweeEvent makeBonusWeekends(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
