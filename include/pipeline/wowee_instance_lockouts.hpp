#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Instance Lockout Schedule catalog (.whld)
// - novel replacement for the engine-side instance reset
// timer logic plus the per-map InstanceTemplate.dbc reset
// fields. Defines how often each (map × difficulty)
// combination resets its lockout, how many boss kills
// each character can claim per lockout window, and the
// number of bonus rolls available (Cataclysm+ feature
// stubbed for forward compatibility).
//
// One entry per (map × difficulty × group size) - Icecrown
// Citadel 10-Normal weekly, ICC 25-Normal weekly, ICC
// 10-Heroic weekly, and ICC 25-Heroic weekly are four
// separate entries with the same mapId but different
// difficultyId and resetIntervalMs.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the instance map.
//   WCDF: difficultyId references the difficulty routing
//         entry that maps base creature -> variants.
//   WBOS: encounters in this lockout are the WBOS entries
//         whose (mapId, difficultyId) pair matches.
//
// Binary layout (little-endian):
//   magic[4]            = "WHLD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     lockoutId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32)
//     difficultyId (uint32)
//     resetIntervalMs (uint32)
//     maxBossKillsPerLockout (uint8) / bonusRolls (uint8)
//     raidLockoutKind (uint8) / raidGroupSize (uint8)
//     iconColorRGBA (uint32)
struct WoweeInstanceLockout {
    enum LockoutKind : uint8_t {
        Daily       = 0,    // 24h reset (5-man heroics, daily quests)
        Weekly      = 1,    // 7d reset (raid lockouts)
        SemiWeekly  = 2,    // 3.5d reset (Cata+ split lockouts)
        Custom      = 3,    // arbitrary intervalMs
    };

    static constexpr uint32_t kDailyMs       = 86400000u;   // 24 * 3600 * 1000
    static constexpr uint32_t kWeeklyMs      = 604800000u;  // 7 * kDailyMs
    static constexpr uint32_t kSemiWeeklyMs  = 302400000u;  // 3.5d

    struct Entry {
        uint32_t lockoutId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t difficultyId = 0;
        uint32_t resetIntervalMs = 0;
        uint8_t maxBossKillsPerLockout = 0;
        uint8_t bonusRolls = 0;
        uint8_t raidLockoutKind = Weekly;
        uint8_t raidGroupSize = 10;       // 5 / 10 / 25 / 40
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t lockoutId) const;

    // Returns all lockouts bound to a given map (typically
    // all 4 difficulty variants of one raid). Used by the
    // raid-finder UI to populate the difficulty picker.
    [[nodiscard]] std::vector<const Entry*> findByMap(uint32_t mapId) const;

    // Returns the next reset wall-clock millis after the
    // given current time, assuming the standard Tuesday
    // reset epoch. Engines override with their server's
    // chosen reset time.
    [[nodiscard]] uint64_t nextResetMs(uint32_t lockoutId,
                         uint64_t currentMs) const;

    static const char* lockoutKindName(uint8_t k);
};

class WoweeInstanceLockoutLoader {
public:
    static bool save(const WoweeInstanceLockout& cat,
                     const std::string& basePath);
    static WoweeInstanceLockout load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-hld* variants.
    //
    //   makeRaidWeekly  - 4 raid weekly lockouts (ICC 10N /
    //                      ICC 25N / ICC 10H / ICC 25H) with
    //                      12-boss kill caps and 7-day
    //                      reset.
    //   makeDungeonDaily - 4 5-man daily heroic lockouts
    //                      (HoR / FoS / PoS / TotC) with
    //                      24h reset and 1-boss caps each.
    //   makeWorldEvent  - 3 special event lockouts
    //                      (Brewfest daily / Hallow's End
    //                      pumpkin daily / Wintergrasp
    //                      battle 2.5h reset).
    static WoweeInstanceLockout makeRaidWeekly(const std::string& catalogName);
    static WoweeInstanceLockout makeDungeonDaily(const std::string& catalogName);
    static WoweeInstanceLockout makeWorldEvent(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
