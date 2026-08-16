#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Group Composition catalog (.wgrp) - novel
// replacement for the hardcoded LFG / Dungeon Finder
// group-composition rules. Defines per-instance role
// quotas: how many tanks, healers, and damage dealers a
// group needs to queue for a given (map, difficulty)
// combination.
//
// 5-man dungeons typically want 1T / 1H / 3D. 10-man
// raids want 2T / 3H / 5D. 25-man raids want 2T / 6H /
// 17D. Server-custom variants like 5-man "all DPS speed
// runs" or "healer-heavy" 25-Heroic fights override the
// stock distribution.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the WMS map entry.
//   WCDF: difficultyId references the WCDF difficulty
//         routing entry.
//   WBOS: encounters in this composition's instance are
//         the WBOS entries whose mapId+difficultyId match.
//   WHLD: lockout schedule for this composition is the
//         WHLD entry with matching mapId+difficultyId.
//
// Binary layout (little-endian):
//   magic[4]            = "WGRP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     compId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32)
//     difficultyId (uint32)
//     requiredTanks (uint8) / requiredHealers (uint8)
//     requiredDamageDealers (uint8) / minPartySize (uint8)
//     maxPartySize (uint8) / requireSpec (uint8) / pad[2]
//     iconColorRGBA (uint32)
struct WoweeGroupComposition {
    struct Entry {
        uint32_t compId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t difficultyId = 0;
        uint8_t requiredTanks = 1;
        uint8_t requiredHealers = 1;
        uint8_t requiredDamageDealers = 3;
        uint8_t minPartySize = 5;
        uint8_t maxPartySize = 5;
        uint8_t requireSpec = 1;     // 0/1 bool
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t compId) const;

    // Returns all compositions registered for one instance
    // (typically the per-difficulty variants). Used by the
    // LFG UI to populate the difficulty/comp picker.
    [[nodiscard]] std::vector<const Entry*> findByMap(uint32_t mapId) const;

    // Returns true if a queueing party of (tanks, healers,
    // dps) satisfies the composition's role requirements.
    // Used by the matchmaker to decide if a group is ready
    // to launch.
    [[nodiscard]] bool partyMeetsComp(uint32_t compId,
                         uint8_t haveTanks,
                         uint8_t haveHealers,
                         uint8_t haveDps) const;
};

class WoweeGroupCompositionLoader {
public:
    static bool save(const WoweeGroupComposition& cat,
                     const std::string& basePath);
    static WoweeGroupComposition load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-grp* variants.
    //
    //   makeFiveMan - 3 5-man composition variants
    //                  (Classic 1T/1H/3D, Heavy-Heal
    //                  1T/2H/2D for trash, Roleless 5D
    //                  speed runs with requireSpec=0).
    //   makeRaid10  - 3 10-man comps (Standard 2T/3H/5D,
    //                  Weighted 2T/4H/4D for healing-
    //                  heavy fights, MeleeStack 1T/2H/7D
    //                  for melee-cleave fights without
    //                  a tank swap mechanic).
    //   makeRaid25  - 3 25-man comps (Standard 2T/6H/17D,
    //                  HealingHeavy 1T/8H/16D for ICC,
    //                  ZergDPS 0T/4H/21D for tank-immune
    //                  bosses).
    static WoweeGroupComposition makeFiveMan(const std::string& catalogName);
    static WoweeGroupComposition makeRaid10(const std::string& catalogName);
    static WoweeGroupComposition makeRaid25(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
