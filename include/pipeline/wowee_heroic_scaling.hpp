#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Heroic Loot Scaling catalog (.whrd) -
// novel replacement for the implicit Heroic-mode loot
// rules vanilla WoW encoded in the dungeon/raid script
// system: a Normal-mode boss drops items from one loot
// table, the Heroic-mode version drops the same items
// at +N item levels with M× drop chance, plus an
// optional Heroic-only currency token (Emblem of Frost,
// etc.). Each WHRD entry binds one (mapId, difficultyId)
// combination to its scaling rules.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the WMS map catalog.
//   WCDF: difficultyId references the WCDF creature/
//         instance difficulty variant catalog.
//   WIT:  heroicTokenItemId references the WIT item
//         catalog (the per-Heroic currency token, 0 if
//         no token reward).
//   WLOT: rules are layered over the base WLOT loot
//         table the encounter normally drops.
//
// Binary layout (little-endian):
//   magic[4]            = "WHRD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     scalingId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32) / difficultyId (uint32)
//     itemLevelDelta (int16)        - bonus ilvl over
//                                      Normal (often
//                                      +13 for 5-man
//                                      Heroic, +13 to +26
//                                      for raid Heroic)
//     bonusQualityChance (uint16)   - 0..10000 (basis
//                                      points, 100 = 1%);
//                                      probability of a
//                                      bonus +1-tier
//                                      quality drop
//     dropChanceMultiplier (float)  - 1.0 = same drop
//                                      rate, 1.5 = +50%,
//                                      etc.
//     heroicTokenItemId (uint32)    - 0 if no token
//     bonusEmblemCount (uint8)      - extra emblem-token
//                                      rewards on top of
//                                      base 1× per boss
//     pad0 / pad1 / pad2 (uint8)
//     iconColorRGBA (uint32)
struct WoweeHeroicScaling {
    struct Entry {
        uint32_t scalingId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t difficultyId = 0;
        int16_t itemLevelDelta = 13;
        uint16_t bonusQualityChance = 0;
        float dropChanceMultiplier = 1.0f;
        uint32_t heroicTokenItemId = 0;
        uint8_t bonusEmblemCount = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t scalingId) const;

    // Returns the scaling rules for a given (map,
    // difficulty) combo, or nullptr if no Heroic
    // scaling is defined (defaults to no scaling).
    // Used by the loot-roll engine when an encounter
    // dies on Heroic-difficulty content.
    [[nodiscard]] const Entry* findForInstance(uint32_t mapId,
                                  uint32_t difficultyId) const;
};

class WoweeHeroicScalingLoader {
public:
    static bool save(const WoweeHeroicScaling& cat,
                     const std::string& basePath);
    static WoweeHeroicScaling load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-hrd* variants.
    //
    //   makeWotLK5manHeroic - 5 WotLK 5-man Heroic
    //                          scalings (Utgarde Keep /
    //                          Nexus / Azjol-Nerub /
    //                          Ahn'kahet / Drak'Tharon).
    //                          +13 ilvl, 1.0× drop
    //                          chance, 2× Emblem of
    //                          Heroism.
    //   makeRaid25Heroic    - 4 25H raid scalings (Naxx
    //                          / EoE / Ulduar / ICC).
    //                          +26 ilvl, 1.5× drop chance
    //                          on rare items, 1× Emblem
    //                          of Frost per boss.
    //   makeChallengeMode   - 3 challenge-mode tier
    //                          scalings (Bronze / Silver
    //                          / Gold). Anachronistic
    //                          for WotLK but useful
    //                          template for custom-server
    //                          content.
    static WoweeHeroicScaling makeWotLK5manHeroic(const std::string& catalogName);
    static WoweeHeroicScaling makeRaid25Heroic(const std::string& catalogName);
    static WoweeHeroicScaling makeChallengeMode(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
