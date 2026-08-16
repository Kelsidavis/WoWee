#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Mount catalog (.wmou) - novel replacement for
// Blizzard's Mount.dbc + MountCapability.dbc + MountType.dbc
// + the mount-related subsets of Spell.dbc / Item.dbc. The
// 32nd open format added to the editor.
//
// Defines all summonable steeds: ground mounts, flying
// mounts, swimming mounts, racial mounts (Tauren Plainsrunner
// for druids), and class mounts (Warlock dreadsteed,
// Paladin charger). Each mount has a summon spell, optional
// teach item, riding skill prerequisite, speed bonus, and
// faction / race availability mask.
//
// Cross-references with previously-added formats:
//   WMOU.entry.summonSpellId      → WSPL.entry.spellId
//   WMOU.entry.itemIdToLearn      → WIT.entry.itemId
//   WMOU.entry.requiredSkillId    → WSKL.entry.skillId (riding)
//   WCHC.race.mountSpellId        ≈ WMOU.entry.summonSpellId
//                                    (racial mount per race)
//
// Binary layout (little-endian):
//   magic[4]            = "WMOU"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     mountId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     displayId (uint32)
//     summonSpellId (uint32)
//     itemIdToLearn (uint32)
//     requiredSkillId (uint32)
//     requiredSkillRank (uint16)
//     speedPercent (uint16)
//     mountKind (uint8) / factionId (uint8) / categoryId (uint8) / pad[1]
//     raceMask (uint32)
struct WoweeMount {
    enum Kind : uint8_t {
        Ground   = 0,
        Flying   = 1,
        Swimming = 2,
        Hybrid   = 3,    // ground + flying (basic flying mount)
        Aquatic  = 4,    // ground + swim (sea horse)
    };

    enum Faction : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
    };

    enum Category : uint8_t {
        Common      = 0,
        Epic        = 1,
        Racial      = 2,
        Event       = 3,
        Achievement = 4,
        Pvp         = 5,
        Quest       = 6,
        ClassMount  = 7,    // warlock dreadsteed / paladin charger
    };

    struct Entry {
        uint32_t mountId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint32_t displayId = 0;
        uint32_t summonSpellId = 0;
        uint32_t itemIdToLearn = 0;
        uint32_t requiredSkillId = 0;
        uint16_t requiredSkillRank = 0;
        uint16_t speedPercent = 60;        // 60 = +60% (apprentice ground)
        uint8_t mountKind = Ground;
        uint8_t factionId = Both;
        uint8_t categoryId = Common;
        uint32_t raceMask = 0;             // 0 = available to all races
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t mountId) const;

    static const char* kindName(uint8_t k);
    static const char* factionName(uint8_t f);
    static const char* categoryName(uint8_t c);
};

class WoweeMountLoader {
public:
    static bool save(const WoweeMount& cat,
                     const std::string& basePath);
    static WoweeMount load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mounts* variants.
    //
    //   makeStarter - 3 mounts: 1 ground / 1 flying / 1
    //                  swimming with appropriate speed +
    //                  riding skill requirements.
    //   makeRacial  - 6 racial mounts (one per Alliance + 2
    //                  Horde races) all with raceMask gating.
    //   makeFlying  - 4 flying mounts spanning common / epic /
    //                  achievement / pvp tiers (60% / 100% /
    //                  280% / 310% speed).
    static WoweeMount makeStarter(const std::string& catalogName);
    static WoweeMount makeRacial(const std::string& catalogName);
    static WoweeMount makeFlying(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
