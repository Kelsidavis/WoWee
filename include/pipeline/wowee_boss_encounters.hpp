#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Boss Encounter catalog (.wbos) - novel
// replacement for AzerothCore's instance_encounter SQL
// table plus the per-boss script bindings. Defines raid
// boss encounter metadata: which creature is the boss,
// which map and difficulty variant it lives in, how many
// phases the encounter has, the soft-enrage timer and
// berserk spell, recommended group size, and item level.
//
// One entry per (boss × difficulty) combination. Lord
// Marrowgar in 10-Normal ICC is one entry; Lord Marrowgar
// in 25-Heroic ICC is a separate entry with a higher
// recommendedItemLevel and a different difficultyId.
//
// Cross-references with previously-added formats:
//   WCRT: bossCreatureId references the WCRT creature
//         template entry for the boss.
//   WMS:  mapId references the WMS map entry (instance).
//   WCDF: difficultyId references the WCDF route that
//         maps base creature -> 10/25/H10/H25 variants.
//   WSPL: berserkSpellId references the WSPL spell that
//         fires when the soft-enrage timer expires.
//   WACR: achievement criteria with KillCreature targetId
//         pointing at this boss reference back to it.
//
// Binary layout (little-endian):
//   magic[4]            = "WBOS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     encounterId (uint32)
//     nameLen + name
//     descLen + description
//     bossCreatureId (uint32)
//     mapId (uint32)
//     difficultyId (uint32)
//     berserkSpellId (uint32)
//     enrageTimerMs (uint32)
//     phaseCount (uint8) / requiredPartySize (uint8) / pad[2]
//     recommendedItemLevel (uint16) / pad[2]
//     iconColorRGBA (uint32)
struct WoweeBossEncounter {
    struct Entry {
        uint32_t encounterId = 0;
        std::string name;
        std::string description;
        uint32_t bossCreatureId = 0;
        uint32_t mapId = 0;
        uint32_t difficultyId = 0;
        uint32_t berserkSpellId = 0;
        uint32_t enrageTimerMs = 0;       // 0 = no soft enrage
        uint8_t phaseCount = 1;
        uint8_t requiredPartySize = 10;   // 5 / 10 / 25 / 40
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint16_t recommendedItemLevel = 0;
        uint16_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t encounterId) const;

    // Returns all encounters bound to a given map id
    // (typically all bosses in one raid instance), in the
    // order they appear in the catalog. Used by the Encounter
    // Journal UI and instance lockout logic.
    [[nodiscard]] std::vector<const Entry*> findByMap(uint32_t mapId) const;

    // Returns all encounters bound to a given boss creature
    // (typically the per-difficulty variants of one boss).
    [[nodiscard]] std::vector<const Entry*> findByBossCreature(
        uint32_t bossCreatureId) const;
};

class WoweeBossEncounterLoader {
public:
    static bool save(const WoweeBossEncounter& cat,
                     const std::string& basePath);
    static WoweeBossEncounter load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bos* variants.
    //
    //   makeFiveMan  - 3 5-man dungeon bosses (trash boss,
    //                   mid boss, final boss) at recommended
    //                   ilvl 200 with no soft-enrage.
    //   makeRaid10   - 4 ICC-style 10-man raid bosses
    //                   (Marrowgar / Deathwhisper / Saurfang
    //                   / Lich King) with multi-phase
    //                   structure and soft-enrage timers.
    //   makeWorldBoss - 2 outdoor world bosses (Doom Lord
    //                   Kazzak / Doomwalker) - single phase,
    //                   no enrage timer, 25-player size.
    static WoweeBossEncounter makeFiveMan(const std::string& catalogName);
    static WoweeBossEncounter makeRaid10(const std::string& catalogName);
    static WoweeBossEncounter makeWorldBoss(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
