#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Difficulty catalog (.wcdf) - novel
// replacement for Blizzard's CreatureDifficulty.dbc. Maps
// a base creature entry to its difficulty variants:
// Normal-10 / Normal-25 / Heroic-10 / Heroic-25 in WotLK
// raid format. Each variant is itself a separate WCRT
// creature entry with its own stats, abilities, and loot.
//
// When a 25-man party engages an instance, the engine
// looks up the encounter base creature's difficultyId,
// reads the normal25Id field, and spawns that variant
// instead. This is how Lord Marrowgar in 25-Heroic ICC
// has 30M HP and hits for 80k while the same encounter
// in 10-Normal has 5M HP and hits for 25k - same spawn
// point, different WCRT entries.
//
// 5-man dungeons typically use only normal10Id +
// heroic10Id (the 25-man fields stay 0). World bosses
// and rare elites that don't scale typically have a single
// non-zero variant matching the base id.
//
// Cross-references with previously-added formats:
//   WCRT: every non-zero *Id field points at a
//         WCRT.creatureId entry. The base creature lives
//         in WCRT too - this catalog is purely the
//         routing table from base id to variants.
//
// Binary layout (little-endian):
//   magic[4]            = "WCDF"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     difficultyId (uint32)
//     nameLen + name
//     descLen + description
//     baseCreatureId (uint32)
//     normal10Id (uint32)
//     normal25Id (uint32)
//     heroic10Id (uint32)
//     heroic25Id (uint32)
//     spawnGroupKind (uint8) / pad[3]
//     iconColorRGBA (uint32)
struct WoweeCreatureDifficulty {
    enum SpawnGroupKind : uint8_t {
        Boss       = 0,    // encounter boss / final boss
        MiniBoss   = 1,    // sub-boss / room boss
        RareElite  = 2,    // outdoor rare spawn
        Trash      = 3,    // pull / hallway mob
        Add        = 4,    // boss-spawned add
        WorldBoss  = 5,    // open-world raid boss (no diff variants)
    };

    struct Entry {
        uint32_t difficultyId = 0;
        std::string name;
        std::string description;
        uint32_t baseCreatureId = 0;
        uint32_t normal10Id = 0;
        uint32_t normal25Id = 0;
        uint32_t heroic10Id = 0;
        uint32_t heroic25Id = 0;
        uint8_t spawnGroupKind = Boss;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t difficultyId) const;
    [[nodiscard]] const Entry* findByBaseCreature(uint32_t baseCreatureId) const;

    // Resolve to the variant creature id for the given
    // difficulty index. mode: 0=Normal-10, 1=Normal-25,
    // 2=Heroic-10, 3=Heroic-25. Returns 0 if no variant
    // is configured for that mode (caller falls back to
    // baseCreatureId).
    [[nodiscard]] uint32_t resolveVariant(uint32_t difficultyId,
                             uint8_t mode) const;

    static const char* spawnGroupKindName(uint8_t k);
};

class WoweeCreatureDifficultyLoader {
public:
    static bool save(const WoweeCreatureDifficulty& cat,
                     const std::string& basePath);
    static WoweeCreatureDifficulty load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cdf* variants.
    //
    //   makeStarter    - 4 example boss entries with full
    //                     4-variant 10/25/H10/H25 routing,
    //                     using fictional creature ids in
    //                     the 8000-8200 range.
    //   makeWotlkRaid  - 4 Icecrown Citadel-style bosses
    //                     (Marrowgar / Deathwhisper /
    //                     Saurfang / Lich King) with full
    //                     diff variants.
    //   makeFiveMan    - 4 five-man dungeon bosses with
    //                     only Normal + Heroic 10-man
    //                     variants set (25-man fields stay
    //                     0 - engine falls through to the
    //                     10-man variant when 25-man is
    //                     queried).
    static WoweeCreatureDifficulty makeStarter(const std::string& catalogName);
    static WoweeCreatureDifficulty makeWotlkRaid(const std::string& catalogName);
    static WoweeCreatureDifficulty makeFiveMan(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
