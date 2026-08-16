#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Skill Cost catalog (.wscs) - novel
// replacement for Blizzard's SkillCostsData.dbc plus the
// per-rank training cost tables. Defines the tiered
// progression of trainable skills: each rank unlocks a
// skill range, requires a minimum character level, and
// costs a fixed amount of gold to learn.
//
// The canonical 6-tier profession progression:
//   Apprentice    skill 0-75     lvl 5    1s
//   Journeyman    skill 50-150   lvl 10   5s
//   Expert        skill 125-225  lvl 20   1g
//   Artisan       skill 200-300  lvl 35   5g
//   Master        skill 275-375  lvl 50   10g
//   Grand Master  skill 350-450  lvl 65   25g
//
// Same shape applies to weapon skills (with different
// caps), riding skills (with level gates per mount tier),
// and class secondary skills (Lockpicking for Rogues,
// First Aid for everyone).
//
// Cross-references with previously-added formats:
//   WSKL: skill entries reference costId here for the
//         tiered training schedule.
//
// Binary layout (little-endian):
//   magic[4]            = "WSCS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     costId (uint32)
//     nameLen + name
//     descLen + description
//     skillRankIndex (uint32)
//     minSkillToLearn (uint16) / maxSkillUnlocked (uint16)
//     requiredLevel (uint8) / costKind (uint8) / pad[2]
//     copperCost (uint32)
//     iconColorRGBA (uint32)
struct WoweeSkillCost {
    enum CostKind : uint8_t {
        Profession   = 0,    // primary/secondary profession rank
        WeaponSkill  = 1,    // weapon skill cap (Sword / Mace / Axe / etc)
        RidingSkill  = 2,    // mount riding skill (60% / 100% / 150% / 280% / Cold)
        ClassSkill   = 3,    // class-specific (Lockpicking / Poisons)
        Misc         = 4,    // catch-all
    };

    struct Entry {
        uint32_t costId = 0;
        std::string name;
        std::string description;
        uint32_t skillRankIndex = 0;
        uint16_t minSkillToLearn = 0;
        uint16_t maxSkillUnlocked = 75;
        uint8_t requiredLevel = 1;
        uint8_t costKind = Profession;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t copperCost = 0;       // 1g = 10000c
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t costId) const;

    // Returns the entry that would be next-trainable for a
    // character with the given current skill points and
    // character level - i.e. the lowest-rank entry the
    // character qualifies for and hasn't already maxed out.
    // Returns nullptr if every entry is either capped or
    // gated by level.
    [[nodiscard]] const Entry* nextTrainable(uint16_t currentSkill,
                                uint8_t characterLevel) const;

    static const char* costKindName(uint8_t k);
};

class WoweeSkillCostLoader {
public:
    static bool save(const WoweeSkillCost& cat,
                     const std::string& basePath);
    static WoweeSkillCost load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-scs* variants.
    //
    //   makeProfession - 6 canonical profession tiers
    //                     (Apprentice through Grand Master)
    //                     with the standard skill ranges and
    //                     gold costs from a Vanilla / TBC /
    //                     WotLK-era server.
    //   makeWeapon     - 5 weapon skill tiers (Beginner /
    //                     Trained / Skilled / Expert /
    //                     Master) for free-to-train weapon
    //                     skills capped at 5x character lvl.
    //   makeRiding     - 5 riding skill tiers (Apprentice
    //                     60% / Journeyman 100% / Expert
    //                     150% / Artisan 280% / Cold Weather
    //                     Flying) with the canonical Vanilla
    //                     /TBC / WotLK gold costs.
    static WoweeSkillCost makeProfession(const std::string& catalogName);
    static WoweeSkillCost makeWeapon(const std::string& catalogName);
    static WoweeSkillCost makeRiding(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
