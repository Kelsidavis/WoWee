#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Skill Catalog (.wskl) - novel replacement for
// Blizzard's SkillLine.dbc + SkillLineCategory.dbc + the
// AzerothCore-style player_classlevelstats / skill base
// tables. The 19th open format added to the editor.
//
// Defines every player-trackable skill: weapon proficiencies
// (Swords, Axes, Bows), professions (Mining, Alchemy,
// Cooking), languages (Common, Dwarvish), class
// specializations (Fire, Frost, Holy, Protection),
// armor proficiencies (Mail, Plate), and secondary skills
// (First Aid, Lockpicking, Riding).
//
// Cross-references with previously-added formats:
//   WLCK.channel.targetId (kind=Lockpick) → WSKL.entry.skillId
//   WGOT.entry.requiredSkill              → WSKL.entry.skillId
//
// Binary layout (little-endian):
//   magic[4]            = "WSKL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     skillId (uint32)
//     nameLen + name
//     descLen + description
//     categoryId (uint8) / canTrain (uint8) / pad[2]
//     maxRank (uint16) / rankPerLevel (uint16)
//     iconLen + iconPath
struct WoweeSkill {
    enum CategoryId : uint8_t {
        Weapon              = 0,
        Class               = 1,    // class spec trees (Fire, Holy, ...)
        Profession          = 2,    // primary: Mining, Alchemy
        SecondaryProfession = 3,    // First Aid, Cooking, Fishing
        Language            = 4,
        ArmorProficiency    = 5,    // Mail, Plate, Shields
        Riding              = 6,
        WeaponSpec          = 7,    // class weapon-specialization talents
    };

    struct Entry {
        uint32_t skillId = 0;
        std::string name;
        std::string description;
        uint8_t categoryId = Profession;
        uint8_t canTrain = 1;          // 1 = requires trainer
        uint16_t maxRank = 300;        // typical classic profession cap
        uint16_t rankPerLevel = 0;     // weapon skills auto-grow
        std::string iconPath;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by skillId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t skillId) const;

    static const char* categoryName(uint8_t c);
};

class WoweeSkillLoader {
public:
    static bool save(const WoweeSkill& cat,
                     const std::string& basePath);
    static WoweeSkill load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-skills* variants.
    //
    //   makeStarter - minimal: Swords + Lockpicking + Mining +
    //                  First Aid + Common (one per category that
    //                  the runtime uses immediately).
    //   makeProfessions - full primary + secondary profession
    //                      set (the 12 classic gathering /
    //                      crafting professions).
    //   makeWeapons - every weapon-skill slot with WoW's
    //                  canonical max-rank scaling (rankPerLevel=5).
    static WoweeSkill makeStarter(const std::string& catalogName);
    static WoweeSkill makeProfessions(const std::string& catalogName);
    static WoweeSkill makeWeapons(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
