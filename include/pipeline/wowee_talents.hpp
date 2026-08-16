#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Talent catalog (.wtal) - novel replacement for
// Blizzard's TalentTab.dbc + Talent.dbc + the AzerothCore-
// style talent_progression SQL tables. The 25th open format
// added to the editor.
//
// Defines class talent specialization trees: a per-class set
// of named tabs (Arms / Fury / Protection for warrior, Fire
// / Frost / Arcane for mage, etc.), each with up to N
// talents arranged in a row/column grid, each talent having
// up to 5 ranks and an optional prerequisite chain.
//
// Cross-references with previously-added formats:
//   WTAL.talent.prereqTalentId → WTAL.talent.talentId
//                                  (intra-format chain)
//   WTAL.talent.rankSpellIds[] → WSPL.entry.spellId
//                                  (spell granted at each rank)
//
// Binary layout (little-endian):
//   magic[4]            = "WTAL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   treeCount (uint32)
//   trees (each):
//     treeId (uint32)
//     nameLen + name
//     iconLen + iconPath
//     requiredClassMask (uint32)
//     talentCount (uint16) + pad[2]
//     talents (talentCount × {
//       talentId (uint32)
//       row (uint8) / col (uint8) / maxRank (uint8) / pad[1]
//       prereqTalentId (uint32) / prereqRank (uint8) / pad[3]
//       rankSpellIds[5] (uint32 each, zero-padded for unused ranks)
//     })
struct WoweeTalent {
    static constexpr int kMaxRanks = 5;

    // Class IDs follow Blizzard's CharClasses.dbc canonical
    // bit positions (bit N corresponds to classId N+1):
    //   1=Warrior 2=Paladin 3=Hunter 4=Rogue 5=Priest
    //   6=DK 7=Shaman 8=Mage 9=Warlock 10=- 11=Druid
    enum ClassMask : uint32_t {
        ClassWarrior = 1u << 0,
        ClassPaladin = 1u << 1,
        ClassHunter  = 1u << 2,
        ClassRogue   = 1u << 3,
        ClassPriest  = 1u << 4,
        ClassDK      = 1u << 5,
        ClassShaman  = 1u << 6,
        ClassMage    = 1u << 7,
        ClassWarlock = 1u << 8,
        ClassDruid   = 1u << 10,
    };

    struct Talent {
        uint32_t talentId = 0;
        uint8_t row = 0;
        uint8_t col = 0;
        uint8_t maxRank = 1;
        uint32_t prereqTalentId = 0;
        uint8_t prereqRank = 0;
        uint32_t rankSpellIds[kMaxRanks] = {0, 0, 0, 0, 0};
    };

    struct Tree {
        uint32_t treeId = 0;
        std::string name;
        std::string iconPath;
        uint32_t requiredClassMask = 0;
        std::vector<Talent> talents;
    };

    std::string name;
    std::vector<Tree> trees;

    [[nodiscard]] bool isValid() const { return !trees.empty(); }

    [[nodiscard]] const Tree* findTree(uint32_t treeId) const;
    // Talent lookup is global across all trees (talentIds are
    // expected to be unique within a single .wtal catalog).
    [[nodiscard]] const Talent* findTalent(uint32_t talentId) const;
};

class WoweeTalentLoader {
public:
    static bool save(const WoweeTalent& cat,
                     const std::string& basePath);
    static WoweeTalent load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-talents* variants.
    //
    //   makeStarter - 1 small tree (3 talents, no prereqs).
    //   makeWarrior - 3 trees (Arms / Fury / Protection) each
    //                  with a handful of talents, prerequisite
    //                  chains, and rankSpellIds wired to WSPL
    //                  warrior preset spells where applicable.
    //   makeMage    - 3 trees (Arcane / Fire / Frost) with
    //                  links to WSPL mage preset spell IDs.
    static WoweeTalent makeStarter(const std::string& catalogName);
    static WoweeTalent makeWarrior(const std::string& catalogName);
    static WoweeTalent makeMage(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
