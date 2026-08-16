#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Talent Tab catalog (.wtle) - novel
// replacement for Blizzard's TalentTab.dbc plus the
// per-tab fields in Spell.dbc / Talent.dbc. Defines the
// three talent trees that each class has - Warrior:
// Arms / Fury / Protection; Mage: Arcane / Fire / Frost;
// Paladin: Holy / Protection / Retribution; etc.
//
// Each tab carries its own name, role hint (DPS / Tank /
// Healer / Hybrid), display order in the talent UI,
// background artwork path, icon, and the class bitmask
// it belongs to.
//
// Distinct from WTAL (Talents) which defines individual
// talent points and their effects. WTLE says "the Arms
// tree exists for Warriors, displays in tab 1, is a DPS
// spec"; WTAL says "Mortal Strike is a 1-point talent in
// the Arms tree, row 7, requires Improved Charge as a
// prerequisite".
//
// Cross-references with previously-added formats:
//   WCHC: classMask uses the same bit layout as WCHC
//         class IDs.
//   WTAL: talent entries reference tabId here.
//
// Binary layout (little-endian):
//   magic[4]            = "WTLE"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tabId (uint32)
//     nameLen + name
//     descLen + description
//     classMask (uint32)
//     displayOrder (uint8) / roleHint (uint8) / pad[2]
//     iconPathLen + iconPath
//     backgroundFileLen + backgroundFile
//     iconColorRGBA (uint32)
struct WoweeTalentTab {
    enum RoleHint : uint8_t {
        DPS         = 0,    // damage-dealing spec
        Tank        = 1,    // mitigation spec
        Healer      = 2,    // healing spec
        Hybrid      = 3,    // can fill multiple roles
        PetClass    = 4,    // hunter/warlock pet-focused tree
    };

    struct Entry {
        uint32_t tabId = 0;
        std::string name;
        std::string description;
        uint32_t classMask = 0;
        uint8_t displayOrder = 0;
        uint8_t roleHint = DPS;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        std::string iconPath;
        std::string backgroundFile;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tabId) const;

    // Return all tabs for the given class, in displayOrder.
    // The talent UI uses this to populate the three (or
    // four, for druids) tab buttons.
    [[nodiscard]] std::vector<const Entry*> findByClass(uint32_t classBit) const;

    static const char* roleHintName(uint8_t r);
};

class WoweeTalentTabLoader {
public:
    static bool save(const WoweeTalentTab& cat,
                     const std::string& basePath);
    static WoweeTalentTab load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tle* variants.
    //
    //   makeWarrior  - 3 tabs (Arms DPS / Fury DPS /
    //                   Protection Tank) with the
    //                   canonical Warrior icon paths.
    //   makeMage     - 3 tabs (Arcane / Fire / Frost),
    //                   all DPS, with the canonical Mage
    //                   icon paths.
    //   makePaladin  - 3 tabs (Holy Healer / Protection
    //                   Tank / Retribution DPS) covering
    //                   all three roles in one preset.
    static WoweeTalentTab makeWarrior(const std::string& catalogName);
    static WoweeTalentTab makeMage(const std::string& catalogName);
    static WoweeTalentTab makePaladin(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
