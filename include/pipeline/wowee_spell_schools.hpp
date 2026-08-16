#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell School catalog (.wsch) - novel
// replacement for Blizzard's SpellSchools.dbc plus the
// Resistances.dbc resistance-cap tables. Defines the damage
// schools spells use: Physical, Holy, Fire, Nature, Frost,
// Shadow, Arcane, plus combined / hybrid schools that count
// as multiple types simultaneously (Spellfire, Spellfrost,
// Spellshadow - relevant for resistance bypass mechanics).
//
// Each school carries the visual identity (color tint,
// icon), the gameplay rules (can be absorbed by shields,
// can crit, can creatures be immune), the resistance cap
// at max level, and audio (cast / impact sound IDs).
//
// Cross-references with previously-added formats:
//   WSCH.entry.castSoundId   → WSND.soundId
//   WSCH.entry.impactSoundId → WSND.soundId
//   WSCH.entry.combinedSchoolMask is a bitmask of OTHER
//                                  WSCH.schoolId values -
//                                  hybrid schools set this
//                                  to expose multi-type
//                                  damage profiles.
//
// Binary layout (little-endian):
//   magic[4]            = "WSCH"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     schoolId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     canBeImmune (uint8) / canBeAbsorbed (uint8) /
//       canBeReflected (uint8) / canCrit (uint8)
//     colorRGBA (uint32)
//     baseResistanceCap (uint32)
//     castSoundId (uint32)
//     impactSoundId (uint32)
//     combinedSchoolMask (uint32)
struct WoweeSpellSchool {
    // Canonical school IDs match WoW's spell school enum so
    // hybrid masks line up with what the spell engine expects.
    static constexpr uint32_t kSchoolPhysical = 1u << 0;
    static constexpr uint32_t kSchoolHoly     = 1u << 1;
    static constexpr uint32_t kSchoolFire     = 1u << 2;
    static constexpr uint32_t kSchoolNature   = 1u << 3;
    static constexpr uint32_t kSchoolFrost    = 1u << 4;
    static constexpr uint32_t kSchoolShadow   = 1u << 5;
    static constexpr uint32_t kSchoolArcane   = 1u << 6;

    struct Entry {
        uint32_t schoolId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t canBeImmune = 1;       // 1 = creatures can be immune
        uint8_t canBeAbsorbed = 1;     // 1 = shields can soak this
        uint8_t canBeReflected = 0;
        uint8_t canCrit = 1;
        uint32_t colorRGBA = 0xFFFFFFFFu;
        uint32_t baseResistanceCap = 0;  // max useful resistance at 80
        uint32_t castSoundId = 0;        // WSND cross-ref
        uint32_t impactSoundId = 0;      // WSND cross-ref
        uint32_t combinedSchoolMask = 0; // hybrid: bitmask of others
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t schoolId) const;
};

class WoweeSpellSchoolLoader {
public:
    static bool save(const WoweeSpellSchool& cat,
                     const std::string& basePath);
    static WoweeSpellSchool load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sch* variants.
    //
    //   makeStarter  - 3 base schools (Physical / Fire /
    //                   Holy) - Physical bypasses absorbs,
    //                   Holy can't be reflected.
    //   makeMagical  - 6 magical schools (Holy / Fire /
    //                   Nature / Frost / Shadow / Arcane)
    //                   covering the full canonical set
    //                   with proper colors + resistance caps.
    //   makeCombined - 3 hybrid schools (Spellfire /
    //                   Spellshadow / Spellfrost) showing
    //                   combinedSchoolMask wiring.
    static WoweeSpellSchool makeStarter(const std::string& catalogName);
    static WoweeSpellSchool makeMagical(const std::string& catalogName);
    static WoweeSpellSchool makeCombined(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
