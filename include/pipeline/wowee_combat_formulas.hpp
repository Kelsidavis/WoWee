#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Combat Formula catalog (.wcfr) -
// novel replacement for the per-stat-conversion
// ratios vanilla WoW carried in the gtChanceTo*.
// dbc gameobject tables + the per-class hard-coded
// constants in the server's StatSystem (the
// "Strength gives 2 AP for Warriors but 1 AP for
// Mages" rule was hard-coded; the "1 Agility = 1
// Crit% for Hunters but 0.5 Crit% for Druids" was
// hard-coded). Each WCFR entry binds one (output
// stat, input stat, class) tuple to its conversion
// ratio in fixed-point units, with optional level-
// band gating.
//
// Cross-references with previously-added formats:
//   WCDB: classRestriction bitmask references the
//         WCDB playable-class catalog.
//   WCST: provides the BASE stat values that this
//         catalog converts INTO derived stats.
//
// Binary layout (little-endian):
//   magic[4]            = "WCFR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     formulaId (uint32)
//     nameLen + name
//     outputStatKind (uint8)       - 0=AP /
//                                     1=SpellPower /
//                                     2=CritPct /
//                                     3=DodgePct /
//                                     4=ParryPct /
//                                     5=HitPct /
//                                     6=SpellCritPct
//                                     /7=HastePct
//     inputStatKind (uint8)        - 0=Strength /
//                                     1=Agility /
//                                     2=Stamina /
//                                     3=Intellect /
//                                     4=Spirit
//     levelMin (uint8)             - 1..80
//     levelMax (uint8)             - 0 = no upper
//                                     gate
//     classRestriction (uint16)    - bitmask of
//                                     classes (1<<
//                                     classId; 0 =
//                                     all classes)
//     pad0 (uint16)
//     conversionRatioFp_x100 (uint32) - fixed-point
//                                       100 = 1.0
//                                       conv. e.g.
//                                       200 means
//                                       1 input stat
//                                       gives 2.0
//                                       output
//                                       stat. 50
//                                       means 0.5.
struct WoweeCombatFormulas {
    enum OutputStatKind : uint8_t {
        AttackPower    = 0,
        SpellPower     = 1,
        CritPct        = 2,
        DodgePct       = 3,
        ParryPct       = 4,
        HitPct         = 5,
        SpellCritPct   = 6,
        HastePct       = 7,
    };

    enum InputStatKind : uint8_t {
        Strength  = 0,
        Agility   = 1,
        Stamina   = 2,
        Intellect = 3,
        Spirit    = 4,
    };

    static constexpr uint32_t kFpScale = 100;

    struct Entry {
        uint32_t formulaId = 0;
        std::string name;
        uint8_t outputStatKind = AttackPower;
        uint8_t inputStatKind = Strength;
        uint8_t levelMin = 0;
        uint8_t levelMax = 0;
        uint16_t classRestriction = 0;
        uint16_t pad0 = 0;
        uint32_t conversionRatioFp_x100 = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t formulaId) const;

    // Returns all formulas that produce a given
    // output stat for a given class+level - used
    // by the combat math hot path to compute
    // derived stats from base stats.
    [[nodiscard]] std::vector<const Entry*> findApplicable(
        uint8_t outputStatKind,
        uint8_t classId,
        uint8_t level) const;
};

class WoweeCombatFormulasLoader {
public:
    static bool save(const WoweeCombatFormulas& cat,
                     const std::string& basePath);
    static WoweeCombatFormulas load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cfr* variants.
    //
    //   makeWarriorFormulas  - 4 Warrior conversion
    //                           formulas (Str→AP 2.0
    //                           / Agi→Crit 0.05 /
    //                           Agi→Dodge 0.05 /
    //                           Sta→ no derived).
    //   makeMageFormulas     - 3 Mage conversion
    //                           formulas (Int→
    //                           SpellPower 1.0 / Int
    //                           →SpellCrit 0.0167 /
    //                           Spi→ regen).
    //   makeRogueFormulas    - 4 Rogue formulas (Str
    //                           →AP 1.0 / Agi→AP 1.0
    //                           / Agi→Crit 0.0714 /
    //                           Agi→Dodge 0.0714).
    //                           Demonstrates per-
    //                           class ratio
    //                           variation: Rogue
    //                           gets Crit from Agi
    //                           at a different rate
    //                           than Warrior.
    static WoweeCombatFormulas makeWarriorFormulas(const std::string& catalogName);
    static WoweeCombatFormulas makeMageFormulas(const std::string& catalogName);
    static WoweeCombatFormulas makeRogueFormulas(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
