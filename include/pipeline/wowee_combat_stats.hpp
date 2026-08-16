#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Combat Stats Baseline catalog (.wcst)
// - novel replacement for the per-class per-level
// base-stat scaling table that vanilla WoW
// scattered across CharBaseInfo.dbc +
// CharStartOutfit.dbc + GtChanceTo*.dbc + the
// hard-coded HP/mana-per-level constants in the
// server's StatSystem. Each WCST entry binds one
// (classId, level) pair to its base health, mana,
// armor, and the five primary stats (Str/Agi/Sta/
// Int/Spi). Entries are sparse - typical preset
// emits ~6 sample levels per class, with the
// runtime interpolating between them for
// intermediate levels.
//
// Cross-references with previously-added formats:
//   WCDB:  classId references the playable-class
//          catalog (1..11 in vanilla, with 6 + 10
//          unused).
//   WSPK:  the spell pack catalog gates spellbook
//          tabs by classId - same id space.
//
// Binary layout (little-endian):
//   magic[4]            = "WCST"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     statId (uint32)        - surrogate primary
//                               key
//     classId (uint8)        - 1..11 vanilla class
//     level (uint8)          - 1..60 vanilla cap
//     pad0 (uint16)
//     baseHealth (uint32)
//     baseMana (uint32)      - 0 if class doesn't
//                               use mana (Warrior,
//                               Rogue)
//     baseStrength (uint16)
//     baseAgility (uint16)
//     baseStamina (uint16)
//     baseIntellect (uint16)
//     baseSpirit (uint16)
//     pad1 (uint16)
//     baseArmor (uint32)
struct WoweeCombatStats {
    struct Entry {
        uint32_t statId = 0;
        uint8_t classId = 0;
        uint8_t level = 0;
        uint16_t pad0 = 0;
        uint32_t baseHealth = 0;
        uint32_t baseMana = 0;
        uint16_t baseStrength = 0;
        uint16_t baseAgility = 0;
        uint16_t baseStamina = 0;
        uint16_t baseIntellect = 0;
        uint16_t baseSpirit = 0;
        uint16_t pad1 = 0;
        uint32_t baseArmor = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t statId) const;

    // Returns the binding for an exact (classId, level)
    // pair - used by the level-up handler to commit
    // base-stat changes when a player dings.
    [[nodiscard]] const Entry* find(uint8_t classId, uint8_t level) const;

    // Returns all entries for a class, sorted by level.
    // Used by the runtime stat-interpolator to find the
    // bracketing two entries for an intermediate level.
    [[nodiscard]] std::vector<const Entry*> findByClass(uint8_t classId) const;
};

class WoweeCombatStatsLoader {
public:
    static bool save(const WoweeCombatStats& cat,
                     const std::string& basePath);
    static WoweeCombatStats load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cst* variants.
    //
    //   makeWarriorStats - Warrior (classId=1) sparse
    //                       sample at levels 1, 10, 20,
    //                       30, 40, 60. baseMana=0 for
    //                       all entries (Warrior uses
    //                       Rage, not mana).
    //   makeMageStats    - Mage (classId=8) sparse
    //                       sample at the same 6
    //                       levels. baseMana grows
    //                       with Intellect.
    //   makeStartingLevels - All 9 vanilla classes
    //                         (Warrior/Paladin/Hunter/
    //                         Rogue/Priest/Shaman/Mage/
    //                         Warlock/Druid) at level 1
    //                         only - illustrates per-
    //                         class flat starting-stat
    //                         differences.
    static WoweeCombatStats makeWarriorStats(const std::string& catalogName);
    static WoweeCombatStats makeMageStats(const std::string& catalogName);
    static WoweeCombatStats makeStartingLevels(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
