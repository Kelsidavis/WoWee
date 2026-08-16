#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Death Knight Rune Cost catalog (.wrun) - novel
// replacement for Blizzard's RuneCost.dbc plus the DK-specific
// portions of ChrPowerType. Defines per-spell rune costs
// (Blood / Frost / Unholy) and runic-power generation /
// consumption for the Death Knight class.
//
// Each entry binds a spell to its rune cost: how many of each
// rune kind the spell consumes, and how much runic power it
// generates (positive) or spends (negative). Death runes -
// the wildcard rune that fills any slot - are tracked
// implicitly: a spell with anyDeathConvertCost > 0 will
// consume Death runes preferentially over its specified type.
//
// Cross-references with previously-added formats:
//   WRUN.entry.spellId → WSPL.spellId (the spell that uses
//                        this rune cost)
//
// Binary layout (little-endian):
//   magic[4]            = "WRUN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     runeCostId (uint32)
//     spellId (uint32)
//     nameLen + name
//     descLen + description
//     bloodCost (uint8) / frostCost (uint8) /
//       unholyCost (uint8) / anyDeathConvertCost (uint8)
//     runicPowerCost (int16) / pad[2]
//     spellTreeBranch (uint8) / pad[3]
struct WoweeRuneCost {
    enum SpellTreeBranch : uint8_t {
        BloodTree    = 0,    // tank-spec blood tree
        FrostTree    = 1,    // 2H DPS frost tree
        UnholyTree   = 2,    // pet-spec unholy tree
        Generic      = 3,    // baseline non-tree-specific
    };

    struct Entry {
        uint32_t runeCostId = 0;
        uint32_t spellId = 0;            // WSPL cross-ref
        std::string name;
        std::string description;
        uint8_t bloodCost = 0;           // # blood runes consumed
        uint8_t frostCost = 0;           // # frost runes consumed
        uint8_t unholyCost = 0;          // # unholy runes consumed
        uint8_t anyDeathConvertCost = 0; // # additional Death-rune-OK runes
        int16_t runicPowerCost = 0;      // < 0 = generator, > 0 = spender
        uint8_t spellTreeBranch = Generic;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t runeCostId) const;

    static const char* spellTreeBranchName(uint8_t b);
};

class WoweeRuneCostLoader {
public:
    static bool save(const WoweeRuneCost& cat,
                     const std::string& basePath);
    static WoweeRuneCost load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-rune* variants.
    //
    //   makeStarter - 3 baseline DK abilities (Death Strike
    //                  1B+1F, Frost Strike +20 RP spender,
    //                  Heart Strike 1B generator).
    //   makeBlood   - 4 blood-tree abilities (Heart Strike,
    //                  Death and Decay, Vampiric Blood, Rune
    //                  Tap) - tanking + self-heal kit.
    //   makeFrost   - 4 frost-tree abilities (Frost Strike,
    //                  Howling Blast, Obliterate, Icy Touch)
    //                  - DPS rotation kit.
    static WoweeRuneCost makeStarter(const std::string& catalogName);
    static WoweeRuneCost makeBlood(const std::string& catalogName);
    static WoweeRuneCost makeFrost(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
