#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Set catalog (.wset) - novel replacement
// for Blizzard's ItemSet.dbc + ItemSetSpell.dbc plus the
// AzerothCore-style item_set_spell SQL data. New open
// format that closes the tier-bonus gap.
//
// Defines tiered item sets like "Battlegear of Wrath" or
// "Vengeful Gladiator's Plate" - N piece IDs (up to 8) plus
// M bonus thresholds (up to 4) where each threshold maps to
// a spell aura that activates when the player wears at least
// `threshold` pieces simultaneously. Standard 2-piece /
// 4-piece / 6-piece / 8-piece set-bonus pattern is the
// canonical case; partial bonuses are supported by leaving
// later thresholds at 0.
//
// Cross-references with previously-added formats:
//   WSET.entry.itemIds[]       → WIT.itemId
//   WSET.entry.bonusSpellIds[] → WSPL.spellId
//   WSET.entry.requiredSkillId → WSKL.skillId   (optional)
//
// Binary layout (little-endian):
//   magic[4]            = "WSET"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     setId (uint32)
//     nameLen + name
//     descLen + description
//     pieceCount (uint8) / bonusCount (uint8) / pad[2]
//     requiredClassMask (uint32)
//     requiredSkillId (uint16) / requiredSkillRank (uint16)
//     itemIds[8] (uint32)            - unused slots = 0
//     bonusThresholds[4] (uint8) / pad[4]
//     bonusSpellIds[4] (uint32)
struct WoweeItemSet {
    static constexpr size_t kMaxPieces = 8;
    static constexpr size_t kMaxBonuses = 4;

    // Class-mask bits matching WCHC.classId (1=Warrior=bit 1,
    // 2=Paladin, 3=Hunter, 4=Rogue, 5=Priest, 6=DK, 7=Shaman,
    // 8=Mage, 9=Warlock, 11=Druid). 32-bit so the wider WotLK
    // class set (Druid bit 11) fits - same convention as WGLY.
    static constexpr uint32_t kClassNone     = 0;
    static constexpr uint32_t kClassWarrior  = 1u << 1;
    static constexpr uint32_t kClassPaladin  = 1u << 2;
    static constexpr uint32_t kClassHunter   = 1u << 3;
    static constexpr uint32_t kClassRogue    = 1u << 4;
    static constexpr uint32_t kClassPriest   = 1u << 5;
    static constexpr uint32_t kClassMage     = 1u << 8;
    static constexpr uint32_t kClassWarlock  = 1u << 9;
    static constexpr uint32_t kClassDruid    = 1u << 11;
    // Convenience composites.
    static constexpr uint32_t kClassPlate    = kClassWarrior | kClassPaladin;
    static constexpr uint32_t kClassCloth    = kClassMage | kClassWarlock |
                                               kClassPriest;

    struct Entry {
        uint32_t setId = 0;
        std::string name;
        std::string description;
        uint8_t pieceCount = 0;       // # of populated itemIds[]
        uint8_t bonusCount = 0;       // # of populated bonus pairs
        uint32_t requiredClassMask = kClassNone;  // wide mask
        uint16_t requiredSkillId = 0;     // WSKL cross-ref
        uint16_t requiredSkillRank = 0;
        uint32_t itemIds[kMaxPieces] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint8_t bonusThresholds[kMaxBonuses] = {0, 0, 0, 0};
        uint32_t bonusSpellIds[kMaxBonuses] = {0, 0, 0, 0};
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t setId) const;
};

class WoweeItemSetLoader {
public:
    static bool save(const WoweeItemSet& cat,
                     const std::string& basePath);
    static WoweeItemSet load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-itset* variants.
    //
    //   makeStarter - 2 raid sets (Battlegear of Wrath
    //                  warrior tier-2; Stormrage Raiment
    //                  druid tier-2) with 8-piece layouts.
    //   makeTier    - 4 progression sets across class-role
    //                  archetypes (warrior plate, mage cloth,
    //                  rogue leather, paladin holy plate).
    //   makePvP     - 3 PvP gladiator sets (Vindication,
    //                  Doomcaller, Predatory) with 5-piece
    //                  layouts and 2/4-piece set bonuses.
    static WoweeItemSet makeStarter(const std::string& catalogName);
    static WoweeItemSet makeTier(const std::string& catalogName);
    static WoweeItemSet makePvP(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
