#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Pet System catalog (.wpet) - novel replacement
// for AzerothCore-style pet_template + pet_levelstats SQL
// + the pet-related subsets of CreatureFamily.dbc and
// SpellFamilyName.dbc. The 38th open format added to the
// editor.
//
// Defines two related kinds of player-controlled NPCs:
//   • Pet families  - hunter pet families (Wolf / Cat / Bear /
//                      Boar / Raptor / Spider / etc.) with
//                      per-family ability sets, base stat
//                      multipliers, and diet preferences
//   • Warlock minions - Imp / Voidwalker / Succubus /
//                        Felhunter / Felguard, each with
//                        their own summon spell and
//                        ability list
//
// Cross-references with previously-added formats:
//   WPET.family.familyId             → WCRT.entry.familyId
//                                       (links to creature family)
//   WPET.family.abilities.spellId    → WSPL.entry.spellId
//   WPET.minion.summonSpellId        → WSPL.entry.spellId
//   WPET.minion.creatureId           → WCRT.entry.creatureId
//   WPET.minion.abilities.spellId    → WSPL.entry.spellId
//
// Binary layout (little-endian):
//   magic[4]            = "WPET"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   familyCount (uint32)
//   families (each):
//     familyId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     petType (uint8) / pad[3]
//     baseAttackSpeed (float) / damageMultiplier (float)
//     armorMultiplier (float)
//     dietMask (uint32)
//     abilityCount (uint8) + pad[3] +
//       abilities (each: spellId(4) + learnedAtLevel(2) + rank(1) + pad)
//   minionCount (uint32)
//   minions (each):
//     minionId (uint32)
//     nameLen + name
//     summonSpellId (uint32) / creatureId (uint32)
//     abilityCount (uint8) + pad[3] +
//       abilities (each: spellId(4) + rank(1) + autocast(1) + pad[2])
struct WoweePet {
    enum PetType : uint8_t {
        Cunning  = 0,
        Ferocity = 1,
        Tenacity = 2,
    };

    enum DietFlags : uint32_t {
        DietMeat    = 0x01,
        DietFish    = 0x02,
        DietBread   = 0x04,
        DietCheese  = 0x08,
        DietFruit   = 0x10,
        DietFungus  = 0x20,
    };

    struct FamilyAbility {
        uint32_t spellId = 0;
        uint16_t learnedAtLevel = 1;
        uint8_t rank = 1;
    };

    struct Family {
        uint32_t familyId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t petType = Cunning;
        float baseAttackSpeed = 2.0f;
        float damageMultiplier = 1.0f;
        float armorMultiplier = 1.0f;
        uint32_t dietMask = 0;
        std::vector<FamilyAbility> abilities;
    };

    struct MinionAbility {
        uint32_t spellId = 0;
        uint8_t rank = 1;
        uint8_t autocastDefault = 0;     // 1 = autocast enabled by default
    };

    struct Minion {
        uint32_t minionId = 0;
        std::string name;
        uint32_t summonSpellId = 0;
        uint32_t creatureId = 0;          // WCRT cross-ref for stats
        std::vector<MinionAbility> abilities;
    };

    std::string name;
    std::vector<Family> families;
    std::vector<Minion> minions;

    [[nodiscard]] bool isValid() const { return !families.empty() || !minions.empty(); }

    [[nodiscard]] const Family* findFamily(uint32_t familyId) const;
    [[nodiscard]] const Minion* findMinion(uint32_t minionId) const;

    static const char* petTypeName(uint8_t t);
    // Decode a dietMask into a short string ("meat+fish").
    static std::string dietMaskName(uint32_t mask);
};

class WoweePetLoader {
public:
    static bool save(const WoweePet& cat,
                     const std::string& basePath);
    static WoweePet load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-pets* variants.
    //
    //   makeStarter  - 2 hunter families (Wolf + Cat) with
    //                   3 abilities each + 1 warlock minion
    //                   (Imp).
    //   makeHunter   - full beast family set (8 classic
    //                   families: Wolf / Cat / Bear / Boar /
    //                   Raptor / Hyena / Spider / Bat) with
    //                   per-type Cunning/Ferocity/Tenacity
    //                   classification.
    //   makeWarlock  - 5 minions (Imp / Voidwalker /
    //                   Succubus / Felhunter / Felguard) with
    //                   summon spell + creature template
    //                   cross-refs.
    static WoweePet makeStarter(const std::string& catalogName);
    static WoweePet makeHunter(const std::string& catalogName);
    static WoweePet makeWarlock(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
