#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Family catalog (.wcef) - novel
// replacement for Blizzard's CreatureFamily.dbc plus the
// per-creature family fields in Creature.dbc. Defines the
// family categorization that pet-able beasts share: Bear /
// Cat / Wolf / Boar / etc. Each family carries its own
// pet talent tree, food preferences, and the minimum
// hunter level required to tame it.
//
// Used by the hunter pet system to:
//   - decide which talent tree (Ferocity / Tenacity /
//     Cunning) a tamed pet uses,
//   - validate that a hunter can tame a creature
//     (minLevelForTame),
//   - match feeding-table food items to pet preferences
//     via the petFoodTypes bitmask,
//   - and gate exotic-beast taming behind the Beast
//     Master 51-point talent.
//
// Cross-references with previously-added formats:
//   WCRT: creature.familyId points back to a WCEF entry.
//   WSPL: family-specific abilities reference WSPL spellId
//         via skillLine.
//
// Binary layout (little-endian):
//   magic[4]            = "WCEF"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     familyId (uint32)
//     nameLen + name
//     descLen + description
//     familyKind (uint8) / petTalentTree (uint8)
//     minLevelForTame (uint8) / pad (uint8)
//     skillLine (uint32)
//     petFoodTypes (uint32)
//     iconColorRGBA (uint32)
struct WoweeCreatureFamily {
    enum FamilyKind : uint8_t {
        Beast      = 0,    // standard tamable beast
        Demon      = 1,    // warlock minion family (Imp, Voidwalker)
        Undead     = 2,    // Death Knight ghoul / unholy minion
        Elemental  = 3,    // shaman elemental totem
        NotPet     = 4,    // creature has a family but is not pet-able
        Exotic     = 5,    // tamable only by Beast Master 51-point talent
    };

    enum PetTalentTree : uint8_t {
        TreeNone     = 0,
        Ferocity     = 1,    // damage-focused (Cat, Wolf, Raptor)
        Tenacity     = 2,    // tank-focused (Bear, Boar, Crab)
        Cunning      = 3,    // utility (Bird of Prey, Spider)
    };

    enum FoodType : uint32_t {
        Meat     = 1u << 0,
        Fish     = 1u << 1,
        Bread    = 1u << 2,
        Cheese   = 1u << 3,
        Fruit    = 1u << 4,
        Fungus   = 1u << 5,
        Raw      = 1u << 6,    // raw/uncooked variant accepted
    };

    struct Entry {
        uint32_t familyId = 0;
        std::string name;
        std::string description;
        uint8_t familyKind = Beast;
        uint8_t petTalentTree = TreeNone;
        uint8_t minLevelForTame = 10;
        uint8_t pad0 = 0;
        uint32_t skillLine = 0;
        uint32_t petFoodTypes = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t familyId) const;

    static const char* familyKindName(uint8_t k);
    static const char* petTalentTreeName(uint8_t t);
};

class WoweeCreatureFamilyLoader {
public:
    static bool save(const WoweeCreatureFamily& cat,
                     const std::string& basePath);
    static WoweeCreatureFamily load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cef* variants.
    //
    //   makeStarter   - 5 baseline beast families (Bear,
    //                    Cat, Wolf, Boar, Crab) covering
    //                    one entry per pet talent tree
    //                    plus a bonus Tenacity entry.
    //   makeFerocity  - 4 Ferocity-tree damage pets
    //                    (Cat, Wolf, Devilsaur, Raptor).
    //   makeExotic    - 4 exotic Beast Master families
    //                    (Worm, Devilsaur, Chimaera,
    //                    CoreHound) - Exotic kind, requires
    //                    51-point BM talent to tame.
    static WoweeCreatureFamily makeStarter(const std::string& catalogName);
    static WoweeCreatureFamily makeFerocity(const std::string& catalogName);
    static WoweeCreatureFamily makeExotic(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
