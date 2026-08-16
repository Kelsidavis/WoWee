#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Companion Pet catalog (.wcmp) - novel
// replacement for the companion-pet portions of CreatureFamily
// .dbc plus the AzerothCore-style critter / vanity-pet SQL
// data. Distinct from WPET (which covers hunter combat pets
// and warlock minions); this format covers non-combat
// "vanity" pets that follow the player around for cosmetic
// reasons - Mechanical Squirrel, Mini Diablo, Panda Cub,
// dragon hatchlings, etc.
//
// Each companion binds a creature template (the rendered
// model) to the spell that summons it, and optionally to the
// item that teaches the spell when used. companionKind groups
// them by visual archetype for filter / collection display;
// rarity drives drop chance and collection-display sort.
//
// Cross-references with previously-added formats:
//   WCMP.entry.creatureId    → WCRT.creatureId
//   WCMP.entry.learnSpellId  → WSPL.spellId
//   WCMP.entry.itemId        → WIT.itemId (teaches the spell)
//   WCMP.entry.idleSoundId   → WSND.soundId (ambient noise)
//
// Binary layout (little-endian):
//   magic[4]            = "WCMP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     companionId (uint32)
//     creatureId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     companionKind (uint8) / rarity (uint8) /
//       factionRestriction (uint8) / pad[1]
//     learnSpellId (uint32)
//     itemId (uint32)
//     idleSoundId (uint32)
struct WoweeCompanion {
    enum CompanionKind : uint8_t {
        Critter         = 0,    // small mundane animals
        Mechanical      = 1,    // engineered constructs
        DragonHatchling = 2,    // small dragons / drakes
        Demonic         = 3,    // imps / felguard pups
        Spectral        = 4,    // ghosts / wisps
        Elemental       = 5,    // fire / water / earth / air sprites
        Plush           = 6,    // promotional toys / collectibles
        UndeadCritter   = 7,    // skeletal pets / ghouls
    };

    enum Rarity : uint8_t {
        Common    = 0,    // vendor / common drop
        Uncommon  = 1,    // dungeon drop / faction quartermaster
        Rare      = 2,    // raid / promo
        Epic      = 3,    // legendary / Blizzcon promo
    };

    enum FactionRestriction : uint8_t {
        AnyFaction = 0,
        AllianceOnly = 1,
        HordeOnly = 2,
    };

    struct Entry {
        uint32_t companionId = 0;
        uint32_t creatureId = 0;          // WCRT cross-ref
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t companionKind = Critter;
        uint8_t rarity = Common;
        uint8_t factionRestriction = AnyFaction;
        uint32_t learnSpellId = 0;        // WSPL cross-ref
        uint32_t itemId = 0;              // WIT cross-ref (optional)
        uint32_t idleSoundId = 0;         // WSND cross-ref (optional)
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t companionId) const;

    static const char* companionKindName(uint8_t k);
    static const char* rarityName(uint8_t r);
    static const char* factionRestrictionName(uint8_t f);
};

class WoweeCompanionLoader {
public:
    static bool save(const WoweeCompanion& cat,
                     const std::string& basePath);
    static WoweeCompanion load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cmp* variants.
    //
    //   makeStarter - 3 common vendor-bought companions
    //                  (Mechanical Squirrel / Cat / Prairie
    //                  Dog) - Common rarity, no faction gate.
    //   makeRare    - 4 rare promo / collector pets (Mini
    //                  Diablo / Panda Cub / Zergling / Murky)
    //                  with Epic / Rare rarity.
    //   makeFaction - 3 faction-specific (Alliance Lion Cub /
    //                  Horde Mottled Boar / Argent Squire)
    //                  using AllianceOnly + HordeOnly +
    //                  AnyFaction.
    static WoweeCompanion makeStarter(const std::string& catalogName);
    static WoweeCompanion makeRare(const std::string& catalogName);
    static WoweeCompanion makeFaction(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
