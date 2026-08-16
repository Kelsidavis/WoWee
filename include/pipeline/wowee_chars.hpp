#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Character Classes/Races catalog (.wchc) -
// novel replacement for Blizzard's CharClasses.dbc +
// CharRaces.dbc + CharStartOutfit.dbc trio. The 27th open
// format added to the editor.
//
// Defines every player class, race, and the starting outfit
// (gear loadout) for each class+race+gender combination.
// One file holds three flat arrays: classes / races /
// outfits.
//
// Cross-references with previously-added formats:
//   WCHC.race.startingMapId        → WMS.map.mapId
//   WCHC.race.startingZoneAreaId   → WMS.area.areaId
//   WCHC.race.defaultLanguageSpellId → WSPL.entry.spellId
//   WCHC.race.mountSpellId         → WSPL.entry.spellId
//   WCHC.outfit.items.itemId       → WIT.entry.itemId
//   WCHC.class.canTrainProfessions → WSKL.entry.skillId
//                                     (bitset by category)
//
// Binary layout (little-endian):
//   magic[4]            = "WCHC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   classCount (uint32) + classes[]
//   raceCount (uint32) + races[]
//   outfitCount (uint32) + outfits[]
struct WoweeChars {
    enum PowerType : uint8_t {
        Mana        = 0,
        Rage        = 1,
        Focus       = 2,
        Energy      = 3,
        RunicPower  = 4,
        Runes       = 6,    // DK only
    };

    enum FactionAvailability : uint8_t {
        AvailableAlliance = 0x01,
        AvailableHorde    = 0x02,
    };

    enum RaceFaction : uint8_t {
        Alliance = 0,
        Horde    = 1,
        Neutral  = 2,    // Pandaren start neutral
    };

    enum Gender : uint8_t {
        Male   = 0,
        Female = 1,
    };

    struct Class {
        uint32_t classId = 0;
        std::string name;
        std::string iconPath;
        uint8_t powerType = Mana;
        uint8_t displayPower = Mana;     // can differ from powerType (druid)
        uint32_t baseHealth = 50;
        uint16_t baseHealthPerLevel = 12;
        uint32_t basePower = 100;
        uint16_t basePowerPerLevel = 5;
        uint8_t factionAvailability =
            AvailableAlliance | AvailableHorde;
    };

    struct Race {
        uint32_t raceId = 0;
        std::string name;
        std::string iconPath;
        uint8_t factionId = Alliance;
        uint32_t maleDisplayId = 0;
        uint32_t femaleDisplayId = 0;
        uint16_t baseStrength = 20;
        uint16_t baseAgility = 20;
        uint16_t baseStamina = 20;
        uint16_t baseIntellect = 20;
        uint16_t baseSpirit = 20;
        uint32_t startingMapId = 0;
        uint32_t startingZoneAreaId = 0;
        uint32_t defaultLanguageSpellId = 0;     // 0 = none
        uint32_t mountSpellId = 0;               // racial mount spell
    };

    struct OutfitItem {
        uint32_t itemId = 0;
        uint8_t displaySlot = 0;        // matches WIT.inventoryType
    };

    struct Outfit {
        uint32_t classId = 0;
        uint32_t raceId = 0;
        uint8_t gender = Male;
        std::vector<OutfitItem> items;
    };

    std::string name;
    std::vector<Class> classes;
    std::vector<Race> races;
    std::vector<Outfit> outfits;

    [[nodiscard]] bool isValid() const { return !classes.empty() || !races.empty(); }

    [[nodiscard]] const Class* findClass(uint32_t classId) const;
    [[nodiscard]] const Race* findRace(uint32_t raceId) const;
    // First outfit matching the (class, race, gender) triple, or nullptr.
    [[nodiscard]] const Outfit* findOutfit(uint32_t classId, uint32_t raceId,
                              uint8_t gender) const;

    static const char* powerTypeName(uint8_t p);
    static const char* raceFactionName(uint8_t f);
};

class WoweeCharsLoader {
public:
    static bool save(const WoweeChars& cat,
                     const std::string& basePath);
    static WoweeChars load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-chars* variants.
    //
    //   makeStarter - 2 classes (Warrior + Mage) + 2 races
    //                  (Human Alliance + Orc Horde) + 4 outfits
    //                  (2 classes × 2 races, male-only).
    //   makeAlliance - full Alliance faction: 4 classes + 4
    //                   races + 8 outfits.
    //   makeAllRaces - 8 classic playable races (Human / Dwarf
    //                   / NightElf / Gnome on Alliance side;
    //                   Orc / Undead / Tauren / Troll on Horde)
    //                   plus 9 classes (no DK).
    static WoweeChars makeStarter(const std::string& catalogName);
    static WoweeChars makeAlliance(const std::string& catalogName);
    static WoweeChars makeAllRaces(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
