#include "game/character.hpp"

namespace wowee {
namespace game {

bool isValidRaceClassCombo(Race race, Class cls) {
    // WoW 3.3.5a valid race/class combinations
    switch (race) {
        case Race::HUMAN:
            return cls == Class::WARRIOR || cls == Class::PALADIN || cls == Class::ROGUE ||
                   cls == Class::PRIEST || cls == Class::MAGE || cls == Class::WARLOCK ||
                   cls == Class::DEATH_KNIGHT;
        case Race::ORC:
            return cls == Class::WARRIOR || cls == Class::HUNTER || cls == Class::ROGUE ||
                   cls == Class::SHAMAN || cls == Class::WARLOCK || cls == Class::DEATH_KNIGHT;
        case Race::DWARF:
            return cls == Class::WARRIOR || cls == Class::PALADIN || cls == Class::HUNTER ||
                   cls == Class::ROGUE || cls == Class::PRIEST || cls == Class::DEATH_KNIGHT;
        case Race::NIGHT_ELF:
            return cls == Class::WARRIOR || cls == Class::HUNTER || cls == Class::ROGUE ||
                   cls == Class::PRIEST || cls == Class::DRUID || cls == Class::DEATH_KNIGHT;
        case Race::UNDEAD:
            return cls == Class::WARRIOR || cls == Class::ROGUE || cls == Class::PRIEST ||
                   cls == Class::MAGE || cls == Class::WARLOCK || cls == Class::DEATH_KNIGHT;
        case Race::TAUREN:
            return cls == Class::WARRIOR || cls == Class::HUNTER || cls == Class::DRUID ||
                   cls == Class::SHAMAN || cls == Class::DEATH_KNIGHT;
        case Race::GNOME:
            return cls == Class::WARRIOR || cls == Class::ROGUE || cls == Class::MAGE ||
                   cls == Class::WARLOCK || cls == Class::DEATH_KNIGHT;
        case Race::TROLL:
            return cls == Class::WARRIOR || cls == Class::HUNTER || cls == Class::ROGUE ||
                   cls == Class::PRIEST || cls == Class::SHAMAN || cls == Class::MAGE ||
                   cls == Class::DEATH_KNIGHT;
        case Race::BLOOD_ELF:
            return cls == Class::PALADIN || cls == Class::HUNTER || cls == Class::ROGUE ||
                   cls == Class::PRIEST || cls == Class::MAGE || cls == Class::WARLOCK ||
                   cls == Class::DEATH_KNIGHT;
        case Race::DRAENEI:
            return cls == Class::WARRIOR || cls == Class::PALADIN || cls == Class::HUNTER ||
                   cls == Class::PRIEST || cls == Class::SHAMAN || cls == Class::MAGE ||
                   cls == Class::DEATH_KNIGHT;
        default:
            return false;
    }
}

uint8_t getMaxSkin(Race /*race*/, Gender /*gender*/) { return 9; }
uint8_t getMaxFace(Race /*race*/, Gender /*gender*/) { return 9; }
uint8_t getMaxHairStyle(Race /*race*/, Gender /*gender*/) { return 11; }
uint8_t getMaxHairColor(Race /*race*/, Gender /*gender*/) { return 9; }
uint8_t getMaxFacialFeature(Race /*race*/, Gender /*gender*/) { return 8; }

const char* getRaceName(Race race) {
    switch (race) {
        case Race::HUMAN:       return "Human";
        case Race::ORC:         return "Orc";
        case Race::DWARF:       return "Dwarf";
        case Race::NIGHT_ELF:   return "Night Elf";
        case Race::UNDEAD:      return "Undead";
        case Race::TAUREN:      return "Tauren";
        case Race::GNOME:       return "Gnome";
        case Race::TROLL:       return "Troll";
        case Race::GOBLIN:      return "Goblin";
        case Race::BLOOD_ELF:   return "Blood Elf";
        case Race::DRAENEI:     return "Draenei";
        default:                return "Unknown";
    }
}

const char* getClassName(Class characterClass) {
    switch (characterClass) {
        case Class::WARRIOR:        return "Warrior";
        case Class::PALADIN:        return "Paladin";
        case Class::HUNTER:         return "Hunter";
        case Class::ROGUE:          return "Rogue";
        case Class::PRIEST:         return "Priest";
        case Class::DEATH_KNIGHT:   return "Death Knight";
        case Class::SHAMAN:         return "Shaman";
        case Class::MAGE:           return "Mage";
        case Class::WARLOCK:        return "Warlock";
        case Class::DRUID:          return "Druid";
        default:                    return "Unknown";
    }
}

const char* getGenderName(Gender gender) {
    switch (gender) {
        case Gender::MALE:      return "Male";
        case Gender::FEMALE:    return "Female";
        default:                return "Unknown";
    }
}

} // namespace game
} // namespace wowee
