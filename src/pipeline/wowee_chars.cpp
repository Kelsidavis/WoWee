#include "pipeline/wowee_chars.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'C', 'H', 'C'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wchc";

} // namespace

const WoweeChars::Class* WoweeChars::findClass(uint32_t classId) const {
    for (const auto& c : classes) if (c.classId == classId) return &c;
    return nullptr;
}

const WoweeChars::Race* WoweeChars::findRace(uint32_t raceId) const {
    for (const auto& r : races) if (r.raceId == raceId) return &r;
    return nullptr;
}

const WoweeChars::Outfit* WoweeChars::findOutfit(uint32_t classId,
                                                  uint32_t raceId,
                                                  uint8_t gender) const {
    for (const auto& o : outfits) {
        if (o.classId == classId && o.raceId == raceId &&
            o.gender == gender) return &o;
    }
    return nullptr;
}

const char* WoweeChars::powerTypeName(uint8_t p) {
    switch (p) {
        case Mana:       return "mana";
        case Rage:       return "rage";
        case Focus:      return "focus";
        case Energy:     return "energy";
        case RunicPower: return "runic-power";
        case Runes:      return "runes";
        default:         return "unknown";
    }
}

const char* WoweeChars::raceFactionName(uint8_t f) {
    switch (f) {
        case Alliance: return "alliance";
        case Horde:    return "horde";
        case Neutral:  return "neutral";
        default:       return "unknown";
    }
}

bool WoweeCharsLoader::save(const WoweeChars& cat,
                            const std::string& basePath) {
    std::ofstream os(normalizePath(basePath, kExtension), std::ios::binary);
    if (!os) return false;
    const uint32_t classCount = static_cast<uint32_t>(cat.classes.size());
    writeCatalogHeader(os, kMagic, kVersion, cat.name, classCount);
    for (const auto& c : cat.classes) {
        writePOD(os, c.classId);
        writeStr(os, c.name);
        writeStr(os, c.iconPath);
        writePOD(os, c.powerType);
        writePOD(os, c.displayPower);
        writePOD(os, c.factionAvailability);
        writePadding(os, 1);
        writePOD(os, c.baseHealth);
        writePOD(os, c.baseHealthPerLevel);
        writePOD(os, c.basePower);
        writePOD(os, c.basePowerPerLevel);
    }
    uint32_t raceCount = static_cast<uint32_t>(cat.races.size());
    writePOD(os, raceCount);
    for (const auto& r : cat.races) {
        writePOD(os, r.raceId);
        writeStr(os, r.name);
        writeStr(os, r.iconPath);
        writePOD(os, r.factionId);
        writePadding(os, 3);
        writePOD(os, r.maleDisplayId);
        writePOD(os, r.femaleDisplayId);
        writePOD(os, r.baseStrength);
        writePOD(os, r.baseAgility);
        writePOD(os, r.baseStamina);
        writePOD(os, r.baseIntellect);
        writePOD(os, r.baseSpirit);
        writePadding(os, 2);
        writePOD(os, r.startingMapId);
        writePOD(os, r.startingZoneAreaId);
        writePOD(os, r.defaultLanguageSpellId);
        writePOD(os, r.mountSpellId);
    }
    uint32_t outfitCount = static_cast<uint32_t>(cat.outfits.size());
    writePOD(os, outfitCount);
    for (const auto& o : cat.outfits) {
        writePOD(os, o.classId);
        writePOD(os, o.raceId);
        writePOD(os, o.gender);
        uint8_t itemCount = static_cast<uint8_t>(
            o.items.size() > 255 ? 255 : o.items.size());
        writePOD(os, itemCount);
        writePadding(os, 2);
        for (uint8_t k = 0; k < itemCount; ++k) {
            const auto& it = o.items[k];
            writePOD(os, it.itemId);
            writePOD(os, it.displaySlot);
            uint8_t ipad[3] = {0, 0, 0};
            os.write(reinterpret_cast<const char*>(ipad), 3);
        }
    }
    return os.good();
}

WoweeChars WoweeCharsLoader::load(const std::string& basePath) {
    WoweeChars out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    uint32_t classCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, classCount)) return out;
    out.classes.resize(classCount);
    for (auto& c : out.classes) {
        if (!readPOD(is, c.classId)) { out.classes.clear(); return out; }
        if (!readStr(is, c.name) || !readStr(is, c.iconPath)) {
            out.classes.clear(); return out;
        }
        if (!readPOD(is, c.powerType) ||
            !readPOD(is, c.displayPower) ||
            !readPOD(is, c.factionAvailability)) {
            out.classes.clear(); return out;
        }
        if (!skipPadding(is, 1)) {
            out.classes.clear(); return out;
        }
        if (!readPOD(is, c.baseHealth) ||
            !readPOD(is, c.baseHealthPerLevel) ||
            !readPOD(is, c.basePower) ||
            !readPOD(is, c.basePowerPerLevel)) {
            out.classes.clear(); return out;
        }
    }
    uint32_t raceCount = 0;
    if (!readPOD(is, raceCount)) {
        out.classes.clear(); return out;
    }
    if (raceCount > (1u << 20)) {
        out.classes.clear(); return out;
    }
    out.races.resize(raceCount);
    for (auto& r : out.races) {
        if (!readPOD(is, r.raceId)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!readStr(is, r.name) || !readStr(is, r.iconPath)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!readPOD(is, r.factionId)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!skipPadding(is, 3)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!readPOD(is, r.maleDisplayId) ||
            !readPOD(is, r.femaleDisplayId) ||
            !readPOD(is, r.baseStrength) ||
            !readPOD(is, r.baseAgility) ||
            !readPOD(is, r.baseStamina) ||
            !readPOD(is, r.baseIntellect) ||
            !readPOD(is, r.baseSpirit)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!skipPadding(is, 2)) {
            out.classes.clear(); out.races.clear(); return out;
        }
        if (!readPOD(is, r.startingMapId) ||
            !readPOD(is, r.startingZoneAreaId) ||
            !readPOD(is, r.defaultLanguageSpellId) ||
            !readPOD(is, r.mountSpellId)) {
            out.classes.clear(); out.races.clear(); return out;
        }
    }
    uint32_t outfitCount = 0;
    if (!readPOD(is, outfitCount)) {
        out.classes.clear(); out.races.clear(); return out;
    }
    if (outfitCount > (1u << 20)) {
        out.classes.clear(); out.races.clear(); return out;
    }
    out.outfits.resize(outfitCount);
    for (auto& o : out.outfits) {
        if (!readPOD(is, o.classId) ||
            !readPOD(is, o.raceId) ||
            !readPOD(is, o.gender)) {
            out.classes.clear(); out.races.clear();
            out.outfits.clear(); return out;
        }
        uint8_t itemCount = 0;
        if (!readPOD(is, itemCount)) {
            out.classes.clear(); out.races.clear();
            out.outfits.clear(); return out;
        }
        if (!skipPadding(is, 2)) {
            out.classes.clear(); out.races.clear();
            out.outfits.clear(); return out;
        }
        o.items.resize(itemCount);
        for (uint8_t k = 0; k < itemCount; ++k) {
            auto& it = o.items[k];
            if (!readPOD(is, it.itemId) ||
                !readPOD(is, it.displaySlot)) {
                out.classes.clear(); out.races.clear();
                out.outfits.clear(); return out;
            }
            uint8_t ipad[3];
            is.read(reinterpret_cast<char*>(ipad), 3);
            if (is.gcount() != 3) {
                out.classes.clear(); out.races.clear();
                out.outfits.clear(); return out;
            }
        }
    }
    return out;
}

bool WoweeCharsLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeChars WoweeCharsLoader::makeStarter(const std::string& catalogName) {
    WoweeChars c;
    c.name = catalogName;
    {
        WoweeChars::Class cls;
        cls.classId = 1; cls.name = "Warrior";
        cls.powerType = WoweeChars::Rage; cls.displayPower = WoweeChars::Rage;
        cls.baseHealth = 60; cls.baseHealthPerLevel = 18;
        cls.basePower = 100; cls.basePowerPerLevel = 0;
        c.classes.push_back(cls);
    }
    {
        WoweeChars::Class cls;
        cls.classId = 8; cls.name = "Mage";
        cls.powerType = WoweeChars::Mana; cls.displayPower = WoweeChars::Mana;
        cls.baseHealth = 35; cls.baseHealthPerLevel = 10;
        cls.basePower = 100; cls.basePowerPerLevel = 30;
        c.classes.push_back(cls);
    }
    {
        WoweeChars::Race r;
        r.raceId = 1; r.name = "Human";
        r.factionId = WoweeChars::Alliance;
        r.maleDisplayId = 49; r.femaleDisplayId = 50;
        r.startingMapId = 0; r.startingZoneAreaId = 12;   // Elwynn Forest
        r.defaultLanguageSpellId = 668;                    // Common
        c.races.push_back(r);
    }
    {
        WoweeChars::Race r;
        r.raceId = 2; r.name = "Orc";
        r.factionId = WoweeChars::Horde;
        r.maleDisplayId = 51; r.femaleDisplayId = 52;
        r.startingMapId = 1; r.startingZoneAreaId = 215;   // Mulgore-ish
        r.defaultLanguageSpellId = 669;                    // Orcish
        c.races.push_back(r);
    }
    auto addOutfit = [&](uint32_t classId, uint32_t raceId,
                          std::vector<WoweeChars::OutfitItem> items) {
        WoweeChars::Outfit o;
        o.classId = classId; o.raceId = raceId; o.gender = WoweeChars::Male;
        o.items = std::move(items);
        c.outfits.push_back(o);
    };
    // Each outfit uses WIT itemIds (1 = Worn Shortsword,
    // 2 = Linen Vest, 3 = Healing Potion).
    addOutfit(1, 1, {{.itemId = 1, .displaySlot = 13}, {.itemId = 2, .displaySlot = 5}, {.itemId = 3, .displaySlot = 0}});            // Human Warrior
    addOutfit(1, 2, {{.itemId = 1, .displaySlot = 13}, {.itemId = 2, .displaySlot = 5}, {.itemId = 3, .displaySlot = 0}});            // Orc Warrior
    addOutfit(8, 1, {{.itemId = 2, .displaySlot = 5}, {.itemId = 3, .displaySlot = 0}});                      // Human Mage
    addOutfit(8, 2, {{.itemId = 2, .displaySlot = 5}, {.itemId = 3, .displaySlot = 0}});                      // Orc Mage
    return c;
}

WoweeChars WoweeCharsLoader::makeAlliance(const std::string& catalogName) {
    WoweeChars c;
    c.name = catalogName;
    auto addClass = [&](uint32_t id, const char* name, uint8_t power,
                         uint32_t baseHp, uint16_t hpPerLvl,
                         uint32_t basePwr, uint16_t pwrPerLvl) {
        WoweeChars::Class cls;
        cls.classId = id; cls.name = name;
        cls.powerType = power; cls.displayPower = power;
        cls.baseHealth = baseHp; cls.baseHealthPerLevel = hpPerLvl;
        cls.basePower = basePwr; cls.basePowerPerLevel = pwrPerLvl;
        cls.factionAvailability = WoweeChars::AvailableAlliance;
        c.classes.push_back(cls);
    };
    addClass(1, "Warrior", WoweeChars::Rage,   60, 18, 100, 0);
    addClass(2, "Paladin", WoweeChars::Mana,   55, 16, 100, 30);
    addClass(4, "Rogue",   WoweeChars::Energy, 45, 14, 100, 0);
    addClass(8, "Mage",    WoweeChars::Mana,   35, 10, 100, 30);
    auto addRace = [&](uint32_t id, const char* name, uint32_t mapId,
                        uint32_t zoneId, uint32_t langSpell) {
        WoweeChars::Race r;
        r.raceId = id; r.name = name;
        r.factionId = WoweeChars::Alliance;
        r.startingMapId = mapId; r.startingZoneAreaId = zoneId;
        r.defaultLanguageSpellId = langSpell;
        c.races.push_back(r);
    };
    addRace(1, "Human",     0, 12,  668);   // Elwynn / Common
    addRace(3, "Dwarf",     0, 1,   672);   // Dun Morogh / Dwarvish
    addRace(4, "NightElf",  1, 141, 671);   // Teldrassil / Darnassian
    addRace(7, "Gnome",     0, 1,   7340);  // Dun Morogh / Gnomish
    return c;
}

WoweeChars WoweeCharsLoader::makeAllRaces(const std::string& catalogName) {
    WoweeChars c;
    c.name = catalogName;
    auto addClass = [&](uint32_t id, const char* name, uint8_t power) {
        WoweeChars::Class cls;
        cls.classId = id; cls.name = name;
        cls.powerType = power; cls.displayPower = power;
        cls.baseHealth = 50; cls.baseHealthPerLevel = 12;
        cls.basePower = 100; cls.basePowerPerLevel = 5;
        c.classes.push_back(cls);
    };
    addClass(1, "Warrior", WoweeChars::Rage);
    addClass(2, "Paladin", WoweeChars::Mana);
    addClass(3, "Hunter",  WoweeChars::Mana);
    addClass(4, "Rogue",   WoweeChars::Energy);
    addClass(5, "Priest",  WoweeChars::Mana);
    addClass(7, "Shaman",  WoweeChars::Mana);
    addClass(8, "Mage",    WoweeChars::Mana);
    addClass(9, "Warlock", WoweeChars::Mana);
    addClass(11, "Druid",  WoweeChars::Mana);
    auto addRace = [&](uint32_t id, const char* name, uint8_t faction) {
        WoweeChars::Race r;
        r.raceId = id; r.name = name; r.factionId = faction;
        c.races.push_back(r);
    };
    // Alliance.
    addRace(1, "Human",    WoweeChars::Alliance);
    addRace(3, "Dwarf",    WoweeChars::Alliance);
    addRace(4, "NightElf", WoweeChars::Alliance);
    addRace(7, "Gnome",    WoweeChars::Alliance);
    // Horde.
    addRace(2, "Orc",      WoweeChars::Horde);
    addRace(5, "Undead",   WoweeChars::Horde);
    addRace(6, "Tauren",   WoweeChars::Horde);
    addRace(8, "Troll",    WoweeChars::Horde);
    return c;
}

} // namespace pipeline
} // namespace wowee
