#include "pipeline/wowee_item_suffixes.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'S', 'U', 'F'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wsuf";

} // namespace

const WoweeItemSuffix::Entry*
WoweeItemSuffix::findById(uint32_t suffixId) const {
    for (const auto& e : entries)
        if (e.suffixId == suffixId) return &e;
    return nullptr;
}

const char* WoweeItemSuffix::suffixCategoryName(uint8_t c) {
    switch (c) {
        case Generic:   return "generic";
        case Elemental: return "elemental";
        case Defensive: return "defensive";
        case PvPSuffix: return "pvp";
        case Crafted:   return "crafted";
        default:        return "unknown";
    }
}

bool WoweeItemSuffixLoader::save(const WoweeItemSuffix& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeItemSuffix::Entry& e) {
        writePOD(os, e.suffixId);
        writeStr(os, e.name);
        writeStr(os, e.description);
        writePOD(os, e.itemQualityFloor);
        writePOD(os, e.itemQualityCeiling);
        writePOD(os, e.suffixCategory);
        writePadding(os, 1);
        writePOD(os, e.restrictedSlotMask);
        for (uint8_t statKind : e.statKind) {
            writePOD(os, statKind);
        }
        writePadding(os, 3);
        for (uint16_t statValuePoint : e.statValuePoints) {
            writePOD(os, statValuePoint);
        }
                       });
}

WoweeItemSuffix WoweeItemSuffixLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeItemSuffix>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeItemSuffix::Entry& e) {
        if (!readPOD(is, e.suffixId)) { return false; }
        if (!readStr(is, e.name) || !readStr(is, e.description)) { return false; }
        if (!readPOD(is, e.itemQualityFloor) ||
            !readPOD(is, e.itemQualityCeiling) ||
            !readPOD(is, e.suffixCategory)) { return false; }
        if (!skipPadding(is, 1)) { return false; }
        if (!readPOD(is, e.restrictedSlotMask)) { return false; }
        for (uint8_t& statKind : e.statKind) {
            if (!readPOD(is, statKind)) { return false; }
        }
        if (!skipPadding(is, 3)) { return false; }
        for (uint16_t& statValuePoint : e.statValuePoints) {
            if (!readPOD(is, statValuePoint)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeItemSuffixLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeItemSuffix WoweeItemSuffixLoader::makeStarter(
    const std::string& catalogName) {
    WoweeItemSuffix c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint8_t s1,
                    uint16_t v1, uint8_t s2, uint16_t v2,
                    const char* desc) {
        WoweeItemSuffix::Entry e;
        e.suffixId = id; e.name = name; e.description = desc;
        e.itemQualityFloor = 2;     // green minimum
        e.itemQualityCeiling = 3;   // blue maximum
        e.suffixCategory = WoweeItemSuffix::Generic;
        e.statKind[0] = s1; e.statValuePoints[0] = v1;
        e.statKind[1] = s2; e.statValuePoints[1] = v2;
        c.entries.push_back(e);
    };
    // statKind values match WIT.statType:
    //  4 = STR, 3 = AGI, 5 = INT, 6 = SPI, 7 = STA.
    add(1, "of the Bear",  4, 100, 7,  80,
        "Strength + Stamina - favored by tanks.");
    add(2, "of the Eagle", 5, 100, 6,  60,
        "Intellect + Spirit - favored by casters.");
    add(3, "of the Tiger", 4, 100, 3,  80,
        "Strength + Agility - favored by physical DPS.");
    return c;
}

WoweeItemSuffix WoweeItemSuffixLoader::makeMagical(
    const std::string& catalogName) {
    WoweeItemSuffix c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint8_t schoolStat,
                    uint16_t value, const char* desc) {
        WoweeItemSuffix::Entry e;
        e.suffixId = id; e.name = name; e.description = desc;
        e.itemQualityFloor = 2;
        e.itemQualityCeiling = 4;
        e.suffixCategory = WoweeItemSuffix::Elemental;
        // schoolStat values 30..36 represent per-school spell
        // power - Fire=30, Frost=31, Shadow=32, Arcane=33,
        // Holy=34, Nature=35, Healing=36 (engine-internal
        // mapping outside the WIT canonical set).
        e.statKind[0] = schoolStat;
        e.statValuePoints[0] = value;
        e.restrictedSlotMask =
            WoweeItemSuffix::kSlotHead | WoweeItemSuffix::kSlotChest |
            WoweeItemSuffix::kSlotShoulder | WoweeItemSuffix::kSlotLegs |
            WoweeItemSuffix::kSlotHands | WoweeItemSuffix::kSlotWeapon;
        c.entries.push_back(e);
    };
    add(100, "of Fire",   30, 150, "+ Fire spell power.");
    add(101, "of Frost",  31, 150, "+ Frost spell power.");
    add(102, "of Shadow", 32, 150, "+ Shadow spell power.");
    add(103, "of Arcane", 33, 150, "+ Arcane spell power.");
    return c;
}

WoweeItemSuffix WoweeItemSuffixLoader::makePvP(
    const std::string& catalogName) {
    WoweeItemSuffix c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint8_t s1,
                    uint16_t v1, uint8_t s2, uint16_t v2,
                    uint8_t s3, uint16_t v3, const char* desc) {
        WoweeItemSuffix::Entry e;
        e.suffixId = id; e.name = name; e.description = desc;
        e.itemQualityFloor = 3;     // PvP suffixes only on blues+
        e.itemQualityCeiling = 4;
        e.suffixCategory = WoweeItemSuffix::PvPSuffix;
        // statKind 50 = Resilience (engine-internal, outside
        // the WIT canonical set).
        e.statKind[0] = s1; e.statValuePoints[0] = v1;
        e.statKind[1] = s2; e.statValuePoints[1] = v2;
        e.statKind[2] = s3; e.statValuePoints[2] = v3;
        c.entries.push_back(e);
    };
    add(200, "of the Champion",  50, 80, 4,  60, 7,  60,
        "Resilience + Strength + Stamina - melee PvP.");
    add(201, "of the Gladiator", 50, 80, 5,  60, 6,  40,
        "Resilience + Intellect + Spirit - caster PvP.");
    add(202, "of Resilience",    50, 120, 7, 80,  0,   0,
        "Pure resilience + extra Stamina - peak PvP defense.");
    return c;
}

} // namespace pipeline
} // namespace wowee
