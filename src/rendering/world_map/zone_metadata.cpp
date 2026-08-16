// zone_metadata.cpp - Zone level ranges, faction data, and label formatting.
// Extracted from WorldMap::initZoneMeta (Phase 4 of refactoring plan).
#include "rendering/world_map/zone_metadata.hpp"

namespace wowee {
namespace rendering {
namespace world_map {

void ZoneMetadata::initialize() {
    if (!table_.empty()) return;

    // Populate known zone level ranges and faction alignment.
    // This covers major open-world zones for Vanilla/TBC/WotLK.
    auto add = [this](const char* name, uint8_t lo, uint8_t hi, ZoneFaction f) {
        table_[name] = {.minLevel = lo, .maxLevel = hi, .faction = f};
    };

    // === Eastern Kingdoms ===
    add("Elwynn",           1, 10,  ZoneFaction::Alliance);
    add("DunMorogh",        1, 10,  ZoneFaction::Alliance);
    add("TirisfalGlades",   1, 10,  ZoneFaction::Horde);
    add("Westfall",        10, 20,  ZoneFaction::Alliance);
    add("LochModan",       10, 20,  ZoneFaction::Alliance);
    add("Silverpine",      10, 20,  ZoneFaction::Horde);
    add("Redridge",        15, 25,  ZoneFaction::Contested);
    add("Duskwood",        18, 30,  ZoneFaction::Alliance);
    add("Wetlands",        20, 30,  ZoneFaction::Alliance);
    add("Hillsbrad",       20, 30,  ZoneFaction::Contested);
    add("Alterac",         30, 40,  ZoneFaction::Contested);
    add("Arathi",          30, 40,  ZoneFaction::Contested);
    add("StranglethornVale",30, 45, ZoneFaction::Contested);
    add("Stranglethorn",   30, 45,  ZoneFaction::Contested);
    add("Badlands",        35, 45,  ZoneFaction::Contested);
    add("SwampOfSorrows",  35, 45,  ZoneFaction::Contested);
    add("TheBlastedLands", 45, 55,  ZoneFaction::Contested);
    add("SearingGorge",    43, 50,  ZoneFaction::Contested);
    add("BurningSteppes",  50, 58,  ZoneFaction::Contested);
    add("WesternPlaguelands",51,58, ZoneFaction::Contested);
    add("EasternPlaguelands",53,60, ZoneFaction::Contested);
    add("Hinterlands",     40, 50,  ZoneFaction::Contested);
    add("DeadwindPass",    55, 60,  ZoneFaction::Contested);

    // === Kalimdor ===
    add("Durotar",          1, 10,  ZoneFaction::Horde);
    add("Mulgore",          1, 10,  ZoneFaction::Horde);
    add("Teldrassil",       1, 10,  ZoneFaction::Alliance);
    add("Darkshore",       10, 20,  ZoneFaction::Alliance);
    add("Barrens",         10, 25,  ZoneFaction::Horde);
    add("Ashenvale",       18, 30,  ZoneFaction::Contested);
    add("StonetalonMountains",15,27,ZoneFaction::Contested);
    add("ThousandNeedles", 25, 35,  ZoneFaction::Contested);
    add("Desolace",        30, 40,  ZoneFaction::Contested);
    add("Dustwallow",      35, 45,  ZoneFaction::Contested);
    add("Feralas",         40, 50,  ZoneFaction::Contested);
    add("Tanaris",         40, 50,  ZoneFaction::Contested);
    add("Azshara",         45, 55,  ZoneFaction::Contested);
    add("UngoroCrater",    48, 55,  ZoneFaction::Contested);
    add("Felwood",         48, 55,  ZoneFaction::Contested);
    add("Winterspring",    55, 60,  ZoneFaction::Contested);
    add("Silithus",        55, 60,  ZoneFaction::Contested);
    add("Moonglade",       55, 60,  ZoneFaction::Contested);

    // === TBC: Outland ===
    add("HellFire",        58, 63,  ZoneFaction::Contested);
    add("Zangarmarsh",     60, 64,  ZoneFaction::Contested);
    add("TerokkarForest",  62, 65,  ZoneFaction::Contested);
    add("Nagrand",         64, 67,  ZoneFaction::Contested);
    add("BladesEdgeMountains",65,68,ZoneFaction::Contested);
    add("Netherstorm",     67, 70,  ZoneFaction::Contested);
    add("ShadowmoonValley",67, 70,  ZoneFaction::Contested);

    // === WotLK: Northrend ===
    add("BoreanTundra",    68, 72,  ZoneFaction::Contested);
    add("HowlingFjord",    68, 72,  ZoneFaction::Contested);
    add("Dragonblight",    71, 75,  ZoneFaction::Contested);
    add("GrizzlyHills",    73, 75,  ZoneFaction::Contested);
    add("ZulDrak",         74, 77,  ZoneFaction::Contested);
    add("SholazarBasin",   76, 78,  ZoneFaction::Contested);
    add("StormPeaks",      77, 80,  ZoneFaction::Contested);
    add("Icecrown",        77, 80,  ZoneFaction::Contested);
    add("CrystalsongForest",77,80,  ZoneFaction::Contested);
    add("LakeWintergrasp", 77, 80,  ZoneFaction::Contested);
}

const ZoneMeta* ZoneMetadata::find(const std::string& areaName) const {
    auto it = table_.find(areaName);
    return it != table_.end() ? &it->second : nullptr;
}

std::string ZoneMetadata::formatLabel(const std::string& areaName,
                                       const ZoneMeta* meta) {
    std::string label = areaName;
    if (meta) {
        if (meta->minLevel > 0 && meta->maxLevel > 0) {
            label += " (" + std::to_string(meta->minLevel) + "-" +
                     std::to_string(meta->maxLevel) + ")";
        }
    }
    return label;
}

std::string ZoneMetadata::formatHoverLabel(const std::string& areaName,
                                            const ZoneMeta* meta) {
    std::string label = formatLabel(areaName, meta);
    if (meta) {
        switch (meta->faction) {
            case ZoneFaction::Alliance:  label += " [Alliance]"; break;
            case ZoneFaction::Horde:     label += " [Horde]"; break;
            case ZoneFaction::Contested: label += " [Contested]"; break;
            default: break;
        }
    }
    return label;
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
