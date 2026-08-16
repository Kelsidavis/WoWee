#include "pipeline/wowee_loot.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'L', 'O', 'T'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wlot";

} // namespace

const WoweeLoot::Entry* WoweeLoot::findByCreatureId(uint32_t creatureId) const {
    for (const auto& e : entries) {
        if (e.creatureId == creatureId) return &e;
    }
    return nullptr;
}

bool WoweeLootLoader::save(const WoweeLoot& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeLoot::Entry& e) {
        writePOD(os, e.creatureId);
        writePOD(os, e.flags);
        writePOD(os, e.dropCount);
        writePadding(os, 3);
        writePOD(os, e.moneyMinCopper);
        writePOD(os, e.moneyMaxCopper);
        uint32_t dropN = static_cast<uint32_t>(e.itemDrops.size());
        writePOD(os, dropN);
        for (const auto& d : e.itemDrops) {
            writePOD(os, d.itemId);
            writePOD(os, d.chancePercent);
            writePOD(os, d.minQty);
            writePOD(os, d.maxQty);
            writePOD(os, d.flags);
            uint8_t dpad = 0;
            writePOD(os, dpad);
        }
                       });
}

WoweeLoot WoweeLootLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeLoot>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeLoot::Entry& e) {
        if (!readPOD(is, e.creatureId) ||
            !readPOD(is, e.flags) ||
            !readPOD(is, e.dropCount)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        if (!readPOD(is, e.moneyMinCopper) ||
            !readPOD(is, e.moneyMaxCopper)) { return false; }
        uint32_t dropN = 0;
        if (!readPOD(is, dropN)) { return false; }
        if (dropN > (1u << 16)) { return false; }
        e.itemDrops.resize(dropN);
        for (auto& d : e.itemDrops) {
            if (!readPOD(is, d.itemId) ||
                !readPOD(is, d.chancePercent) ||
                !readPOD(is, d.minQty) ||
                !readPOD(is, d.maxQty) ||
                !readPOD(is, d.flags)) { return false; }
            uint8_t dpad = 0;
            if (!readPOD(is, dpad)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeLootLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeLoot WoweeLootLoader::makeStarter(const std::string& catalogName) {
    WoweeLoot c;
    c.name = catalogName;
    {
        WoweeLoot::Entry e;
        e.creatureId = 1;
        e.dropCount = 1;
        e.moneyMinCopper = 0; e.moneyMaxCopper = 50;
        e.itemDrops.push_back({.itemId = 3, .chancePercent = 50.0f, .minQty = 1, .maxQty = 1, .flags = 0});  // healing potion 50%
        c.entries.push_back(e);
    }
    return c;
}

WoweeLoot WoweeLootLoader::makeBandit(const std::string& catalogName) {
    WoweeLoot c;
    c.name = catalogName;
    {
        WoweeLoot::Entry e;
        e.creatureId = 1000;     // matches the camp spawns from WSPN
        e.dropCount = 2;
        e.moneyMinCopper = 5; e.moneyMaxCopper = 50;
        // Each item is rolled independently against its
        // chancePercent; the dropCount=2 means up to 2
        // distinct items per kill (the runtime is responsible
        // for pickin which 2 to roll first).
        e.itemDrops.push_back({.itemId = 2,    .chancePercent = 35.0f, .minQty = 1, .maxQty = 1, .flags = 0});  // linen vest @ 35%
        e.itemDrops.push_back({.itemId = 101,  .chancePercent = 25.0f, .minQty = 1, .maxQty = 3, .flags = 0});  // bolt of cloth @ 25%
        e.itemDrops.push_back({.itemId = 1001, .chancePercent = 10.0f, .minQty = 1, .maxQty = 1, .flags = 0});  // apprentice sword @ 10%
        e.itemDrops.push_back({.itemId = 102,  .chancePercent = 60.0f, .minQty = 1, .maxQty = 1, .flags = 0});  // ale flask @ 60%
        c.entries.push_back(e);
    }
    return c;
}

WoweeLoot WoweeLootLoader::makeBoss(const std::string& catalogName) {
    WoweeLoot c;
    c.name = catalogName;
    {
        WoweeLoot::Entry e;
        e.creatureId = 9999;
        e.flags = 0;
        e.dropCount = 4;
        // Boss money: 50..200 silver = 5000..20000 copper.
        e.moneyMinCopper = 5000;
        e.moneyMaxCopper = 20000;
        // Guaranteed quest item.
        e.itemDrops.push_back({.itemId = 4, .chancePercent = 100.0f, .minQty = 1, .maxQty = 1,
            .flags = WoweeLoot::QuestRequired | WoweeLoot::AlwaysDrop});
        // Common drops.
        e.itemDrops.push_back({.itemId = 2,    .chancePercent = 80.0f, .minQty = 1, .maxQty = 1, .flags = 0});   // chest
        e.itemDrops.push_back({.itemId = 1002, .chancePercent = 40.0f, .minQty = 1, .maxQty = 1, .flags = 0});   // journeyman blade
        e.itemDrops.push_back({.itemId = 2002, .chancePercent = 30.0f, .minQty = 1, .maxQty = 1, .flags = 0});   // iron chest
        // Group-only epic drop (low chance).
        e.itemDrops.push_back({.itemId = 1004,  .chancePercent = 5.0f, .minQty = 1, .maxQty = 1,
            .flags = WoweeLoot::GroupRollOnly});                  // bloodforged
        // Mass-loot trade goods.
        e.itemDrops.push_back({.itemId = 101,  .chancePercent = 90.0f, .minQty = 2, .maxQty = 5, .flags = 0});   // bolt of cloth
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
