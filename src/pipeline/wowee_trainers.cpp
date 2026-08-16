#include "pipeline/wowee_trainers.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'T', 'R', 'N'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wtrn";

} // namespace

const WoweeTrainer::Entry* WoweeTrainer::findByNpc(uint32_t npcId) const {
    for (const auto& e : entries) {
        if (e.npcId == npcId) return &e;
    }
    return nullptr;
}

std::string WoweeTrainer::kindMaskName(uint8_t k) {
    std::string s;
    if (k & Trainer) s += "trainer";
    if (k & Vendor)  { if (!s.empty()) s += "+"; s += "vendor"; }
    if (s.empty())   s = "-";
    return s;
}

bool WoweeTrainerLoader::save(const WoweeTrainer& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeTrainer::Entry& e) {
        writePOD(os, e.npcId);
        writePOD(os, e.kindMask);
        writePadding(os, 3);
        writeStr(os, e.greeting);
        uint16_t spellCount = static_cast<uint16_t>(
            e.spells.size() > 0xFFFF ? 0xFFFF : e.spells.size());
        uint16_t itemCount = static_cast<uint16_t>(
            e.items.size() > 0xFFFF ? 0xFFFF : e.items.size());
        writePOD(os, spellCount);
        writePOD(os, itemCount);
        for (uint16_t k = 0; k < spellCount; ++k) {
            const auto& s = e.spells[k];
            writePOD(os, s.spellId);
            writePOD(os, s.moneyCostCopper);
            writePOD(os, s.requiredSkillId);
            writePOD(os, s.requiredSkillRank);
            writePOD(os, s.requiredLevel);
        }
        for (uint16_t k = 0; k < itemCount; ++k) {
            const auto& it = e.items[k];
            writePOD(os, it.itemId);
            writePOD(os, it.stockCount);
            writePOD(os, it.restockSec);
            writePOD(os, it.extendedCost);
            writePOD(os, it.moneyCostCopper);
        }
                       });
}

WoweeTrainer WoweeTrainerLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeTrainer>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeTrainer::Entry& e) {
        if (!readPOD(is, e.npcId) || !readPOD(is, e.kindMask)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        if (!readStr(is, e.greeting)) { return false; }
        uint16_t spellCount = 0, itemCount = 0;
        if (!readPOD(is, spellCount) || !readPOD(is, itemCount)) { return false; }
        e.spells.resize(spellCount);
        for (uint16_t k = 0; k < spellCount; ++k) {
            auto& s = e.spells[k];
            if (!readPOD(is, s.spellId) ||
                !readPOD(is, s.moneyCostCopper) ||
                !readPOD(is, s.requiredSkillId) ||
                !readPOD(is, s.requiredSkillRank) ||
                !readPOD(is, s.requiredLevel)) { return false; }
        }
        e.items.resize(itemCount);
        for (uint16_t k = 0; k < itemCount; ++k) {
            auto& it = e.items[k];
            if (!readPOD(is, it.itemId) ||
                !readPOD(is, it.stockCount) ||
                !readPOD(is, it.restockSec) ||
                !readPOD(is, it.extendedCost) ||
                !readPOD(is, it.moneyCostCopper)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeTrainerLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeTrainer WoweeTrainerLoader::makeStarter(const std::string& catalogName) {
    WoweeTrainer c;
    c.name = catalogName;
    {
        // npcId 4001 matches WCRT.makeStarter / makeMerchants
        // (Bartleby innkeeper).
        WoweeTrainer::Entry e;
        e.npcId = 4001;
        e.kindMask = WoweeTrainer::Trainer | WoweeTrainer::Vendor;
        e.greeting = "Welcome to the inn, traveler. What can I do for you?";
        // Train First Aid (skillId 129 in WSKL.makeProfessions).
        e.spells.push_back({.spellId = 4001, .moneyCostCopper = 100, .requiredSkillId = 129, .requiredSkillRank = 1, .requiredLevel = 1});  // teaches First Aid
        // Sell starter items (itemIds match WIT.makeStarter:
        // 2=Linen Vest, 3=Healing Potion). Use moneyCost=0 to
        // mean "use WIT.buyPrice".
        e.items.push_back({.itemId = 2, .stockCount = WoweeTrainer::kUnlimitedStock, .restockSec = 0, .extendedCost = 0, .moneyCostCopper = 0});
        e.items.push_back({.itemId = 3, .stockCount = WoweeTrainer::kUnlimitedStock, .restockSec = 0, .extendedCost = 0, .moneyCostCopper = 0});
        e.items.push_back({.itemId = 4, .stockCount = 1, .restockSec = 86400, .extendedCost = 0, .moneyCostCopper = 0});  // 1 unique item / 24h
        c.entries.push_back(e);
    }
    return c;
}

WoweeTrainer WoweeTrainerLoader::makeMageTrainer(const std::string& catalogName) {
    WoweeTrainer c;
    c.name = catalogName;
    {
        // npcId 4003 = alchemist NPC repurposed as a mage
        // trainer for the demo. Spell IDs match WSPL.makeMage.
        WoweeTrainer::Entry e;
        e.npcId = 4003;
        e.kindMask = WoweeTrainer::Trainer;
        e.greeting = "Magic is a craft. Will you learn?";
        // Each spell costs scaling copper, requires reagent
        // skill (none here), and a minimum character level.
        e.spells.push_back({.spellId = 116,  .moneyCostCopper = 100,    .requiredSkillId = 0, .requiredSkillRank = 0,  .requiredLevel = 4});   // Frostbolt @ lvl 4
        e.spells.push_back({.spellId = 133,  .moneyCostCopper = 100,    .requiredSkillId = 0, .requiredSkillRank = 0,  .requiredLevel = 1});   // Fireball @ lvl 1
        e.spells.push_back({.spellId = 1459, .moneyCostCopper = 1000,   .requiredSkillId = 0, .requiredSkillRank = 0,  .requiredLevel = 10});  // Arcane Int @ lvl 10
        e.spells.push_back({.spellId = 1953, .moneyCostCopper = 5000,   .requiredSkillId = 0, .requiredSkillRank = 0,  .requiredLevel = 20});  // Blink @ lvl 20
        c.entries.push_back(e);
    }
    return c;
}

WoweeTrainer WoweeTrainerLoader::makeWeaponVendor(const std::string& catalogName) {
    WoweeTrainer c;
    c.name = catalogName;
    {
        // npcId 4002 = smith from WCRT.makeMerchants. Sells
        // weapons matching WIT.makeWeapons itemIds.
        WoweeTrainer::Entry e;
        e.npcId = 4002;
        e.kindMask = WoweeTrainer::Vendor;
        e.greeting = "Strong steel for sturdy folk. Take a look.";
        e.items.push_back({.itemId = 1001, .stockCount = WoweeTrainer::kUnlimitedStock, .restockSec = 0, .extendedCost = 0, .moneyCostCopper = 0});  // Apprentice Sword
        e.items.push_back({.itemId = 1002, .stockCount = WoweeTrainer::kUnlimitedStock, .restockSec = 0, .extendedCost = 0, .moneyCostCopper = 0});  // Journeyman Blade
        e.items.push_back({.itemId = 1003, .stockCount = 3, .restockSec = 3600, .extendedCost = 0, .moneyCostCopper = 0});  // Steelthorn Edge: 3 in stock, refresh 1h
        e.items.push_back({.itemId = 1004, .stockCount = 1, .restockSec = 7200, .extendedCost = 0, .moneyCostCopper = 0});  // Bloodforged: 1 in stock, refresh 2h
        e.items.push_back({.itemId = 1005, .stockCount = 0, .restockSec = 0, .extendedCost = 0, .moneyCostCopper = 0});      // Doombringer: out of stock by default
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
