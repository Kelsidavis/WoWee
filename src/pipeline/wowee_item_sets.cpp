#include "pipeline/wowee_item_sets.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'S', 'E', 'T'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wset";

} // namespace

const WoweeItemSet::Entry*
WoweeItemSet::findById(uint32_t setId) const {
    for (const auto& e : entries) if (e.setId == setId) return &e;
    return nullptr;
}

bool WoweeItemSetLoader::save(const WoweeItemSet& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeItemSet::Entry& e) {
        writePOD(os, e.setId);
        writeStr(os, e.name);
        writeStr(os, e.description);
        writePOD(os, e.pieceCount);
        writePOD(os, e.bonusCount);
        writePadding(os, 2);
        writePOD(os, e.requiredClassMask);
        writePOD(os, e.requiredSkillId);
        writePOD(os, e.requiredSkillRank);
        for (uint32_t itemId : e.itemIds) {
            writePOD(os, itemId);
        }
        for (uint8_t bonusThreshold : e.bonusThresholds) {
            writePOD(os, bonusThreshold);
        }
        for (uint32_t bonusSpellId : e.bonusSpellIds) {
            writePOD(os, bonusSpellId);
        }
                       });
}

WoweeItemSet WoweeItemSetLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeItemSet>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeItemSet::Entry& e) {
        if (!readPOD(is, e.setId)) { return false; }
        if (!readStr(is, e.name) || !readStr(is, e.description)) { return false; }
        if (!readPOD(is, e.pieceCount) ||
            !readPOD(is, e.bonusCount)) { return false; }
        if (!skipPadding(is, 2)) { return false; }
        if (!readPOD(is, e.requiredClassMask) ||
            !readPOD(is, e.requiredSkillId) ||
            !readPOD(is, e.requiredSkillRank)) { return false; }
        for (uint32_t& itemId : e.itemIds) {
            if (!readPOD(is, itemId)) { return false; }
        }
        for (uint8_t& bonusThreshold : e.bonusThresholds) {
            if (!readPOD(is, bonusThreshold)) { return false; }
        }
        for (uint32_t& bonusSpellId : e.bonusSpellIds) {
            if (!readPOD(is, bonusSpellId)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeItemSetLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeItemSet WoweeItemSetLoader::makeStarter(
    const std::string& catalogName) {
    WoweeItemSet c;
    c.name = catalogName;
    {
        // Battlegear of Wrath - Warrior tier-2 (8 pieces).
        // Real WoW item / spell IDs from the canonical set.
        WoweeItemSet::Entry e;
        e.setId = 1; e.name = "Battlegear of Wrath";
        e.description = "Warrior tier-2 plate set from Blackwing Lair "
                         "and Molten Core.";
        e.pieceCount = 8;
        e.requiredClassMask = WoweeItemSet::kClassWarrior;
        // 16959..16966 are the canonical 8 piece IDs.
        e.itemIds[0] = 16959; e.itemIds[1] = 16960;
        e.itemIds[2] = 16961; e.itemIds[3] = 16962;
        e.itemIds[4] = 16963; e.itemIds[5] = 16964;
        e.itemIds[6] = 16965; e.itemIds[7] = 16966;
        // 3-piece, 5-piece, 8-piece set bonuses.
        e.bonusCount = 3;
        e.bonusThresholds[0] = 3; e.bonusSpellIds[0] = 23687;
        e.bonusThresholds[1] = 5; e.bonusSpellIds[1] = 23689;
        e.bonusThresholds[2] = 8; e.bonusSpellIds[2] = 23690;
        c.entries.push_back(e);
    }
    {
        // Stormrage Raiment - Druid tier-2 (8 pieces).
        WoweeItemSet::Entry e;
        e.setId = 2; e.name = "Stormrage Raiment";
        e.description = "Druid tier-2 leather set - Onyxia / BWL.";
        e.pieceCount = 8;
        e.requiredClassMask = WoweeItemSet::kClassDruid;
        e.itemIds[0] = 16897; e.itemIds[1] = 16898;
        e.itemIds[2] = 16899; e.itemIds[3] = 16900;
        e.itemIds[4] = 16901; e.itemIds[5] = 16902;
        e.itemIds[6] = 16903; e.itemIds[7] = 16904;
        e.bonusCount = 3;
        e.bonusThresholds[0] = 3; e.bonusSpellIds[0] = 23734;
        e.bonusThresholds[1] = 5; e.bonusSpellIds[1] = 23737;
        e.bonusThresholds[2] = 8; e.bonusSpellIds[2] = 23738;
        c.entries.push_back(e);
    }
    return c;
}

WoweeItemSet WoweeItemSetLoader::makeTier(
    const std::string& catalogName) {
    WoweeItemSet c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t classMask,
                    uint32_t baseItemId, uint32_t bonusSpell2,
                    uint32_t bonusSpell4, uint32_t bonusSpell6,
                    const char* desc) {
        WoweeItemSet::Entry e;
        e.setId = id; e.name = name; e.description = desc;
        e.requiredClassMask = classMask;
        e.pieceCount = 8;
        for (size_t k = 0; k < 8; ++k) {
            e.itemIds[k] = baseItemId + static_cast<uint32_t>(k);
        }
        // Standard 2 / 4 / 6-piece progression.
        e.bonusCount = 3;
        e.bonusThresholds[0] = 2; e.bonusSpellIds[0] = bonusSpell2;
        e.bonusThresholds[1] = 4; e.bonusSpellIds[1] = bonusSpell4;
        e.bonusThresholds[2] = 6; e.bonusSpellIds[2] = bonusSpell6;
        c.entries.push_back(e);
    };
    add(100, "Tier1Warrior",  WoweeItemSet::kClassWarrior,
        16451, 26460, 26461, 26462,
        "Warrior tier-1 plate (Molten Core).");
    add(101, "Tier1Mage",     WoweeItemSet::kClassMage,
        16811, 26467, 26468, 26469,
        "Mage tier-1 cloth (Molten Core).");
    add(102, "Tier1Rogue",    WoweeItemSet::kClassRogue,
        16723, 26482, 26483, 26484,
        "Rogue tier-1 leather (Molten Core).");
    add(103, "Tier1Paladin",  WoweeItemSet::kClassPaladin,
        16927, 26471, 26472, 26473,
        "Paladin tier-1 holy plate (Molten Core).");
    return c;
}

WoweeItemSet WoweeItemSetLoader::makePvP(
    const std::string& catalogName) {
    WoweeItemSet c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t classMask,
                    uint16_t skillId, uint16_t skillRank,
                    uint32_t baseItemId, uint32_t bonus2,
                    uint32_t bonus4, const char* desc) {
        WoweeItemSet::Entry e;
        e.setId = id; e.name = name; e.description = desc;
        e.requiredClassMask = classMask;
        e.requiredSkillId = skillId;
        e.requiredSkillRank = skillRank;
        // PvP sets typically have 5 pieces (head/shoulder/chest
        // /legs/gloves) - leave slots 5-7 empty.
        e.pieceCount = 5;
        for (size_t k = 0; k < 5; ++k) {
            e.itemIds[k] = baseItemId + static_cast<uint32_t>(k);
        }
        // 2-piece + 4-piece bonuses.
        e.bonusCount = 2;
        e.bonusThresholds[0] = 2; e.bonusSpellIds[0] = bonus2;
        e.bonusThresholds[1] = 4; e.bonusSpellIds[1] = bonus4;
        c.entries.push_back(e);
    };
    // skillId 162 = Unarmed; here we use a hypothetical
    // PvP-ranking skill check (1850 honor / 5000 honor).
    // requiredSkillRank values represent honor thresholds.
    add(200, "GladiatorVindication",
        WoweeItemSet::kClassPlate, 599, 1850, 41868, 35114, 35116,
        "Gladiator's plate set - requires 1850 honor rank.");
    add(201, "Doomcaller",
        WoweeItemSet::kClassMage,  599, 1850, 41888, 35124, 35126,
        "Mage doomcaller PvP set - requires 1850 honor rank.");
    add(202, "Predatory",
        WoweeItemSet::kClassRogue, 599, 1500, 41878, 35134, 35136,
        "Rogue predatory PvP set - requires 1500 honor rank.");
    return c;
}

} // namespace pipeline
} // namespace wowee
