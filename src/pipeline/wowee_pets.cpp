#include "pipeline/wowee_pets.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'P', 'E', 'T'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wpet";

} // namespace

const WoweePet::Family* WoweePet::findFamily(uint32_t familyId) const {
    for (const auto& f : families) if (f.familyId == familyId) return &f;
    return nullptr;
}

const WoweePet::Minion* WoweePet::findMinion(uint32_t minionId) const {
    for (const auto& m : minions) if (m.minionId == minionId) return &m;
    return nullptr;
}

const char* WoweePet::petTypeName(uint8_t t) {
    switch (t) {
        case Cunning:  return "cunning";
        case Ferocity: return "ferocity";
        case Tenacity: return "tenacity";
        default:       return "unknown";
    }
}

std::string WoweePet::dietMaskName(uint32_t mask) {
    std::string s;
    if (mask & DietMeat)   s += "meat+";
    if (mask & DietFish)   s += "fish+";
    if (mask & DietBread)  s += "bread+";
    if (mask & DietCheese) s += "cheese+";
    if (mask & DietFruit)  s += "fruit+";
    if (mask & DietFungus) s += "fungus+";
    if (s.empty()) return "-";
    s.pop_back();   // drop trailing '+'
    return s;
}

bool WoweePetLoader::save(const WoweePet& cat,
                          const std::string& basePath) {
    std::ofstream os(normalizePath(basePath, kExtension), std::ios::binary);
    if (!os) return false;
    const uint32_t famCount = static_cast<uint32_t>(cat.families.size());
    writeCatalogHeader(os, kMagic, kVersion, cat.name, famCount);
    for (const auto& f : cat.families) {
        writePOD(os, f.familyId);
        writeStr(os, f.name);
        writeStr(os, f.description);
        writeStr(os, f.iconPath);
        writePOD(os, f.petType);
        writePadding(os, 3);
        writePOD(os, f.baseAttackSpeed);
        writePOD(os, f.damageMultiplier);
        writePOD(os, f.armorMultiplier);
        writePOD(os, f.dietMask);
        uint8_t abCount = static_cast<uint8_t>(
            f.abilities.size() > 255 ? 255 : f.abilities.size());
        writePOD(os, abCount);
        writePadding(os, 3);
        for (uint8_t k = 0; k < abCount; ++k) {
            const auto& a = f.abilities[k];
            writePOD(os, a.spellId);
            writePOD(os, a.learnedAtLevel);
            writePOD(os, a.rank);
            writePadding(os, 1);
        }
    }
    uint32_t minCount = static_cast<uint32_t>(cat.minions.size());
    writePOD(os, minCount);
    for (const auto& m : cat.minions) {
        writePOD(os, m.minionId);
        writeStr(os, m.name);
        writePOD(os, m.summonSpellId);
        writePOD(os, m.creatureId);
        uint8_t abCount = static_cast<uint8_t>(
            m.abilities.size() > 255 ? 255 : m.abilities.size());
        writePOD(os, abCount);
        writePadding(os, 3);
        for (uint8_t k = 0; k < abCount; ++k) {
            const auto& a = m.abilities[k];
            writePOD(os, a.spellId);
            writePOD(os, a.rank);
            writePOD(os, a.autocastDefault);
            writePadding(os, 2);
        }
    }
    return os.good();
}

WoweePet WoweePetLoader::load(const std::string& basePath) {
    WoweePet out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    auto fail = [&]() {
        out.families.clear(); out.minions.clear();
        return out;
    };
    uint32_t famCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, famCount)) return out;
    out.families.resize(famCount);
    for (auto& f : out.families) {
        if (!readPOD(is, f.familyId)) return fail();
        if (!readStr(is, f.name) || !readStr(is, f.description) ||
            !readStr(is, f.iconPath)) return fail();
        if (!readPOD(is, f.petType)) return fail();
        if (!skipPadding(is, 3)) return fail();
        if (!readPOD(is, f.baseAttackSpeed) ||
            !readPOD(is, f.damageMultiplier) ||
            !readPOD(is, f.armorMultiplier) ||
            !readPOD(is, f.dietMask)) return fail();
        uint8_t abCount = 0;
        if (!readPOD(is, abCount)) return fail();
        if (!skipPadding(is, 3)) return fail();
        f.abilities.resize(abCount);
        for (uint8_t k = 0; k < abCount; ++k) {
            auto& a = f.abilities[k];
            if (!readPOD(is, a.spellId) ||
                !readPOD(is, a.learnedAtLevel) ||
                !readPOD(is, a.rank)) return fail();
            if (!skipPadding(is, 1)) return fail();
        }
    }
    uint32_t minCount = 0;
    if (!readPOD(is, minCount)) return fail();
    if (minCount > (1u << 20)) return fail();
    out.minions.resize(minCount);
    for (auto& m : out.minions) {
        if (!readPOD(is, m.minionId)) return fail();
        if (!readStr(is, m.name)) return fail();
        if (!readPOD(is, m.summonSpellId) ||
            !readPOD(is, m.creatureId)) return fail();
        uint8_t abCount = 0;
        if (!readPOD(is, abCount)) return fail();
        if (!skipPadding(is, 3)) return fail();
        m.abilities.resize(abCount);
        for (uint8_t k = 0; k < abCount; ++k) {
            auto& a = m.abilities[k];
            if (!readPOD(is, a.spellId) ||
                !readPOD(is, a.rank) ||
                !readPOD(is, a.autocastDefault)) return fail();
            if (!skipPadding(is, 2)) return fail();
        }
    }
    return out;
}

bool WoweePetLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweePet WoweePetLoader::makeStarter(const std::string& catalogName) {
    WoweePet c;
    c.name = catalogName;
    {
        // familyId=1 matches WCRT::FamWolf.
        WoweePet::Family f;
        f.familyId = 1; f.name = "Wolf";
        f.description = "Pack hunter; favors meat.";
        f.petType = WoweePet::Ferocity;
        f.dietMask = WoweePet::DietMeat;
        f.abilities.push_back({.spellId = 27050, .learnedAtLevel = 1,  .rank = 1});   // Bite r1
        f.abilities.push_back({.spellId = 27047, .learnedAtLevel = 16, .rank = 1});   // Furious Howl
        f.abilities.push_back({.spellId = 27049, .learnedAtLevel = 30, .rank = 1});   // Dash
        c.families.push_back(f);
    }
    {
        // familyId=2 matches WCRT::FamCat.
        WoweePet::Family f;
        f.familyId = 2; f.name = "Cat";
        f.description = "Stealthy hunter; favors meat or fish.";
        f.petType = WoweePet::Ferocity;
        f.dietMask = WoweePet::DietMeat | WoweePet::DietFish;
        f.abilities.push_back({.spellId = 27049, .learnedAtLevel = 1,  .rank = 1});   // Claw r1
        f.abilities.push_back({.spellId = 16827, .learnedAtLevel = 12, .rank = 1});   // Prowl
        f.abilities.push_back({.spellId = 26064, .learnedAtLevel = 24, .rank = 1});   // Dash
        c.families.push_back(f);
    }
    {
        WoweePet::Minion m;
        m.minionId = 1; m.name = "Imp";
        m.summonSpellId = 688;        // canonical Summon Imp
        m.creatureId = 416;           // canonical Imp creatureId
        m.abilities.push_back({.spellId = 3110,  .rank = 1, .autocastDefault = 1});    // Firebolt r1
        m.abilities.push_back({.spellId = 7813,  .rank = 1, .autocastDefault = 0});    // Blood Pact (autocast off)
        c.minions.push_back(m);
    }
    return c;
}

WoweePet WoweePetLoader::makeHunter(const std::string& catalogName) {
    WoweePet c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint8_t type,
                    uint32_t diet) {
        WoweePet::Family f;
        f.familyId = id; f.name = name;
        f.petType = type; f.dietMask = diet;
        c.families.push_back(f);
    };
    // familyId values match WCRT::FamilyId enum (1..9).
    add(1, "Wolf",    WoweePet::Ferocity, WoweePet::DietMeat);
    add(2, "Cat",     WoweePet::Ferocity,
        WoweePet::DietMeat | WoweePet::DietFish);
    add(3, "Bear",    WoweePet::Tenacity,
        WoweePet::DietMeat | WoweePet::DietFish | WoweePet::DietFruit);
    add(4, "Boar",    WoweePet::Tenacity,
        WoweePet::DietMeat | WoweePet::DietFungus | WoweePet::DietBread);
    add(5, "Raptor",  WoweePet::Cunning,  WoweePet::DietMeat);
    add(6, "Hyena",   WoweePet::Cunning,  WoweePet::DietMeat);
    add(7, "Spider",  WoweePet::Cunning,
        WoweePet::DietMeat | WoweePet::DietFungus);
    add(9, "Crab",    WoweePet::Tenacity,
        WoweePet::DietMeat | WoweePet::DietFish);
    return c;
}

WoweePet WoweePetLoader::makeWarlock(const std::string& catalogName) {
    WoweePet c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name,
                    uint32_t summonSpell, uint32_t creatureId) {
        WoweePet::Minion m;
        m.minionId = id; m.name = name;
        m.summonSpellId = summonSpell; m.creatureId = creatureId;
        c.minions.push_back(m);
    };
    add(1, "Imp",        688,    416);
    add(2, "Voidwalker", 697,    1860);
    add(3, "Succubus",   712,    1863);
    add(4, "Felhunter",  691,    417);
    add(5, "Felguard",   30146,  17252);
    return c;
}

} // namespace pipeline
} // namespace wowee
