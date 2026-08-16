#include "pipeline/wowee_talents.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'T', 'A', 'L'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wtal";

} // namespace

const WoweeTalent::Tree* WoweeTalent::findTree(uint32_t treeId) const {
    for (const auto& t : trees) if (t.treeId == treeId) return &t;
    return nullptr;
}

const WoweeTalent::Talent* WoweeTalent::findTalent(uint32_t talentId) const {
    for (const auto& t : trees) {
        for (const auto& a : t.talents) {
            if (a.talentId == talentId) return &a;
        }
    }
    return nullptr;
}

bool WoweeTalentLoader::save(const WoweeTalent& cat,
                             const std::string& basePath) {
    return saveCatalogEntries(basePath, kMagic, kVersion, kExtension,
                              cat.name, cat.trees,
                              [](std::ofstream& os, const auto& t) {
        writePOD(os, t.treeId);
        writeStr(os, t.name);
        writeStr(os, t.iconPath);
        writePOD(os, t.requiredClassMask);
        uint16_t talentCount = static_cast<uint16_t>(
            t.talents.size() > 0xFFFF ? 0xFFFF : t.talents.size());
        writePOD(os, talentCount);
        writePadding(os, 2);
        for (uint16_t k = 0; k < talentCount; ++k) {
            const auto& a = t.talents[k];
            writePOD(os, a.talentId);
            writePOD(os, a.row);
            writePOD(os, a.col);
            writePOD(os, a.maxRank);
            writePadding(os, 1);
            writePOD(os, a.prereqTalentId);
            writePOD(os, a.prereqRank);
            writePadding(os, 3);
            for (int r = 0; r < WoweeTalent::kMaxRanks; ++r) {
                writePOD(os, a.rankSpellIds[r]);
            }
        }
    });
}

WoweeTalent WoweeTalentLoader::load(const std::string& basePath) {
    WoweeTalent out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    uint32_t treeCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, treeCount)) return out;
    out.trees.resize(treeCount);
    for (auto& t : out.trees) {
        if (!readPOD(is, t.treeId)) { out.trees.clear(); return out; }
        if (!readStr(is, t.name) || !readStr(is, t.iconPath)) {
            out.trees.clear(); return out;
        }
        if (!readPOD(is, t.requiredClassMask)) {
            out.trees.clear(); return out;
        }
        uint16_t talentCount = 0;
        if (!readPOD(is, talentCount)) {
            out.trees.clear(); return out;
        }
        if (!skipPadding(is, 2)) { out.trees.clear(); return out; }
        t.talents.resize(talentCount);
        for (uint16_t k = 0; k < talentCount; ++k) {
            auto& a = t.talents[k];
            if (!readPOD(is, a.talentId) ||
                !readPOD(is, a.row) ||
                !readPOD(is, a.col) ||
                !readPOD(is, a.maxRank)) {
                out.trees.clear(); return out;
            }
            if (!skipPadding(is, 1)) {
                out.trees.clear(); return out;
            }
            if (!readPOD(is, a.prereqTalentId) ||
                !readPOD(is, a.prereqRank)) {
                out.trees.clear(); return out;
            }
            if (!skipPadding(is, 3)) { out.trees.clear(); return out; }
            for (uint32_t& rankSpellId : a.rankSpellIds) {
                if (!readPOD(is, rankSpellId)) {
                    out.trees.clear(); return out;
                }
            }
        }
    }
    return out;
}

bool WoweeTalentLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeTalent WoweeTalentLoader::makeStarter(const std::string& catalogName) {
    WoweeTalent c;
    c.name = catalogName;
    {
        WoweeTalent::Tree t;
        t.treeId = 1; t.name = "Starter Tree";
        t.requiredClassMask = WoweeTalent::ClassWarrior;
        // Tier 0: free entry talent.
        WoweeTalent::Talent a1;
        a1.talentId = 1; a1.row = 0; a1.col = 0; a1.maxRank = 5;
        t.talents.push_back(a1);
        // Tier 1: depends on a1 at rank 5.
        WoweeTalent::Talent a2;
        a2.talentId = 2; a2.row = 1; a2.col = 0; a2.maxRank = 3;
        a2.prereqTalentId = 1; a2.prereqRank = 5;
        t.talents.push_back(a2);
        // Tier 2: capstone, single-rank.
        WoweeTalent::Talent a3;
        a3.talentId = 3; a3.row = 2; a3.col = 0; a3.maxRank = 1;
        a3.prereqTalentId = 2; a3.prereqRank = 3;
        t.talents.push_back(a3);
        c.trees.push_back(t);
    }
    return c;
}

WoweeTalent WoweeTalentLoader::makeWarrior(const std::string& catalogName) {
    WoweeTalent c;
    c.name = catalogName;
    auto addTree = [&](uint32_t id, const char* name) -> WoweeTalent::Tree* {
        WoweeTalent::Tree t;
        t.treeId = id; t.name = name;
        t.requiredClassMask = WoweeTalent::ClassWarrior;
        c.trees.push_back(t);
        return &c.trees.back();
    };
    auto addTalent = [&](WoweeTalent::Tree* t, uint32_t id,
                         uint8_t row, uint8_t col, uint8_t maxRank,
                         uint32_t prereq = 0, uint8_t prereqRank = 0,
                         uint32_t rankSpell0 = 0) {
        WoweeTalent::Talent a;
        a.talentId = id; a.row = row; a.col = col; a.maxRank = maxRank;
        a.prereqTalentId = prereq; a.prereqRank = prereqRank;
        a.rankSpellIds[0] = rankSpell0;
        t->talents.push_back(a);
    };
    {
        // Arms: 4 talents in a vertical chain.
        auto* t = addTree(101, "Arms");
        addTalent(t, 1001, 0, 1, 5);                          // Improved Heroic Strike
        addTalent(t, 1002, 1, 0, 3, 1001, 5);                  // Anger Management
        addTalent(t, 1003, 2, 1, 5, 1002, 3);                  // Two-Handed Spec
        addTalent(t, 1004, 3, 1, 1, 1003, 5, 12294);           // Mortal Strike capstone -> WSPL spellId 12294
    }
    {
        // Fury: 4 talents.
        auto* t = addTree(102, "Fury");
        addTalent(t, 1101, 0, 1, 5);                          // Cruelty
        addTalent(t, 1102, 0, 2, 3);                          // Booming Voice
        addTalent(t, 1103, 1, 1, 5, 1101, 5);                  // Improved Battle Shout -> WSPL 6673
        addTalent(t, 1104, 2, 1, 1, 1103, 5);                  // Bloodthirst capstone
    }
    {
        // Protection: 3 talents.
        auto* t = addTree(103, "Protection");
        addTalent(t, 1201, 0, 0, 5);                          // Improved Bloodrage
        addTalent(t, 1202, 1, 1, 3, 1201, 5);                  // Improved Thunder Clap -> WSPL 6343
        addTalent(t, 1203, 2, 1, 1, 1202, 3);                  // Shield Slam capstone
    }
    return c;
}

WoweeTalent WoweeTalentLoader::makeMage(const std::string& catalogName) {
    WoweeTalent c;
    c.name = catalogName;
    auto addTree = [&](uint32_t id, const char* name) -> WoweeTalent::Tree* {
        WoweeTalent::Tree t;
        t.treeId = id; t.name = name;
        t.requiredClassMask = WoweeTalent::ClassMage;
        c.trees.push_back(t);
        return &c.trees.back();
    };
    auto addTalent = [&](WoweeTalent::Tree* t, uint32_t id,
                         uint8_t row, uint8_t col, uint8_t maxRank,
                         uint32_t prereq = 0, uint8_t prereqRank = 0,
                         uint32_t rankSpell0 = 0) {
        WoweeTalent::Talent a;
        a.talentId = id; a.row = row; a.col = col; a.maxRank = maxRank;
        a.prereqTalentId = prereq; a.prereqRank = prereqRank;
        a.rankSpellIds[0] = rankSpell0;
        t->talents.push_back(a);
    };
    {
        auto* t = addTree(201, "Arcane");
        addTalent(t, 2001, 0, 1, 5);                          // Arcane Focus
        addTalent(t, 2002, 1, 1, 3, 2001, 5);                  // Improved Arcane Intellect -> WSPL 1459
        addTalent(t, 2003, 2, 1, 1, 2002, 3, 1953);            // Improved Blink -> WSPL 1953
    }
    {
        auto* t = addTree(202, "Fire");
        addTalent(t, 2101, 0, 1, 5);                          // Improved Fireball
        addTalent(t, 2102, 1, 1, 5, 2101, 5);                  // Critical Mass
        addTalent(t, 2103, 2, 1, 1, 2102, 5, 133);             // Pyroblast (Fireball ref) -> WSPL 133
    }
    {
        auto* t = addTree(203, "Frost");
        addTalent(t, 2201, 0, 1, 5);                          // Frost Channeling
        addTalent(t, 2202, 1, 1, 5, 2201, 5);                  // Improved Frostbolt
        addTalent(t, 2203, 2, 1, 1, 2202, 5, 116);             // Ice Shards (Frostbolt ref) -> WSPL 116
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
