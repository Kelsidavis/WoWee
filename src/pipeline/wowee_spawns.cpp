#include "pipeline/wowee_spawns.hpp"
#include "core/coordinates.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'S', 'P', 'N'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wspn";

} // namespace

uint32_t WoweeSpawns::countByKind(uint8_t k) const {
    uint32_t n = 0;
    for (const auto& e : entries) if (e.kind == k) ++n;
    return n;
}

const char* WoweeSpawns::kindName(uint8_t k) {
    switch (k) {
        case Creature:   return "creature";
        case GameObject: return "object";
        case Doodad:     return "doodad";
        default:         return "unknown";
    }
}

bool WoweeSpawnsLoader::save(const WoweeSpawns& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeSpawns::Entry& e) {
        writePOD(os, e.kind);
        writePadding(os, 3);
        writePOD(os, e.entryId);
        writePOD(os, e.position.x);
        writePOD(os, e.position.y);
        writePOD(os, e.position.z);
        writePOD(os, e.rotation.x);
        writePOD(os, e.rotation.y);
        writePOD(os, e.rotation.z);
        writePOD(os, e.scale);
        writePOD(os, e.flags);
        writePOD(os, e.respawnSec);
        writePOD(os, e.factionId);
        writePOD(os, e.questIdRequired);
        writePOD(os, e.wanderRadius);
        writeStr(os, e.label);
                       });
}

WoweeSpawns WoweeSpawnsLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeSpawns>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeSpawns::Entry& e) {
        if (!readPOD(is, e.kind)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        if (!readPOD(is, e.entryId)) { return false; }
        if (!readPOD(is, e.position.x) ||
            !readPOD(is, e.position.y) ||
            !readPOD(is, e.position.z) ||
            !readPOD(is, e.rotation.x) ||
            !readPOD(is, e.rotation.y) ||
            !readPOD(is, e.rotation.z) ||
            !readPOD(is, e.scale) ||
            !readPOD(is, e.flags) ||
            !readPOD(is, e.respawnSec) ||
            !readPOD(is, e.factionId) ||
            !readPOD(is, e.questIdRequired) ||
            !readPOD(is, e.wanderRadius)) { return false; }
        if (!readStr(is, e.label)) { return false; }
                                  return true;
                              });
}

bool WoweeSpawnsLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeSpawns WoweeSpawnsLoader::makeStarter(const std::string& catalogName) {
    WoweeSpawns c;
    c.name = catalogName;
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Creature;
        e.entryId = 1; e.position = {0, 0, 0};
        e.factionId = 35; e.respawnSec = 300; e.wanderRadius = 5.0f;
        e.label = "starter creature";
        c.entries.push_back(e);
    }
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::GameObject;
        e.entryId = 2; e.position = {3, 0, 0};
        e.respawnSec = 600;
        e.wanderRadius = 0.0f;        // game objects don't wander
        e.label = "starter chest";
        c.entries.push_back(e);
    }
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Doodad;
        e.entryId = 100; e.position = {-3, 0, 0};
        e.respawnSec = 0;
        e.wanderRadius = 0.0f;        // doodads don't wander
        e.label = "starter tree";
        c.entries.push_back(e);
    }
    return c;
}

WoweeSpawns WoweeSpawnsLoader::makeCamp(const std::string& catalogName) {
    WoweeSpawns c;
    c.name = catalogName;
    // 4 bandits spaced around a wander ring, all sharing
    // the same template entry id and the same wander radius.
    const float ringR = 4.0f;
    for (int k = 0; k < 4; ++k) {
        float a = (k / 4.0f) * 6.2831853f;
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Creature;
        e.entryId = 1000;
        e.position = {std::cos(a) * ringR, 0.0f, std::sin(a) * ringR};
        e.rotation = {0.0f, a + core::coords::PI, 0.0f};   // facing inward
        e.factionId = 14;       // hostile
        e.respawnSec = 240;
        e.wanderRadius = 3.0f;
        e.label = std::string("bandit ") + std::to_string(k + 1);
        c.entries.push_back(e);
    }
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::GameObject;
        e.entryId = 2000;
        e.position = {0, 0, 0};
        e.respawnSec = 1800;
        e.wanderRadius = 0.0f;
        e.label = "bandit chest";
        c.entries.push_back(e);
    }
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Doodad;
        e.entryId = 3000;
        e.position = {2.0f, 0.0f, 2.0f};
        e.respawnSec = 0;
        e.wanderRadius = 0.0f;
        e.label = "tent A";
        c.entries.push_back(e);
    }
    {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Doodad;
        e.entryId = 3000;
        e.position = {-2.0f, 0.0f, -2.0f};
        e.respawnSec = 0;
        e.wanderRadius = 0.0f;
        e.label = "tent B";
        c.entries.push_back(e);
    }
    return c;
}

WoweeSpawns WoweeSpawnsLoader::makeVillage(const std::string& catalogName) {
    WoweeSpawns c;
    c.name = catalogName;
    // 6 friendly NPCs (different roles) spread over a ~30 m
    // square plus 2 signpost game objects + 4 tree doodads.
    struct Npc { float x; float z; uint32_t id; const char* label; };
    // NOLINTBEGIN(modernize-use-designated-initializers) - a table whose
    // columns are its field names, with the struct in view directly above.
    Npc npcs[6] = {
        {  0.0f,   0.0f, 4001, "innkeeper" },
        { 12.0f,  -5.0f, 4002, "smith" },
        {-10.0f,   8.0f, 4003, "alchemist" },
        {  6.0f,  10.0f, 4004, "scribe" },
        { -8.0f,  -7.0f, 4005, "guard captain" },
        { 15.0f,   3.0f, 4006, "stable master" },
    };
    // NOLINTEND(modernize-use-designated-initializers)
    for (const auto& n : npcs) {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Creature;
        e.entryId = n.id;
        e.position = {n.x, 0.0f, n.z};
        e.factionId = 35;       // friendly
        e.respawnSec = 600;
        e.wanderRadius = 1.0f;
        e.label = n.label;
        c.entries.push_back(e);
    }
    for (int k = 0; k < 2; ++k) {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::GameObject;
        e.entryId = 5000 + k;
        e.position = {k == 0 ? -15.0f : 15.0f, 0.0f, 0.0f};
        e.respawnSec = 0;
        e.wanderRadius = 0.0f;
        e.label = (k == 0 ? "north sign" : "south sign");
        c.entries.push_back(e);
    }
    struct Tree { float x; float z; };
    // NOLINTBEGIN(modernize-use-designated-initializers) - a table whose
    // columns are its field names, with the struct in view directly above.
    Tree trees[4] = {
        { -18.0f, -12.0f }, { 18.0f, -12.0f },
        { -18.0f,  12.0f }, { 18.0f,  12.0f },
    };
    // NOLINTEND(modernize-use-designated-initializers)
    for (const auto& t : trees) {
        WoweeSpawns::Entry e;
        e.kind = WoweeSpawns::Doodad;
        e.entryId = 6000;
        e.position = {t.x, 0.0f, t.z};
        e.respawnSec = 0;
        e.wanderRadius = 0.0f;
        e.label = "village tree";
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
