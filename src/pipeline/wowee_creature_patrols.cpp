#include "pipeline/wowee_creature_patrols.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'C', 'M', 'R'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wcmr";

} // namespace

const WoweeCreaturePatrol::Entry*
WoweeCreaturePatrol::findById(uint32_t pathId) const {
    for (const auto& e : entries)
        if (e.pathId == pathId) return &e;
    return nullptr;
}

const WoweeCreaturePatrol::Entry*
WoweeCreaturePatrol::findByCreatureGuid(uint32_t creatureGuid) const {
    for (const auto& e : entries)
        if (e.creatureGuid == creatureGuid) return &e;
    return nullptr;
}

float WoweeCreaturePatrol::pathLengthYards(uint32_t pathId) const {
    const Entry* e = findById(pathId);
    if (!e || e->waypoints.size() < 2) return 0.0f;
    float total = 0.0f;
    for (size_t k = 1; k < e->waypoints.size(); ++k) {
        const auto& a = e->waypoints[k - 1];
        const auto& b = e->waypoints[k];
        float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        total += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Loop kind closes back to first waypoint, so include
    // that closing segment in the length.
    if (e->pathKind == Loop && e->waypoints.size() >= 2) {
        const auto& a = e->waypoints.back();
        const auto& b = e->waypoints.front();
        float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        total += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return total;
}

const char* WoweeCreaturePatrol::pathKindName(uint8_t k) {
    switch (k) {
        case Loop:    return "loop";
        case OneShot: return "one-shot";
        case Reverse: return "reverse";
        case Random:  return "random";
        default:      return "unknown";
    }
}

const char* WoweeCreaturePatrol::moveTypeName(uint8_t m) {
    switch (m) {
        case Walk: return "walk";
        case Run:  return "run";
        case Fly:  return "fly";
        case Swim: return "swim";
        default:   return "unknown";
    }
}

bool WoweeCreaturePatrolLoader::save(const WoweeCreaturePatrol& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeCreaturePatrol::Entry& e) {
        writePOD(os, e.pathId);
        writeStr(os, e.name);
        writeStr(os, e.description);
        writePOD(os, e.creatureGuid);
        writePOD(os, e.pathKind);
        writePOD(os, e.moveType);
        writePOD(os, e.pad0);
        writePOD(os, e.pad1);
        uint32_t wpCount = static_cast<uint32_t>(e.waypoints.size());
        writePOD(os, wpCount);
        for (const auto& w : e.waypoints) {
            writePOD(os, w.x);
            writePOD(os, w.y);
            writePOD(os, w.z);
            writePOD(os, w.delayMs);
        }
        writePOD(os, e.iconColorRGBA);
                       });
}

WoweeCreaturePatrol WoweeCreaturePatrolLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeCreaturePatrol>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeCreaturePatrol::Entry& e) {
        if (!readPOD(is, e.pathId)) { return false; }
        if (!readStr(is, e.name) || !readStr(is, e.description)) { return false; }
        if (!readPOD(is, e.creatureGuid) ||
            !readPOD(is, e.pathKind) ||
            !readPOD(is, e.moveType) ||
            !readPOD(is, e.pad0) ||
            !readPOD(is, e.pad1)) { return false; }
        uint32_t wpCount = 0;
        if (!readPOD(is, wpCount)) { return false; }
        // Cap to keep a corrupted file from allocating
        // gigabytes - 64K waypoints per path is plenty.
        if (wpCount > (1u << 16)) { return false; }
        e.waypoints.resize(wpCount);
        for (auto& w : e.waypoints) {
            if (!readPOD(is, w.x) ||
                !readPOD(is, w.y) ||
                !readPOD(is, w.z) ||
                !readPOD(is, w.delayMs)) { return false; }
        }
        if (!readPOD(is, e.iconColorRGBA)) { return false; }
                                  return true;
                              });
}

bool WoweeCreaturePatrolLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeCreaturePatrol WoweeCreaturePatrolLoader::makePatrol(
    const std::string& catalogName) {
    using P = WoweeCreaturePatrol;
    WoweeCreaturePatrol c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t guid,
                    uint8_t kind, uint8_t move,
                    std::initializer_list<P::Waypoint> wps,
                    uint8_t r, uint8_t g, uint8_t b,
                    const char* desc) {
        P::Entry e;
        e.pathId = id; e.name = name; e.description = desc;
        e.creatureGuid = guid;
        e.pathKind = kind;
        e.moveType = move;
        e.waypoints.assign(wps);
        e.iconColorRGBA = packRgba(r, g, b);
        c.entries.push_back(e);
    };
    // Three small patrols showing each pathKind variant.
    add(1, "GuardLoop4", 100001, P::Loop, P::Walk, {
        { .x = -8910.0f,  .y = -135.0f, .z = 82.0f, .delayMs = 1500 },
        { .x = -8895.0f,  .y = -120.0f, .z = 82.0f, .delayMs = 1500 },
        { .x = -8895.0f,  .y = -150.0f, .z = 82.0f, .delayMs = 1500 },
        { .x = -8910.0f,  .y = -150.0f, .z = 82.0f, .delayMs = 1500 },
    }, 100, 200, 240, "Stormwind guard - 4-point loop with 1.5s dwell at each waypoint.");
    add(2, "RunRouteOneShot6", 100002, P::OneShot, P::Run, {
        { .x = -10000.0f,  .y = 500.0f, .z = 30.0f, .delayMs = 0 },
        {  .x = -9900.0f,  .y = 600.0f, .z = 30.0f, .delayMs = 0 },
        {  .x = -9800.0f,  .y = 700.0f, .z = 30.0f, .delayMs = 0 },
        {  .x = -9700.0f,  .y = 800.0f, .z = 30.0f, .delayMs = 0 },
        {  .x = -9600.0f,  .y = 900.0f, .z = 30.0f, .delayMs = 0 },
        {  .x = -9500.0f, .y = 1000.0f, .z = 30.0f, .delayMs = 0 },
    }, 220, 180, 100, "Westfall harvester - 6-point one-shot run, ends at last waypoint.");
    add(3, "TigerRandom8", 100003, P::Random, P::Walk, {
        { .x = -11000.0f, .y = -2000.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -10800.0f, .y = -2100.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -10600.0f, .y = -2050.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -10500.0f, .y = -2200.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -10700.0f, .y = -2300.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -10900.0f, .y = -2250.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -11100.0f, .y = -2150.0f, .z = 30.0f, .delayMs = 3000 },
        { .x = -11050.0f, .y = -1950.0f, .z = 30.0f, .delayMs = 3000 },
    }, 220, 100, 100, "Stranglethorn tiger - 8-point random patrol, "
        "3s dwell, picks next destination randomly.");
    return c;
}

WoweeCreaturePatrol WoweeCreaturePatrolLoader::makeCity(
    const std::string& catalogName) {
    using P = WoweeCreaturePatrol;
    WoweeCreaturePatrol c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t guid,
                    std::initializer_list<P::Waypoint> wps,
                    const char* desc) {
        P::Entry e;
        e.pathId = id; e.name = name; e.description = desc;
        e.creatureGuid = guid;
        e.pathKind = P::Loop;
        e.moveType = P::Walk;
        e.waypoints.assign(wps);
        e.iconColorRGBA = packRgba(180, 220, 240);   // city blue
        c.entries.push_back(e);
    };
    // Four city guard routes (illustrative coordinates).
    add(100, "StormwindCathedralLoop", 110001, {
        { .x = -8520.0f,  .y = 840.0f, .z = 110.0f, .delayMs = 2000 },
        { .x = -8480.0f,  .y = 860.0f, .z = 110.0f, .delayMs = 2000 },
        { .x = -8460.0f,  .y = 820.0f, .z = 110.0f, .delayMs = 2000 },
        { .x = -8500.0f,  .y = 800.0f, .z = 110.0f, .delayMs = 2000 },
        { .x = -8540.0f,  .y = 820.0f, .z = 110.0f, .delayMs = 2000 },
        { .x = -8540.0f,  .y = 860.0f, .z = 110.0f, .delayMs = 2000 },
    }, "Stormwind cathedral square guard - 6-point perimeter loop.");
    add(101, "OrgrimmarValleyOfStrengthLoop", 110002, {
        {  .x = 1640.0f, .y = -4400.0f, .z = 30.0f, .delayMs = 2000 },
        {  .x = 1680.0f, .y = -4380.0f, .z = 30.0f, .delayMs = 2000 },
        {  .x = 1700.0f, .y = -4420.0f, .z = 30.0f, .delayMs = 2000 },
        {  .x = 1680.0f, .y = -4460.0f, .z = 30.0f, .delayMs = 2000 },
        {  .x = 1640.0f, .y = -4480.0f, .z = 30.0f, .delayMs = 2000 },
        {  .x = 1620.0f, .y = -4440.0f, .z = 30.0f, .delayMs = 2000 },
    }, "Orgrimmar Valley of Strength grunt - 6-point perimeter loop.");
    add(102, "IronforgeBankLoop", 110003, {
        { .x = -4800.0f, .y = -930.0f, .z = 500.0f, .delayMs = 2500 },
        { .x = -4760.0f, .y = -910.0f, .z = 500.0f, .delayMs = 2500 },
        { .x = -4750.0f, .y = -950.0f, .z = 500.0f, .delayMs = 2500 },
        { .x = -4790.0f, .y = -980.0f, .z = 500.0f, .delayMs = 2500 },
        { .x = -4830.0f, .y = -960.0f, .z = 500.0f, .delayMs = 2500 },
        { .x = -4830.0f, .y = -920.0f, .z = 500.0f, .delayMs = 2500 },
    }, "Ironforge bank district sentinel - 6-point perimeter loop.");
    add(103, "ThunderBluffElderRiseLoop", 110004, {
        { .x = -1250.0f,  .y = 120.0f, .z = 130.0f, .delayMs = 2000 },
        { .x = -1200.0f,  .y = 140.0f, .z = 130.0f, .delayMs = 2000 },
        { .x = -1180.0f,  .y = 100.0f, .z = 130.0f, .delayMs = 2000 },
        { .x = -1220.0f,   .y = 80.0f, .z = 130.0f, .delayMs = 2000 },
        { .x = -1270.0f,  .y = 100.0f, .z = 130.0f, .delayMs = 2000 },
        { .x = -1280.0f,  .y = 150.0f, .z = 130.0f, .delayMs = 2000 },
    }, "Thunder Bluff Elder Rise warrior - 6-point loop on the upper plateau.");
    return c;
}

WoweeCreaturePatrol WoweeCreaturePatrolLoader::makeBoss(
    const std::string& catalogName) {
    using P = WoweeCreaturePatrol;
    WoweeCreaturePatrol c;
    c.name = catalogName;
    // Three long-path patrols demonstrating that the
    // variable-length design handles bigger patrol sets
    // gracefully. Each helper-build constructs synthetic
    // waypoints around a center using simple maths.
    auto buildCircle = [&](float cx, float cy, float cz,
                            float radius, int n) {
        std::vector<P::Waypoint> out;
        const float pi2 = 6.28318530718f;
        for (int k = 0; k < n; ++k) {
            float a = pi2 * (static_cast<float>(k) /
                              static_cast<float>(n));
            P::Waypoint w;
            w.x = cx + radius * std::cos(a);
            w.y = cy + radius * std::sin(a);
            w.z = cz;
            w.delayMs = 500;
            out.push_back(w);
        }
        return out;
    };
    P::Entry aq40;
    aq40.pathId = 200; aq40.name = "AQ40TrashLoop12";
    aq40.description = "AQ40 chamber trash - 12-point Loop circle, "
        "500ms dwell.";
    aq40.creatureGuid = 200001;
    aq40.pathKind = P::Loop;
    aq40.moveType = P::Walk;
    aq40.waypoints = buildCircle(-8225.0f, 2050.0f, 130.0f, 25.0f, 12);
    aq40.iconColorRGBA = packRgba(220, 180, 100);
    c.entries.push_back(aq40);

    P::Entry naxx;
    naxx.pathId = 201; naxx.name = "NaxxTrashOneShot8";
    naxx.description = "Naxxramas trash - 8-point one-shot ramp run.";
    naxx.creatureGuid = 200002;
    naxx.pathKind = P::OneShot;
    naxx.moveType = P::Run;
    for (int k = 0; k < 8; ++k) {
        P::Waypoint w;
        w.x = 3470.0f + 10.0f * k;
        w.y = -3110.0f + 8.0f * k;
        w.z = 287.0f + 1.5f * k;     // ramp upward
        w.delayMs = 0;
        naxx.waypoints.push_back(w);
    }
    naxx.iconColorRGBA = packRgba(220, 100, 100);
    c.entries.push_back(naxx);

    P::Entry icc;
    icc.pathId = 202; icc.name = "ICCSpirePatrolRandom16";
    icc.description = "Icecrown Citadel spire patrol - 16-point "
        "Random walk over a 60-yard radius.";
    icc.creatureGuid = 200003;
    icc.pathKind = P::Random;
    icc.moveType = P::Walk;
    icc.waypoints = buildCircle(-3500.0f, 2200.0f, 600.0f, 60.0f, 16);
    for (auto& w : icc.waypoints) w.delayMs = 1000;
    icc.iconColorRGBA = packRgba(180, 100, 240);
    c.entries.push_back(icc);

    return c;
}

} // namespace pipeline
} // namespace wowee
