#include "pipeline/wowee_taxi.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'T', 'A', 'X'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wtax";

} // namespace

const WoweeTaxi::Node* WoweeTaxi::findNode(uint32_t nodeId) const {
    for (const auto& n : nodes) if (n.nodeId == nodeId) return &n;
    return nullptr;
}

const WoweeTaxi::Path* WoweeTaxi::findPath(uint32_t pathId) const {
    for (const auto& p : paths) if (p.pathId == pathId) return &p;
    return nullptr;
}

const WoweeTaxi::Path* WoweeTaxi::findPathBetween(uint32_t fromNodeId,
                                                   uint32_t toNodeId) const {
    for (const auto& p : paths) {
        if (p.fromNodeId == fromNodeId && p.toNodeId == toNodeId) return &p;
    }
    return nullptr;
}

bool WoweeTaxiLoader::save(const WoweeTaxi& cat,
                           const std::string& basePath) {
    std::ofstream os(normalizePath(basePath, kExtension), std::ios::binary);
    if (!os) return false;
    const uint32_t nodeCount = static_cast<uint32_t>(cat.nodes.size());
    writeCatalogHeader(os, kMagic, kVersion, cat.name, nodeCount);
    for (const auto& n : cat.nodes) {
        writePOD(os, n.nodeId);
        writePOD(os, n.mapId);
        writeStr(os, n.name);
        writeStr(os, n.iconPath);
        writePOD(os, n.position.x);
        writePOD(os, n.position.y);
        writePOD(os, n.position.z);
        writePOD(os, n.factionAlliance);
        writePOD(os, n.factionHorde);
    }
    uint32_t pathCount = static_cast<uint32_t>(cat.paths.size());
    writePOD(os, pathCount);
    for (const auto& p : cat.paths) {
        writePOD(os, p.pathId);
        writePOD(os, p.fromNodeId);
        writePOD(os, p.toNodeId);
        writePOD(os, p.moneyCostCopper);
        uint32_t wpCount = static_cast<uint32_t>(p.waypoints.size());
        writePOD(os, wpCount);
        for (const auto& w : p.waypoints) {
            writePOD(os, w.position.x);
            writePOD(os, w.position.y);
            writePOD(os, w.position.z);
            writePOD(os, w.delaySec);
        }
    }
    return os.good();
}

WoweeTaxi WoweeTaxiLoader::load(const std::string& basePath) {
    WoweeTaxi out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    uint32_t nodeCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, nodeCount)) return out;
    out.nodes.resize(nodeCount);
    for (auto& n : out.nodes) {
        if (!readPOD(is, n.nodeId) || !readPOD(is, n.mapId)) {
            out.nodes.clear(); return out;
        }
        if (!readStr(is, n.name) || !readStr(is, n.iconPath)) {
            out.nodes.clear(); return out;
        }
        if (!readPOD(is, n.position.x) ||
            !readPOD(is, n.position.y) ||
            !readPOD(is, n.position.z) ||
            !readPOD(is, n.factionAlliance) ||
            !readPOD(is, n.factionHorde)) {
            out.nodes.clear(); return out;
        }
    }
    uint32_t pathCount = 0;
    if (!readPOD(is, pathCount)) {
        out.nodes.clear(); return out;
    }
    if (pathCount > (1u << 20)) {
        out.nodes.clear(); return out;
    }
    out.paths.resize(pathCount);
    for (auto& p : out.paths) {
        if (!readPOD(is, p.pathId) ||
            !readPOD(is, p.fromNodeId) ||
            !readPOD(is, p.toNodeId) ||
            !readPOD(is, p.moneyCostCopper)) {
            out.nodes.clear(); out.paths.clear(); return out;
        }
        uint32_t wpCount = 0;
        if (!readPOD(is, wpCount)) {
            out.nodes.clear(); out.paths.clear(); return out;
        }
        if (wpCount > (1u << 16)) {
            out.nodes.clear(); out.paths.clear(); return out;
        }
        p.waypoints.resize(wpCount);
        for (auto& w : p.waypoints) {
            if (!readPOD(is, w.position.x) ||
                !readPOD(is, w.position.y) ||
                !readPOD(is, w.position.z) ||
                !readPOD(is, w.delaySec)) {
                out.nodes.clear(); out.paths.clear(); return out;
            }
        }
    }
    return out;
}

bool WoweeTaxiLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeTaxi WoweeTaxiLoader::makeStarter(const std::string& catalogName) {
    WoweeTaxi c;
    c.name = catalogName;
    {
        WoweeTaxi::Node n;
        n.nodeId = 1; n.mapId = 0;
        n.name = "Stormwind Gryphon Master";
        n.position = {-9000.0f, 100.0f, 50.0f};
        c.nodes.push_back(n);
    }
    {
        WoweeTaxi::Node n;
        n.nodeId = 2; n.mapId = 0;
        n.name = "Goldshire Gryphon Master";
        n.position = {-9460.0f, 60.0f, 56.0f};
        c.nodes.push_back(n);
    }
    {
        WoweeTaxi::Path p;
        p.pathId = 1; p.fromNodeId = 1; p.toNodeId = 2;
        p.moneyCostCopper = 5000;        // 50 silver
        // 3 waypoints carving a gentle arc between the cities.
        p.waypoints.push_back({.position = {-9100.0f,  90.0f, 80.0f}, .delaySec = 0.0f});
        p.waypoints.push_back({.position = {-9250.0f,  70.0f, 90.0f}, .delaySec = 0.0f});
        p.waypoints.push_back({.position = {-9460.0f,  60.0f, 56.0f}, .delaySec = 0.0f});
        c.paths.push_back(p);
    }
    {
        WoweeTaxi::Path p;
        p.pathId = 2; p.fromNodeId = 2; p.toNodeId = 1;
        p.moneyCostCopper = 5000;
        p.waypoints.push_back({.position = {-9250.0f,  70.0f, 90.0f}, .delaySec = 0.0f});
        p.waypoints.push_back({.position = {-9100.0f,  90.0f, 80.0f}, .delaySec = 0.0f});
        p.waypoints.push_back({.position = {-9000.0f, 100.0f, 50.0f}, .delaySec = 0.0f});
        c.paths.push_back(p);
    }
    return c;
}

WoweeTaxi WoweeTaxiLoader::makeRegion(const std::string& catalogName) {
    WoweeTaxi c;
    c.name = catalogName;
    // 4 nodes at corners of a 500m square at y=80 altitude.
    struct Pos { float x; float z; const char* name; };
    // NOLINTBEGIN(modernize-use-designated-initializers) - a table whose
    // columns are its field names, with the struct in view directly above.
    Pos posns[4] = {
        { -250.0f, -250.0f, "Northwest Outpost" },
        {  250.0f, -250.0f, "Northeast Outpost" },
        {  250.0f,  250.0f, "Southeast Outpost" },
        { -250.0f,  250.0f, "Southwest Outpost" },
    };
    // NOLINTEND(modernize-use-designated-initializers)
    for (int k = 0; k < 4; ++k) {
        WoweeTaxi::Node n;
        n.nodeId = 100 + k;
        n.mapId = 0;
        n.name = posns[k].name;
        n.position = {posns[k].x, 60.0f, posns[k].z};
        c.nodes.push_back(n);
    }
    // 4 paths forming a directed ring NW -> NE -> SE -> SW -> NW.
    for (int k = 0; k < 4; ++k) {
        int from = 100 + k;
        int to   = 100 + ((k + 1) % 4);
        WoweeTaxi::Path p;
        p.pathId = 100 + k;
        p.fromNodeId = from; p.toNodeId = to;
        p.moneyCostCopper = 2500;
        // 2 intermediate waypoints at altitude 90 (climb +
        // descend pattern).
        const auto& a = c.nodes[k].position;
        const auto& b = c.nodes[(k + 1) % 4].position;
        glm::vec3 mid1 = a + (b - a) * 0.33f;  mid1.y = 90.0f;
        glm::vec3 mid2 = a + (b - a) * 0.67f;  mid2.y = 90.0f;
        p.waypoints.push_back({.position = mid1, .delaySec = 0.0f});
        p.waypoints.push_back({.position = mid2, .delaySec = 0.0f});
        p.waypoints.push_back({.position = b,    .delaySec = 0.0f});
        c.paths.push_back(p);
    }
    return c;
}

WoweeTaxi WoweeTaxiLoader::makeContinent(const std::string& catalogName) {
    WoweeTaxi c;
    c.name = catalogName;
    // 6 nodes spread across a continent - a hub-and-spoke
    // network with 1 central node connected to 5 outliers.
    struct Pos { float x; float z; const char* name; };
    // NOLINTBEGIN(modernize-use-designated-initializers) - a table whose
    // columns are its field names, with the struct in view directly above.
    Pos posns[6] = {
        {     0.0f,     0.0f, "Crossroads (hub)" },
        { -1500.0f, -1500.0f, "Stormwind" },
        {  1500.0f, -1500.0f, "Stranglethorn" },
        {  1500.0f,  1500.0f, "Lordaeron" },
        { -1500.0f,  1500.0f, "Westfall" },
        {     0.0f,  3000.0f, "Tirisfal" },
    };
    // NOLINTEND(modernize-use-designated-initializers)
    for (int k = 0; k < 6; ++k) {
        WoweeTaxi::Node n;
        n.nodeId = 200 + k;
        n.mapId = 0;
        n.name = posns[k].name;
        n.position = {posns[k].x, 80.0f, posns[k].z};
        c.nodes.push_back(n);
    }
    // 8 paths: 5 hub-spoke (out + return) plus 3 cross-route
    // shortcuts on the perimeter.
    auto addPath = [&](uint32_t pid, uint32_t from, uint32_t to,
                       uint32_t cost) {
        WoweeTaxi::Path p;
        p.pathId = pid; p.fromNodeId = from; p.toNodeId = to;
        p.moneyCostCopper = cost;
        const auto& a = c.findNode(from)->position;
        const auto& b = c.findNode(to)->position;
        glm::vec3 mid1 = a + (b - a) * 0.5f;  mid1.y = 120.0f;
        p.waypoints.push_back({.position = mid1, .delaySec = 0.0f});
        p.waypoints.push_back({.position = b,    .delaySec = 0.0f});
        c.paths.push_back(p);
    };
    addPath(200, 200, 201, 8000);    // hub -> Stormwind
    addPath(201, 200, 202, 12000);   // hub -> Stranglethorn
    addPath(202, 200, 203, 10000);   // hub -> Lordaeron
    addPath(203, 200, 204, 6000);    // hub -> Westfall
    addPath(204, 200, 205, 15000);   // hub -> Tirisfal
    addPath(205, 201, 204, 4000);    // Stormwind -> Westfall (perimeter)
    addPath(206, 202, 203, 18000);   // Stranglethorn -> Lordaeron
    addPath(207, 203, 205, 6000);    // Lordaeron -> Tirisfal
    return c;
}

} // namespace pipeline
} // namespace wowee
