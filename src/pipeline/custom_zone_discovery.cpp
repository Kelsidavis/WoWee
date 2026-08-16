#include "pipeline/custom_zone_discovery.hpp"
#include "core/logger.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace wowee {
namespace pipeline {

std::vector<CustomZoneInfo> CustomZoneDiscovery::scan(const std::vector<std::string>& searchPaths) {
    std::vector<CustomZoneInfo> results;
    for (const auto& path : searchPaths) {
        auto found = scanDirectory(path);
        results.insert(results.end(), found.begin(), found.end());
    }
    return results;
}

std::vector<CustomZoneInfo> CustomZoneDiscovery::scanDirectory(const std::string& path) {
    namespace fs = std::filesystem;
    std::vector<CustomZoneInfo> results;

    if (!fs::exists(path)) return results;

    for (auto& entry : fs::directory_iterator(path)) {
        if (!entry.is_directory()) continue;

        std::string zoneJson = entry.path().string() + "/zone.json";
        if (!fs::exists(zoneJson)) continue;

        std::ifstream f(zoneJson);
        if (!f) continue;

        try {
            auto j = nlohmann::json::parse(f);

            CustomZoneInfo info;
            info.name = j.value("mapName", "");
            if (info.name.empty()) info.name = j.value("name", "");
            info.author = j.value("author", "");
            info.description = j.value("description", "");
            info.mapId = j.value("mapId", 9000u);
            // Cap mapId to a reasonable range. Custom maps live in 9000+;
            // 0 / >65535 would either collide with stock maps or wrap u16
            // map fields in DBC indexing.
            if (info.mapId == 0 || info.mapId > 65535) info.mapId = 9000;
            info.directory = entry.path().string();
            info.hasCreatures = j.value("hasCreatures", false) ||
                                fs::exists(entry.path().string() + "/creatures.json");
            info.hasQuests = fs::exists(entry.path().string() + "/quests.json");

            if (j.contains("tiles") && j["tiles"].is_array()) {
                for (const auto& t : j["tiles"]) {
                    if (t.is_array() && t.size() >= 2) {
                        int tx = t[0].get<int>();
                        int ty = t[1].get<int>();
                        // WoW tile grid is 64x64. Drop bad entries instead
                        // of feeding them to the loader.
                        if (tx < 0 || tx > 63 || ty < 0 || ty > 63) continue;
                        info.tiles.emplace_back(tx, ty);
                    }
                }
            }

            if (!info.name.empty()) {
                results.push_back(info);
                LOG_INFO("Discovered custom zone: ", info.name, " in ", info.directory);
            }
        } catch (const std::exception& e) {
            LOG_WARNING("Failed to parse zone.json in ", entry.path().string(), ": ", e.what());
        }
    }

    return results;
}

} // namespace pipeline
} // namespace wowee
