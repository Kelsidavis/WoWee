#include "pipeline/grass_biomes.hpp"

#include <algorithm>

#include <nlohmann/json.hpp>

namespace wowee {
namespace pipeline {

namespace {

std::optional<float> readFloat(const nlohmann::json& j, const char* key) {
    if (!j.contains(key) || !j[key].is_number()) return std::nullopt;
    return j[key].get<float>();
}

std::optional<glm::vec3> readColor(const nlohmann::json& j, const char* key) {
    if (!j.contains(key)) return std::nullopt;
    const auto& v = j[key];
    if (!v.is_array() || v.size() != 3) return std::nullopt;
    for (const auto& c : v) {
        if (!c.is_number()) return std::nullopt;
    }
    return glm::vec3(v[0].get<float>(), v[1].get<float>(), v[2].get<float>());
}

std::vector<uint32_t> readIds(const nlohmann::json& j, const char* key) {
    std::vector<uint32_t> out;
    if (!j.contains(key) || !j[key].is_array()) return out;
    for (const auto& id : j[key]) {
        if (id.is_number_unsigned()) out.push_back(id.get<uint32_t>());
    }
    return out;
}

} // namespace

void GrassBiomeOverride::apply(GrassProfile& profile) const {
    // Scales multiply so a biome shortens or thins what grows without
    // flattening the differences between what grows.
    if (heightScale) profile.heightScale *= *heightScale;
    if (widthScale) profile.widthScale *= *widthScale;
    if (densityScale) profile.densityScale *= *densityScale;
    if (rootColor) profile.rootColor = *rootColor;
    if (tipColor) profile.tipColor = *tipColor;
    if (colorVariation) profile.colorVariation = *colorVariation;
    if (stiffness) profile.stiffness = *stiffness;
    if (bloomChance) profile.bloomChance = *bloomChance;
    if (seedChance) profile.seedChance = *seedChance;
    if (bloomColorA) profile.bloomColorA = *bloomColorA;
    if (bloomColorB) profile.bloomColorB = *bloomColorB;
    if (headColorA) profile.headColorA = *headColorA;
    if (headColorB) profile.headColorB = *headColorB;
}

uint32_t GrassBiomeSet::findFor(uint32_t areaId, uint32_t zoneId) const {
    // Subzone rows first: "the vineyard, not the forest around it".
    for (size_t i = 0; i < biomes_.size(); ++i) {
        const auto& areas = biomes_[i].areas;
        if (std::find(areas.begin(), areas.end(), areaId) != areas.end()) {
            return static_cast<uint32_t>(i + 1);
        }
    }
    for (size_t i = 0; i < biomes_.size(); ++i) {
        const auto& zones = biomes_[i].zones;
        if (std::find(zones.begin(), zones.end(), zoneId) != zones.end()) {
            return static_cast<uint32_t>(i + 1);
        }
    }
    return 0;
}

GrassBiomeSet loadGrassBiomes(const std::string& jsonText, std::string& error) {
    GrassBiomeSet set;
    error.clear();

    // Comments allowed: this is an authored file, and a palette without notes
    // beside it is a palette nobody can maintain.
    const auto j = nlohmann::json::parse(jsonText, nullptr, false, true);
    if (j.is_discarded()) {
        error = "grass_biomes: not valid JSON";
        return set;
    }
    if (!j.contains("biomes") || !j["biomes"].is_array()) {
        error = "grass_biomes: no \"biomes\" array";
        return set;
    }

    for (const auto& b : j["biomes"]) {
        if (!b.is_object()) continue;
        GrassBiome biome;
        biome.name = b.value("name", std::string{});
        biome.zones = readIds(b, "zones");
        biome.areas = readIds(b, "areas");
        // An entry that can never match is an authoring mistake worth
        // surfacing, but not worth losing the rest of the file over.
        if (biome.zones.empty() && biome.areas.empty()) continue;

        GrassBiomeOverride& o = biome.override_;
        o.heightScale = readFloat(b, "heightScale");
        o.widthScale = readFloat(b, "widthScale");
        o.densityScale = readFloat(b, "densityScale");
        o.rootColor = readColor(b, "rootColor");
        o.tipColor = readColor(b, "tipColor");
        o.colorVariation = readFloat(b, "colorVariation");
        o.stiffness = readFloat(b, "stiffness");
        o.bloomChance = readFloat(b, "bloomChance");
        o.seedChance = readFloat(b, "seedChance");
        o.bloomColorA = readColor(b, "bloomColorA");
        o.bloomColorB = readColor(b, "bloomColorB");
        o.headColorA = readColor(b, "headColorA");
        o.headColorB = readColor(b, "headColorB");

        set.biomes_.push_back(std::move(biome));
    }
    return set;
}

} // namespace pipeline
} // namespace wowee
