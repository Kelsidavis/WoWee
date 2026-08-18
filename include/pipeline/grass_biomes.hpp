#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "pipeline/grass_profile.hpp"

// Per-region art direction for grass.
//
// What grows is the map's answer - ground effects, alpha weights, the no-grow
// masks - and stays zone-free (grass_profile.hpp). How it looks is authored
// here: assets/grass_biomes.json maps AreaTable zones to overrides, so Elwynn
// can keep short green grass with tulips while Westfall runs to wheat, from a
// file an artist edits without touching a compiler.
//
// Matching is by the chunk's own areaId: an entry's `areas` list matches the
// subzone id directly and wins; failing that, `zones` matches the id the
// caller resolved by walking AreaTable parentage. First matching entry wins,
// so specific rows belong above general ones.

namespace wowee {
namespace pipeline {

/// What a biome may change about a profile. Absent fields leave the
/// effect-derived value alone, so a biome can retint a zone without
/// flattening the meadow/scrub/scree differences inside it.
///
/// The three scales multiply the profile's own - "short grass here" shortens
/// scrub and meadow alike rather than making them the same height. Colours
/// and chances replace outright, because they are the look being authored.
struct GrassBiomeOverride {
    std::optional<float> heightScale;    ///< multiplies
    std::optional<float> widthScale;     ///< multiplies
    std::optional<float> densityScale;   ///< multiplies; 0 = no grass at all
    std::optional<glm::vec3> rootColor;  ///< replaces
    std::optional<glm::vec3> tipColor;   ///< replaces
    std::optional<float> colorVariation; ///< replaces
    std::optional<float> stiffness;      ///< replaces
    std::optional<float> bloomChance;    ///< replaces
    std::optional<float> seedChance;     ///< replaces
    std::optional<glm::vec3> bloomColorA; ///< replaces
    std::optional<glm::vec3> bloomColorB; ///< replaces
    std::optional<glm::vec3> headColorA;  ///< replaces
    std::optional<glm::vec3> headColorB;  ///< replaces

    void apply(GrassProfile& profile) const;
};

struct GrassBiome {
    std::string name;
    std::vector<uint32_t> zones;  ///< matched against the resolved zone id
    std::vector<uint32_t> areas;  ///< matched against the raw area id; wins
    GrassBiomeOverride override_;
};

class GrassBiomeSet {
public:
    /// Index+1 of the biome for this ground, or 0 for none. Area match beats
    /// zone match; among equals the earlier entry wins.
    [[nodiscard]] uint32_t findFor(uint32_t areaId, uint32_t zoneId) const;

    [[nodiscard]] const GrassBiome* biome(uint32_t idxPlusOne) const {
        if (idxPlusOne == 0 || idxPlusOne > biomes_.size()) return nullptr;
        return &biomes_[idxPlusOne - 1];
    }

    [[nodiscard]] size_t size() const { return biomes_.size(); }

    std::vector<GrassBiome> biomes_;
};

/// Parse the biome table from JSON text. Comments (// and /* */) are allowed.
/// Returns an empty set and fills `error` when the text does not parse;
/// individual malformed entries are skipped rather than failing the file.
[[nodiscard]] GrassBiomeSet loadGrassBiomes(const std::string& jsonText, std::string& error);

} // namespace pipeline
} // namespace wowee
