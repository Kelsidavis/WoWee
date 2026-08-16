// zone_metadata.hpp - Zone level ranges, faction data, and label formatting.
// Extracted from WorldMap::initZoneMeta and inline label formatting
// (Phase 4 of refactoring plan). DRY - formatLabel used by multiple layers.
#pragma once

#include "rendering/world_map/world_map_types.hpp"
#include <string>
#include <unordered_map>

namespace wowee {
namespace rendering {
namespace world_map {

class ZoneMetadata {
public:
    /// Initialize the zone metadata table (level ranges, factions).
    void initialize();

    /// Look up metadata for a zone by area name. Returns nullptr if not found.
    [[nodiscard]] const ZoneMeta* find(const std::string& areaName) const;

    /// Format a zone label with level range and faction tag.
    /// e.g. "Elwynn (1-10) [Alliance]"
    static std::string formatLabel(const std::string& areaName,
                                    const ZoneMeta* meta);

    /// Format hover label with level range and bracket-tag for faction.
    static std::string formatHoverLabel(const std::string& areaName,
                                         const ZoneMeta* meta);

private:
    std::unordered_map<std::string, ZoneMeta> table_;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
