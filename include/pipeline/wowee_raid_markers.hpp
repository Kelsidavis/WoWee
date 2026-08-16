#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Raid Marker Set catalog (.wmar) - novel
// replacement for the hardcoded 8-marker set vanilla
// WoW shipped (Star/Circle/Diamond/Triangle/Moon/Square
// /Cross/Skull) plus world-map pin markers and 5-man
// party markers. Each entry binds one marker slot to
// its icon resource, single-character chat-overlay
// glyph (for "{star}" chat-style links), and priority
// for sort order in the marker-picker UI.
//
// Cross-references with previously-added formats:
//   No catalog cross-references - markers are visual-
//   identity primitives consumed directly by the raid
//   UI / minimap renderer.
//
// Binary layout (little-endian):
//   magic[4]            = "WMAR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     markerId (uint32)
//     nameLen + name
//     descLen + description
//     markerKind (uint8)         - RaidTarget /
//                                   WorldMap / Party /
//                                   Custom
//     priority (uint8)           - sort order in
//                                   picker UI
//     pad0 (uint8) / pad1 (uint8)
//     pathLen + iconPath          - BLP/WOT icon path
//     glyphLen + displayChar      - single-character
//                                   glyph for chat
//                                   overlay (e.g.
//                                   "*" for star)
//     iconColorRGBA (uint32)
struct WoweeRaidMarkers {
    enum MarkerKind : uint8_t {
        RaidTarget = 0,    // 8-marker standard raid set
        WorldMap   = 1,    // map-pin marker
        Party      = 2,    // 5-man party-only set
        Custom     = 3,    // server-custom marker
    };

    struct Entry {
        uint32_t markerId = 0;
        std::string name;
        std::string description;
        uint8_t markerKind = RaidTarget;
        uint8_t priority = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        std::string iconPath;
        std::string displayChar;     // 1 char glyph
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t markerId) const;

    // Returns all markers of one kind, sorted by
    // priority. Used by the marker-picker UI to
    // populate per-tab listings.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t markerKind) const;
};

class WoweeRaidMarkersLoader {
public:
    static bool save(const WoweeRaidMarkers& cat,
                     const std::string& basePath);
    static WoweeRaidMarkers load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mar* variants.
    //
    //   makeRaidTargets - 8 standard raid markers
    //                      (Star / Circle / Diamond /
    //                      Triangle / Moon / Square /
    //                      Cross / Skull) at priorities
    //                      0..7 matching their canonical
    //                      keybind slot.
    //   makeWorldMapPins - 5 world-map pin markers
    //                      (Pin / Flag / Crosshair /
    //                      Question / Compass).
    //   makeParty       - 4 5-man party-only markers
    //                      (Tank / Healer / DPS /
    //                      Caster - role icons for
    //                      groupfinder).
    static WoweeRaidMarkers makeRaidTargets(const std::string& catalogName);
    static WoweeRaidMarkers makeWorldMapPins(const std::string& catalogName);
    static WoweeRaidMarkers makeParty(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
