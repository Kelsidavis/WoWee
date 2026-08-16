#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Loading Screen catalog (.wlds) - novel
// replacement for Blizzard's LoadingScreens.dbc plus the
// per-zone background-image tables. Defines the loading
// screen images shown when the client crosses into a new
// map / instance, with optional level-bracket gating
// (different art for early-zone vs raid-tier visits) and
// expansion gating (TBC art only shown if expansion
// installed).
//
// When multiple screens match the player's current map +
// level + expansion, displayWeight selects randomly between
// them - a zone with 3 weighted variants gets a different
// image roughly proportional to weight.
//
// Cross-references with previously-added formats:
//   WLDS.entry.mapId → WMS.map.mapId (which map triggers
//                      this loading screen)
//
// Binary layout (little-endian):
//   magic[4]            = "WLDS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     screenId (uint32)
//     mapId (uint32)
//     nameLen + name
//     descLen + description
//     texLen + texturePath
//     iconLen + iconPath
//     attribLen + attribution
//     minLevel (uint16) / maxLevel (uint16)
//     displayWeight (uint16) / pad[2]
//     expansionRequired (uint8) / isAnimated (uint8) /
//       isWideAspect (uint8) / pad[1]
struct WoweeLoadingScreen {
    enum ExpansionGate : uint8_t {
        Classic   = 0,
        TBC       = 1,
        WotLK     = 2,
        TurtleWoW = 3,
    };

    struct Entry {
        uint32_t screenId = 0;
        uint32_t mapId = 0;             // WMS cross-ref (0 = catch-all)
        std::string name;
        std::string description;
        std::string texturePath;        // background image
        std::string iconPath;           // small loading-bar icon
        std::string attribution;        // artist credit
        uint16_t minLevel = 1;
        uint16_t maxLevel = 80;
        uint16_t displayWeight = 1;     // weighted random pick
        uint8_t expansionRequired = Classic;
        uint8_t isAnimated = 0;
        uint8_t isWideAspect = 0;       // 1 = 16:9, 0 = 4:3
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t screenId) const;

    static const char* expansionGateName(uint8_t e);
};

class WoweeLoadingScreenLoader {
public:
    static bool save(const WoweeLoadingScreen& cat,
                     const std::string& basePath);
    static WoweeLoadingScreen load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-lds* variants.
    //
    //   makeStarter   - 3 base screens (ElwynnForest,
    //                    OrgrimmarLoading, GenericFallback
    //                    with mapId=0 catch-all).
    //   makeInstances - 5 WotLK dungeon loading screens
    //                    (Halls of Lightning, Halls of
    //                    Stone, Utgarde Pinnacle, Violet
    //                    Hold, Old Kingdom) with proper
    //                    mapId+expansion cross-refs.
    //   makeRaidIntros - 3 raid loading screens (Naxxramas
    //                     dragon-eye reveal, Ulduar Titan
    //                     facility, ToC Argent Crusade) -
    //                     marked isWideAspect for the wider
    //                     16:9 raid intro art.
    static WoweeLoadingScreen makeStarter(const std::string& catalogName);
    static WoweeLoadingScreen makeInstances(const std::string& catalogName);
    static WoweeLoadingScreen makeRaidIntros(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
