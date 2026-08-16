#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open World State UI catalog (.wwui) - novel
// replacement for Blizzard's WorldStateUI.dbc plus the
// AzerothCore-style world_state SQL data. The 48th open
// format added to the editor.
//
// Defines on-screen UI elements that surface server-side
// world-state variables: battleground scoreboards (flag
// captures, base controls), Wintergrasp tank counters,
// Eye of the Storm flag-carrier indicator, dungeon boss
// progress, world-event collection trackers. Each entry
// binds a server-side variableIndex to a UI panel kind
// (counter / timer / progress bar / flag icon) gated by
// map / area, optionally always-visible, optionally
// hidden when the value is zero.
//
// Cross-references with previously-added formats:
//   WWUI.entry.mapId       → WMS.map.mapId
//   WWUI.entry.areaId      → WMS.area.areaId
//
// Binary layout (little-endian):
//   magic[4]            = "WWUI"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     worldStateId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     displayKind (uint8) / panelPosition (uint8) /
//       alwaysVisible (uint8) / hideWhenZero (uint8)
//     mapId (uint32)
//     areaId (uint32)
//     variableIndex (uint32)
//     defaultValue (int32)
//     iconColorRGBA (uint32)
struct WoweeWorldStateUI {
    enum DisplayKind : uint8_t {
        Counter        = 0,    // numeric counter (e.g. flag captures)
        Timer          = 1,    // mm:ss countdown / countup
        FlagIcon       = 2,    // ally/horde icon (flag carrier)
        ProgressBar    = 3,    // 0..max horizontal bar
        TwoSidedScore  = 4,    // ally vs horde dual counter
        Custom         = 5,    // engine-driven custom widget
    };

    enum PanelPosition : uint8_t {
        Top       = 0,
        Bottom    = 1,
        TopLeft   = 2,
        TopRight  = 3,
        Center    = 4,    // big middle-screen banner
    };

    struct Entry {
        uint32_t worldStateId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t displayKind = Counter;
        uint8_t panelPosition = Top;
        uint8_t alwaysVisible = 0;     // 1 = visible while in zone
        uint8_t hideWhenZero = 0;      // 1 = hide when value=0
        uint32_t mapId = 0;            // WMS cross-ref
        uint32_t areaId = 0;           // WMS cross-ref
        uint32_t variableIndex = 0;    // server-side var slot
        int32_t defaultValue = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t worldStateId) const;

    static const char* displayKindName(uint8_t k);
    static const char* panelPositionName(uint8_t p);
};

class WoweeWorldStateUILoader {
public:
    static bool save(const WoweeWorldStateUI& cat,
                     const std::string& basePath);
    static WoweeWorldStateUI load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-wsui* variants.
    //
    //   makeStarter     - 3 BG scoreboards (Warsong Gulch
    //                      flag captures, Arathi Basin
    //                      resource counters, Eye of the
    //                      Storm flag carrier).
    //   makeWintergrasp - 4 Wintergrasp UI (alliance + horde
    //                      tank counts, time remaining,
    //                      towers controlled).
    //   makeDungeon     - 3 dungeon UI (boss progress bar,
    //                      key fragments collected,
    //                      treasure hunt counter).
    static WoweeWorldStateUI makeStarter(const std::string& catalogName);
    static WoweeWorldStateUI makeWintergrasp(const std::string& catalogName);
    static WoweeWorldStateUI makeDungeon(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
