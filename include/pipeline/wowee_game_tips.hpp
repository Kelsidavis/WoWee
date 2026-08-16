#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Game Tips catalog (.wgtp) - novel replacement
// for Blizzard's GameTips.dbc plus loading-screen tutorial
// hint tables. Defines the rotating tips shown during world
// loads, the contextual tutorial hints that fire on first
// gameplay events (first quest accept, first death, first
// dungeon entry), and the persistent tooltip-help strings
// that explain UI elements.
//
// Each tip has filter criteria - audience bitmask (faction /
// new-player / hardcore), level range, optional class mask,
// optional WPCN condition cross-ref - that the runtime uses
// to pick the right pool of tips for the current player.
// displayWeight controls relative frequency within the pool.
//
// Cross-references with previously-added formats:
//   WGTP.entry.conditionId    → WPCN.entry.conditionId
//                                (further gate beyond audience)
//   WGTP.entry.requiredClassMask bit positions match
//                                WCHC.class.classId
//
// Binary layout (little-endian):
//   magic[4]            = "WGTP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tipId (uint32)
//     nameLen + name
//     textLen + text
//     iconLen + iconPath
//     displayKind (uint8) / pad[3]
//     audienceFilter (uint32)
//     minLevel (uint16) / maxLevel (uint16)
//     displayWeight (uint16) / pad[2]
//     conditionId (uint32)
//     requiredClassMask (uint32)
struct WoweeGameTip {
    enum DisplayKind : uint8_t {
        LoadingScreen = 0,    // long load-time scrolling tip
        Tutorial      = 1,    // contextual modal on first event
        TooltipHelp   = 2,    // persistent UI element help
        Hint          = 3,    // brief on-screen flyout
    };

    // audienceFilter bits - combine with bitwise OR to broaden.
    static constexpr uint32_t kAudienceAlliance   = 1u << 0;
    static constexpr uint32_t kAudienceHorde      = 1u << 1;
    static constexpr uint32_t kAudienceNewPlayer  = 1u << 2;
    static constexpr uint32_t kAudienceHardcore   = 1u << 3;
    static constexpr uint32_t kAudiencePvE        = 1u << 4;
    static constexpr uint32_t kAudiencePvP        = 1u << 5;
    static constexpr uint32_t kAudienceRoleplay   = 1u << 6;
    static constexpr uint32_t kAudienceAll        = 0xFFFFFFFFu;

    struct Entry {
        uint32_t tipId = 0;
        std::string name;             // internal stable id ("FirstQuest")
        std::string text;             // the displayed text
        std::string iconPath;
        uint8_t displayKind = LoadingScreen;
        uint32_t audienceFilter = kAudienceAll;
        uint16_t minLevel = 1;
        uint16_t maxLevel = 80;
        uint16_t displayWeight = 1;   // relative frequency
        uint32_t conditionId = 0;     // WPCN cross-ref (0 = none)
        uint32_t requiredClassMask = 0;  // 0 = any class
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tipId) const;

    static const char* displayKindName(uint8_t k);
};

class WoweeGameTipLoader {
public:
    static bool save(const WoweeGameTip& cat,
                     const std::string& basePath);
    static WoweeGameTip load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tips* variants.
    //
    //   makeStarter   - 3 generic loading-screen tips
    //                    (combat hint / movement hint /
    //                    quest hint) - kAudienceAll, no
    //                    condition gate.
    //   makeNewPlayer - 5 onboarding tips for level 1-15
    //                    players (kAudienceNewPlayer bit
    //                    set), Tutorial display kind.
    //   makeAdvanced  - 4 tips for max-level players
    //                    (raid mechanics / PvP mechanics /
    //                    profession dailies / dungeon-finder
    //                    etiquette) gated by minLevel 70+.
    static WoweeGameTip makeStarter(const std::string& catalogName);
    static WoweeGameTip makeNewPlayer(const std::string& catalogName);
    static WoweeGameTip makeAdvanced(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
