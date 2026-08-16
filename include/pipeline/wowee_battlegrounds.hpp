#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Battleground Definition catalog (.wbgd) -
// novel replacement for Blizzard's BattlemasterList.dbc +
// PvpDifficulty.dbc + the AzerothCore-style
// battleground_template SQL tables. The 33rd open format
// added to the editor.
//
// Defines per-BG gameplay rules: player count brackets,
// score-to-win, time limit, objective type (annihilation /
// capture flag / control nodes / king of hill / resource
// race / carry object), per-team start positions, respawn
// timer, and the WTKN currency token awarded on win.
//
// Cross-references with previously-added formats:
//   WBGD.entry.mapId         → WMS.map.mapId
//                              (where mapType=Battleground)
//   WBGD.entry.markTokenId   → WTKN.entry.tokenId
//                              (Mark of Honor for that BG)
//
// Binary layout (little-endian):
//   magic[4]            = "WBGD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     battlegroundId (uint32)
//     mapId (uint32)
//     nameLen + name
//     descLen + description
//     objectiveKind (uint8)
//     minPlayersPerSide (uint8) / maxPlayersPerSide (uint8) / pad[1]
//     minLevel (uint16) / maxLevel (uint16)
//     scoreToWin (uint16) / timeLimitSeconds (uint16)
//     bracketSize (uint8) / pad[3]
//     allianceStart (3 × float) / allianceFacing (float)
//     hordeStart (3 × float) / hordeFacing (float)
//     respawnTimeSeconds (uint16) / pad[2]
//     markTokenId (uint32)
struct WoweeBattleground {
    enum ObjectiveKind : uint8_t {
        Annihilation = 0,    // wipe the opposing team
        CaptureFlag  = 1,    // CTF - Warsong Gulch
        ControlNodes = 2,    // node capture - Arathi Basin / Eye of the Storm
        KingOfHill   = 3,    // hold a single point
        ResourceRace = 4,    // Alterac Valley (general/wing-tower scoring)
        CarryObject  = 5,    // carry a relic / orb - Eye of the Storm flag
    };

    struct Entry {
        uint32_t battlegroundId = 0;
        uint32_t mapId = 0;
        std::string name;
        std::string description;
        uint8_t objectiveKind = Annihilation;
        uint8_t minPlayersPerSide = 5;
        uint8_t maxPlayersPerSide = 10;
        uint16_t minLevel = 10;
        uint16_t maxLevel = 80;
        uint16_t scoreToWin = 3;
        uint16_t timeLimitSeconds = 1800;        // 30 min default
        uint8_t bracketSize = 10;                 // 10-level brackets
        glm::vec3 allianceStart{0};
        float allianceFacing = 0.0f;
        glm::vec3 hordeStart{0};
        float hordeFacing = 0.0f;
        uint16_t respawnTimeSeconds = 30;
        uint32_t markTokenId = 0;                 // WTKN cross-ref
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bgId) const;

    static const char* objectiveKindName(uint8_t k);
};

class WoweeBattlegroundLoader {
public:
    static bool save(const WoweeBattleground& cat,
                     const std::string& basePath);
    static WoweeBattleground load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bg* variants.
    //
    //   makeStarter - 1 simple king-of-hill BG (10v10, 3-cap
    //                  to win, 30 min time limit).
    //   makeClassic - Warsong Gulch (CTF) + Arathi Basin
    //                  (control nodes) + Alterac Valley
    //                  (resource race), each with WMS+WTKN
    //                  cross-references and authentic player
    //                  counts (10 / 15 / 40 per side).
    //   makeArena   - 3 arena formats: 2v2 / 3v3 / 5v5 with
    //                  small map IDs and annihilation
    //                  objectives.
    static WoweeBattleground makeStarter(const std::string& catalogName);
    static WoweeBattleground makeClassic(const std::string& catalogName);
    static WoweeBattleground makeArena(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
