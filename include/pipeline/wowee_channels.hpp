#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Chat Channel catalog (.wchn) - novel
// replacement for Blizzard's ChatChannels.dbc + the
// AzerothCore-style chat_channel SQL tables. The 40th
// open format added to the editor.
//
// Defines the world chat channel system: General, Trade,
// LookingForGroup, GuildRecruitment, LocalDefense, plus
// per-zone area channels and custom user-created channels.
// Each channel has access rules (faction / level), join
// behavior (auto vs opt-in), broadcast policy (announce /
// moderated), and optional area / map gating that
// auto-joins or auto-leaves the channel as the player moves.
//
// Cross-references with previously-added formats:
//   WCHN.entry.areaIdGate  → WMS.area.areaId
//                            (channel auto-attaches in this area)
//   WCHN.entry.mapIdGate   → WMS.map.mapId
//                            (channel auto-attaches on this map)
//
// Binary layout (little-endian):
//   magic[4]            = "WCHN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     channelId (uint32)
//     nameLen + name
//     descLen + description
//     channelType (uint8) / factionAccess (uint8) /
//       autoJoin (uint8) / announce (uint8)
//     moderated (uint8) / pad[1] / minLevel (uint16)
//     areaIdGate (uint32) / mapIdGate (uint32)
struct WoweeChannel {
    enum ChannelType : uint8_t {
        AreaLocal       = 0,    // /local within a sub-zone
        Zone            = 1,    // zone-wide
        Continent       = 2,    // continent-wide
        World           = 3,    // entire realm
        Trade           = 4,    // trade district + opt-in
        LookingForGroup = 5,    // LFG global
        GuildRecruit    = 6,    // GuildFinder global
        LocalDefense    = 7,    // alarm channel for zone attacks
        Custom          = 8,    // user-created
        Pvp             = 9,    // PvP-mode players
    };

    enum FactionAccess : uint8_t {
        Alliance = 0,
        Horde    = 1,
        Both     = 2,
    };

    struct Entry {
        uint32_t channelId = 0;
        std::string name;
        std::string description;
        uint8_t channelType = AreaLocal;
        uint8_t factionAccess = Both;
        uint8_t autoJoin = 0;            // 1 = auto-join on context match
        uint8_t announce = 1;            // 1 = broadcast join/leave
        uint8_t moderated = 0;           // 1 = only operators speak
        uint16_t minLevel = 1;
        uint32_t areaIdGate = 0;         // 0 = no gate
        uint32_t mapIdGate = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t channelId) const;

    static const char* channelTypeName(uint8_t t);
    static const char* factionAccessName(uint8_t f);
};

class WoweeChannelLoader {
public:
    static bool save(const WoweeChannel& cat,
                     const std::string& basePath);
    static WoweeChannel load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-channels* variants.
    //
    //   makeStarter - 4 stock channels (General / Trade /
    //                  LFG / GuildRecruit) with default
    //                  autoJoin and announce rules.
    //   makeCity    - 5 city-specific channels (Stormwind
    //                  General / Trade / LFG + Orgrimmar
    //                  General / Trade) with mapId gates.
    //   makeModerated - 3 moderated / restricted channels
    //                    (LocalDefense level 10+, World event
    //                    chat, custom raid coordination).
    static WoweeChannel makeStarter(const std::string& catalogName);
    static WoweeChannel makeCity(const std::string& catalogName);
    static WoweeChannel makeModerated(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
