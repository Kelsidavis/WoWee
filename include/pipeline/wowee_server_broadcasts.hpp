#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Server Channel Broadcast catalog (.wscb) -
// novel replacement for the hardcoded login-MOTD,
// server-restart-warning, and rotating-help-tip messages
// that vanilla servers nail into source. Each entry is
// one scheduled or event-triggered broadcast: a one-shot
// MOTD shown on login, a periodic system message
// ("server restart in 10 minutes"), a raid-wide warning,
// or a rotating gameplay tip displayed in the /help
// channel.
//
// Cross-references with previously-added formats:
//   WCHN: channelKind values 0..4 map to dispatch sinks;
//         the SystemChannel/HelpTip variants land in the
//         WCHN-defined system / help channels.
//   WCHC: factionFilter uses the WCHC faction-mask bits
//         (1=Alliance, 2=Horde, 3=Both, 0=neither which
//         means "no broadcast" - validator warns).
//
// Binary layout (little-endian):
//   magic[4]            = "WSCB"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     broadcastId (uint32)
//     nameLen + name
//     descLen + description
//     msgLen + messageText
//     intervalSeconds (uint32)  - 0 = login-only / once
//     channelKind (uint8)        - Login / SystemChannel /
//                                  RaidWarning / MOTD /
//                                  HelpTip
//     factionFilter (uint8)      - WCHC faction-mask bits
//     minLevel (uint8)           - earliest level player
//                                  must be to receive (0
//                                  = any)
//     maxLevel (uint8)           - latest level (0 = any)
//     iconColorRGBA (uint32)
struct WoweeServerBroadcasts {
    enum ChannelKind : uint8_t {
        Login          = 0,    // shown once on character
                               // entering world
        SystemChannel  = 1,    // WCHN system channel
                               // (server-side)
        RaidWarning    = 2,    // SMSG_RAID_WARNING (red
                               // banner across screen)
        MOTD           = 3,    // appended to existing
                               // server MOTD chain
        HelpTip        = 4,    // rotating tip shown in
                               // /help channel
    };

    enum FactionFilter : uint8_t {
        AllianceOnly = 1,
        HordeOnly    = 2,
        Both         = 3,
    };

    struct Entry {
        uint32_t broadcastId = 0;
        std::string name;
        std::string description;
        std::string messageText;     // body shown to player
        uint32_t intervalSeconds = 0;
        uint8_t channelKind = MOTD;
        uint8_t factionFilter = Both;
        uint8_t minLevel = 0;
        uint8_t maxLevel = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t broadcastId) const;

    // Returns all broadcasts that should fire for a player
    // of the given faction and level. Used by the
    // BroadcastTicker to build the per-player message
    // queue on login.
    [[nodiscard]] std::vector<const Entry*> findFor(uint8_t playerFaction,
                                       uint8_t playerLevel) const;

    // Returns all broadcasts of one channel kind (used by
    // the periodic ticker to schedule SystemChannel /
    // HelpTip rotations independently from login MOTDs).
    [[nodiscard]] std::vector<const Entry*> findByChannel(uint8_t channelKind) const;
};

class WoweeServerBroadcastsLoader {
public:
    static bool save(const WoweeServerBroadcasts& cat,
                     const std::string& basePath);
    static WoweeServerBroadcasts load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-scb* variants.
    //
    //   makeMotd        - 4 login MOTD entries (welcome /
    //                      patch notes summary / Discord
    //                      link / forum link).
    //   makeMaintenance - 3 raid-wide maintenance warnings
    //                      at decreasing intervals (15min
    //                      / 5min / 1min before restart),
    //                      each with intervalSeconds=0
    //                      (one-shot).
    //   makeHelpTips    - 6 rotating help-channel tips on
    //                      a 600s (10min) cycle covering
    //                      core gameplay (talents, mounts,
    //                      auction, professions, dungeon
    //                      finder, hearthstone).
    static WoweeServerBroadcasts makeMotd(const std::string& catalogName);
    static WoweeServerBroadcasts makeMaintenance(const std::string& catalogName);
    static WoweeServerBroadcasts makeHelpTips(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
