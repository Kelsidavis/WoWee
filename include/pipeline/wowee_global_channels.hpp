#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Global Chat Channel catalog (.wgch) -
// novel replacement for the implicit chat-channel
// configuration vanilla WoW carried in
// ChatChannels.dbc + the per-server zone-default chat
// joins. Each entry binds one server-wide or
// zone-scoped chat channel (LookingForGroup, World,
// Trade, GeneralChat, RP-OOC) to its access policy
// (public-join / invite-only / auto-join on zone
// entry / moderated), level gate, member cap, and
// password requirement.
//
// Cross-references with previously-added formats:
//   WCHN: WGCH supersedes WCHN's narrower chat-channel
//         catalog where access-policy semantics matter.
//   WMS:  zoneDefaultMapId references the WMS map
//         catalog (for AutoJoin channels that auto-
//         enroll players entering a specific zone).
//
// Binary layout (little-endian):
//   magic[4]            = "WGCH"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     channelId (uint32)
//     nameLen + name (channel display name)
//     descLen + description
//     channelKind (uint8)        - Global / RealmZone /
//                                   Faction / Custom
//     accessKind (uint8)         - PublicJoin /
//                                   InviteOnly /
//                                   AutoJoinOnZone /
//                                   Moderated
//     passwordRequired (uint8)   - 0/1 bool
//     levelMin (uint8)
//     maxMembers (uint16)        - 0 = unlimited
//     topicSetByMods (uint8)     - 0/1 bool - false
//                                   means anyone can /topic
//     pad0 (uint8)
//     zoneDefaultMapId (uint32)  - for AutoJoinOnZone
//                                   kind; 0 if not auto-
//                                   joined per-zone
//     iconColorRGBA (uint32)
struct WoweeGlobalChannels {
    enum ChannelKind : uint8_t {
        Global    = 0,    // server-wide
        RealmZone = 1,    // realm-wide zone-default
        Faction   = 2,    // single-faction
        Custom    = 3,    // player-created
    };

    enum AccessKind : uint8_t {
        PublicJoin     = 0,    // /join chan
        InviteOnly     = 1,    // requires owner invite
        AutoJoinOnZone = 2,    // auto-enroll on
                                // zoneDefaultMapId entry
        Moderated      = 3,    // joinable but only mods
                                // can speak
    };

    struct Entry {
        uint32_t channelId = 0;
        std::string name;
        std::string description;
        uint8_t channelKind = Global;
        uint8_t accessKind = PublicJoin;
        uint8_t passwordRequired = 0;
        uint8_t levelMin = 0;
        uint16_t maxMembers = 0;
        uint8_t topicSetByMods = 1;
        uint8_t pad0 = 0;
        uint32_t zoneDefaultMapId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t channelId) const;

    // Returns all channels of one kind. Used by the
    // chat-window UI to populate per-kind tabs (Global
    // tab, Custom tab, etc.).
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t channelKind) const;

    // Returns AutoJoinOnZone channels that should
    // enroll a player entering the given mapId. Used
    // by the zone-load handler.
    [[nodiscard]] std::vector<const Entry*> findAutoJoinForZone(uint32_t mapId) const;
};

class WoweeGlobalChannelsLoader {
public:
    static bool save(const WoweeGlobalChannels& cat,
                     const std::string& basePath);
    static WoweeGlobalChannels load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-gch* variants.
    //
    //   makeStandardChannels - 4 vanilla server-wide
    //                           channels (LookingFor-
    //                           Group / World / Trade
    //                           on auto-join Stormwind /
    //                           General).
    //   makeRoleplay         - 4 RP server channels
    //                           (RP-OOC / RP-IC moderated
    //                           / RP-Forum invite-only /
    //                           RP-Events).
    //   makeAdminChannels    - 3 moderator-only channels
    //                           (GMTraffic invite /
    //                           AuditLog moderated /
    //                           Backstage invite - all
    //                           level 0 require GM flag
    //                           via accessKind).
    static WoweeGlobalChannels makeStandardChannels(const std::string& catalogName);
    static WoweeGlobalChannels makeRoleplay(const std::string& catalogName);
    static WoweeGlobalChannels makeAdminChannels(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
