#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Master Server Profile catalog (.wmsp) -
// novel replacement for the hardcoded realmlist that the
// WoW client receives via SMSG_REALM_LIST during login.
// Each entry is one selectable realm: name, network
// address, type (Normal / PvP / RP / RPPvP / Test),
// expansion gating, population indicator, character cap,
// and access flags.
//
// 100th open format - milestone marker. Server admins use
// this catalog as the single source of truth for which
// realms appear on the realm picker; loading it replaces
// the hardcoded `realmlist.wtf` lookup that vanilla
// servers have nailed into their login daemon.
//
// Cross-references with previously-added formats:
//   No catalog cross-references; WMSP is a TOP-LEVEL
//   bootstrap catalog read before any in-world data.
//   The realmlist is consumed by the login server before
//   any character is loaded, so it cannot reference
//   anything that depends on having a logged-in player.
//
// Binary layout (little-endian):
//   magic[4]            = "WMSP"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     realmId (uint32)
//     nameLen + name
//     descLen + description
//     addrLen + address (host:port)
//     realmType (uint8)         - Normal / PvP / RP /
//                                  RPPvP / Test
//     realmCategory (uint8)     - Public / Private /
//                                  Beta / Dev
//     expansion (uint8)         - Vanilla / TBC / WotLK /
//                                  Cata
//     population (uint8)        - Low / Medium / High /
//                                  Full / Locked
//     characterCap (uint8)
//     gmOnly (uint8)             - 0/1 bool
//     timezone (uint8)
//     pad0 (uint8)
//     versionMajor (uint8) / versionMinor (uint8)
//     versionPatch (uint8) / pad1 (uint8)
//     buildNumber (uint32)
//     iconColorRGBA (uint32)
struct WoweeRealmList {
    enum RealmType : uint8_t {
        Normal = 0,
        PvP    = 1,
        RP     = 6,
        RPPvP  = 8,
        Test   = 4,
    };

    enum RealmCategory : uint8_t {
        Public  = 0,
        Private = 1,
        Beta    = 2,
        Dev     = 3,
    };

    enum Expansion : uint8_t {
        Vanilla = 0,    // 1.12 / 1.x
        TBC     = 1,    // 2.4.3
        WotLK   = 2,    // 3.3.5a
        Cata    = 3,    // 4.x (future)
    };

    enum Population : uint8_t {
        Low    = 0,
        Medium = 1,
        High   = 2,
        Full   = 3,
        Locked = 4,
    };

    struct Entry {
        uint32_t realmId = 0;
        std::string name;
        std::string description;
        std::string address;          // "host:port"
        uint8_t realmType = Normal;
        uint8_t realmCategory = Public;
        uint8_t expansion = WotLK;
        uint8_t population = Medium;
        uint8_t characterCap = 10;
        uint8_t gmOnly = 0;
        uint8_t timezone = 8;          // East Coast US
        uint8_t pad0 = 0;
        uint8_t versionMajor = 3;
        uint8_t versionMinor = 3;
        uint8_t versionPatch = 5;
        uint8_t pad1 = 0;
        uint32_t buildNumber = 12340;  // WotLK 3.3.5a
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t realmId) const;

    // Returns realms a player should see based on their
    // installed expansion (Vanilla clients can only see
    // Vanilla realms; WotLK clients see Vanilla/TBC/WotLK
    // due to the realm picker being expansion-tolerant).
    [[nodiscard]] std::vector<const Entry*> findByExpansion(uint8_t maxExpansion) const;

    // Returns realms of one type - used by the picker UI's
    // "PvP only" / "RP only" filters.
    [[nodiscard]] std::vector<const Entry*> findByType(uint8_t realmType) const;
};

class WoweeRealmListLoader {
public:
    static bool save(const WoweeRealmList& cat,
                     const std::string& basePath);
    static WoweeRealmList load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-msp* variants.
    //
    //   makeSingleRealm    - 1 entry: WoweeMain (WotLK
    //                         Normal, Public, Medium pop,
    //                         10-char cap, US East TZ).
    //   makePvPCluster     - 3 entries: WoweePvE / WoweePvP
    //                         / WoweeRP (same login address,
    //                         3 realm types - players can
    //                         pick rule-set without
    //                         changing servers).
    //   makeMultiExpansion - 4 entries spanning all
    //                         supported expansion gates
    //                         (Vanilla 1.12 / TBC 2.4.3 /
    //                         WotLK 3.3.5a / Cata 4.3.4)
    //                         each with its own buildNumber.
    static WoweeRealmList makeSingleRealm(const std::string& catalogName);
    static WoweeRealmList makePvPCluster(const std::string& catalogName);
    static WoweeRealmList makeMultiExpansion(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
