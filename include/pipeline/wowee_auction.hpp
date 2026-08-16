#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Auction House catalog (.wauc) - novel
// replacement for Blizzard's AuctionHouse.dbc + the
// AzerothCore-style auctionhouse / auctionhousebot SQL
// tables. The 39th open format added to the editor.
//
// Defines per-house rules for the auction system:
// faction access, deposit rate (basis points of buyout
// price), house cut on successful sale, three listing
// duration tiers with per-tier deposit multipliers,
// disallowed item-class bitmask, and the auctioneer NPC
// who acts as the in-world interface.
//
// Cross-references with previously-added formats:
//   WAUC.entry.auctioneerNpcId  → WCRT.entry.creatureId
//                                  (Auctioneer-flagged NPC)
//   WAUC.entry.disallowedClassMask
//                                bitmask of WIT.Class values
//                                that may not be auctioned at
//                                this house (Quest items,
//                                Containers, etc.)
//
// Binary layout (little-endian):
//   magic[4]            = "WAUC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     houseId (uint32)
//     auctioneerNpcId (uint32)
//     nameLen + name
//     factionAccess (uint8) / pad[3]
//     baseDepositRateBp (uint32)        -- basis points (out of 10000)
//     houseCutRateBp (uint32)
//     maxBidCopper (uint32)             -- 0 = unlimited
//     shortHours (uint16) / mediumHours (uint16)
//     longHours (uint16) / pad[2]
//     shortMultBp (uint32) / mediumMultBp (uint32)
//     longMultBp (uint32)
//     disallowedClassMask (uint32)
struct WoweeAuction {
    enum FactionAccess : uint8_t {
        Alliance = 0,
        Horde    = 1,
        Neutral  = 2,    // goblin AHs in classic - both factions can use
        Both     = 3,    // private / shared instance house
    };

    static constexpr uint16_t kBpDenominator = 10000;   // basis-point base

    struct Entry {
        uint32_t houseId = 0;
        uint32_t auctioneerNpcId = 0;
        std::string name;
        uint8_t factionAccess = Alliance;
        uint32_t baseDepositRateBp = 1500;     // 15% per short tier default
        uint32_t houseCutRateBp = 500;         // 5% house cut
        uint32_t maxBidCopper = 0;             // unlimited by default
        uint16_t shortHours = 12;
        uint16_t mediumHours = 24;
        uint16_t longHours = 48;
        uint32_t shortMultBp = 10000;          // 1x base
        uint32_t mediumMultBp = 20000;         // 2x base
        uint32_t longMultBp = 40000;           // 4x base
        uint32_t disallowedClassMask = 0;      // 0 = nothing disallowed
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t houseId) const;

    static const char* factionAccessName(uint8_t f);
};

class WoweeAuctionLoader {
public:
    static bool save(const WoweeAuction& cat,
                     const std::string& basePath);
    static WoweeAuction load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-auction* variants.
    //
    //   makeStarter - 1 neutral house with stock 12h/24h/48h
    //                  duration tiers and 5% cut.
    //   makeFactionPair - Alliance + Horde + Neutral (3
    //                      houses) with the canonical
    //                      neutral 15% cut vs faction 5% cut
    //                      asymmetry.
    //   makeRestricted - 1 house that disallows Quest +
    //                     Container item classes (auction
    //                     house templates with tighter rules).
    static WoweeAuction makeStarter(const std::string& catalogName);
    static WoweeAuction makeFactionPair(const std::string& catalogName);
    static WoweeAuction makeRestricted(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
