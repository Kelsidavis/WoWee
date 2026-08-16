#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Auction House Configuration catalog
// (.wauh) - novel replacement for the implicit
// per-faction auction-house policy vanilla WoW
// carried in AuctionHouse.dbc + the hard-coded
// deposit/cut rate constants in the server's
// AuctionMgr (the 5% Alliance/Horde rate vs 15%
// neutral Booty Bay rate was hard-coded on AH
// faction id). Each WAUH entry binds one auction
// house instance to its faction-access rules,
// deposit-rate (% of vendor sell price held as
// deposit), AH cut (% taken from final sale price
// before crediting seller), allowed listing
// durations, and the auctioneer NPC binding.
//
// Cross-references with previously-added formats:
//   WCRT: npcAuctioneerId references the WCRT
//         creature catalog (the actual NPC that
//         opens the AH UI when right-clicked).
//   WLOC: AH instances typically live at a POI in
//         WLOC (Stormwind AH, Orgrimmar AH, Booty
//         Bay AH).
//
// Binary layout (little-endian):
//   magic[4]            = "WAUH"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     ahId (uint32)
//     nameLen + name
//     factionAccess (uint8)        - 0=Both /
//                                     1=Alliance /
//                                     2=Horde /
//                                     3=Neutral
//     pad0 (uint8)
//     depositRatePct (uint16)      - basis points
//                                     0..10000 (% of
//                                     vendor sell
//                                     price held as
//                                     deposit)
//     cutPct (uint16)              - basis points
//                                     0..10000 (AH
//                                     cut from final
//                                     sale price)
//     minListingDurationHours (uint16)
//     maxListingDurationHours (uint16)
//     pad1 (uint16)
//     feePerSlot (uint32)          - flat copper fee
//                                     per listing
//                                     slot (0 = no
//                                     fee)
//     npcAuctioneerId (uint32)     - WCRT creature
//                                     entry
struct WoweeAuctionHouses {
    enum FactionAccess : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
        Neutral  = 3,
    };

    struct Entry {
        uint32_t ahId = 0;
        std::string name;
        uint8_t factionAccess = Both;
        uint8_t pad0 = 0;
        uint16_t depositRatePct = 0;
        uint16_t cutPct = 0;
        uint16_t minListingDurationHours = 0;
        uint16_t maxListingDurationHours = 0;
        uint16_t pad1 = 0;
        uint32_t feePerSlot = 0;
        uint32_t npcAuctioneerId = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t ahId) const;

    // Returns the AH entry an auctioneer NPC opens
    // when right-clicked. Used by the gossip handler
    // to dispatch to the correct AH config.
    [[nodiscard]] const Entry* findByNpc(uint32_t npcId) const;

    // Returns all AH entries accessible to a faction.
    // Used by the AH-finder UI to suggest reachable
    // auctioneers.
    [[nodiscard]] std::vector<const Entry*> findByFaction(uint8_t faction) const;
};

class WoweeAuctionHousesLoader {
public:
    static bool save(const WoweeAuctionHouses& cat,
                     const std::string& basePath);
    static WoweeAuctionHouses load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-auh* variants.
    //
    //   makeStormwindAH   - Alliance Stormwind AH
    //                        with vanilla rates (5%
    //                        deposit / 5% cut / 12-48
    //                        hr listing).
    //   makeOrgrimmarAH   - Horde Orgrimmar AH with
    //                        same vanilla rates as
    //                        Stormwind. Demonstrates
    //                        a paired faction-AH set.
    //   makeBootyBayAH    - Neutral Booty Bay AH with
    //                        the famous 15% deposit
    //                        + 15% cut neutral rates
    //                        (penalty for cross-
    //                        faction trade).
    static WoweeAuctionHouses makeStormwindAH(const std::string& catalogName);
    static WoweeAuctionHouses makeOrgrimmarAH(const std::string& catalogName);
    static WoweeAuctionHouses makeBootyBayAH(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
