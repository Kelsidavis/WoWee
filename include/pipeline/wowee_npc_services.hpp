#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open NPC Service Definition catalog (.wbkd) -
// novel replacement for AzerothCore's npc_vendor /
// npc_trainer / npc_gossip / npc_options SQL tables plus
// the engine's hard-coded service-type dispatch. Defines
// the kinds of services NPCs can offer (Banker / Mailbox
// / Auctioneer / StableMaster / FlightMaster / Trainer /
// Innkeeper / Battlemaster / etc) and the per-service
// metadata (gold cost, faction gating, gossip text).
//
// When a player right-clicks an NPC, the engine looks
// at the NPC's serviceId list (from WCRT.npcFlags or
// equivalent) and dispatches to the appropriate
// service-frame handler - Banker opens the inventory
// expansion frame, Auctioneer opens the auction house,
// StableMaster opens the pet stable. This catalog
// defines what each service actually does and what
// preconditions it requires.
//
// Cross-references with previously-added formats:
//   WCRT: creature.npcFlags decodes into a list of
//         service ids defined here.
//   WFAC: factionRequiredId references WFAC.factionId
//         for rep-gated services (Argent Tournament
//         vendor only sells to Honored+).
//   WGSP: gossipTextId references WGSP.menuId for the
//         "How can I help you?" dialogue line.
//
// Binary layout (little-endian):
//   magic[4]            = "WBKD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     serviceId (uint32)
//     nameLen + name
//     descLen + description
//     serviceKind (uint8) / pad[3]
//     requiresGold (uint32)
//     factionRequiredId (uint32)
//     gossipTextId (uint32)
//     iconColorRGBA (uint32)
struct WoweeNPCService {
    enum ServiceKind : uint8_t {
        Banker          = 0,    // opens bank inventory frame
        Mailbox         = 1,    // mail send/receive frame
        Auctioneer      = 2,    // auction house frame
        StableMaster    = 3,    // hunter pet stable frame
        FlightMaster    = 4,    // taxi node selection frame
        Trainer         = 5,    // class/profession trainer frame
        Innkeeper       = 6,    // hearthstone bind point + bed
        Battlemaster    = 7,    // BG queue frame
        GuildBanker     = 8,    // guild bank frame (TBC+)
        ReagentVendor   = 9,    // reagent purchase
        TabardVendor    = 10,   // guild tabard customization
        Misc            = 11,   // catch-all
    };

    struct Entry {
        uint32_t serviceId = 0;
        std::string name;
        std::string description;
        uint8_t serviceKind = Banker;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t requiresGold = 0;       // 1g = 10000c
        uint32_t factionRequiredId = 0;  // 0 = no faction gate
        uint32_t gossipTextId = 0;       // 0 = no custom dialogue
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t serviceId) const;

    // Return all services of a given kind across the
    // catalog. Used by NPC-spawning code to find e.g.
    // "all FlightMaster services configured for this
    // server" when populating taxi nodes.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t kind) const;

    static const char* serviceKindName(uint8_t k);
};

class WoweeNPCServiceLoader {
public:
    static bool save(const WoweeNPCService& cat,
                     const std::string& basePath);
    static WoweeNPCService load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bkd* variants.
    //
    //   makeCity     - 5 city services (Banker / Mailbox /
    //                   Innkeeper / Auctioneer /
    //                   FlightMaster) typically present in
    //                   a capital city like Stormwind or
    //                   Orgrimmar.
    //   makeBattle   - 3 battlemaster services (Alterac /
    //                   Warsong / Arathi) for queueing into
    //                   each Vanilla battleground.
    //   makeProfession - 4 profession services (Blacksmith
    //                   Trainer / Tailoring Trainer /
    //                   Reagent Vendor / Stable Master)
    //                   typical of profession districts.
    static WoweeNPCService makeCity(const std::string& catalogName);
    static WoweeNPCService makeBattle(const std::string& catalogName);
    static WoweeNPCService makeProfession(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
