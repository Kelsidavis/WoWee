#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Mage Portal Destinations catalog
// (.wprt) - novel replacement for the implicit
// portal-spell -> destination-coordinate binding
// vanilla WoW carried in scattered pieces:
// SpellEffects.dbc effect-71 (TELEPORT_UNITS) +
// per-spell hard-coded destination tables in the
// server's SpellMgr + AreaTrigger.dbc destination
// rows. Each WPRT entry binds one Teleport/Portal
// spellId to its destination world coords, faction
// access gate, level requirement, and reagent
// requirement.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell catalog
//         (the actual spell-to-cast - Portal: Stormwind
//         is spellId 10059, Teleport: Stormwind is
//         3561, etc.).
//   WMS:  destinationMapId references the WMS map
//         catalog.
//   WIT:  reagentItemId references the WIT item
//         catalog (Rune of Teleportation = 17031,
//         Rune of Portals = 17032).
//
// Binary layout (little-endian):
//   magic[4]            = "WPRT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     portalId (uint32)
//     spellId (uint32)             - WSPL spell to
//                                     cast
//     nameLen + destinationName    - display label
//     destX (float)
//     destY (float)
//     destZ (float)
//     destOrientation (float)      - facing direction
//                                     in radians
//     destMapId (uint32)
//     factionAccess (uint8)        - 0=Both /
//                                     1=Alliance /
//                                     2=Horde /
//                                     3=Neutral
//     portalKind (uint8)           - 0=Teleport (self
//                                     only) /
//                                     1=Portal (group)
//     levelRequirement (uint8)     - minimum mage
//                                     level to learn
//     reagentCount (uint8)
//     reagentItemId (uint32)       - 0 if no reagent
struct WoweeMagePortals {
    enum FactionAccess : uint8_t {
        Both     = 0,
        Alliance = 1,
        Horde    = 2,
        Neutral  = 3,
    };

    enum PortalKind : uint8_t {
        Teleport = 0,    // self-only, lower mana cost
                          //  + reagent
        Portal   = 1,    // group portal, costs Rune
                          //  of Portals
    };

    struct Entry {
        uint32_t portalId = 0;
        uint32_t spellId = 0;
        std::string destinationName;
        float destX = 0.f;
        float destY = 0.f;
        float destZ = 0.f;
        float destOrientation = 0.f;
        uint32_t destMapId = 0;
        uint8_t factionAccess = Both;
        uint8_t portalKind = Teleport;
        uint8_t levelRequirement = 0;
        uint8_t reagentCount = 0;
        uint32_t reagentItemId = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t portalId) const;

    // Returns the binding for a given cast spellId -
    // the lookup the portal-cast handler uses to
    // decide where to teleport the target.
    [[nodiscard]] const Entry* findBySpellId(uint32_t spellId) const;

    // Returns all portals accessible to a faction.
    // Used by the spellbook UI to filter the mage
    // portal tab (Alliance mages don't see Horde
    // city portals).
    [[nodiscard]] std::vector<const Entry*> findByFaction(uint8_t faction) const;
};

class WoweeMagePortalsLoader {
public:
    static bool save(const WoweeMagePortals& cat,
                     const std::string& basePath);
    static WoweeMagePortals load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-prt* variants.
    //
    //   makeAllianceCities - 4 Alliance city portals
    //                          (Stormwind / Ironforge /
    //                          Darnassus / Theramore).
    //                          All Alliance-only,
    //                          require Rune of Portals.
    //   makeHordeCities    - 3 Horde city portals
    //                          (Orgrimmar / Undercity /
    //                          Thunder Bluff). Horde-
    //                          only.
    //   makeTeleports      - 3 self-teleport spells
    //                          (Teleport: Stormwind /
    //                          Teleport: Ironforge /
    //                          Teleport: Orgrimmar) -
    //                          paired Alliance/Horde
    //                          set illustrating the
    //                          self-vs-group portal
    //                          distinction.
    static WoweeMagePortals makeAllianceCities(const std::string& catalogName);
    static WoweeMagePortals makeHordeCities(const std::string& catalogName);
    static WoweeMagePortals makeTeleports(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
