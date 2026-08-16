#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Hearth Bind Point catalog (.whrt) - novel
// replacement for the hardcoded list of hearthstone bind
// locations used by the SMSG_BINDPOINTUPDATE flow. Each
// entry is one valid bind point: a tavern innkeeper, a
// special bind NPC (Karazhan port, Shattered Sun base,
// guild-hall bind clerk), or a city portal-room
// quartermaster. The client uses this catalog to pin
// hearth icons on the world map and to render the
// "Hearthstone bound to: <name>" tooltip text.
//
// Cross-references with previously-added formats:
//   WMS:  mapId references the WMS map; areaId references
//         the WMS sub-area entry.
//   WCRT: npcId references the innkeeper / bind NPC in
//         the WCRT creature catalog.
//   WCHC: faction filter uses the WCHC faction-mask bits
//         (1=Alliance, 2=Horde, 3=Both).
//
// Binary layout (little-endian):
//   magic[4]            = "WHRT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     bindId (uint32)
//     nameLen + name
//     descLen + description
//     mapId (uint32) / areaId (uint32)
//     x (float) / y (float) / z (float)
//     facing (float, radians)
//     npcId (uint32)            - 0 if no NPC bind clerk
//     factionMask (uint8)        - 1=A / 2=H / 3=Both
//     bindKind (uint8)           - Inn / Capital / Quest /
//                                  Guild / SpecialPort
//     levelMin (uint8)           - earliest level allowed
//                                  to bind here (0 = any)
//     pad0 (uint8)
//     iconColorRGBA (uint32)
struct WoweeHearthBinds {
    enum BindKind : uint8_t {
        Inn          = 0,    // tavern innkeeper
        Capital      = 1,    // city portal-room or
                              // capital-hall bind clerk
        Quest        = 2,    // quest-given bind reward
                              // (Theramore, Wyrmrest)
        Guild        = 3,    // guild-hall bind point
        SpecialPort  = 4,    // raid port (Karazhan,
                              // Karazhan Crypts, Tempest
                              // Keep)
        Faction      = 5,    // faction-base bind (SSO
                              // Sunwell, Argent Tournament)
    };

    enum FactionMask : uint8_t {
        AllianceOnly = 1,
        HordeOnly    = 2,
        Both         = 3,
    };

    struct Entry {
        uint32_t bindId = 0;
        std::string name;
        std::string description;
        uint32_t mapId = 0;
        uint32_t areaId = 0;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float facing = 0.0f;
        uint32_t npcId = 0;
        uint8_t factionMask = Both;
        uint8_t bindKind = Inn;
        uint8_t levelMin = 0;
        uint8_t pad0 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bindId) const;

    // Returns all bind points available to a player of the
    // given faction (Alliance=1, Horde=2). Bindings with
    // factionMask=3 (Both) are returned for either query.
    // Used by the world-map UI to filter the inn-icon
    // overlay layer per character.
    [[nodiscard]] std::vector<const Entry*> findByFaction(uint8_t playerFaction) const;

    // Returns all bind points within a given map (for the
    // continent-level inn overlay).
    [[nodiscard]] std::vector<const Entry*> findByMap(uint32_t mapId) const;
};

class WoweeHearthBindsLoader {
public:
    static bool save(const WoweeHearthBinds& cat,
                     const std::string& basePath);
    static WoweeHearthBinds load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-hrt* variants.
    //
    //   makeStarterCities - 4 entries (Stormwind / Ironforge
    //                        / Orgrimmar / Thunder Bluff
    //                        innkeepers, faction-gated).
    //   makeCapitals      - 6 entries (Stormwind / Ironforge
    //                        / Darnassus / Orgrimmar /
    //                        Undercity / Thunder Bluff
    //                        capital-hall bind clerks).
    //   makeStarterInns   - 8 entries (mix of starter-zone
    //                        inns: Goldshire / Brill /
    //                        Razor Hill / Bloodhoof Village
    //                        / Kharanos / Aldrassil /
    //                        Shadowglen / Sun Rock Retreat).
    static WoweeHearthBinds makeStarterCities(const std::string& catalogName);
    static WoweeHearthBinds makeCapitals(const std::string& catalogName);
    static WoweeHearthBinds makeStarterInns(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
