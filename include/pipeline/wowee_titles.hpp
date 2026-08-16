#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Title catalog (.wtit) - novel replacement for
// Blizzard's CharTitles.dbc + the AzerothCore-style
// character_title SQL table. The 30th open format added to
// the editor.
//
// Defines the player-display titles awarded for completing
// achievements ("the Versatile"), reaching PvP ranks
// ("Sergeant Major"), participating in raids ("Champion of
// the Naaru"), levelling a class ("Master Locksmith"), or
// participating in seasonal events ("Brewmaster", "the
// Hallowed").
//
// Cross-references with previously-added formats:
//   WACH.entry.titleReward (string)  ≈ WTIT.entry.name
//                                      (string match - the
//                                       runtime resolves
//                                       achievement-granted
//                                       titles by looking up
//                                       the matching WTIT
//                                       entry by name)
//
// Binary layout (little-endian):
//   magic[4]            = "WTIT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     titleId (uint32)
//     nameLen + name              -- canonical / English form
//     maleLen + nameMale          -- empty = use canonical
//     femaleLen + nameFemale      -- empty = use canonical
//     iconLen + iconPath
//     prefix (uint8)              -- 0=suffix, 1=prefix
//     category (uint8)
//     sortOrder (uint16)
struct WoweeTitle {
    enum Category : uint8_t {
        Achievement = 0,
        Pvp         = 1,
        Raid        = 2,
        ClassTitle  = 3,
        Event       = 4,
        Profession  = 5,
        Lore        = 6,
        Custom      = 7,
    };

    struct Entry {
        uint32_t titleId = 0;
        std::string name;             // canonical (genderless)
        std::string nameMale;         // empty = use canonical
        std::string nameFemale;
        std::string iconPath;
        uint8_t prefix = 0;            // 0 = "Name the Versatile"
        uint8_t category = Achievement;
        uint16_t sortOrder = 0;        // display order in UI
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t titleId) const;
    // String match against the canonical name - used to
    // resolve WACH.titleReward references.
    [[nodiscard]] const Entry* findByName(const std::string& name) const;

    static const char* categoryName(uint8_t c);
};

class WoweeTitleLoader {
public:
    static bool save(const WoweeTitle& cat,
                     const std::string& basePath);
    static WoweeTitle load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-titles* variants.
    //
    //   makeStarter - 4 titles covering Achievement / Pvp /
    //                  Raid / Event categories.
    //   makePvp     - 14-rank Honor System ladder
    //                  (Private/Corporal/...Knight-Lieutenant/
    //                  ...Field Marshal) with both Alliance
    //                  and Horde titles where they differ.
    //   makeAchievement - titles granted by the WACH meta
    //                      preset including "the Versatile"
    //                      from achievement 250.
    static WoweeTitle makeStarter(const std::string& catalogName);
    static WoweeTitle makePvp(const std::string& catalogName);
    static WoweeTitle makeAchievement(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
