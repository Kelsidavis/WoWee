#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Tabard Design catalog (.wtbd) - novel
// replacement for the GuildBankTabard / TabardConfig
// blob that vanilla WoW stores per-guild in the
// guild_member SQL table. Each entry is one tabard
// design: a triplet of (background pattern + color,
// border pattern + color, emblem glyph + color), plus
// optional guild and creator-player attribution and a
// server-approval flag for tabard-moderation policies.
//
// Cross-references with previously-added formats:
//   WGLD: guildId references the WGLD guild catalog
//         (0 if standalone / system tabard).
//   WPSP: creatorPlayerId references the WPSP player
//         spawn profile catalog (the designer).
//
// Binary layout (little-endian):
//   magic[4]            = "WTBD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tabardId (uint32)
//     nameLen + name
//     descLen + description
//     backgroundPattern (uint8)  - Solid / Gradient /
//                                   Chevron / Quartered /
//                                   Starburst
//     borderPattern (uint8)      - None / Thin / Thick /
//                                   Decorative
//     emblemId (uint16)          - 0..1023 glyph index
//     backgroundColor (uint32)   - RGBA
//     borderColor (uint32)       - RGBA
//     emblemColor (uint32)       - RGBA
//     guildId (uint32)            - 0 if standalone
//     creatorPlayerId (uint32)    - 0 if system tabard
//     isApproved (uint8)          - 0/1 bool
//     pad0 (uint8) / pad1 (uint8) / pad2 (uint8)
//     iconColorRGBA (uint32)
struct WoweeTabards {
    enum BackgroundPattern : uint8_t {
        Solid     = 0,    // single color filling the whole
                           // background
        Gradient  = 1,    // 2-color top-to-bottom gradient
        Chevron   = 2,    // V-shaped band
        Quartered = 3,    // 4 quadrants of alternating
                           // color
        Starburst = 4,    // radial sunburst pattern
    };

    enum BorderPattern : uint8_t {
        BorderNone        = 0,
        BorderThin        = 1,
        BorderThick       = 2,
        BorderDecorative  = 3,    // ornamental knotwork /
                                   // braid
    };

    struct Entry {
        uint32_t tabardId = 0;
        std::string name;
        std::string description;
        uint8_t backgroundPattern = Solid;
        uint8_t borderPattern = BorderThin;
        uint16_t emblemId = 0;
        uint32_t backgroundColor = 0xFF000000u;   // black
        uint32_t borderColor = 0xFFFFFFFFu;        // white
        uint32_t emblemColor = 0xFFFFFFFFu;        // white
        uint32_t guildId = 0;
        uint32_t creatorPlayerId = 0;
        uint8_t isApproved = 1;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tabardId) const;

    // Returns all tabards belonging to one guild - used
    // by the guild-bank tabard preview UI to populate
    // the design-history list (guilds can keep multiple
    // approved designs and switch between them).
    [[nodiscard]] std::vector<const Entry*> findByGuild(uint32_t guildId) const;

    // Returns all approved tabards (isApproved=1). Server
    // tabard-moderation policy may hide unapproved
    // designs from the public picker until reviewed.
    [[nodiscard]] std::vector<const Entry*> findApproved() const;
};

class WoweeTabardsLoader {
public:
    static bool save(const WoweeTabards& cat,
                     const std::string& basePath);
    static WoweeTabards load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tbd* variants.
    //
    //   makeAllianceClassic - 4 Alliance-themed system
    //                          tabards (LionEmblem on blue
    //                          / HammerEmblem on silver /
    //                          AnchorEmblem on navy /
    //                          SwordEmblem on gold).
    //   makeHordeClassic    - 4 Horde-themed system
    //                          tabards (WolfheadEmblem on
    //                          crimson / CrossedAxes on
    //                          dark / SkullEmblem on
    //                          black / PyramidEmblem on
    //                          tan).
    //   makeFactionVendor   - 6 faction-rep tabards
    //                          (Argent Crusade /
    //                          Ebon Blade / Sons of
    //                          Hodir / Wyrmrest Accord /
    //                          Kaluak / Frenzyheart).
    static WoweeTabards makeAllianceClassic(const std::string& catalogName);
    static WoweeTabards makeHordeClassic(const std::string& catalogName);
    static WoweeTabards makeFactionVendor(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
