#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Material catalog (.wmat) - novel
// replacement for Blizzard's Material.dbc plus the
// Material/SheatheType fields in ItemDisplayInfo.dbc.
// Defines the material categorization that items
// reference (Cloth / Leather / Mail / Plate / Wood /
// Steel / Crystal / etc), each with its own foley sound
// (played on use), impact sound (played on drop / hit),
// weight category, and material-property flags
// (IsBreakable / IsMagical / IsFlammable / IsConductive).
//
// The engine plays a sword's metallic clang from
// impactSoundId when it hits a stone wall, but a cloth
// tabard makes no such sound - the difference is exactly
// the material assigned by this catalog. Every armor and
// weapon item in WIT references a materialId here.
//
// Cross-references with previously-added formats:
//   WSND: foleySoundId / impactSoundId reference WSND
//         sound entries.
//   WIT:  item entries reference materialId here.
//
// Binary layout (little-endian):
//   magic[4]            = "WMAT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     materialId (uint32)
//     nameLen + name
//     descLen + description
//     materialKind (uint8) / weightCategory (uint8) / pad[2]
//     foleySoundId (uint32)
//     impactSoundId (uint32)
//     materialFlags (uint32)
//     iconColorRGBA (uint32)
struct WoweeItemMaterial {
    enum MaterialKind : uint8_t {
        Cloth     = 0,
        Leather   = 1,
        Mail      = 2,
        Plate     = 3,
        Wood      = 4,
        Stone     = 5,
        Metal     = 6,
        Liquid    = 7,
        Organic   = 8,    // bone / hide / chitin
        Crystal   = 9,
        Ethereal  = 10,   // ghostly / energy / soul
        Hide      = 11,   // raw furred hide (distinct from Leather)
    };

    enum WeightCategory : uint8_t {
        Light  = 0,
        Medium = 1,
        Heavy  = 2,
    };

    enum MaterialFlag : uint32_t {
        IsBreakable    = 1u << 0,    // can be shattered by enough damage
        IsMagical      = 1u << 1,    // glows / has magical properties
        IsFlammable    = 1u << 2,    // ignites on fire damage
        IsConductive   = 1u << 3,    // amplifies lightning damage
        IsHolyCharged  = 1u << 4,    // damages undead on contact
        IsCursed       = 1u << 5,    // applies a debuff when equipped
    };

    struct Entry {
        uint32_t materialId = 0;
        std::string name;
        std::string description;
        uint8_t materialKind = Cloth;
        uint8_t weightCategory = Light;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t foleySoundId = 0;
        uint32_t impactSoundId = 0;
        uint32_t materialFlags = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t materialId) const;

    static const char* materialKindName(uint8_t k);
    static const char* weightCategoryName(uint8_t w);
};

class WoweeItemMaterialLoader {
public:
    static bool save(const WoweeItemMaterial& cat,
                     const std::string& basePath);
    static WoweeItemMaterial load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mat* variants.
    //
    //   makeArmor   - 5 armor-class materials (Cloth /
    //                  Leather / Mail / Plate / Hide)
    //                  with weight categories matching
    //                  WoW's armor classes.
    //   makeWeapon  - 5 weapon materials (Wood / Steel /
    //                  Mithril / Adamantite /
    //                  EnchantedSteel) covering the
    //                  vendor-buy through endgame
    //                  progression.
    //   makeMagical - 4 magical materials (Crystal /
    //                  Ethereal / Cursed / HolyForged)
    //                  carrying special flags
    //                  (IsMagical, IsCursed,
    //                  IsHolyCharged).
    static WoweeItemMaterial makeArmor(const std::string& catalogName);
    static WoweeItemMaterial makeWeapon(const std::string& catalogName);
    static WoweeItemMaterial makeMagical(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
