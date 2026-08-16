#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Character Customization Feature catalog (.wchf)
// - novel replacement for Blizzard's CharHairGeosets.dbc +
// CharFacialHairStyles.dbc plus the variation portions of
// CharSections.dbc. Defines the per-(race, sex) customization
// options that the character creation screen exposes:
// skin colors, face variations, hair styles, hair colors,
// facial hair (beards / mustaches), and race-specific
// markings (Tauren horns, Draenei tendrils, Blood Elf ears).
//
// Each entry describes ONE selectable customization choice
// for ONE (race, sex, featureKind) tuple - variationIndex
// disambiguates between options of the same kind. The
// character renderer reads this catalog at the create-
// character screen to populate the variant carousels and at
// world-load to assemble the player's textures + geosets.
//
// Cross-references with previously-added formats:
//   WCHF.entry.raceId          → WCHC.race.raceId
//   WCHF.entry.requiresExpansion bit positions match the
//                                expansion enum used by WLFG
//                                (Classic=0, TBC=1, WotLK=2,
//                                Turtle=3).
//
// Binary layout (little-endian):
//   magic[4]            = "WCHF"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     featureId (uint32)
//     raceId (uint32)
//     nameLen + name
//     descLen + description
//     texLen + texturePath
//     featureKind (uint8) / sexId (uint8) /
//       variationIndex (uint8) / requiresExpansion (uint8)
//     geosetGroupBits (uint32)
//     hairColorOverlayId (uint32)
struct WoweeCharFeature {
    enum FeatureKind : uint8_t {
        SkinColor      = 0,    // base body skin tone
        FaceVariation  = 1,    // facial features (eyes, nose)
        HairStyle      = 2,    // hairstyle geoset
        HairColor      = 3,    // hair texture overlay
        FacialHair     = 4,    // beard / mustache geoset
        FacialColor    = 5,    // facial hair color
        EarStyle       = 6,    // race-specific ear variations
        Horns          = 7,    // Tauren / Draenei horns
        Markings       = 8,    // tribal paint / scars / freckles
    };

    enum SexId : uint8_t {
        Male   = 0,
        Female = 1,
    };

    enum ExpansionGate : uint8_t {
        Classic   = 0,
        TBC       = 1,    // unlocks Blood Elf + Draenei options
        WotLK     = 2,    // unlocks Death Knight starting features
        TurtleWoW = 3,    // custom-server additions
    };

    struct Entry {
        uint32_t featureId = 0;
        uint32_t raceId = 0;             // WCHC cross-ref
        std::string name;
        std::string description;
        std::string texturePath;         // body / face / hair texture
        uint8_t featureKind = SkinColor;
        uint8_t sexId = Male;
        uint8_t variationIndex = 0;      // 0..N within (race, sex, kind)
        uint8_t requiresExpansion = Classic;
        uint32_t geosetGroupBits = 0;    // M2 geoset enable mask
        uint32_t hairColorOverlayId = 0; // 0 = no overlay
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t featureId) const;

    static const char* featureKindName(uint8_t k);
    static const char* sexIdName(uint8_t s);
    static const char* expansionGateName(uint8_t e);
};

class WoweeCharFeatureLoader {
public:
    static bool save(const WoweeCharFeature& cat,
                     const std::string& basePath);
    static WoweeCharFeature load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-chf* variants.
    //
    //   makeStarter        - 5 Human Male options covering
    //                         the canonical kind triad (1 skin
    //                         color, 1 face, 2 hair styles,
    //                         1 facial hair).
    //   makeBloodElfFemale - 8 Blood Elf Female hair styles
    //                         (TBC iconic feature, requires
    //                         expansion=TBC).
    //   makeTauren         - 6 Tauren Male features (3 horn
    //                         variations + 3 facial hair -
    //                         Tauren get hair on their face
    //                         and chin and have horn variants
    //                         instead of EarStyle).
    static WoweeCharFeature makeStarter(const std::string& catalogName);
    static WoweeCharFeature makeBloodElfFemale(const std::string& catalogName);
    static WoweeCharFeature makeTauren(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
