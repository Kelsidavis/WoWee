#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Glyph catalog (.wgly) - novel replacement for
// Blizzard's GlyphProperties.dbc + GlyphSlot.dbc plus the
// AzerothCore-style glyph_properties SQL tables. The 42nd
// open format added to the editor.
//
// Defines the WotLK-introduced glyph system: per-class
// inscribable glyphs that modify spell behavior. Each entry
// pairs a glyph item (the inscription crafted by an inscriber)
// with the spell aura that applies the modification, and tags
// it with a glyph slot type (Major / Minor / Prime) and a
// classMask of allowed classes.
//
// Cross-references with previously-added formats:
//   WGLY.entry.spellId  → WSPL.spellId   (the aura applied)
//   WGLY.entry.itemId   → WIT.itemId     (the inscribed glyph
//                                          item that teaches it)
//   WGLY.entry.classMask bit positions match WCHC.class.classId
//                                          (1=Warrior, 2=Paladin,
//                                           3=Hunter, 4=Rogue, ...)
//
// Binary layout (little-endian):
//   magic[4]            = "WGLY"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     glyphId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     glyphType (uint8) / pad[3]
//     spellId (uint32)
//     itemId (uint32)
//     classMask (uint32)
//     requiredLevel (uint16) / pad[2]
struct WoweeGlyph {
    enum GlyphType : uint8_t {
        Major = 0,    // Major glyph slot - gameplay-defining
        Minor = 1,    // Minor glyph slot - convenience / cosmetic
        Prime = 2,    // Prime slot (Cataclysm-style backport)
    };

    // Convenient class-bit constants matching WCHC.classId.
    static constexpr uint32_t kClassWarrior     = 1u << 1;
    static constexpr uint32_t kClassPaladin     = 1u << 2;
    static constexpr uint32_t kClassHunter      = 1u << 3;
    static constexpr uint32_t kClassRogue       = 1u << 4;
    static constexpr uint32_t kClassPriest      = 1u << 5;
    static constexpr uint32_t kClassDeathKnight = 1u << 6;
    static constexpr uint32_t kClassShaman      = 1u << 7;
    static constexpr uint32_t kClassMage        = 1u << 8;
    static constexpr uint32_t kClassWarlock     = 1u << 9;
    static constexpr uint32_t kClassDruid       = 1u << 11;
    static constexpr uint32_t kClassAll         = 0xFFFFFFFFu;

    struct Entry {
        uint32_t glyphId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t glyphType = Major;
        uint32_t spellId = 0;       // WSPL cross-ref
        uint32_t itemId = 0;        // WIT cross-ref (inscribed glyph item)
        uint32_t classMask = 0;     // bit per WCHC.classId
        uint16_t requiredLevel = 25;  // WotLK glyphs unlock at 25/50/70/80
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t glyphId) const;

    static const char* glyphTypeName(uint8_t t);
};

class WoweeGlyphLoader {
public:
    static bool save(const WoweeGlyph& cat,
                     const std::string& basePath);
    static WoweeGlyph load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-glyphs* variants.
    //
    //   makeStarter - 3 glyphs (1 warrior, 1 mage, 1 rogue)
    //                  showing a representative Major slot
    //                  pick per role.
    //   makeWarrior - 6 warrior glyphs (3 major + 3 minor)
    //                  demonstrating a per-class allotment.
    //   makeUniversal - 4 glyphs with classMask=kClassAll
    //                    (utility / generic effects that any
    //                    class can inscribe).
    static WoweeGlyph makeStarter(const std::string& catalogName);
    static WoweeGlyph makeWarrior(const std::string& catalogName);
    static WoweeGlyph makeUniversal(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
