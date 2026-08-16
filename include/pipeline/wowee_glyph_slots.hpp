#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Glyph Slot catalog (.wgfs) - novel
// replacement for Blizzard's GlyphSlot.dbc. Defines the
// per-class glyph slot layout: which slots a class
// has (Major / Minor / Prime), in which display order
// they appear in the spellbook UI, and at which character
// level each slot becomes available for use.
//
// Distinct from WGLY (GlyphProperties), which defines the
// individual glyphs themselves. WGLY says "Glyph of
// Polymorph exists, costs 1 inscription dust, modifies
// Polymorph"; WGFS says "the slot that holds Glyph of
// Polymorph is the second Major Glyph Slot, unlocks at
// level 25, and only Mages have it".
//
// Layout grew across expansions:
//   Wrath of the Lich King - 3 Major + 3 Minor (6 slots)
//   Cataclysm              - 3 Prime + 3 Major + 3 Minor (9 slots)
// The presets cover both, plus a starter Classic-style
// "any class" 6-slot layout for the simplest case.
//
// Cross-references with previously-added formats:
//   WGLY: glyph entries reference slotKind to constrain
//         which glyph fits which slot kind.
//   WCHC: requiredClassMask uses the same bit layout as
//         WCHC class IDs (Warrior=0x01, Paladin=0x02, ...).
//
// Binary layout (little-endian):
//   magic[4]            = "WGFS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     slotId (uint32)
//     nameLen + name
//     descLen + description
//     slotKind (uint8) / displayOrder (uint8)
//     minLevelToUnlock (uint8) / pad (uint8)
//     requiredClassMask (uint32)
//     iconColorRGBA (uint32)
struct WoweeGlyphSlot {
    enum SlotKind : uint8_t {
        Major = 0,    // class-defining utility / mechanic glyphs
        Minor = 1,    // cosmetic / convenience glyphs
        Prime = 2,    // damage-output glyphs (Cata+)
    };

    struct Entry {
        uint32_t slotId = 0;
        std::string name;
        std::string description;
        uint8_t slotKind = Major;
        uint8_t displayOrder = 0;
        uint8_t minLevelToUnlock = 25;
        uint8_t pad0 = 0;
        uint32_t requiredClassMask = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t slotId) const;

    // Returns true if a character of the given class+level
    // has access to this slot. classBit is one of the WCHC
    // class-bit flags (Warrior=0x01, Paladin=0x02, ...).
    [[nodiscard]] bool isUnlockedFor(uint32_t slotId,
                        uint32_t classBit,
                        uint8_t characterLevel) const;

    static const char* slotKindName(uint8_t k);
};

class WoweeGlyphSlotLoader {
public:
    static bool save(const WoweeGlyphSlot& cat,
                     const std::string& basePath);
    static WoweeGlyphSlot load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-gfs* variants.
    //
    //   makeStarter - 6 slots: 3 Major + 3 Minor available
    //                  to every class (classMask=0xFFFFFFFF),
    //                  unlocking at 25/50/75 for each kind.
    //                  Simplest baseline layout.
    //   makeWotlk   - 6 slots: 3 Major + 3 Minor matching
    //                  the WotLK 3.3.5a layout (any class).
    //                  Major unlocks at 15/30/50, Minor at
    //                  15/50/70.
    //   makeCata    - 9 slots: 3 Prime + 3 Major + 3 Minor
    //                  matching the Cataclysm layout. Prime
    //                  unlocks at 25/50/75, Major at 25/50/75,
    //                  Minor at 25/50/75.
    static WoweeGlyphSlot makeStarter(const std::string& catalogName);
    static WoweeGlyphSlot makeWotlk(const std::string& catalogName);
    static WoweeGlyphSlot makeCata(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
