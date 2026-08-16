#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Pack catalog (.wspk) - novel
// replacement for the implicit per-class spellbook
// layout that vanilla WoW derived from
// SkillLineAbility.dbc + SpellTabIcon mappings + the
// hard-coded per-spec tab order baked into the client
// UI. Each WSPK entry binds one (classId, tabIndex)
// pair to an ordered list of spellIds shown in that
// spellbook tab.
//
// Cross-references with previously-added formats:
//   WSPL: spellIds in the ordered list are looked up
//         against WSPL spell catalog at runtime.
//   WCDB: classId references the playable-class
//         catalog (currently 1..11 in vanilla:
//         Warrior=1 ... Druid=11).
//
// Binary layout (little-endian):
//   magic[4]            = "WSPK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     packId (uint32)        - surrogate primary key
//                               for cross-format
//                               --catalog-find lookups
//     classId (uint8)        - 1..11 vanilla class
//     tabIndex (uint8)       - 0=General/3 spec tabs
//     iconIndex (uint8)      - SpellIcon row id for
//                               the tab header glyph
//     pad0 (uint8)
//     tabNameLen + tabName   - display label for the
//                               spellbook tab
//     spellCount (uint32)
//     spellIds (uint32 × count)  - ordered display
//                                   list (top-to-
//                                   bottom in tab)
struct WoweeSpellPack {
    struct Entry {
        uint32_t packId = 0;
        uint8_t classId = 0;
        uint8_t tabIndex = 0;
        uint8_t iconIndex = 0;
        uint8_t pad0 = 0;
        std::string tabName;
        std::vector<uint32_t> spellIds;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t packId) const;
    [[nodiscard]] const Entry* findByClassTab(uint8_t classId,
                                 uint8_t tabIndex) const;

    // Returns all packs for a class (typically 4: General
    // + 3 spec tabs). Used by the spellbook-screen UI to
    // populate per-class tab order.
    [[nodiscard]] std::vector<const Entry*> findByClass(uint8_t classId) const;
};

class WoweeSpellPackLoader {
public:
    static bool save(const WoweeSpellPack& cat,
                     const std::string& basePath);
    static WoweeSpellPack load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spk* variants.
    //
    //   makeWarriorPack - 4 tabs (General + 3 trees:
    //                      Arms/Fury/Protection). Each
    //                      tab seeded with canonical
    //                      vanilla spellIds.
    //   makeMagePack    - 4 tabs (General + Arcane/
    //                      Fire/Frost). Frost tab
    //                      includes Frostbolt rank-1
    //                      spellId 116 - the canonical
    //                      "every mage starts here" spell.
    //   makeRoguePack   - 4 tabs (General + Assassin/
    //                      Combat/Subtlety). Combat tab
    //                      seeded with poison-application
    //                      and lethality picks.
    static WoweeSpellPack makeWarriorPack(const std::string& catalogName);
    static WoweeSpellPack makeMagePack(const std::string& catalogName);
    static WoweeSpellPack makeRoguePack(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
