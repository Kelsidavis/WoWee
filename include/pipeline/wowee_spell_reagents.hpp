#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Reagent catalog (.wspr) - novel
// replacement for the per-spell reagent fields in
// Spell.dbc (Reagent[8] + ReagentCount[8]). Defines the
// item reagents that a spell consumes from the caster's
// inventory each time it's cast - Mage Portal needs a
// Rune of Portals, Resurrection needs an Ash of Eternity,
// Warlock summons consume Soul Shards.
//
// One entry per spell that has reagents - most spells
// have none and are absent from this catalog. Each entry
// can list up to 8 (itemId, count) pairs which all must
// be present for the spell to cast. The engine deducts
// them on cast resolution.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell entry.
//   WIT:  every reagentItem*Id references a WIT item
//         entry. The item's own consumeOnUse flag should
//         be set (or the engine defaults to consuming
//         since the spell explicitly references it).
//
// Binary layout (little-endian):
//   magic[4]            = "WSPR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     reagentSetId (uint32)
//     nameLen + name
//     descLen + description
//     spellId (uint32)
//     reagentItemId[8] (uint32 each)
//     reagentCount[8] (uint32 each)
//     reagentKind (uint8) / pad[3]
//     iconColorRGBA (uint32)
struct WoweeSpellReagent {
    enum ReagentKind : uint8_t {
        Standard      = 0,    // ordinary item reagent (most spells)
        SoulShard     = 1,    // warlock-specific shard consumption
        FocusedItem   = 2,    // not consumed; just required to cast
                              // (Symbol of Divinity for Resurrection)
        Catalyst      = 3,    // one reagent enables stronger version
        Tradeable     = 4,    // crafting reagent (Trade Skill recipes)
    };

    static constexpr int kMaxReagentSlots = 8;

    struct Entry {
        uint32_t reagentSetId = 0;
        std::string name;
        std::string description;
        uint32_t spellId = 0;
        uint32_t reagentItemId[kMaxReagentSlots] = {};
        uint32_t reagentCount[kMaxReagentSlots] = {};
        uint8_t reagentKind = Standard;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t reagentSetId) const;
    [[nodiscard]] const Entry* findBySpell(uint32_t spellId) const;

    // Count the slots actually used (slots whose itemId is
    // non-zero). Most reagent sets use only 1-2 slots.
    [[nodiscard]] int usedSlotCount(uint32_t reagentSetId) const;

    static const char* reagentKindName(uint8_t k);
};

class WoweeSpellReagentLoader {
public:
    static bool save(const WoweeSpellReagent& cat,
                     const std::string& basePath);
    static WoweeSpellReagent load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spr* variants.
    //
    //   makeMage     - 4 mage portal/teleport reagents
    //                   (Stormwind, Ironforge, Darnassus,
    //                   Theramore) consuming Rune of
    //                   Teleportation x1.
    //   makeWarlock  - 4 warlock summons consuming Soul
    //                   Shards (Voidwalker, Imp, Succubus,
    //                   Felhunter) - each takes 1 shard.
    //   makeRez      - 3 resurrection reagents (Ankh of
    //                   Reincarnation, Ash of Eternity for
    //                   priest mass-rez, druid Rebirth
    //                   no-cost). Mix of Standard,
    //                   FocusedItem, and no-reagent kinds.
    static WoweeSpellReagent makeMage(const std::string& catalogName);
    static WoweeSpellReagent makeWarlock(const std::string& catalogName);
    static WoweeSpellReagent makeRez(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
