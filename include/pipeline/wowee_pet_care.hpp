#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Pet Care & Action catalog (.wpcr) - novel
// replacement for the implicit pet-management action
// rules vanilla WoW scattered across spell_template
// (Revive Pet / Mend Pet / Feed Pet / Dismiss Pet
// definitions), npc_text (stable master gossip), and
// per-class trainer SQL. Each entry binds one pet
// management action (Revive, Feed, Stable Slot,
// Untrain, etc.) to its dispatching spell, gold cost,
// reagent requirement, cast time, cooldown, and
// pet/NPC pre-conditions.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell catalog
//         (the actual spell the client casts when the
//         action button is pressed).
//   WCHC: classFilter uses the WCHC class-bit
//         convention (4=Hunter, 256=Warlock, 64=Shaman
//         for elementals, etc.).
//   WIT:  reagentItemId references the WIT item catalog
//         (Mend Pet has no reagent, Feed Pet requires
//         food matching the pet's diet, Tame Beast has
//         no reagent but consumes a tame slot).
//   WPET: actions operate on the active pet from the
//         WPET pet catalog.
//
// Binary layout (little-endian):
//   magic[4]            = "WPCR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     actionId (uint32)
//     nameLen + name
//     descLen + description
//     spellId (uint32)
//     classFilter (uint32)
//     actionKind (uint8)         - Revive / Mend / Feed
//                                   / Dismiss / Tame /
//                                   BeastLore / Stable /
//                                   Untrain / Rename /
//                                   Abandon / Summon
//     happinessRestore (int8)    - typically +10 (Feed),
//                                   -10 (Abandon),
//                                   0 (no effect)
//     requiresPet (uint8)        - 0/1 bool
//     requiresStableNPC (uint8)  - 0/1 bool
//     costCopper (uint32)        - 100 = 1 silver,
//                                   10000 = 1 gold
//     reagentItemId (uint32)     - 0 if no reagent
//     castTimeMs (uint32)
//     cooldownSec (uint32)
//     iconColorRGBA (uint32)
struct WoweePetCare {
    enum ActionKind : uint8_t {
        Revive    = 0,    // spell 982
        Mend      = 1,    // spell 136 (channeled HoT)
        Feed      = 2,    // spell 6991 (happiness +)
        Dismiss   = 3,    // spell 2641 (instant despawn)
        Tame      = 4,    // spell 1515 (channeled tame)
        BeastLore = 5,    // spell 1462 (inspect)
        Stable    = 6,    // 50s per slot up to 5
        Untrain   = 7,    // gold ramp 10c..10g
        Rename    = 8,    // free, instant
        Abandon   = 9,    // permanent, free
        Summon    = 10,   // Warlock minion summons
    };

    struct Entry {
        uint32_t actionId = 0;
        std::string name;
        std::string description;
        uint32_t spellId = 0;
        uint32_t classFilter = 0;
        uint8_t actionKind = Revive;
        int8_t happinessRestore = 0;
        uint8_t requiresPet = 1;
        uint8_t requiresStableNPC = 0;
        uint32_t costCopper = 0;
        uint32_t reagentItemId = 0;
        uint32_t castTimeMs = 0;
        uint32_t cooldownSec = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t actionId) const;

    // Returns all actions available to a specific class
    // bit (4=Hunter, 256=Warlock). Used by the action-bar
    // UI to populate the pet-actions tab per character.
    [[nodiscard]] std::vector<const Entry*> findByClass(uint32_t classBit) const;

    // Returns all actions of one kind - used by the
    // stable-master NPC interaction handler to find which
    // actions become available when the player talks to
    // a stable master.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t actionKind) const;
};

class WoweePetCareLoader {
public:
    static bool save(const WoweePetCare& cat,
                     const std::string& basePath);
    static WoweePetCare load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-pcr* variants.
    //
    //   makeHunterCare     - 5 Hunter pet care actions
    //                         (Revive / Mend / Feed /
    //                         Dismiss / Tame).
    //   makeStableActions  - 4 stable-master actions
    //                         (Stable Slots / Untrain /
    //                         Rename / Abandon).
    //   makeWarlockMinions - 4 Warlock minion summons
    //                         (Summon Imp / Voidwalker /
    //                         Succubus / Felhunter).
    static WoweePetCare makeHunterCare(const std::string& catalogName);
    static WoweePetCare makeStableActions(const std::string& catalogName);
    static WoweePetCare makeWarlockMinions(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
