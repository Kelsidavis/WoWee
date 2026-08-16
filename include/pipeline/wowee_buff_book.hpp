#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Buff & Aura Book catalog (.wbab) - novel
// replacement for the implicit rank-chain relationships
// that vanilla WoW encoded by burying nextRank/prevRank
// pointers inside Spell.dbc. Each entry is one long-
// duration class buff (Mark of the Wild, Arcane
// Intellect, Power Word: Fortitude, Battle Shout, etc.)
// at one specific rank, with explicit edges to the
// previous and next ranks via previousRankId /
// nextRankId.
//
// The rank-chain pattern is novel among the catalog set:
// most catalogs have flat entries (one row per concept);
// WBAB is a graph where rows are nodes connected by edge
// fields. Both directions are stored explicitly so the
// rank-step UI ("upgrade to next rank" button) can
// traverse without scanning the full table.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell catalog (the
//         actual spell that gets cast).
//   WCHC: castClassMask uses the WCHC class-bit
//         convention.
//   WBAB: previousRankId / nextRankId reference OTHER
//         entries in the same WBAB catalog - internal
//         self-reference for the rank chain. Validator
//         can check the back-edges (if A.next=B then
//         B.prev should = A).
//
// Binary layout (little-endian):
//   magic[4]            = "WBAB"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     buffId (uint32)
//     nameLen + name
//     descLen + description
//     spellId (uint32)
//     castClassMask (uint32)
//     targetTypeMask (uint8)    - Self / Party / Raid /
//                                  Friendly bitmask
//     statBonusKind (uint8)     - Stamina / Intellect /
//                                  Spirit / AllStats /
//                                  Armor / SpellPower /
//                                  AttackPower / Crit /
//                                  Haste / Mastery
//     rank (uint8)              - 1-based rank number
//     maxStackCount (uint8)     - typically 1
//     statBonusAmount (int32)   - signed magnitude
//                                  (negative = debuff)
//     duration (uint32)         - seconds (0 = until
//                                  cancel / log out)
//     previousRankId (uint32)   - 0 if rank 1
//     nextRankId (uint32)       - 0 if max rank
//     iconColorRGBA (uint32)
struct WoweeBuffBook {
    enum TargetTypeBit : uint8_t {
        TargetSelf     = 0x01,
        TargetParty    = 0x02,
        TargetRaid     = 0x04,
        TargetFriendly = 0x08,    // any friendly target
                                   // including outside party
    };

    enum StatBonusKind : uint8_t {
        Stamina      = 0,
        Intellect    = 1,
        Spirit       = 2,
        AllStats     = 3,
        Armor        = 4,
        SpellPower   = 5,
        AttackPower  = 6,
        CritRating   = 7,
        HasteRating  = 8,
        ManaRegen    = 9,
        Other        = 255,    // not statted (e.g.
                                // Trueshot Aura is %DPS,
                                // not a flat stat)
    };

    struct Entry {
        uint32_t buffId = 0;
        std::string name;
        std::string description;
        uint32_t spellId = 0;
        uint32_t castClassMask = 0;
        uint8_t targetTypeMask = TargetSelf | TargetParty;
        uint8_t statBonusKind = Other;
        uint8_t rank = 1;
        uint8_t maxStackCount = 1;
        int32_t statBonusAmount = 0;
        uint32_t duration = 0;
        uint32_t previousRankId = 0;
        uint32_t nextRankId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t buffId) const;

    // Walks the rank chain from buffId all the way to
    // rank 1, returning entries from first to last. Used
    // by the spellbook UI's "rank picker" widget which
    // shows all ranks the player can currently train.
    [[nodiscard]] std::vector<const Entry*>
    walkChainBackToRoot(uint32_t buffId) const;

    // Returns the highest rank in the chain starting from
    // buffId (the entry with nextRankId=0 reachable from
    // here). Used by the auto-cast logic to always apply
    // the highest rank the caster knows.
    [[nodiscard]] const Entry* findChainTip(uint32_t buffId) const;
};

class WoweeBuffBookLoader {
public:
    static bool save(const WoweeBuffBook& cat,
                     const std::string& basePath);
    static WoweeBuffBook load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bab* variants.
    //
    //   makeMage      - 4 entries: Arcane Intellect ranks
    //                    1-4 with explicit rank chain.
    //   makeDruid     - 5 entries: Mark of the Wild ranks
    //                    1-5 with explicit rank chain.
    //   makeRaidMax   - 6 entries: one max-rank buff per
    //                    buffing class (Mark of the Wild
    //                    R7, Power Word: Fortitude R8,
    //                    Arcane Intellect R6, Blessing
    //                    of Kings R1, Battle Shout R9,
    //                    Trueshot Aura R3) - no chain
    //                    edges since each is standalone.
    static WoweeBuffBook makeMage(const std::string& catalogName);
    static WoweeBuffBook makeDruid(const std::string& catalogName);
    static WoweeBuffBook makeRaidMax(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
