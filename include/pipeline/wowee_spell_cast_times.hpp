#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Cast Time Index catalog (.wsct) - novel
// replacement for Blizzard's SpellCastTimes.dbc plus the
// per-spell castTime fields in Spell.dbc. Defines the
// categorical cast-time buckets that spells reference
// (instant / short / medium / long), so thousands of spells
// can share the same timing metadata instead of each
// embedding their own ms count.
//
// Cast time can scale with character level - Frostbolt at
// rank 1 might cast in 1500ms, but the rank-11 version
// references a bucket where baseCastMs and perLevelMs
// combine to give 2500ms by lvl 60. Haste is then applied
// on top of the bucket result, clamped to [minCastMs,
// maxCastMs].
//
// Companion catalog to WSRG (Spell Range Index) - together
// they let the spell engine look up "Frostbolt's range
// bucket = id 3 (Spell, 0-30y)" and "Frostbolt's cast time
// bucket = id 5 (LongCast, 3000ms base)" with two table
// reads instead of carrying duplicate per-spell data for
// every rank of every spell.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the spell
//   engine. WSPL spell entries reference castTimeId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSCT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     castTimeId (uint32)
//     nameLen + name
//     descLen + description
//     castKind (uint8) / pad[3]
//     baseCastMs (int32)
//     perLevelMs (int32)
//     minCastMs (int32)
//     maxCastMs (int32)
//     iconColorRGBA (uint32)
struct WoweeSpellCastTime {
    enum CastKind : uint8_t {
        Instant      = 0,    // 0ms - fires on key release
        Cast         = 1,    // standard cast, can be haste-shortened
        Channel      = 2,    // channels for full duration
        DelayedCast  = 3,    // queued / next-server-tick cast
        ChargeCast   = 4,    // hold-to-power spell (Heroic Strike)
    };

    struct Entry {
        uint32_t castTimeId = 0;
        std::string name;
        std::string description;
        uint8_t castKind = Cast;
        int32_t baseCastMs = 0;
        int32_t perLevelMs = 0;
        int32_t minCastMs = 0;
        int32_t maxCastMs = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t castTimeId) const;

    // Resolve to the actual ms a spell will cast at the
    // given character level, before haste is applied.
    // Clamps to [minCastMs, maxCastMs] when those are set.
    [[nodiscard]] int32_t resolveAtLevel(uint32_t castTimeId,
                           uint32_t characterLevel) const;

    static const char* castKindName(uint8_t k);
};

class WoweeSpellCastTimeLoader {
public:
    static bool save(const WoweeSpellCastTime& cat,
                     const std::string& basePath);
    static WoweeSpellCastTime load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sct* variants.
    //
    //   makeStarter   - 4 baseline buckets (Instant 0ms,
    //                    FastCast 1000ms, MediumCast 1500ms,
    //                    LongCast 3000ms) covering the
    //                    most common cast time tiers.
    //   makeChannel   - 3 channeled-spell buckets (TickEvery1s
    //                    1000ms, TickEvery2s 2000ms,
    //                    TickEvery3s 3000ms) for AoE channels
    //                    like Arcane Missiles / Drain Life.
    //   makeRamp      - 4 level-scaled buckets where
    //                    perLevelMs > 0 - cast time grows
    //                    with character level, simulating
    //                    higher-rank spells with longer cast
    //                    times.
    static WoweeSpellCastTime makeStarter(const std::string& catalogName);
    static WoweeSpellCastTime makeChannel(const std::string& catalogName);
    static WoweeSpellCastTime makeRamp(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
