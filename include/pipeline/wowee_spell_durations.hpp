#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Duration Index catalog (.wsdr) - novel
// replacement for Blizzard's SpellDuration.dbc plus the
// per-spell duration fields in Spell.dbc. Defines the
// categorical duration buckets that auras / DoTs / HoTs /
// buffs reference (5s / 30s / 5min / 1hr / UntilCancelled /
// UntilDeath).
//
// Completes the WSRG (range) + WSCT (cast time) + WSDR
// (duration) triplet - together these three small catalogs
// let the spell engine resolve every Frostbolt's range,
// cast time, and chill-debuff duration with three table
// lookups instead of duplicating per-rank fields across
// thousands of spells.
//
// Duration can scale with caster level via perLevelMs (a
// rank-1 Renew at 9s grows to 12s at lvl 60), then is
// clamped to maxDurationMs (e.g. world buffs cap at
// 4 hours). A negative baseDurationMs of -1 by convention
// means "until cancelled" (reads as Permanent kind in the
// engine HUD).
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the spell
//   engine. WSPL spell entries reference durationId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSDR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     durationId (uint32)
//     nameLen + name
//     descLen + description
//     durationKind (uint8) / pad[3]
//     baseDurationMs (int32)
//     perLevelMs (int32)
//     maxDurationMs (int32)
//     iconColorRGBA (uint32)
struct WoweeSpellDuration {
    enum DurationKind : uint8_t {
        Instant         = 0,    // 0ms - fires once, no aura
        Timed           = 1,    // standard timed buff/debuff
        TickBased       = 2,    // DoT / HoT (tick interval set elsewhere)
        UntilCancelled  = 3,    // permanent until cancelled
        UntilDeath      = 4,    // permanent until target dies
    };

    struct Entry {
        uint32_t durationId = 0;
        std::string name;
        std::string description;
        uint8_t durationKind = Timed;
        int32_t baseDurationMs = 0;
        int32_t perLevelMs = 0;
        int32_t maxDurationMs = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t durationId) const;

    // Resolve to the actual duration in ms at the given
    // caster level. Clamps to maxDurationMs when set
    // (>0). Returns -1 for kinds with negative base
    // (UntilCancelled / UntilDeath) to signal "no timer".
    [[nodiscard]] int32_t resolveAtLevel(uint32_t durationId,
                           uint32_t casterLevel) const;

    static const char* durationKindName(uint8_t k);
};

class WoweeSpellDurationLoader {
public:
    static bool save(const WoweeSpellDuration& cat,
                     const std::string& basePath);
    static WoweeSpellDuration load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sdr* variants.
    //
    //   makeStarter - 5 baseline buckets (Instant 0,
    //                  Short 5s, Medium 30s, Long 5min,
    //                  Hour 1hr) spanning the most common
    //                  duration tiers from instant fires to
    //                  hour-long world buffs.
    //   makeBuffs   - 4 long-duration buffs (PartyBuff 30m,
    //                  RaidBuff 60m, WorldBuff 4hr,
    //                  UntilDeath -1). UntilDeath uses the
    //                  UntilDeath kind with a sentinel
    //                  baseDurationMs of -1.
    //   makeDot     - 4 DoT/HoT buckets (4-tick 12s,
    //                  5-tick 15s, 6-tick 18s, 8-tick 24s)
    //                  using TickBased kind. Tick interval
    //                  is implied at 3s/tick.
    static WoweeSpellDuration makeStarter(const std::string& catalogName);
    static WoweeSpellDuration makeBuffs(const std::string& catalogName);
    static WoweeSpellDuration makeDot(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
