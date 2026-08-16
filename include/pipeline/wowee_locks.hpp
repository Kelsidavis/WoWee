#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Lock Template (.wlck) - novel replacement for
// Blizzard's Lock.dbc. The 18th open format added to the
// editor. Closes the cross-reference gap from WGOT.entry.lockId
// (and the future WIT lockbox subset) - until now those
// fields pointed to a format that didn't exist yet.
//
// A lock is a multi-channel security check. Each lock has up
// to 5 independent channels; a player can open the lock by
// satisfying ANY ONE channel. Channels can be:
//   • Item    - requires a specific key item (WIT cross-ref)
//   • Lockpick - requires the lockpicking skill at a minimum
//                 rank (rogue / engineering profession)
//   • Spell   - requires casting a specific spell
//   • Damage  - can be forced open with attack damage
//
// Cross-references with previously-added formats:
//   WGOT.entry.lockId  → WLCK.entry.lockId
//   WLCK.channel.targetId (kind=Item)  → WIT.entry.itemId
//   WLCK.channel.targetId (kind=Skill) → future WSKL skillId
//   WLCK.channel.targetId (kind=Spell) → future WSPL spellId
//
// Binary layout (little-endian):
//   magic[4]            = "WLCK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     lockId (uint32)
//     nameLen + name
//     flags (uint32)
//     -- 5 channel slots, all written even when unused; an
//        unused slot has kind=None and zeroed fields.
//     channels[5] × {
//       kind (uint8) + pad[1]
//       skillRequired (uint16)
//       targetId (uint32)
//     }
struct WoweeLock {
    static constexpr int kChannelSlots = 5;

    enum ChannelKind : uint8_t {
        ChannelNone     = 0,
        ChannelItem     = 1,    // requires a key item (targetId = WIT itemId)
        ChannelLockpick = 2,    // requires lockpicking skill (targetId = skill ID)
        ChannelSpell    = 3,    // requires casting a spell (targetId = spell ID)
        ChannelDamage   = 4,    // can be forced open with damage (targetId unused)
    };

    enum Flags : uint32_t {
        DestructOnOpen  = 0x01,    // lock destroyed after one successful open
        RespawnOnKey    = 0x02,    // re-locks itself after key use (timed)
        TrapOnFail      = 0x04,    // failure triggers a trap (script-handled)
    };

    struct Channel {
        uint8_t kind = ChannelNone;
        uint16_t skillRequired = 0;    // only meaningful for ChannelLockpick
        uint32_t targetId = 0;         // item / skill / spell id
    };

    struct Entry {
        uint32_t lockId = 0;
        std::string name;
        uint32_t flags = 0;
        Channel channels[kChannelSlots] = {};
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by lockId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t lockId) const;

    static const char* channelKindName(uint8_t k);
};

class WoweeLockLoader {
public:
    static bool save(const WoweeLock& cat,
                     const std::string& basePath);
    static WoweeLock load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-locks* variants.
    //
    //   makeStarter - 2 locks: a basic wooden chest lock
    //                  (lockId=1, requires no skill, can be
    //                  forced open) plus a small key-required
    //                  lockbox (lockId=2). lockId=1 matches
    //                  WGOT.makeDungeon's iron-door lockId.
    //   makeDungeon - 3 dungeon-tier locks: light lockpick
    //                  (lockId=2 matching WGOT bandit chest),
    //                  steel chest (heavy lockpick OR specific
    //                  key), and a quest-key-only seal.
    //   makeProfessions - 4 profession-keyed locks: lockpick at
    //                      ranks 1/100/175/250 covering the
    //                      classic-tier rogue / engineering
    //                      progression curve.
    static WoweeLock makeStarter(const std::string& catalogName);
    static WoweeLock makeDungeon(const std::string& catalogName);
    static WoweeLock makeProfessions(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
