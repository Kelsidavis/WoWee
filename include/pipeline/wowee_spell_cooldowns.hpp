#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Cooldown Category catalog (.wscd) -
// novel replacement for Blizzard's SpellCooldown.dbc plus
// the per-spell category-cooldown fields in Spell.dbc.
// Defines the shared-cooldown buckets that related spells
// reference, so casting one spell triggers a cooldown on
// every other spell in the same bucket. Examples: every
// Mage Polymorph variant (Sheep / Pig / Turtle / Cat)
// shares a single bucket so polymorphing a target locks
// all the morph spells, not just the one cast. Healing
// potions, mana potions, and similar consumables work the
// same way.
//
// Distinct from WSDR (Spell Duration), which times how
// long an aura stays on a target. WSCD times how long
// before a spell can be cast again, applied to everyone in
// the bucket - the global cooldown (GCD) is the most
// common bucket of all.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the spell
//   engine. WSPL spell entries reference cooldownBucketId.
//
// Binary layout (little-endian):
//   magic[4]            = "WSCD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     bucketId (uint32)
//     nameLen + name
//     descLen + description
//     bucketKind (uint8) / pad[3]
//     cooldownMs (uint32)
//     categoryFlags (uint32)
//     iconColorRGBA (uint32)
struct WoweeSpellCooldown {
    enum BucketKind : uint8_t {
        Spell    = 0,    // spell-only cooldown bucket
        Item     = 1,    // item-only cooldown bucket
        Class    = 2,    // class-shared bucket (e.g. all mage AOE)
        Global   = 3,    // global cooldown - all combat spells
        Misc     = 4,    // catch-all (engineering trinkets, etc.)
    };

    enum CategoryFlag : uint32_t {
        AffectedByHaste     = 1u << 0,   // cooldown shrinks with haste
        SharedWithItems     = 1u << 1,   // spells + items share bucket
        OnGCDStart          = 1u << 2,   // triggers at GCD start, not cast finish
        IgnoresCooldownReduction = 1u << 3, // not affected by CDR talents
    };

    struct Entry {
        uint32_t bucketId = 0;
        std::string name;
        std::string description;
        uint8_t bucketKind = Spell;
        uint32_t cooldownMs = 0;
        uint32_t categoryFlags = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t bucketId) const;

    static const char* bucketKindName(uint8_t k);
};

class WoweeSpellCooldownLoader {
public:
    static bool save(const WoweeSpellCooldown& cat,
                     const std::string& basePath);
    static WoweeSpellCooldown load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cdb* variants.
    //
    //   makeStarter - 4 baseline buckets (GlobalCooldown
    //                  1.5s, ShortItem 5s, MediumItem 30s,
    //                  LongItem 60s) covering the most
    //                  common cooldown tiers including the
    //                  GCD itself.
    //   makeClass   - 5 mage-specific buckets for spells
    //                  that share cooldowns within a class
    //                  (PolymorphFamily 0s - instant on hit
    //                  but exclusive, AlterTime 90s,
    //                  Counterspell 24s, Blink 15s,
    //                  IceBlock 5min).
    //   makeItems   - 5 item-cooldown buckets (HealingPot
    //                  60s, ManaPot 60s, ManaJade 1.5s
    //                  GCD-only, EngineerTrinket 60s,
    //                  HearthstoneFamily 60min).
    static WoweeSpellCooldown makeStarter(const std::string& catalogName);
    static WoweeSpellCooldown makeClass(const std::string& catalogName);
    static WoweeSpellCooldown makeItems(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
