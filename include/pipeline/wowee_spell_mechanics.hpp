#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Mechanic catalog (.wsmc) - novel
// replacement for Blizzard's SpellMechanic.dbc plus the
// AzerothCore-style diminishing-returns (DR) tables. Defines
// crowd-control mechanic categories that spells reference:
// Stun, Silence, Polymorph, Root, Snare, Disorient, Disarm,
// Knockback, Sleep, Fear, Charm, Frenzy. Each mechanic
// carries gameplay metadata (breaks-on-damage, can-be-
// dispelled, default duration) plus a diminishing-returns
// category that the runtime uses to gate repeated CC on
// the same target.
//
// Cross-references with previously-added formats:
//   WSMC.entry.mechanicId is referenced by WSPL.spellId
//                           (each spell tags itself with the
//                            CC category it applies)
//   WSMC.entry.conflictsMask is a bitmask of OTHER WSMC
//                             mechanicIds - only one mechanic
//                             from a conflict-group can apply
//                             to a target simultaneously.
//
// Binary layout (little-endian):
//   magic[4]            = "WSMC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     mechanicId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     breaksOnDamage (uint8) / canBeDispelled (uint8) /
//       drCategory (uint8) / dispelType (uint8)
//     defaultDurationMs (uint32)
//     maxStacks (uint8) / pad[3]
//     conflictsMask (uint32)
struct WoweeSpellMechanic {
    enum DiminishingReturnsCategory : uint8_t {
        DRNone        = 0,    // no DR grouping
        DRStun        = 1,    // stuns share a DR bucket
        DRDisorient   = 2,    // fears + horrors + scares
        DRSilence     = 3,    // silences + interrupts
        DRRoot        = 4,    // roots (entangling, frost nova)
        DRPolymorph   = 5,    // CC polymorph effects
        DRControlled  = 6,    // mind controls + charms
        DRMisc        = 7,    // catch-all bucket
    };

    enum DispelType : uint8_t {
        DispelNone     = 0,    // not dispellable
        DispelMagic    = 1,    // dispel magic / mass dispel
        DispelCurse    = 2,    // remove curse / curse of weakness
        DispelDisease  = 3,    // cure disease / abolish
        DispelPoison   = 4,    // cure poison / cleanse
        DispelEnrage   = 5,    // tranquilizing shot / soothe
        DispelStealth  = 6,    // not really dispellable, removed
                                // by detection
    };

    struct Entry {
        uint32_t mechanicId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t breaksOnDamage = 0;       // 1 = ends on damage
        uint8_t canBeDispelled = 0;
        uint8_t drCategory = DRNone;
        uint8_t dispelType = DispelNone;
        uint32_t defaultDurationMs = 0;   // 0 = caster-defined
        uint8_t maxStacks = 1;
        uint32_t conflictsMask = 0;       // bitmask of other ids
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t mechanicId) const;

    static const char* drCategoryName(uint8_t c);
    static const char* dispelTypeName(uint8_t d);
};

class WoweeSpellMechanicLoader {
public:
    static bool save(const WoweeSpellMechanic& cat,
                     const std::string& basePath);
    static WoweeSpellMechanic load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-smc* variants.
    //
    //   makeStarter - 3 baseline mechanics (Stun / Silence /
    //                  Snare) - covers the most common CC
    //                  cases one each in 3 different DR
    //                  categories.
    //   makeHardCC  - 5 hard-CC mechanics (Stun, Polymorph,
    //                  Sleep, Fear, Knockback) with
    //                  conflictsMask wiring so they DR each
    //                  other in the right buckets.
    //   makeRoots   - 4 movement-impair mechanics (Root,
    //                  Snare, Slow, GroundPin) with the
    //                  breaks-on-damage and stacking flags
    //                  set per real WoW behavior.
    static WoweeSpellMechanic makeStarter(const std::string& catalogName);
    static WoweeSpellMechanic makeHardCC(const std::string& catalogName);
    static WoweeSpellMechanic makeRoots(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
