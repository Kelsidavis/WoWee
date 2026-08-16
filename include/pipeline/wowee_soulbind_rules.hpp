#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Soulbind Rules catalog (.wbnd) -
// novel replacement for the implicit item-binding
// policy vanilla WoW carried in
// ItemTemplate.bondingType + per-item special-case
// rules in the server's LootMgr (the 2-hour
// raid-loot trade window was hard-coded; the
// account-bound-shared-across-faction rule for
// heirlooms was a TBC+ addition with no formal
// data-driven format). Each WBND entry binds one
// soulbind rule to its bind kind (BoP / BoE / BoU /
// BoA / Soulbound / NoBind), tradable-window
// duration, raid-trade allowance, BoE-becomes-BoP
// trigger, and cross-faction sharing flag.
//
// Cross-references with previously-added formats:
//   WIT:  rules apply to items by qualityFloor
//         predicate (every WIT item with quality >=
//         floor uses this rule unless overridden).
//   None to specific itemId - the rule catalog
//   describes POLICY, not per-item bindings.
//
// Binary layout (little-endian):
//   magic[4]            = "WBND"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     ruleId (uint32)
//     nameLen + name
//     bindKind (uint8)               - 0=BindOnPickup
//                                       /1=BindOnEquip
//                                       /2=BindOnUse
//                                       /3=BindOnAccount
//                                       /4=Soulbound
//                                       (already on
//                                       loot)
//                                       /5=NoBind
//     itemQualityFloor (uint8)       - quality at
//                                       which this rule
//                                       applies (0=
//                                       Poor..7=Heir-
//                                       loom)
//     tradableForRaidGroup (uint8)   - 0/1 bool - BoP
//                                       gets the 2hr
//                                       raid-trade
//                                       window
//     boeBecomesBoP (uint8)          - 0/1 bool - BoE
//                                       becomes
//                                       Soulbound on
//                                       pickup
//     accountBoundCrossFaction (uint8) - 0/1 bool -
//                                         BoA can
//                                         transfer
//                                         Alliance<->
//                                         Horde via
//                                         account
//                                         (heirloom
//                                         flag)
//     pad0 (uint8)
//     pad1 (uint16)
//     tradableWindowSec (uint32)     - duration of
//                                       the raid-trade
//                                       window (vanilla
//                                       had no window;
//                                       TBC default
//                                       7200=2hr)
//     descriptionLen + description   - human-readable
//                                       policy summary
//                                       for the editor
struct WoweeSoulbindRules {
    enum BindKind : uint8_t {
        BindOnPickup   = 0,
        BindOnEquip    = 1,
        BindOnUse      = 2,
        BindOnAccount  = 3,
        Soulbound      = 4,    // already-soulbound at
                                // loot time (rare)
        NoBind         = 5,    // tradable forever
    };

    enum ItemQuality : uint8_t {
        Poor      = 0,
        Common    = 1,
        Uncommon  = 2,
        Rare      = 3,
        Epic      = 4,
        Legendary = 5,
        Artifact  = 6,
        Heirloom  = 7,
    };

    struct Entry {
        uint32_t ruleId = 0;
        std::string name;
        uint8_t bindKind = BindOnPickup;
        uint8_t itemQualityFloor = Poor;
        uint8_t tradableForRaidGroup = 0;
        uint8_t boeBecomesBoP = 0;
        uint8_t accountBoundCrossFaction = 0;
        uint8_t pad0 = 0;
        uint16_t pad1 = 0;
        uint32_t tradableWindowSec = 0;
        std::string description;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t ruleId) const;

    // Resolve the rule that applies to an item of a
    // given quality. Returns the most-specific rule
    // (highest qualityFloor that doesn't exceed
    // itemQuality). The runtime walks all rules and
    // picks the best match.
    [[nodiscard]] const Entry* resolveForQuality(uint8_t itemQuality) const;

    // Returns all rules of one bind kind - used by
    // the inventory UI to color-code BoP vs BoE
    // tooltips uniformly.
    [[nodiscard]] std::vector<const Entry*> findByBindKind(uint8_t bindKind) const;
};

class WoweeSoulbindRulesLoader {
public:
    static bool save(const WoweeSoulbindRules& cat,
                     const std::string& basePath);
    static WoweeSoulbindRules load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-bnd* variants.
    //
    //   makeVanillaPolicy - 4 vanilla rules (Poor=
    //                        NoBind, Common=BoE,
    //                        Uncommon+=BoP no-window,
    //                        Epic+=Soulbound). No
    //                        raid-trade window.
    //   makeTBCPolicy     - 5 TBC rules (Poor=NoBind,
    //                        Common=BoE,
    //                        Uncommon+=BoP+2hr window,
    //                        Rare+=BoP+2hr window,
    //                        Epic+=Soulbound). Raid-
    //                        trade window introduced.
    //   makeWotLKPolicy   - 6 WotLK rules adding
    //                        Heirloom=BindOnAccount
    //                        cross-faction. Otherwise
    //                        same as TBC.
    static WoweeSoulbindRules makeVanillaPolicy(const std::string& catalogName);
    static WoweeSoulbindRules makeTBCPolicy(const std::string& catalogName);
    static WoweeSoulbindRules makeWotLKPolicy(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
