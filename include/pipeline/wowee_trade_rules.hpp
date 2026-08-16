#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Trade Window Rules catalog (.wtrd) -
// novel replacement for the implicit player-to-player
// trade policy rules vanilla WoW hardcoded across the
// trade-window message handlers (CMSG_INITIATE_TRADE,
// CMSG_SET_TRADE_ITEM, CMSG_SET_TRADE_GOLD), the
// soulbound-item check, the cross-faction-trade
// rejection, and the GM-trade audit hooks. Each entry
// is one trade policy rule the trade-window state
// machine consults at every state transition.
//
// Cross-references with previously-added formats:
//   WIT:  itemCategoryFilter uses the WIT item-class
//         + subclass bit conventions (Weapon=2,
//         Armor=4, Container=1, etc.).
//   WCHC: targetingFilter uses the WCHC faction-bit
//         convention for SameFactionOnly.
//
// Binary layout (little-endian):
//   magic[4]            = "WTRD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     ruleId (uint32)
//     nameLen + name
//     descLen + description
//     ruleKind (uint8)            - Allowed / Forbidden /
//                                    SoulboundException /
//                                    CrossFactionAllowed /
//                                    LevelGated /
//                                    GoldEscrowMax /
//                                    AuditLogged
//     targetingFilter (uint8)      - AnyPlayer /
//                                    SameRealmOnly /
//                                    SameFactionOnly /
//                                    SameAccountOnly /
//                                    GMOnly
//     levelRequirement (uint8)     - 0 = no level gate
//     priority (uint8)             - higher overrides
//                                    lower in conflict
//     itemCategoryFilter (uint32)  - bitmask of
//                                    WIT item classes
//     goldEscrowMaxCopper (uint64) - max gold side for
//                                    this rule (0 =
//                                    unlimited)
//     iconColorRGBA (uint32)
struct WoweeTradeRules {
    enum RuleKind : uint8_t {
        Allowed              = 0,    // explicitly allow
                                      // (highest-priority
                                      // override)
        Forbidden            = 1,    // block trade if
                                      // category bits
                                      // match
        SoulboundException   = 2,    // 2hr trade-back
                                      // window post-loot
        CrossFactionAllowed  = 3,    // server-custom
                                      // override of base
                                      // same-faction rule
        LevelGated           = 4,    // require both
                                      // players >=
                                      // levelRequirement
        GoldEscrowMax        = 5,    // cap gold side at
                                      // goldEscrowMaxCopper
        AuditLogged          = 6,    // log every trade
                                      // matching this rule
    };

    enum TargetingFilter : uint8_t {
        AnyPlayer        = 0,
        SameRealmOnly    = 1,
        SameFactionOnly  = 2,
        SameAccountOnly  = 3,    // own-character transfer
        GMOnly           = 4,    // requires GM flag on
                                  // initiator
    };

    struct Entry {
        uint32_t ruleId = 0;
        std::string name;
        std::string description;
        uint8_t ruleKind = Forbidden;
        uint8_t targetingFilter = AnyPlayer;
        uint8_t levelRequirement = 0;
        uint8_t priority = 1;
        uint32_t itemCategoryFilter = 0;
        uint64_t goldEscrowMaxCopper = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t ruleId) const;

    // Returns all rules of one kind (e.g. all Forbidden
    // rules to enumerate the explicit blocks). Used by
    // the trade-window state machine to dispatch checks
    // by kind.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t ruleKind) const;
};

class WoweeTradeRulesLoader {
public:
    static bool save(const WoweeTradeRules& cat,
                     const std::string& basePath);
    static WoweeTradeRules load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-trd* variants.
    //
    //   makeStandard   - 4 standard trade rules
    //                     (Soulbound Forbidden globally,
    //                     Quest items Forbidden, 2hr
    //                     SoulboundException for raid
    //                     drops, SameFactionOnly default).
    //   makeServerAdmin - 3 server-admin rules (GM-only
    //                     escrow trade, AccountBound
    //                     own-character transfer, Cross-
    //                     faction at level 80 for custom
    //                     servers).
    //   makeRMTPrevent - 4 anti-RMT rules (LevelGated 30+
    //                     for any gold trade, Gold cap for
    //                     new accounts, AuditLogged for
    //                     all trades > 1000g, mandatory
    //                     delay placeholder).
    static WoweeTradeRules makeStandard(const std::string& catalogName);
    static WoweeTradeRules makeServerAdmin(const std::string& catalogName);
    static WoweeTradeRules makeRMTPrevent(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
