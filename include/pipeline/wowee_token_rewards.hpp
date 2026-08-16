#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Token Reward catalog (.wtbr) - novel
// replacement for the per-vendor token redemption tables
// (currency_token_reward / npc_vendor token rows). Each
// entry says "spend N copies of token X to receive
// reward Y", with reward type polymorphism: Y can be an
// item, a spell (taught to the character), a title, a
// mount, a companion pet, a currency conversion, or an
// heirloom unlock. The rewardId field's interpretation
// depends on rewardKind.
//
// Distinct from WTKN (Token catalog) which defines the
// token currency items themselves. WTKN says "the
// Champion's Seal exists as item 44990"; WTBR says
// "spend 25 Champion's Seals at Argent Tournament for
// the Squire's Belt (item 45517)".
//
// Cross-references with previously-added formats:
//   WTKN: spentTokenItemId references the token currency.
//   WIT:  rewardId references WIT.itemId for Item kind.
//   WSPL: rewardId references WSPL.spellId for Spell kind.
//   WTIT: rewardId references WTIT.titleId for Title kind.
//   WMOU: rewardId references WMOU.mountId for Mount kind.
//   WCMP: rewardId references WCMP.companionId for Pet kind.
//   WCTR: rewardId references WCTR.currencyId for Currency
//         conversion kind (e.g. 1 Trophy -> 50 Justice).
//   WFAC: requiredFactionId references WFAC.factionId for
//         reputation-gated rewards.
//
// Binary layout (little-endian):
//   magic[4]            = "WTBR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tokenRewardId (uint32)
//     nameLen + name
//     descLen + description
//     spentTokenItemId (uint32)
//     spentTokenCount (uint32)
//     rewardKind (uint8) / requiredFactionStanding (uint8) / pad[2]
//     rewardId (uint32)
//     rewardCount (uint32)
//     requiredFactionId (uint32)
//     iconColorRGBA (uint32)
struct WoweeTokenReward {
    enum RewardKind : uint8_t {
        Item        = 0,    // grants WIT item
        Spell       = 1,    // teaches WSPL spell to character
        Title       = 2,    // grants WTIT title
        Mount       = 3,    // unlocks WMOU mount
        Pet         = 4,    // grants WCMP companion pet
        Currency    = 5,    // converts to WCTR currency
        Heirloom    = 6,    // unlocks heirloom from WIT
        Cosmetic    = 7,    // tabard / pennant / fluff
    };

    enum FactionStanding : uint8_t {
        Hated      = 0,
        Hostile    = 1,
        Unfriendly = 2,
        Neutral    = 3,
        Friendly   = 4,
        Honored    = 5,
        Revered    = 6,
        Exalted    = 7,
    };

    struct Entry {
        uint32_t tokenRewardId = 0;
        std::string name;
        std::string description;
        uint32_t spentTokenItemId = 0;
        uint32_t spentTokenCount = 1;
        uint8_t rewardKind = Item;
        uint8_t requiredFactionStanding = Neutral;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t rewardId = 0;
        uint32_t rewardCount = 1;
        uint32_t requiredFactionId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tokenRewardId) const;

    // Returns all rewards offered for spending the given
    // token item id. Used by vendor frames to populate
    // the "what can I buy with these?" list.
    [[nodiscard]] std::vector<const Entry*> findByToken(uint32_t tokenItemId) const;

    static const char* rewardKindName(uint8_t k);
    static const char* factionStandingName(uint8_t s);
};

class WoweeTokenRewardLoader {
public:
    static bool save(const WoweeTokenReward& cat,
                     const std::string& basePath);
    static WoweeTokenReward load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tbr* variants.
    //
    //   makeRaidTokens - 5 raid tier-token redemptions
    //                     (T9 Conqueror's helm, Vanquisher's
    //                     chest, etc) consuming Trophy of
    //                     the Crusade.
    //   makePvP        - 5 PvP redemptions (Honor → BG
    //                     mount, Arena → arena weapon,
    //                     Conquest → PvP tier helm).
    //   makeFaction    - 5 faction-gated rewards (Argent
    //                     Tournament tabard at Honored,
    //                     Hodir mammoth mount at Exalted,
    //                     Cenarion ring at Revered, etc.)
    static WoweeTokenReward makeRaidTokens(const std::string& catalogName);
    static WoweeTokenReward makePvP(const std::string& catalogName);
    static WoweeTokenReward makeFaction(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
