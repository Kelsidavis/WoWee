#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Currency Type catalog (.wctr) - novel
// replacement for Blizzard's CurrencyTypes.dbc plus the
// per-currency cap tables in CurrencyCategory.dbc.
// Defines the in-game currencies that are NOT regular
// item stacks: Honor Points, Arena Points, Justice
// Points, Valor Points, Conquest Points, plus the various
// faction tokens (Champion's Seal, Wintergrasp Mark of
// Honor, Emblem of Frost, etc).
//
// Distinct from regular items in WIT - currencies are
// tracked per-character as scalar quantities with
// weekly+absolute caps, not as stackable inventory slots.
// Some currencies are still backed by a WIT item entry
// for the icon/tooltip (itemId field), while others
// (Honor, Arena) live entirely in the currency system.
//
// Cross-references with previously-added formats:
//   WIT:  itemId references WIT entries for the icon
//         and tooltip text on currency-display tooltips.
//   WFAC: currencies tied to faction reputation
//         (Champion's Seal -> Argent Crusade) reference
//         WFAC factionId via the categoryId field.
//
// Binary layout (little-endian):
//   magic[4]            = "WCTR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     currencyId (uint32)
//     nameLen + name
//     descLen + description
//     itemId (uint32)               // 0 if pure currency
//     maxQuantity (uint32)          // 0 = no cap
//     maxQuantityWeekly (uint32)    // 0 = no weekly cap
//     categoryId (uint32)
//     currencyKind (uint8) / isAccountWide (uint8) / pad[2]
//     iconPathLen + iconPath
//     iconColorRGBA (uint32)
struct WoweeCurrencyType {
    enum CurrencyKind : uint8_t {
        PvPHonor      = 0,    // Honor / Conquest / Arena
        PvERaid       = 1,    // Justice / Valor / Emblems
        FactionToken  = 2,    // faction-rep-gated token
        EventToken    = 3,    // holiday / world-event token
        Crafting      = 4,    // crafting reputation token
        Misc          = 5,    // catch-all
    };

    struct Entry {
        uint32_t currencyId = 0;
        std::string name;
        std::string description;
        uint32_t itemId = 0;
        uint32_t maxQuantity = 0;
        uint32_t maxQuantityWeekly = 0;
        uint32_t categoryId = 0;
        uint8_t currencyKind = PvPHonor;
        uint8_t isAccountWide = 0;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        std::string iconPath;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t currencyId) const;

    // Returns the smaller of (remaining weekly cap,
    // remaining absolute cap) - i.e. the maximum amount a
    // character can earn right now given current balances.
    // Either cap is unbounded if the corresponding field
    // is 0.
    [[nodiscard]] uint32_t earnableNow(uint32_t currencyId,
                          uint32_t currentTotal,
                          uint32_t earnedThisWeek) const;

    static const char* currencyKindName(uint8_t k);
};

class WoweeCurrencyTypeLoader {
public:
    static bool save(const WoweeCurrencyType& cat,
                     const std::string& basePath);
    static WoweeCurrencyType load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-ctr* variants.
    //
    //   makePvP    - 4 PvP currencies (Honor Points 75k
    //                 cap, Arena Points 5k cap weekly,
    //                 Conquest Points 1650 weekly,
    //                 Champion's Seal no cap).
    //   makePvE    - 4 PvE raid currencies (Justice Points
    //                 4k cap, Valor Points 1k weekly,
    //                 Emblem of Frost no weekly cap,
    //                 Trophy of the Crusade no cap).
    //   makeFactionTokens - 4 faction reputation tokens
    //                 (Hodir Spear-fragment, Cenarion Mark,
    //                 Argent Dawn Valor Token, Wintergrasp
    //                 Mark) - no quantity cap, gated by
    //                 reputation thresholds.
    static WoweeCurrencyType makePvP(const std::string& catalogName);
    static WoweeCurrencyType makePvE(const std::string& catalogName);
    static WoweeCurrencyType makeFactionTokens(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
