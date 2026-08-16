#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Token / Currency catalog (.wtkn) - novel
// replacement for Blizzard's Currency.dbc +
// CurrencyCategory.dbc + CurrencyTypes.dbc + the
// AzerothCore-style player_currency SQL tables. The 28th
// open format added to the editor.
//
// Defines secondary currency tokens beyond gold: Honor
// Points (PvP), Arena Points (rated PvP), Marks of Honor
// (per battleground), faction reputation tokens, etc. Each
// token has a balance cap, optional weekly cap (regenerating
// per-week limit on earnings), and a category for grouping
// in the player's currency tab.
//
// Cross-references with previously-added formats:
//   WTRN.item.extendedCost → WTKN.entry.tokenId
//                            (vendors can charge in tokens
//                             instead of copper - when
//                             extendedCost > 0 the runtime
//                             looks up the corresponding token)
//
// Binary layout (little-endian):
//   magic[4]            = "WTKN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tokenId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     category (uint8) + flags_byte (uint8) + pad[2]
//     maxBalance (uint32)        -- 0 = unlimited
//     weeklyCap (uint32)         -- 0 = no weekly cap
//     flags (uint32)
struct WoweeToken {
    enum Category : uint8_t {
        Misc       = 0,
        Pvp        = 1,    // Honor / Arena / Marks
        Reputation = 2,    // faction-specific tokens
        Crafting   = 3,    // profession turn-in tokens
        Seasonal   = 4,    // event-only currencies
        Holiday    = 5,
    };

    enum Flags : uint32_t {
        AccountWide       = 0x01,
        Tradeable         = 0x02,
        HiddenUntilEarned = 0x04,
        ResetsOnLogout    = 0x08,
        ConvertsToGold    = 0x10,
    };

    struct Entry {
        uint32_t tokenId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t category = Misc;
        uint32_t maxBalance = 0;        // 0 = unlimited
        uint32_t weeklyCap = 0;         // 0 = no cap
        uint32_t flags = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tokenId) const;

    static const char* categoryName(uint8_t c);
};

class WoweeTokenLoader {
public:
    static bool save(const WoweeToken& cat,
                     const std::string& basePath);
    static WoweeToken load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tokens* variants.
    //
    //   makeStarter - 3 tokens covering Pvp / Reputation /
    //                  Misc categories with realistic caps.
    //   makePvp     - full PvP currency set: Honor (75k cap)
    //                  + Arena Points (5k cap, weekly) + Marks
    //                  of Honor for 6 classic battlegrounds.
    //   makeSeasonal - 4 holiday-event tokens (Hallow's End
    //                   masks, Brewfest tokens, Winter's Veil
    //                   coins, etc.) all flagged ResetsOnLogout
    //                   to be event-bound.
    static WoweeToken makeStarter(const std::string& catalogName);
    static WoweeToken makePvp(const std::string& catalogName);
    static WoweeToken makeSeasonal(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
