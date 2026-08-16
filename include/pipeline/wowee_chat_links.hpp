#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Chat Hyperlink Templates catalog
// (.wlnk) - novel replacement for the implicit
// chat-link format templates vanilla WoW carried
// in client-side LUA (each link kind - item /
// quest / spell / achievement / talent / trade-
// skill - had a hard-coded sprintf template baked
// into ChatFrame_OnHyperlinkClick with no formal
// data-driven extension point). Each WLNK entry
// binds one hyperlink kind to its sprintf-style
// chat-link template, tooltip-popup template,
// quality color, and server-lookup requirement.
//
// Cross-references with previously-added formats:
//   WIT:  kind=Item link templates reference WIT
//         items by itemId (the first %d in the
//         template).
//   WSPL: kind=Spell templates reference WSPL
//         spells.
//   WQTM: kind=Quest templates reference WQTM
//         quests.
//   WACH: kind=Achievement templates reference
//         the achievements catalog.
//
// Binary layout (little-endian):
//   magic[4]            = "WLNK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     linkId (uint32)
//     nameLen + name
//     linkKind (uint8)             - 0=Item /
//                                     1=Quest /
//                                     2=Spell /
//                                     3=Achievement
//                                     /4=Talent /
//                                     5=Trade
//     requireServerLookup (uint8)  - 0/1 - client
//                                     calls server
//                                     to fetch
//                                     tooltip data
//     pad0 (uint16)
//     colorRGBA (uint32)           - link text
//                                     color (typical
//                                     quality color)
//     templateLen + linkTemplate   - sprintf with
//                                     %d / %s placeholders
//     tooltipLen + tooltipTemplate - first-line
//                                     tooltip header
//     iconRuleLen + iconRule       - string
//                                     describing
//                                     icon source
//                                     ("inv", "spell",
//                                     "achievement"
//                                     etc.)
struct WoweeChatLinks {
    enum LinkKind : uint8_t {
        Item        = 0,
        Quest       = 1,
        Spell       = 2,
        Achievement = 3,
        Talent      = 4,
        Trade       = 5,
    };

    struct Entry {
        uint32_t linkId = 0;
        std::string name;
        uint8_t linkKind = Item;
        uint8_t requireServerLookup = 0;
        uint16_t pad0 = 0;
        uint32_t colorRGBA = 0;
        std::string linkTemplate;
        std::string tooltipTemplate;
        std::string iconRule;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t linkId) const;

    // Returns the template for a given hyperlink
    // kind. Used by the chat-link composer when a
    // player shift-clicks an item/spell/quest into
    // chat - picks the matching template and
    // sprintf-fills the parameters.
    [[nodiscard]] const Entry* findByKind(uint8_t linkKind) const;
};

class WoweeChatLinksLoader {
public:
    static bool save(const WoweeChatLinks& cat,
                     const std::string& basePath);
    static WoweeChatLinks load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-lnk* variants.
    //
    //   makeStandardLinks - 4 standard link kinds
    //                        (Item with 4 %d for
    //                        rune slots / Quest /
    //                        Spell / Achievement)
    //                        with quality-color
    //                        defaults.
    //   makeTalentTrade   - 2 less-common link kinds
    //                        (Talent / Trade-skill
    //                        recipe) with green +
    //                        orange colors.
    //   makeColorVariants - 3 Item-kind variants for
    //                        different quality
    //                        colors (Common gray /
    //                        Epic purple / Legendary
    //                        orange) demonstrating
    //                        per-quality template
    //                        differentiation.
    static WoweeChatLinks makeStandardLinks(const std::string& catalogName);
    static WoweeChatLinks makeTalentTrade(const std::string& catalogName);
    static WoweeChatLinks makeColorVariants(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
