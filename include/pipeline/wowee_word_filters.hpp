#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Word Filter catalog (.wwfl) - novel
// replacement for the implicit chat-moderation patterns
// vanilla WoW carried in the bad-word checker (the
// hardcoded substring list the CMSG_MESSAGECHAT
// handler walked before broadcasting). Each entry
// defines one pattern the chat preprocessor matches
// against outbound messages, the replacement to apply
// (or "drop" / "warn" / "mute" the sender), and the
// filter kind for analytics.
//
// This catalog is intentionally non-profanity focused:
// the ecosystem distributes through CI / public PRs
// where embedded profanity would create reviewer-
// experience and licensing concerns. The included
// presets target SPAM, RMT (real-money-transfer
// solicitations), URL leakage, and all-caps abuse -
// the moderation surfaces server admins actually need.
// Profanity-list integration is left to deployment-
// time configuration where local laws and community
// standards apply.
//
// Cross-references with previously-added formats:
//   WCHN: filters apply per-channel; the chat
//         preprocessor checks channel kind from WCHN
//         to decide whether profanity rules apply.
//
// Binary layout (little-endian):
//   magic[4]            = "WWFL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     filterId (uint32)
//     nameLen + name
//     descLen + description
//     patLen + pattern
//     replLen + replacement
//     filterKind (uint8)         - Spam / GoldSeller /
//                                   AllCaps / RepeatChar
//                                   / URL / AdvertReward
//                                   / Misc
//     severity (uint8)           - Warn / Replace /
//                                   Drop / Mute
//     caseSensitive (uint8)      - 0/1 bool
//     pad0 (uint8)
//     iconColorRGBA (uint32)
struct WoweeWordFilters {
    enum FilterKind : uint8_t {
        Spam         = 0,    // generic noise patterns
        GoldSeller   = 1,    // RMT solicitations
        AllCaps      = 2,    // shouting detection
        RepeatChar   = 3,    // spam-mash detection
                              // (e.g. "aaaaaaaaaa")
        URL          = 4,    // URL leakage
        AdvertReward = 5,    // "FREE GOLD" / contest
                              // adverts
        Misc         = 255,
    };

    enum Severity : uint8_t {
        Warn    = 0,    // log + warn the sender; let
                         // message through
        Replace = 1,    // substitute the matched portion
                         // and forward
        Drop    = 2,    // silently discard the message
        Mute    = 3,    // drop AND mute the sender for
                         // a configured duration
    };

    struct Entry {
        uint32_t filterId = 0;
        std::string name;
        std::string description;
        std::string pattern;          // substring to match
        std::string replacement;      // for Replace
        uint8_t filterKind = Spam;
        uint8_t severity = Warn;
        uint8_t caseSensitive = 0;
        uint8_t pad0 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t filterId) const;

    // Returns all filters of one kind - used by the
    // chat preprocessor to dispatch per-kind handlers
    // (URL kind hits the link expander, AllCaps kind
    // hits the shout-suppressor, etc.).
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t filterKind) const;
};

class WoweeWordFiltersLoader {
public:
    static bool save(const WoweeWordFilters& cat,
                     const std::string& basePath);
    static WoweeWordFilters load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-wfl* variants.
    //
    //   makeSpamRMT    - 5 RMT / spam patterns ("wts
    //                     gold", "wtb gold", typo-
    //                     substituted "g0ld", "1000g",
    //                     "free gold").
    //   makeAllCaps    - 3 all-caps detection patterns
    //                     (10+ uppercase chars, !!! at
    //                     line end, $$$ symbols).
    //   makeURLDetect  - 3 URL leakage patterns
    //                     (http://, www., suspicious
    //                     TLDs).
    static WoweeWordFilters makeSpamRMT(const std::string& catalogName);
    static WoweeWordFilters makeAllCaps(const std::string& catalogName);
    static WoweeWordFilters makeURLDetect(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
