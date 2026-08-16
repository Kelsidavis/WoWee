#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Localization catalog (.wlan) - novel
// replacement for the per-language overlay tables that
// vanilla WoW carried as Locale_*.MPQ patches plus the
// Spell.dbc / Item.dbc trailing 16-locale string
// columns. Each entry binds one (originalKey,
// languageCode, namespace) triple to its localized
// translation.
//
// Cross-references with previously-added formats:
//   No catalog cross-references - WLAN is a pure
//   string-table overlay applied AFTER any per-format
//   catalog has resolved its primary text. The lookup
//   path is: format-default text -> WLAN override (if
//   client locale matches a WLAN entry) -> rendered
//   text.
//
// Binary layout (little-endian):
//   magic[4]            = "WLAN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     stringId (uint32)
//     nameLen + name (English / catalog label)
//     descLen + description (translator notes)
//     languageCode (uint8)       - enUS / enGB / deDE /
//                                   esES / frFR / itIT /
//                                   koKR / ptBR / ruRU /
//                                   zhCN / zhTW
//     namespace_ (uint8)         - UI / Quest / Item /
//                                   Spell / Creature /
//                                   Tooltip / Gossip /
//                                   System
//     pad0 (uint8) / pad1 (uint8)
//     keyLen + originalKey       - lookup key (canonical
//                                   form - usually the
//                                   English source text
//                                   or a dotted ID like
//                                   "QUEST.123.title")
//     locLen + localizedText     - translation in the
//                                   target language
//     iconColorRGBA (uint32)
struct WoweeLocalization {
    enum LanguageCode : uint8_t {
        enUS = 0,    // US English
        enGB = 1,    // UK English (variant of enUS for
                      // colour / armour / etc.)
        deDE = 2,    // German
        esES = 3,    // European Spanish
        frFR = 4,    // French
        itIT = 5,    // Italian
        koKR = 6,    // Korean
        ptBR = 7,    // Brazilian Portuguese
        ruRU = 8,    // Russian
        zhCN = 9,    // Simplified Chinese
        zhTW = 10,   // Traditional Chinese
        Unknown = 255,
    };

    enum Namespace : uint8_t {
        UI       = 0,    // button labels, menus
        Quest    = 1,    // quest titles + objective
                          // text
        Item     = 2,    // item names + descriptions
        Spell    = 3,    // spell names + tooltips
        Creature = 4,    // creature display names
        Tooltip  = 5,    // shared tooltip strings
        Gossip   = 6,    // NPC gossip dialog
        System   = 7,    // system messages /
                          // notifications
    };

    struct Entry {
        uint32_t stringId = 0;
        std::string name;
        std::string description;
        uint8_t languageCode = enUS;
        uint8_t namespace_ = UI;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        std::string originalKey;
        std::string localizedText;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t stringId) const;

    // Returns the localized override (if any) for a
    // given (key, language, namespace) lookup. Used at
    // every render-call by the locale-aware text layer.
    [[nodiscard]] const Entry* findOverride(const std::string& originalKey,
                                uint8_t languageCode,
                                uint8_t namespaceKind) const;

    // Returns all entries in one language. Used by the
    // per-language asset bundling step to package only
    // the strings the client needs.
    [[nodiscard]] std::vector<const Entry*> findByLanguage(uint8_t languageCode) const;
};

class WoweeLocalizationLoader {
public:
    static bool save(const WoweeLocalization& cat,
                     const std::string& basePath);
    static WoweeLocalization load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-lan* variants.
    //
    //   makeUIBasics    - 5 UI-button strings translated
    //                      to deDE/frFR/esES/koKR/zhCN
    //                      (5 entries × 1 key - the
    //                      "Cancel" button across 5
    //                      languages).
    //   makeQuestSample - 3 entries - one quest title
    //                      translated into deDE / frFR /
    //                      koKR.
    //   makeTooltipSet  - 4 item tooltip strings in
    //                      deDE + frFR (high-volume use
    //                      case for client localization).
    static WoweeLocalization makeUIBasics(const std::string& catalogName);
    static WoweeLocalization makeQuestSample(const std::string& catalogName);
    static WoweeLocalization makeTooltipSet(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
