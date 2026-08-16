#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Macro / Slash Command catalog (.wmac) - novel
// open format with no direct DBC equivalent. WoW historically
// stored player macros client-side in the user profile and
// system slash commands as hardcoded engine handlers; this
// format unifies both into a single structured catalog so
// default macros, system slash commands, and shipped player
// presets can be authored, validated, and shipped as content.
//
// Each entry has a macro body (the actual `/cast Foo` text,
// multi-line allowed via `\n`), a kind classification
// (system slash / default macro / player template / guild /
// shared), a class-restriction mask, and an optional default
// key binding to fire the macro from the keyboard.
//
// Cross-references with previously-added formats:
//   WMAC.entry.requiredClassMask uses WCHC.classId bit
//                                positions (same convention as
//                                WGLY/WSET/WGTP).
//   The macroBody text is opaque to this catalog - the
//   client parses /cast / /target / /run lines at runtime
//   against WSPL spell names, action lists, and Lua scripting.
//
// Binary layout (little-endian):
//   magic[4]            = "WMAC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     macroId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     bodyLen + macroBody
//     bindLen + bindKey
//     macroKind (uint8) / pad[3]
//     requiredClassMask (uint32)
//     maxLength (uint16) / pad[2]
struct WoweeMacro {
    enum MacroKind : uint8_t {
        SystemSlash    = 0,    // /sit, /dance, /yell - engine handlers
        DefaultMacro   = 1,    // shipped with client, user can clone
        PlayerTemplate = 2,    // template for player-authored macros
        GuildMacro     = 3,    // guild-wide shared macro
        SharedMacro    = 4,    // account-shared (across alts)
    };

    struct Entry {
        uint32_t macroId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        std::string macroBody;        // can contain \n for multi-line
        std::string bindKey;          // empty = no default binding
        uint8_t macroKind = SystemSlash;
        uint32_t requiredClassMask = 0;  // 0 = any class
        uint16_t maxLength = 255;        // body size cap (UI hint)
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t macroId) const;

    static const char* macroKindName(uint8_t k);
};

class WoweeMacroLoader {
public:
    static bool save(const WoweeMacro& cat,
                     const std::string& basePath);
    static WoweeMacro load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mac* variants.
    //
    //   makeStarter - 3 system slash commands (/sit, /dance,
    //                  /target) - SystemSlash kind, no class
    //                  restriction.
    //   makeCombat  - 4 warrior combat macros (heroic strike
    //                  spam, charge, intercept-on-cooldown,
    //                  victory rush) - PlayerTemplate kind,
    //                  classMask = Warrior.
    //   makeUtility - 3 universal utility macros (/follow
    //                  target, mass invite, /releasecorpse)
    //                  - DefaultMacro kind, no class
    //                  restriction.
    static WoweeMacro makeStarter(const std::string& catalogName);
    static WoweeMacro makeCombat(const std::string& catalogName);
    static WoweeMacro makeUtility(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
