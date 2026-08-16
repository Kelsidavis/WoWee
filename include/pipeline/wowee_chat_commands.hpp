#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Chat Slash Commands catalog (.wcmd)
// - novel replacement for the implicit slash-
// command registry vanilla WoW carried in the
// client's ChatFrame.lua + server-side per-command
// CommandHandler hooks (no formal data-driven
// catalog; commands were registered ad-hoc with
// hard-coded security checks scattered across
// LevelMgr / WorldMgr / CharacterMgr). Each WCMD
// entry binds one command name (e.g. "who",
// "played", "announce") to its aliases, minimum
// security level required, argument schema string,
// help text, per-player throttle, hidden flag (for
// debug-only commands), and category.
//
// Cross-references with previously-added formats:
//   None directly - commands are dispatched by
//   handlerKey hash to server-side handlers, but the
//   handler code itself lives outside the catalog.
//
// Binary layout (little-endian):
//   magic[4]            = "WCMD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     cmdId (uint32)
//     commandLen + command (the canonical /name)
//     minSecurityLevel (uint8)     - 0=Player /
//                                     1=Helper /
//                                     2=Moderator /
//                                     3=GameMaster /
//                                     4=Admin
//     category (uint8)             - 0=Info /
//                                     1=Movement /
//                                     2=Communication
//                                     /3=Admin /
//                                     4=Debug
//     isHidden (uint8)             - 0/1 - debug-
//                                     only commands
//                                     hidden from
//                                     /help listing
//     pad0 (uint8)
//     throttleMs (uint32)          - per-player rate
//                                     limit (0 = no
//                                     throttle)
//     argSchemaLen + argSchema
//     helpTextLen + helpText
//     aliasesCount (uint32)
//     aliases (each: stringLen + string)
struct WoweeChatCommands {
    enum SecurityLevel : uint8_t {
        Player     = 0,
        Helper     = 1,
        Moderator  = 2,
        GameMaster = 3,
        Admin      = 4,
    };

    enum Category : uint8_t {
        Info          = 0,
        Movement      = 1,
        Communication = 2,
        AdminCmd      = 3,
        Debug         = 4,
    };

    struct Entry {
        uint32_t cmdId = 0;
        std::string command;
        uint8_t minSecurityLevel = Player;
        uint8_t category = Info;
        uint8_t isHidden = 0;
        uint8_t pad0 = 0;
        uint32_t throttleMs = 0;
        std::string argSchema;
        std::string helpText;
        std::vector<std::string> aliases;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t cmdId) const;

    // Resolves a chat command by typed string -
    // matches against canonical command name OR
    // any alias. Used by the chat parser hot path.
    [[nodiscard]] const Entry* findByCommand(const std::string& cmd) const;

    // Returns all commands a player at the given
    // security level can use. The /help UI calls
    // this with the player's security level filter.
    [[nodiscard]] std::vector<const Entry*> findByMinSecurity(uint8_t playerSec) const;
};

class WoweeChatCommandsLoader {
public:
    static bool save(const WoweeChatCommands& cat,
                     const std::string& basePath);
    static WoweeChatCommands load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cmd* variants.
    //
    //   makeBasicCommands  - 4 standard player-info
    //                         commands (/who /played
    //                         /time /ginfo) all at
    //                         Player security level,
    //                         no throttle.
    //   makeMovementCommands - 3 emote-style commands
    //                         (/sit /stand /sleep)
    //                         with short aliases.
    //   makeAdminCommands  - 3 admin-only commands
    //                         (/announce /kick /ban)
    //                         at GameMaster security
    //                         level with throttling.
    static WoweeChatCommands makeBasicCommands(const std::string& catalogName);
    static WoweeChatCommands makeMovementCommands(const std::string& catalogName);
    static WoweeChatCommands makeAdminCommands(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
