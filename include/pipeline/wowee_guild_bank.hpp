#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Guild Bank Tabs catalog (.wgbk) -
// novel format providing what vanilla WoW lacked
// entirely: a guild-level shared storage facility
// (Blizzard added guild banks in TBC, but the
// Wowee project predates a TBC-specific binding -
// WGBK provides the catalog format from day one for
// the Classic-1.12 server flavor as well as later
// expansions). Each WGBK entry binds one guild bank
// tab to its display label, slot count, deposit-
// only flag, icon, and a per-guild-rank withdrawal
// limit array (slots/day cap per rank 0..7, where
// rank 0 is GuildMaster).
//
// Cross-references with previously-added formats:
//   WGLD: guildId references the WGLD guilds catalog.
//   WIT:  inventoried itemIds in tabs are looked up
//         against WIT (the runtime structure for tab
//         contents lives elsewhere - WGBK only
//         describes the tab schema, not the items).
//
// Binary layout (little-endian):
//   magic[4]            = "WGBK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tabId (uint32)         - surrogate primary key
//     guildId (uint32)       - owning guild
//     tabNameLen + tabName
//     iconIndex (uint32)     - ItemDisplayInfo row id
//     depositOnly (uint8)    - 0/1 bool - non-rank-0
//                               can deposit but not
//                               withdraw
//     pad0 (uint8)
//     slotCount (uint16)     - 1..98 (vanilla cap)
//     perRankWithdrawalLimit[8] (uint32) - slots/day
//                               per rank 0..7,
//                               0xFFFFFFFF =
//                               unlimited, 0 = no
//                               withdraw access.
//                               Index 0 is GuildMaster.
struct WoweeGuildBank {
    static constexpr uint32_t kRankCount = 8;
    static constexpr uint32_t kUnlimited = 0xFFFFFFFFu;
    static constexpr uint16_t kMaxSlots = 98;

    struct Entry {
        uint32_t tabId = 0;
        uint32_t guildId = 0;
        std::string tabName;
        uint32_t iconIndex = 0;
        uint8_t depositOnly = 0;
        uint8_t pad0 = 0;
        uint16_t slotCount = 0;
        uint32_t perRankWithdrawalLimit[kRankCount] = {0,0,0,0,0,0,0,0};
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tabId) const;

    // Returns all bank tabs owned by a guild - used
    // by the guild-bank UI to populate the per-guild
    // tab strip.
    [[nodiscard]] std::vector<const Entry*> findByGuild(uint32_t guildId) const;
};

class WoweeGuildBankLoader {
public:
    static bool save(const WoweeGuildBank& cat,
                     const std::string& basePath);
    static WoweeGuildBank load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-gbk* variants.
    //
    //   makeStandardBank - 4 tabs for guildId 1
    //                       (General/Materials/
    //                       Consumables/Officer).
    //                       Officer tab only
    //                       withdrawable by ranks
    //                       0-2.
    //   makeRaidGuild    - 5 tabs (Tier1/Tier2/Tier3
    //                       gear sets, Consumables,
    //                       Officer). High slot
    //                       counts on tier tabs.
    //   makeSmallGuild   - 2 tabs (General + Officer)
    //                       with tight per-rank
    //                       withdrawal limits - all
    //                       non-officer ranks capped
    //                       at 5 slots/day.
    static WoweeGuildBank makeStandardBank(const std::string& catalogName);
    static WoweeGuildBank makeRaidGuild(const std::string& catalogName);
    static WoweeGuildBank makeSmallGuild(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
