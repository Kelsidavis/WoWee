#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Guild catalog (.wgld) - novel replacement for
// AzerothCore-style guild + guild_member + guild_rank +
// guild_bank_tab + guild_perk SQL tables. The 36th open
// format added to the editor.
//
// Each guild entry holds:
//   • header   - id, name, faction, leader, MOTD, info,
//                 creation date, level + experience, bank
//                 money, emblem (packed style/color/border/bg)
//   • ranks    - rank ladder (GM down to Initiate) with
//                 permissions bitmask + daily money withdraw
//   • members  - character roster with rank, join date,
//                 public + officer notes
//   • bankTabs - per-tab name, icon, deposit/withdraw/view
//                 permission masks indexed by rank
//   • perks    - purchased guild-level perks (cata-era spell
//                 buffs); each perk references a WSPL spell
//
// Cross-references with previously-added formats:
//   WGLD.entry.factionId         → matches WCHC.race.factionId
//                                  (guilds are faction-locked)
//   WGLD.entry.perks.spellId     → WSPL.entry.spellId
//
// Binary layout (little-endian):
//   magic[4]            = "WGLD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     guildId (uint32)
//     nameLen + name
//     leaderLen + leaderName
//     motdLen + motd
//     infoLen + info
//     creationDate (uint64)
//     experience (uint64)
//     level (uint16) / factionId (uint8) / pad[1]
//     bankCopper (uint32)
//     emblem (uint32)
//     rankCount (uint8) + ranks[]
//     memberCount (uint16) + members[]
//     bankTabCount (uint8) + bankTabs[]
//     perkCount (uint8) + perks[]
struct WoweeGuild {
    enum Faction : uint8_t {
        Alliance = 0,
        Horde    = 1,
    };

    enum RankPermissionFlags : uint32_t {
        PermGuildChat       = 0x0001,
        PermOfficerChat     = 0x0002,
        PermInvite          = 0x0004,
        PermRemove          = 0x0008,
        PermPromote         = 0x0010,
        PermDemote          = 0x0020,
        PermSetMotd         = 0x0040,
        PermEditPublicNote  = 0x0080,
        PermEditOfficerNote = 0x0100,
        PermViewBank        = 0x0200,
        PermDeposit         = 0x0400,
        PermWithdraw        = 0x0800,
        PermDisband         = 0x1000,
        PermRepairFromBank  = 0x2000,
    };

    struct Rank {
        uint8_t rankIndex = 0;             // 0 = GuildMaster
        std::string name;
        uint32_t permissionsMask = 0;
        uint32_t moneyPerDayCopper = 0;
    };

    struct Member {
        std::string characterName;
        uint8_t rankIndex = 0;
        uint64_t joinedDate = 0;           // Unix timestamp seconds
        std::string publicNote;
        std::string officerNote;
    };

    struct BankTab {
        uint8_t tabIndex = 0;
        std::string name;
        std::string iconPath;
        uint32_t depositPermissionMask = 0;     // bit per rank
        uint32_t withdrawPermissionMask = 0;
        uint32_t viewPermissionMask = 0;
    };

    struct Perk {
        uint32_t perkId = 0;
        std::string name;
        uint32_t spellId = 0;               // WSPL cross-ref
        uint16_t requiredGuildLevel = 1;
    };

    struct Entry {
        uint32_t guildId = 0;
        std::string name;
        std::string leaderName;
        std::string motd;
        std::string info;
        uint64_t creationDate = 0;
        uint64_t experience = 0;
        uint16_t level = 1;
        uint8_t factionId = Alliance;
        uint32_t bankCopper = 0;
        uint32_t emblem = 0;                // packed style/color/border/bg
        std::vector<Rank> ranks;
        std::vector<Member> members;
        std::vector<BankTab> bankTabs;
        std::vector<Perk> perks;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t guildId) const;

    static const char* factionName(uint8_t f);
};

class WoweeGuildLoader {
public:
    static bool save(const WoweeGuild& cat,
                     const std::string& basePath);
    static WoweeGuild load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-guilds* variants.
    //
    //   makeStarter - 1 small guild with default 5-rank ladder
    //                  (GM/Officer/Veteran/Member/Initiate),
    //                  3 members, no bank tabs.
    //   makeFull    - 1 fleshed-out guild with 6 ranks, 8
    //                  members, 4 bank tabs (each with
    //                  per-rank permission masks), 3 purchased
    //                  perks referencing WSPL spell IDs.
    //   makeFactionPair - 2 guilds, one Alliance + one Horde,
    //                      with parallel rank structures.
    static WoweeGuild makeStarter(const std::string& catalogName);
    static WoweeGuild makeFull(const std::string& catalogName);
    static WoweeGuild makeFactionPair(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
