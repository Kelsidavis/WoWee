#include "pipeline/wowee_guilds.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'G', 'L', 'D'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wgld";

} // namespace

const WoweeGuild::Entry* WoweeGuild::findById(uint32_t guildId) const {
    for (const auto& e : entries) if (e.guildId == guildId) return &e;
    return nullptr;
}

const char* WoweeGuild::factionName(uint8_t f) {
    switch (f) {
        case Alliance: return "alliance";
        case Horde:    return "horde";
        default:       return "unknown";
    }
}

bool WoweeGuildLoader::save(const WoweeGuild& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeGuild::Entry& e) {
        writePOD(os, e.guildId);
        writeStr(os, e.name);
        writeStr(os, e.leaderName);
        writeStr(os, e.motd);
        writeStr(os, e.info);
        writePOD(os, e.creationDate);
        writePOD(os, e.experience);
        writePOD(os, e.level);
        writePOD(os, e.factionId);
        writePadding(os, 1);
        writePOD(os, e.bankCopper);
        writePOD(os, e.emblem);

        uint8_t rankCount = static_cast<uint8_t>(
            e.ranks.size() > 255 ? 255 : e.ranks.size());
        writePOD(os, rankCount);
        for (uint8_t k = 0; k < rankCount; ++k) {
            const auto& r = e.ranks[k];
            writePOD(os, r.rankIndex);
            writePadding(os, 3);
            writeStr(os, r.name);
            writePOD(os, r.permissionsMask);
            writePOD(os, r.moneyPerDayCopper);
        }
        uint16_t memCount = static_cast<uint16_t>(
            e.members.size() > 0xFFFF ? 0xFFFF : e.members.size());
        writePOD(os, memCount);
        for (uint16_t k = 0; k < memCount; ++k) {
            const auto& m = e.members[k];
            writeStr(os, m.characterName);
            writePOD(os, m.rankIndex);
            writePadding(os, 7);
            writePOD(os, m.joinedDate);
            writeStr(os, m.publicNote);
            writeStr(os, m.officerNote);
        }
        uint8_t tabCount = static_cast<uint8_t>(
            e.bankTabs.size() > 255 ? 255 : e.bankTabs.size());
        writePOD(os, tabCount);
        for (uint8_t k = 0; k < tabCount; ++k) {
            const auto& t = e.bankTabs[k];
            writePOD(os, t.tabIndex);
            writePadding(os, 3);
            writeStr(os, t.name);
            writeStr(os, t.iconPath);
            writePOD(os, t.depositPermissionMask);
            writePOD(os, t.withdrawPermissionMask);
            writePOD(os, t.viewPermissionMask);
        }
        uint8_t perkCount = static_cast<uint8_t>(
            e.perks.size() > 255 ? 255 : e.perks.size());
        writePOD(os, perkCount);
        for (uint8_t k = 0; k < perkCount; ++k) {
            const auto& p = e.perks[k];
            writePOD(os, p.perkId);
            writeStr(os, p.name);
            writePOD(os, p.spellId);
            writePOD(os, p.requiredGuildLevel);
            writePadding(os, 2);
        }
                       });
}

WoweeGuild WoweeGuildLoader::load(const std::string& basePath) {
    WoweeGuild out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    uint32_t entryCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, entryCount)) return out;
    out.entries.resize(entryCount);
    auto fail = [&]() {
        out.entries.clear();
        return out;
    };
    for (auto& e : out.entries) {
        if (!readPOD(is, e.guildId)) return fail();
        if (!readStr(is, e.name) || !readStr(is, e.leaderName) ||
            !readStr(is, e.motd) || !readStr(is, e.info)) return fail();
        if (!readPOD(is, e.creationDate) ||
            !readPOD(is, e.experience) ||
            !readPOD(is, e.level) ||
            !readPOD(is, e.factionId)) return fail();
        if (!skipPadding(is, 1)) return fail();
        if (!readPOD(is, e.bankCopper) || !readPOD(is, e.emblem)) return fail();

        uint8_t rankCount = 0;
        if (!readPOD(is, rankCount)) return fail();
        e.ranks.resize(rankCount);
        for (uint8_t k = 0; k < rankCount; ++k) {
            auto& r = e.ranks[k];
            if (!readPOD(is, r.rankIndex)) return fail();
            if (!skipPadding(is, 3)) return fail();
            if (!readStr(is, r.name)) return fail();
            if (!readPOD(is, r.permissionsMask) ||
                !readPOD(is, r.moneyPerDayCopper)) return fail();
        }
        uint16_t memCount = 0;
        if (!readPOD(is, memCount)) return fail();
        e.members.resize(memCount);
        for (uint16_t k = 0; k < memCount; ++k) {
            auto& m = e.members[k];
            if (!readStr(is, m.characterName)) return fail();
            if (!readPOD(is, m.rankIndex)) return fail();
            if (!skipPadding(is, 7)) return fail();
            if (!readPOD(is, m.joinedDate)) return fail();
            if (!readStr(is, m.publicNote) || !readStr(is, m.officerNote)) return fail();
        }
        uint8_t tabCount = 0;
        if (!readPOD(is, tabCount)) return fail();
        e.bankTabs.resize(tabCount);
        for (uint8_t k = 0; k < tabCount; ++k) {
            auto& t = e.bankTabs[k];
            if (!readPOD(is, t.tabIndex)) return fail();
            if (!skipPadding(is, 3)) return fail();
            if (!readStr(is, t.name) || !readStr(is, t.iconPath)) return fail();
            if (!readPOD(is, t.depositPermissionMask) ||
                !readPOD(is, t.withdrawPermissionMask) ||
                !readPOD(is, t.viewPermissionMask)) return fail();
        }
        uint8_t perkCount = 0;
        if (!readPOD(is, perkCount)) return fail();
        e.perks.resize(perkCount);
        for (uint8_t k = 0; k < perkCount; ++k) {
            auto& p = e.perks[k];
            if (!readPOD(is, p.perkId)) return fail();
            if (!readStr(is, p.name)) return fail();
            if (!readPOD(is, p.spellId) ||
                !readPOD(is, p.requiredGuildLevel)) return fail();
            if (!skipPadding(is, 2)) return fail();
        }
    }
    return out;
}

bool WoweeGuildLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

namespace {

// Default 5-rank ladder used by both starter + faction-pair
// presets. Permissions widen toward the GM end of the ladder.
void addDefaultRanks(WoweeGuild::Entry& e) {
    using G = WoweeGuild;
    e.ranks.push_back({.rankIndex = 0, .name = "Guild Master",
                        .permissionsMask = 0xFFFFFFFFu, .moneyPerDayCopper = 1000000});   // 100g/day
    e.ranks.push_back({.rankIndex = 1, .name = "Officer",
                        .permissionsMask = G::PermGuildChat | G::PermOfficerChat |
                        G::PermInvite | G::PermRemove |
                        G::PermPromote | G::PermDemote |
                        G::PermSetMotd | G::PermViewBank |
                        G::PermDeposit | G::PermWithdraw,
                        .moneyPerDayCopper = 500000});   // 50g/day
    e.ranks.push_back({.rankIndex = 2, .name = "Veteran",
                        .permissionsMask = G::PermGuildChat | G::PermInvite |
                        G::PermViewBank | G::PermDeposit |
                        G::PermWithdraw,
                        .moneyPerDayCopper = 100000});   // 10g/day
    e.ranks.push_back({.rankIndex = 3, .name = "Member",
                        .permissionsMask = G::PermGuildChat | G::PermViewBank |
                        G::PermDeposit,
                        .moneyPerDayCopper = 10000});    // 1g/day
    e.ranks.push_back({.rankIndex = 4, .name = "Initiate",
                        .permissionsMask = G::PermGuildChat,
                        .moneyPerDayCopper = 0});
}

} // namespace

WoweeGuild WoweeGuildLoader::makeStarter(const std::string& catalogName) {
    WoweeGuild c;
    c.name = catalogName;
    {
        WoweeGuild::Entry e;
        e.guildId = 1; e.name = "Sentinels of Dawn";
        e.leaderName = "Bartleby";
        e.motd = "Welcome adventurer! Read the info tab.";
        e.info = "Casual leveling guild. All are welcome.";
        e.factionId = WoweeGuild::Alliance;
        e.level = 1;
        addDefaultRanks(e);
        e.members.push_back({.characterName = "Bartleby",   .rankIndex = 0, .joinedDate = 0,
                              .publicNote = "Founder", .officerNote = "Owns the inn"});
        e.members.push_back({.characterName = "Hank Steelarm", .rankIndex = 1, .joinedDate = 0,
                              .publicNote = "Smith", .officerNote = "Friendly officer"});
        e.members.push_back({.characterName = "Sera Goldroot", .rankIndex = 3, .joinedDate = 0,
                              .publicNote = "Alchemist", .officerNote = ""});
        c.entries.push_back(e);
    }
    return c;
}

WoweeGuild WoweeGuildLoader::makeFull(const std::string& catalogName) {
    WoweeGuild c;
    c.name = catalogName;
    {
        WoweeGuild::Entry e;
        e.guildId = 100; e.name = "Defenders of Stormwind";
        e.leaderName = "Lord Tideborne";
        e.motd = "Raid week starts Tuesday at 8 PM server.";
        e.info = "Heroic raiding guild. Apply on the website.";
        e.factionId = WoweeGuild::Alliance;
        e.level = 25;
        e.experience = 1500000;
        e.bankCopper = 50000000;     // 5000g in bank
        e.emblem = 0x12345678;
        // 6 ranks: GM + Officer + 2 Council tiers + Member + Initiate.
        addDefaultRanks(e);
        e.ranks.push_back({.rankIndex = 5, .name = "Recruit",
                            .permissionsMask = WoweeGuild::PermGuildChat,
                            .moneyPerDayCopper = 0});
        for (int k = 0; k < 8; ++k) {
            WoweeGuild::Member m;
            m.characterName = "Officer" + std::to_string(k);
            m.rankIndex = static_cast<uint8_t>(k % 6);
            m.joinedDate = 1700000000 + k * 86400;
            e.members.push_back(m);
        }
        // 4 bank tabs with progressively more restrictive
        // withdraw permissions (officers only on tabs 3 + 4).
        for (int k = 0; k < 4; ++k) {
            WoweeGuild::BankTab t;
            t.tabIndex = static_cast<uint8_t>(k);
            t.name = "Tab " + std::to_string(k + 1);
            // Bit per rank (rank 0 = bit 0 etc).
            t.depositPermissionMask  = 0x3F;       // ranks 0-5
            t.viewPermissionMask     = 0x3F;
            t.withdrawPermissionMask = (k < 2) ? 0x3F : 0x03;  // tabs 3+4 = GM/Officer only
            e.bankTabs.push_back(t);
        }
        // 3 perks referencing WSPL spell IDs from makeMage / generic.
        e.perks.push_back({.perkId = 1, .name = "Fast Track",     .spellId = 78,    .requiredGuildLevel = 1});  // Heroic Strike (placeholder)
        e.perks.push_back({.perkId = 2, .name = "Cash Flow",      .spellId = 6673,  .requiredGuildLevel = 10}); // Battle Shout (placeholder)
        e.perks.push_back({.perkId = 3, .name = "Reinforce",      .spellId = 6343,  .requiredGuildLevel = 20}); // Thunder Clap (placeholder)
        c.entries.push_back(e);
    }
    return c;
}

WoweeGuild WoweeGuildLoader::makeFactionPair(const std::string& catalogName) {
    WoweeGuild c;
    c.name = catalogName;
    {
        WoweeGuild::Entry e;
        e.guildId = 200; e.name = "Light's Vanguard";
        e.leaderName = "Lothar Crownguard";
        e.factionId = WoweeGuild::Alliance;
        addDefaultRanks(e);
        e.members.push_back({.characterName = "Lothar Crownguard", .rankIndex = 0, .joinedDate = 0, .publicNote = "GM", .officerNote = ""});
        c.entries.push_back(e);
    }
    {
        WoweeGuild::Entry e;
        e.guildId = 201; e.name = "Bloodfang Warband";
        e.leaderName = "Garrok Bloodfang";
        e.factionId = WoweeGuild::Horde;
        addDefaultRanks(e);
        e.members.push_back({.characterName = "Garrok Bloodfang", .rankIndex = 0, .joinedDate = 0, .publicNote = "Chieftain", .officerNote = ""});
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
