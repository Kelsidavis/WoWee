#include "pipeline/wowee_locks.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'L', 'C', 'K'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wlck";
// SkillLine canonical ID for lockpicking (matches AzerothCore).
constexpr uint32_t kLockpickingSkill = 633;

} // namespace

const WoweeLock::Entry* WoweeLock::findById(uint32_t lockId) const {
    for (const auto& e : entries) {
        if (e.lockId == lockId) return &e;
    }
    return nullptr;
}

const char* WoweeLock::channelKindName(uint8_t k) {
    switch (k) {
        case ChannelNone:     return "-";
        case ChannelItem:     return "item";
        case ChannelLockpick: return "lockpick";
        case ChannelSpell:    return "spell";
        case ChannelDamage:   return "damage";
        default:              return "?";
    }
}

bool WoweeLockLoader::save(const WoweeLock& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeLock::Entry& e) {
        writePOD(os, e.lockId);
        writeStr(os, e.name);
        writePOD(os, e.flags);
        for (auto ch : e.channels) {
            writePOD(os, ch.kind);
            writePadding(os, 1);
            writePOD(os, ch.skillRequired);
            writePOD(os, ch.targetId);
        }
                       });
}

WoweeLock WoweeLockLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeLock>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeLock::Entry& e) {
        if (!readPOD(is, e.lockId)) { return false; }
        if (!readStr(is, e.name)) { return false; }
        if (!readPOD(is, e.flags)) { return false; }
        for (auto& ch : e.channels) {
            if (!readPOD(is, ch.kind)) { return false; }
            if (!skipPadding(is, 1)) { return false; }
            if (!readPOD(is, ch.skillRequired) ||
                !readPOD(is, ch.targetId)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeLockLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeLock WoweeLockLoader::makeStarter(const std::string& catalogName) {
    WoweeLock c;
    c.name = catalogName;
    {
        // lockId 1 matches the WGOT.makeDungeon Iron Door lock.
        WoweeLock::Entry e;
        e.lockId = 1; e.name = "Iron Door Lock";
        e.channels[0] = {.kind = WoweeLock::ChannelItem, .skillRequired = 0, .targetId = 5001};   // requires key item 5001
        e.channels[1] = {.kind = WoweeLock::ChannelDamage, .skillRequired = 0, .targetId = 0};    // OR force open
        c.entries.push_back(e);
    }
    {
        WoweeLock::Entry e;
        e.lockId = 100; e.name = "Wooden Chest Lock";
        e.channels[0] = {.kind = WoweeLock::ChannelDamage, .skillRequired = 0, .targetId = 0};   // forceable
        c.entries.push_back(e);
    }
    return c;
}

WoweeLock WoweeLockLoader::makeDungeon(const std::string& catalogName) {
    WoweeLock c;
    c.name = catalogName;
    {
        // lockId 2 matches WGOT.makeDungeon's bandit strongbox.
        WoweeLock::Entry e;
        e.lockId = 2; e.name = "Light Bandit Strongbox";
        e.channels[0] = {.kind = WoweeLock::ChannelLockpick,
                          .skillRequired = 1, .targetId = kLockpickingSkill};   // any skill rank
        c.entries.push_back(e);
    }
    {
        WoweeLock::Entry e;
        e.lockId = 200; e.name = "Steel Chest Lock";
        // Either heavy lockpick OR a specific key.
        e.channels[0] = {.kind = WoweeLock::ChannelLockpick,
                          .skillRequired = 175, .targetId = kLockpickingSkill};
        e.channels[1] = {.kind = WoweeLock::ChannelItem, .skillRequired = 0, .targetId = 5101};
        c.entries.push_back(e);
    }
    {
        WoweeLock::Entry e;
        e.lockId = 300; e.name = "Boss Vault Seal";
        e.flags = WoweeLock::DestructOnOpen;
        // Quest key only - no lockpick option (story-gated).
        e.channels[0] = {.kind = WoweeLock::ChannelItem, .skillRequired = 0, .targetId = 5200};
        c.entries.push_back(e);
    }
    return c;
}

WoweeLock WoweeLockLoader::makeProfessions(const std::string& catalogName) {
    WoweeLock c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint16_t skillReq) {
        WoweeLock::Entry e;
        e.lockId = id; e.name = name;
        e.channels[0] = {.kind = WoweeLock::ChannelLockpick,
                          .skillRequired = skillReq, .targetId = kLockpickingSkill};
        c.entries.push_back(e);
    };
    add(401, "Battered Junkbox",     1);
    add(402, "Worn Junkbox",       100);
    add(403, "Sturdy Junkbox",     175);
    add(404, "Heavy Junkbox",      250);
    return c;
}

} // namespace pipeline
} // namespace wowee
