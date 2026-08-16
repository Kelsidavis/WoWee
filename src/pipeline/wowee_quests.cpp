#include "pipeline/wowee_quests.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'Q', 'T', 'M'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wqt";

} // namespace

const WoweeQuest::Entry* WoweeQuest::findById(uint32_t questId) const {
    for (const auto& e : entries) {
        if (e.questId == questId) return &e;
    }
    return nullptr;
}

const char* WoweeQuest::objectiveKindName(uint8_t k) {
    switch (k) {
        case KillCreature:   return "kill";
        case CollectItem:    return "collect";
        case InteractObject: return "interact";
        case VisitArea:      return "visit";
        case EscortNpc:      return "escort";
        case SpellCast:      return "cast";
        default:             return "unknown";
    }
}

bool WoweeQuestLoader::save(const WoweeQuest& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeQuest::Entry& e) {
        writePOD(os, e.questId);
        writeStr(os, e.title);
        writeStr(os, e.objective);
        writeStr(os, e.description);
        writePOD(os, e.minLevel);
        writePOD(os, e.questLevel);
        writePOD(os, e.maxLevel);
        uint16_t pad16 = 0;
        writePOD(os, pad16);
        writePOD(os, e.requiredClassMask);
        writePOD(os, e.requiredRaceMask);
        writePOD(os, e.prevQuestId);
        writePOD(os, e.nextQuestId);
        writePOD(os, e.giverCreatureId);
        writePOD(os, e.turninCreatureId);
        uint8_t objCount = static_cast<uint8_t>(
            e.objectives.size() > 255 ? 255 : e.objectives.size());
        writePOD(os, objCount);
        writePadding(os, 3);
        for (uint8_t k = 0; k < objCount; ++k) {
            const auto& o = e.objectives[k];
            writePOD(os, o.kind);
            writePadding(os, 3);
            writePOD(os, o.targetId);
            writePOD(os, o.quantity);
            writePOD(os, pad16);
        }
        writePOD(os, e.xpReward);
        writePOD(os, e.moneyCopperReward);
        uint8_t rewCount = static_cast<uint8_t>(
            e.rewardItems.size() > 255 ? 255 : e.rewardItems.size());
        writePOD(os, rewCount);
        writePadding(os, 3);
        for (uint8_t k = 0; k < rewCount; ++k) {
            const auto& r = e.rewardItems[k];
            writePOD(os, r.itemId);
            writePOD(os, r.qty);
            writePOD(os, r.pickFlags);
            writePOD(os, pad16);
        }
        writePOD(os, e.flags);
                       });
}

WoweeQuest WoweeQuestLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeQuest>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeQuest::Entry& e) {
        if (!readPOD(is, e.questId)) { return false; }
        if (!readStr(is, e.title) || !readStr(is, e.objective) ||
            !readStr(is, e.description)) { return false; }
        if (!readPOD(is, e.minLevel) ||
            !readPOD(is, e.questLevel) ||
            !readPOD(is, e.maxLevel)) { return false; }
        uint16_t pad16 = 0;
        if (!readPOD(is, pad16)) { return false; }
        if (!readPOD(is, e.requiredClassMask) ||
            !readPOD(is, e.requiredRaceMask) ||
            !readPOD(is, e.prevQuestId) ||
            !readPOD(is, e.nextQuestId) ||
            !readPOD(is, e.giverCreatureId) ||
            !readPOD(is, e.turninCreatureId)) { return false; }
        uint8_t objCount = 0;
        if (!readPOD(is, objCount)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        e.objectives.resize(objCount);
        for (uint8_t k = 0; k < objCount; ++k) {
            auto& o = e.objectives[k];
            if (!readPOD(is, o.kind)) { return false; }
            if (!skipPadding(is, 3)) { return false; }
            if (!readPOD(is, o.targetId) || !readPOD(is, o.quantity)) { return false; }
            uint16_t opad = 0;
            if (!readPOD(is, opad)) { return false; }
        }
        if (!readPOD(is, e.xpReward) ||
            !readPOD(is, e.moneyCopperReward)) { return false; }
        uint8_t rewCount = 0;
        if (!readPOD(is, rewCount)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        e.rewardItems.resize(rewCount);
        for (uint8_t k = 0; k < rewCount; ++k) {
            auto& r = e.rewardItems[k];
            if (!readPOD(is, r.itemId) ||
                !readPOD(is, r.qty) ||
                !readPOD(is, r.pickFlags)) { return false; }
            uint16_t rpad = 0;
            if (!readPOD(is, rpad)) { return false; }
        }
        if (!readPOD(is, e.flags)) { return false; }
                                  return true;
                              });
}

bool WoweeQuestLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeQuest WoweeQuestLoader::makeStarter(const std::string& catalogName) {
    WoweeQuest c;
    c.name = catalogName;
    {
        WoweeQuest::Entry e;
        e.questId = 1;
        e.title = "Bandit Trouble";
        e.objective = "Kill 10 Defias Bandits.";
        e.description =
            "The Defias have been raiding our farms. Make them pay.";
        e.minLevel = 5;
        e.questLevel = 6;
        e.giverCreatureId = 4001;          // village innkeeper from WCRT
        e.turninCreatureId = 4001;
        e.objectives.push_back({.kind = WoweeQuest::KillCreature, .targetId = 1000, .quantity = 10});
        e.xpReward = 500;
        e.moneyCopperReward = 250;          // 2s 50c
        e.rewardItems.push_back({.itemId = 3, .qty = 2, .pickFlags = WoweeQuest::AutoGiven});  // 2 healing potions
        c.entries.push_back(e);
    }
    return c;
}

WoweeQuest WoweeQuestLoader::makeChain(const std::string& catalogName) {
    WoweeQuest c;
    c.name = catalogName;
    {
        WoweeQuest::Entry e;
        e.questId = 100;
        e.title = "Investigate the Camp";
        e.objective = "Visit the bandit camp clearing.";
        e.description =
            "Scout reports place a bandit camp east of here. "
            "See for yourself.";
        e.minLevel = 5; e.questLevel = 5;
        e.giverCreatureId = 4001;
        e.turninCreatureId = 4001;
        e.nextQuestId = 101;
        e.objectives.push_back({.kind = WoweeQuest::VisitArea, .targetId = 9001, .quantity = 1});
        e.xpReward = 200;
        e.moneyCopperReward = 100;
        c.entries.push_back(e);
    }
    {
        WoweeQuest::Entry e;
        e.questId = 101;
        e.title = "Recover the Stolen Goods";
        e.objective = "Recover 5 Stolen Letters from Defias Bandits.";
        e.description =
            "The bandits stole letters from the inn. "
            "Recover them.";
        e.minLevel = 6; e.questLevel = 7;
        e.giverCreatureId = 4001;
        e.turninCreatureId = 4001;
        e.prevQuestId = 100;
        e.nextQuestId = 102;
        e.objectives.push_back({.kind = WoweeQuest::CollectItem, .targetId = 4, .quantity = 5});
        e.xpReward = 600;
        e.moneyCopperReward = 350;
        e.rewardItems.push_back({.itemId = 2, .qty = 1, .pickFlags = WoweeQuest::AutoGiven});  // linen vest
        c.entries.push_back(e);
    }
    {
        WoweeQuest::Entry e;
        e.questId = 102;
        e.title = "Report Back";
        e.objective = "Return to the Innkeeper with the recovered letters.";
        e.description =
            "Bring the recovered letters to the Innkeeper.";
        e.minLevel = 6; e.questLevel = 7;
        e.giverCreatureId = 4001;
        e.turninCreatureId = 4001;
        e.prevQuestId = 101;
        // No nextQuestId - chain end.
        // No objectives - quest completes on dialogue alone.
        e.flags = WoweeQuest::AutoComplete;
        e.xpReward = 200;
        e.moneyCopperReward = 500;          // 5s
        // Player choice: 1 of 2 weapons.
        e.rewardItems.push_back({.itemId = 1001, .qty = 1, .pickFlags = WoweeQuest::PlayerChoice}); // sword
        e.rewardItems.push_back({.itemId = 1002, .qty = 1, .pickFlags = WoweeQuest::PlayerChoice}); // blade
        c.entries.push_back(e);
    }
    return c;
}

WoweeQuest WoweeQuestLoader::makeDaily(const std::string& catalogName) {
    WoweeQuest c;
    c.name = catalogName;
    {
        WoweeQuest::Entry e;
        e.questId = 200;
        e.title = "Daily: Wolf Cull";
        e.objective = "Kill 8 wolves outside the village gate.";
        e.description = "The wolves are getting bolder. Thin the pack.";
        e.minLevel = 5; e.questLevel = 5;
        e.giverCreatureId = 4002;        // smith from WCRT village
        e.turninCreatureId = 4002;
        e.objectives.push_back({.kind = WoweeQuest::KillCreature, .targetId = 2001, .quantity = 8});
        e.xpReward = 250;
        e.moneyCopperReward = 1000;       // 10s = good daily payout
        e.flags = WoweeQuest::Daily |
                   WoweeQuest::Repeatable |
                   WoweeQuest::AutoAccept;
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
