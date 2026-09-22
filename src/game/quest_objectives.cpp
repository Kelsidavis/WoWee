#include "game/quest_objectives.hpp"

#include "game/quest_progress.hpp"

#include <cstdlib>

namespace wowee {
namespace game {

namespace {

/// What a kill objective is about, by name: a creature for a positive id, a
/// game object for a negative one. Empty while the query is out; this asks
/// again, because a log built from a saved character can reach an objective
/// whose query was never sent.
std::string targetName(GameHandler& gh, int32_t npcOrGoId) {
    const uint32_t entry = static_cast<uint32_t>(std::abs(npcOrGoId));
    if (npcOrGoId > 0) {
        std::string name = gh.getCachedCreatureName(entry);
        if (name.empty()) gh.queryCreatureInfo(entry, 0);
        return name;
    }
    const auto* info = gh.getCachedGameObjectInfo(entry);
    if (!info || info->name.empty()) {
        gh.queryGameObjectInfo(entry, 0);
        return {};
    }
    return info->name;
}

}  // namespace

std::vector<QuestObjective> questObjectives(GameHandler& gh,
                                            const GameHandler::QuestLogEntry& q) {
    std::vector<QuestObjective> out;
    for (const auto& kill : q.killObjectives) {
        if (!isQuestObjective(kill.npcOrGoId, kill.required)) continue;
        const uint32_t key = static_cast<uint32_t>(std::abs(kill.npcOrGoId));
        uint32_t current = 0;
        if (auto it = q.killCounts.find(key); it != q.killCounts.end()) current = it->second.first;
        const bool isObject = kill.npcOrGoId < 0;
        QuestObjective line;
        line.text = questObjectiveLine(targetName(gh, kill.npcOrGoId), isObject, current,
                                       kill.required);
        line.type = isObject ? "object" : "monster";
        line.finished = current >= kill.required;
        out.push_back(std::move(line));
    }
    for (const auto& item : q.itemObjectives) {
        if (!isQuestObjective(static_cast<int32_t>(item.itemId), item.required)) continue;
        uint32_t current = 0;
        if (auto it = q.itemCounts.find(item.itemId); it != q.itemCounts.end()) current = it->second;
        // By name; the number stands in while the item query is out, and the
        // answer lands with a QUEST_LOG_UPDATE behind it.
        std::string name;
        const auto* info = gh.getItemInfo(item.itemId);
        if (info && !info->name.empty()) {
            name = info->name;
        } else {
            gh.queryItemInfo(item.itemId, 0);
            name = "Item #" + std::to_string(item.itemId);
        }
        QuestObjective line;
        line.text = name + ": " + std::to_string(current) + "/" + std::to_string(item.required);
        line.type = "item";
        line.finished = current >= item.required;
        out.push_back(std::move(line));
    }
    return out;
}

}  // namespace game
}  // namespace wowee
