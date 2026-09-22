#pragma once

// A quest's objective lines, the way the log and the tracker write them.
//
// "Warmaul Reaver slain: 2/5", "Muck-ridden Core: 3/5" - one per kill or
// object objective and one per item, kills first. The interface's quest log
// asks for these one at a time through GetQuestLogLeaderBoard, and the map
// window's own quest list asks for them all at once; one builder serves both,
// so the two cannot word an objective differently.

#include "game/game_handler.hpp"

#include <string>
#include <vector>

namespace wowee {
namespace game {

struct QuestObjective {
    std::string text;
    /// "monster", "object" or "item": what GetQuestLogLeaderBoard answers.
    const char* type = "monster";
    bool finished = false;
};

/// A count of none is not an objective. The server's quest template names the
/// creature an item drops from in the kill record beside it - Dextren Ward
/// carries the Hand of Dextren Ward - and writes a required count of zero on
/// that record, because there is nothing to kill. Counting those gave every
/// such quest one more line than it has, and the extra one answered "0/0",
/// which reads as finished.
inline bool isQuestObjective(int32_t id, uint32_t required) { return id != 0 && required > 0; }

/// How many lines questObjectives answers, without asking for any names.
inline int questObjectiveCount(const GameHandler::QuestLogEntry& q) {
    int count = 0;
    for (const auto& kill : q.killObjectives) {
        if (isQuestObjective(kill.npcOrGoId, kill.required)) ++count;
    }
    for (const auto& item : q.itemObjectives) {
        if (isQuestObjective(static_cast<int32_t>(item.itemId), item.required)) ++count;
    }
    return count;
}

/// Every objective the quest carries, in the log's order. A name still out on
/// its query is asked for again, and stands in with a generic word until it
/// answers - see questObjectiveLine.
std::vector<QuestObjective> questObjectives(GameHandler& gh,
                                            const GameHandler::QuestLogEntry& quest);

}  // namespace game
}  // namespace wowee
