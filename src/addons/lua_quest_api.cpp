// lua_quest_api.cpp - Quest log, skills, talents, glyphs, and achievements Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "core/local_time.hpp"
#include "game/item_text.hpp"
#include "addons/lua_api_helpers.hpp"
#include "addons/lua_engine.hpp"
#include "game/auction_filters.hpp"
#include "game/game_utils.hpp"
#include "game/packed_time.hpp"
#include "ui/chat/chat_utils.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

namespace wowee::addons {

// ── The quest log is a list of headers with quests under them ───────────────
//
// WoW's quest log is grouped: a zone header, the quests in that zone, the next
// header, and so on - and every index-taking quest function counts the headers
// as rows. This log was flat, so it listed 23 quests in server order with no
// grouping at all, ExpandQuestHeader/CollapseQuestHeader were no-ops, and
// GetQuestLogTitle answered false for isHeader on every row.
//
// The grouping key is the quest's zoneOrSort: an AreaTable id when positive
// (the zone the quest belongs to) and a QuestSort id negated when negative
// (Epic, Seasonal, a class or a profession). Zero means the query response has
// not arrived yet, and those wait under their own header rather than being
// dropped.
//
// Everything that takes a quest log index goes through questRows() rather than
// indexing getQuestLog() directly, because with headers in the list the two are
// no longer the same number: quest 3 of the log can be row 5 of the display.
namespace {

struct QuestRow {
    bool isHeader = false;
    int32_t group = 0;                              ///< zoneOrSort of the group
    std::string headerTitle;                        ///< headers only
    const game::GameHandler::QuestLogEntry* quest = nullptr;     ///< quests only
};

/// Headers the player has collapsed, by group key. Interface state, so it lives
/// here rather than in the model - and it is deliberately not persisted, which
/// is what WoW does with a fresh session too.
std::set<int32_t>& collapsedQuestGroups() {
    static std::set<int32_t> collapsed;
    return collapsed;
}

/// What a group's header row is called.
std::string questGroupTitle(game::GameHandler* gh, int32_t group) {
    if (!gh) return "Miscellaneous";
    if (group > 0) {
        std::string name = gh->getAreaName(static_cast<uint32_t>(group));
        if (!name.empty()) return name;
    } else if (group < 0) {
        std::string name = gh->getQuestSortName(static_cast<uint32_t>(-group));
        if (!name.empty()) return name;
    }
    // A quest whose query response has not landed has no zone yet, and one
    // whose zone is not in the file still has to sit somewhere nameable.
    return "Miscellaneous";
}

/// The display list: one row per header, then the rows of its quests unless the
/// header is collapsed. Rebuilt per call - the log holds tens of entries, and a
/// cache here would have to be invalidated by every quest update, every query
/// response and every collapse.
std::vector<QuestRow> questRows(game::GameHandler* gh) {
    std::vector<QuestRow> rows;
    if (!gh) return rows;
    const auto& ql = gh->getQuestLog();

    // Group, keeping each group's quests in the order the log holds them.
    std::map<int32_t, std::vector<const game::GameHandler::QuestLogEntry*>> groups;
    for (const auto& q : ql) {
        if (q.questId == 0) continue;
        groups[q.zoneOrSort].push_back(&q);
    }

    // Zones first and in name order, then the QuestSort groups (Epic, class,
    // profession), then the not-yet-known ones - which is the order the real
    // log reads in, and stable so the list does not reshuffle under the cursor
    // as query responses land.
    struct Group { int32_t key; std::string title; int rank; };
    std::vector<Group> ordered;
    ordered.reserve(groups.size());
    for (const auto& [key, _] : groups) {
        const int rank = (key > 0) ? 0 : (key < 0 ? 1 : 2);
        ordered.push_back({key, questGroupTitle(gh, key), rank});
    }
    std::sort(ordered.begin(), ordered.end(), [](const Group& a, const Group& b) {
        if (a.rank != b.rank) return a.rank < b.rank;
        if (a.title != b.title) return a.title < b.title;
        return a.key < b.key;
    });

    for (const Group& g : ordered) {
        QuestRow header;
        header.isHeader = true;
        header.group = g.key;
        header.headerTitle = g.title;
        rows.push_back(std::move(header));
        if (collapsedQuestGroups().count(g.key)) continue;
        for (const auto* q : groups[g.key]) {
            QuestRow row;
            row.group = g.key;
            row.quest = q;
            rows.push_back(std::move(row));
        }
    }
    return rows;
}

/// Server text as the player should read it.
///
/// Quest text arrives with the reader left blank - $n for their name, $c for
/// class, $r for race, $g male:female; for a phrase that splits on gender - and
/// the client is what fills them in. The quest giver's own panels have resolved
/// them since pushQuestText was written, and the quest log did not: the same
/// quest greeted the player by name while it was being offered and by the two
/// characters "$N" once it was accepted, which is where it is read most.
std::string forPlayer(lua_State* L, const std::string& text) {
    if (text.empty()) return text;
    auto* gh = getGameHandler(L);
    return gh ? ui::chat_utils::replaceGenderPlaceholders(text, *gh) : text;
}

}  // namespace

// Declared in lua_api_helpers.hpp; defined here because questRows above is the
// one list every quest log index is counted against.
int questLogRowForQuest(game::GameHandler* gh, uint32_t questId) {
    if (!gh || questId == 0) return 0;
    const auto rows = questRows(gh);
    for (size_t i = 0; i < rows.size(); ++i) {
        if (!rows[i].isHeader && rows[i].quest && rows[i].quest->questId == questId) {
            return static_cast<int>(i) + 1;  // the interface counts from one
        }
    }
    return 0;
}

void openInterfaceQuestLog(game::GameHandler& gh, uint32_t questId) {
    const int row = questLogRowForQuest(&gh, questId);
    if (row > 0) {
        gh.runInterfaceCommand("QuestLog_OpenToQuest(" + std::to_string(row) + ")");
    } else {
        gh.runInterfaceCommand("ToggleFrame(QuestLogFrame)");
    }
}

namespace {

/// The quest at a display index, or null when that row is a header or the index
/// is off either end. This is what every "…(questLogIndex)" binding needs: a
/// header row genuinely has no quest, and answering with the wrong one is how a
/// log shows the details of a quest the player did not click.
const game::GameHandler::QuestLogEntry* questAtRow(game::GameHandler* gh, int index) {
    if (index < 1) return nullptr;
    const auto rows = questRows(gh);
    if (index > static_cast<int>(rows.size())) return nullptr;
    return rows[index - 1].quest;
}

} // namespace

// The same mapping, reachable from the other binding files. Declared in
// lua_api_helpers.hpp because anything holding a quest log index needs it.
const game::GameHandler::QuestLogEntry* questAtLogRow(game::GameHandler* gh, int index) {
    return questAtRow(gh, index);
}

static int lua_GetNumQuestLogEntries(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
    // numEntries counts the rows the log displays - headers included, and the
    // quests under a collapsed header excluded, because that is what the frame
    // walks with GetQuestLogTitle.
    lua_pushnumber(L, questRows(gh).size());
    // numQuests is every quest held, whatever is collapsed: it is what
    // QuestLog_UpdateQuestCount prints against MAX_QUESTLOG_QUESTS, and
    // QuestLogFrame only picks a first selection when it is above zero.
    size_t quests = 0;
    for (const auto& q : gh->getQuestLog()) {
        if (q.questId != 0) ++quests;
    }
    lua_pushnumber(L, quests);
    return 2;
}

// GetQuestLogTitle(index) → title, level, suggestedGroup, isHeader, isCollapsed, isComplete, frequency, questID
// ---- The quest info panel's reward block ----
//
// Shared by the quest giver and the quest log, so a raise here takes both down.
// None of these appeared in a scan of either frame's own file, because the
// panel that draws the rewards is a third file they both pull in.

// GetQuestLogTimeLeft() → seconds left on the selected quest, or nil.
// Backed by the same PLAYER_QUEST_LOG expiry the tracker's timers read.
static int lua_GetQuestLogTimeLeft(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return luaReturnNil(L);
    const int sel = gh->getSelectedQuestLogIndex();
    const auto& ql = gh->getQuestLog();
    if (sel < 1 || sel > static_cast<int>(ql.size())) return luaReturnNil(L);
    const uint32_t questId = ql[static_cast<size_t>(sel) - 1].questId;
    for (const auto& t : gh->getQuestTimers()) {
        if (t.first == questId) { lua_pushnumber(L, t.second); return 1; }
    }
    return luaReturnNil(L);   // not a timed quest
}

// Defined further down, with the other quest-log readers.
static const game::GameHandler::QuestLogEntry* selectedLogEntry(game::GameHandler* gh);

// Whether the quest being looked at has been failed.
//
// The quest slot's state field carries it beside completion - the server names
// the two bits QUEST_STATE_COMPLETE and QUEST_STATE_FAIL - and the field was
// already being read for the first of them.
static int lua_IsCurrentQuestFailed(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* q = selectedLogEntry(gh);
    lua_pushboolean(L, (q && q->failed) ? 1 : 0);
    return 1;
}

// The spell, title and faction rewards a quest can carry. The query response
// this client parses parses none of them, and each is asked behind `if ( ... )`
// before its block is drawn - so nil leaves the block out rather than drawing
// an empty one.
// GetRewardSpell() - the quest *giver's* reward spell.
//
// The offer's own, not the log's: the panel asking this is showing a quest
// being offered, which is usually not the one selected in the log, so the log
// form cannot stand in for it.
//
// It answered nil because the details packet was not read this far. It is now
// - four fields past the XP - so a quest that teaches a recipe shows what it
// teaches. Same four values and same three flags as the log form beside it.
static int lua_GetQuestRewardSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return luaReturnNil(L);
    const uint32_t spellId = gh->getQuestDetails().rewardSpellId;
    if (spellId == 0) return luaReturnNil(L);
    const std::string name = gh->getSpellName(spellId);
    if (name.empty()) return luaReturnNil(L);
    const std::string icon = gh->getSpellIconPath(spellId);
    lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                   : icon.c_str());
    lua_pushstring(L, name.c_str());
    lua_pushboolean(L, gh->isProfessionSpell(spellId) ? 1 : 0);
    lua_pushboolean(L, gh->getKnownSpells().count(spellId) > 0 ? 1 : 0);
    return 4;
}
// GetQuestLogRewardTitle() → the title a quest awards, formatted with the
// player's name, or nil for the many quests that award none. The CharTitleId is
// in the query response (field 19) and the name in CharTitles.dbc, both of
// which this now reads; the old answer was a flat nil.
static int lua_GetQuestRewardTitle(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    // The offer panel (GetRewardTitle) and the log panel (GetQuestLogRewardTitle)
    // share this; read whichever is open, the offer's own title id when it is,
    // else the selected log quest's - the log selection is not the offer's.
    uint32_t titleId = 0;
    if (gh->isQuestOfferRewardOpen()) {
        titleId = gh->getQuestOfferReward().rewardTitleId;
    } else {
        const int index = gh->getSelectedQuestLogIndex();
        const auto& log = gh->getQuestLog();
        if (index >= 1 && index <= static_cast<int>(log.size()))
            titleId = log[static_cast<size_t>(index - 1)].rewardTitleId;
    }
    if (titleId == 0) { return luaReturnNil(L); }
    const std::string name = gh->getFormattedTitleById(titleId);
    if (name.empty()) { return luaReturnNil(L); }
    lua_pushstring(L, name.c_str());
    return 1;
}
static int lua_ProcessQuestLogRewardFactions(lua_State* L) { (void)L; return 0; }

// The selected quest's five reputation reward slots, packed to the front (the
// panel walks 1..GetNumQuestLogRewardFactions expecting no gaps). Inlined
// selection because selectedQuest is defined further down.
/// The quest log entry the player has selected, or null when nothing is.
///
/// The log index is 1-based, as the interface counts it. This was defined
/// twice in this file under two names, identically - one for the reward
/// panels and one for everything else - which is one function.
static const game::QuestHandler::QuestLogEntry* selectedQuest(game::GameHandler* gh) {
    if (!gh) return nullptr;
    const int index = gh->getSelectedQuestLogIndex();
    const auto& log = gh->getQuestLog();
    if (index < 1 || index > static_cast<int>(log.size())) return nullptr;
    return &log[static_cast<size_t>(index - 1)];
}
// The reputation reward slots for whichever panel is open - the offer's own
// when it is up, the selected log quest's otherwise (same reason as the title:
// the log selection is not the offered quest). Copied into a common shape so
// the two source structs, identical in fields, are read the same way.
struct RewardFaction { uint32_t factionId; int32_t valueId; int32_t override; };
static std::vector<RewardFaction> currentFactionRewards(game::GameHandler* gh) {
    std::vector<RewardFaction> out;
    if (!gh) return out;
    if (gh->isQuestOfferRewardOpen()) {
        for (const auto& fr : gh->getQuestOfferReward().factionRewards)
            if (fr.factionId != 0) out.push_back({fr.factionId, fr.valueId, fr.override});
    } else if (const auto* q = selectedQuest(gh)) {
        for (const auto& fr : q->factionRewards)
            if (fr.factionId != 0) out.push_back({fr.factionId, fr.valueId, fr.override});
    }
    return out;
}
// GetNumQuestLogRewardFactions() → how many faction rewards the quest carries.
static int lua_GetNumQuestLogRewardFactions(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(currentFactionRewards(getGameHandler(L)).size()));
    return 1;
}
// GetQuestLogRewardFactionInfo(i) → factionId, amount(in hundredths). The panel
// pairs the id with GetFactionInfoByID and divides the amount by 100 itself.
static int lua_GetQuestLogRewardFactionInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int want = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto list = currentFactionRewards(gh);
    if (want < 1 || want > static_cast<int>(list.size())) { return luaReturnNil(L); }
    const auto& fr = list[static_cast<size_t>(want - 1)];
    auto* qh = gh->getQuestHandler();
    const int32_t amount = qh ? qh->getQuestRewardReputation(fr.valueId, fr.override) : 0;
    lua_pushnumber(L, fr.factionId);
    lua_pushnumber(L, amount);
    return 2;
}

// GetFactionInfoByID(id) - the same thirteen values GetFactionInfo gives by
// position, found by faction id instead. The reputation list carries the id
// already.
//
// This used to answer a name and five nils. The quest reward panel reads the
// ninth and eleventh - isHeader and hasRep - to decide whether to print a
// reputation line, and got nil for both; it survived only because `not nil` is
// true and the test happens to fall the right way.
static int lua_GetFactionInfoByID(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
    if (!gh || id == 0) return luaReturnNil(L);
    for (const auto& r : gh->getReputationList()) {
        if (r.factionId == id) return pushFactionInfo(L, gh, r);
    }
    return luaReturnNil(L);
}

// ---- Quest watch ordering ----
//
// The watch list here is a set of quest ids, so its order is the quest log's
// order rather than one of its own. That is what these three say.

// GetQuestWatchIndex(questLogIndex) → where that quest sits in the watch list.
static int lua_GetQuestWatchIndex(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || idx < 1) return luaReturnNil(L);
    const auto& ql = gh->getQuestLog();
    if (idx > static_cast<int>(ql.size())) return luaReturnNil(L);
    const uint32_t wanted = ql[static_cast<size_t>(idx) - 1].questId;
    int position = 0;
    for (const auto& q : ql) {
        if (!gh->isQuestTracked(q.questId)) continue;
        ++position;
        if (q.questId == wanted) { lua_pushnumber(L, position); return 1; }
    }
    return luaReturnNil(L);   // not watched
}

// SortQuestWatches() → whether the order changed.
//
// False, and that is the truthful answer rather than a shrug: the watch order
// follows the quest log, so there is never a separate order to sort. The
// caller reads it as "did anything move" and rebuilds the tracker when it did.
static int lua_SortQuestWatches(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// ShiftQuestWatches(from, to) - reorder the list by hand. Nothing is stored to
// reorder, so dragging a tracker entry leaves it where the log puts it.
static int lua_ShiftQuestWatches(lua_State* L) { (void)L; return 0; }

// GetQuestSortIndex(questLogIndex) → the header the quest sits under.
//
// Nil: this quest log has no headers - GetQuestLogTitle answers false for
// isHeader on every row - so there is no header index to give and nothing for
// the caller to expand.
static int lua_GetQuestSortIndex(lua_State* L) { (void)L; return luaReturnNil(L); }

// ---- Quest log special items ----
//
// Whether the quest item's target is close enough to use it on. Nothing here
// knows an item's range, and nil is the answer that hides the range indicator
// rather than colouring it wrongly - WatchFrameItem_OnUpdate takes the third
// branch and hides the count text.
static int lua_IsQuestLogSpecialItemInRange(lua_State* L) { (void)L; return luaReturnNil(L); }

// UseQuestLogSpecialItem(questLogIndex) - clicking that button.
//
// By slot rather than by item id, for the same reason UseContainerItem is:
// searching by id can find a different stack of the same thing.
static int lua_UseQuestLogSpecialItem(lua_State* L);

// GetQuestLogSpecialItemCooldown(index) → start, duration, enable.
//
// Enable is one, not zero: zero means the cooldown swipe is switched off, and
// the caller feeds all three straight to CooldownFrame_SetTimer.
//
// The item's cooldown is its on-use spell's, the same relationship the bag
// buttons read. This answered a flat zero, so a quest item just used was drawn
// ready again - and WatchFrameItem_UpdateCooldown runs on every
// BAG_UPDATE_COOLDOWN, so it had something to ask and nothing to hear.
static int lua_GetQuestLogSpecialItemCooldown(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto item = questSpecialItemAt(gh, index);
    double start = 0.0, duration = 0.0;
    if (item.itemId && itemUseCooldown(gh, item.itemId, start, duration)) {
        lua_pushnumber(L, start);
        lua_pushnumber(L, duration);
        lua_pushnumber(L, 1);
        return 3;
    }
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 1);
    return 3;
}

// GetQuestTimers() - the seconds left on each timed quest, as separate values.
//
// QuestTimerFrame counts them with select("#", ...) and reads them with
// select(i, ...), so the count is the return count. Returning nothing is the
// honest answer for a log with no timed quest in it, and the frame hides
// itself - which it could not do while this was missing, because the OnEvent
// that calls it runs on every QUEST_LOG_UPDATE.
static int lua_GetQuestTimers(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const auto timers = gh->getQuestTimers();
    // One per timed quest, against Lua's twenty guaranteed slots. A log holds
    // twenty-five.
    if (!timers.empty() && !lua_checkstack(L, static_cast<int>(timers.size()))) {
        return 0;
    }
    for (const auto& t : timers) lua_pushnumber(L, t.second);
    return static_cast<int>(timers.size());
}

// GetQuestIndexForTimer(i) - the quest log index the i-th timer belongs to,
// so clicking a timer row selects its quest.
static int lua_GetQuestIndexForTimer(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || idx < 1) return luaReturnNil(L);
    const auto timers = gh->getQuestTimers();
    if (idx > static_cast<int>(timers.size())) return luaReturnNil(L);
    const uint32_t questId = timers[static_cast<size_t>(idx) - 1].first;
    // A display index, since that is what every other quest binding takes.
    const auto rows = questRows(gh);
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].quest && rows[i].quest->questId == questId) {
            lua_pushnumber(L, static_cast<double>(i + 1));
            return 1;
        }
    }
    return luaReturnNil(L);
}

/// Which page of an open book is being read, zero-based.
///
/// The reader's own state, not the client's: every page is already in hand by
/// the time the frame draws one, so turning a page moves this and nothing else.
/// Reset when a book is closed, which is the only moment it can be stale.
static int& bookPage() { static int page = 0; return page; }

/// Tell the frame to redraw from whatever the page bindings now answer.
static void fireItemTextReady(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->fireAddonEvent("ITEM_TEXT_READY", {});
}

static int lua_GetQuestLogTitle(lua_State* L) {
    auto* gh = getGameHandler(L);
    // optnumber, not checknumber: FrameXML walks the quest log with an index
    // that can be nil before anything has been selected, and raising there
    // loses whatever asked rather than answering that there is no such quest.
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh) { return luaReturnNil(L); }
    const auto rows = questRows(gh);
    // A header row: the zone (or Epic/class/profession) the quests under it
    // belong to. isHeader is what tells QuestLog_Update to draw it as one and to
    // hang a collapse box off it, and isCollapsed is what that box reads.
    if (index >= 1 && index <= static_cast<int>(rows.size()) && rows[index - 1].isHeader) {
        const auto& row = rows[index - 1];
        lua_pushstring(L, row.headerTitle.c_str());                        // 1: title
        lua_pushnumber(L, 0);                                              // 2: level
        lua_pushnil(L);                                                    // 3: questTag
        lua_pushnumber(L, 0);                                              // 4: suggestedGroup
        lua_pushboolean(L, 1);                                             // 5: isHeader
        lua_pushboolean(L, collapsedQuestGroups().count(row.group) ? 1 : 0); // 6: isCollapsed
        lua_pushnil(L);                                                    // 7: isComplete
        lua_pushboolean(L, 0);                                             // 8: isDaily
        lua_pushnumber(L, 0);                                              // 9: questID
        lua_pushboolean(L, 0);                                             // 10: displayQuestID
        return 10;
    }
    // An index off either end gets an empty row, not a bare nil.
    //
    // QuestLog_Update reads the count once and then walks GetQuestLogTitle in a
    // loop; a server update landing between frames while the log is scrolled
    // can shrink the log under a count the frame still holds, so an index it
    // believes valid comes back past the end. In colourblind mode the very next
    // line is `title = "["..level.."] "..title`, and a nil title raised there
    // and tore down the whole redraw - which is what left the list a mess. An
    // empty title concatenates cleanly and the row corrects itself on the next
    // pass; a raise does not.
    //
    // The low end too, not just the high: the button loop guards only the upper
    // bound (`if questIndex <= numEntries`), so a scroll offset that drives the
    // index to zero or below still reaches here and hit `"  "..title` at a nil
    // - the questlogframe.lua:430 raise seen driving the log headlessly. The
    // `if title` callers all iterate 1..numEntries and never reach a low index,
    // so an empty row here changes nothing for them.
    const game::GameHandler::QuestLogEntry* qp =
        (index >= 1 && index <= static_cast<int>(rows.size())) ? rows[index - 1].quest : nullptr;
    if (!qp) {
        lua_pushstring(L, "");   // 1: title (empty, never nil)
        lua_pushnumber(L, 0);    // 2: level
        lua_pushnil(L);          // 3: questTag
        lua_pushnumber(L, 0);    // 4: suggestedGroup
        lua_pushboolean(L, 0);   // 5: isHeader
        lua_pushboolean(L, 0);   // 6: isCollapsed
        lua_pushnil(L);          // 7: isComplete
        lua_pushboolean(L, 0);   // 8: isDaily
        lua_pushnumber(L, 0);    // 9: questID
        lua_pushboolean(L, 0);   // 10: displayQuestID
        return 10;
    }
    const auto& q = *qp;
    // The client's ten, in its order:
    //
    //   title, level, questTag, suggestedGroup, isHeader, isCollapsed,
    //   isComplete, isDaily, questID, displayQuestID
    //
    // Eight were returned, with questTag and isDaily absent, so everything
    // from the third value on landed one or two places early. isComplete
    // received a zero and so no quest ever showed as complete; isDaily
    // received the quest id, which is a large number and therefore true, so
    // every quest in the log was marked daily; and questID arrived nil.
    lua_pushstring(L, forPlayer(L, q.title).c_str());  // 1: title
    // The level is tracked - the query response carries it - and was being
    // answered as a flat zero beside a comment saying it was not.
    lua_pushnumber(L, q.level);          // 2: level
    lua_pushnil(L);                      // 3: questTag ("Elite", "PvP", …)
    lua_pushnumber(L, 0);                // 4: suggestedGroup
    lua_pushboolean(L, 0);               // 5: isHeader
    lua_pushboolean(L, 0);               // 6: isCollapsed
    // A number, not a boolean: 1 for complete, -1 for failed, nil otherwise.
    // watchframe.lua writes `if ( isComplete and isComplete < 0 )` to tell a
    // failed quest from a finished one, and comparing a boolean with a number
    // raises. Correcting the *position* of this value without correcting its
    // type turned a quiet wrong answer into an error on the quest tracker.
    //
    // Failed is tested first: the server can set both bits, and a quest whose
    // timer ran out is failed whatever else is true of it.
    if (q.failed)        lua_pushnumber(L, -1);
    else if (q.complete) lua_pushnumber(L, 1);
    else                 lua_pushnil(L);                        // 7: isComplete
    lua_pushboolean(L, 0);               // 8: isDaily
    lua_pushnumber(L, q.questId);        // 9: questID
    // Not the id again: this is the developer switch that asks for the id to
    // be *shown*, and the quest log writes "26102 - Wolves Across the Border"
    // when it is set. An id is a large number and therefore true, so every
    // quest in the log wore its own number. Off is what the real client
    // ships.
    lua_pushboolean(L, 0);               // 10: displayQuestID
    return 10;
}

// GetQuestLogQuestText(index) → description, objectives
/// The quest log index an argument-less call means: the one the log has
/// selected. WoW's quest log functions take the index only when asking about
/// some other entry, and demanding it raised a Lua error on every bare call.
static int questLogIndexOrSelected(lua_State* L, int arg) {
    if (!lua_isnoneornil(L, arg)) return static_cast<int>(luaL_checknumber(L, arg));
    auto* gh = getGameHandler(L);
    return gh ? gh->getSelectedQuestLogIndex() : 0;
}

static int lua_GetQuestLogQuestText(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = questLogIndexOrSelected(L, 1);
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto* qp = questAtRow(gh, index);
    if (!qp) { return luaReturnNil(L); }
    const auto& q = *qp;
    // The quest giver's own text. It is the third string in the query
    // response and the parser already walked over it to reach the fifth, so
    // "not stored" was true only of the store - the bytes were in hand and
    // discarded, and the quest log drew a blank panel above every objective
    // list because of it.
    lua_pushstring(L, forPlayer(L, q.description).c_str());  // description
    lua_pushstring(L, forPlayer(L, q.objectives).c_str());   // objectives
    return 2;
}

// IsQuestComplete(questID) → boolean
static int lua_IsQuestComplete(lua_State* L) {
    auto* gh = getGameHandler(L);
    uint32_t questId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    if (!gh) { return luaReturnFalse(L); }
    for (const auto& q : gh->getQuestLog()) {
        if (q.questId == questId) {
            lua_pushboolean(L, q.complete);
            return 1;
        }
    }
    lua_pushboolean(L, 0);
    return 1;
}

// SelectQuestLogEntry(index) - select a quest in the quest log
static int lua_SelectQuestLogEntry(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh) return 0;
    gh->setSelectedQuestLogIndex(index);
    // Fetch the full text if this quest arrived without it.
    //
    // A quest accepted this session carries its description from the offer
    // packet, but a quest already in the log at login comes with only its id,
    // title and objectives - the description is never sent with the log and
    // has to be asked for. Selecting the entry is when the panel wants it, and
    // nothing had ever requested it, so every quest carried from a previous
    // session showed an empty description above its objectives. The query
    // answers with QUEST_LOG_UPDATE, which the panel already redraws on.
    if (const auto* q = questAtRow(gh, index)) {
        if (q->description.empty() && q->questId != 0) {
            gh->requestQuestQuery(q->questId, false);
        }
    }
    return 0;
}

// GetQuestLogSelection() → index
static int lua_GetQuestLogSelection(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getSelectedQuestLogIndex() : 0);
    return 1;
}

/// The quest the log has selected, or null if none is.
// GetQuestLogPushable() → whether the selected quest may be offered to the party.
//
// Yes for any real selection. Which quests the server will actually share is a
// flag on the quest, and no packet this client parses carries it - so the
// choice is between offering the attempt and letting the server refuse, or
// never offering it at all. The button is disabled without this, and sharing
// works, so silence would be the more misleading answer of the two.
static int lua_GetQuestLogPushable(lua_State* L) {
    lua_pushboolean(L, selectedQuest(getGameHandler(L)) != nullptr ? 1 : 0);
    return 1;
}

// GetQuestLogRewardXP() → the experience the reward panel shows for the
// selected quest. It is in no packet - the client derives it from QuestXP.dbc
// at the quest's level and the XP-difficulty index the query carried, which is
// what this now does. Zero (the old hard-coded answer) hid the reward line on
// every quest; a real value restores "You will receive N experience."
static int lua_GetQuestLogRewardXP(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* q = gh ? selectedQuest(gh) : nullptr;
    auto* qh = gh ? gh->getQuestHandler() : nullptr;
    lua_pushnumber(L, (q && qh) ? qh->getQuestRewardXP(q->level, q->rewardXPId) : 0.0);
    return 1;
}

// Honor, talent points and arena points a quest awards - direct values the
// query response carries, and hard zeros before, on a stale "not in any quest
// packet" comment. WotLK-only, so classic/TBC keep zero until their layout is
// read off a serializer.
static int lua_GetQuestLogRewardHonor(lua_State* L) {
    const auto* q = selectedQuest(getGameHandler(L));
    lua_pushnumber(L, q ? q->rewardHonor : 0.0);
    return 1;
}
static int lua_GetQuestLogRewardTalents(lua_State* L) {
    const auto* q = selectedQuest(getGameHandler(L));
    lua_pushnumber(L, q ? q->rewardTalents : 0.0);
    return 1;
}
static int lua_GetQuestLogRewardArenaPoints(lua_State* L) {
    const auto* q = selectedQuest(getGameHandler(L));
    lua_pushnumber(L, q ? q->rewardArenaPoints : 0.0);
    return 1;
}

// QuestLogPushQuest() - offer the selected quest to the party.
static int lua_QuestLogPushQuest(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (const auto* quest = selectedQuest(gh)) {
        gh->shareQuestWithParty(quest->questId);
    }
    return 0;
}

// GetNumQuestWatches() → count
static int lua_GetNumQuestWatches(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getTrackedQuestIds().size() : 0);
    return 1;
}

// GetQuestIndexForWatch(watchIndex) → questLogIndex
// Maps the Nth watched quest to its quest log index (1-based)
static int lua_GetQuestIndexForWatch(lua_State* L) {
    auto* gh = getGameHandler(L);
    int watchIdx = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || watchIdx < 1) { return luaReturnNil(L); }
    // The index this answers is a quest log index, and a quest log index now
    // counts header rows - so it has to be found in the display list rather
    // than by counting through getQuestLog(). Answering the raw log position
    // pointed the watch frame at whatever row happened to hold that number,
    // which after grouping is usually a different quest or a header.
    const auto rows = questRows(gh);
    const auto& tracked = gh->getTrackedQuestIds();
    int found = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
        if (!rows[i].quest) continue;
        if (tracked.count(rows[i].quest->questId)) {
            found++;
            if (found == watchIdx) {
                lua_pushnumber(L, static_cast<int>(i) + 1); // 1-based
                return 1;
            }
        }
    }
    lua_pushnil(L);
    return 1;
}

// AddQuestWatch(questLogIndex) - add a quest to the watch list
static int lua_AddQuestWatch(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) return 0;
    if (const auto* q = questAtRow(gh, index)) {
        gh->setQuestTracked(q->questId, true);
    }
    return 0;
}

// RemoveQuestWatch(questLogIndex) - remove a quest from the watch list
static int lua_RemoveQuestWatch(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) return 0;
    if (const auto* q = questAtRow(gh, index)) {
        gh->setQuestTracked(q->questId, false);
    }
    return 0;
}

// IsQuestWatched(questLogIndex) → boolean
static int lua_IsQuestWatched(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnFalse(L); }
    const auto* q = questAtRow(gh, index);
    lua_pushboolean(L, (q && gh->isQuestTracked(q->questId)) ? 1 : 0);
    return 1;
}

// GetQuestLink(questLogIndex) → "|cff...|Hquest:id:level|h[title]|h|r"
static int lua_GetQuestLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto* qp = questAtRow(gh, index);
    if (!qp) { return luaReturnNil(L); }
    const auto& q = *qp;
    // Yellow quest link format matching WoW
    std::string link = "|cff808000|Hquest:" + std::to_string(q.questId) +
                       ":0|h[" + q.title + "]|h|r";
    lua_pushstring(L, link.c_str());
    return 1;
}

// GetNumQuestLeaderBoards(questLogIndex) → count of objectives
static int lua_GetNumQuestLeaderBoards(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = questLogIndexOrSelected(L, 1);
    if (!gh || index < 1) { return luaReturnZero(L); }
    const auto* qp = questAtRow(gh, index);
    if (!qp) { return luaReturnZero(L); }
    const auto& q = *qp;
    int count = 0;
    for (const auto& ko : q.killObjectives) {
        if (ko.npcOrGoId != 0 || ko.required > 0) ++count;
    }
    for (const auto& io : q.itemObjectives) {
        if (io.itemId != 0 || io.required > 0) ++count;
    }
    lua_pushnumber(L, count);
    return 1;
}

// GetQuestLogLeaderBoard(objIndex, questLogIndex) → text, type, finished
// objIndex is 1-based within the quest's objectives

static int lua_GetQuestLogLeaderBoard(lua_State* L);

// ── Quest points of interest ───────────────────────────────────────────────
//
// The map's quest markers. The server sends these as SMSG_QUEST_POI and this
// client already keeps them - it draws its own markers from the same list - so
// FrameXML's world map can read the real thing rather than an empty one.
//
// WoW numbers them by "visible index", which is a position in the list of
// quests that have a POI on the map now, not a quest log index. The two differ
// as soon as one quest in the log has no marker.

/// The quest ids that have a marker, in the order the server sent them, each
/// appearing once. Built on demand: the list is short and changes whenever the
/// server sends a new one.
static std::vector<uint32_t> questsWithPois(game::GameHandler* gh) {
    std::vector<uint32_t> out;
    if (!gh) return out;
    for (const auto& poi : gh->getGossipPois()) {
        // -2 is an ordinary gossip marker rather than a quest one.
        if (poi.questObjectiveIndex == -2 || poi.data == 0) continue;
        if (std::find(out.begin(), out.end(), poi.data) == out.end()) out.push_back(poi.data);
    }
    return out;
}

/// QuestMapUpdateAllQuests() → how many quests have a marker on the map.
///
/// Both a verb and a question in the real client: it refreshes the POI set and
/// answers how many there are. There is nothing to refresh here - the list is
/// whatever the server last sent - so this is the answer alone.
///
/// It was not bound at all, and WatchFrame_GetCurrentMapQuests reads it
/// straight into `for i = 1, numQuests`. A nil limit there is not an empty
/// loop but an error, so the tracker's map-quest table was never built and the
/// handler around it died on the way. The count it needs was already sitting
/// in the same list QuestPOIGetQuestIDByVisibleIndex indexes.
static int lua_QuestMapUpdateAllQuests(lua_State* L) {
    lua_pushnumber(L, static_cast<lua_Number>(questsWithPois(getGameHandler(L)).size()));
    return 1;
}

/// QuestPOIGetQuestIDByVisibleIndex(i) → questId, questLogIndex.
///
/// Both, because the world map uses the second to reach everything else about
/// the quest: `if ( questLogIndex and questLogIndex > 0 )` gates the whole
/// block that builds the map's quest list, and with only one value returned
/// that gate never opened - so the list was always empty, quietly, with no
/// error to say why.
static int lua_QuestPOIGetQuestIDByVisibleIndex(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    const auto ids = questsWithPois(gh);
    if (index < 1 || index > static_cast<int>(ids.size())) {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        return 2;
    }
    const uint32_t questId = ids[static_cast<size_t>(index - 1)];
    lua_pushnumber(L, questId);
    // Where that quest sits in the log, counted as Lua counts. A marker can
    // outlive the log entry - the server sends POIs separately - so a quest
    // that is no longer held answers zero rather than a stale position.
    int logIndex = 0;
    if (gh) {
        const auto rows = questRows(gh);
        for (size_t i = 0; i < rows.size(); ++i) {
            if (rows[i].quest && rows[i].quest->questId == questId) {
                logIndex = static_cast<int>(i) + 1;
                break;
            }
        }
    }
    lua_pushnumber(L, logIndex);
    return 2;
}

/// QuestPOIGetIconInfo(questId) → completed, x, y.
///
/// The endpoint marker is the one the map draws for a quest, so that is the
/// one reported: objective index -1 identifies it. Completion comes from the
/// quest log rather than the marker, which does not carry it.
static int lua_QuestPOIGetIconInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t questId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    if (!gh || questId == 0) { return luaReturnNil(L); }

    const game::GossipPoi* best = nullptr;
    for (const auto& poi : gh->getGossipPois()) {
        if (poi.data != questId || poi.questObjectiveIndex == -2) continue;
        if (!best || poi.questObjectiveIndex == -1) best = &poi;
        if (poi.questObjectiveIndex == -1) break;
    }
    if (!best) { return luaReturnNil(L); }

    bool complete = false;
    for (const auto& q : gh->getQuestLog()) {
        if (q.questId == questId) { complete = q.complete; break; }
    }
    lua_pushboolean(L, complete);
    lua_pushnumber(L, best->x);
    lua_pushnumber(L, best->y);
    // The fourth value the API names: which objective this marker belongs to.
    // The client has carried it all along as questObjectiveIndex, where -1
    // means the quest itself rather than one of its lines - which is nil here,
    // because that is what "not an objective" is in Lua and a zero would be a
    // real objective number.
    if (best->questObjectiveIndex < 0) lua_pushnil(L);
    else lua_pushnumber(L, best->questObjectiveIndex + 1);
    return 4;
}

/// The markers arrive with the server's own updates, so there is nothing to
/// refresh on demand - but the map asks before drawing and expects the call to
/// exist.
static int lua_QuestPOIUpdateIcons(lua_State* L) { (void)L; return 0; }

/// GetQuestPOILeaderBoard(objectiveIndex, questId) → the objective's text and
/// counts, the same as the quest log's version - except that this one is given
/// a quest id where that one takes a log index. Aliasing the two would look
/// right and read the wrong quest, so the id is turned into an index here.
static int lua_GetQuestPOILeaderBoard(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t questId = static_cast<uint32_t>(luaL_optnumber(L, 2, 0));
    if (!gh || questId == 0) { return luaReturnNil(L); }
    const auto rows = questRows(gh);
    int index = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].quest && rows[i].quest->questId == questId) {
            index = static_cast<int>(i) + 1;
            break;
        }
    }
    if (index == 0) { return luaReturnNil(L); }
    lua_pushvalue(L, 1);            // objective index, unchanged
    lua_pushnumber(L, index);       // the log index the other one wants
    lua_replace(L, 2);
    lua_replace(L, 1);
    return lua_GetQuestLogLeaderBoard(L);
}

static int lua_GetQuestLogLeaderBoard(lua_State* L) {
    auto* gh = getGameHandler(L);
    int objIdx = static_cast<int>(luaL_checknumber(L, 1));
    int questIdx = static_cast<int>(luaL_optnumber(L, 2,
        gh ? gh->getSelectedQuestLogIndex() : 0));
    if (!gh || questIdx < 1 || objIdx < 1) { return luaReturnNil(L); }
    const auto* qp = questAtRow(gh, questIdx);
    if (!qp) { return luaReturnNil(L); }
    const auto& q = *qp;

    // Build ordered list: kill objectives first, then item objectives
    int cur = 0;
    for (int i = 0; i < 4; ++i) {
        if (q.killObjectives[i].npcOrGoId == 0 && q.killObjectives[i].required == 0) continue;
        ++cur;
        if (cur == objIdx) {
            // Get current count from killCounts map (keyed by abs(npcOrGoId))
            uint32_t key = static_cast<uint32_t>(std::abs(q.killObjectives[i].npcOrGoId));
            uint32_t current = 0;
            auto it = q.killCounts.find(key);
            if (it != q.killCounts.end()) current = it->second.first;
            uint32_t required = q.killObjectives[i].required;
            bool finished = (current >= required);
            // Build display text like "Kobold Vermin slain: 3/8"
            std::string text = (q.killObjectives[i].npcOrGoId < 0 ? "Object" : "Creature")
                + std::string(" slain: ") + std::to_string(current) + "/" + std::to_string(required);
            lua_pushstring(L, text.c_str());
            lua_pushstring(L, q.killObjectives[i].npcOrGoId < 0 ? "object" : "monster");
            lua_pushboolean(L, finished ? 1 : 0);
            return 3;
        }
    }
    for (int i = 0; i < 6; ++i) {
        if (q.itemObjectives[i].itemId == 0 && q.itemObjectives[i].required == 0) continue;
        ++cur;
        if (cur == objIdx) {
            uint32_t current = 0;
            auto it = q.itemCounts.find(q.itemObjectives[i].itemId);
            if (it != q.itemCounts.end()) current = it->second;
            uint32_t required = q.itemObjectives[i].required;
            bool finished = (current >= required);
            // Get item name if available
            std::string itemName;
            const auto* info = gh->getItemInfo(q.itemObjectives[i].itemId);
            if (info && !info->name.empty()) itemName = info->name;
            else itemName = "Item #" + std::to_string(q.itemObjectives[i].itemId);
            std::string text = itemName + ": " + std::to_string(current) + "/" + std::to_string(required);
            lua_pushstring(L, text.c_str());
            lua_pushstring(L, "item");
            lua_pushboolean(L, finished ? 1 : 0);
            return 3;
        }
    }
    lua_pushnil(L);
    return 1;
}

// ExpandQuestHeader(index) / CollapseQuestHeader(index) - fold a zone away.
//
// The index is a display index naming a header row, and zero means every header
// at once, which is what the log's own "collapse all" does. Anything that is
// not a header is ignored rather than treated as row zero.
static int setQuestHeaderCollapsed(lua_State* L, bool collapse) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto rows = questRows(gh);
    if (index == 0) {
        collapsedQuestGroups().clear();
        if (collapse) {
            for (const auto& row : rows) {
                if (row.isHeader) collapsedQuestGroups().insert(row.group);
            }
        }
    } else if (index >= 1 && index <= static_cast<int>(rows.size()) &&
               rows[index - 1].isHeader) {
        if (collapse) collapsedQuestGroups().insert(rows[index - 1].group);
        else          collapsedQuestGroups().erase(rows[index - 1].group);
    }
    // The list the frame is walking just changed length underneath it, so it
    // has to be told to walk it again - nothing else fires for this.
    gh->fireAddonEvent("QUEST_LOG_UPDATE", {});
    return 0;
}
static int lua_ExpandQuestHeader(lua_State* L) { return setQuestHeaderCollapsed(L, false); }
static int lua_CollapseQuestHeader(lua_State* L) { return setQuestHeaderCollapsed(L, true); }

// GetQuestLogSpecialItemInfo(questLogIndex) -> link, texture, charges
//
// Answering nil meant no button was ever built, so a quest that gives you
// something to use looked like one that does not.
static int lua_GetQuestLogSpecialItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const QuestSpecialItem item = questSpecialItemAt(gh, index);
    if (!item.itemId) { return luaReturnNil(L); }

    const auto* info = gh->getItemInfo(item.itemId);
    // The name is what goes in the link, and without it there is no link to
    // return. The query was sent when the quest was read; until it answers,
    // there is honestly nothing to draw.
    if (!info || info->name.empty()) { return luaReturnNil(L); }

    const uint32_t quality = info->quality < 8 ? info->quality : 1u;
    const std::string link = game::itemChatLink(item.itemId, quality, info->name);
    lua_pushstring(L, link.c_str());                                        // 1: link

    // A nil texture is an empty slot to the interface, and the button draws
    // its background art instead of the item.
    const std::string icon = info->displayInfoId
        ? gh->getItemIconPath(info->displayInfoId) : std::string();
    lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                   : icon.c_str());                 // 2: texture
    // WatchFrameItem_OnUpdate compares this against the count it drew with and
    // rebuilds the whole frame when it changes, so it has to be stable.
    lua_pushnumber(L, item.count > 0 ? item.count : 1);             // 3: charges
    return 3;
}

static int lua_UseQuestLogSpecialItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const QuestSpecialItem item = questSpecialItemAt(gh, index);
    if (!item.itemId) return 0;
    if (item.bag == 0) gh->useItemBySlot(item.slot - 1);
    else               gh->useItemInBag(item.bag - 1, item.slot - 1);
    return 0;
}

/// The player's skills in the order the list shows them: by name.
///
/// They live in an unordered_map, and the skill list used to be read out of it
/// by counting to the asked-for position. That order is arbitrary - the list
/// came out in no order a player could recognise - and worse, it is not fixed:
/// learning a skill inserts, inserting can rehash, and rehashing reorders a
/// list already drawn under a selection held as an index. The skill being
/// looked at would quietly become a different one.
///
/// By name, with the id breaking ties, so the answer is the same every time it
/// is asked and reads like a list rather than a spill. Ties matter: two skills
/// can share a name before the name cache has resolved either.
static std::vector<uint32_t> skillOrder(game::GameHandler* gh) {
    std::vector<uint32_t> ids;
    if (!gh) return ids;
    const auto& skills = gh->getPlayerSkills();
    ids.reserve(skills.size());
    for (const auto& [id, skill] : skills) ids.push_back(id);
    std::sort(ids.begin(), ids.end(), [gh](uint32_t a, uint32_t b) {
        const std::string &na = gh->getSkillName(a), &nb = gh->getSkillName(b);
        if (na != nb) return na < nb;
        return a < b;
    });
    return ids;
}

/// One drawn line of the skills tab: a heading, or a skill under one.
///
/// The tab is a flat list with a heading row every so often, exactly like the
/// reputation panel - GetSkillLineInfo answers `header` and `isExpanded` for
/// each row, and a closed heading takes its skills out of the list rather than
/// hiding them in place, because GetNumSkillLines is what the scroll frame
/// counts.
///
/// The grouping was previously reported as absent on the reasoning that "the
/// guard is in the data, not in the frame". The data was there the whole time:
/// getSkillCategory already answered for every skill, and SkillLineCategory.dbc
/// names the eight headings and gives the order to draw them in.
struct SkillRow {
    uint32_t skillId = 0;      ///< 0 for a heading
    uint32_t categoryId = 0;
    bool isHeader = false;
};

static std::vector<SkillRow> skillRows(game::GameHandler* gh) {
    std::vector<SkillRow> rows;
    if (!gh) return rows;

    // Category 12 is "Not Displayed", and it means it. A skill the file does
    // not categorise at all is kept rather than dropped: it is something the
    // player has, and losing it silently is worse than filing it last.
    constexpr uint32_t kNotDisplayed = 12;
    std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> grouped;
    for (uint32_t id : skillOrder(gh)) {
        const uint32_t cat = gh->getSkillCategory(id);
        if (cat == kNotDisplayed) continue;
        grouped[{gh->getSkillCategorySortIndex(cat), cat}].push_back(id);
    }

    for (const auto& [key, ids] : grouped) {
        const uint32_t cat = key.second;
        // A category with no name is one this build's file does not describe;
        // its skills are still listed, just without a heading over them.
        if (cat != 0 && !gh->getSkillCategoryName(cat).empty()) {
            rows.push_back({0, cat, true});
            if (gh->isSkillCategoryCollapsed(cat)) continue;
        }
        for (uint32_t id : ids) rows.push_back({id, cat, false});
    }
    return rows;
}

// --- The stable ---
//
// Everything here was already tracked: the pets, their levels, how many slots
// the player has bought, and the three commands that list, store and retrieve.
// Only the questions the original interface asks were missing, so its stable
// opened on nothing however many pets were in it.
//
// Slot 0 is the pet that is out, and 1 upwards are the ones stabled. That is
// the interface's numbering, not this client's - the pets arrive in one list
// with a flag saying which is active.
namespace {

/// An achievement's artwork, as a path.
///
/// Achievement.dbc names an icon by SpellIcon.dbc id and the loader has been
/// caching that id all along; the two bindings that answer an icon were pushing
/// a constant path and the raw id respectively, so every achievement in the
/// panel wore the same picture. The resolver was already here - the spellbook's
/// tabs use it for the icons SkillLine.dbc names the same way.
///
/// The generic one stays as the fallback: eighty-seven of the 1817 rows name no
/// icon at all, and a blank texture reads as a missing file rather than as a
/// row without artwork.
std::string achievementIconPath(game::GameHandler* gh, uint32_t achievementId) {
    static const std::string kGeneric = "Interface\\Icons\\Achievement_General";
    if (!gh) return kGeneric;
    const uint32_t iconId = gh->getAchievementIconId(achievementId);
    if (iconId == 0) return kGeneric;
    std::string path = gh->getIconPath(iconId);
    return path.empty() ? kGeneric : path;
}

/// The stabled pets, in the order they arrived, with the active one left out.
std::vector<const game::GameHandler::StabledPet*> stabledOnly(game::GameHandler* gh) {
    std::vector<const game::GameHandler::StabledPet*> out;
    if (!gh) return out;
    for (const auto& p : gh->getStabledPets()) {
        if (!p.isActive) out.push_back(&p);
    }
    return out;
}

/// The pet that is currently out, or null if none is.
const game::GameHandler::StabledPet* activePet(game::GameHandler* gh) {
    if (!gh) return nullptr;
    for (const auto& p : gh->getStabledPets()) {
        if (p.isActive) return &p;
    }
    return nullptr;
}

/// Which slot the player has clicked. Held here because it is a fact about the
/// window rather than about the character, and the server is never told.
int& selectedStableSlot() {
    static int slot = 0;
    return slot;
}

} // namespace

// GetStablePetInfo(slot) → icon, name, level, family, talent
//
// The icon is what says a slot is occupied: the frame tests it before deciding
// whether the slot reads as a pet or as an empty box, so an occupied slot must
// answer something and an empty one must answer nil.
//
// Family and talent tree are blank. They come from the creature's family, and
// the stable packet carries the creature entry without it - this client's own
// stable window shows a name and a level for the same reason.
static int lua_GetStablePetInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 1, -1));
    if (!gh || slot < 0) { return luaReturnNil(L); }

    const game::GameHandler::StabledPet* pet = nullptr;
    if (slot == 0) {
        pet = activePet(gh);
    } else {
        const auto stabled = stabledOnly(gh);
        if (slot <= static_cast<int>(stabled.size())) {
            pet = stabled[static_cast<size_t>(slot - 1)];
        }
    }
    if (!pet) { return luaReturnNil(L); }

    lua_pushstring(L, "Interface\\Icons\\Ability_Hunter_BeastTaming");
    lua_pushstring(L, pet->name.empty()
                          ? ("Pet #" + std::to_string(pet->petNumber)).c_str()
                          : pet->name.c_str());
    lua_pushnumber(L, pet->level);
    lua_pushstring(L, "");   // family
    lua_pushstring(L, "");   // pet talent tree
    return 5;
}

static int lua_GetNumStablePets(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(stabledOnly(getGameHandler(L)).size()));
    return 1;
}

static int lua_GetNumStableSlots(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getStableSlots() : 0);
    return 1;
}

static int lua_GetSelectedStablePet(lua_State* L) {
    lua_pushnumber(L, selectedStableSlot());
    return 1;
}

static int lua_ClickStablePet(lua_State* L) {
    selectedStableSlot() = static_cast<int>(luaL_optnumber(L, 1, 0));
    // The model preview shows whichever pet is selected, so selecting a
    // different one is exactly when it has to be redrawn. Nothing else changes
    // it, and the frame will not redraw on its own.
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (engine) engine->fireEvent("PET_STABLE_UPDATE_PAPERDOLL", {});
    return 0;
}

// ---- What a pet eats, and what it trains into ----
//
// Both come from the creature's family. This client learns a family from
// SMSG_CREATURE_QUERY_RESPONSE, which only ever arrives for creatures it has
// seen - and a stabled pet is by definition not in the world, so its family is
// never known. Mapping a family to a diet would need CreatureFamily.dbc on top
// of that, whose field layout does not read cleanly enough to trust.
//
// So these answer absent, and the frame is built for that: GetPetIcon and
// GetPetFoodTypes are both tested before use, and the talent tree is taken as
// `GetPetTalentTree() or ""`.
static int lua_GetPetIcon(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !activePet(gh)) { return luaReturnNil(L); }
    lua_pushstring(L, "Interface\\Icons\\Ability_Hunter_BeastTaming");
    return 1;
}
static int lua_GetPetTalentTree(lua_State* L) { return luaReturnNil(L); }
static int lua_GetPetFoodTypes(lua_State* L)  { return luaReturnNil(L); }

/// SetPetStablePaperdoll(model) - put the selected pet in the preview frame.
///
/// This was a deliberate no-op on the reading that there was no path putting
/// an arbitrary creature in a model frame. There is one now: the portraits
/// needed it, so CharacterPreview will load any M2 by path and the display
/// lookup turns a creature's display id into one.
///
/// The slot is the window's own state, held beside ClickStablePet. The list the
/// server sent carries a creature *template entry* - there is no display id on
/// that wire at all - so it is resolved here, at the read, rather than stored
/// converted: the resolution asks the server the first time and answers zero
/// until the reply lands, and a value frozen at parse time would keep that zero
/// for the life of the window. What is written is the display id; the render
/// loop is where the model is built, because that is where the offscreen views
/// live.
static int lua_SetPetStablePaperdoll(lua_State* L) {
    auto* gh = getGameHandler(L);
    auto* tree = getWidgetTree(L);
    if (!gh || !tree || !lua_istable(L, 1)) return 0;
    lua_getfield(L, 1, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    auto* w = tree->get(id);
    if (!w) return 0;

    // Slots count from one and the active pet is slot zero, which the stable
    // list carries as its own entry rather than as a slot.
    const int slot = selectedStableSlot();
    const auto& pets = gh->getStabledPets();
    w->modelDisplayId = 0;
    if (slot >= 1 && slot <= static_cast<int>(pets.size())) {
        w->modelDisplayId = gh->getCreatureDisplayIdForEntry(
            pets[static_cast<size_t>(slot) - 1].entry);
    }
    return 0;
}

/// PickupStablePet(slot) - start dragging a pet between stable slots.
///
/// Also a deliberate no-op. Moving a pet by dragging needs a cursor that can
/// hold one, and this client's cursor holds items and spells; the buttons the
/// stable frame offers for the same moves go through stablePet and
/// unstablePet, which do work. So the frame loses the drag and keeps the
/// operation.
static int lua_PickupStablePet(lua_State* L) { (void)L; return 0; }

/// GetStablePetFoodTypes(slot) - and the one that cannot answer nil.
///
/// Its result goes straight into format(PET_DIET_TEMPLATE,
/// BuildListString(...)) with no test in between. BuildListString hands nil
/// back for nil, and string.format raises on a nil where it wants a string, so
/// answering honestly there takes the stable window down as it opens. An empty
/// string is the one value that says "not known" without doing that: the diet
/// line comes out blank instead of wrong.
static int lua_GetStablePetFoodTypes(lua_State* L) {
    (void)L;
    lua_pushstring(L, "");
    return 1;
}

static int lua_ClosePetStables(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->closeStableWindow();
    return 0;
}

static int lua_IsAtStableMaster(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isStableWindowOpen() ? 1 : 0);
    return 1;
}

// GetNextStableSlotCost() → what the next slot costs, in copper.
//
// Zero, because the server never says. It reaches a money frame, which divides
// it into gold and silver the moment the window opens, so it has to be a number
// - and a made-up price shown as though the server had quoted it is worse than
// a visible nothing.
static int lua_GetNextStableSlotCost(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

static int lua_GetNumSkillLines(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    // The drawn rows, not the skills: headings are rows too, and a closed one
    // takes its skills out of the list entirely.
    lua_pushnumber(L, static_cast<double>(skillRows(gh).size()));
    return 1;
}

/// Open or close the heading at a drawn-row index. Shared by the two verbs,
/// which differ only in the boolean and must resolve the index the same way.
static int skillHeaderSetCollapsed(lua_State* L, bool collapsed) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) return 0;
    const auto rows = skillRows(gh);
    if (index > static_cast<int>(rows.size())) return 0;
    const auto& row = rows[static_cast<size_t>(index) - 1];
    if (row.isHeader) gh->setSkillCategoryCollapsed(row.categoryId, collapsed);
    return 0;
}

// GetSkillLineInfo(index) → skillName, isHeader, isExpanded, skillRank, numTempPoints, skillModifier, skillMaxRank, isAbandonable, stepCost, rankCost, minLevel, skillCostType
static int lua_GetSkillLineInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    // optnumber, not checknumber: a nil index is a question this can answer
    // with nil, and raising instead takes down whatever file asked. SkillFrame
    // calls SkillDetailFrame_SetStatusBar with no selection during its own
    // load, which is exactly that question.
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    // An index with no skill behind it answers as an empty skill rather than a
    // single nil. SkillFrame_UpdateSkills passes GetSelectedSkill() straight in
    // and adds two of the results together on the next line, and before the
    // server has sent any skills there is nothing selected - so a bare nil
    // there is arithmetic on nothing and the file is lost. An empty row is the
    // truthful answer and it costs no more than a blank line in the list.
    // Short-circuit rather than a ternary: binding a reference across both arms
    // would copy the whole skill map on every call.
    const auto rows = gh ? skillRows(gh) : std::vector<SkillRow>{};
    if (!gh || index < 1 || index > static_cast<int>(rows.size())) {
        lua_pushstring(L, "");                          // 1: skillName
        lua_pushboolean(L, 0);                          // 2: isHeader
        lua_pushboolean(L, 1);                          // 3: isExpanded
        for (int i = 4; i <= 7; ++i) lua_pushnumber(L, 0);   // rank, temp, mod, max
        lua_pushboolean(L, 0);                          // 8: isAbandonable
        // stepCost and rankCost nil for the same reason they are nil below:
        // zero is true in Lua, so a zero here sends the empty row down the
        // "Learn <skill>" branch. The fix went in on the real path and this
        // fallback kept its zeros, which is how a fix half-lands.
        lua_pushnil(L);                                 // 9: stepCost
        lua_pushnil(L);                                 // 10: rankCost
        lua_pushnumber(L, 0);                           // 11: minLevel
        lua_pushnumber(L, 0);                           // 12: skillCostType
        lua_pushstring(L, "");                          // 13: skillDescription
        return 13;
    }
    const auto& row = rows[static_cast<size_t>(index) - 1];
    if (row.isHeader) {
        // A heading carries its name and its open/closed state and nothing
        // else. The costs stay nil for the same reason they are nil below -
        // zero is true in Lua and would send the heading down the
        // "Learn <skill>" branch.
        lua_pushstring(L, gh->getSkillCategoryName(row.categoryId).c_str());
        lua_pushboolean(L, 1);                          // 2: isHeader
        lua_pushboolean(L, gh->isSkillCategoryCollapsed(row.categoryId) ? 0 : 1);
        for (int i = 4; i <= 7; ++i) lua_pushnumber(L, 0);
        lua_pushboolean(L, 0);                          // 8: isAbandonable
        lua_pushnil(L);                                 // 9: stepCost
        lua_pushnil(L);                                 // 10: rankCost
        lua_pushnumber(L, 0);                           // 11: minLevel
        lua_pushnumber(L, 0);                           // 12: skillCostType
        lua_pushstring(L, "");                          // 13: skillDescription
        return 13;
    }
    const auto& skills = gh->getPlayerSkills();
    const auto found = skills.find(row.skillId);
    if (found == skills.end()) { return luaReturnNil(L); }
    const auto& skill = found->second;
    std::string name = gh->getSkillName(skill.skillId);
    if (name.empty()) name = "Skill " + std::to_string(skill.skillId);

    lua_pushstring(L, name.c_str());                    // 1: skillName
    lua_pushboolean(L, 0);                              // 2: isHeader
    lua_pushboolean(L, 1);                              // 3: isExpanded
    lua_pushnumber(L, skill.effectiveValue());           // 4: skillRank
    lua_pushnumber(L, skill.bonusTemp);                  // 5: numTempPoints
    lua_pushnumber(L, skill.bonusPerm);                  // 6: skillModifier
    lua_pushnumber(L, skill.maxValue);                   // 7: skillMaxRank
    lua_pushboolean(L, 0);                              // 8: isAbandonable
    // Nil, not zero, and this is the whole of the "Learn Mounts" mystery.
    //
    // SkillFrame_SetStatusBar branches on these three in order:
    //
    //     if ( stepCost ) then          -- a skill that must be bought
    //         statusBarName:SetFormattedText(LEARN_SKILL_TEMPLATE, skillName)
    //     elseif ( rankCost or numTempPoints > 0 ) then   -- trainable
    //     else                                            -- an ordinary skill
    //
    // Zero is *true* in Lua, so the first branch always won and every row in
    // the skills window was titled "Learn <skill>" - First Aid, Axes, Cooking,
    // everything - as though none of them were known.
    //
    // No purchase cost is tracked here, and nil is how the client says a skill
    // has none. With both nil and no temporary points, the third branch runs
    // and the row is simply the skill's name, which is what it should have
    // been reading all along.
    lua_pushnil(L);                                     // 9: stepCost
    lua_pushnil(L);                                     // 10: rankCost
    lua_pushnumber(L, 0);                               // 11: minLevel
    lua_pushnumber(L, 0);                               // 12: skillCostType
    // The sentence the detail panel prints under the selected skill.
    //
    // Returning twelve values left it nil, and SkillDetailFrame_SetStatusBar
    // feeds it straight into SetFormattedText(SKILL_DESCRIPTION, type, desc).
    // string.format raises on a nil %s, so the guarded SetFormattedText fell
    // back to writing the format string itself - the lower half of the skills
    // window showed "%s %s" where the description belongs.
    lua_pushstring(L, gh->getSkillDescription(skill.skillId).c_str());  // 13
    return 13;
}

// --- Friends/Ignore API ---


/// Whose talents a talent binding is being asked about.
///
/// Every talent binding takes an `inspect` flag that this client ignored, so
/// the inspect talent tab enumerated the viewer's own class tabs and read the
/// viewer's own ranks - the wrong tree under the target's name, rather than an
/// empty one. Zero means the inspect result has no class yet, in which case
/// falling back to the player's is the only thing left to do.
static uint8_t talentClassId(game::GameHandler* gh, bool inspect) {
    if (inspect && gh) {
        if (const auto* r = gh->getInspectResult()) {
            if (r->classId) return r->classId;
        }
    }
    return gh ? gh->getPlayerClass() : 0;
}

/// A class's talent tabs, in the order the interface numbers them.
///
/// Which tab is tab 1 is the whole of this: the interface asks for tabs by
/// index and expects them in orderIndex order, and a tab list built any other
/// way silently renames every tab. Four places worked it out for themselves -
/// GetNumTalentTabs, GetTalentTabInfo, the talent lookup and resolveTalentId -
/// and they agreed, which is luck rather than design in a file where the index
/// conventions have bitten before.
static std::vector<const game::GameHandler::TalentTabEntry*> classTalentTabs(
        game::GameHandler* gh, uint8_t classId) {
    std::vector<const game::GameHandler::TalentTabEntry*> tabs;
    if (!gh || classId == 0) return tabs;
    const uint32_t classMask = 1u << (classId - 1);
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) tabs.push_back(&tab);
    }
    std::sort(tabs.begin(), tabs.end(),
              [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });
    return tabs;
}

/// The rank a talent is at, for whoever is being asked about.
static int talentRankFor(game::GameHandler* gh, bool inspect, uint32_t talentId) {
    if (!gh) return 0;
    if (inspect) {
        if (const auto* r = gh->getInspectResult()) {
            auto it = r->talentRanks.find(talentId);
            return (it != r->talentRanks.end()) ? it->second : 0;
        }
        return 0;
    }
    return gh->getTalentRank(talentId);
}

/// The fourth argument asks for the *pet's* tree.
///
/// Nothing here tracks pet talents - no packet fills them and no table holds
/// them - and ignoring the argument answered out of the player's trees
/// instead, so a hunter's pet tab drew the hunter's own talents. Ten nils is
/// the honest answer and leaves the tab empty; talentframebase treats a nil
/// name as "no talent here", which is what it is.
static bool wantsPetTalents(lua_State* L, int index) {
    return lua_toboolean(L, index) != 0;
}

static int lua_GetNumTalentTabs(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (wantsPetTalents(L, 2)) { return luaReturnZero(L); }
    if (!gh) { return luaReturnZero(L); }
    // Count tabs matching the class in question
    uint8_t classId = talentClassId(gh, lua_toboolean(L, 1) != 0);
    uint32_t classMask = (classId > 0) ? (1u << (classId - 1)) : 0;
    int count = 0;
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) count++;
    }
    lua_pushnumber(L, count);
    return 1;
}

/// Points staged in the talent preview but not yet learned, defined below with
/// the functions that change it.
static std::unordered_map<uint32_t, int>& previewPoints();

// GetTalentTabInfo(tabIndex) → name, iconTexture, pointsSpent, background
static int lua_GetTalentTabInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIndex = static_cast<int>(luaL_checknumber(L, 1)); // 1-indexed
    if (!gh || tabIndex < 1) {
        return luaReturnNil(L);
    }
    if (wantsPetTalents(L, 3)) return luaReturnNil(L);
    const bool inspect = lua_toboolean(L, 2) != 0;
    uint8_t classId = talentClassId(gh, inspect);
    const auto classTabs = classTalentTabs(gh, classId);
    if (tabIndex > static_cast<int>(classTabs.size())) {
        return luaReturnNil(L);
    }
    const auto* tab = classTabs[tabIndex - 1];
    // Count points spent in this tab
    int pointsSpent = 0;
    static const std::unordered_map<uint32_t, uint8_t> kNoTalents;
    const auto* inspectResult = inspect ? gh->getInspectResult() : nullptr;
    const auto& learned = inspect
        ? (inspectResult ? inspectResult->talentRanks : kNoTalents)
        : gh->getLearnedTalents();
    for (const auto& [talentId, rank] : learned) {
        const auto* entry = gh->getTalentEntry(talentId);
        if (entry && entry->tabId == tab->tabId) pointsSpent += rank;
    }
    // Points staged in the preview but not yet learned, for this tab. The
    // talent frame adds this to the spent count without checking it -
    //     local displayPointsSpent = pointsSpent + previewPointsSpent;
    // - in a loop over every tab, so leaving it out took the whole frame down
    // as it opened rather than merely showing the wrong total.
    // Staged points are the viewer part-way through spending their own; they
    // have no meaning on someone else's tree.
    int previewSpent = 0;
    if (!inspect) {
        for (const auto& [talentId, staged] : previewPoints()) {
            const auto* entry = gh->getTalentEntry(talentId);
            if (entry && entry->tabId == tab->tabId) previewSpent += staged;
        }
    }

    lua_pushstring(L, tab->name.c_str());              // 1: name
    lua_pushnil(L);                                     // 2: iconTexture (not resolved)
    lua_pushnumber(L, pointsSpent);                     // 3: pointsSpent
    lua_pushstring(L, tab->backgroundFile.c_str());     // 4: background
    lua_pushnumber(L, previewSpent);                    // 5: previewPointsSpent
    return 5;
}

// GetNumTalents(tabIndex) → count
static int lua_GetNumTalents(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIndex = static_cast<int>(luaL_checknumber(L, 1));
    // The counts belong to the same family as the readers below and were
    // missed with them: a pet tab whose talents all answer nil still asked how
    // many to lay out, and got the player's tree's count.
    if (wantsPetTalents(L, 3)) { return luaReturnZero(L); }
    if (!gh || tabIndex < 1) { return luaReturnZero(L); }
    uint8_t classId = talentClassId(gh, lua_toboolean(L, 2) != 0);
    const auto classTabs = classTalentTabs(gh, classId);
    if (tabIndex > static_cast<int>(classTabs.size())) {
        return luaReturnZero(L);
    }
    uint32_t targetTabId = classTabs[tabIndex - 1]->tabId;
    int count = 0;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (entry.tabId == targetTabId) count++;
    }
    lua_pushnumber(L, count);
    return 1;
}

// GetTalentInfo(tabIndex, talentIndex) → name, iconTexture, tier, column, rank, maxRank, isExceptional, available
/// Which quest the confirmation is about. Set when the log asks to abandon one
/// and read back by the popup that confirms it, because the two are separate
/// calls with the player's answer in between.
static uint32_t& pendingAbandonQuest() {
    static uint32_t questId = 0;
    return questId;
}

/// The talent at a tab and index, by the ordering every talent function has to
/// agree on: the player's own tabs in their order index, then the tab's talents
/// by row and column.
///
/// Shared rather than repeated, because two copies that drift disagree about
/// which talent is fourth - and the prerequisite lines are drawn between
/// positions, so a disagreement points an arrow at the wrong button rather than
/// failing outright.
// Not static: the tooltip setters in lua_engine.cpp ask the same question, and
// two copies of this would have to keep the same tab ordering and the same
// row/column sort forever or the tooltip would describe a different talent from
// the one under the cursor. Declared in lua_api_helpers.hpp.

const game::TalentEntry* talentAt(game::GameHandler* gh,
                                  int tabIndex, int talentIndex,
                                  uint8_t classIdOverride) {
    if (!gh || tabIndex < 1 || talentIndex < 1) return nullptr;
    const uint8_t classId = classIdOverride ? classIdOverride : gh->getPlayerClass();
    const auto classTabs = classTalentTabs(gh, classId);
    if (tabIndex > static_cast<int>(classTabs.size())) return nullptr;

    const uint32_t targetTabId = classTabs[tabIndex - 1]->tabId;
    std::vector<const game::GameHandler::TalentEntry*> tabTalents;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (entry.tabId == targetTabId) tabTalents.push_back(&entry);
    }
    std::sort(tabTalents.begin(), tabTalents.end(),
        [](const auto* a, const auto* b) {
            return (a->row != b->row) ? a->row < b->row : a->column < b->column;
        });
    if (talentIndex > static_cast<int>(tabTalents.size())) return nullptr;
    return tabTalents[talentIndex - 1];
}


/// Whether every prerequisite of a talent is satisfied, optionally counting
/// points staged in the preview but not yet learned.
static bool talentPrereqsMet(game::GameHandler* gh,
                             const game::GameHandler::TalentEntry* talent,
                             bool withPreview) {
    if (!gh || !talent) return false;
    for (int p = 0; p < 3; ++p) {
        const uint32_t prereqId = talent->prereqTalent[p];
        if (prereqId == 0) continue;
        // Counted from zero in the DBC, so the rank asked for is one more.
        const int needed = static_cast<int>(talent->prereqRank[p]) + 1;
        int have = gh->getTalentRank(prereqId);
        if (withPreview) {
            const auto staged = previewPoints().find(prereqId);
            if (staged != previewPoints().end()) have += staged->second;
        }
        if (have < needed) return false;
    }
    return true;
}

static int lua_GetTalentInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int tabIndex = static_cast<int>(luaL_checknumber(L, 1));
    const int talentIndex = static_cast<int>(luaL_checknumber(L, 2));
    const bool inspect = lua_toboolean(L, 3) != 0;
    if (wantsPetTalents(L, 4)) {
        for (int i = 0; i < 10; i++) lua_pushnil(L);
        return 10;
    }
    const auto* talent = talentAt(gh, tabIndex, talentIndex, talentClassId(gh, inspect));
    // Ten values, not eight. The frame reads previewRank into the rank it
    // displays and then compares it against maxRank - as nil that is an error
    // rather than a blank, and it happens the moment points are staged, which
    // is how talents are spent at all.
    if (!talent) {
        for (int i = 0; i < 10; i++) lua_pushnil(L);
        return 10;
    }
    const int rank = talentRankFor(gh, inspect, talent->talentId);
    const auto staged = previewPoints().find(talent->talentId);
    const int previewRank = inspect
        ? rank
        : rank + (staged == previewPoints().end() ? 0 : staged->second);

    std::string name = gh->getSpellName(talent->rankSpells[0]);
    if (name.empty()) name = "Talent " + std::to_string(talent->talentId);
    // A nil texture is an empty slot to the interface, so every talent button
    // was drawn blank.
    const std::string icon = gh->getSpellIconPath(talent->rankSpells[0]);

    lua_pushstring(L, name.c_str());          // 1: name
    if (icon.empty()) lua_pushnil(L);          // 2: iconTexture
    else lua_pushstring(L, icon.c_str());
    lua_pushnumber(L, talent->row + 1);        // 3: tier (1-indexed)
    lua_pushnumber(L, talent->column + 1);     // 4: column (1-indexed)
    lua_pushnumber(L, rank);                   // 5: rank
    lua_pushnumber(L, talent->maxRank);        // 6: maxRank
    lua_pushboolean(L, 0);                     // 7: isExceptional
    // Was hardcoded true, which drew every talent as learnable however deep in
    // a chain it sat.
    lua_pushboolean(L, talentPrereqsMet(gh, talent, /*withPreview=*/false) ? 1 : 0);
    lua_pushnumber(L, previewRank);            // 9: previewRank
    lua_pushboolean(L, talentPrereqsMet(gh, talent, /*withPreview=*/true) ? 1 : 0);
    return 10;
}

/// The trainer panel's own selection and filters. The client has no opinion
/// about either - they are what the player last clicked - so they live here
/// rather than being invented on every read.
static int& tradeSkillSelection() {
    static int selected = 0;
    return selected;
}

/// The trade skill list's two filters, and the view every binding reads.
///
/// The panel's search box and its "Have Materials" checkbox were no-ops, so
/// typing in one or ticking the other changed nothing. Filtering has to happen
/// in one place: the panel asks GetNumTradeSkills for a count and then indexes
/// everything else by position in that same list, so a count filtered anywhere
/// but here would have the rows describing different recipes than the ones
/// counted.
static std::string& tradeSkillNameFilter() {
    static std::string filter;
    return filter;
}
static bool& tradeSkillOnlyMakeable() {
    static bool only = false;
    return only;
}

/// The recipes the panel should show, in its order.
///
/// Returned by value, as getCraftingRecipes already was at every one of these
/// call sites - the copy is the list, not an extra one.
/// The item a recipe makes, or null while its details are still being asked
/// for. The spell cache carries createdItemId; the item query fills the rest.
static const game::ItemQueryResponseData* craftedItem(game::GameHandler* gh,
                                                      uint32_t spellId) {
    if (!gh || spellId == 0) return nullptr;
    auto it = gh->spellNameCacheRef().find(spellId);
    if (it == gh->spellNameCacheRef().end()) return nullptr;
    const uint32_t made = it->second.createdItemId;
    if (made == 0) return nullptr;
    gh->ensureItemInfo(made);
    const auto* info = gh->getItemInfo(made);
    return (info && info->valid) ? info : nullptr;
}

/// Which entry of each dropdown is picked. Both are single-select - the click
/// handler calls Set…Filter(id - 1, 1, 1) and nothing ever unchecks - so this
/// is an index rather than a set, and zero is the "all" row the list opens
/// with.
static int& tradeSkillSubClassPick() { static int pick = 0; return pick; }
static int& tradeSkillInvSlotPick()  { static int pick = 0; return pick; }

/// The item level range typed into the search box. The box does double duty:
/// a number, a range "20-30" or an approximate "~25" filters by the level of
/// what a recipe makes, and anything else filters by name - the panel decides
/// which and calls one of the two, clearing the other. Both zero means no
/// range, which is how it clears this one.
static std::pair<int, int>& tradeSkillLevelRange() {
    static std::pair<int, int> range{0, 0};
    return range;
}

/// The two dropdown lists, built from every known recipe rather than from the
/// filtered ones - a list that shrank as it was filtered would renumber itself
/// under the selection, and the selection is an index into it.
static std::vector<std::string> tradeSkillSubClasses(game::GameHandler* gh) {
    std::vector<std::string> out;
    if (!gh) return out;
    for (const auto& r : gh->getCraftingRecipes()) {
        const auto* info = craftedItem(gh, r.spellId);
        if (!info || info->subclassName.empty()) continue;
        if (std::find(out.begin(), out.end(), info->subclassName) == out.end())
            out.push_back(info->subclassName);
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// The equipment slots the recipes make something for, by the same labels the
/// auction house filters by - one table for both rather than a second list of
/// slot names that could drift from it.
static std::vector<std::string> tradeSkillInvSlots(game::GameHandler* gh) {
    std::vector<std::string> out;
    if (!gh) return out;
    for (const auto& r : gh->getCraftingRecipes()) {
        const auto* info = craftedItem(gh, r.spellId);
        if (!info || info->inventoryType == 0) continue;
        for (int i = 1; i < game::kNumAuctionSlots; ++i) {
            if (game::kAuctionSlots[i].invType != info->inventoryType) continue;
            const std::string label = game::kAuctionSlots[i].label;
            if (std::find(out.begin(), out.end(), label) == out.end())
                out.push_back(label);
            break;
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

static std::vector<game::GameHandler::CraftRecipe> visibleCraftingRecipes(
        game::GameHandler* gh) {
    std::vector<game::GameHandler::CraftRecipe> out;
    if (!gh) return out;
    std::string needle = tradeSkillNameFilter();
    toLowerInPlace(needle);
    const auto subs  = (tradeSkillSubClassPick() > 0) ? tradeSkillSubClasses(gh)
                                                      : std::vector<std::string>{};
    const auto slots = (tradeSkillInvSlotPick() > 0)  ? tradeSkillInvSlots(gh)
                                                      : std::vector<std::string>{};
    const auto level = tradeSkillLevelRange();
    for (const auto& r : gh->getCraftingRecipes()) {
        if (tradeSkillOnlyMakeable() && r.canMake <= 0) continue;
        if (!needle.empty()) {
            std::string name = r.name;
            toLowerInPlace(name);
            if (name.find(needle) == std::string::npos) continue;
        }
        if (level.second > 0) {
            const auto* made = craftedItem(gh, r.spellId);
            // Shown while unknown, for the same reason as the two filters
            // below: the details arrive on demand.
            if (made && (static_cast<int>(made->itemLevel) < level.first ||
                         static_cast<int>(made->itemLevel) > level.second)) {
                continue;
            }
        }
        // A recipe whose item has not arrived yet is shown rather than hidden.
        // The details are asked for on demand, so filtering on what is not back
        // would empty the list and refill it as the replies landed.
        if (!subs.empty() || !slots.empty()) {
            const auto* info = craftedItem(gh, r.spellId);
            if (info) {
                const int si = tradeSkillSubClassPick();
                if (si > 0 && si <= static_cast<int>(subs.size()) &&
                    info->subclassName != subs[static_cast<size_t>(si) - 1]) {
                    continue;
                }
                const int vi = tradeSkillInvSlotPick();
                if (vi > 0 && vi <= static_cast<int>(slots.size())) {
                    std::string label;
                    for (int i = 1; i < game::kNumAuctionSlots; ++i) {
                        if (game::kAuctionSlots[i].invType == info->inventoryType) {
                            label = game::kAuctionSlots[i].label;
                            break;
                        }
                    }
                    if (label != slots[static_cast<size_t>(vi) - 1]) continue;
                }
            }
        }
        out.push_back(r);
    }
    return out;
}

/// One drawn line of the crafting list: a subclass heading, or a recipe.
///
/// Blizzard's own name for the verb says what the headings are -
/// CollapseTradeSkillSubClass - and the subclass is the crafted item's, which
/// is the same source the subclass filter dropdown beside the list is built
/// from. One rule for both, so the headings and the filter cannot disagree.
///
/// A recipe whose item has not arrived yet has no subclass to file it under.
/// Those go last, under no heading, rather than being hidden: the details are
/// asked for on demand, and hiding them would empty the list and refill it as
/// the replies landed. They move under their heading once the reply arrives,
/// which is the same churn the dropdown already has.
struct TradeSkillRow {
    game::GameHandler::CraftRecipe recipe;
    std::string subclass;
    bool isHeader = false;
};

/// Headings the player has closed, by subclass name. Not saved: the crafting
/// window is opened against one profession at a time and the original client
/// does not carry the state between openings either.
static std::set<std::string>& collapsedTradeSkillSubClasses() {
    static std::set<std::string> closed;
    return closed;
}

/// Bumped whenever the list is allowed to change shape. See tradeSkillRows.
static uint64_t& tradeSkillRowsGeneration() {
    static uint64_t gen = 1;
    return gen;
}

static std::vector<TradeSkillRow> buildTradeSkillRows(game::GameHandler* gh) {
    std::vector<TradeSkillRow> rows;
    if (!gh) return rows;
    std::map<std::string, std::vector<game::GameHandler::CraftRecipe>> grouped;
    std::vector<game::GameHandler::CraftRecipe> ungrouped;
    for (const auto& r : visibleCraftingRecipes(gh)) {
        const auto* info = craftedItem(gh, r.spellId);
        if (info && !info->subclassName.empty()) grouped[info->subclassName].push_back(r);
        else                                     ungrouped.push_back(r);
    }
    for (const auto& [subclass, recipes] : grouped) {
        TradeSkillRow header;
        header.subclass = subclass;
        header.isHeader = true;
        header.recipe.name = subclass;
        rows.push_back(std::move(header));
        // A closed heading takes its recipes out of the list rather than
        // hiding them in place: GetNumTradeSkills is what the scroll frame
        // counts and every other binding indexes into.
        if (collapsedTradeSkillSubClasses().count(subclass)) continue;
        for (const auto& r : recipes) rows.push_back({r, subclass, false});
    }
    for (const auto& r : ungrouped) rows.push_back({r, {}, false});
    return rows;
}

/// The drawn rows, held still between the events that may change them.
///
/// Every trade skill binding indexes into this list, and the frame reads it
/// many times to draw one selection - GetNumTradeSkills for the count,
/// GetTradeSkillInfo for the row, GetTradeSkillReagentInfo for each reagent.
/// Rebuilding it per call let it change shape *between* those reads, because
/// its shape depends on data that arrives while they run: the headings come
/// from the subclass of the item each recipe makes, craftedItem() asks for that
/// item the first time it is missed, and the answer lands some frames later.
/// So the list starts flat, and grows a heading - and one row - for each reply.
///
/// The frame has no way to survive that. An index means a different recipe on
/// either side of a reply, so the selection's own row read back as a heading:
/// `creatable` was cleared, both Create buttons stayed disabled, the detail
/// pane described one recipe while the highlight sat on another, and a recipe
/// row was drawn with a heading's expand box beside it.
///
/// So the list is built once and held until something the player did is meant
/// to change it - a filter, a heading opened or closed - or the game says the
/// trade skill data itself moved. Late item info then lands at the next
/// TRADE_SKILL_UPDATE rather than in the middle of a redraw.
/// The cached row list, by reference.
///
/// This returned by value, and every caller does
///
///     const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
///
/// which takes a pointer into the temporary the call just made and reads it on
/// the next line, after the temporary is gone. Reported as crafting doing
/// nothing: DoTradeSkill said `row 4 '' spell=0` and then
/// `row 1 '' spell=2661056113` for the same window - an empty name and a spell
/// id out of freed memory, which is what a press of Create was sending.
///
/// Safe as a reference because the list is deliberately held still between
/// TRADE_SKILL_UPDATE and TRADE_SKILL_SHOW - see fireEvent, which invalidates
/// it only there, precisely because the frame reads it many times to draw one
/// selection. Nothing can rebuild it underneath a caller mid-read.
static const std::vector<TradeSkillRow>& tradeSkillRows(game::GameHandler* gh) {
    static std::vector<TradeSkillRow> cached;
    static uint64_t cachedGen = 0;
    static const game::GameHandler* cachedFor = nullptr;
    if (cachedGen == tradeSkillRowsGeneration() && cachedFor == gh) return cached;
    cached = buildTradeSkillRows(gh);
    cachedGen = tradeSkillRowsGeneration();
    cachedFor = gh;
    return cached;
}

void invalidateTradeSkillRows() { ++tradeSkillRowsGeneration(); }

/// The recipe at a drawn-row index, or null for a heading or no such row.
/// Every binding taking a trade skill index goes through this, because an
/// index that lands on a heading must not be treated as the recipe that used
/// to sit at that offset - DoTradeSkill would craft it.
/// Refuses a temporary outright, because the pointer it answers with would
/// outlive it. That is the fault above, and a compiler error is a better guard
/// than remembering.
static const game::GameHandler::CraftRecipe* tradeSkillRecipeAt(
        std::vector<TradeSkillRow>&&, int) = delete;

static const game::GameHandler::CraftRecipe* tradeSkillRecipeAt(
        const std::vector<TradeSkillRow>& rows, int index) {
    if (index < 1 || index > static_cast<int>(rows.size())) return nullptr;
    const auto& row = rows[static_cast<size_t>(index) - 1];
    return row.isHeader ? nullptr : &row.recipe;
}

/// Open or close the heading at a drawn-row index. A click on a recipe row is
/// ignored rather than refused: the panel calls these from the row template
/// whatever the row turns out to be.
static int tradeSkillHeaderSetCollapsed(lua_State* L, bool collapsed) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) return 0;
    const auto rows = tradeSkillRows(gh);
    if (index > static_cast<int>(rows.size())) return 0;
    const auto& row = rows[static_cast<size_t>(index) - 1];
    if (!row.isHeader) return 0;
    if (collapsed) collapsedTradeSkillSubClasses().insert(row.subclass);
    else           collapsedTradeSkillSubClasses().erase(row.subclass);
    ++tradeSkillRowsGeneration();
    return 0;
}

static int& trainerSelection() {
    static int selected = 0;
    return selected;
}
static std::unordered_map<std::string, bool>& trainerFilters() {
    static std::unordered_map<std::string, bool> filters;
    return filters;
}

/// The word the trainer panel colours a service by, and the name of the filter
/// that hides it.
///
/// "used" wins the moment the spell is known rather than only when the packet
/// said so: the list is a snapshot from when the trainer was opened, and
/// learning something fires TRAINER_UPDATE without a fresh one arriving.
static const char* trainerServiceCategory(game::GameHandler* gh,
                                          const game::TrainerSpell& sp) {
    if (gh && gh->getSpellHandler() && gh->getSpellHandler()->hasKnownSpell(sp.spellId))
        return "used";
    return sp.state == 0 ? "available" : sp.state == 2 ? "used" : "unavailable";
}

/// Whether the filter dropdown is currently letting this category through.
/// Unset means showing, which is how a freshly opened panel looks.
static bool trainerCategoryShowing(const char* category) {
    const auto& filters = trainerFilters();
    auto it = filters.find(category);
    return it == filters.end() || it->second;
}

/// The trainer's list as the panel sees it - the services the filters leave.
///
/// The filter was stored and read back and consulted by nothing, so ticking a
/// box in the dropdown changed the tick and not the list. It has to be applied
/// here rather than at each reader, because every one of the nine bindings that
/// takes a service index has to agree about which service index 3 is: filtering
/// in one of them and not the rest would train a different spell than the row
/// that was clicked.
static std::vector<const game::TrainerSpell*> shownTrainerServices(game::GameHandler* gh) {
    std::vector<const game::TrainerSpell*> shown;
    if (!gh) return shown;
    for (const auto& sp : gh->getTrainerSpells().spells) {
        if (trainerCategoryShowing(trainerServiceCategory(gh, sp))) shown.push_back(&sp);
    }
    return shown;
}

/// The nth shown service, counting from one, or null.
static const game::TrainerSpell* shownTrainerService(game::GameHandler* gh, int index) {
    const auto shown = shownTrainerServices(gh);
    if (index < 1 || index > static_cast<int>(shown.size())) return nullptr;
    return shown[index - 1];
}

/// (tabIndex, talentIndex) → talent id, or 0.
///
/// The same ordering GetTalentInfo reports by - tabs in orderIndex, talents by
/// row then column - because the interface identifies a talent by its position
/// in that listing and nothing else.
static uint32_t resolveTalentId(game::GameHandler* gh, int tabIndex, int talentIndex) {
    if (!gh || tabIndex < 1 || talentIndex < 1) return 0;
    const uint8_t classId = gh->getPlayerClass();
    const auto classTabs = classTalentTabs(gh, classId);
    if (tabIndex > static_cast<int>(classTabs.size())) return 0;

    std::vector<const game::GameHandler::TalentEntry*> tabTalents;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (entry.tabId == classTabs[tabIndex - 1]->tabId) tabTalents.push_back(&entry);
    }
    std::sort(tabTalents.begin(), tabTalents.end(),
              [](const auto* a, const auto* b) {
                  return (a->row != b->row) ? a->row < b->row : a->column < b->column;
              });
    if (talentIndex > static_cast<int>(tabTalents.size())) return 0;
    return tabTalents[talentIndex - 1]->talentId;
}

/// The points staged but not yet sent, per talent.
///
/// WotLK's talent frame does not spend a point when one is clicked: it adds to
/// a preview, redraws from the preview, and sends the lot when Learn is
/// pressed. Without somewhere to hold that, clicking a talent did nothing at
/// all. Keyed by talent id so it survives a tab change, and cleared whenever
/// the server confirms what was learned.
static std::unordered_map<uint32_t, int>& previewPoints() {
    static std::unordered_map<uint32_t, int> points;
    return points;
}

// AddPreviewTalentPoints(tabIndex, talentIndex, delta, pet, group)
static int lua_AddPreviewTalentPoints(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int tab   = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    const int delta = static_cast<int>(luaL_optnumber(L, 3, 1));
    // The pet tree again: staging a preview point there resolved against the
    // player's trees, and LearnPreviewTalents would then have spent it.
    if (wantsPetTalents(L, 4)) return 0;
    const uint32_t id = resolveTalentId(gh, tab, index);
    if (!gh || id == 0) return 0;

    const uint8_t have = gh->getTalentRank(id);
    uint8_t maxRank = 0;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (talentId == id) { maxRank = entry.maxRank; break; }
    }
    int& staged = previewPoints()[id];
    // Bounded by what is already learned and the talent's own cap: a preview
    // that goes past either is one the server will refuse, and the frame draws
    // straight from these numbers.
    int upper = static_cast<int>(maxRank) - have;

    // And bounded by the points the player actually has. This was missing, so
    // a preview could stage more than were available and go on staging them:
    // GetUnspentTalentPoints subtracts the staged total and floors the answer
    // at zero, which hid the overspend instead of stopping it - the counter sat
    // at 0 while every further click still took.
    //
    // Counted against every staged point rather than this talent's, because the
    // pool is shared across all three trees, and only when adding: taking a
    // staged point back is always allowed however far past the pool it went.
    if (delta > 0) {
        int stagedTotal = 0;
        for (const auto& [tid, n] : previewPoints()) { (void)tid; stagedTotal += n; }
        const int unspent =
            gh->getUnspentTalentPoints(gh->getActiveTalentSpec()) - stagedTotal;
        upper = std::min(upper, staged + std::max(0, unspent));
    }
    const int wanted = std::clamp(staged + delta, 0, std::max(0, upper));
    staged = wanted;
    if (staged == 0) previewPoints().erase(id);
    // What makes a staged point appear: the talent frame refreshes on this
    // event, and reads the rank back through GetTalentInfo's preview value.
    gh->fireAddonEvent("PREVIEW_TALENT_POINTS_CHANGED", {});
    return 0;
}

// GetTalentPrereqs(tab, index) → tier, column, isLearnable, isPreviewLearnable
//                                 repeated once per prerequisite
//
// The frame draws a line from each prerequisite to the talent that needs it, so
// the positions here have to be the same ones GetTalentInfo reports - 1-indexed
// row and column, from the same ordering.
static int lua_GetTalentPrereqs(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int tabIndex = static_cast<int>(luaL_checknumber(L, 1));
    const int talentIndex = static_cast<int>(luaL_checknumber(L, 2));
    if (wantsPetTalents(L, 4)) return 0;
    const auto* talent = talentAt(gh, tabIndex, talentIndex);
    if (!talent) return 0;

    int pushed = 0;
    for (int p = 0; p < 3; ++p) {
        const uint32_t prereqId = talent->prereqTalent[p];
        if (prereqId == 0) continue;
        const auto* prereq = gh->getTalentEntry(prereqId);
        if (!prereq) continue;

        // The DBC counts ranks from zero, so the rank a prerequisite asks for
        // is one more than the number stored beside it.
        const int needed = static_cast<int>(talent->prereqRank[p]) + 1;
        const int have = gh->getTalentRank(prereqId);
        const auto staged = previewPoints().find(prereqId);
        const int previewHave = have + (staged == previewPoints().end() ? 0 : staged->second);

        lua_pushnumber(L, prereq->row + 1);
        lua_pushnumber(L, prereq->column + 1);
        lua_pushboolean(L, have >= needed ? 1 : 0);
        lua_pushboolean(L, previewHave >= needed ? 1 : 0);
        pushed += 4;
    }
    return pushed;
}

// GetGroupPreviewTalentPointsSpent(pet, group) → points staged
static int lua_GetGroupPreviewTalentPointsSpent(lua_State* L) {
    int total = 0;
    for (const auto& [id, n] : previewPoints()) { (void)id; total += n; }
    lua_pushnumber(L, total);
    return 1;
}

// ResetGroupPreviewTalentPoints(pet, group)
static int lua_ResetGroupPreviewTalentPoints(lua_State* L) {
    previewPoints().clear();
    // The frame redraws on this and on nothing else, so clearing the staged
    // points without it leaves them on screen after they are gone.
    if (auto* gh = getGameHandler(L)) {
        gh->fireAddonEvent("PREVIEW_TALENT_POINTS_CHANGED", {});
    }
    return 0;
}

// LearnPreviewTalents(pet) - commit the staged points
static int lua_LearnPreviewTalents(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    // A copy, because learnTalent goes to the server and what comes back
    // clears this map - iterating it while that happens is not safe.
    const auto staged = previewPoints();
    for (const auto& [id, n] : staged) {
        const uint8_t have = gh->getTalentRank(id);
        // One request per rank: the server counts them, and asking for the
        // final rank in one call is not how the protocol reads it.
        for (int r = 1; r <= n; ++r) gh->learnTalent(id, have + r);
    }
    previewPoints().clear();
    return 0;
}

// LearnTalent(tabIndex, talentIndex, pet, group) - spend one point directly
static int lua_LearnTalent(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Pet talents are not modelled, and this is the argument that made that
    // dangerous rather than merely blank: clicking in a pet tree resolved the
    // same tab and index against the *player's* trees and spent a point there.
    if (wantsPetTalents(L, 3)) return 0;
    const uint32_t id = resolveTalentId(gh, static_cast<int>(luaL_optnumber(L, 1, 0)),
                                            static_cast<int>(luaL_optnumber(L, 2, 0)));
    if (gh && id != 0) gh->learnTalent(id, gh->getTalentRank(id) + 1);
    return 0;
}

// GetUnspentTalentPoints(inspect, pet, group) → points
static int lua_GetUnspentTalentPoints(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); return 1; }
    const int group = static_cast<int>(luaL_optnumber(L, 3, 0));
    const uint8_t spec = (group >= 1 && group <= 2) ? static_cast<uint8_t>(group - 1)
                                                   : gh->getActiveTalentSpec();
    // The raw unspent total, before any staged preview points.
    //
    // The frame subtracts the preview itself: TalentFrame_UpdateTalentPoints is
    // `GetUnspentTalentPoints() - GetGroupPreviewTalentPointsSpent()`. This
    // binding used to subtract the staged points as well, so every point
    // clicked came off the counter twice - stage one and the "Talent Points"
    // line dropped by two. The overspend that subtraction was meant to stop is
    // already stopped in AddPreviewTalentPoints, which bounds each stage by the
    // handler's own raw unspent minus what is already staged, so nothing here
    // needs to guard it. blizzard_talentui reads this raw too, to decide
    // whether there are points left to commit - which must count the staged
    // ones, or the Learn controls vanish the moment the last point is staged.
    const int points = gh->getUnspentTalentPoints(spec);
    lua_pushnumber(L, points);
    return 1;
}

// GetNumTalentGroups(inspect, pet) → how many specs the player has
static int lua_GetNumTalentGroups(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Two only once the second is actually bought: the frame draws a spec tab
    // per group, and reporting two unconditionally offers one that is not there.
    int groups = 1;
    if (gh && lua_toboolean(L, 1)) {
        // The inspected player's spec count comes with their talents; the
        // viewer's own says nothing about how many specs the target bought.
        if (const auto* r = gh->getInspectResult()) {
            if (r->talentGroups > 0) groups = r->talentGroups;
        }
        lua_pushnumber(L, groups);
        return 1;
    }
    if (gh && gh->getUnspentTalentPoints(1) > 0) groups = 2;
    if (gh && !gh->getLearnedTalents(1).empty()) groups = 2;
    lua_pushnumber(L, groups);
    return 1;
}

// SetActiveTalentGroup(group)
static int lua_SetActiveTalentGroup(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int group = static_cast<int>(luaL_optnumber(L, 1, 1));
    if (gh && group >= 1 && group <= 2) {
        gh->switchTalentSpec(static_cast<uint8_t>(group - 1));
    }
    return 0;
}

// GetTalentLink(tabIndex, talentIndex, inspect, group) → hyperlink
static int lua_GetTalentLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t id = resolveTalentId(gh, static_cast<int>(luaL_optnumber(L, 1, 0)),
                                            static_cast<int>(luaL_optnumber(L, 2, 0)));
    if (!gh || id == 0) { lua_pushnil(L); return 1; }
    uint32_t rank1 = 0;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (talentId == id) { rank1 = entry.rankSpells[0]; break; }
    }
    std::string name = gh->getSpellName(rank1);
    if (name.empty()) name = "Talent";
    const std::string link = "|cff4e96f7|Htalent:" + std::to_string(id) + ":" +
                             std::to_string(gh->getTalentRank(id)) + "|h[" + name +
                             "]|h|r";
    lua_pushstring(L, link.c_str());
    return 1;
}

static int lua_GetActiveTalentGroup(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh && lua_toboolean(L, 1)) {
        // Which of the target's specs the talents just read belong to. The
        // inspect frame keeps this as its talentGroup and passes it back into
        // every other talent call.
        const auto* r = gh->getInspectResult();
        lua_pushnumber(L, r ? (r->activeTalentGroup + 1) : 1);
        return 1;
    }
    lua_pushnumber(L, gh ? (gh->getActiveTalentSpec() + 1) : 1);
    return 1;
}


namespace {
/// A template because the dialog carries its rewards in a vector and the quest
/// log in a fixed array, and both are padded the same way.
template <typename Container>
int countRewards(const Container* v) {
    if (!v) return 0;
    int n = 0;
    for (const auto& r : *v) if (r.itemId != 0) ++n;
    return n;
}
}  // namespace

// --- The quest log's own reward panel ---
//
// QuestInfo draws rewards for the quest log as well as for the quest giver, and
// asks a different set of functions for each. The giver's were written earlier;
// these read the same fields out of the log entry the player has selected.

/// The quest log entry the panel is showing, or null.
static const game::GameHandler::QuestLogEntry* selectedLogEntry(game::GameHandler* gh) {
    if (!gh) return nullptr;
    // The selected index is a display index - the log is grouped under zone
    // headers and those are rows too - so it is resolved through the row list
    // rather than subscripting getQuestLog(). Every reward binding reads the
    // selected quest through here, so getting it wrong shows one quest's
    // rewards beside another's text.
    return questAtRow(gh, gh->getSelectedQuestLogIndex());
}

// GetQuestLogRewardSpell() → texture, name, isTradeskillSpell, isSpellLearned
//
// Nil for every quest, and questinfo.lua gates the whole reward-spell row on
// `if ( GetQuestLogRewardSpell() )` - so a quest that teaches a recipe or hands
// out a buff showed no sign of it, and the tooltip setter beside it had nothing
// to describe. The spell is in SMSG_QUEST_QUERY_RESPONSE at the field between
// two the reward parser already reads.
//
// The three flags decide only which line is printed under the icon:
// REWARD_TRADESKILL_SPELL for a recipe, REWARD_AURA for something not yet
// known, REWARD_SPELL otherwise.
static int lua_GetQuestLogRewardSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return luaReturnNil(L);
    const auto* q = selectedLogEntry(gh);
    const uint32_t spellId = q ? q->rewardSpellId : 0u;
    if (spellId == 0) return luaReturnNil(L);
    const std::string name = gh->getSpellName(spellId);
    if (name.empty()) return luaReturnNil(L);
    std::string icon = gh->getSpellIconPath(spellId);
    lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                   : icon.c_str());
    lua_pushstring(L, name.c_str());
    lua_pushboolean(L, gh->isProfessionSpell(spellId) ? 1 : 0);
    lua_pushboolean(L, gh->getKnownSpells().count(spellId) > 0 ? 1 : 0);
    return 4;
}

/// One reward, described the way every reward button reads it.
template <typename Container>
static int pushRewardAt(lua_State* L, game::GameHandler* gh,
                        const Container& list, int index) {
    if (!gh || index < 1) { return luaReturnNil(L); }
    int seen = 0;
    for (const auto& r : list) {
        if (r.itemId == 0) continue;
        if (++seen != index) continue;
        const auto* info = gh->getItemInfo(r.itemId);
        lua_pushstring(L, info ? info->name.c_str() : "");
        lua_pushstring(L, gh->getItemIconPath(
            info && info->displayInfoId ? info->displayInfoId : r.displayInfoId).c_str());
        lua_pushnumber(L, r.count);
        lua_pushnumber(L, info ? info->quality : 1);
        lua_pushboolean(L, 1);
        return 5;
    }
    return luaReturnNil(L);
}

// GetSuggestedGroupNum() / GetQuestLogGroupNum() → how many players a quest
// suggests bringing
//
// QuestInfo assigns one of these and then tests `groupNum > 0`, so an absent
// answer is an error rather than a quest that suggests nothing - and QuestInfo
// draws for the quest giver and the quest log both.
//
// The giver's packet carries the number. The log's does not, and zero is what
// stops the line being drawn at all.
// GetDailyQuestsCompleted() / GetMaxDailyQuests() → the daily allowance
//
// The quest log tests the first against zero before drawing the line at all, so
// an absent answer is an error rather than a hidden line. Nothing here counts
// dailies, and zero completed is what keeps the line hidden - which is the same
// thing the count would do if it were counting and found none.
static int lua_GetDailyQuestsCompleted(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

// Twenty-five is the allowance in this era. Only ever shown beside the count
// above, which stays at zero, so it is a label rather than a limit here.
static int lua_GetMaxDailyQuests(lua_State* L) {
    lua_pushnumber(L, 25);
    return 1;
}

static int lua_GetSuggestedGroupNum(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool open = gh && gh->isQuestDetailsOpen();
    lua_pushnumber(L, open ? gh->getQuestDetails().suggestedPlayers : 0);
    return 1;
}

static int lua_GetQuestLogGroupNum(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

static int lua_GetNumQuestLogRewards(lua_State* L) {
    const auto* q = selectedLogEntry(getGameHandler(L));
    lua_pushnumber(L, q ? countRewards(&q->rewardItems) : 0);
    return 1;
}

static int lua_GetNumQuestLogChoices(lua_State* L) {
    const auto* q = selectedLogEntry(getGameHandler(L));
    lua_pushnumber(L, q ? countRewards(&q->rewardChoiceItems) : 0);
    return 1;
}

static int lua_GetQuestLogRewardInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* q = selectedLogEntry(gh);
    if (!q) { return luaReturnNil(L); }
    return pushRewardAt(L, gh, q->rewardItems,
                        static_cast<int>(luaL_optnumber(L, 1, 0)));
}

static int lua_GetQuestLogChoiceInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* q = selectedLogEntry(gh);
    if (!q) { return luaReturnNil(L); }
    return pushRewardAt(L, gh, q->rewardChoiceItems,
                        static_cast<int>(luaL_optnumber(L, 1, 0)));
}

// GetQuestLogItemLink(type, index) → a reward of the selected quest, as a link.
//
// The quest log's own version of GetQuestItemLink, over the same two lists
// GetQuestLogRewardInfo and GetQuestLogChoiceInfo read. Shift-clicking a
// reward in the log put nothing in chat before.
static int lua_GetQuestLogItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* q = selectedLogEntry(gh);
    const char* type = luaL_optstring(L, 1, "reward");
    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (!gh || !q || index < 1) { return luaReturnNil(L); }

    // The two lists are fixed arrays of different lengths, so they are walked
    // by a template rather than bound to one reference.
    const auto linkAt = [&](const auto& list) -> int {
        int seen = 0;
        for (const auto& r : list) {
            if (r.itemId == 0) continue;
            if (++seen != index) continue;
            const auto* info = gh->getItemInfo(r.itemId);
            const std::string name = info ? info->name : "";
            if (name.empty()) break;
            lua_pushstring(L, game::buildItemLink(r.itemId,
                                                  info ? info->quality : 1u,
                                                  name).c_str());
            return 1;
        }
        return luaReturnNil(L);
    };
    return (std::string(type) == "choice") ? linkAt(q->rewardChoiceItems)
                                           : linkAt(q->rewardItems);
}

// Nil for the same reason GetQuestSpellLink is: nothing this client parses
// says a quest gives a spell, so there is no spell to name.
static int lua_GetQuestLogSpellLink(lua_State* L) {
    return luaReturnNil(L);
}

// The server sends one field for both, and its sign says which: AzerothCore's
// GetRewOrReqMoney returns the reward when it is positive and the *cost* when
// it is negative, and the quest query serializer writes that single int32. So a
// quest that charges the player was answering this with a negative reward, and
// the reward panel drew a money frame with a negative amount in it.
static int lua_GetQuestLogRewardMoney(lua_State* L) {
    const auto* q = selectedLogEntry(getGameHandler(L));
    const int32_t money = q ? q->rewardMoney : 0;
    lua_pushnumber(L, money > 0 ? money : 0);
    return 1;
}

// ...and the other half of that field, which answered zero for every quest.
//
// Named with an index by the world map and without one by the quest info
// panel, which is how most of the quest log accessors are asked: the selected
// entry is the default, not the only answer.
static int lua_GetQuestLogRequiredMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    const game::GameHandler::QuestLogEntry* q = nullptr;
    if (gh && lua_isnumber(L, 1)) {
        q = questAtRow(gh, static_cast<int>(lua_tonumber(L, 1)));
    } else {
        q = selectedLogEntry(gh);
    }
    const int32_t money = q ? q->rewardMoney : 0;
    lua_pushnumber(L, money < 0 ? -money : 0);
    return 1;
}

// --- The quest giver's own dialog ---
//
// WoW's quest-giver functions do not name which quest they mean: GetTitleText
// and the reward accessors serve the dialog and the quest log both, and answer
// for whichever is in front. A dialog is in front while it is open - the
// reward panel over the progress panel over the offer, in the order the server
// walks a player through them - and the log's selection answers otherwise.

namespace {

struct QuestSource {
    const std::string* title = nullptr;
    const std::vector<game::QuestRewardItem>* rewards = nullptr;
    const std::vector<game::QuestRewardItem>* choices = nullptr;
    uint32_t money = 0;
    uint32_t xp = 0;
    uint32_t honor = 0;
    uint32_t talents = 0;
    uint32_t arenaPoints = 0;
};

QuestSource currentQuestSource(game::GameHandler* gh) {
    QuestSource s;
    if (!gh) return s;
    if (gh->isQuestOfferRewardOpen()) {
        const auto& d = gh->getQuestOfferReward();
        s = {&d.title, &d.fixedRewards, &d.choiceRewards, d.rewardMoney, d.rewardXp,
             d.rewardHonor, d.rewardTalents, d.rewardArenaPoints};
    } else if (gh->isQuestRequestItemsOpen()) {
        const auto& d = gh->getQuestRequestItems();
        s.title = &d.title;
    } else if (gh->isQuestDetailsOpen()) {
        const auto& d = gh->getQuestDetails();
        s = {&d.title, &d.rewardItems, &d.rewardChoiceItems, d.rewardMoney, d.rewardXp};
    }
    return s;
}

/// Counts only the filled slots: the server pads a reward list to a fixed
/// width, and a row drawn for an empty one is a blank button the player can
/// click.

int pushQuestText(lua_State* L, const std::string* s) {
    if (!s || s->empty()) { lua_pushstring(L, ""); return 1; }
    // The binding is the first point that has the player to resolve against;
    // the parser only ever turned $b into a line break. See forPlayer.
    lua_pushstring(L, forPlayer(L, *s).c_str());
    return 1;
}

}  // namespace

// GetTitleText() → the quest's name, whichever panel is showing it
static int lua_GetTitleText(lua_State* L) {
    return pushQuestText(L, currentQuestSource(getGameHandler(L)).title);
}

// GetQuestText() → what the quest giver says when offering it
static int lua_GetQuestText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isQuestDetailsOpen()) { lua_pushstring(L, ""); return 1; }
    return pushQuestText(L, &gh->getQuestDetails().details);
}

// GetObjectiveText() → what the quest asks for
static int lua_GetObjectiveText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isQuestDetailsOpen()) { lua_pushstring(L, ""); return 1; }
    return pushQuestText(L, &gh->getQuestDetails().objectives);
}

// GetProgressText() → what is said while the quest is still unfinished
static int lua_GetProgressText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isQuestRequestItemsOpen()) { lua_pushstring(L, ""); return 1; }
    return pushQuestText(L, &gh->getQuestRequestItems().completionText);
}

// GetRewardText() → what is said on handing it in
static int lua_GetRewardText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isQuestOfferRewardOpen()) { lua_pushstring(L, ""); return 1; }
    return pushQuestText(L, &gh->getQuestOfferReward().rewardText);
}

// GetGossipText() → the body of the gossip window
static int lua_GetGossipText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isGossipWindowOpen()) { lua_pushstring(L, ""); return 1; }
    return pushQuestText(L, &gh->getNpcText(gh->getCurrentGossip().titleTextId));
}

// GetRewardMoney() → coin the quest pays out
static int lua_GetRewardMoney(lua_State* L) {
    lua_pushnumber(L, currentQuestSource(getGameHandler(L)).money);
    return 1;
}

// GetRewardXP() → experience the quest pays out
static int lua_GetRewardXP(lua_State* L) {
    lua_pushnumber(L, currentQuestSource(getGameHandler(L)).xp);
    return 1;
}
// The offer panel's honour, talents and arena points - the quest-giver twins of
// GetQuestLogReward*, read from the offer packet (which carries all three after
// the money and XP) rather than the query. Were hard zero.
static int lua_GetRewardHonor(lua_State* L) {
    lua_pushnumber(L, currentQuestSource(getGameHandler(L)).honor);
    return 1;
}
static int lua_GetRewardTalents(lua_State* L) {
    lua_pushnumber(L, currentQuestSource(getGameHandler(L)).talents);
    return 1;
}
static int lua_GetRewardArenaPoints(lua_State* L) {
    lua_pushnumber(L, currentQuestSource(getGameHandler(L)).arenaPoints);
    return 1;
}

// GetQuestMoneyToGet() → coin the player has to *hand over*, which is the
// opposite of the reward and a different field entirely. Some quests ask for
// money; showing the reward here would tell the player they are being paid
// when they are being charged.
static int lua_GetQuestMoneyToGet(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isQuestRequestItemsOpen()) { lua_pushnumber(L, 0); return 1; }
    lua_pushnumber(L, gh->getQuestRequestItems().requiredMoney);
    return 1;
}

// IsQuestCompletable() → whether the turn-in button should work
static int lua_IsQuestCompletable(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool ok = gh && gh->isQuestRequestItemsOpen() &&
                    gh->getQuestRequestItems().isCompletable();
    lua_pushboolean(L, ok ? 1 : 0);
    return 1;
}

// GetQuestReward(choiceIndex) → takes the reward and closes the dialog
static int lua_GetQuestReward(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    // One-based, and zero means the quest offered nothing to choose between.
    const int choice = static_cast<int>(luaL_optnumber(L, 1, 0));
    gh->chooseQuestReward(choice > 0 ? static_cast<uint32_t>(choice - 1) : 0);
    return 0;
}

// CloseQuest() → puts the dialog away without answering it
static int lua_CloseQuest(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    // Without announcing. This is QuestFrame telling the client it has closed
    // - it runs from QuestFrame_OnHide - and answering with QUEST_FINISHED,
    // which is the event that hides QuestFrame, closes it a second time from
    // inside its own closing.
    if (gh->isQuestOfferRewardOpen())       gh->closeQuestOfferReward(false);
    else if (gh->isQuestRequestItemsOpen()) gh->closeQuestRequestItems(false);
    else                                    gh->declineQuest(false);
    return 0;
}

// GetQuestItemInfo(type, index) → name, texture, count, quality, isUsable
//
// type is "choice" for the rewards a player picks between and "reward" for the
// ones always given.
static int lua_GetQuestItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* type = luaL_optstring(L, 1, "reward");
    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    const QuestSource s = currentQuestSource(gh);

    // Three lists, not two. "required" is what the *progress* page asks for -
    // the items a quest wants handed in - and it was falling through to the
    // reward list, so turning in a quest showed what it would pay rather than
    // what it wanted.
    const std::string want(type);
    const std::vector<game::QuestRewardItem>* list = nullptr;
    if (want == "choice")        list = s.choices;
    else if (want == "required") list = gh ? &gh->getQuestRequestItems().requiredItems : nullptr;
    else                         list = s.rewards;
    if (!gh || !list || index < 1) { return luaReturnNil(L); }

    int seen = 0;
    for (const auto& r : *list) {
        if (r.itemId == 0) continue;
        if (++seen != index) continue;
        const auto* info = gh->getItemInfo(r.itemId);
        lua_pushstring(L, info ? info->name.c_str() : "");
        lua_pushstring(L, gh->getItemIconPath(
            info && info->displayInfoId ? info->displayInfoId : r.displayInfoId).c_str());
        lua_pushnumber(L, r.count);
        lua_pushnumber(L, info ? info->quality : 1);
        lua_pushboolean(L, 1);
        return 5;
    }
    return luaReturnNil(L);
}

// GetQuestItemLink(type, index) → the reward as a link, for shift-clicking it
// into chat.
//
// The same two lists GetQuestItemInfo walks, answered as a link rather than as
// pieces. Nil when there is no such reward, which is what the click handler
// checks before doing anything with it.
static int lua_GetQuestItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* type = luaL_optstring(L, 1, "reward");
    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    const QuestSource s = currentQuestSource(gh);

    const auto* list = (std::string(type) == "choice") ? s.choices : s.rewards;
    if (!gh || !list || index < 1) { return luaReturnNil(L); }

    int seen = 0;
    for (const auto& r : *list) {
        if (r.itemId == 0) continue;
        if (++seen != index) continue;
        const auto* info = gh->getItemInfo(r.itemId);
        const std::string name = info ? info->name : "";
        if (name.empty()) { return luaReturnNil(L); }
        lua_pushstring(L, game::buildItemLink(r.itemId,
                                              info ? info->quality : 1u,
                                              name).c_str());
        return 1;
    }
    return luaReturnNil(L);
}

// GetQuestSpellLink(...) - the spell a quest gives, as a link.
//
// Nil, and deliberately: no quest packet this client parses carries a reward
// spell, so there is nothing to name. The click handler passes whatever it
// gets to HandleModifiedItemClick, which does nothing with nil - where a
// made-up link would put a spell in someone's chat that the quest does not
// give.
static int lua_GetQuestSpellLink(lua_State* L) {
    return luaReturnNil(L);
}

void registerQuestLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"GetNumQuestLogEntries",   lua_GetNumQuestLogEntries},
                // Whether a quest has been finished, ever.
                //
                // The client asks the server for this list on entering the
                // world - CMSG_QUERY_QUESTS_COMPLETED - parses the reply into
                // completedQuests_ and keeps it there. Nothing could read it:
                // the two names that ask, which every quest addon uses and
                // which is how a quest giver knows to grey an offer out, were
                // not bound at all.
                {"IsQuestFlaggedCompleted", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            lua_pushboolean(L, (gh && id && gh->isQuestCompleted(id)) ? 1 : 0);
            return 1;
        }},
                // GetQuestsCompleted([table]) - the whole set at once, keyed by
                // quest id, which is the shape the caller indexes. Fills the
                // table it is given, as WoW does, so a caller reusing one does
                // not allocate per call.
                {"GetQuestsCompleted", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!lua_istable(L, 1)) lua_newtable(L);
            else                    lua_pushvalue(L, 1);
            if (gh) {
                const auto& done = gh->getCompletedQuests();
                // Lua promises twenty free slots; this pushes two at a time and
                // pops them, so the depth never grows with the set's size.
                for (uint32_t id : done) {
                    lua_pushnumber(L, static_cast<lua_Number>(id));
                    lua_pushboolean(L, 1);
                    lua_rawset(L, -3);
                }
            }
            return 1;
        }},
                {"GetQuestLogTimeLeft",     lua_GetQuestLogTimeLeft},
                {"IsCurrentQuestFailed",    lua_IsCurrentQuestFailed},
                {"GetQuestLogRewardSpell",  lua_GetQuestLogRewardSpell},
                {"GetRewardSpell",          lua_GetQuestRewardSpell},
                {"GetRewardTitle",          lua_GetQuestRewardTitle},
                {"GetQuestLogRewardTitle",  lua_GetQuestRewardTitle},
                {"ProcessQuestLogRewardFactions", lua_ProcessQuestLogRewardFactions},
                {"GetNumQuestLogRewardFactions",  lua_GetNumQuestLogRewardFactions},
                {"GetQuestLogRewardFactionInfo",  lua_GetQuestLogRewardFactionInfo},
                {"GetFactionInfoByID",      lua_GetFactionInfoByID},
                {"GetQuestWatchIndex",      lua_GetQuestWatchIndex},
                {"SortQuestWatches",        lua_SortQuestWatches},
                {"ShiftQuestWatches",       lua_ShiftQuestWatches},
                {"GetQuestSortIndex",       lua_GetQuestSortIndex},
                {"IsQuestLogSpecialItemInRange", lua_IsQuestLogSpecialItemInRange},
                {"UseQuestLogSpecialItem",  lua_UseQuestLogSpecialItem},
                {"GetQuestLogSpecialItemCooldown", lua_GetQuestLogSpecialItemCooldown},
                {"GetQuestTimers",          lua_GetQuestTimers},
                {"GetQuestIndexForTimer",   lua_GetQuestIndexForTimer},
                {"GetQuestLogTitle",        lua_GetQuestLogTitle},
                // IsUnitOnQuest(questIndex, unit) - whether that unit is also
                // on the quest, which the log prints as "[2]" beside an entry
                // to say how many group mates share it.
                //
                // Only answerable for the player: the server does not tell a
                // client what its party members' quest logs hold, and nothing
                // here tracks them. False for everyone else is the truthful
                // answer and is what leaves the counter hidden, rather than
                // claiming a number nobody can stand behind.
                {"IsUnitOnQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            const char* uid = luaL_optstring(L, 2, "player");
            std::string u(uid);
            for (char& c : u) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (!gh || u != "player" || index < 1) { lua_pushboolean(L, 0); return 1; }
            lua_pushboolean(L, questAtRow(gh, index) != nullptr ? 1 : 0);
            return 1;
        }},
                {"GetQuestLogQuestText",    lua_GetQuestLogQuestText},
                // GetQuestLogCompletionText(index) → what to do now it is done.
                //
                // An empty string rather than nothing when there is none,
                // because the world map hovers a finished quest's pin and
                // writes AddLine("- "..GetQuestLogCompletionText(i)) without
                // checking. Concatenating nil raises, so the pin would have
                // taken the map down; an empty string draws a bare dash.
                //
                // The world map's quest list puts this straight into the row's
                // objectives line for any quest that is complete, so answering
                // "" left the row blank where "Return to Marshal Dughan"
                // belongs. It is the fifth string in SMSG_QUEST_QUERY_RESPONSE
                // and was being walked past.
                {"GetQuestLogCompletionText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const game::GameHandler::QuestLogEntry* q = nullptr;
            if (gh && lua_isnumber(L, 1)) {
                q = questAtRow(gh, static_cast<int>(lua_tonumber(L, 1)));
            } else {
                q = selectedLogEntry(gh);
            }
            lua_pushstring(L, q ? q->completionText.c_str() : "");
            return 1;
        }},
                // The words on a book or a plaque, which this client parses out
                // of SMSG_ITEM_TEXT_QUERY_RESPONSE and kept to itself. This
                // answered with an empty string on the reasoning that nothing
                // opened the window anyway; ITEM_TEXT_READY is fired now, so it
                // does open, and empty is the difference between a book and a
                // blank page.
                //
                // Still a string and never nil: the page is drawn as
                // "\n"..ItemTextGetText()..creator, and a nil there takes the
                // whole window down rather than leaving it empty.
                {"ItemTextGetText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushstring(L, ""); return 1; }
            // Two things arrive on this frame and they come by different
            // opcodes: a letter's body, which is one page, and a book, which
            // chains through nextPageId and is collected whole before anyone
            // reads it. A book wins while one is open, because a letter cannot
            // be open at the same time.
            const auto& pages = gh->getBookPages();
            if (!pages.empty()) {
                const size_t page = static_cast<size_t>(bookPage());
                lua_pushstring(L, page < pages.size() ? pages[page].text.c_str()
                                                      : pages.back().text.c_str());
                return 1;
            }
            lua_pushstring(L, gh->getItemText().c_str());
            return 1;
        }},
                // ItemTextGetItem() - the heading over the page. The page
                // text response carries the words and the id of the page
                // after it and nothing about what they belong to, so this is
                // kept from wherever the book was opened: the game object's
                // name out of its query cache, or the item's out of the bag.
                // It used to answer nil and the frame headed every book, sign
                // and plaque with nothing.
                {"ItemTextGetItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const std::string title = gh ? gh->getBookTitle() : std::string();
            if (title.empty()) return luaReturnNil(L);
            lua_pushstring(L, title.c_str());
            return 1;
        }},
                // A page really has no author on this wire -
                // SMSG_ITEM_TEXT_QUERY_RESPONSE is an id and the words - and
                // the frame drops the "from" line on a nil, which is honest
                // where an invented name would not be.
                {"ItemTextGetCreator",  [](lua_State* L) -> int { return luaReturnNil(L); }},
                // The material is carried, and the claim that it was not stood
                // beside the line discarding it: the item query's four
                // post-description words are PageText, LanguageID,
                // PageMaterial and StartQuest, and all three parsers read the
                // third into nothing. A game object's is in its template two
                // words behind the page id.
                //
                // It names one of PageTextMaterial.dbc's seven backings, which
                // is what itemtextframe wants - it picks both the frame's
                // texture and the text colour by that word, so every letter
                // and tablet was drawn on parchment.
                {"ItemTextGetMaterial", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return luaReturnNil(L);
            const std::string& name = gh->getPageTextMaterialName(gh->getBookMaterial());
            // Nil rather than an empty string for an unknown id: the frame
            // tests `if ( not material )` and substitutes Parchment itself.
            if (name.empty()) return luaReturnNil(L);
            lua_pushstring(L, name.c_str());
            return 1;
        }},
                // A letter is one page and its buttons stay hidden. A book is
                // as many as its chain has, all of them already fetched, so
                // turning one is a move through what is in hand rather than a
                // request - and the frame redraws on ITEM_TEXT_READY, which is
                // what makes the move visible.
                {"ItemTextGetPage", [](lua_State* L) -> int {
            lua_pushnumber(L, bookPage() + 1);
            return 1;
        }},
                {"ItemTextHasNextPage", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const size_t pages = gh ? gh->getBookPages().size() : 0;
            lua_pushboolean(L, static_cast<size_t>(bookPage() + 1) < pages ? 1 : 0);
            return 1;
        }},
                {"ItemTextPrevPage", [](lua_State* L) -> int {
            if (bookPage() > 0) { --bookPage(); fireItemTextReady(L); }
            return 0;
        }},
                {"ItemTextNextPage", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const size_t pages = gh ? gh->getBookPages().size() : 0;
            if (static_cast<size_t>(bookPage() + 1) < pages) {
                ++bookPage();
                fireItemTextReady(L);
            }
            return 0;
        }},
                // Closing is a state change this client owns, and the frame
                // calls it on hide as well as from its close button.
                {"CloseItemText", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                gh->closeItemText();
                // The pages go with the book, and so does the place in it -
                // left behind, the next book opens at whatever page the last
                // one was left on and shows nothing until it is paged back.
                gh->clearBook();
            }
            bookPage() = 0;
            return 0;
        }},
                {"IsQuestComplete",         lua_IsQuestComplete},
                {"SelectQuestLogEntry",     lua_SelectQuestLogEntry},
                {"GetQuestLogSelection",    lua_GetQuestLogSelection},
                {"GetQuestLogPushable",     lua_GetQuestLogPushable},
                {"QuestLogPushQuest",       lua_QuestLogPushQuest},
                {"GetNumQuestWatches",      lua_GetNumQuestWatches},
                {"GetQuestIndexForWatch",   lua_GetQuestIndexForWatch},
                {"AddQuestWatch",           lua_AddQuestWatch},
                {"RemoveQuestWatch",        lua_RemoveQuestWatch},
                {"IsQuestWatched",          lua_IsQuestWatched},
                {"GetQuestLink",            lua_GetQuestLink},
                {"GetNumQuestLeaderBoards", lua_GetNumQuestLeaderBoards},
                {"GetQuestLogLeaderBoard",  lua_GetQuestLogLeaderBoard},
                {"QuestMapUpdateAllQuests", lua_QuestMapUpdateAllQuests},
                {"QuestPOIGetQuestIDByVisibleIndex", lua_QuestPOIGetQuestIDByVisibleIndex},
                {"QuestPOIGetIconInfo",     lua_QuestPOIGetIconInfo},
                {"QuestPOIUpdateIcons",     lua_QuestPOIUpdateIcons},
                {"GetQuestPOILeaderBoard",  lua_GetQuestPOILeaderBoard},
                {"ExpandQuestHeader",       lua_ExpandQuestHeader},
                {"CollapseQuestHeader",     lua_CollapseQuestHeader},
                {"GetQuestLogSpecialItemInfo", lua_GetQuestLogSpecialItemInfo},
                {"GetNumSkillLines",        lua_GetNumSkillLines},
                {"GetStablePetInfo",       lua_GetStablePetInfo},
                {"GetNumStablePets",       lua_GetNumStablePets},
                {"GetNumStableSlots",      lua_GetNumStableSlots},
                {"GetSelectedStablePet",   lua_GetSelectedStablePet},
                {"ClickStablePet",         lua_ClickStablePet},
                {"GetPetIcon",             lua_GetPetIcon},
                {"GetPetTalentTree",       lua_GetPetTalentTree},
                {"GetPetFoodTypes",        lua_GetPetFoodTypes},
                {"GetStablePetFoodTypes",  lua_GetStablePetFoodTypes},
                {"SetPetStablePaperdoll",  lua_SetPetStablePaperdoll},
                {"PickupStablePet",        lua_PickupStablePet},
                {"ClosePetStables",        lua_ClosePetStables},
                {"IsAtStableMaster",       lua_IsAtStableMaster},
                {"GetNextStableSlotCost",  lua_GetNextStableSlotCost},
                {"GetSkillLineInfo",        lua_GetSkillLineInfo},
                // Opening and closing a heading. The index is a position in
                // the drawn rows, and a click on a skill row rather than a
                // heading is ignored rather than refused - the tab calls these
                // from the row label template whatever the row turns out to be.
                {"ExpandSkillHeader",   [](lua_State* L) -> int {
            return skillHeaderSetCollapsed(L, false);
        }},
                {"CollapseSkillHeader", [](lua_State* L) -> int {
            return skillHeaderSetCollapsed(L, true);
        }},
                {"GetNumTalentTabs",        lua_GetNumTalentTabs},
                {"GetTalentTabInfo",        lua_GetTalentTabInfo},
                {"GetNumTalents",           lua_GetNumTalents},
                {"GetTalentInfo",           lua_GetTalentInfo},
                {"GetTalentPrereqs",        lua_GetTalentPrereqs},
                {"AddPreviewTalentPoints",  lua_AddPreviewTalentPoints},
                {"GetGroupPreviewTalentPointsSpent", lua_GetGroupPreviewTalentPointsSpent},
                {"ResetGroupPreviewTalentPoints",    lua_ResetGroupPreviewTalentPoints},
                {"LearnPreviewTalents",     lua_LearnPreviewTalents},
                {"LearnTalent",             lua_LearnTalent},
                {"GetUnspentTalentPoints",  lua_GetUnspentTalentPoints},
                {"GetNumTalentGroups",      lua_GetNumTalentGroups},
                {"SetActiveTalentGroup",    lua_SetActiveTalentGroup},
                {"GetTalentLink",           lua_GetTalentLink},
                {"GetActiveTalentGroup",    lua_GetActiveTalentGroup},
                {"AcceptQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->acceptQuest();
            return 0;
        }},
                {"DeclineQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // Announcing: the button was pressed, and the frame has not hidden
            // itself yet. QUEST_FINISHED is what hides it.
            if (gh) gh->declineQuest(true);
            return 0;
        }},
                // The other party member's quest, not the one on the table.
                // StaticPopup "QUEST_ACCEPT" calls this from OnAccept, so it
                // has to exist before QUEST_ACCEPT_CONFIRM is fired at all:
                // the popup would come up and raise on the yes.
                {"ConfirmAcceptQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->acceptSharedQuest();
            return 0;
        }},
                {"CompleteQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->completeQuest();
            return 0;
        }},
                // Abandoning is two steps, not one: the quest log marks which
                // quest it means, a confirmation shows its name, and only then
                // is it abandoned. AbandonQuest therefore takes no argument in
                // the interface, and requiring one raised a Lua error on every
                // attempt - the id is accepted when given so anything already
                // passing one keeps working.
                {"SetAbandonQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto* q = selectedLogEntry(gh);
            pendingAbandonQuest() = q ? q->questId : 0;
            return 0;
        }},
                {"GetAbandonQuestName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = pendingAbandonQuest();
            if (gh && id != 0) {
                for (const auto& q : gh->getQuestLog()) {
                    if (q.questId == id) {
                        lua_pushstring(L, forPlayer(L, q.title).c_str());
                        return 1;
                    }
                }
            }
            lua_pushstring(L, "");
            return 1;
        }},
                // What is destroyed along with the quest. Nothing here knows
                // which items a quest would take back, and an invented list
                // would warn about items the player keeps.
                // What abandoning the quest destroys, or nil for nothing -
                // and nil is the load-bearing part. Both callers do
                // `if ( items )` to choose between two popups, and an empty
                // string is true in Lua, so answering "" always took the
                // ABANDON_QUEST_WITH_ITEMS branch and showed "you will lose:"
                // with nothing after the colon.
                //
                // The quest's source item is the one the server takes back, and
                // this client has stored it since the reward parser learned to
                // read it - it even queries the item info for it on arrival.
                // Items gathered for the quest are not named: which of those
                // the server destroys depends on flags this client does not
                // parse, and listing the wrong ones is worse than listing none.
                {"GetAbandonQuestItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = pendingAbandonQuest();
            if (!gh || id == 0) { lua_pushnil(L); return 1; }
            for (const auto& q : gh->getQuestLog()) {
                if (q.questId != id) continue;
                if (q.sourceItemId == 0) break;
                const auto* info = gh->getItemInfo(q.sourceItemId);
                if (!info || !info->valid || info->name.empty()) break;
                lua_pushstring(L, info->name.c_str());
                return 1;
            }
            lua_pushnil(L);
            return 1;
        }},
                {"AbandonQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const uint32_t questId = lua_isnoneornil(L, 1)
                ? pendingAbandonQuest()
                : static_cast<uint32_t>(luaL_checknumber(L, 1));
            if (questId != 0) gh->abandonQuest(questId);
            pendingAbandonQuest() = 0;
            return 0;
        }},
                // Both of these answer for the quest giver's dialog while one
                // is open and for the log's selection otherwise, which is the
                // same rule the text accessors follow.
                {"GetNumQuestRewards", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            if (const auto* v = currentQuestSource(gh).rewards) {
                lua_pushnumber(L, countRewards(v));
                return 1;
            }
            int idx = gh->getSelectedQuestLogIndex();
            if (idx < 1) { return luaReturnZero(L); }
            const auto& ql = gh->getQuestLog();
            if (idx > static_cast<int>(ql.size())) { return luaReturnZero(L); }
            lua_pushnumber(L, countRewards(&ql[idx-1].rewardItems));
            return 1;
        }},
                {"GetNumQuestChoices", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            if (const auto* v = currentQuestSource(gh).choices) {
                lua_pushnumber(L, countRewards(v));
                return 1;
            }
            int idx = gh->getSelectedQuestLogIndex();
            if (idx < 1) { return luaReturnZero(L); }
            const auto& ql = gh->getQuestLog();
            if (idx > static_cast<int>(ql.size())) { return luaReturnZero(L); }
            lua_pushnumber(L, countRewards(&ql[idx-1].rewardChoiceItems));
            return 1;
        }},
                {"GetSuggestedGroupNum", lua_GetSuggestedGroupNum},
                {"GetDailyQuestsCompleted", lua_GetDailyQuestsCompleted},
                {"GetMaxDailyQuests",    lua_GetMaxDailyQuests},
                {"GetQuestLogGroupNum",  lua_GetQuestLogGroupNum},
                {"GetNumQuestLogRewards", lua_GetNumQuestLogRewards},
                {"GetNumQuestLogChoices", lua_GetNumQuestLogChoices},
                {"GetQuestLogRewardInfo", lua_GetQuestLogRewardInfo},
                {"GetQuestLogChoiceInfo", lua_GetQuestLogChoiceInfo},
                {"GetQuestLogItemLink",     lua_GetQuestLogItemLink},
                {"GetQuestLogSpellLink",    lua_GetQuestLogSpellLink},
                {"GetQuestLogRewardMoney", lua_GetQuestLogRewardMoney},
                {"GetQuestLogRequiredMoney", lua_GetQuestLogRequiredMoney},
                {"GetTitleText",         lua_GetTitleText},
                {"GetQuestText",         lua_GetQuestText},
                {"GetObjectiveText",     lua_GetObjectiveText},
                {"GetProgressText",      lua_GetProgressText},
                {"GetRewardText",        lua_GetRewardText},
                {"GetGossipText",        lua_GetGossipText},
                {"GetQuestItemInfo",     lua_GetQuestItemInfo},
                // How many items a quest wants handed in.
                // QuestFrameProgressItems_Update reads it straight into
                // `numRequiredItems > 0`, and comparing nil against a number
                // raises - so opening the progress page of any quest that
                // takes items took the page down.
                {"GetNumQuestItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto& req = gh->getQuestRequestItems().requiredItems;
            int n = 0;
            for (const auto& r : req) if (r.itemId != 0) ++n;
            lua_pushnumber(L, n);
            return 1;
        }},
                // The parchment behind the quest text, and nil is right for a
                // different reason than the one that used to be written here.
                //
                // That reason was "the only material this client has art for",
                // and it is not: Data/interface/itemtextframe carries bronze,
                // marble, silver, stone and valentine, and QuestFrame_GetMaterial
                // builds exactly those names - Interface\ItemTextFrame\ItemText-
                // <material>-TopLeft and its three corners. Parchment has no
                // files because it is the frame's own art. The same set backs
                // ItemTextGetMaterial, which does answer.
                //
                // What is missing is a source, not a texture. A book's material
                // is on the item or the game object that opened it;
                // SMSG_QUESTGIVER_QUEST_DETAILS carries no such field, and the
                // quest giver's own is not the quest's. Answering from the
                // giver would be a guess dressed as data, and the caller's
                // substitution of Parchment is what nearly every quest wants.
                {"GetQuestBackgroundMaterial", [](lua_State* L) -> int { return luaReturnNil(L); }},
                // Whether the quest is flagged PvP, and whether the giver
                // opened it without being asked. Neither is parsed from the
                // quest packets here, and false is what the interface does
                // with an absent answer anyway.
                // QuestFlagsPVP() - whether holding this quest forces the PvP
                // flag on. The accept button branches on it and puts up
                // CONFIRM_ACCEPT_PVP_QUEST instead of accepting outright, so a
                // flat false took a quest that flags you for PvP without
                // asking. QUEST_FLAGS_FLAGS_PVP is 0x2000 in AzerothCore's
                // QuestDef.h; the offer packet carried it and the parser threw
                // it away. Zero before Wrath, where the offer has no flags
                // field for it to be in.
                {"QuestFlagsPVP", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            constexpr uint32_t kQuestFlagsPvp = 0x00002000u;
            lua_pushboolean(L, gh && (gh->getQuestDetails().questFlags & kQuestFlagsPvp) ? 1 : 0);
            return 1;
        }},
                // QuestGetAutoAccept() stays false, and that is the answer
                // rather than a gap: QUEST_FLAGS_AUTO_ACCEPT is 0x80000 and
                // AzerothCore's own note beside it says no 3.3.5a quest
                // carries it. The flag is parsed now, so if one ever does this
                // can read it from the same place.
                {"QuestGetAutoAccept", [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                // Shown when a reward is confirmed without one being picked.
                // The message belongs to the server in the real client; there
                // is nothing to say here, and the call is made for its effect
                // rather than its answer.
                {"QuestChooseRewardError", [](lua_State* L) -> int { (void)L; return 0; }},
                {"GetQuestItemLink",        lua_GetQuestItemLink},
                {"GetQuestSpellLink",       lua_GetQuestSpellLink},
                {"GetQuestMoneyToGet",   lua_GetQuestMoneyToGet},
                {"GetRewardMoney",       lua_GetRewardMoney},
                {"GetRewardXP",          lua_GetRewardXP},
                {"GetRewardHonor",              lua_GetRewardHonor},
                {"GetRewardArenaPoints",        lua_GetRewardArenaPoints},
                {"GetRewardTalents",            lua_GetRewardTalents},
                {"GetQuestLogRewardHonor",       lua_GetQuestLogRewardHonor},
                {"GetQuestLogRewardArenaPoints", lua_GetQuestLogRewardArenaPoints},
                {"GetQuestLogRewardTalents",     lua_GetQuestLogRewardTalents},
                {"GetQuestLogRewardXP",          lua_GetQuestLogRewardXP},
                {"IsQuestCompletable",   lua_IsQuestCompletable},
                {"GetQuestReward",       lua_GetQuestReward},
                {"CloseQuest",           lua_CloseQuest},
                // RemoveGlyphFromSocket(socket) - the Accept on the "remove
                // this glyph?" popup, which is the only way to clear a socket.
                // Unbound, the dialog appeared and answering it raised, so a
                // glyph could be put in and never taken out.
                //
                // FrameXML counts sockets from one and the server from zero.
                {"RemoveGlyphFromSocket", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int socket = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (gh && socket >= 1 && socket <= game::GameHandler::MAX_GLYPH_SLOTS)
                gh->removeGlyphFromSocket(static_cast<uint32_t>(socket - 1));
            return 0;
        }},
                {"GetNumGlyphSockets", [](lua_State* L) -> int {
            lua_pushnumber(L, game::GameHandler::MAX_GLYPH_SLOTS);
            return 1;
        }},
                {"GetGlyphSocketInfo", [](lua_State* L) -> int {
            // GetGlyphSocketInfo(index [, talentGroup]) → enabled, glyphType, glyphSpellID, icon
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            int spec = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || index < 1 || index > game::GameHandler::MAX_GLYPH_SLOTS) {
                lua_pushboolean(L, 0); lua_pushnumber(L, 0); lua_pushnil(L); lua_pushnil(L);
                return 4;
            }
            const auto& glyphs = (spec >= 1 && spec <= 2)
                ? gh->getGlyphs(static_cast<uint8_t>(spec - 1)) : gh->getGlyphs();
            uint16_t glyphId = glyphs[index - 1];
            // Glyph type: slots 1,2,3 = major (1), slots 4,5,6 = minor (2)
            int glyphType = (index <= 3) ? 1 : 2;
            lua_pushboolean(L, 1);              // enabled
            lua_pushnumber(L, glyphType);       // glyphType (1=major, 2=minor)
            if (glyphId != 0) {
                // The *spell*, not the properties id the server sent. Every
                // caller treats this as a spell - the glyph panel builds its
                // link from it and the tooltip describes it - and pushing the
                // properties id gave them a number that means something else.
                // GlyphProperties.dbc field 1 is the spell; checked by
                // resolving it for all 362 rows, which come out named "Glyph
                // of Moonfire" and the like.
                gh->ensureGlyphPropertiesLoaded();
                const uint32_t spellId = gh->getGlyphSpellId(glyphId);
                lua_pushnumber(L, spellId ? spellId : glyphId);   // glyphSpellID
                // ...and its own icon rather than a warrior glyph for everyone.
                std::string icon = spellId ? gh->getSpellIconPath(spellId) : std::string();
                lua_pushstring(L, icon.empty()
                    ? "Interface\\Icons\\INV_Misc_QuestionMark" : icon.c_str());
            } else {
                lua_pushnil(L);
                lua_pushnil(L);
            }
            return 4;
        }},
                // ---- Trade skills ---------------------------------------
                //
                // The recipe list comes from GameHandler so this panel and the
                // client's own crafting window agree on every number.
                {"GetNumTradeSkills", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(
                tradeSkillRows(gh).size()) : 0.0);
            return 1;
        }},
                // GetTradeSkillInfo(i) → name, type, numAvailable, isExpanded,
                //                        altVerb, numSkillUps
                {"GetTradeSkillInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            const auto rows = tradeSkillRows(gh);
            // The scroll frame asks past the end on every redraw - it walks a
            // fixed number of buttons and hides the ones that answer nil - so
            // this is routine and says nothing about anything being wrong.
            if (i < 1 || i > static_cast<int>(rows.size())) return luaReturnNil(L);
            const auto& row = rows[static_cast<size_t>(i) - 1];
            if (row.isHeader) {
                // "header" is what the panel branches on; the count and the
                // verb belong to a recipe and a heading has neither.
                lua_pushstring(L, row.subclass.c_str());
                lua_pushstring(L, "header");
                lua_pushnumber(L, 0);
                lua_pushboolean(L,
                    collapsedTradeSkillSubClasses().count(row.subclass) ? 0 : 1);
                lua_pushnil(L);         // altVerb
                lua_pushnumber(L, 0);   // numSkillUps
                return 6;
            }
            const auto& r = row.recipe;
            static const char* kBands[4] = {"optimal", "medium", "easy", "trivial"};
            lua_pushstring(L, r.name.c_str());
            lua_pushstring(L, kBands[r.difficulty < 0 || r.difficulty > 3
                                         ? 0 : r.difficulty]);
            lua_pushnumber(L, r.canMake);
            lua_pushboolean(L, 1);      // a recipe row is never folded
            lua_pushnil(L);             // altVerb
            lua_pushnumber(L, 1);       // numSkillUps
            return 6;
        }},
                {"GetTradeSkillIcon", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            // What the recipe makes, which is what this returns in WoW. A
            // recipe spell carries its profession's icon rather than its
            // product's - every tailoring spell in the shipped Spell.dbc is
            // Ability_Ensnare - so the pane showed the same picture beside
            // every recipe. The spell's icon is the fallback for a recipe
            // whose item has not been answered for yet.
            std::string icon;
            if (const auto* made = craftedItem(gh, rec->spellId)) {
                if (made->displayInfoId) icon = gh->getItemIconPath(made->displayInfoId);
            }
            if (icon.empty()) icon = gh->getSpellIconPath(rec->spellId);
            lua_pushstring(L, icon.empty()
                ? "Interface\\Icons\\INV_Misc_QuestionMark" : icon.c_str());
            return 1;
        }},
                {"GetTradeSkillDescription", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            const uint32_t id = rec->spellId;
            lua_pushstring(L, gh->formatSpellDescription(
                id, gh->getSpellDescription(id)).c_str());
            return 1;
        }},
                {"GetTradeSkillNumReagents", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            int n = 0;
            if (gh) {
                const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
                if (rec) {
                    auto it = gh->spellNameCacheRef().find(rec->spellId);
                    if (it != gh->spellNameCacheRef().end()) {
                        for (const auto& r : it->second.reagents) {
                            if (r.itemId != 0) ++n;
                        }
                    }
                }
            }
            lua_pushnumber(L, n);
            return 1;
        }},
                // GetTradeSkillReagentInfo(i, n) → name, texture, needed, have
                {"GetTradeSkillReagentInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int n = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (!gh) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            auto it = gh->spellNameCacheRef().find(rec->spellId);
            if (it == gh->spellNameCacheRef().end()) return luaReturnNil(L);
            int seen = 0;
            for (const auto& r : it->second.reagents) {
                if (r.itemId == 0) continue;
                if (++seen != n) continue;
                gh->ensureItemInfo(r.itemId);
                const auto* info = gh->getItemInfo(r.itemId);
                lua_pushstring(L, info ? info->name.c_str() : "Reagent");
                const std::string ricon =
                    info ? gh->getItemIconPath(info->displayInfoId) : std::string();
                lua_pushstring(L, ricon.empty()
                    ? "Interface\\Icons\\INV_Misc_QuestionMark" : ricon.c_str());
                lua_pushnumber(L, r.count);
                lua_pushnumber(L, gh->countItemInBags(r.itemId));
                return 4;
            }
            return luaReturnNil(L);
        }},
                {"GetTradeSkillNumMade", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            int lo = 1, hi = 1;
            if (gh) {
                const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
                if (rec) {
                    auto it = gh->spellNameCacheRef().find(rec->spellId);
                    if (it != gh->spellNameCacheRef().end()) {
                        // The count is not in what this client parses; one is
                        // right for the great majority of recipes and wrong
                        // only in the amount, never in the item.
                        (void)it;
                    }
                }
            }
            lua_pushnumber(L, lo);
            lua_pushnumber(L, hi);
            return 2;
        }},
                // GetTradeSkillLine() → name, rank, maxRank
                {"GetTradeSkillLine", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) {
                // The same three values, not one nil. TradeSkillFrame_Update
                // reads the rank straight into `if ( rank < 75 )`, so a nil
                // there raises and takes the rest of the update with it -
                // including the re-selection that is the only thing which
                // re-enables the Create buttons after the Disable at the top.
                lua_pushstring(L, "Trade Skill");
                lua_pushnumber(L, 0);
                lua_pushnumber(L, 0);
                return 3;
            }
            const uint32_t line = gh->getCraftingSkillLine();
            std::string name = gh->getSkillLineName(line);
            if (name.empty()) name = "Trade Skill";
            uint32_t rank = 0, maxRank = 0;
            auto it = gh->getPlayerSkills().find(line);
            if (it != gh->getPlayerSkills().end()) {
                rank    = it->second.effectiveValue();
                maxRank = it->second.maxValue;
            }
            lua_pushstring(L, name.c_str());
            lua_pushnumber(L, rank);
            lua_pushnumber(L, maxRank);
            return 3;
        }},
                // DoTradeSkill(index, repeat) - make `repeat` of a recipe.
                //
                // Both of the trade skill frame's buttons pass a count: Create
                // sends what the box holds and Create All sends numAvailable.
                // Casting once regardless made "Create All" produce a single
                // item, with no error to say why. The craft queue this needs
                // already existed and the client's own crafting window has been
                // using it - one press per craft is not the same as a queue,
                // because each craft has to wait for the last to finish.
                {"DoTradeSkill", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            int count = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (!gh) return 0;
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            // Says what a press of Create resolved to, once per press. A craft
            // that does nothing is indistinguishable from a button that was
            // never wired: the row may be a heading, the index may be off the
            // end, or the cast may go out and be refused with nothing shown.
            // This names which, and it is the only place that knows the index
            // and the spell at the same time.
            // At warning, like the profession chain it sits next to. This was
            // written to answer "a craft that does nothing is indistinguishable
            // from a button that was never wired" and then said it at debug,
            // which the log a bug report arrives with does not carry - so a
            // report of crafting not working came back with nothing about the
            // press at all. The same mistake the Escape chain made.
            if (!rec) {
                LOG_WARNING("DoTradeSkill: no recipe at row ", i,
                            " of ", tradeSkillRows(gh).size(),
                            " - the row is a heading or the index is past the end");
                return 0;
            }
            if (count < 1) count = 1;
            LOG_WARNING("DoTradeSkill: row ", i, " '", rec->name,
                        "' spell=", rec->spellId, " count=", count);
            gh->startCraftQueue(rec->spellId, count);
            return 0;
        }},
                {"CloseTradeSkill", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closeCraftingWindow();
            return 0;
        }},
                {"SelectTradeSkill", [](lua_State* L) -> int {
            tradeSkillSelection() = static_cast<int>(luaL_optnumber(L, 1, 0));
            return 0;
        }},
                {"GetTradeSkillSelectionIndex", [](lua_State* L) -> int {
            lua_pushnumber(L, tradeSkillSelection());
            return 1;
        }},
                {"GetFirstTradeSkill", [](lua_State* L) -> int {
            // The first row that is a recipe rather than a heading, which is
            // what the panel selects when it opens. One now that the list has
            // headings: row one is "Armor" or "Weapon", not something to make.
            auto* gh = getGameHandler(L);
            const auto rows = gh ? tradeSkillRows(gh) : std::vector<TradeSkillRow>{};
            for (size_t n = 0; n < rows.size(); ++n) {
                if (!rows[n].isHeader) { lua_pushnumber(L, static_cast<double>(n + 1)); return 1; }
            }
            lua_pushnumber(L, 1);
            return 1;
        }},
                // GetTradeSkillCooldown(index) → seconds left, or nil.
                //
                // A recipe's cooldown is its spell's - transmutes and the salt
                // shaker are the ones that have any - and this client has been
                // tracking those all along. nil still means "not on cooldown",
                // which is the branch that leaves the line blank.
                {"GetTradeSkillCooldown", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || i < 1) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            const float left = gh->getSpellCooldown(rec->spellId);
            if (left <= 0.0f) return luaReturnNil(L);
            lua_pushnumber(L, left);
            return 1;
        }},
                {"GetTradeSkillTools", [](lua_State* L) -> int {
            // NOTHING, not a nil. The tool requirement is not in what this
            // client parses, and claiming a tool is missing would grey out
            // recipes that work - but answering one nil is not the same as
            // answering none, and the difference is the whole bug.
            //
            // The only caller is
            //   local spellFocus = BuildColoredListString(GetTradeSkillTools(id));
            // and BuildColoredListString gives up only on an empty argument
            // list: `if ( select("#", ...) == 0 ) then return nil end`. One nil
            // is a list of one, so it walked past that, took `text` as nil, and
            // raised concatenating it.
            //
            // That line sits in TradeSkillFrame_SetSelection ABOVE
            // `if ( creatable ) then TradeSkillCreateButton:Enable()`, so the
            // raise took the selection down before either Create button was
            // enabled again - they are disabled at the top of every trade skill
            // update and re-enabled only there. Both stayed greyed with the
            // reagents in the bag and the recipe drawn, because everything the
            // panel had already filled in happened before the raise.
            (void)L;
            return 0;
        }},
                // Links, which need an item this client does not resolve for a
                // recipe, and the play-time limits that only Chinese realms set.
                // The two links a trade skill row can be shift-clicked for:
                // what the recipe makes, and what it takes. Both item ids were
                // already here - createdItemId off the spell and the reagent
                // list GetTradeSkillReagentInfo walks - so the panel could name
                // and count them while answering nil to anyone asking for a
                // link to the same thing.
                {"GetTradeSkillItemLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || i < 1) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            auto it = gh->spellNameCacheRef().find(rec->spellId);
            if (it == gh->spellNameCacheRef().end()) return luaReturnNil(L);
            const uint32_t made = it->second.createdItemId;
            if (made == 0) return luaReturnNil(L);
            gh->ensureItemInfo(made);
            const auto* info = gh->getItemInfo(made);
            if (!info || !info->valid) return luaReturnNil(L);
            lua_pushstring(L, game::buildItemLink(made, info->quality, info->name).c_str());
            return 1;
        }},
                // A recipe link is an |Htrade: hyperlink carrying the whole
                // skill list, which this client has no reader for - a link it
                // cannot resolve on the way back in would be worse than none.
                {"GetTradeSkillRecipeLink",   [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"GetTradeSkillReagentItemLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int n = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (!gh || i < 1 || n < 1) return luaReturnNil(L);
            const auto* rec = tradeSkillRecipeAt(tradeSkillRows(gh), i);
            if (!rec) return luaReturnNil(L);
            auto it = gh->spellNameCacheRef().find(rec->spellId);
            if (it == gh->spellNameCacheRef().end()) return luaReturnNil(L);
            int seen = 0;
            for (const auto& r : it->second.reagents) {
                if (r.itemId == 0) continue;
                if (++seen != n) continue;
                gh->ensureItemInfo(r.itemId);
                const auto* info = gh->getItemInfo(r.itemId);
                if (!info || !info->valid) return luaReturnNil(L);
                lua_pushstring(L,
                    game::buildItemLink(r.itemId, info->quality, info->name).c_str());
                return 1;
            }
            return luaReturnNil(L);
        }},
                {"GetTradeSkillListLink",     [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"IsTradeSkillLinked",        [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"NoPlayTime",                [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"PartialPlayTime",           [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"GetBillingTimeRested",      [](lua_State* L) -> int { lua_pushnumber(L, 0); return 1; }},
                // Filters and sub-classes: panel state, kept so it reads back.
                // The trade skill window's two filter dropdowns. Both answered
                // nothing and both are spread straight into a vararg call -
                // TradeSkillFilterFrame_LoadSubClasses(GetTradeSkillSubClasses())
                // - so each dropdown built its "All" row and stopped, and
                // picking anything was impossible rather than merely inert.
                //
                // Single-select, not a set of ticks: the click handler calls
                // Set…Filter(id - 1, 1, 1) and nothing ever unticks, so the
                // state is one index and zero is the "All" row.
                {"GetTradeSkillSubClasses", [](lua_State* L) -> int {
            const auto subs = tradeSkillSubClasses(getGameHandler(L));
            for (const auto& s : subs) lua_pushstring(L, s.c_str());
            return static_cast<int>(subs.size());
        }},
                {"GetTradeSkillInvSlots", [](lua_State* L) -> int {
            const auto slots = tradeSkillInvSlots(getGameHandler(L));
            for (const auto& s : slots) lua_pushstring(L, s.c_str());
            return static_cast<int>(slots.size());
        }},
                {"GetTradeSkillSubClassFilter", [](lua_State* L) -> int {
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            lua_pushboolean(L, i == tradeSkillSubClassPick() ? 1 : 0);
            return 1;
        }},
                {"GetTradeSkillInvSlotFilter", [](lua_State* L) -> int {
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            lua_pushboolean(L, i == tradeSkillInvSlotPick() ? 1 : 0);
            return 1;
        }},
                {"SetTradeSkillSubClassFilter", [](lua_State* L) -> int {
            tradeSkillSubClassPick() = static_cast<int>(luaL_optnumber(L, 1, 0));
            ++tradeSkillRowsGeneration();
            return 0;
        }},
                {"SetTradeSkillInvSlotFilter", [](lua_State* L) -> int {
            tradeSkillInvSlotPick() = static_cast<int>(luaL_optnumber(L, 1, 0));
            ++tradeSkillRowsGeneration();
            return 0;
        }},
                // The search box above the recipe list. It matched nothing
                // because nothing read it; the panel calls TradeSkillFrame_Update
                // straight after, so the list redraws against the new view.
                {"SetTradeSkillItemNameFilter", [](lua_State* L) -> int {
            tradeSkillNameFilter() = luaL_optstring(L, 1, "");
            ++tradeSkillRowsGeneration();
            return 0;
        }},
                // A number in the search box filters by the level of what the
                // recipe makes rather than by its name - "25", "20-30" and
                // "~25" all reach here, and the panel clears the name filter
                // when they do. Both zero clears this one, which is what it
                // sends when the text is not a number.
                //
                // The arguments arrive as the strings strmatch produced;
                // luaL_optnumber reads those.
                {"SetTradeSkillItemLevelFilter", [](lua_State* L) -> int {
            const int lo = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int hi = static_cast<int>(luaL_optnumber(L, 2, 0));
            tradeSkillLevelRange() = {lo, hi};
            ++tradeSkillRowsGeneration();
            return 0;
        }},
                // The "Have Materials" checkbox, which had nothing behind it.
                // canMake is the count this client already works out for the
                // reagent lines, so the filter is the same number read twice.
                {"TradeSkillOnlyShowMakeable",  [](lua_State* L) -> int {
            tradeSkillOnlyMakeable() = lua_toboolean(L, 1) != 0;
            ++tradeSkillRowsGeneration();
            return 0;
        }},
                {"CollapseTradeSkillSubClass",  [](lua_State* L) -> int {
            return tradeSkillHeaderSetCollapsed(L, true);
        }},
                {"ExpandTradeSkillSubClass",    [](lua_State* L) -> int {
            return tradeSkillHeaderSetCollapsed(L, false);
        }},
                // GetTradeskillRepeatCount() → how many are still to be made.
                //
                // This is not only a readout: the trade skill frame does
                // TradeSkillInputBox:SetNumber(GetTradeskillRepeatCount()) every
                // time a recipe is selected and again on UPDATE_TRADESKILL_RECAST,
                // so answering zero pinned the quantity box at zero and Create
                // asked for none however many the player typed. One when no
                // queue is running, which is what a fresh selection should show.
                {"GetTradeskillRepeatCount", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int left = gh ? gh->getCraftQueueRemaining() : 0;
            lua_pushnumber(L, left > 1 ? left : 1);
            return 1;
        }},
                {"StopTradeSkillRepeat", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->cancelCraftQueue();
            return 0;
        }},
                // ---- Trainer -------------------------------------------
                //
                // The client has parsed the trainer list all along - spell,
                // cost, required level and skill, and whether it is already
                // known - and fires TRAINER_SHOW when it arrives. None of it
                // had a way into the interface, so the panel opened blank at
                // every trainer in the game.
                {"GetNumTrainerServices", [](lua_State* L) -> int {
            // What the filters leave, not what the trainer sent.
            lua_pushnumber(L, static_cast<double>(
                shownTrainerServices(getGameHandler(L)).size()));
            return 1;
        }},
                // GetTrainerServiceInfo(i) → name, rank, category, expanded
                {"GetTrainerServiceInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) return luaReturnNil(L);
            const auto& sp = *svc;
            std::string name = gh->getSpellName(sp.spellId);
            if (name.empty()) name = "Spell " + std::to_string(sp.spellId);
            lua_pushstring(L, name.c_str());
            // The rank, from the same place the name came from. The trainer
            // list on the wire carries neither - both are looked up by spell id
            // - so "this list does not carry it" was true of the packet and
            // not of the client, which has had getSpellRank all along.
            //
            // It is what tells two rows apart. The panel writes it in
            // parentheses after the name, so a trainer offering Fireball at
            // three ranks listed Fireball three times, identically. An empty
            // string still hides the line, which is right for a spell that has
            // no rank.
            lua_pushstring(L, gh->getSpellRank(sp.spellId).c_str());   // rank
            // The three words the panel colours by: green, grey, and already
            // trained.
            //
            // "used" wins the moment the spell is known, not only when the
            // trainer packet said so. The packet is a snapshot from when the
            // list was opened; learning a service adds it to the known set and
            // fires TRAINER_UPDATE, but the cached state stayed at 0. So the
            // panel redrew the just-learned skill as still available - green,
            // trainable, its cost still shown - and only a close and reopen,
            // which fetches a fresh list, corrected it. Asking the known set
            // makes the live redraw right.
            lua_pushstring(L, trainerServiceCategory(gh, sp));
            lua_pushboolean(L, 1);   // expanded: the list here is flat
            return 4;
        }},
                // Three costs: coin, talent points, and a profession slot.
                // The trainer reads the third bare - `if ( cpCost2 > 0 )` - so
                // one value made selecting anything a trainer offers an error.
                //
                // The second and third were zero on the reasoning that nothing
                // in the trainer list says which service is a profession. The
                // list says exactly that and has all along: profDialog and
                // profButton are the packet's two point costs, and AzerothCore
                // fills the second with `primaryProfessionFirstRank ? 1 : 0`
                // - the one thing it is for. Parsed since the parser was
                // written, stored, and read by nothing.
                //
                // What it buys: learning a first profession rank now raises
                // CONFIRM_PROFESSION, and the Train button greys when both
                // slots are already spent. Both come free from the frame once
                // the number is real, because UnitCharacterPoints already
                // answers the free-slot count it is compared against.
                {"GetTrainerServiceCost", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            lua_pushnumber(L, svc ? svc->spellCost  : 0);
            lua_pushnumber(L, svc ? svc->profDialog : 0);
            lua_pushnumber(L, svc ? svc->profButton : 0);
            return 3;
        }},
                {"GetTrainerServiceLevelReq", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                lua_pushnumber(L, 0);
                return 1;
            }
            lua_pushnumber(L, svc->reqLevel);
            return 1;
        }},
                // GetTrainerServiceSkillReq(i) → skill name, required value
                {"GetTrainerServiceSkillReq", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                return luaReturnNil(L);
            }
            const auto& sp = *svc;
            if (sp.reqSkill == 0) return luaReturnNil(L);
            const std::string skill = gh->getSkillLineName(sp.reqSkill);
            lua_pushstring(L, skill.empty() ? "Skill" : skill.c_str());
            lua_pushnumber(L, sp.reqSkillValue);
            // Whether the player already meets it. The trainer window picks
            // between TRAINER_REQ_SKILL_RANK and its _RED twin on this, so
            // leaving it nil painted every requirement red - including the
            // ones already satisfied, next to a spell the player could train.
            const auto& skills = gh->getPlayerSkills();
            const auto it = skills.find(sp.reqSkill);
            const bool met = it != skills.end() &&
                             it->second.effectiveValue() >= sp.reqSkillValue;
            lua_pushboolean(L, met ? 1 : 0);
            return 3;
        }},
                {"GetTrainerServiceIcon", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                return luaReturnNil(L);
            }
            const std::string icon = gh->getSpellIconPath(svc->spellId);
            lua_pushstring(L, icon.empty()
                ? "Interface\\Icons\\INV_Misc_QuestionMark" : icon.c_str());
            return 1;
        }},
                {"GetTrainerServiceDescription", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                return luaReturnNil(L);
            }
            const uint32_t id = svc->spellId;
            const std::string desc =
                gh->formatSpellDescription(id, gh->getSpellDescription(id));
            lua_pushstring(L, desc.c_str());
            return 1;
        }},
                {"GetTrainerServiceSkillLine", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                return luaReturnNil(L);
            }
            const std::string skill = gh->getSkillLineName(svc->reqSkill);
            lua_pushstring(L, skill.empty() ? "" : skill.c_str());
            return 1;
        }},
                // The prerequisite chain, which this list carries as up to
                // three spell ids.
                {"GetTrainerServiceNumAbilityReq", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                lua_pushnumber(L, 0);
                return 1;
            }
            const auto& sp = *svc;
            int n = 0;
            if (sp.chainNode1) ++n;
            if (sp.chainNode2) ++n;
            if (sp.chainNode3) ++n;
            lua_pushnumber(L, n);
            return 1;
        }},
                // GetTrainerServiceAbilityReq(i, n) → name, hasIt
                {"GetTrainerServiceAbilityReq", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int n = static_cast<int>(luaL_optnumber(L, 2, 1));
            const auto* svc = shownTrainerService(gh, i);
            if (!svc) {
                return luaReturnNil(L);
            }
            const auto& sp = *svc;
            const uint32_t chain[3] = {sp.chainNode1, sp.chainNode2, sp.chainNode3};
            if (n < 1 || n > 3 || chain[n - 1] == 0) return luaReturnNil(L);
            const uint32_t req = chain[n - 1];
            std::string name = gh->getSpellName(req);
            if (name.empty()) name = "Ability";
            lua_pushstring(L, name.c_str());
            lua_pushboolean(L, gh->getKnownSpells().count(req) ? 1 : 0);
            return 2;
        }},
                // GetTrainerServiceStepReq(i) → step, met.
                //
                // Nil, not zero. The trainer window does `if ( step ) then`
                // and zero is truthy in Lua, so answering 0 claimed every
                // service had a step requirement - and with `met` missing it
                // took the red branch, printing a bogus unmet requirement of
                // "0" beside every spell on the list.
                //
                // No step requirement is tracked here, and saying so is both
                // true and what removes the line.
                {"GetTrainerServiceStepReq", [](lua_State* L) -> int {
            return luaReturnNil(L);
        }},
                {"GetTrainerServiceItemLink", [](lua_State* L) -> int {
            return luaReturnNil(L);
        }},
                {"GetTrainerGreetingText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const std::string& g = gh ? gh->getTrainerSpells().greeting
                                      : std::string();
            lua_pushstring(L, g.c_str());
            return 1;
        }},
                {"IsTradeskillTrainer", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->getTrainerSpells().trainerType == 2);
            return 1;
        }},
                {"BuyTrainerService", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
            // Through the filtered list like every other index, and this is
            // the one where getting it wrong buys a different spell than the
            // row that was clicked.
            const auto* svc = shownTrainerService(gh, i);
            if (svc) gh->trainSpell(svc->spellId);
            return 0;
        }},
                {"CloseTrainer", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closeTrainer();
            return 0;
        }},
                // Selection and the type filter are the panel's own state; the
                // client has no opinion about either, so they are kept here
                // rather than pretended away - the panel reads back what it set.
                {"SelectTrainerService", [](lua_State* L) -> int {
            trainerSelection() = static_cast<int>(luaL_optnumber(L, 1, 0));
            return 0;
        }},
                {"GetTrainerSelectionIndex", [](lua_State* L) -> int {
            lua_pushnumber(L, trainerSelection());
            return 1;
        }},
                {"SetTrainerServiceTypeFilter", [](lua_State* L) -> int {
            const char* which = luaL_optstring(L, 1, "");
            trainerFilters()[which] = lua_toboolean(L, 2) != 0;
            return 0;
        }},
                {"GetTrainerServiceTypeFilter", [](lua_State* L) -> int {
            const char* which = luaL_optstring(L, 1, "");
            auto it = trainerFilters().find(which);
            // Unset means showing, which is how a freshly opened panel looks.
            lua_pushboolean(L, it == trainerFilters().end() ? 1 : (it->second ? 1 : 0));
            return 1;
        }},
                // Inert, and this one really is - unlike the three other
                // Collapse/Expand pairs, which all turned out to have their
                // grouping sitting in data the client already held.
                //
                // Blizzard's name says the heading is a skill line, and the
                // only skill line in the packet is ReqSkillLine. Across
                // AzerothCore's whole npc_trainer table - 880 trainers, 4934
                // rows - not one trainer has more than a single distinct
                // non-zero value, and 816 have none at all. So grouping by it
                // yields one heading over the entire list, or none, which is
                // strictly worse than the flat list it would replace.
                //
                // Checked against the table rather than argued from the packet,
                // because the packet plainly *could* carry several and the
                // question is whether any server sends them.
                {"CollapseTrainerSkillLine", [](lua_State* L) -> int { (void)L; return 0; }},
                {"ExpandTrainerSkillLine",   [](lua_State* L) -> int { (void)L; return 0; }},
                // GetGlyphLink(socket [, talentGroup]) → hyperlink, or nil
                {"GetGlyphLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int spec  = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || index < 1 || index > game::GameHandler::MAX_GLYPH_SLOTS) {
                lua_pushnil(L);
                return 1;
            }
            const auto& glyphs = (spec >= 1 && spec <= 2)
                ? gh->getGlyphs(static_cast<uint8_t>(spec - 1)) : gh->getGlyphs();
            const uint16_t glyphId = glyphs[index - 1];
            if (glyphId == 0) { lua_pushnil(L); return 1; }
            std::string name = gh->getSpellName(glyphId);
            if (name.empty()) name = "Glyph";
            const std::string link = "|cff66bbff|Hglyph:" + std::to_string(index) +
                                     ":" + std::to_string(glyphId) + "|h[" + name +
                                     "]|h|r";
            lua_pushstring(L, link.c_str());
            return 1;
        }},
                // GlyphMatchesSocket(socket) → whether what is on the cursor
                // fits. Still false, and now for a narrower reason: the cursor
                // does carry the item, but nothing here reads a glyph's own
                // socket type - major glyphs go in three sockets and minor ones
                // in the other three, and lighting all six would offer a drop
                // the server then refuses.
                {"GlyphMatchesSocket", [](lua_State* L) -> int {
            lua_pushboolean(L, 0);
            return 1;
        }},
                
                // SetCursor(art) - the pointer's own image, which this client
                // does not change.
                {"SetCursor", [](lua_State* L) -> int { (void)L; return 0; }},
                // How long until the daily quests reset, which the quest log
                // prints in a tooltip through SecondsToTime. A nil there is
                // arithmetic on nothing rather than a blank line.
                //
                // The server's own figure, asked for once on entering the
                // world: SMSG_QUERY_TIME_RESPONSE's second word is
                // GetNextDailyQuestsResetTime() minus now. This used to count
                // to the next local midnight, under a comment saying the real
                // reset was a realm setting nothing sends - it is sent, in the
                // reply to a packet this client already knew how to ask for,
                // and it was parsed and dropped.
                //
                // Midnight remains the answer until the reply lands, and for
                // any realm that does not send one. Zero would be wrong in a
                // way a wrong number is not: SecondsToTime renders it as "the
                // reset is now", every time the tooltip is opened.
                {"GetQuestResetTime", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                if (const uint32_t left = gh->getSecondsUntilDailyReset(); left > 0) {
                    lua_pushnumber(L, left);
                    return 1;
                }
            }
            const time_t now = time(nullptr);
            const std::tm t = core::localTime(now);
            const int secondsIntoDay = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
            lua_pushnumber(L, 86400 - secondsIntoDay);
            return 1;
        }},
                // The cursor changes at a vendor: the hand that sells, and the
                // hammer that repairs. This client draws its own cursor and
                // has no art switched by name, so these do nothing - which is
                // what they did before, silently, as unknown globals.
                //
                // Bound rather than left to the fallback because the fallback
                // is what makes a typo look like a working call, and because a
                // window that calls nothing undefined is the measure of
                // whether it can be handed over.
                {"ShowMerchantSellCursor", [](lua_State* L) -> int { (void)L; return 0; }},
                // The merchant's repair-an-item button toggles between these
                // two and reads the state back through InRepairMode. All three
                // were stubs, so the button never latched and the per-item
                // repair the bags and the paperdoll gate on it was dead.
                {"ShowRepairCursor",       [](lua_State* L) -> int {
            (void)L; repairCursorUp() = true; return 0; }},
                {"HideRepairCursor",       [](lua_State* L) -> int {
            (void)L; repairCursorUp() = false; return 0; }},
                // GetNumCompletedAchievements() → total, completed.
                //
                // Two values, and the total comes first. This returned one -
                // the *earned* count in the total's place - so the summary bar
                // was scaled to the number earned, and `completed` was nil.
                // AchievementFrameSummaryCategoriesStatusBar_Update then does
                // SetText(completed.."/"..total), and concatenating nil raises,
                // so opening the achievements panel took its own update down.
                {"GetNumCompletedAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
            // The name cache is one entry per achievement in the DBC, which is
            // the only count of "all of them" this client has. Asking for it
            // is what loads it.
            gh->ensureAchievementNamesLoaded();
            const size_t total = gh->achievementNameCacheRef().size();
            const size_t earned = gh->getEarnedAchievements().size();
            // Never fewer total than earned: if the DBC did not load, saying
            // "3 of 0" is worse than saying the total is what we have seen.
            lua_pushnumber(L, static_cast<lua_Number>(total ? total : earned));
            lua_pushnumber(L, static_cast<lua_Number>(earned));
            return 2;
        }},
                // GetCategoryList() → the achievement category ids, as a table.
                {"GetCategoryList", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_newtable(L);
            if (!gh) return 1;
            gh->ensureAchievementCategoriesLoaded();
            int n = 0;
            for (uint32_t id : gh->getAchievementCategoryOrder()) {
                lua_pushnumber(L, ++n);
                lua_pushnumber(L, id);
                lua_settable(L, -3);
            }
            return 1;
        }},
                // GetStatisticsCategoryList() → the same for the other tab.
                //
                // The Statistics tab reads this out of a table built when the
                // panel loads - STAT_FUNCTIONS.categoryAccessor - and then calls
                // it: `local cats = achievementFunctions.categoryAccessor()`.
                // Absent, that is a call on a nil field, so opening Statistics
                // raised rather than showing an empty tab. The list it wants is
                // the half GetCategoryList used to return along with everything
                // else, which put rows nothing can complete under Achievements.
                {"GetStatisticsCategoryList", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_newtable(L);
            if (!gh) return 1;
            gh->ensureAchievementCategoriesLoaded();
            int n = 0;
            for (uint32_t id : gh->getStatisticCategoryOrder()) {
                lua_pushnumber(L, ++n);
                lua_pushnumber(L, id);
                lua_settable(L, -3);
            }
            return 1;
        }},
                // GetCategoryInfo(id) → name, parentCategoryID, flags
                {"GetCategoryInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            gh->ensureAchievementCategoriesLoaded();
            const auto* info = gh->getAchievementCategoryInfo(id);
            if (!info) return luaReturnNil(L);
            lua_pushstring(L, info->name.c_str());
            lua_pushnumber(L, info->parentId);
            lua_pushnumber(L, 0);
            return 3;
        }},
                // GetCategoryNumAchievements(id) → total, completed, incomplete
                {"GetCategoryNumAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 3; }
            gh->ensureAchievementCategoriesLoaded();
            const auto& ids = gh->getCategoryAchievements(id);
            const auto& earned = gh->getEarnedAchievements();
            uint32_t done = 0;
            for (uint32_t a : ids) if (earned.count(a)) ++done;
            lua_pushnumber(L, static_cast<lua_Number>(ids.size()));
            lua_pushnumber(L, done);
            lua_pushnumber(L, static_cast<lua_Number>(ids.size() - done));
            return 3;
        }},
                {"GetAchievementCategory", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh) return luaReturnNil(L);
            gh->ensureAchievementCategoriesLoaded();
            lua_pushnumber(L, gh->getAchievementCategory(id));
            return 1;
        }},
                {"GetAchievementNumCriteria", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            gh->ensureAchievementCriteriaLoaded();
            lua_pushnumber(L, static_cast<lua_Number>(gh->getAchievementCriteria(id).size()));
            return 1;
        }},
                // GetAchievementCriteriaInfo(achievementID, index) →
                //   description, criteriaType, completed, quantity,
                //   reqQuantity, charName, flags, assetID, quantityString,
                //   criteriaID
                {"GetAchievementCriteriaInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id  = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            const int  idx = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || idx < 1) return luaReturnNil(L);
            gh->ensureAchievementCriteriaLoaded();
            const auto& list = gh->getAchievementCriteria(id);
            if (idx > static_cast<int>(list.size())) return luaReturnNil(L);
            const auto& c = list[static_cast<size_t>(idx) - 1];
            // Per-criterion progress is tracked after all: SMSG_ALL_ACHIEVEMENT_DATA
            // carries a counter per criterion id, and this reported all-or-none
            // from the achievement's earned flag while that sat unread beside
            // it. A half-finished achievement now shows which of its criteria
            // are done and how far the rest have got.
            const auto& progress = gh->getCriteriaProgress();
            const auto pit = progress.find(c.id);
            const uint32_t have = (pit != progress.end())
                ? static_cast<uint32_t>(pit->second) : 0u;
            const bool earned = gh->getEarnedAchievements().count(id) > 0;
            // Earned wins over the counter: the server stops counting once an
            // achievement is complete, so a finished one can carry a criterion
            // still short of its quantity.
            const bool done = earned || (c.quantity > 0 && have >= c.quantity);
            const uint32_t shown = earned ? c.quantity : have;
            lua_pushstring(L, c.description.c_str());          // 1: description
            lua_pushnumber(L, c.type);                         // 2: criteriaType
            lua_pushboolean(L, done ? 1 : 0);                  // 3: completed
            lua_pushnumber(L, shown);                          // 4: quantity
            lua_pushnumber(L, c.quantity);                     // 5: reqQuantity
            lua_pushnil(L);                                    // 6: charName
            // The panel reads bit one of this to decide between a progress
            // bar and a tick, so zero drew every counted criterion as a tick.
            lua_pushnumber(L, c.flags);                        // 7: flags
            lua_pushnumber(L, c.assetId);                      // 8: assetID
            lua_pushstring(L, std::to_string(shown).c_str());  // 9: quantityString
            lua_pushnumber(L, c.id);                           // 10: criteriaID
            return 10;
        }},
                // GetLatestCompletedAchievements() → the most recent earned
                // ids, newest first. The summary page fills its top rows from
                // these. The earn date is a WoW PackedTime - year in the low
                // sixteen bits, then day, then month - so it does not sort as
                // a plain integer and has to be unpacked first.
                {"GetLatestCompletedAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            std::vector<std::pair<uint32_t, uint32_t>> byDate;  // sortKey, id
            for (uint32_t id : gh->getEarnedAchievements()) {
                const game::WowDate d =
                    game::unpackWowPackedTime(gh->getAchievementDate(id));
                byDate.emplace_back(
                    (static_cast<uint32_t>(d.yearSince2000) << 9) |
                    (static_cast<uint32_t>(d.month) << 5) |
                    static_cast<uint32_t>(d.day), id);
            }
            std::sort(byDate.begin(), byDate.end(),
                      [](const auto& a, const auto& b) {
                          if (a.first != b.first) return a.first > b.first;
                          return a.second > b.second;  // stable, and a total order
                      });
            const size_t n = std::min<size_t>(byDate.size(), 5);
            for (size_t i = 0; i < n; ++i) lua_pushnumber(L, byDate[i].second);
            return static_cast<int>(n);
        }},
                // GetAchievementInfoFromCriteria(criteriaID) → the achievement
                // that criterion belongs to, in GetAchievementInfo's shape.
                {"GetAchievementInfoFromCriteria", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto critId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || critId == 0) return luaReturnNil(L);
            gh->ensureAchievementCriteriaLoaded();
            // Nothing indexes criteria by their own id - the panel asks this
            // once per link clicked, not per frame, so the walk is cheaper
            // than a second map kept in step with the first.
            for (const auto& [achId, list] : gh->getAchievementCriteriaMap()) {
                for (const auto& c : list) {
                    if (c.id != critId) continue;
                    // Ten, the same shape as GetAchievementInfo: the panel
                    // unpacks it into the same ten names and puts the last one
                    // on a button as its texture.
                    const bool done = gh->getEarnedAchievements().count(achId) > 0;
                    const uint32_t date = gh->getAchievementDate(achId);
                    lua_pushnumber(L, achId);                                    // 1: id
                    lua_pushstring(L, gh->getAchievementName(achId).c_str());    // 2: name
                    lua_pushnumber(L, gh->getAchievementPoints(achId));          // 3: points
                    lua_pushboolean(L, done ? 1 : 0);                            // 4: completed
                    const game::WowDate on = game::unpackWowPackedTime(date);
                    lua_pushnumber(L, done ? on.month : 0);                      // 5: month
                    lua_pushnumber(L, done ? on.day : 0);                        // 6: day
                    // SHORTDATE formats the year "%02d", so it wants the short
                    // form rather than a four-digit one.
                    lua_pushnumber(L, done ? on.yearSince2000 : 0);              // 7: year
                    lua_pushstring(L, gh->getAchievementDescription(achId).c_str()); // 8
                    lua_pushnumber(L, gh->getAchievementFlags(achId));            // 9: flags
                    // A path, not the id. The panel puts this straight on a
                    // button as its texture, and a number is not a texture in
                    // 3.3.5 - the icon id was already cached, and the resolver
                    // that turns one into a path was already here for the
                    // spellbook's tabs.
                    lua_pushstring(L, achievementIconPath(gh, achId).c_str());   // 10: icon
                    return 10;
                }
            }
            return luaReturnNil(L);
        }},
                // The two ends of an achievement chain, off Achievement.dbc's
                // Supercedes column. Answering nil for both made every chained
                // achievement look like a single one: the panel shows the
                // expand arrow only when there is a previous step
                // (blizzard_achievementui.lua:828), and builds the list of
                // finished steps by walking GetPreviousAchievement until it
                // stops. So "Level 20" showed nothing of the "Level 10" behind
                // it, and neither did any cooking, fishing or battleground
                // series.
                {"GetPreviousAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return luaReturnNil(L);
            gh->ensureAchievementCategoriesLoaded();
            const uint32_t prev = gh->getAchievementSupercedes(id);
            if (prev == 0) return luaReturnNil(L);
            lua_pushnumber(L, prev);
            return 1;
        }},
                // ...and forwards, with whether that next step is already done
                // - AchievementFrameSummaryAchievement_OnClick walks
                // `newID, completed = GetNextAchievement(nextID)` to find the
                // furthest one earned, so the second value is what stops it.
                {"GetNextAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return luaReturnNil(L);
            gh->ensureAchievementCategoriesLoaded();
            const uint32_t next = gh->getAchievementSupercededBy(id);
            if (next == 0) return luaReturnNil(L);
            lua_pushnumber(L, next);
            lua_pushboolean(L, gh->getEarnedAchievements().count(next) > 0 ? 1 : 0);
            return 2;
        }},
                // GetStatistic(id) → the statistic's value, as a string.
                // Statistics count criteria progress, and only whether an
                // achievement was earned is tracked here, so this reports the
                // dash WoW itself shows for a statistic with no value.
                // GetStatistic(achievementId) → the counter, as a string.
                //
                // A statistic is an achievement with one criterion and no
                // quantity to reach, so its value is simply that criterion's
                // counter - the same counter GetAchievementCriteriaInfo reads
                // for a half-finished achievement, which was already tracked
                // while this answered "--" for every row. "--" is still the
                // answer when nothing has been counted, because that is what
                // the real client shows for a statistic never triggered.
                {"GetStatistic", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) { lua_pushstring(L, "--"); return 1; }
            gh->ensureAchievementCriteriaLoaded();
            const auto& list = gh->getAchievementCriteria(id);
            if (list.empty()) { lua_pushstring(L, "--"); return 1; }
            const auto& progress = gh->getCriteriaProgress();
            const auto pit = progress.find(list[0].id);
            if (pit == progress.end()) { lua_pushstring(L, "--"); return 1; }
            lua_pushstring(L, std::to_string(
                static_cast<uint64_t>(pit->second)).c_str());
            return 1;
        }},
                // The comparison tab, which used to say the server is never
                // asked for another player's achievements. It is: inspecting a
                // player on Wrath sends CMSG_QUERY_INSPECT_ACHIEVEMENTS beside
                // the inspect itself, and the reply has been parsed into a set
                // per guid the whole time. What was missing was naming whose
                // set to read.
                {"SetAchievementComparisonUnit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "");
            if (!gh || !uid || !*uid) return 0;
            std::string unit(uid);
            toLowerInPlace(unit);
            gh->setAchievementComparisonGuid(resolveUnitGuid(gh, unit));
            return 0;
        }},
                {"ClearAchievementComparisonUnit", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->setAchievementComparisonGuid(0);
            return 0;
        }},
                // GetAchievementComparisonInfo(id) → completed, month, day, year.
                //
                // Nil when nobody is being compared, which is what the tab reads
                // as "no data yet" - distinct from a false that would claim the
                // other player has not earned it.
                {"GetAchievementComparisonInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return luaReturnNil(L);
            const auto* set = gh->getInspectedPlayerAchievements(
                gh->getAchievementComparisonGuid());
            if (!set) return luaReturnNil(L);
            const auto it = set->find(id);
            if (it == set->end()) {
                lua_pushboolean(L, 0);
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0);
                return 4;
            }
            const game::WowDate on = game::unpackWowPackedTime(it->second);
            lua_pushboolean(L, 1);
            lua_pushnumber(L, on.month);
            lua_pushnumber(L, on.day);
            lua_pushnumber(L, on.yearSince2000);
            return 4;
        }},
                // Their statistics, which the reply does carry counters for and
                // this client walks past to reach the end of the packet. Left
                // as the dash until those are kept: a zero would read as a
                // player who has never done the thing.
                {"GetComparisonStatistic",         [](lua_State* L) -> int { lua_pushstring(L, "--"); return 1; }},
                {"GetComparisonAchievementPoints", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto* set = gh->getInspectedPlayerAchievements(
                gh->getAchievementComparisonGuid());
            if (!set) { lua_pushnumber(L, 0); return 1; }
            gh->ensureAchievementCategoriesLoaded();
            uint32_t total = 0;
            for (const auto& [id, when] : *set) {
                (void)when;
                total += gh->getAchievementPoints(id);
            }
            lua_pushnumber(L, total);
            return 1;
        }},
                {"GetTotalAchievementPoints", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getTotalAchievementPoints() : 0);
            return 1;
        }},
                {"GetAchievementLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return luaReturnNil(L);
            gh->ensureAchievementNamesLoaded();
            const std::string& name = gh->getAchievementName(id);
            if (name.empty()) return luaReturnNil(L);
            std::string link = "|cffffff00|Hachievement:" + std::to_string(id) +
                               ":0:0:0:0:0:0:0:0:0|h[" + name + "]|h|r";
            lua_pushstring(L, link.c_str());
            return 1;
        }},
                // ---- Achievement tracking (client-side, like the quest tracker)
                {"IsTrackedAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            lua_pushboolean(L, gh && gh->getTrackedAchievements().count(id) ? 1 : 0);
            return 1;
        }},
                {"AddTrackedAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (gh && id) { gh->setAchievementTracked(id, true); gh->fireAddonEvent("TRACKED_ACHIEVEMENT_UPDATE", {std::to_string(id)}); }
            return 0;
        }},
                {"RemoveTrackedAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (gh && id) { gh->setAchievementTracked(id, false); gh->fireAddonEvent("TRACKED_ACHIEVEMENT_UPDATE", {std::to_string(id)}); }
            return 0;
        }},
                {"GetNumTrackedAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<lua_Number>(gh->getTrackedAchievements().size()) : 0);
            return 1;
        }},
                {"GetAchievementInfo", [](lua_State* L) -> int {
            // GetAchievementInfo(id) → id, name, points, completed, month, day, year, description, flags, icon, rewardText, isGuildAch
            auto* gh = getGameHandler(L);
            uint32_t id = static_cast<uint32_t>(luaL_checknumber(L, 1));
            if (!gh) { return luaReturnNil(L); }
            // The other form the panel uses: GetAchievementInfo(category,
            // index) walks a category's achievements by position. Only the
            // by-id form existed, so every row the achievements panel asked
            // for was read as an achievement id and answered with the wrong
            // achievement, or with nothing.
            if (!lua_isnoneornil(L, 2)) {
                const int idx = static_cast<int>(luaL_optnumber(L, 2, 0));
                gh->ensureAchievementCategoriesLoaded();
                const auto& ids = gh->getCategoryAchievements(id);
                if (idx < 1 || idx > static_cast<int>(ids.size())) return luaReturnNil(L);
                id = ids[static_cast<size_t>(idx) - 1];
            }
            const std::string& name = gh->getAchievementName(id);
            if (name.empty()) { return luaReturnNil(L); }
            bool completed = gh->getEarnedAchievements().count(id) > 0;
            uint32_t date = gh->getAchievementDate(id);
            uint32_t points = gh->getAchievementPoints(id);
            const std::string& desc = gh->getAchievementDescription(id);
            const game::WowDate earned = game::unpackWowPackedTime(date);
            int month = completed ? earned.month : 0;
            int day   = completed ? earned.day : 0;
            // Short form: SHORTDATE is "%2$d/%1$02d/%3$02d".
            int year  = completed ? earned.yearSince2000 : 0;
            lua_pushnumber(L, id);                 // 1: id
            lua_pushstring(L, name.c_str());       // 2: name
            lua_pushnumber(L, points);             // 3: points
            lua_pushboolean(L, completed ? 1 : 0); // 4: completed
            lua_pushnumber(L, month);              // 5: month
            lua_pushnumber(L, day);                // 6: day
            lua_pushnumber(L, year);               // 7: year
            lua_pushstring(L, desc.c_str());       // 8: description
            lua_pushnumber(L, gh->getAchievementFlags(id)); // 9: flags
            lua_pushstring(L, achievementIconPath(gh, id).c_str()); // 10: icon
            lua_pushstring(L, "");                 // 11: rewardText
            lua_pushboolean(L, 0);                 // 12: isGuildAchievement
            return 12;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
