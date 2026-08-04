// lua_quest_api.cpp — Quest log, skills, talents, glyphs, and achievements Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "addons/lua_api_helpers.hpp"
#include "game/packed_time.hpp"

namespace wowee::addons {

static int lua_GetNumQuestLogEntries(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
    const auto& ql = gh->getQuestLog();
    lua_pushnumber(L, ql.size());  // numEntries
    lua_pushnumber(L, 0);          // numQuests (headers not tracked)
    return 2;
}

// GetQuestLogTitle(index) → title, level, suggestedGroup, isHeader, isCollapsed, isComplete, frequency, questID
static int lua_GetQuestLogTitle(lua_State* L) {
    auto* gh = getGameHandler(L);
    // optnumber, not checknumber: FrameXML walks the quest log with an index
    // that can be nil before anything has been selected, and raising there
    // loses whatever asked rather than answering that there is no such quest.
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& ql = gh->getQuestLog();
    if (index > static_cast<int>(ql.size())) { return luaReturnNil(L); }
    const auto& q = ql[index - 1];  // 1-based
    lua_pushstring(L, q.title.c_str());  // title
    lua_pushnumber(L, 0);                // level (not tracked)
    lua_pushnumber(L, 0);                // suggestedGroup
    lua_pushboolean(L, 0);               // isHeader
    lua_pushboolean(L, 0);               // isCollapsed
    lua_pushboolean(L, q.complete);      // isComplete
    lua_pushnumber(L, 0);                // frequency
    lua_pushnumber(L, q.questId);        // questID
    return 8;
}

// GetQuestLogQuestText(index) → description, objectives
static int lua_GetQuestLogQuestText(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& ql = gh->getQuestLog();
    if (index > static_cast<int>(ql.size())) { return luaReturnNil(L); }
    const auto& q = ql[index - 1];
    lua_pushstring(L, "");                    // description (not stored)
    lua_pushstring(L, q.objectives.c_str());  // objectives
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

// SelectQuestLogEntry(index) — select a quest in the quest log
static int lua_SelectQuestLogEntry(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (gh) gh->setSelectedQuestLogIndex(index);
    return 0;
}

// GetQuestLogSelection() → index
static int lua_GetQuestLogSelection(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getSelectedQuestLogIndex() : 0);
    return 1;
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
    const auto& ql = gh->getQuestLog();
    const auto& tracked = gh->getTrackedQuestIds();
    int found = 0;
    for (size_t i = 0; i < ql.size(); ++i) {
        if (tracked.count(ql[i].questId)) {
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

// AddQuestWatch(questLogIndex) — add a quest to the watch list
static int lua_AddQuestWatch(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) return 0;
    const auto& ql = gh->getQuestLog();
    if (index <= static_cast<int>(ql.size())) {
        gh->setQuestTracked(ql[index - 1].questId, true);
    }
    return 0;
}

// RemoveQuestWatch(questLogIndex) — remove a quest from the watch list
static int lua_RemoveQuestWatch(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) return 0;
    const auto& ql = gh->getQuestLog();
    if (index <= static_cast<int>(ql.size())) {
        gh->setQuestTracked(ql[index - 1].questId, false);
    }
    return 0;
}

// IsQuestWatched(questLogIndex) → boolean
static int lua_IsQuestWatched(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnFalse(L); }
    const auto& ql = gh->getQuestLog();
    if (index <= static_cast<int>(ql.size())) {
        lua_pushboolean(L, gh->isQuestTracked(ql[index - 1].questId) ? 1 : 0);
    } else {
        lua_pushboolean(L, 0);
    }
    return 1;
}

// GetQuestLink(questLogIndex) → "|cff...|Hquest:id:level|h[title]|h|r"
static int lua_GetQuestLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& ql = gh->getQuestLog();
    if (index > static_cast<int>(ql.size())) { return luaReturnNil(L); }
    const auto& q = ql[index - 1];
    // Yellow quest link format matching WoW
    std::string link = "|cff808000|Hquest:" + std::to_string(q.questId) +
                       ":0|h[" + q.title + "]|h|r";
    lua_pushstring(L, link.c_str());
    return 1;
}

// GetNumQuestLeaderBoards(questLogIndex) → count of objectives
static int lua_GetNumQuestLeaderBoards(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnZero(L); }
    const auto& ql = gh->getQuestLog();
    if (index > static_cast<int>(ql.size())) { return luaReturnZero(L); }
    const auto& q = ql[index - 1];
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
static int lua_GetQuestLogLeaderBoard(lua_State* L) {
    auto* gh = getGameHandler(L);
    int objIdx = static_cast<int>(luaL_checknumber(L, 1));
    int questIdx = static_cast<int>(luaL_optnumber(L, 2,
        gh ? gh->getSelectedQuestLogIndex() : 0));
    if (!gh || questIdx < 1 || objIdx < 1) { return luaReturnNil(L); }
    const auto& ql = gh->getQuestLog();
    if (questIdx > static_cast<int>(ql.size())) { return luaReturnNil(L); }
    const auto& q = ql[questIdx - 1];

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

// ExpandQuestHeader / CollapseQuestHeader — no-ops (flat quest list, no headers)
static int lua_ExpandQuestHeader(lua_State* L) { (void)L; return 0; }
static int lua_CollapseQuestHeader(lua_State* L) { (void)L; return 0; }

// GetQuestLogSpecialItemInfo(questLogIndex) — returns nil (no special items)
static int lua_GetQuestLogSpecialItemInfo(lua_State* L) { (void)L; lua_pushnil(L); return 1; }

static int lua_GetNumSkillLines(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    lua_pushnumber(L, gh->getPlayerSkills().size());
    return 1;
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
    // server has sent any skills there is nothing selected — so a bare nil
    // there is arithmetic on nothing and the file is lost. An empty row is the
    // truthful answer and it costs no more than a blank line in the list.
    // Short-circuit rather than a ternary: binding a reference across both arms
    // would copy the whole skill map on every call.
    if (!gh || index < 1 ||
        index > static_cast<int>(gh->getPlayerSkills().size())) {
        lua_pushstring(L, "");                          // 1: skillName
        lua_pushboolean(L, 0);                          // 2: isHeader
        lua_pushboolean(L, 1);                          // 3: isExpanded
        for (int i = 4; i <= 7; ++i) lua_pushnumber(L, 0);   // rank, temp, mod, max
        lua_pushboolean(L, 0);                          // 8: isAbandonable
        for (int i = 9; i <= 12; ++i) lua_pushnumber(L, 0); // costs, minLevel, type
        return 12;
    }
    // Skills are in a map — iterate to the Nth entry
    const auto& skills = gh->getPlayerSkills();
    auto it = skills.begin();
    std::advance(it, index - 1);
    const auto& skill = it->second;
    std::string name = gh->getSkillName(skill.skillId);
    if (name.empty()) name = "Skill " + std::to_string(skill.skillId);

    lua_pushstring(L, name.c_str());                    // 1: skillName
    lua_pushboolean(L, 0);                              // 2: isHeader (false — flat list)
    lua_pushboolean(L, 1);                              // 3: isExpanded
    lua_pushnumber(L, skill.effectiveValue());           // 4: skillRank
    lua_pushnumber(L, skill.bonusTemp);                  // 5: numTempPoints
    lua_pushnumber(L, skill.bonusPerm);                  // 6: skillModifier
    lua_pushnumber(L, skill.maxValue);                   // 7: skillMaxRank
    lua_pushboolean(L, 0);                              // 8: isAbandonable
    lua_pushnumber(L, 0);                               // 9: stepCost
    lua_pushnumber(L, 0);                               // 10: rankCost
    lua_pushnumber(L, 0);                               // 11: minLevel
    lua_pushnumber(L, 0);                               // 12: skillCostType
    return 12;
}

// --- Friends/Ignore API ---


static int lua_GetNumTalentTabs(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    // Count tabs matching the player's class
    uint8_t classId = gh->getPlayerClass();
    uint32_t classMask = (classId > 0) ? (1u << (classId - 1)) : 0;
    int count = 0;
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) count++;
    }
    lua_pushnumber(L, count);
    return 1;
}

// GetTalentTabInfo(tabIndex) → name, iconTexture, pointsSpent, background
static int lua_GetTalentTabInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIndex = static_cast<int>(luaL_checknumber(L, 1)); // 1-indexed
    if (!gh || tabIndex < 1) {
        return luaReturnNil(L);
    }
    uint8_t classId = gh->getPlayerClass();
    uint32_t classMask = (classId > 0) ? (1u << (classId - 1)) : 0;
    // Find the Nth tab for this class (sorted by orderIndex)
    std::vector<const game::GameHandler::TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) classTabs.push_back(&tab);
    }
    std::sort(classTabs.begin(), classTabs.end(),
        [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });
    if (tabIndex > static_cast<int>(classTabs.size())) {
        return luaReturnNil(L);
    }
    const auto* tab = classTabs[tabIndex - 1];
    // Count points spent in this tab
    int pointsSpent = 0;
    const auto& learned = gh->getLearnedTalents();
    for (const auto& [talentId, rank] : learned) {
        const auto* entry = gh->getTalentEntry(talentId);
        if (entry && entry->tabId == tab->tabId) pointsSpent += rank;
    }
    lua_pushstring(L, tab->name.c_str());              // 1: name
    lua_pushnil(L);                                     // 2: iconTexture (not resolved)
    lua_pushnumber(L, pointsSpent);                     // 3: pointsSpent
    lua_pushstring(L, tab->backgroundFile.c_str());     // 4: background
    return 4;
}

// GetNumTalents(tabIndex) → count
static int lua_GetNumTalents(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIndex = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || tabIndex < 1) { return luaReturnZero(L); }
    uint8_t classId = gh->getPlayerClass();
    uint32_t classMask = (classId > 0) ? (1u << (classId - 1)) : 0;
    std::vector<const game::GameHandler::TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) classTabs.push_back(&tab);
    }
    std::sort(classTabs.begin(), classTabs.end(),
        [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });
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
static int lua_GetTalentInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIndex = static_cast<int>(luaL_checknumber(L, 1));
    int talentIndex = static_cast<int>(luaL_checknumber(L, 2));
    if (!gh || tabIndex < 1 || talentIndex < 1) {
        for (int i = 0; i < 8; i++) lua_pushnil(L);
        return 8;
    }
    uint8_t classId = gh->getPlayerClass();
    uint32_t classMask = (classId > 0) ? (1u << (classId - 1)) : 0;
    std::vector<const game::GameHandler::TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : gh->getAllTalentTabs()) {
        if (tab.classMask & classMask) classTabs.push_back(&tab);
    }
    std::sort(classTabs.begin(), classTabs.end(),
        [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });
    if (tabIndex > static_cast<int>(classTabs.size())) {
        for (int i = 0; i < 8; i++) lua_pushnil(L);
        return 8;
    }
    uint32_t targetTabId = classTabs[tabIndex - 1]->tabId;
    // Collect talents for this tab, sorted by row then column
    std::vector<const game::GameHandler::TalentEntry*> tabTalents;
    for (const auto& [talentId, entry] : gh->getAllTalents()) {
        if (entry.tabId == targetTabId) tabTalents.push_back(&entry);
    }
    std::sort(tabTalents.begin(), tabTalents.end(),
        [](const auto* a, const auto* b) {
            return (a->row != b->row) ? a->row < b->row : a->column < b->column;
        });
    if (talentIndex > static_cast<int>(tabTalents.size())) {
        for (int i = 0; i < 8; i++) lua_pushnil(L);
        return 8;
    }
    const auto* talent = tabTalents[talentIndex - 1];
    uint8_t rank = gh->getTalentRank(talent->talentId);
    // Get spell name for rank 1 spell
    std::string name = gh->getSpellName(talent->rankSpells[0]);
    if (name.empty()) name = "Talent " + std::to_string(talent->talentId);

    lua_pushstring(L, name.c_str());          // 1: name
    lua_pushnil(L);                            // 2: iconTexture
    lua_pushnumber(L, talent->row + 1);        // 3: tier (1-indexed)
    lua_pushnumber(L, talent->column + 1);     // 4: column (1-indexed)
    lua_pushnumber(L, rank);                   // 5: rank
    lua_pushnumber(L, talent->maxRank);        // 6: maxRank
    lua_pushboolean(L, 0);                     // 7: isExceptional
    lua_pushboolean(L, 1);                     // 8: available
    return 8;
}

static int lua_GetActiveTalentGroup(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? (gh->getActiveTalentSpec() + 1) : 1);
    return 1;
}

void registerQuestLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"GetNumQuestLogEntries",   lua_GetNumQuestLogEntries},
                {"GetQuestLogTitle",        lua_GetQuestLogTitle},
                {"GetQuestLogQuestText",    lua_GetQuestLogQuestText},
                {"IsQuestComplete",         lua_IsQuestComplete},
                {"SelectQuestLogEntry",     lua_SelectQuestLogEntry},
                {"GetQuestLogSelection",    lua_GetQuestLogSelection},
                {"GetNumQuestWatches",      lua_GetNumQuestWatches},
                {"GetQuestIndexForWatch",   lua_GetQuestIndexForWatch},
                {"AddQuestWatch",           lua_AddQuestWatch},
                {"RemoveQuestWatch",        lua_RemoveQuestWatch},
                {"IsQuestWatched",          lua_IsQuestWatched},
                {"GetQuestLink",            lua_GetQuestLink},
                {"GetNumQuestLeaderBoards", lua_GetNumQuestLeaderBoards},
                {"GetQuestLogLeaderBoard",  lua_GetQuestLogLeaderBoard},
                {"ExpandQuestHeader",       lua_ExpandQuestHeader},
                {"CollapseQuestHeader",     lua_CollapseQuestHeader},
                {"GetQuestLogSpecialItemInfo", lua_GetQuestLogSpecialItemInfo},
                {"GetNumSkillLines",        lua_GetNumSkillLines},
                {"GetSkillLineInfo",        lua_GetSkillLineInfo},
                {"GetNumTalentTabs",        lua_GetNumTalentTabs},
                {"GetTalentTabInfo",        lua_GetTalentTabInfo},
                {"GetNumTalents",           lua_GetNumTalents},
                {"GetTalentInfo",           lua_GetTalentInfo},
                {"GetActiveTalentGroup",    lua_GetActiveTalentGroup},
                {"AcceptQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->acceptQuest();
            return 0;
        }},
                {"DeclineQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->declineQuest();
            return 0;
        }},
                {"CompleteQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->completeQuest();
            return 0;
        }},
                {"AbandonQuest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            uint32_t questId = static_cast<uint32_t>(luaL_checknumber(L, 1));
            if (gh) gh->abandonQuest(questId);
            return 0;
        }},
                {"GetNumQuestRewards", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            int idx = gh->getSelectedQuestLogIndex();
            if (idx < 1) { return luaReturnZero(L); }
            const auto& ql = gh->getQuestLog();
            if (idx > static_cast<int>(ql.size())) { return luaReturnZero(L); }
            int count = 0;
            for (const auto& r : ql[idx-1].rewardItems)
                if (r.itemId != 0) ++count;
            lua_pushnumber(L, count);
            return 1;
        }},
                {"GetNumQuestChoices", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            int idx = gh->getSelectedQuestLogIndex();
            if (idx < 1) { return luaReturnZero(L); }
            const auto& ql = gh->getQuestLog();
            if (idx > static_cast<int>(ql.size())) { return luaReturnZero(L); }
            int count = 0;
            for (const auto& r : ql[idx-1].rewardChoiceItems)
                if (r.itemId != 0) ++count;
            lua_pushnumber(L, count);
            return 1;
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
                lua_pushnumber(L, glyphId);     // glyphSpellID
                lua_pushstring(L, "Interface\\Icons\\INV_Glyph_MajorWarrior"); // placeholder icon
            } else {
                lua_pushnil(L);
                lua_pushnil(L);
            }
            return 4;
        }},
                {"GetNumCompletedAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getEarnedAchievements().size() : 0);
            return 1;
        }},
                {"GetAchievementInfo", [](lua_State* L) -> int {
            // GetAchievementInfo(id) → id, name, points, completed, month, day, year, description, flags, icon, rewardText, isGuildAch
            auto* gh = getGameHandler(L);
            uint32_t id = static_cast<uint32_t>(luaL_checknumber(L, 1));
            if (!gh) { return luaReturnNil(L); }
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
            lua_pushnumber(L, 0);                  // 9: flags
            lua_pushstring(L, "Interface\\Icons\\Achievement_General"); // 10: icon
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
