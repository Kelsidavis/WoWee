// lua_api_helpers.hpp - Shared helpers, lookup tables, and utility functions
// used by all lua_*_api.cpp domain files.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#pragma once

#include <string>
#include <chrono>
#include <cstring>
#include <algorithm>

#include "game/item_text.hpp"
#include "addons/lua_services.hpp"
#include "game/game_handler.hpp"
#include "game/entity.hpp"
#include "game/update_field_table.hpp"
#include "game/inventory_slots.hpp"
#include "game/reputation_standing.hpp"
#include "core/logger.hpp"
#include "core/app_clock.hpp"
#include "ui/widget_tree.hpp"

namespace wowee::addons {

/// The row FrameXML's quest log shows this quest at, or 0 for none.
///
/// Not the quest's index in the log: the displayed list carries a header per
/// zone, and a collapsed group hides its quests entirely, so quest three of
/// the log can be row five of the display or absent from it. Everything that
/// takes a quest log index goes through the same row list this does, which is
/// the only reason the number means anything on the other side.
int questLogRowForQuest(game::GameHandler* gh, uint32_t questId);

/// Put FrameXML's quest log in front of `questId`.
///
/// Asked for by a quest link in chat and by the objective tracker, neither of
/// which has a log of its own to open any more. Without a row the log is
/// opened on whatever it had selected: an approximation of the ask beats
/// nothing happening.
void openInterfaceQuestLog(game::GameHandler& gh, uint32_t questId);

}  // namespace wowee::addons

extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

namespace wowee::addons {

// ---- String helper ----
inline void toLowerInPlace(std::string& s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
}

/// Which interface is loaded, as its manifest's `## Interface:` number - 11200
/// for 1.12, 20400 for 2.4.3, 30300 for 3.3.5a, 40300 for 4.3.4 - or 0 where
/// none said.
///
/// Published by the FrameXML loader before the first file is read, because
/// some of what this client answers has to differ by expansion and the answer
/// is needed while a handler is running. An API whose shape changed between
/// expansions is the case this exists for: answering one interface's shape to
/// another's files is a raise inside a handler, and the half of the handler
/// after it never runs.
///
/// Zero reads as the oldest, which is what the loader does with a manifest
/// that states no version.
inline int interfaceVersion(lua_State* L) {
    lua_getglobal(L, "__WoweeInterfaceVersion");
    const int v = lua_isnumber(L, -1) ? static_cast<int>(lua_tonumber(L, -1)) : 0;
    lua_pop(L, 1);
    return v;
}

// ---- Lua return helpers - used 200+ times as guard/fallback returns ----
inline int luaReturnNil(lua_State* L)  { lua_pushnil(L); return 1; }
inline int luaReturnZero(lua_State* L) { lua_pushnumber(L, 0); return 1; }
inline int luaReturnFalse(lua_State* L){ lua_pushboolean(L, 0); return 1; }

// ---- Lenient numeric argument, for values that came out of a widget ----
//
// FrameXML hands a widget's own output straight to bindings that want a
// number: the auction browse passes BrowseMinLevel:GetText() and
// IsUsableCheckButton:GetChecked() to QueryAuctionItems without touching
// either. luaL_optnumber falls back to its default only for nil and none -
// anything else that will not convert raises instead, and an empty edit box
// gives "", not nil. So an untouched level box did not mean "no minimum": it
// aborted QueryAuctionItems before it sent anything, and the browse tab
// answered every search with no items and no error.
//
// Real WoW's C functions convert what they can and treat the rest as unset,
// which is what lets Blizzard's own code pass raw widget output around.
inline double luaOptNumberText(lua_State* L, int idx, double def) {
    switch (lua_type(L, idx)) {
        case LUA_TNUMBER:  return lua_tonumber(L, idx);
        // lua_isnumber answers for the converted value rather than the stored
        // type, so it is the test for "this string is a number".
        case LUA_TSTRING:  return lua_isnumber(L, idx) ? lua_tonumber(L, idx) : def;
        case LUA_TBOOLEAN: return lua_toboolean(L, idx) ? 1.0 : 0.0;
        default:           return def;  // nil, none, and everything unconvertible
    }
}

// ---- Shared GetTime() epoch ----
//
// The application's clock, not one of its own. Both are steady_clock seconds
// and both fixed their origin on first call, so they differed by however long
// separated those two calls - the app's at startup, this one at the first Lua
// GetTime, which is after the addon system comes up.
//
// That gap landed on every cooldown. GetActionCooldown reports a start time on
// this clock and the widget renderer draws the sweep as
// appTimeSeconds() - start, so the difference was added to every elapsed: a
// sweep ran ahead of itself, and one shorter than the gap was already over
// before it was drawn. Anything else comparing a GetTime value against a
// widget's stored time had the same offset.
inline double luaGetTimeNow() {
    return wowee::core::appTimeSeconds();
}

// ---- Shared WoW class/race/power name tables (indexed by ID, element 0 = unknown) ----
inline constexpr const char* kLuaClasses[] = {
    "","Warrior","Paladin","Hunter","Rogue","Priest",
    "Death Knight","Shaman","Mage","Warlock","","Druid"
};
/// What the cursor is carrying, if anything.
///
/// Shared because two sides need it: the bindings that pick an item up and put
/// it down, and the ones that describe a slot to the interface. A slot whose
/// item is on the cursor is "locked", which is what makes it draw greyed while
/// it is being dragged - and without somewhere common to ask, the binding that
/// answers that question had no way to know.
struct CursorItemSlot {
    int bag = -1;    ///< 0 for the backpack, 1-4 for a worn bag, -1 for none
    int slot = 0;    ///< 1-based within the bag, or the equipment slot
    bool equipped = false;
};
/// Which mouse button the click being dispatched right now belongs to -
/// "LeftButton", "RightButton", or empty between clicks.
///
/// A modified-click binding names a button as well as a modifier:
/// SOCKETITEM is SHIFT-BUTTON2 where CHATLINK is SHIFT-BUTTON1. Answering
/// IsModifiedClick from the modifier alone made every shift-click a socket
/// click as well as a link click, so shift-clicking an item in a bag opened
/// the socketing window.
inline std::string& currentClickButton() {
    static std::string button;
    return button;
}

inline CursorItemSlot& cursorItemSlot() {
    static CursorItemSlot held;
    return held;
}

/// The uppercase tokens WoW returns second from UnitClass, and the key every
/// class-indexed table in FrameXML uses - CLASS_ICON_TCOORDS for the class
/// portrait, RAID_CLASS_COLORS for a name's colour. Written out rather than
/// derived from the display names above, because the token for Death Knight
/// has no space in it.
inline constexpr const char* kLuaClassTokens[] = {
    "","WARRIOR","PALADIN","HUNTER","ROGUE","PRIEST",
    "DEATHKNIGHT","SHAMAN","MAGE","WARLOCK","","DRUID"
};
/// The token for a class id, or nullptr when there is not one.
///
/// Every caller wants the same thing and the same refusal, and getting the
/// refusal wrong cost three panels in a day. The rule is that FrameXML guards
/// these - `if ( classFileName ) then RAID_CLASS_COLORS[classFileName]` - so
/// the guard is the caller saying it already handles the value being missing.
/// Anything truthy-but-wrong walks past it and raises a line later, inside a
/// function that looks unrelated:
///
///   * the numeric id took the guild roster down on its first online member,
///   * the string "UNKNOWN" took Blizzard_ArenaUI down at load,
///   * and the empty string would do the same as either, because "" is truthy
///     in Lua as well - which is why this answers a null pointer and not a
///     blank, and why the two unnamed slots in the table below (ids 0 and 10,
///     which WoW does not use) have to be treated as absent rather than
///     returned.
inline const char* luaClassToken(unsigned classId) {
    if (classId >= (sizeof(kLuaClassTokens) / sizeof(kLuaClassTokens[0]))) {
        return nullptr;
    }
    const char* token = kLuaClassTokens[classId];
    return (token && *token) ? token : nullptr;
}

/// Push that token, or nil. The pairing every caller needs, so that the rule
/// above is applied rather than remembered.
inline void luaPushClassToken(lua_State* L, unsigned classId) {
    if (const char* token = luaClassToken(classId)) lua_pushstring(L, token);
    else                                            lua_pushnil(L);
}

inline constexpr const char* kLuaRaces[] = {
    "","Human","Orc","Dwarf","Night Elf","Undead",
    "Tauren","Gnome","Troll","","Blood Elf","Draenei"
};

/// Whether a quest of this level is grey to a player of that one.
///
/// The threshold is AzerothCore's Acore::XP::GetGrayLevel, which the server
/// uses for the same judgement - `creatureOrQuestLevel <= GetGrayLevel(level)`.
/// Written out rather than guessed at a "level - 8 or so", which is what the
/// comment beside IsActiveQuestTrivial used to decline to do; the formula is
/// three branches and it is the server's own.
inline bool questIsTrivial(int playerLevel, int questLevel) {
    if (questLevel <= 0 || playerLevel <= 0) return false;
    int grey;
    if (playerLevel <= 5)       grey = 0;
    else if (playerLevel <= 39) grey = playerLevel - 5 - playerLevel / 10;
    else if (playerLevel <= 59) grey = playerLevel - 1 - playerLevel / 5;
    else                        grey = playerLevel - 9;
    return questLevel <= grey;
}

/// The five loot methods, indexed by the value the group list carries, and the
/// tokens the interface knows them by. unitpopup.lua does
/// `UnitLootMethod[GetLootMethod()].text` - a table keyed by the token, with
/// .text read straight off the result - so a spelling that misses raises rather
/// than losing a label.
///
/// One table for both directions. It was a switch answering id-to-token in the
/// inventory bindings and an if-chain answering token-to-id in the social ones,
/// which is the same fact written twice: they agreed, and nothing but reading
/// both said so.
inline constexpr const char* kLootMethodTokens[] = {
    "freeforall", "roundrobin", "master", "group", "needbeforegreed"
};
inline constexpr uint8_t kNumLootMethods = 5;

/// The wire value for a token, or zero - free-for-all - for one not known,
/// which is what the if-chain's fall-through did.
inline uint8_t lootMethodFromToken(const std::string& lower) {
    for (uint8_t i = 0; i < kNumLootMethods; ++i)
        if (lower == kLootMethodTokens[i]) return i;
    return 0;
}

/// UnitRace returns the display name first and this *file name* second, and the
/// two differ for four races: the file name never has a space in it, and the
/// Undead one is not the display name at all. It is a file name because that is
/// literally what it is spliced into - DressUpTexturePath builds
/// "Interface\DressUpFrame\DressUpBackground-"..fileName, so "Night Elf" asks
/// for an asset that does not exist and the dressing room draws no background.
/// The -- HACK in dressupframe.lua that rewrites GNOME to Dwarf and TROLL to Orc
/// is the tell: those two are remapped precisely because their backgrounds were
/// never shipped, which fixes the spelling of the ones that were.
inline constexpr const char* kLuaRaceFileNames[] = {
    "","Human","Orc","Dwarf","NightElf","Scourge",
    "Tauren","Gnome","Troll","","BloodElf","Draenei"
};

/// Indexed by power type, and the token FrameXML looks up in PowerBarColor for
/// the bar's colour and in _G for its label prefix. Slot 5 is RUNES: a colour
/// missed here still lands, because PowerBarColor carries numeric aliases as a
/// fallback, but _G[""] is nil and the prefix has no such second chance.
inline constexpr const char* kLuaPowerNames[] = {
    "MANA","RAGE","FOCUS","ENERGY","HAPPINESS","RUNES","RUNIC_POWER"
};

// The quality colours live in item_text.hpp and are not restated here. They
// were written out as two tables in this header - one with the alpha, one
// without - and the pair drifted: this one said heirlooms were 00ccff, a later
// expansion's token colour, so an heirloom link from the Lua side came out
// cyan while the same item shift-clicked out of a bag came out gold. Both
// spellings now come from itemQualityColorHex, and the sixteen places that
// built a link around them call itemChatLink instead.

// ---- Retrieve GameHandler pointer stored in Lua registry ----
inline game::GameHandler* getGameHandler(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_game_handler");
    auto* gh = static_cast<game::GameHandler*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return gh;
}

// ---- Retrieve the widget tree that backs CreateFrame/CreateTexture ----
// Null before the UI is up, which the bindings treat as "record nothing and
// carry on" so an addon loaded early cannot crash on a missing tree.
inline wowee::ui::WidgetTree* getWidgetTree(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_widget_tree");
    auto* t = static_cast<wowee::ui::WidgetTree*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return t;
}

// ---- Retrieve LuaServices pointer stored in Lua registry ----
inline LuaServices* getLuaServices(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_services");
    auto* svc = static_cast<LuaServices*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return svc;
}

// ---- Unit resolution helpers ----

// Read UNIT_FIELD_TARGET_LO/HI from an entity's update fields to get what it's targeting
inline uint64_t getEntityTargetGuid(game::GameHandler* gh, uint64_t guid) {
    if (guid == 0) return 0;
    // If asking for the player's target, use direct accessor
    if (guid == gh->getPlayerGuid()) return gh->getTargetGuid();
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity) return 0;
    const auto& fields = entity->getFields();
    auto loIt = fields.find(game::fieldIndex(game::UF::UNIT_FIELD_TARGET_LO));
    if (loIt == fields.end()) return 0;
    uint64_t targetGuid = loIt->second;
    auto hiIt = fields.find(game::fieldIndex(game::UF::UNIT_FIELD_TARGET_HI));
    if (hiIt != fields.end())
        targetGuid |= (static_cast<uint64_t>(hiIt->second) << 32);
    return targetGuid;
}

// Resolve WoW unit IDs to GUID
inline uint64_t resolveUnitGuid(game::GameHandler* gh, const std::string& uid) {
    if (uid == "player")      return gh->getPlayerGuid();
    if (uid == "target")      return gh->getTargetGuid();
    if (uid == "focus")       return gh->getFocusGuid();
    if (uid == "mouseover")   return gh->getMouseoverGuid();
    if (uid == "pet")         return gh->getPetGuid();
    // The NPC whose window is open, which is how FrameXML titles three of
    // them: GossipFrameNpcNameText and BankFrameTitleText both read
    // UnitName("npc"), and QuestFrameNpcNameText reads UnitName("questnpc").
    // Neither token resolved, so all three came up blank - and the
    // UnitExists() beside each is what decides whether the portrait model is
    // set at all.
    //
    // Whichever window is actually open answers: the quest detail carries the
    // giver it came from, the gossip its own npc, and a vendor or a banker
    // theirs. The target is the last resort, and is usually the same NPC.
    if (uid == "npc" || uid == "questnpc") {
        if (uid == "questnpc" && gh->getQuestDetails().npcGuid != 0)
            return gh->getQuestDetails().npcGuid;
        if (gh->getCurrentGossip().npcGuid != 0) return gh->getCurrentGossip().npcGuid;
        if (gh->getQuestDetails().npcGuid != 0)  return gh->getQuestDetails().npcGuid;
        if (gh->getVendorGuid() != 0)            return gh->getVendorGuid();
        if (gh->getBankerGuid() != 0)            return gh->getBankerGuid();
        return gh->getTargetGuid();
    }
    // Compound unit IDs: targettarget, focustarget, pettarget, mouseovertarget
    if (uid == "targettarget")    return getEntityTargetGuid(gh, gh->getTargetGuid());
    if (uid == "focustarget")     return getEntityTargetGuid(gh, gh->getFocusGuid());
    if (uid == "pettarget")       return getEntityTargetGuid(gh, gh->getPetGuid());
    if (uid == "mouseovertarget") return getEntityTargetGuid(gh, gh->getMouseoverGuid());
    // party1-party4, raid1-raid40
    if (uid.rfind("party", 0) == 0 && uid.size() > 5) {
        int idx = 0;
        try { idx = std::stoi(uid.substr(5)); } catch (...) { return 0; }
        if (idx < 1 || idx > 4) return 0;
        const auto& pd = gh->getPartyData();
        // party members exclude self; index 1-based
        int found = 0;
        for (const auto& m : pd.members) {
            if (m.guid == gh->getPlayerGuid()) continue;
            if (++found == idx) return m.guid;
        }
        return 0;
    }
    // boss1-boss5, the encounter frames. Filled by
    // SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT; without this every UnitExists("boss1")
    // answered nil and the frames stayed hidden through the whole fight.
    if (uid.rfind("boss", 0) == 0 && uid.size() > 4) {
        int idx = 0;
        try { idx = std::stoi(uid.substr(4)); } catch (...) { return 0; }
        if (idx < 1 || idx > static_cast<int>(game::GameHandler::kMaxEncounterSlots)) return 0;
        return gh->getEncounterUnitGuid(static_cast<uint32_t>(idx - 1));
    }
    if (uid.rfind("raid", 0) == 0 && uid.size() > 4 && uid[4] != 'p') {
        int idx = 0;
        try { idx = std::stoi(uid.substr(4)); } catch (...) { return 0; }
        if (idx < 1 || idx > 40) return 0;
        const auto& pd = gh->getPartyData();
        if (idx <= static_cast<int>(pd.members.size()))
            return pd.members[idx - 1].guid;
        return 0;
    }
    return 0;
}

/// A unit token, or the name of somebody standing nearby.
///
/// Several of the interface's verbs take either - "/wave Bob" and "/follow Bob"
/// hand over whatever was typed after the command, and the same argument may be
/// a unit token. resolveUnitGuid knows only the tokens and answers zero for a
/// name, so a binding built on it alone silently used the current target
/// instead of the person named.
///
/// The name is matched against entities in range, which is the only place a
/// name can be resolved from: nothing here can ask the server who a name
/// belongs to without a query it would then have to wait for. Case-insensitive,
/// because the player typed it.
inline uint64_t resolveUnitOrName(game::GameHandler* gh, const std::string& text) {
    if (!gh || text.empty()) return 0;
    std::string lower = text;
    toLowerInPlace(lower);
    if (const uint64_t byToken = resolveUnitGuid(gh, lower)) return byToken;
    // Units only - a name aimed at a doodad is not what anyone meant, and
    // getName lives on the concrete types rather than on Entity.
    for (const auto& [guid, entity] : gh->getEntityManager().getEntities()) {
        if (!entity || entity->getType() == game::ObjectType::GAMEOBJECT) continue;
        auto unit = std::static_pointer_cast<game::Unit>(entity);
        std::string name = unit->getName();
        if (name.empty()) continue;
        toLowerInPlace(name);
        if (name == lower) return guid;
    }
    return 0;
}

/// Where the quest's usable item is sitting, if the player is carrying it.
///
/// Some quests hand you an item to use - a horn, a lantern, a disguise - and
/// the watch frame draws a button for it beside the tracked objective. The item
/// is the quest's start item, which arrives in SMSG_QUEST_QUERY_RESPONSE.
///
/// The bags are searched rather than trusted, because the button should
/// disappear along with the item if it is dropped or used up. An itemId of zero
/// means either that the quest has no such item or that it is no longer held,
/// and the caller cannot usefully tell those apart.
struct QuestSpecialItem {
    int bag = -1;          ///< 0 for the backpack, 1-4 for a worn bag
    int slot = 0;          ///< 1-based within that bag
    uint32_t itemId = 0;
    uint32_t count = 0;
};

inline QuestSpecialItem questSpecialItemAt(game::GameHandler* gh, int questIndex) {
    QuestSpecialItem found;
    if (!gh || questIndex < 1) return found;
    const auto& ql = gh->getQuestLog();
    if (questIndex > static_cast<int>(ql.size())) return found;
    const uint32_t itemId = ql[questIndex - 1].sourceItemId;
    if (itemId == 0) return found;

    const auto& inv = gh->getInventory();
    for (int i = 0; i < inv.getBackpackSize(); ++i) {
        const auto& slot = inv.getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            return {.bag = 0, .slot = i + 1, .itemId = itemId, .count = slot.item.stackCount};
        }
    }
    for (int bag = 0; bag < 4; ++bag) {
        const int size = inv.getBagSize(bag);
        for (int i = 0; i < size; ++i) {
            const auto& slot = inv.getBagSlot(bag, i);
            if (!slot.empty() && slot.item.itemId == itemId) {
                return {.bag = bag + 1, .slot = i + 1, .itemId = itemId, .count = slot.item.stackCount};
            }
        }
    }
    return found;
}

/// The thirteen values GetFactionInfo and GetFactionInfoByID both answer with.
///
/// Two bindings ask the same question - one by position in the reputation list,
/// the other by faction id - and only the by-position one was answered in full.
/// The by-id one pushed a name and five nils, which is the shape that goes
/// unnoticed: a caller reading position nine or eleven gets nil, nil is falsy,
/// and the branch it guards silently takes the other path.
/// The tree flags default to "a plain faction row with a standing", which is
/// what GetFactionInfoByID answers: it is asked about one faction by id and
/// has no row, so it has no place in the tree to report.
inline int pushFactionInfo(lua_State* L, game::GameHandler* gh,
                           const game::GameHandler::ReputationEntry& f,
                           bool isHeader = false, bool isCollapsed = false,
                           bool hasRep = true, bool isChild = false) {
    const int32_t value = gh->getFactionStanding(f.factionId);
    const auto& band = game::reputationStandingFor(value);
    const bool atWar = (f.flags & game::GameHandler::FACTION_FLAG_AT_WAR) != 0;
    const bool peaceForced =
        (f.flags & game::GameHandler::FACTION_FLAG_PEACE_FORCED) != 0;

    lua_pushstring(L, f.name.c_str());                          // 1: name
    lua_pushstring(L, "");                                      // 2: description
    lua_pushnumber(L, band.id);                                 // 3: standingId
    lua_pushnumber(L, band.floor);                              // 4: barMin
    // The bar's top is one past the last value still at this standing, so a
    // faction sitting at the ceiling reads as full rather than as over.
    lua_pushnumber(L, band.ceiling + 1);                        // 5: barMax
    lua_pushnumber(L, value);                                   // 6: barValue
    lua_pushboolean(L, atWar ? 1 : 0);                          // 7: atWarWith
    lua_pushboolean(L, peaceForced ? 0 : 1);                    // 8: canToggleAtWar
    // Whether this row is a heading, and whether it is drawn indented. Both
    // come from Faction.dbc's parent chain, and both used to be false because
    // the rule looked for was the wrong one: a header was taken to be a parent
    // with no reputation of its own, and no faction in the file is that -
    // Alliance is ReputationListID 11, Horde is 12, Classic is 96. But isHeader
    // and hasRep are separate answers precisely so that a heading can carry a
    // standing too, which is what Alliance is. The rule is simply "something
    // visible descends from it", and it yields the groups the original panel
    // draws: Classic, The Burning Crusade and Wrath of the Lich King at the
    // top, with Alliance, Horde, Steamwheedle Cartel and the rest inside them.
    lua_pushboolean(L, isHeader ? 1 : 0);                       // 9: isHeader
    lua_pushboolean(L, isCollapsed ? 1 : 0);                    // 10: isCollapsed
    lua_pushboolean(L, hasRep ? 1 : 0);                         // 11: hasRep
    lua_pushboolean(L, f.factionId == gh->getWatchedFactionId() ? 1 : 0);  // 12: isWatched
    lua_pushboolean(L, isChild ? 1 : 0);                        // 13: isChild
    return 13;
}

/// Open or close the group at a drawn-row index. Shared because the two verbs
/// differ only in the boolean, and because the index has to be resolved the
/// same way both times - against the rows, which is what FrameXML counted.
inline int factionHeaderSetCollapsed(lua_State* L, bool collapsed) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) return 0;
    const auto& rows = gh->getReputationRows();
    if (index > static_cast<int>(rows.size())) return 0;
    const auto& row = rows[static_cast<size_t>(index) - 1];
    if (row.isHeader) gh->setFactionCollapsed(row.factionId, collapsed);
    return 0;
}

/// The same thirteen for a drawn row, which knows its place in the tree.
inline int pushFactionRow(lua_State* L, game::GameHandler* gh,
                          const game::GameHandler::ReputationRow& row) {
    game::GameHandler::ReputationEntry e;
    e.factionId = row.factionId;
    e.reputationIndex = row.reputationIndex;
    e.name = row.name;
    e.flags = row.flags;
    return pushFactionInfo(L, gh, e, row.isHeader,
                           row.isHeader && gh->isFactionCollapsed(row.factionId),
                           row.hasRep, row.isChild);
}

/// The item behind a row of the currency list, or zero for no such row.
///
/// A currency is an item held in the bags - 3.3.5 has no separate store for
/// them - so its name, icon and tooltip are all the item's. Declared here
/// because the tooltip methods live with the widget code and the list is built
/// with the inventory bindings, and building it twice is how two answers to one
/// question start to disagree.
uint32_t currencyListItemId(lua_State* L, int index);

// Resolve unit IDs (player, target, focus, mouseover, pet, targettarget, etc.) to entity
inline game::Unit* resolveUnit(lua_State* L, const char* unitId) {
    auto* gh = getGameHandler(L);
    if (!gh || !unitId) return nullptr;
    std::string uid(unitId);
    toLowerInPlace(uid);

    uint64_t guid = resolveUnitGuid(gh, uid);
    if (guid == 0) return nullptr;
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity) return nullptr;
    return dynamic_cast<game::Unit*>(entity.get());
}

/// A unit's race, whoever the unit is.
///
/// The player's is known outright; anyone else's is byte zero of
/// UNIT_FIELD_BYTES_0, with the name-query cache behind it for a unit whose
/// fields have not arrived. Shared because two bindings need it and a second
/// copy of "where a race comes from" is how they drift.
inline uint8_t unitRaceOf(game::GameHandler* gh, const std::string& lowerUid) {
    if (!gh) return 0;
    if (lowerUid == "player") return gh->getPlayerRace();
    const uint64_t guid = resolveUnitGuid(gh, lowerUid);
    if (guid == 0) return 0;
    uint8_t raceId = 0;
    if (auto entity = gh->getEntityManager().getEntity(guid)) {
        const uint32_t bytes0 =
            entity->getField(game::fieldIndex(game::UF::UNIT_FIELD_BYTES_0));
        raceId = static_cast<uint8_t>(bytes0 & 0xFF);
    }
    if (raceId == 0) raceId = gh->lookupPlayerRace(guid);
    return raceId;
}

// Finish an armed item-target use on the item in a slot, if one is armed
//
// A sharpening stone, a weapon oil, an enchanting scroll and a disenchant all
// park the use and wait for the player to pick the item it applies to. Every
// button that can name an item has to be able to be that pick, and until now
// the only one that could was this client's own bag window - which is handed
// over, so nothing could finish one at all.
//
// True means the click was the target and is therefore not also a use or a
// pickup. An empty slot is not a target: the click is still eaten, because the
// cursor stays armed and dropping out of targeting on a miss would be worse
// than doing nothing.
inline bool completedItemTarget(lua_State* L, uint64_t targetGuid) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isAwaitingItemTarget()) return false;
    if (targetGuid != 0) gh->completeItemUseOnItem(targetGuid);
    return true;
}

// The GUID of the item in a FrameXML container slot: bag 0 is the backpack,
// 1-4 are the worn bags, and both are 1-based on the slot.
/// Whether the repair cursor is up at a vendor.
///
/// Three calls asked and answered this and none of them agreed:
/// ShowRepairCursor and HideRepairCursor were no-ops and InRepairMode was a
/// flat false. So the merchant's repair button never latched - every click
/// took the "not in repair mode" branch and showed the cursor again - and the
/// per-item repair the bags and the paperdoll gate on it was unreachable.
inline bool& repairCursorUp() {
    static bool up = false;
    return up;
}

/// The repair click, when there is one: true when this item was repaired here
/// and the caller should do nothing else with it.
///
/// The vendor being open is required rather than assumed. HideRepairCursor is
/// only called from the button itself, so closing the window while the cursor
/// is up leaves the flag set - and a left-click in the bags afterwards would
/// otherwise try to repair against a vendor that is no longer there.
inline bool repairedHeldItem(game::GameHandler* gh, uint64_t itemGuid) {
    if (!gh || !repairCursorUp() || !itemGuid) return false;
    const uint64_t vendor = gh->getVendorGuid();
    if (!vendor || !gh->isVendorWindowOpen()) return false;
    gh->repairItem(vendor, itemGuid);
    return true;
}

/// The keyring, which FrameXML addresses as a container like any other.
///
/// constants.lua sets KEYRING_CONTAINER = -2, and containerframe.lua builds the
/// keyring frame by asking GetContainerNumSlots(-2) and then walking the slots.
/// Every container binding here branched on 0 for the backpack and 1 to 4 for
/// the worn bags and let everything else fall through to zero, so the keyring
/// opened with no slots at all - while the keys themselves were being tracked
/// the whole time, out of PLAYER_FIELD_KEYRING_SLOT_1.
constexpr int kKeyringContainer = -2;

/// The general bank, which the interface names as a container even though its
/// slots are inventory slots on the wire. bankframe.lua reaches its
/// twenty-eight buttons both ways: the texture and the count come from
/// GetInventoryItemTexture by inventory slot id, and the link, the quest info,
/// the cooldown, the split, the pickup and the right-click all come through
/// the container calls with BANK_CONTAINER.
constexpr int kBankContainer = -1;

/// The seven bank bags, which the interface numbers straight on from the four
/// worn ones: NUM_BAG_SLOTS + 1 through NUM_BAG_SLOTS + NUM_BANKBAGSLOTS, so 5
/// to 11. Each is a container of its own and containerframe.lua opens them
/// exactly as it opens a worn bag.
constexpr int kFirstBankBagContainer = 5;
constexpr int kBankBagContainers = 7;
inline bool isBankBagContainer(int c) {
    return c >= kFirstBankBagContainer && c < kFirstBankBagContainer + kBankBagContainers;
}

/// How many slots a container has, in the interface's numbering.
inline int containerSlotCount(const game::Inventory& inv, int container) {
    if (container == 0) return inv.getBackpackSize();
    if (container >= 1 && container <= 4) return inv.getBagSize(container - 1);
    if (container == kKeyringContainer) return inv.getKeyringSize();
    if (container == kBankContainer) return game::Inventory::BANK_SLOTS;
    if (isBankBagContainer(container))
        return inv.getBankBagSize(container - kFirstBankBagContainer);
    return 0;
}

/// One slot of one container, or nullptr when there is no such slot. `slot` is
/// 1-based, as every container binding receives it from Lua.
inline const game::ItemSlot* containerItemSlot(const game::Inventory& inv,
                                               int container, int slot) {
    if (slot < 1 || slot > containerSlotCount(inv, container)) return nullptr;
    if (container == 0) return &inv.getBackpackSlot(slot - 1);
    if (container >= 1 && container <= 4) return &inv.getBagSlot(container - 1, slot - 1);
    if (container == kKeyringContainer) return &inv.getKeyringSlot(slot - 1);
    if (container == kBankContainer) return &inv.getBankSlot(slot - 1);
    if (isBankBagContainer(container))
        return &inv.getBankBagSlot(container - kFirstBankBagContainer, slot - 1);
    return nullptr;
}

/// One slot named by an *inventory* slot id, the interface's 1-based numbering.
///
/// Equipment is 1 to 23 and everything above it was answered as absent, which
/// covered the paperdoll and nothing else. The bank is addressed this way:
/// bankframe.lua draws its twenty-eight general slots by asking
/// GetInventoryItemTexture("player", BankButtonIDToInvSlotID(id)), and those
/// ids land at 40 and up - so every one of them read empty while
/// inventory_handler filled bankSlots_ from the update fields.
inline const game::ItemSlot* inventorySlotItem(const game::Inventory& inv, int slotId) {
    const int wire = game::slots::toWireSlot(slotId);
    if (slotId >= 1 && slotId <= static_cast<int>(game::EquipSlot::NUM_SLOTS)) {
        return &inv.getEquipSlot(static_cast<game::EquipSlot>(slotId - 1));
    }
    if (wire >= game::slots::kBankGeneralFirst &&
        wire <  game::slots::kBankGeneralFirst + game::slots::kBankGeneralCount) {
        return &inv.getBankSlot(wire - game::slots::kBankGeneralFirst);
    }
    // The bag a bank bag *is*, not what is inside it - the seven slots the
    // bank's own bag row draws.
    if (wire >= game::slots::kBankBagFirst &&
        wire <  game::slots::kBankBagFirst + game::slots::kBankBagCount) {
        return &inv.getBankBagItem(wire - game::slots::kBankBagFirst);
    }
    return nullptr;
}

/// Where a container slot lives on the wire: the container byte and the slot
/// within it.
///
/// The backpack, the keyring and the general bank are all slots of the player's
/// own container, 0xFF, at three different offsets; a worn bag and a bank bag
/// are containers in their own right, numbered by the slot the bag sits in.
/// Written out at each call site this cost two fixes in a day - the keyring
/// asked wornBagContainer for container -3, and the first bank bag came out as
/// container 23.
inline uint8_t containerWireBag(int bag) {
    if (bag == 0 || bag == kKeyringContainer || bag == kBankContainer) return game::slots::kNoContainer;
    if (isBankBagContainer(bag))
        return static_cast<uint8_t>(game::slots::bankBagContainer(bag - kFirstBankBagContainer));
    return static_cast<uint8_t>(game::slots::wornBagContainer(bag - 1));
}

inline uint8_t containerWireSlot(int bag, int slot) {
    if (bag == 0)                  return static_cast<uint8_t>(game::slots::backpackWireSlot(slot - 1));
    if (bag == kKeyringContainer)  return static_cast<uint8_t>(game::slots::keyringWireSlot(slot - 1));
    if (bag == kBankContainer)     return static_cast<uint8_t>(game::slots::bankGeneralWireSlot(slot - 1));
    return static_cast<uint8_t>(slot - 1);
}

/// The spell in a book slot, or zero. One place, because four functions below
/// all begin by asking the same question of the same two arguments.
inline uint32_t spellIdForBookSlot(game::GameHandler* gh, int slot) {
    if (!gh || slot < 1) return 0;
    int idx = slot;
    for (const auto& tab : gh->getSpellBookTabs()) {
        if (idx <= static_cast<int>(tab.spellIds.size())) return tab.spellIds[idx - 1];
        idx -= static_cast<int>(tab.spellIds.size());
    }
    return 0;
}

/// The spell a call means, in either of the two forms the client accepts.
///
/// GetSpellTexture, GetSpellCooldown and GetSpellLink are each overloaded:
/// **one** argument is a spell id or a spell name, **two** are a book *slot*
/// and the book holding it. Only the second form is ever used by the
/// spellbook - SpellBook_GetSpellID hands its buttons a slot, never an id.
///
/// Read as an id regardless, a slot of 1, 2, 3 resolved to whatever spells
/// happen to carry those ids, which is why the spellbook drew a page of
/// unrelated icons. The names beside them were right the whole time, because
/// GetSpellName already took the slot form.
inline uint32_t spellIdForCall(lua_State* L, game::GameHandler* gh) {
    if (!gh) return 0;
    // A book beside a number is a slot. A book beside a *name* is not a form
    // the client has, but falling through to the name lookup is free and
    // beats raising out of a caller that only wanted an icon.
    if (!lua_isnoneornil(L, 2) && lua_isnumber(L, 1)) {
        const int slot = static_cast<int>(lua_tonumber(L, 1));
        // The pet book is a list of its own, not a tab in the player's.
        // Resolving a pet slot through the player's tabs answers with one of
        // the player's own spells - a wrong answer that looks like a right
        // one, which is worse than none.
        const char* book = lua_tostring(L, 2);
        if (book && std::string(book) == "pet") {
            const auto& pet = gh->getPetSpells();
            if (slot < 1 || slot > static_cast<int>(pet.size())) return 0;
            return pet[static_cast<size_t>(slot - 1)];
        }
        return spellIdForBookSlot(gh, slot);
    }
    if (lua_isnumber(L, 1)) return static_cast<uint32_t>(lua_tonumber(L, 1));
    const char* name = lua_tostring(L, 1);
    if (!name || !*name) return 0;
    std::string nameLow(name);
    toLowerInPlace(nameLow);
    for (uint32_t sid : gh->getKnownSpells()) {
        std::string sn = gh->getSpellName(sid);
        toLowerInPlace(sn);
        if (sn == nameLow) return sid;
    }
    return 0;
}

inline uint64_t containerSlotGuid(game::GameHandler* gh, int bag, int slot) {
    if (!gh || slot < 1) return 0;
    if (bag == 0) return gh->getBackpackItemGuid(slot - 1);
    if (bag >= 1 && bag <= 4) return gh->getBagItemGuid(bag - 1, slot - 1);
    // The keyring and the bank bags keep their guid on the slot rather than in
    // a side table, so these read it from the inventory directly.
    if (bag == kKeyringContainer || bag == kBankContainer || isBankBagContainer(bag)) {
        const auto* s = containerItemSlot(gh->getInventory(), bag, slot);
        return s ? s->item.guid : 0;
    }
    return 0;
}

/// An item's cooldown, which is its on-use spell's.
///
/// The client tracks cooldowns per spell, and an item on cooldown is one whose
/// use spell is - the same relationship dispatchUseItem walks to decide what
/// using it casts. The original duration is now kept beside the remaining time,
/// so the sweep is wound back to where it actually began: reporting it as
/// starting *now* and lasting what is left drew the right arc only until the
/// interface asked a second time, at which point the swirl jumped back to full
/// and unwound again. BAG_UPDATE_COOLDOWN fires on every use, so it asked often.
///
/// False when the item has no use spell or that spell is ready.
inline bool itemUseCooldown(game::GameHandler* gh, uint32_t itemId,
                            double& start, double& duration) {
    if (!gh || itemId == 0) return false;
    const auto* info = gh->getItemInfo(itemId);
    if (!info || !info->valid) return false;
    for (const auto& sp : info->spells) {
        if (sp.spellId == 0 || (sp.spellTrigger != 0 && sp.spellTrigger != 5)) continue;
        const auto& cds = gh->getSpellCooldowns();
        auto it = cds.find(sp.spellId);
        if (it == cds.end() || it->second <= 0.0f) continue;
        double total = gh->getSpellCooldownTotal(sp.spellId);
        if (total < it->second) total = it->second;
        start = luaGetTimeNow() - (total - it->second);
        duration = total;
        return true;
    }
    return false;
}

// Find GroupMember data for a GUID (for party members out of entity range)
inline const game::GroupMember* findPartyMember(game::GameHandler* gh, uint64_t guid) {
    if (!gh || guid == 0) return nullptr;
    for (const auto& m : gh->getPartyData().members) {
        if (m.guid == guid && m.hasPartyStats) return &m;
    }
    return nullptr;
}

/// The talent at a position in the tree, by the tab and index FrameXML counts
/// in. Defined in lua_quest_api.cpp, which is where everything else about
/// talents lives.
///
/// Declared here because the tooltip setters are in lua_engine.cpp and need the
/// same lookup. Copying it there would mean two answers to "which talent is
/// this", and the pair would agree only for as long as nobody touched either -
/// the tab ordering and the row/column sort both have to match exactly or the
/// tooltip describes a different talent from the one under the cursor.
/// The talent at a position in a class's tree. classIdOverride is zero for the
/// player's own class, or the inspected player's when their tree is being drawn.
const game::TalentEntry* talentAt(game::GameHandler* gh, int tabIndex, int talentIndex,
                                  uint8_t classIdOverride = 0);

/// The item id the cursor is carrying, or zero for anything else.
///
/// The cursor state lives in lua_action_api.cpp, where everything that picks
/// something up and puts it down again is. This is the one thing another file
/// needs from it: CursorCanGoInSlot has to know what is being dragged before
/// it can say which paperdoll slot should light up.
uint32_t cursorItemId();

/// Money picked up onto the cursor, in copper, and zero when there is none.
///
/// A drag of money is routed entirely by the interface: the frame it is
/// dropped on reads the amount, puts it wherever it belongs - a mail's money
/// field, a guild bank deposit, an auction bid - and then clears the cursor.
/// So the whole of the client's part is holding the number.
///
/// Declared beside cursorItemId because it is the same cursor: picking money
/// up has to displace an item and ClearCursor has to drop both.
uint64_t cursorMoney();
void setCursorMoney(lua_State* L, uint64_t copper);

/// The paperdoll slot an item on the cursor was picked up from, or zero when it
/// came from a bag or nothing is held. One-based, as FrameXML numbers slots.
int cursorEquipSlot();

/// Whether the cursor is carrying an item that came out of the player's own
/// inventory - a bag slot or a worn slot - rather than off an action bar.
///
/// The two are let go over the world for opposite reasons. An item on a bar is
/// a reference to one, so dropping it there takes it off the bar and the cursor
/// is simply put down; an item out of a bag is the item, and letting it go over
/// the world is how it is destroyed. Only the second raises the delete prompt,
/// and the prompt reads the cursor - so the drag has to end with the item still
/// on it.
bool cursorItemCameFromInventory();

/// Put a vendor's list entry on the cursor, which is what a left-click in
/// FrameXML's merchant window does. Buying happens when it is dropped.
void pickupMerchantItem(lua_State* L, int index);

/// Pick up part of a stack: the same pickup a whole one does, with the amount
/// riding on the cursor until the drop names a destination.
///
/// What WoW's SplitContainerItem means. Splitting to a slot chosen here instead
/// left the drop with nothing to carry, so part of a stack could never be moved
/// anywhere through the interface.
void pickupSplitFromContainer(lua_State* L, game::GameHandler* gh,
                              int bag, int slot, int count);

/// Buy whatever vendor entry the cursor is holding and put the cursor down.
/// False when it is holding something else, so a caller can fall through to
/// its normal handling.
bool boughtHeldMerchantItem(lua_State* L);

/// The quest at a quest log index, or null when that row is a zone header or
/// the index is off either end.
///
/// A quest log index is a DISPLAY index: the log is grouped under zone headers
/// and the headers are rows too, so index N is not the Nth entry of
/// getQuestLog(). Anything handed an index by FrameXML - including the
/// selected index, which the log sets from a row it drew - has to come through
/// here rather than subscripting the log, or it answers about a different
/// quest than the one asked about.
const game::GameHandler::QuestLogEntry* questAtLogRow(game::GameHandler* gh, int index);

/// Let the trade skill row list be rebuilt on the next read.
///
/// The list is held still between redraws: the frame reads it many times to
/// draw one selection, and its shape depends on item data that arrives while
/// those reads run, so rebuilding per call changed what an index meant halfway
/// through. Called when the game says the trade skill data moved.
void invalidateTradeSkillRows();

/// The team id at an arena team index, or zero when the index is off the end.
///
/// One-based, as the interface counts: ArenaTeamRoster(1) is the first team.
/// Two files had this, under two names, each with its own off-by-one to get
/// right - and an index convention is the kind of thing that goes wrong once
/// and is then copied.
uint32_t arenaTeamIdAtIndex(game::GameHandler* gh, int index);

} // namespace wowee::addons
