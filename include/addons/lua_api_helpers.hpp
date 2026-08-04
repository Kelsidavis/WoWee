// lua_api_helpers.hpp — Shared helpers, lookup tables, and utility functions
// used by all lua_*_api.cpp domain files.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#pragma once

#include <string>
#include <chrono>
#include <cstring>
#include <algorithm>

#include "addons/lua_services.hpp"
#include "game/game_handler.hpp"
#include "game/entity.hpp"
#include "game/update_field_table.hpp"
#include "core/logger.hpp"
#include "core/app_clock.hpp"
#include "ui/widget_tree.hpp"

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

// ---- Lua return helpers — used 200+ times as guard/fallback returns ----
inline int luaReturnNil(lua_State* L)  { lua_pushnil(L); return 1; }
inline int luaReturnZero(lua_State* L) { lua_pushnumber(L, 0); return 1; }
inline int luaReturnFalse(lua_State* L){ lua_pushboolean(L, 0); return 1; }

// ---- Shared GetTime() epoch ----
//
// The application's clock, not one of its own. Both are steady_clock seconds
// and both fixed their origin on first call, so they differed by however long
// separated those two calls — the app's at startup, this one at the first Lua
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
inline constexpr const char* kLuaRaces[] = {
    "","Human","Orc","Dwarf","Night Elf","Undead",
    "Tauren","Gnome","Troll","","Blood Elf","Draenei"
};
inline constexpr const char* kLuaPowerNames[] = {
    "MANA","RAGE","FOCUS","ENERGY","HAPPINESS","","RUNIC_POWER"
};

// ---- Quality hex strings ----
// No alpha prefix — for item links
inline constexpr const char* kQualHexNoAlpha[] = {
    "9d9d9d","ffffff","1eff00","0070dd","a335ee","ff8000","e6cc80","e6cc80"
};
// With ff alpha prefix — for Lua color returns
inline constexpr const char* kQualHexAlpha[] = {
    "ff9d9d9d","ffffffff","ff1eff00","ff0070dd","ffa335ee","ffff8000","ffe6cc80","ff00ccff"
};

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

// Find GroupMember data for a GUID (for party members out of entity range)
inline const game::GroupMember* findPartyMember(game::GameHandler* gh, uint64_t guid) {
    if (!gh || guid == 0) return nullptr;
    for (const auto& m : gh->getPartyData().members) {
        if (m.guid == guid && m.hasPartyStats) return &m;
    }
    return nullptr;
}

} // namespace wowee::addons
