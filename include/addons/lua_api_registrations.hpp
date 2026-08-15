// lua_api_registrations.hpp - Forward declarations for per-domain Lua API
// registration functions.  Called from LuaEngine::registerCoreAPI().
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#pragma once

struct lua_State;

namespace wowee::addons {

void registerUnitLuaAPI(lua_State* L);
void registerSpellLuaAPI(lua_State* L);
void registerInventoryLuaAPI(lua_State* L);
void registerQuestLuaAPI(lua_State* L);
void registerLfgLuaAPI(lua_State* L);
void registerSocketLuaAPI(lua_State* L);
void registerSocialLuaAPI(lua_State* L);
void registerSystemLuaAPI(lua_State* L);

/// Re-apply every CVar restored from disk, now that the services behind them
/// exist. See applyCVarSideEffects.
void applyStoredCVarSideEffects(lua_State* L);

/// One CVar's saved value, read straight from the file.
///
/// For the settings that have to be known before the Lua state exists - the
/// shadow map's size is chosen when the per-frame resources are built, which
/// is long before the interface loads. Returns the fallback when the file has
/// no such key, so a fresh install gets the default rather than nothing.
std::string storedCVarValue(const std::string& key, const std::string& fallback);

/// Tell the CVar store that a setting it drives has been changed elsewhere.
///
/// Six settings are driven by a Blizzard control through kClientCVars, and
/// three of those also have a control in this client's own settings window.
/// The store is applied over the settings file at start-up - it is the record
/// of what the player last set - so a change made in this client's window and
/// saved to settings.cfg was undone at the next start by a CVar nobody had
/// touched since.
///
/// Does nothing for a setting no CVar drives, and writes only when the value
/// actually moves: the settings window applies on every frame of a slider drag,
/// and saving the store means writing a file.
void noteClientSettingChanged(const std::string& settingKey, const std::string& value);
void registerActionLuaAPI(lua_State* L);

/// Whether this client performs a binding command itself, without the
/// interface.
///
/// A key answered by both is answered twice, and most bindings toggle
/// something - so twice means the panel opens and shuts again on one press and
/// the key reads as dead. Consulted before running a binding.
bool clientActsOnBinding(const std::string& command);

} // namespace wowee::addons
