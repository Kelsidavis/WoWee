#pragma once

// The globals an interface written before 3.0 reads instead of its handler's
// arguments: the frame as `this`, the event name as `event`, and the payload as
// `arg1`..`argN`.
//
// 1.12 and 2.4.3 handlers take no parameters at all. Their bodies are written
// as `this:RegisterEvent("PLAYER_LOGIN")` and `if ( arg1 == "player" )`, and the
// client published those names before making the call. 3.0 replaced that with
// the arguments this client already passes, and the two conventions are
// otherwise identical - which is why one client can serve both by publishing
// the older names beside the newer ones for the interfaces that ask.
//
// With this off against a 1.12 interface, `this` is nil on the first line of
// every OnLoad in the file. Nothing registers for an event, nothing is
// positioned and nothing hides itself, so every frame FrameXML declares ends up
// on screen at once, wherever its XML left it.
//
// Its own translation unit, and dependent on nothing but Lua, so that the part
// that has to be exactly right can be tested without a window, a widget tree or
// an interface: see tests/test_lua_handler_globals.cpp.

struct lua_State;

namespace wowee::addons {

/// Publish those globals around every script handler, or do not.
///
/// Set from AddonManager::loadFrameXml, off the interface's own
/// `## Interface:` number, before the first file is read - an OnLoad runs
/// during the load, so deciding this afterwards would be deciding it too late.
void setLegacyHandlerGlobals(bool on);
bool legacyHandlerGlobals();

/// pcall a frame's handler, publishing those globals around the call.
///
/// The same stack contract as lua_pcall - the function, then the frame, then
/// the handler's own arguments - so it stands in for the call it replaces at
/// every site that dispatches a script. That is the point of its shape: the
/// globals have to be set at every one of them or the interface half works, and
/// a site that keeps its lua_pcall is invisible until the one handler that
/// needed it fails.
///
/// `script` decides where the arguments start: OnEvent alone puts the event
/// name before the payload. Zero results, which every dispatch site wants.
int pcallScript(lua_State* L, const char* script, int nargs, int errfunc);

/// __WoweeFireOnLoad(frame) - run a frame's OnLoad now that it is built.
///
/// Emitted by framexml_emitter at the end of every frame it builds, in place of
/// calling the handler straight out of the generated chunk. That call is the
/// one dispatch in the whole system that happens from Lua rather than from
/// lua_engine.cpp, so it is the one place pcallScript cannot reach - and it is
/// the most load-bearing of them for a 1.12 interface, where `this:Hide()` on
/// the first line of an OnLoad is how nearly every frame in the file gets off
/// the screen.
///
/// Not a pcall. An OnLoad that raises has always taken the rest of its file
/// with it, and loadFrameXml's failure list is built on that; this only puts a
/// name in scope around the call.
int lua_WoweeFireOnLoad(lua_State* L);

}  // namespace wowee::addons
