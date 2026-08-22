// `this`, `event` and `arg1`..`argN` - the names a 1.12 handler reads instead
// of its arguments.
//
// The whole of a 1.12 interface rests on these. Its OnLoad bodies open with
// `this:RegisterEvent(...)` and `this:Hide()`, so if `this` is nil there, every
// frame in the file raises on its first line: nothing registers for an event,
// nothing is positioned and nothing hides itself, and what reaches the screen
// is every frame FrameXML declares, at once, wherever its XML left it.
//
// A pure function of the Lua stack, and tested as one. The real dispatch needs
// a window, a widget tree and an interface; what has to be exactly right here
// is which names are set, to what, and that they survive one handler running
// inside another - and none of that needs any of it.
#include <catch_amalgamated.hpp>

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

#include <string>

#include "addons/lua_handler_globals.hpp"

using wowee::addons::legacyHandlerGlobals;
using wowee::addons::pcallScript;
using wowee::addons::setLegacyHandlerGlobals;

namespace {

/// A Lua state with the handler under test installed on a frame-shaped table.
///
/// The frame is `f`, its handler goes in `f.__scripts[script]`, and what the
/// handler saw is left in globals the case can read back.
struct Fixture {
    lua_State* L = luaL_newstate();

    Fixture() {
        // Base only. The vendored Lua leaves out loadlib deliberately - a game
        // client has no business dlopening things - so luaL_openlibs does not
        // link here, and nothing below wants more than error() anyway.
        lua_pushcfunction(L, luaopen_base);
        lua_pushstring(L, "");
        lua_call(L, 1, 0);
        REQUIRE(luaL_dostring(L, "f = { __scripts = {} }\n") == 0);
    }
    ~Fixture() {
        setLegacyHandlerGlobals(false);
        lua_close(L);
    }

    void handler(const char* script, const char* body) {
        const std::string chunk =
            std::string("f.__scripts['") + script + "'] = function(...) " + body + " end";
        INFO(chunk);
        REQUIRE(luaL_dostring(L, chunk.c_str()) == 0);
    }

    /// Dispatch the way lua_engine.cpp does: the handler, the frame, then the
    /// handler's own arguments, all pushed before the call.
    int dispatch(const char* script, const std::vector<std::string>& args) {
        lua_getglobal(L, "f");
        lua_getfield(L, -1, "__scripts");
        lua_getfield(L, -1, script);
        REQUIRE(lua_isfunction(L, -1));
        lua_getglobal(L, "f");
        for (const std::string& a : args) lua_pushstring(L, a.c_str());
        const int rc = pcallScript(L, script, 1 + static_cast<int>(args.size()), 0);
        if (rc != 0) {
            const std::string err = lua_tostring(L, -1) ? lua_tostring(L, -1) : "?";
            INFO(err);
            lua_pop(L, 1);
        }
        lua_pop(L, 2);   // __scripts, f
        return rc;
    }

    std::string global(const char* name) {
        lua_getglobal(L, name);
        const char* got = lua_tostring(L, -1);
        std::string out = got ? got : "";
        lua_pop(L, 1);
        return out;
    }
    bool globalIsNil(const char* name) {
        lua_getglobal(L, name);
        const bool nil = lua_isnil(L, -1);
        lua_pop(L, 1);
        return nil;
    }
};

}  // namespace

TEST_CASE("A 1.12 handler finds the frame in `this`", "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    // No parameters at all, which is how every handler in 1.12's FrameXML is
    // written - including the OnLoad bodies that hide the frames.
    fx.handler("OnLoad", "saw = (this == f)");
    REQUIRE(fx.dispatch("OnLoad", {}) == 0);
    lua_getglobal(fx.L, "saw");
    CHECK(lua_toboolean(fx.L, -1) == 1);
    lua_pop(fx.L, 1);
}

TEST_CASE("OnEvent names the event and numbers the payload", "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    fx.handler("OnEvent", "sawEvent = event; sawOne = arg1; sawTwo = arg2");
    REQUIRE(fx.dispatch("OnEvent", {"UNIT_HEALTH", "player", "1234"}) == 0);
    CHECK(fx.global("sawEvent") == "UNIT_HEALTH");
    CHECK(fx.global("sawOne") == "player");
    CHECK(fx.global("sawTwo") == "1234");
}

TEST_CASE("Every other handler's own arguments start at arg1",
          "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    // OnClick's first argument is the button, and 1.12 reads it as arg1 - there
    // is no `event` in front of it the way there is for OnEvent.
    fx.handler("OnClick", "sawOne = arg1");
    REQUIRE(fx.dispatch("OnClick", {"RightButton"}) == 0);
    CHECK(fx.global("sawOne") == "RightButton");
}

TEST_CASE("A handler that runs inside another gives the names back",
          "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    // This is what Show() does from inside an OnEvent: the inner handler is
    // dispatched while the outer one is still running, and the outer one goes
    // on to read arg1 afterwards. Without the save it would read the inner
    // handler's.
    fx.handler("OnShow", "innerSaw = arg1");
    REQUIRE(luaL_dostring(fx.L,
        "f.__scripts.OnEvent = function()\n"
        "  local before = arg1\n"
        "  local s = f.__scripts.OnShow\n"
        "  __dispatchInner()\n"
        "  outerBefore = before\n"
        "  outerAfter = arg1\n"
        "  outerEvent = event\n"
        "end\n") == 0);
    // The inner dispatch, done the same way the engine would.
    lua_pushcfunction(fx.L, [](lua_State* L) -> int {
        lua_getglobal(L, "f");
        lua_getfield(L, -1, "__scripts");
        lua_getfield(L, -1, "OnShow");
        lua_getglobal(L, "f");
        lua_pushstring(L, "inner");
        pcallScript(L, "OnShow", 2, 0);
        lua_pop(L, 2);
        return 0;
    });
    lua_setglobal(fx.L, "__dispatchInner");

    REQUIRE(fx.dispatch("OnEvent", {"BAG_UPDATE", "outer"}) == 0);
    CHECK(fx.global("innerSaw") == "inner");
    CHECK(fx.global("outerBefore") == "outer");
    CHECK(fx.global("outerAfter") == "outer");
    CHECK(fx.global("outerEvent") == "BAG_UPDATE");
}

TEST_CASE("The OnLoad the emitter fires finds the frame in `this`",
          "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    // The generated chunk ends every frame with __WoweeFireOnLoad(frame), and
    // this is what that reaches. It is the one dispatch that happens from Lua
    // rather than from the engine, and in a 1.12 interface it is the one that
    // decides whether the file's frames are on screen or hidden.
    fx.handler("OnLoad", "saw = (this == f)");
    lua_pushcfunction(fx.L, wowee::addons::lua_WoweeFireOnLoad);
    lua_getglobal(fx.L, "f");
    REQUIRE(lua_pcall(fx.L, 1, 0, 0) == 0);
    lua_getglobal(fx.L, "saw");
    CHECK(lua_toboolean(fx.L, -1) == 1);
    lua_pop(fx.L, 1);
    // And put back: frames load inside other frames' OnLoad - CreateFrame is
    // an ordinary thing for one to call - so the name has to survive that.
    CHECK(fx.globalIsNil("this"));
}

TEST_CASE("An OnLoad that builds a frame gets its own `this` back",
          "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    REQUIRE(luaL_dostring(fx.L,
        "inner = { __scripts = { OnLoad = function() innerSaw = (this == inner) end } }\n"
        "f.__scripts.OnLoad = function()\n"
        "  outerBefore = (this == f)\n"
        "  __fire(inner)\n"
        "  outerAfter = (this == f)\n"
        "end\n") == 0);
    lua_pushcfunction(fx.L, wowee::addons::lua_WoweeFireOnLoad);
    lua_setglobal(fx.L, "__fire");

    lua_pushcfunction(fx.L, wowee::addons::lua_WoweeFireOnLoad);
    lua_getglobal(fx.L, "f");
    REQUIRE(lua_pcall(fx.L, 1, 0, 0) == 0);
    for (const char* name : {"innerSaw", "outerBefore", "outerAfter"}) {
        INFO(name);
        lua_getglobal(fx.L, name);
        CHECK(lua_toboolean(fx.L, -1) == 1);
        lua_pop(fx.L, 1);
    }
}

TEST_CASE("An interface from 3.0 onwards is left alone", "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(false);
    CHECK_FALSE(legacyHandlerGlobals());
    fx.handler("OnEvent", "sawSelf = (... == f)");
    REQUIRE(fx.dispatch("OnEvent", {"UNIT_HEALTH", "player"}) == 0);
    lua_getglobal(fx.L, "sawSelf");
    CHECK(lua_toboolean(fx.L, -1) == 1);
    lua_pop(fx.L, 1);
    // Nothing published, because 3.3.5a's interface names its arguments and a
    // global called `this` is one more name for an addon to trip over.
    CHECK(fx.globalIsNil("this"));
    CHECK(fx.globalIsNil("event"));
    CHECK(fx.globalIsNil("arg1"));
}

TEST_CASE("A handler that raises still gives the names back",
          "[handlerglobals]") {
    Fixture fx;
    setLegacyHandlerGlobals(true);
    fx.handler("OnEvent", "error('boom')");
    REQUIRE(luaL_dostring(fx.L, "this = 'before'; arg1 = 'kept'") == 0);
    CHECK(fx.dispatch("OnEvent", {"UNIT_HEALTH", "player"}) != 0);
    // The interface carries on after a raise - it is reported and the next
    // event is dispatched - so the state it carries on with has to be the one
    // it had, not whatever the handler was holding when it died.
    CHECK(fx.global("this") == "before");
    CHECK(fx.global("arg1") == "kept");
}
