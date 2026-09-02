// The Lua this client injects into the interface, handed to Lua itself.
//
// These are C++ string literals that nothing compiles until the client runs.
// executeString answers false on a syntax error and the client carries on -
// the only sign is a warning in a log that is warning-only, and the panels
// simply are not there. Four hundred lines of Lua with no build step is four
// hundred lines where a stray `end` costs a release.
//
// This does not run them. Running needs the whole widget shim, the settings
// bridge and the interface loaded; that is what framexml_run is for. This asks
// the narrower question that has a cheap answer: does it parse.
#include <catch_amalgamated.hpp>

extern "C" {
#include "lauxlib.h"
#include "lua.h"
}

#include <cstring>
#include <string>

#include "addons/addon_lua_snippets.hpp"

namespace {

/// The Lua compiler's own verdict, and its message when it refuses.
std::string compileError(const char* chunk, const char* name) {
    lua_State* L = luaL_newstate();
    REQUIRE(L != nullptr);
    // No libraries opened: parsing needs none, and the vendored Lua leaves out
    // loadlib deliberately - a game client has no business dlopening things -
    // so luaL_openlibs would not even link here.
    std::string err;
    if (luaL_loadbuffer(L, chunk, std::strlen(chunk), name) != 0) {
        const char* message = lua_tostring(L, -1);
        err = message ? message : "(no message)";
    }
    lua_close(L);
    return err;
}

}  // namespace

TEST_CASE("the options panel script parses", "[addonlua]") {
    // Twelve categories of check button, slider and dropdown, built from the
    // settings schema. Added over several passes, none of which could have
    // been told by the compiler that it had broken the one before.
    const std::string err =
        compileError(wowee::addons::kWoweeOptionsPanelLua, "WoweeOptionsPanel");
    INFO(err);
    CHECK(err.empty());
}

TEST_CASE("the coin clearance script parses", "[addonlua]") {
    // Reaches into MoneyFrame_Update, which is Blizzard's, and hooks rather
    // than edits it - so a mistake here shows up as money that draws wrong
    // rather than as anything that says "script error".
    const std::string err =
        compileError(wowee::addons::kCoinAmountClearanceLua, "CoinAmountClearance");
    INFO(err);
    CHECK(err.empty());
}

TEST_CASE("the chat box visibility script parses", "[addonlua]") {
    // The one that proves the point of this file. It lived in the settings
    // panel as a run of adjacent C++ string literals with no separator, so the
    // `end` closing its loop ran into the `end` closing its `if` and then into
    // the `if` after that. Every click of Chat Style printed a script error and
    // changed nothing, and no build ever said so.
    const std::string err =
        compileError(wowee::addons::kChatBoxVisibilityLua, "ChatBoxVisibility");
    INFO(err);
    CHECK(err.empty());
}
