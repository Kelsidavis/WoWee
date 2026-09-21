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
#include "lualib.h"
}

#include <cstring>
#include <initializer_list>
#include <string>
#include <utility>

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

/// Run several chunks in one state, and answer with what the last one returned.
///
/// More than parsing, and still short of framexml_run: enough of Lua to run a
/// snippet against stand-ins for the frames it reaches for. A failure comes
/// back as the Lua error, so the message says which chunk and which line.
std::string runChunks(std::initializer_list<std::pair<const char*, const char*>> chunks) {
    lua_State* L = luaL_newstate();
    REQUIRE(L != nullptr);
    // Only what the snippets under test use: pairs, ipairs, type and rawget
    // from the base library, insert and remove from the table one.
    lua_pushcfunction(L, luaopen_base);
    lua_call(L, 0, 0);
    lua_pushcfunction(L, luaopen_table);
    lua_pushstring(L, LUA_TABLIBNAME);
    lua_call(L, 1, 0);

    std::string result;
    for (const auto& [chunk, name] : chunks) {
        if (luaL_loadbuffer(L, chunk, std::strlen(chunk), name) != 0 ||
            lua_pcall(L, 0, 1, 0) != 0) {
            const char* message = lua_tostring(L, -1);
            lua_close(L);
            return std::string(name) + ": " + (message ? message : "(no message)");
        }
        const char* value = lua_tostring(L, -1);
        result = value ? value : "";
        lua_pop(L, 1);
    }
    lua_close(L);
    return result;
}

/// The options machinery the removal snippet walks, cut down to what it calls.
constexpr const char* kFakeOptionsPanelLua = R"LUA(
local function frame(name, parent)
    local f = {
        _name = name, _parent = parent, _hidden = false, _hooks = {},
        GetName = function(self) return self._name end,
        GetParent = function(self) return self._parent end,
        GetChildren = function(self) return unpack(self._children or {}) end,
        Hide = function(self) self._hidden = true end,
        HookScript = function(self, script, fn) self._hooks[script] = fn end,
    }
    _G[name] = f
    if parent then
        parent._children = parent._children or {}
        table.insert(parent._children, f)
    end
    return f
end

local panel = frame("VideoOptionsResolutionPanel")
-- The retired checkbox, and one control that stays.
local vsync = frame("VideoOptionsResolutionPanelVSync", panel)
local scale = frame("VideoOptionsResolutionPanelUIScaleSlider", panel)
panel.controls = { vsync, scale }

-- Both carry what BlizzardOptionsPanel_SetupControl would have left on them:
-- the cached value the Okay loops commit, and for the action bar boxes the
-- uvar naming a global FrameXML reads. A retired control has to lose the
-- first and keep the second.
vsync.value = "0"
vsync.uvar = "GX_VSYNC"
scale.value = "1"

local bars = frame("InterfaceOptionsActionBarsPanel")
local bottomLeft = frame("InterfaceOptionsActionBarsPanelBottomLeft", bars)
bottomLeft.value = "1"
bottomLeft.uvar = "SHOW_MULTI_ACTIONBAR_1"
bars.controls = { bottomLeft }
)LUA";

}  // namespace

TEST_CASE("a retired control commits nothing and keeps its place", "[addonlua]") {
    // Hiding a control is not retiring it. VideoOptionsPanel_Okay walks
    // panel.controls and writes every entry's cached value back to its cvar,
    // changed or not - so the hidden vertical-sync checkbox replayed the value
    // it read at load over the Display page's own row, and pressing Okay on the
    // video window turned vertical sync off again each time it was switched on.
    //
    // Both halves are asserted here because fixing the first by unregistering
    // the control broke the second, and shipped: BlizzardOptionsPanel_SetupControl
    // runs only over panel.controls, and it is what assigns the uvar globals, so
    // the action bar checkboxes leaving the list left SHOW_MULTI_ACTIONBAR_1 nil
    // and the player with no bottom action bar.
    const std::string result = runChunks({
        {kFakeOptionsPanelLua, "FakeOptionsPanel"},
        {wowee::addons::kRemovedControlsLua, "RemovedControls"},
        {R"LUA(
            local vsync = VideoOptionsResolutionPanelVSync
            local scale = VideoOptionsResolutionPanelUIScaleSlider
            local bar = InterfaceOptionsActionBarsPanelBottomLeft
            local listed = ""
            for _, c in ipairs(VideoOptionsResolutionPanel.controls) do
                listed = listed .. c:GetName() .. " "
            end
            return "listed=" .. listed
                .. "| vsync.value=" .. tostring(vsync.value)
                .. " scale.value=" .. tostring(scale.value)
                .. " bar.value=" .. tostring(bar.value)
                .. " bar.uvar=" .. tostring(bar.uvar)
                .. " hidden=" .. tostring(vsync._hidden)
        )LUA",
         "Check"},
    });
    // The retired boxes keep their place on the panel, so setup still reaches
    // them, and lose only the value the Okay loops would have written back.
    CHECK(result ==
          "listed=VideoOptionsResolutionPanelVSync VideoOptionsResolutionPanelUIScaleSlider "
          "| vsync.value=nil scale.value=1 bar.value=nil "
          "bar.uvar=SHOW_MULTI_ACTIONBAR_1 hidden=true");
}

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

TEST_CASE("the chat background swatch script parses", "[addonlua]") {
    const std::string err = compileError(wowee::addons::kChatBackgroundSwatchLua,
                                         "ChatBackgroundSwatch");
    INFO(err);
    CHECK(err.empty());
}
