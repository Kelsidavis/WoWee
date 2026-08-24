// `for k, v in t do` - the generic for as Lua 5.0 wrote it, over a plain table.
//
// World of Warcraft 1.12's FrameXML is Lua 5.0 source and uses this form
// throughout. Lua 5.1 calls whatever the expression produced, and a table is
// not callable, so on the vendored 5.1 every one of those loops raised
// "attempt to call a table value" - and because it raised while the file was
// being read, it took the whole file with it. Six of FrameXML's files died
// this way on a 1.12 interface: the player, target and party frames (through
// UnitPopup_HideButtons), the chat frame, the world state frame (through
// UIParent_ManageFramePositions) and the sound options.
//
// The VM iterates a table with next now, which is what 5.0 did. What has to
// hold is that it visits every pair exactly once, that it stops, that a table
// carrying __call is still *called* rather than iterated - a 5.1 iterator
// object must not be broken to rescue a 5.0 one - and that every other type
// still raises the way it always did.
//
// A property of the VM, so it is tested as one: a bare state, no client.
#include <catch_amalgamated.hpp>

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

#include <string>

namespace {

/// A Lua state with the base library, which is all any case here needs.
struct Fixture {
    lua_State* L = luaL_newstate();

    Fixture() {
        lua_pushcfunction(L, luaopen_base);
        lua_pushstring(L, "");
        lua_call(L, 1, 0);
    }
    ~Fixture() { lua_close(L); }

    /// Run a chunk, failing the case with Lua's own message if it raises.
    void ok(const std::string& chunk) {
        INFO(chunk);
        if (luaL_dostring(L, chunk.c_str()) != 0) {
            const char* msg = lua_tostring(L, -1);
            const std::string reported = msg ? msg : "unknown Lua error";
            FAIL(reported);
        }
    }

    /// Run a chunk that must raise, and hand back the message.
    std::string raises(const std::string& chunk) {
        INFO(chunk);
        REQUIRE(luaL_dostring(L, chunk.c_str()) != 0);
        const char* msg = lua_tostring(L, -1);
        std::string out = msg ? msg : "";
        lua_pop(L, 1);
        return out;
    }

    [[nodiscard]] double number(const char* global) {
        lua_getglobal(L, global);
        REQUIRE(lua_isnumber(L, -1));
        const double v = lua_tonumber(L, -1);
        lua_pop(L, 1);
        return v;
    }

    [[nodiscard]] std::string text(const char* global) {
        lua_getglobal(L, global);
        REQUIRE(lua_isstring(L, -1));
        std::string v = lua_tostring(L, -1);
        lua_pop(L, 1);
        return v;
    }
};

}  // namespace

TEST_CASE("a plain table iterates instead of raising", "[lua][genericfor]") {
    Fixture f;
    // The shape UnitPopup_HideButtons and UIParent_ManageFramePositions use:
    // a table named directly, with no pairs() around it.
    f.ok("t = { a = 1, b = 2, c = 3 }\n"
         "keys = 0\n"
         "sum = 0\n"
         "for k, v in t do keys = keys + 1; sum = sum + v end\n");
    CHECK(f.number("keys") == 3);
    CHECK(f.number("sum") == 6);
}

TEST_CASE("an array part is visited once per element", "[lua][genericfor]") {
    Fixture f;
    f.ok("t = { 10, 20, 30, 40 }\n"
         "n = 0\n"
         "sum = 0\n"
         "for i, v in t do n = n + 1; sum = sum + v end\n");
    CHECK(f.number("n") == 4);
    CHECK(f.number("sum") == 100);
}

TEST_CASE("an empty table runs the body no times", "[lua][genericfor]") {
    Fixture f;
    // The loop has to *end*, which is the half a wrong control variable
    // breaks: it would either run forever or never start.
    f.ok("n = 0\n"
         "for k, v in {} do n = n + 1 end\n");
    CHECK(f.number("n") == 0);
}

TEST_CASE("one loop variable takes the key", "[lua][genericfor]") {
    Fixture f;
    // next() answers a pair while this loop reserves a single register. The
    // value has to go somewhere that is not the caller's stack.
    f.ok("t = { only = 'here' }\n"
         "seen = ''\n"
         "for k in t do seen = seen .. k end\n");
    CHECK(f.text("seen") == "only");
}

TEST_CASE("more loop variables than next answers read nil", "[lua][genericfor]") {
    Fixture f;
    f.ok("t = { x = 1 }\n"
         "third = 'unset'\n"
         "for k, v, w in t do third = w end\n");
    lua_getglobal(f.L, "third");
    CHECK(lua_isnil(f.L, -1));
    lua_pop(f.L, 1);
}

TEST_CASE("pairs() is untouched", "[lua][genericfor]") {
    Fixture f;
    f.ok("t = { a = 1, b = 2 }\n"
         "n = 0\n"
         "for k, v in pairs(t) do n = n + 1 end\n");
    CHECK(f.number("n") == 2);
}

TEST_CASE("a table with __call is called, not iterated", "[lua][genericfor]") {
    Fixture f;
    // A 5.1 iterator can be a callable object, and it must keep being called.
    // This one counts down from 3 and has table fields that iterating would
    // find instead - so the two behaviours are told apart by the answer.
    f.ok("local left = 3\n"
         "it = setmetatable({ decoy = 'iterated' }, {\n"
         "  __call = function() \n"
         "    if left == 0 then return nil end\n"
         "    left = left - 1\n"
         "    return 'called', left\n"
         "  end })\n"
         "n = 0\n"
         "last = ''\n"
         "for k, v in it do n = n + 1; last = k end\n");
    CHECK(f.number("n") == 3);
    CHECK(f.text("last") == "called");
}

TEST_CASE("a number still raises", "[lua][genericfor]") {
    Fixture f;
    // Only the table case is rescued. Everything else has to fail as loudly
    // as it always did, or a real mistake becomes an empty loop.
    const std::string msg = f.raises("for k, v in 7 do end");
    CHECK(msg.find("attempt to call a number value") != std::string::npos);
}

TEST_CASE("nil still raises", "[lua][genericfor]") {
    Fixture f;
    const std::string msg = f.raises("for k, v in nil do end");
    CHECK(msg.find("attempt to call a nil value") != std::string::npos);
}

TEST_CASE("a string still raises", "[lua][genericfor]") {
    Fixture f;
    const std::string msg = f.raises("for k, v in 'text' do end");
    CHECK(msg.find("attempt to call a string value") != std::string::npos);
}

TEST_CASE("the loop body may add to a different table", "[lua][genericfor]") {
    Fixture f;
    // What FrameXML actually does inside these loops: read one table, write
    // another. Modifying the table being iterated is undefined in Lua itself
    // and is not asked about here.
    f.ok("src = { a = 1, b = 2, c = 3 }\n"
         "dst = {}\n"
         "n = 0\n"
         "for k, v in src do dst[k] = v * 2; n = n + 1 end\n"
         "sum = 0\n"
         "for k, v in pairs(dst) do sum = sum + v end\n");
    CHECK(f.number("n") == 3);
    CHECK(f.number("sum") == 12);
}

TEST_CASE("nested loops over the same table each complete", "[lua][genericfor]") {
    Fixture f;
    // Two loops sharing a table need separate control variables; a control
    // variable kept anywhere shared would end the outer loop early.
    f.ok("t = { a = 1, b = 2, c = 3 }\n"
         "n = 0\n"
         "for k1, v1 in t do\n"
         "  for k2, v2 in t do n = n + 1 end\n"
         "end\n");
    CHECK(f.number("n") == 9);
}
