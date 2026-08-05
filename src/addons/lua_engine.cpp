#include "addons/lua_engine.hpp"
#include "ui/widget_tree.hpp"
#include <chrono>
#include <cfloat>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <climits>
#include <set>
#include <cstdlib>
#include "addons/lua_api_helpers.hpp"
#include "addons/lua_api_registrations.hpp"
#include "addons/toc_parser.hpp"
#include "core/window.hpp"
#include <imgui.h>
#include <fstream>
#include <filesystem>

extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

namespace wowee::addons {

namespace {
/// Names asked for and not found, while the fallback is on. File-scope because
/// the recorder is a Lua callback and the report runs at shutdown.
std::set<std::string>& missingApiNames() {
    static std::set<std::string> names;
    return names;
}
}


static int lua_wow_print(lua_State* L) {
    int nargs = lua_gettop(L);
    std::string result;
    for (int i = 1; i <= nargs; i++) {
        if (i > 1) result += '\t';
        // Lua 5.1: use lua_tostring (luaL_tolstring is 5.3+)
        if (lua_isstring(L, i) || lua_isnumber(L, i)) {
            const char* s = lua_tostring(L, i);
            if (s) result += s;
        } else if (lua_isboolean(L, i)) {
            result += lua_toboolean(L, i) ? "true" : "false";
        } else if (lua_isnil(L, i)) {
            result += "nil";
        } else {
            result += lua_typename(L, lua_type(L, i));
        }
    }

    auto* gh = getGameHandler(L);
    if (gh) {
        game::MessageChatData msg;
        msg.type = game::ChatType::SYSTEM;
        msg.language = game::ChatLanguage::UNIVERSAL;
        msg.message = result;
        gh->addLocalChatMessage(msg);
    }
    LOG_INFO("[Lua] ", result);
    return 0;
}

// WoW-compatible message() — same as print for now
static int lua_wow_message(lua_State* L) {
    return lua_wow_print(L);
}

// Helper: resolve WoW unit IDs to GUID
// Read UNIT_FIELD_TARGET_LO/HI from an entity's update fields to get what it's targeting

// --- Frame system functions ---

static int lua_Frame_RegisterEvent(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);  // self
    const char* eventName = luaL_checkstring(L, 2);

    // Get frame's registered events table (create if needed)
    lua_getfield(L, 1, "__events");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setfield(L, 1, "__events");
    }
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, eventName);
    lua_pop(L, 1);

    // Also register in global __WoweeFrameEvents for dispatch
    lua_getglobal(L, "__WoweeFrameEvents");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setglobal(L, "__WoweeFrameEvents");
    }
    lua_getfield(L, -1, eventName);
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setfield(L, -3, eventName);
    }
    // Append frame reference
    int len = static_cast<int>(lua_objlen(L, -1));
    lua_pushvalue(L, 1);  // push frame
    lua_rawseti(L, -2, len + 1);
    lua_pop(L, 2);  // pop list + __WoweeFrameEvents
    return 0;
}

// Frame method: frame:UnregisterEvent("EVENT")
static int lua_Frame_UnregisterEvent(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    const char* eventName = luaL_checkstring(L, 2);

    // Remove from frame's own events
    lua_getfield(L, 1, "__events");
    if (lua_istable(L, -1)) {
        lua_pushnil(L);
        lua_setfield(L, -2, eventName);
    }
    lua_pop(L, 1);
    return 0;
}

// Frame method: frame:SetScript("handler", func)
static int lua_Frame_SetScript(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    const char* scriptType = luaL_checkstring(L, 2);
    // arg 3 can be function or nil
    lua_getfield(L, 1, "__scripts");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setfield(L, 1, "__scripts");
    }
    lua_pushvalue(L, 3);
    lua_setfield(L, -2, scriptType);
    lua_pop(L, 1);

    // Track frames with OnUpdate in __WoweeOnUpdateFrames
    if (strcmp(scriptType, "OnUpdate") == 0) {
        lua_getglobal(L, "__WoweeOnUpdateFrames");
        if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
        if (lua_isfunction(L, 3)) {
            // Add frame to the list
            int len = static_cast<int>(lua_objlen(L, -1));
            lua_pushvalue(L, 1);
            lua_rawseti(L, -2, len + 1);
        }
        lua_pop(L, 1);
    }
    return 0;
}

// Frame method: frame:GetScript("handler")
static int lua_Frame_GetScript(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    const char* scriptType = luaL_checkstring(L, 2);
    lua_getfield(L, 1, "__scripts");
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, scriptType);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

// Frame method: frame:GetName()
static int lua_Frame_GetName(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__name");
    return 1;
}

// Frame method: frame:Show() / frame:Hide() / frame:IsShown() / frame:IsVisible()
static int lua_Frame_Show(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_pushboolean(L, 1);
    lua_setfield(L, 1, "__visible");
    return 0;
}
static int lua_Frame_Hide(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_pushboolean(L, 0);
    lua_setfield(L, 1, "__visible");
    return 0;
}
static int lua_Frame_IsShown(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__visible");
    lua_pushboolean(L, lua_toboolean(L, -1));
    return 1;
}


// ── Widget-backed regions ───────────────────────────────────────────────────
//
// Frames and regions are Lua tables, as they were, but each now carries a
// __wid handle into the C++ widget tree that holds its geometry and its art.
// Without that the methods below were a table of no-ops: an addon could call
// SetTexture all day and nothing existed to draw.

namespace {

uint32_t widgetIdOf(lua_State* L, int index) {
    if (!lua_istable(L, index)) return 0;
    lua_getfield(L, index, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    return id;
}

wowee::ui::Widget* widgetOf(lua_State* L, int index) {
    auto* tree = wowee::addons::getWidgetTree(L);
    if (!tree) return nullptr;
    return tree->get(widgetIdOf(L, index));
}

// SetPoint(point [, relativeTo] [, relativePoint] [, x, y]) — every argument
// after the first is optional and the shapes overlap, so decide by type rather
// than by count.
int lua_Region_SetPoint(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const uint32_t id = widgetIdOf(L, 1);
    if (!tree || id == 0) return 0;

    wowee::ui::Anchor a;
    a.point = luaL_optstring(L, 2, "CENTER");
    int argi = 3;
    if (lua_istable(L, argi)) {
        a.relativeTo = widgetIdOf(L, argi);
        ++argi;
    } else if (lua_isstring(L, argi) && !lua_isnumber(L, argi)) {
        // A name rather than the frame itself; resolve through the global it
        // was published under, which is how FrameXML refers to most things.
        lua_getglobal(L, lua_tostring(L, argi));
        if (lua_istable(L, -1)) a.relativeTo = widgetIdOf(L, lua_gettop(L));
        lua_pop(L, 1);
        ++argi;
    }
    // Anchoring to itself is not a position, and a name can resolve to the
    // frame that was just published under it.
    if (a.relativeTo == id) {
        const auto* self = tree->get(id);
        a.relativeTo = self ? self->parent : 0;
    }
    if (lua_isstring(L, argi) && !lua_isnumber(L, argi)) {
        a.relativePoint = lua_tostring(L, argi);
        ++argi;
    } else {
        a.relativePoint = a.point;   // Blizzard's default is the same point
    }
    if (lua_isnumber(L, argi))     a.x = static_cast<float>(lua_tonumber(L, argi));
    if (lua_isnumber(L, argi + 1)) a.y = static_cast<float>(lua_tonumber(L, argi + 1));

    tree->addPoint(id, a);
    return 0;
}

int lua_Region_ClearAllPoints(lua_State* L) {
    if (auto* tree = wowee::addons::getWidgetTree(L)) tree->clearPoints(widgetIdOf(L, 1));
    return 0;
}

int lua_Region_SetAllPoints(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const uint32_t id = widgetIdOf(L, 1);
    if (!tree || id == 0) return 0;
    uint32_t target = 0;
    if (lua_istable(L, 2)) target = widgetIdOf(L, 2);
    else if (lua_isstring(L, 2)) {
        lua_getglobal(L, lua_tostring(L, 2));
        if (lua_istable(L, -1)) target = widgetIdOf(L, lua_gettop(L));
        lua_pop(L, 1);
    }
    // A frame cannot fill itself. FrameXML's own UIParent is declared
    // setAllPoints and its parent is named UIParent — but CreateFrame publishes
    // the new frame under that name first, so by the time this runs the name
    // means the frame itself. Two identical constraints collapse to no size at
    // the origin, and everything anchored to it lands there too.
    const auto* w = tree->get(id);
    if (target == 0 || target == id) target = w ? w->parent : 0;
    tree->setAllPoints(id, target);
    return 0;
}

int lua_Region_SetSize(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->width  = static_cast<float>(luaL_optnumber(L, 2, 0));
        w->height = static_cast<float>(luaL_optnumber(L, 3, 0));
    }
    return 0;
}
int lua_Region_SetWidth(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->width = static_cast<float>(luaL_optnumber(L, 2, 0));
    return 0;
}
int lua_Region_SetHeight(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->height = static_cast<float>(luaL_optnumber(L, 2, 0));
    return 0;
}
/// Width of a string as it would be drawn.
///
/// Through the real font where there is one, so a button sized to its label
/// gets the size the label actually takes. During the FrameXML load there may
/// not be a frame in flight to ask, and an estimate is far better than nothing:
/// the alternative was answering nil, and MoneyFrame does
/// SetWidth(GetTextWidth() + iconWidth) — arithmetic that loses the file.
float measureTextWidth(const std::string& text, float fontHeight) {
    if (text.empty()) return 0.0f;
    const float size = fontHeight > 0.0f ? fontHeight : 12.0f;
    if (ImGui::GetCurrentContext() != nullptr) {
        if (ImFont* font = ImGui::GetFont()) {
            return font->CalcTextSizeA(size, FLT_MAX, 0.0f, text.c_str()).x;
        }
    }
    // Roughly half the height per character, which is about right for the
    // proportional faces the interface uses.
    return static_cast<float>(text.size()) * size * 0.5f;
}

/// The font string a widget measures: itself if it is one, and otherwise the
/// one a button was given, which is where its text actually lives.
const wowee::ui::Widget* textWidgetOf(lua_State* L, int index) {
    const wowee::ui::Widget* w = widgetOf(L, index);
    if (!w || w->kind == wowee::ui::WidgetKind::FontString) return w;
    auto* tree = wowee::addons::getWidgetTree(L);
    if (!tree) return w;
    lua_getfield(L, index, "__fontString");
    const wowee::ui::Widget* fs =
        lua_istable(L, -1) ? tree->get(widgetIdOf(L, lua_gettop(L))) : nullptr;
    lua_pop(L, 1);
    return fs ? fs : w;
}

int lua_Region_GetTextWidth(lua_State* L) {
    const auto* w = textWidgetOf(L, 1);
    lua_pushnumber(L, w ? measureTextWidth(w->text, w->fontHeight) : 0.0);
    return 1;
}

int lua_Region_GetTextHeight(lua_State* L) {
    const auto* w = textWidgetOf(L, 1);
    lua_pushnumber(L, w ? (w->fontHeight > 0.0f ? w->fontHeight : 12.0f) : 0.0);
    return 1;
}

int lua_Region_GetWidth(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? (w->rectW > 0.0f ? w->rectW : w->width) : 0.0);
    return 1;
}
int lua_Region_GetHeight(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? (w->rectH > 0.0f ? w->rectH : w->height) : 0.0);
    return 1;
}
int lua_Region_Show(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->shown = true;
    lua_pushboolean(L, 1); lua_setfield(L, 1, "__visible");
    return 0;
}
int lua_Region_Hide(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->shown = false;
    lua_pushboolean(L, 0); lua_setfield(L, 1, "__visible");
    return 0;
}
int lua_Region_IsShown(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushboolean(L, w ? (w->shown ? 1 : 0) : 0);
    return 1;
}
int lua_Region_SetAlpha(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->alpha = static_cast<float>(luaL_optnumber(L, 2, 1.0));
    return 0;
}
int lua_Region_GetAlpha(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? w->alpha : 1.0);
    return 1;
}

// SetTexture takes either a path or a colour, and addons use both freely.
int lua_Texture_SetTexture(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    if (lua_isnumber(L, 2)) {
        w->solidColor = true;
        w->texturePath.clear();
        w->color[0] = static_cast<float>(lua_tonumber(L, 2));
        w->color[1] = static_cast<float>(luaL_optnumber(L, 3, 0.0));
        w->color[2] = static_cast<float>(luaL_optnumber(L, 4, 0.0));
        w->color[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    } else {
        w->solidColor = false;
        w->texturePath = luaL_optstring(L, 2, "");
    }
    return 0;
}
int lua_Texture_GetTexture(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushstring(L, w ? w->texturePath.c_str() : "");
    return 1;
}
int lua_Texture_SetTexCoord(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->texCoord[0] = static_cast<float>(luaL_optnumber(L, 2, 0.0));
        w->texCoord[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->texCoord[2] = static_cast<float>(luaL_optnumber(L, 4, 0.0));
        w->texCoord[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}
int lua_Region_SetVertexColor(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->color[0] = static_cast<float>(luaL_optnumber(L, 2, 1.0));
        w->color[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->color[2] = static_cast<float>(luaL_optnumber(L, 4, 1.0));
        w->color[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}
int lua_Region_SetDrawLayer(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->layer = wowee::ui::parseDrawLayer(luaL_optstring(L, 2, "ARTWORK"));
        w->subLevel = static_cast<int>(luaL_optnumber(L, 3, 0));
    }
    return 0;
}
int lua_Frame_EnableMouse(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        // Absent argument means true, which is how addons usually write it.
        w->mouseEnabled = lua_isnone(L, 2) ? true : (lua_toboolean(L, 2) != 0);
    }
    return 0;
}
int lua_Frame_IsMouseEnabled(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushboolean(L, w && w->mouseEnabled ? 1 : 0);
    return 1;
}

int lua_Frame_SetFrameStrata(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->strata = wowee::ui::parseStrata(luaL_optstring(L, 2, "MEDIUM"));
        w->strataExplicit = true;
    }
    return 0;
}
int lua_Frame_SetFrameLevel(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->level = static_cast<int>(luaL_optnumber(L, 2, 0));
        w->levelExplicit = true;
    }
    return 0;
}
int lua_FontString_SetText(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->text = luaL_optstring(L, 2, "");
    lua_pushstring(L, luaL_optstring(L, 2, ""));
    lua_setfield(L, 1, "_text");
    return 0;
}
int lua_FontString_GetText(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushstring(L, w ? w->text.c_str() : "");
    return 1;
}
int lua_FontString_SetJustifyH(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->justifyH = luaL_optstring(L, 2, "CENTER");
    return 0;
}

// ── Fonts ───────────────────────────────────────────────────────────────────

int lua_FontString_SetTextColor(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->color[0] = static_cast<float>(luaL_optnumber(L, 2, 1.0));
        w->color[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->color[2] = static_cast<float>(luaL_optnumber(L, 4, 1.0));
        w->color[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}

int lua_FontString_SetFont(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        if (lua_isstring(L, 2)) w->fontFace = lua_tostring(L, 2);
        // The flags argument, where "OUTLINE" and "THICKOUTLINE" arrive.
        if (const char* flags = lua_isstring(L, 4) ? lua_tostring(L, 4) : nullptr) {
            const std::string f(flags);
            if (f.find("THICK") != std::string::npos)        w->fontOutline = "THICK";
            else if (f.find("OUTLINE") != std::string::npos) w->fontOutline = "NORMAL";
            else                                             w->fontOutline.clear();
        }
        // (path, height, flags). Only the height is honoured for now; the path
        // needs a font atlas rebuild, which cannot happen mid-frame.
        const double h = luaL_optnumber(L, 3, 0.0);
        if (h > 0.0) w->fontHeight = static_cast<float>(h);
    }
    return 0;
}

/// GetFont() → path, height, flags.
///
/// The height is the part anything does arithmetic on: WatchFrame measures a
/// test line with local _, fontHeight = line.text:GetFont() and divides by it
/// two lines later, so answering nothing loses the file.
int lua_FontString_GetFont(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushstring(L, "Fonts\\FRIZQT__.TTF");
    lua_pushnumber(L, w && w->fontHeight > 0.0f ? w->fontHeight : 12.0);
    lua_pushstring(L, "");
    return 3;
}

/// Extra space between wrapped lines. Zero unless set, and a number either
/// way: WorldMapFrame adds it to a font height on the line after asking.
int lua_FontString_GetSpacing(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? w->lineSpacing : 0.0);
    return 1;
}

int lua_FontString_SetSpacing(lua_State* L) {
    if (auto* w = widgetOf(L, 1))
        w->lineSpacing = static_cast<float>(luaL_optnumber(L, 2, 0.0));
    return 0;
}

/// SetFontObject(obj) where obj is one of the shared font objects, which carry
/// a height and a colour. FrameXML reaches for these more than three thousand
/// times, so a FontString that ignores them is the wrong size and colour nearly
/// everywhere.
int lua_FontString_SetFontObject(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    if (lua_isstring(L, 2)) {              // by name
        lua_getglobal(L, lua_tostring(L, 2));
    } else {
        lua_pushvalue(L, 2);
    }
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, "height");
        if (lua_isnumber(L, -1)) w->fontHeight = static_cast<float>(lua_tonumber(L, -1));
        lua_pop(L, 1);
        // Which typeface, not only how big. FrameXML sets its headings in
        // MORPHEUS and its damage numbers in SKURRI, and a font object is
        // where it says so.
        lua_getfield(L, -1, "font");
        if (lua_isstring(L, -1)) w->fontFace = lua_tostring(L, -1);
        lua_pop(L, 1);
        lua_getfield(L, -1, "outline");
        if (lua_isstring(L, -1)) w->fontOutline = lua_tostring(L, -1);
        lua_pop(L, 1);
        const char* keys[4] = {"r", "g", "b", "a"};
        for (int i = 0; i < 4; ++i) {
            lua_getfield(L, -1, keys[i]);
            if (lua_isnumber(L, -1)) w->color[i] = static_cast<float>(lua_tonumber(L, -1));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    return 0;
}

/// Attach the shared region methods to the table on top of the stack.
void installRegionMethods(lua_State* L, bool isTexture, bool isFontString) {
    auto set = [&](const char* name, lua_CFunction fn) {
        lua_pushcfunction(L, fn);
        lua_setfield(L, -2, name);
    };
    set("SetPoint", lua_Region_SetPoint);
    set("ClearAllPoints", lua_Region_ClearAllPoints);
    set("SetAllPoints", lua_Region_SetAllPoints);
    set("SetSize", lua_Region_SetSize);
    set("SetWidth", lua_Region_SetWidth);
    set("SetHeight", lua_Region_SetHeight);
    set("GetWidth", lua_Region_GetWidth);
    set("GetTextWidth", lua_Region_GetTextWidth);
    set("GetStringWidth", lua_Region_GetTextWidth);
    set("GetTextHeight", lua_Region_GetTextHeight);
    set("GetStringHeight", lua_Region_GetTextHeight);
    set("GetHeight", lua_Region_GetHeight);
    set("Show", lua_Region_Show);
    set("Hide", lua_Region_Hide);
    set("IsShown", lua_Region_IsShown);
    set("IsVisible", lua_Region_IsShown);
    set("SetAlpha", lua_Region_SetAlpha);
    set("GetAlpha", lua_Region_GetAlpha);
    set("SetVertexColor", lua_Region_SetVertexColor);
    set("SetDrawLayer", lua_Region_SetDrawLayer);
    if (isTexture) {
        set("SetTexture", lua_Texture_SetTexture);
        set("GetTexture", lua_Texture_GetTexture);
        set("SetTexCoord", lua_Texture_SetTexCoord);
    }
    if (isFontString) {
        set("SetText", lua_FontString_SetText);
        set("GetText", lua_FontString_GetText);
        set("SetJustifyH", lua_FontString_SetJustifyH);
        set("SetTextColor", lua_FontString_SetTextColor);
        set("SetFont", lua_FontString_SetFont);
        set("GetFont", lua_FontString_GetFont);
        set("GetSpacing", lua_FontString_GetSpacing);
        set("SetSpacing", lua_FontString_SetSpacing);
        set("SetFontObject", lua_FontString_SetFontObject);
    }
    // Anything still unimplemented stays a no-op rather than an error, which is
    // what keeps a large addon running while the surface is filled in.
    // The same enumerated set the frame metatable uses, and for the same
    // reason: a region carries data fields beside its methods, and answering
    // both with a no-op makes a field that was never set look present.
    // Built once and shared. This used to compile a fresh chunk for every
    // region created, which over a FrameXML load is thousands of compiles of
    // the same three lines.
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_region_mt");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        luaL_dostring(L,
            "local known = __WoweeWidgetMethods or {} "
            "return { __index = function(_, k) "
            "  if type(k)=='string' and known[k] then return function() end end "
            "end }");
        lua_pushvalue(L, -1);
        lua_setfield(L, LUA_REGISTRYINDEX, "wowee_region_mt");
    }
    lua_setmetatable(L, -2);
}

} // namespace



// ── Backdrop and StatusBar ──────────────────────────────────────────────────

int lua_Frame_SetBackdrop(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    if (!lua_istable(L, 2)) {          // SetBackdrop(nil) clears it
        w->hasBackdrop = false;
        return 0;
    }
    w->hasBackdrop = true;
    auto str = [&](const char* key, std::string& out) {
        lua_getfield(L, 2, key);
        if (lua_isstring(L, -1)) out = lua_tostring(L, -1);
        lua_pop(L, 1);
    };
    auto num = [&](const char* key, float& out) {
        lua_getfield(L, 2, key);
        if (lua_isnumber(L, -1)) out = static_cast<float>(lua_tonumber(L, -1));
        lua_pop(L, 1);
    };
    str("bgFile", w->bgFile);
    str("edgeFile", w->edgeFile);
    num("edgeSize", w->edgeSize);
    // tileSize describes the background's repeat, and edgeSize the border tile.
    // Where only one is given the other is the sensible stand-in.
    num("tileSize", w->edgeSize);
    num("edgeSize", w->edgeSize);
    lua_getfield(L, 2, "tile");
    w->tileBackground = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);

    lua_getfield(L, 2, "insets");
    if (lua_istable(L, -1)) {
        auto inset = [&](const char* key, float& out) {
            lua_getfield(L, -1, key);
            if (lua_isnumber(L, -1)) out = static_cast<float>(lua_tonumber(L, -1));
            lua_pop(L, 1);
        };
        inset("left", w->insetLeft);
        inset("right", w->insetRight);
        inset("top", w->insetTop);
        inset("bottom", w->insetBottom);
    }
    lua_pop(L, 1);
    return 0;
}

int lua_Frame_SetBackdropColor(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->backdropColor[0] = static_cast<float>(luaL_optnumber(L, 2, 1.0));
        w->backdropColor[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->backdropColor[2] = static_cast<float>(luaL_optnumber(L, 4, 1.0));
        w->backdropColor[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}

int lua_Frame_SetBackdropBorderColor(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->borderColor[0] = static_cast<float>(luaL_optnumber(L, 2, 1.0));
        w->borderColor[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->borderColor[2] = static_cast<float>(luaL_optnumber(L, 4, 1.0));
        w->borderColor[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}

int lua_StatusBar_SetMinMaxValues(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->barMin = static_cast<float>(luaL_optnumber(L, 2, 0.0));
        w->barMax = static_cast<float>(luaL_optnumber(L, 3, 1.0));
    }
    return 0;
}
int lua_StatusBar_GetMinMaxValues(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? w->barMin : 0.0);
    lua_pushnumber(L, w ? w->barMax : 1.0);
    return 2;
}
int lua_StatusBar_SetValue(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->barValue = static_cast<float>(luaL_optnumber(L, 2, 0.0));
    return 0;
}
/// SetCooldown(start, duration) — both on GetTime's clock. A zero duration is
/// how FrameXML clears one, and it must read as nothing running rather than as
/// a sweep that never finishes.
/// An edit box keeps its own text, so SetText on one is not the font string's.
/// FrameXML uses the same name for both and the widget decides which it means.
int lua_EditBox_SetText(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    w->editText = luaL_optstring(L, 2, "");
    w->cursorPos = w->editText.size();
    return 0;
}
int lua_EditBox_GetText(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushstring(L, w ? w->editText.c_str() : "");
    return 1;
}
int lua_EditBox_GetNumber(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? std::atof(w->editText.c_str()) : 0.0);
    return 1;
}
int lua_EditBox_Insert(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    const std::string add = luaL_optstring(L, 2, "");
    const size_t at = std::min(w->cursorPos, w->editText.size());
    w->editText.insert(at, add);
    w->cursorPos = at + add.size();
    return 0;
}
int lua_EditBox_SetMaxLetters(lua_State* L) {
    if (auto* w = widgetOf(L, 1))
        w->editMaxLetters = static_cast<int>(luaL_optnumber(L, 2, 0));
    return 0;
}
int lua_EditBox_SetNumeric(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->editNumeric = lua_toboolean(L, 2) != 0;
    return 0;
}
int lua_EditBox_SetMultiLine(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) w->editMultiLine = lua_toboolean(L, 2) != 0;
    return 0;
}
int lua_EditBox_SetCursorPosition(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    const double at = luaL_optnumber(L, 2, 0.0);
    w->cursorPos = static_cast<size_t>(std::clamp(
        at, 0.0, static_cast<double>(w->editText.size())));
    return 0;
}
int lua_EditBox_GetCursorPosition(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? static_cast<double>(w->cursorPos) : 0.0);
    return 1;
}

int lua_Cooldown_SetCooldown(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->cooldownStart = luaL_optnumber(L, 2, 0.0);
        w->cooldownDuration = luaL_optnumber(L, 3, 0.0);
    }
    return 0;
}
int lua_Cooldown_GetCooldownTimes(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    // Milliseconds, which is what this one answers in.
    lua_pushnumber(L, w ? w->cooldownStart * 1000.0 : 0.0);
    lua_pushnumber(L, w ? w->cooldownDuration * 1000.0 : 0.0);
    return 2;
}
int lua_Cooldown_Clear(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) { w->cooldownStart = 0.0; w->cooldownDuration = 0.0; }
    return 0;
}

int lua_Slider_SetValueStep(lua_State* L) {
    if (auto* w = widgetOf(L, 1))
        w->sliderStep = static_cast<float>(luaL_optnumber(L, 2, 0.0));
    return 0;
}
int lua_Slider_GetValueStep(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? w->sliderStep : 0.0);
    return 1;
}
/// The draggable part. Given a path rather than a texture here, the same way
/// the button art setters take one.
int lua_Slider_SetThumbTexture(lua_State* L) {
    auto* w = widgetOf(L, 1);
    if (!w) return 0;
    if (lua_isstring(L, 2)) {
        w->thumbTexture = lua_tostring(L, 2);
    } else if (lua_istable(L, 2)) {
        auto* tree = wowee::addons::getWidgetTree(L);
        const auto* t = tree ? tree->get(widgetIdOf(L, 2)) : nullptr;
        if (t) w->thumbTexture = t->texturePath;
    }
    return 0;
}

int lua_StatusBar_GetValue(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushnumber(L, w ? w->barValue : 0.0);
    return 1;
}
int lua_StatusBar_SetStatusBarTexture(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        // Takes a path or an existing texture object, and addons use both.
        if (lua_isstring(L, 2)) w->barTexture = lua_tostring(L, 2);
        else if (lua_istable(L, 2)) {
            if (auto* tex = widgetOf(L, 2)) w->barTexture = tex->texturePath;
        }
    }
    return 0;
}
int lua_StatusBar_SetStatusBarColor(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        w->barColor[0] = static_cast<float>(luaL_optnumber(L, 2, 1.0));
        w->barColor[1] = static_cast<float>(luaL_optnumber(L, 3, 1.0));
        w->barColor[2] = static_cast<float>(luaL_optnumber(L, 4, 1.0));
        w->barColor[3] = static_cast<float>(luaL_optnumber(L, 5, 1.0));
    }
    return 0;
}
int lua_StatusBar_SetOrientation(lua_State* L) {
    if (auto* w = widgetOf(L, 1)) {
        const std::string o = luaL_optstring(L, 2, "HORIZONTAL");
        w->barVertical = (o == "VERTICAL");
    }
    return 0;
}

// Frame method: frame:CreateTexture(name, layer) → a real region
static int lua_Frame_CreateTexture(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const uint32_t parent = widgetIdOf(L, 1);
    const char* name = luaL_optstring(L, 2, "");
    const char* layer = luaL_optstring(L, 3, "ARTWORK");

    lua_newtable(L);
    if (tree) {
        const uint32_t id = tree->create(wowee::ui::WidgetKind::Texture, parent, name ? name : "");
        if (auto* w = tree->get(id)) w->layer = wowee::ui::parseDrawLayer(layer);
        lua_pushinteger(L, static_cast<lua_Integer>(id));
        lua_setfield(L, -2, "__wid");
    }
    installRegionMethods(L, /*isTexture=*/true, /*isFontString=*/false);
    if (name && *name) {
        lua_pushvalue(L, -1);
        lua_setglobal(L, name);
    }
    return 1;
}

// Frame method: frame:CreateFontString(name, layer, template) → a real region
static int lua_Frame_CreateFontString(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const uint32_t parent = widgetIdOf(L, 1);
    const char* name = luaL_optstring(L, 2, "");
    const char* layer = luaL_optstring(L, 3, "ARTWORK");

    lua_newtable(L);
    if (tree) {
        const uint32_t id = tree->create(wowee::ui::WidgetKind::FontString, parent, name ? name : "");
        if (auto* w = tree->get(id)) w->layer = wowee::ui::parseDrawLayer(layer);
        lua_pushinteger(L, static_cast<lua_Integer>(id));
        lua_setfield(L, -2, "__wid");
    }
    lua_pushstring(L, "");
    lua_setfield(L, -2, "_text");
    installRegionMethods(L, /*isTexture=*/false, /*isFontString=*/true);
    if (name && *name) {
        lua_pushvalue(L, -1);
        lua_setglobal(L, name);
    }
    return 1;
}

static int lua_GetFramerate(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(ImGui::GetIO().Framerate));
    return 1;
}

// GetCursorPosition() → x, y (screen coordinates, origin top-left)
static int lua_GetCursorPosition(lua_State* L) {
    const auto& io = ImGui::GetIO();
    lua_pushnumber(L, io.MousePos.x);
    lua_pushnumber(L, io.MousePos.y);
    return 2;
}

// GetScreenWidth() → width
/// The screen in interface units, not pixels — which is what FrameXML means
/// by it. On a 1528-tall window GetScreenHeight() is 768, the same as it would
/// be on any other, and a frame sized against it comes out the same size.
static int lua_GetScreenWidth(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const auto* root = tree ? tree->get(tree->rootId()) : nullptr;
    if (root && root->rectW > 0.0f) { lua_pushnumber(L, root->rectW); return 1; }
    auto* svc = getLuaServices(L);
    auto* window = svc ? svc->window : nullptr;
    lua_pushnumber(L, window ? window->getWidth() : 1920);
    return 1;
}

// GetScreenHeight() → height
static int lua_GetScreenHeight(lua_State* L) {
    auto* tree = wowee::addons::getWidgetTree(L);
    const auto* root = tree ? tree->get(tree->rootId()) : nullptr;
    if (root && root->rectH > 0.0f) { lua_pushnumber(L, root->rectH); return 1; }
    auto* svc = getLuaServices(L);
    auto* window = svc ? svc->window : nullptr;
    lua_pushnumber(L, window ? window->getHeight() : 1080);
    return 1;
}

// Modifier key state queries using ImGui IO

static int lua_Frame_SetPoint(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    const char* point = luaL_optstring(L, 2, "CENTER");
    // Store point info in frame table
    lua_pushstring(L, point);
    lua_setfield(L, 1, "__point");
    // Optional x/y offsets (args 4,5 if relativeTo is given, or 3,4 if not)
    double xOfs = 0, yOfs = 0;
    if (lua_isnumber(L, 4)) { xOfs = lua_tonumber(L, 4); yOfs = lua_tonumber(L, 5); }
    else if (lua_isnumber(L, 3)) { xOfs = lua_tonumber(L, 3); yOfs = lua_tonumber(L, 4); }
    lua_pushnumber(L, xOfs);
    lua_setfield(L, 1, "__xOfs");
    lua_pushnumber(L, yOfs);
    lua_setfield(L, 1, "__yOfs");
    return 0;
}

static int lua_Frame_SetSize(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    double w = luaL_optnumber(L, 2, 0);
    double h = luaL_optnumber(L, 3, 0);
    lua_pushnumber(L, w);
    lua_setfield(L, 1, "__width");
    lua_pushnumber(L, h);
    lua_setfield(L, 1, "__height");
    return 0;
}

static int lua_Frame_SetWidth(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_pushnumber(L, luaL_checknumber(L, 2));
    lua_setfield(L, 1, "__width");
    return 0;
}

static int lua_Frame_SetHeight(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_pushnumber(L, luaL_checknumber(L, 2));
    lua_setfield(L, 1, "__height");
    return 0;
}

static int lua_Frame_GetWidth(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__width");
    if (lua_isnil(L, -1)) { lua_pop(L, 1); lua_pushnumber(L, 0); }
    return 1;
}

static int lua_Frame_GetHeight(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__height");
    if (lua_isnil(L, -1)) { lua_pop(L, 1); lua_pushnumber(L, 0); }
    return 1;
}

static int lua_Frame_GetCenter(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__xOfs");
    double x = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0;
    lua_pop(L, 1);
    lua_getfield(L, 1, "__yOfs");
    double y = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0;
    lua_pop(L, 1);
    lua_pushnumber(L, x);
    lua_pushnumber(L, y);
    return 2;
}

static int lua_Frame_SetAlpha(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_pushnumber(L, luaL_checknumber(L, 2));
    lua_setfield(L, 1, "__alpha");
    return 0;
}

static int lua_Frame_GetAlpha(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__alpha");
    if (lua_isnil(L, -1)) { lua_pop(L, 1); lua_pushnumber(L, 1.0); }
    return 1;
}

static int lua_Frame_SetParent(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    if (lua_istable(L, 2) || lua_isnil(L, 2)) {
        lua_pushvalue(L, 2);
        lua_setfield(L, 1, "__parent");
    }
    return 0;
}

static int lua_Frame_GetParent(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "__parent");
    return 1;
}


/// Records a global FrameXML or an addon asked for and did not find. Logged
/// once per name; the set is reported at shutdown so the gap can be read off a
/// run rather than guessed at.
static int lua_RecordMissingApi(lua_State* L) {
    const char* name = luaL_optstring(L, 1, "");
    if (name && *name) {
        // Once per name, so a warning here is a bounded list rather than
        // a stream, and it is the only trace of a gap as it happens.
        LOG_WARNING("[Lua] missing API called: ", name);
        missingApiNames().insert(name);
    }
    return 0;
}

// CreateFrame(frameType, name, parent, template)
static int lua_CreateFrame(lua_State* L) {
    const char* frameType = luaL_optstring(L, 1, "Frame");
    const char* name = luaL_optstring(L, 2, nullptr);

    // Create the frame table
    lua_newtable(L);

    // Record the parent table, not only the widget id. GetParent() is
    // everywhere in FrameXML — a nested button's OnLoad opens with
    // self:GetParent().toggle = self — and it answered nil for every frame
    // ever created, because only an explicit SetParent recorded one. That
    // failed the template declaring the button, so the button's owner never
    // got its size, and the loop sizing a list by its first button's height
    // divided by zero.
    if (lua_istable(L, 3)) {
        lua_pushvalue(L, 3);
        lua_setfield(L, -2, "__parent");
    } else {
        // A name, or nothing at all, which means UIParent.
        if (lua_isstring(L, 3)) lua_getglobal(L, lua_tostring(L, 3));
        else lua_getglobal(L, "UIParent");
        if (lua_istable(L, -1)) lua_setfield(L, -2, "__parent");
        else lua_pop(L, 1);
    }

    // Back it with a real widget so its geometry is somewhere the renderer can
    // reach. Parent is the third argument when given, and UIParent otherwise,
    // which is what an addon means by leaving it out.
    if (auto* tree = wowee::addons::getWidgetTree(L)) {
        uint32_t parent = 0;
        if (lua_istable(L, 3)) {
            parent = widgetIdOf(L, 3);
        } else if (lua_isstring(L, 3)) {
            lua_getglobal(L, lua_tostring(L, 3));
            if (lua_istable(L, -1)) parent = widgetIdOf(L, lua_gettop(L));
            lua_pop(L, 1);
        }
        const uint32_t id = tree->create(wowee::ui::WidgetKind::Frame, parent,
                                         name ? name : "");
        // A Button takes the mouse without being asked; a plain Frame does not,
        // which is what EnableMouse is for.
        if (auto* w = tree->get(id)) {
            const std::string ft = frameType ? frameType : "Frame";
            w->mouseEnabled = (ft == "Button" || ft == "CheckButton");
            w->isStatusBar = (ft == "StatusBar");
            // A slider takes the mouse by nature: it exists to be dragged.
            w->isSlider = (ft == "Slider");
            w->isCooldown = (ft == "Cooldown");
            // An edit box is clicked into, so it takes the mouse as well.
            w->isEditBox = (ft == "EditBox");
            if (w->isEditBox) {
                w->mouseEnabled = true;
                lua_pushboolean(L, 1);
                lua_setfield(L, -2, "__isEditBox");
            }
            if (w->isSlider) w->mouseEnabled = true;
        }
        lua_pushinteger(L, static_cast<lua_Integer>(id));
        lua_setfield(L, -2, "__wid");

        // Remember the table against its widget id so input dispatch can get
        // back from a hit to the frame whose scripts must run.
        lua_getglobal(L, "__WoweeFramesByWid");
        if (lua_istable(L, -1)) {
            lua_pushinteger(L, static_cast<lua_Integer>(id));
            lua_pushvalue(L, -3);
            lua_rawset(L, -3);
        }
        lua_pop(L, 1);
    }

    // Set frame name
    if (name && *name) {
        lua_pushstring(L, name);
        lua_setfield(L, -2, "__name");
        // Also set as a global so other addons can find it by name
        lua_pushvalue(L, -1);
        lua_setglobal(L, name);
    }

    // Set initial visibility
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, "__visible");

    // Apply frame metatable with methods
    lua_getglobal(L, "__WoweeFrameMT");
    lua_setmetatable(L, -2);

    // The fourth argument names a template, which FrameXML uses constantly:
    // CreateFrame("BUTTON", name, self, "OptionsListButtonTemplate"). Ignoring
    // it was not merely a missing feature. OptionsList_OnLoad makes one button,
    // divides the list's height by that button's height to decide how many fit,
    // and loops to that number — so a template that never arrives means no
    // size, a height of zero, a count of (h-8)/0, and Lua divides by zero
    // happily. The loop then creates frames under fresh names until memory runs
    // out, which is exactly what froze the client on VideoOptionsFrame.
    //
    // Applied after the metatable, so the template's body can call methods on
    // what it is given.
    if (const char* templates = lua_isstring(L, 4) ? lua_tostring(L, 4) : nullptr) {
        const std::string list(templates);
        size_t start = 0;
        while (start <= list.size()) {
            const size_t comma = list.find(',', start);
            std::string one = list.substr(
                start, comma == std::string::npos ? std::string::npos : comma - start);
            const size_t b = one.find_first_not_of(" \t");
            const size_t e = one.find_last_not_of(" \t");
            one = (b == std::string::npos) ? std::string() : one.substr(b, e - b + 1);

            if (!one.empty()) {
                lua_getglobal(L, "__WoweeTemplates");
                if (lua_istable(L, -1)) {
                    lua_getfield(L, -1, one.c_str());
                    if (lua_isfunction(L, -1)) {
                        lua_pushvalue(L, -3);            // the frame
                        if (lua_pcall(L, 1, 0, 0) != 0) {
                            // Once per template. A template that fails fails
                            // for every frame using it, and the loop this very
                            // failure causes then repeats it: one run wrote the
                            // same line 675,000 times, which cost more than the
                            // fault it was reporting.
                            static std::set<std::string> reported;
                            if (reported.insert(one).second) {
                                LOG_WARNING("CreateFrame: template '", one, "' failed: ",
                                            lua_tostring(L, -1) ? lua_tostring(L, -1) : "?");
                            }
                            lua_pop(L, 1);               // error
                        }
                    } else {
                        lua_pop(L, 1);                   // not a function
                    }
                }
                lua_pop(L, 1);                           // __WoweeTemplates
            }
            if (comma == std::string::npos) break;
            start = comma + 1;
        }

        // Built from a template here, so it is loaded here — which is what
        // CreateFrame does in the real client. The XML path does not pass a
        // template to this function; it applies them separately and fires
        // OnLoad once, after the frame's own body. So this covers exactly the
        // frames Lua builds, and OptionsList_OnLoad builds a list of them:
        // their OnLoad is what gives each button the .text it is asked for
        // moments later.
        lua_getfield(L, -1, "__scripts");
        if (lua_istable(L, -1)) {
            lua_getfield(L, -1, "OnLoad");
            if (lua_isfunction(L, -1)) {
                lua_pushvalue(L, -3);            // the frame
                if (lua_pcall(L, 1, 0, 0) != 0) {
                    LOG_WARNING("CreateFrame: OnLoad failed: ",
                                lua_tostring(L, -1) ? lua_tostring(L, -1) : "?");
                    lua_pop(L, 1);
                }
            } else {
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
    }

    return 1;
}

// --- WoW Utility Functions ---

// strsplit(delimiter, str) — WoW's string split

LuaEngine::LuaEngine() = default;

LuaEngine::~LuaEngine() {
    shutdown();
}

bool LuaEngine::initialize() {
    if (L_) return true;

    L_ = luaL_newstate();
    if (!L_) {
        LOG_ERROR("LuaEngine: failed to create Lua state");
        return false;
    }

    // Open safe standard libraries (no io, os, debug, package)
    luaopen_base(L_);
    luaopen_table(L_);
    luaopen_string(L_);
    luaopen_math(L_);

    // Remove unsafe globals from base library.
    //
    // newproxy is not among them, despite the name. It returns a userdata with
    // a fresh metatable and reaches nothing else; what it buys is __index and
    // __newindex on a value that cannot be tampered with, which is exactly how
    // Blizzard's own RestrictedFrames builds secure frame handles. Removing it
    // cost us SecureHandlerTemplates and everything inheriting from it.
    const char* unsafeGlobals[] = {
        "dofile", "loadfile", "load", "collectgarbage", nullptr
    };
    for (const char** g = unsafeGlobals; *g; ++g) {
        lua_pushnil(L_);
        lua_setglobal(L_, *g);
    }

    // Publish the widget tree before any API is registered, so a script that
    // runs during registration still finds it.
    lua_pushlightuserdata(L_, &widgets_);
    lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_widget_tree");

    // The engine itself, for the few bindings that need to do more than touch a
    // widget — taking focus fires handlers on the frame losing it as well as
    // the one gaining it, and only the engine knows which that was.
    lua_pushlightuserdata(L_, this);
    lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_lua_engine");

    registerCoreAPI();
    registerEventAPI();

    // Last, so every bootstrap block above reads a _G that answers honestly.
    installMissingApiFallback();

    LOG_INFO("LuaEngine: initialized (Lua 5.1)");
    return true;
}

void LuaEngine::shutdown() {
    reportMissingApi();
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
        LOG_INFO("LuaEngine: shut down");
    }
}

void LuaEngine::setGameHandler(game::GameHandler* handler) {
    gameHandler_ = handler;
    if (L_) {
        lua_pushlightuserdata(L_, handler);
        lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_game_handler");
    }
}

void LuaEngine::setLuaServices(const LuaServices& services) {
    luaServices_ = services;
    if (L_) {
        lua_pushlightuserdata(L_, &luaServices_);
        lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_lua_services");
    }
}


void LuaEngine::registerCoreAPI() {
    // Override print() to go to chat
    lua_pushcfunction(L_, lua_wow_print);
    lua_setglobal(L_, "print");

    lua_pushcfunction(L_, [](lua_State* L) -> int {
        LOG_WARNING("[FrameXML] ", luaL_optstring(L, 1, ""));
        return 0;
    });
    lua_setglobal(L_, "__WoweeLogWarning");

    // WoW API stubs
    lua_pushcfunction(L_, lua_wow_message);
    lua_setglobal(L_, "message");

    // --- Per-domain Lua API registration ---
    registerUnitLuaAPI(L_);
    registerSpellLuaAPI(L_);
    registerInventoryLuaAPI(L_);
    registerQuestLuaAPI(L_);
    registerSocialLuaAPI(L_);
    registerSystemLuaAPI(L_);
    registerActionLuaAPI(L_);

    // WoW aliases
    lua_getglobal(L_, "string");
    lua_getfield(L_, -1, "format");
    lua_setglobal(L_, "format");
    lua_pop(L_, 1);  // pop string table

    // tinsert/tremove aliases
    lua_getglobal(L_, "table");
    lua_getfield(L_, -1, "insert");
    lua_setglobal(L_, "tinsert");
    lua_getfield(L_, -1, "remove");
    lua_setglobal(L_, "tremove");
    lua_pop(L_, 1);  // pop table

    // WoW's Lua predates the 5.1 module tables and exposes most of math, string
    // and table as bare globals as well. FrameXML calls min, ceil and PI
    // directly at file scope, and one nil there loses the whole file: mainmenubar
    // and spellbookframe each died on a single arithmetic name.
    //
    // Skipped rather than assumed where the vendored Lua lacks one — getn is
    // compiled out here, and setting a global to nil would be no better than
    // leaving it absent.
    struct Alias { const char* lib; const char* field; const char* global; };
    static constexpr Alias kAliases[] = {
        {"math", "abs", "abs"},        {"math", "ceil", "ceil"},
        {"math", "floor", "floor"},    {"math", "max", "max"},
        {"math", "min", "min"},        {"math", "fmod", "mod"},
        {"math", "sqrt", "sqrt"},      {"math", "random", "random"},
        {"string", "gsub", "gsub"},    {"string", "sub", "strsub"},
        {"string", "len", "strlen"},   {"string", "upper", "strupper"},
        {"string", "lower", "strlower"}, {"string", "find", "strfind"},
        {"string", "rep", "strrep"},   {"string", "byte", "strbyte"},
        {"string", "char", "strchar"}, {"string", "match", "strmatch"},
        {"string", "gmatch", "gmatch"}, {"table", "sort", "sort"},
        {"table", "getn", "getn"},     {"table", "concat", "tconcat"},
    };
    for (const auto& a : kAliases) {
        lua_getglobal(L_, a.lib);
        if (lua_istable(L_, -1)) {
            lua_getfield(L_, -1, a.field);
            if (lua_isnil(L_, -1)) lua_pop(L_, 1);
            else lua_setglobal(L_, a.global);
        }
        lua_pop(L_, 1);
    }
    // A constant, not a function, so the fallback's rule for SCREAMING_SNAKE
    // names leaves it nil and gametime.lua does arithmetic on nothing.
    lua_getglobal(L_, "math");
    lua_getfield(L_, -1, "pi");
    lua_setglobal(L_, "PI");
    lua_pop(L_, 1);

    // WoW-specific and not derivable from a standard library.
    bootstrap(
        "function wipe(t) for k in pairs(t) do t[k] = nil end return t end\n"
        "function strtrim(s, chars)\n"
        "  chars = chars or ' \\t\\r\\n'\n"
        "  local p = '[' .. chars:gsub('(%W)', '%%%1') .. ']'\n"
        "  return (s:gsub('^' .. p .. '*', ''):gsub(p .. '*$', ''))\n"
        "end\n");

    // SlashCmdList table — addons register slash commands here
    lua_newtable(L_);
    lua_setglobal(L_, "SlashCmdList");

    // Frame metatable with methods
    lua_newtable(L_);  // metatable
    lua_pushvalue(L_, -1);
    lua_setfield(L_, -2, "__index"); // metatable.__index = metatable

    // Defined with the other edit-box bindings further down; declared here
    // because the table below refers to them first.
    int lua_EditBox_SetFocus(lua_State* L);
    int lua_EditBox_ClearFocus(lua_State* L);
    int lua_EditBox_HasFocus(lua_State* L);

    static const struct luaL_Reg frameMethods[] = {
        {"RegisterEvent",   lua_Frame_RegisterEvent},
        {"UnregisterEvent", lua_Frame_UnregisterEvent},
        {"SetScript",       lua_Frame_SetScript},
        {"GetScript",       lua_Frame_GetScript},
        {"GetName",         lua_Frame_GetName},
        {"Show",            lua_Region_Show},
        {"Hide",            lua_Region_Hide},
        {"IsShown",         lua_Region_IsShown},
        {"IsVisible",       lua_Region_IsShown}, // alias
        // Geometry goes through the widget tree. The older table-field
        // versions kept the numbers where only Lua could see them, which is
        // why a frame could be sized and positioned and still never appear.
        {"SetPoint",        lua_Region_SetPoint},
        {"ClearAllPoints",  lua_Region_ClearAllPoints},
        {"SetAllPoints",    lua_Region_SetAllPoints},
        {"SetSize",         lua_Region_SetSize},
        {"SetWidth",        lua_Region_SetWidth},
        {"SetHeight",       lua_Region_SetHeight},
        {"GetWidth",        lua_Region_GetWidth},
        {"GetTextWidth",    lua_Region_GetTextWidth},
        {"GetStringWidth",  lua_Region_GetTextWidth},
        {"GetTextHeight",   lua_Region_GetTextHeight},
        {"GetStringHeight", lua_Region_GetTextHeight},
        {"GetHeight",       lua_Region_GetHeight},
        {"GetCenter",       lua_Frame_GetCenter},
        {"SetAlpha",        lua_Region_SetAlpha},
        {"GetAlpha",        lua_Region_GetAlpha},
        {"EnableMouse",     lua_Frame_EnableMouse},
        {"IsMouseEnabled",  lua_Frame_IsMouseEnabled},
        {"SetBackdrop",           lua_Frame_SetBackdrop},
        {"SetBackdropColor",      lua_Frame_SetBackdropColor},
        {"SetBackdropBorderColor",lua_Frame_SetBackdropBorderColor},
        {"SetMinMaxValues",       lua_StatusBar_SetMinMaxValues},
        {"GetMinMaxValues",       lua_StatusBar_GetMinMaxValues},
        {"SetValue",              lua_StatusBar_SetValue},
        {"GetValue",              lua_StatusBar_GetValue},
        {"SetStatusBarTexture",   lua_StatusBar_SetStatusBarTexture},
        {"SetStatusBarColor",     lua_StatusBar_SetStatusBarColor},
        {"SetOrientation",        lua_StatusBar_SetOrientation},
        {"SetValueStep",          lua_Slider_SetValueStep},
        {"GetValueStep",          lua_Slider_GetValueStep},
        {"SetThumbTexture",       lua_Slider_SetThumbTexture},
        {"SetCooldown",           lua_Cooldown_SetCooldown},
        {"GetNumber",             lua_EditBox_GetNumber},
        {"Insert",                lua_EditBox_Insert},
        {"SetMaxLetters",         lua_EditBox_SetMaxLetters},        // The limit here is applied against the text's size in bytes, which is
        // what SetMaxBytes asks for; SetMaxLetters is the same field because
        // this counts the same way for both. Reporting it back matters more
        // than the distinction: an edit box that answers nothing for its limit
        // is one FrameXML will not stop typing into.
        {"SetMaxBytes",          lua_EditBox_SetMaxLetters},
        {"SetNumeric",            lua_EditBox_SetNumeric},
        {"SetMultiLine",          lua_EditBox_SetMultiLine},
        {"SetCursorPosition",     lua_EditBox_SetCursorPosition},
        {"GetCursorPosition",     lua_EditBox_GetCursorPosition},
        {"SetFocus",              lua_EditBox_SetFocus},
        {"ClearFocus",            lua_EditBox_ClearFocus},
        {"HasFocus",              lua_EditBox_HasFocus},
        {"GetCooldownTimes",      lua_Cooldown_GetCooldownTimes},
        {"SetFrameStrata",  lua_Frame_SetFrameStrata},
        {"SetFrameLevel",   lua_Frame_SetFrameLevel},
        {"SetParent",       lua_Frame_SetParent},
        {"GetParent",       lua_Frame_GetParent},
        {"CreateTexture",   lua_Frame_CreateTexture},
        {"CreateFontString", lua_Frame_CreateFontString},
        {nullptr, nullptr}
    };
    auto applyFrameMethods = [&]() {
        lua_getglobal(L_, "__WoweeFrameMT");
        for (const luaL_Reg* r = frameMethods; r->name; r++) {
            lua_pushcfunction(L_, r->func);
            lua_setfield(L_, -2, r->name);
        }
        lua_pop(L_, 1);
    };

    for (const luaL_Reg* r = frameMethods; r->name; r++) {
        lua_pushcfunction(L_, r->func);
        lua_setfield(L_, -2, r->name);
    }
    lua_setglobal(L_, "__WoweeFrameMT");

    // Commonly called frame methods that are no-ops for now, so an addon
    // calling one gets silence rather than an error.
    //
    // Anything bound in C above must not appear here. These run afterwards and
    // simply overwrite it, turning a working method into a no-op that still
    // answers — EnableMouse was defined here and so no frame ever took the
    // mouse, however plainly the call read in the addon.
    bootstrap(
        "local mt = __WoweeFrameMT\n"

        "function mt:GetFrameLevel() return self.__frameLevel or 1 end\n"
        "function mt:GetFrameStrata() return self.__strata or 'MEDIUM' end\n"
        "function mt:EnableMouseWheel(enable) end\n"
        "function mt:SetMovable(movable) end\n"
        "function mt:SetResizable(resizable) end\n"
        "function mt:RegisterForDrag(...) end\n"
        "function mt:SetClampedToScreen(clamped) end\n"
        "function mt:SetBackdrop(backdrop) end\n"
        "function mt:SetBackdropColor(...) end\n"
        "function mt:SetBackdropBorderColor(...) end\n"
        "function mt:SetID(id) self.__id = id end\n"
        "function mt:GetID() return self.__id or 0 end\n"
        "function mt:SetScale(scale) self.__scale = scale end\n"
        "function mt:GetScale() return self.__scale or 1.0 end\n"
        "function mt:GetEffectiveScale() return self.__scale or 1.0 end\n"
        "function mt:SetToplevel(top) end\n"
        "function mt:Raise() end\n"
        "function mt:Lower() end\n"
        "function mt:GetLeft() return 0 end\n"
        "function mt:GetRight() return 0 end\n"
        "function mt:GetTop() return 0 end\n"
        "function mt:GetBottom() return 0 end\n"
        "function mt:GetNumPoints() return 0 end\n"
        "function mt:GetPoint(n) return 'CENTER', nil, 'CENTER', 0, 0 end\n"
        "function mt:SetHitRectInsets(...) end\n"
        // Recorded, because a frame only receives the clicks it asks for.
        // FrameXML calls RegisterForClicks("LeftButtonUp", "RightButtonUp") on
        // the frames that want a context menu, and without this every frame
        // would answer a right-click whether it wanted one or not.
        "function mt:RegisterForClicks(...)\n"
        "    local set = {}\n"
        "    for i = 1, select('#', ...) do set[select(i, ...)] = true end\n"
        "    self.__clicks = set\n"
        "end\n"

        "function mt:SetAttribute(name, value) self['attr_'..name] = value end\n"
        "function mt:GetAttribute(name) return self['attr_'..name] end\n"
        "function mt:HookScript(scriptType, fn)\n"
        "    local orig = self.__scripts and self.__scripts[scriptType]\n"
        "    if orig then\n"
        "        self:SetScript(scriptType, function(...) orig(...); fn(...) end)\n"
        "    else\n"
        "        self:SetScript(scriptType, fn)\n"
        "    end\n"
        "end\n"
        "function mt:SetMinResize(...) end\n"
        "function mt:SetMaxResize(...) end\n"
        "function mt:StartMoving() end\n"
        "function mt:StopMovingOrSizing() end\n"
        "function mt:IsMouseOver() return false end\n"
        "function mt:GetObjectType() return 'Frame' end\n"
    );

    // Button art, which XML declares as <NormalTexture>, <HighlightTexture>,
    // <ButtonText> and so on. The catch-all below would answer these with a
    // no-op, which is worse than it sounds: the setter would appear to work and
    // the matching getter would hand back nil, so button:GetNormalTexture()
    // :SetVertexColor(...) — which FrameXML does constantly to grey out an
    // unusable action — fails somewhere far from the cause.
    bootstrap(
        "local mt = __WoweeFrameMT\n"
        // A path is as valid an argument as a texture, and FrameXML uses both:
        // LoadMicroButtonTextures does
        // self:SetDisabledTexture("Interface\\Buttons\\...-Disabled"), then the
        // next line asks for it back and calls SetDesaturated on it. Storing
        // the string verbatim handed a string back, and a string has no widget
        // methods at all. A path makes or updates the slot's own texture.
        "for _, slot in ipairs({'NormalTexture', 'PushedTexture', 'HighlightTexture',\n"
        "                       'DisabledTexture', 'CheckedTexture',\n"
        "                       'DisabledCheckedTexture'}) do\n"
        "    local key = '__' .. slot\n"
        "    local layer = (slot == 'HighlightTexture') and 'HIGHLIGHT' or 'ARTWORK'\n"
        "    mt['Set' .. slot] = function(self, tex)\n"
        "        if type(tex) == 'string' then\n"
        "            local existing = self[key]\n"
        "            if type(existing) == 'table' then existing:SetTexture(tex) return end\n"
        "            local made = self:CreateTexture(nil, layer)\n"
        "            made:SetTexture(tex)\n"
        "            made:SetAllPoints(self)\n"
        "            self[key] = made\n"
        "            return\n"
        "        end\n"
        "        self[key] = tex\n"
        "    end\n"
        "    mt['Get' .. slot] = function(self) return self[key] end\n"
        "end\n"
        // Attributes, and the OnAttributeChanged they fire.
        //
        // This is how FrameXML passes state to a handler without a global:
        // UIDropDownMenu_Initialize does
        // UIDropDownMenuDelegate:SetAttribute("initmenu", frame), and the
        // delegate's OnAttributeChanged is what actually sets
        // UIDROPDOWNMENU_INIT_MENU. No-opping SetAttribute left that nil, so
        // every menu built afterwards indexed nothing.
        "function mt:SetAttribute(name, value)\n"
        "    self.__attributes = self.__attributes or {}\n"
        "    self.__attributes[name] = value\n"
        "    local handler = self.__scripts and self.__scripts.OnAttributeChanged\n"
        "    if handler then handler(self, name, value) end\n"
        "end\n"
        "function mt:GetAttribute(a, b, c)\n"
        "    if not self.__attributes then return nil end\n"
        // The three-argument form names one attribute in pieces.
        "    local key = (b ~= nil) and ((a or '') .. b .. (c or '')) or a\n"
        "    return self.__attributes[key]\n"
        "end\n"
        // A scroll frame's content frame.
        // Nothing to scroll until the tree has been laid out, and zero is the
        // honest answer then. ScrollFrame_OnScrollRangeChanged compares the
        // bar value against this the moment a scroll frame is built.
        "function mt:GetVerticalScrollRange() return 0 end\n"
        "function mt:GetHorizontalScrollRange() return 0 end\n"
        "function mt:GetVerticalScroll() return 0 end\n"
        "function mt:GetHorizontalScroll() return 0 end\n"
        // Zero when unset, which is what the real client answers and what
        // FrameXML concatenates into a name without checking.
        "function mt:SetID(id) self.__id = id end\n"
        "function mt:GetID() return self.__id or 0 end\n"
        "function mt:SetScrollChild(child) self.__scrollChild = child end\n"
        "function mt:GetScrollChild() return self.__scrollChild end\n"
        "function mt:SetFontString(fs) self.__fontString = fs end\n"
        // Made on demand when a button is asked for one it has not been
        // given. Every button has a font string in the real client, and
        // FrameXML assumes it: FCF_SetTabColor does
        // minFrame:GetFontString():SetTextColor(...) without checking.
        "function mt:GetFontString()\n"
        "    if not self.__fontString then\n"
        "        self.__fontString = self:CreateFontString(nil, 'OVERLAY')\n"
        "    end\n"
        "    return self.__fontString\n"
        "end\n"
        // A button's text is its font string's text; keeping them apart means
        // SetText on the button quietly does nothing, which is how a bar full
        // of blank buttons happens.
        // An edit box keeps its own text; a button shows its font string's.
        // FrameXML calls SetText on both and the widget decides which it means.
        "function mt:SetText(text)\n"
        "    if self.__isEditBox then return __WoweeEditSetText(self, text) end\n"
        "    self.__text = text\n"
        "    if self.__fontString then self.__fontString:SetText(text) end\n"
        "end\n"
        "function mt:GetText()\n"
        "    if self.__isEditBox then return __WoweeEditGetText(self) end\n"
        "    if self.__fontString then return self.__fontString:GetText() end\n"
        "    return self.__text\n"
        "end\n"
    );

    // Catch-all for unimplemented widget methods. Frames are logic-only stubs (not
    // natively rendered), so UI-heavy addons call many widget methods we don't model
    // (sliders: SetMinMaxValues/SetValue; check buttons: SetChecked; buttons:
    // SetNormalTexture; etc.). Without this, the first such call raises "attempt to
    // call a nil value" and aborts the addon before it can register its slash commands.
    // WoW widget methods are PascalCase, so an unknown key starting with an uppercase
    // letter is treated as an unimplemented method (harmless no-op); anything else
    // falls through to nil so ordinary addon fields keep their normal (falsy) meaning.
    // The widget methods this stands in for, named rather than guessed at.
    //
    // Answering every PascalCase key with a no-op was wrong for data. A field
    // is PascalCase as readily as a method — textStatusBar.TextString is the
    // one that surfaced it — and a function is truthy, so FrameXML's own
    // "if (x.Field) then use it" ran the branch against something that was
    // never there. Methods and data cannot be told apart by shape: of the 307
    // method names FrameXML calls, eighteen read as nouns (AppendText,
    // NumLines, PageUp, AtBottom), and of the PascalCase fields it assigns,
    // several are method names held in a table.
    //
    // So the set is enumerated: every method FrameXML calls on a widget, plus
    // the standard widget API for addons. A name in it answers with a no-op;
    // anything else is data and answers nil, which is what it would be.
    bootstrap(
        "__WoweeWidgetMethods = {\n"
        "AddDoubleLine=1,AddHistoryLine=1,AddLine=1,AddMessage=1,AddTexture=1,\n"
        "AddToAutoHide=1,AllowAttributeChanges=1,Animate=1,AppendText=1,AtBottom=1,\n"
        "CallMethod=1,CanSaveTabardNow=1,ChildUpdate=1,Clear=1,ClearAllPoints=1,\n"
        "ClearBinding=1,ClearBindings=1,ClearFocus=1,ClearHistory=1,ClearLines=1,\n"
        "ClearModel=1,Click=1,CreateFontString=1,CreatePlayerArrowFrame=1,\n"
        "CreateTexture=1,CreateTitleRegion=1,CycleVariation=1,Disable=1,DrawQuestBlob=1,\n"
        "Dress=1,Enable=1,EnableKeyboard=1,EnableMouse=1,EnableMouseWheel=1,\n"
        "EnableSubtitles=1,FadeOut=1,Free=1,GetAlpha=1,GetAnchorType=1,GetAttribute=1,\n"
        "GetBackdrop=1,GetBottom=1,GetButtonState=1,GetCenter=1,GetChecked=1,\n"
        "GetCheckedTexture=1,GetChildList=1,GetChildren=1,GetColorRGB=1,\n"
        "GetCurrentValue=1,GetCursorPosition=1,GetDisabledCheckedTexture=1,\n"
        "GetDisabledTexture=1,GetDrawLayer=1,GetEffectiveAttribute=1,\n"
        "GetEffectiveScale=1,GetFieldSize=1,GetFileHeight=1,GetFileWidth=1,GetFont=1,\n"
        "GetFontObject=1,GetFontString=1,GetFrame=1,GetFrameLevel=1,GetFrameRef=1,\n"
        "GetFrameStrata=1,GetHeight=1,GetHighlightTexture=1,GetHorizontalScroll=1,\n"
        "GetHorizontalScrollRange=1,GetID=1,GetInputLanguage=1,GetInventorySlot=1,\n"
        "GetItem=1,GetLeft=1,GetLowerEmblemTexture=1,GetMessageInfo=1,GetMinimumWidth=1,\n"
        "GetMinMaxValues=1,GetMousePosition=1,GetName=1,GetNormalTexture=1,GetNumber=1,\n"
        "GetNumChildren=1,GetNumMessages=1,GetNumPoints=1,GetNumTooltips=1,\n"
        "GetObjectType=1,GetOwner=1,GetParent=1,GetPoint=1,GetPushedTexture=1,GetRect=1,\n"
        "GetRegionParent=1,GetRegions=1,GetRight=1,GetScale=1,GetScript=1,\n"
        "GetScrollChild=1,GetSize=1,GetSpacing=1,GetStatusBarTexture=1,\n"
        "GetStringHeight=1,GetStringWidth=1,GetTexCoord=1,GetText=1,GetTextColor=1,\n"
        "GetTextHeight=1,GetTexture=1,GetTextWidth=1,GetTooltipIndex=1,GetTop=1,\n"
        "GetUIPanel=1,GetUpperEmblemTexture=1,GetUTF8CursorPosition=1,GetValue=1,\n"
        "GetVertexColor=1,GetVerticalScroll=1,GetVerticalScrollRange=1,GetWidth=1,\n"
        "GetZoom=1,GetZoomLevels=1,HasFocus=1,HasScript=1,Hide=1,HideUIPanel=1,\n"
        "HighlightText=1,HookScript=1,IgnoreDepth=1,InitializeTabardColors=1,Insert=1,\n"
        "IsEnabled=1,IsEquippedItem=1,IsEventRegistered=1,IsMouseEnabled=1,\n"
        "IsMouseOver=1,IsObjectType=1,IsOwned=1,IsProtected=1,IsShown=1,IsUnderMouse=1,\n"
        "IsUnit=1,IsUserPlaced=1,IsVisible=1,LockHighlight=1,Lower=1,MoveUIPanel=1,\n"
        "New=1,NumLines=1,OnFinished=1,OnUpdate=1,PageDown=1,PageUp=1,PingLocation=1,\n"
        "Play=1,Raise=1,RefreshUnit=1,RefreshValue=1,RegisterAutoHide=1,RegisterEvent=1,\n"
        "RegisterForClicks=1,RegisterForDrag=1,ReleaseFrame=1,\n"
        "RemoveMessagesByAccessID=1,ReplaceIconTexture=1,Reset=1,Reuse=1,Run=1,\n"
        "RunAttribute=1,RunFor=1,Save=1,ScrollDown=1,ScrollToBottom=1,ScrollUp=1,\n"
        "SelectWindow=1,SetAction=1,SetAllPoints=1,SetAlpha=1,SetAlphaGradient=1,\n"
        "SetAnchorType=1,SetAttribute=1,SetAutoFocus=1,SetBackdrop=1,\n"
        "SetBackdropBorderColor=1,SetBackdropColor=1,SetBagItem=1,SetBinding=1,\n"
        "SetBindingClick=1,SetBindingItem=1,SetBindingMacro=1,SetBindingSpell=1,\n"
        "SetBlendMode=1,SetBorderAlpha=1,SetBorderScalar=1,SetBorderTexture=1,\n"
        "SetButtonState=1,SetBuybackItem=1,SetCamera=1,SetChecked=1,SetCheckedTexture=1,\n"
        "SetClampedToScreen=1,SetClampRectInsets=1,SetColorRGB=1,SetCooldown=1,\n"
        "SetCreature=1,SetCursorPosition=1,SetDesaturated=1,SetDisabledCheckedTexture=1,\n"
        "SetDisabledFontObject=1,SetDisabledTexture=1,SetDisplayValue=1,SetDrawLayer=1,\n"
        "SetEquipmentSet=1,SetFacing=1,SetFillAlpha=1,SetFillTexture=1,SetFocus=1,\n"
        "SetFont=1,SetFontObject=1,SetFontString=1,SetFormattedText=1,SetFrameLevel=1,\n"
        "SetFrameRate=1,SetFrameStrata=1,SetHeight=1,SetHighlightFontObject=1,\n"
        "SetHighlightTexture=1,SetHitRectInsets=1,SetHorizontalScroll=1,SetHyperlink=1,\n"
        "SetHyperlinkCompareItem=1,SetHyperlinksEnabled=1,SetID=1,SetInboxItem=1,\n"
        "SetInventoryItem=1,SetJustifyH=1,SetJustifyV=1,SetLFGCompletionReward=1,\n"
        "SetLFGDungeonReward=1,SetLight=1,SetLootItem=1,SetLootRollItem=1,SetMaxBytes=1,\n"
        "SetMaxLetters=1,SetMaxResize=1,SetMerchantCostItem=1,SetMerchantItem=1,\n"
        "SetMinimumWidth=1,SetMinMaxValues=1,SetMinResize=1,SetModel=1,SetModelScale=1,\n"
        "SetMovable=1,SetMultiLine=1,SetNormalFontObject=1,SetNormalTexture=1,\n"
        "SetNumber=1,SetNumeric=1,SetOwner=1,SetPadding=1,SetParent=1,SetPetAction=1,\n"
        "SetPlayerTextureHeight=1,SetPlayerTextureWidth=1,SetPoint=1,SetPosition=1,\n"
        "SetPossession=1,SetPropagateKeyboardInput=1,SetPushedTexture=1,SetQuestItem=1,\n"
        "SetQuestLogItem=1,SetQuestLogRewardSpell=1,SetQuestLogSpecialItem=1,\n"
        "SetQuestRewardSpell=1,SetResizable=1,SetRotation=1,SetScale=1,SetScript=1,\n"
        "SetScrollChild=1,SetSelection=1,SetSendMailItem=1,SetSequence=1,\n"
        "SetSequenceTime=1,SetShadowOffset=1,SetShapeshift=1,SetShown=1,SetSize=1,\n"
        "SetSpacing=1,SetSpell=1,SetSpellByID=1,SetStartDelay=1,SetStatusBarColor=1,\n"
        "SetStatusBarTexture=1,SetTexCoord=1,SetText=1,SetTextColor=1,SetTextHeight=1,\n"
        "SetTextInsets=1,SetTexture=1,SetToplevel=1,SetTotem=1,SetTracking=1,\n"
        "SetTradePlayerItem=1,SetTradeTargetItem=1,SetUIPanel=1,SetUnit=1,SetUnitAura=1,\n"
        "SetUnitBuff=1,SetUnitDebuff=1,SetUserPlaced=1,SetValue=1,SetValueStep=1,\n"
        "SetVertexColor=1,SetVerticalScroll=1,SetWidth=1,SetZoom=1,Show=1,ShowUIPanel=1,\n"
        "ShowUIPanelFailed=1,StartMovie=1,StartMoving=1,StartSizing=1,Stop=1,\n"
        "StopMovie=1,StopMovingOrSizing=1,ToggleInputLanguage=1,TryOn=1,\n"
        "UIParentManageFramePositions=1,UnlockHighlight=1,UnregisterAllEvents=1,\n"
        "UnregisterAutoHide=1,UnregisterEvent=1,UpdateColorByID=1,\n"
        "UpdateMouseOverTooltip=1,UpdateScrollChildRect=1,UpdateTooltip=1,\n"
        "UpdateUIPanelPositions=1,\n"
        "}\n"
    );
    bootstrap(
        "local mt = __WoweeFrameMT\n"
        "local methods = mt\n"
        "local known = __WoweeWidgetMethods\n"
        "local noop = function() end\n"
        "local seen = {}\n"
        "mt.__index = function(tbl, key)\n"
        "    local v = rawget(methods, key)\n"
        "    if v ~= nil then return v end\n"
        "    if type(key) ~= 'string' then return nil end\n"
        "    if known[key] then return noop end\n"
        // Recorded once so a method missing from the set is visible rather
        // than silently answering nil, which is the failure this trades for.
        // Not On*: those are script handler names, and reading one as a field
        // is how FrameXML asks whether a handler is set. Nil is the right
        // answer there, so recording it would be reporting correct behaviour
        // as a gap.
        "    if string.find(key, '^%u') and not string.find(key, '^On%u')\n"
        "       and not seen[key] then\n"
        "        seen[key] = true\n"
        "        if __WoweeRecordMissingApi then __WoweeRecordMissingApi('widget:' .. key) end\n"
        "    end\n"
        "    return nil\n"
        "end\n"
    );

    // The fallback is installed at the very end of initialize(), not here.
    // Everything below is still bootstrap Lua, and much of it opens with the
    // "LibStub = LibStub or {}" idiom — which reads nil only while _G answers
    // honestly. With the fallback already in place those never see nil, and
    // hang their tables off the fallback object instead of a fresh one.

    // Put the C bindings back over anything the Lua above defined with the same
    // name. That block exists to give unimplemented methods a harmless no-op,
    // and it runs later, so any name it shares with a real binding silently
    // replaces it — a method that answers and does nothing, which is far harder
    // to spot than one that errors. EnableMouse was lost this way and no frame
    // took the mouse at all; SetBackdrop and its two colour setters were about
    // to go the same way. Ordering the two makes the class of mistake
    // impossible rather than something to keep noticing.
    applyFrameMethods();

    // CreateFrame function
    lua_pushcfunction(L_, lua_EditBox_SetText);
    lua_setglobal(L_, "__WoweeEditSetText");
    lua_pushcfunction(L_, lua_EditBox_GetText);
    lua_setglobal(L_, "__WoweeEditGetText");

    lua_pushcfunction(L_, lua_CreateFrame);
    lua_setglobal(L_, "CreateFrame");

    // Cursor/screen/FPS functions
    lua_pushcfunction(L_, lua_GetCursorPosition);
    lua_setglobal(L_, "GetCursorPosition");
    lua_pushcfunction(L_, lua_GetScreenWidth);
    lua_setglobal(L_, "GetScreenWidth");
    lua_pushcfunction(L_, lua_GetScreenHeight);
    lua_setglobal(L_, "GetScreenHeight");
    lua_pushcfunction(L_, lua_GetFramerate);
    lua_setglobal(L_, "GetFramerate");

    // Frame event dispatch table
    lua_newtable(L_);
    lua_setglobal(L_, "__WoweeFrameEvents");

    // OnUpdate frame tracking table
    lua_newtable(L_);
    lua_setglobal(L_, "__WoweeOnUpdateFrames");

    // widget id -> frame table, so a hit test can find the scripts to run.
    lua_newtable(L_);
    lua_setglobal(L_, "__WoweeFramesByWid");

    // Where XML templates land. A virtual frame compiles to a function that
    // replays itself onto a real frame, and inherits= calls it; both halves are
    // emitted by the FrameXML loader and meet here.
    bootstrap(
        "__WoweeTemplates = {}\n"
        "local reported = {}\n"
        "function __WoweeMissingTemplate(name)\n"
        "  if reported[name] then return end\n"
        "  reported[name] = true\n"
        "  -- Said once per template. A frame inheriting one that never loaded\n"
        "  -- still gets built, just without whatever the template gave it,\n"
        "  -- which is a much better outcome than refusing the whole file.\n"
        "  __WoweeLogWarning('missing XML template: ' .. tostring(name))\n"
        "end\n");

    // C_Timer implementation via Lua (uses OnUpdate internally)
    bootstrap(
        "C_Timer = {}\n"
        "local timers = {}\n"
        "local timerFrame = CreateFrame('Frame', '__WoweeTimerFrame')\n"
        "timerFrame:SetScript('OnUpdate', function(self, elapsed)\n"
        "    local i = 1\n"
        "    while i <= #timers do\n"
        "        timers[i].remaining = timers[i].remaining - elapsed\n"
        "        if timers[i].remaining <= 0 then\n"
        "            local cb = timers[i].callback\n"
        "            table.remove(timers, i)\n"
        "            cb()\n"
        "        else\n"
        "            i = i + 1\n"
        "        end\n"
        "    end\n"
        "    if #timers == 0 then self:Hide() end\n"
        "end)\n"
        "timerFrame:Hide()\n"
        "function C_Timer.After(seconds, callback)\n"
        "    tinsert(timers, {remaining = seconds, callback = callback})\n"
        "    timerFrame:Show()\n"
        "end\n"
        "function C_Timer.NewTicker(seconds, callback, iterations)\n"
        "    local count = 0\n"
        "    local maxIter = iterations or -1\n"
        "    local ticker = {cancelled = false}\n"
        "    local function tick()\n"
        "        if ticker.cancelled then return end\n"
        "        count = count + 1\n"
        "        callback(ticker)\n"
        "        if maxIter > 0 and count >= maxIter then return end\n"
        "        C_Timer.After(seconds, tick)\n"
        "    end\n"
        "    C_Timer.After(seconds, tick)\n"
        "    function ticker:Cancel() self.cancelled = true end\n"
        "    return ticker\n"
        "end\n"
    );

    // DEFAULT_CHAT_FRAME with AddMessage method (used by many addons)
    bootstrap(
        "DEFAULT_CHAT_FRAME = {}\n"
        "function DEFAULT_CHAT_FRAME:AddMessage(text, r, g, b)\n"
        "    if r and g and b then\n"
        "        local hex = format('|cff%02x%02x%02x', "
        "            math.floor(r*255), math.floor(g*255), math.floor(b*255))\n"
        "        print(hex .. tostring(text) .. '|r')\n"
        "    else\n"
        "        print(tostring(text))\n"
        "    end\n"
        "end\n"
        "ChatFrame1 = DEFAULT_CHAT_FRAME\n"
    );

    // hooksecurefunc — hook a function to run additional code after it
    bootstrap(
        "function hooksecurefunc(tblOrName, nameOrFunc, funcOrNil)\n"
        "    local tbl, name, hook\n"
        "    if type(tblOrName) == 'table' then\n"
        "        tbl, name, hook = tblOrName, nameOrFunc, funcOrNil\n"
        "    else\n"
        "        tbl, name, hook = _G, tblOrName, nameOrFunc\n"
        "    end\n"
        "    local orig = tbl[name]\n"
        "    if type(orig) ~= 'function' then return end\n"
        "    tbl[name] = function(...)\n"
        "        local r = {orig(...)}\n"
        "        hook(...)\n"
        "        return unpack(r)\n"
        "    end\n"
        "end\n"
    );

    // LibStub — universal library version management used by Ace3 and virtually all addon libs.
    // This is the standard WoW LibStub implementation that addons embed/expect globally.
    bootstrap(
        // rawget, so the missing-API fallback cannot answer this. Read through
        // the metatable, "LibStub or {}" is never nil — it is the fallback
        // object — and the shim then hangs its tables off that instead of a
        // fresh one, so every library registering against it dies indexing a
        // field that was never really there.
        "local LibStub = rawget(_G, 'LibStub') or {}\n"
        "LibStub.libs = LibStub.libs or {}\n"
        "LibStub.minors = LibStub.minors or {}\n"
        "function LibStub:NewLibrary(major, minor)\n"
        "    assert(type(major) == 'string', 'LibStub:NewLibrary: bad argument #1 (string expected)')\n"
        "    minor = assert(tonumber(minor or (type(minor) == 'string' and minor:match('(%d+)'))), 'LibStub:NewLibrary: bad argument #2 (number expected)')\n"
        "    local oldMinor = self.minors[major]\n"
        "    if oldMinor and oldMinor >= minor then return nil end\n"
        "    local lib = self.libs[major] or {}\n"
        "    self.libs[major] = lib\n"
        "    self.minors[major] = minor\n"
        "    return lib, oldMinor\n"
        "end\n"
        "function LibStub:GetLibrary(major, silent)\n"
        "    if not self.libs[major] and not silent then\n"
        "        error('Cannot find a library instance of \"' .. tostring(major) .. '\".')\n"
        "    end\n"
        "    return self.libs[major], self.minors[major]\n"
        "end\n"
        "function LibStub:IterateLibraries() return pairs(self.libs) end\n"
        "setmetatable(LibStub, { __call = LibStub.GetLibrary })\n"
        "_G['LibStub'] = LibStub\n"
    );

    // CallbackHandler-1.0 — minimal implementation for Ace3-based addons
    bootstrap(
        "if LibStub then\n"
        "  local CBH = LibStub:NewLibrary('CallbackHandler-1.0', 7)\n"
        "  if CBH then\n"
        "    CBH.mixins = { 'RegisterCallback', 'UnregisterCallback', 'UnregisterAllCallbacks', 'Fire' }\n"
        "    function CBH:New(target, regName, unregName, unregAllName, onUsed)\n"
        "      local registry = setmetatable({}, { __index = CBH })\n"
        "      registry.callbacks = {}\n"
        "      target = target or {}\n"
        "      target[regName or 'RegisterCallback'] = function(self, event, method, ...)\n"
        "        if not registry.callbacks[event] then registry.callbacks[event] = {} end\n"
        "        local handler = type(method) == 'function' and method or self[method]\n"
        "        registry.callbacks[event][self] = handler\n"
        "      end\n"
        "      target[unregName or 'UnregisterCallback'] = function(self, event)\n"
        "        if registry.callbacks[event] then registry.callbacks[event][self] = nil end\n"
        "      end\n"
        "      target[unregAllName or 'UnregisterAllCallbacks'] = function(self)\n"
        "        for event, handlers in pairs(registry.callbacks) do handlers[self] = nil end\n"
        "      end\n"
        "      registry.Fire = function(self, event, ...)\n"
        "        if not self.callbacks[event] then return end\n"
        "        for obj, handler in pairs(self.callbacks[event]) do\n"
        "          handler(obj, event, ...)\n"
        "        end\n"
        "      end\n"
        "      return registry\n"
        "    end\n"
        "  end\n"
        "end\n"
    );

    // Noop stubs for commonly called functions that don't need implementation
    bootstrap(
        "function SetDesaturation() end\n"
        "function SetPortraitTexture() end\n"
        "function StopSound() end\n"
        "function UIParent_OnEvent() end\n"
        // Filling the screen, not sitting at a point on it. The widget tree's
        // root is already the screen, and a frame created with no anchors falls
        // to the centre-on-parent default with no size — so every frame
        // FrameXML hangs off UIParent inherited a zero-size box in the middle,
        // including its own UIParent, which fills this one. That is why the
        // player frame's name was drawn in the centre of the world.
        //
        // SetAllPoints with no argument fills the parent, which for these is
        // the root.
        "UIParent = CreateFrame('Frame', 'UIParent')\n"
        "UIParent:SetAllPoints()\n"
        "UIPanelWindows = {}\n"
        "WorldFrame = CreateFrame('Frame', 'WorldFrame')\n"
        "WorldFrame:SetAllPoints()\n"
        // GameTooltip: global tooltip frame used by virtually all addons
        "GameTooltip = CreateFrame('Frame', 'GameTooltip')\n"
        "GameTooltip.__lines = {}\n"
        "function GameTooltip:SetOwner(owner, anchor) self.__owner = owner; self.__anchor = anchor end\n"
        "function GameTooltip:ClearLines() self.__lines = {} end\n"
        "function GameTooltip:AddLine(text, r, g, b, wrap) table.insert(self.__lines, {text=text or '',r=r,g=g,b=b}) end\n"
        "function GameTooltip:AddDoubleLine(l, r, lr, lg, lb, rr, rg, rb) table.insert(self.__lines, {text=(l or '')..'  '..(r or '')}) end\n"
        "function GameTooltip:SetText(text, r, g, b) self.__lines = {{text=text or '',r=r,g=g,b=b}} end\n"
        "function GameTooltip:GetItem()\n"
        "    if self.__itemId and self.__itemId > 0 then\n"
        "        local name = GetItemInfo(self.__itemId)\n"
        "        local _, itemLink = GetItemInfo(self.__itemId)\n"
        "        return name, itemLink or ('|cffffffff|Hitem:'..self.__itemId..':0|h['..tostring(name)..']|h|r')\n"
        "    end\n"
        "    return nil\n"
        "end\n"
        "function GameTooltip:GetSpell()\n"
        "    if self.__spellId and self.__spellId > 0 then\n"
        "        local name = GetSpellInfo(self.__spellId)\n"
        "        return name, nil, self.__spellId\n"
        "    end\n"
        "    return nil\n"
        "end\n"
        "function GameTooltip:GetUnit() return nil end\n"
        "function GameTooltip:NumLines() return #self.__lines end\n"
        "function GameTooltip:GetText() return self.__lines[1] and self.__lines[1].text or '' end\n"
        "function GameTooltip:SetUnitBuff(unit, index, filter)\n"
        "    self:ClearLines()\n"
        "    local name, rank, icon, count, debuffType, duration, expTime, caster, steal, consolidate, spellId = UnitBuff(unit, index, filter)\n"
        "    if name then\n"
        "        self:SetText(name, 1, 1, 1)\n"
        "        if duration and duration > 0 then\n"
        "            self:AddLine(string.format('%.0f sec remaining', expTime - GetTime()), 1, 1, 1)\n"
        "        end\n"
        "        self.__spellId = spellId\n"
        "    end\n"
        "end\n"
        "function GameTooltip:SetUnitDebuff(unit, index, filter)\n"
        "    self:ClearLines()\n"
        "    local name, rank, icon, count, debuffType, duration, expTime, caster, steal, consolidate, spellId = UnitDebuff(unit, index, filter)\n"
        "    if name then\n"
        "        self:SetText(name, 1, 0, 0)\n"
        "        if debuffType then self:AddLine(debuffType, 0.5, 0.5, 0.5) end\n"
        "        self.__spellId = spellId\n"
        "    end\n"
        "end\n"
        "function GameTooltip:SetHyperlink(link)\n"
        "    self:ClearLines()\n"
        "    if not link then return end\n"
        "    local id = link:match('item:(%d+)')\n"
        "    if id then\n"
        "        _WoweePopulateItemTooltip(self, tonumber(id))\n"
        "        return\n"
        "    end\n"
        "    id = link:match('spell:(%d+)')\n"
        "    if id then\n"
        "        self:SetSpellByID(tonumber(id))\n"
        "        return\n"
        "    end\n"
        "end\n"
        // Shared item tooltip builder using GetItemInfo return values
        "function _WoweePopulateItemTooltip(self, itemId)\n"
        "    local name, itemLink, quality, iLevel, reqLevel, class, subclass, maxStack, equipSlot, texture, sellPrice = GetItemInfo(itemId)\n"
        "    if not name then return false end\n"
        "    local qColors = {[0]={0.62,0.62,0.62},[1]={1,1,1},[2]={0.12,1,0},[3]={0,0.44,0.87},[4]={0.64,0.21,0.93},[5]={1,0.5,0},[6]={0.9,0.8,0.5},[7]={0,0.8,1}}\n"
        "    local c = qColors[quality or 1] or {1,1,1}\n"
        "    self:SetText(name, c[1], c[2], c[3])\n"
        "    -- Item level for equipment\n"
        "    if equipSlot and equipSlot ~= '' and iLevel and iLevel > 0 then\n"
        "        self:AddLine('Item Level '..iLevel, 1, 0.82, 0)\n"
        "    end\n"
        "    -- Equip slot and subclass on same line\n"
        "    if equipSlot and equipSlot ~= '' then\n"
        "        local slotNames = {INVTYPE_HEAD='Head',INVTYPE_NECK='Neck',INVTYPE_SHOULDER='Shoulder',\n"
        "            INVTYPE_CHEST='Chest',INVTYPE_WAIST='Waist',INVTYPE_LEGS='Legs',INVTYPE_FEET='Feet',\n"
        "            INVTYPE_WRIST='Wrist',INVTYPE_HAND='Hands',INVTYPE_FINGER='Finger',\n"
        "            INVTYPE_TRINKET='Trinket',INVTYPE_CLOAK='Back',INVTYPE_WEAPON='One-Hand',\n"
        "            INVTYPE_SHIELD='Off Hand',INVTYPE_2HWEAPON='Two-Hand',INVTYPE_RANGED='Ranged',\n"
        "            INVTYPE_WEAPONMAINHAND='Main Hand',INVTYPE_WEAPONOFFHAND='Off Hand',\n"
        "            INVTYPE_HOLDABLE='Held In Off-Hand',INVTYPE_TABARD='Tabard',INVTYPE_ROBE='Chest'}\n"
        "        local slotText = slotNames[equipSlot] or ''\n"
        "        local subText = (subclass and subclass ~= '') and subclass or ''\n"
        "        if slotText ~= '' or subText ~= '' then\n"
        "            self:AddDoubleLine(slotText, subText, 1,1,1, 1,1,1)\n"
        "        end\n"
        "    elseif class and class ~= '' then\n"
        "        self:AddLine(class, 1, 1, 1)\n"
        "    end\n"
        "    -- Fetch detailed stats from C side\n"
        "    local data = _GetItemTooltipData(itemId)\n"
        "    if data then\n"
        "        -- Bind type\n"
        "        if data.isHeroic then self:AddLine('Heroic', 0, 1, 0) end\n"
        "        if data.isUnique then self:AddLine('Unique', 1, 1, 1)\n"
        "        elseif data.isUniqueEquipped then self:AddLine('Unique-Equipped', 1, 1, 1) end\n"
        "        if data.bindType == 1 then self:AddLine('Binds when picked up', 1, 1, 1)\n"
        "        elseif data.bindType == 2 then self:AddLine('Binds when equipped', 1, 1, 1)\n"
        "        elseif data.bindType == 3 then self:AddLine('Binds when used', 1, 1, 1) end\n"
        "        -- Armor\n"
        "        if data.armor and data.armor > 0 then\n"
        "            self:AddLine(data.armor..' Armor', 1, 1, 1)\n"
        "        end\n"
        "        -- Weapon damage and speed\n"
        "        if data.damageMin and data.damageMax and data.damageMin > 0 then\n"
        "            local speed = (data.speed or 0) / 1000\n"
        "            if speed > 0 then\n"
        "                self:AddDoubleLine(string.format('%.0f - %.0f Damage', data.damageMin, data.damageMax), string.format('Speed %.2f', speed), 1,1,1, 1,1,1)\n"
        "                local dps = (data.damageMin + data.damageMax) / 2 / speed\n"
        "                self:AddLine(string.format('(%.1f damage per second)', dps), 1, 1, 1)\n"
        "            end\n"
        "        end\n"
        "        -- Stats\n"
        "        if data.stamina then self:AddLine('+'..data.stamina..' Stamina', 0, 1, 0) end\n"
        "        if data.strength then self:AddLine('+'..data.strength..' Strength', 0, 1, 0) end\n"
        "        if data.agility then self:AddLine('+'..data.agility..' Agility', 0, 1, 0) end\n"
        "        if data.intellect then self:AddLine('+'..data.intellect..' Intellect', 0, 1, 0) end\n"
        "        if data.spirit then self:AddLine('+'..data.spirit..' Spirit', 0, 1, 0) end\n"
        "        -- Extra stats (hit, crit, haste, AP, SP, etc.)\n"
        "        if data.extraStats then\n"
        "            local statNames = {[3]='Agility',[4]='Strength',[5]='Intellect',[6]='Spirit',[7]='Stamina',\n"
        "                [12]='Defense Rating',[13]='Dodge Rating',[14]='Parry Rating',[15]='Block Rating',\n"
        "                [16]='Melee Hit Rating',[17]='Ranged Hit Rating',[18]='Spell Hit Rating',\n"
        "                [19]='Melee Crit Rating',[20]='Ranged Crit Rating',[21]='Spell Crit Rating',\n"
        "                [28]='Melee Haste Rating',[29]='Ranged Haste Rating',[30]='Spell Haste Rating',\n"
        "                [31]='Hit Rating',[32]='Crit Rating',[36]='Haste Rating',\n"
        "                [33]='Resilience Rating',[34]='Attack Power',[35]='Spell Power',\n"
        "                [37]='Expertise Rating',[38]='Attack Power',[39]='Ranged Attack Power',\n"
        "                [43]='Mana per 5 sec.',[44]='Armor Penetration Rating',\n"
        "                [45]='Spell Power',[46]='Health per 5 sec.',[47]='Spell Penetration'}\n"
        "            for _, stat in ipairs(data.extraStats) do\n"
        "                local name = statNames[stat.type]\n"
        "                if name and stat.value ~= 0 then\n"
        "                    local prefix = stat.value > 0 and '+' or ''\n"
        "                    self:AddLine(prefix..stat.value..' '..name, 0, 1, 0)\n"
        "                end\n"
        "            end\n"
        "        end\n"
        "        -- Resistances\n"
        "        if data.fireRes and data.fireRes ~= 0 then self:AddLine('+'..data.fireRes..' Fire Resistance', 0, 1, 0) end\n"
        "        if data.natureRes and data.natureRes ~= 0 then self:AddLine('+'..data.natureRes..' Nature Resistance', 0, 1, 0) end\n"
        "        if data.frostRes and data.frostRes ~= 0 then self:AddLine('+'..data.frostRes..' Frost Resistance', 0, 1, 0) end\n"
        "        if data.shadowRes and data.shadowRes ~= 0 then self:AddLine('+'..data.shadowRes..' Shadow Resistance', 0, 1, 0) end\n"
        "        if data.arcaneRes and data.arcaneRes ~= 0 then self:AddLine('+'..data.arcaneRes..' Arcane Resistance', 0, 1, 0) end\n"
        "        -- Item spell effects (Use: / Equip: / Chance on Hit:)\n"
        "        if data.itemSpells then\n"
        "            local triggerLabels = {[0]='Use: ',[1]='Equip: ',[2]='Chance on hit: ',[5]=''}\n"
        "            for _, sp in ipairs(data.itemSpells) do\n"
        "                local label = triggerLabels[sp.trigger] or ''\n"
        "                local text = sp.description or sp.name or ''\n"
        "                if text ~= '' then\n"
        "                    self:AddLine(label .. text, 0, 1, 0)\n"
        "                end\n"
        "            end\n"
        "        end\n"
        "        -- Gem sockets\n"
        "        if data.sockets then\n"
        "            local socketNames = {[1]='Meta',[2]='Red',[4]='Yellow',[8]='Blue'}\n"
        "            for _, sock in ipairs(data.sockets) do\n"
        "                local colorName = socketNames[sock.color] or 'Prismatic'\n"
        "                self:AddLine('[' .. colorName .. ' Socket]', 0.5, 0.5, 0.5)\n"
        "            end\n"
        "        end\n"
        "        -- Required level\n"
        "        if data.requiredLevel and data.requiredLevel > 1 then\n"
        "            self:AddLine('Requires Level '..data.requiredLevel, 1, 1, 1)\n"
        "        end\n"
        "        -- Flavor text\n"
        "        if data.description then self:AddLine('\"'..data.description..'\"', 1, 0.82, 0) end\n"
        "        if data.startsQuest then self:AddLine('This Item Begins a Quest', 1, 0.82, 0) end\n"
        "    end\n"
        "    -- Sell price from GetItemInfo\n"
        "    if sellPrice and sellPrice > 0 then\n"
        "        local gold = math.floor(sellPrice / 10000)\n"
        "        local silver = math.floor((sellPrice % 10000) / 100)\n"
        "        local copper = sellPrice % 100\n"
        "        local parts = {}\n"
        "        if gold > 0 then table.insert(parts, gold..'g') end\n"
        "        if silver > 0 then table.insert(parts, silver..'s') end\n"
        "        if copper > 0 then table.insert(parts, copper..'c') end\n"
        "        if #parts > 0 then self:AddLine('Sell Price: '..table.concat(parts, ' '), 1, 1, 1) end\n"
        "    end\n"
        "    self.__itemId = itemId\n"
        "    return true\n"
        "end\n"
        "function GameTooltip:SetInventoryItem(unit, slot)\n"
        "    self:ClearLines()\n"
        "    if unit ~= 'player' then return false, false, 0 end\n"
        "    local link = GetInventoryItemLink(unit, slot)\n"
        "    if not link then return false, false, 0 end\n"
        "    local id = link:match('item:(%d+)')\n"
        "    if not id then return false, false, 0 end\n"
        "    local ok = _WoweePopulateItemTooltip(self, tonumber(id))\n"
        "    return ok or false, false, 0\n"
        "end\n"
        "function GameTooltip:SetBagItem(bag, slot)\n"
        "    self:ClearLines()\n"
        "    local tex, count, locked, quality, readable, lootable, link = GetContainerItemInfo(bag, slot)\n"
        "    if not link then return end\n"
        "    local id = link:match('item:(%d+)')\n"
        "    if not id then return end\n"
        "    _WoweePopulateItemTooltip(self, tonumber(id))\n"
        "    if count and count > 1 then self:AddLine('Count: '..count, 0.5, 0.5, 0.5) end\n"
        "end\n"
        "function GameTooltip:SetSpellByID(spellId)\n"
        "    self:ClearLines()\n"
        "    if not spellId or spellId == 0 then return end\n"
        "    local name, rank, icon, castTime, minRange, maxRange = GetSpellInfo(spellId)\n"
        "    if name then\n"
        "        self:SetText(name, 1, 1, 1)\n"
        "        if rank and rank ~= '' then self:AddLine(rank, 0.5, 0.5, 0.5) end\n"
        "        -- Mana cost\n"
        "        local cost, costType = GetSpellPowerCost(spellId)\n"
        "        if cost and cost > 0 then\n"
        "            local powerNames = {[0]='Mana',[1]='Rage',[2]='Focus',[3]='Energy',[6]='Runic Power'}\n"
        "            self:AddLine(cost..' '..(powerNames[costType] or 'Mana'), 1, 1, 1)\n"
        "        end\n"
        "        -- Range\n"
        "        if maxRange and maxRange > 0 then\n"
        "            self:AddDoubleLine(string.format('%.0f yd range', maxRange), '', 1,1,1, 1,1,1)\n"
        "        end\n"
        "        -- Cast time\n"
        "        if castTime and castTime > 0 then\n"
        "            self:AddDoubleLine(string.format('%.1f sec cast', castTime / 1000), '', 1,1,1, 1,1,1)\n"
        "        else\n"
        "            self:AddDoubleLine('Instant', '', 1,1,1, 1,1,1)\n"
        "        end\n"
        "        -- Description\n"
        "        local desc = GetSpellDescription(spellId)\n"
        "        if desc and desc ~= '' then\n"
        "            self:AddLine(desc, 1, 0.82, 0)\n"
        "        end\n"
        "        -- Cooldown\n"
        "        local start, dur = GetSpellCooldown(spellId)\n"
        "        if dur and dur > 0 then\n"
        "            local rem = start + dur - GetTime()\n"
        "            if rem > 0.1 then self:AddLine(string.format('%.0f sec cooldown', rem), 1, 0, 0) end\n"
        "        end\n"
        "        self.__spellId = spellId\n"
        "    end\n"
        "end\n"
        "function GameTooltip:SetAction(slot)\n"
        "    self:ClearLines()\n"
        "    if not slot then return end\n"
        "    local actionType, id = GetActionInfo(slot)\n"
        "    if actionType == 'spell' and id and id > 0 then\n"
        "        self:SetSpellByID(id)\n"
        "    elseif actionType == 'item' and id and id > 0 then\n"
        "        _WoweePopulateItemTooltip(self, id)\n"
        "    end\n"
        "end\n"
        "function GameTooltip:FadeOut() end\n"
        "function GameTooltip:SetFrameStrata(...) end\n"
        "function GameTooltip:SetClampedToScreen(...) end\n"
        "function GameTooltip:IsOwned(f) return self.__owner == f end\n"
        // ShoppingTooltip: used by comparison tooltips
        "ShoppingTooltip1 = CreateFrame('Frame', 'ShoppingTooltip1')\n"
        "ShoppingTooltip2 = CreateFrame('Frame', 'ShoppingTooltip2')\n"
        // Error handling stubs (used by many addons)
        "local _errorHandler = function(err) return err end\n"
        "function geterrorhandler() return _errorHandler end\n"
        "function seterrorhandler(fn) if type(fn)=='function' then _errorHandler=fn end end\n"
        "function debugstack(start, count1, count2) return '' end\n"
        // A name is as valid as a function here, and FrameXML mostly passes a
        // name: UIDropDownMenu_Initialize does
        // securecall("UIDropDownMenu_InitializeHelper", frame), and the helper
        // is what sets UIDROPDOWNMENU_INIT_MENU and zeroes every list's
        // numButtons. Accepting only a function meant that call did nothing at
        // all, silently, and eight files died further on indexing what it
        // should have set.
        //
        // rawget, so a name this client does not have stays nil rather than
        // becoming the missing-API object, which is not callable as a function.
        "function securecall(fn, ...)\n"
        "    if type(fn) == 'string' then fn = rawget(_G, fn) end\n"
        "    if type(fn) == 'function' then return fn(...) end\n"
        "end\n"
        // Iterating a table the secure way, which for our purposes is next.
        "SecureNext = next\n"
        "function issecurevariable(...) return false end\n"
        "function issecure() return false end\n"
        // GetCVarBool wraps C-side GetCVar (registered in table) for boolean queries
        "function GetCVarBool(name) return GetCVar(name) == '1' end\n"
        // Misc compatibility stubs
        // GetScreenWidth, GetScreenHeight, GetNumLootItems are now C functions
        // GetFramerate is now a C function
        "function GetNetStats() return 0, 0, 0, 0 end\n"
        "function IsLoggedIn() return true end\n"
        "function StaticPopup_Show() end\n"
        "function StaticPopup_Hide() end\n"
        // UI Panel management — Show/Hide standard WoW panels
        "UIPanelWindows = {}\n"
        "function ShowUIPanel(frame, force)\n"
        "    if frame and frame.Show then frame:Show() end\n"
        "end\n"
        "function HideUIPanel(frame)\n"
        "    if frame and frame.Hide then frame:Hide() end\n"
        "end\n"
        "function ToggleFrame(frame)\n"
        "    if frame then\n"
        "        if frame:IsShown() then frame:Hide() else frame:Show() end\n"
        "    end\n"
        "end\n"
        "function GetUIPanel(which) return nil end\n"
        "function CloseWindows(ignoreCenter) return false end\n"
        // TEXT localization stub — returns input string unchanged
        "function TEXT(text) return text end\n"
        // Faux scroll frame helpers (used by many list UIs)
        "function FauxScrollFrame_GetOffset(frame)\n"
        "    return frame and frame.offset or 0\n"
        "end\n"
        "function FauxScrollFrame_Update(frame, numItems, numVisible, valueStep, button, smallWidth, bigWidth, highlightFrame, smallHighlightWidth, bigHighlightWidth)\n"
        "    if not frame then return false end\n"
        "    frame.offset = frame.offset or 0\n"
        "    local showScrollBar = numItems > numVisible\n"
        "    return showScrollBar\n"
        "end\n"
        "function FauxScrollFrame_SetOffset(frame, offset)\n"
        "    if frame then frame.offset = offset or 0 end\n"
        "end\n"
        "function FauxScrollFrame_OnVerticalScroll(frame, value, itemHeight, updateFunction)\n"
        "    if not frame then return end\n"
        "    frame.offset = math.floor(value / (itemHeight or 1) + 0.5)\n"
        "    if updateFunction then updateFunction() end\n"
        "end\n"
        // SecureCmdOptionParse — parses conditional macros like [target=focus]
        "function SecureCmdOptionParse(options)\n"
        "    if not options then return nil end\n"
        "    -- Simple: return the unconditional fallback (text after last semicolon or the whole string)\n"
        "    local result = options:match(';%s*(.-)$') or options:match('^%[.*%]%s*(.-)$') or options\n"
        "    return result\n"
        "end\n"
        // ChatFrame message group stubs
        "function ChatFrame_AddMessageGroup(frame, group) end\n"
        "function ChatFrame_RemoveMessageGroup(frame, group) end\n"
        "function ChatFrame_AddChannel(frame, channel) end\n"
        "function ChatFrame_RemoveChannel(frame, channel) end\n"
        // CreateTexture/CreateFontString are now C frame methods in the metatable
        "do\n"
        "  local function cc(r,g,b)\n"
        "    local t = {r=r, g=g, b=b}\n"
        "    t.colorStr = string.format('%02x%02x%02x', math.floor(r*255), math.floor(g*255), math.floor(b*255))\n"
        "    function t:GenerateHexColor() return '|cff' .. self.colorStr end\n"
        "    function t:GenerateHexColorMarkup() return '|cff' .. self.colorStr end\n"
        "    return t\n"
        "  end\n"
        "  RAID_CLASS_COLORS = {\n"
        "    WARRIOR=cc(0.78,0.61,0.43), PALADIN=cc(0.96,0.55,0.73),\n"
        "    HUNTER=cc(0.67,0.83,0.45), ROGUE=cc(1.0,0.96,0.41),\n"
        "    PRIEST=cc(1.0,1.0,1.0), DEATHKNIGHT=cc(0.77,0.12,0.23),\n"
        "    SHAMAN=cc(0.0,0.44,0.87), MAGE=cc(0.41,0.80,0.94),\n"
        "    WARLOCK=cc(0.58,0.51,0.79), DRUID=cc(1.0,0.49,0.04),\n"
        "  }\n"
        "end\n"
        // GetClassColor(className) — returns r, g, b, colorString
        "function GetClassColor(className)\n"
        "    local c = RAID_CLASS_COLORS[className]\n"
        "    if c then return c.r, c.g, c.b, c.colorStr end\n"
        "    return 1, 1, 1, 'ffffffff'\n"
        "end\n"
        // QuestDifficultyColors table for quest level coloring
        "QuestDifficultyColors = {\n"
        "    impossible = {r=1.0,g=0.1,b=0.1,font='QuestDifficulty_Impossible'},\n"
        "    verydifficult = {r=1.0,g=0.5,b=0.25,font='QuestDifficulty_VeryDifficult'},\n"
        "    difficult = {r=1.0,g=1.0,b=0.0,font='QuestDifficulty_Difficult'},\n"
        "    standard = {r=0.25,g=0.75,b=0.25,font='QuestDifficulty_Standard'},\n"
        "    trivial = {r=0.5,g=0.5,b=0.5,font='QuestDifficulty_Trivial'},\n"
        "    header = {r=1.0,g=0.82,b=0.0,font='QuestDifficulty_Header'},\n"
        "}\n"
        // Money formatting utility
        "function GetCoinTextureString(copper)\n"
        "    if not copper or copper == 0 then return '0c' end\n"
        "    copper = math.floor(copper)\n"
        "    local g = math.floor(copper / 10000)\n"
        "    local s = math.floor(math.fmod(copper, 10000) / 100)\n"
        "    local c = math.fmod(copper, 100)\n"
        "    local r = ''\n"
        "    if g > 0 then r = r .. g .. 'g ' end\n"
        "    if s > 0 then r = r .. s .. 's ' end\n"
        "    if c > 0 or r == '' then r = r .. c .. 'c' end\n"
        "    return r\n"
        "end\n"
        "GetCoinText = GetCoinTextureString\n"
    );

    // UIDropDownMenu framework — minimal compat for addons using dropdown menus
    bootstrap(
        "UIDROPDOWNMENU_MENU_LEVEL = 1\n"
        "UIDROPDOWNMENU_MENU_VALUE = nil\n"
        "UIDROPDOWNMENU_OPEN_MENU = nil\n"
        "local _ddMenuList = {}\n"
        "function UIDropDownMenu_Initialize(frame, initFunc, displayMode, level, menuList)\n"
        "    if frame then frame.__initFunc = initFunc end\n"
        "end\n"
        "function UIDropDownMenu_CreateInfo() return {} end\n"
        "function UIDropDownMenu_AddButton(info, level) table.insert(_ddMenuList, info) end\n"
        "function UIDropDownMenu_SetWidth(frame, width) end\n"
        "function UIDropDownMenu_SetButtonWidth(frame, width) end\n"
        "function UIDropDownMenu_SetText(frame, text)\n"
        "    if frame then frame.__text = text end\n"
        "end\n"
        "function UIDropDownMenu_GetText(frame)\n"
        "    return frame and frame.__text or ''\n"
        "end\n"
        "function UIDropDownMenu_SetSelectedID(frame, id) end\n"
        "function UIDropDownMenu_SetSelectedValue(frame, value) end\n"
        "function UIDropDownMenu_GetSelectedID(frame) return 1 end\n"
        "function UIDropDownMenu_GetSelectedValue(frame) return nil end\n"
        "function UIDropDownMenu_JustifyText(frame, justify) end\n"
        "function UIDropDownMenu_EnableDropDown(frame) end\n"
        "function UIDropDownMenu_DisableDropDown(frame) end\n"
        "function CloseDropDownMenus() end\n"
        "function ToggleDropDownMenu(level, value, frame, anchor, xOfs, yOfs) end\n"
    );

    // UISpecialFrames: frames in this list close on Escape key
    bootstrap(
        "UISpecialFrames = {}\n"
        // Shared font objects, carrying the height and colour a FontString takes
        // from them. They were empty tables, so inheriting one changed nothing
        // and every label came out the same size in the same colour — and
        // FrameXML inherits one more than three thousand times.
        //
        // The colours are Blizzard's: normal is the familiar gold, highlight is
        // white, disabled grey, and the quest fonts near-black on parchment.
        "local function font(h, r, g, b) return { height = h, r = r, g = g, b = b, a = 1 } end\n"
        "GameFontNormal            = font(12, 1.00, 0.82, 0.00)\n"
        "GameFontNormalSmall       = font(10, 1.00, 0.82, 0.00)\n"
        "GameFontNormalLarge       = font(16, 1.00, 0.82, 0.00)\n"
        "GameFontNormalHuge        = font(20, 1.00, 0.82, 0.00)\n"
        "GameFontHighlight         = font(12, 1.00, 1.00, 1.00)\n"
        "GameFontHighlightSmall    = font(10, 1.00, 1.00, 1.00)\n"
        "GameFontHighlightLarge    = font(16, 1.00, 1.00, 1.00)\n"
        "GameFontDisable           = font(12, 0.50, 0.50, 0.50)\n"
        "GameFontDisableSmall      = font(10, 0.50, 0.50, 0.50)\n"
        "GameFontDisableLarge      = font(16, 0.50, 0.50, 0.50)\n"
        "GameFontWhite             = font(12, 1.00, 1.00, 1.00)\n"
        "GameFontRed               = font(12, 1.00, 0.13, 0.13)\n"
        "GameFontGreen             = font(12, 0.13, 1.00, 0.13)\n"
        "NumberFontNormal          = font(12, 1.00, 1.00, 1.00)\n"
        "NumberFontNormalSmall     = font(10, 1.00, 1.00, 1.00)\n"
        "NumberFontNormalLarge     = font(16, 1.00, 1.00, 1.00)\n"
        "ChatFontNormal            = font(12, 1.00, 1.00, 1.00)\n"
        "SystemFont                = font(12, 1.00, 0.82, 0.00)\n"
        "SystemFontSmall           = font(10, 1.00, 0.82, 0.00)\n"
        "QuestFont                 = font(13, 0.18, 0.12, 0.06)\n"
        "QuestFontNormalSmall      = font(11, 0.18, 0.12, 0.06)\n"
        "QuestTitleFont            = font(15, 0.00, 0.00, 0.00)\n"
        "Tooltip_Med               = font(12, 1.00, 1.00, 1.00)\n"
        "Tooltip_Small             = font(10, 1.00, 1.00, 1.00)\n"
        // InterfaceOptionsFrame: addons register settings panels here
        "InterfaceOptionsFrame = CreateFrame('Frame', 'InterfaceOptionsFrame')\n"
        "InterfaceOptionsFramePanelContainer = CreateFrame('Frame', 'InterfaceOptionsFramePanelContainer')\n"
        "function InterfaceOptions_AddCategory(panel) end\n"
        "function InterfaceOptionsFrame_OpenToCategory(panel) end\n"
        // Commonly expected global tables
        "SLASH_RELOAD1 = '/reload'\n"
        "SLASH_RELOADUI1 = '/reloadui'\n"
        "GRAY_FONT_COLOR = {r=0.5,g=0.5,b=0.5}\n"
        "NORMAL_FONT_COLOR = {r=1.0,g=0.82,b=0.0}\n"
        "HIGHLIGHT_FONT_COLOR = {r=1.0,g=1.0,b=1.0}\n"
        "GREEN_FONT_COLOR = {r=0.1,g=1.0,b=0.1}\n"
        "RED_FONT_COLOR = {r=1.0,g=0.1,b=0.1}\n"
        // C_ChatInfo — addon message prefix API used by some addons
        "C_ChatInfo = C_ChatInfo or {}\n"
        "C_ChatInfo.RegisterAddonMessagePrefix = RegisterAddonMessagePrefix\n"
        "C_ChatInfo.IsAddonMessagePrefixRegistered = IsAddonMessagePrefixRegistered\n"
        "C_ChatInfo.SendAddonMessage = SendAddonMessage\n"
    );

    // Action bar constants and functions used by action bar addons
    bootstrap(
        "NUM_ACTIONBAR_BUTTONS = 12\n"
        "NUM_ACTIONBAR_PAGES = 6\n"
        "ACTION_BUTTON_SHOW_GRID_REASON_CVAR = 1\n"
        "ACTION_BUTTON_SHOW_GRID_REASON_EVENT = 2\n"
        // Action bar page tracking
        "local _actionBarPage = 1\n"
        "function GetActionBarPage() return _actionBarPage end\n"
        "function ChangeActionBarPage(page) _actionBarPage = page end\n"
        "function GetBonusBarOffset() return 0 end\n"
        // Action type query
        "function GetActionText(slot) return nil end\n"
        "function GetActionCount(slot) return 0 end\n"
        // Binding functions
        "function GetBindingKey(action) return nil end\n"
        "function GetBindingAction(key) return nil end\n"
        "function SetBinding(key, action) end\n"
        "function SaveBindings(which) end\n"
        "function GetCurrentBindingSet() return 1 end\n"
        // Macro functions
        "function GetNumMacros() return 0, 0 end\n"
        "function GetMacroInfo(id) return nil end\n"
        "function GetMacroBody(id) return nil end\n"
        "function GetMacroIndexByName(name) return 0 end\n"
        // Stance bar
        "function GetNumShapeshiftForms() return 0 end\n"
        "function GetShapeshiftFormInfo(index) return nil, nil, nil, nil end\n"
        // Pet action bar
        "NUM_PET_ACTION_SLOTS = 10\n"
        // Common WoW constants used by many addons
        "MAX_TALENT_TABS = 3\n"
        // Values as 3.3.5 has them. These are pre-set so an addon has them
        // before anything else runs, and on this branch the original interface
        // is not loaded at all — so these are the only values there are, and a
        // wrong one is wrong for good.
        //
        // Both book types were numbers where the game uses strings, which is
        // the quietest kind of wrong: every comparison against them is false
        // rather than an error, so an addon asking "is this the pet book"
        // always heard no and nothing said why.
        "MAX_NUM_TALENTS = 40\n"
        "BOOKTYPE_SPELL = 'spell'\n"
        "BOOKTYPE_PET = 'pet'\n"
        "MAX_PARTY_MEMBERS = 4\n"
        "MAX_RAID_MEMBERS = 40\n"
        "MAX_ARENA_TEAMS = 3\n"
        "INVSLOT_FIRST_EQUIPPED = 1\n"
        "INVSLOT_LAST_EQUIPPED = 19\n"
        "NUM_BAG_SLOTS = 4\n"
        "NUM_BANKBAGSLOTS = 7\n"
        // What a bag index is offset by to name its container, which is what
        // PutItemInBag adds. Zero made that arithmetic name the backpack.
        "CONTAINER_BAG_OFFSET = 19\n"
        "MAX_SKILLLINE_TABS = 8\n"
        "TRADE_ENCHANT_SLOT = 7\n"
        "function GetPetActionInfo(slot) return nil end\n"
        "function GetPetActionsUsable() return false end\n"
    );

    // WoW table/string utility functions used by many addons
    bootstrap(
        // Table utilities
        "function tContains(tbl, item)\n"
        "    for _, v in pairs(tbl) do if v == item then return true end end\n"
        "    return false\n"
        "end\n"
        "function tInvert(tbl)\n"
        "    local inv = {}\n"
        "    for k, v in pairs(tbl) do inv[v] = k end\n"
        "    return inv\n"
        "end\n"
        "function CopyTable(src)\n"
        "    if type(src) ~= 'table' then return src end\n"
        "    local copy = {}\n"
        "    for k, v in pairs(src) do copy[k] = CopyTable(v) end\n"
        "    return setmetatable(copy, getmetatable(src))\n"
        "end\n"
        "function tDeleteItem(tbl, item)\n"
        "    for i = #tbl, 1, -1 do if tbl[i] == item then table.remove(tbl, i) end end\n"
        "end\n"
        // Mixin pattern — used by modern addons for OOP-style object creation
        "function Mixin(obj, ...)\n"
        "    for i = 1, select('#', ...) do\n"
        "        local mixin = select(i, ...)\n"
        "        for k, v in pairs(mixin) do obj[k] = v end\n"
        "    end\n"
        "    return obj\n"
        "end\n"
        "function CreateFromMixins(...)\n"
        "    return Mixin({}, ...)\n"
        "end\n"
        "function CreateAndInitFromMixin(mixin, ...)\n"
        "    local obj = CreateFromMixins(mixin)\n"
        "    if obj.Init then obj:Init(...) end\n"
        "    return obj\n"
        "end\n"
        "function MergeTable(dest, src)\n"
        "    for k, v in pairs(src) do dest[k] = v end\n"
        "    return dest\n"
        "end\n"
        // String utilities (WoW globals that alias Lua string functions)
        "strupper = string.upper\n"
        "strlower = string.lower\n"
        "strfind = string.find\n"
        "strsub = string.sub\n"
        "strlen = string.len\n"
        "strrep = string.rep\n"
        "strbyte = string.byte\n"
        "strchar = string.char\n"
        "strgfind = string.gmatch\n"
        "function tostringall(...)\n"
        "    local n = select('#', ...)\n"
        "    if n == 0 then return end\n"
        "    local r = {}\n"
        "    for i = 1, n do r[i] = tostring(select(i, ...)) end\n"
        "    return unpack(r, 1, n)\n"
        "end\n"
        "strrev = string.reverse\n"
        "gsub = string.gsub\n"
        "gmatch = string.gmatch\n"
        "strjoin = function(delim, ...)\n"
        "    return table.concat({...}, delim)\n"
        "end\n"
        // Math utilities
        "function Clamp(val, lo, hi) return math.min(math.max(val, lo), hi) end\n"
        "function Round(val) return math.floor(val + 0.5) end\n"
        // Bit operations (WoW provides these; Lua 5.1 doesn't have native bit ops)
        "bit = bit or {}\n"
        "bit.band = bit.band or function(a, b) local r,m=0,1 for i=0,31 do if a%2==1 and b%2==1 then r=r+m end a=math.floor(a/2) b=math.floor(b/2) m=m*2 end return r end\n"
        "bit.bor = bit.bor or function(a, b) local r,m=0,1 for i=0,31 do if a%2==1 or b%2==1 then r=r+m end a=math.floor(a/2) b=math.floor(b/2) m=m*2 end return r end\n"
        "bit.bxor = bit.bxor or function(a, b) local r,m=0,1 for i=0,31 do if (a%2==1)~=(b%2==1) then r=r+m end a=math.floor(a/2) b=math.floor(b/2) m=m*2 end return r end\n"
        "bit.bnot = bit.bnot or function(a) return 4294967295 - a end\n"
        "bit.lshift = bit.lshift or function(a, n) return a * (2^n) end\n"
        "bit.rshift = bit.rshift or function(a, n) return math.floor(a / (2^n)) end\n"
    );
}

// ---- Event System ----
// Lua-side: WoweeEvents table holds { ["EVENT_NAME"] = { handler1, handler2, ... } }
// RegisterEvent("EVENT", handler) adds a handler function
// UnregisterEvent("EVENT", handler) removes it


static int lua_RegisterEvent(lua_State* L) {
    const char* eventName = luaL_checkstring(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);

    // Get or create the WoweeEvents table
    lua_getglobal(L, "__WoweeEvents");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setglobal(L, "__WoweeEvents");
    }

    // Get or create the handler list for this event
    lua_getfield(L, -1, eventName);
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setfield(L, -3, eventName);
    }

    // Append the handler function to the list
    int len = static_cast<int>(lua_objlen(L, -1));
    lua_pushvalue(L, 2);  // push the handler function
    lua_rawseti(L, -2, len + 1);

    lua_pop(L, 2);  // pop handler list + WoweeEvents
    return 0;
}

static int lua_UnregisterEvent(lua_State* L) {
    const char* eventName = luaL_checkstring(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);

    lua_getglobal(L, "__WoweeEvents");
    if (lua_isnil(L, -1)) { lua_pop(L, 1); return 0; }

    lua_getfield(L, -1, eventName);
    if (lua_isnil(L, -1)) { lua_pop(L, 2); return 0; }

    // Remove matching handler from the list
    int len = static_cast<int>(lua_objlen(L, -1));
    for (int i = 1; i <= len; i++) {
        lua_rawgeti(L, -1, i);
        if (lua_rawequal(L, -1, 2)) {
            lua_pop(L, 1);
            // Shift remaining elements down
            for (int j = i; j < len; j++) {
                lua_rawgeti(L, -1, j + 1);
                lua_rawseti(L, -2, j);
            }
            lua_pushnil(L);
            lua_rawseti(L, -2, len);
            break;
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 2);
    return 0;
}

void LuaEngine::registerEventAPI() {
    lua_pushcfunction(L_, lua_RegisterEvent);
    lua_setglobal(L_, "RegisterEvent");

    lua_pushcfunction(L_, lua_UnregisterEvent);
    lua_setglobal(L_, "UnregisterEvent");

    // Create the events table
    lua_newtable(L_);
    lua_setglobal(L_, "__WoweeEvents");
}

void LuaEngine::fireEvent(const std::string& eventName,
                           const std::vector<std::string>& args) {
    if (!L_) return;

    // An event handler may cause another event, which is ordinary and has to
    // keep working — but a cycle between two of them recurses through both this
    // stack and Lua's, inside one frame, until the process dies. Reporting a
    // script error used to be such a cycle: the report fired an event, the
    // handler for it errored, and the error was reported the same way.
    //
    // Deep enough that no legitimate chain reaches it, and it says which event
    // it stopped, because the name is the only clue to which cycle it was.
    constexpr int kMaxEventDepth = 8;
    struct DepthGuard {
        int& d;
        explicit DepthGuard(int& v) : d(v) { ++d; }
        ~DepthGuard() { --d; }
    } depthGuard{eventDepth_};
    if (eventDepth_ > kMaxEventDepth) {
        LOG_WARNING("Event '", eventName, "' is ", eventDepth_,
                    " deep and was dropped — handlers are triggering each other");
        return;
    }

    lua_getglobal(L_, "__WoweeEvents");
    if (lua_isnil(L_, -1)) { lua_pop(L_, 1); return; }

    lua_getfield(L_, -1, eventName.c_str());
    if (lua_isnil(L_, -1)) { lua_pop(L_, 2); return; }

    int handlerCount = static_cast<int>(lua_objlen(L_, -1));
    for (int i = 1; i <= handlerCount; i++) {
        lua_rawgeti(L_, -1, i);
        if (!lua_isfunction(L_, -1)) { lua_pop(L_, 1); continue; }

        // Push arguments: event name first, then extra args
        lua_pushstring(L_, eventName.c_str());
        for (const auto& arg : args) {
            lua_pushstring(L_, arg.c_str());
        }

        int nargs = 1 + static_cast<int>(args.size());
        if (lua_pcall(L_, nargs, 0, 0) != 0) {
            const char* err = lua_tostring(L_, -1);
            std::string errStr = err ? err : "(unknown)";
            LOG_ERROR("LuaEngine: event '", eventName, "' handler error: ", errStr);
            if (luaErrorCallback_) luaErrorCallback_(errStr);
            lua_pop(L_, 1);
        }
    }
    lua_pop(L_, 2);  // pop handler list + WoweeEvents

    // Also dispatch to frames that registered for this event via frame:RegisterEvent()
    lua_getglobal(L_, "__WoweeFrameEvents");
    if (lua_istable(L_, -1)) {
        lua_getfield(L_, -1, eventName.c_str());
        if (lua_istable(L_, -1)) {
            int frameCount = static_cast<int>(lua_objlen(L_, -1));
            for (int i = 1; i <= frameCount; i++) {
                lua_rawgeti(L_, -1, i);
                if (!lua_istable(L_, -1)) { lua_pop(L_, 1); continue; }

                // Get the frame's OnEvent script
                lua_getfield(L_, -1, "__scripts");
                if (lua_istable(L_, -1)) {
                    lua_getfield(L_, -1, "OnEvent");
                    if (lua_isfunction(L_, -1)) {
                        lua_pushvalue(L_, -3);  // self (frame)
                        lua_pushstring(L_, eventName.c_str());
                        for (const auto& arg : args) lua_pushstring(L_, arg.c_str());
                        int nargs = 2 + static_cast<int>(args.size());
                        if (lua_pcall(L_, nargs, 0, 0) != 0) {
                            const char* ferr = lua_tostring(L_, -1);
                            std::string ferrStr = ferr ? ferr : "(unknown)";
                            LOG_ERROR("LuaEngine: frame OnEvent error: ", ferrStr);
                            if (luaErrorCallback_) luaErrorCallback_(ferrStr);
                            lua_pop(L_, 1);
                        }
                    } else {
                        lua_pop(L_, 1); // pop non-function
                    }
                }
                lua_pop(L_, 2); // pop __scripts + frame
            }
        }
        lua_pop(L_, 1); // pop event frame list
    }
    lua_pop(L_, 1); // pop __WoweeFrameEvents
}

namespace {
/// Defined with the other pcall helpers further down; declared here because
/// callFrameScript needs it and comes first.
int luaTracebackHandler(lua_State* L);
}  // namespace

void LuaEngine::callFrameScript(uint32_t wid, const char* script,
                                const char* arg) {
    if (!L_ || wid == 0) return;
    lua_getglobal(L_, "__WoweeFramesByWid");
    if (!lua_istable(L_, -1)) { lua_pop(L_, 1); return; }
    lua_pushinteger(L_, static_cast<lua_Integer>(wid));
    lua_rawget(L_, -2);
    if (!lua_istable(L_, -1)) { lua_pop(L_, 2); return; }

    lua_getfield(L_, -1, "__scripts");
    if (!lua_istable(L_, -1)) { lua_pop(L_, 3); return; }
    // The traceback handler has to sit below the function it is handling for,
    // so it goes on before the script is fetched. A handler that fails now says
    // where it was called from, the same as one that fails during the load.
    lua_pushcfunction(L_, luaTracebackHandler);
    const int handlerIdx = lua_gettop(L_);
    lua_getfield(L_, handlerIdx - 1, script);
    if (!lua_isfunction(L_, -1)) { lua_pop(L_, 5); return; }

    lua_pushvalue(L_, handlerIdx - 2);  // self
    int nargs = 1;
    if (arg) { lua_pushstring(L_, arg); ++nargs; }
    if (lua_pcall(L_, nargs, 0, handlerIdx) != 0) {
        const char* err = lua_tostring(L_, -1);
        LOG_ERROR("LuaEngine: ", script, " error: ", err ? err : "?");
        if (luaErrorCallback_) luaErrorCallback_(err ? err : "script error");
        lua_pop(L_, 1);
    }
    // Four, not three: the traceback handler is still below.
    lua_pop(L_, 4);
}


void LuaEngine::installMissingApiFallback() {
    // Off unless asked for. With it on, every unknown global answers, so code
    // that checks whether a function exists before using it — which addons do
    // constantly — sees everything as present and takes branches meant for a
    // newer client. That is the right trade for bringing FrameXML up, where the
    // point is to get past a missing name and find out what actually matters,
    // and the wrong one for everyday addon loading.
    auto isSet = [](const char* name) {
        const char* v = std::getenv(name);
        return v && *v && std::string(v) != "0";
    };
    // Loading FrameXML implies it. FrameXML cannot get through its own load
    // without the fallback, so two separate switches where one is useless
    // without the other is only a way to be handed a wall of failures for
    // setting the obvious one.
    //
    // Said explicitly, though, the setting wins either way. The fallback is not
    // free — it makes every feature check read as present — and now that the
    // real gaps are closing it is worth being able to ask what it is still
    // buying, which needs a way to turn it off with FrameXML on.
    const char* explicitSetting = std::getenv("WOWEE_LUA_API_FALLBACK");
    const bool enabled = (explicitSetting && *explicitSetting)
                             ? std::string(explicitSetting) != "0"
                             : isSet("WOWEE_LOAD_FRAMEXML");
    if (!enabled) return;

    lua_pushcfunction(L_, lua_RecordMissingApi);
    lua_setglobal(L_, "__WoweeRecordMissingApi");

    // A name in SCREAMING_SNAKE_CASE is a constant, and handing back a function
    // where a number or a string was wanted turns a missing value into a
    // confusing type error further away. Those stay nil. UpperCamelCase is a
    // function, and gets one that does nothing.
    bootstrap(
        // Callable, and every field of it is a method answering nil.
        //
        // A bare function was not enough. FrameXML looks frames up by name as
        // often as it calls functions — local t = _G[name.."PrefixText"] — and
        // it guards them properly, with if (t) then t:GetText(). A function
        // passes that guard and then dies on the indexing, so the correct check
        // was worse than no check at all: eleven files went down on that one
        // line. Answering nil from every method lets the guarded branch run and
        // come to nothing, which is what a missing frame should look like.
        // Methods answer; data fields do not.
        //
        // Answering everything made feature checks on a missing frame's own
        // state read as present: FCFMin_UpdateColors tests
        // minFrame.selectedColorTable and takes the branch that dereferences
        // it. The same convention the fallback already uses for names —
        // PascalCase is a method, anything else is data — applies inside the
        // object too, so a field is nil and the guard around it works.
        "local missing = setmetatable({}, {\n"
        "  __call = function() end,\n"
        "  __index = function(_, k)\n"
        "    if type(k) == 'string' and string.find(k, '^%u') then\n"
        "      return function() return nil end\n"
        "    end\n"
        "    return nil\n"
        "  end,\n"
        "})\n"
        "local seen = {}\n"
        "setmetatable(_G, { __index = function(_, k)\n"
        "  if type(k) ~= 'string' then return nil end\n"
        "  if not string.find(k, '^%u') then return nil end\n"
        "  if string.find(k, '^[A-Z][A-Z0-9_]*$') then return nil end\n"
        // A digit in the name means an instance, not an API function, and an
        // instance that does not exist must read as absent. FrameXML looks
        // frames up by building the name — _G["ChatFrame"..id.."Minimized"] —
        // and then guards the result properly with if (frame). Answering makes
        // that guard pass and the branch behind it runs against nothing.
        //
        // Measured rather than assumed: of the 4,100 distinct names FrameXML
        // calls as functions, four contain a digit, and three of those it
        // defines itself. Being wrong here costs a no-op for one API name,
        // which is where this started.
        "  if string.find(k, '%d') then return nil end\n"
        "  if not seen[k] then seen[k] = true; __WoweeRecordMissingApi(k) end\n"
        "  return missing\n"
        "end })\n");

    LOG_WARNING("LuaEngine: missing-API fallback is ON — unknown globals answer "
                "with a no-op, so feature detection will read as present");
}

void LuaEngine::reportMissingApi() const {
    const auto& names = missingApiNames();
    if (names.empty()) return;
    // At warning level, because release builds drop INFO and this is the whole
    // point of recording them: the list is the measured gap, once per session,
    // and it was being written where nobody could read it.
    LOG_WARNING("LuaEngine: ", names.size(), " distinct API names were called "
                "and not found this session");
    std::string line;
    for (const auto& n : names) {
        line += n;
        line += ' ';
        if (line.size() > 900) { LOG_WARNING("  missing: ", line); line.clear(); }
    }
    if (!line.empty()) LOG_WARNING("  missing: ", line);
}

/// Whether a frame asked for this button's clicks.
///
/// WoW gives a button LeftButtonUp and nothing else unless it says otherwise,
/// and FrameXML says otherwise exactly where a context menu is wanted. Without
/// the check every frame would answer a right-click, which is a menu opening
/// under a cursor that never asked for one.
bool LuaEngine::frameAcceptsClick(uint32_t wid, const char* button) {
    lua_getglobal(L_, "__WoweeFramesByWid");
    if (!lua_istable(L_, -1)) { lua_pop(L_, 1); return false; }
    lua_pushinteger(L_, static_cast<lua_Integer>(wid));
    lua_rawget(L_, -2);
    if (!lua_istable(L_, -1)) { lua_pop(L_, 2); return false; }

    lua_getfield(L_, -1, "__clicks");
    bool accepts;
    if (lua_istable(L_, -1)) {
        // Registered explicitly: either edge counts, since this only models
        // the release.
        const std::string up = std::string(button) + "Up";
        const std::string down = std::string(button) + "Down";
        lua_getfield(L_, -1, up.c_str());
        accepts = lua_toboolean(L_, -1) != 0;
        lua_pop(L_, 1);
        if (!accepts) {
            lua_getfield(L_, -1, down.c_str());
            accepts = lua_toboolean(L_, -1) != 0;
            lua_pop(L_, 1);
        }
    } else {
        accepts = (std::strcmp(button, "LeftButton") == 0);
    }
    lua_pop(L_, 3);
    return accepts;
}

namespace {
LuaEngine* engineFrom(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* e = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return e;
}
}  // namespace

int lua_EditBox_SetFocus(lua_State* L) {
    if (auto* e = engineFrom(L)) e->setEditFocus(widgetIdOf(L, 1));
    return 0;
}
int lua_EditBox_ClearFocus(lua_State* L) {
    if (auto* e = engineFrom(L)) e->setEditFocus(0);
    return 0;
}
int lua_EditBox_HasFocus(lua_State* L) {
    const auto* w = widgetOf(L, 1);
    lua_pushboolean(L, w && w->editFocused ? 1 : 0);
    return 1;
}

void LuaEngine::setEditFocus(uint32_t wid) {
    if (focusedWid_ == wid) return;
    if (focusedWid_ != 0) {
        if (auto* old = widgets_.get(focusedWid_)) old->editFocused = false;
        callFrameScript(focusedWid_, "OnEditFocusLost");
    }
    focusedWid_ = wid;
    if (focusedWid_ != 0) {
        if (auto* w = widgets_.get(focusedWid_)) w->editFocused = true;
        callFrameScript(focusedWid_, "OnEditFocusGained");
    }
}

void LuaEngine::dispatchText(const char* utf8) {
    if (!L_ || focusedWid_ == 0 || !utf8) return;
    auto* w = widgets_.get(focusedWid_);
    if (!w || !w->isEditBox) return;

    std::string add(utf8);
    if (add.empty()) return;
    // A numeric box takes digits and nothing else, which is what stops a
    // quantity field filling with letters.
    if (w->editNumeric) {
        add.erase(std::remove_if(add.begin(), add.end(),
                                 [](unsigned char c) { return std::isdigit(c) == 0; }),
                  add.end());
        if (add.empty()) return;
    }
    if (w->editMaxLetters > 0 &&
        static_cast<int>(w->editText.size() + add.size()) > w->editMaxLetters) {
        const int room = w->editMaxLetters - static_cast<int>(w->editText.size());
        if (room <= 0) return;
        add.resize(static_cast<size_t>(room));
    }

    const size_t at = std::min(w->cursorPos, w->editText.size());
    w->editText.insert(at, add);
    w->cursorPos = at + add.size();
    // The handler that tells a search field to filter, and a chat box to look
    // for a channel prefix.
    callFrameScript(focusedWid_, "OnTextChanged");
}

void LuaEngine::dispatchKey(int sdlKeycode, bool ctrlHeld) {
    if (!L_ || focusedWid_ == 0) return;
    auto* w = widgets_.get(focusedWid_);
    if (!w || !w->isEditBox) return;
    (void)ctrlHeld;

    // Keycodes are SDL's, which is what the window reports; the caller does not
    // translate them so this stays the only place that knows.
    constexpr int kBackspace = '\b';
    constexpr int kReturn    = '\r';
    constexpr int kEscape    = 27;
    constexpr int kDelete    = 0x4000004C;  // SDLK_DELETE
    constexpr int kLeft      = 0x40000050;
    constexpr int kRight     = 0x4000004F;
    constexpr int kHome      = 0x4000004A;
    constexpr int kEnd       = 0x4000004D;

    const size_t len = w->editText.size();
    switch (sdlKeycode) {
        case kBackspace:
            if (w->cursorPos > 0 && len > 0) {
                w->editText.erase(w->cursorPos - 1, 1);
                --w->cursorPos;
                callFrameScript(focusedWid_, "OnTextChanged");
            }
            break;
        case kDelete:
            if (w->cursorPos < len) {
                w->editText.erase(w->cursorPos, 1);
                callFrameScript(focusedWid_, "OnTextChanged");
            }
            break;
        case kLeft:  if (w->cursorPos > 0) --w->cursorPos; break;
        case kRight: if (w->cursorPos < len) ++w->cursorPos; break;
        case kHome:  w->cursorPos = 0; break;
        case kEnd:   w->cursorPos = len; break;
        case kReturn:
            // The handler decides what to do with it, including whether to let
            // go of focus — a chat box does, a search field does not.
            callFrameScript(focusedWid_, "OnEnterPressed");
            break;
        case kEscape:
            callFrameScript(focusedWid_, "OnEscapePressed");
            setEditFocus(0);
            break;
        default: break;
    }
}

void LuaEngine::dispatchMouse(float x, float y, MouseButtons buttons) {
    if (!L_) return;
    // The cursor arrives in pixels and the tree is in interface units, so this
    // is where the two meet. Hit testing against unconverted pixels would miss
    // every frame by the scale factor.
    const float s = widgets_.uiScale();
    if (s > 0.0f) { x /= s; y /= s; }
    const uint32_t hit = widgets_.hitTest(x, y);

    // Throttled, and only while there is something to hit. Whether the mouse
    // reaches the widget tree at all is otherwise invisible: a frame that never
    // lights up looks the same whether the dispatch is not running, the
    // coordinates are wrong, or the frame is not taking the mouse.
    static double lastReport = 0.0;
    const double now = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()) / 1000.0;
    if (now - lastReport >= 1.0) {
        size_t mouseFrames = 0;
        for (uint32_t id = 1; id < widgets_.size(); ++id) {
            const auto* w = widgets_.get(id);
            if (w && w->mouseEnabled && w->visible) ++mouseFrames;
        }
        // Reported whenever anything is on screen at all, not only when
        // something is mouse-enabled. Gating on that hid the one case that was
        // actually happening: no frame took the mouse, so the count was zero,
        // so nothing was logged, so the silence looked like the dispatch never
        // running. A diagnostic must not go quiet in the state it exists to
        // report.
        size_t visibleFrames = 0;
        for (uint32_t id = 1; id < widgets_.size(); ++id) {
            const auto* w = widgets_.get(id);
            if (w && w->visible) ++visibleFrames;
        }
        if (visibleFrames > 0) {
            lastReport = now;
            LOG_INFO("WidgetInput: mouse=(", x, ",", y, ") hit=", hit,
                     " hover=", hoverWid_, " mouseEnabled=", mouseFrames,
                     " visible=", visibleFrames);
        }
    }

    // Hover first, so a frame that appears under a stationary cursor still gets
    // its OnEnter rather than waiting for the mouse to move.
    if (hit != hoverWid_) {
        if (hoverWid_ != 0) callFrameScript(hoverWid_, "OnLeave");
        hoverWid_ = hit;
        if (hoverWid_ != 0) callFrameScript(hoverWid_, "OnEnter");
    }

    // A slider follows the cursor for as long as it is held, which is the only
    // widget where what happens between press and release is the point. The
    // frame keeps the grab even when the cursor leaves it, because letting go
    // of a scroll bar by sliding sideways is not what anyone means.
    if (buttonDown_[0] && pressedWid_[0] != 0) {
        if (auto* w = widgets_.get(pressedWid_[0]); w && w->isSlider) {
            const float span = w->barMax - w->barMin;
            if (span > 0.0f) {
                // Vertical sliders run top to bottom, and the tree's y grows
                // upward, so the fraction is measured from the far edge.
                const float extent = w->barVertical ? w->rectH : w->rectW;
                float f = 0.0f;
                if (extent > 0.0f) {
                    f = w->barVertical ? (w->bottom + w->rectH - y) / extent
                                       : (x - w->left) / extent;
                }
                f = std::clamp(f, 0.0f, 1.0f);
                float value = w->barMin + f * span;
                if (w->sliderStep > 0.0f) {
                    value = w->barMin +
                            std::round((value - w->barMin) / w->sliderStep) * w->sliderStep;
                    value = std::clamp(value, w->barMin, w->barMax);
                }
                if (value != w->barValue) {
                    w->barValue = value;
                    // OnValueChanged is what a scroll frame listens to; without
                    // it the thumb would move and nothing would scroll.
                    callFrameScript(pressedWid_[0], "OnValueChanged");
                }
            }
        }
    }

    // The names WoW uses, in the order the state arrays are indexed.
    struct Button { const char* name; bool down; };
    const Button pressed[kMouseButtons] = {
        {"LeftButton",   buttons.left},
        {"RightButton",  buttons.right},
        {"MiddleButton", buttons.middle},
    };

    for (int i = 0; i < kMouseButtons; ++i) {
        const Button& b = pressed[i];
        if (b.down && !buttonDown_[i]) {
            buttonDown_[i] = true;
            pressedWid_[i] = hit;
            // Clicking into an edit box takes focus; clicking anywhere else
            // gives it up, which is what makes a chat box stop eating keys.
            if (i == 0) {
                const auto* hw = hit ? widgets_.get(hit) : nullptr;
                setEditFocus(hw && hw->isEditBox ? hit : 0);
            }
            if (pressedWid_[i] != 0)
                callFrameScript(pressedWid_[i], "OnMouseDown", b.name);
        } else if (!b.down && buttonDown_[i]) {
            buttonDown_[i] = false;
            if (pressedWid_[i] != 0) {
                callFrameScript(pressedWid_[i], "OnMouseUp", b.name);
                // A click is press and release on the same frame, which is what
                // lets a player slide off a button to change their mind.
                if (pressedWid_[i] == hit &&
                    frameAcceptsClick(pressedWid_[i], b.name))
                    callFrameScript(pressedWid_[i], "OnClick", b.name);
            }
            pressedWid_[i] = 0;
        }
    }
}

void LuaEngine::dispatchOnUpdate(float elapsed) {
    if (!L_) return;

    lua_getglobal(L_, "__WoweeOnUpdateFrames");
    if (!lua_istable(L_, -1)) { lua_pop(L_, 1); return; }

    int count = static_cast<int>(lua_objlen(L_, -1));
    for (int i = 1; i <= count; i++) {
        lua_rawgeti(L_, -1, i);
        if (!lua_istable(L_, -1)) { lua_pop(L_, 1); continue; }

        // Check if frame is visible
        lua_getfield(L_, -1, "__visible");
        bool visible = lua_toboolean(L_, -1);
        lua_pop(L_, 1);
        if (!visible) { lua_pop(L_, 1); continue; }

        // Get OnUpdate script
        lua_getfield(L_, -1, "__scripts");
        if (lua_istable(L_, -1)) {
            // Below the function, so a handler that fails every frame says
            // where it was reached from rather than only which line broke.
            lua_pushcfunction(L_, luaTracebackHandler);
            const int hIdx = lua_gettop(L_);
            lua_getfield(L_, hIdx - 1, "OnUpdate");
            if (lua_isfunction(L_, -1)) {
                lua_pushvalue(L_, hIdx - 2);  // self (frame)
                lua_pushnumber(L_, static_cast<double>(elapsed));
                if (lua_pcall(L_, 2, 0, hIdx) != 0) {
                    const char* uerr = lua_tostring(L_, -1);
                    std::string uerrStr = uerr ? uerr : "(unknown)";
                    lua_pop(L_, 1);

                    // A handler that fails once will fail every frame, and this
                    // runs every frame: five broken OnUpdates produced five and
                    // a half thousand identical errors in one session, which
                    // costs time and buries everything else in the log.
                    //
                    // After a few tries the handler is unhooked and said so
                    // once. The frame keeps working — it simply stops being
                    // asked to do the thing it cannot do.
                    // Indexed from the handler rather than the top: hIdx - 1
                    // is __scripts, and the traceback handler now sits above
                    // it, so the old relative offsets pointed at the wrong
                    // table.
                    constexpr int kMaxConsecutiveFailures = 5;
                    const int scriptsIdx = hIdx - 1;
                    lua_getfield(L_, scriptsIdx, "__onUpdateFailures");
                    const int failures = static_cast<int>(lua_tointeger(L_, -1)) + 1;
                    lua_pop(L_, 1);
                    lua_pushinteger(L_, failures);
                    lua_setfield(L_, scriptsIdx, "__onUpdateFailures");

                    if (failures >= kMaxConsecutiveFailures) {
                        lua_pushnil(L_);
                        lua_setfield(L_, scriptsIdx, "OnUpdate");
                        LOG_ERROR("LuaEngine: OnUpdate disabled after ", failures,
                                  " failures: ", uerrStr);
                        if (luaErrorCallback_) luaErrorCallback_(uerrStr);
                    } else if (failures == 1) {
                        LOG_ERROR("LuaEngine: OnUpdate error: ", uerrStr);
                        if (luaErrorCallback_) luaErrorCallback_(uerrStr);
                    }
                } else {
                    // Consecutive, so a handler that recovers is not punished
                    // for an early stumble.
                    lua_pushinteger(L_, 0);
                    lua_setfield(L_, hIdx - 1, "__onUpdateFailures");
                }
            } else {
                lua_pop(L_, 1);   // the OnUpdate field, which was not a function
            }
            lua_pop(L_, 1);       // the traceback handler
        }
        lua_pop(L_, 2); // pop __scripts + frame
    }
    lua_pop(L_, 1); // pop __WoweeOnUpdateFrames
}

bool LuaEngine::dispatchSlashCommand(const std::string& command, const std::string& args) {
    if (!L_) return false;

    // Check each SlashCmdList entry: for key NAME, check SLASH_NAME1, SLASH_NAME2, etc.
    lua_getglobal(L_, "SlashCmdList");
    if (!lua_istable(L_, -1)) { lua_pop(L_, 1); return false; }

    std::string cmdLower = command;
    toLowerInPlace(cmdLower);

    lua_pushnil(L_);
    while (lua_next(L_, -2) != 0) {
        // Stack: SlashCmdList, key, handler
        if (!lua_isfunction(L_, -1) || !lua_isstring(L_, -2)) {
            lua_pop(L_, 1);
            continue;
        }
        const char* name = lua_tostring(L_, -2);

        // Check SLASH_<NAME>1 through SLASH_<NAME>9
        for (int i = 1; i <= 9; i++) {
            std::string globalName = "SLASH_" + std::string(name) + std::to_string(i);
            lua_getglobal(L_, globalName.c_str());
            if (lua_isstring(L_, -1)) {
                std::string slashStr = lua_tostring(L_, -1);
                toLowerInPlace(slashStr);
                if (slashStr == cmdLower) {
                    lua_pop(L_, 1); // pop global
                    // Call the handler with args
                    lua_pushvalue(L_, -1); // copy handler
                    lua_pushstring(L_, args.c_str());
                    if (lua_pcall(L_, 1, 0, 0) != 0) {
                        LOG_ERROR("LuaEngine: SlashCmdList['", name, "'] error: ",
                                  lua_tostring(L_, -1));
                        lua_pop(L_, 1);
                    }
                    lua_pop(L_, 3); // pop handler, key, SlashCmdList
                    return true;
                }
            }
            lua_pop(L_, 1); // pop global
        }
        lua_pop(L_, 1); // pop handler, keep key for next iteration
    }
    lua_pop(L_, 1); // pop SlashCmdList
    return false;
}

// ---- SavedVariables serialization ----

static void serializeLuaValue(lua_State* L, int idx, std::string& out, int indent);

static void serializeLuaTable(lua_State* L, int idx, std::string& out, int indent) {
    out += "{\n";
    std::string pad(indent + 2, ' ');
    lua_pushnil(L);
    while (lua_next(L, idx) != 0) {
        out += pad;
        // Key
        if (lua_type(L, -2) == LUA_TSTRING) {
            const char* k = lua_tostring(L, -2);
            out += "[\"";
            for (const char* p = k; *p; ++p) {
                if (*p == '"' || *p == '\\') out += '\\';
                out += *p;
            }
            out += "\"] = ";
        } else if (lua_type(L, -2) == LUA_TNUMBER) {
            out += "[" + std::to_string(static_cast<long long>(lua_tonumber(L, -2))) + "] = ";
        } else {
            lua_pop(L, 1);
            continue;
        }
        // Value
        serializeLuaValue(L, lua_gettop(L), out, indent + 2);
        out += ",\n";
        lua_pop(L, 1);
    }
    out += std::string(indent, ' ') + "}";
}

static void serializeLuaValue(lua_State* L, int idx, std::string& out, int indent) {
    switch (lua_type(L, idx)) {
        case LUA_TNIL:     out += "nil"; break;
        case LUA_TBOOLEAN: out += lua_toboolean(L, idx) ? "true" : "false"; break;
        case LUA_TNUMBER: {
            double v = lua_tonumber(L, idx);
            char buf[64];
            snprintf(buf, sizeof(buf), "%.17g", v);
            out += buf;
            break;
        }
        case LUA_TSTRING: {
            const char* s = lua_tostring(L, idx);
            out += "\"";
            for (const char* p = s; *p; ++p) {
                if (*p == '"' || *p == '\\') out += '\\';
                else if (*p == '\n') { out += "\\n"; continue; }
                else if (*p == '\r') continue;
                out += *p;
            }
            out += "\"";
            break;
        }
        case LUA_TTABLE:
            serializeLuaTable(L, idx, out, indent);
            break;
        default:
            out += "nil"; // Functions, userdata, etc. can't be serialized
            break;
    }
}

void LuaEngine::setAddonList(const std::vector<TocFile>& addons) {
    if (!L_) return;
    lua_pushnumber(L_, static_cast<double>(addons.size()));
    lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_addon_count");

    lua_newtable(L_);
    for (size_t i = 0; i < addons.size(); i++) {
        lua_newtable(L_);
        lua_pushstring(L_, addons[i].addonName.c_str());
        lua_setfield(L_, -2, "name");
        lua_pushstring(L_, addons[i].getTitle().c_str());
        lua_setfield(L_, -2, "title");
        auto notesIt = addons[i].directives.find("Notes");
        lua_pushstring(L_, notesIt != addons[i].directives.end() ? notesIt->second.c_str() : "");
        lua_setfield(L_, -2, "notes");
        // Store all TOC directives for GetAddOnMetadata
        lua_newtable(L_);
        for (const auto& [key, val] : addons[i].directives) {
            lua_pushstring(L_, val.c_str());
            lua_setfield(L_, -2, key.c_str());
        }
        lua_setfield(L_, -2, "metadata");
        lua_rawseti(L_, -2, static_cast<int>(i + 1));
    }
    lua_setfield(L_, LUA_REGISTRYINDEX, "wowee_addon_info");
}

bool LuaEngine::loadSavedVariables(const std::string& path) {
    if (!L_) return false;
    std::ifstream f(path);
    if (!f.is_open()) return false; // No saved data yet — not an error
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (content.empty()) return true;
    int err = luaL_dostring(L_, content.c_str());
    if (err != 0) {
        LOG_WARNING("LuaEngine: error loading saved variables from '", path, "': ",
                    lua_tostring(L_, -1));
        lua_pop(L_, 1);
        return false;
    }
    return true;
}

bool LuaEngine::saveSavedVariables(const std::string& path, const std::vector<std::string>& varNames) {
    if (!L_ || varNames.empty()) return false;
    std::string output;
    for (const auto& name : varNames) {
        lua_getglobal(L_, name.c_str());
        if (!lua_isnil(L_, -1)) {
            output += name + " = ";
            serializeLuaValue(L_, lua_gettop(L_), output, 0);
            output += "\n";
        }
        lua_pop(L_, 1);
    }
    if (output.empty()) return true;

    // Ensure directory exists
    size_t lastSlash = path.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        std::error_code ec;
        std::filesystem::create_directories(path.substr(0, lastSlash), ec);
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("LuaEngine: cannot write saved variables to '", path, "'");
        return false;
    }
    f << output;
    LOG_INFO("LuaEngine: saved variables to '", path, "' (", output.size(), " bytes)");
    return true;
}

namespace {

/// Appends the Lua call stack to an error message.
///
/// An error says where it happened; the interesting part is nearly always how
/// it got there. "dropdownMenu is nil at unitpopup.lua:484" cost several rounds
/// of reading to trace back to the OnLoad that started it, and the stack was
/// there the whole time — it just was not being asked for. Installed as the
/// message handler so it runs before the stack unwinds.
///
/// Written by hand rather than through debug.traceback because the debug
/// library is deliberately not opened.
int luaTracebackHandler(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    std::string out = msg ? msg : "(error)";
    for (int level = 1; level < 12; ++level) {
        lua_Debug ar;
        if (!lua_getstack(L, level, &ar)) break;
        if (!lua_getinfo(L, "Sln", &ar)) break;
        out += "\n      at ";
        out += (ar.short_src[0] ? ar.short_src : "?");
        out += ":" + std::to_string(ar.currentline);
        if (ar.name) { out += " in "; out += ar.name; }
    }
    lua_pushstring(L, out.c_str());
    return 1;
}

/// Loads and runs a chunk with the traceback handler in place. Returns the
/// same non-zero-on-error convention as luaL_dostring.
int runChunk(lua_State* L, const char* chunk, size_t len, const char* name) {
    const int base = lua_gettop(L);
    lua_pushcfunction(L, luaTracebackHandler);
    if (luaL_loadbuffer(L, chunk, len, name) != 0) {
        // A syntax error has no stack to walk; leave the message where the
        // caller expects it and drop the handler underneath it.
        lua_remove(L, base + 1);
        return 1;
    }
    const int rc = lua_pcall(L, 0, 0, base + 1);
    lua_remove(L, base + 1);
    return rc;
}

/// When the running chunk must give up. Wall clock rather than a count of VM
/// instructions: the runaway this was written for spends nearly all its time
/// inside one C binding — a table rehash that grows with every call — so it
/// executes very few Lua instructions per second and a generous instruction
/// budget never came due while the client sat frozen.
std::chrono::steady_clock::time_point gChunkDeadline{};

/// Reports where the VM actually is — the Lua source and line — which a C++
/// backtrace cannot tell you: that only names the binding being called, not
/// the loop calling it.
void runawayHook(lua_State* L, lua_Debug*) {
    if (std::chrono::steady_clock::now() < gChunkDeadline) return;

    std::string where = "unknown";
    lua_Debug info;
    if (lua_getstack(L, 0, &info) && lua_getinfo(L, "Sl", &info)) {
        where = std::string(info.short_src[0] ? info.short_src : "?") + ":" +
                std::to_string(info.currentline);
    }
    // Several levels of it, because the innermost line is often a helper and
    // the loop that will not end is the caller.
    for (int level = 1; level < 6; ++level) {
        lua_Debug up;
        if (!lua_getstack(L, level, &up) || !lua_getinfo(L, "Sln", &up)) break;
        LOG_ERROR("LuaEngine:   called from ",
                  up.short_src[0] ? up.short_src : "?", ":", up.currentline,
                  up.name ? " in " : "", up.name ? up.name : "");
    }
    // Off before unwinding, or it fires again inside the error path.
    lua_sethook(L, nullptr, 0, 0);
    LOG_ERROR("LuaEngine: runaway script aborted at ", where);
    luaL_error(L, "runaway script aborted at %s", where.c_str());
}

/// Installs the deadline for one chunk and takes it off again however that
/// chunk leaves — including by error, which is the case that matters.
struct BudgetGuard {
    lua_State* L;
    explicit BudgetGuard(lua_State* state, unsigned long long ms) : L(state) {
        if (L && ms > 0) {
            gChunkDeadline = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(ms);
            // Every few hundred instructions. A deadline is only as sharp as
            // how often it is looked at, and a loop whose every iteration sits
            // in a slow C call executes very few per second: at 10,000
            // this overran 5s by 43s and then by 99s before the check came
            // round. The check is a clock read, which costs nothing beside the
            // work it is bounding.
            lua_sethook(L, runawayHook, LUA_MASKCOUNT, 500);
        }
    }
    ~BudgetGuard() { if (L) lua_sethook(L, nullptr, 0, 0); }
};

} // namespace

void LuaEngine::bootstrap(const char* code) {
    if (luaL_dostring(L_, code) == 0) return;
    const char* e = lua_tostring(L_, -1);
    const std::string head(code, std::min<size_t>(70, std::strlen(code)));
    LOG_ERROR("LuaEngine: bootstrap chunk failed: ", e ? e : "?",
              "  [chunk began: ", head, "]");
    lua_pop(L_, 1);
}

bool LuaEngine::executeFile(const std::string& path) {
    if (!L_) return false;

    BudgetGuard guard(L_, chunkTimeoutMs_);
    // Read and run rather than luaL_dofile, so the traceback handler is in
    // place: a file that fails deep inside a handler otherwise reports only
    // the line that broke, never the OnLoad that reached it.
    std::string source;
    {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            lastError_ = "cannot open " + path;
            LOG_ERROR("LuaEngine: cannot open '", path, "'");
            return false;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        source = ss.str();
    }
    const std::string chunkName = "@" + path;
    int err = runChunk(L_, source.c_str(), source.size(), chunkName.c_str());
    if (err != 0) {
        const char* errMsg = lua_tostring(L_, -1);
        std::string msg = errMsg ? errMsg : "(unknown error)";
        lastError_ = msg;
        LOG_ERROR("LuaEngine: error loading '", path, "': ", msg);
        if (luaErrorCallback_) luaErrorCallback_(msg);
        if (gameHandler_) {
            game::MessageChatData errChat;
            errChat.type = game::ChatType::SYSTEM;
            errChat.language = game::ChatLanguage::UNIVERSAL;
            errChat.message = "|cffff4040[Lua Error] " + msg + "|r";
            gameHandler_->addLocalChatMessage(errChat);
        }
        lua_pop(L_, 1);
        return false;
    }
    return true;
}

bool LuaEngine::executeString(const std::string& code) {
    if (!L_) return false;

    BudgetGuard guard(L_, chunkTimeoutMs_);
    int err = runChunk(L_, code.c_str(), code.size(), code.c_str());
    if (err != 0) {
        const char* errMsg = lua_tostring(L_, -1);
        std::string msg = errMsg ? errMsg : "(unknown error)";
        lastError_ = msg;
        LOG_ERROR("LuaEngine: script error: ", msg);
        if (luaErrorCallback_) luaErrorCallback_(msg);
        if (gameHandler_) {
            game::MessageChatData errChat;
            errChat.type = game::ChatType::SYSTEM;
            errChat.language = game::ChatLanguage::UNIVERSAL;
            errChat.message = "|cffff4040[Lua Error] " + msg + "|r";
            gameHandler_->addLocalChatMessage(errChat);
        }
        lua_pop(L_, 1);
        return false;
    }
    return true;
}

} // namespace wowee::addons
