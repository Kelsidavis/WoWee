// lua_system_api.cpp — System, time, sound, locale, map, addons, instances, and utilities Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include <algorithm>
#include <cstring>
#include <set>
#include "addons/lua_api_helpers.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "core/app_clock.hpp"
#include "core/window.hpp"

#include <SDL2/SDL.h>
#include "game/expansion_profile.hpp"

namespace wowee::addons {

static int lua_PlaySound(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* ac = svc ? svc->audioCoordinator : nullptr;
    if (!ac) return 0;
    auto* sfx = ac->getUiSoundManager();
    if (!sfx) return 0;

    // Accept numeric sound ID or string name
    std::string sound;
    if (lua_isnumber(L, 1)) {
        uint32_t id = static_cast<uint32_t>(lua_tonumber(L, 1));
        // Map common WoW sound IDs to named sounds
        switch (id) {
            case 856: case 1115: sfx->playButtonClick(); return 0; // igMainMenuOption
            case 840: sfx->playQuestActivate(); return 0;          // igQuestListOpen
            case 841: sfx->playQuestComplete(); return 0;           // igQuestListComplete
            case 862: sfx->playBagOpen(); return 0;                // igBackPackOpen
            case 863: sfx->playBagClose(); return 0;               // igBackPackClose
            case 867: sfx->playError(); return 0;                  // igPlayerInvite
            case 888: sfx->playLevelUp(); return 0;                // LEVELUPSOUND
            default: return 0;
        }
    } else {
        const char* name = luaL_optstring(L, 1, "");
        sound = name;
        for (char& c : sound) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (sound == "IGMAINMENUOPTION" || sound == "IGMAINMENUOPTIONCHECKBOXON")
            sfx->playButtonClick();
        else if (sound == "IGQUESTLISTOPEN") sfx->playQuestActivate();
        else if (sound == "IGQUESTLISTCOMPLETE") sfx->playQuestComplete();
        else if (sound == "IGBACKPACKOPEN") sfx->playBagOpen();
        else if (sound == "IGBACKPACKCLOSE") sfx->playBagClose();
        else if (sound == "LEVELUPSOUND") sfx->playLevelUp();
        else if (sound == "IGPLAYERINVITEACCEPTED") sfx->playButtonClick();
        else if (sound == "TALENTSCREENOPEN") sfx->playCharacterSheetOpen();
        else if (sound == "TALENTSCREENCLOSE") sfx->playCharacterSheetClose();
    }
    return 0;
}

// PlaySoundFile(path) — stub (file-based sounds not loaded from Lua)
static int lua_PlaySoundFile(lua_State* L) { (void)L; return 0; }

static int lua_GetPlayerMapPosition(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) {
        const auto& mi = gh->getMovementInfo();
        lua_pushnumber(L, mi.x);
        lua_pushnumber(L, mi.y);
        return 2;
    }
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
}

// GetPlayerFacing() → radians (0 = north, increasing counter-clockwise)
static int lua_GetPlayerFacing(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) {
        float facing = gh->getMovementInfo().orientation;
        // Normalize to [0, 2π)
        while (facing < 0) facing += 6.2831853f;
        while (facing >= 6.2831853f) facing -= 6.2831853f;
        lua_pushnumber(L, facing);
    } else {
        lua_pushnumber(L, 0);
    }
    return 1;
}

// GetCVar(name) → value string (stub for most, real for a few)
static int lua_GetCVar(lua_State* L) {
    const char* name = luaL_checkstring(L, 1);
    std::string n(name);
    // Return sensible defaults for commonly queried CVars
    if (n == "uiScale") lua_pushstring(L, "1");
    else if (n == "useUIScale") lua_pushstring(L, "1");
    else if (n == "screenWidth" || n == "gxResolution") {
        auto* svc = getLuaServices(L);
        auto* win = svc ? svc->window : nullptr;
        lua_pushstring(L, std::to_string(win ? win->getWidth() : 1920).c_str());
    } else if (n == "screenHeight" || n == "gxFullscreenResolution") {
        auto* svc = getLuaServices(L);
        auto* win = svc ? svc->window : nullptr;
        lua_pushstring(L, std::to_string(win ? win->getHeight() : 1080).c_str());
    } else if (n == "nameplateShowFriends") lua_pushstring(L, "1");
    else if (n == "nameplateShowEnemies") lua_pushstring(L, "1");
    else if (n == "Sound_EnableSFX") lua_pushstring(L, "1");
    else if (n == "Sound_EnableMusic") lua_pushstring(L, "1");
    else if (n == "chatBubbles") lua_pushstring(L, "1");
    else if (n == "autoLootDefault") lua_pushstring(L, "1");
    else lua_pushstring(L, "0");
    return 1;
}

// SetCVar(name, value) — no-op stub
static int lua_SetCVar(lua_State* L) {
    (void)L;
    return 0;
}


static int lua_GetNumAddOns(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_count");
    return 1;
}

static int lua_GetAddOnInfo(lua_State* L) {
    // Accept index (1-based) or addon name
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_info");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return luaReturnNil(L);
    }

    int idx = 0;
    if (lua_isnumber(L, 1)) {
        idx = static_cast<int>(lua_tonumber(L, 1));
    } else if (lua_isstring(L, 1)) {
        // Search by name
        const char* name = lua_tostring(L, 1);
        int count = static_cast<int>(lua_objlen(L, -1));
        for (int i = 1; i <= count; i++) {
            lua_rawgeti(L, -1, i);
            lua_getfield(L, -1, "name");
            const char* aName = lua_tostring(L, -1);
            lua_pop(L, 1);
            if (aName && strcmp(aName, name) == 0) { idx = i; lua_pop(L, 1); break; }
            lua_pop(L, 1);
        }
    }

    if (idx < 1) { lua_pop(L, 1); lua_pushnil(L); return 1; }

    lua_rawgeti(L, -1, idx);
    if (!lua_istable(L, -1)) { lua_pop(L, 2); lua_pushnil(L); return 1; }

    lua_getfield(L, -1, "name");
    lua_getfield(L, -2, "title");
    lua_getfield(L, -3, "notes");
    lua_pushboolean(L, 1); // loadable (always true for now)
    lua_pushstring(L, "INSECURE"); // security
    lua_pop(L, 1); // pop addon info entry (keep others)
    // Return: name, title, notes, loadable, reason, security
    return 5;
}

// GetAddOnMetadata(addonNameOrIndex, key) → value
static int lua_GetAddOnMetadata(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_info");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); lua_pushnil(L); return 1; }

    int idx = 0;
    if (lua_isnumber(L, 1)) {
        idx = static_cast<int>(lua_tonumber(L, 1));
    } else if (lua_isstring(L, 1)) {
        const char* name = lua_tostring(L, 1);
        int count = static_cast<int>(lua_objlen(L, -1));
        for (int i = 1; i <= count; i++) {
            lua_rawgeti(L, -1, i);
            lua_getfield(L, -1, "name");
            const char* aName = lua_tostring(L, -1);
            lua_pop(L, 1);
            if (aName && strcmp(aName, name) == 0) { idx = i; lua_pop(L, 1); break; }
            lua_pop(L, 1);
        }
    }
    if (idx < 1) { lua_pop(L, 1); lua_pushnil(L); return 1; }

    const char* key = luaL_checkstring(L, 2);
    lua_rawgeti(L, -1, idx);
    if (!lua_istable(L, -1)) { lua_pop(L, 2); lua_pushnil(L); return 1; }
    lua_getfield(L, -1, "metadata");
    if (!lua_istable(L, -1)) { lua_pop(L, 3); lua_pushnil(L); return 1; }
    lua_getfield(L, -1, key);
    return 1;
}

// UnitBuff(unitId, index) / UnitDebuff(unitId, index)
// Returns: name, rank, icon, count, debuffType, duration, expirationTime, caster, isStealable, shouldConsolidate, spellId

static int lua_GetLocale(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* profile = svc && svc->expansionRegistry
        ? svc->expansionRegistry->getActive() : nullptr;
    lua_pushstring(L, profile ? profile->locale.c_str() : "enUS");
    return 1;
}

static int lua_GetBuildInfo(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* profile = svc && svc->expansionRegistry
        ? svc->expansionRegistry->getActive() : nullptr;
    if (!profile) {
        lua_pushstring(L, "3.3.5a");
        lua_pushnumber(L, 12340);
        lua_pushstring(L, "");
        lua_pushnumber(L, 30300);
        return 4;
    }

    const std::string version = profile->versionString();
    uint32_t tocVersion = 11200;
    if (profile->majorVersion == 2) tocVersion = 20400;
    else if (profile->majorVersion >= 3) tocVersion = 30300;

    lua_pushstring(L, version.c_str());
    lua_pushnumber(L, profile->build);
    lua_pushstring(L, "");
    lua_pushnumber(L, tocVersion);
    return 4;
}

static int lua_GetCurrentMapAreaID(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getCurrentMapId() : 0);
    return 1;
}

// GetZoneText() / GetRealZoneText() → current zone name
static int lua_GetZoneText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, ""); return 1; }
    uint32_t zoneId = gh->getWorldStateZoneId();
    if (zoneId != 0) {
        std::string name = gh->getWhoAreaName(zoneId);
        if (!name.empty()) { lua_pushstring(L, name.c_str()); return 1; }
    }
    lua_pushstring(L, "");
    return 1;
}

// GetSubZoneText() → subzone name (same as zone for now — server doesn't always send subzone)
static int lua_GetSubZoneText(lua_State* L) {
    return lua_GetZoneText(L);  // Best-effort: zone and subzone often overlap
}

// GetMinimapZoneText() → zone name displayed near minimap
static int lua_GetMinimapZoneText(lua_State* L) {
    return lua_GetZoneText(L);
}

// --- World Map Navigation API ---

// Map ID → continent mapping
static int mapIdToContinent(uint32_t mapId) {
    switch (mapId) {
        case 0:   return 2; // Eastern Kingdoms
        case 1:   return 1; // Kalimdor
        case 530: return 3; // Outland
        case 571: return 4; // Northrend
        default:  return 0; // Instance or unknown
    }
}

// Internal tracked map state (which continent/zone the map UI is viewing)
static int s_mapContinent = 0;
static int s_mapZone = 0;

// SetMapToCurrentZone() — sets map view to the player's current zone
static int lua_SetMapToCurrentZone(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) {
        s_mapContinent = mapIdToContinent(gh->getCurrentMapId());
        s_mapZone = static_cast<int>(gh->getWorldStateZoneId());
    }
    return 0;
}

// GetCurrentMapContinent() → continentId (1=Kalimdor, 2=EK, 3=Outland, 4=Northrend)
static int lua_GetCurrentMapContinent(lua_State* L) {
    if (s_mapContinent == 0) {
        auto* gh = getGameHandler(L);
        if (gh) s_mapContinent = mapIdToContinent(gh->getCurrentMapId());
    }
    lua_pushnumber(L, s_mapContinent);
    return 1;
}

// GetCurrentMapZone() → zoneId
static int lua_GetCurrentMapZone(lua_State* L) {
    if (s_mapZone == 0) {
        auto* gh = getGameHandler(L);
        if (gh) s_mapZone = static_cast<int>(gh->getWorldStateZoneId());
    }
    lua_pushnumber(L, s_mapZone);
    return 1;
}

// SetMapZoom(continent [, zone]) — sets map view to continent/zone
static int lua_SetMapZoom(lua_State* L) {
    s_mapContinent = static_cast<int>(luaL_checknumber(L, 1));
    s_mapZone = static_cast<int>(luaL_optnumber(L, 2, 0));
    return 0;
}

// GetMapContinents() → "Kalimdor", "Eastern Kingdoms", ...
static int lua_GetMapContinents(lua_State* L) {
    lua_pushstring(L, "Kalimdor");
    lua_pushstring(L, "Eastern Kingdoms");
    lua_pushstring(L, "Outland");
    lua_pushstring(L, "Northrend");
    return 4;
}

// GetMapZones(continent) → zone names for that continent
// Returns a basic list; addons mainly need this to not error
static int lua_GetMapZones(lua_State* L) {
    int cont = static_cast<int>(luaL_checknumber(L, 1));
    // Return a minimal representative set per continent
    switch (cont) {
        case 1: // Kalimdor
            lua_pushstring(L, "Durotar"); lua_pushstring(L, "Mulgore");
            lua_pushstring(L, "The Barrens"); lua_pushstring(L, "Teldrassil");
            return 4;
        case 2: // Eastern Kingdoms
            lua_pushstring(L, "Elwynn Forest"); lua_pushstring(L, "Westfall");
            lua_pushstring(L, "Dun Morogh"); lua_pushstring(L, "Tirisfal Glades");
            return 4;
        case 3: // Outland
            lua_pushstring(L, "Hellfire Peninsula"); lua_pushstring(L, "Zangarmarsh");
            return 2;
        case 4: // Northrend
            lua_pushstring(L, "Borean Tundra"); lua_pushstring(L, "Howling Fjord");
            return 2;
        default:
            return 0;
    }
}

// GetNumMapLandmarks() → 0 (no landmark data exposed yet)
static int lua_GetNumMapLandmarks(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}


static int lua_GetGameTime(lua_State* L) {
    // Returns server game time as hours, minutes
    auto* gh = getGameHandler(L);
    if (gh) {
        float gt = gh->getGameTime();
        int hours = static_cast<int>(gt) % 24;
        int mins = static_cast<int>((gt - static_cast<int>(gt)) * 60.0f);
        lua_pushnumber(L, hours);
        lua_pushnumber(L, mins);
    } else {
        lua_pushnumber(L, 12);
        lua_pushnumber(L, 0);
    }
    return 2;
}

static int lua_GetServerTime(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(std::time(nullptr)));
    return 1;
}


static int lua_IsInInstance(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushboolean(L, 0); lua_pushstring(L, "none"); return 2; }
    bool inInstance = gh->isInInstance();
    lua_pushboolean(L, inInstance);
    lua_pushstring(L, inInstance ? "party" : "none");  // simplified: "none", "party", "raid", "pvp", "arena"
    return 2;
}

// GetInstanceInfo() → name, type, difficultyIndex, difficultyName, maxPlayers, ...
static int lua_GetInstanceInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) {
        lua_pushstring(L, ""); lua_pushstring(L, "none"); lua_pushnumber(L, 0);
        lua_pushstring(L, "Normal"); lua_pushnumber(L, 0);
        return 5;
    }
    std::string mapName = gh->getMapName(gh->getCurrentMapId());
    lua_pushstring(L, mapName.c_str());                    // 1: name
    lua_pushstring(L, gh->isInInstance() ? "party" : "none"); // 2: instanceType
    lua_pushnumber(L, gh->getInstanceDifficulty());        // 3: difficultyIndex
    static constexpr const char* kDiff[] = {"Normal", "Heroic", "25 Normal", "25 Heroic"};
    uint32_t diff = gh->getInstanceDifficulty();
    lua_pushstring(L, (diff < 4) ? kDiff[diff] : "Normal"); // 4: difficultyName
    lua_pushnumber(L, 5);                                   // 5: maxPlayers (default 5-man)
    return 5;
}

static int lua_GetInstanceDifficulty(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? (gh->getInstanceDifficulty() + 1) : 1);
    return 1;
}

static int lua_strsplit(lua_State* L) {
    const char* delim = luaL_checkstring(L, 1);
    const char* str = luaL_checkstring(L, 2);
    if (!delim[0]) { lua_pushstring(L, str); return 1; }
    int count = 0;
    std::string s(str);
    // One value per field, and the field count follows from the string rather
    // than from anything this client chose. Lua guarantees only a small slack
    // above the arguments, and pushing past the top corrupts the heap rather
    // than raising — so the room is asked for before any of it is used.
    // A chat line of commas is all it takes.
    const size_t fields = static_cast<size_t>(
        std::count(s.begin(), s.end(), delim[0])) + 1;
    if (!lua_checkstack(L, static_cast<int>(fields) + 1)) {
        lua_pushstring(L, str);
        return 1;
    }
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t found = s.find(delim[0], pos);
        if (found == std::string::npos) {
            lua_pushstring(L, s.substr(pos).c_str());
            count++;
            break;
        }
        lua_pushstring(L, s.substr(pos, found - pos).c_str());
        count++;
        pos = found + 1;
    }
    return count;
}

// strtrim(str) — remove leading/trailing whitespace
static int lua_strtrim(lua_State* L) {
    const char* str = luaL_checkstring(L, 1);
    std::string s(str);
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    lua_pushstring(L, (start == std::string::npos) ? "" : s.substr(start, end - start + 1).c_str());
    return 1;
}

// wipe(table) — clear all entries from a table
static int lua_wipe(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    // Remove all integer keys
    int len = static_cast<int>(lua_objlen(L, 1));
    for (int i = len; i >= 1; i--) {
        lua_pushnil(L);
        lua_rawseti(L, 1, i);
    }
    // Remove all string keys
    lua_pushnil(L);
    while (lua_next(L, 1) != 0) {
        lua_pop(L, 1);       // pop value
        lua_pushvalue(L, -1); // copy key
        lua_pushnil(L);
        lua_rawset(L, 1);    // table[key] = nil
    }
    lua_pushvalue(L, 1);
    return 1;
}

// date(format) — safe date function (os.date was removed)
static int lua_wow_date(lua_State* L) {
    const char* fmt = luaL_optstring(L, 1, "%c");
    time_t now = time(nullptr);
    struct tm* tm = localtime(&now);
    char buf[256];
    strftime(buf, sizeof(buf), fmt, tm);
    lua_pushstring(L, buf);
    return 1;
}

// time() — current unix timestamp
static int lua_wow_time(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(time(nullptr)));
    return 1;
}

// GetTime() — returns elapsed seconds since engine start (shared epoch)
static int lua_wow_gettime(lua_State* L) {
    lua_pushnumber(L, luaGetTimeNow());
    return 1;
}

// Names FrameXML reaches for that this client has no state behind yet.
//
// Found by running the load with the missing-API fallback off, which is the
// only way to see them: with it on they answer and the gap is invisible.
// IsThreatWarningEnabled alone was asked 58 times in one load.
//
// Answering falsely is the point. Each returns what the feature being absent
// looks like — no threat warnings, no runes, nobody to pass loot to — so the
// caller takes the branch it would take on a client where that feature is off,
// rather than dividing by a nil.
static int lua_ReturnFalse(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_ReturnTrue(lua_State* L)  { lua_pushboolean(L, 1); return 1; }
static int lua_ReturnNil(lua_State* L)   { lua_pushnil(L); return 1; }
static int lua_ReturnZero(lua_State* L)  { lua_pushnumber(L, 0.0); return 1; }
static int lua_ReturnNothing(lua_State*) { return 0; }

/// A cooldown that is not running: start and duration both zero. Two values,
/// because the caller adds them together on the next line —
/// local start, duration = GetSummonFriendCooldown(); start + duration — and
/// one of them missing is arithmetic on nil.
/// The resolutions this client offers, as "WIDTHxHEIGHT" strings, and which of
/// them is current. One entry — the window as it actually is — because this
/// client does not enumerate modes.
///
/// UpdateMenuBarTop reads them together and immediately divides:
///   string.match((({GetScreenResolutions()})[GetCurrentResolution()] or ""),
///                "(%d+).-(%d+)")
/// then tonumber(width) / tonumber(height). An empty list makes both nil.
static int lua_GetScreenResolutions(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* win = svc ? svc->window : nullptr;
    const int w = win ? win->getWidth() : 1920;
    const int h = win ? win->getHeight() : 1080;
    lua_pushstring(L, (std::to_string(w) + "x" + std::to_string(h)).c_str());
    return 1;
}

static int lua_GetCurrentResolution(lua_State* L) {
    lua_pushnumber(L, 1.0);
    return 1;
}

/// A position on the battlefield map for someone who is not there: origin and
/// no name. Three values, because WorldMapFrame_Update multiplies the first
/// two by the map's dimensions on the line after reading them, and its loop is
/// bounded by MAX_RAID_MEMBERS rather than by how many are actually present.
static int lua_GetBattlefieldPosition(lua_State* L) {
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    lua_pushstring(L, "");
    return 3;
}

/// A chat window's saved settings. FCF_SetWindowAlpha takes the alpha from
/// here and remembers it as oldAlpha, which the fade handlers then hand to
/// max() on every mouse-over — so a missing alpha is not a cosmetic gap, it is
/// an error every time the cursor crosses the frame.
static int lua_GetChatWindowInfo(lua_State* L) {
    // Which window is being asked about matters: docked is that window's place
    // on the dock, and FCF_DockFrame asserts that whatever claims position one
    // is the dock's primary. Answering one for every window claimed each was
    // first. WoW's default layout docks General and the combat log and leaves
    // the rest neither shown nor docked.
    const int id = static_cast<int>(luaL_optnumber(L, 1, 1));
    const bool inDefaultLayout = (id >= 1 && id <= 2);

    lua_pushstring(L, "");      // name
    lua_pushnumber(L, 14.0);    // fontSize
    lua_pushnumber(L, 1.0);     // r
    lua_pushnumber(L, 1.0);     // g
    lua_pushnumber(L, 1.0);     // b
    lua_pushnumber(L, 1.0);     // alpha
    // Numbers and nil, not booleans. docked is a dock position, not a flag —
    // FCF_LoadChatSettings hands it straight to FCF_DockFrame as the index to
    // insert at, and that compares it against a count. A boolean there is a
    // comparison between a boolean and a number, which is an error rather than
    // a wrong answer.
    if (inDefaultLayout) lua_pushnumber(L, 1.0); else lua_pushnil(L);  // shown
    lua_pushnil(L);                                                     // locked
    if (inDefaultLayout) lua_pushnumber(L, id); else lua_pushnil(L);    // docked
    lua_pushnil(L);                                                     // uninteractable
    return 10;
}

/// Names FrameXML reached for and did not find, harvested from a run rather
/// than guessed at: the fallback records each once and reports the list at
/// shutdown. Each answers with what the feature being absent looks like, so
/// the caller takes the branch it would take on a client where that feature is
/// switched off.
static int lua_GetMapInfo(lua_State* L) {
    // mapFileName, textureHeight, textureWidth. WorldMapFrame builds a texture
    // path out of the first and divides by the other two.
    lua_pushstring(L, "");
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    return 3;
}

/// The expansion this client speaks. Two is Wrath, which is what the wire
/// format and the DBC layouts here assume.
static int lua_GetExpansionLevel(lua_State* L) {
    lua_pushnumber(L, 2.0);
    return 1;
}

/// Normal, which is the difficulty a fresh group is on.
static int lua_ReturnOne(lua_State* L) {
    lua_pushnumber(L, 1.0);
    return 1;
}

static int lua_GetDefaultLanguage(lua_State* L) {
    auto* gh = getGameHandler(L);
    static const std::set<uint8_t> kHordeRaces = {2, 5, 6, 8, 10};
    const bool horde = gh && kHordeRaces.count(gh->getPlayerRace()) > 0;
    lua_pushstring(L, horde ? "Orcish" : "Common");
    return 1;
}

/// No enchant on either hand: four values per hand, and MainMenuBar reads the
/// expiry as a number.
static int lua_GetWeaponEnchantInfo(lua_State* L) {
    for (int i = 0; i < 2; ++i) {
        lua_pushboolean(L, 0);   // hasEnchant
        lua_pushnumber(L, 0.0);  // expiration
        lua_pushnumber(L, 0.0);  // charges
    }
    return 6;
}

/// Which modifier keys are held. Answered from the real keyboard rather than
/// falsely, because a shift-click means something different from a click and
/// FrameXML asks these on every button press.
static int lua_IsShiftKeyDown(lua_State* L) {
    lua_pushboolean(L, (SDL_GetModState() & KMOD_SHIFT) != 0);
    return 1;
}
static int lua_IsControlKeyDown(lua_State* L) {
    lua_pushboolean(L, (SDL_GetModState() & KMOD_CTRL) != 0);
    return 1;
}
static int lua_IsAltKeyDown(lua_State* L) {
    lua_pushboolean(L, (SDL_GetModState() & KMOD_ALT) != 0);
    return 1;
}
static int lua_IsModifierKeyDown(lua_State* L) {
    const SDL_Keymod m = SDL_GetModState();
    lua_pushboolean(L, (m & (KMOD_SHIFT | KMOD_CTRL | KMOD_ALT)) != 0);
    return 1;
}

/// Whether an addon is loaded. Real rather than false: the registry holds the
/// addons that were enabled and loaded this session, and FrameXML asks before
/// deciding whether a feature exists — answering no where the answer is yes
/// hides an addon from the interface that is meant to work with it.
static int lua_IsAddOnLoaded(lua_State* L) {
    const char* wanted = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
    if (!wanted) { lua_pushboolean(L, 0); return 1; }

    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_info");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); lua_pushboolean(L, 0); return 1; }
    const int count = static_cast<int>(lua_objlen(L, -1));
    bool found = false;
    for (int i = 1; i <= count && !found; ++i) {
        lua_rawgeti(L, -1, i);
        if (lua_istable(L, -1)) {
            lua_getfield(L, -1, "name");
            const char* name = lua_tostring(L, -1);
            found = name && std::strcmp(name, wanted) == 0;
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    lua_pushboolean(L, found ? 1 : 0);
    return 1;
}

/// GetTime() → seconds since the client started, as a float.
///
/// The interface's shared reference for anything timed: a cooldown records
/// GetTime() + duration and something else compares against it later. It was
/// never implemented, so every one of those comparisons was against nil —
/// including the ones in this client's own bootstrap.
static int lua_GetTime(lua_State* L) {
    lua_pushnumber(L, wowee::core::appTimeSeconds());
    return 1;
}

/// LoadAddOn(name) → loaded, reason.
///
/// The reason is not optional when loaded is false: UIParentLoadAddOn builds
/// an error message out of _G["ADDON_" .. reason], so nil there is a
/// concatenation against nothing. These are Blizzard's own load-on-demand
/// panels — the talent frame and its like — which this client does not ship,
/// and MISSING is the reason string for exactly that.
static int lua_LoadAddOn(lua_State* L) {
    lua_pushboolean(L, 0);
    lua_pushstring(L, "MISSING");
    return 2;
}

static int lua_ReturnNoCooldown(lua_State* L) {
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    return 2;
}

/// Alliance or Horde for the player, and the same for anyone else until this
/// client tracks other units' factions. Returns the English tag and the
/// localised name, which is the pair FrameXML expects.
static int lua_UnitFactionGroup(lua_State* L) {
    auto* gh = getGameHandler(L);
    bool horde = false;
    if (gh) {
        // Orc, Undead, Tauren, Troll, Blood Elf.
        static const std::set<uint8_t> kHordeRaces = {2, 5, 6, 8, 10};
        horde = kHordeRaces.count(gh->getPlayerRace()) > 0;
    }
    lua_pushstring(L, horde ? "Horde" : "Alliance");
    lua_pushstring(L, horde ? "Horde" : "Alliance");
    return 2;
}

void registerSystemLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"GetTime",                  lua_GetTime},
                {"GetCVarDefault",           lua_GetCVar},
                {"IsAddOnLoaded",            lua_IsAddOnLoaded},
                {"LoadAddOn",                lua_LoadAddOn},
                {"UIParentLoadAddOn",        lua_LoadAddOn},
                {"HasCompletedAnyAchievement", lua_ReturnFalse},
                {"TurnInGuildCharter",       lua_ReturnNothing},
                // Nothing is being driven, so aiming it does nothing
                // and there is nothing to climb out of.
                {"VehicleAimUpStart",        lua_ReturnNothing},
                {"VehicleAimUpStop",         lua_ReturnNothing},
                {"VehicleAimDownStart",      lua_ReturnNothing},
                {"VehicleAimDownStop",       lua_ReturnNothing},
                {"VehicleExit",              lua_ReturnNothing},
                {"VehicleAimGetNormAngle",   lua_ReturnZero},
                {"VehicleAimGetNormPower",   lua_ReturnZero},
                {"GetMapInfo",               lua_GetMapInfo},
                {"GetExpansionLevel",        lua_GetExpansionLevel},
                {"GetDungeonDifficulty",     lua_ReturnOne},
                {"GetRaidDifficulty",        lua_ReturnOne},
                {"GetChatTypeIndex",         lua_ReturnOne},
                {"GetDefaultLanguage",       lua_GetDefaultLanguage},
                {"GetWeaponEnchantInfo",     lua_GetWeaponEnchantInfo},
                {"GetCorpseRecoveryDelay",   lua_ReturnZero},
                {"GetAdjustedSkillPoints",   lua_ReturnZero},
                {"GetPartyLeaderIndex",      lua_ReturnZero},
                {"GetNumArenaOpponents",     lua_ReturnZero},
                {"GetCurrentMultisampleFormat", lua_ReturnOne},
                // These hand back a list, not a value: the caller walks it with
                // select("#", ...) and reads it in groups. One number makes the
                // loop run once against nils, which is worse than an empty
                // list — for anything returning a list, nothing is the right
                // way to say there is none.
                {"GetMultisampleFormats",    lua_ReturnNothing},
                {"GetRefreshRates",          lua_ReturnNothing},
                {"GetCompanionInfo",         lua_ReturnNil},
                {"GetMultiCastTotemSpells",  lua_ReturnNil},
                {"GetPossessInfo",           lua_ReturnNil},
                {"GetVoiceStatus",           lua_ReturnFalse},
                {"GetMuteStatus",            lua_ReturnFalse},
                {"GetActiveVoiceChannel",    lua_ReturnNil},
                {"GetVoiceCurrentSessionID", lua_ReturnNil},
                {"GetOptOutOfLoot",          lua_ReturnFalse},
                {"GetPartyMember",           lua_ReturnFalse},
                {"GetZonePVPInfo",           lua_ReturnNil},
                {"GetMouseButtonClicked",    lua_ReturnNil},
                {"GetChatWindowSavedPosition",   lua_ReturnNil},
                {"GetChatWindowSavedDimensions", lua_ReturnNil},
                {"GetExistingLocales",       lua_ReturnNil},
                {"GetGuildRosterSelection",  lua_ReturnZero},
                // Read from the real keyboard: a shift-click means something
                // different from a click, and FrameXML asks on every press.
                {"IsShiftKeyDown",           lua_IsShiftKeyDown},
                {"IsLeftShiftKeyDown",       lua_IsShiftKeyDown},
                {"IsRightShiftKeyDown",      lua_IsShiftKeyDown},
                {"IsControlKeyDown",         lua_IsControlKeyDown},
                {"IsLeftControlKeyDown",     lua_IsControlKeyDown},
                {"IsRightControlKeyDown",    lua_IsControlKeyDown},
                {"IsAltKeyDown",             lua_IsAltKeyDown},
                {"IsLeftAltKeyDown",         lua_IsAltKeyDown},
                {"IsRightAltKeyDown",        lua_IsAltKeyDown},
                {"IsModifierKeyDown",        lua_IsModifierKeyDown},
                {"IsAttackAction",           lua_ReturnFalse},
                {"IsConsumableAction",       lua_ReturnFalse},
                {"IsEquippedAction",         lua_ReturnFalse},
                {"IsStackableAction",        lua_ReturnFalse},
                {"IsFlyableArea",            lua_ReturnFalse},
                {"IsIndoors",                lua_ReturnFalse},
                {"IsOutdoors",               lua_ReturnTrue},
                {"IsHarmfulItem",            lua_ReturnFalse},
                {"IsHelpfulItem",            lua_ReturnFalse},
                {"IsHarmfulSpell",           lua_ReturnFalse},
                {"IsHelpfulSpell",           lua_ReturnFalse},
                {"IsPossessBarVisible",      lua_ReturnFalse},
                {"IsRaidOfficer",            lua_ReturnFalse},
                {"IsReferAFriendLinked",     lua_ReturnFalse},
                {"IsStereoVideoAvailable",   lua_ReturnFalse},
                {"IsVoiceChatEnabled",       lua_ReturnFalse},
                {"IsZoomOutAvailable",       lua_ReturnFalse},
                {"HasDebugZoneMap",          lua_ReturnFalse},
                {"CanQueueForWintergrasp",   lua_ReturnFalse},
                {"CancelSkillUps",           lua_ReturnNothing},
                {"ConvertToRaid",            lua_ReturnNothing},
                {"FillLocalizedClassList",   lua_ReturnNothing},
                {"QueryGuildEventLog",       lua_ReturnNothing},
                {"RegisterForSave",          lua_ReturnNothing},
                {"RegisterStaticConstants",  lua_ReturnNothing},
                {"SetAbandonQuest",          lua_ReturnNothing},
                {"SetChatWindowDocked",      lua_ReturnNothing},
                {"SetChatWindowLocked",      lua_ReturnNothing},
                {"SetChatWindowName",        lua_ReturnNothing},
                {"SetGuildRosterSelection",  lua_ReturnNothing},
                {"SetupFullscreenScale",     lua_ReturnNothing},
                {"DropCursorMoney",          lua_ReturnNothing},
                {"BNFeaturesEnabled",        lua_ReturnFalse},
                {"BNFeaturesEnabledAndConnected", lua_ReturnFalse},
                {"BNGetMaxPlayersInConversation", lua_ReturnZero},
                {"GetSummonFriendCooldown",  lua_ReturnNoCooldown},
                {"GetScreenResolutions",     lua_GetScreenResolutions},
                {"GetCurrentResolution",     lua_GetCurrentResolution},
                // Counts a loop bounds itself with. FrameXML writes
                // "for i = 0, num-1" straight after asking, so nothing is not
                // an answer — it is arithmetic on nil and the file is lost.
                {"GetNumDisplayChannels",    lua_ReturnZero},
                {"GetNumMapOverlays",        lua_ReturnZero},
                {"GetNumMapDebugObjects",    lua_ReturnZero},
                {"GetNumBattlefieldPositions", lua_ReturnZero},
                {"GetBattlefieldPosition",   lua_GetBattlefieldPosition},
                {"GetCorpseMapPosition",     lua_GetBattlefieldPosition},
                {"GetDeathReleasePosition",  lua_GetBattlefieldPosition},
                {"GetNumBattlefieldVehicles", lua_ReturnZero},
                {"GetBattlefieldVehicleInfo", lua_ReturnNil},
                {"GetChatWindowInfo",        lua_GetChatWindowInfo},
                {"SetChatWindowAlpha",       lua_ReturnNothing},
                {"GetNumBattlefieldFlagPositions", lua_ReturnZero},
                {"GetBattlefieldFlagPosition",     lua_GetBattlefieldPosition},
                {"GuildControlGetNumRanks",  lua_ReturnZero},
                // Time left on a loot roll that is not running, which
                // GroupLootFrame compares against a bar range at once.
                {"GetLootRollTimeLeft",      lua_ReturnZero},
                {"GetNumDungeonMapLevels",   lua_ReturnZero},
                // Bar offsets, added to a page number the line they
                // are read on. No bonus or multi-cast bar is showing,
                // and that is zero rather than nothing.
                {"GetMultiCastBarOffset",    lua_ReturnZero},
                {"GetBonusBarOffset",        lua_ReturnZero},
                {"GetNumBattlegroundTypes",  lua_ReturnZero},
                {"GetCurrentMapDungeonLevel", lua_ReturnZero},
                {"Sound_GameSystem_GetNumOutputDrivers", lua_ReturnZero},
                {"Sound_ChatSystem_GetNumInputDrivers",  lua_ReturnZero},
                {"Sound_ChatSystem_GetNumOutputDrivers", lua_ReturnZero},
                {"Sound_ChatSystem_GetInputDriverNameByIndex",  lua_ReturnNil},
                {"Sound_ChatSystem_GetOutputDriverNameByIndex", lua_ReturnNil},
                // Nothing selected, which is a number rather than
                // nothing: SkillFrame passes the result straight to
                // GetSkillLineInfo as an index.
                {"GetSelectedSkill",         lua_ReturnZero},
                {"DungeonUsesTerrainMap",    lua_ReturnFalse},
                {"GetChannelDisplayInfo",    lua_ReturnNil},
                {"IsThreatWarningEnabled",   lua_ReturnFalse},
                {"IsAutoRepeatAction",       lua_ReturnFalse},
                {"IsPetAttackAction",        lua_ReturnFalse},
                {"IsMacClient",              lua_ReturnFalse},
                {"IsPartyLeader",            lua_ReturnFalse},
                {"UnitFactionGroup",         lua_UnitFactionGroup},
                {"HasPetSpells",             lua_ReturnNil},
                {"GetRuneType",              lua_ReturnNil},
                {"GetMasterLootCandidate",   lua_ReturnNil},
                {"GetSelectedDisplayChannel", lua_ReturnNil},
                {"GetWintergraspWaitTime",   lua_ReturnNil},
                {"GetNumWorldStateUI",       lua_ReturnZero},
                {"GetNumVoiceSessions",      lua_ReturnZero},
                {"RequestBattlefieldPositions", lua_ReturnNothing},
                {"UpdateWorldMapArrowFrames",   lua_ReturnNothing},
                {"SetSelectedSkill",         lua_ReturnNothing},
                {"Sound_GameSystem_GetOutputDriverNameByIndex", lua_ReturnNil},
                {"PlaySound",           lua_PlaySound},
                {"PlaySoundFile",       lua_PlaySoundFile},
                {"GetPlayerMapPosition", lua_GetPlayerMapPosition},
                {"GetPlayerFacing",     lua_GetPlayerFacing},
                {"GetCVar",             lua_GetCVar},
                {"SetCVar",             lua_SetCVar},
                {"GetLocale",         lua_GetLocale},
                {"GetBuildInfo",      lua_GetBuildInfo},
                {"GetCurrentMapAreaID", lua_GetCurrentMapAreaID},
                {"SetMapToCurrentZone", lua_SetMapToCurrentZone},
                {"GetCurrentMapContinent", lua_GetCurrentMapContinent},
                {"GetCurrentMapZone",   lua_GetCurrentMapZone},
                {"SetMapZoom",          lua_SetMapZoom},
                {"GetMapContinents",    lua_GetMapContinents},
                {"GetMapZones",         lua_GetMapZones},
                {"GetNumMapLandmarks",  lua_GetNumMapLandmarks},
                {"GetZoneText",          lua_GetZoneText},
                {"GetRealZoneText",      lua_GetZoneText},
                {"GetSubZoneText",       lua_GetSubZoneText},
                {"GetMinimapZoneText",   lua_GetMinimapZoneText},
                {"GetGameTime",             lua_GetGameTime},
                {"GetServerTime",           lua_GetServerTime},
                {"GetNumAddOns",      lua_GetNumAddOns},
                {"GetAddOnInfo",      lua_GetAddOnInfo},
                {"GetAddOnMetadata",  lua_GetAddOnMetadata},
                {"IsInInstance",         lua_IsInInstance},
                {"GetInstanceInfo",      lua_GetInstanceInfo},
                {"GetInstanceDifficulty", lua_GetInstanceDifficulty},
                {"strsplit",          lua_strsplit},
                {"strtrim",           lua_strtrim},
                {"wipe",              lua_wipe},
                {"date",              lua_wow_date},
                {"time",              lua_wow_time},
                {"GetTime",           lua_wow_gettime},
                {"IsConnectedToServer", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->isConnected() ? 1 : 0);
            return 1;
        }},
                {"GetRealmName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) {
                const auto* ac = gh->getActiveCharacter();
                lua_pushstring(L, ac ? "WoWee" : "Unknown");
            } else lua_pushstring(L, "Unknown");
            return 1;
        }},
                {"GetNormalizedRealmName", [](lua_State* L) -> int {
            lua_pushstring(L, "WoWee");
            return 1;
        }},
                {"ShowHelm", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->toggleHelm(); // Toggles helm visibility
            return 0;
        }},
                {"ShowCloak", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->toggleCloak();
            return 0;
        }},
                {"TogglePVP", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->togglePvp();
            return 0;
        }},
                {"Minimap_Ping", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            float x = static_cast<float>(luaL_optnumber(L, 1, 0));
            float y = static_cast<float>(luaL_optnumber(L, 2, 0));
            if (gh) gh->sendMinimapPing(x, y);
            return 0;
        }},
                {"RequestTimePlayed", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->requestPlayedTime();
            return 0;
        }},
                {"Logout", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->requestLogout();
            return 0;
        }},
                {"CancelLogout", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->cancelLogout();
            return 0;
        }},
                {"NumTaxiNodes", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getTaxiNodes().size() : 0);
            return 1;
        }},
                {"TaxiNodeName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh) { lua_pushstring(L, ""); return 1; }
            int i = 0;
            for (const auto& [id, node] : gh->getTaxiNodes()) {
                if (++i == index) {
                    lua_pushstring(L, node.name.c_str());
                    return 1;
                }
            }
            lua_pushstring(L, "");
            return 1;
        }},
                {"TaxiNodeGetType", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh) { return luaReturnZero(L); }
            int i = 0;
            for (const auto& [id, node] : gh->getTaxiNodes()) {
                if (++i == index) {
                    bool known = gh->isKnownTaxiNode(id);
                    lua_pushnumber(L, known ? 1 : 0); // 0=none, 1=reachable, 2=current
                    return 1;
                }
            }
            lua_pushnumber(L, 0);
            return 1;
        }},
                {"TakeTaxiNode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh) return 0;
            int i = 0;
            for (const auto& [id, node] : gh->getTaxiNodes()) {
                if (++i == index) {
                    gh->activateTaxi(id);
                    break;
                }
            }
            return 0;
        }},
                {"GetNetStats", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            uint32_t ms = gh ? gh->getLatencyMs() : 0;
            lua_pushnumber(L, 0);   // bandwidthIn
            lua_pushnumber(L, 0);   // bandwidthOut
            lua_pushnumber(L, ms);  // latencyHome
            lua_pushnumber(L, ms);  // latencyWorld
            return 4;
        }},
                {"GetCurrentTitle", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getChosenTitleBit() : -1);
            return 1;
        }},
                {"GetTitleName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int bit = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || bit < 0) { return luaReturnNil(L); }
            std::string title = gh->getFormattedTitle(static_cast<uint32_t>(bit));
            if (title.empty()) { return luaReturnNil(L); }
            lua_pushstring(L, title.c_str());
            return 1;
        }},
                {"SetCurrentTitle", [](lua_State* L) -> int {
            (void)L; // Title changes require CMSG_SET_TITLE which we don't expose yet
            return 0;
        }},
                {"GetInspectSpecialization", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* ir = gh ? gh->getInspectResult() : nullptr;
            lua_pushnumber(L, ir ? ir->activeTalentGroup : 0);
            return 1;
        }},
                {"NotifyInspect", [](lua_State* L) -> int {
            (void)L; // Inspect is auto-triggered by the C++ side when targeting a player
            return 0;
        }},
                {"ClearInspectPlayer", [](lua_State* L) -> int {
            (void)L;
            return 0;
        }},
                {"GetHonorCurrency", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getHonorPoints() : 0);
            return 1;
        }},
                {"GetArenaCurrency", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getArenaPoints() : 0);
            return 1;
        }},
                {"GetTimePlayed", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
            lua_pushnumber(L, gh->getTotalTimePlayed());
            lua_pushnumber(L, gh->getLevelTimePlayed());
            return 2;
        }},
                {"GetBindLocation", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushstring(L, "Unknown"); return 1; }
            lua_pushstring(L, gh->getWhoAreaName(gh->getHomeBindZoneId()).c_str());
            return 1;
        }},
                {"GetNumSavedInstances", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getInstanceLockouts().size() : 0);
            return 1;
        }},
                {"GetSavedInstanceInfo", [](lua_State* L) -> int {
            // GetSavedInstanceInfo(index) → name, id, reset, difficulty, locked, extended, instanceIDMostSig, isRaid, maxPlayers
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) { return luaReturnNil(L); }
            const auto& lockouts = gh->getInstanceLockouts();
            if (index > static_cast<int>(lockouts.size())) { return luaReturnNil(L); }
            const auto& l = lockouts[index - 1];
            lua_pushstring(L, ("Instance " + std::to_string(l.mapId)).c_str()); // name (would need MapDBC for real names)
            lua_pushnumber(L, l.mapId);             // id
            lua_pushnumber(L, static_cast<double>(l.resetTime - static_cast<uint64_t>(time(nullptr)))); // reset (seconds until)
            lua_pushnumber(L, l.difficulty);        // difficulty
            lua_pushboolean(L, l.locked ? 1 : 0);  // locked
            lua_pushboolean(L, l.extended ? 1 : 0); // extended
            lua_pushnumber(L, 0);                   // instanceIDMostSig
            lua_pushboolean(L, l.difficulty >= 2 ? 1 : 0); // isRaid (25-man = raid)
            lua_pushnumber(L, l.difficulty >= 2 ? 25 : (l.difficulty >= 1 ? 10 : 5)); // maxPlayers
            return 9;
        }},
                {"CalendarGetDate", [](lua_State* L) -> int {
            // CalendarGetDate() → weekday, month, day, year
            time_t now = time(nullptr);
            struct tm* t = localtime(&now);
            lua_pushnumber(L, t->tm_wday + 1); // weekday (1=Sun)
            lua_pushnumber(L, t->tm_mon + 1);  // month (1-12)
            lua_pushnumber(L, t->tm_mday);     // day
            lua_pushnumber(L, t->tm_year + 1900); // year
            return 4;
        }},
                {"CalendarGetNumPendingInvites", [](lua_State* L) -> int {
            return luaReturnZero(L);
        }},
                {"CalendarGetNumDayEvents", [](lua_State* L) -> int {
            return luaReturnZero(L);
        }},
                {"GetDifficultyInfo", [](lua_State* L) -> int {
            // GetDifficultyInfo(id) → name, groupType, isHeroic, maxPlayers
            int diff = static_cast<int>(luaL_checknumber(L, 1));
            struct DiffInfo { const char* name; const char* group; int heroic; int maxPlayers; };
            static const DiffInfo infos[] = {
                {"5 Player", "party", 0, 5},          // 0: Normal 5-man
                {"5 Player (Heroic)", "party", 1, 5},  // 1: Heroic 5-man
                {"10 Player", "raid", 0, 10},          // 2: 10-man Normal
                {"25 Player", "raid", 0, 25},          // 3: 25-man Normal
                {"10 Player (Heroic)", "raid", 1, 10}, // 4: 10-man Heroic
                {"25 Player (Heroic)", "raid", 1, 25}, // 5: 25-man Heroic
            };
            if (diff >= 0 && diff < 6) {
                lua_pushstring(L, infos[diff].name);
                lua_pushstring(L, infos[diff].group);
                lua_pushboolean(L, infos[diff].heroic);
                lua_pushnumber(L, infos[diff].maxPlayers);
            } else {
                lua_pushstring(L, "Unknown");
                lua_pushstring(L, "party");
                lua_pushboolean(L, 0);
                lua_pushnumber(L, 5);
            }
            return 4;
        }},
                {"GetWeatherInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
            lua_pushnumber(L, gh->getWeatherType());
            lua_pushnumber(L, gh->getWeatherIntensity());
            return 2;
        }},
                {"GetMaxPlayerLevel", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            auto* reg = svc ? svc->expansionRegistry : nullptr;
            auto* prof = reg ? reg->getActive() : nullptr;
            if (prof && prof->id == "wotlk") lua_pushnumber(L, 80);
            else if (prof && prof->id == "tbc") lua_pushnumber(L, 70);
            else lua_pushnumber(L, 60);
            return 1;
        }},
                {"GetAccountExpansionLevel", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            auto* reg = svc ? svc->expansionRegistry : nullptr;
            auto* prof = reg ? reg->getActive() : nullptr;
            if (prof && prof->id == "wotlk") lua_pushnumber(L, 3);
            else if (prof && prof->id == "tbc") lua_pushnumber(L, 2);
            else lua_pushnumber(L, 1);
            return 1;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
