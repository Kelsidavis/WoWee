// lua_system_api.cpp - System, time, sound, locale, map, addons, instances, and utilities Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <set>
#include <utility>
#include <vector>
#include "game/group_defines.hpp"
#include "core/version.hpp"
#include "core/config_paths.hpp"
#include "ui/settings_schema.hpp"
#include "imgui.h"
#include "addons/lua_api_helpers.hpp"
#include "addons/chat_window_background.hpp"
#include "ui/display_modes.hpp"
#include "ui/widget_tree.hpp"
#include "rendering/camera_controller.hpp"
#include "addons/lua_engine.hpp"
#include "game/bg_score_defs.hpp"
#include "game/calendar_month.hpp"
#include "game/calendar_data.hpp"
#include "game/pet_action.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "audio/music_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "core/app_clock.hpp"
#include "core/window.hpp"

#include <SDL2/SDL.h>
#include "game/expansion_profile.hpp"
#include "core/coordinates.hpp"
#include "rendering/world_map/coordinate_projection.hpp"
#include "core/local_time.hpp"

namespace wowee::addons {

// CombatLog_Object_IsA(unitFlags, mask) - does a combat log unit match a filter.
//
// The flags are four exclusive categories packed together (affiliation,
// reaction, control, unit type) plus a set of non-exclusive special bits, and a
// filter names every value it accepts within a category. COMBATLOG_FILTER_MINE
// is AFFILIATION_MINE + REACTION_FRIENDLY + CONTROL_PLAYER + TYPE_PLAYER +
// TYPE_OBJECT, and a player only ever carries one of those two type bits - so
// the obvious (flags & mask) == mask never matches anything, and the whole
// combat log filters itself empty.
//
// A category the mask says nothing about is not a constraint.
/// The month the calendar is looking at, as (month, year).
///
/// State, because it is state in WoW as well: CalendarSetMonth moves it by an
/// offset and CalendarGetMonth reads it back. Recomputing it from the clock on
/// every call would answer correctly once and make the month buttons do
/// nothing. Seeded from the local clock the first time it is asked, which is
/// the month the calendar opens on.
static std::pair<int, int>& calendarViewedMonthState() {
    static std::pair<int, int> viewed{0, 0};
    if (viewed.first == 0) {
        const std::time_t now = std::time(nullptr);
        const std::tm t = core::localTime(now);
        viewed = {t.tm_mon + 1, t.tm_year + 1900};
    }
    return viewed;
}

static std::pair<int, int> calendarViewedMonth() {
    return calendarViewedMonthState();
}

static void calendarSetViewedMonth(int month, int year) {
    if (month < 1 || month > 12) return;
    calendarViewedMonthState() = {month, year};
}

/// One end of the range the calendar offers, as weekday, month, day, year -
/// the order the interface unpacks it in.
static int pushCalendarBoundDate(lua_State* L, int monthOffset) {
    // From today's month rather than the viewed one: a bound that moved as the
    // player paged would let them page for ever.
    const std::time_t now = std::time(nullptr);
    const std::tm t = core::localTime(now);
    const int baseMonth = t.tm_mon + 1;
    const int baseYear  = t.tm_year + 1900;
    const auto info =
        wowee::game::calendarMonthAt(baseMonth, baseYear, monthOffset);
    // The first of that month at the near end, its last day at the far end, so
    // the whole bounding month is inside the range rather than half of it.
    const int day = (monthOffset < 0) ? 1 : info.numDays;
    lua_pushnumber(L, wowee::game::weekdayOf(info.month, day, info.year));
    lua_pushnumber(L, info.month);
    lua_pushnumber(L, day);
    lua_pushnumber(L, info.year);
    return 4;
}

/// The rows on the day an interface call is asking about.
///
/// Every day-indexed calendar getter takes (monthOffset, day) and indexes the
/// same list, so resolving it lives here once - three getters computing "which
/// month is offset -1 of the viewed one" separately is how they would drift
/// apart on the first month that ends on a Sunday.
static std::vector<wowee::game::CalendarDayEntry> calendarDayRows(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return {};
    const int monthOffset = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int day = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (day < 1) return {};
    const auto viewed = calendarViewedMonth();
    const auto info = wowee::game::calendarMonthAt(viewed.first, viewed.second,
                                                   monthOffset);
    if (day > info.numDays) return {};
    return wowee::game::calendarEntriesForDay(gh->getCalendarData(), info.month,
                                              day, info.year);
}

/// The event being built up before it is sent.
///
/// State, because the interface builds one over several calls and commits it
/// with another: CalendarNewEvent starts it, CalendarEventSetTitle and the
/// rest fill it in, CalendarAddEvent sends it. Nothing is carried between
/// those calls except this.
static wowee::game::CalendarEventDraft& calendarDraft() {
    static wowee::game::CalendarEventDraft draft;
    return draft;
}

/// The row a right-click menu is about: which month, which day, which entry.
///
/// State because the menu needs it to be. UIDropDownMenu builds its list in
/// one call and runs the click in another, and nothing is carried between the
/// two - so the row is named once by CalendarContextSelectEvent and every verb
/// on the menu reads it back.
struct CalendarContextRow { int monthOffset = 0; int day = 0; int index = 0; };

static CalendarContextRow& calendarContextRow() {
    static CalendarContextRow row;
    return row;
}

/// The rows of a day, for a caller that already has the handler and the day.
static std::vector<wowee::game::CalendarDayEntry> calendarRowsFor(
        wowee::game::GameHandler& gh, int monthOffset, int day) {
    if (day < 1) return {};
    const auto viewed = calendarViewedMonth();
    const auto info = wowee::game::calendarMonthAt(viewed.first, viewed.second,
                                                   monthOffset);
    if (day > info.numDays) return {};
    return wowee::game::calendarEntriesForDay(gh.getCalendarData(), info.month,
                                              day, info.year);
}

/// Which of the six kinds of row this event is, as the string the interface
/// compares against. Guild events carry a flag; everything else the player can
/// see on their own calendar is theirs.
static const char* calendarTypeName(const wowee::game::CalendarEvent& ev) {
    // CALENDAR_EVENTTYPE_* from the server: 4 is the guild announcement, and
    // the guild flag distinguishes an event from an announcement.
    constexpr uint32_t kGuildEventFlag = 0x0400;
    if (ev.type == 4) return "GUILD_ANNOUNCEMENT";
    if (ev.flags & kGuildEventFlag) return "GUILD_EVENT";
    return "PLAYER";
}

/// "CREATOR" when this is the player's own event, otherwise nothing. The
/// interface shows the edit controls on the first of those.
static const char* calendarModStatus(lua_State* L,
                                     const wowee::game::CalendarEvent& ev) {
    auto* gh = getGameHandler(L);
    return (gh && ev.creatorGuid != 0 && ev.creatorGuid == gh->getPlayerGuid())
               ? "CREATOR" : "";
}

/// Answer the invitation on the menu's row, if that row has one.
///
/// The invite id comes from the invite list the calendar arrived with - the
/// server wants both it and the event id, and an event the player was never
/// invited to has no invite to answer. Silent when there is none rather than
/// sending a request the server would refuse.
static int calendarRespondToContextInvite(lua_State* L, uint32_t status) {
    auto* gh = getGameHandler(L);
    const auto& row = calendarContextRow();
    if (!gh || row.day < 1) return 0;
    const auto rows = calendarRowsFor(*gh, row.monthOffset, row.day);
    if (row.index < 1 || static_cast<size_t>(row.index) > rows.size()) return 0;
    const auto& entry = rows[static_cast<size_t>(row.index) - 1];
    if (entry.kind != wowee::game::CalendarEntryKind::Event) return 0;
    const auto& cal = gh->getCalendarData();
    const uint64_t eventId = cal.events[entry.index].eventId;
    for (const auto& invite : cal.invites) {
        if (invite.eventId != eventId) continue;
        gh->respondToCalendarInvite(eventId, invite.inviteId, status);
        return 0;
    }
    return 0;
}

/// The player's own answer to an event, from the invite list that came with
/// it. Zero is CALENDAR_INVITESTATUS_INVITED, which is what an event carries
/// while it is unanswered.
static double calendarInviteStatusFor(const wowee::game::CalendarData& cal,
                                      uint64_t eventId) {
    for (const auto& invite : cal.invites) {
        if (invite.eventId == eventId) return static_cast<double>(invite.status);
    }
    return 0;
}

static int lua_CombatLog_Object_IsA(lua_State* L) {
    const auto flags = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
    const auto mask  = static_cast<uint32_t>(luaL_optnumber(L, 2, 0));

    static constexpr uint32_t kCategories[] = {
        0x0000000Fu,  // affiliation: mine / party / raid / outsider
        0x000000F0u,  // reaction: friendly / neutral / hostile
        0x00000300u,  // control: player / npc
        0x0000FC00u,  // type: player / npc / pet / guardian / object
    };
    for (const uint32_t cat : kCategories) {
        const uint32_t wanted = mask & cat;
        if (wanted == 0) continue;            // unconstrained
        if ((flags & wanted) == 0) { lua_pushboolean(L, 0); return 1; }
    }
    // The special bits are non-exclusive, so every one asked for must be present.
    const uint32_t special = mask & 0xFFFF0000u;
    if (special != 0 && (flags & special) != special) { lua_pushboolean(L, 0); return 1; }

    lua_pushboolean(L, 1);
    return 1;
}

static int lua_PlaySound(lua_State* L) {
    // Silent while the interface is building itself, before anything else -
    // including the log below, so a line here means a sound that was played.
    // See LuaEngine::setUiSoundsSuppressed and AddonManager::loadAllAddons.
    if (LuaEngine::uiSoundsSuppressed()) return 0;

    // Which sound the interface asked for, and who asked.
    //
    // A sound reported as playing loudly on every world entry turned out to be
    // igMainMenuOpen - uEscapeScreenOpen.wav - played seven times inside
    // thirty milliseconds, which stacks into one hit. Seven callers or one
    // caller seven times is the question a byte count cannot answer, and there
    // are seven separate PlaySound("igMainMenuOpen") sites in the interface.
    // The traceback names it in one line.
    if (core::Logger::getInstance().shouldLog(core::LogLevel::INFO)) {
        const char* asked = lua_isstring(L, 1) ? lua_tostring(L, 1) : "<id>";
        // From the C side rather than through debug.traceback: this Lua is
        // sandboxed and has no debug table, so the traceback came back empty
        // and named nothing. lua_getstack walks the same frames without it.
        std::string where;
        lua_Debug ar;
        for (int level = 1; level <= 3 && lua_getstack(L, level, &ar); ++level) {
            if (!lua_getinfo(L, "Sl", &ar)) break;
            if (!where.empty()) where += " <- ";
            where += ar.short_src;
            where += ":" + std::to_string(ar.currentline);
        }
        LOG_INFO("PlaySound: ", asked, " <- ", where);
    }

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
        // The real sound first. These names are SoundEntries.dbc rows - the
        // interface is naming one, and the row names the files - so looking it
        // up plays what WoW plays rather than the nearest thing this client had
        // already loaded, and it covers all sixty-eight rather than the
        // forty-four below.
        //
        // The table below stays as the fallback: an install without the sound,
        // or without the dbc, still gets something for the names it can be
        // approximated for.
        if (sfx->playByName(name)) return 0;

        sound = name;
        for (char& c : sound) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

        // FrameXML asks for eighty-nine names across the files that load -
        // sixty-eight when this was written - and nine of them were answered,
        // so nearly every panel this branch has handed over opened, closed and
        // was clicked in silence. The sound manager already had most of these
        // under its own names; only the mapping was missing.
        //
        // Mapped where the client has the sound the name asks for, and left
        // alone where it does not. Substituting something that merely exists
        // is worse than silence: a wrong sound is a wrong sound forever, while
        // a missing one is a gap someone can still hear.
        struct Mapping { const char* name; void (audio::UiSoundManager::*play)(); };
        static const Mapping kMappings[] = {
            // A click is a click, and these are all buttons.
            {"IGMAINMENUOPTION",            &audio::UiSoundManager::playButtonClick},
            {"IGMAINMENUOPTIONCHECKBOXON",  &audio::UiSoundManager::playButtonClick},
            {"IGMAINMENUOPTIONCHECKBOXOFF", &audio::UiSoundManager::playButtonClick},
            {"IGPLAYERINVITEACCEPTED",      &audio::UiSoundManager::playButtonClick},
            {"IGCHARACTERINFOTAB",          &audio::UiSoundManager::playButtonClick},
            {"UCHATSCROLLBUTTON",           &audio::UiSoundManager::playButtonClick},
            {"IGCHATSCROLLUP",              &audio::UiSoundManager::playButtonClick},
            {"IGCHATSCROLLDOWN",            &audio::UiSoundManager::playButtonClick},
            {"IGCHATBOTTOM",                &audio::UiSoundManager::playButtonClick},
            {"IGMAINMENUOPEN",              &audio::UiSoundManager::playMenuButtonClick},
            {"IGMAINMENUCLOSE",             &audio::UiSoundManager::playMenuButtonClick},
            {"IGMAINMENUCONTINUE",          &audio::UiSoundManager::playMenuButtonClick},
            {"GSTITLEOPTIONOK",             &audio::UiSoundManager::playButtonClick},
            {"GSTITLEOPTIONEXIT",           &audio::UiSoundManager::playButtonClick},
            // The panels, each of which has its own pair here already.
            {"IGCHARACTERINFOOPEN",         &audio::UiSoundManager::playCharacterSheetOpen},
            {"IGCHARACTERINFOCLOSE",        &audio::UiSoundManager::playCharacterSheetClose},
            {"TALENTSCREENOPEN",            &audio::UiSoundManager::playCharacterSheetOpen},
            {"TALENTSCREENCLOSE",           &audio::UiSoundManager::playCharacterSheetClose},
            // The guild vault, which was not silent: GuildVaultOpen and
            // GuildVaultClose are rows in SoundEntries.dbc, so playByName above
            // has been finding and playing them all along.
            //
            // What was missing is the fallback this table is for. Its two
            // samples were preloaded from a fixed path and had no method to
            // reach them, so an install with the wavs but no SoundEntries.dbc -
            // the case the table exists to cover - had them in memory and no
            // way to ask for them.
            {"GUILDVAULTOPEN",              &audio::UiSoundManager::playGuildBankOpen},
            {"GUILDVAULTCLOSE",             &audio::UiSoundManager::playGuildBankClose},
            {"IGBACKPACKOPEN",              &audio::UiSoundManager::playBagOpen},
            {"IGBACKPACKCLOSE",             &audio::UiSoundManager::playBagClose},
            {"KEYRINGOPEN",                 &audio::UiSoundManager::playBagOpen},
            {"KEYRINGCLOSE",                &audio::UiSoundManager::playBagClose},
            {"IGQUESTLOGOPEN",              &audio::UiSoundManager::playQuestLogOpen},
            {"IGQUESTLOGCLOSE",             &audio::UiSoundManager::playQuestLogClose},
            {"IGQUESTLISTOPEN",             &audio::UiSoundManager::playQuestActivate},
            {"IGQUESTLISTCOMPLETE",         &audio::UiSoundManager::playQuestComplete},
            {"IGQUESTLOGABANDONQUEST",      &audio::UiSoundManager::playQuestFailed},
            {"IGQUESTCANCEL",               &audio::UiSoundManager::playQuestFailed},
            {"WRITEQUEST",                  &audio::UiSoundManager::playQuestUpdate},
            {"IGSPELLBOOKOPEN",             &audio::UiSoundManager::playPickupBook},
            {"IGSPELLBOOKCLOSE",            &audio::UiSoundManager::playPickupBook},
            {"IGABILITYOPEN",               &audio::UiSoundManager::playPickupBook},
            {"IGABILITYCLOSE",              &audio::UiSoundManager::playPickupBook},
            {"IGABILIITYPAGETURN",          &audio::UiSoundManager::playPickupBook},
            {"AUCTIONWINDOWOPEN",           &audio::UiSoundManager::playAuctionHouseOpen},
            {"AUCTIONWINDOWCLOSE",          &audio::UiSoundManager::playAuctionHouseClose},
            // The rest, each with an exact counterpart.
            {"LEVELUPSOUND",                &audio::UiSoundManager::playLevelUp},
            {"MAPPING",                     &audio::UiSoundManager::playMinimapPing},
            {"TELLMESSAGE",                 &audio::UiSoundManager::playWhisperReceived},
            {"IGCHARACTERNPCSELECT",        &audio::UiSoundManager::playTargetSelect},
            {"IGCREATUREAGGROSELECT",       &audio::UiSoundManager::playTargetSelect},
            {"IGCREATURENEUTRALSELECT",     &audio::UiSoundManager::playTargetSelect},
            {"INTERFACESOUND_LOSTTARGETUNIT", &audio::UiSoundManager::playTargetDeselect},
            {"IGBACKPACKCOINSELECT",        &audio::UiSoundManager::playLootCoinSmall},
            {"IGBACKPACKCOINOK",            &audio::UiSoundManager::playLootCoinSmall},
            {"LOOTWINDOWOPENEMPTY",         &audio::UiSoundManager::playError},
            {"LFG_DENIED",                  &audio::UiSoundManager::playError},
            {"LFG_REWARDS",                 &audio::UiSoundManager::playQuestComplete},
        };
        bool mapped = false;
        for (const Mapping& m : kMappings) {
            if (sound == m.name) { (sfx->*m.play)(); mapped = true; break; }
        }
        // Said once per name, because a sound that resolves to nothing is the
        // one gap in this file that leaves no trace at all: the row is missing
        // from the dbc and the name is not in the table above, so the call
        // returns having done nothing and the panel is silent.
        //
        // Silence is the right answer - substituting a sound that merely
        // exists would be wrong forever, where a missing one can still be
        // heard - but it should be possible to find out which name it was
        // without guessing from a quiet button. The interface asks for eighty
        // nine of these across the files that load, and the table below covers
        // the ones the client has an equivalent for.
        if (!mapped) {
            static std::set<std::string> saidAlready;
            if (saidAlready.insert(sound).second) {
                LOG_WARNING("PlaySound: '", name, "' names no row this install "
                            "has and nothing here stands in for it, so it is "
                            "silent");
            }
        }
    }
    return 0;
}

// PlaySoundFile(path) - stub (file-based sounds not loaded from Lua)
/// PlaySoundFile(path) - a sound named by where it lives rather than by a
/// SoundEntries row.
///
/// One caller in the interface, on the login screen, and a great many in
/// addons: it is how an addon ships a sound of its own. Reads and caches the
/// same way PlaySound does, because a path and a name are both just keys.
static int lua_PlaySoundFile(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* ac = svc ? svc->audioCoordinator : nullptr;
    auto* sfx = ac ? ac->getUiSoundManager() : nullptr;
    const char* path = luaL_optstring(L, 1, "");
    if (sfx && path && *path) sfx->playFile(path);
    return 0;
}

/// GetPlayerMapPosition(unit) → where that unit is on the map now showing,
/// as a fraction across it.
///
/// Two faults, both silent. It answered raw world coordinates, and
/// worldmapframe.lua multiplies what it gets by WorldMapDetailFrame's width -
/// so the player arrow was placed some thousands of pixels off the parchment
/// rather than on it. And it ignored the unit it was asked about, so the four
/// party markers and the forty raid ones all described the player.
///
/// Zero and zero is the answer for "not on this map": every caller tests
/// `if ( x == 0 and y == 0 )` and hides its marker, which is what should happen
/// for someone in another zone.
static int lua_GetPlayerMapPosition(lua_State* L) {
    auto* gh = getGameHandler(L);
    auto* svc = getLuaServices(L);
    const char* uid = luaL_optstring(L, 1, "player");
    if (!gh || !svc || !svc->mapUVForWorldPos || !uid) {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        return 2;
    }
    float wx = 0.0f, wy = 0.0f, wz = 0.0f;
    std::string unit(uid);
    toLowerInPlace(unit);
    if (unit == "player") {
        const auto& mi = gh->getMovementInfo();
        wx = mi.x; wy = mi.y; wz = mi.z;
    } else if (const uint64_t guid = resolveUnitGuid(gh, unit)) {
        // Through the entity when it is in range, and the group list when it is
        // not - a raid member across the zone is exactly who this is asked
        // about, and they have no entity here.
        if (auto e = gh->getEntityManager().getEntity(guid)) {
            wx = e->getX(); wy = e->getY(); wz = e->getZ();
        } else if (const auto* m = findPartyMember(gh, guid)) {
            // The group list carries a coarse position - SMSG_PARTY_MEMBER_STATS
            // truncates it to a pair of int16 yards - which is ample for a dot
            // on a zone map and is the only position there is for someone out
            // of range.
            wx = static_cast<float>(m->posX);
            wy = static_cast<float>(m->posY);
        }
    }
    float u = 0.0f, v = 0.0f;
    if ((wx == 0.0f && wy == 0.0f) || !svc->mapUVForWorldPos(wx, wy, wz, u, v)) {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        return 2;
    }
    lua_pushnumber(L, u);
    lua_pushnumber(L, v);
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
/// CVars the player or the interface has actually set, which win over the
/// defaults below.
///
/// SetCVar was a no-op, so every option the interface changed reverted the
/// instant it was read back: ticking a box in the interface options did
/// nothing, and any code that writes a CVar and then reads it to confirm - of
/// which FrameXML has a fair amount - saw its own write disappear.
static std::unordered_map<std::string, std::string>& cvarStore() {
    static std::unordered_map<std::string, std::string> store;
    return store;
}

/// Where the CVars live between runs.
///
/// Its own file rather than settings.cfg, which this client's own panel writes
/// whole from its own fields: a CVar written into that file would be dropped
/// the next time that panel saved.
static std::string cvarStorePath() {
    return core::getConfigRoot() + "/cvars.cfg";
}

/// Read the stored CVars back. Called once, before the interface loads,
/// because a panel reads its checkbox out of the CVar as it is built and
/// anything arriving later leaves the box disagreeing with the setting.
///
/// Storing them in memory alone made every interface option last exactly one
/// session. That is not a small thing: the equipment manager is off until its
/// box is ticked, the box writes equipmentManager, and the paperdoll reads that
/// on VARIABLES_LOADED - so it came back off on every login, and so did every
/// other option the player had set.
static void loadStoredCVars() {
    std::ifstream in(cvarStorePath());
    if (!in.is_open()) return;
    std::string line;
    size_t loaded = 0;
    while (std::getline(in, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos || eq == 0) continue;
        std::string key = line.substr(0, eq);
        toLowerInPlace(key);
        cvarStore()[key] = line.substr(eq + 1);
        ++loaded;
    }
    LOG_INFO("CVars: loaded ", loaded, " from ", cvarStorePath());
}

/// Write them back. Called on every change rather than at shutdown: the file is
/// a few hundred bytes, and a setting that survives only a clean exit is not
/// one a player can rely on.
static void saveStoredCVars() {
    const std::string path = cvarStorePath();
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);
    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save CVars to ", path);
        return;
    }
    // Sorted, so a diff of the file reads as a change of settings rather than
    // as the hash order moving underneath it.
    std::vector<const std::pair<const std::string, std::string>*> rows;
    rows.reserve(cvarStore().size());
    for (const auto& kv : cvarStore()) rows.push_back(&kv);
    std::sort(rows.begin(), rows.end(),
              [](const auto* a, const auto* b) { return a->first < b->first; });
    for (const auto* kv : rows) {
        // A value with a newline in it would come back as two broken lines.
        if (kv->second.find('\n') != std::string::npos) continue;
        out << kv->first << '=' << kv->second << '\n';
    }
}

/// Tutorials the player has already been shown.
///
/// The stock client keeps these in account data, which this client is not
/// sent, so they ride in the CVar file beside everything else that has to
/// survive a login - one row, the ids comma separated. An in-memory set would
/// look finished and re-show every tutorial on the next login, which is the
/// shape this repository keeps producing.
///
/// TutorialFrame_NewTutorial refuses to queue a tutorial that is already
/// flagged, so an answer of "no" here means every tutorial fires again on
/// every login, and an unanswered call raises inside TutorialFrame_Update.
static const char* kTutorialCVar = "wowee_tutorialsFlagged";

static bool tutorialFlagged(int id) {
    const std::string& all = cvarStore()[kTutorialCVar];
    const std::string needle = std::to_string(id);
    size_t at = 0;
    while ((at = all.find(needle, at)) != std::string::npos) {
        const bool startOk = (at == 0) || all[at - 1] == ',';
        const size_t end = at + needle.size();
        const bool endOk = (end == all.size()) || all[end] == ',';
        if (startOk && endOk) return true;
        at = end;
    }
    return false;
}

/// Every flagged tutorial, in id order.
///
/// Stored in the order they were seen, which is not the order they are browsed
/// in: the next and previous buttons walk the ids, so the list is sorted here
/// rather than kept sorted, and a file hand-edited into any order still reads.
static std::vector<int> tutorialIds() {
    std::vector<int> ids;
    const std::string& all = cvarStore()[kTutorialCVar];
    size_t at = 0;
    while (at < all.size()) {
        const size_t comma = all.find(',', at);
        const std::string piece = all.substr(
            at, comma == std::string::npos ? std::string::npos : comma - at);
        if (!piece.empty()) {
            try { ids.push_back(std::stoi(piece)); } catch (...) {}
        }
        if (comma == std::string::npos) break;
        at = comma + 1;
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

static void flagTutorial(int id) {
    if (id <= 0 || tutorialFlagged(id)) return;
    std::string& all = cvarStore()[kTutorialCVar];
    if (!all.empty()) all += ',';
    all += std::to_string(id);
    saveStoredCVars();
}

/// A sound CVar's value, or the stock client's default for it.
///
/// The defaults matter as much as the store does. Sound_MasterVolumeUp reads
/// the CVar, runs it through tonumber and adds a step - with nothing stored and
/// no default it reads nil, the `if (volume)` guard below it fails, and the
/// volume keys do nothing at all rather than anything visible.
static float soundCVar(const char* key, float fallback) {
    if (auto it = cvarStore().find(key); it != cvarStore().end()) {
        try {
            return std::stof(it->second);
        } catch (const std::exception&) {
            return fallback;
        }
    }
    return fallback;
}

/// Push the sound CVars at the audio system, which is what makes them settings
/// rather than a record of what was clicked.
///
/// Every channel is recomputed from the store rather than from the one CVar
/// that changed, so enable and volume compose and the order they arrive in does
/// not matter - Sound_ToggleSound writes EnableSFX and EnableAmbience one after
/// the other, and the interface's options panel writes a volume and an enable
/// together on apply.
///
/// This client splits sound finer than the interface does: it has separate
/// volumes for combat, spells, movement, footsteps and the rest, where FrameXML
/// has one "sound effects". Turning SFX off and on again therefore levels those
/// channels rather than restoring them. That is the retail behaviour - there is
/// no finer control there to restore - and the client's own settings panel
/// re-applies its sliders whenever it is used, so neither owner is stuck with
/// the other's answer.
static void applySoundCVars(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* ac = svc ? svc->audioCoordinator : nullptr;
    if (!ac) return;

    // The sliders first, then the switches on top. Each switch is applied by
    // zeroing a channel and zeroing has no inverse, so without this the "on"
    // case wrote nothing at all: turning sound effects off and on again left
    // all nine effect channels silent until the client's own settings window
    // was touched. Recomputing from the store is what the note above says this
    // does, and now does.
    if (svc->reapplyAudioVolumes) svc->reapplyAudioVolumes();

    // Master, music and ambience are the client's own settings now, reached
    // through SetCVar above - this function must not write them too, or the two
    // fight and the last writer wins.
    //
    // What is left here are the three enable switches, which the client's panel
    // has no equivalent for. They multiply whatever the settings applied.
    const bool musicOn = soundCVar("sound_enablemusic", 1.0f) != 0.0f;
    if (!musicOn) {
        if (auto* music = ac->getMusicManager()) music->setVolume(0);
    }

    const bool ambOn = soundCVar("sound_enableambience", 1.0f) != 0.0f;
    if (!ambOn) {
        if (auto* ambient = ac->getAmbientSoundManager()) {
            ambient->setVolumeScale(0.0f);
            ambient->setBellVolumeScale(0.0f);
        }
    }

    // The character's own voice lines for a refused action - "I can't do that
    // yet", "Not enough rage". PlayerVoiceManager has carried an enabled_ flag
    // all along and playError honours it; nothing ever set it, so the switch in
    // the Sound panel was a checkbox that moved and changed nothing.
    //
    // Not folded into the sfx switch below: this is the one people actually
    // reach for, and turning it off should leave every other effect alone.
    if (auto* voice = ac->getPlayerVoiceManager()) {
        voice->setEnabled(soundCVar("sound_enableerrorspeech", 1.0f) != 0.0f);
    }

    // The volume itself is a client setting now, reached through SetCVar, and
    // applied by applyAudioVolumes as one scale over the seven effect volumes
    // this client keeps separately. Only the switch is left here, and it zeroes
    // them on top of whatever that applied.
    const bool sfxOn = soundCVar("sound_enablesfx", 1.0f) != 0.0f;
    if (!sfxOn) {
        if (auto* m = ac->getUiSoundManager())        m->setVolumeScale(0.0f);
        if (auto* m = ac->getCombatSoundManager())    m->setVolumeScale(0.0f);
        if (auto* m = ac->getSpellSoundManager())     m->setVolumeScale(0.0f);
        if (auto* m = ac->getMovementSoundManager())  m->setVolumeScale(0.0f);
        if (auto* m = ac->getFootstepManager())       m->setVolumeScale(0.0f);
        if (auto* m = ac->getActivitySoundManager())  m->setVolumeScale(0.0f);
        if (auto* m = ac->getMountSoundManager())     m->setVolumeScale(0.0f);
        if (auto* m = ac->getNpcVoiceManager())       m->setVolumeScale(0.0f);
        if (auto* m = ac->getPlayerVoiceManager())    m->setVolumeScale(0.0f);
    }
}


/// The value a CVar has when nobody has set it.
///
/// Split out so GetCVarDefault can answer it. That was aliased straight to
/// GetCVar, which answers the *current* value - and every options panel
/// captures control.defaultValue from it as the panel loads, so the Defaults
/// button restored each control to whatever it had been when the panel was
/// opened. Nothing looked broken: the button worked, it just always agreed
/// with wherever the controls already were.

static void pushCvarDefault(lua_State* L, const std::string& n) {
    // Return sensible defaults for commonly queried CVars
    // The sound ones read back as on and at full, which is what this client
    // starts as. Volume up/down step from whatever is read here, so answering
    // nothing leaves those keys inert rather than merely at a default.
    // Off, as the real client has it: alt-tabbing away silences the game.
    // Ahead of the sound_enable prefix below, which would otherwise answer it
    // "1" - that rule swallows every name it starts with, so a default of any
    // other value has to be stated before it rather than after.
    if (n == "sound_enablesoundwhengameisinbg") lua_pushstring(L, "0");
    else if (n.rfind("sound_enable", 0) == 0) lua_pushstring(L, "1");
    else if (n == "sound_mastervolume" || n == "sound_musicvolume" ||
             n == "sound_ambiencevolume") {
        lua_pushstring(L, "1");
    }
    // Windowed, as one - the way round the checkbox is labelled. Answered from
    // the window only as a *default*, below the store, unlike the settings this
    // client owns outright: gxWindow is applied by RestartGx after the panel
    // has written it, so between the tick and the Okay the stored value is the
    // truth and the window is still the old state. Asking the window first
    // would have made RestartGx read back what it was about to set.
    else if (n == "gxwindow") {
        auto* svc = getLuaServices(L);
        const bool full = (svc && svc->getFullscreen) ? svc->getFullscreen() : false;
        lua_pushstring(L, full ? "0" : "1");
    }
    // Both at their maximum, because this client has no lower setting to be
    // at. miniaudio mixes every voice it is given at the device's own rate:
    // there is no channel cap to raise and no quality tier to pick. Falling
    // through to the "0" at the end of this chain parked both sliders at the
    // far left of the Sound panel - 32 channels and Low quality - which read
    // as a client running at its worst and was not a setting at all.
    //
    // The controls are taken off the panels; see kRemovedControlsLua.
    else if (n == "sound_numchannels") lua_pushstring(L, "64");
    else if (n == "sound_outputquality") lua_pushstring(L, "2");
    else if (n == "uiscale") lua_pushstring(L, "1");
    else if (n == "useuiscale") lua_pushstring(L, "1");
    else if (n == "screenwidth" || n == "gxresolution") {
        auto* svc = getLuaServices(L);
        auto* win = svc ? svc->window : nullptr;
        lua_pushstring(L, std::to_string(win ? win->getWidth() : 1920).c_str());
    } else if (n == "screenheight" || n == "gxfullscreenresolution") {
        auto* svc = getLuaServices(L);
        auto* win = svc ? svc->window : nullptr;
        lua_pushstring(L, std::to_string(win ? win->getHeight() : 1080).c_str());
    } else if (n == "nameplateshowfriends") lua_pushstring(L, "1");
    else if (n == "nameplateshowenemies") lua_pushstring(L, "1");
    else if (n == "sound_enablesfx") lua_pushstring(L, "1");
    else if (n == "sound_enableerrorspeech") lua_pushstring(L, "1");
    // One, not zero: the slider is a multiple of the original client's limit
    // and zero is not a position on it. Answering zero pinned it at minimum.
    else if (n == "cameradistancemaxfactor") lua_pushstring(L, "1");
    else if (n == "weatherdensity") lua_pushstring(L, "3");
    else if (n == "particledensity") lua_pushstring(L, "1");
    else if (n == "environmentdetail") lua_pushstring(L, "1");
    // The top of the slider: this client has always drawn at 16x, so anything
    // less would be a quality setting the player never asked for.
    else if (n == "texturefilteringmode") lua_pushstring(L, "5");
    else if (n == "groundeffectdist") lua_pushstring(L, "140");
    // Level 3 is 4096, which is the shadow map this client drew before the
    // setting existed. Answering the size it has always used keeps a player who
    // never touches this from being quietly downgraded by it appearing.
    else if (n == "extshadowquality") lua_pushstring(L, "3");
    // Clicking open ground clears the target, which is the real client's
    // behaviour and this one's. Without saying so it fell to the generic zero,
    // and zero here means sticky targeting - so the checkbox would have shown
    // itself ticked while the client went on clearing, the panel and the game
    // disagreeing about the same switch.
    else if (n == "deselectonclick") lua_pushstring(L, "1");
    // On, which is what this client has always done and what the real one
    // defaults to. Unset it fell to zero, so the checkbox would have shown the
    // numbers switched off while they were drawn.
    else if (n == "enablecombattext") lua_pushstring(L, "1");
    // The three kinds this client draws. All on, which is what it did before
    // any of them were read; unset they fell to zero, which would have shown
    // every one of these boxes unticked while the numbers were on screen.
    else if (n == "fctdamage" || n == "fcthealing" ||
             n == "fctdodgeparrymiss") lua_pushstring(L, "1");
    // The four kinds of unit this client can tell apart on a nameplate. On,
    // which is what it drew before any of them were read.
    else if (n == "unitnameenemyplayername" || n == "unitnamefriendlyplayername" ||
             n == "unitnamenpc" || n == "unitnamenoncombatcreaturename")
        lua_pushstring(L, "1");
    // Off, as the real client has it: bars start unlocked and a player who
    // wants them held down says so.
    else if (n == "lockactionbars") lua_pushstring(L, "0");
    // On, which is what the tooltip has always drawn. Unset it fell to zero,
    // and that would have taken the line away the moment anything read it.
    else if (n == "showitemlevel") lua_pushstring(L, "1");
    // Off: trades arrive as they always have unless the player says otherwise.
    else if (n == "blocktrades") lua_pushstring(L, "0");
    // On, which is what the aura icons have always drawn.
    else if (n == "buffdurations") lua_pushstring(L, "1");
    // Off, which is the behaviour there has been: changing target leaves the
    // swing running and it follows to whoever is selected next.
    else if (n == "stopautoattackontargetchange") lua_pushstring(L, "0");
    // On, which is what has always been printed.
    else if (n == "showlootspam") lua_pushstring(L, "1");
    // On, as the real client has it: coming back and talking takes the flag
    // off rather than leaving it for the player to notice.
    else if (n == "autoclearafk") lua_pushstring(L, "1");
    // On, which is what has always been announced.
    else if (n == "guildmembernotify") lua_pushstring(L, "1");
    // Off, as the real client has it: attacking an ally does nothing unless
    // the player has asked for it to mean assist.
    else if (n == "assistattack") lua_pushstring(L, "0");
    // Off, as the real client has it: the threat indicator is drawn either
    // way, and the noise is opt-in.
    else if (n == "threatplaysounds") lua_pushstring(L, "0");
    // Two that were relying on the generic zero rather than saying so. Both
    // want off, so the behaviour was right - but by coincidence, and the rule
    // this file keeps proving is that a value nobody registers is a value
    // nobody has checked. Stated, they agree by construction.
    else if (n == "autodismountflying") lua_pushstring(L, "0");
    else if (n == "colorblindmode") lua_pushstring(L, "0");
    // Off, as the real client has it: a second press stops the swing unless
    // the player has asked to be protected from that.
    else if (n == "secureabilitytoggle") lua_pushstring(L, "0");
    // Quest titles on the world map colour by difficulty out of the box, and a
    // quest whose progress changes starts being watched - both on in the real
    // client. The other two are opt-in there and stay opt-in here.
    else if (n == "mapquestdifficulty") lua_pushstring(L, "1");
    else if (n == "autoquestprogress") lua_pushstring(L, "1");
    else if (n == "consolidatebuffs") lua_pushstring(L, "0");
    else if (n == "watchframewidth") lua_pushstring(L, "0");
    // Nameplates and names for totems. The real client shows an enemy's totems
    // and hides your own side's, and names both when they are shown.
    else if (n == "nameplateshowenemytotems") lua_pushstring(L, "1");
    else if (n == "nameplateshowfriendlytotems") lua_pushstring(L, "0");
    else if (n == "unitnameenemytotemname") lua_pushstring(L, "1");
    else if (n == "unitnamefriendlytotemname") lua_pushstring(L, "1");
    // Reaction, not class, is what a world-space bar is for here; the setting
    // is offered and honoured, but green stays the default.
    else if (n == "showclasscolorinnameplate") lua_pushstring(L, "0");
    // The Combat Text panel's own filters. Effects on units other than your
    // target are off in the real client; everything else here is on.
    else if (n == "combatdamage") lua_pushstring(L, "1");
    else if (n == "combathealing") lua_pushstring(L, "1");
    else if (n == "combatlogperiodicspells") lua_pushstring(L, "1");
    else if (n == "petmeleedamage") lua_pushstring(L, "1");
    else if (n == "fctspellmechanics") lua_pushstring(L, "1");
    else if (n == "fctspellmechanicsother") lua_pushstring(L, "0");
    // The camera does not keep lerping through a turn unless asked, which is
    // this client's own default, and the smoothing rate it starts at.
    else if (n == "camerasmoothstyle") lua_pushstring(L, "0");
    else if (n == "camerayawsmoothspeed") lua_pushstring(L, "30");
    // Titles on player names, as the real client shows them.
    else if (n == "unitnameplayerpvptitle") lua_pushstring(L, "1");
    // Cast bars over enemy nameplates are on; party lines float only if asked.
    else if (n == "showvkeycastbar") lua_pushstring(L, "1");
    else if (n == "chatbubblesparty") lua_pushstring(L, "0");
    // Pets and guardians, plated and named on both sides, as they ship.
    else if (n == "nameplateshowenemypets") lua_pushstring(L, "1");
    else if (n == "nameplateshowfriendlypets") lua_pushstring(L, "1");
    else if (n == "nameplateshowenemyguardians") lua_pushstring(L, "1");
    else if (n == "nameplateshowfriendlyguardians") lua_pushstring(L, "1");
    else if (n == "unitnameenemypetname") lua_pushstring(L, "1");
    else if (n == "unitnamefriendlypetname") lua_pushstring(L, "1");
    else if (n == "unitnameenemyguardianname") lua_pushstring(L, "1");
    else if (n == "unitnamefriendlyguardianname") lua_pushstring(L, "1");
    // Guild names over players are shown; your own name over your own head is
    // not, both as the real client has them.
    else if (n == "unitnameplayerguild") lua_pushstring(L, "1");
    else if (n == "unitnameown") lua_pushstring(L, "0");
    // Plates are kept apart unless overlapping is asked for, as they ship.
    else if (n == "nameplateallowoverlap") lua_pushstring(L, "0");
    // Spam filtering is on (the checkbox in front of it reads "Disable Spam
    // Filter"), and mature language filtering is off, as they ship.
    else if (n == "spamfilter") lua_pushstring(L, "1");
    else if (n == "profanityfilter") lua_pushstring(L, "0");
    // Off, as it ships: a zone track stops at its end and the next one starts
    // when the zone asks for it.
    else if (n == "sound_zonemusicnodelay") lua_pushstring(L, "0");
    // The mouse speed this client starts at.
    else if (n == "camerayawmovespeed") lua_pushstring(L, "0.2");
    // Not joined unless asked for, as it ships.
    else if (n == "guildrecruitmentchannel") lua_pushstring(L, "0");

    // The uvarInfo table's own defaults, for the CVars it names that nothing
    // here answered. An unanswered CVar is not "off": GetCVar gives back
    // nothing, which is neither "0" nor "1", so InterfaceOptionsFrame_LoadUVars
    // compares it against the default, finds them unequal, and takes the arm
    // meant for a value the player has changed - on the very first login,
    // before anyone has touched the panel.
    //
    // Every value here is copied from that table rather than chosen. It is the
    // interface's own statement of what each setting starts as, and a default
    // invented next to it would be a second answer to a question already
    // answered a few lines away in the file this reads.
    else if (n == "alwaysshowactionbars") lua_pushstring(L, "0");
    else if (n == "autoquestwatch") lua_pushstring(L, "1");
    else if (n == "combattextfloatmode") lua_pushstring(L, "1");
    else if (n == "displayworldpvpobjectives") lua_pushstring(L, "2");
    else if (n == "fctauras") lua_pushstring(L, "0");
    else if (n == "fctcombatstate") lua_pushstring(L, "0");
    else if (n == "fctcombopoints") lua_pushstring(L, "0");
    else if (n == "fctdamagereduction") lua_pushstring(L, "0");
    else if (n == "fctenergygains") lua_pushstring(L, "0");
    else if (n == "fctfriendlyhealers") lua_pushstring(L, "0");
    else if (n == "fcthonorgains") lua_pushstring(L, "0");
    else if (n == "fctlowmanahealth") lua_pushstring(L, "1");
    else if (n == "fctperiodicenergygains") lua_pushstring(L, "0");
    else if (n == "fctreactives") lua_pushstring(L, "0");
    else if (n == "fctrepchanges") lua_pushstring(L, "0");
    else if (n == "hidepartyinraid") lua_pushstring(L, "0");
    else if (n == "lootundermouse") lua_pushstring(L, "0");
    else if (n == "questfadingdisable") lua_pushstring(L, "0");
    else if (n == "removechatdelay") lua_pushstring(L, "0");
    else if (n == "showarenaenemycastbar") lua_pushstring(L, "1");
    else if (n == "showarenaenemypets") lua_pushstring(L, "1");
    else if (n == "showpartybackground") lua_pushstring(L, "0");
    else if (n == "showpartypets") lua_pushstring(L, "1");
    else if (n == "showtargetoftarget") lua_pushstring(L, "0");
    else if (n == "targetoftargetmode") lua_pushstring(L, "5");
    else if (n == "sound_enablemusic") lua_pushstring(L, "1");
    else if (n == "chatbubbles") lua_pushstring(L, "1");
    // Off, which is what a stock client has and what interfaceoptionsframe.lua
    // itself declares as the default. Only reached before the handler exists;
    // once it does, the branch above answers from the setting itself.
    else if (n == "autolootdefault") lua_pushstring(L, "0");
    // On, as it is for a fresh account. The XP bar and the unit frames put
    // their whole tooltip behind this one: GameTooltip_AddNewbieTip is called
    // with noNormalText set, so with tips off it does nothing at all and
    // hovering the experience bar says nothing.
    else if (n == "shownewbietips") lua_pushstring(L, "1");
    // Off, which is what the real client ships and is a decision here rather
    // than an accident. GearManagerDialog_OnEvent shows GearManagerToggleButton
    // only `if ( GetCVarBool("equipmentManager") )`, and that button is the only
    // way onto the character sheet's gear manager - so this looks at first like
    // an unanswered CVar hiding a feature this client implements in full.
    //
    // It is not hidden, it is opt-in, exactly as in 3.3.5: Interface >
    // Features > Use Equipment Manager is a real panel here, its checkbox
    // writes this CVar, the store answers ahead of these defaults, and the
    // button appears on the next VARIABLES_LOADED. Turning it on by default
    // would be this client deciding something the real one leaves to the
    // player.
    //
    // Answered rather than left silent so the reasoning is on record: reading
    // as off by accident and reading as off on purpose look identical from
    // Lua and are not the same thing to whoever reads this next.
    else if (n == "equipmentmanager") lua_pushstring(L, "0");
    // On, which is the default interfaceoptionsframe itself declares for it:
    // `["SHOW_DISPELLABLE_DEBUFFS"] = { default = "1", ... }`. Answering
    // nothing read as off and contradicted that, so the party frames showed
    // every debuff on a member rather than the ones this character can lift.
    //
    // Answered only now that it drives something. UnitDebuff took the "RAID"
    // filter FrameXML passes and ignored it, so turning this on before would
    // have claimed a filter that does not filter - which is worse than the
    // wrong default, because it reads as working.
    else if (n == "showdispeldebuffs") lua_pushstring(L, "1");
    // Off, also as declared. It asks the same "RAID" filter of *buffs* - only
    // the ones this character could cast - and that half is not implemented,
    // so this stays where the real client leaves it rather than being turned
    // on into a filter that would not filter.
    else if (n == "showcastablebuffs") lua_pushstring(L, "0");
    // The numbers on a unit frame's bars. A stock 3.3.5 client keeps these off
    // and shows them on mouseover; on this one they are wanted permanently,
    // which is what the Status Text interface option turns on.
    // The unit frames each ask about their own, not about "statusText" - the
    // player frame's bars carry cvar = "playerStatusText". Defaulting only the
    // general one left every bar's numbers hidden, correct text and all.
    else if (n == "statustext" || n == "playerstatustext" ||
             n == "targetstatustext" || n == "petstatustext" ||
             n == "partystatustext") {
        lua_pushstring(L, "1");
    }
    else if (n == "statustextpercentage") lua_pushstring(L, "0");
    // Which stat category each column of the character sheet shows. These are
    // not preferences with a sensible fallback - UpdatePaperdollStats compares
    // the value against five names and fills the column from whichever matches,
    // so an unrecognised one matches nothing and every row is left blank. That
    // is what "0" gave it, and it is why the character sheet showed two empty
    // panels below the model with no error anywhere to say why: the code ran to
    // completion and simply had nothing to write.
    //
    // The two names below are what a fresh 3.3.5 account has.
    else if (n == "playerstatleftdropdown")  lua_pushstring(L, "PLAYERSTAT_BASE_STATS");
    else if (n == "playerstatrightdropdown") lua_pushstring(L, "PLAYERSTAT_MELEE_COMBAT");
    // Whether a conversation opens in its own window or in the chat frame.
    // "0" already behaved as "inline" - the only test is against "popout" -
    // so this changes nothing today. It is written out because the value is a
    // name rather than a number, which is the case where falling through to
    // "0" is luck rather than a default.
    else if (n == "conversationmode") lua_pushstring(L, "inline");
    // Who last spoke to you as a GM, and empty means nobody has.
    //
    // uiparent.lua does `if ( lastTalkedToGM ~= "" )` at login and, when that
    // passes, loads Blizzard_GMChatUI and *shows* it with a "your last session"
    // line. Falling through to "0" made that test pass every time, so the GM
    // chat window opened on every login for a player no GM had ever contacted.
    //
    // The empty string is not a placeholder here - it is the value the client
    // stores until a GM actually writes.
    else if (n == "lasttalkedtogm") lua_pushstring(L, "");
    // On, as a stock client has them. Each of these gates something off
    // entirely when it reads false, so "0" is not a quiet preference - it is
    // the feature missing with no way to ask for it back.
    //
    //   chatMouseScroll  the chat frame only calls EnableMouseWheel(true)
    //                    inside this test, so the wheel did nothing over chat
    //   showKeyring      MainMenuBar_UpdateKeyRing only ever calls
    //                    KeyRingButton:Show() inside it, so the keyring was
    //                    unreachable despite the slots being tracked
    // On, as WotLK has it - and everything behind it is built.
    //
    // This was left off last time for being untested, on the grounds that
    // turning it on changes what clicking a talent does. Checking rather than
    // assuming: all four preview functions are implemented, not stubbed -
    // AddPreviewTalentPoints stages against the real max rank,
    // GetGroupPreviewTalentPointsSpent totals the staging map,
    // LearnPreviewTalents sends one request per rank, and
    // ResetGroupPreviewTalentPoints clears it and fires the event the frame
    // redraws on. GetTalentInfo already answers previewRank and the
    // preview-aware availability flag.
    //
    // So the staging flow was written deliberately and then reached by
    // nothing, because the CVar that gates every one of those eight call
    // sites answered false.
    else if (n == "previewtalents") lua_pushstring(L, "1");
    else if (n == "chatmousescroll") lua_pushstring(L, "1");
    else if (n == "showkeyring")     lua_pushstring(L, "1");
    // The quest tracker's filter, and it is not a preference - it is a bitmask
    // fed to bit.band. watchframe.lua starts it at 0 on load and then, on
    // VARIABLES_LOADED, overwrites it with tonumber(GetCVar("trackerFilter")).
    // An unanswered CVar makes that nil, and bit.band(nil, x) raises: the
    // tracker worked until the login event that is supposed to configure it,
    // then went down on its next update, every session.
    //
    // Three: achievements and completed quests, and *not* quests from other
    // zones - which is what a stock 3.3.5 client has, with "Quests in other
    // zones" unticked in the tracker's own filter menu. Seven ticked it, so
    // the tracker listed every watched quest in the log whatever continent it
    // was on.
    //
    // Zero would not raise but is worse than it looks: the two tests at
    // watchframe.lua:813 skip a quest that is complete and a quest outside the
    // current map, so an empty mask hides most of the log.
    //
    // Clearing the remote-zones bit only filters if CURRENT_MAP_QUESTS is
    // filled, which is why it was left set - see the override of
    // WatchFrame_GetCurrentMapQuests in AddonManager, which fills it from the
    // log's zone headers rather than from map POIs.
    else if (n == "trackerfilter") lua_pushstring(L, "3");
    // Manual, which is WATCHFRAME_SORT_MANUAL. Only ever compared with ==, so
    // nil was survivable here - it is answered for the same reason its
    // neighbour is, and because the sort menu's ticks read it.
    else if (n == "trackersorting") lua_pushstring(L, "0");
    // The narrow tracker, which is what a stock client has.
    // WatchFrame_SetWidth tests `width == "0"` and takes the wide branch for
    // anything else, so nil quietly chose the wide one.
    else if (n == "watchframewidth") lua_pushstring(L, "0");
    // Opaque. WorldMapFrame_SetOpacity computes 0.5 + (1.0 - opacity) * 0.5,
    // which raises on a nil - and WorldMap_ToggleSizeDown calls it, so putting
    // the map into windowed mode took it down. Zero is opaque here: the
    // arithmetic reads the value as how transparent to be.
    else if (n == "worldmapopacity") lua_pushstring(L, "0");
    // Half, as the arena frames have it. The same arithmetic-on-nil shape,
    // reached only if the arena addon loads, which is why it is a default
    // rather than a fix.
    else if (n == "partybackgroundopacity") lua_pushstring(L, "0.5");
    // Full volume and sound on, which is what a fresh client has. These are
    // read as numbers by the sound options, where zero reads as silence
    // rather than as "unset".
    else if (n == "sound_mastervolume")   lua_pushstring(L, "1");
    else if (n == "sound_enableallsound") lua_pushstring(L, "1");
    // On, as a stock client has it. ActionButton_SetTooltip branches on this:
    // with it off the tooltip is anchored to the right of the button itself, so
    // an action bar tooltip appeared at the bottom of the screen across the
    // icons. On, it goes through GameTooltip_SetDefaultAnchor to the
    // bottom-right corner, clear of the bar, which is where WoW puts it.
    else if (n == "ubertooltips") lua_pushstring(L, "1");
    // The target's cast bar, which a stock client shows. targetframe.lua tests
    // this as `if ( GetCVar("showTargetCastbar") == "0" )` and sets
    // showCastbar false - and an unrecognised name answers exactly "0" here,
    // so that branch was taken every time and the target frame is one of the
    // elements FrameXML draws by default. Nothing about it looked broken: the
    // bar was switched off by a setting nobody had touched.
    else if (n == "showtargetcastbar") lua_pushstring(L, "1");
    // Quest markers on the world map, on in a stock WotLK client. The
    // objectives checkbox reads this for its initial state, so "0" started it
    // unchecked and the markers off.
    else if (n == "questpoi") lua_pushstring(L, "1");
    // The aggro warning on the unit frames. Three is "always", which is what
    // the Display panel lists first and what a stock WotLK client ships with;
    // one is "in an instance", two "in a party", zero never. Falling through
    // to the "0" below would have left the indicator off with the option
    // reading Never, which is a preference nobody chose.
    else if (n == "threatwarning") lua_pushstring(L, "3");
    // The social options panel branches on this and raises on anything it does
    // not recognise, so "0" - what an unknown CVar answers - took its whole
    // update down. "classic" is the stock setting.
    else if (n == "chatstyle") lua_pushstring(L, "classic");
    // "none", which is the word this one is switched off with - and the blanket
    // default below is a number, which is not off but a format string.
    //
    // This is not a preference that read wrong. Timestamps are held as the
    // strftime format to print, and the interface tests the *word* "none" for
    // off: anything else is truthy and gets printed. So the fallback "0" became
    // CHAT_TIMESTAMP_FORMAT = "0", strftime copied that digit through as a
    // literal, and every line of chat in the game arrived with a 0 stuck to the
    // front of it.
    //
    // Worth remembering when adding a CVar here: the fallback answers "0" for
    // everything unlisted, which is right for a flag and wrong for any setting
    // whose value is a word.
    else if (n == "showtimestamps") lua_pushstring(L, "none");
    else lua_pushstring(L, "0");
}


/// The CVars whose value is a client setting.
///
/// FrameXML's Video and Interface panels are bound to CVar names. The values
/// behind these names are settings this client already had - they simply had
/// never been introduced, so the panels wrote to a store nothing read and the
/// controls did nothing.
///
/// A row here, a key in Application's bridge, and the control works. The
/// alternative - which the entries below this table still are - is a getter and
/// a setter on LuaServices, a lambda in Application, and a branch in each of
/// GetCVar and SetCVar, four places per option.
struct ClientCVarBinding {
    const char* cvar;      ///< lower case, as both sides fold it
    const char* setting;   ///< the key Application's bridge answers to
    /// What the setting reads when the CVar reads one, where the two are not
    /// counting the same thing.
    ///
    /// Almost always they are: a volume is 0 to 1 on both sides, a checkbox is
    /// 0 or 1. Ground clutter is not. Blizzard's Ground Density slider counts
    /// doodads and runs 16 to 64; this client's setting is a proportion and
    /// runs 0 to 1.5. Handing one straight to the other put 64 into a field
    /// that stops at 1.5, so every position of that slider - all seven of them
    /// - came out as the most clutter the client will draw, and stayed there:
    /// the value written to the config was clamped back down on the next load,
    /// which made it permanent.
    double scale = 1.0;
};

// NOLINTBEGIN(modernize-use-designated-initializers) - read as text by
// tools/settings_without_a_control.py and tools/cvar_slider_range_fit.py,
// which match {"cvar", "setting"} to find which settings a Blizzard slider
// drives. Naming the fields makes both of them see an empty table.
constexpr ClientCVarBinding kClientCVars[] = {
    {"farclip",              "viewdistance"},
    // Mouse Sensitivity. Its shipped range is 0.5 to 1.5, a multiplier around
    // 1.0, and this client's sensitivity is an amount that runs 0.05 to 1.0 -
    // so passed through unchanged the slowest setting that slider offered was
    // two and a half times this client's default. Its range is redefined in
    // kCVarRanges rather than converted here, which is what farclip and
    // cameraYawMoveSpeed do, and it has to be: Mouse Look Speed writes this
    // same setting through that route, so a scale on this one alone would have
    // the two sliders disagree about the value they share.
    {"mousespeed",           "mousespeed"},
    {"showclock",            "minimapclock"},
    {"nameplateshowfriends", "friendlyplates"},
    // Deliberately not gxWindow. It is answered further down, from the store
    // first and the window only as a default - RestartGx applies it after the
    // panel has written it, so between the tick and the Okay the stored value
    // is the truth and the window is still showing the old state. A binding
    // here would sit above the store and hand RestartGx back what it was about
    // to set.
    // 64 doodads is the most Blizzard's slider asks for and 1.5 is the most
    // this client draws, so one of theirs is 1.5/64 of ours.
    {"groundeffectdensity",  "groundclutter", 1.5 / 64.0},
    {"sound_sfxvolume",      "effectsvolume"},
    // Three that were each written out four times over - a getter and a setter
    // on LuaServices, a lambda in Application, and a branch in each of GetCVar
    // and SetCVar - because the client had no key for them when they were
    // added. It has now, so they are rows like the rest.
    {"gxvsync",              "vsync"},
    {"mouseinvertpitch",     "invertmouse"},
    // The client keeps this one and pushes it at the game handler each frame,
    // so writing the setting is what makes the checkbox stick across a session
    // as well as within one. The branch it replaces wrote the handler directly
    // and never saved.
    {"autolootdefault",      "autoloot"},
};
// NOLINTEND(modernize-use-designated-initializers)

/// What another CVar currently reads as, for the settings that only mean
/// something in pairs. The store first, then the default, which is the same
/// order GetCVar answers in.
std::string cvarValueOr(lua_State* L, const char* name, const char* fallback) {
    (void)L;
    if (auto it = cvarStore().find(name); it != cvarStore().end()) return it->second;
    return fallback;
}

/// Set while a CVar is being applied to the setting it drives.
///
/// The store already holds the value in that direction, and echoing it back
/// rewrites it with whatever survives the trip: ground clutter is kept as a
/// whole percent, so a Ground Density of 24 came back as 23.893333 and the
/// store no longer said what the player picked - nor any position that slider
/// can be put in.
static bool g_applyingCVarToSetting = false;

const ClientCVarBinding* findClientCVar(const std::string& lowerName) {
    for (const auto& b : kClientCVars) {
        if (lowerName == b.cvar) return &b;
    }
    return nullptr;
}

static int lua_GetCVar(lua_State* L) {
    const char* name = luaL_checkstring(L, 1);
    // Folded to lower case, because the client's CVar names are not
    // case-sensitive and the interface does not spell them consistently.
    // uidropdownmenu.lua asks for "uiscale" where everything else says
    // "uiScale"; an exact match answered "0" for it, tonumber("0") is 0, and
    // every dropdown menu in the interface opened at SetScale(0) - laid out,
    // drawn, and invisible.
    std::string n(name);
    toLowerInPlace(n);
    // Asked of the client before the store, for the two settings it also owns.
    // The V key toggles nameplates and the settings panel turns the minimap,
    // neither of which goes through SetCVar; answering from the store would
    // report whatever the interface last wrote, which by then is a guess.
    if (n == "nameplateshowenemies") {
        if (auto* svc = getLuaServices(L); svc && svc->getNameplatesShown) {
            lua_pushstring(L, svc->getNameplatesShown() ? "1" : "0");
            return 1;
        }
    } else if (n == "rotateminimap") {
        if (auto* svc = getLuaServices(L); svc && svc->getMinimapRotate) {
            lua_pushstring(L, svc->getMinimapRotate() ? "1" : "0");
            return 1;
        }
    } else if (n == "chatbubbles") {
        if (auto* svc = getLuaServices(L); svc && svc->getChatBubblesShown) {
            lua_pushstring(L, svc->getChatBubblesShown() ? "1" : "0");
            return 1;
        }
    } else if (n == "sound_mastervolume" || n == "sound_musicvolume" ||
               n == "sound_ambiencevolume" || n == "sound_enableallsound") {
        // FrameXML's Sound options are bound to these, and the client owns the
        // values. Answering from the CVar store would report whatever the panel
        // last wrote and drift from what is actually playing - the same fault
        // the nameplate and minimap entries above exist for.
        if (auto* svc = getLuaServices(L); svc && svc->getAudioSetting) {
            const bool isSwitch = (n == "sound_enableallsound");
            const char* key = n == "sound_mastervolume"   ? "master"
                            : n == "sound_musicvolume"    ? "music"
                            : n == "sound_ambiencevolume" ? "ambient"
                                                          : "enableall";
            const float v = svc->getAudioSetting(key);
            // A switch answers "1" or "0" and nothing else. The options panels
            // compare the string exactly - `if ( value == "1" )` - so a float
            // formatted as "1.000000" reads as off, and the box unticks itself
            // the moment the panel is opened again.
            if (isSwitch) {
                lua_pushstring(L, v != 0.0f ? "1" : "0");
            } else {
                lua_pushstring(L, ui::settingNumberText(v).c_str());
            }
            return 1;
        }
    } else if (n == "autoselfcast") {
        if (auto* gh = getGameHandler(L)) {
            lua_pushstring(L, gh->isAutoSelfCast() ? "1" : "0");
            return 1;
        }
    } else if (n == "showtimestamps") {
        // A stored "0" here was never chosen. It is what the old default
        // answered, and the options panel writes back whatever it read, so a
        // config saved while that default was live has the fault baked into
        // it - and fixing only the default would leave those players with the
        // 0 in front of every line for good. The word for off is "none"; a
        // digit is a strftime format that prints itself.
        if (auto it = cvarStore().find(n); it != cvarStore().end() &&
            (it->second.empty() || it->second == "0")) {
            lua_pushstring(L, "none");
            return 1;
        }
    }
    if (n == "uiscale") {
        // From the tree, above the store. The tree clamps to the range the
        // slider offers, so a stored value would report back what was asked for
        // rather than what was applied - which is how a control comes to show a
        // number the interface is not using.
        if (auto* tree = getWidgetTree(L)) {
            lua_pushstring(L, ui::settingNumberText(tree->userScale()).c_str());
            return 1;
        }
    }
    if (const auto* binding = findClientCVar(n)) {
        if (auto* svc = getLuaServices(L); svc && svc->getClientSetting) {
            const std::string v = svc->getClientSetting(binding->setting);
            if (!v.empty()) {
                // Back into the units the slider is built in. Left alone when
                // the two count the same thing, so a "1" stays "1" rather than
                // becoming "1.000000" on its way through a double.
                if (binding->scale != 1.0) {
                    lua_pushstring(L, ui::settingNumberText(std::atof(v.c_str()) /
                                                           binding->scale).c_str());
                } else {
                    lua_pushstring(L, v.c_str());
                }
                return 1;
            }
        }
    }
    if (auto it = cvarStore().find(n); it != cvarStore().end()) {
        lua_pushstring(L, it->second.c_str());
        return 1;
    }
    pushCvarDefault(L, n);
    return 1;
}

/// GetCVarDefault(name) → what it would be if nobody had set it.
///
/// Not the same question as GetCVar, and it used to be answered by the same
/// function. The options panels capture control.defaultValue from this as they
/// load and the Defaults button writes it back, so aliasing the two made that
/// button restore each control to the value it already had.
static int lua_GetCVarDefault(lua_State* L) {
    const char* name = luaL_checkstring(L, 1);
    std::string n(name);
    toLowerInPlace(n);
    pushCvarDefault(L, n);
    return 1;
}

/// GetCVarBool(name) → the setting as a boolean.
///
/// FrameXML branches on this, and one of those branches decides how a unit
/// frame's health bar keeps itself current: predictedHealth sends it down an
/// OnUpdate poll instead of registering UNIT_HEALTH. Unimplemented, the call
/// answered nil through the fallback and took the event branch by luck - which
/// is the branch that works here, but only until the fallback is off, when the
/// same call errors instead.
static int lua_GetCVarBool(lua_State* L) {
    lua_pushvalue(L, 1);
    lua_GetCVar(L);
    const char* v = lua_tostring(L, -1);
    const bool on = v && *v && std::string(v) != "0";
    lua_pop(L, 1);
    lua_pushboolean(L, on);
    return 1;
}

/// What CVAR_UPDATE carries as its first argument: the CVar's *label*, not its
/// name. The two are different spellings of the same setting, and FrameXML uses
/// both within two lines of each other -
///
///     if ( (event == "CVAR_UPDATE") and (arg1 == "SHOW_TARGET_CASTBAR") ) then
///         if ( GetCVar("showTargetCastbar") == "0") then
///
/// - so there is no ambiguity about which belongs where. Firing the name meant
/// every consumer in the interface compared a camelCase name against an
/// upper-case label and took the other branch, silently: the health and mana
/// numbers on unit frames never appeared or disappeared, the free-bag-slots
/// count never switched on, and the target and focus cast bars never followed
/// their setting.
///
/// Mostly the label is the name in upper snake case, and where it is not,
/// FrameXML states the pair itself: the status-text bars set self.cvar and
/// self.cvarLabel on consecutive lines of their OnLoad, and the rest are named
/// in the comparisons that read them. The real client keeps this mapping in a
/// CVar registry inside the binary, which is not something this client has, so
/// what is written here is what the interface itself says.
static std::string cvarLabelFor(const std::string& name) {
    // The ones the transform below would get wrong. Every entry is a pair
    // FrameXML writes down somewhere.
    static const std::pair<const char*, const char*> kNamed[] = {
        {"playerstatustext",     "STATUS_TEXT_PLAYER"},
        {"partystatustext",      "STATUS_TEXT_PARTY"},
        {"petstatustext",        "STATUS_TEXT_PET"},
        {"targetstatustext",     "STATUS_TEXT_TARGET"},
        {"statustextpercentage", "STATUS_TEXT_PERCENT"},
        {"showcastabledebuffs",  "SHOW_CASTABLE_DEBUFFS_TEXT"},
        {"showarenaenemyframes", "SHOW_ARENA_ENEMY_FRAMES_TEXT"},
    };
    std::string key(name);
    toLowerInPlace(key);
    for (const auto& [cvar, label] : kNamed) {
        if (key == cvar) return label;
    }
    // showTargetCastbar → SHOW_TARGET_CASTBAR, displayFreeBagSlots →
    // DISPLAY_FREE_BAG_SLOTS, xpBarText → XP_BAR_TEXT.
    std::string out;
    for (size_t i = 0; i < name.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(name[i]);
        if (i > 0 && std::isupper(c) && !std::isupper(static_cast<unsigned char>(name[i - 1]))) {
            out += '_';
        }
        out += static_cast<char>(std::toupper(c));
    }
    return out;
}

// SetCVar(name, value [, scriptCVar])
/// Everything a CVar does besides being remembered.
///
/// Split out of lua_SetCVar so the same work can be done for the values
/// restored from disk. loadStoredCVars fills the map directly - it has to, it
/// runs before any of these services exist - so without this every setting
/// here was saved on exit and never applied again. The panel read the stored
/// value back and showed it correctly while the client ran on the default,
/// which is the shape of "I set it, it looks set, and nothing happened".
///
/// CVAR_UPDATE is deliberately left behind in lua_SetCVar: that event says
/// somebody changed a setting, which is not what restoring one is.
static void applyCVarSideEffects(lua_State* L, const std::string& key,
                                 const std::string& value) {
    // A sound CVar is a setting, not a note. Without this the interface's
    // volume keys and its Sound options both wrote to a map nobody read, so
    // turning music off left it playing.
    // The interface's own scale, which is the widget tree's to keep - not a
    // client setting, and not the ImGui window scale beside it in the settings
    // window. Those are two different interfaces that happen to share a word.
    if (key == "uiscale") {
        if (auto* tree = getWidgetTree(L)) {
            // Only when the switch beside it is on. Otherwise the stored value
            // is kept for when it is turned back on, but not applied.
            const std::string useIt = cvarValueOr(L, "useuiscale", "1");
            if (useIt != "0") {
                tree->setUserScale(static_cast<float>(std::atof(value.c_str())));
            }
        }
    }
    // Unticking it puts the interface back to the size the screen's own height
    // gives, which is what the tick means: use a scale of mine rather than the
    // default. It did nothing at all before - the box moved and the interface
    // stayed exactly as it was, at whatever scale had been set.
    // How far back the camera may be pulled. Straight to the camera rather than
    // through a client setting, so there is one control for it and not two.
    if (key == "cameradistancemaxfactor") {
        if (auto* svc = getLuaServices(L); svc && svc->setCameraMaxDistanceFactor) {
            svc->setCameraMaxDistanceFactor(static_cast<float>(std::atof(value.c_str())));
        }
    }
    // Camera Following Style, whose one meaningful distinction here is its
    // first option: "Never adjust camera" against the three that do. The
    // client has the same switch already, reached from its own panel, so this
    // is routed into that setting rather than to the camera directly - two
    // controls over one value, which is what setSettingValue exists for, and
    // it persists on the way past.
    if (key == "camerasmoothstyle") {
        if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
            svc->setClientSetting("smoothfollow", value == "0" ? "0" : "1");
        }
    }
    // Camera Following Speed. The shipped CVar is in degrees a second and this
    // camera smooths with a rate constant, so the two numbers do not convert -
    // and inventing a conversion would put a slider in front of a value it
    // does not describe. The range is redefined instead, to exactly what
    // CameraController::setCameraSmoothSpeed accepts, so the control covers
    // what this client can do and the number passes through untouched. Same
    // reasoning as farclip's range below.
    if (key == "camerayawsmoothspeed") {
        if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
            svc->setClientSetting("camerastiffness", value);
        }
    }
    // Mouse Look Speed. Named cameraYawMoveSpeed, which is what it turns, but
    // the control is the mouse slider on the Mouse panel - and this client has
    // that setting already, so it goes there rather than to the camera, one
    // value with two controls.
    //
    // The range is redefined below to the one the client's own slider uses,
    // for the same reason farclip's is: passing a number through unchanged
    // beats converting between two scales that were never the same.
    if (key == "camerayawmovespeed") {
        if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
            svc->setClientSetting("mousespeed", value);
        }
    }
    // Loop Music. The checkbox says Loop Music and the CVar behind it says
    // ZoneMusicNoDelay, which is the same thing said from the other side: a
    // track that runs on leaves no silence between it and the next.
    if (key == "sound_zonemusicnodelay") {
        if (auto* svc = getLuaServices(L); svc && svc->setZoneMusicLooping) {
            svc->setZoneMusicLooping(value != "0");
        }
    }
    // Ground Clutter Radius, in yards and used as it stands.
    if (key == "groundeffectdist") {
        if (auto* svc = getLuaServices(L); svc && svc->setGroundDetailDistance) {
            svc->setGroundDetailDistance(static_cast<float>(std::atof(value.c_str())));
        }
    }

    // Texture Filtering, offered as levels 0 to 5 rather than as a number of
    // samples: each step doubles, and the last two are both the 16x that is
    // every desktop GPU's maximum. The panel marks this one gameRestart, so
    // what it changes is the textures loaded from here on.
    if (key == "texturefilteringmode") {
        if (auto* svc = getLuaServices(L); svc && svc->setAnisotropyLimit) {
            const int level = std::clamp(std::atoi(value.c_str()), 0, 5);
            svc->setAnisotropyLimit(static_cast<float>(std::min(1 << level, 16)));
        }
    }

    // Environment Detail, offered as 0.5 to 1.5 and passed on as it stands.
    if (key == "environmentdetail") {
        if (auto* svc = getLuaServices(L); svc && svc->setEnvironmentDetail) {
            svc->setEnvironmentDetail(static_cast<float>(std::atof(value.c_str())));
        }
    }

    // Particle Density, which the panel already offers as a fraction - 0.1 to
    // 1 - so it needs no conversion, only passing on.
    if (key == "particledensity") {
        if (auto* svc = getLuaServices(L); svc && svc->setParticleDensity) {
            svc->setParticleDensity(static_cast<float>(std::atof(value.c_str())));
        }
    }

    // Weather Detail, which the panel offers as 0 to 3. That is a count of
    // steps rather than a fraction, so it is divided by its own maximum: 0
    // draws no weather at all, which is what the bottom of that slider means.
    if (key == "weatherdensity") {
        if (auto* svc = getLuaServices(L); svc && svc->setWeatherDensity) {
            constexpr float kWeatherDetailSteps = 3.0f;
            svc->setWeatherDensity(
                static_cast<float>(std::atof(value.c_str())) / kWeatherDetailSteps);
        }
    }
    if (key == "useuiscale") {
        if (auto* tree = getWidgetTree(L)) {
            if (value == "0") {
                tree->setUserScale(1.0f);
            } else {
                tree->setUserScale(
                    static_cast<float>(std::atof(cvarValueOr(L, "uiscale", "1").c_str())));
            }
        }
    }
    // The table first: these are settings the client owns and the panels drive.
    if (const auto* binding = findClientCVar(key)) {
        if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
            // The store is the source here, so what the setting does with the
            // value must not come back and overwrite it.
            g_applyingCVarToSetting = true;
            if (binding->scale != 1.0) {
                svc->setClientSetting(binding->setting,
                                      ui::settingNumberText(std::atof(value.c_str()) *
                                                            binding->scale));
            } else {
                svc->setClientSetting(binding->setting, value);
            }
            g_applyingCVarToSetting = false;
        }
    }
    // The four the client owns go to its settings, which then apply and save.
    // Both systems used to write the same AudioEngine - the CVar store from
    // here and the settings panel from its own sliders - and whichever ran last
    // won. A client left muted by its own setting came back silent however the
    // interface's Sound panel was set, because the next thing to touch the
    // settings slammed the master volume back to zero.
    if (key == "sound_mastervolume" || key == "sound_musicvolume" ||
        key == "sound_ambiencevolume" || key == "sound_enableallsound") {
        if (auto* svc = getLuaServices(L); svc && svc->setAudioSetting) {
            const char* which = key == "sound_mastervolume"   ? "master"
                              : key == "sound_musicvolume"    ? "music"
                              : key == "sound_ambiencevolume" ? "ambient"
                                                              : "enableall";
            svc->setAudioSetting(which, static_cast<float>(std::atof(value.c_str())));
            return;
        }
        applySoundCVars(L);
    }
    else if (key.rfind("sound_", 0) == 0) applySoundCVars(L);
    // The two other CVars this client can act on. "0" is the only false value
    // a CVar carries - and it arrives as a string, which in Lua would be true.
    else if (key == "nameplateshowenemies") {
        if (auto* svc = getLuaServices(L); svc && svc->setNameplatesShown)
            svc->setNameplatesShown(value != "0");
    } else if (key == "rotateminimap") {
        if (auto* svc = getLuaServices(L); svc && svc->setMinimapRotate)
            svc->setMinimapRotate(value != "0");
    } else if (key == "autoselfcast") {
        if (auto* gh = getGameHandler(L)) gh->setAutoSelfCast(value != "0");
    } else if (key == "chatbubbles") {
        if (auto* svc = getLuaServices(L); svc && svc->setChatBubblesShown)
            svc->setChatBubblesShown(value != "0");
    }
}

/// Apply what was loaded from disk, once the services behind it exist.
///
/// Called after the interface is up rather than at load: the store is filled
/// before any renderer, camera or audio manager is wired, and every branch
/// above checks its service and would quietly do nothing that early - which is
/// indistinguishable from the fault this exists to fix.
std::string storedCVarValue(const std::string& key, const std::string& fallback) {
    // The store first, for a call made after the interface is up; the file
    // otherwise, which is the case this exists for. Reading the file twice
    // costs nothing and keeps the two answers the same.
    std::string wanted = key;
    toLowerInPlace(wanted);
    if (auto it = cvarStore().find(wanted); it != cvarStore().end()) return it->second;

    std::ifstream in(cvarStorePath());
    if (!in.is_open()) return fallback;
    std::string line;
    while (std::getline(in, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos || eq == 0) continue;
        std::string k = line.substr(0, eq);
        toLowerInPlace(k);
        if (k == wanted) return line.substr(eq + 1);
    }
    return fallback;
}

void noteClientSettingChanged(const std::string& settingKey, const std::string& value) {
    if (g_applyingCVarToSetting) return;
    for (const auto& binding : kClientCVars) {
        if (settingKey != binding.setting) continue;
        // Back into the CVar's own units, the same conversion GetCVar makes.
        const std::string text =
            binding.scale != 1.0
                ? ui::settingNumberText(std::atof(value.c_str()) / binding.scale)
                : value;
        auto it = cvarStore().find(binding.cvar);
        if (it != cvarStore().end() && it->second == text) return;
        cvarStore()[binding.cvar] = text;
        saveStoredCVars();
        return;
    }
}

void applyStoredCVarSideEffects(lua_State* L) {
    for (const auto& [key, value] : cvarStore()) {
        applyCVarSideEffects(L, key, value);
    }
    LOG_INFO("CVars: applied ", cvarStore().size(), " stored values");
}

/// Report the controls kRemovedControlsLua could not find.
///
/// An entry there naming nothing is a bug in the list rather than a state of
/// the interface - nine of them named CVars where frames were wanted and were
/// skipped in silence, which is how the list came to be half decorative.
static int lua_WoweeReportMissingFixedControls(lua_State* L) {
    const char* names = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
    if (names && *names) {
        LOG_WARNING("Fixed controls: no such frame(s): ", names);
    }
    return 0;
}

static int lua_SetCVar(lua_State* L) {
    const char* name = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
    if (!name) return 0;
    // A number is as valid as a string here and FrameXML passes both.
    std::string value;
    if (lua_isstring(L, 2) || lua_isnumber(L, 2)) {
        value = lua_tostring(L, 2);
    } else if (lua_isboolean(L, 2)) {
        value = lua_toboolean(L, 2) ? "1" : "0";
    } else {
        // nil is how an unticked box reports itself, and these are written
        // straight through: SetCVar("questPOI", self:GetChecked()). Storing
        // the empty string for it would make GetCVar answer something that is
        // neither "0" nor a number, so tonumber(GetCVar(...)) - which the
        // options panels do - would give nil where it wanted a zero.
        value = "0";
    }
    // The same folding as the read side, or a value written as "uiScale"
    // would be invisible to a read of "uiscale".
    std::string key(name);
    toLowerInPlace(key);
    const auto existing = cvarStore().find(key);
    const bool changed = (existing == cvarStore().end() || existing->second != value);
    cvarStore()[key] = value;
    // Only on a real change. A slider drag calls SetCVar on every frame it
    // moves, and most of those calls set the value it already has.
    if (changed) saveStoredCVars();
    applyCVarSideEffects(L, key, value);
    // Announced, because nine frames listen for it - the options panels redraw
    // themselves from this rather than from the click that caused it.
    // Through the engine in the registry, which is where it puts itself; the
    // event tables are its business rather than this file's.
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (engine) engine->fireEvent("CVAR_UPDATE", {cvarLabelFor(name), value});
    return 0;
}


static int lua_GetNumAddOns(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_count");
    return 1;
}

/// Pushes the addon registry table and resolves argument `arg` to a 1-based
/// index into it.
///
/// FrameXML passes either an index or a name to every addon query, so both
/// GetAddOnInfo and GetAddOnMetadata had to accept both, and each carried its
/// own copy of the search.
///
/// On success the table is left on the stack and the caller indexes it. On
/// failure the stack is left exactly as it was found and the answer is 0, so
/// the caller only has to push its own nil. Getting that asymmetry wrong is
/// how a binding leaks a table per call and the interpreter grows all session.
///
/// Names are matched exactly. FrameXML's own addon list passes back the name
/// it was given, so a case difference here would be one it introduced itself.
int pushAddonRegistryAndIndex(lua_State* L, int arg) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_info");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return 0;
    }

    int idx = 0;
    if (lua_isnumber(L, arg)) {
        idx = static_cast<int>(lua_tonumber(L, arg));
    } else if (lua_isstring(L, arg)) {
        const char* wanted = lua_tostring(L, arg);
        const int count = static_cast<int>(lua_objlen(L, -1));
        for (int i = 1; i <= count; i++) {
            lua_rawgeti(L, -1, i);
            lua_getfield(L, -1, "name");
            const char* name = lua_tostring(L, -1);
            lua_pop(L, 1);
            if (name && strcmp(name, wanted) == 0) { idx = i; lua_pop(L, 1); break; }
            lua_pop(L, 1);
        }
    }

    if (idx < 1) {
        lua_pop(L, 1);
        return 0;
    }
    return idx;
}

static int lua_GetAddOnInfo(lua_State* L) {
    // Accept index (1-based) or addon name
    const int idx = pushAddonRegistryAndIndex(L, 1);
    if (idx < 1) { lua_pushnil(L); return 1; }

    lua_rawgeti(L, -1, idx);
    if (!lua_istable(L, -1)) { lua_pop(L, 2); lua_pushnil(L); return 1; }

    // name, title, notes, url, loadable, reason, security, newVersion.
    //
    // Five were returned and the fourth was not url, so everything from there
    // shifted: the loadable flag landed in url's place, the security string in
    // loadable's - truthy, so addons read as loadable by accident - and the
    // last three came back nil. addonlist.lua reads security seventh.
    lua_getfield(L, -1, "name");
    lua_getfield(L, -2, "title");
    lua_getfield(L, -3, "notes");

    // The registry table and the entry are still underneath, and `return 8`
    // takes the top eight of the stack whatever they are. Leaving them there
    // shifted the answer by one: the entry table arrived as the name, the name
    // as the title, and loadable came back nil - falsy - so every addon read
    // as unloadable while the security string landed in newVersion. The pop
    // this replaces removed the last value pushed rather than either of them.
    lua_remove(L, -4);              // the addon info entry
    lua_remove(L, -4);              // the registry table

    lua_pushnil(L);                 // 4: url - not in a 3.3.5 manifest
    lua_pushboolean(L, 1);          // 5: loadable
    lua_pushnil(L);                 // 6: reason it is not, and it is
    lua_pushstring(L, "INSECURE");  // 7: security
    lua_pushnil(L);                 // 8: newVersion
    return 8;
}

// GetAddOnMetadata(addonNameOrIndex, key) → value
static int lua_GetAddOnMetadata(lua_State* L) {
    const int idx = pushAddonRegistryAndIndex(L, 1);
    if (idx < 1) { lua_pushnil(L); return 1; }

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
    // One past the WorldMapArea id, which is what 3.3.5 answers here. The
    // interface's own arithmetic says so and is the authority: it takes this
    // minus one and hands the result to SetMapByID, and treats a negative
    // result as "no zone, ask which continent instead". So zero means a
    // continent is showing, and every other value is an id plus one.
    //
    // The physical map id was answered instead - 0 for Eastern Kingdoms, 571
    // for Northrend - which is a different number in a different space. It
    // made the Wintergrasp check (area 502) never true, and the windowed-size
    // toggle reopen the map somewhere else.
    if (auto* svc = getLuaServices(L)) {
        if (svc->getMapWorldAreaId) {
            const uint32_t wma = svc->getMapWorldAreaId();
            lua_pushnumber(L, wma == 0 ? 0 : static_cast<lua_Number>(wma + 1));
            return 1;
        }
    }
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getCurrentMapId() : 0);
    return 1;
}

// GetZoneText() / GetRealZoneText() → current zone name
static int lua_GetZoneText(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, ""); return 1; }
    // The terrain under the player first, and the server's zone only as a
    // fallback.
    //
    // It was the other way round, and the server's zone reaches us on
    // SMSG_INIT_WORLD_STATES alone - sent when the server notices a zone
    // change, and not otherwise. So the name stayed on the last zone the
    // server announced while the player walked out of it, which is what
    // "Silverpine Forest" over Hillsbrad Foothills is. The real client works
    // this out from its own map data for that reason, and this one already
    // does the same work every frame for weather and music.
    uint32_t zoneId = 0;
    if (auto* svc = getLuaServices(L)) {
        if (svc->getLiveZoneId) zoneId = svc->getLiveZoneId();
    }
    if (zoneId == 0) zoneId = gh->getWorldStateZoneId();
    if (zoneId != 0) {
        std::string name = gh->getWhoAreaName(zoneId);
        if (!name.empty()) { lua_pushstring(L, name.c_str()); return 1; }
    }
    lua_pushstring(L, "");
    return 1;
}

// GetSubZoneText() → subzone name (same as zone for now - server doesn't always send subzone)
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

/// The map view changed, so say so.
///
/// Fired unconditionally, including when the view was already what it is being
/// set to. That is not laziness - watchframe.lua calls SetMapToCurrentZone
/// purely for the side effect, and says so beside the call: "forces WatchFrame
/// event via the WORLD_MAP_UPDATE event, needed to restore the POIs in the
/// tracker to the current zone". Firing only on a change would drop exactly the
/// case that line exists for.
static void fireWorldMapUpdate(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (engine) engine->fireEvent("WORLD_MAP_UPDATE", {});
}

// SetMapToCurrentZone() - sets map view to the player's current zone
static int lua_SetMapToCurrentZone(lua_State* L) {
    // Called every time the map is shown. It set two statics that nothing
    // reads any more and never told the map, so a map left on another zone
    // stayed there when reopened.
    if (auto* svc = getLuaServices(L)) {
        if (svc->showPlayerMapZone) svc->showPlayerMapZone();
    }
    auto* gh = getGameHandler(L);
    if (gh) {
        s_mapContinent = mapIdToContinent(gh->getCurrentMapId());
        s_mapZone = static_cast<int>(gh->getWorldStateZoneId());
    }
    fireWorldMapUpdate(L);
    return 0;
}

// GetCurrentMapContinent() → continentId (1=Kalimdor, 2=EK, 3=Outland, 4=Northrend)
static int lua_GetCurrentMapContinent(lua_State* L) {
    // What the map is showing, asked of the map. This kept a static of its
    // own instead, set from the player's position on first use and from
    // SetMapZoom after that, and the map itself was never consulted or told -
    // so the two dropdowns and the zoom-out button moved a number no one drew
    // from while the map stayed where it was.
    if (auto* svc = getLuaServices(L)) {
        if (svc->getMapContinentIndex) {
            const int shown = svc->getMapContinentIndex();
            // Said once per disagreement, because this one is asked from
            // WorldMapFrame_OnUpdate and a line per frame is not a report.
            //
            // Zero from the map means WORLD or COSMIC level, and both
            // dropdowns answer it with UIDropDownMenu_ClearAll - so a pick
            // that moved the number here but did not move the map leaves the
            // dropdown blank, which reads as the dropdown not working.
            static int lastReported = -1;
            if (shown == 0 && s_mapContinent != 0 && lastReported != s_mapContinent) {
                lastReported = s_mapContinent;
                LOG_WARNING("GetCurrentMapContinent: asked for continent ",
                            s_mapContinent, " and the map is showing none, so"
                            " both dropdowns clear themselves");
            } else if (shown != 0) {
                lastReported = -1;
            }
            lua_pushnumber(L, shown);
            return 1;
        }
    }
    if (s_mapContinent == 0) {
        auto* gh = getGameHandler(L);
        if (gh) s_mapContinent = mapIdToContinent(gh->getCurrentMapId());
    }
    lua_pushnumber(L, s_mapContinent);
    return 1;
}

// GetCurrentMapZone() → zoneId
static int lua_GetCurrentMapZone(lua_State* L) {
    // A row in the zone dropdown, not an area id. The fallback below answered
    // the area id - a number in the thousands where the dropdown wanted a
    // position in a list of a few dozen - so the selected row was never the
    // one being shown. SetMapZoom, meanwhile, wrote a row number into the same
    // static, and the two meanings sat in one variable.
    if (auto* svc = getLuaServices(L)) {
        if (svc->getMapZoneIndex) {
            lua_pushnumber(L, svc->getMapZoneIndex());
            return 1;
        }
    }
    if (s_mapZone == 0) {
        auto* gh = getGameHandler(L);
        if (gh) s_mapZone = static_cast<int>(gh->getWorldStateZoneId());
    }
    lua_pushnumber(L, s_mapZone);
    return 1;
}

// SetMapZoom(continent [, zone]) - sets map view to continent/zone
static int lua_SetMapZoom(lua_State* L) {
    s_mapContinent = static_cast<int>(luaL_checknumber(L, 1));
    s_mapZone = static_cast<int>(luaL_optnumber(L, 2, 0));
    // Tell the map, which this never did. Every route out of a zone map runs
    // through here - the zone dropdown, the continent dropdown, and four of
    // the zoom-out button's six branches - so none of them changed what was
    // drawn.
    // At warning when the map turns the pair down, because every symptom of
    // that is a non-event: the dropdown keeps the row it was clicked on, the
    // map stays where it was, and nothing is written down. An unknown
    // continent and a zone row past the end of that continent's list are the
    // two ways in, and they are the two the dropdowns can produce.
    if (auto* svc = getLuaServices(L)) {
        if (!svc->setMapByIndex) {
            LOG_WARNING("SetMapZoom(", s_mapContinent, ", ", s_mapZone,
                        "): no map to tell - the view number moved and nothing"
                        " drew from it");
        } else if (!svc->setMapByIndex(s_mapContinent, s_mapZone)) {
            LOG_WARNING("SetMapZoom(", s_mapContinent, ", ", s_mapZone,
                        "): the map refused the pair, so it stays where it was");
        }
    }
    fireWorldMapUpdate(L);
    return 0;
}

// GetMapContinents() → "Kalimdor", "Eastern Kingdoms", ...
static int lua_GetMapContinents(lua_State* L) {
    if (auto* svc = getLuaServices(L)) {
        if (svc->getMapContinentNames) {
            const auto names = svc->getMapContinentNames();
            if (!names.empty() &&
                lua_checkstack(L, static_cast<int>(names.size()))) {
                for (const auto& n : names) lua_pushstring(L, n.c_str());
                return static_cast<int>(names.size());
            }
        }
    }
    lua_pushstring(L, "Kalimdor");
    lua_pushstring(L, "Eastern Kingdoms");
    lua_pushstring(L, "Outland");
    lua_pushstring(L, "Northrend");
    return 4;
}

// GetMapZones(continent) → the zones on that continent, in dropdown order
//
// Four made-up names per continent before this - "a minimal representative
// set", which is a list nobody's zone is on. The row picked from it was then
// handed to SetMapZoom as the zone to show, so the dropdown offered four
// zones out of dozens and choosing one of them did nothing anyway.
static int lua_GetMapZones(lua_State* L) {
    const int cont = static_cast<int>(luaL_checknumber(L, 1));
    auto* zonesSvc = getLuaServices(L);
    if (!zonesSvc || !zonesSvc->getMapZoneNames) {
        LOG_WARNING("GetMapZones(", cont, "): no map to ask, so the zone"
                    " dropdown is built empty");
    }
    if (auto* svc = zonesSvc) {
        if (svc->getMapZoneNames) {
            const auto names = svc->getMapZoneNames(cont);
            // Lua promises a C function twenty free slots and no more. Four
            // hard-coded names never came near it; a continent's real zone
            // list is dozens, and pushing them unasked writes past the stack.
            if (!names.empty() &&
                !lua_checkstack(L, static_cast<int>(names.size()))) {
                LOG_WARNING("GetMapZones(", cont, "): no room on the Lua stack"
                            " for ", names.size(), " zones, so the dropdown is"
                            " built empty");
                return 0;
            }
            // An empty list is a blank zone dropdown and no other trace. The
            // continent may be one this client has no zone data for, or the
            // map data may not be loaded yet - either way the dropdown that
            // opens is the same empty one.
            if (names.empty()) {
                LOG_WARNING("GetMapZones(", cont, "): the map lists no zones on"
                            " that continent, so the zone dropdown is empty");
            }
            for (const auto& n : names) lua_pushstring(L, n.c_str());
            return static_cast<int>(names.size());
        }
    }
    return 0;
}

// GetNumMapLandmarks() → 0 (no landmark data exposed yet)
static int lua_GetNumMapLandmarks(lua_State* L) {
    auto* svc = getLuaServices(L);
    lua_pushnumber(L, (svc && svc->getMapLandmarks)
        ? static_cast<lua_Number>(svc->getMapLandmarks().size()) : 0);
    return 1;
}

/// GetMapLandmarkInfo(i) → name, description, textureIndex, x, y, mapLinkID
///
/// The area POIs on whatever the map is showing. Answered zero of them while
/// the client had them loaded from AreaPOI.dbc and was drawing them on its own
/// map, so FrameXML's map came up with no pins on it at all.
static int lua_GetMapLandmarkInfo(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!svc || !svc->getMapLandmarks || index < 1) return luaReturnNil(L);
    const auto marks = svc->getMapLandmarks();
    if (index > static_cast<int>(marks.size())) return luaReturnNil(L);
    const auto& m = marks[static_cast<size_t>(index) - 1];

    lua_pushstring(L, m.name.c_str());          // 1: name
    lua_pushstring(L, m.description.c_str());   // 2: description
    // Which cell of the POI icon atlas to draw. AreaPOI's icon type is that
    // index in the client's own drawing too.
    lua_pushnumber(L, m.icon);                  // 3: textureIndex
    lua_pushnumber(L, m.x);                     // 4: x across the map
    lua_pushnumber(L, m.y);                     // 5: y down the map
    // A landmark that leads to another map. Nothing here links one to another,
    // and nil is what a plain point of interest answers.
    lua_pushnil(L);                             // 6: mapLinkID
    // Whether the battlefield minimap shows it too. Seven values, not six:
    // blizzard_battlefieldminimap.lua gates every pin on this one, so a nil
    // here is a battle map with no points of interest on it at all. AreaPOI
    // carries no such flag, and a landmark worth drawing on the world map is
    // worth drawing on the smaller one.
    lua_pushboolean(L, 1);                      // 7: showInBattleMap
    return 7;
}

/// GetNumMapOverlays() / GetMapOverlayInfo(i)
///
/// The explored-area art: one entry per overlay, each a texture prefix and the
/// pixel rectangle it occupies on the zone map. The interface slices it into
/// 256-pixel tiles named prefix1, prefix2 and so on, which is the same
/// arrangement the client's own map renderer reads.
///
/// The count answered zero, so the loop that builds them never ran once and
/// the map was drawn with nothing on it but its background.
static int lua_GetNumMapOverlays(lua_State* L) {
    auto* svc = getLuaServices(L);
    lua_pushnumber(L, (svc && svc->getMapOverlays)
        ? static_cast<lua_Number>(svc->getMapOverlays().size()) : 0);
    return 1;
}

static int lua_GetMapOverlayInfo(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!svc || !svc->getMapOverlays || index < 1) return luaReturnNil(L);
    const auto overlays = svc->getMapOverlays();
    if (index > static_cast<int>(overlays.size())) return luaReturnNil(L);
    const auto& o = overlays[static_cast<size_t>(index) - 1];

    lua_pushstring(L, o.texture.c_str());   // 1: texture prefix
    lua_pushnumber(L, o.width);             // 2: width in pixels
    lua_pushnumber(L, o.height);            // 3: height in pixels
    lua_pushnumber(L, o.offsetX);           // 4: x offset on the zone map
    lua_pushnumber(L, o.offsetY);           // 5: y offset
    // The "map point" pair, which WorldMapFrame_Update reads and never uses.
    lua_pushnumber(L, 0);                   // 6
    lua_pushnumber(L, 0);                   // 7
    return 7;
}

/// GetTrackingTexture() → the icon for what the minimap is tracking, or nil.
///
/// Nothing is tracked here: tracking is a spell effect this client does not
/// model, and GetNumTrackingTypes already answers none. Said explicitly
/// because the missing-API fallback answers with an object, and an object is
/// not nil - MiniMapTrackingIcon:SetTexture(GetTrackingTexture()) would then
/// be handed a table where a path belongs and the button would show the
/// tracking icon for a tracking type that does not exist.
// ── Minimap tracking ───────────────────────────────────────────────────────
//
// The tracking menu is not a fixed list. It is whatever tracking the player
// has learned, which is the known spells applying aura 44 (creatures) or 45
// (resources) - Spell.dbc's EffectApplyAuraName, and the only thing in the
// spell data that tells a tracking spell from any other buff. The effect id
// beside it says an aura is applied but never which one.
//
// All three of these used to be stubs: no texture, no types, and no binding
// at all for the count. GetNumTrackingTypes being absent was the worst of
// them, because a missing global answers nil and the initialiser opens with
// `for id = 1, count`, which raises on nil and took the whole menu down.

/// Aura ids that make a spell a tracking spell.
/// The spell possessing the player, or zero.
///
/// Which aura it is decides what the possess bar's cancel button cancels, so
/// this selects on the aura *type* rather than on anything looser: only a
/// spell that applies one of the four possession auras can be returned, and if
/// none is found the caller shows no bar rather than a button that would
/// cancel something else. Values from AzerothCore's SpellAuraDefines.
static uint32_t possessAuraSpellId(wowee::game::GameHandler* gh) {
    if (!gh) return 0;
    constexpr uint32_t kModPossess = 2;
    constexpr uint32_t kModCharm = 6;
    constexpr uint32_t kModPossessPet = 128;
    constexpr uint32_t kAoeCharm = 177;
    for (const auto& aura : gh->getPlayerAuras()) {
        if (aura.isEmpty()) continue;
        // Asking for the name is what fills the cache; the entry is not there
        // to be read until something has.
        gh->getSpellName(aura.spellId);
        auto it = gh->spellNameCacheRef().find(aura.spellId);
        if (it == gh->spellNameCacheRef().end()) continue;
        for (uint32_t type : it->second.effectAuraIds) {
            if (type == kModPossess || type == kModCharm ||
                type == kModPossessPet || type == kAoeCharm) {
                return aura.spellId;
            }
        }
    }
    return 0;
}

static constexpr uint32_t kAuraTrackCreatures = 44;
static constexpr uint32_t kAuraTrackResources = 45;

/// The player's tracking spells, ordered by name.
///
/// Sorted rather than left in the known-spell set's own order, which is a hash
/// order: the menu would otherwise list the same spells differently from one
/// session to the next.
static std::vector<uint32_t> trackingSpells(game::GameHandler* gh) {
    std::vector<uint32_t> out;
    if (!gh) return out;
    for (uint32_t sid : gh->getKnownSpells()) {
        // Asking for the name is what fills the cache; the entry is not there
        // to be read until something has.
        gh->getSpellName(sid);
        auto it = gh->spellNameCacheRef().find(sid);
        if (it == gh->spellNameCacheRef().end()) continue;
        for (uint32_t aura : it->second.effectAuraIds) {
            if (aura == kAuraTrackCreatures || aura == kAuraTrackResources) {
                out.push_back(sid);
                break;
            }
        }
    }
    std::sort(out.begin(), out.end(), [gh](uint32_t a, uint32_t b) {
        return gh->getSpellName(a) < gh->getSpellName(b);
    });
    return out;
}

/// Whether that tracking is the one currently running.
static bool trackingActive(game::GameHandler* gh, uint32_t spellId) {
    if (!gh) return false;
    for (const auto& aura : gh->getPlayerAuras()) {
        if (aura.spellId == spellId) return true;
    }
    return false;
}

/// GetTrackingTexture() → the icon on the minimap button.
///
/// The button's own art is empty in the XML and comes entirely from here, so
/// answering nil left a blank square on the minimap. Nothing tracked is not
/// nothing to draw: it is the magnifying glass, which is what a stock client
/// shows and what makes the button look like something to click.
static int lua_GetTrackingTexture(lua_State* L) {
    auto* gh = getGameHandler(L);
    for (uint32_t sid : trackingSpells(gh)) {
        if (!trackingActive(gh, sid)) continue;
        const std::string icon = gh->getSpellIconPath(sid);
        if (!icon.empty()) { lua_pushstring(L, icon.c_str()); return 1; }
    }
    lua_pushstring(L, "Interface\\Minimap\\Tracking\\None");
    return 1;
}

/// GetNumTrackingTypes() → how many the player knows.
static int lua_GetNumTrackingTypes(lua_State* L) {
    lua_pushnumber(L, static_cast<lua_Number>(trackingSpells(getGameHandler(L)).size()));
    return 1;
}

/// GetTrackingInfo(index) → name, texture, active, category.
///
/// "spell" for the category, because these are spell icons and the menu uses
/// that to crop the icon's border - the same trim the action bar gives them.
static int lua_GetTrackingInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto spells = trackingSpells(gh);
    if (index < 1 || index > static_cast<int>(spells.size())) return 0;
    const uint32_t sid = spells[static_cast<size_t>(index - 1)];
    lua_pushstring(L, gh->getSpellName(sid).c_str());
    lua_pushstring(L, gh->getSpellIconPath(sid).c_str());
    lua_pushboolean(L, trackingActive(gh, sid) ? 1 : 0);
    lua_pushstring(L, "spell");
    return 4;
}

/// SetTracking(index) - casting the spell is how tracking is turned on; there
/// is no separate message for it. A nil index is the menu's "None" entry,
/// which in a stock client cancels the running tracking aura. Cancelling a
/// player's own buff is not wired up here, so that entry does nothing rather
/// than casting something arbitrary.
static int lua_SetTracking(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || lua_isnoneornil(L, 1)) return 0;
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto spells = trackingSpells(gh);
    if (index < 1 || index > static_cast<int>(spells.size())) return 0;
    gh->castSpell(spells[static_cast<size_t>(index - 1)], 0);
    return 0;
}


static int lua_GetGameTime(lua_State* L) {
    // Returns server game time as hours, minutes
    auto* gh = getGameHandler(L);
    float gt = gh ? gh->getGameTime() : -1.0f;
    if (gt < 0.0f) {
        // The server has not said yet. Answer with the local clock rather than
        // a negative hour, which is what the interface's own clock would then
        // print - and it is the same clock the sky falls back to, so the two
        // agree until the server's time arrives.
        const std::time_t now = std::time(nullptr);
        const std::tm lt = core::localTime(now);
        gt = static_cast<float>(lt.tm_hour) + static_cast<float>(lt.tm_min) / 60.0f;
    }
    const int hours = static_cast<int>(gt) % 24;
    const int mins = static_cast<int>((gt - static_cast<int>(gt)) * 60.0f);
    lua_pushnumber(L, hours);
    lua_pushnumber(L, mins);
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
        // Seven, and a difficulty of one, so this branch answers in the same
        // shape and the same numbering as the real one below it.
        lua_pushstring(L, ""); lua_pushstring(L, "none"); lua_pushnumber(L, 1);
        lua_pushstring(L, "Normal"); lua_pushnumber(L, 0);
        lua_pushnumber(L, 0); lua_pushboolean(L, 0);
        return 7;
    }
    std::string mapName = gh->getMapName(gh->getCurrentMapId());
    const uint32_t diff = gh->getInstanceDifficulty();
    lua_pushstring(L, mapName.c_str());                    // 1: name
    lua_pushstring(L, gh->isInInstance() ? "party" : "none"); // 2: instanceType
    // Counted from one, which is what the interface compares against.
    //
    // The wire value is zero-based - social_handler reads heroic as
    // difficulty == 1 - and this pushed it straight through. minimap.lua tests
    // `difficulty == 1 and maxPlayers == 5` to decide there is nothing worth
    // showing, and `difficulty == 2` for heroic, so a normal dungeon failed
    // the first test and hung a difficulty banner on the minimap, while a
    // heroic failed the second and had that banner read "Normal".
    //
    // GetInstanceDifficulty beside this already added the one; only this path
    // did not.
    lua_pushnumber(L, diff + 1);                           // 3: difficultyIndex
    const char* diffName = game::instanceDifficultyName(diff);
    lua_pushstring(L, diffName ? diffName : "Normal");      // 4: difficultyName
    lua_pushnumber(L, 5);                                   // 5: maxPlayers (default 5-man)
    // The two the raid branch reads. Neither is tracked here, and both are
    // only consulted for a dynamic-difficulty raid - but nil reaches
    // `playerDifficulty == 1` and `if ( isDynamicInstance )` in minimap.lua,
    // and the interface unpacks all seven on one line.
    lua_pushnumber(L, 0);                                   // 6: playerDifficulty
    lua_pushboolean(L, 0);                                  // 7: isDynamicInstance
    return 7;
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
    // than raising - so the room is asked for before any of it is used.
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

// strtrim(str) - remove leading/trailing whitespace
static int lua_strtrim(lua_State* L) {
    const char* str = luaL_checkstring(L, 1);
    std::string s(str);
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    lua_pushstring(L, (start == std::string::npos) ? "" : s.substr(start, end - start + 1).c_str());
    return 1;
}

/// strlenutf8(s) - the number of characters, where string.len counts bytes.
///
/// Unbound, it answered nil through the fallback, and both callers do
/// arithmetic on the result rather than checking it: autocomplete.lua and
/// chatframe.lua compute an offset as
/// `GetUTF8CursorPosition() - strlenutf8(command) - 1`. A nil there raises, so
/// typing a slash command or a player name took chat autocomplete down.
static int lua_strlenutf8(lua_State* L) {
    size_t len = 0;
    const char* s = luaL_optlstring(L, 1, "", &len);
    int chars = 0;
    for (size_t i = 0; i < len; ++i) {
        // A continuation byte is 10xxxxxx; every other byte opens a character.
        if ((static_cast<unsigned char>(s[i]) & 0xC0) != 0x80) ++chars;
    }
    lua_pushnumber(L, chars);
    return 1;
}

// wipe(table) - clear all entries from a table
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

// date(format) - safe date function (os.date was removed)
/// date(format, time) - the clock, as WoW exposes it.
///
/// Both shapes FrameXML uses: "*t" for a table of parts, a strftime string
/// otherwise, and an optional timestamp. Formatting "*t" as a strftime string
/// yields the literal "*t", which is what BetterDate then indexed for an hour.
static int lua_wow_date(lua_State* L) {
    const char* fmt = luaL_optstring(L, 1, "%c");
    const std::time_t when = lua_isnumber(L, 2)
        ? static_cast<std::time_t>(lua_tonumber(L, 2))
        : std::time(nullptr);

    std::tm parts{};
    parts = core::localTime(when);

    if (std::strcmp(fmt, "*t") == 0 || std::strcmp(fmt, "!*t") == 0) {
        lua_newtable(L);
        auto set = [&](const char* key, int value) {
            lua_pushinteger(L, value);
            lua_setfield(L, -2, key);
        };
        set("year", parts.tm_year + 1900);
        set("month", parts.tm_mon + 1);
        set("day", parts.tm_mday);
        set("hour", parts.tm_hour);
        set("min", parts.tm_min);
        set("sec", parts.tm_sec);
        set("wday", parts.tm_wday + 1);
        set("yday", parts.tm_yday + 1);
        lua_pushboolean(L, parts.tm_isdst > 0);
        lua_setfield(L, -2, "isdst");
        return 1;
    }

    char out[256];
    const size_t n = std::strftime(out, sizeof(out), fmt, &parts);
    lua_pushlstring(L, out, n);
    return 1;
}

// time() - current unix timestamp
static int lua_wow_time(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(time(nullptr)));
    return 1;
}

// GetTime() - returns elapsed seconds since engine start (shared epoch)
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
// looks like - no threat warnings, no runes, nobody to pass loot to - so the
// caller takes the branch it would take on a client where that feature is off,
// rather than dividing by a nil.
static int lua_ReturnFalse(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_ReturnTrue(lua_State* L)  { lua_pushboolean(L, 1); return 1; }
static int lua_ReturnNil(lua_State* L)   { lua_pushnil(L); return 1; }
static int lua_ReturnZero(lua_State* L)  { lua_pushnumber(L, 0.0); return 1; }

/// Which row the skill list has selected, which is UI state rather than
/// anything the game knows. Same shape as selectedFriend and selectedIgnore.
static int& selectedSkill() { static int v = 0; return v; }
static int lua_ReturnNothing(lua_State*) { return 0; }

/// Which channel the channel-list panel has highlighted. Panel state with no
/// counterpart in the game, kept here so the getter and setter agree.
static int& selectedDisplayChannel() { static int selected = 0; return selected; }

/// Which of the four extra action bars are switched on: bottom-left,
/// bottom-right, right and the second right, in that order.
///
/// The real client mirrors these from the server's account data. Here they are
/// a preference with nowhere else to live, so they are kept beside the pair of
/// calls that read and write them - the setter used to accept them and forget,
/// which left the getter with nothing to answer.
static std::array<bool, 4>& actionBarToggles() {
    static std::array<bool, 4> shown{};
    return shown;
}


/// A cooldown that is not running: start and duration both zero. Two values,
/// because the caller adds them together on the next line -
/// local start, duration = GetSummonFriendCooldown(); start + duration - and
/// one of them missing is arithmetic on nil.
/// The resolutions this client offers, as "WIDTHxHEIGHT" strings, and which of
/// them is current. One entry - the window as it actually is - because this
/// client does not enumerate modes.
///
/// UpdateMenuBarTop reads them together and immediately divides:
///   string.match((({GetScreenResolutions()})[GetCurrentResolution()] or ""),
///                "(%d+).-(%d+)")
/// then tonumber(width) / tonumber(height). An empty list makes both nil.
// GetScreenResolutions() → every mode, as "WxH", one return value each.
//
// It used to answer the window's current size and nothing else, so the video
// panel's dropdown had a single row and there was nothing to pick. The list is
// the one this client's own settings panel offers, shared rather than written
// twice, because the dropdown carries a *position* in it: it hands that index
// straight back to SetScreenResolution.
static int lua_GetScreenResolutions(lua_State* L) {
    for (int i = 0; i < ui::kNumDisplayResolutions; ++i)
        lua_pushstring(L, ui::displayResolutionLabel(i).c_str());
    return ui::kNumDisplayResolutions;
}

// GetCurrentResolution() → which of those the window is, counted from one.
static int lua_GetCurrentResolution(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int idx = (svc && svc->getResolutionIndex) ? svc->getResolutionIndex() : 0;
    lua_pushnumber(L, idx + 1);
    return 1;
}

// SetScreenResolution(index) - the dropdown's row, counted from one.
//
// A no-op before this, which is why the comment beside RestoreVideoResolutionDefaults
// could say nothing above it was settable. Both records are moved together, so
// this client's own options do not show the size it had before.
/// The four anti-aliasing modes, in the shape the video panel reads them.
///
/// VideoOptionsResolutionPanel_GetMultisampleFormats walks its varargs in
/// threes - colour bits, depth bits, sample count - and formats each into
/// MULTISAMPLING_FORMAT_STRING. The colour and depth numbers are this client's
/// swapchain and depth buffer, which do not change with the mode; only the
/// sample count does, and those are the same four the client's own
/// Anti-aliasing setting offers.
static int lua_GetMultisampleFormats(lua_State* L) {
    static constexpr int kSamples[] = {1, 2, 4, 8};
    for (int samples : kSamples) {
        lua_pushnumber(L, 32);        // colour bits
        lua_pushnumber(L, 24);        // depth bits
        lua_pushnumber(L, samples);
    }
    return 3 * static_cast<int>(sizeof(kSamples) / sizeof(kSamples[0]));
}

/// The row of the list above that is in force, counted from one.
static int lua_GetCurrentMultisampleFormat(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int row = (svc && svc->getAntiAliasingIndex) ? svc->getAntiAliasingIndex() : 0;
    lua_pushnumber(L, std::clamp(row, 0, 3) + 1);
    return 1;
}

/// SetMultisampleFormat(row) - the dropdown's row, counted from one.
static int lua_SetMultisampleFormat(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int row = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (svc && svc->setAntiAliasingIndex && row >= 1) svc->setAntiAliasingIndex(row - 1);
    return 0;
}

/// GetRefreshRates() - one zero, which is this API's own word for "none".
///
/// Not nothing. VideoOptionsResolutionPanel_GetRefreshRates tests for exactly
/// one argument equal to zero and, finding it, disables the dropdown and greys
/// its label and text - which is what the real client does in windowed mode,
/// where the desktop owns the refresh rate. Returning nothing instead runs its
/// loop zero times, and the control was left blank, enabled and clickable with
/// nothing behind it.
///
/// The general rule that a list-returning function says "none" by returning
/// nothing still holds; this one is a caller that defined its own sentinel and
/// checks for it first.
/// The sound output drivers the Sound panel can offer, which is one.
///
/// AudioOptionsSoundPanelHardwareDropDown_Initialize loops from 0 to num-1 and
/// adds a button per driver. With num zero it added none, and an empty
/// dropdown does not draw empty - UIDropDownMenu_Refresh walks the shared
/// DropDownList1 buttons, so it reads whichever list was built last and takes
/// its text. The same shape that put a screen resolution in the Multisampling
/// box put someone else's answer in Game Sound Output.
///
/// This client opens whichever playback device the system offers and does not
/// switch between them, so the list is that one device by name - the truth
/// rather than a stub, and the shape GetScreenResolutions already takes.
static int lua_Sound_GetNumOutputDrivers(lua_State* L) {
    lua_pushnumber(L, audio::AudioEngine::instance().isInitialized() ? 1 : 0);
    return 1;
}

/// The driver at that index, counted from zero as the panel counts it.
static int lua_Sound_GetOutputDriverNameByIndex(lua_State* L) {
    const int index = static_cast<int>(luaL_optnumber(L, 1, -1));
    std::string name = audio::AudioEngine::instance().getOutputDeviceName();
    if (index != 0 || name.empty()) { lua_pushnil(L); return 1; }
    lua_pushstring(L, name.c_str());
    return 1;
}

static int lua_GetRefreshRates(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

static int lua_SetScreenResolution(lua_State* L) {
    auto* svc = getLuaServices(L);
    const int row = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (svc && svc->setResolutionIndex && row >= 1) svc->setResolutionIndex(row - 1);
    return 0;
}

/// The battleground position list carries two kinds of entry, and the packet
/// says which by the block it arrived in.
///
/// AzerothCore's writer sends `m_numPlayerPositions` first - a count it always
/// writes as zero, with a commented-out loop beside it - and then the flag
/// carriers. So group 0 is the team positions, which no AzerothCore realm
/// sends, and group 1 is the carriers. The parser has recorded which all along
/// and the first version of these accessors ignored it, which would have drawn
/// flag carriers as ordinary party dots.
static const game::BgPlayerPosition* bgEntryAt(game::GameHandler* gh,
                                               int group, int index) {
    if (!gh || index < 1) return nullptr;
    int seen = 0;
    for (const auto& p : gh->getBgPlayerPositions()) {
        if (p.group != group) continue;
        if (++seen == index) return &p;
    }
    return nullptr;
}

static int bgCount(game::GameHandler* gh, int group) {
    if (!gh) return 0;
    int n = 0;
    for (const auto& p : gh->getBgPlayerPositions()) if (p.group == group) ++n;
    return n;
}

/// GetBattlefieldPosition(index) → where a team mate is, as a fraction across
/// the map, and their name.
///
/// Origin and no name past the end, because WorldMapFrame_Update's loop is
/// bounded by MAX_RAID_MEMBERS rather than by how many are present and hides
/// each frame whose position comes back as zero. On an AzerothCore realm that
/// is every one of them: the server writes the count and never the entries.
static int lua_GetBattlefieldPosition(lua_State* L) {
    auto* gh = getGameHandler(L);
    auto* svc = getLuaServices(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    float u = 0.0f, v = 0.0f;
    if (const auto* p = bgEntryAt(gh, 0, index)) {
        if (svc && svc->mapUVForWorldPos &&
            svc->mapUVForWorldPos(p->wowX, p->wowY, 0.0f, u, v)) {
            lua_pushnumber(L, u);
            lua_pushnumber(L, v);
            lua_pushstring(L, gh->lookupName(p->guid).c_str());
            return 3;
        }
    }
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    lua_pushstring(L, "");
    return 3;
}

/// GetBattlefieldFlagPosition(index) → where a carried flag is, and which flag.
///
/// The third value names a texture under Interface\\WorldStateFrame, so a
/// wrong one is not a missing icon but the enemy's flag drawn over your own.
/// The server writes the alliance carrier and then the horde one, skipping
/// whichever is not held - so with both carried the order says which is which,
/// and with one carried it does not. Rather than guess, that case answers zero
/// and the frame hides: no flag on the map is a smaller lie than the wrong one.
static int lua_GetBattlefieldFlagPosition(lua_State* L) {
    auto* gh = getGameHandler(L);
    auto* svc = getLuaServices(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    float u = 0.0f, v = 0.0f;
    const auto* p = bgEntryAt(gh, 1, index);
    if (p && bgCount(gh, 1) == 2 && svc && svc->mapUVForWorldPos &&
        svc->mapUVForWorldPos(p->wowX, p->wowY, 0.0f, u, v)) {
        lua_pushnumber(L, u);
        lua_pushnumber(L, v);
        lua_pushstring(L, index == 1 ? "AllianceFlag" : "HordeFlag");
        return 3;
    }
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    lua_pushstring(L, "");
    return 3;
}

// ---- Chat window settings ----
//
// The three getters here answered with fixed numbers and every setter was
// missing, so the interface could read a chat window's settings and never save
// one. Moving a window, resizing it, recolouring it or closing it all went
// through a setter that did nothing, and the next FCF_LoadChatSettings read
// back the same defaults it read the first time.
//
// This is entirely client-side - WoW keeps it in the config, not on the server
// - so a store here is the whole feature rather than a stand-in for one.
struct ChatWindowSettings {
    std::string name;
    float fontSize = 14.0f;
    // Black at a quarter alpha, which is what the interface seeds a fresh
    // window with: DEFAULT_CHATFRAME_COLOR is {0,0,0} and
    // DEFAULT_CHATFRAME_ALPHA is 0.25, both in floatingchatframe.lua.
    //
    // These were white at full alpha. FCF_LoadChatSettings hands whatever is
    // here to FCF_SetWindowColor and FCF_SetWindowAlpha for every window that
    // has no saved settings of its own, so the chat opened behind an opaque
    // white panel with the text on it. Installs that ran such a build saved
    // that white, and loadInterfaceState repairs it on the way back in.
    float r = kChatBackgroundDefault[0], g = kChatBackgroundDefault[1];
    float b = kChatBackgroundDefault[2], alpha = kChatBackgroundDefault[3];
    bool shown = false;
    bool locked = false;
    int  docked = 0;            // 0 = not docked; otherwise its place on the dock
    bool uninteractable = false;
    // Saved geometry. Absent until something saves it, which is not the same as
    // zero: FCF_RestorePositionAndDimensions only restores what it is given,
    // and a zero width is a window with no width.
    bool hasPosition = false;
    std::string point = "TOPLEFT";
    float xOffset = 0.0f, yOffset = 0.0f;
    bool hasDimensions = false;
    float width = 0.0f, height = 0.0f;
    /// Which of ChatTypeGroup's names this window shows, and which chat
    /// channels it carries. Both are read once per login and are the whole of
    /// what a chat window listens to: ChatFrame_RegisterForMessages walks the
    /// first list and calls RegisterEvent for every event in each group, and
    /// nothing else in FrameXML registers a chat frame for a CHAT_MSG_ event.
    std::vector<std::string> messageGroups;
    std::vector<std::pair<std::string, int>> channels;
};

/// Every group name ChatTypeGroup defines in 3.3.5, which is what the General
/// window shows by default - say and yell through to loot, experience and the
/// error line. Names FrameXML does not know are skipped by the reader rather
/// than raising, so the cost of listing one too many is nothing and the cost of
/// missing one is that kind of message never appearing.
inline constexpr const char* kDefaultChatGroups[] = {
    "SYSTEM", "SAY", "EMOTE", "YELL", "WHISPER", "PARTY", "PARTY_LEADER",
    "RAID", "RAID_LEADER", "RAID_WARNING", "BATTLEGROUND",
    "BATTLEGROUND_LEADER", "GUILD", "OFFICER", "MONSTER_SAY", "MONSTER_YELL",
    "MONSTER_EMOTE", "MONSTER_WHISPER", "MONSTER_BOSS_EMOTE",
    "MONSTER_BOSS_WHISPER", "ERRORS", "AFK", "DND", "IGNORED", "BG_HORDE",
    "BG_ALLIANCE", "BG_NEUTRAL", "COMBAT_XP_GAIN", "COMBAT_HONOR_GAIN",
    "COMBAT_FACTION_CHANGE", "SKILL", "LOOT", "MONEY", "OPENING",
    "TRADESKILLS", "PET_INFO", "COMBAT_MISC_INFO", "ACHIEVEMENT",
    "GUILD_ACHIEVEMENT", "CHANNEL", "TARGETICONS",
};

/// NUM_CHAT_WINDOWS in 3.3.5. Indices are one-based from Lua.
static constexpr int kNumChatWindows = 10;

static std::array<ChatWindowSettings, kNumChatWindows>& chatWindows() {
    static std::array<ChatWindowSettings, kNumChatWindows> windows = [] {
        std::array<ChatWindowSettings, kNumChatWindows> w{};
        // WoW's default layout docks General and the combat log and leaves the
        // rest neither shown nor docked. Which window is being asked about
        // matters: docked is that window's place on the dock, and FCF_DockFrame
        // asserts that whatever claims position one is the dock's primary, so
        // answering one for every window claimed each was first.
        w[0].shown = true; w[0].docked = 1;
        w[1].shown = true; w[1].docked = 2;
        // General carries everything; the combat log window is driven by
        // Blizzard_CombatLog and registers its own events, so its list is
        // empty here exactly as it is in the real client.
        for (const char* g : kDefaultChatGroups) w[0].messageGroups.emplace_back(g);
        // The channels the default layout carries. FrameXML never adds one by
        // itself - ChatFrame_RegisterForChannels reads this list and nothing
        // else fills it - so with it empty every channel line was matched
        // against nothing and dropped, however well the message parsed.
        //
        // Names without the zone after them, which is what the frame compares:
        // the message carries "General - Blasted Lands" and the short name
        // beside it, and the match is on the short one. Zero for the zone id,
        // which this client does not learn; the name settles it.
        for (const char* c : {"General", "Trade", "LocalDefense",
                              "LookingForGroup", "GuildRecruitment"}) {
            w[0].channels.emplace_back(c, 0);
        }
        return w;
    }();
    return windows;
}

/// Where the chat layout and the action-bar toggles live between runs.
///
/// The CVars got their own file first and these two stores were left in memory,
/// which is the same defect twice: a player who moves a chat window, renames a
/// tab, sends whispers to their own window or turns on the right-hand action
/// bars finds all of it back to default on the next login. Nothing about that
/// looks like a bug - it looks like the setting never took.
///
/// One file for both, because they are the same kind of thing: interface state
/// the real client keeps in account data this one has no equivalent of.
static std::string interfaceStatePath() {
    return core::getConfigRoot() + "/interface_state.cfg";
}

static void saveInterfaceState();

/// The window a one-based index names, or nullptr if it names none.
static ChatWindowSettings* chatWindow(lua_State* L, int argIndex) {
    const int id = static_cast<int>(luaL_optnumber(L, argIndex, 0));
    if (id < 1 || id > kNumChatWindows) return nullptr;
    return &chatWindows()[static_cast<size_t>(id - 1)];
}

/// Write the chat layout and the bar toggles out.
///
/// Sorted and one key per line, the same shape as the CVar file beside it, so
/// the two read alike and a diff shows a settings change rather than hash order
/// moving. A name with a newline in it is dropped rather than written, since it
/// would come back as two broken lines.
static void saveInterfaceState() {
    const std::string path = interfaceStatePath();
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);
    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save interface state to ", path);
        return;
    }
    const auto& bars = actionBarToggles();
    for (size_t i = 0; i < bars.size(); ++i)
        out << "bar." << (i + 1) << "=" << (bars[i] ? 1 : 0) << "\n";
    const auto& windows = chatWindows();
    for (size_t i = 0; i < windows.size(); ++i) {
        const auto& w = windows[i];
        const std::string k = "chat." + std::to_string(i + 1) + ".";
        if (w.name.find('\n') == std::string::npos)
            out << k << "name=" << w.name << "\n";
        out << k << "shown="   << (w.shown ? 1 : 0) << "\n";
        out << k << "locked="  << (w.locked ? 1 : 0) << "\n";
        out << k << "docked="  << w.docked << "\n";
        out << k << "uninteractable=" << (w.uninteractable ? 1 : 0) << "\n";
        out << k << "fontSize=" << w.fontSize << "\n";
        out << k << "colour=" << w.r << "," << w.g << "," << w.b << "," << w.alpha << "\n";
        // Position and dimensions only once something has saved them. Absent is
        // not zero: FCF_RestorePositionAndDimensions restores what it is given,
        // and a zero width is a window with no width.
        if (w.hasPosition)
            out << k << "position=" << w.point << "," << w.xOffset << "," << w.yOffset << "\n";
        if (w.hasDimensions)
            out << k << "size=" << w.width << "," << w.height << "\n";
        // Written even when empty, because empty is a real setting: a window
        // whose groups were all removed must come back empty rather than
        // falling to the defaults and refilling itself on the next login.
        std::string groups;
        for (const auto& g : w.messageGroups) {
            if (g.find_first_of(",\n") != std::string::npos) continue;
            if (!groups.empty()) groups += ",";
            groups += g;
        }
        out << k << "groups=" << groups << "\n";
        std::string chans;
        for (const auto& c : w.channels) {
            if (c.first.find_first_of(",:\n") != std::string::npos) continue;
            if (!chans.empty()) chans += ",";
            chans += c.first + ":" + std::to_string(c.second);
        }
        out << k << "channels=" << chans << "\n";
    }
}

/// Read them back, before the interface loads: a chat window is built from
/// these as it is created, and a value arriving later leaves the window
/// disagreeing with the setting.
static void loadInterfaceState() {
    std::ifstream in(interfaceStatePath());
    if (!in.is_open()) return;
    auto& bars = actionBarToggles();
    auto& windows = chatWindows();
    std::string line;
    size_t loaded = 0;
    bool repaired = false;
    while (std::getline(in, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos || eq == 0) continue;
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        auto num = [&value] {
            try { return std::stof(value); } catch (const std::exception&) { return 0.0f; }
        };
        if (key.rfind("bar.", 0) == 0) {
            const int idx = std::atoi(key.c_str() + 4);
            if (idx >= 1 && idx <= static_cast<int>(bars.size()))
                bars[static_cast<size_t>(idx - 1)] = (value == "1");
            ++loaded;
            continue;
        }
        if (key.rfind("chat.", 0) != 0) continue;
        const size_t dot = key.find('.', 5);
        if (dot == std::string::npos) continue;
        const int idx = std::atoi(key.substr(5, dot - 5).c_str());
        if (idx < 1 || idx > kNumChatWindows) continue;
        auto& w = windows[static_cast<size_t>(idx - 1)];
        const std::string field = key.substr(dot + 1);
        if      (field == "name")     w.name = value;
        else if (field == "shown")    w.shown = (value == "1");
        else if (field == "locked")   w.locked = (value == "1");
        else if (field == "docked")   w.docked = std::atoi(value.c_str());
        else if (field == "uninteractable") w.uninteractable = (value == "1");
        else if (field == "fontSize") {
            // Only a height text can be read at. num() answers 0 for anything
            // it cannot parse, and a zero here reaches SetFont as the size of
            // every line in the window. The interface offers 12 through 18.
            const float size = num();
            if (std::isfinite(size) && size >= 6.0f && size <= 32.0f) w.fontSize = size;
        }
        else if (field == "colour") {
            // Read into locals and only kept if all four are numbers, for the
            // same reason position and size below do it: this file is written
            // back out verbatim, so a nan taken in here is a nan saved again
            // every session after. On a miss the window keeps the fresh-window
            // colour the struct already holds.
            float cr = 0.0f, cg = 0.0f, cb = 0.0f, ca = 0.0f;
            if (std::sscanf(value.c_str(), "%f,%f,%f,%f", &cr, &cg, &cb, &ca) == 4 &&
                std::isfinite(cr) && std::isfinite(cg) && std::isfinite(cb) &&
                std::isfinite(ca)) {
                // A white panel is the one this client wrote before it had the
                // interface's own default, not one anybody picked, and at full
                // alpha it covers the chat with the text lost in it.
                if (repairChatBackground(cr, cg, cb, ca)) {
                    LOG_WARNING("Chat window ", idx,
                                " had a white background saved, which hides the "
                                "text; restoring the default.");
                    repaired = true;
                }
                w.r = cr;
                w.g = cg;
                w.b = cb;
                w.alpha = ca;
            }
        } else if (field == "position") {
            char pt[32] = {0};
            float px = 0.0f, py = 0.0f;
            // Read into locals and only kept if they are numbers. A file
            // written before this was refused on the way out still holds a
            // nan, and taking it back would restore the frame that swallows
            // every hit test - see lua_SetChatWindowSavedPosition.
            if (std::sscanf(value.c_str(), "%31[^,],%f,%f", pt, &px, &py) == 3 &&
                std::isfinite(px) && std::isfinite(py)) {
                w.point = pt;
                w.xOffset = px;
                w.yOffset = py;
                w.hasPosition = true;
            }
        } else if (field == "size") {
            float pw = 0.0f, ph = 0.0f;
            if (std::sscanf(value.c_str(), "%f,%f", &pw, &ph) == 2 &&
                std::isfinite(pw) && std::isfinite(ph)) {
                w.width = pw;
                w.height = ph;
                w.hasDimensions = true;
            }
        } else if (field == "groups") {
            w.messageGroups.clear();
            for (size_t at = 0; at <= value.size();) {
                const size_t comma = value.find(',', at);
                const std::string one = value.substr(at, comma - at);
                if (!one.empty()) w.messageGroups.push_back(one);
                if (comma == std::string::npos) break;
                at = comma + 1;
            }
        } else if (field == "channels") {
            w.channels.clear();
            for (size_t at = 0; at <= value.size();) {
                const size_t comma = value.find(',', at);
                const std::string one = value.substr(at, comma - at);
                const size_t colon = one.rfind(':');
                if (colon != std::string::npos && colon > 0) {
                    w.channels.emplace_back(one.substr(0, colon),
                                            std::atoi(one.c_str() + colon + 1));
                }
                if (comma == std::string::npos) break;
                at = comma + 1;
            }
        } else continue;
        ++loaded;
    }
    LOG_INFO("Interface state: loaded ", loaded, " value(s) from ",
             interfaceStatePath());
    // Written back at once rather than left for whatever saves next, so the
    // repair happens once and a player reading the file sees what the client
    // is actually using.
    if (repaired) saveInterfaceState();
}

/// A chat window's saved settings. FCF_SetWindowAlpha takes the alpha from
/// here and remembers it as oldAlpha, which the fade handlers then hand to
/// max() on every mouse-over - so a missing alpha is not a cosmetic gap, it is
/// an error every time the cursor crosses the frame.
static int lua_GetChatWindowInfo(lua_State* L) {
    const ChatWindowSettings* w = chatWindow(L, 1);
    if (!w) w = &chatWindows()[0];

    lua_pushstring(L, w->name.c_str());
    lua_pushnumber(L, w->fontSize);
    lua_pushnumber(L, w->r);
    lua_pushnumber(L, w->g);
    lua_pushnumber(L, w->b);
    lua_pushnumber(L, w->alpha);
    // Numbers and nil, not booleans. docked is a dock position, not a flag -
    // FCF_LoadChatSettings hands it straight to FCF_DockFrame as the index to
    // insert at, and that compares it against a count. A boolean there is a
    // comparison between a boolean and a number, which is an error rather than
    // a wrong answer.
    if (w->shown) lua_pushnumber(L, 1.0); else lua_pushnil(L);
    if (w->locked) lua_pushnumber(L, 1.0); else lua_pushnil(L);
    if (w->docked > 0) lua_pushnumber(L, w->docked); else lua_pushnil(L);
    if (w->uninteractable) lua_pushnumber(L, 1.0); else lua_pushnil(L);
    return 10;
}

// GetChatWindowSavedPosition(index) → point, xOffset, yOffset
static int lua_GetChatWindowSavedPosition(lua_State* L) {
    const ChatWindowSettings* w = chatWindow(L, 1);
    if (!w || !w->hasPosition) return luaReturnNil(L);
    lua_pushstring(L, w->point.c_str());
    lua_pushnumber(L, w->xOffset);
    lua_pushnumber(L, w->yOffset);
    return 3;
}

// GetChatWindowSavedDimensions(index) → width, height
static int lua_GetChatWindowSavedDimensions(lua_State* L) {
    const ChatWindowSettings* w = chatWindow(L, 1);
    if (!w || !w->hasDimensions) return luaReturnNil(L);
    lua_pushnumber(L, w->width);
    lua_pushnumber(L, w->height);
    return 2;
}

static int lua_SetChatWindowName(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) w->name = luaL_optstring(L, 2, "");
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowSize(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) w->fontSize = static_cast<float>(luaL_optnumber(L, 2, 14.0));
    saveInterfaceState();
    return 0;
}
// A missing component keeps what the window already has rather than falling to
// a number of this function's choosing. FCF_SetWindowColor passes on whatever
// FCF_GetChatWindowInfo answered, and that is nil for a window the interface
// has no settings for - so a default of 1 here reads "the interface said
// nothing" as "the interface said white", and saves it. The white background
// this client shipped with is what that looks like once it is on disk.
static int lua_SetChatWindowColor(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) {
        w->r = static_cast<float>(luaL_optnumber(L, 2, w->r));
        w->g = static_cast<float>(luaL_optnumber(L, 3, w->g));
        w->b = static_cast<float>(luaL_optnumber(L, 4, w->b));
    }
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowAlpha(lua_State* L) {
    if (auto* w = chatWindow(L, 1))
        w->alpha = static_cast<float>(luaL_optnumber(L, 2, w->alpha));
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowShown(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) w->shown = lua_toboolean(L, 2) != 0;
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowLocked(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) w->locked = lua_toboolean(L, 2) != 0;
    saveInterfaceState();
    return 0;
}
/// The dock position, which is a number and not a flag - see GetChatWindowInfo.
/// A false or nil means undocked, which is zero here rather than a missing key.
static int lua_SetChatWindowDocked(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) {
        if (lua_isnumber(L, 2)) w->docked = static_cast<int>(lua_tonumber(L, 2));
        else w->docked = lua_toboolean(L, 2) ? 1 : 0;
    }
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowUninteractable(lua_State* L) {
    if (auto* w = chatWindow(L, 1)) w->uninteractable = lua_toboolean(L, 2) != 0;
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowSavedPosition(lua_State* L) {
    auto* w = chatWindow(L, 1);
    if (!w) return 0;
    // Called with nil to forget the position, which is how a window that has
    // been re-docked stops being restored to where it floated.
    if (lua_isnoneornil(L, 2)) { w->hasPosition = false; saveInterfaceState(); return 0; }
    const float x = static_cast<float>(luaL_optnumber(L, 3, 0.0));
    const float y = static_cast<float>(luaL_optnumber(L, 4, 0.0));
    // A position is saved as a fraction of the screen, so anything that made
    // the screen zero for a frame makes this nan - and a nan written out is
    // permanent, because it is read back on every login and put straight into
    // the frame's rect. From there it is not a misplaced window: a rect with a
    // nan in it matches every hit test, since every comparison against a nan
    // is false, and one mouse-enabled frame answering every hit test stops the
    // camera turning for good. Refused rather than stored.
    if (!std::isfinite(x) || !std::isfinite(y)) {
        LOG_WARNING("Refusing a chat window position that is not a number: ",
                    x, ",", y);
        return 0;
    }
    w->point   = luaL_optstring(L, 2, "TOPLEFT");
    w->xOffset = x;
    w->yOffset = y;
    w->hasPosition = true;
    saveInterfaceState();
    return 0;
}
static int lua_SetChatWindowSavedDimensions(lua_State* L) {
    auto* w = chatWindow(L, 1);
    if (!w) return 0;
    if (lua_isnoneornil(L, 2)) { w->hasDimensions = false; saveInterfaceState(); return 0; }
    const float wd = static_cast<float>(luaL_optnumber(L, 2, 0.0));
    const float ht = static_cast<float>(luaL_optnumber(L, 3, 0.0));
    if (!std::isfinite(wd) || !std::isfinite(ht)) {
        LOG_WARNING("Refusing chat window dimensions that are not numbers: ",
                    wd, "x", ht);
        return 0;
    }
    w->width  = wd;
    w->height = ht;
    w->hasDimensions = true;
    saveInterfaceState();
    return 0;
}

/// ResetChatWindows() - back to the layout a new character starts with.
///
/// Announced, because the settings changing is not something the windows can
/// see. Every floating chat frame answers UPDATE_FLOATING_CHAT_WINDOWS by
/// re-reading its own settings, which is exactly what has just changed
/// underneath it; without the event the store is reset and the windows stay
/// where they were until something else happens to reload them.
static int lua_ResetChatWindows(lua_State* L) {
    auto& windows = chatWindows();
    windows = std::array<ChatWindowSettings, kNumChatWindows>{};
    windows[0].shown = true; windows[0].docked = 1;
    windows[1].shown = true; windows[1].docked = 2;

    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (engine) engine->fireEvent("UPDATE_FLOATING_CHAT_WINDOWS", {});
    saveInterfaceState();
    return 0;
}

/// Whether a chat type colours player names by class. Stored per chat type
/// rather than globally, which is how the chat options panel presents it -
/// a row per type, each with its own tick.
static std::set<std::string>& chatColorByClass() {
    static std::set<std::string> types;
    return types;
}
static int lua_SetChatColorNameByClass(lua_State* L) {
    std::string type(luaL_optstring(L, 1, ""));
    toLowerInPlace(type);
    if (type.empty()) return 0;
    if (lua_toboolean(L, 2)) chatColorByClass().insert(type);
    else chatColorByClass().erase(type);
    return 0;
}
static int lua_GetChatColorNameByClass(lua_State* L) {
    std::string type(luaL_optstring(L, 1, ""));
    toLowerInPlace(type);
    lua_pushboolean(L, chatColorByClass().count(type) ? 1 : 0);
    return 1;
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
/// The expansion, counted from zero: 0 vanilla, 1 TBC, 2 Wrath. That is the
/// numbering LFGDungeons.dbc's expansion column uses and the one
/// MAX_PLAYER_LEVEL_TABLE is keyed by - [0]=60, [1]=70, [2]=80.
static int expansionLevelZeroBased(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* reg = svc ? svc->expansionRegistry : nullptr;
    auto* prof = reg ? reg->getActive() : nullptr;
    if (!prof) return 2;
    if (prof->id == "wotlk") return 2;
    if (prof->id == "tbc") return 1;
    return 0;   // classic and turtle
}

static int lua_GetExpansionLevel(lua_State* L) {
    lua_pushnumber(L, expansionLevelZeroBased(L));
    return 1;
}

/// Normal, which is the difficulty a fresh group is on.
static int lua_ReturnOne(lua_State* L) {
    lua_pushnumber(L, 1.0);
    return 1;
}

// The always-up battleground score lines, in the order FrameXML asks for them.
// The server sends world states as bare key/value pairs, so the labels come
// from the shared table this client's own heads-up display also reads.
struct WorldStateLine {
    std::string text;
};

static std::vector<WorldStateLine> worldStateLines(game::GameHandler* gh) {
    std::vector<WorldStateLine> out;
    if (!gh) return out;
    const game::BgScoreDef* def = game::findBgScoreDef(gh->getWorldStateMapId());
    if (!def) return out;

    auto alliance = gh->getWorldState(def->allianceKey);
    auto horde    = gh->getWorldState(def->hordeKey);
    if (!alliance && !horde) return out;

    uint32_t maxScore = def->hardcodedMax;
    if (def->maxKey != 0) {
        if (auto mv = gh->getWorldState(def->maxKey)) maxScore = *mv;
    }
    const bool showMax = maxScore > 0 && def->unit && def->unit[0] != '\0';

    char buf[96];
    for (int side = 0; side < 2; ++side) {
        const char* label = side == 0 ? "Alliance" : "Horde";
        const uint32_t score = (side == 0 ? alliance : horde).value_or(0);
        if (showMax) snprintf(buf, sizeof(buf), "%s: %u/%u", label, score, maxScore);
        else         snprintf(buf, sizeof(buf), "%s: %u", label, score);
        out.push_back({buf});
    }
    return out;
}

// GetNumWorldStateUI() - how many always-up lines there are to draw.
static int lua_GetNumWorldStateUI(lua_State* L) {
    lua_pushinteger(L, static_cast<lua_Integer>(worldStateLines(getGameHandler(L)).size()));
    return 1;
}

// GetWorldStateUIInfo(index) → uiType, state, text, icon, dynamicIcon, tooltip,
// dynamicTooltip, extendedUI, extendedUIState1, extendedUIState2, extendedUIState3
//
// uiType 0 keeps worldstateframe.lua out of its world-PvP branch, which is the
// one gated on a CVar and on IsSubZonePVPPOI. state 1 means "show, no flash".
static int lua_GetWorldStateUIInfo(lua_State* L) {
    const int index = static_cast<int>(luaL_optinteger(L, 1, 0));
    const auto lines = worldStateLines(getGameHandler(L));
    if (index < 1 || index > static_cast<int>(lines.size())) return 0;

    lua_pushinteger(L, 0);                              // uiType
    lua_pushinteger(L, 1);                              // state
    lua_pushstring(L, lines[index - 1].text.c_str());   // text
    lua_pushstring(L, "");                              // icon
    lua_pushstring(L, "");                              // dynamicIcon
    lua_pushstring(L, "");                              // tooltip
    lua_pushstring(L, "");                              // dynamicTooltip
    lua_pushstring(L, "");                              // extendedUI
    lua_pushinteger(L, 0);                              // extendedUIState1
    lua_pushinteger(L, 0);                              // extendedUIState2
    lua_pushinteger(L, 0);                              // extendedUIState3
    return 11;
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
/// GetWeaponEnchantInfo() → per hand: hasEnchant, expiration, charges.
///
/// It answered no for both hands unconditionally, so TemporaryEnchantFrame
/// took its early exit and hid itself - a sharpening stone or an oil showed
/// nothing at all. The buff bar is handed over, so this client's own weapon
/// enchant display beside it is suppressed and this was the only one left.
///
/// The enchant is tracked per equipped item; its remaining time is not, and
/// the frame reads expiration only to write a countdown under an icon it has
/// already decided to show. Zero there costs the countdown, not the icon.
static int lua_GetWeaponEnchantInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const game::EquipSlot kHands[2] = {game::EquipSlot::MAIN_HAND,
                                       game::EquipSlot::OFF_HAND};
    for (const game::EquipSlot hand : kHands) {
        bool enchanted = false;
        if (gh) {
            const uint64_t guid = gh->getEquipSlotGuid(static_cast<int>(hand));
            if (guid != 0) enchanted = gh->getItemEnchantIds(guid).second != 0;
        }
        lua_pushboolean(L, enchanted ? 1 : 0);   // hasEnchant
        lua_pushnumber(L, 0.0);                  // expiration, not tracked
        lua_pushnumber(L, 0.0);                  // charges
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
/// deciding whether a feature exists - answering no where the answer is yes
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
            lua_getfield(L, -1, "loadOnDemand");
            const bool lod = lua_toboolean(L, -1) != 0;
            lua_pop(L, 1);
            lua_getfield(L, -1, "name");
            const char* name = lua_tostring(L, -1);
            // Being listed means loaded only for addons that load at startup.
            // A load-on-demand one is listed from the start and loaded later,
            // so its state is the manager's to answer, below.
            found = !lod && name && std::strcmp(name, wanted) == 0;
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    // A load-on-demand addon is not in the list handed to the VM at startup -
    // it is not loaded then - so its loaded state is asked of the manager.
    if (!found) {
        auto* svc = getLuaServices(L);
        if (svc && svc->isAddOnLoaded) found = svc->isAddOnLoaded(wanted);
    }
    lua_pushboolean(L, found ? 1 : 0);
    return 1;
}

/// GetTime() → seconds since the client started, as a float.
///
/// The interface's shared reference for anything timed: a cooldown records
/// GetTime() + duration and something else compares against it later. It was
/// never implemented, so every one of those comparisons was against nil -
/// including the ones in this client's own bootstrap.
/// LoadAddOn(name) → loaded, reason.
///
/// The reason is not optional when loaded is false: UIParentLoadAddOn builds
/// an error message out of _G["ADDON_" .. reason], so nil there is a
/// concatenation against nothing. These are Blizzard's own load-on-demand
/// panels - the talent frame and its like - which this client does not ship,
/// and MISSING is the reason string for exactly that.
/// LoadAddOn(name) → loaded, reason
///
/// How the interface reaches half its own panels: the talent tree, the
/// achievement window, the macro editor, the key bindings, the trade skill and
/// glyph frames are all load-on-demand addons that FrameXML asks for the first
/// time one is opened. This answered "MISSING" unconditionally, so none of them
/// ever appeared - and an addon that ships an optional module got the same.
static int lua_LoadAddOn(lua_State* L) {
    const char* name = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
    auto* svc = getLuaServices(L);
    if (!name || !svc || !svc->loadAddOn) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, "MISSING");
        return 2;
    }
    // Blizzard_Calendar loads. It was refused for a long time, and the note
    // that stood here is worth keeping in outline: the minimap's date button
    // calls Calendar_LoadUI, whose CalendarFrame_OnShow calls OpenCalendar, so
    // loading the addon with its verbs missing raised on one click. Saying no
    // made the button do nothing, which was the truth.
    //
    // What changed is that there is a calendar behind it now - the packet is
    // parsed into CalendarData, and the month grid, the day lists and the
    // holidays are answered from it. The write side is still missing, and a
    // missing global is a callable stand-in rather than a raise, so creating
    // an event does nothing rather than breaking the window it is in.
    std::string reason;
    const bool ok = svc->loadAddOn(name, reason);
    lua_pushboolean(L, ok ? 1 : 0);
    if (ok) { lua_pushnil(L); return 2; }
    // Only a reason the interface has a string for. UIParentLoadAddOn does
    //
    //     message(format(ADDON_LOAD_FAILED, name, _G["ADDON_"..reason]))
    //
    // so a token globalstrings does not define makes that lookup nil and
    // format raise - the report of a failed load failing, inside whichever
    // panel was being opened. Every load-on-demand panel opens this way.
    //
    // The list is globalstrings' own ADDON_* names, which is where the far
    // side reads them from. Anything else is answered as CORRUPT, which is
    // what the real client says when an addon is there and will not run.
    static const char* kKnown[] = {
        "BANNED", "CORRUPT", "DEMAND_LOADED", "DEP_BANNED", "DEP_CORRUPT",
        "DEP_DEMAND_LOADED", "DEP_DISABLED", "DEP_INCOMPATIBLE",
        "DEP_INSECURE", "DEP_INTERFACE_VERSION", "DEP_MISSING", "DISABLED",
        "INCOMPATIBLE", "INSECURE", "INTERFACE_VERSION", "MISSING",
    };
    bool known = false;
    for (const char* k : kKnown) {
        if (reason == k) { known = true; break; }
    }
    lua_pushstring(L, known ? reason.c_str() : "CORRUPT");
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
    // Whose faction. This answered the player's for every unit, and the target
    // frame asks it about the *target* to pick the PvP badge - so an Alliance
    // player saw an Alliance badge over a Horde target, and the party frames
    // did the same for anyone in the group.
    std::string uid(luaL_optstring(L, 1, "player"));
    toLowerInPlace(uid);
    // Orc, Undead, Tauren, Troll, Blood Elf.
    static const std::set<uint8_t> kHordeRaces = {2, 5, 6, 8, 10};
    const uint8_t race = unitRaceOf(getGameHandler(L), uid);
    // Nothing rather than a guess when the race is unknown: FrameXML compares
    // the answer against the player's and hides the badge when it has neither,
    // which is better than a badge of the wrong side.
    if (race == 0) return 0;
    const bool horde = kHordeRaces.count(race) > 0;
    lua_pushstring(L, horde ? "Horde" : "Alliance");
    lua_pushstring(L, horde ? "Horde" : "Alliance");
    return 2;
}

// RunScript(body) → compiles and runs Lua, the way /script and every macro do
//
// A compile failure is reported the same way a runtime one is, because to
// whoever typed it they are the same mistake. The error goes through the normal
// path, which shows it without telling any script about it.
static int lua_RunScript(lua_State* L) {
    const char* body = luaL_optstring(L, 1, nullptr);
    if (!body || !*body) return 0;
    if (luaL_loadstring(L, body) != 0) {
        const char* err = lua_tostring(L, -1);
        LOG_WARNING("RunScript would not compile: ", err ? err : "?");
        lua_pop(L, 1);
        return 0;
    }
    if (lua_pcall(L, 0, 0, 0) != 0) {
        const char* err = lua_tostring(L, -1);
        LOG_WARNING("RunScript failed: ", err ? err : "?");
        lua_pop(L, 1);
    }
    return 0;
}

// IsMouseButtonDown(button) → whether it is held right now
//
// Named as WoW names them - "LeftButton", "RightButton", "MiddleButton" - and
// answers for any button when asked for none, which is what a bare call means.
static int lua_IsMouseButtonDown(lua_State* L) {
    const char* which = luaL_optstring(L, 1, nullptr);
    bool down = false;
    if (!which || !*which) {
        down = ImGui::IsMouseDown(ImGuiMouseButton_Left) ||
               ImGui::IsMouseDown(ImGuiMouseButton_Right) ||
               ImGui::IsMouseDown(ImGuiMouseButton_Middle);
    } else {
        std::string name(which);
        for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (name == "leftbutton" || name == "left")        down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        else if (name == "rightbutton" || name == "right") down = ImGui::IsMouseDown(ImGuiMouseButton_Right);
        else if (name == "middlebutton" || name == "middle") down = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
    }
    lua_pushboolean(L, down ? 1 : 0);
    return 1;
}

// Screenshot() → saves one where the client's own binding puts it
static int lua_Screenshot(lua_State* L) {
    auto* svc = getLuaServices(L);
    if (svc && svc->takeScreenshot) svc->takeScreenshot();
    return 0;
}

// HasLFGRestrictions() → whether the player is in a dungeon-finder group
//
// There is a dungeon finder here - the client tracks the queue, the proposal
// and the dungeon - so this is answered from it rather than declared false the
// way it was when the comment here said no such thing existed.
static int lua_HasLFGRestrictions(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isLfgInDungeon() ? 1 : 0);
    return 1;
}

// GetLFGProposal() → proposalExists, typeID, id, name, texture, role,
//                    hasResponded, totalEncounters, completedEncounters,
//                    numMembers, isLeader
//
// Eleven values whether or not there is a proposal: lfdframe reads the eighth
// on its own with select(8, GetLFGProposal()), and a short return makes that
// an error rather than an answer of "none".
static int lua_GetLFGProposal(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool pending = gh && gh->getLfgState() == game::LfgState::Proposal;
    if (!pending) { for (int i = 0; i < 11; ++i) lua_pushnil(L); return 11; }

    // Nils here are not survivable, which only showed once the dialog could
    // actually open. LFDDungeonReadyPopup_Update builds a path with
    // "UI-LFG-BACKGROUND-"..texture, and concatenating nil raises; numMembers
    // goes into `for i = numMembers+1, ...`, where nil is arithmetic on
    // nothing; completedEncounters is compared with > 0. The dialog was
    // unreachable before, so none of it had ever run.
    const uint32_t dungeonId = gh->getLfgDungeonId();
    const game::LfgDungeon* dungeon = nullptr;
    for (const auto& d : gh->getLfgDungeons()) {
        if (d.id == dungeonId) { dungeon = &d; break; }
    }

    // The role the player queued as. The dialog names it and picks its icon
    // from it, and these are the tokens GetTexCoordsForRole indexes by.
    const uint8_t roles = gh->getLfgOfferedRoles();
    const char* role = (roles & 0x02) ? "TANK"
                     : (roles & 0x04) ? "HEALER"
                                      : "DAMAGER";

    lua_pushboolean(L, 1);                                       // 1: proposalExists
    lua_pushnumber(L, dungeon ? dungeon->typeId : 1);            // 2: typeID
    // The dungeon, not the proposal: LFDDungeonReadyPopup keeps this as
    // dungeonID and looks the dungeon up by it.
    lua_pushnumber(L, dungeonId);                                // 3: id
    lua_pushstring(L, dungeon ? dungeon->name.c_str()
                              : gh->getCurrentLfgDungeonName().c_str());  // 4: name
    // TextureFilename from LFGDungeons.dbc, which is exactly the suffix that
    // path is built from.
    lua_pushstring(L, dungeon ? dungeon->texture.c_str() : "");  // 5: texture
    lua_pushstring(L, role);                                     // 6: role
    lua_pushboolean(L, 0);                                       // 7: hasResponded
    // Encounter progress is not parsed out of the proposal, and zero is the
    // honest answer - it reads as "nothing cleared yet", which is what a fresh
    // pop is, and keeps the dialog off its in-progress layout.
    lua_pushnumber(L, 0);                                        // 8: totalEncounters
    lua_pushnumber(L, 0);                                        // 9: completedEncounters
    // The real count now that the roster is parsed - each row is a role icon
    // and a tick, both of which GetLFGProposalMember can answer.
    lua_pushnumber(L, static_cast<double>(gh->getLfgProposalMembers().size()));  // 10

    lua_pushboolean(L, 0);                                       // 11: isLeader
    return 11;
}

// GetLFGInfoServer() → inParty, joined, queued, noPartialClear, achievements,
//                      lfgComment, slotCount
static int lua_GetLFGInfoServer(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool queued = gh && gh->isLfgQueued();
    lua_pushnil(L);                        // 1: inParty
    lua_pushboolean(L, queued ? 1 : 0);    // 2: joined
    lua_pushboolean(L, queued ? 1 : 0);    // 3: queued
    lua_pushnil(L);                        // 4: noPartialClear
    lua_pushnil(L);                        // 5: achievements
    lua_pushnil(L);                        // 6: lfgComment
    lua_pushnil(L);                        // 7: slotCount
    return 7;
}

// GetLFGRoleUpdate() → roleCheckInProgress, slots, members
//
// The last two were nil, and nil is not a harmless blank here.
// LFDRoleCheckPopupDescription_OnEnter opens with `if ( slots <= 1 )`, which
// compares nil to a number and raises - hovering the role-check popup took the
// file down with it. LFDRoleCheckPopup_Update branches on `slots == 1` too, so
// every check also described itself as being for multiple dungeons.
//
// Both come off SMSG_LFG_ROLE_CHECK_UPDATE, which carries the dungeon list and
// the group size behind the count the handler used to stop at.
static int lua_GetLFGRoleUpdate(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool checking = gh && gh->getLfgState() == game::LfgState::RoleCheck;
    if (checking) lua_pushboolean(L, 1); else lua_pushnil(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getLfgRoleCheckDungeons().size()) : 0.0);
    lua_pushnumber(L, gh ? gh->getLfgRoleCheckMembers() : 0);
    return 3;
}

static int lua_IsListedInLFR(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->getLfgState() == game::LfgState::RaidBrowser ? 1 : 0);
    return 1;
}

// IsPartyLFG() → was this group put together by the dungeon finder
static int lua_IsPartyLFG(lua_State* L) {
    auto* gh = getGameHandler(L);
    bool viaFinder = false;
    if (gh) {
        const auto st = gh->getLfgState();
        viaFinder = (st == game::LfgState::InDungeon ||
                     st == game::LfgState::FinishedDungeon ||
                     st == game::LfgState::Boot);
    }
    lua_pushboolean(L, viaFinder ? 1 : 0);
    return 1;
}

// The two cooldowns that gate re-queuing. Neither is tracked, and both are read
// as `if ( expiration )` - so nil, because a zero would read as a live cooldown
// and park the queue frame behind a countdown that never ends.
static int lua_GetLFGDeserterExpiration(lua_State* L) { lua_pushnil(L); return 1; }
static int lua_GetLFGRandomCooldownExpiration(lua_State* L) { lua_pushnil(L); return 1; }

// RefreshLFGList() - the raid browser's refresh button.
static int lua_RefreshLFGList(lua_State* L) { (void)L; return 0; }

// GetTrackedAchievements() → the achievement ids being watched, as separate
// values. The watch frame counts them by return count.
static int lua_GetTrackedAchievements(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const auto& tracked = gh->getTrackedAchievements();
    // The watch frame shows ten, but nothing here enforces that, and Lua
    // guarantees twenty free slots.
    if (!tracked.empty() && !lua_checkstack(L, static_cast<int>(tracked.size()))) {
        return 0;
    }
    for (uint32_t id : tracked) lua_pushnumber(L, id);
    return static_cast<int>(tracked.size());
}

// The PvP flag's countdown. The client knows whether the flag is set but not
// how long it has left, so the timer is reported as not running and the player
// frame hides the text rather than showing a number it cannot compute.
static int lua_IsPVPTimerRunning(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// Never reached while the above answers false, but playerframe reads it on the
// line after and a missing global there would raise before the branch is taken.
static int lua_GetPVPTimer(lua_State* L) { lua_pushnumber(L, 0); return 1; }

// GetCurrentArenaSeason() → the season number, or NO_ARENA_SEASON.
//
// Zero is the right answer here rather than the usual trap: arenaframe compares
// it with == and ~= against NO_ARENA_SEASON, which is itself 0, so a nil would
// fail both tests instead of meaning "no season".
static int lua_GetCurrentArenaSeason(lua_State* L) { lua_pushnumber(L, 0); return 1; }

// GetPVPRankProgress() → how far through the current rank, 0..1.
// Fed straight to HonorFrameProgressBar:SetValue, which needs a number.
static int lua_GetPVPRankProgress(lua_State* L) { lua_pushnumber(L, 0); return 1; }

// ---- Arena team roster ----
//
// Which team an index names comes from the same list GetArenaTeam reads.
// ArenaTeamRoster(index) - ask the server for the roster.
static int lua_ArenaTeamRoster(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t teamId = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (gh && teamId) gh->requestArenaTeamRoster(teamId);
    return 0;
}

// GetArenaTeamRosterInfo(teamIndex, memberIndex) →
//   name, rank, level, class, online, played, win, seasonPlayed, seasonWin, rating
//
// The six counts are numbers rather than nil: the panel subtracts them the line
// after - `loss = played - win` - so a nil raises there. Class stays nil, which
// the panel does test before using, and rank and level are zero because the
// roster carries neither.
static int lua_GetArenaTeamRosterInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t teamId = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    const int member = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (!gh || teamId == 0 || member < 1) return luaReturnNil(L);
    const auto* roster = gh->getArenaTeamRoster(teamId);
    if (!roster || member > static_cast<int>(roster->members.size())) return luaReturnNil(L);
    const auto& m = roster->members[static_cast<size_t>(member) - 1];

    lua_pushstring(L, m.name.c_str());       // 1: name
    lua_pushnumber(L, 0);                    // 2: rank
    lua_pushnumber(L, 0);                    // 3: level
    lua_pushnil(L);                          // 4: class
    lua_pushboolean(L, m.online ? 1 : 0);    // 5: online
    lua_pushnumber(L, m.weekGames);          // 6: played
    lua_pushnumber(L, m.weekWins);           // 7: win
    lua_pushnumber(L, m.seasonGames);        // 8: seasonPlayed
    lua_pushnumber(L, m.seasonWins);         // 9: seasonWin
    lua_pushnumber(L, m.personalRating);     // 10: rating
    return 10;
}

// Which roster row is selected, and closing the roster. Both are the panel's
// own state - nothing is sent for either.
static int lua_GetArenaTeamRosterSelection(lua_State* L) { lua_pushnumber(L, 0); return 1; }
static int lua_SetArenaTeamRosterSelection(lua_State* L) { (void)L; return 0; }
static int lua_CloseArenaTeamRoster(lua_State* L) { (void)L; return 0; }

// Team captaincy is not reported by anything this client parses, so no team
// reads as the player's to run.
static int lua_IsArenaTeamCaptain(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// Closing the battlemaster's window, which is a client-side dismissal.
static int lua_CloseBattlefield(lua_State* L) { (void)L; return 0; }

// Leaving a vehicle, and whether its aim can be raised or lowered. Vehicles
// are not modelled here, so neither is possible.
// CanExitVehicle() - whether the player is riding something they can get off.
//
// False always, so the leave-vehicle button on the main bar stayed hidden and
// the unit menu's Leave Vehicle entry was removed from the menu every time it
// was built. The state behind it has been tracked the whole time.
static int lua_CanExitVehicle(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isInVehicle() ? 1 : 0);
    return 1;
}
static int lua_IsVehicleAimAngleAdjustable(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// HasKey() - whether the player carries a key ring at all. The keyring exists
// and holds keys, so the button that opens it is offered.
static int lua_HasKey(lua_State* L) { lua_pushboolean(L, 1); return 1; }

// GetArenaTeam(index) →
//   teamName, teamSize, teamRating, teamPlayed, teamWins, seasonTeamPlayed,
//   seasonTeamWins, playerPlayed, seasonPlayerPlayed, teamRank, playerRating,
//   backgroundR, backgroundG, backgroundB, emblem, emblemR, emblemG, emblemB,
//   border, borderR, borderG, borderB
//
// Twenty-two, because PVPTeam_Update unpacks every one of them on a single line
// and then feeds the colour components straight to SetVertexColor. Answering
// just the name - which is all the promote and kick confirmations need - left
// twenty-one nils behind it and took the team list down.
//
// The tabard is not tracked: SMSG_ARENA_TEAM_QUERY_RESPONSE carries the emblem
// style and colours and nothing here reads them. Those eight answer zero rather
// than nil, so the tabard draws black instead of raising mid-arithmetic.
static int lua_GetArenaTeam(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || idx < 1) return luaReturnNil(L);
    const auto& teams = gh->getArenaTeamStats();
    if (idx > static_cast<int>(teams.size())) return luaReturnNil(L);
    const auto& t = teams[static_cast<size_t>(idx) - 1];

    lua_pushstring(L, t.teamName.c_str());  // 1: teamName
    lua_pushnumber(L, t.teamType);          // 2: teamSize (2, 3 or 5)
    lua_pushnumber(L, t.rating);            // 3: teamRating
    lua_pushnumber(L, t.weekGames);         // 4: teamPlayed
    lua_pushnumber(L, t.weekWins);          // 5: teamWins
    lua_pushnumber(L, t.seasonGames);       // 6: seasonTeamPlayed
    lua_pushnumber(L, t.seasonWins);        // 7: seasonTeamWins
    // Per-player totals come with the roster, not the team summary.
    lua_pushnumber(L, 0);                   // 8: playerPlayed
    lua_pushnumber(L, 0);                   // 9: seasonPlayerPlayed
    lua_pushnumber(L, t.rank);              // 10: teamRank
    lua_pushnumber(L, 0);                   // 11: playerRating
    for (int i = 0; i < 11; ++i) lua_pushnumber(L, 0);  // 12-22: tabard
    return 22;
}

// The daily-win bonus on a random or holiday battleground.
//
// → hasWin, winHonor, winArena, lossHonor, lossArena
//
// Zero rather than nil for the four amounts, and the difference matters: the
// frame writes `if (winHonor ~= 0)` and shows a reward line when that passes.
// nil passes it - nil is not zero - and the line appears with nothing in it.
// Zero says "no bonus" and the line is correctly skipped.
//
// The bonus itself is a per-character daily the server tracks and does not
// volunteer, so "none available" is the honest answer rather than a placeholder
// for one. Reachable, unlike most of this file's absences: the random
// battleground row is drawn from GetBattlegroundInfo, which answers for real.
static int lua_BattlegroundHonorBonusesNone(lua_State* L) {
    lua_pushboolean(L, 0);   // hasWin - the daily is not known to be waiting
    lua_pushnumber(L, 0);    // winHonor
    lua_pushnumber(L, 0);    // winArena
    lua_pushnumber(L, 0);    // lossHonor
    lua_pushnumber(L, 0);    // lossArena
    return 5;
}

// GetBattlefieldInstanceInfo(index) → which numbered instance that row is.
//
// Zero, which the list reads as unnumbered - the same "first available" the
// selection means. The server numbers instances only for a client that asks to
// pick one, and nothing here does.
static int lua_GetBattlefieldInstanceInfo(lua_State* L) {
    // The instance number of the i-th battleground instance on offer, counting
    // from one. A flat zero before, so every row in the list read "Warsong
    // Gulch 0" - and the count beside it, GetNumBattlefields, was not bound at
    // all, so the list never got that far: it raised on a nil global.
    //
    // SMSG_BATTLEFIELD_LIST carries these and they were parsed into
    // AvailableBgInfo::instanceIds all along.
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) { lua_pushnumber(L, 0); return 1; }
    const auto& bgs = gh->getAvailableBgs();
    if (bgs.empty()) { lua_pushnumber(L, 0); return 1; }
    const auto& ids = bgs.back().instanceIds;
    if (index > static_cast<int>(ids.size())) { lua_pushnumber(L, 0); return 1; }
    lua_pushnumber(L, ids[static_cast<size_t>(index) - 1]);
    return 1;
}

// GetNumBattlefields() - how many instances of it there are to choose between.
static int lua_GetNumBattlefields(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); return 1; }
    const auto& bgs = gh->getAvailableBgs();
    lua_pushnumber(L, bgs.empty()
        ? 0 : static_cast<lua_Number>(bgs.back().instanceIds.size()));
    return 1;
}

// IsInLFGDungeon() - standing inside a dungeon the finder put you in.
//
// The state is already tracked: SocialHandler keeps an LfgState and InDungeon
// is one of its values. Nothing read it, so the minimap's dungeon button could
// not tell inside from outside and offered the wrong direction.
static int lua_IsInLFGDungeon(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->getLfgState() == game::LfgState::InDungeon ? 1 : 0);
    return 1;
}

// LFGTeleport(out) - out to the dungeon's entrance, or back in.
static int lua_LFGTeleport(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->lfgTeleport(lua_toboolean(L, 1) != 0);
    return 0;
}

// Whether the world the player is standing in is an arena.
//
// From BattlemasterList.dbc, which names each row's maps and which this client
// already reads for the queue list - the arena rows are loaded alongside the
// battleground ones and only the battlegrounds are kept in the queue list, so
// the arena maps were sitting there unasked.
//
// Both of these answered nil, which the battlefield frame and the arena frame
// each read as "not an arena". That is right until the player is in one.
static int lua_IsBattlefieldArena(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isArenaMap(gh->getCurrentMapId()) ? 1 : 0);
    return 1;
}

// IsActiveBattlefieldArena() → isArena, isRegistered
//
// The second is whether the team is a registered one rather than a skirmish,
// which needs the arena team the server never mentions outside a match. Left
// nil, and the frame treats that as unregistered - which a skirmish is.
static int lua_IsActiveBattlefieldArena(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh && gh->isArenaMap(gh->getCurrentMapId())) lua_pushboolean(L, 1);
    else lua_pushnil(L);
    lua_pushnil(L);
    return 2;
}

// CanHearthAndResurrectFromArea() → whether the zone offers the combined
// hearth-and-release button. Only world PvP zones do, and none are tracked.
static int lua_CanHearthAndResurrectFromArea(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// GetWorldPVPQueueStatus(i) → status, mapName, queueID
//
// "none", not nil. An empty world PvP queue is a status of its own, and the
// interface tests for it by name:
//
//     status, mapName, queueID = GetWorldPVPQueueStatus(i);
//     if ( status ~= "none" ) then numberQueues = numberQueues + 1; end
//
// nil is not "none", so this counted a queue that did not exist on every pass -
// and BattlefieldFrame_UpdateStatus ends by hiding the minimap's battlefield
// icon only when numberQueues reaches zero, which it now never did. The icon
// sat beside the minimap permanently, with no queue behind it and so nothing in
// its tooltip: a queue tracker that was always there and never tracked
// anything.
//
// There is no Wintergrasp queue in this client, so "none" is also the true
// answer. mapName and queueID stay nil, which is what an empty slot answers.
static int lua_GetWorldPVPQueueStatus(lua_State* L) {
    lua_pushstring(L, "none");
    lua_pushnil(L);
    lua_pushnil(L);
    return 3;
}

// LeaveBattlefield() - walk out of the battleground currently being played.
static int lua_LeaveBattlefield(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->leaveBattlefield();
    return 0;
}

// ---- World map player arrow and ping ----
//
// In the real client these manage a rotating 3D arrow model showing where the
// player is standing and which way they face, plus the ping that plays when a
// party member signals a spot on the map.
//
// This client already draws its own player marker, party dots and quest POIs
// over the map, so the arrow would be a second marker on top of the first.
// They are defined anyway because two of them are called from
// WorldMapFrame_OnLoad: without them that function raised on its fifth line,
// so the black separator never got its colour, the ping never initialised, and
// the WorldMapFrame_Update() call that ends OnLoad never ran at all.
//
// UpdateWorldMapArrowFrames is not among them: it was already registered
// elsewhere in this file, which is why it never appeared as missing.
static int lua_CreateWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }
static int lua_ShowWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }
static int lua_PositionWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }
static int lua_InitWorldMapPing(lua_State* L) { (void)L; return 0; }

// The battlefield minimap's own copy of the arrow, for the same reason and
// with the same answer as the world map's above.
static int lua_CreateMiniWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }
static int lua_PositionMiniWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }
static int lua_ShowMiniWorldMapArrowFrame(lua_State* L) { (void)L; return 0; }

// ---- The stored combat log (Blizzard_CombatLog's refilter) ----
//
// The client keeps no history of combat events - they are handled as they
// arrive - so there is nothing to walk back through and the log rebuilds itself
// from new events only.
//
// CombatLogGetCurrentEntry answers nil rather than zero, and that distinction
// is the whole of it. Blizzard_CombatLog_RefilterUpdate loops
// `while (valid and total < COMBATLOG_LIMIT_PER_FRAME)`, and a zero is true in
// Lua - it would add a line per iteration from an entry that does not exist,
// stop only on the per-frame cap, and be re-armed by the OnUpdate that
// scheduled it, every frame, for as long as the log was open.
static int lua_CombatLogGetNumEntries(lua_State* L) { lua_pushnumber(L, 0); return 1; }
static int lua_CombatLogGetCurrentEntry(lua_State* L) { (void)L; return luaReturnNil(L); }
static int lua_CombatLogAdvanceEntry(lua_State* L) { (void)L; return luaReturnNil(L); }
static int lua_CombatLogSetCurrentEntry(lua_State* L) { (void)L; return 0; }
static int lua_CombatLogAddFilter(lua_State* L) { (void)L; return 0; }
static int lua_CombatLogResetFilter(lua_State* L) { (void)L; return 0; }

// CombatTextSetActiveUnit(unit) - which unit the floating combat text follows.
// It tells the client where to aim the events it already sends; the events do
// not change, so this is recorded by the caller and nothing is needed here.
static int lua_CombatTextSetActiveUnit(lua_State* L) { (void)L; return 0; }

// GetBattlefieldMapIconScale() → what to multiply the map's icon sizes by.
// One is the client's own default; the icons are sized in the frame itself,
// and every use here is a multiply, so a nil would take the arithmetic down.
static int lua_GetBattlefieldMapIconScale(lua_State* L) { lua_pushnumber(L, 1.0); return 1; }

// PlayerIsPVPInactive(unit) → whether a battleground member has gone idle and
// is about to be removed. The server reports this per-player in a battleground
// and nothing here parses it, so nobody reads as idle.
static int lua_PlayerIsPVPInactive(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// ---- The battlemaster's battleground list ----
//
// GetBattlefieldInfo() → mapName, mapDescription, maxGroup, canEnter,
//                        isHoliday, isRandom
//
// About the battleground the battlemaster being spoken to offers, which is
// what the last SMSG_BATTLEFIELD_LIST described. The frame returns early on a
// nil name, so before this the list drew nothing at all.
//
// The description is nil: BattlemasterList.dbc carries no blurb, and the panel
// treats a missing one as "no description" rather than raising on it.
static int lua_GetBattlefieldInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return luaReturnNil(L);
    const auto& bgs = gh->getAvailableBgs();
    if (bgs.empty()) return luaReturnNil(L);
    const auto* info = gh->getBattlemasterInfo(bgs.back().bgTypeId);
    if (!info || info->name.empty()) return luaReturnNil(L);
    const auto& bg = bgs.back();
    lua_pushstring(L, info->name.c_str());   // 1: mapName
    lua_pushnil(L);                          // 2: mapDescription
    lua_pushnumber(L, info->maxGroupSize);   // 3: maxGroup
    // Three more, all of which the panel reads. Whether the player is inside
    // the level bracket the server sent, whether this is the week's call to
    // arms, and whether it is the random battleground rather than a named one
    // - type 32, which is what BattlefieldFrame keys the rewards block on.
    const uint32_t level = gh->getPlayerLevel();
    const bool inBracket = (bg.minLevel == 0 || level >= bg.minLevel) &&
                           (bg.maxLevel == 0 || level <= bg.maxLevel);
    constexpr uint32_t kRandomBattleground = 32;
    lua_pushboolean(L, inBracket ? 1 : 0);                        // 4: canEnter
    lua_pushboolean(L, bg.isHoliday ? 1 : 0);                     // 5: isHoliday
    lua_pushboolean(L, bg.bgTypeId == kRandomBattleground ? 1 : 0); // 6: isRandom
    return 6;
}

// GetNumBattlegroundTypes() → how many battlegrounds there are to queue for.
//
// This is the PvP frame's own list, not the battlemaster's offering, and it
// comes from BattlemasterList.dbc - which this client already loads. It was
// answering zero from the counting stub, so the list drew no rows, nothing was
// ever assigned frame.BGindex, and the click handler read that field as nil.
//
// Arenas are excluded: they share the table with battlegrounds and are told
// apart by the instance type, so offering them here would queue the player for
// the wrong thing.
static int lua_GetNumBattlegroundTypes(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getBattlegroundTypes().size()) : 0.0);
    return 1;
}

// GetBattlegroundInfo(index) → localizedName, canEnter, isHoliday, isRandom, id
//
// canEnter is the level range from the row, which is what the client itself
// knows; the server still has the final say when the queue request arrives.
// isHoliday would need the call-to-arms world state, which nothing here parses,
// and claiming a holiday that is not running is worse than not mentioning one.
static int lua_GetBattlegroundInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh) return luaReturnNil(L);
    const auto& list = gh->getBattlegroundTypes();
    if (index < 1 || index > static_cast<int>(list.size())) return luaReturnNil(L);
    const auto& bg = list[static_cast<size_t>(index - 1)];

    // A row naming no level range - the random battleground is the only one -
    // is not a row saying nobody qualifies.
    const uint32_t level = gh->getPlayerLevel();
    const bool hasRange = bg.minLevel != 0 || bg.maxLevel != 0;
    const bool canEnter = !hasRange || (level >= bg.minLevel && level <= bg.maxLevel);

    lua_pushstring(L, bg.name.c_str());
    lua_pushboolean(L, canEnter ? 1 : 0);
    lua_pushboolean(L, 0);                       // isHoliday: world state not parsed
    lua_pushboolean(L, bg.mapCount > 1 ? 1 : 0); // several maps means a pool, i.e. random
    lua_pushnumber(L, bg.id);
    return 5;
}

// RequestBattlegroundInstanceInfo(index) - ask which instances are running.
//
// The reply is SMSG_BATTLEFIELD_LIST, which this client already handled and
// had no way to ask for. The index is a row in the list above, not a
// battleground id, so it is translated before it goes on the wire.
static int lua_RequestBattlegroundInstanceInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh) return 0;
    const auto& list = gh->getBattlegroundTypes();
    if (index < 1 || index > static_cast<int>(list.size())) return 0;
    gh->requestBattlefieldList(list[static_cast<size_t>(index - 1)].id);
    return 0;
}

// Which instance of a battleground is picked in the list.
//
// Kept here because the pair has to round-trip: the frame highlights the row
// matching what it last set, so a getter that always answered zero left the
// first row highlighted whatever was clicked.
//
// Zero remains the default and still means "first available", which is what
// the server understands as no preference - the list opens on it, and nothing
// here queues for a specific instance regardless. This is the selection the
// interface is showing, not an instruction to the server.
static int& selectedBattlefield() { static int selected = 0; return selected; }

static int lua_GetSelectedBattlefield(lua_State* L) {
    lua_pushnumber(L, selectedBattlefield());
    return 1;
}

// SetSelectedBattlefield(index) - called unguarded from two places, so it has
// to exist before it has to do anything.
static int lua_SetSelectedBattlefield(lua_State* L) {
    selectedBattlefield() = static_cast<int>(luaL_optnumber(L, 1, 0));
    return 0;
}

// JoinBattlefield(index, asGroup, isArena) - queue for it.
static int lua_JoinBattlefield(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const auto& bgs = gh->getAvailableBgs();
    if (bgs.empty()) return 0;
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const bool asGroup = lua_toboolean(L, 2) != 0;
    // The index names an instance in the list, and zero means first available.
    const auto& bg = bgs.back();
    uint32_t instanceId = 0;
    if (index > 0 && index <= static_cast<int>(bg.instanceIds.size())) {
        instanceId = bg.instanceIds[static_cast<size_t>(index) - 1];
    }
    gh->joinBattlefield(gh->getCurrentGossip().npcGuid, bg.bgTypeId, instanceId, asGroup);
    return 0;
}

// GetLFGCompletionReward() →
//   name, typeID, textureFilename, moneyBase, moneyVar, experienceBase,
//   experienceVar, numStrangers, numRewards
//
// What a finished dungeon finder run paid out, from SMSG_LFG_PLAYER_REWARD.
//
// The six counts answer zero rather than nil even with no reward to report,
// which is the opposite of the usual choice and for the opposite reason: the
// alert frame does not test them, it does arithmetic with them -
//
//     local moneyAmount = moneyBase + moneyVar * numStrangers
//
// so a nil raises there rather than reading as absent.
//
// The variable halves are always zero. They are the per-stranger bonus for
// filling a group with people from other realms, which 3.3.5a's server does not
// send and this realm could not have anyway; numStrangers is zero to match, so
// the arithmetic above lands on the base whichever way the frame reads it.
static int lua_GetLFGCompletionReward(lua_State* L) {
    auto* gh = getGameHandler(L);
    const game::LfgCompletionReward* r = gh ? &gh->getLfgCompletionReward() : nullptr;
    if (!r || !r->valid) {
        lua_pushnil(L);      // 1: name
        lua_pushnil(L);      // 2: typeID
        lua_pushnil(L);      // 3: textureFilename
        for (int i = 0; i < 6; ++i) lua_pushnumber(L, 0);
        return 9;
    }
    // The dungeon finished, not the random entry queued for: the toast names
    // the instance the player actually walked out of.
    const game::LfgDungeon* d = nullptr;
    for (const auto& e : gh->getLfgDungeons()) {
        if (e.id == r->dungeonId) { d = &e; break; }
    }
    // Pushed once each and in order, rather than an if/else per slot: two
    // pushes on one line read as two return values to anything counting them,
    // this file's own return-order sweep included.
    const std::string name = (d && !d->name.empty()) ? d->name : std::string();
    // Appended to "Interface\\LFGFrame\\LFGIcon-" by the frame, so it is the
    // bare suffix out of LFGDungeons.dbc and not a path.
    const std::string texture = (d && !d->texture.empty()) ? d->texture : std::string();
    name.empty() ? lua_pushnil(L) : lua_pushstring(L, name.c_str());          // 1
    d ? lua_pushinteger(L, static_cast<lua_Integer>(d->typeId)) : lua_pushnil(L);  // 2
    texture.empty() ? lua_pushnil(L) : lua_pushstring(L, texture.c_str());    // 3
    lua_pushnumber(L, static_cast<double>(r->money));  // 4: moneyBase
    lua_pushnumber(L, 0);                              // 5: moneyVar
    lua_pushnumber(L, static_cast<double>(r->xp));     // 6: experienceBase
    lua_pushnumber(L, 0);                              // 7: experienceVar
    lua_pushnumber(L, 0);                              // 8: numStrangers
    lua_pushnumber(L, static_cast<double>(r->items.size()));  // 9: numRewards
    return 9;
}

// GetLFGCompletionRewardItem(index) → texturePath, quantity
//
// One-based, and the index runs over the rewards the call above counted. The
// texture is the item's own icon, which is where the display id in the packet
// earns its place - the reward can name an item the bags have never held and
// so have no cached query behind it.
static int lua_GetLFGCompletionRewardItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const game::LfgCompletionReward* r = gh ? &gh->getLfgCompletionReward() : nullptr;
    if (!r || !r->valid || index < 1 ||
        index > static_cast<int>(r->items.size())) {
        lua_pushnil(L);
        lua_pushnumber(L, 0);
        return 2;
    }
    const auto& item = r->items[static_cast<size_t>(index) - 1];
    const std::string icon = gh->getItemIconPath(item.displayId);
    if (!icon.empty()) lua_pushstring(L, icon.c_str()); else lua_pushnil(L);
    lua_pushnumber(L, static_cast<double>(item.count));
    return 2;
}

// RunMacroText(body) - run a macro body, one command per line.
//
// The same path the action bar takes for a macro button, so a macro run from a
// party frame's click behaves as one run from the bar: /stopmacro is honoured,
// and each line goes through the slash dispatch rather than being sent as chat.
static int lua_RunMacroText(lua_State* L) {
    auto* svc = getLuaServices(L);
    const char* body = luaL_optstring(L, 1, "");
    if (svc && svc->runMacroText && body && *body) svc->runMacroText(body);
    return 0;
}

// RunMacro(id or name) - run a saved macro by which one it is.
static int lua_RunMacro(lua_State* L) {
    auto* svc = getLuaServices(L);
    auto* gh = getGameHandler(L);
    if (!svc || !svc->runMacroText || !gh) return 0;
    uint32_t macroId = 0;
    if (lua_isnumber(L, 1)) {
        macroId = static_cast<uint32_t>(lua_tonumber(L, 1));
    } else if (const char* name = lua_tostring(L, 1)) {
        // By name, which is how a macro written into another macro names it.
        for (uint32_t id : gh->getMacroIds()) {
            if (gh->getMacroName(id) == name) { macroId = id; break; }
        }
    }
    if (macroId == 0) return 0;
    const std::string& body = gh->getMacroText(macroId);
    if (!body.empty()) svc->runMacroText(body);
    return 0;
}

// TriggerTutorial(id) - show one of the interface's tutorial pop-outs.
//
// Tutorials are a saved per-account set of which have been seen, and none of
// that is kept here, so nothing is shown. Answered rather than left missing
// because the bag bar fires it whenever a bag is picked up.
static int lua_TriggerTutorial(lua_State* L) { (void)L; return 0; }

// Quit() - leave the game, as the game menu's Exit button does.
//
// The same path /exit takes: a clean logout that ends the process rather than
// dropping to character select.
static int lua_Quit(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) {
        gh->requestLogout(/*exitAfterLogout=*/true);
        return 0;
    }
    // No handler to route through - the login screen, or before one exists.
    // Exit Game still has to exit; returning here left the button inert.
    if (auto* svc = getLuaServices(L); svc && svc->quitApplication) {
        svc->quitApplication();
    }
    return 0;
}

/// ForceQuit() / ForceLogout() - leave now rather than after the timer.
///
/// QUIT's OnAccept calls the first, and the popup exists to offer exactly this:
/// the server has imposed a countdown and the player would rather not wait.
/// There is no message that shortens it - the server owns the timer - so what
/// this can honestly do is ask again, which is what the real client's button
/// amounts to once the timer is already running.
///
/// Bound together because CAMP carries the same pair and it would be strange
/// for one to answer and not the other, even though Blizzard has that call
/// commented out with a note saying forced logout is unfinished.
static int lua_ForceQuit(lua_State* L) {
    // The same thing Quit does, and deliberately the same code: asking again
    // is what the popup's button amounts to once the server owns the timer.
    return lua_Quit(L);
}

static int lua_ForceLogout(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->requestLogout(/*exitAfterLogout=*/false);
    return 0;
}

// ReloadUI() - rebuild the interface, as /reload does.
//
// Only asks. The reload shuts this Lua state down and builds a new one, and
// every caller is inside it: a static popup's OnAccept after a setting that
// needs one, or /reload typed at the interface rather than at the client's own
// command handler. Doing the work here would free the state mid-call.
static int lua_ReloadUI(lua_State* L) {
    auto* svc = getLuaServices(L);
    if (svc && svc->requestReloadUI) svc->requestReloadUI();
    return 0;
}

// WoweeSettingList() - every client setting FrameXML has no control for.
//
// Answers an array of { key, label, kind, min, max, step, category }, from the
// one schema in ui/settings_schema.hpp. The Wowee options category is built
// from this rather than written out in Lua, so adding a setting is a row in the
// schema and nothing else.
static int lua_WoweeSettingList(lua_State* L) {
    std::size_t count = 0;
    const auto* schema = ui::clientSettingsSchema(count);
    lua_newtable(L);
    for (std::size_t i = 0; i < count; ++i) {
        const auto& d = schema[i];
        lua_newtable(L);
        lua_pushstring(L, d.key);      lua_setfield(L, -2, "key");
        lua_pushstring(L, d.label);    lua_setfield(L, -2, "label");
        lua_pushstring(L, d.kind == ui::SettingKind::Bool  ? "bool"
                        : d.kind == ui::SettingKind::Int   ? "int"
                        : d.kind == ui::SettingKind::Enum  ? "enum"
                                                           : "float");
        lua_setfield(L, -2, "kind");
        lua_pushnumber(L, d.minValue); lua_setfield(L, -2, "min");
        lua_pushnumber(L, d.maxValue); lua_setfield(L, -2, "max");
        lua_pushnumber(L, d.step);     lua_setfield(L, -2, "step");
        lua_pushstring(L, d.category); lua_setfield(L, -2, "category");
        lua_pushstring(L, d.section);  lua_setfield(L, -2, "section");
        lua_pushstring(L, d.tooltip);  lua_setfield(L, -2, "tooltip");
        lua_pushstring(L, d.choices);  lua_setfield(L, -2, "choices");
        lua_pushnumber(L, d.defaultValue); lua_setfield(L, -2, "default");
        lua_pushstring(L, d.enabledWhen);  lua_setfield(L, -2, "enabledwhen");
        lua_rawseti(L, -2, static_cast<int>(i) + 1);
    }
    return 1;
}

// WoweeVersion() - what this client calls itself, and when it was built.
//
// The same string the login screen shows. Here so the options panel can say it
// without the version being written out a second time in Lua, where it would go
// stale the first time a tag was cut.
static int lua_WoweeVersion(lua_State* L) {
    lua_pushstring(L, core::kVersionString);
    return 1;
}

// WoweeGetSetting(key) / WoweeSetSetting(key, value) - the values behind that
// list. Strings both ways, as a CVar is.
static int lua_WoweeGetSetting(lua_State* L) {
    const char* key = luaL_checkstring(L, 1);
    auto* svc = getLuaServices(L);
    if (svc && svc->getClientSetting) {
        lua_pushstring(L, svc->getClientSetting(key).c_str());
        return 1;
    }
    return luaReturnNil(L);
}

static int lua_WoweeSetSetting(lua_State* L) {
    const char* key = luaL_checkstring(L, 1);
    std::string value;
    if (lua_isboolean(L, 2)) value = lua_toboolean(L, 2) ? "1" : "0";
    else if (lua_isstring(L, 2) || lua_isnumber(L, 2)) value = lua_tostring(L, 2);
    else value = "0";
    if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
        svc->setClientSetting(key, value);
    }
    return 0;
}

// WoweeShowSettings(tab) - open this client's settings window.
//
// FrameXML's game menu has Video, Sound and Interface buttons, and the frames
// they open are shells in this shim: InterfaceOptionsFrame exists so addons can
// register panels against it, and nothing draws it. The settings themselves -
// sixty-odd of them - are in this client's own window, so the menu's buttons
// come here instead of opening an empty frame.
//
// The tab name is optional and matches the window's own tab labels.
static int lua_WoweeShowSettings(lua_State* L) {
    auto* svc = getLuaServices(L);
    const char* tab = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
    if (svc && svc->openSettings) svc->openSettings(tab ? tab : "");
    return 0;
}

// GetGamma() / SetGamma(value) - screen brightness, as the video options mean
// it. One is neutral. Backed by the client's own brightness setting, so the
// two sliders move together instead of disagreeing.
static int lua_GetGamma(lua_State* L) {
    auto* svc = getLuaServices(L);
    lua_pushnumber(L, (svc && svc->getGamma) ? svc->getGamma() : 1.0);
    return 1;
}

static int lua_SetGamma(lua_State* L) {
    auto* svc = getLuaServices(L);
    if (svc && svc->setGamma) svc->setGamma(static_cast<float>(luaL_optnumber(L, 1, 1.0)));
    return 0;
}

// GetVideoCaps() →
//   anisotropic, pixelShaders, vertexShaders, trilinear, buffering,
//   maxAnisotropy, hardwareCursor
//
// Everything here runs on Vulkan, so the capability questions all answer yes -
// they were written for a Direct3D 9 client that could genuinely lack them.
// maxAnisotropy is the one real number, and sixteen is the ceiling every
// device this client will start on supports; the options panel uses it only to
// bound its own dropdown.
/// Terrain texture detail, as the video panel's TerrainDetail slider sets it.
///
/// Both halves were missing, and the slider reads its own value on every
/// refresh of the panel - `self.GetCurrentValue = function (self) return
/// GetTerrainMip(); end` - so opening Video Options called nil, and moving the
/// slider called nil again.
///
/// Held rather than applied: nothing in the terrain renderer takes a mip bias
/// yet. The slider works and remembers where it was put, which is the whole of
/// what the panel can observe, and a value is here for the renderer to read
/// when it grows one.
static int& terrainMip() { static int level = 1; return level; }

static int lua_GetTerrainMip(lua_State* L) {
    lua_pushnumber(L, terrainMip());
    return 1;
}

static int lua_SetTerrainMip(lua_State* L) {
    terrainMip() = static_cast<int>(luaL_optnumber(L, 1, 1));
    return 0;
}

static int lua_GetVideoCaps(lua_State* L) {
    lua_pushboolean(L, 1);   // 1: anisotropic
    lua_pushboolean(L, 1);   // 2: pixelShaders
    lua_pushboolean(L, 1);   // 3: vertexShaders
    lua_pushboolean(L, 1);   // 4: trilinear
    lua_pushboolean(L, 1);   // 5: buffering
    lua_pushnumber(L, 16);   // 6: maxAnisotropy
    lua_pushboolean(L, 1);   // 7: hardwareCursor
    return 7;
}

// GetCVarMin(name) / GetCVarMax(name) → the range a CVar is allowed, if it
// declares one.
//
// Nil for all but the few listed here, and every caller is written for nil:
// BlizzardOptionsPanel_GetCVarMinSafe passes it through tonumber, the slider
// setup falls back with `or entry.minValue`, and the clamp reads
// `if ( minValue and value < minValue )`. Answering a made-up zero for the
// rest would clamp every graphics slider in the options panel to it.
//
// This is the seam Blizzard's own panel code offers for a range the client
// owns rather than the interface: BlizzardOptionsPanel_OnEvent takes
// GetCVarMax first and only falls back to its table. So widening a slider is
// a row here, not an edit to a shipped Lua file - which matters because the
// interface data is extracted game content and is not ours to keep changes in.
struct CVarRange {
    const char* cvar;
    float minValue;
    float maxValue;
};

constexpr CVarRange kCVarRanges[] = {
    // The interface's own scale. 1 is the size the screen's height alone gives,
    // and the shipped table stops there because the slider was only ever for
    // making the interface smaller. A screen across a room needs the other
    // direction. The floor is Blizzard's; it was the ceiling that was wrong.
    //
    // The ceiling is the tree's, not a number repeated here: past it the
    // interface's own frames no longer fit on screen, options frame included,
    // and a scale you cannot undo from inside the game is worse than one that
    // is merely too small.
    {"uiscale", 0.64f, ui::WidgetTree::kMaxUserScale},
    // View distance. The shipped table stops at 1277 - the number the original
    // client's own renderer could reach - and this one draws to 2400, so the
    // slider could not ask for the range the engine has. Both ends are what
    // Renderer::setViewDistance clamps to, so the control now covers exactly
    // what the client can do and nothing it cannot.
    {"farclip", 400.0f, 2400.0f},
    // Mouse Look Speed, over the range this client's own mouse slider uses.
    {"camerayawmovespeed", 0.05f, 1.0f},
    // Mouse Sensitivity, which writes the same setting as Mouse Look Speed
    // above and so has to offer the same range. Shipped as 0.5 to 1.5, a
    // multiplier around 1.0, against a setting that sits at 0.2 and stops at 1.
    {"mousespeed", 0.05f, 1.0f},
    // Camera Following Speed. Not the shipped range: see the note in
    // applyCVarSideEffects - these are the bounds the camera itself clamps to,
    // so every position on the slider is a speed this client can actually run.
    {"camerayawsmoothspeed", 5.0f, 100.0f},
    // Max camera distance, as a multiple of the original client's limit. The
    // shipped table stops at 2; this client has always been willing to go
    // further, and did it through a checkbox of its own until this slider was
    // wired to mean it.
    {"cameradistancemaxfactor", 1.0f, rendering::CameraController::kMaxDistanceFactorLimit},
};

const CVarRange* findCVarRange(lua_State* L) {
    const char* name = lua_tostring(L, 1);
    if (!name) return nullptr;
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (const auto& r : kCVarRanges) {
        if (lower == r.cvar) return &r;
    }
    return nullptr;
}

static int lua_GetCVarMin(lua_State* L) {
    if (const auto* r = findCVarRange(L)) {
        lua_pushstring(L, ui::settingNumberText(r->minValue).c_str());
        return 1;
    }
    return luaReturnNil(L);
}

static int lua_GetCVarMax(lua_State* L) {
    if (const auto* r = findCVarRange(L)) {
        lua_pushstring(L, ui::settingNumberText(r->maxValue).c_str());
        return 1;
    }
    return luaReturnNil(L);
}

// ---- Voice chat ----
//
// There is no voice chat in this client and no plan for one. These are answered
// rather than left missing because the panels that ask are otherwise perfectly
// usable: the audio options page reads the microphone level from an OnUpdate,
// so one absent global there raises every frame the page is open.
static int lua_IsVoiceChatAllowedByServer(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_VoiceChat_IsRecordingLoopbackSound(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_VoiceChat_IsPlayingLoopbackSound(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_VoiceChat_GetCurrentMicrophoneSignalLevel(lua_State* L) { lua_pushnumber(L, 0); return 1; }

// GetVoiceSessionInfo(i) → name, active
static int lua_GetVoiceSessionInfo(lua_State* L) {
    lua_pushnil(L);
    lua_pushnil(L);
    return 2;
}

static int lua_GetNumVoiceSessionMembersBySessionID(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

// RequestRaidInfo() - ask the server for saved instance lockouts. The reply is
// SMSG_RAID_INSTANCE_INFO, which the client already parses for
// GetSavedInstanceInfo; nothing was asking for it.
static int lua_RequestRaidInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->requestRaidInfo();
    return 0;
}

// CanShowAchievementUI() → whether the achievement panel may open
static int lua_CanShowAchievementUI(lua_State* L) {
    lua_pushboolean(L, 1);
    return 1;
}

// GetAddOnMemoryUsage(index) → kilobytes, and UpdateAddOnMemoryUsage() to
// refresh them
//
// Zero rather than nothing. The performance bar's tooltip adds these up -
// `totalMem = totalMem + mem` for every addon loaded - and nil there is an
// error, on an interface element that is drawn by default. Nothing here
// measures per-addon memory, and zero is what an unmeasured addon costs as far
// as this client knows.
static int lua_GetAddOnMemoryUsage(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

// IsXPUserDisabled() → whether the player has turned experience off
//
// Nothing here can turn it off, so this is a definite no rather than an absent
// answer - the reputation panel reads it to decide whether to offer the bar.
static int lua_IsXPUserDisabled(lua_State* L) {
    lua_pushboolean(L, 0);
    return 1;
}

/// The taxi nodes the flight map is showing, in a fixed order.
///
/// Every taxi function is asked about a node by position in this list, and the
/// answers only line up if each of them walks the same order. They used to walk
/// `getTaxiNodes()` directly, which is an unordered_map: its order is arbitrary,
/// and rehashing it - which learning a flight path while the window is open
/// does - reorders it under a list already drawn. Clicking a destination would
/// then fly somewhere else. Sorting by node id costs a copy of a few hundred
/// integers and makes the order the same every time it is asked.
///
/// Only the nodes on the map the player is standing on are listed. That is what
/// the flight map shows, and it is what this client's own window already lists.
static std::vector<uint32_t> taxiNodeOrder(game::GameHandler* gh) {
    std::vector<uint32_t> ids;
    if (!gh) return ids;

    const auto& nodes = gh->getTaxiNodes();
    const auto current = nodes.find(gh->getTaxiCurrentNode());
    if (current == nodes.end()) return ids;

    const uint32_t mapId = current->second.mapId;
    ids.reserve(nodes.size());
    for (const auto& [id, node] : nodes) {
        if (node.mapId == mapId) ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

/// The node at a one-based position on the flight map, or 0 for no such node.
static uint32_t taxiNodeAt(game::GameHandler* gh, int index) {
    const auto ids = taxiNodeOrder(gh);
    if (index < 1 || index > static_cast<int>(ids.size())) return 0;
    return ids[static_cast<size_t>(index - 1)];
}

/// Where a taxi node sits on the flight map, as a fraction of its width and
/// height.
///
/// The flight map is the continent's map, so this is the same projection the
/// world map does - the one place it lives, in rendering/world_map, reached
/// through the continent rectangle the game handler now reads.
///
/// The order of the two conversions is the whole difficulty. TaxiNodes.dbc
/// holds positions in server order, and feeding those straight to
/// canonicalToRender transposes every marker, which is exactly how the flight
/// map's node markers came out mirrored once before. Server to canonical
/// first, then canonical to render, then project.
static bool taxiNodeMapPos(game::GameHandler* gh, uint32_t nodeId,
                           float& outU, float& outV) {
    if (!gh) return false;
    const auto& nodes = gh->getTaxiNodes();
    const auto it = nodes.find(nodeId);
    if (it == nodes.end()) return false;

    const auto& bounds = gh->getContinentBounds(it->second.mapId);
    if (!bounds.valid) return false;

    const glm::vec3 canonical = core::coords::serverToCanonical(
        glm::vec3(it->second.x, it->second.y, it->second.z));
    const glm::vec3 render = core::coords::canonicalToRender(canonical);

    rendering::world_map::ZoneBounds zb;
    zb.locLeft = bounds.left;   zb.locRight  = bounds.right;
    zb.locTop  = bounds.top;    zb.locBottom = bounds.bottom;
    const glm::vec2 uv =
        rendering::world_map::renderPosToMapUV(render, zb, /*isContinent=*/true);
    outU = uv.x;
    // Measured up from the bottom, which is not the convention the rest of the
    // interface uses and is the whole of why the flight map was wrong.
    //
    // The world map anchors what it places to TOPLEFT and negates the y it was
    // given - worldmapframe.lua:789 - so GetPlayerMapPosition counts down from
    // the top. The flight map anchors to BOTTOMLEFT and adds it, for the nodes
    // (taxiframe.lua:80) and for both ends of every route leg (140-142,
    // 179-181), so TaxiNodePosition counts up from the bottom.
    //
    // renderPosToMapUV answers the world map's way, so handing it over
    // unchanged mirrored the map vertically: Rut'theran sits 0.17 down from the
    // top of Kalimdor and was drawn 0.17 up from the bottom, which is Tanaris.
    // Every node was over somewhere it did not belong, which is what made them
    // read as the wrong places rather than as merely misplaced.
    outV = 1.0f - uv.y;
    return true;
}

/// One end of one leg of a flight, as a fraction of the map.
///
/// `End` is 0 for the leg's start and 1 for its finish; `Horizontal` picks x
/// over y. Four names, one body - they differ only in which number they read.
template <int End, bool Horizontal>
static int lua_TaxiLegCoord(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t dest = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    const int hop = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (dest == 0 || hop < 1 || !gh) { lua_pushnumber(L, 0); return 1; }

    const auto route = gh->getTaxiRouteTo(dest);
    // Hop N runs from route[N-1] to route[N], so the last hop is size-1.
    if (hop >= static_cast<int>(route.size())) { lua_pushnumber(L, 0); return 1; }

    float u = 0, v = 0;
    if (!taxiNodeMapPos(gh, route[static_cast<size_t>(hop - 1 + End)], u, v)) {
        lua_pushnumber(L, 0);
        return 1;
    }
    lua_pushnumber(L, Horizontal ? u : v);
    return 1;
}

/// The hops of the route the flight map is drawing lines for.
///
/// TaxiNodeSetCurrent names the node the player is hovering, and the frame then
/// asks for each leg of the journey to it in turn. Kept here rather than asked
/// for again per leg because getTaxiRouteTo searches, and the frame asks for the
/// same route once per hop and again for every line it draws.
static std::vector<uint32_t>& taxiRouteShown() {
    static std::vector<uint32_t> route;
    return route;
}

void registerSystemLuaAPI(lua_State* L) {
    // Before any binding is registered, so the first GetCVar of the run - which
    // happens while a panel is being built - sees what the player last set.
    loadStoredCVars();
    loadInterfaceState();
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"Screenshot",               lua_Screenshot},
                {"WoweeShowSettings",        lua_WoweeShowSettings},
                {"WoweeSettingList",         lua_WoweeSettingList},
                {"WoweeVersion",             lua_WoweeVersion},
                {"WoweeGetSetting",          lua_WoweeGetSetting},
                {"WoweeSetSetting",          lua_WoweeSetSetting},
                {"HasLFGRestrictions",       lua_HasLFGRestrictions},
                {"GetLFGProposal",           lua_GetLFGProposal},
                {"GetLFGInfoServer",         lua_GetLFGInfoServer},
                {"GetLFGRoleUpdate",         lua_GetLFGRoleUpdate},
                {"IsListedInLFR",            lua_IsListedInLFR},
                {"IsPartyLFG",               lua_IsPartyLFG},
                {"GetLFGDeserterExpiration", lua_GetLFGDeserterExpiration},
                {"GetLFGRandomCooldownExpiration", lua_GetLFGRandomCooldownExpiration},
                {"RefreshLFGList",           lua_RefreshLFGList},
                {"GetTrackedAchievements",   lua_GetTrackedAchievements},
                {"RequestRaidInfo",          lua_RequestRaidInfo},
                {"IsPVPTimerRunning",        lua_IsPVPTimerRunning},
                {"GetPVPTimer",              lua_GetPVPTimer},
                {"GetCurrentArenaSeason",    lua_GetCurrentArenaSeason},
                {"GetPVPRankProgress",       lua_GetPVPRankProgress},
                {"ArenaTeamRoster",              lua_ArenaTeamRoster},
                {"GetArenaTeamRosterInfo",       lua_GetArenaTeamRosterInfo},
                // How many rows that reader has. Unbound, so opening a team's
                // details called a nil global and raised before the first row
                // was read - the reader beside it worked the whole time.
                {"GetNumArenaTeamMembers", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t teamId =
                arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            const auto* roster = (gh && teamId) ? gh->getArenaTeamRoster(teamId) : nullptr;
            lua_pushnumber(L, roster
                ? static_cast<lua_Number>(roster->members.size()) : 0);
            return 1;
        }},
                // SortArenaTeamRoster(column) - the details frame's column
                // headers. Unbound, every one of them raised on click.
                {"SortArenaTeamRoster", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* key = luaL_optstring(L, 1, "");
            if (gh && key && *key) gh->sortArenaTeamRosters(key);
            return 0;
        }},
                {"GetArenaTeamRosterSelection",  lua_GetArenaTeamRosterSelection},
                {"SetArenaTeamRosterSelection",  lua_SetArenaTeamRosterSelection},
                {"CloseArenaTeamRoster",         lua_CloseArenaTeamRoster},
                {"IsArenaTeamCaptain",           lua_IsArenaTeamCaptain},
                {"CloseBattlefield",             lua_CloseBattlefield},
                {"CanExitVehicle",               lua_CanExitVehicle},
                {"IsVehicleAimAngleAdjustable",  lua_IsVehicleAimAngleAdjustable},
                {"HasKey",                       lua_HasKey},
                {"GetArenaTeam",             lua_GetArenaTeam},
                {"GetRandomBGHonorCurrencyBonuses",  lua_BattlegroundHonorBonusesNone},
                {"GetHolidayBGHonorCurrencyBonuses", lua_BattlegroundHonorBonusesNone},
                {"GetBattlefieldInstanceInfo",       lua_GetBattlefieldInstanceInfo},
                {"GetNumBattlefields",       lua_GetNumBattlefields},
                // Sorting the battleground list. This client sorts its own, so
                // there is nothing to do - but the name has to exist, because
                // PVPBattlegroundFrame_OnShow calls it and a nil global raises
                // as the panel opens.
                {"SortBGList",               lua_ReturnNothing},
                // Stationery for a letter. None is carried, and the picker
                // draws the default when the count is zero - which it could
                // not do while the count raised.
                // One: the plain parchment. Zero left the stationery popup
                // empty, and SendMailFrame_CanSend will not enable the Send
                // button until a stationery has been picked - so with none to
                // pick, no letter could be sent. See GetStationeryInfo.
                {"GetNumStationeries",       [](lua_State* L) -> int {
                    lua_pushnumber(L, 1);
                    return 1;
                }},
                // Voice mutes, which this client has no voice chat to keep.
                // Read while the ignore list is drawn, so a nil global took
                // the whole ignore tab with it.
                {"GetNumMutes",              lua_ReturnZero},
                {"IsInLFGDungeon",           lua_IsInLFGDungeon},
                {"LFGTeleport",              lua_LFGTeleport},
                {"IsBattlefieldArena",       lua_IsBattlefieldArena},
                {"IsActiveBattlefieldArena", lua_IsActiveBattlefieldArena},
                {"CanHearthAndResurrectFromArea", lua_CanHearthAndResurrectFromArea},
                {"GetWorldPVPQueueStatus",   lua_GetWorldPVPQueueStatus},
                {"LeaveBattlefield",         lua_LeaveBattlefield},
                {"CreateWorldMapArrowFrame",   lua_CreateWorldMapArrowFrame},
                {"ShowWorldMapArrowFrame",     lua_ShowWorldMapArrowFrame},
                {"PositionWorldMapArrowFrame", lua_PositionWorldMapArrowFrame},
                {"InitWorldMapPing",           lua_InitWorldMapPing},
                {"CreateMiniWorldMapArrowFrame",   lua_CreateMiniWorldMapArrowFrame},
                {"PositionMiniWorldMapArrowFrame", lua_PositionMiniWorldMapArrowFrame},
                {"ShowMiniWorldMapArrowFrame",     lua_ShowMiniWorldMapArrowFrame},
                {"GetBattlefieldMapIconScale",     lua_GetBattlefieldMapIconScale},
                {"PlayerIsPVPInactive",            lua_PlayerIsPVPInactive},
                {"CombatTextSetActiveUnit",        lua_CombatTextSetActiveUnit},
                {"CombatLogGetNumEntries",         lua_CombatLogGetNumEntries},
                {"CombatLogGetCurrentEntry",       lua_CombatLogGetCurrentEntry},
                {"CombatLogAdvanceEntry",          lua_CombatLogAdvanceEntry},
                {"CombatLogSetCurrentEntry",       lua_CombatLogSetCurrentEntry},
                {"CombatLogAddFilter",             lua_CombatLogAddFilter},
                {"CombatLogResetFilter",           lua_CombatLogResetFilter},
                {"GetBattlefieldInfo",       lua_GetBattlefieldInfo},
                {"SetSelectedBattlefield",   lua_SetSelectedBattlefield},
                {"GetSelectedBattlefield",   lua_GetSelectedBattlefield},
                {"JoinBattlefield",          lua_JoinBattlefield},
                {"GetLFGCompletionReward",     lua_GetLFGCompletionReward},
                {"GetLFGCompletionRewardItem", lua_GetLFGCompletionRewardItem},
                {"RunMacroText",             lua_RunMacroText},
                // This client's own slash commands, for the bootstrap chunk
                // that puts them into SlashCmdList. Not WoW API - the names
                // are prefixed so nothing in FrameXML can collide with them.
                {"__WoweeClientCommandNames", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            lua_newtable(L);
            if (!svc || !svc->clientChatCommandNames) return 1;
            int i = 1;
            for (const auto& name : svc->clientChatCommandNames()) {
                lua_pushstring(L, name.c_str());
                lua_rawseti(L, -2, i++);
            }
            return 1; }},
                {"__WoweeRunClientCommand", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const char* alias = luaL_optstring(L, 1, "");
            const char* args  = luaL_optstring(L, 2, "");
            bool ok = false;
            if (svc && svc->runClientChatCommand && alias && *alias)
                ok = svc->runClientChatCommand(alias, args);
            lua_pushboolean(L, ok ? 1 : 0);
            return 1; }},
                {"RunMacro",                 lua_RunMacro},
                {"TriggerTutorial",          lua_TriggerTutorial},
                {"Quit",                     lua_Quit},
                {"ForceQuit",                lua_ForceQuit},
                {"ForceLogout",              lua_ForceLogout},
                {"ReloadUI",                 lua_ReloadUI},
                {"GetGamma",                 lua_GetGamma},
                {"GetTerrainMip",            lua_GetTerrainMip},
                {"SetTerrainMip",            lua_SetTerrainMip},
                {"SetGamma",                 lua_SetGamma},
                {"WoweeReportMissingFixedControls",
                                             lua_WoweeReportMissingFixedControls},
                {"GetVideoCaps",             lua_GetVideoCaps},
                {"GetCVarMin",               lua_GetCVarMin},
                {"GetCVarMax",               lua_GetCVarMax},
                {"IsVoiceChatAllowedByServer", lua_IsVoiceChatAllowedByServer},
                {"VoiceChat_IsRecordingLoopbackSound", lua_VoiceChat_IsRecordingLoopbackSound},
                {"VoiceChat_IsPlayingLoopbackSound",   lua_VoiceChat_IsPlayingLoopbackSound},
                {"VoiceChat_GetCurrentMicrophoneSignalLevel", lua_VoiceChat_GetCurrentMicrophoneSignalLevel},
                {"GetVoiceSessionInfo",      lua_GetVoiceSessionInfo},
                {"GetNumVoiceSessionMembersBySessionID", lua_GetNumVoiceSessionMembersBySessionID},
                {"CanShowAchievementUI",     lua_CanShowAchievementUI},
                {"IsXPUserDisabled",         lua_IsXPUserDisabled},
                {"GetAddOnMemoryUsage",      lua_GetAddOnMemoryUsage},
                {"UpdateAddOnMemoryUsage",   lua_ReturnNothing},
                {"RunScript",                lua_RunScript},
                {"IsMouseButtonDown",        lua_IsMouseButtonDown},
                {"GetCVarDefault",           lua_GetCVarDefault},
                {"IsAddOnLoaded",            lua_IsAddOnLoaded},
                {"LoadAddOn",                lua_LoadAddOn},
                {"UIParentLoadAddOn",        lua_LoadAddOn},
                // This is the whole of whether the achievement micro button is
                // clickable: mainmenubarmicrobuttons.lua disables it unless
                // this and CanShowAchievementUI both answer yes, and the other
                // already did. A flat false left the button greyed out while
                // the client knew exactly which achievements were earned.
                {"HasCompletedAnyAchievement", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && !gh->getEarnedAchievements().empty() ? 1 : 0);
            return 1;
        }},
                {"TurnInGuildCharter",       lua_ReturnNothing},
                // Nothing is being driven, so aiming it does nothing
                // and there is nothing to climb out of.
                {"VehicleAimUpStart",        lua_ReturnNothing},
                {"VehicleAimUpStop",         lua_ReturnNothing},
                {"VehicleAimDownStart",      lua_ReturnNothing},
                {"VehicleAimDownStop",       lua_ReturnNothing},
                // The button and the slash command both end here, and it did
                // nothing - so /leavevehicle, the main bar's button and the
                // unit menu's entry were three ways of not getting off.
                // CMSG_REQUEST_VEHICLE_EXIT was already written and had no
                // caller outside this client's own bar.
                {"VehicleExit", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->sendRequestVehicleExit();
            return 0;
        }},
                {"VehicleAimGetNormAngle",   lua_ReturnZero},
                {"VehicleAimGetNormPower",   lua_ReturnZero},
                {"GetMapInfo",               lua_GetMapInfo},
                {"GetExpansionLevel",        lua_GetExpansionLevel},
                // The difficulty the player is set to, which this client is
                // told by SMSG_INSTANCE_DIFFICULTY and kept answering as one.
                // A dropdown reading a constant shows the wrong tick and, worse,
                // sends a change nobody asked for when the menu is opened.
                // The wire counts difficulties from zero and the interface from
                // one: DUNGEON_DIFFICULTY_NORMAL is 0 on AzerothCore, and the
                // popup checks its menu entry with `GetDungeonDifficulty() ==
                // index`, index being the 1-based row. Answering the stored
                // value straight made Heroic read as Normal - `d ? d : 1` turns
                // both 0 and 1 into 1 - so the tick sat on the wrong row and
                // unitpopup's `GetDungeonDifficulty() == 1` heroic-lockout
                // check was true inside a heroic.
                //
                // Both answer from the one value this client tracks. Only one
                // of the two applies at a time, so that holds until a party
                // sets a raid difficulty while standing in a dungeon.
                {"GetDungeonDifficulty", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, (gh ? gh->getInstanceDifficulty() : 0u) + 1u);
            return 1;
        }},
                {"GetRaidDifficulty", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, (gh ? gh->getInstanceDifficulty() : 0u) + 1u);
            return 1;
        }},
                {"GetChatTypeIndex",         lua_ReturnOne},
                {"GetDefaultLanguage",       lua_GetDefaultLanguage},
                {"GetWeaponEnchantInfo",     lua_GetWeaponEnchantInfo},
                // The PvP reclaim timer, which this client already tracks.
                // Stubbed to zero this read as "reclaim now" and made the
                // delay text on FrameXML's corpse prompt always empty.
                {"GetCorpseRecoveryDelay", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getCorpseReclaimDelaySec() : 0.0);
            return 1;
        }},
                {"GetAdjustedSkillPoints",   lua_ReturnZero},
                // Which party slot holds the leader, in the same 1-to-4
                // ordering resolveUnitGuid gives "party1".."party4": members
                // other than the player, in order. Zero means the player leads
                // it, which is what WoW answers and what the crown reads as
                // "not on this frame".
                //
                // A constant zero meant PartyMemberFrame_UpdateLeader hid the
                // icon on every frame it ran for, so no party ever showed who
                // was leading it - while partyData has carried leaderGuid all
                // along.
                {"GetPartyLeaderIndex", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto& pd = gh->getPartyData();
            if (pd.leaderGuid == 0 || pd.leaderGuid == gh->getPlayerGuid()) {
                lua_pushnumber(L, 0);
                return 1;
            }
            int slot = 0;
            for (const auto& m : pd.members) {
                if (m.guid == gh->getPlayerGuid()) continue;
                ++slot;
                if (m.guid == pd.leaderGuid) { lua_pushnumber(L, slot); return 1; }
            }
            lua_pushnumber(L, 0);
            return 1;
        }},
                {"GetNumArenaOpponents",     lua_ReturnZero},
                {"GetCurrentMultisampleFormat", lua_GetCurrentMultisampleFormat},
                // These hand back a list, not a value: the caller walks it with
                // select("#", ...) and reads it in groups. One number makes the
                // loop run once against nils, which is worse than an empty
                // list - for anything returning a list, nothing is the right
                // way to say there is none.
                // The anti-aliasing dropdown, which used to be left empty on
                // the argument that offering modes was worse than offering
                // none. Empty was worse than either: UIDropDownMenu_Refresh
                // walks the shared DropDownList1 buttons, so a dropdown that
                // adds none of its own reads whichever list was built last and
                // takes its text. Multisampling showed "1280x720 (Wide)",
                // copied from the Resolution dropdown three controls away.
                //
                // The four modes are real and are the same four this client's
                // own panel offers, so choosing one applies it. MSAA is moot
                // while FSR is upscaling - which is why the client's own combo
                // disables itself there - but the choice is still recorded and
                // still takes effect when FSR is off, which is what that combo
                // does with it too.
                {"GetMultisampleFormats",    lua_GetMultisampleFormats},
                {"GetRefreshRates",          lua_GetRefreshRates},
                // ---- The options panels behind the game menu ----
                //
                // Every reader here was bound and none of the writers were, so
                // opening Video or Sound and touching anything raised. The
                // panels sit one click inside the game menu, which is a handed-
                // over element, and none of their files was scanned by the
                // readiness report until the question "which FrameXML files
                // does no element reach" was asked.
                //
                // The answers are shaped by what the readers already say.
                // GetScreenResolutions offers exactly one mode -- the window
                // this client is already in -- and GetMultisampleFormats offers
                // none, so choosing from either list can only ever re-choose
                // what is set. These accept the call and change nothing, which
                // is the truth rather than a stub.
                {"SetScreenResolution",      lua_SetScreenResolution},
                {"SetMultisampleFormat",     lua_SetMultisampleFormat},
                // Resolution and windowed mode are settable now, so this one
                // has something to put back and does not do it. Left rather
                // than guessed: the defaults live as function-local constants
                // in the settings panel, and a Restore that picks its own would
                // disagree with the Defaults button beside it.
                {"RestoreVideoResolutionDefaults", lua_ReturnNothing},
                {"RestoreVideoEffectsDefaults",    lua_ReturnNothing},
                {"RestoreVideoStereoDefaults",     lua_ReturnNothing},
                // Applying video settings restarts the graphics device on the
                // real client. This one applies what it can as it goes and has
                // no device to tear down.
                // RestartGx() - apply the display CVars marked `restart`.
                //
                // The video panel writes every changed CVar and then, if any of
                // them wanted it, restarts the device once so they take effect
                // together; gxWindow is one of those, so the SetCVar that
                // precedes this is deliberately not where the change lands.
                // A no-op here meant ticking Windowed wrote a value and left
                // the window exactly as it was.
                //
                // Only gxWindow is acted on. gxResolution is a dropdown whose
                // value format has not been read off the control here, and a
                // resolution applied from a misread string is a window nobody
                // can put back.
                {"RestartGx", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            if (!svc || !svc->setFullscreen) return 0;
            lua_getglobal(L, "GetCVar");
            lua_pushstring(L, "gxWindow");
            if (lua_pcall(L, 1, 1, 0) != 0) { lua_pop(L, 1); return 0; }
            const char* windowed = lua_tostring(L, -1);
            const bool wantFullscreen = windowed && std::string(windowed) == "0";
            lua_pop(L, 1);
            svc->setFullscreen(wantFullscreen);
            return 0;
        }},
                {"Sound_GameSystem_RestartSoundSystem", lua_ReturnNothing},
                // A separate render scale for the player model, which this
                // client does not have. False disables the control rather than
                // leaving it offering something that would do nothing.
                {"IsPlayerResolutionAvailable", lua_ReturnFalse},
                // Which extra action bars are shown. FrameXML draws the bars
                // and MultiActionBar_Update decides from the SHOW_MULTI_ACTIONBAR_*
                // globals, which are CVars and persist on their own - so the
                // C-side toggle this mirrors has nothing left to do here.
                // Set/GetActionBarToggles - which of the four extra action
                // bars are shown.
                //
                // The setter was a no-op and the getter was not bound at all.
                // Each of the four checkboxes in Interface Options reads its
                // own state with
                //     self.value or ((select(n, GetActionBarToggles()) ...
                // and self.value is nil until something sets it, so opening
                // that panel called nil four times.
                // The Interface Options checkboxes for the extra bars.
                //
                // These were stored here and nowhere else. This client draws
                // its own action bars and suppresses FrameXML's MultiBar
                // frames, so the toggle changed a number that only this file
                // read: ticking "Bottom Left Bar" did nothing at all, which is
                // the whole of "no way to add action bars".
                //
                // Only the two the client has a bar for are passed on. Bottom
                // Right and Right Bar 2 have nothing to drive, so they are
                // still remembered but move nothing - saying so here rather
                // than mapping them onto a bar that is not theirs.
                {"SetActionBarToggles", [](lua_State* L) -> int {
            auto& shown = actionBarToggles();
            shown[0] = lua_toboolean(L, 1) != 0;   // Bottom Left
            shown[1] = lua_toboolean(L, 2) != 0;   // Bottom Right
            shown[2] = lua_toboolean(L, 3) != 0;   // Right
            shown[3] = lua_toboolean(L, 4) != 0;   // Right 2
            // The fifth is ALWAYS_SHOW_MULTIBARS, which asks for the empty
            // slots of a shown bar to stay visible. This client draws its bars
            // whole, so there is nothing for it to change - read here so it is
            // plainly ignored rather than silently dropped.
            const bool alwaysShow = lua_toboolean(L, 5) != 0;
            (void)alwaysShow;
            saveInterfaceState();
            // The fourth is this client's left bar, despite being labelled
            // "Right Bar 2" where the player ticks it: the interface's fourth
            // multi-bar is MultiBarLeft, which is action page 4, which is the
            // upright bar this client draws at the left edge. Without it the
            // tick did nothing and the bar could only be turned on from this
            // client's own panel, where its checkbox then disagreed.
            if (auto* svc = getLuaServices(L); svc && svc->setClientSetting) {
                svc->setClientSetting("showbar2",     shown[0] ? "1" : "0");
                svc->setClientSetting("showrightbar", shown[2] ? "1" : "0");
                svc->setClientSetting("showleftbar",  shown[3] ? "1" : "0");
            }
            return 0;
        }},
                // Read back from the client's own settings for the two that
                // drive a bar, so the checkbox shows what is on screen rather
                // than what this file last stored. The two disagreed whenever
                // the bar was turned on from the client's own settings window.
                {"GetActionBarToggles", [](lua_State* L) -> int {
            auto shown = actionBarToggles();
            if (auto* svc = getLuaServices(L); svc && svc->getClientSetting) {
                const auto readBool = [&svc](const char* key, bool fallback) {
                    const std::string v = svc->getClientSetting(key);
                    return v.empty() ? fallback : (v != "0");
                };
                shown[0] = readBool("showbar2", shown[0]);
                shown[2] = readBool("showrightbar", shown[2]);
                shown[3] = readBool("showleftbar", shown[3]);
            }
            for (bool on : shown) lua_pushboolean(L, on ? 1 : 0);
            return 4;
        }},
                // Voice chat, which this client has none of. The enumerations
                // hand back lists, so they answer with nothing rather than with
                // a zero; IsVoiceChatEnabled already answers false beside them.
                {"VoiceIsDisabledByClient",  lua_ReturnTrue},
                {"VoiceEnumerateCaptureDevices", lua_ReturnNothing},
                {"VoiceEnumerateOutputDevices",  lua_ReturnNothing},
                {"VoiceSelectCaptureDevice", lua_ReturnNothing},
                {"VoiceSelectOutputDevice",  lua_ReturnNothing},
                {"VoiceChat_StopPlayingLoopbackSound",   lua_ReturnNothing},
                {"VoiceChat_StopRecordingLoopbackSound", lua_ReturnNothing},
                // GetCompanionInfo(type, index) → creatureID, creatureName,
                // spellID, icon, active
                //
                // "MOUNT" or "CRITTER". Both are spells the player knows, told
                // apart by what the spell does - see rebuildCompanions.
                {"GetCompanionInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* kind = luaL_optstring(L, 1, "");
            const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || index < 1) return luaReturnNil(L);
            const bool mounts = std::string(kind) == "MOUNT";
            const auto& list = gh->getCompanions(mounts);
            if (index > static_cast<int>(list.size())) return luaReturnNil(L);
            const auto& c = list[static_cast<size_t>(index) - 1];
            // A display id, and both kinds have to be asked for one. What is
            // held is a creature *template entry* either way: the summon's
            // misc value for a critter, and the mounted aura's for a mount -
            // AzerothCore reads that one with GetCreatureTemplate and picks
            // the model with ChooseDisplayId, so it is an entry there too.
            //
            // A mount's was handed to CompanionModelFrame:SetCreature
            // unresolved, on the reading that it was already a display id. An
            // entry used as a display id names a real CreatureDisplayInfo row
            // belonging to something else, so the mount tab previewed whatever
            // creature happened to sit at that row.
            //
            // Zero until the answer arrives, which draws nothing for a round
            // trip rather than drawing the wrong thing; COMPANION_UPDATE brings
            // the tab back when it lands.
            const uint32_t displayId = gh->getCreatureDisplayIdForEntry(c.creatureId);
            lua_pushnumber(L, displayId);
            lua_pushstring(L, c.name.c_str());
            lua_pushnumber(L, c.spellId);
            lua_pushstring(L, gh->getSpellIconPath(c.spellId).c_str());
            // Whether it is out. A mount is an aura on the player; a critter is
            // the pet that is following, and neither is tracked per companion -
            // so this reads from what is actually active rather than from a
            // flag nobody sets.
            bool active = false;
            for (const auto& a : gh->getPlayerAuras()) {
                if (a.spellId == c.spellId) { active = true; break; }
            }
            lua_pushboolean(L, active ? 1 : 0);
            return 5;
        }},
                // Counts, and the count is the whole point: each of these is
                // read straight into `for i = 1, X()`, where a nil limit is
                // not an empty loop but an error - "'for' limit must be a
                // number" - that takes down the handler around it. Unbound,
                // the fallback answered nil and the character sheet's title
                // list, the companion tab and the token frame each raised
                // rather than showing nothing.
                //
                // Zero is the honest answer, not a placeholder: no known-title
                // bitmask, companion list or currency list is tracked here, so
                // there is genuinely nothing to count. They stop being zero
                // when something starts reading that data, and the frames
                // above will fill themselves in when it does.
                // GetNumTitles() - how many title *bits* there are to ask
                // about, not how many are owned. paperdollframe.lua walks
                // 1..GetNumTitles() calling IsTitleKnown on each, so this is
                // the size of the space: KNOWN_TITLES_SIZE * 64 in
                // AzerothCore's Player.h, three uint64 fields.
                {"GetNumTitles", [](lua_State* L) -> int {
            lua_pushnumber(L, 192); return 1;
        }},
                {"GetNumCompanions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* kind = luaL_optstring(L, 1, "");
            const bool mounts = std::string(kind) == "MOUNT";
            lua_pushnumber(L, gh ? static_cast<double>(gh->getCompanions(mounts).size()) : 0.0);
            return 1;
        }},
                // The knowledge base is the server's FAQ, and there is no
                // server here answering for it. Its category dropdown is
                // reached from the "?" micro button beside the action bar, so
                // the raise was one click away rather than in a corner.
                {"KBSetup_GetCategoryCount",    lua_ReturnZero},
                {"KBSetup_GetSubCategoryCount", lua_ReturnZero},
                // The other thirteen, which the note above should have covered
                // and did not: two counts were bound and the rest of the same
                // window was not, so the "?" button still raised - on
                // KBSetup_BeginLoading, which KnowledgeBaseFrame_OnShow calls
                // before anything else.
                //
                // Every caller here guards, and guards on exactly what an
                // absent knowledge base should say. KnowledgeBaseFrame_Search
                // returns early unless KBSetup_IsLoaded, the MOTD and notice
                // are both `if ( x )`, and the article lists are walked by the
                // counts. Never loaded, nothing in it.
                {"KBSetup_IsLoaded",            lua_ReturnFalse},
                {"KBSetup_BeginLoading",        lua_ReturnNothing},
                {"KBQuery_BeginLoading",        lua_ReturnNothing},
                {"KBArticle_BeginLoading",      lua_ReturnNothing},
                {"KBSetup_GetArticleHeaderCount", lua_ReturnZero},
                {"KBSetup_GetTotalArticleCount",  lua_ReturnZero},
                {"KBQuery_GetArticleHeaderCount", lua_ReturnZero},
                {"KBQuery_GetTotalArticleCount",  lua_ReturnZero},
                {"KBSetup_GetCategoryData",     lua_ReturnNil},
                {"KBSetup_GetSubCategoryData",  lua_ReturnNil},
                {"KBArticle_GetData",           lua_ReturnNil},
                {"KBSystem_GetMOTD",            lua_ReturnNil},
                {"KBSystem_GetServerNotice",    lua_ReturnNil},
                // Three more counts read straight into a comparison or a
                // format, with the same result: the socketing window walks
                // `i <= numSockets`, the PvP frame formats the season number
                // into its off-season line, and the achievement comparison
                // concatenates its total. None of the three has data behind it
                // here - no socketing, no arena seasons, and no way to read
                // another player's achievements - so zero is what is true.
                // Three battlefield timers, all read as `X()/1000`. This
                // client has partial battlefield support - status, score,
                // winner and positions are all bound - so these are gaps in a
                // system a player reaches rather than one that does not exist,
                // and each raises the moment a battleground is queued for or
                // entered. Zero reads correctly at every call site: no time in
                // queue, no shutdown pending, no elapsed run time.
                // Both timers are read as X()/1000, so milliseconds. The
                // queue slot carries each in seconds and this answered zero for
                // both - a queue that always read as just-joined with no
                // estimate, which is the whole content of that window.
                {"GetBattlefieldTimeWaited", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 1));
            double ms = 0.0;
            if (gh && index >= 1 && index <= 3)
                ms = gh->getBgQueues()[static_cast<size_t>(index - 1)].timeInQueueSec * 1000.0;
            lua_pushnumber(L, ms);
            return 1;
        }},
                {"GetBattlefieldEstimatedWaitTime", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 1));
            double ms = 0.0;
            if (gh && index >= 1 && index <= 3)
                ms = gh->getBgQueues()[static_cast<size_t>(index - 1)].avgWaitTimeSec * 1000.0;
            lua_pushnumber(L, ms);
            return 1;
        }},
                // CanJoinBattlefieldAsGroup() - whether the queue button offers
                // to take the party in.
                //
                // The server decides for itself when the request arrives; this
                // only says whether it is worth offering, which is a party the
                // player leads. Solo, there is no group to bring.
                {"CanJoinBattlefieldAsGroup", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto& pd = gh->getPartyData();
            const bool leadsAParty = !pd.isEmpty() && pd.leaderGuid == gh->getPlayerGuid();
            lua_pushboolean(L, leadsAParty ? 1 : 0);
            return 1;
        }},
                {"GetBattlefieldInstanceExpiration", lua_ReturnZero},
                {"GetBattlefieldInstanceRunTime",    lua_ReturnZero},
                // The aspect ratio the Mac options panel builds its recording
                // resolution from: `"640x"..floor(640*ratio)`. Answering the
                // window's own ratio is both truthful and what makes that read
                // 640x360 on a 16:9 display rather than raising.
                // The rest of the movie recorder, which this client does not
                // have. Not stubs standing in for something - "we are not
                // recording" is simply true, and the two toggles have nothing
                // to toggle.
                //
                // Bound because they hang off keybindings. Those are declared
                // platform="mac" and so should never be reachable here, but a
                // binding is dispatched by name at the moment a key is pressed
                // and answering nothing there raises in the key handler, which
                // is a bad place to find out the filter was not applied.
                {"MovieRecording_IsRecording",   lua_ReturnFalse},
                {"MovieRecording_IsCompressing", lua_ReturnFalse},
                {"MovieRecording_Toggle",        lua_ReturnNothing},
                {"MovieRecording_ToggleGUI",     lua_ReturnNothing},
                {"MovieRecording_Cancel",        lua_ReturnNothing},
                {"MovieRecording_GetAspectRatio", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            auto* win = svc ? svc->window : nullptr;
            const float w = win ? static_cast<float>(win->getWidth())  : 1920.0f;
            const float h = win ? static_cast<float>(win->getHeight()) : 1080.0f;
            lua_pushnumber(L, (w > 0.0f) ? (h / w) : 0.5625);
            return 1;
        }},
                // GetNumSockets answered zero here, which said every item has
                // no sockets. It is real now, and in lua_socket_api.cpp.
                {"GetPreviousArenaSeason",      lua_ReturnZero},
                // GetInstanceBootTimeRemaining() - the countdown on the
                // "you are not in this instance's group" dialog, which reads
                // it on show and hides itself when it is not positive.
                {"GetInstanceBootTimeRemaining", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getInstanceBootTimeRemaining() : 0);
            return 1;
        }},
                // The tutorial popups. Both raise where they are called -
                // FlagTutorial from TutorialFrame_Update as a tutorial is
                // shown, IsTutorialFlagged from TutorialFrame_NewTutorial
                // before one is queued.
                {"FlagTutorial", [](lua_State* L) -> int {
            flagTutorial(static_cast<int>(luaL_optnumber(L, 1, 0)));
            return 0;
        }},
                // Walking the tutorials already seen. Both answer an id or
                // nothing, and nothing is what disables the button - so a
                // wrong answer here is a button that looks available and does
                // not move.
                {"GetNextCompleatedTutorial", [](lua_State* L) -> int {
            const int from = static_cast<int>(luaL_optnumber(L, 1, 0));
            for (int id : tutorialIds()) {
                if (id > from) { lua_pushnumber(L, id); return 1; }
            }
            return 0;
        }},
                {"GetPrevCompleatedTutorial", [](lua_State* L) -> int {
            const int from = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto ids = tutorialIds();
            for (int id : std::views::reverse(ids)) {
                if (id < from) { lua_pushnumber(L, id); return 1; }
            }
            return 0;
        }},
                {"IsTutorialFlagged", [](lua_State* L) -> int {
            lua_pushboolean(L, tutorialFlagged(
                static_cast<int>(luaL_optnumber(L, 1, 0))) ? 1 : 0);
            return 1;
        }},
                // GetDebugStats() fills the debug stats overlay, which is one
                // SetText of whatever string this hands back. Framerate is the
                // part of it this client actually knows.
                {"GetDebugStats", [](lua_State* L) -> int {
            const double fps = static_cast<double>(ImGui::GetIO().Framerate);
            char line[64];
            std::snprintf(line, sizeof(line), "%.1f fps", fps);
            lua_pushstring(L, line);
            return 1;
        }},
                // The GM survey. Four DBCs describe one, but a survey is
                // something a game master sends and no opcode here delivers,
                // so there is never one in progress: nil questions means the
                // frame counts zero of them and draws empty, which is what an
                // account with no survey pending should see.
                {"GMSurveyQuestion",            lua_ReturnNil},
                {"GMSurveyAnswer",              lua_ReturnNil},
                // Movie recording, which is a Mac client feature this build
                // does not have. Its siblings above already answer; these are
                // the ones MacOptionsFrame reaches as it loads, so each one
                // was a raise on opening the Mac options panel.
                {"MovieRecording_IsSupported",  lua_ReturnFalse},
                {"MovieRecording_IsCursorRecordingSupported", lua_ReturnFalse},
                {"MovieRecording_DataRate",     lua_ReturnZero},
                {"MovieRecording_MaxLength",    lua_ReturnZero},
                {"MovieRecording_GetMovieFullPath", [](lua_State* L) -> int {
            lua_pushstring(L, "");
            return 1;
        }},
                {"MovieRecording_GetViewportWidth", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            auto* win = svc ? svc->window : nullptr;
            lua_pushnumber(L, win ? win->getWidth() : 1920);
            return 1;
        }},
                // How many of a category the compared player has earned - one
                // value, which the summary puts straight into a bar and a
                // "n/total" label beside the player's own. Zero for everyone
                // drew their bar empty however much they had done.
                {"GetComparisonCategoryNumAchievements", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto category = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto* set = gh->getInspectedPlayerAchievements(
                gh->getAchievementComparisonGuid());
            if (!set) { lua_pushnumber(L, 0); return 1; }
            gh->ensureAchievementCategoriesLoaded();
            uint32_t done = 0;
            for (uint32_t id : gh->getCategoryAchievements(category))
                if (set->count(id)) ++done;
            lua_pushnumber(L, done);
            return 1;
        }},
                {"GetMultiCastTotemSpells",  lua_ReturnNil},
                {"GetVoiceStatus",           lua_ReturnFalse},
                {"GetMuteStatus",            lua_ReturnFalse},
                {"GetActiveVoiceChannel",    lua_ReturnNil},
                {"GetVoiceCurrentSessionID", lua_ReturnNil},
                {"GetPartyMember",           lua_ReturnFalse},
                // GetZonePVPInfo() → pvpType, isSubZonePvP, factionName
                //
                // minimap.lua unpacks three and answered nil for all of them,
                // so the zone name on the minimap was always the default
                // colour and its tooltip never said whose territory it was.
                // Middle value stays nil: it is for a sub-zone that differs
                // from its parent, which this reads at zone granularity.
                {"GetZonePVPInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnil(L); return 1; }
            const auto [type, faction] = gh->getZonePvpInfo(gh->getWorldStateZoneId());
            if (type.empty()) { lua_pushnil(L); return 1; }
            lua_pushstring(L, type.c_str());
            lua_pushnil(L);
            if (faction.empty()) lua_pushnil(L); else lua_pushstring(L, faction.c_str());
            return 3;
        }},
                {"GetMouseButtonClicked",    lua_ReturnNil},
                {"GetChatWindowSavedPosition",   lua_GetChatWindowSavedPosition},
                {"GetChatWindowSavedDimensions", lua_GetChatWindowSavedDimensions},
                {"SetChatWindowSavedPosition",   lua_SetChatWindowSavedPosition},
                {"SetChatWindowSavedDimensions", lua_SetChatWindowSavedDimensions},
                {"SetChatWindowSize",            lua_SetChatWindowSize},
                {"SetChatWindowColor",           lua_SetChatWindowColor},
                {"SetChatWindowAlpha",           lua_SetChatWindowAlpha},
                {"SetChatWindowShown",           lua_SetChatWindowShown},
                {"SetChatWindowLocked",          lua_SetChatWindowLocked},
                {"SetChatWindowDocked",          lua_SetChatWindowDocked},
                {"SetChatWindowUninteractable",  lua_SetChatWindowUninteractable},
                {"ResetChatWindows",             lua_ResetChatWindows},
                {"SetChatColorNameByClass",      lua_SetChatColorNameByClass},
                {"GetChatColorNameByClass",      lua_GetChatColorNameByClass},
                {"GetExistingLocales",       lua_ReturnNil},
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
                // The four predicates actionbutton.lua asks about a slot.
                //
                // All answered false, and the pair below the first is why a
                // stack of potions on the bar showed no number: the count is
                // drawn only inside `if ( IsConsumableAction(action) or
                // IsStackableAction(action) )`, and GetActionCount underneath
                // it has been counting across every bag the whole time.
                {"IsAttackAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh || slot < 0) { lua_pushboolean(L, 0); return 1; }
            const auto& bar = gh->getActionBar();
            // 6603 is Auto Attack, the one action that flashes the button red
            // for as long as the swing keeps going.
            constexpr uint32_t kAutoAttack = game::SPELL_ID_ATTACK;
            lua_pushboolean(L, slot < static_cast<int>(bar.size()) &&
                               bar[slot].type == game::ActionBarSlot::SPELL &&
                               bar[slot].id == kAutoAttack);
            return 1;
        }},
                {"IsConsumableAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh || slot < 0) { lua_pushboolean(L, 0); return 1; }
            const auto& bar = gh->getActionBar();
            bool consumable = false;
            if (slot < static_cast<int>(bar.size()) &&
                bar[slot].type == game::ActionBarSlot::ITEM) {
                const auto* info = gh->getItemInfo(bar[slot].id);
                consumable = info && info->valid && info->itemClass == 0;  // Consumable
            }
            lua_pushboolean(L, consumable);
            return 1;
        }},
                {"IsEquippedAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh || slot < 0) { lua_pushboolean(L, 0); return 1; }
            const auto& bar = gh->getActionBar();
            bool worn = false;
            if (slot < static_cast<int>(bar.size()) &&
                bar[slot].type == game::ActionBarSlot::ITEM) {
                const auto& inv = gh->getInventory();
                for (int e = 0; e < game::Inventory::NUM_EQUIP_SLOTS && !worn; ++e) {
                    const auto& s = inv.getEquipSlot(static_cast<game::EquipSlot>(e));
                    worn = !s.empty() && s.item.itemId == bar[slot].id;
                }
            }
            lua_pushboolean(L, worn);
            return 1;
        }},
                {"IsStackableAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh || slot < 0) { lua_pushboolean(L, 0); return 1; }
            const auto& bar = gh->getActionBar();
            bool stackable = false;
            if (slot < static_cast<int>(bar.size()) &&
                bar[slot].type == game::ActionBarSlot::ITEM) {
                const auto* info = gh->getItemInfo(bar[slot].id);
                stackable = info && info->valid && info->maxStack > 1;
            }
            lua_pushboolean(L, stackable);
            return 1;
        }},
                {"IsFlyableArea",            lua_ReturnFalse},
                // The renderer knows whether the camera is inside a WMO, and
                // the macro conditionals [indoors] / [outdoors] have read it
                // all along. These two answered a flat false and a flat true.
                {"IsIndoors", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            lua_pushboolean(L, svc && svc->isPlayerIndoors && svc->isPlayerIndoors());
            return 1;
        }},
                {"IsOutdoors", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            lua_pushboolean(L, !(svc && svc->isPlayerIndoors && svc->isPlayerIndoors()));
            return 1;
        }},
                {"IsHarmfulItem",            lua_ReturnFalse},
                {"IsHelpfulItem",            lua_ReturnFalse},
                {"IsHarmfulSpell",           lua_ReturnFalse},
                {"IsHelpfulSpell",           lua_ReturnFalse},
                // Shown only while there is something to possess *and* a way
                // out of it. The bar's second button is the escape - see
                // GetPossessInfo below - so offering the bar without being
                // able to name the aura it cancels would be a bar that traps
                // rather than releases.
                {"IsPossessBarVisible", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            bool possessing = false;
            if (gh) {
                const uint16_t idx = wowee::game::fieldIndex(
                    wowee::game::UF::UNIT_FIELD_CHARM);
                if (idx != 0xFFFF) {
                    if (auto e = gh->getEntityManager().getEntity(gh->getPlayerGuid())) {
                        const uint64_t lo = e->getField(idx);
                        const uint64_t hi = e->getField(static_cast<uint16_t>(idx + 1));
                        possessing = ((hi << 32) | lo) != 0;
                    }
                }
            }
            lua_pushboolean(L,
                (possessing && possessAuraSpellId(gh) != 0) ? 1 : 0);
            return 1;
        }},
                // GetPossessInfo(slot) → texture, name, enabled
                //
                // Slot two only, which is POSSESS_CANCEL_SLOT.
                // PossessButton_OnClick reads the *name* from here and hands it
                // to CancelUnitBuff("player", name) - that button is how
                // someone gets out of a mind control, so it has to name the
                // possessing aura and nothing else.
                //
                // The other slot answers nil, which hides its button. What
                // belongs there is the possessed unit's own action, and this
                // client has no way to know which of its ten action slots that
                // is - a wrong icon on a bar whose other button works is worse
                // than one button.
                // GetPossessInfo(slot) → texture, name, enabled.
                //
                // Two slots, and they are different things. Slot two is the
                // way out: PossessButton_OnClick reads the name back and
                // cancels that buff on the player, so it has to be the
                // possessing aura's. Slot one is the possessed unit's own
                // action, and its button does nothing when clicked - it is an
                // icon and a tooltip.
                //
                // Which action was recorded as unknown and is not.
                // Player::PossessSpellInitialize sends SMSG_PET_SPELLS built by
                // CharmInfo::BuildActionBar, and InitPossessCreateSpells fills
                // that bar by calling AddSpellToActionBar(spell, ACT_PASSIVE, i)
                // for the creature's spells in order - pinned to index i, since
                // that function skips every slot but the one it was given. So
                // the possessed creature's first spell is bar slot zero, which
                // is the slot this button is for.
                {"GetPossessInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
            constexpr int kCancelSlot = 2;
            if (!gh) return luaReturnNil(L);
            if (slot == 1) {
                const uint32_t packed = gh->getPetActionSlot(0);
                if (packed == 0) return luaReturnNil(L);
                const auto type = game::pet::petActionType(packed);
                // A command or a stance is not something to show here - an
                // empty possess bar carries one of those in slot zero, and
                // drawing its icon would put a Follow arrow where the
                // creature's spell belongs.
                if (type == game::pet::ActionType::Command ||
                    type == game::pet::ActionType::Reaction) {
                    return luaReturnNil(L);
                }
                const uint32_t spellId = game::pet::petActionId(packed);
                if (spellId == 0) return luaReturnNil(L);
                const std::string icon = gh->getSpellIconPath(spellId);
                if (icon.empty()) lua_pushnil(L); else lua_pushstring(L, icon.c_str());
                lua_pushstring(L, gh->getSpellName(spellId).c_str());
                lua_pushboolean(L, 1);
                return 3;
            }
            if (slot != kCancelSlot) return luaReturnNil(L);
            const uint32_t spellId = possessAuraSpellId(gh);
            if (spellId == 0) return luaReturnNil(L);
            const std::string icon = gh->getSpellIconPath(spellId);
            if (icon.empty()) lua_pushnil(L); else lua_pushstring(L, icon.c_str());
            lua_pushstring(L, gh->getSpellName(spellId).c_str());
            lua_pushboolean(L, 1);
            return 3;
        }},
                // IsRaidOfficer() - whether this player is an assistant.
                //
                // The same question UnitIsRaidOfficer already answers for
                // anyone else, and off the same bit: MEMBER_FLAG_ASSISTANT is
                // 0x01 in AzerothCore's Group.h, and the party members carry
                // their flags. Only the player's own answer was hardcoded no.
                //
                // It gates the assistant-only entries on the unit menus and,
                // with IsPartyLeader beside it, whether the chat frame offers
                // to send a raid warning at all.
                {"IsRaidOfficer", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const uint64_t self = gh->getPlayerGuid();
            constexpr uint8_t kMemberFlagAssistant = 0x01;
            for (const auto& mem : gh->getPartyData().members) {
                if (mem.guid == self) {
                    lua_pushboolean(L, (mem.flags & kMemberFlagAssistant) ? 1 : 0);
                    return 1;
                }
            }
            lua_pushboolean(L, 0);
            return 1;
        }},
                {"IsReferAFriendLinked",     lua_ReturnFalse},
                {"IsStereoVideoAvailable",   lua_ReturnFalse},
                {"IsVoiceChatEnabled",       lua_ReturnFalse},
                // False always, which is what disabled the zoom-out button at
                // every level of the map. The only way back out of a zone was
                // the right-click that reaches the same handler without
                // asking this first.
                {"IsZoomOutAvailable", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const bool can = svc && svc->canZoomMapOut && svc->canZoomMapOut();
            lua_pushboolean(L, can ? 1 : 0);
            return 1;
        }},
                {"HasDebugZoneMap",          lua_ReturnFalse},
                {"CanQueueForWintergrasp",   lua_ReturnFalse},
                {"CancelSkillUps",           lua_ReturnNothing},
                {"ConvertToRaid",            lua_ReturnNothing},
                {"FillLocalizedClassList",   lua_ReturnNothing},
                // Sends the request rather than doing nothing. The reply is
                // parsed, fires GUILD_EVENT_LOG_UPDATE and friendsframe
                // listens for it, so answering nothing here was the one link
                // missing at this end - and FrameXML's own call to it is
                // commented out, which is the link missing at the other.
                {"QueryGuildEventLog", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestGuildEventLog();
            return 0;
        }},
                {"RegisterForSave",          lua_ReturnNothing},
                {"RegisterStaticConstants",  lua_ReturnNothing},
                {"SetChatWindowName",        lua_SetChatWindowName},
                {"SetupFullscreenScale",     lua_ReturnNothing},
                // DropCursorMoney is real now, and lives with the rest of the
                // money cursor in lua_inventory_api.cpp. Two registrations of
                // one name would be settled by load order.
                // AchievementMicroButton_Update() - called by the achievement
                // addon and defined nowhere. mainmenubarmicrobuttons.lua has
                // AchievementMicroButton_OnEvent but not this, so it is a hole
                // in this FrameXML rather than a binding this client owes. A
                // no-op, because what it would do is show a micro button that
                // is hidden, and leaving it hidden is the honest outcome.
                {"AchievementMicroButton_Update", [](lua_State* L) -> int {
            (void)L; return 0; }},
                {"BNFeaturesEnabled",        lua_ReturnFalse},
                {"BNFeaturesEnabledAndConnected", lua_ReturnFalse},
                {"BNGetMaxPlayersInConversation", lua_ReturnZero},
                {"GetSummonFriendCooldown",  lua_ReturnNoCooldown},
                {"GetScreenResolutions",     lua_GetScreenResolutions},
                {"GetCurrentResolution",     lua_GetCurrentResolution},
                // Counts a loop bounds itself with. FrameXML writes
                // "for i = 0, num-1" straight after asking, so nothing is not
                // an answer - it is arithmetic on nil and the file is lost.
                // The channel list panel walks these two, and both answered
                // "there are none" while the client knew exactly which
                // channels the player had joined - GetChannelList reports them
                // from the same vector. A stub saying empty is how a working
                // panel shows nothing.
                {"GetNumDisplayChannels", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(gh->getJoinedChannels().size()) : 0.0);
            return 1;
        }},
                {"GetNumMapOverlays",        lua_GetNumMapOverlays},
                {"GetNumMapDebugObjects",    lua_ReturnZero},
                // How many team mates the battleground has reported. The loop
                // that draws them does not use this - it walks to
                // MAX_RAID_MEMBERS and hides whatever answers zero - but the
                // count is asked for beside it and a flat zero said the
                // positions were not there while they were.
                {"GetNumBattlefieldPositions", [](lua_State* L) -> int {
            lua_pushnumber(L, bgCount(getGameHandler(L), 0));
            return 1;
        }},
                {"GetBattlefieldPosition",   lua_GetBattlefieldPosition},
                // Both of these want a position normalised to the map frame
                // currently on screen, which only the map that is drawing knows
                // - and this client draws its own, so FrameXML's WorldMapFrame
                // is suppressed and neither is reached.
                //
                // Zero is not a placeholder here, it is the answer: both call
                // sites read `if ( x == 0 and y == 0 )` and hide the marker.
                // The corpse and the spirit healer are drawn by this client's
                // own map, from getCorpseCanonicalPos and
                // getDeathReleaseLocation. Handing the world map over means
                // giving these the projection, not just filling them in.
                {"GetCorpseMapPosition",     lua_GetBattlefieldPosition},
                {"GetDeathReleasePosition",  lua_GetBattlefieldPosition},
                {"GetNumBattlefieldVehicles", lua_ReturnZero},
                {"GetBattlefieldVehicleInfo", lua_ReturnNil},
                {"GetChatWindowInfo",        lua_GetChatWindowInfo},
                // What a chat window listens to, and the whole reason chat
                // shows anything. ChatFrame_OnLoad registers a chat frame for
                // no CHAT_MSG_ event at all - every one of them comes from
                // ChatFrame_RegisterForMessages walking this list on
                // UPDATE_CHAT_WINDOWS. Answering nothing registered nothing,
                // so every message the client parsed, coloured and routed
                // arrived at a frame that had not asked for it: the chat
                // window stayed empty from login to logout.
                {"GetChatWindowMessages", [](lua_State* L) -> int {
            const ChatWindowSettings* w = chatWindow(L, 1);
            if (!w) return 0;
            // An empty General is unset, not configured, and answers the
            // defaults - which is what the real client does and what FrameXML
            // is written against.
            //
            // FCF_ResetChatWindows calls ChatFrame_RemoveAllMessageGroups on
            // ChatFrame1 and never adds anything back; the defaults it expects
            // to reappear come from the client. Ours took the removals
            // literally, wrote "groups=" empty to disk, and answered nothing
            // ever after - so ChatFrame_RegisterForMessages registered for no
            // event at all and every line of chat was filtered out. The window
            // was not blank because nothing arrived: it was blank because
            // nothing was listening.
            //
            // Only window one, and only when it holds nothing. A window the
            // player has emptied one group at a time is a real setting and is
            // left alone; General with none at all is the wipe above.
            if (w->messageGroups.empty() && lua_tonumber(L, 1) == 1) {
                for (const char* g : kDefaultChatGroups) lua_pushstring(L, g);
                return static_cast<int>(std::size(kDefaultChatGroups));
            }
            for (const auto& g : w->messageGroups) lua_pushstring(L, g.c_str());
            return static_cast<int>(w->messageGroups.size());
        }},
                // Name and zone-channel id in pairs, which is the shape
                // ChatFrame_RegisterForChannels reads: it steps two at a time
                // and fills channelList and zoneChannelList together.
                {"GetChatWindowChannels", [](lua_State* L) -> int {
            const ChatWindowSettings* w = chatWindow(L, 1);
            if (!w) return 0;
            // Same rule, same reason: ChatFrame_RemoveAllChannels runs beside
            // the group wipe in FCF_ResetChatWindows, and a General with no
            // channels matched every channel line against nothing.
            if (w->channels.empty() && lua_tonumber(L, 1) == 1) {
                for (const char* c : {"General", "Trade", "LocalDefense",
                                      "LookingForGroup", "GuildRecruitment"}) {
                    lua_pushstring(L, c);
                    lua_pushnumber(L, 0);
                }
                return 10;
            }
            for (const auto& c : w->channels) {
                lua_pushstring(L, c.first.c_str());
                lua_pushnumber(L, c.second);
            }
            return static_cast<int>(w->channels.size()) * 2;
        }},
                // The four the chat settings panel edits. Ticking a message
                // group or joining a channel goes through these, and they did
                // nothing - so a change made in the panel lasted until the
                // frame was next asked what it listened to, which is the next
                // login, and then went back.
                {"AddChatWindowMessages", [](lua_State* L) -> int {
            ChatWindowSettings* w = chatWindow(L, 1);
            const char* group = lua_isstring(L, 2) ? lua_tostring(L, 2) : nullptr;
            if (!w || !group) return 0;
            for (const auto& g : w->messageGroups) if (g == group) return 0;
            w->messageGroups.emplace_back(group);
            saveInterfaceState();
            return 0;
        }},
                {"RemoveChatWindowMessages", [](lua_State* L) -> int {
            ChatWindowSettings* w = chatWindow(L, 1);
            const char* group = lua_isstring(L, 2) ? lua_tostring(L, 2) : nullptr;
            if (!w || !group) return 0;
            auto& v = w->messageGroups;
            v.erase(std::remove(v.begin(), v.end(), std::string(group)), v.end());
            saveInterfaceState();
            return 0;
        }},
                // Answers the zone channel id, and has to: ChatFrame_AddChannel
                // puts the channel into the frame's own list only `if
                // ( zoneChannel )`, so returning nothing meant the channel was
                // recorded here and never reached the frame that shows it.
                // Zero is a real answer for a channel that is not zone based,
                // and passes that test - zero being true in Lua.
                {"AddChatWindowChannel", [](lua_State* L) -> int {
            ChatWindowSettings* w = chatWindow(L, 1);
            const char* name = lua_isstring(L, 2) ? lua_tostring(L, 2) : nullptr;
            if (!w || !name) return 0;
            const int zone = static_cast<int>(luaL_optnumber(L, 3, 0));
            for (auto& c : w->channels) {
                if (c.first != name) continue;
                if (zone != 0) c.second = zone;
                lua_pushnumber(L, c.second);
                return 1;
            }
            w->channels.emplace_back(name, zone);
            saveInterfaceState();
            lua_pushnumber(L, zone);
            return 1;
        }},
                {"RemoveChatWindowChannel", [](lua_State* L) -> int {
            ChatWindowSettings* w = chatWindow(L, 1);
            const char* name = lua_isstring(L, 2) ? lua_tostring(L, 2) : nullptr;
            if (!w || !name) return 0;
            auto& v = w->channels;
            v.erase(std::remove_if(v.begin(), v.end(),
                        [&](const std::pair<std::string, int>& c) { return c.first == name; }),
                    v.end());
            saveInterfaceState();
            return 0;
        }},
                {"GetNumBattlefieldFlagPositions", [](lua_State* L) -> int {
            // Only when both are carried, for the reason the accessor gives:
            // one carrier cannot be told from the other, and the loop this
            // bounds draws a named flag texture per entry.
            auto* gh = getGameHandler(L);
            const int n = bgCount(gh, 1);
            lua_pushnumber(L, n == 2 ? 2 : 0);
            return 1;
        }},
                {"GetBattlefieldFlagPosition",     lua_GetBattlefieldFlagPosition},
                // Time left on a loot roll that is not running, which
                // GroupLootFrame compares against a bar range at once.
                {"GetNumDungeonMapLevels",   lua_ReturnZero},
                // Bar offsets, added to a page number the line they
                // are read on. No bonus or multi-cast bar is showing,
                // and that is zero rather than nothing.
                {"GetMultiCastBarOffset",    lua_ReturnZero},
                // Which extra action bar the current form or stance uses.
                // actionbutton.lua adds it to NUM_ACTIONBAR_PAGES to pick the
                // page a bonus button reads from, so a flat zero left a druid
                // or a warrior looking at the wrong page in every form.
                //
                // The mapping is in SpellShapeshiftForm.dbc rather than written
                // out here - a table of class-and-form guesses is not
                // checkable, and this one is: cat 1, bear 3, moonkin 4, the
                // three warrior stances 1 to 3, 0 for the travel forms.
                {"GetBonusBarOffset", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(gh->getBonusActionBarOffset()) : 0.0);
            return 1;
        }},
                {"GetNumBattlegroundTypes",  lua_GetNumBattlegroundTypes},
                {"GetBattlegroundInfo",      lua_GetBattlegroundInfo},
                {"RequestBattlegroundInstanceInfo", lua_RequestBattlegroundInstanceInfo},
                {"GetCurrentMapDungeonLevel", lua_ReturnZero},
                {"Sound_GameSystem_GetNumOutputDrivers", lua_Sound_GetNumOutputDrivers},
                {"Sound_ChatSystem_GetNumInputDrivers",  lua_ReturnZero},
                {"Sound_ChatSystem_GetNumOutputDrivers", lua_ReturnZero},
                {"Sound_ChatSystem_GetInputDriverNameByIndex",  lua_ReturnNil},
                {"Sound_ChatSystem_GetOutputDriverNameByIndex", lua_ReturnNil},
                // Nothing selected, which is a number rather than
                // nothing: SkillFrame passes the result straight to
                // GetSkillLineInfo as an index.
                // Which row the skill list has selected. The client has no
                // opinion about it - it is what the player last clicked - so
                // it is held here, the way the friends and ignore lists are.
                //
                // Answering a constant zero meant no row ever matched, because
                // the list is one-based: SkillFrame_SetStatusBar compares each
                // row against this to decide which border to light, so nothing
                // ever looked selected however many times it was clicked.
                {"GetSelectedSkill", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedSkill()); return 1; }},
                {"DungeonUsesTerrainMap",    lua_ReturnFalse},
                // GetChannelDisplayInfo(i) → name, header, collapsed,
                //   channelNumber, count, active, category, voiceEnabled,
                //   voiceActive
                //
                // header is FALSE, not "". An empty string is true in Lua, so
                // returning one marks every channel as a category header:
                // channelframe.lua does `if ( self.header )` and would draw
                // each row as a heading, then call ExpandChannelHeader on it.
                {"GetChannelDisplayInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) { lua_pushnil(L); return 1; }
            const auto& joined = gh->getJoinedChannels();
            if (index > static_cast<int>(joined.size())) { lua_pushnil(L); return 1; }
            lua_pushstring(L, joined[static_cast<size_t>(index) - 1].c_str());
            lua_pushboolean(L, 0);      // header - flat list, no categories
            lua_pushboolean(L, 0);      // collapsed
            lua_pushnumber(L, index);   // channelNumber
            // The roster arrives with SMSG_CHANNEL_LIST, which the panel asks
            // for by calling GetNumChannelMembers on the row it is drawing.
            lua_pushnumber(L, static_cast<lua_Number>(
                gh->getChannelRoster(joined[static_cast<size_t>(index) - 1]).size()));
            lua_pushboolean(L, 1);      // active
            lua_pushstring(L, "CHANNEL_CATEGORY_CUSTOM");
            lua_pushboolean(L, 0);      // voiceEnabled
            lua_pushboolean(L, 0);      // voiceActive
            return 9;
        }},
                // Whether the unit frames should draw their threat
                // indicator, which is the threatWarning CVar read against
                // where the player is. A flat false meant the Display panel's
                // aggro-warning dropdown set a value nothing ever looked at.
                {"IsThreatWarningEnabled", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int setting = 3;
            if (auto it = cvarStore().find("threatwarning"); it != cvarStore().end()) {
                try { setting = std::stoi(it->second); } catch (const std::exception&) {}
            }
            bool on = false;
            switch (setting) {
                case 1: on = gh && gh->isInInstance(); break;
                case 2: on = gh && !gh->getPartyData().members.empty(); break;
                case 3: on = true; break;
                default: on = false; break;   // 0, and anything unrecognised
            }
            lua_pushboolean(L, on ? 1 : 0);
            return 1;
        }},
                // IsAutoRepeatAction(slot) - the button flashes for as long as
                // an auto-repeat is running. There are exactly two in 3.3.5,
                // Auto Shot and the wand's Shoot, which is how IsAttackAction
                // beside it identifies auto-attack: by id rather than by an
                // attribute word this client does not cache.
                {"IsAutoRepeatAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            bool repeating = false;
            if (gh && slot >= 0) {
                const auto& bar = gh->getActionBar();
                if (slot < static_cast<int>(bar.size()) &&
                    bar[slot].type == game::ActionBarSlot::SPELL) {
                    constexpr uint32_t kAutoShot = 75, kShoot = 5019;
                    repeating = bar[slot].id == kAutoShot || bar[slot].id == kShoot;
                }
            }
            lua_pushboolean(L, repeating);
            return 1;
        }},
                // No, and deliberately no even on macOS, which this client does
                // run on. It gates two things. The first is the game menu's Mac
                // Options button, and the panel behind it is Blizzard's movie
                // recorder and compression settings - every one of which this
                // client has nothing behind, so answering yes would add a
                // button that opens a window of dead controls. The second is
                // the Mac spelling of key names in binding text, which is
                // cosmetic and wrong in the smaller direction.
                //
                // Turn this on with the recorder, not before it.
                {"IsMacClient",              lua_ReturnFalse},
                // IsPartyLeader() - whether *this* player leads the group.
                //
                // The client has known this all along: the party data carries a
                // leader guid and PARTY_LEADER_CHANGED is fired when it moves.
                // Answering a flat false told FrameXML the player never leads,
                // which is what gates the leader-only entries on the unit
                // right-click menus and the loot-method controls.
                {"IsPartyLeader", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const uint64_t leader = gh->getPartyData().leaderGuid;
            lua_pushboolean(L, leader != 0 && leader == gh->getPlayerGuid());
            return 1;
        }},
                {"UnitFactionGroup",         lua_UnitFactionGroup},
                // HasPetSpells() → numSpells, petToken
                //
                // Answering nil meant the pet tab was never set up, so a hunter
                // or a warlock with a pet out had no pet spell book at all -
                // SpellBookFrame_Update only calls SpellBookFrame_SetTabType
                // for it when this says there are spells.
                //
                // Both values or neither: the tab's label is built as
                // _G["PET_TYPE_"..token], which raises on a nil token. Only two
                // of those globals exist, DEMON and PET, and WoW picks the
                // first for warlocks and the second for everyone else.
                {"HasPetSpells", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnil(L); return 1; }
            const auto& spells = gh->getPetSpells();
            if (spells.empty()) { lua_pushnil(L); return 1; }
            lua_pushnumber(L, static_cast<lua_Number>(spells.size()));
            constexpr uint8_t kWarlock = 9;
            lua_pushstring(L, gh->getPlayerClass() == kWarlock ? "DEMON" : "PET");
            return 2;
        }},
                // The death knight rune bar, which this client has tracked
                // since it started parsing rune state and never answered for.
                // runeframe.lua is reached through the player frame, handed
                // over by default, so a death knight has been looking at six
                // runes drawn from a nil type and no cooldown at all.
                //
                // RuneType here is Blood, Unholy, Frost, Death from zero;
                // runeframe.lua numbers the same four from one.
                {"GetRuneType", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int id = static_cast<int>(luaL_optinteger(L, 1, 0));
            if (!gh || id < 1 || id > 6) { lua_pushnil(L); return 1; }
            const auto& runes = gh->getPlayerRunes();
            lua_pushinteger(L,
                static_cast<lua_Integer>(runes[static_cast<size_t>(id) - 1].type) + 1);
            return 1;
        }},
                // GetRuneCooldown(id) → start, duration, runeReady
                //
                // The server sends how far along a rune is rather than when it
                // started, so the start is worked back from the fraction. It
                // has to come off the same clock GetTime answers with, or the
                // sweep is drawn against a different origin than it was
                // measured on - CooldownFrame_SetTimer compares the two.
                {"GetRuneCooldown", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int id = static_cast<int>(luaL_optinteger(L, 1, 0));
            if (!gh || id < 1 || id > 6) return 0;
            const auto& rune = gh->getPlayerRunes()[static_cast<size_t>(id) - 1];
            if (rune.ready) {
                lua_pushnumber(L, 0.0);
                lua_pushnumber(L, 0.0);
                lua_pushboolean(L, 1);
                return 3;
            }
            constexpr double kRuneCooldownSec = 10.0;  // fixed in WotLK
            const double elapsed = kRuneCooldownSec *
                static_cast<double>(rune.readyFraction);
            lua_pushnumber(L, luaGetTimeNow() - elapsed);
            lua_pushnumber(L, kRuneCooldownSec);
            lua_pushboolean(L, 0);
            return 3;
        }},
                // The selection is the panel's own state and nothing else
                // reads it, so it lives here and round-trips. Answering nil
                // for the getter meant the highlight never moved.
                {"GetSelectedDisplayChannel", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedDisplayChannel());
            return 1;
        }},
                // Whether the player owns the channel that panel has selected.
                // It answered a flat false, and that is the whole of whether
                // Make Moderator and Remove Moderator appear on the unit menu -
                // so ChannelModerator and ChannelUnmoderator were built and
                // could not be reached from the place that names them.
                //
                // Ownership comes from SMSG_CHANNEL_NOTIFY's OWNER_CHANGED,
                // whose guid is the new owner's: Channel::MakeOwnerChanged
                // writes _ownerGUID into it. The index is the panel's own
                // one-based one, the same GetChannelDisplayInfo reads.
                {"IsDisplayChannelOwner", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int sel = selectedDisplayChannel();
            if (!gh || sel < 1) { lua_pushboolean(L, 0); return 1; }
            const auto& joined = gh->getJoinedChannels();
            if (sel > static_cast<int>(joined.size())) { lua_pushboolean(L, 0); return 1; }
            lua_pushboolean(L, gh->ownsChatChannel(joined[static_cast<size_t>(sel) - 1]) ? 1 : 0);
            return 1;
        }},
                {"SetSelectedDisplayChannel", [](lua_State* L) -> int {
            selectedDisplayChannel() = static_cast<int>(luaL_optnumber(L, 1, 0));
            return 0;
        }},
                // No categories exist to open or close - see the header note
                // on GetChannelDisplayInfo - but the row click handler calls
                // one of these on whatever it was given.
                {"ExpandChannelHeader",      lua_ReturnNothing},
                {"CollapseChannelHeader",    lua_ReturnNothing},
                // Who is in a channel. The server sends a roster only on
                // request and this client never asks, so there is nobody to
                // report; the count above is zero for the same reason.
                // GetNumChannelMembers(channelIndex) - how many are in it, and
                // the request that makes that true. The panel calls this in
                // statement position when a channel row is clicked, which is
                // the only thing that asks the server for a roster at all.
                {"GetNumChannelMembers", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int channel = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || channel < 1) { lua_pushnumber(L, 0); return 1; }
            const auto& joined = gh->getJoinedChannels();
            if (channel > static_cast<int>(joined.size())) { lua_pushnumber(L, 0); return 1; }
            const auto& name = joined[static_cast<size_t>(channel) - 1];
            gh->requestChannelList(name);
            lua_pushnumber(L, static_cast<lua_Number>(gh->getChannelRoster(name).size()));
            return 1;
        }},
                // GetChannelRosterInfo(channelIndex, rosterIndex) →
                //   name, owner, moderator, muted, active, enabled
                //
                // The list was printed to chat and dropped, so the channel
                // panel had nothing to draw and reported every channel as
                // empty. The last two are about voice chat, which this client
                // has none of.
                {"GetChannelRosterInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int channel = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int member = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || channel < 1 || member < 1) { return luaReturnNil(L); }
            const auto& joined = gh->getJoinedChannels();
            if (channel > static_cast<int>(joined.size())) { return luaReturnNil(L); }
            const auto& roster = gh->getChannelRoster(joined[static_cast<size_t>(channel) - 1]);
            if (member > static_cast<int>(roster.size())) { return luaReturnNil(L); }
            const auto& m = roster[static_cast<size_t>(member) - 1];
            lua_pushstring(L, m.name.c_str());          // 1: name
            lua_pushboolean(L, m.owner ? 1 : 0);        // 2: owner
            lua_pushboolean(L, m.moderator ? 1 : 0);    // 3: moderator
            lua_pushboolean(L, m.muted ? 1 : 0);        // 4: muted
            lua_pushboolean(L, 0);                      // 5: active (voice)
            lua_pushboolean(L, 0);                      // 6: enabled (voice)
            return 6;
        }},
                {"GetWintergraspWaitTime",   lua_ReturnNil},
                {"GetNumWorldStateUI",       lua_GetNumWorldStateUI},
                {"GetWorldStateUIInfo",      lua_GetWorldStateUIInfo},
                // Whether the player is standing on a world PvP objective.
                //
                // This gates the whole of uiType 1 in worldstateframe, and the
                // CVar it is gated behind defaults to "2" - show these only in
                // a PvP area - so a flat false hid every world PvP objective
                // display anywhere but inside a battleground, where the
                // instanceType check lets them through instead. The note that
                // stood here said the branch is unreachable; the branch is
                // reachable and this was what closed it.
                //
                // Answered from AreaTable's Flags against the *area* under the
                // player rather than the zone: the bit sits on the subzone -
                // Halaa, The Overlook, the Plaguelands towers - and resolving
                // to the zone loses it. Wintergrasp is marked with a flag of
                // its own and is the one that would have survived either way.
                {"IsSubZonePVPPOI", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const bool on = svc && svc->isOnOutdoorPvpObjective &&
                            svc->isOnOutdoorPvpObjective();
            lua_pushboolean(L, on ? 1 : 0);
            return 1;
        }},
                {"GetNumVoiceSessions",      lua_ReturnZero},
                // Asked from WorldMapFrame_OnUpdate, so every frame the map is
                // open. The handler throttles and refuses outside a
                // battleground; before this nothing ever asked, and the reply
                // that fills the position list is only ever sent on request -
                // so the list was empty for FrameXML's map and for this
                // client's own minimap alike.
                {"RequestBattlefieldPositions", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestBattlefieldPositions();
            return 0;
        }},
                {"UpdateWorldMapArrowFrames",   lua_ReturnNothing},
                {"SetSelectedSkill", [](lua_State* L) -> int {
            selectedSkill() = static_cast<int>(luaL_optnumber(L, 1, 0)); return 0; }},
                {"Sound_GameSystem_GetOutputDriverNameByIndex", lua_Sound_GetOutputDriverNameByIndex},
                {"PlaySound",           lua_PlaySound},
                {"PlaySoundFile",       lua_PlaySoundFile},
                {"GetPlayerMapPosition", lua_GetPlayerMapPosition},
                {"GetPlayerFacing",     lua_GetPlayerFacing},
                {"GetCVar",             lua_GetCVar},
                {"GetCVarBool",         lua_GetCVarBool},
                {"SetCVar",             lua_SetCVar},
                {"GetLocale",         lua_GetLocale},
                // Gender-aware lookup of a global string by token name.
                //
                // The reputation list is the caller that matters:
                // ReputationFrame_Update builds each row's standing label as
                // GetText("FACTION_STANDING_LABEL"..standingID, gender), and an
                // unbound global there is not a blank label but an error that
                // takes the whole list down with it.
                {"GetText", [](lua_State* L) -> int {
            const char* token = luaL_optstring(L, 1, nullptr);
            if (!token) { lua_pushstring(L, ""); return 1; }
            // globalstrings.lua carries FACTION_STANDING_LABEL3_FEMALE beside
            // FACTION_STANDING_LABEL3, so a gendered token wins when it exists.
            // 2 = male, 3 = female; 1 and absent mean neuter/unknown.
            const int gender = static_cast<int>(luaL_optnumber(L, 2, 0));
            const char* suffix = gender == 3 ? "_FEMALE" : (gender == 2 ? "_MALE" : nullptr);
            if (suffix) {
                lua_getglobal(L, (std::string(token) + suffix).c_str());
                if (lua_isstring(L, -1)) return 1;
                lua_pop(L, 1);
            }
            lua_getglobal(L, token);
            if (lua_isstring(L, -1)) return 1;
            lua_pop(L, 1);
            // A token with no string behind it shows as itself rather than as
            // nil, which would clear whatever label asked for it.
            lua_pushstring(L, token);
            return 1; }},
                {"GetBuildInfo",      lua_GetBuildInfo},
                {"GetCurrentMapAreaID", lua_GetCurrentMapAreaID},
                {"SetMapToCurrentZone", lua_SetMapToCurrentZone},
                {"GetCurrentMapContinent", lua_GetCurrentMapContinent},
                {"GetCurrentMapZone",   lua_GetCurrentMapZone},
                {"SetMapZoom",          lua_SetMapZoom},
                {"GetMapContinents",    lua_GetMapContinents},
                {"GetMapZones",         lua_GetMapZones},
                {"GetNumMapLandmarks",  lua_GetNumMapLandmarks},
                // The rest of the world map's own calls. None of them was
                // bound, and two run on every open: WorldMapFrame_OnUpdate
                // asks UpdateMapHighlight for whatever is under the cursor on
                // every frame the mouse is over the map, and the zoom-out
                // button reaches ZoomOut whenever the map is not a continent.
                //
                // Answered rather than filled in. This client draws its own
                // world map and keeps its zones, overlays and area POIs in
                // rendering/world_map, which no callback reaches from here yet
                // - so these say "nothing there" honestly instead of raising,
                // and the map opens empty rather than not at all.
                //
                // Nothing under the cursor: eight values, and the first is the
                // area name the label is set from. Nil leaves it blank, which
                // is what a cursor over no zone should do.
                // UpdateMapHighlight(x, y) →
                //   name, fileName, texPercentageX, texPercentageY,
                //   textureX, textureY, scrollChildX, scrollChildY
                //
                // Asked on every frame the cursor is over the map, and only
                // the first value is read for anything this client can answer:
                // WorldMapFrame_OnUpdate sets the area label from it. The
                // other seven place a highlight *texture* over the zone, which
                // needs the per-zone highlight art the client's own map layer
                // loads on demand - nil leaves the label named and the glow
                // off, which is the honest half.
                {"UpdateMapHighlight", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const float u = static_cast<float>(luaL_optnumber(L, 1, -1.0));
            const float v = static_cast<float>(luaL_optnumber(L, 2, -1.0));
            std::string name;
            if (svc && svc->getMapZoneNameAt) name = svc->getMapZoneNameAt(u, v);
            if (name.empty()) lua_pushnil(L);
            else              lua_pushstring(L, name.c_str());
            for (int i = 0; i < 7; ++i) lua_pushnil(L);
            return 8;
        }},
                // Clicking a continent drills into the zone under the point.
                // Doing nothing leaves the map where it was, which is a map
                // that does not zoom rather than one that zooms wrongly.
                // Clicking a continent drills into the zone under the point,
                // which is the same ZMP lookup the hover uses.
                {"ProcessMapClick", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            if (svc && svc->clickMapPoint) {
                svc->clickMapPoint(static_cast<float>(luaL_optnumber(L, 1, -1.0)),
                                   static_cast<float>(luaL_optnumber(L, 2, -1.0)));
            }
            fireWorldMapUpdate(L);
            return 0;
        }},
                // Zoom out one step. SetMapZoom answers the continent and
                // cosmic cases beside it; this is the dungeon-floor and cosmic
                // step, and neither is reachable while GetNumDungeonMapLevels
                // answers none.
                {"ZoomOut", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            if (svc && svc->zoomMapOut) svc->zoomMapOut();
            fireWorldMapUpdate(L);
            return 0;
        }},
                // Show a zone by its WorldMapArea id. Nothing at all before,
                // which is how the quest log's "show on map" and the map's own
                // windowed-size toggle both landed wherever the map already
                // was.
                {"SetMapByID", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (svc && svc->setMapWorldAreaId && id != 0) svc->setMapWorldAreaId(id);
            fireWorldMapUpdate(L);
            return 0;
        }},
                {"SetDungeonMapLevel",  lua_ReturnNothing},
                {"ClickLandmark",       lua_ReturnNothing},
                // Landmark rows, which GetNumMapLandmarks already reports none
                // of. Bound so that stays true if it ever reports some.
                {"GetMapLandmarkInfo",  lua_GetMapLandmarkInfo},
                {"GetMapOverlayInfo",   lua_GetMapOverlayInfo},
                // Which map a quest's objectives are on, and what a quest item
                // drops from. Both are read while building the map's quest
                // list; nil is "not on this map", which is what the list does
                // with a quest it cannot place.
                // Which map a quest wants opened, and on which dungeon floor.
                // Nothing here tracks either, and the answer for "none" is a
                // pair of zeroes rather than nil because that is what the
                // caller tests: WorldMap_OpenToQuest asks `if ( mapID ~= 0 )`
                // and then `if ( floorNumber ~= 0 )`, and nil ~= 0 is true in
                // Lua - so answering nil ran both branches, on the path the
                // quest tracker takes every time a tracked quest is clicked.
                // SetMapByID drops a zero id and SetDungeonMapLevel does
                // nothing at all, so it was inert; it was inert by luck.
                {"GetQuestWorldMapAreaID", [](lua_State* L) -> int {
            lua_pushnumber(L, 0);   // mapID
            lua_pushnumber(L, 0);   // floorNumber
            return 2; }},
                {"GetQuestLogItemDrop",    lua_ReturnNil},
                // How many of those there are. Unbound, so the world map's
                // quest tooltip called a nil global and raised - the guard
                // beneath it reads the count and would have taken the right
                // branch, but the call never got that far. Zero, matching the
                // reader above, which has no item drops to describe.
                {"GetNumQuestItemDrops",   lua_ReturnZero},
                // Two debug readers, for the zone-map overlay Blizzard ships
                // switched off. Nothing here has a debug zone map at all.
                {"GetDebugZoneMap",        lua_ReturnNil},
                {"GetMapDebugObjectInfo",  lua_ReturnNil},
                {"GetTrackingTexture",  lua_GetTrackingTexture},
                {"GetNumTrackingTypes", lua_GetNumTrackingTypes},
                {"GetTrackingInfo",     lua_GetTrackingInfo},
                {"SetTracking",         lua_SetTracking},
                {"GetZoneText",          lua_GetZoneText},
                {"GetRealZoneText",      lua_GetZoneText},
                {"GetSubZoneText",       lua_GetSubZoneText},
                {"GetMinimapZoneText",   lua_GetMinimapZoneText},
                {"GetGameTime",             lua_GetGameTime},
                {"GetServerTime",           lua_GetServerTime},
                {"CombatLog_Object_IsA", lua_CombatLog_Object_IsA},
                {"GetNumAddOns",      lua_GetNumAddOns},
                {"GetAddOnInfo",      lua_GetAddOnInfo},
                // Turning an addon off, which the "an addon did something it
                // is not allowed to" popup offers as its first button before
                // reloading. Answered with nothing, that button reloaded and
                // brought the same addon back.
                //
                // The change lands on the next run rather than at once: an
                // addon already loaded has its functions in the state and its
                // frames on screen, and neither can be taken back out.
                {"DisableAddOn", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const char* name = luaL_optstring(L, 1, nullptr);
            if (svc && svc->setAddOnEnabled && name && *name)
                svc->setAddOnEnabled(name, false);
            return 0;
        }},
                {"EnableAddOn", [](lua_State* L) -> int {
            auto* svc = getLuaServices(L);
            const char* name = luaL_optstring(L, 1, nullptr);
            if (svc && svc->setAddOnEnabled && name && *name)
                svc->setAddOnEnabled(name, true);
            return 0;
        }},
                {"GetAddOnMetadata",  lua_GetAddOnMetadata},
                {"IsInInstance",         lua_IsInInstance},
                {"GetInstanceInfo",      lua_GetInstanceInfo},
                {"GetInstanceDifficulty", lua_GetInstanceDifficulty},
                {"strsplit",          lua_strsplit},
                {"strtrim",           lua_strtrim},
                {"strlenutf8",        lua_strlenutf8},
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
                // ShowHelm(show) and ShowCloak(show) are setters, and these
                // toggled regardless of what they were passed.
                //
                // interfaceoptionspanels.xml drives both from a checkbox:
                // `self:SetChecked(value); ShowHelm(value)`. Toggling on a set
                // inverts the answer whenever the state already matched - the
                // box would tick and the helm would go away - and the panel
                // re-applies its value on every open, so it flipped again each
                // time the options were shown.
                //
                // The client only has a toggle, so this toggles only when that
                // lands on what was asked for. Same shape as SetPVP.
                // The two getters beside ShowHelm and ShowCloak below, which
                // have worked all along - so the checkbox wrote the setting
                // correctly and could not show it, and reading it raised as
                // the Display panel was built.
                {"ShowingHelm", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && gh->isHelmVisible()) ? 1 : 0);
            return 1;
        }},
                {"ShowingCloak", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && gh->isCloakVisible()) ? 1 : 0);
            return 1;
        }},
                // The equipment manager exists here - GetNumEquipmentSets and
                // the rest of its API are bound - so the panel's checkbox is
                // offering something real.
                {"CanUseEquipmentSets",         lua_ReturnTrue},
                // Tutorials can be reset because they are kept: the flags live
                // in the CVar file. Answering false would grey out a button
                // that would have worked.
                {"CanResetTutorials",           lua_ReturnTrue},
                {"ResetTutorials", [](lua_State* L) -> int {
            (void)L;
            cvarStore()[kTutorialCVar].clear();
            saveStoredCVars();
            return 0;
        }},
                // A Mac-only mouse this build does not look for.
                {"DetectWowMouse",              lua_ReturnFalse},
                // No Battle.net, so no chat to filter.
                {"BNSetMatureLanguageFilter",   lua_ReturnNothing},
                {"ShowHelm", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const bool want = lua_isnumber(L, 1) ? (lua_tonumber(L, 1) != 0)
                                                 : (lua_toboolean(L, 1) != 0);
            if (want != gh->isHelmVisible()) gh->toggleHelm();
            return 0;
        }},
                {"ShowCloak", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const bool want = lua_isnumber(L, 1) ? (lua_tonumber(L, 1) != 0)
                                                 : (lua_toboolean(L, 1) != 0);
            if (want != gh->isCloakVisible()) gh->toggleCloak();
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
            lua_pushnumber(L, static_cast<double>(taxiNodeOrder(getGameHandler(L)).size()));
            return 1;
        }},
                {"TaxiNodeName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (id == 0) { lua_pushstring(L, ""); return 1; }
            const auto& nodes = gh->getTaxiNodes();
            const auto it = nodes.find(id);
            lua_pushstring(L, it != nodes.end() ? it->second.name.c_str() : "");
            return 1;
        }},
                // TaxiNodeGetType(index) → "CURRENT" | "REACHABLE" | "DISTANT" | "NONE"
                //
                // A name, not a number. The flight map compares this against
                // those four words - to pick the pin's colour, and first of all
                // to decide whether the node is on the map at all. Answering 0
                // or 1 is never equal to any of them, so every node counted as
                // shown, including the ones the player has never been to.
                {"TaxiNodeGetType", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (id == 0) { lua_pushstring(L, "NONE"); return 1; }
            if (id == gh->getTaxiCurrentNode())  { lua_pushstring(L, "CURRENT"); return 1; }
            if (!gh->isKnownTaxiNode(id))        { lua_pushstring(L, "NONE"); return 1; }
            // Known, but nothing flies there from where the player is standing:
            // shown in yellow rather than hidden, so it reads as somewhere they
            // have been rather than somewhere that does not exist.
            lua_pushstring(L, gh->hasTaxiRouteTo(id) ? "REACHABLE" : "DISTANT");
            return 1;
        }},
                {"TakeTaxiNode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (id != 0) gh->activateTaxi(id);
            return 0;
        }},
                // TaxiNodePosition(index) → x, y as fractions of the map.
                {"TaxiNodePosition", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            float u = 0, v = 0;
            if (id == 0 || !taxiNodeMapPos(gh, id, u, v)) { return luaReturnNil(L); }
            lua_pushnumber(L, u);
            lua_pushnumber(L, v);
            return 2;
        }},
                // TaxiNodeSetCurrent(index) - the node the map is drawing a
                // route to. Works out the journey once; the frame then asks
                // about each leg of it.
                {"TaxiNodeSetCurrent", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            taxiRouteShown() = (id != 0 && gh) ? gh->getTaxiRouteTo(id)
                                               : std::vector<uint32_t>{};
            return 0;
        }},
                // GetNumRoutes(index) → how many hops the flight takes.
                //
                // The map draws one line per hop, and reads this to decide how
                // many. It also asks about every node in the list to find the
                // ones a single hop away, so this answers for the node asked
                // about rather than for whatever was last set current.
                {"GetNumRoutes", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (id == 0 || !gh) { lua_pushnumber(L, 0); return 1; }
            const auto route = gh->getTaxiRouteTo(id);
            // A route of N nodes is N-1 hops; one of fewer than two is no
            // journey at all.
            lua_pushnumber(L, route.size() >= 2
                                  ? static_cast<double>(route.size() - 1) : 0.0);
            return 1;
        }},
                // TaxiGetSrcX/Y(index, hop) and TaxiGetDestX/Y(index, hop) -
                // the ends of one leg, on the same scale as TaxiNodePosition.
                //
                // The index names the destination and the hop names the leg, so
                // these route afresh rather than trusting whatever was last set
                // current: DrawOneHopLines walks every node without ever
                // calling TaxiNodeSetCurrent.
                {"TaxiGetSrcX",  lua_TaxiLegCoord<0, true>},
                {"TaxiGetSrcY",  lua_TaxiLegCoord<0, false>},
                {"TaxiGetDestX", lua_TaxiLegCoord<1, true>},
                {"TaxiGetDestY", lua_TaxiLegCoord<1, false>},
                // SetTaxiMap(texture) - the flight map's own picture.
                //
                // This client has continent artwork, but not in the shape this
                // call wants: its flight map is a mode of the world map, which
                // draws the continent from tiles and puts taxi nodes over it,
                // where SetTaxiMap hands over one texture to point at a single
                // TAXIMAP image. So the texture keeps whatever its XML set.
                // Named rather than left out so it does not read as a gap that
                // was missed.
                // SetTaxiMap(texture) - the continent behind the flight points.
                //
                // TaxiFrame_OnEvent hands its TaxiMap texture to this and
                // expects the picture of the continent to come back on it; the
                // node buttons are then placed over it by TaxiNodePosition,
                // which answers a fraction of the map rather than a position on
                // screen. A no-op here left those buttons floating on an empty
                // panel, which is not a flight map - it is a set of unlabelled
                // dots with nothing to read them against.
                //
                // The files are keyed by map id, not by name: this install has
                // taximap0, taximap1, taximap530 and taximap571 - Eastern
                // Kingdoms, Kalimdor, Outland and Northrend. So the id is the
                // whole of the lookup and no table of continent names is
                // needed, which is as well, because a name table would be a
                // second place for the same fact to be written down.
                {"SetTaxiMap", [](lua_State* L) -> int {
            auto* tree = getWidgetTree(L);
            if (!tree || !lua_istable(L, 1)) return 0;
            // The widget id the frame table carries, read the same way
            // SetPortraitTexture reads it - widgetIdOf is not visible here.
            lua_getfield(L, 1, "__wid");
            const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
            lua_pop(L, 1);
            if (id == 0) return 0;
            auto* w = tree->get(id);
            if (!w) return 0;
            w->solidColor = false;
            // The handler is not required to reach this: without one the map
            // id is zero, which is a real map rather than a refusal, and a
            // client with nobody logged in has no flight window to draw
            // anyway. Requiring it would only make the wiring untestable
            // outside a session.
            auto* gh = getGameHandler(L);
            w->texturePath = "Interface\\TaxiFrame\\TaxiMap" +
                             std::to_string(gh ? gh->getCurrentMapId() : 0u);
            return 0;
        }},
                {"CloseTaxiMap", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closeTaxi();
            return 0;
        }},
                // TaxiNodeCost(index) → the fare in copper, for the tooltip.
                {"TaxiNodeCost", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = taxiNodeAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            lua_pushnumber(L, id != 0 ? static_cast<double>(gh->getTaxiCostTo(id)) : 0.0);
            return 1;
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
                // SetCurrentTitle(bit) - and the comment that used to sit here
                // saying CMSG_SET_TITLE was not exposed was stale:
                // sendSetTitle builds and sends it, and has for a while.
                //
                // The server validates it. HandleSetTitleOpcode refuses any bit
                // the character does not own and silently sets none, so there
                // is nothing to check here that the server does not check
                // better.
                {"SetCurrentTitle", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->sendSetTitle(static_cast<int32_t>(luaL_optnumber(L, 1, -1)));
            return 0;
        }},
                {"GetInspectSpecialization", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* ir = gh ? gh->getInspectResult() : nullptr;
            lua_pushnumber(L, ir ? ir->activeTalentGroup : 0);
            return 1;
        }},
                // GetInspectArenaTeamData(index) →
                //   name, size, rating, weekPlayed, weekWins, seasonPlayed,
                //   seasonWins, playerRating
                //
                // Real: the inspect reply carries these and this client already
                // parses them into InspectResult.
                {"GetInspectArenaTeamData", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* ir = gh ? gh->getInspectResult() : nullptr;
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!ir || index < 1 ||
                index > static_cast<int>(ir->arenaTeams.size())) {
                return 0;   // nothing, which is how WoW says "no team here"
            }
            const auto& t = ir->arenaTeams[index - 1];
            // The type is the team size: 2, 3 or 5.
            lua_pushstring(L, t.name.c_str());
            lua_pushnumber(L, t.type);
            lua_pushnumber(L, t.personalRating);
            lua_pushnumber(L, t.weekGames);
            lua_pushnumber(L, t.weekWins);
            lua_pushnumber(L, t.seasonGames);
            lua_pushnumber(L, t.seasonWins);
            lua_pushnumber(L, t.personalRating);
            return 8;
        }},
                // CanInspect(unit [, showError]) - a player other than a
                // corpse, which is as much as this client can judge; the server
                // refuses the rest and the reply simply does not arrive.
                {"CanInspect", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "target");
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            std::string uidStr(uid);
            for (char& c : uidStr) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            const uint64_t guid = resolveUnitGuid(gh, uidStr);
            if (guid == 0) { lua_pushboolean(L, 0); return 1; }
            // High word 0x0000 marks a player guid in this range; creatures
            // carry 0xF13/0xF14 and cannot be inspected at all.
            const bool isPlayer = ((guid >> 48) & 0xFFFF) == 0;
            lua_pushboolean(L, isPlayer ? 1 : 0);
            return 1;
        }},
                // Honour data for an inspected player is not in the reply this
                // client parses, so it says so plainly rather than reporting
                // zeros as though they were the answer: HasInspectHonorData is
                // false, and the panel's honour section stays empty instead of
                // claiming the player has never won anything.
                {"HasInspectHonorData", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* r = gh ? gh->getInspectResult() : nullptr;
            lua_pushboolean(L, (r && r->hasHonorData) ? 1 : 0);
            return 1;
        }},
                // The tab asks for these when it opens and redraws on
                // INSPECT_HONOR_UPDATE. It did nothing, so the answer never
                // came and the tab sat empty behind a HasInspectHonorData that
                // was false for good.
                {"RequestInspectHonorData", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* r = gh ? gh->getInspectResult() : nullptr;
            if (gh && r && r->guid != 0) gh->requestInspectHonorData(r->guid);
            return 0;
        }},
                {"GetInspectHonorData", [](lua_State* L) -> int {
            // Six: today's kills and honour, yesterday's, lifetime kills, and
            // the lifetime *rank*, which inspecthonorframe feeds straight into
            // GetPVPRankInfo and shows NONE for when that answers nothing.
            //
            // The rank byte is what the server puts in that slot, and what it
            // puts there is the honour points truncated to eight bits - the
            // rank ladder was retired in this expansion and the field was
            // reused rather than removed. Passed on as sent rather than
            // interpreted here.
            auto* gh = getGameHandler(L);
            const auto* r = gh ? gh->getInspectResult() : nullptr;
            if (!r || !r->hasHonorData) {
                for (int i = 0; i < 6; ++i) lua_pushnumber(L, 0);
                return 6;
            }
            lua_pushnumber(L, r->honorTodayKills);
            lua_pushnumber(L, r->honorTodayContribution);
            lua_pushnumber(L, r->honorYesterdayKills);
            lua_pushnumber(L, r->honorYesterdayContribution);
            lua_pushnumber(L, r->honorLifetimeKills);
            lua_pushnumber(L, r->honorRank);
            return 6;
        }},
                {"GetInspectPVPRankProgress", [](lua_State* L) -> int {
            lua_pushnumber(L, 0);
            return 1;
        }},
                // UnitPVPRank(unit) - the old honour rank, which no WotLK
                // server sends; GetPVPRankInfo is fed from it and handles zero.
                {"UnitPVPRank", [](lua_State* L) -> int {
            (void)L;
            lua_pushnumber(L, 0);
            return 1;
        }},
                // NotifyInspect(unit) - ask the server for this player's gear,
                // talents and achievements. The inspect window calls it as it
                // opens and again whenever it is pointed at someone else, and
                // it is the only request either of those makes.
                //
                // This did nothing, on the reasoning that the client inspects
                // by itself when a player is targeted. It does, but that is a
                // different thing: a background queue that keeps gear visuals
                // right, throttled to one request every two seconds and only
                // for players already spawned nearby. So the window opened on
                // whatever that queue happened to have fetched last - often
                // another player entirely, and on a first inspect nothing.
                //
                // inspectUnit sends the achievements query alongside on Wrath,
                // which is what the comparison tab reads.
                {"NotifyInspect", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const uint64_t guid = resolveUnitGuid(gh, luaL_optstring(L, 1, "target"));
            if (guid) gh->inspectUnit(guid);
            return 0;
        }},
                {"ClearInspectPlayer", [](lua_State* L) -> int {
            (void)L;
            return 0;
        }},
                // Both answer the amount held *and the cap*. staticpopup.lua
                // does `MerchantFrame.honorPoints + currentHonor > maxHonor`
                // when confirming a PvP refund, and comparing a number against
                // a nil raises - so refunding a honour purchase took the
                // confirmation down rather than showing it.
                //
                // The caps are the client's own constants for this expansion,
                // not something the server sends: 75000 honour, 5000 arena
                // points. They are only ever used to warn that a refund would
                // overflow, which is exactly what they are right for.
                {"GetHonorCurrency", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getHonorPoints() : 0);
            lua_pushnumber(L, 75000);
            return 2;
        }},
                {"GetArenaCurrency", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getArenaPoints() : 0);
            lua_pushnumber(L, 5000);
            return 2;
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
                // SetSavedInstanceExtend(index, doExtend) - the Extend button
                // on the raid lockout list, which raidframe.lua enables for any
                // selected row. Everything around it was bound; this one was
                // not, so pressing it raised.
                //
                // The call gives a position in the list and the wire wants the
                // map and difficulty, which is what the lockout at that
                // position holds.
                {"SetSavedInstanceExtend", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) return 0;
            const auto& locks = gh->getInstanceLockouts();
            if (index > static_cast<int>(locks.size())) return 0;
            const auto& l = locks[static_cast<size_t>(index) - 1];
            gh->setSavedInstanceExtend(l.mapId, l.difficulty, lua_toboolean(L, 2) != 0);
            return 0;
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
            // The difficulty in words, which the raid lockout row prints
            // beside the instance name. It was not returned, so that column
            // was blank on every saved instance.
            const char* diffName = game::instanceDifficultyName(l.difficulty);
            lua_pushstring(L, diffName ? diffName : "Normal");
            return 10;
        }},
                {"CalendarGetDate", [](lua_State* L) -> int {
            // CalendarGetDate() → weekday, month, day, year
            time_t now = time(nullptr);
            const std::tm t = core::localTime(now);
            lua_pushnumber(L, t.tm_wday + 1); // weekday (1=Sun)
            lua_pushnumber(L, t.tm_mon + 1);  // month (1-12)
            lua_pushnumber(L, t.tm_mday);     // day
            lua_pushnumber(L, t.tm_year + 1900); // year
            return 4;
        }},
                // How many calendar invites are waiting to be answered. The
                // calendar addon is refused by name, but this is not only its
                // question: gametime.lua asks it for the indicator on the
                // minimap's date button, and that file is core FrameXML. A
                // constant zero left the button dark however many invites had
                // arrived.
                {"CalendarGetNumPendingInvites", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(gh->getCalendarPendingInvites()) : 0);
            return 1;
        }},
                {"CalendarGetNumDayEvents", [](lua_State* L) -> int {
            lua_pushnumber(L, static_cast<double>(calendarDayRows(L).size()));
            return 1;
        }},
                // Fifteen values, in the order the interface unpacks them:
                //
                //   title, hour, minute, calendarType, sequenceType, eventType,
                //   texture, modStatus, inviteStatus, invitedBy, difficulty,
                //   inviteType, sequenceIndex, numSequenceDays, difficultyName
                //
                // A nil title is how the list ends - the interface tests
                // `if ( title and sequenceType ~= "ONGOING" )` - so an index
                // past the day's rows answers nothing at all rather than a row
                // of blanks, which would draw an empty button on every day.
                {"CalendarGetDayEvent", [](lua_State* L) -> int {
            const auto rows = calendarDayRows(L);
            const int index = static_cast<int>(luaL_optnumber(L, 3, 0));
            if (index < 1 || static_cast<size_t>(index) > rows.size()) return 0;
            const auto& row = rows[static_cast<size_t>(index) - 1];
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const wowee::game::CalendarData& cal = gh->getCalendarData();

            if (row.kind == wowee::game::CalendarEntryKind::RaidLockout) {
                const auto& lock = cal.lockouts[row.index];
                lua_pushstring(L, gh->getMapName(lock.mapId).c_str());
                lua_pushnumber(L, row.hour);
                lua_pushnumber(L, row.minute);
                lua_pushstring(L, "RAID_LOCKOUT");
                lua_pushstring(L, "");             // sequenceType
                lua_pushnumber(L, 0);              // eventType
                lua_pushnumber(L, 0);              // texture
                lua_pushstring(L, "");             // modStatus
                lua_pushnumber(L, 1);              // inviteStatus
                lua_pushstring(L, "");             // invitedBy
                lua_pushnumber(L, lock.difficulty);
                lua_pushnumber(L, 1);              // inviteType
                lua_pushnil(L);                    // sequenceIndex
                lua_pushnil(L);                    // numSequenceDays
                lua_pushstring(L, "");             // difficultyName
                return 15;
            }
            if (row.kind == wowee::game::CalendarEntryKind::Holiday) {
                const auto& holiday = cal.holidays[row.index];
                lua_pushstring(L, holiday.textureFilename.c_str());
                lua_pushnumber(L, 0);              // hour
                lua_pushnumber(L, 0);              // minute
                lua_pushstring(L, "HOLIDAY");
                // The one distinction the interface acts on: a holiday's name
                // is drawn on its first day and its later days are skipped.
                lua_pushstring(L, row.ongoing ? "ONGOING" : "START");
                lua_pushnumber(L, 0);              // eventType
                lua_pushnumber(L, static_cast<double>(holiday.id));  // texture
                lua_pushstring(L, "");             // modStatus
                lua_pushnumber(L, 0);              // inviteStatus
                lua_pushstring(L, "");             // invitedBy
                lua_pushnumber(L, 0);              // difficulty
                lua_pushnumber(L, 0);              // inviteType
                lua_pushnil(L);                    // sequenceIndex
                lua_pushnil(L);                    // numSequenceDays
                lua_pushstring(L, "");             // difficultyName
                return 15;
            }

            const auto& ev = cal.events[row.index];
            lua_pushstring(L, ev.title.c_str());
            lua_pushnumber(L, ev.eventTime.hour);
            lua_pushnumber(L, ev.eventTime.minute);
            lua_pushstring(L, calendarTypeName(ev));
            lua_pushstring(L, "");                 // sequenceType: single day
            lua_pushnumber(L, static_cast<double>(ev.type) + 1);
            lua_pushnumber(L, 0);                  // texture
            lua_pushstring(L, calendarModStatus(L, ev));
            lua_pushnumber(L, calendarInviteStatusFor(cal, ev.eventId) + 1);
            lua_pushstring(L, "");                 // invitedBy
            lua_pushnumber(L, 0);                  // difficulty
            lua_pushnumber(L, 1);                  // inviteType: NORMAL
            lua_pushnil(L);                        // sequenceIndex
            lua_pushnil(L);                        // numSequenceDays
            lua_pushstring(L, "");                 // difficultyName
            return 15;
        }},
                // name, description, texture - indexed into the same day list
                // as CalendarGetDayEvent, so a row that is not a holiday
                // answers nothing rather than the wrong holiday.
                {"CalendarGetHolidayInfo", [](lua_State* L) -> int {
            const auto rows = calendarDayRows(L);
            const int index = static_cast<int>(luaL_optnumber(L, 3, 0));
            if (index < 1 || static_cast<size_t>(index) > rows.size()) return 0;
            const auto& row = rows[static_cast<size_t>(index) - 1];
            if (row.kind != wowee::game::CalendarEntryKind::Holiday) return 0;
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& holiday = gh->getCalendarData().holidays[row.index];
            lua_pushstring(L, holiday.textureFilename.c_str());
            lua_pushstring(L, "");
            lua_pushstring(L, holiday.textureFilename.c_str());
            return 3;
        }},
                // name, calendarType, raidID, hour, minute, difficulty,
                // difficultyName - the seven the raid view unpacks. Indexed
                // into the same day list as the other two readers, so a row
                // that is not a lockout answers nothing rather than the wrong
                // raid.
                {"CalendarGetRaidInfo", [](lua_State* L) -> int {
            const auto rows = calendarDayRows(L);
            const int index = static_cast<int>(luaL_optnumber(L, 3, 0));
            if (index < 1 || static_cast<size_t>(index) > rows.size()) return 0;
            const auto& row = rows[static_cast<size_t>(index) - 1];
            if (row.kind != wowee::game::CalendarEntryKind::RaidLockout) return 0;
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& lock = gh->getCalendarData().lockouts[row.index];
            lua_pushstring(L, gh->getMapName(lock.mapId).c_str());
            lua_pushstring(L, "RAID_LOCKOUT");
            lua_pushnumber(L, static_cast<double>(lock.mapId));
            lua_pushnumber(L, row.hour);
            lua_pushnumber(L, row.minute);
            lua_pushnumber(L, lock.difficulty);
            lua_pushstring(L, "");
            return 7;
        }},
                // Ask the server for the calendar. The addon's OnShow calls
                // this, and the answer arrives as CALENDAR_UPDATE_EVENT_LIST.
                {"OpenCalendar", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestCalendar();
            return 0;
        }},
                // Whether a request is in flight. Nothing here queues one, so
                // the answer is no - and false rather than nothing, because
                // the interface disables buttons on it and nil would read the
                // same as false only by accident.
                {"CalendarIsActionPending", [](lua_State* L) -> int {
            lua_pushboolean(L, 0);
            return 1;
        }},
                // minLevel, maxLevel, rank - the default filter a guild event
                // is created with. The whole guild, which is what the server
                // defaults to as well.
                {"CalendarDefaultGuildFilter", [](lua_State* L) -> int {
            lua_pushnumber(L, 1);
            lua_pushnumber(L, 80);
            lua_pushnumber(L, 0);
            return 3;
        }},
                // Whether the player may put an event on the guild's calendar.
                // The server decides for real; this is the guild-membership
                // half of it, which is the half that gates the button.
                {"CanEditGuildEvent", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && !gh->getGuildName().empty()) ? 1 : 0);
            return 1;
        }},
                // Arena team events need a team. No arena team is tracked, so
                // the honest answer is no and the option stays out of the
                // create menu rather than leading to a request that fails.
                {"IsInArenaTeam", [](lua_State* L) -> int {
            lua_pushboolean(L, 0);
            return 1;
        }},
                // ---- Opening one event ----
                //
                // The interface names a row, this turns it into an event id
                // and asks the server; the answer arrives as
                // CALENDAR_UPDATE_EVENT and CalendarGetEventInfo reads it.
                {"CalendarOpenEvent", [](lua_State* L) -> int {
            const auto rows = calendarDayRows(L);
            const int index = static_cast<int>(luaL_optnumber(L, 3, 0));
            auto* gh = getGameHandler(L);
            if (!gh || index < 1 || static_cast<size_t>(index) > rows.size()) return 0;
            const auto& row = rows[static_cast<size_t>(index) - 1];
            if (row.kind != wowee::game::CalendarEntryKind::Event) return 0;
            gh->requestCalendarEvent(gh->getCalendarData().events[row.index].eventId);
            return 0;
        }},
                {"CalendarCloseEvent", [](lua_State* L) -> int {
            (void)L;
            return 0;
        }},
                // Twenty-five values, in the order the view frame unpacks
                // them. A nil title is how it knows there is nothing to show -
                // `if ( not title ) then return end` - so an unopened event
                // answers nothing rather than a row of blanks.
                {"CalendarGetEventInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& ev = gh->getCalendarEventDetail();
            if (ev.eventId == 0) return 0;
            lua_pushstring(L, ev.title.c_str());
            lua_pushstring(L, ev.description.c_str());
            lua_pushstring(L, gh->lookupName(ev.creatorGuid).c_str());
            lua_pushnumber(L, ev.type + 1);          // wire counts from zero
            lua_pushnumber(L, ev.repeatOption + 1);  // and so does this
            lua_pushnumber(L, ev.maxInvites);
            lua_pushnumber(L, ev.dungeonId);        // textureIndex
            lua_pushnumber(L, ev.eventTime.weekday + 1);
            lua_pushnumber(L, ev.eventTime.month);
            lua_pushnumber(L, ev.eventTime.day);
            lua_pushnumber(L, ev.eventTime.fullYear());
            lua_pushnumber(L, ev.eventTime.hour);
            lua_pushnumber(L, ev.eventTime.minute);
            // The lockout date, which only a raid-reset event carries. The
            // same date rather than nothing: the frame formats it without
            // guarding, so nil there raises where a repeated date only reads
            // oddly on an event that has no lockout to show.
            lua_pushnumber(L, ev.eventTime.weekday + 1);
            lua_pushnumber(L, ev.eventTime.month);
            lua_pushnumber(L, ev.eventTime.day);
            lua_pushnumber(L, ev.eventTime.fullYear());
            lua_pushnumber(L, ev.eventTime.hour);
            lua_pushnumber(L, ev.eventTime.minute);
            constexpr uint32_t kFlagInvitesLocked = 0x1000;
            lua_pushboolean(L, (ev.flags & kFlagInvitesLocked) ? 1 : 0);
            lua_pushboolean(L, 0);              // autoApprove
            lua_pushboolean(L, 0);              // pendingInvite
            lua_pushnumber(L, calendarInviteStatusFor(gh->getCalendarData(),
                                                      ev.eventId) + 1);
            // CALENDAR_INVITETYPE_NORMAL, which is 1 - the interface's range
            // starts there, so zero is not "normal" but out of range.
            lua_pushnumber(L, 1);
            lua_pushstring(L, (ev.flags & 0x0400u) ? "GUILD_EVENT" : "PLAYER");
            return 25;
        }},
                // CalendarCanSendInvite() - may this player invite to the event
                // that is open?
                //
                // Five places in the calendar ask it, and it was bound nowhere.
                // With the missing-API stand-in on, an unknown global answers
                // with a callable table and a table is truthy - so every one of
                // those five took the "yes, you may" branch, and the invite
                // controls appeared on events belonging to other people. With
                // the stand-in off it raises outright, which is how it was
                // found: clicking through a panel loads the calendar, and the
                // calendar raised on the way up.
                //
                // The event's creator always may. Anyone else may only if their
                // own row on the invite list carries at least moderator -
                // CALENDAR_RANK_PLAYER 0, MODERATOR 1, OWNER 2, from
                // CalendarMgr.h. The rank is per invitee rather than on the
                // event, so it is this player's row that answers.
                {"CalendarCanSendInvite", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto& ev = gh->getCalendarEventDetail();
            const uint64_t me = gh->getPlayerGuid();
            bool can = (me != 0 && ev.creatorGuid == me);
            if (!can) {
                for (const auto& inv : ev.invitees) {
                    if (inv.guid != me) continue;
                    can = inv.rank >= 1;
                    break;
                }
            }
            lua_pushboolean(L, can ? 1 : 0);
            return 1;
        }},
                {"CalendarEventGetNumInvites", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(
                gh->getCalendarEventDetail().invitees.size()) : 0);
            return 1;
        }},
                // name, level, className, classFilename, inviteStatus - the
                // five the invite list reads.
                {"CalendarEventGetInvite", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return 0;
            const auto& list = gh->getCalendarEventDetail().invitees;
            if (index < 1 || static_cast<size_t>(index) > list.size()) return 0;
            const auto& inv = list[static_cast<size_t>(index) - 1];
            lua_pushstring(L, gh->lookupName(inv.guid).c_str());
            lua_pushnumber(L, inv.level);
            lua_pushstring(L, "");              // className
            lua_pushstring(L, "");              // classFilename
            lua_pushnumber(L, inv.status + 1);   // wire counts from zero
            lua_pushnumber(L, inv.rank);
            lua_pushboolean(L, inv.isGuildMember ? 1 : 0);
            return 7;
        }},
                // ---- Creating an event ----
                //
                // The staging half of the calendar's write side. Each of these
                // writes one field of the draft and CalendarAddEvent sends it;
                // none of them talks to the server on its own, which is what
                // lets the create form be filled in in any order and cancelled
                // without trace.
                {"CalendarNewEvent", [](lua_State* L) -> int {
            (void)L;
            calendarDraft() = wowee::game::CalendarEventDraft{};
            // Today, so an event committed without touching the date lands
            // somewhere real rather than on 1 January 2000.
            const auto viewed = calendarViewedMonth();
            const std::time_t now = std::time(nullptr);
            const std::tm t = core::localTime(now);
            auto& d = calendarDraft();
            d.eventTime.month = viewed.first;
            d.eventTime.yearSince2000 = viewed.second - 2000;
            d.eventTime.day = t.tm_mday;
            d.eventTime.hour = 12;
            return 0;
        }},
                {"CalendarNewGuildEvent", [](lua_State* L) -> int {
            (void)L;
            calendarDraft() = wowee::game::CalendarEventDraft{};
            // CALENDAR_FLAG_GUILD_EVENT, which is what the server reads to
            // decide the event belongs to the guild rather than the player.
            calendarDraft().flags |= 0x0400u;
            return 0;
        }},
                {"CalendarEventSetTitle", [](lua_State* L) -> int {
            // Clipped to what the server accepts. It refuses the whole packet
            // over 31 characters rather than truncating, so a long title would
            // silently create nothing at all.
            std::string title(luaL_optstring(L, 1, ""));
            if (title.size() > 31) title.resize(31);
            calendarDraft().title = std::move(title);
            return 0;
        }},
                {"CalendarEventSetDescription", [](lua_State* L) -> int {
            std::string desc(luaL_optstring(L, 1, ""));
            if (desc.size() > 255) desc.resize(255);
            calendarDraft().description = std::move(desc);
            return 0;
        }},
                // Interface types count from one and the wire counts from
                // zero, in the same order: constants.lua has
                // CALENDAR_EVENTTYPE_RAID = 1 where CalendarMgr.h has
                // CALENDAR_TYPE_RAID = 0. Passed straight through, every event
                // would be created as the next type along - a raid saved as a
                // dungeon, with nothing to see but the wrong icon.
                {"CalendarEventSetType", [](lua_State* L) -> int {
            const int uiType = static_cast<int>(luaL_optnumber(L, 1, 1));
            calendarDraft().type =
                static_cast<uint8_t>(uiType > 0 ? uiType - 1 : 0);
            return 0;
        }},
                {"CalendarEventSetDate", [](lua_State* L) -> int {
            auto& d = calendarDraft();
            d.eventTime.month = static_cast<int>(luaL_optnumber(L, 1, d.eventTime.month));
            d.eventTime.day   = static_cast<int>(luaL_optnumber(L, 2, d.eventTime.day));
            const int year    = static_cast<int>(luaL_optnumber(L, 3, d.eventTime.fullYear()));
            d.eventTime.yearSince2000 = year - 2000;
            // The weekday is part of the packed field, so it has to follow the
            // date rather than be left at whatever the last one was.
            d.eventTime.weekday = wowee::game::weekdayOf(
                d.eventTime.month, d.eventTime.day, d.eventTime.fullYear()) - 1;
            return 0;
        }},
                {"CalendarEventSetTime", [](lua_State* L) -> int {
            auto& d = calendarDraft();
            d.eventTime.hour   = static_cast<int>(luaL_optnumber(L, 1, d.eventTime.hour));
            d.eventTime.minute = static_cast<int>(luaL_optnumber(L, 2, d.eventTime.minute));
            return 0;
        }},
                // The same boundary as the type. The interface hands over a
                // dropdown button id, which counts from one, and the wire's
                // CalendarRepeatType counts from zero - so "Never" would have
                // been sent as "Weekly".
                {"CalendarEventSetRepeatOption", [](lua_State* L) -> int {
            const int uiOption = static_cast<int>(luaL_optnumber(L, 1, 1));
            calendarDraft().repeatOption =
                static_cast<uint8_t>(uiOption > 0 ? uiOption - 1 : 0);
            return 0;
        }},
                {"CalendarEventSetTextureID", [](lua_State* L) -> int {
            calendarDraft().dungeonId =
                static_cast<int32_t>(luaL_optnumber(L, 1, -1));
            return 0;
        }},
                // Editing the open event, and deleting it.
                //
                // Both act on whichever event is open rather than on the
                // draft, which is what the interface means: the edit form is
                // filled from the open event and CalendarUpdateEvent commits
                // it back. With nothing open there is nothing to change, so
                // both do nothing rather than inventing an id.
                {"CalendarUpdateEvent", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& open = gh->getCalendarEventDetail();
            if (open.eventId == 0) return 0;
            gh->updateCalendarEvent(open.eventId, 0, calendarDraft());
            return 0;
        }},
                {"CalendarRemoveEvent", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& open = gh->getCalendarEventDetail();
            if (open.eventId == 0) return 0;
            gh->removeCalendarEvent(open.eventId, 0);
            return 0;
        }},
                // Whether the edit form has anything to commit. The interface
                // greys its save button on this, so answering true always
                // would offer to save an event nobody had touched.
                {"CalendarEventHaveSettingsChanged", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto& open = gh->getCalendarEventDetail();
            const auto& d = calendarDraft();
            const bool changed =
                open.eventId != 0 &&
                (d.title != open.title || d.description != open.description ||
                 d.type != open.type ||
                 d.eventTime.day != open.eventTime.day ||
                 d.eventTime.month != open.eventTime.month ||
                 d.eventTime.hour != open.eventTime.hour ||
                 d.eventTime.minute != open.eventTime.minute);
            lua_pushboolean(L, changed ? 1 : 0);
            return 1;
        }},
                // Setting another invitee's status or moderator rank, which
                // is what an event's owner does to confirm, bench or promote
                // someone. The player's own answer is a different packet
                // (CMSG_CALENDAR_EVENT_RSVP) and goes through the RSVP verbs.
                //
                // The index is into the open event's invite list, and the
                // status arrives one-based from the interface where the wire
                // counts from zero.
                {"CalendarEventSetStatus", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int uiStatus = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (!gh) return 0;
            const auto& ev = gh->getCalendarEventDetail();
            if (index < 1 || static_cast<size_t>(index) > ev.invitees.size()) return 0;
            const auto& inv = ev.invitees[static_cast<size_t>(index) - 1];
            gh->setCalendarInviteStatus(inv.guid, ev.eventId, inv.inviteId,
                                        static_cast<uint8_t>(uiStatus > 0 ? uiStatus - 1 : 0));
            return 0;
        }},
                {"CalendarEventSetModerator", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return 0;
            const auto& ev = gh->getCalendarEventDetail();
            if (index < 1 || static_cast<size_t>(index) > ev.invitees.size()) return 0;
            const auto& inv = ev.invitees[static_cast<size_t>(index) - 1];
            gh->setCalendarInviteModerator(inv.guid, ev.eventId, inv.inviteId, 1);
            return 0;
        }},
                {"CalendarEventClearModerator", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return 0;
            const auto& ev = gh->getCalendarEventDetail();
            if (index < 1 || static_cast<size_t>(index) > ev.invitees.size()) return 0;
            const auto& inv = ev.invitees[static_cast<size_t>(index) - 1];
            gh->setCalendarInviteModerator(inv.guid, ev.eventId, inv.inviteId, 0);
            return 0;
        }},
                // Inviting someone.
                //
                // Two cases in one call, which is how WoW does it. An event
                // that exists is invited to by id; an event still being
                // created has no id yet, so the invitation is a *pre-invite*
                // and the server holds it against the creator until the event
                // is committed. Telling them apart by whether an event is
                // open, which is the same thing the interface means.
                {"CalendarEventInvite", [](lua_State* L) -> int {
            const char* name = luaL_optstring(L, 1, "");
            auto* gh = getGameHandler(L);
            if (!gh || !name || !*name) return 0;
            const auto& open = gh->getCalendarEventDetail();
            const bool preInvite = (open.eventId == 0);
            const uint32_t flags = preInvite ? calendarDraft().flags : open.flags;
            gh->inviteToCalendarEvent(open.eventId, 0, name, preInvite,
                                      (flags & 0x0400u) != 0);
            return 0;
        }},
                // The three bounds go on the wire: this has its own opcode
                // rather than being an invitation with the name left out, and
                // the server picks the members from them.
                {"CalendarMassInviteGuild", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            gh->massInviteGuildToCalendarEvent(
                static_cast<uint32_t>(luaL_optnumber(L, 1, 1)),
                static_cast<uint32_t>(luaL_optnumber(L, 2, 80)),
                static_cast<uint32_t>(luaL_optnumber(L, 3, 0)));
            return 0;
        }},
                // And the commit.
                {"CalendarAddEvent", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                gh->createCalendarEvent(calendarDraft());
            }
            return 0;
        }},
                // ---- The right-click menu on a day's event ----
                //
                // CalendarContextSelectEvent names the row the menu is about
                // and every verb below acts on that one. Kept as state because
                // the menu asks for it back - the dropdown is built in one
                // call and clicked in another, with nothing carried between.
                {"CalendarContextSelectEvent", [](lua_State* L) -> int {
            calendarContextRow() = {
                .monthOffset = static_cast<int>(luaL_optnumber(L, 1, 0)),
                .day = static_cast<int>(luaL_optnumber(L, 2, 0)),
                .index = static_cast<int>(luaL_optnumber(L, 3, 0))};
            return 0;
        }},
                {"CalendarContextGetEventIndex", [](lua_State* L) -> int {
            const auto& row = calendarContextRow();
            if (row.day < 1 || row.index < 1) {
                // Three nils rather than none: the interface unpacks this as a
                // group, and a short answer is the shape framexml_short_returns
                // exists to catch.
                lua_pushnil(L); lua_pushnil(L); lua_pushnil(L);
                return 3;
            }
            lua_pushnumber(L, row.monthOffset);
            lua_pushnumber(L, row.day);
            lua_pushnumber(L, row.index);
            return 3;
        }},
                // Answering an invitation, which is the one calendar action
                // that needs nothing staged first. The status values are the
                // server's CalendarInviteStatus (CalendarMgr.h:73).
                {"CalendarContextInviteAvailable", [](lua_State* L) -> int {
            return calendarRespondToContextInvite(L, 1);   // accepted
        }},
                {"CalendarContextInviteDecline", [](lua_State* L) -> int {
            return calendarRespondToContextInvite(L, 2);   // declined
        }},
                {"CalendarContextInviteTentative", [](lua_State* L) -> int {
            return calendarRespondToContextInvite(L, 8);   // tentative
        }},
                {"CalendarContextInviteRemove", [](lua_State* L) -> int {
            return calendarRespondToContextInvite(L, 9);   // removed
        }},
                // Whether the player may edit or delete the row the menu is
                // about, which is whether they created it. The server decides
                // for real and refuses otherwise; this is what greys the menu
                // entry rather than letting it be clicked and rejected.
                {"CalendarContextEventCanEdit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto rows = calendarDayRows(L);
            const int index = static_cast<int>(luaL_optnumber(L, 3, 0));
            if (index < 1 || static_cast<size_t>(index) > rows.size()) {
                lua_pushboolean(L, 0);
                return 1;
            }
            const auto& row = rows[static_cast<size_t>(index) - 1];
            if (row.kind != wowee::game::CalendarEntryKind::Event) {
                lua_pushboolean(L, 0);
                return 1;
            }
            const auto& ev = gh->getCalendarData().events[row.index];
            lua_pushboolean(L, ev.creatorGuid != 0 &&
                                   ev.creatorGuid == gh->getPlayerGuid() ? 1 : 0);
            return 1;
        }},
                // Reporting an event is a GM feature this client has no
                // packet for. False rather than nothing: the menu tests it to
                // decide whether to draw the entry, and a stand-in would draw
                // one that does nothing when clicked.
                {"CalendarContextEventCanComplain", [](lua_State* L) -> int {
            lua_pushboolean(L, 0);
            return 1;
        }},
                // Delete the row the menu is about.
                {"CalendarContextEventRemove", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& row = calendarContextRow();
            const auto rows = calendarRowsFor(*gh, row.monthOffset, row.day);
            if (row.index < 1 || static_cast<size_t>(row.index) > rows.size()) return 0;
            const auto& entry = rows[static_cast<size_t>(row.index) - 1];
            if (entry.kind != wowee::game::CalendarEntryKind::Event) return 0;
            gh->removeCalendarEvent(
                gh->getCalendarData().events[entry.index].eventId, 0);
            return 0;
        }},
                // Signing up to a guild event, which is an RSVP with the
                // signed-up status rather than a separate request.
                {"CalendarContextEventSignUp", [](lua_State* L) -> int {
            return calendarRespondToContextInvite(L, 6);   // SIGNED_UP
        }},
                // Which of the six kinds the menu's row is, so the menu can
                // offer the right verbs. Read from the same day list the row
                // was chosen out of.
                {"CalendarContextEventGetCalendarType", [](lua_State* L) -> int {
            const auto& row = calendarContextRow();
            auto* gh = getGameHandler(L);
            // Nil rather than nothing, for the same reason as
            // CalendarContextGetEventIndex above: the arity belongs at the
            // binding rather than resting on Lua filling the gap.
            if (!gh || row.day < 1) { lua_pushnil(L); return 1; }
            const auto rows = calendarRowsFor(*gh, row.monthOffset, row.day);
            if (row.index < 1 || static_cast<size_t>(row.index) > rows.size()) {
                lua_pushnil(L);
                return 1;
            }
            const auto& entry = rows[static_cast<size_t>(row.index) - 1];
            if (entry.kind == wowee::game::CalendarEntryKind::Holiday) {
                lua_pushstring(L, "HOLIDAY");
            } else {
                lua_pushstring(L, calendarTypeName(
                    gh->getCalendarData().events[entry.index]));
            }
            return 1;
        }},
                // Which event is selected, as monthOffset, day, index.
                //
                // Nothing is selected until an event view exists to select
                // into, so all three are nil - but three nils rather than no
                // values at all. The interface unpacks it as a group, and a
                // binding that answers short is the shape framexml_short_returns
                // is pinned at zero to catch: it reads as correct here only
                // because Lua happens to fill the rest with nil.
                {"CalendarGetEventIndex", [](lua_State* L) -> int {
            lua_pushnil(L);
            lua_pushnil(L);
            lua_pushnil(L);
            return 3;
        }},
                // The first unanswered invite on a day, or zero.
                //
                // A number always, never nothing: the interface compares it
                // straight away - `if ( pendingInviteIndex > 0 )` with no
                // guard in front - so nil raises rather than reading as "no
                // invite". The invite list the server sends is not per-day, so
                // a day has one only when one of its events is in it.
                {"CalendarGetFirstPendingInvite", [](lua_State* L) -> int {
            const auto rows = calendarDayRows(L);
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto& cal = gh->getCalendarData();
            for (size_t i = 0; i < rows.size(); ++i) {
                if (rows[i].kind != wowee::game::CalendarEntryKind::Event) continue;
                const uint64_t id = cal.events[rows[i].index].eventId;
                for (const auto& invite : cal.invites) {
                    // 0 is CALENDAR_INVITESTATUS_INVITED - still unanswered.
                    if (invite.eventId == id && invite.status == 0) {
                        lua_pushnumber(L, static_cast<double>(i + 1));
                        return 1;
                    }
                }
            }
            lua_pushnumber(L, 0);
            return 1;
        }},
                // ---- The month the grid is drawn from ----
                //
                // The viewed month is the interface's own state in WoW too:
                // CalendarSetMonth and CalendarSetAbsMonth move it and
                // CalendarGetMonth reads it back, so it is kept here rather
                // than recomputed from the clock on every call - which would
                // make the previous-month button do nothing.
                {"CalendarGetMonth", [](lua_State* L) -> int {
            // The offset is optional: CalendarGetMonth() means the viewed
            // month, and the interface calls it that way and with -1 and 1 in
            // the same breath to fill the leading and trailing cells.
            const int offset = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto viewed = calendarViewedMonth();
            const auto info = wowee::game::calendarMonthAt(
                viewed.first, viewed.second, offset);
            lua_pushnumber(L, info.month);
            lua_pushnumber(L, info.year);
            lua_pushnumber(L, info.numDays);
            lua_pushnumber(L, info.firstWeekday);
            return 4;
        }},
                {"CalendarSetMonth", [](lua_State* L) -> int {
            const int offset = static_cast<int>(luaL_checknumber(L, 1));
            const auto viewed = calendarViewedMonth();
            const auto info = wowee::game::calendarMonthAt(
                viewed.first, viewed.second, offset);
            calendarSetViewedMonth(info.month, info.year);
            return 0;
        }},
                {"CalendarSetAbsMonth", [](lua_State* L) -> int {
            calendarSetViewedMonth(static_cast<int>(luaL_checknumber(L, 1)),
                                   static_cast<int>(luaL_checknumber(L, 2)));
            return 0;
        }},
                // The ends of the range, both as weekday, month, day, year.
                //
                // Nothing on the wire says how far the server will let the
                // player look or book, so this client offers the year either
                // side of today. A year ahead is the limit the interface
                // already has a string for - CALENDAR_ERROR_CREATEDATE_AFTER_MAX
                // - and the same span back keeps past events reachable, which
                // is what the previous-month button is for.
                {"CalendarGetMinDate", [](lua_State* L) -> int {
            return pushCalendarBoundDate(L, -12);
        }},
                {"CalendarGetMaxCreateDate", [](lua_State* L) -> int {
            return pushCalendarBoundDate(L, 12);
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
                // Counted from zero, like GetExpansionLevel beside it and like
                // the table it is used to index. This answered one higher, and
                // reputationframe.lua does
                //     MAX_PLAYER_LEVEL = MAX_PLAYER_LEVEL_TABLE[GetAccountExpansionLevel()]
                // against a table holding only 0, 1 and 2 - so on Wrath it read
                // nothing and MAX_PLAYER_LEVEL became nil, which
                // `newLevel < MAX_PLAYER_LEVEL` then raised on every level gained.
                // On the earlier two it simply came out an expansion too high,
                // and a level 60 was never treated as capped.
                {"GetAccountExpansionLevel", [](lua_State* L) -> int {
            lua_pushnumber(L, expansionLevelZeroBased(L));
            return 1;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
